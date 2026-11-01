/*
 * Apple Network Server 500/700 ("Shiner ESB"; the firmware calls itself
 * "Boot TNT").
 *
 * The board is PowerSurge-class: two Bandit PCI host bridges (Apple 343S1126)
 * at 0xF2000000 and 0xF4000000, a Hammerhead memory controller (343S1142) at
 * 0xF8000000, and a Grand Central I/O ASIC (343S1125, PCI 106B:0002) whose
 * 16 MB BAR carries the mac-io-family children at fixed offsets:
 *
 *     0x10000 53c94 (MESH)     0x11000 mace        0x13000 escc
 *     0x14000 awacs            0x15000 swim3       0x16000 via-cuda
 *     0x1C000 lcd              0x1D000 nvram
 *
 * The ROM window is 4 MB at 0xFFC00000 and, as it turns out, holds two images
 * (see mac99-series/ANS-ROM-DUMPS.md): a 3 MB Mac-side 68K Mac ROM image at
 * the window base, and the PowerPC Open Firmware at window + 0x300000 -- so
 * the PPC reset vector at 0xFFF00100 lands in the OF entry stub table.
 *
 * This file is stage5/0001: CPU, RAM, the ROM window and stubbed register
 * windows, enough to start the real firmware and find out what it touches
 * first.  Bandit, Hammerhead and Grand Central become real devices from that
 * evidence.
 *
 * Copyright (c) 2026 the QEMU PPC mac99/PReP work
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/datadir.h"
#include "qemu/units.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "hw/core/boards.h"
#include "hw/core/loader.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/ans_gc.h"
#include "hw/misc/hammerhead.h"
#include "hw/pci-host/bandit.h"
#include "hw/misc/unimp.h"
#include "hw/ppc/ppc.h"
#include "system/system.h"
#include "system/block-backend.h"
#include "system/reset.h"
#include "qemu/timer.h"

#define ANS_ROM_BASE        0xffc00000ULL
#define ANS_ROM_SIZE        (4 * MiB)
#define ANS_OF_OFFSET       0x00300000ULL
#define ANS_ROM_FILE        "ans.rom"

/* The ROM's own device tree: timebase 12.5 MHz, 200 MHz CPU clock. */
#define ANS_TB_FREQ         12500000u

/*
 * Diagnostic: rewrite the install miniroot's boot-type probe while the
 * ramfs is populated.  ANS_PATCH_BOOTTYPE=1 selects a 64 MB layout.
 */
#define ANS_BOOTPATCH_RAMFS 0x3770000u
#define ANS_BOOTPATCH_LINE  0x39492c4u
#define ANS_BOOTPATCH_MOUNT 0x3948f3eu
static QEMUTimer *ans_bootpatch_timer;
static uint8_t *ans_bootpatch_ram;
static int64_t ans_bootpatch_end;
static bool ans_bootinfo_patched;
static bool ans_iplcb_patched;

static void ans_bootpatch_cb(void *opaque)
{
    static const char pat[] = "BOOTYPE=`bootinfo -t`";
    uint8_t *p = ans_bootpatch_ram + ANS_BOOTPATCH_LINE;
    uint8_t *m = ans_bootpatch_ram + ANS_BOOTPATCH_MOUNT;

    (void)opaque;
    if (getenv("ANS_PATCH_IPLCB") && !ans_iplcb_patched) {
        uint8_t *cb = ans_bootpatch_ram + 0x3ee0000;

        if (!memcmp(cb + 0x80, "ROSIPL", 6)) {
            uint8_t *inq = cb + 0x11dc;

            inq[0] = 5;      /* peripheral type: CD-ROM */
            inq[1] = 0x80;   /* removable */
            inq[2] = 1;      /* SCSI-1 */
            inq[3] = 1;      /* CCS */
            inq[4] = 0x31;
            memcpy(inq + 8, "    SONY", 8);
            memcpy(inq + 16, "CD-ROM CDU-8008 ", 16);
            memcpy(inq + 32, "1.8g", 4);
            ans_iplcb_patched = true;
            fprintf(stderr, "ans: IPL-CB boot-device record set to CD\n");
        }
    }
    if (getenv("ANS_PATCH_BOOTINFO") && !ans_bootinfo_patched) {
        static const char line[] =
            "# Start the boot process for a particular device.";
        static const char repl[] =
            "bootinfo -t >/y;odmget CuDv >/z;echo M>>/y       ";
        uint8_t *hit = memmem(ans_bootpatch_ram + ANS_BOOTPATCH_RAMFS,
                              8 * MiB, line, sizeof(line) - 1);

        if (hit) {
            memcpy(hit, repl, sizeof(repl) - 1);
            ans_bootinfo_patched = true;
            fprintf(stderr, "ans: bootinfo probe line patched\n");
        }
    }
    if (getenv("ANS_PATCH_BOOTTYPE") && !memcmp(p, pat, sizeof(pat) - 1)) {
        memcpy(p + 8, "3            ", 13);
        fprintf(stderr, "ans: BOOTYPE forced to 3\n");
    }
    if (getenv("ANS_PATCH_BOOTTYPE") &&
        !memcmp(m, "mount -v cdrfs -o ro /dev/`bootinfo -b` /SPOT", 45)) {
        memcpy(m, "mount -v cdrfs -o ro /dev/cd0 /SPOT 2>/x     ", 45);
        fprintf(stderr, "ans: /SPOT mount rewritten\n");
    }
    if (qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) < ans_bootpatch_end) {
        timer_mod(ans_bootpatch_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  (getenv("ANS_PATCH_BOOTINFO") ? 1000000 : 100000));
    }
}

/*
 * Grand Central is pre-decoded at $F3000000 on this board (dev notes 4.6.1)
 * and this ROM never enumerates it, so its BAR is assigned here -- and again
 * on every reset, because the PCI core's device reset restores the BAR to its
 * power-on value and would otherwise unmap it.
 */
static void ans_gc_bar_reset(void *opaque)
{
    PCIDevice *d = opaque;

    pci_default_write_config(d, PCI_BASE_ADDRESS_0,
                             0xf3000000 | PCI_BASE_ADDRESS_MEM_TYPE_32, 4);
    pci_default_write_config(d, PCI_COMMAND,
                             PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER, 2);
}

static void ans_cpu_reset(void *opaque)
{
    PowerPCCPU *cpu = opaque;

    cpu_ppc_tb_reset(&cpu->env);
    cpu_reset(CPU(cpu));
}

static void ans_init(MachineState *machine)
{
    const char *bios_name = machine->firmware ?: ANS_ROM_FILE;
    MemoryRegion *sysmem = get_system_memory();
    MemoryRegion *rom = g_new(MemoryRegion, 1);
    uint64_t rom_load_addr = ANS_ROM_BASE;
    char *filename;
    int64_t rom_size;
    BanditState *bandit = NULL;
    DeviceState *bandits[2] = { NULL, NULL };
    DeviceState *dev;
    PowerPCCPU *cpu0 = NULL;
    int i;

    /* init CPUs */
    for (i = 0; i < machine->smp.cpus; i++) {
        PowerPCCPU *cpu = POWERPC_CPU(cpu_create(machine->cpu_type));

        cpu_ppc_tb_init(&cpu->env, ANS_TB_FREQ);
        qemu_register_reset(ans_cpu_reset, cpu);
        if (!cpu0) {
            cpu0 = cpu;
        }
    }

    memory_region_add_subregion(sysmem, 0, machine->ram);
    if ((getenv("ANS_PATCH_BOOTTYPE") || getenv("ANS_PATCH_BOOTINFO") ||
         getenv("ANS_PATCH_IPLCB")) &&
        machine->ram_size == 64 * MiB) {
        ans_bootpatch_ram = memory_region_get_ram_ptr(machine->ram);
        ans_bootpatch_end = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                            (getenv("ANS_PATCH_BOOTINFO") ? 60 : 20) *
                            NANOSECONDS_PER_SECOND;
        ans_bootpatch_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                           ans_bootpatch_cb, NULL);
        timer_mod(ans_bootpatch_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 100000);
    }

    memory_region_init_rom(rom, NULL, "ans.rom", ANS_ROM_SIZE, &error_fatal);
    memory_region_add_subregion(sysmem, ANS_ROM_BASE, rom);

    filename = qemu_find_file(QEMU_FILE_TYPE_BIOS, bios_name);
    if (!filename) {
        error_report("Could not find firmware '%s'", bios_name);
        exit(1);
    }
    rom_size = get_image_size(filename, &error_fatal);
    if (rom_size < 0) {
        error_report("Could not load firmware '%s'", filename);
        exit(1);
    }
    /*
     * The firmware can be given either as the whole 4 MB window image or as
     * the 1 MB PowerPC Open Firmware image on its own; the latter is what
     * tools/ansrom.py --of produces.
     */
    if (rom_size == 1 * MiB) {
        rom_load_addr = ANS_ROM_BASE + ANS_OF_OFFSET;
    }
    if (load_image_targphys(filename, rom_load_addr,
                            ANS_ROM_SIZE - (rom_load_addr - ANS_ROM_BASE),
                            &error_fatal) < 0) {
        error_report("Could not load firmware '%s'", filename);
        exit(1);
    }
    g_free(filename);

    /* The two Bandit bridges (device notes 4.4): config registers, the 16 MB
     * identity window at the bridge, and the 256 MB window above it. */
    for (i = 0; i < 2; i++) {
        uint64_t base = i ? 0xf4000000 : 0xf2000000;
        uint64_t win = i ? 0x90000000 : 0x80000000;
        uint64_t small = i ? 0xf5000000 : 0xf3000000;

        dev = qdev_new(TYPE_BANDIT_PCI_HOST_BRIDGE);
        qdev_prop_set_uint32(dev, "small-win", small);
        qdev_prop_set_uint32(dev, "large-win", win);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, base + 0x00800000);
        sysbus_mmio_map(SYS_BUS_DEVICE(dev), 1, base + 0x00c00000);
        sysbus_mmio_map(SYS_BUS_DEVICE(dev), 2, small);
        sysbus_mmio_map(SYS_BUS_DEVICE(dev), 3, win);
        /* PCI I/O space: 8 MB at the bridge base (the bridge's ranges). */
        sysbus_mmio_map(SYS_BUS_DEVICE(dev), 4, base);
        bandits[i] = dev;
        if (i == 0) {
            bandit = BANDIT_PCI_HOST_BRIDGE(dev);
        }
        (void)bandit;
    }
    /* Grand Central: PCI 106B:0002 at devfn 0x10 on the first bus. */
    dev = DEVICE(pci_create_simple(PCI_HOST_BRIDGE(bandit)->bus,
                                   PCI_DEVFN(16, 0), TYPE_ANS_GC));
    /*
     * The dev notes say Grand Central "is configured by Open Firmware to
     * live at xF300000" -- it is pre-decoded there on this board, so give it
     * that BAR and enable it up front rather than waiting for an enumeration
     * pass that this ROM never performs.
     */
    ans_gc_bar_reset(PCI_DEVICE(dev));
    qemu_register_reset(ans_gc_bar_reset, PCI_DEVICE(dev));
    /* Grand Central's interrupt controller drives the CPU's INT line. */
    qdev_connect_gpio_out(dev, 0,
                          qdev_get_gpio_in(DEVICE(cpu0), PPC6xx_INPUT_INT));
    /*
     * ...and the PCI side arrives on its EXT lines (dev notes 4.2): the two
     * on-board 53C825As are FW0_Int/FW1_Int, the slots keep the TNT INTx
     * pattern, and the bridges' own errors are ganged on EXT1.
     */
    {
        static const int bandit_int[2][4] = {
            { ANS_GC_INT_FW0, ANS_GC_INT_FW1, ANS_GC_INT_SLOT1,
              ANS_GC_INT_SLOT2 },
            { ANS_GC_INT_SLOT3, ANS_GC_INT_SLOT4, ANS_GC_INT_ERROR,
              ANS_GC_INT_ERROR },
        };
        int j;

        for (i = 0; i < 2; i++) {
            for (j = 0; j < 4; j++) {
                sysbus_connect_irq(SYS_BUS_DEVICE(bandits[i]), j,
                                   qdev_get_gpio_in(dev, bandit_int[i][j]));
            }
        }
    }
    /* ...and expose it where the board decodes it. */
    memory_region_add_subregion(get_system_memory(), 0xf3000000,
                                &ANS_GC(dev)->bar);
    /* The two on-board 53C825As at IDSEL 17 and 18 on the first Bandit; Open
     * Firmware assigns their BARs inside that bridge's 16 MB window. */
    for (i = 0; i < 2; i++) {
        dev = DEVICE(pci_new(PCI_DEVFN(17 + i, 0), "lsi53c825"));
        /* the ROM's scripts expect COMMAND after a non-ATN Select and complete
         * on UNEXPECTED DISCONNECT; see the properties in lsi53c895a.c */
        qdev_prop_set_bit(dev, "select-no-atn-cmd", true);
        qdev_prop_set_bit(dev, "disconnect-udc", true);
        /* An absent target arbitrates for STIME0's time-out before the part
         * reports STO + the stacked UDC (TM v3.1, STIME0).  Failing the
         * Select inside the instruction makes AIX's pscsidd restart its ring
         * on the same target forever instead of walking on. */
        qdev_prop_set_bit(dev, "select-timeout", true);
        /* A store starts the engine; the command completes later (see
         * lsi53c895a.c's defer-start). */
        qdev_prop_set_bit(dev, "defer-start", true);
        /* AIX's driver sends an IDENTIFY message after the select, so its
         * SATN/ flag is the one in DCMD bit 24 (see lsi53c895a.c). */
        qdev_prop_set_bit(dev, "select-atn-bit24", true);
        /* A SCSI RST/ is a bus reset, not a device reset: do not re-arm
         * unit attention on the targets (see lsi53c895a.c). */
        qdev_prop_set_bit(dev, "reset-devices", false);
        pci_realize_and_unref(PCI_DEVICE(dev), PCI_HOST_BRIDGE(bandit)->bus,
                              &error_fatal);
    }
    /*
     * The built-in Cirrus 54M30 video at IDSEL 15 (no interrupt line):
     * POST's "MainLBU Video" test reads $1013:$00A0 out of config space and
     * then checks that BAR0 takes a write.
     */
    dev = DEVICE(pci_create_simple(PCI_HOST_BRIDGE(bandit)->bus,
                                   PCI_DEVFN(15, 0), "cirrus-54m30"));
    (void)dev;
    /* The bridge's own header is at IDSEL 11 (106B:0001); not modelled yet. */
    dev = qdev_new(TYPE_HAMMERHEAD);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, 0xf8000000);
}

static void ans_machine_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "Apple Network Server 500/700";
    mc->init = ans_init;
    mc->default_cpu_type = POWERPC_CPU_TYPE_NAME("604e_v2.2");
    mc->max_cpus = 1;
    mc->default_ram_size = 128 * MiB;
    mc->default_ram_id = "ans.ram";
}

static const TypeInfo ans_machine_type = {
    .name = MACHINE_TYPE_NAME("ans"),
    .parent = TYPE_MACHINE,
    .class_init = ans_machine_class_init,
};

static void ans_machine_register_types(void)
{
    type_register_static(&ans_machine_type);
}

type_init(ans_machine_register_types);
