/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * AlphaPC 164 (EB164) motherboard emulation.
 *
 * References:
 *   AlphaPC 164 Motherboard Technical Reference Manual, EC-QPFYB-TE
 *   21172 Core Logic Chipset Technical Reference Manual, EC-QUQJA-TE
 */

#include "qemu/osdep.h"
#include "qemu/cutils.h"
#include "qemu/datadir.h"
#include "qemu/error-report.h"
#include "qemu/log.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "net/net.h"
#include "hw/alpha/alcor.h"
#include "hw/block/flash.h"
#include "hw/block/fdc.h"
#include "hw/char/parallel-isa.h"
#include "hw/core/boards.h"
#include "hw/core/clock.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/sysbus.h"
#include "hw/core/loader.h"
#include "hw/ide/pci.h"
#include "../ide/ide-internal.h"
#include "hw/input/ps2.h"
#include "hw/isa/isa.h"
#include "hw/isa/superio.h"
#include "hw/pci/pci.h"
#include "hw/rtc/mc146818rtc.h"
#include "system/address-spaces.h"
#include "system/block-backend-global-state.h"
#include "system/reset.h"
#include "system/system.h"
#include "exec/target_page.h"
#include "cpu.h"
#include "internals.h"

#define TYPE_EB164_MACHINE MACHINE_TYPE_NAME("eb164")
OBJECT_DECLARE_SIMPLE_TYPE(EB164MachineState, EB164_MACHINE)

#define EB164_MAX_RAM_SIZE (1 * GiB)

#define SRM_LOAD_ADDRESS        0x0
#define PALCODE_ROM_SIZE        (2 * MiB)

#define FLASH_ROM_FILENAME      "eb164.rom"

/*
 * The board flash is an Intel 28F008SA, 1MB in two 512KB banks
 * selected by FLASH_ADR19 (ISA port 0x800 bit 0).  Model it as a CFI
 * flash so the console drivers' command/status protocol
 * (0x50/0x70/0xFF/0x90) works.
 */
#define EB164_FLASH_BLOCKS      16
#define EB164_FLASH_SECTOR      (64 * KiB)
#define EB164_FLASH_BANK_SIZE   (512 * KiB)
/*
 * The last flash sector holds the AlphaBIOS NVRAM/environment.  The
 * board also makes it visible in the legacy BIOS area at PCI memory
 * address 000F.0000, which is how AlphaBIOS reads and programs it
 * (dense-space byte accesses).
 */
#define EB164_FLASH_NVRAM_OFF   (15 * EB164_FLASH_SECTOR)
#define EB164_FLASH_NVRAM_SIZE  EB164_FLASH_SECTOR

/*
 * State passed to the CPU reset handler: the SROM handoff must be
 * applied after the CPU's own reset clears the registers.
 */
typedef struct EB164ResetInfo {
    AlphaCPU *cpu;
    uint64_t entry;
    uint64_t ram_size;
} EB164ResetInfo;

struct EB164MachineState {
    /*< private >*/
    MachineState parent_obj;

    /*< public >*/
    MemoryRegion flash_bank_low;
    MemoryRegion flash_bank_high;
    MemoryRegion flash_nvram;
    MemoryRegion flash_byte;
    MemoryRegion flash_int8;
    MemoryRegion flash_isa;
    MemoryRegion vga_legacy;
    AddressSpace vga_pci_as;
    MemoryRegion vga_hose;
    bool vga_hole;
};

/*
 * ISA port 0x800 bit 0 = FLASH_ADR19: selects the upper 512KB bank
 * of the 1MB flash in the FFF8.0000-FFFF.FFFF window (TRM 4.3.5).
 */
#define TYPE_EB164_FLASH_ADDR "eb164-flash-addr"
OBJECT_DECLARE_SIMPLE_TYPE(EB164FlashAddrState, EB164_FLASH_ADDR)

struct EB164FlashAddrState {
    ISADevice parent_obj;
    EB164MachineState *machine;
    bool upper;
    MemoryRegion io;
};

static void eb164_flash_addr_write(void *opaque, hwaddr addr,
                                   uint64_t val, unsigned size)
{
    EB164FlashAddrState *s = opaque;
    EB164MachineState *ms = s->machine;
    bool upper;

    if (addr != 0) {
        /* 0x801 is the read-only board configuration register.  */
        return;
    }
    upper = (val & 1) != 0;
    s->upper = upper;
    memory_region_set_enabled(&ms->flash_bank_low, !upper);
    memory_region_set_enabled(&ms->flash_bank_high, upper);
}

static uint64_t eb164_flash_addr_read(void *opaque, hwaddr addr,
                                      unsigned size)
{
    EB164FlashAddrState *s = opaque;

    switch (addr) {
    case 0:
        /* FLASH_ADR19: current 512 KiB flash bank select.  */
        return s->upper;
    case 1:
        /*
         * 0x801: AlphaPC 164 configuration jumpers (CF0-CF7).
         */
        return 0xff;
    default:
        return 0xff;
    }
}

static const MemoryRegionOps eb164_flash_addr_ops = {
    .read = eb164_flash_addr_read,
    .write = eb164_flash_addr_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 1,
    },
};

static void eb164_flash_addr_realize(DeviceState *dev, Error **errp)
{
    EB164FlashAddrState *s = EB164_FLASH_ADDR(dev);

    memory_region_init_io(&s->io, OBJECT(dev), &eb164_flash_addr_ops, s,
                          "eb164.flash-addr", 2);
    isa_register_ioport(ISA_DEVICE(dev), &s->io, 0x800);
}

static const Property eb164_flash_addr_properties[] = {
    DEFINE_PROP_LINK("machine", EB164FlashAddrState, machine,
                     TYPE_EB164_MACHINE, EB164MachineState *),
};

static void eb164_flash_addr_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = eb164_flash_addr_realize;
    device_class_set_props(dc, eb164_flash_addr_properties);
}

static const TypeInfo eb164_flash_addr_info = {
    .name          = TYPE_EB164_FLASH_ADDR,
    .parent        = TYPE_ISA_DEVICE,
    .instance_size = sizeof(EB164FlashAddrState),
    .class_init    = eb164_flash_addr_class_init,
};

static void eb164_flash_addr_register_types(void)
{
    type_register_static(&eb164_flash_addr_info);
}

type_init(eb164_flash_addr_register_types)

static void eb164_create_flash(EB164MachineState *ms, PCIBus *pci_bus,
                               MemoryRegion *sysmem, const char *filename)
{
    MemoryRegion *flash_mem;
    MemoryRegion *pci_mem;
    PFlashCFI01 *flash;
    DeviceState *dev;
    uint8_t *storage;
    int size, loaded;
    DriveInfo *dinfo = drive_get(IF_PFLASH, 0, 0);

    size = get_image_size(filename, NULL);
    if (size < 0 || size > EB164_FLASH_BLOCKS * EB164_FLASH_SECTOR) {
        error_report("flash image '%s' does not fit the 1MB flash", filename);
        exit(EXIT_FAILURE);
    }

    dev = qdev_new(TYPE_PFLASH_CFI01);
    qdev_prop_set_string(dev, "name", "eb164.flash");
    qdev_prop_set_uint32(dev, "num-blocks", EB164_FLASH_BLOCKS);
    qdev_prop_set_uint64(dev, "sector-length", EB164_FLASH_SECTOR);
    qdev_prop_set_uint8(dev, "width", 1);
    qdev_prop_set_uint8(dev, "device-width", 1);
    qdev_prop_set_uint8(dev, "max-device-width", 1);
    qdev_prop_set_uint16(dev, "id0", 0x89);    /* Intel */
    qdev_prop_set_uint16(dev, "id1", 0xa2);    /* 28F008SA (1MB) */
    qdev_prop_set_uint16(dev, "id2", 0x00);
    qdev_prop_set_uint16(dev, "id3", 0x00);
    if (dinfo) {
        qdev_prop_set_drive(dev, "drive", blk_by_legacy_dinfo(dinfo));
    }
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    flash = PFLASH_CFI01(dev);
    flash_mem = pflash_cfi01_get_memory(flash);

    /*
     * Without a backing block device, fill the flash array directly from
     * the image (an erased bank would read 0xFF).  With one (i.e. the
     * user supplied -drive if=pflash), the drive supplies the contents,
     * so the console's environment and ISA table survive across runs.
     */
    if (!dinfo) {
        storage = memory_region_get_ram_ptr(flash_mem);
        memset(storage, 0xff, EB164_FLASH_BLOCKS * EB164_FLASH_SECTOR);
        loaded = load_image_size(filename, storage + EB164_FLASH_SECTOR,
                                 size);
        if (loaded != size) {
            error_report("could not load flash image '%s'", filename);
            exit(EXIT_FAILURE);
        }
    }

    /*
     * The CIA presents the ISA FROM_BASE window (PCI memory
     * FFF8.0000-FFFF.FFFF) to the CPU through both the sparse memory
     * windows (HAE_MEM) and the INT8 dense alias, i.e. every firmware
     * flash access lands in the pchip's PCI memory space at
     * 0xFFF80000.  Map the two 512KB banks there, selected by
     * FLASH_ADR19.
     */
    pci_mem = pci_bus->address_space_mem;
    memory_region_init_alias(&ms->flash_bank_low, NULL,
                             "eb164.flash.bank0", flash_mem, 0,
                             EB164_FLASH_BANK_SIZE);
    memory_region_init_alias(&ms->flash_bank_high, NULL,
                             "eb164.flash.bank1", flash_mem,
                             EB164_FLASH_BANK_SIZE, EB164_FLASH_BANK_SIZE);
    memory_region_add_subregion_overlap(pci_mem, 0xFFF80000,
                                        &ms->flash_bank_low, 1);
    memory_region_add_subregion_overlap(pci_mem, 0xFFF80000,
                                        &ms->flash_bank_high, 1);
    memory_region_set_enabled(&ms->flash_bank_high, false);

    /*
     * Legacy BIOS area alias of the NVRAM sector at PCI 000F.0000.
     * AlphaBIOS reaches its NVRAM through dense-space byte accesses
     * to this address.
     */
    memory_region_init_alias(&ms->flash_nvram, NULL, "eb164.flash.nvram",
                             flash_mem, EB164_FLASH_NVRAM_OFF,
                             EB164_FLASH_NVRAM_SIZE);
    memory_region_add_subregion_overlap(pci_mem, EB164_FLASH_NVRAM_OFF,
                                        &ms->flash_nvram, 1);

    /* CPU-space windows: byte window, INT8 alias, top-of-space ISA alias. */
    memory_region_init_alias(&ms->flash_byte, NULL, "eb164.flash.byte",
                             flash_mem, 0,
                             EB164_FLASH_BLOCKS * EB164_FLASH_SECTOR);
    memory_region_add_subregion_overlap(sysmem, ALCOR_FLASH,
                                        &ms->flash_byte, 1);
    memory_region_init_alias(&ms->flash_int8, NULL, "eb164.flash.int8",
                             flash_mem, 0,
                             EB164_FLASH_BLOCKS * EB164_FLASH_SECTOR);
    memory_region_add_subregion_overlap(sysmem, ALCOR_FLASH_INT8,
                                        &ms->flash_int8, 1);
    memory_region_init_alias(&ms->flash_isa, NULL, "eb164.flash.isa",
                             flash_mem, 0,
                             EB164_FLASH_BLOCKS * EB164_FLASH_SECTOR);
    memory_region_add_subregion(sysmem, 0xfffff00000ULL, &ms->flash_isa);
}

/*
 * AlphaPC 164 firmware images are "makerom" images: a header with a
 * validation pattern, image size, destination address and firmware id,
 * followed by the console image.  The bootstrap ROM scans flash for
 * this header, copies the image to its destination and enters it in
 * PAL mode.
 */
static ssize_t eb164_load_srom(EB164ResetInfo *ri, const char *filename,
                               Error **errp)
{
    g_autoptr(GError) gerr = NULL;
    g_autofree char *file_data = NULL;
    const uint32_t *p, *end;
    gsize file_len;
    uint32_t header_size, image_size;
    uint64_t dest;
    bool found = false;

    if (!g_file_get_contents(filename, &file_data, &file_len, &gerr)) {
        error_setg(errp, "%s", gerr->message);
        return -1;
    }

    /*
     * Scan for the makerom validation pattern (0x5A5AC3C3) and its
     * inverse (0xA5A53C3C) at longword boundaries, like the SROM's
     * LoadSystemCode routine.
     */
    p = (const uint32_t *)(const void *)file_data;
    end = (const uint32_t *)(const void *)(file_data + file_len);
    for (; p + 1 < end; p++) {
        if (ldl_le_p(p) == 0x5a5ac3c3 &&
            ldl_le_p(p + 1) == 0xa5a53c3c) {
            found = true;
            break;
        }
    }

    if (unlikely(!found)) {
        return 0;               /* not a makerom image */
    }

    header_size = ldl_le_p(p + 2);
    image_size = ldl_le_p(p + 4);
    dest = ldl_le_p(p + 6) | ((uint64_t)ldl_le_p(p + 7) << 32);

    if (header_size < 0x20 ||
        (const char *)p + header_size + image_size > file_data + file_len) {
        error_setg(errp, "makerom header in '%s' is malformed", filename);
        return -1;
    }

    rom_add_blob_fixed("eb164.srom", (const char *)p + header_size,
                       image_size, dest);
    ri->entry = dest;
    return image_size;
}

/*
 * The SROM leaves the TOY clock running with the 1024 Hz periodic interrupt
 * enabled.  QEMU's RTC resets with PIE clear, so restore the SROM's state
 * after each reset (see the call site for the register values).
 */
static void eb164_rtc_srom_reset(void *opaque)
{
    MC146818RtcState *rtc = opaque;

    mc146818rtc_set_cmos_data(rtc, 0x0a, 0x26);   /* divider + 1024 Hz rate */
    mc146818rtc_set_cmos_data(rtc, 0x0b, 0x46);   /* 24h, BCD, PIE on */
    mc146818rtc_set_cmos_data(rtc, 0x0c, 0x00);   /* no stale flags */
}

/*
 * The PCI0646 comes up in legacy mode, where its INTRQ drives the ISA
 * interrupt lines 14/15 of the 82378ZB's internal 8259 pair, not (only)
 * PCI INTA.  Firmware depends on that PC-compatible path: ARC 4.49's
 * ATAPI driver detects command completion by polling bit 6 of the slave
 * 8259's interrupt request register (IRQ 14) through PCI I/O port 0A0h,
 * and the installed-OS drivers use the kit's base=1f0 irq=14.  Keep the
 * PCI INTA path as well - the interrupt PLD has an IDE input.
 */
static void eb164_ide_irq_fanout(void *opaque, int n, int level)
{
    qemu_irq *fan = opaque;

    qemu_set_irq(fan[0], level);
    qemu_set_irq(fan[1], level);
}

static void eb164_cpu_reset(void *opaque)
{
    EB164ResetInfo *ri = opaque;
    AlphaCPU *cpu = ri->cpu;
    CPUState *cs = CPU(cpu);

    cpu_reset(cs);

    /*
     * Bootstrap ROM handoff (SROM output parameters, AlphaPC 164 TRM
     * table C-1): r1/r2/r3 Bcache values, r17 memory size, r18 cycle
     * count, r19 signature 0xDECB, r20 processor mask, r21 context.
     */
    cpu->env.ipr.pal_base = 0;
    /*
     * Cache/memory interface state the SROM leaves behind: ICM cleared,
     * DC_MODE = 8, ICSR = (2 << 32) | 0x4e000000.
     */
    cpu->env.ipr.icm = 0;
    cpu->env.ipr.dc_mode = 8;
    cpu->env.ipr.icsr = (2ULL << 32) | 0x4e000000ULL;
    alpha_rebuild_hflags(&cpu->env);
    cpu->env.gpregs[1] = 0x8050;              /* BC_CTL, no bcache */
    cpu->env.gpregs[2] = 0x03f24690;          /* BC_CONFIG */
    cpu->env.gpregs[3] = 0x03f24690;          /* BC_CONFIG, cache off */
    cpu->env.gpregs[15] = 0;                  /* SROM revision */
    cpu->env.gpregs[16] = 0x0000000100000007; /* EV56 pass 1 */
    cpu->env.gpregs[17] = ri->ram_size;       /* memory size, bytes */
    cpu->env.gpregs[18] = 2000;               /* cycle count, ps */
    cpu->env.gpregs[19] = 0xdecb0001;         /* signature + revision */
    cpu->env.gpregs[20] = 1;                  /* active processor mask */
    cpu->env.gpregs[21] = 0;                  /* system context */
    cpu_set_pc(cs, ri->entry | R_PC_PAL_MODE_MASK);
}

/*
 * VGA legacy window as the operating system reaches it (the "hose" view).
 *
 * The EB164's ARC firmware talks to the S3 through the CIA's PCI aperture
 * (PCI 0A.0000 -> PCI memory space, see eb164_create_flash() and the
 * 0x86.000A.0000 mapping), and a PC-style driver reaches the same memory
 * through a second, byte-expanded window whose CPU base is 0xFC00.C00000:
 * NT 3.51's vga driver maps the VGA text plane at physical 0xFC01.728000 and
 * writes it with the sparse encoding (one PCI byte per 32 bytes of CPU
 * address), i.e. VA 0xB28080 = sparse(PCI 0x59404).  Measured with `-d mmu`
 * (alpha_cpu_tlb_fill logs virt=0xB28000 -> phys=0xFC01728000) and by tracing
 * setupdd's console stores.  The firmware's own console, which reaches the
 * card through PCI 0A.0000, renders correctly, so the window below only has
 * to give the OS the same 128 KB the firmware sees:
 *
 *     CPU 0xFC00.C00000 + (pci_byte << 5) <-> PCI 0A.0000 + pci_byte
 *
 * (the OS's "hose" view numbers the legacy window from PCI 0x4.0000, which is
 * exactly 0x60000 below the chipset's 0A.0000).  Without it the NT setup
 * console output is dropped and the screen stays black even though the
 * keyboard, disk and interrupts all work.
 */
static uint64_t eb164_vga_hose_read(void *opaque, hwaddr addr, unsigned size)
{
    EB164MachineState *ms = opaque;
    hwaddr pci = 0xa0000 + ((addr >> 8) << 3) + (((addr >> 7) & 1) << 2);
    unsigned lane = (addr >> 5) & 3;
    unsigned xsize = MIN(size, 4u);
    uint8_t buf[4] = { 0 };
    uint64_t val = 0;

    address_space_read(&ms->vga_pci_as, pci + lane, MEMTXATTRS_UNSPECIFIED,
                       buf, xsize);
    memcpy(&val, buf, xsize);
    return val << (8 * lane);
}

static void eb164_vga_hose_write(void *opaque, hwaddr addr, uint64_t val,
                                 unsigned size)
{
    EB164MachineState *ms = opaque;
    hwaddr pci = 0xa0000 + ((addr >> 8) << 3) + (((addr >> 7) & 1) << 2);
    unsigned lane = (addr >> 5) & 3;
    unsigned xsize = MIN(size, 4u);
    uint8_t buf[4];

    val >>= 8 * lane;
    memcpy(buf, &val, xsize);
    address_space_write(&ms->vga_pci_as, pci + lane, MEMTXATTRS_UNSPECIFIED,
                        buf, xsize);
}

static const MemoryRegionOps eb164_vga_hose_ops = {
    .read = eb164_vga_hose_read,
    .write = eb164_vga_hose_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
    .impl = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
};

static void eb164_machine_init(MachineState *machine)
{
    EB164MachineState *ms = EB164_MACHINE(machine);
    MachineClass *mc = MACHINE_GET_CLASS(machine);
    MemoryRegion *sysmem = get_system_memory();
    AlphaCPU *cpu;
    DeviceState *alcor, *i82378, *ide, *dev, *pld;
    EB164ResetInfo *ri;
    PCIBus *pci_bus;
    ISABus *isa_bus;
    qemu_irq rtc_irq;
    MC146818RtcState *rtc;
    Clock *cpu_refclk;
    const char *firmware;
    int i;
    g_autofree char *filename = NULL;
    int size;

    if (machine->ram_size > EB164_MAX_RAM_SIZE) {
        g_autofree char *sz = size_to_str(EB164_MAX_RAM_SIZE);
        error_report("can't model more than %s of RAM", sz);
        exit(EXIT_FAILURE);
    }

    /*
     * 21164 (EV5) at 500MHz.  The SRM console uses RPCC for timing, so the
     * reference clock must be connected.
     */
    cpu_refclk = clock_new(OBJECT(machine), "system-refclk");
    clock_set_hz(cpu_refclk, 500000000);

    ri = g_new0(EB164ResetInfo, 1);
    cpu = ALPHA_CPU(object_new(machine->cpu_type));
    qdev_prop_set_bit(DEVICE(cpu), "start-powered-off", false);
    qdev_connect_clock_in(DEVICE(cpu), "cpu-refclk", cpu_refclk);
    qdev_realize_and_unref(DEVICE(cpu), NULL, &error_fatal);
    ri->cpu = cpu;
    ri->ram_size = machine->ram_size;
    qemu_register_reset(eb164_cpu_reset, ri);

    memory_region_add_subregion(sysmem, 0, machine->ram);

    /* Alcor chipset. */
    alcor = qdev_new(TYPE_ALCOR_CHIPSET);
    qdev_prop_set_uint32(alcor, "ram-size", machine->ram_size / MiB);
    qdev_prop_set_uint32(alcor, "num-cpus", 1);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(alcor), &error_fatal);

    sysbus_mmio_map(SYS_BUS_DEVICE(alcor), 0, ALCOR_IO_BASE);

    /* CPU interrupt lines (TRM table 4-2). */
    qdev_connect_gpio_out_named(alcor, "cpu-mchk", 0,
                                qdev_get_gpio_in(DEVICE(cpu),
                                                 ALPHA_CPU_INPUT_IRQ0));
    qdev_connect_gpio_out_named(alcor, "cpu-device", 0,
                                qdev_get_gpio_in(DEVICE(cpu),
                                                 ALPHA_CPU_INPUT_IRQ1));
    qdev_connect_gpio_out_named(alcor, "cpu-timer", 0,
                                qdev_get_gpio_in(DEVICE(cpu),
                                                 ALPHA_CPU_INPUT_IRQ2));
    qdev_connect_gpio_out_named(alcor, "cpu-ipi", 0,
                                qdev_get_gpio_in(DEVICE(cpu),
                                                 ALPHA_CPU_INPUT_IRQ3));

    pci_bus = alcor_get_pci_bus(alcor);

    /*
     * Intel 82378ZB PCI-to-ISA bridge at IDSEL ad19 (device 5).
     * Its interrupt output (the cascade of the two 8259 PICs) feeds
     * the board interrupt PLD.
     */
    i82378 = DEVICE(pci_create_simple(pci_bus, PCI_DEVFN(5, 0), "i82378"));
    qdev_connect_gpio_out(i82378, 0,
                          qdev_get_gpio_in_named(alcor, "irq", 0));
    isa_bus = ISA_BUS(qdev_get_child_bus(i82378, "isa.0"));

    /* Interrupt PLD (MACH210A) at ISA 0x804-0x806 (TRM 4.4.1). */
    pld = DEVICE(isa_new(TYPE_ALCOR_PLD));
    object_property_set_link(OBJECT(pld), "upstream", OBJECT(alcor),
                             &error_fatal);
    isa_realize_and_unref(ISA_DEVICE(pld), isa_bus, &error_fatal);

    /* FLASH_ADR19 bank-select latch (ISA 0x800, TRM 4.3.5). */
    dev = DEVICE(isa_new(TYPE_EB164_FLASH_ADDR));
    object_property_set_link(OBJECT(dev), "machine", OBJECT(ms),
                             &error_fatal);
    isa_realize_and_unref(ISA_DEVICE(dev), isa_bus, &error_fatal);

    /* DS1287-compatible time-of-year clock; its IRQ is the TOY clock. */
    rtc_irq = qdev_get_gpio_in_named(alcor, "irq", 1);
    rtc = mc146818_rtc_init(isa_bus, 1900, rtc_irq);
    /*
     * The SROM leaves the TOY clock running with the 1024 Hz periodic
     * interrupt enabled: register A = 0x26 (32.768 kHz divider, rate select
     * 1024 Hz) and register B = 0x46 (24-hour, BCD, PIE).  The DS1287's IRQ
     * pin is wired to the CIA's TOY clock input (alcor interrupt source 1)
     * and firmware entered from the SROM handoff relies on those ticks, but
     * QEMU's RTC resets with PIE clear - so put the SROM's state back after
     * every reset.  Registered here (after the device exists, i.e. after the
     * device's own reset handler) so it runs last.  The ROM would poke these
     * through the RTC's index/data ports; the periodic timer re-reads PIE on
     * every tick, so a direct register write is equivalent.
     */
    qemu_register_reset(eb164_rtc_srom_reset, rtc);

    /* SMC FDC37C935 combination controller (floppy, UARTs, parallel, K/M). */
    {
        DeviceState *sio = isa_create_simple(isa_bus, TYPE_SMC37C669_SUPERIO);

        /*
         * The bootstrap ROM/BIOS leaves the keyboard scanning before it
         * hands over, and QEMU's PS/2 keyboard silently drops every key
         * event while scanning is disabled - firmware entered straight from
         * the handoff would never see a keypress.  Apply the state the SROM
         * would have left instead of sending the 0xf4 command, so no
         * acknowledge byte is pending for the guest to trip over.
         */
        Object *kbc = object_resolve_path_component(OBJECT(sio), "i8042");
        Object *kbd = kbc ? object_resolve_path_component(kbc, "ps2kbd")
                          : NULL;

        if (kbd) {
            PS2_KBD_DEVICE(kbd)->scan_enabled = 1;
        }
    }

    /*
     * CMD PCI0646 IDE controller at IDSEL ad22 (device 8).  Both
     * channels are wired on the board and the part resets in legacy
     * mode (task file registers at 1F0h/3F4h/170h/374h).
     */
    ide = DEVICE(pci_new(PCI_DEVFN(8, 0), "cmd646-ide"));
    qdev_prop_set_uint32(ide, "secondary", 1);
    pci_realize_and_unref(PCI_DEVICE(ide), pci_bus, &error_fatal);
    pci_ide_create_devs(PCI_DEVICE(ide));

    for (i = 0; i < 2; i++) {
        qemu_irq *fan = g_new0(qemu_irq, 2);

        fan[0] = qdev_get_gpio_in(ide, i);  /* PCI INTA -> interrupt PLD */
        fan[1] = isa_get_irq(NULL, 14 + i); /* ISA IRQ14/15 -> 8259 pair */
        /*
         * Only replace the bus's interrupt output with the board fan-out.
         * Do NOT use ide_bus_init_output_irq() here: it re-initialises the
         * bus and sets bus->dma = &ide_dma_nop, which throws away the PCI
         * bus-master DMA ops that bmdma_init() installed when cmd646 was
         * realized.  With the nop DMA ops every bus-master transfer
         * silently moves no data, so the guest's CD/file reads come back
         * with pages missing (measured 2026-09-26: only 9 of the 23 pages
         * of ALPHA\USETUP.EXE landed in RAM) and NT setup cannot start its
         * user-mode phase.
         */
        PCI_IDE(ide)->bus[i].irq =
            qemu_allocate_irq(eb164_ide_irq_fanout, fan, 0);
    }

    /* SCSI disk setup (no on-board SCSI; 53C810 in a slot). */
    if (drive_get_max_bus(IF_SCSI) >= 0) {
        dev = DEVICE(pci_create_simple(pci_bus, -1, "lsi53c810"));
        lsi53c8xx_handle_legacy_cmdline(dev);
    }

    /* VGA device (expansion slot). */
    if (machine->enable_graphics && vga_interface_type != VGA_NONE) {
        pci_vga_init(pci_bus);
    }

    /* Networking devices. */
    pci_init_nic_devices(pci_bus, mc->default_nic);

    /* Load the console image. */
    if ((firmware = machine->firmware) == NULL) {
        firmware = FLASH_ROM_FILENAME;
    }

    if ((filename = qemu_find_file(QEMU_FILE_TYPE_BIOS, firmware)) != NULL) {
        if ((size = eb164_load_srom(ri, filename, NULL)) == 0) {
            size = load_image_targphys(filename, SRM_LOAD_ADDRESS,
                                       PALCODE_ROM_SIZE, NULL);
            ri->entry = SRM_LOAD_ADDRESS;
        }
    } else {
        size = -1;
    }

    if (size < 0) {
        error_report("could not load firmware image '%s'; use -bios with an "
                     "AlphaPC 164 SRM/AlphaBIOS image (e.g. pc164srm.rom "
                     "or pc164_v5_4.rom)", firmware);
        exit(1);
    }

    eb164_create_flash(ms, pci_bus, sysmem, filename);

    /*
     * VGA hole (AlphaPC 164 TRM 4.3.6): the chipset decodes the classic PC
     * VGA windows - A.0000-B.FFFF framebuffer and C.0000-C.7FFF option ROM -
     * from PCI space, so the CPU does not see RAM there.  Model it as an
     * overlay on the RAM region; without it a PC-style VGA driver (and the
     * Windows NT vga/videoprt stack) writes its text/graphics into RAM and
     * the display never changes.
     *
     * Default off: the EB164 ARC firmware in our model keeps its CD/file
     * buffers inside A.0000-B.FFFF, so with the hole enabled its reads land
     * in VGA memory and the loader fails ("ntkrnlmp.exe could not be
     * loaded").  Enable with '-machine eb164,vga-hole=on' when running an OS
     * whose VGA driver needs the legacy windows.
     */
    if (ms->vga_hole) {
        memory_region_init_alias(&ms->vga_legacy, NULL, "eb164.vga-legacy",
                                 pci_bus->address_space_mem, 0xa0000, 0x20000);
        memory_region_add_subregion_overlap(sysmem, 0xa0000,
                                            &ms->vga_legacy, 2);
    }

    /*
     * The OS's view of the legacy VGA window (see eb164_vga_hose_read()).
     * Always present: the ARC firmware does not touch this range, and without
     * it a PC-style VGA driver's output is dropped and the OS console stays
     * black.
     */
    address_space_init(&ms->vga_pci_as, pci_bus->address_space_mem,
                       "eb164.vga-pci");
    memory_region_init_io(&ms->vga_hose, NULL, &eb164_vga_hose_ops, ms,
                          "eb164.vga-hose", 0x20000ULL << 5);
    memory_region_add_subregion_overlap(sysmem,
                                        0xFC00C00000ULL + (0x40000ULL << 5),
                                        &ms->vga_hose, 0);
}

static bool eb164_get_vga_hole(Object *obj, Error **errp)
{
    return EB164_MACHINE(obj)->vga_hole;
}

static void eb164_set_vga_hole(Object *obj, bool value, Error **errp)
{
    EB164_MACHINE(obj)->vga_hole = value;
}

static void eb164_class_init(ObjectClass *oc, const void *data)
{
    static const char * const valid_cpu_types[] = {
        ALPHA_CPU_TYPE_NAME("ev5"),
        ALPHA_CPU_TYPE_NAME("ev56"),
        NULL
    };
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "AlphaPC 164 (21164 + 21172 Alcor)";
    mc->init = eb164_machine_init;
    mc->block_default_type = IF_IDE;
    mc->default_cpu_type = ALPHA_CPU_TYPE_NAME("ev5");
    mc->default_ram_id = "eb164.ram";
    mc->default_ram_size = 128 * MiB;
    mc->default_nic = "tulip";
    mc->max_cpus = 1;
    mc->valid_cpu_types = valid_cpu_types;
    mc->no_floppy = !module_object_class_by_name(TYPE_ISA_FDC);
    mc->no_parallel = !module_object_class_by_name(TYPE_ISA_PARALLEL);

    /*
     * Decode the legacy VGA windows (A.0000-B.FFFF framebuffer,
     * C.0000-C.7FFF option ROM) from PCI space instead of RAM, the way the
     * AlphaPC 164 chipset does.  Off by default because the ARC firmware in
     * this model keeps its file buffers in A.0000-B.FFFF; an OS whose VGA
     * driver needs the legacy windows wants it on
     * ('-machine eb164,vga-hole=on').
     */
    object_class_property_add_bool(oc, "vga-hole", eb164_get_vga_hole,
                                   eb164_set_vga_hole);
    object_class_property_set_description(
        oc, "vga-hole",
        "decode the legacy VGA windows from PCI space (default: off)");
}

static const TypeInfo eb164_machine_info = {
    .name          = TYPE_EB164_MACHINE,
    .parent        = TYPE_MACHINE,
    .class_init    = eb164_class_init,
    .instance_size = sizeof(EB164MachineState),
};

static void eb164_register_types(void)
{
    type_register_static(&eb164_machine_info);
}

type_init(eb164_register_types)
