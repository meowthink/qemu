/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * AlphaPC 164LX motherboard emulation.
 *
 * References:
 *   AlphaPC 164LX Motherboard Technical Reference Manual, EC-R46WC-TE
 *   AlphaPC 164LX User Manual
 */

#include "qemu/osdep.h"
#include "qemu/cutils.h"
#include "qemu/datadir.h"
#include "qemu/error-report.h"
#include "qemu/log.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "net/net.h"
#include "hw/alpha/pyxis.h"
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
#include "hw/isa/isa.h"
#include "hw/isa/superio.h"
#include "hw/pci/pci.h"
#include "hw/rtc/mc146818rtc.h"
#include "system/reset.h"
#include "system/system.h"
#include "exec/target_page.h"
#include "cpu.h"
#include "internals.h"

#define TYPE_LX164_MACHINE MACHINE_TYPE_NAME("lx164")
OBJECT_DECLARE_SIMPLE_TYPE(LX164MachineState, LX164_MACHINE)

#define LX164_MAX_RAM_SIZE (1 * GiB)

#define SRM_LOAD_ADDRESS        0x0
#define PALCODE_ROM_SIZE        (2 * MiB)

#define FLASH_ROM_FILENAME      "lx164.rom"

/*
 * The board flash is an Intel 28F001BX, 1MB in two 512KB banks
 * selected by FLASH_ADR19 (ISA port 0x800 bit 0).  Model it as a CFI
 * flash so the console drivers' command/status protocol
 * (0x50/0x70/0xFF/0x90) works.
 */
#define LX164_FLASH_BLOCKS      16
#define LX164_FLASH_SECTOR      (64 * KiB)
#define LX164_FLASH_BANK_SIZE   (512 * KiB)

/*
 * State passed to the CPU reset handler: the SROM handoff must be
 * applied after the CPU's own reset clears the registers.
 */
typedef struct LX164ResetInfo {
    AlphaCPU *cpu;
    uint64_t entry;
    uint64_t ram_size;
} LX164ResetInfo;

struct LX164MachineState {
    /*< private >*/
    MachineState parent_obj;

    /*< public >*/
    MemoryRegion flash_bank_low;
    MemoryRegion flash_bank_high;
    MemoryRegion flash_byte;
    MemoryRegion flash_int8;
    MemoryRegion flash_isa;
};

/*
 * ISA port 0x800 bit 0 = FLASH_ADR19: selects the upper 512KB bank
 * of the 1MB flash in the FFF8.0000-FFFF.FFFF window (TRM 4.3.5).
 */
#define TYPE_LX164_FLASH_ADDR "lx164-flash-addr"
OBJECT_DECLARE_SIMPLE_TYPE(LX164FlashAddrState, LX164_FLASH_ADDR)

struct LX164FlashAddrState {
    ISADevice parent_obj;
    LX164MachineState *machine;
    MemoryRegion io;
};

static void lx164_flash_addr_write(void *opaque, hwaddr addr,
                                   uint64_t val, unsigned size)
{
    LX164FlashAddrState *s = opaque;
    LX164MachineState *ms = s->machine;
    bool upper = (val & 1) != 0;

    memory_region_set_enabled(&ms->flash_bank_low, !upper);
    memory_region_set_enabled(&ms->flash_bank_high, upper);
}

static const MemoryRegionOps lx164_flash_addr_ops = {
    .write = lx164_flash_addr_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 1,
    },
};

static void lx164_flash_addr_realize(DeviceState *dev, Error **errp)
{
    LX164FlashAddrState *s = LX164_FLASH_ADDR(dev);

    memory_region_init_io(&s->io, OBJECT(dev), &lx164_flash_addr_ops, s,
                          "lx164.flash-addr", 1);
    isa_register_ioport(ISA_DEVICE(dev), &s->io, 0x800);
}

static const Property lx164_flash_addr_properties[] = {
    DEFINE_PROP_LINK("machine", LX164FlashAddrState, machine,
                     TYPE_LX164_MACHINE, LX164MachineState *),
};

static void lx164_flash_addr_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = lx164_flash_addr_realize;
    device_class_set_props(dc, lx164_flash_addr_properties);
}

static const TypeInfo lx164_flash_addr_info = {
    .name          = TYPE_LX164_FLASH_ADDR,
    .parent        = TYPE_ISA_DEVICE,
    .instance_size = sizeof(LX164FlashAddrState),
    .class_init    = lx164_flash_addr_class_init,
};

static void lx164_flash_addr_register_types(void)
{
    type_register_static(&lx164_flash_addr_info);
}

type_init(lx164_flash_addr_register_types)

static void lx164_create_flash(LX164MachineState *ms, PCIBus *pci_bus,
                               MemoryRegion *sysmem, const char *filename)
{
    MemoryRegion *flash_mem;
    MemoryRegion *pci_mem;
    PFlashCFI01 *flash;
    DeviceState *dev;
    uint8_t *storage;
    int size, loaded;

    size = get_image_size(filename, NULL);
    if (size < 0 || size > LX164_FLASH_BLOCKS * LX164_FLASH_SECTOR) {
        error_report("flash image '%s' does not fit the 1MB flash", filename);
        exit(EXIT_FAILURE);
    }

    dev = qdev_new(TYPE_PFLASH_CFI01);
    qdev_prop_set_string(dev, "name", "lx164.flash");
    qdev_prop_set_uint32(dev, "num-blocks", LX164_FLASH_BLOCKS);
    qdev_prop_set_uint64(dev, "sector-length", LX164_FLASH_SECTOR);
    qdev_prop_set_uint8(dev, "width", 1);
    qdev_prop_set_uint8(dev, "device-width", 1);
    qdev_prop_set_uint8(dev, "max-device-width", 1);
    qdev_prop_set_uint16(dev, "id0", 0x89);    /* Intel */
    qdev_prop_set_uint16(dev, "id1", 0xb0);    /* 28F001BX */
    qdev_prop_set_uint16(dev, "id2", 0x00);
    qdev_prop_set_uint16(dev, "id3", 0x00);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    flash = PFLASH_CFI01(dev);
    flash_mem = pflash_cfi01_get_memory(flash);

    /*
     * No backing block device: fill the flash array directly from the
     * image (an erased bank would read 0xFF).
     */
    storage = memory_region_get_ram_ptr(flash_mem);
    memset(storage, 0xff, LX164_FLASH_BLOCKS * LX164_FLASH_SECTOR);
    loaded = load_image_size(filename, storage, size);
    if (loaded != size) {
        error_report("could not load flash image '%s'", filename);
        exit(EXIT_FAILURE);
    }

    /*
     * The 21174 presents the ISA FROM_BASE window (PCI memory
     * FFF8.0000-FFFF.FFFF) to the CPU through both the sparse memory
     * windows (HAE_MEM) and the INT8 dense alias, i.e. every firmware
     * flash access lands in the pchip's PCI memory space at
     * 0xFFF80000.  Map the two 512KB banks there, selected by
     * FLASH_ADR19.
     */
    pci_mem = pci_bus->address_space_mem;
    memory_region_init_alias(&ms->flash_bank_low, NULL,
                             "lx164.flash.bank0", flash_mem, 0,
                             LX164_FLASH_BANK_SIZE);
    memory_region_init_alias(&ms->flash_bank_high, NULL,
                             "lx164.flash.bank1", flash_mem,
                             LX164_FLASH_BANK_SIZE, LX164_FLASH_BANK_SIZE);
    memory_region_add_subregion_overlap(pci_mem, 0xFFF80000,
                                        &ms->flash_bank_low, 1);
    memory_region_add_subregion_overlap(pci_mem, 0xFFF80000,
                                        &ms->flash_bank_high, 1);
    memory_region_set_enabled(&ms->flash_bank_high, false);

    /* CPU-space windows: byte window, INT8 alias, top-of-space ISA alias. */
    memory_region_init_alias(&ms->flash_byte, NULL, "lx164.flash.byte",
                             flash_mem, 0,
                             LX164_FLASH_BLOCKS * LX164_FLASH_SECTOR);
    memory_region_add_subregion_overlap(sysmem, PYXIS_FLASH,
                                        &ms->flash_byte, 1);
    memory_region_init_alias(&ms->flash_int8, NULL, "lx164.flash.int8",
                             flash_mem, 0,
                             LX164_FLASH_BLOCKS * LX164_FLASH_SECTOR);
    memory_region_add_subregion_overlap(sysmem, PYXIS_FLASH_INT8,
                                        &ms->flash_int8, 1);
    memory_region_init_alias(&ms->flash_isa, NULL, "lx164.flash.isa",
                             flash_mem, 0,
                             LX164_FLASH_BLOCKS * LX164_FLASH_SECTOR);
    memory_region_add_subregion(sysmem, 0xfffff00000ULL, &ms->flash_isa);
}

/*
 * AlphaPC 164LX firmware images are "makerom" images: a header with a
 * validation pattern, image size, destination address and firmware id,
 * followed by the console image.  The bootstrap ROM scans flash for
 * this header, copies the image to its destination and enters it in
 * PAL mode.
 */
static ssize_t lx164_load_srom(LX164ResetInfo *ri, const char *filename,
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

    rom_add_blob_fixed("lx164.srom", (const char *)p + header_size,
                       image_size, dest);
    ri->entry = dest;
    return image_size;
}

static void lx164_cpu_reset(void *opaque)
{
    LX164ResetInfo *ri = opaque;
    AlphaCPU *cpu = ri->cpu;
    CPUState *cs = CPU(cpu);

    cpu_reset(cs);

    /*
     * Bootstrap ROM handoff ("SROM Initialization Output Parameters"):
     * r1/r2/r3 backup-cache control/config, r15 the SROM revision,
     * r16-r21 the processor id, memory size, cycle count, signature,
     * processor mask and system context.  The console reset PALcode
     * verifies that (r19 >> 16) & 0xffff equals 0xDECB and saves the
     * whole set into its handoff area.
     */
    cpu->env.ipr.pal_base = 0;
    cpu->env.gpregs[1] = 0x8050;              /* BC_CTL, no bcache */
    cpu->env.gpregs[2] = 0x03f24690;          /* BC_CONFIG, 500MHz */
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

static void lx164_machine_init(MachineState *machine)
{
    LX164MachineState *ms = LX164_MACHINE(machine);
    MachineClass *mc = MACHINE_GET_CLASS(machine);
    MemoryRegion *sysmem = get_system_memory();
    AlphaCPU *cpu;
    DeviceState *pyxis, *i82378, *ide, *dev, *pld;
    LX164ResetInfo *ri;
    PCIBus *pci_bus;
    ISABus *isa_bus;
    qemu_irq rtc_irq;
    Clock *cpu_refclk;
    const char *firmware;
    g_autofree char *filename = NULL;
    int size;

    if (machine->ram_size > LX164_MAX_RAM_SIZE) {
        g_autofree char *sz = size_to_str(LX164_MAX_RAM_SIZE);
        error_report("can't model more than %s of RAM", sz);
        exit(EXIT_FAILURE);
    }

    /*
     * 21164A at 500MHz.  The SRM console uses RPCC for timing, so the
     * reference clock must be connected.
     */
    cpu_refclk = clock_new(OBJECT(machine), "system-refclk");
    clock_set_hz(cpu_refclk, 500000000);

    ri = g_new0(LX164ResetInfo, 1);
    cpu = ALPHA_CPU(object_new(machine->cpu_type));
    qdev_prop_set_bit(DEVICE(cpu), "start-powered-off", false);
    qdev_connect_clock_in(DEVICE(cpu), "cpu-refclk", cpu_refclk);
    qdev_realize_and_unref(DEVICE(cpu), NULL, &error_fatal);
    ri->cpu = cpu;
    ri->ram_size = machine->ram_size;
    qemu_register_reset(lx164_cpu_reset, ri);

    memory_region_add_subregion(sysmem, 0, machine->ram);

    /* Pyxis chipset. */
    pyxis = qdev_new(TYPE_PYXIS_CHIPSET);
    qdev_prop_set_uint32(pyxis, "ram-size", machine->ram_size / MiB);
    qdev_prop_set_uint32(pyxis, "num-cpus", 1);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(pyxis), &error_fatal);

    sysbus_mmio_map(SYS_BUS_DEVICE(pyxis), 0, PYXIS_IO_BASE);

    /* CPU interrupt lines (TRM table 4-2). */
    qdev_connect_gpio_out_named(pyxis, "cpu-mchk", 0,
                                qdev_get_gpio_in(DEVICE(cpu),
                                                 ALPHA_CPU_INPUT_IRQ0));
    qdev_connect_gpio_out_named(pyxis, "cpu-device", 0,
                                qdev_get_gpio_in(DEVICE(cpu),
                                                 ALPHA_CPU_INPUT_IRQ1));
    qdev_connect_gpio_out_named(pyxis, "cpu-timer", 0,
                                qdev_get_gpio_in(DEVICE(cpu),
                                                 ALPHA_CPU_INPUT_IRQ2));
    qdev_connect_gpio_out_named(pyxis, "cpu-ipi", 0,
                                qdev_get_gpio_in(DEVICE(cpu),
                                                 ALPHA_CPU_INPUT_IRQ3));

    pci_bus = pyxis_get_pci_bus(pyxis);

    /*
     * Intel 82378ZB PCI-to-ISA bridge at IDSEL ad19 (device 5).
     * Its interrupt output (the cascade of the two 8259 PICs) feeds
     * the board interrupt PLD.
     */
    i82378 = DEVICE(pci_create_simple(pci_bus, PCI_DEVFN(5, 0), "i82378"));
    qdev_connect_gpio_out(i82378, 0,
                          qdev_get_gpio_in_named(pyxis, "irq", 0));
    isa_bus = ISA_BUS(qdev_get_child_bus(i82378, "isa.0"));

    /* Interrupt PLD (MACH210A) at ISA 0x804-0x806 (TRM 4.4.1). */
    pld = DEVICE(isa_new(TYPE_PYXIS_PLD));
    object_property_set_link(OBJECT(pld), "upstream", OBJECT(pyxis),
                             &error_fatal);
    isa_realize_and_unref(ISA_DEVICE(pld), isa_bus, &error_fatal);

    /* FLASH_ADR19 bank-select latch (ISA 0x800, TRM 4.3.5). */
    dev = DEVICE(isa_new(TYPE_LX164_FLASH_ADDR));
    object_property_set_link(OBJECT(dev), "machine", OBJECT(ms),
                             &error_fatal);
    isa_realize_and_unref(ISA_DEVICE(dev), isa_bus, &error_fatal);

    /* DS1287-compatible time-of-year clock; its IRQ is the TOY clock. */
    rtc_irq = qdev_get_gpio_in_named(pyxis, "irq", 1);
    mc146818_rtc_init(isa_bus, 1900, rtc_irq);

    /* SMC FDC37C935 combination controller (floppy, UARTs, parallel, K/M). */
    isa_create_simple(isa_bus, TYPE_SMC37C669_SUPERIO);

    /* CMD PCI0646 IDE controller at IDSEL ad22 (device 8). */
    ide = DEVICE(pci_create_simple(pci_bus, PCI_DEVFN(8, 0), "cmd646-ide"));
    pci_ide_create_devs(PCI_DEVICE(ide));

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
        if ((size = lx164_load_srom(ri, filename, NULL)) == 0) {
            size = load_image_targphys(filename, SRM_LOAD_ADDRESS,
                                       PALCODE_ROM_SIZE, NULL);
            ri->entry = SRM_LOAD_ADDRESS;
        }
    } else {
        size = -1;
    }

    if (size < 0) {
        error_report("could not load SROM image '%s'; use -bios with an "
                     "AlphaPC 164LX SRM/AlphaBIOS image (e.g. "
                     "lx164srm.rom)", firmware);
        exit(1);
    }

    lx164_create_flash(ms, pci_bus, sysmem, filename);
}

static void lx164_class_init(ObjectClass *oc, const void *data)
{
    static const char * const valid_cpu_types[] = {
        ALPHA_CPU_TYPE_NAME("ev5"),
        ALPHA_CPU_TYPE_NAME("ev56"),
        ALPHA_CPU_TYPE_NAME("pca56"),
        NULL
    };
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "AlphaPC 164LX (21164A + 21174 Pyxis)";
    mc->init = lx164_machine_init;
    mc->block_default_type = IF_IDE;
    mc->default_cpu_type = ALPHA_CPU_TYPE_NAME("ev56");
    mc->default_ram_id = "lx164.ram";
    mc->default_ram_size = 128 * MiB;
    mc->default_nic = "tulip";
    mc->max_cpus = 1;
    mc->valid_cpu_types = valid_cpu_types;
    mc->no_floppy = !module_object_class_by_name(TYPE_ISA_FDC);
    mc->no_parallel = !module_object_class_by_name(TYPE_ISA_PARALLEL);
}

static const TypeInfo lx164_machine_info = {
    .name          = TYPE_LX164_MACHINE,
    .parent        = TYPE_MACHINE,
    .class_init    = lx164_class_init,
    .instance_size = sizeof(LX164MachineState),
};

static void lx164_register_types(void)
{
    type_register_static(&lx164_machine_info);
}

type_init(lx164_register_types)
