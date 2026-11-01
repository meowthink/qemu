/*
 * Apple "Bandit" ARBus-to-PCI bridge (343S1126).
 *
 * Apple, Network Server Hardware Developer Notes, 4.4: config cycles are
 * indirect, "similar to mechanism #1", with the Config Address register at
 * $F2800000 (bus 0) / $F4800000 (bus 1) and the Config Data register at
 * $F2C00000 / $F4C00000.  The address register's contents are "driven
 * directly on the PCI AD lines, un-modified", so the ROM writes plain PCI
 * configuration addresses into it ($00020008 = device 2, register 8).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/irq.h"
#include "hw/pci/pci_device.h"
#include "hw/pci-host/bandit.h"
#include "qapi/error.h"
#include "qemu/module.h"
#include "qemu/log.h"
#include "qom/object.h"
#include "system/ioport.h"

#define BANDIT_CFG_SELF_DEV     11
#define BANDIT_CFG_ADDR_SELECT  0x48
#define BANDIT_CFG_MODE_SELECT  0x50
#define BANDIT_BIG_ENDIAN       0x01000000

static bool bandit_lanes_reversed(BanditState *s)
{
    return !(s->mode_select & BANDIT_BIG_ENDIAN);
}

static hwaddr bandit_port_offset(BanditState *s, hwaddr addr, unsigned size)
{
    return bandit_lanes_reversed(s) ? addr ^ (8 - size) : addr;
}

static uint64_t bandit_port_value(BanditState *s, uint64_t val,
                                  unsigned size)
{
    if (!bandit_lanes_reversed(s)) {
        return val;
    }
    return size == 4 ? bswap32(val) : bswap16(val);
}

/* The config ports are little-endian (dev notes 4.4: "All register accesses
 * will be default swapped by Bandit"): byte 0 of the port holds bits 7-0. */
static uint64_t bandit_config_addr_read(void *opaque, hwaddr addr, unsigned size)
{
    BanditState *s = opaque;
    uint64_t mask = size == 4 ? 0xffffffffu : (1u << (8 * size)) - 1;

    addr = bandit_port_offset(s, addr, size);
    return bandit_port_value(s, s->config_reg >> (8 * (addr & 3)), size)
           & mask;
}

static void bandit_config_addr_write(void *opaque, hwaddr addr, uint64_t val,
                                     unsigned size)
{
    BanditState *s = opaque;
    uint32_t bytes = size >= 4 ? 0xffffffffu : (1u << (8 * size)) - 1;
    unsigned shift;

    addr = bandit_port_offset(s, addr, size);
    val = bandit_port_value(s, val, size);
    shift = 8 * (addr & 3);
    s->config_reg = (s->config_reg & ~(bytes << shift))
                    | (((uint32_t)val & bytes) << shift);
}

/*
 * Config address decode (reference bandit.c decode_type0): bit 0 set = a
 * type-1 cycle (no subordinate buses here), zero = idle, and bits 31-11
 * carry a ONE-HOT IDSEL -- device N is bit N.  Function is bits 10-8 and
 * the register select is bits 7-2.
 */
static bool bandit_decode(uint32_t a, uint32_t *devfn_out, unsigned *reg_out)
{
    uint32_t idsel = a & 0xfffff800u;
    unsigned dev, fn;

    if (a == 0 || (a & 1u) || idsel == 0 || (idsel & (idsel - 1)) != 0) {
        return false;
    }
    dev = 31 - clz32(idsel);
    fn = (a >> 8) & 7u;
    *devfn_out = (dev << 3) | fn;
    *reg_out = a & 0xfcu;
    return true;
}

/* The data port decodes eight bytes; its low two bits select the byte lane of
 * the latched dword.  Those bits are not part of the config address register
 * (bit 0 there means type-1 cycle), so they are added after the latch decode. */
static bool bandit_cfg_byte(uint32_t config_reg, hwaddr port_off, uint32_t *out)
{
    uint32_t devfn;
    unsigned reg;

    if (!bandit_decode(config_reg, &devfn, &reg)) {
        return false;
    }
    *out = (devfn << 8) | reg | (port_off & 3u);
    return true;
}

static uint32_t bandit_addr_select(BanditState *s)
{
    if (s->small_win == 0xf3000000) {
        return 0x0100000cu;
    }
    return 0x02000030u;
}

static bool bandit_self_cfg_byte(BanditState *s, uint32_t config_reg,
                                 hwaddr port_off, uint8_t *out, bool write,
                                 uint8_t value)
{
    uint32_t devfn;
    unsigned reg;
    unsigned off;
    uint32_t v;

    if (!bandit_decode(config_reg, &devfn, &reg)) {
        return false;
    }
    if (PCI_SLOT(devfn) != BANDIT_CFG_SELF_DEV || PCI_FUNC(devfn) != 0) {
        return false;
    }

    off = reg + (port_off & 3u);
    switch (off & ~3u) {
    case 0x00:
        v = 0x0001106b;
        break;
    case 0x08:
        v = 0x06000003;
        break;
    case BANDIT_CFG_ADDR_SELECT:
        v = bandit_addr_select(s);
        break;
    case BANDIT_CFG_MODE_SELECT:
        v = s->mode_select;
        break;
    default:
        v = 0;
        break;
    }

    if (write) {
        if ((off & ~3u) != BANDIT_CFG_MODE_SELECT) {
            return true;
        }
        unsigned shift = 8 * (off & 3u);
        s->mode_select = (s->mode_select & ~(0xffu << shift))
                         | ((uint32_t)value << shift);
    }
    *out = v >> (8 * (off & 3u));
    return true;
}

static uint64_t bandit_config_data_read(void *opaque, hwaddr addr, unsigned size)
{
    BanditState *s = opaque;
    PCIBus *bus = PCI_HOST_BRIDGE(s)->bus;
    uint32_t val = 0;
    unsigned i;

    addr = bandit_port_offset(s, addr, size);
    for (i = 0; i < size; i++) {
        uint32_t a;
        uint8_t b;

        if (bandit_self_cfg_byte(s, s->config_reg, addr + i, &b, false, 0)) {
            val |= (uint32_t)b << (8 * i);
        } else if (!bandit_cfg_byte(s->config_reg, addr + i, &a)) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "bandit: cfg read  idle/type1 (a=0x%x)\n",
                          s->config_reg);
            return 0xffffffffu;
        } else {
            val |= (uint32_t)pci_data_read(bus, a, 1) << (8 * i);
        }
    }
    qemu_log_mask(LOG_GUEST_ERROR,
                  "bandit: cfg read  idsel=%u fn=%u reg=0x%02x -> 0x%x\n",
                  (s->config_reg >> 11) & 0x1f, (s->config_reg >> 8) & 7,
                  (unsigned)((s->config_reg & 0xfcu) | (addr & 3u)), val);
    return bandit_port_value(s, val, size);
}

static void bandit_config_data_write(void *opaque, hwaddr addr, uint64_t val,
                                     unsigned size)
{
    BanditState *s = opaque;
    PCIBus *bus = PCI_HOST_BRIDGE(s)->bus;
    unsigned i;

    addr = bandit_port_offset(s, addr, size);
    val = bandit_port_value(s, val, size);
    for (i = 0; i < size; i++) {
        uint32_t a;
        uint8_t b;

        if (bandit_self_cfg_byte(s, s->config_reg, addr + i, &b, true,
                                 (val >> (8 * i)) & 0xff)) {
            continue;
        } else if (!bandit_cfg_byte(s->config_reg, addr + i, &a)) {
            return;
        } else {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "bandit: cfg write idsel=%u fn=%u reg=0x%02x "
                          "val=0x%x\n",
                          (a >> 11) & 0x1f, (a >> 8) & 7, a & 0xff,
                          (unsigned)(val >> (8 * i)) & 0xff);
            pci_data_write(bus, a, (val >> (8 * i)) & 0xff, 1);
        }
    }
}

static const MemoryRegionOps bandit_config_addr_ops = {
    .read = bandit_config_addr_read,
    .write = bandit_config_addr_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

static const MemoryRegionOps bandit_config_data_ops = {
    .read = bandit_config_data_read,
    .write = bandit_config_data_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

static void bandit_set_irq(void *opaque, int irq_num, int level)
{
    BanditState *s = opaque;

    qemu_set_irq(s->irqs[irq_num], level);
}

/*
 * The board's external interrupt map (dev notes 4.2): the two on-board
 * 53C825As leave on Grand Central's FW0_Int/FW1_Int lines and the slots keep
 * the TNT INTx-by-slot pattern; the bridges' own errors are ganged on the
 * last output, Error_Int.
 */
static int bandit_map_irq(PCIDevice *d, int irq_num)
{
    unsigned slot = PCI_SLOT(d->devfn);

    switch (slot) {
    case 17:
        return 0;                       /* FW0_Int */
    case 18:
        return 1;                       /* FW1_Int */
    default:
        return 2 + (slot & 1);          /* slot INTx, Error_Int */
    }
}

static const Property bandit_props[] = {
    DEFINE_PROP_UINT32("small-win", BanditState, small_win, 0xf3000000),
    DEFINE_PROP_UINT32("large-win", BanditState, large_win, 0x80000000),
};

static void bandit_realize(DeviceState *dev, Error **errp)
{
    BanditState *s = BANDIT_PCI_HOST_BRIDGE(dev);
    PCIHostState *phb = PCI_HOST_BRIDGE(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    int i;

    for (i = 0; i < ARRAY_SIZE(s->irqs); i++) {
        sysbus_init_irq(sbd, &s->irqs[i]);
    }

    s->mode_select = BANDIT_BIG_ENDIAN;
    memory_region_init(&s->pci_mem, OBJECT(s), "bandit-pci-mem", 0x100000000ULL);
    memory_region_init_io(&s->config_addr, OBJECT(s), &bandit_config_addr_ops,
                          s, "bandit-config-addr", 8);
    memory_region_init_io(&s->config_data, OBJECT(s), &bandit_config_data_ops,
                          s, "bandit-config-data", 8);
    memory_region_init_alias(&s->win_small, OBJECT(s), "bandit-win-small",
                             &s->pci_mem, s->small_win, 16 * MiB);
    memory_region_init_alias(&s->win_large, OBJECT(s), "bandit-win-large",
                             &s->pci_mem, s->large_win, 256 * MiB);
    /*
     * PCI I/O address 0 sits at the bridge base and the window is 8 MB (the
     * bridge's own `ranges`): a guest reaches a device's I/O ports, the
     * legacy VGA block at $3B0-$3DF included, by touching this window.
     * Bandit byte-swaps I/O cycles as it does configuration cycles.
     */
    memory_region_init_io(&s->pci_io, OBJECT(s), &unassigned_io_ops, s,
                          "bandit-pci-io", 8 * MiB);

    sysbus_init_mmio(sbd, &s->config_addr);
    sysbus_init_mmio(sbd, &s->config_data);
    sysbus_init_mmio(sbd, &s->win_small);
    sysbus_init_mmio(sbd, &s->win_large);
    sysbus_init_mmio(sbd, &s->pci_io);

    phb->bus = pci_register_root_bus(dev, NULL, bandit_set_irq, bandit_map_irq,
                                     s,
                                     &s->pci_mem, &s->pci_io, 0, 4,
                                     TYPE_PCI_BUS);
}

static void bandit_reset(DeviceState *dev)
{
    BanditState *s = BANDIT_PCI_HOST_BRIDGE(dev);

    s->config_reg = 0;
    s->mode_select = BANDIT_BIG_ENDIAN;
}

static void bandit_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = bandit_realize;
    dc->legacy_reset = bandit_reset;
    device_class_set_props(dc, bandit_props);
    set_bit(DEVICE_CATEGORY_BRIDGE, dc->categories);
}

static const TypeInfo bandit_type_info = {
    .name = TYPE_BANDIT_PCI_HOST_BRIDGE,
    .parent = TYPE_PCI_HOST_BRIDGE,
    .instance_size = sizeof(BanditState),
    .class_init = bandit_class_init,
    .interfaces = (InterfaceInfo[]) {
        { INTERFACE_CONVENTIONAL_PCI_DEVICE },
        { },
    },
};

static void bandit_register_types(void)
{
    type_register_static(&bandit_type_info);
}

type_init(bandit_register_types);
