/*
 * TNT "Hammerhead" system/memory controller (Apple 343S1142).
 *
 * The ANS's memory controller.  The early boot code only needs the
 * identification and status registers to come back with the values a TNT
 * board reports, plus a writable register file for the timing, bank-base and
 * L2 configuration registers it programs:
 *
 *   0xF8000000 part ID    0x39000000  (first byte $39 = the TNT path the
 *                                      ROM's identification routine takes;
 *                                      $3001xxxx would be a 7200/Catalyst)
 *   0xF8000020 MACHID     0x40000000  (bit 30 = the 9500/Shiner class)
 *   0xF8000030 CPUSpeed   0x00000003  (read-only; bits 0:2 are the MuxAdr
 *                                      strap, 3 = 50 MHz ARBus, this board)
 *   0xF8000090 ArbConfig  0x00000000  (TwoCPU clear: uniprocessor)
 *   0xF80000B0 WhoAmI     0x10000000  (primary CPU)
 *   0xF80000E0 L2 config  0x8n000000  (bit 7 = present, bits 1:0 = size code:
 *                                      0 = 512 KB, 1 = 256 KB, 2 = 1 MB,
 *                                      3 = 4 MB; bit 2 ignored)
 *   0xF80001C0+            DRAM bank base register pairs (A/B per bank)
 *
 * The values are the ones the shipping ANS ROM and Apple's developer notes
 * attest; the L2 size code was decoded on the front panel (see the reference
 * in ANS-BRINGUP.md).  The +0xE0 presence/size bits are a *strap*: the ROM
 * writes $70 there mid-test and reads it back, so they have to survive a
 * write or the cache it reports comes out as 0 KB.
 *
 * ERS Table 12; section numbers in the comments.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/core/sysbus.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/hammerhead.h"
#include "qemu/module.h"
#include "qemu/log.h"
#include "migration/vmstate.h"

enum {
    HH_ID        = 0x000,   /* 3.2.1 */
    HH_MACHID    = 0x020,   /* 3.2.3 */
    HH_CPUSPEED  = 0x030,   /* 3.2.4 */
    HH_ARBCONFIG = 0x090,   /* 3.2.9 */
    HH_WHOAMI    = 0x0b0,   /* 3.2.11 */
    HH_L2CFG     = 0x0e0,   /* 3.2.14 */
};

/*
 * +0x30 CPUSpeed: bits 0:2 are strapped from MuxAdr[3:5] at reset and are
 * read-only.  The ANS's ARBus runs at 50 MHz, i.e. strap code 3.
 */
#define HH_CPUSPEED_STRAP  0x00000003u

/* +0xE0 strap for this board: 1 MB cache DIMM present (ANS 700/150). */
#define HH_L2CFG_STRAP  0x82000000u
#define HH_L2CFG_MASK   0x83000000u

/* ANS_HH_DBG=1: log every register access (addr, value, size, direction). */
static bool hammerhead_dbg_enabled(void)
{
    static int dbg = -1;

    if (dbg < 0) {
        dbg = getenv("ANS_HH_DBG") != NULL;
    }
    return dbg;
}

static uint32_t hammerhead_reset_value(hwaddr off)
{
    switch (off) {
    case HH_ID:       return 0x39000000u;
    case HH_MACHID:   return 0x40000000u;
    case HH_CPUSPEED: return HH_CPUSPEED_STRAP;
    case HH_ARBCONFIG: return 0x00000000u;
    case HH_WHOAMI:   return 0x10000000u;   /* primary CPU */
    case HH_L2CFG:    return HH_L2CFG_STRAP;
    default:          return 0x00000000u;
    }
}

static uint64_t hammerhead_read(void *opaque, hwaddr addr, unsigned size)
{
    HammerheadState *s = opaque;
    uint64_t val = 0;
    unsigned i;

    for (i = 0; i < size; i++) {
        hwaddr a = (addr + i) % HAMMERHEAD_REGION_SIZE;
        unsigned lane = a % HAMMERHEAD_STRIDE;
        uint8_t b = lane < 4
            ? (s->regs[a / HAMMERHEAD_STRIDE] >> (8 * (3 - lane))) & 0xff
            : 0x00;
        val |= (uint64_t)b << (8 * (size - 1 - i));
    }
    if (hammerhead_dbg_enabled()) {
        fprintf(stderr, "hammerhead: read  +0x%03x size %u -> 0x%x\n",
                (unsigned)addr, size, (unsigned)val);
    }
    return val;
}

static void hammerhead_write(void *opaque, hwaddr addr, uint64_t val,
                             unsigned size)
{
    HammerheadState *s = opaque;
    unsigned i;

    if (hammerhead_dbg_enabled()) {
        fprintf(stderr, "hammerhead: write +0x%03x size %u <- 0x%x\n",
                (unsigned)addr, size, (unsigned)val);
    }
    for (i = 0; i < size; i++) {
        hwaddr a = (addr + i) % HAMMERHEAD_REGION_SIZE;
        unsigned lane = a % HAMMERHEAD_STRIDE;
        uint32_t *reg = &s->regs[a / HAMMERHEAD_STRIDE];
        uint8_t b = val >> (8 * (size - 1 - i));

        if (lane >= 4) {
            continue;
        }
        if (a == HH_CPUSPEED) {
            /* Strapped and read-only: the PLL code cannot be written. */
            continue;
        }
        *reg &= ~(0xffu << (8 * (3 - lane)));
        *reg |= (uint32_t)b << (8 * (3 - lane));
        if (a == HH_L2CFG) {
            /* The presence and size bits are strapped, not software state. */
            *reg = (*reg & ~HH_L2CFG_MASK) | HH_L2CFG_STRAP;
        }
    }
}

static const MemoryRegionOps hammerhead_ops = {
    .read = hammerhead_read,
    .write = hammerhead_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static void hammerhead_reset(DeviceState *dev)
{
    HammerheadState *s = HAMMERHEAD(dev);
    unsigned i;

    for (i = 0; i < HAMMERHEAD_NREGS; i++) {
        s->regs[i] = hammerhead_reset_value(i * HAMMERHEAD_STRIDE);
    }
    s->l2_strap = HH_L2CFG_STRAP;
}

static void hammerhead_realize(DeviceState *dev, Error **errp)
{
    HammerheadState *s = HAMMERHEAD(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &hammerhead_ops, s,
                          "hammerhead", HAMMERHEAD_REGION_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
}

static const VMStateDescription vmstate_hammerhead = {
    .name = "hammerhead",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, HammerheadState, HAMMERHEAD_NREGS),
        VMSTATE_END_OF_LIST()
    },
};

static void hammerhead_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = hammerhead_realize;
    device_class_set_legacy_reset(dc, hammerhead_reset);
    dc->vmsd = &vmstate_hammerhead;
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo hammerhead_info = {
    .name = TYPE_HAMMERHEAD,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(HammerheadState),
    .class_init = hammerhead_class_init,
};

static void hammerhead_register_types(void)
{
    type_register_static(&hammerhead_info);
}

type_init(hammerhead_register_types);
