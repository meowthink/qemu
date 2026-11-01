/*
 * TNT "Hammerhead" system/memory controller (Apple 343S1142).
 *
 * Register values from the TNT Hammerhead ASIC ERS Rev 1.1 (1995-09-25).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_MISC_HAMMERHEAD_H
#define HW_MISC_HAMMERHEAD_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_HAMMERHEAD "hammerhead"
OBJECT_DECLARE_SIMPLE_TYPE(HammerheadState, HAMMERHEAD)

/*
 * Registers are 32-bit words on 16-byte boundaries (ERS Table 12 describes
 * them as 8-bit fields; the ROM reads each 16-byte slot as one big-endian word
 * whose byte lanes 0-3 are live).
 */
#define HAMMERHEAD_REGION_SIZE 0x400
#define HAMMERHEAD_STRIDE      0x10
#define HAMMERHEAD_NREGS       (HAMMERHEAD_REGION_SIZE / HAMMERHEAD_STRIDE)

struct HammerheadState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t regs[HAMMERHEAD_NREGS];
    uint32_t l2_strap;      /* +0xE0 bits that are a hardware strap */
};

#endif /* HW_MISC_HAMMERHEAD_H */
