/*
 * Pulsar clock chip of the PowerMac7,3 (U3/K2), on the UniNorth KeyWest
 * I2C bus at 0xd2.  Mac OS X writes its registers to freeze the timebase
 * while starting a second CPU.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_MISC_MACIO_PULSAR_H
#define HW_MISC_MACIO_PULSAR_H

#include "hw/i2c/i2c.h"
#include "qom/object.h"

#define TYPE_PULSAR_CLOCK "pulsar-clock"
OBJECT_DECLARE_SIMPLE_TYPE(PulsarState, PULSAR_CLOCK)

struct PulsarState {
    I2CSlave parent_obj;

    uint8_t regs[256];
    uint8_t ptr;
    bool ptr_set;
};

#endif /* HW_MISC_MACIO_PULSAR_H */
