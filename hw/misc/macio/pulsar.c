/*
 * Pulsar clock chip of the PowerMac7,3 -- a register file with an
 * auto-incrementing pointer.  Ported from cat7's powermac73
 * hw/misc/macio/keywest_i2c.c.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/i2c/i2c.h"
#include "hw/misc/macio/pulsar.h"
#include "migration/vmstate.h"
#include "qemu/module.h"

static int pulsar_event(I2CSlave *i2c, enum i2c_event event)
{
    PulsarState *s = PULSAR_CLOCK(i2c);

    if (event == I2C_START_SEND) {
        s->ptr_set = false;
    }
    return 0;
}

static int pulsar_send(I2CSlave *i2c, uint8_t data)
{
    PulsarState *s = PULSAR_CLOCK(i2c);

    if (!s->ptr_set) {
        s->ptr = data;
        s->ptr_set = true;
    } else {
        s->regs[s->ptr++] = data;
    }
    return 0;
}

static uint8_t pulsar_recv(I2CSlave *i2c)
{
    PulsarState *s = PULSAR_CLOCK(i2c);

    return s->regs[s->ptr++];
}

static const VMStateDescription vmstate_pulsar = {
    .name = TYPE_PULSAR_CLOCK,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, PulsarState),
        VMSTATE_UINT8_ARRAY(regs, PulsarState, 256),
        VMSTATE_UINT8(ptr, PulsarState),
        VMSTATE_BOOL(ptr_set, PulsarState),
        VMSTATE_END_OF_LIST()
    }
};

static void pulsar_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(oc);

    k->event = pulsar_event;
    k->send = pulsar_send;
    k->recv = pulsar_recv;
    dc->vmsd = &vmstate_pulsar;
}

static const TypeInfo pulsar_info = {
    .name          = TYPE_PULSAR_CLOCK,
    .parent        = TYPE_I2C_SLAVE,
    .instance_size = sizeof(PulsarState),
    .class_init    = pulsar_class_init,
};

static void pulsar_register_types(void)
{
    type_register_static(&pulsar_info);
}

type_init(pulsar_register_types)
