/*
 * PowerMac NewWorld MacIO GPIO emulation
 *
 * Copyright (c) 2016 Benjamin Herrenschmidt
 * Copyright (c) 2018 Mark Cave-Ayland
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "qemu/osdep.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "hw/misc/macio/macio.h"
#include "hw/misc/macio/gpio.h"
#include "hw/core/irq.h"
#include "hw/core/nmi.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "trace.h"

enum MacioGPIORegisterBits {
    OUT_DATA   = 1,
    IN_DATA    = 2,
    OUT_ENABLE = 4,
};

void macio_set_gpio(MacIOGPIOState *s, uint32_t gpio, bool state)
{
    uint8_t new_reg;


    if (s->gpio_regs[gpio] & OUT_ENABLE) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "GPIO: Setting GPIO %d while it's an output\n", gpio);
    }

    new_reg = s->gpio_regs[gpio] & ~IN_DATA;
    if (state) {
        new_reg |= IN_DATA;
    }

    /* For CPU reset lines (GPIO 4, 15, 16), always update so we can assert/deassert */
    if ((gpio != 4) && (gpio != 15) && (gpio != 16) && (new_reg == s->gpio_regs[gpio])) {
        return;
    }

    s->gpio_regs[gpio] = new_reg;

    switch (gpio) {
    case 1:
        /* Level low - original GPIO 1 handling */
        if (!state) {
            qemu_irq_raise(s->gpio_extirqs[gpio]);
        } else {
            qemu_irq_lower(s->gpio_extirqs[gpio]);
        }
        break;

    case 3:
        /* Level low */
        if (!state) {
            qemu_irq_raise(s->gpio_extirqs[gpio]);
        } else {
            qemu_irq_lower(s->gpio_extirqs[gpio]);
        }
        break;

    case 4:
        /* Active low, CPU1 reset */
        if (!state) {
            qemu_irq_raise(s->gpio_extirqs[gpio]);
        } else {
            qemu_irq_lower(s->gpio_extirqs[gpio]);
        }
        break;

    case 15:
        /* Active low, CPU2 reset */
        if (!state) {
            qemu_irq_raise(s->gpio_extirqs[gpio]);
        } else {
            qemu_irq_lower(s->gpio_extirqs[gpio]);
        }
        break;

    case 16:
        /* Active low, CPU3 reset */
        if (!state) {
            qemu_irq_raise(s->gpio_extirqs[gpio]);
        } else {
            qemu_irq_lower(s->gpio_extirqs[gpio]);
        }
        break;

    case 9:
        /* Edge, triggered by NMI below */
        if (state) {
            qemu_irq_raise(s->gpio_extirqs[gpio]);
        } else {
            qemu_irq_lower(s->gpio_extirqs[gpio]);
        }
        break;

    default:
        qemu_log_mask(LOG_UNIMP, "GPIO: setting unimplemented GPIO %d", gpio);
    }
}

static void macio_gpio_write(void *opaque, hwaddr addr, uint64_t value,
                             unsigned size)
{
    MacIOGPIOState *s = opaque;
    uint8_t ibit;

    /* Level registers are read-only */
    if (addr < 8) {
        return;
    }

    /* Adjust for the first 8 bytes being level registers */
    addr -= 8;
    if (addr < 36) {
        value &= ~IN_DATA;

        if (value & OUT_ENABLE) {
            ibit = (value & OUT_DATA) << 1;
        } else {
            ibit = s->gpio_regs[addr] & IN_DATA;
        }

        if (addr == (KL_GPIO_RESET_CPU1 - KEYLARGO_GPIO_EXTINT_0)) {
            /* For CPU1 reset: (0x58+0x04) - 0x58 = 0x04 */
            if (!(value & OUT_ENABLE)) {
                ibit = 1; /* Ensure pulled high unless driven low */
            }
            macio_set_gpio(s, 4, ibit);
            qemu_set_irq(s->cpu_reset[1],
                         (value & OUT_ENABLE) && !(value & OUT_DATA));
        } else if (addr == (KL_GPIO_RESET_CPU2 - KEYLARGO_GPIO_EXTINT_0)) {
            /* For CPU2 reset: (0x58+0x0f) - 0x58 = 0x0F */
            if (!(value & OUT_ENABLE)) {
                ibit = 1; /* Ensure pulled high unless driven low */
            }
            macio_set_gpio(s, 15, ibit);
            qemu_set_irq(s->cpu_reset[2],
                         (value & OUT_ENABLE) && !(value & OUT_DATA));
        } else if (addr == (KL_GPIO_RESET_CPU3 - KEYLARGO_GPIO_EXTINT_0)) {
            /* For CPU3 reset: (0x58+0x10) - 0x58 = 0x10 */
            if (!(value & OUT_ENABLE)) {
                ibit = 1; /* Ensure pulled high unless driven low */
            }
            macio_set_gpio(s, 16, ibit);
            qemu_set_irq(s->cpu_reset[3],
                         (value & OUT_ENABLE) && !(value & OUT_DATA));
        } else if (addr == (KL_GPIO_SMU_DOORBELL -
                            KEYLARGO_GPIO_EXTINT_0) ||
                   addr == (KL_GPIO_SMU_DOORBELL2 -
                            KEYLARGO_GPIO_EXTINT_0)) {
            /*
             * The Apple BootROM drives the doorbell low, then polls it until
             * the SMU acknowledges.  Model the SMU's immediate response.
             */
            s->gpio_regs[addr] = (value & ~IN_DATA) | IN_DATA;
        } else {
            s->gpio_regs[addr] = value | ibit;
        }
    }
}

static uint64_t macio_gpio_read(void *opaque, hwaddr addr, unsigned size)
{
    MacIOGPIOState *s = opaque;
    uint64_t val = 0;

    /* Levels regs */
    if (addr < 8) {
        val = s->gpio_levels[addr];
    } else {
        addr -= 8;

        if (addr < 36) {
            val = s->gpio_regs[addr];
        }
    }
    
    return val;
}

static const MemoryRegionOps macio_gpio_ops = {
    .read = macio_gpio_read,
    .write = macio_gpio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 1,
    },
};

static void macio_gpio_init(Object *obj)
{
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    MacIOGPIOState *s = MACIO_GPIO(obj);
    int i;

    for (i = 0; i < KEYLARGO_GPIO_EXTINT_CNT; i++) {
        sysbus_init_irq(sbd, &s->gpio_extirqs[i]);
    }
    qdev_init_gpio_out_named(DEVICE(obj), s->cpu_reset, "cpu-reset",
                             MACIO_GPIO_MAX_CPUS);

    memory_region_init_io(&s->gpiomem, OBJECT(s), &macio_gpio_ops, obj,
                          "gpio", 0x30);
    sysbus_init_mmio(sbd, &s->gpiomem);
}

static const VMStateDescription vmstate_macio_gpio = {
    .name = "macio_gpio",
    .version_id = 0,
    .minimum_version_id = 0,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8_ARRAY(gpio_levels, MacIOGPIOState, 8),
        VMSTATE_UINT8_ARRAY(gpio_regs, MacIOGPIOState, 36),
        VMSTATE_END_OF_LIST()
    }
};

static void macio_gpio_reset(DeviceState *dev)
{
    MacIOGPIOState *s = MACIO_GPIO(dev);

    /* GPIO 1 is up by default (original behavior) */
    macio_set_gpio(s, 1, true);
    /* GPIO 3 is up by default */
    macio_set_gpio(s, 3, true);
    
    /* 
     * CPU reset GPIOs (4, 15, 16) are NOT initialized here.
     * They start low (CPUs held in reset).
     * The OS will explicitly deassert them to start secondary CPUs.
     */
}

static void macio_gpio_nmi(NMIState *n, int cpu_index, Error **errp)
{
    macio_set_gpio(MACIO_GPIO(n), 9, true);
    macio_set_gpio(MACIO_GPIO(n), 9, false);
}

static const Property macio_gpio_properties[] = {
    DEFINE_PROP_UINT32("nb-cpus", MacIOGPIOState, nb_cpus, 1),
    DEFINE_PROP_BOOL("k2", MacIOGPIOState, k2, false),
};

static void macio_gpio_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    NMIClass *nc = NMI_CLASS(oc);

    device_class_set_props(dc, macio_gpio_properties);

    device_class_set_legacy_reset(dc, macio_gpio_reset);
    dc->vmsd = &vmstate_macio_gpio;
    nc->nmi_monitor_handler = macio_gpio_nmi;
}

static const TypeInfo macio_gpio_init_info = {
    .name          = TYPE_MACIO_GPIO,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MacIOGPIOState),
    .instance_init = macio_gpio_init,
    .class_init    = macio_gpio_class_init,
    .interfaces = (const InterfaceInfo[]) {
        { TYPE_NMI },
        { }
    },
};

static void macio_gpio_register_types(void)
{
    type_register_static(&macio_gpio_init_info);
}

type_init(macio_gpio_register_types)
