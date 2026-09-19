/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/log.h"
#include "hw/core/registerfields.h"
#include "hw/alpha/tigbus.h"
#include "hw/alpha/tigcontrol.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "system/runstate.h"
#include "trace.h"

struct TigControlState {
    /*< private >*/
    TigBusDevice parent_obj;

    /*< public >*/
    MemoryRegion iomem;
    uint32_t base_address;
    uint8_t regs[64];
    qemu_irq halt_irq[4];
};

static void tig_control_update_halt_lines(TigControlState *s);
static int tig_control_post_load(void *opaque, int version_id);

static const VMStateDescription vmstate_tig_control = {
    .name = "tig-control",
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = tig_control_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8_ARRAY(regs, TigControlState, 64),
        VMSTATE_END_OF_LIST()
    }
};

static const Property tig_control_properties[] = {
    DEFINE_PROP_UINT32("base-address", TigControlState, base_address,
                       0xc00000),
};

REG8(TRR,               0x00)
REG8(SMIR,              0x01)
REG8(CPUIR,             0x02)
REG8(PSIR,              0x03)
REG8(MOD_INFO,          0x04)
REG8(CLK_INFO,          0x05)
REG8(CHIP_INFO,         0x06)
/*
 * The TIG registers are sparse: each register occupies one 64-byte
 * slot in the TIG bus address space, so the device offset is the
 * documented TIG address divided by 0x40 (see the ES40 TIG table:
 * tpcr 0x200, ttcr 0x3c0, ev6_halt 0x5c0, ipcr0 0xa00, ...).
 */
REG8(TPCR,              0x08)
REG8(PLL_DATA,          0x0a)
REG8(PLL_CLK,           0x0b)
REG8(EV6_INIT,          0x0c)
REG8(CSLEEP,            0x0d)
REG8(SMCR,              0x0e)
REG8(TTCR,              0x0f)
REG8(CLR_IRQ5,          0x10)
REG8(CLR_IRQ4,          0x11)
REG8(CLR_PWR_FLT_DET,   0x12)
REG8(CLR_TEMP_WARN,     0x13)
REG8(CLR_TEMP_FAIL,     0x14)
REG8(EV6_HALT,          0x17)
REG8(SRCR0,             0x18)
REG8(SRCR1,             0x19)
REG8(FRAR0,             0x1c)
REG8(FRAR1,             0x1d)
REG8(FWMR0,             0x20)
REG8(FWMR1,             0x21)
REG8(FWMR2,             0x22)
REG8(FWMR3,             0x23)
REG8(IPCR0,             0x28)
REG8(IPCR1,             0x29)
REG8(IPCR2,             0x2a)
REG8(IPCR3,             0x2b)
REG8(IPCR4,             0x2c)

static void tig_control_update_halt_lines(TigControlState *s)
{
    uint8_t lines = s->regs[A_TTCR] | s->regs[A_EV6_HALT];
    int i;

    /* Bit n of (TTCR | EV6_HALT) is CPU n's halt/IRQ4 line.  */
    for (i = 0; i < ARRAY_SIZE(s->halt_irq); i++) {
        qemu_set_irq(s->halt_irq[i], (lines >> i) & 1);
    }
}

static int tig_control_post_load(void *opaque, int version_id)
{
    tig_control_update_halt_lines(opaque);
    return 0;
}

static uint64_t tig_control_read(void *opaque, hwaddr addr, unsigned size)
{
    TigControlState *s = opaque;
    uint64_t ret;

    switch (addr) {
    case R_TRR:
        ret = s->regs[A_TRR];
        break;
    case R_SMIR:
        ret = s->regs[A_SMIR];
        break;
    case R_CPUIR:
        ret = s->regs[A_CPUIR];
        break;
    case R_PSIR:
        ret = s->regs[A_PSIR];
        break;
    case R_MOD_INFO:
        ret = s->regs[A_MOD_INFO];
        break;
    case R_CLK_INFO:
        ret = s->regs[A_CLK_INFO];
        break;
    case R_CHIP_INFO:
        ret = s->regs[A_CHIP_INFO];
        break;
    case R_TPCR:
        ret = s->regs[A_TPCR];
        break;
    case R_PLL_DATA:
        ret = s->regs[A_PLL_DATA];
        break;
    case R_PLL_CLK:
        ret = s->regs[A_PLL_CLK];
        break;
    case R_EV6_INIT:
        ret = s->regs[A_EV6_INIT];
        break;
    case R_CSLEEP:
        ret = s->regs[A_CSLEEP];
        break;
    case R_SMCR:
        ret = s->regs[A_SMCR];
        break;
    case R_TTCR:
        ret = s->regs[A_TTCR];
        break;
    case R_CLR_IRQ5:
    case R_CLR_IRQ4:
        /* Write-only latch clears; IRQ4/IRQ5 are level-driven.  */
        ret = 0;
        break;
    case R_CLR_PWR_FLT_DET:
        ret = s->regs[A_CLR_PWR_FLT_DET];
        break;
    case R_CLR_TEMP_WARN:
        ret = s->regs[A_CLR_TEMP_WARN];
        break;
    case R_CLR_TEMP_FAIL:
        ret = s->regs[A_CLR_TEMP_FAIL];
        break;
    case R_EV6_HALT:
        ret = s->regs[A_EV6_HALT];
        break;
    case R_SRCR0:
        ret = s->regs[A_SRCR0];
        break;
    case R_SRCR1:
        ret = s->regs[A_SRCR1];
        break;
    case R_FRAR0:
        ret = s->regs[A_FRAR0];
        break;
    case R_FRAR1:
        ret = s->regs[A_FRAR1];
        break;
    case R_FWMR0:
        ret = s->regs[A_FWMR0];
        break;
    case R_FWMR1:
        ret = s->regs[A_FWMR1];
        break;
    case R_FWMR2:
        ret = s->regs[A_FWMR2];
        break;
    case R_FWMR3:
        ret = s->regs[A_FWMR3];
        break;
    case R_IPCR0:
        ret = s->regs[A_IPCR0];
        break;
    case R_IPCR1:
        ret = s->regs[A_IPCR1];
        break;
    case R_IPCR2:
        ret = s->regs[A_IPCR2];
        break;
    case R_IPCR3:
        ret = s->regs[A_IPCR3];
        break;
    case R_IPCR4:
        ret = s->regs[A_IPCR4];
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: invalid offset 0x%04" HWADDR_PRIx "\n",
                      __func__, addr);
        ret = 0;
    }

    trace_tig_control_read(addr, size, size);
    return ret;
}

static void tig_control_write(void *opaque, hwaddr addr, uint64_t val,
                           unsigned size)
{
    TigControlState *s = opaque;

    trace_tig_control_write(addr, val, size);
    switch (addr) {
    case R_TRR:
        s->regs[A_TRR] = val;
        break;
    case R_SMIR:
        s->regs[A_SMIR] = val;
        break;
    case R_CPUIR:
        s->regs[A_CPUIR] = val;
        break;
    case R_PSIR:
        s->regs[A_PSIR] = val;
        break;
    case R_MOD_INFO:
        s->regs[A_MOD_INFO] = val;
        break;
    case R_CLK_INFO:
        s->regs[A_CLK_INFO] = val;
        break;
    case R_CHIP_INFO:
        s->regs[A_CHIP_INFO] = val;
        break;
    case R_TPCR:
        s->regs[A_TPCR] = val;
        break;
    case R_PLL_DATA:
        s->regs[A_PLL_DATA] = val;
        break;
    case R_PLL_CLK:
        s->regs[A_PLL_CLK] = val;
        break;
    case R_EV6_INIT:
        s->regs[A_EV6_INIT] = val;
        break;
    case R_CSLEEP:
        s->regs[A_CSLEEP] = val;
        break;
    case R_SMCR:
        s->regs[A_SMCR] = val;
        break;
    case R_TTCR:
        s->regs[A_TTCR] = val;
        tig_control_update_halt_lines(s);
        break;
    case R_CLR_IRQ5:
    case R_CLR_IRQ4:
        /* Write-only latch clears.  */
        break;
    case R_CLR_PWR_FLT_DET:
        s->regs[A_CLR_PWR_FLT_DET] = val;
        break;
    case R_CLR_TEMP_WARN:
        s->regs[A_CLR_TEMP_WARN] = val;
        break;
    case R_CLR_TEMP_FAIL:
        s->regs[A_CLR_TEMP_FAIL] = val;
        break;
    case R_EV6_HALT:
        s->regs[A_EV6_HALT] = val;
        tig_control_update_halt_lines(s);
        break;
    case R_SRCR0:
    case R_SRCR1:
        s->regs[addr] = val;
        if (val & 0x30) {
            /*
             * The firmware updater writes 0x30 here when it wants the
             * system to restart after updating the flash.
             */
            qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
        }
        break;
    case R_FRAR0:
        s->regs[A_FRAR0] = val;
        break;
    case R_FRAR1:
        s->regs[A_FRAR1] = val;
        break;
    case R_FWMR0:
        s->regs[A_FWMR0] = val;
        break;
    case R_FWMR1:
        s->regs[A_FWMR1] = val;
        break;
    case R_FWMR2:
        s->regs[A_FWMR2] = val;
        break;
    case R_FWMR3:
        s->regs[A_FWMR3] = val;
        break;
    case R_IPCR0:
        s->regs[A_IPCR0] = val;
        break;
    case R_IPCR1:
        s->regs[A_IPCR1] = val;
        break;
    case R_IPCR2:
        s->regs[A_IPCR2] = val;
        break;
    case R_IPCR3:
        s->regs[A_IPCR3] = val;
        break;
    case R_IPCR4:
        s->regs[A_IPCR4] = val;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: invalid offset 0x%04" HWADDR_PRIx "\n",
                      __func__, addr);
        break;
    }
}

static const MemoryRegionOps tig_control_ops = {
    .read = tig_control_read,
    .write = tig_control_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 1,
    },
    .impl = {
        .min_access_size = 1,
        .max_access_size = 1,
    },
};

static void tig_control_realize(DeviceState *dev, Error **errp)
{
    TigControlState *s = TIG_CONTROL(dev);
    TigBusDevice *tbd = TIG_BUS_DEVICE(dev);
    MemoryRegion *iomem;

    iomem = tigbus_address_space(tigbus_from_device(tbd));
    memory_region_add_subregion(iomem, s->base_address, &s->iomem);
}

static void tig_control_init(Object *obj)
{
    TigControlState *s = TIG_CONTROL(obj);

    qdev_init_gpio_out(DEVICE(obj), s->halt_irq, ARRAY_SIZE(s->halt_irq));

    memory_region_init_io(&s->iomem, obj, &tig_control_ops, s,
                          "tig-control", sizeof(s->regs));
}

static void tig_control_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = tig_control_realize;
    dc->vmsd = &vmstate_tig_control;
    /* Reason: needs to be wired up by board code. */
    dc->user_creatable = false;
    device_class_set_props(dc, tig_control_properties);
}

static const TypeInfo tig_control_info = {
    .name          = TYPE_TIG_CONTROL,
    .parent        = TYPE_TIG_BUS_DEVICE,
    .instance_size = sizeof(TigControlState),
    .instance_init = tig_control_init,
    .class_init    = tig_control_class_init,
};

static void tig_control_register_types(void)
{
    type_register_static(&tig_control_info);
}

type_init(tig_control_register_types)
