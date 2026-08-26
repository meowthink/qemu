/*
 * Signetics SCN2681 Dual Asynchronous Receiver/Transmitter (DUART)
 *
 * Written for the Apollo DN3000 ("SIO": channel A keyboard, channel B
 * serial console).  Registers are on 2-byte boundaries (index =
 * offset >> 1), matching the Apollo bus wiring.
 *
 * What is modelled: both channels' MR/SR/CR/CSR, 3-deep RX FIFOs,
 * instant TX to a chardev, the ISR/IMR interrupt logic, the IP input
 * port (fed from the "input-port" property - the DN3000 presents the
 * RAM configuration byte on IP0-IP6), and a lazy counter/timer good
 * enough for firmware delay loops (ISR[3] + readable count).
 *
 * Not modelled: real baud timing, break generation/detection details,
 * IP change interrupts, OP outputs (stored only).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/char/scn2681.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "migration/vmstate.h"
#include "qapi/error.h"

/* Register indexes (offset >> 1) */
#define REG_MRA         0x0     /* R/W: MR1A/MR2A */
#define REG_SRA         0x1     /* R: SRA,  W: CSRA */
#define REG_BRGTEST     0x2     /* R: BRG test, W: CRA */
#define REG_RHRA        0x3     /* R: RHRA, W: THRA */
#define REG_IPCR        0x4     /* R: IPCR, W: ACR */
#define REG_ISR         0x5     /* R: ISR,  W: IMR */
#define REG_CTU         0x6     /* R: CTU,  W: CTUR */
#define REG_CTL         0x7     /* R: CTL,  W: CTLR */
#define REG_MRB         0x8     /* R/W: MR1B/MR2B */
#define REG_SRB         0x9     /* R: SRB,  W: CSRB */
#define REG_1X16XTEST   0xa     /* R: 1x/16x test, W: CRB */
#define REG_RHRB        0xb     /* R: RHRB, W: THRB */
#define REG_IVR         0xc     /* R/W: IVR (reserved on 2681) */
#define REG_IP          0xd     /* R: input port, W: OPCR */
#define REG_STARTCNT    0xe     /* R: start counter cmd, W: set OP bits */
#define REG_STOPCNT     0xf     /* R: stop counter cmd, W: reset OP bits */

/* Status register bits */
#define SR_RXRDY        0x01
#define SR_FFULL        0x02
#define SR_TXRDY        0x04
#define SR_TXEMT        0x08
#define SR_OVERRUN      0x10

/* ISR bits */
#define ISR_TXRDYA      0x01
#define ISR_RXRDYA      0x02
#define ISR_BRKA        0x04
#define ISR_COUNTER     0x08
#define ISR_TXRDYB      0x10
#define ISR_RXRDYB      0x20
#define ISR_BRKB        0x40
#define ISR_IPC         0x80

static void scn2681_update_irq(SCN2681State *s)
{
    qemu_set_irq(s->irq, (s->isr & s->imr) != 0);
}

/* Recompute SR RX/TX bits and the per-channel ISR bits */
static void scn2681_update_status(SCN2681Channel *ch)
{
    SCN2681State *s = ch->duart;
    uint8_t rx_isr_bit = ch->idx ? ISR_RXRDYB : ISR_RXRDYA;
    uint8_t tx_isr_bit = ch->idx ? ISR_TXRDYB : ISR_TXRDYA;
    bool rx_int;

    if (ch->rx_count > 0) {
        ch->sr |= SR_RXRDY;
    } else {
        ch->sr &= ~SR_RXRDY;
    }
    if (ch->rx_count >= SCN2681_FIFO_DEPTH) {
        ch->sr |= SR_FFULL;
    } else {
        ch->sr &= ~SR_FFULL;
    }

    if (ch->tx_enabled) {
        /* TX is instantaneous: THR always empty, shifter always idle */
        ch->sr |= SR_TXRDY | SR_TXEMT;
        s->isr |= tx_isr_bit;
    } else {
        ch->sr &= ~(SR_TXRDY | SR_TXEMT);
        s->isr &= ~tx_isr_bit;
    }

    /* MR1[6]: 0 = interrupt on RxRDY, 1 = interrupt on FFULL */
    if (ch->mr[0] & 0x40) {
        rx_int = ch->rx_count >= SCN2681_FIFO_DEPTH;
    } else {
        rx_int = ch->rx_count > 0;
    }
    if (rx_int) {
        s->isr |= rx_isr_bit;
    } else {
        s->isr &= ~rx_isr_bit;
    }

    scn2681_update_irq(s);
}

static uint8_t scn2681_rx_pop(SCN2681Channel *ch)
{
    uint8_t data;

    if (ch->rx_count == 0) {
        /* reading an empty FIFO returns the last char again */
        return ch->rx_fifo[ch->rx_head];
    }
    data = ch->rx_fifo[ch->rx_head];
    ch->rx_head = (ch->rx_head + 1) % SCN2681_FIFO_DEPTH;
    ch->rx_count--;
    scn2681_update_status(ch);
    qemu_chr_fe_accept_input(&ch->chr);
    return data;
}

static void scn2681_do_command(SCN2681Channel *ch, uint8_t cmd)
{
    SCN2681State *s = ch->duart;

    switch (cmd & 0x03) {       /* receiver */
    case 0x01:
        ch->rx_enabled = true;
        qemu_chr_fe_accept_input(&ch->chr);
        break;
    case 0x02:
        ch->rx_enabled = false;
        break;
    default:
        break;
    }
    switch ((cmd >> 2) & 0x03) { /* transmitter */
    case 0x01:
        ch->tx_enabled = true;
        break;
    case 0x02:
        ch->tx_enabled = false;
        break;
    default:
        break;
    }

    switch ((cmd >> 4) & 0x07) { /* misc commands */
    case 0x1:                   /* reset MR pointer */
        ch->mr_ptr = 0;
        break;
    case 0x2:                   /* reset receiver */
        ch->rx_enabled = false;
        ch->rx_count = 0;
        ch->rx_head = 0;
        break;
    case 0x3:                   /* reset transmitter */
        ch->tx_enabled = false;
        break;
    case 0x4:                   /* reset error status */
        ch->sr &= ~0xf0;
        break;
    case 0x5:                   /* reset break change interrupt */
        s->isr &= ~(ch->idx ? ISR_BRKB : ISR_BRKA);
        break;
    case 0x6:                   /* start break */
    case 0x7:                   /* stop break */
        break;
    default:
        break;
    }
    scn2681_update_status(ch);
}

/*
 * Counter/timer.
 *
 * ACR[6:4] selects mode and clock source.  We model the internal
 * crystal sources exactly and approximate the external/TxC sources
 * with xtal/16 (nothing on the Apollo uses them).
 */
static uint64_t scn2681_ct_rate(SCN2681State *s)
{
    switch ((s->acr >> 4) & 0x7) {
    case 0x6:                   /* timer, XTAL */
        return s->xtal;
    default:                    /* XTAL/16 and all external sources */
        return s->xtal / 16;
    }
}

static bool scn2681_ct_is_timer_mode(SCN2681State *s)
{
    return (s->acr & 0x40) != 0;
}

static uint64_t scn2681_ct_elapsed_ticks(SCN2681State *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    return (uint64_t)(now - s->ct_start_ns) * scn2681_ct_rate(s)
        / NANOSECONDS_PER_SECOND;
}

static uint16_t scn2681_ct_count(SCN2681State *s)
{
    uint32_t preset = s->ct_preset ? s->ct_preset : 0x10000;
    uint64_t ticks;

    if (!s->ct_running) {
        return s->ct_preset;
    }
    ticks = scn2681_ct_elapsed_ticks(s);
    if (scn2681_ct_is_timer_mode(s)) {
        return preset - (ticks % preset);
    }
    /* counter mode: counts down and wraps through 0xffff */
    return (uint16_t)(s->ct_preset - ticks);
}

static void scn2681_ct_arm(SCN2681State *s)
{
    uint32_t preset = s->ct_preset ? s->ct_preset : 0x10000;
    uint64_t period_ns = (uint64_t)preset * NANOSECONDS_PER_SECOND
        / scn2681_ct_rate(s);

    /* don't allow a pathological IRQ storm */
    period_ns = MAX(period_ns, 1000);
    timer_mod(s->ct_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + period_ns);
}

static void scn2681_ct_expired(void *opaque)
{
    SCN2681State *s = opaque;

    s->isr |= ISR_COUNTER;
    scn2681_update_irq(s);
    if (scn2681_ct_is_timer_mode(s) && s->ct_running) {
        scn2681_ct_arm(s);
    }
}

static void scn2681_ct_start(SCN2681State *s)
{
    s->ct_running = true;
    s->ct_start_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    scn2681_ct_arm(s);
}

static void scn2681_ct_stop(SCN2681State *s)
{
    s->isr &= ~ISR_COUNTER;
    if (!scn2681_ct_is_timer_mode(s)) {
        /* counter mode: the stop command actually stops counting */
        s->ct_preset = scn2681_ct_count(s);
        s->ct_running = false;
        timer_del(s->ct_timer);
    }
    scn2681_update_irq(s);
}

static uint64_t scn2681_read(void *opaque, hwaddr offset, unsigned size)
{
    SCN2681State *s = opaque;
    unsigned reg = (offset >> 1) & 0xf;
    SCN2681Channel *ch = &s->ch[reg >= 0x8];
    uint64_t data;

    switch (reg) {
    case REG_MRA:
    case REG_MRB:
        data = ch->mr[ch->mr_ptr];
        ch->mr_ptr = 1;
        break;
    case REG_SRA:
    case REG_SRB:
        data = ch->sr;
        break;
    case REG_BRGTEST:
    case REG_1X16XTEST:
        data = 0;
        break;
    case REG_RHRA:
    case REG_RHRB:
        data = scn2681_rx_pop(ch);
        /*
         * Apollo MD PROM autobaud quirk (mirrors MAME's apollo_sio):
         * the PROM listens at 2000 baud (CSR 0x77) and deduces the
         * console's real rate from the garbled character a 9600-baud
         * CR produces.  We have no bit-level timing, so serve the
         * 0xff that a 9600-baud CR yields at 2000 baud; the PROM
         * then switches to 9600 and re-reads the next CR cleanly.
         */
        if (s->apollo_autobaud && ch->csr == 0x77) {
            data = 0xff;
        }
        break;
    case REG_IPCR:
        /* IP0-IP3 state in the low nibble, no delta detection */
        data = s->input_port & 0x0f;
        break;
    case REG_ISR:
        data = s->isr;
        break;
    case REG_CTU:
        data = scn2681_ct_count(s) >> 8;
        break;
    case REG_CTL:
        data = scn2681_ct_count(s) & 0xff;
        break;
    case REG_IVR:
        data = s->ivr;
        break;
    case REG_IP:
        /* IP6..IP0 in bits 6..0; bit 7 reads as one */
        data = 0x80 | (s->input_port & 0x7f);
        break;
    case REG_STARTCNT:
        scn2681_ct_start(s);
        data = 0;
        break;
    case REG_STOPCNT:
        scn2681_ct_stop(s);
        data = 0;
        break;
    default:
        data = 0;
        break;
    }
    return data;
}

static void scn2681_write(void *opaque, hwaddr offset, uint64_t value,
                          unsigned size)
{
    SCN2681State *s = opaque;
    unsigned reg = (offset >> 1) & 0xf;
    SCN2681Channel *ch = &s->ch[reg >= 0x8];
    uint8_t data = value;

    switch (reg) {
    case REG_MRA:
    case REG_MRB:
        ch->mr[ch->mr_ptr] = data;
        ch->mr_ptr = 1;
        scn2681_update_status(ch);
        break;
    case REG_SRA:               /* CSRA */
    case REG_SRB:               /* CSRB */
        ch->csr = data;
        break;
    case REG_BRGTEST:           /* CRA */
    case REG_1X16XTEST:         /* CRB */
        scn2681_do_command(ch, data);
        break;
    case REG_RHRA:              /* THRA */
    case REG_RHRB:              /* THRB */
        if (ch->tx_enabled) {
            /* instant transmission; chardev may be unconnected */
            qemu_chr_fe_write_all(&ch->chr, &data, 1);
        }
        break;
    case REG_IPCR:              /* ACR */
        s->acr = data;
        break;
    case REG_ISR:               /* IMR */
        s->imr = data;
        scn2681_update_irq(s);
        break;
    case REG_CTU:               /* CTUR */
        s->ct_preset = (s->ct_preset & 0x00ff) | (data << 8);
        break;
    case REG_CTL:               /* CTLR */
        s->ct_preset = (s->ct_preset & 0xff00) | data;
        break;
    case REG_IVR:
        s->ivr = data;
        break;
    case REG_IP:                /* OPCR */
        s->opcr = data;
        break;
    case REG_STARTCNT:          /* set output port bits */
        s->opr |= data;
        break;
    case REG_STOPCNT:           /* reset output port bits */
        s->opr &= ~data;
        break;
    default:
        break;
    }
}

static const MemoryRegionOps scn2681_ops = {
    .read = scn2681_read,
    .write = scn2681_write,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 1,
    },
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
    .endianness = DEVICE_BIG_ENDIAN,
};

static int scn2681_chr_can_receive(void *opaque)
{
    SCN2681Channel *ch = opaque;

    if (!ch->rx_enabled) {
        return 0;
    }
    return SCN2681_FIFO_DEPTH - ch->rx_count;
}

static void scn2681_chr_receive(void *opaque, const uint8_t *buf, int size)
{
    SCN2681Channel *ch = opaque;
    int i;

    for (i = 0; i < size && ch->rx_count < SCN2681_FIFO_DEPTH; i++) {
        int tail = (ch->rx_head + ch->rx_count) % SCN2681_FIFO_DEPTH;

        ch->rx_fifo[tail] = buf[i];
        ch->rx_count++;
    }
    scn2681_update_status(ch);
}

static void scn2681_reset_channel(SCN2681Channel *ch)
{
    ch->mr[0] = ch->mr[1] = 0;
    ch->mr_ptr = 0;
    ch->sr = 0;
    ch->csr = 0;
    ch->rx_enabled = false;
    ch->tx_enabled = false;
    ch->rx_head = 0;
    ch->rx_count = 0;
}

static void scn2681_reset(DeviceState *dev)
{
    SCN2681State *s = SCN2681(dev);

    scn2681_reset_channel(&s->ch[0]);
    scn2681_reset_channel(&s->ch[1]);
    s->acr = 0;
    s->imr = 0;
    s->isr = 0;
    s->ivr = 0x0f;
    s->opcr = 0;
    s->opr = 0;
    s->ct_preset = 0;
    s->ct_running = false;
    timer_del(s->ct_timer);
    if (s->preinit_console) {
        /*
         * -kernel boot skips the PROM, whose job it is to bring up the
         * console channel (B).  Pre-enable TX/RX so the kernel's polled
         * early console (which assumes an already-initialised SIO) works.
         */
        s->ch[1].tx_enabled = true;
        s->ch[1].rx_enabled = true;
    }
    scn2681_update_status(&s->ch[0]);
    scn2681_update_status(&s->ch[1]);
}

static void scn2681_realize(DeviceState *dev, Error **errp)
{
    SCN2681State *s = SCN2681(dev);
    int i;

    s->ct_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, scn2681_ct_expired, s);

    for (i = 0; i < 2; i++) {
        s->ch[i].duart = s;
        s->ch[i].idx = i;
        qemu_chr_fe_set_handlers(&s->ch[i].chr, scn2681_chr_can_receive,
                                 scn2681_chr_receive, NULL, NULL,
                                 &s->ch[i], NULL, true);
    }
}

static void scn2681_init(Object *obj)
{
    SCN2681State *s = SCN2681(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &scn2681_ops, s,
                          "scn2681", 0x400);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static const Property scn2681_properties[] = {
    DEFINE_PROP_CHR("chardev-a", SCN2681State, ch[0].chr),
    DEFINE_PROP_CHR("chardev-b", SCN2681State, ch[1].chr),
    DEFINE_PROP_UINT8("input-port", SCN2681State, input_port, 0),
    DEFINE_PROP_UINT32("xtal", SCN2681State, xtal, 3686400),
    DEFINE_PROP_BOOL("apollo-autobaud", SCN2681State, apollo_autobaud, false),
    DEFINE_PROP_BOOL("preinit-console", SCN2681State, preinit_console, false),
};

static void scn2681_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = scn2681_realize;
    device_class_set_legacy_reset(dc, scn2681_reset);
    device_class_set_props(dc, scn2681_properties);
    set_bit(DEVICE_CATEGORY_INPUT, dc->categories);
}

static const TypeInfo scn2681_type_info = {
    .name = TYPE_SCN2681,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SCN2681State),
    .instance_init = scn2681_init,
    .class_init = scn2681_class_init,
};

static void scn2681_register_types(void)
{
    type_register_static(&scn2681_type_info);
}

type_init(scn2681_register_types)
