/*
 * Signetics SCN2681 Dual Asynchronous Receiver/Transmitter (DUART)
 *
 * Minimal model: enough for the Apollo DN3000 boot PROM's serial
 * console (channel B) and keyboard channel (channel A), plus the
 * on-chip counter/timer.
 *
 * Register layout follows the SCN2681/MC68681 datasheet; the MMIO
 * region uses 2-byte register spacing (register index = offset >> 1)
 * matching the Apollo DN3000/DN3500 wiring (see MAME apollo_sio and
 * Linux arch/m68k/include/asm/apollohw.h struct SCN2681).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_CHAR_SCN2681_H
#define HW_CHAR_SCN2681_H

#include "hw/core/sysbus.h"
#include "chardev/char-fe.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_SCN2681 "scn2681"
OBJECT_DECLARE_SIMPLE_TYPE(SCN2681State, SCN2681)

#define SCN2681_FIFO_DEPTH 3

typedef struct SCN2681State SCN2681State;

typedef struct SCN2681Channel {
    SCN2681State *duart;
    int idx;                    /* 0 = A, 1 = B */
    CharFrontend chr;
    uint8_t mr[2];
    int mr_ptr;
    uint8_t sr;
    uint8_t csr;
    bool rx_enabled;
    bool tx_enabled;
    uint8_t rx_fifo[SCN2681_FIFO_DEPTH];
    int rx_head;
    int rx_count;
} SCN2681Channel;

struct SCN2681State {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;

    SCN2681Channel ch[2];

    uint8_t acr;
    uint8_t imr;
    uint8_t isr;
    uint8_t ivr;
    uint8_t opcr;
    uint8_t opr;

    /* counter/timer */
    uint16_t ct_preset;
    bool ct_running;
    int64_t ct_start_ns;
    QEMUTimer *ct_timer;

    /* properties */
    uint8_t input_port;         /* IP0-IP6 pin state */
    uint32_t xtal;              /* crystal frequency, Hz */
    bool apollo_autobaud;       /* Apollo MD PROM autobaud quirk */
    bool preinit_console;       /* enable channel B TX/RX at reset */
};

#endif
