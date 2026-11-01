/*
 * QEMU PowerMac CUDA device support
 *
 * Copyright (c) 2004-2007 Fabrice Bellard
 * Copyright (c) 2007 Jocelyn Mayer
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
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "migration/vmstate.h"
#include "hw/misc/macio/cuda.h"
#include "qemu/timer.h"
#include "system/block-backend.h"
#include "system/runstate.h"
#include "system/rtc.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "qobject/qdict.h"
#include "qemu/cutils.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "trace.h"

/* Bits in B data register: all active low */
#define TREQ            0x08    /* Transfer request (input) */
#define TACK            0x10    /* Transfer acknowledge (output) */
#define TIP             0x20    /* Transfer in progress (output) */

/* commands (1st byte) */
#define ADB_PACKET      0
#define CUDA_PACKET     1
#define ERROR_PACKET    2
#define TIMER_PACKET    3
#define POWER_PACKET    4
#define MACIIC_PACKET   5
#define PMU_PACKET      6

#define TYPE_CUDA_I2C_STUB "cuda-i2c-stub"

/* status byte (2nd byte) of an ADB_PACKET reply */
#define ADB_STAT_RESPONSE 0x80
/* Cuda packet status bit 1: the addressed device returned no data. */
#define ADB_STAT_NODATA   0x02

#define CUDA_TIMER_FREQ (4700000 / 6)

/*
 * SR_INT delay, by triggering event -- confirmed against DingusPPC's
 * ViaCuda::write()/schedule_sr_int() call sites, which model these three
 * cases with distinct, specific microsecond values (not a single generic
 * delay): a dummy idle-acknowledge/attention pulse whenever TIP rises
 * (regardless of which of the two TIP-rising sub-cases caused it), and
 * separate per-byte "I've clocked this one in/out" acks that differ
 * depending on transfer direction.
 */
#define CUDA_SR_DELAY_TIP_PULSE     (61 * SCALE_US)
#define CUDA_SR_DELAY_HOST_TO_CUDA  (71 * SCALE_US)
#define CUDA_SR_DELAY_CUDA_TO_HOST  (88 * SCALE_US)
#define CUDA_SR_DELAY_ATTENTION     (30 * SCALE_US)
/*
 * Backstop for the guest-paced TREQ negation (see cuda_ans_access_entry()):
 * the host's own read of port B is what normally releases it, and a
 * host-clock deadline loses that race whenever the host is slow.
 */
#define CUDA_SYNC_TREQ_DELAY        (8 * SCALE_US)
/*
 * Not itself a modeled hardware timing -- just how soon to check again
 * after a one-shot SR_INT pulse was missed (see cuda_set_sr_int()).
 */
#define CUDA_SR_DELAY_RETRY         (20 * SCALE_US)

/* CUDA returns time_t's offset from Jan 1, 1904, not 1970 */
#define RTC_OFFSET                      2082844800

static void cuda_receive_packet_from_host(CUDAState *s,
                                          const uint8_t *data, int len);

/* MacOS uses timer 1 for calibration on startup, so we use
 * the timebase frequency and cuda_get_counter_value() with
 * cuda_get_load_time() to steer MacOS to calculate calibrate its timers
 * correctly for both TCG and KVM (see commit b981289c49 "PPC: Cuda: Use cuda
 * timer to expose tbfreq to guest" for more information) */

static uint64_t cuda_get_counter_value(MOS6522State *s, MOS6522Timer *ti)
{
    MOS6522CUDAState *mcs = container_of(s, MOS6522CUDAState, parent_obj);
    CUDAState *cs = container_of(mcs, CUDAState, mos6522_cuda);

    /* Reverse of the tb calculation algorithm that Mac OS X uses on bootup */
    uint64_t tb_diff = muldiv64(qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL),
                                cs->tb_frequency, NANOSECONDS_PER_SECOND) -
                           ti->load_time;

    return (tb_diff * 0xBF401675E5DULL) / (cs->tb_frequency << 24);
}

static uint64_t cuda_get_load_time(MOS6522State *s, MOS6522Timer *ti)
{
    MOS6522CUDAState *mcs = container_of(s, MOS6522CUDAState, parent_obj);
    CUDAState *cs = container_of(mcs, CUDAState, mos6522_cuda);

    uint64_t load_time = muldiv64(qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL),
                                  cs->tb_frequency, NANOSECONDS_PER_SECOND);
    return load_time;
}

static void cuda_delay_set_sr_int(CUDAState *s, uint64_t delay_ns);
static void cuda_treq_release(void *opaque);

static void cuda_update(CUDAState *s);
static bool cuda_unsolicited_busy(CUDAState *s);
static void cuda_flush_ap(CUDAState *s);
static void cuda_delay_clock_byte(CUDAState *s, uint8_t byte,
                                  uint64_t delay_ns);
static void cuda_set_state(CUDAState *s, CudaTransportState state);

static void cuda_set_treq(CUDAState *s, bool high)
{
    MOS6522State *ms = MOS6522(&s->mos6522_cuda);

    s->treq_high = high;
    mos6522_set_port_input(ms, 1, 3, high);
}

static void cuda_cancel_push(CUDAState *s)
{
    timer_del(s->sr_delay_timer);
    s->sr_pending_byte_valid = false;
    s->sr_pending_needs_delay = false;
}

/*
 * A host command written while an autopoll attention byte is pending
 * takes priority: return the packet to the output queue and let the
 * command byte replace the SR.
 */
static void cuda_yield_autopoll(CUDAState *s)
{
    if (!s->byte_push || s->tx_idx > 1 || s->tx_len < 3) {
        return;
    }
    if (s->tx_buf[1] != ADB_PACKET || !(s->tx_buf[2] & 0x40)) {
        return;
    }
    memcpy(s->ap_park, s->tx_buf + 1, s->tx_len - 1);
    s->ap_park_len = s->tx_len - 1;
    s->tx_len = 0;
    s->tx_idx = 0;
    cuda_cancel_push(s);
    cuda_set_state(s, CUDA_STATE_IDLE);
    cuda_set_treq(s, true);
}

static bool cuda_park_dbg(void)
{
    static int cached = -1;
    static unsigned lines;

    if (cached < 0) {
        cached = getenv("ANS_PARK_DBG") ? 1 : 0;
    }
    return cached &&
           qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) > 600 * NANOSECONDS_PER_SECOND &&
           ++lines <= 3000;
}

/* A response the host has not engaged after a few milliseconds is parked
 * (TREQ released) and re-presented on a later idle pass, as the chip does. */
static void cuda_park_timer(void *opaque)
{
    CUDAState *s = opaque;
    MOS6522State *ms = MOS6522(&s->mos6522_cuda);

    if (cuda_park_dbg() && (s->parked || s->tx_len > 0)) {
        fprintf(stderr, "PARKDBG %8" PRId64 " timer size=%d idx=%d parked=%d b=%02x\n",
                qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) / 1000,
                s->tx_len, s->tx_idx, s->parked, ms->b);
    }
    if (s->parked) {
        /* Re-present the parked response as soon as the bus is quiet, exactly
         * like a real Cuda coming round to it again. */
        if (cuda_unsolicited_busy(s)) {
            timer_mod(s->park_timer,
                      qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + s->park_ns);
            return;
        }
        memcpy(s->tx_buf, s->park_buf, s->park_len);
        s->tx_len = s->park_len;
        s->tx_idx = 0;
        s->parked = false;
        if (cuda_park_dbg()) {
            fprintf(stderr, "PARKDBG %8" PRId64 " unpark len=%d b0=%02x b1=%02x\n",
                    qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) / 1000,
                    s->tx_len, s->tx_buf[0],
                    s->tx_len > 1 ? s->tx_buf[1] : 0);
        }
        s->push_idle_ack_done = false;
        cuda_set_state(s, CUDA_STATE_SENDING);
        cuda_update(s);
        if (s->byte_push && s->tx_len > 0) {
            s->tx_idx = 1;
            cuda_delay_clock_byte(s, s->tx_buf[0], CUDA_SR_DELAY_ATTENTION);
        }
        timer_mod(s->park_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + s->park_ns);
        return;
    }
    if (s->tx_len <= 0 || s->tx_idx > 1) {
        /* Nothing queued, or the host has engaged it (it toggled a byte in):
         * leave it alone. */
        return;
    }
    trace_cuda_delay_set_sr_int();
    /* Park the bytes the host never took and release TREQ. */
    /*
     * Whole queue, attention byte included: the watchdog only parks a
     * response the host has not engaged (tx_idx <= 1), and
     * re-presenting it means clocking the attention byte again.
     */
    s->park_len = s->tx_len;
    memcpy(s->park_buf, s->tx_buf, s->park_len);
    s->parked = true;
    if (cuda_park_dbg()) {
        fprintf(stderr, "PARKDBG %8" PRId64 " park len=%d idx=%d b0=%02x b1=%02x\n",
                qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) / 1000,
                s->park_len, s->tx_idx, s->park_buf[0],
                s->park_len > 1 ? s->park_buf[1] : 0);
    }
    s->tx_len = 0;
    s->tx_idx = 0;
    s->sr_pending_byte_valid = false;
    cuda_set_state(s, CUDA_STATE_IDLE);
    cuda_set_treq(s, true);
    if (s->adb_bus.autopoll_blocked) {
        adb_autopoll_unblock(&s->adb_bus);
    }
    timer_mod(s->park_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + s->park_ns);
    cuda_flush_ap(s);
}

/* TEMPORARY debugging aid (ANS_VIA_DBG=1): Cuda transport events with the
 * same virtual timestamps as the VIA access log in mos6522.c. */
static bool cuda_dbg_enabled(void)
{
    static int cached = -1;

    if (cached < 0) {
        cached = getenv("ANS_VIA_DBG") ? 1 : 0;
    }
    return cached;
}

/* Sync-cycle timings, in ns; overridable for measurement.  The idle-
 * acknowledge byte must land after the host's clear-read of the SR. */
static uint64_t cuda_env_delay_ns(const char *name, uint64_t def_ns)
{
    const char *v = getenv(name);

    return v ? (uint64_t)strtoull(v, NULL, 0) * 1000 : def_ns;
}

static uint64_t cuda_sync_byte_delay_ns(void)
{
    static uint64_t cached;

    if (!cached) {
        cached = cuda_env_delay_ns("ANS_SYNC_BYTE_US", 30 * SCALE_US);
    }
    return cached;
}

static uint64_t cuda_sync_ack_delay_ns(void)
{
    static uint64_t cached;

    if (!cached) {
        cached = cuda_env_delay_ns("ANS_SYNC_ACK_US", 30 * SCALE_US);
    }
    return cached;
}

#define CUDA_DBG(...) do { \
        if (cuda_dbg_enabled()) { \
            fprintf(stderr, "CUDA %8" PRId64 " ", \
                    qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) / 1000); \
            fprintf(stderr, __VA_ARGS__); \
        } \
    } while (0)

static const char *cuda_state_name(CudaTransportState state)
{
    switch (state) {
    case CUDA_STATE_IDLE:
        return "idle";
    case CUDA_STATE_RECEIVING:
        return "receiving";
    case CUDA_STATE_SENDING:
        return "sending";
    case CUDA_STATE_SYNC:
        return "sync";
    }
    return "?";
}

static void cuda_set_state(CUDAState *s, CudaTransportState state)
{
    if (s->state != state) {
        CUDA_DBG("state %s -> %s\n", cuda_state_name(s->state),
                 cuda_state_name(state));
    }
    s->state = state;
}

static void cuda_set_sr_int(void *opaque)
{
    CUDAState *s = opaque;
    MOS6522CUDAState *mcs = &s->mos6522_cuda;
    MOS6522State *ms = MOS6522(mcs);
    qemu_irq irq = qdev_get_gpio_in(DEVICE(ms), SR_INT_BIT);

    CUDA_DBG("set_sr_int b=%02x sr=%02x in=%d/%d out=%d\n", ms->b, ms->sr,
             s->tx_idx, s->tx_len, s->rx_len);
    if (s->sr_pending_byte_valid) {
        s->sr_pending_byte_valid = false;
        mos6522_set_sr_input(ms, s->sr_pending_byte);
    } else {
        qemu_set_irq(irq, 1);
    }

    /*
     * The pulse is one-shot, so a host that polls past it would never drain
     * the response; re-pulse while undrained.  Not on the ANS, whose driver
     * clocks each byte itself: an interrupt with no new byte makes it read a
     * stale SR.
     */
    if (s->tx_idx < s->tx_len && !s->byte_push) {
        cuda_delay_set_sr_int(s, CUDA_SR_DELAY_RETRY);
    }
}

static void cuda_delay_set_sr_int(CUDAState *s, uint64_t delay_ns)
{
    int64_t expire;

    trace_cuda_delay_set_sr_int();

    expire = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + delay_ns;
    s->sr_int_due_ns = expire;
    timer_mod(s->sr_delay_timer, expire);
}

/*
 * Negate TREQ a microcontroller's reaction time after the host released
 * BYTEACK in a sync cycle.  Deliberately *not* synchronous with the store:
 * see CUDA_SYNC_TREQ_DELAY.
 */
static void cuda_treq_release(void *opaque)
{
    CUDAState *s = opaque;

    s->treq_release_pending = false;
    s->treq_release_seen = false;
    cuda_set_treq(s, true);
    CUDA_DBG("treq released (sync)\n");
}

/*
 * Clock one byte from Cuda into the SR, delay_ns from now.  The
 * microcontroller clocks bytes on its own schedule, so a host that takes one
 * byte per SR interrupt must find the byte already in the SR when the
 * interrupt arrives; raising the interrupt alone would leave it reading a
 * stale register.
 */
static void cuda_delay_clock_byte(CUDAState *s, uint8_t byte, uint64_t delay_ns)
{
    s->sr_pending_byte = byte;
    s->sr_pending_byte_valid = true;
    s->sr_pending_needs_delay = false;
    cuda_delay_set_sr_int(s, delay_ns);
}

/*
 * Clock the next byte of the queued response out of the SR (byte_push
 * machines only).  TREQ rises with the last byte, which is the host's "that
 * was the last one" test, and the packet is drained once every byte has been
 * clocked.
 */
static void cuda_clock_next_byte(CUDAState *s)
{
    uint8_t byte;

    if (s->tx_idx >= s->tx_len) {
        return;
    }
    byte = s->tx_buf[s->tx_idx++];
    trace_cuda_data_recv(byte);
    CUDA_DBG("clock_next_byte 0x%02x -> idx %d/%d\n", byte,
             s->tx_idx, s->tx_len);
    cuda_delay_clock_byte(s, byte, CUDA_SR_DELAY_CUDA_TO_HOST);
    if (s->tx_idx >= s->tx_len) {
        cuda_set_treq(s, true);
        adb_autopoll_unblock(&s->adb_bus);
    } else {
        cuda_set_treq(s, false);
    }
}

static void cuda_push_advance(CUDAState *s)
{
    if (s->tx_idx < s->tx_len) {
        cuda_clock_next_byte(s);
    } else if (s->tx_len > 0 && !s->push_idle_ack_done) {
        s->push_idle_ack_done = true;
        cuda_delay_clock_byte(s, 0x00, CUDA_SR_DELAY_TIP_PULSE);
        s->sr_pending_needs_delay = true;
    }
}

/*
 * Deliver a staged SR interrupt on the guest's next VIA access: byte-push
 * hosts poll one byte per interrupt.  Acknowledge bytes keep their delay.
 */
static void cuda_sr_int_catch_up(CUDAState *s)
{
    MOS6522State *ms = MOS6522(&s->mos6522_cuda);

    if (!timer_pending(s->sr_delay_timer)) {
        return;
    }
    /*
     * A byte-push host reads one byte per SR interrupt; this rule makes that
     * handshake host-speed independent.  Other hosts keep the deadline, so
     * a host that reads the SR early sees the register as it was.
     */
    /* A staged byte may only land once the byte already in the SR has been
     * read (IFR.SR still set means unread): a host that reads the attention
     * byte would otherwise be handed the first data byte in its place and
     * never frame the response. */
    if (((s->byte_push && !s->sr_pending_needs_delay) ||
         qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) >= s->sr_int_due_ns) &&
        !(ms->ifr & SR_INT)) {
        timer_del(s->sr_delay_timer);
        cuda_set_sr_int(s);
    }
}

/* The timer path takes the same rule: an unread SR defers the next byte. */
static void cuda_sr_int_deadline(void *opaque)
{
    CUDAState *s = opaque;
    MOS6522State *ms = MOS6522(&s->mos6522_cuda);

    if (ms->ifr & SR_INT) {
        timer_mod(s->sr_delay_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + CUDA_SR_DELAY_RETRY);
        return;
    }
    cuda_set_sr_int(s);
}

/* Clock the sync cycle's pending acknowledge byte, if one is owed. */
static void cuda_ans_sync_ack_deliver(CUDAState *s)
{
    if (!s->sync_ack_pending) {
        return;
    }
    s->sync_ack_pending = false;
    s->sync_ack_after_clear = false;
    s->sr_pending_byte = 0x00;
    s->sr_pending_byte_valid = true;
    s->sr_pending_needs_delay = false;
    cuda_set_sr_int(s);
}

/*
 * The chip clocks the acknowledge whether or not the host still polls, so a
 * host that stops reading the SR mid-cycle must not lose it.
 */
static void cuda_ans_sync_ack_timeout(void *opaque)
{
    cuda_ans_sync_ack_deliver(opaque);
}

/*
 * ans-sync-lazy's acknowledgements -- the TREQ release after the host's
 * BYTEACK store and the idle-acknowledge byte after its SR clear -- are
 * applied at the host's next VIA access, with the timer above as a backstop.
 */
static void cuda_ans_access_entry(CUDAState *s)
{
    /* The host's first port-B read must still find TREQ asserted; the
     * negation lands at the access after it has looked. */
    if (s->treq_release_pending && s->treq_release_seen) {
        timer_del(s->treq_delay_timer);
        s->treq_release_pending = false;
        s->treq_release_seen = false;
        cuda_treq_release(s);
    }
    if (s->sync_ack_pending && s->sync_ack_after_clear) {
        timer_del(s->sync_ack_timer);
        cuda_ans_sync_ack_deliver(s);
    }
    cuda_sr_int_catch_up(s);
}

/* NOTE: TIP and TREQ are negated */
static void cuda_update(CUDAState *s)
{
    MOS6522CUDAState *mcs = &s->mos6522_cuda;
    MOS6522State *ms = MOS6522(mcs);
    ADBBusState *adb_bus = &s->adb_bus;
    int packet_received, len;

    CUDA_DBG("update ENTRY b=%02x last_b=%02x acr=%02x sr=%02x in=%d/%d"
             " out=%d\n", ms->b, s->last_b, ms->acr, ms->sr,
             s->tx_idx, s->tx_len, s->rx_len);
    packet_received = 0;
    if (!(ms->b & TIP)) {
        /* transfer requested from host */

        if (ms->acr & SR_OUT) {
            /* data output */
            if ((ms->b & (TACK | TIP)) != (s->last_b & (TACK | TIP))) {
                if ((s->last_b & TIP) && !(ms->b & TIP)) {
                    cuda_set_state(s, CUDA_STATE_RECEIVING);
                }
                if (s->rx_len < sizeof(s->rx_buf)) {
                    if (s->rx_len == 0) {
                        adb_autopoll_block(adb_bus);
                    }
                    trace_cuda_data_send(ms->sr);
                    s->rx_buf[s->rx_len++] = ms->sr;
                    cuda_delay_set_sr_int(s, CUDA_SR_DELAY_HOST_TO_CUDA);
                }
            }
        } else {
            if (s->byte_push) {
                /*
                 * A reader that waits for an SR interrupt per byte never
                 * toggles BYTEACK first, so bytes are clocked on the
                 * acknowledges: attention byte when the packet is queued,
                 * first data byte when the host takes it, each further byte
                 * after the previous acknowledge.
                 */
                if ((ms->b & (TACK | TIP)) != (s->last_b & (TACK | TIP))) {
                    CUDA_DBG("push-branch toggle b=%02x last_b=%02x idx=%d/%d\n",
                             ms->b, s->last_b, s->tx_idx,
                             s->tx_len);
                    if (ms->ifr & SR_INT) {
                        s->push_waiting_read = true;
                    } else {
                        cuda_push_advance(s);
                    }
                }
            } else if (s->tx_idx < s->tx_len) {
                /* data input */
                if ((ms->b & (TACK | TIP)) != (s->last_b & (TACK | TIP))) {
                    ms->sr = s->tx_buf[s->tx_idx++];
                    trace_cuda_data_recv(ms->sr);
                    /* indicate end of transfer */
                    if (s->tx_idx >= s->tx_len) {
                        cuda_set_treq(s, true);
                        adb_autopoll_unblock(adb_bus);
                        cuda_flush_ap(s);
                    }
                    cuda_delay_set_sr_int(s, CUDA_SR_DELAY_CUDA_TO_HOST);
                }
            }
        }
    } else {
        /* no transfer requested: handle sync case */
        if ((s->last_b & TIP) && (ms->b & TACK) != (s->last_b & TACK)) {
            cuda_set_state(s, CUDA_STATE_SYNC);
            if (s->byte_push) {
            /*
             * CudaInit's sync cycle: BYTEACK falling with TIP high asserts
             * TREQ and clocks the acknowledge byte; BYTEACK rising negates
             * TREQ and clocks the idle acknowledge, delayed so the host's
             * clear-read cannot eat it.
             */
                CUDA_DBG("sync edge: ba=%d b=%02x state=%d\n",
                         !!(ms->b & TACK), ms->b, s->sync_state);
                uint64_t delay = s->sync_state == 1 ? cuda_sync_ack_delay_ns()
                                                    : cuda_sync_byte_delay_ns();

                if (s->tx_idx < s->tx_len) {
                    /*
                     * A sync cycle resets the transport, not the output
                     * queue: a response the host has not engaged yet stays
                     * queued and the abandonment watchdog re-presents it
                     * afterwards.  A partially read response is dropped --
                     * the host that engaged it is gone.
                     */
                    if (s->tx_idx > 1) {
                        s->tx_idx = 0;
                        s->tx_len = 0;
                    }
                    s->push_idle_ack_done = false;
                    s->sr_pending_byte_valid = false;
                }
                if (!(ms->b & TACK)) {
                    cuda_set_treq(s, false);        /* assert TREQ */
                    s->sync_state = 1;
                } else if (s->ans_sync_lazy) {
                    /* Released once the host has taken a look at port B. */
                    s->treq_release_pending = true;
                    s->treq_release_seen = false;
                    s->sync_state = 2;
                } else {
                    /*
                     * BYTEACK released: the host is about to poll TREQ and
                     * must still find it asserted, so the negation waits for
                     * its read of port B (CUDA_SYNC_TREQ_DELAY is only the
                     * backstop for a host that never reads again).
                     */
                    s->treq_release_pending = true;
                    s->treq_release_seen = false;
                    timer_mod(s->treq_delay_timer,
                              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)
                              + CUDA_SYNC_TREQ_DELAY);
                    s->sync_state = 2;
                }
                if (s->ans_sync_lazy) {
                    if (s->sync_state == 1) {
                        /* TREQ just rose: the acknowledge byte is handed over
                         * at the host's next access, i.e. when it polls for
                         * the SR interrupt. */
                        s->sync_ack_pending = true;
                        s->sync_ack_after_clear = true;
                    } else {
                        /* After BYTEACK: clocked once the host has read SR
                         * (see cuda_ans_access_entry), not on a delay. */
                        s->sync_ack_pending = true;
                        s->sync_ack_after_clear = false;
                    }
                    timer_mod(s->sync_ack_timer,
                              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + delay);
                } else {
                    cuda_delay_clock_byte(s, 0x00, delay);
                    /* A sync-cycle acknowledge: the delay is mandatory here. */
                    s->sr_pending_needs_delay = true;
                }
                if (ms->b & TACK) {
                    cuda_set_state(s, CUDA_STATE_IDLE);
                }
                s->last_acr = ms->acr;
                s->last_b = ms->b;
                return;
            }
        /*
         * TREQ follows TACK: asserted on the host's sync/attention strobe,
         * negated at end-of-cycle unless a response is part-read (keep
         * inviting the host to finish it) or untouched by this cycle (it
         * belongs to another exchange).
         */
            if ((ms->b & TACK) &&
                (s->tx_idx == 0 || s->tx_idx >= s->tx_len)) {
                cuda_set_treq(s, true);
            } else {
                cuda_set_treq(s, false);
            }
            cuda_delay_set_sr_int(s, CUDA_SR_DELAY_TIP_PULSE);
            cuda_set_state(s, (ms->b & TACK) ? CUDA_STATE_IDLE
                                             : CUDA_STATE_SYNC);
        } else {
            if (!(s->last_b & TIP)) {
                /* handle end of host to cuda transfer */
                cuda_set_state(s, CUDA_STATE_IDLE);
                packet_received = (s->rx_len > 0);
                s->sync_state = 0;

                /* The host may stop clocking before tx_buf is drained;
                 * drop the remainder or autopoll stays blocked. */
                if (s->tx_idx > 0 && s->tx_idx < s->tx_len) {
                    s->tx_len = s->tx_idx;
                    adb_autopoll_unblock(adb_bus);
                    cuda_flush_ap(s);
                }

                /*
                 * The transaction just ended (TIP rose): Open Firmware's
                 * manager waits for the idle-acknowledge *byte*'s SR
                 * interrupt before it calls the bus idle, so put a real byte
                 * in the shift register, not just an interrupt pulse.
                 */
                if (s->byte_push &&
                    s->tx_idx >= s->tx_len) {
                    cuda_delay_clock_byte(s, 0x00, CUDA_SR_DELAY_TIP_PULSE);
                    s->sr_pending_needs_delay = true;
                } else {
                    cuda_delay_set_sr_int(s, CUDA_SR_DELAY_TIP_PULSE);
                }
            }
            /* signal if there is data to read */
            if (s->tx_idx < s->tx_len) {
                cuda_set_treq(s, false);
            }
        }
    }

    s->last_acr = ms->acr;
    s->last_b = ms->b;

    CUDA_DBG("update b=%02x last_b=%02x acr=%02x sr=%02x in=%d/%d out=%d"
             " last_acr=%02x pkt=%d\n", ms->b, s->last_b, ms->acr, ms->sr,
             s->tx_idx, s->tx_len, s->rx_len,
             s->last_acr, packet_received);

    /* NOTE: cuda_receive_packet_from_host() can call cuda_update()
       recursively */
    if (packet_received) {
        len = s->rx_len;
        s->rx_len = 0;
        cuda_receive_packet_from_host(s, s->rx_buf, len);
    }
}

static void cuda_send_packet_to_host(CUDAState *s,
                                     const uint8_t *data, int len)
{
    int i;
    bool autopoll = len >= 2 && data[0] == ADB_PACKET &&
                    (data[1] & 0x40);
    bool push = s->byte_push && !autopoll;

    trace_cuda_packet_send(len);
    for (i = 0; i < len; i++) {
        trace_cuda_packet_send_data(i, data[i]);
    }

    if (cuda_dbg_enabled()) {
        fprintf(stderr, "CUDA %8" PRId64 " send_pkt len=%d:",
                qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) / 1000, len);
        for (i = 0; i < len; i++) {
            fprintf(stderr, " %02x", data[i]);
        }
        fprintf(stderr, " (byte_push=%d)\n", s->byte_push);
    }

    /* A newer response supersedes one the host never engaged. */
    s->parked = false;
    s->park_len = 0;

    cuda_set_state(s, CUDA_STATE_SENDING);
    if (push) {
        /* [attention][type][flags][command][data...] -- see the header. */
        s->tx_buf[0] = 0x00;
        memcpy(s->tx_buf + 1, data, len);
        len++;
    } else {
        memcpy(s->tx_buf, data, len);
    }
    s->tx_len = len;
    s->tx_idx = 0;
    s->push_idle_ack_done = false;
    s->push_waiting_read = false;
    cuda_update(s);
    if (s->park_ns > 0) {
        timer_mod(s->park_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + s->park_ns);
    }
    if (push && len > 0) {
        /*
         * The attention byte is clocked as Cuda's own first byte; hosts read
         * and discard exactly one byte before the packet.  Handing it over on
         * the host's first BYTEACK toggle instead let a host that samples the
         * SR early fold it into the packet.
         */
        s->tx_idx = 1;
        cuda_delay_clock_byte(s, s->tx_buf[0], CUDA_SR_DELAY_ATTENTION);
    }
    if (!push) {
        /* Announcement pulse for unsolicited (autopoll/one-second) packets,
         * as in DingusPPC's ViaCuda::autopoll_handler(); direct responses are
         * pulsed by cuda_update()'s TIP-transition paths. */
        cuda_delay_set_sr_int(s, CUDA_SR_DELAY_ATTENTION);
    }
}

/*
 * True while the host is inside a byte transfer or a response is not fully
 * read.  Unsolicited packets are emitted only from the idle loop, so they
 * must not overwrite tx_buf under the guest's read; skipping is safe, as
 * deltas accumulate and a missed RTC tick is superseded.
 */
static bool cuda_unsolicited_busy(CUDAState *s)
{
    MOS6522State *ms = MOS6522(&s->mos6522_cuda);

    return s->state != CUDA_STATE_IDLE ||
           s->sr_pending_byte_valid ||
           !(ms->b & TIP) ||
           s->tx_idx < s->tx_len;
}

static void cuda_adb_poll(void *opaque)
{
    CUDAState *s = opaque;
    ADBBusState *adb_bus = &s->adb_bus;
    uint8_t obuf[ADB_MAX_OUT_LEN + 2];
    int olen;

    olen = adb_poll(adb_bus, obuf + 2, adb_bus->autopoll_mask);
    if (olen > 0) {
        if (olen == 2 && (obuf[2] & 0x0f) == 0x0c) {
            uint8_t cmd = obuf[2];
            int n;

            /* Keyboard register 0 is a two-byte packet: a make and its
             * release can be pending together; 0xFF means no second
             * event. */
            adb_autopoll_block(adb_bus);
            n = adb_request(adb_bus, obuf + 4, &cmd, 1);
            adb_autopoll_unblock(adb_bus);
            obuf[4] = n > 0 ? obuf[4] : 0xff;
            olen = 3;
        }
        obuf[0] = ADB_PACKET;
        obuf[1] = 0x40; /* polled data */
        if (cuda_unsolicited_busy(s)) {
            /* The ADB side is independent of the host link; keep the reply
             * in the output queue until the link is free. */
            memcpy(s->ap_park, obuf, olen + 2);
            s->ap_park_len = olen + 2;
            return;
        }
        cuda_send_packet_to_host(s, obuf, olen + 2);
    }
}

static void cuda_flush_ap(CUDAState *s)
{
    uint8_t buf[ADB_MAX_OUT_LEN + 3];
    int len;

    if (s->ap_park_len > 0 && !cuda_unsolicited_busy(s)) {
        len = s->ap_park_len;
        s->ap_park_len = 0;
        memcpy(buf, s->ap_park, len);
        cuda_send_packet_to_host(s, buf, len);
    }
}

/*
 * One-second RTC packet (CUDA_SET_ONE_SECOND_MODE): unsolicited, once per
 * second, independent of ADB autopoll.  Mode 1's full packet is a valid
 * response whatever the guest asked for, so modes 2 and 3 are not modelled.
 */
static void cuda_one_sec_tick(void *opaque)
{
    CUDAState *s = opaque;
    uint8_t obuf[7];
    uint32_t real_time;

    if (s->one_sec_mode == 0) {
        return;
    }

    /*
     * Never preempt an in-progress or unread exchange (see
     * cuda_unsolicited_busy()); this tick's time value is superseded
     * by the next one anyway.
     */
    if (!cuda_unsolicited_busy(s)) {
        if (!s->tick_rdtime) {
            obuf[0] = 0x03; /* bare tick packet */
            cuda_send_packet_to_host(s, obuf, 1);
        } else {
            real_time = s->tick_offset +
                        (qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)
                         / NANOSECONDS_PER_SECOND);
            obuf[0] = CUDA_PACKET;
            obuf[1] = 0;
            obuf[2] = CUDA_GET_TIME;
            obuf[3] = real_time >> 24;
            obuf[4] = real_time >> 16;
            obuf[5] = real_time >> 8;
            obuf[6] = real_time;
            cuda_send_packet_to_host(s, obuf, sizeof(obuf));
        }
    }

    timer_mod(s->one_sec_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)
                                 + NANOSECONDS_PER_SECOND);
}

/* description of commands */
typedef struct CudaCommand {
    uint8_t command;
    const char *name;
    bool (*handler)(CUDAState *s,
                    const uint8_t *in_args, int in_len,
                    uint8_t *out_args, int *out_len);
} CudaCommand;

static bool cuda_cmd_autopoll(CUDAState *s,
                              const uint8_t *in_data, int in_len,
                              uint8_t *out_data, int *out_len)
{
    ADBBusState *adb_bus = &s->adb_bus;
    bool autopoll;

    if (in_len != 1) {
        return false;
    }

    autopoll = (in_data[0] != 0) ? true : false;

    adb_set_autopoll_enabled(adb_bus, autopoll);
    return true;
}

static bool cuda_cmd_set_autorate(CUDAState *s,
                                  const uint8_t *in_data, int in_len,
                                  uint8_t *out_data, int *out_len)
{
    ADBBusState *adb_bus = &s->adb_bus;

    if (in_len != 1) {
        return false;
    }

    /* we don't want a period of 0 ms */
    /* FIXME: check what real hardware does */
    if (in_data[0] == 0) {
        return false;
    }

    adb_set_autopoll_rate_ms(adb_bus, in_data[0]);
    return true;
}

static bool cuda_cmd_set_device_list(CUDAState *s,
                                     const uint8_t *in_data, int in_len,
                                     uint8_t *out_data, int *out_len)
{
    ADBBusState *adb_bus = &s->adb_bus;
    uint16_t mask;

    if (in_len != 2) {
        return false;
    }

    mask = (((uint16_t)in_data[0]) << 8) | in_data[1];

    adb_set_autopoll_mask(adb_bus, mask);
    return true;
}

static bool cuda_cmd_powerdown(CUDAState *s,
                               const uint8_t *in_data, int in_len,
                               uint8_t *out_data, int *out_len)
{
    if (in_len != 0) {
        return false;
    }

    qemu_system_shutdown_request(SHUTDOWN_CAUSE_GUEST_SHUTDOWN);
    return true;
}

static bool cuda_cmd_reset_system(CUDAState *s,
                                  const uint8_t *in_data, int in_len,
                                  uint8_t *out_data, int *out_len)
{
    if (in_len != 0) {
        return false;
    }

    qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
    return true;
}

static bool cuda_cmd_set_file_server_flag(CUDAState *s,
                                          const uint8_t *in_data, int in_len,
                                          uint8_t *out_data, int *out_len)
{
    if (in_len != 1) {
        return false;
    }

    qemu_log_mask(LOG_UNIMP,
                  "CUDA: unimplemented command FILE_SERVER_FLAG %d\n",
                  in_data[0]);
    return true;
}

static bool cuda_cmd_set_power_message(CUDAState *s,
                                       const uint8_t *in_data, int in_len,
                                       uint8_t *out_data, int *out_len)
{
    if (in_len != 1) {
        return false;
    }

    qemu_log_mask(LOG_UNIMP,
                  "CUDA: unimplemented command SET_POWER_MESSAGE %d\n",
                  in_data[0]);
    return true;
}

/*
 * RdDevList: the ADB address bitmap, high byte first.  AIX's cfgcuda builds
 * its keyboard and mouse adapters from the bits, so the machine has to
 * answer with where they actually live; a zero list describes a computer
 * with no keyboard.
 */
static bool cuda_cmd_get_device_list(CUDAState *s,
                                     const uint8_t *in_data, int in_len,
                                     uint8_t *out_data, int *out_len)
{
    uint16_t list = 0;
    int i;

    for (i = 0; i < s->adb_bus.nb_devices; i++) {
        list |= 1 << s->adb_bus.devices[i]->devaddr;
    }
    out_data[0] = list >> 8;
    out_data[1] = list;
    *out_len = 2;
    return true;
}

static bool cuda_cmd_get_time(CUDAState *s,
                              const uint8_t *in_data, int in_len,
                              uint8_t *out_data, int *out_len)
{
    uint32_t ti;

    if (in_len != 0) {
        return false;
    }

    ti = s->tick_offset + (qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)
                           / NANOSECONDS_PER_SECOND);
    out_data[0] = ti >> 24;
    out_data[1] = ti >> 16;
    out_data[2] = ti >> 8;
    out_data[3] = ti;
    *out_len = 4;
    return true;
}

static bool cuda_cmd_set_time(CUDAState *s,
                              const uint8_t *in_data, int in_len,
                              uint8_t *out_data, int *out_len)
{
    uint32_t ti;

    if (in_len != 4) {
        return false;
    }

    ti = (((uint32_t)in_data[0]) << 24) + (((uint32_t)in_data[1]) << 16)
         + (((uint32_t)in_data[2]) << 8) + in_data[3];
    s->tick_offset = ti - (qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)
                           / NANOSECONDS_PER_SECOND);
    return true;
}

/*
 * Real hardware never reads more than a handful of I2C bytes back in a
 * single CUDA pseudo-command (drivers loop, one subaddress per call, for
 * bulk reads like SPD). Cap our synchronous reply to a small fixed size;
 * the guest simply stops clocking once it has what it wants.
 */
#define CUDA_IIC_MAX_LEN 8

static bool cuda_cmd_get_set_iic(CUDAState *s,
                                 const uint8_t *in_data, int in_len,
                                 uint8_t *out_data, int *out_len)
{
    uint8_t addr;
    bool is_recv;
    int i;

    if (in_len < 1) {
        return false;
    }

    addr = in_data[0] >> 1;
    is_recv = in_data[0] & 1;

    if (i2c_start_transfer(s->i2c_bus, addr, is_recv)) {
        trace_cuda_iic_nack(addr, is_recv);
        s->iic_nack = true;
        return false;
    }

    if (is_recv) {
        for (i = 0; i < CUDA_IIC_MAX_LEN; i++) {
            out_data[i] = i2c_recv(s->i2c_bus);
        }
        *out_len = CUDA_IIC_MAX_LEN;
    } else {
        for (i = 1; i < in_len; i++) {
            if (i2c_send(s->i2c_bus, in_data[i])) {
                i2c_end_transfer(s->i2c_bus);
                trace_cuda_iic_nack(addr, is_recv);
                s->iic_nack = true;
                return false;
            }
        }
        *out_len = 0;
    }

    i2c_end_transfer(s->i2c_bus);
    trace_cuda_iic(addr, is_recv, is_recv ? *out_len : in_len - 1,
                   is_recv ? out_data[0] : (in_len > 1 ? in_data[1] : 0),
                   is_recv ? out_data[1] : (in_len > 2 ? in_data[2] : 0),
                   is_recv ? out_data[2] : (in_len > 3 ? in_data[3] : 0));
    return true;
}

static bool cuda_cmd_combined_format_iic(CUDAState *s,
                                         const uint8_t *in_data, int in_len,
                                         uint8_t *out_data, int *out_len)
{
    uint8_t addr, addr1, sub_addr;
    bool is_recv;
    int i;

    if (in_len < 3) {
        return false;
    }

    addr = in_data[0] >> 1;
    sub_addr = in_data[1];
    addr1 = in_data[2] >> 1;
    is_recv = in_data[2] & 1;

    if ((in_data[0] & 0xfe) != (in_data[2] & 0xfe)) {
        return false;
    }

    if (i2c_start_send(s->i2c_bus, addr)) {
        trace_cuda_iic_nack(addr, is_recv);
        s->iic_nack = true;
        return false;
    }
    if (i2c_send(s->i2c_bus, sub_addr)) {
        i2c_end_transfer(s->i2c_bus);
        trace_cuda_iic_nack(addr, is_recv);
        s->iic_nack = true;
        return false;
    }

    if (is_recv) {
        /*
         * Repeated start: do NOT end the transfer here. QEMU's I2C core
         * treats a start_transfer while devices are already selected as a
         * repeated start (see i2c_do_start_transfer()), which is exactly
         * the SMBus "write subaddress, restart, read" protocol real SPD
         * EEPROM drivers use.
         */
        if (i2c_start_recv(s->i2c_bus, addr1)) {
            i2c_end_transfer(s->i2c_bus);
            trace_cuda_iic_nack(addr1, is_recv);
            s->iic_nack = true;
            return false;
        }
        for (i = 0; i < CUDA_IIC_MAX_LEN; i++) {
            out_data[i] = i2c_recv(s->i2c_bus);
        }
        *out_len = CUDA_IIC_MAX_LEN;
    } else {
        for (i = 3; i < in_len; i++) {
            if (i2c_send(s->i2c_bus, in_data[i])) {
                i2c_end_transfer(s->i2c_bus);
                s->iic_nack = true;
                return false;
            }
        }
        *out_len = 0;
    }

    i2c_end_transfer(s->i2c_bus);
    trace_cuda_iic_combined(addr, sub_addr, is_recv,
                            is_recv ? *out_len : in_len - 3,
                            is_recv ? out_data[0] : (in_len > 3 ? in_data[3] : 0),
                            is_recv ? out_data[1] : (in_len > 4 ? in_data[4] : 0));
    return true;
}

/*
 * Real hardware wires one CUDA VIA port-B bit (PB0) out to a board-specific
 * general-purpose line; nothing on a Beige G3 desktop reads it back. Real
 * firmware issues this as a fire-and-forget pseudo-command expecting a
 * trivial ack -- matches DingusPPC's ViaCuda handling of the same command,
 * which just logs and acks. Without an ack, the caller gets an ERROR_PACKET
 * it isn't expecting instead.
 */
static bool cuda_cmd_out_pb0(CUDAState *s,
                             const uint8_t *in_data, int in_len,
                             uint8_t *out_data, int *out_len)
{
    if (in_len != 1) {
        return false;
    }

    *out_len = 0;
    return true;
}

/*
 * Enables/disables the periodic real-time-clock "tick" packet handled by
 * cuda_one_sec_tick() above. Real classic Mac OS issues this during boot
 * (confirmed via -d guest_errors on this exact ROM: "CUDA: unknown
 * command 0x1b" fired before this was implemented) and DingusPPC's own
 * ViaCuda handles it the same minimal way -- store the mode, ack, and
 * let the timer pick it up.
 */
static bool cuda_cmd_set_one_second_mode(CUDAState *s,
                                         const uint8_t *in_data, int in_len,
                                         uint8_t *out_data, int *out_len)
{
    if (in_len != 1) {
        return false;
    }

    s->one_sec_mode = in_data[0];
    if (s->one_sec_mode != 0) {
        timer_mod(s->one_sec_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)
                                     + NANOSECONDS_PER_SECOND);
    } else {
        timer_del(s->one_sec_timer);
    }

    *out_len = 0;
    return true;
}

/*
 * CUDA_TIMER_TICKLE (0x24): real firmware/OS use is undocumented and
 * DingusPPC's own ViaCuda logs it as an "unsupported pseudo command"
 * too -- but it still sends a trivial ack rather than rejecting it.
 * Before this, our lack of any handler meant an ERROR_PACKET response,
 * and this exact ROM was observed retrying the command 19 times in a
 * row waiting for a real acknowledgement it never got.
 */
static bool cuda_cmd_timer_tickle(CUDAState *s,
                                  const uint8_t *in_data, int in_len,
                                  uint8_t *out_data, int *out_len)
{
    if (in_len != 1) {
        return false;
    }

    *out_len = 0;
    return true;
}

/*
 * CUDA_READ_MCU_MEM / CUDA_WRITE_MCU_MEM (a.k.a GET/SET_6805_ADDR):
 * real hardware exposes the Cuda MCU's own internal memory (including a
 * second path into PRAM) through these; DingusPPC implements the PRAM
 * aliasing and a small fake ROM-header reply for other addresses. We
 * only need the trivial ack here -- this exact ROM was observed issuing
 * these but only to probe/initialize, not to stream real data through
 * this path (PRAM is already reachable via CUDA_GET_PRAM/SET_PRAM).
 */
static bool cuda_cmd_read_mcu_mem(CUDAState *s,
                                  const uint8_t *in_data, int in_len,
                                  uint8_t *out_data, int *out_len)
{
    if (in_len != 2) {
        return false;
    }

    *out_len = 0;
    return true;
}

static bool cuda_cmd_write_mcu_mem(CUDAState *s,
                                   const uint8_t *in_data, int in_len,
                                   uint8_t *out_data, int *out_len)
{
    if (in_len < 2) {
        return false;
    }

    *out_len = 0;
    return true;
}

/*
 * Extended Parameter RAM access. Real Old World ROMs read/write PRAM
 * through the Cuda chip rather than a memory-mapped register: the
 * request carries a 2-byte big-endian offset, and GET_PRAM replies with
 * the single byte at that offset while SET_PRAM takes a 3rd byte to
 * store there.
 */
static bool cuda_cmd_get_pram(CUDAState *s,
                              const uint8_t *in_data, int in_len,
                              uint8_t *out_data, int *out_len)
{
    uint16_t addr;
    int i;

    if (in_len != 2) {
        return false;
    }

    /*
     * RdPram is open-ended: Cuda streams PRAM from the address until the host
     * ends the transaction, so the reply is the rest of the page and each
     * host takes as many bytes as it asked for.  AIX's Cuda transport reads
     * it that way (cuda_read_open_ended_data) and hangs on a one-byte reply.
     */
    addr = (((uint16_t)in_data[0]) << 8) | in_data[1];
    for (i = 0; i < sizeof(s->pram); i++) {
        out_data[i] = s->pram[(addr + i) & (sizeof(s->pram) - 1)];
    }
    *out_len = sizeof(s->pram);
    return true;
}

/*
 * Auto-manage a default PRAM backing file, "pram.img", when the user
 * hasn't attached one explicitly via -global cuda.drive=... -- mirrors
 * mac_oldworld.c's mac_oldworld_default_nvram_blk() for "nvram.img"
 * exactly (same auto-create-if-missing, auto-size-up semantics), so
 * PRAM-resident guest settings persist across runs by default just
 * like real hardware's battery-backed PRAM, without the user needing
 * to pre-create an empty file by hand.
 */
static BlockBackend *cuda_default_pram_blk(void)
{
    static const char filename[] = "pram.img";
    struct stat st;
    BlockBackend *blk;
    Error *local_err = NULL;
    int fd;

    fd = qemu_open_old(filename, O_RDWR | O_CREAT, 0644);
    if (fd < 0) {
        warn_report("could not open/create default PRAM image '%s': %s",
                    filename, strerror(errno));
        return NULL;
    }
    if (fstat(fd, &st) == 0 && st.st_size < (int64_t)sizeof(((CUDAState *)0)->pram)) {
        if (ftruncate(fd, sizeof(((CUDAState *)0)->pram)) != 0) {
            warn_report("could not size default PRAM image '%s': %s",
                        filename, strerror(errno));
        }
    }
    close(fd);

    QDict *options = qdict_new();
    qdict_put_str(options, "driver", "raw");
    blk = blk_new_open(filename, NULL, options, BDRV_O_RDWR, &local_err);
    if (!blk) {
        warn_report_err(local_err);
        return NULL;
    }
    return blk;
}

/* Write PRAM back out to the optional backing file (see the "drive"
 * property), matching q800's mac_via pram_update() -- without this,
 * PRAM-resident settings (e.g. Memory control panel's virtual memory
 * on/off flag) reset to zero on every fresh QEMU invocation. */
static void cuda_pram_update(CUDAState *s)
{
    if (s->pram_blk) {
        if (blk_pwrite(s->pram_blk, 0, sizeof(s->pram), s->pram, 0) < 0) {
            qemu_log("cuda_pram_update: cannot write to file\n");
        }
    }
}

static bool cuda_cmd_set_pram(CUDAState *s,
                              const uint8_t *in_data, int in_len,
                              uint8_t *out_data, int *out_len)
{
    uint16_t addr;

    if (in_len != 3) {
        return false;
    }

    addr = (((uint16_t)in_data[0]) << 8) | in_data[1];
    s->pram[addr % sizeof(s->pram)] = in_data[2];
    cuda_pram_update(s);
    *out_len = 0;
    return true;
}

static const CudaCommand handlers[] = {
    { CUDA_AUTOPOLL, "AUTOPOLL", cuda_cmd_autopoll },
    { CUDA_SET_AUTO_RATE, "SET_AUTO_RATE",  cuda_cmd_set_autorate },
    { CUDA_SET_DEVICE_LIST, "SET_DEVICE_LIST", cuda_cmd_set_device_list },
    { CUDA_POWERDOWN, "POWERDOWN", cuda_cmd_powerdown },
    { CUDA_RESET_SYSTEM, "RESET_SYSTEM", cuda_cmd_reset_system },
    { CUDA_FILE_SERVER_FLAG, "FILE_SERVER_FLAG",
      cuda_cmd_set_file_server_flag },
    { CUDA_SET_POWER_MESSAGES, "SET_POWER_MESSAGES",
      cuda_cmd_set_power_message },
    { CUDA_GET_DEVICE_LIST, "GET_DEVICE_LIST", cuda_cmd_get_device_list },
    { CUDA_GET_TIME, "GET_TIME", cuda_cmd_get_time },
    { CUDA_SET_TIME, "SET_TIME", cuda_cmd_set_time },
    { CUDA_GET_PRAM, "GET_PRAM", cuda_cmd_get_pram },
    { CUDA_SET_PRAM, "SET_PRAM", cuda_cmd_set_pram },
    { CUDA_GET_SET_IIC, "GET_SET_IIC", cuda_cmd_get_set_iic },
    { CUDA_COMBINED_FORMAT_IIC, "COMBINED_FORMAT_IIC",
      cuda_cmd_combined_format_iic },
    { CUDA_OUT_PB0, "OUT_PB0", cuda_cmd_out_pb0 },
    { CUDA_SET_ONE_SECOND_MODE, "SET_ONE_SECOND_MODE",
      cuda_cmd_set_one_second_mode },
    { CUDA_TIMER_TICKLE, "TIMER_TICKLE", cuda_cmd_timer_tickle },
    { CUDA_GET_6805_ADDR, "READ_MCU_MEM", cuda_cmd_read_mcu_mem },
    { CUDA_SET_6805_ADDR, "WRITE_MCU_MEM", cuda_cmd_write_mcu_mem },
};

/* Cuda 2.37's dispatch rejects these slots instead of acknowledging them. */
#define CUDA_MAX_PSEUDO 0x26
static const uint8_t cuda_rejected_cmds[] = {
    0x04, 0x05, 0x06, 0x0f, 0x15, 0x17, 0x18,
    0x1c, 0x1d, 0x1e, 0x1f, 0x20,
};

static bool cuda_cmd_rejected(uint8_t cmd)
{
    int i;

    if (cmd > CUDA_MAX_PSEUDO) {
        return true;
    }
    for (i = 0; i < ARRAY_SIZE(cuda_rejected_cmds); i++) {
        if (cuda_rejected_cmds[i] == cmd) {
            return true;
        }
    }
    return false;
}

static void cuda_receive_packet(CUDAState *s,
                                const uint8_t *data, int len)
{
    uint8_t obuf[3 + 256] = { CUDA_PACKET, 0, data[0] };
    int i, out_len = 0;

    s->iic_nack = false;
    if (cuda_cmd_rejected(data[0])) {
        obuf[0] = ERROR_PACKET;
        obuf[1] = 0x2;
        obuf[2] = CUDA_PACKET;
        obuf[3] = data[0];
        cuda_send_packet_to_host(s, obuf, 4);
        return;
    }

    for (i = 0; i < ARRAY_SIZE(handlers); i++) {
        const CudaCommand *desc = &handlers[i];
        if (desc->command == data[0]) {
            trace_cuda_receive_packet_cmd(desc->name);
            out_len = 0;
            if (desc->handler(s, data + 1, len - 1, obuf + 3, &out_len)) {
                cuda_send_packet_to_host(s, obuf, 3 + out_len);
            } else {
                qemu_log_mask(LOG_GUEST_ERROR,
                              "CUDA: %s: wrong parameters %d\n",
                              desc->name, len);
                obuf[0] = ERROR_PACKET;
                obuf[1] = s->iic_nack ? 0x2 : 0x5;
                obuf[2] = CUDA_PACKET;
                obuf[3] = data[0];
                cuda_send_packet_to_host(s, obuf, 4);
            }
            return;
        }
    }

    /* Accepted slots with no modelled action get a header-only reply. */
    cuda_send_packet_to_host(s, obuf, 3);
}

static void cuda_receive_packet_from_host(CUDAState *s,
                                          const uint8_t *data, int len)
{
    int i;

    trace_cuda_packet_receive(len);
    for (i = 0; i < len; i++) {
        trace_cuda_packet_receive_data(i, data[i]);
    }
    if (cuda_dbg_enabled()) {
        fprintf(stderr, "CUDA %8" PRId64 " recv_pkt len=%d:",
                qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) / 1000, len);
        for (i = 0; i < len && i < 8; i++) {
            fprintf(stderr, " %02x", data[i]);
        }
        fprintf(stderr, "\n");
    }

    switch(data[0]) {
    case ADB_PACKET:
        {
            uint8_t obuf[ADB_MAX_OUT_LEN + 3];
            int olen;
            /*
             * The ADB layer requires autopoll held while it is being asked;
             * the host starting a packet normally takes it, but a response
             * drained in between releases it again.
             */
            adb_autopoll_block(&s->adb_bus);
            olen = adb_request(&s->adb_bus, obuf + 3, data + 1, len - 1);
            if (olen == 1 && (data[1] & 0x0f) == 0x0c) {
                /* Keyboard register 0 is a two-byte packet: a make and its
                 * release can be pending together; 0xFF means no second
                 * event. */
                olen = adb_request(&s->adb_bus, obuf + 4, data + 1,
                                   len - 1);
                obuf[4] = olen > 0 ? obuf[4] : 0xff;
                olen = 2;
            }
            obuf[0] = ADB_PACKET;
            /* 0x80 marks a command response the host must read back
             * (DingusPPC's ViaCuda::process_adb_command always sets it). */
            if (olen > 0) {
                obuf[1] = ADB_STAT_RESPONSE;
            } else if (olen == 0) {
                obuf[1] = ADB_STAT_RESPONSE |
                          (((data[1] & 0x0c) == 0x0c) ?
                           ADB_STAT_NODATA : 0x00);
            } else {
                obuf[1] = ADB_STAT_RESPONSE | -olen;
                olen = 0;
            }
            obuf[2] = data[1];
            cuda_send_packet_to_host(s, obuf, olen + 3);
        }
        break;
    case CUDA_PACKET:
        cuda_receive_packet(s, data + 1, len - 1);
        break;
    }
}

static uint64_t mos6522_cuda_read(void *opaque, hwaddr addr, unsigned size)
{
    CUDAState *s = opaque;
    MOS6522CUDAState *mcs = &s->mos6522_cuda;
    MOS6522State *ms = MOS6522(mcs);

    addr = (addr >> 9) & 0xf;
    cuda_ans_access_entry(s);
    if (addr == 0x0 && s->treq_release_pending) {
        /* The host has looked at TREQ; the negation lands next access. */
        s->treq_release_seen = true;
    }
    if (addr == 0xa && s->sync_ack_pending) {
        /* The host's SR read clears the interrupt; the acknowledge byte is
         * clocked at the access after it. */
        s->sync_ack_after_clear = true;
    }
    {
        uint64_t val = mos6522_read(ms, addr, size);

        if (addr == 0xa && s->byte_push && s->push_waiting_read) {
            s->push_waiting_read = false;
            cuda_push_advance(s);
        }
        return val;
    }
}

static void mos6522_cuda_write(void *opaque, hwaddr addr, uint64_t val,
                               unsigned size)
{
    CUDAState *s = opaque;
    MOS6522CUDAState *mcs = &s->mos6522_cuda;
    MOS6522State *ms = MOS6522(mcs);

    addr = (addr >> 9) & 0xf;
    cuda_ans_access_entry(s);
    if (addr == 0xa) {
        cuda_yield_autopoll(s);
        cuda_cancel_push(s);
        if (s->state != CUDA_STATE_SYNC) {
            cuda_set_state(s, CUDA_STATE_RECEIVING);
        }
    }
    mos6522_write(ms, addr, val, size);
}

static const MemoryRegionOps mos6522_cuda_ops = {
    .read = mos6522_cuda_read,
    .write = mos6522_cuda_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 1,
    },
};

static const VMStateDescription vmstate_cuda = {
    .name = "cuda",
    .version_id = 11,
    .minimum_version_id = 11,
    .fields = (const VMStateField[]) {
        VMSTATE_STRUCT(mos6522_cuda.parent_obj, CUDAState, 0, vmstate_mos6522,
                       MOS6522State),
        VMSTATE_UINT8(last_b, CUDAState),
        VMSTATE_UINT8(last_acr, CUDAState),
        VMSTATE_INT32(tx_len, CUDAState),
        VMSTATE_INT32(tx_idx, CUDAState),
        VMSTATE_INT32(rx_len, CUDAState),
        VMSTATE_BUFFER(tx_buf, CUDAState),
        VMSTATE_BUFFER(rx_buf, CUDAState),
        VMSTATE_UINT32(tick_offset, CUDAState),
        VMSTATE_TIMER_PTR(sr_delay_timer, CUDAState),
        VMSTATE_BUFFER(pram, CUDAState),
        VMSTATE_TIMER_PTR(one_sec_timer, CUDAState),
        VMSTATE_UINT8(one_sec_mode, CUDAState),
        VMSTATE_BOOL_V(treq_high, CUDAState, 9),
        VMSTATE_UINT8_V(state, CUDAState, 10),
        VMSTATE_END_OF_LIST()
    }
};

static void cuda_reset(DeviceState *dev)
{
    CUDAState *s = CUDA(dev);
    ADBBusState *adb_bus = &s->adb_bus;

    s->tx_len = 0;
    s->tx_idx = 0;
    s->rx_len = 0;
    s->state = CUDA_STATE_IDLE;
    s->push_waiting_read = false;
    s->iic_nack = false;
    s->sync_state = 0;
    /* Finish a reaction the chip owed before the reset: leaving TREQ asserted
     * through it wedges the next handshake. */
    if (s->treq_release_pending) {
        s->treq_release_pending = false;
        cuda_treq_release(s);
    }
    s->sync_ack_pending = false;
    s->sync_ack_after_clear = false;
    s->sr_pending_byte_valid = false;
    /*
     * A handshake event armed before the reset would land in the middle of
     * the next one: cancel the pending deliveries and drop a response the
     * abandonment watchdog still holds (the host that was waiting is gone).
     */
    timer_del(s->sr_delay_timer);
    timer_del(s->treq_delay_timer);
    timer_del(s->sync_ack_timer);
    timer_del(s->park_timer);
    s->sr_pending_needs_delay = false;
    s->push_idle_ack_done = false;
    s->parked = false;
    s->park_len = 0;
    s->ap_park_len = 0;

    s->one_sec_mode = 0;
    timer_del(s->one_sec_timer);

    adb_set_autopoll_enabled(adb_bus, false);
}

static void cuda_realize(DeviceState *dev, Error **errp)
{
    CUDAState *s = CUDA(dev);
    SysBusDevice *sbd;
    ADBBusState *adb_bus = &s->adb_bus;
    struct tm tm;

    if (!sysbus_realize(SYS_BUS_DEVICE(&s->mos6522_cuda), errp)) {
        return;
    }

    /* Pass IRQ from 6522 */
    sbd = SYS_BUS_DEVICE(s);
    sysbus_pass_irq(sbd, SYS_BUS_DEVICE(&s->mos6522_cuda));

    qemu_get_timedate(&tm, 0);
    s->tick_offset = (uint32_t)mktimegm(&tm) + RTC_OFFSET;

    s->sr_delay_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, cuda_sr_int_deadline,
                                     s);
    s->treq_delay_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, cuda_treq_release, s);
    s->sync_ack_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                     cuda_ans_sync_ack_timeout, s);
    s->one_sec_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, cuda_one_sec_tick, s);
    /*
     * `byte-push` clocks each response byte, including the attention and
     * idle-acknowledge bytes, and makes the staged byte visible on the
     * guest's next VIA access.  ADB reply framing and the abandonment
     * watchdog below are chip behaviour and are not gated.
     */
    s->park_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, cuda_park_timer, s);
    s->park_ns = getenv("CUDA_PARK_MS")
        ? (int64_t)strtoull(getenv("CUDA_PARK_MS"), NULL, 0) * 1000000
        : 20 * 1000000;
    if (cuda_dbg_enabled() || getenv("CUDA_PARK_MS")) {
        fprintf(stderr, "cuda: abandonment watchdog = %" PRId64 " ns\n",
                s->park_ns);
    }

    adb_register_autopoll_callback(adb_bus, cuda_adb_poll, s);

    if (s->i2c_stub) {
        i2c_slave_create_simple(s->i2c_bus, TYPE_CUDA_I2C_STUB, 0x44);
    }

    if (!s->pram_blk) {
        s->pram_blk = cuda_default_pram_blk();
    }

    if (s->pram_blk) {
        int64_t len = blk_getlength(s->pram_blk);
        int ret;

        if (len < 0) {
            error_setg_errno(errp, -len,
                             "could not get length of PRAM backing image");
            return;
        }
        ret = blk_set_perm(s->pram_blk,
                           BLK_PERM_CONSISTENT_READ | BLK_PERM_WRITE,
                           BLK_PERM_ALL, errp);
        if (ret < 0) {
            return;
        }

        ret = blk_pread(s->pram_blk, 0, sizeof(s->pram), s->pram, 0);
        if (ret < 0) {
            error_setg(errp, "can't read PRAM contents");
            return;
        }
    }
}

static void cuda_init(Object *obj)
{
    CUDAState *s = CUDA(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    object_initialize_child(obj, "mos6522-cuda", &s->mos6522_cuda,
                            TYPE_MOS6522_CUDA);

    memory_region_init_io(&s->mem, obj, &mos6522_cuda_ops, s, "cuda", 0x2000);
    sysbus_init_mmio(sbd, &s->mem);

    qbus_init(&s->adb_bus, sizeof(s->adb_bus), TYPE_ADB_BUS,
              DEVICE(obj), "adb.0");

    s->i2c_bus = i2c_init_bus(DEVICE(obj), "i2c");
}

static const Property cuda_properties[] = {
    DEFINE_PROP_UINT64("timebase-frequency", CUDAState, tb_frequency, 0),
    DEFINE_PROP_DRIVE("drive", CUDAState, pram_blk),
    /* Cuda clocks response bytes itself; off is the legacy toggle path. */
    DEFINE_PROP_BOOL("byte-push", CUDAState, byte_push, true),
    /* RdTime form for the one-second tick; off is the bare Tick packet. */
    DEFINE_PROP_BOOL("tick-rdtime", CUDAState, tick_rdtime, true),
    /* ACK-only I2C target at 0x44 (the ANS video/clock probe) */
    DEFINE_PROP_BOOL("i2c-stub", CUDAState, i2c_stub, true),
    /* apply the sync acknowledgements at the next VIA access (off = on a delay) */
    DEFINE_PROP_BOOL("ans-sync-lazy", CUDAState, ans_sync_lazy, false),
};

static void cuda_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = cuda_realize;
    device_class_set_legacy_reset(dc, cuda_reset);
    dc->vmsd = &vmstate_cuda;
    device_class_set_props(dc, cuda_properties);
    set_bit(DEVICE_CATEGORY_BRIDGE, dc->categories);
}

static const TypeInfo cuda_type_info = {
    .name = TYPE_CUDA,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(CUDAState),
    .instance_init = cuda_init,
    .class_init = cuda_class_init,
};

static void mos6522_cuda_portB_write(MOS6522State *s)
{
    MOS6522CUDAState *mcs = container_of(s, MOS6522CUDAState, parent_obj);
    CUDAState *cs = container_of(mcs, CUDAState, mos6522_cuda);

    CUDA_DBG("portB_write b=%02x ddrb=%02x\n", s->b, s->dirb);
    cuda_update(cs);
}

static void mos6522_cuda_reset_hold(Object *obj, ResetType type)
{
    MOS6522State *ms = MOS6522(obj);
    MOS6522CUDAState *mcs = MOS6522_CUDA(obj);
    CUDAState *cs = container_of(mcs, CUDAState, mos6522_cuda);
    MOS6522DeviceClass *mdc = MOS6522_GET_CLASS(ms);

    if (mdc->parent_phases.hold) {
        mdc->parent_phases.hold(obj, type);
    }

    /*
     * Both VIA timers share a single real hardware clock -- there is only
     * one Phi2 input feeding the whole 6522, confirmed against DingusPPC's
     * ViaCuda (a single VIA_CLOCK_HZ = 783360, used unscaled for both
     * timers; our own CUDA_TIMER_FREQ = 4700000/6 = 783333.33 is the same
     * real value modulo rounding). Timer2 here previously used a wildly
     * different, much smaller rate ((SCALE_US*6000)/4700 = ~1276.6, over
     * 600x slower) dating back to a 2015 MacOS-9-hang fix (commit
     * a53cfdcca2) -- whatever that value compensated for, it leaves Timer2
     * decrementing far slower than real hardware. This ROM's own boot-time
     * hardware-timing-calibration loop (reads Timer2's live counter twice,
     * subtracts, compares against a threshold to measure real elapsed
     * time -- see project notes on the 0xffc34766-347a2 loop) depends on
     * Timer2 actually ticking at the real VIA rate; at 1/600th speed the
     * loop's exit condition is essentially never satisfied.
     */
    ms->timers[0].frequency = CUDA_TIMER_FREQ;
    ms->timers[1].frequency = CUDA_TIMER_FREQ;

    /*
     * Generic MOS6522 reset zeroes port B, which leaves TREQ (active low)
     * asserted. Real hardware powers up with TREQ negated (nothing
     * pending) -- confirmed against DingusPPC's ViaCuda::cuda_init(),
     * which explicitly sets treq=1 at reset. Without this, real Beige G3
     * ROM's early ADB bus reset/enumeration code -- which watches TREQ
     * going idle as its loop-exit condition -- waits forever, since
     * nothing else in the idle-state TACK/TREQ handling corrects a
     * bogus initial assertion that was never a real transfer.
     */
    cuda_set_treq(cs, true);
}

static void mos6522_cuda_class_init(ObjectClass *oc, const void *data)
{
    ResettableClass *rc = RESETTABLE_CLASS(oc);
    MOS6522DeviceClass *mdc = MOS6522_CLASS(oc);

    resettable_class_set_parent_phases(rc, NULL, mos6522_cuda_reset_hold,
                                       NULL, &mdc->parent_phases);
    mdc->portB_write = mos6522_cuda_portB_write;
    /*
     * Mac OS only uses Timer 1 for its own PowerPC-timebase calibration
     * on startup (see the comment on cuda_get_counter_value() above) --
     * these overrides return load_time/counter_value scaled into
     * PowerPC-timebase-tick units instead of plain nanoseconds, which is
     * only correct for that specific Timer 1 calibration use. Timer 2 is
     * used generically for ordinary real-time-based interrupt scheduling
     * (ADB protocol delays, misc timeouts) by hw/misc/mos6522.c's shared
     * get_next_irq_time()/mos6522_read(), which always treats
     * ti->load_time as plain nanoseconds -- wiring Timer 2 to these same
     * timebase-scaled overrides (as this code previously did) corrupts
     * every such computation by a factor of NANOSECONDS_PER_SECOND /
     * tb_frequency (~59.87x at this board's real 16,705,000 Hz
     * timebase), so ti->load_time (in tick units) would be subtracted from
     * qemu_clock_get_ns() (in ns), producing
     * wildly wrong elapsed-time and next-fire computations and a
     * genuine, confirmed livelock in this ROM's VIA-Timer2-driven ADB/
     * boot code. Leaving Timer 2 on the base class's plain, correct,
     * nanosecond-based defaults (MOS6522_CLASS's own class_init already
     * sets both timers to those; simply not overriding Timer 2 here is
     * sufficient) fixes this without touching Timer 1's own, genuinely
     * correct and still-needed calibration behavior.
     */
    mdc->get_timer1_counter_value = cuda_get_counter_value;
    mdc->get_timer1_load_time = cuda_get_load_time;
}

static const TypeInfo mos6522_cuda_type_info = {
    .name = TYPE_MOS6522_CUDA,
    .parent = TYPE_MOS6522,
    .instance_size = sizeof(MOS6522CUDAState),
    .class_init = mos6522_cuda_class_init,
};

static int cuda_i2c_stub_event(I2CSlave *s, enum i2c_event event)
{
    return 0;
}

static uint8_t cuda_i2c_stub_recv(I2CSlave *s)
{
    return 0;
}

static int cuda_i2c_stub_send(I2CSlave *s, uint8_t data)
{
    return 0;
}

static void cuda_i2c_stub_class_init(ObjectClass *oc, const void *data)
{
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(oc);

    sc->event = cuda_i2c_stub_event;
    sc->recv = cuda_i2c_stub_recv;
    sc->send = cuda_i2c_stub_send;
}

static const TypeInfo cuda_i2c_stub_type_info = {
    .name = TYPE_CUDA_I2C_STUB,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(I2CSlave),
    .class_init = cuda_i2c_stub_class_init,
};

static void cuda_register_types(void)
{
    type_register_static(&mos6522_cuda_type_info);
    type_register_static(&cuda_type_info);
    type_register_static(&cuda_i2c_stub_type_info);
}

type_init(cuda_register_types)
