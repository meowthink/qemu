/*
 * KeyLargo MacIO cells that the New World machine did not previously model:
 * the Feature Control Registers, the two I2S sound cells and the "Keywest"
 * I2C controller.
 *
 * OpenBIOS never touches any of these, which is why mac99 got by without
 * them. A real Apple boot ROM drives all three during early hardware
 * bring-up and spins forever if they do not answer.
 *
 * Register layout and bit meanings come from Linux
 * (arch/powerpc/include/asm/keylargo.h, sound/aoa/soundbus/i2sbus/,
 * arch/powerpc/platforms/powermac/low_i2c.c), Apple's own AppleOnboardAudio
 * sources (AudioI2SHardwareConstants.h, I2STransportInterface.cpp) and the
 * OpenBSD/NetBSD macppc i2s drivers.
 *
 * All KeyLargo registers are little-endian: Linux uses in_le32()/out_le32(),
 * Apple's driver wraps every access in OSRead/WriteLittleInt32(), and the
 * Apple ROM reaches them with lwbrx/stwbrx.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/misc/macio/macio.h"
#include "hw/misc/macio/keylargo.h"
#include "hw/i2c/i2c.h"
#include "hw/core/irq.h"
#include "qemu/module.h"
#include "qom/object.h"
#include "hw/ppc/mac_dbdma.h"
#include "qemu/timer.h"
#include "system/dma.h"
#include "system/reset.h"
#include "trace.h"

/* ---------------------------------------------------------------------- */
/* Feature Control Registers                                              */
/* ---------------------------------------------------------------------- */

/*
 * MBCR at 0x34, then FCR0..FCR5 at 0x38..0x4c. The Apple ROM writes every
 * one of them before reading any back, so plain storage is faithful enough;
 * what matters is that FCR1's I2S clock-gating bits are visible to the I2S
 * cells below.
 */
#define KEYLARGO_FCR_BASE       0x34
#define KEYLARGO_FCR_SIZE       0x1c

static uint64_t keylargo_fcr_read(void *opaque, hwaddr addr, unsigned size)
{
    KeyLargoState *s = opaque;
    uint32_t val = s->fcr[addr >> 2];

    trace_keylargo_fcr_read(KEYLARGO_FCR_BASE + addr, val);
    return val;
}

static void keylargo_fcr_write(void *opaque, hwaddr addr, uint64_t value,
                               unsigned size)
{
    KeyLargoState *s = opaque;

    trace_keylargo_fcr_write(KEYLARGO_FCR_BASE + addr, (uint32_t)value);
    s->fcr[addr >> 2] = value;

    /*
     * Gating an I2S cell's clock is what makes its "clocks stopped" status
     * latch fire; firmware polls for that before reprogramming the cell.
     */
    keylargo_i2s_update_clocks(s);
}

static const MemoryRegionOps keylargo_fcr_ops = {
    .read = keylargo_fcr_read,
    .write = keylargo_fcr_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = { .min_access_size = 4, .max_access_size = 4 },
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

/* ---------------------------------------------------------------------- */
/* I2S cells                                                              */
/* ---------------------------------------------------------------------- */

/*
 * An I2S cell is considered to be running only when its FCR1 bits say the
 * cell is enabled, not held in software reset, and its clock is un-gated.
 * The moment that stops being true the "clocks stopped" interrupt latches,
 * and it stays latched until firmware writes a 1 to it.
 *
 * This is the handshake the Apple ROM performs: pulse I2S0's reset bit, wait
 * for clocks-stopped, then reprogram the serial format. Nothing in this
 * register reports "cell ready" -- getting that backwards means firmware
 * waits forever.
 */
static bool keylargo_i2s_clocks_running(KeyLargoState *s, int cell)
{
    uint32_t fcr1 = s->fcr[(0x3c - KEYLARGO_FCR_BASE) >> 2];
    uint32_t enable, reset, clk;

    if (cell == 0) {
        enable = KL1_I2S0_CELL_ENABLE;
        reset = KL1_I2S0_RESET;
        clk = KL1_I2S0_CLK_ENABLE_BIT;
    } else {
        enable = KL1_I2S1_CELL_ENABLE;
        reset = KL1_I2S1_RESET;
        clk = KL1_I2S1_CLK_ENABLE_BIT;
    }

    return (fcr1 & enable) && (fcr1 & clk) && !(fcr1 & reset);
}

static uint32_t keylargo_i2s_frame_count_now(KeyLargoI2SState *c);

void keylargo_i2s_update_clocks(KeyLargoState *s)
{
    int cell;

    for (cell = 0; cell < 2; cell++) {
        KeyLargoI2SState *c = &s->i2s[cell];
        bool running = keylargo_i2s_clocks_running(s, cell);

        if (!running) {
            c->intr_ctl |= I2S_INT_CLOCKS_STOPPED_PENDING;
        }
    }
}

static int keylargo_i2s_rate(KeyLargoI2SState *c);

/*
 * How far the reported FRAME_COUNT trails real time. The guest's whole
 * position model hangs off this counter: IOAudioEngine's eraser zeroes
 * the ring up to it, and the mixer writes a couple of blocks ahead of
 * it. Our DMA walk jitters within [now - MAX_DEBT, now] around the
 * main loop, so the counter has to sit far enough in the past that the
 * eraser (behind the counter) can never overtake the walk -- reading
 * freshly erased frames was audible as constant 5-20ms dropouts. Keep
 * COUNT_LAG > MAX_DEBT with margin.
 */
#define KL_I2S_COUNT_LAG_NS     (20 * 1000 * 1000)


static uint32_t keylargo_i2s_frame_count_now(KeyLargoI2SState *c)
{
    int64_t rate = keylargo_i2s_rate(c);
    uint64_t lag_frames = rate > 0 ?
        (uint64_t)rate * KL_I2S_COUNT_LAG_NS / NANOSECONDS_PER_SECOND : 0;
    uint64_t counted = c->walk_frames > lag_frames ?
        c->walk_frames - lag_frames : 0;

    return (uint32_t)counted + c->frame_count_bias;
}

static uint64_t keylargo_i2s_read(void *opaque, hwaddr addr, unsigned size)
{
    KeyLargoI2SState *c = opaque;
    uint32_t val;

    switch (addr) {
    case I2S_REG_INT_CTL:
        val = c->intr_ctl;
        break;
    case I2S_REG_SERIAL_FORMAT:
        val = c->serial_format;
        break;
    case I2S_REG_DATA_WORD_SIZES:
        val = c->data_word_sizes;
        break;
    case I2S_REG_FRAME_COUNT:
        val = keylargo_i2s_frame_count_now(c);
        break;
    default:
        val = 0;
        break;
    }

    trace_keylargo_i2s_read(c->cell, addr, val);
    return val;
}

static void keylargo_i2s_write(void *opaque, hwaddr addr, uint64_t value,
                               unsigned size)
{
    KeyLargoI2SState *c = opaque;

    trace_keylargo_i2s_write(c->cell, addr, (uint32_t)value);

    switch (addr) {
    case I2S_REG_INT_CTL:
        /* Pending bits are write-1-to-clear; enable bits are read/write. */
        c->intr_ctl &= ~(value & I2S_INT_PENDING_MASK);
        c->intr_ctl = (c->intr_ctl & I2S_INT_PENDING_MASK) |
                      (value & ~I2S_INT_PENDING_MASK);
        break;
    case I2S_REG_SERIAL_FORMAT:
        c->serial_format = value;
        break;
    case I2S_REG_DATA_WORD_SIZES:
        c->data_word_sizes = value;
        break;
    case I2S_REG_FRAME_COUNT:
        /*
         * Bias against the raw walk count, not the lagged readback:
         * biasing against the lagged value would cancel the lag, and
         * the eraser (which chases this register) would race the DMA
         * walk again.
         */
        c->frame_count_bias = value - (uint32_t)c->walk_frames;
        break;
    default:
        break;
    }
}

/*
 * Decode the playback rate from the serial format register. Bits 31:30 pick
 * the clock source, 28:24 the MClk divisor and 23:20 the SClk divisor, and
 * an I2S frame is 64 SClks. The divisor encoding is irregular: 0x14, 0x13
 * and 0x12 mean divide by 1, 3 and 5, and every other value n means
 * divide by (n + 1) * 2.
 */
static int keylargo_i2s_divisor(uint32_t field)
{
    switch (field) {
    case 0x14: return 1;
    case 0x13: return 3;
    case 0x12: return 5;
    default:   return (field + 1) * 2;
    }
}

static int keylargo_i2s_rate(KeyLargoI2SState *c)
{
    static const int clock_hz[4] = { 0, 45158400, 49152000, 18432000 };
    uint32_t fmt = c->serial_format;
    int src = clock_hz[(fmt >> 30) & 3];
    int mdiv, sdiv, rate;

    if (!src) {
        return 44100;
    }
    mdiv = keylargo_i2s_divisor((fmt >> 24) & 0x1f);
    sdiv = keylargo_i2s_divisor((fmt >> 20) & 0x0f);
    rate = src / (mdiv * sdiv * 64);

    return rate > 0 ? rate : 44100;
}

/* Bits 1:0 of the word-size register select 16- or 24-bit samples. */
static int keylargo_i2s_frame_bytes(KeyLargoI2SState *c)
{
    return ((c->data_word_sizes & 3) == 3) ? 8 : 4;   /* stereo */
}

static void keylargo_i2s_out_complete(void *opaque)
{
    KeyLargoI2SState *c = opaque;
    DBDMA_io *io = c->pending_out_io;

    c->pending_out_io = NULL;
    if (io) {
        io->dma_end(io);
    }
}

/*
 * Nothing renders the samples yet, but the descriptor still has to be
 * retired only once its audio would have finished playing. Without that the
 * firmware's chime ring never stops.
 */
#define KL_I2S_FRAME_BYTES 4

/*
 * Playout cushion (ported from awacs.c, where these values are
 * ear-verified): target FIFO depth before a freshly (re)started stream
 * begins draining, and how long to keep waiting for it once the guest
 * stops pushing (so a short final burst still plays out).
 */
#define KL_I2S_PREBUF_NS        (60 * 1000 * 1000)
#define KL_I2S_PREBUF_GIVEUP_NS (100 * 1000 * 1000)

/*
 * How far completion pacing may fall behind before descriptors stop
 * retiring back-to-back; also bounds the burst (and the dma_end
 * recursion depth, ~28 of Mac OS X's 128-byte ring pieces) after a
 * stall.
 */
#define KL_I2S_MAX_DEBT_NS      (10 * 1000 * 1000)


static bool keylargo_i2s_codec_muted(KeyLargoI2SState *c);

/*
 * The i2s cell keeps clocking zeros whenever the guest has nothing
 * queued, and the emulated voice must do the same: the mixing engine
 * only advances a shared hw voice by the minimum progress across all
 * active sw voices, so an active voice that writes nothing pins that
 * minimum at zero and starves every other frontend on the same
 * audiodev (usb-audio being the audible victim).
 */
static void keylargo_i2s_write_silence(KeyLargoI2SState *c, int avail)
{
    static const uint8_t zeros[4096];

    avail -= avail % KL_I2S_FRAME_BYTES;
    while (avail >= KL_I2S_FRAME_BYTES) {
        size_t chunk = MIN(avail, (int)sizeof(zeros));
        size_t written;

        written = audio_be_write(c->audio_be, c->voice,
                                 (void *)zeros, chunk);
        written -= written % KL_I2S_FRAME_BYTES;
        avail -= written;
        if (written < chunk) {
            break;
        }
    }
}

static void keylargo_i2s_audio_cb(void *opaque, int avail)
{
    KeyLargoI2SState *c = opaque;
    bool muted = keylargo_i2s_codec_muted(c);

    trace_keylargo_i2s_cb(c->cell, avail, c->fifo_count, c->prebuffering);

    if (c->fifo_count == 0) {
        /* Stream drained (or never started): next data prebuffers. */
        c->prebuffering = true;
        keylargo_i2s_write_silence(c, avail);
        return;
    }
    if (c->prebuffering) {
        int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        int rate = c->voice_rate ? c->voice_rate : 44100;
        uint32_t want = (uint64_t)rate * KL_I2S_FRAME_BYTES *
                        KL_I2S_PREBUF_NS / NANOSECONDS_PER_SECOND;

        if (c->fifo_count < want &&
            now - c->last_push_ns < KL_I2S_PREBUF_GIVEUP_NS) {
            keylargo_i2s_write_silence(c, avail);
            return;
        }
        c->prebuffering = false;
    }

    avail -= avail % KL_I2S_FRAME_BYTES;
    while (avail >= KL_I2S_FRAME_BYTES &&
           c->fifo_count >= KL_I2S_FRAME_BYTES) {
        uint8_t staging[4096];
        int chunk = MIN(avail, (int)sizeof(staging));
        size_t written;
        int i;

        chunk = MIN(chunk, (int)c->fifo_count);
        chunk -= chunk % KL_I2S_FRAME_BYTES;
        for (i = 0; i < chunk; i++) {
            staging[i] = c->out_fifo[c->fifo_rptr];
            c->fifo_rptr = (c->fifo_rptr + 1) % sizeof(c->out_fifo);
        }
        c->fifo_count -= chunk;

        if (muted) {
            memset(staging, 0, chunk);
        }
        written = audio_be_write(c->audio_be, c->voice, staging, chunk);
        written -= written % KL_I2S_FRAME_BYTES;
        avail -= written;
        if (written < (size_t)chunk) {
            c->fifo_rptr = (c->fifo_rptr + sizeof(c->out_fifo) -
                            (chunk - written)) % sizeof(c->out_fifo);
            c->fifo_count += chunk - written;
            break;
        }
    }
}

static void keylargo_i2s_tap_out(KeyLargoI2SState *c, DBDMA_io *io, int rate)
{
    uint8_t buf[4096];
    hwaddr addr = io->addr;
    int remaining = io->len;

    if (!c->audio_be) {
        return;
    }
    if (!c->voice || c->voice_rate != rate) {
        struct audsettings as = {
            .freq = rate,
            .nchannels = 2,
            .fmt = AUDIO_FORMAT_S16,
            .big_endian = true,      /* I2S frames are big-endian S16 */
        };

        c->voice_rate = rate;
        /*
         * "Tumbler" is Apple's name for this TAS3001C-based sound
         * subsystem (AppleLegacyAudio's Texas driver); the codec sits
         * on i2s-a, so that cell carries the name.
         */
        c->voice = audio_be_open_out(c->audio_be, c->voice,
                                     c->cell ? "tumbler-i2sb.out"
                                             : "tumbler.out",
                                     c, keylargo_i2s_audio_cb, &as);
        audio_be_set_active_out(c->audio_be, c->voice, true);
        c->prebuffering = true;
    }

    c->last_push_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    while (remaining > 0) {
        int len = MIN(remaining, (int)sizeof(buf));
        int i;

        dma_memory_read(&address_space_memory, addr, buf, len,
                        MEMTXATTRS_UNSPECIFIED);
        if (c->dump_fp) {
            fwrite(buf, 1, len, c->dump_fp);
        }
        for (i = 0; i < len && c->fifo_count < sizeof(c->out_fifo); i++) {
            c->out_fifo[c->fifo_wptr] = buf[i];
            c->fifo_wptr = (c->fifo_wptr + 1) % sizeof(c->out_fifo);
            c->fifo_count++;
        }
        if (i < len) {
            trace_keylargo_i2s_fifo_overflow(c->cell, len - i);
        }
        addr += len;
        remaining -= len;
    }
}

static void keylargo_i2s_dma_rw(DBDMA_io *io)
{
    KeyLargoI2SState *c = io->opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    int rate = keylargo_i2s_rate(c);
    int frames = io->len / keylargo_i2s_frame_bytes(c);
    int64_t dur_ns = (int64_t)frames * NANOSECONDS_PER_SECOND / rate;

    trace_keylargo_i2s_dma(c->cell, io->addr, io->len, rate);

    if (io->is_dma_out) {
        c->walk_frames += frames;
        if (keylargo_i2s_frame_bytes(c) == KL_I2S_FRAME_BYTES) {
            keylargo_i2s_tap_out(c, io, rate);
        }
    }

    /*
     * Cap the pacing debt so a stalled stream doesn't burst unboundedly
     * when it resumes, then charge this descriptor.
     */
    if (c->play_deadline_ns < now - KL_I2S_MAX_DEBT_NS) {
        c->play_deadline_ns = now - KL_I2S_MAX_DEBT_NS;
    }
    c->play_deadline_ns += dur_ns;

    if (c->play_deadline_ns <= now) {
        /*
         * Behind schedule: retire immediately. Waiting out a timer costs
         * a main-loop round trip per descriptor, and Mac OS X's engine
         * runs the ring in 128-byte pieces (725us each) -- paying ~0.5ms
         * of turnaround on every one capped the stream at half the
         * sample rate, so the FIFO underran continuously (heard as
         * chopped, repeating audio). The debt cap above bounds how many
         * descriptors can retire back-to-back this way.
         */
        io->dma_end(io);
        return;
    }

    c->pending_out_io = io;
    timer_mod(c->out_complete_timer, c->play_deadline_ns);
}

static void keylargo_i2s_dma_flush(DBDMA_io *io)
{
    KeyLargoI2SState *c = io->opaque;

    /*
     * DBDMA calls this on FLUSH and whenever the channel stops; the
     * contract (see mac_dbdma.c) is that the outstanding command must
     * complete before ACTIVE drops. Left pending, our completion timer
     * fired dma_end() into whatever the guest had reprogrammed the
     * channel to by then: Mac OS X's audio engine stop/start sequence
     * (IODBDMAStop + IODBDMAReset, then IODBDMAStart at the ring base
     * with a zeroed FRAME_COUNT) got its fresh ring skipped ahead by a
     * stale continuation on every restart, sliding the DMA walk out of
     * phase with the driver's ring-position model -- each output-device
     * switch degraded playback further until only slivers of each sound
     * survived the eraser.
     */
    if (c->pending_out_io) {
        DBDMA_io *pending = c->pending_out_io;

        c->pending_out_io = NULL;
        timer_del(c->out_complete_timer);
        pending->dma_end(pending);
    }
    /* No pacing debt survives into a restarted stream. */
    c->play_deadline_ns = 0;
}

void keylargo_i2s_register_dma(KeyLargoState *s, void *dbdma,
                               qemu_irq tx0_irq, qemu_irq rx0_irq)
{
    int cell;

    for (cell = 0; cell < 2; cell++) {
        /*
         * The channel interrupt is not optional decoration: Mac OS X's
         * audio engine takes its sample-clock timestamp from the ring's
         * interrupt descriptor. With no interrupt delivered the engine
         * clock never advances and every sound loops at its current
         * position forever.
         */
        DBDMA_register_channel(dbdma, KEYLARGO_I2S_DMA_OUT(cell),
                               cell == 0 ? tx0_irq : NULL,
                               keylargo_i2s_dma_rw, keylargo_i2s_dma_flush,
                               &s->i2s[cell]);
        DBDMA_register_channel(dbdma, KEYLARGO_I2S_DMA_IN(cell),
                               cell == 0 ? rx0_irq : NULL,
                               keylargo_i2s_dma_rw, keylargo_i2s_dma_flush,
                               &s->i2s[cell]);
    }
}

static const MemoryRegionOps keylargo_i2s_ops = {
    .read = keylargo_i2s_read,
    .write = keylargo_i2s_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = { .min_access_size = 4, .max_access_size = 4 },
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};


/* ---------------------------------------------------------------------- */
/* TAS3001 "deq" audio equalizer                                          */
/* ---------------------------------------------------------------------- */

/*
 * The Digital Audio's sound path runs through a TI TAS3001 digital EQ,
 * which the real PowerMac3,4 device tree lists as the "deq" node on the
 * KeyLargo I2C bus at i2c-address 0x68 (8-bit) = 0x34 (7-bit). Mac OS X's
 * AppleLegacyAudio programs it at boot and its failure path is drastic:
 * repeated failed writes to it were the direct prelude to
 * ApplePMU::shutdownSystem powering the machine off on 10.4.
 *
 * The chip is write-mostly (subaddress + up to a dozen coefficient bytes
 * per register); nothing reads back audio state during boot. Accepting
 * and storing the writes is enough to keep the driver satisfied.
 */
#define TYPE_TAS3001 "tas3001"
OBJECT_DECLARE_SIMPLE_TYPE(TAS3001State, TAS3001)

#define TAS3001_MAX_REG_LEN 16

struct TAS3001State {
    I2CSlave parent_obj;
    uint8_t subaddr;
    uint8_t pos;
    bool vol_written;
    uint8_t regs[0x100][TAS3001_MAX_REG_LEN];
};

static int tas3001_event(I2CSlave *i2c, enum i2c_event event)
{
    TAS3001State *t = TAS3001(i2c);

    if (event == I2C_START_SEND) {
        t->pos = 0;
    }
    return 0;
}

static int tas3001_send(I2CSlave *i2c, uint8_t data)
{
    TAS3001State *t = TAS3001(i2c);

    if (t->pos == 0) {
        t->subaddr = data;
    } else if (t->pos - 1 < TAS3001_MAX_REG_LEN) {
        t->regs[t->subaddr][t->pos - 1] = data;
        if (t->subaddr == 0x04) {
            t->vol_written = true;
        }
        trace_tas3001_reg_write(t->subaddr, t->pos - 1, data);
    }
    t->pos++;
    return 0;
}

static uint8_t tas3001_recv(I2CSlave *i2c)
{
    TAS3001State *t = TAS3001(i2c);
    uint8_t val = t->regs[t->subaddr][0];

    return val;
}

static void tas3001_class_init(ObjectClass *klass, const void *data)
{
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    k->event = tas3001_event;
    k->send = tas3001_send;
    k->recv = tas3001_recv;
}

static const TypeInfo tas3001_info = {
    .name = TYPE_TAS3001,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(TAS3001State),
    .class_init = tas3001_class_init,
};

/*
 * Mac OS X does not stop the I2S DMA when sound finishes: the HAL's ring
 * keeps cycling and the driver instead silences the TAS3001 -- its idle
 * path ramps the codec volume register (subaddress 4: three bytes each
 * of left and right 4.16 fixed-point gain) down to zero. Honour that,
 * or the last fragment of audio in the ring plays forever.
 */
static bool keylargo_i2s_codec_muted(KeyLargoI2SState *c)
{
    TAS3001State *t;
    const uint8_t *vol;
    int i;

    if (!c->codec) {
        return false;
    }
    t = TAS3001((I2CSlave *)c->codec);
    if (!t->vol_written) {
        /*
         * Nothing has programmed the codec yet -- the boot chime plays
         * before any OS driver touches it, and the reset-state registers
         * read zero. Only a driver-written all-zero volume means mute.
         */
        return false;
    }
    vol = t->regs[0x04];
    for (i = 0; i < 6; i++) {
        if (vol[i]) {
            return false;
        }
    }
    return true;
}

static void keylargo_register_types(void)
{
    type_register_static(&tas3001_info);
}

type_init(keylargo_register_types)

static void keylargo_reset(void *opaque)
{
    KeyLargoState *s = opaque;

    /* FCRs back to their cold-boot state: cells disabled, clocks stopped */
    memset(s->fcr, 0, sizeof(s->fcr));
    keylargo_i2s_update_clocks(s);
    keywest_i2c_reset(&s->i2c);
}

KeyLargoState *keylargo_cells_init(DeviceState *owner, MemoryRegion *bar)
{
    KeyLargoState *s = g_new0(KeyLargoState, 1);
    int cell;

    memory_region_init_io(&s->fcr_mem, OBJECT(owner), &keylargo_fcr_ops, s,
                          "keylargo-fcr", KEYLARGO_FCR_SIZE);
    memory_region_add_subregion(bar, KEYLARGO_FCR_BASE, &s->fcr_mem);

    for (cell = 0; cell < 2; cell++) {
        g_autofree char *name = g_strdup_printf("keylargo-i2s%d", cell);

        s->i2s[cell].cell = cell;
        s->i2s[cell].out_complete_timer =
            timer_new_ns(QEMU_CLOCK_VIRTUAL, keylargo_i2s_out_complete,
                         &s->i2s[cell]);
        memory_region_init_io(&s->i2s[cell].mem, OBJECT(owner),
                              &keylargo_i2s_ops, &s->i2s[cell], name,
                              KEYLARGO_I2S_SIZE);
        memory_region_add_subregion(bar, KEYLARGO_I2S0_BASE +
                                    cell * KEYLARGO_I2S_STRIDE,
                                    &s->i2s[cell].mem);
    }

    /* Both cells come out of reset with their clocks stopped. */
    keylargo_i2s_update_clocks(s);

    keywest_i2c_init(&s->i2c, owner, "keylargo-i2c", KEYLARGO_I2C_SIZE);
    memory_region_add_subregion(bar, KEYLARGO_I2C_BASE, &s->i2c.mem);

    /* The "deq" TAS3001 audio EQ, at 0x34 like the real machine's. */
    s->i2s[0].codec = i2c_slave_create_simple(s->i2c.bus, TYPE_TAS3001, 0x34);

    qemu_register_reset(keylargo_reset, s);

    return s;
}

