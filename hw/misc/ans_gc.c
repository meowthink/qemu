/*
 * Apple Network Server's Grand Central (343S1125) and its GBUS island.
 *
 * Grand Central is a PCI device (106B:0002, revision 2) with a 1 MB BAR.
 * Apple, Network Server Hardware Developer Notes, 4.6.1 gives the decode:
 *
 *   1_dddd_RRRR_RRRR_rrrr  device d at +0x10000 + d*0x1000
 *   DMA controller registers at +0x0000, channel n at +0x8000 + n*0x100
 *
 * This device currently models the ANS-specific half -- the GBUS island:
 * the front-panel LCD (write-only), the NVRAM (bank port + data window),
 * the two board registers and the Ethernet address PROM.  The mac-io-family
 * children (MACE, SCC, audio, SWIM3, VIA) and the DBDMA engine are the next
 * step and are plain scratch here.
 *
 * Board registers 1 and 2: every bit is ACTIVE LOW unless it ends in H, so
 * "healthy" reads all ones; board register 2 is polled and never
 * interrupts.  The front keyswitch is read from board register 1 bits 13
 * (ServiceL) and 14 (LockedL).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/ans_gc.h"
#include "hw/pci/pci_device.h"
#include "qapi/error.h"
#include "qemu/module.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "migration/vmstate.h"
#include "system/system.h"
#include "hw/input/adb.h"
#include "hw/core/irq.h"

#define ANS_OFF_EPROM 0x19000
#define ANS_OFF_BREG1 0x1A000
#define ANS_OFF_LCD   0x1C000
#define ANS_OFF_NVCTL 0x1D000
#define ANS_OFF_BREG2 0x1E000
#define ANS_OFF_NVDAT 0x1F000

/* Same clock and register stride the mac-io's ESCC is driven with. */
#define ANS_GC_ESCC_CLOCK 3686400
#define ANS_GC_ESCC_SHIFT 4
/* The board's timebase, which the VIA/CUDA timers are clocked from. */
#define ANS_GC_TB_FREQ 12500000

/*
 * Grand Central's interrupt block: one 32-bit little-endian event/mask/clear/
 * levels set at +$20..+$2C, not the mac-io's two-bank Heathrow PIC.  Line
 * numbering is the Mac TNT map (DBDMA 0-10, SCSI 0xC, MESH 0xD, MACE 0xE,
 * SCC-A 0xF, SCC-B 0x10, AWACS 0x11, VIA/Cuda 0x12, SWIM3 0x13, NMI 0x14,
 * EXT1-11 from 0x15); the ANS re-purposes only the EXT lines.
 */
#define ANS_GC_INT_EVENTS 0x20
#define ANS_GC_INT_MASK   0x24
#define ANS_GC_INT_CLEAR  0x28
#define ANS_GC_INT_LEVELS 0x2c
#define ANS_GC_INT_MODE_ACK 0x80000000u

#define ANS_GC_INT_SCSI0 12  /* the 53C94 */
#define ANS_GC_INT_SCCA  15
#define ANS_GC_INT_SCCB  16
#define ANS_GC_INT_VIA   18

static void ans_gc_int_update(AnsGCState *s)
{
    bool line;

    /*
     * Clear-mode 1 (selected by the $80000000 acknowledge, the NanoKernel's
     * scheme) drives the CPU line from a per-source latch; mode 0
     * (power-on, the MkLinux scheme) is combinational.
     */
    if (s->int_mode1) {
        line = (s->int_latch & s->int_mask) != 0;
    } else {
        line = ((s->int_events | s->int_levels) & s->int_mask) != 0;
    }
    qemu_set_irq(s->pic_out, line);
}

static void ans_gc_int_edge(AnsGCState *s, uint32_t bit)
{
    s->int_events |= bit;
    if (s->int_mask & bit) {
        s->int_latch |= bit;
    }
}

static void ans_gc_int_set_source(AnsGCState *s, int n, bool level)
{
    uint32_t bit = 1u << n;
    bool was = (s->int_levels & bit) != 0;

    if (level) {
        s->int_levels |= bit;
        if (!was) {
            ans_gc_int_edge(s, bit);
        }
    } else {
        s->int_levels &= ~bit;
        /* Mode 1 is interrupt-on-change: a deassertion latches too. */
        if (was && (s->int_mask & bit)) {
            s->int_latch |= bit;
        }
    }
    ans_gc_int_update(s);
}

static void ans_gc_int_in(void *opaque, int n, int level)
{
    ans_gc_int_set_source(opaque, n, level != 0);
}

static uint64_t ans_gc_int_read(void *opaque, hwaddr addr, unsigned size)
{
    AnsGCState *s = opaque;
    uint32_t off = addr & 0xffc;
    uint32_t val;

    switch (off) {
    case ANS_GC_INT_EVENTS:
        val = s->int_events;
        break;
    case ANS_GC_INT_MASK:
        val = s->int_mask;
        break;
    case ANS_GC_INT_LEVELS:
        val = s->int_levels;
        break;
    default:
        val = 0;
        break;
    }
    return val >> (8 * (addr & 3));
}

static void ans_gc_int_write(void *opaque, hwaddr addr, uint64_t value,
                             unsigned size)
{
    AnsGCState *s = opaque;
    uint32_t off = addr & 0xffc;
    unsigned shift = 8 * (addr & 3);
    uint32_t bytes = size >= 4 ? 0xffffffffu : (1u << (8 * size)) - 1;
    uint32_t val = ((uint32_t)value & bytes) << shift;

    switch (off) {
    case ANS_GC_INT_EVENTS:
    case ANS_GC_INT_CLEAR:
        s->int_events &= ~(val & ~ANS_GC_INT_MODE_ACK);
        if (off == ANS_GC_INT_CLEAR) {
            s->int_mode1 = (val & ANS_GC_INT_MODE_ACK) != 0;
            if (s->int_mode1) {
                s->int_latch = 0;
            }
        }
        break;
    case ANS_GC_INT_MASK: {
        uint32_t newly = val & ~s->int_mask;

        s->int_mask = (s->int_mask & ~(bytes << shift)) | val;
        s->int_latch |= (s->int_events | s->int_levels) & newly;
        break;
    }
    default:
        break;
    }
    ans_gc_int_update(s);
}

static const MemoryRegionOps ans_gc_int_ops = {
    .read = ans_gc_int_read,
    .write = ans_gc_int_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

/*
 * Temporary scaffolding: the GC device island slots we have not modelled
 * yet (SCSI, MACE, audio, SWIM3, VIA) read open bus.  Logging the accesses
 * is how the firmware tells us what it wants from each one.
 */
static uint64_t ans_gc_slot_read(void *opaque, hwaddr addr, unsigned size)
{
    unsigned slot = (unsigned)(uintptr_t)opaque;

    qemu_log_mask(LOG_GUEST_ERROR,
                  "ans-gc: slot %u read  +0x%05x size %u\n",
                  slot, (unsigned)(slot * 0x1000 + addr), size);
    return 0;
}

static void ans_gc_slot_write(void *opaque, hwaddr addr, uint64_t val,
                              unsigned size)
{
    unsigned slot = (unsigned)(uintptr_t)opaque;

    qemu_log_mask(LOG_GUEST_ERROR,
                  "ans-gc: slot %u write +0x%05x size %u = 0x%x\n",
                  slot, (unsigned)(slot * 0x1000 + addr), size,
                  (unsigned)val);
}

static const MemoryRegionOps ans_gc_slot_ops = {
    .read = ans_gc_slot_read,
    .write = ans_gc_slot_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

/*
 * The SCC "compatibility" port at +$12000 is the 68k-era aperture onto the
 * same ESCC the MacRISC port at +$13000 drives: address bit 1 is the A/B pin
 * and bit 2 the D/C pin (dev notes 4.6.1).  The 68k Serial Driver and the AIX
 * bootstrap use this port; everything else uses the other.
 */
#define ANS_GC_LEGACY_SCC 0x12000

static hwaddr ans_gc_legacy_scc_reg(hwaddr addr)
{
    return (((addr >> 1) & 1) << 5) | (((addr >> 2) & 1) << 4);
}

static uint64_t ans_gc_legacy_scc_read(void *opaque, hwaddr addr, unsigned size)
{
    AnsGCState *s = opaque;
    MemoryRegion *escc = sysbus_mmio_get_region(SYS_BUS_DEVICE(&s->escc), 0);
    uint64_t val = 0;
    unsigned i;

    for (i = 0; i < size; i++) {
        uint64_t b = 0;

        memory_region_dispatch_read(escc, ans_gc_legacy_scc_reg(addr + i), &b,
                                    size_memop(1), MEMTXATTRS_UNSPECIFIED);
        val = (val << 8) | (b & 0xff);
    }
    return val;
}

static void ans_gc_legacy_scc_write(void *opaque, hwaddr addr, uint64_t val,
                                    unsigned size)
{
    AnsGCState *s = opaque;
    MemoryRegion *escc = sysbus_mmio_get_region(SYS_BUS_DEVICE(&s->escc), 0);
    unsigned i;

    for (i = 0; i < size; i++) {
        memory_region_dispatch_write(escc, ans_gc_legacy_scc_reg(addr + i),
                                     val >> (8 * (size - 1 - i)),
                                     size_memop(1), MEMTXATTRS_UNSPECIFIED);
    }
}

static const MemoryRegionOps ans_gc_legacy_scc_ops = {
    .read = ans_gc_legacy_scc_read,
    .write = ans_gc_legacy_scc_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 2 },
};

/*
 * Board register 1 (dev notes 4.1.1.1).  Active low unless the name ends in
 * H: bit 13 ServiceL and bit 14 LockedL are the front keyswitch, bit 15 is
 * TwoSuppliesH, bits 11/12 are BoxId0/BoxId1, and bit 8 is the factory-test
 * strap POST checks (clear sends the ROM into its serial test monitor).
 */
#define ANS_BREG1_STRAP   0x0900u
#define ANS_BREG1_LOCKED  0x2000u   /* LockedL low */
#define ANS_BREG1_SERVICE 0x4000u   /* ServiceL low */
#define ANS_BREG1_TWO_PSU 0x8000u
#define ANS_BREG2_HEALTHY 0xffffu

static uint64_t ans_gc_control_read(void *opaque, hwaddr addr, unsigned size)
{
    AnsGCState *s = opaque;
    return s->control_reg[(addr >> 2) & 0x3ff] >> (8 * ((addr & 3) ^ (4 - size)));
}

static void ans_gc_control_write(void *opaque, hwaddr addr, uint64_t val,
                                 unsigned size)
{
    AnsGCState *s = opaque;
    unsigned i = (addr >> 2) & 0x3ff;
    uint32_t mask = size >= 4 ? 0xffffffffu : (0xffu << (8 * (addr & 3)));

    s->control_reg[i] = (s->control_reg[i] & ~mask) | ((uint32_t)val << (8 * (addr & 3)));
}

static uint64_t ans_gc_lcd_read(void *opaque, hwaddr addr, unsigned size)
{
    AnsGCState *s = opaque;

    /* Write-only interface; the ROM only ever stores into it. */
    if (addr < 0x10) {
        return s->lcd_cmd;
    }
    if (addr < 0x20) {
        return s->lcd_data;
    }
    return 0xffff;
}

/* HD44780 4x20 DDRAM map: lines start at $00, $40, $14 and $54. */
static const int ans_lcd_line_base[4] = { 0x00, 0x40, 0x14, 0x54 };

/* A panel state is logged once.  A blinking code rewrites three or four
 * states thousands of times a second, which is how one 15-minute run
 * produced 1.4 GB of log. */
#define ANS_LCD_SEEN 128
static bool ans_lcd_state_seen(char line[4][24])
{
    static char seen[ANS_LCD_SEEN][4][24];
    static unsigned nseen;
    unsigned i;

    for (i = 0; i < nseen; i++) {
        if (memcmp(seen[i], line, sizeof(seen[0])) == 0) {
            return true;
        }
    }
    if (nseen < ANS_LCD_SEEN) {
        memcpy(seen[nseen++], line, sizeof(seen[0]));
    } else {
        /* Evict the oldest so a long run can still report new states. */
        memmove(seen, seen + 1, sizeof(seen) - sizeof(seen[0]));
        memcpy(seen[ANS_LCD_SEEN - 1], line, sizeof(seen[0]));
    }
    return false;
}

/* The panel is this machine's only narrator until Open Firmware is up.
 * ANS_LCD_LOG=path appends each new panel state to that file;
 * ANS_LCD_TRACE=1 is the unbounded per-write hammer. */
static void ans_lcd_dump(AnsGCState *s, bool trace)
{
    char line[4][24];
    const char *log = getenv("ANS_LCD_LOG");
    int i;

    for (i = 0; i < 4; i++) {
        memcpy(line[i], &s->lcd_ddram[ans_lcd_line_base[i]], 20);
        line[i][20] = '\0';
    }
    if (trace) {
        fprintf(stderr, "ans-lcd: %" PRId64 "ms |%.20s|%.20s|%.20s|%.20s|\n",
                qemu_clock_get_ms(QEMU_CLOCK_REALTIME),
                line[0], line[1], line[2], line[3]);
    } else {
        qemu_log_mask(LOG_GUEST_ERROR, "ans-lcd: |%.20s|%.20s|%.20s|%.20s|\n",
                      line[0], line[1], line[2], line[3]);
    }
    if (log && !ans_lcd_state_seen(line)) {
        FILE *f = fopen(log, "a");

        if (f) {
            fprintf(f, "%" PRId64 " |%.20s|%.20s|%.20s|%.20s|\n",
                    qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL),
                    line[0], line[1], line[2], line[3]);
            fclose(f);
        }
    }
}

static void ans_gc_lcd_write(void *opaque, hwaddr addr, uint64_t val,
                             unsigned size)
{
    AnsGCState *s = opaque;

    if (addr < 0x10) {
        /* Command register: only the DDRAM address and clear/home matter. */
        s->lcd_cmd = val;
        if (val & 0x80) {
            s->lcd_cursor = val & 0x7f;
        } else if (val == 0x01) {
            memset(s->lcd_ddram, ' ', sizeof(s->lcd_ddram));
            s->lcd_cursor = 0;
        } else if (val == 0x02) {
            s->lcd_cursor = 0;
        }
        if ((val & 0xfc) == 0x80 || val == 0x01) {
            ans_lcd_dump(s, false);
        }
    } else if (addr < 0x20) {
        s->lcd_data = val;
        s->lcd_ddram[s->lcd_cursor & 0x7f] = val;
        s->lcd_cursor = (s->lcd_cursor + 1) & 0x7f;
        if (getenv("ANS_LCD_TRACE")) {
            ans_lcd_dump(s, true);
        } else if (getenv("ANS_LCD_LOG") && val != ' ') {
            /* A code left standing still never reaches the next cursor write. */
            ans_lcd_dump(s, false);
        }
    }
    /* +0x20 = timebase enable, +0x30 = POST writes 0xffff once. */
}

/*
 * ANS_NVRAM_DBG[=<max lines>]: log byte-changing writes, reads that do not
 * match the last value written there, and a digest of the scanned offsets.
 * Capped by design: the install's store loop issues millions of accesses a
 * minute, and an unbounded log of it filled the disk once.
 */
static uint8_t nvdbg_val[ANS_GC_NVRAM_SIZE];
static uint8_t nvdbg_wr[ANS_GC_NVRAM_SIZE];
static uint64_t nvdbg_lines, nvdbg_reads, nvdbg_rd_min, nvdbg_rd_max;
static uint64_t nvdbg_limit = 20000;
static bool nvdbg_on, nvdbg_setup_done;

static void ans_nvram_dbg_setup(void)
{
    const char *e;

    if (nvdbg_setup_done) {
        return;
    }
    nvdbg_setup_done = true;
    e = getenv("ANS_NVRAM_DBG");
    if (!e) {
        return;
    }
    nvdbg_on = true;
    if (atoi(e) > 0) {
        nvdbg_limit = atoi(e);
    }
}

static bool ans_nvram_dbg_room(void)
{
    if (nvdbg_lines >= nvdbg_limit) {
        if (nvdbg_lines == nvdbg_limit) {
            fprintf(stderr, "ans-nvram: %" PRId64 "ms log limit (%" PRIu64 " lines)\n",
                    qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL), nvdbg_limit);
            nvdbg_lines++;
        }
        return false;
    }
    nvdbg_lines++;
    return true;
}

/*
 * The bank port is a 32-bit little-endian register: the ROM stores
 * (address >> 5) into it with `stwbrx`, the big-endian CPU's byte-reversed
 * store, so a plain store-and-readback model has to latch it the same way.
 */
static uint64_t ans_gc_nvctl_read(void *opaque, hwaddr addr, unsigned size)
{
    AnsGCState *s = opaque;
    return s->nvram_addr >> (8 * (addr & 3));
}

static void ans_gc_nvctl_write(void *opaque, hwaddr addr, uint64_t val,
                               unsigned size)
{
    AnsGCState *s = opaque;
    unsigned shift = 8 * (addr & 3);
    uint32_t bytes = size >= 4 ? 0xffffffffu : (1u << (8 * size)) - 1;

    s->nvram_addr = (s->nvram_addr & ~(bytes << shift))
                    | (((uint32_t)val & bytes) << shift);
}

/* NVRAM addressing: bank port gets (A >> 5), data window is touched at
 * ((A << 4) & 0x7f0), i.e. one byte per 16-byte window centre. */
static uint32_t ans_gc_nvdata_off(AnsGCState *s, hwaddr addr)
{
    return (((s->nvram_addr & 0x3ff) << 5) | ((addr >> 4) & 0x1f))
           & (ANS_GC_NVRAM_SIZE - 1);
}

static uint64_t ans_gc_nvdata_read(void *opaque, hwaddr addr, unsigned size)
{
    AnsGCState *s = opaque;
    uint32_t off = ans_gc_nvdata_off(s, addr);

    ans_nvram_dbg_setup();
    if (nvdbg_on) {
        nvdbg_reads++;
        if (!nvdbg_rd_min || off < nvdbg_rd_min) {
            nvdbg_rd_min = off ? off : 1;
        }
        if (off > nvdbg_rd_max) {
            nvdbg_rd_max = off;
        }
        if (nvdbg_wr[off] && nvdbg_val[off] != s->nvram[off] &&
            ans_nvram_dbg_room()) {
            fprintf(stderr,
                    "ans-nvram: %" PRId64 "ms VERIFY [0x%04X] read 0x%02x, wrote 0x%02x\n",
                    qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL), off, s->nvram[off],
                    nvdbg_val[off]);
        }
        if (nvdbg_reads % 100000 == 0) {
            fprintf(stderr,
                    "ans-nvram: %" PRId64 "ms %" PRIu64 " reads, offsets 0x%03" PRIx64 "-0x%03" PRIx64 "\n",
                    qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL), nvdbg_reads,
                    nvdbg_rd_min, nvdbg_rd_max);
        }
    }
    qemu_log_mask(LOG_GUEST_ERROR, "ans-nvram: read  [0x%04X] = 0x%02x\n",
                  off, s->nvram[off]);
    return s->nvram[off];
}

static void ans_gc_nvdata_write(void *opaque, hwaddr addr, uint64_t val,
                                unsigned size)
{
    AnsGCState *s = opaque;
    uint32_t off = ans_gc_nvdata_off(s, addr);

    ans_nvram_dbg_setup();
    if (nvdbg_on) {
        if (!nvdbg_wr[off] && ans_nvram_dbg_room()) {
            fprintf(stderr,
                    "ans-nvram: %" PRId64 "ms W [0x%04X] first = 0x%02x (bank 0x%03x)\n",
                    qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL), off,
                    (unsigned)val & 0xff, s->nvram_addr & 0x3ff);
        } else if (nvdbg_wr[off] && nvdbg_val[off] != ((uint8_t)val) &&
                   ans_nvram_dbg_room()) {
            fprintf(stderr,
                    "ans-nvram: %" PRId64 "ms W [0x%04X] 0x%02x -> 0x%02x (bank 0x%03x)\n",
                    qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL), off, nvdbg_val[off],
                    (unsigned)val & 0xff, s->nvram_addr & 0x3ff);
        }
        nvdbg_val[off] = val;
        nvdbg_wr[off] = 1;
    }
    qemu_log_mask(LOG_GUEST_ERROR, "ans-nvram: write [0x%04X] = 0x%02x\n",
                  off, (unsigned)val & 0xff);
    s->nvram[off] = val;
}

/* The Ethernet address PROM: one byte every 16 addresses. */
static uint64_t ans_gc_eprom_read(void *opaque, hwaddr addr, unsigned size)
{
    AnsGCState *s = opaque;
    uint64_t val = 0;
    unsigned i;

    for (i = 0; i < size; i++) {
        val |= (uint64_t)s->eprom_data[(addr + i) & 0xff] << (8 * (size - 1 - i));
    }
    return val;
}

static void ans_gc_eprom_write(void *opaque, hwaddr addr, uint64_t val,
                               unsigned size)
{
}

/*
 * Board registers 1 and 2, read-only.  Byte j of the register is bits
 * 8j..8j+7, not the big-endian peripheral's lane order: the board wires the
 * register so that the guest's `lwbrx` recovers it, and the ROM reads byte
 * +1 for the keyswitch and the BoxId straps.
 */
static uint64_t ans_gc_breg_lanes(uint32_t reg, hwaddr addr, unsigned size)
{
    uint64_t val = 0;
    unsigned i;

    for (i = 0; i < size; i++) {
        unsigned lane = (addr + i) & 0x3;

        val |= (uint64_t)((reg >> (8 * lane)) & 0xff) << (8 * (size - 1 - i));
    }
    return val;
}

static uint64_t ans_gc_breg1_read(void *opaque, hwaddr addr, unsigned size)
{
    AnsGCState *s = opaque;

    return ans_gc_breg_lanes(s->breg1_val, addr, size);
}

static uint64_t ans_gc_breg2_read(void *opaque, hwaddr addr, unsigned size)
{
    AnsGCState *s = opaque;

    return ans_gc_breg_lanes(s->breg2_val, addr, size);
}

static void ans_gc_breg_write(void *opaque, hwaddr addr, uint64_t val,
                              unsigned size)
{
    /* Strap inputs: the guest drives nothing. */
}


static const MemoryRegionOps ans_gc_control_ops = {
    .read = ans_gc_control_read,
    .write = ans_gc_control_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

static const MemoryRegionOps ans_gc_lcd_ops = {
    .read = ans_gc_lcd_read,
    .write = ans_gc_lcd_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

static const MemoryRegionOps ans_gc_nvctl_ops = {
    .read = ans_gc_nvctl_read,
    .write = ans_gc_nvctl_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

static const MemoryRegionOps ans_gc_nvdata_ops = {
    .read = ans_gc_nvdata_read,
    .write = ans_gc_nvdata_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 1 },
};

static const MemoryRegionOps ans_gc_eprom_ops = {
    .read = ans_gc_eprom_read,
    .write = ans_gc_eprom_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 2 },
};

static const MemoryRegionOps ans_gc_breg1_ops = {
    .read = ans_gc_breg1_read,
    .write = ans_gc_breg_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 2 },
};

static const MemoryRegionOps ans_gc_breg2_ops = {
    .read = ans_gc_breg2_read,
    .write = ans_gc_breg_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 2 },
};

static void ans_gc_realize(PCIDevice *d, Error **errp)
{
    AnsGCState *s = ANS_GC(d);
    static const uint8_t mac[3] = { 0x00, 0x05, 0x02 };  /* from the ROM's tree */
    static const uint8_t seq[3] = { 0x00, 0x05, 0x02 };
    uint8_t cx = 0;
    int i;

    /* The row's "blank store" is a board whose battery was pulled: an
     * erased part, not one that reads back zero. */
    if (s->nvram_erased) {
        memset(s->nvram, 0xff, sizeof(s->nvram));
    }
    pci_config_set_vendor_id(d->config, 0x106b);
    pci_config_set_device_id(d->config, 0x0002);
    pci_config_set_revision(d->config, 2);
    pci_set_word(d->config + PCI_CLASS_DEVICE, 0xff00);
    pci_set_byte(d->config + PCI_HEADER_TYPE, PCI_HEADER_TYPE_NORMAL);

    /*
     * The 1 MB register space.  This board hard-decodes Grand Central at
     * $F3000000 (dev notes 4.6.1: "configured by Open Firmware to live at
     * xF300000"), so the machine maps this region there directly rather than
     * going through a PCI BAR the ROM never assigns.
     */
    pci_set_long(d->config + PCI_BASE_ADDRESS_0, PCI_BASE_ADDRESS_MEM_TYPE_32);
    memory_region_init(&s->bar, OBJECT(d), "ans-gc-bar", 1 * MiB);

    /* Ethernet address PROM: six bytes, 0xAA, xor, then the inverses. */
    memset(s->eprom_data, 0, sizeof(s->eprom_data));
    for (i = 0; i < 3; i++) {
        s->eprom_data[i * 0x10] = mac[i];
        s->eprom_data[0x30 + i * 0x10] = seq[i];
        cx ^= mac[i] ^ seq[i];
    }
    s->eprom_data[0x60] = 0xaa;
    cx ^= 0xaa;
    s->eprom_data[0x70] = ~cx;
    for (i = 0; i < 8; i++) {
        s->eprom_data[0x80 + i * 0x10] = ~s->eprom_data[i * 0x10];
    }

    s->breg1_val = ANS_BREG1_STRAP;
    if (s->keyswitch == NULL || !strcmp(s->keyswitch, "normal")) {
        s->breg1_val |= ANS_BREG1_LOCKED | ANS_BREG1_SERVICE;
    } else if (!strcmp(s->keyswitch, "service")) {
        s->breg1_val |= ANS_BREG1_SERVICE;
    } else if (!strcmp(s->keyswitch, "locked")) {
        s->breg1_val |= ANS_BREG1_LOCKED;
    } else {
        error_setg(errp, "bad keyswitch position '%s'", s->keyswitch);
        return;
    }
    if (s->two_supplies) {
        s->breg1_val |= ANS_BREG1_TWO_PSU;
    }
    s->breg2_val = ANS_BREG2_HEALTHY;

    memory_region_init_io(&s->control, OBJECT(d), &ans_gc_control_ops, s,
                          "ans-gc-control", 0x1000);
    /*
     * The island's DMA engine at +0x8000: channel n sits at 0x8000 + n*0x100
     * (mac_dbdma.c's DBDMA_CHANNEL_SHIFT), the same addressing mac-io uses.
     */
    object_initialize_child(OBJECT(d), "dbdma", &s->dbdma, TYPE_MAC_DBDMA);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->dbdma), errp)) {
        return;
    }
    memory_region_init_io(&s->eprom, OBJECT(d), &ans_gc_eprom_ops, s,
                          "ans-gc-eprom", 0x100);
    memory_region_init_io(&s->breg1, OBJECT(d), &ans_gc_breg1_ops, s,
                          "ans-gc-breg1", 0x10);
    memory_region_init_io(&s->lcd, OBJECT(d), &ans_gc_lcd_ops, s,
                          "ans-gc-lcd", 0x40);
    memory_region_init_io(&s->nvram_ctl, OBJECT(d), &ans_gc_nvctl_ops, s,
                          "ans-gc-nvram-ctl", 0x10);
    memory_region_init_io(&s->breg2, OBJECT(d), &ans_gc_breg2_ops, s,
                          "ans-gc-breg2", 0x10);
    memory_region_init_io(&s->nvram_data, OBJECT(d), &ans_gc_nvdata_ops, s,
                          "ans-gc-nvram", 0x800);
    memory_region_init_io(&s->scc_legacy, OBJECT(d), &ans_gc_legacy_scc_ops, s,
                          "ans-gc-scc-legacy", 0x1000);

    /* The mac-io-family island children live on one bus of our own. */
    qbus_init(&s->escc_bus, sizeof(s->escc_bus), TYPE_MACIO_BUS, DEVICE(d),
              "ans-gc-bus");

    /*
     * The GC's register block at +0x0000 is the interrupt controller.  Its
     * output line (0) is the CPU's INT; its input lines are the island
     * sources, numbered per the Mac TNT map, so the islands below connect
     * with qdev_get_gpio_in(DEVICE(d), line).
     */
    qdev_init_gpio_out(DEVICE(d), &s->pic_out, 1);
    qdev_init_gpio_in(DEVICE(d), ans_gc_int_in, 64);
    memory_region_init_io(&s->intc, OBJECT(d), &ans_gc_int_ops, s,
                          "ans-gc-int", 0x1000);

    /*
     * The SCC island at +0x13000.  Open Firmware's console polls the byte at
     * +0x13020 (channel A RR0) for STATUS_TXEMPTY and writes its characters
     * to +0x13030, so the mac-io ESCC drops in with the same 16-byte-centre
     * stride; the machine's serial ports come out as ttya/ttyb here.
     */
    object_initialize_child(OBJECT(d), "escc", &s->escc, TYPE_ESCC);
    qdev_prop_set_uint32(DEVICE(&s->escc), "disabled", 0);
    qdev_prop_set_uint32(DEVICE(&s->escc), "frequency", ANS_GC_ESCC_CLOCK);
    qdev_prop_set_uint32(DEVICE(&s->escc), "it_shift", ANS_GC_ESCC_SHIFT);
    qdev_prop_set_uint32(DEVICE(&s->escc), "chnBtype", escc_serial);
    qdev_prop_set_uint32(DEVICE(&s->escc), "chnAtype", escc_serial);
    qdev_prop_set_chr(DEVICE(&s->escc), "chrA", serial_hd(0));
    qdev_prop_set_chr(DEVICE(&s->escc), "chrB", serial_hd(1));
    if (!qdev_realize(DEVICE(&s->escc), BUS(&s->escc_bus), errp)) {
        return;
    }
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->escc), 0,
                       qdev_get_gpio_in(DEVICE(d), ANS_GC_INT_SCCB));
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->escc), 1,
                       qdev_get_gpio_in(DEVICE(d), ANS_GC_INT_SCCA));
    /*
     * The ESCC's four serial DMA channels: A's tx/rx are DBDMA 4/5 and B's
     * 6/7, which is also what the firmware's ch-a node publishes
     * (`reg` = 0x8400/0x8500, `AAPL,interrupts` = $0F 4 5).
     */
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->escc), 2,
                       qdev_get_gpio_in(DEVICE(d), 6));
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->escc), 3,
                       qdev_get_gpio_in(DEVICE(d), 7));
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->escc), 4,
                       qdev_get_gpio_in(DEVICE(d), 4));
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->escc), 5,
                       qdev_get_gpio_in(DEVICE(d), 5));
    escc_register_dma(&s->escc, &s->dbdma);

    /*
     * The SCSI island at +0x10000 is the machine's narrow 53C94 (the
     * "53c94" node in Open Firmware's tree, aliased `scsi`).  Its first
     * register access is a write of CMD_BUSRESET to register 3, so the
     * generic ESP model drops straight in at the same 16-byte stride.
     */
    object_initialize_child(OBJECT(d), "esp", &s->esp, TYPE_SYSBUS_ESP);
    /* The ESP has no `it_shift` property; next-cube.c sets it the same way. */
    s->esp.it_shift = ANS_GC_ESCC_SHIFT;
    /*
     * AIX's hscsidd self-test sends Message accepted (0x12) with the chip
     * reset, which an ASC-mode gate would reject.
     */
    qdev_prop_set_bit(DEVICE(&s->esp), "cmd-any-state", true);
    if (!qdev_realize(DEVICE(&s->esp), BUS(&s->escc_bus), errp)) {
        return;
    }
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->esp), 0,
                       qdev_get_gpio_in(DEVICE(d), ANS_GC_INT_SCSI0));

    /*
     * The VIA island at +0x16000 (GC devices 6 and 7) is a 6522-style part
     * spanning the two 4 KB slots, exactly what the mac-io's CUDA/VIA model
     * covers; POST polls its registers heavily during "InitVia".
     */
    object_initialize_child(OBJECT(d), "cuda", &s->cuda, TYPE_CUDA);
    qdev_prop_set_uint64(DEVICE(&s->cuda), "timebase-frequency",
                         ANS_GC_TB_FREQ);
    /*
     * The ROM's CudaInit runs the full sync cycle and its driver reads each
     * response byte on an SR interrupt; the default Cuda transport provides
     * both.
     */
    if (!qdev_realize(DEVICE(&s->cuda), BUS(&s->escc_bus), errp)) {
        return;
    }
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->cuda), 0,
                       qdev_get_gpio_in(DEVICE(d), ANS_GC_INT_VIA));
    /*
     * The board's ADB keyboard and mouse hang off the Cuda, the same way the
     * mac-io machines attach theirs.  Without them the firmware's ADB probe
     * (Open Firmware's `adb-kbd` node and the `check-key-map` word) sends an
     * ADB packet, gets no reply, and spins on the VIA handshake forever.
     */
    {
        BusState *adb_bus = qdev_get_child_bus(DEVICE(&s->cuda), "adb.0");

        if (adb_bus) {
            qdev_realize_and_unref(qdev_new(TYPE_ADB_KEYBOARD), adb_bus, errp);
            qdev_realize_and_unref(qdev_new(TYPE_ADB_MOUSE), adb_bus, errp);
        }
    }

    memory_region_add_subregion(&s->bar, 0x00000,
                                &s->intc);
    memory_region_add_subregion(&s->bar, 0x08000,
                                sysbus_mmio_get_region(SYS_BUS_DEVICE(&s->dbdma),
                                                       0));
    memory_region_add_subregion(&s->bar, ANS_GC_OFF_SCSI,
                                sysbus_mmio_get_region(SYS_BUS_DEVICE(&s->esp),
                                                       0));
    memory_region_add_subregion(&s->bar, ANS_GC_OFF_SCC,
                                sysbus_mmio_get_region(SYS_BUS_DEVICE(&s->escc),
                                                       0));
    memory_region_add_subregion(&s->bar, ANS_GC_LEGACY_SCC, &s->scc_legacy);
    memory_region_add_subregion(&s->bar, 0x16000,
                                sysbus_mmio_get_region(SYS_BUS_DEVICE(&s->cuda),
                                                       0));
    memory_region_add_subregion(&s->bar, ANS_OFF_EPROM, &s->eprom);
    memory_region_add_subregion(&s->bar, ANS_OFF_BREG1, &s->breg1);
    memory_region_add_subregion(&s->bar, ANS_OFF_LCD, &s->lcd);
    memory_region_add_subregion(&s->bar, ANS_OFF_NVCTL, &s->nvram_ctl);
    memory_region_add_subregion(&s->bar, ANS_OFF_BREG2, &s->breg2);
    memory_region_add_subregion(&s->bar, ANS_OFF_NVDAT, &s->nvram_data);

    /* Log-only stand-ins for the device-island slots we have not modelled. */
    {
        static const unsigned slots[] = { 1, 4, 5, 8 };
        unsigned j;

        for (j = 0; j < ARRAY_SIZE(slots); j++) {
            MemoryRegion *mr = g_new0(MemoryRegion, 1);

            memory_region_init_io(mr, OBJECT(d), &ans_gc_slot_ops,
                                  (void *)(uintptr_t)slots[j],
                                  "ans-gc-unimpl", 0x1000);
            memory_region_add_subregion(&s->bar,
                                        0x10000 + slots[j] * 0x1000, mr);
        }
    }
}

static const VMStateDescription vmstate_ans_gc = {
    .name = "ans-gc",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8_ARRAY(nvram, AnsGCState, ANS_GC_NVRAM_SIZE),
        VMSTATE_UINT16(nvram_addr, AnsGCState),
        VMSTATE_UINT8(lcd_cmd, AnsGCState),
        VMSTATE_UINT8(lcd_data, AnsGCState),
        VMSTATE_END_OF_LIST()
    },
};

static const Property ans_gc_props[] = {
    DEFINE_PROP_STRING("keyswitch", AnsGCState, keyswitch),
    DEFINE_PROP_BOOL("two-supplies", AnsGCState, two_supplies, false),
    DEFINE_PROP_BOOL("nvram-erased", AnsGCState, nvram_erased, false),
};

static void ans_gc_class_init(ObjectClass *oc, const void *data)
{
    PCIDeviceClass *pc = PCI_DEVICE_CLASS(oc);
    DeviceClass *dc = DEVICE_CLASS(oc);

    pc->realize = ans_gc_realize;
    dc->vmsd = &vmstate_ans_gc;
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
    device_class_set_props(dc, ans_gc_props);
}

static const TypeInfo ans_gc_type_info = {
    .name = TYPE_ANS_GC,
    .parent = TYPE_PCI_DEVICE,
    .instance_size = sizeof(AnsGCState),
    .class_init = ans_gc_class_init,
    .interfaces = (InterfaceInfo[]) {
        { INTERFACE_CONVENTIONAL_PCI_DEVICE },
        { },
    },
};

static void ans_gc_register_types(void)
{
    type_register_static(&ans_gc_type_info);
}

type_init(ans_gc_register_types);
