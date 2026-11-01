/*
 * QEMU PCI S3 Trio (VGA compatible)
 *
 * Copyright (c) 2017 Hervé Poussineau
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>.
 */

/* S3 Trio is a very complex graphic card. Only some parts of them have
 * been implemented.
 */

#include "qemu/osdep.h"
#include "qemu/datadir.h"
#include "hw/pci/pci_device.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "ui/console.h"
#include "ui/pixel_ops.h"
#include "vga_int.h"
#include "hw/display/vga.h"
#include "hw/display/vga_regs.h"
#include "qom/object.h"
#include "trace.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "system/reset.h"
#include "system/memory.h"
#include "hw/core/loader.h"

#define TYPE_S3_TRIO "s3-trio"
#define TYPE_S3_V864 "s3-v864"

enum {
    REG_DISP_STAT      = 0x00,
    REG_H_DISP         = 0x01,
    REG_H_SYNC_START   = 0x02,
    REG_H_SYNC_WID     = 0x03,
    REG_V_TOTAL        = 0x04,
    REG_V_DISP         = 0x05,
    REG_V_SYNC_STRT    = 0x06,
    REG_V_SYNC_WID     = 0x07,
    REG_DISP_CNTL      = 0x08,
    REG_H_TOTAL        = 0x09,
    REG_SUBSYS_STAT    = 0x10, /* read-only */
    REG_SUBSYS_CNTL    = 0x10, /* write-only */
    REG_ROM_PAGE_SEL   = 0x11,
    REG_ADVFUNC_CNTL   = 0x12,
    REG_CUR_Y          = 0x20,
    REG_CUR_X          = 0x21,
    REG_DESTY_AXSTP    = 0x22,
    REG_DESTX_DIASTP   = 0x23,
    REG_ERR_TERM       = 0x24,
    REG_MAJ_AXIS_PCNT  = 0x25,
    REG_GP_STAT        = 0x26, /* read-only */
    REG_CMD            = 0x26, /* write-only */
    REG_SHORT_STROKE   = 0x27,
    REG_BKGD_COLOR     = 0x28,
    REG_FRGD_COLOR     = 0x29,
    REG_WRT_MASK       = 0x2A,
    REG_RD_MASK        = 0x2B,
    REG_COLOR_CMP      = 0x2C,
    REG_BKGD_MIX       = 0x2D,
    REG_FRGD_MIX       = 0x2E,
    REG_MULTIFUNC_CNTL = 0x2F,
    REG_PIX_TRANS      = 0x38,
};

enum {
    DISP_STAT_SENSE = 0x0001,
};

enum {
    GP_STAT_BUSY = 0x0200,
    GP_STAT_DATA_AVAIL = 0x0100,
    GP_STAT_FIFO_EMPTY = 0x0400,
};

enum {
    CMD_WRTDATA  = 0x0001,
    CMD_PLANAR   = 0x0002,
    CMD_LASTPIX  = 0x0004,
    CMD_LINETYPE = 0x0008,
    CMD_DRAW     = 0x0010,
    CMD_INC_X    = 0x0020,
    CMD_YMAJAXIS = 0x0040,
    CMD_INC_Y    = 0x0080,
    CMD_PCDATA   = 0x0100,
    CMD_16BIT    = 0x0200,
    CMD_32BIT    = 0x0400,
    CMD_BYTSEQ   = 0x1000,
};

#define CMD_CMD_MASK 0xE000
enum {
    CMD_CMD_NOP    = 0x0000,
    CMD_CMD_LINE   = 0x2000,
    CMD_CMD_RECT   = 0x4000,
    CMD_CMD_RECTV1 = 0x6000,
    CMD_CMD_RECTV2 = 0x8000,
    CMD_CMD_LINEAF = 0xA000,
    CMD_CMD_BITBLT = 0xC000,
    CMD_CMD_PATBLT = 0xE000,
};

#define BKGD_MIX_BSS_MASK 0x0060
enum {
    BKGD_MIX_BSS_BKGD = 0x0000,
    BKGD_MIX_BSS_FRGD = 0x0020,
    BKGD_MIX_BSS_PIX  = 0x0040,
    BKGD_MIX_BSS_BMP  = 0x0060,
};

#define FRGD_MIX_FSS_MASK 0x0060
enum {
    FRGD_MIX_FSS_BKGD = 0x0000,
    FRGD_MIX_FSS_FRGD = 0x0020,
    FRGD_MIX_FSS_PIX  = 0x0040,
    FRGD_MIX_FSS_BMP  = 0x0060,
};

#define PIX_CNTL_MIXSEL_MASK 0x00C0
enum {
    PIX_CNTL_MIXSEL_FOREMIX = 0x0000,
    PIX_CNTL_MIXSEL_PATTERN = 0x0040,
    PIX_CNTL_MIXSEL_VAR     = 0x0080,
    PIX_CNTL_MIXSEL_TRANS   = 0x00C0,
};

typedef struct S3TrioState {
    PCIDevice dev;
    VGACommonState vga;
    MemoryRegion rom;
    MemoryRegion rom_bar;
    MemoryRegion vram_iomem;
    MemoryRegion mmio_window;
    uint16_t maj_axis, min_axis;
    PortioList portio;
    bool crtc_seq_shared;

    uint32_t dclk;
    uint32_t mclk;

    uint32_t device_id;
    bool mmio_window_mapped;

    uint16_t disp_stat; /* 02e8 */
    uint16_t h_disp; /* 06e8 */
    uint16_t h_sync_strt; /* 0ae8 */
    uint16_t h_sync_wid; /* 0ee8 */
    uint16_t v_total; /* 12e8 */
    uint16_t v_disp; /* 16e8 */
    uint16_t v_sync_strt; /* 1ae8 */
    uint16_t v_sync_wid; /* 1ee8 */
    uint16_t disp_cntl; /* 22e8 */
    uint16_t h_total; /* 26e8 */
    uint16_t subsys_cntl; /* 42e8 (W) */
    uint16_t subsys_stat; /* 42e8 (R) */
    uint16_t rom_page_sel; /* 46e8 */
    uint16_t advfunc_cntl; /* 4ae8 */
    /* the generic line offset omits the S3 CR51 extension bits */
    void (*base_get_params)(VGACommonState *vga, VGADisplayParams *params);
    uint16_t cur_y; /* 82e8 */
    uint16_t cur_x; /* 86e8 */
    uint16_t desty_axstep; /* 8ae8 */
    uint16_t destx_diastp; /* 8ee8 */
    uint16_t err_term; /* 92e8 */
    uint16_t maj_axis_pcnt; /* 96e8 */
    uint16_t gp_stat; /* 9ae8 (R) */
    uint16_t cmd; /* 9ae8 (W) */
    uint16_t short_stroke; /* 9ee8 */
    uint16_t bkgd_color; /* a2e8 */
    uint16_t frgd_color; /* a6e8 */
    uint16_t wrt_mask; /* aae8 */
    uint16_t rd_mask; /* aee8 */
    uint16_t color_cmp; /* b2e8 */
    uint16_t bkgd_mix; /* b6e8 */
    uint16_t frgd_mix; /* bae8 */
    uint16_t mfc[16]; /* bee8 */
    uint16_t pix_trans; /* e2e8 */
    uint16_t pix_trans_ext; /* e2ea (image data extension, 32-bit transfers) */
    uint32_t xfer_pixel;    /* CPU pixel data received so far */
    int xfer_pixel_len;
    uint16_t xfer_lo;       /* low byte held for a pending 16-bit CPU transfer */
    bool xfer_lo_valid;

    /* S3 hardware graphics cursor */
    uint8_t cursor_fg[4];
    uint8_t cursor_bg[4];
    uint8_t cursor_color_pos;
    int last_cursor_x;
    int last_cursor_y;
    bool last_cursor_on;
    uint32_t last_cursor_hash;
    uint32_t last_cursor_addr;

    uint8_t origin_x;
    uint8_t origin_y;
    uint8_t unlock_pll;
    uint8_t unlock_compatibility_registers;
    uint8_t unlock_control_registers_1;
    uint8_t unlock_control_registers_2;
} S3TrioState;

OBJECT_DECLARE_SIMPLE_TYPE(S3TrioState, S3_TRIO)

/* FIXME: remove forward declarations */
static uint32_t get_color_from_mix(S3TrioState *s, uint16_t mix);
static uint32_t get_current_destination_bitmap(S3TrioState *s);
static uint32_t s3_mix_op(unsigned op, uint32_t src, uint32_t dst,
                          uint32_t mask);
static uint32_t s3_pixel_data_mix(S3TrioState *s, uint32_t cpu);
static void s3_pixel_data_write(S3TrioState *s, unsigned offset, uint64_t val,
                                unsigned size);
static void s3_pixel_data_write32(S3TrioState *s, uint32_t ext);
static uint32_t raster_op(S3TrioState *s, uint16_t mix);
static uint32_t get_color(S3TrioState *s);
static bool s3_cursor_enabled(S3TrioState *s);
static uint32_t s3_cursor_addr(S3TrioState *s);

#define min_axis_pcnt mfc[0]
#define scissors_t    mfc[1]
#define scissors_l    mfc[2]
#define scissors_b    mfc[3]
#define scissors_r    mfc[4]
#define mem_cntl      mfc[5]
#define pattern_l     mfc[8]
#define pattern_h     mfc[9]
#define pix_cntl      mfc[10]
#define color_compare mfc[14]

/* CR50 bits 5-4 select 1, 2 or 4 bytes per pixel (DB008-B table 9-3). */
static int s3_pixel_bytes(S3TrioState *s)
{
    switch ((s->vga.cr[0x50] >> 4) & 3) {
    case 1:
        return 2;
    case 3:
        return 4;
    default:
        return 1;
    }
}

/* 2-byte pixels are 5:6:5 or 5:5:5, selected by CR67 bits 7-4
 * (CR43 bit 3 on the 928). */
static bool s3_pixel_565(S3TrioState *s)
{
    unsigned pa16 = (s->vga.cr[0x67] >> 4) & 0xf;

    if (pa16 == 0x5 || pa16 == 0x6) {
        return true;
    }
    if (pa16 == 0x2 || pa16 == 0x3) {
        return false;
    }
    return (s->vga.cr[0x43] & 0x08) || pa16 == 0;
}

/* Colour depth reported to the VGA core (VGA_DRAW_LINE* selector). */
static int s3_pixel_depth(S3TrioState *s)
{
    switch (s3_pixel_bytes(s)) {
    case 2:
        return s3_pixel_565(s) ? 16 : 15;
    case 4:
        return 32;
    default:
        return 8;
    }
}

/* Packed pixels require enhanced mode: CR3A bit 4 or ADVFUNC_CNTL bit 0. */
static bool s3_enhanced_mode(S3TrioState *s)
{
    return (s->vga.cr[0x3a] & 0x10) || (s->advfunc_cntl & 1);
}

static uint32_t s3_pixel_mask(S3TrioState *s)
{
    switch (s3_pixel_bytes(s)) {
    case 2:
        return 0xffff;
    case 4:
        return 0xffffffffu;
    default:
        return 0xff;
    }
}

/* Display memory holds pixels in the byte order the CPU provides (LE). */
static uint32_t s3_read_pixel(S3TrioState *s, uint32_t offset)
{
    int bpp = s3_pixel_bytes(s);
    uint32_t value = 0;
    int i;

    for (i = 0; i < bpp; i++) {
        value |= (uint32_t)s->vga.vram_ptr[offset + i] << (8 * i);
    }
    return value;
}

static void s3_write_pixel(S3TrioState *s, uint32_t offset, uint32_t value)
{
    int bpp = s3_pixel_bytes(s);
    int i;

    for (i = 0; i < bpp; i++) {
        s->vga.vram_ptr[offset + i] = (value >> (8 * i)) & 0xff;
    }
    memory_region_set_dirty(&s->vga.vram, offset, bpp);
}

static uint32_t s3_pixel_offset(S3TrioState *s, int x, int y, int *width)
{
    int w, h;

    s->vga.get_resolution(&s->vga, &w, &h);
    if (width) {
        *width = w;
    }
    return ((uint32_t)y * w + x) * s3_pixel_bytes(s);
}

static inline int address_to_reg(uint32_t addr)
{
    assert((addr & 0x3ff) == 0x2e8);
    return addr >> 10;
}

static inline uint32_t reg_to_address(int reg)
{
    return (reg << 10) + 0x2e8;
}

/*
 * V864 specific feature:
 * shadowed extended CRTC registers into the sequencer registers
 * 0x1/0x7/0x12/0x42/0x5D/0x5E
 */
static bool s3_crtc_seq_shared(S3TrioState *s, unsigned int idx)
{
    if (s->crtc_seq_shared) {
        switch (idx) {
            case 0x01: /* CR1: horizontal display end */
            case 0x07: /* CR7: vertical overflow */
            case 0x12: /* CR12: vertical display end (low) */
            case 0x42: /* CR42: mode control */
            case 0x5d: /* CR5D: horizontal overflow */
            case 0x5e: /* CR5E: vertical overflow high */
                return true;
        }
    }
    return false;
}

static inline void do_cmd_done(S3TrioState *s)
{
    s->gp_stat = GP_STAT_FIFO_EMPTY;
}

static void move_to_next_pixel(S3TrioState *s)
{
    uint16_t line_len;
    int dx;
    int dy;

    switch (s->cmd & CMD_CMD_MASK) {
    case CMD_CMD_RECT:
        /* The rectangle is (MAJ_AXIS_PCNT+1) x (MIN_AXIS_PCNT+1) pixels. */
        line_len = s->maj_axis_pcnt + 1;
        dx = s->cmd & CMD_INC_X ? 1 : -1;
        dy = s->cmd & CMD_INC_Y ? 1 : -1;
        ++s->maj_axis;
        if (s->maj_axis < line_len) {
            s->cur_x += dx;
        } else {
            s->maj_axis = 0;
            s->cur_x -= (line_len - 1) * dx;
            s->cur_y += dy;
            s->min_axis++;
            if (s->min_axis == s->min_axis_pcnt + 1) {
                do_cmd_done(s);
            }
        }
        break;
    case CMD_CMD_LINE:
        if ((s->cmd & CMD_LINETYPE) == 0) {
            assert(0);
        } else {
            static const int xstep[] = { 1,  1,  0, -1, -1, -1, 0, 1 };
            static const int ystep[] = { 0, -1, -1, -1,  0,  1, 1, 1 };
            s->cur_x += xstep[(s->cmd >> 5) & 7];
            s->cur_y += ystep[(s->cmd >> 5) & 7];
            if (s->maj_axis_pcnt-- == 0) {
                do_cmd_done(s);
            }
        }
        break;
    default:
        assert(0);
        break;
    }
}

static void do_cmd_write_one_pixel(S3TrioState *s, uint32_t value)
{
    uint32_t offset;

    if (s->color_compare & 0x100) {
        uint32_t cmp = s->color_cmp & s3_pixel_mask(s);

        if ((s->color_compare & 0x80) == 0x80 &&
            cmp != (value & s3_pixel_mask(s))) {
            return;
        } else if ((s->color_compare & 0x80) == 0x00 &&
                   cmp == (value & s3_pixel_mask(s))) {
            return;
        }
    }

    if ((s->maj_axis < s->maj_axis_pcnt) ||
        (s->maj_axis == s->maj_axis_pcnt && !(s->cmd & CMD_LASTPIX))) {
        int bpp = s3_pixel_bytes(s);

        offset = s3_pixel_offset(s, s->cur_x, s->cur_y, NULL);
        trace_s3_vga_pixel(offset, value, s->cur_x, s->cur_y, s->maj_axis,
                           s->maj_axis_pcnt, s->min_axis, s->mfc[0],
                           s->gp_stat, s->cmd);
        if (s3_cursor_enabled(s) && offset < s->vga.vram_size) {
            uint32_t caddr = s3_cursor_addr(s);

            if (offset < caddr + 1024 && offset + bpp > caddr) {
                trace_s3_vga_cursor_pixel(offset, value, s->cur_x, s->cur_y);
            }
        }
        if (offset >= s->vga.vram_size ||
            offset + bpp > s->vga.vram_size) {
            return;
        }
        s3_write_pixel(s, offset, value);
    }
}

static void do_cmd_write_pixel(S3TrioState *s, uint32_t value, int data_bits)
{
    int bpp = s3_pixel_bytes(s);
    int i, size;
    uint32_t color;

    if (!(s->gp_stat & (GP_STAT_BUSY | GP_STAT_DATA_AVAIL))) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "s3_trio: %s called with no command expecting data\n",
                      __func__);
        return;
    }

    if (!(s->cmd & CMD_PCDATA)) {
        s->gp_stat |= GP_STAT_BUSY;
    }

    if (s->cmd & CMD_PLANAR) {
        trace_s3_vga_planar(value, s->cur_x, s->cur_y, s->maj_axis_pcnt,
                            s->min_axis_pcnt, s->cmd, s->frgd_mix,
                            s->bkgd_mix);
        /* Mask bits are consumed in the order written, MSB first
         * (DB008-B 10-9); bit 12 reverses the bytes. */
        size = data_bits;
        for (i = 0; i < size; i++) {
            int min_axis = s->min_axis;
            unsigned nbytes = size / 8;
            unsigned byteidx = (s->cmd & CMD_BYTSEQ) ?
                (i / 8) : (nbytes - 1 - i / 8);
            uint8_t byte = (value >> (8 * byteidx)) & 0xff;

            if (byte & (1u << (7 - (i % 8)))) {
                color = raster_op(s, s->frgd_mix);
            } else {
                color = raster_op(s, s->bkgd_mix);
            }
            do_cmd_write_one_pixel(s, color);
            move_to_next_pixel(s);
            if (!(s->gp_stat & (GP_STAT_BUSY | GP_STAT_DATA_AVAIL))) {
                break;
            }
            if (s->min_axis != min_axis) {
                break;
            }
        }
    } else {
        /* The engine deposits the transfer MSB first; bit 12 reverses a
         * unit's bytes, and one pixel may take several transfers. */
        size = data_bits / 8;
        for (i = 0; i < size; i++) {
            int lane = (s->cmd & CMD_BYTSEQ) ? (size - i - 1) : i;
            uint8_t byte = (value >> (8 * (size - lane - 1))) & 0xff;
            int min_axis = s->min_axis;

            s->xfer_pixel |= (uint32_t)byte << (8 * s->xfer_pixel_len);
            s->xfer_pixel_len++;
            if (s->xfer_pixel_len < bpp) {
                continue;
            }
            do_cmd_write_one_pixel(s, s3_pixel_data_mix(s, s->xfer_pixel));
            s->xfer_pixel = 0;
            s->xfer_pixel_len = 0;
            move_to_next_pixel(s);
            if (!(s->gp_stat & (GP_STAT_BUSY | GP_STAT_DATA_AVAIL))) {
                break;
            }
            /* Line data is padded to a whole transfer unit; the leftover
             * pixels of that unit are dropped at the line end (DB008-B 10-9). */
            if (s->min_axis != min_axis) {
                break;
            }
        }
    }
}

/* Engine-generated pixels: the mix colour is already the memory value. */
static void do_cmd_write_engine_pixel(S3TrioState *s)
{
    uint32_t color;

    if (!(s->gp_stat & (GP_STAT_BUSY | GP_STAT_DATA_AVAIL))) {
        return;
    }
    if (s->cmd & CMD_PLANAR) {
        /* across-the-plane fill with no CPU data: mask bits are all ones */
        color = raster_op(s, s->frgd_mix);
    } else {
        color = get_color(s);
    }
    do_cmd_write_one_pixel(s, color);
    move_to_next_pixel(s);
}

/* A 32-bit transfer arrives as E2E8 (low half) then E2EA (high half);
 * the extension write is what completes it (DB008-B 10-9). */
static void s3_pixel_data_write32(S3TrioState *s, uint32_t ext)
{
    s3_pixel_data_write(s, 0, ((uint32_t)ext << 16) | s->pix_trans, 4);
}

/* CPU pixel data for an "across the plane" (mask) command */
static void s3_pixel_data_write(S3TrioState *s, unsigned offset, uint64_t val,
                                unsigned size)
{
    unsigned i;

    if ((s->cmd & CMD_PLANAR) && (s->cmd & CMD_16BIT)) {
        for (i = 0; i < size; i++) {
            unsigned lane = (offset + i) & 1;
            uint8_t byte = (val >> (8 * i)) & 0xff;

            if (lane == 0) {
                s->xfer_lo = byte;
                s->xfer_lo_valid = true;
            } else {
                uint32_t unit = (s->xfer_lo_valid ? s->xfer_lo : 0) |
                                (byte << 8);

                do_cmd_write_pixel(s, unit, 16);
            }
        }
        return;
    }

    do_cmd_write_pixel(s, val, size * 8);
}

static uint32_t s3_mix_op(unsigned op, uint32_t src, uint32_t dst,
                          uint32_t mask)
{
    switch (op) {
    case 0x00: return ~dst & mask;
    case 0x01: return 0;
    case 0x02: return mask;
    case 0x03: return dst;
    case 0x04: return ~src & mask;
    case 0x05: return (src ^ dst) & mask;
    case 0x06: return ~(src ^ dst) & mask;
    case 0x07: return src;
    case 0x08: return ~(src & dst) & mask;
    case 0x09: return (~src | dst) & mask;
    case 0x0a: return (src | ~dst) & mask;
    case 0x0b: return (src | dst) & mask;
    case 0x0c: return (src & dst) & mask;
    case 0x0d: return (src & ~dst) & mask;
    case 0x0e: return (~src & dst) & mask;
    case 0x0f: return ~(src | dst) & mask;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "s3_trio: invalid MIX operation 0x%x\n",
                      op);
        return src;
    }
}

/* Apply a color mix (foreground or background) to an explicit src/dst pair. */
static uint32_t s3_mix_pixel(S3TrioState *s, uint16_t mix, uint32_t src,
                             uint32_t dst)
{
    uint32_t color, mask = s3_pixel_mask(s);

    switch (mix & FRGD_MIX_FSS_MASK) {
    case FRGD_MIX_FSS_BKGD:
        color = s->bkgd_color & mask;
        break;
    case FRGD_MIX_FSS_FRGD:
        color = s->frgd_color & mask;
        break;
    case FRGD_MIX_FSS_PIX:
        color = s->pix_trans & mask;
        break;
    case FRGD_MIX_FSS_BMP:
    default:
        color = src & mask;
        break;
    }

    return s3_mix_op(mix & 0x0f, color & mask, dst & mask, mask);
}

/* The MIXSEL register supplies the colour source for through-the-plane data
 * (DB008-B 10-13, 10-17). */
static uint32_t s3_pixel_data_mix(S3TrioState *s, uint32_t cpu)
{
    uint32_t mask = s3_pixel_mask(s);
    uint16_t mix = s->frgd_mix;
    uint32_t dst, color;

    if ((s->pix_cntl & PIX_CNTL_MIXSEL_MASK) == PIX_CNTL_MIXSEL_TRANS) {
        /* The destination's read-enabled planes select the mix register. */
        dst = get_current_destination_bitmap(s);
        if ((dst & s->rd_mask) != (s->rd_mask & mask)) {
            mix = s->bkgd_mix;
        }
    } else if ((s->pix_cntl & PIX_CNTL_MIXSEL_MASK) != PIX_CNTL_MIXSEL_FOREMIX) {
        qemu_log_mask(LOG_UNIMP, "s3_trio: pixel transfer MIXSEL 0x%x\n",
                      (s->pix_cntl & PIX_CNTL_MIXSEL_MASK) >> 6);
    }

    switch (mix & FRGD_MIX_FSS_MASK) {
    case FRGD_MIX_FSS_BKGD:
        color = s->bkgd_color;
        break;
    case FRGD_MIX_FSS_FRGD:
        color = s->frgd_color;
        break;
    case FRGD_MIX_FSS_PIX:
    default:
        color = cpu;
        break;
    }

    dst = get_current_destination_bitmap(s);
    return s3_mix_op(mix & 0x0f, color & mask, dst & mask, mask);
}

static void s3_do_cmd_patblt(S3TrioState *s)
{
    int bpp = s3_pixel_bytes(s);
    int width = s->maj_axis_pcnt + 1;
    int height = s->min_axis_pcnt + 1;
    int dx = (s->cmd & CMD_INC_X) ? 1 : -1;
    int dy = (s->cmd & CMD_INC_Y) ? 1 : -1;
    int src_x = s->cur_x;
    int src_y = s->cur_y;
    int dst_x = s->destx_diastp;
    int dst_y = s->desty_axstep;
    int res_width, res_height;
    int x, y;

    s->vga.get_resolution(&s->vga, &res_width, &res_height);

    for (y = 0; y < height; y++) {
        for (x = 0; x < width; x++) {
            int px = dst_x + dx * x;
            int py = dst_y + dy * y;
            int sx = src_x + ((dst_x + dx * x) & 7);
            int sy = src_y + ((dst_y + dy * y) & 7);
            uint32_t src_off, dst_off;
            uint32_t src, dst, out;

            if (px < 0 || py < 0 || px >= res_width || py >= res_height) {
                continue;
            }
            /*
             * The pattern is usually kept in off-screen display memory, so
             * only screen bounds apply to the destination; the source may be
             * outside the visible area.
             */
            src_off = s3_pixel_offset(s, sx, sy, NULL);
            dst_off = s3_pixel_offset(s, px, py, NULL);
            if (sx < 0 || sx >= res_width || sy < 0 ||
                src_off + bpp > s->vga.vram_size ||
                dst_off + bpp > s->vga.vram_size) {
                continue;
            }
            src = s3_read_pixel(s, src_off);
            dst = s3_read_pixel(s, dst_off);

            switch (s->pix_cntl & PIX_CNTL_MIXSEL_MASK) {
            case PIX_CNTL_MIXSEL_TRANS: {
                uint32_t mask = s->rd_mask ? s->rd_mask : s3_pixel_mask(s);
                bool use_fg = ((src & mask) == mask) || !mask;

                out = s3_mix_pixel(s, use_fg ? s->frgd_mix : s->bkgd_mix,
                                   src, dst);
                break;
            }
            case PIX_CNTL_MIXSEL_FOREMIX:
                out = s3_mix_pixel(s, s->frgd_mix, src, dst);
                break;
            default:
                qemu_log_mask(LOG_UNIMP,
                              "s3_trio: PatBLT MIXSEL 0x%x unimplemented\n",
                              (s->pix_cntl & PIX_CNTL_MIXSEL_MASK) >> 6);
                out = s3_mix_pixel(s, s->frgd_mix, src, dst);
                break;
            }

            s3_write_pixel(s, dst_off, out);
        }
    }
}

static uint32_t get_current_source_bitmap(S3TrioState *s)
{
    qemu_log_mask(LOG_UNIMP,
                  "s3_trio: unimplemented source operand BMP\n");
    return 0;
}

static uint32_t get_current_destination_bitmap(S3TrioState *s)
{
    int bpp = s3_pixel_bytes(s);
    uint32_t off = s3_pixel_offset(s, s->cur_x, s->cur_y, NULL);

    if (off >= s->vga.vram_size || off + bpp > s->vga.vram_size) {
        return 0;
    }
    return s3_read_pixel(s, off);
}

static uint32_t get_color_from_mix(S3TrioState *s, uint16_t mix)
{
    switch (mix & FRGD_MIX_FSS_MASK) {
    case FRGD_MIX_FSS_BKGD:
        return s->bkgd_color;
    case FRGD_MIX_FSS_FRGD:
        return s->frgd_color;
    case FRGD_MIX_FSS_PIX:
        return s->pix_trans;
    case FRGD_MIX_FSS_BMP:
        return get_current_source_bitmap(s);
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "s3_trio: invalid FSS 0x%x\n",
                      (s->frgd_mix & FRGD_MIX_FSS_MASK) >> 5);
        return 0;
    }
}

static uint32_t raster_op(S3TrioState *s, uint16_t mix)
{
    uint32_t src = get_color_from_mix(s, mix);
    uint32_t dst = get_current_destination_bitmap(s);
    uint32_t mask = s3_pixel_mask(s);

    return s3_mix_op(mix & 0x0f, src & mask, dst & mask, mask);
}

static uint32_t get_foreground_color(S3TrioState *s)
{
    return raster_op(s, s->frgd_mix);
}

#if 0
static uint16_t get_background_color(S3TrioState *s)
{
    return raster_op(s, s->bkgd_mix);
}
#endif

static uint32_t get_color(S3TrioState *s)
{
    assert(!(s->cmd & CMD_PCDATA));

    if (s->cmd & CMD_PLANAR) {
        return 0xffff;
    }

    switch (s->pix_cntl & PIX_CNTL_MIXSEL_MASK) {
    case PIX_CNTL_MIXSEL_FOREMIX:
        return get_foreground_color(s);
    case PIX_CNTL_MIXSEL_PATTERN:
        qemu_log_mask(LOG_UNIMP, "s3_trio: unimplemented mixel PATTERN\n");
        return 0;
    case PIX_CNTL_MIXSEL_VAR:
        qemu_log_mask(LOG_UNIMP, "s3_trio: unimplemented mixel VAR\n");
        return 0;
    case PIX_CNTL_MIXSEL_TRANS:
        qemu_log_mask(LOG_UNIMP, "s3_trio: unimplemented mixel TRANS\n");
        return 0;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "s3_trio: invalid MIXSEL 0x%x\n",
                      (s->pix_cntl & PIX_CNTL_MIXSEL_MASK) >> 6);
        return 0;
    }
}

static void s3_do_cmd_bitblt(S3TrioState *s)
{
    int bpp = s3_pixel_bytes(s);
    int xs = (s->cmd & CMD_INC_X) ? 1 : -1;
    int ys = (s->cmd & CMD_INC_Y) ? 1 : -1;
    int x1 = s->cur_x;
    int y1 = s->cur_y;
    int x2 = s->destx_diastp;
    int y2 = s->desty_axstep;
    int width = s->maj_axis_pcnt + 1;
    int height = s->min_axis_pcnt + 1;
    int x, y;

    for (y = 0; y < height; y++) {
        for (x = 0; x < width; x++) {
            int sx = x1 + x * xs;
            int sy = y1 + y * ys;
            int dx = x2 + x * xs;
            int dy = y2 + y * ys;
            uint32_t src_off = s3_pixel_offset(s, sx, sy, NULL);
            uint32_t dst_off = s3_pixel_offset(s, dx, dy, NULL);
            uint16_t mix = s->frgd_mix;
            uint32_t src, dst;

            if (sx < 0 || sy < 0 || dx < 0 || dy < 0 ||
                src_off >= s->vga.vram_size ||
                dst_off >= s->vga.vram_size ||
                src_off + bpp > s->vga.vram_size ||
                dst_off + bpp > s->vga.vram_size) {
                continue;
            }
            src = s3_read_pixel(s, src_off);
            dst = s3_read_pixel(s, dst_off);
            s3_write_pixel(s, dst_off, s3_mix_pixel(s, mix, src, dst));
        }
    }
}

static void do_cmd_init(S3TrioState *s)
{
    s->gp_stat = GP_STAT_FIFO_EMPTY;
    if (!(s->cmd & CMD_PCDATA)) {
        s->gp_stat |= GP_STAT_BUSY;
    }
    s->maj_axis = 0;
    s->min_axis = 0;
}

static void do_cmd(S3TrioState *s)
{
    trace_s3_vga_cmd(s->cmd);
    trace_s3_vga_cmd_full(s->cmd, s->cur_x, s->cur_y, s->destx_diastp,
                          s->desty_axstep, s->maj_axis_pcnt, s->min_axis_pcnt,
                          s->pix_cntl);
    trace_s3_vga_mix_state(s->frgd_mix, s->bkgd_mix, s->rd_mask, s->wrt_mask,
                           s->frgd_color, s->bkgd_color, s->color_cmp,
                           s->color_compare);

    s->xfer_lo_valid = false;
    s->xfer_pixel = 0;
    s->xfer_pixel_len = 0;

    do_cmd_init(s);

    if (s->cmd & CMD_PCDATA) {
        s->gp_stat |= GP_STAT_DATA_AVAIL;
    }

    if ((s->cmd & CMD_WRTDATA) == 0) {
        qemu_log_mask(LOG_UNIMP,
                      "s3_trio: CMD_WRTDATA=0 not implemented (%04x)\n", s->cmd);
    }

    switch (s->cmd & CMD_CMD_MASK) {
    case CMD_CMD_NOP:
        qemu_log_mask(LOG_UNIMP, "s3_trio: CMD_NOP not implemented (%04x)\n",
                      s->cmd);
        do_cmd_done(s);
        break;
    case CMD_CMD_LINE:
        if ((s->cmd & CMD_LINETYPE) == 0) {
            trace_s3_vga_cmd_line_bresenham(s->cur_x, s->cur_y,
                                            s->cmd & CMD_INC_X ? 1 : -1,
                                            s->cmd & CMD_INC_Y ? 1 : -1,
                                            s->maj_axis_pcnt,
                                            s->cmd & CMD_YMAJAXIS ? 'Y' : 'X');
            qemu_log_mask(LOG_UNIMP,
                          "s3_trio: CMD_LINE (Bresenham) not implemented (%04x)\n",
                          s->cmd);
            do_cmd_done(s);
        } else {
            trace_s3_vga_cmd_line_vector(s->cur_x, s->cur_y, (s->cmd >> 5) & 7,
                                         s->maj_axis_pcnt);
            if (!(s->cmd & CMD_PCDATA)) {
                while (s->gp_stat & GP_STAT_BUSY) {
                    do_cmd_write_engine_pixel(s);
                }
            }
        }
        break;
    case CMD_CMD_RECT:
        trace_s3_vga_cmd_rect(s->cur_x, s->cur_y, s->cmd & CMD_INC_X ? 1 : -1,
                              s->cmd & CMD_INC_Y ? 1 : -1, s->maj_axis_pcnt,
                              s->min_axis_pcnt);
        if (!(s->cmd & CMD_PCDATA)) {
            while (s->gp_stat & GP_STAT_BUSY) {
                do_cmd_write_engine_pixel(s);
            }
        }
        break;
    case CMD_CMD_RECTV1:
        qemu_log_mask(LOG_UNIMP, "s3_trio: CMD_RECTV1 not implemented (%04x)\n",
                      s->cmd);
        do_cmd_done(s);
        break;
    case CMD_CMD_RECTV2:
        qemu_log_mask(LOG_UNIMP, "s3_trio: CMD_RECTV2 not implemented (%04x)\n",
                      s->cmd);
        do_cmd_done(s);
        break;
    case CMD_CMD_LINEAF:
        qemu_log_mask(LOG_UNIMP, "s3_trio: CMD_LINEAF not implemented (%04x)\n",
                      s->cmd);
        do_cmd_done(s);
        break;
    case CMD_CMD_BITBLT:
        trace_s3_vga_cmd_bitblt(s->cur_x, s->cur_y, s->destx_diastp, s->desty_axstep,
                                s->maj_axis_pcnt, s->min_axis_pcnt);
        s3_do_cmd_bitblt(s);
        do_cmd_done(s);
        break;
    case CMD_CMD_PATBLT:
        s3_do_cmd_patblt(s);
        do_cmd_done(s);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "s3_trio: illegal command %04x\n",
                      s->cmd);
        do_cmd_done(s);
        break;
    }
}

static uint32_t s3_trio_enable_readb(void *opaque, uint32_t addr)
{
    uint32_t val;
    val = 0;
    trace_s3_vga_enable_readb(addr, val);
    return val;
}

static void s3_trio_enable_writeb(void *opaque, uint32_t addr, uint32_t val)
{
    trace_s3_vga_enable_writeb(addr, val);
}

static uint32_t s3_trio_dac_ioport_readb(void *opaque, uint32_t addr)
{
    S3TrioState *s = opaque;
    uint32_t val;

    val = vga_ioport_read(&s->vga, addr - 0x2ea + VGA_PEL_MSK);
    trace_s3_vga_dac_readb(addr, val);
    return val;
}

static void s3_trio_dac_ioport_writeb(void *opaque, uint32_t addr, uint32_t val)
{
    S3TrioState *s = opaque;
    trace_s3_vga_dac_writeb(addr, val);
    /*
     * V864's RAMDAC is 8 bits per channel, but VGA-compatible software
     * writes 6-bit palette values and expects the RAMDAC expands.  Take
     * the two conventions apart from the values themselves by 0x3f.
     */
    if (addr == 0x2ec) {
        s->vga.dac_8bit = 0;
    } else if (addr == 0x2ed && val > 0x3f) {
        s->vga.dac_8bit = 1;
    }
    vga_ioport_write(&s->vga, addr - 0x2ea + VGA_PEL_MSK, val);
}

static uint16_t* s3_trio_get_register(S3TrioState *s, uint32_t addr, int is_write, uint32_t* val_if_write)
{
    uint16_t *p;

    switch (addr) {
    case REG_DISP_STAT:
        p = is_write ? &s->h_total : &s->disp_stat;
        break;
    case REG_H_DISP:
        p = is_write ? &s->h_disp : NULL;
        break;
    case REG_H_SYNC_START:
        p = is_write ? &s->h_sync_strt : NULL;
        break;
    case REG_H_SYNC_WID:
        p = is_write ? &s->h_sync_wid : NULL;
        break;
    case REG_V_TOTAL:
        p = is_write ? &s->v_total : NULL;
        break;
    case REG_V_DISP:
        p = is_write ? &s->v_disp : NULL;
        break;
    case REG_V_SYNC_STRT:
        p = is_write ? &s->v_sync_strt : NULL;
        break;
    case REG_V_SYNC_WID:
        p = is_write ? &s->v_sync_wid : NULL;
        break;
    case REG_DISP_CNTL:
        p = is_write ? &s->disp_cntl : NULL;
        break;
    case REG_H_TOTAL:
        p = is_write ? NULL: &s->h_total;
        break;
    case REG_SUBSYS_STAT: /* or REG_SUBSYS_CNTL */
        p = is_write ? &s->subsys_cntl : &s->subsys_stat;
        break;
    case REG_ROM_PAGE_SEL:
        p = is_write ? &s->rom_page_sel : NULL;
        break;
    case REG_ADVFUNC_CNTL:
        p = is_write ? &s->advfunc_cntl : NULL;
        break;
    case REG_CUR_Y:
        p = &s->cur_y;
        break;
    case REG_CUR_X:
        p = &s->cur_x;
        break;
    case REG_DESTY_AXSTP:
        p = is_write ? &s->desty_axstep : NULL;
        break;
    case REG_DESTX_DIASTP:
        p = is_write ? &s->destx_diastp : NULL;
        break;
    case REG_ERR_TERM:
        p = &s->err_term;
        break;
    case REG_MAJ_AXIS_PCNT:
        p = is_write ? &s->maj_axis_pcnt : NULL;
        break;
    case REG_GP_STAT: /* or REG_CMD */
        p = is_write ? &s->cmd : &s->gp_stat;
        break;
    case REG_SHORT_STROKE:
        p = is_write ? &s->short_stroke : NULL;
        break;
    case REG_BKGD_COLOR:
        p = is_write ? &s->bkgd_color : NULL;
        break;
    case REG_FRGD_COLOR:
        p = is_write ? &s->frgd_color : NULL;
        break;
    case REG_WRT_MASK:
        p = is_write ? &s->wrt_mask : NULL;
        break;
    case REG_RD_MASK:
        p = is_write ? &s->rd_mask : NULL;
        break;
    case REG_COLOR_CMP:
        p = is_write ? &s->color_cmp : NULL;
        break;
    case REG_BKGD_MIX:
        p = is_write ? &s->bkgd_mix : NULL;
        break;
    case REG_FRGD_MIX:
        p = is_write ? &s->frgd_mix : NULL;
        break;
    case REG_MULTIFUNC_CNTL:
        if (is_write) {
            p = &s->mfc[(*val_if_write >> 12) & 0xf];
            *val_if_write &= 0x0fff;
        } else {
            p = NULL;
        }
        break;
    case REG_PIX_TRANS:
        p = &s->pix_trans;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "s3_trio: invalid register 0x%04x\n",
                      addr);
        break;
    }

    return p;
}

static uint32_t s3_trio_ioport_readb(void *opaque, uint32_t addr)
{
    S3TrioState *s = opaque;
    uint32_t val;
    uint16_t *p;

    p = s3_trio_get_register(s, address_to_reg(addr & ~0x1), 0, NULL);

    if (p) {
        val = (*p >> ((~addr & 1) * 8)) & 0xff;
    } else {
        val = 0;
    }

    trace_s3_vga_io_readb(addr, val);
    return val;
}

static uint32_t s3_trio_ioport_readw(void *opaque, uint32_t addr)
{
    S3TrioState *s = opaque;
    uint32_t val;
    uint16_t *p;

    p = s3_trio_get_register(s, address_to_reg(addr), 0, NULL);

    if (p) {
        val = *p;
    } else {
        val = 0;
    }

    trace_s3_vga_io_readw(addr, val);
    return val;
}

static void s3_trio_update_mmio_window(S3TrioState *s);

static void s3_trio_post_write(S3TrioState* s, uint32_t addr)
{
    switch (address_to_reg(addr)) {
    case REG_H_DISP:
        qemu_log_mask(LOG_UNIMP, "s3_trio: unimplemented write to H_DISP\n");
        break;
    case REG_V_DISP:
        qemu_log_mask(LOG_UNIMP, "s3_trio: unimplemented write to V_DISP\n");
        break;
    case REG_SUBSYS_CNTL:
        s->subsys_stat &= ~(s->subsys_cntl & 0x000f); /* clear interrupts */
        break;
    case REG_CMD:
        do_cmd(s);
        break;
    default:
        break;
    }
    s3_trio_update_mmio_window(s);
}

static void s3_trio_ioport_writeb(void *opaque, uint32_t addr, uint32_t val)
{
    S3TrioState *s = opaque;
    uint16_t *p;
    uint8_t *c;

    trace_s3_vga_io_writeb(addr, val);
    /*
     * PIX_TRANS_EXT register
     */
    if ((addr & 0x3fe) == 0x2ea) {
        uint8_t *ec = (uint8_t *)&s->pix_trans_ext;

        ec[~addr & 1] = val;
        if (s->gp_stat & (GP_STAT_BUSY | GP_STAT_DATA_AVAIL)) {
            if ((s->cmd & CMD_32BIT) && !(addr & 1)) {
                /* E2EA's even byte closes the 32-bit transfer */
                s3_pixel_data_write32(s, s->pix_trans_ext);
            } else if (!(s->cmd & CMD_32BIT)) {
                qemu_log_mask(LOG_UNIMP, "s3_trio: pixel data extension "
                              "without 32-bit transfer\n");
            }
        }
        return;
    }
    p = s3_trio_get_register(s, address_to_reg(addr & ~0x1), 1, &val);

    if (p) {
        c = (uint8_t*)p;
        c[~addr & 1] = val;
    }

    if (address_to_reg(addr & ~0x1) == REG_PIX_TRANS) {
        if (s->cmd & CMD_32BIT) {
            /* the low half waits for the extension port */
        } else if (s->gp_stat & (GP_STAT_BUSY | GP_STAT_DATA_AVAIL)) {
            s3_pixel_data_write(s, (addr - 0xe2e8) & 0xf, val & 0xff, 1);
        }
    }
    s3_trio_post_write(s, addr & ~0x1);
}

static void s3_trio_ioport_writew(void *opaque, uint32_t addr, uint32_t val)
{
    S3TrioState *s = opaque;
    uint16_t *p;

    trace_s3_vga_io_writew(addr, val);
    if ((addr & 0x3ff) == 0x2ea) {
        if (!(s->gp_stat & (GP_STAT_BUSY | GP_STAT_DATA_AVAIL))) {
            return;
        }
        if (s->cmd & CMD_32BIT) {
            s3_pixel_data_write32(s, val & 0xffff);
        } else {
            qemu_log_mask(LOG_UNIMP,
                          "s3_trio: pixel data extension without 32-bit transfer\n");
        }
        return;
    }
    p = s3_trio_get_register(s, address_to_reg(addr), 1, &val);

    if (p) {
        *p = val & 0xffff;
    }

    if (address_to_reg(addr) == REG_PIX_TRANS) {
        if (s->cmd & CMD_32BIT) {
            /* Low half stays latched until the extension port supplies
             * the upper half. */
        } else if (s->gp_stat & (GP_STAT_BUSY | GP_STAT_DATA_AVAIL)) {
            s3_pixel_data_write(s, (addr - 0xe2e8) & 0xf, val & 0xffff, 2);
        }
    }
    s3_trio_post_write(s, addr & ~0x1);
}

/*
 * MMIO mode enabled by CR53 bit 4 or 4AE8H bit 5
 */
static bool s3_enhanced_window_enabled(S3TrioState *s)
{
    if (!((s->advfunc_cntl & 0x0020) || (s->vga.cr[0x53] & 0x10))) {
        return false;
    }
    if ((s->vga.cr[0x58] | s->advfunc_cntl) & 0x10) {
        if (!(s->vga.cr[0x31] & 0x01) || (s->vga.cr[0x58] & 0x03) ||
            s->vga.cr[0x59] != 0x00 || s->vga.cr[0x5a] != 0x0a) {
            return false;
        }
    }
    return true;
}

static bool s3_mmio_window_is_reg(hwaddr addr, bool is_write)
{
    if (addr < 0x82e8 || addr > 0xbee9 || (addr & 0x3fe) != 0x2e8) {
        return false;
    }
    return is_write || address_to_reg(addr & ~0x1) != REG_MULTIFUNC_CNTL;
}

static void s3_mmio_window_write(void *opaque, hwaddr addr, uint64_t val,
                                 unsigned size)
{
    S3TrioState *s = opaque;
    unsigned i;

    if (s3_enhanced_window_enabled(s)) {
        if (addr < 0x8000) {
            unsigned off = 0;
            unsigned unit = (s->cmd & CMD_32BIT) ? 4 :
                            (s->cmd & CMD_16BIT) ? 2 : 1;

            /*
             * The image-data half feeds PIX_TRANS: the data is consumed in
             * units of the command's bus size (DB008-B 9.4.4, 10-9), so a
             * wider access must not be split at a fixed two bytes.
             */
            while (off < size) {
                unsigned piece = MIN(size - off, unit);
                uint64_t mask = piece == 4 ? 0xffffffffu :
                                piece == 2 ? 0xffffu : 0xffu;

                s3_pixel_data_write(s, addr + off,
                                    (val >> (8 * off)) & mask, piece);
                off += piece;
            }
            return;
        }
        if (s3_mmio_window_is_reg(addr, true)) {
            if (size == 1) {
                uint32_t byte = val & 0xff;
                uint16_t *p;

                p = s3_trio_get_register(s, address_to_reg(addr & ~0x1), 1,
                                         &byte);
                if (p) {
                    ((uint8_t *)p)[addr & 1] = byte;
                }
                s3_trio_post_write(s, addr & ~0x1);
            } else {
                s3_trio_ioport_writew(s, addr, val & 0xffff);
            }
            return;
        }
        /* No other location decodes in the MMIO window. */
        return;
    }

    /* MMIO off: the aperture is the normal planar/chain-4 VGA memory. */
    for (i = 0; i < size; i++) {
        vga_mem_writeb(&s->vga, addr + i, (val >> (8 * i)) & 0xff);
    }
}

static uint64_t s3_mmio_window_read(void *opaque, hwaddr addr, unsigned size)
{
    S3TrioState *s = opaque;
    uint64_t val = 0;
    unsigned i;

    if (s3_enhanced_window_enabled(s) && s3_mmio_window_is_reg(addr, false)) {
        uint16_t *p = s3_trio_get_register(s, address_to_reg(addr & ~0x1), 0,
                                           NULL);
        uint16_t w = p ? *p : 0;

        if (size == 1) {
            val = (w >> (8 * (addr & 1))) & 0xff;
        } else {
            val = w;
        }
        return val;
    }

    for (i = 0; i < size; i++) {
        val |= (uint64_t)vga_mem_readb(&s->vga, addr + i) << (8 * i);
    }
    return val;
}

static const MemoryRegionOps s3_mmio_window_ops = {
    .read = s3_mmio_window_read,
    .write = s3_mmio_window_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static uint32_t s3_trio_vga_ioport_read(void *opaque, uint32_t addr)
{
    S3TrioState *s = opaque;
    uint32_t val;

    switch (addr) {
    case VGA_CRT_DM:
    case VGA_CRT_DC:
        switch (s->vga.cr_index) {
        case 0x2d:
            val = 0x88;
            break;
        case 0x2e:
            val = 0x10;
            break;
        case 0x30:
            val = 0xe0;
            break;
        case 0x36:
        {
            static const uint8_t smem[] = { 7, 6, 4, 2, 0, 0, 5, 5, 3 };
            if (s->vga.vram_size_mb < sizeof(smem)) {
                val = smem[s->vga.vram_size_mb];
            } else {
                val = smem[sizeof(smem) - 1];
            }
            val = val << 5;
            break;
        }
        case 0x45:
            /* reading CR45 resets the cursor color stack pointer */
            s->cursor_color_pos = 0;
            val = vga_ioport_read(&s->vga, addr);
            break;
        case 0x47:
            val = s->origin_x;
            break;
        case 0x49:
            val = s->origin_y;
            break;
        default:
            val = vga_ioport_read(&s->vga, addr);
            break;
        }
        break;
    case VGA_SEQ_D:
        switch (s->vga.sr_index) {
        case 0x17: /* CLKSYN */
        {
            uint32_t *clk;
            if (!(s->vga.sr[0x14] & 0x01)) {
                s->dclk++;
            }
            if (!(s->vga.sr[0x14] & 0x03)) {
                s->mclk++;
            }
            if (s->vga.sr[0x14] & 0x04) {
                clk = &s->mclk;
            } else {
                clk = &s->dclk;
            }
            val = (*clk) & 0xff;
            break;
        }
        default:
            if (s3_crtc_seq_shared(s, s->vga.sr_index)) {
                val = s->vga.cr[s->vga.sr_index];
            } else {
                val = vga_ioport_read(&s->vga, addr);
            }
            break;
        }
        break;
    default:
        val = vga_ioport_read(&s->vga, addr);
        break;
    }

    trace_s3_vga_io_readb(addr, val);
    return val;
}

static void s3_trio_update_bank_offset(S3TrioState *s)
{
    s->vga.bank_offset = (s->vga.cr[0x31] & 0x01) ?
                         ((uint32_t)(s->vga.cr[0x6a] & 0x3f) << 16) : 0;
    vga_update_memory_access(&s->vga);
}

static void s3_trio_update_mmio_window(S3TrioState *s)
{
    bool want = s3_enhanced_window_enabled(s);

    if (want == s->mmio_window_mapped) {
        return;
    }
    if (want) {
        memory_region_add_subregion_overlap(s->vga.legacy_address_space,
                                            0x000a0000, &s->mmio_window, 3);
    } else {
        memory_region_del_subregion(s->vga.legacy_address_space,
                                    &s->mmio_window);
    }
    s->mmio_window_mapped = want;
}

static void s3_trio_update_cursor_shadow(S3TrioState *s)
{
    s->vga.force_shadow = s3_cursor_enabled(s);
}

static void s3_trio_vga_ioport_write(void *opaque, uint32_t addr, uint32_t val)
{
    S3TrioState *s = opaque;

    trace_s3_vga_io_writeb(addr, val);
    switch (addr) {
    case VGA_CRT_DM:
    case VGA_CRT_DC:
        trace_s3_vga_crtc_write(s->vga.cr_index, val);
        switch (s->vga.cr_index) {
        case 0x08:
            s->unlock_pll = (val == 0x06);
            break;
        case 0x10: /* memory pll data */
        case 0x11: /* memory pll data */
        case 0x12: /* video pll data */
        case 0x13: /* video pll data */
        case 0x15:
        case 0x18:
            if (s->unlock_pll) {
                qemu_log_mask(LOG_UNIMP,
                              "s3_trio: unimplemented PLL change\n");
            } else {
                vga_ioport_write(&s->vga, addr, val);
            }
            break;
        case 0x33:
            s->unlock_compatibility_registers = ((val & ~0xad) == 0);
            break;
        case 0x31:
        case 0x6a:
            vga_ioport_write(&s->vga, addr, val);
            s3_trio_update_bank_offset(s);
            break;
        case 0x38:
            s->unlock_control_registers_1 = (val == 0x48);
            break;
        case 0x39:
            s->unlock_control_registers_2 = (val == 0xa5);
            break;
        case 0x45:
            vga_ioport_write(&s->vga, addr, val);
            s3_trio_update_cursor_shadow(s);
            break;
        case 0x47:
            s->origin_x = val;
            break;
        case 0x4a:
        case 0x4b:
        {
            uint8_t *stack = (s->vga.cr_index == 0x4a) ?
                             s->cursor_fg : s->cursor_bg;

            stack[s->cursor_color_pos] = val;
            s->cursor_color_pos = (s->cursor_color_pos + 1) % 3;
            vga_ioport_write(&s->vga, addr, val);
            break;
        }
        case 0x49:
            s->origin_y = val;
            break;
        default:
            vga_ioport_write(&s->vga, addr, val);
            break;
        }
        s3_trio_update_mmio_window(s);
        break;
    case VGA_SEQ_I:
        s->vga.sr_index = val;
        break;
    case VGA_SEQ_D:
        switch (s->vga.sr_index) {
        case 0x00 ... 0x07:
            /* hack: drop CR1/CR5D bit */
            if (s->crtc_seq_shared && s->vga.sr_index == 0x01) {
                val &= ~0x08;
            }
            vga_ioport_write(&s->vga, addr, val);
            break;
        case 0x08:
            s->vga.sr[s->vga.sr_index] = val;
            break;
        case 0x09 ... 0x1c:
            if ((s->vga.sr[0x08] & 0x0f) == 0x06) {
                s->vga.sr[s->vga.sr_index] = val;
            }
            break;
        default:
            /* SR >= 0x1d are plain R/W on the real chip; store so the
             * read-back answers like real. */
            s->vga.sr[s->vga.sr_index] = val;
            break;
        }
        break;
    default:
        if (addr == VGA_PEL_IW) {
            s->vga.dac_8bit = 0;
        } else if (addr == VGA_PEL_D && val > 0x3f) {
            s->vga.dac_8bit = 1;
        }
        vga_ioport_write(&s->vga, addr, val);
        break;
    }
}

static const MemoryRegionPortio s3_trio_portio_list[] = {
    { 0x0102, 1, 1, .read = s3_trio_enable_readb, .write = s3_trio_enable_writeb, },
    { 0x02e8, 1, 2, .read = s3_trio_ioport_readw, .write = s3_trio_ioport_writew, },
    { 0x02e8, 2, 1, .read = s3_trio_ioport_readb, .write = s3_trio_ioport_writeb, },
    { 0x02ea, 4, 1, .read = s3_trio_dac_ioport_readb, .write = s3_trio_dac_ioport_writeb, },
    { 0x03b4,  2, 1, .read = s3_trio_vga_ioport_read, .write = s3_trio_vga_ioport_write },
    { 0x03ba,  1, 1, .read = s3_trio_vga_ioport_read, .write = s3_trio_vga_ioport_write },
    { 0x03c0, 16, 1, .read = s3_trio_vga_ioport_read, .write = s3_trio_vga_ioport_write },
    { 0x03d4,  2, 1, .read = s3_trio_vga_ioport_read, .write = s3_trio_vga_ioport_write },
    { 0x03da,  1, 1, .read = s3_trio_vga_ioport_read, .write = s3_trio_vga_ioport_write },
    { 0x06e8, 1, 2, .read = s3_trio_ioport_readw, .write = s3_trio_ioport_writew, },
    { 0x06e8, 2, 1, .read = s3_trio_ioport_readb, .write = s3_trio_ioport_writeb, },
    { 0x0ae8, 1, 2, .read = s3_trio_ioport_readw, .write = s3_trio_ioport_writew, },
    { 0x0ae8, 2, 1, .read = s3_trio_ioport_readb, .write = s3_trio_ioport_writeb, },
    { 0x0ee8, 1, 2, .read = s3_trio_ioport_readw, .write = s3_trio_ioport_writew, },
    { 0x0ee8, 2, 1, .read = s3_trio_ioport_readb, .write = s3_trio_ioport_writeb, },
    { 0x16e8, 2, 1, .read = s3_trio_ioport_readb, .write = s3_trio_ioport_writeb, },
    { 0x1ae8, 1, 2, .read = s3_trio_ioport_readw, .write = s3_trio_ioport_writew, },
    { 0x1ae8, 2, 1, .read = s3_trio_ioport_readb, .write = s3_trio_ioport_writeb, },
    { 0x1ee8, 1, 2, .read = s3_trio_ioport_readw, .write = s3_trio_ioport_writew, },
    { 0x1ee8, 2, 1, .read = s3_trio_ioport_readb, .write = s3_trio_ioport_writeb, },
    { 0x22e8, 1, 2, .read = s3_trio_ioport_readw, .write = s3_trio_ioport_writew, },
    { 0x22e8, 2, 1, .read = s3_trio_ioport_readb, .write = s3_trio_ioport_writeb, },
    { 0x26e8, 1, 2, .read = s3_trio_ioport_readw, .write = s3_trio_ioport_writew, },
    { 0x26e8, 2, 1, .read = s3_trio_ioport_readb, .write = s3_trio_ioport_writeb, },
    { 0x2ae8, 1, 2, .read = s3_trio_ioport_readw, .write = s3_trio_ioport_writew, },
    { 0x2ae8, 2, 1, .read = s3_trio_ioport_readb, .write = s3_trio_ioport_writeb, },
    { 0x2ee8, 1, 2, .read = s3_trio_ioport_readw, .write = s3_trio_ioport_writew, },
    { 0x2ee8, 2, 1, .read = s3_trio_ioport_readb, .write = s3_trio_ioport_writeb, },
    { 0x32e8, 1, 2, .read = s3_trio_ioport_readw, .write = s3_trio_ioport_writew, },
    { 0x32e8, 2, 1, .read = s3_trio_ioport_readb, .write = s3_trio_ioport_writeb, },
    { 0x36e8, 1, 2, .read = s3_trio_ioport_readw, .write = s3_trio_ioport_writew, },
    { 0x36e8, 2, 1, .read = s3_trio_ioport_readb, .write = s3_trio_ioport_writeb, },
    { 0x3ae8, 1, 2, .read = s3_trio_ioport_readw, .write = s3_trio_ioport_writew, },
    { 0x3ae8, 2, 1, .read = s3_trio_ioport_readb, .write = s3_trio_ioport_writeb, },
    { 0x3ee8, 1, 2, .read = s3_trio_ioport_readw, .write = s3_trio_ioport_writew, },
    { 0x3ee8, 2, 1, .read = s3_trio_ioport_readb, .write = s3_trio_ioport_writeb, },
    { 0x42e8, 1, 2, .read = s3_trio_ioport_readw, .write = s3_trio_ioport_writew, },
    { 0x42e8, 2, 1, .read = s3_trio_ioport_readb, .write = s3_trio_ioport_writeb, },
    { 0x46e8, 1, 2, .read = s3_trio_ioport_readw, .write = s3_trio_ioport_writew, },
    { 0x46e8, 2, 1, .read = s3_trio_ioport_readb, .write = s3_trio_ioport_writeb, },
    { 0x4ae8, 1, 2, .read = s3_trio_ioport_readw, .write = s3_trio_ioport_writew, },
    { 0x4ae8, 2, 1, .read = s3_trio_ioport_readb, .write = s3_trio_ioport_writeb, },
    { 0x82e8, 1, 2, .read = s3_trio_ioport_readw, .write = s3_trio_ioport_writew, },
    { 0x82e8, 2, 1, .read = s3_trio_ioport_readb, .write = s3_trio_ioport_writeb, },
    { 0x86e8, 1, 2, .read = s3_trio_ioport_readw, .write = s3_trio_ioport_writew, },
    { 0x86e8, 2, 1, .read = s3_trio_ioport_readb, .write = s3_trio_ioport_writeb, },
    { 0x8ae8, 1, 2, .read = s3_trio_ioport_readw, .write = s3_trio_ioport_writew, },
    { 0x8ae8, 2, 1, .read = s3_trio_ioport_readb, .write = s3_trio_ioport_writeb, },
    { 0x8ee8, 1, 2, .read = s3_trio_ioport_readw, .write = s3_trio_ioport_writew, },
    { 0x8ee8, 2, 1, .read = s3_trio_ioport_readb, .write = s3_trio_ioport_writeb, },
    { 0x92e8, 1, 2, .read = s3_trio_ioport_readw, .write = s3_trio_ioport_writew, },
    { 0x92e8, 2, 1, .read = s3_trio_ioport_readb, .write = s3_trio_ioport_writeb, },
    { 0x96e8, 1, 2, .read = s3_trio_ioport_readw, .write = s3_trio_ioport_writew, },
    { 0x96e8, 2, 1, .read = s3_trio_ioport_readb, .write = s3_trio_ioport_writeb, },
    { 0x9ae8, 1, 2, .read = s3_trio_ioport_readw, .write = s3_trio_ioport_writew, },
    { 0x9ae8, 2, 1, .read = s3_trio_ioport_readb, .write = s3_trio_ioport_writeb, },
    { 0x9ee8, 1, 2, .read = s3_trio_ioport_readw, .write = s3_trio_ioport_writew, },
    { 0x9ee8, 2, 1, .read = s3_trio_ioport_readb, .write = s3_trio_ioport_writeb, },
    { 0xa2e8, 1, 2, .read = s3_trio_ioport_readw, .write = s3_trio_ioport_writew, },
    { 0xa2e8, 2, 1, .read = s3_trio_ioport_readb, .write = s3_trio_ioport_writeb, },
    { 0xa6e8, 1, 2, .read = s3_trio_ioport_readw, .write = s3_trio_ioport_writew, },
    { 0xa6e8, 2, 1, .read = s3_trio_ioport_readb, .write = s3_trio_ioport_writeb, },
    { 0xaae8, 1, 2, .read = s3_trio_ioport_readw, .write = s3_trio_ioport_writew, },
    { 0xaae8, 2, 1, .read = s3_trio_ioport_readb, .write = s3_trio_ioport_writeb, },
    { 0xaee8, 1, 2, .read = s3_trio_ioport_readw, .write = s3_trio_ioport_writew, },
    { 0xaee8, 2, 1, .read = s3_trio_ioport_readb, .write = s3_trio_ioport_writeb, },
    { 0xb2e8, 1, 2, .read = s3_trio_ioport_readw, .write = s3_trio_ioport_writew, },
    { 0xb2e8, 2, 1, .read = s3_trio_ioport_readb, .write = s3_trio_ioport_writeb, },
    { 0xb6e8, 1, 2, .read = s3_trio_ioport_readw, .write = s3_trio_ioport_writew, },
    { 0xb6e8, 2, 1, .read = s3_trio_ioport_readb, .write = s3_trio_ioport_writeb, },
    { 0xbae8, 1, 2, .read = s3_trio_ioport_readw, .write = s3_trio_ioport_writew, },
    { 0xbae8, 2, 1, .read = s3_trio_ioport_readb, .write = s3_trio_ioport_writeb, },
    { 0xbee8, 1, 2, .read = s3_trio_ioport_readw, .write = s3_trio_ioport_writew, },
    { 0xbee8, 2, 1, .read = s3_trio_ioport_readb, .write = s3_trio_ioport_writeb, },
    { 0xe2e8, 1, 2, .read = s3_trio_ioport_readw, .write = s3_trio_ioport_writew, },
    { 0xe2e8, 2, 1, .read = s3_trio_ioport_readb, .write = s3_trio_ioport_writeb, },
    { 0xe2ea, 1, 2, .read = s3_trio_ioport_readw, .write = s3_trio_ioport_writew, },
    { 0xe2ea, 2, 1, .read = s3_trio_ioport_readb, .write = s3_trio_ioport_writeb, },
    PORTIO_END_OF_LIST()
};

static int s3_trio_get_bpp(VGACommonState *s)
{
    S3TrioState *st = container_of(s, S3TrioState, vga);

    if (s->vbe_regs[VBE_DISPI_INDEX_ENABLE] & VBE_DISPI_ENABLED) {
        return s->vbe_regs[VBE_DISPI_INDEX_BPP];
    }
    trace_s3_vga_bpp(s->cr[0x50], s->cr[0x67], s->cr[0x3a], st->advfunc_cntl,
                     s3_pixel_depth(st), s3_enhanced_mode(st));
    return s3_pixel_depth(st);
}

static bool s3_trio_native_mode(VGACommonState *s)
{
    S3TrioState *st = container_of(s, S3TrioState, vga);

    if (!s3_enhanced_mode(st)) {
        return false;
    }
    /* With 1-byte pixels 4AE8 bit 2 means 4-bit planes or packed 8 bpp,
     * so GR05 cannot be trusted for it. */
    return s3_pixel_bytes(st) > 1 || !(st->advfunc_cntl & 0x04);
}

/* CR51 bits 5-4 extend the line offset register to 10 bits (DB008-B 17-2). */
static uint32_t s3_trio_line_offset(S3TrioState *s)
{
    unsigned ofs = s->vga.cr[0x13] | (((s->vga.cr[0x51] >> 4) & 3) << 8);

    return ofs << 3;
}

static void s3_trio_get_params(VGACommonState *s, VGADisplayParams *params)
{
    S3TrioState *st = container_of(s, S3TrioState, vga);

    st->base_get_params(s, params);
    /* doubleword addressing (CR31 bit 3) is the packed-pixel case */
    if (!(s->vbe_regs[VBE_DISPI_INDEX_ENABLE] & VBE_DISPI_ENABLED) &&
        (s->cr[0x31] & 0x08)) {
        params->line_offset = s3_trio_line_offset(st);
    }
}

/* Enhanced-mode CRTC counts memory units, not pixels (DB008-B 9-3). */
static void s3_trio_get_resolution(VGACommonState *s, int *pwidth, int *pheight)
{
    int w = (s->cr[0x01] + 1 + ((s->cr[0x5d] & 0x02) << 7)) * 8;
    int h = s->cr[0x12] | ((s->cr[0x07] & 0x02) << 7) |
            ((s->cr[0x07] & 0x40) << 3) | ((s->cr[0x5e] & 0x02) << 9);

    if (s3_enhanced_mode(container_of(s, S3TrioState, vga))) {
        switch (s3_pixel_depth(container_of(s, S3TrioState, vga))) {
        case 15:
        case 16:
            w >>= 1;
            break;
        case 24:
            w /= 3;
            break;
        case 32:
            w >>= 2;
            break;
        default:
            break;
        }
        /* CR50 gives the bitmap width in pixels, which the CRTC count
         * can disagree with (DB008-B table 9-3). */
        switch (((s->cr[0x50] & 0x01) << 2) | ((s->cr[0x50] & 0x80) >> 6) |
                ((s->cr[0x50] & 0x40) >> 6)) {
        case 1:
            w = 640;
            break;
        case 2:
            w = 800;
            break;
        case 3:
            w = 1280;
            break;
        case 4:
            w = 1152;
            break;
        case 6:
            w = 1600;
            break;
        default:
            break;
        }
    }
    /* Interlaced modes count one field in CR12/CR7/CR5E (CR42 bit 5). */
    if (s->cr[0x42] & 0x20) {
        h = (h << 1) | 1;
    }
    *pwidth = w;
    *pheight = h + 1;
}

static VMStateDescription vmstate_s3_trio = {
    .name = TYPE_S3_TRIO,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField []) {
        VMSTATE_PCI_DEVICE(dev, S3TrioState),
        VMSTATE_STRUCT(vga, S3TrioState, 0, vmstate_vga_common, VGACommonState),
        VMSTATE_UINT16(maj_axis, S3TrioState),
        VMSTATE_UINT16(min_axis, S3TrioState),
        VMSTATE_UINT16(disp_stat, S3TrioState),
        VMSTATE_UINT16(h_disp, S3TrioState),
        VMSTATE_UINT16(h_sync_strt, S3TrioState),
        VMSTATE_UINT16(h_sync_wid, S3TrioState),
        VMSTATE_UINT16(v_total, S3TrioState),
        VMSTATE_UINT16(v_disp, S3TrioState),
        VMSTATE_UINT16(v_sync_strt, S3TrioState),
        VMSTATE_UINT16(v_sync_wid, S3TrioState),
        VMSTATE_UINT16(disp_cntl, S3TrioState),
        VMSTATE_UINT16(h_total, S3TrioState),
        VMSTATE_UINT16(subsys_cntl, S3TrioState),
        VMSTATE_UINT16(subsys_stat, S3TrioState),
        VMSTATE_UINT16(rom_page_sel, S3TrioState),
        VMSTATE_UINT16(advfunc_cntl, S3TrioState),
        VMSTATE_UINT16(cur_y, S3TrioState),
        VMSTATE_UINT16(cur_x, S3TrioState),
        VMSTATE_UINT16(desty_axstep, S3TrioState),
        VMSTATE_UINT16(destx_diastp, S3TrioState),
        VMSTATE_UINT16(err_term, S3TrioState),
        VMSTATE_UINT16(maj_axis_pcnt, S3TrioState),
        VMSTATE_UINT16(gp_stat, S3TrioState),
        VMSTATE_UINT16(cmd, S3TrioState),
        VMSTATE_UINT16(short_stroke, S3TrioState),
        VMSTATE_UINT16(bkgd_color, S3TrioState),
        VMSTATE_UINT16(frgd_color, S3TrioState),
        VMSTATE_UINT16(wrt_mask, S3TrioState),
        VMSTATE_UINT16(rd_mask, S3TrioState),
        VMSTATE_UINT16(color_cmp, S3TrioState),
        VMSTATE_UINT16(bkgd_mix, S3TrioState),
        VMSTATE_UINT16(frgd_mix, S3TrioState),
        VMSTATE_UINT16_ARRAY(mfc, S3TrioState, 16),
        VMSTATE_UINT16(pix_trans, S3TrioState),
        VMSTATE_END_OF_LIST()
    },
};

static const Property s3_trio_properties[] = {
    DEFINE_PROP_UINT32("vram_size_mb", S3TrioState, vga.vram_size_mb, 0),
    DEFINE_PROP_UINT32("device_id", S3TrioState, device_id, 0),
};

static void s3_trio_reset(DeviceState *d)
{
    S3TrioState *s = S3_TRIO(d);

    if (s->mmio_window_mapped) {
        memory_region_del_subregion(s->vga.legacy_address_space,
                                    &s->mmio_window);
        s->mmio_window_mapped = false;
    }

    vga_common_reset(&s->vga);

    s->vga.force_shadow = false;
    s->cursor_color_pos = 0;
    s->last_cursor_on = false;
    s->last_cursor_x = -1;
    s->last_cursor_y = -1;
    s->last_cursor_addr = 0;
    s->last_cursor_hash = 0;

    if (object_dynamic_cast(OBJECT(d), TYPE_S3_V864) != NULL) {
        s->crtc_seq_shared = true;
    }

    s->disp_stat |= DISP_STAT_SENSE;
    s->gp_stat = GP_STAT_FIFO_EMPTY;
    s->xfer_lo = 0;
    s->xfer_lo_valid = false;
}

static bool s3_cursor_enabled(S3TrioState *s)
{
    return s->vga.cr[0x45] & 1;
}

static int s3_cursor_x(S3TrioState *s)
{
    return ((s->vga.cr[0x46] & 7) << 8) | s->origin_x;
}

static int s3_cursor_y(S3TrioState *s)
{
    return ((s->vga.cr[0x48] & 7) << 8) | s->origin_y;
}

static int s3_cursor_start_x(S3TrioState *s)
{
    return s->vga.cr[0x4e] & 0x3f;
}

static int s3_cursor_start_y(S3TrioState *s)
{
    return s->vga.cr[0x4f] & 0x3f;
}

static uint32_t s3_cursor_addr(S3TrioState *s)
{
    uint32_t addr = (((s->vga.cr[0x4c] & 0x0f) << 8) |
                     s->vga.cr[0x4d]) * 1024u;

    return addr % s->vga.vram_size;
}

static void s3_cursor_bits(S3TrioState *s, int px, int py,
                           int *and_bit, int *xor_bit)
{
    uint32_t base = s3_cursor_addr(s) + 16u * py;
    uint32_t idx = base + ((px >> 4) * 4) + ((px >> 3) & 1);
    uint8_t mask = 1 << (7 - (px & 7));

    if (idx + 2 >= s->vga.vram_size) {
        *and_bit = 0;
        *xor_bit = 0;
        return;
    }
    *and_bit = !!(s->vga.vram_ptr[idx] & mask);
    *xor_bit = !!(s->vga.vram_ptr[idx + 2] & mask);
}

static uint32_t s3_cursor_color(S3TrioState *s, bool fg)
{
    unsigned csel = (s->vga.cr[0x45] >> 2) & 3;
    unsigned r, g, b;

    if (csel == 0 && s3_pixel_bytes(s) == 1) {
        /* 4/8 bpp cursor colours are CR0E/CR0F palette indices
         * (DB008-B 11.3.3.11, 86C928 11.3.13). */
        uint8_t idx = fg ? s->vga.cr[0x0e] : s->vga.cr[0x0f];

        r = s->vga.palette[idx * 3];
        g = s->vga.palette[idx * 3 + 1];
        b = s->vga.palette[idx * 3 + 2];
        if (!s->vga.dac_8bit) {
            r = c6_to_8(r);
            g = c6_to_8(g);
            b = c6_to_8(b);
        }
    } else {
        const uint8_t *c = fg ? s->cursor_fg : s->cursor_bg;

        if (csel == 1) {
            uint32_t v = c[0] | (c[1] << 8);

            if (s3_pixel_565(s)) {
                r = ((v >> 11) & 0x1f) << 3;
                g = ((v >> 5) & 0x3f) << 2;
                b = (v & 0x1f) << 3;
            } else {
                r = ((v >> 10) & 0x1f) << 3;
                g = ((v >> 5) & 0x1f) << 3;
                b = (v & 0x1f) << 3;
            }
        } else {
            r = c[0];
            g = c[1];
            b = c[2];
        }
    }
    return rgb_to_pixel32(r, g, b);
}

static uint32_t s3_cursor_hash(S3TrioState *s)
{
    uint32_t addr = s3_cursor_addr(s);
    uint32_t h = 2166136261u;
    int i;

    for (i = 0; i < 1024; i++) {
        h ^= s->vga.vram_ptr[(addr + i) % s->vga.vram_size];
        h *= 16777619u;
    }
    return h;
}

static void s3_cursor_invalidate(VGACommonState *vga)
{
    S3TrioState *s = container_of(vga, S3TrioState, vga);
    bool on = s3_cursor_enabled(s);
    int x = s3_cursor_x(s);
    int y = s3_cursor_y(s);
    uint32_t addr = s3_cursor_addr(s);
    uint32_t hash = on ? s3_cursor_hash(s) : 0;

    if (s->last_cursor_on == on && s->last_cursor_x == x &&
        s->last_cursor_y == y && s->last_cursor_addr == addr &&
        s->last_cursor_hash == hash) {
        return;
    }

    if (s->last_cursor_on) {
        vga_invalidate_scanlines(vga, s->last_cursor_y,
                                 s->last_cursor_y + 64);
    }
    if (on) {
        vga_invalidate_scanlines(vga, y, y + 64);
    }
    s->last_cursor_on = on;
    s->last_cursor_x = x;
    s->last_cursor_y = y;
    s->last_cursor_addr = addr;
    s->last_cursor_hash = hash;
}

static void s3_cursor_draw_line(VGACommonState *vga, uint8_t *d, int scr_y)
{
    S3TrioState *s = container_of(vga, S3TrioState, vga);
    int start_x, start_y, origin_x, origin_y, pattern_y, px;
    int width = vga->last_scr_width;
    bool x11_mode;
    uint32_t fg, bg;
    uint32_t *dst = (uint32_t *)d;

    if (!s3_cursor_enabled(s) || width <= 0) {
        return;
    }

    start_x = s3_cursor_start_x(s);
    start_y = s3_cursor_start_y(s);
    origin_x = s3_cursor_x(s);
    origin_y = s3_cursor_y(s);
    pattern_y = scr_y - origin_y + start_y;
    if (pattern_y < 0 || pattern_y >= 64) {
        return;
    }

    x11_mode = s->vga.cr[0x55] & 0x10;
    fg = s3_cursor_color(s, true);
    bg = s3_cursor_color(s, false);

    for (px = start_x; px < 64; px++) {
        int sx = origin_x + px - start_x;
        int and_bit, xor_bit;

        if (sx < 0 || sx >= width) {
            continue;
        }
        s3_cursor_bits(s, px, pattern_y, &and_bit, &xor_bit);

        if (x11_mode) {
            if (and_bit && xor_bit) {
                dst[sx] = fg;
            } else if (and_bit && !xor_bit) {
                dst[sx] = bg;
            }
        } else {
            if (!and_bit && !xor_bit) {
                dst[sx] = bg;
            } else if (!and_bit && xor_bit) {
                dst[sx] = fg;
            } else if (and_bit && xor_bit) {
                dst[sx] ^= 0x00ffffff;
            }
        }
    }
}

static void s3_trio_realize(PCIDevice *dev, Error **errp)

{
    S3TrioState *s = S3_TRIO(dev);
    Object *o = OBJECT(dev);
    const MemoryRegionPortio *vga_ports, *vbe_ports;
    MemoryRegion* vga_io_memory;

    /* 0 = default size: set 8 MB for Trio64, 2 MB for V864 */
    if (s->vga.vram_size_mb == 0) {
        s->vga.vram_size_mb =
            object_dynamic_cast(OBJECT(dev), TYPE_S3_V864) ? 2 : 8;
    }

    if (s->device_id) {
        pci_config_set_device_id(dev->config, s->device_id);
    }

    /* setup VGA */
    if (!vga_common_init(&s->vga, OBJECT(dev), errp)) {
        return;
    }
    s->vga.legacy_address_space = pci_address_space(dev);
    vga_io_memory = vga_init_io(&s->vga, o, &vga_ports, &vbe_ports);
    memory_region_add_subregion_overlap(s->vga.legacy_address_space,
                                        0x000a0000, vga_io_memory, 1);
    memory_region_set_coalescing(vga_io_memory);
    memory_region_set_coalescing(&s->vga.vram);

    memory_region_init_io(&s->mmio_window, o, &s3_mmio_window_ops, s,
                          "s3-mmio-window", 0x20000);

    /*
     * On 40p, ARC probes the option ROM but firmware didn't shadowed it.
     * It is mapped in such non-standard way, so clean the property then.
     */
    g_autofree char *rom_name =
        g_strdup(dev->romfile && dev->romfile[0] ?
                 dev->romfile : "vgabios-s3.bin");
    g_free(dev->romfile);
    dev->romfile = NULL;

    char *rom_path = qemu_find_file(QEMU_FILE_TYPE_BIOS, rom_name);
    if (rom_path) {
        memory_region_init_rom(&s->rom, OBJECT(dev), "s3-trio.rom",
                               0x10000, &error_fatal);
        if (load_image_mr(rom_path, &s->rom) > 0) {
            memory_region_add_subregion_overlap(s->vga.legacy_address_space,
                                                0x000c0000, &s->rom, 1);
            memory_region_init_alias(&s->rom_bar, OBJECT(dev),
                                     "s3-trio.rom-bar", &s->rom, 0, 0x10000);
            pci_register_bar(dev, PCI_ROM_SLOT, 0, &s->rom_bar);
        }
        g_free(rom_path);
    } else {
        error_report("s3-trio: option ROM '%s' not found", rom_name);
    }

    s->vga.con = qemu_graphic_console_create(DEVICE(s), 0, s->vga.hw_ops, &s->vga);

    s->vga.get_bpp = s3_trio_get_bpp;
    s->base_get_params = s->vga.get_params;
    s->vga.get_params = s3_trio_get_params;
    s->vga.native_mode = s3_trio_native_mode;
    s->vga.get_resolution = s3_trio_get_resolution;
    s->vga.big_endian_fb = false;
    s->vga.default_endian_fb = false;
    s->vga.cursor_invalidate = s3_cursor_invalidate;
    s->vga.cursor_draw_line = s3_cursor_draw_line;

    isa_register_portio_list(NULL, &s->portio, 0, s3_trio_portio_list, s, "s3_trio");

    /* setup PCI */
    pci_register_bar(dev, 0, PCI_BASE_ADDRESS_MEM_PREFETCH, &s->vga.vram);

    /* fixed video window in PReP I/O mem */
    memory_region_init_alias(&s->vram_iomem, o, "s3-trio-vram-iomem",
                             &s->vga.vram, 0, s->vga.vram_size);
    memory_region_add_subregion_overlap(pci_address_space(dev), 0x02800000,
                                        &s->vram_iomem, 0);
}

static void s3_trio_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->realize = s3_trio_realize;
    //k->romfile = "vgabios-s3.bin";
    k->vendor_id = PCI_VENDOR_ID_S3;
    k->device_id = PCI_DEVICE_ID_S3_TRIO;
    k->class_id = PCI_CLASS_DISPLAY_VGA;
    device_class_set_legacy_reset(dc, s3_trio_reset);
    dc->desc = "S3 Trio 32 VGA";
    dc->vmsd  = &vmstate_s3_trio;
    device_class_set_props(dc, s3_trio_properties);
    set_bit(DEVICE_CATEGORY_DISPLAY, dc->categories);
}

static const TypeInfo s3_trio_info = {
    .name          = "s3-trio",
    .parent        = TYPE_PCI_DEVICE,
    .instance_size = sizeof(S3TrioState),
    .class_init    = s3_trio_class_init,
    .interfaces = (const InterfaceInfo[]) {
        { INTERFACE_CONVENTIONAL_PCI_DEVICE },
        { },
    },
};

static void s3_v864_class_init(ObjectClass *klass, const void *data)
{
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    // This default device_id of V864 is what can got pass Windows NT
    k->device_id = 0x88b0;
}

static const TypeInfo s3_v864_info = {
    .name          = TYPE_S3_V864,
    .parent        = TYPE_S3_TRIO,
    .class_init    = s3_v864_class_init,
};

static void s3_register_types(void)
{
    type_register_static(&s3_trio_info);
    type_register_static(&s3_v864_info);
}

type_init(s3_register_types)
