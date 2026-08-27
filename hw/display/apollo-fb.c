/*
 * Apollo DN3000 display framebuffer.
 *
 * The real DN3000 graphics (research §2.11, MAME apollo_graphics_15i) is a
 * bespoke 1024x800 mono/color planar system with a 2D blitter and a Brooktree
 * Bt458 RAMDAC.  simpledrm/simplefb only consume linear RGB, so instead of the
 * planar blitter path we model the *post-scanout* view: a linear xrgb8888
 * framebuffer that Linux drives directly as a "simple-framebuffer".  A small
 * MCR (display enable) register and a Bt458 CLUT are modelled for faithfulness;
 * the CLUT is bypassed in the truecolor scanout (documented, unused by simplefb).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/core/sysbus.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "ui/console.h"
#include "framebuffer.h"
#include "system/memory.h"
#include "hw/display/apollo-fb.h"

struct ApolloFbState {
    SysBusDevice parent_obj;

    MemoryRegion fb_mr;                 /* linear xrgb8888 framebuffer RAM */
    MemoryRegion ctrl_mr;               /* MCR + Bt458 RAMDAC registers */
    MemoryRegionSection fbsection;
    QemuConsole *con;

    uint32_t width;
    uint32_t height;
    uint32_t stride;
    int invalidate;

    /* control state (MCR / Bt458 RAMDAC) */
    uint8_t mcr;                        /* bit0 = display enable */
    uint8_t bt458_addr;                 /* RAMDAC address pointer */
    uint8_t bt458_sub;                  /* R/G/B sub-index within an entry */
    uint8_t bt458_clut[256][3];         /* modelled, bypassed for truecolor */
};

/* Convert one row of big-endian xrgb8888 source pixels to the QEMU surface. */
static void apollo_fb_draw_line(void *opaque, uint8_t *d, const uint8_t *s,
                                int width, int pitch)
{
    uint32_t *dst = (uint32_t *)d;
    int i;

    for (i = 0; i < width; i++) {
        /*
         * The 68k guest writes each pixel as a big-endian 32-bit word
         * 0x00RRGGBB (simplefb "x8r8g8b8").  The QEMU console surface is
         * native x8r8g8b8, so the reconstructed native value maps directly.
         */
        dst[i] = ldl_be_p(s + i * 4);
    }
}

static void apollo_fb_blank(ApolloFbState *s, DisplaySurface *surface)
{
    uint8_t *data = surface_data(surface);
    int stride = surface_stride(surface);
    int y;

    for (y = 0; y < s->height; y++) {
        memset(data + y * stride, 0, s->width * 4);
    }
    qemu_console_update(s->con, 0, 0, s->width, s->height);
}

static bool apollo_fb_update(void *opaque)
{
    ApolloFbState *s = APOLLO_FB(opaque);
    DisplaySurface *surface = qemu_console_surface(s->con);
    int src_width = s->stride;
    int dest_width = s->width * 4;
    int first = 0, last = 0;

    if (!surface) {
        return false;
    }

    /* MCR display-enable gates scanout (default enabled). */
    if (!(s->mcr & APOLLO_FB_MCR_ENABLE)) {
        if (s->invalidate) {
            apollo_fb_blank(s, surface);
            s->invalidate = 0;
        }
        return true;
    }

    if (s->invalidate) {
        framebuffer_update_memory_section(&s->fbsection, &s->fb_mr, 0,
                                          s->height, src_width);
    }

    framebuffer_update_display(surface, &s->fbsection, s->width, s->height,
                               src_width, dest_width, 0, s->invalidate,
                               apollo_fb_draw_line, s, &first, &last);

    if (first >= 0) {
        qemu_console_update(s->con, 0, first, s->width, last - first + 1);
    }
    s->invalidate = 0;
    return true;
}

static void apollo_fb_invalidate(void *opaque)
{
    ApolloFbState *s = APOLLO_FB(opaque);

    s->invalidate = 1;
}

static const GraphicHwOps apollo_fb_ops = {
    .invalidate = apollo_fb_invalidate,
    .gfx_update = apollo_fb_update,
};

/* ------------------------------------------------------------------ */
/* MCR + Bt458 control window.  These are not wired to the real Apollo */
/* MCR/CCR probe addresses (0x5d800/0x5e800) so the verified PROM       */
/* graphics soft-probe is left undisturbed; the simplefb path never     */
/* touches them and they default to display-enabled.                    */

static uint64_t apollo_fb_ctrl_read(void *opaque, hwaddr addr, unsigned size)
{
    ApolloFbState *s = opaque;

    switch (addr) {
    case 0x00:                          /* MCR */
        return s->mcr;
    case 0x04:                          /* Bt458 address register */
        return s->bt458_addr;
    case 0x08:                          /* Bt458 CLUT data (auto-increment) */
        return s->bt458_clut[s->bt458_addr][s->bt458_sub];
    default:
        return 0;
    }
}

static void apollo_fb_ctrl_write(void *opaque, hwaddr addr, uint64_t val,
                                 unsigned size)
{
    ApolloFbState *s = opaque;

    switch (addr) {
    case 0x00:                          /* MCR */
        s->mcr = val;
        s->invalidate = 1;
        break;
    case 0x04:                          /* Bt458 address register */
        s->bt458_addr = val;
        s->bt458_sub = 0;
        break;
    case 0x08:                          /* Bt458 CLUT data */
        s->bt458_clut[s->bt458_addr][s->bt458_sub] = val;
        if (++s->bt458_sub == 3) {
            s->bt458_sub = 0;
            s->bt458_addr++;
        }
        break;
    default:
        break;
    }
}

static const MemoryRegionOps apollo_fb_ctrl_ops = {
    .read = apollo_fb_ctrl_read,
    .write = apollo_fb_ctrl_write,
    .impl = { .min_access_size = 1, .max_access_size = 4 },
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .endianness = DEVICE_BIG_ENDIAN,
};

static void apollo_fb_realize(DeviceState *dev, Error **errp)
{
    ApolloFbState *s = APOLLO_FB(dev);

    s->width = APOLLO_FB_WIDTH;
    s->height = APOLLO_FB_HEIGHT;
    s->stride = APOLLO_FB_STRIDE;
    s->mcr = APOLLO_FB_MCR_ENABLE;      /* display enabled at reset */
    s->invalidate = 1;

    memory_region_init_ram(&s->fb_mr, OBJECT(dev), "apollo.fb",
                           APOLLO_FB_WINDOW, &error_fatal);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->fb_mr);

    memory_region_init_io(&s->ctrl_mr, OBJECT(dev), &apollo_fb_ctrl_ops, s,
                          "apollo.fb-ctrl", APOLLO_FB_CTRL_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->ctrl_mr);

    s->con = qemu_graphic_console_create(dev, 0, &apollo_fb_ops, s);
    qemu_console_resize(s->con, s->width, s->height);
}

static const VMStateDescription vmstate_apollo_fb = {
    .name = "apollo-fb",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8(mcr, ApolloFbState),
        VMSTATE_UINT8(bt458_addr, ApolloFbState),
        VMSTATE_UINT8(bt458_sub, ApolloFbState),
        VMSTATE_UINT8_2DARRAY(bt458_clut, ApolloFbState, 256, 3),
        VMSTATE_END_OF_LIST()
    },
};

static void apollo_fb_reset(DeviceState *dev)
{
    ApolloFbState *s = APOLLO_FB(dev);

    s->mcr = APOLLO_FB_MCR_ENABLE;
    s->bt458_addr = 0;
    s->bt458_sub = 0;
    s->invalidate = 1;
}

static void apollo_fb_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    set_bit(DEVICE_CATEGORY_DISPLAY, dc->categories);
    dc->realize = apollo_fb_realize;
    dc->vmsd = &vmstate_apollo_fb;
    device_class_set_legacy_reset(dc, apollo_fb_reset);
}

static const TypeInfo apollo_fb_info = {
    .name = TYPE_APOLLO_FB,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ApolloFbState),
    .class_init = apollo_fb_class_init,
};

static void apollo_fb_register_types(void)
{
    type_register_static(&apollo_fb_info);
}

type_init(apollo_fb_register_types)
