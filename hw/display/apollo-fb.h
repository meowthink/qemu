/*
 * Apollo DN3000 display: linear framebuffer presented to the guest as a
 * "simple-framebuffer" (xrgb8888), scanned out to the QEMU display.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_DISPLAY_APOLLO_FB_H
#define HW_DISPLAY_APOLLO_FB_H

#include "qom/object.h"

#define TYPE_APOLLO_FB "apollo-fb"
OBJECT_DECLARE_SIMPLE_TYPE(ApolloFbState, APOLLO_FB)

/*
 * Fixed 1024x800 visible mode (the authentic DN3000 "15i" visible area,
 * research §2.11).  Presented to Linux as a linear xrgb8888 buffer so the
 * in-tree simplefb/simpledrm driver binds without a native DRM driver.
 */
#define APOLLO_FB_WIDTH    1024
#define APOLLO_FB_HEIGHT   800
#define APOLLO_FB_BPP      4                       /* xrgb8888 */
#define APOLLO_FB_STRIDE   (APOLLO_FB_WIDTH * APOLLO_FB_BPP)
#define APOLLO_FB_SIZE     (APOLLO_FB_STRIDE * APOLLO_FB_HEIGHT) /* 0x320000 */

/*
 * Physical placement.  The real MGM/CGM frame stores (0xfa0000 / 0x0a0000)
 * are far too small for a 3.1 MiB xrgb8888 buffer, so the linear scanout
 * buffer lives in a dedicated 4 MiB window well above the 128 MiB RAM
 * aperture (a BAR-like window in the high, otherwise-unmapped physical
 * space, research §1 memory map).  The MCR/Bt458 control window follows it.
 * Both the QEMU device and the Linux arch glue advertise these addresses.
 */
#define APOLLO_FB_BASE       0x10000000
#define APOLLO_FB_WINDOW     0x00400000            /* 4 MiB linear aperture */
#define APOLLO_FB_CTRL_BASE  0x10400000
#define APOLLO_FB_CTRL_SIZE  0x00001000

/* MCR (mono control register) bits we model */
#define APOLLO_FB_MCR_ENABLE 0x01                  /* display enable */

#endif /* HW_DISPLAY_APOLLO_FB_H */
