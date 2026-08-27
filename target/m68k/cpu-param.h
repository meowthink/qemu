/*
 * m68k cpu parameters for qemu.
 *
 * Copyright (c) 2005-2007 CodeSourcery
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#ifndef M68K_CPU_PARAM_H
#define M68K_CPU_PARAM_H

/*
 * Coldfire Linux uses 8k pages and m68k Linux uses 4k pages, but the
 * 68030/MC68851 PMMU supports much smaller pages and real systems use
 * them: Apollo Domain/OS on the DN3000 runs the MC68851 with 1 KiB
 * pages whose page frames are not 4 KiB aligned.  QEMU's softmmu maps a
 * whole TARGET_PAGE at a single physical base, so a TARGET_PAGE coarser
 * than the guest page silently drops the low physical-address bits of a
 * sub-page frame (frame 0x101800 rounds to 0x101000 in a 4 KiB page,
 * corrupting every translation in that page).  Use a 1 KiB TARGET_PAGE
 * so the softmmu granularity is never coarser than the guest MMU page;
 * larger guest pages (4 KiB/8 KiB) still work as multi-page mappings.
 */
#define TARGET_PAGE_BITS 10
#define TARGET_VIRT_ADDR_SPACE_BITS 32

#endif
