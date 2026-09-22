/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * DEC 21172 core logic chipset ("Alcor") register definitions.
 *
 * 21172-CA (CIA) plus four 21172-BA (DSW) slices; only the CIA has
 * registers.
 */

#ifndef _ALPHA_ALCOR_REGS_H
#define _ALPHA_ALCOR_REGS_H

#include "qemu/bitops.h"

/* Register offsets (21172 TRM EC-QUQJA-TE, chapter 4). */

/* General CSRs, base 0x8740000000 */
#define CIA_REV                 0x0080
#define PCI_LAT                 0x00c0
#define CIA_CTRL                0x0100
#define CIA_CNFG                0x0140
#define HAE_MEM                 0x0400
#define HAE_IO                  0x0440
#define CFG                     0x0480
#define CACK_EN                 0x0600
#define CIA_DIAG                0x2000
#define DIAG_CHECK              0x3000
#define PERF_MONITOR            0x4000
#define PERF_CONTROL            0x4040
#define CPU_ERR0                0x8000
#define CPU_ERR1                0x8040
#define CIA_ERR                 0x8200
#define CIA_STAT                0x8240
#define ERR_MASK                0x8280
#define CIA_SYN                 0x8300
#define MEM_ERR0                0x8400
#define MEM_ERR1                0x8440
#define PCI_ERR0                0x8800
#define PCI_ERR1                0x8840
#define PCI_ERR2                0x8880

/* Memory controller CSRs, base 0x8750000000 */
#define MCR                     0x0000
#define MBA(n)                  (0x0600 + 0x80 * (n))  /* n = 0,2,4,6,8,a,c,e */
#define TMG0                    0x0b00
#define TMG1                    0x0b40
#define TMG2                    0x0b80

/* PCI window control CSRs, base 0x8760000000 */
#define TBIA                    0x0100
#define Wn_BASE(n)              (0x0400 + 0x100 * (n))
#define Wn_MASK(n)              (0x0440 + 0x100 * (n))
#define Tn_BASE(n)              (0x0480 + 0x100 * (n))
#define W_DAC                   0x07c0
#define LTB_TAG(n)              (0x0800 + 0x40 * (n))
#define TB_TAG(n)               (0x0900 + 0x40 * (n))
#define TB_PAGE(m, n)           (0x1000 + 0x40 * (4 * (m) + (n)))

/* Miscellaneous CSRs, base 0x8780000000 */
#define CCR                     0x0000
#define CLK_STAT                0x0100
#define RESET                   0x0900

/* Backing store for the miscellaneous CSR block (4.7). */
#define ALCOR_MISC_REGS         0x400

/* Power-up CCR (table 5-51): CLK/PCLK divide 1/3, PLL range 2, long reset,
 * DCLK forced, DRAM clock delay 0x18. */
#define ALCOR_CCR_RESET         ((0x18u << 24) | (1u << 17) | (1u << 10) | \
                                 (2u << 8) | (3u << 4) | 1u)

/*
 * Interrupt control CSRs, base 0x87A0000000.  Used as the PLD-to-CPU
 * interrupt router; a 21172 board has no chipset interrupt block.
 */
#define INT_REQ                 0x0000
#define INT_MASK                0x0040
#define INT_HILO                0x00c0
#define INT_ROUTE               0x0140
#define GPO                     0x0180
#define INT_CNFG                0x01c0
#define RT_COUNT                0x0200
#define INT_TIME                0x0240
#define IIC_CTRL                0x02c0

/* Window base register fields */
#define W_EN                    BIT(0)
#define W_SG                    BIT(1)
#define W0_MEMCS_EN             BIT(2)
#define W3_DAC_EN               BIT(3)
#define W_BASE_ADDR_MASK        0xfff00000

#endif /* _ALPHA_ALCOR_REGS_H */
