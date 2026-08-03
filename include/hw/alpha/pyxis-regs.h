/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * DEC 21174 ("Pyxis") register definitions.
 */

#ifndef _ALPHA_PYXIS_REGS_H
#define _ALPHA_PYXIS_REGS_H

#include "qemu/bitops.h"

/*
 * Register offsets within the 21174 CSR groups (HRM 4.3-4.8).
 */

/* General CSRs, base 0x8740000000 */
#define PYXIS_REV               0x0080
#define PCI_LAT                 0x00c0
#define PYXIS_CTRL              0x0100
#define PYXIS_CTRL1             0x0140
#define FLASH_CTRL              0x0200
#define HAE_MEM                 0x0400
#define HAE_IO                  0x0440
#define CFG                     0x0480
#define PYXIS_DIAG              0x2000
#define DIAG_CHECK              0x3000
#define PERF_MONITOR            0x4000
#define PERF_CONTROL            0x4040
#define PYXIS_ERR               0x8200
#define PYXIS_STAT              0x8240
#define ERR_MASK                0x8280
#define PYXIS_SYN               0x8300
#define PYXIS_ERR_DATA          0x8308
#define MEAR                    0x8400
#define MESR                    0x8440
#define PCI_ERR0                0x8800
#define PCI_ERR1                0x8840
#define PCI_ERR2                0x8880

/* Memory controller CSRs, base 0x8750000000 */
#define MCR                     0x0000
#define MCMR                    0x0040
#define GTR                     0x0200
#define RTR                     0x0300
#define RHPR                    0x0400
#define MDR1                    0x0500
#define MDR2                    0x0540
#define BBAR(n)                 (0x0600 + 0x40 * (n))
#define BCR(n)                  (0x0800 + 0x40 * (n))
#define BTR(n)                  (0x0a00 + 0x40 * (n))
#define CVM                     0x0c00

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

/* Interrupt control CSRs, base 0x87A0000000 */
#define INT_REQ                 0x0000
#define INT_MASK                0x0040
#define INT_HILO                0x00c0
#define INT_ROUTE               0x0140
#define GPO                     0x0180
#define INT_CNFG                0x01c0
#define RT_COUNT                0x0200
#define INT_TIME                0x0240
#define IIC_CTRL                0x02c0

/* Window base register fields (HRM 5.6.2) */
#define W_EN                    BIT(0)
#define W_SG                    BIT(1)
#define W0_MEMCS_EN             BIT(2)
#define W3_DAC_EN               BIT(3)
#define W_BASE_ADDR_MASK        0xfff00000

#endif /* _ALPHA_PYXIS_REGS_H */
