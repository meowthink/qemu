/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * DEC 21172 core logic chipset ("Alcor") emulation.
 */

#ifndef _ALPHA_ALCOR_H
#define _ALPHA_ALCOR_H

#include "hw/pci/pci_bus.h"
#include "qemu/units.h"
#include "qom/object.h"

#define TYPE_ALCOR_CHIPSET "alcor-chipset"
OBJECT_DECLARE_SIMPLE_TYPE(AlcorState, ALCOR_CHIPSET)

#define TYPE_ALCOR_PLD "alcor-pld"

PCIBus *alcor_get_pci_bus(DeviceState *dev);

/*
 * Alcor base address definitions.
 *
 * 40-bit physical address map of the 21172 (TRM 4.1; the AlphaPC 164
 * TRM table A-1 lists the same 87.xxxx.xxxx layout).
 * Byte/word mode disabled (IOA_BEN=0) is the power-on state; in that
 * state only the sparse windows and the 87.xxxx.xxxx CSR/conf ranges
 * exist.  Byte/word mode (IOA_BEN=1) adds the dense INT8 windows.
 */

#define ALCOR_IO_BASE           0x8000000000ULL

#define ALCOR_MEM_SPARSE0       0x8000000000ULL   /* 16GB -> 512MB PCI */
#define ALCOR_MEM_SPARSE1       0x8400000000ULL   /* 4GB  -> 128MB PCI */
#define ALCOR_MEM_SPARSE2       0x8500000000ULL   /* 2GB  -> 64MB PCI  */
#define ALCOR_IO_SPARSE_A       0x8580000000ULL   /* 1GB  -> 32MB I/O  */
#define ALCOR_IO_SPARSE_B       0x85C0000000ULL   /* 1GB  -> 32MB I/O  */
#define ALCOR_CONF_SPARSE       0x8700000000ULL   /* PCI config (sparse) */
#define ALCOR_SPECIAL           0x8720000000ULL   /* special/IACK cycles */
#define ALCOR_CSR               0x8740000000ULL   /* general CSRs */
#define ALCOR_MCTL              0x8750000000ULL   /* memory controller */
#define ALCOR_PA                0x8760000000ULL   /* PCI windows + SG */
#define ALCOR_MISC              0x8780000000ULL   /* clock/reset */
#define ALCOR_PWR               0x8790000000ULL   /* power management */
#define ALCOR_IRQ               0x87A0000000ULL   /* interrupt control */
#define ALCOR_FLASH             0x87C0000000ULL   /* flash, byte accesses */
#define ALCOR_MEM_DENSE         0x8600000000ULL   /* 4GB PCI dense memory */
#define ALCOR_MEM_INT8          0x8800000000ULL   /* 4GB PCI memory space INT8 */
#define ALCOR_IO_DENSE          0x8900000000ULL   /* 4GB PCI I/O space INT8 */
#define ALCOR_CONF_DENSE0       0x8A00000000ULL   /* config, type 0 */
#define ALCOR_CONF_DENSE1       0x8B00000000ULL   /* config, type 1 */
#define ALCOR_FLASH_INT8        0xC7C0000000ULL   /* flash, INT8 mode */
#define ALCOR_DUMMY             0xE000000000ULL   /* dummy memory region */

#define ALCOR_FLASH_SIZE        (1 * MiB)

#define ALCOR_REGION(region)    (ALCOR_##region - ALCOR_IO_BASE)

#endif /* _ALPHA_ALCOR_H */
