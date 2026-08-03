/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * DEC 21174 ("Pyxis") PCI host bridge emulation.
 */

#ifndef _ALPHA_PYXIS_H
#define _ALPHA_PYXIS_H

#include "hw/pci/pci_bus.h"
#include "qemu/units.h"
#include "qom/object.h"

#define TYPE_PYXIS_CHIPSET "pyxis-chipset"
OBJECT_DECLARE_SIMPLE_TYPE(PyxisState, PYXIS_CHIPSET)

#define TYPE_PYXIS_PLD "pyxis-pld"

PCIBus *pyxis_get_pci_bus(DeviceState *dev);

/*
 * Pyxis base address definitions.
 *
 * 40-bit physical address map of the 21174 (HRM 6.1, tables 6-1/6-2).
 * Byte/word mode disabled (IOA_BEN=0) is the power-on state; in that
 * state only the sparse windows and the 87.xxxx.xxxx CSR/conf ranges
 * exist.  Byte/word mode (IOA_BEN=1) adds the dense INT8 windows.
 */

#define PYXIS_IO_BASE           0x8000000000ULL

#define PYXIS_MEM_SPARSE0       0x8000000000ULL   /* 16GB -> 512MB PCI */
#define PYXIS_MEM_SPARSE1       0x8400000000ULL   /* 4GB  -> 128MB PCI */
#define PYXIS_MEM_SPARSE2       0x8500000000ULL   /* 2GB  -> 64MB PCI  */
#define PYXIS_IO_SPARSE_A       0x8580000000ULL   /* 1GB  -> 32MB I/O  */
#define PYXIS_IO_SPARSE_B       0x85C0000000ULL   /* 1GB  -> 32MB I/O  */
#define PYXIS_CONF_SPARSE       0x8700000000ULL   /* PCI config (sparse) */
#define PYXIS_SPECIAL           0x8720000000ULL   /* special/IACK cycles */
#define PYXIS_CSR               0x8740000000ULL   /* general CSRs */
#define PYXIS_MCTL              0x8750000000ULL   /* memory controller */
#define PYXIS_PA                0x8760000000ULL   /* PCI windows + SG */
#define PYXIS_MISC              0x8780000000ULL   /* clock/reset */
#define PYXIS_PWR               0x8790000000ULL   /* power management */
#define PYXIS_IRQ               0x87A0000000ULL   /* interrupt control */
#define PYXIS_FLASH             0x87C0000000ULL   /* flash, byte accesses */
#define PYXIS_MEM_DENSE         0x8600000000ULL   /* 4GB PCI dense memory (HRM 6-1) */
#define PYXIS_MEM_INT8          0x8800000000ULL   /* 4GB PCI memory space INT8 (HRM 6-2) */
#define PYXIS_IO_DENSE          0x8900000000ULL   /* 4GB PCI I/O space INT8 */
#define PYXIS_CONF_DENSE0       0x8A00000000ULL   /* config, type 0 */
#define PYXIS_CONF_DENSE1       0x8B00000000ULL   /* config, type 1 */
#define PYXIS_FLASH_INT8        0xC7C0000000ULL   /* flash, INT8 mode */
#define PYXIS_DUMMY             0xE000000000ULL   /* dummy memory region */

#define PYXIS_FLASH_SIZE        (1 * MiB)

#define PYXIS_REGION(region)    (PYXIS_##region - PYXIS_IO_BASE)

#endif /* _ALPHA_PYXIS_H */
