/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * DEC 21174 ("Pyxis") PCI host bridge emulation.
 */

#ifndef _ALPHA_PYXIS_INTERNAL_H
#define _ALPHA_PYXIS_INTERNAL_H

#include "qemu/osdep.h"
#include "qom/object.h"
#include "hw/alpha/pyxis.h"
#include "hw/alpha/pyxis-regs.h"
#include "hw/core/sysbus.h"
#include "hw/pci/pci_host.h"

#define TYPE_PYXIS_PCI_HOST_BRIDGE "pyxis-pci-host-bridge"
OBJECT_DECLARE_SIMPLE_TYPE(PyxisPchipState, PYXIS_PCI_HOST_BRIDGE)

OBJECT_DECLARE_SIMPLE_TYPE(PyxisPLDState, PYXIS_PLD)

#define PYXIS_NIRQS             8
#define PYXIS_CPU_MAX           4
#define PYXIS_DRAM_SIZE_MIN     16      /* MiB */
#define PYXIS_DRAM_SIZE_MAX     1024    /* MiB */

enum {
    PYXIS_IRQ_STAT_DEVICE = 0,
    PYXIS_IRQ_STAT_DEVICE_LAST = PYXIS_NIRQS - 1,
    PYXIS_IRQ_STAT_COUNT,
};

/*
 * IOMMU definitions.
 */

#define TYPE_PYXIS_IOMMU_MEMORY_REGION "pyxis-iommu-memory-region"
DECLARE_INSTANCE_CHECKER(IOMMUMemoryRegion, PYXIS_IOMMU_MEMORY_REGION,
                         TYPE_PYXIS_IOMMU_MEMORY_REGION)

/* Scatter-gather PTE (HRM 6.14) */
#define PYXIS_PTE_V             0x0000000000000001
#define PYXIS_PTE_MASK          MAKE_64BIT_MASK(1, 21)
#define PYXIS_PAGE_SHIFT        13
#define PYXIS_PAGE_MASK         MAKE_64BIT_MASK(0, PYXIS_PAGE_SHIFT)
#define PYXIS_SG_MASK           MAKE_64BIT_MASK(13, 7)
#define PYXIS_MWIN_MASK         MAKE_64BIT_MASK(0, 35)

typedef enum PyxisIOMMUStatus {
    IOMMU_STATUS_ABORT,
    IOMMU_STATUS_SUCCESS,
} PyxisIOMMUStatus;

typedef struct PyxisIOTLBEntry {
    IOMMUTLBEntry entry;
} PyxisIOTLBEntry;

static inline bool pyxis_pte_valid(uint64_t pte)
{
    return pte & PYXIS_PTE_V;
}

static inline hwaddr pyxis_pte_address(uint64_t pte)
{
    return (pte & PYXIS_PTE_MASK) << 12;
}

static inline dma_addr_t pyxis_iotlb_translate(PyxisIOTLBEntry *ent,
                                               dma_addr_t addr)
{
    return ent->entry.translated_addr + (addr & ent->entry.addr_mask);
}

typedef struct PyxisDMAWindow {
    uint32_t w_base;
    uint32_t w_mask;
    uint32_t t_base;
} PyxisDMAWindow;

struct PyxisPchipState {
    /*< private >*/
    PCIHostState parent_obj;

    /*< public >*/
    PyxisState *upstream;

    MemoryRegion reg_iack;
    MemoryRegion reg_conf;          /* sparse config, 0x8700000000 */
    MemoryRegion reg_conf_dense[2]; /* INT8 config, 0x8A/0x8B */
    MemoryRegion reg_mem;           /* dense PCI memory, 0x8800000000 */
    MemoryRegion reg_io;            /* dense PCI I/O, 0x8900000000 */
    MemoryRegion reg_mem_dense;     /* alias of reg_mem at 0x8600000000 */
    MemoryRegion reg_io_dense;      /* alias of reg_io at 0x8900000000 */
    MemoryRegion reg_mem_sparse[3]; /* sparse memory regions 0-2 */
    MemoryRegion reg_io_sparse[2];  /* sparse I/O regions A/B */

    IOMMUMemoryRegion iommu;
    AddressSpace iommu_as;
    AddressSpace mem_as;            /* over reg_mem (dense PCI mem) */
    AddressSpace io_as;             /* over reg_io (PCI I/O space)  */

    PyxisDMAWindow win[4];
    uint32_t w_dac;

    /* Raw PCI interrupt line state, as reported to the board PLD. */
    uint32_t pci_irq_level;
};

struct PyxisState {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    PyxisPchipState pchip;

    /* MMIO container, mapped by the machine at PYXIS_IO_BASE. */
    MemoryRegion iomem;
    /* Chipset CSR groups: csr, mctl, pa, misc, pwr, irq. */
    MemoryRegion csr_regs[6];
    MemoryRegion dummy;
    /* INT8 alias of the PCI dense memory window (HRM table 6-2). */
    MemoryRegion mem_int8;

    /* RAM size in MiB; only used for validation. */
    uint32_t ram_size;
    uint32_t num_cpus;
    uint32_t revision;

    /* General CSRs. */
    uint32_t pci_lat;
    uint32_t ctrl;
    uint32_t ctrl1;
    uint32_t flash_ctrl;
    uint32_t hae_mem;
    uint32_t hae_io;
    uint32_t cfg;
    uint32_t diag;
    uint32_t diag_check;
    uint32_t perf_monitor;
    uint32_t perf_control;
    uint32_t err;
    uint32_t err_mask;
    uint32_t syn;
    uint32_t err_data;

    /* Memory controller CSR storage. */
    uint32_t mctl[0x400];

    /* Interrupt control CSRs. */
    uint32_t int_mask;
    uint32_t int_hilo;
    uint32_t int_route;
    uint32_t gpo;
    uint32_t int_cnfg;
    uint32_t int_time;
    uint32_t iic_ctrl;

    /* Miscellaneous CSRs. */
    uint32_t ccr;
    uint32_t clk_stat;
    uint32_t reset_reg;

    /* External interrupt shift register inputs (active high). */
    bool irq_level[PYXIS_NIRQS];
    bool sio_level;

    /* Interrupt PLD (MACH210A) mask at ISA ports 0x804-0x806. */
    uint32_t pld_mask;

    qemu_irq cpu_irq[4];
    uint64_t irq_counts[PYXIS_IRQ_STAT_COUNT];
};

struct PyxisPLDState {
    /*< private >*/
    ISADevice parent_obj;

    /*< public >*/
    PyxisState *upstream;
    MemoryRegion io;
};

#endif /* _ALPHA_PYXIS_INTERNAL_H */
