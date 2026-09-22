/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * DEC 21172 core logic chipset ("Alcor") emulation - internal definitions.
 */

#ifndef _ALPHA_ALCOR_INTERNAL_H
#define _ALPHA_ALCOR_INTERNAL_H

#include "qemu/osdep.h"
#include "qom/object.h"
#include "hw/alpha/alcor.h"
#include "hw/alpha/alcor-regs.h"
#include "hw/core/sysbus.h"
#include "hw/pci/pci_host.h"

#define TYPE_ALCOR_PCI_HOST_BRIDGE "alcor-pci-host-bridge"
OBJECT_DECLARE_SIMPLE_TYPE(AlcorPchipState, ALCOR_PCI_HOST_BRIDGE)

OBJECT_DECLARE_SIMPLE_TYPE(AlcorPLDState, ALCOR_PLD)

#define ALCOR_NIRQS             8
#define ALCOR_CPU_MAX           4
#define ALCOR_DRAM_SIZE_MIN     16      /* MiB */
#define ALCOR_DRAM_SIZE_MAX     1024    /* MiB */

enum {
    ALCOR_IRQ_STAT_DEVICE = 0,
    ALCOR_IRQ_STAT_DEVICE_LAST = ALCOR_NIRQS - 1,
    ALCOR_IRQ_STAT_COUNT,
};

/*
 * IOMMU definitions.
 */

#define TYPE_ALCOR_IOMMU_MEMORY_REGION "alcor-iommu-memory-region"
DECLARE_INSTANCE_CHECKER(IOMMUMemoryRegion, ALCOR_IOMMU_MEMORY_REGION,
                         TYPE_ALCOR_IOMMU_MEMORY_REGION)

/* Scatter-gather PTE */
#define ALCOR_PTE_V             0x0000000000000001
#define ALCOR_PTE_MASK          MAKE_64BIT_MASK(1, 21)
#define ALCOR_PAGE_SHIFT        13
#define ALCOR_PAGE_MASK         MAKE_64BIT_MASK(0, ALCOR_PAGE_SHIFT)
#define ALCOR_SG_MASK           MAKE_64BIT_MASK(13, 7)
#define ALCOR_MWIN_MASK         MAKE_64BIT_MASK(0, 35)

typedef enum AlcorIOMMUStatus {
    IOMMU_STATUS_ABORT,
    IOMMU_STATUS_SUCCESS,
} AlcorIOMMUStatus;

typedef struct AlcorIOTLBEntry {
    IOMMUTLBEntry entry;
} AlcorIOTLBEntry;

static inline bool alcor_pte_valid(uint64_t pte)
{
    return pte & ALCOR_PTE_V;
}

static inline hwaddr alcor_pte_address(uint64_t pte)
{
    return (pte & ALCOR_PTE_MASK) << 12;
}

static inline dma_addr_t alcor_iotlb_translate(AlcorIOTLBEntry *ent,
                                               dma_addr_t addr)
{
    return ent->entry.translated_addr + (addr & ent->entry.addr_mask);
}

typedef struct AlcorDMAWindow {
    uint32_t w_base;
    uint32_t w_mask;
    uint32_t t_base;
} AlcorDMAWindow;

/*
 * Sparse-space window: the HAE_MEM/HAE_IO registers relocate the window
 * base, so each window needs to know which region it decodes for.
 */
typedef struct AlcorSparseWindow {
    struct AlcorPchipState *pcs;
    unsigned region;
} AlcorSparseWindow;

struct AlcorPchipState {
    /*< private >*/
    PCIHostState parent_obj;

    /*< public >*/
    AlcorState *upstream;

    MemoryRegion reg_iack;
    MemoryRegion reg_conf;          /* sparse config, 0x8700000000 */
    MemoryRegion reg_conf_dense[2]; /* INT8 config, 0x8A/0x8B */
    MemoryRegion reg_mem;           /* dense PCI memory, 0x8800000000 */
    MemoryRegion reg_io;            /* dense PCI I/O, 0x8900000000 */
    MemoryRegion reg_mem_dense;     /* alias of reg_mem at 0x8600000000 */
    MemoryRegion reg_io_dense;      /* alias of reg_io at 0x8900000000 */
    MemoryRegion reg_mem_sparse[3]; /* sparse memory regions 0-2 */
    MemoryRegion reg_io_sparse[2];  /* sparse I/O regions A/B */
    AlcorSparseWindow mem_win[3];
    AlcorSparseWindow io_win[2];

    IOMMUMemoryRegion iommu;
    AddressSpace iommu_as;
    AddressSpace mem_as;            /* over reg_mem (dense PCI mem) */
    AddressSpace io_as;             /* over reg_io (PCI I/O space)  */

    AlcorDMAWindow win[4];
    uint32_t w_dac;

    /* Raw PCI interrupt line state, as reported to the board PLD. */
    uint32_t pci_irq_level;
};

struct AlcorState {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    AlcorPchipState pchip;

    /* MMIO container, mapped by the machine at ALCOR_IO_BASE. */
    MemoryRegion iomem;
    /* Chipset CSR groups: csr, mctl, pa, misc, pwr, irq. */
    MemoryRegion csr_regs[6];
    MemoryRegion dummy;
    /* INT8 alias of the PCI dense memory window. */
    MemoryRegion mem_int8;

    /* RAM size in MiB; only used for validation. */
    uint32_t ram_size;
    uint32_t num_cpus;
    uint32_t revision;

    /* General CSRs (21172 TRM 4.2). */
    uint32_t pci_lat;
    uint32_t ctrl;
    uint32_t cnfg;
    uint32_t cack_en;
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

    /* Memory configuration CSRs (21172 TRM 4.6): MCR, MBA0..MBAE, TMG0-2. */
    uint32_t mctl[0x400];

    /* Interrupt control CSRs. */
    uint32_t int_mask;
    uint32_t int_hilo;
    uint32_t int_route;
    uint32_t gpo;
    uint32_t int_cnfg;
    uint32_t int_time;
    uint32_t iic_ctrl;

    /* Miscellaneous CSRs (4.7): CCR, CLK_STAT, RESET and reserved space. */
    uint32_t misc_regs[ALCOR_MISC_REGS];

    /* External interrupt shift register inputs (active high). */
    bool irq_level[ALCOR_NIRQS];
    bool sio_level;

    /* Interrupt PLD (MACH210A) mask at ISA ports 0x804-0x806. */
    uint32_t pld_mask;

    qemu_irq cpu_irq[4];
    uint64_t irq_counts[ALCOR_IRQ_STAT_COUNT];
};

struct AlcorPLDState {
    /*< private >*/
    ISADevice parent_obj;

    /*< public >*/
    AlcorState *upstream;
    MemoryRegion io;
};

#endif /* _ALPHA_ALCOR_INTERNAL_H */
