/*
 * Apple "Bandit" ARBus-to-PCI bridge (343S1126) -- the TNT (Power Surge)
 * family's PCI host bridge.  The ANS has two: bus 0 at $F2000000 and bus 1
 * at $F4000000.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_PCI_HOST_BANDIT_H
#define HW_PCI_HOST_BANDIT_H

#include "hw/pci/pci_host.h"

#define TYPE_BANDIT_PCI_HOST_BRIDGE "bandit-pcihost"
OBJECT_DECLARE_SIMPLE_TYPE(BanditState, BANDIT_PCI_HOST_BRIDGE)

struct BanditState {
    PCIHostState parent_obj;

    uint32_t config_reg;
    uint32_t mode_select;
    uint32_t small_win;          /* PCI/CPU base of the 16 MB window */
    uint32_t large_win;          /* PCI/CPU base of the 256 MB window */
    MemoryRegion config_addr;    /* +0x800000 */
    MemoryRegion config_data;    /* +0xC00000 */
    MemoryRegion pci_mem;        /* the PCI memory space itself */
    MemoryRegion pci_io;         /* 8 MB PCI I/O window at the bridge base */
    MemoryRegion win_small;      /* 16 MB identity window at the bridge */
    MemoryRegion win_large;      /* 256 MB identity window at 0x80000000 */
    qemu_irq irqs[4];
};

#endif /* HW_PCI_HOST_BANDIT_H */
