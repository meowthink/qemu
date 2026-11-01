/*
 * Apple Network Server's Grand Central (343S1125) with the GBUS island.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_MISC_ANS_GC_H
#define HW_MISC_ANS_GC_H

#include "hw/pci/pci_device.h"
#include "hw/ppc/mac_dbdma.h"
#include "hw/char/escc.h"
#include "hw/misc/macio/macio.h"
#include "hw/misc/macio/cuda.h"
#include "hw/scsi/esp.h"

#define TYPE_ANS_GC "ans-gc"
OBJECT_DECLARE_SIMPLE_TYPE(AnsGCState, ANS_GC)

#define ANS_GC_NVRAM_SIZE 0x2000

/* Unit offsets of the GC device island (dev notes 4.6.1). */
#define ANS_GC_OFF_SCSI    0x10000   /* 53C94: registers every 16 bytes */
#define ANS_GC_OFF_SCC     0x13000   /* ESCC: ch B at +0, ch A at +0x20 */
#define ANS_GC_OFF_EPROM   0x19000
#define ANS_GC_OFF_BREG1   0x1a000
#define ANS_GC_OFF_LCD     0x1c000
#define ANS_GC_OFF_NVCTL   0x1d000
#define ANS_GC_OFF_BREG2   0x1e000
#define ANS_GC_OFF_NVDAT   0x1f000

/*
 * Grand Central's interrupt input lines (the Mac TNT map).  The ANS keeps
 * the TNT numbering for the island sources and re-purposes the EXT lines for
 * the PCI side (dev notes 4.2): EXT2/EXT6 are the two fast/wide SCSI
 * interrupts, EXT3-5 and EXT7 the slots, and EXT1 both bridges.
 */
#define ANS_GC_INT_ERROR   0x15
#define ANS_GC_INT_FW0     0x16
#define ANS_GC_INT_SLOT1   0x17
#define ANS_GC_INT_SLOT2   0x18
#define ANS_GC_INT_SLOT3   0x19
#define ANS_GC_INT_FW1     0x1a
#define ANS_GC_INT_SLOT4   0x1b

struct AnsGCState {
    PCIDevice parent_obj;

    MemoryRegion bar;
    MemoryRegion control;
    /* The island's DMA engine at +0x8000, Mac TNT channel layout. */
    DBDMAState dbdma;
    MemoryRegion eprom;
    MemoryRegion breg1;
    MemoryRegion lcd;
    MemoryRegion nvram_ctl;
    MemoryRegion breg2;
    MemoryRegion nvram_data;
    MemoryRegion scc_legacy;

    /*
     * The serial ports are the one mac-io-family child we can borrow whole
     * (the SCC island at +0x13000 has the same 16-byte-centre register
     * layout the mac-io's ESCC uses).
     */
    ESCCState escc;
    MacIOBusState escc_bus;
    SysBusESPState esp;
    CUDAState cuda;
    /*
     * Grand Central's own interrupt block (not the Heathrow PIC): a single
     * 32-bit register set at +$20 (events), +$24 (mask), +$28 (clear) and
     * +$2C (levels), little-endian.
     */
    uint32_t int_events;
    uint32_t int_mask;
    uint32_t int_levels;
    uint32_t int_latch;
    bool int_mode1;
    MemoryRegion intc;
    qemu_irq pic_out;

    uint32_t control_reg[0x1000 / 4];
    uint8_t eprom_data[0x100];
    uint16_t breg1_val;
    uint16_t breg2_val;
    /* "normal", "service" or "locked": the board's front keyswitch. */
    char *keyswitch;
    bool two_supplies;
    /* power-on store content: false = zeros, true = erased (0xff) */
    bool nvram_erased;
    uint8_t lcd_cmd;
    uint8_t lcd_data;
    int lcd_cursor;
    uint8_t lcd_ddram[0x80];
    uint16_t nvram_addr;
    uint8_t nvram[ANS_GC_NVRAM_SIZE];
};

#endif /* HW_MISC_ANS_GC_H */
