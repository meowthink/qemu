/*
 * Apollo DN3000 (Motorola 68020 Domain workstation)
 *
 * Phase 1: boot the DN3000 "MD" PROM (md8-rev-7.0) to the Service
 * Mode debugger prompt over the SCN2681 serial console.
 *
 * Register map and chipset behaviour reverse engineered from MAME's
 * src/mame/apollo/{apollo.cpp,apollo.h,apollo_m.cpp} and the Linux
 * m68k Apollo port (arch/m68k/include/asm/apollohw.h):
 *
 *   0x000000-0x007fff  boot PROM (writes ignored)
 *   0x008000           CPU Status Register (16 bit)
 *   0x008100           CPU Control Register (16 bit)
 *   0x008400-0x0087ff  SIO: SCN2681 DUART, regs on 2-byte boundaries
 *                      (ch A keyboard, ch B serial console)
 *   0x008800-0x0088ff  MC6840 PTM (regs on odd bytes)
 *   0x008900-0x0089ff  MC146818 RTC (direct byte-addressed registers)
 *   0x009000/0x009100  Am9517A DMA controllers (stubbed)
 *   0x009200           DMA page registers (stubbed)
 *   0x009300           latch-page-on-parity-error register (stub)
 *   0x009400/0x009500  i8259 PIC master/slave (2 byte regs each)
 *   0x009600           node ID PROM (16-bit regs, byte in high lane)
 *   0x040000-0x05ffff  AT bus I/O   (unmapped: reads 0xff, no fault)
 *   0x080000-0xffffff  AT bus memory (unmapped: reads 0xff, no fault)
 *   0x100000-0x8fffff  RAM (8 MB max)
 *
 * Everything else bus-errors (and sets CSR "CPU timeout"), which the
 * PROM's self-test relies on for RAM sizing and device probing.
 *
 * Interrupts: dual cascaded i8259 (slave INT on master IR3), master
 * INT drives CPU IPL 6 with the 8259-supplied vector; all other
 * levels would autovector (nothing drives them).  IRQ0 = PTM,
 * IRQ1 = SIO, IRQ3 = cascade, IRQ8 = RTC; vector base 0xa0 is
 * programmed into the PICs by the PROM itself.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/cutils.h"
#include "qemu/log.h"
#include "qemu/units.h"
#include "qemu/error-report.h"
#include "qemu/datadir.h"
#include "qapi/error.h"
#include "system/reset.h"
#include "system/system.h"
#include "system/memory.h"
#include "exec/target_page.h"
#include "hw/core/boards.h"
#include "hw/core/loader.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "hw/core/irq.h"
#include "hw/core/sysbus.h"
#include "hw/char/scn2681.h"
#include "hw/isa/isa.h"
#include "hw/isa/i8259_internal.h"
#include "hw/intc/i8259.h"
#include "hw/rtc/mc146818rtc.h"
#include "system/block-backend.h"
#include "system/blockdev.h"
#include "hw/display/apollo-fb.h"
#include "target/m68k/cpu.h"
#include "elf.h"
#include "standard-headers/asm-m68k/bootinfo.h"
#include "bootinfo.h"

/* Apollo-specific bootinfo tag (not in the QEMU standard-headers) */
#define BI_APOLLO_MODEL   0x8000
#define APOLLO_MODEL_DN3000  1

#define APOLLO_ROM_BASE        0x000000
#define APOLLO_ROM_SIZE        0x8000
#define APOLLO_CSR_SR_ADDR     0x008000
#define APOLLO_CSR_CR_ADDR     0x008100
#define APOLLO_SIO_ADDR        0x008400
#define APOLLO_PTM_ADDR        0x008800
#define APOLLO_RTC_ADDR        0x008900
#define APOLLO_DMA1_ADDR       0x009000
#define APOLLO_DMA2_ADDR       0x009100
#define APOLLO_DMAPAGE_ADDR    0x009200
#define APOLLO_PARLATCH_ADDR   0x009300
#define APOLLO_PIC1_ADDR       0x009400
#define APOLLO_PIC2_ADDR       0x009500
#define APOLLO_NODEID_ADDR     0x009600
#define APOLLO_ATBUS_IO_BASE   0x040000
#define APOLLO_ATBUS_IO_SIZE   0x020000
#define APOLLO_ATBUS_MEM_BASE  0x080000
#define APOLLO_ATBUS_MEM_SIZE  0xf80000
#define APOLLO_RAM_BASE        0x100000
/*
 * The real DN3000 tops out at 8 MB of on-board RAM, which is all the
 * PROM path needs.  Booting Linux with an initramfs wants more head
 * room, so allow a larger aperture (the extra RAM simply overlays the
 * otherwise-unmapped high physical space); the PROM is unaffected -
 * it sizes RAM from the fixed config byte, not by probing.
 */
#define APOLLO_RAM_MAX         (128 * MiB)

/*
 * The kernel's head.S early console (debug_cons/serial_putc) is
 * hard-wired to the DN3500/SAU7 SIO and CPU-control offsets (LSRB0
 * 0x10412, LTHRB0 0x10416, LCPUCTRL 0x10100) and runs before the MMU
 * is enabled with iobase 0, i.e. at physical 0x104xx / 0x101xx.  Alias
 * the DUART and the CSR-control register there so earlyprintk works on
 * the DN3000 too (the runtime C code uses the correct SAU8 0x8400 /
 * 0x8100 addresses).
 */
#define APOLLO_SIO_ALIAS_ADDR     0x010400
#define APOLLO_CPUCTRL_ALIAS_ADDR 0x010100

/*
 * DMA address-translation map (SAU7 offset).  Linux's config_apollo()
 * clears 0x400 16-bit entries here unconditionally (addr_xlat_map =
 * IO_BASE + 0x17000) regardless of model, so it must be backed even on
 * the DN3000.  Model it as plain storage (no DMA remapping is wired up).
 */
#define APOLLO_XLAT_ADDR          0x017000
#define APOLLO_XLAT_SIZE          0x001000

/* CPU Status Register bits (read only, write clears) */
#define CSR_SR_SERVICE           0x0001
#define CSR_SR_ATBUS_IO_TIMEOUT  0x0002
#define CSR_SR_FP_TRAP           0x0004
#define CSR_SR_INTERRUPT_PENDING 0x0008 /* DN3000 only */
#define CSR_SR_PARITY_BYTE_MASK  0x00f0
#define CSR_SR_CPU_TIMEOUT       0x0100
#define CSR_SR_ATBUS_MEM_TIMEOUT 0x2000
#define CSR_SR_BIT15             0x8000

/* CPU Control Register bits */
#define CSR_CR_INTERRUPT_ENABLE  0x0001
#define CSR_CR_RESET_DEVICES     0x0002
#define CSR_CR_FPU_TRAP_ENABLE   0x0004
#define CSR_CR_FORCE_BAD_PARITY  0x0008

/* DN3000 8MB RAM configuration byte, presented on the SIO input port */
#define DN3000_RAM_CONFIG_8MB  0x20 /* 2-2-2-2 */

#define APOLLO_NODE_ID_DEFAULT 0x12345

/* ------------------------------------------------------------------ */
/* OMTI-8621 Winchester disk controller (SMS OMTI-8621, Apollo variant) */
/*                                                                      */
/* The DN3000 boot PROM has a built-in Winchester driver that drives   */
/* this controller directly over the AT/ISA bus.  The controller sits  */
/* at ISA I/O 0x1a0 (ESDI base, jumper default), which the Apollo ISA  */
/* I/O address translation                                             */
/*     isa_addr = (off & 3) + ((off & ~0x1ff) >> 7),  off = cpuaddr/2  */
/* places at CPU physical 0x04d000 (verified at runtime: EX AEGIS      */
/* status-polls exactly there).  Ported from MAME                      */
/* src/devices/bus/isa/omti8621.cpp (Ostermeyer/Belmont).              */
/*                                                                      */
/* Data transfers are PIO through the 16-bit data register (word       */
/* access to port 0); the disk path does not use the Am9517A DMA (in   */
/* MAME the DMA/dack path only feeds the floppy).  Disk images are raw */
/* ".awd" containers: 1056-byte sectors (1024 data + 32 overhead)      */
/* stored back-to-back, byte offset = diskaddr * 1056.                 */

#define OMTI_ADDR              0x04d000 /* ISA 0x1a0 after Apollo xlat */
#define OMTI_SECTOR_SIZE       1056
#define OMTI_MAX_BLOCK_COUNT   32
#define OMTI_MAX_LUN           1

/* ISA register ports (byte offset within the 8-byte window) */
#define OMTI_PORT_DATA         0x00 /* R: data/status  W: data/command */
#define OMTI_PORT_STATUS       0x01 /* R: status       W: reset        */
#define OMTI_PORT_CONFIG       0x02 /* R: config       W: select       */
#define OMTI_PORT_MASK         0x03 /* W: mask (DMA/INT enable)        */

/* Status register bits */
#define OMTI_STATUS_REQ        0x01
#define OMTI_STATUS_IO         0x02 /* 1 = controller -> host */
#define OMTI_STATUS_CD         0x04 /* 1 = command/status byte */
#define OMTI_STATUS_BUSY       0x08
#define OMTI_STATUS_DREQ       0x10
#define OMTI_STATUS_IREQ       0x20 /* command complete interrupt */
#define OMTI_STATUS_NU6        0x40
#define OMTI_STATUS_NU7        0x80

/* Config (jumper) bits */
#define OMTI_CONFIG_W20        0x08
#define OMTI_CONFIG_W22        0x02
#define OMTI_CONFIG_W23        0x01

/* Mask register bits */
#define OMTI_MASK_DMAE         0x01
#define OMTI_MASK_INTE         0x02

/* command status */
#define OMTI_CMDSTATUS_ERROR   0x02
#define OMTI_CMDSTATUS_LUN     0x20

/* sense codes */
#define OMTI_SENSE_NO_ERROR        0x00
#define OMTI_SENSE_DRIVE_NOT_READY 0x04
#define OMTI_SENSE_ADDRESS_VALID   0x80
#define OMTI_SENSE_ILLEGAL_ADDRESS 0x21
#define OMTI_SENSE_INVALID_COMMAND 0x20

/* commands */
#define OMTI_CMD_TEST_DRIVE_READY   0x00
#define OMTI_CMD_RECALIBRATE        0x01
#define OMTI_CMD_REQUEST_SENSE      0x03
#define OMTI_CMD_READ_VERIFY        0x05
#define OMTI_CMD_FORMAT_TRACK       0x06
#define OMTI_CMD_FORMAT_BAD_TRACK   0x07
#define OMTI_CMD_READ               0x08
#define OMTI_CMD_WRITE              0x0a
#define OMTI_CMD_SEEK               0x0b
#define OMTI_CMD_READ_SECTOR_BUFFER 0x0e
#define OMTI_CMD_WRITE_SECTOR_BUFFER 0x0f
#define OMTI_CMD_READ_DATA_TO_BUFFER 0x1e
#define OMTI_CMD_WRITE_DATA_FROM_BUFFER 0x1f
#define OMTI_CMD_RAM_DIAGNOSTICS    0xe0
#define OMTI_CMD_CONTROLLER_INT_DIAG 0xe4
#define OMTI_CMD_READ_CONFIGURATION 0xec
#define OMTI_CMD_INVALID            0xff

enum {
    OMTI_STATE_IDLE,
    OMTI_STATE_COMMAND,
    OMTI_STATE_DATA,
    OMTI_STATE_STATUS,
};

/* ------------------------------------------------------------------ */
/* Archive SC-499 cartridge-tape controller + DMA controller 1        */
/*                                                                    */
/* The SR10.4 install boots the standalone utilities off cartridge    */
/* tape ('DI C' at the MD prompt), then EX INVOL / EX CALENDAR /      */
/* EX DOMAIN_OS install onto the Winchester.  The SC-499 is an ISA8   */
/* card at I/O base 0x200 (-> CPU 0x050000 via the Apollo ISA I/O     */
/* translation, same math that placed the OMTI), IRQ5, DRQ1.  Tape    */
/* data streams over ISA DMA (controller 1, channel 1), so unlike the */
/* OMTI this needs a functional Am9517A.  Ported from MAME            */
/* src/devices/bus/isa/sc499.cpp (Ostermeyer/Belmont).  ".ct" images  */
/* are a raw stream of 512-byte blocks; a filemark block is 128 reps  */
/* of DE AF FA ED.                                                     */

#define SC499_ADDR            0x050000 /* ISA 0x200 after Apollo xlat */
#define SC499_BLOCK_SIZE      512

/* ports (byte offset within the 8-byte window) */
#define SC499_PORT_DATA       0x00 /* R data   W command */
#define SC499_PORT_STATUS     0x01 /* R status W control  */
#define SC499_PORT_DMAGO      0x02 /* W start DMA */
#define SC499_PORT_RSTDMA     0x03 /* W reset DMA */

/* tape status word ST0/ST1 (returned by READ STATUS) */
#define SC499_ST0     0x8000
#define SC499_ST0_NOC 0x4000
#define SC499_ST0_WP  0x1000
#define SC499_ST0_EOM 0x0800
#define SC499_ST0_UDE 0x0400
#define SC499_ST0_BNL 0x0200
#define SC499_ST0_FM  0x0100
#define SC499_ST0_MASK 0x7f00
#define SC499_ST1     0x0080
#define SC499_ST1_ILL 0x0040
#define SC499_ST1_NOD 0x0020
#define SC499_ST1_BOM 0x0008
#define SC499_ST1_POR 0x0001
#define SC499_ST1_MASK 0x007f
#define SC499_ST_CLEAR_ALL \
    ((uint16_t)~(SC499_ST0_NOC | SC499_ST0_WP | SC499_ST0_EOM | SC499_ST1_BOM))
#define SC499_ST_READ_ERROR (SC499_ST0_UDE | SC499_ST0_BNL | SC499_ST1_NOD)

/* commands (top 3 bits = type) */
#define SC499_CMD_SELECT         0x00
#define SC499_CMD_POSITION       0x20
#define SC499_CMD_REWIND         0x21
#define SC499_CMD_ERASE          0x22
#define SC499_CMD_RETEN          0x24
#define SC499_CMD_WRITE_DATA     0x40
#define SC499_CMD_WRITE_FILEMARK 0x60
#define SC499_CMD_READ_DATA      0x80
#define SC499_CMD_READ_FILE_MARK 0xa0
#define SC499_CMD_READ_STATUS    0xc0
#define SC499_CMD_NO_COMMAND     0xff
#define SC499_CMD_TYPE_MASK      0xe0

/* status port bits */
#define SC499_STAT_IRQ 0x80
#define SC499_STAT_RDY 0x40
#define SC499_STAT_EXC 0x20
#define SC499_STAT_DON 0x10
#define SC499_STAT_DIR 0x08

/* control port bits */
#define SC499_CTR_RST 0x80
#define SC499_CTR_REQ 0x40
#define SC499_CTR_IEN 0x20
#define SC499_CTR_DNI 0x10

/* internal timer params (MAME SC499_TIMER_n) */
#define SC499_TIMER_1 1
#define SC499_TIMER_3 3
#define SC499_TIMER_4 4
#define SC499_TIMER_5 5
#define SC499_TIMER_6 6
#define SC499_TIMER_7 7
#define SC499_READ_BLOCK_TIME_US 40

typedef struct ApolloPTMTimer {
    struct ApolloState *as;
    int idx;
    uint16_t latch;
    uint8_t cr;
    bool flag;                  /* interrupt flag in status register */
    bool status_read;           /* status read with flag set (clear protocol) */
    int64_t start_ns;
    uint8_t msb_buf;            /* MSB buffer for latch writes */
    uint8_t lsb_buf;            /* LSB buffer for counter reads */
    QEMUTimer *timer;
} ApolloPTMTimer;

typedef struct ApolloState {
    M68kCPU *cpu;

    MemoryRegion rom;
    MemoryRegion csr_sr_mem;
    MemoryRegion csr_cr_mem;
    MemoryRegion ptm_mem;
    MemoryRegion rtc_mem;
    MemoryRegion dma_mem[4];
    MemoryRegion pic_mem[2];
    MemoryRegion nodeid_mem;
    MemoryRegion atbus_io;
    MemoryRegion atbus_mem;
    MemoryRegion buserr_mem;
    MemoryRegion sio_alias;
    MemoryRegion cpuctrl_alias;
    MemoryRegion xlat_mem;

    uint16_t csr_sr;
    uint16_t csr_cr;

    PICCommonState *pic[2];     /* 0 = master, 1 = slave */
    MC146818RtcState *rtc;

    uint8_t dma_regs[4][0x20];

    ApolloPTMTimer ptm[3];

    uint32_t node_id;

    /* OMTI-8621 Winchester controller */
    MemoryRegion omti_mem;
    BlockBackend *omti_blk;
    qemu_irq omti_irq;          /* Apollo IRQ14 (slave PIC IR6) */
    QEMUTimer *omti_irq_timer;  /* command-completion interrupt latency */
    uint16_t omti_cylinders;
    uint16_t omti_heads;
    uint16_t omti_sectors;
    uint8_t omti_state;
    uint8_t omti_status;        /* status port */
    uint8_t omti_config;        /* config port (~jumper) */
    uint8_t omti_mask;          /* mask port */
    uint8_t omti_cmd_status;    /* command completion status byte */
    uint8_t omti_cmd_buf[10];
    uint8_t omti_cmd_index;
    uint8_t omti_cmd_length;
    uint8_t omti_sense[4];
    uint8_t omti_config_data[10];
    uint8_t *omti_data;         /* current PIO transfer buffer */
    uint32_t omti_data_index;
    uint32_t omti_data_length;
    uint8_t omti_sector_buf[OMTI_SECTOR_SIZE * OMTI_MAX_BLOCK_COUNT];

    /* Am9517A DMA controller 1 (functional; ctape uses channel 1) */
    MemoryRegion dma1_mem;
    struct {
        uint16_t base_addr, cur_addr, base_count, cur_count;
        uint8_t mode;
    } dma1_ch[4];
    uint8_t dma1_mask;          /* bit per channel, 1 = masked */
    uint8_t dma1_cmd;
    uint8_t dma1_status;        /* TC bits 0-3, request bits 4-7 */
    bool dma1_ff;               /* byte pointer flip-flop */

    /* Archive SC-499 cartridge-tape controller */
    MemoryRegion sc499_mem;
    BlockBackend *ct_blk;
    qemu_irq sc499_irq;         /* Apollo IRQ5 (master PIC IR5) */
    QEMUTimer *ct_timer;        /* one-shot, param in ct_timer_param */
    QEMUTimer *ct_timer1;       /* periodic block reader */
    int ct_timer_param;
    int ct_timer1_param;
    int64_t ct_timer1_period;
    uint8_t ct_data, ct_command, ct_status, ct_control, ct_current_command;
    uint8_t ct_first_block_hack, ct_nasty_readahead, ct_read_block_pending;
    uint8_t ct_has_cartridge, ct_is_writable, ct_reset_pending;
    uint16_t ct_data_index;
    uint16_t ct_tape_status, ct_data_err, ct_underrun;
    uint32_t ct_tape_pos, ct_block_count, ct_block_index;
    int64_t ct_image_length;
    uint8_t ct_block_buf[512];
    int ct_irq_state, ct_drq_state;

    /* -kernel direct boot */
    bool linux_boot;
    uint32_t reset_pc;
    uint32_t reset_sp;
} ApolloState;

/* ------------------------------------------------------------------ */
/* CPU status/control registers                                       */

static uint64_t apollo_csr_sr_read(void *opaque, hwaddr addr, unsigned size)
{
    ApolloState *s = opaque;

    return s->csr_sr;
}

static void apollo_csr_sr_write(void *opaque, hwaddr addr, uint64_t val,
                                unsigned size)
{
    ApolloState *s = opaque;

    /* any write clears the latched fault bits */
    s->csr_sr &= CSR_SR_BIT15 | CSR_SR_FP_TRAP | CSR_SR_SERVICE |
                 CSR_SR_INTERRUPT_PENDING;
}

static const MemoryRegionOps apollo_csr_sr_ops = {
    .read = apollo_csr_sr_read,
    .write = apollo_csr_sr_write,
    .impl = { .min_access_size = 2, .max_access_size = 2 },
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .endianness = DEVICE_BIG_ENDIAN,
};

static uint64_t apollo_csr_cr_read(void *opaque, hwaddr addr, unsigned size)
{
    ApolloState *s = opaque;

    return s->csr_cr;
}

static void apollo_csr_cr_write(void *opaque, hwaddr addr, uint64_t val,
                                unsigned size)
{
    ApolloState *s = opaque;

    s->csr_cr = val;
    /*
     * CSR_CR_FPU_TRAP_ENABLE=0 enables the FPU opcodes on real
     * hardware.  The QEMU m68020 model always has FPU decode
     * enabled, so nothing to do here.  RESET_DEVICES is ignored
     * (MAME does not implement it either).
     */
}

static const MemoryRegionOps apollo_csr_cr_ops = {
    .read = apollo_csr_cr_read,
    .write = apollo_csr_cr_write,
    .impl = { .min_access_size = 2, .max_access_size = 2 },
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .endianness = DEVICE_BIG_ENDIAN,
};

/* ------------------------------------------------------------------ */
/* Bus error background: everything unmapped raises a real 68k bus    */
/* error and latches CSR_SR_CPU_TIMEOUT, as the PROM self-test        */
/* expects for RAM sizing and device probes.                          */

static MemTxResult apollo_buserr_read(void *opaque, hwaddr addr,
                                      uint64_t *val, unsigned size,
                                      MemTxAttrs attrs)
{
    ApolloState *s = opaque;

    s->csr_sr |= CSR_SR_CPU_TIMEOUT;
    qemu_log_mask(LOG_GUEST_ERROR, "apollo: bus error read at 0x%08"
                  HWADDR_PRIx "\n", addr);
    *val = ~0ULL;
    return MEMTX_DECODE_ERROR;
}

static MemTxResult apollo_buserr_write(void *opaque, hwaddr addr,
                                       uint64_t val, unsigned size,
                                       MemTxAttrs attrs)
{
    ApolloState *s = opaque;

    s->csr_sr |= CSR_SR_CPU_TIMEOUT;
    qemu_log_mask(LOG_GUEST_ERROR, "apollo: bus error write at 0x%08"
                  HWADDR_PRIx "\n", addr);
    return MEMTX_DECODE_ERROR;
}

static const MemoryRegionOps apollo_buserr_ops = {
    .read_with_attrs = apollo_buserr_read,
    .write_with_attrs = apollo_buserr_write,
    .impl = { .min_access_size = 1, .max_access_size = 8 },
    .valid = { .min_access_size = 1, .max_access_size = 8 },
    .endianness = DEVICE_BIG_ENDIAN,
};

/* AT bus: unmapped ISA addresses just float high, no bus error */
static uint64_t apollo_atbus_read(void *opaque, hwaddr addr, unsigned size)
{
    return ~0ULL;
}

static void apollo_atbus_write(void *opaque, hwaddr addr, uint64_t val,
                               unsigned size)
{
}

static const MemoryRegionOps apollo_atbus_ops = {
    .read = apollo_atbus_read,
    .write = apollo_atbus_write,
    .impl = { .min_access_size = 1, .max_access_size = 4 },
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .endianness = DEVICE_BIG_ENDIAN,
};

/* boot PROM: writes are logged and ignored (MAME apollo_rom_w) */
static void apollo_rom_write(void *opaque, hwaddr addr, uint64_t val,
                             unsigned size)
{
    qemu_log_mask(LOG_GUEST_ERROR, "apollo: write to boot ROM at 0x%"
                  HWADDR_PRIx " = 0x%" PRIx64 " (ignored)\n", addr, val);
}

static uint64_t apollo_rom_read(void *opaque, hwaddr addr, unsigned size)
{
    /* not reached: reads are served from the ROM's RAM backing */
    return 0;
}

static const MemoryRegionOps apollo_rom_ops = {
    .read = apollo_rom_read,
    .write = apollo_rom_write,
    .endianness = DEVICE_BIG_ENDIAN,
};

/* ------------------------------------------------------------------ */
/* Dual i8259 PIC: registers live behind an unmapped private ISA I/O  */
/* space; these MMIO wrappers forward the two 8-bit registers of each */
/* chip.  Interrupt delivery: master INT -> IPL 6 vectored.           */

static uint64_t apollo_pic_reg_read(void *opaque, hwaddr addr, unsigned size)
{
    PICCommonState *pic = opaque;
    uint64_t val = 0xff;

    memory_region_dispatch_read(&pic->base_io, addr & 1, &val, MO_8,
                                MEMTXATTRS_UNSPECIFIED);
    return val;
}

static void apollo_pic_reg_write(void *opaque, hwaddr addr, uint64_t val,
                                 unsigned size)
{
    PICCommonState *pic = opaque;
    bool icw1 = ((addr & 1) == 0) && (val & 0x10);
    uint8_t saved_last_irr = pic->last_irr;

    memory_region_dispatch_write(&pic->base_io, addr & 1, val, MO_8,
                                 MEMTXATTRS_UNSPECIFIED);

    if (icw1) {
        /*
         * ICW1 makes QEMU's i8259 zero its edge-sense memory (last_irr).  On a
         * real 8259 a request line that is *already* high across the re-init
         * still needs a fresh low->high transition to be recognised; zeroing
         * last_irr instead fabricates a phantom edge the next time the
         * (still-high) line is sampled.  The Apollo MC6840 heartbeat holds its
         * IRQ line high (timer flag set) while INVOL reprograms the master PIC
         * to edge mode, so that phantom edge is delivered as a spurious timer
         * interrupt whose PTM status register reads empty -- INVOL treats it as
         * a fatal unexpected interrupt (Crash_Status 000D0004).  Preserve the
         * edge-sense memory so a line held high across ICW1 is not mistaken for
         * a new request; it will interrupt only on the next genuine 0->1 edge.
         */
        pic->last_irr = saved_last_irr;
    }
}

static const MemoryRegionOps apollo_pic_reg_ops = {
    .read = apollo_pic_reg_read,
    .write = apollo_pic_reg_write,
    .impl = { .min_access_size = 1, .max_access_size = 1 },
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .endianness = DEVICE_BIG_ENDIAN,
};

/*
 * Priority resolution copied from hw/intc/i8259.c (the originals are
 * static, and QEMU's pic_read_irq() hard-codes the PC scheme of the
 * slave cascaded on IR2 -- on the Apollo the slave hangs off IR3).
 */
static int apollo_pic_get_priority(PICCommonState *s, int mask)
{
    int priority;

    if (mask == 0) {
        return 8;
    }
    priority = 0;
    while ((mask & (1 << ((priority + s->priority_add) & 7))) == 0) {
        priority++;
    }
    return priority;
}

static int apollo_pic_get_irq(PICCommonState *s)
{
    int mask, cur_priority, priority;

    mask = s->irr & ~s->imr;
    priority = apollo_pic_get_priority(s, mask);
    if (priority == 8) {
        return -1;
    }
    mask = s->isr;
    if (s->special_mask) {
        mask &= ~s->imr;
    }
    if (s->special_fully_nested_mode && s->master) {
        mask &= ~(1 << 3);      /* Apollo: cascade on IR3 */
    }
    cur_priority = apollo_pic_get_priority(s, mask);
    if (priority < cur_priority) {
        return (priority + s->priority_add) & 7;
    }
    return -1;
}

static void apollo_pic_intack(PICCommonState *s, int irq)
{
    if (s->auto_eoi) {
        if (s->rotate_on_auto_eoi) {
            s->priority_add = (irq + 1) & 7;
        }
    } else {
        s->isr |= (1 << irq);
    }
    if (!s->ltim && !(s->elcr & (1 << irq))) {
        s->irr &= ~(1 << irq);
    }
    /* re-evaluate the INT output */
    qemu_set_irq(s->int_out[0], apollo_pic_get_irq(s) >= 0);
}

/*
 * Compute the vector the PIC pair would supply for an IACK, without
 * modifying any state.  QEMU's m68k core wants the vector when the
 * interrupt is signalled; the destructive acknowledge runs from the
 * CPU's iack-out hook once the exception is actually taken.
 */
static uint8_t apollo_pic_peek_vector(ApolloState *s)
{
    int irq = apollo_pic_get_irq(s->pic[0]);
    int irq2;

    if (irq < 0) {
        return s->pic[0]->irq_base + 7; /* spurious */
    }
    if (irq == 3) {             /* slave cascade */
        irq2 = apollo_pic_get_irq(s->pic[1]);
        if (irq2 < 0) {
            irq2 = 7;           /* spurious on slave */
        }
        return s->pic[1]->irq_base + irq2;
    }
    return s->pic[0]->irq_base + irq;
}

/* master PIC INT output */
static void apollo_pic_int(void *opaque, int n, int level)
{
    ApolloState *s = opaque;

    if (level) {
        s->csr_sr |= CSR_SR_INTERRUPT_PENDING;
        m68k_set_irq_level(s->cpu, 6, apollo_pic_peek_vector(s));
    } else {
        s->csr_sr &= ~CSR_SR_INTERRUPT_PENDING;
        m68k_set_irq_level(s->cpu, 0, 0);
    }
}

/* CPU level-6 interrupt acknowledge cycle */
static void apollo_pic_iack(void *opaque, int n, int level)
{
    ApolloState *s = opaque;
    int irq = apollo_pic_get_irq(s->pic[0]);
    int irq2;

    if (irq < 0) {
        return;                 /* spurious, nothing to acknowledge */
    }
    if (irq == 3) {
        irq2 = apollo_pic_get_irq(s->pic[1]);
        if (irq2 >= 0) {
            apollo_pic_intack(s->pic[1], irq2);
        }
    }
    apollo_pic_intack(s->pic[0], irq);
}

/* ------------------------------------------------------------------ */
/* MC146818 RTC: the Apollo maps its registers directly (byte offset  */
/* = register number); QEMU's model is index/data, so forward each    */
/* access through the RTC's own I/O region.                           */

/*
 * MC146818 day-of-week (register 6) convention differs: QEMU's model uses
 * 1=Sunday..7=Saturday (recomputing the field on every update tick), while
 * Domain/OS uses 0=Sunday..6=Saturday.  CALENDAR writes the Domain value and
 * INVOL re-validates the day-of-week against the date, so without a
 * translation an RTC update overwrites the field with QEMU's value and
 * INVOL's calendar check fails (crash status 000D0004).  Bridge the two.
 */
#define APOLLO_RTC_REG_WEEKDAY 6

static uint8_t apollo_rtc_raw(ApolloState *s, unsigned reg)
{
    uint64_t v = 0;

    memory_region_dispatch_write(&s->rtc->io, 0, reg, MO_8,
                                 MEMTXATTRS_UNSPECIFIED);
    memory_region_dispatch_read(&s->rtc->io, 1, &v, MO_8,
                                MEMTXATTRS_UNSPECIFIED);
    return v;
}

/* Sakamoto's algorithm, 0 = Sunday */
static int apollo_day_of_week(int y, int m, int d)
{
    static const int t[] = { 0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4 };

    if (m < 3) {
        y -= 1;
    }
    return (y + y / 4 - y / 100 + y / 400 + t[m - 1] + d) % 7;
}

static uint64_t apollo_rtc_read(void *opaque, hwaddr addr, unsigned size)
{
    ApolloState *s = opaque;
    unsigned reg = addr & 0x7f;
    uint64_t val = 0xff;

    if (reg == APOLLO_RTC_REG_WEEKDAY) {
        /*
         * QEMU's mc146818 derives the day-of-week from an internal epoch
         * whose century/base-year handling does not match what Domain/OS
         * expects, so it can return the wrong weekday for the date the
         * user set (e.g. Sunday for 1992-02-14, a Friday) - and invol
         * cross-checks the weekday against the date, panicking on a
         * mismatch.  Compute it directly from the date registers instead,
         * in Domain's 0=Sunday convention.  (DM=1 binary mode; calendar
         * sets that.)
         */
        int yy = apollo_rtc_raw(s, 9);      /* year within century */
        int mm = apollo_rtc_raw(s, 8);
        int dd = apollo_rtc_raw(s, 7);
        int year = yy + (yy >= 70 ? 1900 : 2000);

        if (mm >= 1 && mm <= 12 && dd >= 1 && dd <= 31) {
            return apollo_day_of_week(year, mm, dd);
        }
        return 0;
    }

    memory_region_dispatch_write(&s->rtc->io, 0, reg, MO_8,
                                 MEMTXATTRS_UNSPECIFIED);
    memory_region_dispatch_read(&s->rtc->io, 1, &val, MO_8,
                                MEMTXATTRS_UNSPECIFIED);
#ifdef APOLLO_RTC_TRACE
    {
        uint32_t pc = current_cpu ? M68K_CPU(current_cpu)->env.pc : 0;
        fprintf(stderr, "RTCRD reg=%02x val=%02x pc=%08x\n",
                reg, (unsigned)(val & 0xff), pc);
    }
#endif
    return val;
}

static void apollo_rtc_write(void *opaque, hwaddr addr, uint64_t val,
                             unsigned size)
{
    ApolloState *s = opaque;
    unsigned reg = addr & 0x7f;

    memory_region_dispatch_write(&s->rtc->io, 0, reg, MO_8,
                                 MEMTXATTRS_UNSPECIFIED);
    memory_region_dispatch_write(&s->rtc->io, 1, val, MO_8,
                                 MEMTXATTRS_UNSPECIFIED);
}

static const MemoryRegionOps apollo_rtc_ops = {
    .read = apollo_rtc_read,
    .write = apollo_rtc_write,
    .impl = { .min_access_size = 1, .max_access_size = 1 },
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .endianness = DEVICE_BIG_ENDIAN,
};

/* ------------------------------------------------------------------ */
/* MC6840 PTM.  Registers on odd byte addresses (reg = offset >> 1).  */
/* Input clocks (MAME/apollo docs): T1 250 kHz, T2 125 kHz, T3        */
/* 62.5 kHz (optionally /8).  Lazily-computed down counters.          */

#define PTM_CR_INTERNAL_RESET 0x01 /* CR1 only */
#define PTM_CR_CR1_SELECT     0x01 /* CR2 only */
#define PTM_CR_T3_PRESCALE    0x01 /* CR3 only */
#define PTM_CR_DUAL8          0x04
#define PTM_CR_IRQ_ENABLE     0x40

static uint64_t apollo_ptm_rate(ApolloState *s, int idx)
{
    static const uint64_t rates[3] = { 250000, 125000, 62500 };
    uint64_t rate = rates[idx];

    if (idx == 2 && (s->ptm[2].cr & PTM_CR_T3_PRESCALE)) {
        rate /= 8;
    }
    return rate;
}

static void apollo_ptm_update_irq(ApolloState *s)
{
    bool level = false;
    int i;

    for (i = 0; i < 3; i++) {
        if (s->ptm[i].flag && (s->ptm[i].cr & PTM_CR_IRQ_ENABLE)) {
            level = true;
        }
    }
    /* PTM output feeds PIC IRQ 0 */
#ifdef APOLLO_RTC_TRACE
    {
        uint32_t pc = current_cpu ? M68K_CPU(current_cpu)->env.pc : 0;
        fprintf(stderr, "PTM irq level=%d f=[%d%d%d] en=[%d%d%d] pc=%08x\n",
                level, s->ptm[0].flag, s->ptm[1].flag, s->ptm[2].flag,
                !!(s->ptm[0].cr & PTM_CR_IRQ_ENABLE),
                !!(s->ptm[1].cr & PTM_CR_IRQ_ENABLE),
                !!(s->ptm[2].cr & PTM_CR_IRQ_ENABLE), pc);
    }
#endif
    qemu_set_irq(qdev_get_gpio_in(DEVICE(s->pic[0]), 0), level);
}

static bool apollo_ptm_running(ApolloState *s)
{
    return !(s->ptm[0].cr & PTM_CR_INTERNAL_RESET);
}

static uint16_t apollo_ptm_count(ApolloState *s, int idx)
{
    ApolloPTMTimer *t = &s->ptm[idx];
    uint32_t period = (uint32_t)t->latch + 1;
    uint64_t ticks;

    if (!apollo_ptm_running(s)) {
        return t->latch;
    }
    ticks = (uint64_t)(qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - t->start_ns)
        * apollo_ptm_rate(s, idx) / NANOSECONDS_PER_SECOND;
    return t->latch - (ticks % period);
}

static void apollo_ptm_arm(ApolloState *s, int idx)
{
    ApolloPTMTimer *t = &s->ptm[idx];
    uint64_t period_ns = ((uint64_t)t->latch + 1) * NANOSECONDS_PER_SECOND
        / apollo_ptm_rate(s, idx);

    period_ns = MAX(period_ns, 1000);
    timer_mod(t->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + period_ns);
}

static void apollo_ptm_reload(ApolloState *s, int idx)
{
    ApolloPTMTimer *t = &s->ptm[idx];

    t->start_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    if (apollo_ptm_running(s)) {
        apollo_ptm_arm(s, idx);
    } else {
        timer_del(t->timer);
    }
}

static void apollo_ptm_expired(void *opaque)
{
    ApolloPTMTimer *t = opaque;
    ApolloState *s = t->as;

    t->flag = true;
#ifdef APOLLO_RTC_TRACE
    fprintf(stderr, "PTM EXP idx=%d\n", t->idx);
#endif
    apollo_ptm_update_irq(s);
    if (apollo_ptm_running(s)) {
        /* continuous operation: reload and keep going */
        apollo_ptm_arm(s, t->idx);
    }
}

static void apollo_ptm_set_reset(ApolloState *s, bool reset)
{
    int i;

    for (i = 0; i < 3; i++) {
        if (reset) {
            s->ptm[i].flag = false;
            s->ptm[i].status_read = false;
            timer_del(s->ptm[i].timer);
        } else {
            apollo_ptm_reload(s, i);
        }
    }
    apollo_ptm_update_irq(s);
}

static uint64_t apollo_ptm_read(void *opaque, hwaddr addr, unsigned size)
{
    ApolloState *s = opaque;
    unsigned reg = (addr >> 1) & 7;
    ApolloPTMTimer *t;
    uint64_t data = 0;
    uint16_t count;
    int i;

    switch (reg) {
    case 0:                     /* no read register */
        break;
    case 1:                     /* status register */
        for (i = 0; i < 3; i++) {
            if (s->ptm[i].flag) {
                data |= 1 << i;
                if (s->ptm[i].cr & PTM_CR_IRQ_ENABLE) {
                    data |= 0x80;
                }
                s->ptm[i].status_read = true;
            }
        }
#ifdef APOLLO_RTC_TRACE
        {
            uint32_t pc = current_cpu ? M68K_CPU(current_cpu)->env.pc : 0;
            fprintf(stderr, "PTM statusrd=%02x pc=%08x\n",
                    (unsigned)data, pc);
        }
#endif
        break;
    case 2:                     /* T1 MSB */
    case 4:                     /* T2 MSB */
    case 6:                     /* T3 MSB */
        t = &s->ptm[(reg - 2) >> 1];
        count = apollo_ptm_count(s, t->idx);
        t->lsb_buf = count & 0xff;
        data = count >> 8;
        /* interrupt-clearing protocol: status read then counter read */
        if (t->status_read) {
            t->flag = false;
            t->status_read = false;
#ifdef APOLLO_RTC_TRACE
            {
                uint32_t pc = current_cpu ? M68K_CPU(current_cpu)->env.pc : 0;
                fprintf(stderr, "PTM CLRrd idx=%d pc=%08x\n", t->idx, pc);
            }
#endif
            apollo_ptm_update_irq(s);
        }
        break;
    case 3:                     /* T1 LSB */
    case 5:                     /* T2 LSB */
    case 7:                     /* T3 LSB */
        t = &s->ptm[(reg - 3) >> 1];
        data = t->lsb_buf;
        break;
    }
    return data;
}

static void apollo_ptm_write(void *opaque, hwaddr addr, uint64_t val,
                             unsigned size)
{
    ApolloState *s = opaque;
    unsigned reg = (addr >> 1) & 7;
    ApolloPTMTimer *t;
    uint8_t data = val;
    bool was_reset;

    switch (reg) {
    case 0:                     /* CR1 or CR3, selected by CR2 bit 0 */
        if (s->ptm[1].cr & PTM_CR_CR1_SELECT) {
            was_reset = !apollo_ptm_running(s);
            s->ptm[0].cr = data;
            if (was_reset != !apollo_ptm_running(s)) {
                apollo_ptm_set_reset(s, !apollo_ptm_running(s));
            }
        } else {
            s->ptm[2].cr = data;
        }
        apollo_ptm_update_irq(s);
        break;
    case 1:                     /* CR2 */
        s->ptm[1].cr = data;
        apollo_ptm_update_irq(s);
        break;
    case 2:                     /* T1 MSB buffer */
    case 4:
    case 6:
        s->ptm[(reg - 2) >> 1].msb_buf = data;
        break;
    case 3:                     /* T1 LSB: transfers latch */
    case 5:
    case 7:
        t = &s->ptm[(reg - 3) >> 1];
        t->latch = (t->msb_buf << 8) | data;
        /* writing the latch clears the interrupt flag and reinitialises */
        t->flag = false;
        t->status_read = false;
        apollo_ptm_reload(s, t->idx);
        apollo_ptm_update_irq(s);
        break;
    }
}

static const MemoryRegionOps apollo_ptm_ops = {
    .read = apollo_ptm_read,
    .write = apollo_ptm_write,
    .impl = { .min_access_size = 1, .max_access_size = 1 },
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .endianness = DEVICE_BIG_ENDIAN,
};

/* ------------------------------------------------------------------ */
/* DMA controllers, page registers, parity latch: RAM-backed stubs    */
/* (enough for the self-test's register read/write probes).           */

static uint64_t apollo_dma_read(void *opaque, hwaddr addr, unsigned size)
{
    uint8_t *regs = opaque;

    return regs[addr & 0x1f];
}

static void apollo_dma_write(void *opaque, hwaddr addr, uint64_t val,
                             unsigned size)
{
    uint8_t *regs = opaque;

    regs[addr & 0x1f] = val;
}

static const MemoryRegionOps apollo_dma_ops = {
    .read = apollo_dma_read,
    .write = apollo_dma_write,
    .impl = { .min_access_size = 1, .max_access_size = 1 },
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .endianness = DEVICE_BIG_ENDIAN,
};

/* ------------------------------------------------------------------ */
/* Node ID PROM: 16-bit registers, id bytes in the high byte lane     */

static uint64_t apollo_nodeid_read(void *opaque, hwaddr addr, unsigned size)
{
    ApolloState *s = opaque;
    unsigned reg = (addr >> 1) & 0xf;
    uint16_t data;

    switch (reg) {
    case 1:
        data = (s->node_id >> 16) & 0xff;
        break;
    case 2:
        data = (s->node_id >> 8) & 0xff;
        break;
    case 3:
        data = s->node_id & 0xff;
        break;
    case 15:                    /* checksum */
        data = ((s->node_id >> 16) + (s->node_id >> 8) + s->node_id) & 0xff;
        break;
    default:
        data = 0;
        break;
    }
    return data << 8;
}

static void apollo_nodeid_write(void *opaque, hwaddr addr, uint64_t val,
                                unsigned size)
{
    qemu_log_mask(LOG_GUEST_ERROR, "apollo: write to node id PROM at 0x%"
                  HWADDR_PRIx "\n", addr);
}

static const MemoryRegionOps apollo_nodeid_ops = {
    .read = apollo_nodeid_read,
    .write = apollo_nodeid_write,
    .impl = { .min_access_size = 2, .max_access_size = 2 },
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .endianness = DEVICE_BIG_ENDIAN,
};

/* ------------------------------------------------------------------ */
/* OMTI-8621 Winchester disk controller implementation                */

static void omti_irq_fire(void *opaque)
{
    ApolloState *s = opaque;

    if (getenv("APOLLO_DEVTRACE")) {
        fprintf(stderr, "[DEV] OMTI irq=1 (completion)\n");
    }
    if (s->omti_irq) {
        qemu_set_irq(s->omti_irq, 1);
    }
}

static void omti_set_interrupt(ApolloState *s, int level)
{
    if (level) {
        /*
         * Real controllers signal command completion after a latency, not
         * in the same bus cycle as the last host access.  Deferring the
         * assert (rather than raising it re-entrantly from inside the CDB
         * or data-port write) matches that and avoids delivering IRQ14 to
         * the driver mid-transfer.
         */
        {
            uint64_t lat = 20000;
            const char *e = getenv("APOLLO_OMTI_LAT_NS");
            if (e) {
                qemu_strtou64(e, NULL, 10, &lat);
            }
            timer_mod(s->omti_irq_timer,
                      qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + lat);
        }
    } else {
        timer_del(s->omti_irq_timer);
        if (s->omti_irq) {
            qemu_set_irq(s->omti_irq, 0);
        }
    }
}

static void omti_setup_geometry(ApolloState *s)
{
    int64_t len = s->omti_blk ? blk_getlength(s->omti_blk) : 0;
    uint32_t total = len > 0 ? len / OMTI_SECTOR_SIZE : 0;

    /* MAME device_reset(): >=300000 sectors -> Maxtor 348MB, else 155MB */
    if (total >= 300000) {
        s->omti_cylinders = 1223;
        s->omti_heads = 15;
        s->omti_sectors = 18;
        s->omti_config = (uint8_t)~(OMTI_CONFIG_W22 | OMTI_CONFIG_W23);
    } else {
        s->omti_cylinders = 1023;
        s->omti_heads = 8;
        s->omti_sectors = 18;
        s->omti_config = (uint8_t)~OMTI_CONFIG_W20;
    }
}

static void omti_sw_reset(ApolloState *s)
{
    omti_setup_geometry(s);
    s->omti_state = OMTI_STATE_IDLE;
    s->omti_status = OMTI_STATUS_NU6 | OMTI_STATUS_NU7;
    s->omti_mask = 0;
    s->omti_cmd_index = 0;
    s->omti_cmd_length = 0;
    s->omti_cmd_status = 0;
    s->omti_data_index = 0;
    s->omti_data_length = 0;
    memset(s->omti_sense, 0, sizeof(s->omti_sense));

    /* controller identity in the sector buffer (READ SECTOR BUFFER) */
    memset(s->omti_sector_buf, 0, OMTI_SECTOR_SIZE);
    memcpy(s->omti_sector_buf, "8621VB.4060487xx", 0x10);
    s->omti_sector_buf[0x14] = 0xc0; /* 32K buffer size */

    omti_set_interrupt(s, 0);
}

static uint8_t omti_get_lun(const uint8_t *cdb)
{
    return (cdb[1] & 0x20) >> 5;
}

static uint32_t omti_disk_track(ApolloState *s, const uint8_t *cdb)
{
    uint16_t head = cdb[1] & 0x1f;
    uint32_t cyl = cdb[3] + ((cdb[2] & 0xc0) << 2) + ((cdb[1] & 0x80) << 3);

    return cyl * s->omti_heads + head;
}

static uint32_t omti_disk_address(ApolloState *s, const uint8_t *cdb)
{
    uint16_t sector = cdb[2] & 0x3f;

    return omti_disk_track(s, cdb) * s->omti_sectors + sector;
}

static void omti_set_sense(ApolloState *s, uint8_t code, const uint8_t *cdb)
{
    s->omti_sense[0] = code;
    s->omti_sense[1] = cdb[1];
    s->omti_sense[2] = cdb[2];
    s->omti_sense[3] = cdb[3];
}

/* returns true if the CDB disk address is valid */
static bool omti_check_disk_address(ApolloState *s, const uint8_t *cdb)
{
    uint8_t lun = omti_get_lun(cdb);
    uint16_t head = cdb[1] & 0x1f;
    uint16_t sector = cdb[2] & 0x3f;
    uint32_t cyl = cdb[3] + ((cdb[2] & 0xc0) << 2) + ((cdb[1] & 0x80) << 3);
    uint8_t block_count = cdb[4];
    uint8_t sense = OMTI_SENSE_NO_ERROR;

    if (block_count > OMTI_MAX_BLOCK_COUNT) {
        sense = OMTI_SENSE_ILLEGAL_ADDRESS | OMTI_SENSE_ADDRESS_VALID;
    } else if (lun > OMTI_MAX_LUN || !s->omti_blk) {
        sense = OMTI_SENSE_DRIVE_NOT_READY;
    } else if (sector >= OMTI_MAX_BLOCK_COUNT || head >= s->omti_heads ||
               cyl >= s->omti_cylinders) {
        sense = OMTI_SENSE_ILLEGAL_ADDRESS | OMTI_SENSE_ADDRESS_VALID;
    }

    if (sense != OMTI_SENSE_NO_ERROR) {
        omti_set_sense(s, sense, cdb);
        s->omti_cmd_status |= OMTI_CMDSTATUS_ERROR;
        return false;
    }
    omti_set_sense(s, OMTI_SENSE_NO_ERROR, cdb);
    return true;
}

static void omti_set_config_data(ApolloState *s)
{
    s->omti_config_data[0] = (s->omti_cylinders - 1) >> 8;
    s->omti_config_data[1] = (s->omti_cylinders - 1) & 0xff;
    s->omti_config_data[2] = s->omti_heads - 1;
    s->omti_config_data[3] = s->omti_sectors - 1;
    s->omti_config_data[4] = 0x02;
    s->omti_config_data[5] = 0x44;
    s->omti_config_data[6] = 0x00;
    s->omti_config_data[7] = 0x00;
    s->omti_config_data[8] = 0x00;
    s->omti_config_data[9] = 0x00;
}

/* arm a controller->host PIO data transfer */
static void omti_set_data_transfer(ApolloState *s, uint8_t *data, uint32_t len)
{
    s->omti_state = OMTI_STATE_DATA;
    s->omti_status |= OMTI_STATUS_REQ | OMTI_STATUS_IO | OMTI_STATUS_BUSY;
    s->omti_status &= ~OMTI_STATUS_CD;
    s->omti_data = data;
    s->omti_data_length = len;
    s->omti_data_index = 0;
}

static void omti_read_sectors(ApolloState *s, uint32_t diskaddr, uint8_t count)
{
    uint8_t *buf = s->omti_sector_buf;

    while (count-- > 0) {
        if (!s->omti_blk ||
            blk_pread(s->omti_blk, (int64_t)diskaddr * OMTI_SECTOR_SIZE,
                      OMTI_SECTOR_SIZE, buf, 0) < 0) {
            memset(buf, 0, OMTI_SECTOR_SIZE);
        }
        diskaddr++;
        buf += OMTI_SECTOR_SIZE;
    }
}

static void omti_write_sectors(ApolloState *s, uint32_t diskaddr, uint8_t count)
{
    uint8_t *buf = s->omti_sector_buf;

    while (count-- > 0) {
        if (s->omti_blk) {
            blk_pwrite(s->omti_blk, (int64_t)diskaddr * OMTI_SECTOR_SIZE,
                       OMTI_SECTOR_SIZE, buf, 0);
        }
        diskaddr++;
        buf += OMTI_SECTOR_SIZE;
    }
}

static void omti_do_command(ApolloState *s)
{
    const uint8_t *cdb = s->omti_cmd_buf;
    uint8_t lun = omti_get_lun(cdb);

    if (getenv("APOLLO_DEVTRACE")) {
        uint32_t pc = current_cpu ? M68K_CPU(current_cpu)->env.pc : 0;
        fprintf(stderr, "[DEV] OMTI cmd=%02x lba=%02x%02x%02x n=%02x pc=%08x\n",
                cdb[0], cdb[1] & 0x1f, cdb[2], cdb[3], cdb[4], pc);
    }

    /* default: read status, successful completion */
    s->omti_state = OMTI_STATE_STATUS;
    s->omti_status |= OMTI_STATUS_IO | OMTI_STATUS_CD;
    s->omti_cmd_status = lun ? OMTI_CMDSTATUS_LUN : 0;

    if (s->omti_mask & OMTI_MASK_INTE) {
        omti_set_interrupt(s, 0);
    }

    if (!s->omti_blk) {
        s->omti_cmd_status |= OMTI_CMDSTATUS_ERROR; /* no such drive */
    }

    switch (cdb[0]) {
    case OMTI_CMD_TEST_DRIVE_READY:
        if (!s->omti_blk) {
            omti_set_sense(s, OMTI_SENSE_DRIVE_NOT_READY, cdb);
        }
        break;
    case OMTI_CMD_RECALIBRATE:
    case OMTI_CMD_SEEK:
    case OMTI_CMD_READ_VERIFY:
        omti_check_disk_address(s, cdb);
        break;
    case OMTI_CMD_REQUEST_SENSE:
        omti_set_data_transfer(s, s->omti_sense, sizeof(s->omti_sense));
        break;
    case OMTI_CMD_READ:
        /* disk -> controller buffer, then streamed out to the host */
        if (omti_check_disk_address(s, cdb)) {
            omti_read_sectors(s, omti_disk_address(s, cdb), cdb[4]);
            omti_set_data_transfer(s, s->omti_sector_buf,
                                   OMTI_SECTOR_SIZE * cdb[4]);
        }
        break;
    case OMTI_CMD_READ_DATA_TO_BUFFER:
        /*
         * disk -> controller sector buffer only, with no host data phase (the
         * host retrieves it separately with READ_SECTOR_BUFFER).  Setting up a
         * host data-in transfer here would leave the controller waiting for the
         * host to drain a buffer it never reads, hanging the completion poll
         * (INVOL panic Crash_Status 00080016).
         */
        if (omti_check_disk_address(s, cdb)) {
            omti_read_sectors(s, omti_disk_address(s, cdb), cdb[4]);
        }
        break;
    case OMTI_CMD_READ_SECTOR_BUFFER:
        omti_set_data_transfer(s, s->omti_sector_buf,
                               OMTI_SECTOR_SIZE * cdb[4]);
        break;
    case OMTI_CMD_WRITE:
    case OMTI_CMD_WRITE_DATA_FROM_BUFFER:
        if (omti_check_disk_address(s, cdb)) {
            omti_write_sectors(s, omti_disk_address(s, cdb), cdb[4]);
        }
        break;
    case OMTI_CMD_WRITE_SECTOR_BUFFER:
        break;
    case OMTI_CMD_FORMAT_TRACK:
        /* fill the whole track with the 0x6c format pattern */
        if (omti_check_disk_address(s, cdb)) {
            uint32_t da = omti_disk_track(s, cdb) * s->omti_sectors;
            memset(s->omti_sector_buf, 0x6c,
                   OMTI_SECTOR_SIZE * s->omti_sectors);
            omti_write_sectors(s, da, s->omti_sectors);
        }
        break;
    case OMTI_CMD_FORMAT_BAD_TRACK:
        /* no emulated bad tracks: accept and succeed */
        omti_check_disk_address(s, cdb);
        break;
    case OMTI_CMD_READ_CONFIGURATION:
        omti_set_config_data(s);
        omti_set_data_transfer(s, s->omti_config_data,
                               sizeof(s->omti_config_data));
        break;
    case OMTI_CMD_RAM_DIAGNOSTICS:
    case OMTI_CMD_CONTROLLER_INT_DIAG:
        break;
    case OMTI_CMD_INVALID:
    default:
        omti_set_sense(s, OMTI_SENSE_INVALID_COMMAND, cdb);
        s->omti_cmd_status |= OMTI_CMDSTATUS_ERROR;
        break;
    }

    if (s->omti_mask & OMTI_MASK_INTE) {
        s->omti_status |= OMTI_STATUS_IREQ;
        omti_set_interrupt(s, 1);
    }
}

static uint16_t omti_get_data(ApolloState *s)
{
    uint16_t data = 0xffff;

    if (s->omti_data && s->omti_data_index + 1 < s->omti_data_length) {
        /* big-endian assembly: low disk-byte at the lower CPU address */
        data = (s->omti_data[s->omti_data_index] << 8) |
               s->omti_data[s->omti_data_index + 1];
        s->omti_data_index += 2;
        if (s->omti_data_index >= s->omti_data_length) {
            s->omti_state = OMTI_STATE_STATUS;
            s->omti_status |= OMTI_STATUS_IO | OMTI_STATUS_CD;
        }
    }
    return data;
}

static void omti_set_data(ApolloState *s, uint16_t data)
{
    if (s->omti_data && s->omti_data_index + 1 < s->omti_data_length) {
        s->omti_data[s->omti_data_index] = data >> 8;
        s->omti_data[s->omti_data_index + 1] = data & 0xff;
        s->omti_data_index += 2;
        if (s->omti_data_index >= s->omti_data_length) {
            omti_do_command(s);
        }
    }
}

static void omti_write8(ApolloState *s, unsigned port, uint8_t data)
{
    switch (port) {
    case OMTI_PORT_DATA:
        if (s->omti_state != OMTI_STATE_COMMAND) {
            break;
        }
        if (s->omti_cmd_index == 0) {
            s->omti_cmd_length = (data == 0x20) ? 10 : 6; /* COPY = 10 */
        }
        if (s->omti_cmd_index < s->omti_cmd_length) {
            s->omti_cmd_buf[s->omti_cmd_index++] = data;
        }
        if (s->omti_cmd_index == s->omti_cmd_length) {
#ifdef APOLLO_OMTI_TRACE
            fprintf(stderr,
                    "OMTI CDB cmd=%02x lun/head=%02x c=%02x%02x "
                    "cnt=%02x ctl=%02x\n",
                    s->omti_cmd_buf[0], s->omti_cmd_buf[1],
                    s->omti_cmd_buf[2], s->omti_cmd_buf[3],
                    s->omti_cmd_buf[4], s->omti_cmd_buf[5]);
#endif
            switch (s->omti_cmd_buf[0]) {
            case OMTI_CMD_WRITE:
            case OMTI_CMD_WRITE_SECTOR_BUFFER:
                /*
                 * host -> controller data phase.  WRITE (0x0a) streams the
                 * sectors in from the host; WRITE_SECTOR_BUFFER (0x0f) loads
                 * the controller sector buffer from the host.  WRITE_DATA_FROM_
                 * BUFFER (0x1f) is deliberately NOT here: it flushes the
                 * already-loaded sector buffer to disk with no host transfer,
                 * so it executes immediately via the default path.  Waiting for
                 * a phantom data-out phase would leave the controller stuck in
                 * DATA forever, so INVOL's completion poll times out and it
                 * panics (Crash_Status 00080016).
                 */
                omti_set_data_transfer(s, s->omti_sector_buf,
                                       OMTI_SECTOR_SIZE * s->omti_cmd_buf[4]);
                s->omti_status &= ~OMTI_STATUS_IO;
                break;
            default:
                omti_do_command(s);
                break;
            }
        }
        break;
    case OMTI_PORT_STATUS:      /* write = reset */
        omti_sw_reset(s);
        break;
    case OMTI_PORT_CONFIG:      /* write = select */
        s->omti_state = OMTI_STATE_COMMAND;
        s->omti_status |= OMTI_STATUS_BUSY | OMTI_STATUS_REQ | OMTI_STATUS_CD;
        s->omti_status &= ~OMTI_STATUS_IO;
        s->omti_cmd_status = 0;
        s->omti_cmd_index = 0;
        break;
    case OMTI_PORT_MASK:
        s->omti_mask = data;
        if (!(data & OMTI_MASK_INTE)) {
            s->omti_status &= ~OMTI_STATUS_IREQ;
            omti_set_interrupt(s, 0);
        }
        if (!(data & OMTI_MASK_DMAE)) {
            s->omti_status &= ~OMTI_STATUS_DREQ;
        }
        break;
    }
}

static uint8_t omti_read8(ApolloState *s, unsigned port)
{
    uint8_t data = 0xff;

    switch (port) {
    case OMTI_PORT_DATA:
        if (s->omti_status & OMTI_STATUS_CD) {
            data = s->omti_cmd_status;
            if (s->omti_state == OMTI_STATE_STATUS) {
                s->omti_state = OMTI_STATE_IDLE;
                s->omti_status &= ~(OMTI_STATUS_BUSY | OMTI_STATUS_CD |
                                    OMTI_STATUS_IO | OMTI_STATUS_REQ);
            }
        }
        break;
    case OMTI_PORT_STATUS:
        data = s->omti_status;
        break;
    case OMTI_PORT_CONFIG:
        data = s->omti_config;
        break;
    case OMTI_PORT_MASK:
        data = s->omti_mask;
        break;
    }
    return data;
}

static uint64_t omti_read(void *opaque, hwaddr addr, unsigned size)
{
    ApolloState *s = opaque;

    if (size == 2 && addr == OMTI_PORT_DATA) {
        return omti_get_data(s);
    }
    if (size == 1) {
        return omti_read8(s, addr & 3);
    }
    /* unexpected wide access: assemble from bytes */
    return (omti_read8(s, addr & 3) << 8) | omti_read8(s, (addr + 1) & 3);
}

static void omti_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    ApolloState *s = opaque;

    if (size == 2 && addr == OMTI_PORT_DATA) {
        omti_set_data(s, val);
        return;
    }
    if (size == 1) {
        omti_write8(s, addr & 3, val);
        return;
    }
    omti_write8(s, addr & 3, val >> 8);
    omti_write8(s, (addr + 1) & 3, val & 0xff);
}

static const MemoryRegionOps omti_ops = {
    .read = omti_read,
    .write = omti_write,
    .impl = { .min_access_size = 1, .max_access_size = 2 },
    .valid = { .min_access_size = 1, .max_access_size = 2 },
    .endianness = DEVICE_BIG_ENDIAN,
};

/* ------------------------------------------------------------------ */
/* SC-499 forward declarations (the DMA engine calls into it)         */
static uint8_t sc499_dack_r(ApolloState *s);
static void sc499_dack_w(ApolloState *s, uint8_t data);
static void sc499_eop(ApolloState *s);
static void sc499_deliver_status_byte(ApolloState *s);

/* ------------------------------------------------------------------ */
/* Am9517A DMA controller 1 (functional).  Byte-wide registers at     */
/* 0x9000+reg.  Only the transfer path used by the ctape (channel 1,  */
/* device->memory) is meaningfully exercised; the rest is a faithful  */
/* i8237 register file.  The 24-bit DMA address is                    */
/* (page_register[channel2page[ch]] << 16) | channel 16-bit address.  */

/* MAME channel2page_register[0..3] for controller 1 */
static const uint8_t dma1_channel2page[4] = { 7, 3, 1, 2 };

static void apollo_dma1_run(ApolloState *s, int c)
{
    unsigned type = (s->dma1_ch[c].mode >> 2) & 3;
    int step = (s->dma1_ch[c].mode & 0x20) ? -1 : 1;
    uint32_t page = s->dma_regs[2][dma1_channel2page[c] & 0x1f];
    uint32_t base = page << 16;
    uint32_t n = (uint32_t)s->dma1_ch[c].cur_count + 1;
    uint32_t i;

    for (i = 0; i < n; i++) {
        uint32_t addr = base | s->dma1_ch[c].cur_addr;
        uint8_t byte;

        if (type == 1) {        /* write transfer: device -> memory (read) */
            byte = sc499_dack_r(s);
            address_space_write(&address_space_memory, addr,
                                MEMTXATTRS_UNSPECIFIED, &byte, 1);
        } else if (type == 2) { /* read transfer: memory -> device (write) */
            address_space_read(&address_space_memory, addr,
                               MEMTXATTRS_UNSPECIFIED, &byte, 1);
            sc499_dack_w(s, byte);
        }
        s->dma1_ch[c].cur_addr += step;
    }

    s->dma1_ch[c].cur_count = 0xffff;   /* terminal count underflow */
    s->dma1_status |= 1 << c;           /* TC reached on channel c */
    if (s->dma1_ch[c].mode & 0x10) {    /* autoinit */
        s->dma1_ch[c].cur_addr = s->dma1_ch[c].base_addr;
        s->dma1_ch[c].cur_count = s->dma1_ch[c].base_count;
    } else {
        s->dma1_mask |= 1 << c;
    }
    sc499_eop(s);                       /* EOP -> device (asserted) */
}

static void apollo_dma1_drq(ApolloState *s, int c, int level)
{
    if (level && !(s->dma1_mask & (1 << c)) && !(s->dma1_cmd & 0x04)) {
        apollo_dma1_run(s, c);
    }
}

static uint64_t apollo_dma1_read(void *opaque, hwaddr addr, unsigned size)
{
    ApolloState *s = opaque;
    unsigned reg = addr & 0xf;
    uint8_t data = 0xff;
    int c;

    if (reg < 8) {
        c = reg >> 1;
        if (reg & 1) {          /* count */
            data = s->dma1_ff ? s->dma1_ch[c].cur_count >> 8
                              : s->dma1_ch[c].cur_count & 0xff;
        } else {                /* address */
            data = s->dma1_ff ? s->dma1_ch[c].cur_addr >> 8
                              : s->dma1_ch[c].cur_addr & 0xff;
        }
        s->dma1_ff = !s->dma1_ff;
    } else if (reg == 8) {      /* status: reading clears the TC bits */
        data = s->dma1_status;
        s->dma1_status &= 0xf0;
    }
    return data;
}

static void apollo_dma1_write(void *opaque, hwaddr addr, uint64_t val,
                              unsigned size)
{
    ApolloState *s = opaque;
    unsigned reg = addr & 0xf;
    uint8_t data = val;
    int c;

    if (reg < 8) {
        c = reg >> 1;
        if (reg & 1) {          /* count */
            if (!s->dma1_ff) {
                s->dma1_ch[c].base_count =
                    (s->dma1_ch[c].base_count & 0xff00) | data;
            } else {
                s->dma1_ch[c].base_count =
                    (s->dma1_ch[c].base_count & 0x00ff) | (data << 8);
            }
            s->dma1_ch[c].cur_count = s->dma1_ch[c].base_count;
        } else {                /* address */
            if (!s->dma1_ff) {
                s->dma1_ch[c].base_addr =
                    (s->dma1_ch[c].base_addr & 0xff00) | data;
            } else {
                s->dma1_ch[c].base_addr =
                    (s->dma1_ch[c].base_addr & 0x00ff) | (data << 8);
            }
            s->dma1_ch[c].cur_addr = s->dma1_ch[c].base_addr;
        }
        s->dma1_ff = !s->dma1_ff;
        return;
    }
    switch (reg) {
    case 8:                     /* command */
        s->dma1_cmd = data;
        break;
    case 9:                     /* request (software) - ignored */
        break;
    case 0xa:                   /* single mask bit */
        if (data & 4) {
            s->dma1_mask |= 1 << (data & 3);
        } else {
            s->dma1_mask &= ~(1 << (data & 3));
        }
        break;
    case 0xb:                   /* mode */
        s->dma1_ch[data & 3].mode = data;
        break;
    case 0xc:                   /* clear byte pointer flip-flop */
        s->dma1_ff = false;
        break;
    case 0xd:                   /* master clear */
        s->dma1_ff = false;
        s->dma1_mask = 0x0f;
        s->dma1_status = 0;
        s->dma1_cmd = 0;
        break;
    case 0xe:                   /* clear mask */
        s->dma1_mask = 0;
        break;
    case 0xf:                   /* write all mask bits */
        s->dma1_mask = data & 0x0f;
        break;
    }
    /*
     * A device asserts DRQ (e.g. the ctape when DMAGO is written) before
     * the driver has finished programming the channel; the transfer must
     * start when channel 1 is finally unmasked with DRQ still pending.
     * Modelling that (rather than a fixed timer delay) keeps the tape
     * transfer deterministic.
     */
    if (s->ct_drq_state && !(s->dma1_mask & (1 << 1)) &&
        !(s->dma1_cmd & 0x04)) {
        apollo_dma1_run(s, 1);
    }
}

static const MemoryRegionOps apollo_dma1_ops = {
    .read = apollo_dma1_read,
    .write = apollo_dma1_write,
    .impl = { .min_access_size = 1, .max_access_size = 1 },
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .endianness = DEVICE_BIG_ENDIAN,
};

/* ------------------------------------------------------------------ */
/* SC-499 cartridge-tape controller                                   */

static void sc499_set_interrupt(ApolloState *s, int state)
{
    if (state != s->ct_irq_state) {
        if (getenv("APOLLO_DEVTRACE")) {
            uint32_t pc = current_cpu ? M68K_CPU(current_cpu)->env.pc : 0;
            fprintf(stderr, "[DEV] SC499 irq=%d pc=%08x\n", state, pc);
        }
        if (s->sc499_irq) {
            qemu_set_irq(s->sc499_irq, state);
        }
        s->ct_irq_state = state;
    }
}

static void sc499_set_dma_drq(ApolloState *s, int state)
{
    if (state != s->ct_drq_state) {
        s->ct_drq_state = state;
        apollo_dma1_drq(s, 1, state);   /* ctape = DRQ1 -> DMA1 ch1 */
    }
}

static void sc499_tape_status_set(ApolloState *s, uint16_t value)
{
    s->ct_tape_status |= value;
    s->ct_tape_status &= ~(SC499_ST0 | SC499_ST1);
    if (s->ct_tape_status & SC499_ST0_MASK) {
        s->ct_tape_status |= SC499_ST0;
    }
    if (s->ct_tape_status & SC499_ST1_MASK) {
        s->ct_tape_status |= SC499_ST1;
    }
}

static void sc499_tape_status_clear(ApolloState *s, uint16_t value)
{
    s->ct_tape_status &= ~value;
    sc499_tape_status_set(s, 0);
}

static void sc499_check_tape(ApolloState *s)
{
    s->ct_tape_status = 0;
    s->ct_is_writable = s->ct_blk && blk_is_writable(s->ct_blk);
    if (s->ct_blk) {
        int64_t len = blk_getlength(s->ct_blk);

        s->ct_has_cartridge = 1;
        sc499_tape_status_set(s, SC499_ST1_BOM);
        if (!s->ct_is_writable) {
            sc499_tape_status_set(s, SC499_ST0_WP);
        }
        s->ct_image_length = len > 0 ? len : 0;
        s->ct_block_count = (s->ct_image_length + SC499_BLOCK_SIZE - 1) /
                            SC499_BLOCK_SIZE;
    } else {
        s->ct_has_cartridge = 0;
        sc499_tape_status_set(s, SC499_ST0_NOC);
        s->ct_image_length = 0;
        s->ct_block_count = 0;
    }
}

static int sc499_block_is_filemark(ApolloState *s)
{
    static const uint8_t fm[4] = { 0xDE, 0xAF, 0xFA, 0xED };

    return memcmp(s->ct_block_buf, fm, 4) == 0 &&
           memcmp(s->ct_block_buf, s->ct_block_buf + 4,
                  SC499_BLOCK_SIZE - 4) == 0;
}

static void sc499_read_block(ApolloState *s)
{
    if (s->ct_tape_pos == 0) {
        sc499_check_tape(s);
    }

    if (!s->ct_blk || s->ct_tape_pos >= s->ct_block_count ||
        blk_pread(s->ct_blk, (int64_t)s->ct_tape_pos * SC499_BLOCK_SIZE,
                  SC499_BLOCK_SIZE, s->ct_block_buf, 0) < 0) {
        /* no tape or beyond end-of-tape */
        s->ct_status &= ~(SC499_STAT_EXC | SC499_STAT_DIR | SC499_STAT_DON);
        sc499_tape_status_clear(s, SC499_ST_CLEAR_ALL);
        sc499_tape_status_set(s, SC499_ST_READ_ERROR);
        return;
    }

    s->ct_block_index = 0;
    s->ct_tape_pos++;
    if (s->ct_first_block_hack) {
        /* MAME: the first block must be read twice (MD 'di c'/'ld'/'ex') */
        s->ct_tape_pos = 0;
    }
    s->ct_first_block_hack = 0;

    sc499_tape_status_clear(s, SC499_ST1_BOM);
    if (sc499_block_is_filemark(s)) {
        s->ct_status &= ~(SC499_STAT_EXC | SC499_STAT_DIR);
        sc499_tape_status_clear(s, SC499_ST_CLEAR_ALL);
        sc499_tape_status_set(s, SC499_ST0_FM);
    } else {
        sc499_tape_status_clear(s, SC499_ST0_FM);
    }
}

static void sc499_write_block(ApolloState *s)
{
    if (s->ct_tape_pos == 0) {
        sc499_check_tape(s);
    }
    if (s->ct_blk) {
        blk_pwrite(s->ct_blk, (int64_t)s->ct_tape_pos * SC499_BLOCK_SIZE,
                   SC499_BLOCK_SIZE, s->ct_block_buf, 0);
    }
    s->ct_block_count = s->ct_tape_pos;
    s->ct_block_index = 0;
    s->ct_tape_pos++;
    sc499_tape_status_clear(s, SC499_ST1_BOM);
}

/* DMA data path */
static uint8_t sc499_dack_r(ApolloState *s)
{
    if (s->ct_block_index >= SC499_BLOCK_SIZE) {
        sc499_read_block(s);
        s->ct_nasty_readahead++;
        if (sc499_block_is_filemark(s)) {
            sc499_set_dma_drq(s, 0);
            s->ct_status &= ~(SC499_STAT_EXC | SC499_STAT_DIR);
        }
    }
    return s->ct_block_buf[s->ct_block_index++];
}

static void sc499_dack_w(ApolloState *s, uint8_t data)
{
    if (s->ct_block_index < SC499_BLOCK_SIZE) {
        s->ct_block_buf[s->ct_block_index++] = data;
    }
    if (s->ct_block_index == SC499_BLOCK_SIZE) {
        sc499_write_block(s);
    }
}

static void sc499_eop(ApolloState *s)
{
    s->ct_status |= SC499_STAT_DON;
    sc499_set_dma_drq(s, 0);
    switch (s->ct_current_command & SC499_CMD_TYPE_MASK) {
    case SC499_CMD_READ_DATA:
        s->ct_read_block_pending = 0;
        s->ct_underrun = 0;     /* block consumed: no underrun */
        break;
    case SC499_CMD_WRITE_DATA:
        s->ct_status &= ~SC499_STAT_RDY;
        if ((s->ct_control & SC499_CTR_IEN) &&
            (s->ct_control & SC499_CTR_DNI)) {
            sc499_set_interrupt(s, 1);
            s->ct_status |= SC499_STAT_IRQ;
        }
        break;
    }
}

/* timers */
static void sc499_timer_arm(ApolloState *s, int param, int64_t us)
{
    s->ct_timer_param = param;
    timer_mod(s->ct_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + us * 1000);
}

static void sc499_timer1_arm(ApolloState *s, int param, int64_t initial_us,
                             int64_t period_us)
{
    s->ct_timer1_param = param;
    s->ct_timer1_period = period_us;
    timer_mod(s->ct_timer1,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + initial_us * 1000);
}

static void sc499_timer1_stop(ApolloState *s)
{
    s->ct_timer1_period = 0;
    timer_del(s->ct_timer1);
}

static void sc499_do_command(ApolloState *s, uint8_t data)
{
    s->ct_status |= SC499_STAT_RDY;
    s->ct_status &= ~SC499_STAT_DON;
    s->ct_read_block_pending = 0;

    switch (data & SC499_CMD_TYPE_MASK) {
    case SC499_CMD_SELECT:
        sc499_timer_arm(s, SC499_TIMER_1, 100);
        break;
    case SC499_CMD_POSITION:
        s->ct_first_block_hack = 0;
        s->ct_tape_pos = 0;
        sc499_tape_status_clear(s, SC499_ST_CLEAR_ALL);
        sc499_tape_status_set(s, SC499_ST1_BOM);
        sc499_timer_arm(s, SC499_TIMER_4, 500);
        break;
    case SC499_CMD_WRITE_DATA:
        if (!s->ct_is_writable) {
            s->ct_status &= ~SC499_STAT_EXC;
        }
        sc499_timer_arm(s, SC499_TIMER_1, 100);
        break;
    case SC499_CMD_WRITE_FILEMARK:
        if (!s->ct_is_writable) {
            s->ct_status &= ~SC499_STAT_EXC;
        } else {
            memset(s->ct_block_buf, 0, SC499_BLOCK_SIZE);
            s->ct_block_buf[0] = 0xDE; s->ct_block_buf[1] = 0xAF;
            s->ct_block_buf[2] = 0xFA; s->ct_block_buf[3] = 0xED;
            for (int i = 4; i < SC499_BLOCK_SIZE; i++) {
                s->ct_block_buf[i] = s->ct_block_buf[i - 4];
            }
            sc499_write_block(s);
        }
        sc499_timer_arm(s, SC499_TIMER_1, 100);
        break;
    case SC499_CMD_READ_DATA:
        /*
         * timer1 pre-reads each block (setting ct_block_index=0) and
         * clears RDY to signal the driver a block is ready to DMA; the
         * DMA burst then consumes it (dack_r, index 0..BLOCK_SIZE) and
         * eop resets the underrun counter.  DMA fires deterministically
         * on channel-1 unmask (apollo_dma1_write), and icount keeps the
         * timer pacing reproducible.
         */
        s->ct_status |= SC499_STAT_DIR;
        s->ct_nasty_readahead = 0;
        sc499_timer1_arm(s, SC499_TIMER_5, 200, SC499_READ_BLOCK_TIME_US);
        break;
    case SC499_CMD_READ_FILE_MARK:
        if (s->ct_current_command == SC499_CMD_READ_DATA) {
            s->ct_status &= ~SC499_STAT_DIR;
        } else {
            sc499_timer1_arm(s, SC499_TIMER_6, 200,
                             SC499_READ_BLOCK_TIME_US);
        }
        break;
    case SC499_CMD_READ_STATUS:
        s->ct_status |= SC499_STAT_DIR | SC499_STAT_EXC;
        sc499_set_interrupt(s, 0);
        sc499_set_dma_drq(s, 0);
        sc499_timer1_stop(s);
        s->ct_data_index = 0;
        sc499_timer_arm(s, SC499_TIMER_4, 100);
        break;
    default:
        sc499_tape_status_clear(s, SC499_ST_CLEAR_ALL);
        sc499_tape_status_set(s, SC499_ST1 | SC499_ST1_ILL);
        s->ct_status &= ~SC499_STAT_EXC;
        sc499_timer_arm(s, SC499_TIMER_1, 200);
        break;
    }
}

static void sc499_do_reset(ApolloState *s)
{
    s->ct_data = 0;
    s->ct_command = SC499_CMD_NO_COMMAND;
    s->ct_current_command = s->ct_command;
    s->ct_status = ~(SC499_STAT_DIR | SC499_STAT_EXC);
    s->ct_control = 0;
    s->ct_first_block_hack = 1;
    s->ct_nasty_readahead = 0;
    s->ct_read_block_pending = 0;
    if (s->ct_has_cartridge) {
        sc499_tape_status_set(s, SC499_ST1_POR | SC499_ST1_BOM);
    }
    s->ct_tape_pos = 0;
    s->ct_data_err = 0;
    s->ct_underrun = 0;
    sc499_set_interrupt(s, 0);
    sc499_set_dma_drq(s, 0);
    sc499_timer1_stop(s);
}

/* deliver the next READ STATUS byte into the data port (synchronous) */
static void sc499_deliver_status_byte(ApolloState *s)
{
    s->ct_status &= ~SC499_STAT_RDY;
    switch (++s->ct_data_index) {
    case 1:
        s->ct_data = s->ct_tape_status >> 8;
        break;
    case 2:
        s->ct_data = s->ct_tape_status & 0xff;
        break;
    case 3:
        s->ct_data = s->ct_data_err >> 8;
        break;
    case 4:
        s->ct_data = s->ct_data_err & 0xff;
        break;
    case 5:
        s->ct_data = s->ct_underrun >> 8;
        break;
    case 6:
        s->ct_data = s->ct_underrun & 0xff;
        break;
    default:
        s->ct_data_index = 0;
        s->ct_command = SC499_CMD_NO_COMMAND;
        s->ct_current_command = s->ct_command;
        s->ct_status &= ~SC499_STAT_DIR;
        sc499_tape_status_clear(s, SC499_ST_CLEAR_ALL);
        break;
    }
    if (s->ct_control & SC499_CTR_IEN) {
        sc499_set_interrupt(s, 1);
        s->ct_status |= SC499_STAT_IRQ;
    }
}

static void sc499_timer_cb(void *opaque)
{
    ApolloState *s = opaque;

    switch (s->ct_timer_param) {
    case SC499_TIMER_1:         /* set ready + interrupt */
        s->ct_status &= ~SC499_STAT_RDY;
        if (s->ct_control & SC499_CTR_IEN) {
            sc499_set_interrupt(s, 1);
            s->ct_status |= SC499_STAT_IRQ;
        }
        break;
    case SC499_TIMER_4:         /* deliver next READ STATUS byte */
        sc499_deliver_status_byte(s);
        break;
    case SC499_TIMER_7:         /* reset done: set exception */
        s->ct_status &= ~SC499_STAT_EXC;
        if (s->ct_control & SC499_CTR_IEN) {
            sc499_set_interrupt(s, 1);
            s->ct_status |= SC499_STAT_IRQ;
        }
        break;
    case SC499_TIMER_3:         /* start dma to read data */
        sc499_set_dma_drq(s, 1);
        s->ct_status |= SC499_STAT_RDY;
        break;
    }
}

static void sc499_timer1_cb(void *opaque)
{
    ApolloState *s = opaque;
    int param = s->ct_timer1_param;

    /* SC499_TIMER_5 / _6: read next data block */
    if (s->ct_read_block_pending && (s->ct_status & SC499_STAT_DIR)) {
        if (++s->ct_underrun >= 5000) {
            sc499_timer1_stop(s);
            s->ct_status &= ~(SC499_STAT_EXC | SC499_STAT_DIR);
            sc499_tape_status_clear(s, SC499_ST_CLEAR_ALL);
            sc499_tape_status_set(s, SC499_ST_READ_ERROR);
        }
    } else if (s->ct_tape_pos > s->ct_block_count ||
               !(s->ct_status & SC499_STAT_RDY)) {
        sc499_timer1_stop(s);
        s->ct_status &= ~(SC499_STAT_EXC | SC499_STAT_DIR);
        sc499_tape_status_clear(s, SC499_ST_CLEAR_ALL);
        sc499_tape_status_set(s, SC499_ST_READ_ERROR);
    } else if (s->ct_nasty_readahead > 0) {
        s->ct_nasty_readahead = 0;
    } else {
        sc499_read_block(s);
        s->ct_underrun = 0;
        if (sc499_block_is_filemark(s)) {
            sc499_timer1_stop(s);
        } else if (s->ct_current_command == SC499_CMD_READ_DATA) {
            s->ct_read_block_pending = 1;
            s->ct_status &= ~SC499_STAT_RDY;
        }
    }

    if (s->ct_control & SC499_CTR_IEN) {
        if (s->ct_current_command == SC499_CMD_READ_DATA ||
            (s->ct_status & SC499_STAT_EXC) == 0) {
            sc499_set_interrupt(s, 1);
            s->ct_status |= SC499_STAT_IRQ;
        }
    }

    if (s->ct_timer1_period > 0) {
        s->ct_timer1_param = param;
        timer_mod(s->ct_timer1, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  s->ct_timer1_period * 1000);
    }
}

/* register ports */
static void sc499_write_control(ApolloState *s, uint8_t data)
{
    if ((data ^ s->ct_control) & SC499_CTR_RST) {
        if (data & SC499_CTR_RST) {
            sc499_do_reset(s);
        } else {
            /*
             * Reset-complete handshake, done deterministically instead of
             * on a wall-clock timer: EXC reads back set once (reset in
             * progress) then clears on the next status read.
             */
            s->ct_status |= SC499_STAT_EXC;
            s->ct_reset_pending = 1;
        }
    }
    if ((data ^ s->ct_control) & SC499_CTR_REQ) {
        if (data & SC499_CTR_REQ) {
            if (s->ct_command == SC499_CMD_READ_STATUS) {
                s->ct_status |= SC499_STAT_EXC;
            }
            if (s->ct_command == SC499_CMD_READ_FILE_MARK &&
                s->ct_current_command == SC499_CMD_READ_DATA) {
                s->ct_current_command = SC499_CMD_READ_FILE_MARK;
                s->ct_status &= ~SC499_STAT_DIR;
            }
            if (!(s->ct_status & SC499_STAT_DIR)) {
                s->ct_status &= ~SC499_STAT_RDY;
            } else {
                s->ct_status |= SC499_STAT_RDY;
            }
        } else {
            if (!(s->ct_status & SC499_STAT_DIR)) {
                sc499_do_command(s, s->ct_command);
                s->ct_current_command = s->ct_command;
            } else if (s->ct_command == SC499_CMD_READ_STATUS) {
                sc499_timer_arm(s, SC499_TIMER_4, 20);
            }
        }
    }
    s->ct_control = data;
}

static uint8_t sc499_read_data(ApolloState *s)
{
    if (s->ct_control & SC499_CTR_IEN) {
        sc499_set_interrupt(s, 0);
        s->ct_status &= ~SC499_STAT_IRQ;
    }
    return s->ct_data;
}

static uint8_t sc499_read_status(ApolloState *s)
{
    uint8_t data = s->ct_status;

    sc499_set_interrupt(s, 0);
    s->ct_status &= ~SC499_STAT_IRQ;
    if (s->ct_reset_pending) {
        /* first read after reset returns EXC set, then reset completes */
        s->ct_reset_pending = 0;
        s->ct_status &= ~SC499_STAT_EXC;
        if (s->ct_control & SC499_CTR_IEN) {
            sc499_set_interrupt(s, 1);
            s->ct_status |= SC499_STAT_IRQ;
        }
    }
    return data;
}

static uint64_t sc499_read(void *opaque, hwaddr addr, unsigned size)
{
    ApolloState *s = opaque;

    switch (addr & 7) {
    case SC499_PORT_DATA:
        return sc499_read_data(s);
    case SC499_PORT_STATUS:
        return sc499_read_status(s);
    default:
        return 0xff;
    }
}

static void sc499_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    ApolloState *s = opaque;
    uint8_t data = val;

    switch (addr & 7) {
    case SC499_PORT_DATA:       /* write command */
        s->ct_command = data;
        break;
    case SC499_PORT_STATUS:     /* write control */
        sc499_write_control(s, data);
        break;
    case SC499_PORT_DMAGO:      /* start DMA */
        s->ct_status &= ~SC499_STAT_DON;
        if (data == 0) {
            /*
             * Assert DRQ now; the actual burst fires once the driver
             * finishes programming DMA controller 1 and unmasks channel 1
             * (see apollo_dma1_write).  RDY reflects that DMA is armed.
             */
            s->ct_status |= SC499_STAT_RDY;
            sc499_set_dma_drq(s, 1);
        }
        break;
    case SC499_PORT_RSTDMA:     /* reset DMA */
        s->ct_status &= ~SC499_STAT_DON;
        s->ct_control = 0;
        break;
    }
}

static const MemoryRegionOps sc499_ops = {
    .read = sc499_read,
    .write = sc499_write,
    .impl = { .min_access_size = 1, .max_access_size = 1 },
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .endianness = DEVICE_BIG_ENDIAN,
};

static void sc499_reset(ApolloState *s)
{
    s->ct_data = 0;
    s->ct_command = SC499_CMD_NO_COMMAND;
    s->ct_status = SC499_STAT_RDY;
    s->ct_control = 0;
    s->ct_first_block_hack = 1;
    s->ct_nasty_readahead = 0;
    s->ct_read_block_pending = 0;
    s->ct_current_command = s->ct_command;
    s->ct_data_err = 0;
    s->ct_underrun = 0;
    s->ct_image_length = 0;
    sc499_check_tape(s);
    s->ct_data_index = 0;
    s->ct_block_index = SC499_BLOCK_SIZE;
    s->ct_tape_pos = 0;
    s->ct_irq_state = 0;
    s->ct_drq_state = 0;
    s->ct_timer1_period = 0;
    if (s->ct_timer) {
        timer_del(s->ct_timer);
    }
    if (s->ct_timer1) {
        timer_del(s->ct_timer1);
    }
}

/* ------------------------------------------------------------------ */
/* Machine                                                            */

/* single-instance board; used by the machine-level reset hook */
static ApolloState *apollo_singleton;

/*
 * Program the dual 8259 the way the boot PROM would have.  Linux's
 * Apollo port (dn_init_IRQ/apollo_irq_chip) never issues the 8259 ICW
 * init sequence - it assumes the PROM already set the vector base to
 * APOLLO_IRQ_VECTOR (0xa0) and left the on-board interrupt sources
 * unmasked.  On -kernel boot the PROM never runs, so replay it here:
 * master vectors 0xa0-0xa7, slave 0xa8-0xaf, slave cascaded on master
 * IR3, and the master IMR leaving the PTM timer (IR0) and the cascade
 * (IR3) enabled (the kernel's irq_startup only touches the slave mask).
 */
static void apollo_program_pics(ApolloState *s)
{
    static const uint8_t master_init[] = {
        0x11,   /* ICW1: edge, cascade, ICW4 follows */
        0xa0,   /* ICW2: vector base 0xa0 */
        0x08,   /* ICW3: slave attached on IR3 */
        0x01,   /* ICW4: 8086 mode, normal EOI */
    };
    static const uint8_t slave_init[] = {
        0x11,   /* ICW1 */
        0xa8,   /* ICW2: vector base 0xa8 */
        0x03,   /* ICW3: slave ID 3 */
        0x01,   /* ICW4 */
    };
    AddressSpace *as = &address_space_memory;
    uint8_t v;
    int i;

    v = master_init[0];
    address_space_write(as, APOLLO_PIC1_ADDR, MEMTXATTRS_UNSPECIFIED, &v, 1);
    for (i = 1; i < 4; i++) {
        address_space_write(as, APOLLO_PIC1_ADDR + 1, MEMTXATTRS_UNSPECIFIED,
                            &master_init[i], 1);
    }
    v = slave_init[0];
    address_space_write(as, APOLLO_PIC2_ADDR, MEMTXATTRS_UNSPECIFIED, &v, 1);
    for (i = 1; i < 4; i++) {
        address_space_write(as, APOLLO_PIC2_ADDR + 1, MEMTXATTRS_UNSPECIFIED,
                            &slave_init[i], 1);
    }
    /* master IMR: unmask IR0 (timer) and IR3 (cascade) */
    v = (uint8_t)~(0x01 | 0x08);
    address_space_write(as, APOLLO_PIC1_ADDR + 1, MEMTXATTRS_UNSPECIFIED,
                        &v, 1);
    /* slave IMR: all masked; the kernel unmasks what it needs */
    v = 0xff;
    address_space_write(as, APOLLO_PIC2_ADDR + 1, MEMTXATTRS_UNSPECIFIED,
                        &v, 1);
}

/*
 * The Apollo vmlinux is linked at physical 0 but RAM starts at 1 MB, so
 * bias every ELF segment (and the entry) up by the RAM base.  The kernel
 * is PC-relative at entry and recomputes its own physical base, so any
 * fixed offset into RAM works.
 */
static uint64_t apollo_kernel_translate(void *opaque, uint64_t addr)
{
    return addr + APOLLO_RAM_BASE;
}

/*
 * The 8259s are qdev devices whose reset runs *after* the legacy reset
 * handlers (including apollo_cpu_reset), so they can only be programmed
 * once the full device reset has completed - hence a machine-level reset
 * hook that runs qemu_devices_reset() first, then replays the PROM's PIC
 * setup for the -kernel path.
 */
static void apollo_machine_reset(MachineState *machine, ResetType type)
{
    qemu_devices_reset(type);

    if (apollo_singleton && apollo_singleton->linux_boot) {
        apollo_program_pics(apollo_singleton);
    }
}

static void apollo_cpu_reset(void *opaque)
{
    ApolloState *s = opaque;
    CPUM68KState *env = &s->cpu->env;
    uint8_t *rom_ptr = memory_region_get_ram_ptr(&s->rom);
    uint32_t reset_sp, reset_pc;

    cpu_reset(CPU(s->cpu));

    if (s->linux_boot) {
        /*
         * -kernel direct boot: enter the vmlinux ELF entry point with a
         * temporary supervisor stack.  head.S sets up its own stack and
         * the MMU almost immediately.
         */
        reset_pc = s->reset_pc;
        reset_sp = s->reset_sp;
    } else {
        /*
         * 68020 reset: SSP and PC from the exception vectors in the PROM.
         * Read at reset time: the ROM loader only copies the image in at
         * the first system reset, after machine init.
         */
        reset_sp = ldl_be_p(rom_ptr);
        reset_pc = ldl_be_p(rom_ptr + 4);
    }
    env->pc = reset_pc;
    env->aregs[7] = reset_sp;
    env->sp[env->current_sp] = reset_sp;

    /* board boots in Service Mode; BIT15 mirrors MAME's behaviour */
    s->csr_sr = CSR_SR_BIT15 | CSR_SR_SERVICE;
    s->csr_cr = 0;

    /* reset the OMTI Winchester controller */
    omti_sw_reset(s);

    /* reset the SC-499 cartridge-tape controller and DMA controller 1 */
    if (s->ct_timer) {
        sc499_reset(s);
    }
    memset(s->dma1_ch, 0, sizeof(s->dma1_ch));
    s->dma1_mask = 0x0f;
    s->dma1_cmd = 0;
    s->dma1_status = 0;
    s->dma1_ff = false;
}

static void apollo_dn3000_init(MachineState *machine)
{
    ApolloState *s = g_new0(ApolloState, 1);
    MemoryRegion *sysmem = get_system_memory();
    MemoryRegion *isa_io_container = g_new(MemoryRegion, 1);
    MemoryRegion *isa_mem_container = g_new(MemoryRegion, 1);
    ISABus *isa_bus;
    ISADevice *pic_dev;
    DeviceState *sio_dev;
    const char *bios_name;
    char *filename;
    int64_t bios_size;
    int i;

    if (machine->ram_size > APOLLO_RAM_MAX) {
        error_report("apollo-dn3000 supports at most %d MiB of RAM",
                     (int)(APOLLO_RAM_MAX / MiB));
        exit(1);
    }

    s->cpu = M68K_CPU(cpu_create(machine->cpu_type));
    s->node_id = APOLLO_NODE_ID_DEFAULT;
    s->linux_boot = machine->kernel_filename != NULL;
    apollo_singleton = s;

    /* Bus-error background over the entire address space */
    memory_region_init_io(&s->buserr_mem, NULL, &apollo_buserr_ops, s,
                          "apollo.buserr", UINT64_MAX);
    memory_region_add_subregion_overlap(sysmem, 0, &s->buserr_mem, -2);

    /* boot PROM */
    memory_region_init_rom_device(&s->rom, NULL, &apollo_rom_ops, s,
                                  "apollo.rom", APOLLO_ROM_SIZE,
                                  &error_fatal);
    memory_region_add_subregion(sysmem, APOLLO_ROM_BASE, &s->rom);

    bios_name = machine->firmware;
    if (!bios_name && !s->linux_boot) {
        error_report("apollo-dn3000 requires a boot PROM image (-bios)");
        exit(1);
    }
    if (bios_name) {
        filename = qemu_find_file(QEMU_FILE_TYPE_BIOS, bios_name);
        if (!filename) {
            filename = g_strdup(bios_name);
        }
        /*
         * Load eagerly (not via the deferred ROM loader): the reset
         * handler reads the initial SSP/PC out of the image.
         */
        bios_size = load_image_size(filename,
                                    memory_region_get_ram_ptr(&s->rom),
                                    APOLLO_ROM_SIZE);
        g_free(filename);
        if (bios_size <= 0) {
            error_report("could not load boot PROM '%s'", bios_name);
            exit(1);
        }
    }

    /* RAM */
    memory_region_add_subregion_overlap(sysmem, APOLLO_RAM_BASE,
                                        machine->ram, 1);

    /* AT bus windows: unmapped ISA space floats high, no bus error */
    memory_region_init_io(&s->atbus_io, NULL, &apollo_atbus_ops, s,
                          "apollo.atbus-io", APOLLO_ATBUS_IO_SIZE);
    memory_region_add_subregion(sysmem, APOLLO_ATBUS_IO_BASE, &s->atbus_io);
    memory_region_init_io(&s->atbus_mem, NULL, &apollo_atbus_ops, s,
                          "apollo.atbus-mem", APOLLO_ATBUS_MEM_SIZE);
    memory_region_add_subregion_overlap(sysmem, APOLLO_ATBUS_MEM_BASE,
                                        &s->atbus_mem, 0);

    /* CPU status / control registers */
    memory_region_init_io(&s->csr_sr_mem, NULL, &apollo_csr_sr_ops, s,
                          "apollo.csr-status", 0x100);
    memory_region_add_subregion(sysmem, APOLLO_CSR_SR_ADDR, &s->csr_sr_mem);
    memory_region_init_io(&s->csr_cr_mem, NULL, &apollo_csr_cr_ops, s,
                          "apollo.csr-control", 0x100);
    memory_region_add_subregion(sysmem, APOLLO_CSR_CR_ADDR, &s->csr_cr_mem);

    /* SAU7 CPU-control alias for the kernel head.S set_leds/LCPUCTRL */
    memory_region_init_alias(&s->cpuctrl_alias, NULL, "apollo.cpuctrl-alias",
                             &s->csr_cr_mem, 0, 0x100);
    memory_region_add_subregion(sysmem, APOLLO_CPUCTRL_ALIAS_ADDR,
                                &s->cpuctrl_alias);

    /*
     * Private ISA bus hosting the QEMU i8259 and MC146818 models.
     * Its I/O space is not CPU-visible; the Apollo-specific MMIO
     * wrappers forward register accesses into it.
     */
    memory_region_init(isa_io_container, NULL, "apollo.isa-io", 0x10000);
    memory_region_init(isa_mem_container, NULL, "apollo.isa-mem", 0x1000000);
    isa_bus = isa_bus_new(NULL, isa_mem_container, isa_io_container,
                          &error_abort);

    /* dual i8259, slave INT cascaded into master IR3 */
    pic_dev = i8259_init_chip("isa-i8259", isa_bus, true);
    s->pic[0] = PIC_COMMON(pic_dev);
    pic_dev = i8259_init_chip("isa-i8259", isa_bus, false);
    s->pic[1] = PIC_COMMON(pic_dev);

    qdev_connect_gpio_out(DEVICE(s->pic[0]), 0,
                          qemu_allocate_irq(apollo_pic_int, s, 0));
    qdev_connect_gpio_out(DEVICE(s->pic[1]), 0,
                          qdev_get_gpio_in(DEVICE(s->pic[0]), 3));

    /* the CPU's level-6 interrupt-acknowledge cycle acks the PICs */
    qdev_connect_gpio_out_named(DEVICE(s->cpu), "iack-out", 6,
                                qemu_allocate_irq(apollo_pic_iack, s, 0));

    for (i = 0; i < 2; i++) {
        memory_region_init_io(&s->pic_mem[i], NULL, &apollo_pic_reg_ops,
                              s->pic[i], i ? "apollo.pic-slave"
                                           : "apollo.pic-master", 0x100);
        memory_region_add_subregion(sysmem,
                                    i ? APOLLO_PIC2_ADDR : APOLLO_PIC1_ADDR,
                                    &s->pic_mem[i]);
    }

    /* MC146818 RTC, IRQ on slave PIC input 0 (Apollo IRQ 8) */
    s->rtc = mc146818_rtc_init(isa_bus, 1980,
                               qdev_get_gpio_in(DEVICE(s->pic[1]), 0));
    /*
     * Preset the RTC to a fixed binary-mode date after the kernel build
     * stamp (1992-02-14 11:50).  QEMU's mc146818 powers up in BCD at the
     * host date, so Domain/OS / the SAU utilities see a garbage/"slow"
     * calendar and demand `EX CALENDAR` before they run.  A real Apollo's
     * RTC is battery-backed and already binary; present it that way here so
     * `EX DOMAIN_OS` (and invol) boot without the interactive CALENDAR
     * dance -- which also removes a wall-clock-dependent input, making the
     * (icount) boot deterministic.  Register B: SET(0x80) to halt updates
     * while writing, DM(0x04)=binary, 24/12(0x02)=24h.  Values are written
     * in binary (DM=1) directly.
     */
    {
        static const struct { uint8_t reg, val; } rtcinit[] = {
            { 0x0b, 0x86 },     /* SET | binary | 24h: halt + binary mode  */
            { 0x00, 0 },        /* seconds                                  */
            { 0x02, 30 },       /* minutes                                  */
            { 0x04, 12 },       /* hours (24h)                              */
            { 0x06, 6 },        /* weekday (Fri; apollo_rtc_read recomputes)*/
            { 0x07, 14 },       /* day of month                             */
            { 0x08, 2 },        /* month                                    */
            { 0x09, 92 },       /* year within century -> 1992              */
            { 0x0b, 0x06 },     /* release SET, keep binary + 24h           */
        };
        for (i = 0; i < ARRAY_SIZE(rtcinit); i++) {
            memory_region_dispatch_write(&s->rtc->io, 0, rtcinit[i].reg, MO_8,
                                         MEMTXATTRS_UNSPECIFIED);
            memory_region_dispatch_write(&s->rtc->io, 1, rtcinit[i].val, MO_8,
                                         MEMTXATTRS_UNSPECIFIED);
        }
    }
    memory_region_init_io(&s->rtc_mem, NULL, &apollo_rtc_ops, s,
                          "apollo.rtc", 0x100);
    memory_region_add_subregion(sysmem, APOLLO_RTC_ADDR, &s->rtc_mem);

    /*
     * SIO: SCN2681, ch A keyboard, ch B serial console.  The RAM
     * configuration byte is presented on the input port pins.
     */
    sio_dev = qdev_new(TYPE_SCN2681);
    qdev_prop_set_chr(sio_dev, "chardev-a", serial_hd(1));
    qdev_prop_set_chr(sio_dev, "chardev-b", serial_hd(0));
    {
        uint8_t ramcfg = DN3000_RAM_CONFIG_8MB;
        const char *e = getenv("APOLLO_RAM_CFG");   /* debug: override strap */
        if (e) {
            unsigned long value;

            if (qemu_strtoul(e, NULL, 0, &value) == 0) {
                ramcfg = value;
            }
        }
        qdev_prop_set_uint8(sio_dev, "input-port", ramcfg);
    }
    qdev_prop_set_bit(sio_dev, "apollo-autobaud", true);
    /*
     * For -kernel boot the PROM never runs, so the kernel's head.S early
     * console relies on the SIO already having console channel B enabled
     * ("We count on the PROM initializing SIO1"); pre-enable it.
     */
    qdev_prop_set_bit(sio_dev, "preinit-console", s->linux_boot);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(sio_dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(sio_dev), 0, APOLLO_SIO_ADDR);
    /* SAU7 SIO alias for the kernel head.S early console (LTHRB0/LSRB0) */
    memory_region_init_alias(&s->sio_alias, NULL, "apollo.sio-alias",
                             sysbus_mmio_get_region(SYS_BUS_DEVICE(sio_dev), 0),
                             0, 0x100);
    memory_region_add_subregion(sysmem, APOLLO_SIO_ALIAS_ADDR, &s->sio_alias);
    /* SIO interrupt is Apollo IRQ 1 */
    sysbus_connect_irq(SYS_BUS_DEVICE(sio_dev), 0,
                       qdev_get_gpio_in(DEVICE(s->pic[0]), 1));

    /* MC6840 PTM, IRQ 0 */
    for (i = 0; i < 3; i++) {
        s->ptm[i].as = s;
        s->ptm[i].idx = i;
        s->ptm[i].cr = (i == 0) ? PTM_CR_INTERNAL_RESET : 0;
        s->ptm[i].latch = 0xffff;
        s->ptm[i].timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                       apollo_ptm_expired, &s->ptm[i]);
    }
    memory_region_init_io(&s->ptm_mem, NULL, &apollo_ptm_ops, s,
                          "apollo.ptm", 0x100);
    memory_region_add_subregion(sysmem, APOLLO_PTM_ADDR, &s->ptm_mem);

    /*
     * DMA controller 1 is functional (the ctape streams over channel 1).
     * DMA controller 2, the page registers and the parity latch remain
     * RAM-backed stubs; the page-register storage (dma_regs[2]) supplies
     * the high address bits used by the functional controller 1.
     */
    memory_region_init_io(&s->dma1_mem, NULL, &apollo_dma1_ops, s,
                          "apollo.dma1", 0x100);
    memory_region_add_subregion(sysmem, APOLLO_DMA1_ADDR, &s->dma1_mem);
    for (i = 1; i < 4; i++) {
        static const char *names[4] = {
            "apollo.dma1", "apollo.dma2", "apollo.dma-page",
            "apollo.parity-latch"
        };
        static const hwaddr addrs[4] = {
            APOLLO_DMA1_ADDR, APOLLO_DMA2_ADDR, APOLLO_DMAPAGE_ADDR,
            APOLLO_PARLATCH_ADDR
        };
        memory_region_init_io(&s->dma_mem[i], NULL, &apollo_dma_ops,
                              s->dma_regs[i], names[i], 0x100);
        memory_region_add_subregion(sysmem, addrs[i], &s->dma_mem[i]);
    }

    /* node ID PROM */
    memory_region_init_io(&s->nodeid_mem, NULL, &apollo_nodeid_ops, s,
                          "apollo.node-id", 0x100);
    memory_region_add_subregion(sysmem, APOLLO_NODEID_ADDR, &s->nodeid_mem);

    /*
     * OMTI-8621 Winchester controller: 8-byte register window at the
     * ISA-translated address 0x04d000 (ISA I/O 0x1a0), overlaid on the
     * AT-bus I/O float-high background.  Backed by a raw ".awd" image via
     * -drive if=mtd (unit 0); IRQ14 = slave PIC IR6 (Apollo Winchester).
     */
    {
        DriveInfo *dinfo = drive_get(IF_MTD, 0, 0);

        if (dinfo) {
            s->omti_blk = blk_by_legacy_dinfo(dinfo);
            blk_set_perm(s->omti_blk,
                         BLK_PERM_CONSISTENT_READ | BLK_PERM_WRITE,
                         BLK_PERM_ALL, &error_fatal);
            /* claim the backend so it is not reported as orphaned */
            blk_attach_dev(s->omti_blk, DEVICE(s->cpu));
        }
        s->omti_irq = qdev_get_gpio_in(DEVICE(s->pic[1]), 6);
        s->omti_irq_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, omti_irq_fire, s);
        memory_region_init_io(&s->omti_mem, NULL, &omti_ops, s,
                              "apollo.omti8621", 8);
        memory_region_add_subregion_overlap(sysmem, OMTI_ADDR,
                                            &s->omti_mem, 1);
    }

    /*
     * Archive SC-499 cartridge-tape controller: 8-byte window at the
     * ISA-translated address 0x050000 (ISA I/O 0x200), IRQ5 (master PIC
     * IR5), DRQ1 (DMA controller 1 channel 1).  Backed by a raw ".ct"
     * tape image via -drive if=mtd (unit 1); the medium can be swapped
     * between install tapes from the monitor.
     */
    {
        DriveInfo *dinfo = drive_get(IF_MTD, 0, 1);

        if (dinfo) {
            s->ct_blk = blk_by_legacy_dinfo(dinfo);
            blk_set_perm(s->ct_blk, BLK_PERM_CONSISTENT_READ,
                         BLK_PERM_ALL, &error_fatal);
            blk_attach_dev(s->ct_blk, DEVICE(s->cpu));
        }
        s->sc499_irq = qdev_get_gpio_in(DEVICE(s->pic[0]), 5);
        s->ct_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, sc499_timer_cb, s);
        s->ct_timer1 = timer_new_ns(QEMU_CLOCK_VIRTUAL, sc499_timer1_cb, s);
        memory_region_init_io(&s->sc499_mem, NULL, &sc499_ops, s,
                              "apollo.sc499", 8);
        memory_region_add_subregion_overlap(sysmem, SC499_ADDR,
                                            &s->sc499_mem, 1);
    }

    /* DMA address-translation map (SAU7): plain RAM-backed storage */
    memory_region_init_ram(&s->xlat_mem, NULL, "apollo.xlat-map",
                           APOLLO_XLAT_SIZE, &error_fatal);
    memory_region_add_subregion(sysmem, APOLLO_XLAT_ADDR, &s->xlat_mem);

    /*
     * Display: a linear xrgb8888 framebuffer in a dedicated high-memory
     * window, scanned out to the QEMU display.  Presented to Linux as a
     * "simple-framebuffer" (see apollo-fb.c) so the in-tree simplefb/
     * simpledrm driver binds without a native DRM driver.  Region 0 is the
     * linear buffer (APOLLO_FB_BASE), region 1 the MCR/Bt458 control window.
     */
    {
        DeviceState *fb_dev = qdev_new(TYPE_APOLLO_FB);

        sysbus_realize_and_unref(SYS_BUS_DEVICE(fb_dev), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(fb_dev), 0, APOLLO_FB_BASE);
        sysbus_mmio_map(SYS_BUS_DEVICE(fb_dev), 1, APOLLO_FB_CTRL_BASE);
    }

    /*
     * -kernel: load a raw vmlinux ELF into RAM and hand it a classic
     * m68k bootinfo block, instead of running the PROM.  The kernel is
     * linked at physical 0 but RAM is at 1 MB, so bias the load (and the
     * bootinfo/initrd that follow it) up by the RAM base.
     */
    if (s->linux_boot) {
        CPUState *cs = CPU(s->cpu);
        uint64_t elf_entry, high;
        ssize_t kernel_size;
        hwaddr parameters_base;
        hwaddr top_of_ram = APOLLO_RAM_BASE + machine->ram_size;
        void *param_blob, *param_ptr;

        kernel_size = load_elf(machine->kernel_filename, NULL,
                               apollo_kernel_translate, NULL,
                               &elf_entry, NULL, &high, NULL, ELFDATA2MSB,
                               EM_68K, 0, 0);
        if (kernel_size < 0) {
            error_report("could not load kernel '%s'",
                         machine->kernel_filename);
            exit(1);
        }
        s->reset_pc = elf_entry + APOLLO_RAM_BASE;

        parameters_base = (high + 1) & ~1;
        param_blob = g_malloc(machine->kernel_cmdline ?
                              strlen(machine->kernel_cmdline) + 1024 : 1024);
        param_ptr = param_blob;

        BOOTINFO1(param_ptr, BI_MACHTYPE, MACH_APOLLO);
        /*
         * The DN3000 is a 68020 + external MC68851.  Advertise that even
         * though the emulated core is the 68030 stand-in: the kernel's
         * MMU-enable path for 020 uses the 68851-compatible 030 table
         * format, which the 030 core walks.
         */
        BOOTINFO1(param_ptr, BI_CPUTYPE, CPU_68020);
        BOOTINFO1(param_ptr, BI_MMUTYPE, MMU_68851);
        BOOTINFO1(param_ptr, BI_APOLLO_MODEL, APOLLO_MODEL_DN3000);
        BOOTINFO2(param_ptr, BI_MEMCHUNK, APOLLO_RAM_BASE, machine->ram_size);

        if (machine->kernel_cmdline) {
            BOOTINFOSTR(param_ptr, BI_COMMAND_LINE, machine->kernel_cmdline);
        }

        if (machine->initrd_filename) {
            int64_t initrd_size =
                get_image_size(machine->initrd_filename, NULL);
            hwaddr initrd_base;

            if (initrd_size < 0) {
                error_report("could not load initial ram disk '%s'",
                             machine->initrd_filename);
                exit(1);
            }
            initrd_base = (top_of_ram - initrd_size) & TARGET_PAGE_MASK;
            load_image_targphys(machine->initrd_filename, initrd_base,
                                top_of_ram - initrd_base, &error_fatal);
            BOOTINFO2(param_ptr, BI_RAMDISK, initrd_base, initrd_size);
            top_of_ram = initrd_base;
        }

        BOOTINFO0(param_ptr, BI_LAST);
        rom_add_blob_fixed_as("bootinfo", param_blob, param_ptr - param_blob,
                              parameters_base, cs->as);
        g_free(param_blob);

        s->reset_sp = (top_of_ram - 0x1000) & ~3;
    }

    /* reset: SSP/PC straight out of the PROM's vector table */
    qemu_register_reset(apollo_cpu_reset, s);
}

static void apollo_dn3000_machine_init(MachineClass *mc)
{
    mc->desc = "Apollo DN3000 (68020)";
    mc->init = apollo_dn3000_init;
    /*
     * The DN3000 is a 68020 with an external MC68851 PMMU.  With real
     * 68851 support now in target/m68k (the add-68851 work), model it
     * faithfully: the PROM probes the PMMU (pmove %d0,%tc) and Linux
     * turns on paging through the 68851's long-format tables.
     */
    mc->default_cpu_type = M68K_CPU_TYPE_NAME("m68020-68851");
    mc->reset = apollo_machine_reset;
    mc->max_cpus = 1;
    mc->default_ram_size = APOLLO_RAM_MAX;
    mc->default_ram_id = "apollo.ram";
    mc->no_floppy = 1;
    mc->no_parallel = 1;
}

DEFINE_MACHINE("apollo-dn3000", apollo_dn3000_machine_init)
