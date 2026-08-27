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

    memory_region_dispatch_write(&pic->base_io, addr & 1, val, MO_8,
                                 MEMTXATTRS_UNSPECIFIED);
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

static uint64_t apollo_rtc_read(void *opaque, hwaddr addr, unsigned size)
{
    ApolloState *s = opaque;
    uint64_t val = 0xff;

    memory_region_dispatch_write(&s->rtc->io, 0, addr & 0x7f, MO_8,
                                 MEMTXATTRS_UNSPECIFIED);
    memory_region_dispatch_read(&s->rtc->io, 1, &val, MO_8,
                                MEMTXATTRS_UNSPECIFIED);
    return val;
}

static void apollo_rtc_write(void *opaque, hwaddr addr, uint64_t val,
                             unsigned size)
{
    ApolloState *s = opaque;

    memory_region_dispatch_write(&s->rtc->io, 0, addr & 0x7f, MO_8,
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
    memory_region_init_io(&s->rtc_mem, NULL, &apollo_rtc_ops, s,
                          "apollo.rtc", 0x100);
    memory_region_add_subregion(sysmem, APOLLO_RTC_ADDR, &s->rtc_mem);

    /* SIO: SCN2681, ch A keyboard, ch B serial console.  The RAM
     * configuration byte is presented on the input port pins. */
    sio_dev = qdev_new(TYPE_SCN2681);
    qdev_prop_set_chr(sio_dev, "chardev-a", serial_hd(1));
    qdev_prop_set_chr(sio_dev, "chardev-b", serial_hd(0));
    qdev_prop_set_uint8(sio_dev, "input-port", DN3000_RAM_CONFIG_8MB);
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

    /* DMA controllers / page registers / parity latch stubs */
    for (i = 0; i < 4; i++) {
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
