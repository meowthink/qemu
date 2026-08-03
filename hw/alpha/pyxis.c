/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * DEC 21174 ("Pyxis") PCI host bridge emulation.
 *
 * References:
 *   21174 Core Logic Chip Technical Reference Manual, EC-R12GB-TE
 *   AlphaPC 164LX Motherboard Technical Reference Manual, EC-R46WC-TE
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "qemu/units.h"
#include "qemu/cutils.h"
#include "qemu/bitops.h"
#include "qapi/error.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/sysbus.h"
#include "hw/intc/intc.h"
#include "hw/intc/i8259.h"
#include "hw/isa/isa.h"
#include "hw/pci/pci_host.h"
#include "hw/pci/pci_device.h"
#include "hw/pci/pci_bus.h"
#include "migration/vmstate.h"
#include "qemu/error-report.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "system/dma.h"
#include "system/memory.h"
#include "pyxis-internal.h"
#include "trace.h"

/*
 * Chipset Implementation
 */

static const Property pyxis_chipset_properties[] = {
    DEFINE_PROP_UINT32("ram-size", PyxisState, ram_size, 0), /* MiB */
    DEFINE_PROP_UINT32("num-cpus", PyxisState, num_cpus, 1),
    DEFINE_PROP_UINT32("revision", PyxisState, revision, 0x00000000),
};

/*
 * Interrupt routing (HRM 2.7, 5.9).
 *
 * Eight external interrupt sources are read through a shift register;
 * the INT_ROUTE register selects which 21164 irq_h line each source is
 * delivered to.  On the AlphaPC 164LX the PLD output (pci_isa_irq) is
 * source 0, the TOY/RTC interrupt is source 1 and everything else is
 * unused.
 */

static void pyxis_irq_update(PyxisState *s)
{
    bool lvl[4] = {};
    int i;

    for (i = 0; i < PYXIS_NIRQS; i++) {
        if (s->irq_level[i] && !((s->int_mask >> i) & 1)) {
            int n = (s->int_route >> (2 * i)) & 3;

            lvl[n] = true;
        }
    }

    for (i = 0; i < 4; i++) {
        qemu_set_irq(s->cpu_irq[i], lvl[i]);
    }
}

/*
 * Interrupt PLD (HRM 4.4.1 / TRM fig 4-5).
 *
 * The MACH210A PLD ORs the 16 PCI slot interrupts, the SIO interrupt
 * and the IDE interrupt into pci_isa_irq, which feeds 21174 interrupt
 * source 0.  Reads at ISA ports 804-806 return the raw interrupt state;
 * writes program the write-only interrupt mask (1 = disabled).
 */

static uint32_t pld_status(PyxisState *s)
{
    PyxisPchipState *pcs = &s->pchip;
    uint32_t lvl = pcs->pci_irq_level;
    uint32_t st = 0;

    /* Port 0x804: INTA0-3, SIO, IDE, INTB0 */
    if (lvl & BIT(0)) {
        st |= BIT(0);                   /* INTA0 (slot 2) */
    }
    if (lvl & BIT(4)) {
        st |= BIT(1);                   /* INTA1 (slot 3) */
    }
    if (lvl & BIT(8)) {
        st |= BIT(2);                   /* INTA2 (slot 4) */
    }
    if (lvl & BIT(12)) {
        st |= BIT(3);                   /* INTA3 (slot 6) */
    }
    if (s->sio_level) {
        st |= BIT(4);                   /* SIO */
    }
    if (lvl & BIT(18)) {
        st |= BIT(5);                   /* IDE */
    }
    if (lvl & BIT(1)) {
        st |= BIT(7);                   /* INTB0 */
    }

    /* Port 0x805: INTB1-3, INTC0-3, INTD0 */
    if (lvl & BIT(5)) {
        st |= BIT(8);
    }
    if (lvl & BIT(9)) {
        st |= BIT(9);
    }
    if (lvl & BIT(13)) {
        st |= BIT(10);
    }
    if (lvl & BIT(2)) {
        st |= BIT(11);
    }
    if (lvl & BIT(6)) {
        st |= BIT(12);
    }
    if (lvl & BIT(10)) {
        st |= BIT(13);
    }
    if (lvl & BIT(14)) {
        st |= BIT(14);
    }
    if (lvl & BIT(3)) {
        st |= BIT(15);
    }

    /* Port 0x806: INTD1-3 */
    if (lvl & BIT(7)) {
        st |= BIT(17);
    }
    if (lvl & BIT(11)) {
        st |= BIT(18);
    }
    if (lvl & BIT(15)) {
        st |= BIT(19);
    }

    return st;
}

static void pld_update(PyxisState *s)
{
    uint32_t st = pld_status(s);

    s->irq_level[0] = (st & ~s->pld_mask) != 0;
    pyxis_irq_update(s);
}

static uint64_t pld_read(void *opaque, hwaddr addr, unsigned size)
{
    PyxisState *s = opaque;
    uint32_t st = pld_status(s);

    return (st >> (8 * (addr & 3))) & 0xff;
}

static void pld_write(void *opaque, hwaddr addr, uint64_t val,
                      unsigned size)
{
    PyxisState *s = opaque;
    unsigned shift = 8 * (addr & 3);

    s->pld_mask &= ~(0xff << shift);
    s->pld_mask |= (val & 0xff) << shift;
    pld_update(s);
}

static const MemoryRegionOps pld_ops = {
    .read = pld_read,
    .write = pld_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

/*
 * General CSRs (HRM 4.3).
 */

static uint64_t csr_read(void *opaque, hwaddr addr, unsigned size)
{
    PyxisState *s = opaque;
    uint32_t ret;

    switch (addr) {
    case PYXIS_REV:
        ret = s->revision;
        break;
    case PCI_LAT:
        ret = s->pci_lat;
        break;
    case PYXIS_CTRL:
        ret = s->ctrl;
        break;
    case PYXIS_CTRL1:
        ret = s->ctrl1;
        break;
    case FLASH_CTRL:
        ret = s->flash_ctrl;
        break;
    case HAE_MEM:
        ret = s->hae_mem;
        break;
    case HAE_IO:
        ret = s->hae_io;
        break;
    case CFG:
        ret = s->cfg;
        break;
    case PYXIS_DIAG:
        ret = s->diag;
        break;
    case DIAG_CHECK:
        ret = s->diag_check;
        break;
    case PERF_MONITOR:
        ret = s->perf_monitor;
        break;
    case PERF_CONTROL:
        ret = s->perf_control;
        break;
    case PYXIS_ERR:
        ret = s->err;
        break;
    case PYXIS_STAT:
        ret = 0;
        break;
    case ERR_MASK:
        ret = s->err_mask;
        break;
    case PYXIS_SYN:
        ret = s->syn;
        break;
    case PYXIS_ERR_DATA:
        ret = s->err_data;
        break;
    case MEAR:
    case MESR:
    case PCI_ERR0:
    case PCI_ERR1:
    case PCI_ERR2:
        ret = 0;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: invalid offset 0x%04" HWADDR_PRIx "\n",
                      __func__, addr);
        ret = 0;
        break;
    }

    trace_pyxis_csr_read(addr, ret, size);
    return ret;
}

static void csr_write(void *opaque, hwaddr addr, uint64_t val,
                      unsigned size)
{
    PyxisState *s = opaque;

    trace_pyxis_csr_write(addr, val, size);

    switch (addr) {
    case PCI_LAT:
        s->pci_lat = val;
        break;
    case PYXIS_CTRL:
        s->ctrl = val;
        break;
    case PYXIS_CTRL1:
        s->ctrl1 = val;
        break;
    case FLASH_CTRL:
        s->flash_ctrl = val;
        break;
    case HAE_MEM:
        s->hae_mem = val;
        break;
    case HAE_IO:
        s->hae_io = val;
        break;
    case CFG:
        s->cfg = val & 3;
        break;
    case PYXIS_DIAG:
        s->diag = val;
        break;
    case DIAG_CHECK:
        s->diag_check = val;
        break;
    case PERF_MONITOR:
        s->perf_monitor = val;
        break;
    case PERF_CONTROL:
        s->perf_control = val;
        break;
    case PYXIS_ERR:
        s->err &= ~val;
        break;
    case ERR_MASK:
        s->err_mask = val;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: invalid offset 0x%04" HWADDR_PRIx "\n",
                      __func__, addr);
        break;
    }
}

static const MemoryRegionOps csr_ops = {
    .read = csr_read,
    .write = csr_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 8,
    },
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

/*
 * Memory controller CSRs (HRM 4.4): plain RW storage.  The SRM programs
 * the bank base/config/timing registers and reads them back; the actual
 * SDRAM is the plain RAM region mapped by the machine.
 */

static uint64_t mctl_read(void *opaque, hwaddr addr, unsigned size)
{
    PyxisState *s = opaque;
    uint32_t ret;

    if (addr < sizeof(s->mctl)) {
        ret = s->mctl[addr >> 2];
    } else {
        ret = 0;
    }

    trace_pyxis_mctl_read(addr, ret, size);
    return ret;
}

static void mctl_write(void *opaque, hwaddr addr, uint64_t val,
                       unsigned size)
{
    PyxisState *s = opaque;

    trace_pyxis_mctl_write(addr, val, size);
    if (addr < sizeof(s->mctl)) {
        s->mctl[addr >> 2] = val;
    }
}

static const MemoryRegionOps mctl_ops = {
    .read = mctl_read,
    .write = mctl_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 8,
    },
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

/*
 * PCI window control CSRs (HRM 4.5, 5.6).  These configure the DMA
 * target windows used by the IOMMU below.
 */

static uint64_t pa_read(void *opaque, hwaddr addr, unsigned size)
{
    PyxisPchipState *pcs = opaque;
    int n;

    for (n = 0; n < 4; n++) {
        if (addr == Wn_BASE(n)) {
            return pcs->win[n].w_base;
        }
        if (addr == Wn_MASK(n)) {
            return pcs->win[n].w_mask;
        }
        if (addr == Tn_BASE(n)) {
            return pcs->win[n].t_base;
        }
    }
    if (addr == W_DAC) {
        return pcs->w_dac;
    }
    if (addr == TBIA) {
        return 0;
    }

    /* TB_TAG/LTB_TAG/TB_PAGE: read-only scratch storage, all zero. */
    if (addr >= LTB_TAG(0) && addr <= TB_PAGE(7, 3)) {
        return 0;
    }

    qemu_log_mask(LOG_GUEST_ERROR,
                  "%s: invalid offset 0x%04" HWADDR_PRIx "\n",
                  __func__, addr);
    return 0;
}

static void pa_write(void *opaque, hwaddr addr, uint64_t val,
                     unsigned size)
{
    PyxisPchipState *pcs = opaque;
    int n;

    for (n = 0; n < 4; n++) {
        uint32_t wmask = W_BASE_ADDR_MASK | W_EN | W_SG;

        if (n == 0) {
            wmask |= W0_MEMCS_EN;
        }
        if (n == 3) {
            wmask |= W3_DAC_EN;
        }

        if (addr == Wn_BASE(n)) {
            pcs->win[n].w_base = val & wmask;
            return;
        }
        if (addr == Wn_MASK(n)) {
            pcs->win[n].w_mask = val & W_BASE_ADDR_MASK;
            return;
        }
        if (addr == Tn_BASE(n)) {
            pcs->win[n].t_base = val & 0xffffff00;
            return;
        }
    }
    if (addr == W_DAC) {
        pcs->w_dac = val & 0xff;
    }
}

static const MemoryRegionOps pa_ops = {
    .read = pa_read,
    .write = pa_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 8,
    },
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

/*
 * Miscellaneous CSRs (HRM 4.6, 5.8): clock control, clock status,
 * reset.
 */

static uint64_t misc_read(void *opaque, hwaddr addr, unsigned size)
{
    PyxisState *s = opaque;
    uint32_t ret = 0;

    switch (addr) {
    case CCR:
        ret = s->ccr;
        break;
    case CLK_STAT:
        ret = s->clk_stat;
        break;
    case RESET:
        ret = s->reset_reg;
        break;
    default:
        break;
    }
    return ret;
}

static void misc_write(void *opaque, hwaddr addr, uint64_t val,
                       unsigned size)
{
    PyxisState *s = opaque;

    switch (addr) {
    case CCR:
        s->ccr = val;
        break;
    case CLK_STAT:
        break;
    case RESET:
        s->reset_reg = val;
        break;
    default:
        break;
    }
}

static const MemoryRegionOps misc_ops = {
    .read = misc_read,
    .write = misc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 8,
    },
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

/*
 * Power management CSRs (HRM 4.8, base 0x8790000000): unimplemented
 * storage.
 */

static uint64_t pwr_read(void *opaque, hwaddr addr, unsigned size)
{
    return 0;
}

static void pwr_write(void *opaque, hwaddr addr, uint64_t val,
                      unsigned size)
{
}

static const MemoryRegionOps pwr_ops = {
    .read = pwr_read,
    .write = pwr_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 8,
    },
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

/*
 * Interrupt control CSRs (HRM 4.7, 5.9).
 */

static uint64_t irq_read(void *opaque, hwaddr addr, unsigned size)
{
    PyxisState *s = opaque;
    uint32_t req = 0;
    uint32_t ret;
    int i;

    for (i = 0; i < PYXIS_NIRQS; i++) {
        if (s->irq_level[i]) {
            req |= BIT(i);
        }
    }

    switch (addr) {
    case INT_REQ:
        ret = req;
        break;
    case INT_MASK:
        ret = s->int_mask;
        break;
    case INT_HILO:
        ret = s->int_hilo;
        break;
    case INT_ROUTE:
        ret = s->int_route;
        break;
    case GPO:
        ret = s->gpo;
        break;
    case INT_CNFG:
        ret = s->int_cnfg;
        break;
    case RT_COUNT:
        ret = (uint32_t)(qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) * 100);
        break;
    case INT_TIME:
        ret = s->int_time;
        break;
    case IIC_CTRL:
        ret = s->iic_ctrl;
        break;
    default:
        ret = 0;
        break;
    }

    trace_pyxis_irq_read(addr, ret, size);
    return ret;
}

static void irq_write(void *opaque, hwaddr addr, uint64_t val,
                      unsigned size)
{
    PyxisState *s = opaque;

    trace_pyxis_irq_write(addr, val, size);

    switch (addr) {
    case INT_REQ:
        /* Writes clear the corresponding request bits. */
        break;
    case INT_MASK:
        s->int_mask = val & 0xff;
        pyxis_irq_update(s);
        break;
    case INT_HILO:
        s->int_hilo = val & 0xff;
        break;
    case INT_ROUTE:
        s->int_route = val;
        pyxis_irq_update(s);
        break;
    case GPO:
        s->gpo = val;
        break;
    case INT_CNFG:
        s->int_cnfg = val;
        break;
    case INT_TIME:
        s->int_time = val;
        break;
    case IIC_CTRL:
        s->iic_ctrl = val;
        break;
    default:
        break;
    }
}

static const MemoryRegionOps irq_ops = {
    .read = irq_read,
    .write = irq_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 8,
    },
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

/*
 * Sparse space (HRM 6.4, 6.7, 6.8; Tables 6-6 and 6-8).
 *
 * In sparse space the 21164 encodes the PCI transaction size and byte
 * enables in addr_h<7:3>: bits <4:3> carry the transaction size
 * (00 byte, 01 word, 10 tribyte, 11 longword/quadword), bits <6:5>
 * the byte offset within the PCI longword (the PCI byte-enable lane),
 * bit <7> ad<2>, and the remaining address bits form the PCI longword
 * address ad<25:3> / ad<24:3>.  Each PCI byte therefore occupies 32
 * bytes of 21164 space.
 */

static void sparse_decode(hwaddr off, uint64_t *pci_addr,
                          unsigned *xact_size, unsigned *lane)
{
    unsigned size_code = (off >> 3) & 3;
    unsigned pos = (off >> 5) & 3;

    /*
     * HRM 6.7.2 (sparse memory) / 6.8.2 (sparse I/O): the PCI byte
     * address is formed from addr<29:8> (ad<24:3> for I/O, ad<25:3> for
     * memory) plus addr<7> (ad<2>); ad<1:0> is always zero.
     */
    *pci_addr = ((off >> 8) << 3) | (((off >> 7) & 1) << 2);
    switch (size_code) {
    case 0:                         /* byte */
        *xact_size = 1;
        break;
    case 1:                         /* word */
        *xact_size = 2;
        break;
    case 2:                         /* tribyte */
        *xact_size = 3;
        break;
    default:                        /* longword / quadword */
        *xact_size = 4;
        break;
    }
    *lane = pos;
}

static uint64_t sparse_read_common(AddressSpace *as,
                                   hwaddr off, unsigned size)
{
    uint64_t pci_addr;
    uint64_t val = 0;
    uint8_t buf[8];
    unsigned xsize, lane;
    unsigned size_code = (off >> 3) & 3;
    unsigned pos = (off >> 5) & 3;

    sparse_decode(off, &pci_addr, &xsize, &lane);

    if (size == 8 && size_code == 3 && pos == 3) {
        /*
         * Quadword: the byte offset is 11, ad<2:0> = 000, and all
         * eight byte lanes carry data (HRM Table 6-6).
         */
        address_space_read(as, pci_addr & ~(uint64_t)4,
                           MEMTXATTRS_UNSPECIFIED, buf, 8);
        return ldq_le_p(buf);
    }

    if (size_code == 2) {
        /*
         * Tribyte: byte enables 1000 (bytes 1-3) for byte offset 0,
         * or 0001 (bytes 0-2) for byte offset 1.
         */
        uint64_t lo = 0, hi = 0;
        hwaddr base = pos ? 0 : 1;

        address_space_read(as, pci_addr + base, MEMTXATTRS_UNSPECIFIED,
                           buf, 2);
        lo = lduw_le_p(buf);
        address_space_read(as, pci_addr + base + 2, MEMTXATTRS_UNSPECIFIED,
                           buf, 1);
        hi = buf[0];
        val = lo | (hi << 16);
        return val << (pos ? 0 : 8);
    }

    address_space_read(as, pci_addr | pos, MEMTXATTRS_UNSPECIFIED,
                       buf, xsize);
    switch (xsize) {
    case 1:
        val = buf[0];
        break;
    case 2:
        val = lduw_le_p(buf);
        break;
    case 4:
        val = ldl_le_p(buf);
        break;
    default:
        g_assert_not_reached();
    }
    return val << (8 * pos);
}

static void sparse_write_common(AddressSpace *as,
                                hwaddr off, uint64_t val,
                                unsigned size)
{
    uint64_t pci_addr;
    uint8_t buf[8];
    unsigned xsize, lane;
    unsigned size_code = (off >> 3) & 3;
    unsigned pos = (off >> 5) & 3;

    sparse_decode(off, &pci_addr, &xsize, &lane);

    if (size_code == 3 && pos == 3 && size == 8) {
        /* Quadword access (LDQ/STQ): ad<2:0> = 000. */
        stq_le_p(buf, val);
        address_space_write(as, pci_addr & ~(uint64_t)4,
                            MEMTXATTRS_UNSPECIFIED, buf, 8);
    } else if (size_code == 2) {
        /* Tribyte: bytes 1-3 (byte offset 0) or 0-2 (byte offset 1). */
        hwaddr base = pos ? 0 : 1;

        stw_le_p(buf, (val >> (8 * (pos ? 0 : 1))) & 0xffff);
        address_space_write(as, pci_addr + base,
                            MEMTXATTRS_UNSPECIFIED, buf, 2);
        buf[0] = (val >> (8 * (pos ? 2 : 3))) & 0xff;
        address_space_write(as, pci_addr + base + 2,
                            MEMTXATTRS_UNSPECIFIED, buf, 1);
    } else {
        switch (xsize) {
        case 1:
            buf[0] = (val >> (8 * pos)) & 0xff;
            break;
        case 2:
            stw_le_p(buf, (val >> (8 * pos)) & 0xffff);
            break;
        case 4:
            stl_le_p(buf, (val >> (8 * pos)) & 0xffffffffULL);
            break;
        default:
            g_assert_not_reached();
        }
        address_space_write(as, pci_addr | pos,
                            MEMTXATTRS_UNSPECIFIED, buf, xsize);
    }
}

static uint64_t sparse_mem_read(void *opaque, hwaddr addr,
                                unsigned size)
{
    PyxisPchipState *pcs = opaque;

    return sparse_read_common(&pcs->mem_as, addr, size);
}

static void sparse_mem_write(void *opaque, hwaddr addr, uint64_t val,
                             unsigned size)
{
    PyxisPchipState *pcs = opaque;

    sparse_write_common(&pcs->mem_as, addr, val, size);
}

static const MemoryRegionOps sparse_mem_ops = {
    .read = sparse_mem_read,
    .write = sparse_mem_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 8,
    },
    .impl = {
        .min_access_size = 4,
        .max_access_size = 8,
    },
};

static uint64_t sparse_io_read(void *opaque, hwaddr addr,
                               unsigned size)
{
    PyxisPchipState *pcs = opaque;

    return sparse_read_common(&pcs->io_as, addr, size);
}

static void sparse_io_write(void *opaque, hwaddr addr, uint64_t val,
                            unsigned size)
{
    PyxisPchipState *pcs = opaque;

    sparse_write_common(&pcs->io_as, addr, val, size);
}

static const MemoryRegionOps sparse_io_ops = {
    .read = sparse_io_read,
    .write = sparse_io_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 8,
    },
    .impl = {
        .min_access_size = 4,
        .max_access_size = 8,
    },
};

/*
 * Dummy memory region (HRM 6.1): reads return all ones, writes are
 * dropped.
 */

static uint64_t blackhole_read(void *opaque, hwaddr addr, unsigned size)
{
    return UINT64_MAX;
}

static void blackhole_write(void *opaque, hwaddr addr, uint64_t val,
                            unsigned size)
{
    trace_pyxis_blackhole_write(addr, val, size);
}

static const MemoryRegionOps blackhole_ops = {
    .read = blackhole_read,
    .write = blackhole_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
};

PCIBus *pyxis_get_pci_bus(DeviceState *dev)
{
    PyxisState *s = PYXIS_CHIPSET(dev);
    PCIHostState *phb = PCI_HOST_BRIDGE(&s->pchip);

    return phb->bus;
}

static void pyxis_external_irq_handler(void *opaque, int irq, int level)
{
    PyxisState *s = opaque;

    if (irq == 0) {
        s->sio_level = level;
        pld_update(s);
    } else if (irq < PYXIS_NIRQS) {
        s->irq_level[irq] = level;
        pyxis_irq_update(s);
    }

    if (level) {
        s->irq_counts[irq]++;
    }
    trace_pyxis_device_irq_update(irq, level);
}

static void pyxis_chipset_realize(DeviceState *dev, Error **errp)
{
    static const hwaddr csr_offsets[] = {
        PYXIS_REGION(CSR), PYXIS_REGION(MCTL), PYXIS_REGION(PA),
        PYXIS_REGION(MISC), PYXIS_REGION(PWR), PYXIS_REGION(IRQ),
    };
    static const char *csr_names[] = {
        "pyxis.csr", "pyxis.mctl", "pyxis.pa",
        "pyxis.misc", "pyxis.pwr", "pyxis.irq",
    };
    static const MemoryRegionOps *csr_group_ops[] = {
        &csr_ops, &mctl_ops, &pa_ops,
        &misc_ops, &pwr_ops, &irq_ops,
    };
    PyxisState *s = PYXIS_CHIPSET(dev);
    PyxisPchipState *pcs = &s->pchip;
    const void *csr_group_opaque[] = {
        s, s, pcs, s, s, s,
    };
    int i;

    if (unlikely(!s->ram_size)) {
        error_setg(errp, "%s: 'ram-size' property is not set", __func__);
        return;
    }

    if (unlikely(s->ram_size < PYXIS_DRAM_SIZE_MIN ||
                 s->ram_size > PYXIS_DRAM_SIZE_MAX)) {
        g_autofree char *sz = size_to_str(s->ram_size * MiB);
        g_autofree char *min_sz = size_to_str(PYXIS_DRAM_SIZE_MIN * MiB);
        g_autofree char *max_sz = size_to_str(PYXIS_DRAM_SIZE_MAX * MiB);

        error_setg(errp, "%s: unsupported DRAM size: %s", __func__, sz);
        error_append_hint(errp,
                          "DRAM size must be between %s and %s\n",
                          min_sz, max_sz);
        return;
    }

    if (unlikely(s->num_cpus != 1)) {
        error_setg(errp, "%s: unsupported cpu count (%u)",
                   __func__, s->num_cpus);
        error_append_hint(errp, "the AlphaPC 164LX is a single-CPU board\n");
        return;
    }

    /*
     * MMIO space.
     *
     * The 21174 decodes the whole 0x80.0000.0000-0xE1.0000.0000 range
     * of the 40-bit address space; present it as one sysbus region.
     */
    memory_region_init(&s->iomem, OBJECT(s), "pyxis.iomem",
                       0x6100000000ULL);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);

    /* Pchip initialization. */
    object_initialize_child(OBJECT(dev), "pchip", pcs,
                            TYPE_PYXIS_PCI_HOST_BRIDGE);
    object_property_set_link(OBJECT(pcs), "upstream", OBJECT(s),
                             &error_abort);
    if (!sysbus_realize(SYS_BUS_DEVICE(pcs), errp)) {
        return;
    }

    /* Sparse memory windows. */
    memory_region_add_subregion(&s->iomem, PYXIS_REGION(MEM_SPARSE0),
                                &pcs->reg_mem_sparse[0]);
    memory_region_add_subregion(&s->iomem, PYXIS_REGION(MEM_SPARSE1),
                                &pcs->reg_mem_sparse[1]);
    memory_region_add_subregion(&s->iomem, PYXIS_REGION(MEM_SPARSE2),
                                &pcs->reg_mem_sparse[2]);

    /* Sparse I/O windows. */
    memory_region_add_subregion(&s->iomem, PYXIS_REGION(IO_SPARSE_A),
                                &pcs->reg_io_sparse[0]);
    memory_region_add_subregion(&s->iomem, PYXIS_REGION(IO_SPARSE_B),
                                &pcs->reg_io_sparse[1]);

    /* Sparse PCI configuration and special/IACK cycles. */
    memory_region_add_subregion(&s->iomem, PYXIS_REGION(CONF_SPARSE),
                                &pcs->reg_conf);
    memory_region_add_subregion(&s->iomem, PYXIS_REGION(SPECIAL),
                                &pcs->reg_iack);

    /* Chipset CSR groups. */
    for (i = 0; i < 6; i++) {
        memory_region_init_io(&s->csr_regs[i], OBJECT(s), csr_group_ops[i],
                              (void *)csr_group_opaque[i],
                              csr_names[i], 256 * MiB);
        memory_region_add_subregion(&s->iomem, csr_offsets[i],
                                    &s->csr_regs[i]);
    }

    /*
     * Dense memory and the INT8 memory alias (HRM tables 6-1/6-2).
     *
     * reg_mem/reg_io stay at offset 0 so they can serve as AddressSpace
     * roots for the sparse windows; the CPU-facing dense windows are
     * aliases mapped at 0x8600000000/0x8900000000.
     */
    memory_region_init_alias(&pcs->reg_mem_dense, OBJECT(s),
                             "pyxis.mem-dense", &pcs->reg_mem, 0,
                             memory_region_size(&pcs->reg_mem));
    memory_region_add_subregion(&s->iomem, PYXIS_REGION(MEM_DENSE),
                                &pcs->reg_mem_dense);
    memory_region_init_alias(&s->mem_int8, OBJECT(s), "pyxis.mem-int8",
                             &pcs->reg_mem, 0,
                             memory_region_size(&pcs->reg_mem));
    memory_region_add_subregion(&s->iomem, PYXIS_REGION(MEM_INT8),
                                &s->mem_int8);

    /* INT8 I/O and configuration windows. */
    memory_region_init_alias(&pcs->reg_io_dense, OBJECT(s),
                             "pyxis.io-dense", &pcs->reg_io, 0,
                             memory_region_size(&pcs->reg_io));
    memory_region_add_subregion(&s->iomem, PYXIS_REGION(IO_DENSE),
                                &pcs->reg_io_dense);
    memory_region_add_subregion(&s->iomem, PYXIS_REGION(CONF_DENSE0),
                                &pcs->reg_conf_dense[0]);
    memory_region_add_subregion(&s->iomem, PYXIS_REGION(CONF_DENSE1),
                                &pcs->reg_conf_dense[1]);

    /* Dummy memory region (HRM 6.1). */
    memory_region_init_io(&s->dummy, OBJECT(s), &blackhole_ops, s,
                          "pyxis.dummy", 4 * GiB);
    memory_region_add_subregion(&s->iomem, PYXIS_REGION(DUMMY), &s->dummy);

    /* Initialize the CPU interfaces. */
    qdev_init_gpio_out_named(dev, &s->cpu_irq[0], "cpu-mchk", 1);
    qdev_init_gpio_out_named(dev, &s->cpu_irq[1], "cpu-device", 1);
    qdev_init_gpio_out_named(dev, &s->cpu_irq[2], "cpu-timer", 1);
    qdev_init_gpio_out_named(dev, &s->cpu_irq[3], "cpu-ipi", 1);

    /* Initialize external interfaces. */
    qdev_init_gpio_in_named(dev, pyxis_external_irq_handler, "irq",
                            PYXIS_NIRQS);
}

static void pyxis_chipset_reset(DeviceState *dev)
{
    PyxisState *s = PYXIS_CHIPSET(dev);
    int i;

    memset(s->mctl, 0, sizeof(s->mctl));
    s->pci_lat = 0;
    s->ctrl = 0;
    s->ctrl1 = 0;
    s->flash_ctrl = 0;
    s->hae_mem = 0;
    s->hae_io = 0;
    s->cfg = 0;
    s->diag = 0;
    s->diag_check = 0;
    s->perf_monitor = 0;
    s->perf_control = 0;
    s->err = 0;
    s->err_mask = 0;
    s->syn = 0;
    s->err_data = 0;
    s->int_mask = 0;
    s->int_hilo = 0;
    /* Default: all sources to irq<1>, except RTC (source 1) to irq<2>. */
    s->int_route = (0x5555 & ~(3u << 2)) | (2u << 2);
    s->gpo = 0;
    s->int_cnfg = 0;
    s->int_time = 0;
    s->iic_ctrl = 0;
    s->ccr = 0;
    s->clk_stat = 0;
    s->reset_reg = 0;
    s->pld_mask = 0;
    memset(s->irq_level, 0, sizeof(s->irq_level));
    s->sio_level = false;

    for (i = 0; i < ARRAY_SIZE(s->irq_counts); i++) {
        s->irq_counts[i] = 0;
    }
}

static bool pyxis_chipset_get_statistics(InterruptStatsProvider *obj,
                                         uint64_t **irq_counts,
                                         unsigned int *nb_irqs)
{
    PyxisState *s = PYXIS_CHIPSET(obj);

    *irq_counts = s->irq_counts;
    *nb_irqs = ARRAY_SIZE(s->irq_counts);
    return true;
}

static void pyxis_chipset_print_info(InterruptStatsProvider *obj,
                                     GString *buf)
{
    PyxisState *s = PYXIS_CHIPSET(obj);
    uint32_t req = 0;
    int i;

    for (i = 0; i < PYXIS_NIRQS; i++) {
        if (s->irq_level[i]) {
            req |= BIT(i);
        }
    }

    g_string_append_printf(buf, "pyxis: request=0x%02x\n", req);
    for (i = 0; i < PYXIS_NIRQS; i++) {
        int n = (s->int_route >> (2 * i)) & 3;

        g_string_append_printf(buf,
                               "  irq %d: level=%d route=irq<%d> "
                               "count=%" PRIu64 "\n",
                               i, s->irq_level[i], n, s->irq_counts[i]);
    }
}

static const VMStateDescription vmstate_pyxis_chipset = {
    .name = TYPE_PYXIS_CHIPSET,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(pci_lat, PyxisState),
        VMSTATE_UINT32(ctrl, PyxisState),
        VMSTATE_UINT32(ctrl1, PyxisState),
        VMSTATE_UINT32(flash_ctrl, PyxisState),
        VMSTATE_UINT32(hae_mem, PyxisState),
        VMSTATE_UINT32(hae_io, PyxisState),
        VMSTATE_UINT32(cfg, PyxisState),
        VMSTATE_UINT32(diag, PyxisState),
        VMSTATE_UINT32(diag_check, PyxisState),
        VMSTATE_UINT32(perf_monitor, PyxisState),
        VMSTATE_UINT32(perf_control, PyxisState),
        VMSTATE_UINT32(err, PyxisState),
        VMSTATE_UINT32(err_mask, PyxisState),
        VMSTATE_UINT32(int_mask, PyxisState),
        VMSTATE_UINT32(int_hilo, PyxisState),
        VMSTATE_UINT32(int_route, PyxisState),
        VMSTATE_UINT32(gpo, PyxisState),
        VMSTATE_UINT32(int_cnfg, PyxisState),
        VMSTATE_UINT32(int_time, PyxisState),
        VMSTATE_UINT32(iic_ctrl, PyxisState),
        VMSTATE_UINT32(ccr, PyxisState),
        VMSTATE_UINT32(clk_stat, PyxisState),
        VMSTATE_UINT32(reset_reg, PyxisState),
        VMSTATE_UINT32(pld_mask, PyxisState),
        VMSTATE_END_OF_LIST()
    },
};

static void pyxis_chipset_instance_init(Object *obj)
{
    if (object_resolve_path_type("", TYPE_PYXIS_CHIPSET, NULL) != NULL) {
        error_report("'%s' can't be instantiated twice!",
                     TYPE_PYXIS_CHIPSET);
        return;
    }
}

static void pyxis_chipset_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    InterruptStatsProviderClass *ispc = INTERRUPT_STATS_PROVIDER_CLASS(oc);

    dc->realize = pyxis_chipset_realize;
    /* Reason: part of the Pyxis chipset, wired up in machine code. */
    dc->user_creatable = false;
    dc->vmsd = &vmstate_pyxis_chipset;
    ispc->get_statistics = pyxis_chipset_get_statistics;
    ispc->print_info = pyxis_chipset_print_info;
    device_class_set_legacy_reset(dc, pyxis_chipset_reset);
    device_class_set_props(dc, pyxis_chipset_properties);
}

static const TypeInfo pyxis_chipset_info = {
    .name          = TYPE_PYXIS_CHIPSET,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(PyxisState),
    .instance_init = pyxis_chipset_instance_init,
    .class_init    = pyxis_chipset_class_init,
    .interfaces = (InterfaceInfo[]) {
         { TYPE_INTERRUPT_STATS_PROVIDER },
         { }
    },
};

/*
 * Board interrupt PLD (ISA ports 0x804-0x806).
 */

static const Property pld_properties[] = {
    DEFINE_PROP_LINK("upstream", PyxisPLDState, upstream,
                     TYPE_PYXIS_CHIPSET, PyxisState *),
};

static void pld_realize(DeviceState *dev, Error **errp)
{
    PyxisPLDState *s = PYXIS_PLD(dev);

    if (unlikely(!s->upstream)) {
        error_setg(errp, "%s: 'upstream' property is not set", __func__);
        return;
    }

    memory_region_init_io(&s->io, OBJECT(dev), &pld_ops, s->upstream,
                          "pyxis.pld", 4);
    isa_register_ioport(ISA_DEVICE(dev), &s->io, 0x804);
}

static void pld_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = pld_realize;
    dc->user_creatable = false;
    device_class_set_props(dc, pld_properties);
}

static const TypeInfo pld_info = {
    .name          = TYPE_PYXIS_PLD,
    .parent        = TYPE_ISA_DEVICE,
    .instance_size = sizeof(PyxisPLDState),
    .class_init    = pld_class_init,
};

/*
 * PCI IOMMU Implementation
 *
 * DMA window translation (HRM 6.12-6.15).
 */

static int pyxis_get_pte(AddressSpace *as, dma_addr_t pte_addr,
                         uint64_t *pte)
{
    int ret;

    /* TODO: guarantee 64-bit single-copy atomicity */
    ret = ldq_le_dma(as, pte_addr, pte, MEMTXATTRS_UNSPECIFIED);
    if (ret != MEMTX_OK) {
        return -EINVAL;
    }
    trace_pyxis_iommu_get_pte(pte_addr, *pte);
    return 0;
}

static int pyxis_get_pte_sg(PyxisDMAWindow *win, AddressSpace *as,
                            dma_addr_t iova, uint64_t *pte)
{
    dma_addr_t tba = ((dma_addr_t)win->t_base & 0xffffff00) << 2;
    dma_addr_t pte_addr;

    pte_addr  = tba & ~(win->w_mask >> 10);
    pte_addr |= (iova & (win->w_mask | PYXIS_SG_MASK)) >> 10;
    return pyxis_get_pte(as, pte_addr, pte);
}

static bool pyxis_window_translate(PyxisDMAWindow *win, dma_addr_t iova,
                                   PyxisIOTLBEntry *tlbe)
{
    uint32_t wmask_ext = win->w_mask | ~W_BASE_ADDR_MASK;
    dma_addr_t tba = ((dma_addr_t)win->t_base & 0xffffff00) << 2;

    /* Check for window disable. */
    if (!(win->w_base & W_EN)) {
        goto error;
    }

    /* Check for window hit: compare masked ad<31:20> with Wn_BASE. */
    if ((iova & wmask_ext) != (win->w_base & W_BASE_ADDR_MASK)) {
        goto error;
    }

    if (win->w_base & W_SG) {
        uint64_t pte;

        /* Scatter-gather translation, 8KB pages (HRM 6.14). */
        if (pyxis_get_pte_sg(win, &address_space_memory, iova, &pte)) {
            goto error;
        }
        if (!pyxis_pte_valid(pte)) {
            trace_pyxis_iommu_invalid_pte(pte);
            goto error;
        }

        tlbe->entry.iova = iova & ~PYXIS_PAGE_MASK;
        tlbe->entry.translated_addr = pyxis_pte_address(pte);
        tlbe->entry.addr_mask = PYXIS_PAGE_MASK;
        tlbe->entry.perm = IOMMU_RW;
    } else {
        /* Direct-mapped translation (HRM 6.13). */
        tlbe->entry.iova = iova & ~wmask_ext;
        tlbe->entry.translated_addr = tba & ~wmask_ext;
        tlbe->entry.addr_mask = wmask_ext;
        tlbe->entry.perm = IOMMU_RW;
    }
    return true;

error:
    return false;
}

static PyxisIOMMUStatus pyxis_iommu_walk(PyxisPchipState *pcs,
                                         dma_addr_t iova,
                                         IOMMUAccessFlags perm,
                                         PyxisIOTLBEntry *tlbe)
{
    int i;

    if (iova <= UINT32_MAX) {
        /* Single-address cycle: windows 0-2 are SAC-only. */
        for (i = 0; i < 3; i++) {
            if (pyxis_window_translate(&pcs->win[i], iova, tlbe)) {
                return IOMMU_STATUS_SUCCESS;
            }
        }

        /* Window 3 is used for SAC only when DAC is disabled. */
        if (!(pcs->win[3].w_base & W3_DAC_EN) &&
            pyxis_window_translate(&pcs->win[3], iova, tlbe)) {
            return IOMMU_STATUS_SUCCESS;
        }
    } else if ((iova & MAKE_64BIT_MASK(41, 23)) == 0 &&
               (iova & BIT_ULL(40))) {
        /*
         * Monster window (HRM 6.2): DAC cycles with ad<40> = 1 and
         * ad<63:41> = 0 map directly to memory.
         */
        tlbe->entry.iova = 0;
        tlbe->entry.translated_addr = 0;
        tlbe->entry.addr_mask = PYXIS_MWIN_MASK;
        tlbe->entry.perm = IOMMU_RW;
        return IOMMU_STATUS_SUCCESS;
    } else if ((iova & MAKE_64BIT_MASK(40, 24)) == 0) {
        /*
         * Window 3 in DAC mode: ad<63:40> must be zero and ad<39:32>
         * must match W_DAC.
         */
        if (pcs->win[3].w_base & W3_DAC_EN &&
            ((iova >> 32) & 0xff) == pcs->w_dac &&
            pyxis_window_translate(&pcs->win[3], iova & UINT32_MAX, tlbe)) {
            return IOMMU_STATUS_SUCCESS;
        }
    }

    tlbe->entry.perm = IOMMU_NONE;
    return IOMMU_STATUS_ABORT;
}

/* Entry point to the IOMMU, does everything. */
static IOMMUTLBEntry pyxis_translate_iommu(IOMMUMemoryRegion *mr,
                                           hwaddr addr,
                                           IOMMUAccessFlags flag,
                                           int iommu_idx)
{
    PyxisPchipState *pcs = container_of(mr, PyxisPchipState, iommu);
    PyxisIOTLBEntry tlbe = {};
    IOMMUTLBEntry entry = {
        .target_as = &address_space_memory,
        .iova = addr,
        .translated_addr = addr,
        .addr_mask = ~(hwaddr)0,
        .perm = IOMMU_NONE,
    };

    switch (pyxis_iommu_walk(pcs, addr, flag, &tlbe)) {
    case IOMMU_STATUS_ABORT:
        trace_pyxis_iommu_translate_abort(mr->parent_obj.name, addr);
        break;
    case IOMMU_STATUS_SUCCESS:
        entry.perm = tlbe.entry.perm;
        entry.addr_mask = tlbe.entry.addr_mask;
        entry.translated_addr = pyxis_iotlb_translate(&tlbe, addr);
        trace_pyxis_iommu_translate_success(mr->parent_obj.name, entry.iova,
                                            entry.translated_addr);
        break;
    default:
        g_assert_not_reached();
    }
    return entry;
}

static AddressSpace *pyxis_pci_dma_iommu_as(PCIBus *bus, void *opaque,
                                            int devfn)
{
    PyxisPchipState *pcs = opaque;

    return &pcs->iommu_as;
}

static void pyxis_iommu_memory_region_class_init(ObjectClass *klass,
                                                 const void *data)
{
    IOMMUMemoryRegionClass *imrc = IOMMU_MEMORY_REGION_CLASS(klass);

    imrc->translate = pyxis_translate_iommu;
}

static const PCIIOMMUOps pyxis_pci_iommu_ops = {
    .get_address_space = pyxis_pci_dma_iommu_as,
};

static const TypeInfo pyxis_iommu_memory_region_info = {
    .parent     = TYPE_IOMMU_MEMORY_REGION,
    .name       = TYPE_PYXIS_IOMMU_MEMORY_REGION,
    .class_init = pyxis_iommu_memory_region_class_init,
};

/*
 * PCI Host Bridge Implementation
 */

static const Property pyxis_pcihost_properties[] = {
    DEFINE_PROP_LINK("upstream", PyxisPchipState, upstream,
                     TYPE_PYXIS_CHIPSET, PyxisState *),
};

static void pchip_reset(PyxisPchipState *pcs)
{
    int i;

    for (i = 0; i < 4; i++) {
        pcs->win[i].w_base = 0;
        pcs->win[i].w_mask = 0;
        pcs->win[i].t_base = 0;
    }
    pcs->w_dac = 0;
    pcs->pci_irq_level = 0;
}

/*
 * PCI configuration space (HRM 6.9).
 *
 * Sparse config:  cpu addr<28:7> maps to ad<23:2> (offset >> 5), and
 * the transaction size/lane bits are ignored (firmware uses longword
 * config cycles).  Dense config (INT8, byte/word mode): the window
 * offset is the standard PCI config address directly.
 */

static uint64_t pci_conf_sparse_read(void *opaque, hwaddr addr,
                                     unsigned size)
{
    PyxisPchipState *pcs = opaque;
    PCIHostState *phb = PCI_HOST_BRIDGE(pcs);
    uint64_t pci_addr;
    uint64_t ret;

    pci_addr = (addr >> 5) & 0x3fffffc;
    ret = pci_data_read(phb->bus, pci_addr, size);
    return ret;
}

static void pci_conf_sparse_write(void *opaque, hwaddr addr, uint64_t val,
                                  unsigned size)
{
    PyxisPchipState *pcs = opaque;
    PCIHostState *phb = PCI_HOST_BRIDGE(pcs);
    uint64_t pci_addr;

    pci_addr = (addr >> 5) & 0x3fffffc;
    pci_data_write(phb->bus, pci_addr, val, size);
}

static const MemoryRegionOps pci_conf_sparse_ops = {
    .read = pci_conf_sparse_read,
    .write = pci_conf_sparse_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static uint64_t pci_conf_dense_read(void *opaque, hwaddr addr,
                                    unsigned size)
{
    PyxisPchipState *pcs = opaque;
    PCIHostState *phb = PCI_HOST_BRIDGE(pcs);

    return pci_data_read(phb->bus, addr, size);
}

static void pci_conf_dense_write(void *opaque, hwaddr addr, uint64_t val,
                                 unsigned size)
{
    PyxisPchipState *pcs = opaque;
    PCIHostState *phb = PCI_HOST_BRIDGE(pcs);

    pci_data_write(phb->bus, addr, val, size);
}

static const MemoryRegionOps pci_conf_dense_ops = {
    .read = pci_conf_dense_read,
    .write = pci_conf_dense_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

/* PCI special/interrupt acknowledge space. */
static uint64_t iack_read(void *opaque, hwaddr addr, unsigned size)
{
    return pic_read_irq(isa_pic);
}

static void iack_write(void *opaque, hwaddr addr, uint64_t val,
                       unsigned size)
{
}

static const MemoryRegionOps iack_ops = {
    .read = iack_read,
    .write = iack_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static int pyxis_pcihost_map_irq(PCIDevice *d, int irq_num)
{
    int slot = PCI_SLOT(d->devfn);

    /*
     * IDSEL mapping on the AlphaPC 164LX (TRM fig 4-3):
     *   slot 2 -> ad16, slot 0 -> ad17, slot 1 -> ad18,
     *   SIO (82378ZB) -> ad19, slot 3 -> ad20, reserved -> ad21,
     *   IDE (PCI0646) -> ad22.
     */
    switch (slot) {
    case 2:
    case 3:
    case 4:
    case 6:
        return (slot - 2) * 4 + (irq_num - 1);
    case 5:                     /* SIO */
        return 16;
    case 7:
        return 17;
    case 8:                     /* IDE */
        return 18;
    default:
        return 24 + slot;
    }
}

static void pyxis_pcihost_set_irq(void *opaque, int n, int level)
{
    PyxisPchipState *pcs = opaque;

    trace_pyxis_pcihost_set_irq(n, level);
    if (level) {
        pcs->pci_irq_level |= BIT(n);
    } else {
        pcs->pci_irq_level &= ~BIT(n);
    }
    pld_update(pcs->upstream);
}

static void pyxis_pcihost_reset(DeviceState *dev)
{
    PyxisPchipState *pcs = PYXIS_PCI_HOST_BRIDGE(dev);

    pchip_reset(pcs);
}

static int pyxis_pcihost_hose_number(PCIHostState *host_bridge)
{
    return 0;
}

static const char *pyxis_pcihost_root_bus_path(PCIHostState *host_bridge,
                                               PCIBus *rootbus)
{
    return "0000:00";
}

static void pyxis_pcihost_realize(DeviceState *dev, Error **errp)
{
    PyxisPchipState *s = PYXIS_PCI_HOST_BRIDGE(dev);
    PCIHostState *phb = PCI_HOST_BRIDGE(dev);

    if (unlikely(!s->upstream)) {
        error_setg(errp, "%s: 'upstream' property is not set", __func__);
        return;
    }

    memory_region_init_iommu(&s->iommu, sizeof(s->iommu),
                             TYPE_PYXIS_IOMMU_MEMORY_REGION, OBJECT(s),
                             "pyxis.iommu", 16 * TiB);
    address_space_init(&s->iommu_as, MEMORY_REGION(&s->iommu), "pci.0");

    memory_region_init_io(&s->reg_iack, OBJECT(s), &iack_ops, s,
                          "pyxis.iack", 512 * MiB);
    memory_region_init_io(&s->reg_conf, OBJECT(s), &pci_conf_sparse_ops,
                          s, "pyxis.conf-sparse", 512 * MiB);
    memory_region_init_io(&s->reg_conf_dense[0], OBJECT(s),
                          &pci_conf_dense_ops, s, "pyxis.conf-dense0",
                          4 * GiB);
    memory_region_init_io(&s->reg_conf_dense[1], OBJECT(s),
                          &pci_conf_dense_ops, s, "pyxis.conf-dense1",
                          4 * GiB);
    memory_region_init(&s->reg_mem, OBJECT(s), "pyxis.pci-mem", 4 * GiB);
    memory_region_init(&s->reg_io, OBJECT(s), "pyxis.pci-io", 4 * GiB);
    address_space_init(&s->mem_as, &s->reg_mem, "pyxis.pci-mem");
    address_space_init(&s->io_as, &s->reg_io, "pyxis.pci-io");

    memory_region_init_io(&s->reg_mem_sparse[0], OBJECT(s),
                          &sparse_mem_ops, s, "pyxis.mem-sparse0",
                          16 * GiB);
    memory_region_init_io(&s->reg_mem_sparse[1], OBJECT(s),
                          &sparse_mem_ops, s, "pyxis.mem-sparse1",
                          4 * GiB);
    memory_region_init_io(&s->reg_mem_sparse[2], OBJECT(s),
                          &sparse_mem_ops, s, "pyxis.mem-sparse2",
                          2 * GiB);
    memory_region_init_io(&s->reg_io_sparse[0], OBJECT(s),
                          &sparse_io_ops, s, "pyxis.io-sparseA",
                          1 * GiB);
    memory_region_init_io(&s->reg_io_sparse[1], OBJECT(s),
                          &sparse_io_ops, s, "pyxis.io-sparseB",
                          1 * GiB);

    phb->bus = pci_register_root_bus(dev, "pci.0",
                                     pyxis_pcihost_set_irq,
                                     pyxis_pcihost_map_irq, s,
                                     &s->reg_mem, &s->reg_io,
                                     PCI_DEVFN(2, 0), 32, TYPE_PCI_BUS);
    pci_setup_iommu(phb->bus, &pyxis_pci_iommu_ops, s);
}

static const VMStateDescription vmstate_pyxis_pcihost_window = {
    .name = "pyxis-dma-window",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(w_base, PyxisDMAWindow),
        VMSTATE_UINT32(w_mask, PyxisDMAWindow),
        VMSTATE_UINT32(t_base, PyxisDMAWindow),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_pyxis_pcihost = {
    .name = TYPE_PYXIS_PCI_HOST_BRIDGE,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_STRUCT_ARRAY(win, PyxisPchipState, 4, 1,
                             vmstate_pyxis_pcihost_window, PyxisDMAWindow),
        VMSTATE_UINT32(w_dac, PyxisPchipState),
        VMSTATE_UINT32(pci_irq_level, PyxisPchipState),
        VMSTATE_END_OF_LIST()
    },
};

static void pyxis_pcihost_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIHostBridgeClass *hc = PCI_HOST_BRIDGE_CLASS(klass);

    hc->hose_number = pyxis_pcihost_hose_number;
    hc->root_bus_path = pyxis_pcihost_root_bus_path;
    dc->realize = pyxis_pcihost_realize;
    /* Reason: Must be wired up in code (sysbus MRs and IRQ) */
    dc->user_creatable = false;
    dc->vmsd = &vmstate_pyxis_pcihost;
    dc->fw_name = "pci";
    device_class_set_legacy_reset(dc, pyxis_pcihost_reset);
    device_class_set_props(dc, pyxis_pcihost_properties);
    set_bit(DEVICE_CATEGORY_BRIDGE, dc->categories);
}

static const TypeInfo pyxis_pcihost_info = {
    .name          = TYPE_PYXIS_PCI_HOST_BRIDGE,
    .parent        = TYPE_PCI_HOST_BRIDGE,
    .instance_size = sizeof(PyxisPchipState),
    .class_init    = pyxis_pcihost_class_init,
};

static void pyxis_register_types(void)
{
    type_register_static(&pyxis_chipset_info);
    type_register_static(&pyxis_pcihost_info);
    type_register_static(&pyxis_iommu_memory_region_info);
    type_register_static(&pld_info);
}

type_init(pyxis_register_types)
