/*
 *  m68k op helpers
 *
 *  Copyright (c) 2006-2007 CodeSourcery
 *  Written by Paul Brook
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu/osdep.h"
#include "cpu.h"
#include "exec/cputlb.h"
#include "exec/page-protection.h"
#include "exec/target_page.h"
#include "exec/gdbstub.h"
#include "exec/helper-proto.h"
#include "accel/tcg/cpu-loop.h"
#include "hw/core/irq.h"
#include "qemu/main-loop.h"
#include "system/memory.h"
#include "gdbstub/helpers.h"
#include "fpu/softfloat.h"
#include "qemu/qemu-print.h"
#include "qemu/log.h"

#define SIGNBIT (1u << 31)

static int cf_fpu_gdb_get_reg(CPUState *cs, GByteArray *mem_buf, int n)
{
    M68kCPU *cpu = M68K_CPU(cs);
    CPUM68KState *env = &cpu->env;

    if (n < 8) {
        /* Use scratch float_status so any exceptions don't change CPU state */
        float_status s = env->fp_status;
        return gdb_get_reg64(mem_buf, floatx80_to_float64(env->fregs[n].d, &s));
    }
    switch (n) {
    case 8: /* fpcontrol */
        return gdb_get_reg32(mem_buf, env->fpcr);
    case 9: /* fpstatus */
        return gdb_get_reg32(mem_buf, env->fpsr);
    case 10: /* fpiar */
        return gdb_get_reg32(mem_buf, env->fpiar);
    }
    return 0;
}

static int cf_fpu_gdb_set_reg(CPUState *cs, uint8_t *mem_buf, int n)
{
    M68kCPU *cpu = M68K_CPU(cs);
    CPUM68KState *env = &cpu->env;

    if (n < 8) {
        /* Use scratch float_status so any exceptions don't change CPU state */
        float_status s = env->fp_status;
        env->fregs[n].d = float64_to_floatx80(ldq_be_p(mem_buf), &s);
        return 8;
    }
    switch (n) {
    case 8: /* fpcontrol */
        cpu_m68k_set_fpcr(env, ldl_be_p(mem_buf));
        return 4;
    case 9: /* fpstatus */
        env->fpsr = ldl_be_p(mem_buf);
        return 4;
    case 10: /* fpiar */
        env->fpiar = ldl_be_p(mem_buf);
        return 4;
    }
    return 0;
}

static int m68k_fpu_gdb_get_reg(CPUState *cs, GByteArray *mem_buf, int n)
{
    M68kCPU *cpu = M68K_CPU(cs);
    CPUM68KState *env = &cpu->env;

    if (n < 8) {
        int len = gdb_get_reg16(mem_buf, env->fregs[n].l.upper);
        len += gdb_get_reg16(mem_buf, 0);
        len += gdb_get_reg64(mem_buf, env->fregs[n].l.lower);
        return len;
    }
    switch (n) {
    case 8: /* fpcontrol */
        return gdb_get_reg32(mem_buf, env->fpcr);
    case 9: /* fpstatus */
        return gdb_get_reg32(mem_buf, cpu_m68k_get_fpsr(env));
    case 10: /* fpiar */
        return gdb_get_reg32(mem_buf, env->fpiar);
    }
    return 0;
}

static int m68k_fpu_gdb_set_reg(CPUState *cs, uint8_t *mem_buf, int n)
{
    M68kCPU *cpu = M68K_CPU(cs);
    CPUM68KState *env = &cpu->env;

    if (n < 8) {
        env->fregs[n].l.upper = lduw_be_p(mem_buf);
        env->fregs[n].l.lower = ldq_be_p(mem_buf + 4);
        return 12;
    }
    switch (n) {
    case 8: /* fpcontrol */
        cpu_m68k_set_fpcr(env, ldl_be_p(mem_buf));
        return 4;
    case 9: /* fpstatus */
        cpu_m68k_set_fpsr(env, ldl_be_p(mem_buf));
        return 4;
    case 10: /* fpiar */
        env->fpiar = ldl_be_p(mem_buf);
        return 4;
    }
    return 0;
}

void m68k_cpu_init_gdb(M68kCPU *cpu)
{
    CPUState *cs = CPU(cpu);
    CPUM68KState *env = &cpu->env;

    if (m68k_feature(env, M68K_FEATURE_CF_FPU)) {
        gdb_register_coprocessor(cs, cf_fpu_gdb_get_reg, cf_fpu_gdb_set_reg,
                                 gdb_find_static_feature("cf-fp.xml"));
    } else if (m68k_feature(env, M68K_FEATURE_FPU)) {
        gdb_register_coprocessor(cs, m68k_fpu_gdb_get_reg, m68k_fpu_gdb_set_reg,
                                 gdb_find_static_feature("m68k-fp.xml"));
    }
    /* TODO: Add [E]MAC registers.  */
}

void HELPER(cf_movec_to)(CPUM68KState *env, uint32_t reg, uint32_t val)
{
    switch (reg) {
    case M68K_CR_CACR:
        env->cacr = val;
        m68k_switch_sp(env);
        break;
    case M68K_CR_ACR0:
    case M68K_CR_ACR1:
    case M68K_CR_ACR2:
    case M68K_CR_ACR3:
        /* TODO: Implement Access Control Registers.  */
        break;
    case M68K_CR_VBR:
        env->vbr = val;
        break;
    /* TODO: Implement control registers.  */
    default:
        cpu_abort(env_cpu(env),
                  "Unimplemented control register write 0x%x = 0x%x\n",
                  reg, val);
    }
}

static void raise_exception_ra(CPUM68KState *env, int tt, uintptr_t raddr)
{
    CPUState *cs = env_cpu(env);

    cs->exception_index = tt;
    cpu_loop_exit_restore(cs, raddr);
}

void HELPER(m68k_movec_to)(CPUM68KState *env, uint32_t reg, uint32_t val)
{
    switch (reg) {
    /* MC680[12346]0 */
    case M68K_CR_SFC:
        env->sfc = val & 7;
        return;
    /* MC680[12346]0 */
    case M68K_CR_DFC:
        env->dfc = val & 7;
        return;
    /* MC680[12346]0 */
    case M68K_CR_VBR:
        env->vbr = val;
        return;
    /* MC680[2346]0 */
    case M68K_CR_CACR:
        if (m68k_feature(env, M68K_FEATURE_M68020)) {
            env->cacr = val & 0x0000000f;
        } else if (m68k_feature(env, M68K_FEATURE_M68030)) {
            env->cacr = val & 0x00003f1f;
        } else if (m68k_feature(env, M68K_FEATURE_M68040)) {
            env->cacr = val & 0x80008000;
        } else if (m68k_feature(env, M68K_FEATURE_M68060)) {
            env->cacr = val & 0xf8e0e000;
        } else {
            break;
        }
        m68k_switch_sp(env);
        return;
    /* MC680[46]0 */
    case M68K_CR_TC:
        if (m68k_feature(env, M68K_FEATURE_M68040)
         || m68k_feature(env, M68K_FEATURE_M68060)) {
            env->mmu.tcr = val;
            return;
        }
        break;
    /* MC68040 */
    case M68K_CR_MMUSR:
        if (m68k_feature(env, M68K_FEATURE_M68040)) {
            env->mmu.mmusr = val;
            return;
        }
        break;
    /* MC680[46]0 */
    case M68K_CR_SRP:
        if (m68k_feature(env, M68K_FEATURE_M68040)
         || m68k_feature(env, M68K_FEATURE_M68060)) {
            env->mmu.srp = val;
            return;
        }
        break;
    /* MC680[46]0 */
    case M68K_CR_URP:
        if (m68k_feature(env, M68K_FEATURE_M68040)
         || m68k_feature(env, M68K_FEATURE_M68060)) {
            env->mmu.urp = val;
            return;
        }
        break;
    /* MC680[12346]0 */
    case M68K_CR_USP:
        env->sp[M68K_USP] = val;
        return;
    /* MC680[234]0 */
    case M68K_CR_MSP:
        if (m68k_feature(env, M68K_FEATURE_M68020)
         || m68k_feature(env, M68K_FEATURE_M68030)
         || m68k_feature(env, M68K_FEATURE_M68040)) {
            env->sp[M68K_SSP] = val;
            return;
        }
        break;
    /* MC680[234]0 */
    case M68K_CR_ISP:
        if (m68k_feature(env, M68K_FEATURE_M68020)
         || m68k_feature(env, M68K_FEATURE_M68030)
         || m68k_feature(env, M68K_FEATURE_M68040)) {
            env->sp[M68K_ISP] = val;
            return;
        }
        break;
    /* MC68040/MC68LC040 */
    case M68K_CR_ITT0: /* MC68EC040 only: M68K_CR_IACR0 */
        if (m68k_feature(env, M68K_FEATURE_M68040)) {
            env->mmu.ttr[M68K_ITTR0] = val;
            return;
        }
        break;
    /* MC68040/MC68LC040 */
    case M68K_CR_ITT1: /* MC68EC040 only: M68K_CR_IACR1 */
        if (m68k_feature(env, M68K_FEATURE_M68040)) {
            env->mmu.ttr[M68K_ITTR1] = val;
            return;
        }
        break;
    /* MC68040/MC68LC040 */
    case M68K_CR_DTT0: /* MC68EC040 only: M68K_CR_DACR0 */
        if (m68k_feature(env, M68K_FEATURE_M68040)) {
            env->mmu.ttr[M68K_DTTR0] = val;
            return;
        }
        break;
    /* MC68040/MC68LC040 */
    case M68K_CR_DTT1: /* MC68EC040 only: M68K_CR_DACR1 */
        if (m68k_feature(env, M68K_FEATURE_M68040)) {
            env->mmu.ttr[M68K_DTTR1] = val;
            return;
        }
        break;
    /* Unimplemented Registers */
    case M68K_CR_CAAR:
    /* MC68060 only, unimplemented */
    case M68K_CR_PCR:
    case M68K_CR_BUSCR:
        if (m68k_feature(env, M68K_FEATURE_M68060)) {
            cpu_abort(env_cpu(env),
                      "Unimplemented control register write 0x%x = 0x%x\n",
                      reg, val);
        }
        break;
    }

    /* Invalid control registers will generate an exception. */
    raise_exception_ra(env, EXCP_ILLEGAL, 0);
}

uint32_t HELPER(m68k_movec_from)(CPUM68KState *env, uint32_t reg)
{
    switch (reg) {
    /* MC680[12346]0 */
    case M68K_CR_SFC:
        return env->sfc;
    /* MC680[12346]0 */
    case M68K_CR_DFC:
        return env->dfc;
    /* MC680[12346]0 */
    case M68K_CR_VBR:
        return env->vbr;
    /* MC680[2346]0 */
    case M68K_CR_CACR:
        if (m68k_feature(env, M68K_FEATURE_M68020)
         || m68k_feature(env, M68K_FEATURE_M68030)
         || m68k_feature(env, M68K_FEATURE_M68040)
         || m68k_feature(env, M68K_FEATURE_M68060)) {
            return env->cacr;
        }
        break;
    /* MC680[46]0 */
    case M68K_CR_TC:
        if (m68k_feature(env, M68K_FEATURE_M68040)
         || m68k_feature(env, M68K_FEATURE_M68060)) {
            return env->mmu.tcr;
        }
        break;
    /* MC68040 */
    case M68K_CR_MMUSR:
        if (m68k_feature(env, M68K_FEATURE_M68040)) {
            return env->mmu.mmusr;
        }
        break;
    /* MC680[46]0 */
    case M68K_CR_SRP:
        if (m68k_feature(env, M68K_FEATURE_M68040)
         || m68k_feature(env, M68K_FEATURE_M68060)) {
            return env->mmu.srp;
        }
        break;
    /* MC68040/MC68LC040 */
    case M68K_CR_URP:
        if (m68k_feature(env, M68K_FEATURE_M68040)
         || m68k_feature(env, M68K_FEATURE_M68060)) {
            return env->mmu.urp;
        }
        break;
    /* MC680[46]0 */
    case M68K_CR_USP:
        return env->sp[M68K_USP];
    /* MC680[234]0 */
    case M68K_CR_MSP:
        if (m68k_feature(env, M68K_FEATURE_M68020)
         || m68k_feature(env, M68K_FEATURE_M68030)
         || m68k_feature(env, M68K_FEATURE_M68040)) {
            return env->sp[M68K_SSP];
        }
        break;
    /* MC680[234]0 */
    case M68K_CR_ISP:
        if (m68k_feature(env, M68K_FEATURE_M68020)
         || m68k_feature(env, M68K_FEATURE_M68030)
         || m68k_feature(env, M68K_FEATURE_M68040)) {
            return env->sp[M68K_ISP];
        }
        break;
    /* MC68040/MC68LC040 */
    case M68K_CR_ITT0: /* MC68EC040 only: M68K_CR_IACR0 */
        if (m68k_feature(env, M68K_FEATURE_M68040)) {
            return env->mmu.ttr[M68K_ITTR0];
        }
        break;
    /* MC68040/MC68LC040 */
    case M68K_CR_ITT1: /* MC68EC040 only: M68K_CR_IACR1 */
        if (m68k_feature(env, M68K_FEATURE_M68040)) {
            return env->mmu.ttr[M68K_ITTR1];
        }
        break;
    /* MC68040/MC68LC040 */
    case M68K_CR_DTT0: /* MC68EC040 only: M68K_CR_DACR0 */
        if (m68k_feature(env, M68K_FEATURE_M68040)) {
            return env->mmu.ttr[M68K_DTTR0];
        }
        break;
    /* MC68040/MC68LC040 */
    case M68K_CR_DTT1: /* MC68EC040 only: M68K_CR_DACR1 */
        if (m68k_feature(env, M68K_FEATURE_M68040)) {
            return env->mmu.ttr[M68K_DTTR1];
        }
        break;
    /* Unimplemented Registers */
    case M68K_CR_CAAR:
    /* MC68060 only, unimplemented */
    case M68K_CR_PCR:
    case M68K_CR_BUSCR:
        if (m68k_feature(env, M68K_FEATURE_M68060)) {
            cpu_abort(env_cpu(env),
                      "Unimplemented control register read 0x%x\n", reg);
        }
        break;
    }

    /* Invalid control registers will generate an exception. */
    raise_exception_ra(env, EXCP_ILLEGAL, 0);

    return 0;
}

void HELPER(set_macsr)(CPUM68KState *env, uint32_t val)
{
    uint32_t acc;
    int8_t exthigh;
    uint8_t extlow;
    uint64_t regval;
    int i;
    if ((env->macsr ^ val) & (MACSR_FI | MACSR_SU)) {
        for (i = 0; i < 4; i++) {
            regval = env->macc[i];
            exthigh = regval >> 40;
            if (env->macsr & MACSR_FI) {
                acc = regval >> 8;
                extlow = regval;
            } else {
                acc = regval;
                extlow = regval >> 32;
            }
            if (env->macsr & MACSR_FI) {
                regval = (((uint64_t)acc) << 8) | extlow;
                regval |= ((int64_t)exthigh) << 40;
            } else if (env->macsr & MACSR_SU) {
                regval = acc | (((int64_t)extlow) << 32);
                regval |= ((int64_t)exthigh) << 40;
            } else {
                regval = acc | (((uint64_t)extlow) << 32);
                regval |= ((uint64_t)(uint8_t)exthigh) << 40;
            }
            env->macc[i] = regval;
        }
    }
    env->macsr = val;
}

void m68k_switch_sp(CPUM68KState *env)
{
    int new_sp;

    env->sp[env->current_sp] = env->aregs[7];
    if (m68k_feature(env, M68K_FEATURE_M68K)) {
        if (env->sr & SR_S) {
            /* SR:Master-Mode bit unimplemented then ISP is not available */
            if (!m68k_feature(env, M68K_FEATURE_MSP) || env->sr & SR_M) {
                new_sp = M68K_SSP;
            } else {
                new_sp = M68K_ISP;
            }
        } else {
            new_sp = M68K_USP;
        }
    } else {
        new_sp = (env->sr & SR_S && env->cacr & M68K_CACR_EUSP)
                 ? M68K_SSP : M68K_USP;
    }
    env->aregs[7] = env->sp[new_sp];
    env->current_sp = new_sp;
}

#if !defined(CONFIG_USER_ONLY)
/* MMU: 68040 only */

static void print_address_zone(uint32_t logical, uint32_t physical,
                               uint32_t size, int attr)
{
    qemu_printf("%08x - %08x -> %08x - %08x %c ",
                logical, logical + size - 1,
                physical, physical + size - 1,
                attr & 4 ? 'W' : '-');
    size >>= 10;
    if (size < 1024) {
        qemu_printf("(%d KiB)\n", size);
    } else {
        size >>= 10;
        if (size < 1024) {
            qemu_printf("(%d MiB)\n", size);
        } else {
            size >>= 10;
            qemu_printf("(%d GiB)\n", size);
        }
    }
}

static void dump_address_map(CPUM68KState *env, uint32_t root_pointer)
{
    int tic_size, tic_shift;
    uint32_t tib_mask;
    uint32_t tia, tib, tic;
    uint32_t logical = 0xffffffff, physical = 0xffffffff;
    uint32_t first_logical = 0xffffffff, first_physical = 0xffffffff;
    uint32_t last_logical, last_physical;
    int32_t size;
    int last_attr = -1, attr = -1;
    CPUState *cs = env_cpu(env);
    MemTxResult txres;

    if (env->mmu.tcr & M68K_TCR_PAGE_8K) {
        /* 8k page */
        tic_size = 32;
        tic_shift = 13;
        tib_mask = M68K_8K_PAGE_MASK;
    } else {
        /* 4k page */
        tic_size = 64;
        tic_shift = 12;
        tib_mask = M68K_4K_PAGE_MASK;
    }
    for (unsigned i = 0; i < M68K_ROOT_POINTER_ENTRIES; i++) {
        tia = address_space_ldl(cs->as, M68K_POINTER_BASE(root_pointer) + i * 4,
                                MEMTXATTRS_UNSPECIFIED, &txres);
        if (txres != MEMTX_OK || !M68K_UDT_VALID(tia)) {
            continue;
        }
        for (unsigned j = 0; j < M68K_ROOT_POINTER_ENTRIES; j++) {
            tib = address_space_ldl(cs->as, M68K_POINTER_BASE(tia) + j * 4,
                                    MEMTXATTRS_UNSPECIFIED, &txres);
            if (txres != MEMTX_OK || !M68K_UDT_VALID(tib)) {
                continue;
            }
            for (unsigned k = 0; k < tic_size; k++) {
                tic = address_space_ldl(cs->as, (tib & tib_mask) + k * 4,
                                        MEMTXATTRS_UNSPECIFIED, &txres);
                if (txres != MEMTX_OK || !M68K_PDT_VALID(tic)) {
                    continue;
                }
                if (M68K_PDT_INDIRECT(tic)) {
                    tic = address_space_ldl(cs->as, M68K_INDIRECT_POINTER(tic),
                                            MEMTXATTRS_UNSPECIFIED, &txres);
                    if (txres != MEMTX_OK) {
                        continue;
                    }
                }

                last_logical = logical;
                logical = (i << M68K_TTS_ROOT_SHIFT) |
                          (j << M68K_TTS_POINTER_SHIFT) |
                          (k << tic_shift);

                last_physical = physical;
                physical = tic & ~((1 << tic_shift) - 1);

                last_attr = attr;
                attr = tic & ((1 << tic_shift) - 1);

                if ((logical != (last_logical + (1 << tic_shift))) ||
                    (physical != (last_physical + (1 << tic_shift))) ||
                    (attr & 4) != (last_attr & 4)) {

                    if (first_logical != 0xffffffff) {
                        size = last_logical + (1 << tic_shift) -
                               first_logical;
                        print_address_zone(first_logical,
                                           first_physical, size, last_attr);
                    }
                    first_logical = logical;
                    first_physical = physical;
                }
            }
        }
    }
    if (first_logical != logical || (attr & 4) != (last_attr & 4)) {
        size = logical + (1 << tic_shift) - first_logical;
        print_address_zone(first_logical, first_physical, size, last_attr);
    }
}

#define DUMP_CACHEFLAGS(a) \
    switch (a & M68K_DESC_CACHEMODE) { \
    case M68K_DESC_CM_WRTHRU: /* cacheable, write-through */ \
        qemu_printf("T"); \
        break; \
    case M68K_DESC_CM_COPYBK: /* cacheable, copyback */ \
        qemu_printf("C"); \
        break; \
    case M68K_DESC_CM_SERIAL: /* noncachable, serialized */ \
        qemu_printf("S"); \
        break; \
    case M68K_DESC_CM_NCACHE: /* noncachable */ \
        qemu_printf("N"); \
        break; \
    }

static void dump_ttr(uint32_t ttr)
{
    if ((ttr & M68K_TTR_ENABLED) == 0) {
        qemu_printf("disabled\n");
        return;
    }
    qemu_printf("Base: 0x%08x Mask: 0x%08x Control: ",
                ttr & M68K_TTR_ADDR_BASE,
                (ttr & M68K_TTR_ADDR_MASK) << M68K_TTR_ADDR_MASK_SHIFT);
    switch (ttr & M68K_TTR_SFIELD) {
    case M68K_TTR_SFIELD_USER:
        qemu_printf("U");
        break;
    case M68K_TTR_SFIELD_SUPER:
        qemu_printf("S");
        break;
    default:
        qemu_printf("*");
        break;
    }
    DUMP_CACHEFLAGS(ttr);
    if (ttr & M68K_DESC_WRITEPROT) {
        qemu_printf("R");
    } else {
        qemu_printf("W");
    }
    qemu_printf(" U: %d\n", (ttr & M68K_DESC_USERATTR) >>
                               M68K_DESC_USERATTR_SHIFT);
}

void dump_mmu(CPUM68KState *env)
{
    if ((env->mmu.tcr & M68K_TCR_ENABLED) == 0) {
        qemu_printf("Translation disabled\n");
        return;
    }
    qemu_printf("Page Size: ");
    if (env->mmu.tcr & M68K_TCR_PAGE_8K) {
        qemu_printf("8kB\n");
    } else {
        qemu_printf("4kB\n");
    }

    qemu_printf("MMUSR: ");
    if (env->mmu.mmusr & M68K_MMU_B_040) {
        qemu_printf("BUS ERROR\n");
    } else {
        qemu_printf("Phy=%08x Flags: ", env->mmu.mmusr & 0xfffff000);
        /* flags found on the page descriptor */
        if (env->mmu.mmusr & M68K_MMU_G_040) {
            qemu_printf("G"); /* Global */
        } else {
            qemu_printf(".");
        }
        if (env->mmu.mmusr & M68K_MMU_S_040) {
            qemu_printf("S"); /* Supervisor */
        } else {
            qemu_printf(".");
        }
        if (env->mmu.mmusr & M68K_MMU_M_040) {
            qemu_printf("M"); /* Modified */
        } else {
            qemu_printf(".");
        }
        if (env->mmu.mmusr & M68K_MMU_WP_040) {
            qemu_printf("W"); /* Write protect */
        } else {
            qemu_printf(".");
        }
        if (env->mmu.mmusr & M68K_MMU_T_040) {
            qemu_printf("T"); /* Transparent */
        } else {
            qemu_printf(".");
        }
        if (env->mmu.mmusr & M68K_MMU_R_040) {
            qemu_printf("R"); /* Resident */
        } else {
            qemu_printf(".");
        }
        qemu_printf(" Cache: ");
        DUMP_CACHEFLAGS(env->mmu.mmusr);
        qemu_printf(" U: %d\n", (env->mmu.mmusr >> 8) & 3);
        qemu_printf("\n");
    }

    qemu_printf("ITTR0: ");
    dump_ttr(env->mmu.ttr[M68K_ITTR0]);
    qemu_printf("ITTR1: ");
    dump_ttr(env->mmu.ttr[M68K_ITTR1]);
    qemu_printf("DTTR0: ");
    dump_ttr(env->mmu.ttr[M68K_DTTR0]);
    qemu_printf("DTTR1: ");
    dump_ttr(env->mmu.ttr[M68K_DTTR1]);

    qemu_printf("SRP: 0x%08x\n", env->mmu.srp);
    dump_address_map(env, env->mmu.srp);

    qemu_printf("URP: 0x%08x\n", env->mmu.urp);
    dump_address_map(env, env->mmu.urp);
}

static int check_TTR(uint32_t ttr, int *prot, target_ulong addr,
                     int access_type)
{
    uint32_t base, mask;

    /* check if transparent translation is enabled */
    if ((ttr & M68K_TTR_ENABLED) == 0) {
        return 0;
    }

    /* check mode access */
    switch (ttr & M68K_TTR_SFIELD) {
    case M68K_TTR_SFIELD_USER:
        /* match only if user */
        if ((access_type & ACCESS_SUPER) != 0) {
            return 0;
        }
        break;
    case M68K_TTR_SFIELD_SUPER:
        /* match only if supervisor */
        if ((access_type & ACCESS_SUPER) == 0) {
            return 0;
        }
        break;
    default:
        /* all other values disable mode matching (FC2) */
        break;
    }

    /* check address matching */

    base = ttr & M68K_TTR_ADDR_BASE;
    mask = (ttr & M68K_TTR_ADDR_MASK) ^ M68K_TTR_ADDR_MASK;
    mask <<= M68K_TTR_ADDR_MASK_SHIFT;

    if ((addr & mask) != (base & mask)) {
        return 0;
    }

    *prot = PAGE_READ | PAGE_EXEC;
    if ((ttr & M68K_DESC_WRITEPROT) == 0) {
        *prot |= PAGE_WRITE;
    }

    return 1;
}

static int get_physical_address(CPUM68KState *env, hwaddr *physical,
                                int *prot, target_ulong address,
                                int access_type, target_ulong *page_size)
{
    CPUState *cs = env_cpu(env);
    uint32_t entry;
    uint32_t next;
    target_ulong page_mask;
    bool debug = access_type & ACCESS_DEBUG;
    int page_bits;
    int i;
    MemTxResult txres;

    /* Transparent Translation (physical = logical) */
    for (i = 0; i < M68K_MAX_TTR; i++) {
        if (check_TTR(env->mmu.TTR(access_type, i),
                      prot, address, access_type)) {
            if (access_type & ACCESS_PTEST) {
                /* Transparent Translation Register bit */
                env->mmu.mmusr = M68K_MMU_T_040 | M68K_MMU_R_040;
            }
            *physical = address;
            *page_size = TARGET_PAGE_SIZE;
            return 0;
        }
    }

    /* Page Table Root Pointer */
    *prot = PAGE_READ | PAGE_WRITE;
    if (access_type & ACCESS_CODE) {
        *prot |= PAGE_EXEC;
    }
    if (access_type & ACCESS_SUPER) {
        next = env->mmu.srp;
    } else {
        next = env->mmu.urp;
    }

    /* Root Index */
    entry = M68K_POINTER_BASE(next) | M68K_ROOT_INDEX(address);

    next = address_space_ldl(cs->as, entry, MEMTXATTRS_UNSPECIFIED, &txres);
    if (txres != MEMTX_OK) {
        goto txfail;
    }
    if (!M68K_UDT_VALID(next)) {
        return -1;
    }
    if (!(next & M68K_DESC_USED) && !debug) {
        address_space_stl(cs->as, entry, next | M68K_DESC_USED,
                          MEMTXATTRS_UNSPECIFIED, &txres);
        if (txres != MEMTX_OK) {
            goto txfail;
        }
    }
    if (next & M68K_DESC_WRITEPROT) {
        if (access_type & ACCESS_PTEST) {
            env->mmu.mmusr |= M68K_MMU_WP_040;
        }
        *prot &= ~PAGE_WRITE;
        if (access_type & ACCESS_STORE) {
            return -1;
        }
    }

    /* Pointer Index */
    entry = M68K_POINTER_BASE(next) | M68K_POINTER_INDEX(address);

    next = address_space_ldl(cs->as, entry, MEMTXATTRS_UNSPECIFIED, &txres);
    if (txres != MEMTX_OK) {
        goto txfail;
    }
    if (!M68K_UDT_VALID(next)) {
        return -1;
    }
    if (!(next & M68K_DESC_USED) && !debug) {
        address_space_stl(cs->as, entry, next | M68K_DESC_USED,
                          MEMTXATTRS_UNSPECIFIED, &txres);
        if (txres != MEMTX_OK) {
            goto txfail;
        }
    }
    if (next & M68K_DESC_WRITEPROT) {
        if (access_type & ACCESS_PTEST) {
            env->mmu.mmusr |= M68K_MMU_WP_040;
        }
        *prot &= ~PAGE_WRITE;
        if (access_type & ACCESS_STORE) {
            return -1;
        }
    }

    /* Page Index */
    if (env->mmu.tcr & M68K_TCR_PAGE_8K) {
        entry = M68K_8K_PAGE_BASE(next) | M68K_8K_PAGE_INDEX(address);
    } else {
        entry = M68K_4K_PAGE_BASE(next) | M68K_4K_PAGE_INDEX(address);
    }

    next = address_space_ldl(cs->as, entry, MEMTXATTRS_UNSPECIFIED, &txres);
    if (txres != MEMTX_OK) {
        goto txfail;
    }

    if (!M68K_PDT_VALID(next)) {
        return -1;
    }
    if (M68K_PDT_INDIRECT(next)) {
        /* U/M bits belong to the resolved page descriptor, not the link. */
        entry = M68K_INDIRECT_POINTER(next);
        next = address_space_ldl(cs->as, entry, MEMTXATTRS_UNSPECIFIED,
                                 &txres);
        if (txres != MEMTX_OK) {
            goto txfail;
        }
        if (!(next & 1)) {
            /* An invalid or second indirect descriptor is not a page. */
            return -1;
        }
    }
    if (access_type & ACCESS_STORE) {
        if (next & M68K_DESC_WRITEPROT) {
            if (!(next & M68K_DESC_USED) && !debug) {
                address_space_stl(cs->as, entry, next | M68K_DESC_USED,
                                  MEMTXATTRS_UNSPECIFIED, &txres);
                if (txres != MEMTX_OK) {
                    goto txfail;
                }
            }
        } else if ((next & (M68K_DESC_MODIFIED | M68K_DESC_USED)) !=
                           (M68K_DESC_MODIFIED | M68K_DESC_USED) && !debug) {
            address_space_stl(cs->as, entry,
                              next | (M68K_DESC_MODIFIED | M68K_DESC_USED),
                              MEMTXATTRS_UNSPECIFIED, &txres);
            if (txres != MEMTX_OK) {
                goto txfail;
            }
        }
    } else {
        if (!(next & M68K_DESC_USED) && !debug) {
            address_space_stl(cs->as, entry, next | M68K_DESC_USED,
                              MEMTXATTRS_UNSPECIFIED, &txres);
            if (txres != MEMTX_OK) {
                goto txfail;
            }
        }
    }

    if (env->mmu.tcr & M68K_TCR_PAGE_8K) {
        page_bits = 13;
    } else {
        page_bits = 12;
    }
    *page_size = 1 << page_bits;
    page_mask = ~(*page_size - 1);
    *physical = (next & page_mask) + (address & (*page_size - 1));

    if (access_type & ACCESS_PTEST) {
        env->mmu.mmusr |= next & M68K_MMU_SR_MASK_040;
        env->mmu.mmusr |= *physical & 0xfffff000;
        env->mmu.mmusr |= M68K_MMU_R_040;
    }

    if (next & M68K_DESC_WRITEPROT) {
        *prot &= ~PAGE_WRITE;
        if (access_type & ACCESS_STORE) {
            return -1;
        }
    }
    if (next & M68K_DESC_SUPERONLY) {
        if ((access_type & ACCESS_SUPER) == 0) {
            return -1;
        }
    }

    if (!(access_type & ACCESS_STORE) && !(next & M68K_DESC_MODIFIED)) {
        /*
         * A read can populate the QEMU TLB, but the first later write must
         * still revisit the descriptor to set its 68040 Modified bit.
         */
        *prot &= ~PAGE_WRITE;
    }

    return 0;

txfail:
    /*
     * A page table load/store failed. TODO: we should really raise a
     * suitable guest fault here if this is not a debug access.
     * For now just return that the translation failed.
     */
    return -1;
}

/*
 * 68030 PMMU translation.  The table geometry is described by tc030: an
 * initial shift (IS) drops high address bits, then up to four table
 * index fields (TIA..TID) walk the tree, and the low PS bits are the
 * page offset.  Each root pointer / table descriptor names the size of
 * the descriptors in the table it points at (short 4-byte or long
 * 8-byte); a descriptor type of "page" ends the walk, and a page found
 * before the index fields are exhausted is an early-terminated (larger)
 * page.
 */
/* 68030 MMU status register (PSR) bits, as read by PMOVE from PSR */
#define M68K_MMU030_PSR_B   0x8000  /* bus error during table walk */
#define M68K_MMU030_PSR_L   0x4000  /* limit violation */
#define M68K_MMU030_PSR_S   0x2000  /* supervisor-only violation */
#define M68K_MMU030_PSR_W   0x0800  /* write protected */
#define M68K_MMU030_PSR_I   0x0400  /* invalid descriptor */

static int get_physical_address_030(CPUM68KState *env, hwaddr *physical,
                                    int *prot, target_ulong address,
                                    int access_type, target_ulong *page_size)
{
    CPUState *cs = env_cpu(env);
    uint32_t tc = env->mmu.tc030;
    uint32_t *rp, table, desc = 0;
    int dt, is, shift, wp = 0, super_only = 0, level, i;
    bool ptest = access_type & ACCESS_PTEST;
    MemTxResult txres;

    /* transparent translation: the 030 TT registers match the 040 TTR */
    for (i = 0; i < 2; i++) {
        if (check_TTR(env->mmu.tt030[i], prot, address, access_type)) {
            *physical = address;
            *page_size = TARGET_PAGE_SIZE;
            return 0;
        }
    }

    /*
     * Software ATC (see cpu.h): serve repeat translations of a guest
     * page from the cache without touching the tables, like the real
     * 22-entry ATC.  PTEST must really probe the tables, and a first
     * STORE to a page cached by a read re-walks so the M bit lands in
     * the descriptor.
     */
    /*
     * The IS field's initial shift removes the top address bits from
     * translation entirely, so they must not distinguish ATC entries
     * either (MacOS reaches the same page through plain and through
     * 24-bit-tagged pointers, e.g. 0x004F9E vs 0x80004F9E under IS=8).
     */
    uint32_t vkey = address & (0xffffffffu >> M68K_TC030_IS(tc));

    if (!(access_type & (ACCESS_PTEST | ACCESS_DEBUG))) {
        for (i = 0; i < ARRAY_SIZE(env->mmu.atc030); i++) {
            typeof(env->mmu.atc030[0]) *e = &env->mmu.atc030[i];

            if (e->mask && (vkey & ~e->mask) == e->vaddr) {
                if (e->super_only && !(access_type & ACCESS_SUPER)) {
                    break;      /* re-walk for the correct violation */
                }
                if ((access_type & ACCESS_STORE) &&
                    (!(e->prot & PAGE_WRITE) || !e->dirty)) {
                    break;      /* WP fault or M-bit update: re-walk */
                }
                *physical = e->paddr | (address & e->mask);
                *page_size = e->mask + 1;
                *prot = e->prot;
                return 0;
            }
        }
    }

    uint32_t limit_word;
    bool limited;

    rp = ((access_type & ACCESS_SUPER) && (tc & M68K_TC030_SRE))
         ? env->mmu.srp030 : env->mmu.crp030;
    dt = rp[0] & 3;
    table = rp[1] & M68K_DESC030_ADDR;

    /*
     * Long-format table descriptors (and the root pointer) carry a
     * 15-bit index limit for the table they point at, with bit 31
     * selecting lower-limit (1) or upper-limit (0) semantics.  An
     * index outside the range is a limit violation (bus error).  The
     * Mac IIsi ROM depends on this: its compact 32-bit map bounds the
     * RAM level-B table to a single entry with limit 0 and leaves the
     * rest of the table area as RAM-test garbage.
     */
    limit_word = rp[0];
    limited = true;

    is = M68K_TC030_IS(tc);
    shift = 32 - is;

    for (level = 0; level < 4; level++) {
        int tw = M68K_TC030_TI(tc, level);
        uint32_t entry;
        int index;

        if (dt == M68K_DT_INVALID) {
            if (ptest) {
                env->mmu.mmusr |= M68K_MMU030_PSR_I;
            }
            return -1;
        }
        if (dt == M68K_DT_PAGE || tw == 0) {
            break;
        }
        shift -= tw;
        index = (address >> shift) & ((1 << tw) - 1);
        if (limited) {
            uint32_t limit = (limit_word >> 16) & 0x7fff;

            if (limit_word & 0x80000000) {
                if ((uint32_t)index < limit) {
                    if (ptest) {
                        env->mmu.mmusr |= M68K_MMU030_PSR_L;
                    }
                    return -1;
                }
            } else {
                if ((uint32_t)index > limit) {
                    if (ptest) {
                        env->mmu.mmusr |= M68K_MMU030_PSR_L;
                    }
                    return -1;
                }
            }
        }
        entry = table + index * (dt == M68K_DT_LONG ? 8 : 4);

        desc = address_space_ldl(cs->as, entry, MEMTXATTRS_UNSPECIFIED, &txres);
        if (txres != MEMTX_OK) {
            if (ptest) {
                env->mmu.mmusr |= M68K_MMU030_PSR_B;
            }
            return -1;
        }
        if (dt == M68K_DT_LONG) {
            uint32_t lo = address_space_ldl(cs->as, entry + 4,
                                            MEMTXATTRS_UNSPECIFIED, &txres);
            if (txres != MEMTX_OK) {
                if (ptest) {
                    env->mmu.mmusr |= M68K_MMU030_PSR_B;
                }
                return -1;
            }
            table = lo & M68K_DESC030_ADDR;
            limit_word = desc;
            limited = true;
        } else {
            table = desc & M68K_DESC030_ADDR;
            limited = false;
        }
        if (desc & M68K_DESC030_WP) {
            wp = 1;
        }
        if (desc & 0x100) {           /* long descriptor supervisor bit */
            super_only = 1;
        }
        dt = desc & 3;

        /*
         * Set the descriptor's Used bit (and Modified on the leaf page
         * for a store) so the guest sees the MMU exercising its tables.
         * PTEST is a probe and must leave the tables untouched.
         */
        if (!(access_type & (ACCESS_DEBUG | ACCESS_PTEST))) {
            uint32_t nd = desc | M68K_DESC030_U;
            if (dt == M68K_DT_PAGE && (access_type & ACCESS_STORE) &&
                !(desc & M68K_DESC030_WP)) {
                nd |= M68K_DESC030L_M;
            }
            if (nd != desc) {
                address_space_stl(cs->as, entry, nd,
                                  MEMTXATTRS_UNSPECIFIED, &txres);
            }
        }
    }

    if (dt != M68K_DT_PAGE) {
        if (ptest) {
            env->mmu.mmusr |= M68K_MMU030_PSR_I;
        }
        return -1;
    }
    /*
     * PAGE descriptors (both formats) hold the page address in bits
     * 31-8; bits 7-4 are status (M, CI, ...), unlike table descriptors
     * whose address field extends down to bit 4.  Masking only the low
     * nibble leaked a set M/U bit (e.g. short page descriptor
     * 0x007F8019 -> "frame" 0x7F8010) into the physical address: the
     * Mac IIsi System boot then wrote its MMU-table wipe 16 bytes
     * high, which crossed a 4K boundary at offset 0xFFC and corrupted
     * the live level-A table.
     */
    table &= 0xffffff00;
    if (ptest && wp) {
        env->mmu.mmusr |= M68K_MMU030_PSR_W;
    }
    if (super_only && !(access_type & ACCESS_SUPER)) {
        if (ptest) {
            env->mmu.mmusr |= M68K_MMU030_PSR_S;
        }
        return -1;
    }

    /* whatever address bits the index fields did not consume are offset */
    if (shift >= 32) {
        /*
         * Early termination in the root pointer with IS=0: one "page"
         * spans the whole 4G space and the descriptor address is added
         * to the full logical address (147Bug's MMU register test uses
         * this as an identity map while the test patterns are live).
         * 1u << 32 is undefined C, so special-case it.
         */
        *physical = table + address;
        *page_size = TARGET_PAGE_SIZE;
    } else {
        /*
         * Early termination: the descriptor's page address is only
         * required to be page-size (PS) aligned, NOT aligned to the
         * whole span covered by the remaining logical bits, which are
         * ADDED to it (68030 UM 9.5.3).  The Mac IIsi ROM relies on
         * this: its 24-bit map relocates each 1 MB of logical RAM to
         * frame N*0x100000 + 0x50000 (the bottom 320 KiB of RAM is the
         * onboard video frame buffer).  Masking the frame down to the
         * span made that map an identity map, so the PrimaryInit gray
         * fill of the screen destroyed low memory.  Only advertise the
         * full span to the TLB when the frame is span-aligned (the
         * addend is span-constant either way, but tlb_set_page expects
         * aligned large pages).
         */
        uint32_t span = 1u << shift;

        *physical = table + (address & (span - 1));
        *page_size = (table & (span - 1)) ? TARGET_PAGE_SIZE : span;
    }
    *prot = PAGE_READ | PAGE_EXEC;
    if (!wp) {
        *prot |= PAGE_WRITE;
    } else if (access_type & ACCESS_STORE) {
        return -1;
    }

    /* record the walk in the software ATC */
    if (!(access_type & (ACCESS_PTEST | ACCESS_DEBUG))) {
        uint32_t mask = *page_size - 1;
        uint32_t vpage = vkey & ~mask;
        typeof(env->mmu.atc030[0]) *e = NULL;

        for (i = 0; i < ARRAY_SIZE(env->mmu.atc030); i++) {
            if (env->mmu.atc030[i].mask &&
                env->mmu.atc030[i].vaddr == vpage) {
                e = &env->mmu.atc030[i];
                break;
            }
        }
        if (!e) {
            e = &env->mmu.atc030[env->mmu.atc030_next];
            env->mmu.atc030_next = (env->mmu.atc030_next + 1) %
                                   ARRAY_SIZE(env->mmu.atc030);
        }
        e->vaddr = vpage;
        e->mask = mask;
        e->paddr = *physical & ~mask;
        e->prot = *prot;
        e->super_only = super_only;
        e->dirty = (access_type & ACCESS_STORE) != 0;
    }
    return 0;
}

hwaddr m68k_cpu_get_phys_addr_debug(CPUState *cs, vaddr addr)
{
    CPUM68KState *env = cpu_env(cs);
    hwaddr phys_addr;
    int prot;
    int access_type;
    target_ulong page_size;

    if ((env->mmu.tcr & M68K_TCR_ENABLED) == 0) {
        /* MMU disabled */
        return addr;
    }

    access_type = ACCESS_DATA | ACCESS_DEBUG;
    if (env->sr & SR_S) {
        access_type |= ACCESS_SUPER;
    }

    if (get_physical_address(env, &phys_addr, &prot,
                             addr, access_type, &page_size) != 0) {
        return -1;
    }

    return phys_addr;
}

/*
 * Notify CPU of a pending interrupt.  Prioritization and vectoring should
 * be handled by the interrupt controller.  Real hardware only requests
 * the vector when the interrupt is acknowledged by the CPU.  For
 * simplicity we calculate it when the interrupt is signalled.
 */
void m68k_set_irq_level(M68kCPU *cpu, int level, uint8_t vector)
{
    CPUState *cs = CPU(cpu);
    CPUM68KState *env = &cpu->env;

    env->pending_level = level;
    env->pending_vector = vector;
    if (level) {
        cpu_interrupt(cs, CPU_INTERRUPT_HARD);
    } else {
        cpu_reset_interrupt(cs, CPU_INTERRUPT_HARD);
    }
}

bool m68k_cpu_tlb_fill(CPUState *cs, vaddr address, int size,
                       MMUAccessType qemu_access_type, int mmu_idx,
                       bool probe, uintptr_t retaddr)
{
    CPUM68KState *env = cpu_env(cs);
    hwaddr physical;
    int prot;
    int access_type;
    int ret;
    target_ulong page_size;
    bool is_030 = m68k_feature(env, M68K_FEATURE_M68030);
    bool mmu_enabled = is_030 ? (env->mmu.tc030 & M68K_TC030_ENABLE)
                              : (env->mmu.tcr & M68K_TCR_ENABLED);

    if (!mmu_enabled) {
        /* MMU disabled */
        tlb_set_page(cs, address & TARGET_PAGE_MASK,
                     address & TARGET_PAGE_MASK,
                     PAGE_READ | PAGE_WRITE | PAGE_EXEC,
                     mmu_idx, TARGET_PAGE_SIZE);
        return true;
    }

    if (qemu_access_type == MMU_INST_FETCH) {
        access_type = ACCESS_CODE;
    } else {
        access_type = ACCESS_DATA;
        if (qemu_access_type == MMU_DATA_STORE) {
            access_type |= ACCESS_STORE;
        }
    }
    if (mmu_idx != MMU_USER_IDX) {
        access_type |= ACCESS_SUPER;
    }

    ret = is_030
        ? get_physical_address_030(env, &physical, &prot,
                                   address, access_type, &page_size)
        : get_physical_address(env, &physical, &prot,
                               address, access_type, &page_size);
    if (likely(ret == 0)) {
        tlb_set_page(cs, address & TARGET_PAGE_MASK,
                     physical & TARGET_PAGE_MASK, prot, mmu_idx, page_size);
        return true;
    }

    if (probe) {
        return false;
    }

    if (is_030) {
        /*
         * 030 table walk miss: deliver a data/instruction bus fault.
         * The SSW carries the function code of the faulted cycle;
         * Linux keys its user/kernel fault split off it and feeds it
         * back into PTEST when it re-probes the address.
         */
        int fc = (qemu_access_type == MMU_INST_FETCH ? 2 : 1) |
                 (mmu_idx == MMU_USER_IDX ? 0 : 4);

        env->mmu.ssw = M68K_SSW_FC_030(fc);
        if (qemu_access_type != MMU_INST_FETCH) {
            env->mmu.ssw |= M68K_SSW_DF_030;
        }
        if (qemu_access_type != MMU_DATA_STORE) {
            env->mmu.ssw |= M68K_SSW_RW_030;
        }
        cs->exception_index = EXCP_ACCESS;
        env->mmu.ar = address;
        cpu_loop_exit_restore(cs, retaddr);
    }

    /* page fault */
    env->mmu.ssw = M68K_ATC_040;
    switch (size) {
    case 1:
        env->mmu.ssw |= M68K_BA_SIZE_BYTE;
        break;
    case 2:
        env->mmu.ssw |= M68K_BA_SIZE_WORD;
        break;
    case 4:
        env->mmu.ssw |= M68K_BA_SIZE_LONG;
        break;
    }
    if (access_type & ACCESS_SUPER) {
        env->mmu.ssw |= M68K_TM_040_SUPER;
    }
    if (access_type & ACCESS_CODE) {
        env->mmu.ssw |= M68K_TM_040_CODE;
    } else {
        env->mmu.ssw |= M68K_TM_040_DATA;
    }
    if (!(access_type & ACCESS_STORE)) {
        env->mmu.ssw |= M68K_RW_040;
    }

    cs->exception_index = EXCP_ACCESS;
    env->mmu.ar = address;
    cpu_loop_exit_restore(cs, retaddr);
}
#endif /* !CONFIG_USER_ONLY */

uint32_t HELPER(bitrev)(uint32_t x)
{
    x = ((x >> 1) & 0x55555555u) | ((x << 1) & 0xaaaaaaaau);
    x = ((x >> 2) & 0x33333333u) | ((x << 2) & 0xccccccccu);
    x = ((x >> 4) & 0x0f0f0f0fu) | ((x << 4) & 0xf0f0f0f0u);
    return bswap32(x);
}

uint32_t HELPER(ff1)(uint32_t x)
{
    int n;
    for (n = 32; x; n--)
        x >>= 1;
    return n;
}

uint32_t HELPER(sats)(uint32_t val, uint32_t v)
{
    /* The result has the opposite sign to the original value.  */
    if ((int32_t)v < 0) {
        val = (((int32_t)val) >> 31) ^ SIGNBIT;
    }
    return val;
}

void cpu_m68k_set_sr(CPUM68KState *env, uint32_t sr)
{
    env->sr = sr & 0xffe0;
    cpu_m68k_set_ccr(env, sr);
    m68k_switch_sp(env);
}

void HELPER(set_sr)(CPUM68KState *env, uint32_t val)
{
    cpu_m68k_set_sr(env, val);
}

/* MAC unit.  */
/*
 * FIXME: The MAC unit implementation is a bit of a mess.  Some helpers
 * take values,  others take register numbers and manipulate the contents
 * in-place.
 */
void HELPER(mac_move)(CPUM68KState *env, uint32_t dest, uint32_t src)
{
    uint32_t mask;
    env->macc[dest] = env->macc[src];
    mask = MACSR_PAV0 << dest;
    if (env->macsr & (MACSR_PAV0 << src))
        env->macsr |= mask;
    else
        env->macsr &= ~mask;
}

uint64_t HELPER(macmuls)(CPUM68KState *env, uint32_t op1, uint32_t op2)
{
    int64_t product;
    int64_t res;

    product = (uint64_t)op1 * op2;
    res = (product << 24) >> 24;
    if (res != product) {
        env->macsr |= MACSR_V;
        if (env->macsr & MACSR_OMC) {
            /* Make sure the accumulate operation overflows.  */
            if (product < 0)
                res = ~(1ll << 50);
            else
                res = 1ll << 50;
        }
    }
    return res;
}

uint64_t HELPER(macmulu)(CPUM68KState *env, uint32_t op1, uint32_t op2)
{
    uint64_t product;

    product = (uint64_t)op1 * op2;
    if (product & (0xffffffull << 40)) {
        env->macsr |= MACSR_V;
        if (env->macsr & MACSR_OMC) {
            /* Make sure the accumulate operation overflows.  */
            product = 1ll << 50;
        } else {
            product &= ((1ull << 40) - 1);
        }
    }
    return product;
}

uint64_t HELPER(macmulf)(CPUM68KState *env, uint32_t op1, uint32_t op2)
{
    uint64_t product;
    uint32_t remainder;

    product = (uint64_t)op1 * op2;
    if (env->macsr & MACSR_RT) {
        remainder = product & 0xffffff;
        product >>= 24;
        if (remainder > 0x800000)
            product++;
        else if (remainder == 0x800000)
            product += (product & 1);
    } else {
        product >>= 24;
    }
    return product;
}

void HELPER(macsats)(CPUM68KState *env, uint32_t acc)
{
    int64_t tmp;
    int64_t result;
    tmp = env->macc[acc];
    result = ((tmp << 16) >> 16);
    if (result != tmp) {
        env->macsr |= MACSR_V;
    }
    if (env->macsr & MACSR_V) {
        env->macsr |= MACSR_PAV0 << acc;
        if (env->macsr & MACSR_OMC) {
            /*
             * The result is saturated to 32 bits, despite overflow occurring
             * at 48 bits.  Seems weird, but that's what the hardware docs
             * say.
             */
            result = (result >> 63) ^ 0x7fffffff;
        }
    }
    env->macc[acc] = result;
}

void HELPER(macsatu)(CPUM68KState *env, uint32_t acc)
{
    uint64_t val;

    val = env->macc[acc];
    if (val & (0xffffull << 48)) {
        env->macsr |= MACSR_V;
    }
    if (env->macsr & MACSR_V) {
        env->macsr |= MACSR_PAV0 << acc;
        if (env->macsr & MACSR_OMC) {
            if (val > (1ull << 53))
                val = 0;
            else
                val = (1ull << 48) - 1;
        } else {
            val &= ((1ull << 48) - 1);
        }
    }
    env->macc[acc] = val;
}

void HELPER(macsatf)(CPUM68KState *env, uint32_t acc)
{
    int64_t sum;
    int64_t result;

    sum = env->macc[acc];
    result = (sum << 16) >> 16;
    if (result != sum) {
        env->macsr |= MACSR_V;
    }
    if (env->macsr & MACSR_V) {
        env->macsr |= MACSR_PAV0 << acc;
        if (env->macsr & MACSR_OMC) {
            result = (result >> 63) ^ 0x7fffffffffffll;
        }
    }
    env->macc[acc] = result;
}

void HELPER(mac_set_flags)(CPUM68KState *env, uint32_t acc)
{
    uint64_t val;
    val = env->macc[acc];
    if (val == 0) {
        env->macsr |= MACSR_Z;
    } else if (val & (1ull << 47)) {
        env->macsr |= MACSR_N;
    }
    if (env->macsr & (MACSR_PAV0 << acc)) {
        env->macsr |= MACSR_V;
    }
    if (env->macsr & MACSR_FI) {
        val = ((int64_t)val) >> 40;
        if (val != 0 && val != -1)
            env->macsr |= MACSR_EV;
    } else if (env->macsr & MACSR_SU) {
        val = ((int64_t)val) >> 32;
        if (val != 0 && val != -1)
            env->macsr |= MACSR_EV;
    } else {
        if ((val >> 32) != 0)
            env->macsr |= MACSR_EV;
    }
}

#define EXTSIGN(val, index) (     \
    (index == 0) ? (int8_t)(val) : ((index == 1) ? (int16_t)(val) : (val)) \
)

#define COMPUTE_CCR(op, x, n, z, v, c) {                                   \
    switch (op) {                                                          \
    case CC_OP_FLAGS:                                                      \
        /* Everything in place.  */                                        \
        break;                                                             \
    case CC_OP_ADDB:                                                       \
    case CC_OP_ADDW:                                                       \
    case CC_OP_ADDL:                                                       \
        res = n;                                                           \
        src2 = v;                                                          \
        src1 = EXTSIGN(res - src2, op - CC_OP_ADDB);                       \
        c = x;                                                             \
        z = n;                                                             \
        v = (res ^ src1) & ~(src1 ^ src2);                                 \
        break;                                                             \
    case CC_OP_SUBB:                                                       \
    case CC_OP_SUBW:                                                       \
    case CC_OP_SUBL:                                                       \
        res = n;                                                           \
        src2 = v;                                                          \
        src1 = EXTSIGN(res + src2, op - CC_OP_SUBB);                       \
        c = x;                                                             \
        z = n;                                                             \
        v = (res ^ src1) & (src1 ^ src2);                                  \
        break;                                                             \
    case CC_OP_CMPB:                                                       \
    case CC_OP_CMPW:                                                       \
    case CC_OP_CMPL:                                                       \
        src1 = n;                                                          \
        src2 = v;                                                          \
        res = EXTSIGN(src1 - src2, op - CC_OP_CMPB);                       \
        n = res;                                                           \
        z = res;                                                           \
        c = src1 < src2;                                                   \
        v = (res ^ src1) & (src1 ^ src2);                                  \
        break;                                                             \
    case CC_OP_LOGIC:                                                      \
        c = v = 0;                                                         \
        z = n;                                                             \
        break;                                                             \
    default:                                                               \
        cpu_abort(env_cpu(env), "Bad CC_OP %d", op);                       \
    }                                                                      \
} while (0)

uint32_t cpu_m68k_get_ccr(CPUM68KState *env)
{
    uint32_t x, c, n, z, v;
    uint32_t res, src1, src2;

    x = env->cc_x;
    n = env->cc_n;
    z = env->cc_z;
    v = env->cc_v;
    c = env->cc_c;

    COMPUTE_CCR(env->cc_op, x, n, z, v, c);

    n = n >> 31;
    z = (z == 0);
    v = v >> 31;

    return x * CCF_X + n * CCF_N + z * CCF_Z + v * CCF_V + c * CCF_C;
}

uint32_t HELPER(get_ccr)(CPUM68KState *env)
{
    return cpu_m68k_get_ccr(env);
}

void cpu_m68k_set_ccr(CPUM68KState *env, uint32_t ccr)
{
    env->cc_x = (ccr & CCF_X ? 1 : 0);
    env->cc_n = (ccr & CCF_N ? -1 : 0);
    env->cc_z = (ccr & CCF_Z ? 0 : 1);
    env->cc_v = (ccr & CCF_V ? -1 : 0);
    env->cc_c = (ccr & CCF_C ? 1 : 0);
    env->cc_op = CC_OP_FLAGS;
}

void HELPER(set_ccr)(CPUM68KState *env, uint32_t ccr)
{
    cpu_m68k_set_ccr(env, ccr);
}

void HELPER(flush_flags)(CPUM68KState *env, uint32_t cc_op)
{
    uint32_t res, src1, src2;

    COMPUTE_CCR(cc_op, env->cc_x, env->cc_n, env->cc_z, env->cc_v, env->cc_c);
    env->cc_op = CC_OP_FLAGS;
}

uint32_t HELPER(get_macf)(CPUM68KState *env, uint64_t val)
{
    int rem;
    uint32_t result;

    if (env->macsr & MACSR_SU) {
        /* 16-bit rounding.  */
        rem = val & 0xffffff;
        val = (val >> 24) & 0xffffu;
        if (rem > 0x800000)
            val++;
        else if (rem == 0x800000)
            val += (val & 1);
    } else if (env->macsr & MACSR_RT) {
        /* 32-bit rounding.  */
        rem = val & 0xff;
        val >>= 8;
        if (rem > 0x80)
            val++;
        else if (rem == 0x80)
            val += (val & 1);
    } else {
        /* No rounding.  */
        val >>= 8;
    }
    if (env->macsr & MACSR_OMC) {
        /* Saturate.  */
        if (env->macsr & MACSR_SU) {
            if (val != (uint16_t) val) {
                result = ((val >> 63) ^ 0x7fff) & 0xffff;
            } else {
                result = val & 0xffff;
            }
        } else {
            if (val != (uint32_t)val) {
                result = ((uint32_t)(val >> 63) & 0x7fffffff);
            } else {
                result = (uint32_t)val;
            }
        }
    } else {
        /* No saturation.  */
        if (env->macsr & MACSR_SU) {
            result = val & 0xffff;
        } else {
            result = (uint32_t)val;
        }
    }
    return result;
}

uint32_t HELPER(get_macs)(uint64_t val)
{
    if (val == (int32_t)val) {
        return (int32_t)val;
    } else {
        return (val >> 61) ^ ~SIGNBIT;
    }
}

uint32_t HELPER(get_macu)(uint64_t val)
{
    if ((val >> 32) == 0) {
        return (uint32_t)val;
    } else {
        return 0xffffffffu;
    }
}

uint32_t HELPER(get_mac_extf)(CPUM68KState *env, uint32_t acc)
{
    uint32_t val;
    val = env->macc[acc] & 0x00ff;
    val |= (env->macc[acc] >> 32) & 0xff00;
    val |= (env->macc[acc + 1] << 16) & 0x00ff0000;
    val |= (env->macc[acc + 1] >> 16) & 0xff000000;
    return val;
}

uint32_t HELPER(get_mac_exti)(CPUM68KState *env, uint32_t acc)
{
    uint32_t val;
    val = (env->macc[acc] >> 32) & 0xffff;
    val |= (env->macc[acc + 1] >> 16) & 0xffff0000;
    return val;
}

void HELPER(set_mac_extf)(CPUM68KState *env, uint32_t val, uint32_t acc)
{
    int64_t res;
    int32_t tmp;
    res = env->macc[acc] & 0xffffffff00ull;
    tmp = (int16_t)(val & 0xff00);
    res |= ((int64_t)tmp) << 32;
    res |= val & 0xff;
    env->macc[acc] = res;
    res = env->macc[acc + 1] & 0xffffffff00ull;
    tmp = (val & 0xff000000);
    res |= ((int64_t)tmp) << 16;
    res |= (val >> 16) & 0xff;
    env->macc[acc + 1] = res;
}

void HELPER(set_mac_exts)(CPUM68KState *env, uint32_t val, uint32_t acc)
{
    int64_t res;
    int32_t tmp;
    res = (uint32_t)env->macc[acc];
    tmp = (int16_t)val;
    res |= ((int64_t)tmp) << 32;
    env->macc[acc] = res;
    res = (uint32_t)env->macc[acc + 1];
    tmp = val & 0xffff0000;
    res |= (int64_t)tmp << 16;
    env->macc[acc + 1] = res;
}

void HELPER(set_mac_extu)(CPUM68KState *env, uint32_t val, uint32_t acc)
{
    uint64_t res;
    res = (uint32_t)env->macc[acc];
    res |= ((uint64_t)(val & 0xffff)) << 32;
    env->macc[acc] = res;
    res = (uint32_t)env->macc[acc + 1];
    res |= (uint64_t)(val & 0xffff0000) << 16;
    env->macc[acc + 1] = res;
}

#if !defined(CONFIG_USER_ONLY)
void HELPER(ptest)(CPUM68KState *env, uint32_t addr, uint32_t is_read)
{
    hwaddr physical;
    int access_type;
    int prot;
    int ret;
    target_ulong page_size;

    access_type = ACCESS_PTEST;
    if (env->dfc & 4) {
        access_type |= ACCESS_SUPER;
    }
    if ((env->dfc & 3) == 2) {
        access_type |= ACCESS_CODE;
    }
    if (!is_read) {
        access_type |= ACCESS_STORE;
    }

    env->mmu.mmusr = 0;
    env->mmu.ssw = 0;
    ret = get_physical_address(env, &physical, &prot, addr,
                               access_type, &page_size);
    if (ret == 0) {
        tlb_set_page(env_cpu(env), addr & TARGET_PAGE_MASK,
                     physical & TARGET_PAGE_MASK,
                     prot, access_type & ACCESS_SUPER ?
                     MMU_KERNEL_IDX : MMU_USER_IDX, page_size);
    }
}

/*
 * 68030 PTEST: walk the translation tables for the address in the
 * function code named by the extension word and report the result in
 * the PSR (mmusr).  Linux's bus-error handler re-probes every data
 * fault this way and keys the page-fault path off the I and WP bits.
 */
void HELPER(ptest030)(CPUM68KState *env, uint32_t addr, uint32_t ext)
{
    hwaddr physical;
    int prot;
    target_ulong page_size;
    int access_type = ACCESS_PTEST | ACCESS_DATA;
    int fcfield = ext & 0x1f;
    int fc;

    /* the fc operand: immediate, data register, or SFC/DFC */
    if (fcfield & 0x10) {
        fc = fcfield & 7;
    } else if (fcfield & 0x08) {
        fc = env->dregs[fcfield & 7] & 7;
    } else if (fcfield & 0x01) {
        fc = env->dfc & 7;
    } else {
        fc = env->sfc & 7;
    }
    if (fc & 4) {
        access_type |= ACCESS_SUPER;
    }
    if ((fc & 3) == 2) {
        access_type |= ACCESS_CODE;
    }
    if (!(ext & (1 << 9))) {            /* PTESTW */
        access_type |= ACCESS_STORE;
    }

    env->mmu.mmusr = 0;
    get_physical_address_030(env, &physical, &prot, addr,
                             access_type, &page_size);
}

void HELPER(pflush)(CPUM68KState *env, uint32_t addr, uint32_t opmode)
{
    CPUState *cs = env_cpu(env);

    switch (opmode) {
    case 0: /* Flush page entry if not global */
    case 1: /* Flush page entry */
        tlb_flush_page(cs, addr);
        break;
    case 2: /* Flush all except global entries */
        tlb_flush(cs);
        break;
    case 3: /* Flush all entries */
        tlb_flush(cs);
        break;
    }
}

/* 030 PMMU register writes and PFLUSH invalidate any cached translation */
void HELPER(pmmu030_flush)(CPUM68KState *env)
{
    memset(env->mmu.atc030, 0, sizeof(env->mmu.atc030));
    tlb_flush(env_cpu(env));
}

void HELPER(reset)(CPUM68KState *env)
{
    M68kCPU *cpu = env_archcpu(env);

    /*
     * RESET asserts the external reset line: the board resets while
     * the CPU itself carries on with the next instruction.  Software
     * depends on the side effects - the Amiga ROM reboot code runs
     * RESET and then jumps through address 4, counting on the reset
     * to have re-enabled the Kickstart overlay there.
     */
    bql_lock();
    qemu_irq_pulse(cpu->reset_out);
    bql_unlock();
}
#endif /* !CONFIG_USER_ONLY */
