/* SPDX-License-Identifier: MIT */
/*
 * QEMU monitor
 */

#include "qemu/osdep.h"
#include "qemu/qemu-print.h"
#include "monitor/monitor.h"
#include "monitor/hmp.h"
#include "qobject/qdict.h"
#include "cpu.h"
#include "internals.h"

void hmp_info_iprs(Monitor *mon, const QDict *qdict)
{
    bool all_cpus = qdict_get_try_bool(qdict, "cpustate_all", false);
    int vcpu = qdict_get_try_int(qdict, "vcpu", -1);
    CPUState *cs;

    if (all_cpus) {
        CPU_FOREACH(cs) {
            monitor_printf(mon, "\nCPU#%d\n", cs->cpu_index);
            alpha_cpu_dump_iprs(cs);
        }
    } else {
        cs = vcpu >= 0 ? qemu_get_cpu(vcpu) : mon_get_cpu(mon);

        if (!cs) {
            if (vcpu >= 0) {
                monitor_printf(mon, "CPU#%d not available\n", vcpu);
            } else {
                monitor_printf(mon, "No CPU available\n");
            }
            return;
        }

        monitor_printf(mon, "\nCPU#%d\n", cs->cpu_index);
        alpha_cpu_dump_iprs(cs);
    }
}

void hmp_info_tlb(Monitor *mon, const QDict *qdict)
{
    CPUState *cs = mon_get_cpu(mon);

    if (!cs) {
        monitor_printf(mon, "No CPU available\n");
        return;
    }
    alpha_cpu_dump_mmu(cs);
}

