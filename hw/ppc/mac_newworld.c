/*
 * QEMU PowerPC CHRP (currently NewWorld PowerMac) hardware System Emulator
 *
 * Copyright (c) 2004-2007 Fabrice Bellard
 * Copyright (c) 2007 Jocelyn Mayer
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * PCI bus layout on a real G5 (U3 based):
 *
 * 0000:f0:0b.0 Host bridge [0600]: Apple Computer Inc. U3 AGP [106b:004b]
 * 0000:f0:10.0 VGA compatible controller [0300]: ATI Technologies Inc RV350 AP [Radeon 9600] [1002:4150]
 * 0001:00:00.0 Host bridge [0600]: Apple Computer Inc. CPC945 HT Bridge [106b:004a]
 * 0001:00:01.0 PCI bridge [0604]: Advanced Micro Devices [AMD] AMD-8131 PCI-X Bridge [1022:7450] (rev 12)
 * 0001:00:02.0 PCI bridge [0604]: Advanced Micro Devices [AMD] AMD-8131 PCI-X Bridge [1022:7450] (rev 12)
 * 0001:00:03.0 PCI bridge [0604]: Apple Computer Inc. K2 HT-PCI Bridge [106b:0045]
 * 0001:00:04.0 PCI bridge [0604]: Apple Computer Inc. K2 HT-PCI Bridge [106b:0046]
 * 0001:00:05.0 PCI bridge [0604]: Apple Computer Inc. K2 HT-PCI Bridge [106b:0047]
 * 0001:00:06.0 PCI bridge [0604]: Apple Computer Inc. K2 HT-PCI Bridge [106b:0048]
 * 0001:00:07.0 PCI bridge [0604]: Apple Computer Inc. K2 HT-PCI Bridge [106b:0049]
 * 0001:01:07.0 Class [ff00]: Apple Computer Inc. K2 KeyLargo Mac/IO [106b:0041] (rev 20)
 * 0001:01:08.0 USB Controller [0c03]: Apple Computer Inc. K2 KeyLargo USB [106b:0040]
 * 0001:01:09.0 USB Controller [0c03]: Apple Computer Inc. K2 KeyLargo USB [106b:0040]
 * 0001:02:0b.0 USB Controller [0c03]: NEC Corporation USB [1033:0035] (rev 43)
 * 0001:02:0b.1 USB Controller [0c03]: NEC Corporation USB [1033:0035] (rev 43)
 * 0001:02:0b.2 USB Controller [0c03]: NEC Corporation USB 2.0 [1033:00e0] (rev 04)
 * 0001:03:0d.0 Class [ff00]: Apple Computer Inc. K2 ATA/100 [106b:0043]
 * 0001:03:0e.0 FireWire (IEEE 1394) [0c00]: Apple Computer Inc. K2 FireWire [106b:0042]
 * 0001:04:0f.0 Ethernet controller [0200]: Apple Computer Inc. K2 GMAC (Sun GEM) [106b:004c]
 * 0001:05:0c.0 IDE interface [0101]: Broadcom K2 SATA [1166:0240]
 */

#include "qemu/osdep.h"
#include "qemu/datadir.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "exec/target_page.h"
#include "hw/ppc/ppc.h"
#include "hw/core/qdev-properties.h"
#include "hw/nvram/mac_nvram.h"
#include "system/block-backend.h"
#include "system/blockdev.h"
#include "hw/core/boards.h"
#include "hw/pci-host/uninorth.h"
#include "hw/input/adb.h"
#include "hw/ppc/mac_dbdma.h"
#include "hw/pci/pci.h"
#include "hw/pci/pci_bus.h"
#include "net/net.h"
#include "system/system.h"
#include "hw/nvram/fw_cfg.h"
#include "hw/char/escc.h"
#include "hw/misc/macio/macio.h"
#include "hw/nvram/eeprom_at24c.h"
#include "hw/nvram/mac_spd.h"
#include "hw/ppc/openpic.h"
#include "hw/core/loader.h"
#include "hw/core/fw-path-provider.h"
#include "elf.h"
#include <zlib.h>                /* adler32() for the CHRP nvram outer checksum */
#include "qemu/error-report.h"
#include "system/kvm.h"
#include "system/reset.h"
#include "kvm_ppc.h"
#include "hw/usb/usb.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "system/cpus.h"
#include "exec/cpu-common.h"
#include "trace.h"

#define MAX_IDE_BUS 2
#define CFG_ADDR 0xf0000510
#define U3_TBFREQ 33333333UL

#define NDRV_VGA_FILENAME "qemu_vga.ndrv"

#define PROM_FILENAME "openbios-ppc"
#define PROM_BASE 0xfff00000

static uint64_t core99_rom_read(void *opaque, hwaddr addr, unsigned size)
{
    /* never called: reads are served from the ROM device's RAM backing */
    return 0;
}

static void core99_rom_write(void *opaque, hwaddr addr, uint64_t val,
                             unsigned size)
{
    trace_mac99_rom_write(addr, size, val);
}

static const MemoryRegionOps core99_rom_ops = {
    .read = core99_rom_read,
    .write = core99_rom_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 8,
};

/* KeyLargo/MacIO base decoded by real hardware (and by real Apple ROMs) */
#define MACIO_BASE 0xf3000000
#define MACIO_WIN_SIZE 0x01000000
#define PROM_SIZE (1 * MiB)
/*
 * A real Apple ROM stores its Open Firmware variables in the boot flash --
 * the ROM's own info property lists fff04000 as a 0x4000 nvram section and
 * its write-characteristic as "flash" -- and reaches it as flat bytes rather
 * than through the Old World one-byte-per-halfword window.
 */
#define NVRAM_FLASH_SIZE 0x4000
/*
 * The ROM socket's full decode window; only the top PROM_SIZE is populated.
 * Mac OS 9's "nvram,flash" driver reaches the two NVRAM banks at
 * 0xff004000/0xff006000 -- the same 0x4000/0x6000 offsets of the flash part,
 * decoded 16MB below the top -- which is what ROM_WINDOW_SIZE has to cover.
 */
#define ROM_WINDOW_BASE 0xff000000
#define ROM_WINDOW_SIZE (16 * MiB)
#define KERNEL_LOAD_ADDR 0x01000000
#define KERNEL_GAP       0x00100000

#define TYPE_CORE99_MACHINE MACHINE_TYPE_NAME("mac99")
typedef struct Core99MachineState Core99MachineState;
DECLARE_INSTANCE_CHECKER(Core99MachineState, CORE99_MACHINE,
                         TYPE_CORE99_MACHINE)

typedef enum {
    CORE99_VIA_CONFIG_CUDA = 0,
    CORE99_VIA_CONFIG_PMU,
    CORE99_VIA_CONFIG_PMU_ADB
} Core99ViaConfig;

/*
 * The board data a boot ROM reads for itself, one row per machine.  This
 * generalizes the source branch's per-model files: a model is a table row.
 */
typedef struct Core99ModelInfo {
    const char *name;
    const char *nvram_flash_file;
    const char *cpu_type;
    const uint8_t *spd_donor;
    uint32_t tbfreq;
    uint32_t clockfreq;
    uint32_t busfreq;
    uint32_t hid1;      /* PLL_CFG is HID1[0:3] on 7400, [15:19] on 7455 */
    unsigned spd_slots;
} Core99ModelInfo;

static const Core99ModelInfo core99_models[] = {
    /* PowerMac3,1 "Sawtooth", 100MHz bus */
    { "pm31", "nvram-flash-31.img", POWERPC_CPU_TYPE_NAME("7400_v2.9"),
      mac_spd_dimm0, 25000000, 500000000, 100000000, 0xB0000000, 3 },
    /* PowerMac3,4 "Digital Audio", 933MHz (the Quicksilver ratio) */
    { "pm34", "nvram-flash-34.img", POWERPC_CPU_TYPE_NAME("7400_v2.9"),
      mac_spd_dimm0, 33290001, 933333331, 133160004, 0x20000000, 3 },
    /* PowerMac3,6 "Mirrored Drive Doors", dual 867MHz, 4 DIMM slots */
    { "pm36", "nvram-flash-36.img", POWERPC_CPU_TYPE_NAME("7455_v2.1"),
      mac_spd_pm36_dimm0, 33306279, 866666664, 133225119, 0x0000A000, 4 },
};

#define CORE99_DEFAULT_MODEL 1  /* pm34 */

struct Core99MachineState {
    /*< private >*/
    MachineState parent;

    Core99ViaConfig via_config;
    unsigned model;
    bool apple_rom;
    PCIBus *agp_bus;
};

static const Core99ModelInfo *core99_model_info(const Core99MachineState *cms)
{
    return &core99_models[cms->model];
}

static uint32_t core99_tbfreq(const Core99MachineState *cms)
{
    return core99_model_info(cms)->tbfreq;
}

static uint32_t core99_clockfreq(const Core99MachineState *cms)
{
    return core99_model_info(cms)->clockfreq;
}

static uint32_t core99_busfreq(const Core99MachineState *cms)
{
    return core99_model_info(cms)->busfreq;
}

/*
 * The Apple ROM measures the bus and multiplies by HID1's ratio itself, so
 * this has to agree with clockfreq; SPRs snap back to their registered
 * defaults on reset, so the default matters as much as the live value.
 */
static void core99_cpu_defaults(const Core99MachineState *cms,
                                PowerPCCPU *cpu)
{
    uint32_t hid1 = core99_model_info(cms)->hid1;

    if (!hid1) {
        return;
    }
    cpu->env.spr_cb[SPR_HID1].default_value = hid1;
    cpu->env.spr[SPR_HID1] = hid1;
}

/*
 * The DIMM SPD EEPROMs at 0x50 + slot, which is how a real Apple ROM sizes
 * RAM -- it never asks QEMU.  One donor module's geometry bytes are patched
 * and its byte-63 checksum re-summed per slot, so every -m has images.
 */
static void core99_add_spd_dimms(const Core99MachineState *cms, I2CBus *bus,
                                 uint64_t ram_size)
{
    static const struct {
        unsigned mb, rows, cols, ranks, density;
    } spd_geom[] = {
        { 512, 13, 10, 2, 0x40 },
        { 256, 13, 10, 1, 0x40 },
        { 128, 12, 10, 1, 0x20 },
        {  64, 12,  9, 1, 0x10 },
        {  32, 11,  9, 1, 0x08 },
    };
    const Core99ModelInfo *model = core99_model_info(cms);
    uint64_t left = ram_size;
    int slot = 0;
    unsigned g = 0;

    while (left && slot < model->spd_slots && g < ARRAY_SIZE(spd_geom)) {
        uint8_t spd[MAC_SPD_SIZE];
        unsigned sum, b;

        if (left < (uint64_t)spd_geom[g].mb * MiB) {
            g++;
            continue;
        }
        memcpy(spd, model->spd_donor, MAC_SPD_SIZE);
        spd[3] = spd_geom[g].rows;
        spd[4] = spd_geom[g].cols;
        spd[5] = spd_geom[g].ranks;
        spd[31] = spd_geom[g].density;
        for (sum = 0, b = 0; b < 63; b++) {
            sum += spd[b];
        }
        spd[63] = sum & 0xff;
        at24c_eeprom_init_rom(bus, 0x50 + slot, MAC_SPD_SIZE, spd,
                              MAC_SPD_SIZE);
        left -= (uint64_t)spd_geom[g].mb * MiB;
        slot++;
    }
    if (left) {
        warn_report("mac99: %" PRIu64 "MB of RAM not representable as "
                    "1-%u %s DIMMs; the firmware will see %" PRIu64 "MB",
                    left / MiB, model->spd_slots, model->name,
                    (ram_size - left) / MiB);
    }
}

static void fw_cfg_boot_set(void *opaque, const char *boot_device,
                            Error **errp)
{
    fw_cfg_modify_i16(opaque, FW_CFG_BOOT_DEVICE, boot_device[0]);
}

static uint64_t translate_kernel_address(void *opaque, uint64_t addr)
{
    return (addr & 0x0fffffff) + KERNEL_LOAD_ADDR;
}

/*
 * Return a NUL-terminated copy of an OF variable's value from the nvram
 * "common" (0x70) partition, or NULL if not present. Caller frees.
 *
 * On flash-backed machines the 16 KB flash is two mirrored 8 KB copies and
 * the active data can live in either one (the other is left erased, 0xff, as
 * the next write target), so both copies are scanned; with the 8 KB MacIO
 * NVRAM there is a single copy and the loop runs once.
 */
static char *core99_nvram_get_var(const uint8_t *data, uint32_t size,
                                  const char *name)
{
    const uint32_t COPY = 0x2000;
    size_t nlen = strlen(name);
    uint32_t base, best = 0, best_gen = 0;
    bool have_best = false;

    /*
     * Two 8KB banks; the ROM (and Mac OS's nvram,flash driver) use the one
     * with the higher generation count (core99 header: CHRP header, adler32
     * at +0x10, generation at +0x14). Report from that one, not the first.
     */
    for (base = 0; base + COPY <= size; base += COPY) {
        uint32_t gen;

        if (data[base] != 0x5a || memcmp(&data[base + 4], "nvram", 5) != 0) {
            continue;
        }
        gen = ldl_be_p(&data[base + 0x14]);
        if (!have_best || gen > best_gen) {
            best = base;
            best_gen = gen;
            have_best = true;
        }
    }
    for (base = have_best ? best : 0; base + COPY <= size; base += COPY) {
        uint32_t off;

        for (off = base; off + 16 <= base + COPY; ) {
            uint32_t plen = (((data[off + 2] << 8) | data[off + 3]) * 16);
            uint32_t dstart, dend, i;

            if (plen == 0) {
                break;
            }
            /*
             * The Open Firmware variable partition is "common" on Apple's
             * own flash layout (a real ROM's) but "system" in QEMU's
             * chrp_nvram.c, which is what OpenBIOS writes into the MacIO
             * NVRAM; accept either so this reports on both paths.
             */
            if (data[off] != 0x70 ||
                (memcmp(&data[off + 4], "common", 6) != 0 &&
                 memcmp(&data[off + 4], "system", 6) != 0)) {
                off += plen;
                continue;
            }
            dstart = off + 16;
            dend = (plen > 16 && off + plen <= base + COPY) ? off + plen
                                                            : base + COPY;
            for (i = dstart; i < dend && data[i]; ) {
                const char *v = (const char *)&data[i];
                size_t vl = strnlen(v, dend - i);

                if (vl > nlen && v[nlen] == '=' && !memcmp(v, name, nlen)) {
                    return g_strndup(v + nlen + 1, vl - nlen - 1);
                }
                i += vl + 1;
            }
            break;
        }
    }
    return NULL;
}

/*
 * Report which device the machine is configured to boot from -- the OF
 * "boot-device" nvram variable (a trace event, `-trace mac99_boot_device`).
 * This is what the guest is *set* to boot from; OpenBIOS may still fall back
 * to scanning other devices when it is absent.
 */
static void core99_report_boot_device(MacIONVRAMState *nvr)
{
    g_autofree char *dev = nvr->data ?
        core99_nvram_get_var(nvr->data, nvr->size, "boot-device") : NULL;

    trace_mac99_boot_device(dev ? dev : "(unset)");
}

/*
 * A blank/never-written flash NVRAM (all bytes 0xff, matching real erased-
 * flash semantics) has no valid CHRP partition structure at all for
 * core99_nvram_graft()/core99_nvram_set_bootcmd() to patch into -- their
 * search loop for the "common" (0x70) partition finds nothing and silently
 * no-ops. The real Apple ROM detects this as "NVRAM corrupted" and rebuilds
 * its own default structure, but only in its own in-guest working copy;
 * that repair is never observed to persist back to the emulated flash's
 * backing file (confirmed live: even a full, successful boot leaves a
 * freshly-created backing file completely unchanged). Net effect: every
 * boot-command graft (gmac MAC, FireWire GUID, cache-POST) silently never
 * applies on a genuinely fresh nvram-flash-*.img, only on one a real ROM
 * has already formatted at some point in the past.
 *
 * Seed a real, byte-for-byte, ROM-validated default structure instead of
 * hand-constructing one -- extracted directly from a real PowerMac3,4
 * nvram-flash-34.img's second 8KB copy (the first copy in that same real
 * file is itself blank/unused, matching typical flash wear-leveling; only
 * one live copy is needed for the ROM's own scan to find). This is the
 * exact same "copy a real captured byte blob rather than reconstruct the
 * format from a spec" approach already used for the Old World OF partition
 * template above (see oldworld_of_partition_head/_strings) -- safer than
 * hand-rolling the "system" (0x5f, Apple-specific, not the generic CHRP
 * tag) and "APL,MacOS75" partitions this real capture also contains, which
 * this code has no need to understand the contents of, only to reproduce
 * faithfully enough that the real ROM's own validation accepts the whole
 * copy as uncorrupted.
 */
#define CORE99_NVRAM_TEMPLATE_SIZE 0x2000
static const uint8_t core99_nvram_flash_template[CORE99_NVRAM_TEMPLATE_SIZE] = {
    [0]=0x5a, [1]=0x82, [3]=0x02, [4]=0x6e, [5]=0x76, [6]=0x72, [7]=0x61, [8]=0x6d,
    [16]=0x7c, [17]=0x26, [18]=0x12, [19]=0x0a, [32]=0x5f, [33]=0x25, [35]=0x1e, [36]=0x73,
    [37]=0x79, [38]=0x73, [39]=0x74, [40]=0x65, [41]=0x6d, [512]=0x70, [513]=0xfd, [514]=0x01,
    [515]=0x01, [516]=0x63, [517]=0x6f, [518]=0x6d, [519]=0x6d, [520]=0x6f, [521]=0x6e, [528]=0x6c,
    [529]=0x69, [530]=0x74, [531]=0x74, [532]=0x6c, [533]=0x65, [534]=0x2d, [535]=0x65, [536]=0x6e,
    [537]=0x64, [538]=0x69, [539]=0x61, [540]=0x6e, [541]=0x3f, [542]=0x3d, [543]=0x66, [544]=0x61,
    [545]=0x6c, [546]=0x73, [547]=0x65, [549]=0x72, [550]=0x65, [551]=0x61, [552]=0x6c, [553]=0x2d,
    [554]=0x6d, [555]=0x6f, [556]=0x64, [557]=0x65, [558]=0x3f, [559]=0x3d, [560]=0x66, [561]=0x61,
    [562]=0x6c, [563]=0x73, [564]=0x65, [566]=0x61, [567]=0x75, [568]=0x74, [569]=0x6f, [570]=0x2d,
    [571]=0x62, [572]=0x6f, [573]=0x6f, [574]=0x74, [575]=0x3f, [576]=0x3d, [577]=0x74, [578]=0x72,
    [579]=0x75, [580]=0x65, [582]=0x64, [583]=0x69, [584]=0x61, [585]=0x67, [586]=0x2d, [587]=0x73,
    [588]=0x77, [589]=0x69, [590]=0x74, [591]=0x63, [592]=0x68, [593]=0x3f, [594]=0x3d, [595]=0x66,
    [596]=0x61, [597]=0x6c, [598]=0x73, [599]=0x65, [601]=0x66, [602]=0x63, [603]=0x6f, [604]=0x64,
    [605]=0x65, [606]=0x2d, [607]=0x64, [608]=0x65, [609]=0x62, [610]=0x75, [611]=0x67, [612]=0x3f,
    [613]=0x3d, [614]=0x66, [615]=0x61, [616]=0x6c, [617]=0x73, [618]=0x65, [620]=0x6f, [621]=0x65,
    [622]=0x6d, [623]=0x2d, [624]=0x62, [625]=0x61, [626]=0x6e, [627]=0x6e, [628]=0x65, [629]=0x72,
    [630]=0x3f, [631]=0x3d, [632]=0x66, [633]=0x61, [634]=0x6c, [635]=0x73, [636]=0x65, [638]=0x6f,
    [639]=0x65, [640]=0x6d, [641]=0x2d, [642]=0x6c, [643]=0x6f, [644]=0x67, [645]=0x6f, [646]=0x3f,
    [647]=0x3d, [648]=0x66, [649]=0x61, [650]=0x6c, [651]=0x73, [652]=0x65, [654]=0x75, [655]=0x73,
    [656]=0x65, [657]=0x2d, [658]=0x6e, [659]=0x76, [660]=0x72, [661]=0x61, [662]=0x6d, [663]=0x72,
    [664]=0x63, [665]=0x3f, [666]=0x3d, [667]=0x66, [668]=0x61, [669]=0x6c, [670]=0x73, [671]=0x65,
    [673]=0x75, [674]=0x73, [675]=0x65, [676]=0x2d, [677]=0x67, [678]=0x65, [679]=0x6e, [680]=0x65,
    [681]=0x72, [682]=0x69, [683]=0x63, [684]=0x3f, [685]=0x3d, [686]=0x66, [687]=0x61, [688]=0x6c,
    [689]=0x73, [690]=0x65, [692]=0x64, [693]=0x65, [694]=0x66, [695]=0x61, [696]=0x75, [697]=0x6c,
    [698]=0x74, [699]=0x2d, [700]=0x6d, [701]=0x61, [702]=0x63, [703]=0x2d, [704]=0x61, [705]=0x64,
    [706]=0x64, [707]=0x72, [708]=0x65, [709]=0x73, [710]=0x73, [711]=0x3f, [712]=0x3d, [713]=0x66,
    [714]=0x61, [715]=0x6c, [716]=0x73, [717]=0x65, [719]=0x72, [720]=0x65, [721]=0x61, [722]=0x6c,
    [723]=0x2d, [724]=0x62, [725]=0x61, [726]=0x73, [727]=0x65, [728]=0x3d, [729]=0x2d, [730]=0x31,
    [732]=0x72, [733]=0x65, [734]=0x61, [735]=0x6c, [736]=0x2d, [737]=0x73, [738]=0x69, [739]=0x7a,
    [740]=0x65, [741]=0x3d, [742]=0x2d, [743]=0x31, [745]=0x6c, [746]=0x6f, [747]=0x61, [748]=0x64,
    [749]=0x2d, [750]=0x62, [751]=0x61, [752]=0x73, [753]=0x65, [754]=0x3d, [755]=0x30, [756]=0x78,
    [757]=0x38, [758]=0x30, [759]=0x30, [760]=0x30, [761]=0x30, [762]=0x30, [764]=0x76, [765]=0x69,
    [766]=0x72, [767]=0x74, [768]=0x2d, [769]=0x62, [770]=0x61, [771]=0x73, [772]=0x65, [773]=0x3d,
    [774]=0x2d, [775]=0x31, [777]=0x76, [778]=0x69, [779]=0x72, [780]=0x74, [781]=0x2d, [782]=0x73,
    [783]=0x69, [784]=0x7a, [785]=0x65, [786]=0x3d, [787]=0x2d, [788]=0x31, [790]=0x70, [791]=0x63,
    [792]=0x69, [793]=0x2d, [794]=0x70, [795]=0x72, [796]=0x6f, [797]=0x62, [798]=0x65, [799]=0x2d,
    [800]=0x6d, [801]=0x61, [802]=0x73, [803]=0x6b, [804]=0x3d, [805]=0x2d, [806]=0x31, [808]=0x73,
    [809]=0x63, [810]=0x72, [811]=0x65, [812]=0x65, [813]=0x6e, [814]=0x2d, [815]=0x23, [816]=0x63,
    [817]=0x6f, [818]=0x6c, [819]=0x75, [820]=0x6d, [821]=0x6e, [822]=0x73, [823]=0x3d, [824]=0x31,
    [825]=0x30, [826]=0x30, [828]=0x73, [829]=0x63, [830]=0x72, [831]=0x65, [832]=0x65, [833]=0x6e,
    [834]=0x2d, [835]=0x23, [836]=0x72, [837]=0x6f, [838]=0x77, [839]=0x73, [840]=0x3d, [841]=0x34,
    [842]=0x30, [844]=0x73, [845]=0x65, [846]=0x6c, [847]=0x66, [848]=0x74, [849]=0x65, [850]=0x73,
    [851]=0x74, [852]=0x2d, [853]=0x23, [854]=0x6d, [855]=0x65, [856]=0x67, [857]=0x73, [858]=0x3d,
    [859]=0x30, [861]=0x62, [862]=0x6f, [863]=0x6f, [864]=0x74, [865]=0x2d, [866]=0x64, [867]=0x65,
    [868]=0x76, [869]=0x69, [870]=0x63, [871]=0x65, [872]=0x3d, [873]=0x68, [874]=0x64, [875]=0x3a,
    [876]=0x2c, [877]=0x5c, [878]=0x5c, [879]=0x3a, [880]=0x74, [881]=0x62, [882]=0x78, [883]=0x69,
    [885]=0x62, [886]=0x6f, [887]=0x6f, [888]=0x74, [889]=0x2d, [890]=0x66, [891]=0x69, [892]=0x6c,
    [893]=0x65, [894]=0x3d, [896]=0x62, [897]=0x6f, [898]=0x6f, [899]=0x74, [900]=0x2d, [901]=0x73,
    [902]=0x63, [903]=0x72, [904]=0x65, [905]=0x65, [906]=0x6e, [907]=0x3d, [909]=0x63, [910]=0x6f,
    [911]=0x6e, [912]=0x73, [913]=0x6f, [914]=0x6c, [915]=0x65, [916]=0x2d, [917]=0x73, [918]=0x63,
    [919]=0x72, [920]=0x65, [921]=0x65, [922]=0x6e, [923]=0x3d, [925]=0x64, [926]=0x69, [927]=0x61,
    [928]=0x67, [929]=0x2d, [930]=0x64, [931]=0x65, [932]=0x76, [933]=0x69, [934]=0x63, [935]=0x65,
    [936]=0x3d, [937]=0x65, [938]=0x6e, [939]=0x65, [940]=0x74, [942]=0x64, [943]=0x69, [944]=0x61,
    [945]=0x67, [946]=0x2d, [947]=0x66, [948]=0x69, [949]=0x6c, [950]=0x65, [951]=0x3d, [952]=0x2c,
    [953]=0x64, [954]=0x69, [955]=0x61, [956]=0x67, [957]=0x73, [959]=0x69, [960]=0x6e, [961]=0x70,
    [962]=0x75, [963]=0x74, [964]=0x2d, [965]=0x64, [966]=0x65, [967]=0x76, [968]=0x69, [969]=0x63,
    [970]=0x65, [971]=0x3d, [972]=0x6b, [973]=0x65, [974]=0x79, [975]=0x62, [976]=0x6f, [977]=0x61,
    [978]=0x72, [979]=0x64, [981]=0x6f, [982]=0x75, [983]=0x74, [984]=0x70, [985]=0x75, [986]=0x74,
    [987]=0x2d, [988]=0x64, [989]=0x65, [990]=0x76, [991]=0x69, [992]=0x63, [993]=0x65, [994]=0x3d,
    [995]=0x73, [996]=0x63, [997]=0x72, [998]=0x65, [999]=0x65, [1000]=0x6e, [1002]=0x69, [1003]=0x6e,
    [1004]=0x70, [1005]=0x75, [1006]=0x74, [1007]=0x2d, [1008]=0x64, [1009]=0x65, [1010]=0x76, [1011]=0x69,
    [1012]=0x63, [1013]=0x65, [1014]=0x2d, [1015]=0x31, [1016]=0x3d, [1017]=0x73, [1018]=0x63, [1019]=0x63,
    [1020]=0x61, [1022]=0x6f, [1023]=0x75, [1024]=0x74, [1025]=0x70, [1026]=0x75, [1027]=0x74, [1028]=0x2d,
    [1029]=0x64, [1030]=0x65, [1031]=0x76, [1032]=0x69, [1033]=0x63, [1034]=0x65, [1035]=0x2d, [1036]=0x31,
    [1037]=0x3d, [1038]=0x73, [1039]=0x63, [1040]=0x63, [1041]=0x61, [1043]=0x6d, [1044]=0x6f, [1045]=0x75,
    [1046]=0x73, [1047]=0x65, [1048]=0x2d, [1049]=0x64, [1050]=0x65, [1051]=0x76, [1052]=0x69, [1053]=0x63,
    [1054]=0x65, [1055]=0x3d, [1056]=0x6d, [1057]=0x6f, [1058]=0x75, [1059]=0x73, [1060]=0x65, [1062]=0x6f,
    [1063]=0x65, [1064]=0x6d, [1065]=0x2d, [1066]=0x62, [1067]=0x61, [1068]=0x6e, [1069]=0x6e, [1070]=0x65,
    [1071]=0x72, [1072]=0x3d, [1074]=0x6f, [1075]=0x65, [1076]=0x6d, [1077]=0x2d, [1078]=0x6c, [1079]=0x6f,
    [1080]=0x67, [1081]=0x6f, [1082]=0x3d, [1084]=0x6e, [1085]=0x76, [1086]=0x72, [1087]=0x61, [1088]=0x6d,
    [1089]=0x72, [1090]=0x63, [1091]=0x3d, [1093]=0x62, [1094]=0x6f, [1095]=0x6f, [1096]=0x74, [1097]=0x2d,
    [1098]=0x63, [1099]=0x6f, [1100]=0x6d, [1101]=0x6d, [1102]=0x61, [1103]=0x6e, [1104]=0x64, [1105]=0x3d,
    [1106]=0x6d, [1107]=0x61, [1108]=0x63, [1109]=0x2d, [1110]=0x62, [1111]=0x6f, [1112]=0x6f, [1113]=0x74,
    [1115]=0x64, [1116]=0x65, [1117]=0x66, [1118]=0x61, [1119]=0x75, [1120]=0x6c, [1121]=0x74, [1122]=0x2d,
    [1123]=0x63, [1124]=0x6c, [1125]=0x69, [1126]=0x65, [1127]=0x6e, [1128]=0x74, [1129]=0x2d, [1130]=0x69,
    [1131]=0x70, [1132]=0x3d, [1134]=0x64, [1135]=0x65, [1136]=0x66, [1137]=0x61, [1138]=0x75, [1139]=0x6c,
    [1140]=0x74, [1141]=0x2d, [1142]=0x73, [1143]=0x65, [1144]=0x72, [1145]=0x76, [1146]=0x65, [1147]=0x72,
    [1148]=0x2d, [1149]=0x69, [1150]=0x70, [1151]=0x3d, [1153]=0x64, [1154]=0x65, [1155]=0x66, [1156]=0x61,
    [1157]=0x75, [1158]=0x6c, [1159]=0x74, [1160]=0x2d, [1161]=0x67, [1162]=0x61, [1163]=0x74, [1164]=0x65,
    [1165]=0x77, [1166]=0x61, [1167]=0x79, [1168]=0x2d, [1169]=0x69, [1170]=0x70, [1171]=0x3d, [1173]=0x64,
    [1174]=0x65, [1175]=0x66, [1176]=0x61, [1177]=0x75, [1178]=0x6c, [1179]=0x74, [1180]=0x2d, [1181]=0x73,
    [1182]=0x75, [1183]=0x62, [1184]=0x6e, [1185]=0x65, [1186]=0x74, [1187]=0x2d, [1188]=0x6d, [1189]=0x61,
    [1190]=0x73, [1191]=0x6b, [1192]=0x3d, [1194]=0x64, [1195]=0x65, [1196]=0x66, [1197]=0x61, [1198]=0x75,
    [1199]=0x6c, [1200]=0x74, [1201]=0x2d, [1202]=0x72, [1203]=0x6f, [1204]=0x75, [1205]=0x74, [1206]=0x65,
    [1207]=0x72, [1208]=0x2d, [1209]=0x69, [1210]=0x70, [1211]=0x3d, [1213]=0x62, [1214]=0x6f, [1215]=0x6f,
    [1216]=0x74, [1217]=0x2d, [1218]=0x73, [1219]=0x63, [1220]=0x72, [1221]=0x69, [1222]=0x70, [1223]=0x74,
    [1224]=0x3d, [1226]=0x61, [1227]=0x61, [1228]=0x70, [1229]=0x6c, [1230]=0x2c, [1231]=0x70, [1232]=0x63,
    [1233]=0x69, [1234]=0x3d, [1236]=0x72, [1237]=0x61, [1238]=0x6d, [1239]=0x2d, [1240]=0x73, [1241]=0x69,
    [1242]=0x7a, [1243]=0x65, [1244]=0x3d, [1245]=0x30, [1246]=0x78, [1247]=0x34, [1248]=0x30, [1249]=0x30,
    [1250]=0x30, [1251]=0x30, [1252]=0x30, [1253]=0x30, [1254]=0x30, [4624]=0xa0, [4625]=0x1d, [4627]=0x51,
    [4628]=0x41, [4629]=0x50, [4630]=0x4c, [4631]=0x2c, [4632]=0x4d, [4633]=0x61, [4634]=0x63, [4635]=0x4f,
    [4636]=0x53, [4637]=0x37, [4638]=0x35, [5920]=0x7f, [5921]=0xa7, [5923]=0x8e, [5924]=0x77, [5925]=0x77,
    [5926]=0x77, [5927]=0x77, [5928]=0x77, [5929]=0x77, [5930]=0x77, [5931]=0x77, [5932]=0x77, [5933]=0x77,
    [5934]=0x77, [5935]=0x77,
};

/*
 * True if the flash is genuinely blank (all-0xff, real erased-flash state --
 * see macio_nvram_default_blk()'s own "must start out erased" comment).
 * Deliberately not macio_nvram_is_blank(), which checks for all-*zero*
 * bytes -- the correct blank state for the *non*-flash nvram.img path this
 * function is not used for.
 */
static bool core99_nvram_flash_is_blank(const uint8_t *data, uint32_t size)
{
    uint32_t i;

    for (i = 0; i < size; i++) {
        if (data[i] != 0xff) {
            return false;
        }
    }
    return true;
}

/*
 * Seed the real default structure into a blank flash nvram so the grafts
 * below always have a valid "common" partition to patch into, even on the
 * very first boot against a brand-new nvram-flash-*.img.
 */
static void core99_nvram_seed_if_blank(uint8_t *data, uint32_t size)
{
    if (size < 2 * CORE99_NVRAM_TEMPLATE_SIZE ||
        !core99_nvram_flash_is_blank(data, size)) {
        return;
    }
    memcpy(&data[CORE99_NVRAM_TEMPLATE_SIZE], core99_nvram_flash_template,
          CORE99_NVRAM_TEMPLATE_SIZE);
}

static uint8_t core99_chrp_cksum(const uint8_t *hdr)
{
    unsigned int sum = hdr[0];
    int i;

    for (i = 0; i < 14; i++) {
        sum += hdr[2 + i];
        sum = (sum + ((sum & 0xff00) >> 8)) & 0xff;
    }
    return sum & 0xff;
}

/*
 * Make the built-in gmac usable under Mac OS by publishing its MAC on the
 * Open Firmware ethernet node, and graft the missing L2-cache device-tree
 * data so Mac OS stops reporting a cache fault.
 *
 * Mac OS (classic AppleTalk/OT and OS X's AppleGMACEthernet) reads the
 * Ethernet MAC solely from the OF "local-mac-address" property and will not
 * create en0 without it; a "built-in" property additionally makes it show as
 * "Built-in Ethernet" rather than a PCI slot.  The real Apple ROM's own gmac
 * driver never publishes these, so they are injected through the OF
 * "boot-command" variable, honouring whatever MAC the gmac was given on the
 * command line (or QEMU's default when none was specified).
 *
 * A real PowerMac3,4 publishes /cpus/PowerPC,G4@0/l2cr = 0xb9000000 and a
 * child "l2-cache" node (1MB unified, 64-byte lines, 0x2000 sets, 233MHz --
 * verbatim from a real PowerMac3,4 lsprop dump).  With the 74xx L2CR/HID0
 * read-side fixes (repo-lineage/10) the CPU now reports sane values, but the
 * ROM's own L2 bring-up/POST still never runs on QEMU's transparent memory
 * model, so the device tree ends up with neither property and
 * /diagnostics/post-results (the POST failure bitmask a real healthy Mac
 * reads back as 0x00000000) is left non-zero -- which is what Mac OS
 * surfaces as "a problem with cache memory".  Graft both: the l2cr/l2-cache
 * data a real machine has, and a zeroed post-results, so Mac OS sees exactly
 * what a passing POST would have produced.
 *
 * boot-command (not nvramrc) is used deliberately: nvramrc runs before
 * probe-all, when /pci@f4000000/ethernet@f does not yet exist; boot-command
 * runs at the end of start-up, after the PCI ethernet node has been probed
 * and after /diagnostics/post-results has already been written by the ROM's
 * own (failing) POST, so it is the right point to overwrite both.
 *
 * The nvram is Apple/CHRP format: an 8KB image mirrored inside the 16KB
 * flash, OF variables stored as name=value NUL strings in the "common"
 * partition.  Every edit must fix both the per-partition header checksum and
 * the outer Adler32 (at copy+0x10, over [copy+0x14 : copy+0x2000]); without
 * the latter the ROM rejects the image ("NVRAM corrupted") and wipes it.
 */
static void core99_nvram_set_bootcmd(uint8_t *data, uint32_t size,
                                     const uint8_t *mac, uint32_t l2clock)
{
    const uint32_t COPY = 0x2000;
    /* gmac graft, only when a MAC is supplied (i.e. a sungem is present) */
    g_autofree char *gmac = mac ? g_strdup_printf(
        "dev /pci@f4000000/ethernet@f "
        "here h# %02x over c! 1+ h# %02x over c! 1+ h# %02x over c! 1+ "
        "h# %02x over c! 1+ h# %02x over c! 1+ h# %02x swap c! "
        "here 6 encode-bytes \" local-mac-address\" property "
        "0 0 encode-bytes \" built-in\" property device-end ",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]) : g_strdup("");
    /*
     * The l2-cache node's own clock-frequency is not a constant: L2CR
     * 0xb9000000 has L2CLK = divide-by-2, so the backside cache runs at half
     * the core clock and the property has to follow whatever that is.  The
     * donor PowerMac3,4's 233MHz is exactly 466.67/2; raise the core and this
     * must move with it or the grafted node contradicts the l2cr beside it.
     */
    g_autofree char *l2clk = g_strdup_printf(
        "h# %x encode-int \" clock-frequency\" property ", l2clock);
    g_autofree char *bootcmd = g_strconcat("boot-command=", gmac,
        "dev /cpus/PowerPC,G4@0 "
        "h# b9000000 encode-int \" l2cr\" property "
        "new-device \" l2-cache\" device-name \" cache\" device-type ",
        l2clk,
        "0 0 encode-bytes \" cache-unified\" property "
        "h# 40 encode-int \" d-cache-line-size\" property "
        "h# 40 encode-int \" i-cache-line-size\" property "
        "h# 2000 encode-int \" d-cache-sets\" property "
        "h# 2000 encode-int \" i-cache-sets\" property "
        "h# 100000 encode-int \" d-cache-size\" property "
        "h# 100000 encode-int \" i-cache-size\" property "
        "finish-device device-end "
        "dev /diagnostics 0 encode-int \" post-results\" property "
        "device-end mac-boot", NULL);
    uint32_t base;

    for (base = 0; base + COPY <= size; base += COPY) {
        uint32_t off;

        /* Replace boot-command in this copy's "common" partition. */
        for (off = base; off + 16 <= base + COPY; ) {
            uint32_t plen = (((data[off + 2] << 8) | data[off + 3]) * 16);

            if (plen == 0) {
                break;
            }
            if (data[off] == 0x70 &&
                memcmp(&data[off + 4], "common", 6) == 0) {
                uint32_t dstart = off + 16, dlen = plen - 16, i;
                GString *vars = g_string_new(NULL);
                bool seen = false;

                for (i = dstart; i < dstart + dlen && data[i]; ) {
                    const char *v = (const char *)&data[i];
                    size_t vl = strnlen(v, dstart + dlen - i);

                    if (!strncmp(v, "boot-command=", 13)) {
                        g_string_append(vars, bootcmd);
                        seen = true;
                    } else {
                        g_string_append_len(vars, v, vl);
                    }
                    g_string_append_c(vars, '\0');
                    i += vl + 1;
                }
                if (!seen) {
                    g_string_append(vars, bootcmd);
                    g_string_append_c(vars, '\0');
                }
                g_string_append_c(vars, '\0');   /* end-of-list marker */

                if (vars->len <= dlen) {
                    memset(&data[dstart], 0, dlen);
                    memcpy(&data[dstart], vars->str, vars->len);
                    data[off + 1] = core99_chrp_cksum(&data[off]);
                } else {
                    warn_report("nvram: vars too large, "
                                "boot-command graft not applied");
                }
                g_string_free(vars, TRUE);
                break;
            }
            off += plen;
        }

        /* Fix the outer Adler32 in this copy's "nvram" (0x5a) header. */
        if (data[base] == 0x5a && !memcmp(&data[base + 4], "nvram", 5)) {
            /* Standard Adler-32 (initial value 1), as the real Apple ROM uses */
            uint32_t a = adler32(1, &data[base + 0x14], COPY - 0x14);

            stl_be_p(&data[base + 0x10], a);
        }
    }
}

/*
 * Seed a blank flash image and graft the boot-command.  The FireWire GUID and
 * AGP-slot-enable grafts of the source branch are PM34-specific (they need
 * the firewire stub and the AGP work); the cache and gmac grafts are the two
 * that apply to any mac99.
 */
static void core99_nvram_graft(MacIONVRAMState *nvr, uint32_t l2clock)
{
    Object *o = object_resolve_path_type("", "sungem", NULL);
    g_autofree char *macstr = o ? object_property_get_str(o, "mac", NULL)
                                : NULL;
    uint8_t mac[6];
    bool have_mac = false;

    if (!nvr->data) {
        return;
    }
    if (macstr && sscanf(macstr, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
                         &mac[0], &mac[1], &mac[2], &mac[3], &mac[4],
                         &mac[5]) == 6) {
        have_mac = true;
    }
    core99_nvram_seed_if_blank(nvr->data, nvr->size);
    /* L2CR 0xb9000000 is divide-by-2: half the core clock, per model */
    core99_nvram_set_bootcmd(nvr->data, nvr->size, have_mac ? mac : NULL,
                             l2clock);
}


static void ppc_core99_reset(void *opaque)
{
    PowerPCCPU *cpu = opaque;
    CPUState *cs = CPU(cpu);

    cpu_ppc_tb_reset(&cpu->env);
    cpu_reset(cs);
    /* 970 CPUs want to get their initial IP as part of their boot protocol */
    cpu->env.nip = PROM_BASE + 0x100;
    
    /* Keep secondary CPUs halted until explicitly started via GPIO */
    if (cs->cpu_index > 0) {
        cs->halted = 1;
    }
}

static void cpu_kick(void *opaque, int n, int level)
{
    PowerPCCPU *cpu = opaque;
    CPUState *cs = CPU(cpu);
    CPUState *primary_cs;
    PowerPCCPU *primary_cpu;

    /* 
     * GPIO reset lines are active LOW:
     * - GPIO=0 (level=1 from IRQ raise): CPU held in reset
     * - GPIO=1 (level=0 from IRQ lower): CPU released from reset
     * 
     * We start the CPU when reset is DEASSERTED (level=0)
     */
    if (!level && cs->halted) {
        /*
         * Re-sync timebase offset with primary CPU before starting.
         * The offsets were initially synced at machine init, but we
         * re-sync here in case there was any drift while halted.
         */
        primary_cs = first_cpu;
        if (primary_cs && primary_cs != cs) {
            primary_cpu = POWERPC_CPU(primary_cs);
            cpu->env.tb_env->tb_offset = primary_cpu->env.tb_env->tb_offset;
            cpu->env.tb_env->atb_offset = primary_cpu->env.tb_env->atb_offset;
        }
        
        /*
         * When the OS wakes a secondary CPU via GPIO reset deassertion,
         * the CPU should start executing at the reset vector (0x100).
         * The OS has placed a spin loop there that reads a mailbox
         * to find the real entry point.
         */
        
        /* Clear exception prefix - use low memory reset vector */
        cpu->env.excp_prefix = 0;
        
        /* Set up CPU to start at reset vector in RAM */
        cpu->env.nip = 0x100;
        
        /* Clear MSR to minimal supervisor state */
        cpu->env.msr = 0;
        
        /* Unhalt the CPU */
        cs->halted = 0;
        cs->exception_index = -1;
        
        /* Kick the CPU thread to start executing */
        qemu_cpu_kick(cs);
    }
}

/* OpenBIOS is an ELF; a real Apple ROM is a flat binary image. */
static bool firmware_is_elf(const char *filename)
{
    uint8_t magic[4];
    bool is_elf = false;
    FILE *f = fopen(filename, "rb");

    if (f) {
        is_elf = fread(magic, 1, sizeof(magic), f) == sizeof(magic) &&
                 !memcmp(magic, ELFMAG, sizeof(magic));
        fclose(f);
    }
    return is_elf;
}

static void macio_map_at_hw_base(void *opaque)
{
    PCIDevice *macio = opaque;

    pci_default_write_config(macio, PCI_BASE_ADDRESS_0, MACIO_BASE, 4);
    pci_default_write_config(macio, PCI_COMMAND,
                             PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER, 2);
}

typedef struct MacIOHwBaseNotifier {
    Notifier notifier;
    PCIDevice *macio;
} MacIOHwBaseNotifier;

static void macio_arm_hw_base_reset(Notifier *n, void *opaque)
{
    MacIOHwBaseNotifier *mn = container_of(n, MacIOHwBaseNotifier, notifier);

    /*
     * Deferred to machine-init-done on purpose: the sysbus (and with it the
     * whole device tree) only becomes resettable at the end of machine
     * creation, so a handler registered during ppc_core99_init() would run
     * *before* MacIO's own reset and have its config write thrown away.
     */
    qemu_register_reset(macio_map_at_hw_base, mn->macio);
}

/* PowerPC Mac99 hardware initialisation */
static void ppc_core99_init(MachineState *machine)
{
    Core99MachineState *core99_machine = CORE99_MACHINE(machine);
    MachineClass *mc = MACHINE_GET_CLASS(machine);
    qemu_irq kick;
    PowerPCCPU **cpus = NULL;
    CPUPPCState *env = NULL;
    char *filename;
    IrqLines *openpic_irqs;
    int i, j, k, ppc_boot_device, machine_arch, bios_size = -1;
    const char *bios_name = machine->firmware ?: PROM_FILENAME;
    MemoryRegion *bios = g_new(MemoryRegion, 1);
    MemoryRegion *rom_chip = g_new(MemoryRegion, 1);
    bool rom_is_flash;
    hwaddr kernel_base = 0, initrd_base = 0, cmdline_base = 0;
    long kernel_size = 0, initrd_size = 0;
    PCIBus *pci_bus;
    bool has_pmu, has_adb;
    Object *macio;
    MACIOIDEState *macio_ide;
    BusState *adb_bus;
    MacIONVRAMState *nvr;
    DriveInfo *dinfo, *hd[MAX_IDE_BUS * MAX_IDE_DEVS];
    void *fw_cfg;
    SysBusDevice *s;
    DeviceState *dev, *pic_dev, *uninorth_pci_dev;
    DeviceState *uninorth_internal_dev = NULL, *uninorth_agp_dev = NULL;
    MemoryRegion *macio_win;
    I2CBus *unin_i2c;
    MacIOHwBaseNotifier *macio_notifier;
    hwaddr nvram_addr = 0xFFF04000;
    uint64_t tbfreq = kvm_enabled() ? kvmppc_get_tbfreq()
                                    : core99_tbfreq(core99_machine);

    /* init CPUs */
    cpus = g_new0(PowerPCCPU *, machine->smp.cpus);
    for (i = 0; i < machine->smp.cpus; i++) {
        cpus[i] = POWERPC_CPU(cpu_create(machine->cpu_type));
        
        /* Each CPU needs its own tb_env for decrementer (per-CPU timer) */
        cpu_ppc_tb_init(&cpus[i]->env,
                        PPC_INPUT(&cpus[i]->env) == PPC_FLAGS_INPUT_970
                            ? U3_TBFREQ : core99_tbfreq(core99_machine));

        if (PPC_INPUT(&cpus[i]->env) != PPC_FLAGS_INPUT_970) {
            core99_cpu_defaults(core99_machine, cpus[i]);
        }
        
        /*
         * Secondary CPUs: sync timebase offset with primary CPU.
         * On real PowerPC hardware, the timebase counter is shared,
         * but each CPU has its own decrementer. We emulate this by
         * giving each CPU its own tb_env but with the same tb_offset.
         */
        if (i > 0) {
            cpus[i]->env.tb_env->tb_offset = cpus[0]->env.tb_env->tb_offset;
            cpus[i]->env.tb_env->atb_offset = cpus[0]->env.tb_env->atb_offset;
        }
        
        qemu_register_reset(ppc_core99_reset, cpus[i]);
        
        /* Secondary CPUs start halted - they will be kicked via GPIO */
        if (i > 0) {
            CPU(cpus[i])->halted = 1;
        }
    }
    env = &cpus[0]->env;

    /* allocate RAM */
    if (machine->ram_size > 2 * GiB) {
        error_report("RAM size more than 2 GiB is not supported");
        exit(1);
    }
    memory_region_add_subregion(get_system_memory(), 0, machine->ram);

    filename = qemu_find_file(QEMU_FILE_TYPE_BIOS, bios_name);
    rom_is_flash = filename && !firmware_is_elf(filename);
    core99_machine->apple_rom = rom_is_flash;

    /*
     * A real PowerMac3,4/3,6 has a PMU, never a CUDA, and a real Apple ROM
     * depends on it for more than the VIA itself: the KeyLargo GPIO block
     * only exists alongside the PMU. Left on the OpenBIOS-era CUDA default,
     * the ROM's POST reads the programmer's switch line from unmapped space,
     * gets 0 -- "switch held down at power-on" -- and enters firmware-update
     * mode, which never boots. An explicit via= still wins.
     */
    if (rom_is_flash && core99_machine->via_config == CORE99_VIA_CONFIG_CUDA) {
        core99_machine->via_config = CORE99_VIA_CONFIG_PMU;
    }

    /*
     * A ROM *device* rather than plain ROM so that guest stores into the
     * boot-flash range are visible (traced) instead of silently dropped:
     * the real part is one flash chip, and a guest flash driver may issue
     * its command/ID cycles anywhere in it, not only inside the 16KB
     * NVRAM window that mac_nvram.c models. Reads still come straight
     * from the RAM backing.
     */
    memory_region_init_rom_device(bios, NULL, &core99_rom_ops, NULL,
                                  "ppc_core99.bios", PROM_SIZE,
                                  &error_fatal);
    /*
     * "rom_chip" is the whole 1MB flash part: the ROM image with, on the
     * Apple ROM machines, the 16KB NVRAM window overlaid at 0x4000 (added
     * below once the nvram device exists). Everything that decodes the
     * part -- the primary slot at PROM_BASE and every alias -- goes
     * through this container, so the NVRAM shows up in every alias too.
     */
    memory_region_init(rom_chip, NULL, "ppc_core99.rom-chip", PROM_SIZE);
    memory_region_add_subregion(rom_chip, 0, bios);
    memory_region_add_subregion(get_system_memory(), PROM_BASE, rom_chip);

    /*
     * The ROM socket decodes ROM_WINDOW_SIZE at ROM_WINDOW_BASE, but only
     * the top 1MB is populated, so the part aliases through the rest of the
     * window. The real Apple ROM makes use of that: after its low-level init
     * it jumps to code via a window-relative address such as 0xff80b644,
     * which on hardware is simply offset 0xb644 of the same flash part --
     * and Mac OS 9's "nvram,flash" native driver (Mac OS ROM 8.x parcel
     * nvram,flash-1.0d1) reads and programs the two NVRAM banks at
     * 0xff004000/0xff006000: the same 0x4000/0x6000 offsets of the part,
     * decoded 16MB below the top. With only 8MB mirrored it read zeros
     * there, found no 0x5a bank signature, and silently never wrote --
     * Startup Disk / PRAM changes were lost on every restart.
     */
    for (i = 0; i < ROM_WINDOW_SIZE / PROM_SIZE - 1; i++) {
        MemoryRegion *mirror = g_new(MemoryRegion, 1);
        g_autofree char *name = g_strdup_printf("ppc_core99.bios.mirror%d", i);

        memory_region_init_alias(mirror, NULL, name, rom_chip, 0, PROM_SIZE);
        memory_region_add_subregion(get_system_memory(),
                                    ROM_WINDOW_BASE + (hwaddr)i * PROM_SIZE,
                                    mirror);
    }

    if (filename) {
        /* Load OpenBIOS (ELF) */
        bios_size = load_elf(filename, NULL, NULL, NULL, NULL,
                             NULL, NULL, NULL,
                             ELFDATA2MSB, PPC_ELF_MACHINE, 0, 0);

        if (bios_size <= 0) {
            /* or load binary ROM image */
            bios_size = load_image_targphys(filename, PROM_BASE, PROM_SIZE,
                                            &error_fatal);
        }
        g_free(filename);
    }
    if (bios_size < 0 || bios_size > PROM_SIZE) {
        error_report("could not load PowerPC bios '%s'", bios_name);
        exit(1);
    }

    if (machine->kernel_filename) {
        kernel_base = KERNEL_LOAD_ADDR;
        kernel_size = load_elf(machine->kernel_filename, NULL,
                               translate_kernel_address, NULL, NULL, NULL,
                               NULL, NULL, ELFDATA2MSB, PPC_ELF_MACHINE, 0, 0);
        if (kernel_size < 0) {
            kernel_size = load_aout(machine->kernel_filename, kernel_base,
                                    machine->ram_size - kernel_base,
                                    true, TARGET_PAGE_SIZE);
        }
        if (kernel_size < 0) {
            kernel_size = load_image_targphys(machine->kernel_filename,
                                              kernel_base,
                                              machine->ram_size - kernel_base,
                                              &error_fatal);
        }
        /* load initrd */
        if (machine->initrd_filename) {
            initrd_base = TARGET_PAGE_ALIGN(kernel_base + kernel_size + KERNEL_GAP);
            initrd_size = load_image_targphys(machine->initrd_filename,
                                              initrd_base,
                                              machine->ram_size - initrd_base,
                                              &error_fatal);
            cmdline_base = TARGET_PAGE_ALIGN(initrd_base + initrd_size);
        } else {
            cmdline_base = TARGET_PAGE_ALIGN(kernel_base + kernel_size + KERNEL_GAP);
        }
        ppc_boot_device = 'm';
    } else {
        ppc_boot_device = '\0';
        /* We consider that NewWorld PowerMac never have any floppy drive
         * For now, OHW cannot boot from the network.
         */
        for (i = 0; machine->boot_config.order[i] != '\0'; i++) {
            if (machine->boot_config.order[i] >= 'c' &&
                machine->boot_config.order[i] <= 'f') {
                ppc_boot_device = machine->boot_config.order[i];
                break;
            }
        }
        if (ppc_boot_device == '\0') {
            error_report("No valid boot device for Mac99 machine");
            exit(1);
        }
    }

    openpic_irqs = g_new0(IrqLines, machine->smp.cpus);
    for (i = 0; i < machine->smp.cpus; i++) {
        dev = DEVICE(cpus[i]);
        /* Mac99 IRQ connection between OpenPIC outputs pins and PowerPC input pins */
        switch (PPC_INPUT(env)) {
        case PPC_FLAGS_INPUT_6xx:
            openpic_irqs[i].irq[OPENPIC_OUTPUT_INT] =
                qdev_get_gpio_in(dev, PPC6xx_INPUT_INT);
            openpic_irqs[i].irq[OPENPIC_OUTPUT_CINT] =
                qdev_get_gpio_in(dev, PPC6xx_INPUT_INT);
            openpic_irqs[i].irq[OPENPIC_OUTPUT_MCK] =
                qdev_get_gpio_in(dev, PPC6xx_INPUT_MCP);
            openpic_irqs[i].irq[OPENPIC_OUTPUT_DEBUG] = NULL;
            openpic_irqs[i].irq[OPENPIC_OUTPUT_RESET] =
                qdev_get_gpio_in(dev, PPC6xx_INPUT_HRESET);
            break;
#if defined(TARGET_PPC64)
        case PPC_FLAGS_INPUT_970:
            openpic_irqs[i].irq[OPENPIC_OUTPUT_INT] =
                qdev_get_gpio_in(dev, PPC970_INPUT_INT);
            openpic_irqs[i].irq[OPENPIC_OUTPUT_CINT] =
                qdev_get_gpio_in(dev, PPC970_INPUT_INT);
            openpic_irqs[i].irq[OPENPIC_OUTPUT_MCK] =
                qdev_get_gpio_in(dev, PPC970_INPUT_MCP);
            openpic_irqs[i].irq[OPENPIC_OUTPUT_DEBUG] = NULL;
            openpic_irqs[i].irq[OPENPIC_OUTPUT_RESET] =
                qdev_get_gpio_in(dev, PPC970_INPUT_HRESET);
            break;
#endif
        default:
            error_report("Bus model not supported on mac99 machine");
            exit(1);
        }
    }

    /* UniN init */
    s = SYS_BUS_DEVICE(qdev_new(TYPE_UNI_NORTH));
    sysbus_realize_and_unref(s, &error_fatal);
    memory_region_add_subregion(get_system_memory(), 0xf8000000,
                                sysbus_mmio_get_region(s, 0));

    /*
     * UniNorth's own I2C controller sits inside that 16MB window (at
     * +0x1000), so it needs the higher priority to win the overlap. Its bus
     * reaches the DIMM SPD EEPROMs and the processor module's configuration
     * EEPROM, which a real Apple ROM reads during early bring-up; OpenBIOS
     * never touches it.
     */
    memory_region_add_subregion_overlap(get_system_memory(), UNINORTH_I2C_BASE,
                                        sysbus_mmio_get_region(s, 1), 1);

    /*
     * The processor module carries an SPD-format configuration EEPROM. The
     * Tangent schematic labels it "ADDRESS = 6 (AC/AD)" next to the module
     * connector's I2C pins, i.e. 8-bit 0xac/0xad, 7-bit 0x56. A real Apple
     * ROM reads it during bring-up and refuses to continue without it.
     */
    unin_i2c = UNI_NORTH(s)->i2c.bus;
    at24c_eeprom_init_rom(unin_i2c, 0x56, 256, NULL, 0);

    core99_add_spd_dimms(core99_machine, unin_i2c, machine->ram_size);

    if (PPC_INPUT(env) == PPC_FLAGS_INPUT_970) {
        machine_arch = ARCH_MAC99_U3;
        /* 970 gets a U3 bus */
        /* Uninorth AGP bus */
        uninorth_pci_dev = qdev_new(TYPE_U3_AGP_HOST_BRIDGE);
        s = SYS_BUS_DEVICE(uninorth_pci_dev);
        sysbus_realize_and_unref(s, &error_fatal);
        sysbus_mmio_map(s, 0, 0xf0800000);
        sysbus_mmio_map(s, 1, 0xf0c00000);
        /* PCI hole */
        memory_region_add_subregion(get_system_memory(), 0x80000000,
                                    sysbus_mmio_get_region(s, 2));
        /* Register 8 MB of ISA IO space */
        memory_region_add_subregion(get_system_memory(), 0xf2000000,
                                    sysbus_mmio_get_region(s, 3));
    } else {
        machine_arch = ARCH_MAC99;
        /* Use values found on a real PowerMac */
        /* Uninorth AGP bus */
        uninorth_agp_dev = qdev_new(TYPE_UNI_NORTH_AGP_HOST_BRIDGE);
        qdev_prop_set_bit(uninorth_agp_dev, "real-irq-map", rom_is_flash);
        s = SYS_BUS_DEVICE(uninorth_agp_dev);
        sysbus_realize_and_unref(s, &error_fatal);
        sysbus_mmio_map(s, 0, 0xf0800000);
        sysbus_mmio_map(s, 1, 0xf0c00000);
        /*
         * AGP bus memory window (see pci_unin_agp_init()).  A card in the AGP
         * slot gets its BARs assigned in here, so without this mapping the CPU
         * cannot reach them at all and the display never lights up.  The I/O
         * space matches the real /pci@f0000000 ranges, at low overlap priority
         * so the fw_cfg registers mapped inside it keep their bytes.
         */
        sysbus_mmio_map(s, 2, 0x90000000);
        memory_region_add_subregion_overlap(get_system_memory(), 0xf0000000,
                                            sysbus_mmio_get_region(s, 3), -1);
        core99_machine->agp_bus = PCI_HOST_BRIDGE(uninorth_agp_dev)->bus;

        /* Uninorth internal bus */
        uninorth_internal_dev = qdev_new(
                                TYPE_UNI_NORTH_INTERNAL_PCI_HOST_BRIDGE);
        qdev_prop_set_bit(uninorth_internal_dev, "real-irq-map", rom_is_flash);
        qdev_prop_set_bit(uninorth_internal_dev, "no-self-func", true);
        s = SYS_BUS_DEVICE(uninorth_internal_dev);
        sysbus_realize_and_unref(s, &error_fatal);
        sysbus_mmio_map(s, 0, 0xf4800000);
        sysbus_mmio_map(s, 1, 0xf4c00000);
        /* The real pci@f4000000's one memory window (its ranges prop) */
        sysbus_mmio_map(s, 2, 0xf5000000);

        /* Uninorth main bus - this must be last to make it the default */
        uninorth_pci_dev = qdev_new(TYPE_UNI_NORTH_PCI_HOST_BRIDGE);
        qdev_prop_set_uint32(uninorth_pci_dev, "ofw-addr", 0xf2000000);
        qdev_prop_set_bit(uninorth_pci_dev, "real-irq-map", rom_is_flash);
        s = SYS_BUS_DEVICE(uninorth_pci_dev);
        sysbus_realize_and_unref(s, &error_fatal);
        sysbus_mmio_map(s, 0, 0xf2800000);
        sysbus_mmio_map(s, 1, 0xf2c00000);
        /* PCI hole */
        memory_region_add_subregion(get_system_memory(), 0x80000000,
                                    sysbus_mmio_get_region(s, 2));
        /* Register 8 MB of ISA IO space */
        memory_region_add_subregion(get_system_memory(), 0xf2000000,
                                    sysbus_mmio_get_region(s, 3));
    }

    machine->usb |= defaults_enabled() && !machine->usb_disabled;
    has_pmu = (core99_machine->via_config != CORE99_VIA_CONFIG_CUDA);
    has_adb = (core99_machine->via_config == CORE99_VIA_CONFIG_CUDA ||
               core99_machine->via_config == CORE99_VIA_CONFIG_PMU_ADB);

    /* init basic PC hardware */
    pci_bus = PCI_HOST_BRIDGE(uninorth_pci_dev)->bus;

    /* MacIO */
    /*
     * Slot numbers matter to a real Apple ROM, which expects the built-in
     * devices where the hardware puts them: the PowerMac3,4 device tree has
     * mac-io@17 and usb@18 / usb@19 on this bus.
     */
    macio = OBJECT(pci_new(PCI_DEVFN(0x17, 0), TYPE_NEWWORLD_MACIO));
    dev = DEVICE(macio);
    qdev_prop_set_uint64(dev, "frequency", tbfreq);
    qdev_prop_set_bit(dev, "has-pmu", has_pmu);
    qdev_prop_set_bit(dev, "has-adb", has_adb);

    dev = DEVICE(object_resolve_path_component(macio, "escc"));
    qdev_prop_set_chr(dev, "chrA", serial_hd(0));
    qdev_prop_set_chr(dev, "chrB", serial_hd(1));

    pic_dev = DEVICE(object_resolve_path_component(macio, "pic"));
    qdev_prop_set_uint32(pic_dev, "nb_cpus", machine->smp.cpus);

    pci_realize_and_unref(PCI_DEVICE(macio), pci_bus, &error_fatal);

    /*
     * Have MacIO come out of reset already decoding at the address real
     * hardware uses, instead of leaving it to firmware. Registered as a
     * reset handler because PCI config space (including BARs) is cleared when
     * the device is reset, which happens after machine init.
     *
     * OpenBIOS enumerates the PCI bus and assigns BARs itself, so it never
     * needed this -- and the notifier arms a *reset* handler, so arming it
     * unconditionally would rewrite the BAR on every later reset as well,
     * behind the back of firmware that had legitimately assigned it
     * elsewhere (that is what broke the Mac OS 9 OpenBIOS boot in 3d1a4add).
     * A real Apple boot ROM does not enumerate: its early hardware-init code
     * reaches straight for KeyLargo at the hardwired 0xf3000000 and spins
     * forever if nothing answers.
     */
    if (rom_is_flash) {
        macio_notifier = g_new0(MacIOHwBaseNotifier, 1);
        macio_notifier->notifier.notify = macio_arm_hw_base_reset;
        macio_notifier->macio = PCI_DEVICE(macio);
        qemu_add_machine_init_done_notifier(&macio_notifier->notifier);

        /*
         * UniNorth's only CPU-visible window into PCI memory space is the
         * 256MB "PCI hole" at 0x80000000, so a BAR at 0xf3000000 would decode
         * on the bus but be unreachable from the CPU. Real UniNorth also
         * forwards the 0xf3000000 range, which is where MacIO lives; add that
         * window so the BAR above is actually usable.
         */
        macio_win = g_new(MemoryRegion, 1);
        memory_region_init_alias(macio_win, macio, "unin-pci-macio-hole",
                                 pci_address_space(PCI_DEVICE(macio)),
                                 MACIO_BASE, MACIO_WIN_SIZE);
        memory_region_add_subregion(get_system_memory(), MACIO_BASE, macio_win);
    }

    /* Connect GPIO reset IRQs for secondary CPUs */
    s = SYS_BUS_DEVICE(object_resolve_path_component(macio, "gpio"));
    if (machine->smp.cpus > 1) {
        /* CPU1 reset: use GPIO 4 */
        kick = qemu_allocate_irq(cpu_kick, cpus[1], 1);
        sysbus_connect_irq(s, 4, kick);
    }
    if (machine->smp.cpus > 2) {
        /* CPU2 reset: use GPIO 15 */
        kick = qemu_allocate_irq(cpu_kick, cpus[2], 2);
        sysbus_connect_irq(s, 15, kick);
    }
    if (machine->smp.cpus > 3) {
        /* CPU3 reset: use GPIO 16 */
        kick = qemu_allocate_irq(cpu_kick, cpus[3], 3);
        sysbus_connect_irq(s, 16, kick);
    }
    /*
     * The Apple ROM hands the guest the real machine's interrupt maps, so in
     * that mode the slots interrupt where those tables say they do; OpenBIOS
     * builds its own tree around the legacy four-line hash.
     */
    if (rom_is_flash) {
        static const int real_pci_irqs[7] = {
            0x34, 0x35, 0x36, 0x3a, 0x1b, 0x1c, 0x3f
        };

        for (i = 0; i < ARRAY_SIZE(real_pci_irqs); i++) {
            qdev_connect_gpio_out(uninorth_pci_dev, i,
                                  qdev_get_gpio_in(pic_dev,
                                                   real_pci_irqs[i]));
        }
    } else {
        for (i = 0; i < 4; i++) {
            qdev_connect_gpio_out(uninorth_pci_dev, i,
                                  qdev_get_gpio_in(pic_dev, 0x1b + i));
        }
    }

    /* TODO: additional PCI buses only wired up for 32-bit machines */
    if (PPC_INPUT(env) != PPC_FLAGS_INPUT_970) {
        /* Uninorth AGP bus */
        if (rom_is_flash) {
            /* The AGP slot (device 0x10) is the real map's only entry */
            qdev_connect_gpio_out(uninorth_agp_dev, 0,
                                  qdev_get_gpio_in(pic_dev, 0x30));
        } else {
            for (i = 0; i < 4; i++) {
                qdev_connect_gpio_out(uninorth_agp_dev, i,
                                      qdev_get_gpio_in(pic_dev, 0x1b + i));
            }
        }

        /* Uninorth internal bus */
        if (rom_is_flash) {
            /* Real pci@f4000000 map: slots 0x0e/0x0f */
            qdev_connect_gpio_out(uninorth_internal_dev, 0,
                                  qdev_get_gpio_in(pic_dev, 0x28));
            qdev_connect_gpio_out(uninorth_internal_dev, 1,
                                  qdev_get_gpio_in(pic_dev, 0x29));
        } else {
            for (i = 0; i < 4; i++) {
                qdev_connect_gpio_out(uninorth_internal_dev, i,
                                      qdev_get_gpio_in(pic_dev, 0x1b + i));
            }
        }
    }

    /* OpenPIC */
    s = SYS_BUS_DEVICE(pic_dev);
    k = 0;
    for (i = 0; i < machine->smp.cpus; i++) {
        for (j = 0; j < OPENPIC_OUTPUT_NB; j++) {
            sysbus_connect_irq(s, k++, openpic_irqs[i].irq[j]);
        }
    }
    g_free(openpic_irqs);

    /* We only emulate 2 out of 3 IDE controllers for now */
    ide_drive_get(hd, ARRAY_SIZE(hd));

    macio_ide = MACIO_IDE(object_resolve_path_component(macio, "ide[0]"));
    macio_ide_init_drives(macio_ide, hd);

    macio_ide = MACIO_IDE(object_resolve_path_component(macio, "ide[1]"));
    macio_ide_init_drives(macio_ide, &hd[MAX_IDE_DEVS]);

    if (has_adb) {
        if (has_pmu) {
            dev = DEVICE(object_resolve_path_component(macio, "pmu"));
        } else {
            dev = DEVICE(object_resolve_path_component(macio, "cuda"));
        }

        adb_bus = qdev_get_child_bus(dev, "adb.0");
        dev = qdev_new(TYPE_ADB_KEYBOARD);
        qdev_realize_and_unref(dev, adb_bus, &error_fatal);

        dev = qdev_new(TYPE_ADB_MOUSE);
        qdev_realize_and_unref(dev, adb_bus, &error_fatal);
    }

    if (machine->usb) {
        pci_create_simple(pci_bus, rom_is_flash ? PCI_DEVFN(0x18, 0) : -1,
                          "pci-ohci");

        /* U3 needs to use USB for input because Linux doesn't support via-cuda
        on PPC64 */
        if (!has_adb || machine_arch == ARCH_MAC99_U3) {
            USBBus *usb_bus;

            usb_bus = USB_BUS(object_resolve_type_unambiguous(TYPE_USB_BUS,
                                                              &error_abort));
            usb_create_simple(usb_bus, "usb-kbd");
            usb_create_simple(usb_bus, "usb-mouse");
        }
    }

    pci_vga_init(pci_bus);

    if (!graphic_width) {
        graphic_width = 800;
    }
    if (!graphic_height) {
        graphic_height = 600;
    }
    if (!graphic_depth) {
        graphic_depth = 32;
    }
    if (graphic_depth != 15 && graphic_depth != 32 && graphic_depth != 8) {
        graphic_depth = 15;
    }

    pci_init_nic_devices(pci_bus, mc->default_nic);

    /* The NewWorld NVRAM is not located in the MacIO device */
    if (kvm_enabled() && qemu_real_host_page_size() > 4096) {
        /* We can't combine read-write and read-only in a single page, so
           move the NVRAM out of ROM again for KVM */
        nvram_addr = 0xFFE00000;
    }
    dev = qdev_new(TYPE_MACIO_NVRAM);
    if (rom_is_flash) {
        /* Flat bytes, flash command set, and erased rather than zeroed. */
        qdev_prop_set_uint32(dev, "size", NVRAM_FLASH_SIZE);
        qdev_prop_set_uint32(dev, "it_shift", 0);
        qdev_prop_set_bit(dev, "flash", true);
    } else {
        qdev_prop_set_uint32(dev, "size", MACIO_NVRAM_SIZE);
        qdev_prop_set_uint32(dev, "it_shift", 1);
    }

    /*
     * Back the NVRAM with a file so that what firmware stores in it (the OF
     * variables, boot-device among them) survives a restart: the user's own
     * -drive if=mtd,... if there is one, otherwise a default image in the
     * working directory, the same convention g3beige uses. Without this the
     * contents were re-stamped from scratch on every run. The flash image
     * must start out erased (all ones), not zeroed: firmware tells "never
     * written" from "written and invalid", and a block of zeroes is neither.
     */
    dinfo = drive_get(IF_MTD, 0, 0);
    if (dinfo) {
        qdev_prop_set_drive(dev, "drive", blk_by_legacy_dinfo(dinfo));
    } else {
        /* Per-model filename: their contents are model-specific */
        BlockBackend *nvram_blk = rom_is_flash ?
            macio_nvram_default_blk(core99_model_info(core99_machine)
                                        ->nvram_flash_file,
                                    NVRAM_FLASH_SIZE, 0xff) :
            macio_nvram_default_blk("nvram.img", MACIO_NVRAM_SIZE, 0);

        if (nvram_blk) {
            qdev_prop_set_drive(dev, "drive", nvram_blk);
        }
    }

    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    if (rom_is_flash && nvram_addr == 0xFFF04000) {
        /* Part of the flash chip: overlay it inside every ROM alias. */
        memory_region_add_subregion_overlap(rom_chip, nvram_addr - PROM_BASE,
                                            sysbus_mmio_get_region(
                                                SYS_BUS_DEVICE(dev), 0), 1);
    } else {
        sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, nvram_addr);
    }
    nvr = MACIO_NVRAM(dev);
    /*
     * The Apple ROM lays out and validates its own partitions; only the
     * OpenBIOS-style NVRAM wants the CHRP ones seeded, and even then only
     * into a genuinely fresh image -- otherwise the firmware's own persisted
     * variables get overwritten on every boot, which is exactly what the
     * backing file is there to stop.
     */
    if (!rom_is_flash && macio_nvram_is_blank(nvr, MACIO_NVRAM_SIZE)) {
        pmac_format_nvram_partition(nvr, MACIO_NVRAM_SIZE);
    }
    /*
     * A brand-new flash image is all-0xff and has no partition structure at
     * all -- seed the real default one, otherwise the ROM sees "NVRAM
     * corrupted" and rebuilds its own in-guest copy that never reaches the
     * backing file.  Then graft the boot-command (gmac MAC + the cache POST
     * data); both patch the in-RAM image only, as the source branch does.
     */
    if (rom_is_flash) {
        core99_nvram_graft(nvr, core99_clockfreq(core99_machine) / 2);
    }
    core99_report_boot_device(nvr);
    /* No PCI init: the BIOS will do it */

    /*
     * Real PM34 hardware's onboard FireWire (11c1:5811), at
     * /pci@f4000000/firewire@e.  A stub: Apple Hardware Test's Core99
     * FireWire TCM needs the node to exist before it can test it.
     */
    if (rom_is_flash && uninorth_internal_dev) {
        pci_create_simple(PCI_HOST_BRIDGE(uninorth_internal_dev)->bus,
                          PCI_DEVFN(0x0e, 0), "mac99-firewire-stub");
    }

    dev = qdev_new(TYPE_FW_CFG_MEM);
    fw_cfg = FW_CFG(dev);
    qdev_prop_set_uint32(dev, "data_width", 1);
    qdev_prop_set_bit(dev, "dma_enabled", false);
    object_property_add_child(OBJECT(machine), TYPE_FW_CFG, OBJECT(fw_cfg));
    s = SYS_BUS_DEVICE(dev);
    sysbus_realize_and_unref(s, &error_fatal);
    sysbus_mmio_map(s, 0, CFG_ADDR);
    sysbus_mmio_map(s, 1, CFG_ADDR + 2);

    fw_cfg_add_i16(fw_cfg, FW_CFG_NB_CPUS, (uint16_t)machine->smp.cpus);
    fw_cfg_add_i16(fw_cfg, FW_CFG_MAX_CPUS, (uint16_t)machine->smp.max_cpus);
    fw_cfg_add_i64(fw_cfg, FW_CFG_RAM_SIZE, (uint64_t)machine->ram_size);
    fw_cfg_add_i16(fw_cfg, FW_CFG_MACHINE_ID, machine_arch);
    fw_cfg_add_i32(fw_cfg, FW_CFG_KERNEL_ADDR, kernel_base);
    fw_cfg_add_i32(fw_cfg, FW_CFG_KERNEL_SIZE, kernel_size);
    if (machine->kernel_cmdline) {
        fw_cfg_add_i32(fw_cfg, FW_CFG_KERNEL_CMDLINE, cmdline_base);
        pstrcpy_targphys("cmdline", cmdline_base, TARGET_PAGE_SIZE,
                         machine->kernel_cmdline);
    } else {
        fw_cfg_add_i32(fw_cfg, FW_CFG_KERNEL_CMDLINE, 0);
    }
    fw_cfg_add_i32(fw_cfg, FW_CFG_INITRD_ADDR, initrd_base);
    fw_cfg_add_i32(fw_cfg, FW_CFG_INITRD_SIZE, initrd_size);
    fw_cfg_add_i16(fw_cfg, FW_CFG_BOOT_DEVICE, ppc_boot_device);

    fw_cfg_add_i16(fw_cfg, FW_CFG_PPC_WIDTH, graphic_width);
    fw_cfg_add_i16(fw_cfg, FW_CFG_PPC_HEIGHT, graphic_height);
    fw_cfg_add_i16(fw_cfg, FW_CFG_PPC_DEPTH, graphic_depth);

    fw_cfg_add_i32(fw_cfg, FW_CFG_PPC_VIACONFIG, core99_machine->via_config);

    fw_cfg_add_i32(fw_cfg, FW_CFG_PPC_IS_KVM, kvm_enabled());
    if (kvm_enabled()) {
        uint8_t *hypercall;

        hypercall = g_malloc(16);
        kvmppc_get_hypercall(env, hypercall, 16);
        fw_cfg_add_bytes(fw_cfg, FW_CFG_PPC_KVM_HC, hypercall, 16);
        fw_cfg_add_i32(fw_cfg, FW_CFG_PPC_KVM_PID, getpid());
    }
    fw_cfg_add_i32(fw_cfg, FW_CFG_PPC_TBFREQ, tbfreq);
    /* Mac OS X requires a "known good" clock-frequency value; pass it one. */
    fw_cfg_add_i32(fw_cfg, FW_CFG_PPC_CLOCKFREQ,
                   core99_clockfreq(core99_machine));
    fw_cfg_add_i32(fw_cfg, FW_CFG_PPC_BUSFREQ,
                   core99_busfreq(core99_machine));
    fw_cfg_add_i32(fw_cfg, FW_CFG_PPC_NVRAM_ADDR, nvram_addr);

    /* MacOS NDRV VGA driver */
    filename = qemu_find_file(QEMU_FILE_TYPE_BIOS, NDRV_VGA_FILENAME);
    if (filename) {
        gchar *ndrv_file;
        gsize ndrv_size;

        if (g_file_get_contents(filename, &ndrv_file, &ndrv_size, NULL)) {
            fw_cfg_add_file(fw_cfg, "ndrv/qemu_vga.ndrv", ndrv_file, ndrv_size);
        }
        g_free(filename);
    }

    qemu_register_boot_set(fw_cfg_boot_set, fw_cfg);
}

/*
 * Implementation of an interface to adjust firmware path
 * for the bootindex property handling.
 */
static char *core99_fw_dev_path(FWPathProvider *p, BusState *bus,
                                DeviceState *dev)
{
    PCIDevice *pci;
    MACIOIDEState *macio_ide;

    if (!strcmp(object_get_typename(OBJECT(dev)), "macio-newworld")) {
        pci = PCI_DEVICE(dev);
        return g_strdup_printf("mac-io@%x", PCI_SLOT(pci->devfn));
    }

    if (!strcmp(object_get_typename(OBJECT(dev)), "macio-ide")) {
        macio_ide = MACIO_IDE(dev);
        return g_strdup_printf("ata-3@%x", macio_ide->addr);
    }

    if (!strcmp(object_get_typename(OBJECT(dev)), "ide-hd")) {
        return g_strdup("disk");
    }

    if (!strcmp(object_get_typename(OBJECT(dev)), "ide-cd")) {
        return g_strdup("cdrom");
    }

    if (!strcmp(object_get_typename(OBJECT(dev)), "virtio-blk-device")) {
        return g_strdup("disk");
    }

    return NULL;
}
static int core99_kvm_type(MachineState *machine, const char *arg)
{
    /* Always force PR KVM */
    return 2;
}

#define CORE99_AGP_SLOT 0x10

/*
 * The AGP bus has one slot, device 0x10.  A real Apple ROM only looks for
 * its display there: a card anywhere else is sized and then abandoned, so a
 * display card given no addr= is moved into it while that slot is free, and
 * a card that can report an AGP identity (1002:5046) is told to.
 */
static bool core99_is_display_card(Core99MachineState *cms, DeviceState *dev)
{
    return cms->apple_rom && cms->agp_bus &&
           object_dynamic_cast(OBJECT(dev), TYPE_PCI_DEVICE) &&
           (PCI_DEVICE_GET_CLASS(dev)->class_id >> 8) ==
               PCI_BASE_CLASS_DISPLAY;
}

static void core99_machine_device_pre_plug(HotplugHandler *hotplug_dev,
                                           DeviceState *dev, Error **errp)
{
    Core99MachineState *cms = CORE99_MACHINE(hotplug_dev);
    PCIDevice *pci_dev = PCI_DEVICE(dev);
    int agp_devfn = PCI_DEVFN(CORE99_AGP_SLOT, 0);

    if (pci_dev->devfn == -1 && !cms->agp_bus->devices[agp_devfn]) {
        if (qdev_get_parent_bus(dev) != BUS(cms->agp_bus) &&
            !qdev_set_parent_bus(dev, BUS(cms->agp_bus), errp)) {
            return;
        }
        pci_dev->devfn = agp_devfn;
    }
    if (qdev_get_parent_bus(dev) == BUS(cms->agp_bus) &&
        pci_dev->devfn == agp_devfn &&
        object_property_find(OBJECT(dev), "agp")) {
        object_property_set_bool(OBJECT(dev), "agp", true, errp);
    }
}

static HotplugHandler *core99_get_hotplug_handler(MachineState *machine,
                                                  DeviceState *dev)
{
    if (core99_is_display_card(CORE99_MACHINE(machine), dev)) {
        return HOTPLUG_HANDLER(machine);
    }
    return NULL;
}

/*
 * PowerMac3,6's CPU is a 7455, not the 7400 the other models default to.
 * Hooking get_default_cpu_type means an explicit -cpu still wins.
 */
#ifndef TARGET_PPC64
static const char *core99_get_default_cpu_type(const MachineState *ms)
{
    Core99MachineState *cms = CORE99_MACHINE((MachineState *)ms);

    return core99_model_info(cms)->cpu_type;
}
#endif

static void core99_machine_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);
    FWPathProviderClass *fwc = FW_PATH_PROVIDER_CLASS(oc);
    HotplugHandlerClass *hc = HOTPLUG_HANDLER_CLASS(oc);

    mc->desc = "Mac99 based PowerMac";
    mc->init = ppc_core99_init;
    mc->block_default_type = IF_IDE;
    /* SMP supported via GPIO-based CPU reset mechanism */
    mc->max_cpus = 4;
    mc->default_boot_order = "cd";
    mc->default_display = "std";
    mc->default_nic = "sungem";
    mc->kvm_type = core99_kvm_type;
#ifdef TARGET_PPC64
    mc->default_cpu_type = POWERPC_CPU_TYPE_NAME("970fx_v3.1");
#else
    mc->default_cpu_type = POWERPC_CPU_TYPE_NAME("7400_v2.9");
    mc->get_default_cpu_type = core99_get_default_cpu_type;
#endif
    mc->default_ram_id = "ppc_core99.ram";
    mc->ignore_boot_device_suffixes = true;
    fwc->get_dev_path = core99_fw_dev_path;
    mc->get_hotplug_handler = core99_get_hotplug_handler;
    hc->pre_plug = core99_machine_device_pre_plug;
}

static char *core99_get_via_config(Object *obj, Error **errp)
{
    Core99MachineState *cms = CORE99_MACHINE(obj);

    switch (cms->via_config) {
    default:
    case CORE99_VIA_CONFIG_CUDA:
        return g_strdup("cuda");

    case CORE99_VIA_CONFIG_PMU:
        return g_strdup("pmu");

    case CORE99_VIA_CONFIG_PMU_ADB:
        return g_strdup("pmu-adb");
    }
}

static void core99_set_via_config(Object *obj, const char *value, Error **errp)
{
    Core99MachineState *cms = CORE99_MACHINE(obj);

    if (!strcmp(value, "cuda")) {
        cms->via_config = CORE99_VIA_CONFIG_CUDA;
    } else if (!strcmp(value, "pmu")) {
        cms->via_config = CORE99_VIA_CONFIG_PMU;
    } else if (!strcmp(value, "pmu-adb")) {
        cms->via_config = CORE99_VIA_CONFIG_PMU_ADB;
    } else {
        error_setg(errp, "Invalid via value");
        error_append_hint(errp, "Valid values are cuda, pmu, pmu-adb.\n");
    }
}

static char *core99_get_model(Object *obj, Error **errp)
{
    Core99MachineState *cms = CORE99_MACHINE(obj);

    return g_strdup(core99_model_info(cms)->name);
}

/*
 * "pm34", not "3,4": -M's option string is comma-separated, so a literal
 * comma in a property value has to be doubled to parse at all.
 */
static void core99_set_model(Object *obj, const char *value, Error **errp)
{
    Core99MachineState *cms = CORE99_MACHINE(obj);
    unsigned i;

    for (i = 0; i < ARRAY_SIZE(core99_models); i++) {
        if (!strcmp(value, core99_models[i].name)) {
            cms->model = i;
            return;
        }
    }
    error_setg(errp, "Invalid model value");
    error_append_hint(errp, "Valid values are");
    for (i = 0; i < ARRAY_SIZE(core99_models); i++) {
        error_append_hint(errp, "%s %s", i ? "," : "", core99_models[i].name);
    }
    error_append_hint(errp, ".\n");
}

static void core99_instance_init(Object *obj)
{
    Core99MachineState *cms = CORE99_MACHINE(obj);

    /* Default via_config is CORE99_VIA_CONFIG_CUDA */
    cms->via_config = CORE99_VIA_CONFIG_CUDA;
    object_property_add_str(obj, "via", core99_get_via_config,
                            core99_set_via_config);
    object_property_set_description(obj, "via",
                                    "Set VIA configuration. "
                                    "Valid values are cuda, pmu and pmu-adb");

    cms->model = CORE99_DEFAULT_MODEL;
    object_property_add_str(obj, "model", core99_get_model,
                            core99_set_model);
    object_property_set_description(obj, "model",
                                    "Set the machine model to emulate. "
                                    "Valid values are pm31, pm34 and pm36");
}

static const TypeInfo core99_machine_info = {
    .name          = MACHINE_TYPE_NAME("mac99"),
    .parent        = TYPE_MACHINE,
    .class_init    = core99_machine_class_init,
    .instance_init = core99_instance_init,
    .instance_size = sizeof(Core99MachineState),
    .interfaces = (const InterfaceInfo[]) {
        { TYPE_FW_PATH_PROVIDER },
        { TYPE_HOTPLUG_HANDLER },
        { }
    },
};

static void mac_machine_register_types(void)
{
    type_register_static(&core99_machine_info);
}

type_init(mac_machine_register_types)
