/*
 * PowerMac NVRAM emulation
 *
 * Copyright (c) 2005-2007 Fabrice Bellard
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
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/nvram/chrp_nvram.h"
#include "hw/nvram/mac_nvram.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "system/block-backend.h"
#include "block/block_int.h"
#include "migration/vmstate.h"
#include "qemu/cutils.h"
#include "qemu/module.h"
#include "qemu/error-report.h"
#include "system/system.h"
#include "qobject/qdict.h"
#include "trace.h"
#include <zlib.h> /* for adler32 */

#define DEF_SYSTEM_SIZE 0xc10

/*
 * Flash NVRAM, Intel command set as used by Apple firmware and by the
 * Sharp/Micron parts Linux drives:
 *   erase: 20, d0 to the block, poll status bit 0x80, ff
 *   write: 40, data to the byte, poll status, ff
 * Erase granularity is 8 KB.
 *
 * The PowerMac3,6 ROM (4.6.0f1) additionally identifies the part before it
 * touches it: id-part writes ff then 90 to the window base and reads a
 * 16-bit manufacturer/device word back, accepting only Sharp b04b/b0ed,
 * Micron 8999/899d or (after the AMD unlock sequence) 0137. Anything else
 * takes its "unknown-flash-part" branch, and that branch is broken in the
 * ROM itself -- it leaves the string's address where encode-string expects
 * its length, so the property allocator advances `here` by ~4GB and Open
 * Firmware dies in a DSI storm before it ever probes a boot device. Real
 * hardware never gets there, so neither may we: answer as the Sharp part,
 * the first one probed. Its ready?/reset words use 70 (read status) and 50
 * (clear status) on top of the erase/program commands above.
 */
#define NVRAM_FLASH_SECTOR   0x2000
#define NVRAM_FLASH_READY    0x80
#define NVRAM_FLASH_MFR_ID   0xb0   /* Sharp */
#define NVRAM_FLASH_DEV_ID   0x4b

enum {
    NVRAM_FLASH_CMD_NONE        = 0x00,
    NVRAM_FLASH_CMD_ERASE_SETUP = 0x20,
    NVRAM_FLASH_CMD_PROGRAM     = 0x40,
    NVRAM_FLASH_CMD_CLEAR_STATUS = 0x50,
    NVRAM_FLASH_CMD_READ_STATUS = 0x70,
    NVRAM_FLASH_CMD_READ_ID     = 0x90,
    NVRAM_FLASH_CMD_ERASE_CONF  = 0xd0,
    NVRAM_FLASH_CMD_READ_ARRAY  = 0xff,
};

static void macio_nvram_flush(MacIONVRAMState *s, hwaddr addr, int len)
{
    if (s->blk) {
        if (blk_pwrite(s->blk, addr, len, &s->data[addr], 0) < 0) {
            error_report("%s: write of NVRAM data to backing store failed",
                         blk_name(s->blk));
        }
    }
}

/* Returns true if the write was a command rather than plain data */
static bool macio_nvram_flash_write(MacIONVRAMState *s, hwaddr addr,
                                    uint8_t value)
{
    if (!s->flash) {
        return false;
    }

    switch (s->flash_cmd) {
    case NVRAM_FLASH_CMD_ERASE_SETUP:
        s->flash_cmd = NVRAM_FLASH_CMD_NONE;
        if (value == NVRAM_FLASH_CMD_ERASE_CONF) {
            hwaddr base = addr & ~(hwaddr)(NVRAM_FLASH_SECTOR - 1);

            memset(&s->data[base], 0xff, NVRAM_FLASH_SECTOR);
            macio_nvram_flush(s, base, NVRAM_FLASH_SECTOR);
            s->flash_status = NVRAM_FLASH_READY;
        }
        return true;

    case NVRAM_FLASH_CMD_PROGRAM:
        /* programming can only clear bits */
        s->flash_cmd = NVRAM_FLASH_CMD_NONE;
        s->data[addr] &= value;
        macio_nvram_flush(s, addr, 1);
        s->flash_status = NVRAM_FLASH_READY;
        s->flash_read_id = false;
        return true;

    default:
        break;
    }

    switch (value) {
    case NVRAM_FLASH_CMD_ERASE_SETUP:
    case NVRAM_FLASH_CMD_PROGRAM:
        s->flash_cmd = value;
        s->flash_status = 0;
        s->flash_read_id = false;
        break;
    case NVRAM_FLASH_CMD_READ_ID:
        s->flash_cmd = NVRAM_FLASH_CMD_NONE;
        s->flash_status = 0;
        s->flash_read_id = true;
        break;
    case NVRAM_FLASH_CMD_READ_STATUS:
        /* Nothing here takes time, so the part is always ready. */
        s->flash_cmd = NVRAM_FLASH_CMD_NONE;
        s->flash_status = NVRAM_FLASH_READY;
        s->flash_read_id = false;
        break;
    case NVRAM_FLASH_CMD_CLEAR_STATUS:
        /* No error bits are modelled; the read mode is left alone. */
        break;
    case NVRAM_FLASH_CMD_READ_ARRAY:
        s->flash_cmd = NVRAM_FLASH_CMD_NONE;
        s->flash_status = 0;
        s->flash_read_id = false;
        break;
    default:
        /* status and other commands leave the array untouched */
        break;
    }
    return true;
}

/* macio style NVRAM device */
static void macio_nvram_writeb(void *opaque, hwaddr addr,
                               uint64_t value, unsigned size)
{
    MacIONVRAMState *s = opaque;

    addr = (addr >> s->it_shift) & (s->size - 1);
    trace_macio_nvram_write(addr, value);
    if (macio_nvram_flash_write(s, addr, value)) {
        return;
    }
    s->data[addr] = value;
    if (s->blk) {
        if (blk_pwrite(s->blk, addr, 1, &s->data[addr], 0) < 0) {
            error_report("%s: write of NVRAM data to backing store failed",
                         blk_name(s->blk));
        }
    }
}

static uint64_t macio_nvram_readb(void *opaque, hwaddr addr,
                                  unsigned size)
{
    MacIONVRAMState *s = opaque;
    uint32_t value;

    addr = (addr >> s->it_shift) & (s->size - 1);
    value = s->data[addr];
    /* while a command is in flight the part answers with its status */
    if (s->flash && s->flash_status) {
        value = s->flash_status;
    } else if (s->flash && s->flash_read_id) {
        value = (addr & 1) ? NVRAM_FLASH_DEV_ID : NVRAM_FLASH_MFR_ID;
    }
    trace_macio_nvram_read(addr, value);

    return value;
}

static const MemoryRegionOps macio_nvram_ops = {
    .read = macio_nvram_readb,
    .write = macio_nvram_writeb,
    .valid.min_access_size = 1,
    .valid.max_access_size = 8,
    .impl.min_access_size = 1,
    .impl.max_access_size = 1,
    .endianness = DEVICE_BIG_ENDIAN,
};

static const VMStateDescription vmstate_macio_nvram = {
    .name = "macio_nvram",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_VBUFFER_UINT32(data, MacIONVRAMState, 0, NULL, size),
        VMSTATE_END_OF_LIST()
    }
};


static void macio_nvram_reset(DeviceState *dev)
{
    MacIONVRAMState *s = MACIO_NVRAM(dev);

    /* A reset part comes back in read-array mode. */
    s->flash_cmd = NVRAM_FLASH_CMD_NONE;
    s->flash_status = 0;
    s->flash_read_id = false;
}

static void macio_nvram_realizefn(DeviceState *dev, Error **errp)
{
    SysBusDevice *d = SYS_BUS_DEVICE(dev);
    MacIONVRAMState *s = MACIO_NVRAM(dev);

    s->data = g_malloc0(s->size);

    if (s->blk) {
        int64_t len = blk_getlength(s->blk);
        if (len < 0) {
            error_setg_errno(errp, -len,
                             "could not get length of nvram backing image");
            return;
        } else if (len != s->size) {
            error_setg_errno(errp, -len,
                             "invalid size nvram backing image");
            return;
        }
        if (blk_set_perm(s->blk, BLK_PERM_CONSISTENT_READ | BLK_PERM_WRITE,
                         BLK_PERM_ALL, errp) < 0) {
            return;
        }
        if (blk_pread(s->blk, 0, s->size, s->data, 0) < 0) {
            error_setg(errp, "can't read-nvram contents");
            return;
        }
    }

    memory_region_init_io(&s->mem, OBJECT(s), &macio_nvram_ops, s,
                          "macio-nvram", s->size << s->it_shift);
    sysbus_init_mmio(d, &s->mem);
}

static void macio_nvram_unrealizefn(DeviceState *dev)
{
    MacIONVRAMState *s = MACIO_NVRAM(dev);

    g_free(s->data);
}

static const Property macio_nvram_properties[] = {
    DEFINE_PROP_UINT32("size", MacIONVRAMState, size, 0),
    DEFINE_PROP_UINT32("it_shift", MacIONVRAMState, it_shift, 0),
    DEFINE_PROP_BOOL("flash", MacIONVRAMState, flash, false),
    DEFINE_PROP_DRIVE("drive", MacIONVRAMState, blk),
};

static void macio_nvram_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = macio_nvram_realizefn;
    dc->unrealize = macio_nvram_unrealizefn;
    device_class_set_legacy_reset(dc, macio_nvram_reset);
    dc->vmsd = &vmstate_macio_nvram;
    device_class_set_props(dc, macio_nvram_properties);
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo macio_nvram_type_info = {
    .name = TYPE_MACIO_NVRAM,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MacIONVRAMState),
    .class_init = macio_nvram_class_init,
};

static void macio_nvram_register_types(void)
{
    type_register_static(&macio_nvram_type_info);
}

/* Set up a system OpenBIOS NVRAM partition */
static void pmac_format_nvram_partition_of(MacIONVRAMState *nvr, int off,
                                           int len)
{
    int sysp_end;

    /* OpenBIOS nvram variables partition */
    sysp_end = chrp_nvram_create_system_partition(&nvr->data[off],
                                                  DEF_SYSTEM_SIZE, len) + off;

    /* Free space partition */
    chrp_nvram_create_free_partition(&nvr->data[sysp_end], len - sysp_end);
}

/*
 * Old World Macs' genuine Open Firmware NVRAM partition: signature 0x1275,
 * version 5, at a fixed offset (0x1800) -- a completely different layout
 * from the CHRP/OpenBIOS text partition above (which is what New World
 * ROMs use instead; see DingusPPC's devices/common/ofnvram.cpp, whose own
 * comments label OfConfigAppl "Old World" vs OfConfigChrp "New World").
 * A real Beige G3 ROM's NanoKernel-level NVRAM validation looks for this
 * exact structure; without it, the boot never leaves the 68K low-memory
 * init code (confirmed by comparing against a live DingusPPC capture of
 * the identical ROM, whose own NVRAM has this partition populated and
 * has nothing at all at the offsets the CHRP partition above uses).
 *
 * The two chunks below are copied verbatim from a real, checksum-valid
 * partition captured from a successful DingusPPC boot of this same ROM
 * (header + int vars, then the string heap grown down from the partition
 * end); everything in between is zero, matching the captured data.
 */
#define OLDWORLD_OF_OFFSET      0x1800
#define OLDWORLD_OF_SIZE        0x800

static const uint8_t oldworld_of_partition_head[] = {
    /* sig=0x1275 version=5 num_pages=8 checksum=0x78fb here=0x185c top=0x1fe2 */
    0x12, 0x75, 0x05, 0x08, 0x78, 0xfb, 0x18, 0x5c, 0x1f, 0xe2, 0x00, 0x00,
    /* flags (auto-boot? only) */
    0x20, 0x00, 0x00, 0x00,
    /* real-base, real-size, virt-base, virt-size, load-base */
    0xff, 0xff, 0xff, 0xff, 0x00, 0x10, 0x00, 0x00,
    0xff, 0xff, 0xff, 0xff, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00,
    /* pci-probe-list, screen-#columns, screen-#rows */
    0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x64, 0x00, 0x00, 0x00, 0x28,
    /* selftest-#megs */
    0x00, 0x00, 0x00, 0x00,
    /* boot-device, boot-file, diag-device, diag-file, input-device,
     * output-device, oem-banner, oem-logo, nvramrc, boot-command
     * (offset,length) pairs into the string heap below */
    0x1f, 0xf7, 0x00, 0x09, 0x1f, 0xf7, 0x00, 0x00,
    0x1f, 0xef, 0x00, 0x08, 0x1f, 0xef, 0x00, 0x00, 0x1f, 0xec, 0x00, 0x03,
    0x1f, 0xe6, 0x00, 0x06, 0x1f, 0xe6, 0x00, 0x00, 0x1f, 0xe6, 0x00, 0x00,
    0x1f, 0xe6, 0x00, 0x00, 0x1f, 0xe2, 0x00, 0x04,
};

/* "/AAPL,ROM" (boot-device), "fd:diags" (diag-device), "kbd"
 * (input-device), "screen" (output-device), "boot" (boot-command) */
static const uint8_t oldworld_of_partition_strings[] = {
    'b', 'o', 'o', 't', 's', 'c', 'r', 'e', 'e', 'n', 'k', 'b', 'd',
    'f', 'd', ':', 'd', 'i', 'a', 'g', 's',
    '/', 'A', 'A', 'P', 'L', ',', 'R', 'O', 'M',
};

/* Checks the same signature/version/checksum a real Old World ROM's own
 * NVRAM validation would (see DingusPPC's OfConfigAppl::validate()) --
 * used to tell an already-valid, persisted partition (loaded from an
 * attached backing file, e.g. a prior boot's saved boot-device) apart
 * from a fresh/blank one that still needs our defaults. */
static bool oldworld_of_partition_valid(const uint8_t *buf)
{
    uint8_t tmp[OLDWORLD_OF_SIZE];
    uint32_t acc = 0;
    uint16_t stored;
    int i;

    if (lduw_be_p(buf) != 0x1275 || buf[2] != 5) {
        return false;
    }

    stored = lduw_be_p(buf + 4);
    memcpy(tmp, buf, OLDWORLD_OF_SIZE);
    tmp[4] = tmp[5] = 0;
    for (i = 0; i < OLDWORLD_OF_SIZE; i += 2) {
        acc += lduw_be_p(tmp + i);
    }
    acc = (acc + (acc >> 16)) & 0xffff;

    return ((~acc) & 0xffff) == stored;
}

static uint16_t oldworld_of_checksum(const uint8_t *buf)
{
    uint8_t tmp[OLDWORLD_OF_SIZE];
    uint32_t acc = 0;
    int i;

    memcpy(tmp, buf, OLDWORLD_OF_SIZE);
    tmp[4] = tmp[5] = 0;
    for (i = 0; i < OLDWORLD_OF_SIZE; i += 2) {
        acc += lduw_be_p(tmp + i);
    }
    acc = (acc + (acc >> 16)) & 0xffff;

    return (~acc) & 0xffff;
}

#define OLDWORLD_OF_TOP         0x08
#define OLDWORLD_OF_FLAGS       0x0c
#define OLDWORLD_OF_VARS        0x34

enum {
    OF_VAR_BOOT_DEVICE, OF_VAR_BOOT_FILE, OF_VAR_DIAG_DEVICE,
    OF_VAR_DIAG_FILE, OF_VAR_INPUT_DEVICE, OF_VAR_OUTPUT_DEVICE,
    OF_VAR_OEM_BANNER, OF_VAR_OEM_LOGO, OF_VAR_NVRAMRC,
    OF_VAR_BOOT_COMMAND, OF_VAR__COUNT
};

#define OLDWORLD_OF_AUTO_BOOT       0x20000000
#define OLDWORLD_OF_USE_NVRAMRC     0x01000000

static const char *oldworld_of_get_var(const uint8_t *buf, int var,
                                       unsigned *len)
{
    unsigned off = lduw_be_p(buf + OLDWORLD_OF_VARS + 4 * var);

    *len = lduw_be_p(buf + OLDWORLD_OF_VARS + 4 * var + 2);
    if (off < OLDWORLD_OF_OFFSET ||
        off + *len > OLDWORLD_OF_OFFSET + OLDWORLD_OF_SIZE) {
        *len = 0;
        return NULL;
    }
    return (const char *)buf + (off - OLDWORLD_OF_OFFSET);
}

static bool oldworld_of_set_vars(uint8_t *buf,
                                 const char *const value[OF_VAR__COUNT],
                                 uint32_t flags)
{
    unsigned heap = OLDWORLD_OF_SIZE, off[OF_VAR__COUNT], len[OF_VAR__COUNT];
    uint8_t new_part[OLDWORLD_OF_SIZE];
    unsigned table_end = lduw_be_p(buf + 0x06) - OLDWORLD_OF_OFFSET;
    int i;

    memcpy(new_part, buf, OLDWORLD_OF_SIZE);

    for (i = 0; i < OF_VAR__COUNT; i++) {
        len[i] = value[i] ? strlen(value[i]) : 0;
        if (len[i] > heap || heap - len[i] < table_end) {
            return false;
        }
        heap -= len[i];
        memcpy(new_part + heap, value[i], len[i]);
        off[i] = heap;
    }

    memset(new_part + table_end, 0, heap - table_end);
    for (i = 0; i < OF_VAR__COUNT; i++) {
        stw_be_p(new_part + OLDWORLD_OF_VARS + 4 * i,
                 OLDWORLD_OF_OFFSET + (len[i] ? off[i] : heap));
        stw_be_p(new_part + OLDWORLD_OF_VARS + 4 * i + 2, len[i]);
    }
    stw_be_p(new_part + OLDWORLD_OF_TOP, OLDWORLD_OF_OFFSET + heap);
    stl_be_p(new_part + OLDWORLD_OF_FLAGS, flags);
    stw_be_p(new_part + 4, 0);
    stw_be_p(new_part + 4, oldworld_of_checksum(new_part));

    memcpy(buf, new_part, OLDWORLD_OF_SIZE);
    return true;
}

#define OLDWORLD_POST_REQ_OFFSET 0x1043

/* Set up the Old World genuine-OF NVRAM partition (not the CHRP/OpenBIOS
 * one below, which real Old World ROMs never look at). If a backing file
 * (nvr->blk) already holds a valid partition -- persisted from an earlier
 * run -- it's left untouched instead of being clobbered with defaults. */
void pmac_format_nvram_partition_oldworld(MacIONVRAMState *nvr)
{
    uint8_t *buf = &nvr->data[OLDWORLD_OF_OFFSET];

    if (nvr->data[OLDWORLD_POST_REQ_OFFSET]) {
        nvr->data[OLDWORLD_POST_REQ_OFFSET] = 0;
        if (nvr->blk) {
            blk_pwrite(nvr->blk, OLDWORLD_POST_REQ_OFFSET, 1,
                       &nvr->data[OLDWORLD_POST_REQ_OFFSET], 0);
        }
    }

    if (oldworld_of_partition_valid(buf)) {
        return;
    }

    memset(buf, 0, OLDWORLD_OF_SIZE);
    memcpy(buf, oldworld_of_partition_head, sizeof(oldworld_of_partition_head));
    memcpy(buf + OLDWORLD_OF_SIZE - sizeof(oldworld_of_partition_strings),
           oldworld_of_partition_strings, sizeof(oldworld_of_partition_strings));

    if (nvr->blk) {
        if (blk_pwrite(nvr->blk, OLDWORLD_OF_OFFSET, OLDWORLD_OF_SIZE, buf,
                       0) < 0) {
            error_report("%s: failed to write default Old World NVRAM "
                        "partition", blk_name(nvr->blk));
        }
    }
}

static const char oldworld_osx_boot_command[] = "0 bootr ";

static const char oldworld_osx_boot_shim[] =
    "hex\r"
    ": qE device-end ;\r"
    ": qR BRpatch ;\r"
    ": qprop 0 to my-self property ;\r"
    ": qargs \" \" encode-string \" machargs\" \" /chosen\" find-device "
        "qprop qE ;\r"
    ": qmem ['] install-interrupt-vectors ['] noop qR\r"
    "0 4000 release-mem 8000 2000 release-mem ;\r"
    "dev mac-io\r"
    ": decode-unit parse-1hex ;\r"
    "qE\r"
    ": bootr qargs qmem boot ;\r";

bool pmac_oldworld_nvram_is_default(MacIONVRAMState *nvr)
{
    const uint8_t *buf = &nvr->data[OLDWORLD_OF_OFFSET];
    const char *dev;
    unsigned dev_len, rc_len;

    if (!oldworld_of_partition_valid(buf)) {
        return false;
    }
    dev = oldworld_of_get_var(buf, OF_VAR_BOOT_DEVICE, &dev_len);
    oldworld_of_get_var(buf, OF_VAR_NVRAMRC, &rc_len);

    return rc_len == 0 && dev_len == strlen("/AAPL,ROM") &&
           !memcmp(dev, "/AAPL,ROM", dev_len);
}

void pmac_oldworld_nvram_set_osx_startup(MacIONVRAMState *nvr,
                                         const char *device)
{
    uint8_t *buf = &nvr->data[OLDWORLD_OF_OFFSET];
    const char *value[OF_VAR__COUNT] = { NULL };
    unsigned len;
    int i;

    for (i = 0; i < OF_VAR__COUNT; i++) {
        value[i] = oldworld_of_get_var(buf, i, &len);
        value[i] = len ? g_strndup(value[i], len) : NULL;
    }
    g_free((char *)value[OF_VAR_BOOT_DEVICE]);
    value[OF_VAR_BOOT_DEVICE] = g_strdup(device);
    g_free((char *)value[OF_VAR_NVRAMRC]);
    value[OF_VAR_NVRAMRC] = g_strdup(oldworld_osx_boot_shim);
    g_free((char *)value[OF_VAR_BOOT_COMMAND]);
    value[OF_VAR_BOOT_COMMAND] = g_strdup(oldworld_osx_boot_command);

    if (!oldworld_of_set_vars(buf, value,
                              ldl_be_p(buf + OLDWORLD_OF_FLAGS) |
                              OLDWORLD_OF_AUTO_BOOT |
                              OLDWORLD_OF_USE_NVRAMRC)) {
        warn_report("NVRAM: Mac OS X startup shim does not fit, leaving "
                    "the default partition alone");
    } else if (nvr->blk &&
               blk_pwrite(nvr->blk, OLDWORLD_OF_OFFSET, OLDWORLD_OF_SIZE,
                          buf, 0) < 0) {
        error_report("%s: failed to write the Mac OS X startup NVRAM "
                     "partition", blk_name(nvr->blk));
    }

    for (i = 0; i < OF_VAR__COUNT; i++) {
        g_free((char *)value[i]);
    }
}

#define OSX_NVRAM_SIGNATURE     (0x5A)

/* Set up a Mac OS X NVRAM partition */
static void pmac_format_nvram_partition_osx(MacIONVRAMState *nvr, int off,
                                            int len)
{
    uint32_t start = off;
    ChrpNvramPartHdr *part_header;
    unsigned char *data = &nvr->data[start];

    /* empty partition */
    part_header = (ChrpNvramPartHdr *)data;
    part_header->signature = OSX_NVRAM_SIGNATURE;
    pstrcpy(part_header->name, sizeof(part_header->name), "wwwwwwwwwwww");

    chrp_nvram_finish_partition(part_header, len);

    /* Generation */
    stl_be_p(&data[20], 2);

    /* Adler32 checksum */
    stl_be_p(&data[16], adler32(0, &data[20], len - 20));
}

/*
 * A G5's flash NVRAM, as its firmware, Mac OS X and Linux keep it: two
 * 8 KB banks, the live one being the valid bank with the higher
 * generation. Each starts with a header partition (0x5a "nvram": Adler-32
 * of the bank from byte 20, then the generation), followed by the Open
 * Firmware variables in "common" and free space. Bank A is written,
 * bank B left erased.
 */
void pmac_format_nvram_core99(MacIONVRAMState *nvr)
{
    uint8_t *bank = nvr->data;
    ChrpNvramPartHdr *hdr = (ChrpNvramPartHdr *)bank;
    ChrpNvramPartHdr *common;
    int end;

    memset(nvr->data, 0xff, nvr->size);

    memset(bank, 0, 32);
    hdr->signature = OSX_NVRAM_SIGNATURE;
    pstrcpy(hdr->name, sizeof(hdr->name), "nvram");
    chrp_nvram_finish_partition(hdr, 32);

    common = (ChrpNvramPartHdr *)&bank[32];
    end = chrp_nvram_create_system_partition(&bank[32], DEF_SYSTEM_SIZE,
                                             MACIO_NVRAM_SIZE - 32);
    pstrcpy(common->name, sizeof(common->name), "common");
    chrp_nvram_finish_partition(common, end);
    chrp_nvram_create_free_partition(&bank[32 + end],
                                     MACIO_NVRAM_SIZE - 32 - end);

    stl_be_p(&bank[20], 1);
    stl_be_p(&bank[16], adler32(1, &bank[20], MACIO_NVRAM_SIZE - 20));

    if (nvr->blk &&
        blk_pwrite(nvr->blk, 0, nvr->size, nvr->data, 0) < 0) {
        error_report("%s: write of default NVRAM failed", blk_name(nvr->blk));
    }
}

/* Length of the CHRP partition at p, 0 if its header is not valid */
static uint32_t chrp_partition_len(const uint8_t *p)
{
    ChrpNvramPartHdr hdr;
    uint32_t len;

    memcpy(&hdr, p, sizeof(hdr));
    len = be16_to_cpu(hdr.len) << 4;
    chrp_nvram_finish_partition(&hdr, len);
    return hdr.checksum == p[1] ? len : 0;
}

/* Generation of a valid Core99 bank, 0 if it is not one */
static uint32_t core99_bank_generation(const uint8_t *bank)
{
    if (bank[0] != OSX_NVRAM_SIGNATURE || chrp_partition_len(bank) == 0 ||
        (uint32_t)ldl_be_p(&bank[16]) !=
        adler32(1, &bank[20], MACIO_NVRAM_SIZE - 20)) {
        return 0;
    }
    return ldl_be_p(&bank[20]);
}

/*
 * True if the bank the firmware will pick (the valid one with the higher
 * generation, B on a tie) is a valid bank whose partitions tile it and
 * include the Open Firmware "common" partition.
 */
bool pmac_nvram_core99_valid(MacIONVRAMState *nvr)
{
    uint32_t gen_a, gen_b, off, len;
    const uint8_t *bank;
    bool common = false;

    if (nvr->size != MACIO_NVRAM_FLASH_SIZE) {
        return false;
    }
    gen_a = core99_bank_generation(nvr->data);
    gen_b = core99_bank_generation(nvr->data + MACIO_NVRAM_SIZE);
    if (MAX(gen_a, gen_b) == 0) {
        return false;
    }
    bank = nvr->data + (gen_a > gen_b ? 0 : MACIO_NVRAM_SIZE);
    for (off = 0; off < MACIO_NVRAM_SIZE; off += len) {
        len = chrp_partition_len(&bank[off]);
        if (len == 0 || len > MACIO_NVRAM_SIZE - off) {
            return false;
        }
        if (bank[off] == CHRP_NVPART_SYSTEM &&
            !strncmp((const char *)&bank[off + 4], "common", 12)) {
            common = true;
        }
    }
    return common;
}

/* True if the -prom-env list sets the variable of the "name=value" var */
static bool prom_env_sets(const char *var)
{
    const char *eq = strchr(var, '=');
    unsigned int i;

    for (i = 0; eq && i < nb_prom_envs; i++) {
        if (!strncmp(prom_envs[i], var, eq - var + 1)) {
            return true;
        }
    }
    return false;
}

#define PANIC_INFO_VAR "aapl,panic-info="

/*
 * Undo the IODTNVRAM escaping of a data value: 0xff, then a byte whose
 * low 7 bits count a run of 0x00 (bit 7 clear) or 0xff (bit 7 set).
 * Returns the length of the result, at most n.
 */
static size_t nvram_unescape_data(uint8_t *out, const uint8_t *in, size_t n)
{
    size_t i, o = 0;
    unsigned int run;

    for (i = 0; i < n; i++) {
        if (in[i] != 0xff) {
            out[o++] = in[i];
            continue;
        }
        if (++i == n) {
            break;
        }
        for (run = in[i] & 0x7f; run && o < n; run--) {
            out[o++] = (in[i] & 0x80) ? 0xff : 0x00;
        }
    }
    return o;
}

/*
 * Unpack text stored as 7-bit characters, 8 in 7 bytes, most significant
 * bit first. Returns a g_malloc'd string.
 */
static char *nvram_unpack7(const uint8_t *in, size_t n)
{
    size_t chars = n * 8 / 7, i;
    char *out = g_malloc(chars + 1);

    for (i = 0; i < chars; i++) {
        size_t bit = i * 7, b = bit >> 3, sh = bit & 7;
        unsigned int v = in[b] << 8 | (b + 1 < n ? in[b + 1] : 0);

        out[i] = (v >> (9 - sh)) & 0x7f;
    }
    out[chars] = 0;
    return out;
}

static bool nvram_text_ok(const char *t)
{
    size_t i, good = 0;

    for (i = 0; t[i]; i++) {
        good += (t[i] >= 0x20 && t[i] < 0x7f) || t[i] == '\n' ||
                t[i] == '\r' || t[i] == '\t';
    }
    return i >= 8 && good * 10 >= i * 9;
}

/*
 * The panic log of an aapl,panic-info value as text: unescaped, then
 * unpacked from the 7-bit stream after the best-fitting header length
 * if that reads as text, else a hex dump of the unescaped bytes.
 */
static char *nvram_panic_text(const uint8_t *val, size_t n)
{
    g_autofree uint8_t *raw = g_malloc(n + 1);
    size_t len = nvram_unescape_data(raw, val, n), hdr, i;
    GString *str;

    for (hdr = 0; hdr <= 16 && hdr < len; hdr += 4) {
        char *t = nvram_unpack7(raw + hdr, len - hdr);

        if (nvram_text_ok(t)) {
            return t;
        }
        g_free(t);
    }
    str = g_string_new(NULL);
    for (i = 0; i < len; i++) {
        g_string_append_printf(str, "%02x%c", raw[i],
                               (i & 15) == 15 || i + 1 == len ? '\n' : ' ');
    }
    return g_string_free(str, false);
}

/* Save the panic log in val next to the NVRAM image; its path, or NULL */
static char *nvram_save_panic(MacIONVRAMState *nvr, const uint8_t *val,
                              size_t n)
{
    g_autofree char *dir = NULL;
    g_autofree char *text = NULL;
    const char *name;
    char *path;

    if (!nvr->blk || !blk_bs(nvr->blk)) {
        return NULL;
    }
    name = blk_bs(nvr->blk)->filename;
    if (!name[0] || strstart(name, "json:", NULL)) {
        return NULL;
    }
    dir = g_path_get_dirname(name);
    path = g_build_filename(dir, "last-panic.txt", NULL);
    text = nvram_panic_text(val, n);
    if (!g_file_set_contents(path, text, -1, NULL)) {
        g_free(path);
        return NULL;
    }
    return path;
}

/*
 * Set the -prom-env variables in the "common" partition of the live bank
 * of a valid Core99 NVRAM, in place of any of the same name.
 */
void pmac_nvram_core99_set_prom_env(MacIONVRAMState *nvr)
{
    g_autofree uint8_t *buf = NULL;
    g_autofree char *panic_path = NULL;
    uint32_t gen_a, gen_b, off, len, size, pos, p, n;
    uint8_t *bank, *vars;
    unsigned int i;
    bool drop_panic = false;

    if (!nb_prom_envs || !pmac_nvram_core99_valid(nvr)) {
        return;
    }
    gen_a = core99_bank_generation(nvr->data);
    gen_b = core99_bank_generation(nvr->data + MACIO_NVRAM_SIZE);
    bank = nvr->data + (gen_a > gen_b ? 0 : MACIO_NVRAM_SIZE);
    for (off = 0; ; off += len) {
        len = chrp_partition_len(&bank[off]);
        if (bank[off] == CHRP_NVPART_SYSTEM &&
            !strncmp((const char *)&bank[off + 4], "common", 12)) {
            break;
        }
    }
    vars = &bank[off + sizeof(ChrpNvramPartHdr)];
    size = len - sizeof(ChrpNvramPartHdr);
    buf = g_malloc0(size);

retry:
    memset(buf, 0, size);
    pos = 0;
    for (p = 0; p < size && vars[p]; p += n + 1) {
        n = strnlen((const char *)&vars[p], size - p);
        if (n == size - p) {
            break;
        }
        if (prom_env_sets((const char *)&vars[p]) ||
            (drop_panic && strstart((const char *)&vars[p], PANIC_INFO_VAR,
                                    NULL))) {
            continue;
        }
        if (pos + n + 1 >= size) {
            goto full;
        }
        memcpy(&buf[pos], &vars[p], n + 1);
        pos += n + 1;
    }
    for (i = 0; i < nb_prom_envs; i++) {
        n = strlen(prom_envs[i]);
        if (pos + n + 1 >= size) {
            goto full;
        }
        memcpy(&buf[pos], prom_envs[i], n + 1);
        pos += n + 1;
    }

    memcpy(vars, buf, size);
    if (drop_panic) {
        if (panic_path) {
            warn_report("NVRAM was full: saved the last panic log to %s",
                        panic_path);
        } else {
            warn_report("NVRAM was full: dropped the last panic log");
        }
    }
    stl_be_p(&bank[16], adler32(1, &bank[20], MACIO_NVRAM_SIZE - 20));
    if (nvr->blk && blk_pwrite(nvr->blk, bank - nvr->data, MACIO_NVRAM_SIZE,
                               bank, 0) < 0) {
        error_report("%s: write of NVRAM failed", blk_name(nvr->blk));
    }
    return;

full:
    if (!drop_panic) {
        for (p = 0; p < size && vars[p]; p += n + 1) {
            n = strnlen((const char *)&vars[p], size - p);
            if (n == size - p) {
                break;
            }
            if (strstart((const char *)&vars[p], PANIC_INFO_VAR, NULL)) {
                char *path = nvram_save_panic(nvr,
                    &vars[p + strlen(PANIC_INFO_VAR)],
                    n - strlen(PANIC_INFO_VAR));

                if (path || !nvr->blk || !blk_bs(nvr->blk)) {
                    panic_path = path;
                    drop_panic = true;
                    goto retry;
                }
                break;
            }
        }
    }
    warn_report("NVRAM has no room for the -prom-env variables");
}


/*
 * Manage a default NVRAM backing file when the user has not attached one
 * with -drive if=mtd,... Without this the contents are re-stamped from
 * scratch on every run, so nothing a firmware writes -- boot-device,
 * auto-boot?, the lot -- ever survives a restart.
 */
BlockBackend *macio_nvram_default_blk(const char *filename, uint32_t size,
                                     uint8_t fill)
{
    struct stat st;
    BlockBackend *blk;
    Error *local_err = NULL;
    QDict *options;
    int fd;

    fd = qemu_open_old(filename, O_RDWR | O_CREAT, 0644);
    if (fd < 0) {
        warn_report("could not open/create default NVRAM image '%s': %s",
                    filename, strerror(errno));
        return NULL;
    }
    if (fstat(fd, &st) == 0 && st.st_size < size) {
        /*
         * Flash-backed NVRAM must start out erased (all ones), not zeroed:
         * firmware tells "never written" from "written and invalid", and a
         * block of zeroes is neither.
         */
        g_autofree uint8_t *blank = g_malloc(size);

        memset(blank, fill, size);
        if (ftruncate(fd, 0) != 0 || write(fd, blank, size) != size) {
            warn_report("could not size default NVRAM image '%s': %s",
                        filename, strerror(errno));
        }
    }
    close(fd);

    options = qdict_new();
    qdict_put_str(options, "driver", "raw");
    blk = blk_new_open(filename, NULL, options, BDRV_O_RDWR, &local_err);
    if (!blk) {
        warn_report_err(local_err);
        return NULL;
    }
    return blk;
}

/* Set up NVRAM with OF and OSX partitions */
void pmac_format_nvram_partition(MacIONVRAMState *nvr, int len)
{
    /*
     * Mac OS X expects side "B" of the flash at the second half of NVRAM,
     * so we use half of the chip for OF and the other half for a free OSX
     * partition.
     */
    pmac_format_nvram_partition_of(nvr, 0, len / 2);
    pmac_format_nvram_partition_osx(nvr, len / 2, len / 2);
}
type_init(macio_nvram_register_types)

bool macio_nvram_is_blank(MacIONVRAMState *nvr, int len)
{
    int i;

    for (i = 0; i < len; i++) {
        if (nvr->data[i]) {
            return false;
        }
    }
    return true;
}
