#!/bin/sh
# SPDX-License-Identifier: GPL-2.0-or-later
set -eu

qemu=${QEMU_M68K_SYSTEM:-qemu-system-m68k}
cross=${M68K_CROSS_PREFIX:-m68k-suse-linux-}
srcdir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
tmpdir=$(mktemp -d)
trap 'rm -rf "$tmpdir"' EXIT HUP INT TERM

"${cross}as" -m68020 -o "$tmpdir/apollo-smoke.o" \
    "$srcdir/apollo-smoke.S"
"${cross}ld" -T "$srcdir/kernel.ld" -o "$tmpdir/apollo-smoke.elf" \
    "$tmpdir/apollo-smoke.o"

set +e
timeout 5s "$qemu" \
    -M apollo-dn3000 -kernel "$tmpdir/apollo-smoke.elf" -m 8M \
    -display none -monitor none -serial "file:$tmpdir/serial"
rc=$?
set -e

test "$rc" -eq 124
grep -q "DN3000 OK" "$tmpdir/serial"
