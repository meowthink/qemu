/* SPDX-License-Identifier: GPL-2.0-or-later */

/* Write through the DN3000 SCN2681 channel B console. */

        .text
        .globl  _start
_start:
        lea     message,%a1
next:
        move.b  (%a1)+,%d1
        tst.b   %d1
        beq.s   done
wait:
        move.b  0x008412,%d0
        andi.b  #0x04,%d0
        beq.s   wait
        move.b  %d1,0x008416
        bra.s   next
done:
        bra.s   done

message:
        .asciz  "DN3000 OK\r\n"
