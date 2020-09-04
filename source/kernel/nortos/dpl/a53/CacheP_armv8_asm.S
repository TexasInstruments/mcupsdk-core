/*
 *  Copyright (C) 2018-2021 Texas Instruments Incorporated
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


.macro ICACHE_LINE_SIZE reg0, reg1
        mrs     \reg1, ctr_el0
        and     \reg1, \reg1, #0xf          /* extract cache line size bits */
        mov     \reg0, #4
        lsl     \reg0, \reg0, \reg1         /* compute cache line size */
.endm


.macro DCACHE_LINE_SIZE reg0, reg1
        mrs     \reg1, ctr_el0
        ubfm    \reg1, \reg1, #16, #19      /* extract cache line size bits */
        mov     \reg0, #4
        lsl     \reg0, \reg0, \reg1         /* compute cache line size */
.endm

/*
 * ======== CacheP_invL1pAll ========
 * Invalidates all entries in the instruction cache
 */
/* FUNCTION DEF: void CacheP_invL1pAll(void) */
        .global CacheP_invL1pAll
        .section .text.CacheP_invL1pAll
        .func CacheP_invL1pAll

CacheP_invL1pAll:
        ic      iallu                    /* invalidate all ICache */
        dsb     sy
        isb
        ret
        .endfunc



/* FUNCTION DEF: void CacheP_getEnabled(void) */
        .global CacheP_getEnabled
        .section .text.CacheP_getEnabled
        .func CacheP_getEnabled
CacheP_getEnabled:
        mov     x0, #0
        mrs     x1, sctlr_el1            /* read SCTLR_EL1 */

        /* Check if program cache enabled */
        tst     x1, #0x1000              /* test I bit for program cache */
        beq     1f
        mov     x0, #5                   /* if I bit is set, program caches */
                                         /* are enabled */
1:
        /* Check if data cache enabled */
        tst     x1, #0x0004              /* test C bit for data cache */
        beq     2f
        add     x0, x0, #0xa             /* if C bit is set, L1D and unified */
                                         /* L2 cache is enabled */
2:
        /* Check if all caches enabled */
        cmp     x0, #0xf
        bne     3f
        mov     x0, #0x7fff
3:
        ret
        .endfunc


/* FUNCTION DEF: void CacheP_disableL1D(void) */
        .global CacheP_disableL1D
        .section .text.CacheP_disableL1D
        .func CacheP_disableL1D

CacheP_disableL1D:
        stp     x0, x30, [sp, #-16]!
        mrs     x0, sctlr_el1            /* read SCTLR_EL1 */
        bic     x0, x0, #0x0004          /* clear C bit */
        msr     sctlr_el1, x0            /* DCache disabled */
        ldr     x0, =CacheP_wbInvAll
        blr     x0                       /* push out all of L1D */
        ldp     x0, x30, [sp], #16
        ret
        .endfunc


/* FUNCTION DEF: void CacheP_wait(void) */
        .global CacheP_wait
        .section .text.CacheP_wait
        .func CacheP_wait

CacheP_wait:
        dsb     sy
        ret
        .endfunc


/* FUNCTION DEF: void CacheP_wbInvAll(uint32_t type) */
        .global CacheP_wbInvAll
        .section .text.CacheP_wbInvAll
        .func CacheP_wbInvAll

CacheP_wbInvAll:
        mrs     x0, clidr_el1
        and     w3, w0, #0x07000000     /* Get 2 x Level of Coherence */
        lsr     w3, w3, #23
        cbz     w3, 5f
        mov     w10, #0                 /* w10 = 2 x cache level */
        mov     w8, #1                  /* w8 = Constant 0b1 */
1:
        add     w2, w10, w10, lsr #1    /* Caclulate 3x cache level */
        lsr     w1, w0, w2              /* Extract cache type for this level */
        and     w1, w1, #0x7
        cmp     w1, #2
        blt     4f                      /* No data or unified cache */
        msr     csselr_el1, x10         /* Select this cache level */
        isb                             /* Synchronize change of csselr */
        mrs     x1, ccsidr_el1          /* Read ccsidr */
        and     w2, w1, #7              /* w2 = log2(linelen)-4 */
        add     w2, w2, #4              /* w2 = log2(linelen) */
        ubfx    w4, w1, #3, #10         /* w4 = max way number, right aligned */
        clz     w5, w4                  /* w5 = 32-log2(ways), bit position of
                                           way in dc operand */
        lsl     w9, w4, w5              /* w9 = max way number, aligned to
                                           position in DC operand */
        lsl     w16, w8, w5             /* w16 = amount to decrement way number
                                           per iteration */
2:
        ubfx    w7, w1, #13, #15        /* w7 = max set number, right aligned */
        lsl     w7, w7, w2              /* w7 = max set number, aligned to
                                           position in DC operand */
        lsl     w17, w8, w2             /* w17 = amount to decrement set number
                                           per iteration */
3:
        orr     w11, w10, w9            /* w11 = combine way num & cache ...*/
        orr     w11, w11, w7            /* ... num and set num for DC operand */
        dc      cisw, x11               /* Do data cache clean and invalidate
                                           by set and way */
        subs    w7, w7, w17             /* Decrement set number */
        bge     3b
        subs    x9, x9, x16             /* Decrement way number */
        bge     2b
4:
        add     w10, w10, #2            /* Increment 2 x cache level */
        cmp     w3, w10
        dsb     sy                      /* Ensure completion of previous cache
                                           maintenance operation */
        bgt     1b
5:
        ret
        .endfunc


/* FUNCTION DEF: void CacheP_disableL1P(void) */
        .global CacheP_disableL1P
        .section .text.CacheP_disableL1P
        .func CacheP_disableL1P

CacheP_disableL1P:
        mrs     x0, sctlr_el1            /* read SCTLR_EL1 */
        bic     x0, x0, #0x1000          /* clear I bit */
        msr     sctlr_el1, x0            /* ICache disabled */
        ic      iallu                    /* invalidate all ICache */
        dsb     sy
        isb
        ret
        .endfunc


/* FUNCTION DEF: void CacheP_enableL1D(void)  */
        .global CacheP_enableL1D
        .section .text.CacheP_enableL1D
        .func CacheP_enableL1D

CacheP_enableL1D:
        dmb     sy                       /* ensure all pending memory */
                                         /* ...operations such as pending */
                                         /* ...stores/writes have completed */
        mrs     x0, sctlr_el1            /* read SCTLR_EL1 */
        orr     x0, x0, #0x0004          /* set C bit */
        msr     sctlr_el1, x0            /* DCache enabled */
        isb
        ret
        .endfunc


/* FUNCTION DEF: void CacheP_enableL1P(void) */
        .global CacheP_enableL1P
        .section .text.CacheP_enableL1P
        .func CacheP_enableL1P

CacheP_enableL1P:
        mrs     x0, sctlr_el1            /* read SCTLR_EL1 */
        orr     x0, x0, #0x1000          /* set I bit */
        msr     sctlr_el1, x0            /* ICache enabled */
        isb
        ret
        .endfunc


/* FUNCTION DEF: void CacheP_invL1p(uintptr_t blockPtr, uint32_t byteCnt) */
        .global CacheP_invL1p
        .section .text.CacheP_invL1p
        .func CacheP_invL1p

CacheP_invL1p:
        add     x1, x0, x1               /* calculate last address */
        ICACHE_LINE_SIZE x3, x4
        sub     x4, x3, #1
        bic     x0, x0, x4               /* align blockPtr to cache line */
1:
        ic      ivau, x0                 /* invalidate single entry in ICache */
        add     x0, x0, x3               /* increment address by cache line
                                            size */
        cmp     x0, x1                   /* compare to last address */
        blo     1b                       /* loop if > 0 */
        dsb     sy
        isb
        ret
        .endfunc


/* FUNCTION DEF: void CacheP_invL1d(uintptr_t blockPtr, uint32_t byteCnt) */
        .global CacheP_invL1d
        .section .text.CacheP_invL1d
        .func CacheP_invL1d

CacheP_invL1d:
        add     x1, x0, x1               /* calculate last address */
        DCACHE_LINE_SIZE x3, x4
        sub     x4, x3, #1
        bic     x0, x0, x4               /* align blockPtr to cache line */
1:
        dc      ivac, x0                 /* invalidate single entry in DCache */
        add     x0, x0, x3               /* increment address by cache line
                                            size */
        cmp     x0, x1                   /* compare to last address */
        blo     1b                       /* loop if > 0 */
        dsb     sy
        ret
        .endfunc


/* FUNCTION DEF: void CacheP_wb(uintptr_t blockPtr, uint32_t byteCnt, uint32_t type) */
        .global CacheP_wb
        .section .text.CacheP_wb
        .func CacheP_wb

CacheP_wb:
        add     x1, x0, x1               /* calculate last address */
        DCACHE_LINE_SIZE x3, x4
        sub     x4, x3, #1
        bic     x0, x0, x4               /* align blockPtr to cache line */
1:
        dc      cvac, x0                 /* clean single entry in DCache to
                                            PoC */
        add     x0, x0, x3               /* increment address by cache line
                                            size */
        cmp     x0, x1                   /* compare to last address */
        blo     1b                       /* loop if > 0 */
        dsb     sy
        ret
        .endfunc


/* FUNCTION DEF: void CacheP_wbAll(uint32_t type) */
        .global CacheP_wbAll
        .section .text.CacheP_wbAll
        .func CacheP_wbAll

CacheP_wbAll:
        mrs     x0, clidr_el1
        and     w3, w0, #0x07000000     /* Get 2 x Level of Coherence */
        lsr     w3, w3, #23
        cbz     w3, 5f
        mov     w10, #0                 /* w10 = 2 x cache level */
        mov     w8, #1                  /* w8 = Constant 0b1 */
1:
        add     w2, w10, w10, lsr #1    /* Caclulate 3x cache level */
        lsr     w1, w0, w2              /* Extract cache type for this level */
        and     w1, w1, #0x7
        cmp     w1, #2
        blt     4f                      /* No data or unified cache */
        msr     csselr_el1, x10         /* Select this cache level */
        isb                             /* Synchronize change of csselr */
        mrs     x1, ccsidr_el1          /* Read ccsidr */
        and     w2, w1, #7              /* w2 = log2(linelen)-4 */
        add     w2, w2, #4              /* w2 = log2(linelen) */
        ubfx    w4, w1, #3, #10         /* w4 = max way number, right aligned */
        clz     w5, w4                  /* w5 = 32-log2(ways), bit position of
                                           way in dc operand */
        lsl     w9, w4, w5              /* w9 = max way number, aligned to
                                           position in DC operand */
        lsl     w16, w8, w5             /* w16 = amount to decrement way number
                                           per iteration */
2:
        ubfx    w7, w1, #13, #15        /* w7 = max set number, right aligned */
        lsl     w7, w7, w2              /* w7 = max set number, aligned to
                                           position in DC operand */
        lsl     w17, w8, w2             /* w17 = amount to decrement set number
                                           per iteration */
3:
        orr     w11, w10, w9            /* w11 = combine way num & cache ...*/
        orr     w11, w11, w7            /* ... num and set num for DC operand */
        dc      csw, x11                /* Do data cache clean by set and way */
        subs    w7, w7, w17             /* Decrement set number */
        bge     3b
        subs    x9, x9, x16             /* Decrement way number */
        bge     2b
4:
        add     w10, w10, #2            /* Increment 2 x cache level */
        cmp     w3, w10
        dsb     sy                      /* Ensure completion of previous cache
                                           maintenance operation */
        bgt     1b
5:
        ret
        .endfunc


/* FUNCTION DEF: void CacheP_wbInv(uintptr_t blockPtr, uint32_t byteCnt, uint32_t type) */
        .global CacheP_wbInv
        .section .text.CacheP_wbInv
        .func CacheP_wbInv

CacheP_wbInv:
        add     x1, x0, x1               /* calculate last address */
        DCACHE_LINE_SIZE x3, x4
        sub     x4, x3, #1
        bic     x0, x0, x4               /* align blockPtr to cache line */
1:
        dc      civac, x0                /* clean and invalidate single entry
                                            in DCache to PoC */
        add     x0, x0, x3               /* increment address by cache line
                                            size */
        cmp     x0, x1                   /* compare to last address */
        blo     1b                       /* loop if > 0 */
        dsb     sy
        ret
        .endfunc


/* FUNCTION DEF: void CacheP_disableEL3(void) */
        .global CacheP_disableEL3
        .section .text.CacheP_disableEL3
        .func CacheP_disableEL3

CacheP_disableEL3:
        mrs     x0, sctlr_el3            /* read SCTLR_EL3 */
        bic     x0, x0, #0x0004          /* clear C bit */
        msr     sctlr_el3, x0            /* DCache disabled */

        mrs     x0, sctlr_el3            /* read SCTLR_EL3 */
        bic     x0, x0, #0x1000          /* clear I bit */
        msr     sctlr_el3, x0            /* ICache disabled */
        ic      iallu                    /* invalidate all ICache */
        dsb     sy
        isb
        ret
        .endfunc

#if defined (SMP_FREERTOS)
/* FUNCTION DEF: void CacheP_enableSMP(void) */
        .global CacheP_enableSMP
        .section .text.CacheP_enableSMP
        .func CacheP_enableSMP

CacheP_enableSMP:
        mrs     x0, s3_1_c15_c2_1           /* Read CPUECTLR_EL1 */
        orr     x0, x0, #0x40
        msr     s3_1_c15_c2_1, x0           /* Enable data coherency with other cores */
        ret
        .endfunc
#endif
