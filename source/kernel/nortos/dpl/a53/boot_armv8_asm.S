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

//************************** Global symbols ************************************
        .global _c_int00
        .global _stack
        .global _bss_start
        .global _bss_end
        .global __system_start
        .global HwiP_gicv3Vectors
        .global CacheP_wbInvAll
        .global CacheP_disableEL3
        .global HwiP_armv8GetGicxAddr

#define HWIP_GIC_BASE_ADDR      (0x1800000U)
#define HWIP_GICR_BASE_ADDR     (0x1840000U)
#define HWIP_GICS_BASE_ADDR     (0x1850000U)

.macro GICD_WRITE_LOOP x, y, offset
    str     w1, [x0, #\offset]
    .if \y-\x
    GICD_WRITE_LOOP "(\x+1)", \y, "(\offset+4)"
    .endif
.endm


.macro INIT_GICD_IGROUPR
    ldr     x0, =HWIP_GIC_BASE_ADDR
    mvn     w1, wzr
    GICD_WRITE_LOOP 0, 31, 0x0080
.endm

.macro INIT_GICD_IGRPMODR
    ldr     x0, =HWIP_GIC_BASE_ADDR
    mov     w1, wzr
    GICD_WRITE_LOOP 0, 31, 0x0D00
.endm


    .section .text.boot, "ax"
    .func _c_int00
_c_int00:
    mrs     x0, currentel
    cmp     x0, #0xC
    bne     2f
    ldr     x0, =CacheP_wbInvAll
    blr     x0
    ldr     x0, =CacheP_disableEL3
    blr     x0
    /*
     * ----------------------
     * Switch from EL3 to EL2
     * ----------------------
     */
    mov     x0, #0x0531
    msr     scr_el3, x0
    msr     cptr_el3, xzr       /* Disable all trap bits */
    mov     x0, #0x33ff
    msr     cptr_el2, x0        /* Disable all trap bits */
    ldr     x0, =HWIP_GIC_BASE_ADDR
    mov     w1, #0x37
    str     w1, [x0]            /* Enable GIC */
    mov     x0, #0xf
    msr     s3_6_c12_c12_5, x0     /* Enable GIC system register interface
                                   and disable IRQ/FIQ bypass */
    isb
    mov     x0, #0x1
    msr     s3_6_c12_c12_7, x0 /* Enable Non-secure group 1 interrupts */
    isb
    INIT_GICD_IGROUPR
    INIT_GICD_IGRPMODR
    ldr     x0, =HWIP_GICR_BASE_ADDR
    ldr     x1, =HwiP_armv8GetGicxAddr
    blr     x1
    ldr     w1, [x0, #0x14]
    mov     w2, #0x2
    bic     w1, w1, w2
    str     w1, [x0, #0x14]     /* wakeup GICR */
1:  ldr     w1, [x0, #0x14]
    and     w1, w1, #0x4
    cbnz    w1, 1b
    ldr     x0, =HWIP_GICS_BASE_ADDR
    ldr     x1, =HwiP_armv8GetGicxAddr
    blr     x1
    mvn     x1, xzr
    str     w1, [x0, #0x80]     /* Configure all SGIs & PPIs as Group 1 ints */
    str     wzr, [x0, #0xD00]   /* Clear GICR_IGRPMODR0 */
    mov     x0, #0xff
    msr     s3_0_c4_c6_0, x0     /* Set priority mask icc_pmr_el1 */
    mov     x0, #0x0830
    movk    x0, #0x30c5, lsl #16
    msr     sctlr_el2, x0       /* Initialize sctlr_el2 to reset values */
    mrs     x0, actlr_el3
    orr     x0, x0, #0x2
    msr     actlr_el3, x0       /* Enable CPUECTLR_EL1 access from EL2 */
    isb
    bl      switch_to_el2

2:
    mrs     x0, currentel       /* Read again as currentEL may have changed */
    cmp     x0, #0x8
    bne     3f

    /*
     * ----------------------
     * Switch from EL2 to EL1
     * ----------------------
     */
    mrs     x0, cnthctl_el2
    orr     x0, x0, #0x3        /* Enable EL1/EL0 access to timers */
    msr     cnthctl_el2, x0
    msr     cntvoff_el2, xzr
    mov     x0, #0x33ff
    msr     cptr_el2, x0        /* Disable all trap bits */
    msr     hstr_el2, xzr       /* Disable all coprocessor trap bits */
    mov     x0, #0x0002         /* 64bit EL1, Disable hypervisor call (HVC)
                                   instruction, Set/Way Invalidation Override */
    movk    x0, #0xA000, lsl #16
    msr     hcr_el2, x0
    mov     x0, #0x0838
    movk    x0, #0x30d0, lsl #16
    msr     sctlr_el1, x0
    mrs     x0, actlr_el2
    orr     x0, x0, #2
    msr     actlr_el2, x0
    isb
    mov     x0, #0xF
    msr     s3_4_c12_c9_5, x0     /* Enable GIC system register interface
                                   and disable IRQ/FIQ bypass (icc_sre_el2) */
    bl      switch_to_el1
3:
    ldr     x0, =gnu_targets_arm_rtsv8A_startupAsm
    br      x0

switch_to_el1:
    mov     x0, #0x3c5
    msr     spsr_el2, x0
    msr     elr_el2, x30
    eret

switch_to_el2:
    mov     x0, #0x3c9
    msr     spsr_el3, x0
    msr     elr_el3, x30
    eret

    .endfunc


    .section .text.boot, "ax"
    .func gnu_targets_arm_rtsv8A_startupAsm
gnu_targets_arm_rtsv8A_startupAsm:
    /*
     * ---------------------
     * Boot code starts here
     * ---------------------
     */

    mov     x0, #0x3C0
    msr     daif, x0            /* Mask all interrupts */

#if defined(__ARM_FP)
    mov     x0, #(3 << 20)
    msr     cpacr_el1, x0       /* Enable FP/SIMD at EL1 */
#endif

    isb                         /* Synchronize processor context */

    /*
     * ------------------------
     * Initialize stack pointer
     * ------------------------
     */
    msr     spsel, #1           /* Use SP_ELx for ELx */
#if defined(SMP_FREERTOS)
    /* Initialize stack for each cores */
    mrs     x0, mpidr_el1
    and     x0, x0, #0xff
    cmp     x0, #1
    beq      core1_stackinit

core0_stackinit:
    ldr     x0, =__TI_STACK_BASE
    ldr     x1, =__TI_STACK_SIZE
    add     x0, x0, x1
    and     x0, x0, #(~0xf)     /* 16-byte align SP */
    mov     sp, x0              /* Initialize SP */
    b       stack_initend

core1_stackinit:
    ldr     x0, =__TI_STACK_BASE1
    ldr     x1, =__TI_STACK_SIZE
    add     x0, x0, x1
    and     x0, x0, #(~0xf)     /* 16-byte align SP */
    mov     sp, x0              /* Initialize SP */
    b       stack_initend

stack_initend:

#else
    ldr     x0, =__TI_STACK_BASE
    ldr     x1, =__TI_STACK_SIZE
    add     x0, x0, x1
    and     x0, x0, #(~0xf)     /* 16-byte align SP */
    mov     sp, x0              /* Initialize SP */
#endif
    /* Setup the vector table for all levels */
    ldr     x0, =HwiP_gicv3Vectors
    msr     vbar_el1, x0         /* Set vector table base address */

    /* do more initialization in C, go to main() */
    ldr     x0, =__system_start
    blr     x0

    .endfunc

    .end
