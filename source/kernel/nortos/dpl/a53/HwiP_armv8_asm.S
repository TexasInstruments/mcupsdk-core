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

#define HWIP_GICX_OFFSET    0x0020000

/* FUNCTION DEF: uintptr_t HwiP_disable(void) */
        .global HwiP_disable
        .type   HwiP_disable, %function
HwiP_disable:
         mrs x0, daif
         msr daifset, #2
         ret

/* FUNCTION DEF: void HwiP_enable(void) */
        .global HwiP_enable
        .type   HwiP_enable, %function
HwiP_enable:
         mrs x0, daif
         msr daifclr, #2
         ret

/* FUNCTION DEF: void HwiP_restore(uintptr_t key) */
        .global HwiP_restore
        .type   HwiP_restore, %function
HwiP_restore:
         and x2, x0, #128
         mrs x1, daif
         bic x1, x1, #128
         orr x1, x1, x2
         msr daif, x1
         ret


    .global HwiP_armv8GetGicxAddr
    .section .text.csl_startup, "ax"
    .func HwiP_armv8GetGicxAddr

HwiP_armv8GetGicxAddr:
    ldr     x1, =HWIP_GICX_OFFSET
    mrs     x2, mpidr_el1
    ubfx    x3, x2, #8, #8      /* x3 = Cluster Id */
    and     x2, x2, #0xFF       /* x2 = Core Id */
    sub     x3, x3, #0
    mrs     x4, s3_1_c11_c0_2   /* Read L2CTLR_EL1 */
    ubfx    x4, x4, #24, #2     /* x4 = Number of cores per cluster */
    lsl     x3, x3, x4          /* x3 = clusterIdx * numCoresPerCluster */
    add     x2, x2, x3
    madd    x0, x1, x2, x0
    ret
    .endfunc