/*
 * FreeRTOS Kernel V10.4.1
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * https://www.FreeRTOS.org
 * https://github.com/FreeRTOS
 *
 * 1 tab == 4 spaces!
 */
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

/* make this 0 to disable FPU context save/restore */
#define ENABLE_FPU_SAVE_RESTORE (1)

        .text
        .arm

        .set SYS_MODE,  0x1f
        .set SVC_MODE,  0x13
        .set IRQ_MODE,  0x12

.macro portSAVE_CONTEXT
        /* Save the LR and SPSR onto the system mode stack before switching to
        system mode to save the remaining system mode registers. */
        SRSDB   sp!, #SYS_MODE
        CPS     #SYS_MODE
        PUSH    {R0-R12, R14}

        /* Push the critical nesting count. */
        LDR     R2, ulCriticalNestingConst
        LDR     R1, [R2]
        PUSH    {R1}

        /* Does the task have a floating point context that needs saving?  If
        ulPortTaskHasFPUContext is 0 then no. */
        LDR     R2, ulPortTaskHasFPUContextConst
        LDR     R3, [R2]
        CMP     R3, #0

        /* Save the floating point context, if any. */
        FMRXNE  R1,  FPSCR
        VPUSHNE {D0-D15}
        /* VPUSHNE      {D16-D31} */
        PUSHNE  {R1}

        /* Save ulPortTaskHasFPUContext itself. */
        PUSH    {R3}

        /* Save the stack pointer in the TCB. */
        LDR     R0, pxCurrentTCBConst
        LDR     R1, [R0]
        STR     SP, [R1]

        .endm

.macro portRESTORE_CONTEXT
        /* Set the SP to point to the stack of the task being restored. */
        LDR     R0, pxCurrentTCBConst
        LDR     R1, [R0]
        LDR     SP, [R1]

        /* Is there a floating point context to restore?  If the restored
        ulPortTaskHasFPUContext is zero then no. */
        LDR     R0, ulPortTaskHasFPUContextConst
        POP     {R1}
        STR     R1, [R0]
        CMP     R1, #0

        /* Restore the floating point context, if any. */
        POPNE   {R0}
        /* VPOPNE       {D16-D31} */
        VPOPNE  {D0-D15}
        VMSRNE  FPSCR, R0

        /* Restore the critical section nesting depth. */
        LDR     R0, ulCriticalNestingConst
        POP     {R1}
        STR     R1, [R0]

        /* reset local monitor, this is needed for atomics which use LDREX, STREX to work properly */
        CLREX

        /* Restore all system mode registers other than the SP (which is already
        being used). */
        POP     {R0-R12, R14}

        /* Return to the task code, loading CPSR on the way. */
        RFEIA   sp!

        .endm

/* FUNCTION DEF: void vPortRestoreTaskContext(void) */
        .global vPortRestoreTaskContext
        .type vPortRestoreTaskContext,%function
        .section ".text.hwi","ax",%progbits
        .arm
vPortRestoreTaskContext:
        /* Switch to system mode. */
        CPS     #SYS_MODE
        portRESTORE_CONTEXT

/* FUNCTION DEF: void HwiP_svc_handler(void) */
        .global HwiP_svc_handler
        .type HwiP_svc_handler,%function
        .section ".text.hwi","ax",%progbits
        .arm
HwiP_svc_handler:
        /* Save the context of the current task and select a new task to run. */
        portSAVE_CONTEXT
        LDR     R0, vTaskSwitchContextConst
        BLX     R0
        portRESTORE_CONTEXT

/* FUNCTION DEF: void HwiP_irq_handler(void) */
        .global HwiP_irq_handler
        .type HwiP_irq_handler,%function
        .section ".text.hwi","ax",%progbits
        .arm
HwiP_irq_handler:
        /* Return to the interrupted instruction. */
        SUB             lr, lr, #4

        /* Push the return address and SPSR. */
        PUSH    {lr}
        MRS     lr, SPSR
        PUSH    {lr}

        /* Change to supervisor mode to allow reentry. */
        CPS  #SVC_MODE

        /* Push used registers. */
        PUSH    {r0-r4, r12}

        /* Increment nesting count.  r3 holds the address of ulPortInterruptNesting
         * for future use.  r1 holds the original ulPortInterruptNesting value for
         * future use.
         */
        LDR     r3, ulPortInterruptNestingConst
        LDR     r1, [r3]
        ADD     r4, r1, #1
        STR     r4, [r3]

        #if ENABLE_FPU_SAVE_RESTORE
        FMRX  R0, FPSCR
        VPUSH {D0-D15}
        /* VPUSH {D16-D31} */
        PUSH  {R0}
        #endif

        /* Ensure bit 2 of the stack pointer is clear.  r2 holds the bit 2 value for
         * future use.  _RB_ Is this ever needed provided the start of the stack is
         * alligned on an 8-byte boundary?
         */
        MOV     r2, sp
        AND     r2, r2, #4
        SUB     sp, sp, r2

        /* Call the interrupt handler. */
        PUSH    {r0-r4, lr}
        LDR     r1, vApplicationIRQHandlerConst
        BLX     r1
        POP     {r0-r4, lr}
        ADD     sp, sp, r2

        /* disable IRQ */
        CPSID   i
        DSB
        ISB

        #if ENABLE_FPU_SAVE_RESTORE
        POP   {R0}
        /* VPOP {D16-D31} */
        VPOP  {D0-D15}
        VMSR  FPSCR, R0
        #endif

        /* Restore the old nesting count. */
        STR     r1, [r3]

        /* A context switch is never performed if the nesting count is not 0. */
        CMP     r1, #0
        BNE     exit_without_switch

        /* Did the interrupt request a context switch?  r1 holds the address of
         * ulPortYieldRequired and r0 the value of ulPortYieldRequired for future
         * use.
         */
        LDR     r1, =ulPortYieldRequired
        LDR     r0, [r1]
        CMP     r0, #0
        BNE     switch_before_exit

exit_without_switch:
        /* No context switch.  Restore used registers, LR_irq and SPSR before
         * returning.
         */
        POP     {r0-r4, r12}
        CPS  #IRQ_MODE
        POP     {LR}
        MSR     SPSR_cxsf, LR
        POP     {LR}
        MOVS    PC, LR

switch_before_exit:
        /* A context swtich is to be performed.  Clear the context switch pending
         * flag.
         */
        MOV     r0, #0
        STR     r0, [r1]

        /* Restore used registers, LR-irq and SPSR before saving the context
         * to the task stack.
         */
        POP     {r0-r4, r12}
        CPS  #IRQ_MODE
        POP     {LR}
        MSR     SPSR_cxsf, LR
        POP     {LR}
        portSAVE_CONTEXT

        /* Call the function that selects the new task to execute.
         * vTaskSwitchContext() if vTaskSwitchContext() uses LDRD or STRD
         * instructions, or 8 byte aligned stack allocated data.  LR does not need
         * saving as a new LR will be loaded by portRESTORE_CONTEXT anyway.
         */
        LDR     R0, vTaskSwitchContextConst
        BLX     R0

        /* Restore the context of, and branch to, the task selected to execute
         * next.
         */
        portRESTORE_CONTEXT

pxCurrentTCBConst: .word pxCurrentTCB
ulCriticalNestingConst: .word ulCriticalNesting
ulPortTaskHasFPUContextConst: .word ulPortTaskHasFPUContext
vTaskSwitchContextConst: .word vTaskSwitchContext
ulPortInterruptNestingConst: .word ulPortInterruptNesting
vApplicationIRQHandlerConst: .word HwiP_irq_handler_c

        .end
