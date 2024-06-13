@/***************************************************************************
@ * Copyright (c) 2024 Microsoft Corporation 
@ * 
@ * This program and the accompanying materials are made available under the
@ * terms of the MIT License which is available at
@ * https://opensource.org/licenses/MIT.
@ * 
@ * SPDX-License-Identifier: MIT
@ **************************************************************************/


@  Copyright (C) 2024 Texas Instruments Incorporated
@
@  Redistribution and use in source and binary forms, with or without
@  modification, are permitted provided that the following conditions
@  are met:
@
@    Redistributions of source code must retain the above copyright
@    notice, this list of conditions and the following disclaimer.
@
@    Redistributions in binary form must reproduce the above copyright
@    notice, this list of conditions and the following disclaimer in the
@    documentation and/or other materials provided with the
@    distribution.
@
@    Neither the name of Texas Instruments Incorporated nor the names of
@    its contributors may be used to endorse or promote products derived
@    from this software without specific prior written permission.
@
@  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
@  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
@  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
@  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
@  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
@  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
@  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
@  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
@  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
@  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
@  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


@
@
@/**************************************************************************/
@/**************************************************************************/
@/**                                                                       */ 
@/** ThreadX Component                                                     */ 
@/**                                                                       */
@/**   Thread                                                              */
@/**                                                                       */
@/**************************************************************************/
@/**************************************************************************/
#ifdef TX_INCLUDE_USER_DEFINE_FILE
#include "tx_user.h"
#endif

    .arm

SYS_MODE        =     0x9F               @ Disable IRQ, SYS mode
IRQ_MODE        =     0x92               @ Disable IRQ, IRQ mode
@
    .global     _tx_thread_system_state
    .global     _tx_thread_current_ptr
    .global     _tx_thread_execute_ptr
    .global     _tx_timer_time_slice
    .global     _tx_thread_schedule
    .global     _tx_thread_preempt_disable
    .global     _tx_execution_isr_exit
@
@
@/* No 16-bit Thumb mode veneer code is needed for _tx_thread_context_restore
@   since it will never be called 16-bit mode.  */
@ 
    .arm
    .text
    .align 2
@/**************************************************************************/ 
@/*                                                                        */ 
@/*  FUNCTION                                               RELEASE        */ 
@/*                                                                        */ 
@/*    _tx_thread_context_restore                        Cortex-R5/GNU     */ 
@/*                                                           6.1          */
@/*  AUTHOR                                                                */
@/*                                                                        */
@/*    William E. Lamie, Microsoft Corporation                             */
@/*                                                                        */
@/*  DESCRIPTION                                                           */
@/*                                                                        */ 
@/*    This function restores the interrupt context if it is processing a  */ 
@/*    nested interrupt.  If not, it returns to the interrupt thread if no */ 
@/*    preemption is necessary.  Otherwise, if preemption is necessary or  */ 
@/*    if no thread was running, the function returns to the scheduler.    */ 
@/*                                                                        */ 
@/*  INPUT                                                                 */ 
@/*                                                                        */ 
@/*    None                                                                */ 
@/*                                                                        */ 
@/*  OUTPUT                                                                */ 
@/*                                                                        */ 
@/*    None                                                                */ 
@/*                                                                        */ 
@/*  CALLS                                                                 */ 
@/*                                                                        */ 
@/*    _tx_thread_schedule                   Thread scheduling routine     */ 
@/*                                                                        */ 
@/*  CALLED BY                                                             */ 
@/*                                                                        */ 
@/*    ISRs                                  Interrupt Service Routines    */ 
@/*                                                                        */ 
@/*  RELEASE HISTORY                                                       */ 
@/*                                                                        */ 
@/*    DATE              NAME                      DESCRIPTION             */
@/*                                                                        */
@/*  09-30-2020     William E. Lamie         Initial Version 6.1           */
@/*                                                                        */
@/**************************************************************************/
@VOID   _tx_thread_context_restore(VOID)
@{
    .global _tx_thread_context_restore
    .type   _tx_thread_context_restore,function
_tx_thread_context_restore:
@
@    /* Lockout interrupts.  */
@
    CPSID   i                               @ Disable IRQ interrupts

#ifdef TX_ENABLE_EXECUTION_CHANGE_NOTIFY
@
@    /* Call the ISR exit function to indicate an ISR is complete.  */
@
    BL      _tx_execution_isr_exit          @ Call the ISR exit function
#endif
@
@    /* Determine if interrupts are nested.  */
@    if (--_tx_thread_system_state)
@    {
@
    LDR     r3, =_tx_thread_system_state    @ Pickup address of system state variable
    LDR     r2, [r3]                        @ Pickup system state
    SUB     r2, r2, #1                      @ Decrement the counter
    STR     r2, [r3]                        @ Store the counter 
    CMP     r2, #0                          @ Was this the first interrupt?
    BEQ     __tx_thread_not_nested_restore  @ If so, not a nested restore
@
@    /* Interrupts are nested.  */
@
@    /* Just recover the saved registers and return to the point of 
@       interrupt.  */
@
    LDMIA   sp!, {r0, r10, r12, lr}         @ Recover SPSR, POI, and scratch regs
    MSR     SPSR_cxsf, r0                   @ Put SPSR back
    LDMIA   sp!, {r0-r3}                    @ Recover r0-r3
    MOVS    pc, lr                          @ Return to point of interrupt
@
@    }
__tx_thread_not_nested_restore:
@
@    /* Determine if a thread was interrupted and no preemption is required.  */
@    else if (((_tx_thread_current_ptr) && (_tx_thread_current_ptr == _tx_thread_execute_ptr))
@               || (_tx_thread_preempt_disable))
@    {
@
    LDR     r1, =_tx_thread_current_ptr     @ Pickup address of current thread ptr
    LDR     r0, [r1]                        @ Pickup actual current thread pointer
    CMP     r0, #0                          @ Is it NULL?
    BEQ     __tx_thread_idle_system_restore @ Yes, idle system was interrupted
@
    LDR     r3, =_tx_thread_preempt_disable @ Pickup preempt disable address
    LDR     r2, [r3]                        @ Pickup actual preempt disable flag
    CMP     r2, #0                          @ Is it set?
    BNE     __tx_thread_no_preempt_restore  @ Yes, don't preempt this thread
    LDR     r3, =_tx_thread_execute_ptr     @ Pickup address of execute thread ptr
    LDR     r2, [r3]                        @ Pickup actual execute thread pointer
    CMP     r0, r2                          @ Is the same thread highest priority?
    BNE     __tx_thread_preempt_restore     @ No, preemption needs to happen
@
@
__tx_thread_no_preempt_restore:
@
@    /* Restore interrupted thread or ISR.  */
@
@    /* Pickup the saved stack pointer.  */
@    tmp_ptr =  _tx_thread_current_ptr -> tx_thread_stack_ptr;
@
@   /* Recover the saved context and return to the point of interrupt.  */
@
    LDMIA   sp!, {r0, r10, r12, lr}         @ Recover SPSR, POI, and scratch regs
    MSR     SPSR_cxsf, r0                   @ Put SPSR back
    LDMIA   sp!, {r0-r3}                    @ Recover r0-r3
    MOVS    pc, lr                          @ Return to point of interrupt
@
@    }
@    else
@    {
__tx_thread_preempt_restore:
@
    LDMIA   sp!, {r3, r10, r12, lr}         @ Recover temporarily saved registers
    MOV     r1, lr                          @ Save lr (point of interrupt)
    MOV     r2, #SYS_MODE                   @ Build SYS mode CPSR
    MSR     CPSR_c, r2                      @ Enter SYS mode

    STMDB   sp!, {r3}                       @ SPSR save.
    STMDB   sp!, {r1}                       @ Save point of interrupt

    STMDB   sp!, {r4-r12, lr}               @ Save upper half of registers
    MOV     r4, r3                          @ Save SPSR in r4
    MOV     r2, #IRQ_MODE                   @ Build IRQ mode CPSR
    MSR     CPSR_c, r2                      @ Enter IRQ mode
    LDMIA   sp!, {r0-r3}                    @ Recover r0-r3
    MOV     r5, #SYS_MODE                   @ Build SYS mode CPSR
    MSR     CPSR_c, r5                      @ Enter SYS mode
    STMDB   sp!, {r0-r3}                    @ Save r0-r3 on thread's stack
@
    LDR     r1, =_tx_thread_current_ptr     @ Pickup address of current thread ptr
    LDR     r0, [r1]                        @ Pickup current thread pointer
@
#if !defined(__SOFT_FP__) && defined(__ARM_FP)
    VMRS    r2, FPSCR                       @ Pickup the FPSCR
    STR     r2, [sp, #-4]!                  @ Save FPSCR
    VSTMDB  sp!, {D0-D15}                   @ Save D0-D15
#endif // #if !defined(__SOFT_FP__) && defined(__ARM_FP)
@
    MOV     r3, #1                          @ Build interrupt stack type
    STMDB   sp!, {r3}                       @ Save interrupt stack type
    STR     sp, [r0, #8]                    @ Save stack pointer in thread control
                                            @   block
@
@    /* Save the remaining time-slice and disable it.  */
@    if (_tx_timer_time_slice)
@    {
@
    LDR     r3, =_tx_timer_time_slice       @ Pickup time-slice variable address
    LDR     r2, [r3]                        @ Pickup time-slice
    CMP     r2, #0                          @ Is it active?
    BEQ     __tx_thread_dont_save_ts        @ No, don't save it
@
@        _tx_thread_current_ptr -> tx_thread_time_slice =  _tx_timer_time_slice;
@        _tx_timer_time_slice =  0;
@
    STR     r2, [r0, #24]                   @ Save thread's time-slice
    MOV     r2, #0                          @ Clear value
    STR     r2, [r3]                        @ Disable global time-slice flag
@
@    }
__tx_thread_dont_save_ts:
@
@
@    /* Clear the current task pointer.  */
@    _tx_thread_current_ptr =  TX_NULL;
@
    MOV     r0, #0                          @ NULL value
    STR     r0, [r1]                        @ Clear current thread pointer
@
@    /* Return to the scheduler.  */
@    _tx_thread_schedule();
@
    B       _tx_thread_schedule             @ Return to scheduler
@    }
@
__tx_thread_idle_system_restore:
@
@    /* Just return back to the scheduler!  */
@
    MOV     r0, #SYS_MODE                   @ Build SYS mode CPSR
    MSR     CPSR_c, r0                      @ Enter SYS mode
    B       _tx_thread_schedule             @ Return to scheduler
@}



