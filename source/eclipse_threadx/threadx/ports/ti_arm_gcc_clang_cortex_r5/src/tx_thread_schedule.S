@/***************************************************************************
@ * Copyright (c) 2024 Microsoft Corporation 
@ * 
@ * This program and the accompanying materials are made available under the
@ * terms of the MIT License which is available at
@ * https://opensource.org/licenses/MIT.
@ * 
@ * SPDX-License-Identifier: MIT
@ **************************************************************************/
@
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

    .global     _tx_thread_execute_ptr
    .global     _tx_thread_current_ptr
    .global     _tx_timer_time_slice
    .global     _tx_execution_thread_enter
@
@/**************************************************************************/ 
@/*                                                                        */ 
@/*  FUNCTION                                               RELEASE        */ 
@/*                                                                        */ 
@/*    _tx_thread_schedule                               Cortex-R5/GNU     */ 
@/*                                                           6.1          */
@/*  AUTHOR                                                                */
@/*                                                                        */
@/*    William E. Lamie, Microsoft Corporation                             */
@/*                                                                        */
@/*  DESCRIPTION                                                           */
@/*                                                                        */ 
@/*    This function waits for a thread control block pointer to appear in */ 
@/*    the _tx_thread_execute_ptr variable.  Once a thread pointer appears */ 
@/*    in the variable, the corresponding thread is resumed.               */ 
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
@/*    None                                                                */
@/*                                                                        */ 
@/*  CALLED BY                                                             */ 
@/*                                                                        */ 
@/*    _tx_initialize_kernel_enter          ThreadX entry function         */ 
@/*    _tx_thread_system_return             Return to system from thread   */ 
@/*    _tx_thread_context_restore           Restore thread's context       */ 
@/*                                                                        */ 
@/*  RELEASE HISTORY                                                       */ 
@/*                                                                        */ 
@/*    DATE              NAME                      DESCRIPTION             */
@/*                                                                        */
@/*  09-30-2020     William E. Lamie         Initial Version 6.1           */
@/*                                                                        */
@/**************************************************************************/
@VOID   _tx_thread_schedule(VOID)
@{
    .global _tx_thread_schedule
        .type  _tx_thread_schedule,function
_tx_thread_schedule:
@
@    /* Enable interrupts.  */
@
    CPSIE   i                                   @ Enable IRQ interrupts
@
@    /* Wait for a thread to execute.  */
@    do
@    {
    LDR     r1, =_tx_thread_execute_ptr         @ Address of thread execute ptr
@
__tx_thread_schedule_loop:
@
    LDR     r0, [r1]                            @ Pickup next thread to execute
    CMP     r0, #0                              @ Is it NULL?
    BEQ     __tx_thread_schedule_loop           @ If so, keep looking for a thread
@
@    }
@    while(_tx_thread_execute_ptr == TX_NULL);
@    
@    /* Yes! We have a thread to execute.  Lockout interrupts and
@       transfer control to it.  */
@
    CPSID   i                                   @ Disable IRQ interrupts
@
@    /* Setup the current thread pointer.  */
@    _tx_thread_current_ptr =  _tx_thread_execute_ptr;
@
    LDR     r1, =_tx_thread_current_ptr         @ Pickup address of current thread 
    STR     r0, [r1]                            @ Setup current thread pointer
@
@    /* Increment the run count for this thread.  */
@    _tx_thread_current_ptr -> tx_thread_run_count++;
@
    LDR     r2, [r0, #4]                        @ Pickup run counter
    LDR     r3, [r0, #24]                       @ Pickup time-slice for this thread
    ADD     r2, r2, #1                          @ Increment thread run-counter
    STR     r2, [r0, #4]                        @ Store the new run counter
@
@    /* Setup time-slice, if present.  */
@    _tx_timer_time_slice =  _tx_thread_current_ptr -> tx_thread_time_slice;
@
    LDR     r2, =_tx_timer_time_slice           @ Pickup address of time-slice 
                                                @   variable
    LDR     sp, [r0, #8]                        @ Switch stack pointers
    STR     r3, [r2]                            @ Setup time-slice
@
@    /* Switch to the thread's stack.  */
@    sp =  _tx_thread_execute_ptr -> tx_thread_stack_ptr;
@
#ifdef TX_ENABLE_EXECUTION_CHANGE_NOTIFY
@
@    /* Call the thread entry function to indicate the thread is executing.  */
@
    BL      _tx_execution_thread_enter      @ Call the thread execution enter function
#endif
@
@    /* Determine if an interrupt frame or a synchronous task suspension frame
@   is present.  */
@
    LDMIA   sp!, {r4}                           @ Pickup the stack type and saved CPSR
    CMP     r4, #0                              @ Check for synchronous context switch
    BEQ     _tx_solicited_return
#if !defined(__SOFT_FP__) && defined(__ARM_FP)
    VLDMIA  sp!, {D0-D15}                       @ Recover D0-D15
    LDR     r4, [sp], #4                        @ Pickup FPSCR
    VMSR    FPSCR, r4                           @ Restore FPSCR
#endif // #if !defined(__SOFT_FP__) && defined(__ARM_FP)
    LDMIA   sp!, {r0-r12, lr}                   @ Return to point of thread interrupt
    RFEIA   sp!
@
_tx_solicited_return:
    LDMIA   sp!, {r0} // CPSR.
#if !defined(__SOFT_FP__) && defined(__ARM_FP)
    VLDMIA  sp!, {D0-D15}                       @ Recover D8-D15
    LDR     r4, [sp], #4                        @ Pickup FPSCR
    VMSR    FPSCR, r4                           @ Restore FPSCR
#endif // #if !defined(__SOFT_FP__) && defined(__ARM_FP)
    LDMIA   sp!, {r4-r11, lr}                   @ Return to thread synchronously
    MSR     CPSR_cxsf, r0                       @ Recover CPSR
@
    BX      lr                                  @ Return to caller
@
@}
@
