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

    .arm
@
@
    .global     _tx_thread_current_ptr
    .global     _tx_timer_time_slice
    .global     _tx_thread_schedule
    .global     _tx_execution_thread_exit
@
@/**************************************************************************/ 
@/*                                                                        */ 
@/*  FUNCTION                                               RELEASE        */ 
@/*                                                                        */ 
@/*    _tx_thread_system_return                          Cortex-R5/GNU     */ 
@/*                                                           6.1          */
@/*  AUTHOR                                                                */
@/*                                                                        */
@/*    William E. Lamie, Microsoft Corporation                             */
@/*                                                                        */
@/*  DESCRIPTION                                                           */
@/*                                                                        */ 
@/*    This function is target processor specific.  It is used to transfer */ 
@/*    control from a thread back to the ThreadX system.  Only a           */ 
@/*    minimal context is saved since the compiler assumes temp registers  */ 
@/*    are going to get slicked by a function call anyway.                 */ 
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
@/*    _tx_thread_schedule                   Thread scheduling loop        */ 
@/*                                                                        */ 
@/*  CALLED BY                                                             */ 
@/*                                                                        */ 
@/*    ThreadX components                                                  */ 
@/*                                                                        */ 
@/*  RELEASE HISTORY                                                       */ 
@/*                                                                        */ 
@/*    DATE              NAME                      DESCRIPTION             */
@/*                                                                        */
@/*  09-30-2020     William E. Lamie         Initial Version 6.1           */
@/*                                                                        */
@/**************************************************************************/
@VOID   _tx_thread_system_return(VOID)
@{
    .global _tx_thread_system_return
    .type   _tx_thread_system_return,function
_tx_thread_system_return:
@
@   /* Lockout interrupts.  */
@
    MRS     r1, CPSR                            @ Pickup the CPSR
    CPSID   i                                   @ Disable IRQ interrupts
@    /* Save minimal context on the stack.  */
@
    STMDB   sp!, {r4-r11, lr}                   @ Save minimal context
    LDR     r5, =_tx_thread_current_ptr         @ Pickup address of current ptr
    LDR     r6, [r5, #0]                        @ Pickup current thread pointer
@
#if !defined(__SOFT_FP__) && defined(__ARM_FP)
    VMRS    r4, FPSCR                           @ Pickup the FPSCR
    STR     r4, [sp, #-4]!                      @ Save FPSCR
    VSTMDB  sp!, {D0-D15}                       @ Save D8-D15
#endif // #if !defined(__SOFT_FP__) && defined(__ARM_FP)
@
    MOV     r0, #0                              @ Build a solicited stack type
    STMDB   sp!, {r0-r1}                        @ Save type and CPSR
@   
@
#ifdef TX_ENABLE_EXECUTION_CHANGE_NOTIFY
@
@    /* Call the thread exit function to indicate the thread is no longer executing.  */
@
    BL      _tx_execution_thread_exit           @ Call the thread exit function
#endif
@
    LDR     r2, =_tx_timer_time_slice           @ Pickup address of time slice
    LDR     r1, [r2, #0]                        @ Pickup current time slice
@
@    /* Save current stack and switch to system stack.  */
@    _tx_thread_current_ptr -> tx_thread_stack_ptr =  sp;
@    sp = _tx_thread_system_stack_ptr;
@
    STR     sp, [r6, #8]                        @ Save thread stack pointer
@
@    /* Determine if the time-slice is active.  */
@    if (_tx_timer_time_slice)
@    {
@
    MOV     r4, #0                              @ Build clear value
    CMP     r1, #0                              @ Is a time-slice active?
    BEQ     __tx_thread_dont_save_ts            @ No, don't save the time-slice
@
@       /* Save time-slice for the thread and clear the current time-slice.  */
@       _tx_thread_current_ptr -> tx_thread_time_slice =  _tx_timer_time_slice;
@       _tx_timer_time_slice =  0;
@
    STR     r4, [r2, #0]                        @ Clear time-slice
    STR     r1, [r6, #24]                       @ Save current time-slice
@
@    }
__tx_thread_dont_save_ts:
@
@    /* Clear the current thread pointer.  */
@    _tx_thread_current_ptr =  TX_NULL;
@
    STR     r4, [r5, #0]                        @ Clear current thread pointer
    B       _tx_thread_schedule                 @ Jump to scheduler!
@
@}

