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

SYS_MODE        =       0x1F                    @ SYS mode
CPSR_MASK       =       0x9F                    @ Mask initial CPSR, IRQ interrupts enabled
@
@
@/**************************************************************************/ 
@/*                                                                        */ 
@/*  FUNCTION                                               RELEASE        */ 
@/*                                                                        */ 
@/*    _tx_thread_stack_build                            Cortex-R5/GNU     */ 
@/*                                                           6.1          */
@/*  AUTHOR                                                                */
@/*                                                                        */
@/*    William E. Lamie, Microsoft Corporation                             */
@/*                                                                        */
@/*  DESCRIPTION                                                           */
@/*                                                                        */ 
@/*    This function builds a stack frame on the supplied thread's stack.  */
@/*    The stack frame results in a fake interrupt return to the supplied  */
@/*    function pointer.                                                   */ 
@/*                                                                        */ 
@/*  INPUT                                                                 */ 
@/*                                                                        */ 
@/*    thread_ptr                            Pointer to thread control blk */
@/*    function_ptr                          Pointer to return function    */
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
@/*    _tx_thread_create                     Create thread service         */
@/*                                                                        */ 
@/*  RELEASE HISTORY                                                       */ 
@/*                                                                        */ 
@/*    DATE              NAME                      DESCRIPTION             */
@/*                                                                        */
@/*  09-30-2020     William E. Lamie         Initial Version 6.1           */
@/*                                                                        */
@/**************************************************************************/
@VOID   _tx_thread_stack_build(TX_THREAD *thread_ptr, VOID (*function_ptr)(VOID))
@{
    .global tx_threadx_stack_build_cpsr_get
    .global  _tx_thread_stack_build
    .type    _tx_thread_stack_build,function
_tx_thread_stack_build:
@
@       
@    /* Build a fake interrupt frame.  The form of the fake interrupt stack
@       on the ARM9 should look like the following after it is built:
@       
@       Stack Top:      1           Interrupt stack frame type
@                       CPSR        Initial value for CPSR
@                       a1 (r0)     Initial value for a1
@                       a2 (r1)     Initial value for a2
@                       a3 (r2)     Initial value for a3
@                       a4 (r3)     Initial value for a4
@                       v1 (r4)     Initial value for v1
@                       v2 (r5)     Initial value for v2
@                       v3 (r6)     Initial value for v3
@                       v4 (r7)     Initial value for v4
@                       v5 (r8)     Initial value for v5
@                       sb (r9)     Initial value for sb
@                       sl (r10)    Initial value for sl
@                       fp (r11)    Initial value for fp
@                       ip (r12)    Initial value for ip
@                       lr (r14)    Initial value for lr
@                       pc (r15)    Initial value for pc
@                       0       For stack backtracing
@
@    Stack Bottom: (higher memory address)  */
@

    PUSH {r4-r7}                            @ Save some working registers.

@
@   Fetch the CPSR and thumb bit from the C helper function.
@   This is done to support both thumb and arm builds.
    PUSH {r0-r3, lr}
    BL tx_threadx_stack_build_cpsr_get
    MOV r4, r0
    POP {r0-r3, lr}

    LDR     r2, [r0, #16]                   @ Pickup end of stack area
    BIC     r2, r2, #7                      @ Ensure 8-byte alignment
#if !defined(__SOFT_FP__) && defined(__ARM_FP)
    SUB     r2, r2, #204                    @ Allocate space for the stack frame
#define FP_OFF 132
#else
    SUB     r2, r2, #72                     @ Allocate space for the stack frame
#define FP_OFF 0
#endif // #if !defined(__SOFT_FP__) && defined(__ARM_FP)
@
@    /* Actually build the stack frame.  */
@
    MOV     r3, #1                          @ Build interrupt stack type
    STR     r3, [r2, #0]                    @ Store stack type

    MOV     r6, #0x0                        @ Build initial VFP register value
    MOV     r7, #0x0

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
    STRD     r6, r7, [r2, #(4+0)]           @ Store cleared VFP register values.
    STRD     r6, r7, [r2, #(4+8)]
    STRD     r6, r7, [r2, #(4+16)]
    STRD     r6, r7, [r2, #(4+24)]
    STRD     r6, r7, [r2, #(4+32)]
    STRD     r6, r7, [r2, #(4+40)]
    STRD     r6, r7, [r2, #(4+48)]
    STRD     r6, r7, [r2, #(4+56)]
    STRD     r6, r7, [r2, #(4+64)]
    STRD     r6, r7, [r2, #(4+72)]
    STRD     r6, r7, [r2, #(4+80)]
    STRD     r6, r7, [r2, #(4+88)]
    STRD     r6, r7, [r2, #(4+96)]
    STRD     r6, r7, [r2, #(4+104)]
    STRD     r6, r7, [r2, #(4+112)]
    STRD     r6, r7, [r2, #(4+120)]
    STR     r6, [r2, #(4+128)]                       @ Save initial FPSCR value
#endif // #if !defined(__SOFT_FP__) && defined(__ARM_FP)

    MOV     r3, #0x0                                 @ Build initial register value
    STR     r3, [r2, #(4+FP_OFF)]                    @ Store initial r0
    STR     r3, [r2, #(8+FP_OFF)]                    @ Store initial r1
    STR     r3, [r2, #(12+FP_OFF)]                   @ Store initial r2
    STR     r3, [r2, #(16+FP_OFF)]                   @ Store initial r3
    STR     r3, [r2, #(20+FP_OFF)]                   @ Store initial r4
    STR     r3, [r2, #(24+FP_OFF)]                   @ Store initial r5
    STR     r3, [r2, #(28+FP_OFF)]                   @ Store initial r6
    STR     r3, [r2, #(32+FP_OFF)]                   @ Store initial r7
    STR     r3, [r2, #(36+FP_OFF)]                   @ Store initial r8
    STR     r3, [r2, #(40+FP_OFF)]                   @ Store initial r9

    LDR     r3, [r0, #(12)]                   @ Pickup stack starting address
    STR     r3, [r2, #(44+FP_OFF)]                   @ Store initial r10 (sl)

    STR     r3, [r2, #(48+FP_OFF)]                   @ Store initial r11
    STR     r3, [r2, #(52+FP_OFF)]                   @ Store initial r12

    LDR     r3,=_tx_thread_schedule         @ Pickup address of _tx_thread_schedule for GDB backtrace
    STR     r3, [r2, #(56+FP_OFF)]                   @ Store initial r14 (lr)

    STR     r1, [r2, #(60+FP_OFF)]                   @ Store initial pc

    BIC     r1, r4, #CPSR_MASK              @ Mask mode bits of CPSR
    ORR     r3, r1, #SYS_MODE               @ Build CPSR, SYS mode, interrupts enabled
    STR     r3, [r2, #(64+FP_OFF)]                   @ Store initial CPSR

    STR     r3, [r2, #(68+FP_OFF)]                   @ 0 for back-trace
@
@    /* Setup stack pointer.  */
@    thread_ptr -> tx_thread_stack_ptr =  r2;
@
    STR     r2, [r0, #8]                    @ Save stack pointer in thread's
                                            @   control block

    POP {r4-r7}                             @ Restore working registers
    BX      lr                              @ Return to caller
@}


