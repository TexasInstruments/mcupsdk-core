;/*
; *  Copyright (C) 2021 Texas Instruments Incorporated
; *
; *  Redistribution and use in source and binary forms, with or without
; *  modification, are permitted provided that the following conditions
; *  are met:
; *
; *    Redistributions of source code must retain the above copyright
; *    notice, this list of conditions and the following disclaimer.
; *
; *    Redistributions in binary form must reproduce the above copyright
; *    notice, this list of conditions and the following disclaimer in the
; *    documentation and/or other materials provided with the
; *    distribution.
; *
; *    Neither the name of Texas Instruments Incorporated nor the names of
; *    its contributors may be used to endorse or promote products derived
; *    from this software without specific prior written permission.
; *
; *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
; *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
; *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
; *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
; *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
; *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
; *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
; *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
; *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
; *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
; *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
; */
;
; ======== port_tasksupport.asm ========
;
;

        .asg    b15, SP

STD_TARGWORDMAUS        .set 4
STD_TARGALIGN           .set 8

;
; 8-byte stack alignment needs to be maintained.
; Given the architecture of Task_start below, SWREGS must be an
; even number to maintain 8-byte alignment.
;
SWREGS  .set    10                      ; number of regs saved in _Task_switch

    .if $isdefed("__TI_ELFABI__")
    .if __TI_ELFABI__
        .asg TaskSupport_buildTaskStack, _TaskSupport_buildTaskStack
        .asg TaskSupport_glue, _TaskSupport_glue
        .asg TaskSupport_swap__E, _TaskSupport_swap__E
    .endif
    .endif

        .global _TaskSupport_buildTaskStack
        .global _TaskSupport_glue
        .global _TaskSupport_swap__E


;
;
;  Here is a pseudo image of the initial C64 task stack:
;
;   saved-by-callee registers
;   Task enter()
;   Task glue()
;   arg0
;   arg1
;   User's task func()
;   Task_exit()
;
;  The initial stack consists of the registers that are preserved by a
;  called C function as defined by the C compiler. These are the registers
;  pushed and popped by TaskSupport_swap.
;
;  Below (or above, depending on your view of the stack) those registers
;  are the Task enter() function which sets up the first invocation of the
;  task in the same manner as if it had been "returned" to from Task_restore()
;  which is the normal path back to an unblocked task.
;
;  Below the Task enter() function address on the initial task stack is the
;  address of the Glue function. This address is popped by the return from
;  Task enter(). Glue pops the 2 Args below it on the stack into the
;  appropriate registers for a fxn(arg0, arg1) call, and then "returns"
;  into the user's task function, which is the next to last address on
;  the stack.
;
;  Below the Task's function address on the stack is the Task_exit()
;  function address which is returned into if the task function falls
;  out the bottom.
;
;  The TaskSupport_swap() function pushes all the saved-by-callee
;  registers onto the task stack, then saves the updated SP into
;  the "old" task object's context (SP) address passed to it. Then it
;  loads the "new" task object's context (SP) (the second arg passed
;  to swap) into the SP, unrolls the saved registers and returns into
;  the new task.
;
;  The pointer returned by the buildTaskStack function is the task's initial
;  stack pointer. This address is stored into the task object's context
;  field and is passed to the TaskSupport_swap() function later.
;

;
;  ======== Task_buildTaskStack ========
;
;  Task_buildTaskStack(stack, fxn, argv, exit, entry)
;
;
;

;
;    sp = TaskSupport_buildTaskStack((Ptr)((size_t)params->stack + params->stackSize-8), params->fxn, params->exit, params->enter, params->arg0, params->arg1);
;
    .sect ".text:_TaskSupport_buildTaskStack"
    .clink
_TaskSupport_buildTaskStack:

STACK   .set    a4                      ; 1st argument
FXN     .set    b4                      ; 2nd argument
EXIT    .set    a6                      ; 4th argument
ENTRY   .set    b6                      ; 5th argument
ARG0    .set    a8
ARG1    .set    b8
RETVAL  .set    a4                      ; return value

        .asmfunc

        ; the following two values are used by glue
        stw     EXIT, *STACK--[1]               ; push exit address (TSK_exit)
        stw     FXN, *STACK--[1]                ; push fxn address

        mvkl    _TaskSupport_glue, a3
        mvkh    _TaskSupport_glue, a3

        ; the following two values are used by glue
        stw     b8, *STACK--[1]                 ; push argv[1]
        stw     a8, *STACK--[1]                 ; push argv[0]

        zero    a1
        mvk     SWREGS - 1, a2                  ; loop executes (a2 + 1) iters

        ; this value represents B3 in the swap fxn
        stw     a3, *STACK--[1]                 ; glue address

        ; these 10 values (0) represent A10-A15/B10-B13 in the swap fxn
loop:
  [ a2] b       loop
        stw     a1, *STACK--[1]                 ; push 0
        sub     a2, 1, a2                       ; decr. loop counter
        nop     3

        ; this value represents RET in the swap fxn
        stw     ENTRY, *STACK--[1]              ; push entry addr (Task_exit)

        b       b3
        mv      STACK, RETVAL                   ; return stack pointer
        nop     4
        .endasmfunc


; word offsets
Task_SWITCHFRAMEWSIZE   .set    12
Task_SWITCHB3           .set    12      ; not used, same as SWITCHFRAMEWSIZE
Task_SWITCHRET          .set    1
Task_SWITCHA10          .set    2
Task_SWITCHA11          .set    3
Task_SWITCHB10          .set    4
Task_SWITCHB11          .set    5
Task_SWITCHA12          .set    6
Task_SWITCHA13          .set    7
Task_SWITCHB12          .set    8
Task_SWITCHB13          .set    9
Task_SWITCHA14          .set    10
Task_SWITCHA15          .set    11


;
;  ======== TaskSupport_swap ========
;
;  TaskSupport_swap(oldSP, newSP)
;
    .sect ".text:_TaskSupport_swap__E"
    .clink
_TaskSupport_swap__E:
        .asmfunc
        stw     b3, *b15--[Task_SWITCHFRAMEWSIZE]
        ; setup new B-side SP so we can load the new SP early
        mv      b15, b0
||      mv      b15, a0
||      stw     b15, *a4
        ; must load new SP after storing old SP, to support a "self-yield"
        ldw     *b4, b15
||      stw    a11, *a0[Task_SWITCHA11]
        stw    a10, *a0[Task_SWITCHA10]
||      stw    b10, *b0[Task_SWITCHB10]
        stw    a12, *a0[Task_SWITCHA12]
||      stw    b12, *b0[Task_SWITCHB12]
        stw    a13, *a0[Task_SWITCHA13]
||      stw    b13, *b0[Task_SWITCHB13]
        stw    a15, *a0[Task_SWITCHA15]
||      stw    b11, *b0[Task_SWITCHB11]
        stw    a14, *a0[Task_SWITCHA14]
||      stw     b3, *b0[Task_SWITCHRET]

;
;   Pop return address and return.
;   For the first task switch, we will return to Task_enter.
;   The first thing Task_enter will do is to save the return address (b3) on the
;   stack. We want Task_enter to return to glue. This is accomplished by using
;   b4 for this return, and loading the address of glue into b3.
;

        ;
        ; b15 is now the new SP
        ;
        ldw     *b15[Task_SWITCHRET], b4       ; 1st switch, b4 = Task_enter
||      mv      b15, a0
        ldw    *a0[Task_SWITCHA10], a10
||      ldw    *b15[Task_SWITCHB10], b10
        ldw    *a0[Task_SWITCHA11], a11
||      ldw    *b15[Task_SWITCHB11], b11
        ldw    *a0[Task_SWITCHA12], a12
||      ldw    *b15[Task_SWITCHB12], b12
        ldw    *a0[Task_SWITCHA13], a13
||      ldw    *b15[Task_SWITCHB13], b13
        b       b4
||      ldw    *b15[Task_SWITCHA14], a14
        ldw    *a0[Task_SWITCHA15], a15
||      ldw     *++b15[Task_SWITCHFRAMEWSIZE], b3       ; load b3 with glue
        nop     4

        .endasmfunc


;
;  ======== TaskSupport_glue ========
;
;  This glue function is executed immediately before the executing "fxn"
;  passed to Task_start.  This function loads the parameters to "fxn" into
;  registers - necessary for the C62 register passing model.
;
    .sect ".text:_TaskSupport_glue"
    .clink
_TaskSupport_glue:
        .asmfunc

; We must maintain 8-byte stack alignment, so do a one-time even-word
; adjustment after loading registers from the stack frame.
        ldw     *+SP[3], b2             ; start address of new task
        ldw     *+SP[1], a4             ; arg 0
        ldw     *+SP[2], b4             ; arg 1
        ldw     *++SP[4], b3            ; start address of TSK_exit

;   Pop return address and return to newly created task.
;   The first thing the new task will do is to save the return address (b3) on
;   the stack. We want the task to return to TSK_exit. This is accomplished by
;   using b2 for this return, and loading the address of TSK_exit into b3.

        nop
        b       b2                      ; return to new task
        nop     5

        .endasmfunc


