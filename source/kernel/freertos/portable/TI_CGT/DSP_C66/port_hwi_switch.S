;
; ======== Hwi_asm.s62 ========
;
;

        .cdecls C,NOLIST,"port_hwi.h"

;
; This logic is needed to allow this code to work with 6.1.x codegen
; which didn't define the .TI_C6X_TESLA symbol.  We need to work
; with both old and new codegen, so we define our own symbol.
;
    .if ($isdefed(".TI_C6X_TESLA"))
        .if (.TI_C6X_TESLA)
_TESLA_         .set 1
        .endif
    .endif

    .if $isdefed("__TI_ELFABI__")
    .if __TI_ELFABI__
        .asg Hwi_switchAndRunFunc, _Hwi_switchAndRunFunc
        .asg Hwi_switchAndDispatch__I, _Hwi_switchAndDispatch__I
        .asg Hwi_Module__state__V, _Hwi_Module__state__V
        .asg Hwi_dispatchCore__I, _Hwi_dispatchCore__I
    .endif
    .endif

        .global _Hwi_switchAndRunFunc
        .global _Hwi_switchAndDispatch__I
    .global _Hwi_dispatchCore__I

_Hwi_Module__state__V .tag Hwi_Module_State__

        .text

SP      .set    b15

;
;  ======== Hwi_switchAndRunFunc ========
;  Hwi_switchAndRunFunc(Void (*func)());
;
;  Switch to ISR stack, run func() and then switch back to Task
;  stack.
;
;
    .sect ".text:_Hwi_switchAndRunFunc"
        .clink
_Hwi_switchAndRunFunc:
        .asmfunc
        stw     a10, *SP--[2]               ; save a10 so it can be used below
||      mvkl    _Hwi_Module__state__V, a0
        stw     b3, *+SP[1]                 ; save b3 (return address) on stack
||      mvkh    _Hwi_Module__state__V, a0

        ; get current taskSP value
        ldw     *a0(Hwi_Module_State__.taskSP), b0
        ldw     *a0(Hwi_Module_State__.isrStack), b1
        b       a4                          ; call func()
        mvkl    funcReturn, b3
        mvkh    funcReturn, b3
        ; save old SP
  [!b0] stw     SP, *a0(Hwi_Module_State__.taskSP)
  [!b0] mv      b1, SP                      ; return SP on isrStack
        mv      b0, a10                     ; save old old task SP to a10
funcReturn:
        mv      a10, b0                     ; restore old task SP to b0
        mvkl    _Hwi_Module__state__V.taskSP, a0
        mvkh    _Hwi_Module__state__V.taskSP, a0
  [!b0] ldw     *a0, SP                     ; switch to saved Task SP
  [!b0] stw     b0, *a0                     ; store old task SP
        nop     3                           ; wait for load of SP to finish
        ldw     *+SP[1], b3                 ; restore b3 from stack
        ldw     *++SP[2], a10               ; restore a10 from stack
        nop     3                           ; wait for load of b3 to finish
        bnop    b3, 5                       ; function return
        .endasmfunc

;
;  ======== Hwi_switchAndDispatch ========
;
        .sect ".text:_Hwi_switchAndDispatch__I"
        .clink
_Hwi_switchAndDispatch__I
        .asmfunc
        mvkl    _Hwi_Module__state__V, a0
        mvkh    _Hwi_Module__state__V, a0
        ldw     *a0(Hwi_Module_State__.taskSP), b0   ; get current taskSP value
        ldw     *a0(Hwi_Module_State__.isrStack), b1
        mvkl    _Hwi_dispatchCore__I, a1
        mvkh    _Hwi_dispatchCore__I, a1
        b       a1
  [!b0] stw     SP, *a0(Hwi_Module_State__.taskSP)   ; save old SP
  [!b0] mv      b1, SP                  ; set SP to isrStack
        stw     b10, *SP--[2]           ; save b10 on the stack
||      mv  b3, b10                     ; move b3 into b10
        mvkl    dispatch_return, b3     ; update b3 to dispatch_return
        stw b0, *+SP[1]                 ; save old taskSP on stack
||      mvkh    dispatch_return, b3

dispatch_return:
        ldw *+SP[1], b0     ; restore old taskSP to b0
        mvkl    _Hwi_Module__state__V, a0
        mvkh    _Hwi_Module__state__V, a0
        mvk 0, a1           ; move 0 to a1
        mv  b10, b3         ; restore b3 from b10
  [!b0] ldw     *a0(Hwi_Module_State__.taskSP), b1   ; get current taskSP value
        b       b3
        ldw *++SP[2], b10   ; restore b10 from the stack
        nop     2
  [!b0] mv  b1, SP          ; set SP = taskSP
  [!b0] stw a1, *a0(Hwi_Module_State__.taskSP) ; taskSP = 0

        .endasmfunc
