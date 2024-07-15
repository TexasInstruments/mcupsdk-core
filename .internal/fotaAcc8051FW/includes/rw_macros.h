
/*
 *  Copyright (C) 2024 Texas Instruments Incorporated
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

void _gptrput (char *gptr, char c)
{
/* This is the new version with pointers up to 16 bits.
   B cannot be trashed */

//    gptr; c; /* hush the compiler */

//    gptr;
    __asm
        
        ;
    ;   depending on the pointer type according to SDCCsymt.h
            ;
        jb      _B_7,putcodeptr$        ; >0x80 code       ; 3
        jnb     _B_6,putxdataptr$       ; <0x40 far        ; 3

        mov     dph,r0 ; save r0 independant of regbank ; 2
        mov     r0,dpl ; use only low order address     ; 2

        jb      _B_5,putpdataptr$       ; >0x60 pdata      ; 3
    ;
    ;   store into near/idata space
    ;
        mov     @r0,a                                   ; 1
 putdataptrrestore$:
        mov     r0,dph ; restore r0                     ; 2
        mov     dph,#0 ; restore dph                    ; 2
        ret                                             ; 1
    ;
    ;   cannot store into code space, lock up
    ;
 putcodeptr$:
        sjmp    .                                       ; 2
    ;
    ;   store into external stack/pdata space
    ;
 putpdataptr$:
        movx    @r0,a                                   ; 1
        sjmp    putdataptrrestore$                         ; 2
    ;
    ;   store into far space
    ;
 putxdataptr$:
        movx    @dptr,a                                 ; 1
        nop
        nop
        nop
        nop
        ret                                             ; 1

                                                        ;26 bytes
    __endasm;
}



void
_gptrget (char *gptr) __naked
{
/* This is the new version with pointers up to 16 bits.
   B cannot be trashed */

    gptr; /* hush the compiler */

    __asm
    ;
    ;   depending on the pointer type acc. to SDCCsymt.h
    ;
        jb      _B_7,getcodeptr$        ; >0x80 code       ; 3
        jnb     _B_6,getxdataptr$       ; <0x40 far        ; 3

        mov     dph,r0 ; save r0 independant of regbank ; 2
        mov     r0,dpl ; use only low order address     ; 2

        jb      _B_5,getpdataptr$       ; >0x60 pdata      ; 3
    ;
    ;   Pointer to data space
    ;
        mov     a,@r0                                   ; 1
 getdataptrrestore$:
        mov     r0,dph ; restore r0                     ; 2
        mov     dph,#0 ; restore dph                    ; 2
        ret                                             ; 1
    ;
    ;   pointer to xternal stack or pdata
    ;
 getpdataptr$:
        movx    a,@r0                                   ; 1
        sjmp    getdataptrrestore$                         ; 2
    ;
    ;   pointer to code area, max 16 bits
    ;
 getcodeptr$:
        clr     a                                       ; 1
        movc    a,@a+dptr                               ; 1
        ret                                             ; 1
    ;
    ;   pointer to xternal data, max 16 bits
    ;
 getxdataptr$:
        movx    a,@dptr                                 ; 1
        nop
        nop
        nop
        nop
        ret                                             ; 1
                                                        ;===
                                                        ;27 bytes
 __endasm;
}

