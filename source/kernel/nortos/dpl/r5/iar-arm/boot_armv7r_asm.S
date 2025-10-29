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

//****************************************************************************
//* BOOT.S
//*
//* THIS IS THE INITAL BOOT ROUTINE FOR TMS470 C++ PROGRAMS.
//* IT MUST BE LINKED AND LOADED WITH ALL C++ PROGRAMS.
//*
//* THIS MODULE PERFORMS THE FOLLOWING ACTIONS:
//*   1) ALLOCATES THE STACK AND INITIALIZES THE STACK POINTER
//*   2) CALLS AUTO-INITIALIZATION ROUTINE
//*   3) CALLS THE FUNCTION MAIN TO START THE C++ PROGRAM
//*   4) CALLS THE STANDARD EXIT ROUTINE
//*
//* THIS MODULE DEFINES THE FOLLOWING GLOBAL SYMBOLS:
//*   1) __stack     STACK MEMORY AREA
//*   2) _c_int00    BOOT ROUTINE
//*
//****************************************************************************

        MODULE ?cstartup
//***************************************************************
//* DEFINE THE SYSTEM MODE STACK (DEFAULT SIZE IS 512)
//***************************************************************
        SECTION IRQ_STACK:DATA:NOROOT(3)
        SECTION FIQ_STACK:DATA:NOROOT(3)
        SECTION SVC_STACK:DATA:NOROOT(3)
        SECTION ABORT_STACK:DATA:NOROOT(3)
        SECTION UNDEFINED_STACK:DATA:NOROOT(3)
        SECTION CSTACK:DATA:NOROOT(3)

        PUBLIC __iar_program_start_sbl
        SECTION `.text.boot`:CODE:NOROOT(2)
        ARM
        ALIGNRAM 2

__iar_program_start_sbl:
        // entry point for bootloader, if Core 1, then loop forever, else goto _c_int00
        MRC     p15, #0, r1, c0, c0, #5
        BFC     r1, #8, #24
        CMP     r1, #0
        BNE     _sblLoopForever
        LDR     r1, _c_int00_addr
        BLX     r1
_sblLoopForever:
        WFI
        B       _sblLoopForever

_c_int00_addr:  DCD __iar_program_start

        PUBLIC __iar_program_start
        SECTION `.text.boot`:CODE:NOROOT(2)
        ARM
        ALIGNRAM 2

//***************************************************************
//* FUNCTION DEF: _c_int00
//***************************************************************
__iar_program_start:
?cstartup:
        //*------------------------------------------------------
        //* Disable FIQ and IRQ
        //*------------------------------------------------------
        mrs     r0, cpsr
        orr     r0, r0, #0xC0
        msr     cpsr_cf, r0

        #ifdef __ARM_FP
        //*------------------------------------------------------
        //* SETUP PRIVILEGED AND USER MODE ACCESS TO COPROCESSORS
        //* 10 AND 11, REQUIRED TO ENABLE NEON/VFP
        //* COPROCESSOR ACCESS CONTROL REG
        //* BITS [23:22] - CP11, [21:20] - CP10
        //* SET TO 0b11 TO ENABLE USER AND PRIV MODE ACCESS
        //*------------------------------------------------------
        MRC      p15,#0x0,r0,c1,c0,#2
        MOV      r3,#0xf00000
        ORR      r0,r0,r3
        MCR      p15,#0x0,r0,c1,c0,#2

        //*------------------------------------------------------
        // SET THE EN BIT, FPEXC[30] TO ENABLE NEON AND VFP
        //*------------------------------------------------------
        MOV      r0,#0x40000000
        FMXR     FPEXC,r0
        #endif

        //*------------------------------------------------------
        //* SET TO FIQ MODE
        //*------------------------------------------------------
        MRS     r0, cpsr
        BIC     r0, r0, #0x1F  // CLEAR MODES
        ORR     r0, r0, #0x11  // SET FIQ MODE
        MSR     cpsr_cf, r0

        //*------------------------------------------------------
        //* INITIALIZE THE FIQ MODE STACK
        //*------------------------------------------------------
        LDR     sp, =SFE(FIQ_STACK)     ; End of FIQ_STACK

        //*------------------------------------------------------
        //* SET TO IRQ MODE
        //*------------------------------------------------------
        MRS     r0, cpsr
        BIC     r0, r0, #0x1F  // CLEAR MODES
        ORR     r0, r0, #0x12  // SET IRQ MODE
        MSR     cpsr_cf, r0

        //*------------------------------------------------------
        //* INITIALIZE THE IRQ MODE STACK
        //*------------------------------------------------------
        LDR     sp, =SFE(IRQ_STACK)     ; End of IRQ_STACK

        //*------------------------------------------------------
        //* SET TO SVC MODE
        //*------------------------------------------------------
        MRS     r0, cpsr
        BIC     r0, r0, #0x1F  // CLEAR MODES
        ORR     r0, r0, #0x13  // SET SVC MODE
        MSR     cpsr_cf, r0

        //*------------------------------------------------------
        //* INITIALIZE THE SVC MODE STACK
        //*------------------------------------------------------
        LDR     sp, =SFE(SVC_STACK)     ; End of SVC_STACK

        //*------------------------------------------------------
        //* SET TO ABORT MODE
        //*------------------------------------------------------
        MRS     r0, cpsr
        BIC     r0, r0, #0x1F  // CLEAR MODES
        ORR     r0, r0, #0x17  // SET ABORT MODE
        MSR     cpsr_cf, r0

        //*------------------------------------------------------
        //* INITIALIZE THE ABORT MODE STACK
        //*------------------------------------------------------
        LDR     sp, =SFE(ABORT_STACK)     ; End of ABORT_STACK

        //*------------------------------------------------------
        //* SET TO ABORT MODE
        //*------------------------------------------------------
        MRS     r0, cpsr
        BIC     r0, r0, #0x1F  // CLEAR MODES
        ORR     r0, r0, #0x1B  // SET ABORT MODE
        MSR     cpsr_cf, r0

        //*------------------------------------------------------
        //* INITIALIZE THE UNDEFINED MODE STACK
        //*------------------------------------------------------
        LDR     sp, =SFE(UNDEFINED_STACK)     ; End of UNDEFINED_STACK

        //*------------------------------------------------------
        //* SET TO SYSTEM MODE
        //*------------------------------------------------------
        MRS     r0, cpsr
        BIC     r0, r0, #0x1F  // CLEAR MODES
        ORR     r0, r0, #0x1F  // SET SYSTEM MODE
        MSR     cpsr_cf, r0

        //*------------------------------------------------------
        //* INITIALIZE THE SYSTEM MODE STACK
        //*------------------------------------------------------
        LDR     sp, =SFE(CSTACK)     ; End of CSTACK
        //*------------------------------------------------------
        //* Enable ECC for R5F Cache
        //*------------------------------------------------------
        MRC p15, 0, r0, c1, c0, 0  // Read System Control Register
        BIC r0, r0, #0x1 << 2      // Disable data cache bit
        BIC r0, r0, #0x1 << 12     // Disable instruction cache bit
        DSB
        MCR p15, 0, r0, c1, c0, 0  // Write System Control Register
        ISB                        // Ensures following instructions are not executed from cache
        MRC p15, 0, r1, c1, c0, 1  // Read Auxiliary Control Register
        ORR r1, r1, #(0x1 << 3)    //Enable ECC for Cache
        ORR r1,r1,#(0x1 << 5)
        MCR p15, 0, r1, c1, c0, 1  // Write Auxiliary Control Register

        MCR p15, 0, r0, c15, c5, 0 // Invalidate entire data cache
        MCR p15, 0, r0, c7, c5, 0  // Invalidate entire instruction cache
        MRC p15, 0, r0, c1, c0, 0  // Read System Control Register
        ORR r0, r0, #0x1 << 2      // Enable data cache bit
        ORR r0, r0, #0x1 << 12     // Enable instruction cache bit
        DSB
        MCR p15, 0, r0, c1, c0, 0  // Write System Control Register
        ISB
        //*------------------------------------------------------
        //* Call the __mpu_init hook function.
        //*------------------------------------------------------
        BL      __mpu_init

        //*------------------------------------------------------
        //* Init .bss section to zero, this needs .bss to be
        //* defined as below in the linker command file.
        //*
        //* .bss:    {} palign(8)
        //* RUN_START(__BSS_START)
        //* RUN_END(__BSS_END)
        //*
        //* We do this after __mpu_init, since now cache is
        //* enabled and there memset speed will be optimal
        //*
        //* Perform all the required initializations when
        //* _system_pre_init() returns non-zero:
        //*   - Process BINIT Table
        //*   - Perform C auto initialization
        //*   - Call global constructors
        //*------------------------------------------------------
        FUNCALL __iar_program_start, __iar_data_init3
        BL      __iar_data_init3

//        BL      _system_pre_init
        CMP     R0, #0
        BEQ     bypass_auto_init
        BL      __TI_auto_init
bypass_auto_init:

        //*------------------------------------------------------
        //* CALL APPLICATION
        //*------------------------------------------------------
        BL      main

        //*------------------------------------------------------
        //* IF APPLICATION DIDN'T CALL EXIT, CALL EXIT(1)
        //*------------------------------------------------------
        MOV     R0, #1
        BL      exit

        //*------------------------------------------------------
        //* DONE, LOOP FOREVER
        //*------------------------------------------------------
L1:     B       L1


//***************************************************************
//* CONSTANTS USED BY THIS MODULE
//***************************************************************


//******************************************************
//* UNDEFINED REFERENCES                               *
//******************************************************
        EXTERN _system_pre_init
        EXTERN __TI_auto_init
        EXTERN main
        EXTERN exit
        EXTERN __mpu_init
        EXTERN __bss_init
        EXTERN __iar_data_init3
        EXTERN __vectors


        SECTION .text:CODE:ROOT(2)

        ; Keep vector table even if it's not referenced
        REQUIRE __vectors

        END
