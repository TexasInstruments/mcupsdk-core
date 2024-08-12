/******************************************************************************
* SDL UTILS
*
* Software Diagnostics Library Utils module
*
*  (C) Copyright 2022-2024 Texas Instruments, Inc.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   Redistributions of source code must retain the above copyright
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
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES LOSS OF USE,
*  DATA, OR PROFITS OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************
*  @file sdl_r5_utils.S
*
*  @brief
*   Implementation file for the ARM R5 utils
*
*   Contains the different control command and status query functions definitions
*
*   \par
******************************************************************************/
    .text
/*==============================================================================
*   The SCTLR Provides control and configuration information for-
*   memory alignment, endianness, protection, and fault behavior.
*   MPU and cache enables and cache replacement strategy.
*   interrupts and the behavior of interrupt latency.
*   the location for exception vectors.
*   program flow prediction.
==============================================================================*/
    .global SDL_UTILS_getSCTLR
SDL_UTILS_getSCTLR:
    MRC p15, #0, r0, c1, c0, #0
    BX      lr

/*==============================================================================
*   The ACTLR Provides control for-
*   branch prediction.
*   performance features.
*   error and parity logic.
==============================================================================*/
    .global SDL_UTILS_getACTLR
SDL_UTILS_getACTLR:
    MRC p15, #0, r0, c1, c0, #1
    BX      lr

/*==============================================================================
*   The Secondary ACTLR Provides control for-
*   branch prediction.
*   performance features.
*   error and parity logic.
==============================================================================*/
    .global SDL_UTILS_getSecondaryACTLR
SDL_UTILS_getSecondaryACTLR:
    MRC p15, #0, r0, c15, c0, #0
    BX      lr

/*==============================================================================
*  The CPACR characteristics are-
*  Sets access rights for coprocessors.
==============================================================================*/
    .global SDL_UTILS_getCPACR
SDL_UTILS_getCPACR:
    MRC p15, #0, r0, c1, c0, #2
    BX      lr

/*==============================================================================
*  The BTCM Region Register characteristics are:
*  Holds the base address and size of the BTCM.
*  Determines if the BTCM is enabled.
==============================================================================*/
    .global SDL_UTILS_getBTCMRegionR
SDL_UTILS_getBTCMRegionR:
    MRC p15, #0, r0, c9, c1, #0
    BX      lr

/*==============================================================================
*  The ATCM Region Register characteristics are:
*  Holds the base address and size of the ATCM.
*  Determines if the ATCM is enabled.
==============================================================================*/
    .global SDL_UTILS_getATCMRegionR
SDL_UTILS_getATCMRegionR:
    MRC p15, #0, r0, c9, c1, #1
    BX      lr

/*==============================================================================
*  The Slave Port Control Register characteristics are:
*  Enables or disables TCM access to the AXI slave port in Privileged or User mode.
*  Enables access to the cache RAMs through the AXI slave port.
    See c1, Auxiliary Control Register.
==============================================================================*/
    .global SDL_UTILS_getSlavePortControlR
SDL_UTILS_getSlavePortControlR:
    MRC p15, #0, r0, c11, c0, #0
    BX      lr

/*==============================================================================
*  The CONTEXTIDR characteristics are:
*  Holds a process IDentification (ID) value for the running process.
*  The Embedded Trace Macrocell (ETM) and the debug logic use this register.
   The ETM can broadcast its value to indicate the process that is running.
   You must program each process with a unique number.
*   Enables process dependent breakpoints and instructions.
==============================================================================*/
    .global SDL_UTILS_getCONTEXTIDR
SDL_UTILS_getCONTEXTIDR:
    MRC p15, #0, r0, c13, c0, #1
    BX      lr
/*==============================================================================
*  The nVAL IRQ Enable Set Register characteristics are:
*  Enables any of the PMXEVCNTR Registers, PMXEVCNTR0-PMXEVCNTR2, and CCNT,
  to generate an interrupt request on overflow. If enabled, the interrupt request
  is signaled by nVALIRQm being asserted LOW.
==============================================================================*/
    .global SDL_UTILS_getnVALIRQSET
SDL_UTILS_getnVALIRQSET:
    MRC p15, #0, r0, c15, c1, #0
    BX      lr

/*==============================================================================
*  The nVAL FIQ Enable Set Register are:
* Enables any of the PMXEVCNTR Registers, PMXEVCNTR0-PMXEVCNTR2, and CCNT,
  to generate an fast interrupt request on overflow. If enabled, the interrupt
  request is signaled by nVALFIQm being asserted LOW..
==============================================================================*/
    .global SDL_UTILS_getnVALFIQSET
SDL_UTILS_getnVALFIQSET:
    MRC p15, #0, r0, c15, c1, #1
    BX      lr

/*==============================================================================
*  The nVAL RESET Enable Set Register are:
* Enables any of the PMXEVCNTR Registers, PMXEVCNTR0-PMXEVCNTR2, and CCNT,
  to generate a reset request on overflow. If enabled, the reset
  request is signaled by nVALRESETm being asserted LOW..
==============================================================================*/
    .global SDL_UTILS_getnVALRESETSET
SDL_UTILS_getnVALRESETSET:
    MRC p15, #0, r0, c15, c1, #2
    BX      lr

/*==============================================================================
*  The VAL Debug Request Enable Set Register characteristics are:
* Enables any of the PMXEVCNTR Registers, PMXEVCNTR0-PMXEVCNTR2, and CCNT,
  to generate a debug request on overflow. If enabled, the debug
  request is signaled by VALEDBGRQm being asserted HIGH.
==============================================================================*/
    .global SDL_UTILS_getnVALDEBUGSET
SDL_UTILS_getnVALDEBUGSET:
    MRC p15, #0, r0, c15, c1, #3
    BX      lr

/*==============================================================================
*  The VAL IRQ Enable Clear Register characteristics are:
* Disables overflow IRQ requests from any of the PMXEVCNTR Registers,
 PMXEVCNTR0-PMXEVCNTR2, and CCNT, for which they have been enabled.
==============================================================================*/
    .global SDL_UTILS_getnVALIRQCLEAR
SDL_UTILS_getnVALIRQCLEAR:
    MRC p15, #0, r0, c15, c1, #4
    BX      lr

/*==============================================================================
*  The nVAL FIQ Enable Clear Register characteristics are:
* Disables overflow FIQ requests from any of the PMXEVCNTR Registers,
 PMXEVCNTR0-PMXEVCNTR2, and CCNT, for which they have been enabled.
==============================================================================*/
    .global SDL_UTILS_getnVALFIQCLEAR
SDL_UTILS_getnVALFIQCLEAR:
    MRC p15, #0, r0, c15, c1, #5
    BX      lr

/*==============================================================================
* The nVAL Reset Enable Clear Register characteristics are:
* Disables overflow reset requests from any of the PMXEVCNTR Registers,
  PMXEVCNTR0-PMXEVCNTR2, and CCNT, that are enabled..
==============================================================================*/
    .global SDL_UTILS_getnVALRESETCLEAR
SDL_UTILS_getnVALRESETCLEAR:
    MRC p15, #0, r0, c15, c1, #6
    BX      lr

/*==============================================================================
*  The VAL Debug Request Enable Clear Register characteristics are:
*  Disables overflow debug requests from any of the PMXEVCNTR Registers,
   PMXEVCNTR0-PMXEVCNTR2, and CCNT, that are enabled.
==============================================================================*/
    .global SDL_UTILS_getnVALDEBUGCLEAR
SDL_UTILS_getnVALDEBUGCLEAR:
    MRC p15, #0, r0, c15, c1, #3
    BX      lr


/*==============================================================================
*  The Build Options 1 Register characteristics are:
*  Reflects the build configuration options used to build the processor.
==============================================================================*/
    .global SDL_UTILS_getBuildOption1R
SDL_UTILS_getBuildOption1R:
    MRC p15, #0, r0, c15, c2, #0
    BX      lr

/*==============================================================================
*  The Build Options 2 Register characteristics are:
*  Reflects the build configuration options used to build the processor.
==============================================================================*/
    .global SDL_UTILS_getBuildOption2R
SDL_UTILS_getBuildOption2R:
    MRC p15, #0, r0, c15, c2, #1
    BX      lr

/*==============================================================================
*  The Pin Options Register characteristics are:
*  Describes the value of any pins that control processor options,
   that are not visible because they:
*  are exposed in registers.
*  control the initial value of control registers, and are visible in that way.
==============================================================================*/
    .global SDL_UTILS_getPinOptionR
SDL_UTILS_getPinOptionR:
    MRC p15, #0, r0, c15, c2, #7
    BX      lr

/*==============================================================================
*  The Peripheral Interface Region Register characteristics are:
*  Describe the size and base of the interface, and contain an enable bit for
   the interface
   Read LLPP Normal AXI region register
==============================================================================*/
    .global SDL_UTILS_getLLPPnormalAXIRR
SDL_UTILS_getLLPPnormalAXIRR:
    MRC p15, #0, r0, c15, c0, #1
    BX      lr

/*==============================================================================
*  The Peripheral Interface Region Register characteristics are:
*  Describe the size and base of the interface, and contain an enable bit for
   the interface
    Read LLPP Virtual AXI region register
==============================================================================*/
    .global SDL_UTILS_getLLPPvirtualAXIRR
SDL_UTILS_getLLPPvirtualAXIRR:
    MRC p15, #0, r0, c15, c0, #2
    BX      lr

/*==============================================================================
*  The Peripheral Interface Region Register characteristics are:
*  Describe the size and base of the interface, and contain an enable bit for
   the interface
   Read AHB peripheral interface region register
==============================================================================*/
    .global SDL_UTILS_getAHBRR
SDL_UTILS_getAHBRR:
    MRC p15, #0, r0, c15, c0, #3
    BX      lr


/*==============================================================================
*   Get Correctable Fault Location Register (CFLR) value
==============================================================================*/
    .global SDL_UTILS_getCFLR
SDL_UTILS_getCFLR:
    MRC p15, #0, r0, c15, c3, #0
    BX      lr

/*=============================================================================
*   Get Performance Monitors Overflow Flag Status Register (PMOVSR) value
==============================================================================*/
    .global SDL_UTILS_getPMOVSR
SDL_UTILS_getPMOVSR:
    MRC p15, #0, r0, c9, c12, #3
    BX      lr

/*==============================================================================
*   Get Data Fault Status Register (DFSR) value
==============================================================================*/
    .global SDL_UTILS_getDFSR
SDL_UTILS_getDFSR:
    MRC p15, #0, r0, c5, c0, #0
    BX      lr

/*==============================================================================
*   Get Data Fault Status Register (ADFSR) value
==============================================================================*/
    .global SDL_UTILS_getADFSR
SDL_UTILS_getADFSR:
    MRC p15, #0, r0, c5, c1, #0
    BX      lr

/*==============================================================================
*   Get Data Fault Address Register (DFAR) value
==============================================================================*/
    .global SDL_UTILS_getDFAR
SDL_UTILS_getDFAR:
    MRC p15, #0, r0, c6, c0, #0
    BX      lr

/*==============================================================================
*   Get Instruction Fault Status Register Register (IFSR) value
==============================================================================*/
    .global SDL_UTILS_getIFSR
SDL_UTILS_getIFSR:
    MRC p15, #0, r0, c5, c0, #1
    BX      lr

/*==============================================================================
*   Get Instruction Fault Status Register Register (AIFSR) value
==============================================================================*/
    .global SDL_UTILS_getAIFSR
SDL_UTILS_getAIFSR:
    MRC p15, #0, r0, c5, c1, #1
    BX      lr

/*==============================================================================
*   Get Instruction Fault Address Register Register (IFAR) value
==============================================================================*/
    .global SDL_UTILS_getIFAR
SDL_UTILS_getIFAR:
    MRC p15, #0, r0, c6, c0, #2
    BX      lr

/*==============================================================================
*   Get Performance Monitor Control Register (PMCR) value
==============================================================================*/
    .global SDL_UTILS_getPMCR
SDL_UTILS_getPMCR:
    MRC p15, #0, r0, c9, c12, #0
    BX      lr

/*==============================================================================
*   Get Performance Monitor Count Enable Set Register (PMCNTENSET) value
==============================================================================*/
    .global SDL_UTILS_getPMCNTENSET
SDL_UTILS_getPMCNTENSET:
    MRC p15, #0, r0, c9, c12, #1
    BX      lr

/*==============================================================================
*   Get Performance Monitor User Enable Register (PMUSERENR) value
==============================================================================*/
    .global SDL_UTILS_getPMUSERENR
SDL_UTILS_getPMUSERENR:
    MRC p15, #0, r0, c9, c14, #0
    BX      lr

/*==============================================================================
*   Get Performance Monitor Interrupt Enable Set Register (PMINTENSET) value
==============================================================================*/
    .global SDL_UTILS_getPMINTENSET
SDL_UTILS_getPMINTENSET:
    MRC p15, #0, r0, c9, c14, #1
    BX      lr

/*==============================================================================
*   Get Performance Monitor Interrupt Enable Clear Register (PMINTENCLR) value
==============================================================================*/
    .global SDL_UTILS_getPMINTENCLR
SDL_UTILS_getPMINTENCLR:
    MRC p15, #0, r0, c9, c14, #2
    BX      lr

/*****************************************************************************
* Enable Event Bus
******************************************************************************/
	.global SDL_UTILS_enable_event_bus
SDL_UTILS_enable_event_bus:
	ORR r1,r1,#0x7 << 25           	/* Don't need to force dpu_tcm_check_parity_en_o */
	MCR p15, #0, r1, c1, c0, #1   	/* ACTLR enable ECC for cache */
	MRC p15, #0, r0, c1, c0, #0   	/* Read System Control Register */
	NOP
	MRC P15,#0, R5, C9, C12,#0 		/* enable export of events to Eventbus for monitoring */
	ORR R5, R5, #0x11
	MCR P15,#0, R5, C9, C12,#0
	BX      lr

/*****************************************************************************
* Enable Cache Event Bus
******************************************************************************/
	.global SDL_UTILS_enable_cache_event_bus
SDL_UTILS_enable_cache_event_bus:
    MRC P15,#0, R5, C9, C12,#0      /* enable export of events to Eventbus for monitoring */
    ORR R5, R5, #0x11
    MCR P15,#0, R5, C9, C12,#0
    MRC p15, #0, r1, c1, c0, #1     /* read ACTLR */
    ORR r1,r1,#0x7 << 3 			/* do not generate ecc aborts for cache, enable hardware recovery */
    BIC r1,r1,#0x1 << 4
    MCR p15, #0, r1, c1, c0, #1 	/* ACTLR enable ECC for cache */
    MRC p15, #0, r0, c1, c0, #0     /* Read System Control Register */
    NOP
	BX      lr

/*****************************************************************************
*   void SDL_R5MPU_readStaticRegisters(SDL_R5MPU_staticRegs *pMPUStaticRegs,uint32_t regionNum)
* 	Design: PROC_SDL-2496
******************************************************************************/
     .global SDL_R5MPU_readStaticRegisters
SDL_R5MPU_readStaticRegisters:
    PUSH    {r4}
    AND     r1, r1, #0xF
    MCR     p15, #0, r1, c6, c2, #0         /* Write RGNR (MPU Region Number Register) */
	EOR     r1,r1,r1   
    MRC     p15, #0, r1, c1, c0, #0         /* Read SCTLR (System Control Register) */
	STR     r1, [r0]
	ADD     r0, r0, #0x4
    MRC     p15, #0, r2, c0, c0, #4         /* Read MPUIR (MPU Type Register) */
	STR     r2, [r0]
	ADD     r0, r0, #0x4
    MRC     p15, #0, r4, c6, c2, #0         /* Read RGNR (MPU Region Number Register) */
    AND     r4, r4, #0xF    
    STR 	r4, [r0]
	ADD     r0, r0, #0x4					
    MRC     p15, #0, r4, c6, c1, #0         /* Read MPU Region Base Address Register */
    BIC     r4, r4, #0x1F                   /* Base address must be 16-bit aligned  */
    STR 	r4, [r0]
	ADD     r0, r0, #0x4
    MRC     p15, #0, r4, c6, c1, #2         /* Read Data MPU Region Size and Enable Register */
    STR 	r4, [r0]
	ADD     r0, r0, #0x4
    MRC     p15, #0, r4, c6, c1, #4         /* Read MPU Region Access Control Register */
    STR 	r4, [r0]
    POP     {r4}
    BX      lr
