/******************************************************************************
;  @file  csl_arm_r5_pmu.asm
;
;  @brief
;   Implementation file for the ARM R5 PMU module.
;
;   Contains the different control command and status query functions definitions
;
;   \par
;   ============================================================================
;   Copyright (C) 2023 Texas Instruments Incorporated
;
;   Redistribution and use in source and binary forms, with or without
;   modification, are permitted provided that the following conditions
;   are met:
;
;     Redistributions of source code must retain the above copyright
;     notice, this list of conditions and the following disclaimer.
;
;     Redistributions in binary form must reproduce the above copyright
;     notice, this list of conditions and the following disclaimer in the
;     documentation and/or other materials provided with the
;     distribution.
;
;     Neither the name of Texas Instruments Incorporated nor the names of
;     its contributors may be used to endorse or promote products derived
;     from this software without specific prior written permission.
;
;   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
;   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
;   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
;   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
;   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
;   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
;   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;  LOSS OF USE,
;   DATA, OR PROFITS;  OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
;   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
;   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
;   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
;******************************************************************************/
    .text

    .set     CSL_ARM_R5_PMU_CYCLE_COUNTER_NUM, 0x1f

#if defined(__GNUC__) && !defined(__clang__)
    .thumb
    .syntax unified
#endif

/*
;==============================================================================
;   void CSL_armR5PmuSelectCntr( uint32_t cntrNum )
;==============================================================================
*/

CSL_armR5PmuSelectCntr:
    AND     r0, r0, #0x1F
    MCR     p15, #0, r0, c9, c12, #5        /* Write PMSELR Register */
    BX      lr

/*
;==============================================================================
;   void CSL_armR5PmuCfg( uint32_t cycleCntDiv, uint32_t exportEvents, uint32_t userEnable )
;
;==============================================================================
*/
    .global CSL_armR5PmuCfg
CSL_armR5PmuCfg:
    MRC     p15, #0, r3, c9, c12, #0        /* Read PMCR Register */
    BIC     r3, r3, #((1<<3) | (1<<4))      /* Clear D and X bits */
    CMP     r0, #0
    BEQ     armR5PmuCfg_00
    ORR     r3, r3, #(1<<3)                 /* Set D bit */
armR5PmuCfg_00:
    CMP     r1, #0
    BEQ     armR5PmuCfg_01
    ORR     r3, r3, #(1<<4)                 /* Set X bit */
armR5PmuCfg_01:
    MCR     p15, #0, r3, c9, c12, #0        /* Write PMCR Register */
    EOR     r3, r3, r3                      /* Clear r3 */
    CMP     r2, #0
    BEQ     armR5PmuCfg_02
    ORR     r3, r3, #1
armR5PmuCfg_02:
    MCR     p15, #0, r3, c9, c14, #0        /* Write PMUSERENR Register */
    BX      lr

/*
;==============================================================================
;   void CSL_armR5PmuEnableAllCntrs( uint32_t enable )
;
;==============================================================================
*/

    .global CSL_armR5PmuEnableAllCntrs
CSL_armR5PmuEnableAllCntrs:
    MRC     p15, #0, r1, c9, c12, #0        /* Read PMCR */
    CMP     r0, #0
    BEQ     armR5PmuEnableAllCntrs_disable
    ORR     r1, r1, #0x1                    /* Set E bit to enable all counters */
    B       armR5PmuEnableAllCntrs_00
armR5PmuEnableAllCntrs_disable:
    BIC     r1, r1, #0x1                    /* Clr E bit to disable all counters */
armR5PmuEnableAllCntrs_00:
    MCR     p15, #0, r1, c9, c12, #0        /* Write modified PMCR */
    BX      lr

/*
;==============================================================================
;   uint32_t CSL_armR5PmuGetNumCntrs( void )
;
;==============================================================================
*/
    .global CSL_armR5PmuGetNumCntrs
CSL_armR5PmuGetNumCntrs:
    MRC     p15, #0, r0, c9, c12, #0        /* Read PMCR (Performance Monitor Control Register) */
    LSR     r0, r0, #11                     /* Shift and */
    AND     r0, r0, #0x1F                   /*   mask to get N */
    BX      lr

/*
;==============================================================================
;   void CSL_armR5PmuCfgCntr( uint32_t cntrNum, uint8_t eventType )
;
;==============================================================================
*/
    .global CSL_armR5PmuCfgCntr
CSL_armR5PmuCfgCntr:
    PUSH    {lr}
    BL      CSL_armR5PmuSelectCntr          /* Select register */
    AND     r1, r1, #0xFF
    MCR     p15, #0, r1, c9, c13, #1        /* Write PMXEVTYPERx Register */
    POP     {lr}
    BX      lr

/*
;==============================================================================
;   void CSL_armR5PmuEnableCntrOverflowIntr( uint32_t cntrNum, uint32_t enable )
;
;==============================================================================
*/
    .global CSL_armR5PmuEnableCntrOverflowIntr
CSL_armR5PmuEnableCntrOverflowIntr:
    AND     r0, r0, #0x1F                   /* cntrNum must be <= 31 */
    MOV     r2, #1
    LSL     r0, r2, r0                      /* r0 is bit-mask corresponding to cntrNum */
    CMP     r1, #0
    BEQ     armR5PmuEnableCntrOverflowIntr_clear
    MCR     p15, #0, r0, c9, c14, #1        /* Write PMINTENSET Register (writes of 0 have no effect) */
    B       armR5PmuEnableCntrOverflowIntr_00
armR5PmuEnableCntrOverflowIntr_clear:
    MCR     p15, #0, r0, c9, c14, #2        /* Write PMINTENCLR Register (writes of 0 have no effect) */
armR5PmuEnableCntrOverflowIntr_00:
    BX      lr

/*
;==============================================================================
;   void CSL_armR5PmuEnableCntr( uint32_t cntrNum, uint32_t enable )
;
;==============================================================================
*/
    .global CSL_armR5PmuEnableCntr
CSL_armR5PmuEnableCntr:
    AND     r0, r0, #0x1F                   /* cntrNum must be <= 31 */
    MOV     r2, #1
    LSL     r0, r2, r0                      /* r0 is bit-mask corresponding to cntrNum */
    CMP     r1, #0
    BEQ     armR5PmuEnableCntrs_clear
    MCR     p15, #0, r0, c9, c12, #1        /* Write PMCNTENSET Register (writes of 0 have no effect) */
    B       armR5PmuEnableCntrs_00
armR5PmuEnableCntrs_clear:
    MCR     p15, #0, r0, c9, c12, #2        /* Write PMCNTENCLR Register (writes of 0 have no effect) */
armR5PmuEnableCntrs_00:
    BX      lr

/*
;==============================================================================
;   uint32_t CSL_armR5PmuReadCntr( uint32_t cntrNum );
;
;==============================================================================
*/
    .global CSL_armR5PmuReadCntr
CSL_armR5PmuReadCntr:
    PUSH    {lr}
    CMP     r0, #CSL_ARM_R5_PMU_CYCLE_COUNTER_NUM
    BEQ     armR5PmuReadCntr_cycles
    BL      CSL_armR5PmuSelectCntr          /* Select register */
    MRC     p15, #0, r0, c9, c13, #2        /* Read current PMNx Register */
    B       armR5PmuReadCntr_00
armR5PmuReadCntr_cycles:
    MRC     p15, #0, r0, c9, c13, #0        /* Read PMCCNTR Register */
armR5PmuReadCntr_00:
    POP     {lr}
    BX      lr

/*
;==============================================================================
;   void  CSL_armR5PmuSetCntr( uint32_t cntrNum, uint32_t cntrVal );
;
;==============================================================================
*/
    .global CSL_armR5PmuSetCntr
CSL_armR5PmuSetCntr:
    PUSH    {lr}
    CMP     r0, #CSL_ARM_R5_PMU_CYCLE_COUNTER_NUM
    BEQ     armR5PmuSetCntr_cycles
    BL      CSL_armR5PmuSelectCntr          /* Select register */
    MCR     p15, #0, r1, c9, c13, #2        /* Write current PMNx Register */
    B       armR5PmuSetCntr_00
armR5PmuSetCntr_cycles:
    MCR     p15, #0, r1, c9, c13, #0        /* Write PMCCNTR Register */
armR5PmuSetCntr_00:
    POP     {lr}
    BX      lr

/*
;==============================================================================
;   uint32_t CSL_armR5PmuReadCntrOverflowStatus( void )
;
;==============================================================================
*/
    .global CSL_armR5PmuReadCntrOverflowStatus
CSL_armR5PmuReadCntrOverflowStatus:
    MRC     p15, #0, r0, c9, c12, #3        /* Read PMOVSR Register */
    BX      lr

/*
;==============================================================================
;   void CSL_armR5PmuClearCntrOverflowStatus( uint32_t cntrMask )
;
;==============================================================================
*/
    .global CSL_armR5PmuClearCntrOverflowStatus
CSL_armR5PmuClearCntrOverflowStatus:
    MCR     p15, #0, r0, c9, c12, #3        /* Write PMOVSR Register */
    BX      lr

/*
;==============================================================================
;   void CSL_armR5PmuResetCycleCnt( void )
;
;==============================================================================
*/
    .global CSL_armR5PmuResetCycleCnt
CSL_armR5PmuResetCycleCnt:
    MRC     p15, #0, r0, c9, c12, #0        /* Read PMCR */
    ORR     r0, r0, #(1<<2)                 /* Set C bit to reset the cycle counter, PMCCNTR, to zero */
    MCR     p15, #0, r0, c9, c12, #0        /* Write modified PMCR*/
    BX      lr

/*
;==============================================================================
;   void CSL_armR5PmuResetCntrs( void )
;
;==============================================================================
*/
    .global CSL_armR5PmuResetCntrs
CSL_armR5PmuResetCntrs:
    MRC     p15, #0, r0, c9, c12, #0        /* Read PMCR */
    ORR     r0, r0, #(1<<1)                 /* Set P bit to reset all event counters to zero */
    MCR     p15, #0, r0, c9, c12, #0        /* Write modified PMCR */
    BX      lr

    .end
