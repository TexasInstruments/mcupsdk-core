/******************************************************************************
* SDL ECC UTILS
*
* Software Diagnostics Library module for ECC
*
*  (C) Copyright 2023 Texas Instruments, Inc.
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
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES LOSS OF USE,
*  DATA, OR PROFITS OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************
*  @file sdl_ecc_utils.asm
*
*  @brief
*   Implementation file for the ARM R5 ecc utils
*
*   Contains the different control command and status query functions definitions
*
*   \par
******************************************************************************/
    .text

SDL_ECCUTILS_ATCM_ENABLE_MASK:     .word        0x2000000
SDL_ECCUTILS_B0TCM_ENABLE_MASK:    .word        0x4000000
SDL_ECCUTILS_B1TCM_ENABLE_MASK:    .word        0x8000000

SDL_ECCUTILS_PMCINTSET_COUNT_ENABLE_VALUE:    .word        0x80000002
/*==============================================================================
*   void _enable_pmu_cntrs( void )
*   Enable PMU counters
==============================================================================*/
    .global _enable_pmu_cntrs
_enable_pmu_cntrs:
    /* Enable all PMU counters*/
    MOV     r5, #0x0
    MRC     p15, #0, r5, c9, c12, #0
    ORR     r5, r5, #0x2	     	    /* Reset event counter */
    MCR     p15, #0, r5, c9, c12, #0
    MRC     p15, #0, r5, c9, c12, #0
    ORR     r5, r5, #0x11
    MCR     p15, #0, r5, c9, c12, #0
    BX      lr


/*==============================================================================
*   void SDL_ECC_UTILS_configSecIntr( uint32_t eventCountInitValue,
*                                    uint32_t eventTypeBitMap,
*                                    uint32_t counterNumber )
==============================================================================*/
    .global SDL_ECC_UTILS_configSecIntr
SDL_ECC_UTILS_configSecIntr:
    PUSH {r4-r6}
    MOV r6, lr
    BL _enable_pmu_cntrs
    LDR r4, SDL_ECCUTILS_PMCINTSET_COUNT_ENABLE_VALUE   /* Enable cycle counter and counter 1 */
    MCR p15, #0, r4, c9, c12, #1         /*Count Enable Set Register, PMCNTENSET enabling counter 1 */
    MCR p15, #0, r4, c9, c14, #1         /* Interrupt Enable Set Register, PMINTENSET */
    MOV r4, r2
    MCR p15, #0, r4, c9, c12, #5         /* Event Counter Selection Register, PMSELR, Event counter 1 is selected */
    MCR p15, #0, r0, c9, c13, #2         /* Event Count Register, PMXEVCNTR, Set inital count value to 0xFFFFFFF0 */
    MOV r4, r1                           /*get event type from parameters */
    MCR p15, #0, r4, c9, c13, #1         /* Event Type Select Register, PMXEVTYPER Count event in secured/non-secured USER/PRIVILEGED, Event number 0x4D */
    MOV lr, r6
    POP {r4-r6}
    BX      lr

/*==============================================================================
*   void _SDL_ECC_UTILS_enableECC(uint32_t bitMask)
==============================================================================*/
    .global _SDL_ECC_UTILS_enableECC
_SDL_ECC_UTILS_enableECC:
    /* Read ACTLR register */
    MRC p15, #0, r1, c1, c0, #1

    /* Set ECC enable for ATCM */
    ORR r1, r1, r0

    /*  Write ACTLR register */
    MCR p15, #0, r1, c1, c0, #1
    BX      lr

/*==============================================================================
*   void SDL_ECC_UTILS_enableECCATCM(void)
============================================================================== */
    .global SDL_ECC_UTILS_enableECCATCM
SDL_ECC_UTILS_enableECCATCM:
    LDR r0, SDL_ECCUTILS_ATCM_ENABLE_MASK
    B _SDL_ECC_UTILS_enableECC

/*==============================================================================
*   void SDL_ECC_UTILS_enableECCB0TCM(void)
============================================================================== */
    .global SDL_ECC_UTILS_enableECCB0TCM
SDL_ECC_UTILS_enableECCB0TCM:
    LDR r0, SDL_ECCUTILS_B0TCM_ENABLE_MASK
    B _SDL_ECC_UTILS_enableECC

/*==============================================================================
*   void SDL_ECC_UTILS_enableECCB1TCM(void)
============================================================================== */
    .global SDL_ECC_UTILS_enableECCB1TCM
SDL_ECC_UTILS_enableECCB1TCM:
    LDR r0, SDL_ECCUTILS_B1TCM_ENABLE_MASK
    B _SDL_ECC_UTILS_enableECC
