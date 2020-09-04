/******************************************************************************
* SDL ECC UTILS
*
* Software Diagnostics Library module for ECC
*
*  (C) Copyright 2022 Texas Instruments, Inc.
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
