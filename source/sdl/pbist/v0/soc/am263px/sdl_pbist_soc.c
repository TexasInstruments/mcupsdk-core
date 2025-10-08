/*
 *   Copyright (c) Texas Instruments Incorporated 2022-2024
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
 *
 */

/**
 *  \file     sdl_pbist_soc.c
 *
 *  \brief    SDL implementation file for the SoC-specific pbist implementations.
 */

#include <stdint.h>
#include <sdl/include/sdl_types.h>
#include <sdl/sdl_pbist.h>
#include <sdl/pbist/sdl_pbist_priv.h>
#include <sdl/include/soc_config.h>

/* Properties of PBIST instances in: PBIST1 - TOP PBIST*/
#define SDL_PBIST1_NUM_TEST_VECTORS                                                                (1U)
#define SDL_PBIST1_ALGO_BITMAP_0                                                                   (0x0000000000000080U)
#define SDL_PBIST1_MEM_BITMAP_0                                                                    (0x0000000000004000U)
#define SDL_PBIST0_FAIL_INSERTION_TEST_VECTOR_CA0                                                  (0x00002000U)
#define SDL_PBIST0_FAIL_INSERTION_TEST_VECTOR_CA1                                                  (0x000027FFU)
#define SDL_PBIST0_FAIL_INSERTION_TEST_VECTOR_CA2                                                  (0x000007FFU)
#define SDL_PBIST0_FAIL_INSERTION_TEST_VECTOR_CA3                                                  (0x00000000U)
#define SDL_PBIST0_FAIL_INSERTION_TEST_VECTOR_CL0                                                  (0x0000007FU)
#define SDL_PBIST0_FAIL_INSERTION_TEST_VECTOR_CL1                                                  (0x0000000FU)
#define SDL_PBIST0_FAIL_INSERTION_TEST_VECTOR_CL2                                                  (0x0000000AU)
#define SDL_PBIST0_FAIL_INSERTION_TEST_VECTOR_CL3                                                  (0x000007FFU)
#define SDL_PBIST0_FAIL_INSERTION_TEST_VECTOR_CMS                                                  (0x00000001U)
#define SDL_PBIST0_FAIL_INSERTION_TEST_VECTOR_CSR                                                  (0x00000001U)
#define SDL_PBIST0_FAIL_INSERTION_TEST_VECTOR_I0                                                   (0x00000001U)
#define SDL_PBIST0_FAIL_INSERTION_TEST_VECTOR_I1                                                   (0x00000010U)
#define SDL_PBIST0_FAIL_INSERTION_TEST_VECTOR_I2                                                   (0x0000000AU)
#define SDL_PBIST0_FAIL_INSERTION_TEST_VECTOR_I3                                                   (0x00000000U)
#define SDL_PBIST0_FAIL_INSERTION_TEST_VECTOR_RAMT                                                 (0x4900200CU)


/*
 * Note: the order for the below must match order of SDL_PBIST_inst
 */
static SDL_pbistInstInfo SDL_PBIST_InstInfoArray[1] =
{
    /* MCU R5F Instance */
    {
        .PBISTRegsHiAddress  = 0u,
        .pPBISTRegs          = (SDL_pbistRegs *)SDL_TOP_PBIST_U_BASE,

        #if defined (SUBSYS_R5SS0)
        .numPBISTRuns        = PBIST_MAX_NUM_RUNS,
        .PBISTConfigRun =
        {
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 0u,
                .memoryGroupsBitMap = 0u,  /* Choose R5 STC*/
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 0u,
                .memoryGroupsBitMap = 1u,  /* Choose R51 STC*/
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 1u,
                .memoryGroupsBitMap = 2u,  /* Choose R50 TMU1 */
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 2u,
                .memoryGroupsBitMap = 3u,  /* Choose R50 TMU2 */
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 3u,
                .memoryGroupsBitMap = 4u,  /* Choose R50 TMU3 */
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 1u,
                .memoryGroupsBitMap = 5u,  /* Choose R50 TMU4 */
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 2u,
                .memoryGroupsBitMap = 6u,  /* Choose R50 TMU5 */
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 3u,
                .memoryGroupsBitMap = 7u,  /* Choose R50 TMU6 */
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 1u,
                .memoryGroupsBitMap = 8u,  /* Choose R51 TMU1 */
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 2u,
                .memoryGroupsBitMap = 9u,  /* Choose R51 TMU2 */
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 3u,
                .memoryGroupsBitMap = 10u,  /* Choose R51 TMU3 */
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 1u,
                .memoryGroupsBitMap = 11u,  /* Choose R51 TMU4 */
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 2u,
                .memoryGroupsBitMap = 12u,  /* Choose R51 TMU5 */
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 3u,
                .memoryGroupsBitMap = 13u,  /* Choose R51 TMU6 */
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 4u,
                .memoryGroupsBitMap = 14u,  /* Choose PBISTROM */
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 5u,
                .memoryGroupsBitMap = 15u,  /* Choose ROM0 */
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 6u,
                .memoryGroupsBitMap = 16u,  /* Choose ROM1 */
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 7u,
                .memoryGroupsBitMap = 17u,  /* Choose CPSW*/
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 7u,
                .memoryGroupsBitMap = 18u,  /* ECU_PERIPH */
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 7u,
                .memoryGroupsBitMap = 19u,  /* FOTA */
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 7u,
                .memoryGroupsBitMap = 20u,  /* Choose ICSSM RAM */
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 7u,
                .memoryGroupsBitMap = 21u,  /* Choose MBOX */
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 7u,
                .memoryGroupsBitMap = 23u,  /* Choose MSS_L2_1 */
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 7u,
                .memoryGroupsBitMap = 24u,  /* Choose MSS_L2_2 */
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 7u,
                .memoryGroupsBitMap = 25u,  /* Choose MSS_L2_3 */
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 7u,
                .memoryGroupsBitMap = 26u,  /* Choose MSS_L2_4 */
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 7u,
                .memoryGroupsBitMap = 27u,  /* Choose MSS_L2_5 */
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 7u,
                .memoryGroupsBitMap = 28u,  /* Choose TPCC */
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 7u,
                .memoryGroupsBitMap = 29u,  /* Choose OSPI */
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 7u,
                .memoryGroupsBitMap = 30u,  /* Choose R5SS0_VIM0 */
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 7u,
                .memoryGroupsBitMap = 31u,  /* Choose R5SS0_VIM1 */
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 7u,
                .memoryGroupsBitMap = 32u,  /* Choose MSS R5SS0 CPU0 RL2 */
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 7u,
                .memoryGroupsBitMap = 33u,  /* Choose MSS R5SS0 CPU1 RL2 */
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 7u,
                .memoryGroupsBitMap = 36u,  /* Choose MSS R5SS1 CPU0 RL2 */
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 7u,
                .memoryGroupsBitMap = 37u,  /* Choose MSS R5SS1 CPU1 RL2 */
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 7u,
                .memoryGroupsBitMap = 47u,  /* Choose MSS R5SS0 C0*/
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 7u,
                .memoryGroupsBitMap = 48u,  /* Choose MSS R5SS0 C1*/
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 7u,
                .memoryGroupsBitMap = 49u,  /* Choose MSS R5SS1 C0*/
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 7u,
                .memoryGroupsBitMap = 50u,  /* Choose MSS R5SS1 C1*/
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
                        {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 7u,
                .memoryGroupsBitMap = 51u,  /* Choose MSS MMCH0*/
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 7u,
                .memoryGroupsBitMap = 52u,  /* Choose MSS MMCH1*/
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 7u,
                .memoryGroupsBitMap = 43u,  /* Choose MSS CR5B ATCM0 R5SS0 */
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 7u,
                .memoryGroupsBitMap = 44u,  /* Choose MSS CR5B ATCM0 R5SS1*/
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 7u,
                .memoryGroupsBitMap = 45u,  /* MSS CR5B BTCM0 R5SS0*/
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 7u,
                .memoryGroupsBitMap = 46u,  /* Choose MSS CR5B BTCM0 R5SS1*/
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 7u,
                .memoryGroupsBitMap = 34u,  /* Choose MSS R5SS1 VIM0*/
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 7u,
                .memoryGroupsBitMap = 35u,  /* Choose MSS R5SS1 VIM1*/
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
        },
        #endif
        #if defined (SUBSYS_R5SS1)
        .numPBISTRuns        = PBIST_MAX_NUM_RUNS,
        .PBISTConfigRun =
        {
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 0u,
                .memoryGroupsBitMap = 0u,  /* Choose R5 STC*/
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 0u,
                .memoryGroupsBitMap = 1u,  /* Choose R51 STC*/
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 1u,
                .memoryGroupsBitMap = 2u,  /* Choose R50 TMU1 */
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 2u,
                .memoryGroupsBitMap = 3u,  /* Choose R50 TMU2 */
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 3u,
                .memoryGroupsBitMap = 4u,  /* Choose R50 TMU3 */
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 1u,
                .memoryGroupsBitMap = 5u,  /* Choose R50 TMU4 */
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 2u,
                .memoryGroupsBitMap = 6u,  /* Choose R50 TMU5 */
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 3u,
                .memoryGroupsBitMap = 7u,  /* Choose R50 TMU6 */
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 1u,
                .memoryGroupsBitMap = 8u,  /* Choose R51 TMU1 */
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 2u,
                .memoryGroupsBitMap = 9u,  /* Choose R51 TMU2 */
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 3u,
                .memoryGroupsBitMap = 10u,  /* Choose R51 TMU3 */
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 1u,
                .memoryGroupsBitMap = 11u,  /* Choose R51 TMU4 */
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 2u,
                .memoryGroupsBitMap = 12u,  /* Choose R51 TMU5 */
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 3u,
                .memoryGroupsBitMap = 13u,  /* Choose R51 TMU6 */
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 4u,
                .memoryGroupsBitMap = 14u,  /* Choose PBISTROM */
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 5u,
                .memoryGroupsBitMap = 15u,  /* Choose ROM0 */
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 6u,
                .memoryGroupsBitMap = 16u,  /* Choose ROM1 */
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 7u,
                .memoryGroupsBitMap = 17u,  /* Choose CPSW*/
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 7u,
                .memoryGroupsBitMap = 18u,  /* ECU_PERIPH */
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 7u,
                .memoryGroupsBitMap = 19u,  /* FOTA */
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 7u,
                .memoryGroupsBitMap = 20u,  /* Choose ICSSM RAM */
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 7u,
                .memoryGroupsBitMap = 21u,  /* Choose MBOX */
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 7u,
                .memoryGroupsBitMap = 23u,  /* Choose MSS_L2_1 */
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 7u,
                .memoryGroupsBitMap = 24u,  /* Choose MSS_L2_2 */
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 7u,
                .memoryGroupsBitMap = 25u,  /* Choose MSS_L2_3 */
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 7u,
                .memoryGroupsBitMap = 26u,  /* Choose MSS_L2_4 */
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 7u,
                .memoryGroupsBitMap = 27u,  /* Choose MSS_L2_5 */
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 7u,
                .memoryGroupsBitMap = 28u,  /* Choose TPCC */
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 7u,
                .memoryGroupsBitMap = 29u,  /* Choose OSPI */
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 7u,
                .memoryGroupsBitMap = 30u,  /* Choose R5SS0_VIM0 */
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 7u,
                .memoryGroupsBitMap = 31u,  /* Choose R5SS0_VIM1 */
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 7u,
                .memoryGroupsBitMap = 32u,  /* Choose MSS R5SS0 CPU0 RL2 */
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 7u,
                .memoryGroupsBitMap = 33u,  /* Choose MSS R5SS0 CPU1 RL2 */
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 7u,
                .memoryGroupsBitMap = 36u,  /* Choose MSS R5SS1 CPU0 RL2 */
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 7u,
                .memoryGroupsBitMap = 37u,  /* Choose MSS R5SS1 CPU1 RL2 */
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 7u,
                .memoryGroupsBitMap = 47u,  /* Choose MSS R5SS0 C0*/
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 7u,
                .memoryGroupsBitMap = 48u,  /* Choose MSS R5SS0 C1*/
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 7u,
                .memoryGroupsBitMap = 49u,  /* Choose MSS R5SS1 C0*/
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 7u,
                .memoryGroupsBitMap = 50u,  /* Choose MSS R5SS1 C1*/
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
                        {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 7u,
                .memoryGroupsBitMap = 51u,  /* Choose MSS MMCH0*/
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 7u,
                .memoryGroupsBitMap = 52u,  /* Choose MSS MMCH1*/
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 7u,
                .memoryGroupsBitMap = 43u,  /* Choose MSS CR5B ATCM0 R5SS0 */
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 7u,
                .memoryGroupsBitMap = 44u,  /* Choose MSS CR5B ATCM0 R5SS1*/
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 7u,
                .memoryGroupsBitMap = 45u,  /* MSS CR5B BTCM0 R5SS0*/
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 7u,
                .memoryGroupsBitMap = 46u,  /* Choose MSS CR5B BTCM0 R5SS1*/
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 7u,
                .memoryGroupsBitMap = 34u,  /* Choose MSS R5SS1 VIM0*/
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
            {
                .override           = 0x0u,
                /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                .algorithmsBitMap   = 7u,
                .memoryGroupsBitMap = 35u,  /* Choose MSS R5SS1 VIM1*/
                .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
            },
        },
        #endif
        .PBISTNegConfigRun =
        {
            .CA0   = SDL_PBIST0_FAIL_INSERTION_TEST_VECTOR_CA0,
            .CA1   = SDL_PBIST0_FAIL_INSERTION_TEST_VECTOR_CA1,
            .CA2   = SDL_PBIST0_FAIL_INSERTION_TEST_VECTOR_CA2,
            .CA3   = SDL_PBIST0_FAIL_INSERTION_TEST_VECTOR_CA3,
            .CL0   = SDL_PBIST0_FAIL_INSERTION_TEST_VECTOR_CL0,
            .CL1   = SDL_PBIST0_FAIL_INSERTION_TEST_VECTOR_CL1,
            .CL2   = SDL_PBIST0_FAIL_INSERTION_TEST_VECTOR_CL2,
            .CL3   = SDL_PBIST0_FAIL_INSERTION_TEST_VECTOR_CL3,
            .CMS   = SDL_PBIST0_FAIL_INSERTION_TEST_VECTOR_CMS,
            .CSR   = SDL_PBIST0_FAIL_INSERTION_TEST_VECTOR_CSR,
            .I0    = SDL_PBIST0_FAIL_INSERTION_TEST_VECTOR_I0,
            .I1    = SDL_PBIST0_FAIL_INSERTION_TEST_VECTOR_I1,
            .I2    = SDL_PBIST0_FAIL_INSERTION_TEST_VECTOR_I2,
            .I3    = SDL_PBIST0_FAIL_INSERTION_TEST_VECTOR_I3,
            .RAMT  = SDL_PBIST0_FAIL_INSERTION_TEST_VECTOR_RAMT
        },

        .interruptNumber        = SDL_INTR_PBIST_DONE,
        .doneFlag               = PBIST_NOT_DONE
    },
};


SDL_pbistInstInfo * SDL_PBIST_getInstInfo(SDL_PBIST_inst instance)
{
    SDL_pbistInstInfo *pInfo = (SDL_pbistInstInfo *)NULL;

    if ((uint32_t)instance < SDL_PBIST_NUM_INSTANCES)
    {
        pInfo = &SDL_PBIST_InstInfoArray[instance];
    }

    return pInfo;
}


void SDL_PBIST_eventHandler( uint32_t instanceId)
{

    SDL_PBIST_InstInfoArray[instanceId].doneFlag = PBIST_DONE;

    return;
}
