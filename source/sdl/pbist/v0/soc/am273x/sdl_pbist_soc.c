/*
 *   Copyright (c) Texas Instruments Incorporated 2022
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


#define SDL_PBIST1_NUM_TEST_VECTORS                                                                (1U)
#define SDL_PBIST1_ALGO_BITMAP_0                                                                   (0x0000000000000010U)
#define SDL_PBIST1_MEM_BITMAP_0                                                                    (0x0000000000000008U)
#define SDL_PBIST1_FAIL_INSERTION_TEST_VECTOR_CA0                                                  (0x00001800U)
#define SDL_PBIST1_FAIL_INSERTION_TEST_VECTOR_CA1                                                  (0x00001FFFU)
#define SDL_PBIST1_FAIL_INSERTION_TEST_VECTOR_CA2                                                  (0x000007FFU)
#define SDL_PBIST1_FAIL_INSERTION_TEST_VECTOR_CA3                                                  (0x00000000U)
#define SDL_PBIST1_FAIL_INSERTION_TEST_VECTOR_CL0                                                  (0x0000007FU)
#define SDL_PBIST1_FAIL_INSERTION_TEST_VECTOR_CL1                                                  (0x0000000FU)
#define SDL_PBIST1_FAIL_INSERTION_TEST_VECTOR_CL2                                                  (0x0000000AU)
#define SDL_PBIST1_FAIL_INSERTION_TEST_VECTOR_CL3                                                  (0x000007FFU)
#define SDL_PBIST1_FAIL_INSERTION_TEST_VECTOR_CMS                                                  (0x00000001U)
#define SDL_PBIST1_FAIL_INSERTION_TEST_VECTOR_CSR                                                  (0x00000001U)
#define SDL_PBIST1_FAIL_INSERTION_TEST_VECTOR_I0                                                   (0x00000001U)
#define SDL_PBIST1_FAIL_INSERTION_TEST_VECTOR_I1                                                   (0x00000010U)
#define SDL_PBIST1_FAIL_INSERTION_TEST_VECTOR_I2                                                   (0x0000000AU)
#define SDL_PBIST1_FAIL_INSERTION_TEST_VECTOR_I3                                                   (0x00000000U)
#define SDL_PBIST1_FAIL_INSERTION_TEST_VECTOR_RAMT                                                 (0x1B00200CU)

#define SDL_PBIST2_NUM_TEST_VECTORS                                                                (1U)
#define SDL_PBIST2_ALGO_BITMAP_0                                                                   (0x0000000000000001U)
#define SDL_PBIST2_MEM_BITMAP_0                                                                    (0x0000000000000004U)
#define SDL_PBIST2_FAIL_INSERTION_TEST_VECTOR_CA0                                                  (0x00001800U)
#define SDL_PBIST2_FAIL_INSERTION_TEST_VECTOR_CA1                                                  (0x00001FFFU)
#define SDL_PBIST2_FAIL_INSERTION_TEST_VECTOR_CA2                                                  (0x000007FFU)
#define SDL_PBIST2_FAIL_INSERTION_TEST_VECTOR_CA3                                                  (0x00000000U)
#define SDL_PBIST2_FAIL_INSERTION_TEST_VECTOR_CL0                                                  (0x0000007FU)
#define SDL_PBIST2_FAIL_INSERTION_TEST_VECTOR_CL1                                                  (0x0000000FU)
#define SDL_PBIST2_FAIL_INSERTION_TEST_VECTOR_CL2                                                  (0x0000000AU)
#define SDL_PBIST2_FAIL_INSERTION_TEST_VECTOR_CL3                                                  (0x000007FFU)
#define SDL_PBIST2_FAIL_INSERTION_TEST_VECTOR_CMS                                                  (0x00000001U)
#define SDL_PBIST2_FAIL_INSERTION_TEST_VECTOR_CSR                                                  (0x00000001U)
#define SDL_PBIST2_FAIL_INSERTION_TEST_VECTOR_I0                                                   (0x00000001U)
#define SDL_PBIST2_FAIL_INSERTION_TEST_VECTOR_I1                                                   (0x00000010U)
#define SDL_PBIST2_FAIL_INSERTION_TEST_VECTOR_I2                                                   (0x0000000AU)
#define SDL_PBIST2_FAIL_INSERTION_TEST_VECTOR_I3                                                   (0x00000000U)
#define SDL_PBIST2_FAIL_INSERTION_TEST_VECTOR_RAMT                                                 (0x7900200CU)


/*
 * Note: the order for the below must match order of SDL_PBIST_inst
 */
 #if defined(SUBSYS_R5FSS0)
 static SDL_pbistInstInfo SDL_PBIST_InstInfoArray[SDL_PBIST_NUM_INSTANCES] =
 {
   /* MCU R5F Instance */
   {
       .PBISTRegsHiAddress  = 0u,
       .pPBISTRegs          = (SDL_pbistRegs *)SDL_TOP_PBIST_U_BASE,
       /*
        * Golden vectors not currently supported running from MCU core because
        * this will be self destructive. Below select memory which is guaranteed
        * not to be utilized can be run.
        */
       .numPBISTRuns        = 15u,
       .PBISTConfigRun1 =
       {
           {
               .override           = 0x0u,
               /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
               /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
               .algorithmsBitMap   = 4u,   /* Choose March 13N Single Port */
               .memoryGroupsBitMap = 19u,  /* Choose TPCC RAM*/
               .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
           },
     {
               .override           = 0x0u,
               /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
               /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
               .algorithmsBitMap   = 4u,   /* Choose March 13N Single Port */
               .memoryGroupsBitMap = 16u,  /* Choose MBOX RAM*/
               .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
           },
     {
               .override           = 0x0u,
               /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
               /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
               .algorithmsBitMap   = 4u,   /* Choose March 13N Single Port */
               .memoryGroupsBitMap = 21u,  /* Choose CB VIM RAM*/
               .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
           },
     {
               .override           = 0x0u,
               /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
               /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
               .algorithmsBitMap   = 4u,   /* Choose March 13N Single Port */
               .memoryGroupsBitMap = 17u,  /* Choose MCAN RAM */
               .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
           },
     {
               .override           = 0x0u,
               /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
               /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
               .algorithmsBitMap   = 5u,   /* Choose March 13N Two Port */
               .memoryGroupsBitMap = 24u,  /* Choose SPIA RAM */
               .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
           },
     {
               .override           = 0x0u,
               /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
               /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
               .algorithmsBitMap   = 5u,   /* Choose March 13N Two Port */
               .memoryGroupsBitMap = 25u,  /* Choose SPIB RAM */
               .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
           },
     {
               .override           = 0x0u,
               /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
               /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
               .algorithmsBitMap   = 4u,   /* Choose March 13N Single Port */
               .memoryGroupsBitMap = 10u,  /* Choose CB R5SS RAM */
               .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
           },
     {
               .override           = 0x0u,
               /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
               /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
               .algorithmsBitMap   = 4u,   /* Choose March 13N Single Port */
               .memoryGroupsBitMap = 15u,  /* Choose MSS_L2_1 RAM */
               .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
           },
     {
               .override           = 0x0u,
               /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
               /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
               .algorithmsBitMap   = 4u,   /* Choose March 13N Single Port */
               .memoryGroupsBitMap = 4u,  /* Choose CPSW RAM */
               .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
           },
     {
               .override           = 0x0u,
               /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
               /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
               .algorithmsBitMap   = 4u,   /* Choose March 13N Single Port */
               .memoryGroupsBitMap = 13u,  /* Choose GPADC RAM */
               .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
           },
     {
               .override           = 0x0u,
               /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
               /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
               .algorithmsBitMap   = 4u,   /* Choose March 13N Single Port */
               .memoryGroupsBitMap = 18u,  /* Choose RETRAM RAM */
               .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
           },
     {
               .override           = 0x0u,
               /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
               /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
               .algorithmsBitMap   = 2u,   /* Choose ROM - Triple_Read_XOR_Read */
               .memoryGroupsBitMap = 2u,  /* Choose STCROM RAM */
               .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
           },
           {
               .override           = 0x0u,
               /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
               /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
               .algorithmsBitMap   = 4u,   /* Choose March 13N Single Port */
               .memoryGroupsBitMap = 7u,   /* Choose CB ATCM RAM */
               .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
           },
           {
               .override           = 0x0u,
               /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
               /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
               .algorithmsBitMap   = 4u,   /* Choose March 13N Single Port */
               .memoryGroupsBitMap = 8u,   /* Choose CB BTCM RAM */
               .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
             },
       },
       .PBISTNegConfigRun =
       {
           .CA0   = SDL_PBIST1_FAIL_INSERTION_TEST_VECTOR_CA0,
           .CA1   = SDL_PBIST1_FAIL_INSERTION_TEST_VECTOR_CA1,
           .CA2   = SDL_PBIST1_FAIL_INSERTION_TEST_VECTOR_CA2,
           .CA3   = SDL_PBIST1_FAIL_INSERTION_TEST_VECTOR_CA3,
           .CL0   = SDL_PBIST1_FAIL_INSERTION_TEST_VECTOR_CL0,
           .CL1   = SDL_PBIST1_FAIL_INSERTION_TEST_VECTOR_CL1,
           .CL2   = SDL_PBIST1_FAIL_INSERTION_TEST_VECTOR_CL2,
           .CL3   = SDL_PBIST1_FAIL_INSERTION_TEST_VECTOR_CL3,
           .CMS   = SDL_PBIST1_FAIL_INSERTION_TEST_VECTOR_CMS,
           .CSR   = SDL_PBIST1_FAIL_INSERTION_TEST_VECTOR_CSR,
           .I0    = SDL_PBIST1_FAIL_INSERTION_TEST_VECTOR_I0,
           .I1    = SDL_PBIST1_FAIL_INSERTION_TEST_VECTOR_I1,
           .I2    = SDL_PBIST1_FAIL_INSERTION_TEST_VECTOR_I2,
           .I3    = SDL_PBIST1_FAIL_INSERTION_TEST_VECTOR_I3,
           .RAMT  = SDL_PBIST1_FAIL_INSERTION_TEST_VECTOR_RAMT
       },
       .interruptNumber        = SDL_MSS_INTR_TOP_PBIST_DONE_INT,
       .doneFlag               = PBIST_NOT_DONE
   },
   /* C66 DSP Instance */
   {
       .PBISTRegsHiAddress  = 0u,
       .pPBISTRegs          = (SDL_pbistRegs *)SDL_DSS_DSP_PBIST_U_BASE,
       /*
        * Golden vectors not currently supported running from MCU core because
        * this will be self destructive. Below select memory which is guaranteed
        * not to be utilized can be run.
        */
       .numPBISTRuns        = 22u,
       .PBISTConfigRun2 = {

         {
             .override           = 0x0u,
             /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
             /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
             .algorithmsBitMap   = 0u,   /* ROM - Triple_Read_XOR_Read */
             .memoryGroupsBitMap = 0u,  /* Choose DSS C66 STCROM*/
             .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
         },
         {
             .override           = 0x0u,
             /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
             /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
             .algorithmsBitMap   = 1u,   /* ROM - Triple_Read_XOR_Read */
             .memoryGroupsBitMap = 1u,  /* Choose HWA STCROM*/
             .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
         },
         {
             .override           = 0x0u,
             /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
             /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
             .algorithmsBitMap   = 2u,   /* ROM - Triple_Read_XOR_Read */
             .memoryGroupsBitMap = 2u,  /* Choose DSS PBIST ROM*/
             .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
         },
         {
             .override           = 0x0u,
             /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
             /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
             .algorithmsBitMap   = 3u,   /* ROM - Triple_Read_XOR_Read */
             .memoryGroupsBitMap = 3u,  /* Choose C66x L1D*/
             .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
         },
         {
             .override           = 0x0u,
             /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
             /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
             .algorithmsBitMap   = 3u,   /* ROM - Triple_Read_XOR_Read */
             .memoryGroupsBitMap = 3u,  /* Choose C66x L1P*/
             .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
         },
         {
                   .override           = 0x0u,
                   /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                   /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                   .algorithmsBitMap   = 3u,   /* ROM - Triple_Read_XOR_Read */
                   .memoryGroupsBitMap = 3u,  /* Choose PBIST C66 L2 TAG*/
                   .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
          },
          {
                    .override           = 0x0u,
                    /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                    /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                    .algorithmsBitMap   = 3u,   /* ROM - Triple_Read_XOR_Read */
                    .memoryGroupsBitMap = 6u,  /* Choose DSS HWA*/
                    .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
           },
           {
                       .override           = 0x0u,
                       /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                       /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                       .algorithmsBitMap   = 3u,   /* ROM - Triple_Read_XOR_Read */
                       .memoryGroupsBitMap = 7u,  /* Choose DSS HWA MBOX*/
                       .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
           },
           {
                     .override           = 0x0u,
                     /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                     /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                     .algorithmsBitMap   = 3u,   /* ROM - Triple_Read_XOR_Read */
                     .memoryGroupsBitMap = 8u,  /* Choose PBIST DSS L3 BANKA SUB0*/
                     .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
           },
           {
                     .override           = 0x0u,
                     /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                     /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                     .algorithmsBitMap   = 3u,   /* ROM - Triple_Read_XOR_Read */
                     .memoryGroupsBitMap = 9u,  /* Choose PBIST DSS L3 BANKB SUB0*/
                     .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
           },
           {
                     .override           = 0x0u,
                     /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                     /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                     .algorithmsBitMap   = 3u,   /* ROM - Triple_Read_XOR_Read */
                     .memoryGroupsBitMap = 10u,  /* Choose PBIST DSS L3 BANKC SUB0*/
                     .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
           },
           {
                   .override           = 0x0u,
                   /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                   /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                   .algorithmsBitMap   = 3u,   /* ROM - Triple_Read_XOR_Read */
                   .memoryGroupsBitMap = 12u,  /* Choose DSS MBOX RAM*/
                   .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
           },
           {
                   .override           = 0x0u,
                   /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                   /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                   .algorithmsBitMap   = 3u,   /* ROM - Triple_Read_XOR_Read */
                   .memoryGroupsBitMap = 13u,  /* Choose DSS TPCC RAM*/
                   .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
           },
           {
                   .override           = 0x0u,
                   /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                   /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                   .algorithmsBitMap   = 3u,   /* ROM - Triple_Read_XOR_Read */
                   .memoryGroupsBitMap = 14u,  /* Choose DSS L2 BANK0*/
                   .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
           },
           {
                   .override           = 0x0u,
                   /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                   /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                   .algorithmsBitMap   = 3u,   /* ROM - Triple_Read_XOR_Read */
                   .memoryGroupsBitMap = 15u,  /* Choose DSS L2 BANK1*/
                   .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
           },
           {
                   .override           = 0x0u,
                   /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                   /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                   .algorithmsBitMap   = 3u,   /* ROM - Triple_Read_XOR_Read */
                   .memoryGroupsBitMap = 16u,  /* Choose DSS L2 PARITY */
                   .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
           },
           {
                   .override           = 0x0u,
                   /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                   /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                   .algorithmsBitMap   = 3u,   /* ROM - Triple_Read_XOR_Read */
                   .memoryGroupsBitMap = 17u,  /* Choose HWA RAM*/
                   .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
           },
           {
                     .override           = 0x0u,
                     /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                     /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                     .algorithmsBitMap   = 4u,   /* Choose March 13N Two Port */
                     .memoryGroupsBitMap = 18u,  /* Choose DSS CBUF */
                     .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
           },
           {
                    .override           = 0x0u,
                    /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                    /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                    .algorithmsBitMap   = 3u,   /* ROM - Triple_Read_XOR_Read */
                    .memoryGroupsBitMap = 19u,  /* Choose PBIST DSS L3 BANKA SUB1*/
                    .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
           },
           {
                    .override           = 0x0u,
                    /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                    /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                    .algorithmsBitMap   = 3u,   /* ROM - Triple_Read_XOR_Read */
                    .memoryGroupsBitMap = 21u,  /* Choose PBIST DSS L3 BANKB SUB1*/
                    .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
           },
           {
                    .override           = 0x0u,
                    /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                    /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                    .algorithmsBitMap   = 3u,   /* ROM - Triple_Read_XOR_Read */
                    .memoryGroupsBitMap = 22u,  /* Choose PBIST DSS L3 BANKB SUB2*/
                    .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
           },
           {
                    .override           = 0x0u,
                    /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                    /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                    .algorithmsBitMap   = 3u,   /* ROM - Triple_Read_XOR_Read */
                    .memoryGroupsBitMap = 23u,  /* Choose PBIST DSS L3 BANKC SUB1*/
                    .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
           },

       },
       .PBISTNegConfigRun =
       {
         .CA0   = SDL_PBIST2_FAIL_INSERTION_TEST_VECTOR_CA0,
         .CA1   = SDL_PBIST2_FAIL_INSERTION_TEST_VECTOR_CA1,
         .CA2   = SDL_PBIST2_FAIL_INSERTION_TEST_VECTOR_CA2,
         .CA3   = SDL_PBIST2_FAIL_INSERTION_TEST_VECTOR_CA3,
         .CL0   = SDL_PBIST2_FAIL_INSERTION_TEST_VECTOR_CL0,
         .CL1   = SDL_PBIST2_FAIL_INSERTION_TEST_VECTOR_CL1,
         .CL2   = SDL_PBIST2_FAIL_INSERTION_TEST_VECTOR_CL2,
         .CL3   = SDL_PBIST2_FAIL_INSERTION_TEST_VECTOR_CL3,
         .CMS   = SDL_PBIST2_FAIL_INSERTION_TEST_VECTOR_CMS,
         .CSR   = SDL_PBIST2_FAIL_INSERTION_TEST_VECTOR_CSR,
         .I0    = SDL_PBIST2_FAIL_INSERTION_TEST_VECTOR_I0,
         .I1    = SDL_PBIST2_FAIL_INSERTION_TEST_VECTOR_I1,
         .I2    = SDL_PBIST2_FAIL_INSERTION_TEST_VECTOR_I2,
         .I3    = SDL_PBIST2_FAIL_INSERTION_TEST_VECTOR_I3,
         .RAMT  = SDL_PBIST2_FAIL_INSERTION_TEST_VECTOR_RAMT
       },
       .interruptNumber        = SDL_MSS_INTR_DSS_DSP_PBIST_CTRL_DONE,
       .doneFlag               = PBIST_NOT_DONE
   },
 };
 #endif
 #if defined(SUBSYS_R5FSS1)
 static SDL_pbistInstInfo SDL_PBIST_InstInfoArray[1] =
 {
     /* MCU R5F Instance */
     {
         .PBISTRegsHiAddress  = 0u,
         .pPBISTRegs          = (SDL_pbistRegs *)SDL_TOP_PBIST_U_BASE,
         /*
          * Golden vectors not currently supported running from MCU core because
          * this will be self destructive. Below select memory which is guaranteed
          * not to be utilized can be run.
          */
         .numPBISTRuns        = 10u,
         .PBISTConfigRun1 = {
           {
               .override           = 0x0u,
               /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
               /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
               .algorithmsBitMap   = 0u,   /* Choose March 13N Single Port */
               .memoryGroupsBitMap = 0u,   /* Choose MSS_TCMAROM_0 RAM */
               .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
             },
             {
                 .override           = 0x0u,
                 /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                 /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                 .algorithmsBitMap   = 1u,   /* Choose March 13N Single Port */
                 .memoryGroupsBitMap = 1u,   /* Choose MSS_TCMAROM_1 RAM */
                 .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
               },

 			{
                 .override           = 0x0u,
                 /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                 /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                 .algorithmsBitMap   = 3u,   /* Choose ROM - Triple_Read_XOR_Read */
                 .memoryGroupsBitMap = 3u,  /* Choose PBISTROM */
                 .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
             },
 			{
                 .override           = 0x0u,
                 /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                 /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                 .algorithmsBitMap   = 4u,   /* Choose March 13N Single Port */
                 .memoryGroupsBitMap = 20u,  /* Choose CA VIM RAM*/
                 .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
             },
 			{
                 .override           = 0x0u,
                 /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                 /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                 .algorithmsBitMap   = 4u,   /* Choose March 13N Single Port */
                 .memoryGroupsBitMap = 14u,  /* Choose MSS_L2_0 RAM */
                 .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
             },
 			      {
                 .override           = 0x0u,
                 /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                 /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                 .algorithmsBitMap   = 4u,   /* Choose March 13N Single Port */
                 .memoryGroupsBitMap = 5u,   /* Choose CA ATCM RAM */
                 .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
             },
             {
                 .override           = 0x0u,
                 /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                 /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                 .algorithmsBitMap   = 4u,   /* Choose March 13N Single Port */
                 .memoryGroupsBitMap = 6u,   /* Choose CA BTCM RAM */
                 .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
               },
             {
                   .override           = 0x0u,
                   /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                   /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                   .algorithmsBitMap   = 4u,   /* Choose March 13N Single Port */
                   .memoryGroupsBitMap = 11u,   /* Choose MEM_TOP_AURORA RAM */
                   .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
             },
             {
                 .override           = 0x0u,
                 /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                 /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                 .algorithmsBitMap   = 4u,   /* Choose March 13N Single Port */
                 .memoryGroupsBitMap = 12u,   /* Choose MEM_TOP_MDO RAM */
                 .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
               },
               {
                   .override           = 0x0u,
                   /* Override bit set to 0 to use memoryGroupsBitMap & algorithmsBitMap */
                   /* NOTE: As the MCU R5f is running this code override bit cannot be set to enable all memory test */
                   .algorithmsBitMap   = 4u,   /* Choose March 13N Single Port */
                   .memoryGroupsBitMap = 23u,   /* Choose DBGSS_TRACE RAM */
                   .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
                 },

 			{
                 .override           = 0x0u,
                 /* Fault injection test: Configure incorrect algorithm and memory
                  * group combination. This is as per the safety manual.
                  * Select SDL_PBIST_MEMGRP_MEM_TOP_PBISTROM with MARCH_13 algorithm */
                 .algorithmsBitMap   = 4u,   /* Choose March 13N Single Port */
                 .memoryGroupsBitMap = 3u,   /* Choose PBISTROM */
                 .scrambleValue      = 0xFEDCBA9876543210U,  /* Scramble Value */
             },
         },
         .PBISTNegConfigRun =
         {
           .CA0   = SDL_PBIST1_FAIL_INSERTION_TEST_VECTOR_CA0,
           .CA1   = SDL_PBIST1_FAIL_INSERTION_TEST_VECTOR_CA1,
           .CA2   = SDL_PBIST1_FAIL_INSERTION_TEST_VECTOR_CA2,
           .CA3   = SDL_PBIST1_FAIL_INSERTION_TEST_VECTOR_CA3,
           .CL0   = SDL_PBIST1_FAIL_INSERTION_TEST_VECTOR_CL0,
           .CL1   = SDL_PBIST1_FAIL_INSERTION_TEST_VECTOR_CL1,
           .CL2   = SDL_PBIST1_FAIL_INSERTION_TEST_VECTOR_CL2,
           .CL3   = SDL_PBIST1_FAIL_INSERTION_TEST_VECTOR_CL3,
           .CMS   = SDL_PBIST1_FAIL_INSERTION_TEST_VECTOR_CMS,
           .CSR   = SDL_PBIST1_FAIL_INSERTION_TEST_VECTOR_CSR,
           .I0    = SDL_PBIST1_FAIL_INSERTION_TEST_VECTOR_I0,
           .I1    = SDL_PBIST1_FAIL_INSERTION_TEST_VECTOR_I1,
           .I2    = SDL_PBIST1_FAIL_INSERTION_TEST_VECTOR_I2,
           .I3    = SDL_PBIST1_FAIL_INSERTION_TEST_VECTOR_I3,
           .RAMT  = SDL_PBIST1_FAIL_INSERTION_TEST_VECTOR_RAMT
         },
         .interruptNumber        = SDL_MSS_INTR_TOP_PBIST_DONE_INT,
         .doneFlag               = PBIST_NOT_DONE
     },
 };
 #endif

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
