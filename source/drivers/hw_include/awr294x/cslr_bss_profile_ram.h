/********************************************************************
 * Copyright (C) 2021 Texas Instruments Incorporated.
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
 *  Name        : cslr_bss_profile_ram.h
*/
#ifndef CSLR_BSS_PROFILE_RAM_H_
#define CSLR_BSS_PROFILE_RAM_H_

#include <drivers/hw_include/cslr.h>
#include <drivers/hw_include/tistdtypes.h>

#ifdef __cplusplus
extern "C"
{
#endif

/**************************************************************************
* Hardware Region  :
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t PROFILE1_R1;
    volatile uint32_t PROFILE1_R2;
    volatile uint32_t PROFILE1_R3;
    volatile uint32_t PROFILE1_R4;
    volatile uint32_t PROFILE1_R5;
    volatile uint32_t PROFILE1_R6;
    volatile uint32_t PROFILE1_R7;
    volatile uint32_t PROFILE1_R8;
    volatile uint32_t PROFILE1_R9;
    volatile uint32_t PROFILE1_R10;
    volatile uint32_t PROFILE1_R11;
    volatile uint32_t PROFILE1_R12;
    volatile uint32_t PROFILE1_R13;
    volatile uint8_t  Resv_64[12];
    volatile uint32_t PROFILE2_R1;
    volatile uint32_t PROFILE2_R2;
    volatile uint32_t PROFILE2_R3;
    volatile uint32_t PROFILE2_R4;
    volatile uint32_t PROFILE2_R5;
    volatile uint32_t PROFILE2_R6;
    volatile uint32_t PROFILE2_R7;
    volatile uint32_t PROFILE2_R8;
    volatile uint32_t PROFILE2_R9;
    volatile uint32_t PROFILE2_R10;
    volatile uint32_t PROFILE2_R11;
    volatile uint32_t PROFILE2_R12;
    volatile uint32_t PROFILE2_R13;
    volatile uint8_t  Resv_128[12];
    volatile uint32_t PROFILE3_R1;
    volatile uint32_t PROFILE3_R2;
    volatile uint32_t PROFILE3_R3;
    volatile uint32_t PROFILE3_R4;
    volatile uint32_t PROFILE3_R5;
    volatile uint32_t PROFILE3_R6;
    volatile uint32_t PROFILE3_R7;
    volatile uint32_t PROFILE3_R8;
    volatile uint32_t PROFILE3_R9;
    volatile uint32_t PROFILE3_R10;
    volatile uint32_t PROFILE3_R11;
    volatile uint32_t PROFILE3_R12;
    volatile uint32_t PROFILE3_R13;
    volatile uint8_t  Resv_192[12];
    volatile uint32_t PROFILE4_R1;
    volatile uint32_t PROFILE4_R2;
    volatile uint32_t PROFILE4_R3;
    volatile uint32_t PROFILE4_R4;
    volatile uint32_t PROFILE4_R5;
    volatile uint32_t PROFILE4_R6;
    volatile uint32_t PROFILE4_R7;
    volatile uint32_t PROFILE4_R8;
    volatile uint32_t PROFILE4_R9;
    volatile uint32_t PROFILE4_R10;
    volatile uint32_t PROFILE4_R11;
    volatile uint32_t PROFILE4_R12;
    volatile uint32_t PROFILE4_R13;
    volatile uint8_t  Resv_256[12];
    volatile uint32_t PROFILE5_R1;
    volatile uint32_t PROFILE5_R2;
    volatile uint32_t PROFILE5_R3;
    volatile uint32_t PROFILE5_R4;
    volatile uint32_t PROFILE5_R5;
    volatile uint32_t PROFILE5_R6;
    volatile uint32_t PROFILE5_R7;
    volatile uint32_t PROFILE5_R8;
    volatile uint32_t PROFILE5_R9;
    volatile uint32_t PROFILE5_R10;
    volatile uint32_t PROFILE5_R11;
    volatile uint32_t PROFILE5_R12;
    volatile uint32_t PROFILE5_R13;
    volatile uint8_t  Resv_320[12];
    volatile uint32_t PROFILE6_R1;
    volatile uint32_t PROFILE6_R2;
    volatile uint32_t PROFILE6_R3;
    volatile uint32_t PROFILE6_R4;
    volatile uint32_t PROFILE6_R5;
    volatile uint32_t PROFILE6_R6;
    volatile uint32_t PROFILE6_R7;
    volatile uint32_t PROFILE6_R8;
    volatile uint32_t PROFILE6_R9;
    volatile uint32_t PROFILE6_R10;
    volatile uint32_t PROFILE6_R11;
    volatile uint32_t PROFILE6_R12;
    volatile uint32_t PROFILE6_R13;
    volatile uint8_t  Resv_384[12];
    volatile uint32_t PROFILE7_R1;
    volatile uint32_t PROFILE7_R2;
    volatile uint32_t PROFILE7_R3;
    volatile uint32_t PROFILE7_R4;
    volatile uint32_t PROFILE7_R5;
    volatile uint32_t PROFILE7_R6;
    volatile uint32_t PROFILE7_R7;
    volatile uint32_t PROFILE7_R8;
    volatile uint32_t PROFILE7_R9;
    volatile uint32_t PROFILE7_R10;
    volatile uint32_t PROFILE7_R11;
    volatile uint32_t PROFILE7_R12;
    volatile uint32_t PROFILE7_R13;
    volatile uint8_t  Resv_448[12];
    volatile uint32_t PROFILE8_R1;
    volatile uint32_t PROFILE8_R2;
    volatile uint32_t PROFILE8_R3;
    volatile uint32_t PROFILE8_R4;
    volatile uint32_t PROFILE8_R5;
    volatile uint32_t PROFILE8_R6;
    volatile uint32_t PROFILE8_R7;
    volatile uint32_t PROFILE8_R8;
    volatile uint32_t PROFILE8_R9;
    volatile uint32_t PROFILE8_R10;
    volatile uint32_t PROFILE8_R11;
    volatile uint32_t PROFILE8_R12;
    volatile uint32_t PROFILE8_R13;
    volatile uint8_t  Resv_512[12];
    volatile uint32_t PROFILE9_R1;
    volatile uint32_t PROFILE9_R2;
    volatile uint32_t PROFILE9_R3;
    volatile uint32_t PROFILE9_R4;
    volatile uint32_t PROFILE9_R5;
    volatile uint32_t PROFILE9_R6;
    volatile uint32_t PROFILE9_R7;
    volatile uint32_t PROFILE9_R8;
    volatile uint32_t PROFILE9_R9;
    volatile uint32_t PROFILE9_R10;
    volatile uint32_t PROFILE9_R11;
    volatile uint32_t PROFILE9_R12;
    volatile uint32_t PROFILE9_R13;
    volatile uint8_t  Resv_576[12];
    volatile uint32_t PROFILE10_R1;
    volatile uint32_t PROFILE10_R2;
    volatile uint32_t PROFILE10_R3;
    volatile uint32_t PROFILE10_R4;
    volatile uint32_t PROFILE10_R5;
    volatile uint32_t PROFILE10_R6;
    volatile uint32_t PROFILE10_R7;
    volatile uint32_t PROFILE10_R8;
    volatile uint32_t PROFILE10_R9;
    volatile uint32_t PROFILE10_R10;
    volatile uint32_t PROFILE10_R11;
    volatile uint32_t PROFILE10_R12;
    volatile uint32_t PROFILE10_R13;
    volatile uint8_t  Resv_640[12];
    volatile uint32_t PROFILE11_R1;
    volatile uint32_t PROFILE11_R2;
    volatile uint32_t PROFILE11_R3;
    volatile uint32_t PROFILE11_R4;
    volatile uint32_t PROFILE11_R5;
    volatile uint32_t PROFILE11_R6;
    volatile uint32_t PROFILE11_R7;
    volatile uint32_t PROFILE11_R8;
    volatile uint32_t PROFILE11_R9;
    volatile uint32_t PROFILE11_R10;
    volatile uint32_t PROFILE11_R11;
    volatile uint32_t PROFILE11_R12;
    volatile uint32_t PROFILE11_R13;
    volatile uint8_t  Resv_704[12];
    volatile uint32_t PROFILE12_R1;
    volatile uint32_t PROFILE12_R2;
    volatile uint32_t PROFILE12_R3;
    volatile uint32_t PROFILE12_R4;
    volatile uint32_t PROFILE12_R5;
    volatile uint32_t PROFILE12_R6;
    volatile uint32_t PROFILE12_R7;
    volatile uint32_t PROFILE12_R8;
    volatile uint32_t PROFILE12_R9;
    volatile uint32_t PROFILE12_R10;
    volatile uint32_t PROFILE12_R11;
    volatile uint32_t PROFILE12_R12;
    volatile uint32_t PROFILE12_R13;
    volatile uint8_t  Resv_768[12];
    volatile uint32_t PROFILE13_R1;
    volatile uint32_t PROFILE13_R2;
    volatile uint32_t PROFILE13_R3;
    volatile uint32_t PROFILE13_R4;
    volatile uint32_t PROFILE13_R5;
    volatile uint32_t PROFILE13_R6;
    volatile uint32_t PROFILE13_R7;
    volatile uint32_t PROFILE13_R8;
    volatile uint32_t PROFILE13_R9;
    volatile uint32_t PROFILE13_R10;
    volatile uint32_t PROFILE13_R11;
    volatile uint32_t PROFILE13_R12;
    volatile uint32_t PROFILE13_R13;
    volatile uint8_t  Resv_832[12];
    volatile uint32_t PROFILE14_R1;
    volatile uint32_t PROFILE14_R2;
    volatile uint32_t PROFILE14_R3;
    volatile uint32_t PROFILE14_R4;
    volatile uint32_t PROFILE14_R5;
    volatile uint32_t PROFILE14_R6;
    volatile uint32_t PROFILE14_R7;
    volatile uint32_t PROFILE14_R8;
    volatile uint32_t PROFILE14_R9;
    volatile uint32_t PROFILE14_R10;
    volatile uint32_t PROFILE14_R11;
    volatile uint32_t PROFILE14_R12;
    volatile uint32_t PROFILE14_R13;
    volatile uint8_t  Resv_896[12];
    volatile uint32_t PROFILE15_R1;
    volatile uint32_t PROFILE15_R2;
    volatile uint32_t PROFILE15_R3;
    volatile uint32_t PROFILE15_R4;
    volatile uint32_t PROFILE15_R5;
    volatile uint32_t PROFILE15_R6;
    volatile uint32_t PROFILE15_R7;
    volatile uint32_t PROFILE15_R8;
    volatile uint32_t PROFILE15_R9;
    volatile uint32_t PROFILE15_R10;
    volatile uint32_t PROFILE15_R11;
    volatile uint32_t PROFILE15_R12;
    volatile uint32_t PROFILE15_R13;
    volatile uint8_t  Resv_960[12];
    volatile uint32_t PROFILE16_R1;
    volatile uint32_t PROFILE16_R2;
    volatile uint32_t PROFILE16_R3;
    volatile uint32_t PROFILE16_R4;
    volatile uint32_t PROFILE16_R5;
    volatile uint32_t PROFILE16_R6;
    volatile uint32_t PROFILE16_R7;
    volatile uint32_t PROFILE16_R8;
    volatile uint32_t PROFILE16_R9;
    volatile uint32_t PROFILE16_R10;
    volatile uint32_t PROFILE16_R11;
    volatile uint32_t PROFILE16_R12;
    volatile uint32_t PROFILE16_R13;
} CSL_bss_profile_ramRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_BSS_PROFILE_RAM_PROFILE1_R1                                        (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R2                                        (0x00000004U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R3                                        (0x00000008U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R4                                        (0x0000000CU)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R5                                        (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R6                                        (0x00000014U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R7                                        (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R8                                        (0x0000001CU)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R9                                        (0x00000020U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R10                                       (0x00000024U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R11                                       (0x00000028U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R12                                       (0x0000002CU)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R13                                       (0x00000030U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R1                                        (0x00000040U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R2                                        (0x00000044U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R3                                        (0x00000048U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R4                                        (0x0000004CU)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R5                                        (0x00000050U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R6                                        (0x00000054U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R7                                        (0x00000058U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R8                                        (0x0000005CU)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R9                                        (0x00000060U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R10                                       (0x00000064U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R11                                       (0x00000068U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R12                                       (0x0000006CU)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R13                                       (0x00000070U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R1                                        (0x00000080U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R2                                        (0x00000084U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R3                                        (0x00000088U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R4                                        (0x0000008CU)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R5                                        (0x00000090U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R6                                        (0x00000094U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R7                                        (0x00000098U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R8                                        (0x0000009CU)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R9                                        (0x000000A0U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R10                                       (0x000000A4U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R11                                       (0x000000A8U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R12                                       (0x000000ACU)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R13                                       (0x000000B0U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R1                                        (0x000000C0U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R2                                        (0x000000C4U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R3                                        (0x000000C8U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R4                                        (0x000000CCU)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R5                                        (0x000000D0U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R6                                        (0x000000D4U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R7                                        (0x000000D8U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R8                                        (0x000000DCU)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R9                                        (0x000000E0U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R10                                       (0x000000E4U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R11                                       (0x000000E8U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R12                                       (0x000000ECU)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R13                                       (0x000000F0U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R1                                        (0x00000100U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R2                                        (0x00000104U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R3                                        (0x00000108U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R4                                        (0x0000010CU)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R5                                        (0x00000110U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R6                                        (0x00000114U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R7                                        (0x00000118U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R8                                        (0x0000011CU)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R9                                        (0x00000120U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R10                                       (0x00000124U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R11                                       (0x00000128U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R12                                       (0x0000012CU)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R13                                       (0x00000130U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R1                                        (0x00000140U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R2                                        (0x00000144U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R3                                        (0x00000148U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R4                                        (0x0000014CU)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R5                                        (0x00000150U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R6                                        (0x00000154U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R7                                        (0x00000158U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R8                                        (0x0000015CU)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R9                                        (0x00000160U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R10                                       (0x00000164U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R11                                       (0x00000168U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R12                                       (0x0000016CU)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R13                                       (0x00000170U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R1                                        (0x00000180U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R2                                        (0x00000184U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R3                                        (0x00000188U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R4                                        (0x0000018CU)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R5                                        (0x00000190U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R6                                        (0x00000194U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R7                                        (0x00000198U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R8                                        (0x0000019CU)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R9                                        (0x000001A0U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R10                                       (0x000001A4U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R11                                       (0x000001A8U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R12                                       (0x000001ACU)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R13                                       (0x000001B0U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R1                                        (0x000001C0U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R2                                        (0x000001C4U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R3                                        (0x000001C8U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R4                                        (0x000001CCU)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R5                                        (0x000001D0U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R6                                        (0x000001D4U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R7                                        (0x000001D8U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R8                                        (0x000001DCU)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R9                                        (0x000001E0U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R10                                       (0x000001E4U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R11                                       (0x000001E8U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R12                                       (0x000001ECU)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R13                                       (0x000001F0U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R1                                        (0x00000200U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R2                                        (0x00000204U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R3                                        (0x00000208U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R4                                        (0x0000020CU)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R5                                        (0x00000210U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R6                                        (0x00000214U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R7                                        (0x00000218U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R8                                        (0x0000021CU)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R9                                        (0x00000220U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R10                                       (0x00000224U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R11                                       (0x00000228U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R12                                       (0x0000022CU)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R13                                       (0x00000230U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R1                                       (0x00000240U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R2                                       (0x00000244U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R3                                       (0x00000248U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R4                                       (0x0000024CU)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R5                                       (0x00000250U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R6                                       (0x00000254U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R7                                       (0x00000258U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R8                                       (0x0000025CU)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R9                                       (0x00000260U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R10                                      (0x00000264U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R11                                      (0x00000268U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R12                                      (0x0000026CU)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R13                                      (0x00000270U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R1                                       (0x00000280U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R2                                       (0x00000284U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R3                                       (0x00000288U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R4                                       (0x0000028CU)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R5                                       (0x00000290U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R6                                       (0x00000294U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R7                                       (0x00000298U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R8                                       (0x0000029CU)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R9                                       (0x000002A0U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R10                                      (0x000002A4U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R11                                      (0x000002A8U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R12                                      (0x000002ACU)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R13                                      (0x000002B0U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R1                                       (0x000002C0U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R2                                       (0x000002C4U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R3                                       (0x000002C8U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R4                                       (0x000002CCU)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R5                                       (0x000002D0U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R6                                       (0x000002D4U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R7                                       (0x000002D8U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R8                                       (0x000002DCU)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R9                                       (0x000002E0U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R10                                      (0x000002E4U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R11                                      (0x000002E8U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R12                                      (0x000002ECU)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R13                                      (0x000002F0U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R1                                       (0x00000300U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R2                                       (0x00000304U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R3                                       (0x00000308U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R4                                       (0x0000030CU)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R5                                       (0x00000310U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R6                                       (0x00000314U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R7                                       (0x00000318U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R8                                       (0x0000031CU)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R9                                       (0x00000320U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R10                                      (0x00000324U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R11                                      (0x00000328U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R12                                      (0x0000032CU)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R13                                      (0x00000330U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R1                                       (0x00000340U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R2                                       (0x00000344U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R3                                       (0x00000348U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R4                                       (0x0000034CU)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R5                                       (0x00000350U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R6                                       (0x00000354U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R7                                       (0x00000358U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R8                                       (0x0000035CU)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R9                                       (0x00000360U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R10                                      (0x00000364U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R11                                      (0x00000368U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R12                                      (0x0000036CU)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R13                                      (0x00000370U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R1                                       (0x00000380U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R2                                       (0x00000384U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R3                                       (0x00000388U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R4                                       (0x0000038CU)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R5                                       (0x00000390U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R6                                       (0x00000394U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R7                                       (0x00000398U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R8                                       (0x0000039CU)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R9                                       (0x000003A0U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R10                                      (0x000003A4U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R11                                      (0x000003A8U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R12                                      (0x000003ACU)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R13                                      (0x000003B0U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R1                                       (0x000003C0U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R2                                       (0x000003C4U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R3                                       (0x000003C8U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R4                                       (0x000003CCU)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R5                                       (0x000003D0U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R6                                       (0x000003D4U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R7                                       (0x000003D8U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R8                                       (0x000003DCU)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R9                                       (0x000003E0U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R10                                      (0x000003E4U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R11                                      (0x000003E8U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R12                                      (0x000003ECU)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R13                                      (0x000003F0U)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* PROFILE1_R1 */

#define CSL_BSS_PROFILE_RAM_PROFILE1_R1_N_START_CONSTANT_MASK                  (0x7FFFFFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R1_N_START_CONSTANT_SHIFT                 (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R1_N_START_CONSTANT_RESETVAL              (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R1_N_START_CONSTANT_MAX                   (0x7FFFFFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE1_R1_NU_MASK                                (0x80000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R1_NU_SHIFT                               (0x0000001FU)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R1_NU_RESETVAL                            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R1_NU_MAX                                 (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE1_R1_RESETVAL                               (0x00000000U)

/* PROFILE1_R2 */

#define CSL_BSS_PROFILE_RAM_PROFILE1_R2_N_SLOPE_CONSTANT_MASK                  (0x0000FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R2_N_SLOPE_CONSTANT_SHIFT                 (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R2_N_SLOPE_CONSTANT_RESETVAL              (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R2_N_SLOPE_CONSTANT_MAX                   (0x0000FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE1_R2_FAST_RESET_END_TIME_MASK               (0x1FFF0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R2_FAST_RESET_END_TIME_SHIFT              (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R2_FAST_RESET_END_TIME_RESETVAL           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R2_FAST_RESET_END_TIME_MAX                (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE1_R2_DISABLE_FAST_RESET_MASK                (0x20000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R2_DISABLE_FAST_RESET_SHIFT               (0x0000001DU)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R2_DISABLE_FAST_RESET_RESETVAL            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R2_DISABLE_FAST_RESET_MAX                 (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE1_R2_NU_MASK                                (0xC0000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R2_NU_SHIFT                               (0x0000001EU)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R2_NU_RESETVAL                            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R2_NU_MAX                                 (0x00000003U)

#define CSL_BSS_PROFILE_RAM_PROFILE1_R2_RESETVAL                               (0x00000000U)

/* PROFILE1_R3 */

#define CSL_BSS_PROFILE_RAM_PROFILE1_R3_IDLE_TIME_CONSTANT_MASK                (0x0007FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R3_IDLE_TIME_CONSTANT_SHIFT               (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R3_IDLE_TIME_CONSTANT_RESETVAL            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R3_IDLE_TIME_CONSTANT_MAX                 (0x0007FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE1_R3_NU1_MASK                               (0x00F80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R3_NU1_SHIFT                              (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R3_NU1_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R3_NU1_MAX                                (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE1_R3_DISABLE_MONITORING_MASK                (0x01000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R3_DISABLE_MONITORING_SHIFT               (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R3_DISABLE_MONITORING_RESETVAL            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R3_DISABLE_MONITORING_MAX                 (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE1_R3_NU2_MASK                               (0xFE000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R3_NU2_SHIFT                              (0x00000019U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R3_NU2_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R3_NU2_MAX                                (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE1_R3_RESETVAL                               (0x00000000U)

/* PROFILE1_R4 */

#define CSL_BSS_PROFILE_RAM_PROFILE1_R4_ADC_VALID_START_TIME_MASK              (0x0007FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R4_ADC_VALID_START_TIME_SHIFT             (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R4_ADC_VALID_START_TIME_RESETVAL          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R4_ADC_VALID_START_TIME_MAX               (0x0007FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE1_R4_NU1_MASK                               (0x00F80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R4_NU1_SHIFT                              (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R4_NU1_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R4_NU1_MAX                                (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE1_R4_DONT_TURN_OFF_MASK                     (0x01000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R4_DONT_TURN_OFF_SHIFT                    (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R4_DONT_TURN_OFF_RESETVAL                 (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R4_DONT_TURN_OFF_MAX                      (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE1_R4_NU2_MASK                               (0xFE000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R4_NU2_SHIFT                              (0x00000019U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R4_NU2_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R4_NU2_MAX                                (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE1_R4_RESETVAL                               (0x00000000U)

/* PROFILE1_R5 */

#define CSL_BSS_PROFILE_RAM_PROFILE1_R5_ADC_SAMPLING_TIME_MASK                 (0x0007FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R5_ADC_SAMPLING_TIME_SHIFT                (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R5_ADC_SAMPLING_TIME_RESETVAL             (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R5_ADC_SAMPLING_TIME_MAX                  (0x0007FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE1_R5_NU_MASK                                (0xFFF80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R5_NU_SHIFT                               (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R5_NU_RESETVAL                            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R5_NU_MAX                                 (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE1_R5_RESETVAL                               (0x00000000U)

/* PROFILE1_R6 */

#define CSL_BSS_PROFILE_RAM_PROFILE1_R6_TX_START_TIME_MASK                     (0x00001FFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R6_TX_START_TIME_SHIFT                    (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R6_TX_START_TIME_RESETVAL                 (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R6_TX_START_TIME_MAX                      (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE1_R6_NU1_MASK                               (0x0000E000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R6_NU1_SHIFT                              (0x0000000DU)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R6_NU1_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R6_NU1_MAX                                (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE1_R6_ENABLE_SEQ_EXTENSION_MASK              (0x00010000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R6_ENABLE_SEQ_EXTENSION_SHIFT             (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R6_ENABLE_SEQ_EXTENSION_RESETVAL          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R6_ENABLE_SEQ_EXTENSION_MAX               (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE1_R6_NU2_MASK                               (0x00FE0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R6_NU2_SHIFT                              (0x00000011U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R6_NU2_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R6_NU2_MAX                                (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE1_R6_VCO_SELECT_MASK                        (0x01000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R6_VCO_SELECT_SHIFT                       (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R6_VCO_SELECT_RESETVAL                    (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R6_VCO_SELECT_MAX                         (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE1_R6_STITCHING_FIRST_HALF_MASK              (0x02000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R6_STITCHING_FIRST_HALF_SHIFT             (0x00000019U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R6_STITCHING_FIRST_HALF_RESETVAL          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R6_STITCHING_FIRST_HALF_MAX               (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE1_R6_NU3_MASK                               (0xFC000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R6_NU3_SHIFT                              (0x0000001AU)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R6_NU3_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R6_NU3_MAX                                (0x0000003FU)

#define CSL_BSS_PROFILE_RAM_PROFILE1_R6_RESETVAL                               (0x00000000U)

/* PROFILE1_R7 */

#define CSL_BSS_PROFILE_RAM_PROFILE1_R7_MONITOR_START_TIME_MASK                (0x00001FFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R7_MONITOR_START_TIME_SHIFT               (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R7_MONITOR_START_TIME_RESETVAL            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R7_MONITOR_START_TIME_MAX                 (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE1_R7_NU1_MASK                               (0x0000E000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R7_NU1_SHIFT                              (0x0000000DU)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R7_NU1_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R7_NU1_MAX                                (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE1_R7_RX_DFE_RESET_TIME_MASK                 (0x1FFF0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R7_RX_DFE_RESET_TIME_SHIFT                (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R7_RX_DFE_RESET_TIME_RESETVAL             (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R7_RX_DFE_RESET_TIME_MAX                  (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE1_R7_NU2_MASK                               (0xE0000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R7_NU2_SHIFT                              (0x0000001DU)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R7_NU2_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R7_NU2_MAX                                (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE1_R7_RESETVAL                               (0x00000000U)

/* PROFILE1_R8 */

#define CSL_BSS_PROFILE_RAM_PROFILE1_R8_RAMP_END_TIME_MASK                     (0x0007FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R8_RAMP_END_TIME_SHIFT                    (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R8_RAMP_END_TIME_RESETVAL                 (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R8_RAMP_END_TIME_MAX                      (0x0007FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE1_R8_NU_MASK                                (0xFFF80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R8_NU_SHIFT                               (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R8_NU_RESETVAL                            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R8_NU_MAX                                 (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE1_R8_RESETVAL                               (0x00000000U)

/* PROFILE1_R9 */

#define CSL_BSS_PROFILE_RAM_PROFILE1_R9_SYNTH_ANA_CTRL_MASK                    (0x0FFFFFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R9_SYNTH_ANA_CTRL_SHIFT                   (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R9_SYNTH_ANA_CTRL_RESETVAL                (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R9_SYNTH_ANA_CTRL_MAX                     (0x0FFFFFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE1_R9_NU_MASK                                (0xF0000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R9_NU_SHIFT                               (0x0000001CU)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R9_NU_RESETVAL                            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R9_NU_MAX                                 (0x0000000FU)

#define CSL_BSS_PROFILE_RAM_PROFILE1_R9_RESETVAL                               (0x00000000U)

/* PROFILE1_R10 */

#define CSL_BSS_PROFILE_RAM_PROFILE1_R10_FREQ_MON_THRESHOLD_MASK               (0x001FFFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R10_FREQ_MON_THRESHOLD_SHIFT              (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R10_FREQ_MON_THRESHOLD_RESETVAL           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R10_FREQ_MON_THRESHOLD_MAX                (0x001FFFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE1_R10_NU1_MASK                              (0x00E00000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R10_NU1_SHIFT                             (0x00000015U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R10_NU1_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R10_NU1_MAX                               (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE1_R10_ENABLE_LIN_MEAS_MASK                  (0x01000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R10_ENABLE_LIN_MEAS_SHIFT                 (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R10_ENABLE_LIN_MEAS_RESETVAL              (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R10_ENABLE_LIN_MEAS_MAX                   (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE1_R10_NU2_MASK                              (0xFE000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R10_NU2_SHIFT                             (0x00000019U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R10_NU2_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R10_NU2_MAX                               (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE1_R10_RESETVAL                              (0x00000000U)

/* PROFILE1_R11 */

#define CSL_BSS_PROFILE_RAM_PROFILE1_R11_FREQ_MON_L1_MASK                      (0x0000007FU)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R11_FREQ_MON_L1_SHIFT                     (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R11_FREQ_MON_L1_RESETVAL                  (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R11_FREQ_MON_L1_MAX                       (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE1_R11_NU1_MASK                              (0x00000080U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R11_NU1_SHIFT                             (0x00000007U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R11_NU1_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R11_NU1_MAX                               (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE1_R11_FREQ_MON_L2_MASK                      (0x00007F00U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R11_FREQ_MON_L2_SHIFT                     (0x00000008U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R11_FREQ_MON_L2_RESETVAL                  (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R11_FREQ_MON_L2_MAX                       (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE1_R11_NU2_MASK                              (0x00008000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R11_NU2_SHIFT                             (0x0000000FU)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R11_NU2_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R11_NU2_MAX                               (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE1_R11_FREQ_MON_S1_MASK                      (0x00070000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R11_FREQ_MON_S1_SHIFT                     (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R11_FREQ_MON_S1_RESETVAL                  (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R11_FREQ_MON_S1_MAX                       (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE1_R11_NU3_MASK                              (0x00F80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R11_NU3_SHIFT                             (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R11_NU3_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R11_NU3_MAX                               (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE1_R11_FREQ_MON_S2_MASK                      (0x07000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R11_FREQ_MON_S2_SHIFT                     (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R11_FREQ_MON_S2_RESETVAL                  (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R11_FREQ_MON_S2_MAX                       (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE1_R11_NU4_MASK                              (0xF8000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R11_NU4_SHIFT                             (0x0000001BU)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R11_NU4_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R11_NU4_MAX                               (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE1_R11_RESETVAL                              (0x00000000U)

/* PROFILE1_R12 */

#define CSL_BSS_PROFILE_RAM_PROFILE1_R12_HPF_FAST_INIT_START_TIME_MASK         (0x000007FFU)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R12_HPF_FAST_INIT_START_TIME_SHIFT        (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R12_HPF_FAST_INIT_START_TIME_RESETVAL     (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R12_HPF_FAST_INIT_START_TIME_MAX          (0x000007FFU)

#define CSL_BSS_PROFILE_RAM_PROFILE1_R12_HPF_FAST_INIT_PULSE_WIDTH_MASK        (0x0000F800U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R12_HPF_FAST_INIT_PULSE_WIDTH_SHIFT       (0x0000000BU)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R12_HPF_FAST_INIT_PULSE_WIDTH_RESETVAL    (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R12_HPF_FAST_INIT_PULSE_WIDTH_MAX         (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE1_R12_HPF_FAST_INIT_FEATURE_EN_MASK         (0x00010000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R12_HPF_FAST_INIT_FEATURE_EN_SHIFT        (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R12_HPF_FAST_INIT_FEATURE_EN_RESETVAL     (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R12_HPF_FAST_INIT_FEATURE_EN_MAX          (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE1_R12_NU3_MASK                              (0xFFFE0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R12_NU3_SHIFT                             (0x00000011U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R12_NU3_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R12_NU3_MAX                               (0x00007FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE1_R12_RESETVAL                              (0x00000000U)

/* PROFILE1_R13 */

#define CSL_BSS_PROFILE_RAM_PROFILE1_R13_CRD_ENABLE_MASK                       (0x00000001U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R13_CRD_ENABLE_SHIFT                      (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R13_CRD_ENABLE_RESETVAL                   (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R13_CRD_ENABLE_MAX                        (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE1_R13_CRD_DITHER_ENABLE_MASK                (0x00000002U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R13_CRD_DITHER_ENABLE_SHIFT               (0x00000001U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R13_CRD_DITHER_ENABLE_RESETVAL            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R13_CRD_DITHER_ENABLE_MAX                 (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE1_R13_NU_MASK                               (0x00000004U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R13_NU_SHIFT                              (0x00000002U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R13_NU_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R13_NU_MAX                                (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE1_R13_CRD_FREQ_OFFSET_TIME_MASK             (0x0000FFF8U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R13_CRD_FREQ_OFFSET_TIME_SHIFT            (0x00000003U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R13_CRD_FREQ_OFFSET_TIME_RESETVAL         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R13_CRD_FREQ_OFFSET_TIME_MAX              (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE1_R13_CRD_NSLOPE_MAG_MASK                   (0xFFFF0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R13_CRD_NSLOPE_MAG_SHIFT                  (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R13_CRD_NSLOPE_MAG_RESETVAL               (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE1_R13_CRD_NSLOPE_MAG_MAX                    (0x0000FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE1_R13_RESETVAL                              (0x00000000U)

/* PROFILE2_R1 */

#define CSL_BSS_PROFILE_RAM_PROFILE2_R1_N_START_CONSTANT_MASK                  (0x7FFFFFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R1_N_START_CONSTANT_SHIFT                 (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R1_N_START_CONSTANT_RESETVAL              (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R1_N_START_CONSTANT_MAX                   (0x7FFFFFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE2_R1_NU_MASK                                (0x80000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R1_NU_SHIFT                               (0x0000001FU)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R1_NU_RESETVAL                            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R1_NU_MAX                                 (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE2_R1_RESETVAL                               (0x00000000U)

/* PROFILE2_R2 */

#define CSL_BSS_PROFILE_RAM_PROFILE2_R2_N_SLOPE_CONSTANT_MASK                  (0x0000FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R2_N_SLOPE_CONSTANT_SHIFT                 (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R2_N_SLOPE_CONSTANT_RESETVAL              (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R2_N_SLOPE_CONSTANT_MAX                   (0x0000FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE2_R2_FAST_RESET_END_TIME_MASK               (0x1FFF0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R2_FAST_RESET_END_TIME_SHIFT              (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R2_FAST_RESET_END_TIME_RESETVAL           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R2_FAST_RESET_END_TIME_MAX                (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE2_R2_DISABLE_FAST_RESET_MASK                (0x20000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R2_DISABLE_FAST_RESET_SHIFT               (0x0000001DU)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R2_DISABLE_FAST_RESET_RESETVAL            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R2_DISABLE_FAST_RESET_MAX                 (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE2_R2_NU_MASK                                (0xC0000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R2_NU_SHIFT                               (0x0000001EU)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R2_NU_RESETVAL                            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R2_NU_MAX                                 (0x00000003U)

#define CSL_BSS_PROFILE_RAM_PROFILE2_R2_RESETVAL                               (0x00000000U)

/* PROFILE2_R3 */

#define CSL_BSS_PROFILE_RAM_PROFILE2_R3_IDLE_TIME_CONSTANT_MASK                (0x0007FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R3_IDLE_TIME_CONSTANT_SHIFT               (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R3_IDLE_TIME_CONSTANT_RESETVAL            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R3_IDLE_TIME_CONSTANT_MAX                 (0x0007FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE2_R3_NU1_MASK                               (0x00F80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R3_NU1_SHIFT                              (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R3_NU1_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R3_NU1_MAX                                (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE2_R3_DISABLE_MONITORING_MASK                (0x01000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R3_DISABLE_MONITORING_SHIFT               (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R3_DISABLE_MONITORING_RESETVAL            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R3_DISABLE_MONITORING_MAX                 (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE2_R3_NU2_MASK                               (0xFE000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R3_NU2_SHIFT                              (0x00000019U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R3_NU2_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R3_NU2_MAX                                (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE2_R3_RESETVAL                               (0x00000000U)

/* PROFILE2_R4 */

#define CSL_BSS_PROFILE_RAM_PROFILE2_R4_ADC_VALID_START_TIME_MASK              (0x0007FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R4_ADC_VALID_START_TIME_SHIFT             (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R4_ADC_VALID_START_TIME_RESETVAL          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R4_ADC_VALID_START_TIME_MAX               (0x0007FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE2_R4_NU1_MASK                               (0x00F80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R4_NU1_SHIFT                              (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R4_NU1_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R4_NU1_MAX                                (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE2_R4_DONT_TURN_OFF_MASK                     (0x01000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R4_DONT_TURN_OFF_SHIFT                    (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R4_DONT_TURN_OFF_RESETVAL                 (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R4_DONT_TURN_OFF_MAX                      (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE2_R4_NU2_MASK                               (0xFE000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R4_NU2_SHIFT                              (0x00000019U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R4_NU2_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R4_NU2_MAX                                (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE2_R4_RESETVAL                               (0x00000000U)

/* PROFILE2_R5 */

#define CSL_BSS_PROFILE_RAM_PROFILE2_R5_ADC_SAMPLING_TIME_MASK                 (0x0007FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R5_ADC_SAMPLING_TIME_SHIFT                (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R5_ADC_SAMPLING_TIME_RESETVAL             (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R5_ADC_SAMPLING_TIME_MAX                  (0x0007FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE2_R5_NU_MASK                                (0xFFF80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R5_NU_SHIFT                               (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R5_NU_RESETVAL                            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R5_NU_MAX                                 (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE2_R5_RESETVAL                               (0x00000000U)

/* PROFILE2_R6 */

#define CSL_BSS_PROFILE_RAM_PROFILE2_R6_TX_START_TIME_MASK                     (0x00001FFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R6_TX_START_TIME_SHIFT                    (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R6_TX_START_TIME_RESETVAL                 (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R6_TX_START_TIME_MAX                      (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE2_R6_NU1_MASK                               (0x0000E000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R6_NU1_SHIFT                              (0x0000000DU)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R6_NU1_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R6_NU1_MAX                                (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE2_R6_ENABLE_SEQ_EXTENSION_MASK              (0x00010000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R6_ENABLE_SEQ_EXTENSION_SHIFT             (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R6_ENABLE_SEQ_EXTENSION_RESETVAL          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R6_ENABLE_SEQ_EXTENSION_MAX               (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE2_R6_NU2_MASK                               (0x00FE0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R6_NU2_SHIFT                              (0x00000011U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R6_NU2_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R6_NU2_MAX                                (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE2_R6_VCO_SELECT_MASK                        (0x01000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R6_VCO_SELECT_SHIFT                       (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R6_VCO_SELECT_RESETVAL                    (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R6_VCO_SELECT_MAX                         (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE2_R6_STITCHING_FIRST_HALF_MASK              (0x02000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R6_STITCHING_FIRST_HALF_SHIFT             (0x00000019U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R6_STITCHING_FIRST_HALF_RESETVAL          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R6_STITCHING_FIRST_HALF_MAX               (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE2_R6_NU3_MASK                               (0xFC000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R6_NU3_SHIFT                              (0x0000001AU)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R6_NU3_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R6_NU3_MAX                                (0x0000003FU)

#define CSL_BSS_PROFILE_RAM_PROFILE2_R6_RESETVAL                               (0x00000000U)

/* PROFILE2_R7 */

#define CSL_BSS_PROFILE_RAM_PROFILE2_R7_MONITOR_START_TIME_MASK                (0x00001FFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R7_MONITOR_START_TIME_SHIFT               (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R7_MONITOR_START_TIME_RESETVAL            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R7_MONITOR_START_TIME_MAX                 (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE2_R7_NU1_MASK                               (0x0000E000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R7_NU1_SHIFT                              (0x0000000DU)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R7_NU1_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R7_NU1_MAX                                (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE2_R7_RX_DFE_RESET_TIME_MASK                 (0x1FFF0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R7_RX_DFE_RESET_TIME_SHIFT                (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R7_RX_DFE_RESET_TIME_RESETVAL             (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R7_RX_DFE_RESET_TIME_MAX                  (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE2_R7_NU2_MASK                               (0xE0000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R7_NU2_SHIFT                              (0x0000001DU)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R7_NU2_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R7_NU2_MAX                                (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE2_R7_RESETVAL                               (0x00000000U)

/* PROFILE2_R8 */

#define CSL_BSS_PROFILE_RAM_PROFILE2_R8_RAMP_END_TIME_MASK                     (0x0007FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R8_RAMP_END_TIME_SHIFT                    (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R8_RAMP_END_TIME_RESETVAL                 (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R8_RAMP_END_TIME_MAX                      (0x0007FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE2_R8_NU_MASK                                (0xFFF80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R8_NU_SHIFT                               (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R8_NU_RESETVAL                            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R8_NU_MAX                                 (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE2_R8_RESETVAL                               (0x00000000U)

/* PROFILE2_R9 */

#define CSL_BSS_PROFILE_RAM_PROFILE2_R9_SYNTH_ANA_CTRL_MASK                    (0x0FFFFFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R9_SYNTH_ANA_CTRL_SHIFT                   (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R9_SYNTH_ANA_CTRL_RESETVAL                (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R9_SYNTH_ANA_CTRL_MAX                     (0x0FFFFFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE2_R9_NU_MASK                                (0xF0000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R9_NU_SHIFT                               (0x0000001CU)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R9_NU_RESETVAL                            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R9_NU_MAX                                 (0x0000000FU)

#define CSL_BSS_PROFILE_RAM_PROFILE2_R9_RESETVAL                               (0x00000000U)

/* PROFILE2_R10 */

#define CSL_BSS_PROFILE_RAM_PROFILE2_R10_FREQ_MON_THRESHOLD_MASK               (0x001FFFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R10_FREQ_MON_THRESHOLD_SHIFT              (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R10_FREQ_MON_THRESHOLD_RESETVAL           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R10_FREQ_MON_THRESHOLD_MAX                (0x001FFFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE2_R10_NU1_MASK                              (0x00E00000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R10_NU1_SHIFT                             (0x00000015U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R10_NU1_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R10_NU1_MAX                               (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE2_R10_ENABLE_LIN_MEAS_MASK                  (0x01000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R10_ENABLE_LIN_MEAS_SHIFT                 (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R10_ENABLE_LIN_MEAS_RESETVAL              (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R10_ENABLE_LIN_MEAS_MAX                   (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE2_R10_NU2_MASK                              (0xFE000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R10_NU2_SHIFT                             (0x00000019U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R10_NU2_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R10_NU2_MAX                               (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE2_R10_RESETVAL                              (0x00000000U)

/* PROFILE2_R11 */

#define CSL_BSS_PROFILE_RAM_PROFILE2_R11_FREQ_MON_L1_MASK                      (0x0000007FU)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R11_FREQ_MON_L1_SHIFT                     (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R11_FREQ_MON_L1_RESETVAL                  (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R11_FREQ_MON_L1_MAX                       (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE2_R11_NU1_MASK                              (0x00000080U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R11_NU1_SHIFT                             (0x00000007U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R11_NU1_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R11_NU1_MAX                               (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE2_R11_FREQ_MON_L2_MASK                      (0x00007F00U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R11_FREQ_MON_L2_SHIFT                     (0x00000008U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R11_FREQ_MON_L2_RESETVAL                  (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R11_FREQ_MON_L2_MAX                       (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE2_R11_NU2_MASK                              (0x00008000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R11_NU2_SHIFT                             (0x0000000FU)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R11_NU2_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R11_NU2_MAX                               (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE2_R11_FREQ_MON_S1_MASK                      (0x00070000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R11_FREQ_MON_S1_SHIFT                     (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R11_FREQ_MON_S1_RESETVAL                  (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R11_FREQ_MON_S1_MAX                       (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE2_R11_NU3_MASK                              (0x00F80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R11_NU3_SHIFT                             (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R11_NU3_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R11_NU3_MAX                               (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE2_R11_FREQ_MON_S2_MASK                      (0x07000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R11_FREQ_MON_S2_SHIFT                     (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R11_FREQ_MON_S2_RESETVAL                  (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R11_FREQ_MON_S2_MAX                       (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE2_R11_NU4_MASK                              (0xF8000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R11_NU4_SHIFT                             (0x0000001BU)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R11_NU4_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R11_NU4_MAX                               (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE2_R11_RESETVAL                              (0x00000000U)

/* PROFILE2_R12 */

#define CSL_BSS_PROFILE_RAM_PROFILE2_R12_HPF_FAST_INIT_START_TIME_MASK         (0x000007FFU)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R12_HPF_FAST_INIT_START_TIME_SHIFT        (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R12_HPF_FAST_INIT_START_TIME_RESETVAL     (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R12_HPF_FAST_INIT_START_TIME_MAX          (0x000007FFU)

#define CSL_BSS_PROFILE_RAM_PROFILE2_R12_HPF_FAST_INIT_PULSE_WIDTH_MASK        (0x0000F800U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R12_HPF_FAST_INIT_PULSE_WIDTH_SHIFT       (0x0000000BU)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R12_HPF_FAST_INIT_PULSE_WIDTH_RESETVAL    (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R12_HPF_FAST_INIT_PULSE_WIDTH_MAX         (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE2_R12_HPF_FAST_INIT_FEATURE_EN_MASK         (0x00010000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R12_HPF_FAST_INIT_FEATURE_EN_SHIFT        (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R12_HPF_FAST_INIT_FEATURE_EN_RESETVAL     (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R12_HPF_FAST_INIT_FEATURE_EN_MAX          (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE2_R12_NU3_MASK                              (0xFFFE0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R12_NU3_SHIFT                             (0x00000011U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R12_NU3_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R12_NU3_MAX                               (0x00007FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE2_R12_RESETVAL                              (0x00000000U)

/* PROFILE2_R13 */

#define CSL_BSS_PROFILE_RAM_PROFILE2_R13_CRD_ENABLE_MASK                       (0x00000001U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R13_CRD_ENABLE_SHIFT                      (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R13_CRD_ENABLE_RESETVAL                   (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R13_CRD_ENABLE_MAX                        (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE2_R13_CRD_DITHER_ENABLE_MASK                (0x00000002U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R13_CRD_DITHER_ENABLE_SHIFT               (0x00000001U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R13_CRD_DITHER_ENABLE_RESETVAL            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R13_CRD_DITHER_ENABLE_MAX                 (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE2_R13_NU_MASK                               (0x00000004U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R13_NU_SHIFT                              (0x00000002U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R13_NU_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R13_NU_MAX                                (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE2_R13_CRD_FREQ_OFFSET_TIME_MASK             (0x0000FFF8U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R13_CRD_FREQ_OFFSET_TIME_SHIFT            (0x00000003U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R13_CRD_FREQ_OFFSET_TIME_RESETVAL         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R13_CRD_FREQ_OFFSET_TIME_MAX              (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE2_R13_CRD_NSLOPE_MAG_MASK                   (0xFFFF0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R13_CRD_NSLOPE_MAG_SHIFT                  (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R13_CRD_NSLOPE_MAG_RESETVAL               (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE2_R13_CRD_NSLOPE_MAG_MAX                    (0x0000FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE2_R13_RESETVAL                              (0x00000000U)

/* PROFILE3_R1 */

#define CSL_BSS_PROFILE_RAM_PROFILE3_R1_N_START_CONSTANT_MASK                  (0x7FFFFFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R1_N_START_CONSTANT_SHIFT                 (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R1_N_START_CONSTANT_RESETVAL              (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R1_N_START_CONSTANT_MAX                   (0x7FFFFFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE3_R1_NU_MASK                                (0x80000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R1_NU_SHIFT                               (0x0000001FU)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R1_NU_RESETVAL                            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R1_NU_MAX                                 (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE3_R1_RESETVAL                               (0x00000000U)

/* PROFILE3_R2 */

#define CSL_BSS_PROFILE_RAM_PROFILE3_R2_N_SLOPE_CONSTANT_MASK                  (0x0000FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R2_N_SLOPE_CONSTANT_SHIFT                 (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R2_N_SLOPE_CONSTANT_RESETVAL              (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R2_N_SLOPE_CONSTANT_MAX                   (0x0000FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE3_R2_FAST_RESET_END_TIME_MASK               (0x1FFF0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R2_FAST_RESET_END_TIME_SHIFT              (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R2_FAST_RESET_END_TIME_RESETVAL           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R2_FAST_RESET_END_TIME_MAX                (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE3_R2_DISABLE_FAST_RESET_MASK                (0x20000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R2_DISABLE_FAST_RESET_SHIFT               (0x0000001DU)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R2_DISABLE_FAST_RESET_RESETVAL            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R2_DISABLE_FAST_RESET_MAX                 (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE3_R2_NU_MASK                                (0xC0000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R2_NU_SHIFT                               (0x0000001EU)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R2_NU_RESETVAL                            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R2_NU_MAX                                 (0x00000003U)

#define CSL_BSS_PROFILE_RAM_PROFILE3_R2_RESETVAL                               (0x00000000U)

/* PROFILE3_R3 */

#define CSL_BSS_PROFILE_RAM_PROFILE3_R3_IDLE_TIME_CONSTANT_MASK                (0x0007FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R3_IDLE_TIME_CONSTANT_SHIFT               (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R3_IDLE_TIME_CONSTANT_RESETVAL            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R3_IDLE_TIME_CONSTANT_MAX                 (0x0007FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE3_R3_NU1_MASK                               (0x00F80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R3_NU1_SHIFT                              (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R3_NU1_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R3_NU1_MAX                                (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE3_R3_DISABLE_MONITORING_MASK                (0x01000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R3_DISABLE_MONITORING_SHIFT               (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R3_DISABLE_MONITORING_RESETVAL            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R3_DISABLE_MONITORING_MAX                 (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE3_R3_NU2_MASK                               (0xFE000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R3_NU2_SHIFT                              (0x00000019U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R3_NU2_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R3_NU2_MAX                                (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE3_R3_RESETVAL                               (0x00000000U)

/* PROFILE3_R4 */

#define CSL_BSS_PROFILE_RAM_PROFILE3_R4_ADC_VALID_START_TIME_MASK              (0x0007FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R4_ADC_VALID_START_TIME_SHIFT             (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R4_ADC_VALID_START_TIME_RESETVAL          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R4_ADC_VALID_START_TIME_MAX               (0x0007FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE3_R4_NU1_MASK                               (0x00F80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R4_NU1_SHIFT                              (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R4_NU1_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R4_NU1_MAX                                (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE3_R4_DONT_TURN_OFF_MASK                     (0x01000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R4_DONT_TURN_OFF_SHIFT                    (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R4_DONT_TURN_OFF_RESETVAL                 (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R4_DONT_TURN_OFF_MAX                      (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE3_R4_NU2_MASK                               (0xFE000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R4_NU2_SHIFT                              (0x00000019U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R4_NU2_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R4_NU2_MAX                                (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE3_R4_RESETVAL                               (0x00000000U)

/* PROFILE3_R5 */

#define CSL_BSS_PROFILE_RAM_PROFILE3_R5_ADC_SAMPLING_TIME_MASK                 (0x0007FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R5_ADC_SAMPLING_TIME_SHIFT                (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R5_ADC_SAMPLING_TIME_RESETVAL             (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R5_ADC_SAMPLING_TIME_MAX                  (0x0007FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE3_R5_NU_MASK                                (0xFFF80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R5_NU_SHIFT                               (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R5_NU_RESETVAL                            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R5_NU_MAX                                 (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE3_R5_RESETVAL                               (0x00000000U)

/* PROFILE3_R6 */

#define CSL_BSS_PROFILE_RAM_PROFILE3_R6_TX_START_TIME_MASK                     (0x00001FFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R6_TX_START_TIME_SHIFT                    (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R6_TX_START_TIME_RESETVAL                 (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R6_TX_START_TIME_MAX                      (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE3_R6_NU1_MASK                               (0x0000E000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R6_NU1_SHIFT                              (0x0000000DU)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R6_NU1_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R6_NU1_MAX                                (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE3_R6_ENABLE_SEQ_EXTENSION_MASK              (0x00010000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R6_ENABLE_SEQ_EXTENSION_SHIFT             (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R6_ENABLE_SEQ_EXTENSION_RESETVAL          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R6_ENABLE_SEQ_EXTENSION_MAX               (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE3_R6_NU2_MASK                               (0x00FE0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R6_NU2_SHIFT                              (0x00000011U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R6_NU2_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R6_NU2_MAX                                (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE3_R6_VCO_SELECT_MASK                        (0x01000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R6_VCO_SELECT_SHIFT                       (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R6_VCO_SELECT_RESETVAL                    (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R6_VCO_SELECT_MAX                         (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE3_R6_STITCHING_FIRST_HALF_MASK              (0x02000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R6_STITCHING_FIRST_HALF_SHIFT             (0x00000019U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R6_STITCHING_FIRST_HALF_RESETVAL          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R6_STITCHING_FIRST_HALF_MAX               (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE3_R6_NU3_MASK                               (0xFC000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R6_NU3_SHIFT                              (0x0000001AU)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R6_NU3_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R6_NU3_MAX                                (0x0000003FU)

#define CSL_BSS_PROFILE_RAM_PROFILE3_R6_RESETVAL                               (0x00000000U)

/* PROFILE3_R7 */

#define CSL_BSS_PROFILE_RAM_PROFILE3_R7_MONITOR_START_TIME_MASK                (0x00001FFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R7_MONITOR_START_TIME_SHIFT               (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R7_MONITOR_START_TIME_RESETVAL            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R7_MONITOR_START_TIME_MAX                 (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE3_R7_NU1_MASK                               (0x0000E000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R7_NU1_SHIFT                              (0x0000000DU)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R7_NU1_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R7_NU1_MAX                                (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE3_R7_RX_DFE_RESET_TIME_MASK                 (0x1FFF0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R7_RX_DFE_RESET_TIME_SHIFT                (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R7_RX_DFE_RESET_TIME_RESETVAL             (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R7_RX_DFE_RESET_TIME_MAX                  (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE3_R7_NU2_MASK                               (0xE0000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R7_NU2_SHIFT                              (0x0000001DU)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R7_NU2_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R7_NU2_MAX                                (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE3_R7_RESETVAL                               (0x00000000U)

/* PROFILE3_R8 */

#define CSL_BSS_PROFILE_RAM_PROFILE3_R8_RAMP_END_TIME_MASK                     (0x0007FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R8_RAMP_END_TIME_SHIFT                    (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R8_RAMP_END_TIME_RESETVAL                 (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R8_RAMP_END_TIME_MAX                      (0x0007FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE3_R8_NU_MASK                                (0xFFF80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R8_NU_SHIFT                               (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R8_NU_RESETVAL                            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R8_NU_MAX                                 (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE3_R8_RESETVAL                               (0x00000000U)

/* PROFILE3_R9 */

#define CSL_BSS_PROFILE_RAM_PROFILE3_R9_SYNTH_ANA_CTRL_MASK                    (0x0FFFFFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R9_SYNTH_ANA_CTRL_SHIFT                   (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R9_SYNTH_ANA_CTRL_RESETVAL                (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R9_SYNTH_ANA_CTRL_MAX                     (0x0FFFFFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE3_R9_NU_MASK                                (0xF0000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R9_NU_SHIFT                               (0x0000001CU)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R9_NU_RESETVAL                            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R9_NU_MAX                                 (0x0000000FU)

#define CSL_BSS_PROFILE_RAM_PROFILE3_R9_RESETVAL                               (0x00000000U)

/* PROFILE3_R10 */

#define CSL_BSS_PROFILE_RAM_PROFILE3_R10_FREQ_MON_THRESHOLD_MASK               (0x001FFFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R10_FREQ_MON_THRESHOLD_SHIFT              (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R10_FREQ_MON_THRESHOLD_RESETVAL           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R10_FREQ_MON_THRESHOLD_MAX                (0x001FFFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE3_R10_NU1_MASK                              (0x00E00000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R10_NU1_SHIFT                             (0x00000015U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R10_NU1_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R10_NU1_MAX                               (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE3_R10_ENABLE_LIN_MEAS_MASK                  (0x01000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R10_ENABLE_LIN_MEAS_SHIFT                 (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R10_ENABLE_LIN_MEAS_RESETVAL              (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R10_ENABLE_LIN_MEAS_MAX                   (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE3_R10_NU2_MASK                              (0xFE000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R10_NU2_SHIFT                             (0x00000019U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R10_NU2_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R10_NU2_MAX                               (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE3_R10_RESETVAL                              (0x00000000U)

/* PROFILE3_R11 */

#define CSL_BSS_PROFILE_RAM_PROFILE3_R11_FREQ_MON_L1_MASK                      (0x0000007FU)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R11_FREQ_MON_L1_SHIFT                     (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R11_FREQ_MON_L1_RESETVAL                  (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R11_FREQ_MON_L1_MAX                       (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE3_R11_NU1_MASK                              (0x00000080U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R11_NU1_SHIFT                             (0x00000007U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R11_NU1_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R11_NU1_MAX                               (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE3_R11_FREQ_MON_L2_MASK                      (0x00007F00U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R11_FREQ_MON_L2_SHIFT                     (0x00000008U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R11_FREQ_MON_L2_RESETVAL                  (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R11_FREQ_MON_L2_MAX                       (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE3_R11_NU2_MASK                              (0x00008000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R11_NU2_SHIFT                             (0x0000000FU)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R11_NU2_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R11_NU2_MAX                               (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE3_R11_FREQ_MON_S1_MASK                      (0x00070000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R11_FREQ_MON_S1_SHIFT                     (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R11_FREQ_MON_S1_RESETVAL                  (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R11_FREQ_MON_S1_MAX                       (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE3_R11_NU3_MASK                              (0x00F80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R11_NU3_SHIFT                             (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R11_NU3_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R11_NU3_MAX                               (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE3_R11_FREQ_MON_S2_MASK                      (0x07000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R11_FREQ_MON_S2_SHIFT                     (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R11_FREQ_MON_S2_RESETVAL                  (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R11_FREQ_MON_S2_MAX                       (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE3_R11_NU4_MASK                              (0xF8000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R11_NU4_SHIFT                             (0x0000001BU)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R11_NU4_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R11_NU4_MAX                               (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE3_R11_RESETVAL                              (0x00000000U)

/* PROFILE3_R12 */

#define CSL_BSS_PROFILE_RAM_PROFILE3_R12_HPF_FAST_INIT_START_TIME_MASK         (0x000007FFU)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R12_HPF_FAST_INIT_START_TIME_SHIFT        (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R12_HPF_FAST_INIT_START_TIME_RESETVAL     (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R12_HPF_FAST_INIT_START_TIME_MAX          (0x000007FFU)

#define CSL_BSS_PROFILE_RAM_PROFILE3_R12_HPF_FAST_INIT_PULSE_WIDTH_MASK        (0x0000F800U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R12_HPF_FAST_INIT_PULSE_WIDTH_SHIFT       (0x0000000BU)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R12_HPF_FAST_INIT_PULSE_WIDTH_RESETVAL    (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R12_HPF_FAST_INIT_PULSE_WIDTH_MAX         (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE3_R12_HPF_FAST_INIT_FEATURE_EN_MASK         (0x00010000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R12_HPF_FAST_INIT_FEATURE_EN_SHIFT        (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R12_HPF_FAST_INIT_FEATURE_EN_RESETVAL     (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R12_HPF_FAST_INIT_FEATURE_EN_MAX          (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE3_R12_NU3_MASK                              (0xFFFE0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R12_NU3_SHIFT                             (0x00000011U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R12_NU3_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R12_NU3_MAX                               (0x00007FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE3_R12_RESETVAL                              (0x00000000U)

/* PROFILE3_R13 */

#define CSL_BSS_PROFILE_RAM_PROFILE3_R13_CRD_ENABLE_MASK                       (0x00000001U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R13_CRD_ENABLE_SHIFT                      (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R13_CRD_ENABLE_RESETVAL                   (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R13_CRD_ENABLE_MAX                        (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE3_R13_CRD_DITHER_ENABLE_MASK                (0x00000002U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R13_CRD_DITHER_ENABLE_SHIFT               (0x00000001U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R13_CRD_DITHER_ENABLE_RESETVAL            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R13_CRD_DITHER_ENABLE_MAX                 (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE3_R13_NU_MASK                               (0x00000004U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R13_NU_SHIFT                              (0x00000002U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R13_NU_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R13_NU_MAX                                (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE3_R13_CRD_FREQ_OFFSET_TIME_MASK             (0x0000FFF8U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R13_CRD_FREQ_OFFSET_TIME_SHIFT            (0x00000003U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R13_CRD_FREQ_OFFSET_TIME_RESETVAL         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R13_CRD_FREQ_OFFSET_TIME_MAX              (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE3_R13_CRD_NSLOPE_MAG_MASK                   (0xFFFF0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R13_CRD_NSLOPE_MAG_SHIFT                  (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R13_CRD_NSLOPE_MAG_RESETVAL               (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE3_R13_CRD_NSLOPE_MAG_MAX                    (0x0000FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE3_R13_RESETVAL                              (0x00000000U)

/* PROFILE4_R1 */

#define CSL_BSS_PROFILE_RAM_PROFILE4_R1_N_START_CONSTANT_MASK                  (0x7FFFFFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R1_N_START_CONSTANT_SHIFT                 (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R1_N_START_CONSTANT_RESETVAL              (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R1_N_START_CONSTANT_MAX                   (0x7FFFFFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE4_R1_NU_MASK                                (0x80000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R1_NU_SHIFT                               (0x0000001FU)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R1_NU_RESETVAL                            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R1_NU_MAX                                 (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE4_R1_RESETVAL                               (0x00000000U)

/* PROFILE4_R2 */

#define CSL_BSS_PROFILE_RAM_PROFILE4_R2_N_SLOPE_CONSTANT_MASK                  (0x0000FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R2_N_SLOPE_CONSTANT_SHIFT                 (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R2_N_SLOPE_CONSTANT_RESETVAL              (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R2_N_SLOPE_CONSTANT_MAX                   (0x0000FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE4_R2_FAST_RESET_END_TIME_MASK               (0x1FFF0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R2_FAST_RESET_END_TIME_SHIFT              (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R2_FAST_RESET_END_TIME_RESETVAL           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R2_FAST_RESET_END_TIME_MAX                (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE4_R2_DISABLE_FAST_RESET_MASK                (0x20000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R2_DISABLE_FAST_RESET_SHIFT               (0x0000001DU)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R2_DISABLE_FAST_RESET_RESETVAL            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R2_DISABLE_FAST_RESET_MAX                 (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE4_R2_NU_MASK                                (0xC0000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R2_NU_SHIFT                               (0x0000001EU)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R2_NU_RESETVAL                            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R2_NU_MAX                                 (0x00000003U)

#define CSL_BSS_PROFILE_RAM_PROFILE4_R2_RESETVAL                               (0x00000000U)

/* PROFILE4_R3 */

#define CSL_BSS_PROFILE_RAM_PROFILE4_R3_IDLE_TIME_CONSTANT_MASK                (0x0007FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R3_IDLE_TIME_CONSTANT_SHIFT               (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R3_IDLE_TIME_CONSTANT_RESETVAL            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R3_IDLE_TIME_CONSTANT_MAX                 (0x0007FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE4_R3_NU1_MASK                               (0x00F80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R3_NU1_SHIFT                              (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R3_NU1_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R3_NU1_MAX                                (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE4_R3_DISABLE_MONITORING_MASK                (0x01000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R3_DISABLE_MONITORING_SHIFT               (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R3_DISABLE_MONITORING_RESETVAL            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R3_DISABLE_MONITORING_MAX                 (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE4_R3_NU2_MASK                               (0xFE000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R3_NU2_SHIFT                              (0x00000019U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R3_NU2_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R3_NU2_MAX                                (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE4_R3_RESETVAL                               (0x00000000U)

/* PROFILE4_R4 */

#define CSL_BSS_PROFILE_RAM_PROFILE4_R4_ADC_VALID_START_TIME_MASK              (0x0007FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R4_ADC_VALID_START_TIME_SHIFT             (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R4_ADC_VALID_START_TIME_RESETVAL          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R4_ADC_VALID_START_TIME_MAX               (0x0007FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE4_R4_NU1_MASK                               (0x00F80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R4_NU1_SHIFT                              (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R4_NU1_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R4_NU1_MAX                                (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE4_R4_DONT_TURN_OFF_MASK                     (0x01000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R4_DONT_TURN_OFF_SHIFT                    (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R4_DONT_TURN_OFF_RESETVAL                 (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R4_DONT_TURN_OFF_MAX                      (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE4_R4_NU2_MASK                               (0xFE000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R4_NU2_SHIFT                              (0x00000019U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R4_NU2_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R4_NU2_MAX                                (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE4_R4_RESETVAL                               (0x00000000U)

/* PROFILE4_R5 */

#define CSL_BSS_PROFILE_RAM_PROFILE4_R5_ADC_SAMPLING_TIME_MASK                 (0x0007FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R5_ADC_SAMPLING_TIME_SHIFT                (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R5_ADC_SAMPLING_TIME_RESETVAL             (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R5_ADC_SAMPLING_TIME_MAX                  (0x0007FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE4_R5_NU_MASK                                (0xFFF80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R5_NU_SHIFT                               (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R5_NU_RESETVAL                            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R5_NU_MAX                                 (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE4_R5_RESETVAL                               (0x00000000U)

/* PROFILE4_R6 */

#define CSL_BSS_PROFILE_RAM_PROFILE4_R6_TX_START_TIME_MASK                     (0x00001FFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R6_TX_START_TIME_SHIFT                    (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R6_TX_START_TIME_RESETVAL                 (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R6_TX_START_TIME_MAX                      (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE4_R6_NU1_MASK                               (0x0000E000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R6_NU1_SHIFT                              (0x0000000DU)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R6_NU1_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R6_NU1_MAX                                (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE4_R6_ENABLE_SEQ_EXTENSION_MASK              (0x00010000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R6_ENABLE_SEQ_EXTENSION_SHIFT             (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R6_ENABLE_SEQ_EXTENSION_RESETVAL          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R6_ENABLE_SEQ_EXTENSION_MAX               (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE4_R6_NU2_MASK                               (0x00FE0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R6_NU2_SHIFT                              (0x00000011U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R6_NU2_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R6_NU2_MAX                                (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE4_R6_VCO_SELECT_MASK                        (0x01000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R6_VCO_SELECT_SHIFT                       (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R6_VCO_SELECT_RESETVAL                    (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R6_VCO_SELECT_MAX                         (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE4_R6_STITCHING_FIRST_HALF_MASK              (0x02000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R6_STITCHING_FIRST_HALF_SHIFT             (0x00000019U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R6_STITCHING_FIRST_HALF_RESETVAL          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R6_STITCHING_FIRST_HALF_MAX               (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE4_R6_NU3_MASK                               (0xFC000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R6_NU3_SHIFT                              (0x0000001AU)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R6_NU3_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R6_NU3_MAX                                (0x0000003FU)

#define CSL_BSS_PROFILE_RAM_PROFILE4_R6_RESETVAL                               (0x00000000U)

/* PROFILE4_R7 */

#define CSL_BSS_PROFILE_RAM_PROFILE4_R7_MONITOR_START_TIME_MASK                (0x00001FFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R7_MONITOR_START_TIME_SHIFT               (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R7_MONITOR_START_TIME_RESETVAL            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R7_MONITOR_START_TIME_MAX                 (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE4_R7_NU1_MASK                               (0x0000E000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R7_NU1_SHIFT                              (0x0000000DU)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R7_NU1_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R7_NU1_MAX                                (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE4_R7_RX_DFE_RESET_TIME_MASK                 (0x1FFF0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R7_RX_DFE_RESET_TIME_SHIFT                (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R7_RX_DFE_RESET_TIME_RESETVAL             (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R7_RX_DFE_RESET_TIME_MAX                  (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE4_R7_NU2_MASK                               (0xE0000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R7_NU2_SHIFT                              (0x0000001DU)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R7_NU2_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R7_NU2_MAX                                (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE4_R7_RESETVAL                               (0x00000000U)

/* PROFILE4_R8 */

#define CSL_BSS_PROFILE_RAM_PROFILE4_R8_RAMP_END_TIME_MASK                     (0x0007FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R8_RAMP_END_TIME_SHIFT                    (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R8_RAMP_END_TIME_RESETVAL                 (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R8_RAMP_END_TIME_MAX                      (0x0007FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE4_R8_NU_MASK                                (0xFFF80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R8_NU_SHIFT                               (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R8_NU_RESETVAL                            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R8_NU_MAX                                 (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE4_R8_RESETVAL                               (0x00000000U)

/* PROFILE4_R9 */

#define CSL_BSS_PROFILE_RAM_PROFILE4_R9_SYNTH_ANA_CTRL_MASK                    (0x0FFFFFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R9_SYNTH_ANA_CTRL_SHIFT                   (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R9_SYNTH_ANA_CTRL_RESETVAL                (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R9_SYNTH_ANA_CTRL_MAX                     (0x0FFFFFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE4_R9_NU_MASK                                (0xF0000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R9_NU_SHIFT                               (0x0000001CU)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R9_NU_RESETVAL                            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R9_NU_MAX                                 (0x0000000FU)

#define CSL_BSS_PROFILE_RAM_PROFILE4_R9_RESETVAL                               (0x00000000U)

/* PROFILE4_R10 */

#define CSL_BSS_PROFILE_RAM_PROFILE4_R10_FREQ_MON_THRESHOLD_MASK               (0x001FFFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R10_FREQ_MON_THRESHOLD_SHIFT              (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R10_FREQ_MON_THRESHOLD_RESETVAL           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R10_FREQ_MON_THRESHOLD_MAX                (0x001FFFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE4_R10_NU1_MASK                              (0x00E00000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R10_NU1_SHIFT                             (0x00000015U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R10_NU1_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R10_NU1_MAX                               (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE4_R10_ENABLE_LIN_MEAS_MASK                  (0x01000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R10_ENABLE_LIN_MEAS_SHIFT                 (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R10_ENABLE_LIN_MEAS_RESETVAL              (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R10_ENABLE_LIN_MEAS_MAX                   (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE4_R10_NU2_MASK                              (0xFE000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R10_NU2_SHIFT                             (0x00000019U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R10_NU2_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R10_NU2_MAX                               (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE4_R10_RESETVAL                              (0x00000000U)

/* PROFILE4_R11 */

#define CSL_BSS_PROFILE_RAM_PROFILE4_R11_FREQ_MON_L1_MASK                      (0x0000007FU)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R11_FREQ_MON_L1_SHIFT                     (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R11_FREQ_MON_L1_RESETVAL                  (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R11_FREQ_MON_L1_MAX                       (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE4_R11_NU1_MASK                              (0x00000080U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R11_NU1_SHIFT                             (0x00000007U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R11_NU1_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R11_NU1_MAX                               (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE4_R11_FREQ_MON_L2_MASK                      (0x00007F00U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R11_FREQ_MON_L2_SHIFT                     (0x00000008U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R11_FREQ_MON_L2_RESETVAL                  (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R11_FREQ_MON_L2_MAX                       (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE4_R11_NU2_MASK                              (0x00008000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R11_NU2_SHIFT                             (0x0000000FU)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R11_NU2_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R11_NU2_MAX                               (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE4_R11_FREQ_MON_S1_MASK                      (0x00070000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R11_FREQ_MON_S1_SHIFT                     (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R11_FREQ_MON_S1_RESETVAL                  (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R11_FREQ_MON_S1_MAX                       (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE4_R11_NU3_MASK                              (0x00F80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R11_NU3_SHIFT                             (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R11_NU3_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R11_NU3_MAX                               (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE4_R11_FREQ_MON_S2_MASK                      (0x07000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R11_FREQ_MON_S2_SHIFT                     (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R11_FREQ_MON_S2_RESETVAL                  (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R11_FREQ_MON_S2_MAX                       (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE4_R11_NU4_MASK                              (0xF8000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R11_NU4_SHIFT                             (0x0000001BU)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R11_NU4_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R11_NU4_MAX                               (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE4_R11_RESETVAL                              (0x00000000U)

/* PROFILE4_R12 */

#define CSL_BSS_PROFILE_RAM_PROFILE4_R12_HPF_FAST_INIT_START_TIME_MASK         (0x000007FFU)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R12_HPF_FAST_INIT_START_TIME_SHIFT        (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R12_HPF_FAST_INIT_START_TIME_RESETVAL     (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R12_HPF_FAST_INIT_START_TIME_MAX          (0x000007FFU)

#define CSL_BSS_PROFILE_RAM_PROFILE4_R12_HPF_FAST_INIT_PULSE_WIDTH_MASK        (0x0000F800U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R12_HPF_FAST_INIT_PULSE_WIDTH_SHIFT       (0x0000000BU)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R12_HPF_FAST_INIT_PULSE_WIDTH_RESETVAL    (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R12_HPF_FAST_INIT_PULSE_WIDTH_MAX         (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE4_R12_HPF_FAST_INIT_FEATURE_EN_MASK         (0x00010000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R12_HPF_FAST_INIT_FEATURE_EN_SHIFT        (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R12_HPF_FAST_INIT_FEATURE_EN_RESETVAL     (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R12_HPF_FAST_INIT_FEATURE_EN_MAX          (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE4_R12_NU3_MASK                              (0xFFFE0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R12_NU3_SHIFT                             (0x00000011U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R12_NU3_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R12_NU3_MAX                               (0x00007FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE4_R12_RESETVAL                              (0x00000000U)

/* PROFILE4_R13 */

#define CSL_BSS_PROFILE_RAM_PROFILE4_R13_CRD_ENABLE_MASK                       (0x00000001U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R13_CRD_ENABLE_SHIFT                      (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R13_CRD_ENABLE_RESETVAL                   (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R13_CRD_ENABLE_MAX                        (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE4_R13_CRD_DITHER_ENABLE_MASK                (0x00000002U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R13_CRD_DITHER_ENABLE_SHIFT               (0x00000001U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R13_CRD_DITHER_ENABLE_RESETVAL            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R13_CRD_DITHER_ENABLE_MAX                 (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE4_R13_NU_MASK                               (0x00000004U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R13_NU_SHIFT                              (0x00000002U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R13_NU_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R13_NU_MAX                                (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE4_R13_CRD_FREQ_OFFSET_TIME_MASK             (0x0000FFF8U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R13_CRD_FREQ_OFFSET_TIME_SHIFT            (0x00000003U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R13_CRD_FREQ_OFFSET_TIME_RESETVAL         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R13_CRD_FREQ_OFFSET_TIME_MAX              (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE4_R13_CRD_NSLOPE_MAG_MASK                   (0xFFFF0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R13_CRD_NSLOPE_MAG_SHIFT                  (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R13_CRD_NSLOPE_MAG_RESETVAL               (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE4_R13_CRD_NSLOPE_MAG_MAX                    (0x0000FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE4_R13_RESETVAL                              (0x00000000U)

/* PROFILE5_R1 */

#define CSL_BSS_PROFILE_RAM_PROFILE5_R1_N_START_CONSTANT_MASK                  (0x7FFFFFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R1_N_START_CONSTANT_SHIFT                 (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R1_N_START_CONSTANT_RESETVAL              (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R1_N_START_CONSTANT_MAX                   (0x7FFFFFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE5_R1_NU_MASK                                (0x80000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R1_NU_SHIFT                               (0x0000001FU)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R1_NU_RESETVAL                            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R1_NU_MAX                                 (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE5_R1_RESETVAL                               (0x00000000U)

/* PROFILE5_R2 */

#define CSL_BSS_PROFILE_RAM_PROFILE5_R2_N_SLOPE_CONSTANT_MASK                  (0x0000FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R2_N_SLOPE_CONSTANT_SHIFT                 (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R2_N_SLOPE_CONSTANT_RESETVAL              (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R2_N_SLOPE_CONSTANT_MAX                   (0x0000FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE5_R2_FAST_RESET_END_TIME_MASK               (0x1FFF0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R2_FAST_RESET_END_TIME_SHIFT              (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R2_FAST_RESET_END_TIME_RESETVAL           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R2_FAST_RESET_END_TIME_MAX                (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE5_R2_DISABLE_FAST_RESET_MASK                (0x20000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R2_DISABLE_FAST_RESET_SHIFT               (0x0000001DU)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R2_DISABLE_FAST_RESET_RESETVAL            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R2_DISABLE_FAST_RESET_MAX                 (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE5_R2_NU_MASK                                (0xC0000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R2_NU_SHIFT                               (0x0000001EU)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R2_NU_RESETVAL                            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R2_NU_MAX                                 (0x00000003U)

#define CSL_BSS_PROFILE_RAM_PROFILE5_R2_RESETVAL                               (0x00000000U)

/* PROFILE5_R3 */

#define CSL_BSS_PROFILE_RAM_PROFILE5_R3_IDLE_TIME_CONSTANT_MASK                (0x0007FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R3_IDLE_TIME_CONSTANT_SHIFT               (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R3_IDLE_TIME_CONSTANT_RESETVAL            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R3_IDLE_TIME_CONSTANT_MAX                 (0x0007FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE5_R3_NU1_MASK                               (0x00F80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R3_NU1_SHIFT                              (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R3_NU1_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R3_NU1_MAX                                (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE5_R3_DISABLE_MONITORING_MASK                (0x01000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R3_DISABLE_MONITORING_SHIFT               (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R3_DISABLE_MONITORING_RESETVAL            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R3_DISABLE_MONITORING_MAX                 (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE5_R3_NU2_MASK                               (0xFE000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R3_NU2_SHIFT                              (0x00000019U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R3_NU2_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R3_NU2_MAX                                (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE5_R3_RESETVAL                               (0x00000000U)

/* PROFILE5_R4 */

#define CSL_BSS_PROFILE_RAM_PROFILE5_R4_ADC_VALID_START_TIME_MASK              (0x0007FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R4_ADC_VALID_START_TIME_SHIFT             (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R4_ADC_VALID_START_TIME_RESETVAL          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R4_ADC_VALID_START_TIME_MAX               (0x0007FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE5_R4_NU1_MASK                               (0x00F80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R4_NU1_SHIFT                              (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R4_NU1_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R4_NU1_MAX                                (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE5_R4_DONT_TURN_OFF_MASK                     (0x01000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R4_DONT_TURN_OFF_SHIFT                    (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R4_DONT_TURN_OFF_RESETVAL                 (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R4_DONT_TURN_OFF_MAX                      (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE5_R4_NU2_MASK                               (0xFE000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R4_NU2_SHIFT                              (0x00000019U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R4_NU2_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R4_NU2_MAX                                (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE5_R4_RESETVAL                               (0x00000000U)

/* PROFILE5_R5 */

#define CSL_BSS_PROFILE_RAM_PROFILE5_R5_ADC_SAMPLING_TIME_MASK                 (0x0007FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R5_ADC_SAMPLING_TIME_SHIFT                (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R5_ADC_SAMPLING_TIME_RESETVAL             (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R5_ADC_SAMPLING_TIME_MAX                  (0x0007FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE5_R5_NU_MASK                                (0xFFF80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R5_NU_SHIFT                               (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R5_NU_RESETVAL                            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R5_NU_MAX                                 (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE5_R5_RESETVAL                               (0x00000000U)

/* PROFILE5_R6 */

#define CSL_BSS_PROFILE_RAM_PROFILE5_R6_TX_START_TIME_MASK                     (0x00001FFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R6_TX_START_TIME_SHIFT                    (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R6_TX_START_TIME_RESETVAL                 (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R6_TX_START_TIME_MAX                      (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE5_R6_NU1_MASK                               (0x0000E000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R6_NU1_SHIFT                              (0x0000000DU)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R6_NU1_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R6_NU1_MAX                                (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE5_R6_ENABLE_SEQ_EXTENSION_MASK              (0x00010000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R6_ENABLE_SEQ_EXTENSION_SHIFT             (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R6_ENABLE_SEQ_EXTENSION_RESETVAL          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R6_ENABLE_SEQ_EXTENSION_MAX               (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE5_R6_NU2_MASK                               (0x00FE0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R6_NU2_SHIFT                              (0x00000011U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R6_NU2_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R6_NU2_MAX                                (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE5_R6_VCO_SELECT_MASK                        (0x01000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R6_VCO_SELECT_SHIFT                       (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R6_VCO_SELECT_RESETVAL                    (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R6_VCO_SELECT_MAX                         (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE5_R6_STITCHING_FIRST_HALF_MASK              (0x02000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R6_STITCHING_FIRST_HALF_SHIFT             (0x00000019U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R6_STITCHING_FIRST_HALF_RESETVAL          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R6_STITCHING_FIRST_HALF_MAX               (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE5_R6_NU3_MASK                               (0xFC000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R6_NU3_SHIFT                              (0x0000001AU)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R6_NU3_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R6_NU3_MAX                                (0x0000003FU)

#define CSL_BSS_PROFILE_RAM_PROFILE5_R6_RESETVAL                               (0x00000000U)

/* PROFILE5_R7 */

#define CSL_BSS_PROFILE_RAM_PROFILE5_R7_MONITOR_START_TIME_MASK                (0x00001FFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R7_MONITOR_START_TIME_SHIFT               (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R7_MONITOR_START_TIME_RESETVAL            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R7_MONITOR_START_TIME_MAX                 (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE5_R7_NU1_MASK                               (0x0000E000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R7_NU1_SHIFT                              (0x0000000DU)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R7_NU1_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R7_NU1_MAX                                (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE5_R7_RX_DFE_RESET_TIME_MASK                 (0x1FFF0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R7_RX_DFE_RESET_TIME_SHIFT                (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R7_RX_DFE_RESET_TIME_RESETVAL             (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R7_RX_DFE_RESET_TIME_MAX                  (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE5_R7_NU2_MASK                               (0xE0000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R7_NU2_SHIFT                              (0x0000001DU)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R7_NU2_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R7_NU2_MAX                                (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE5_R7_RESETVAL                               (0x00000000U)

/* PROFILE5_R8 */

#define CSL_BSS_PROFILE_RAM_PROFILE5_R8_RAMP_END_TIME_MASK                     (0x0007FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R8_RAMP_END_TIME_SHIFT                    (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R8_RAMP_END_TIME_RESETVAL                 (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R8_RAMP_END_TIME_MAX                      (0x0007FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE5_R8_NU_MASK                                (0xFFF80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R8_NU_SHIFT                               (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R8_NU_RESETVAL                            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R8_NU_MAX                                 (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE5_R8_RESETVAL                               (0x00000000U)

/* PROFILE5_R9 */

#define CSL_BSS_PROFILE_RAM_PROFILE5_R9_SYNTH_ANA_CTRL_MASK                    (0x0FFFFFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R9_SYNTH_ANA_CTRL_SHIFT                   (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R9_SYNTH_ANA_CTRL_RESETVAL                (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R9_SYNTH_ANA_CTRL_MAX                     (0x0FFFFFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE5_R9_NU_MASK                                (0xF0000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R9_NU_SHIFT                               (0x0000001CU)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R9_NU_RESETVAL                            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R9_NU_MAX                                 (0x0000000FU)

#define CSL_BSS_PROFILE_RAM_PROFILE5_R9_RESETVAL                               (0x00000000U)

/* PROFILE5_R10 */

#define CSL_BSS_PROFILE_RAM_PROFILE5_R10_FREQ_MON_THRESHOLD_MASK               (0x001FFFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R10_FREQ_MON_THRESHOLD_SHIFT              (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R10_FREQ_MON_THRESHOLD_RESETVAL           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R10_FREQ_MON_THRESHOLD_MAX                (0x001FFFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE5_R10_NU1_MASK                              (0x00E00000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R10_NU1_SHIFT                             (0x00000015U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R10_NU1_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R10_NU1_MAX                               (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE5_R10_ENABLE_LIN_MEAS_MASK                  (0x01000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R10_ENABLE_LIN_MEAS_SHIFT                 (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R10_ENABLE_LIN_MEAS_RESETVAL              (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R10_ENABLE_LIN_MEAS_MAX                   (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE5_R10_NU2_MASK                              (0xFE000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R10_NU2_SHIFT                             (0x00000019U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R10_NU2_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R10_NU2_MAX                               (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE5_R10_RESETVAL                              (0x00000000U)

/* PROFILE5_R11 */

#define CSL_BSS_PROFILE_RAM_PROFILE5_R11_FREQ_MON_L1_MASK                      (0x0000007FU)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R11_FREQ_MON_L1_SHIFT                     (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R11_FREQ_MON_L1_RESETVAL                  (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R11_FREQ_MON_L1_MAX                       (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE5_R11_NU1_MASK                              (0x00000080U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R11_NU1_SHIFT                             (0x00000007U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R11_NU1_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R11_NU1_MAX                               (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE5_R11_FREQ_MON_L2_MASK                      (0x00007F00U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R11_FREQ_MON_L2_SHIFT                     (0x00000008U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R11_FREQ_MON_L2_RESETVAL                  (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R11_FREQ_MON_L2_MAX                       (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE5_R11_NU2_MASK                              (0x00008000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R11_NU2_SHIFT                             (0x0000000FU)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R11_NU2_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R11_NU2_MAX                               (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE5_R11_FREQ_MON_S1_MASK                      (0x00070000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R11_FREQ_MON_S1_SHIFT                     (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R11_FREQ_MON_S1_RESETVAL                  (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R11_FREQ_MON_S1_MAX                       (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE5_R11_NU3_MASK                              (0x00F80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R11_NU3_SHIFT                             (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R11_NU3_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R11_NU3_MAX                               (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE5_R11_FREQ_MON_S2_MASK                      (0x07000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R11_FREQ_MON_S2_SHIFT                     (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R11_FREQ_MON_S2_RESETVAL                  (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R11_FREQ_MON_S2_MAX                       (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE5_R11_NU4_MASK                              (0xF8000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R11_NU4_SHIFT                             (0x0000001BU)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R11_NU4_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R11_NU4_MAX                               (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE5_R11_RESETVAL                              (0x00000000U)

/* PROFILE5_R12 */

#define CSL_BSS_PROFILE_RAM_PROFILE5_R12_HPF_FAST_INIT_START_TIME_MASK         (0x000007FFU)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R12_HPF_FAST_INIT_START_TIME_SHIFT        (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R12_HPF_FAST_INIT_START_TIME_RESETVAL     (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R12_HPF_FAST_INIT_START_TIME_MAX          (0x000007FFU)

#define CSL_BSS_PROFILE_RAM_PROFILE5_R12_HPF_FAST_INIT_PULSE_WIDTH_MASK        (0x0000F800U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R12_HPF_FAST_INIT_PULSE_WIDTH_SHIFT       (0x0000000BU)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R12_HPF_FAST_INIT_PULSE_WIDTH_RESETVAL    (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R12_HPF_FAST_INIT_PULSE_WIDTH_MAX         (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE5_R12_HPF_FAST_INIT_FEATURE_EN_MASK         (0x00010000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R12_HPF_FAST_INIT_FEATURE_EN_SHIFT        (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R12_HPF_FAST_INIT_FEATURE_EN_RESETVAL     (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R12_HPF_FAST_INIT_FEATURE_EN_MAX          (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE5_R12_NU3_MASK                              (0xFFFE0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R12_NU3_SHIFT                             (0x00000011U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R12_NU3_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R12_NU3_MAX                               (0x00007FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE5_R12_RESETVAL                              (0x00000000U)

/* PROFILE5_R13 */

#define CSL_BSS_PROFILE_RAM_PROFILE5_R13_CRD_ENABLE_MASK                       (0x00000001U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R13_CRD_ENABLE_SHIFT                      (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R13_CRD_ENABLE_RESETVAL                   (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R13_CRD_ENABLE_MAX                        (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE5_R13_CRD_DITHER_ENABLE_MASK                (0x00000002U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R13_CRD_DITHER_ENABLE_SHIFT               (0x00000001U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R13_CRD_DITHER_ENABLE_RESETVAL            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R13_CRD_DITHER_ENABLE_MAX                 (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE5_R13_NU_MASK                               (0x00000004U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R13_NU_SHIFT                              (0x00000002U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R13_NU_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R13_NU_MAX                                (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE5_R13_CRD_FREQ_OFFSET_TIME_MASK             (0x0000FFF8U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R13_CRD_FREQ_OFFSET_TIME_SHIFT            (0x00000003U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R13_CRD_FREQ_OFFSET_TIME_RESETVAL         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R13_CRD_FREQ_OFFSET_TIME_MAX              (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE5_R13_CRD_NSLOPE_MAG_MASK                   (0xFFFF0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R13_CRD_NSLOPE_MAG_SHIFT                  (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R13_CRD_NSLOPE_MAG_RESETVAL               (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE5_R13_CRD_NSLOPE_MAG_MAX                    (0x0000FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE5_R13_RESETVAL                              (0x00000000U)

/* PROFILE6_R1 */

#define CSL_BSS_PROFILE_RAM_PROFILE6_R1_N_START_CONSTANT_MASK                  (0x7FFFFFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R1_N_START_CONSTANT_SHIFT                 (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R1_N_START_CONSTANT_RESETVAL              (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R1_N_START_CONSTANT_MAX                   (0x7FFFFFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE6_R1_NU_MASK                                (0x80000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R1_NU_SHIFT                               (0x0000001FU)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R1_NU_RESETVAL                            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R1_NU_MAX                                 (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE6_R1_RESETVAL                               (0x00000000U)

/* PROFILE6_R2 */

#define CSL_BSS_PROFILE_RAM_PROFILE6_R2_N_SLOPE_CONSTANT_MASK                  (0x0000FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R2_N_SLOPE_CONSTANT_SHIFT                 (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R2_N_SLOPE_CONSTANT_RESETVAL              (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R2_N_SLOPE_CONSTANT_MAX                   (0x0000FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE6_R2_FAST_RESET_END_TIME_MASK               (0x1FFF0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R2_FAST_RESET_END_TIME_SHIFT              (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R2_FAST_RESET_END_TIME_RESETVAL           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R2_FAST_RESET_END_TIME_MAX                (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE6_R2_DISABLE_FAST_RESET_MASK                (0x20000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R2_DISABLE_FAST_RESET_SHIFT               (0x0000001DU)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R2_DISABLE_FAST_RESET_RESETVAL            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R2_DISABLE_FAST_RESET_MAX                 (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE6_R2_NU_MASK                                (0xC0000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R2_NU_SHIFT                               (0x0000001EU)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R2_NU_RESETVAL                            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R2_NU_MAX                                 (0x00000003U)

#define CSL_BSS_PROFILE_RAM_PROFILE6_R2_RESETVAL                               (0x00000000U)

/* PROFILE6_R3 */

#define CSL_BSS_PROFILE_RAM_PROFILE6_R3_IDLE_TIME_CONSTANT_MASK                (0x0007FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R3_IDLE_TIME_CONSTANT_SHIFT               (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R3_IDLE_TIME_CONSTANT_RESETVAL            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R3_IDLE_TIME_CONSTANT_MAX                 (0x0007FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE6_R3_NU1_MASK                               (0x00F80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R3_NU1_SHIFT                              (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R3_NU1_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R3_NU1_MAX                                (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE6_R3_DISABLE_MONITORING_MASK                (0x01000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R3_DISABLE_MONITORING_SHIFT               (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R3_DISABLE_MONITORING_RESETVAL            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R3_DISABLE_MONITORING_MAX                 (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE6_R3_NU2_MASK                               (0xFE000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R3_NU2_SHIFT                              (0x00000019U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R3_NU2_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R3_NU2_MAX                                (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE6_R3_RESETVAL                               (0x00000000U)

/* PROFILE6_R4 */

#define CSL_BSS_PROFILE_RAM_PROFILE6_R4_ADC_VALID_START_TIME_MASK              (0x0007FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R4_ADC_VALID_START_TIME_SHIFT             (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R4_ADC_VALID_START_TIME_RESETVAL          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R4_ADC_VALID_START_TIME_MAX               (0x0007FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE6_R4_NU1_MASK                               (0x00F80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R4_NU1_SHIFT                              (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R4_NU1_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R4_NU1_MAX                                (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE6_R4_DONT_TURN_OFF_MASK                     (0x01000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R4_DONT_TURN_OFF_SHIFT                    (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R4_DONT_TURN_OFF_RESETVAL                 (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R4_DONT_TURN_OFF_MAX                      (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE6_R4_NU2_MASK                               (0xFE000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R4_NU2_SHIFT                              (0x00000019U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R4_NU2_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R4_NU2_MAX                                (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE6_R4_RESETVAL                               (0x00000000U)

/* PROFILE6_R5 */

#define CSL_BSS_PROFILE_RAM_PROFILE6_R5_ADC_SAMPLING_TIME_MASK                 (0x0007FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R5_ADC_SAMPLING_TIME_SHIFT                (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R5_ADC_SAMPLING_TIME_RESETVAL             (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R5_ADC_SAMPLING_TIME_MAX                  (0x0007FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE6_R5_NU_MASK                                (0xFFF80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R5_NU_SHIFT                               (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R5_NU_RESETVAL                            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R5_NU_MAX                                 (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE6_R5_RESETVAL                               (0x00000000U)

/* PROFILE6_R6 */

#define CSL_BSS_PROFILE_RAM_PROFILE6_R6_TX_START_TIME_MASK                     (0x00001FFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R6_TX_START_TIME_SHIFT                    (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R6_TX_START_TIME_RESETVAL                 (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R6_TX_START_TIME_MAX                      (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE6_R6_NU1_MASK                               (0x0000E000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R6_NU1_SHIFT                              (0x0000000DU)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R6_NU1_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R6_NU1_MAX                                (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE6_R6_ENABLE_SEQ_EXTENSION_MASK              (0x00010000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R6_ENABLE_SEQ_EXTENSION_SHIFT             (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R6_ENABLE_SEQ_EXTENSION_RESETVAL          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R6_ENABLE_SEQ_EXTENSION_MAX               (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE6_R6_NU2_MASK                               (0x00FE0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R6_NU2_SHIFT                              (0x00000011U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R6_NU2_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R6_NU2_MAX                                (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE6_R6_VCO_SELECT_MASK                        (0x01000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R6_VCO_SELECT_SHIFT                       (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R6_VCO_SELECT_RESETVAL                    (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R6_VCO_SELECT_MAX                         (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE6_R6_STITCHING_FIRST_HALF_MASK              (0x02000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R6_STITCHING_FIRST_HALF_SHIFT             (0x00000019U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R6_STITCHING_FIRST_HALF_RESETVAL          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R6_STITCHING_FIRST_HALF_MAX               (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE6_R6_NU3_MASK                               (0xFC000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R6_NU3_SHIFT                              (0x0000001AU)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R6_NU3_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R6_NU3_MAX                                (0x0000003FU)

#define CSL_BSS_PROFILE_RAM_PROFILE6_R6_RESETVAL                               (0x00000000U)

/* PROFILE6_R7 */

#define CSL_BSS_PROFILE_RAM_PROFILE6_R7_MONITOR_START_TIME_MASK                (0x00001FFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R7_MONITOR_START_TIME_SHIFT               (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R7_MONITOR_START_TIME_RESETVAL            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R7_MONITOR_START_TIME_MAX                 (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE6_R7_NU1_MASK                               (0x0000E000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R7_NU1_SHIFT                              (0x0000000DU)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R7_NU1_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R7_NU1_MAX                                (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE6_R7_RX_DFE_RESET_TIME_MASK                 (0x1FFF0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R7_RX_DFE_RESET_TIME_SHIFT                (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R7_RX_DFE_RESET_TIME_RESETVAL             (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R7_RX_DFE_RESET_TIME_MAX                  (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE6_R7_NU2_MASK                               (0xE0000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R7_NU2_SHIFT                              (0x0000001DU)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R7_NU2_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R7_NU2_MAX                                (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE6_R7_RESETVAL                               (0x00000000U)

/* PROFILE6_R8 */

#define CSL_BSS_PROFILE_RAM_PROFILE6_R8_RAMP_END_TIME_MASK                     (0x0007FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R8_RAMP_END_TIME_SHIFT                    (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R8_RAMP_END_TIME_RESETVAL                 (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R8_RAMP_END_TIME_MAX                      (0x0007FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE6_R8_NU_MASK                                (0xFFF80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R8_NU_SHIFT                               (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R8_NU_RESETVAL                            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R8_NU_MAX                                 (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE6_R8_RESETVAL                               (0x00000000U)

/* PROFILE6_R9 */

#define CSL_BSS_PROFILE_RAM_PROFILE6_R9_SYNTH_ANA_CTRL_MASK                    (0x0FFFFFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R9_SYNTH_ANA_CTRL_SHIFT                   (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R9_SYNTH_ANA_CTRL_RESETVAL                (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R9_SYNTH_ANA_CTRL_MAX                     (0x0FFFFFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE6_R9_NU_MASK                                (0xF0000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R9_NU_SHIFT                               (0x0000001CU)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R9_NU_RESETVAL                            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R9_NU_MAX                                 (0x0000000FU)

#define CSL_BSS_PROFILE_RAM_PROFILE6_R9_RESETVAL                               (0x00000000U)

/* PROFILE6_R10 */

#define CSL_BSS_PROFILE_RAM_PROFILE6_R10_FREQ_MON_THRESHOLD_MASK               (0x001FFFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R10_FREQ_MON_THRESHOLD_SHIFT              (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R10_FREQ_MON_THRESHOLD_RESETVAL           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R10_FREQ_MON_THRESHOLD_MAX                (0x001FFFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE6_R10_NU1_MASK                              (0x00E00000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R10_NU1_SHIFT                             (0x00000015U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R10_NU1_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R10_NU1_MAX                               (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE6_R10_ENABLE_LIN_MEAS_MASK                  (0x01000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R10_ENABLE_LIN_MEAS_SHIFT                 (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R10_ENABLE_LIN_MEAS_RESETVAL              (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R10_ENABLE_LIN_MEAS_MAX                   (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE6_R10_NU2_MASK                              (0xFE000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R10_NU2_SHIFT                             (0x00000019U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R10_NU2_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R10_NU2_MAX                               (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE6_R10_RESETVAL                              (0x00000000U)

/* PROFILE6_R11 */

#define CSL_BSS_PROFILE_RAM_PROFILE6_R11_FREQ_MON_L1_MASK                      (0x0000007FU)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R11_FREQ_MON_L1_SHIFT                     (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R11_FREQ_MON_L1_RESETVAL                  (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R11_FREQ_MON_L1_MAX                       (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE6_R11_NU1_MASK                              (0x00000080U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R11_NU1_SHIFT                             (0x00000007U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R11_NU1_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R11_NU1_MAX                               (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE6_R11_FREQ_MON_L2_MASK                      (0x00007F00U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R11_FREQ_MON_L2_SHIFT                     (0x00000008U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R11_FREQ_MON_L2_RESETVAL                  (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R11_FREQ_MON_L2_MAX                       (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE6_R11_NU2_MASK                              (0x00008000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R11_NU2_SHIFT                             (0x0000000FU)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R11_NU2_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R11_NU2_MAX                               (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE6_R11_FREQ_MON_S1_MASK                      (0x00070000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R11_FREQ_MON_S1_SHIFT                     (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R11_FREQ_MON_S1_RESETVAL                  (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R11_FREQ_MON_S1_MAX                       (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE6_R11_NU3_MASK                              (0x00F80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R11_NU3_SHIFT                             (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R11_NU3_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R11_NU3_MAX                               (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE6_R11_FREQ_MON_S2_MASK                      (0x07000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R11_FREQ_MON_S2_SHIFT                     (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R11_FREQ_MON_S2_RESETVAL                  (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R11_FREQ_MON_S2_MAX                       (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE6_R11_NU4_MASK                              (0xF8000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R11_NU4_SHIFT                             (0x0000001BU)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R11_NU4_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R11_NU4_MAX                               (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE6_R11_RESETVAL                              (0x00000000U)

/* PROFILE6_R12 */

#define CSL_BSS_PROFILE_RAM_PROFILE6_R12_HPF_FAST_INIT_START_TIME_MASK         (0x000007FFU)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R12_HPF_FAST_INIT_START_TIME_SHIFT        (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R12_HPF_FAST_INIT_START_TIME_RESETVAL     (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R12_HPF_FAST_INIT_START_TIME_MAX          (0x000007FFU)

#define CSL_BSS_PROFILE_RAM_PROFILE6_R12_HPF_FAST_INIT_PULSE_WIDTH_MASK        (0x0000F800U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R12_HPF_FAST_INIT_PULSE_WIDTH_SHIFT       (0x0000000BU)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R12_HPF_FAST_INIT_PULSE_WIDTH_RESETVAL    (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R12_HPF_FAST_INIT_PULSE_WIDTH_MAX         (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE6_R12_HPF_FAST_INIT_FEATURE_EN_MASK         (0x00010000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R12_HPF_FAST_INIT_FEATURE_EN_SHIFT        (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R12_HPF_FAST_INIT_FEATURE_EN_RESETVAL     (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R12_HPF_FAST_INIT_FEATURE_EN_MAX          (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE6_R12_NU3_MASK                              (0xFFFE0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R12_NU3_SHIFT                             (0x00000011U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R12_NU3_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R12_NU3_MAX                               (0x00007FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE6_R12_RESETVAL                              (0x00000000U)

/* PROFILE6_R13 */

#define CSL_BSS_PROFILE_RAM_PROFILE6_R13_CRD_ENABLE_MASK                       (0x00000001U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R13_CRD_ENABLE_SHIFT                      (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R13_CRD_ENABLE_RESETVAL                   (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R13_CRD_ENABLE_MAX                        (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE6_R13_CRD_DITHER_ENABLE_MASK                (0x00000002U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R13_CRD_DITHER_ENABLE_SHIFT               (0x00000001U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R13_CRD_DITHER_ENABLE_RESETVAL            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R13_CRD_DITHER_ENABLE_MAX                 (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE6_R13_NU_MASK                               (0x00000004U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R13_NU_SHIFT                              (0x00000002U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R13_NU_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R13_NU_MAX                                (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE6_R13_CRD_FREQ_OFFSET_TIME_MASK             (0x0000FFF8U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R13_CRD_FREQ_OFFSET_TIME_SHIFT            (0x00000003U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R13_CRD_FREQ_OFFSET_TIME_RESETVAL         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R13_CRD_FREQ_OFFSET_TIME_MAX              (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE6_R13_CRD_NSLOPE_MAG_MASK                   (0xFFFF0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R13_CRD_NSLOPE_MAG_SHIFT                  (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R13_CRD_NSLOPE_MAG_RESETVAL               (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE6_R13_CRD_NSLOPE_MAG_MAX                    (0x0000FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE6_R13_RESETVAL                              (0x00000000U)

/* PROFILE7_R1 */

#define CSL_BSS_PROFILE_RAM_PROFILE7_R1_N_START_CONSTANT_MASK                  (0x7FFFFFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R1_N_START_CONSTANT_SHIFT                 (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R1_N_START_CONSTANT_RESETVAL              (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R1_N_START_CONSTANT_MAX                   (0x7FFFFFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE7_R1_NU_MASK                                (0x80000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R1_NU_SHIFT                               (0x0000001FU)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R1_NU_RESETVAL                            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R1_NU_MAX                                 (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE7_R1_RESETVAL                               (0x00000000U)

/* PROFILE7_R2 */

#define CSL_BSS_PROFILE_RAM_PROFILE7_R2_N_SLOPE_CONSTANT_MASK                  (0x0000FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R2_N_SLOPE_CONSTANT_SHIFT                 (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R2_N_SLOPE_CONSTANT_RESETVAL              (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R2_N_SLOPE_CONSTANT_MAX                   (0x0000FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE7_R2_FAST_RESET_END_TIME_MASK               (0x1FFF0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R2_FAST_RESET_END_TIME_SHIFT              (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R2_FAST_RESET_END_TIME_RESETVAL           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R2_FAST_RESET_END_TIME_MAX                (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE7_R2_DISABLE_FAST_RESET_MASK                (0x20000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R2_DISABLE_FAST_RESET_SHIFT               (0x0000001DU)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R2_DISABLE_FAST_RESET_RESETVAL            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R2_DISABLE_FAST_RESET_MAX                 (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE7_R2_NU_MASK                                (0xC0000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R2_NU_SHIFT                               (0x0000001EU)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R2_NU_RESETVAL                            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R2_NU_MAX                                 (0x00000003U)

#define CSL_BSS_PROFILE_RAM_PROFILE7_R2_RESETVAL                               (0x00000000U)

/* PROFILE7_R3 */

#define CSL_BSS_PROFILE_RAM_PROFILE7_R3_IDLE_TIME_CONSTANT_MASK                (0x0007FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R3_IDLE_TIME_CONSTANT_SHIFT               (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R3_IDLE_TIME_CONSTANT_RESETVAL            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R3_IDLE_TIME_CONSTANT_MAX                 (0x0007FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE7_R3_NU1_MASK                               (0x00F80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R3_NU1_SHIFT                              (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R3_NU1_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R3_NU1_MAX                                (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE7_R3_DISABLE_MONITORING_MASK                (0x01000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R3_DISABLE_MONITORING_SHIFT               (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R3_DISABLE_MONITORING_RESETVAL            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R3_DISABLE_MONITORING_MAX                 (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE7_R3_NU2_MASK                               (0xFE000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R3_NU2_SHIFT                              (0x00000019U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R3_NU2_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R3_NU2_MAX                                (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE7_R3_RESETVAL                               (0x00000000U)

/* PROFILE7_R4 */

#define CSL_BSS_PROFILE_RAM_PROFILE7_R4_ADC_VALID_START_TIME_MASK              (0x0007FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R4_ADC_VALID_START_TIME_SHIFT             (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R4_ADC_VALID_START_TIME_RESETVAL          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R4_ADC_VALID_START_TIME_MAX               (0x0007FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE7_R4_NU1_MASK                               (0x00F80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R4_NU1_SHIFT                              (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R4_NU1_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R4_NU1_MAX                                (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE7_R4_DONT_TURN_OFF_MASK                     (0x01000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R4_DONT_TURN_OFF_SHIFT                    (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R4_DONT_TURN_OFF_RESETVAL                 (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R4_DONT_TURN_OFF_MAX                      (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE7_R4_NU2_MASK                               (0xFE000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R4_NU2_SHIFT                              (0x00000019U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R4_NU2_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R4_NU2_MAX                                (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE7_R4_RESETVAL                               (0x00000000U)

/* PROFILE7_R5 */

#define CSL_BSS_PROFILE_RAM_PROFILE7_R5_ADC_SAMPLING_TIME_MASK                 (0x0007FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R5_ADC_SAMPLING_TIME_SHIFT                (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R5_ADC_SAMPLING_TIME_RESETVAL             (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R5_ADC_SAMPLING_TIME_MAX                  (0x0007FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE7_R5_NU_MASK                                (0xFFF80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R5_NU_SHIFT                               (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R5_NU_RESETVAL                            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R5_NU_MAX                                 (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE7_R5_RESETVAL                               (0x00000000U)

/* PROFILE7_R6 */

#define CSL_BSS_PROFILE_RAM_PROFILE7_R6_TX_START_TIME_MASK                     (0x00001FFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R6_TX_START_TIME_SHIFT                    (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R6_TX_START_TIME_RESETVAL                 (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R6_TX_START_TIME_MAX                      (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE7_R6_NU1_MASK                               (0x0000E000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R6_NU1_SHIFT                              (0x0000000DU)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R6_NU1_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R6_NU1_MAX                                (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE7_R6_ENABLE_SEQ_EXTENSION_MASK              (0x00010000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R6_ENABLE_SEQ_EXTENSION_SHIFT             (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R6_ENABLE_SEQ_EXTENSION_RESETVAL          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R6_ENABLE_SEQ_EXTENSION_MAX               (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE7_R6_NU2_MASK                               (0x00FE0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R6_NU2_SHIFT                              (0x00000011U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R6_NU2_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R6_NU2_MAX                                (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE7_R6_VCO_SELECT_MASK                        (0x01000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R6_VCO_SELECT_SHIFT                       (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R6_VCO_SELECT_RESETVAL                    (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R6_VCO_SELECT_MAX                         (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE7_R6_STITCHING_FIRST_HALF_MASK              (0x02000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R6_STITCHING_FIRST_HALF_SHIFT             (0x00000019U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R6_STITCHING_FIRST_HALF_RESETVAL          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R6_STITCHING_FIRST_HALF_MAX               (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE7_R6_NU3_MASK                               (0xFC000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R6_NU3_SHIFT                              (0x0000001AU)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R6_NU3_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R6_NU3_MAX                                (0x0000003FU)

#define CSL_BSS_PROFILE_RAM_PROFILE7_R6_RESETVAL                               (0x00000000U)

/* PROFILE7_R7 */

#define CSL_BSS_PROFILE_RAM_PROFILE7_R7_MONITOR_START_TIME_MASK                (0x00001FFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R7_MONITOR_START_TIME_SHIFT               (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R7_MONITOR_START_TIME_RESETVAL            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R7_MONITOR_START_TIME_MAX                 (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE7_R7_NU1_MASK                               (0x0000E000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R7_NU1_SHIFT                              (0x0000000DU)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R7_NU1_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R7_NU1_MAX                                (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE7_R7_RX_DFE_RESET_TIME_MASK                 (0x1FFF0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R7_RX_DFE_RESET_TIME_SHIFT                (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R7_RX_DFE_RESET_TIME_RESETVAL             (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R7_RX_DFE_RESET_TIME_MAX                  (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE7_R7_NU2_MASK                               (0xE0000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R7_NU2_SHIFT                              (0x0000001DU)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R7_NU2_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R7_NU2_MAX                                (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE7_R7_RESETVAL                               (0x00000000U)

/* PROFILE7_R8 */

#define CSL_BSS_PROFILE_RAM_PROFILE7_R8_RAMP_END_TIME_MASK                     (0x0007FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R8_RAMP_END_TIME_SHIFT                    (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R8_RAMP_END_TIME_RESETVAL                 (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R8_RAMP_END_TIME_MAX                      (0x0007FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE7_R8_NU_MASK                                (0xFFF80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R8_NU_SHIFT                               (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R8_NU_RESETVAL                            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R8_NU_MAX                                 (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE7_R8_RESETVAL                               (0x00000000U)

/* PROFILE7_R9 */

#define CSL_BSS_PROFILE_RAM_PROFILE7_R9_SYNTH_ANA_CTRL_MASK                    (0x0FFFFFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R9_SYNTH_ANA_CTRL_SHIFT                   (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R9_SYNTH_ANA_CTRL_RESETVAL                (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R9_SYNTH_ANA_CTRL_MAX                     (0x0FFFFFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE7_R9_NU_MASK                                (0xF0000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R9_NU_SHIFT                               (0x0000001CU)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R9_NU_RESETVAL                            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R9_NU_MAX                                 (0x0000000FU)

#define CSL_BSS_PROFILE_RAM_PROFILE7_R9_RESETVAL                               (0x00000000U)

/* PROFILE7_R10 */

#define CSL_BSS_PROFILE_RAM_PROFILE7_R10_FREQ_MON_THRESHOLD_MASK               (0x001FFFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R10_FREQ_MON_THRESHOLD_SHIFT              (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R10_FREQ_MON_THRESHOLD_RESETVAL           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R10_FREQ_MON_THRESHOLD_MAX                (0x001FFFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE7_R10_NU1_MASK                              (0x00E00000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R10_NU1_SHIFT                             (0x00000015U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R10_NU1_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R10_NU1_MAX                               (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE7_R10_ENABLE_LIN_MEAS_MASK                  (0x01000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R10_ENABLE_LIN_MEAS_SHIFT                 (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R10_ENABLE_LIN_MEAS_RESETVAL              (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R10_ENABLE_LIN_MEAS_MAX                   (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE7_R10_NU2_MASK                              (0xFE000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R10_NU2_SHIFT                             (0x00000019U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R10_NU2_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R10_NU2_MAX                               (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE7_R10_RESETVAL                              (0x00000000U)

/* PROFILE7_R11 */

#define CSL_BSS_PROFILE_RAM_PROFILE7_R11_FREQ_MON_L1_MASK                      (0x0000007FU)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R11_FREQ_MON_L1_SHIFT                     (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R11_FREQ_MON_L1_RESETVAL                  (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R11_FREQ_MON_L1_MAX                       (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE7_R11_NU1_MASK                              (0x00000080U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R11_NU1_SHIFT                             (0x00000007U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R11_NU1_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R11_NU1_MAX                               (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE7_R11_FREQ_MON_L2_MASK                      (0x00007F00U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R11_FREQ_MON_L2_SHIFT                     (0x00000008U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R11_FREQ_MON_L2_RESETVAL                  (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R11_FREQ_MON_L2_MAX                       (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE7_R11_NU2_MASK                              (0x00008000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R11_NU2_SHIFT                             (0x0000000FU)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R11_NU2_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R11_NU2_MAX                               (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE7_R11_FREQ_MON_S1_MASK                      (0x00070000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R11_FREQ_MON_S1_SHIFT                     (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R11_FREQ_MON_S1_RESETVAL                  (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R11_FREQ_MON_S1_MAX                       (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE7_R11_NU3_MASK                              (0x00F80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R11_NU3_SHIFT                             (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R11_NU3_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R11_NU3_MAX                               (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE7_R11_FREQ_MON_S2_MASK                      (0x07000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R11_FREQ_MON_S2_SHIFT                     (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R11_FREQ_MON_S2_RESETVAL                  (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R11_FREQ_MON_S2_MAX                       (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE7_R11_NU4_MASK                              (0xF8000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R11_NU4_SHIFT                             (0x0000001BU)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R11_NU4_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R11_NU4_MAX                               (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE7_R11_RESETVAL                              (0x00000000U)

/* PROFILE7_R12 */

#define CSL_BSS_PROFILE_RAM_PROFILE7_R12_HPF_FAST_INIT_START_TIME_MASK         (0x000007FFU)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R12_HPF_FAST_INIT_START_TIME_SHIFT        (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R12_HPF_FAST_INIT_START_TIME_RESETVAL     (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R12_HPF_FAST_INIT_START_TIME_MAX          (0x000007FFU)

#define CSL_BSS_PROFILE_RAM_PROFILE7_R12_HPF_FAST_INIT_PULSE_WIDTH_MASK        (0x0000F800U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R12_HPF_FAST_INIT_PULSE_WIDTH_SHIFT       (0x0000000BU)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R12_HPF_FAST_INIT_PULSE_WIDTH_RESETVAL    (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R12_HPF_FAST_INIT_PULSE_WIDTH_MAX         (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE7_R12_HPF_FAST_INIT_FEATURE_EN_MASK         (0x00010000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R12_HPF_FAST_INIT_FEATURE_EN_SHIFT        (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R12_HPF_FAST_INIT_FEATURE_EN_RESETVAL     (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R12_HPF_FAST_INIT_FEATURE_EN_MAX          (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE7_R12_NU3_MASK                              (0xFFFE0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R12_NU3_SHIFT                             (0x00000011U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R12_NU3_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R12_NU3_MAX                               (0x00007FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE7_R12_RESETVAL                              (0x00000000U)

/* PROFILE7_R13 */

#define CSL_BSS_PROFILE_RAM_PROFILE7_R13_CRD_ENABLE_MASK                       (0x00000001U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R13_CRD_ENABLE_SHIFT                      (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R13_CRD_ENABLE_RESETVAL                   (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R13_CRD_ENABLE_MAX                        (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE7_R13_CRD_DITHER_ENABLE_MASK                (0x00000002U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R13_CRD_DITHER_ENABLE_SHIFT               (0x00000001U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R13_CRD_DITHER_ENABLE_RESETVAL            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R13_CRD_DITHER_ENABLE_MAX                 (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE7_R13_NU_MASK                               (0x00000004U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R13_NU_SHIFT                              (0x00000002U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R13_NU_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R13_NU_MAX                                (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE7_R13_CRD_FREQ_OFFSET_TIME_MASK             (0x0000FFF8U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R13_CRD_FREQ_OFFSET_TIME_SHIFT            (0x00000003U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R13_CRD_FREQ_OFFSET_TIME_RESETVAL         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R13_CRD_FREQ_OFFSET_TIME_MAX              (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE7_R13_CRD_NSLOPE_MAG_MASK                   (0xFFFF0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R13_CRD_NSLOPE_MAG_SHIFT                  (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R13_CRD_NSLOPE_MAG_RESETVAL               (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE7_R13_CRD_NSLOPE_MAG_MAX                    (0x0000FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE7_R13_RESETVAL                              (0x00000000U)

/* PROFILE8_R1 */

#define CSL_BSS_PROFILE_RAM_PROFILE8_R1_N_START_CONSTANT_MASK                  (0x7FFFFFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R1_N_START_CONSTANT_SHIFT                 (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R1_N_START_CONSTANT_RESETVAL              (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R1_N_START_CONSTANT_MAX                   (0x7FFFFFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE8_R1_NU_MASK                                (0x80000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R1_NU_SHIFT                               (0x0000001FU)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R1_NU_RESETVAL                            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R1_NU_MAX                                 (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE8_R1_RESETVAL                               (0x00000000U)

/* PROFILE8_R2 */

#define CSL_BSS_PROFILE_RAM_PROFILE8_R2_N_SLOPE_CONSTANT_MASK                  (0x0000FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R2_N_SLOPE_CONSTANT_SHIFT                 (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R2_N_SLOPE_CONSTANT_RESETVAL              (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R2_N_SLOPE_CONSTANT_MAX                   (0x0000FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE8_R2_FAST_RESET_END_TIME_MASK               (0x1FFF0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R2_FAST_RESET_END_TIME_SHIFT              (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R2_FAST_RESET_END_TIME_RESETVAL           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R2_FAST_RESET_END_TIME_MAX                (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE8_R2_DISABLE_FAST_RESET_MASK                (0x20000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R2_DISABLE_FAST_RESET_SHIFT               (0x0000001DU)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R2_DISABLE_FAST_RESET_RESETVAL            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R2_DISABLE_FAST_RESET_MAX                 (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE8_R2_NU_MASK                                (0xC0000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R2_NU_SHIFT                               (0x0000001EU)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R2_NU_RESETVAL                            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R2_NU_MAX                                 (0x00000003U)

#define CSL_BSS_PROFILE_RAM_PROFILE8_R2_RESETVAL                               (0x00000000U)

/* PROFILE8_R3 */

#define CSL_BSS_PROFILE_RAM_PROFILE8_R3_IDLE_TIME_CONSTANT_MASK                (0x0007FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R3_IDLE_TIME_CONSTANT_SHIFT               (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R3_IDLE_TIME_CONSTANT_RESETVAL            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R3_IDLE_TIME_CONSTANT_MAX                 (0x0007FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE8_R3_NU1_MASK                               (0x00F80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R3_NU1_SHIFT                              (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R3_NU1_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R3_NU1_MAX                                (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE8_R3_DISABLE_MONITORING_MASK                (0x01000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R3_DISABLE_MONITORING_SHIFT               (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R3_DISABLE_MONITORING_RESETVAL            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R3_DISABLE_MONITORING_MAX                 (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE8_R3_NU2_MASK                               (0xFE000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R3_NU2_SHIFT                              (0x00000019U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R3_NU2_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R3_NU2_MAX                                (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE8_R3_RESETVAL                               (0x00000000U)

/* PROFILE8_R4 */

#define CSL_BSS_PROFILE_RAM_PROFILE8_R4_ADC_VALID_START_TIME_MASK              (0x0007FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R4_ADC_VALID_START_TIME_SHIFT             (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R4_ADC_VALID_START_TIME_RESETVAL          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R4_ADC_VALID_START_TIME_MAX               (0x0007FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE8_R4_NU1_MASK                               (0x00F80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R4_NU1_SHIFT                              (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R4_NU1_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R4_NU1_MAX                                (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE8_R4_DONT_TURN_OFF_MASK                     (0x01000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R4_DONT_TURN_OFF_SHIFT                    (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R4_DONT_TURN_OFF_RESETVAL                 (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R4_DONT_TURN_OFF_MAX                      (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE8_R4_NU2_MASK                               (0xFE000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R4_NU2_SHIFT                              (0x00000019U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R4_NU2_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R4_NU2_MAX                                (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE8_R4_RESETVAL                               (0x00000000U)

/* PROFILE8_R5 */

#define CSL_BSS_PROFILE_RAM_PROFILE8_R5_ADC_SAMPLING_TIME_MASK                 (0x0007FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R5_ADC_SAMPLING_TIME_SHIFT                (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R5_ADC_SAMPLING_TIME_RESETVAL             (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R5_ADC_SAMPLING_TIME_MAX                  (0x0007FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE8_R5_NU_MASK                                (0xFFF80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R5_NU_SHIFT                               (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R5_NU_RESETVAL                            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R5_NU_MAX                                 (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE8_R5_RESETVAL                               (0x00000000U)

/* PROFILE8_R6 */

#define CSL_BSS_PROFILE_RAM_PROFILE8_R6_TX_START_TIME_MASK                     (0x00001FFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R6_TX_START_TIME_SHIFT                    (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R6_TX_START_TIME_RESETVAL                 (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R6_TX_START_TIME_MAX                      (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE8_R6_NU1_MASK                               (0x0000E000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R6_NU1_SHIFT                              (0x0000000DU)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R6_NU1_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R6_NU1_MAX                                (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE8_R6_ENABLE_SEQ_EXTENSION_MASK              (0x00010000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R6_ENABLE_SEQ_EXTENSION_SHIFT             (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R6_ENABLE_SEQ_EXTENSION_RESETVAL          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R6_ENABLE_SEQ_EXTENSION_MAX               (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE8_R6_NU2_MASK                               (0x00FE0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R6_NU2_SHIFT                              (0x00000011U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R6_NU2_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R6_NU2_MAX                                (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE8_R6_VCO_SELECT_MASK                        (0x01000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R6_VCO_SELECT_SHIFT                       (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R6_VCO_SELECT_RESETVAL                    (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R6_VCO_SELECT_MAX                         (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE8_R6_STITCHING_FIRST_HALF_MASK              (0x02000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R6_STITCHING_FIRST_HALF_SHIFT             (0x00000019U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R6_STITCHING_FIRST_HALF_RESETVAL          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R6_STITCHING_FIRST_HALF_MAX               (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE8_R6_NU3_MASK                               (0xFC000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R6_NU3_SHIFT                              (0x0000001AU)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R6_NU3_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R6_NU3_MAX                                (0x0000003FU)

#define CSL_BSS_PROFILE_RAM_PROFILE8_R6_RESETVAL                               (0x00000000U)

/* PROFILE8_R7 */

#define CSL_BSS_PROFILE_RAM_PROFILE8_R7_MONITOR_START_TIME_MASK                (0x00001FFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R7_MONITOR_START_TIME_SHIFT               (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R7_MONITOR_START_TIME_RESETVAL            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R7_MONITOR_START_TIME_MAX                 (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE8_R7_NU1_MASK                               (0x0000E000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R7_NU1_SHIFT                              (0x0000000DU)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R7_NU1_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R7_NU1_MAX                                (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE8_R7_RX_DFE_RESET_TIME_MASK                 (0x1FFF0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R7_RX_DFE_RESET_TIME_SHIFT                (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R7_RX_DFE_RESET_TIME_RESETVAL             (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R7_RX_DFE_RESET_TIME_MAX                  (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE8_R7_NU2_MASK                               (0xE0000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R7_NU2_SHIFT                              (0x0000001DU)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R7_NU2_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R7_NU2_MAX                                (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE8_R7_RESETVAL                               (0x00000000U)

/* PROFILE8_R8 */

#define CSL_BSS_PROFILE_RAM_PROFILE8_R8_RAMP_END_TIME_MASK                     (0x0007FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R8_RAMP_END_TIME_SHIFT                    (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R8_RAMP_END_TIME_RESETVAL                 (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R8_RAMP_END_TIME_MAX                      (0x0007FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE8_R8_NU_MASK                                (0xFFF80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R8_NU_SHIFT                               (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R8_NU_RESETVAL                            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R8_NU_MAX                                 (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE8_R8_RESETVAL                               (0x00000000U)

/* PROFILE8_R9 */

#define CSL_BSS_PROFILE_RAM_PROFILE8_R9_SYNTH_ANA_CTRL_MASK                    (0x0FFFFFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R9_SYNTH_ANA_CTRL_SHIFT                   (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R9_SYNTH_ANA_CTRL_RESETVAL                (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R9_SYNTH_ANA_CTRL_MAX                     (0x0FFFFFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE8_R9_NU_MASK                                (0xF0000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R9_NU_SHIFT                               (0x0000001CU)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R9_NU_RESETVAL                            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R9_NU_MAX                                 (0x0000000FU)

#define CSL_BSS_PROFILE_RAM_PROFILE8_R9_RESETVAL                               (0x00000000U)

/* PROFILE8_R10 */

#define CSL_BSS_PROFILE_RAM_PROFILE8_R10_FREQ_MON_THRESHOLD_MASK               (0x001FFFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R10_FREQ_MON_THRESHOLD_SHIFT              (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R10_FREQ_MON_THRESHOLD_RESETVAL           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R10_FREQ_MON_THRESHOLD_MAX                (0x001FFFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE8_R10_NU1_MASK                              (0x00E00000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R10_NU1_SHIFT                             (0x00000015U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R10_NU1_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R10_NU1_MAX                               (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE8_R10_ENABLE_LIN_MEAS_MASK                  (0x01000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R10_ENABLE_LIN_MEAS_SHIFT                 (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R10_ENABLE_LIN_MEAS_RESETVAL              (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R10_ENABLE_LIN_MEAS_MAX                   (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE8_R10_NU2_MASK                              (0xFE000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R10_NU2_SHIFT                             (0x00000019U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R10_NU2_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R10_NU2_MAX                               (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE8_R10_RESETVAL                              (0x00000000U)

/* PROFILE8_R11 */

#define CSL_BSS_PROFILE_RAM_PROFILE8_R11_FREQ_MON_L1_MASK                      (0x0000007FU)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R11_FREQ_MON_L1_SHIFT                     (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R11_FREQ_MON_L1_RESETVAL                  (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R11_FREQ_MON_L1_MAX                       (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE8_R11_NU1_MASK                              (0x00000080U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R11_NU1_SHIFT                             (0x00000007U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R11_NU1_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R11_NU1_MAX                               (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE8_R11_FREQ_MON_L2_MASK                      (0x00007F00U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R11_FREQ_MON_L2_SHIFT                     (0x00000008U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R11_FREQ_MON_L2_RESETVAL                  (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R11_FREQ_MON_L2_MAX                       (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE8_R11_NU2_MASK                              (0x00008000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R11_NU2_SHIFT                             (0x0000000FU)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R11_NU2_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R11_NU2_MAX                               (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE8_R11_FREQ_MON_S1_MASK                      (0x00070000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R11_FREQ_MON_S1_SHIFT                     (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R11_FREQ_MON_S1_RESETVAL                  (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R11_FREQ_MON_S1_MAX                       (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE8_R11_NU3_MASK                              (0x00F80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R11_NU3_SHIFT                             (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R11_NU3_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R11_NU3_MAX                               (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE8_R11_FREQ_MON_S2_MASK                      (0x07000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R11_FREQ_MON_S2_SHIFT                     (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R11_FREQ_MON_S2_RESETVAL                  (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R11_FREQ_MON_S2_MAX                       (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE8_R11_NU4_MASK                              (0xF8000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R11_NU4_SHIFT                             (0x0000001BU)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R11_NU4_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R11_NU4_MAX                               (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE8_R11_RESETVAL                              (0x00000000U)

/* PROFILE8_R12 */

#define CSL_BSS_PROFILE_RAM_PROFILE8_R12_HPF_FAST_INIT_START_TIME_MASK         (0x000007FFU)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R12_HPF_FAST_INIT_START_TIME_SHIFT        (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R12_HPF_FAST_INIT_START_TIME_RESETVAL     (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R12_HPF_FAST_INIT_START_TIME_MAX          (0x000007FFU)

#define CSL_BSS_PROFILE_RAM_PROFILE8_R12_HPF_FAST_INIT_PULSE_WIDTH_MASK        (0x0000F800U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R12_HPF_FAST_INIT_PULSE_WIDTH_SHIFT       (0x0000000BU)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R12_HPF_FAST_INIT_PULSE_WIDTH_RESETVAL    (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R12_HPF_FAST_INIT_PULSE_WIDTH_MAX         (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE8_R12_HPF_FAST_INIT_FEATURE_EN_MASK         (0x00010000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R12_HPF_FAST_INIT_FEATURE_EN_SHIFT        (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R12_HPF_FAST_INIT_FEATURE_EN_RESETVAL     (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R12_HPF_FAST_INIT_FEATURE_EN_MAX          (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE8_R12_NU3_MASK                              (0xFFFE0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R12_NU3_SHIFT                             (0x00000011U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R12_NU3_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R12_NU3_MAX                               (0x00007FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE8_R12_RESETVAL                              (0x00000000U)

/* PROFILE8_R13 */

#define CSL_BSS_PROFILE_RAM_PROFILE8_R13_CRD_ENABLE_MASK                       (0x00000001U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R13_CRD_ENABLE_SHIFT                      (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R13_CRD_ENABLE_RESETVAL                   (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R13_CRD_ENABLE_MAX                        (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE8_R13_CRD_DITHER_ENABLE_MASK                (0x00000002U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R13_CRD_DITHER_ENABLE_SHIFT               (0x00000001U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R13_CRD_DITHER_ENABLE_RESETVAL            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R13_CRD_DITHER_ENABLE_MAX                 (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE8_R13_NU_MASK                               (0x00000004U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R13_NU_SHIFT                              (0x00000002U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R13_NU_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R13_NU_MAX                                (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE8_R13_CRD_FREQ_OFFSET_TIME_MASK             (0x0000FFF8U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R13_CRD_FREQ_OFFSET_TIME_SHIFT            (0x00000003U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R13_CRD_FREQ_OFFSET_TIME_RESETVAL         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R13_CRD_FREQ_OFFSET_TIME_MAX              (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE8_R13_CRD_NSLOPE_MAG_MASK                   (0xFFFF0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R13_CRD_NSLOPE_MAG_SHIFT                  (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R13_CRD_NSLOPE_MAG_RESETVAL               (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE8_R13_CRD_NSLOPE_MAG_MAX                    (0x0000FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE8_R13_RESETVAL                              (0x00000000U)

/* PROFILE9_R1 */

#define CSL_BSS_PROFILE_RAM_PROFILE9_R1_N_START_CONSTANT_MASK                  (0x7FFFFFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R1_N_START_CONSTANT_SHIFT                 (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R1_N_START_CONSTANT_RESETVAL              (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R1_N_START_CONSTANT_MAX                   (0x7FFFFFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE9_R1_NU_MASK                                (0x80000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R1_NU_SHIFT                               (0x0000001FU)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R1_NU_RESETVAL                            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R1_NU_MAX                                 (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE9_R1_RESETVAL                               (0x00000000U)

/* PROFILE9_R2 */

#define CSL_BSS_PROFILE_RAM_PROFILE9_R2_N_SLOPE_CONSTANT_MASK                  (0x0000FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R2_N_SLOPE_CONSTANT_SHIFT                 (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R2_N_SLOPE_CONSTANT_RESETVAL              (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R2_N_SLOPE_CONSTANT_MAX                   (0x0000FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE9_R2_FAST_RESET_END_TIME_MASK               (0x1FFF0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R2_FAST_RESET_END_TIME_SHIFT              (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R2_FAST_RESET_END_TIME_RESETVAL           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R2_FAST_RESET_END_TIME_MAX                (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE9_R2_DISABLE_FAST_RESET_MASK                (0x20000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R2_DISABLE_FAST_RESET_SHIFT               (0x0000001DU)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R2_DISABLE_FAST_RESET_RESETVAL            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R2_DISABLE_FAST_RESET_MAX                 (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE9_R2_NU_MASK                                (0xC0000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R2_NU_SHIFT                               (0x0000001EU)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R2_NU_RESETVAL                            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R2_NU_MAX                                 (0x00000003U)

#define CSL_BSS_PROFILE_RAM_PROFILE9_R2_RESETVAL                               (0x00000000U)

/* PROFILE9_R3 */

#define CSL_BSS_PROFILE_RAM_PROFILE9_R3_IDLE_TIME_CONSTANT_MASK                (0x0007FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R3_IDLE_TIME_CONSTANT_SHIFT               (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R3_IDLE_TIME_CONSTANT_RESETVAL            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R3_IDLE_TIME_CONSTANT_MAX                 (0x0007FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE9_R3_NU1_MASK                               (0x00F80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R3_NU1_SHIFT                              (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R3_NU1_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R3_NU1_MAX                                (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE9_R3_DISABLE_MONITORING_MASK                (0x01000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R3_DISABLE_MONITORING_SHIFT               (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R3_DISABLE_MONITORING_RESETVAL            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R3_DISABLE_MONITORING_MAX                 (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE9_R3_NU2_MASK                               (0xFE000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R3_NU2_SHIFT                              (0x00000019U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R3_NU2_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R3_NU2_MAX                                (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE9_R3_RESETVAL                               (0x00000000U)

/* PROFILE9_R4 */

#define CSL_BSS_PROFILE_RAM_PROFILE9_R4_ADC_VALID_START_TIME_MASK              (0x0007FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R4_ADC_VALID_START_TIME_SHIFT             (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R4_ADC_VALID_START_TIME_RESETVAL          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R4_ADC_VALID_START_TIME_MAX               (0x0007FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE9_R4_NU1_MASK                               (0x00F80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R4_NU1_SHIFT                              (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R4_NU1_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R4_NU1_MAX                                (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE9_R4_DONT_TURN_OFF_MASK                     (0x01000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R4_DONT_TURN_OFF_SHIFT                    (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R4_DONT_TURN_OFF_RESETVAL                 (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R4_DONT_TURN_OFF_MAX                      (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE9_R4_NU2_MASK                               (0xFE000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R4_NU2_SHIFT                              (0x00000019U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R4_NU2_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R4_NU2_MAX                                (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE9_R4_RESETVAL                               (0x00000000U)

/* PROFILE9_R5 */

#define CSL_BSS_PROFILE_RAM_PROFILE9_R5_ADC_SAMPLING_TIME_MASK                 (0x0007FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R5_ADC_SAMPLING_TIME_SHIFT                (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R5_ADC_SAMPLING_TIME_RESETVAL             (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R5_ADC_SAMPLING_TIME_MAX                  (0x0007FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE9_R5_NU_MASK                                (0xFFF80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R5_NU_SHIFT                               (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R5_NU_RESETVAL                            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R5_NU_MAX                                 (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE9_R5_RESETVAL                               (0x00000000U)

/* PROFILE9_R6 */

#define CSL_BSS_PROFILE_RAM_PROFILE9_R6_TX_START_TIME_MASK                     (0x00001FFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R6_TX_START_TIME_SHIFT                    (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R6_TX_START_TIME_RESETVAL                 (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R6_TX_START_TIME_MAX                      (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE9_R6_NU1_MASK                               (0x0000E000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R6_NU1_SHIFT                              (0x0000000DU)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R6_NU1_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R6_NU1_MAX                                (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE9_R6_ENABLE_SEQ_EXTENSION_MASK              (0x00010000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R6_ENABLE_SEQ_EXTENSION_SHIFT             (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R6_ENABLE_SEQ_EXTENSION_RESETVAL          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R6_ENABLE_SEQ_EXTENSION_MAX               (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE9_R6_NU2_MASK                               (0x00FE0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R6_NU2_SHIFT                              (0x00000011U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R6_NU2_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R6_NU2_MAX                                (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE9_R6_VCO_SELECT_MASK                        (0x01000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R6_VCO_SELECT_SHIFT                       (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R6_VCO_SELECT_RESETVAL                    (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R6_VCO_SELECT_MAX                         (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE9_R6_STITCHING_FIRST_HALF_MASK              (0x02000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R6_STITCHING_FIRST_HALF_SHIFT             (0x00000019U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R6_STITCHING_FIRST_HALF_RESETVAL          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R6_STITCHING_FIRST_HALF_MAX               (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE9_R6_NU3_MASK                               (0xFC000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R6_NU3_SHIFT                              (0x0000001AU)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R6_NU3_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R6_NU3_MAX                                (0x0000003FU)

#define CSL_BSS_PROFILE_RAM_PROFILE9_R6_RESETVAL                               (0x00000000U)

/* PROFILE9_R7 */

#define CSL_BSS_PROFILE_RAM_PROFILE9_R7_MONITOR_START_TIME_MASK                (0x00001FFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R7_MONITOR_START_TIME_SHIFT               (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R7_MONITOR_START_TIME_RESETVAL            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R7_MONITOR_START_TIME_MAX                 (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE9_R7_NU1_MASK                               (0x0000E000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R7_NU1_SHIFT                              (0x0000000DU)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R7_NU1_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R7_NU1_MAX                                (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE9_R7_RX_DFE_RESET_TIME_MASK                 (0x1FFF0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R7_RX_DFE_RESET_TIME_SHIFT                (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R7_RX_DFE_RESET_TIME_RESETVAL             (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R7_RX_DFE_RESET_TIME_MAX                  (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE9_R7_NU2_MASK                               (0xE0000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R7_NU2_SHIFT                              (0x0000001DU)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R7_NU2_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R7_NU2_MAX                                (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE9_R7_RESETVAL                               (0x00000000U)

/* PROFILE9_R8 */

#define CSL_BSS_PROFILE_RAM_PROFILE9_R8_RAMP_END_TIME_MASK                     (0x0007FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R8_RAMP_END_TIME_SHIFT                    (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R8_RAMP_END_TIME_RESETVAL                 (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R8_RAMP_END_TIME_MAX                      (0x0007FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE9_R8_NU_MASK                                (0xFFF80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R8_NU_SHIFT                               (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R8_NU_RESETVAL                            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R8_NU_MAX                                 (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE9_R8_RESETVAL                               (0x00000000U)

/* PROFILE9_R9 */

#define CSL_BSS_PROFILE_RAM_PROFILE9_R9_SYNTH_ANA_CTRL_MASK                    (0x0FFFFFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R9_SYNTH_ANA_CTRL_SHIFT                   (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R9_SYNTH_ANA_CTRL_RESETVAL                (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R9_SYNTH_ANA_CTRL_MAX                     (0x0FFFFFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE9_R9_NU_MASK                                (0xF0000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R9_NU_SHIFT                               (0x0000001CU)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R9_NU_RESETVAL                            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R9_NU_MAX                                 (0x0000000FU)

#define CSL_BSS_PROFILE_RAM_PROFILE9_R9_RESETVAL                               (0x00000000U)

/* PROFILE9_R10 */

#define CSL_BSS_PROFILE_RAM_PROFILE9_R10_FREQ_MON_THRESHOLD_MASK               (0x001FFFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R10_FREQ_MON_THRESHOLD_SHIFT              (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R10_FREQ_MON_THRESHOLD_RESETVAL           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R10_FREQ_MON_THRESHOLD_MAX                (0x001FFFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE9_R10_NU1_MASK                              (0x00E00000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R10_NU1_SHIFT                             (0x00000015U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R10_NU1_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R10_NU1_MAX                               (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE9_R10_ENABLE_LIN_MEAS_MASK                  (0x01000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R10_ENABLE_LIN_MEAS_SHIFT                 (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R10_ENABLE_LIN_MEAS_RESETVAL              (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R10_ENABLE_LIN_MEAS_MAX                   (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE9_R10_NU2_MASK                              (0xFE000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R10_NU2_SHIFT                             (0x00000019U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R10_NU2_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R10_NU2_MAX                               (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE9_R10_RESETVAL                              (0x00000000U)

/* PROFILE9_R11 */

#define CSL_BSS_PROFILE_RAM_PROFILE9_R11_FREQ_MON_L1_MASK                      (0x0000007FU)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R11_FREQ_MON_L1_SHIFT                     (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R11_FREQ_MON_L1_RESETVAL                  (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R11_FREQ_MON_L1_MAX                       (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE9_R11_NU1_MASK                              (0x00000080U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R11_NU1_SHIFT                             (0x00000007U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R11_NU1_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R11_NU1_MAX                               (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE9_R11_FREQ_MON_L2_MASK                      (0x00007F00U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R11_FREQ_MON_L2_SHIFT                     (0x00000008U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R11_FREQ_MON_L2_RESETVAL                  (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R11_FREQ_MON_L2_MAX                       (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE9_R11_NU2_MASK                              (0x00008000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R11_NU2_SHIFT                             (0x0000000FU)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R11_NU2_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R11_NU2_MAX                               (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE9_R11_FREQ_MON_S1_MASK                      (0x00070000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R11_FREQ_MON_S1_SHIFT                     (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R11_FREQ_MON_S1_RESETVAL                  (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R11_FREQ_MON_S1_MAX                       (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE9_R11_NU3_MASK                              (0x00F80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R11_NU3_SHIFT                             (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R11_NU3_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R11_NU3_MAX                               (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE9_R11_FREQ_MON_S2_MASK                      (0x07000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R11_FREQ_MON_S2_SHIFT                     (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R11_FREQ_MON_S2_RESETVAL                  (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R11_FREQ_MON_S2_MAX                       (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE9_R11_NU4_MASK                              (0xF8000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R11_NU4_SHIFT                             (0x0000001BU)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R11_NU4_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R11_NU4_MAX                               (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE9_R11_RESETVAL                              (0x00000000U)

/* PROFILE9_R12 */

#define CSL_BSS_PROFILE_RAM_PROFILE9_R12_HPF_FAST_INIT_START_TIME_MASK         (0x000007FFU)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R12_HPF_FAST_INIT_START_TIME_SHIFT        (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R12_HPF_FAST_INIT_START_TIME_RESETVAL     (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R12_HPF_FAST_INIT_START_TIME_MAX          (0x000007FFU)

#define CSL_BSS_PROFILE_RAM_PROFILE9_R12_HPF_FAST_INIT_PULSE_WIDTH_MASK        (0x0000F800U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R12_HPF_FAST_INIT_PULSE_WIDTH_SHIFT       (0x0000000BU)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R12_HPF_FAST_INIT_PULSE_WIDTH_RESETVAL    (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R12_HPF_FAST_INIT_PULSE_WIDTH_MAX         (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE9_R12_HPF_FAST_INIT_FEATURE_EN_MASK         (0x00010000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R12_HPF_FAST_INIT_FEATURE_EN_SHIFT        (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R12_HPF_FAST_INIT_FEATURE_EN_RESETVAL     (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R12_HPF_FAST_INIT_FEATURE_EN_MAX          (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE9_R12_NU3_MASK                              (0xFFFE0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R12_NU3_SHIFT                             (0x00000011U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R12_NU3_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R12_NU3_MAX                               (0x00007FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE9_R12_RESETVAL                              (0x00000000U)

/* PROFILE9_R13 */

#define CSL_BSS_PROFILE_RAM_PROFILE9_R13_CRD_ENABLE_MASK                       (0x00000001U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R13_CRD_ENABLE_SHIFT                      (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R13_CRD_ENABLE_RESETVAL                   (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R13_CRD_ENABLE_MAX                        (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE9_R13_CRD_DITHER_ENABLE_MASK                (0x00000002U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R13_CRD_DITHER_ENABLE_SHIFT               (0x00000001U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R13_CRD_DITHER_ENABLE_RESETVAL            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R13_CRD_DITHER_ENABLE_MAX                 (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE9_R13_NU_MASK                               (0x00000004U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R13_NU_SHIFT                              (0x00000002U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R13_NU_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R13_NU_MAX                                (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE9_R13_CRD_FREQ_OFFSET_TIME_MASK             (0x0000FFF8U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R13_CRD_FREQ_OFFSET_TIME_SHIFT            (0x00000003U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R13_CRD_FREQ_OFFSET_TIME_RESETVAL         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R13_CRD_FREQ_OFFSET_TIME_MAX              (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE9_R13_CRD_NSLOPE_MAG_MASK                   (0xFFFF0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R13_CRD_NSLOPE_MAG_SHIFT                  (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R13_CRD_NSLOPE_MAG_RESETVAL               (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE9_R13_CRD_NSLOPE_MAG_MAX                    (0x0000FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE9_R13_RESETVAL                              (0x00000000U)

/* PROFILE10_R1 */

#define CSL_BSS_PROFILE_RAM_PROFILE10_R1_N_START_CONSTANT_MASK                 (0x7FFFFFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R1_N_START_CONSTANT_SHIFT                (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R1_N_START_CONSTANT_RESETVAL             (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R1_N_START_CONSTANT_MAX                  (0x7FFFFFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE10_R1_NU_MASK                               (0x80000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R1_NU_SHIFT                              (0x0000001FU)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R1_NU_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R1_NU_MAX                                (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE10_R1_RESETVAL                              (0x00000000U)

/* PROFILE10_R2 */

#define CSL_BSS_PROFILE_RAM_PROFILE10_R2_N_SLOPE_CONSTANT_MASK                 (0x0000FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R2_N_SLOPE_CONSTANT_SHIFT                (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R2_N_SLOPE_CONSTANT_RESETVAL             (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R2_N_SLOPE_CONSTANT_MAX                  (0x0000FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE10_R2_FAST_RESET_END_TIME_MASK              (0x1FFF0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R2_FAST_RESET_END_TIME_SHIFT             (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R2_FAST_RESET_END_TIME_RESETVAL          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R2_FAST_RESET_END_TIME_MAX               (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE10_R2_DISABLE_FAST_RESET_MASK               (0x20000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R2_DISABLE_FAST_RESET_SHIFT              (0x0000001DU)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R2_DISABLE_FAST_RESET_RESETVAL           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R2_DISABLE_FAST_RESET_MAX                (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE10_R2_NU_MASK                               (0xC0000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R2_NU_SHIFT                              (0x0000001EU)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R2_NU_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R2_NU_MAX                                (0x00000003U)

#define CSL_BSS_PROFILE_RAM_PROFILE10_R2_RESETVAL                              (0x00000000U)

/* PROFILE10_R3 */

#define CSL_BSS_PROFILE_RAM_PROFILE10_R3_IDLE_TIME_CONSTANT_MASK               (0x0007FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R3_IDLE_TIME_CONSTANT_SHIFT              (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R3_IDLE_TIME_CONSTANT_RESETVAL           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R3_IDLE_TIME_CONSTANT_MAX                (0x0007FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE10_R3_NU1_MASK                              (0x00F80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R3_NU1_SHIFT                             (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R3_NU1_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R3_NU1_MAX                               (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE10_R3_DISABLE_MONITORING_MASK               (0x01000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R3_DISABLE_MONITORING_SHIFT              (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R3_DISABLE_MONITORING_RESETVAL           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R3_DISABLE_MONITORING_MAX                (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE10_R3_NU2_MASK                              (0xFE000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R3_NU2_SHIFT                             (0x00000019U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R3_NU2_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R3_NU2_MAX                               (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE10_R3_RESETVAL                              (0x00000000U)

/* PROFILE10_R4 */

#define CSL_BSS_PROFILE_RAM_PROFILE10_R4_ADC_VALID_START_TIME_MASK             (0x0007FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R4_ADC_VALID_START_TIME_SHIFT            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R4_ADC_VALID_START_TIME_RESETVAL         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R4_ADC_VALID_START_TIME_MAX              (0x0007FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE10_R4_NU1_MASK                              (0x00F80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R4_NU1_SHIFT                             (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R4_NU1_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R4_NU1_MAX                               (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE10_R4_DONT_TURN_OFF_MASK                    (0x01000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R4_DONT_TURN_OFF_SHIFT                   (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R4_DONT_TURN_OFF_RESETVAL                (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R4_DONT_TURN_OFF_MAX                     (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE10_R4_NU2_MASK                              (0xFE000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R4_NU2_SHIFT                             (0x00000019U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R4_NU2_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R4_NU2_MAX                               (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE10_R4_RESETVAL                              (0x00000000U)

/* PROFILE10_R5 */

#define CSL_BSS_PROFILE_RAM_PROFILE10_R5_ADC_SAMPLING_TIME_MASK                (0x0007FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R5_ADC_SAMPLING_TIME_SHIFT               (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R5_ADC_SAMPLING_TIME_RESETVAL            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R5_ADC_SAMPLING_TIME_MAX                 (0x0007FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE10_R5_NU_MASK                               (0xFFF80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R5_NU_SHIFT                              (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R5_NU_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R5_NU_MAX                                (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE10_R5_RESETVAL                              (0x00000000U)

/* PROFILE10_R6 */

#define CSL_BSS_PROFILE_RAM_PROFILE10_R6_TX_START_TIME_MASK                    (0x00001FFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R6_TX_START_TIME_SHIFT                   (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R6_TX_START_TIME_RESETVAL                (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R6_TX_START_TIME_MAX                     (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE10_R6_NU1_MASK                              (0x0000E000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R6_NU1_SHIFT                             (0x0000000DU)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R6_NU1_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R6_NU1_MAX                               (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE10_R6_ENABLE_SEQ_EXTENSION_MASK             (0x00010000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R6_ENABLE_SEQ_EXTENSION_SHIFT            (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R6_ENABLE_SEQ_EXTENSION_RESETVAL         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R6_ENABLE_SEQ_EXTENSION_MAX              (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE10_R6_NU2_MASK                              (0x00FE0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R6_NU2_SHIFT                             (0x00000011U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R6_NU2_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R6_NU2_MAX                               (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE10_R6_VCO_SELECT_MASK                       (0x01000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R6_VCO_SELECT_SHIFT                      (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R6_VCO_SELECT_RESETVAL                   (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R6_VCO_SELECT_MAX                        (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE10_R6_STITCHING_FIRST_HALF_MASK             (0x02000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R6_STITCHING_FIRST_HALF_SHIFT            (0x00000019U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R6_STITCHING_FIRST_HALF_RESETVAL         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R6_STITCHING_FIRST_HALF_MAX              (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE10_R6_NU3_MASK                              (0xFC000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R6_NU3_SHIFT                             (0x0000001AU)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R6_NU3_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R6_NU3_MAX                               (0x0000003FU)

#define CSL_BSS_PROFILE_RAM_PROFILE10_R6_RESETVAL                              (0x00000000U)

/* PROFILE10_R7 */

#define CSL_BSS_PROFILE_RAM_PROFILE10_R7_MONITOR_START_TIME_MASK               (0x00001FFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R7_MONITOR_START_TIME_SHIFT              (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R7_MONITOR_START_TIME_RESETVAL           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R7_MONITOR_START_TIME_MAX                (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE10_R7_NU1_MASK                              (0x0000E000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R7_NU1_SHIFT                             (0x0000000DU)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R7_NU1_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R7_NU1_MAX                               (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE10_R7_RX_DFE_RESET_TIME_MASK                (0x1FFF0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R7_RX_DFE_RESET_TIME_SHIFT               (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R7_RX_DFE_RESET_TIME_RESETVAL            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R7_RX_DFE_RESET_TIME_MAX                 (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE10_R7_NU2_MASK                              (0xE0000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R7_NU2_SHIFT                             (0x0000001DU)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R7_NU2_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R7_NU2_MAX                               (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE10_R7_RESETVAL                              (0x00000000U)

/* PROFILE10_R8 */

#define CSL_BSS_PROFILE_RAM_PROFILE10_R8_RAMP_END_TIME_MASK                    (0x0007FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R8_RAMP_END_TIME_SHIFT                   (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R8_RAMP_END_TIME_RESETVAL                (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R8_RAMP_END_TIME_MAX                     (0x0007FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE10_R8_NU_MASK                               (0xFFF80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R8_NU_SHIFT                              (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R8_NU_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R8_NU_MAX                                (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE10_R8_RESETVAL                              (0x00000000U)

/* PROFILE10_R9 */

#define CSL_BSS_PROFILE_RAM_PROFILE10_R9_SYNTH_ANA_CTRL_MASK                   (0x0FFFFFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R9_SYNTH_ANA_CTRL_SHIFT                  (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R9_SYNTH_ANA_CTRL_RESETVAL               (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R9_SYNTH_ANA_CTRL_MAX                    (0x0FFFFFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE10_R9_NU_MASK                               (0xF0000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R9_NU_SHIFT                              (0x0000001CU)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R9_NU_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R9_NU_MAX                                (0x0000000FU)

#define CSL_BSS_PROFILE_RAM_PROFILE10_R9_RESETVAL                              (0x00000000U)

/* PROFILE10_R10 */

#define CSL_BSS_PROFILE_RAM_PROFILE10_R10_FREQ_MON_THRESHOLD_MASK              (0x001FFFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R10_FREQ_MON_THRESHOLD_SHIFT             (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R10_FREQ_MON_THRESHOLD_RESETVAL          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R10_FREQ_MON_THRESHOLD_MAX               (0x001FFFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE10_R10_NU1_MASK                             (0x00E00000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R10_NU1_SHIFT                            (0x00000015U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R10_NU1_RESETVAL                         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R10_NU1_MAX                              (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE10_R10_ENABLE_LIN_MEAS_MASK                 (0x01000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R10_ENABLE_LIN_MEAS_SHIFT                (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R10_ENABLE_LIN_MEAS_RESETVAL             (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R10_ENABLE_LIN_MEAS_MAX                  (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE10_R10_NU2_MASK                             (0xFE000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R10_NU2_SHIFT                            (0x00000019U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R10_NU2_RESETVAL                         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R10_NU2_MAX                              (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE10_R10_RESETVAL                             (0x00000000U)

/* PROFILE10_R11 */

#define CSL_BSS_PROFILE_RAM_PROFILE10_R11_FREQ_MON_L1_MASK                     (0x0000007FU)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R11_FREQ_MON_L1_SHIFT                    (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R11_FREQ_MON_L1_RESETVAL                 (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R11_FREQ_MON_L1_MAX                      (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE10_R11_NU1_MASK                             (0x00000080U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R11_NU1_SHIFT                            (0x00000007U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R11_NU1_RESETVAL                         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R11_NU1_MAX                              (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE10_R11_FREQ_MON_L2_MASK                     (0x00007F00U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R11_FREQ_MON_L2_SHIFT                    (0x00000008U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R11_FREQ_MON_L2_RESETVAL                 (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R11_FREQ_MON_L2_MAX                      (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE10_R11_NU2_MASK                             (0x00008000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R11_NU2_SHIFT                            (0x0000000FU)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R11_NU2_RESETVAL                         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R11_NU2_MAX                              (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE10_R11_FREQ_MON_S1_MASK                     (0x00070000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R11_FREQ_MON_S1_SHIFT                    (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R11_FREQ_MON_S1_RESETVAL                 (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R11_FREQ_MON_S1_MAX                      (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE10_R11_NU3_MASK                             (0x00F80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R11_NU3_SHIFT                            (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R11_NU3_RESETVAL                         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R11_NU3_MAX                              (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE10_R11_FREQ_MON_S2_MASK                     (0x07000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R11_FREQ_MON_S2_SHIFT                    (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R11_FREQ_MON_S2_RESETVAL                 (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R11_FREQ_MON_S2_MAX                      (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE10_R11_NU4_MASK                             (0xF8000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R11_NU4_SHIFT                            (0x0000001BU)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R11_NU4_RESETVAL                         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R11_NU4_MAX                              (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE10_R11_RESETVAL                             (0x00000000U)

/* PROFILE10_R12 */

#define CSL_BSS_PROFILE_RAM_PROFILE10_R12_HPF_FAST_INIT_START_TIME_MASK        (0x000007FFU)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R12_HPF_FAST_INIT_START_TIME_SHIFT       (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R12_HPF_FAST_INIT_START_TIME_RESETVAL    (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R12_HPF_FAST_INIT_START_TIME_MAX         (0x000007FFU)

#define CSL_BSS_PROFILE_RAM_PROFILE10_R12_HPF_FAST_INIT_PULSE_WIDTH_MASK       (0x0000F800U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R12_HPF_FAST_INIT_PULSE_WIDTH_SHIFT      (0x0000000BU)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R12_HPF_FAST_INIT_PULSE_WIDTH_RESETVAL   (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R12_HPF_FAST_INIT_PULSE_WIDTH_MAX        (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE10_R12_HPF_FAST_INIT_FEATURE_EN_MASK        (0x00010000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R12_HPF_FAST_INIT_FEATURE_EN_SHIFT       (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R12_HPF_FAST_INIT_FEATURE_EN_RESETVAL    (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R12_HPF_FAST_INIT_FEATURE_EN_MAX         (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE10_R12_NU3_MASK                             (0xFFFE0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R12_NU3_SHIFT                            (0x00000011U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R12_NU3_RESETVAL                         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R12_NU3_MAX                              (0x00007FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE10_R12_RESETVAL                             (0x00000000U)

/* PROFILE10_R13 */

#define CSL_BSS_PROFILE_RAM_PROFILE10_R13_CRD_ENABLE_MASK                      (0x00000001U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R13_CRD_ENABLE_SHIFT                     (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R13_CRD_ENABLE_RESETVAL                  (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R13_CRD_ENABLE_MAX                       (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE10_R13_CRD_DITHER_ENABLE_MASK               (0x00000002U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R13_CRD_DITHER_ENABLE_SHIFT              (0x00000001U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R13_CRD_DITHER_ENABLE_RESETVAL           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R13_CRD_DITHER_ENABLE_MAX                (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE10_R13_NU_MASK                              (0x00000004U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R13_NU_SHIFT                             (0x00000002U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R13_NU_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R13_NU_MAX                               (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE10_R13_CRD_FREQ_OFFSET_TIME_MASK            (0x0000FFF8U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R13_CRD_FREQ_OFFSET_TIME_SHIFT           (0x00000003U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R13_CRD_FREQ_OFFSET_TIME_RESETVAL        (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R13_CRD_FREQ_OFFSET_TIME_MAX             (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE10_R13_CRD_NSLOPE_MAG_MASK                  (0xFFFF0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R13_CRD_NSLOPE_MAG_SHIFT                 (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R13_CRD_NSLOPE_MAG_RESETVAL              (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE10_R13_CRD_NSLOPE_MAG_MAX                   (0x0000FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE10_R13_RESETVAL                             (0x00000000U)

/* PROFILE11_R1 */

#define CSL_BSS_PROFILE_RAM_PROFILE11_R1_N_START_CONSTANT_MASK                 (0x7FFFFFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R1_N_START_CONSTANT_SHIFT                (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R1_N_START_CONSTANT_RESETVAL             (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R1_N_START_CONSTANT_MAX                  (0x7FFFFFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE11_R1_NU_MASK                               (0x80000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R1_NU_SHIFT                              (0x0000001FU)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R1_NU_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R1_NU_MAX                                (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE11_R1_RESETVAL                              (0x00000000U)

/* PROFILE11_R2 */

#define CSL_BSS_PROFILE_RAM_PROFILE11_R2_N_SLOPE_CONSTANT_MASK                 (0x0000FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R2_N_SLOPE_CONSTANT_SHIFT                (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R2_N_SLOPE_CONSTANT_RESETVAL             (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R2_N_SLOPE_CONSTANT_MAX                  (0x0000FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE11_R2_FAST_RESET_END_TIME_MASK              (0x1FFF0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R2_FAST_RESET_END_TIME_SHIFT             (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R2_FAST_RESET_END_TIME_RESETVAL          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R2_FAST_RESET_END_TIME_MAX               (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE11_R2_DISABLE_FAST_RESET_MASK               (0x20000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R2_DISABLE_FAST_RESET_SHIFT              (0x0000001DU)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R2_DISABLE_FAST_RESET_RESETVAL           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R2_DISABLE_FAST_RESET_MAX                (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE11_R2_NU_MASK                               (0xC0000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R2_NU_SHIFT                              (0x0000001EU)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R2_NU_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R2_NU_MAX                                (0x00000003U)

#define CSL_BSS_PROFILE_RAM_PROFILE11_R2_RESETVAL                              (0x00000000U)

/* PROFILE11_R3 */

#define CSL_BSS_PROFILE_RAM_PROFILE11_R3_IDLE_TIME_CONSTANT_MASK               (0x0007FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R3_IDLE_TIME_CONSTANT_SHIFT              (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R3_IDLE_TIME_CONSTANT_RESETVAL           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R3_IDLE_TIME_CONSTANT_MAX                (0x0007FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE11_R3_NU1_MASK                              (0x00F80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R3_NU1_SHIFT                             (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R3_NU1_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R3_NU1_MAX                               (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE11_R3_DISABLE_MONITORING_MASK               (0x01000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R3_DISABLE_MONITORING_SHIFT              (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R3_DISABLE_MONITORING_RESETVAL           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R3_DISABLE_MONITORING_MAX                (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE11_R3_NU2_MASK                              (0xFE000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R3_NU2_SHIFT                             (0x00000019U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R3_NU2_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R3_NU2_MAX                               (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE11_R3_RESETVAL                              (0x00000000U)

/* PROFILE11_R4 */

#define CSL_BSS_PROFILE_RAM_PROFILE11_R4_ADC_VALID_START_TIME_MASK             (0x0007FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R4_ADC_VALID_START_TIME_SHIFT            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R4_ADC_VALID_START_TIME_RESETVAL         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R4_ADC_VALID_START_TIME_MAX              (0x0007FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE11_R4_NU1_MASK                              (0x00F80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R4_NU1_SHIFT                             (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R4_NU1_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R4_NU1_MAX                               (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE11_R4_DONT_TURN_OFF_MASK                    (0x01000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R4_DONT_TURN_OFF_SHIFT                   (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R4_DONT_TURN_OFF_RESETVAL                (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R4_DONT_TURN_OFF_MAX                     (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE11_R4_NU2_MASK                              (0xFE000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R4_NU2_SHIFT                             (0x00000019U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R4_NU2_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R4_NU2_MAX                               (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE11_R4_RESETVAL                              (0x00000000U)

/* PROFILE11_R5 */

#define CSL_BSS_PROFILE_RAM_PROFILE11_R5_ADC_SAMPLING_TIME_MASK                (0x0007FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R5_ADC_SAMPLING_TIME_SHIFT               (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R5_ADC_SAMPLING_TIME_RESETVAL            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R5_ADC_SAMPLING_TIME_MAX                 (0x0007FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE11_R5_NU_MASK                               (0xFFF80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R5_NU_SHIFT                              (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R5_NU_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R5_NU_MAX                                (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE11_R5_RESETVAL                              (0x00000000U)

/* PROFILE11_R6 */

#define CSL_BSS_PROFILE_RAM_PROFILE11_R6_TX_START_TIME_MASK                    (0x00001FFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R6_TX_START_TIME_SHIFT                   (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R6_TX_START_TIME_RESETVAL                (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R6_TX_START_TIME_MAX                     (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE11_R6_NU1_MASK                              (0x0000E000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R6_NU1_SHIFT                             (0x0000000DU)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R6_NU1_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R6_NU1_MAX                               (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE11_R6_ENABLE_SEQ_EXTENSION_MASK             (0x00010000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R6_ENABLE_SEQ_EXTENSION_SHIFT            (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R6_ENABLE_SEQ_EXTENSION_RESETVAL         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R6_ENABLE_SEQ_EXTENSION_MAX              (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE11_R6_NU2_MASK                              (0x00FE0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R6_NU2_SHIFT                             (0x00000011U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R6_NU2_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R6_NU2_MAX                               (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE11_R6_VCO_SELECT_MASK                       (0x01000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R6_VCO_SELECT_SHIFT                      (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R6_VCO_SELECT_RESETVAL                   (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R6_VCO_SELECT_MAX                        (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE11_R6_STITCHING_FIRST_HALF_MASK             (0x02000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R6_STITCHING_FIRST_HALF_SHIFT            (0x00000019U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R6_STITCHING_FIRST_HALF_RESETVAL         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R6_STITCHING_FIRST_HALF_MAX              (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE11_R6_NU3_MASK                              (0xFC000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R6_NU3_SHIFT                             (0x0000001AU)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R6_NU3_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R6_NU3_MAX                               (0x0000003FU)

#define CSL_BSS_PROFILE_RAM_PROFILE11_R6_RESETVAL                              (0x00000000U)

/* PROFILE11_R7 */

#define CSL_BSS_PROFILE_RAM_PROFILE11_R7_MONITOR_START_TIME_MASK               (0x00001FFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R7_MONITOR_START_TIME_SHIFT              (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R7_MONITOR_START_TIME_RESETVAL           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R7_MONITOR_START_TIME_MAX                (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE11_R7_NU1_MASK                              (0x0000E000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R7_NU1_SHIFT                             (0x0000000DU)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R7_NU1_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R7_NU1_MAX                               (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE11_R7_RX_DFE_RESET_TIME_MASK                (0x1FFF0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R7_RX_DFE_RESET_TIME_SHIFT               (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R7_RX_DFE_RESET_TIME_RESETVAL            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R7_RX_DFE_RESET_TIME_MAX                 (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE11_R7_NU2_MASK                              (0xE0000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R7_NU2_SHIFT                             (0x0000001DU)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R7_NU2_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R7_NU2_MAX                               (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE11_R7_RESETVAL                              (0x00000000U)

/* PROFILE11_R8 */

#define CSL_BSS_PROFILE_RAM_PROFILE11_R8_RAMP_END_TIME_MASK                    (0x0007FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R8_RAMP_END_TIME_SHIFT                   (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R8_RAMP_END_TIME_RESETVAL                (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R8_RAMP_END_TIME_MAX                     (0x0007FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE11_R8_NU_MASK                               (0xFFF80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R8_NU_SHIFT                              (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R8_NU_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R8_NU_MAX                                (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE11_R8_RESETVAL                              (0x00000000U)

/* PROFILE11_R9 */

#define CSL_BSS_PROFILE_RAM_PROFILE11_R9_SYNTH_ANA_CTRL_MASK                   (0x0FFFFFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R9_SYNTH_ANA_CTRL_SHIFT                  (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R9_SYNTH_ANA_CTRL_RESETVAL               (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R9_SYNTH_ANA_CTRL_MAX                    (0x0FFFFFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE11_R9_NU_MASK                               (0xF0000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R9_NU_SHIFT                              (0x0000001CU)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R9_NU_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R9_NU_MAX                                (0x0000000FU)

#define CSL_BSS_PROFILE_RAM_PROFILE11_R9_RESETVAL                              (0x00000000U)

/* PROFILE11_R10 */

#define CSL_BSS_PROFILE_RAM_PROFILE11_R10_FREQ_MON_THRESHOLD_MASK              (0x001FFFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R10_FREQ_MON_THRESHOLD_SHIFT             (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R10_FREQ_MON_THRESHOLD_RESETVAL          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R10_FREQ_MON_THRESHOLD_MAX               (0x001FFFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE11_R10_NU1_MASK                             (0x00E00000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R10_NU1_SHIFT                            (0x00000015U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R10_NU1_RESETVAL                         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R10_NU1_MAX                              (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE11_R10_ENABLE_LIN_MEAS_MASK                 (0x01000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R10_ENABLE_LIN_MEAS_SHIFT                (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R10_ENABLE_LIN_MEAS_RESETVAL             (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R10_ENABLE_LIN_MEAS_MAX                  (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE11_R10_NU2_MASK                             (0xFE000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R10_NU2_SHIFT                            (0x00000019U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R10_NU2_RESETVAL                         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R10_NU2_MAX                              (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE11_R10_RESETVAL                             (0x00000000U)

/* PROFILE11_R11 */

#define CSL_BSS_PROFILE_RAM_PROFILE11_R11_FREQ_MON_L1_MASK                     (0x0000007FU)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R11_FREQ_MON_L1_SHIFT                    (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R11_FREQ_MON_L1_RESETVAL                 (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R11_FREQ_MON_L1_MAX                      (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE11_R11_NU1_MASK                             (0x00000080U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R11_NU1_SHIFT                            (0x00000007U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R11_NU1_RESETVAL                         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R11_NU1_MAX                              (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE11_R11_FREQ_MON_L2_MASK                     (0x00007F00U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R11_FREQ_MON_L2_SHIFT                    (0x00000008U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R11_FREQ_MON_L2_RESETVAL                 (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R11_FREQ_MON_L2_MAX                      (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE11_R11_NU2_MASK                             (0x00008000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R11_NU2_SHIFT                            (0x0000000FU)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R11_NU2_RESETVAL                         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R11_NU2_MAX                              (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE11_R11_FREQ_MON_S1_MASK                     (0x00070000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R11_FREQ_MON_S1_SHIFT                    (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R11_FREQ_MON_S1_RESETVAL                 (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R11_FREQ_MON_S1_MAX                      (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE11_R11_NU3_MASK                             (0x00F80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R11_NU3_SHIFT                            (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R11_NU3_RESETVAL                         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R11_NU3_MAX                              (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE11_R11_FREQ_MON_S2_MASK                     (0x07000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R11_FREQ_MON_S2_SHIFT                    (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R11_FREQ_MON_S2_RESETVAL                 (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R11_FREQ_MON_S2_MAX                      (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE11_R11_NU4_MASK                             (0xF8000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R11_NU4_SHIFT                            (0x0000001BU)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R11_NU4_RESETVAL                         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R11_NU4_MAX                              (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE11_R11_RESETVAL                             (0x00000000U)

/* PROFILE11_R12 */

#define CSL_BSS_PROFILE_RAM_PROFILE11_R12_HPF_FAST_INIT_START_TIME_MASK        (0x000007FFU)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R12_HPF_FAST_INIT_START_TIME_SHIFT       (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R12_HPF_FAST_INIT_START_TIME_RESETVAL    (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R12_HPF_FAST_INIT_START_TIME_MAX         (0x000007FFU)

#define CSL_BSS_PROFILE_RAM_PROFILE11_R12_HPF_FAST_INIT_PULSE_WIDTH_MASK       (0x0000F800U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R12_HPF_FAST_INIT_PULSE_WIDTH_SHIFT      (0x0000000BU)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R12_HPF_FAST_INIT_PULSE_WIDTH_RESETVAL   (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R12_HPF_FAST_INIT_PULSE_WIDTH_MAX        (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE11_R12_HPF_FAST_INIT_FEATURE_EN_MASK        (0x00010000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R12_HPF_FAST_INIT_FEATURE_EN_SHIFT       (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R12_HPF_FAST_INIT_FEATURE_EN_RESETVAL    (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R12_HPF_FAST_INIT_FEATURE_EN_MAX         (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE11_R12_NU3_MASK                             (0xFFFE0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R12_NU3_SHIFT                            (0x00000011U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R12_NU3_RESETVAL                         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R12_NU3_MAX                              (0x00007FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE11_R12_RESETVAL                             (0x00000000U)

/* PROFILE11_R13 */

#define CSL_BSS_PROFILE_RAM_PROFILE11_R13_CRD_ENABLE_MASK                      (0x00000001U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R13_CRD_ENABLE_SHIFT                     (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R13_CRD_ENABLE_RESETVAL                  (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R13_CRD_ENABLE_MAX                       (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE11_R13_CRD_DITHER_ENABLE_MASK               (0x00000002U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R13_CRD_DITHER_ENABLE_SHIFT              (0x00000001U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R13_CRD_DITHER_ENABLE_RESETVAL           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R13_CRD_DITHER_ENABLE_MAX                (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE11_R13_NU_MASK                              (0x00000004U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R13_NU_SHIFT                             (0x00000002U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R13_NU_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R13_NU_MAX                               (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE11_R13_CRD_FREQ_OFFSET_TIME_MASK            (0x0000FFF8U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R13_CRD_FREQ_OFFSET_TIME_SHIFT           (0x00000003U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R13_CRD_FREQ_OFFSET_TIME_RESETVAL        (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R13_CRD_FREQ_OFFSET_TIME_MAX             (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE11_R13_CRD_NSLOPE_MAG_MASK                  (0xFFFF0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R13_CRD_NSLOPE_MAG_SHIFT                 (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R13_CRD_NSLOPE_MAG_RESETVAL              (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE11_R13_CRD_NSLOPE_MAG_MAX                   (0x0000FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE11_R13_RESETVAL                             (0x00000000U)

/* PROFILE12_R1 */

#define CSL_BSS_PROFILE_RAM_PROFILE12_R1_N_START_CONSTANT_MASK                 (0x7FFFFFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R1_N_START_CONSTANT_SHIFT                (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R1_N_START_CONSTANT_RESETVAL             (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R1_N_START_CONSTANT_MAX                  (0x7FFFFFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE12_R1_NU_MASK                               (0x80000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R1_NU_SHIFT                              (0x0000001FU)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R1_NU_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R1_NU_MAX                                (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE12_R1_RESETVAL                              (0x00000000U)

/* PROFILE12_R2 */

#define CSL_BSS_PROFILE_RAM_PROFILE12_R2_N_SLOPE_CONSTANT_MASK                 (0x0000FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R2_N_SLOPE_CONSTANT_SHIFT                (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R2_N_SLOPE_CONSTANT_RESETVAL             (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R2_N_SLOPE_CONSTANT_MAX                  (0x0000FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE12_R2_FAST_RESET_END_TIME_MASK              (0x1FFF0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R2_FAST_RESET_END_TIME_SHIFT             (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R2_FAST_RESET_END_TIME_RESETVAL          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R2_FAST_RESET_END_TIME_MAX               (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE12_R2_DISABLE_FAST_RESET_MASK               (0x20000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R2_DISABLE_FAST_RESET_SHIFT              (0x0000001DU)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R2_DISABLE_FAST_RESET_RESETVAL           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R2_DISABLE_FAST_RESET_MAX                (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE12_R2_NU_MASK                               (0xC0000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R2_NU_SHIFT                              (0x0000001EU)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R2_NU_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R2_NU_MAX                                (0x00000003U)

#define CSL_BSS_PROFILE_RAM_PROFILE12_R2_RESETVAL                              (0x00000000U)

/* PROFILE12_R3 */

#define CSL_BSS_PROFILE_RAM_PROFILE12_R3_IDLE_TIME_CONSTANT_MASK               (0x0007FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R3_IDLE_TIME_CONSTANT_SHIFT              (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R3_IDLE_TIME_CONSTANT_RESETVAL           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R3_IDLE_TIME_CONSTANT_MAX                (0x0007FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE12_R3_NU1_MASK                              (0x00F80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R3_NU1_SHIFT                             (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R3_NU1_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R3_NU1_MAX                               (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE12_R3_DISABLE_MONITORING_MASK               (0x01000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R3_DISABLE_MONITORING_SHIFT              (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R3_DISABLE_MONITORING_RESETVAL           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R3_DISABLE_MONITORING_MAX                (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE12_R3_NU2_MASK                              (0xFE000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R3_NU2_SHIFT                             (0x00000019U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R3_NU2_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R3_NU2_MAX                               (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE12_R3_RESETVAL                              (0x00000000U)

/* PROFILE12_R4 */

#define CSL_BSS_PROFILE_RAM_PROFILE12_R4_ADC_VALID_START_TIME_MASK             (0x0007FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R4_ADC_VALID_START_TIME_SHIFT            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R4_ADC_VALID_START_TIME_RESETVAL         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R4_ADC_VALID_START_TIME_MAX              (0x0007FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE12_R4_NU1_MASK                              (0x00F80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R4_NU1_SHIFT                             (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R4_NU1_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R4_NU1_MAX                               (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE12_R4_DONT_TURN_OFF_MASK                    (0x01000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R4_DONT_TURN_OFF_SHIFT                   (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R4_DONT_TURN_OFF_RESETVAL                (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R4_DONT_TURN_OFF_MAX                     (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE12_R4_NU2_MASK                              (0xFE000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R4_NU2_SHIFT                             (0x00000019U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R4_NU2_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R4_NU2_MAX                               (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE12_R4_RESETVAL                              (0x00000000U)

/* PROFILE12_R5 */

#define CSL_BSS_PROFILE_RAM_PROFILE12_R5_ADC_SAMPLING_TIME_MASK                (0x0007FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R5_ADC_SAMPLING_TIME_SHIFT               (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R5_ADC_SAMPLING_TIME_RESETVAL            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R5_ADC_SAMPLING_TIME_MAX                 (0x0007FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE12_R5_NU_MASK                               (0xFFF80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R5_NU_SHIFT                              (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R5_NU_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R5_NU_MAX                                (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE12_R5_RESETVAL                              (0x00000000U)

/* PROFILE12_R6 */

#define CSL_BSS_PROFILE_RAM_PROFILE12_R6_TX_START_TIME_MASK                    (0x00001FFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R6_TX_START_TIME_SHIFT                   (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R6_TX_START_TIME_RESETVAL                (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R6_TX_START_TIME_MAX                     (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE12_R6_NU1_MASK                              (0x0000E000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R6_NU1_SHIFT                             (0x0000000DU)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R6_NU1_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R6_NU1_MAX                               (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE12_R6_ENABLE_SEQ_EXTENSION_MASK             (0x00010000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R6_ENABLE_SEQ_EXTENSION_SHIFT            (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R6_ENABLE_SEQ_EXTENSION_RESETVAL         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R6_ENABLE_SEQ_EXTENSION_MAX              (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE12_R6_NU2_MASK                              (0x00FE0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R6_NU2_SHIFT                             (0x00000011U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R6_NU2_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R6_NU2_MAX                               (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE12_R6_VCO_SELECT_MASK                       (0x01000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R6_VCO_SELECT_SHIFT                      (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R6_VCO_SELECT_RESETVAL                   (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R6_VCO_SELECT_MAX                        (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE12_R6_STITCHING_FIRST_HALF_MASK             (0x02000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R6_STITCHING_FIRST_HALF_SHIFT            (0x00000019U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R6_STITCHING_FIRST_HALF_RESETVAL         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R6_STITCHING_FIRST_HALF_MAX              (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE12_R6_NU3_MASK                              (0xFC000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R6_NU3_SHIFT                             (0x0000001AU)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R6_NU3_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R6_NU3_MAX                               (0x0000003FU)

#define CSL_BSS_PROFILE_RAM_PROFILE12_R6_RESETVAL                              (0x00000000U)

/* PROFILE12_R7 */

#define CSL_BSS_PROFILE_RAM_PROFILE12_R7_MONITOR_START_TIME_MASK               (0x00001FFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R7_MONITOR_START_TIME_SHIFT              (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R7_MONITOR_START_TIME_RESETVAL           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R7_MONITOR_START_TIME_MAX                (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE12_R7_NU1_MASK                              (0x0000E000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R7_NU1_SHIFT                             (0x0000000DU)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R7_NU1_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R7_NU1_MAX                               (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE12_R7_RX_DFE_RESET_TIME_MASK                (0x1FFF0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R7_RX_DFE_RESET_TIME_SHIFT               (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R7_RX_DFE_RESET_TIME_RESETVAL            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R7_RX_DFE_RESET_TIME_MAX                 (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE12_R7_NU2_MASK                              (0xE0000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R7_NU2_SHIFT                             (0x0000001DU)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R7_NU2_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R7_NU2_MAX                               (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE12_R7_RESETVAL                              (0x00000000U)

/* PROFILE12_R8 */

#define CSL_BSS_PROFILE_RAM_PROFILE12_R8_RAMP_END_TIME_MASK                    (0x0007FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R8_RAMP_END_TIME_SHIFT                   (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R8_RAMP_END_TIME_RESETVAL                (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R8_RAMP_END_TIME_MAX                     (0x0007FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE12_R8_NU_MASK                               (0xFFF80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R8_NU_SHIFT                              (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R8_NU_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R8_NU_MAX                                (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE12_R8_RESETVAL                              (0x00000000U)

/* PROFILE12_R9 */

#define CSL_BSS_PROFILE_RAM_PROFILE12_R9_SYNTH_ANA_CTRL_MASK                   (0x0FFFFFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R9_SYNTH_ANA_CTRL_SHIFT                  (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R9_SYNTH_ANA_CTRL_RESETVAL               (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R9_SYNTH_ANA_CTRL_MAX                    (0x0FFFFFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE12_R9_NU_MASK                               (0xF0000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R9_NU_SHIFT                              (0x0000001CU)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R9_NU_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R9_NU_MAX                                (0x0000000FU)

#define CSL_BSS_PROFILE_RAM_PROFILE12_R9_RESETVAL                              (0x00000000U)

/* PROFILE12_R10 */

#define CSL_BSS_PROFILE_RAM_PROFILE12_R10_FREQ_MON_THRESHOLD_MASK              (0x001FFFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R10_FREQ_MON_THRESHOLD_SHIFT             (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R10_FREQ_MON_THRESHOLD_RESETVAL          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R10_FREQ_MON_THRESHOLD_MAX               (0x001FFFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE12_R10_NU1_MASK                             (0x00E00000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R10_NU1_SHIFT                            (0x00000015U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R10_NU1_RESETVAL                         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R10_NU1_MAX                              (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE12_R10_ENABLE_LIN_MEAS_MASK                 (0x01000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R10_ENABLE_LIN_MEAS_SHIFT                (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R10_ENABLE_LIN_MEAS_RESETVAL             (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R10_ENABLE_LIN_MEAS_MAX                  (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE12_R10_NU2_MASK                             (0xFE000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R10_NU2_SHIFT                            (0x00000019U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R10_NU2_RESETVAL                         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R10_NU2_MAX                              (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE12_R10_RESETVAL                             (0x00000000U)

/* PROFILE12_R11 */

#define CSL_BSS_PROFILE_RAM_PROFILE12_R11_FREQ_MON_L1_MASK                     (0x0000007FU)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R11_FREQ_MON_L1_SHIFT                    (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R11_FREQ_MON_L1_RESETVAL                 (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R11_FREQ_MON_L1_MAX                      (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE12_R11_NU1_MASK                             (0x00000080U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R11_NU1_SHIFT                            (0x00000007U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R11_NU1_RESETVAL                         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R11_NU1_MAX                              (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE12_R11_FREQ_MON_L2_MASK                     (0x00007F00U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R11_FREQ_MON_L2_SHIFT                    (0x00000008U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R11_FREQ_MON_L2_RESETVAL                 (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R11_FREQ_MON_L2_MAX                      (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE12_R11_NU2_MASK                             (0x00008000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R11_NU2_SHIFT                            (0x0000000FU)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R11_NU2_RESETVAL                         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R11_NU2_MAX                              (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE12_R11_FREQ_MON_S1_MASK                     (0x00070000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R11_FREQ_MON_S1_SHIFT                    (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R11_FREQ_MON_S1_RESETVAL                 (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R11_FREQ_MON_S1_MAX                      (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE12_R11_NU3_MASK                             (0x00F80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R11_NU3_SHIFT                            (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R11_NU3_RESETVAL                         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R11_NU3_MAX                              (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE12_R11_FREQ_MON_S2_MASK                     (0x07000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R11_FREQ_MON_S2_SHIFT                    (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R11_FREQ_MON_S2_RESETVAL                 (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R11_FREQ_MON_S2_MAX                      (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE12_R11_NU4_MASK                             (0xF8000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R11_NU4_SHIFT                            (0x0000001BU)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R11_NU4_RESETVAL                         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R11_NU4_MAX                              (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE12_R11_RESETVAL                             (0x00000000U)

/* PROFILE12_R12 */

#define CSL_BSS_PROFILE_RAM_PROFILE12_R12_HPF_FAST_INIT_START_TIME_MASK        (0x000007FFU)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R12_HPF_FAST_INIT_START_TIME_SHIFT       (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R12_HPF_FAST_INIT_START_TIME_RESETVAL    (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R12_HPF_FAST_INIT_START_TIME_MAX         (0x000007FFU)

#define CSL_BSS_PROFILE_RAM_PROFILE12_R12_HPF_FAST_INIT_PULSE_WIDTH_MASK       (0x0000F800U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R12_HPF_FAST_INIT_PULSE_WIDTH_SHIFT      (0x0000000BU)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R12_HPF_FAST_INIT_PULSE_WIDTH_RESETVAL   (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R12_HPF_FAST_INIT_PULSE_WIDTH_MAX        (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE12_R12_HPF_FAST_INIT_FEATURE_EN_MASK        (0x00010000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R12_HPF_FAST_INIT_FEATURE_EN_SHIFT       (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R12_HPF_FAST_INIT_FEATURE_EN_RESETVAL    (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R12_HPF_FAST_INIT_FEATURE_EN_MAX         (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE12_R12_NU3_MASK                             (0xFFFE0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R12_NU3_SHIFT                            (0x00000011U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R12_NU3_RESETVAL                         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R12_NU3_MAX                              (0x00007FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE12_R12_RESETVAL                             (0x00000000U)

/* PROFILE12_R13 */

#define CSL_BSS_PROFILE_RAM_PROFILE12_R13_CRD_ENABLE_MASK                      (0x00000001U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R13_CRD_ENABLE_SHIFT                     (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R13_CRD_ENABLE_RESETVAL                  (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R13_CRD_ENABLE_MAX                       (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE12_R13_CRD_DITHER_ENABLE_MASK               (0x00000002U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R13_CRD_DITHER_ENABLE_SHIFT              (0x00000001U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R13_CRD_DITHER_ENABLE_RESETVAL           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R13_CRD_DITHER_ENABLE_MAX                (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE12_R13_NU_MASK                              (0x00000004U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R13_NU_SHIFT                             (0x00000002U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R13_NU_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R13_NU_MAX                               (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE12_R13_CRD_FREQ_OFFSET_TIME_MASK            (0x0000FFF8U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R13_CRD_FREQ_OFFSET_TIME_SHIFT           (0x00000003U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R13_CRD_FREQ_OFFSET_TIME_RESETVAL        (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R13_CRD_FREQ_OFFSET_TIME_MAX             (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE12_R13_CRD_NSLOPE_MAG_MASK                  (0xFFFF0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R13_CRD_NSLOPE_MAG_SHIFT                 (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R13_CRD_NSLOPE_MAG_RESETVAL              (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE12_R13_CRD_NSLOPE_MAG_MAX                   (0x0000FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE12_R13_RESETVAL                             (0x00000000U)

/* PROFILE13_R1 */

#define CSL_BSS_PROFILE_RAM_PROFILE13_R1_N_START_CONSTANT_MASK                 (0x7FFFFFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R1_N_START_CONSTANT_SHIFT                (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R1_N_START_CONSTANT_RESETVAL             (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R1_N_START_CONSTANT_MAX                  (0x7FFFFFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE13_R1_NU_MASK                               (0x80000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R1_NU_SHIFT                              (0x0000001FU)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R1_NU_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R1_NU_MAX                                (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE13_R1_RESETVAL                              (0x00000000U)

/* PROFILE13_R2 */

#define CSL_BSS_PROFILE_RAM_PROFILE13_R2_N_SLOPE_CONSTANT_MASK                 (0x0000FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R2_N_SLOPE_CONSTANT_SHIFT                (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R2_N_SLOPE_CONSTANT_RESETVAL             (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R2_N_SLOPE_CONSTANT_MAX                  (0x0000FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE13_R2_FAST_RESET_END_TIME_MASK              (0x1FFF0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R2_FAST_RESET_END_TIME_SHIFT             (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R2_FAST_RESET_END_TIME_RESETVAL          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R2_FAST_RESET_END_TIME_MAX               (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE13_R2_DISABLE_FAST_RESET_MASK               (0x20000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R2_DISABLE_FAST_RESET_SHIFT              (0x0000001DU)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R2_DISABLE_FAST_RESET_RESETVAL           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R2_DISABLE_FAST_RESET_MAX                (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE13_R2_NU_MASK                               (0xC0000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R2_NU_SHIFT                              (0x0000001EU)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R2_NU_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R2_NU_MAX                                (0x00000003U)

#define CSL_BSS_PROFILE_RAM_PROFILE13_R2_RESETVAL                              (0x00000000U)

/* PROFILE13_R3 */

#define CSL_BSS_PROFILE_RAM_PROFILE13_R3_IDLE_TIME_CONSTANT_MASK               (0x0007FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R3_IDLE_TIME_CONSTANT_SHIFT              (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R3_IDLE_TIME_CONSTANT_RESETVAL           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R3_IDLE_TIME_CONSTANT_MAX                (0x0007FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE13_R3_NU1_MASK                              (0x00F80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R3_NU1_SHIFT                             (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R3_NU1_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R3_NU1_MAX                               (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE13_R3_DISABLE_MONITORING_MASK               (0x01000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R3_DISABLE_MONITORING_SHIFT              (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R3_DISABLE_MONITORING_RESETVAL           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R3_DISABLE_MONITORING_MAX                (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE13_R3_NU2_MASK                              (0xFE000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R3_NU2_SHIFT                             (0x00000019U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R3_NU2_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R3_NU2_MAX                               (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE13_R3_RESETVAL                              (0x00000000U)

/* PROFILE13_R4 */

#define CSL_BSS_PROFILE_RAM_PROFILE13_R4_ADC_VALID_START_TIME_MASK             (0x0007FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R4_ADC_VALID_START_TIME_SHIFT            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R4_ADC_VALID_START_TIME_RESETVAL         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R4_ADC_VALID_START_TIME_MAX              (0x0007FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE13_R4_NU1_MASK                              (0x00F80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R4_NU1_SHIFT                             (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R4_NU1_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R4_NU1_MAX                               (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE13_R4_DONT_TURN_OFF_MASK                    (0x01000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R4_DONT_TURN_OFF_SHIFT                   (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R4_DONT_TURN_OFF_RESETVAL                (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R4_DONT_TURN_OFF_MAX                     (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE13_R4_NU2_MASK                              (0xFE000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R4_NU2_SHIFT                             (0x00000019U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R4_NU2_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R4_NU2_MAX                               (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE13_R4_RESETVAL                              (0x00000000U)

/* PROFILE13_R5 */

#define CSL_BSS_PROFILE_RAM_PROFILE13_R5_ADC_SAMPLING_TIME_MASK                (0x0007FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R5_ADC_SAMPLING_TIME_SHIFT               (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R5_ADC_SAMPLING_TIME_RESETVAL            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R5_ADC_SAMPLING_TIME_MAX                 (0x0007FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE13_R5_NU_MASK                               (0xFFF80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R5_NU_SHIFT                              (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R5_NU_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R5_NU_MAX                                (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE13_R5_RESETVAL                              (0x00000000U)

/* PROFILE13_R6 */

#define CSL_BSS_PROFILE_RAM_PROFILE13_R6_TX_START_TIME_MASK                    (0x00001FFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R6_TX_START_TIME_SHIFT                   (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R6_TX_START_TIME_RESETVAL                (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R6_TX_START_TIME_MAX                     (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE13_R6_NU1_MASK                              (0x0000E000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R6_NU1_SHIFT                             (0x0000000DU)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R6_NU1_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R6_NU1_MAX                               (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE13_R6_ENABLE_SEQ_EXTENSION_MASK             (0x00010000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R6_ENABLE_SEQ_EXTENSION_SHIFT            (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R6_ENABLE_SEQ_EXTENSION_RESETVAL         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R6_ENABLE_SEQ_EXTENSION_MAX              (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE13_R6_NU2_MASK                              (0x00FE0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R6_NU2_SHIFT                             (0x00000011U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R6_NU2_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R6_NU2_MAX                               (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE13_R6_VCO_SELECT_MASK                       (0x01000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R6_VCO_SELECT_SHIFT                      (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R6_VCO_SELECT_RESETVAL                   (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R6_VCO_SELECT_MAX                        (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE13_R6_STITCHING_FIRST_HALF_MASK             (0x02000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R6_STITCHING_FIRST_HALF_SHIFT            (0x00000019U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R6_STITCHING_FIRST_HALF_RESETVAL         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R6_STITCHING_FIRST_HALF_MAX              (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE13_R6_NU3_MASK                              (0xFC000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R6_NU3_SHIFT                             (0x0000001AU)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R6_NU3_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R6_NU3_MAX                               (0x0000003FU)

#define CSL_BSS_PROFILE_RAM_PROFILE13_R6_RESETVAL                              (0x00000000U)

/* PROFILE13_R7 */

#define CSL_BSS_PROFILE_RAM_PROFILE13_R7_MONITOR_START_TIME_MASK               (0x00001FFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R7_MONITOR_START_TIME_SHIFT              (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R7_MONITOR_START_TIME_RESETVAL           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R7_MONITOR_START_TIME_MAX                (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE13_R7_NU1_MASK                              (0x0000E000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R7_NU1_SHIFT                             (0x0000000DU)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R7_NU1_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R7_NU1_MAX                               (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE13_R7_RX_DFE_RESET_TIME_MASK                (0x1FFF0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R7_RX_DFE_RESET_TIME_SHIFT               (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R7_RX_DFE_RESET_TIME_RESETVAL            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R7_RX_DFE_RESET_TIME_MAX                 (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE13_R7_NU2_MASK                              (0xE0000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R7_NU2_SHIFT                             (0x0000001DU)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R7_NU2_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R7_NU2_MAX                               (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE13_R7_RESETVAL                              (0x00000000U)

/* PROFILE13_R8 */

#define CSL_BSS_PROFILE_RAM_PROFILE13_R8_RAMP_END_TIME_MASK                    (0x0007FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R8_RAMP_END_TIME_SHIFT                   (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R8_RAMP_END_TIME_RESETVAL                (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R8_RAMP_END_TIME_MAX                     (0x0007FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE13_R8_NU_MASK                               (0xFFF80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R8_NU_SHIFT                              (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R8_NU_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R8_NU_MAX                                (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE13_R8_RESETVAL                              (0x00000000U)

/* PROFILE13_R9 */

#define CSL_BSS_PROFILE_RAM_PROFILE13_R9_SYNTH_ANA_CTRL_MASK                   (0x0FFFFFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R9_SYNTH_ANA_CTRL_SHIFT                  (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R9_SYNTH_ANA_CTRL_RESETVAL               (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R9_SYNTH_ANA_CTRL_MAX                    (0x0FFFFFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE13_R9_NU_MASK                               (0xF0000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R9_NU_SHIFT                              (0x0000001CU)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R9_NU_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R9_NU_MAX                                (0x0000000FU)

#define CSL_BSS_PROFILE_RAM_PROFILE13_R9_RESETVAL                              (0x00000000U)

/* PROFILE13_R10 */

#define CSL_BSS_PROFILE_RAM_PROFILE13_R10_FREQ_MON_THRESHOLD_MASK              (0x001FFFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R10_FREQ_MON_THRESHOLD_SHIFT             (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R10_FREQ_MON_THRESHOLD_RESETVAL          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R10_FREQ_MON_THRESHOLD_MAX               (0x001FFFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE13_R10_NU1_MASK                             (0x00E00000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R10_NU1_SHIFT                            (0x00000015U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R10_NU1_RESETVAL                         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R10_NU1_MAX                              (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE13_R10_ENABLE_LIN_MEAS_MASK                 (0x01000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R10_ENABLE_LIN_MEAS_SHIFT                (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R10_ENABLE_LIN_MEAS_RESETVAL             (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R10_ENABLE_LIN_MEAS_MAX                  (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE13_R10_NU2_MASK                             (0xFE000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R10_NU2_SHIFT                            (0x00000019U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R10_NU2_RESETVAL                         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R10_NU2_MAX                              (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE13_R10_RESETVAL                             (0x00000000U)

/* PROFILE13_R11 */

#define CSL_BSS_PROFILE_RAM_PROFILE13_R11_FREQ_MON_L1_MASK                     (0x0000007FU)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R11_FREQ_MON_L1_SHIFT                    (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R11_FREQ_MON_L1_RESETVAL                 (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R11_FREQ_MON_L1_MAX                      (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE13_R11_NU1_MASK                             (0x00000080U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R11_NU1_SHIFT                            (0x00000007U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R11_NU1_RESETVAL                         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R11_NU1_MAX                              (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE13_R11_FREQ_MON_L2_MASK                     (0x00007F00U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R11_FREQ_MON_L2_SHIFT                    (0x00000008U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R11_FREQ_MON_L2_RESETVAL                 (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R11_FREQ_MON_L2_MAX                      (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE13_R11_NU2_MASK                             (0x00008000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R11_NU2_SHIFT                            (0x0000000FU)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R11_NU2_RESETVAL                         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R11_NU2_MAX                              (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE13_R11_FREQ_MON_S1_MASK                     (0x00070000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R11_FREQ_MON_S1_SHIFT                    (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R11_FREQ_MON_S1_RESETVAL                 (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R11_FREQ_MON_S1_MAX                      (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE13_R11_NU3_MASK                             (0x00F80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R11_NU3_SHIFT                            (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R11_NU3_RESETVAL                         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R11_NU3_MAX                              (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE13_R11_FREQ_MON_S2_MASK                     (0x07000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R11_FREQ_MON_S2_SHIFT                    (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R11_FREQ_MON_S2_RESETVAL                 (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R11_FREQ_MON_S2_MAX                      (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE13_R11_NU4_MASK                             (0xF8000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R11_NU4_SHIFT                            (0x0000001BU)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R11_NU4_RESETVAL                         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R11_NU4_MAX                              (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE13_R11_RESETVAL                             (0x00000000U)

/* PROFILE13_R12 */

#define CSL_BSS_PROFILE_RAM_PROFILE13_R12_HPF_FAST_INIT_START_TIME_MASK        (0x000007FFU)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R12_HPF_FAST_INIT_START_TIME_SHIFT       (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R12_HPF_FAST_INIT_START_TIME_RESETVAL    (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R12_HPF_FAST_INIT_START_TIME_MAX         (0x000007FFU)

#define CSL_BSS_PROFILE_RAM_PROFILE13_R12_HPF_FAST_INIT_PULSE_WIDTH_MASK       (0x0000F800U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R12_HPF_FAST_INIT_PULSE_WIDTH_SHIFT      (0x0000000BU)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R12_HPF_FAST_INIT_PULSE_WIDTH_RESETVAL   (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R12_HPF_FAST_INIT_PULSE_WIDTH_MAX        (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE13_R12_HPF_FAST_INIT_FEATURE_EN_MASK        (0x00010000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R12_HPF_FAST_INIT_FEATURE_EN_SHIFT       (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R12_HPF_FAST_INIT_FEATURE_EN_RESETVAL    (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R12_HPF_FAST_INIT_FEATURE_EN_MAX         (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE13_R12_NU3_MASK                             (0xFFFE0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R12_NU3_SHIFT                            (0x00000011U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R12_NU3_RESETVAL                         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R12_NU3_MAX                              (0x00007FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE13_R12_RESETVAL                             (0x00000000U)

/* PROFILE13_R13 */

#define CSL_BSS_PROFILE_RAM_PROFILE13_R13_CRD_ENABLE_MASK                      (0x00000001U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R13_CRD_ENABLE_SHIFT                     (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R13_CRD_ENABLE_RESETVAL                  (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R13_CRD_ENABLE_MAX                       (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE13_R13_CRD_DITHER_ENABLE_MASK               (0x00000002U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R13_CRD_DITHER_ENABLE_SHIFT              (0x00000001U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R13_CRD_DITHER_ENABLE_RESETVAL           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R13_CRD_DITHER_ENABLE_MAX                (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE13_R13_NU_MASK                              (0x00000004U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R13_NU_SHIFT                             (0x00000002U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R13_NU_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R13_NU_MAX                               (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE13_R13_CRD_FREQ_OFFSET_TIME_MASK            (0x0000FFF8U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R13_CRD_FREQ_OFFSET_TIME_SHIFT           (0x00000003U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R13_CRD_FREQ_OFFSET_TIME_RESETVAL        (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R13_CRD_FREQ_OFFSET_TIME_MAX             (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE13_R13_CRD_NSLOPE_MAG_MASK                  (0xFFFF0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R13_CRD_NSLOPE_MAG_SHIFT                 (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R13_CRD_NSLOPE_MAG_RESETVAL              (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE13_R13_CRD_NSLOPE_MAG_MAX                   (0x0000FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE13_R13_RESETVAL                             (0x00000000U)

/* PROFILE14_R1 */

#define CSL_BSS_PROFILE_RAM_PROFILE14_R1_N_START_CONSTANT_MASK                 (0x7FFFFFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R1_N_START_CONSTANT_SHIFT                (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R1_N_START_CONSTANT_RESETVAL             (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R1_N_START_CONSTANT_MAX                  (0x7FFFFFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE14_R1_NU_MASK                               (0x80000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R1_NU_SHIFT                              (0x0000001FU)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R1_NU_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R1_NU_MAX                                (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE14_R1_RESETVAL                              (0x00000000U)

/* PROFILE14_R2 */

#define CSL_BSS_PROFILE_RAM_PROFILE14_R2_N_SLOPE_CONSTANT_MASK                 (0x0000FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R2_N_SLOPE_CONSTANT_SHIFT                (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R2_N_SLOPE_CONSTANT_RESETVAL             (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R2_N_SLOPE_CONSTANT_MAX                  (0x0000FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE14_R2_FAST_RESET_END_TIME_MASK              (0x1FFF0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R2_FAST_RESET_END_TIME_SHIFT             (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R2_FAST_RESET_END_TIME_RESETVAL          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R2_FAST_RESET_END_TIME_MAX               (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE14_R2_DISABLE_FAST_RESET_MASK               (0x20000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R2_DISABLE_FAST_RESET_SHIFT              (0x0000001DU)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R2_DISABLE_FAST_RESET_RESETVAL           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R2_DISABLE_FAST_RESET_MAX                (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE14_R2_NU_MASK                               (0xC0000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R2_NU_SHIFT                              (0x0000001EU)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R2_NU_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R2_NU_MAX                                (0x00000003U)

#define CSL_BSS_PROFILE_RAM_PROFILE14_R2_RESETVAL                              (0x00000000U)

/* PROFILE14_R3 */

#define CSL_BSS_PROFILE_RAM_PROFILE14_R3_IDLE_TIME_CONSTANT_MASK               (0x0007FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R3_IDLE_TIME_CONSTANT_SHIFT              (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R3_IDLE_TIME_CONSTANT_RESETVAL           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R3_IDLE_TIME_CONSTANT_MAX                (0x0007FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE14_R3_NU1_MASK                              (0x00F80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R3_NU1_SHIFT                             (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R3_NU1_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R3_NU1_MAX                               (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE14_R3_DISABLE_MONITORING_MASK               (0x01000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R3_DISABLE_MONITORING_SHIFT              (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R3_DISABLE_MONITORING_RESETVAL           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R3_DISABLE_MONITORING_MAX                (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE14_R3_NU2_MASK                              (0xFE000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R3_NU2_SHIFT                             (0x00000019U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R3_NU2_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R3_NU2_MAX                               (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE14_R3_RESETVAL                              (0x00000000U)

/* PROFILE14_R4 */

#define CSL_BSS_PROFILE_RAM_PROFILE14_R4_ADC_VALID_START_TIME_MASK             (0x0007FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R4_ADC_VALID_START_TIME_SHIFT            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R4_ADC_VALID_START_TIME_RESETVAL         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R4_ADC_VALID_START_TIME_MAX              (0x0007FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE14_R4_NU1_MASK                              (0x00F80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R4_NU1_SHIFT                             (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R4_NU1_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R4_NU1_MAX                               (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE14_R4_DONT_TURN_OFF_MASK                    (0x01000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R4_DONT_TURN_OFF_SHIFT                   (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R4_DONT_TURN_OFF_RESETVAL                (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R4_DONT_TURN_OFF_MAX                     (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE14_R4_NU2_MASK                              (0xFE000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R4_NU2_SHIFT                             (0x00000019U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R4_NU2_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R4_NU2_MAX                               (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE14_R4_RESETVAL                              (0x00000000U)

/* PROFILE14_R5 */

#define CSL_BSS_PROFILE_RAM_PROFILE14_R5_ADC_SAMPLING_TIME_MASK                (0x0007FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R5_ADC_SAMPLING_TIME_SHIFT               (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R5_ADC_SAMPLING_TIME_RESETVAL            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R5_ADC_SAMPLING_TIME_MAX                 (0x0007FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE14_R5_NU_MASK                               (0xFFF80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R5_NU_SHIFT                              (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R5_NU_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R5_NU_MAX                                (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE14_R5_RESETVAL                              (0x00000000U)

/* PROFILE14_R6 */

#define CSL_BSS_PROFILE_RAM_PROFILE14_R6_TX_START_TIME_MASK                    (0x00001FFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R6_TX_START_TIME_SHIFT                   (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R6_TX_START_TIME_RESETVAL                (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R6_TX_START_TIME_MAX                     (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE14_R6_NU1_MASK                              (0x0000E000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R6_NU1_SHIFT                             (0x0000000DU)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R6_NU1_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R6_NU1_MAX                               (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE14_R6_ENABLE_SEQ_EXTENSION_MASK             (0x00010000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R6_ENABLE_SEQ_EXTENSION_SHIFT            (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R6_ENABLE_SEQ_EXTENSION_RESETVAL         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R6_ENABLE_SEQ_EXTENSION_MAX              (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE14_R6_NU2_MASK                              (0x00FE0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R6_NU2_SHIFT                             (0x00000011U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R6_NU2_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R6_NU2_MAX                               (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE14_R6_VCO_SELECT_MASK                       (0x01000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R6_VCO_SELECT_SHIFT                      (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R6_VCO_SELECT_RESETVAL                   (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R6_VCO_SELECT_MAX                        (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE14_R6_STITCHING_FIRST_HALF_MASK             (0x02000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R6_STITCHING_FIRST_HALF_SHIFT            (0x00000019U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R6_STITCHING_FIRST_HALF_RESETVAL         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R6_STITCHING_FIRST_HALF_MAX              (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE14_R6_NU3_MASK                              (0xFC000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R6_NU3_SHIFT                             (0x0000001AU)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R6_NU3_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R6_NU3_MAX                               (0x0000003FU)

#define CSL_BSS_PROFILE_RAM_PROFILE14_R6_RESETVAL                              (0x00000000U)

/* PROFILE14_R7 */

#define CSL_BSS_PROFILE_RAM_PROFILE14_R7_MONITOR_START_TIME_MASK               (0x00001FFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R7_MONITOR_START_TIME_SHIFT              (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R7_MONITOR_START_TIME_RESETVAL           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R7_MONITOR_START_TIME_MAX                (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE14_R7_NU1_MASK                              (0x0000E000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R7_NU1_SHIFT                             (0x0000000DU)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R7_NU1_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R7_NU1_MAX                               (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE14_R7_RX_DFE_RESET_TIME_MASK                (0x1FFF0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R7_RX_DFE_RESET_TIME_SHIFT               (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R7_RX_DFE_RESET_TIME_RESETVAL            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R7_RX_DFE_RESET_TIME_MAX                 (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE14_R7_NU2_MASK                              (0xE0000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R7_NU2_SHIFT                             (0x0000001DU)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R7_NU2_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R7_NU2_MAX                               (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE14_R7_RESETVAL                              (0x00000000U)

/* PROFILE14_R8 */

#define CSL_BSS_PROFILE_RAM_PROFILE14_R8_RAMP_END_TIME_MASK                    (0x0007FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R8_RAMP_END_TIME_SHIFT                   (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R8_RAMP_END_TIME_RESETVAL                (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R8_RAMP_END_TIME_MAX                     (0x0007FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE14_R8_NU_MASK                               (0xFFF80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R8_NU_SHIFT                              (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R8_NU_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R8_NU_MAX                                (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE14_R8_RESETVAL                              (0x00000000U)

/* PROFILE14_R9 */

#define CSL_BSS_PROFILE_RAM_PROFILE14_R9_SYNTH_ANA_CTRL_MASK                   (0x0FFFFFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R9_SYNTH_ANA_CTRL_SHIFT                  (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R9_SYNTH_ANA_CTRL_RESETVAL               (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R9_SYNTH_ANA_CTRL_MAX                    (0x0FFFFFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE14_R9_NU_MASK                               (0xF0000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R9_NU_SHIFT                              (0x0000001CU)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R9_NU_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R9_NU_MAX                                (0x0000000FU)

#define CSL_BSS_PROFILE_RAM_PROFILE14_R9_RESETVAL                              (0x00000000U)

/* PROFILE14_R10 */

#define CSL_BSS_PROFILE_RAM_PROFILE14_R10_FREQ_MON_THRESHOLD_MASK              (0x001FFFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R10_FREQ_MON_THRESHOLD_SHIFT             (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R10_FREQ_MON_THRESHOLD_RESETVAL          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R10_FREQ_MON_THRESHOLD_MAX               (0x001FFFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE14_R10_NU1_MASK                             (0x00E00000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R10_NU1_SHIFT                            (0x00000015U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R10_NU1_RESETVAL                         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R10_NU1_MAX                              (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE14_R10_ENABLE_LIN_MEAS_MASK                 (0x01000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R10_ENABLE_LIN_MEAS_SHIFT                (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R10_ENABLE_LIN_MEAS_RESETVAL             (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R10_ENABLE_LIN_MEAS_MAX                  (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE14_R10_NU2_MASK                             (0xFE000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R10_NU2_SHIFT                            (0x00000019U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R10_NU2_RESETVAL                         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R10_NU2_MAX                              (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE14_R10_RESETVAL                             (0x00000000U)

/* PROFILE14_R11 */

#define CSL_BSS_PROFILE_RAM_PROFILE14_R11_FREQ_MON_L1_MASK                     (0x0000007FU)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R11_FREQ_MON_L1_SHIFT                    (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R11_FREQ_MON_L1_RESETVAL                 (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R11_FREQ_MON_L1_MAX                      (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE14_R11_NU1_MASK                             (0x00000080U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R11_NU1_SHIFT                            (0x00000007U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R11_NU1_RESETVAL                         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R11_NU1_MAX                              (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE14_R11_FREQ_MON_L2_MASK                     (0x00007F00U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R11_FREQ_MON_L2_SHIFT                    (0x00000008U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R11_FREQ_MON_L2_RESETVAL                 (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R11_FREQ_MON_L2_MAX                      (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE14_R11_NU2_MASK                             (0x00008000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R11_NU2_SHIFT                            (0x0000000FU)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R11_NU2_RESETVAL                         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R11_NU2_MAX                              (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE14_R11_FREQ_MON_S1_MASK                     (0x00070000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R11_FREQ_MON_S1_SHIFT                    (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R11_FREQ_MON_S1_RESETVAL                 (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R11_FREQ_MON_S1_MAX                      (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE14_R11_NU3_MASK                             (0x00F80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R11_NU3_SHIFT                            (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R11_NU3_RESETVAL                         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R11_NU3_MAX                              (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE14_R11_FREQ_MON_S2_MASK                     (0x07000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R11_FREQ_MON_S2_SHIFT                    (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R11_FREQ_MON_S2_RESETVAL                 (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R11_FREQ_MON_S2_MAX                      (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE14_R11_NU4_MASK                             (0xF8000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R11_NU4_SHIFT                            (0x0000001BU)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R11_NU4_RESETVAL                         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R11_NU4_MAX                              (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE14_R11_RESETVAL                             (0x00000000U)

/* PROFILE14_R12 */

#define CSL_BSS_PROFILE_RAM_PROFILE14_R12_HPF_FAST_INIT_START_TIME_MASK        (0x000007FFU)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R12_HPF_FAST_INIT_START_TIME_SHIFT       (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R12_HPF_FAST_INIT_START_TIME_RESETVAL    (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R12_HPF_FAST_INIT_START_TIME_MAX         (0x000007FFU)

#define CSL_BSS_PROFILE_RAM_PROFILE14_R12_HPF_FAST_INIT_PULSE_WIDTH_MASK       (0x0000F800U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R12_HPF_FAST_INIT_PULSE_WIDTH_SHIFT      (0x0000000BU)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R12_HPF_FAST_INIT_PULSE_WIDTH_RESETVAL   (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R12_HPF_FAST_INIT_PULSE_WIDTH_MAX        (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE14_R12_HPF_FAST_INIT_FEATURE_EN_MASK        (0x00010000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R12_HPF_FAST_INIT_FEATURE_EN_SHIFT       (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R12_HPF_FAST_INIT_FEATURE_EN_RESETVAL    (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R12_HPF_FAST_INIT_FEATURE_EN_MAX         (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE14_R12_NU3_MASK                             (0xFFFE0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R12_NU3_SHIFT                            (0x00000011U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R12_NU3_RESETVAL                         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R12_NU3_MAX                              (0x00007FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE14_R12_RESETVAL                             (0x00000000U)

/* PROFILE14_R13 */

#define CSL_BSS_PROFILE_RAM_PROFILE14_R13_CRD_ENABLE_MASK                      (0x00000001U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R13_CRD_ENABLE_SHIFT                     (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R13_CRD_ENABLE_RESETVAL                  (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R13_CRD_ENABLE_MAX                       (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE14_R13_CRD_DITHER_ENABLE_MASK               (0x00000002U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R13_CRD_DITHER_ENABLE_SHIFT              (0x00000001U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R13_CRD_DITHER_ENABLE_RESETVAL           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R13_CRD_DITHER_ENABLE_MAX                (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE14_R13_NU_MASK                              (0x00000004U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R13_NU_SHIFT                             (0x00000002U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R13_NU_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R13_NU_MAX                               (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE14_R13_CRD_FREQ_OFFSET_TIME_MASK            (0x0000FFF8U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R13_CRD_FREQ_OFFSET_TIME_SHIFT           (0x00000003U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R13_CRD_FREQ_OFFSET_TIME_RESETVAL        (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R13_CRD_FREQ_OFFSET_TIME_MAX             (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE14_R13_CRD_NSLOPE_MAG_MASK                  (0xFFFF0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R13_CRD_NSLOPE_MAG_SHIFT                 (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R13_CRD_NSLOPE_MAG_RESETVAL              (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE14_R13_CRD_NSLOPE_MAG_MAX                   (0x0000FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE14_R13_RESETVAL                             (0x00000000U)

/* PROFILE15_R1 */

#define CSL_BSS_PROFILE_RAM_PROFILE15_R1_N_START_CONSTANT_MASK                 (0x7FFFFFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R1_N_START_CONSTANT_SHIFT                (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R1_N_START_CONSTANT_RESETVAL             (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R1_N_START_CONSTANT_MAX                  (0x7FFFFFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE15_R1_NU_MASK                               (0x80000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R1_NU_SHIFT                              (0x0000001FU)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R1_NU_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R1_NU_MAX                                (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE15_R1_RESETVAL                              (0x00000000U)

/* PROFILE15_R2 */

#define CSL_BSS_PROFILE_RAM_PROFILE15_R2_N_SLOPE_CONSTANT_MASK                 (0x0000FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R2_N_SLOPE_CONSTANT_SHIFT                (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R2_N_SLOPE_CONSTANT_RESETVAL             (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R2_N_SLOPE_CONSTANT_MAX                  (0x0000FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE15_R2_FAST_RESET_END_TIME_MASK              (0x1FFF0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R2_FAST_RESET_END_TIME_SHIFT             (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R2_FAST_RESET_END_TIME_RESETVAL          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R2_FAST_RESET_END_TIME_MAX               (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE15_R2_DISABLE_FAST_RESET_MASK               (0x20000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R2_DISABLE_FAST_RESET_SHIFT              (0x0000001DU)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R2_DISABLE_FAST_RESET_RESETVAL           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R2_DISABLE_FAST_RESET_MAX                (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE15_R2_NU_MASK                               (0xC0000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R2_NU_SHIFT                              (0x0000001EU)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R2_NU_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R2_NU_MAX                                (0x00000003U)

#define CSL_BSS_PROFILE_RAM_PROFILE15_R2_RESETVAL                              (0x00000000U)

/* PROFILE15_R3 */

#define CSL_BSS_PROFILE_RAM_PROFILE15_R3_IDLE_TIME_CONSTANT_MASK               (0x0007FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R3_IDLE_TIME_CONSTANT_SHIFT              (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R3_IDLE_TIME_CONSTANT_RESETVAL           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R3_IDLE_TIME_CONSTANT_MAX                (0x0007FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE15_R3_NU1_MASK                              (0x00F80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R3_NU1_SHIFT                             (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R3_NU1_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R3_NU1_MAX                               (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE15_R3_DISABLE_MONITORING_MASK               (0x01000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R3_DISABLE_MONITORING_SHIFT              (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R3_DISABLE_MONITORING_RESETVAL           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R3_DISABLE_MONITORING_MAX                (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE15_R3_NU2_MASK                              (0xFE000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R3_NU2_SHIFT                             (0x00000019U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R3_NU2_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R3_NU2_MAX                               (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE15_R3_RESETVAL                              (0x00000000U)

/* PROFILE15_R4 */

#define CSL_BSS_PROFILE_RAM_PROFILE15_R4_ADC_VALID_START_TIME_MASK             (0x0007FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R4_ADC_VALID_START_TIME_SHIFT            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R4_ADC_VALID_START_TIME_RESETVAL         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R4_ADC_VALID_START_TIME_MAX              (0x0007FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE15_R4_NU1_MASK                              (0x00F80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R4_NU1_SHIFT                             (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R4_NU1_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R4_NU1_MAX                               (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE15_R4_DONT_TURN_OFF_MASK                    (0x01000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R4_DONT_TURN_OFF_SHIFT                   (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R4_DONT_TURN_OFF_RESETVAL                (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R4_DONT_TURN_OFF_MAX                     (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE15_R4_NU2_MASK                              (0xFE000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R4_NU2_SHIFT                             (0x00000019U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R4_NU2_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R4_NU2_MAX                               (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE15_R4_RESETVAL                              (0x00000000U)

/* PROFILE15_R5 */

#define CSL_BSS_PROFILE_RAM_PROFILE15_R5_ADC_SAMPLING_TIME_MASK                (0x0007FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R5_ADC_SAMPLING_TIME_SHIFT               (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R5_ADC_SAMPLING_TIME_RESETVAL            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R5_ADC_SAMPLING_TIME_MAX                 (0x0007FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE15_R5_NU_MASK                               (0xFFF80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R5_NU_SHIFT                              (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R5_NU_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R5_NU_MAX                                (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE15_R5_RESETVAL                              (0x00000000U)

/* PROFILE15_R6 */

#define CSL_BSS_PROFILE_RAM_PROFILE15_R6_TX_START_TIME_MASK                    (0x00001FFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R6_TX_START_TIME_SHIFT                   (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R6_TX_START_TIME_RESETVAL                (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R6_TX_START_TIME_MAX                     (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE15_R6_NU1_MASK                              (0x0000E000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R6_NU1_SHIFT                             (0x0000000DU)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R6_NU1_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R6_NU1_MAX                               (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE15_R6_ENABLE_SEQ_EXTENSION_MASK             (0x00010000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R6_ENABLE_SEQ_EXTENSION_SHIFT            (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R6_ENABLE_SEQ_EXTENSION_RESETVAL         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R6_ENABLE_SEQ_EXTENSION_MAX              (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE15_R6_NU2_MASK                              (0x00FE0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R6_NU2_SHIFT                             (0x00000011U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R6_NU2_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R6_NU2_MAX                               (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE15_R6_VCO_SELECT_MASK                       (0x01000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R6_VCO_SELECT_SHIFT                      (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R6_VCO_SELECT_RESETVAL                   (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R6_VCO_SELECT_MAX                        (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE15_R6_STITCHING_FIRST_HALF_MASK             (0x02000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R6_STITCHING_FIRST_HALF_SHIFT            (0x00000019U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R6_STITCHING_FIRST_HALF_RESETVAL         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R6_STITCHING_FIRST_HALF_MAX              (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE15_R6_NU3_MASK                              (0xFC000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R6_NU3_SHIFT                             (0x0000001AU)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R6_NU3_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R6_NU3_MAX                               (0x0000003FU)

#define CSL_BSS_PROFILE_RAM_PROFILE15_R6_RESETVAL                              (0x00000000U)

/* PROFILE15_R7 */

#define CSL_BSS_PROFILE_RAM_PROFILE15_R7_MONITOR_START_TIME_MASK               (0x00001FFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R7_MONITOR_START_TIME_SHIFT              (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R7_MONITOR_START_TIME_RESETVAL           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R7_MONITOR_START_TIME_MAX                (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE15_R7_NU1_MASK                              (0x0000E000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R7_NU1_SHIFT                             (0x0000000DU)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R7_NU1_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R7_NU1_MAX                               (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE15_R7_RX_DFE_RESET_TIME_MASK                (0x1FFF0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R7_RX_DFE_RESET_TIME_SHIFT               (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R7_RX_DFE_RESET_TIME_RESETVAL            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R7_RX_DFE_RESET_TIME_MAX                 (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE15_R7_NU2_MASK                              (0xE0000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R7_NU2_SHIFT                             (0x0000001DU)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R7_NU2_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R7_NU2_MAX                               (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE15_R7_RESETVAL                              (0x00000000U)

/* PROFILE15_R8 */

#define CSL_BSS_PROFILE_RAM_PROFILE15_R8_RAMP_END_TIME_MASK                    (0x0007FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R8_RAMP_END_TIME_SHIFT                   (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R8_RAMP_END_TIME_RESETVAL                (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R8_RAMP_END_TIME_MAX                     (0x0007FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE15_R8_NU_MASK                               (0xFFF80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R8_NU_SHIFT                              (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R8_NU_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R8_NU_MAX                                (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE15_R8_RESETVAL                              (0x00000000U)

/* PROFILE15_R9 */

#define CSL_BSS_PROFILE_RAM_PROFILE15_R9_SYNTH_ANA_CTRL_MASK                   (0x0FFFFFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R9_SYNTH_ANA_CTRL_SHIFT                  (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R9_SYNTH_ANA_CTRL_RESETVAL               (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R9_SYNTH_ANA_CTRL_MAX                    (0x0FFFFFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE15_R9_NU_MASK                               (0xF0000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R9_NU_SHIFT                              (0x0000001CU)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R9_NU_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R9_NU_MAX                                (0x0000000FU)

#define CSL_BSS_PROFILE_RAM_PROFILE15_R9_RESETVAL                              (0x00000000U)

/* PROFILE15_R10 */

#define CSL_BSS_PROFILE_RAM_PROFILE15_R10_FREQ_MON_THRESHOLD_MASK              (0x001FFFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R10_FREQ_MON_THRESHOLD_SHIFT             (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R10_FREQ_MON_THRESHOLD_RESETVAL          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R10_FREQ_MON_THRESHOLD_MAX               (0x001FFFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE15_R10_NU1_MASK                             (0x00E00000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R10_NU1_SHIFT                            (0x00000015U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R10_NU1_RESETVAL                         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R10_NU1_MAX                              (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE15_R10_ENABLE_LIN_MEAS_MASK                 (0x01000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R10_ENABLE_LIN_MEAS_SHIFT                (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R10_ENABLE_LIN_MEAS_RESETVAL             (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R10_ENABLE_LIN_MEAS_MAX                  (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE15_R10_NU2_MASK                             (0xFE000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R10_NU2_SHIFT                            (0x00000019U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R10_NU2_RESETVAL                         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R10_NU2_MAX                              (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE15_R10_RESETVAL                             (0x00000000U)

/* PROFILE15_R11 */

#define CSL_BSS_PROFILE_RAM_PROFILE15_R11_FREQ_MON_L1_MASK                     (0x0000007FU)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R11_FREQ_MON_L1_SHIFT                    (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R11_FREQ_MON_L1_RESETVAL                 (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R11_FREQ_MON_L1_MAX                      (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE15_R11_NU1_MASK                             (0x00000080U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R11_NU1_SHIFT                            (0x00000007U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R11_NU1_RESETVAL                         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R11_NU1_MAX                              (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE15_R11_FREQ_MON_L2_MASK                     (0x00007F00U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R11_FREQ_MON_L2_SHIFT                    (0x00000008U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R11_FREQ_MON_L2_RESETVAL                 (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R11_FREQ_MON_L2_MAX                      (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE15_R11_NU2_MASK                             (0x00008000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R11_NU2_SHIFT                            (0x0000000FU)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R11_NU2_RESETVAL                         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R11_NU2_MAX                              (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE15_R11_FREQ_MON_S1_MASK                     (0x00070000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R11_FREQ_MON_S1_SHIFT                    (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R11_FREQ_MON_S1_RESETVAL                 (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R11_FREQ_MON_S1_MAX                      (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE15_R11_NU3_MASK                             (0x00F80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R11_NU3_SHIFT                            (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R11_NU3_RESETVAL                         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R11_NU3_MAX                              (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE15_R11_FREQ_MON_S2_MASK                     (0x07000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R11_FREQ_MON_S2_SHIFT                    (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R11_FREQ_MON_S2_RESETVAL                 (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R11_FREQ_MON_S2_MAX                      (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE15_R11_NU4_MASK                             (0xF8000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R11_NU4_SHIFT                            (0x0000001BU)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R11_NU4_RESETVAL                         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R11_NU4_MAX                              (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE15_R11_RESETVAL                             (0x00000000U)

/* PROFILE15_R12 */

#define CSL_BSS_PROFILE_RAM_PROFILE15_R12_HPF_FAST_INIT_START_TIME_MASK        (0x000007FFU)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R12_HPF_FAST_INIT_START_TIME_SHIFT       (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R12_HPF_FAST_INIT_START_TIME_RESETVAL    (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R12_HPF_FAST_INIT_START_TIME_MAX         (0x000007FFU)

#define CSL_BSS_PROFILE_RAM_PROFILE15_R12_HPF_FAST_INIT_PULSE_WIDTH_MASK       (0x0000F800U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R12_HPF_FAST_INIT_PULSE_WIDTH_SHIFT      (0x0000000BU)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R12_HPF_FAST_INIT_PULSE_WIDTH_RESETVAL   (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R12_HPF_FAST_INIT_PULSE_WIDTH_MAX        (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE15_R12_HPF_FAST_INIT_FEATURE_EN_MASK        (0x00010000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R12_HPF_FAST_INIT_FEATURE_EN_SHIFT       (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R12_HPF_FAST_INIT_FEATURE_EN_RESETVAL    (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R12_HPF_FAST_INIT_FEATURE_EN_MAX         (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE15_R12_NU3_MASK                             (0xFFFE0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R12_NU3_SHIFT                            (0x00000011U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R12_NU3_RESETVAL                         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R12_NU3_MAX                              (0x00007FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE15_R12_RESETVAL                             (0x00000000U)

/* PROFILE15_R13 */

#define CSL_BSS_PROFILE_RAM_PROFILE15_R13_CRD_ENABLE_MASK                      (0x00000001U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R13_CRD_ENABLE_SHIFT                     (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R13_CRD_ENABLE_RESETVAL                  (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R13_CRD_ENABLE_MAX                       (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE15_R13_CRD_DITHER_ENABLE_MASK               (0x00000002U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R13_CRD_DITHER_ENABLE_SHIFT              (0x00000001U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R13_CRD_DITHER_ENABLE_RESETVAL           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R13_CRD_DITHER_ENABLE_MAX                (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE15_R13_NU_MASK                              (0x00000004U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R13_NU_SHIFT                             (0x00000002U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R13_NU_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R13_NU_MAX                               (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE15_R13_CRD_FREQ_OFFSET_TIME_MASK            (0x0000FFF8U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R13_CRD_FREQ_OFFSET_TIME_SHIFT           (0x00000003U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R13_CRD_FREQ_OFFSET_TIME_RESETVAL        (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R13_CRD_FREQ_OFFSET_TIME_MAX             (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE15_R13_CRD_NSLOPE_MAG_MASK                  (0xFFFF0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R13_CRD_NSLOPE_MAG_SHIFT                 (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R13_CRD_NSLOPE_MAG_RESETVAL              (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE15_R13_CRD_NSLOPE_MAG_MAX                   (0x0000FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE15_R13_RESETVAL                             (0x00000000U)

/* PROFILE16_R1 */

#define CSL_BSS_PROFILE_RAM_PROFILE16_R1_N_START_CONSTANT_MASK                 (0x7FFFFFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R1_N_START_CONSTANT_SHIFT                (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R1_N_START_CONSTANT_RESETVAL             (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R1_N_START_CONSTANT_MAX                  (0x7FFFFFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE16_R1_NU_MASK                               (0x80000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R1_NU_SHIFT                              (0x0000001FU)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R1_NU_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R1_NU_MAX                                (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE16_R1_RESETVAL                              (0x00000000U)

/* PROFILE16_R2 */

#define CSL_BSS_PROFILE_RAM_PROFILE16_R2_N_SLOPE_CONSTANT_MASK                 (0x0000FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R2_N_SLOPE_CONSTANT_SHIFT                (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R2_N_SLOPE_CONSTANT_RESETVAL             (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R2_N_SLOPE_CONSTANT_MAX                  (0x0000FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE16_R2_FAST_RESET_END_TIME_MASK              (0x1FFF0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R2_FAST_RESET_END_TIME_SHIFT             (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R2_FAST_RESET_END_TIME_RESETVAL          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R2_FAST_RESET_END_TIME_MAX               (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE16_R2_DISABLE_FAST_RESET_MASK               (0x20000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R2_DISABLE_FAST_RESET_SHIFT              (0x0000001DU)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R2_DISABLE_FAST_RESET_RESETVAL           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R2_DISABLE_FAST_RESET_MAX                (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE16_R2_NU_MASK                               (0xC0000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R2_NU_SHIFT                              (0x0000001EU)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R2_NU_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R2_NU_MAX                                (0x00000003U)

#define CSL_BSS_PROFILE_RAM_PROFILE16_R2_RESETVAL                              (0x00000000U)

/* PROFILE16_R3 */

#define CSL_BSS_PROFILE_RAM_PROFILE16_R3_IDLE_TIME_CONSTANT_MASK               (0x0007FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R3_IDLE_TIME_CONSTANT_SHIFT              (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R3_IDLE_TIME_CONSTANT_RESETVAL           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R3_IDLE_TIME_CONSTANT_MAX                (0x0007FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE16_R3_NU1_MASK                              (0x00F80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R3_NU1_SHIFT                             (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R3_NU1_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R3_NU1_MAX                               (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE16_R3_DISABLE_MONITORING_MASK               (0x01000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R3_DISABLE_MONITORING_SHIFT              (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R3_DISABLE_MONITORING_RESETVAL           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R3_DISABLE_MONITORING_MAX                (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE16_R3_NU2_MASK                              (0xFE000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R3_NU2_SHIFT                             (0x00000019U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R3_NU2_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R3_NU2_MAX                               (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE16_R3_RESETVAL                              (0x00000000U)

/* PROFILE16_R4 */

#define CSL_BSS_PROFILE_RAM_PROFILE16_R4_ADC_VALID_START_TIME_MASK             (0x0007FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R4_ADC_VALID_START_TIME_SHIFT            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R4_ADC_VALID_START_TIME_RESETVAL         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R4_ADC_VALID_START_TIME_MAX              (0x0007FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE16_R4_NU1_MASK                              (0x00F80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R4_NU1_SHIFT                             (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R4_NU1_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R4_NU1_MAX                               (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE16_R4_DONT_TURN_OFF_MASK                    (0x01000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R4_DONT_TURN_OFF_SHIFT                   (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R4_DONT_TURN_OFF_RESETVAL                (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R4_DONT_TURN_OFF_MAX                     (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE16_R4_NU2_MASK                              (0xFE000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R4_NU2_SHIFT                             (0x00000019U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R4_NU2_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R4_NU2_MAX                               (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE16_R4_RESETVAL                              (0x00000000U)

/* PROFILE16_R5 */

#define CSL_BSS_PROFILE_RAM_PROFILE16_R5_ADC_SAMPLING_TIME_MASK                (0x0007FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R5_ADC_SAMPLING_TIME_SHIFT               (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R5_ADC_SAMPLING_TIME_RESETVAL            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R5_ADC_SAMPLING_TIME_MAX                 (0x0007FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE16_R5_NU_MASK                               (0xFFF80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R5_NU_SHIFT                              (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R5_NU_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R5_NU_MAX                                (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE16_R5_RESETVAL                              (0x00000000U)

/* PROFILE16_R6 */

#define CSL_BSS_PROFILE_RAM_PROFILE16_R6_TX_START_TIME_MASK                    (0x00001FFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R6_TX_START_TIME_SHIFT                   (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R6_TX_START_TIME_RESETVAL                (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R6_TX_START_TIME_MAX                     (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE16_R6_NU1_MASK                              (0x0000E000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R6_NU1_SHIFT                             (0x0000000DU)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R6_NU1_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R6_NU1_MAX                               (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE16_R6_ENABLE_SEQ_EXTENSION_MASK             (0x00010000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R6_ENABLE_SEQ_EXTENSION_SHIFT            (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R6_ENABLE_SEQ_EXTENSION_RESETVAL         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R6_ENABLE_SEQ_EXTENSION_MAX              (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE16_R6_NU2_MASK                              (0x00FE0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R6_NU2_SHIFT                             (0x00000011U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R6_NU2_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R6_NU2_MAX                               (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE16_R6_VCO_SELECT_MASK                       (0x01000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R6_VCO_SELECT_SHIFT                      (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R6_VCO_SELECT_RESETVAL                   (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R6_VCO_SELECT_MAX                        (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE16_R6_STITCHING_FIRST_HALF_MASK             (0x02000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R6_STITCHING_FIRST_HALF_SHIFT            (0x00000019U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R6_STITCHING_FIRST_HALF_RESETVAL         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R6_STITCHING_FIRST_HALF_MAX              (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE16_R6_NU3_MASK                              (0xFC000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R6_NU3_SHIFT                             (0x0000001AU)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R6_NU3_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R6_NU3_MAX                               (0x0000003FU)

#define CSL_BSS_PROFILE_RAM_PROFILE16_R6_RESETVAL                              (0x00000000U)

/* PROFILE16_R7 */

#define CSL_BSS_PROFILE_RAM_PROFILE16_R7_MONITOR_START_TIME_MASK               (0x00001FFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R7_MONITOR_START_TIME_SHIFT              (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R7_MONITOR_START_TIME_RESETVAL           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R7_MONITOR_START_TIME_MAX                (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE16_R7_NU1_MASK                              (0x0000E000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R7_NU1_SHIFT                             (0x0000000DU)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R7_NU1_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R7_NU1_MAX                               (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE16_R7_RX_DFE_RESET_TIME_MASK                (0x1FFF0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R7_RX_DFE_RESET_TIME_SHIFT               (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R7_RX_DFE_RESET_TIME_RESETVAL            (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R7_RX_DFE_RESET_TIME_MAX                 (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE16_R7_NU2_MASK                              (0xE0000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R7_NU2_SHIFT                             (0x0000001DU)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R7_NU2_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R7_NU2_MAX                               (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE16_R7_RESETVAL                              (0x00000000U)

/* PROFILE16_R8 */

#define CSL_BSS_PROFILE_RAM_PROFILE16_R8_RAMP_END_TIME_MASK                    (0x0007FFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R8_RAMP_END_TIME_SHIFT                   (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R8_RAMP_END_TIME_RESETVAL                (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R8_RAMP_END_TIME_MAX                     (0x0007FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE16_R8_NU_MASK                               (0xFFF80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R8_NU_SHIFT                              (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R8_NU_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R8_NU_MAX                                (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE16_R8_RESETVAL                              (0x00000000U)

/* PROFILE16_R9 */

#define CSL_BSS_PROFILE_RAM_PROFILE16_R9_SYNTH_ANA_CTRL_MASK                   (0x0FFFFFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R9_SYNTH_ANA_CTRL_SHIFT                  (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R9_SYNTH_ANA_CTRL_RESETVAL               (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R9_SYNTH_ANA_CTRL_MAX                    (0x0FFFFFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE16_R9_NU_MASK                               (0xF0000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R9_NU_SHIFT                              (0x0000001CU)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R9_NU_RESETVAL                           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R9_NU_MAX                                (0x0000000FU)

#define CSL_BSS_PROFILE_RAM_PROFILE16_R9_RESETVAL                              (0x00000000U)

/* PROFILE16_R10 */

#define CSL_BSS_PROFILE_RAM_PROFILE16_R10_FREQ_MON_THRESHOLD_MASK              (0x001FFFFFU)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R10_FREQ_MON_THRESHOLD_SHIFT             (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R10_FREQ_MON_THRESHOLD_RESETVAL          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R10_FREQ_MON_THRESHOLD_MAX               (0x001FFFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE16_R10_NU1_MASK                             (0x00E00000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R10_NU1_SHIFT                            (0x00000015U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R10_NU1_RESETVAL                         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R10_NU1_MAX                              (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE16_R10_ENABLE_LIN_MEAS_MASK                 (0x01000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R10_ENABLE_LIN_MEAS_SHIFT                (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R10_ENABLE_LIN_MEAS_RESETVAL             (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R10_ENABLE_LIN_MEAS_MAX                  (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE16_R10_NU2_MASK                             (0xFE000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R10_NU2_SHIFT                            (0x00000019U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R10_NU2_RESETVAL                         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R10_NU2_MAX                              (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE16_R10_RESETVAL                             (0x00000000U)

/* PROFILE16_R11 */

#define CSL_BSS_PROFILE_RAM_PROFILE16_R11_FREQ_MON_L1_MASK                     (0x0000007FU)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R11_FREQ_MON_L1_SHIFT                    (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R11_FREQ_MON_L1_RESETVAL                 (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R11_FREQ_MON_L1_MAX                      (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE16_R11_NU1_MASK                             (0x00000080U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R11_NU1_SHIFT                            (0x00000007U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R11_NU1_RESETVAL                         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R11_NU1_MAX                              (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE16_R11_FREQ_MON_L2_MASK                     (0x00007F00U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R11_FREQ_MON_L2_SHIFT                    (0x00000008U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R11_FREQ_MON_L2_RESETVAL                 (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R11_FREQ_MON_L2_MAX                      (0x0000007FU)

#define CSL_BSS_PROFILE_RAM_PROFILE16_R11_NU2_MASK                             (0x00008000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R11_NU2_SHIFT                            (0x0000000FU)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R11_NU2_RESETVAL                         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R11_NU2_MAX                              (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE16_R11_FREQ_MON_S1_MASK                     (0x00070000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R11_FREQ_MON_S1_SHIFT                    (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R11_FREQ_MON_S1_RESETVAL                 (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R11_FREQ_MON_S1_MAX                      (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE16_R11_NU3_MASK                             (0x00F80000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R11_NU3_SHIFT                            (0x00000013U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R11_NU3_RESETVAL                         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R11_NU3_MAX                              (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE16_R11_FREQ_MON_S2_MASK                     (0x07000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R11_FREQ_MON_S2_SHIFT                    (0x00000018U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R11_FREQ_MON_S2_RESETVAL                 (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R11_FREQ_MON_S2_MAX                      (0x00000007U)

#define CSL_BSS_PROFILE_RAM_PROFILE16_R11_NU4_MASK                             (0xF8000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R11_NU4_SHIFT                            (0x0000001BU)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R11_NU4_RESETVAL                         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R11_NU4_MAX                              (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE16_R11_RESETVAL                             (0x00000000U)

/* PROFILE16_R12 */

#define CSL_BSS_PROFILE_RAM_PROFILE16_R12_HPF_FAST_INIT_START_TIME_MASK        (0x000007FFU)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R12_HPF_FAST_INIT_START_TIME_SHIFT       (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R12_HPF_FAST_INIT_START_TIME_RESETVAL    (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R12_HPF_FAST_INIT_START_TIME_MAX         (0x000007FFU)

#define CSL_BSS_PROFILE_RAM_PROFILE16_R12_HPF_FAST_INIT_PULSE_WIDTH_MASK       (0x0000F800U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R12_HPF_FAST_INIT_PULSE_WIDTH_SHIFT      (0x0000000BU)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R12_HPF_FAST_INIT_PULSE_WIDTH_RESETVAL   (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R12_HPF_FAST_INIT_PULSE_WIDTH_MAX        (0x0000001FU)

#define CSL_BSS_PROFILE_RAM_PROFILE16_R12_HPF_FAST_INIT_FEATURE_EN_MASK        (0x00010000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R12_HPF_FAST_INIT_FEATURE_EN_SHIFT       (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R12_HPF_FAST_INIT_FEATURE_EN_RESETVAL    (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R12_HPF_FAST_INIT_FEATURE_EN_MAX         (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE16_R12_NU3_MASK                             (0xFFFE0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R12_NU3_SHIFT                            (0x00000011U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R12_NU3_RESETVAL                         (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R12_NU3_MAX                              (0x00007FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE16_R12_RESETVAL                             (0x00000000U)

/* PROFILE16_R13 */

#define CSL_BSS_PROFILE_RAM_PROFILE16_R13_CRD_ENABLE_MASK                      (0x00000001U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R13_CRD_ENABLE_SHIFT                     (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R13_CRD_ENABLE_RESETVAL                  (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R13_CRD_ENABLE_MAX                       (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE16_R13_CRD_DITHER_ENABLE_MASK               (0x00000002U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R13_CRD_DITHER_ENABLE_SHIFT              (0x00000001U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R13_CRD_DITHER_ENABLE_RESETVAL           (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R13_CRD_DITHER_ENABLE_MAX                (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE16_R13_NU_MASK                              (0x00000004U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R13_NU_SHIFT                             (0x00000002U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R13_NU_RESETVAL                          (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R13_NU_MAX                               (0x00000001U)

#define CSL_BSS_PROFILE_RAM_PROFILE16_R13_CRD_FREQ_OFFSET_TIME_MASK            (0x0000FFF8U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R13_CRD_FREQ_OFFSET_TIME_SHIFT           (0x00000003U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R13_CRD_FREQ_OFFSET_TIME_RESETVAL        (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R13_CRD_FREQ_OFFSET_TIME_MAX             (0x00001FFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE16_R13_CRD_NSLOPE_MAG_MASK                  (0xFFFF0000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R13_CRD_NSLOPE_MAG_SHIFT                 (0x00000010U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R13_CRD_NSLOPE_MAG_RESETVAL              (0x00000000U)
#define CSL_BSS_PROFILE_RAM_PROFILE16_R13_CRD_NSLOPE_MAG_MAX                   (0x0000FFFFU)

#define CSL_BSS_PROFILE_RAM_PROFILE16_R13_RESETVAL                             (0x00000000U)

#ifdef __cplusplus
}
#endif
#endif
