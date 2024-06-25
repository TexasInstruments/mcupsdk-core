/********************************************************************
 * Copyright (C) 2022 Texas Instruments Incorporated.
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
 *  Name        : cslr_tmu.h
*/
#ifndef CSLR_TMU_H_
#define CSLR_TMU_H_

#ifdef __cplusplus
extern "C"
{
#endif
#include <drivers/hw_include/cslr.h>
#include <stdint.h>

/**************************************************************************
* Hardware Region  :
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t REVISION;
    volatile uint8_t  Resv_64[60];
    volatile uint32_t SINPUF32_R0;
    volatile uint8_t  Resv_72[4];
    volatile uint32_t SINPUF32_R1;
    volatile uint8_t  Resv_80[4];
    volatile uint32_t SINPUF32_R2;
    volatile uint8_t  Resv_88[4];
    volatile uint32_t SINPUF32_R3;
    volatile uint8_t  Resv_96[4];
    volatile uint32_t SINPUF32_R4;
    volatile uint8_t  Resv_104[4];
    volatile uint32_t SINPUF32_R5;
    volatile uint8_t  Resv_112[4];
    volatile uint32_t SINPUF32_R6;
    volatile uint8_t  Resv_120[4];
    volatile uint32_t SINPUF32_R7;
    volatile uint8_t  Resv_128[4];
    volatile uint32_t COSPUF32_R0;
    volatile uint8_t  Resv_136[4];
    volatile uint32_t COSPUF32_R1;
    volatile uint8_t  Resv_144[4];
    volatile uint32_t COSPUF32_R2;
    volatile uint8_t  Resv_152[4];
    volatile uint32_t COSPUF32_R3;
    volatile uint8_t  Resv_160[4];
    volatile uint32_t COSPUF32_R4;
    volatile uint8_t  Resv_168[4];
    volatile uint32_t COSPUF32_R5;
    volatile uint8_t  Resv_176[4];
    volatile uint32_t COSPUF32_R6;
    volatile uint8_t  Resv_184[4];
    volatile uint32_t COSPUF32_R7;
    volatile uint8_t  Resv_192[4];
    volatile uint32_t ATANPUF32_R0;
    volatile uint8_t  Resv_200[4];
    volatile uint32_t ATANPUF32_R1;
    volatile uint8_t  Resv_208[4];
    volatile uint32_t ATANPUF32_R2;
    volatile uint8_t  Resv_216[4];
    volatile uint32_t ATANPUF32_R3;
    volatile uint8_t  Resv_224[4];
    volatile uint32_t ATANPUF32_R4;
    volatile uint8_t  Resv_232[4];
    volatile uint32_t ATANPUF32_R5;
    volatile uint8_t  Resv_240[4];
    volatile uint32_t ATANPUF32_R6;
    volatile uint8_t  Resv_248[4];
    volatile uint32_t ATANPUF32_R7;
    volatile uint8_t  Resv_256[4];
    volatile uint32_t SQRTF32_R0;
    volatile uint8_t  Resv_264[4];
    volatile uint32_t SQRTF32_R1;
    volatile uint8_t  Resv_272[4];
    volatile uint32_t SQRTF32_R2;
    volatile uint8_t  Resv_280[4];
    volatile uint32_t SQRTF32_R3;
    volatile uint8_t  Resv_288[4];
    volatile uint32_t SQRTF32_R4;
    volatile uint8_t  Resv_296[4];
    volatile uint32_t SQRTF32_R5;
    volatile uint8_t  Resv_304[4];
    volatile uint32_t SQRTF32_R6;
    volatile uint8_t  Resv_312[4];
    volatile uint32_t SQRTF32_R7;
    volatile uint8_t  Resv_320[4];
    volatile uint32_t IEXP2F32_R0;
    volatile uint8_t  Resv_328[4];
    volatile uint32_t IEXP2F32_R1;
    volatile uint8_t  Resv_336[4];
    volatile uint32_t IEXP2F32_R2;
    volatile uint8_t  Resv_344[4];
    volatile uint32_t IEXP2F32_R3;
    volatile uint8_t  Resv_352[4];
    volatile uint32_t IEXP2F32_R4;
    volatile uint8_t  Resv_360[4];
    volatile uint32_t IEXP2F32_R5;
    volatile uint8_t  Resv_368[4];
    volatile uint32_t IEXP2F32_R6;
    volatile uint8_t  Resv_376[4];
    volatile uint32_t IEXP2F32_R7;
    volatile uint8_t  Resv_384[4];
    volatile uint32_t LOG2F32_R0;
    volatile uint8_t  Resv_392[4];
    volatile uint32_t LOG2F32_R1;
    volatile uint8_t  Resv_400[4];
    volatile uint32_t LOG2F32_R2;
    volatile uint8_t  Resv_408[4];
    volatile uint32_t LOG2F32_R3;
    volatile uint8_t  Resv_416[4];
    volatile uint32_t LOG2F32_R4;
    volatile uint8_t  Resv_424[4];
    volatile uint32_t LOG2F32_R5;
    volatile uint8_t  Resv_432[4];
    volatile uint32_t LOG2F32_R6;
    volatile uint8_t  Resv_440[4];
    volatile uint32_t LOG2F32_R7;
    volatile uint8_t  Resv_448[4];
    volatile uint32_t QUADF32_X_R0_R1;
    volatile uint8_t  Resv_456[4];
    volatile uint32_t QUADF32_X_R1_R2;
    volatile uint8_t  Resv_464[4];
    volatile uint32_t QUADF32_X_R2_R3;
    volatile uint8_t  Resv_472[4];
    volatile uint32_t QUADF32_X_R3_R4;
    volatile uint8_t  Resv_480[4];
    volatile uint32_t QUADF32_X_R4_R5;
    volatile uint8_t  Resv_488[4];
    volatile uint32_t QUADF32_X_R5_R6;
    volatile uint8_t  Resv_496[4];
    volatile uint32_t QUADF32_X_R6_R7;
    volatile uint8_t  Resv_512[12];
    volatile uint32_t DIVF32_N_R0;
    volatile uint8_t  Resv_520[4];
    volatile uint32_t DIVF32_N_R1;
    volatile uint8_t  Resv_528[4];
    volatile uint32_t DIVF32_N_R2;
    volatile uint8_t  Resv_536[4];
    volatile uint32_t DIVF32_N_R3;
    volatile uint8_t  Resv_544[4];
    volatile uint32_t DIVF32_N_R4;
    volatile uint8_t  Resv_552[4];
    volatile uint32_t DIVF32_N_R5;
    volatile uint8_t  Resv_560[4];
    volatile uint32_t DIVF32_N_R6;
    volatile uint8_t  Resv_568[4];
    volatile uint32_t DIVF32_N_R7;
    volatile uint8_t  Resv_576[4];
    volatile uint32_t QUADF32_DIVF32_OP2;
    volatile uint8_t  Resv_640[60];
    volatile uint32_t RESULT_R0;
    volatile uint8_t  Resv_648[4];
    volatile uint32_t RESULT_R1;
    volatile uint8_t  Resv_656[4];
    volatile uint32_t RESULT_R2;
    volatile uint8_t  Resv_664[4];
    volatile uint32_t RESULT_R3;
    volatile uint8_t  Resv_672[4];
    volatile uint32_t RESULT_R4;
    volatile uint8_t  Resv_680[4];
    volatile uint32_t RESULT_R5;
    volatile uint8_t  Resv_688[4];
    volatile uint32_t RESULT_R6;
    volatile uint8_t  Resv_696[4];
    volatile uint32_t RESULT_R7;
    volatile uint8_t  Resv_704[4];
    volatile uint32_t CSAVE_R0;
    volatile uint8_t  Resv_712[4];
    volatile uint32_t CSAVE_R1;
    volatile uint8_t  Resv_720[4];
    volatile uint32_t CSAVE_R2;
    volatile uint8_t  Resv_728[4];
    volatile uint32_t CSAVE_R3;
    volatile uint8_t  Resv_736[4];
    volatile uint32_t CSAVE_R4;
    volatile uint8_t  Resv_744[4];
    volatile uint32_t CSAVE_R5;
    volatile uint8_t  Resv_752[4];
    volatile uint32_t CSAVE_R6;
    volatile uint8_t  Resv_760[4];
    volatile uint32_t CSAVE_R7;
    volatile uint8_t  Resv_768[4];
    volatile uint32_t CSAVE_OP2;
    volatile uint8_t  Resv_776[4];
    volatile uint32_t CONTEXT_SAVE;
    volatile uint8_t  Resv_784[4];
    volatile uint32_t CONTEXT_RESTORE;
    volatile uint8_t  Resv_840[52];
    volatile uint32_t STF;
    volatile uint8_t  Resv_896[52];
    volatile uint32_t PARITY_TEST;
    volatile uint8_t  Resv_912[12];
    volatile uint32_t LCM_LOCK;
    volatile uint8_t  Resv_928[12];
    volatile uint32_t LCM_COMMIT;
} CSL_tmuRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_TMU_REVISION                                                       (0x00000000U)
#define CSL_TMU_SINPUF32_R0                                                    (0x00000040U)
#define CSL_TMU_SINPUF32_R1                                                    (0x00000048U)
#define CSL_TMU_SINPUF32_R2                                                    (0x00000050U)
#define CSL_TMU_SINPUF32_R3                                                    (0x00000058U)
#define CSL_TMU_SINPUF32_R4                                                    (0x00000060U)
#define CSL_TMU_SINPUF32_R5                                                    (0x00000068U)
#define CSL_TMU_SINPUF32_R6                                                    (0x00000070U)
#define CSL_TMU_SINPUF32_R7                                                    (0x00000078U)
#define CSL_TMU_COSPUF32_R0                                                    (0x00000080U)
#define CSL_TMU_COSPUF32_R1                                                    (0x00000088U)
#define CSL_TMU_COSPUF32_R2                                                    (0x00000090U)
#define CSL_TMU_COSPUF32_R3                                                    (0x00000098U)
#define CSL_TMU_COSPUF32_R4                                                    (0x000000A0U)
#define CSL_TMU_COSPUF32_R5                                                    (0x000000A8U)
#define CSL_TMU_COSPUF32_R6                                                    (0x000000B0U)
#define CSL_TMU_COSPUF32_R7                                                    (0x000000B8U)
#define CSL_TMU_ATANPUF32_R0                                                   (0x000000C0U)
#define CSL_TMU_ATANPUF32_R1                                                   (0x000000C8U)
#define CSL_TMU_ATANPUF32_R2                                                   (0x000000D0U)
#define CSL_TMU_ATANPUF32_R3                                                   (0x000000D8U)
#define CSL_TMU_ATANPUF32_R4                                                   (0x000000E0U)
#define CSL_TMU_ATANPUF32_R5                                                   (0x000000E8U)
#define CSL_TMU_ATANPUF32_R6                                                   (0x000000F0U)
#define CSL_TMU_ATANPUF32_R7                                                   (0x000000F8U)
#define CSL_TMU_SQRTF32_R0                                                     (0x00000100U)
#define CSL_TMU_SQRTF32_R1                                                     (0x00000108U)
#define CSL_TMU_SQRTF32_R2                                                     (0x00000110U)
#define CSL_TMU_SQRTF32_R3                                                     (0x00000118U)
#define CSL_TMU_SQRTF32_R4                                                     (0x00000120U)
#define CSL_TMU_SQRTF32_R5                                                     (0x00000128U)
#define CSL_TMU_SQRTF32_R6                                                     (0x00000130U)
#define CSL_TMU_SQRTF32_R7                                                     (0x00000138U)
#define CSL_TMU_IEXP2F32_R0                                                    (0x00000140U)
#define CSL_TMU_IEXP2F32_R1                                                    (0x00000148U)
#define CSL_TMU_IEXP2F32_R2                                                    (0x00000150U)
#define CSL_TMU_IEXP2F32_R3                                                    (0x00000158U)
#define CSL_TMU_IEXP2F32_R4                                                    (0x00000160U)
#define CSL_TMU_IEXP2F32_R5                                                    (0x00000168U)
#define CSL_TMU_IEXP2F32_R6                                                    (0x00000170U)
#define CSL_TMU_IEXP2F32_R7                                                    (0x00000178U)
#define CSL_TMU_LOG2F32_R0                                                     (0x00000180U)
#define CSL_TMU_LOG2F32_R1                                                     (0x00000188U)
#define CSL_TMU_LOG2F32_R2                                                     (0x00000190U)
#define CSL_TMU_LOG2F32_R3                                                     (0x00000198U)
#define CSL_TMU_LOG2F32_R4                                                     (0x000001A0U)
#define CSL_TMU_LOG2F32_R5                                                     (0x000001A8U)
#define CSL_TMU_LOG2F32_R6                                                     (0x000001B0U)
#define CSL_TMU_LOG2F32_R7                                                     (0x000001B8U)
#define CSL_TMU_QUADF32_X_R0_R1                                                (0x000001C0U)
#define CSL_TMU_QUADF32_X_R1_R2                                                (0x000001C8U)
#define CSL_TMU_QUADF32_X_R2_R3                                                (0x000001D0U)
#define CSL_TMU_QUADF32_X_R3_R4                                                (0x000001D8U)
#define CSL_TMU_QUADF32_X_R4_R5                                                (0x000001E0U)
#define CSL_TMU_QUADF32_X_R5_R6                                                (0x000001E8U)
#define CSL_TMU_QUADF32_X_R6_R7                                                (0x000001F0U)
#define CSL_TMU_DIVF32_N_R0                                                    (0x00000200U)
#define CSL_TMU_DIVF32_N_R1                                                    (0x00000208U)
#define CSL_TMU_DIVF32_N_R2                                                    (0x00000210U)
#define CSL_TMU_DIVF32_N_R3                                                    (0x00000218U)
#define CSL_TMU_DIVF32_N_R4                                                    (0x00000220U)
#define CSL_TMU_DIVF32_N_R5                                                    (0x00000228U)
#define CSL_TMU_DIVF32_N_R6                                                    (0x00000230U)
#define CSL_TMU_DIVF32_N_R7                                                    (0x00000238U)
#define CSL_TMU_QUADF32_DIVF32_OP2                                             (0x00000240U)
#define CSL_TMU_RESULT_R0                                                      (0x00000280U)
#define CSL_TMU_RESULT_R1                                                      (0x00000288U)
#define CSL_TMU_RESULT_R2                                                      (0x00000290U)
#define CSL_TMU_RESULT_R3                                                      (0x00000298U)
#define CSL_TMU_RESULT_R4                                                      (0x000002A0U)
#define CSL_TMU_RESULT_R5                                                      (0x000002A8U)
#define CSL_TMU_RESULT_R6                                                      (0x000002B0U)
#define CSL_TMU_RESULT_R7                                                      (0x000002B8U)
#define CSL_TMU_CSAVE_R0                                                       (0x000002C0U)
#define CSL_TMU_CSAVE_R1                                                       (0x000002C8U)
#define CSL_TMU_CSAVE_R2                                                       (0x000002D0U)
#define CSL_TMU_CSAVE_R3                                                       (0x000002D8U)
#define CSL_TMU_CSAVE_R4                                                       (0x000002E0U)
#define CSL_TMU_CSAVE_R5                                                       (0x000002E8U)
#define CSL_TMU_CSAVE_R6                                                       (0x000002F0U)
#define CSL_TMU_CSAVE_R7                                                       (0x000002F8U)
#define CSL_TMU_CSAVE_OP2                                                      (0x00000300U)
#define CSL_TMU_CONTEXT_SAVE                                                   (0x00000308U)
#define CSL_TMU_CONTEXT_RESTORE                                                (0x00000310U)
#define CSL_TMU_STF                                                            (0x00000348U)
#define CSL_TMU_PARITY_TEST                                                    (0x00000380U)
#define CSL_TMU_LCM_LOCK                                                       (0x00000390U)
#define CSL_TMU_LCM_COMMIT                                                     (0x000003A0U)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* REVISION */

#define CSL_TMU_REVISION_MINOR_MASK                                            (0x0000003FU)
#define CSL_TMU_REVISION_MINOR_SHIFT                                           (0x00000000U)
#define CSL_TMU_REVISION_MINOR_RESETVAL                                        (0x00000000U)
#define CSL_TMU_REVISION_MINOR_MAX                                             (0x0000003FU)

#define CSL_TMU_REVISION_CUSTOM_MASK                                           (0x000000C0U)
#define CSL_TMU_REVISION_CUSTOM_SHIFT                                          (0x00000006U)
#define CSL_TMU_REVISION_CUSTOM_RESETVAL                                       (0x00000000U)
#define CSL_TMU_REVISION_CUSTOM_MAX                                            (0x00000003U)

#define CSL_TMU_REVISION_MAJOR_MASK                                            (0x00000700U)
#define CSL_TMU_REVISION_MAJOR_SHIFT                                           (0x00000008U)
#define CSL_TMU_REVISION_MAJOR_RESETVAL                                        (0x00000000U)
#define CSL_TMU_REVISION_MAJOR_MAX                                             (0x00000007U)

#define CSL_TMU_REVISION_RTL_MASK                                              (0x0000F800U)
#define CSL_TMU_REVISION_RTL_SHIFT                                             (0x0000000BU)
#define CSL_TMU_REVISION_RTL_RESETVAL                                          (0x00000000U)
#define CSL_TMU_REVISION_RTL_MAX                                               (0x0000001FU)

#define CSL_TMU_REVISION_FUNC_MASK                                             (0x0FFF0000U)
#define CSL_TMU_REVISION_FUNC_SHIFT                                            (0x00000010U)
#define CSL_TMU_REVISION_FUNC_RESETVAL                                         (0x00000000U)
#define CSL_TMU_REVISION_FUNC_MAX                                              (0x00000FFFU)

#define CSL_TMU_REVISION_RESERVED_1_MASK                                       (0x30000000U)
#define CSL_TMU_REVISION_RESERVED_1_SHIFT                                      (0x0000001CU)
#define CSL_TMU_REVISION_RESERVED_1_RESETVAL                                   (0x00000000U)
#define CSL_TMU_REVISION_RESERVED_1_MAX                                        (0x00000003U)

#define CSL_TMU_REVISION_SCHEME_MASK                                           (0xC0000000U)
#define CSL_TMU_REVISION_SCHEME_SHIFT                                          (0x0000001EU)
#define CSL_TMU_REVISION_SCHEME_RESETVAL                                       (0x00000001U)
#define CSL_TMU_REVISION_SCHEME_MAX                                            (0x00000003U)

#define CSL_TMU_REVISION_RESETVAL                                              (0x40000000U)

/* SINPUF32_R0 */

#define CSL_TMU_SINPUF32_R0_SINPUF32_R0_MASK                                   (0xFFFFFFFFU)
#define CSL_TMU_SINPUF32_R0_SINPUF32_R0_SHIFT                                  (0x00000000U)
#define CSL_TMU_SINPUF32_R0_SINPUF32_R0_RESETVAL                               (0x00000000U)
#define CSL_TMU_SINPUF32_R0_SINPUF32_R0_MAX                                    (0xFFFFFFFFU)

#define CSL_TMU_SINPUF32_R0_RESETVAL                                           (0x00000000U)

/* SINPUF32_R1 */

#define CSL_TMU_SINPUF32_R1_SINPUF32_R1_MASK                                   (0xFFFFFFFFU)
#define CSL_TMU_SINPUF32_R1_SINPUF32_R1_SHIFT                                  (0x00000000U)
#define CSL_TMU_SINPUF32_R1_SINPUF32_R1_RESETVAL                               (0x00000000U)
#define CSL_TMU_SINPUF32_R1_SINPUF32_R1_MAX                                    (0xFFFFFFFFU)

#define CSL_TMU_SINPUF32_R1_RESETVAL                                           (0x00000000U)

/* SINPUF32_R2 */

#define CSL_TMU_SINPUF32_R2_SINPUF32_R2_MASK                                   (0xFFFFFFFFU)
#define CSL_TMU_SINPUF32_R2_SINPUF32_R2_SHIFT                                  (0x00000000U)
#define CSL_TMU_SINPUF32_R2_SINPUF32_R2_RESETVAL                               (0x00000000U)
#define CSL_TMU_SINPUF32_R2_SINPUF32_R2_MAX                                    (0xFFFFFFFFU)

#define CSL_TMU_SINPUF32_R2_RESETVAL                                           (0x00000000U)

/* SINPUF32_R3 */

#define CSL_TMU_SINPUF32_R3_SINPUF32_R3_MASK                                   (0xFFFFFFFFU)
#define CSL_TMU_SINPUF32_R3_SINPUF32_R3_SHIFT                                  (0x00000000U)
#define CSL_TMU_SINPUF32_R3_SINPUF32_R3_RESETVAL                               (0x00000000U)
#define CSL_TMU_SINPUF32_R3_SINPUF32_R3_MAX                                    (0xFFFFFFFFU)

#define CSL_TMU_SINPUF32_R3_RESETVAL                                           (0x00000000U)

/* SINPUF32_R4 */

#define CSL_TMU_SINPUF32_R4_SINPUF32_R4_MASK                                   (0xFFFFFFFFU)
#define CSL_TMU_SINPUF32_R4_SINPUF32_R4_SHIFT                                  (0x00000000U)
#define CSL_TMU_SINPUF32_R4_SINPUF32_R4_RESETVAL                               (0x00000000U)
#define CSL_TMU_SINPUF32_R4_SINPUF32_R4_MAX                                    (0xFFFFFFFFU)

#define CSL_TMU_SINPUF32_R4_RESETVAL                                           (0x00000000U)

/* SINPUF32_R5 */

#define CSL_TMU_SINPUF32_R5_SINPUF32_R5_MASK                                   (0xFFFFFFFFU)
#define CSL_TMU_SINPUF32_R5_SINPUF32_R5_SHIFT                                  (0x00000000U)
#define CSL_TMU_SINPUF32_R5_SINPUF32_R5_RESETVAL                               (0x00000000U)
#define CSL_TMU_SINPUF32_R5_SINPUF32_R5_MAX                                    (0xFFFFFFFFU)

#define CSL_TMU_SINPUF32_R5_RESETVAL                                           (0x00000000U)

/* SINPUF32_R6 */

#define CSL_TMU_SINPUF32_R6_SINPUF32_R6_MASK                                   (0xFFFFFFFFU)
#define CSL_TMU_SINPUF32_R6_SINPUF32_R6_SHIFT                                  (0x00000000U)
#define CSL_TMU_SINPUF32_R6_SINPUF32_R6_RESETVAL                               (0x00000000U)
#define CSL_TMU_SINPUF32_R6_SINPUF32_R6_MAX                                    (0xFFFFFFFFU)

#define CSL_TMU_SINPUF32_R6_RESETVAL                                           (0x00000000U)

/* SINPUF32_R7 */

#define CSL_TMU_SINPUF32_R7_SINPUF32_R7_MASK                                   (0xFFFFFFFFU)
#define CSL_TMU_SINPUF32_R7_SINPUF32_R7_SHIFT                                  (0x00000000U)
#define CSL_TMU_SINPUF32_R7_SINPUF32_R7_RESETVAL                               (0x00000000U)
#define CSL_TMU_SINPUF32_R7_SINPUF32_R7_MAX                                    (0xFFFFFFFFU)

#define CSL_TMU_SINPUF32_R7_RESETVAL                                           (0x00000000U)

/* COSPUF32_R0 */

#define CSL_TMU_COSPUF32_R0_COSPUF32_R0_MASK                                   (0xFFFFFFFFU)
#define CSL_TMU_COSPUF32_R0_COSPUF32_R0_SHIFT                                  (0x00000000U)
#define CSL_TMU_COSPUF32_R0_COSPUF32_R0_RESETVAL                               (0x00000000U)
#define CSL_TMU_COSPUF32_R0_COSPUF32_R0_MAX                                    (0xFFFFFFFFU)

#define CSL_TMU_COSPUF32_R0_RESETVAL                                           (0x00000000U)

/* COSPUF32_R1 */

#define CSL_TMU_COSPUF32_R1_COSPUF32_R1_MASK                                   (0xFFFFFFFFU)
#define CSL_TMU_COSPUF32_R1_COSPUF32_R1_SHIFT                                  (0x00000000U)
#define CSL_TMU_COSPUF32_R1_COSPUF32_R1_RESETVAL                               (0x00000000U)
#define CSL_TMU_COSPUF32_R1_COSPUF32_R1_MAX                                    (0xFFFFFFFFU)

#define CSL_TMU_COSPUF32_R1_RESETVAL                                           (0x00000000U)

/* COSPUF32_R2 */

#define CSL_TMU_COSPUF32_R2_COSPUF32_R2_MASK                                   (0xFFFFFFFFU)
#define CSL_TMU_COSPUF32_R2_COSPUF32_R2_SHIFT                                  (0x00000000U)
#define CSL_TMU_COSPUF32_R2_COSPUF32_R2_RESETVAL                               (0x00000000U)
#define CSL_TMU_COSPUF32_R2_COSPUF32_R2_MAX                                    (0xFFFFFFFFU)

#define CSL_TMU_COSPUF32_R2_RESETVAL                                           (0x00000000U)

/* COSPUF32_R3 */

#define CSL_TMU_COSPUF32_R3_COSPUF32_R3_MASK                                   (0xFFFFFFFFU)
#define CSL_TMU_COSPUF32_R3_COSPUF32_R3_SHIFT                                  (0x00000000U)
#define CSL_TMU_COSPUF32_R3_COSPUF32_R3_RESETVAL                               (0x00000000U)
#define CSL_TMU_COSPUF32_R3_COSPUF32_R3_MAX                                    (0xFFFFFFFFU)

#define CSL_TMU_COSPUF32_R3_RESETVAL                                           (0x00000000U)

/* COSPUF32_R4 */

#define CSL_TMU_COSPUF32_R4_COSPUF32_R4_MASK                                   (0xFFFFFFFFU)
#define CSL_TMU_COSPUF32_R4_COSPUF32_R4_SHIFT                                  (0x00000000U)
#define CSL_TMU_COSPUF32_R4_COSPUF32_R4_RESETVAL                               (0x00000000U)
#define CSL_TMU_COSPUF32_R4_COSPUF32_R4_MAX                                    (0xFFFFFFFFU)

#define CSL_TMU_COSPUF32_R4_RESETVAL                                           (0x00000000U)

/* COSPUF32_R5 */

#define CSL_TMU_COSPUF32_R5_COSPUF32_R5_MASK                                   (0xFFFFFFFFU)
#define CSL_TMU_COSPUF32_R5_COSPUF32_R5_SHIFT                                  (0x00000000U)
#define CSL_TMU_COSPUF32_R5_COSPUF32_R5_RESETVAL                               (0x00000000U)
#define CSL_TMU_COSPUF32_R5_COSPUF32_R5_MAX                                    (0xFFFFFFFFU)

#define CSL_TMU_COSPUF32_R5_RESETVAL                                           (0x00000000U)

/* COSPUF32_R6 */

#define CSL_TMU_COSPUF32_R6_COSPUF32_R6_MASK                                   (0xFFFFFFFFU)
#define CSL_TMU_COSPUF32_R6_COSPUF32_R6_SHIFT                                  (0x00000000U)
#define CSL_TMU_COSPUF32_R6_COSPUF32_R6_RESETVAL                               (0x00000000U)
#define CSL_TMU_COSPUF32_R6_COSPUF32_R6_MAX                                    (0xFFFFFFFFU)

#define CSL_TMU_COSPUF32_R6_RESETVAL                                           (0x00000000U)

/* COSPUF32_R7 */

#define CSL_TMU_COSPUF32_R7_COSPUF32_R7_MASK                                   (0xFFFFFFFFU)
#define CSL_TMU_COSPUF32_R7_COSPUF32_R7_SHIFT                                  (0x00000000U)
#define CSL_TMU_COSPUF32_R7_COSPUF32_R7_RESETVAL                               (0x00000000U)
#define CSL_TMU_COSPUF32_R7_COSPUF32_R7_MAX                                    (0xFFFFFFFFU)

#define CSL_TMU_COSPUF32_R7_RESETVAL                                           (0x00000000U)

/* ATANPUF32_R0 */

#define CSL_TMU_ATANPUF32_R0_ATANPUF32_R0_MASK                                 (0xFFFFFFFFU)
#define CSL_TMU_ATANPUF32_R0_ATANPUF32_R0_SHIFT                                (0x00000000U)
#define CSL_TMU_ATANPUF32_R0_ATANPUF32_R0_RESETVAL                             (0x00000000U)
#define CSL_TMU_ATANPUF32_R0_ATANPUF32_R0_MAX                                  (0xFFFFFFFFU)

#define CSL_TMU_ATANPUF32_R0_RESETVAL                                          (0x00000000U)

/* ATANPUF32_R1 */

#define CSL_TMU_ATANPUF32_R1_ATANPUF32_R1_MASK                                 (0xFFFFFFFFU)
#define CSL_TMU_ATANPUF32_R1_ATANPUF32_R1_SHIFT                                (0x00000000U)
#define CSL_TMU_ATANPUF32_R1_ATANPUF32_R1_RESETVAL                             (0x00000000U)
#define CSL_TMU_ATANPUF32_R1_ATANPUF32_R1_MAX                                  (0xFFFFFFFFU)

#define CSL_TMU_ATANPUF32_R1_RESETVAL                                          (0x00000000U)

/* ATANPUF32_R2 */

#define CSL_TMU_ATANPUF32_R2_ATANPUF32_R2_MASK                                 (0xFFFFFFFFU)
#define CSL_TMU_ATANPUF32_R2_ATANPUF32_R2_SHIFT                                (0x00000000U)
#define CSL_TMU_ATANPUF32_R2_ATANPUF32_R2_RESETVAL                             (0x00000000U)
#define CSL_TMU_ATANPUF32_R2_ATANPUF32_R2_MAX                                  (0xFFFFFFFFU)

#define CSL_TMU_ATANPUF32_R2_RESETVAL                                          (0x00000000U)

/* ATANPUF32_R3 */

#define CSL_TMU_ATANPUF32_R3_ATANPUF32_R3_MASK                                 (0xFFFFFFFFU)
#define CSL_TMU_ATANPUF32_R3_ATANPUF32_R3_SHIFT                                (0x00000000U)
#define CSL_TMU_ATANPUF32_R3_ATANPUF32_R3_RESETVAL                             (0x00000000U)
#define CSL_TMU_ATANPUF32_R3_ATANPUF32_R3_MAX                                  (0xFFFFFFFFU)

#define CSL_TMU_ATANPUF32_R3_RESETVAL                                          (0x00000000U)

/* ATANPUF32_R4 */

#define CSL_TMU_ATANPUF32_R4_ATANPUF32_R4_MASK                                 (0xFFFFFFFFU)
#define CSL_TMU_ATANPUF32_R4_ATANPUF32_R4_SHIFT                                (0x00000000U)
#define CSL_TMU_ATANPUF32_R4_ATANPUF32_R4_RESETVAL                             (0x00000000U)
#define CSL_TMU_ATANPUF32_R4_ATANPUF32_R4_MAX                                  (0xFFFFFFFFU)

#define CSL_TMU_ATANPUF32_R4_RESETVAL                                          (0x00000000U)

/* ATANPUF32_R5 */

#define CSL_TMU_ATANPUF32_R5_ATANPUF32_R5_MASK                                 (0xFFFFFFFFU)
#define CSL_TMU_ATANPUF32_R5_ATANPUF32_R5_SHIFT                                (0x00000000U)
#define CSL_TMU_ATANPUF32_R5_ATANPUF32_R5_RESETVAL                             (0x00000000U)
#define CSL_TMU_ATANPUF32_R5_ATANPUF32_R5_MAX                                  (0xFFFFFFFFU)

#define CSL_TMU_ATANPUF32_R5_RESETVAL                                          (0x00000000U)

/* ATANPUF32_R6 */

#define CSL_TMU_ATANPUF32_R6_ATANPUF32_R6_MASK                                 (0xFFFFFFFFU)
#define CSL_TMU_ATANPUF32_R6_ATANPUF32_R6_SHIFT                                (0x00000000U)
#define CSL_TMU_ATANPUF32_R6_ATANPUF32_R6_RESETVAL                             (0x00000000U)
#define CSL_TMU_ATANPUF32_R6_ATANPUF32_R6_MAX                                  (0xFFFFFFFFU)

#define CSL_TMU_ATANPUF32_R6_RESETVAL                                          (0x00000000U)

/* ATANPUF32_R7 */

#define CSL_TMU_ATANPUF32_R7_ATANPUF32_R7_MASK                                 (0xFFFFFFFFU)
#define CSL_TMU_ATANPUF32_R7_ATANPUF32_R7_SHIFT                                (0x00000000U)
#define CSL_TMU_ATANPUF32_R7_ATANPUF32_R7_RESETVAL                             (0x00000000U)
#define CSL_TMU_ATANPUF32_R7_ATANPUF32_R7_MAX                                  (0xFFFFFFFFU)

#define CSL_TMU_ATANPUF32_R7_RESETVAL                                          (0x00000000U)

/* SQRTF32_R0 */

#define CSL_TMU_SQRTF32_R0_SQRTF32_R0_MASK                                     (0xFFFFFFFFU)
#define CSL_TMU_SQRTF32_R0_SQRTF32_R0_SHIFT                                    (0x00000000U)
#define CSL_TMU_SQRTF32_R0_SQRTF32_R0_RESETVAL                                 (0x00000000U)
#define CSL_TMU_SQRTF32_R0_SQRTF32_R0_MAX                                      (0xFFFFFFFFU)

#define CSL_TMU_SQRTF32_R0_RESETVAL                                            (0x00000000U)

/* SQRTF32_R1 */

#define CSL_TMU_SQRTF32_R1_SQRTF32_R1_MASK                                     (0xFFFFFFFFU)
#define CSL_TMU_SQRTF32_R1_SQRTF32_R1_SHIFT                                    (0x00000000U)
#define CSL_TMU_SQRTF32_R1_SQRTF32_R1_RESETVAL                                 (0x00000000U)
#define CSL_TMU_SQRTF32_R1_SQRTF32_R1_MAX                                      (0xFFFFFFFFU)

#define CSL_TMU_SQRTF32_R1_RESETVAL                                            (0x00000000U)

/* SQRTF32_R2 */

#define CSL_TMU_SQRTF32_R2_SQRTF32_R2_MASK                                     (0xFFFFFFFFU)
#define CSL_TMU_SQRTF32_R2_SQRTF32_R2_SHIFT                                    (0x00000000U)
#define CSL_TMU_SQRTF32_R2_SQRTF32_R2_RESETVAL                                 (0x00000000U)
#define CSL_TMU_SQRTF32_R2_SQRTF32_R2_MAX                                      (0xFFFFFFFFU)

#define CSL_TMU_SQRTF32_R2_RESETVAL                                            (0x00000000U)

/* SQRTF32_R3 */

#define CSL_TMU_SQRTF32_R3_SQRTF32_R3_MASK                                     (0xFFFFFFFFU)
#define CSL_TMU_SQRTF32_R3_SQRTF32_R3_SHIFT                                    (0x00000000U)
#define CSL_TMU_SQRTF32_R3_SQRTF32_R3_RESETVAL                                 (0x00000000U)
#define CSL_TMU_SQRTF32_R3_SQRTF32_R3_MAX                                      (0xFFFFFFFFU)

#define CSL_TMU_SQRTF32_R3_RESETVAL                                            (0x00000000U)

/* SQRTF32_R4 */

#define CSL_TMU_SQRTF32_R4_SQRTF32_R4_MASK                                     (0xFFFFFFFFU)
#define CSL_TMU_SQRTF32_R4_SQRTF32_R4_SHIFT                                    (0x00000000U)
#define CSL_TMU_SQRTF32_R4_SQRTF32_R4_RESETVAL                                 (0x00000000U)
#define CSL_TMU_SQRTF32_R4_SQRTF32_R4_MAX                                      (0xFFFFFFFFU)

#define CSL_TMU_SQRTF32_R4_RESETVAL                                            (0x00000000U)

/* SQRTF32_R5 */

#define CSL_TMU_SQRTF32_R5_SQRTF32_R5_MASK                                     (0xFFFFFFFFU)
#define CSL_TMU_SQRTF32_R5_SQRTF32_R5_SHIFT                                    (0x00000000U)
#define CSL_TMU_SQRTF32_R5_SQRTF32_R5_RESETVAL                                 (0x00000000U)
#define CSL_TMU_SQRTF32_R5_SQRTF32_R5_MAX                                      (0xFFFFFFFFU)

#define CSL_TMU_SQRTF32_R5_RESETVAL                                            (0x00000000U)

/* SQRTF32_R6 */

#define CSL_TMU_SQRTF32_R6_SQRTF32_R6_MASK                                     (0xFFFFFFFFU)
#define CSL_TMU_SQRTF32_R6_SQRTF32_R6_SHIFT                                    (0x00000000U)
#define CSL_TMU_SQRTF32_R6_SQRTF32_R6_RESETVAL                                 (0x00000000U)
#define CSL_TMU_SQRTF32_R6_SQRTF32_R6_MAX                                      (0xFFFFFFFFU)

#define CSL_TMU_SQRTF32_R6_RESETVAL                                            (0x00000000U)

/* SQRTF32_R7 */

#define CSL_TMU_SQRTF32_R7_SQRTF32_R7_MASK                                     (0xFFFFFFFFU)
#define CSL_TMU_SQRTF32_R7_SQRTF32_R7_SHIFT                                    (0x00000000U)
#define CSL_TMU_SQRTF32_R7_SQRTF32_R7_RESETVAL                                 (0x00000000U)
#define CSL_TMU_SQRTF32_R7_SQRTF32_R7_MAX                                      (0xFFFFFFFFU)

#define CSL_TMU_SQRTF32_R7_RESETVAL                                            (0x00000000U)

/* IEXP2F32_R0 */

#define CSL_TMU_IEXP2F32_R0_IEXP2F32_R0_MASK                                   (0xFFFFFFFFU)
#define CSL_TMU_IEXP2F32_R0_IEXP2F32_R0_SHIFT                                  (0x00000000U)
#define CSL_TMU_IEXP2F32_R0_IEXP2F32_R0_RESETVAL                               (0x00000000U)
#define CSL_TMU_IEXP2F32_R0_IEXP2F32_R0_MAX                                    (0xFFFFFFFFU)

#define CSL_TMU_IEXP2F32_R0_RESETVAL                                           (0x00000000U)

/* IEXP2F32_R1 */

#define CSL_TMU_IEXP2F32_R1_IEXP2F32_R1_MASK                                   (0xFFFFFFFFU)
#define CSL_TMU_IEXP2F32_R1_IEXP2F32_R1_SHIFT                                  (0x00000000U)
#define CSL_TMU_IEXP2F32_R1_IEXP2F32_R1_RESETVAL                               (0x00000000U)
#define CSL_TMU_IEXP2F32_R1_IEXP2F32_R1_MAX                                    (0xFFFFFFFFU)

#define CSL_TMU_IEXP2F32_R1_RESETVAL                                           (0x00000000U)

/* IEXP2F32_R2 */

#define CSL_TMU_IEXP2F32_R2_IEXP2F32_R2_MASK                                   (0xFFFFFFFFU)
#define CSL_TMU_IEXP2F32_R2_IEXP2F32_R2_SHIFT                                  (0x00000000U)
#define CSL_TMU_IEXP2F32_R2_IEXP2F32_R2_RESETVAL                               (0x00000000U)
#define CSL_TMU_IEXP2F32_R2_IEXP2F32_R2_MAX                                    (0xFFFFFFFFU)

#define CSL_TMU_IEXP2F32_R2_RESETVAL                                           (0x00000000U)

/* IEXP2F32_R3 */

#define CSL_TMU_IEXP2F32_R3_IEXP2F32_R3_MASK                                   (0xFFFFFFFFU)
#define CSL_TMU_IEXP2F32_R3_IEXP2F32_R3_SHIFT                                  (0x00000000U)
#define CSL_TMU_IEXP2F32_R3_IEXP2F32_R3_RESETVAL                               (0x00000000U)
#define CSL_TMU_IEXP2F32_R3_IEXP2F32_R3_MAX                                    (0xFFFFFFFFU)

#define CSL_TMU_IEXP2F32_R3_RESETVAL                                           (0x00000000U)

/* IEXP2F32_R4 */

#define CSL_TMU_IEXP2F32_R4_IEXP2F32_R4_MASK                                   (0xFFFFFFFFU)
#define CSL_TMU_IEXP2F32_R4_IEXP2F32_R4_SHIFT                                  (0x00000000U)
#define CSL_TMU_IEXP2F32_R4_IEXP2F32_R4_RESETVAL                               (0x00000000U)
#define CSL_TMU_IEXP2F32_R4_IEXP2F32_R4_MAX                                    (0xFFFFFFFFU)

#define CSL_TMU_IEXP2F32_R4_RESETVAL                                           (0x00000000U)

/* IEXP2F32_R5 */

#define CSL_TMU_IEXP2F32_R5_IEXP2F32_R5_MASK                                   (0xFFFFFFFFU)
#define CSL_TMU_IEXP2F32_R5_IEXP2F32_R5_SHIFT                                  (0x00000000U)
#define CSL_TMU_IEXP2F32_R5_IEXP2F32_R5_RESETVAL                               (0x00000000U)
#define CSL_TMU_IEXP2F32_R5_IEXP2F32_R5_MAX                                    (0xFFFFFFFFU)

#define CSL_TMU_IEXP2F32_R5_RESETVAL                                           (0x00000000U)

/* IEXP2F32_R6 */

#define CSL_TMU_IEXP2F32_R6_IEXP2F32_R6_MASK                                   (0xFFFFFFFFU)
#define CSL_TMU_IEXP2F32_R6_IEXP2F32_R6_SHIFT                                  (0x00000000U)
#define CSL_TMU_IEXP2F32_R6_IEXP2F32_R6_RESETVAL                               (0x00000000U)
#define CSL_TMU_IEXP2F32_R6_IEXP2F32_R6_MAX                                    (0xFFFFFFFFU)

#define CSL_TMU_IEXP2F32_R6_RESETVAL                                           (0x00000000U)

/* IEXP2F32_R7 */

#define CSL_TMU_IEXP2F32_R7_IEXP2F32_R7_MASK                                   (0xFFFFFFFFU)
#define CSL_TMU_IEXP2F32_R7_IEXP2F32_R7_SHIFT                                  (0x00000000U)
#define CSL_TMU_IEXP2F32_R7_IEXP2F32_R7_RESETVAL                               (0x00000000U)
#define CSL_TMU_IEXP2F32_R7_IEXP2F32_R7_MAX                                    (0xFFFFFFFFU)

#define CSL_TMU_IEXP2F32_R7_RESETVAL                                           (0x00000000U)

/* LOG2F32_R0 */

#define CSL_TMU_LOG2F32_R0_LOG2F32_R0_MASK                                     (0xFFFFFFFFU)
#define CSL_TMU_LOG2F32_R0_LOG2F32_R0_SHIFT                                    (0x00000000U)
#define CSL_TMU_LOG2F32_R0_LOG2F32_R0_RESETVAL                                 (0x00000000U)
#define CSL_TMU_LOG2F32_R0_LOG2F32_R0_MAX                                      (0xFFFFFFFFU)

#define CSL_TMU_LOG2F32_R0_RESETVAL                                            (0x00000000U)

/* LOG2F32_R1 */

#define CSL_TMU_LOG2F32_R1_LOG2F32_R1_MASK                                     (0xFFFFFFFFU)
#define CSL_TMU_LOG2F32_R1_LOG2F32_R1_SHIFT                                    (0x00000000U)
#define CSL_TMU_LOG2F32_R1_LOG2F32_R1_RESETVAL                                 (0x00000000U)
#define CSL_TMU_LOG2F32_R1_LOG2F32_R1_MAX                                      (0xFFFFFFFFU)

#define CSL_TMU_LOG2F32_R1_RESETVAL                                            (0x00000000U)

/* LOG2F32_R2 */

#define CSL_TMU_LOG2F32_R2_LOG2F32_R2_MASK                                     (0xFFFFFFFFU)
#define CSL_TMU_LOG2F32_R2_LOG2F32_R2_SHIFT                                    (0x00000000U)
#define CSL_TMU_LOG2F32_R2_LOG2F32_R2_RESETVAL                                 (0x00000000U)
#define CSL_TMU_LOG2F32_R2_LOG2F32_R2_MAX                                      (0xFFFFFFFFU)

#define CSL_TMU_LOG2F32_R2_RESETVAL                                            (0x00000000U)

/* LOG2F32_R3 */

#define CSL_TMU_LOG2F32_R3_LOG2F32_R3_MASK                                     (0xFFFFFFFFU)
#define CSL_TMU_LOG2F32_R3_LOG2F32_R3_SHIFT                                    (0x00000000U)
#define CSL_TMU_LOG2F32_R3_LOG2F32_R3_RESETVAL                                 (0x00000000U)
#define CSL_TMU_LOG2F32_R3_LOG2F32_R3_MAX                                      (0xFFFFFFFFU)

#define CSL_TMU_LOG2F32_R3_RESETVAL                                            (0x00000000U)

/* LOG2F32_R4 */

#define CSL_TMU_LOG2F32_R4_LOG2F32_R4_MASK                                     (0xFFFFFFFFU)
#define CSL_TMU_LOG2F32_R4_LOG2F32_R4_SHIFT                                    (0x00000000U)
#define CSL_TMU_LOG2F32_R4_LOG2F32_R4_RESETVAL                                 (0x00000000U)
#define CSL_TMU_LOG2F32_R4_LOG2F32_R4_MAX                                      (0xFFFFFFFFU)

#define CSL_TMU_LOG2F32_R4_RESETVAL                                            (0x00000000U)

/* LOG2F32_R5 */

#define CSL_TMU_LOG2F32_R5_LOG2F32_R5_MASK                                     (0xFFFFFFFFU)
#define CSL_TMU_LOG2F32_R5_LOG2F32_R5_SHIFT                                    (0x00000000U)
#define CSL_TMU_LOG2F32_R5_LOG2F32_R5_RESETVAL                                 (0x00000000U)
#define CSL_TMU_LOG2F32_R5_LOG2F32_R5_MAX                                      (0xFFFFFFFFU)

#define CSL_TMU_LOG2F32_R5_RESETVAL                                            (0x00000000U)

/* LOG2F32_R6 */

#define CSL_TMU_LOG2F32_R6_LOG2F32_R6_MASK                                     (0xFFFFFFFFU)
#define CSL_TMU_LOG2F32_R6_LOG2F32_R6_SHIFT                                    (0x00000000U)
#define CSL_TMU_LOG2F32_R6_LOG2F32_R6_RESETVAL                                 (0x00000000U)
#define CSL_TMU_LOG2F32_R6_LOG2F32_R6_MAX                                      (0xFFFFFFFFU)

#define CSL_TMU_LOG2F32_R6_RESETVAL                                            (0x00000000U)

/* LOG2F32_R7 */

#define CSL_TMU_LOG2F32_R7_LOG2F32_R7_MASK                                     (0xFFFFFFFFU)
#define CSL_TMU_LOG2F32_R7_LOG2F32_R7_SHIFT                                    (0x00000000U)
#define CSL_TMU_LOG2F32_R7_LOG2F32_R7_RESETVAL                                 (0x00000000U)
#define CSL_TMU_LOG2F32_R7_LOG2F32_R7_MAX                                      (0xFFFFFFFFU)

#define CSL_TMU_LOG2F32_R7_RESETVAL                                            (0x00000000U)

/* QUADF32_X_R0_R1 */

#define CSL_TMU_QUADF32_X_R0_R1_QUADF32_X_R0_R1_MASK                           (0xFFFFFFFFU)
#define CSL_TMU_QUADF32_X_R0_R1_QUADF32_X_R0_R1_SHIFT                          (0x00000000U)
#define CSL_TMU_QUADF32_X_R0_R1_QUADF32_X_R0_R1_RESETVAL                       (0x00000000U)
#define CSL_TMU_QUADF32_X_R0_R1_QUADF32_X_R0_R1_MAX                            (0xFFFFFFFFU)

#define CSL_TMU_QUADF32_X_R0_R1_RESETVAL                                       (0x00000000U)

/* QUADF32_X_R1_R2 */

#define CSL_TMU_QUADF32_X_R1_R2_QUADF32_X_R1_R2_MASK                           (0xFFFFFFFFU)
#define CSL_TMU_QUADF32_X_R1_R2_QUADF32_X_R1_R2_SHIFT                          (0x00000000U)
#define CSL_TMU_QUADF32_X_R1_R2_QUADF32_X_R1_R2_RESETVAL                       (0x00000000U)
#define CSL_TMU_QUADF32_X_R1_R2_QUADF32_X_R1_R2_MAX                            (0xFFFFFFFFU)

#define CSL_TMU_QUADF32_X_R1_R2_RESETVAL                                       (0x00000000U)

/* QUADF32_X_R2_R3 */

#define CSL_TMU_QUADF32_X_R2_R3_QUADF32_X_R2_R3_MASK                           (0xFFFFFFFFU)
#define CSL_TMU_QUADF32_X_R2_R3_QUADF32_X_R2_R3_SHIFT                          (0x00000000U)
#define CSL_TMU_QUADF32_X_R2_R3_QUADF32_X_R2_R3_RESETVAL                       (0x00000000U)
#define CSL_TMU_QUADF32_X_R2_R3_QUADF32_X_R2_R3_MAX                            (0xFFFFFFFFU)

#define CSL_TMU_QUADF32_X_R2_R3_RESETVAL                                       (0x00000000U)

/* QUADF32_X_R3_R4 */

#define CSL_TMU_QUADF32_X_R3_R4_QUADF32_X_R3_R4_MASK                           (0xFFFFFFFFU)
#define CSL_TMU_QUADF32_X_R3_R4_QUADF32_X_R3_R4_SHIFT                          (0x00000000U)
#define CSL_TMU_QUADF32_X_R3_R4_QUADF32_X_R3_R4_RESETVAL                       (0x00000000U)
#define CSL_TMU_QUADF32_X_R3_R4_QUADF32_X_R3_R4_MAX                            (0xFFFFFFFFU)

#define CSL_TMU_QUADF32_X_R3_R4_RESETVAL                                       (0x00000000U)

/* QUADF32_X_R4_R5 */

#define CSL_TMU_QUADF32_X_R4_R5_QUADF32_X_R4_R5_MASK                           (0xFFFFFFFFU)
#define CSL_TMU_QUADF32_X_R4_R5_QUADF32_X_R4_R5_SHIFT                          (0x00000000U)
#define CSL_TMU_QUADF32_X_R4_R5_QUADF32_X_R4_R5_RESETVAL                       (0x00000000U)
#define CSL_TMU_QUADF32_X_R4_R5_QUADF32_X_R4_R5_MAX                            (0xFFFFFFFFU)

#define CSL_TMU_QUADF32_X_R4_R5_RESETVAL                                       (0x00000000U)

/* QUADF32_X_R5_R6 */

#define CSL_TMU_QUADF32_X_R5_R6_QUADF32_X_R5_R6_MASK                           (0xFFFFFFFFU)
#define CSL_TMU_QUADF32_X_R5_R6_QUADF32_X_R5_R6_SHIFT                          (0x00000000U)
#define CSL_TMU_QUADF32_X_R5_R6_QUADF32_X_R5_R6_RESETVAL                       (0x00000000U)
#define CSL_TMU_QUADF32_X_R5_R6_QUADF32_X_R5_R6_MAX                            (0xFFFFFFFFU)

#define CSL_TMU_QUADF32_X_R5_R6_RESETVAL                                       (0x00000000U)

/* QUADF32_X_R6_R7 */

#define CSL_TMU_QUADF32_X_R6_R7_QUADF32_X_R6_R7_MASK                           (0xFFFFFFFFU)
#define CSL_TMU_QUADF32_X_R6_R7_QUADF32_X_R6_R7_SHIFT                          (0x00000000U)
#define CSL_TMU_QUADF32_X_R6_R7_QUADF32_X_R6_R7_RESETVAL                       (0x00000000U)
#define CSL_TMU_QUADF32_X_R6_R7_QUADF32_X_R6_R7_MAX                            (0xFFFFFFFFU)

#define CSL_TMU_QUADF32_X_R6_R7_RESETVAL                                       (0x00000000U)

/* DIVF32_N_R0 */

#define CSL_TMU_DIVF32_N_R0_DIVF32_N_R0_MASK                                   (0xFFFFFFFFU)
#define CSL_TMU_DIVF32_N_R0_DIVF32_N_R0_SHIFT                                  (0x00000000U)
#define CSL_TMU_DIVF32_N_R0_DIVF32_N_R0_RESETVAL                               (0x00000000U)
#define CSL_TMU_DIVF32_N_R0_DIVF32_N_R0_MAX                                    (0xFFFFFFFFU)

#define CSL_TMU_DIVF32_N_R0_RESETVAL                                           (0x00000000U)

/* DIVF32_N_R1 */

#define CSL_TMU_DIVF32_N_R1_DIVF32_N_R1_MASK                                   (0xFFFFFFFFU)
#define CSL_TMU_DIVF32_N_R1_DIVF32_N_R1_SHIFT                                  (0x00000000U)
#define CSL_TMU_DIVF32_N_R1_DIVF32_N_R1_RESETVAL                               (0x00000000U)
#define CSL_TMU_DIVF32_N_R1_DIVF32_N_R1_MAX                                    (0xFFFFFFFFU)

#define CSL_TMU_DIVF32_N_R1_RESETVAL                                           (0x00000000U)

/* DIVF32_N_R2 */

#define CSL_TMU_DIVF32_N_R2_DIVF32_N_R2_MASK                                   (0xFFFFFFFFU)
#define CSL_TMU_DIVF32_N_R2_DIVF32_N_R2_SHIFT                                  (0x00000000U)
#define CSL_TMU_DIVF32_N_R2_DIVF32_N_R2_RESETVAL                               (0x00000000U)
#define CSL_TMU_DIVF32_N_R2_DIVF32_N_R2_MAX                                    (0xFFFFFFFFU)

#define CSL_TMU_DIVF32_N_R2_RESETVAL                                           (0x00000000U)

/* DIVF32_N_R3 */

#define CSL_TMU_DIVF32_N_R3_DIVF32_N_R3_MASK                                   (0xFFFFFFFFU)
#define CSL_TMU_DIVF32_N_R3_DIVF32_N_R3_SHIFT                                  (0x00000000U)
#define CSL_TMU_DIVF32_N_R3_DIVF32_N_R3_RESETVAL                               (0x00000000U)
#define CSL_TMU_DIVF32_N_R3_DIVF32_N_R3_MAX                                    (0xFFFFFFFFU)

#define CSL_TMU_DIVF32_N_R3_RESETVAL                                           (0x00000000U)

/* DIVF32_N_R4 */

#define CSL_TMU_DIVF32_N_R4_DIVF32_N_R4_MASK                                   (0xFFFFFFFFU)
#define CSL_TMU_DIVF32_N_R4_DIVF32_N_R4_SHIFT                                  (0x00000000U)
#define CSL_TMU_DIVF32_N_R4_DIVF32_N_R4_RESETVAL                               (0x00000000U)
#define CSL_TMU_DIVF32_N_R4_DIVF32_N_R4_MAX                                    (0xFFFFFFFFU)

#define CSL_TMU_DIVF32_N_R4_RESETVAL                                           (0x00000000U)

/* DIVF32_N_R5 */

#define CSL_TMU_DIVF32_N_R5_DIVF32_N_R5_MASK                                   (0xFFFFFFFFU)
#define CSL_TMU_DIVF32_N_R5_DIVF32_N_R5_SHIFT                                  (0x00000000U)
#define CSL_TMU_DIVF32_N_R5_DIVF32_N_R5_RESETVAL                               (0x00000000U)
#define CSL_TMU_DIVF32_N_R5_DIVF32_N_R5_MAX                                    (0xFFFFFFFFU)

#define CSL_TMU_DIVF32_N_R5_RESETVAL                                           (0x00000000U)

/* DIVF32_N_R6 */

#define CSL_TMU_DIVF32_N_R6_DIVF32_N_R6_MASK                                   (0xFFFFFFFFU)
#define CSL_TMU_DIVF32_N_R6_DIVF32_N_R6_SHIFT                                  (0x00000000U)
#define CSL_TMU_DIVF32_N_R6_DIVF32_N_R6_RESETVAL                               (0x00000000U)
#define CSL_TMU_DIVF32_N_R6_DIVF32_N_R6_MAX                                    (0xFFFFFFFFU)

#define CSL_TMU_DIVF32_N_R6_RESETVAL                                           (0x00000000U)

/* DIVF32_N_R7 */

#define CSL_TMU_DIVF32_N_R7_DIVF32_N_R7_MASK                                   (0xFFFFFFFFU)
#define CSL_TMU_DIVF32_N_R7_DIVF32_N_R7_SHIFT                                  (0x00000000U)
#define CSL_TMU_DIVF32_N_R7_DIVF32_N_R7_RESETVAL                               (0x00000000U)
#define CSL_TMU_DIVF32_N_R7_DIVF32_N_R7_MAX                                    (0xFFFFFFFFU)

#define CSL_TMU_DIVF32_N_R7_RESETVAL                                           (0x00000000U)

/* QUADF32_DIVF32_OP2 */

#define CSL_TMU_QUADF32_DIVF32_OP2_QUADF32_DIVF32_OP2_MASK                     (0xFFFFFFFFU)
#define CSL_TMU_QUADF32_DIVF32_OP2_QUADF32_DIVF32_OP2_SHIFT                    (0x00000000U)
#define CSL_TMU_QUADF32_DIVF32_OP2_QUADF32_DIVF32_OP2_RESETVAL                 (0x00000000U)
#define CSL_TMU_QUADF32_DIVF32_OP2_QUADF32_DIVF32_OP2_MAX                      (0xFFFFFFFFU)

#define CSL_TMU_QUADF32_DIVF32_OP2_RESETVAL                                    (0x00000000U)

/* RESULT_R0 */

#define CSL_TMU_RESULT_R0_R0_MASK                                              (0xFFFFFFFFU)
#define CSL_TMU_RESULT_R0_R0_SHIFT                                             (0x00000000U)
#define CSL_TMU_RESULT_R0_R0_RESETVAL                                          (0x00000000U)
#define CSL_TMU_RESULT_R0_R0_MAX                                               (0xFFFFFFFFU)

#define CSL_TMU_RESULT_R0_RESETVAL                                             (0x00000000U)

/* RESULT_R1 */

#define CSL_TMU_RESULT_R1_R1_MASK                                              (0xFFFFFFFFU)
#define CSL_TMU_RESULT_R1_R1_SHIFT                                             (0x00000000U)
#define CSL_TMU_RESULT_R1_R1_RESETVAL                                          (0x00000000U)
#define CSL_TMU_RESULT_R1_R1_MAX                                               (0xFFFFFFFFU)

#define CSL_TMU_RESULT_R1_RESETVAL                                             (0x00000000U)

/* RESULT_R2 */

#define CSL_TMU_RESULT_R2_R2_MASK                                              (0xFFFFFFFFU)
#define CSL_TMU_RESULT_R2_R2_SHIFT                                             (0x00000000U)
#define CSL_TMU_RESULT_R2_R2_RESETVAL                                          (0x00000000U)
#define CSL_TMU_RESULT_R2_R2_MAX                                               (0xFFFFFFFFU)

#define CSL_TMU_RESULT_R2_RESETVAL                                             (0x00000000U)

/* RESULT_R3 */

#define CSL_TMU_RESULT_R3_R3_MASK                                              (0xFFFFFFFFU)
#define CSL_TMU_RESULT_R3_R3_SHIFT                                             (0x00000000U)
#define CSL_TMU_RESULT_R3_R3_RESETVAL                                          (0x00000000U)
#define CSL_TMU_RESULT_R3_R3_MAX                                               (0xFFFFFFFFU)

#define CSL_TMU_RESULT_R3_RESETVAL                                             (0x00000000U)

/* RESULT_R4 */

#define CSL_TMU_RESULT_R4_R4_MASK                                              (0xFFFFFFFFU)
#define CSL_TMU_RESULT_R4_R4_SHIFT                                             (0x00000000U)
#define CSL_TMU_RESULT_R4_R4_RESETVAL                                          (0x00000000U)
#define CSL_TMU_RESULT_R4_R4_MAX                                               (0xFFFFFFFFU)

#define CSL_TMU_RESULT_R4_RESETVAL                                             (0x00000000U)

/* RESULT_R5 */

#define CSL_TMU_RESULT_R5_R5_MASK                                              (0xFFFFFFFFU)
#define CSL_TMU_RESULT_R5_R5_SHIFT                                             (0x00000000U)
#define CSL_TMU_RESULT_R5_R5_RESETVAL                                          (0x00000000U)
#define CSL_TMU_RESULT_R5_R5_MAX                                               (0xFFFFFFFFU)

#define CSL_TMU_RESULT_R5_RESETVAL                                             (0x00000000U)

/* RESULT_R6 */

#define CSL_TMU_RESULT_R6_R6_MASK                                              (0xFFFFFFFFU)
#define CSL_TMU_RESULT_R6_R6_SHIFT                                             (0x00000000U)
#define CSL_TMU_RESULT_R6_R6_RESETVAL                                          (0x00000000U)
#define CSL_TMU_RESULT_R6_R6_MAX                                               (0xFFFFFFFFU)

#define CSL_TMU_RESULT_R6_RESETVAL                                             (0x00000000U)

/* RESULT_R7 */

#define CSL_TMU_RESULT_R7_R7_MASK                                              (0xFFFFFFFFU)
#define CSL_TMU_RESULT_R7_R7_SHIFT                                             (0x00000000U)
#define CSL_TMU_RESULT_R7_R7_RESETVAL                                          (0x00000000U)
#define CSL_TMU_RESULT_R7_R7_MAX                                               (0xFFFFFFFFU)

#define CSL_TMU_RESULT_R7_RESETVAL                                             (0x00000000U)

/* CSAVE_R0 */

#define CSL_TMU_CSAVE_R0_CSAVE_R0_MASK                                         (0xFFFFFFFFU)
#define CSL_TMU_CSAVE_R0_CSAVE_R0_SHIFT                                        (0x00000000U)
#define CSL_TMU_CSAVE_R0_CSAVE_R0_RESETVAL                                     (0x00000000U)
#define CSL_TMU_CSAVE_R0_CSAVE_R0_MAX                                          (0xFFFFFFFFU)

#define CSL_TMU_CSAVE_R0_RESETVAL                                              (0x00000000U)

/* CSAVE_R1 */

#define CSL_TMU_CSAVE_R1_CSAVE_R1_MASK                                         (0xFFFFFFFFU)
#define CSL_TMU_CSAVE_R1_CSAVE_R1_SHIFT                                        (0x00000000U)
#define CSL_TMU_CSAVE_R1_CSAVE_R1_RESETVAL                                     (0x00000000U)
#define CSL_TMU_CSAVE_R1_CSAVE_R1_MAX                                          (0xFFFFFFFFU)

#define CSL_TMU_CSAVE_R1_RESETVAL                                              (0x00000000U)

/* CSAVE_R2 */

#define CSL_TMU_CSAVE_R2_CSAVE_R2_MASK                                         (0xFFFFFFFFU)
#define CSL_TMU_CSAVE_R2_CSAVE_R2_SHIFT                                        (0x00000000U)
#define CSL_TMU_CSAVE_R2_CSAVE_R2_RESETVAL                                     (0x00000000U)
#define CSL_TMU_CSAVE_R2_CSAVE_R2_MAX                                          (0xFFFFFFFFU)

#define CSL_TMU_CSAVE_R2_RESETVAL                                              (0x00000000U)

/* CSAVE_R3 */

#define CSL_TMU_CSAVE_R3_CSAVE_R3_MASK                                         (0xFFFFFFFFU)
#define CSL_TMU_CSAVE_R3_CSAVE_R3_SHIFT                                        (0x00000000U)
#define CSL_TMU_CSAVE_R3_CSAVE_R3_RESETVAL                                     (0x00000000U)
#define CSL_TMU_CSAVE_R3_CSAVE_R3_MAX                                          (0xFFFFFFFFU)

#define CSL_TMU_CSAVE_R3_RESETVAL                                              (0x00000000U)

/* CSAVE_R4 */

#define CSL_TMU_CSAVE_R4_CSAVE_R4_MASK                                         (0xFFFFFFFFU)
#define CSL_TMU_CSAVE_R4_CSAVE_R4_SHIFT                                        (0x00000000U)
#define CSL_TMU_CSAVE_R4_CSAVE_R4_RESETVAL                                     (0x00000000U)
#define CSL_TMU_CSAVE_R4_CSAVE_R4_MAX                                          (0xFFFFFFFFU)

#define CSL_TMU_CSAVE_R4_RESETVAL                                              (0x00000000U)

/* CSAVE_R5 */

#define CSL_TMU_CSAVE_R5_CSAVE_R5_MASK                                         (0xFFFFFFFFU)
#define CSL_TMU_CSAVE_R5_CSAVE_R5_SHIFT                                        (0x00000000U)
#define CSL_TMU_CSAVE_R5_CSAVE_R5_RESETVAL                                     (0x00000000U)
#define CSL_TMU_CSAVE_R5_CSAVE_R5_MAX                                          (0xFFFFFFFFU)

#define CSL_TMU_CSAVE_R5_RESETVAL                                              (0x00000000U)

/* CSAVE_R6 */

#define CSL_TMU_CSAVE_R6_CSAVE_R6_MASK                                         (0xFFFFFFFFU)
#define CSL_TMU_CSAVE_R6_CSAVE_R6_SHIFT                                        (0x00000000U)
#define CSL_TMU_CSAVE_R6_CSAVE_R6_RESETVAL                                     (0x00000000U)
#define CSL_TMU_CSAVE_R6_CSAVE_R6_MAX                                          (0xFFFFFFFFU)

#define CSL_TMU_CSAVE_R6_RESETVAL                                              (0x00000000U)

/* CSAVE_R7 */

#define CSL_TMU_CSAVE_R7_CSAVE_R7_MASK                                         (0xFFFFFFFFU)
#define CSL_TMU_CSAVE_R7_CSAVE_R7_SHIFT                                        (0x00000000U)
#define CSL_TMU_CSAVE_R7_CSAVE_R7_RESETVAL                                     (0x00000000U)
#define CSL_TMU_CSAVE_R7_CSAVE_R7_MAX                                          (0xFFFFFFFFU)

#define CSL_TMU_CSAVE_R7_RESETVAL                                              (0x00000000U)

/* CSAVE_OP2 */

#define CSL_TMU_CSAVE_OP2_CSAVE_OP2_MASK                                       (0xFFFFFFFFU)
#define CSL_TMU_CSAVE_OP2_CSAVE_OP2_SHIFT                                      (0x00000000U)
#define CSL_TMU_CSAVE_OP2_CSAVE_OP2_RESETVAL                                   (0x00000000U)
#define CSL_TMU_CSAVE_OP2_CSAVE_OP2_MAX                                        (0xFFFFFFFFU)

#define CSL_TMU_CSAVE_OP2_RESETVAL                                             (0x00000000U)

/* CONTEXT_SAVE */

#define CSL_TMU_CONTEXT_SAVE_SAVE_MASK                                         (0x00000001U)
#define CSL_TMU_CONTEXT_SAVE_SAVE_SHIFT                                        (0x00000000U)
#define CSL_TMU_CONTEXT_SAVE_SAVE_RESETVAL                                     (0x00000000U)
#define CSL_TMU_CONTEXT_SAVE_SAVE_MAX                                          (0x00000001U)

#define CSL_TMU_CONTEXT_SAVE_RESERVED_1_MASK                                   (0xFFFFFFFEU)
#define CSL_TMU_CONTEXT_SAVE_RESERVED_1_SHIFT                                  (0x00000001U)
#define CSL_TMU_CONTEXT_SAVE_RESERVED_1_RESETVAL                               (0x00000000U)
#define CSL_TMU_CONTEXT_SAVE_RESERVED_1_MAX                                    (0x7FFFFFFFU)

#define CSL_TMU_CONTEXT_SAVE_RESETVAL                                          (0x00000000U)

/* CONTEXT_RESTORE */

#define CSL_TMU_CONTEXT_RESTORE_RESTORE_MASK                                   (0x00000001U)
#define CSL_TMU_CONTEXT_RESTORE_RESTORE_SHIFT                                  (0x00000000U)
#define CSL_TMU_CONTEXT_RESTORE_RESTORE_RESETVAL                               (0x00000000U)
#define CSL_TMU_CONTEXT_RESTORE_RESTORE_MAX                                    (0x00000001U)

#define CSL_TMU_CONTEXT_RESTORE_RESERVED_1_MASK                                (0xFFFFFFFEU)
#define CSL_TMU_CONTEXT_RESTORE_RESERVED_1_SHIFT                               (0x00000001U)
#define CSL_TMU_CONTEXT_RESTORE_RESERVED_1_RESETVAL                            (0x00000000U)
#define CSL_TMU_CONTEXT_RESTORE_RESERVED_1_MAX                                 (0x7FFFFFFFU)

#define CSL_TMU_CONTEXT_RESTORE_RESETVAL                                       (0x00000000U)

/* STF */

#define CSL_TMU_STF_LVF_MASK                                                   (0x00000001U)
#define CSL_TMU_STF_LVF_SHIFT                                                  (0x00000000U)
#define CSL_TMU_STF_LVF_RESETVAL                                               (0x00000000U)
#define CSL_TMU_STF_LVF_MAX                                                    (0x00000001U)

#define CSL_TMU_STF_LUF_MASK                                                   (0x00000002U)
#define CSL_TMU_STF_LUF_SHIFT                                                  (0x00000001U)
#define CSL_TMU_STF_LUF_RESETVAL                                               (0x00000000U)
#define CSL_TMU_STF_LUF_MAX                                                    (0x00000001U)

#define CSL_TMU_STF_RESERVED_1_MASK                                            (0x000000FCU)
#define CSL_TMU_STF_RESERVED_1_SHIFT                                           (0x00000002U)
#define CSL_TMU_STF_RESERVED_1_RESETVAL                                        (0x00000000U)
#define CSL_TMU_STF_RESERVED_1_MAX                                             (0x0000003FU)

#define CSL_TMU_STF_LVF_WR_EN_MASK                                             (0x00000100U)
#define CSL_TMU_STF_LVF_WR_EN_SHIFT                                            (0x00000008U)
#define CSL_TMU_STF_LVF_WR_EN_RESETVAL                                         (0x00000000U)
#define CSL_TMU_STF_LVF_WR_EN_MAX                                              (0x00000001U)

#define CSL_TMU_STF_LUF_WR_EN_MASK                                             (0x00000200U)
#define CSL_TMU_STF_LUF_WR_EN_SHIFT                                            (0x00000009U)
#define CSL_TMU_STF_LUF_WR_EN_RESETVAL                                         (0x00000000U)
#define CSL_TMU_STF_LUF_WR_EN_MAX                                              (0x00000001U)

#define CSL_TMU_STF_RESERVED_2_MASK                                            (0xFFFFFC00U)
#define CSL_TMU_STF_RESERVED_2_SHIFT                                           (0x0000000AU)
#define CSL_TMU_STF_RESERVED_2_RESETVAL                                        (0x00000000U)
#define CSL_TMU_STF_RESERVED_2_MAX                                             (0x003FFFFFU)

#define CSL_TMU_STF_RESETVAL                                                   (0x00000000U)

/* PARITY_TEST */

#define CSL_TMU_PARITY_TEST_TESTEN_MASK                                        (0x0000000FU)
#define CSL_TMU_PARITY_TEST_TESTEN_SHIFT                                       (0x00000000U)
#define CSL_TMU_PARITY_TEST_TESTEN_RESETVAL                                    (0x00000000U)
#define CSL_TMU_PARITY_TEST_TESTEN_MAX                                         (0x0000000FU)

#define CSL_TMU_PARITY_TEST_RESERVED_1_MASK                                    (0x0000FFF0U)
#define CSL_TMU_PARITY_TEST_RESERVED_1_SHIFT                                   (0x00000004U)
#define CSL_TMU_PARITY_TEST_RESERVED_1_RESETVAL                                (0x00000000U)
#define CSL_TMU_PARITY_TEST_RESERVED_1_MAX                                     (0x00000FFFU)

#define CSL_TMU_PARITY_TEST_RESERVED_2_MASK                                    (0xFFFF0000U)
#define CSL_TMU_PARITY_TEST_RESERVED_2_SHIFT                                   (0x00000010U)
#define CSL_TMU_PARITY_TEST_RESERVED_2_RESETVAL                                (0x00000000U)
#define CSL_TMU_PARITY_TEST_RESERVED_2_MAX                                     (0x0000FFFFU)

#define CSL_TMU_PARITY_TEST_RESETVAL                                           (0x00000000U)

/* LCM_LOCK */

#define CSL_TMU_LCM_LOCK_PARITY_TEST_MASK                                      (0x00000001U)
#define CSL_TMU_LCM_LOCK_PARITY_TEST_SHIFT                                     (0x00000000U)
#define CSL_TMU_LCM_LOCK_PARITY_TEST_RESETVAL                                  (0x00000000U)
#define CSL_TMU_LCM_LOCK_PARITY_TEST_MAX                                       (0x00000001U)

#define CSL_TMU_LCM_LOCK_RESERVED_1_MASK                                       (0xFFFFFFFEU)
#define CSL_TMU_LCM_LOCK_RESERVED_1_SHIFT                                      (0x00000001U)
#define CSL_TMU_LCM_LOCK_RESERVED_1_RESETVAL                                   (0x00000000U)
#define CSL_TMU_LCM_LOCK_RESERVED_1_MAX                                        (0x7FFFFFFFU)

#define CSL_TMU_LCM_LOCK_RESETVAL                                              (0x00000000U)

/* LCM_COMMIT */

#define CSL_TMU_LCM_COMMIT_PARITY_TEST_MASK                                    (0x00000001U)
#define CSL_TMU_LCM_COMMIT_PARITY_TEST_SHIFT                                   (0x00000000U)
#define CSL_TMU_LCM_COMMIT_PARITY_TEST_RESETVAL                                (0x00000000U)
#define CSL_TMU_LCM_COMMIT_PARITY_TEST_MAX                                     (0x00000001U)

#define CSL_TMU_LCM_COMMIT_RESERVED_1_MASK                                     (0xFFFFFFFEU)
#define CSL_TMU_LCM_COMMIT_RESERVED_1_SHIFT                                    (0x00000001U)
#define CSL_TMU_LCM_COMMIT_RESERVED_1_RESETVAL                                 (0x00000000U)
#define CSL_TMU_LCM_COMMIT_RESERVED_1_MAX                                      (0x7FFFFFFFU)

#define CSL_TMU_LCM_COMMIT_RESETVAL                                            (0x00000000U)

#ifdef __cplusplus
}
#endif
#endif
