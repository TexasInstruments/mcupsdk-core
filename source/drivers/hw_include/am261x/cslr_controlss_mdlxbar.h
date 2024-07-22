/********************************************************************
 * Copyright (C) 2023 Texas Instruments Incorporated.
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
 *  Name        : cslr_controlss_mdlxbar.h
*/
#ifndef CSLR_CONTROLSS_MDLXBAR_H_
#define CSLR_CONTROLSS_MDLXBAR_H_

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
    XBAR INPUT Macros
**************************************************************************/

/******************* GO *********************/
#define MDL_XBAR_EPWM0A_SCLK         (0x00000001)
#define MDL_XBAR_EPWM1A_SCLK         (0x00000002)
#define MDL_XBAR_EPWM2A_SCLK         (0x00000004)
#define MDL_XBAR_EPWM3A_SCLK         (0x00000008)
#define MDL_XBAR_EPWM4A_SCLK         (0x00000010)
#define MDL_XBAR_EPWM5A_SCLK         (0x00000020)
#define MDL_XBAR_EPWM6A_SCLK         (0x00000040)
#define MDL_XBAR_EPWM7A_SCLK         (0x00000080)
#define MDL_XBAR_EPWM8A_SCLK         (0x00000100)
#define MDL_XBAR_EPWM9A_SCLK         (0x00000200)

/******************* G1 *********************/
#define MDL_XBAR_EPWM0B_SCLK         (0x00000001)
#define MDL_XBAR_EPWM1B_SCLK         (0x00000002)
#define MDL_XBAR_EPWM2B_SCLK         (0x00000004)
#define MDL_XBAR_EPWM3B_SCLK         (0x00000008)
#define MDL_XBAR_EPWM4B_SCLK         (0x00000010)
#define MDL_XBAR_EPWM5B_SCLK         (0x00000020)
#define MDL_XBAR_EPWM6B_SCLK         (0x00000040)
#define MDL_XBAR_EPWM7B_SCLK         (0x00000080)
#define MDL_XBAR_EPWM8B_SCLK         (0x00000100)
#define MDL_XBAR_EPWM9B_SCLK         (0x00000200)

/******************* G2 *********************/
#define MDL_XBAR_ICSS0_PORT0_GPO0    (0x00000001)
#define MDL_XBAR_ICSS0_PORT0_GPO1    (0x00000002)
#define MDL_XBAR_ICSS0_PORT0_GPO2    (0x00000004)
#define MDL_XBAR_ICSS0_PORT0_GPO3    (0x00000008)
#define MDL_XBAR_ICSS0_PORT0_GPO4    (0x00000010)
#define MDL_XBAR_ICSS0_PORT0_GPO5    (0x00000020)
#define MDL_XBAR_ICSS0_PORT0_GPO6    (0x00000040)
#define MDL_XBAR_ICSS0_PORT0_GPO7    (0x00000080)
#define MDL_XBAR_ICSS0_PORT0_GPO8    (0x00000100)
#define MDL_XBAR_ICSS0_PORT0_GPO9    (0x00000200)
#define MDL_XBAR_ICSS0_PORT0_GPO10   (0x00000400)
#define MDL_XBAR_ICSS0_PORT0_GPO11   (0x00000800)
#define MDL_XBAR_ICSS0_PORT0_GPO12   (0x00001000)
#define MDL_XBAR_ICSS0_PORT0_GPO13   (0x00002000)
#define MDL_XBAR_ICSS0_PORT0_GPO14   (0x00004000)
#define MDL_XBAR_ICSS0_PORT0_GPO15   (0x00008000)
#define MDL_XBAR_ICSS0_PORT1_GPO0    (0x00010000)
#define MDL_XBAR_ICSS0_PORT1_GPO1    (0x00020000)
#define MDL_XBAR_ICSS0_PORT1_GPO2    (0x00040000)
#define MDL_XBAR_ICSS0_PORT1_GPO3    (0x00080000)
#define MDL_XBAR_ICSS0_PORT1_GPO4    (0x00100000)
#define MDL_XBAR_ICSS0_PORT1_GPO5    (0x00200000)
#define MDL_XBAR_ICSS0_PORT1_GPO6    (0x00400000)
#define MDL_XBAR_ICSS0_PORT1_GPO7    (0x00800000)
#define MDL_XBAR_ICSS0_PORT1_GPO8    (0x01000000)
#define MDL_XBAR_ICSS0_PORT1_GPO9    (0x02000000)
#define MDL_XBAR_ICSS0_PORT1_GPO10   (0x04000000)
#define MDL_XBAR_ICSS0_PORT1_GPO11   (0x08000000)
#define MDL_XBAR_ICSS0_PORT1_GPO12   (0x10000000)
#define MDL_XBAR_ICSS0_PORT1_GPO13   (0x20000000)
#define MDL_XBAR_ICSS0_PORT1_GPO14   (0x40000000)
#define MDL_XBAR_ICSS0_PORT1_GPO15   (0x80000000)

/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t PID;
    volatile uint8_t  Resv_256[252];
    volatile uint32_t MDLXBAR0_G0;
    volatile uint32_t MDLXBAR0_G1;
    volatile uint32_t MDLXBAR0_G2;
    volatile uint8_t  Resv_320[52];
    volatile uint32_t MDLXBAR1_G0;
    volatile uint32_t MDLXBAR1_G1;
    volatile uint32_t MDLXBAR1_G2;
    volatile uint8_t  Resv_384[52];
    volatile uint32_t MDLXBAR2_G0;
    volatile uint32_t MDLXBAR2_G1;
    volatile uint32_t MDLXBAR2_G2;
    volatile uint8_t  Resv_448[52];
    volatile uint32_t MDLXBAR3_G0;
    volatile uint32_t MDLXBAR3_G1;
    volatile uint32_t MDLXBAR3_G2;
    volatile uint8_t  Resv_512[52];
    volatile uint32_t MDLXBAR4_G0;
    volatile uint32_t MDLXBAR4_G1;
    volatile uint32_t MDLXBAR4_G2;
    volatile uint8_t  Resv_576[52];
    volatile uint32_t MDLXBAR5_G0;
    volatile uint32_t MDLXBAR5_G1;
    volatile uint32_t MDLXBAR5_G2;
    volatile uint8_t  Resv_640[52];
    volatile uint32_t MDLXBAR6_G0;
    volatile uint32_t MDLXBAR6_G1;
    volatile uint32_t MDLXBAR6_G2;
    volatile uint8_t  Resv_704[52];
    volatile uint32_t MDLXBAR7_G0;
    volatile uint32_t MDLXBAR7_G1;
    volatile uint32_t MDLXBAR7_G2;
    volatile uint8_t  Resv_768[52];
    volatile uint32_t MDLXBAR8_G0;
    volatile uint32_t MDLXBAR8_G1;
    volatile uint32_t MDLXBAR8_G2;
    volatile uint8_t  Resv_832[52];
    volatile uint32_t MDLXBAR9_G0;
    volatile uint32_t MDLXBAR9_G1;
    volatile uint32_t MDLXBAR9_G2;
    volatile uint8_t  Resv_896[52];
    volatile uint32_t MDLXBAR10_G0;
    volatile uint32_t MDLXBAR10_G1;
    volatile uint32_t MDLXBAR10_G2;
    volatile uint8_t  Resv_960[52];
    volatile uint32_t MDLXBAR11_G0;
    volatile uint32_t MDLXBAR11_G1;
    volatile uint32_t MDLXBAR11_G2;
    volatile uint8_t  Resv_1024[52];
    volatile uint32_t MDLXBAR12_G0;
    volatile uint32_t MDLXBAR12_G1;
    volatile uint32_t MDLXBAR12_G2;
    volatile uint8_t  Resv_1088[52];
    volatile uint32_t MDLXBAR13_G0;
    volatile uint32_t MDLXBAR13_G1;
    volatile uint32_t MDLXBAR13_G2;
    volatile uint8_t  Resv_1152[52];
    volatile uint32_t MDLXBAR14_G0;
    volatile uint32_t MDLXBAR14_G1;
    volatile uint32_t MDLXBAR14_G2;
    volatile uint8_t  Resv_1216[52];
    volatile uint32_t MDLXBAR15_G0;
    volatile uint32_t MDLXBAR15_G1;
    volatile uint32_t MDLXBAR15_G2;
} CSL_controlss_mdlxbarRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_CONTROLSS_MDLXBAR_PID                                              (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR0_G0                                      (0x00000100U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR0_G1                                      (0x00000104U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR0_G2                                      (0x00000108U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR1_G0                                      (0x00000140U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR1_G1                                      (0x00000144U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR1_G2                                      (0x00000148U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR2_G0                                      (0x00000180U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR2_G1                                      (0x00000184U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR2_G2                                      (0x00000188U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR3_G0                                      (0x000001C0U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR3_G1                                      (0x000001C4U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR3_G2                                      (0x000001C8U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR4_G0                                      (0x00000200U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR4_G1                                      (0x00000204U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR4_G2                                      (0x00000208U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR5_G0                                      (0x00000240U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR5_G1                                      (0x00000244U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR5_G2                                      (0x00000248U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR6_G0                                      (0x00000280U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR6_G1                                      (0x00000284U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR6_G2                                      (0x00000288U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR7_G0                                      (0x000002C0U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR7_G1                                      (0x000002C4U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR7_G2                                      (0x000002C8U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR8_G0                                      (0x00000300U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR8_G1                                      (0x00000304U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR8_G2                                      (0x00000308U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR9_G0                                      (0x00000340U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR9_G1                                      (0x00000344U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR9_G2                                      (0x00000348U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR10_G0                                     (0x00000380U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR10_G1                                     (0x00000384U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR10_G2                                     (0x00000388U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR11_G0                                     (0x000003C0U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR11_G1                                     (0x000003C4U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR11_G2                                     (0x000003C8U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR12_G0                                     (0x00000400U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR12_G1                                     (0x00000404U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR12_G2                                     (0x00000408U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR13_G0                                     (0x00000440U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR13_G1                                     (0x00000444U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR13_G2                                     (0x00000448U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR14_G0                                     (0x00000480U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR14_G1                                     (0x00000484U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR14_G2                                     (0x00000488U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR15_G0                                     (0x000004C0U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR15_G1                                     (0x000004C4U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR15_G2                                     (0x000004C8U)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* PID */

#define CSL_CONTROLSS_MDLXBAR_PID_PID_MINOR_MASK                               (0x0000003FU)
#define CSL_CONTROLSS_MDLXBAR_PID_PID_MINOR_SHIFT                              (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_PID_PID_MINOR_RESETVAL                           (0x00000014U)
#define CSL_CONTROLSS_MDLXBAR_PID_PID_MINOR_MAX                                (0x0000003FU)

#define CSL_CONTROLSS_MDLXBAR_PID_PID_CUSTOM_MASK                              (0x000000C0U)
#define CSL_CONTROLSS_MDLXBAR_PID_PID_CUSTOM_SHIFT                             (0x00000006U)
#define CSL_CONTROLSS_MDLXBAR_PID_PID_CUSTOM_RESETVAL                          (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_PID_PID_CUSTOM_MAX                               (0x00000003U)

#define CSL_CONTROLSS_MDLXBAR_PID_PID_MAJOR_MASK                               (0x00000700U)
#define CSL_CONTROLSS_MDLXBAR_PID_PID_MAJOR_SHIFT                              (0x00000008U)
#define CSL_CONTROLSS_MDLXBAR_PID_PID_MAJOR_RESETVAL                           (0x00000002U)
#define CSL_CONTROLSS_MDLXBAR_PID_PID_MAJOR_MAX                                (0x00000007U)

#define CSL_CONTROLSS_MDLXBAR_PID_PID_MISC_MASK                                (0x0000F800U)
#define CSL_CONTROLSS_MDLXBAR_PID_PID_MISC_SHIFT                               (0x0000000BU)
#define CSL_CONTROLSS_MDLXBAR_PID_PID_MISC_RESETVAL                            (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_PID_PID_MISC_MAX                                 (0x0000001FU)

#define CSL_CONTROLSS_MDLXBAR_PID_PID_MSB16_MASK                               (0xFFFF0000U)
#define CSL_CONTROLSS_MDLXBAR_PID_PID_MSB16_SHIFT                              (0x00000010U)
#define CSL_CONTROLSS_MDLXBAR_PID_PID_MSB16_RESETVAL                           (0x00006180U)
#define CSL_CONTROLSS_MDLXBAR_PID_PID_MSB16_MAX                                (0x0000FFFFU)

#define CSL_CONTROLSS_MDLXBAR_PID_RESETVAL                                     (0x61800214U)

/* MDLXBAR0_G0 */

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR0_G0_SEL_MASK                             (0x000003FFU)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR0_G0_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR0_G0_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR0_G0_SEL_MAX                              (0x000003FFU)

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR0_G0_RESETVAL                             (0x00000000U)

/* MDLXBAR0_G1 */

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR0_G1_SEL_MASK                             (0x000003FFU)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR0_G1_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR0_G1_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR0_G1_SEL_MAX                              (0x000003FFU)

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR0_G1_RESETVAL                             (0x00000000U)

/* MDLXBAR0_G2 */

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR0_G2_SEL_MASK                             (0xFFFFFFFFU)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR0_G2_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR0_G2_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR0_G2_SEL_MAX                              (0xFFFFFFFFU)

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR0_G2_RESETVAL                             (0x00000000U)

/* MDLXBAR1_G0 */

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR1_G0_SEL_MASK                             (0x000003FFU)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR1_G0_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR1_G0_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR1_G0_SEL_MAX                              (0x000003FFU)

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR1_G0_RESETVAL                             (0x00000000U)

/* MDLXBAR1_G1 */

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR1_G1_SEL_MASK                             (0x000003FFU)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR1_G1_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR1_G1_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR1_G1_SEL_MAX                              (0x000003FFU)

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR1_G1_RESETVAL                             (0x00000000U)

/* MDLXBAR1_G2 */

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR1_G2_SEL_MASK                             (0xFFFFFFFFU)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR1_G2_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR1_G2_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR1_G2_SEL_MAX                              (0xFFFFFFFFU)

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR1_G2_RESETVAL                             (0x00000000U)

/* MDLXBAR2_G0 */

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR2_G0_SEL_MASK                             (0x000003FFU)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR2_G0_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR2_G0_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR2_G0_SEL_MAX                              (0x000003FFU)

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR2_G0_RESETVAL                             (0x00000000U)

/* MDLXBAR2_G1 */

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR2_G1_SEL_MASK                             (0x000003FFU)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR2_G1_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR2_G1_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR2_G1_SEL_MAX                              (0x000003FFU)

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR2_G1_RESETVAL                             (0x00000000U)

/* MDLXBAR2_G2 */

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR2_G2_SEL_MASK                             (0xFFFFFFFFU)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR2_G2_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR2_G2_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR2_G2_SEL_MAX                              (0xFFFFFFFFU)

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR2_G2_RESETVAL                             (0x00000000U)

/* MDLXBAR3_G0 */

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR3_G0_SEL_MASK                             (0x000003FFU)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR3_G0_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR3_G0_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR3_G0_SEL_MAX                              (0x000003FFU)

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR3_G0_RESETVAL                             (0x00000000U)

/* MDLXBAR3_G1 */

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR3_G1_SEL_MASK                             (0x000003FFU)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR3_G1_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR3_G1_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR3_G1_SEL_MAX                              (0x000003FFU)

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR3_G1_RESETVAL                             (0x00000000U)

/* MDLXBAR3_G2 */

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR3_G2_SEL_MASK                             (0xFFFFFFFFU)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR3_G2_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR3_G2_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR3_G2_SEL_MAX                              (0xFFFFFFFFU)

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR3_G2_RESETVAL                             (0x00000000U)

/* MDLXBAR4_G0 */

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR4_G0_SEL_MASK                             (0x000003FFU)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR4_G0_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR4_G0_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR4_G0_SEL_MAX                              (0x000003FFU)

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR4_G0_RESETVAL                             (0x00000000U)

/* MDLXBAR4_G1 */

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR4_G1_SEL_MASK                             (0x000003FFU)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR4_G1_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR4_G1_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR4_G1_SEL_MAX                              (0x000003FFU)

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR4_G1_RESETVAL                             (0x00000000U)

/* MDLXBAR4_G2 */

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR4_G2_SEL_MASK                             (0xFFFFFFFFU)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR4_G2_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR4_G2_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR4_G2_SEL_MAX                              (0xFFFFFFFFU)

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR4_G2_RESETVAL                             (0x00000000U)

/* MDLXBAR5_G0 */

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR5_G0_SEL_MASK                             (0x000003FFU)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR5_G0_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR5_G0_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR5_G0_SEL_MAX                              (0x000003FFU)

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR5_G0_RESETVAL                             (0x00000000U)

/* MDLXBAR5_G1 */

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR5_G1_SEL_MASK                             (0x000003FFU)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR5_G1_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR5_G1_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR5_G1_SEL_MAX                              (0x000003FFU)

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR5_G1_RESETVAL                             (0x00000000U)

/* MDLXBAR5_G2 */

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR5_G2_SEL_MASK                             (0xFFFFFFFFU)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR5_G2_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR5_G2_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR5_G2_SEL_MAX                              (0xFFFFFFFFU)

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR5_G2_RESETVAL                             (0x00000000U)

/* MDLXBAR6_G0 */

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR6_G0_SEL_MASK                             (0x000003FFU)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR6_G0_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR6_G0_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR6_G0_SEL_MAX                              (0x000003FFU)

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR6_G0_RESETVAL                             (0x00000000U)

/* MDLXBAR6_G1 */

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR6_G1_SEL_MASK                             (0x000003FFU)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR6_G1_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR6_G1_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR6_G1_SEL_MAX                              (0x000003FFU)

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR6_G1_RESETVAL                             (0x00000000U)

/* MDLXBAR6_G2 */

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR6_G2_SEL_MASK                             (0xFFFFFFFFU)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR6_G2_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR6_G2_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR6_G2_SEL_MAX                              (0xFFFFFFFFU)

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR6_G2_RESETVAL                             (0x00000000U)

/* MDLXBAR7_G0 */

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR7_G0_SEL_MASK                             (0x000003FFU)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR7_G0_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR7_G0_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR7_G0_SEL_MAX                              (0x000003FFU)

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR7_G0_RESETVAL                             (0x00000000U)

/* MDLXBAR7_G1 */

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR7_G1_SEL_MASK                             (0x000003FFU)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR7_G1_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR7_G1_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR7_G1_SEL_MAX                              (0x000003FFU)

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR7_G1_RESETVAL                             (0x00000000U)

/* MDLXBAR7_G2 */

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR7_G2_SEL_MASK                             (0xFFFFFFFFU)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR7_G2_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR7_G2_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR7_G2_SEL_MAX                              (0xFFFFFFFFU)

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR7_G2_RESETVAL                             (0x00000000U)

/* MDLXBAR8_G0 */

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR8_G0_SEL_MASK                             (0x000003FFU)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR8_G0_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR8_G0_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR8_G0_SEL_MAX                              (0x000003FFU)

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR8_G0_RESETVAL                             (0x00000000U)

/* MDLXBAR8_G1 */

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR8_G1_SEL_MASK                             (0x000003FFU)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR8_G1_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR8_G1_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR8_G1_SEL_MAX                              (0x000003FFU)

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR8_G1_RESETVAL                             (0x00000000U)

/* MDLXBAR8_G2 */

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR8_G2_SEL_MASK                             (0xFFFFFFFFU)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR8_G2_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR8_G2_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR8_G2_SEL_MAX                              (0xFFFFFFFFU)

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR8_G2_RESETVAL                             (0x00000000U)

/* MDLXBAR9_G0 */

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR9_G0_SEL_MASK                             (0x000003FFU)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR9_G0_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR9_G0_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR9_G0_SEL_MAX                              (0x000003FFU)

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR9_G0_RESETVAL                             (0x00000000U)

/* MDLXBAR9_G1 */

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR9_G1_SEL_MASK                             (0x000003FFU)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR9_G1_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR9_G1_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR9_G1_SEL_MAX                              (0x000003FFU)

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR9_G1_RESETVAL                             (0x00000000U)

/* MDLXBAR9_G2 */

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR9_G2_SEL_MASK                             (0xFFFFFFFFU)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR9_G2_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR9_G2_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR9_G2_SEL_MAX                              (0xFFFFFFFFU)

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR9_G2_RESETVAL                             (0x00000000U)

/* MDLXBAR10_G0 */

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR10_G0_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR10_G0_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR10_G0_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR10_G0_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR10_G0_RESETVAL                            (0x00000000U)

/* MDLXBAR10_G1 */

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR10_G1_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR10_G1_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR10_G1_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR10_G1_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR10_G1_RESETVAL                            (0x00000000U)

/* MDLXBAR10_G2 */

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR10_G2_SEL_MASK                            (0xFFFFFFFFU)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR10_G2_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR10_G2_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR10_G2_SEL_MAX                             (0xFFFFFFFFU)

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR10_G2_RESETVAL                            (0x00000000U)

/* MDLXBAR11_G0 */

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR11_G0_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR11_G0_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR11_G0_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR11_G0_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR11_G0_RESETVAL                            (0x00000000U)

/* MDLXBAR11_G1 */

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR11_G1_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR11_G1_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR11_G1_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR11_G1_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR11_G1_RESETVAL                            (0x00000000U)

/* MDLXBAR11_G2 */

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR11_G2_SEL_MASK                            (0xFFFFFFFFU)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR11_G2_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR11_G2_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR11_G2_SEL_MAX                             (0xFFFFFFFFU)

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR11_G2_RESETVAL                            (0x00000000U)

/* MDLXBAR12_G0 */

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR12_G0_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR12_G0_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR12_G0_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR12_G0_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR12_G0_RESETVAL                            (0x00000000U)

/* MDLXBAR12_G1 */

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR12_G1_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR12_G1_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR12_G1_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR12_G1_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR12_G1_RESETVAL                            (0x00000000U)

/* MDLXBAR12_G2 */

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR12_G2_SEL_MASK                            (0xFFFFFFFFU)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR12_G2_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR12_G2_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR12_G2_SEL_MAX                             (0xFFFFFFFFU)

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR12_G2_RESETVAL                            (0x00000000U)

/* MDLXBAR13_G0 */

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR13_G0_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR13_G0_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR13_G0_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR13_G0_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR13_G0_RESETVAL                            (0x00000000U)

/* MDLXBAR13_G1 */

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR13_G1_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR13_G1_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR13_G1_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR13_G1_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR13_G1_RESETVAL                            (0x00000000U)

/* MDLXBAR13_G2 */

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR13_G2_SEL_MASK                            (0xFFFFFFFFU)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR13_G2_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR13_G2_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR13_G2_SEL_MAX                             (0xFFFFFFFFU)

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR13_G2_RESETVAL                            (0x00000000U)

/* MDLXBAR14_G0 */

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR14_G0_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR14_G0_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR14_G0_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR14_G0_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR14_G0_RESETVAL                            (0x00000000U)

/* MDLXBAR14_G1 */

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR14_G1_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR14_G1_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR14_G1_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR14_G1_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR14_G1_RESETVAL                            (0x00000000U)

/* MDLXBAR14_G2 */

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR14_G2_SEL_MASK                            (0xFFFFFFFFU)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR14_G2_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR14_G2_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR14_G2_SEL_MAX                             (0xFFFFFFFFU)

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR14_G2_RESETVAL                            (0x00000000U)

/* MDLXBAR15_G0 */

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR15_G0_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR15_G0_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR15_G0_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR15_G0_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR15_G0_RESETVAL                            (0x00000000U)

/* MDLXBAR15_G1 */

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR15_G1_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR15_G1_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR15_G1_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR15_G1_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR15_G1_RESETVAL                            (0x00000000U)

/* MDLXBAR15_G2 */

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR15_G2_SEL_MASK                            (0xFFFFFFFFU)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR15_G2_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR15_G2_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_MDLXBAR_MDLXBAR15_G2_SEL_MAX                             (0xFFFFFFFFU)

#define CSL_CONTROLSS_MDLXBAR_MDLXBAR15_G2_RESETVAL                            (0x00000000U)

#ifdef __cplusplus
}
#endif
#endif
