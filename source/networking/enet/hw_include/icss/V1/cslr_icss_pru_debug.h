/********************************************************************
 * Copyright (C) 2013-2014 Texas Instruments Incorporated.
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
#ifndef CSLR_ICSSPRUDEBUG_H_
#define CSLR_ICSSPRUDEBUG_H_

#ifdef __cplusplus
extern "C"
{
#endif
#include <drivers/hw_include/cslr.h>
#include <drivers/hw_include/tistdtypes.h>


/**************************************************************************
* Register Overlay Structure for __ALL__
**************************************************************************/
typedef struct {
    volatile Uint32 GPR0;
    volatile Uint32 GPR1;
    volatile Uint32 GPR2;
    volatile Uint32 GPR3;
    volatile Uint32 GPR4;
    volatile Uint32 GPR5;
    volatile Uint32 GPR6;
    volatile Uint32 GPR7;
    volatile Uint32 GPR8;
    volatile Uint32 GPR9;
    volatile Uint32 GPR10;
    volatile Uint32 GPR11;
    volatile Uint32 GPR12;
    volatile Uint32 GPR13;
    volatile Uint32 GPR14;
    volatile Uint32 GPR15;
    volatile Uint32 GPR16;
    volatile Uint32 GPR17;
    volatile Uint32 GPR18;
    volatile Uint32 GPR19;
    volatile Uint32 GPR20;
    volatile Uint32 GPR21;
    volatile Uint32 GPR22;
    volatile Uint32 GPR23;
    volatile Uint32 GPR24;
    volatile Uint32 GPR25;
    volatile Uint32 GPR26;
    volatile Uint32 GPR27;
    volatile Uint32 GPR28;
    volatile Uint32 GPR29;
    volatile Uint32 GPR30;
    volatile Uint32 GPR31;
    volatile Uint32 CTR0;
    volatile Uint32 CTR1;
    volatile Uint32 CTR2;
    volatile Uint32 CTR3;
    volatile Uint32 CTR4;
    volatile Uint32 CTR5;
    volatile Uint32 CTR6;
    volatile Uint32 CTR7;
    volatile Uint32 CTR8;
    volatile Uint32 CTR9;
    volatile Uint32 CTR10;
    volatile Uint32 CTR11;
    volatile Uint32 CTR12;
    volatile Uint32 CTR13;
    volatile Uint32 CTR14;
    volatile Uint32 CTR15;
    volatile Uint32 CTR16;
    volatile Uint32 CTR17;
    volatile Uint32 CTR18;
    volatile Uint32 CTR19;
    volatile Uint32 CTR20;
    volatile Uint32 CTR21;
    volatile Uint32 CTR22;
    volatile Uint32 CTR23;
    volatile Uint32 CTR24;
    volatile Uint32 CTR25;
    volatile Uint32 CTR26;
    volatile Uint32 CTR27;
    volatile Uint32 CTR28;
    volatile Uint32 CTR29;
    volatile Uint32 CTR30;
    volatile Uint32 CTR31;
    volatile Uint8  RSVD0[756];
} CSL_IcssPruDebugRegs;


/**************************************************************************
* Register Macros
**************************************************************************/
#define CSL_ICSSPRUDEBUG_GPR0                                   (0x0U)
#define CSL_ICSSPRUDEBUG_GPR1                                   (0x4U)
#define CSL_ICSSPRUDEBUG_GPR2                                   (0x8U)
#define CSL_ICSSPRUDEBUG_GPR3                                   (0xCU)
#define CSL_ICSSPRUDEBUG_GPR4                                   (0x10U)
#define CSL_ICSSPRUDEBUG_GPR5                                   (0x14U)
#define CSL_ICSSPRUDEBUG_GPR6                                   (0x18U)
#define CSL_ICSSPRUDEBUG_GPR7                                   (0x1CU)
#define CSL_ICSSPRUDEBUG_GPR8                                   (0x20U)
#define CSL_ICSSPRUDEBUG_GPR9                                   (0x24U)
#define CSL_ICSSPRUDEBUG_GPR10                                  (0x28U)
#define CSL_ICSSPRUDEBUG_GPR11                                  (0x2CU)
#define CSL_ICSSPRUDEBUG_GPR12                                  (0x30U)
#define CSL_ICSSPRUDEBUG_GPR13                                  (0x34U)
#define CSL_ICSSPRUDEBUG_GPR14                                  (0x38U)
#define CSL_ICSSPRUDEBUG_GPR15                                  (0x3CU)
#define CSL_ICSSPRUDEBUG_GPR16                                  (0x40U)
#define CSL_ICSSPRUDEBUG_GPR17                                  (0x44U)
#define CSL_ICSSPRUDEBUG_GPR18                                  (0x48U)
#define CSL_ICSSPRUDEBUG_GPR19                                  (0x4CU)
#define CSL_ICSSPRUDEBUG_GPR20                                  (0x50U)
#define CSL_ICSSPRUDEBUG_GPR21                                  (0x54U)
#define CSL_ICSSPRUDEBUG_GPR22                                  (0x58U)
#define CSL_ICSSPRUDEBUG_GPR23                                  (0x5CU)
#define CSL_ICSSPRUDEBUG_GPR24                                  (0x60U)
#define CSL_ICSSPRUDEBUG_GPR25                                  (0x64U)
#define CSL_ICSSPRUDEBUG_GPR26                                  (0x68U)
#define CSL_ICSSPRUDEBUG_GPR27                                  (0x6CU)
#define CSL_ICSSPRUDEBUG_GPR28                                  (0x70U)
#define CSL_ICSSPRUDEBUG_GPR29                                  (0x74U)
#define CSL_ICSSPRUDEBUG_GPR30                                  (0x78U)
#define CSL_ICSSPRUDEBUG_GPR31                                  (0x7CU)
#define CSL_ICSSPRUDEBUG_CTR0                                   (0x80U)
#define CSL_ICSSPRUDEBUG_CTR1                                   (0x84U)
#define CSL_ICSSPRUDEBUG_CTR2                                   (0x88U)
#define CSL_ICSSPRUDEBUG_CTR3                                   (0x8CU)
#define CSL_ICSSPRUDEBUG_CTR4                                   (0x90U)
#define CSL_ICSSPRUDEBUG_CTR5                                   (0x94U)
#define CSL_ICSSPRUDEBUG_CTR6                                   (0x98U)
#define CSL_ICSSPRUDEBUG_CTR7                                   (0x9CU)
#define CSL_ICSSPRUDEBUG_CTR8                                   (0xA0U)
#define CSL_ICSSPRUDEBUG_CTR9                                   (0xA4U)
#define CSL_ICSSPRUDEBUG_CTR10                                  (0xA8U)
#define CSL_ICSSPRUDEBUG_CTR11                                  (0xACU)
#define CSL_ICSSPRUDEBUG_CTR12                                  (0xB0U)
#define CSL_ICSSPRUDEBUG_CTR13                                  (0xB4U)
#define CSL_ICSSPRUDEBUG_CTR14                                  (0xB8U)
#define CSL_ICSSPRUDEBUG_CTR15                                  (0xBCU)
#define CSL_ICSSPRUDEBUG_CTR16                                  (0xC0U)
#define CSL_ICSSPRUDEBUG_CTR17                                  (0xC4U)
#define CSL_ICSSPRUDEBUG_CTR18                                  (0xC8U)
#define CSL_ICSSPRUDEBUG_CTR19                                  (0xCCU)
#define CSL_ICSSPRUDEBUG_CTR20                                  (0xD0U)
#define CSL_ICSSPRUDEBUG_CTR21                                  (0xD4U)
#define CSL_ICSSPRUDEBUG_CTR22                                  (0xD8U)
#define CSL_ICSSPRUDEBUG_CTR23                                  (0xDCU)
#define CSL_ICSSPRUDEBUG_CTR24                                  (0xE0U)
#define CSL_ICSSPRUDEBUG_CTR25                                  (0xE4U)
#define CSL_ICSSPRUDEBUG_CTR26                                  (0xE8U)
#define CSL_ICSSPRUDEBUG_CTR27                                  (0xECU)
#define CSL_ICSSPRUDEBUG_CTR28                                  (0xF0U)
#define CSL_ICSSPRUDEBUG_CTR29                                  (0xF4U)
#define CSL_ICSSPRUDEBUG_CTR30                                  (0xF8U)
#define CSL_ICSSPRUDEBUG_CTR31                                  (0xFCU)


/**************************************************************************
* Field Definition Macros
**************************************************************************/

/* GPR0 */

#define CSL_ICSSPRUDEBUG_GPR0_GP_MASK                           (0xFFFFFFFFU)
#define CSL_ICSSPRUDEBUG_GPR0_GP_SHIFT                          (0U)
#define CSL_ICSSPRUDEBUG_GPR0_GP_RESETVAL                       (0x00000000U)
#define CSL_ICSSPRUDEBUG_GPR0_GP_MAX                            (0xffffffffU)

#define CSL_ICSSPRUDEBUG_GPR0_RESETVAL                          (0x00000000U)

/* GPR1 */

#define CSL_ICSSPRUDEBUG_GPR1_GP_MASK                           (0xFFFFFFFFU)
#define CSL_ICSSPRUDEBUG_GPR1_GP_SHIFT                          (0U)
#define CSL_ICSSPRUDEBUG_GPR1_GP_RESETVAL                       (0x00000000U)
#define CSL_ICSSPRUDEBUG_GPR1_GP_MAX                            (0xffffffffU)

#define CSL_ICSSPRUDEBUG_GPR1_RESETVAL                          (0x00000000U)

/* GPR2 */

#define CSL_ICSSPRUDEBUG_GPR2_GP_MASK                           (0xFFFFFFFFU)
#define CSL_ICSSPRUDEBUG_GPR2_GP_SHIFT                          (0U)
#define CSL_ICSSPRUDEBUG_GPR2_GP_RESETVAL                       (0x00000000U)
#define CSL_ICSSPRUDEBUG_GPR2_GP_MAX                            (0xffffffffU)

#define CSL_ICSSPRUDEBUG_GPR2_RESETVAL                          (0x00000000U)

/* GPR3 */

#define CSL_ICSSPRUDEBUG_GPR3_GP_MASK                           (0xFFFFFFFFU)
#define CSL_ICSSPRUDEBUG_GPR3_GP_SHIFT                          (0U)
#define CSL_ICSSPRUDEBUG_GPR3_GP_RESETVAL                       (0x00000000U)
#define CSL_ICSSPRUDEBUG_GPR3_GP_MAX                            (0xffffffffU)

#define CSL_ICSSPRUDEBUG_GPR3_RESETVAL                          (0x00000000U)

/* GPR4 */

#define CSL_ICSSPRUDEBUG_GPR4_GP_MASK                           (0xFFFFFFFFU)
#define CSL_ICSSPRUDEBUG_GPR4_GP_SHIFT                          (0U)
#define CSL_ICSSPRUDEBUG_GPR4_GP_RESETVAL                       (0x00000000U)
#define CSL_ICSSPRUDEBUG_GPR4_GP_MAX                            (0xffffffffU)

#define CSL_ICSSPRUDEBUG_GPR4_RESETVAL                          (0x00000000U)

/* GPR5 */

#define CSL_ICSSPRUDEBUG_GPR5_GP_MASK                           (0xFFFFFFFFU)
#define CSL_ICSSPRUDEBUG_GPR5_GP_SHIFT                          (0U)
#define CSL_ICSSPRUDEBUG_GPR5_GP_RESETVAL                       (0x00000000U)
#define CSL_ICSSPRUDEBUG_GPR5_GP_MAX                            (0xffffffffU)

#define CSL_ICSSPRUDEBUG_GPR5_RESETVAL                          (0x00000000U)

/* GPR6 */

#define CSL_ICSSPRUDEBUG_GPR6_GP_MASK                           (0xFFFFFFFFU)
#define CSL_ICSSPRUDEBUG_GPR6_GP_SHIFT                          (0U)
#define CSL_ICSSPRUDEBUG_GPR6_GP_RESETVAL                       (0x00000000U)
#define CSL_ICSSPRUDEBUG_GPR6_GP_MAX                            (0xffffffffU)

#define CSL_ICSSPRUDEBUG_GPR6_RESETVAL                          (0x00000000U)

/* GPR7 */

#define CSL_ICSSPRUDEBUG_GPR7_GP_MASK                           (0xFFFFFFFFU)
#define CSL_ICSSPRUDEBUG_GPR7_GP_SHIFT                          (0U)
#define CSL_ICSSPRUDEBUG_GPR7_GP_RESETVAL                       (0x00000000U)
#define CSL_ICSSPRUDEBUG_GPR7_GP_MAX                            (0xffffffffU)

#define CSL_ICSSPRUDEBUG_GPR7_RESETVAL                          (0x00000000U)

/* GPR8 */

#define CSL_ICSSPRUDEBUG_GPR8_GP_MASK                           (0xFFFFFFFFU)
#define CSL_ICSSPRUDEBUG_GPR8_GP_SHIFT                          (0U)
#define CSL_ICSSPRUDEBUG_GPR8_GP_RESETVAL                       (0x00000000U)
#define CSL_ICSSPRUDEBUG_GPR8_GP_MAX                            (0xffffffffU)

#define CSL_ICSSPRUDEBUG_GPR8_RESETVAL                          (0x00000000U)

/* GPR9 */

#define CSL_ICSSPRUDEBUG_GPR9_GP_MASK                           (0xFFFFFFFFU)
#define CSL_ICSSPRUDEBUG_GPR9_GP_SHIFT                          (0U)
#define CSL_ICSSPRUDEBUG_GPR9_GP_RESETVAL                       (0x00000000U)
#define CSL_ICSSPRUDEBUG_GPR9_GP_MAX                            (0xffffffffU)

#define CSL_ICSSPRUDEBUG_GPR9_RESETVAL                          (0x00000000U)

/* GPR10 */

#define CSL_ICSSPRUDEBUG_GPR10_GP_MASK                          (0xFFFFFFFFU)
#define CSL_ICSSPRUDEBUG_GPR10_GP_SHIFT                         (0U)
#define CSL_ICSSPRUDEBUG_GPR10_GP_RESETVAL                      (0x00000000U)
#define CSL_ICSSPRUDEBUG_GPR10_GP_MAX                           (0xffffffffU)

#define CSL_ICSSPRUDEBUG_GPR10_RESETVAL                         (0x00000000U)

/* GPR11 */

#define CSL_ICSSPRUDEBUG_GPR11_GP_MASK                          (0xFFFFFFFFU)
#define CSL_ICSSPRUDEBUG_GPR11_GP_SHIFT                         (0U)
#define CSL_ICSSPRUDEBUG_GPR11_GP_RESETVAL                      (0x00000000U)
#define CSL_ICSSPRUDEBUG_GPR11_GP_MAX                           (0xffffffffU)

#define CSL_ICSSPRUDEBUG_GPR11_RESETVAL                         (0x00000000U)

/* GPR12 */

#define CSL_ICSSPRUDEBUG_GPR12_GP_MASK                          (0xFFFFFFFFU)
#define CSL_ICSSPRUDEBUG_GPR12_GP_SHIFT                         (0U)
#define CSL_ICSSPRUDEBUG_GPR12_GP_RESETVAL                      (0x00000000U)
#define CSL_ICSSPRUDEBUG_GPR12_GP_MAX                           (0xffffffffU)

#define CSL_ICSSPRUDEBUG_GPR12_RESETVAL                         (0x00000000U)

/* GPR13 */

#define CSL_ICSSPRUDEBUG_GPR13_GP_MASK                          (0xFFFFFFFFU)
#define CSL_ICSSPRUDEBUG_GPR13_GP_SHIFT                         (0U)
#define CSL_ICSSPRUDEBUG_GPR13_GP_RESETVAL                      (0x00000000U)
#define CSL_ICSSPRUDEBUG_GPR13_GP_MAX                           (0xffffffffU)

#define CSL_ICSSPRUDEBUG_GPR13_RESETVAL                         (0x00000000U)

/* GPR14 */

#define CSL_ICSSPRUDEBUG_GPR14_GP_MASK                          (0xFFFFFFFFU)
#define CSL_ICSSPRUDEBUG_GPR14_GP_SHIFT                         (0U)
#define CSL_ICSSPRUDEBUG_GPR14_GP_RESETVAL                      (0x00000000U)
#define CSL_ICSSPRUDEBUG_GPR14_GP_MAX                           (0xffffffffU)

#define CSL_ICSSPRUDEBUG_GPR14_RESETVAL                         (0x00000000U)

/* GPR15 */

#define CSL_ICSSPRUDEBUG_GPR15_GP_MASK                          (0xFFFFFFFFU)
#define CSL_ICSSPRUDEBUG_GPR15_GP_SHIFT                         (0U)
#define CSL_ICSSPRUDEBUG_GPR15_GP_RESETVAL                      (0x00000000U)
#define CSL_ICSSPRUDEBUG_GPR15_GP_MAX                           (0xffffffffU)

#define CSL_ICSSPRUDEBUG_GPR15_RESETVAL                         (0x00000000U)

/* GPR16 */

#define CSL_ICSSPRUDEBUG_GPR16_GP_MASK                          (0xFFFFFFFFU)
#define CSL_ICSSPRUDEBUG_GPR16_GP_SHIFT                         (0U)
#define CSL_ICSSPRUDEBUG_GPR16_GP_RESETVAL                      (0x00000000U)
#define CSL_ICSSPRUDEBUG_GPR16_GP_MAX                           (0xffffffffU)

#define CSL_ICSSPRUDEBUG_GPR16_RESETVAL                         (0x00000000U)

/* GPR17 */

#define CSL_ICSSPRUDEBUG_GPR17_GP_MASK                          (0xFFFFFFFFU)
#define CSL_ICSSPRUDEBUG_GPR17_GP_SHIFT                         (0U)
#define CSL_ICSSPRUDEBUG_GPR17_GP_RESETVAL                      (0x00000000U)
#define CSL_ICSSPRUDEBUG_GPR17_GP_MAX                           (0xffffffffU)

#define CSL_ICSSPRUDEBUG_GPR17_RESETVAL                         (0x00000000U)

/* GPR18 */

#define CSL_ICSSPRUDEBUG_GPR18_GP_MASK                          (0xFFFFFFFFU)
#define CSL_ICSSPRUDEBUG_GPR18_GP_SHIFT                         (0U)
#define CSL_ICSSPRUDEBUG_GPR18_GP_RESETVAL                      (0x00000000U)
#define CSL_ICSSPRUDEBUG_GPR18_GP_MAX                           (0xffffffffU)

#define CSL_ICSSPRUDEBUG_GPR18_RESETVAL                         (0x00000000U)

/* GPR19 */

#define CSL_ICSSPRUDEBUG_GPR19_GP_MASK                          (0xFFFFFFFFU)
#define CSL_ICSSPRUDEBUG_GPR19_GP_SHIFT                         (0U)
#define CSL_ICSSPRUDEBUG_GPR19_GP_RESETVAL                      (0x00000000U)
#define CSL_ICSSPRUDEBUG_GPR19_GP_MAX                           (0xffffffffU)

#define CSL_ICSSPRUDEBUG_GPR19_RESETVAL                         (0x00000000U)

/* GPR20 */

#define CSL_ICSSPRUDEBUG_GPR20_GP_MASK                          (0xFFFFFFFFU)
#define CSL_ICSSPRUDEBUG_GPR20_GP_SHIFT                         (0U)
#define CSL_ICSSPRUDEBUG_GPR20_GP_RESETVAL                      (0x00000000U)
#define CSL_ICSSPRUDEBUG_GPR20_GP_MAX                           (0xffffffffU)

#define CSL_ICSSPRUDEBUG_GPR20_RESETVAL                         (0x00000000U)

/* GPR21 */

#define CSL_ICSSPRUDEBUG_GPR21_GP_MASK                          (0xFFFFFFFFU)
#define CSL_ICSSPRUDEBUG_GPR21_GP_SHIFT                         (0U)
#define CSL_ICSSPRUDEBUG_GPR21_GP_RESETVAL                      (0x00000000U)
#define CSL_ICSSPRUDEBUG_GPR21_GP_MAX                           (0xffffffffU)

#define CSL_ICSSPRUDEBUG_GPR21_RESETVAL                         (0x00000000U)

/* GPR22 */

#define CSL_ICSSPRUDEBUG_GPR22_GP_MASK                          (0xFFFFFFFFU)
#define CSL_ICSSPRUDEBUG_GPR22_GP_SHIFT                         (0U)
#define CSL_ICSSPRUDEBUG_GPR22_GP_RESETVAL                      (0x00000000U)
#define CSL_ICSSPRUDEBUG_GPR22_GP_MAX                           (0xffffffffU)

#define CSL_ICSSPRUDEBUG_GPR22_RESETVAL                         (0x00000000U)

/* GPR23 */

#define CSL_ICSSPRUDEBUG_GPR23_GP_MASK                          (0xFFFFFFFFU)
#define CSL_ICSSPRUDEBUG_GPR23_GP_SHIFT                         (0U)
#define CSL_ICSSPRUDEBUG_GPR23_GP_RESETVAL                      (0x00000000U)
#define CSL_ICSSPRUDEBUG_GPR23_GP_MAX                           (0xffffffffU)

#define CSL_ICSSPRUDEBUG_GPR23_RESETVAL                         (0x00000000U)

/* GPR24 */

#define CSL_ICSSPRUDEBUG_GPR24_GP_MASK                          (0xFFFFFFFFU)
#define CSL_ICSSPRUDEBUG_GPR24_GP_SHIFT                         (0U)
#define CSL_ICSSPRUDEBUG_GPR24_GP_RESETVAL                      (0x00000000U)
#define CSL_ICSSPRUDEBUG_GPR24_GP_MAX                           (0xffffffffU)

#define CSL_ICSSPRUDEBUG_GPR24_RESETVAL                         (0x00000000U)

/* GPR25 */

#define CSL_ICSSPRUDEBUG_GPR25_GP_MASK                          (0xFFFFFFFFU)
#define CSL_ICSSPRUDEBUG_GPR25_GP_SHIFT                         (0U)
#define CSL_ICSSPRUDEBUG_GPR25_GP_RESETVAL                      (0x00000000U)
#define CSL_ICSSPRUDEBUG_GPR25_GP_MAX                           (0xffffffffU)

#define CSL_ICSSPRUDEBUG_GPR25_RESETVAL                         (0x00000000U)

/* GPR26 */

#define CSL_ICSSPRUDEBUG_GPR26_GP_MASK                          (0xFFFFFFFFU)
#define CSL_ICSSPRUDEBUG_GPR26_GP_SHIFT                         (0U)
#define CSL_ICSSPRUDEBUG_GPR26_GP_RESETVAL                      (0x00000000U)
#define CSL_ICSSPRUDEBUG_GPR26_GP_MAX                           (0xffffffffU)

#define CSL_ICSSPRUDEBUG_GPR26_RESETVAL                         (0x00000000U)

/* GPR27 */

#define CSL_ICSSPRUDEBUG_GPR27_GP_MASK                          (0xFFFFFFFFU)
#define CSL_ICSSPRUDEBUG_GPR27_GP_SHIFT                         (0U)
#define CSL_ICSSPRUDEBUG_GPR27_GP_RESETVAL                      (0x00000000U)
#define CSL_ICSSPRUDEBUG_GPR27_GP_MAX                           (0xffffffffU)

#define CSL_ICSSPRUDEBUG_GPR27_RESETVAL                         (0x00000000U)

/* GPR28 */

#define CSL_ICSSPRUDEBUG_GPR28_GP_MASK                          (0xFFFFFFFFU)
#define CSL_ICSSPRUDEBUG_GPR28_GP_SHIFT                         (0U)
#define CSL_ICSSPRUDEBUG_GPR28_GP_RESETVAL                      (0x00000000U)
#define CSL_ICSSPRUDEBUG_GPR28_GP_MAX                           (0xffffffffU)

#define CSL_ICSSPRUDEBUG_GPR28_RESETVAL                         (0x00000000U)

/* GPR29 */

#define CSL_ICSSPRUDEBUG_GPR29_GP_MASK                          (0xFFFFFFFFU)
#define CSL_ICSSPRUDEBUG_GPR29_GP_SHIFT                         (0U)
#define CSL_ICSSPRUDEBUG_GPR29_GP_RESETVAL                      (0x00000000U)
#define CSL_ICSSPRUDEBUG_GPR29_GP_MAX                           (0xffffffffU)

#define CSL_ICSSPRUDEBUG_GPR29_RESETVAL                         (0x00000000U)

/* GPR30 */

#define CSL_ICSSPRUDEBUG_GPR30_GP_MASK                          (0xFFFFFFFFU)
#define CSL_ICSSPRUDEBUG_GPR30_GP_SHIFT                         (0U)
#define CSL_ICSSPRUDEBUG_GPR30_GP_RESETVAL                      (0x00000000U)
#define CSL_ICSSPRUDEBUG_GPR30_GP_MAX                           (0xffffffffU)

#define CSL_ICSSPRUDEBUG_GPR30_RESETVAL                         (0x00000000U)

/* GPR31 */

#define CSL_ICSSPRUDEBUG_GPR31_RESETVAL                         (0x00000000U)

/* CTR0 */

#define CSL_ICSSPRUDEBUG_CTR0_CT_MASK                           (0xFFFFFFFFU)
#define CSL_ICSSPRUDEBUG_CTR0_CT_SHIFT                          (0U)
#define CSL_ICSSPRUDEBUG_CTR0_CT_RESETVAL                       (0x00020000U)
#define CSL_ICSSPRUDEBUG_CTR0_CT_MAX                            (0xffffffffU)

#define CSL_ICSSPRUDEBUG_CTR0_RESETVAL                          (0x00020000U)

/* CTR1 */

#define CSL_ICSSPRUDEBUG_CTR1_CT_MASK                           (0xFFFFFFFFU)
#define CSL_ICSSPRUDEBUG_CTR1_CT_SHIFT                          (0U)
#define CSL_ICSSPRUDEBUG_CTR1_CT_RESETVAL                       (0x48040000U)
#define CSL_ICSSPRUDEBUG_CTR1_CT_MAX                            (0xffffffffU)

#define CSL_ICSSPRUDEBUG_CTR1_RESETVAL                          (0x48040000U)

/* CTR2 */

#define CSL_ICSSPRUDEBUG_CTR2_CT_MASK                           (0xFFFFFFFFU)
#define CSL_ICSSPRUDEBUG_CTR2_CT_SHIFT                          (0U)
#define CSL_ICSSPRUDEBUG_CTR2_CT_RESETVAL                       (0x4802a000U)
#define CSL_ICSSPRUDEBUG_CTR2_CT_MAX                            (0xffffffffU)

#define CSL_ICSSPRUDEBUG_CTR2_RESETVAL                          (0x4802a000U)

/* CTR3 */

#define CSL_ICSSPRUDEBUG_CTR3_CT_MASK                           (0xFFFFFFFFU)
#define CSL_ICSSPRUDEBUG_CTR3_CT_SHIFT                          (0U)
#define CSL_ICSSPRUDEBUG_CTR3_CT_RESETVAL                       (0x00030000U)
#define CSL_ICSSPRUDEBUG_CTR3_CT_MAX                            (0xffffffffU)

#define CSL_ICSSPRUDEBUG_CTR3_RESETVAL                          (0x00030000U)

/* CTR4 */

#define CSL_ICSSPRUDEBUG_CTR4_CT_MASK                           (0xFFFFFFFFU)
#define CSL_ICSSPRUDEBUG_CTR4_CT_SHIFT                          (0U)
#define CSL_ICSSPRUDEBUG_CTR4_CT_RESETVAL                       (0x00026000U)
#define CSL_ICSSPRUDEBUG_CTR4_CT_MAX                            (0xffffffffU)

#define CSL_ICSSPRUDEBUG_CTR4_RESETVAL                          (0x00026000U)

/* CTR5 */

#define CSL_ICSSPRUDEBUG_CTR5_CT_MASK                           (0xFFFFFFFFU)
#define CSL_ICSSPRUDEBUG_CTR5_CT_SHIFT                          (0U)
#define CSL_ICSSPRUDEBUG_CTR5_CT_RESETVAL                       (0x48060000U)
#define CSL_ICSSPRUDEBUG_CTR5_CT_MAX                            (0xffffffffU)

#define CSL_ICSSPRUDEBUG_CTR5_RESETVAL                          (0x48060000U)

/* CTR6 */

#define CSL_ICSSPRUDEBUG_CTR6_CT_MASK                           (0xFFFFFFFFU)
#define CSL_ICSSPRUDEBUG_CTR6_CT_SHIFT                          (0U)
#define CSL_ICSSPRUDEBUG_CTR6_CT_RESETVAL                       (0x48030000U)
#define CSL_ICSSPRUDEBUG_CTR6_CT_MAX                            (0xffffffffU)

#define CSL_ICSSPRUDEBUG_CTR6_RESETVAL                          (0x48030000U)

/* CTR7 */

#define CSL_ICSSPRUDEBUG_CTR7_CT_MASK                           (0xFFFFFFFFU)
#define CSL_ICSSPRUDEBUG_CTR7_CT_SHIFT                          (0U)
#define CSL_ICSSPRUDEBUG_CTR7_CT_RESETVAL                       (0x00028000U)
#define CSL_ICSSPRUDEBUG_CTR7_CT_MAX                            (0xffffffffU)

#define CSL_ICSSPRUDEBUG_CTR7_RESETVAL                          (0x00028000U)

/* CTR8 */

#define CSL_ICSSPRUDEBUG_CTR8_CT_MASK                           (0xFFFFFFFFU)
#define CSL_ICSSPRUDEBUG_CTR8_CT_SHIFT                          (0U)
#define CSL_ICSSPRUDEBUG_CTR8_CT_RESETVAL                       (0x46000000U)
#define CSL_ICSSPRUDEBUG_CTR8_CT_MAX                            (0xffffffffU)

#define CSL_ICSSPRUDEBUG_CTR8_RESETVAL                          (0x46000000U)

/* CTR9 */

#define CSL_ICSSPRUDEBUG_CTR9_CT_MASK                           (0xFFFFFFFFU)
#define CSL_ICSSPRUDEBUG_CTR9_CT_SHIFT                          (0U)
#define CSL_ICSSPRUDEBUG_CTR9_CT_RESETVAL                       (0x4a100000U)
#define CSL_ICSSPRUDEBUG_CTR9_CT_MAX                            (0xffffffffU)

#define CSL_ICSSPRUDEBUG_CTR9_RESETVAL                          (0x4a100000U)

/* CTR10 */

#define CSL_ICSSPRUDEBUG_CTR10_CT_MASK                          (0xFFFFFFFFU)
#define CSL_ICSSPRUDEBUG_CTR10_CT_SHIFT                         (0U)
#define CSL_ICSSPRUDEBUG_CTR10_CT_RESETVAL                      (0x48318000U)
#define CSL_ICSSPRUDEBUG_CTR10_CT_MAX                           (0xffffffffU)

#define CSL_ICSSPRUDEBUG_CTR10_RESETVAL                         (0x48318000U)

/* CTR11 */

#define CSL_ICSSPRUDEBUG_CTR11_CT_MASK                          (0xFFFFFFFFU)
#define CSL_ICSSPRUDEBUG_CTR11_CT_SHIFT                         (0U)
#define CSL_ICSSPRUDEBUG_CTR11_CT_RESETVAL                      (0x48022000U)
#define CSL_ICSSPRUDEBUG_CTR11_CT_MAX                           (0xffffffffU)

#define CSL_ICSSPRUDEBUG_CTR11_RESETVAL                         (0x48022000U)

/* CTR12 */

#define CSL_ICSSPRUDEBUG_CTR12_CT_MASK                          (0xFFFFFFFFU)
#define CSL_ICSSPRUDEBUG_CTR12_CT_SHIFT                         (0U)
#define CSL_ICSSPRUDEBUG_CTR12_CT_RESETVAL                      (0x48024000U)
#define CSL_ICSSPRUDEBUG_CTR12_CT_MAX                           (0xffffffffU)

#define CSL_ICSSPRUDEBUG_CTR12_RESETVAL                         (0x48024000U)

/* CTR13 */

#define CSL_ICSSPRUDEBUG_CTR13_CT_MASK                          (0xFFFFFFFFU)
#define CSL_ICSSPRUDEBUG_CTR13_CT_SHIFT                         (0U)
#define CSL_ICSSPRUDEBUG_CTR13_CT_RESETVAL                      (0x48310000U)
#define CSL_ICSSPRUDEBUG_CTR13_CT_MAX                           (0xffffffffU)

#define CSL_ICSSPRUDEBUG_CTR13_RESETVAL                         (0x48310000U)

/* CTR14 */

#define CSL_ICSSPRUDEBUG_CTR14_CT_MASK                          (0xFFFFFFFFU)
#define CSL_ICSSPRUDEBUG_CTR14_CT_SHIFT                         (0U)
#define CSL_ICSSPRUDEBUG_CTR14_CT_RESETVAL                      (0x481cc000U)
#define CSL_ICSSPRUDEBUG_CTR14_CT_MAX                           (0xffffffffU)

#define CSL_ICSSPRUDEBUG_CTR14_RESETVAL                         (0x481cc000U)

/* CTR15 */

#define CSL_ICSSPRUDEBUG_CTR15_CT_MASK                          (0xFFFFFFFFU)
#define CSL_ICSSPRUDEBUG_CTR15_CT_SHIFT                         (0U)
#define CSL_ICSSPRUDEBUG_CTR15_CT_RESETVAL                      (0x481d0000U)
#define CSL_ICSSPRUDEBUG_CTR15_CT_MAX                           (0xffffffffU)

#define CSL_ICSSPRUDEBUG_CTR15_RESETVAL                         (0x481d0000U)

/* CTR16 */

#define CSL_ICSSPRUDEBUG_CTR16_CT_MASK                          (0xFFFFFFFFU)
#define CSL_ICSSPRUDEBUG_CTR16_CT_SHIFT                         (0U)
#define CSL_ICSSPRUDEBUG_CTR16_CT_RESETVAL                      (0x481a0000U)
#define CSL_ICSSPRUDEBUG_CTR16_CT_MAX                           (0xffffffffU)

#define CSL_ICSSPRUDEBUG_CTR16_RESETVAL                         (0x481a0000U)

/* CTR17 */

#define CSL_ICSSPRUDEBUG_CTR17_CT_MASK                          (0xFFFFFFFFU)
#define CSL_ICSSPRUDEBUG_CTR17_CT_SHIFT                         (0U)
#define CSL_ICSSPRUDEBUG_CTR17_CT_RESETVAL                      (0x4819c000U)
#define CSL_ICSSPRUDEBUG_CTR17_CT_MAX                           (0xffffffffU)

#define CSL_ICSSPRUDEBUG_CTR17_RESETVAL                         (0x4819c000U)

/* CTR18 */

#define CSL_ICSSPRUDEBUG_CTR18_CT_MASK                          (0xFFFFFFFFU)
#define CSL_ICSSPRUDEBUG_CTR18_CT_SHIFT                         (0U)
#define CSL_ICSSPRUDEBUG_CTR18_CT_RESETVAL                      (0x48300000U)
#define CSL_ICSSPRUDEBUG_CTR18_CT_MAX                           (0xffffffffU)

#define CSL_ICSSPRUDEBUG_CTR18_RESETVAL                         (0x48300000U)

/* CTR19 */

#define CSL_ICSSPRUDEBUG_CTR19_CT_MASK                          (0xFFFFFFFFU)
#define CSL_ICSSPRUDEBUG_CTR19_CT_SHIFT                         (0U)
#define CSL_ICSSPRUDEBUG_CTR19_CT_RESETVAL                      (0x48302000U)
#define CSL_ICSSPRUDEBUG_CTR19_CT_MAX                           (0xffffffffU)

#define CSL_ICSSPRUDEBUG_CTR19_RESETVAL                         (0x48302000U)

/* CTR20 */

#define CSL_ICSSPRUDEBUG_CTR20_CT_MASK                          (0xFFFFFFFFU)
#define CSL_ICSSPRUDEBUG_CTR20_CT_SHIFT                         (0U)
#define CSL_ICSSPRUDEBUG_CTR20_CT_RESETVAL                      (0x48304000U)
#define CSL_ICSSPRUDEBUG_CTR20_CT_MAX                           (0xffffffffU)

#define CSL_ICSSPRUDEBUG_CTR20_RESETVAL                         (0x48304000U)

/* CTR21 */

#define CSL_ICSSPRUDEBUG_CTR21_CT_MASK                          (0xFFFFFFFFU)
#define CSL_ICSSPRUDEBUG_CTR21_CT_SHIFT                         (0U)
#define CSL_ICSSPRUDEBUG_CTR21_CT_RESETVAL                      (0x00032400U)
#define CSL_ICSSPRUDEBUG_CTR21_CT_MAX                           (0xffffffffU)

#define CSL_ICSSPRUDEBUG_CTR21_RESETVAL                         (0x00032400U)

/* CTR22 */

#define CSL_ICSSPRUDEBUG_CTR22_CT_MASK                          (0xFFFFFFFFU)
#define CSL_ICSSPRUDEBUG_CTR22_CT_SHIFT                         (0U)
#define CSL_ICSSPRUDEBUG_CTR22_CT_RESETVAL                      (0x480c8000U)
#define CSL_ICSSPRUDEBUG_CTR22_CT_MAX                           (0xffffffffU)

#define CSL_ICSSPRUDEBUG_CTR22_RESETVAL                         (0x480c8000U)

/* CTR23 */

#define CSL_ICSSPRUDEBUG_CTR23_CT_MASK                          (0xFFFFFFFFU)
#define CSL_ICSSPRUDEBUG_CTR23_CT_SHIFT                         (0U)
#define CSL_ICSSPRUDEBUG_CTR23_CT_RESETVAL                      (0x480ca000U)
#define CSL_ICSSPRUDEBUG_CTR23_CT_MAX                           (0xffffffffU)

#define CSL_ICSSPRUDEBUG_CTR23_RESETVAL                         (0x480ca000U)

/* CTR24 */

#define CSL_ICSSPRUDEBUG_CTR24_CT_MASK                          (0xFFFFFFFFU)
#define CSL_ICSSPRUDEBUG_CTR24_CT_SHIFT                         (0U)
#define CSL_ICSSPRUDEBUG_CTR24_CT_RESETVAL                      (0x00000000U)
#define CSL_ICSSPRUDEBUG_CTR24_CT_MAX                           (0xffffffffU)

#define CSL_ICSSPRUDEBUG_CTR24_RESETVAL                         (0x00000000U)

/* CTR25 */

#define CSL_ICSSPRUDEBUG_CTR25_CT_MASK                          (0xFFFFFFFFU)
#define CSL_ICSSPRUDEBUG_CTR25_CT_SHIFT                         (0U)
#define CSL_ICSSPRUDEBUG_CTR25_CT_RESETVAL                      (0x00000000U)
#define CSL_ICSSPRUDEBUG_CTR25_CT_MAX                           (0xffffffffU)

#define CSL_ICSSPRUDEBUG_CTR25_RESETVAL                         (0x00000000U)

/* CTR26 */

#define CSL_ICSSPRUDEBUG_CTR26_CT_MASK                          (0xFFFFFFFFU)
#define CSL_ICSSPRUDEBUG_CTR26_CT_SHIFT                         (0U)
#define CSL_ICSSPRUDEBUG_CTR26_CT_RESETVAL                      (0x00000000U)
#define CSL_ICSSPRUDEBUG_CTR26_CT_MAX                           (0xffffffffU)

#define CSL_ICSSPRUDEBUG_CTR26_RESETVAL                         (0x00000000U)

/* CTR27 */

#define CSL_ICSSPRUDEBUG_CTR27_CT_MASK                          (0xFFFFFFFFU)
#define CSL_ICSSPRUDEBUG_CTR27_CT_SHIFT                         (0U)
#define CSL_ICSSPRUDEBUG_CTR27_CT_RESETVAL                      (0x00000000U)
#define CSL_ICSSPRUDEBUG_CTR27_CT_MAX                           (0xffffffffU)

#define CSL_ICSSPRUDEBUG_CTR27_RESETVAL                         (0x00000000U)

/* CTR28 */

#define CSL_ICSSPRUDEBUG_CTR28_CT_MASK                          (0xFFFFFFFFU)
#define CSL_ICSSPRUDEBUG_CTR28_CT_SHIFT                         (0U)
#define CSL_ICSSPRUDEBUG_CTR28_CT_RESETVAL                      (0x00000000U)
#define CSL_ICSSPRUDEBUG_CTR28_CT_MAX                           (0xffffffffU)

#define CSL_ICSSPRUDEBUG_CTR28_RESETVAL                         (0x00000000U)

/* CTR29 */

#define CSL_ICSSPRUDEBUG_CTR29_CT_MASK                          (0xFFFFFFFFU)
#define CSL_ICSSPRUDEBUG_CTR29_CT_SHIFT                         (0U)
#define CSL_ICSSPRUDEBUG_CTR29_CT_RESETVAL                      (0x00000000U)
#define CSL_ICSSPRUDEBUG_CTR29_CT_MAX                           (0xffffffffU)

#define CSL_ICSSPRUDEBUG_CTR29_RESETVAL                         (0x00000000U)

/* CTR30 */

#define CSL_ICSSPRUDEBUG_CTR30_CT_MASK                          (0xFFFFFFFFU)
#define CSL_ICSSPRUDEBUG_CTR30_CT_SHIFT                         (0U)
#define CSL_ICSSPRUDEBUG_CTR30_CT_RESETVAL                      (0x00000000U)
#define CSL_ICSSPRUDEBUG_CTR30_CT_MAX                           (0xffffffffU)

#define CSL_ICSSPRUDEBUG_CTR30_RESETVAL                         (0x00000000U)

/* CTR31 */

#define CSL_ICSSPRUDEBUG_CTR31_CT_MASK                          (0xFFFFFFFFU)
#define CSL_ICSSPRUDEBUG_CTR31_CT_SHIFT                         (0U)
#define CSL_ICSSPRUDEBUG_CTR31_CT_RESETVAL                      (0x00000000U)
#define CSL_ICSSPRUDEBUG_CTR31_CT_MAX                           (0xffffffffU)

#define CSL_ICSSPRUDEBUG_CTR31_RESETVAL                         (0x00000000U)

#ifdef __cplusplus
}
#endif
#endif
