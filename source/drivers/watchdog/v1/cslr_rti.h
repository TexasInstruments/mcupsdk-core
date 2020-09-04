/********************************************************************
 * Copyright (C) 2020 Texas Instruments Incorporated.
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
 *  Name        : cslr_rti.h
*/
#ifndef CSLR_RTI_H_
#define CSLR_RTI_H_

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
    volatile uint32_t RTIGCTRL;
    volatile uint32_t RTITBCTRL;
    volatile uint32_t RTICAPCTRL;
    volatile uint32_t RTICOMPCTRL;
    volatile uint32_t RTIFRC0;
    volatile uint32_t RTIUC0;
    volatile uint32_t RTICPUC0;
    volatile uint8_t  Resv_32[4];
    volatile uint32_t RTICAFRC0;
    volatile uint32_t RTICAUC0;
    volatile uint8_t  Resv_48[8];
    volatile uint32_t RTIFRC1;
    volatile uint32_t RTIUC1;
    volatile uint32_t RTICPUC1;
    volatile uint8_t  Resv_64[4];
    volatile uint32_t RTICAFRC1;
    volatile uint32_t RTICAUC1;
    volatile uint8_t  Resv_80[8];
    volatile uint32_t RTICOMP0;
    volatile uint32_t RTIUDCP0;
    volatile uint32_t RTICOMP1;
    volatile uint32_t RTIUDCP1;
    volatile uint32_t RTICOMP2;
    volatile uint32_t RTIUDCP2;
    volatile uint32_t RTICOMP3;
    volatile uint32_t RTIUDCP3;
    volatile uint32_t RTITBLCOMP;
    volatile uint32_t RTITBHCOMP;
    volatile uint8_t  Resv_128[8];
    volatile uint32_t RTISETINT;
    volatile uint32_t RTICLEARINT;
    volatile uint32_t RTIINTFLAG;
    volatile uint8_t  Resv_144[4];
    volatile uint32_t RTIDWDCTRL;
    volatile uint32_t RTIDWDPRLD;
    volatile uint32_t RTIWDSTATUS;
    volatile uint32_t RTIWDKEY;
    volatile uint32_t RTIDWDCNTR;
    volatile uint32_t RTIWWDRXNCTRL;
    volatile uint32_t RTIWWDSIZECTRL;
    volatile uint32_t RTIINTCLRENABLE;
    volatile uint32_t RTICOMP0CLR;
    volatile uint32_t RTICOMP1CLR;
    volatile uint32_t RTICOMP2CLR;
    volatile uint32_t RTICOMP3CLR;
} CSL_rtiRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_RTI_RTIGCTRL                                                   (0x00000000U)
#define CSL_RTI_RTITBCTRL                                                  (0x00000004U)
#define CSL_RTI_RTICAPCTRL                                                 (0x00000008U)
#define CSL_RTI_RTICOMPCTRL                                                (0x0000000CU)
#define CSL_RTI_RTIFRC0                                                    (0x00000010U)
#define CSL_RTI_RTIUC0                                                     (0x00000014U)
#define CSL_RTI_RTICPUC0                                                   (0x00000018U)
#define CSL_RTI_RTICAFRC0                                                  (0x00000020U)
#define CSL_RTI_RTICAUC0                                                   (0x00000024U)
#define CSL_RTI_RTIFRC1                                                    (0x00000030U)
#define CSL_RTI_RTIUC1                                                     (0x00000034U)
#define CSL_RTI_RTICPUC1                                                   (0x00000038U)
#define CSL_RTI_RTICAFRC1                                                  (0x00000040U)
#define CSL_RTI_RTICAUC1                                                   (0x00000044U)
#define CSL_RTI_RTICOMP0                                                   (0x00000050U)
#define CSL_RTI_RTIUDCP0                                                   (0x00000054U)
#define CSL_RTI_RTICOMP1                                                   (0x00000058U)
#define CSL_RTI_RTIUDCP1                                                   (0x0000005CU)
#define CSL_RTI_RTICOMP2                                                   (0x00000060U)
#define CSL_RTI_RTIUDCP2                                                   (0x00000064U)
#define CSL_RTI_RTICOMP3                                                   (0x00000068U)
#define CSL_RTI_RTIUDCP3                                                   (0x0000006CU)
#define CSL_RTI_RTITBLCOMP                                                 (0x00000070U)
#define CSL_RTI_RTITBHCOMP                                                 (0x00000074U)
#define CSL_RTI_RTISETINT                                                  (0x00000080U)
#define CSL_RTI_RTICLEARINT                                                (0x00000084U)
#define CSL_RTI_RTIINTFLAG                                                 (0x00000088U)
#define CSL_RTI_RTIDWDCTRL                                                 (0x00000090U)
#define CSL_RTI_RTIDWDPRLD                                                 (0x00000094U)
#define CSL_RTI_RTIWDSTATUS                                                (0x00000098U)
#define CSL_RTI_RTIWDKEY                                                   (0x0000009CU)
#define CSL_RTI_RTIDWDCNTR                                                 (0x000000A0U)
#define CSL_RTI_RTIWWDRXNCTRL                                              (0x000000A4U)
#define CSL_RTI_RTIWWDSIZECTRL                                             (0x000000A8U)
#define CSL_RTI_RTIINTCLRENABLE                                            (0x000000ACU)
#define CSL_RTI_RTICOMP0CLR                                                (0x000000B0U)
#define CSL_RTI_RTICOMP1CLR                                                (0x000000B4U)
#define CSL_RTI_RTICOMP2CLR                                                (0x000000B8U)
#define CSL_RTI_RTICOMP3CLR                                                (0x000000BCU)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* RTIGCTRL */

#define CSL_RTI_RTIGCTRL_CNT0EN_MASK                                       (0x00000001U)
#define CSL_RTI_RTIGCTRL_CNT0EN_SHIFT                                      (0x00000000U)
#define CSL_RTI_RTIGCTRL_CNT0EN_RESETVAL                                   (0x00000000U)
#define CSL_RTI_RTIGCTRL_CNT0EN_MAX                                        (0x00000001U)

#define CSL_RTI_RTIGCTRL_CNT1EN_MASK                                       (0x00000002U)
#define CSL_RTI_RTIGCTRL_CNT1EN_SHIFT                                      (0x00000001U)
#define CSL_RTI_RTIGCTRL_CNT1EN_RESETVAL                                   (0x00000000U)
#define CSL_RTI_RTIGCTRL_CNT1EN_MAX                                        (0x00000001U)

#define CSL_RTI_RTIGCTRL_RESERVED1_MASK                                    (0x00007FFCU)
#define CSL_RTI_RTIGCTRL_RESERVED1_SHIFT                                   (0x00000002U)
#define CSL_RTI_RTIGCTRL_RESERVED1_RESETVAL                                (0x00000000U)
#define CSL_RTI_RTIGCTRL_RESERVED1_MAX                                     (0x00001FFFU)

#define CSL_RTI_RTIGCTRL_COS_MASK                                          (0x00008000U)
#define CSL_RTI_RTIGCTRL_COS_SHIFT                                         (0x0000000FU)
#define CSL_RTI_RTIGCTRL_COS_RESETVAL                                      (0x00000000U)
#define CSL_RTI_RTIGCTRL_COS_MAX                                           (0x00000001U)
#define CSL_RTI_RTIGCTRL_COS_STOPPED                                       (0x00000000U)
#define CSL_RTI_RTIGCTRL_COS_RUNNING                                       (0x00000001U)

#define CSL_RTI_RTIGCTRL_NTUSEL_MASK                                       (0x000F0000U)
#define CSL_RTI_RTIGCTRL_NTUSEL_SHIFT                                      (0x00000010U)
#define CSL_RTI_RTIGCTRL_NTUSEL_RESETVAL                                   (0x00000000U)
#define CSL_RTI_RTIGCTRL_NTUSEL_MAX                                        (0x0000000FU)

#define CSL_RTI_RTIGCTRL_RESERVED2_MASK                                    (0xFFF00000U)
#define CSL_RTI_RTIGCTRL_RESERVED2_SHIFT                                   (0x00000014U)
#define CSL_RTI_RTIGCTRL_RESERVED2_RESETVAL                                (0x00000000U)
#define CSL_RTI_RTIGCTRL_RESERVED2_MAX                                     (0x00000FFFU)

#define CSL_RTI_RTIGCTRL_RESETVAL                                          (0x00000000U)

/* RTITBCTRL */

#define CSL_RTI_RTITBCTRL_TBEXT_MASK                                       (0x00000001U)
#define CSL_RTI_RTITBCTRL_TBEXT_SHIFT                                      (0x00000000U)
#define CSL_RTI_RTITBCTRL_TBEXT_RESETVAL                                   (0x00000000U)
#define CSL_RTI_RTITBCTRL_TBEXT_MAX                                        (0x00000001U)

#define CSL_RTI_RTITBCTRL_INC_MASK                                         (0x00000002U)
#define CSL_RTI_RTITBCTRL_INC_SHIFT                                        (0x00000001U)
#define CSL_RTI_RTITBCTRL_INC_RESETVAL                                     (0x00000000U)
#define CSL_RTI_RTITBCTRL_INC_MAX                                          (0x00000001U)

#define CSL_RTI_RTITBCTRL_RESERVED3_MASK                                   (0xFFFFFFFCU)
#define CSL_RTI_RTITBCTRL_RESERVED3_SHIFT                                  (0x00000002U)
#define CSL_RTI_RTITBCTRL_RESERVED3_RESETVAL                               (0x00000000U)
#define CSL_RTI_RTITBCTRL_RESERVED3_MAX                                    (0x3FFFFFFFU)

#define CSL_RTI_RTITBCTRL_RESETVAL                                         (0x00000000U)

/* RTICAPCTRL */

#define CSL_RTI_RTICAPCTRL_CAPCNTR0_MASK                                   (0x00000001U)
#define CSL_RTI_RTICAPCTRL_CAPCNTR0_SHIFT                                  (0x00000000U)
#define CSL_RTI_RTICAPCTRL_CAPCNTR0_RESETVAL                               (0x00000000U)
#define CSL_RTI_RTICAPCTRL_CAPCNTR0_MAX                                    (0x00000001U)

#define CSL_RTI_RTICAPCTRL_CAPCNTR1_MASK                                   (0x00000002U)
#define CSL_RTI_RTICAPCTRL_CAPCNTR1_SHIFT                                  (0x00000001U)
#define CSL_RTI_RTICAPCTRL_CAPCNTR1_RESETVAL                               (0x00000000U)
#define CSL_RTI_RTICAPCTRL_CAPCNTR1_MAX                                    (0x00000001U)

#define CSL_RTI_RTICAPCTRL_RESERVED4_MASK                                  (0xFFFFFFFCU)
#define CSL_RTI_RTICAPCTRL_RESERVED4_SHIFT                                 (0x00000002U)
#define CSL_RTI_RTICAPCTRL_RESERVED4_RESETVAL                              (0x00000000U)
#define CSL_RTI_RTICAPCTRL_RESERVED4_MAX                                   (0x3FFFFFFFU)

#define CSL_RTI_RTICAPCTRL_RESETVAL                                        (0x00000000U)

/* RTICOMPCTRL */

#define CSL_RTI_RTICOMPCTRL_COMP0SEL_MASK                                  (0x00000001U)
#define CSL_RTI_RTICOMPCTRL_COMP0SEL_SHIFT                                 (0x00000000U)
#define CSL_RTI_RTICOMPCTRL_COMP0SEL_RESETVAL                              (0x00000000U)
#define CSL_RTI_RTICOMPCTRL_COMP0SEL_MAX                                   (0x00000001U)

#define CSL_RTI_RTICOMPCTRL_RESERVED5_MASK                                 (0x0000000EU)
#define CSL_RTI_RTICOMPCTRL_RESERVED5_SHIFT                                (0x00000001U)
#define CSL_RTI_RTICOMPCTRL_RESERVED5_RESETVAL                             (0x00000000U)
#define CSL_RTI_RTICOMPCTRL_RESERVED5_MAX                                  (0x00000007U)

#define CSL_RTI_RTICOMPCTRL_COMP1SEL_MASK                                  (0x00000010U)
#define CSL_RTI_RTICOMPCTRL_COMP1SEL_SHIFT                                 (0x00000004U)
#define CSL_RTI_RTICOMPCTRL_COMP1SEL_RESETVAL                              (0x00000000U)
#define CSL_RTI_RTICOMPCTRL_COMP1SEL_MAX                                   (0x00000001U)

#define CSL_RTI_RTICOMPCTRL_RESERVED6_MASK                                 (0x000000E0U)
#define CSL_RTI_RTICOMPCTRL_RESERVED6_SHIFT                                (0x00000005U)
#define CSL_RTI_RTICOMPCTRL_RESERVED6_RESETVAL                             (0x00000000U)
#define CSL_RTI_RTICOMPCTRL_RESERVED6_MAX                                  (0x00000007U)

#define CSL_RTI_RTICOMPCTRL_COMP2SEL_MASK                                  (0x00000100U)
#define CSL_RTI_RTICOMPCTRL_COMP2SEL_SHIFT                                 (0x00000008U)
#define CSL_RTI_RTICOMPCTRL_COMP2SEL_RESETVAL                              (0x00000000U)
#define CSL_RTI_RTICOMPCTRL_COMP2SEL_MAX                                   (0x00000001U)

#define CSL_RTI_RTICOMPCTRL_RESERVED7_MASK                                 (0x00000E00U)
#define CSL_RTI_RTICOMPCTRL_RESERVED7_SHIFT                                (0x00000009U)
#define CSL_RTI_RTICOMPCTRL_RESERVED7_RESETVAL                             (0x00000000U)
#define CSL_RTI_RTICOMPCTRL_RESERVED7_MAX                                  (0x00000007U)

#define CSL_RTI_RTICOMPCTRL_COMP3SEL_MASK                                  (0x00001000U)
#define CSL_RTI_RTICOMPCTRL_COMP3SEL_SHIFT                                 (0x0000000CU)
#define CSL_RTI_RTICOMPCTRL_COMP3SEL_RESETVAL                              (0x00000000U)
#define CSL_RTI_RTICOMPCTRL_COMP3SEL_MAX                                   (0x00000001U)

#define CSL_RTI_RTICOMPCTRL_RESERVED8_MASK                                 (0xFFFFE000U)
#define CSL_RTI_RTICOMPCTRL_RESERVED8_SHIFT                                (0x0000000DU)
#define CSL_RTI_RTICOMPCTRL_RESERVED8_RESETVAL                             (0x00000000U)
#define CSL_RTI_RTICOMPCTRL_RESERVED8_MAX                                  (0x0007FFFFU)

#define CSL_RTI_RTICOMPCTRL_RESETVAL                                       (0x00000000U)

/* RTIFRC0 */

#define CSL_RTI_RTIFRC0_FRC0_MASK                                          (0xFFFFFFFFU)
#define CSL_RTI_RTIFRC0_FRC0_SHIFT                                         (0x00000000U)
#define CSL_RTI_RTIFRC0_FRC0_RESETVAL                                      (0x00000000U)
#define CSL_RTI_RTIFRC0_FRC0_MAX                                           (0xFFFFFFFFU)

#define CSL_RTI_RTIFRC0_RESETVAL                                           (0x00000000U)

/* RTIUC0 */

#define CSL_RTI_RTIUC0_UC0_MASK                                            (0xFFFFFFFFU)
#define CSL_RTI_RTIUC0_UC0_SHIFT                                           (0x00000000U)
#define CSL_RTI_RTIUC0_UC0_RESETVAL                                        (0x00000000U)
#define CSL_RTI_RTIUC0_UC0_MAX                                             (0xFFFFFFFFU)

#define CSL_RTI_RTIUC0_RESETVAL                                            (0x00000000U)

/* RTICPUC0 */

#define CSL_RTI_RTICPUC0_CPUC0_MASK                                        (0xFFFFFFFFU)
#define CSL_RTI_RTICPUC0_CPUC0_SHIFT                                       (0x00000000U)
#define CSL_RTI_RTICPUC0_CPUC0_RESETVAL                                    (0x00000000U)
#define CSL_RTI_RTICPUC0_CPUC0_MAX                                         (0xFFFFFFFFU)

#define CSL_RTI_RTICPUC0_RESETVAL                                          (0x00000000U)

/* RTICAFRC0 */

#define CSL_RTI_RTICAFRC0_CAFRC0_MASK                                      (0xFFFFFFFFU)
#define CSL_RTI_RTICAFRC0_CAFRC0_SHIFT                                     (0x00000000U)
#define CSL_RTI_RTICAFRC0_CAFRC0_RESETVAL                                  (0x00000000U)
#define CSL_RTI_RTICAFRC0_CAFRC0_MAX                                       (0xFFFFFFFFU)

#define CSL_RTI_RTICAFRC0_RESETVAL                                         (0x00000000U)

/* RTICAUC0 */

#define CSL_RTI_RTICAUC0_CAUC0_MASK                                        (0xFFFFFFFFU)
#define CSL_RTI_RTICAUC0_CAUC0_SHIFT                                       (0x00000000U)
#define CSL_RTI_RTICAUC0_CAUC0_RESETVAL                                    (0x00000000U)
#define CSL_RTI_RTICAUC0_CAUC0_MAX                                         (0xFFFFFFFFU)

#define CSL_RTI_RTICAUC0_RESETVAL                                          (0x00000000U)

/* RTIFRC1 */

#define CSL_RTI_RTIFRC1_FRC1_MASK                                          (0xFFFFFFFFU)
#define CSL_RTI_RTIFRC1_FRC1_SHIFT                                         (0x00000000U)
#define CSL_RTI_RTIFRC1_FRC1_RESETVAL                                      (0x00000000U)
#define CSL_RTI_RTIFRC1_FRC1_MAX                                           (0xFFFFFFFFU)

#define CSL_RTI_RTIFRC1_RESETVAL                                           (0x00000000U)

/* RTIUC1 */

#define CSL_RTI_RTIUC1_UC1_MASK                                            (0xFFFFFFFFU)
#define CSL_RTI_RTIUC1_UC1_SHIFT                                           (0x00000000U)
#define CSL_RTI_RTIUC1_UC1_RESETVAL                                        (0x00000000U)
#define CSL_RTI_RTIUC1_UC1_MAX                                             (0xFFFFFFFFU)

#define CSL_RTI_RTIUC1_RESETVAL                                            (0x00000000U)

/* RTICPUC1 */

#define CSL_RTI_RTICPUC1_CPUC1_MASK                                        (0xFFFFFFFFU)
#define CSL_RTI_RTICPUC1_CPUC1_SHIFT                                       (0x00000000U)
#define CSL_RTI_RTICPUC1_CPUC1_RESETVAL                                    (0x00000000U)
#define CSL_RTI_RTICPUC1_CPUC1_MAX                                         (0xFFFFFFFFU)

#define CSL_RTI_RTICPUC1_RESETVAL                                          (0x00000000U)

/* RTICAFRC1 */

#define CSL_RTI_RTICAFRC1_CAFRC1_MASK                                      (0xFFFFFFFFU)
#define CSL_RTI_RTICAFRC1_CAFRC1_SHIFT                                     (0x00000000U)
#define CSL_RTI_RTICAFRC1_CAFRC1_RESETVAL                                  (0x00000000U)
#define CSL_RTI_RTICAFRC1_CAFRC1_MAX                                       (0xFFFFFFFFU)

#define CSL_RTI_RTICAFRC1_RESETVAL                                         (0x00000000U)

/* RTICAUC1 */

#define CSL_RTI_RTICAUC1_CAUC1_MASK                                        (0xFFFFFFFFU)
#define CSL_RTI_RTICAUC1_CAUC1_SHIFT                                       (0x00000000U)
#define CSL_RTI_RTICAUC1_CAUC1_RESETVAL                                    (0x00000000U)
#define CSL_RTI_RTICAUC1_CAUC1_MAX                                         (0xFFFFFFFFU)

#define CSL_RTI_RTICAUC1_RESETVAL                                          (0x00000000U)

/* RTICOMP0 */

#define CSL_RTI_RTICOMP0_COMP0_MASK                                        (0xFFFFFFFFU)
#define CSL_RTI_RTICOMP0_COMP0_SHIFT                                       (0x00000000U)
#define CSL_RTI_RTICOMP0_COMP0_RESETVAL                                    (0x00000000U)
#define CSL_RTI_RTICOMP0_COMP0_MAX                                         (0xFFFFFFFFU)

#define CSL_RTI_RTICOMP0_RESETVAL                                          (0x00000000U)

/* RTIUDCP0 */

#define CSL_RTI_RTIUDCP0_UDCP0_MASK                                        (0xFFFFFFFFU)
#define CSL_RTI_RTIUDCP0_UDCP0_SHIFT                                       (0x00000000U)
#define CSL_RTI_RTIUDCP0_UDCP0_RESETVAL                                    (0x00000000U)
#define CSL_RTI_RTIUDCP0_UDCP0_MAX                                         (0xFFFFFFFFU)

#define CSL_RTI_RTIUDCP0_RESETVAL                                          (0x00000000U)

/* RTICOMP1 */

#define CSL_RTI_RTICOMP1_COMP1_MASK                                        (0xFFFFFFFFU)
#define CSL_RTI_RTICOMP1_COMP1_SHIFT                                       (0x00000000U)
#define CSL_RTI_RTICOMP1_COMP1_RESETVAL                                    (0x00000000U)
#define CSL_RTI_RTICOMP1_COMP1_MAX                                         (0xFFFFFFFFU)

#define CSL_RTI_RTICOMP1_RESETVAL                                          (0x00000000U)

/* RTIUDCP1 */

#define CSL_RTI_RTIUDCP1_UDCP1_MASK                                        (0xFFFFFFFFU)
#define CSL_RTI_RTIUDCP1_UDCP1_SHIFT                                       (0x00000000U)
#define CSL_RTI_RTIUDCP1_UDCP1_RESETVAL                                    (0x00000000U)
#define CSL_RTI_RTIUDCP1_UDCP1_MAX                                         (0xFFFFFFFFU)

#define CSL_RTI_RTIUDCP1_RESETVAL                                          (0x00000000U)

/* RTICOMP2 */

#define CSL_RTI_RTICOMP2_COMP2_MASK                                        (0xFFFFFFFFU)
#define CSL_RTI_RTICOMP2_COMP2_SHIFT                                       (0x00000000U)
#define CSL_RTI_RTICOMP2_COMP2_RESETVAL                                    (0x00000000U)
#define CSL_RTI_RTICOMP2_COMP2_MAX                                         (0xFFFFFFFFU)

#define CSL_RTI_RTICOMP2_RESETVAL                                          (0x00000000U)

/* RTIUDCP2 */

#define CSL_RTI_RTIUDCP2_UDCP2_MASK                                        (0xFFFFFFFFU)
#define CSL_RTI_RTIUDCP2_UDCP2_SHIFT                                       (0x00000000U)
#define CSL_RTI_RTIUDCP2_UDCP2_RESETVAL                                    (0x00000000U)
#define CSL_RTI_RTIUDCP2_UDCP2_MAX                                         (0xFFFFFFFFU)

#define CSL_RTI_RTIUDCP2_RESETVAL                                          (0x00000000U)

/* RTICOMP3 */

#define CSL_RTI_RTICOMP3_COMP3_MASK                                        (0xFFFFFFFFU)
#define CSL_RTI_RTICOMP3_COMP3_SHIFT                                       (0x00000000U)
#define CSL_RTI_RTICOMP3_COMP3_RESETVAL                                    (0x00000000U)
#define CSL_RTI_RTICOMP3_COMP3_MAX                                         (0xFFFFFFFFU)

#define CSL_RTI_RTICOMP3_RESETVAL                                          (0x00000000U)

/* RTIUDCP3 */

#define CSL_RTI_RTIUDCP3_UDCP3_MASK                                        (0xFFFFFFFFU)
#define CSL_RTI_RTIUDCP3_UDCP3_SHIFT                                       (0x00000000U)
#define CSL_RTI_RTIUDCP3_UDCP3_RESETVAL                                    (0x00000000U)
#define CSL_RTI_RTIUDCP3_UDCP3_MAX                                         (0xFFFFFFFFU)

#define CSL_RTI_RTIUDCP3_RESETVAL                                          (0x00000000U)

/* RTITBLCOMP */

#define CSL_RTI_RTITBLCOMP_TBLCOMP_MASK                                    (0xFFFFFFFFU)
#define CSL_RTI_RTITBLCOMP_TBLCOMP_SHIFT                                   (0x00000000U)
#define CSL_RTI_RTITBLCOMP_TBLCOMP_RESETVAL                                (0x00000000U)
#define CSL_RTI_RTITBLCOMP_TBLCOMP_MAX                                     (0xFFFFFFFFU)

#define CSL_RTI_RTITBLCOMP_RESETVAL                                        (0x00000000U)

/* RTITBHCOMP */

#define CSL_RTI_RTITBHCOMP_TBHCOMP_MASK                                    (0xFFFFFFFFU)
#define CSL_RTI_RTITBHCOMP_TBHCOMP_SHIFT                                   (0x00000000U)
#define CSL_RTI_RTITBHCOMP_TBHCOMP_RESETVAL                                (0x00000000U)
#define CSL_RTI_RTITBHCOMP_TBHCOMP_MAX                                     (0xFFFFFFFFU)

#define CSL_RTI_RTITBHCOMP_RESETVAL                                        (0x00000000U)

/* RTISETINT */

#define CSL_RTI_RTISETINT_SETINT0_MASK                                     (0x00000001U)
#define CSL_RTI_RTISETINT_SETINT0_SHIFT                                    (0x00000000U)
#define CSL_RTI_RTISETINT_SETINT0_RESETVAL                                 (0x00000000U)
#define CSL_RTI_RTISETINT_SETINT0_MAX                                      (0x00000001U)

#define CSL_RTI_RTISETINT_SETINT1_MASK                                     (0x00000002U)
#define CSL_RTI_RTISETINT_SETINT1_SHIFT                                    (0x00000001U)
#define CSL_RTI_RTISETINT_SETINT1_RESETVAL                                 (0x00000000U)
#define CSL_RTI_RTISETINT_SETINT1_MAX                                      (0x00000001U)

#define CSL_RTI_RTISETINT_SETINT2_MASK                                     (0x00000004U)
#define CSL_RTI_RTISETINT_SETINT2_SHIFT                                    (0x00000002U)
#define CSL_RTI_RTISETINT_SETINT2_RESETVAL                                 (0x00000000U)
#define CSL_RTI_RTISETINT_SETINT2_MAX                                      (0x00000001U)

#define CSL_RTI_RTISETINT_SETINT3_MASK                                     (0x00000008U)
#define CSL_RTI_RTISETINT_SETINT3_SHIFT                                    (0x00000003U)
#define CSL_RTI_RTISETINT_SETINT3_RESETVAL                                 (0x00000000U)
#define CSL_RTI_RTISETINT_SETINT3_MAX                                      (0x00000001U)

#define CSL_RTI_RTISETINT_RESERVED9_MASK                                   (0x000000F0U)
#define CSL_RTI_RTISETINT_RESERVED9_SHIFT                                  (0x00000004U)
#define CSL_RTI_RTISETINT_RESERVED9_RESETVAL                               (0x00000000U)
#define CSL_RTI_RTISETINT_RESERVED9_MAX                                    (0x0000000FU)

#define CSL_RTI_RTISETINT_SETDMA0_MASK                                     (0x00000100U)
#define CSL_RTI_RTISETINT_SETDMA0_SHIFT                                    (0x00000008U)
#define CSL_RTI_RTISETINT_SETDMA0_RESETVAL                                 (0x00000000U)
#define CSL_RTI_RTISETINT_SETDMA0_MAX                                      (0x00000001U)

#define CSL_RTI_RTISETINT_SETDMA1_MASK                                     (0x00000200U)
#define CSL_RTI_RTISETINT_SETDMA1_SHIFT                                    (0x00000009U)
#define CSL_RTI_RTISETINT_SETDMA1_RESETVAL                                 (0x00000000U)
#define CSL_RTI_RTISETINT_SETDMA1_MAX                                      (0x00000001U)

#define CSL_RTI_RTISETINT_SETDMA2_MASK                                     (0x00000400U)
#define CSL_RTI_RTISETINT_SETDMA2_SHIFT                                    (0x0000000AU)
#define CSL_RTI_RTISETINT_SETDMA2_RESETVAL                                 (0x00000000U)
#define CSL_RTI_RTISETINT_SETDMA2_MAX                                      (0x00000001U)

#define CSL_RTI_RTISETINT_SETDMA3_MASK                                     (0x00000800U)
#define CSL_RTI_RTISETINT_SETDMA3_SHIFT                                    (0x0000000BU)
#define CSL_RTI_RTISETINT_SETDMA3_RESETVAL                                 (0x00000000U)
#define CSL_RTI_RTISETINT_SETDMA3_MAX                                      (0x00000001U)

#define CSL_RTI_RTISETINT_RESERVED10_MASK                                  (0x0000F000U)
#define CSL_RTI_RTISETINT_RESERVED10_SHIFT                                 (0x0000000CU)
#define CSL_RTI_RTISETINT_RESERVED10_RESETVAL                              (0x00000000U)
#define CSL_RTI_RTISETINT_RESERVED10_MAX                                   (0x0000000FU)

#define CSL_RTI_RTISETINT_SETTBINT_MASK                                    (0x00010000U)
#define CSL_RTI_RTISETINT_SETTBINT_SHIFT                                   (0x00000010U)
#define CSL_RTI_RTISETINT_SETTBINT_RESETVAL                                (0x00000000U)
#define CSL_RTI_RTISETINT_SETTBINT_MAX                                     (0x00000001U)

#define CSL_RTI_RTISETINT_SETOVL0INT_MASK                                  (0x00020000U)
#define CSL_RTI_RTISETINT_SETOVL0INT_SHIFT                                 (0x00000011U)
#define CSL_RTI_RTISETINT_SETOVL0INT_RESETVAL                              (0x00000000U)
#define CSL_RTI_RTISETINT_SETOVL0INT_MAX                                   (0x00000001U)

#define CSL_RTI_RTISETINT_SETOVL1INT_MASK                                  (0x00040000U)
#define CSL_RTI_RTISETINT_SETOVL1INT_SHIFT                                 (0x00000012U)
#define CSL_RTI_RTISETINT_SETOVL1INT_RESETVAL                              (0x00000000U)
#define CSL_RTI_RTISETINT_SETOVL1INT_MAX                                   (0x00000001U)

#define CSL_RTI_RTISETINT_RESERVED11_MASK                                  (0xFFF80000U)
#define CSL_RTI_RTISETINT_RESERVED11_SHIFT                                 (0x00000013U)
#define CSL_RTI_RTISETINT_RESERVED11_RESETVAL                              (0x00000000U)
#define CSL_RTI_RTISETINT_RESERVED11_MAX                                   (0x00001FFFU)

#define CSL_RTI_RTISETINT_RESETVAL                                         (0x00000000U)

/* RTICLEARINT */

#define CSL_RTI_RTICLEARINT_CLEARINT0_MASK                                 (0x00000001U)
#define CSL_RTI_RTICLEARINT_CLEARINT0_SHIFT                                (0x00000000U)
#define CSL_RTI_RTICLEARINT_CLEARINT0_RESETVAL                             (0x00000000U)
#define CSL_RTI_RTICLEARINT_CLEARINT0_MAX                                  (0x00000001U)

#define CSL_RTI_RTICLEARINT_CLEARINT1_MASK                                 (0x00000002U)
#define CSL_RTI_RTICLEARINT_CLEARINT1_SHIFT                                (0x00000001U)
#define CSL_RTI_RTICLEARINT_CLEARINT1_RESETVAL                             (0x00000000U)
#define CSL_RTI_RTICLEARINT_CLEARINT1_MAX                                  (0x00000001U)

#define CSL_RTI_RTICLEARINT_CLEARINT2_MASK                                 (0x00000004U)
#define CSL_RTI_RTICLEARINT_CLEARINT2_SHIFT                                (0x00000002U)
#define CSL_RTI_RTICLEARINT_CLEARINT2_RESETVAL                             (0x00000000U)
#define CSL_RTI_RTICLEARINT_CLEARINT2_MAX                                  (0x00000001U)

#define CSL_RTI_RTICLEARINT_CLEARINT3_MASK                                 (0x00000008U)
#define CSL_RTI_RTICLEARINT_CLEARINT3_SHIFT                                (0x00000003U)
#define CSL_RTI_RTICLEARINT_CLEARINT3_RESETVAL                             (0x00000000U)
#define CSL_RTI_RTICLEARINT_CLEARINT3_MAX                                  (0x00000001U)

#define CSL_RTI_RTICLEARINT_RESERVED12_MASK                                (0x000000F0U)
#define CSL_RTI_RTICLEARINT_RESERVED12_SHIFT                               (0x00000004U)
#define CSL_RTI_RTICLEARINT_RESERVED12_RESETVAL                            (0x00000000U)
#define CSL_RTI_RTICLEARINT_RESERVED12_MAX                                 (0x0000000FU)

#define CSL_RTI_RTICLEARINT_CLEARDMA0_MASK                                 (0x00000100U)
#define CSL_RTI_RTICLEARINT_CLEARDMA0_SHIFT                                (0x00000008U)
#define CSL_RTI_RTICLEARINT_CLEARDMA0_RESETVAL                             (0x00000000U)
#define CSL_RTI_RTICLEARINT_CLEARDMA0_MAX                                  (0x00000001U)

#define CSL_RTI_RTICLEARINT_CLEARDMA1_MASK                                 (0x00000200U)
#define CSL_RTI_RTICLEARINT_CLEARDMA1_SHIFT                                (0x00000009U)
#define CSL_RTI_RTICLEARINT_CLEARDMA1_RESETVAL                             (0x00000000U)
#define CSL_RTI_RTICLEARINT_CLEARDMA1_MAX                                  (0x00000001U)

#define CSL_RTI_RTICLEARINT_CLEARDMA2_MASK                                 (0x00000400U)
#define CSL_RTI_RTICLEARINT_CLEARDMA2_SHIFT                                (0x0000000AU)
#define CSL_RTI_RTICLEARINT_CLEARDMA2_RESETVAL                             (0x00000000U)
#define CSL_RTI_RTICLEARINT_CLEARDMA2_MAX                                  (0x00000001U)

#define CSL_RTI_RTICLEARINT_CLEARDMA3_MASK                                 (0x00000800U)
#define CSL_RTI_RTICLEARINT_CLEARDMA3_SHIFT                                (0x0000000BU)
#define CSL_RTI_RTICLEARINT_CLEARDMA3_RESETVAL                             (0x00000000U)
#define CSL_RTI_RTICLEARINT_CLEARDMA3_MAX                                  (0x00000001U)

#define CSL_RTI_RTICLEARINT_RESERVED13_MASK                                (0x0000F000U)
#define CSL_RTI_RTICLEARINT_RESERVED13_SHIFT                               (0x0000000CU)
#define CSL_RTI_RTICLEARINT_RESERVED13_RESETVAL                            (0x00000000U)
#define CSL_RTI_RTICLEARINT_RESERVED13_MAX                                 (0x0000000FU)

#define CSL_RTI_RTICLEARINT_CLEARTBINT_MASK                                (0x00010000U)
#define CSL_RTI_RTICLEARINT_CLEARTBINT_SHIFT                               (0x00000010U)
#define CSL_RTI_RTICLEARINT_CLEARTBINT_RESETVAL                            (0x00000000U)
#define CSL_RTI_RTICLEARINT_CLEARTBINT_MAX                                 (0x00000001U)

#define CSL_RTI_RTICLEARINT_CLEAROVL0INT_MASK                              (0x00020000U)
#define CSL_RTI_RTICLEARINT_CLEAROVL0INT_SHIFT                             (0x00000011U)
#define CSL_RTI_RTICLEARINT_CLEAROVL0INT_RESETVAL                          (0x00000000U)
#define CSL_RTI_RTICLEARINT_CLEAROVL0INT_MAX                               (0x00000001U)

#define CSL_RTI_RTICLEARINT_CLEAROVL1INT_MASK                              (0x00040000U)
#define CSL_RTI_RTICLEARINT_CLEAROVL1INT_SHIFT                             (0x00000012U)
#define CSL_RTI_RTICLEARINT_CLEAROVL1INT_RESETVAL                          (0x00000000U)
#define CSL_RTI_RTICLEARINT_CLEAROVL1INT_MAX                               (0x00000001U)

#define CSL_RTI_RTICLEARINT_RESERVED14_MASK                                (0xFFF80000U)
#define CSL_RTI_RTICLEARINT_RESERVED14_SHIFT                               (0x00000013U)
#define CSL_RTI_RTICLEARINT_RESERVED14_RESETVAL                            (0x00000000U)
#define CSL_RTI_RTICLEARINT_RESERVED14_MAX                                 (0x00001FFFU)

#define CSL_RTI_RTICLEARINT_RESETVAL                                       (0x00000000U)

/* RTIINTFLAG */

#define CSL_RTI_RTIINTFLAG_INT0_MASK                                       (0x00000001U)
#define CSL_RTI_RTIINTFLAG_INT0_SHIFT                                      (0x00000000U)
#define CSL_RTI_RTIINTFLAG_INT0_RESETVAL                                   (0x00000000U)
#define CSL_RTI_RTIINTFLAG_INT0_MAX                                        (0x00000001U)

#define CSL_RTI_RTIINTFLAG_INT1_MASK                                       (0x00000002U)
#define CSL_RTI_RTIINTFLAG_INT1_SHIFT                                      (0x00000001U)
#define CSL_RTI_RTIINTFLAG_INT1_RESETVAL                                   (0x00000000U)
#define CSL_RTI_RTIINTFLAG_INT1_MAX                                        (0x00000001U)

#define CSL_RTI_RTIINTFLAG_INT2_MASK                                       (0x00000004U)
#define CSL_RTI_RTIINTFLAG_INT2_SHIFT                                      (0x00000002U)
#define CSL_RTI_RTIINTFLAG_INT2_RESETVAL                                   (0x00000000U)
#define CSL_RTI_RTIINTFLAG_INT2_MAX                                        (0x00000001U)

#define CSL_RTI_RTIINTFLAG_INT3_MASK                                       (0x00000008U)
#define CSL_RTI_RTIINTFLAG_INT3_SHIFT                                      (0x00000003U)
#define CSL_RTI_RTIINTFLAG_INT3_RESETVAL                                   (0x00000000U)
#define CSL_RTI_RTIINTFLAG_INT3_MAX                                        (0x00000001U)

#define CSL_RTI_RTIINTFLAG_RESERVED15_MASK                                 (0x0000FFF0U)
#define CSL_RTI_RTIINTFLAG_RESERVED15_SHIFT                                (0x00000004U)
#define CSL_RTI_RTIINTFLAG_RESERVED15_RESETVAL                             (0x00000000U)
#define CSL_RTI_RTIINTFLAG_RESERVED15_MAX                                  (0x00000FFFU)

#define CSL_RTI_RTIINTFLAG_TBINT_MASK                                      (0x00010000U)
#define CSL_RTI_RTIINTFLAG_TBINT_SHIFT                                     (0x00000010U)
#define CSL_RTI_RTIINTFLAG_TBINT_RESETVAL                                  (0x00000000U)
#define CSL_RTI_RTIINTFLAG_TBINT_MAX                                       (0x00000001U)

#define CSL_RTI_RTIINTFLAG_OVL0INT_MASK                                    (0x00020000U)
#define CSL_RTI_RTIINTFLAG_OVL0INT_SHIFT                                   (0x00000011U)
#define CSL_RTI_RTIINTFLAG_OVL0INT_RESETVAL                                (0x00000000U)
#define CSL_RTI_RTIINTFLAG_OVL0INT_MAX                                     (0x00000001U)

#define CSL_RTI_RTIINTFLAG_OVL1INT_MASK                                    (0x00040000U)
#define CSL_RTI_RTIINTFLAG_OVL1INT_SHIFT                                   (0x00000012U)
#define CSL_RTI_RTIINTFLAG_OVL1INT_RESETVAL                                (0x00000000U)
#define CSL_RTI_RTIINTFLAG_OVL1INT_MAX                                     (0x00000001U)

#define CSL_RTI_RTIINTFLAG_RESERVED16_MASK                                 (0xFFF80000U)
#define CSL_RTI_RTIINTFLAG_RESERVED16_SHIFT                                (0x00000013U)
#define CSL_RTI_RTIINTFLAG_RESERVED16_RESETVAL                             (0x00000000U)
#define CSL_RTI_RTIINTFLAG_RESERVED16_MAX                                  (0x00001FFFU)

#define CSL_RTI_RTIINTFLAG_RESETVAL                                        (0x00000000U)

/* RTIDWDCTRL */
#define CSL_RTI_RTIDWDCTRL_DWDCTRL_15_0_SHIFT                              (0x00000000U)
#define CSL_RTI_RTIDWDCTRL_DWDCTRL_15_0_MASK                               (0x0000FFFFU)
#define CSL_RTI_RTIDWDCTRL_DWDCTRL_31_16_SHIFT                             (0x00000010U)
#define CSL_RTI_RTIDWDCTRL_DWDCTRL_31_16_MASK                              (0xFFFF0000U)
#define CSL_RTI_RTIDWDCTRL_DWDCTRL_DISABLE                                 (0x5312ACEDU)
#define CSL_RTI_RTIDWDCTRL_DWDCTRL_ENABLE                                  (0xA98559DAU)

#define CSL_RTI_RTIDWDCTRL_DWDCTRL_MASK                                    (0xFFFFFFFFU)
#define CSL_RTI_RTIDWDCTRL_DWDCTRL_SHIFT                                   (0x00000000U)
#define CSL_RTI_RTIDWDCTRL_DWDCTRL_RESETVAL                                (0x00000000U)
#define CSL_RTI_RTIDWDCTRL_DWDCTRL_MAX                                     (0xFFFFFFFFU)

#define CSL_RTI_RTIDWDCTRL_RESETVAL                                        (0x00000000U)

/* RTIDWDPRLD */

#define CSL_RTI_RTIDWDPRLD_DWDPRLD_MASK                                    (0x00000FFFU)
#define CSL_RTI_RTIDWDPRLD_DWDPRLD_SHIFT                                   (0x00000000U)
#define CSL_RTI_RTIDWDPRLD_DWDPRLD_RESETVAL                                (0x00000000U)
#define CSL_RTI_RTIDWDPRLD_DWDPRLD_MAX                                     (0x00000FFFU)

#define CSL_RTI_RTIDWDPRLD_RESERVED17_MASK                                 (0xFFFFF000U)
#define CSL_RTI_RTIDWDPRLD_RESERVED17_SHIFT                                (0x0000000CU)
#define CSL_RTI_RTIDWDPRLD_RESERVED17_RESETVAL                             (0x00000000U)
#define CSL_RTI_RTIDWDPRLD_RESERVED17_MAX                                  (0x000FFFFFU)

#define CSL_RTI_RTIDWDPRLD_RESETVAL                                        (0x00000000U)

/* RTIWDSTATUS */

#define CSL_RTI_RTIWDSTATUS_AWDST_MASK                                     (0x00000001U)
#define CSL_RTI_RTIWDSTATUS_AWDST_SHIFT                                    (0x00000000U)
#define CSL_RTI_RTIWDSTATUS_AWDST_RESETVAL                                 (0x00000000U)
#define CSL_RTI_RTIWDSTATUS_AWDST_MAX                                      (0x00000001U)

#define CSL_RTI_RTIWDSTATUS_DWDST_MASK                                     (0x00000002U)
#define CSL_RTI_RTIWDSTATUS_DWDST_SHIFT                                    (0x00000001U)
#define CSL_RTI_RTIWDSTATUS_DWDST_RESETVAL                                 (0x00000000U)
#define CSL_RTI_RTIWDSTATUS_DWDST_MAX                                      (0x00000001U)

#define CSL_RTI_RTIWDSTATUS_KEYST_MASK                                     (0x00000004U)
#define CSL_RTI_RTIWDSTATUS_KEYST_SHIFT                                    (0x00000002U)
#define CSL_RTI_RTIWDSTATUS_KEYST_RESETVAL                                 (0x00000000U)
#define CSL_RTI_RTIWDSTATUS_KEYST_MAX                                      (0x00000001U)
#define CSL_RTI_RTIWDSTATUS_KEYST_CORRECT_KEY                              (0x00000000U)
#define CSL_RTI_RTIWDSTATUS_KEYST_INCORRECT_KEY                            (0x00000001U)

#define CSL_RTI_RTIWDSTATUS_STARTTIMEVIOL_MASK                             (0x00000008U)
#define CSL_RTI_RTIWDSTATUS_STARTTIMEVIOL_SHIFT                            (0x00000003U)
#define CSL_RTI_RTIWDSTATUS_STARTTIMEVIOL_RESETVAL                         (0x00000000U)
#define CSL_RTI_RTIWDSTATUS_STARTTIMEVIOL_MAX                              (0x00000001U)

#define CSL_RTI_RTIWDSTATUS_ENDTIMEVIOL_MASK                               (0x00000010U)
#define CSL_RTI_RTIWDSTATUS_ENDTIMEVIOL_SHIFT                              (0x00000004U)
#define CSL_RTI_RTIWDSTATUS_ENDTIMEVIOL_RESETVAL                           (0x00000000U)
#define CSL_RTI_RTIWDSTATUS_ENDTIMEVIOL_MAX                                (0x00000001U)

#define CSL_RTI_RTIWDSTATUS_DWWD_ST_MASK                                   (0x00000020U)
#define CSL_RTI_RTIWDSTATUS_DWWD_ST_SHIFT                                  (0x00000005U)
#define CSL_RTI_RTIWDSTATUS_DWWD_ST_RESETVAL                               (0x00000000U)
#define CSL_RTI_RTIWDSTATUS_DWWD_ST_MAX                                    (0x00000001U)

#define CSL_RTI_RTIWDSTATUS_RESERVED18_MASK                                (0xFFFFFFC0U)
#define CSL_RTI_RTIWDSTATUS_RESERVED18_SHIFT                               (0x00000006U)
#define CSL_RTI_RTIWDSTATUS_RESERVED18_RESETVAL                            (0x00000000U)
#define CSL_RTI_RTIWDSTATUS_RESERVED18_MAX                                 (0x03FFFFFFU)

#define CSL_RTI_RTIWDSTATUS_RESETVAL                                       (0x00000000U)

/* RTIWDKEY */

#define CSL_RTI_RTIWDKEY_WDKEY_MASK                                        (0x0000FFFFU)
#define CSL_RTI_RTIWDKEY_WDKEY_SHIFT                                       (0x00000000U)
#define CSL_RTI_RTIWDKEY_WDKEY_RESETVAL                                    (0x00000000U)
#define CSL_RTI_RTIWDKEY_WDKEY_MAX                                         (0x0000FFFFU)
#define CSL_RTI_RTIWDKEY_WDKEY_FIRST_WRITE                                 (0x0000E51AU)
#define CSL_RTI_RTIWDKEY_WDKEY_SECOND_WRITE                                (0x0000A35CU)

#define CSL_RTI_RTIWDKEY_RESERVED19_MASK                                   (0xFFFF0000U)
#define CSL_RTI_RTIWDKEY_RESERVED19_SHIFT                                  (0x00000010U)
#define CSL_RTI_RTIWDKEY_RESERVED19_RESETVAL                               (0x00000000U)
#define CSL_RTI_RTIWDKEY_RESERVED19_MAX                                    (0x0000FFFFU)

#define CSL_RTI_RTIWDKEY_RESETVAL                                          (0x00000000U)

/* RTIDWDCNTR */

#define CSL_RTI_RTIDWDCNTR_DWDCNTR_MASK                                    (0x01FFFFFFU)
#define CSL_RTI_RTIDWDCNTR_DWDCNTR_SHIFT                                   (0x00000000U)
#define CSL_RTI_RTIDWDCNTR_DWDCNTR_RESETVAL                                (0x00000000U)
#define CSL_RTI_RTIDWDCNTR_DWDCNTR_MAX                                     (0x01FFFFFFU)

#define CSL_RTI_RTIDWDCNTR_RESERVED20_MASK                                 (0xFE000000U)
#define CSL_RTI_RTIDWDCNTR_RESERVED20_SHIFT                                (0x00000019U)
#define CSL_RTI_RTIDWDCNTR_RESERVED20_RESETVAL                             (0x00000000U)
#define CSL_RTI_RTIDWDCNTR_RESERVED20_MAX                                  (0x0000007FU)

#define CSL_RTI_RTIDWDCNTR_RESETVAL                                        (0x00000000U)

/* RTIWWDRXNCTRL */

#define CSL_RTI_RTIWWDRXNCTRL_WWDRXN_MASK                                  (0x0000000FU)
#define CSL_RTI_RTIWWDRXNCTRL_WWDRXN_SHIFT                                 (0x00000000U)
#define CSL_RTI_RTIWWDRXNCTRL_WWDRXN_RESETVAL                              (0x00000005U)
#define CSL_RTI_RTIWWDRXNCTRL_WWDRXN_INTERRUPT                             (0x0000000AU)
#define CSL_RTI_RTIWWDRXNCTRL_WWDRXN_MAX                                   (0x0000000FU)

#define CSL_RTI_RTIWWDRXNCTRL_RESERVED21_MASK                              (0xFFFFFFF0U)
#define CSL_RTI_RTIWWDRXNCTRL_RESERVED21_SHIFT                             (0x00000004U)
#define CSL_RTI_RTIWWDRXNCTRL_RESERVED21_RESETVAL                          (0x00000000U)
#define CSL_RTI_RTIWWDRXNCTRL_RESERVED21_MAX                               (0x0FFFFFFFU)

#define CSL_RTI_RTIWWDRXNCTRL_RESETVAL                                     (0x00000000U)

/* RTIWWDSIZECTRL */

#define CSL_RTI_RTIWWDSIZECTRL_WWDSIZE_MASK                                (0xFFFFFFFFU)
#define CSL_RTI_RTIWWDSIZECTRL_WWDSIZE_SHIFT                               (0x00000000U)
#define CSL_RTI_RTIWWDSIZECTRL_WWDSIZE_RESETVAL                            (0x00000000U)
#define CSL_RTI_RTIWWDSIZECTRL_WWDSIZE_MAX                                 (0xFFFFFFFFU)
#define CSL_RTI_RTIWWDSIZECTRL_WWDSIZE_100_PERCENT                         (0x00000005U)
#define CSL_RTI_RTIWWDSIZECTRL_WWDSIZE_100_PERCENT_SHIFT                   (0x00000000U)
#define CSL_RTI_RTIWWDSIZECTRL_WWDSIZE_50_PERCENT                          (0x00000050U)
#define CSL_RTI_RTIWWDSIZECTRL_WWDSIZE_50_PERCENT_SHIFT                    (0x00000001U)
#define CSL_RTI_RTIWWDSIZECTRL_WWDSIZE_25_PERCENT                          (0x00000500U)
#define CSL_RTI_RTIWWDSIZECTRL_WWDSIZE_25_PERCENT_SHIFT                    (0x00000002U)
#define CSL_RTI_RTIWWDSIZECTRL_WWDSIZE_12_5_PERCENT                        (0x00005000U)
#define CSL_RTI_RTIWWDSIZECTRL_WWDSIZE_12_5_PERCENT_SHIFT                  (0x00000003U)
#define CSL_RTI_RTIWWDSIZECTRL_WWDSIZE_6_25_PERCENT                        (0x00050000U)
#define CSL_RTI_RTIWWDSIZECTRL_WWDSIZE_6_25_PERCENT_SHIFT                  (0x00000004U)
#define CSL_RTI_RTIWWDSIZECTRL_WWDSIZE_3_125_PERCENT                       (0x00500000U)
#define CSL_RTI_RTIWWDSIZECTRL_WWDSIZE_3_125_PERCENT_SHIFT                 (0x00000005U)

#define CSL_RTI_RTIWWDSIZECTRL_RESETVAL                                    (0x00000000U)

/* RTIINTCLRENABLE */

#define CSL_RTI_RTIINTCLRENABLE_INTCLRENABLE0_MASK                         (0x0000000FU)
#define CSL_RTI_RTIINTCLRENABLE_INTCLRENABLE0_SHIFT                        (0x00000000U)
#define CSL_RTI_RTIINTCLRENABLE_INTCLRENABLE0_RESETVAL                     (0x00000000U)
#define CSL_RTI_RTIINTCLRENABLE_INTCLRENABLE0_MAX                          (0x0000000FU)

#define CSL_RTI_RTIINTCLRENABLE_RESERVED22_MASK                            (0x000000F0U)
#define CSL_RTI_RTIINTCLRENABLE_RESERVED22_SHIFT                           (0x00000004U)
#define CSL_RTI_RTIINTCLRENABLE_RESERVED22_RESETVAL                        (0x00000000U)
#define CSL_RTI_RTIINTCLRENABLE_RESERVED22_MAX                             (0x0000000FU)

#define CSL_RTI_RTIINTCLRENABLE_INTCLRENABLE1_MASK                         (0x00000F00U)
#define CSL_RTI_RTIINTCLRENABLE_INTCLRENABLE1_SHIFT                        (0x00000008U)
#define CSL_RTI_RTIINTCLRENABLE_INTCLRENABLE1_RESETVAL                     (0x00000000U)
#define CSL_RTI_RTIINTCLRENABLE_INTCLRENABLE1_MAX                          (0x0000000FU)

#define CSL_RTI_RTIINTCLRENABLE_RESERVED23_MASK                            (0x0000F000U)
#define CSL_RTI_RTIINTCLRENABLE_RESERVED23_SHIFT                           (0x0000000CU)
#define CSL_RTI_RTIINTCLRENABLE_RESERVED23_RESETVAL                        (0x00000000U)
#define CSL_RTI_RTIINTCLRENABLE_RESERVED23_MAX                             (0x0000000FU)

#define CSL_RTI_RTIINTCLRENABLE_INTCLRENABLE2_MASK                         (0x000F0000U)
#define CSL_RTI_RTIINTCLRENABLE_INTCLRENABLE2_SHIFT                        (0x00000010U)
#define CSL_RTI_RTIINTCLRENABLE_INTCLRENABLE2_RESETVAL                     (0x00000000U)
#define CSL_RTI_RTIINTCLRENABLE_INTCLRENABLE2_MAX                          (0x0000000FU)

#define CSL_RTI_RTIINTCLRENABLE_RESERVED24_MASK                            (0x00F00000U)
#define CSL_RTI_RTIINTCLRENABLE_RESERVED24_SHIFT                           (0x00000014U)
#define CSL_RTI_RTIINTCLRENABLE_RESERVED24_RESETVAL                        (0x00000000U)
#define CSL_RTI_RTIINTCLRENABLE_RESERVED24_MAX                             (0x0000000FU)

#define CSL_RTI_RTIINTCLRENABLE_INTCLRENABLE3_MASK                         (0x0F000000U)
#define CSL_RTI_RTIINTCLRENABLE_INTCLRENABLE3_SHIFT                        (0x00000018U)
#define CSL_RTI_RTIINTCLRENABLE_INTCLRENABLE3_RESETVAL                     (0x00000000U)
#define CSL_RTI_RTIINTCLRENABLE_INTCLRENABLE3_MAX                          (0x0000000FU)

#define CSL_RTI_RTIINTCLRENABLE_RESERVED25_MASK                            (0xF0000000U)
#define CSL_RTI_RTIINTCLRENABLE_RESERVED25_SHIFT                           (0x0000001CU)
#define CSL_RTI_RTIINTCLRENABLE_RESERVED25_RESETVAL                        (0x00000000U)
#define CSL_RTI_RTIINTCLRENABLE_RESERVED25_MAX                             (0x0000000FU)

#define CSL_RTI_RTIINTCLRENABLE_RESETVAL                                   (0x00000000U)

/* RTICOMP0CLR */

#define CSL_RTI_RTICOMP0CLR_COMP0CLR_MASK                                  (0xFFFFFFFFU)
#define CSL_RTI_RTICOMP0CLR_COMP0CLR_SHIFT                                 (0x00000000U)
#define CSL_RTI_RTICOMP0CLR_COMP0CLR_RESETVAL                              (0x00000000U)
#define CSL_RTI_RTICOMP0CLR_COMP0CLR_MAX                                   (0xFFFFFFFFU)

#define CSL_RTI_RTICOMP0CLR_RESETVAL                                       (0x00000000U)

/* RTICOMP1CLR */

#define CSL_RTI_RTICOMP1CLR_COMP1CLR_MASK                                  (0xFFFFFFFFU)
#define CSL_RTI_RTICOMP1CLR_COMP1CLR_SHIFT                                 (0x00000000U)
#define CSL_RTI_RTICOMP1CLR_COMP1CLR_RESETVAL                              (0x00000000U)
#define CSL_RTI_RTICOMP1CLR_COMP1CLR_MAX                                   (0xFFFFFFFFU)

#define CSL_RTI_RTICOMP1CLR_RESETVAL                                       (0x00000000U)

/* RTICOMP2CLR */

#define CSL_RTI_RTICOMP2CLR_COMP2CLR_MASK                                  (0xFFFFFFFFU)
#define CSL_RTI_RTICOMP2CLR_COMP2CLR_SHIFT                                 (0x00000000U)
#define CSL_RTI_RTICOMP2CLR_COMP2CLR_RESETVAL                              (0x00000000U)
#define CSL_RTI_RTICOMP2CLR_COMP2CLR_MAX                                   (0xFFFFFFFFU)

#define CSL_RTI_RTICOMP2CLR_RESETVAL                                       (0x00000000U)

/* RTICOMP3CLR */

#define CSL_RTI_RTICOMP3CLR_COMP3CLR_MASK                                  (0xFFFFFFFFU)
#define CSL_RTI_RTICOMP3CLR_COMP3CLR_SHIFT                                 (0x00000000U)
#define CSL_RTI_RTICOMP3CLR_COMP3CLR_RESETVAL                              (0x00000000U)
#define CSL_RTI_RTICOMP3CLR_COMP3CLR_MAX                                   (0xFFFFFFFFU)

#define CSL_RTI_RTICOMP3CLR_RESETVAL                                       (0x00000000U)

#ifdef __cplusplus
}
#endif
#endif
