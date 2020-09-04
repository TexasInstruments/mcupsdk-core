/*
 *  Copyright (C) 2021 Texas Instruments Incorporated.
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

#ifndef CSLR_GPIO_H_
#define CSLR_GPIO_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/**************************************************************************
* Register Overlay Structure for BANK_REGISTERS
**************************************************************************/
typedef struct {
    volatile uint32_t DIR;
    volatile uint32_t OUT_DATA;
    volatile uint32_t SET_DATA;
    volatile uint32_t CLR_DATA;
    volatile uint32_t IN_DATA;
    volatile uint32_t SET_RIS_TRIG;
    volatile uint32_t CLR_RIS_TRIG;
    volatile uint32_t SET_FAL_TRIG;
    volatile uint32_t CLR_FAL_TRIG;
    volatile uint32_t INTSTAT;
} CSL_GpioBank_registersRegs;


/**************************************************************************
* Register Overlay Structure
**************************************************************************/
typedef struct {
    volatile uint32_t PID;
    volatile uint32_t PCR;
    volatile uint32_t BINTEN;
    volatile uint8_t  RSVD0[4];
    CSL_GpioBank_registersRegs  BANK_REGISTERS[9];
} CSL_GpioRegs;




/**************************************************************************
* Register Macros
**************************************************************************/
#define CSL_GPIO_DIR(n)                                         (0x10U + ((n) * 0x28U))
#define CSL_GPIO_OUT_DATA(n)                                    (0x14U + ((n) * 0x28U))
#define CSL_GPIO_SET_DATA(n)                                    (0x18U + ((n) * 0x28U))
#define CSL_GPIO_CLR_DATA(n)                                    (0x1CU + ((n) * 0x28U))
#define CSL_GPIO_IN_DATA(n)                                     (0x20U + ((n) * 0x28U))
#define CSL_GPIO_SET_RIS_TRIG(n)                                (0x24U + ((n) * 0x28U))
#define CSL_GPIO_CLR_RIS_TRIG(n)                                (0x28U + ((n) * 0x28U))
#define CSL_GPIO_SET_FAL_TRIG(n)                                (0x2CU + ((n) * 0x28U))
#define CSL_GPIO_CLR_FAL_TRIG(n)                                (0x30U + ((n) * 0x28U))
#define CSL_GPIO_INTSTAT(n)                                     (0x34U + ((n) * 0x28U))
#define CSL_GPIO_PID                                            (0x0U)
#define CSL_GPIO_PCR                                            (0x4U)
#define CSL_GPIO_BINTEN                                         (0x8U)


/**************************************************************************
* Field Definition Macros
**************************************************************************/

/* DIR */

#define CSL_GPIO_DIR_DIR_SHIFT                                  (0U)
#define CSL_GPIO_DIR_DIR_MASK                                   (0x0000FFFFU)
#define CSL_GPIO_DIR_DIR_RESETVAL                               (0x00000000U)
#define CSL_GPIO_DIR_DIR_MAX                                    (0x0000ffffU)

#define CSL_GPIO_DIR_RESETVAL                                   (0x00000000U)

/* OUT_DATA */

#define CSL_GPIO_OUT_DATA_OUT_SHIFT                             (0U)
#define CSL_GPIO_OUT_DATA_OUT_MASK                              (0x0000FFFFU)
#define CSL_GPIO_OUT_DATA_OUT_RESETVAL                          (0x00000000U)
#define CSL_GPIO_OUT_DATA_OUT_MAX                               (0x0000ffffU)

#define CSL_GPIO_OUT_DATA_RESETVAL                              (0x00000000U)

/* SET_DATA */

#define CSL_GPIO_SET_DATA_SET_SHIFT                             (0U)
#define CSL_GPIO_SET_DATA_SET_MASK                              (0x0000FFFFU)
#define CSL_GPIO_SET_DATA_SET_RESETVAL                          (0x00000000U)
#define CSL_GPIO_SET_DATA_SET_MAX                               (0x0000ffffU)

#define CSL_GPIO_SET_DATA_RESETVAL                              (0x00000000U)

/* CLR_DATA */

#define CSL_GPIO_CLR_DATA_CLR_SHIFT                             (0U)
#define CSL_GPIO_CLR_DATA_CLR_MASK                              (0x0000FFFFU)
#define CSL_GPIO_CLR_DATA_CLR_RESETVAL                          (0x00000000U)
#define CSL_GPIO_CLR_DATA_CLR_MAX                               (0x0000ffffU)

#define CSL_GPIO_CLR_DATA_RESETVAL                              (0x00000000U)

/* IN_DATA */

#define CSL_GPIO_IN_DATA_IN_SHIFT                               (0U)
#define CSL_GPIO_IN_DATA_IN_MASK                                (0x0000FFFFU)
#define CSL_GPIO_IN_DATA_IN_RESETVAL                            (0x00000000U)
#define CSL_GPIO_IN_DATA_IN_MAX                                 (0x0000ffffU)

#define CSL_GPIO_IN_DATA_RESETVAL                               (0x00000000U)

/* SET_RIS_TRIG */

#define CSL_GPIO_SET_RIS_TRIG_SETRIS_SHIFT                      (0U)
#define CSL_GPIO_SET_RIS_TRIG_SETRIS_MASK                       (0x0000FFFFU)
#define CSL_GPIO_SET_RIS_TRIG_SETRIS_RESETVAL                   (0x00000000U)
#define CSL_GPIO_SET_RIS_TRIG_SETRIS_MAX                        (0x0000ffffU)

#define CSL_GPIO_SET_RIS_TRIG_RESETVAL                          (0x00000000U)

/* CLR_RIS_TRIG */

#define CSL_GPIO_CLR_RIS_TRIG_CLRRIS_SHIFT                      (0U)
#define CSL_GPIO_CLR_RIS_TRIG_CLRRIS_MASK                       (0x0000FFFFU)
#define CSL_GPIO_CLR_RIS_TRIG_CLRRIS_RESETVAL                   (0x00000000U)
#define CSL_GPIO_CLR_RIS_TRIG_CLRRIS_MAX                        (0x0000ffffU)

#define CSL_GPIO_CLR_RIS_TRIG_RESETVAL                          (0x00000000U)

/* SET_FAL_TRIG */

#define CSL_GPIO_SET_FAL_TRIG_SETFAL_SHIFT                      (0U)
#define CSL_GPIO_SET_FAL_TRIG_SETFAL_MASK                       (0x0000FFFFU)
#define CSL_GPIO_SET_FAL_TRIG_SETFAL_RESETVAL                   (0x00000000U)
#define CSL_GPIO_SET_FAL_TRIG_SETFAL_MAX                        (0x0000ffffU)

#define CSL_GPIO_SET_FAL_TRIG_RESETVAL                          (0x00000000U)

/* CLR_FAL_TRIG */

#define CSL_GPIO_CLR_FAL_TRIG_CLRFAL_SHIFT                      (0U)
#define CSL_GPIO_CLR_FAL_TRIG_CLRFAL_MASK                       (0x0000FFFFU)
#define CSL_GPIO_CLR_FAL_TRIG_CLRFAL_RESETVAL                   (0x00000000U)
#define CSL_GPIO_CLR_FAL_TRIG_CLRFAL_MAX                        (0x0000ffffU)

#define CSL_GPIO_CLR_FAL_TRIG_RESETVAL                          (0x00000000U)

/* INTSTAT */

#define CSL_GPIO_INTSTAT_STAT_SHIFT                             (0U)
#define CSL_GPIO_INTSTAT_STAT_MASK                              (0x0000FFFFU)
#define CSL_GPIO_INTSTAT_STAT_RESETVAL                          (0x00000000U)
#define CSL_GPIO_INTSTAT_STAT_MAX                               (0x0000ffffU)

#define CSL_GPIO_INTSTAT_RESETVAL                               (0x00000000U)

/* PID */

#define CSL_GPIO_PID_MINOR_SHIFT                                (0U)
#define CSL_GPIO_PID_MINOR_MASK                                 (0x0000003FU)
#define CSL_GPIO_PID_MINOR_RESETVAL                             (0x00000000U)
#define CSL_GPIO_PID_MINOR_MAX                                  (0x0000003fU)

#define CSL_GPIO_PID_CUSTOM_SHIFT                               (6U)
#define CSL_GPIO_PID_CUSTOM_MASK                                (0x000000C0U)
#define CSL_GPIO_PID_CUSTOM_RESETVAL                            (0x00000000U)
#define CSL_GPIO_PID_CUSTOM_MAX                                 (0x00000003U)

#define CSL_GPIO_PID_MAJOR_SHIFT                                (8U)
#define CSL_GPIO_PID_MAJOR_MASK                                 (0x00000700U)
#define CSL_GPIO_PID_MAJOR_RESETVAL                             (0x00000000U)
#define CSL_GPIO_PID_MAJOR_MAX                                  (0x00000007U)

#define CSL_GPIO_PID_RTL_SHIFT                                  (11U)
#define CSL_GPIO_PID_RTL_MASK                                   (0x0000F800U)
#define CSL_GPIO_PID_RTL_RESETVAL                               (0x00000000U)
#define CSL_GPIO_PID_RTL_MAX                                    (0x0000001fU)

#define CSL_GPIO_PID_FUNC_SHIFT                                 (16U)
#define CSL_GPIO_PID_FUNC_MASK                                  (0x0FFF0000U)
#define CSL_GPIO_PID_FUNC_RESETVAL                              (0x00000000U)
#define CSL_GPIO_PID_FUNC_MAX                                   (0x00000fffU)

#define CSL_GPIO_PID_SCHEME_SHIFT                               (30U)
#define CSL_GPIO_PID_SCHEME_MASK                                (0xC0000000U)
#define CSL_GPIO_PID_SCHEME_RESETVAL                            (0x00000000U)
#define CSL_GPIO_PID_SCHEME_MAX                                 (0x00000003U)

#define CSL_GPIO_PID_RESETVAL                                   (0x00000000U)

/* PCR */

#define CSL_GPIO_PCR_FREE_SHIFT                                 (0U)
#define CSL_GPIO_PCR_FREE_MASK                                  (0x00000001U)
#define CSL_GPIO_PCR_FREE_RESETVAL                              (0x00000000U)
#define CSL_GPIO_PCR_FREE_MAX                                   (0x00000001U)

#define CSL_GPIO_PCR_SOFT_SHIFT                                 (1U)
#define CSL_GPIO_PCR_SOFT_MASK                                  (0x00000002U)
#define CSL_GPIO_PCR_SOFT_RESETVAL                              (0x00000000U)
#define CSL_GPIO_PCR_SOFT_MAX                                   (0x00000001U)

#define CSL_GPIO_PCR_RESETVAL                                   (0x00000000U)

/* BINTEN */

#define CSL_GPIO_BINTEN_EN_SHIFT                                (0U)
#define CSL_GPIO_BINTEN_EN_MASK                                 (0x000000FFU)
#define CSL_GPIO_BINTEN_EN_RESETVAL                             (0x00000000U)
#define CSL_GPIO_BINTEN_EN_MAX                                  (0x000000ffU)

#define CSL_GPIO_BINTEN_RESETVAL                                (0x00000000U)

#ifdef __cplusplus
}
#endif
#endif
