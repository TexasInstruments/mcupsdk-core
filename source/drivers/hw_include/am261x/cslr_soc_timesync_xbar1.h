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
 *  Name        : cslr_soc_timesync_xbar1.h
*/
#ifndef CSLR_SOC_TIMESYNC_XBAR1_H_
#define CSLR_SOC_TIMESYNC_XBAR1_H_

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

#define SOC_TIMESYNC_XBAR1_CPTS_COMP                    0
#define SOC_TIMESYNC_XBAR1_CPTS_GENF0                   1
#define SOC_TIMESYNC_XBAR1_CPTS_GENF1                   2
#define SOC_TIMESYNC_XBAR1_CPTS_SYNC                    3
#define SOC_TIMESYNC_XBAR1_ICSSM0_EDC0_SYNC_OUT_0       4
#define SOC_TIMESYNC_XBAR1_ICSSM0_EDC0_SYNC_OUT_1       5
#define SOC_TIMESYNC_XBAR1_ICSSM1_EDC0_SYNC_OUT_0       6
#define SOC_TIMESYNC_XBAR1_ICSSM1_EDC0_SYNC_OUT_1       7
#define SOC_TIMESYNC_XBAR1_PWM_SYNCOUT_XBAR_OUT_0       8
#define SOC_TIMESYNC_XBAR1_PWM_SYNCOUT_XBAR_OUT_1       9
#define SOC_TIMESYNC_XBAR1_PWM_SYNCOUT_XBAR_OUT_2       10
#define SOC_TIMESYNC_XBAR1_PWM_SYNCOUT_XBAR_OUT_3       11
#define SOC_TIMESYNC_XBAR1_GPIO_INT_XBAR_OUT_8          12
#define SOC_TIMESYNC_XBAR1_GPIO_INT_XBAR_OUT_9          13
#define SOC_TIMESYNC_XBAR1_GPIO_INT_XBAR_OUT_10         14
#define SOC_TIMESYNC_XBAR1_GPIO_INT_XBAR_OUT_11         15
#define SOC_TIMESYNC_XBAR1_GPIO_INT_XBAR_OUT_12         16
#define SOC_TIMESYNC_XBAR1_GPIO_INT_XBAR_OUT_13         17

/**************************************************************************
    XBAR OUTPUT Macros
**************************************************************************/

#define SOC_TIMESYNC_XBAR1_DMA_TRIG_XBAR_0          0
#define SOC_TIMESYNC_XBAR1_DMA_TRIG_XBAR_1          1
#define SOC_TIMESYNC_XBAR1_VIM_MODULE0_0            2
#define SOC_TIMESYNC_XBAR1_VIM_MODULE0_1            3
#define SOC_TIMESYNC_XBAR1_VIM_MODULE0_2            4
#define SOC_TIMESYNC_XBAR1_VIM_MODULE0_3            5
#define SOC_TIMESYNC_XBAR1_VIM_MODULE1_0            6
#define SOC_TIMESYNC_XBAR1_VIM_MODULE1_1            7
#define SOC_TIMESYNC_XBAR1_VIM_MODULE1_2            8
#define SOC_TIMESYNC_XBAR1_VIM_MODULE1_3            9
#define SOC_TIMESYNC_XBAR1_ICSS_MODULE_0            10
#define SOC_TIMESYNC_XBAR1_ICSS_MODULE_1            11
#define SOC_TIMESYNC_XBAR1_ICSS_MODULE_2            12
#define SOC_TIMESYNC_XBAR1_ICSS_MODULE_3            13
#define SOC_TIMESYNC_XBAR1_ICSS_MODULE_4            14
#define SOC_TIMESYNC_XBAR1_ICSS_MODULE_5            15
#define SOC_TIMESYNC_XBAR1_ICSS_MODULE_6            16
#define SOC_TIMESYNC_XBAR1_ICSS_MODULE_7            17
#define SOC_TIMESYNC_XBAR1_CPSW_MODULE_0            18
#define SOC_TIMESYNC_XBAR1_CPSW_MODULE_1            19
#define SOC_TIMESYNC_XBAR1_CPSW_MODULE_2            20
#define SOC_TIMESYNC_XBAR1_CPSW_MODULE_3            21
#define SOC_TIMESYNC_XBAR1_CPSW_MODULE_4            22
#define SOC_TIMESYNC_XBAR1_CPSW_MODULE_5            23
#define SOC_TIMESYNC_XBAR1_CPSW_MODULE_6            24
#define SOC_TIMESYNC_XBAR1_CPSW_MODULE_7            25
#define SOC_TIMESYNC_XBAR1_VIM_MODULE2_0            26
#define SOC_TIMESYNC_XBAR1_VIM_MODULE2_1            27
#define SOC_TIMESYNC_XBAR1_VIM_MODULE2_2            28
#define SOC_TIMESYNC_XBAR1_VIM_MODULE2_3            29
#define SOC_TIMESYNC_XBAR1_VIM_MODULE3_0            30
#define SOC_TIMESYNC_XBAR1_VIM_MODULE3_1            31
#define SOC_TIMESYNC_XBAR1_VIM_MODULE3_2            32
#define SOC_TIMESYNC_XBAR1_VIM_MODULE3_3            33

/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t PID;
    volatile uint32_t MUXCNTL[12];
} CSL_soc_timesync_xbar1Regs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_SOC_TIMESYNC_XBAR1_PID                                             (0x00000000U)
#define CSL_SOC_TIMESYNC_XBAR1_MUXCNTL(MUXCNTL)                                (0x00000004U+((MUXCNTL)*0x4U))

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* PID */

#define CSL_SOC_TIMESYNC_XBAR1_PID_SCHEME_MASK                                 (0xC0000000U)
#define CSL_SOC_TIMESYNC_XBAR1_PID_SCHEME_SHIFT                                (0x0000001EU)
#define CSL_SOC_TIMESYNC_XBAR1_PID_SCHEME_RESETVAL                             (0x00000001U)
#define CSL_SOC_TIMESYNC_XBAR1_PID_SCHEME_MAX                                  (0x00000003U)

#define CSL_SOC_TIMESYNC_XBAR1_PID_BU_MASK                                     (0x30000000U)
#define CSL_SOC_TIMESYNC_XBAR1_PID_BU_SHIFT                                    (0x0000001CU)
#define CSL_SOC_TIMESYNC_XBAR1_PID_BU_RESETVAL                                 (0x00000002U)
#define CSL_SOC_TIMESYNC_XBAR1_PID_BU_MAX                                      (0x00000003U)

#define CSL_SOC_TIMESYNC_XBAR1_PID_FUNCTION_MASK                               (0x0FFF0000U)
#define CSL_SOC_TIMESYNC_XBAR1_PID_FUNCTION_SHIFT                              (0x00000010U)
#define CSL_SOC_TIMESYNC_XBAR1_PID_FUNCTION_RESETVAL                           (0x00000694U)
#define CSL_SOC_TIMESYNC_XBAR1_PID_FUNCTION_MAX                                (0x00000FFFU)

#define CSL_SOC_TIMESYNC_XBAR1_PID_RTLVER_MASK                                 (0x0000F800U)
#define CSL_SOC_TIMESYNC_XBAR1_PID_RTLVER_SHIFT                                (0x0000000BU)
#define CSL_SOC_TIMESYNC_XBAR1_PID_RTLVER_RESETVAL                             (0x00000010U)
#define CSL_SOC_TIMESYNC_XBAR1_PID_RTLVER_MAX                                  (0x0000001FU)

#define CSL_SOC_TIMESYNC_XBAR1_PID_MAJREV_MASK                                 (0x00000700U)
#define CSL_SOC_TIMESYNC_XBAR1_PID_MAJREV_SHIFT                                (0x00000008U)
#define CSL_SOC_TIMESYNC_XBAR1_PID_MAJREV_RESETVAL                             (0x00000001U)
#define CSL_SOC_TIMESYNC_XBAR1_PID_MAJREV_MAX                                  (0x00000007U)

#define CSL_SOC_TIMESYNC_XBAR1_PID_CUSTOM_MASK                                 (0x000000C0U)
#define CSL_SOC_TIMESYNC_XBAR1_PID_CUSTOM_SHIFT                                (0x00000006U)
#define CSL_SOC_TIMESYNC_XBAR1_PID_CUSTOM_RESETVAL                             (0x00000000U)
#define CSL_SOC_TIMESYNC_XBAR1_PID_CUSTOM_MAX                                  (0x00000003U)

#define CSL_SOC_TIMESYNC_XBAR1_PID_MINREV_MASK                                 (0x0000003FU)
#define CSL_SOC_TIMESYNC_XBAR1_PID_MINREV_SHIFT                                (0x00000000U)
#define CSL_SOC_TIMESYNC_XBAR1_PID_MINREV_RESETVAL                             (0x00000000U)
#define CSL_SOC_TIMESYNC_XBAR1_PID_MINREV_MAX                                  (0x0000003FU)

#define CSL_SOC_TIMESYNC_XBAR1_PID_RESETVAL                                    (0x66948100U)

/* MUXCNTL */

#define CSL_SOC_TIMESYNC_XBAR1_MUXCNTL_INT_ENABLE_MASK                         (0x00010000U)
#define CSL_SOC_TIMESYNC_XBAR1_MUXCNTL_INT_ENABLE_SHIFT                        (0x00000010U)
#define CSL_SOC_TIMESYNC_XBAR1_MUXCNTL_INT_ENABLE_RESETVAL                     (0x00000000U)
#define CSL_SOC_TIMESYNC_XBAR1_MUXCNTL_INT_ENABLE_MAX                          (0x00000001U)

#define CSL_SOC_TIMESYNC_XBAR1_MUXCNTL_ENABLE_MASK                             (0x0000003FU)
#define CSL_SOC_TIMESYNC_XBAR1_MUXCNTL_ENABLE_SHIFT                            (0x00000000U)
#define CSL_SOC_TIMESYNC_XBAR1_MUXCNTL_ENABLE_RESETVAL                         (0x00000000U)
#define CSL_SOC_TIMESYNC_XBAR1_MUXCNTL_ENABLE_MAX                              (0x0000003FU)

#define CSL_SOC_TIMESYNC_XBAR1_MUXCNTL_RESETVAL                                (0x00000000U)

#ifdef __cplusplus
}
#endif
#endif
