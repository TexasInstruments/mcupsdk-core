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
 *  Name        : cslr_dss_rcm.h
*/
#ifndef SDLR_DSS_RCM_H_
#define SDLR_DSS_RCM_H_

#include <sdl/include/sdlr.h>


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
    volatile uint32_t PID;
    volatile uint32_t HW_REG0;
    volatile uint32_t HW_REG1;
    volatile uint32_t HW_REG2;
    volatile uint32_t HW_REG3;
    volatile uint32_t DSP_PD_CTRL;
    volatile uint32_t DSP_PD_TRIGGER_WAKUP;
    volatile uint32_t DSP_PD_TRIGGER_SLEEP;
    volatile uint32_t DSP_PD_STATUS;
    volatile uint32_t DSP_PD_CTRL_MISC0;
    volatile uint32_t DSP_PD_CTRL_MISC1;
    volatile uint32_t DSP_PD_STATUS_MISC0;
    volatile uint32_t DSP_PD_WAKEUP_MASK0;
    volatile uint32_t DSP_PD_WAKEUP_MASK1;
    volatile uint32_t DSP_PD_WAKEUP_MASK2;
    volatile uint32_t DSP_PD_WAKEUP_STATUS0;
    volatile uint32_t DSP_PD_WAKEUP_STATUS1;
    volatile uint32_t DSP_PD_WAKEUP_STATUS2;
    volatile uint32_t DSP_PD_WAKEUP_STATUS0_CLR;
    volatile uint32_t DSP_PD_WAKEUP_STATUS1_CLR;
    volatile uint32_t DSP_PD_WAKEUP_STATUS2_CLR;
    volatile uint32_t DSP_PD_MISSED_EVENT_MASK0;
    volatile uint32_t DSP_PD_MISSED_EVENT_MASK1;
    volatile uint32_t DSP_PD_MISSED_EVENT_MASK2;
    volatile uint32_t DSP_PD_MISSED_EVENT_STATUS0;
    volatile uint32_t DSP_PD_MISSED_EVENT_STATUS1;
    volatile uint32_t DSP_PD_MISSED_EVENT_STATUS2;
    volatile uint32_t DSP_RST_CAUSE;
    volatile uint32_t DSP_RST_CAUSE_CLR;
    volatile uint32_t DSP_STC_PBIST_CTRL;
    volatile uint32_t DSP_STC_PBIST_STATUS;
    volatile uint32_t DSP_STC_PBIST_CTRL_MISC0;
    volatile uint32_t DSP_STC_PBIST_CTRL_MISC1;
    volatile uint32_t DSP_STC_PBIST_START;
    volatile uint32_t DSP_STC_PBIST_STATUS_CLR;
    volatile uint32_t DSS_DSP_CLK_SRC_SEL;
    volatile uint32_t DSS_HWA_CLK_SRC_SEL;
    volatile uint32_t DSS_RTIA_CLK_SRC_SEL;
    volatile uint32_t DSS_RTIB_CLK_SRC_SEL;
    volatile uint32_t DSS_WDT_CLK_SRC_SEL;
    volatile uint32_t DSS_SCIA_CLK_SRC_SEL;
    volatile uint32_t DSS_DSP_CLK_DIV_VAL;
    volatile uint32_t DSS_RTIA_CLK_DIV_VAL;
    volatile uint32_t DSS_RTIB_CLK_DIV_VAL;
    volatile uint32_t DSS_WDT_CLK_DIV_VAL;
    volatile uint32_t DSS_SCIA_CLK_DIV_VAL;
    volatile uint32_t DSS_DSP_CLK_GATE;
    volatile uint32_t DSS_HWA_CLK_GATE;
    volatile uint32_t DSS_RTIA_CLK_GATE;
    volatile uint32_t DSS_RTIB_CLK_GATE;
    volatile uint32_t DSS_WDT_CLK_GATE;
    volatile uint32_t DSS_SCIA_CLK_GATE;
    volatile uint32_t DSS_CBUFF_CLK_GATE;
    volatile uint32_t DSS_DSP_CLK_STATUS;
    volatile uint32_t DSS_HWA_CLK_STATUS;
    volatile uint32_t DSS_RTIA_CLK_STATUS;
    volatile uint32_t DSS_RTIB_CLK_STATUS;
    volatile uint32_t DSS_WDT_CLK_STATUS;
    volatile uint32_t DSS_SCIA_CLK_STATUS;
    volatile uint32_t DSS_DSP_RST_CTRL;
    volatile uint32_t DSS_ESM_RST_CTRL;
    volatile uint32_t DSS_SCIA_RST_CTRL;
    volatile uint32_t DSS_RTIA_RST_CTRL;
    volatile uint32_t DSS_RTIB_RST_CTRL;
    volatile uint32_t DSS_WDT_RST_CTRL;
    volatile uint32_t DSS_DCCA_RST_CTRL;
    volatile uint32_t DSS_DCCB_RST_CTRL;
    volatile uint32_t DSS_MCRC_RST_CTRL;
    volatile uint32_t DSP_DFT_DIV_CTRL;
    volatile uint32_t DSS_DSP_L2_PD_CTRL;
    volatile uint32_t DSS_L3_BANKA0_PD_CTRL;
    volatile uint32_t DSS_L3_BANKA1_PD_CTRL;
    volatile uint32_t DSS_L3_BANKA2_PD_CTRL;
    volatile uint32_t DSS_L3_BANKA3_PD_CTRL;
    volatile uint32_t DSS_L3_BANKB0_PD_CTRL;
    volatile uint32_t DSS_L3_BANKB1_PD_CTRL;
    volatile uint32_t DSS_L3_BANKB2_PD_CTRL;
    volatile uint32_t DSS_L3_BANKB3_PD_CTRL;
    volatile uint32_t DSS_L3_BANKC0_PD_CTRL;
    volatile uint32_t DSS_L3_BANKC1_PD_CTRL;
    volatile uint32_t DSS_L3_BANKC2_PD_CTRL;
    volatile uint32_t DSS_L3_BANKC3_PD_CTRL;
    volatile uint32_t DSS_L3_BANKD0_PD_CTRL;
    volatile uint32_t DSS_L3_BANKD1_PD_CTRL;
    volatile uint32_t DSS_L3_BANKD2_PD_CTRL;
    volatile uint8_t  Resv_344[4];
    volatile uint32_t DSS_HWA_PD_CTRL;
    volatile uint32_t DSS_DSP_L2_PD_STATUS;
    volatile uint32_t DSS_L3_BANKA0_PD_STATUS;
    volatile uint32_t DSS_L3_BANKA1_PD_STATUS;
    volatile uint32_t DSS_L3_BANKA2_PD_STATUS;
    volatile uint32_t DSS_L3_BANKA3_PD_STATUS;
    volatile uint32_t DSS_L3_BANKB0_PD_STATUS;
    volatile uint32_t DSS_L3_BANKB1_PD_STATUS;
    volatile uint32_t DSS_L3_BANKB2_PD_STATUS;
    volatile uint32_t DSS_L3_BANKB3_PD_STATUS;
    volatile uint32_t DSS_L3_BANKC0_PD_STATUS;
    volatile uint32_t DSS_L3_BANKC1_PD_STATUS;
    volatile uint32_t DSS_L3_BANKC2_PD_STATUS;
    volatile uint32_t DSS_L3_BANKC3_PD_STATUS;
    volatile uint32_t DSS_L3_BANKD0_PD_STATUS;
    volatile uint32_t DSS_L3_BANKD1_PD_STATUS;
    volatile uint32_t DSS_L3_BANKD2_PD_STATUS;
    volatile uint8_t  Resv_416[4];
    volatile uint32_t DSS_HWA_PD_STATUS;
    volatile uint32_t DSS_DSP_TRCCLK_DIVRATIO;
    volatile uint32_t DSS_DSP_TCLK_DIVRATIO;
    volatile uint32_t DSS_DSP_DITHERED_CLK_CTRL;
    volatile uint32_t DSS_L3_PD_CTRL_STICKYBIT;
    volatile uint32_t DSP_PD_CTRL_MISC2;
    volatile uint32_t DSP_PD_CTRL_MISC3;
    volatile uint32_t DSP_PD_CTRL_OVERRIDE0;
    volatile uint32_t DSP_PD_CTRL_OVERRIDE1;
    volatile uint32_t DSP_PD_CTRL_OVERRIDE2;
    volatile uint32_t DSS_HWA_RST_CTRL;
    volatile uint32_t DSS_TPCCA_RST_CTRL;
    volatile uint32_t DSS_TPCCB_RST_CTRL;
    volatile uint32_t DSS_TPCCC_RST_CTRL;
    volatile uint32_t DSS_TPTCA_RST_CTRL;
    volatile uint32_t DSS_TPTCB_RST_CTRL;
    volatile uint32_t DSS_TPTCC_RST_CTRL;
    volatile uint8_t  Resv_4048[3564];
    volatile uint32_t HW_SPARE_RW0;
    volatile uint32_t HW_SPARE_RW1;
    volatile uint32_t HW_SPARE_RW2;
    volatile uint32_t HW_SPARE_RW3;
    volatile uint32_t HW_SPARE_RO0;
    volatile uint32_t HW_SPARE_RO1;
    volatile uint32_t HW_SPARE_RO2;
    volatile uint32_t HW_SPARE_RO3;
    volatile uint32_t HW_SPARE_WPH;
    volatile uint32_t HW_SPARE_REC;
    volatile uint8_t  Resv_4104[16];
    volatile uint32_t LOCK0_KICK0;
    volatile uint32_t LOCK0_KICK1;
    volatile uint32_t INTR_RAW_STATUS;
    volatile uint32_t INTR_ENABLED_STATUS_CLEAR;
    volatile uint32_t INTR_ENABLE;
    volatile uint32_t INTR_ENABLE_CLEAR;
    volatile uint32_t EOI;
    volatile uint32_t FAULT_ADDRESS;
    volatile uint32_t FAULT_TYPE_STATUS;
    volatile uint32_t FAULT_ATTR_STATUS;
    volatile uint32_t FAULT_CLEAR;
} SDL_dss_rcmRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define SDL_DSS_RCM_PID                                                        (0x00000000U)
#define SDL_DSS_RCM_HW_REG0                                                    (0x00000004U)
#define SDL_DSS_RCM_HW_REG1                                                    (0x00000008U)
#define SDL_DSS_RCM_HW_REG2                                                    (0x0000000CU)
#define SDL_DSS_RCM_HW_REG3                                                    (0x00000010U)
#define SDL_DSS_RCM_DSP_PD_CTRL                                                (0x00000014U)
#define SDL_DSS_RCM_DSP_PD_TRIGGER_WAKUP                                       (0x00000018U)
#define SDL_DSS_RCM_DSP_PD_TRIGGER_SLEEP                                       (0x0000001CU)
#define SDL_DSS_RCM_DSP_PD_STATUS                                              (0x00000020U)
#define SDL_DSS_RCM_DSP_PD_CTRL_MISC0                                          (0x00000024U)
#define SDL_DSS_RCM_DSP_PD_CTRL_MISC1                                          (0x00000028U)
#define SDL_DSS_RCM_DSP_PD_STATUS_MISC0                                        (0x0000002CU)
#define SDL_DSS_RCM_DSP_PD_WAKEUP_MASK0                                        (0x00000030U)
#define SDL_DSS_RCM_DSP_PD_WAKEUP_MASK1                                        (0x00000034U)
#define SDL_DSS_RCM_DSP_PD_WAKEUP_MASK2                                        (0x00000038U)
#define SDL_DSS_RCM_DSP_PD_WAKEUP_STATUS0                                      (0x0000003CU)
#define SDL_DSS_RCM_DSP_PD_WAKEUP_STATUS1                                      (0x00000040U)
#define SDL_DSS_RCM_DSP_PD_WAKEUP_STATUS2                                      (0x00000044U)
#define SDL_DSS_RCM_DSP_PD_WAKEUP_STATUS0_CLR                                  (0x00000048U)
#define SDL_DSS_RCM_DSP_PD_WAKEUP_STATUS1_CLR                                  (0x0000004CU)
#define SDL_DSS_RCM_DSP_PD_WAKEUP_STATUS2_CLR                                  (0x00000050U)
#define SDL_DSS_RCM_DSP_PD_MISSED_EVENT_MASK0                                  (0x00000054U)
#define SDL_DSS_RCM_DSP_PD_MISSED_EVENT_MASK1                                  (0x00000058U)
#define SDL_DSS_RCM_DSP_PD_MISSED_EVENT_MASK2                                  (0x0000005CU)
#define SDL_DSS_RCM_DSP_PD_MISSED_EVENT_STATUS0                                (0x00000060U)
#define SDL_DSS_RCM_DSP_PD_MISSED_EVENT_STATUS1                                (0x00000064U)
#define SDL_DSS_RCM_DSP_PD_MISSED_EVENT_STATUS2                                (0x00000068U)
#define SDL_DSS_RCM_DSP_RST_CAUSE                                              (0x0000006CU)
#define SDL_DSS_RCM_DSP_RST_CAUSE_CLR                                          (0x00000070U)
#define SDL_DSS_RCM_DSP_STC_PBIST_CTRL                                         (0x00000074U)
#define SDL_DSS_RCM_DSP_STC_PBIST_STATUS                                       (0x00000078U)
#define SDL_DSS_RCM_DSP_STC_PBIST_CTRL_MISC0                                   (0x0000007CU)
#define SDL_DSS_RCM_DSP_STC_PBIST_CTRL_MISC1                                   (0x00000080U)
#define SDL_DSS_RCM_DSP_STC_PBIST_START                                        (0x00000084U)
#define SDL_DSS_RCM_DSP_STC_PBIST_STATUS_CLR                                   (0x00000088U)
#define SDL_DSS_RCM_DSS_DSP_CLK_SRC_SEL                                        (0x0000008CU)
#define SDL_DSS_RCM_DSS_HWA_CLK_SRC_SEL                                        (0x00000090U)
#define SDL_DSS_RCM_DSS_RTIA_CLK_SRC_SEL                                       (0x00000094U)
#define SDL_DSS_RCM_DSS_RTIB_CLK_SRC_SEL                                       (0x00000098U)
#define SDL_DSS_RCM_DSS_WDT_CLK_SRC_SEL                                        (0x0000009CU)
#define SDL_DSS_RCM_DSS_SCIA_CLK_SRC_SEL                                       (0x000000A0U)
#define SDL_DSS_RCM_DSS_DSP_CLK_DIV_VAL                                        (0x000000A4U)
#define SDL_DSS_RCM_DSS_RTIA_CLK_DIV_VAL                                       (0x000000A8U)
#define SDL_DSS_RCM_DSS_RTIB_CLK_DIV_VAL                                       (0x000000ACU)
#define SDL_DSS_RCM_DSS_WDT_CLK_DIV_VAL                                        (0x000000B0U)
#define SDL_DSS_RCM_DSS_SCIA_CLK_DIV_VAL                                       (0x000000B4U)
#define SDL_DSS_RCM_DSS_DSP_CLK_GATE                                           (0x000000B8U)
#define SDL_DSS_RCM_DSS_HWA_CLK_GATE                                           (0x000000BCU)
#define SDL_DSS_RCM_DSS_RTIA_CLK_GATE                                          (0x000000C0U)
#define SDL_DSS_RCM_DSS_RTIB_CLK_GATE                                          (0x000000C4U)
#define SDL_DSS_RCM_DSS_WDT_CLK_GATE                                           (0x000000C8U)
#define SDL_DSS_RCM_DSS_SCIA_CLK_GATE                                          (0x000000CCU)
#define SDL_DSS_RCM_DSS_CBUFF_CLK_GATE                                         (0x000000D0U)
#define SDL_DSS_RCM_DSS_DSP_CLK_STATUS                                         (0x000000D4U)
#define SDL_DSS_RCM_DSS_HWA_CLK_STATUS                                         (0x000000D8U)
#define SDL_DSS_RCM_DSS_RTIA_CLK_STATUS                                        (0x000000DCU)
#define SDL_DSS_RCM_DSS_RTIB_CLK_STATUS                                        (0x000000E0U)
#define SDL_DSS_RCM_DSS_WDT_CLK_STATUS                                         (0x000000E4U)
#define SDL_DSS_RCM_DSS_SCIA_CLK_STATUS                                        (0x000000E8U)
#define SDL_DSS_RCM_DSS_DSP_RST_CTRL                                           (0x000000ECU)
#define SDL_DSS_RCM_DSS_ESM_RST_CTRL                                           (0x000000F0U)
#define SDL_DSS_RCM_DSS_SCIA_RST_CTRL                                          (0x000000F4U)
#define SDL_DSS_RCM_DSS_RTIA_RST_CTRL                                          (0x000000F8U)
#define SDL_DSS_RCM_DSS_RTIB_RST_CTRL                                          (0x000000FCU)
#define SDL_DSS_RCM_DSS_WDT_RST_CTRL                                           (0x00000100U)
#define SDL_DSS_RCM_DSS_DCCA_RST_CTRL                                          (0x00000104U)
#define SDL_DSS_RCM_DSS_DCCB_RST_CTRL                                          (0x00000108U)
#define SDL_DSS_RCM_DSS_MCRC_RST_CTRL                                          (0x0000010CU)
#define SDL_DSS_RCM_DSP_DFT_DIV_CTRL                                           (0x00000110U)
#define SDL_DSS_RCM_DSS_DSP_L2_PD_CTRL                                         (0x00000114U)
#define SDL_DSS_RCM_DSS_L3_BANKA0_PD_CTRL                                      (0x00000118U)
#define SDL_DSS_RCM_DSS_L3_BANKA1_PD_CTRL                                      (0x0000011CU)
#define SDL_DSS_RCM_DSS_L3_BANKA2_PD_CTRL                                      (0x00000120U)
#define SDL_DSS_RCM_DSS_L3_BANKA3_PD_CTRL                                      (0x00000124U)
#define SDL_DSS_RCM_DSS_L3_BANKB0_PD_CTRL                                      (0x00000128U)
#define SDL_DSS_RCM_DSS_L3_BANKB1_PD_CTRL                                      (0x0000012CU)
#define SDL_DSS_RCM_DSS_L3_BANKB2_PD_CTRL                                      (0x00000130U)
#define SDL_DSS_RCM_DSS_L3_BANKB3_PD_CTRL                                      (0x00000134U)
#define SDL_DSS_RCM_DSS_L3_BANKC0_PD_CTRL                                      (0x00000138U)
#define SDL_DSS_RCM_DSS_L3_BANKC1_PD_CTRL                                      (0x0000013CU)
#define SDL_DSS_RCM_DSS_L3_BANKC2_PD_CTRL                                      (0x00000140U)
#define SDL_DSS_RCM_DSS_L3_BANKC3_PD_CTRL                                      (0x00000144U)
#define SDL_DSS_RCM_DSS_L3_BANKD0_PD_CTRL                                      (0x00000148U)
#define SDL_DSS_RCM_DSS_L3_BANKD1_PD_CTRL                                      (0x0000014CU)
#define SDL_DSS_RCM_DSS_L3_BANKD2_PD_CTRL                                      (0x00000150U)
#define SDL_DSS_RCM_DSS_HWA_PD_CTRL                                            (0x00000158U)
#define SDL_DSS_RCM_DSS_DSP_L2_PD_STATUS                                       (0x0000015CU)
#define SDL_DSS_RCM_DSS_L3_BANKA0_PD_STATUS                                    (0x00000160U)
#define SDL_DSS_RCM_DSS_L3_BANKA1_PD_STATUS                                    (0x00000164U)
#define SDL_DSS_RCM_DSS_L3_BANKA2_PD_STATUS                                    (0x00000168U)
#define SDL_DSS_RCM_DSS_L3_BANKA3_PD_STATUS                                    (0x0000016CU)
#define SDL_DSS_RCM_DSS_L3_BANKB0_PD_STATUS                                    (0x00000170U)
#define SDL_DSS_RCM_DSS_L3_BANKB1_PD_STATUS                                    (0x00000174U)
#define SDL_DSS_RCM_DSS_L3_BANKB2_PD_STATUS                                    (0x00000178U)
#define SDL_DSS_RCM_DSS_L3_BANKB3_PD_STATUS                                    (0x0000017CU)
#define SDL_DSS_RCM_DSS_L3_BANKC0_PD_STATUS                                    (0x00000180U)
#define SDL_DSS_RCM_DSS_L3_BANKC1_PD_STATUS                                    (0x00000184U)
#define SDL_DSS_RCM_DSS_L3_BANKC2_PD_STATUS                                    (0x00000188U)
#define SDL_DSS_RCM_DSS_L3_BANKC3_PD_STATUS                                    (0x0000018CU)
#define SDL_DSS_RCM_DSS_L3_BANKD0_PD_STATUS                                    (0x00000190U)
#define SDL_DSS_RCM_DSS_L3_BANKD1_PD_STATUS                                    (0x00000194U)
#define SDL_DSS_RCM_DSS_L3_BANKD2_PD_STATUS                                    (0x00000198U)
#define SDL_DSS_RCM_DSS_HWA_PD_STATUS                                          (0x000001A0U)
#define SDL_DSS_RCM_DSS_DSP_TRCCLK_DIVRATIO                                    (0x000001A4U)
#define SDL_DSS_RCM_DSS_DSP_TCLK_DIVRATIO                                      (0x000001A8U)
#define SDL_DSS_RCM_DSS_DSP_DITHERED_CLK_CTRL                                  (0x000001ACU)
#define SDL_DSS_RCM_DSS_L3_PD_CTRL_STICKYBIT                                   (0x000001B0U)
#define SDL_DSS_RCM_DSP_PD_CTRL_MISC2                                          (0x000001B4U)
#define SDL_DSS_RCM_DSP_PD_CTRL_MISC3                                          (0x000001B8U)
#define SDL_DSS_RCM_DSP_PD_CTRL_OVERRIDE0                                      (0x000001BCU)
#define SDL_DSS_RCM_DSP_PD_CTRL_OVERRIDE1                                      (0x000001C0U)
#define SDL_DSS_RCM_DSP_PD_CTRL_OVERRIDE2                                      (0x000001C4U)
#define SDL_DSS_RCM_DSS_HWA_RST_CTRL                                           (0x000001C8U)
#define SDL_DSS_RCM_DSS_TPCCA_RST_CTRL                                         (0x000001CCU)
#define SDL_DSS_RCM_DSS_TPCCB_RST_CTRL                                         (0x000001D0U)
#define SDL_DSS_RCM_DSS_TPCCC_RST_CTRL                                         (0x000001D4U)
#define SDL_DSS_RCM_DSS_TPTCA_RST_CTRL                                         (0x000001D8U)
#define SDL_DSS_RCM_DSS_TPTCB_RST_CTRL                                         (0x000001DCU)
#define SDL_DSS_RCM_DSS_TPTCC_RST_CTRL                                         (0x000001E0U)
#define SDL_DSS_RCM_HW_SPARE_RW0                                               (0x00000FD0U)
#define SDL_DSS_RCM_HW_SPARE_RW1                                               (0x00000FD4U)
#define SDL_DSS_RCM_HW_SPARE_RW2                                               (0x00000FD8U)
#define SDL_DSS_RCM_HW_SPARE_RW3                                               (0x00000FDCU)
#define SDL_DSS_RCM_HW_SPARE_RO0                                               (0x00000FE0U)
#define SDL_DSS_RCM_HW_SPARE_RO1                                               (0x00000FE4U)
#define SDL_DSS_RCM_HW_SPARE_RO2                                               (0x00000FE8U)
#define SDL_DSS_RCM_HW_SPARE_RO3                                               (0x00000FECU)
#define SDL_DSS_RCM_HW_SPARE_WPH                                               (0x00000FF0U)
#define SDL_DSS_RCM_HW_SPARE_REC                                               (0x00000FF4U)
#define SDL_DSS_RCM_LOCK0_KICK0                                                (0x00001008U)
#define SDL_DSS_RCM_LOCK0_KICK1                                                (0x0000100CU)
#define SDL_DSS_RCM_INTR_RAW_STATUS                                            (0x00001010U)
#define SDL_DSS_RCM_INTR_ENABLED_STATUS_CLEAR                                  (0x00001014U)
#define SDL_DSS_RCM_INTR_ENABLE                                                (0x00001018U)
#define SDL_DSS_RCM_INTR_ENABLE_CLEAR                                          (0x0000101CU)
#define SDL_DSS_RCM_EOI                                                        (0x00001020U)
#define SDL_DSS_RCM_FAULT_ADDRESS                                              (0x00001024U)
#define SDL_DSS_RCM_FAULT_TYPE_STATUS                                          (0x00001028U)
#define SDL_DSS_RCM_FAULT_ATTR_STATUS                                          (0x0000102CU)
#define SDL_DSS_RCM_FAULT_CLEAR                                                (0x00001030U)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* PID */

#define SDL_DSS_RCM_PID_PID_MINOR_MASK                                         (0x0000003FU)
#define SDL_DSS_RCM_PID_PID_MINOR_SHIFT                                        (0x00000000U)
#define SDL_DSS_RCM_PID_PID_MINOR_RESETVAL                                     (0x00000013U)
#define SDL_DSS_RCM_PID_PID_MINOR_MAX                                          (0x0000003FU)

#define SDL_DSS_RCM_PID_PID_CUSTOM_MASK                                        (0x000000C0U)
#define SDL_DSS_RCM_PID_PID_CUSTOM_SHIFT                                       (0x00000006U)
#define SDL_DSS_RCM_PID_PID_CUSTOM_RESETVAL                                    (0x00000000U)
#define SDL_DSS_RCM_PID_PID_CUSTOM_MAX                                         (0x00000003U)

#define SDL_DSS_RCM_PID_PID_MAJOR_MASK                                         (0x00000700U)
#define SDL_DSS_RCM_PID_PID_MAJOR_SHIFT                                        (0x00000008U)
#define SDL_DSS_RCM_PID_PID_MAJOR_RESETVAL                                     (0x00000002U)
#define SDL_DSS_RCM_PID_PID_MAJOR_MAX                                          (0x00000007U)

#define SDL_DSS_RCM_PID_PID_MISC_MASK                                          (0x0000F800U)
#define SDL_DSS_RCM_PID_PID_MISC_SHIFT                                         (0x0000000BU)
#define SDL_DSS_RCM_PID_PID_MISC_RESETVAL                                      (0x00000000U)
#define SDL_DSS_RCM_PID_PID_MISC_MAX                                           (0x0000001FU)

#define SDL_DSS_RCM_PID_PID_MSB16_MASK                                         (0xFFFF0000U)
#define SDL_DSS_RCM_PID_PID_MSB16_SHIFT                                        (0x00000010U)
#define SDL_DSS_RCM_PID_PID_MSB16_RESETVAL                                     (0x00006180U)
#define SDL_DSS_RCM_PID_PID_MSB16_MAX                                          (0x0000FFFFU)

#define SDL_DSS_RCM_PID_RESETVAL                                               (0x61800213U)

/* HW_REG0 */

#define SDL_DSS_RCM_HW_REG0_HW_REG0_HWREG_MASK                                 (0xFFFFFFFFU)
#define SDL_DSS_RCM_HW_REG0_HW_REG0_HWREG_SHIFT                                (0x00000000U)
#define SDL_DSS_RCM_HW_REG0_HW_REG0_HWREG_RESETVAL                             (0x00000000U)
#define SDL_DSS_RCM_HW_REG0_HW_REG0_HWREG_MAX                                  (0xFFFFFFFFU)

#define SDL_DSS_RCM_HW_REG0_RESETVAL                                           (0x00000000U)

/* HW_REG1 */

#define SDL_DSS_RCM_HW_REG1_HW_REG1_HWREG_MASK                                 (0xFFFFFFFFU)
#define SDL_DSS_RCM_HW_REG1_HW_REG1_HWREG_SHIFT                                (0x00000000U)
#define SDL_DSS_RCM_HW_REG1_HW_REG1_HWREG_RESETVAL                             (0x00000000U)
#define SDL_DSS_RCM_HW_REG1_HW_REG1_HWREG_MAX                                  (0xFFFFFFFFU)

#define SDL_DSS_RCM_HW_REG1_RESETVAL                                           (0x00000000U)

/* HW_REG2 */

#define SDL_DSS_RCM_HW_REG2_HW_REG2_HWREG_MASK                                 (0xFFFFFFFFU)
#define SDL_DSS_RCM_HW_REG2_HW_REG2_HWREG_SHIFT                                (0x00000000U)
#define SDL_DSS_RCM_HW_REG2_HW_REG2_HWREG_RESETVAL                             (0x00000000U)
#define SDL_DSS_RCM_HW_REG2_HW_REG2_HWREG_MAX                                  (0xFFFFFFFFU)

#define SDL_DSS_RCM_HW_REG2_RESETVAL                                           (0x00000000U)

/* HW_REG3 */

#define SDL_DSS_RCM_HW_REG3_HW_REG3_HWREG_MASK                                 (0xFFFFFFFFU)
#define SDL_DSS_RCM_HW_REG3_HW_REG3_HWREG_SHIFT                                (0x00000000U)
#define SDL_DSS_RCM_HW_REG3_HW_REG3_HWREG_RESETVAL                             (0x00000000U)
#define SDL_DSS_RCM_HW_REG3_HW_REG3_HWREG_MAX                                  (0xFFFFFFFFU)

#define SDL_DSS_RCM_HW_REG3_RESETVAL                                           (0x00000000U)

/* DSP_PD_CTRL */

#define SDL_DSS_RCM_DSP_PD_CTRL_DSP_PD_CTRL_INTERRUPT_MASK_MASK                (0x00000001U)
#define SDL_DSS_RCM_DSP_PD_CTRL_DSP_PD_CTRL_INTERRUPT_MASK_SHIFT               (0x00000000U)
#define SDL_DSS_RCM_DSP_PD_CTRL_DSP_PD_CTRL_INTERRUPT_MASK_RESETVAL            (0x00000001U)
#define SDL_DSS_RCM_DSP_PD_CTRL_DSP_PD_CTRL_INTERRUPT_MASK_MAX                 (0x00000001U)

#define SDL_DSS_RCM_DSP_PD_CTRL_DSP_PD_CTRL_PROC_HALT_MASK                     (0x00000010U)
#define SDL_DSS_RCM_DSP_PD_CTRL_DSP_PD_CTRL_PROC_HALT_SHIFT                    (0x00000004U)
#define SDL_DSS_RCM_DSP_PD_CTRL_DSP_PD_CTRL_PROC_HALT_RESETVAL                 (0x00000001U)
#define SDL_DSS_RCM_DSP_PD_CTRL_DSP_PD_CTRL_PROC_HALT_MAX                      (0x00000001U)

#define SDL_DSS_RCM_DSP_PD_CTRL_RESETVAL                                       (0x00000011U)

/* DSP_PD_TRIGGER_WAKUP */

#define SDL_DSS_RCM_DSP_PD_TRIGGER_WAKUP_DSP_PD_TRIGGER_WAKUP_WAKEUP_TRIGGER_MASK (0x00000001U)
#define SDL_DSS_RCM_DSP_PD_TRIGGER_WAKUP_DSP_PD_TRIGGER_WAKUP_WAKEUP_TRIGGER_SHIFT (0x00000000U)
#define SDL_DSS_RCM_DSP_PD_TRIGGER_WAKUP_DSP_PD_TRIGGER_WAKUP_WAKEUP_TRIGGER_RESETVAL (0x00000000U)
#define SDL_DSS_RCM_DSP_PD_TRIGGER_WAKUP_DSP_PD_TRIGGER_WAKUP_WAKEUP_TRIGGER_MAX (0x00000001U)

#define SDL_DSS_RCM_DSP_PD_TRIGGER_WAKUP_RESETVAL                              (0x00000000U)

/* DSP_PD_TRIGGER_SLEEP */

#define SDL_DSS_RCM_DSP_PD_TRIGGER_SLEEP_DSP_PD_TRIGGER_SLEEP_SLEEP_TRIGGER_MASK (0x00000001U)
#define SDL_DSS_RCM_DSP_PD_TRIGGER_SLEEP_DSP_PD_TRIGGER_SLEEP_SLEEP_TRIGGER_SHIFT (0x00000000U)
#define SDL_DSS_RCM_DSP_PD_TRIGGER_SLEEP_DSP_PD_TRIGGER_SLEEP_SLEEP_TRIGGER_RESETVAL (0x00000000U)
#define SDL_DSS_RCM_DSP_PD_TRIGGER_SLEEP_DSP_PD_TRIGGER_SLEEP_SLEEP_TRIGGER_MAX (0x00000001U)

#define SDL_DSS_RCM_DSP_PD_TRIGGER_SLEEP_RESETVAL                              (0x00000000U)

/* DSP_PD_STATUS */

#define SDL_DSS_RCM_DSP_PD_STATUS_DSP_PD_STATUS_PROC_HALTED_MASK               (0x00000001U)
#define SDL_DSS_RCM_DSP_PD_STATUS_DSP_PD_STATUS_PROC_HALTED_SHIFT              (0x00000000U)
#define SDL_DSS_RCM_DSP_PD_STATUS_DSP_PD_STATUS_PROC_HALTED_RESETVAL           (0x00000000U)
#define SDL_DSS_RCM_DSP_PD_STATUS_DSP_PD_STATUS_PROC_HALTED_MAX                (0x00000001U)

#define SDL_DSS_RCM_DSP_PD_STATUS_DSP_PD_STATUS_PD_STATUS_MASK                 (0x00000030U)
#define SDL_DSS_RCM_DSP_PD_STATUS_DSP_PD_STATUS_PD_STATUS_SHIFT                (0x00000004U)
#define SDL_DSS_RCM_DSP_PD_STATUS_DSP_PD_STATUS_PD_STATUS_RESETVAL             (0x00000000U)
#define SDL_DSS_RCM_DSP_PD_STATUS_DSP_PD_STATUS_PD_STATUS_MAX                  (0x00000003U)

#define SDL_DSS_RCM_DSP_PD_STATUS_DSP_PD_STATUS_PWRSM_DBG_OVRD_MASK            (0x00000100U)
#define SDL_DSS_RCM_DSP_PD_STATUS_DSP_PD_STATUS_PWRSM_DBG_OVRD_SHIFT           (0x00000008U)
#define SDL_DSS_RCM_DSP_PD_STATUS_DSP_PD_STATUS_PWRSM_DBG_OVRD_RESETVAL        (0x00000000U)
#define SDL_DSS_RCM_DSP_PD_STATUS_DSP_PD_STATUS_PWRSM_DBG_OVRD_MAX             (0x00000001U)

#define SDL_DSS_RCM_DSP_PD_STATUS_RESETVAL                                     (0x00000000U)

/* DSP_PD_CTRL_MISC0 */

#define SDL_DSS_RCM_DSP_PD_CTRL_MISC0_DSP_PD_CTRL_MISC0_PWRSM_PORRST_ASSERTCNT_MASK (0x0000003FU)
#define SDL_DSS_RCM_DSP_PD_CTRL_MISC0_DSP_PD_CTRL_MISC0_PWRSM_PORRST_ASSERTCNT_SHIFT (0x00000000U)
#define SDL_DSS_RCM_DSP_PD_CTRL_MISC0_DSP_PD_CTRL_MISC0_PWRSM_PORRST_ASSERTCNT_RESETVAL (0x00000014U)
#define SDL_DSS_RCM_DSP_PD_CTRL_MISC0_DSP_PD_CTRL_MISC0_PWRSM_PORRST_ASSERTCNT_MAX (0x0000003FU)

#define SDL_DSS_RCM_DSP_PD_CTRL_MISC0_DSP_PD_CTRL_MISC0_PWRSM_GRST_ASSERTCNT_MASK (0x00000FC0U)
#define SDL_DSS_RCM_DSP_PD_CTRL_MISC0_DSP_PD_CTRL_MISC0_PWRSM_GRST_ASSERTCNT_SHIFT (0x00000006U)
#define SDL_DSS_RCM_DSP_PD_CTRL_MISC0_DSP_PD_CTRL_MISC0_PWRSM_GRST_ASSERTCNT_RESETVAL (0x00000014U)
#define SDL_DSS_RCM_DSP_PD_CTRL_MISC0_DSP_PD_CTRL_MISC0_PWRSM_GRST_ASSERTCNT_MAX (0x0000003FU)

#define SDL_DSS_RCM_DSP_PD_CTRL_MISC0_DSP_PD_CTRL_MISC0_PWRSM_LRST_ASSERTCNT_MASK (0x0003F000U)
#define SDL_DSS_RCM_DSP_PD_CTRL_MISC0_DSP_PD_CTRL_MISC0_PWRSM_LRST_ASSERTCNT_SHIFT (0x0000000CU)
#define SDL_DSS_RCM_DSP_PD_CTRL_MISC0_DSP_PD_CTRL_MISC0_PWRSM_LRST_ASSERTCNT_RESETVAL (0x00000014U)
#define SDL_DSS_RCM_DSP_PD_CTRL_MISC0_DSP_PD_CTRL_MISC0_PWRSM_LRST_ASSERTCNT_MAX (0x0000003FU)

#define SDL_DSS_RCM_DSP_PD_CTRL_MISC0_DSP_PD_CTRL_MISC0_PWRSM_PORRST_DEASSERTCNT_MASK (0x00FC0000U)
#define SDL_DSS_RCM_DSP_PD_CTRL_MISC0_DSP_PD_CTRL_MISC0_PWRSM_PORRST_DEASSERTCNT_SHIFT (0x00000012U)
#define SDL_DSS_RCM_DSP_PD_CTRL_MISC0_DSP_PD_CTRL_MISC0_PWRSM_PORRST_DEASSERTCNT_RESETVAL (0x00000014U)
#define SDL_DSS_RCM_DSP_PD_CTRL_MISC0_DSP_PD_CTRL_MISC0_PWRSM_PORRST_DEASSERTCNT_MAX (0x0000003FU)

#define SDL_DSS_RCM_DSP_PD_CTRL_MISC0_DSP_PD_CTRL_MISC0_PWRSM_GRST_DEASSERTCNT_MASK (0x3F000000U)
#define SDL_DSS_RCM_DSP_PD_CTRL_MISC0_DSP_PD_CTRL_MISC0_PWRSM_GRST_DEASSERTCNT_SHIFT (0x00000018U)
#define SDL_DSS_RCM_DSP_PD_CTRL_MISC0_DSP_PD_CTRL_MISC0_PWRSM_GRST_DEASSERTCNT_RESETVAL (0x00000014U)
#define SDL_DSS_RCM_DSP_PD_CTRL_MISC0_DSP_PD_CTRL_MISC0_PWRSM_GRST_DEASSERTCNT_MAX (0x0000003FU)

#define SDL_DSS_RCM_DSP_PD_CTRL_MISC0_RESETVAL                                 (0x14514514U)

/* DSP_PD_CTRL_MISC1 */

#define SDL_DSS_RCM_DSP_PD_CTRL_MISC1_DSP_PD_CTRL_MISC1_PWRSM_LRST_DEASSERTCNT_MASK (0x0000003FU)
#define SDL_DSS_RCM_DSP_PD_CTRL_MISC1_DSP_PD_CTRL_MISC1_PWRSM_LRST_DEASSERTCNT_SHIFT (0x00000000U)
#define SDL_DSS_RCM_DSP_PD_CTRL_MISC1_DSP_PD_CTRL_MISC1_PWRSM_LRST_DEASSERTCNT_RESETVAL (0x00000014U)
#define SDL_DSS_RCM_DSP_PD_CTRL_MISC1_DSP_PD_CTRL_MISC1_PWRSM_LRST_DEASSERTCNT_MAX (0x0000003FU)

#define SDL_DSS_RCM_DSP_PD_CTRL_MISC1_DSP_PD_CTRL_MISC1_PWRSM_CLKSTOP_DEASSERTCNT_MASK (0x00000FC0U)
#define SDL_DSS_RCM_DSP_PD_CTRL_MISC1_DSP_PD_CTRL_MISC1_PWRSM_CLKSTOP_DEASSERTCNT_SHIFT (0x00000006U)
#define SDL_DSS_RCM_DSP_PD_CTRL_MISC1_DSP_PD_CTRL_MISC1_PWRSM_CLKSTOP_DEASSERTCNT_RESETVAL (0x00000014U)
#define SDL_DSS_RCM_DSP_PD_CTRL_MISC1_DSP_PD_CTRL_MISC1_PWRSM_CLKSTOP_DEASSERTCNT_MAX (0x0000003FU)

#define SDL_DSS_RCM_DSP_PD_CTRL_MISC1_DSP_PD_CTRL_MISC1_PWRSM_ISOEN_ASSERTCNT_MASK (0x0003F000U)
#define SDL_DSS_RCM_DSP_PD_CTRL_MISC1_DSP_PD_CTRL_MISC1_PWRSM_ISOEN_ASSERTCNT_SHIFT (0x0000000CU)
#define SDL_DSS_RCM_DSP_PD_CTRL_MISC1_DSP_PD_CTRL_MISC1_PWRSM_ISOEN_ASSERTCNT_RESETVAL (0x00000014U)
#define SDL_DSS_RCM_DSP_PD_CTRL_MISC1_DSP_PD_CTRL_MISC1_PWRSM_ISOEN_ASSERTCNT_MAX (0x0000003FU)

#define SDL_DSS_RCM_DSP_PD_CTRL_MISC1_DSP_PD_CTRL_MISC1_PWRSM_LRESETOUT_MASK_MASK (0x00040000U)
#define SDL_DSS_RCM_DSP_PD_CTRL_MISC1_DSP_PD_CTRL_MISC1_PWRSM_LRESETOUT_MASK_SHIFT (0x00000012U)
#define SDL_DSS_RCM_DSP_PD_CTRL_MISC1_DSP_PD_CTRL_MISC1_PWRSM_LRESETOUT_MASK_RESETVAL (0x00000000U)
#define SDL_DSS_RCM_DSP_PD_CTRL_MISC1_DSP_PD_CTRL_MISC1_PWRSM_LRESETOUT_MASK_MAX (0x00000001U)

#define SDL_DSS_RCM_DSP_PD_CTRL_MISC1_DSP_PD_CTRL_MISC1_RST_SYNC_BYPASS_MASK   (0x00700000U)
#define SDL_DSS_RCM_DSP_PD_CTRL_MISC1_DSP_PD_CTRL_MISC1_RST_SYNC_BYPASS_SHIFT  (0x00000014U)
#define SDL_DSS_RCM_DSP_PD_CTRL_MISC1_DSP_PD_CTRL_MISC1_RST_SYNC_BYPASS_RESETVAL (0x00000000U)
#define SDL_DSS_RCM_DSP_PD_CTRL_MISC1_DSP_PD_CTRL_MISC1_RST_SYNC_BYPASS_MAX    (0x00000007U)

#define SDL_DSS_RCM_DSP_PD_CTRL_MISC1_DSP_PD_CTRL_MISC1_ISO_SYNC_BYPASS_MASK   (0x07000000U)
#define SDL_DSS_RCM_DSP_PD_CTRL_MISC1_DSP_PD_CTRL_MISC1_ISO_SYNC_BYPASS_SHIFT  (0x00000018U)
#define SDL_DSS_RCM_DSP_PD_CTRL_MISC1_DSP_PD_CTRL_MISC1_ISO_SYNC_BYPASS_RESETVAL (0x00000000U)
#define SDL_DSS_RCM_DSP_PD_CTRL_MISC1_DSP_PD_CTRL_MISC1_ISO_SYNC_BYPASS_MAX    (0x00000007U)

#define SDL_DSS_RCM_DSP_PD_CTRL_MISC1_RESETVAL                                 (0x00014514U)

/* DSP_PD_STATUS_MISC0 */

#define SDL_DSS_RCM_DSP_PD_STATUS_MISC0_DSP_PD_STATUS_MISC0_STATE_MASK         (0x0000003FU)
#define SDL_DSS_RCM_DSP_PD_STATUS_MISC0_DSP_PD_STATUS_MISC0_STATE_SHIFT        (0x00000000U)
#define SDL_DSS_RCM_DSP_PD_STATUS_MISC0_DSP_PD_STATUS_MISC0_STATE_RESETVAL     (0x0000001FU)
#define SDL_DSS_RCM_DSP_PD_STATUS_MISC0_DSP_PD_STATUS_MISC0_STATE_MAX          (0x0000003FU)

#define SDL_DSS_RCM_DSP_PD_STATUS_MISC0_DSP_PD_STATUS_MISC0_PWRSM_PONOUT_MASK  (0x00000080U)
#define SDL_DSS_RCM_DSP_PD_STATUS_MISC0_DSP_PD_STATUS_MISC0_PWRSM_PONOUT_SHIFT (0x00000007U)
#define SDL_DSS_RCM_DSP_PD_STATUS_MISC0_DSP_PD_STATUS_MISC0_PWRSM_PONOUT_RESETVAL (0x00000000U)
#define SDL_DSS_RCM_DSP_PD_STATUS_MISC0_DSP_PD_STATUS_MISC0_PWRSM_PONOUT_MAX   (0x00000001U)

#define SDL_DSS_RCM_DSP_PD_STATUS_MISC0_DSP_PD_STATUS_MISC0_PWRSM_PGOODOUT_MASK (0x00000100U)
#define SDL_DSS_RCM_DSP_PD_STATUS_MISC0_DSP_PD_STATUS_MISC0_PWRSM_PGOODOUT_SHIFT (0x00000008U)
#define SDL_DSS_RCM_DSP_PD_STATUS_MISC0_DSP_PD_STATUS_MISC0_PWRSM_PGOODOUT_RESETVAL (0x00000000U)
#define SDL_DSS_RCM_DSP_PD_STATUS_MISC0_DSP_PD_STATUS_MISC0_PWRSM_PGOODOUT_MAX (0x00000001U)

#define SDL_DSS_RCM_DSP_PD_STATUS_MISC0_DSP_PD_STATUS_MISC0_PWRSM_MEM_PONOUT_MASK (0x00000200U)
#define SDL_DSS_RCM_DSP_PD_STATUS_MISC0_DSP_PD_STATUS_MISC0_PWRSM_MEM_PONOUT_SHIFT (0x00000009U)
#define SDL_DSS_RCM_DSP_PD_STATUS_MISC0_DSP_PD_STATUS_MISC0_PWRSM_MEM_PONOUT_RESETVAL (0x00000000U)
#define SDL_DSS_RCM_DSP_PD_STATUS_MISC0_DSP_PD_STATUS_MISC0_PWRSM_MEM_PONOUT_MAX (0x00000001U)

#define SDL_DSS_RCM_DSP_PD_STATUS_MISC0_DSP_PD_STATUS_MISC0_PWRSM_MEM_PGOODOUT_MASK (0x00000400U)
#define SDL_DSS_RCM_DSP_PD_STATUS_MISC0_DSP_PD_STATUS_MISC0_PWRSM_MEM_PGOODOUT_SHIFT (0x0000000AU)
#define SDL_DSS_RCM_DSP_PD_STATUS_MISC0_DSP_PD_STATUS_MISC0_PWRSM_MEM_PGOODOUT_RESETVAL (0x00000000U)
#define SDL_DSS_RCM_DSP_PD_STATUS_MISC0_DSP_PD_STATUS_MISC0_PWRSM_MEM_PGOODOUT_MAX (0x00000001U)

#define SDL_DSS_RCM_DSP_PD_STATUS_MISC0_DSP_PD_STATUS_MISC0_PWRSM_MEM_AONOUT_MASK (0x00000800U)
#define SDL_DSS_RCM_DSP_PD_STATUS_MISC0_DSP_PD_STATUS_MISC0_PWRSM_MEM_AONOUT_SHIFT (0x0000000BU)
#define SDL_DSS_RCM_DSP_PD_STATUS_MISC0_DSP_PD_STATUS_MISC0_PWRSM_MEM_AONOUT_RESETVAL (0x00000000U)
#define SDL_DSS_RCM_DSP_PD_STATUS_MISC0_DSP_PD_STATUS_MISC0_PWRSM_MEM_AONOUT_MAX (0x00000001U)

#define SDL_DSS_RCM_DSP_PD_STATUS_MISC0_DSP_PD_STATUS_MISC0_PWRSM_MEM_AGOODOUT_MASK (0x00001000U)
#define SDL_DSS_RCM_DSP_PD_STATUS_MISC0_DSP_PD_STATUS_MISC0_PWRSM_MEM_AGOODOUT_SHIFT (0x0000000CU)
#define SDL_DSS_RCM_DSP_PD_STATUS_MISC0_DSP_PD_STATUS_MISC0_PWRSM_MEM_AGOODOUT_RESETVAL (0x00000000U)
#define SDL_DSS_RCM_DSP_PD_STATUS_MISC0_DSP_PD_STATUS_MISC0_PWRSM_MEM_AGOODOUT_MAX (0x00000001U)

#define SDL_DSS_RCM_DSP_PD_STATUS_MISC0_DSP_PD_STATUS_MISC0_PWRSM_SDMA_SCR2ASYNC_CLKSTOP_REQ_MASK (0x00002000U)
#define SDL_DSS_RCM_DSP_PD_STATUS_MISC0_DSP_PD_STATUS_MISC0_PWRSM_SDMA_SCR2ASYNC_CLKSTOP_REQ_SHIFT (0x0000000DU)
#define SDL_DSS_RCM_DSP_PD_STATUS_MISC0_DSP_PD_STATUS_MISC0_PWRSM_SDMA_SCR2ASYNC_CLKSTOP_REQ_RESETVAL (0x00000001U)
#define SDL_DSS_RCM_DSP_PD_STATUS_MISC0_DSP_PD_STATUS_MISC0_PWRSM_SDMA_SCR2ASYNC_CLKSTOP_REQ_MAX (0x00000001U)

#define SDL_DSS_RCM_DSP_PD_STATUS_MISC0_DSP_PD_STATUS_MISC0_PWRSM_SDMA_ASYNC2RCM_CLKSTOP_REQ_MASK (0x00004000U)
#define SDL_DSS_RCM_DSP_PD_STATUS_MISC0_DSP_PD_STATUS_MISC0_PWRSM_SDMA_ASYNC2RCM_CLKSTOP_REQ_SHIFT (0x0000000EU)
#define SDL_DSS_RCM_DSP_PD_STATUS_MISC0_DSP_PD_STATUS_MISC0_PWRSM_SDMA_ASYNC2RCM_CLKSTOP_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_RCM_DSP_PD_STATUS_MISC0_DSP_PD_STATUS_MISC0_PWRSM_SDMA_ASYNC2RCM_CLKSTOP_REQ_MAX (0x00000001U)

#define SDL_DSS_RCM_DSP_PD_STATUS_MISC0_DSP_PD_STATUS_MISC0_PWRSM_SDMA_ASYNC2SCR_CLKSTOP_ACK_MASK (0x00008000U)
#define SDL_DSS_RCM_DSP_PD_STATUS_MISC0_DSP_PD_STATUS_MISC0_PWRSM_SDMA_ASYNC2SCR_CLKSTOP_ACK_SHIFT (0x0000000FU)
#define SDL_DSS_RCM_DSP_PD_STATUS_MISC0_DSP_PD_STATUS_MISC0_PWRSM_SDMA_ASYNC2SCR_CLKSTOP_ACK_RESETVAL (0x00000001U)
#define SDL_DSS_RCM_DSP_PD_STATUS_MISC0_DSP_PD_STATUS_MISC0_PWRSM_SDMA_ASYNC2SCR_CLKSTOP_ACK_MAX (0x00000001U)

#define SDL_DSS_RCM_DSP_PD_STATUS_MISC0_DSP_PD_STATUS_MISC0_PWRSM_C66_CLKSTOP_ACK_MASK (0x00010000U)
#define SDL_DSS_RCM_DSP_PD_STATUS_MISC0_DSP_PD_STATUS_MISC0_PWRSM_C66_CLKSTOP_ACK_SHIFT (0x00000010U)
#define SDL_DSS_RCM_DSP_PD_STATUS_MISC0_DSP_PD_STATUS_MISC0_PWRSM_C66_CLKSTOP_ACK_RESETVAL (0x00000001U)
#define SDL_DSS_RCM_DSP_PD_STATUS_MISC0_DSP_PD_STATUS_MISC0_PWRSM_C66_CLKSTOP_ACK_MAX (0x00000001U)

#define SDL_DSS_RCM_DSP_PD_STATUS_MISC0_DSP_PD_STATUS_MISC0_PWRSM_LRSTOUT_MASK (0x00020000U)
#define SDL_DSS_RCM_DSP_PD_STATUS_MISC0_DSP_PD_STATUS_MISC0_PWRSM_LRSTOUT_SHIFT (0x00000011U)
#define SDL_DSS_RCM_DSP_PD_STATUS_MISC0_DSP_PD_STATUS_MISC0_PWRSM_LRSTOUT_RESETVAL (0x00000001U)
#define SDL_DSS_RCM_DSP_PD_STATUS_MISC0_DSP_PD_STATUS_MISC0_PWRSM_LRSTOUT_MAX  (0x00000001U)

#define SDL_DSS_RCM_DSP_PD_STATUS_MISC0_RESETVAL                               (0x0003A01FU)

/* DSP_PD_WAKEUP_MASK0 */

#define SDL_DSS_RCM_DSP_PD_WAKEUP_MASK0_DSP_PD_WAKEUP_MASK0_WAKEUP_MASK0_MASK  (0xFFFFFFFFU)
#define SDL_DSS_RCM_DSP_PD_WAKEUP_MASK0_DSP_PD_WAKEUP_MASK0_WAKEUP_MASK0_SHIFT (0x00000000U)
#define SDL_DSS_RCM_DSP_PD_WAKEUP_MASK0_DSP_PD_WAKEUP_MASK0_WAKEUP_MASK0_RESETVAL (0xFFFFFFFFU)
#define SDL_DSS_RCM_DSP_PD_WAKEUP_MASK0_DSP_PD_WAKEUP_MASK0_WAKEUP_MASK0_MAX   (0xFFFFFFFFU)

#define SDL_DSS_RCM_DSP_PD_WAKEUP_MASK0_RESETVAL                               (0xFFFFFFFFU)

/* DSP_PD_WAKEUP_MASK1 */

#define SDL_DSS_RCM_DSP_PD_WAKEUP_MASK1_DSP_PD_WAKEUP_MASK1_WAKEUP_MASK1_MASK  (0xFFFFFFFFU)
#define SDL_DSS_RCM_DSP_PD_WAKEUP_MASK1_DSP_PD_WAKEUP_MASK1_WAKEUP_MASK1_SHIFT (0x00000000U)
#define SDL_DSS_RCM_DSP_PD_WAKEUP_MASK1_DSP_PD_WAKEUP_MASK1_WAKEUP_MASK1_RESETVAL (0xFFFFFFFFU)
#define SDL_DSS_RCM_DSP_PD_WAKEUP_MASK1_DSP_PD_WAKEUP_MASK1_WAKEUP_MASK1_MAX   (0xFFFFFFFFU)

#define SDL_DSS_RCM_DSP_PD_WAKEUP_MASK1_RESETVAL                               (0xFFFFFFFFU)

/* DSP_PD_WAKEUP_MASK2 */

#define SDL_DSS_RCM_DSP_PD_WAKEUP_MASK2_DSP_PD_WAKEUP_MASK2_WAKEUP_MASK2_MASK  (0xFFFFFFFFU)
#define SDL_DSS_RCM_DSP_PD_WAKEUP_MASK2_DSP_PD_WAKEUP_MASK2_WAKEUP_MASK2_SHIFT (0x00000000U)
#define SDL_DSS_RCM_DSP_PD_WAKEUP_MASK2_DSP_PD_WAKEUP_MASK2_WAKEUP_MASK2_RESETVAL (0xFFFFFFFFU)
#define SDL_DSS_RCM_DSP_PD_WAKEUP_MASK2_DSP_PD_WAKEUP_MASK2_WAKEUP_MASK2_MAX   (0xFFFFFFFFU)

#define SDL_DSS_RCM_DSP_PD_WAKEUP_MASK2_RESETVAL                               (0xFFFFFFFFU)

/* DSP_PD_WAKEUP_STATUS0 */

#define SDL_DSS_RCM_DSP_PD_WAKEUP_STATUS0_DSP_PD_WAKEUP_STATUS0_WAKEUP_STATUS0_MASK (0xFFFFFFFFU)
#define SDL_DSS_RCM_DSP_PD_WAKEUP_STATUS0_DSP_PD_WAKEUP_STATUS0_WAKEUP_STATUS0_SHIFT (0x00000000U)
#define SDL_DSS_RCM_DSP_PD_WAKEUP_STATUS0_DSP_PD_WAKEUP_STATUS0_WAKEUP_STATUS0_RESETVAL (0x00000000U)
#define SDL_DSS_RCM_DSP_PD_WAKEUP_STATUS0_DSP_PD_WAKEUP_STATUS0_WAKEUP_STATUS0_MAX (0xFFFFFFFFU)

#define SDL_DSS_RCM_DSP_PD_WAKEUP_STATUS0_RESETVAL                             (0x00000000U)

/* DSP_PD_WAKEUP_STATUS1 */

#define SDL_DSS_RCM_DSP_PD_WAKEUP_STATUS1_DSP_PD_WAKEUP_STATUS1_WAKEUP_STATUS1_MASK (0xFFFFFFFFU)
#define SDL_DSS_RCM_DSP_PD_WAKEUP_STATUS1_DSP_PD_WAKEUP_STATUS1_WAKEUP_STATUS1_SHIFT (0x00000000U)
#define SDL_DSS_RCM_DSP_PD_WAKEUP_STATUS1_DSP_PD_WAKEUP_STATUS1_WAKEUP_STATUS1_RESETVAL (0x00000000U)
#define SDL_DSS_RCM_DSP_PD_WAKEUP_STATUS1_DSP_PD_WAKEUP_STATUS1_WAKEUP_STATUS1_MAX (0xFFFFFFFFU)

#define SDL_DSS_RCM_DSP_PD_WAKEUP_STATUS1_RESETVAL                             (0x00000000U)

/* DSP_PD_WAKEUP_STATUS2 */

#define SDL_DSS_RCM_DSP_PD_WAKEUP_STATUS2_DSP_PD_WAKEUP_STATUS2_WAKEUP_STATUS2_MASK (0xFFFFFFFFU)
#define SDL_DSS_RCM_DSP_PD_WAKEUP_STATUS2_DSP_PD_WAKEUP_STATUS2_WAKEUP_STATUS2_SHIFT (0x00000000U)
#define SDL_DSS_RCM_DSP_PD_WAKEUP_STATUS2_DSP_PD_WAKEUP_STATUS2_WAKEUP_STATUS2_RESETVAL (0x00000000U)
#define SDL_DSS_RCM_DSP_PD_WAKEUP_STATUS2_DSP_PD_WAKEUP_STATUS2_WAKEUP_STATUS2_MAX (0xFFFFFFFFU)

#define SDL_DSS_RCM_DSP_PD_WAKEUP_STATUS2_RESETVAL                             (0x00000000U)

/* DSP_PD_WAKEUP_STATUS0_CLR */

#define SDL_DSS_RCM_DSP_PD_WAKEUP_STATUS0_CLR_DSP_PD_WAKEUP_STATUS0_CLR_WAKEUP_STATUS0_CLR_MASK (0xFFFFFFFFU)
#define SDL_DSS_RCM_DSP_PD_WAKEUP_STATUS0_CLR_DSP_PD_WAKEUP_STATUS0_CLR_WAKEUP_STATUS0_CLR_SHIFT (0x00000000U)
#define SDL_DSS_RCM_DSP_PD_WAKEUP_STATUS0_CLR_DSP_PD_WAKEUP_STATUS0_CLR_WAKEUP_STATUS0_CLR_RESETVAL (0x00000000U)
#define SDL_DSS_RCM_DSP_PD_WAKEUP_STATUS0_CLR_DSP_PD_WAKEUP_STATUS0_CLR_WAKEUP_STATUS0_CLR_MAX (0xFFFFFFFFU)

#define SDL_DSS_RCM_DSP_PD_WAKEUP_STATUS0_CLR_RESETVAL                         (0x00000000U)

/* DSP_PD_WAKEUP_STATUS1_CLR */

#define SDL_DSS_RCM_DSP_PD_WAKEUP_STATUS1_CLR_DSP_PD_WAKEUP_STATUS1_CLR_WAKEUP_STATUS1_CLR_MASK (0xFFFFFFFFU)
#define SDL_DSS_RCM_DSP_PD_WAKEUP_STATUS1_CLR_DSP_PD_WAKEUP_STATUS1_CLR_WAKEUP_STATUS1_CLR_SHIFT (0x00000000U)
#define SDL_DSS_RCM_DSP_PD_WAKEUP_STATUS1_CLR_DSP_PD_WAKEUP_STATUS1_CLR_WAKEUP_STATUS1_CLR_RESETVAL (0x00000000U)
#define SDL_DSS_RCM_DSP_PD_WAKEUP_STATUS1_CLR_DSP_PD_WAKEUP_STATUS1_CLR_WAKEUP_STATUS1_CLR_MAX (0xFFFFFFFFU)

#define SDL_DSS_RCM_DSP_PD_WAKEUP_STATUS1_CLR_RESETVAL                         (0x00000000U)

/* DSP_PD_WAKEUP_STATUS2_CLR */

#define SDL_DSS_RCM_DSP_PD_WAKEUP_STATUS2_CLR_DSP_PD_WAKEUP_STATUS2_CLR_WAKEUP_STATUS2_CLR_MASK (0xFFFFFFFFU)
#define SDL_DSS_RCM_DSP_PD_WAKEUP_STATUS2_CLR_DSP_PD_WAKEUP_STATUS2_CLR_WAKEUP_STATUS2_CLR_SHIFT (0x00000000U)
#define SDL_DSS_RCM_DSP_PD_WAKEUP_STATUS2_CLR_DSP_PD_WAKEUP_STATUS2_CLR_WAKEUP_STATUS2_CLR_RESETVAL (0x00000000U)
#define SDL_DSS_RCM_DSP_PD_WAKEUP_STATUS2_CLR_DSP_PD_WAKEUP_STATUS2_CLR_WAKEUP_STATUS2_CLR_MAX (0xFFFFFFFFU)

#define SDL_DSS_RCM_DSP_PD_WAKEUP_STATUS2_CLR_RESETVAL                         (0x00000000U)

/* DSP_PD_MISSED_EVENT_MASK0 */

#define SDL_DSS_RCM_DSP_PD_MISSED_EVENT_MASK0_DSP_PD_MISSED_EVENT_MASK0_MISSED_EVENT_MASK0_MASK (0xFFFFFFFFU)
#define SDL_DSS_RCM_DSP_PD_MISSED_EVENT_MASK0_DSP_PD_MISSED_EVENT_MASK0_MISSED_EVENT_MASK0_SHIFT (0x00000000U)
#define SDL_DSS_RCM_DSP_PD_MISSED_EVENT_MASK0_DSP_PD_MISSED_EVENT_MASK0_MISSED_EVENT_MASK0_RESETVAL (0xFFFFFFFFU)
#define SDL_DSS_RCM_DSP_PD_MISSED_EVENT_MASK0_DSP_PD_MISSED_EVENT_MASK0_MISSED_EVENT_MASK0_MAX (0xFFFFFFFFU)

#define SDL_DSS_RCM_DSP_PD_MISSED_EVENT_MASK0_RESETVAL                         (0xFFFFFFFFU)

/* DSP_PD_MISSED_EVENT_MASK1 */

#define SDL_DSS_RCM_DSP_PD_MISSED_EVENT_MASK1_DSP_PD_MISSED_EVENT_MASK1_MISSED_EVENT_MASK1_MASK (0xFFFFFFFFU)
#define SDL_DSS_RCM_DSP_PD_MISSED_EVENT_MASK1_DSP_PD_MISSED_EVENT_MASK1_MISSED_EVENT_MASK1_SHIFT (0x00000000U)
#define SDL_DSS_RCM_DSP_PD_MISSED_EVENT_MASK1_DSP_PD_MISSED_EVENT_MASK1_MISSED_EVENT_MASK1_RESETVAL (0xFFFFFFFFU)
#define SDL_DSS_RCM_DSP_PD_MISSED_EVENT_MASK1_DSP_PD_MISSED_EVENT_MASK1_MISSED_EVENT_MASK1_MAX (0xFFFFFFFFU)

#define SDL_DSS_RCM_DSP_PD_MISSED_EVENT_MASK1_RESETVAL                         (0xFFFFFFFFU)

/* DSP_PD_MISSED_EVENT_MASK2 */

#define SDL_DSS_RCM_DSP_PD_MISSED_EVENT_MASK2_DSP_PD_MISSED_EVENT_MASK2_MISSED_EVENT_MASK2_MASK (0xFFFFFFFFU)
#define SDL_DSS_RCM_DSP_PD_MISSED_EVENT_MASK2_DSP_PD_MISSED_EVENT_MASK2_MISSED_EVENT_MASK2_SHIFT (0x00000000U)
#define SDL_DSS_RCM_DSP_PD_MISSED_EVENT_MASK2_DSP_PD_MISSED_EVENT_MASK2_MISSED_EVENT_MASK2_RESETVAL (0xFFFFFFFFU)
#define SDL_DSS_RCM_DSP_PD_MISSED_EVENT_MASK2_DSP_PD_MISSED_EVENT_MASK2_MISSED_EVENT_MASK2_MAX (0xFFFFFFFFU)

#define SDL_DSS_RCM_DSP_PD_MISSED_EVENT_MASK2_RESETVAL                         (0xFFFFFFFFU)

/* DSP_PD_MISSED_EVENT_STATUS0 */

#define SDL_DSS_RCM_DSP_PD_MISSED_EVENT_STATUS0_DSP_PD_MISSED_EVENT_STATUS0_MISSED_EVENT_STATUS0_MASK (0xFFFFFFFFU)
#define SDL_DSS_RCM_DSP_PD_MISSED_EVENT_STATUS0_DSP_PD_MISSED_EVENT_STATUS0_MISSED_EVENT_STATUS0_SHIFT (0x00000000U)
#define SDL_DSS_RCM_DSP_PD_MISSED_EVENT_STATUS0_DSP_PD_MISSED_EVENT_STATUS0_MISSED_EVENT_STATUS0_RESETVAL (0x00000000U)
#define SDL_DSS_RCM_DSP_PD_MISSED_EVENT_STATUS0_DSP_PD_MISSED_EVENT_STATUS0_MISSED_EVENT_STATUS0_MAX (0xFFFFFFFFU)

#define SDL_DSS_RCM_DSP_PD_MISSED_EVENT_STATUS0_RESETVAL                       (0x00000000U)

/* DSP_PD_MISSED_EVENT_STATUS1 */

#define SDL_DSS_RCM_DSP_PD_MISSED_EVENT_STATUS1_DSP_PD_MISSED_EVENT_STATUS1_MISSED_EVENT_STATUS1_MASK (0xFFFFFFFFU)
#define SDL_DSS_RCM_DSP_PD_MISSED_EVENT_STATUS1_DSP_PD_MISSED_EVENT_STATUS1_MISSED_EVENT_STATUS1_SHIFT (0x00000000U)
#define SDL_DSS_RCM_DSP_PD_MISSED_EVENT_STATUS1_DSP_PD_MISSED_EVENT_STATUS1_MISSED_EVENT_STATUS1_RESETVAL (0x00000000U)
#define SDL_DSS_RCM_DSP_PD_MISSED_EVENT_STATUS1_DSP_PD_MISSED_EVENT_STATUS1_MISSED_EVENT_STATUS1_MAX (0xFFFFFFFFU)

#define SDL_DSS_RCM_DSP_PD_MISSED_EVENT_STATUS1_RESETVAL                       (0x00000000U)

/* DSP_PD_MISSED_EVENT_STATUS2 */

#define SDL_DSS_RCM_DSP_PD_MISSED_EVENT_STATUS2_DSP_PD_MISSED_EVENT_STATUS2_MISSED_EVENT_STATUS2_MASK (0xFFFFFFFFU)
#define SDL_DSS_RCM_DSP_PD_MISSED_EVENT_STATUS2_DSP_PD_MISSED_EVENT_STATUS2_MISSED_EVENT_STATUS2_SHIFT (0x00000000U)
#define SDL_DSS_RCM_DSP_PD_MISSED_EVENT_STATUS2_DSP_PD_MISSED_EVENT_STATUS2_MISSED_EVENT_STATUS2_RESETVAL (0x00000000U)
#define SDL_DSS_RCM_DSP_PD_MISSED_EVENT_STATUS2_DSP_PD_MISSED_EVENT_STATUS2_MISSED_EVENT_STATUS2_MAX (0xFFFFFFFFU)

#define SDL_DSS_RCM_DSP_PD_MISSED_EVENT_STATUS2_RESETVAL                       (0x00000000U)

/* DSP_RST_CAUSE */

#define SDL_DSS_RCM_DSP_RST_CAUSE_DSP_RST_CAUSE_LRST_CAUSE_MASK                (0x000000FFU)
#define SDL_DSS_RCM_DSP_RST_CAUSE_DSP_RST_CAUSE_LRST_CAUSE_SHIFT               (0x00000000U)
#define SDL_DSS_RCM_DSP_RST_CAUSE_DSP_RST_CAUSE_LRST_CAUSE_RESETVAL            (0x00000001U)
#define SDL_DSS_RCM_DSP_RST_CAUSE_DSP_RST_CAUSE_LRST_CAUSE_MAX                 (0x000000FFU)

#define SDL_DSS_RCM_DSP_RST_CAUSE_DSP_RST_CAUSE_GRST_CAUSE_MASK                (0x0000FF00U)
#define SDL_DSS_RCM_DSP_RST_CAUSE_DSP_RST_CAUSE_GRST_CAUSE_SHIFT               (0x00000008U)
#define SDL_DSS_RCM_DSP_RST_CAUSE_DSP_RST_CAUSE_GRST_CAUSE_RESETVAL            (0x00000001U)
#define SDL_DSS_RCM_DSP_RST_CAUSE_DSP_RST_CAUSE_GRST_CAUSE_MAX                 (0x000000FFU)

#define SDL_DSS_RCM_DSP_RST_CAUSE_DSP_RST_CAUSE_POR_CAUSE_MASK                 (0x00FF0000U)
#define SDL_DSS_RCM_DSP_RST_CAUSE_DSP_RST_CAUSE_POR_CAUSE_SHIFT                (0x00000010U)
#define SDL_DSS_RCM_DSP_RST_CAUSE_DSP_RST_CAUSE_POR_CAUSE_RESETVAL             (0x00000001U)
#define SDL_DSS_RCM_DSP_RST_CAUSE_DSP_RST_CAUSE_POR_CAUSE_MAX                  (0x000000FFU)

#define SDL_DSS_RCM_DSP_RST_CAUSE_RESETVAL                                     (0x00010101U)

/* DSP_RST_CAUSE_CLR */

#define SDL_DSS_RCM_DSP_RST_CAUSE_CLR_DSP_RST_CAUSE_CLR_CLEAR_MASK             (0x00000001U)
#define SDL_DSS_RCM_DSP_RST_CAUSE_CLR_DSP_RST_CAUSE_CLR_CLEAR_SHIFT            (0x00000000U)
#define SDL_DSS_RCM_DSP_RST_CAUSE_CLR_DSP_RST_CAUSE_CLR_CLEAR_RESETVAL         (0x00000000U)
#define SDL_DSS_RCM_DSP_RST_CAUSE_CLR_DSP_RST_CAUSE_CLR_CLEAR_MAX              (0x00000001U)

#define SDL_DSS_RCM_DSP_RST_CAUSE_CLR_RESETVAL                                 (0x00000000U)

/* DSP_STC_PBIST_CTRL */

#define SDL_DSS_RCM_DSP_STC_PBIST_CTRL_DSP_STC_PBIST_CTRL_MODE_ENABLE_MASK     (0x00000003U)
#define SDL_DSS_RCM_DSP_STC_PBIST_CTRL_DSP_STC_PBIST_CTRL_MODE_ENABLE_SHIFT    (0x00000000U)
#define SDL_DSS_RCM_DSP_STC_PBIST_CTRL_DSP_STC_PBIST_CTRL_MODE_ENABLE_RESETVAL (0x00000000U)
#define SDL_DSS_RCM_DSP_STC_PBIST_CTRL_DSP_STC_PBIST_CTRL_MODE_ENABLE_MAX      (0x00000003U)

#define SDL_DSS_RCM_DSP_STC_PBIST_CTRL_DSP_STC_PBIST_CTRL_STC_BOOT_EN_MASK     (0x00000004U)
#define SDL_DSS_RCM_DSP_STC_PBIST_CTRL_DSP_STC_PBIST_CTRL_STC_BOOT_EN_SHIFT    (0x00000002U)
#define SDL_DSS_RCM_DSP_STC_PBIST_CTRL_DSP_STC_PBIST_CTRL_STC_BOOT_EN_RESETVAL (0x00000000U)
#define SDL_DSS_RCM_DSP_STC_PBIST_CTRL_DSP_STC_PBIST_CTRL_STC_BOOT_EN_MAX      (0x00000001U)

#define SDL_DSS_RCM_DSP_STC_PBIST_CTRL_DSP_STC_PBIST_CTRL_PROC_HALT_MASK       (0x00000008U)
#define SDL_DSS_RCM_DSP_STC_PBIST_CTRL_DSP_STC_PBIST_CTRL_PROC_HALT_SHIFT      (0x00000003U)
#define SDL_DSS_RCM_DSP_STC_PBIST_CTRL_DSP_STC_PBIST_CTRL_PROC_HALT_RESETVAL   (0x00000001U)
#define SDL_DSS_RCM_DSP_STC_PBIST_CTRL_DSP_STC_PBIST_CTRL_PROC_HALT_MAX        (0x00000001U)

#define SDL_DSS_RCM_DSP_STC_PBIST_CTRL_DSP_STC_PBIST_CTRL_STC_CLK_STP_ACK_MASK_MASK (0x00000010U)
#define SDL_DSS_RCM_DSP_STC_PBIST_CTRL_DSP_STC_PBIST_CTRL_STC_CLK_STP_ACK_MASK_SHIFT (0x00000004U)
#define SDL_DSS_RCM_DSP_STC_PBIST_CTRL_DSP_STC_PBIST_CTRL_STC_CLK_STP_ACK_MASK_RESETVAL (0x00000001U)
#define SDL_DSS_RCM_DSP_STC_PBIST_CTRL_DSP_STC_PBIST_CTRL_STC_CLK_STP_ACK_MASK_MAX (0x00000001U)

#define SDL_DSS_RCM_DSP_STC_PBIST_CTRL_DSP_STC_PBIST_CTRL_STC_B2B_EN_MASK      (0x00000020U)
#define SDL_DSS_RCM_DSP_STC_PBIST_CTRL_DSP_STC_PBIST_CTRL_STC_B2B_EN_SHIFT     (0x00000005U)
#define SDL_DSS_RCM_DSP_STC_PBIST_CTRL_DSP_STC_PBIST_CTRL_STC_B2B_EN_RESETVAL  (0x00000000U)
#define SDL_DSS_RCM_DSP_STC_PBIST_CTRL_DSP_STC_PBIST_CTRL_STC_B2B_EN_MAX       (0x00000001U)

#define SDL_DSS_RCM_DSP_STC_PBIST_CTRL_DSP_STC_PBIST_CTRL_PBIST_SELFTEST_KEY_MASK (0x000003C0U)
#define SDL_DSS_RCM_DSP_STC_PBIST_CTRL_DSP_STC_PBIST_CTRL_PBIST_SELFTEST_KEY_SHIFT (0x00000006U)
#define SDL_DSS_RCM_DSP_STC_PBIST_CTRL_DSP_STC_PBIST_CTRL_PBIST_SELFTEST_KEY_RESETVAL (0x00000000U)
#define SDL_DSS_RCM_DSP_STC_PBIST_CTRL_DSP_STC_PBIST_CTRL_PBIST_SELFTEST_KEY_MAX (0x0000000FU)

#define SDL_DSS_RCM_DSP_STC_PBIST_CTRL_DSP_STC_PBIST_CTRL_PBIST_TMODE_VLCT_DEASSERTCNT_MASK (0x0000FC00U)
#define SDL_DSS_RCM_DSP_STC_PBIST_CTRL_DSP_STC_PBIST_CTRL_PBIST_TMODE_VLCT_DEASSERTCNT_SHIFT (0x0000000AU)
#define SDL_DSS_RCM_DSP_STC_PBIST_CTRL_DSP_STC_PBIST_CTRL_PBIST_TMODE_VLCT_DEASSERTCNT_RESETVAL (0x00000010U)
#define SDL_DSS_RCM_DSP_STC_PBIST_CTRL_DSP_STC_PBIST_CTRL_PBIST_TMODE_VLCT_DEASSERTCNT_MAX (0x0000003FU)

#define SDL_DSS_RCM_DSP_STC_PBIST_CTRL_DSP_STC_PBIST_CTRL_PBIST_TMODE_VLCT_ASSERTCNT_MASK (0x003F0000U)
#define SDL_DSS_RCM_DSP_STC_PBIST_CTRL_DSP_STC_PBIST_CTRL_PBIST_TMODE_VLCT_ASSERTCNT_SHIFT (0x00000010U)
#define SDL_DSS_RCM_DSP_STC_PBIST_CTRL_DSP_STC_PBIST_CTRL_PBIST_TMODE_VLCT_ASSERTCNT_RESETVAL (0x00000010U)
#define SDL_DSS_RCM_DSP_STC_PBIST_CTRL_DSP_STC_PBIST_CTRL_PBIST_TMODE_VLCT_ASSERTCNT_MAX (0x0000003FU)

#define SDL_DSS_RCM_DSP_STC_PBIST_CTRL_RESETVAL                                (0x00104018U)

/* DSP_STC_PBIST_STATUS */

#define SDL_DSS_RCM_DSP_STC_PBIST_STATUS_DSP_STC_PBIST_STATUS_PBIST_STATUS_MASK (0x00000003U)
#define SDL_DSS_RCM_DSP_STC_PBIST_STATUS_DSP_STC_PBIST_STATUS_PBIST_STATUS_SHIFT (0x00000000U)
#define SDL_DSS_RCM_DSP_STC_PBIST_STATUS_DSP_STC_PBIST_STATUS_PBIST_STATUS_RESETVAL (0x00000000U)
#define SDL_DSS_RCM_DSP_STC_PBIST_STATUS_DSP_STC_PBIST_STATUS_PBIST_STATUS_MAX (0x00000003U)

#define SDL_DSS_RCM_DSP_STC_PBIST_STATUS_DSP_STC_PBIST_STATUS_STC_PBIST_SM_STATUS_MASK (0x000000FCU)
#define SDL_DSS_RCM_DSP_STC_PBIST_STATUS_DSP_STC_PBIST_STATUS_STC_PBIST_SM_STATUS_SHIFT (0x00000002U)
#define SDL_DSS_RCM_DSP_STC_PBIST_STATUS_DSP_STC_PBIST_STATUS_STC_PBIST_SM_STATUS_RESETVAL (0x00000000U)
#define SDL_DSS_RCM_DSP_STC_PBIST_STATUS_DSP_STC_PBIST_STATUS_STC_PBIST_SM_STATUS_MAX (0x0000003FU)

#define SDL_DSS_RCM_DSP_STC_PBIST_STATUS_RESETVAL                              (0x00000000U)

/* DSP_STC_PBIST_CTRL_MISC0 */

#define SDL_DSS_RCM_DSP_STC_PBIST_CTRL_MISC0_DSP_STC_PBIST_CTRL_MISC0_BYP_EN_MASK (0x0000FFFFU)
#define SDL_DSS_RCM_DSP_STC_PBIST_CTRL_MISC0_DSP_STC_PBIST_CTRL_MISC0_BYP_EN_SHIFT (0x00000000U)
#define SDL_DSS_RCM_DSP_STC_PBIST_CTRL_MISC0_DSP_STC_PBIST_CTRL_MISC0_BYP_EN_RESETVAL (0x00000000U)
#define SDL_DSS_RCM_DSP_STC_PBIST_CTRL_MISC0_DSP_STC_PBIST_CTRL_MISC0_BYP_EN_MAX (0x0000FFFFU)

#define SDL_DSS_RCM_DSP_STC_PBIST_CTRL_MISC0_DSP_STC_PBIST_CTRL_MISC0_BYP_VALUE_MASK (0xFFFF0000U)
#define SDL_DSS_RCM_DSP_STC_PBIST_CTRL_MISC0_DSP_STC_PBIST_CTRL_MISC0_BYP_VALUE_SHIFT (0x00000010U)
#define SDL_DSS_RCM_DSP_STC_PBIST_CTRL_MISC0_DSP_STC_PBIST_CTRL_MISC0_BYP_VALUE_RESETVAL (0x00000000U)
#define SDL_DSS_RCM_DSP_STC_PBIST_CTRL_MISC0_DSP_STC_PBIST_CTRL_MISC0_BYP_VALUE_MAX (0x0000FFFFU)

#define SDL_DSS_RCM_DSP_STC_PBIST_CTRL_MISC0_RESETVAL                          (0x00000000U)

/* DSP_STC_PBIST_CTRL_MISC1 */

#define SDL_DSS_RCM_DSP_STC_PBIST_CTRL_MISC1_DSP_STC_PBIST_CTRL_MISC1_SM_OVR_EN_MASK (0x00000008U)
#define SDL_DSS_RCM_DSP_STC_PBIST_CTRL_MISC1_DSP_STC_PBIST_CTRL_MISC1_SM_OVR_EN_SHIFT (0x00000003U)
#define SDL_DSS_RCM_DSP_STC_PBIST_CTRL_MISC1_DSP_STC_PBIST_CTRL_MISC1_SM_OVR_EN_RESETVAL (0x00000000U)
#define SDL_DSS_RCM_DSP_STC_PBIST_CTRL_MISC1_DSP_STC_PBIST_CTRL_MISC1_SM_OVR_EN_MAX (0x00000001U)

#define SDL_DSS_RCM_DSP_STC_PBIST_CTRL_MISC1_DSP_STC_PBIST_CTRL_MISC1_SM_OVR_VAL_MASK (0x000003F0U)
#define SDL_DSS_RCM_DSP_STC_PBIST_CTRL_MISC1_DSP_STC_PBIST_CTRL_MISC1_SM_OVR_VAL_SHIFT (0x00000004U)
#define SDL_DSS_RCM_DSP_STC_PBIST_CTRL_MISC1_DSP_STC_PBIST_CTRL_MISC1_SM_OVR_VAL_RESETVAL (0x00000000U)
#define SDL_DSS_RCM_DSP_STC_PBIST_CTRL_MISC1_DSP_STC_PBIST_CTRL_MISC1_SM_OVR_VAL_MAX (0x0000003FU)

#define SDL_DSS_RCM_DSP_STC_PBIST_CTRL_MISC1_RESETVAL                          (0x00000000U)

/* DSP_STC_PBIST_START */

#define SDL_DSS_RCM_DSP_STC_PBIST_START_DSP_STC_PBIST_START_SM_TRIG_MASK       (0x00000001U)
#define SDL_DSS_RCM_DSP_STC_PBIST_START_DSP_STC_PBIST_START_SM_TRIG_SHIFT      (0x00000000U)
#define SDL_DSS_RCM_DSP_STC_PBIST_START_DSP_STC_PBIST_START_SM_TRIG_RESETVAL   (0x00000000U)
#define SDL_DSS_RCM_DSP_STC_PBIST_START_DSP_STC_PBIST_START_SM_TRIG_MAX        (0x00000001U)

#define SDL_DSS_RCM_DSP_STC_PBIST_START_RESETVAL                               (0x00000000U)

/* DSP_STC_PBIST_STATUS_CLR */

#define SDL_DSS_RCM_DSP_STC_PBIST_STATUS_CLR_DSP_STC_PBIST_STATUS_CLR_CLEAR_MASK (0x00000001U)
#define SDL_DSS_RCM_DSP_STC_PBIST_STATUS_CLR_DSP_STC_PBIST_STATUS_CLR_CLEAR_SHIFT (0x00000000U)
#define SDL_DSS_RCM_DSP_STC_PBIST_STATUS_CLR_DSP_STC_PBIST_STATUS_CLR_CLEAR_RESETVAL (0x00000000U)
#define SDL_DSS_RCM_DSP_STC_PBIST_STATUS_CLR_DSP_STC_PBIST_STATUS_CLR_CLEAR_MAX (0x00000001U)

#define SDL_DSS_RCM_DSP_STC_PBIST_STATUS_CLR_RESETVAL                          (0x00000000U)

/* DSS_DSP_CLK_SRC_SEL */

#define SDL_DSS_RCM_DSS_DSP_CLK_SRC_SEL_DSS_DSP_CLK_SRC_SEL_CLKSRCSEL_MASK     (0x00000FFFU)
#define SDL_DSS_RCM_DSS_DSP_CLK_SRC_SEL_DSS_DSP_CLK_SRC_SEL_CLKSRCSEL_SHIFT    (0x00000000U)
#define SDL_DSS_RCM_DSS_DSP_CLK_SRC_SEL_DSS_DSP_CLK_SRC_SEL_CLKSRCSEL_RESETVAL (0x00000000U)
#define SDL_DSS_RCM_DSS_DSP_CLK_SRC_SEL_DSS_DSP_CLK_SRC_SEL_CLKSRCSEL_MAX      (0x00000FFFU)

#define SDL_DSS_RCM_DSS_DSP_CLK_SRC_SEL_RESETVAL                               (0x00000000U)

/* DSS_HWA_CLK_SRC_SEL */

#define SDL_DSS_RCM_DSS_HWA_CLK_SRC_SEL_DSS_HWA_CLK_SRC_SEL_CLKSRCSEL_MASK     (0x00000007U)
#define SDL_DSS_RCM_DSS_HWA_CLK_SRC_SEL_DSS_HWA_CLK_SRC_SEL_CLKSRCSEL_SHIFT    (0x00000000U)
#define SDL_DSS_RCM_DSS_HWA_CLK_SRC_SEL_DSS_HWA_CLK_SRC_SEL_CLKSRCSEL_RESETVAL (0x00000000U)
#define SDL_DSS_RCM_DSS_HWA_CLK_SRC_SEL_DSS_HWA_CLK_SRC_SEL_CLKSRCSEL_MAX      (0x00000007U)

#define SDL_DSS_RCM_DSS_HWA_CLK_SRC_SEL_RESETVAL                               (0x00000000U)

/* DSS_RTIA_CLK_SRC_SEL */

#define SDL_DSS_RCM_DSS_RTIA_CLK_SRC_SEL_DSS_RTIA_CLK_SRC_SEL_CLKSRCSEL_MASK   (0x00000FFFU)
#define SDL_DSS_RCM_DSS_RTIA_CLK_SRC_SEL_DSS_RTIA_CLK_SRC_SEL_CLKSRCSEL_SHIFT  (0x00000000U)
#define SDL_DSS_RCM_DSS_RTIA_CLK_SRC_SEL_DSS_RTIA_CLK_SRC_SEL_CLKSRCSEL_RESETVAL (0x00000000U)
#define SDL_DSS_RCM_DSS_RTIA_CLK_SRC_SEL_DSS_RTIA_CLK_SRC_SEL_CLKSRCSEL_MAX    (0x00000FFFU)

#define SDL_DSS_RCM_DSS_RTIA_CLK_SRC_SEL_RESETVAL                              (0x00000000U)

/* DSS_RTIB_CLK_SRC_SEL */

#define SDL_DSS_RCM_DSS_RTIB_CLK_SRC_SEL_DSS_RTIB_CLK_SRC_SEL_CLKSRCSEL_MASK   (0x00000FFFU)
#define SDL_DSS_RCM_DSS_RTIB_CLK_SRC_SEL_DSS_RTIB_CLK_SRC_SEL_CLKSRCSEL_SHIFT  (0x00000000U)
#define SDL_DSS_RCM_DSS_RTIB_CLK_SRC_SEL_DSS_RTIB_CLK_SRC_SEL_CLKSRCSEL_RESETVAL (0x00000000U)
#define SDL_DSS_RCM_DSS_RTIB_CLK_SRC_SEL_DSS_RTIB_CLK_SRC_SEL_CLKSRCSEL_MAX    (0x00000FFFU)

#define SDL_DSS_RCM_DSS_RTIB_CLK_SRC_SEL_RESETVAL                              (0x00000000U)

/* DSS_WDT_CLK_SRC_SEL */

#define SDL_DSS_RCM_DSS_WDT_CLK_SRC_SEL_DSS_WDT_CLK_SRC_SEL_CLKSRCSEL_MASK     (0x00000FFFU)
#define SDL_DSS_RCM_DSS_WDT_CLK_SRC_SEL_DSS_WDT_CLK_SRC_SEL_CLKSRCSEL_SHIFT    (0x00000000U)
#define SDL_DSS_RCM_DSS_WDT_CLK_SRC_SEL_DSS_WDT_CLK_SRC_SEL_CLKSRCSEL_RESETVAL (0x00000000U)
#define SDL_DSS_RCM_DSS_WDT_CLK_SRC_SEL_DSS_WDT_CLK_SRC_SEL_CLKSRCSEL_MAX      (0x00000FFFU)

#define SDL_DSS_RCM_DSS_WDT_CLK_SRC_SEL_RESETVAL                               (0x00000000U)

/* DSS_SCIA_CLK_SRC_SEL */

#define SDL_DSS_RCM_DSS_SCIA_CLK_SRC_SEL_DSS_SCIA_CLK_SRC_SEL_CLKSRCSEL_MASK   (0x00000FFFU)
#define SDL_DSS_RCM_DSS_SCIA_CLK_SRC_SEL_DSS_SCIA_CLK_SRC_SEL_CLKSRCSEL_SHIFT  (0x00000000U)
#define SDL_DSS_RCM_DSS_SCIA_CLK_SRC_SEL_DSS_SCIA_CLK_SRC_SEL_CLKSRCSEL_RESETVAL (0x00000000U)
#define SDL_DSS_RCM_DSS_SCIA_CLK_SRC_SEL_DSS_SCIA_CLK_SRC_SEL_CLKSRCSEL_MAX    (0x00000FFFU)

#define SDL_DSS_RCM_DSS_SCIA_CLK_SRC_SEL_RESETVAL                              (0x00000000U)

/* DSS_DSP_CLK_DIV_VAL */

#define SDL_DSS_RCM_DSS_DSP_CLK_DIV_VAL_DSS_DSP_CLK_DIV_VAL_CLKDIV_MASK        (0x00000FFFU)
#define SDL_DSS_RCM_DSS_DSP_CLK_DIV_VAL_DSS_DSP_CLK_DIV_VAL_CLKDIV_SHIFT       (0x00000000U)
#define SDL_DSS_RCM_DSS_DSP_CLK_DIV_VAL_DSS_DSP_CLK_DIV_VAL_CLKDIV_RESETVAL    (0x00000000U)
#define SDL_DSS_RCM_DSS_DSP_CLK_DIV_VAL_DSS_DSP_CLK_DIV_VAL_CLKDIV_MAX         (0x00000FFFU)

#define SDL_DSS_RCM_DSS_DSP_CLK_DIV_VAL_RESETVAL                               (0x00000000U)

/* DSS_RTIA_CLK_DIV_VAL */

#define SDL_DSS_RCM_DSS_RTIA_CLK_DIV_VAL_DSS_RTIA_CLK_DIV_VAL_CLKDIV_MASK      (0x00000FFFU)
#define SDL_DSS_RCM_DSS_RTIA_CLK_DIV_VAL_DSS_RTIA_CLK_DIV_VAL_CLKDIV_SHIFT     (0x00000000U)
#define SDL_DSS_RCM_DSS_RTIA_CLK_DIV_VAL_DSS_RTIA_CLK_DIV_VAL_CLKDIV_RESETVAL  (0x00000000U)
#define SDL_DSS_RCM_DSS_RTIA_CLK_DIV_VAL_DSS_RTIA_CLK_DIV_VAL_CLKDIV_MAX       (0x00000FFFU)

#define SDL_DSS_RCM_DSS_RTIA_CLK_DIV_VAL_RESETVAL                              (0x00000000U)

/* DSS_RTIB_CLK_DIV_VAL */

#define SDL_DSS_RCM_DSS_RTIB_CLK_DIV_VAL_DSS_RTIB_CLK_DIV_VAL_CLKDIV_MASK      (0x00000FFFU)
#define SDL_DSS_RCM_DSS_RTIB_CLK_DIV_VAL_DSS_RTIB_CLK_DIV_VAL_CLKDIV_SHIFT     (0x00000000U)
#define SDL_DSS_RCM_DSS_RTIB_CLK_DIV_VAL_DSS_RTIB_CLK_DIV_VAL_CLKDIV_RESETVAL  (0x00000000U)
#define SDL_DSS_RCM_DSS_RTIB_CLK_DIV_VAL_DSS_RTIB_CLK_DIV_VAL_CLKDIV_MAX       (0x00000FFFU)

#define SDL_DSS_RCM_DSS_RTIB_CLK_DIV_VAL_RESETVAL                              (0x00000000U)

/* DSS_WDT_CLK_DIV_VAL */

#define SDL_DSS_RCM_DSS_WDT_CLK_DIV_VAL_DSS_WDT_CLK_DIV_VAL_CLKDIV_MASK        (0x00000FFFU)
#define SDL_DSS_RCM_DSS_WDT_CLK_DIV_VAL_DSS_WDT_CLK_DIV_VAL_CLKDIV_SHIFT       (0x00000000U)
#define SDL_DSS_RCM_DSS_WDT_CLK_DIV_VAL_DSS_WDT_CLK_DIV_VAL_CLKDIV_RESETVAL    (0x00000000U)
#define SDL_DSS_RCM_DSS_WDT_CLK_DIV_VAL_DSS_WDT_CLK_DIV_VAL_CLKDIV_MAX         (0x00000FFFU)

#define SDL_DSS_RCM_DSS_WDT_CLK_DIV_VAL_RESETVAL                               (0x00000000U)

/* DSS_SCIA_CLK_DIV_VAL */

#define SDL_DSS_RCM_DSS_SCIA_CLK_DIV_VAL_DSS_SCIA_CLK_DIV_VAL_CLKDIV_MASK      (0x00000FFFU)
#define SDL_DSS_RCM_DSS_SCIA_CLK_DIV_VAL_DSS_SCIA_CLK_DIV_VAL_CLKDIV_SHIFT     (0x00000000U)
#define SDL_DSS_RCM_DSS_SCIA_CLK_DIV_VAL_DSS_SCIA_CLK_DIV_VAL_CLKDIV_RESETVAL  (0x00000000U)
#define SDL_DSS_RCM_DSS_SCIA_CLK_DIV_VAL_DSS_SCIA_CLK_DIV_VAL_CLKDIV_MAX       (0x00000FFFU)

#define SDL_DSS_RCM_DSS_SCIA_CLK_DIV_VAL_RESETVAL                              (0x00000000U)

/* DSS_DSP_CLK_GATE */

#define SDL_DSS_RCM_DSS_DSP_CLK_GATE_DSS_DSP_CLK_GATE_GATED_MASK               (0x00000007U)
#define SDL_DSS_RCM_DSS_DSP_CLK_GATE_DSS_DSP_CLK_GATE_GATED_SHIFT              (0x00000000U)
#define SDL_DSS_RCM_DSS_DSP_CLK_GATE_DSS_DSP_CLK_GATE_GATED_RESETVAL           (0x00000000U)
#define SDL_DSS_RCM_DSS_DSP_CLK_GATE_DSS_DSP_CLK_GATE_GATED_MAX                (0x00000007U)

#define SDL_DSS_RCM_DSS_DSP_CLK_GATE_RESETVAL                                  (0x00000000U)

/* DSS_HWA_CLK_GATE */

#define SDL_DSS_RCM_DSS_HWA_CLK_GATE_DSS_HWA_CLK_GATE_GATED_MASK               (0x00000007U)
#define SDL_DSS_RCM_DSS_HWA_CLK_GATE_DSS_HWA_CLK_GATE_GATED_SHIFT              (0x00000000U)
#define SDL_DSS_RCM_DSS_HWA_CLK_GATE_DSS_HWA_CLK_GATE_GATED_RESETVAL           (0x00000000U)
#define SDL_DSS_RCM_DSS_HWA_CLK_GATE_DSS_HWA_CLK_GATE_GATED_MAX                (0x00000007U)

#define SDL_DSS_RCM_DSS_HWA_CLK_GATE_RESETVAL                                  (0x00000000U)

/* DSS_RTIA_CLK_GATE */

#define SDL_DSS_RCM_DSS_RTIA_CLK_GATE_DSS_RTIA_CLK_GATE_GATED_MASK             (0x00000007U)
#define SDL_DSS_RCM_DSS_RTIA_CLK_GATE_DSS_RTIA_CLK_GATE_GATED_SHIFT            (0x00000000U)
#define SDL_DSS_RCM_DSS_RTIA_CLK_GATE_DSS_RTIA_CLK_GATE_GATED_RESETVAL         (0x00000000U)
#define SDL_DSS_RCM_DSS_RTIA_CLK_GATE_DSS_RTIA_CLK_GATE_GATED_MAX              (0x00000007U)

#define SDL_DSS_RCM_DSS_RTIA_CLK_GATE_RESETVAL                                 (0x00000000U)

/* DSS_RTIB_CLK_GATE */

#define SDL_DSS_RCM_DSS_RTIB_CLK_GATE_DSS_RTIB_CLK_GATE_GATED_MASK             (0x00000007U)
#define SDL_DSS_RCM_DSS_RTIB_CLK_GATE_DSS_RTIB_CLK_GATE_GATED_SHIFT            (0x00000000U)
#define SDL_DSS_RCM_DSS_RTIB_CLK_GATE_DSS_RTIB_CLK_GATE_GATED_RESETVAL         (0x00000000U)
#define SDL_DSS_RCM_DSS_RTIB_CLK_GATE_DSS_RTIB_CLK_GATE_GATED_MAX              (0x00000007U)

#define SDL_DSS_RCM_DSS_RTIB_CLK_GATE_RESETVAL                                 (0x00000000U)

/* DSS_WDT_CLK_GATE */

#define SDL_DSS_RCM_DSS_WDT_CLK_GATE_DSS_WDT_CLK_GATE_GATED_MASK               (0x00000007U)
#define SDL_DSS_RCM_DSS_WDT_CLK_GATE_DSS_WDT_CLK_GATE_GATED_SHIFT              (0x00000000U)
#define SDL_DSS_RCM_DSS_WDT_CLK_GATE_DSS_WDT_CLK_GATE_GATED_RESETVAL           (0x00000000U)
#define SDL_DSS_RCM_DSS_WDT_CLK_GATE_DSS_WDT_CLK_GATE_GATED_MAX                (0x00000007U)

#define SDL_DSS_RCM_DSS_WDT_CLK_GATE_RESETVAL                                  (0x00000000U)

/* DSS_SCIA_CLK_GATE */

#define SDL_DSS_RCM_DSS_SCIA_CLK_GATE_DSS_SCIA_CLK_GATE_GATED_MASK             (0x00000007U)
#define SDL_DSS_RCM_DSS_SCIA_CLK_GATE_DSS_SCIA_CLK_GATE_GATED_SHIFT            (0x00000000U)
#define SDL_DSS_RCM_DSS_SCIA_CLK_GATE_DSS_SCIA_CLK_GATE_GATED_RESETVAL         (0x00000000U)
#define SDL_DSS_RCM_DSS_SCIA_CLK_GATE_DSS_SCIA_CLK_GATE_GATED_MAX              (0x00000007U)

#define SDL_DSS_RCM_DSS_SCIA_CLK_GATE_RESETVAL                                 (0x00000000U)

/* DSS_CBUFF_CLK_GATE */

#define SDL_DSS_RCM_DSS_CBUFF_CLK_GATE_DSS_CBUFF_CLK_GATE_GATED_MASK           (0x00000007U)
#define SDL_DSS_RCM_DSS_CBUFF_CLK_GATE_DSS_CBUFF_CLK_GATE_GATED_SHIFT          (0x00000000U)
#define SDL_DSS_RCM_DSS_CBUFF_CLK_GATE_DSS_CBUFF_CLK_GATE_GATED_RESETVAL       (0x00000000U)
#define SDL_DSS_RCM_DSS_CBUFF_CLK_GATE_DSS_CBUFF_CLK_GATE_GATED_MAX            (0x00000007U)

#define SDL_DSS_RCM_DSS_CBUFF_CLK_GATE_RESETVAL                                (0x00000000U)

/* DSS_DSP_CLK_STATUS */

#define SDL_DSS_RCM_DSS_DSP_CLK_STATUS_DSS_DSP_CLK_STATUS_CLKINUSE_MASK        (0x000000FFU)
#define SDL_DSS_RCM_DSS_DSP_CLK_STATUS_DSS_DSP_CLK_STATUS_CLKINUSE_SHIFT       (0x00000000U)
#define SDL_DSS_RCM_DSS_DSP_CLK_STATUS_DSS_DSP_CLK_STATUS_CLKINUSE_RESETVAL    (0x00000001U)
#define SDL_DSS_RCM_DSS_DSP_CLK_STATUS_DSS_DSP_CLK_STATUS_CLKINUSE_MAX         (0x000000FFU)

#define SDL_DSS_RCM_DSS_DSP_CLK_STATUS_DSS_DSP_CLK_STATUS_CURRDIVIDER_MASK     (0x0000FF00U)
#define SDL_DSS_RCM_DSS_DSP_CLK_STATUS_DSS_DSP_CLK_STATUS_CURRDIVIDER_SHIFT    (0x00000008U)
#define SDL_DSS_RCM_DSS_DSP_CLK_STATUS_DSS_DSP_CLK_STATUS_CURRDIVIDER_RESETVAL (0x00000000U)
#define SDL_DSS_RCM_DSS_DSP_CLK_STATUS_DSS_DSP_CLK_STATUS_CURRDIVIDER_MAX      (0x000000FFU)

#define SDL_DSS_RCM_DSS_DSP_CLK_STATUS_RESETVAL                                (0x00000001U)

/* DSS_HWA_CLK_STATUS */

#define SDL_DSS_RCM_DSS_HWA_CLK_STATUS_DSS_HWA_CLK_STATUS_CLKINUSE_MASK        (0x00000003U)
#define SDL_DSS_RCM_DSS_HWA_CLK_STATUS_DSS_HWA_CLK_STATUS_CLKINUSE_SHIFT       (0x00000000U)
#define SDL_DSS_RCM_DSS_HWA_CLK_STATUS_DSS_HWA_CLK_STATUS_CLKINUSE_RESETVAL    (0x00000001U)
#define SDL_DSS_RCM_DSS_HWA_CLK_STATUS_DSS_HWA_CLK_STATUS_CLKINUSE_MAX         (0x00000003U)

#define SDL_DSS_RCM_DSS_HWA_CLK_STATUS_RESETVAL                                (0x00000001U)

/* DSS_RTIA_CLK_STATUS */

#define SDL_DSS_RCM_DSS_RTIA_CLK_STATUS_DSS_RTIA_CLK_STATUS_CLKINUSE_MASK      (0x000000FFU)
#define SDL_DSS_RCM_DSS_RTIA_CLK_STATUS_DSS_RTIA_CLK_STATUS_CLKINUSE_SHIFT     (0x00000000U)
#define SDL_DSS_RCM_DSS_RTIA_CLK_STATUS_DSS_RTIA_CLK_STATUS_CLKINUSE_RESETVAL  (0x00000001U)
#define SDL_DSS_RCM_DSS_RTIA_CLK_STATUS_DSS_RTIA_CLK_STATUS_CLKINUSE_MAX       (0x000000FFU)

#define SDL_DSS_RCM_DSS_RTIA_CLK_STATUS_DSS_RTIA_CLK_STATUS_CURRDIVIDER_MASK   (0x0000FF00U)
#define SDL_DSS_RCM_DSS_RTIA_CLK_STATUS_DSS_RTIA_CLK_STATUS_CURRDIVIDER_SHIFT  (0x00000008U)
#define SDL_DSS_RCM_DSS_RTIA_CLK_STATUS_DSS_RTIA_CLK_STATUS_CURRDIVIDER_RESETVAL (0x00000000U)
#define SDL_DSS_RCM_DSS_RTIA_CLK_STATUS_DSS_RTIA_CLK_STATUS_CURRDIVIDER_MAX    (0x000000FFU)

#define SDL_DSS_RCM_DSS_RTIA_CLK_STATUS_RESETVAL                               (0x00000001U)

/* DSS_RTIB_CLK_STATUS */

#define SDL_DSS_RCM_DSS_RTIB_CLK_STATUS_DSS_RTIB_CLK_STATUS_CLKINUSE_MASK      (0x000000FFU)
#define SDL_DSS_RCM_DSS_RTIB_CLK_STATUS_DSS_RTIB_CLK_STATUS_CLKINUSE_SHIFT     (0x00000000U)
#define SDL_DSS_RCM_DSS_RTIB_CLK_STATUS_DSS_RTIB_CLK_STATUS_CLKINUSE_RESETVAL  (0x00000001U)
#define SDL_DSS_RCM_DSS_RTIB_CLK_STATUS_DSS_RTIB_CLK_STATUS_CLKINUSE_MAX       (0x000000FFU)

#define SDL_DSS_RCM_DSS_RTIB_CLK_STATUS_DSS_RTIB_CLK_STATUS_CURRDIVIDER_MASK   (0x0000FF00U)
#define SDL_DSS_RCM_DSS_RTIB_CLK_STATUS_DSS_RTIB_CLK_STATUS_CURRDIVIDER_SHIFT  (0x00000008U)
#define SDL_DSS_RCM_DSS_RTIB_CLK_STATUS_DSS_RTIB_CLK_STATUS_CURRDIVIDER_RESETVAL (0x00000000U)
#define SDL_DSS_RCM_DSS_RTIB_CLK_STATUS_DSS_RTIB_CLK_STATUS_CURRDIVIDER_MAX    (0x000000FFU)

#define SDL_DSS_RCM_DSS_RTIB_CLK_STATUS_RESETVAL                               (0x00000001U)

/* DSS_WDT_CLK_STATUS */

#define SDL_DSS_RCM_DSS_WDT_CLK_STATUS_DSS_WDT_CLK_STATUS_CLKINUSE_MASK        (0x000000FFU)
#define SDL_DSS_RCM_DSS_WDT_CLK_STATUS_DSS_WDT_CLK_STATUS_CLKINUSE_SHIFT       (0x00000000U)
#define SDL_DSS_RCM_DSS_WDT_CLK_STATUS_DSS_WDT_CLK_STATUS_CLKINUSE_RESETVAL    (0x00000001U)
#define SDL_DSS_RCM_DSS_WDT_CLK_STATUS_DSS_WDT_CLK_STATUS_CLKINUSE_MAX         (0x000000FFU)

#define SDL_DSS_RCM_DSS_WDT_CLK_STATUS_DSS_WDT_CLK_STATUS_CURRDIVIDER_MASK     (0x0000FF00U)
#define SDL_DSS_RCM_DSS_WDT_CLK_STATUS_DSS_WDT_CLK_STATUS_CURRDIVIDER_SHIFT    (0x00000008U)
#define SDL_DSS_RCM_DSS_WDT_CLK_STATUS_DSS_WDT_CLK_STATUS_CURRDIVIDER_RESETVAL (0x00000000U)
#define SDL_DSS_RCM_DSS_WDT_CLK_STATUS_DSS_WDT_CLK_STATUS_CURRDIVIDER_MAX      (0x000000FFU)

#define SDL_DSS_RCM_DSS_WDT_CLK_STATUS_RESETVAL                                (0x00000001U)

/* DSS_SCIA_CLK_STATUS */

#define SDL_DSS_RCM_DSS_SCIA_CLK_STATUS_DSS_SCIA_CLK_STATUS_CLKINUSE_MASK      (0x000000FFU)
#define SDL_DSS_RCM_DSS_SCIA_CLK_STATUS_DSS_SCIA_CLK_STATUS_CLKINUSE_SHIFT     (0x00000000U)
#define SDL_DSS_RCM_DSS_SCIA_CLK_STATUS_DSS_SCIA_CLK_STATUS_CLKINUSE_RESETVAL  (0x00000001U)
#define SDL_DSS_RCM_DSS_SCIA_CLK_STATUS_DSS_SCIA_CLK_STATUS_CLKINUSE_MAX       (0x000000FFU)

#define SDL_DSS_RCM_DSS_SCIA_CLK_STATUS_DSS_SCIA_CLK_STATUS_CURRDIVIDER_MASK   (0x0000FF00U)
#define SDL_DSS_RCM_DSS_SCIA_CLK_STATUS_DSS_SCIA_CLK_STATUS_CURRDIVIDER_SHIFT  (0x00000008U)
#define SDL_DSS_RCM_DSS_SCIA_CLK_STATUS_DSS_SCIA_CLK_STATUS_CURRDIVIDER_RESETVAL (0x00000000U)
#define SDL_DSS_RCM_DSS_SCIA_CLK_STATUS_DSS_SCIA_CLK_STATUS_CURRDIVIDER_MAX    (0x000000FFU)

#define SDL_DSS_RCM_DSS_SCIA_CLK_STATUS_RESETVAL                               (0x00000001U)

/* DSS_DSP_RST_CTRL */

#define SDL_DSS_RCM_DSS_DSP_RST_CTRL_DSS_DSP_RST_CTRL_ASSERT_POR_MASK          (0x00000007U)
#define SDL_DSS_RCM_DSS_DSP_RST_CTRL_DSS_DSP_RST_CTRL_ASSERT_POR_SHIFT         (0x00000000U)
#define SDL_DSS_RCM_DSS_DSP_RST_CTRL_DSS_DSP_RST_CTRL_ASSERT_POR_RESETVAL      (0x00000007U)
#define SDL_DSS_RCM_DSS_DSP_RST_CTRL_DSS_DSP_RST_CTRL_ASSERT_POR_MAX           (0x00000007U)

#define SDL_DSS_RCM_DSS_DSP_RST_CTRL_DSS_DSP_RST_CTRL_ASSERT_GLOBAL_MASK       (0x00000070U)
#define SDL_DSS_RCM_DSS_DSP_RST_CTRL_DSS_DSP_RST_CTRL_ASSERT_GLOBAL_SHIFT      (0x00000004U)
#define SDL_DSS_RCM_DSS_DSP_RST_CTRL_DSS_DSP_RST_CTRL_ASSERT_GLOBAL_RESETVAL   (0x00000007U)
#define SDL_DSS_RCM_DSS_DSP_RST_CTRL_DSS_DSP_RST_CTRL_ASSERT_GLOBAL_MAX        (0x00000007U)

#define SDL_DSS_RCM_DSS_DSP_RST_CTRL_DSS_DSP_RST_CTRL_ASSERT_LOCAL_MASK        (0x00000700U)
#define SDL_DSS_RCM_DSS_DSP_RST_CTRL_DSS_DSP_RST_CTRL_ASSERT_LOCAL_SHIFT       (0x00000008U)
#define SDL_DSS_RCM_DSS_DSP_RST_CTRL_DSS_DSP_RST_CTRL_ASSERT_LOCAL_RESETVAL    (0x00000007U)
#define SDL_DSS_RCM_DSS_DSP_RST_CTRL_DSS_DSP_RST_CTRL_ASSERT_LOCAL_MAX         (0x00000007U)

#define SDL_DSS_RCM_DSS_DSP_RST_CTRL_RESETVAL                                  (0x00000777U)

/* DSS_ESM_RST_CTRL */

#define SDL_DSS_RCM_DSS_ESM_RST_CTRL_DSS_ESM_RST_CTRL_ASSERT_MASK              (0x00000007U)
#define SDL_DSS_RCM_DSS_ESM_RST_CTRL_DSS_ESM_RST_CTRL_ASSERT_SHIFT             (0x00000000U)
#define SDL_DSS_RCM_DSS_ESM_RST_CTRL_DSS_ESM_RST_CTRL_ASSERT_RESETVAL          (0x00000000U)
#define SDL_DSS_RCM_DSS_ESM_RST_CTRL_DSS_ESM_RST_CTRL_ASSERT_MAX               (0x00000007U)

#define SDL_DSS_RCM_DSS_ESM_RST_CTRL_RESETVAL                                  (0x00000000U)

/* DSS_SCIA_RST_CTRL */

#define SDL_DSS_RCM_DSS_SCIA_RST_CTRL_DSS_SCIA_RST_CTRL_ASSERT_MASK            (0x00000007U)
#define SDL_DSS_RCM_DSS_SCIA_RST_CTRL_DSS_SCIA_RST_CTRL_ASSERT_SHIFT           (0x00000000U)
#define SDL_DSS_RCM_DSS_SCIA_RST_CTRL_DSS_SCIA_RST_CTRL_ASSERT_RESETVAL        (0x00000000U)
#define SDL_DSS_RCM_DSS_SCIA_RST_CTRL_DSS_SCIA_RST_CTRL_ASSERT_MAX             (0x00000007U)

#define SDL_DSS_RCM_DSS_SCIA_RST_CTRL_RESETVAL                                 (0x00000000U)

/* DSS_RTIA_RST_CTRL */

#define SDL_DSS_RCM_DSS_RTIA_RST_CTRL_DSS_RTIA_RST_CTRL_ASSERT_MASK            (0x00000007U)
#define SDL_DSS_RCM_DSS_RTIA_RST_CTRL_DSS_RTIA_RST_CTRL_ASSERT_SHIFT           (0x00000000U)
#define SDL_DSS_RCM_DSS_RTIA_RST_CTRL_DSS_RTIA_RST_CTRL_ASSERT_RESETVAL        (0x00000000U)
#define SDL_DSS_RCM_DSS_RTIA_RST_CTRL_DSS_RTIA_RST_CTRL_ASSERT_MAX             (0x00000007U)

#define SDL_DSS_RCM_DSS_RTIA_RST_CTRL_RESETVAL                                 (0x00000000U)

/* DSS_RTIB_RST_CTRL */

#define SDL_DSS_RCM_DSS_RTIB_RST_CTRL_DSS_RTIB_RST_CTRL_ASSERT_MASK            (0x00000007U)
#define SDL_DSS_RCM_DSS_RTIB_RST_CTRL_DSS_RTIB_RST_CTRL_ASSERT_SHIFT           (0x00000000U)
#define SDL_DSS_RCM_DSS_RTIB_RST_CTRL_DSS_RTIB_RST_CTRL_ASSERT_RESETVAL        (0x00000000U)
#define SDL_DSS_RCM_DSS_RTIB_RST_CTRL_DSS_RTIB_RST_CTRL_ASSERT_MAX             (0x00000007U)

#define SDL_DSS_RCM_DSS_RTIB_RST_CTRL_RESETVAL                                 (0x00000000U)

/* DSS_WDT_RST_CTRL */

#define SDL_DSS_RCM_DSS_WDT_RST_CTRL_DSS_WDT_RST_CTRL_ASSERT_MASK              (0x00000007U)
#define SDL_DSS_RCM_DSS_WDT_RST_CTRL_DSS_WDT_RST_CTRL_ASSERT_SHIFT             (0x00000000U)
#define SDL_DSS_RCM_DSS_WDT_RST_CTRL_DSS_WDT_RST_CTRL_ASSERT_RESETVAL          (0x00000000U)
#define SDL_DSS_RCM_DSS_WDT_RST_CTRL_DSS_WDT_RST_CTRL_ASSERT_MAX               (0x00000007U)

#define SDL_DSS_RCM_DSS_WDT_RST_CTRL_RESETVAL                                  (0x00000000U)

/* DSS_DCCA_RST_CTRL */

#define SDL_DSS_RCM_DSS_DCCA_RST_CTRL_DSS_DCCA_RST_CTRL_ASSERT_MASK            (0x00000007U)
#define SDL_DSS_RCM_DSS_DCCA_RST_CTRL_DSS_DCCA_RST_CTRL_ASSERT_SHIFT           (0x00000000U)
#define SDL_DSS_RCM_DSS_DCCA_RST_CTRL_DSS_DCCA_RST_CTRL_ASSERT_RESETVAL        (0x00000000U)
#define SDL_DSS_RCM_DSS_DCCA_RST_CTRL_DSS_DCCA_RST_CTRL_ASSERT_MAX             (0x00000007U)

#define SDL_DSS_RCM_DSS_DCCA_RST_CTRL_RESETVAL                                 (0x00000000U)

/* DSS_DCCB_RST_CTRL */

#define SDL_DSS_RCM_DSS_DCCB_RST_CTRL_DSS_DCCB_RST_CTRL_ASSERT_MASK            (0x00000007U)
#define SDL_DSS_RCM_DSS_DCCB_RST_CTRL_DSS_DCCB_RST_CTRL_ASSERT_SHIFT           (0x00000000U)
#define SDL_DSS_RCM_DSS_DCCB_RST_CTRL_DSS_DCCB_RST_CTRL_ASSERT_RESETVAL        (0x00000000U)
#define SDL_DSS_RCM_DSS_DCCB_RST_CTRL_DSS_DCCB_RST_CTRL_ASSERT_MAX             (0x00000007U)

#define SDL_DSS_RCM_DSS_DCCB_RST_CTRL_RESETVAL                                 (0x00000000U)

/* DSS_MCRC_RST_CTRL */

#define SDL_DSS_RCM_DSS_MCRC_RST_CTRL_DSS_MCRC_RST_CTRL_ASSERT_MASK            (0x00000007U)
#define SDL_DSS_RCM_DSS_MCRC_RST_CTRL_DSS_MCRC_RST_CTRL_ASSERT_SHIFT           (0x00000000U)
#define SDL_DSS_RCM_DSS_MCRC_RST_CTRL_DSS_MCRC_RST_CTRL_ASSERT_RESETVAL        (0x00000000U)
#define SDL_DSS_RCM_DSS_MCRC_RST_CTRL_DSS_MCRC_RST_CTRL_ASSERT_MAX             (0x00000007U)

#define SDL_DSS_RCM_DSS_MCRC_RST_CTRL_RESETVAL                                 (0x00000000U)

/* DSP_DFT_DIV_CTRL */

#define SDL_DSS_RCM_DSP_DFT_DIV_CTRL_DSP_DFT_DIV_CTRL_DIV_FACTOR_MASK          (0x0000000FU)
#define SDL_DSS_RCM_DSP_DFT_DIV_CTRL_DSP_DFT_DIV_CTRL_DIV_FACTOR_SHIFT         (0x00000000U)
#define SDL_DSS_RCM_DSP_DFT_DIV_CTRL_DSP_DFT_DIV_CTRL_DIV_FACTOR_RESETVAL      (0x00000003U)
#define SDL_DSS_RCM_DSP_DFT_DIV_CTRL_DSP_DFT_DIV_CTRL_DIV_FACTOR_MAX           (0x0000000FU)

#define SDL_DSS_RCM_DSP_DFT_DIV_CTRL_DSP_DFT_DIV_CTRL_CLK_DISABLE_MASK         (0x00000070U)
#define SDL_DSS_RCM_DSP_DFT_DIV_CTRL_DSP_DFT_DIV_CTRL_CLK_DISABLE_SHIFT        (0x00000004U)
#define SDL_DSS_RCM_DSP_DFT_DIV_CTRL_DSP_DFT_DIV_CTRL_CLK_DISABLE_RESETVAL     (0x00000000U)
#define SDL_DSS_RCM_DSP_DFT_DIV_CTRL_DSP_DFT_DIV_CTRL_CLK_DISABLE_MAX          (0x00000007U)

#define SDL_DSS_RCM_DSP_DFT_DIV_CTRL_RESETVAL                                  (0x00000003U)

/* DSS_DSP_L2_PD_CTRL */

#define SDL_DSS_RCM_DSS_DSP_L2_PD_CTRL_DSS_DSP_L2_PD_CTRL_ISO_MASK             (0x00000007U)
#define SDL_DSS_RCM_DSS_DSP_L2_PD_CTRL_DSS_DSP_L2_PD_CTRL_ISO_SHIFT            (0x00000000U)
#define SDL_DSS_RCM_DSS_DSP_L2_PD_CTRL_DSS_DSP_L2_PD_CTRL_ISO_RESETVAL         (0x00000000U)
#define SDL_DSS_RCM_DSS_DSP_L2_PD_CTRL_DSS_DSP_L2_PD_CTRL_ISO_MAX              (0x00000007U)

#define SDL_DSS_RCM_DSS_DSP_L2_PD_CTRL_DSS_DSP_L2_PD_CTRL_AONIN_MASK           (0x00000070U)
#define SDL_DSS_RCM_DSS_DSP_L2_PD_CTRL_DSS_DSP_L2_PD_CTRL_AONIN_SHIFT          (0x00000004U)
#define SDL_DSS_RCM_DSS_DSP_L2_PD_CTRL_DSS_DSP_L2_PD_CTRL_AONIN_RESETVAL       (0x00000007U)
#define SDL_DSS_RCM_DSS_DSP_L2_PD_CTRL_DSS_DSP_L2_PD_CTRL_AONIN_MAX            (0x00000007U)

#define SDL_DSS_RCM_DSS_DSP_L2_PD_CTRL_DSS_DSP_L2_PD_CTRL_AGOODIN_MASK         (0x00000700U)
#define SDL_DSS_RCM_DSS_DSP_L2_PD_CTRL_DSS_DSP_L2_PD_CTRL_AGOODIN_SHIFT        (0x00000008U)
#define SDL_DSS_RCM_DSS_DSP_L2_PD_CTRL_DSS_DSP_L2_PD_CTRL_AGOODIN_RESETVAL     (0x00000007U)
#define SDL_DSS_RCM_DSS_DSP_L2_PD_CTRL_DSS_DSP_L2_PD_CTRL_AGOODIN_MAX          (0x00000007U)

#define SDL_DSS_RCM_DSS_DSP_L2_PD_CTRL_RESETVAL                                (0x00000770U)

/* DSS_L3_BANKA0_PD_CTRL */

#define SDL_DSS_RCM_DSS_L3_BANKA0_PD_CTRL_DSS_L3_BANKA0_PD_CTRL_ISO_MASK       (0x00000007U)
#define SDL_DSS_RCM_DSS_L3_BANKA0_PD_CTRL_DSS_L3_BANKA0_PD_CTRL_ISO_SHIFT      (0x00000000U)
#define SDL_DSS_RCM_DSS_L3_BANKA0_PD_CTRL_DSS_L3_BANKA0_PD_CTRL_ISO_RESETVAL   (0x00000000U)
#define SDL_DSS_RCM_DSS_L3_BANKA0_PD_CTRL_DSS_L3_BANKA0_PD_CTRL_ISO_MAX        (0x00000007U)

#define SDL_DSS_RCM_DSS_L3_BANKA0_PD_CTRL_DSS_L3_BANKA0_PD_CTRL_AONIN_MASK     (0x00000070U)
#define SDL_DSS_RCM_DSS_L3_BANKA0_PD_CTRL_DSS_L3_BANKA0_PD_CTRL_AONIN_SHIFT    (0x00000004U)
#define SDL_DSS_RCM_DSS_L3_BANKA0_PD_CTRL_DSS_L3_BANKA0_PD_CTRL_AONIN_RESETVAL (0x00000007U)
#define SDL_DSS_RCM_DSS_L3_BANKA0_PD_CTRL_DSS_L3_BANKA0_PD_CTRL_AONIN_MAX      (0x00000007U)

#define SDL_DSS_RCM_DSS_L3_BANKA0_PD_CTRL_DSS_L3_BANKA0_PD_CTRL_AGOODIN_MASK   (0x00000700U)
#define SDL_DSS_RCM_DSS_L3_BANKA0_PD_CTRL_DSS_L3_BANKA0_PD_CTRL_AGOODIN_SHIFT  (0x00000008U)
#define SDL_DSS_RCM_DSS_L3_BANKA0_PD_CTRL_DSS_L3_BANKA0_PD_CTRL_AGOODIN_RESETVAL (0x00000007U)
#define SDL_DSS_RCM_DSS_L3_BANKA0_PD_CTRL_DSS_L3_BANKA0_PD_CTRL_AGOODIN_MAX    (0x00000007U)

#define SDL_DSS_RCM_DSS_L3_BANKA0_PD_CTRL_RESETVAL                             (0x00000770U)

/* DSS_L3_BANKA1_PD_CTRL */

#define SDL_DSS_RCM_DSS_L3_BANKA1_PD_CTRL_DSS_L3_BANKA1_PD_CTRL_ISO_MASK       (0x00000007U)
#define SDL_DSS_RCM_DSS_L3_BANKA1_PD_CTRL_DSS_L3_BANKA1_PD_CTRL_ISO_SHIFT      (0x00000000U)
#define SDL_DSS_RCM_DSS_L3_BANKA1_PD_CTRL_DSS_L3_BANKA1_PD_CTRL_ISO_RESETVAL   (0x00000000U)
#define SDL_DSS_RCM_DSS_L3_BANKA1_PD_CTRL_DSS_L3_BANKA1_PD_CTRL_ISO_MAX        (0x00000007U)

#define SDL_DSS_RCM_DSS_L3_BANKA1_PD_CTRL_DSS_L3_BANKA1_PD_CTRL_AONIN_MASK     (0x00000070U)
#define SDL_DSS_RCM_DSS_L3_BANKA1_PD_CTRL_DSS_L3_BANKA1_PD_CTRL_AONIN_SHIFT    (0x00000004U)
#define SDL_DSS_RCM_DSS_L3_BANKA1_PD_CTRL_DSS_L3_BANKA1_PD_CTRL_AONIN_RESETVAL (0x00000007U)
#define SDL_DSS_RCM_DSS_L3_BANKA1_PD_CTRL_DSS_L3_BANKA1_PD_CTRL_AONIN_MAX      (0x00000007U)

#define SDL_DSS_RCM_DSS_L3_BANKA1_PD_CTRL_DSS_L3_BANKA1_PD_CTRL_AGOODIN_MASK   (0x00000700U)
#define SDL_DSS_RCM_DSS_L3_BANKA1_PD_CTRL_DSS_L3_BANKA1_PD_CTRL_AGOODIN_SHIFT  (0x00000008U)
#define SDL_DSS_RCM_DSS_L3_BANKA1_PD_CTRL_DSS_L3_BANKA1_PD_CTRL_AGOODIN_RESETVAL (0x00000007U)
#define SDL_DSS_RCM_DSS_L3_BANKA1_PD_CTRL_DSS_L3_BANKA1_PD_CTRL_AGOODIN_MAX    (0x00000007U)

#define SDL_DSS_RCM_DSS_L3_BANKA1_PD_CTRL_RESETVAL                             (0x00000770U)

/* DSS_L3_BANKA2_PD_CTRL */

#define SDL_DSS_RCM_DSS_L3_BANKA2_PD_CTRL_DSS_L3_BANKA2_PD_CTRL_ISO_MASK       (0x00000007U)
#define SDL_DSS_RCM_DSS_L3_BANKA2_PD_CTRL_DSS_L3_BANKA2_PD_CTRL_ISO_SHIFT      (0x00000000U)
#define SDL_DSS_RCM_DSS_L3_BANKA2_PD_CTRL_DSS_L3_BANKA2_PD_CTRL_ISO_RESETVAL   (0x00000000U)
#define SDL_DSS_RCM_DSS_L3_BANKA2_PD_CTRL_DSS_L3_BANKA2_PD_CTRL_ISO_MAX        (0x00000007U)

#define SDL_DSS_RCM_DSS_L3_BANKA2_PD_CTRL_DSS_L3_BANKA2_PD_CTRL_AONIN_MASK     (0x00000070U)
#define SDL_DSS_RCM_DSS_L3_BANKA2_PD_CTRL_DSS_L3_BANKA2_PD_CTRL_AONIN_SHIFT    (0x00000004U)
#define SDL_DSS_RCM_DSS_L3_BANKA2_PD_CTRL_DSS_L3_BANKA2_PD_CTRL_AONIN_RESETVAL (0x00000007U)
#define SDL_DSS_RCM_DSS_L3_BANKA2_PD_CTRL_DSS_L3_BANKA2_PD_CTRL_AONIN_MAX      (0x00000007U)

#define SDL_DSS_RCM_DSS_L3_BANKA2_PD_CTRL_DSS_L3_BANKA2_PD_CTRL_AGOODIN_MASK   (0x00000700U)
#define SDL_DSS_RCM_DSS_L3_BANKA2_PD_CTRL_DSS_L3_BANKA2_PD_CTRL_AGOODIN_SHIFT  (0x00000008U)
#define SDL_DSS_RCM_DSS_L3_BANKA2_PD_CTRL_DSS_L3_BANKA2_PD_CTRL_AGOODIN_RESETVAL (0x00000007U)
#define SDL_DSS_RCM_DSS_L3_BANKA2_PD_CTRL_DSS_L3_BANKA2_PD_CTRL_AGOODIN_MAX    (0x00000007U)

#define SDL_DSS_RCM_DSS_L3_BANKA2_PD_CTRL_RESETVAL                             (0x00000770U)

/* DSS_L3_BANKA3_PD_CTRL */

#define SDL_DSS_RCM_DSS_L3_BANKA3_PD_CTRL_DSS_L3_BANKA3_PD_CTRL_ISO_MASK       (0x00000007U)
#define SDL_DSS_RCM_DSS_L3_BANKA3_PD_CTRL_DSS_L3_BANKA3_PD_CTRL_ISO_SHIFT      (0x00000000U)
#define SDL_DSS_RCM_DSS_L3_BANKA3_PD_CTRL_DSS_L3_BANKA3_PD_CTRL_ISO_RESETVAL   (0x00000000U)
#define SDL_DSS_RCM_DSS_L3_BANKA3_PD_CTRL_DSS_L3_BANKA3_PD_CTRL_ISO_MAX        (0x00000007U)

#define SDL_DSS_RCM_DSS_L3_BANKA3_PD_CTRL_DSS_L3_BANKA3_PD_CTRL_AONIN_MASK     (0x00000070U)
#define SDL_DSS_RCM_DSS_L3_BANKA3_PD_CTRL_DSS_L3_BANKA3_PD_CTRL_AONIN_SHIFT    (0x00000004U)
#define SDL_DSS_RCM_DSS_L3_BANKA3_PD_CTRL_DSS_L3_BANKA3_PD_CTRL_AONIN_RESETVAL (0x00000007U)
#define SDL_DSS_RCM_DSS_L3_BANKA3_PD_CTRL_DSS_L3_BANKA3_PD_CTRL_AONIN_MAX      (0x00000007U)

#define SDL_DSS_RCM_DSS_L3_BANKA3_PD_CTRL_DSS_L3_BANKA3_PD_CTRL_AGOODIN_MASK   (0x00000700U)
#define SDL_DSS_RCM_DSS_L3_BANKA3_PD_CTRL_DSS_L3_BANKA3_PD_CTRL_AGOODIN_SHIFT  (0x00000008U)
#define SDL_DSS_RCM_DSS_L3_BANKA3_PD_CTRL_DSS_L3_BANKA3_PD_CTRL_AGOODIN_RESETVAL (0x00000007U)
#define SDL_DSS_RCM_DSS_L3_BANKA3_PD_CTRL_DSS_L3_BANKA3_PD_CTRL_AGOODIN_MAX    (0x00000007U)

#define SDL_DSS_RCM_DSS_L3_BANKA3_PD_CTRL_RESETVAL                             (0x00000770U)

/* DSS_L3_BANKB0_PD_CTRL */

#define SDL_DSS_RCM_DSS_L3_BANKB0_PD_CTRL_DSS_L3_BANKB0_PD_CTRL_ISO_MASK       (0x00000007U)
#define SDL_DSS_RCM_DSS_L3_BANKB0_PD_CTRL_DSS_L3_BANKB0_PD_CTRL_ISO_SHIFT      (0x00000000U)
#define SDL_DSS_RCM_DSS_L3_BANKB0_PD_CTRL_DSS_L3_BANKB0_PD_CTRL_ISO_RESETVAL   (0x00000000U)
#define SDL_DSS_RCM_DSS_L3_BANKB0_PD_CTRL_DSS_L3_BANKB0_PD_CTRL_ISO_MAX        (0x00000007U)

#define SDL_DSS_RCM_DSS_L3_BANKB0_PD_CTRL_DSS_L3_BANKB0_PD_CTRL_AONIN_MASK     (0x00000070U)
#define SDL_DSS_RCM_DSS_L3_BANKB0_PD_CTRL_DSS_L3_BANKB0_PD_CTRL_AONIN_SHIFT    (0x00000004U)
#define SDL_DSS_RCM_DSS_L3_BANKB0_PD_CTRL_DSS_L3_BANKB0_PD_CTRL_AONIN_RESETVAL (0x00000007U)
#define SDL_DSS_RCM_DSS_L3_BANKB0_PD_CTRL_DSS_L3_BANKB0_PD_CTRL_AONIN_MAX      (0x00000007U)

#define SDL_DSS_RCM_DSS_L3_BANKB0_PD_CTRL_DSS_L3_BANKB0_PD_CTRL_AGOODIN_MASK   (0x00000700U)
#define SDL_DSS_RCM_DSS_L3_BANKB0_PD_CTRL_DSS_L3_BANKB0_PD_CTRL_AGOODIN_SHIFT  (0x00000008U)
#define SDL_DSS_RCM_DSS_L3_BANKB0_PD_CTRL_DSS_L3_BANKB0_PD_CTRL_AGOODIN_RESETVAL (0x00000007U)
#define SDL_DSS_RCM_DSS_L3_BANKB0_PD_CTRL_DSS_L3_BANKB0_PD_CTRL_AGOODIN_MAX    (0x00000007U)

#define SDL_DSS_RCM_DSS_L3_BANKB0_PD_CTRL_RESETVAL                             (0x00000770U)

/* DSS_L3_BANKB1_PD_CTRL */

#define SDL_DSS_RCM_DSS_L3_BANKB1_PD_CTRL_DSS_L3_BANKB1_PD_CTRL_ISO_MASK       (0x00000007U)
#define SDL_DSS_RCM_DSS_L3_BANKB1_PD_CTRL_DSS_L3_BANKB1_PD_CTRL_ISO_SHIFT      (0x00000000U)
#define SDL_DSS_RCM_DSS_L3_BANKB1_PD_CTRL_DSS_L3_BANKB1_PD_CTRL_ISO_RESETVAL   (0x00000000U)
#define SDL_DSS_RCM_DSS_L3_BANKB1_PD_CTRL_DSS_L3_BANKB1_PD_CTRL_ISO_MAX        (0x00000007U)

#define SDL_DSS_RCM_DSS_L3_BANKB1_PD_CTRL_DSS_L3_BANKB1_PD_CTRL_AONIN_MASK     (0x00000070U)
#define SDL_DSS_RCM_DSS_L3_BANKB1_PD_CTRL_DSS_L3_BANKB1_PD_CTRL_AONIN_SHIFT    (0x00000004U)
#define SDL_DSS_RCM_DSS_L3_BANKB1_PD_CTRL_DSS_L3_BANKB1_PD_CTRL_AONIN_RESETVAL (0x00000007U)
#define SDL_DSS_RCM_DSS_L3_BANKB1_PD_CTRL_DSS_L3_BANKB1_PD_CTRL_AONIN_MAX      (0x00000007U)

#define SDL_DSS_RCM_DSS_L3_BANKB1_PD_CTRL_DSS_L3_BANKB1_PD_CTRL_AGOODIN_MASK   (0x00000700U)
#define SDL_DSS_RCM_DSS_L3_BANKB1_PD_CTRL_DSS_L3_BANKB1_PD_CTRL_AGOODIN_SHIFT  (0x00000008U)
#define SDL_DSS_RCM_DSS_L3_BANKB1_PD_CTRL_DSS_L3_BANKB1_PD_CTRL_AGOODIN_RESETVAL (0x00000007U)
#define SDL_DSS_RCM_DSS_L3_BANKB1_PD_CTRL_DSS_L3_BANKB1_PD_CTRL_AGOODIN_MAX    (0x00000007U)

#define SDL_DSS_RCM_DSS_L3_BANKB1_PD_CTRL_RESETVAL                             (0x00000770U)

/* DSS_L3_BANKB2_PD_CTRL */

#define SDL_DSS_RCM_DSS_L3_BANKB2_PD_CTRL_DSS_L3_BANKB2_PD_CTRL_ISO_MASK       (0x00000007U)
#define SDL_DSS_RCM_DSS_L3_BANKB2_PD_CTRL_DSS_L3_BANKB2_PD_CTRL_ISO_SHIFT      (0x00000000U)
#define SDL_DSS_RCM_DSS_L3_BANKB2_PD_CTRL_DSS_L3_BANKB2_PD_CTRL_ISO_RESETVAL   (0x00000000U)
#define SDL_DSS_RCM_DSS_L3_BANKB2_PD_CTRL_DSS_L3_BANKB2_PD_CTRL_ISO_MAX        (0x00000007U)

#define SDL_DSS_RCM_DSS_L3_BANKB2_PD_CTRL_DSS_L3_BANKB2_PD_CTRL_AONIN_MASK     (0x00000070U)
#define SDL_DSS_RCM_DSS_L3_BANKB2_PD_CTRL_DSS_L3_BANKB2_PD_CTRL_AONIN_SHIFT    (0x00000004U)
#define SDL_DSS_RCM_DSS_L3_BANKB2_PD_CTRL_DSS_L3_BANKB2_PD_CTRL_AONIN_RESETVAL (0x00000007U)
#define SDL_DSS_RCM_DSS_L3_BANKB2_PD_CTRL_DSS_L3_BANKB2_PD_CTRL_AONIN_MAX      (0x00000007U)

#define SDL_DSS_RCM_DSS_L3_BANKB2_PD_CTRL_DSS_L3_BANKB2_PD_CTRL_AGOODIN_MASK   (0x00000700U)
#define SDL_DSS_RCM_DSS_L3_BANKB2_PD_CTRL_DSS_L3_BANKB2_PD_CTRL_AGOODIN_SHIFT  (0x00000008U)
#define SDL_DSS_RCM_DSS_L3_BANKB2_PD_CTRL_DSS_L3_BANKB2_PD_CTRL_AGOODIN_RESETVAL (0x00000007U)
#define SDL_DSS_RCM_DSS_L3_BANKB2_PD_CTRL_DSS_L3_BANKB2_PD_CTRL_AGOODIN_MAX    (0x00000007U)

#define SDL_DSS_RCM_DSS_L3_BANKB2_PD_CTRL_RESETVAL                             (0x00000770U)

/* DSS_L3_BANKB3_PD_CTRL */

#define SDL_DSS_RCM_DSS_L3_BANKB3_PD_CTRL_DSS_L3_BANKB3_PD_CTRL_ISO_MASK       (0x00000007U)
#define SDL_DSS_RCM_DSS_L3_BANKB3_PD_CTRL_DSS_L3_BANKB3_PD_CTRL_ISO_SHIFT      (0x00000000U)
#define SDL_DSS_RCM_DSS_L3_BANKB3_PD_CTRL_DSS_L3_BANKB3_PD_CTRL_ISO_RESETVAL   (0x00000000U)
#define SDL_DSS_RCM_DSS_L3_BANKB3_PD_CTRL_DSS_L3_BANKB3_PD_CTRL_ISO_MAX        (0x00000007U)

#define SDL_DSS_RCM_DSS_L3_BANKB3_PD_CTRL_DSS_L3_BANKB3_PD_CTRL_AONIN_MASK     (0x00000070U)
#define SDL_DSS_RCM_DSS_L3_BANKB3_PD_CTRL_DSS_L3_BANKB3_PD_CTRL_AONIN_SHIFT    (0x00000004U)
#define SDL_DSS_RCM_DSS_L3_BANKB3_PD_CTRL_DSS_L3_BANKB3_PD_CTRL_AONIN_RESETVAL (0x00000007U)
#define SDL_DSS_RCM_DSS_L3_BANKB3_PD_CTRL_DSS_L3_BANKB3_PD_CTRL_AONIN_MAX      (0x00000007U)

#define SDL_DSS_RCM_DSS_L3_BANKB3_PD_CTRL_DSS_L3_BANKB3_PD_CTRL_AGOODIN_MASK   (0x00000700U)
#define SDL_DSS_RCM_DSS_L3_BANKB3_PD_CTRL_DSS_L3_BANKB3_PD_CTRL_AGOODIN_SHIFT  (0x00000008U)
#define SDL_DSS_RCM_DSS_L3_BANKB3_PD_CTRL_DSS_L3_BANKB3_PD_CTRL_AGOODIN_RESETVAL (0x00000007U)
#define SDL_DSS_RCM_DSS_L3_BANKB3_PD_CTRL_DSS_L3_BANKB3_PD_CTRL_AGOODIN_MAX    (0x00000007U)

#define SDL_DSS_RCM_DSS_L3_BANKB3_PD_CTRL_RESETVAL                             (0x00000770U)

/* DSS_L3_BANKC0_PD_CTRL */

#define SDL_DSS_RCM_DSS_L3_BANKC0_PD_CTRL_DSS_L3_BANKC0_PD_CTRL_ISO_MASK       (0x00000007U)
#define SDL_DSS_RCM_DSS_L3_BANKC0_PD_CTRL_DSS_L3_BANKC0_PD_CTRL_ISO_SHIFT      (0x00000000U)
#define SDL_DSS_RCM_DSS_L3_BANKC0_PD_CTRL_DSS_L3_BANKC0_PD_CTRL_ISO_RESETVAL   (0x00000000U)
#define SDL_DSS_RCM_DSS_L3_BANKC0_PD_CTRL_DSS_L3_BANKC0_PD_CTRL_ISO_MAX        (0x00000007U)

#define SDL_DSS_RCM_DSS_L3_BANKC0_PD_CTRL_DSS_L3_BANKC0_PD_CTRL_AONIN_MASK     (0x00000070U)
#define SDL_DSS_RCM_DSS_L3_BANKC0_PD_CTRL_DSS_L3_BANKC0_PD_CTRL_AONIN_SHIFT    (0x00000004U)
#define SDL_DSS_RCM_DSS_L3_BANKC0_PD_CTRL_DSS_L3_BANKC0_PD_CTRL_AONIN_RESETVAL (0x00000007U)
#define SDL_DSS_RCM_DSS_L3_BANKC0_PD_CTRL_DSS_L3_BANKC0_PD_CTRL_AONIN_MAX      (0x00000007U)

#define SDL_DSS_RCM_DSS_L3_BANKC0_PD_CTRL_DSS_L3_BANKC0_PD_CTRL_AGOODIN_MASK   (0x00000700U)
#define SDL_DSS_RCM_DSS_L3_BANKC0_PD_CTRL_DSS_L3_BANKC0_PD_CTRL_AGOODIN_SHIFT  (0x00000008U)
#define SDL_DSS_RCM_DSS_L3_BANKC0_PD_CTRL_DSS_L3_BANKC0_PD_CTRL_AGOODIN_RESETVAL (0x00000007U)
#define SDL_DSS_RCM_DSS_L3_BANKC0_PD_CTRL_DSS_L3_BANKC0_PD_CTRL_AGOODIN_MAX    (0x00000007U)

#define SDL_DSS_RCM_DSS_L3_BANKC0_PD_CTRL_RESETVAL                             (0x00000770U)

/* DSS_L3_BANKC1_PD_CTRL */

#define SDL_DSS_RCM_DSS_L3_BANKC1_PD_CTRL_DSS_L3_BANKC1_PD_CTRL_ISO_MASK       (0x00000007U)
#define SDL_DSS_RCM_DSS_L3_BANKC1_PD_CTRL_DSS_L3_BANKC1_PD_CTRL_ISO_SHIFT      (0x00000000U)
#define SDL_DSS_RCM_DSS_L3_BANKC1_PD_CTRL_DSS_L3_BANKC1_PD_CTRL_ISO_RESETVAL   (0x00000000U)
#define SDL_DSS_RCM_DSS_L3_BANKC1_PD_CTRL_DSS_L3_BANKC1_PD_CTRL_ISO_MAX        (0x00000007U)

#define SDL_DSS_RCM_DSS_L3_BANKC1_PD_CTRL_DSS_L3_BANKC1_PD_CTRL_AONIN_MASK     (0x00000070U)
#define SDL_DSS_RCM_DSS_L3_BANKC1_PD_CTRL_DSS_L3_BANKC1_PD_CTRL_AONIN_SHIFT    (0x00000004U)
#define SDL_DSS_RCM_DSS_L3_BANKC1_PD_CTRL_DSS_L3_BANKC1_PD_CTRL_AONIN_RESETVAL (0x00000007U)
#define SDL_DSS_RCM_DSS_L3_BANKC1_PD_CTRL_DSS_L3_BANKC1_PD_CTRL_AONIN_MAX      (0x00000007U)

#define SDL_DSS_RCM_DSS_L3_BANKC1_PD_CTRL_DSS_L3_BANKC1_PD_CTRL_AGOODIN_MASK   (0x00000700U)
#define SDL_DSS_RCM_DSS_L3_BANKC1_PD_CTRL_DSS_L3_BANKC1_PD_CTRL_AGOODIN_SHIFT  (0x00000008U)
#define SDL_DSS_RCM_DSS_L3_BANKC1_PD_CTRL_DSS_L3_BANKC1_PD_CTRL_AGOODIN_RESETVAL (0x00000007U)
#define SDL_DSS_RCM_DSS_L3_BANKC1_PD_CTRL_DSS_L3_BANKC1_PD_CTRL_AGOODIN_MAX    (0x00000007U)

#define SDL_DSS_RCM_DSS_L3_BANKC1_PD_CTRL_RESETVAL                             (0x00000770U)

/* DSS_L3_BANKC2_PD_CTRL */

#define SDL_DSS_RCM_DSS_L3_BANKC2_PD_CTRL_DSS_L3_BANKC2_PD_CTRL_ISO_MASK       (0x00000007U)
#define SDL_DSS_RCM_DSS_L3_BANKC2_PD_CTRL_DSS_L3_BANKC2_PD_CTRL_ISO_SHIFT      (0x00000000U)
#define SDL_DSS_RCM_DSS_L3_BANKC2_PD_CTRL_DSS_L3_BANKC2_PD_CTRL_ISO_RESETVAL   (0x00000000U)
#define SDL_DSS_RCM_DSS_L3_BANKC2_PD_CTRL_DSS_L3_BANKC2_PD_CTRL_ISO_MAX        (0x00000007U)

#define SDL_DSS_RCM_DSS_L3_BANKC2_PD_CTRL_DSS_L3_BANKC2_PD_CTRL_AONIN_MASK     (0x00000070U)
#define SDL_DSS_RCM_DSS_L3_BANKC2_PD_CTRL_DSS_L3_BANKC2_PD_CTRL_AONIN_SHIFT    (0x00000004U)
#define SDL_DSS_RCM_DSS_L3_BANKC2_PD_CTRL_DSS_L3_BANKC2_PD_CTRL_AONIN_RESETVAL (0x00000007U)
#define SDL_DSS_RCM_DSS_L3_BANKC2_PD_CTRL_DSS_L3_BANKC2_PD_CTRL_AONIN_MAX      (0x00000007U)

#define SDL_DSS_RCM_DSS_L3_BANKC2_PD_CTRL_DSS_L3_BANKC2_PD_CTRL_AGOODIN_MASK   (0x00000700U)
#define SDL_DSS_RCM_DSS_L3_BANKC2_PD_CTRL_DSS_L3_BANKC2_PD_CTRL_AGOODIN_SHIFT  (0x00000008U)
#define SDL_DSS_RCM_DSS_L3_BANKC2_PD_CTRL_DSS_L3_BANKC2_PD_CTRL_AGOODIN_RESETVAL (0x00000007U)
#define SDL_DSS_RCM_DSS_L3_BANKC2_PD_CTRL_DSS_L3_BANKC2_PD_CTRL_AGOODIN_MAX    (0x00000007U)

#define SDL_DSS_RCM_DSS_L3_BANKC2_PD_CTRL_RESETVAL                             (0x00000770U)

/* DSS_L3_BANKC3_PD_CTRL */

#define SDL_DSS_RCM_DSS_L3_BANKC3_PD_CTRL_DSS_L3_BANKC3_PD_CTRL_ISO_MASK       (0x00000007U)
#define SDL_DSS_RCM_DSS_L3_BANKC3_PD_CTRL_DSS_L3_BANKC3_PD_CTRL_ISO_SHIFT      (0x00000000U)
#define SDL_DSS_RCM_DSS_L3_BANKC3_PD_CTRL_DSS_L3_BANKC3_PD_CTRL_ISO_RESETVAL   (0x00000000U)
#define SDL_DSS_RCM_DSS_L3_BANKC3_PD_CTRL_DSS_L3_BANKC3_PD_CTRL_ISO_MAX        (0x00000007U)

#define SDL_DSS_RCM_DSS_L3_BANKC3_PD_CTRL_DSS_L3_BANKC3_PD_CTRL_AONIN_MASK     (0x00000070U)
#define SDL_DSS_RCM_DSS_L3_BANKC3_PD_CTRL_DSS_L3_BANKC3_PD_CTRL_AONIN_SHIFT    (0x00000004U)
#define SDL_DSS_RCM_DSS_L3_BANKC3_PD_CTRL_DSS_L3_BANKC3_PD_CTRL_AONIN_RESETVAL (0x00000007U)
#define SDL_DSS_RCM_DSS_L3_BANKC3_PD_CTRL_DSS_L3_BANKC3_PD_CTRL_AONIN_MAX      (0x00000007U)

#define SDL_DSS_RCM_DSS_L3_BANKC3_PD_CTRL_DSS_L3_BANKC3_PD_CTRL_AGOODIN_MASK   (0x00000700U)
#define SDL_DSS_RCM_DSS_L3_BANKC3_PD_CTRL_DSS_L3_BANKC3_PD_CTRL_AGOODIN_SHIFT  (0x00000008U)
#define SDL_DSS_RCM_DSS_L3_BANKC3_PD_CTRL_DSS_L3_BANKC3_PD_CTRL_AGOODIN_RESETVAL (0x00000007U)
#define SDL_DSS_RCM_DSS_L3_BANKC3_PD_CTRL_DSS_L3_BANKC3_PD_CTRL_AGOODIN_MAX    (0x00000007U)

#define SDL_DSS_RCM_DSS_L3_BANKC3_PD_CTRL_RESETVAL                             (0x00000770U)

/* DSS_L3_BANKD0_PD_CTRL */

#define SDL_DSS_RCM_DSS_L3_BANKD0_PD_CTRL_DSS_L3_BANKD0_PD_CTRL_ISO_MASK       (0x00000007U)
#define SDL_DSS_RCM_DSS_L3_BANKD0_PD_CTRL_DSS_L3_BANKD0_PD_CTRL_ISO_SHIFT      (0x00000000U)
#define SDL_DSS_RCM_DSS_L3_BANKD0_PD_CTRL_DSS_L3_BANKD0_PD_CTRL_ISO_RESETVAL   (0x00000000U)
#define SDL_DSS_RCM_DSS_L3_BANKD0_PD_CTRL_DSS_L3_BANKD0_PD_CTRL_ISO_MAX        (0x00000007U)

#define SDL_DSS_RCM_DSS_L3_BANKD0_PD_CTRL_DSS_L3_BANKD0_PD_CTRL_AONIN_MASK     (0x00000070U)
#define SDL_DSS_RCM_DSS_L3_BANKD0_PD_CTRL_DSS_L3_BANKD0_PD_CTRL_AONIN_SHIFT    (0x00000004U)
#define SDL_DSS_RCM_DSS_L3_BANKD0_PD_CTRL_DSS_L3_BANKD0_PD_CTRL_AONIN_RESETVAL (0x00000007U)
#define SDL_DSS_RCM_DSS_L3_BANKD0_PD_CTRL_DSS_L3_BANKD0_PD_CTRL_AONIN_MAX      (0x00000007U)

#define SDL_DSS_RCM_DSS_L3_BANKD0_PD_CTRL_DSS_L3_BANKD0_PD_CTRL_AGOODIN_MASK   (0x00000700U)
#define SDL_DSS_RCM_DSS_L3_BANKD0_PD_CTRL_DSS_L3_BANKD0_PD_CTRL_AGOODIN_SHIFT  (0x00000008U)
#define SDL_DSS_RCM_DSS_L3_BANKD0_PD_CTRL_DSS_L3_BANKD0_PD_CTRL_AGOODIN_RESETVAL (0x00000007U)
#define SDL_DSS_RCM_DSS_L3_BANKD0_PD_CTRL_DSS_L3_BANKD0_PD_CTRL_AGOODIN_MAX    (0x00000007U)

#define SDL_DSS_RCM_DSS_L3_BANKD0_PD_CTRL_RESETVAL                             (0x00000770U)

/* DSS_L3_BANKD1_PD_CTRL */

#define SDL_DSS_RCM_DSS_L3_BANKD1_PD_CTRL_DSS_L3_BANKD1_PD_CTRL_ISO_MASK       (0x00000007U)
#define SDL_DSS_RCM_DSS_L3_BANKD1_PD_CTRL_DSS_L3_BANKD1_PD_CTRL_ISO_SHIFT      (0x00000000U)
#define SDL_DSS_RCM_DSS_L3_BANKD1_PD_CTRL_DSS_L3_BANKD1_PD_CTRL_ISO_RESETVAL   (0x00000000U)
#define SDL_DSS_RCM_DSS_L3_BANKD1_PD_CTRL_DSS_L3_BANKD1_PD_CTRL_ISO_MAX        (0x00000007U)

#define SDL_DSS_RCM_DSS_L3_BANKD1_PD_CTRL_DSS_L3_BANKD1_PD_CTRL_AONIN_MASK     (0x00000070U)
#define SDL_DSS_RCM_DSS_L3_BANKD1_PD_CTRL_DSS_L3_BANKD1_PD_CTRL_AONIN_SHIFT    (0x00000004U)
#define SDL_DSS_RCM_DSS_L3_BANKD1_PD_CTRL_DSS_L3_BANKD1_PD_CTRL_AONIN_RESETVAL (0x00000007U)
#define SDL_DSS_RCM_DSS_L3_BANKD1_PD_CTRL_DSS_L3_BANKD1_PD_CTRL_AONIN_MAX      (0x00000007U)

#define SDL_DSS_RCM_DSS_L3_BANKD1_PD_CTRL_DSS_L3_BANKD1_PD_CTRL_AGOODIN_MASK   (0x00000700U)
#define SDL_DSS_RCM_DSS_L3_BANKD1_PD_CTRL_DSS_L3_BANKD1_PD_CTRL_AGOODIN_SHIFT  (0x00000008U)
#define SDL_DSS_RCM_DSS_L3_BANKD1_PD_CTRL_DSS_L3_BANKD1_PD_CTRL_AGOODIN_RESETVAL (0x00000007U)
#define SDL_DSS_RCM_DSS_L3_BANKD1_PD_CTRL_DSS_L3_BANKD1_PD_CTRL_AGOODIN_MAX    (0x00000007U)

#define SDL_DSS_RCM_DSS_L3_BANKD1_PD_CTRL_RESETVAL                             (0x00000770U)

/* DSS_L3_BANKD2_PD_CTRL */

#define SDL_DSS_RCM_DSS_L3_BANKD2_PD_CTRL_DSS_L3_BANKD2_PD_CTRL_ISO_MASK       (0x00000007U)
#define SDL_DSS_RCM_DSS_L3_BANKD2_PD_CTRL_DSS_L3_BANKD2_PD_CTRL_ISO_SHIFT      (0x00000000U)
#define SDL_DSS_RCM_DSS_L3_BANKD2_PD_CTRL_DSS_L3_BANKD2_PD_CTRL_ISO_RESETVAL   (0x00000000U)
#define SDL_DSS_RCM_DSS_L3_BANKD2_PD_CTRL_DSS_L3_BANKD2_PD_CTRL_ISO_MAX        (0x00000007U)

#define SDL_DSS_RCM_DSS_L3_BANKD2_PD_CTRL_DSS_L3_BANKD2_PD_CTRL_AONIN_MASK     (0x00000070U)
#define SDL_DSS_RCM_DSS_L3_BANKD2_PD_CTRL_DSS_L3_BANKD2_PD_CTRL_AONIN_SHIFT    (0x00000004U)
#define SDL_DSS_RCM_DSS_L3_BANKD2_PD_CTRL_DSS_L3_BANKD2_PD_CTRL_AONIN_RESETVAL (0x00000007U)
#define SDL_DSS_RCM_DSS_L3_BANKD2_PD_CTRL_DSS_L3_BANKD2_PD_CTRL_AONIN_MAX      (0x00000007U)

#define SDL_DSS_RCM_DSS_L3_BANKD2_PD_CTRL_DSS_L3_BANKD2_PD_CTRL_AGOODIN_MASK   (0x00000700U)
#define SDL_DSS_RCM_DSS_L3_BANKD2_PD_CTRL_DSS_L3_BANKD2_PD_CTRL_AGOODIN_SHIFT  (0x00000008U)
#define SDL_DSS_RCM_DSS_L3_BANKD2_PD_CTRL_DSS_L3_BANKD2_PD_CTRL_AGOODIN_RESETVAL (0x00000007U)
#define SDL_DSS_RCM_DSS_L3_BANKD2_PD_CTRL_DSS_L3_BANKD2_PD_CTRL_AGOODIN_MAX    (0x00000007U)

#define SDL_DSS_RCM_DSS_L3_BANKD2_PD_CTRL_RESETVAL                             (0x00000770U)

/* DSS_HWA_PD_CTRL */

#define SDL_DSS_RCM_DSS_HWA_PD_CTRL_DSS_HWA_PD_CTRL_ISO_MASK                   (0x00000007U)
#define SDL_DSS_RCM_DSS_HWA_PD_CTRL_DSS_HWA_PD_CTRL_ISO_SHIFT                  (0x00000000U)
#define SDL_DSS_RCM_DSS_HWA_PD_CTRL_DSS_HWA_PD_CTRL_ISO_RESETVAL               (0x00000000U)
#define SDL_DSS_RCM_DSS_HWA_PD_CTRL_DSS_HWA_PD_CTRL_ISO_MAX                    (0x00000007U)

#define SDL_DSS_RCM_DSS_HWA_PD_CTRL_DSS_HWA_PD_CTRL_AONIN_MASK                 (0x00000070U)
#define SDL_DSS_RCM_DSS_HWA_PD_CTRL_DSS_HWA_PD_CTRL_AONIN_SHIFT                (0x00000004U)
#define SDL_DSS_RCM_DSS_HWA_PD_CTRL_DSS_HWA_PD_CTRL_AONIN_RESETVAL             (0x00000007U)
#define SDL_DSS_RCM_DSS_HWA_PD_CTRL_DSS_HWA_PD_CTRL_AONIN_MAX                  (0x00000007U)

#define SDL_DSS_RCM_DSS_HWA_PD_CTRL_DSS_HWA_PD_CTRL_AGOODIN_MASK               (0x00000700U)
#define SDL_DSS_RCM_DSS_HWA_PD_CTRL_DSS_HWA_PD_CTRL_AGOODIN_SHIFT              (0x00000008U)
#define SDL_DSS_RCM_DSS_HWA_PD_CTRL_DSS_HWA_PD_CTRL_AGOODIN_RESETVAL           (0x00000007U)
#define SDL_DSS_RCM_DSS_HWA_PD_CTRL_DSS_HWA_PD_CTRL_AGOODIN_MAX                (0x00000007U)

#define SDL_DSS_RCM_DSS_HWA_PD_CTRL_DSS_HWA_PD_CTRL_PONIN_MASK                 (0x00007000U)
#define SDL_DSS_RCM_DSS_HWA_PD_CTRL_DSS_HWA_PD_CTRL_PONIN_SHIFT                (0x0000000CU)
#define SDL_DSS_RCM_DSS_HWA_PD_CTRL_DSS_HWA_PD_CTRL_PONIN_RESETVAL             (0x00000007U)
#define SDL_DSS_RCM_DSS_HWA_PD_CTRL_DSS_HWA_PD_CTRL_PONIN_MAX                  (0x00000007U)

#define SDL_DSS_RCM_DSS_HWA_PD_CTRL_DSS_HWA_PD_CTRL_PGOODIN_MASK               (0x00070000U)
#define SDL_DSS_RCM_DSS_HWA_PD_CTRL_DSS_HWA_PD_CTRL_PGOODIN_SHIFT              (0x00000010U)
#define SDL_DSS_RCM_DSS_HWA_PD_CTRL_DSS_HWA_PD_CTRL_PGOODIN_RESETVAL           (0x00000007U)
#define SDL_DSS_RCM_DSS_HWA_PD_CTRL_DSS_HWA_PD_CTRL_PGOODIN_MAX                (0x00000007U)

#define SDL_DSS_RCM_DSS_HWA_PD_CTRL_RESETVAL                                   (0x00077770U)

/* DSS_DSP_L2_PD_STATUS */

#define SDL_DSS_RCM_DSS_DSP_L2_PD_STATUS_DSS_DSP_L2_PD_STATUS_AONOUT_MASK      (0x00000001U)
#define SDL_DSS_RCM_DSS_DSP_L2_PD_STATUS_DSS_DSP_L2_PD_STATUS_AONOUT_SHIFT     (0x00000000U)
#define SDL_DSS_RCM_DSS_DSP_L2_PD_STATUS_DSS_DSP_L2_PD_STATUS_AONOUT_RESETVAL  (0x00000001U)
#define SDL_DSS_RCM_DSS_DSP_L2_PD_STATUS_DSS_DSP_L2_PD_STATUS_AONOUT_MAX       (0x00000001U)

#define SDL_DSS_RCM_DSS_DSP_L2_PD_STATUS_DSS_DSP_L2_PD_STATUS_AGOODOUT_MASK    (0x00000002U)
#define SDL_DSS_RCM_DSS_DSP_L2_PD_STATUS_DSS_DSP_L2_PD_STATUS_AGOODOUT_SHIFT   (0x00000001U)
#define SDL_DSS_RCM_DSS_DSP_L2_PD_STATUS_DSS_DSP_L2_PD_STATUS_AGOODOUT_RESETVAL (0x00000001U)
#define SDL_DSS_RCM_DSS_DSP_L2_PD_STATUS_DSS_DSP_L2_PD_STATUS_AGOODOUT_MAX     (0x00000001U)

#define SDL_DSS_RCM_DSS_DSP_L2_PD_STATUS_RESETVAL                              (0x00000003U)

/* DSS_L3_BANKA0_PD_STATUS */

#define SDL_DSS_RCM_DSS_L3_BANKA0_PD_STATUS_DSS_L3_BANKA0_PD_STATUS_AONOUT_MASK (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKA0_PD_STATUS_DSS_L3_BANKA0_PD_STATUS_AONOUT_SHIFT (0x00000000U)
#define SDL_DSS_RCM_DSS_L3_BANKA0_PD_STATUS_DSS_L3_BANKA0_PD_STATUS_AONOUT_RESETVAL (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKA0_PD_STATUS_DSS_L3_BANKA0_PD_STATUS_AONOUT_MAX (0x00000001U)

#define SDL_DSS_RCM_DSS_L3_BANKA0_PD_STATUS_DSS_L3_BANKA0_PD_STATUS_AGOODOUT_MASK (0x00000002U)
#define SDL_DSS_RCM_DSS_L3_BANKA0_PD_STATUS_DSS_L3_BANKA0_PD_STATUS_AGOODOUT_SHIFT (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKA0_PD_STATUS_DSS_L3_BANKA0_PD_STATUS_AGOODOUT_RESETVAL (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKA0_PD_STATUS_DSS_L3_BANKA0_PD_STATUS_AGOODOUT_MAX (0x00000001U)

#define SDL_DSS_RCM_DSS_L3_BANKA0_PD_STATUS_DSS_L3_BANKA0_PD_STATUS_AONIN_MASK (0x00000004U)
#define SDL_DSS_RCM_DSS_L3_BANKA0_PD_STATUS_DSS_L3_BANKA0_PD_STATUS_AONIN_SHIFT (0x00000002U)
#define SDL_DSS_RCM_DSS_L3_BANKA0_PD_STATUS_DSS_L3_BANKA0_PD_STATUS_AONIN_RESETVAL (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKA0_PD_STATUS_DSS_L3_BANKA0_PD_STATUS_AONIN_MAX  (0x00000001U)

#define SDL_DSS_RCM_DSS_L3_BANKA0_PD_STATUS_DSS_L3_BANKA0_PD_STATUS_AGOODIN_MASK (0x00000008U)
#define SDL_DSS_RCM_DSS_L3_BANKA0_PD_STATUS_DSS_L3_BANKA0_PD_STATUS_AGOODIN_SHIFT (0x00000003U)
#define SDL_DSS_RCM_DSS_L3_BANKA0_PD_STATUS_DSS_L3_BANKA0_PD_STATUS_AGOODIN_RESETVAL (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKA0_PD_STATUS_DSS_L3_BANKA0_PD_STATUS_AGOODIN_MAX (0x00000001U)

#define SDL_DSS_RCM_DSS_L3_BANKA0_PD_STATUS_RESETVAL                           (0x0000000FU)

/* DSS_L3_BANKA1_PD_STATUS */

#define SDL_DSS_RCM_DSS_L3_BANKA1_PD_STATUS_DSS_L3_BANKA1_PD_STATUS_AONOUT_MASK (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKA1_PD_STATUS_DSS_L3_BANKA1_PD_STATUS_AONOUT_SHIFT (0x00000000U)
#define SDL_DSS_RCM_DSS_L3_BANKA1_PD_STATUS_DSS_L3_BANKA1_PD_STATUS_AONOUT_RESETVAL (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKA1_PD_STATUS_DSS_L3_BANKA1_PD_STATUS_AONOUT_MAX (0x00000001U)

#define SDL_DSS_RCM_DSS_L3_BANKA1_PD_STATUS_DSS_L3_BANKA1_PD_STATUS_AGOODOUT_MASK (0x00000002U)
#define SDL_DSS_RCM_DSS_L3_BANKA1_PD_STATUS_DSS_L3_BANKA1_PD_STATUS_AGOODOUT_SHIFT (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKA1_PD_STATUS_DSS_L3_BANKA1_PD_STATUS_AGOODOUT_RESETVAL (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKA1_PD_STATUS_DSS_L3_BANKA1_PD_STATUS_AGOODOUT_MAX (0x00000001U)

#define SDL_DSS_RCM_DSS_L3_BANKA1_PD_STATUS_DSS_L3_BANKA1_PD_STATUS_AONIN_MASK (0x00000004U)
#define SDL_DSS_RCM_DSS_L3_BANKA1_PD_STATUS_DSS_L3_BANKA1_PD_STATUS_AONIN_SHIFT (0x00000002U)
#define SDL_DSS_RCM_DSS_L3_BANKA1_PD_STATUS_DSS_L3_BANKA1_PD_STATUS_AONIN_RESETVAL (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKA1_PD_STATUS_DSS_L3_BANKA1_PD_STATUS_AONIN_MAX  (0x00000001U)

#define SDL_DSS_RCM_DSS_L3_BANKA1_PD_STATUS_DSS_L3_BANKA1_PD_STATUS_AGOODIN_MASK (0x00000008U)
#define SDL_DSS_RCM_DSS_L3_BANKA1_PD_STATUS_DSS_L3_BANKA1_PD_STATUS_AGOODIN_SHIFT (0x00000003U)
#define SDL_DSS_RCM_DSS_L3_BANKA1_PD_STATUS_DSS_L3_BANKA1_PD_STATUS_AGOODIN_RESETVAL (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKA1_PD_STATUS_DSS_L3_BANKA1_PD_STATUS_AGOODIN_MAX (0x00000001U)

#define SDL_DSS_RCM_DSS_L3_BANKA1_PD_STATUS_RESETVAL                           (0x0000000FU)

/* DSS_L3_BANKA2_PD_STATUS */

#define SDL_DSS_RCM_DSS_L3_BANKA2_PD_STATUS_DSS_L3_BANKA2_PD_STATUS_AONOUT_MASK (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKA2_PD_STATUS_DSS_L3_BANKA2_PD_STATUS_AONOUT_SHIFT (0x00000000U)
#define SDL_DSS_RCM_DSS_L3_BANKA2_PD_STATUS_DSS_L3_BANKA2_PD_STATUS_AONOUT_RESETVAL (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKA2_PD_STATUS_DSS_L3_BANKA2_PD_STATUS_AONOUT_MAX (0x00000001U)

#define SDL_DSS_RCM_DSS_L3_BANKA2_PD_STATUS_DSS_L3_BANKA2_PD_STATUS_AGOODOUT_MASK (0x00000002U)
#define SDL_DSS_RCM_DSS_L3_BANKA2_PD_STATUS_DSS_L3_BANKA2_PD_STATUS_AGOODOUT_SHIFT (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKA2_PD_STATUS_DSS_L3_BANKA2_PD_STATUS_AGOODOUT_RESETVAL (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKA2_PD_STATUS_DSS_L3_BANKA2_PD_STATUS_AGOODOUT_MAX (0x00000001U)

#define SDL_DSS_RCM_DSS_L3_BANKA2_PD_STATUS_DSS_L3_BANKA2_PD_STATUS_AONIN_MASK (0x00000004U)
#define SDL_DSS_RCM_DSS_L3_BANKA2_PD_STATUS_DSS_L3_BANKA2_PD_STATUS_AONIN_SHIFT (0x00000002U)
#define SDL_DSS_RCM_DSS_L3_BANKA2_PD_STATUS_DSS_L3_BANKA2_PD_STATUS_AONIN_RESETVAL (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKA2_PD_STATUS_DSS_L3_BANKA2_PD_STATUS_AONIN_MAX  (0x00000001U)

#define SDL_DSS_RCM_DSS_L3_BANKA2_PD_STATUS_DSS_L3_BANKA2_PD_STATUS_AGOODIN_MASK (0x00000008U)
#define SDL_DSS_RCM_DSS_L3_BANKA2_PD_STATUS_DSS_L3_BANKA2_PD_STATUS_AGOODIN_SHIFT (0x00000003U)
#define SDL_DSS_RCM_DSS_L3_BANKA2_PD_STATUS_DSS_L3_BANKA2_PD_STATUS_AGOODIN_RESETVAL (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKA2_PD_STATUS_DSS_L3_BANKA2_PD_STATUS_AGOODIN_MAX (0x00000001U)

#define SDL_DSS_RCM_DSS_L3_BANKA2_PD_STATUS_RESETVAL                           (0x0000000FU)

/* DSS_L3_BANKA3_PD_STATUS */

#define SDL_DSS_RCM_DSS_L3_BANKA3_PD_STATUS_DSS_L3_BANKA3_PD_STATUS_AONOUT_MASK (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKA3_PD_STATUS_DSS_L3_BANKA3_PD_STATUS_AONOUT_SHIFT (0x00000000U)
#define SDL_DSS_RCM_DSS_L3_BANKA3_PD_STATUS_DSS_L3_BANKA3_PD_STATUS_AONOUT_RESETVAL (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKA3_PD_STATUS_DSS_L3_BANKA3_PD_STATUS_AONOUT_MAX (0x00000001U)

#define SDL_DSS_RCM_DSS_L3_BANKA3_PD_STATUS_DSS_L3_BANKA3_PD_STATUS_AGOODOUT_MASK (0x00000002U)
#define SDL_DSS_RCM_DSS_L3_BANKA3_PD_STATUS_DSS_L3_BANKA3_PD_STATUS_AGOODOUT_SHIFT (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKA3_PD_STATUS_DSS_L3_BANKA3_PD_STATUS_AGOODOUT_RESETVAL (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKA3_PD_STATUS_DSS_L3_BANKA3_PD_STATUS_AGOODOUT_MAX (0x00000001U)

#define SDL_DSS_RCM_DSS_L3_BANKA3_PD_STATUS_DSS_L3_BANKA3_PD_STATUS_AONIN_MASK (0x00000004U)
#define SDL_DSS_RCM_DSS_L3_BANKA3_PD_STATUS_DSS_L3_BANKA3_PD_STATUS_AONIN_SHIFT (0x00000002U)
#define SDL_DSS_RCM_DSS_L3_BANKA3_PD_STATUS_DSS_L3_BANKA3_PD_STATUS_AONIN_RESETVAL (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKA3_PD_STATUS_DSS_L3_BANKA3_PD_STATUS_AONIN_MAX  (0x00000001U)

#define SDL_DSS_RCM_DSS_L3_BANKA3_PD_STATUS_DSS_L3_BANKA3_PD_STATUS_AGOODIN_MASK (0x00000008U)
#define SDL_DSS_RCM_DSS_L3_BANKA3_PD_STATUS_DSS_L3_BANKA3_PD_STATUS_AGOODIN_SHIFT (0x00000003U)
#define SDL_DSS_RCM_DSS_L3_BANKA3_PD_STATUS_DSS_L3_BANKA3_PD_STATUS_AGOODIN_RESETVAL (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKA3_PD_STATUS_DSS_L3_BANKA3_PD_STATUS_AGOODIN_MAX (0x00000001U)

#define SDL_DSS_RCM_DSS_L3_BANKA3_PD_STATUS_RESETVAL                           (0x0000000FU)

/* DSS_L3_BANKB0_PD_STATUS */

#define SDL_DSS_RCM_DSS_L3_BANKB0_PD_STATUS_DSS_L3_BANKB0_PD_STATUS_AONOUT_MASK (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKB0_PD_STATUS_DSS_L3_BANKB0_PD_STATUS_AONOUT_SHIFT (0x00000000U)
#define SDL_DSS_RCM_DSS_L3_BANKB0_PD_STATUS_DSS_L3_BANKB0_PD_STATUS_AONOUT_RESETVAL (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKB0_PD_STATUS_DSS_L3_BANKB0_PD_STATUS_AONOUT_MAX (0x00000001U)

#define SDL_DSS_RCM_DSS_L3_BANKB0_PD_STATUS_DSS_L3_BANKB0_PD_STATUS_AGOODOUT_MASK (0x00000002U)
#define SDL_DSS_RCM_DSS_L3_BANKB0_PD_STATUS_DSS_L3_BANKB0_PD_STATUS_AGOODOUT_SHIFT (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKB0_PD_STATUS_DSS_L3_BANKB0_PD_STATUS_AGOODOUT_RESETVAL (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKB0_PD_STATUS_DSS_L3_BANKB0_PD_STATUS_AGOODOUT_MAX (0x00000001U)

#define SDL_DSS_RCM_DSS_L3_BANKB0_PD_STATUS_DSS_L3_BANKB0_PD_STATUS_AONIN_MASK (0x00000004U)
#define SDL_DSS_RCM_DSS_L3_BANKB0_PD_STATUS_DSS_L3_BANKB0_PD_STATUS_AONIN_SHIFT (0x00000002U)
#define SDL_DSS_RCM_DSS_L3_BANKB0_PD_STATUS_DSS_L3_BANKB0_PD_STATUS_AONIN_RESETVAL (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKB0_PD_STATUS_DSS_L3_BANKB0_PD_STATUS_AONIN_MAX  (0x00000001U)

#define SDL_DSS_RCM_DSS_L3_BANKB0_PD_STATUS_DSS_L3_BANKB0_PD_STATUS_AGOODIN_MASK (0x00000008U)
#define SDL_DSS_RCM_DSS_L3_BANKB0_PD_STATUS_DSS_L3_BANKB0_PD_STATUS_AGOODIN_SHIFT (0x00000003U)
#define SDL_DSS_RCM_DSS_L3_BANKB0_PD_STATUS_DSS_L3_BANKB0_PD_STATUS_AGOODIN_RESETVAL (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKB0_PD_STATUS_DSS_L3_BANKB0_PD_STATUS_AGOODIN_MAX (0x00000001U)

#define SDL_DSS_RCM_DSS_L3_BANKB0_PD_STATUS_RESETVAL                           (0x0000000FU)

/* DSS_L3_BANKB1_PD_STATUS */

#define SDL_DSS_RCM_DSS_L3_BANKB1_PD_STATUS_DSS_L3_BANKB1_PD_STATUS_AONOUT_MASK (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKB1_PD_STATUS_DSS_L3_BANKB1_PD_STATUS_AONOUT_SHIFT (0x00000000U)
#define SDL_DSS_RCM_DSS_L3_BANKB1_PD_STATUS_DSS_L3_BANKB1_PD_STATUS_AONOUT_RESETVAL (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKB1_PD_STATUS_DSS_L3_BANKB1_PD_STATUS_AONOUT_MAX (0x00000001U)

#define SDL_DSS_RCM_DSS_L3_BANKB1_PD_STATUS_DSS_L3_BANKB1_PD_STATUS_AGOODOUT_MASK (0x00000002U)
#define SDL_DSS_RCM_DSS_L3_BANKB1_PD_STATUS_DSS_L3_BANKB1_PD_STATUS_AGOODOUT_SHIFT (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKB1_PD_STATUS_DSS_L3_BANKB1_PD_STATUS_AGOODOUT_RESETVAL (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKB1_PD_STATUS_DSS_L3_BANKB1_PD_STATUS_AGOODOUT_MAX (0x00000001U)

#define SDL_DSS_RCM_DSS_L3_BANKB1_PD_STATUS_DSS_L3_BANKB1_PD_STATUS_AONIN_MASK (0x00000004U)
#define SDL_DSS_RCM_DSS_L3_BANKB1_PD_STATUS_DSS_L3_BANKB1_PD_STATUS_AONIN_SHIFT (0x00000002U)
#define SDL_DSS_RCM_DSS_L3_BANKB1_PD_STATUS_DSS_L3_BANKB1_PD_STATUS_AONIN_RESETVAL (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKB1_PD_STATUS_DSS_L3_BANKB1_PD_STATUS_AONIN_MAX  (0x00000001U)

#define SDL_DSS_RCM_DSS_L3_BANKB1_PD_STATUS_DSS_L3_BANKB1_PD_STATUS_AGOODIN_MASK (0x00000008U)
#define SDL_DSS_RCM_DSS_L3_BANKB1_PD_STATUS_DSS_L3_BANKB1_PD_STATUS_AGOODIN_SHIFT (0x00000003U)
#define SDL_DSS_RCM_DSS_L3_BANKB1_PD_STATUS_DSS_L3_BANKB1_PD_STATUS_AGOODIN_RESETVAL (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKB1_PD_STATUS_DSS_L3_BANKB1_PD_STATUS_AGOODIN_MAX (0x00000001U)

#define SDL_DSS_RCM_DSS_L3_BANKB1_PD_STATUS_RESETVAL                           (0x0000000FU)

/* DSS_L3_BANKB2_PD_STATUS */

#define SDL_DSS_RCM_DSS_L3_BANKB2_PD_STATUS_DSS_L3_BANKB2_PD_STATUS_AONOUT_MASK (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKB2_PD_STATUS_DSS_L3_BANKB2_PD_STATUS_AONOUT_SHIFT (0x00000000U)
#define SDL_DSS_RCM_DSS_L3_BANKB2_PD_STATUS_DSS_L3_BANKB2_PD_STATUS_AONOUT_RESETVAL (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKB2_PD_STATUS_DSS_L3_BANKB2_PD_STATUS_AONOUT_MAX (0x00000001U)

#define SDL_DSS_RCM_DSS_L3_BANKB2_PD_STATUS_DSS_L3_BANKB2_PD_STATUS_AGOODOUT_MASK (0x00000002U)
#define SDL_DSS_RCM_DSS_L3_BANKB2_PD_STATUS_DSS_L3_BANKB2_PD_STATUS_AGOODOUT_SHIFT (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKB2_PD_STATUS_DSS_L3_BANKB2_PD_STATUS_AGOODOUT_RESETVAL (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKB2_PD_STATUS_DSS_L3_BANKB2_PD_STATUS_AGOODOUT_MAX (0x00000001U)

#define SDL_DSS_RCM_DSS_L3_BANKB2_PD_STATUS_DSS_L3_BANKB2_PD_STATUS_AONIN_MASK (0x00000004U)
#define SDL_DSS_RCM_DSS_L3_BANKB2_PD_STATUS_DSS_L3_BANKB2_PD_STATUS_AONIN_SHIFT (0x00000002U)
#define SDL_DSS_RCM_DSS_L3_BANKB2_PD_STATUS_DSS_L3_BANKB2_PD_STATUS_AONIN_RESETVAL (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKB2_PD_STATUS_DSS_L3_BANKB2_PD_STATUS_AONIN_MAX  (0x00000001U)

#define SDL_DSS_RCM_DSS_L3_BANKB2_PD_STATUS_DSS_L3_BANKB2_PD_STATUS_AGOODIN_MASK (0x00000008U)
#define SDL_DSS_RCM_DSS_L3_BANKB2_PD_STATUS_DSS_L3_BANKB2_PD_STATUS_AGOODIN_SHIFT (0x00000003U)
#define SDL_DSS_RCM_DSS_L3_BANKB2_PD_STATUS_DSS_L3_BANKB2_PD_STATUS_AGOODIN_RESETVAL (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKB2_PD_STATUS_DSS_L3_BANKB2_PD_STATUS_AGOODIN_MAX (0x00000001U)

#define SDL_DSS_RCM_DSS_L3_BANKB2_PD_STATUS_RESETVAL                           (0x0000000FU)

/* DSS_L3_BANKB3_PD_STATUS */

#define SDL_DSS_RCM_DSS_L3_BANKB3_PD_STATUS_DSS_L3_BANKB3_PD_STATUS_AONOUT_MASK (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKB3_PD_STATUS_DSS_L3_BANKB3_PD_STATUS_AONOUT_SHIFT (0x00000000U)
#define SDL_DSS_RCM_DSS_L3_BANKB3_PD_STATUS_DSS_L3_BANKB3_PD_STATUS_AONOUT_RESETVAL (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKB3_PD_STATUS_DSS_L3_BANKB3_PD_STATUS_AONOUT_MAX (0x00000001U)

#define SDL_DSS_RCM_DSS_L3_BANKB3_PD_STATUS_DSS_L3_BANKB3_PD_STATUS_AGOODOUT_MASK (0x00000002U)
#define SDL_DSS_RCM_DSS_L3_BANKB3_PD_STATUS_DSS_L3_BANKB3_PD_STATUS_AGOODOUT_SHIFT (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKB3_PD_STATUS_DSS_L3_BANKB3_PD_STATUS_AGOODOUT_RESETVAL (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKB3_PD_STATUS_DSS_L3_BANKB3_PD_STATUS_AGOODOUT_MAX (0x00000001U)

#define SDL_DSS_RCM_DSS_L3_BANKB3_PD_STATUS_DSS_L3_BANKB3_PD_STATUS_AONIN_MASK (0x00000004U)
#define SDL_DSS_RCM_DSS_L3_BANKB3_PD_STATUS_DSS_L3_BANKB3_PD_STATUS_AONIN_SHIFT (0x00000002U)
#define SDL_DSS_RCM_DSS_L3_BANKB3_PD_STATUS_DSS_L3_BANKB3_PD_STATUS_AONIN_RESETVAL (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKB3_PD_STATUS_DSS_L3_BANKB3_PD_STATUS_AONIN_MAX  (0x00000001U)

#define SDL_DSS_RCM_DSS_L3_BANKB3_PD_STATUS_DSS_L3_BANKB3_PD_STATUS_AGOODIN_MASK (0x00000008U)
#define SDL_DSS_RCM_DSS_L3_BANKB3_PD_STATUS_DSS_L3_BANKB3_PD_STATUS_AGOODIN_SHIFT (0x00000003U)
#define SDL_DSS_RCM_DSS_L3_BANKB3_PD_STATUS_DSS_L3_BANKB3_PD_STATUS_AGOODIN_RESETVAL (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKB3_PD_STATUS_DSS_L3_BANKB3_PD_STATUS_AGOODIN_MAX (0x00000001U)

#define SDL_DSS_RCM_DSS_L3_BANKB3_PD_STATUS_RESETVAL                           (0x0000000FU)

/* DSS_L3_BANKC0_PD_STATUS */

#define SDL_DSS_RCM_DSS_L3_BANKC0_PD_STATUS_DSS_L3_BANKC0_PD_STATUS_AONOUT_MASK (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKC0_PD_STATUS_DSS_L3_BANKC0_PD_STATUS_AONOUT_SHIFT (0x00000000U)
#define SDL_DSS_RCM_DSS_L3_BANKC0_PD_STATUS_DSS_L3_BANKC0_PD_STATUS_AONOUT_RESETVAL (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKC0_PD_STATUS_DSS_L3_BANKC0_PD_STATUS_AONOUT_MAX (0x00000001U)

#define SDL_DSS_RCM_DSS_L3_BANKC0_PD_STATUS_DSS_L3_BANKC0_PD_STATUS_AGOODOUT_MASK (0x00000002U)
#define SDL_DSS_RCM_DSS_L3_BANKC0_PD_STATUS_DSS_L3_BANKC0_PD_STATUS_AGOODOUT_SHIFT (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKC0_PD_STATUS_DSS_L3_BANKC0_PD_STATUS_AGOODOUT_RESETVAL (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKC0_PD_STATUS_DSS_L3_BANKC0_PD_STATUS_AGOODOUT_MAX (0x00000001U)

#define SDL_DSS_RCM_DSS_L3_BANKC0_PD_STATUS_DSS_L3_BANKC0_PD_STATUS_AONIN_MASK (0x00000004U)
#define SDL_DSS_RCM_DSS_L3_BANKC0_PD_STATUS_DSS_L3_BANKC0_PD_STATUS_AONIN_SHIFT (0x00000002U)
#define SDL_DSS_RCM_DSS_L3_BANKC0_PD_STATUS_DSS_L3_BANKC0_PD_STATUS_AONIN_RESETVAL (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKC0_PD_STATUS_DSS_L3_BANKC0_PD_STATUS_AONIN_MAX  (0x00000001U)

#define SDL_DSS_RCM_DSS_L3_BANKC0_PD_STATUS_DSS_L3_BANKC0_PD_STATUS_AGOODIN_MASK (0x00000008U)
#define SDL_DSS_RCM_DSS_L3_BANKC0_PD_STATUS_DSS_L3_BANKC0_PD_STATUS_AGOODIN_SHIFT (0x00000003U)
#define SDL_DSS_RCM_DSS_L3_BANKC0_PD_STATUS_DSS_L3_BANKC0_PD_STATUS_AGOODIN_RESETVAL (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKC0_PD_STATUS_DSS_L3_BANKC0_PD_STATUS_AGOODIN_MAX (0x00000001U)

#define SDL_DSS_RCM_DSS_L3_BANKC0_PD_STATUS_RESETVAL                           (0x0000000FU)

/* DSS_L3_BANKC1_PD_STATUS */

#define SDL_DSS_RCM_DSS_L3_BANKC1_PD_STATUS_DSS_L3_BANKC1_PD_STATUS_AONOUT_MASK (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKC1_PD_STATUS_DSS_L3_BANKC1_PD_STATUS_AONOUT_SHIFT (0x00000000U)
#define SDL_DSS_RCM_DSS_L3_BANKC1_PD_STATUS_DSS_L3_BANKC1_PD_STATUS_AONOUT_RESETVAL (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKC1_PD_STATUS_DSS_L3_BANKC1_PD_STATUS_AONOUT_MAX (0x00000001U)

#define SDL_DSS_RCM_DSS_L3_BANKC1_PD_STATUS_DSS_L3_BANKC1_PD_STATUS_AGOODOUT_MASK (0x00000002U)
#define SDL_DSS_RCM_DSS_L3_BANKC1_PD_STATUS_DSS_L3_BANKC1_PD_STATUS_AGOODOUT_SHIFT (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKC1_PD_STATUS_DSS_L3_BANKC1_PD_STATUS_AGOODOUT_RESETVAL (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKC1_PD_STATUS_DSS_L3_BANKC1_PD_STATUS_AGOODOUT_MAX (0x00000001U)

#define SDL_DSS_RCM_DSS_L3_BANKC1_PD_STATUS_DSS_L3_BANKC1_PD_STATUS_AONIN_MASK (0x00000004U)
#define SDL_DSS_RCM_DSS_L3_BANKC1_PD_STATUS_DSS_L3_BANKC1_PD_STATUS_AONIN_SHIFT (0x00000002U)
#define SDL_DSS_RCM_DSS_L3_BANKC1_PD_STATUS_DSS_L3_BANKC1_PD_STATUS_AONIN_RESETVAL (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKC1_PD_STATUS_DSS_L3_BANKC1_PD_STATUS_AONIN_MAX  (0x00000001U)

#define SDL_DSS_RCM_DSS_L3_BANKC1_PD_STATUS_DSS_L3_BANKC1_PD_STATUS_AGOODIN_MASK (0x00000008U)
#define SDL_DSS_RCM_DSS_L3_BANKC1_PD_STATUS_DSS_L3_BANKC1_PD_STATUS_AGOODIN_SHIFT (0x00000003U)
#define SDL_DSS_RCM_DSS_L3_BANKC1_PD_STATUS_DSS_L3_BANKC1_PD_STATUS_AGOODIN_RESETVAL (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKC1_PD_STATUS_DSS_L3_BANKC1_PD_STATUS_AGOODIN_MAX (0x00000001U)

#define SDL_DSS_RCM_DSS_L3_BANKC1_PD_STATUS_RESETVAL                           (0x0000000FU)

/* DSS_L3_BANKC2_PD_STATUS */

#define SDL_DSS_RCM_DSS_L3_BANKC2_PD_STATUS_DSS_L3_BANKC2_PD_STATUS_AONOUT_MASK (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKC2_PD_STATUS_DSS_L3_BANKC2_PD_STATUS_AONOUT_SHIFT (0x00000000U)
#define SDL_DSS_RCM_DSS_L3_BANKC2_PD_STATUS_DSS_L3_BANKC2_PD_STATUS_AONOUT_RESETVAL (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKC2_PD_STATUS_DSS_L3_BANKC2_PD_STATUS_AONOUT_MAX (0x00000001U)

#define SDL_DSS_RCM_DSS_L3_BANKC2_PD_STATUS_DSS_L3_BANKC2_PD_STATUS_AGOODOUT_MASK (0x00000002U)
#define SDL_DSS_RCM_DSS_L3_BANKC2_PD_STATUS_DSS_L3_BANKC2_PD_STATUS_AGOODOUT_SHIFT (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKC2_PD_STATUS_DSS_L3_BANKC2_PD_STATUS_AGOODOUT_RESETVAL (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKC2_PD_STATUS_DSS_L3_BANKC2_PD_STATUS_AGOODOUT_MAX (0x00000001U)

#define SDL_DSS_RCM_DSS_L3_BANKC2_PD_STATUS_DSS_L3_BANKC2_PD_STATUS_AONIN_MASK (0x00000004U)
#define SDL_DSS_RCM_DSS_L3_BANKC2_PD_STATUS_DSS_L3_BANKC2_PD_STATUS_AONIN_SHIFT (0x00000002U)
#define SDL_DSS_RCM_DSS_L3_BANKC2_PD_STATUS_DSS_L3_BANKC2_PD_STATUS_AONIN_RESETVAL (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKC2_PD_STATUS_DSS_L3_BANKC2_PD_STATUS_AONIN_MAX  (0x00000001U)

#define SDL_DSS_RCM_DSS_L3_BANKC2_PD_STATUS_DSS_L3_BANKC2_PD_STATUS_AGOODIN_MASK (0x00000008U)
#define SDL_DSS_RCM_DSS_L3_BANKC2_PD_STATUS_DSS_L3_BANKC2_PD_STATUS_AGOODIN_SHIFT (0x00000003U)
#define SDL_DSS_RCM_DSS_L3_BANKC2_PD_STATUS_DSS_L3_BANKC2_PD_STATUS_AGOODIN_RESETVAL (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKC2_PD_STATUS_DSS_L3_BANKC2_PD_STATUS_AGOODIN_MAX (0x00000001U)

#define SDL_DSS_RCM_DSS_L3_BANKC2_PD_STATUS_RESETVAL                           (0x0000000FU)

/* DSS_L3_BANKC3_PD_STATUS */

#define SDL_DSS_RCM_DSS_L3_BANKC3_PD_STATUS_DSS_L3_BANKC3_PD_STATUS_AONOUT_MASK (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKC3_PD_STATUS_DSS_L3_BANKC3_PD_STATUS_AONOUT_SHIFT (0x00000000U)
#define SDL_DSS_RCM_DSS_L3_BANKC3_PD_STATUS_DSS_L3_BANKC3_PD_STATUS_AONOUT_RESETVAL (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKC3_PD_STATUS_DSS_L3_BANKC3_PD_STATUS_AONOUT_MAX (0x00000001U)

#define SDL_DSS_RCM_DSS_L3_BANKC3_PD_STATUS_DSS_L3_BANKC3_PD_STATUS_AGOODOUT_MASK (0x00000002U)
#define SDL_DSS_RCM_DSS_L3_BANKC3_PD_STATUS_DSS_L3_BANKC3_PD_STATUS_AGOODOUT_SHIFT (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKC3_PD_STATUS_DSS_L3_BANKC3_PD_STATUS_AGOODOUT_RESETVAL (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKC3_PD_STATUS_DSS_L3_BANKC3_PD_STATUS_AGOODOUT_MAX (0x00000001U)

#define SDL_DSS_RCM_DSS_L3_BANKC3_PD_STATUS_DSS_L3_BANKC3_PD_STATUS_AONIN_MASK (0x00000004U)
#define SDL_DSS_RCM_DSS_L3_BANKC3_PD_STATUS_DSS_L3_BANKC3_PD_STATUS_AONIN_SHIFT (0x00000002U)
#define SDL_DSS_RCM_DSS_L3_BANKC3_PD_STATUS_DSS_L3_BANKC3_PD_STATUS_AONIN_RESETVAL (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKC3_PD_STATUS_DSS_L3_BANKC3_PD_STATUS_AONIN_MAX  (0x00000001U)

#define SDL_DSS_RCM_DSS_L3_BANKC3_PD_STATUS_DSS_L3_BANKC3_PD_STATUS_AGOODIN_MASK (0x00000008U)
#define SDL_DSS_RCM_DSS_L3_BANKC3_PD_STATUS_DSS_L3_BANKC3_PD_STATUS_AGOODIN_SHIFT (0x00000003U)
#define SDL_DSS_RCM_DSS_L3_BANKC3_PD_STATUS_DSS_L3_BANKC3_PD_STATUS_AGOODIN_RESETVAL (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKC3_PD_STATUS_DSS_L3_BANKC3_PD_STATUS_AGOODIN_MAX (0x00000001U)

#define SDL_DSS_RCM_DSS_L3_BANKC3_PD_STATUS_RESETVAL                           (0x0000000FU)

/* DSS_L3_BANKD0_PD_STATUS */

#define SDL_DSS_RCM_DSS_L3_BANKD0_PD_STATUS_DSS_L3_BANKD0_PD_STATUS_AONOUT_MASK (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKD0_PD_STATUS_DSS_L3_BANKD0_PD_STATUS_AONOUT_SHIFT (0x00000000U)
#define SDL_DSS_RCM_DSS_L3_BANKD0_PD_STATUS_DSS_L3_BANKD0_PD_STATUS_AONOUT_RESETVAL (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKD0_PD_STATUS_DSS_L3_BANKD0_PD_STATUS_AONOUT_MAX (0x00000001U)

#define SDL_DSS_RCM_DSS_L3_BANKD0_PD_STATUS_DSS_L3_BANKD0_PD_STATUS_AGOODOUT_MASK (0x00000002U)
#define SDL_DSS_RCM_DSS_L3_BANKD0_PD_STATUS_DSS_L3_BANKD0_PD_STATUS_AGOODOUT_SHIFT (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKD0_PD_STATUS_DSS_L3_BANKD0_PD_STATUS_AGOODOUT_RESETVAL (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKD0_PD_STATUS_DSS_L3_BANKD0_PD_STATUS_AGOODOUT_MAX (0x00000001U)

#define SDL_DSS_RCM_DSS_L3_BANKD0_PD_STATUS_DSS_L3_BANKD0_PD_STATUS_AONIN_MASK (0x00000004U)
#define SDL_DSS_RCM_DSS_L3_BANKD0_PD_STATUS_DSS_L3_BANKD0_PD_STATUS_AONIN_SHIFT (0x00000002U)
#define SDL_DSS_RCM_DSS_L3_BANKD0_PD_STATUS_DSS_L3_BANKD0_PD_STATUS_AONIN_RESETVAL (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKD0_PD_STATUS_DSS_L3_BANKD0_PD_STATUS_AONIN_MAX  (0x00000001U)

#define SDL_DSS_RCM_DSS_L3_BANKD0_PD_STATUS_DSS_L3_BANKD0_PD_STATUS_AGOODIN_MASK (0x00000008U)
#define SDL_DSS_RCM_DSS_L3_BANKD0_PD_STATUS_DSS_L3_BANKD0_PD_STATUS_AGOODIN_SHIFT (0x00000003U)
#define SDL_DSS_RCM_DSS_L3_BANKD0_PD_STATUS_DSS_L3_BANKD0_PD_STATUS_AGOODIN_RESETVAL (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKD0_PD_STATUS_DSS_L3_BANKD0_PD_STATUS_AGOODIN_MAX (0x00000001U)

#define SDL_DSS_RCM_DSS_L3_BANKD0_PD_STATUS_RESETVAL                           (0x0000000FU)

/* DSS_L3_BANKD1_PD_STATUS */

#define SDL_DSS_RCM_DSS_L3_BANKD1_PD_STATUS_DSS_L3_BANKD1_PD_STATUS_AONOUT_MASK (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKD1_PD_STATUS_DSS_L3_BANKD1_PD_STATUS_AONOUT_SHIFT (0x00000000U)
#define SDL_DSS_RCM_DSS_L3_BANKD1_PD_STATUS_DSS_L3_BANKD1_PD_STATUS_AONOUT_RESETVAL (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKD1_PD_STATUS_DSS_L3_BANKD1_PD_STATUS_AONOUT_MAX (0x00000001U)

#define SDL_DSS_RCM_DSS_L3_BANKD1_PD_STATUS_DSS_L3_BANKD1_PD_STATUS_AGOODOUT_MASK (0x00000002U)
#define SDL_DSS_RCM_DSS_L3_BANKD1_PD_STATUS_DSS_L3_BANKD1_PD_STATUS_AGOODOUT_SHIFT (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKD1_PD_STATUS_DSS_L3_BANKD1_PD_STATUS_AGOODOUT_RESETVAL (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKD1_PD_STATUS_DSS_L3_BANKD1_PD_STATUS_AGOODOUT_MAX (0x00000001U)

#define SDL_DSS_RCM_DSS_L3_BANKD1_PD_STATUS_DSS_L3_BANKD1_PD_STATUS_AONIN_MASK (0x00000004U)
#define SDL_DSS_RCM_DSS_L3_BANKD1_PD_STATUS_DSS_L3_BANKD1_PD_STATUS_AONIN_SHIFT (0x00000002U)
#define SDL_DSS_RCM_DSS_L3_BANKD1_PD_STATUS_DSS_L3_BANKD1_PD_STATUS_AONIN_RESETVAL (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKD1_PD_STATUS_DSS_L3_BANKD1_PD_STATUS_AONIN_MAX  (0x00000001U)

#define SDL_DSS_RCM_DSS_L3_BANKD1_PD_STATUS_DSS_L3_BANKD1_PD_STATUS_AGOODIN_MASK (0x00000008U)
#define SDL_DSS_RCM_DSS_L3_BANKD1_PD_STATUS_DSS_L3_BANKD1_PD_STATUS_AGOODIN_SHIFT (0x00000003U)
#define SDL_DSS_RCM_DSS_L3_BANKD1_PD_STATUS_DSS_L3_BANKD1_PD_STATUS_AGOODIN_RESETVAL (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKD1_PD_STATUS_DSS_L3_BANKD1_PD_STATUS_AGOODIN_MAX (0x00000001U)

#define SDL_DSS_RCM_DSS_L3_BANKD1_PD_STATUS_RESETVAL                           (0x0000000FU)

/* DSS_L3_BANKD2_PD_STATUS */

#define SDL_DSS_RCM_DSS_L3_BANKD2_PD_STATUS_DSS_L3_BANKD2_PD_STATUS_AONOUT_MASK (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKD2_PD_STATUS_DSS_L3_BANKD2_PD_STATUS_AONOUT_SHIFT (0x00000000U)
#define SDL_DSS_RCM_DSS_L3_BANKD2_PD_STATUS_DSS_L3_BANKD2_PD_STATUS_AONOUT_RESETVAL (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKD2_PD_STATUS_DSS_L3_BANKD2_PD_STATUS_AONOUT_MAX (0x00000001U)

#define SDL_DSS_RCM_DSS_L3_BANKD2_PD_STATUS_DSS_L3_BANKD2_PD_STATUS_AGOODOUT_MASK (0x00000002U)
#define SDL_DSS_RCM_DSS_L3_BANKD2_PD_STATUS_DSS_L3_BANKD2_PD_STATUS_AGOODOUT_SHIFT (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKD2_PD_STATUS_DSS_L3_BANKD2_PD_STATUS_AGOODOUT_RESETVAL (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKD2_PD_STATUS_DSS_L3_BANKD2_PD_STATUS_AGOODOUT_MAX (0x00000001U)

#define SDL_DSS_RCM_DSS_L3_BANKD2_PD_STATUS_DSS_L3_BANKD2_PD_STATUS_AONIN_MASK (0x00000004U)
#define SDL_DSS_RCM_DSS_L3_BANKD2_PD_STATUS_DSS_L3_BANKD2_PD_STATUS_AONIN_SHIFT (0x00000002U)
#define SDL_DSS_RCM_DSS_L3_BANKD2_PD_STATUS_DSS_L3_BANKD2_PD_STATUS_AONIN_RESETVAL (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKD2_PD_STATUS_DSS_L3_BANKD2_PD_STATUS_AONIN_MAX  (0x00000001U)

#define SDL_DSS_RCM_DSS_L3_BANKD2_PD_STATUS_DSS_L3_BANKD2_PD_STATUS_AGOODIN_MASK (0x00000008U)
#define SDL_DSS_RCM_DSS_L3_BANKD2_PD_STATUS_DSS_L3_BANKD2_PD_STATUS_AGOODIN_SHIFT (0x00000003U)
#define SDL_DSS_RCM_DSS_L3_BANKD2_PD_STATUS_DSS_L3_BANKD2_PD_STATUS_AGOODIN_RESETVAL (0x00000001U)
#define SDL_DSS_RCM_DSS_L3_BANKD2_PD_STATUS_DSS_L3_BANKD2_PD_STATUS_AGOODIN_MAX (0x00000001U)

#define SDL_DSS_RCM_DSS_L3_BANKD2_PD_STATUS_RESETVAL                           (0x0000000FU)

/* DSS_HWA_PD_STATUS */

#define SDL_DSS_RCM_DSS_HWA_PD_STATUS_DSS_HWA_PD_STATUS_AONOUT_MASK            (0x00000001U)
#define SDL_DSS_RCM_DSS_HWA_PD_STATUS_DSS_HWA_PD_STATUS_AONOUT_SHIFT           (0x00000000U)
#define SDL_DSS_RCM_DSS_HWA_PD_STATUS_DSS_HWA_PD_STATUS_AONOUT_RESETVAL        (0x00000001U)
#define SDL_DSS_RCM_DSS_HWA_PD_STATUS_DSS_HWA_PD_STATUS_AONOUT_MAX             (0x00000001U)

#define SDL_DSS_RCM_DSS_HWA_PD_STATUS_DSS_HWA_PD_STATUS_AGOODOUT_MASK          (0x00000002U)
#define SDL_DSS_RCM_DSS_HWA_PD_STATUS_DSS_HWA_PD_STATUS_AGOODOUT_SHIFT         (0x00000001U)
#define SDL_DSS_RCM_DSS_HWA_PD_STATUS_DSS_HWA_PD_STATUS_AGOODOUT_RESETVAL      (0x00000001U)
#define SDL_DSS_RCM_DSS_HWA_PD_STATUS_DSS_HWA_PD_STATUS_AGOODOUT_MAX           (0x00000001U)

#define SDL_DSS_RCM_DSS_HWA_PD_STATUS_DSS_HWA_PD_STATUS_PONOUT_MASK            (0x00000004U)
#define SDL_DSS_RCM_DSS_HWA_PD_STATUS_DSS_HWA_PD_STATUS_PONOUT_SHIFT           (0x00000002U)
#define SDL_DSS_RCM_DSS_HWA_PD_STATUS_DSS_HWA_PD_STATUS_PONOUT_RESETVAL        (0x00000001U)
#define SDL_DSS_RCM_DSS_HWA_PD_STATUS_DSS_HWA_PD_STATUS_PONOUT_MAX             (0x00000001U)

#define SDL_DSS_RCM_DSS_HWA_PD_STATUS_DSS_HWA_PD_STATUS_PGOODOUT_MASK          (0x00000008U)
#define SDL_DSS_RCM_DSS_HWA_PD_STATUS_DSS_HWA_PD_STATUS_PGOODOUT_SHIFT         (0x00000003U)
#define SDL_DSS_RCM_DSS_HWA_PD_STATUS_DSS_HWA_PD_STATUS_PGOODOUT_RESETVAL      (0x00000001U)
#define SDL_DSS_RCM_DSS_HWA_PD_STATUS_DSS_HWA_PD_STATUS_PGOODOUT_MAX           (0x00000001U)

#define SDL_DSS_RCM_DSS_HWA_PD_STATUS_RESETVAL                                 (0x0000000FU)

/* DSS_DSP_TRCCLK_DIVRATIO */

#define SDL_DSS_RCM_DSS_DSP_TRCCLK_DIVRATIO_DSS_DSP_TRCCLK_DIVRATIO_DIVRATIO_MASK (0x0000000FU)
#define SDL_DSS_RCM_DSS_DSP_TRCCLK_DIVRATIO_DSS_DSP_TRCCLK_DIVRATIO_DIVRATIO_SHIFT (0x00000000U)
#define SDL_DSS_RCM_DSS_DSP_TRCCLK_DIVRATIO_DSS_DSP_TRCCLK_DIVRATIO_DIVRATIO_RESETVAL (0x00000003U)
#define SDL_DSS_RCM_DSS_DSP_TRCCLK_DIVRATIO_DSS_DSP_TRCCLK_DIVRATIO_DIVRATIO_MAX (0x0000000FU)

#define SDL_DSS_RCM_DSS_DSP_TRCCLK_DIVRATIO_RESETVAL                           (0x00000003U)

/* DSS_DSP_TCLK_DIVRATIO */

#define SDL_DSS_RCM_DSS_DSP_TCLK_DIVRATIO_DSS_DSP_TCLK_DIVRATIO_DIVRATIO_MASK  (0x0000000FU)
#define SDL_DSS_RCM_DSS_DSP_TCLK_DIVRATIO_DSS_DSP_TCLK_DIVRATIO_DIVRATIO_SHIFT (0x00000000U)
#define SDL_DSS_RCM_DSS_DSP_TCLK_DIVRATIO_DSS_DSP_TCLK_DIVRATIO_DIVRATIO_RESETVAL (0x00000003U)
#define SDL_DSS_RCM_DSS_DSP_TCLK_DIVRATIO_DSS_DSP_TCLK_DIVRATIO_DIVRATIO_MAX   (0x0000000FU)

#define SDL_DSS_RCM_DSS_DSP_TCLK_DIVRATIO_RESETVAL                             (0x00000003U)

/* DSS_DSP_DITHERED_CLK_CTRL */

#define SDL_DSS_RCM_DSS_DSP_DITHERED_CLK_CTRL_DSS_DSP_DITHERED_CLK_CTRL_SEED_MASK (0x0FFFFFFFU)
#define SDL_DSS_RCM_DSS_DSP_DITHERED_CLK_CTRL_DSS_DSP_DITHERED_CLK_CTRL_SEED_SHIFT (0x00000000U)
#define SDL_DSS_RCM_DSS_DSP_DITHERED_CLK_CTRL_DSS_DSP_DITHERED_CLK_CTRL_SEED_RESETVAL (0x00000000U)
#define SDL_DSS_RCM_DSS_DSP_DITHERED_CLK_CTRL_DSS_DSP_DITHERED_CLK_CTRL_SEED_MAX (0x0FFFFFFFU)

#define SDL_DSS_RCM_DSS_DSP_DITHERED_CLK_CTRL_DSS_DSP_DITHERED_CLK_CTRL_ENABLE_MASK (0x70000000U)
#define SDL_DSS_RCM_DSS_DSP_DITHERED_CLK_CTRL_DSS_DSP_DITHERED_CLK_CTRL_ENABLE_SHIFT (0x0000001CU)
#define SDL_DSS_RCM_DSS_DSP_DITHERED_CLK_CTRL_DSS_DSP_DITHERED_CLK_CTRL_ENABLE_RESETVAL (0x00000000U)
#define SDL_DSS_RCM_DSS_DSP_DITHERED_CLK_CTRL_DSS_DSP_DITHERED_CLK_CTRL_ENABLE_MAX (0x00000007U)

#define SDL_DSS_RCM_DSS_DSP_DITHERED_CLK_CTRL_DSS_DSP_DITHERED_CLK_CTRL_LOAD_MASK (0x80000000U)
#define SDL_DSS_RCM_DSS_DSP_DITHERED_CLK_CTRL_DSS_DSP_DITHERED_CLK_CTRL_LOAD_SHIFT (0x0000001FU)
#define SDL_DSS_RCM_DSS_DSP_DITHERED_CLK_CTRL_DSS_DSP_DITHERED_CLK_CTRL_LOAD_RESETVAL (0x00000000U)
#define SDL_DSS_RCM_DSS_DSP_DITHERED_CLK_CTRL_DSS_DSP_DITHERED_CLK_CTRL_LOAD_MAX (0x00000001U)

#define SDL_DSS_RCM_DSS_DSP_DITHERED_CLK_CTRL_RESETVAL                         (0x00000000U)

/* DSS_L3_PD_CTRL_STICKYBIT */

#define SDL_DSS_RCM_DSS_L3_PD_CTRL_STICKYBIT_DSS_L3_PD_CTRL_STICKYBIT_SET_MASK (0x00000007U)
#define SDL_DSS_RCM_DSS_L3_PD_CTRL_STICKYBIT_DSS_L3_PD_CTRL_STICKYBIT_SET_SHIFT (0x00000000U)
#define SDL_DSS_RCM_DSS_L3_PD_CTRL_STICKYBIT_DSS_L3_PD_CTRL_STICKYBIT_SET_RESETVAL (0x00000000U)
#define SDL_DSS_RCM_DSS_L3_PD_CTRL_STICKYBIT_DSS_L3_PD_CTRL_STICKYBIT_SET_MAX  (0x00000007U)

#define SDL_DSS_RCM_DSS_L3_PD_CTRL_STICKYBIT_RESETVAL                          (0x00000000U)

/* DSP_PD_CTRL_MISC2 */

#define SDL_DSS_RCM_DSP_PD_CTRL_MISC2_DSP_PD_CTRL_MISC2_PWRSM_PGOOD_ASSERTCNT_MASK (0x0000FFFFU)
#define SDL_DSS_RCM_DSP_PD_CTRL_MISC2_DSP_PD_CTRL_MISC2_PWRSM_PGOOD_ASSERTCNT_SHIFT (0x00000000U)
#define SDL_DSS_RCM_DSP_PD_CTRL_MISC2_DSP_PD_CTRL_MISC2_PWRSM_PGOOD_ASSERTCNT_RESETVAL (0x00000010U)
#define SDL_DSS_RCM_DSP_PD_CTRL_MISC2_DSP_PD_CTRL_MISC2_PWRSM_PGOOD_ASSERTCNT_MAX (0x0000FFFFU)

#define SDL_DSS_RCM_DSP_PD_CTRL_MISC2_DSP_PD_CTRL_MISC2_PWRSM_AGOOD_ASSERTCNT_MASK (0xFFFF0000U)
#define SDL_DSS_RCM_DSP_PD_CTRL_MISC2_DSP_PD_CTRL_MISC2_PWRSM_AGOOD_ASSERTCNT_SHIFT (0x00000010U)
#define SDL_DSS_RCM_DSP_PD_CTRL_MISC2_DSP_PD_CTRL_MISC2_PWRSM_AGOOD_ASSERTCNT_RESETVAL (0x00000010U)
#define SDL_DSS_RCM_DSP_PD_CTRL_MISC2_DSP_PD_CTRL_MISC2_PWRSM_AGOOD_ASSERTCNT_MAX (0x0000FFFFU)

#define SDL_DSS_RCM_DSP_PD_CTRL_MISC2_RESETVAL                                 (0x00100010U)

/* DSP_PD_CTRL_MISC3 */

#define SDL_DSS_RCM_DSP_PD_CTRL_MISC3_DSP_PD_CTRL_MISC3_PWRS_PD_WAITCNT_MASK   (0x0000FFFFU)
#define SDL_DSS_RCM_DSP_PD_CTRL_MISC3_DSP_PD_CTRL_MISC3_PWRS_PD_WAITCNT_SHIFT  (0x00000000U)
#define SDL_DSS_RCM_DSP_PD_CTRL_MISC3_DSP_PD_CTRL_MISC3_PWRS_PD_WAITCNT_RESETVAL (0x00000010U)
#define SDL_DSS_RCM_DSP_PD_CTRL_MISC3_DSP_PD_CTRL_MISC3_PWRS_PD_WAITCNT_MAX    (0x0000FFFFU)

#define SDL_DSS_RCM_DSP_PD_CTRL_MISC3_DSP_PD_CTRL_MISC3_LRESET_REQ_GATE_MASK   (0x00010000U)
#define SDL_DSS_RCM_DSP_PD_CTRL_MISC3_DSP_PD_CTRL_MISC3_LRESET_REQ_GATE_SHIFT  (0x00000010U)
#define SDL_DSS_RCM_DSP_PD_CTRL_MISC3_DSP_PD_CTRL_MISC3_LRESET_REQ_GATE_RESETVAL (0x00000000U)
#define SDL_DSS_RCM_DSP_PD_CTRL_MISC3_DSP_PD_CTRL_MISC3_LRESET_REQ_GATE_MAX    (0x00000001U)

#define SDL_DSS_RCM_DSP_PD_CTRL_MISC3_RESETVAL                                 (0x00000010U)

/* DSP_PD_CTRL_OVERRIDE0 */

#define SDL_DSS_RCM_DSP_PD_CTRL_OVERRIDE0_DSP_PD_CTRL_OVERRIDE0_BYPASS_VAL_MASK (0x00FFFFFFU)
#define SDL_DSS_RCM_DSP_PD_CTRL_OVERRIDE0_DSP_PD_CTRL_OVERRIDE0_BYPASS_VAL_SHIFT (0x00000000U)
#define SDL_DSS_RCM_DSP_PD_CTRL_OVERRIDE0_DSP_PD_CTRL_OVERRIDE0_BYPASS_VAL_RESETVAL (0x00000000U)
#define SDL_DSS_RCM_DSP_PD_CTRL_OVERRIDE0_DSP_PD_CTRL_OVERRIDE0_BYPASS_VAL_MAX (0x00FFFFFFU)

#define SDL_DSS_RCM_DSP_PD_CTRL_OVERRIDE0_DSP_PD_CTRL_OVERRIDE0_STATE_BYPASS_VAL_MASK (0x3F000000U)
#define SDL_DSS_RCM_DSP_PD_CTRL_OVERRIDE0_DSP_PD_CTRL_OVERRIDE0_STATE_BYPASS_VAL_SHIFT (0x00000018U)
#define SDL_DSS_RCM_DSP_PD_CTRL_OVERRIDE0_DSP_PD_CTRL_OVERRIDE0_STATE_BYPASS_VAL_RESETVAL (0x00000000U)
#define SDL_DSS_RCM_DSP_PD_CTRL_OVERRIDE0_DSP_PD_CTRL_OVERRIDE0_STATE_BYPASS_VAL_MAX (0x0000003FU)

#define SDL_DSS_RCM_DSP_PD_CTRL_OVERRIDE0_RESETVAL                             (0x00000000U)

/* DSP_PD_CTRL_OVERRIDE1 */

#define SDL_DSS_RCM_DSP_PD_CTRL_OVERRIDE1_DSP_PD_CTRL_OVERRIDE1_BYPASS_EN_MASK (0x00FFFFFFU)
#define SDL_DSS_RCM_DSP_PD_CTRL_OVERRIDE1_DSP_PD_CTRL_OVERRIDE1_BYPASS_EN_SHIFT (0x00000000U)
#define SDL_DSS_RCM_DSP_PD_CTRL_OVERRIDE1_DSP_PD_CTRL_OVERRIDE1_BYPASS_EN_RESETVAL (0x00000000U)
#define SDL_DSS_RCM_DSP_PD_CTRL_OVERRIDE1_DSP_PD_CTRL_OVERRIDE1_BYPASS_EN_MAX  (0x00FFFFFFU)

#define SDL_DSS_RCM_DSP_PD_CTRL_OVERRIDE1_DSP_PD_CTRL_OVERRIDE1_STATE_BYPASS_EN_MASK (0x01000000U)
#define SDL_DSS_RCM_DSP_PD_CTRL_OVERRIDE1_DSP_PD_CTRL_OVERRIDE1_STATE_BYPASS_EN_SHIFT (0x00000018U)
#define SDL_DSS_RCM_DSP_PD_CTRL_OVERRIDE1_DSP_PD_CTRL_OVERRIDE1_STATE_BYPASS_EN_RESETVAL (0x00000000U)
#define SDL_DSS_RCM_DSP_PD_CTRL_OVERRIDE1_DSP_PD_CTRL_OVERRIDE1_STATE_BYPASS_EN_MAX (0x00000001U)

#define SDL_DSS_RCM_DSP_PD_CTRL_OVERRIDE1_RESETVAL                             (0x00000000U)

/* DSP_PD_CTRL_OVERRIDE2 */

#define SDL_DSS_RCM_DSP_PD_CTRL_OVERRIDE2_DSP_PD_CTRL_OVERRIDE2_OVERRIDE_ENABLE_MASK (0x00000007U)
#define SDL_DSS_RCM_DSP_PD_CTRL_OVERRIDE2_DSP_PD_CTRL_OVERRIDE2_OVERRIDE_ENABLE_SHIFT (0x00000000U)
#define SDL_DSS_RCM_DSP_PD_CTRL_OVERRIDE2_DSP_PD_CTRL_OVERRIDE2_OVERRIDE_ENABLE_RESETVAL (0x00000000U)
#define SDL_DSS_RCM_DSP_PD_CTRL_OVERRIDE2_DSP_PD_CTRL_OVERRIDE2_OVERRIDE_ENABLE_MAX (0x00000007U)

#define SDL_DSS_RCM_DSP_PD_CTRL_OVERRIDE2_RESETVAL                             (0x00000000U)

/* DSS_HWA_RST_CTRL */

#define SDL_DSS_RCM_DSS_HWA_RST_CTRL_DSS_HWA_RST_CTRL_ASSERT_MASK              (0x00000007U)
#define SDL_DSS_RCM_DSS_HWA_RST_CTRL_DSS_HWA_RST_CTRL_ASSERT_SHIFT             (0x00000000U)
#define SDL_DSS_RCM_DSS_HWA_RST_CTRL_DSS_HWA_RST_CTRL_ASSERT_RESETVAL          (0x00000000U)
#define SDL_DSS_RCM_DSS_HWA_RST_CTRL_DSS_HWA_RST_CTRL_ASSERT_MAX               (0x00000007U)

#define SDL_DSS_RCM_DSS_HWA_RST_CTRL_RESETVAL                                  (0x00000000U)

/* DSS_TPCCA_RST_CTRL */

#define SDL_DSS_RCM_DSS_TPCCA_RST_CTRL_DSS_TPCCA_RST_CTRL_ASSERT_MASK          (0x00000007U)
#define SDL_DSS_RCM_DSS_TPCCA_RST_CTRL_DSS_TPCCA_RST_CTRL_ASSERT_SHIFT         (0x00000000U)
#define SDL_DSS_RCM_DSS_TPCCA_RST_CTRL_DSS_TPCCA_RST_CTRL_ASSERT_RESETVAL      (0x00000000U)
#define SDL_DSS_RCM_DSS_TPCCA_RST_CTRL_DSS_TPCCA_RST_CTRL_ASSERT_MAX           (0x00000007U)

#define SDL_DSS_RCM_DSS_TPCCA_RST_CTRL_RESETVAL                                (0x00000000U)

/* DSS_TPCCB_RST_CTRL */

#define SDL_DSS_RCM_DSS_TPCCB_RST_CTRL_DSS_TPCCB_RST_CTRL_ASSERT_MASK          (0x00000007U)
#define SDL_DSS_RCM_DSS_TPCCB_RST_CTRL_DSS_TPCCB_RST_CTRL_ASSERT_SHIFT         (0x00000000U)
#define SDL_DSS_RCM_DSS_TPCCB_RST_CTRL_DSS_TPCCB_RST_CTRL_ASSERT_RESETVAL      (0x00000000U)
#define SDL_DSS_RCM_DSS_TPCCB_RST_CTRL_DSS_TPCCB_RST_CTRL_ASSERT_MAX           (0x00000007U)

#define SDL_DSS_RCM_DSS_TPCCB_RST_CTRL_RESETVAL                                (0x00000000U)

/* DSS_TPCCC_RST_CTRL */

#define SDL_DSS_RCM_DSS_TPCCC_RST_CTRL_DSS_TPCCC_RST_CTRL_ASSERT_MASK          (0x00000007U)
#define SDL_DSS_RCM_DSS_TPCCC_RST_CTRL_DSS_TPCCC_RST_CTRL_ASSERT_SHIFT         (0x00000000U)
#define SDL_DSS_RCM_DSS_TPCCC_RST_CTRL_DSS_TPCCC_RST_CTRL_ASSERT_RESETVAL      (0x00000000U)
#define SDL_DSS_RCM_DSS_TPCCC_RST_CTRL_DSS_TPCCC_RST_CTRL_ASSERT_MAX           (0x00000007U)

#define SDL_DSS_RCM_DSS_TPCCC_RST_CTRL_RESETVAL                                (0x00000000U)

/* DSS_TPTCA_RST_CTRL */

#define SDL_DSS_RCM_DSS_TPTCA_RST_CTRL_DSS_TPTCA_RST_CTRL_ASSERT_TC0_MASK      (0x00000007U)
#define SDL_DSS_RCM_DSS_TPTCA_RST_CTRL_DSS_TPTCA_RST_CTRL_ASSERT_TC0_SHIFT     (0x00000000U)
#define SDL_DSS_RCM_DSS_TPTCA_RST_CTRL_DSS_TPTCA_RST_CTRL_ASSERT_TC0_RESETVAL  (0x00000000U)
#define SDL_DSS_RCM_DSS_TPTCA_RST_CTRL_DSS_TPTCA_RST_CTRL_ASSERT_TC0_MAX       (0x00000007U)

#define SDL_DSS_RCM_DSS_TPTCA_RST_CTRL_DSS_TPTCA_RST_CTRL_ASSERT_TC1_MASK      (0x00000070U)
#define SDL_DSS_RCM_DSS_TPTCA_RST_CTRL_DSS_TPTCA_RST_CTRL_ASSERT_TC1_SHIFT     (0x00000004U)
#define SDL_DSS_RCM_DSS_TPTCA_RST_CTRL_DSS_TPTCA_RST_CTRL_ASSERT_TC1_RESETVAL  (0x00000000U)
#define SDL_DSS_RCM_DSS_TPTCA_RST_CTRL_DSS_TPTCA_RST_CTRL_ASSERT_TC1_MAX       (0x00000007U)

#define SDL_DSS_RCM_DSS_TPTCA_RST_CTRL_RESETVAL                                (0x00000000U)

/* DSS_TPTCB_RST_CTRL */

#define SDL_DSS_RCM_DSS_TPTCB_RST_CTRL_DSS_TPTCB_RST_CTRL_ASSERT_TC0_MASK      (0x00000007U)
#define SDL_DSS_RCM_DSS_TPTCB_RST_CTRL_DSS_TPTCB_RST_CTRL_ASSERT_TC0_SHIFT     (0x00000000U)
#define SDL_DSS_RCM_DSS_TPTCB_RST_CTRL_DSS_TPTCB_RST_CTRL_ASSERT_TC0_RESETVAL  (0x00000000U)
#define SDL_DSS_RCM_DSS_TPTCB_RST_CTRL_DSS_TPTCB_RST_CTRL_ASSERT_TC0_MAX       (0x00000007U)

#define SDL_DSS_RCM_DSS_TPTCB_RST_CTRL_DSS_TPTCB_RST_CTRL_ASSERT_TC1_MASK      (0x00000070U)
#define SDL_DSS_RCM_DSS_TPTCB_RST_CTRL_DSS_TPTCB_RST_CTRL_ASSERT_TC1_SHIFT     (0x00000004U)
#define SDL_DSS_RCM_DSS_TPTCB_RST_CTRL_DSS_TPTCB_RST_CTRL_ASSERT_TC1_RESETVAL  (0x00000000U)
#define SDL_DSS_RCM_DSS_TPTCB_RST_CTRL_DSS_TPTCB_RST_CTRL_ASSERT_TC1_MAX       (0x00000007U)

#define SDL_DSS_RCM_DSS_TPTCB_RST_CTRL_RESETVAL                                (0x00000000U)

/* DSS_TPTCC_RST_CTRL */

#define SDL_DSS_RCM_DSS_TPTCC_RST_CTRL_DSS_TPTCC_RST_CTRL_ASSERT_TC0_MASK      (0x00000007U)
#define SDL_DSS_RCM_DSS_TPTCC_RST_CTRL_DSS_TPTCC_RST_CTRL_ASSERT_TC0_SHIFT     (0x00000000U)
#define SDL_DSS_RCM_DSS_TPTCC_RST_CTRL_DSS_TPTCC_RST_CTRL_ASSERT_TC0_RESETVAL  (0x00000000U)
#define SDL_DSS_RCM_DSS_TPTCC_RST_CTRL_DSS_TPTCC_RST_CTRL_ASSERT_TC0_MAX       (0x00000007U)

#define SDL_DSS_RCM_DSS_TPTCC_RST_CTRL_DSS_TPTCC_RST_CTRL_ASSERT_TC1_MASK      (0x00000070U)
#define SDL_DSS_RCM_DSS_TPTCC_RST_CTRL_DSS_TPTCC_RST_CTRL_ASSERT_TC1_SHIFT     (0x00000004U)
#define SDL_DSS_RCM_DSS_TPTCC_RST_CTRL_DSS_TPTCC_RST_CTRL_ASSERT_TC1_RESETVAL  (0x00000000U)
#define SDL_DSS_RCM_DSS_TPTCC_RST_CTRL_DSS_TPTCC_RST_CTRL_ASSERT_TC1_MAX       (0x00000007U)

#define SDL_DSS_RCM_DSS_TPTCC_RST_CTRL_DSS_TPTCC_RST_CTRL_ASSERT_TC2_MASK      (0x00000700U)
#define SDL_DSS_RCM_DSS_TPTCC_RST_CTRL_DSS_TPTCC_RST_CTRL_ASSERT_TC2_SHIFT     (0x00000008U)
#define SDL_DSS_RCM_DSS_TPTCC_RST_CTRL_DSS_TPTCC_RST_CTRL_ASSERT_TC2_RESETVAL  (0x00000000U)
#define SDL_DSS_RCM_DSS_TPTCC_RST_CTRL_DSS_TPTCC_RST_CTRL_ASSERT_TC2_MAX       (0x00000007U)

#define SDL_DSS_RCM_DSS_TPTCC_RST_CTRL_DSS_TPTCC_RST_CTRL_ASSERT_TC3_MASK      (0x00007000U)
#define SDL_DSS_RCM_DSS_TPTCC_RST_CTRL_DSS_TPTCC_RST_CTRL_ASSERT_TC3_SHIFT     (0x0000000CU)
#define SDL_DSS_RCM_DSS_TPTCC_RST_CTRL_DSS_TPTCC_RST_CTRL_ASSERT_TC3_RESETVAL  (0x00000000U)
#define SDL_DSS_RCM_DSS_TPTCC_RST_CTRL_DSS_TPTCC_RST_CTRL_ASSERT_TC3_MAX       (0x00000007U)

#define SDL_DSS_RCM_DSS_TPTCC_RST_CTRL_DSS_TPTCC_RST_CTRL_ASSERT_TC4_MASK      (0x00070000U)
#define SDL_DSS_RCM_DSS_TPTCC_RST_CTRL_DSS_TPTCC_RST_CTRL_ASSERT_TC4_SHIFT     (0x00000010U)
#define SDL_DSS_RCM_DSS_TPTCC_RST_CTRL_DSS_TPTCC_RST_CTRL_ASSERT_TC4_RESETVAL  (0x00000000U)
#define SDL_DSS_RCM_DSS_TPTCC_RST_CTRL_DSS_TPTCC_RST_CTRL_ASSERT_TC4_MAX       (0x00000007U)

#define SDL_DSS_RCM_DSS_TPTCC_RST_CTRL_DSS_TPTCC_RST_CTRL_ASSERT_TC5_MASK      (0x00700000U)
#define SDL_DSS_RCM_DSS_TPTCC_RST_CTRL_DSS_TPTCC_RST_CTRL_ASSERT_TC5_SHIFT     (0x00000014U)
#define SDL_DSS_RCM_DSS_TPTCC_RST_CTRL_DSS_TPTCC_RST_CTRL_ASSERT_TC5_RESETVAL  (0x00000000U)
#define SDL_DSS_RCM_DSS_TPTCC_RST_CTRL_DSS_TPTCC_RST_CTRL_ASSERT_TC5_MAX       (0x00000007U)

#define SDL_DSS_RCM_DSS_TPTCC_RST_CTRL_RESETVAL                                (0x00000000U)

/* HW_SPARE_RW0 */

#define SDL_DSS_RCM_HW_SPARE_RW0_HW_SPARE_RW0_HW_SPARE_RW0_MASK                (0xFFFFFFFFU)
#define SDL_DSS_RCM_HW_SPARE_RW0_HW_SPARE_RW0_HW_SPARE_RW0_SHIFT               (0x00000000U)
#define SDL_DSS_RCM_HW_SPARE_RW0_HW_SPARE_RW0_HW_SPARE_RW0_RESETVAL            (0x00000000U)
#define SDL_DSS_RCM_HW_SPARE_RW0_HW_SPARE_RW0_HW_SPARE_RW0_MAX                 (0xFFFFFFFFU)

#define SDL_DSS_RCM_HW_SPARE_RW0_RESETVAL                                      (0x00000000U)

/* HW_SPARE_RW1 */

#define SDL_DSS_RCM_HW_SPARE_RW1_HW_SPARE_RW1_HW_SPARE_RW1_MASK                (0xFFFFFFFFU)
#define SDL_DSS_RCM_HW_SPARE_RW1_HW_SPARE_RW1_HW_SPARE_RW1_SHIFT               (0x00000000U)
#define SDL_DSS_RCM_HW_SPARE_RW1_HW_SPARE_RW1_HW_SPARE_RW1_RESETVAL            (0x00000000U)
#define SDL_DSS_RCM_HW_SPARE_RW1_HW_SPARE_RW1_HW_SPARE_RW1_MAX                 (0xFFFFFFFFU)

#define SDL_DSS_RCM_HW_SPARE_RW1_RESETVAL                                      (0x00000000U)

/* HW_SPARE_RW2 */

#define SDL_DSS_RCM_HW_SPARE_RW2_HW_SPARE_RW2_HW_SPARE_RW2_MASK                (0xFFFFFFFFU)
#define SDL_DSS_RCM_HW_SPARE_RW2_HW_SPARE_RW2_HW_SPARE_RW2_SHIFT               (0x00000000U)
#define SDL_DSS_RCM_HW_SPARE_RW2_HW_SPARE_RW2_HW_SPARE_RW2_RESETVAL            (0x00000000U)
#define SDL_DSS_RCM_HW_SPARE_RW2_HW_SPARE_RW2_HW_SPARE_RW2_MAX                 (0xFFFFFFFFU)

#define SDL_DSS_RCM_HW_SPARE_RW2_RESETVAL                                      (0x00000000U)

/* HW_SPARE_RW3 */

#define SDL_DSS_RCM_HW_SPARE_RW3_HW_SPARE_RW3_HW_SPARE_RW3_MASK                (0xFFFFFFFFU)
#define SDL_DSS_RCM_HW_SPARE_RW3_HW_SPARE_RW3_HW_SPARE_RW3_SHIFT               (0x00000000U)
#define SDL_DSS_RCM_HW_SPARE_RW3_HW_SPARE_RW3_HW_SPARE_RW3_RESETVAL            (0x00000000U)
#define SDL_DSS_RCM_HW_SPARE_RW3_HW_SPARE_RW3_HW_SPARE_RW3_MAX                 (0xFFFFFFFFU)

#define SDL_DSS_RCM_HW_SPARE_RW3_RESETVAL                                      (0x00000000U)

/* HW_SPARE_RO0 */

#define SDL_DSS_RCM_HW_SPARE_RO0_HW_SPARE_RO0_HW_SPARE_RO0_MASK                (0xFFFFFFFFU)
#define SDL_DSS_RCM_HW_SPARE_RO0_HW_SPARE_RO0_HW_SPARE_RO0_SHIFT               (0x00000000U)
#define SDL_DSS_RCM_HW_SPARE_RO0_HW_SPARE_RO0_HW_SPARE_RO0_RESETVAL            (0x00000000U)
#define SDL_DSS_RCM_HW_SPARE_RO0_HW_SPARE_RO0_HW_SPARE_RO0_MAX                 (0xFFFFFFFFU)

#define SDL_DSS_RCM_HW_SPARE_RO0_RESETVAL                                      (0x00000000U)

/* HW_SPARE_RO1 */

#define SDL_DSS_RCM_HW_SPARE_RO1_HW_SPARE_RO1_HW_SPARE_RO1_MASK                (0xFFFFFFFFU)
#define SDL_DSS_RCM_HW_SPARE_RO1_HW_SPARE_RO1_HW_SPARE_RO1_SHIFT               (0x00000000U)
#define SDL_DSS_RCM_HW_SPARE_RO1_HW_SPARE_RO1_HW_SPARE_RO1_RESETVAL            (0x00000000U)
#define SDL_DSS_RCM_HW_SPARE_RO1_HW_SPARE_RO1_HW_SPARE_RO1_MAX                 (0xFFFFFFFFU)

#define SDL_DSS_RCM_HW_SPARE_RO1_RESETVAL                                      (0x00000000U)

/* HW_SPARE_RO2 */

#define SDL_DSS_RCM_HW_SPARE_RO2_HW_SPARE_RO2_HW_SPARE_RO2_MASK                (0xFFFFFFFFU)
#define SDL_DSS_RCM_HW_SPARE_RO2_HW_SPARE_RO2_HW_SPARE_RO2_SHIFT               (0x00000000U)
#define SDL_DSS_RCM_HW_SPARE_RO2_HW_SPARE_RO2_HW_SPARE_RO2_RESETVAL            (0x00000000U)
#define SDL_DSS_RCM_HW_SPARE_RO2_HW_SPARE_RO2_HW_SPARE_RO2_MAX                 (0xFFFFFFFFU)

#define SDL_DSS_RCM_HW_SPARE_RO2_RESETVAL                                      (0x00000000U)

/* HW_SPARE_RO3 */

#define SDL_DSS_RCM_HW_SPARE_RO3_HW_SPARE_RO3_HW_SPARE_RO3_MASK                (0xFFFFFFFFU)
#define SDL_DSS_RCM_HW_SPARE_RO3_HW_SPARE_RO3_HW_SPARE_RO3_SHIFT               (0x00000000U)
#define SDL_DSS_RCM_HW_SPARE_RO3_HW_SPARE_RO3_HW_SPARE_RO3_RESETVAL            (0x00000000U)
#define SDL_DSS_RCM_HW_SPARE_RO3_HW_SPARE_RO3_HW_SPARE_RO3_MAX                 (0xFFFFFFFFU)

#define SDL_DSS_RCM_HW_SPARE_RO3_RESETVAL                                      (0x00000000U)

/* HW_SPARE_WPH */

#define SDL_DSS_RCM_HW_SPARE_WPH_HW_SPARE_WPH_HW_SPARE_WPH_MASK                (0xFFFFFFFFU)
#define SDL_DSS_RCM_HW_SPARE_WPH_HW_SPARE_WPH_HW_SPARE_WPH_SHIFT               (0x00000000U)
#define SDL_DSS_RCM_HW_SPARE_WPH_HW_SPARE_WPH_HW_SPARE_WPH_RESETVAL            (0x00000000U)
#define SDL_DSS_RCM_HW_SPARE_WPH_HW_SPARE_WPH_HW_SPARE_WPH_MAX                 (0xFFFFFFFFU)

#define SDL_DSS_RCM_HW_SPARE_WPH_RESETVAL                                      (0x00000000U)

/* HW_SPARE_REC */

#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC0_MASK               (0x00000001U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC0_SHIFT              (0x00000000U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC0_RESETVAL           (0x00000000U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC0_MAX                (0x00000001U)

#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC1_MASK               (0x00000002U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC1_SHIFT              (0x00000001U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC1_RESETVAL           (0x00000000U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC1_MAX                (0x00000001U)

#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC2_MASK               (0x00000004U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC2_SHIFT              (0x00000002U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC2_RESETVAL           (0x00000000U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC2_MAX                (0x00000001U)

#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC3_MASK               (0x00000008U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC3_SHIFT              (0x00000003U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC3_RESETVAL           (0x00000000U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC3_MAX                (0x00000001U)

#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC4_MASK               (0x00000010U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC4_SHIFT              (0x00000004U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC4_RESETVAL           (0x00000000U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC4_MAX                (0x00000001U)

#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC5_MASK               (0x00000020U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC5_SHIFT              (0x00000005U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC5_RESETVAL           (0x00000000U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC5_MAX                (0x00000001U)

#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC6_MASK               (0x00000040U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC6_SHIFT              (0x00000006U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC6_RESETVAL           (0x00000000U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC6_MAX                (0x00000001U)

#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC7_MASK               (0x00000080U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC7_SHIFT              (0x00000007U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC7_RESETVAL           (0x00000000U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC7_MAX                (0x00000001U)

#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC8_MASK               (0x00000100U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC8_SHIFT              (0x00000008U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC8_RESETVAL           (0x00000000U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC8_MAX                (0x00000001U)

#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC9_MASK               (0x00000200U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC9_SHIFT              (0x00000009U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC9_RESETVAL           (0x00000000U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC9_MAX                (0x00000001U)

#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC10_MASK              (0x00000400U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC10_SHIFT             (0x0000000AU)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC10_RESETVAL          (0x00000000U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC10_MAX               (0x00000001U)

#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC11_MASK              (0x00000800U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC11_SHIFT             (0x0000000BU)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC11_RESETVAL          (0x00000000U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC11_MAX               (0x00000001U)

#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC12_MASK              (0x00001000U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC12_SHIFT             (0x0000000CU)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC12_RESETVAL          (0x00000000U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC12_MAX               (0x00000001U)

#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC13_MASK              (0x00002000U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC13_SHIFT             (0x0000000DU)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC13_RESETVAL          (0x00000000U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC13_MAX               (0x00000001U)

#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC14_MASK              (0x00004000U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC14_SHIFT             (0x0000000EU)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC14_RESETVAL          (0x00000000U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC14_MAX               (0x00000001U)

#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC15_MASK              (0x00008000U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC15_SHIFT             (0x0000000FU)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC15_RESETVAL          (0x00000000U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC15_MAX               (0x00000001U)

#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC16_MASK              (0x00010000U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC16_SHIFT             (0x00000010U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC16_RESETVAL          (0x00000000U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC16_MAX               (0x00000001U)

#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC17_MASK              (0x00020000U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC17_SHIFT             (0x00000011U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC17_RESETVAL          (0x00000000U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC17_MAX               (0x00000001U)

#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC18_MASK              (0x00040000U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC18_SHIFT             (0x00000012U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC18_RESETVAL          (0x00000000U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC18_MAX               (0x00000001U)

#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC19_MASK              (0x00080000U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC19_SHIFT             (0x00000013U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC19_RESETVAL          (0x00000000U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC19_MAX               (0x00000001U)

#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC20_MASK              (0x00100000U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC20_SHIFT             (0x00000014U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC20_RESETVAL          (0x00000000U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC20_MAX               (0x00000001U)

#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC21_MASK              (0x00200000U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC21_SHIFT             (0x00000015U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC21_RESETVAL          (0x00000000U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC21_MAX               (0x00000001U)

#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC22_MASK              (0x00400000U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC22_SHIFT             (0x00000016U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC22_RESETVAL          (0x00000000U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC22_MAX               (0x00000001U)

#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC23_MASK              (0x00800000U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC23_SHIFT             (0x00000017U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC23_RESETVAL          (0x00000000U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC23_MAX               (0x00000001U)

#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC24_MASK              (0x01000000U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC24_SHIFT             (0x00000018U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC24_RESETVAL          (0x00000000U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC24_MAX               (0x00000001U)

#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC25_MASK              (0x02000000U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC25_SHIFT             (0x00000019U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC25_RESETVAL          (0x00000000U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC25_MAX               (0x00000001U)

#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC26_MASK              (0x04000000U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC26_SHIFT             (0x0000001AU)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC26_RESETVAL          (0x00000000U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC26_MAX               (0x00000001U)

#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC27_MASK              (0x08000000U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC27_SHIFT             (0x0000001BU)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC27_RESETVAL          (0x00000000U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC27_MAX               (0x00000001U)

#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC28_MASK              (0x10000000U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC28_SHIFT             (0x0000001CU)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC28_RESETVAL          (0x00000000U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC28_MAX               (0x00000001U)

#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC29_MASK              (0x20000000U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC29_SHIFT             (0x0000001DU)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC29_RESETVAL          (0x00000000U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC29_MAX               (0x00000001U)

#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC30_MASK              (0x40000000U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC30_SHIFT             (0x0000001EU)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC30_RESETVAL          (0x00000000U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC30_MAX               (0x00000001U)

#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC31_MASK              (0x80000000U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC31_SHIFT             (0x0000001FU)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC31_RESETVAL          (0x00000000U)
#define SDL_DSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC31_MAX               (0x00000001U)

#define SDL_DSS_RCM_HW_SPARE_REC_RESETVAL                                      (0x00000000U)

/* LOCK0_KICK0 */

#define SDL_DSS_RCM_LOCK0_KICK0_LOCK0_KICK0_MASK                               (0xFFFFFFFFU)
#define SDL_DSS_RCM_LOCK0_KICK0_LOCK0_KICK0_SHIFT                              (0x00000000U)
#define SDL_DSS_RCM_LOCK0_KICK0_LOCK0_KICK0_RESETVAL                           (0x00000000U)
#define SDL_DSS_RCM_LOCK0_KICK0_LOCK0_KICK0_MAX                                (0xFFFFFFFFU)

#define SDL_DSS_RCM_LOCK0_KICK0_RESETVAL                                       (0x00000000U)

/* LOCK0_KICK1 */

#define SDL_DSS_RCM_LOCK0_KICK1_LOCK0_KICK1_MASK                               (0xFFFFFFFFU)
#define SDL_DSS_RCM_LOCK0_KICK1_LOCK0_KICK1_SHIFT                              (0x00000000U)
#define SDL_DSS_RCM_LOCK0_KICK1_LOCK0_KICK1_RESETVAL                           (0x00000000U)
#define SDL_DSS_RCM_LOCK0_KICK1_LOCK0_KICK1_MAX                                (0xFFFFFFFFU)

#define SDL_DSS_RCM_LOCK0_KICK1_RESETVAL                                       (0x00000000U)

/* INTR_RAW_STATUS */

#define SDL_DSS_RCM_INTR_RAW_STATUS_PROT_ERR_MASK                              (0x00000001U)
#define SDL_DSS_RCM_INTR_RAW_STATUS_PROT_ERR_SHIFT                             (0x00000000U)
#define SDL_DSS_RCM_INTR_RAW_STATUS_PROT_ERR_RESETVAL                          (0x00000000U)
#define SDL_DSS_RCM_INTR_RAW_STATUS_PROT_ERR_MAX                               (0x00000001U)

#define SDL_DSS_RCM_INTR_RAW_STATUS_ADDR_ERR_MASK                              (0x00000002U)
#define SDL_DSS_RCM_INTR_RAW_STATUS_ADDR_ERR_SHIFT                             (0x00000001U)
#define SDL_DSS_RCM_INTR_RAW_STATUS_ADDR_ERR_RESETVAL                          (0x00000000U)
#define SDL_DSS_RCM_INTR_RAW_STATUS_ADDR_ERR_MAX                               (0x00000001U)

#define SDL_DSS_RCM_INTR_RAW_STATUS_KICK_ERR_MASK                              (0x00000004U)
#define SDL_DSS_RCM_INTR_RAW_STATUS_KICK_ERR_SHIFT                             (0x00000002U)
#define SDL_DSS_RCM_INTR_RAW_STATUS_KICK_ERR_RESETVAL                          (0x00000000U)
#define SDL_DSS_RCM_INTR_RAW_STATUS_KICK_ERR_MAX                               (0x00000001U)

#define SDL_DSS_RCM_INTR_RAW_STATUS_PROXY_ERR_MASK                             (0x00000008U)
#define SDL_DSS_RCM_INTR_RAW_STATUS_PROXY_ERR_SHIFT                            (0x00000003U)
#define SDL_DSS_RCM_INTR_RAW_STATUS_PROXY_ERR_RESETVAL                         (0x00000000U)
#define SDL_DSS_RCM_INTR_RAW_STATUS_PROXY_ERR_MAX                              (0x00000001U)

#define SDL_DSS_RCM_INTR_RAW_STATUS_RESETVAL                                   (0x00000000U)

/* INTR_ENABLED_STATUS_CLEAR */

#define SDL_DSS_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR_MASK            (0x00000001U)
#define SDL_DSS_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR_SHIFT           (0x00000000U)
#define SDL_DSS_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR_RESETVAL        (0x00000000U)
#define SDL_DSS_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR_MAX             (0x00000001U)

#define SDL_DSS_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR_MASK            (0x00000002U)
#define SDL_DSS_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR_SHIFT           (0x00000001U)
#define SDL_DSS_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR_RESETVAL        (0x00000000U)
#define SDL_DSS_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR_MAX             (0x00000001U)

#define SDL_DSS_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_KICK_ERR_MASK            (0x00000004U)
#define SDL_DSS_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_KICK_ERR_SHIFT           (0x00000002U)
#define SDL_DSS_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_KICK_ERR_RESETVAL        (0x00000000U)
#define SDL_DSS_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_KICK_ERR_MAX             (0x00000001U)

#define SDL_DSS_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROXY_ERR_MASK           (0x00000008U)
#define SDL_DSS_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROXY_ERR_SHIFT          (0x00000003U)
#define SDL_DSS_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROXY_ERR_RESETVAL       (0x00000000U)
#define SDL_DSS_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROXY_ERR_MAX            (0x00000001U)

#define SDL_DSS_RCM_INTR_ENABLED_STATUS_CLEAR_RESETVAL                         (0x00000000U)

/* INTR_ENABLE */

#define SDL_DSS_RCM_INTR_ENABLE_PROT_ERR_EN_MASK                               (0x00000001U)
#define SDL_DSS_RCM_INTR_ENABLE_PROT_ERR_EN_SHIFT                              (0x00000000U)
#define SDL_DSS_RCM_INTR_ENABLE_PROT_ERR_EN_RESETVAL                           (0x00000000U)
#define SDL_DSS_RCM_INTR_ENABLE_PROT_ERR_EN_MAX                                (0x00000001U)

#define SDL_DSS_RCM_INTR_ENABLE_ADDR_ERR_EN_MASK                               (0x00000002U)
#define SDL_DSS_RCM_INTR_ENABLE_ADDR_ERR_EN_SHIFT                              (0x00000001U)
#define SDL_DSS_RCM_INTR_ENABLE_ADDR_ERR_EN_RESETVAL                           (0x00000000U)
#define SDL_DSS_RCM_INTR_ENABLE_ADDR_ERR_EN_MAX                                (0x00000001U)

#define SDL_DSS_RCM_INTR_ENABLE_KICK_ERR_EN_MASK                               (0x00000004U)
#define SDL_DSS_RCM_INTR_ENABLE_KICK_ERR_EN_SHIFT                              (0x00000002U)
#define SDL_DSS_RCM_INTR_ENABLE_KICK_ERR_EN_RESETVAL                           (0x00000000U)
#define SDL_DSS_RCM_INTR_ENABLE_KICK_ERR_EN_MAX                                (0x00000001U)

#define SDL_DSS_RCM_INTR_ENABLE_PROXY_ERR_EN_MASK                              (0x00000008U)
#define SDL_DSS_RCM_INTR_ENABLE_PROXY_ERR_EN_SHIFT                             (0x00000003U)
#define SDL_DSS_RCM_INTR_ENABLE_PROXY_ERR_EN_RESETVAL                          (0x00000000U)
#define SDL_DSS_RCM_INTR_ENABLE_PROXY_ERR_EN_MAX                               (0x00000001U)

#define SDL_DSS_RCM_INTR_ENABLE_RESETVAL                                       (0x00000000U)

/* INTR_ENABLE_CLEAR */

#define SDL_DSS_RCM_INTR_ENABLE_CLEAR_PROT_ERR_EN_CLR_MASK                     (0x00000001U)
#define SDL_DSS_RCM_INTR_ENABLE_CLEAR_PROT_ERR_EN_CLR_SHIFT                    (0x00000000U)
#define SDL_DSS_RCM_INTR_ENABLE_CLEAR_PROT_ERR_EN_CLR_RESETVAL                 (0x00000000U)
#define SDL_DSS_RCM_INTR_ENABLE_CLEAR_PROT_ERR_EN_CLR_MAX                      (0x00000001U)

#define SDL_DSS_RCM_INTR_ENABLE_CLEAR_ADDR_ERR_EN_CLR_MASK                     (0x00000002U)
#define SDL_DSS_RCM_INTR_ENABLE_CLEAR_ADDR_ERR_EN_CLR_SHIFT                    (0x00000001U)
#define SDL_DSS_RCM_INTR_ENABLE_CLEAR_ADDR_ERR_EN_CLR_RESETVAL                 (0x00000000U)
#define SDL_DSS_RCM_INTR_ENABLE_CLEAR_ADDR_ERR_EN_CLR_MAX                      (0x00000001U)

#define SDL_DSS_RCM_INTR_ENABLE_CLEAR_KICK_ERR_EN_CLR_MASK                     (0x00000004U)
#define SDL_DSS_RCM_INTR_ENABLE_CLEAR_KICK_ERR_EN_CLR_SHIFT                    (0x00000002U)
#define SDL_DSS_RCM_INTR_ENABLE_CLEAR_KICK_ERR_EN_CLR_RESETVAL                 (0x00000000U)
#define SDL_DSS_RCM_INTR_ENABLE_CLEAR_KICK_ERR_EN_CLR_MAX                      (0x00000001U)

#define SDL_DSS_RCM_INTR_ENABLE_CLEAR_PROXY_ERR_EN_CLR_MASK                    (0x00000008U)
#define SDL_DSS_RCM_INTR_ENABLE_CLEAR_PROXY_ERR_EN_CLR_SHIFT                   (0x00000003U)
#define SDL_DSS_RCM_INTR_ENABLE_CLEAR_PROXY_ERR_EN_CLR_RESETVAL                (0x00000000U)
#define SDL_DSS_RCM_INTR_ENABLE_CLEAR_PROXY_ERR_EN_CLR_MAX                     (0x00000001U)

#define SDL_DSS_RCM_INTR_ENABLE_CLEAR_RESETVAL                                 (0x00000000U)

/* EOI */

#define SDL_DSS_RCM_EOI_EOI_VECTOR_MASK                                        (0x000000FFU)
#define SDL_DSS_RCM_EOI_EOI_VECTOR_SHIFT                                       (0x00000000U)
#define SDL_DSS_RCM_EOI_EOI_VECTOR_RESETVAL                                    (0x00000000U)
#define SDL_DSS_RCM_EOI_EOI_VECTOR_MAX                                         (0x000000FFU)

#define SDL_DSS_RCM_EOI_RESETVAL                                               (0x00000000U)

/* FAULT_ADDRESS */

#define SDL_DSS_RCM_FAULT_ADDRESS_FAULT_ADDR_MASK                              (0xFFFFFFFFU)
#define SDL_DSS_RCM_FAULT_ADDRESS_FAULT_ADDR_SHIFT                             (0x00000000U)
#define SDL_DSS_RCM_FAULT_ADDRESS_FAULT_ADDR_RESETVAL                          (0x00000000U)
#define SDL_DSS_RCM_FAULT_ADDRESS_FAULT_ADDR_MAX                               (0xFFFFFFFFU)

#define SDL_DSS_RCM_FAULT_ADDRESS_RESETVAL                                     (0x00000000U)

/* FAULT_TYPE_STATUS */

#define SDL_DSS_RCM_FAULT_TYPE_STATUS_FAULT_TYPE_MASK                          (0x0000003FU)
#define SDL_DSS_RCM_FAULT_TYPE_STATUS_FAULT_TYPE_SHIFT                         (0x00000000U)
#define SDL_DSS_RCM_FAULT_TYPE_STATUS_FAULT_TYPE_RESETVAL                      (0x00000000U)
#define SDL_DSS_RCM_FAULT_TYPE_STATUS_FAULT_TYPE_MAX                           (0x0000003FU)

#define SDL_DSS_RCM_FAULT_TYPE_STATUS_FAULT_NS_MASK                            (0x00000040U)
#define SDL_DSS_RCM_FAULT_TYPE_STATUS_FAULT_NS_SHIFT                           (0x00000006U)
#define SDL_DSS_RCM_FAULT_TYPE_STATUS_FAULT_NS_RESETVAL                        (0x00000000U)
#define SDL_DSS_RCM_FAULT_TYPE_STATUS_FAULT_NS_MAX                             (0x00000001U)

#define SDL_DSS_RCM_FAULT_TYPE_STATUS_RESETVAL                                 (0x00000000U)

/* FAULT_ATTR_STATUS */

#define SDL_DSS_RCM_FAULT_ATTR_STATUS_FAULT_PRIVID_MASK                        (0x000000FFU)
#define SDL_DSS_RCM_FAULT_ATTR_STATUS_FAULT_PRIVID_SHIFT                       (0x00000000U)
#define SDL_DSS_RCM_FAULT_ATTR_STATUS_FAULT_PRIVID_RESETVAL                    (0x00000000U)
#define SDL_DSS_RCM_FAULT_ATTR_STATUS_FAULT_PRIVID_MAX                         (0x000000FFU)

#define SDL_DSS_RCM_FAULT_ATTR_STATUS_FAULT_ROUTEID_MASK                       (0x000FFF00U)
#define SDL_DSS_RCM_FAULT_ATTR_STATUS_FAULT_ROUTEID_SHIFT                      (0x00000008U)
#define SDL_DSS_RCM_FAULT_ATTR_STATUS_FAULT_ROUTEID_RESETVAL                   (0x00000000U)
#define SDL_DSS_RCM_FAULT_ATTR_STATUS_FAULT_ROUTEID_MAX                        (0x00000FFFU)

#define SDL_DSS_RCM_FAULT_ATTR_STATUS_FAULT_XID_MASK                           (0xFFF00000U)
#define SDL_DSS_RCM_FAULT_ATTR_STATUS_FAULT_XID_SHIFT                          (0x00000014U)
#define SDL_DSS_RCM_FAULT_ATTR_STATUS_FAULT_XID_RESETVAL                       (0x00000000U)
#define SDL_DSS_RCM_FAULT_ATTR_STATUS_FAULT_XID_MAX                            (0x00000FFFU)

#define SDL_DSS_RCM_FAULT_ATTR_STATUS_RESETVAL                                 (0x00000000U)

/* FAULT_CLEAR */

#define SDL_DSS_RCM_FAULT_CLEAR_FAULT_CLR_MASK                                 (0x00000001U)
#define SDL_DSS_RCM_FAULT_CLEAR_FAULT_CLR_SHIFT                                (0x00000000U)
#define SDL_DSS_RCM_FAULT_CLEAR_FAULT_CLR_RESETVAL                             (0x00000000U)
#define SDL_DSS_RCM_FAULT_CLEAR_FAULT_CLR_MAX                                  (0x00000001U)

#define SDL_DSS_RCM_FAULT_CLEAR_RESETVAL                                       (0x00000000U)

#ifdef __cplusplus
}
#endif
#endif
