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
 *  Name        : sdlr_mss_rcm.h
*/
#ifndef SDLR_MSS_RCM_H_
#define SDLR_MSS_RCM_H_

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
    volatile uint32_t MSS_RST_CAUSE_CLR;
    volatile uint32_t MSS_RST_STATUS;
    volatile uint32_t SYSRST_BY_DBG_RST;
    volatile uint32_t RST_ASSERDLY;
    volatile uint32_t RST2ASSERTDLY;
    volatile uint32_t RST_WFICHECK;
    volatile uint32_t MSS_MCANA_CLK_SRC_SEL;
    volatile uint32_t MSS_MCANB_CLK_SRC_SEL;
    volatile uint32_t MSS_QSPI_CLK_SRC_SEL;
    volatile uint32_t MSS_RTIA_CLK_SRC_SEL;
    volatile uint32_t MSS_RTIB_CLK_SRC_SEL;
    volatile uint32_t MSS_RTIC_CLK_SRC_SEL;
    volatile uint32_t MSS_WDT_CLK_SRC_SEL;
    volatile uint32_t MSS_SPIA_CLK_SRC_SEL;
    volatile uint32_t MSS_SPIB_CLK_SRC_SEL;
    volatile uint32_t MSS_I2C_CLK_SRC_SEL;
    volatile uint32_t MSS_SCIA_CLK_SRC_SEL;
    volatile uint32_t MSS_SCIB_CLK_SRC_SEL;
    volatile uint32_t MSS_CPTS_CLK_SRC_SEL;
    volatile uint32_t MSS_CPSW_CLK_SRC_SEL;
    volatile uint32_t MSS_MCANA_CLK_DIV_VAL;
    volatile uint32_t MSS_MCANB_CLK_DIV_VAL;
    volatile uint32_t MSS_QSPI_CLK_DIV_VAL;
    volatile uint32_t MSS_RTIA_CLK_DIV_VAL;
    volatile uint32_t MSS_RTIB_CLK_DIV_VAL;
    volatile uint32_t MSS_RTIC_CLK_DIV_VAL;
    volatile uint32_t MSS_WDT_CLK_DIV_VAL;
    volatile uint32_t MSS_SPIA_CLK_DIV_VAL;
    volatile uint32_t MSS_SPIB_CLK_DIV_VAL;
    volatile uint32_t MSS_I2C_CLK_DIV_VAL;
    volatile uint32_t MSS_SCIA_CLK_DIV_VAL;
    volatile uint32_t MSS_SCIB_CLK_DIV_VAL;
    volatile uint32_t MSS_CPTS_CLK_DIV_VAL;
    volatile uint32_t MSS_CPSW_CLK_DIV_VAL;
    volatile uint32_t MSS_RGMII_CLK_DIV_VAL;
    volatile uint32_t MSS_MII100_CLK_DIV_VAL;
    volatile uint32_t MSS_MII10_CLK_DIV_VAL;
    volatile uint32_t MSS_GPADC_CLK_DIV_VAL;
    volatile uint32_t MSS_MCANA_CLK_GATE;
    volatile uint32_t MSS_MCANB_CLK_GATE;
    volatile uint32_t MSS_QSPI_CLK_GATE;
    volatile uint32_t MSS_RTIA_CLK_GATE;
    volatile uint32_t MSS_RTIB_CLK_GATE;
    volatile uint32_t MSS_RTIC_CLK_GATE;
    volatile uint32_t MSS_WDT_CLK_GATE;
    volatile uint32_t MSS_SPIA_CLK_GATE;
    volatile uint32_t MSS_SPIB_CLK_GATE;
    volatile uint32_t MSS_I2C_CLK_GATE;
    volatile uint32_t MSS_SCIA_CLK_GATE;
    volatile uint32_t MSS_SCIB_CLK_GATE;
    volatile uint32_t MSS_CPTS_CLK_GATE;
    volatile uint32_t MSS_CPSW_CLK_GATE;
    volatile uint32_t MSS_RGMII_CLK_GATE;
    volatile uint32_t MSS_MII100_CLK_GATE;
    volatile uint32_t MSS_MII10_CLK_GATE;
    volatile uint32_t MSS_GPADC_CLK_GATE;
    volatile uint32_t MSS_MCANA_CLK_STATUS;
    volatile uint32_t MSS_MCANB_CLK_STATUS;
    volatile uint32_t MSS_QSPI_CLK_STATUS;
    volatile uint32_t MSS_RTIA_CLK_STATUS;
    volatile uint32_t MSS_RTIB_CLK_STATUS;
    volatile uint32_t MSS_RTIC_CLK_STATUS;
    volatile uint32_t MSS_WDT_CLK_STATUS;
    volatile uint32_t MSS_SPIA_CLK_STATUS;
    volatile uint32_t MSS_SPIB_CLK_STATUS;
    volatile uint32_t MSS_I2C_CLK_STATUS;
    volatile uint32_t MSS_SCIA_CLK_STATUS;
    volatile uint32_t MSS_SCIB_CLK_STATUS;
    volatile uint32_t MSS_CPTS_CLK_STATUS;
    volatile uint32_t MSS_CPSW_CLK_STATUS;
    volatile uint32_t MSS_RGMII_CLK_STATUS;
    volatile uint32_t MSS_MII100_CLK_STATUS;
    volatile uint32_t MSS_MII10_CLK_STATUS;
    volatile uint32_t MSS_GPADC_CLK_STATUS;
    volatile uint32_t MSS_CR5SS_POR_RST_CTRL;
    volatile uint32_t MSS_CR5SSA_RST_CTRL;
    volatile uint32_t MSS_CR5SSB_RST_CTRL;
    volatile uint32_t MSS_CR5A_RST_CTRL;
    volatile uint32_t MSS_CR5B_RST_CTRL;
    volatile uint32_t MSS_VIMA_RST_CTRL;
    volatile uint32_t MSS_VIMB_RST_CTRL;
    volatile uint32_t MSS_CRC_RST_CTRL;
    volatile uint32_t MSS_RTIA_RST_CTRL;
    volatile uint32_t MSS_RTIB_RST_CTRL;
    volatile uint32_t MSS_RTIC_RST_CTRL;
    volatile uint32_t MSS_WDT_RST_CTRL;
    volatile uint32_t MSS_ESM_RST_CTRL;
    volatile uint32_t MSS_DCCA_RST_CTRL;
    volatile uint32_t MSS_DCCB_RST_CTRL;
    volatile uint32_t MSS_DCCC_RST_CTRL;
    volatile uint32_t MSS_DCCD_RST_CTRL;
    volatile uint32_t MSS_GIO_RST_CTRL;
    volatile uint32_t MSS_SPIA_RST_CTRL;
    volatile uint32_t MSS_SPIB_RST_CTRL;
    volatile uint32_t MSS_QSPI_RST_CTRL;
    volatile uint32_t MSS_PWM1_RST_CTRL;
    volatile uint32_t MSS_PWM2_RST_CTRL;
    volatile uint32_t MSS_PWM3_RST_CTRL;
    volatile uint32_t MSS_MCANA_RST_CTRL;
    volatile uint32_t MSS_MCANB_RST_CTRL;
    volatile uint32_t MSS_I2C_RST_CTRL;
    volatile uint32_t MSS_SCIA_RST_CTRL;
    volatile uint32_t MSS_SCIB_RST_CTRL;
    volatile uint32_t MSS_EDMA_RST_CTRL;
    volatile uint32_t MSS_INFRA_RST_CTRL;
    volatile uint32_t MSS_CPSW_RST_CTRL;
    volatile uint32_t MSS_GPADC_RST_CTRL;
    volatile uint32_t MSS_DMM_RST_CTRL;
    volatile uint32_t R5_COREA_GATE;
    volatile uint32_t R5_COREB_GATE;
    volatile uint32_t MSS_L2_BANKA_PD_CTRL;
    volatile uint32_t MSS_L2_BANKB_PD_CTRL;
    volatile uint32_t MSS_L2_BANKA_PD_STATUS;
    volatile uint32_t MSS_L2_BANKB_PD_STATUS;
    volatile uint32_t HW_REG0;
    volatile uint32_t HW_REG1;
    volatile uint32_t HW_REG2;
    volatile uint32_t HW_REG3;
    volatile uint32_t MSS_CR5F_CLK_SRC_SEL_CTRL;
    volatile uint32_t MSS_CPSW_MII_CLK_SRC_SEL;
    volatile uint32_t MSS_CPSW_MII_CLK_STATUS;
    volatile uint8_t  Resv_1024[536];
    volatile uint32_t HSM_RTIA_CLK_SRC_SEL;
    volatile uint32_t HSM_WDT_CLK_SRC_SEL;
    volatile uint32_t HSM_RTC_CLK_SRC_SEL;
    volatile uint32_t HSM_DMTA_CLK_SRC_SEL;
    volatile uint32_t HSM_DMTB_CLK_SRC_SEL;
    volatile uint32_t HSM_RTI_CLK_DIV_VAL;
    volatile uint32_t HSM_WDT_CLK_DIV_VAL;
    volatile uint32_t HSM_RTC_CLK_DIV_VAL;
    volatile uint32_t HSM_DMTA_CLK_DIV_VAL;
    volatile uint32_t HSM_DMTB_CLK_DIV_VAL;
    volatile uint32_t HSM_RTI_CLK_GATE;
    volatile uint32_t HSM_WDT_CLK_GATE;
    volatile uint32_t HSM_RTC_CLK_GATE;
    volatile uint32_t HSM_DMTA_CLK_GATE;
    volatile uint32_t HSM_DMTB_CLK_GATE;
    volatile uint32_t HSM_RTI_CLK_STATUS;
    volatile uint32_t HSM_WDT_CLK_STATUS;
    volatile uint32_t HSM_RTC_CLK_STATUS;
    volatile uint32_t HSM_DMTA_CLK_STATUS;
    volatile uint32_t HSM_DMTB_CLK_STATUS;
    volatile uint8_t  Resv_4048[2944];
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
} SDL_mss_rcmRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define SDL_MSS_RCM_PID                                                        (0x00000000U)
#define SDL_MSS_RCM_MSS_RST_CAUSE_CLR                                          (0x00000004U)
#define SDL_MSS_RCM_MSS_RST_STATUS                                             (0x00000008U)
#define SDL_MSS_RCM_SYSRST_BY_DBG_RST                                          (0x0000000CU)
#define SDL_MSS_RCM_RST_ASSERDLY                                               (0x00000010U)
#define SDL_MSS_RCM_RST2ASSERTDLY                                              (0x00000014U)
#define SDL_MSS_RCM_RST_WFICHECK                                               (0x00000018U)
#define SDL_MSS_RCM_MSS_MCANA_CLK_SRC_SEL                                      (0x0000001CU)
#define SDL_MSS_RCM_MSS_MCANB_CLK_SRC_SEL                                      (0x00000020U)
#define SDL_MSS_RCM_MSS_QSPI_CLK_SRC_SEL                                       (0x00000024U)
#define SDL_MSS_RCM_MSS_RTIA_CLK_SRC_SEL                                       (0x00000028U)
#define SDL_MSS_RCM_MSS_RTIB_CLK_SRC_SEL                                       (0x0000002CU)
#define SDL_MSS_RCM_MSS_RTIC_CLK_SRC_SEL                                       (0x00000030U)
#define SDL_MSS_RCM_MSS_WDT_CLK_SRC_SEL                                        (0x00000034U)
#define SDL_MSS_RCM_MSS_SPIA_CLK_SRC_SEL                                       (0x00000038U)
#define SDL_MSS_RCM_MSS_SPIB_CLK_SRC_SEL                                       (0x0000003CU)
#define SDL_MSS_RCM_MSS_I2C_CLK_SRC_SEL                                        (0x00000040U)
#define SDL_MSS_RCM_MSS_SCIA_CLK_SRC_SEL                                       (0x00000044U)
#define SDL_MSS_RCM_MSS_SCIB_CLK_SRC_SEL                                       (0x00000048U)
#define SDL_MSS_RCM_MSS_CPTS_CLK_SRC_SEL                                       (0x0000004CU)
#define SDL_MSS_RCM_MSS_CPSW_CLK_SRC_SEL                                       (0x00000050U)
#define SDL_MSS_RCM_MSS_MCANA_CLK_DIV_VAL                                      (0x00000054U)
#define SDL_MSS_RCM_MSS_MCANB_CLK_DIV_VAL                                      (0x00000058U)
#define SDL_MSS_RCM_MSS_QSPI_CLK_DIV_VAL                                       (0x0000005CU)
#define SDL_MSS_RCM_MSS_RTIA_CLK_DIV_VAL                                       (0x00000060U)
#define SDL_MSS_RCM_MSS_RTIB_CLK_DIV_VAL                                       (0x00000064U)
#define SDL_MSS_RCM_MSS_RTIC_CLK_DIV_VAL                                       (0x00000068U)
#define SDL_MSS_RCM_MSS_WDT_CLK_DIV_VAL                                        (0x0000006CU)
#define SDL_MSS_RCM_MSS_SPIA_CLK_DIV_VAL                                       (0x00000070U)
#define SDL_MSS_RCM_MSS_SPIB_CLK_DIV_VAL                                       (0x00000074U)
#define SDL_MSS_RCM_MSS_I2C_CLK_DIV_VAL                                        (0x00000078U)
#define SDL_MSS_RCM_MSS_SCIA_CLK_DIV_VAL                                       (0x0000007CU)
#define SDL_MSS_RCM_MSS_SCIB_CLK_DIV_VAL                                       (0x00000080U)
#define SDL_MSS_RCM_MSS_CPTS_CLK_DIV_VAL                                       (0x00000084U)
#define SDL_MSS_RCM_MSS_CPSW_CLK_DIV_VAL                                       (0x00000088U)
#define SDL_MSS_RCM_MSS_RGMII_CLK_DIV_VAL                                      (0x0000008CU)
#define SDL_MSS_RCM_MSS_MII100_CLK_DIV_VAL                                     (0x00000090U)
#define SDL_MSS_RCM_MSS_MII10_CLK_DIV_VAL                                      (0x00000094U)
#define SDL_MSS_RCM_MSS_GPADC_CLK_DIV_VAL                                      (0x00000098U)
#define SDL_MSS_RCM_MSS_MCANA_CLK_GATE                                         (0x0000009CU)
#define SDL_MSS_RCM_MSS_MCANB_CLK_GATE                                         (0x000000A0U)
#define SDL_MSS_RCM_MSS_QSPI_CLK_GATE                                          (0x000000A4U)
#define SDL_MSS_RCM_MSS_RTIA_CLK_GATE                                          (0x000000A8U)
#define SDL_MSS_RCM_MSS_RTIB_CLK_GATE                                          (0x000000ACU)
#define SDL_MSS_RCM_MSS_RTIC_CLK_GATE                                          (0x000000B0U)
#define SDL_MSS_RCM_MSS_WDT_CLK_GATE                                           (0x000000B4U)
#define SDL_MSS_RCM_MSS_SPIA_CLK_GATE                                          (0x000000B8U)
#define SDL_MSS_RCM_MSS_SPIB_CLK_GATE                                          (0x000000BCU)
#define SDL_MSS_RCM_MSS_I2C_CLK_GATE                                           (0x000000C0U)
#define SDL_MSS_RCM_MSS_SCIA_CLK_GATE                                          (0x000000C4U)
#define SDL_MSS_RCM_MSS_SCIB_CLK_GATE                                          (0x000000C8U)
#define SDL_MSS_RCM_MSS_CPTS_CLK_GATE                                          (0x000000CCU)
#define SDL_MSS_RCM_MSS_CPSW_CLK_GATE                                          (0x000000D0U)
#define SDL_MSS_RCM_MSS_RGMII_CLK_GATE                                         (0x000000D4U)
#define SDL_MSS_RCM_MSS_MII100_CLK_GATE                                        (0x000000D8U)
#define SDL_MSS_RCM_MSS_MII10_CLK_GATE                                         (0x000000DCU)
#define SDL_MSS_RCM_MSS_GPADC_CLK_GATE                                         (0x000000E0U)
#define SDL_MSS_RCM_MSS_MCANA_CLK_STATUS                                       (0x000000E4U)
#define SDL_MSS_RCM_MSS_MCANB_CLK_STATUS                                       (0x000000E8U)
#define SDL_MSS_RCM_MSS_QSPI_CLK_STATUS                                        (0x000000ECU)
#define SDL_MSS_RCM_MSS_RTIA_CLK_STATUS                                        (0x000000F0U)
#define SDL_MSS_RCM_MSS_RTIB_CLK_STATUS                                        (0x000000F4U)
#define SDL_MSS_RCM_MSS_RTIC_CLK_STATUS                                        (0x000000F8U)
#define SDL_MSS_RCM_MSS_WDT_CLK_STATUS                                         (0x000000FCU)
#define SDL_MSS_RCM_MSS_SPIA_CLK_STATUS                                        (0x00000100U)
#define SDL_MSS_RCM_MSS_SPIB_CLK_STATUS                                        (0x00000104U)
#define SDL_MSS_RCM_MSS_I2C_CLK_STATUS                                         (0x00000108U)
#define SDL_MSS_RCM_MSS_SCIA_CLK_STATUS                                        (0x0000010CU)
#define SDL_MSS_RCM_MSS_SCIB_CLK_STATUS                                        (0x00000110U)
#define SDL_MSS_RCM_MSS_CPTS_CLK_STATUS                                        (0x00000114U)
#define SDL_MSS_RCM_MSS_CPSW_CLK_STATUS                                        (0x00000118U)
#define SDL_MSS_RCM_MSS_RGMII_CLK_STATUS                                       (0x0000011CU)
#define SDL_MSS_RCM_MSS_MII100_CLK_STATUS                                      (0x00000120U)
#define SDL_MSS_RCM_MSS_MII10_CLK_STATUS                                       (0x00000124U)
#define SDL_MSS_RCM_MSS_GPADC_CLK_STATUS                                       (0x00000128U)
#define SDL_MSS_RCM_MSS_CR5SS_POR_RST_CTRL                                     (0x0000012CU)
#define SDL_MSS_RCM_MSS_CR5SSA_RST_CTRL                                        (0x00000130U)
#define SDL_MSS_RCM_MSS_CR5SSB_RST_CTRL                                        (0x00000134U)
#define SDL_MSS_RCM_MSS_CR5A_RST_CTRL                                          (0x00000138U)
#define SDL_MSS_RCM_MSS_CR5B_RST_CTRL                                          (0x0000013CU)
#define SDL_MSS_RCM_MSS_VIMA_RST_CTRL                                          (0x00000140U)
#define SDL_MSS_RCM_MSS_VIMB_RST_CTRL                                          (0x00000144U)
#define SDL_MSS_RCM_MSS_CRC_RST_CTRL                                           (0x00000148U)
#define SDL_MSS_RCM_MSS_RTIA_RST_CTRL                                          (0x0000014CU)
#define SDL_MSS_RCM_MSS_RTIB_RST_CTRL                                          (0x00000150U)
#define SDL_MSS_RCM_MSS_RTIC_RST_CTRL                                          (0x00000154U)
#define SDL_MSS_RCM_MSS_WDT_RST_CTRL                                           (0x00000158U)
#define SDL_MSS_RCM_MSS_ESM_RST_CTRL                                           (0x0000015CU)
#define SDL_MSS_RCM_MSS_DCCA_RST_CTRL                                          (0x00000160U)
#define SDL_MSS_RCM_MSS_DCCB_RST_CTRL                                          (0x00000164U)
#define SDL_MSS_RCM_MSS_DCCC_RST_CTRL                                          (0x00000168U)
#define SDL_MSS_RCM_MSS_DCCD_RST_CTRL                                          (0x0000016CU)
#define SDL_MSS_RCM_MSS_GIO_RST_CTRL                                           (0x00000170U)
#define SDL_MSS_RCM_MSS_SPIA_RST_CTRL                                          (0x00000174U)
#define SDL_MSS_RCM_MSS_SPIB_RST_CTRL                                          (0x00000178U)
#define SDL_MSS_RCM_MSS_QSPI_RST_CTRL                                          (0x0000017CU)
#define SDL_MSS_RCM_MSS_PWM1_RST_CTRL                                          (0x00000180U)
#define SDL_MSS_RCM_MSS_PWM2_RST_CTRL                                          (0x00000184U)
#define SDL_MSS_RCM_MSS_PWM3_RST_CTRL                                          (0x00000188U)
#define SDL_MSS_RCM_MSS_MCANA_RST_CTRL                                         (0x0000018CU)
#define SDL_MSS_RCM_MSS_MCANB_RST_CTRL                                         (0x00000190U)
#define SDL_MSS_RCM_MSS_I2C_RST_CTRL                                           (0x00000194U)
#define SDL_MSS_RCM_MSS_SCIA_RST_CTRL                                          (0x00000198U)
#define SDL_MSS_RCM_MSS_SCIB_RST_CTRL                                          (0x0000019CU)
#define SDL_MSS_RCM_MSS_EDMA_RST_CTRL                                          (0x000001A0U)
#define SDL_MSS_RCM_MSS_INFRA_RST_CTRL                                         (0x000001A4U)
#define SDL_MSS_RCM_MSS_CPSW_RST_CTRL                                          (0x000001A8U)
#define SDL_MSS_RCM_MSS_GPADC_RST_CTRL                                         (0x000001ACU)
#define SDL_MSS_RCM_MSS_DMM_RST_CTRL                                           (0x000001B0U)
#define SDL_MSS_RCM_R5_COREA_GATE                                              (0x000001B4U)
#define SDL_MSS_RCM_R5_COREB_GATE                                              (0x000001B8U)
#define SDL_MSS_RCM_MSS_L2_BANKA_PD_CTRL                                       (0x000001BCU)
#define SDL_MSS_RCM_MSS_L2_BANKB_PD_CTRL                                       (0x000001C0U)
#define SDL_MSS_RCM_MSS_L2_BANKA_PD_STATUS                                     (0x000001C4U)
#define SDL_MSS_RCM_MSS_L2_BANKB_PD_STATUS                                     (0x000001C8U)
#define SDL_MSS_RCM_HW_REG0                                                    (0x000001CCU)
#define SDL_MSS_RCM_HW_REG1                                                    (0x000001D0U)
#define SDL_MSS_RCM_HW_REG2                                                    (0x000001D4U)
#define SDL_MSS_RCM_HW_REG3                                                    (0x000001D8U)
#define SDL_MSS_RCM_MSS_CR5F_CLK_SRC_SEL_CTRL                                  (0x000001DCU)
#define SDL_MSS_RCM_MSS_CPSW_MII_CLK_SRC_SEL                                   (0x000001E0U)
#define SDL_MSS_RCM_MSS_CPSW_MII_CLK_STATUS                                    (0x000001E4U)
#define SDL_MSS_RCM_HSM_RTIA_CLK_SRC_SEL                                       (0x00000400U)
#define SDL_MSS_RCM_HSM_WDT_CLK_SRC_SEL                                        (0x00000404U)
#define SDL_MSS_RCM_HSM_RTC_CLK_SRC_SEL                                        (0x00000408U)
#define SDL_MSS_RCM_HSM_DMTA_CLK_SRC_SEL                                       (0x0000040CU)
#define SDL_MSS_RCM_HSM_DMTB_CLK_SRC_SEL                                       (0x00000410U)
#define SDL_MSS_RCM_HSM_RTI_CLK_DIV_VAL                                        (0x00000414U)
#define SDL_MSS_RCM_HSM_WDT_CLK_DIV_VAL                                        (0x00000418U)
#define SDL_MSS_RCM_HSM_RTC_CLK_DIV_VAL                                        (0x0000041CU)
#define SDL_MSS_RCM_HSM_DMTA_CLK_DIV_VAL                                       (0x00000420U)
#define SDL_MSS_RCM_HSM_DMTB_CLK_DIV_VAL                                       (0x00000424U)
#define SDL_MSS_RCM_HSM_RTI_CLK_GATE                                           (0x00000428U)
#define SDL_MSS_RCM_HSM_WDT_CLK_GATE                                           (0x0000042CU)
#define SDL_MSS_RCM_HSM_RTC_CLK_GATE                                           (0x00000430U)
#define SDL_MSS_RCM_HSM_DMTA_CLK_GATE                                          (0x00000434U)
#define SDL_MSS_RCM_HSM_DMTB_CLK_GATE                                          (0x00000438U)
#define SDL_MSS_RCM_HSM_RTI_CLK_STATUS                                         (0x0000043CU)
#define SDL_MSS_RCM_HSM_WDT_CLK_STATUS                                         (0x00000440U)
#define SDL_MSS_RCM_HSM_RTC_CLK_STATUS                                         (0x00000444U)
#define SDL_MSS_RCM_HSM_DMTA_CLK_STATUS                                        (0x00000448U)
#define SDL_MSS_RCM_HSM_DMTB_CLK_STATUS                                        (0x0000044CU)
#define SDL_MSS_RCM_HW_SPARE_RW0                                               (0x00000FD0U)
#define SDL_MSS_RCM_HW_SPARE_RW1                                               (0x00000FD4U)
#define SDL_MSS_RCM_HW_SPARE_RW2                                               (0x00000FD8U)
#define SDL_MSS_RCM_HW_SPARE_RW3                                               (0x00000FDCU)
#define SDL_MSS_RCM_HW_SPARE_RO0                                               (0x00000FE0U)
#define SDL_MSS_RCM_HW_SPARE_RO1                                               (0x00000FE4U)
#define SDL_MSS_RCM_HW_SPARE_RO2                                               (0x00000FE8U)
#define SDL_MSS_RCM_HW_SPARE_RO3                                               (0x00000FECU)
#define SDL_MSS_RCM_HW_SPARE_WPH                                               (0x00000FF0U)
#define SDL_MSS_RCM_HW_SPARE_REC                                               (0x00000FF4U)
#define SDL_MSS_RCM_LOCK0_KICK0                                                (0x00001008U)
#define SDL_MSS_RCM_LOCK0_KICK1                                                (0x0000100CU)
#define SDL_MSS_RCM_INTR_RAW_STATUS                                            (0x00001010U)
#define SDL_MSS_RCM_INTR_ENABLED_STATUS_CLEAR                                  (0x00001014U)
#define SDL_MSS_RCM_INTR_ENABLE                                                (0x00001018U)
#define SDL_MSS_RCM_INTR_ENABLE_CLEAR                                          (0x0000101CU)
#define SDL_MSS_RCM_EOI                                                        (0x00001020U)
#define SDL_MSS_RCM_FAULT_ADDRESS                                              (0x00001024U)
#define SDL_MSS_RCM_FAULT_TYPE_STATUS                                          (0x00001028U)
#define SDL_MSS_RCM_FAULT_ATTR_STATUS                                          (0x0000102CU)
#define SDL_MSS_RCM_FAULT_CLEAR                                                (0x00001030U)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* PID */

#define SDL_MSS_RCM_PID_PID_MINOR_MASK                                         (0x0000003FU)
#define SDL_MSS_RCM_PID_PID_MINOR_SHIFT                                        (0x00000000U)
#define SDL_MSS_RCM_PID_PID_MINOR_RESETVAL                                     (0x00000014U)
#define SDL_MSS_RCM_PID_PID_MINOR_MAX                                          (0x0000003FU)

#define SDL_MSS_RCM_PID_PID_CUSTOM_MASK                                        (0x000000C0U)
#define SDL_MSS_RCM_PID_PID_CUSTOM_SHIFT                                       (0x00000006U)
#define SDL_MSS_RCM_PID_PID_CUSTOM_RESETVAL                                    (0x00000000U)
#define SDL_MSS_RCM_PID_PID_CUSTOM_MAX                                         (0x00000003U)

#define SDL_MSS_RCM_PID_PID_MAJOR_MASK                                         (0x00000700U)
#define SDL_MSS_RCM_PID_PID_MAJOR_SHIFT                                        (0x00000008U)
#define SDL_MSS_RCM_PID_PID_MAJOR_RESETVAL                                     (0x00000002U)
#define SDL_MSS_RCM_PID_PID_MAJOR_MAX                                          (0x00000007U)

#define SDL_MSS_RCM_PID_PID_MISC_MASK                                          (0x0000F800U)
#define SDL_MSS_RCM_PID_PID_MISC_SHIFT                                         (0x0000000BU)
#define SDL_MSS_RCM_PID_PID_MISC_RESETVAL                                      (0x00000000U)
#define SDL_MSS_RCM_PID_PID_MISC_MAX                                           (0x0000001FU)

#define SDL_MSS_RCM_PID_PID_MSB16_MASK                                         (0xFFFF0000U)
#define SDL_MSS_RCM_PID_PID_MSB16_SHIFT                                        (0x00000010U)
#define SDL_MSS_RCM_PID_PID_MSB16_RESETVAL                                     (0x00006180U)
#define SDL_MSS_RCM_PID_PID_MSB16_MAX                                          (0x0000FFFFU)

#define SDL_MSS_RCM_PID_RESETVAL                                               (0x61800214U)

/* MSS_RST_CAUSE_CLR */

#define SDL_MSS_RCM_MSS_RST_CAUSE_CLR_MSS_RST_CAUSE_CLR_CLR_MASK               (0x00000007U)
#define SDL_MSS_RCM_MSS_RST_CAUSE_CLR_MSS_RST_CAUSE_CLR_CLR_SHIFT              (0x00000000U)
#define SDL_MSS_RCM_MSS_RST_CAUSE_CLR_MSS_RST_CAUSE_CLR_CLR_RESETVAL           (0x00000000U)
#define SDL_MSS_RCM_MSS_RST_CAUSE_CLR_MSS_RST_CAUSE_CLR_CLR_MAX                (0x00000007U)

#define SDL_MSS_RCM_MSS_RST_CAUSE_CLR_RESETVAL                                 (0x00000000U)

/* MSS_RST_STATUS */

#define SDL_MSS_RCM_MSS_RST_STATUS_MSS_RST_STATUS_CAUSE_MASK                   (0x0000FFFFU)
#define SDL_MSS_RCM_MSS_RST_STATUS_MSS_RST_STATUS_CAUSE_SHIFT                  (0x00000000U)
#define SDL_MSS_RCM_MSS_RST_STATUS_MSS_RST_STATUS_CAUSE_RESETVAL               (0x00000003U)
#define SDL_MSS_RCM_MSS_RST_STATUS_MSS_RST_STATUS_CAUSE_MAX                    (0x0000FFFFU)

#define SDL_MSS_RCM_MSS_RST_STATUS_RESETVAL                                    (0x00000003U)

/* SYSRST_BY_DBG_RST */

#define SDL_MSS_RCM_SYSRST_BY_DBG_RST_SYSRST_BY_DBG_RST_R5A_MASK               (0x00000007U)
#define SDL_MSS_RCM_SYSRST_BY_DBG_RST_SYSRST_BY_DBG_RST_R5A_SHIFT              (0x00000000U)
#define SDL_MSS_RCM_SYSRST_BY_DBG_RST_SYSRST_BY_DBG_RST_R5A_RESETVAL           (0x00000000U)
#define SDL_MSS_RCM_SYSRST_BY_DBG_RST_SYSRST_BY_DBG_RST_R5A_MAX                (0x00000007U)

#define SDL_MSS_RCM_SYSRST_BY_DBG_RST_SYSRST_BY_DBG_RST_R5B_MASK               (0x00070000U)
#define SDL_MSS_RCM_SYSRST_BY_DBG_RST_SYSRST_BY_DBG_RST_R5B_SHIFT              (0x00000010U)
#define SDL_MSS_RCM_SYSRST_BY_DBG_RST_SYSRST_BY_DBG_RST_R5B_RESETVAL           (0x00000000U)
#define SDL_MSS_RCM_SYSRST_BY_DBG_RST_SYSRST_BY_DBG_RST_R5B_MAX                (0x00000007U)

#define SDL_MSS_RCM_SYSRST_BY_DBG_RST_RESETVAL                                 (0x00000000U)

/* RST_ASSERDLY */

#define SDL_MSS_RCM_RST_ASSERDLY_RST_ASSERDLY_COMMON_MASK                      (0x000000FFU)
#define SDL_MSS_RCM_RST_ASSERDLY_RST_ASSERDLY_COMMON_SHIFT                     (0x00000000U)
#define SDL_MSS_RCM_RST_ASSERDLY_RST_ASSERDLY_COMMON_RESETVAL                  (0x0000000FU)
#define SDL_MSS_RCM_RST_ASSERDLY_RST_ASSERDLY_COMMON_MAX                       (0x000000FFU)

#define SDL_MSS_RCM_RST_ASSERDLY_RESETVAL                                      (0x0000000FU)

/* RST2ASSERTDLY */

#define SDL_MSS_RCM_RST2ASSERTDLY_RST2ASSERTDLY_R5SSA_MASK                     (0x000000FFU)
#define SDL_MSS_RCM_RST2ASSERTDLY_RST2ASSERTDLY_R5SSA_SHIFT                    (0x00000000U)
#define SDL_MSS_RCM_RST2ASSERTDLY_RST2ASSERTDLY_R5SSA_RESETVAL                 (0x00000000U)
#define SDL_MSS_RCM_RST2ASSERTDLY_RST2ASSERTDLY_R5SSA_MAX                      (0x000000FFU)

#define SDL_MSS_RCM_RST2ASSERTDLY_RST2ASSERTDLY_R5SSB_MASK                     (0x0000FF00U)
#define SDL_MSS_RCM_RST2ASSERTDLY_RST2ASSERTDLY_R5SSB_SHIFT                    (0x00000008U)
#define SDL_MSS_RCM_RST2ASSERTDLY_RST2ASSERTDLY_R5SSB_RESETVAL                 (0x00000000U)
#define SDL_MSS_RCM_RST2ASSERTDLY_RST2ASSERTDLY_R5SSB_MAX                      (0x000000FFU)

#define SDL_MSS_RCM_RST2ASSERTDLY_RST2ASSERTDLY_R5A_MASK                       (0x00FF0000U)
#define SDL_MSS_RCM_RST2ASSERTDLY_RST2ASSERTDLY_R5A_SHIFT                      (0x00000010U)
#define SDL_MSS_RCM_RST2ASSERTDLY_RST2ASSERTDLY_R5A_RESETVAL                   (0x00000000U)
#define SDL_MSS_RCM_RST2ASSERTDLY_RST2ASSERTDLY_R5A_MAX                        (0x000000FFU)

#define SDL_MSS_RCM_RST2ASSERTDLY_RST2ASSERTDLY_R5B_MASK                       (0xFF000000U)
#define SDL_MSS_RCM_RST2ASSERTDLY_RST2ASSERTDLY_R5B_SHIFT                      (0x00000018U)
#define SDL_MSS_RCM_RST2ASSERTDLY_RST2ASSERTDLY_R5B_RESETVAL                   (0x00000000U)
#define SDL_MSS_RCM_RST2ASSERTDLY_RST2ASSERTDLY_R5B_MAX                        (0x000000FFU)

#define SDL_MSS_RCM_RST2ASSERTDLY_RESETVAL                                     (0x00000000U)

/* RST_WFICHECK */

#define SDL_MSS_RCM_RST_WFICHECK_RST_WFICHECK_R5SSA_MASK                       (0x00000007U)
#define SDL_MSS_RCM_RST_WFICHECK_RST_WFICHECK_R5SSA_SHIFT                      (0x00000000U)
#define SDL_MSS_RCM_RST_WFICHECK_RST_WFICHECK_R5SSA_RESETVAL                   (0x00000007U)
#define SDL_MSS_RCM_RST_WFICHECK_RST_WFICHECK_R5SSA_MAX                        (0x00000007U)

#define SDL_MSS_RCM_RST_WFICHECK_RST_WFICHECK_R5SSB_MASK                       (0x00000700U)
#define SDL_MSS_RCM_RST_WFICHECK_RST_WFICHECK_R5SSB_SHIFT                      (0x00000008U)
#define SDL_MSS_RCM_RST_WFICHECK_RST_WFICHECK_R5SSB_RESETVAL                   (0x00000007U)
#define SDL_MSS_RCM_RST_WFICHECK_RST_WFICHECK_R5SSB_MAX                        (0x00000007U)

#define SDL_MSS_RCM_RST_WFICHECK_RST_WFICHECK_R5A_MASK                         (0x00070000U)
#define SDL_MSS_RCM_RST_WFICHECK_RST_WFICHECK_R5A_SHIFT                        (0x00000010U)
#define SDL_MSS_RCM_RST_WFICHECK_RST_WFICHECK_R5A_RESETVAL                     (0x00000007U)
#define SDL_MSS_RCM_RST_WFICHECK_RST_WFICHECK_R5A_MAX                          (0x00000007U)

#define SDL_MSS_RCM_RST_WFICHECK_RST_WFICHECK_R5B_MASK                         (0x07000000U)
#define SDL_MSS_RCM_RST_WFICHECK_RST_WFICHECK_R5B_SHIFT                        (0x00000018U)
#define SDL_MSS_RCM_RST_WFICHECK_RST_WFICHECK_R5B_RESETVAL                     (0x00000007U)
#define SDL_MSS_RCM_RST_WFICHECK_RST_WFICHECK_R5B_MAX                          (0x00000007U)

#define SDL_MSS_RCM_RST_WFICHECK_RESETVAL                                      (0x07070707U)

/* MSS_MCANA_CLK_SRC_SEL */

#define SDL_MSS_RCM_MSS_MCANA_CLK_SRC_SEL_MSS_MCANA_CLK_SRC_SEL_CLKSRCSEL_MASK (0x00000FFFU)
#define SDL_MSS_RCM_MSS_MCANA_CLK_SRC_SEL_MSS_MCANA_CLK_SRC_SEL_CLKSRCSEL_SHIFT (0x00000000U)
#define SDL_MSS_RCM_MSS_MCANA_CLK_SRC_SEL_MSS_MCANA_CLK_SRC_SEL_CLKSRCSEL_RESETVAL (0x00000000U)
#define SDL_MSS_RCM_MSS_MCANA_CLK_SRC_SEL_MSS_MCANA_CLK_SRC_SEL_CLKSRCSEL_MAX  (0x00000FFFU)

#define SDL_MSS_RCM_MSS_MCANA_CLK_SRC_SEL_RESETVAL                             (0x00000000U)

/* MSS_MCANB_CLK_SRC_SEL */

#define SDL_MSS_RCM_MSS_MCANB_CLK_SRC_SEL_MSS_MCANB_CLK_SRC_SEL_CLKSRCSEL_MASK (0x00000FFFU)
#define SDL_MSS_RCM_MSS_MCANB_CLK_SRC_SEL_MSS_MCANB_CLK_SRC_SEL_CLKSRCSEL_SHIFT (0x00000000U)
#define SDL_MSS_RCM_MSS_MCANB_CLK_SRC_SEL_MSS_MCANB_CLK_SRC_SEL_CLKSRCSEL_RESETVAL (0x00000000U)
#define SDL_MSS_RCM_MSS_MCANB_CLK_SRC_SEL_MSS_MCANB_CLK_SRC_SEL_CLKSRCSEL_MAX  (0x00000FFFU)

#define SDL_MSS_RCM_MSS_MCANB_CLK_SRC_SEL_RESETVAL                             (0x00000000U)

/* MSS_QSPI_CLK_SRC_SEL */

#define SDL_MSS_RCM_MSS_QSPI_CLK_SRC_SEL_MSS_QSPI_CLK_SRC_SEL_CLKSRCSEL_MASK   (0x00000FFFU)
#define SDL_MSS_RCM_MSS_QSPI_CLK_SRC_SEL_MSS_QSPI_CLK_SRC_SEL_CLKSRCSEL_SHIFT  (0x00000000U)
#define SDL_MSS_RCM_MSS_QSPI_CLK_SRC_SEL_MSS_QSPI_CLK_SRC_SEL_CLKSRCSEL_RESETVAL (0x00000000U)
#define SDL_MSS_RCM_MSS_QSPI_CLK_SRC_SEL_MSS_QSPI_CLK_SRC_SEL_CLKSRCSEL_MAX    (0x00000FFFU)

#define SDL_MSS_RCM_MSS_QSPI_CLK_SRC_SEL_RESETVAL                              (0x00000000U)

/* MSS_RTIA_CLK_SRC_SEL */

#define SDL_MSS_RCM_MSS_RTIA_CLK_SRC_SEL_MSS_RTIA_CLK_SRC_SEL_CLKSRCSEL_MASK   (0x00000FFFU)
#define SDL_MSS_RCM_MSS_RTIA_CLK_SRC_SEL_MSS_RTIA_CLK_SRC_SEL_CLKSRCSEL_SHIFT  (0x00000000U)
#define SDL_MSS_RCM_MSS_RTIA_CLK_SRC_SEL_MSS_RTIA_CLK_SRC_SEL_CLKSRCSEL_RESETVAL (0x00000000U)
#define SDL_MSS_RCM_MSS_RTIA_CLK_SRC_SEL_MSS_RTIA_CLK_SRC_SEL_CLKSRCSEL_MAX    (0x00000FFFU)

#define SDL_MSS_RCM_MSS_RTIA_CLK_SRC_SEL_RESETVAL                              (0x00000000U)

/* MSS_RTIB_CLK_SRC_SEL */

#define SDL_MSS_RCM_MSS_RTIB_CLK_SRC_SEL_MSS_RTIB_CLK_SRC_SEL_CLKSRCSEL_MASK   (0x00000FFFU)
#define SDL_MSS_RCM_MSS_RTIB_CLK_SRC_SEL_MSS_RTIB_CLK_SRC_SEL_CLKSRCSEL_SHIFT  (0x00000000U)
#define SDL_MSS_RCM_MSS_RTIB_CLK_SRC_SEL_MSS_RTIB_CLK_SRC_SEL_CLKSRCSEL_RESETVAL (0x00000000U)
#define SDL_MSS_RCM_MSS_RTIB_CLK_SRC_SEL_MSS_RTIB_CLK_SRC_SEL_CLKSRCSEL_MAX    (0x00000FFFU)

#define SDL_MSS_RCM_MSS_RTIB_CLK_SRC_SEL_RESETVAL                              (0x00000000U)

/* MSS_RTIC_CLK_SRC_SEL */

#define SDL_MSS_RCM_MSS_RTIC_CLK_SRC_SEL_MSS_RTIC_CLK_SRC_SEL_CLKSRCSEL_MASK   (0x00000FFFU)
#define SDL_MSS_RCM_MSS_RTIC_CLK_SRC_SEL_MSS_RTIC_CLK_SRC_SEL_CLKSRCSEL_SHIFT  (0x00000000U)
#define SDL_MSS_RCM_MSS_RTIC_CLK_SRC_SEL_MSS_RTIC_CLK_SRC_SEL_CLKSRCSEL_RESETVAL (0x00000000U)
#define SDL_MSS_RCM_MSS_RTIC_CLK_SRC_SEL_MSS_RTIC_CLK_SRC_SEL_CLKSRCSEL_MAX    (0x00000FFFU)

#define SDL_MSS_RCM_MSS_RTIC_CLK_SRC_SEL_RESETVAL                              (0x00000000U)

/* MSS_WDT_CLK_SRC_SEL */

#define SDL_MSS_RCM_MSS_WDT_CLK_SRC_SEL_MSS_WDT_CLK_SRC_SEL_CLKSRCSEL_MASK     (0x00000FFFU)
#define SDL_MSS_RCM_MSS_WDT_CLK_SRC_SEL_MSS_WDT_CLK_SRC_SEL_CLKSRCSEL_SHIFT    (0x00000000U)
#define SDL_MSS_RCM_MSS_WDT_CLK_SRC_SEL_MSS_WDT_CLK_SRC_SEL_CLKSRCSEL_RESETVAL (0x00000000U)
#define SDL_MSS_RCM_MSS_WDT_CLK_SRC_SEL_MSS_WDT_CLK_SRC_SEL_CLKSRCSEL_MAX      (0x00000FFFU)

#define SDL_MSS_RCM_MSS_WDT_CLK_SRC_SEL_RESETVAL                               (0x00000000U)

/* MSS_SPIA_CLK_SRC_SEL */

#define SDL_MSS_RCM_MSS_SPIA_CLK_SRC_SEL_MSS_SPIA_CLK_SRC_SEL_CLKSRCSEL_MASK   (0x00000FFFU)
#define SDL_MSS_RCM_MSS_SPIA_CLK_SRC_SEL_MSS_SPIA_CLK_SRC_SEL_CLKSRCSEL_SHIFT  (0x00000000U)
#define SDL_MSS_RCM_MSS_SPIA_CLK_SRC_SEL_MSS_SPIA_CLK_SRC_SEL_CLKSRCSEL_RESETVAL (0x00000000U)
#define SDL_MSS_RCM_MSS_SPIA_CLK_SRC_SEL_MSS_SPIA_CLK_SRC_SEL_CLKSRCSEL_MAX    (0x00000FFFU)

#define SDL_MSS_RCM_MSS_SPIA_CLK_SRC_SEL_RESETVAL                              (0x00000000U)

/* MSS_SPIB_CLK_SRC_SEL */

#define SDL_MSS_RCM_MSS_SPIB_CLK_SRC_SEL_MSS_SPIB_CLK_SRC_SEL_CLKSRCSEL_MASK   (0x00000FFFU)
#define SDL_MSS_RCM_MSS_SPIB_CLK_SRC_SEL_MSS_SPIB_CLK_SRC_SEL_CLKSRCSEL_SHIFT  (0x00000000U)
#define SDL_MSS_RCM_MSS_SPIB_CLK_SRC_SEL_MSS_SPIB_CLK_SRC_SEL_CLKSRCSEL_RESETVAL (0x00000000U)
#define SDL_MSS_RCM_MSS_SPIB_CLK_SRC_SEL_MSS_SPIB_CLK_SRC_SEL_CLKSRCSEL_MAX    (0x00000FFFU)

#define SDL_MSS_RCM_MSS_SPIB_CLK_SRC_SEL_RESETVAL                              (0x00000000U)

/* MSS_I2C_CLK_SRC_SEL */

#define SDL_MSS_RCM_MSS_I2C_CLK_SRC_SEL_MSS_I2C_CLK_SRC_SEL_CLKSRCSEL_MASK     (0x00000FFFU)
#define SDL_MSS_RCM_MSS_I2C_CLK_SRC_SEL_MSS_I2C_CLK_SRC_SEL_CLKSRCSEL_SHIFT    (0x00000000U)
#define SDL_MSS_RCM_MSS_I2C_CLK_SRC_SEL_MSS_I2C_CLK_SRC_SEL_CLKSRCSEL_RESETVAL (0x00000000U)
#define SDL_MSS_RCM_MSS_I2C_CLK_SRC_SEL_MSS_I2C_CLK_SRC_SEL_CLKSRCSEL_MAX      (0x00000FFFU)

#define SDL_MSS_RCM_MSS_I2C_CLK_SRC_SEL_RESETVAL                               (0x00000000U)

/* MSS_SCIA_CLK_SRC_SEL */

#define SDL_MSS_RCM_MSS_SCIA_CLK_SRC_SEL_MSS_SCIA_CLK_SRC_SEL_CLKSRCSEL_MASK   (0x00000FFFU)
#define SDL_MSS_RCM_MSS_SCIA_CLK_SRC_SEL_MSS_SCIA_CLK_SRC_SEL_CLKSRCSEL_SHIFT  (0x00000000U)
#define SDL_MSS_RCM_MSS_SCIA_CLK_SRC_SEL_MSS_SCIA_CLK_SRC_SEL_CLKSRCSEL_RESETVAL (0x00000000U)
#define SDL_MSS_RCM_MSS_SCIA_CLK_SRC_SEL_MSS_SCIA_CLK_SRC_SEL_CLKSRCSEL_MAX    (0x00000FFFU)

#define SDL_MSS_RCM_MSS_SCIA_CLK_SRC_SEL_RESETVAL                              (0x00000000U)

/* MSS_SCIB_CLK_SRC_SEL */

#define SDL_MSS_RCM_MSS_SCIB_CLK_SRC_SEL_MSS_SCIB_CLK_SRC_SEL_CLKSRCSEL_MASK   (0x00000FFFU)
#define SDL_MSS_RCM_MSS_SCIB_CLK_SRC_SEL_MSS_SCIB_CLK_SRC_SEL_CLKSRCSEL_SHIFT  (0x00000000U)
#define SDL_MSS_RCM_MSS_SCIB_CLK_SRC_SEL_MSS_SCIB_CLK_SRC_SEL_CLKSRCSEL_RESETVAL (0x00000000U)
#define SDL_MSS_RCM_MSS_SCIB_CLK_SRC_SEL_MSS_SCIB_CLK_SRC_SEL_CLKSRCSEL_MAX    (0x00000FFFU)

#define SDL_MSS_RCM_MSS_SCIB_CLK_SRC_SEL_RESETVAL                              (0x00000000U)

/* MSS_CPTS_CLK_SRC_SEL */

#define SDL_MSS_RCM_MSS_CPTS_CLK_SRC_SEL_MSS_CPTS_CLK_SRC_SEL_CLKSRCSEL_MASK   (0x00000FFFU)
#define SDL_MSS_RCM_MSS_CPTS_CLK_SRC_SEL_MSS_CPTS_CLK_SRC_SEL_CLKSRCSEL_SHIFT  (0x00000000U)
#define SDL_MSS_RCM_MSS_CPTS_CLK_SRC_SEL_MSS_CPTS_CLK_SRC_SEL_CLKSRCSEL_RESETVAL (0x00000000U)
#define SDL_MSS_RCM_MSS_CPTS_CLK_SRC_SEL_MSS_CPTS_CLK_SRC_SEL_CLKSRCSEL_MAX    (0x00000FFFU)

#define SDL_MSS_RCM_MSS_CPTS_CLK_SRC_SEL_RESETVAL                              (0x00000000U)

/* MSS_CPSW_CLK_SRC_SEL */

#define SDL_MSS_RCM_MSS_CPSW_CLK_SRC_SEL_MSS_CPSW_CLK_SRC_SEL_CLKSRCSEL_MASK   (0x00000FFFU)
#define SDL_MSS_RCM_MSS_CPSW_CLK_SRC_SEL_MSS_CPSW_CLK_SRC_SEL_CLKSRCSEL_SHIFT  (0x00000000U)
#define SDL_MSS_RCM_MSS_CPSW_CLK_SRC_SEL_MSS_CPSW_CLK_SRC_SEL_CLKSRCSEL_RESETVAL (0x00000000U)
#define SDL_MSS_RCM_MSS_CPSW_CLK_SRC_SEL_MSS_CPSW_CLK_SRC_SEL_CLKSRCSEL_MAX    (0x00000FFFU)

#define SDL_MSS_RCM_MSS_CPSW_CLK_SRC_SEL_RESETVAL                              (0x00000000U)

/* MSS_MCANA_CLK_DIV_VAL */

#define SDL_MSS_RCM_MSS_MCANA_CLK_DIV_VAL_MSS_MCANA_CLK_DIV_VAL_CLKDIVR_MASK   (0x00000FFFU)
#define SDL_MSS_RCM_MSS_MCANA_CLK_DIV_VAL_MSS_MCANA_CLK_DIV_VAL_CLKDIVR_SHIFT  (0x00000000U)
#define SDL_MSS_RCM_MSS_MCANA_CLK_DIV_VAL_MSS_MCANA_CLK_DIV_VAL_CLKDIVR_RESETVAL (0x00000000U)
#define SDL_MSS_RCM_MSS_MCANA_CLK_DIV_VAL_MSS_MCANA_CLK_DIV_VAL_CLKDIVR_MAX    (0x00000FFFU)

#define SDL_MSS_RCM_MSS_MCANA_CLK_DIV_VAL_RESETVAL                             (0x00000000U)

/* MSS_MCANB_CLK_DIV_VAL */

#define SDL_MSS_RCM_MSS_MCANB_CLK_DIV_VAL_MSS_MCANB_CLK_DIV_VAL_CLKDIVR_MASK   (0x00000FFFU)
#define SDL_MSS_RCM_MSS_MCANB_CLK_DIV_VAL_MSS_MCANB_CLK_DIV_VAL_CLKDIVR_SHIFT  (0x00000000U)
#define SDL_MSS_RCM_MSS_MCANB_CLK_DIV_VAL_MSS_MCANB_CLK_DIV_VAL_CLKDIVR_RESETVAL (0x00000000U)
#define SDL_MSS_RCM_MSS_MCANB_CLK_DIV_VAL_MSS_MCANB_CLK_DIV_VAL_CLKDIVR_MAX    (0x00000FFFU)

#define SDL_MSS_RCM_MSS_MCANB_CLK_DIV_VAL_RESETVAL                             (0x00000000U)

/* MSS_QSPI_CLK_DIV_VAL */

#define SDL_MSS_RCM_MSS_QSPI_CLK_DIV_VAL_MSS_QSPI_CLK_DIV_VAL_CLKDIVR_MASK     (0x00000FFFU)
#define SDL_MSS_RCM_MSS_QSPI_CLK_DIV_VAL_MSS_QSPI_CLK_DIV_VAL_CLKDIVR_SHIFT    (0x00000000U)
#define SDL_MSS_RCM_MSS_QSPI_CLK_DIV_VAL_MSS_QSPI_CLK_DIV_VAL_CLKDIVR_RESETVAL (0x00000000U)
#define SDL_MSS_RCM_MSS_QSPI_CLK_DIV_VAL_MSS_QSPI_CLK_DIV_VAL_CLKDIVR_MAX      (0x00000FFFU)

#define SDL_MSS_RCM_MSS_QSPI_CLK_DIV_VAL_RESETVAL                              (0x00000000U)

/* MSS_RTIA_CLK_DIV_VAL */

#define SDL_MSS_RCM_MSS_RTIA_CLK_DIV_VAL_MSS_RTIA_CLK_DIV_VAL_CLKDIVR_MASK     (0x00000FFFU)
#define SDL_MSS_RCM_MSS_RTIA_CLK_DIV_VAL_MSS_RTIA_CLK_DIV_VAL_CLKDIVR_SHIFT    (0x00000000U)
#define SDL_MSS_RCM_MSS_RTIA_CLK_DIV_VAL_MSS_RTIA_CLK_DIV_VAL_CLKDIVR_RESETVAL (0x00000000U)
#define SDL_MSS_RCM_MSS_RTIA_CLK_DIV_VAL_MSS_RTIA_CLK_DIV_VAL_CLKDIVR_MAX      (0x00000FFFU)

#define SDL_MSS_RCM_MSS_RTIA_CLK_DIV_VAL_RESETVAL                              (0x00000000U)

/* MSS_RTIB_CLK_DIV_VAL */

#define SDL_MSS_RCM_MSS_RTIB_CLK_DIV_VAL_MSS_RTIB_CLK_DIV_VAL_CLKDIVR_MASK     (0x00000FFFU)
#define SDL_MSS_RCM_MSS_RTIB_CLK_DIV_VAL_MSS_RTIB_CLK_DIV_VAL_CLKDIVR_SHIFT    (0x00000000U)
#define SDL_MSS_RCM_MSS_RTIB_CLK_DIV_VAL_MSS_RTIB_CLK_DIV_VAL_CLKDIVR_RESETVAL (0x00000000U)
#define SDL_MSS_RCM_MSS_RTIB_CLK_DIV_VAL_MSS_RTIB_CLK_DIV_VAL_CLKDIVR_MAX      (0x00000FFFU)

#define SDL_MSS_RCM_MSS_RTIB_CLK_DIV_VAL_RESETVAL                              (0x00000000U)

/* MSS_RTIC_CLK_DIV_VAL */

#define SDL_MSS_RCM_MSS_RTIC_CLK_DIV_VAL_MSS_RTIC_CLK_DIV_VAL_CLKDIVR_MASK     (0x00000FFFU)
#define SDL_MSS_RCM_MSS_RTIC_CLK_DIV_VAL_MSS_RTIC_CLK_DIV_VAL_CLKDIVR_SHIFT    (0x00000000U)
#define SDL_MSS_RCM_MSS_RTIC_CLK_DIV_VAL_MSS_RTIC_CLK_DIV_VAL_CLKDIVR_RESETVAL (0x00000000U)
#define SDL_MSS_RCM_MSS_RTIC_CLK_DIV_VAL_MSS_RTIC_CLK_DIV_VAL_CLKDIVR_MAX      (0x00000FFFU)

#define SDL_MSS_RCM_MSS_RTIC_CLK_DIV_VAL_RESETVAL                              (0x00000000U)

/* MSS_WDT_CLK_DIV_VAL */

#define SDL_MSS_RCM_MSS_WDT_CLK_DIV_VAL_MSS_WDT_CLK_DIV_VAL_CLKDIVR_MASK       (0x00000FFFU)
#define SDL_MSS_RCM_MSS_WDT_CLK_DIV_VAL_MSS_WDT_CLK_DIV_VAL_CLKDIVR_SHIFT      (0x00000000U)
#define SDL_MSS_RCM_MSS_WDT_CLK_DIV_VAL_MSS_WDT_CLK_DIV_VAL_CLKDIVR_RESETVAL   (0x00000000U)
#define SDL_MSS_RCM_MSS_WDT_CLK_DIV_VAL_MSS_WDT_CLK_DIV_VAL_CLKDIVR_MAX        (0x00000FFFU)

#define SDL_MSS_RCM_MSS_WDT_CLK_DIV_VAL_RESETVAL                               (0x00000000U)

/* MSS_SPIA_CLK_DIV_VAL */

#define SDL_MSS_RCM_MSS_SPIA_CLK_DIV_VAL_MSS_SPIA_CLK_DIV_VAL_CLKDIVR_MASK     (0x00000FFFU)
#define SDL_MSS_RCM_MSS_SPIA_CLK_DIV_VAL_MSS_SPIA_CLK_DIV_VAL_CLKDIVR_SHIFT    (0x00000000U)
#define SDL_MSS_RCM_MSS_SPIA_CLK_DIV_VAL_MSS_SPIA_CLK_DIV_VAL_CLKDIVR_RESETVAL (0x00000000U)
#define SDL_MSS_RCM_MSS_SPIA_CLK_DIV_VAL_MSS_SPIA_CLK_DIV_VAL_CLKDIVR_MAX      (0x00000FFFU)

#define SDL_MSS_RCM_MSS_SPIA_CLK_DIV_VAL_RESETVAL                              (0x00000000U)

/* MSS_SPIB_CLK_DIV_VAL */

#define SDL_MSS_RCM_MSS_SPIB_CLK_DIV_VAL_MSS_SPIB_CLK_DIV_VAL_CLKDIVR_MASK     (0x00000FFFU)
#define SDL_MSS_RCM_MSS_SPIB_CLK_DIV_VAL_MSS_SPIB_CLK_DIV_VAL_CLKDIVR_SHIFT    (0x00000000U)
#define SDL_MSS_RCM_MSS_SPIB_CLK_DIV_VAL_MSS_SPIB_CLK_DIV_VAL_CLKDIVR_RESETVAL (0x00000000U)
#define SDL_MSS_RCM_MSS_SPIB_CLK_DIV_VAL_MSS_SPIB_CLK_DIV_VAL_CLKDIVR_MAX      (0x00000FFFU)

#define SDL_MSS_RCM_MSS_SPIB_CLK_DIV_VAL_RESETVAL                              (0x00000000U)

/* MSS_I2C_CLK_DIV_VAL */

#define SDL_MSS_RCM_MSS_I2C_CLK_DIV_VAL_MSS_I2C_CLK_DIV_VAL_CLKDIVR_MASK       (0x00000FFFU)
#define SDL_MSS_RCM_MSS_I2C_CLK_DIV_VAL_MSS_I2C_CLK_DIV_VAL_CLKDIVR_SHIFT      (0x00000000U)
#define SDL_MSS_RCM_MSS_I2C_CLK_DIV_VAL_MSS_I2C_CLK_DIV_VAL_CLKDIVR_RESETVAL   (0x00000000U)
#define SDL_MSS_RCM_MSS_I2C_CLK_DIV_VAL_MSS_I2C_CLK_DIV_VAL_CLKDIVR_MAX        (0x00000FFFU)

#define SDL_MSS_RCM_MSS_I2C_CLK_DIV_VAL_RESETVAL                               (0x00000000U)

/* MSS_SCIA_CLK_DIV_VAL */

#define SDL_MSS_RCM_MSS_SCIA_CLK_DIV_VAL_MSS_SCIA_CLK_DIV_VAL_CLKDIVR_MASK     (0x00000FFFU)
#define SDL_MSS_RCM_MSS_SCIA_CLK_DIV_VAL_MSS_SCIA_CLK_DIV_VAL_CLKDIVR_SHIFT    (0x00000000U)
#define SDL_MSS_RCM_MSS_SCIA_CLK_DIV_VAL_MSS_SCIA_CLK_DIV_VAL_CLKDIVR_RESETVAL (0x00000000U)
#define SDL_MSS_RCM_MSS_SCIA_CLK_DIV_VAL_MSS_SCIA_CLK_DIV_VAL_CLKDIVR_MAX      (0x00000FFFU)

#define SDL_MSS_RCM_MSS_SCIA_CLK_DIV_VAL_RESETVAL                              (0x00000000U)

/* MSS_SCIB_CLK_DIV_VAL */

#define SDL_MSS_RCM_MSS_SCIB_CLK_DIV_VAL_MSS_SCIB_CLK_DIV_VAL_CLKDIVR_MASK     (0x00000FFFU)
#define SDL_MSS_RCM_MSS_SCIB_CLK_DIV_VAL_MSS_SCIB_CLK_DIV_VAL_CLKDIVR_SHIFT    (0x00000000U)
#define SDL_MSS_RCM_MSS_SCIB_CLK_DIV_VAL_MSS_SCIB_CLK_DIV_VAL_CLKDIVR_RESETVAL (0x00000000U)
#define SDL_MSS_RCM_MSS_SCIB_CLK_DIV_VAL_MSS_SCIB_CLK_DIV_VAL_CLKDIVR_MAX      (0x00000FFFU)

#define SDL_MSS_RCM_MSS_SCIB_CLK_DIV_VAL_RESETVAL                              (0x00000000U)

/* MSS_CPTS_CLK_DIV_VAL */

#define SDL_MSS_RCM_MSS_CPTS_CLK_DIV_VAL_MSS_CPTS_CLK_DIV_VAL_CLKDIVR_MASK     (0x00000FFFU)
#define SDL_MSS_RCM_MSS_CPTS_CLK_DIV_VAL_MSS_CPTS_CLK_DIV_VAL_CLKDIVR_SHIFT    (0x00000000U)
#define SDL_MSS_RCM_MSS_CPTS_CLK_DIV_VAL_MSS_CPTS_CLK_DIV_VAL_CLKDIVR_RESETVAL (0x00000000U)
#define SDL_MSS_RCM_MSS_CPTS_CLK_DIV_VAL_MSS_CPTS_CLK_DIV_VAL_CLKDIVR_MAX      (0x00000FFFU)

#define SDL_MSS_RCM_MSS_CPTS_CLK_DIV_VAL_RESETVAL                              (0x00000000U)

/* MSS_CPSW_CLK_DIV_VAL */

#define SDL_MSS_RCM_MSS_CPSW_CLK_DIV_VAL_MSS_CPSW_CLK_DIV_VAL_CLKDIVR_MASK     (0x00000FFFU)
#define SDL_MSS_RCM_MSS_CPSW_CLK_DIV_VAL_MSS_CPSW_CLK_DIV_VAL_CLKDIVR_SHIFT    (0x00000000U)
#define SDL_MSS_RCM_MSS_CPSW_CLK_DIV_VAL_MSS_CPSW_CLK_DIV_VAL_CLKDIVR_RESETVAL (0x00000000U)
#define SDL_MSS_RCM_MSS_CPSW_CLK_DIV_VAL_MSS_CPSW_CLK_DIV_VAL_CLKDIVR_MAX      (0x00000FFFU)

#define SDL_MSS_RCM_MSS_CPSW_CLK_DIV_VAL_RESETVAL                              (0x00000000U)

/* MSS_RGMII_CLK_DIV_VAL */

#define SDL_MSS_RCM_MSS_RGMII_CLK_DIV_VAL_MSS_RGMII_CLK_DIV_VAL_CLKDIVR_MASK   (0x00000FFFU)
#define SDL_MSS_RCM_MSS_RGMII_CLK_DIV_VAL_MSS_RGMII_CLK_DIV_VAL_CLKDIVR_SHIFT  (0x00000000U)
#define SDL_MSS_RCM_MSS_RGMII_CLK_DIV_VAL_MSS_RGMII_CLK_DIV_VAL_CLKDIVR_RESETVAL (0x00000000U)
#define SDL_MSS_RCM_MSS_RGMII_CLK_DIV_VAL_MSS_RGMII_CLK_DIV_VAL_CLKDIVR_MAX    (0x00000FFFU)

#define SDL_MSS_RCM_MSS_RGMII_CLK_DIV_VAL_RESETVAL                             (0x00000000U)

/* MSS_MII100_CLK_DIV_VAL */

#define SDL_MSS_RCM_MSS_MII100_CLK_DIV_VAL_MSS_MII100_CLK_DIV_VAL_CLKDIVR_MASK (0x00000FFFU)
#define SDL_MSS_RCM_MSS_MII100_CLK_DIV_VAL_MSS_MII100_CLK_DIV_VAL_CLKDIVR_SHIFT (0x00000000U)
#define SDL_MSS_RCM_MSS_MII100_CLK_DIV_VAL_MSS_MII100_CLK_DIV_VAL_CLKDIVR_RESETVAL (0x00000000U)
#define SDL_MSS_RCM_MSS_MII100_CLK_DIV_VAL_MSS_MII100_CLK_DIV_VAL_CLKDIVR_MAX  (0x00000FFFU)

#define SDL_MSS_RCM_MSS_MII100_CLK_DIV_VAL_RESETVAL                            (0x00000000U)

/* MSS_MII10_CLK_DIV_VAL */

#define SDL_MSS_RCM_MSS_MII10_CLK_DIV_VAL_MSS_MII10_CLK_DIV_VAL_CLKDIVR_MASK   (0x00FFFFFFU)
#define SDL_MSS_RCM_MSS_MII10_CLK_DIV_VAL_MSS_MII10_CLK_DIV_VAL_CLKDIVR_SHIFT  (0x00000000U)
#define SDL_MSS_RCM_MSS_MII10_CLK_DIV_VAL_MSS_MII10_CLK_DIV_VAL_CLKDIVR_RESETVAL (0x00000000U)
#define SDL_MSS_RCM_MSS_MII10_CLK_DIV_VAL_MSS_MII10_CLK_DIV_VAL_CLKDIVR_MAX    (0x00FFFFFFU)

#define SDL_MSS_RCM_MSS_MII10_CLK_DIV_VAL_RESETVAL                             (0x00000000U)

/* MSS_GPADC_CLK_DIV_VAL */

#define SDL_MSS_RCM_MSS_GPADC_CLK_DIV_VAL_MSS_GPADC_CLK_DIV_VAL_CLKDIVR_MASK   (0x00FFFFFFU)
#define SDL_MSS_RCM_MSS_GPADC_CLK_DIV_VAL_MSS_GPADC_CLK_DIV_VAL_CLKDIVR_SHIFT  (0x00000000U)
#define SDL_MSS_RCM_MSS_GPADC_CLK_DIV_VAL_MSS_GPADC_CLK_DIV_VAL_CLKDIVR_RESETVAL (0x00000000U)
#define SDL_MSS_RCM_MSS_GPADC_CLK_DIV_VAL_MSS_GPADC_CLK_DIV_VAL_CLKDIVR_MAX    (0x00FFFFFFU)

#define SDL_MSS_RCM_MSS_GPADC_CLK_DIV_VAL_RESETVAL                             (0x00000000U)

/* MSS_MCANA_CLK_GATE */

#define SDL_MSS_RCM_MSS_MCANA_CLK_GATE_MSS_MCANA_CLK_GATE_GATED_MASK           (0x00000007U)
#define SDL_MSS_RCM_MSS_MCANA_CLK_GATE_MSS_MCANA_CLK_GATE_GATED_SHIFT          (0x00000000U)
#define SDL_MSS_RCM_MSS_MCANA_CLK_GATE_MSS_MCANA_CLK_GATE_GATED_RESETVAL       (0x00000000U)
#define SDL_MSS_RCM_MSS_MCANA_CLK_GATE_MSS_MCANA_CLK_GATE_GATED_MAX            (0x00000007U)

#define SDL_MSS_RCM_MSS_MCANA_CLK_GATE_RESETVAL                                (0x00000000U)

/* MSS_MCANB_CLK_GATE */

#define SDL_MSS_RCM_MSS_MCANB_CLK_GATE_MSS_MCANB_CLK_GATE_GATED_MASK           (0x00000007U)
#define SDL_MSS_RCM_MSS_MCANB_CLK_GATE_MSS_MCANB_CLK_GATE_GATED_SHIFT          (0x00000000U)
#define SDL_MSS_RCM_MSS_MCANB_CLK_GATE_MSS_MCANB_CLK_GATE_GATED_RESETVAL       (0x00000000U)
#define SDL_MSS_RCM_MSS_MCANB_CLK_GATE_MSS_MCANB_CLK_GATE_GATED_MAX            (0x00000007U)

#define SDL_MSS_RCM_MSS_MCANB_CLK_GATE_RESETVAL                                (0x00000000U)

/* MSS_QSPI_CLK_GATE */

#define SDL_MSS_RCM_MSS_QSPI_CLK_GATE_MSS_QSPI_CLK_GATE_GATED_MASK             (0x00000007U)
#define SDL_MSS_RCM_MSS_QSPI_CLK_GATE_MSS_QSPI_CLK_GATE_GATED_SHIFT            (0x00000000U)
#define SDL_MSS_RCM_MSS_QSPI_CLK_GATE_MSS_QSPI_CLK_GATE_GATED_RESETVAL         (0x00000000U)
#define SDL_MSS_RCM_MSS_QSPI_CLK_GATE_MSS_QSPI_CLK_GATE_GATED_MAX              (0x00000007U)

#define SDL_MSS_RCM_MSS_QSPI_CLK_GATE_RESETVAL                                 (0x00000000U)

/* MSS_RTIA_CLK_GATE */

#define SDL_MSS_RCM_MSS_RTIA_CLK_GATE_MSS_RTIA_CLK_GATE_GATED_MASK             (0x00000007U)
#define SDL_MSS_RCM_MSS_RTIA_CLK_GATE_MSS_RTIA_CLK_GATE_GATED_SHIFT            (0x00000000U)
#define SDL_MSS_RCM_MSS_RTIA_CLK_GATE_MSS_RTIA_CLK_GATE_GATED_RESETVAL         (0x00000000U)
#define SDL_MSS_RCM_MSS_RTIA_CLK_GATE_MSS_RTIA_CLK_GATE_GATED_MAX              (0x00000007U)

#define SDL_MSS_RCM_MSS_RTIA_CLK_GATE_RESETVAL                                 (0x00000000U)

/* MSS_RTIB_CLK_GATE */

#define SDL_MSS_RCM_MSS_RTIB_CLK_GATE_MSS_RTIB_CLK_GATE_GATED_MASK             (0x00000007U)
#define SDL_MSS_RCM_MSS_RTIB_CLK_GATE_MSS_RTIB_CLK_GATE_GATED_SHIFT            (0x00000000U)
#define SDL_MSS_RCM_MSS_RTIB_CLK_GATE_MSS_RTIB_CLK_GATE_GATED_RESETVAL         (0x00000000U)
#define SDL_MSS_RCM_MSS_RTIB_CLK_GATE_MSS_RTIB_CLK_GATE_GATED_MAX              (0x00000007U)

#define SDL_MSS_RCM_MSS_RTIB_CLK_GATE_RESETVAL                                 (0x00000000U)

/* MSS_RTIC_CLK_GATE */

#define SDL_MSS_RCM_MSS_RTIC_CLK_GATE_MSS_RTIC_CLK_GATE_GATED_MASK             (0x00000007U)
#define SDL_MSS_RCM_MSS_RTIC_CLK_GATE_MSS_RTIC_CLK_GATE_GATED_SHIFT            (0x00000000U)
#define SDL_MSS_RCM_MSS_RTIC_CLK_GATE_MSS_RTIC_CLK_GATE_GATED_RESETVAL         (0x00000000U)
#define SDL_MSS_RCM_MSS_RTIC_CLK_GATE_MSS_RTIC_CLK_GATE_GATED_MAX              (0x00000007U)

#define SDL_MSS_RCM_MSS_RTIC_CLK_GATE_RESETVAL                                 (0x00000000U)

/* MSS_WDT_CLK_GATE */

#define SDL_MSS_RCM_MSS_WDT_CLK_GATE_MSS_WDT_CLK_GATE_GATED_MASK               (0x00000007U)
#define SDL_MSS_RCM_MSS_WDT_CLK_GATE_MSS_WDT_CLK_GATE_GATED_SHIFT              (0x00000000U)
#define SDL_MSS_RCM_MSS_WDT_CLK_GATE_MSS_WDT_CLK_GATE_GATED_RESETVAL           (0x00000000U)
#define SDL_MSS_RCM_MSS_WDT_CLK_GATE_MSS_WDT_CLK_GATE_GATED_MAX                (0x00000007U)

#define SDL_MSS_RCM_MSS_WDT_CLK_GATE_RESETVAL                                  (0x00000000U)

/* MSS_SPIA_CLK_GATE */

#define SDL_MSS_RCM_MSS_SPIA_CLK_GATE_MSS_SPIA_CLK_GATE_GATED_MASK             (0x00000007U)
#define SDL_MSS_RCM_MSS_SPIA_CLK_GATE_MSS_SPIA_CLK_GATE_GATED_SHIFT            (0x00000000U)
#define SDL_MSS_RCM_MSS_SPIA_CLK_GATE_MSS_SPIA_CLK_GATE_GATED_RESETVAL         (0x00000000U)
#define SDL_MSS_RCM_MSS_SPIA_CLK_GATE_MSS_SPIA_CLK_GATE_GATED_MAX              (0x00000007U)

#define SDL_MSS_RCM_MSS_SPIA_CLK_GATE_RESETVAL                                 (0x00000000U)

/* MSS_SPIB_CLK_GATE */

#define SDL_MSS_RCM_MSS_SPIB_CLK_GATE_MSS_SPIB_CLK_GATE_GATED_MASK             (0x00000007U)
#define SDL_MSS_RCM_MSS_SPIB_CLK_GATE_MSS_SPIB_CLK_GATE_GATED_SHIFT            (0x00000000U)
#define SDL_MSS_RCM_MSS_SPIB_CLK_GATE_MSS_SPIB_CLK_GATE_GATED_RESETVAL         (0x00000000U)
#define SDL_MSS_RCM_MSS_SPIB_CLK_GATE_MSS_SPIB_CLK_GATE_GATED_MAX              (0x00000007U)

#define SDL_MSS_RCM_MSS_SPIB_CLK_GATE_RESETVAL                                 (0x00000000U)

/* MSS_I2C_CLK_GATE */

#define SDL_MSS_RCM_MSS_I2C_CLK_GATE_MSS_I2C_CLK_GATE_GATED_MASK               (0x00000007U)
#define SDL_MSS_RCM_MSS_I2C_CLK_GATE_MSS_I2C_CLK_GATE_GATED_SHIFT              (0x00000000U)
#define SDL_MSS_RCM_MSS_I2C_CLK_GATE_MSS_I2C_CLK_GATE_GATED_RESETVAL           (0x00000000U)
#define SDL_MSS_RCM_MSS_I2C_CLK_GATE_MSS_I2C_CLK_GATE_GATED_MAX                (0x00000007U)

#define SDL_MSS_RCM_MSS_I2C_CLK_GATE_RESETVAL                                  (0x00000000U)

/* MSS_SCIA_CLK_GATE */

#define SDL_MSS_RCM_MSS_SCIA_CLK_GATE_MSS_SCIA_CLK_GATE_GATED_MASK             (0x00000007U)
#define SDL_MSS_RCM_MSS_SCIA_CLK_GATE_MSS_SCIA_CLK_GATE_GATED_SHIFT            (0x00000000U)
#define SDL_MSS_RCM_MSS_SCIA_CLK_GATE_MSS_SCIA_CLK_GATE_GATED_RESETVAL         (0x00000000U)
#define SDL_MSS_RCM_MSS_SCIA_CLK_GATE_MSS_SCIA_CLK_GATE_GATED_MAX              (0x00000007U)

#define SDL_MSS_RCM_MSS_SCIA_CLK_GATE_RESETVAL                                 (0x00000000U)

/* MSS_SCIB_CLK_GATE */

#define SDL_MSS_RCM_MSS_SCIB_CLK_GATE_MSS_SCIB_CLK_GATE_GATED_MASK             (0x00000007U)
#define SDL_MSS_RCM_MSS_SCIB_CLK_GATE_MSS_SCIB_CLK_GATE_GATED_SHIFT            (0x00000000U)
#define SDL_MSS_RCM_MSS_SCIB_CLK_GATE_MSS_SCIB_CLK_GATE_GATED_RESETVAL         (0x00000000U)
#define SDL_MSS_RCM_MSS_SCIB_CLK_GATE_MSS_SCIB_CLK_GATE_GATED_MAX              (0x00000007U)

#define SDL_MSS_RCM_MSS_SCIB_CLK_GATE_RESETVAL                                 (0x00000000U)

/* MSS_CPTS_CLK_GATE */

#define SDL_MSS_RCM_MSS_CPTS_CLK_GATE_MSS_CPTS_CLK_GATE_GATED_MASK             (0x00000007U)
#define SDL_MSS_RCM_MSS_CPTS_CLK_GATE_MSS_CPTS_CLK_GATE_GATED_SHIFT            (0x00000000U)
#define SDL_MSS_RCM_MSS_CPTS_CLK_GATE_MSS_CPTS_CLK_GATE_GATED_RESETVAL         (0x00000000U)
#define SDL_MSS_RCM_MSS_CPTS_CLK_GATE_MSS_CPTS_CLK_GATE_GATED_MAX              (0x00000007U)

#define SDL_MSS_RCM_MSS_CPTS_CLK_GATE_RESETVAL                                 (0x00000000U)

/* MSS_CPSW_CLK_GATE */

#define SDL_MSS_RCM_MSS_CPSW_CLK_GATE_MSS_CPSW_CLK_GATE_GATED_MASK             (0x00000007U)
#define SDL_MSS_RCM_MSS_CPSW_CLK_GATE_MSS_CPSW_CLK_GATE_GATED_SHIFT            (0x00000000U)
#define SDL_MSS_RCM_MSS_CPSW_CLK_GATE_MSS_CPSW_CLK_GATE_GATED_RESETVAL         (0x00000000U)
#define SDL_MSS_RCM_MSS_CPSW_CLK_GATE_MSS_CPSW_CLK_GATE_GATED_MAX              (0x00000007U)

#define SDL_MSS_RCM_MSS_CPSW_CLK_GATE_RESETVAL                                 (0x00000000U)

/* MSS_RGMII_CLK_GATE */

#define SDL_MSS_RCM_MSS_RGMII_CLK_GATE_MSS_RGMII_CLK_GATE_GATED_MASK           (0x00000007U)
#define SDL_MSS_RCM_MSS_RGMII_CLK_GATE_MSS_RGMII_CLK_GATE_GATED_SHIFT          (0x00000000U)
#define SDL_MSS_RCM_MSS_RGMII_CLK_GATE_MSS_RGMII_CLK_GATE_GATED_RESETVAL       (0x00000000U)
#define SDL_MSS_RCM_MSS_RGMII_CLK_GATE_MSS_RGMII_CLK_GATE_GATED_MAX            (0x00000007U)

#define SDL_MSS_RCM_MSS_RGMII_CLK_GATE_RESETVAL                                (0x00000000U)

/* MSS_MII100_CLK_GATE */

#define SDL_MSS_RCM_MSS_MII100_CLK_GATE_MSS_MII100_CLK_GATE_GATED_MASK         (0x00000007U)
#define SDL_MSS_RCM_MSS_MII100_CLK_GATE_MSS_MII100_CLK_GATE_GATED_SHIFT        (0x00000000U)
#define SDL_MSS_RCM_MSS_MII100_CLK_GATE_MSS_MII100_CLK_GATE_GATED_RESETVAL     (0x00000000U)
#define SDL_MSS_RCM_MSS_MII100_CLK_GATE_MSS_MII100_CLK_GATE_GATED_MAX          (0x00000007U)

#define SDL_MSS_RCM_MSS_MII100_CLK_GATE_RESETVAL                               (0x00000000U)

/* MSS_MII10_CLK_GATE */

#define SDL_MSS_RCM_MSS_MII10_CLK_GATE_MSS_MII10_CLK_GATE_GATED_MASK           (0x00000007U)
#define SDL_MSS_RCM_MSS_MII10_CLK_GATE_MSS_MII10_CLK_GATE_GATED_SHIFT          (0x00000000U)
#define SDL_MSS_RCM_MSS_MII10_CLK_GATE_MSS_MII10_CLK_GATE_GATED_RESETVAL       (0x00000000U)
#define SDL_MSS_RCM_MSS_MII10_CLK_GATE_MSS_MII10_CLK_GATE_GATED_MAX            (0x00000007U)

#define SDL_MSS_RCM_MSS_MII10_CLK_GATE_RESETVAL                                (0x00000000U)

/* MSS_GPADC_CLK_GATE */

#define SDL_MSS_RCM_MSS_GPADC_CLK_GATE_MSS_GPADC_CLK_GATE_GATED_MASK           (0x00000007U)
#define SDL_MSS_RCM_MSS_GPADC_CLK_GATE_MSS_GPADC_CLK_GATE_GATED_SHIFT          (0x00000000U)
#define SDL_MSS_RCM_MSS_GPADC_CLK_GATE_MSS_GPADC_CLK_GATE_GATED_RESETVAL       (0x00000000U)
#define SDL_MSS_RCM_MSS_GPADC_CLK_GATE_MSS_GPADC_CLK_GATE_GATED_MAX            (0x00000007U)

#define SDL_MSS_RCM_MSS_GPADC_CLK_GATE_RESETVAL                                (0x00000000U)

/* MSS_MCANA_CLK_STATUS */

#define SDL_MSS_RCM_MSS_MCANA_CLK_STATUS_MSS_MCANA_CLK_STATUS_CLKINUSE_MASK    (0x000000FFU)
#define SDL_MSS_RCM_MSS_MCANA_CLK_STATUS_MSS_MCANA_CLK_STATUS_CLKINUSE_SHIFT   (0x00000000U)
#define SDL_MSS_RCM_MSS_MCANA_CLK_STATUS_MSS_MCANA_CLK_STATUS_CLKINUSE_RESETVAL (0x00000001U)
#define SDL_MSS_RCM_MSS_MCANA_CLK_STATUS_MSS_MCANA_CLK_STATUS_CLKINUSE_MAX     (0x000000FFU)

#define SDL_MSS_RCM_MSS_MCANA_CLK_STATUS_MSS_MCANA_CLK_STATUS_CURRDIVIDER_MASK (0x0000FF00U)
#define SDL_MSS_RCM_MSS_MCANA_CLK_STATUS_MSS_MCANA_CLK_STATUS_CURRDIVIDER_SHIFT (0x00000008U)
#define SDL_MSS_RCM_MSS_MCANA_CLK_STATUS_MSS_MCANA_CLK_STATUS_CURRDIVIDER_RESETVAL (0x00000000U)
#define SDL_MSS_RCM_MSS_MCANA_CLK_STATUS_MSS_MCANA_CLK_STATUS_CURRDIVIDER_MAX  (0x000000FFU)

#define SDL_MSS_RCM_MSS_MCANA_CLK_STATUS_RESETVAL                              (0x00000001U)

/* MSS_MCANB_CLK_STATUS */

#define SDL_MSS_RCM_MSS_MCANB_CLK_STATUS_MSS_MCANB_CLK_STATUS_CLKINUSE_MASK    (0x000000FFU)
#define SDL_MSS_RCM_MSS_MCANB_CLK_STATUS_MSS_MCANB_CLK_STATUS_CLKINUSE_SHIFT   (0x00000000U)
#define SDL_MSS_RCM_MSS_MCANB_CLK_STATUS_MSS_MCANB_CLK_STATUS_CLKINUSE_RESETVAL (0x00000001U)
#define SDL_MSS_RCM_MSS_MCANB_CLK_STATUS_MSS_MCANB_CLK_STATUS_CLKINUSE_MAX     (0x000000FFU)

#define SDL_MSS_RCM_MSS_MCANB_CLK_STATUS_MSS_MCANB_CLK_STATUS_CURRDIVIDER_MASK (0x0000FF00U)
#define SDL_MSS_RCM_MSS_MCANB_CLK_STATUS_MSS_MCANB_CLK_STATUS_CURRDIVIDER_SHIFT (0x00000008U)
#define SDL_MSS_RCM_MSS_MCANB_CLK_STATUS_MSS_MCANB_CLK_STATUS_CURRDIVIDER_RESETVAL (0x00000000U)
#define SDL_MSS_RCM_MSS_MCANB_CLK_STATUS_MSS_MCANB_CLK_STATUS_CURRDIVIDER_MAX  (0x000000FFU)

#define SDL_MSS_RCM_MSS_MCANB_CLK_STATUS_RESETVAL                              (0x00000001U)

/* MSS_QSPI_CLK_STATUS */

#define SDL_MSS_RCM_MSS_QSPI_CLK_STATUS_MSS_QSPI_CLK_STATUS_CLKINUSE_MASK      (0x000000FFU)
#define SDL_MSS_RCM_MSS_QSPI_CLK_STATUS_MSS_QSPI_CLK_STATUS_CLKINUSE_SHIFT     (0x00000000U)
#define SDL_MSS_RCM_MSS_QSPI_CLK_STATUS_MSS_QSPI_CLK_STATUS_CLKINUSE_RESETVAL  (0x00000001U)
#define SDL_MSS_RCM_MSS_QSPI_CLK_STATUS_MSS_QSPI_CLK_STATUS_CLKINUSE_MAX       (0x000000FFU)

#define SDL_MSS_RCM_MSS_QSPI_CLK_STATUS_MSS_QSPI_CLK_STATUS_CURRDIVIDER_MASK   (0x0000FF00U)
#define SDL_MSS_RCM_MSS_QSPI_CLK_STATUS_MSS_QSPI_CLK_STATUS_CURRDIVIDER_SHIFT  (0x00000008U)
#define SDL_MSS_RCM_MSS_QSPI_CLK_STATUS_MSS_QSPI_CLK_STATUS_CURRDIVIDER_RESETVAL (0x00000000U)
#define SDL_MSS_RCM_MSS_QSPI_CLK_STATUS_MSS_QSPI_CLK_STATUS_CURRDIVIDER_MAX    (0x000000FFU)

#define SDL_MSS_RCM_MSS_QSPI_CLK_STATUS_RESETVAL                               (0x00000001U)

/* MSS_RTIA_CLK_STATUS */

#define SDL_MSS_RCM_MSS_RTIA_CLK_STATUS_MSS_RTIA_CLK_STATUS_CLKINUSE_MASK      (0x000000FFU)
#define SDL_MSS_RCM_MSS_RTIA_CLK_STATUS_MSS_RTIA_CLK_STATUS_CLKINUSE_SHIFT     (0x00000000U)
#define SDL_MSS_RCM_MSS_RTIA_CLK_STATUS_MSS_RTIA_CLK_STATUS_CLKINUSE_RESETVAL  (0x00000001U)
#define SDL_MSS_RCM_MSS_RTIA_CLK_STATUS_MSS_RTIA_CLK_STATUS_CLKINUSE_MAX       (0x000000FFU)

#define SDL_MSS_RCM_MSS_RTIA_CLK_STATUS_MSS_RTIA_CLK_STATUS_CURRDIVIDER_MASK   (0x0000FF00U)
#define SDL_MSS_RCM_MSS_RTIA_CLK_STATUS_MSS_RTIA_CLK_STATUS_CURRDIVIDER_SHIFT  (0x00000008U)
#define SDL_MSS_RCM_MSS_RTIA_CLK_STATUS_MSS_RTIA_CLK_STATUS_CURRDIVIDER_RESETVAL (0x00000000U)
#define SDL_MSS_RCM_MSS_RTIA_CLK_STATUS_MSS_RTIA_CLK_STATUS_CURRDIVIDER_MAX    (0x000000FFU)

#define SDL_MSS_RCM_MSS_RTIA_CLK_STATUS_RESETVAL                               (0x00000001U)

/* MSS_RTIB_CLK_STATUS */

#define SDL_MSS_RCM_MSS_RTIB_CLK_STATUS_MSS_RTIB_CLK_STATUS_CLKINUSE_MASK      (0x000000FFU)
#define SDL_MSS_RCM_MSS_RTIB_CLK_STATUS_MSS_RTIB_CLK_STATUS_CLKINUSE_SHIFT     (0x00000000U)
#define SDL_MSS_RCM_MSS_RTIB_CLK_STATUS_MSS_RTIB_CLK_STATUS_CLKINUSE_RESETVAL  (0x00000001U)
#define SDL_MSS_RCM_MSS_RTIB_CLK_STATUS_MSS_RTIB_CLK_STATUS_CLKINUSE_MAX       (0x000000FFU)

#define SDL_MSS_RCM_MSS_RTIB_CLK_STATUS_MSS_RTIB_CLK_STATUS_CURRDIVIDER_MASK   (0x0000FF00U)
#define SDL_MSS_RCM_MSS_RTIB_CLK_STATUS_MSS_RTIB_CLK_STATUS_CURRDIVIDER_SHIFT  (0x00000008U)
#define SDL_MSS_RCM_MSS_RTIB_CLK_STATUS_MSS_RTIB_CLK_STATUS_CURRDIVIDER_RESETVAL (0x00000000U)
#define SDL_MSS_RCM_MSS_RTIB_CLK_STATUS_MSS_RTIB_CLK_STATUS_CURRDIVIDER_MAX    (0x000000FFU)

#define SDL_MSS_RCM_MSS_RTIB_CLK_STATUS_RESETVAL                               (0x00000001U)

/* MSS_RTIC_CLK_STATUS */

#define SDL_MSS_RCM_MSS_RTIC_CLK_STATUS_MSS_RTIC_CLK_STATUS_CLKINUSE_MASK      (0x000000FFU)
#define SDL_MSS_RCM_MSS_RTIC_CLK_STATUS_MSS_RTIC_CLK_STATUS_CLKINUSE_SHIFT     (0x00000000U)
#define SDL_MSS_RCM_MSS_RTIC_CLK_STATUS_MSS_RTIC_CLK_STATUS_CLKINUSE_RESETVAL  (0x00000001U)
#define SDL_MSS_RCM_MSS_RTIC_CLK_STATUS_MSS_RTIC_CLK_STATUS_CLKINUSE_MAX       (0x000000FFU)

#define SDL_MSS_RCM_MSS_RTIC_CLK_STATUS_MSS_RTIC_CLK_STATUS_CURRDIVIDER_MASK   (0x0000FF00U)
#define SDL_MSS_RCM_MSS_RTIC_CLK_STATUS_MSS_RTIC_CLK_STATUS_CURRDIVIDER_SHIFT  (0x00000008U)
#define SDL_MSS_RCM_MSS_RTIC_CLK_STATUS_MSS_RTIC_CLK_STATUS_CURRDIVIDER_RESETVAL (0x00000000U)
#define SDL_MSS_RCM_MSS_RTIC_CLK_STATUS_MSS_RTIC_CLK_STATUS_CURRDIVIDER_MAX    (0x000000FFU)

#define SDL_MSS_RCM_MSS_RTIC_CLK_STATUS_RESETVAL                               (0x00000001U)

/* MSS_WDT_CLK_STATUS */

#define SDL_MSS_RCM_MSS_WDT_CLK_STATUS_MSS_WDT_CLK_STATUS_CLKINUSE_MASK        (0x000000FFU)
#define SDL_MSS_RCM_MSS_WDT_CLK_STATUS_MSS_WDT_CLK_STATUS_CLKINUSE_SHIFT       (0x00000000U)
#define SDL_MSS_RCM_MSS_WDT_CLK_STATUS_MSS_WDT_CLK_STATUS_CLKINUSE_RESETVAL    (0x00000001U)
#define SDL_MSS_RCM_MSS_WDT_CLK_STATUS_MSS_WDT_CLK_STATUS_CLKINUSE_MAX         (0x000000FFU)

#define SDL_MSS_RCM_MSS_WDT_CLK_STATUS_MSS_WDT_CLK_STATUS_CURRDIVIDER_MASK     (0x0000FF00U)
#define SDL_MSS_RCM_MSS_WDT_CLK_STATUS_MSS_WDT_CLK_STATUS_CURRDIVIDER_SHIFT    (0x00000008U)
#define SDL_MSS_RCM_MSS_WDT_CLK_STATUS_MSS_WDT_CLK_STATUS_CURRDIVIDER_RESETVAL (0x00000000U)
#define SDL_MSS_RCM_MSS_WDT_CLK_STATUS_MSS_WDT_CLK_STATUS_CURRDIVIDER_MAX      (0x000000FFU)

#define SDL_MSS_RCM_MSS_WDT_CLK_STATUS_RESETVAL                                (0x00000001U)

/* MSS_SPIA_CLK_STATUS */

#define SDL_MSS_RCM_MSS_SPIA_CLK_STATUS_MSS_SPIA_CLK_STATUS_CLKINUSE_MASK      (0x000000FFU)
#define SDL_MSS_RCM_MSS_SPIA_CLK_STATUS_MSS_SPIA_CLK_STATUS_CLKINUSE_SHIFT     (0x00000000U)
#define SDL_MSS_RCM_MSS_SPIA_CLK_STATUS_MSS_SPIA_CLK_STATUS_CLKINUSE_RESETVAL  (0x00000001U)
#define SDL_MSS_RCM_MSS_SPIA_CLK_STATUS_MSS_SPIA_CLK_STATUS_CLKINUSE_MAX       (0x000000FFU)

#define SDL_MSS_RCM_MSS_SPIA_CLK_STATUS_MSS_SPIA_CLK_STATUS_CURRDIVIDER_MASK   (0x0000FF00U)
#define SDL_MSS_RCM_MSS_SPIA_CLK_STATUS_MSS_SPIA_CLK_STATUS_CURRDIVIDER_SHIFT  (0x00000008U)
#define SDL_MSS_RCM_MSS_SPIA_CLK_STATUS_MSS_SPIA_CLK_STATUS_CURRDIVIDER_RESETVAL (0x00000000U)
#define SDL_MSS_RCM_MSS_SPIA_CLK_STATUS_MSS_SPIA_CLK_STATUS_CURRDIVIDER_MAX    (0x000000FFU)

#define SDL_MSS_RCM_MSS_SPIA_CLK_STATUS_RESETVAL                               (0x00000001U)

/* MSS_SPIB_CLK_STATUS */

#define SDL_MSS_RCM_MSS_SPIB_CLK_STATUS_MSS_SPIB_CLK_STATUS_CLKINUSE_MASK      (0x000000FFU)
#define SDL_MSS_RCM_MSS_SPIB_CLK_STATUS_MSS_SPIB_CLK_STATUS_CLKINUSE_SHIFT     (0x00000000U)
#define SDL_MSS_RCM_MSS_SPIB_CLK_STATUS_MSS_SPIB_CLK_STATUS_CLKINUSE_RESETVAL  (0x00000001U)
#define SDL_MSS_RCM_MSS_SPIB_CLK_STATUS_MSS_SPIB_CLK_STATUS_CLKINUSE_MAX       (0x000000FFU)

#define SDL_MSS_RCM_MSS_SPIB_CLK_STATUS_MSS_SPIB_CLK_STATUS_CURRDIVIDER_MASK   (0x0000FF00U)
#define SDL_MSS_RCM_MSS_SPIB_CLK_STATUS_MSS_SPIB_CLK_STATUS_CURRDIVIDER_SHIFT  (0x00000008U)
#define SDL_MSS_RCM_MSS_SPIB_CLK_STATUS_MSS_SPIB_CLK_STATUS_CURRDIVIDER_RESETVAL (0x00000000U)
#define SDL_MSS_RCM_MSS_SPIB_CLK_STATUS_MSS_SPIB_CLK_STATUS_CURRDIVIDER_MAX    (0x000000FFU)

#define SDL_MSS_RCM_MSS_SPIB_CLK_STATUS_RESETVAL                               (0x00000001U)

/* MSS_I2C_CLK_STATUS */

#define SDL_MSS_RCM_MSS_I2C_CLK_STATUS_MSS_I2C_CLK_STATUS_CLKINUSE_MASK        (0x000000FFU)
#define SDL_MSS_RCM_MSS_I2C_CLK_STATUS_MSS_I2C_CLK_STATUS_CLKINUSE_SHIFT       (0x00000000U)
#define SDL_MSS_RCM_MSS_I2C_CLK_STATUS_MSS_I2C_CLK_STATUS_CLKINUSE_RESETVAL    (0x00000001U)
#define SDL_MSS_RCM_MSS_I2C_CLK_STATUS_MSS_I2C_CLK_STATUS_CLKINUSE_MAX         (0x000000FFU)

#define SDL_MSS_RCM_MSS_I2C_CLK_STATUS_MSS_I2C_CLK_STATUS_CURRDIVIDER_MASK     (0x0000FF00U)
#define SDL_MSS_RCM_MSS_I2C_CLK_STATUS_MSS_I2C_CLK_STATUS_CURRDIVIDER_SHIFT    (0x00000008U)
#define SDL_MSS_RCM_MSS_I2C_CLK_STATUS_MSS_I2C_CLK_STATUS_CURRDIVIDER_RESETVAL (0x00000000U)
#define SDL_MSS_RCM_MSS_I2C_CLK_STATUS_MSS_I2C_CLK_STATUS_CURRDIVIDER_MAX      (0x000000FFU)

#define SDL_MSS_RCM_MSS_I2C_CLK_STATUS_RESETVAL                                (0x00000001U)

/* MSS_SCIA_CLK_STATUS */

#define SDL_MSS_RCM_MSS_SCIA_CLK_STATUS_MSS_SCIA_CLK_STATUS_CLKINUSE_MASK      (0x000000FFU)
#define SDL_MSS_RCM_MSS_SCIA_CLK_STATUS_MSS_SCIA_CLK_STATUS_CLKINUSE_SHIFT     (0x00000000U)
#define SDL_MSS_RCM_MSS_SCIA_CLK_STATUS_MSS_SCIA_CLK_STATUS_CLKINUSE_RESETVAL  (0x00000001U)
#define SDL_MSS_RCM_MSS_SCIA_CLK_STATUS_MSS_SCIA_CLK_STATUS_CLKINUSE_MAX       (0x000000FFU)

#define SDL_MSS_RCM_MSS_SCIA_CLK_STATUS_MSS_SCIA_CLK_STATUS_CURRDIVIDER_MASK   (0x0000FF00U)
#define SDL_MSS_RCM_MSS_SCIA_CLK_STATUS_MSS_SCIA_CLK_STATUS_CURRDIVIDER_SHIFT  (0x00000008U)
#define SDL_MSS_RCM_MSS_SCIA_CLK_STATUS_MSS_SCIA_CLK_STATUS_CURRDIVIDER_RESETVAL (0x00000000U)
#define SDL_MSS_RCM_MSS_SCIA_CLK_STATUS_MSS_SCIA_CLK_STATUS_CURRDIVIDER_MAX    (0x000000FFU)

#define SDL_MSS_RCM_MSS_SCIA_CLK_STATUS_RESETVAL                               (0x00000001U)

/* MSS_SCIB_CLK_STATUS */

#define SDL_MSS_RCM_MSS_SCIB_CLK_STATUS_MSS_SCIB_CLK_STATUS_CLKINUSE_MASK      (0x000000FFU)
#define SDL_MSS_RCM_MSS_SCIB_CLK_STATUS_MSS_SCIB_CLK_STATUS_CLKINUSE_SHIFT     (0x00000000U)
#define SDL_MSS_RCM_MSS_SCIB_CLK_STATUS_MSS_SCIB_CLK_STATUS_CLKINUSE_RESETVAL  (0x00000001U)
#define SDL_MSS_RCM_MSS_SCIB_CLK_STATUS_MSS_SCIB_CLK_STATUS_CLKINUSE_MAX       (0x000000FFU)

#define SDL_MSS_RCM_MSS_SCIB_CLK_STATUS_MSS_SCIB_CLK_STATUS_CURRDIVIDER_MASK   (0x0000FF00U)
#define SDL_MSS_RCM_MSS_SCIB_CLK_STATUS_MSS_SCIB_CLK_STATUS_CURRDIVIDER_SHIFT  (0x00000008U)
#define SDL_MSS_RCM_MSS_SCIB_CLK_STATUS_MSS_SCIB_CLK_STATUS_CURRDIVIDER_RESETVAL (0x00000000U)
#define SDL_MSS_RCM_MSS_SCIB_CLK_STATUS_MSS_SCIB_CLK_STATUS_CURRDIVIDER_MAX    (0x000000FFU)

#define SDL_MSS_RCM_MSS_SCIB_CLK_STATUS_RESETVAL                               (0x00000001U)

/* MSS_CPTS_CLK_STATUS */

#define SDL_MSS_RCM_MSS_CPTS_CLK_STATUS_MSS_CPTS_CLK_STATUS_CLKINUSE_MASK      (0x000000FFU)
#define SDL_MSS_RCM_MSS_CPTS_CLK_STATUS_MSS_CPTS_CLK_STATUS_CLKINUSE_SHIFT     (0x00000000U)
#define SDL_MSS_RCM_MSS_CPTS_CLK_STATUS_MSS_CPTS_CLK_STATUS_CLKINUSE_RESETVAL  (0x00000001U)
#define SDL_MSS_RCM_MSS_CPTS_CLK_STATUS_MSS_CPTS_CLK_STATUS_CLKINUSE_MAX       (0x000000FFU)

#define SDL_MSS_RCM_MSS_CPTS_CLK_STATUS_MSS_CPTS_CLK_STATUS_CURRDIVIDER_MASK   (0x0000FF00U)
#define SDL_MSS_RCM_MSS_CPTS_CLK_STATUS_MSS_CPTS_CLK_STATUS_CURRDIVIDER_SHIFT  (0x00000008U)
#define SDL_MSS_RCM_MSS_CPTS_CLK_STATUS_MSS_CPTS_CLK_STATUS_CURRDIVIDER_RESETVAL (0x00000000U)
#define SDL_MSS_RCM_MSS_CPTS_CLK_STATUS_MSS_CPTS_CLK_STATUS_CURRDIVIDER_MAX    (0x000000FFU)

#define SDL_MSS_RCM_MSS_CPTS_CLK_STATUS_RESETVAL                               (0x00000001U)

/* MSS_CPSW_CLK_STATUS */

#define SDL_MSS_RCM_MSS_CPSW_CLK_STATUS_MSS_CPSW_CLK_STATUS_CLKINUSE_MASK      (0x000000FFU)
#define SDL_MSS_RCM_MSS_CPSW_CLK_STATUS_MSS_CPSW_CLK_STATUS_CLKINUSE_SHIFT     (0x00000000U)
#define SDL_MSS_RCM_MSS_CPSW_CLK_STATUS_MSS_CPSW_CLK_STATUS_CLKINUSE_RESETVAL  (0x00000001U)
#define SDL_MSS_RCM_MSS_CPSW_CLK_STATUS_MSS_CPSW_CLK_STATUS_CLKINUSE_MAX       (0x000000FFU)

#define SDL_MSS_RCM_MSS_CPSW_CLK_STATUS_MSS_CPSW_CLK_STATUS_CURRDIVIDER_MASK   (0x0000FF00U)
#define SDL_MSS_RCM_MSS_CPSW_CLK_STATUS_MSS_CPSW_CLK_STATUS_CURRDIVIDER_SHIFT  (0x00000008U)
#define SDL_MSS_RCM_MSS_CPSW_CLK_STATUS_MSS_CPSW_CLK_STATUS_CURRDIVIDER_RESETVAL (0x00000000U)
#define SDL_MSS_RCM_MSS_CPSW_CLK_STATUS_MSS_CPSW_CLK_STATUS_CURRDIVIDER_MAX    (0x000000FFU)

#define SDL_MSS_RCM_MSS_CPSW_CLK_STATUS_RESETVAL                               (0x00000001U)

/* MSS_RGMII_CLK_STATUS */

#define SDL_MSS_RCM_MSS_RGMII_CLK_STATUS_MSS_RGMII_CLK_STATUS_CURRDIVIDER_MASK (0x0000FF00U)
#define SDL_MSS_RCM_MSS_RGMII_CLK_STATUS_MSS_RGMII_CLK_STATUS_CURRDIVIDER_SHIFT (0x00000008U)
#define SDL_MSS_RCM_MSS_RGMII_CLK_STATUS_MSS_RGMII_CLK_STATUS_CURRDIVIDER_RESETVAL (0x00000001U)
#define SDL_MSS_RCM_MSS_RGMII_CLK_STATUS_MSS_RGMII_CLK_STATUS_CURRDIVIDER_MAX  (0x000000FFU)

#define SDL_MSS_RCM_MSS_RGMII_CLK_STATUS_RESETVAL                              (0x00000100U)

/* MSS_MII100_CLK_STATUS */

#define SDL_MSS_RCM_MSS_MII100_CLK_STATUS_MSS_MII100_CLK_STATUS_CURRDIVIDER_MASK (0x0000FF00U)
#define SDL_MSS_RCM_MSS_MII100_CLK_STATUS_MSS_MII100_CLK_STATUS_CURRDIVIDER_SHIFT (0x00000008U)
#define SDL_MSS_RCM_MSS_MII100_CLK_STATUS_MSS_MII100_CLK_STATUS_CURRDIVIDER_RESETVAL (0x00000001U)
#define SDL_MSS_RCM_MSS_MII100_CLK_STATUS_MSS_MII100_CLK_STATUS_CURRDIVIDER_MAX (0x000000FFU)

#define SDL_MSS_RCM_MSS_MII100_CLK_STATUS_RESETVAL                             (0x00000100U)

/* MSS_MII10_CLK_STATUS */

#define SDL_MSS_RCM_MSS_MII10_CLK_STATUS_MSS_MII10_CLK_STATUS_CURRDIVIDER_MASK (0x0000FF00U)
#define SDL_MSS_RCM_MSS_MII10_CLK_STATUS_MSS_MII10_CLK_STATUS_CURRDIVIDER_SHIFT (0x00000008U)
#define SDL_MSS_RCM_MSS_MII10_CLK_STATUS_MSS_MII10_CLK_STATUS_CURRDIVIDER_RESETVAL (0x00000001U)
#define SDL_MSS_RCM_MSS_MII10_CLK_STATUS_MSS_MII10_CLK_STATUS_CURRDIVIDER_MAX  (0x000000FFU)

#define SDL_MSS_RCM_MSS_MII10_CLK_STATUS_RESETVAL                              (0x00000100U)

/* MSS_GPADC_CLK_STATUS */

#define SDL_MSS_RCM_MSS_GPADC_CLK_STATUS_MSS_GPADC_CLK_STATUS_CURRDIVIDER_MASK (0x0000FF00U)
#define SDL_MSS_RCM_MSS_GPADC_CLK_STATUS_MSS_GPADC_CLK_STATUS_CURRDIVIDER_SHIFT (0x00000008U)
#define SDL_MSS_RCM_MSS_GPADC_CLK_STATUS_MSS_GPADC_CLK_STATUS_CURRDIVIDER_RESETVAL (0x00000000U)
#define SDL_MSS_RCM_MSS_GPADC_CLK_STATUS_MSS_GPADC_CLK_STATUS_CURRDIVIDER_MAX  (0x000000FFU)

#define SDL_MSS_RCM_MSS_GPADC_CLK_STATUS_RESETVAL                              (0x00000000U)

/* MSS_CR5SS_POR_RST_CTRL */

#define SDL_MSS_RCM_MSS_CR5SS_POR_RST_CTRL_MSS_CR5SS_POR_RST_CTRL_ASSERT_MASK  (0x00000007U)
#define SDL_MSS_RCM_MSS_CR5SS_POR_RST_CTRL_MSS_CR5SS_POR_RST_CTRL_ASSERT_SHIFT (0x00000000U)
#define SDL_MSS_RCM_MSS_CR5SS_POR_RST_CTRL_MSS_CR5SS_POR_RST_CTRL_ASSERT_RESETVAL (0x00000000U)
#define SDL_MSS_RCM_MSS_CR5SS_POR_RST_CTRL_MSS_CR5SS_POR_RST_CTRL_ASSERT_MAX   (0x00000007U)

#define SDL_MSS_RCM_MSS_CR5SS_POR_RST_CTRL_RESETVAL                            (0x00000000U)

/* MSS_CR5SSA_RST_CTRL */

#define SDL_MSS_RCM_MSS_CR5SSA_RST_CTRL_MSS_CR5SSA_RST_CTRL_ASSERT_MASK        (0x00000007U)
#define SDL_MSS_RCM_MSS_CR5SSA_RST_CTRL_MSS_CR5SSA_RST_CTRL_ASSERT_SHIFT       (0x00000000U)
#define SDL_MSS_RCM_MSS_CR5SSA_RST_CTRL_MSS_CR5SSA_RST_CTRL_ASSERT_RESETVAL    (0x00000000U)
#define SDL_MSS_RCM_MSS_CR5SSA_RST_CTRL_MSS_CR5SSA_RST_CTRL_ASSERT_MAX         (0x00000007U)

#define SDL_MSS_RCM_MSS_CR5SSA_RST_CTRL_RESETVAL                               (0x00000000U)

/* MSS_CR5SSB_RST_CTRL */

#define SDL_MSS_RCM_MSS_CR5SSB_RST_CTRL_MSS_CR5SSB_RST_CTRL_ASSERT_MASK        (0x00000007U)
#define SDL_MSS_RCM_MSS_CR5SSB_RST_CTRL_MSS_CR5SSB_RST_CTRL_ASSERT_SHIFT       (0x00000000U)
#define SDL_MSS_RCM_MSS_CR5SSB_RST_CTRL_MSS_CR5SSB_RST_CTRL_ASSERT_RESETVAL    (0x00000000U)
#define SDL_MSS_RCM_MSS_CR5SSB_RST_CTRL_MSS_CR5SSB_RST_CTRL_ASSERT_MAX         (0x00000007U)

#define SDL_MSS_RCM_MSS_CR5SSB_RST_CTRL_RESETVAL                               (0x00000000U)

/* MSS_CR5A_RST_CTRL */

#define SDL_MSS_RCM_MSS_CR5A_RST_CTRL_MSS_CR5A_RST_CTRL_ASSERT_MASK            (0x00000007U)
#define SDL_MSS_RCM_MSS_CR5A_RST_CTRL_MSS_CR5A_RST_CTRL_ASSERT_SHIFT           (0x00000000U)
#define SDL_MSS_RCM_MSS_CR5A_RST_CTRL_MSS_CR5A_RST_CTRL_ASSERT_RESETVAL        (0x00000000U)
#define SDL_MSS_RCM_MSS_CR5A_RST_CTRL_MSS_CR5A_RST_CTRL_ASSERT_MAX             (0x00000007U)

#define SDL_MSS_RCM_MSS_CR5A_RST_CTRL_RESETVAL                                 (0x00000000U)

/* MSS_CR5B_RST_CTRL */

#define SDL_MSS_RCM_MSS_CR5B_RST_CTRL_MSS_CR5B_RST_CTRL_ASSERT_MASK            (0x00000007U)
#define SDL_MSS_RCM_MSS_CR5B_RST_CTRL_MSS_CR5B_RST_CTRL_ASSERT_SHIFT           (0x00000000U)
#define SDL_MSS_RCM_MSS_CR5B_RST_CTRL_MSS_CR5B_RST_CTRL_ASSERT_RESETVAL        (0x00000000U)
#define SDL_MSS_RCM_MSS_CR5B_RST_CTRL_MSS_CR5B_RST_CTRL_ASSERT_MAX             (0x00000007U)

#define SDL_MSS_RCM_MSS_CR5B_RST_CTRL_RESETVAL                                 (0x00000000U)

/* MSS_VIMA_RST_CTRL */

#define SDL_MSS_RCM_MSS_VIMA_RST_CTRL_MSS_VIMA_RST_CTRL_ASSERT_MASK            (0x00000007U)
#define SDL_MSS_RCM_MSS_VIMA_RST_CTRL_MSS_VIMA_RST_CTRL_ASSERT_SHIFT           (0x00000000U)
#define SDL_MSS_RCM_MSS_VIMA_RST_CTRL_MSS_VIMA_RST_CTRL_ASSERT_RESETVAL        (0x00000000U)
#define SDL_MSS_RCM_MSS_VIMA_RST_CTRL_MSS_VIMA_RST_CTRL_ASSERT_MAX             (0x00000007U)

#define SDL_MSS_RCM_MSS_VIMA_RST_CTRL_RESETVAL                                 (0x00000000U)

/* MSS_VIMB_RST_CTRL */

#define SDL_MSS_RCM_MSS_VIMB_RST_CTRL_MSS_VIMB_RST_CTRL_ASSERT_MASK            (0x00000007U)
#define SDL_MSS_RCM_MSS_VIMB_RST_CTRL_MSS_VIMB_RST_CTRL_ASSERT_SHIFT           (0x00000000U)
#define SDL_MSS_RCM_MSS_VIMB_RST_CTRL_MSS_VIMB_RST_CTRL_ASSERT_RESETVAL        (0x00000000U)
#define SDL_MSS_RCM_MSS_VIMB_RST_CTRL_MSS_VIMB_RST_CTRL_ASSERT_MAX             (0x00000007U)

#define SDL_MSS_RCM_MSS_VIMB_RST_CTRL_RESETVAL                                 (0x00000000U)

/* MSS_CRC_RST_CTRL */

#define SDL_MSS_RCM_MSS_CRC_RST_CTRL_MSS_CRC_RST_CTRL_ASSERT_MASK              (0x00000007U)
#define SDL_MSS_RCM_MSS_CRC_RST_CTRL_MSS_CRC_RST_CTRL_ASSERT_SHIFT             (0x00000000U)
#define SDL_MSS_RCM_MSS_CRC_RST_CTRL_MSS_CRC_RST_CTRL_ASSERT_RESETVAL          (0x00000000U)
#define SDL_MSS_RCM_MSS_CRC_RST_CTRL_MSS_CRC_RST_CTRL_ASSERT_MAX               (0x00000007U)

#define SDL_MSS_RCM_MSS_CRC_RST_CTRL_RESETVAL                                  (0x00000000U)

/* MSS_RTIA_RST_CTRL */

#define SDL_MSS_RCM_MSS_RTIA_RST_CTRL_MSS_RTIA_RST_CTRL_ASSERT_MASK            (0x00000007U)
#define SDL_MSS_RCM_MSS_RTIA_RST_CTRL_MSS_RTIA_RST_CTRL_ASSERT_SHIFT           (0x00000000U)
#define SDL_MSS_RCM_MSS_RTIA_RST_CTRL_MSS_RTIA_RST_CTRL_ASSERT_RESETVAL        (0x00000000U)
#define SDL_MSS_RCM_MSS_RTIA_RST_CTRL_MSS_RTIA_RST_CTRL_ASSERT_MAX             (0x00000007U)

#define SDL_MSS_RCM_MSS_RTIA_RST_CTRL_RESETVAL                                 (0x00000000U)

/* MSS_RTIB_RST_CTRL */

#define SDL_MSS_RCM_MSS_RTIB_RST_CTRL_MSS_RTIB_RST_CTRL_ASSERT_MASK            (0x00000007U)
#define SDL_MSS_RCM_MSS_RTIB_RST_CTRL_MSS_RTIB_RST_CTRL_ASSERT_SHIFT           (0x00000000U)
#define SDL_MSS_RCM_MSS_RTIB_RST_CTRL_MSS_RTIB_RST_CTRL_ASSERT_RESETVAL        (0x00000000U)
#define SDL_MSS_RCM_MSS_RTIB_RST_CTRL_MSS_RTIB_RST_CTRL_ASSERT_MAX             (0x00000007U)

#define SDL_MSS_RCM_MSS_RTIB_RST_CTRL_RESETVAL                                 (0x00000000U)

/* MSS_RTIC_RST_CTRL */

#define SDL_MSS_RCM_MSS_RTIC_RST_CTRL_MSS_RTIC_RST_CTRL_ASSERT_MASK            (0x00000007U)
#define SDL_MSS_RCM_MSS_RTIC_RST_CTRL_MSS_RTIC_RST_CTRL_ASSERT_SHIFT           (0x00000000U)
#define SDL_MSS_RCM_MSS_RTIC_RST_CTRL_MSS_RTIC_RST_CTRL_ASSERT_RESETVAL        (0x00000000U)
#define SDL_MSS_RCM_MSS_RTIC_RST_CTRL_MSS_RTIC_RST_CTRL_ASSERT_MAX             (0x00000007U)

#define SDL_MSS_RCM_MSS_RTIC_RST_CTRL_RESETVAL                                 (0x00000000U)

/* MSS_WDT_RST_CTRL */

#define SDL_MSS_RCM_MSS_WDT_RST_CTRL_MSS_WDT_RST_CTRL_ASSERT_MASK              (0x00000007U)
#define SDL_MSS_RCM_MSS_WDT_RST_CTRL_MSS_WDT_RST_CTRL_ASSERT_SHIFT             (0x00000000U)
#define SDL_MSS_RCM_MSS_WDT_RST_CTRL_MSS_WDT_RST_CTRL_ASSERT_RESETVAL          (0x00000000U)
#define SDL_MSS_RCM_MSS_WDT_RST_CTRL_MSS_WDT_RST_CTRL_ASSERT_MAX               (0x00000007U)

#define SDL_MSS_RCM_MSS_WDT_RST_CTRL_RESETVAL                                  (0x00000000U)

/* MSS_ESM_RST_CTRL */

#define SDL_MSS_RCM_MSS_ESM_RST_CTRL_MSS_ESM_RST_CTRL_ASSERT_MASK              (0x00000007U)
#define SDL_MSS_RCM_MSS_ESM_RST_CTRL_MSS_ESM_RST_CTRL_ASSERT_SHIFT             (0x00000000U)
#define SDL_MSS_RCM_MSS_ESM_RST_CTRL_MSS_ESM_RST_CTRL_ASSERT_RESETVAL          (0x00000000U)
#define SDL_MSS_RCM_MSS_ESM_RST_CTRL_MSS_ESM_RST_CTRL_ASSERT_MAX               (0x00000007U)

#define SDL_MSS_RCM_MSS_ESM_RST_CTRL_RESETVAL                                  (0x00000000U)

/* MSS_DCCA_RST_CTRL */

#define SDL_MSS_RCM_MSS_DCCA_RST_CTRL_MSS_DCCA_RST_CTRL_ASSERT_MASK            (0x00000007U)
#define SDL_MSS_RCM_MSS_DCCA_RST_CTRL_MSS_DCCA_RST_CTRL_ASSERT_SHIFT           (0x00000000U)
#define SDL_MSS_RCM_MSS_DCCA_RST_CTRL_MSS_DCCA_RST_CTRL_ASSERT_RESETVAL        (0x00000000U)
#define SDL_MSS_RCM_MSS_DCCA_RST_CTRL_MSS_DCCA_RST_CTRL_ASSERT_MAX             (0x00000007U)

#define SDL_MSS_RCM_MSS_DCCA_RST_CTRL_RESETVAL                                 (0x00000000U)

/* MSS_DCCB_RST_CTRL */

#define SDL_MSS_RCM_MSS_DCCB_RST_CTRL_MSS_DCCB_RST_CTRL_ASSERT_MASK            (0x00000007U)
#define SDL_MSS_RCM_MSS_DCCB_RST_CTRL_MSS_DCCB_RST_CTRL_ASSERT_SHIFT           (0x00000000U)
#define SDL_MSS_RCM_MSS_DCCB_RST_CTRL_MSS_DCCB_RST_CTRL_ASSERT_RESETVAL        (0x00000000U)
#define SDL_MSS_RCM_MSS_DCCB_RST_CTRL_MSS_DCCB_RST_CTRL_ASSERT_MAX             (0x00000007U)

#define SDL_MSS_RCM_MSS_DCCB_RST_CTRL_RESETVAL                                 (0x00000000U)

/* MSS_DCCC_RST_CTRL */

#define SDL_MSS_RCM_MSS_DCCC_RST_CTRL_MSS_DCCC_RST_CTRL_ASSERT_MASK            (0x00000007U)
#define SDL_MSS_RCM_MSS_DCCC_RST_CTRL_MSS_DCCC_RST_CTRL_ASSERT_SHIFT           (0x00000000U)
#define SDL_MSS_RCM_MSS_DCCC_RST_CTRL_MSS_DCCC_RST_CTRL_ASSERT_RESETVAL        (0x00000000U)
#define SDL_MSS_RCM_MSS_DCCC_RST_CTRL_MSS_DCCC_RST_CTRL_ASSERT_MAX             (0x00000007U)

#define SDL_MSS_RCM_MSS_DCCC_RST_CTRL_RESETVAL                                 (0x00000000U)

/* MSS_DCCD_RST_CTRL */

#define SDL_MSS_RCM_MSS_DCCD_RST_CTRL_MSS_DCCD_RST_CTRL_ASSERT_MASK            (0x00000007U)
#define SDL_MSS_RCM_MSS_DCCD_RST_CTRL_MSS_DCCD_RST_CTRL_ASSERT_SHIFT           (0x00000000U)
#define SDL_MSS_RCM_MSS_DCCD_RST_CTRL_MSS_DCCD_RST_CTRL_ASSERT_RESETVAL        (0x00000000U)
#define SDL_MSS_RCM_MSS_DCCD_RST_CTRL_MSS_DCCD_RST_CTRL_ASSERT_MAX             (0x00000007U)

#define SDL_MSS_RCM_MSS_DCCD_RST_CTRL_RESETVAL                                 (0x00000000U)

/* MSS_GIO_RST_CTRL */

#define SDL_MSS_RCM_MSS_GIO_RST_CTRL_MSS_GIO_RST_CTRL_ASSERT_MASK              (0x00000007U)
#define SDL_MSS_RCM_MSS_GIO_RST_CTRL_MSS_GIO_RST_CTRL_ASSERT_SHIFT             (0x00000000U)
#define SDL_MSS_RCM_MSS_GIO_RST_CTRL_MSS_GIO_RST_CTRL_ASSERT_RESETVAL          (0x00000000U)
#define SDL_MSS_RCM_MSS_GIO_RST_CTRL_MSS_GIO_RST_CTRL_ASSERT_MAX               (0x00000007U)

#define SDL_MSS_RCM_MSS_GIO_RST_CTRL_RESETVAL                                  (0x00000000U)

/* MSS_SPIA_RST_CTRL */

#define SDL_MSS_RCM_MSS_SPIA_RST_CTRL_MSS_SPIA_RST_CTRL_ASSERT_MASK            (0x00000007U)
#define SDL_MSS_RCM_MSS_SPIA_RST_CTRL_MSS_SPIA_RST_CTRL_ASSERT_SHIFT           (0x00000000U)
#define SDL_MSS_RCM_MSS_SPIA_RST_CTRL_MSS_SPIA_RST_CTRL_ASSERT_RESETVAL        (0x00000000U)
#define SDL_MSS_RCM_MSS_SPIA_RST_CTRL_MSS_SPIA_RST_CTRL_ASSERT_MAX             (0x00000007U)

#define SDL_MSS_RCM_MSS_SPIA_RST_CTRL_RESETVAL                                 (0x00000000U)

/* MSS_SPIB_RST_CTRL */

#define SDL_MSS_RCM_MSS_SPIB_RST_CTRL_MSS_SPIB_RST_CTRL_ASSERT_MASK            (0x00000007U)
#define SDL_MSS_RCM_MSS_SPIB_RST_CTRL_MSS_SPIB_RST_CTRL_ASSERT_SHIFT           (0x00000000U)
#define SDL_MSS_RCM_MSS_SPIB_RST_CTRL_MSS_SPIB_RST_CTRL_ASSERT_RESETVAL        (0x00000000U)
#define SDL_MSS_RCM_MSS_SPIB_RST_CTRL_MSS_SPIB_RST_CTRL_ASSERT_MAX             (0x00000007U)

#define SDL_MSS_RCM_MSS_SPIB_RST_CTRL_RESETVAL                                 (0x00000000U)

/* MSS_QSPI_RST_CTRL */

#define SDL_MSS_RCM_MSS_QSPI_RST_CTRL_MSS_QSPI_RST_CTRL_ASSERT_MASK            (0x00000007U)
#define SDL_MSS_RCM_MSS_QSPI_RST_CTRL_MSS_QSPI_RST_CTRL_ASSERT_SHIFT           (0x00000000U)
#define SDL_MSS_RCM_MSS_QSPI_RST_CTRL_MSS_QSPI_RST_CTRL_ASSERT_RESETVAL        (0x00000000U)
#define SDL_MSS_RCM_MSS_QSPI_RST_CTRL_MSS_QSPI_RST_CTRL_ASSERT_MAX             (0x00000007U)

#define SDL_MSS_RCM_MSS_QSPI_RST_CTRL_RESETVAL                                 (0x00000000U)

/* MSS_PWM1_RST_CTRL */

#define SDL_MSS_RCM_MSS_PWM1_RST_CTRL_MSS_PWM1_RST_CTRL_ASSERT_MASK            (0x00000007U)
#define SDL_MSS_RCM_MSS_PWM1_RST_CTRL_MSS_PWM1_RST_CTRL_ASSERT_SHIFT           (0x00000000U)
#define SDL_MSS_RCM_MSS_PWM1_RST_CTRL_MSS_PWM1_RST_CTRL_ASSERT_RESETVAL        (0x00000000U)
#define SDL_MSS_RCM_MSS_PWM1_RST_CTRL_MSS_PWM1_RST_CTRL_ASSERT_MAX             (0x00000007U)

#define SDL_MSS_RCM_MSS_PWM1_RST_CTRL_RESETVAL                                 (0x00000000U)

/* MSS_PWM2_RST_CTRL */

#define SDL_MSS_RCM_MSS_PWM2_RST_CTRL_MSS_PWM2_RST_CTRL_ASSERT_MASK            (0x00000007U)
#define SDL_MSS_RCM_MSS_PWM2_RST_CTRL_MSS_PWM2_RST_CTRL_ASSERT_SHIFT           (0x00000000U)
#define SDL_MSS_RCM_MSS_PWM2_RST_CTRL_MSS_PWM2_RST_CTRL_ASSERT_RESETVAL        (0x00000000U)
#define SDL_MSS_RCM_MSS_PWM2_RST_CTRL_MSS_PWM2_RST_CTRL_ASSERT_MAX             (0x00000007U)

#define SDL_MSS_RCM_MSS_PWM2_RST_CTRL_RESETVAL                                 (0x00000000U)

/* MSS_PWM3_RST_CTRL */

#define SDL_MSS_RCM_MSS_PWM3_RST_CTRL_MSS_PWM3_RST_CTRL_ASSERT_MASK            (0x00000007U)
#define SDL_MSS_RCM_MSS_PWM3_RST_CTRL_MSS_PWM3_RST_CTRL_ASSERT_SHIFT           (0x00000000U)
#define SDL_MSS_RCM_MSS_PWM3_RST_CTRL_MSS_PWM3_RST_CTRL_ASSERT_RESETVAL        (0x00000000U)
#define SDL_MSS_RCM_MSS_PWM3_RST_CTRL_MSS_PWM3_RST_CTRL_ASSERT_MAX             (0x00000007U)

#define SDL_MSS_RCM_MSS_PWM3_RST_CTRL_RESETVAL                                 (0x00000000U)

/* MSS_MCANA_RST_CTRL */

#define SDL_MSS_RCM_MSS_MCANA_RST_CTRL_MSS_MCANA_RST_CTRL_ASSERT_MASK          (0x00000007U)
#define SDL_MSS_RCM_MSS_MCANA_RST_CTRL_MSS_MCANA_RST_CTRL_ASSERT_SHIFT         (0x00000000U)
#define SDL_MSS_RCM_MSS_MCANA_RST_CTRL_MSS_MCANA_RST_CTRL_ASSERT_RESETVAL      (0x00000000U)
#define SDL_MSS_RCM_MSS_MCANA_RST_CTRL_MSS_MCANA_RST_CTRL_ASSERT_MAX           (0x00000007U)

#define SDL_MSS_RCM_MSS_MCANA_RST_CTRL_RESETVAL                                (0x00000000U)

/* MSS_MCANB_RST_CTRL */

#define SDL_MSS_RCM_MSS_MCANB_RST_CTRL_MSS_MCANB_RST_CTRL_ASSERT_MASK          (0x00000007U)
#define SDL_MSS_RCM_MSS_MCANB_RST_CTRL_MSS_MCANB_RST_CTRL_ASSERT_SHIFT         (0x00000000U)
#define SDL_MSS_RCM_MSS_MCANB_RST_CTRL_MSS_MCANB_RST_CTRL_ASSERT_RESETVAL      (0x00000000U)
#define SDL_MSS_RCM_MSS_MCANB_RST_CTRL_MSS_MCANB_RST_CTRL_ASSERT_MAX           (0x00000007U)

#define SDL_MSS_RCM_MSS_MCANB_RST_CTRL_RESETVAL                                (0x00000000U)

/* MSS_I2C_RST_CTRL */

#define SDL_MSS_RCM_MSS_I2C_RST_CTRL_MSS_I2C_RST_CTRL_ASSERT_MASK              (0x00000007U)
#define SDL_MSS_RCM_MSS_I2C_RST_CTRL_MSS_I2C_RST_CTRL_ASSERT_SHIFT             (0x00000000U)
#define SDL_MSS_RCM_MSS_I2C_RST_CTRL_MSS_I2C_RST_CTRL_ASSERT_RESETVAL          (0x00000000U)
#define SDL_MSS_RCM_MSS_I2C_RST_CTRL_MSS_I2C_RST_CTRL_ASSERT_MAX               (0x00000007U)

#define SDL_MSS_RCM_MSS_I2C_RST_CTRL_RESETVAL                                  (0x00000000U)

/* MSS_SCIA_RST_CTRL */

#define SDL_MSS_RCM_MSS_SCIA_RST_CTRL_MSS_SCIA_RST_CTRL_ASSERT_MASK            (0x00000007U)
#define SDL_MSS_RCM_MSS_SCIA_RST_CTRL_MSS_SCIA_RST_CTRL_ASSERT_SHIFT           (0x00000000U)
#define SDL_MSS_RCM_MSS_SCIA_RST_CTRL_MSS_SCIA_RST_CTRL_ASSERT_RESETVAL        (0x00000000U)
#define SDL_MSS_RCM_MSS_SCIA_RST_CTRL_MSS_SCIA_RST_CTRL_ASSERT_MAX             (0x00000007U)

#define SDL_MSS_RCM_MSS_SCIA_RST_CTRL_RESETVAL                                 (0x00000000U)

/* MSS_SCIB_RST_CTRL */

#define SDL_MSS_RCM_MSS_SCIB_RST_CTRL_MSS_SCIB_RST_CTRL_ASSERT_MASK            (0x00000007U)
#define SDL_MSS_RCM_MSS_SCIB_RST_CTRL_MSS_SCIB_RST_CTRL_ASSERT_SHIFT           (0x00000000U)
#define SDL_MSS_RCM_MSS_SCIB_RST_CTRL_MSS_SCIB_RST_CTRL_ASSERT_RESETVAL        (0x00000000U)
#define SDL_MSS_RCM_MSS_SCIB_RST_CTRL_MSS_SCIB_RST_CTRL_ASSERT_MAX             (0x00000007U)

#define SDL_MSS_RCM_MSS_SCIB_RST_CTRL_RESETVAL                                 (0x00000000U)

/* MSS_EDMA_RST_CTRL */

#define SDL_MSS_RCM_MSS_EDMA_RST_CTRL_MSS_EDMA_RST_CTRL_ASSERT_MASK            (0x00000007U)
#define SDL_MSS_RCM_MSS_EDMA_RST_CTRL_MSS_EDMA_RST_CTRL_ASSERT_SHIFT           (0x00000000U)
#define SDL_MSS_RCM_MSS_EDMA_RST_CTRL_MSS_EDMA_RST_CTRL_ASSERT_RESETVAL        (0x00000000U)
#define SDL_MSS_RCM_MSS_EDMA_RST_CTRL_MSS_EDMA_RST_CTRL_ASSERT_MAX             (0x00000007U)

#define SDL_MSS_RCM_MSS_EDMA_RST_CTRL_MSS_EDMA_RST_CTRL_TPCCA_ASSERT_MASK      (0x00000070U)
#define SDL_MSS_RCM_MSS_EDMA_RST_CTRL_MSS_EDMA_RST_CTRL_TPCCA_ASSERT_SHIFT     (0x00000004U)
#define SDL_MSS_RCM_MSS_EDMA_RST_CTRL_MSS_EDMA_RST_CTRL_TPCCA_ASSERT_RESETVAL  (0x00000000U)
#define SDL_MSS_RCM_MSS_EDMA_RST_CTRL_MSS_EDMA_RST_CTRL_TPCCA_ASSERT_MAX       (0x00000007U)

#define SDL_MSS_RCM_MSS_EDMA_RST_CTRL_MSS_EDMA_RST_CTRL_TPTCA0_ASSERT_MASK     (0x00000700U)
#define SDL_MSS_RCM_MSS_EDMA_RST_CTRL_MSS_EDMA_RST_CTRL_TPTCA0_ASSERT_SHIFT    (0x00000008U)
#define SDL_MSS_RCM_MSS_EDMA_RST_CTRL_MSS_EDMA_RST_CTRL_TPTCA0_ASSERT_RESETVAL (0x00000000U)
#define SDL_MSS_RCM_MSS_EDMA_RST_CTRL_MSS_EDMA_RST_CTRL_TPTCA0_ASSERT_MAX      (0x00000007U)

#define SDL_MSS_RCM_MSS_EDMA_RST_CTRL_MSS_EDMA_RST_CTRL_TPTCA1_ASSERT_MASK     (0x00007000U)
#define SDL_MSS_RCM_MSS_EDMA_RST_CTRL_MSS_EDMA_RST_CTRL_TPTCA1_ASSERT_SHIFT    (0x0000000CU)
#define SDL_MSS_RCM_MSS_EDMA_RST_CTRL_MSS_EDMA_RST_CTRL_TPTCA1_ASSERT_RESETVAL (0x00000000U)
#define SDL_MSS_RCM_MSS_EDMA_RST_CTRL_MSS_EDMA_RST_CTRL_TPTCA1_ASSERT_MAX      (0x00000007U)

#define SDL_MSS_RCM_MSS_EDMA_RST_CTRL_MSS_EDMA_RST_CTRL_TPCCB_ASSERT_MASK      (0x00070000U)
#define SDL_MSS_RCM_MSS_EDMA_RST_CTRL_MSS_EDMA_RST_CTRL_TPCCB_ASSERT_SHIFT     (0x00000010U)
#define SDL_MSS_RCM_MSS_EDMA_RST_CTRL_MSS_EDMA_RST_CTRL_TPCCB_ASSERT_RESETVAL  (0x00000000U)
#define SDL_MSS_RCM_MSS_EDMA_RST_CTRL_MSS_EDMA_RST_CTRL_TPCCB_ASSERT_MAX       (0x00000007U)

#define SDL_MSS_RCM_MSS_EDMA_RST_CTRL_MSS_EDMA_RST_CTRL_TPTCB0_ASSERT_MASK     (0x07000000U)
#define SDL_MSS_RCM_MSS_EDMA_RST_CTRL_MSS_EDMA_RST_CTRL_TPTCB0_ASSERT_SHIFT    (0x00000018U)
#define SDL_MSS_RCM_MSS_EDMA_RST_CTRL_MSS_EDMA_RST_CTRL_TPTCB0_ASSERT_RESETVAL (0x00000000U)
#define SDL_MSS_RCM_MSS_EDMA_RST_CTRL_MSS_EDMA_RST_CTRL_TPTCB0_ASSERT_MAX      (0x00000007U)

#define SDL_MSS_RCM_MSS_EDMA_RST_CTRL_RESETVAL                                 (0x00000000U)

/* MSS_INFRA_RST_CTRL */

#define SDL_MSS_RCM_MSS_INFRA_RST_CTRL_MSS_INFRA_RST_CTRL_ASSERT_MASK          (0x00000007U)
#define SDL_MSS_RCM_MSS_INFRA_RST_CTRL_MSS_INFRA_RST_CTRL_ASSERT_SHIFT         (0x00000000U)
#define SDL_MSS_RCM_MSS_INFRA_RST_CTRL_MSS_INFRA_RST_CTRL_ASSERT_RESETVAL      (0x00000000U)
#define SDL_MSS_RCM_MSS_INFRA_RST_CTRL_MSS_INFRA_RST_CTRL_ASSERT_MAX           (0x00000007U)

#define SDL_MSS_RCM_MSS_INFRA_RST_CTRL_RESETVAL                                (0x00000000U)

/* MSS_CPSW_RST_CTRL */

#define SDL_MSS_RCM_MSS_CPSW_RST_CTRL_MSS_CPSW_RST_CTRL_ASSERT_MASK            (0x00000007U)
#define SDL_MSS_RCM_MSS_CPSW_RST_CTRL_MSS_CPSW_RST_CTRL_ASSERT_SHIFT           (0x00000000U)
#define SDL_MSS_RCM_MSS_CPSW_RST_CTRL_MSS_CPSW_RST_CTRL_ASSERT_RESETVAL        (0x00000000U)
#define SDL_MSS_RCM_MSS_CPSW_RST_CTRL_MSS_CPSW_RST_CTRL_ASSERT_MAX             (0x00000007U)

#define SDL_MSS_RCM_MSS_CPSW_RST_CTRL_RESETVAL                                 (0x00000000U)

/* MSS_GPADC_RST_CTRL */

#define SDL_MSS_RCM_MSS_GPADC_RST_CTRL_MSS_GPADC_RST_CTRL_ASSERT_MASK          (0x00000007U)
#define SDL_MSS_RCM_MSS_GPADC_RST_CTRL_MSS_GPADC_RST_CTRL_ASSERT_SHIFT         (0x00000000U)
#define SDL_MSS_RCM_MSS_GPADC_RST_CTRL_MSS_GPADC_RST_CTRL_ASSERT_RESETVAL      (0x00000000U)
#define SDL_MSS_RCM_MSS_GPADC_RST_CTRL_MSS_GPADC_RST_CTRL_ASSERT_MAX           (0x00000007U)

#define SDL_MSS_RCM_MSS_GPADC_RST_CTRL_RESETVAL                                (0x00000000U)

/* MSS_DMM_RST_CTRL */

#define SDL_MSS_RCM_MSS_DMM_RST_CTRL_MSS_DMM_RST_CTRL_ASSERT_MASK              (0x00000007U)
#define SDL_MSS_RCM_MSS_DMM_RST_CTRL_MSS_DMM_RST_CTRL_ASSERT_SHIFT             (0x00000000U)
#define SDL_MSS_RCM_MSS_DMM_RST_CTRL_MSS_DMM_RST_CTRL_ASSERT_RESETVAL          (0x00000000U)
#define SDL_MSS_RCM_MSS_DMM_RST_CTRL_MSS_DMM_RST_CTRL_ASSERT_MAX               (0x00000007U)

#define SDL_MSS_RCM_MSS_DMM_RST_CTRL_RESETVAL                                  (0x00000000U)

/* R5_COREA_GATE */

#define SDL_MSS_RCM_R5_COREA_GATE_R5_COREA_GATE_CLKGATE_MASK                   (0x00000007U)
#define SDL_MSS_RCM_R5_COREA_GATE_R5_COREA_GATE_CLKGATE_SHIFT                  (0x00000000U)
#define SDL_MSS_RCM_R5_COREA_GATE_R5_COREA_GATE_CLKGATE_RESETVAL               (0x00000000U)
#define SDL_MSS_RCM_R5_COREA_GATE_R5_COREA_GATE_CLKGATE_MAX                    (0x00000007U)

#define SDL_MSS_RCM_R5_COREA_GATE_RESETVAL                                     (0x00000000U)

/* R5_COREB_GATE */

#define SDL_MSS_RCM_R5_COREB_GATE_R5_COREB_GATE_CLKGATE_MASK                   (0x00000007U)
#define SDL_MSS_RCM_R5_COREB_GATE_R5_COREB_GATE_CLKGATE_SHIFT                  (0x00000000U)
#define SDL_MSS_RCM_R5_COREB_GATE_R5_COREB_GATE_CLKGATE_RESETVAL               (0x00000000U)
#define SDL_MSS_RCM_R5_COREB_GATE_R5_COREB_GATE_CLKGATE_MAX                    (0x00000007U)

#define SDL_MSS_RCM_R5_COREB_GATE_RESETVAL                                     (0x00000000U)

/* MSS_L2_BANKA_PD_CTRL */

#define SDL_MSS_RCM_MSS_L2_BANKA_PD_CTRL_MSS_L2_BANKA_PD_CTRL_ISO_MASK         (0x00000007U)
#define SDL_MSS_RCM_MSS_L2_BANKA_PD_CTRL_MSS_L2_BANKA_PD_CTRL_ISO_SHIFT        (0x00000000U)
#define SDL_MSS_RCM_MSS_L2_BANKA_PD_CTRL_MSS_L2_BANKA_PD_CTRL_ISO_RESETVAL     (0x00000000U)
#define SDL_MSS_RCM_MSS_L2_BANKA_PD_CTRL_MSS_L2_BANKA_PD_CTRL_ISO_MAX          (0x00000007U)

#define SDL_MSS_RCM_MSS_L2_BANKA_PD_CTRL_MSS_L2_BANKA_PD_CTRL_AONIN_MASK       (0x00000070U)
#define SDL_MSS_RCM_MSS_L2_BANKA_PD_CTRL_MSS_L2_BANKA_PD_CTRL_AONIN_SHIFT      (0x00000004U)
#define SDL_MSS_RCM_MSS_L2_BANKA_PD_CTRL_MSS_L2_BANKA_PD_CTRL_AONIN_RESETVAL   (0x00000007U)
#define SDL_MSS_RCM_MSS_L2_BANKA_PD_CTRL_MSS_L2_BANKA_PD_CTRL_AONIN_MAX        (0x00000007U)

#define SDL_MSS_RCM_MSS_L2_BANKA_PD_CTRL_MSS_L2_BANKA_PD_CTRL_AGOODIN_MASK     (0x00000700U)
#define SDL_MSS_RCM_MSS_L2_BANKA_PD_CTRL_MSS_L2_BANKA_PD_CTRL_AGOODIN_SHIFT    (0x00000008U)
#define SDL_MSS_RCM_MSS_L2_BANKA_PD_CTRL_MSS_L2_BANKA_PD_CTRL_AGOODIN_RESETVAL (0x00000007U)
#define SDL_MSS_RCM_MSS_L2_BANKA_PD_CTRL_MSS_L2_BANKA_PD_CTRL_AGOODIN_MAX      (0x00000007U)

#define SDL_MSS_RCM_MSS_L2_BANKA_PD_CTRL_RESETVAL                              (0x00000770U)

/* MSS_L2_BANKB_PD_CTRL */

#define SDL_MSS_RCM_MSS_L2_BANKB_PD_CTRL_MSS_L2_BANKB_PD_CTRL_ISO_MASK         (0x00000007U)
#define SDL_MSS_RCM_MSS_L2_BANKB_PD_CTRL_MSS_L2_BANKB_PD_CTRL_ISO_SHIFT        (0x00000000U)
#define SDL_MSS_RCM_MSS_L2_BANKB_PD_CTRL_MSS_L2_BANKB_PD_CTRL_ISO_RESETVAL     (0x00000000U)
#define SDL_MSS_RCM_MSS_L2_BANKB_PD_CTRL_MSS_L2_BANKB_PD_CTRL_ISO_MAX          (0x00000007U)

#define SDL_MSS_RCM_MSS_L2_BANKB_PD_CTRL_MSS_L2_BANKB_PD_CTRL_AONIN_MASK       (0x00000070U)
#define SDL_MSS_RCM_MSS_L2_BANKB_PD_CTRL_MSS_L2_BANKB_PD_CTRL_AONIN_SHIFT      (0x00000004U)
#define SDL_MSS_RCM_MSS_L2_BANKB_PD_CTRL_MSS_L2_BANKB_PD_CTRL_AONIN_RESETVAL   (0x00000007U)
#define SDL_MSS_RCM_MSS_L2_BANKB_PD_CTRL_MSS_L2_BANKB_PD_CTRL_AONIN_MAX        (0x00000007U)

#define SDL_MSS_RCM_MSS_L2_BANKB_PD_CTRL_MSS_L2_BANKB_PD_CTRL_AGOODIN_MASK     (0x00000700U)
#define SDL_MSS_RCM_MSS_L2_BANKB_PD_CTRL_MSS_L2_BANKB_PD_CTRL_AGOODIN_SHIFT    (0x00000008U)
#define SDL_MSS_RCM_MSS_L2_BANKB_PD_CTRL_MSS_L2_BANKB_PD_CTRL_AGOODIN_RESETVAL (0x00000007U)
#define SDL_MSS_RCM_MSS_L2_BANKB_PD_CTRL_MSS_L2_BANKB_PD_CTRL_AGOODIN_MAX      (0x00000007U)

#define SDL_MSS_RCM_MSS_L2_BANKB_PD_CTRL_RESETVAL                              (0x00000770U)

/* MSS_L2_BANKA_PD_STATUS */

#define SDL_MSS_RCM_MSS_L2_BANKA_PD_STATUS_MSS_L2_BANKA_PD_STATUS_AONOUT_MASK  (0x00000001U)
#define SDL_MSS_RCM_MSS_L2_BANKA_PD_STATUS_MSS_L2_BANKA_PD_STATUS_AONOUT_SHIFT (0x00000000U)
#define SDL_MSS_RCM_MSS_L2_BANKA_PD_STATUS_MSS_L2_BANKA_PD_STATUS_AONOUT_RESETVAL (0x00000001U)
#define SDL_MSS_RCM_MSS_L2_BANKA_PD_STATUS_MSS_L2_BANKA_PD_STATUS_AONOUT_MAX   (0x00000001U)

#define SDL_MSS_RCM_MSS_L2_BANKA_PD_STATUS_MSS_L2_BANKA_PD_STATUS_AGOODOUT_MASK (0x00000002U)
#define SDL_MSS_RCM_MSS_L2_BANKA_PD_STATUS_MSS_L2_BANKA_PD_STATUS_AGOODOUT_SHIFT (0x00000001U)
#define SDL_MSS_RCM_MSS_L2_BANKA_PD_STATUS_MSS_L2_BANKA_PD_STATUS_AGOODOUT_RESETVAL (0x00000001U)
#define SDL_MSS_RCM_MSS_L2_BANKA_PD_STATUS_MSS_L2_BANKA_PD_STATUS_AGOODOUT_MAX (0x00000001U)

#define SDL_MSS_RCM_MSS_L2_BANKA_PD_STATUS_RESETVAL                            (0x00000003U)

/* MSS_L2_BANKB_PD_STATUS */

#define SDL_MSS_RCM_MSS_L2_BANKB_PD_STATUS_MSS_L2_BANKB_PD_STATUS_AONOUT_MASK  (0x00000001U)
#define SDL_MSS_RCM_MSS_L2_BANKB_PD_STATUS_MSS_L2_BANKB_PD_STATUS_AONOUT_SHIFT (0x00000000U)
#define SDL_MSS_RCM_MSS_L2_BANKB_PD_STATUS_MSS_L2_BANKB_PD_STATUS_AONOUT_RESETVAL (0x00000001U)
#define SDL_MSS_RCM_MSS_L2_BANKB_PD_STATUS_MSS_L2_BANKB_PD_STATUS_AONOUT_MAX   (0x00000001U)

#define SDL_MSS_RCM_MSS_L2_BANKB_PD_STATUS_MSS_L2_BANKB_PD_STATUS_AGOODOUT_MASK (0x00000002U)
#define SDL_MSS_RCM_MSS_L2_BANKB_PD_STATUS_MSS_L2_BANKB_PD_STATUS_AGOODOUT_SHIFT (0x00000001U)
#define SDL_MSS_RCM_MSS_L2_BANKB_PD_STATUS_MSS_L2_BANKB_PD_STATUS_AGOODOUT_RESETVAL (0x00000001U)
#define SDL_MSS_RCM_MSS_L2_BANKB_PD_STATUS_MSS_L2_BANKB_PD_STATUS_AGOODOUT_MAX (0x00000001U)

#define SDL_MSS_RCM_MSS_L2_BANKB_PD_STATUS_RESETVAL                            (0x00000003U)

/* HW_REG0 */

#define SDL_MSS_RCM_HW_REG0_HW_REG0_HWREG_MASK                                 (0xFFFFFFFFU)
#define SDL_MSS_RCM_HW_REG0_HW_REG0_HWREG_SHIFT                                (0x00000000U)
#define SDL_MSS_RCM_HW_REG0_HW_REG0_HWREG_RESETVAL                             (0x00000000U)
#define SDL_MSS_RCM_HW_REG0_HW_REG0_HWREG_MAX                                  (0xFFFFFFFFU)

#define SDL_MSS_RCM_HW_REG0_RESETVAL                                           (0x00000000U)

/* HW_REG1 */

#define SDL_MSS_RCM_HW_REG1_HW_REG1_HWREG_MASK                                 (0xFFFFFFFFU)
#define SDL_MSS_RCM_HW_REG1_HW_REG1_HWREG_SHIFT                                (0x00000000U)
#define SDL_MSS_RCM_HW_REG1_HW_REG1_HWREG_RESETVAL                             (0x00000000U)
#define SDL_MSS_RCM_HW_REG1_HW_REG1_HWREG_MAX                                  (0xFFFFFFFFU)

#define SDL_MSS_RCM_HW_REG1_RESETVAL                                           (0x00000000U)

/* HW_REG2 */

#define SDL_MSS_RCM_HW_REG2_HW_REG2_HWREG_MASK                                 (0xFFFFFFFFU)
#define SDL_MSS_RCM_HW_REG2_HW_REG2_HWREG_SHIFT                                (0x00000000U)
#define SDL_MSS_RCM_HW_REG2_HW_REG2_HWREG_RESETVAL                             (0x00000000U)
#define SDL_MSS_RCM_HW_REG2_HW_REG2_HWREG_MAX                                  (0xFFFFFFFFU)

#define SDL_MSS_RCM_HW_REG2_RESETVAL                                           (0x00000000U)

/* HW_REG3 */

#define SDL_MSS_RCM_HW_REG3_HW_REG3_HWREG_MASK                                 (0xFFFFFFFFU)
#define SDL_MSS_RCM_HW_REG3_HW_REG3_HWREG_SHIFT                                (0x00000000U)
#define SDL_MSS_RCM_HW_REG3_HW_REG3_HWREG_RESETVAL                             (0x00000000U)
#define SDL_MSS_RCM_HW_REG3_HW_REG3_HWREG_MAX                                  (0xFFFFFFFFU)

#define SDL_MSS_RCM_HW_REG3_RESETVAL                                           (0x00000000U)

/* MSS_CR5F_CLK_SRC_SEL_CTRL */

#define SDL_MSS_RCM_MSS_CR5F_CLK_SRC_SEL_CTRL_MSS_CR5F_CLK_SRC_SEL_CTRL_CLKSRCSEL_MASK (0x00000007U)
#define SDL_MSS_RCM_MSS_CR5F_CLK_SRC_SEL_CTRL_MSS_CR5F_CLK_SRC_SEL_CTRL_CLKSRCSEL_SHIFT (0x00000000U)
#define SDL_MSS_RCM_MSS_CR5F_CLK_SRC_SEL_CTRL_MSS_CR5F_CLK_SRC_SEL_CTRL_CLKSRCSEL_RESETVAL (0x00000000U)
#define SDL_MSS_RCM_MSS_CR5F_CLK_SRC_SEL_CTRL_MSS_CR5F_CLK_SRC_SEL_CTRL_CLKSRCSEL_MAX (0x00000007U)

#define SDL_MSS_RCM_MSS_CR5F_CLK_SRC_SEL_CTRL_RESETVAL                         (0x00000000U)

/* MSS_CPSW_MII_CLK_SRC_SEL */

#define SDL_MSS_RCM_MSS_CPSW_MII_CLK_SRC_SEL_MSS_CPSW_MII_CLK_SRC_SEL_CLKSRCSEL_MASK (0x00000FFFU)
#define SDL_MSS_RCM_MSS_CPSW_MII_CLK_SRC_SEL_MSS_CPSW_MII_CLK_SRC_SEL_CLKSRCSEL_SHIFT (0x00000000U)
#define SDL_MSS_RCM_MSS_CPSW_MII_CLK_SRC_SEL_MSS_CPSW_MII_CLK_SRC_SEL_CLKSRCSEL_RESETVAL (0x00000000U)
#define SDL_MSS_RCM_MSS_CPSW_MII_CLK_SRC_SEL_MSS_CPSW_MII_CLK_SRC_SEL_CLKSRCSEL_MAX (0x00000FFFU)

#define SDL_MSS_RCM_MSS_CPSW_MII_CLK_SRC_SEL_RESETVAL                          (0x00000000U)

/* MSS_CPSW_MII_CLK_STATUS */

#define SDL_MSS_RCM_MSS_CPSW_MII_CLK_STATUS_MSS_CPSW_MII_CLK_STATUS_CLKINUSE_MASK (0x000000FFU)
#define SDL_MSS_RCM_MSS_CPSW_MII_CLK_STATUS_MSS_CPSW_MII_CLK_STATUS_CLKINUSE_SHIFT (0x00000000U)
#define SDL_MSS_RCM_MSS_CPSW_MII_CLK_STATUS_MSS_CPSW_MII_CLK_STATUS_CLKINUSE_RESETVAL (0x00000001U)
#define SDL_MSS_RCM_MSS_CPSW_MII_CLK_STATUS_MSS_CPSW_MII_CLK_STATUS_CLKINUSE_MAX (0x000000FFU)

#define SDL_MSS_RCM_MSS_CPSW_MII_CLK_STATUS_RESETVAL                           (0x00000001U)

/* HSM_RTIA_CLK_SRC_SEL */

#define SDL_MSS_RCM_HSM_RTIA_CLK_SRC_SEL_HSM_RTIA_CLK_SRC_SEL_CLKSRCSEL_MASK   (0x00000FFFU)
#define SDL_MSS_RCM_HSM_RTIA_CLK_SRC_SEL_HSM_RTIA_CLK_SRC_SEL_CLKSRCSEL_SHIFT  (0x00000000U)
#define SDL_MSS_RCM_HSM_RTIA_CLK_SRC_SEL_HSM_RTIA_CLK_SRC_SEL_CLKSRCSEL_RESETVAL (0x00000000U)
#define SDL_MSS_RCM_HSM_RTIA_CLK_SRC_SEL_HSM_RTIA_CLK_SRC_SEL_CLKSRCSEL_MAX    (0x00000FFFU)

#define SDL_MSS_RCM_HSM_RTIA_CLK_SRC_SEL_RESETVAL                              (0x00000000U)

/* HSM_WDT_CLK_SRC_SEL */

#define SDL_MSS_RCM_HSM_WDT_CLK_SRC_SEL_HSM_WDT_CLK_SRC_SEL_CLKSRCSEL_MASK     (0x00000FFFU)
#define SDL_MSS_RCM_HSM_WDT_CLK_SRC_SEL_HSM_WDT_CLK_SRC_SEL_CLKSRCSEL_SHIFT    (0x00000000U)
#define SDL_MSS_RCM_HSM_WDT_CLK_SRC_SEL_HSM_WDT_CLK_SRC_SEL_CLKSRCSEL_RESETVAL (0x00000000U)
#define SDL_MSS_RCM_HSM_WDT_CLK_SRC_SEL_HSM_WDT_CLK_SRC_SEL_CLKSRCSEL_MAX      (0x00000FFFU)

#define SDL_MSS_RCM_HSM_WDT_CLK_SRC_SEL_RESETVAL                               (0x00000000U)

/* HSM_RTC_CLK_SRC_SEL */

#define SDL_MSS_RCM_HSM_RTC_CLK_SRC_SEL_HSM_RTC_CLK_SRC_SEL_CLKSRCSEL_MASK     (0x00000FFFU)
#define SDL_MSS_RCM_HSM_RTC_CLK_SRC_SEL_HSM_RTC_CLK_SRC_SEL_CLKSRCSEL_SHIFT    (0x00000000U)
#define SDL_MSS_RCM_HSM_RTC_CLK_SRC_SEL_HSM_RTC_CLK_SRC_SEL_CLKSRCSEL_RESETVAL (0x00000777U)
#define SDL_MSS_RCM_HSM_RTC_CLK_SRC_SEL_HSM_RTC_CLK_SRC_SEL_CLKSRCSEL_MAX      (0x00000FFFU)

#define SDL_MSS_RCM_HSM_RTC_CLK_SRC_SEL_RESETVAL                               (0x00000777U)

/* HSM_DMTA_CLK_SRC_SEL */

#define SDL_MSS_RCM_HSM_DMTA_CLK_SRC_SEL_HSM_DMTA_CLK_SRC_SEL_CLKSRCSEL_MASK   (0x00000FFFU)
#define SDL_MSS_RCM_HSM_DMTA_CLK_SRC_SEL_HSM_DMTA_CLK_SRC_SEL_CLKSRCSEL_SHIFT  (0x00000000U)
#define SDL_MSS_RCM_HSM_DMTA_CLK_SRC_SEL_HSM_DMTA_CLK_SRC_SEL_CLKSRCSEL_RESETVAL (0x00000000U)
#define SDL_MSS_RCM_HSM_DMTA_CLK_SRC_SEL_HSM_DMTA_CLK_SRC_SEL_CLKSRCSEL_MAX    (0x00000FFFU)

#define SDL_MSS_RCM_HSM_DMTA_CLK_SRC_SEL_RESETVAL                              (0x00000000U)

/* HSM_DMTB_CLK_SRC_SEL */

#define SDL_MSS_RCM_HSM_DMTB_CLK_SRC_SEL_HSM_DMTB_CLK_SRC_SEL_CLKSRCSEL_MASK   (0x00000FFFU)
#define SDL_MSS_RCM_HSM_DMTB_CLK_SRC_SEL_HSM_DMTB_CLK_SRC_SEL_CLKSRCSEL_SHIFT  (0x00000000U)
#define SDL_MSS_RCM_HSM_DMTB_CLK_SRC_SEL_HSM_DMTB_CLK_SRC_SEL_CLKSRCSEL_RESETVAL (0x00000000U)
#define SDL_MSS_RCM_HSM_DMTB_CLK_SRC_SEL_HSM_DMTB_CLK_SRC_SEL_CLKSRCSEL_MAX    (0x00000FFFU)

#define SDL_MSS_RCM_HSM_DMTB_CLK_SRC_SEL_RESETVAL                              (0x00000000U)

/* HSM_RTI_CLK_DIV_VAL */

#define SDL_MSS_RCM_HSM_RTI_CLK_DIV_VAL_HSM_RTI_CLK_DIV_VAL_CLKDIVR_MASK       (0x00000FFFU)
#define SDL_MSS_RCM_HSM_RTI_CLK_DIV_VAL_HSM_RTI_CLK_DIV_VAL_CLKDIVR_SHIFT      (0x00000000U)
#define SDL_MSS_RCM_HSM_RTI_CLK_DIV_VAL_HSM_RTI_CLK_DIV_VAL_CLKDIVR_RESETVAL   (0x00000000U)
#define SDL_MSS_RCM_HSM_RTI_CLK_DIV_VAL_HSM_RTI_CLK_DIV_VAL_CLKDIVR_MAX        (0x00000FFFU)

#define SDL_MSS_RCM_HSM_RTI_CLK_DIV_VAL_RESETVAL                               (0x00000000U)

/* HSM_WDT_CLK_DIV_VAL */

#define SDL_MSS_RCM_HSM_WDT_CLK_DIV_VAL_HSM_WDT_CLK_DIV_VAL_CLKDIVR_MASK       (0x00000FFFU)
#define SDL_MSS_RCM_HSM_WDT_CLK_DIV_VAL_HSM_WDT_CLK_DIV_VAL_CLKDIVR_SHIFT      (0x00000000U)
#define SDL_MSS_RCM_HSM_WDT_CLK_DIV_VAL_HSM_WDT_CLK_DIV_VAL_CLKDIVR_RESETVAL   (0x00000000U)
#define SDL_MSS_RCM_HSM_WDT_CLK_DIV_VAL_HSM_WDT_CLK_DIV_VAL_CLKDIVR_MAX        (0x00000FFFU)

#define SDL_MSS_RCM_HSM_WDT_CLK_DIV_VAL_RESETVAL                               (0x00000000U)

/* HSM_RTC_CLK_DIV_VAL */

#define SDL_MSS_RCM_HSM_RTC_CLK_DIV_VAL_HSM_RTC_CLK_DIV_VAL_CLKDIVR_MASK       (0x00000FFFU)
#define SDL_MSS_RCM_HSM_RTC_CLK_DIV_VAL_HSM_RTC_CLK_DIV_VAL_CLKDIVR_SHIFT      (0x00000000U)
#define SDL_MSS_RCM_HSM_RTC_CLK_DIV_VAL_HSM_RTC_CLK_DIV_VAL_CLKDIVR_RESETVAL   (0x00000000U)
#define SDL_MSS_RCM_HSM_RTC_CLK_DIV_VAL_HSM_RTC_CLK_DIV_VAL_CLKDIVR_MAX        (0x00000FFFU)

#define SDL_MSS_RCM_HSM_RTC_CLK_DIV_VAL_RESETVAL                               (0x00000000U)

/* HSM_DMTA_CLK_DIV_VAL */

#define SDL_MSS_RCM_HSM_DMTA_CLK_DIV_VAL_HSM_DMTA_CLK_DIV_VAL_CLKDIVR_MASK     (0x00000FFFU)
#define SDL_MSS_RCM_HSM_DMTA_CLK_DIV_VAL_HSM_DMTA_CLK_DIV_VAL_CLKDIVR_SHIFT    (0x00000000U)
#define SDL_MSS_RCM_HSM_DMTA_CLK_DIV_VAL_HSM_DMTA_CLK_DIV_VAL_CLKDIVR_RESETVAL (0x00000000U)
#define SDL_MSS_RCM_HSM_DMTA_CLK_DIV_VAL_HSM_DMTA_CLK_DIV_VAL_CLKDIVR_MAX      (0x00000FFFU)

#define SDL_MSS_RCM_HSM_DMTA_CLK_DIV_VAL_RESETVAL                              (0x00000000U)

/* HSM_DMTB_CLK_DIV_VAL */

#define SDL_MSS_RCM_HSM_DMTB_CLK_DIV_VAL_HSM_DMTB_CLK_DIV_VAL_CLKDIVR_MASK     (0x00000FFFU)
#define SDL_MSS_RCM_HSM_DMTB_CLK_DIV_VAL_HSM_DMTB_CLK_DIV_VAL_CLKDIVR_SHIFT    (0x00000000U)
#define SDL_MSS_RCM_HSM_DMTB_CLK_DIV_VAL_HSM_DMTB_CLK_DIV_VAL_CLKDIVR_RESETVAL (0x00000000U)
#define SDL_MSS_RCM_HSM_DMTB_CLK_DIV_VAL_HSM_DMTB_CLK_DIV_VAL_CLKDIVR_MAX      (0x00000FFFU)

#define SDL_MSS_RCM_HSM_DMTB_CLK_DIV_VAL_RESETVAL                              (0x00000000U)

/* HSM_RTI_CLK_GATE */

#define SDL_MSS_RCM_HSM_RTI_CLK_GATE_HSM_RTI_CLK_GATE_GATED_MASK               (0x00000007U)
#define SDL_MSS_RCM_HSM_RTI_CLK_GATE_HSM_RTI_CLK_GATE_GATED_SHIFT              (0x00000000U)
#define SDL_MSS_RCM_HSM_RTI_CLK_GATE_HSM_RTI_CLK_GATE_GATED_RESETVAL           (0x00000000U)
#define SDL_MSS_RCM_HSM_RTI_CLK_GATE_HSM_RTI_CLK_GATE_GATED_MAX                (0x00000007U)

#define SDL_MSS_RCM_HSM_RTI_CLK_GATE_RESETVAL                                  (0x00000000U)

/* HSM_WDT_CLK_GATE */

#define SDL_MSS_RCM_HSM_WDT_CLK_GATE_HSM_WDT_CLK_GATE_GATED_MASK               (0x00000007U)
#define SDL_MSS_RCM_HSM_WDT_CLK_GATE_HSM_WDT_CLK_GATE_GATED_SHIFT              (0x00000000U)
#define SDL_MSS_RCM_HSM_WDT_CLK_GATE_HSM_WDT_CLK_GATE_GATED_RESETVAL           (0x00000000U)
#define SDL_MSS_RCM_HSM_WDT_CLK_GATE_HSM_WDT_CLK_GATE_GATED_MAX                (0x00000007U)

#define SDL_MSS_RCM_HSM_WDT_CLK_GATE_RESETVAL                                  (0x00000000U)

/* HSM_RTC_CLK_GATE */

#define SDL_MSS_RCM_HSM_RTC_CLK_GATE_HSM_RTC_CLK_GATE_GATED_MASK               (0x00000007U)
#define SDL_MSS_RCM_HSM_RTC_CLK_GATE_HSM_RTC_CLK_GATE_GATED_SHIFT              (0x00000000U)
#define SDL_MSS_RCM_HSM_RTC_CLK_GATE_HSM_RTC_CLK_GATE_GATED_RESETVAL           (0x00000000U)
#define SDL_MSS_RCM_HSM_RTC_CLK_GATE_HSM_RTC_CLK_GATE_GATED_MAX                (0x00000007U)

#define SDL_MSS_RCM_HSM_RTC_CLK_GATE_RESETVAL                                  (0x00000000U)

/* HSM_DMTA_CLK_GATE */

#define SDL_MSS_RCM_HSM_DMTA_CLK_GATE_HSM_DMTA_CLK_GATE_GATED_MASK             (0x00000007U)
#define SDL_MSS_RCM_HSM_DMTA_CLK_GATE_HSM_DMTA_CLK_GATE_GATED_SHIFT            (0x00000000U)
#define SDL_MSS_RCM_HSM_DMTA_CLK_GATE_HSM_DMTA_CLK_GATE_GATED_RESETVAL         (0x00000000U)
#define SDL_MSS_RCM_HSM_DMTA_CLK_GATE_HSM_DMTA_CLK_GATE_GATED_MAX              (0x00000007U)

#define SDL_MSS_RCM_HSM_DMTA_CLK_GATE_RESETVAL                                 (0x00000000U)

/* HSM_DMTB_CLK_GATE */

#define SDL_MSS_RCM_HSM_DMTB_CLK_GATE_HSM_DMTB_CLK_GATE_GATED_MASK             (0x00000007U)
#define SDL_MSS_RCM_HSM_DMTB_CLK_GATE_HSM_DMTB_CLK_GATE_GATED_SHIFT            (0x00000000U)
#define SDL_MSS_RCM_HSM_DMTB_CLK_GATE_HSM_DMTB_CLK_GATE_GATED_RESETVAL         (0x00000000U)
#define SDL_MSS_RCM_HSM_DMTB_CLK_GATE_HSM_DMTB_CLK_GATE_GATED_MAX              (0x00000007U)

#define SDL_MSS_RCM_HSM_DMTB_CLK_GATE_RESETVAL                                 (0x00000000U)

/* HSM_RTI_CLK_STATUS */

#define SDL_MSS_RCM_HSM_RTI_CLK_STATUS_HSM_RTI_CLK_STATUS_CLKINUSE_MASK        (0x000000FFU)
#define SDL_MSS_RCM_HSM_RTI_CLK_STATUS_HSM_RTI_CLK_STATUS_CLKINUSE_SHIFT       (0x00000000U)
#define SDL_MSS_RCM_HSM_RTI_CLK_STATUS_HSM_RTI_CLK_STATUS_CLKINUSE_RESETVAL    (0x00000001U)
#define SDL_MSS_RCM_HSM_RTI_CLK_STATUS_HSM_RTI_CLK_STATUS_CLKINUSE_MAX         (0x000000FFU)

#define SDL_MSS_RCM_HSM_RTI_CLK_STATUS_HSM_RTI_CLK_STATUS_CURRDIVIDER_MASK     (0x0000FF00U)
#define SDL_MSS_RCM_HSM_RTI_CLK_STATUS_HSM_RTI_CLK_STATUS_CURRDIVIDER_SHIFT    (0x00000008U)
#define SDL_MSS_RCM_HSM_RTI_CLK_STATUS_HSM_RTI_CLK_STATUS_CURRDIVIDER_RESETVAL (0x00000000U)
#define SDL_MSS_RCM_HSM_RTI_CLK_STATUS_HSM_RTI_CLK_STATUS_CURRDIVIDER_MAX      (0x000000FFU)

#define SDL_MSS_RCM_HSM_RTI_CLK_STATUS_RESETVAL                                (0x00000001U)

/* HSM_WDT_CLK_STATUS */

#define SDL_MSS_RCM_HSM_WDT_CLK_STATUS_HSM_WDT_CLK_STATUS_CLKINUSE_MASK        (0x000000FFU)
#define SDL_MSS_RCM_HSM_WDT_CLK_STATUS_HSM_WDT_CLK_STATUS_CLKINUSE_SHIFT       (0x00000000U)
#define SDL_MSS_RCM_HSM_WDT_CLK_STATUS_HSM_WDT_CLK_STATUS_CLKINUSE_RESETVAL    (0x00000001U)
#define SDL_MSS_RCM_HSM_WDT_CLK_STATUS_HSM_WDT_CLK_STATUS_CLKINUSE_MAX         (0x000000FFU)

#define SDL_MSS_RCM_HSM_WDT_CLK_STATUS_HSM_WDT_CLK_STATUS_CURRDIVIDER_MASK     (0x0000FF00U)
#define SDL_MSS_RCM_HSM_WDT_CLK_STATUS_HSM_WDT_CLK_STATUS_CURRDIVIDER_SHIFT    (0x00000008U)
#define SDL_MSS_RCM_HSM_WDT_CLK_STATUS_HSM_WDT_CLK_STATUS_CURRDIVIDER_RESETVAL (0x00000000U)
#define SDL_MSS_RCM_HSM_WDT_CLK_STATUS_HSM_WDT_CLK_STATUS_CURRDIVIDER_MAX      (0x000000FFU)

#define SDL_MSS_RCM_HSM_WDT_CLK_STATUS_RESETVAL                                (0x00000001U)

/* HSM_RTC_CLK_STATUS */

#define SDL_MSS_RCM_HSM_RTC_CLK_STATUS_HSM_RTC_CLK_STATUS_CLKINUSE_MASK        (0x000000FFU)
#define SDL_MSS_RCM_HSM_RTC_CLK_STATUS_HSM_RTC_CLK_STATUS_CLKINUSE_SHIFT       (0x00000000U)
#define SDL_MSS_RCM_HSM_RTC_CLK_STATUS_HSM_RTC_CLK_STATUS_CLKINUSE_RESETVAL    (0x00000080U)
#define SDL_MSS_RCM_HSM_RTC_CLK_STATUS_HSM_RTC_CLK_STATUS_CLKINUSE_MAX         (0x000000FFU)

#define SDL_MSS_RCM_HSM_RTC_CLK_STATUS_HSM_RTC_CLK_STATUS_CURRDIVIDER_MASK     (0x0000FF00U)
#define SDL_MSS_RCM_HSM_RTC_CLK_STATUS_HSM_RTC_CLK_STATUS_CURRDIVIDER_SHIFT    (0x00000008U)
#define SDL_MSS_RCM_HSM_RTC_CLK_STATUS_HSM_RTC_CLK_STATUS_CURRDIVIDER_RESETVAL (0x00000000U)
#define SDL_MSS_RCM_HSM_RTC_CLK_STATUS_HSM_RTC_CLK_STATUS_CURRDIVIDER_MAX      (0x000000FFU)

#define SDL_MSS_RCM_HSM_RTC_CLK_STATUS_RESETVAL                                (0x00000080U)

/* HSM_DMTA_CLK_STATUS */

#define SDL_MSS_RCM_HSM_DMTA_CLK_STATUS_HSM_DMTA_CLK_STATUS_CLKINUSE_MASK      (0x000000FFU)
#define SDL_MSS_RCM_HSM_DMTA_CLK_STATUS_HSM_DMTA_CLK_STATUS_CLKINUSE_SHIFT     (0x00000000U)
#define SDL_MSS_RCM_HSM_DMTA_CLK_STATUS_HSM_DMTA_CLK_STATUS_CLKINUSE_RESETVAL  (0x00000001U)
#define SDL_MSS_RCM_HSM_DMTA_CLK_STATUS_HSM_DMTA_CLK_STATUS_CLKINUSE_MAX       (0x000000FFU)

#define SDL_MSS_RCM_HSM_DMTA_CLK_STATUS_HSM_DMTA_CLK_STATUS_CURRDIVIDER_MASK   (0x0000FF00U)
#define SDL_MSS_RCM_HSM_DMTA_CLK_STATUS_HSM_DMTA_CLK_STATUS_CURRDIVIDER_SHIFT  (0x00000008U)
#define SDL_MSS_RCM_HSM_DMTA_CLK_STATUS_HSM_DMTA_CLK_STATUS_CURRDIVIDER_RESETVAL (0x00000000U)
#define SDL_MSS_RCM_HSM_DMTA_CLK_STATUS_HSM_DMTA_CLK_STATUS_CURRDIVIDER_MAX    (0x000000FFU)

#define SDL_MSS_RCM_HSM_DMTA_CLK_STATUS_RESETVAL                               (0x00000001U)

/* HSM_DMTB_CLK_STATUS */

#define SDL_MSS_RCM_HSM_DMTB_CLK_STATUS_HSM_DMTB_CLK_STATUS_CLKINUSE_MASK      (0x000000FFU)
#define SDL_MSS_RCM_HSM_DMTB_CLK_STATUS_HSM_DMTB_CLK_STATUS_CLKINUSE_SHIFT     (0x00000000U)
#define SDL_MSS_RCM_HSM_DMTB_CLK_STATUS_HSM_DMTB_CLK_STATUS_CLKINUSE_RESETVAL  (0x00000001U)
#define SDL_MSS_RCM_HSM_DMTB_CLK_STATUS_HSM_DMTB_CLK_STATUS_CLKINUSE_MAX       (0x000000FFU)

#define SDL_MSS_RCM_HSM_DMTB_CLK_STATUS_HSM_DMTB_CLK_STATUS_CURRDIVIDER_MASK   (0x0000FF00U)
#define SDL_MSS_RCM_HSM_DMTB_CLK_STATUS_HSM_DMTB_CLK_STATUS_CURRDIVIDER_SHIFT  (0x00000008U)
#define SDL_MSS_RCM_HSM_DMTB_CLK_STATUS_HSM_DMTB_CLK_STATUS_CURRDIVIDER_RESETVAL (0x00000000U)
#define SDL_MSS_RCM_HSM_DMTB_CLK_STATUS_HSM_DMTB_CLK_STATUS_CURRDIVIDER_MAX    (0x000000FFU)

#define SDL_MSS_RCM_HSM_DMTB_CLK_STATUS_RESETVAL                               (0x00000001U)

/* HW_SPARE_RW0 */

#define SDL_MSS_RCM_HW_SPARE_RW0_HW_SPARE_RW0_HW_SPARE_RW0_MASK                (0xFFFFFFFFU)
#define SDL_MSS_RCM_HW_SPARE_RW0_HW_SPARE_RW0_HW_SPARE_RW0_SHIFT               (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_RW0_HW_SPARE_RW0_HW_SPARE_RW0_RESETVAL            (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_RW0_HW_SPARE_RW0_HW_SPARE_RW0_MAX                 (0xFFFFFFFFU)

#define SDL_MSS_RCM_HW_SPARE_RW0_RESETVAL                                      (0x00000000U)

/* HW_SPARE_RW1 */

#define SDL_MSS_RCM_HW_SPARE_RW1_HW_SPARE_RW1_HW_SPARE_RW1_MASK                (0xFFFFFFFFU)
#define SDL_MSS_RCM_HW_SPARE_RW1_HW_SPARE_RW1_HW_SPARE_RW1_SHIFT               (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_RW1_HW_SPARE_RW1_HW_SPARE_RW1_RESETVAL            (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_RW1_HW_SPARE_RW1_HW_SPARE_RW1_MAX                 (0xFFFFFFFFU)

#define SDL_MSS_RCM_HW_SPARE_RW1_RESETVAL                                      (0x00000000U)

/* HW_SPARE_RW2 */

#define SDL_MSS_RCM_HW_SPARE_RW2_HW_SPARE_RW2_HW_SPARE_RW2_MASK                (0xFFFFFFFFU)
#define SDL_MSS_RCM_HW_SPARE_RW2_HW_SPARE_RW2_HW_SPARE_RW2_SHIFT               (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_RW2_HW_SPARE_RW2_HW_SPARE_RW2_RESETVAL            (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_RW2_HW_SPARE_RW2_HW_SPARE_RW2_MAX                 (0xFFFFFFFFU)

#define SDL_MSS_RCM_HW_SPARE_RW2_RESETVAL                                      (0x00000000U)

/* HW_SPARE_RW3 */

#define SDL_MSS_RCM_HW_SPARE_RW3_HW_SPARE_RW3_HW_SPARE_RW3_MASK                (0xFFFFFFFFU)
#define SDL_MSS_RCM_HW_SPARE_RW3_HW_SPARE_RW3_HW_SPARE_RW3_SHIFT               (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_RW3_HW_SPARE_RW3_HW_SPARE_RW3_RESETVAL            (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_RW3_HW_SPARE_RW3_HW_SPARE_RW3_MAX                 (0xFFFFFFFFU)

#define SDL_MSS_RCM_HW_SPARE_RW3_RESETVAL                                      (0x00000000U)

/* HW_SPARE_RO0 */

#define SDL_MSS_RCM_HW_SPARE_RO0_HW_SPARE_RO0_HW_SPARE_RO0_MASK                (0xFFFFFFFFU)
#define SDL_MSS_RCM_HW_SPARE_RO0_HW_SPARE_RO0_HW_SPARE_RO0_SHIFT               (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_RO0_HW_SPARE_RO0_HW_SPARE_RO0_RESETVAL            (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_RO0_HW_SPARE_RO0_HW_SPARE_RO0_MAX                 (0xFFFFFFFFU)

#define SDL_MSS_RCM_HW_SPARE_RO0_RESETVAL                                      (0x00000000U)

/* HW_SPARE_RO1 */

#define SDL_MSS_RCM_HW_SPARE_RO1_HW_SPARE_RO1_HW_SPARE_RO1_MASK                (0xFFFFFFFFU)
#define SDL_MSS_RCM_HW_SPARE_RO1_HW_SPARE_RO1_HW_SPARE_RO1_SHIFT               (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_RO1_HW_SPARE_RO1_HW_SPARE_RO1_RESETVAL            (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_RO1_HW_SPARE_RO1_HW_SPARE_RO1_MAX                 (0xFFFFFFFFU)

#define SDL_MSS_RCM_HW_SPARE_RO1_RESETVAL                                      (0x00000000U)

/* HW_SPARE_RO2 */

#define SDL_MSS_RCM_HW_SPARE_RO2_HW_SPARE_RO2_HW_SPARE_RO2_MASK                (0xFFFFFFFFU)
#define SDL_MSS_RCM_HW_SPARE_RO2_HW_SPARE_RO2_HW_SPARE_RO2_SHIFT               (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_RO2_HW_SPARE_RO2_HW_SPARE_RO2_RESETVAL            (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_RO2_HW_SPARE_RO2_HW_SPARE_RO2_MAX                 (0xFFFFFFFFU)

#define SDL_MSS_RCM_HW_SPARE_RO2_RESETVAL                                      (0x00000000U)

/* HW_SPARE_RO3 */

#define SDL_MSS_RCM_HW_SPARE_RO3_HW_SPARE_RO3_HW_SPARE_RO3_MASK                (0xFFFFFFFFU)
#define SDL_MSS_RCM_HW_SPARE_RO3_HW_SPARE_RO3_HW_SPARE_RO3_SHIFT               (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_RO3_HW_SPARE_RO3_HW_SPARE_RO3_RESETVAL            (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_RO3_HW_SPARE_RO3_HW_SPARE_RO3_MAX                 (0xFFFFFFFFU)

#define SDL_MSS_RCM_HW_SPARE_RO3_RESETVAL                                      (0x00000000U)

/* HW_SPARE_WPH */

#define SDL_MSS_RCM_HW_SPARE_WPH_HW_SPARE_WPH_HW_SPARE_WPH_MASK                (0xFFFFFFFFU)
#define SDL_MSS_RCM_HW_SPARE_WPH_HW_SPARE_WPH_HW_SPARE_WPH_SHIFT               (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_WPH_HW_SPARE_WPH_HW_SPARE_WPH_RESETVAL            (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_WPH_HW_SPARE_WPH_HW_SPARE_WPH_MAX                 (0xFFFFFFFFU)

#define SDL_MSS_RCM_HW_SPARE_WPH_RESETVAL                                      (0x00000000U)

/* HW_SPARE_REC */

#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC0_MASK               (0x00000001U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC0_SHIFT              (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC0_RESETVAL           (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC0_MAX                (0x00000001U)

#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC1_MASK               (0x00000002U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC1_SHIFT              (0x00000001U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC1_RESETVAL           (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC1_MAX                (0x00000001U)

#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC2_MASK               (0x00000004U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC2_SHIFT              (0x00000002U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC2_RESETVAL           (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC2_MAX                (0x00000001U)

#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC3_MASK               (0x00000008U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC3_SHIFT              (0x00000003U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC3_RESETVAL           (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC3_MAX                (0x00000001U)

#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC4_MASK               (0x00000010U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC4_SHIFT              (0x00000004U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC4_RESETVAL           (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC4_MAX                (0x00000001U)

#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC5_MASK               (0x00000020U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC5_SHIFT              (0x00000005U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC5_RESETVAL           (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC5_MAX                (0x00000001U)

#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC6_MASK               (0x00000040U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC6_SHIFT              (0x00000006U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC6_RESETVAL           (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC6_MAX                (0x00000001U)

#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC7_MASK               (0x00000080U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC7_SHIFT              (0x00000007U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC7_RESETVAL           (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC7_MAX                (0x00000001U)

#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC8_MASK               (0x00000100U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC8_SHIFT              (0x00000008U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC8_RESETVAL           (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC8_MAX                (0x00000001U)

#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC9_MASK               (0x00000200U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC9_SHIFT              (0x00000009U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC9_RESETVAL           (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC9_MAX                (0x00000001U)

#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC10_MASK              (0x00000400U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC10_SHIFT             (0x0000000AU)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC10_RESETVAL          (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC10_MAX               (0x00000001U)

#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC11_MASK              (0x00000800U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC11_SHIFT             (0x0000000BU)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC11_RESETVAL          (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC11_MAX               (0x00000001U)

#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC12_MASK              (0x00001000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC12_SHIFT             (0x0000000CU)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC12_RESETVAL          (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC12_MAX               (0x00000001U)

#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC13_MASK              (0x00002000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC13_SHIFT             (0x0000000DU)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC13_RESETVAL          (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC13_MAX               (0x00000001U)

#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC14_MASK              (0x00004000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC14_SHIFT             (0x0000000EU)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC14_RESETVAL          (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC14_MAX               (0x00000001U)

#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC15_MASK              (0x00008000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC15_SHIFT             (0x0000000FU)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC15_RESETVAL          (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC15_MAX               (0x00000001U)

#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC16_MASK              (0x00010000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC16_SHIFT             (0x00000010U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC16_RESETVAL          (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC16_MAX               (0x00000001U)

#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC17_MASK              (0x00020000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC17_SHIFT             (0x00000011U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC17_RESETVAL          (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC17_MAX               (0x00000001U)

#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC18_MASK              (0x00040000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC18_SHIFT             (0x00000012U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC18_RESETVAL          (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC18_MAX               (0x00000001U)

#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC19_MASK              (0x00080000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC19_SHIFT             (0x00000013U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC19_RESETVAL          (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC19_MAX               (0x00000001U)

#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC20_MASK              (0x00100000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC20_SHIFT             (0x00000014U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC20_RESETVAL          (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC20_MAX               (0x00000001U)

#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC21_MASK              (0x00200000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC21_SHIFT             (0x00000015U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC21_RESETVAL          (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC21_MAX               (0x00000001U)

#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC22_MASK              (0x00400000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC22_SHIFT             (0x00000016U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC22_RESETVAL          (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC22_MAX               (0x00000001U)

#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC23_MASK              (0x00800000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC23_SHIFT             (0x00000017U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC23_RESETVAL          (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC23_MAX               (0x00000001U)

#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC24_MASK              (0x01000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC24_SHIFT             (0x00000018U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC24_RESETVAL          (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC24_MAX               (0x00000001U)

#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC25_MASK              (0x02000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC25_SHIFT             (0x00000019U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC25_RESETVAL          (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC25_MAX               (0x00000001U)

#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC26_MASK              (0x04000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC26_SHIFT             (0x0000001AU)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC26_RESETVAL          (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC26_MAX               (0x00000001U)

#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC27_MASK              (0x08000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC27_SHIFT             (0x0000001BU)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC27_RESETVAL          (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC27_MAX               (0x00000001U)

#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC28_MASK              (0x10000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC28_SHIFT             (0x0000001CU)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC28_RESETVAL          (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC28_MAX               (0x00000001U)

#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC29_MASK              (0x20000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC29_SHIFT             (0x0000001DU)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC29_RESETVAL          (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC29_MAX               (0x00000001U)

#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC30_MASK              (0x40000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC30_SHIFT             (0x0000001EU)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC30_RESETVAL          (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC30_MAX               (0x00000001U)

#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC31_MASK              (0x80000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC31_SHIFT             (0x0000001FU)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC31_RESETVAL          (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC31_MAX               (0x00000001U)

#define SDL_MSS_RCM_HW_SPARE_REC_RESETVAL                                      (0x00000000U)

/* LOCK0_KICK0 */

#define SDL_MSS_RCM_LOCK0_KICK0_LOCK0_KICK0_MASK                               (0xFFFFFFFFU)
#define SDL_MSS_RCM_LOCK0_KICK0_LOCK0_KICK0_SHIFT                              (0x00000000U)
#define SDL_MSS_RCM_LOCK0_KICK0_LOCK0_KICK0_RESETVAL                           (0x00000000U)
#define SDL_MSS_RCM_LOCK0_KICK0_LOCK0_KICK0_MAX                                (0xFFFFFFFFU)

#define SDL_MSS_RCM_LOCK0_KICK0_RESETVAL                                       (0x00000000U)

/* LOCK0_KICK1 */

#define SDL_MSS_RCM_LOCK0_KICK1_LOCK0_KICK1_MASK                               (0xFFFFFFFFU)
#define SDL_MSS_RCM_LOCK0_KICK1_LOCK0_KICK1_SHIFT                              (0x00000000U)
#define SDL_MSS_RCM_LOCK0_KICK1_LOCK0_KICK1_RESETVAL                           (0x00000000U)
#define SDL_MSS_RCM_LOCK0_KICK1_LOCK0_KICK1_MAX                                (0xFFFFFFFFU)

#define SDL_MSS_RCM_LOCK0_KICK1_RESETVAL                                       (0x00000000U)

/* INTR_RAW_STATUS */

#define SDL_MSS_RCM_INTR_RAW_STATUS_PROT_ERR_MASK                              (0x00000001U)
#define SDL_MSS_RCM_INTR_RAW_STATUS_PROT_ERR_SHIFT                             (0x00000000U)
#define SDL_MSS_RCM_INTR_RAW_STATUS_PROT_ERR_RESETVAL                          (0x00000000U)
#define SDL_MSS_RCM_INTR_RAW_STATUS_PROT_ERR_MAX                               (0x00000001U)

#define SDL_MSS_RCM_INTR_RAW_STATUS_ADDR_ERR_MASK                              (0x00000002U)
#define SDL_MSS_RCM_INTR_RAW_STATUS_ADDR_ERR_SHIFT                             (0x00000001U)
#define SDL_MSS_RCM_INTR_RAW_STATUS_ADDR_ERR_RESETVAL                          (0x00000000U)
#define SDL_MSS_RCM_INTR_RAW_STATUS_ADDR_ERR_MAX                               (0x00000001U)

#define SDL_MSS_RCM_INTR_RAW_STATUS_KICK_ERR_MASK                              (0x00000004U)
#define SDL_MSS_RCM_INTR_RAW_STATUS_KICK_ERR_SHIFT                             (0x00000002U)
#define SDL_MSS_RCM_INTR_RAW_STATUS_KICK_ERR_RESETVAL                          (0x00000000U)
#define SDL_MSS_RCM_INTR_RAW_STATUS_KICK_ERR_MAX                               (0x00000001U)

#define SDL_MSS_RCM_INTR_RAW_STATUS_PROXY_ERR_MASK                             (0x00000008U)
#define SDL_MSS_RCM_INTR_RAW_STATUS_PROXY_ERR_SHIFT                            (0x00000003U)
#define SDL_MSS_RCM_INTR_RAW_STATUS_PROXY_ERR_RESETVAL                         (0x00000000U)
#define SDL_MSS_RCM_INTR_RAW_STATUS_PROXY_ERR_MAX                              (0x00000001U)

#define SDL_MSS_RCM_INTR_RAW_STATUS_RESETVAL                                   (0x00000000U)

/* INTR_ENABLED_STATUS_CLEAR */

#define SDL_MSS_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR_MASK            (0x00000001U)
#define SDL_MSS_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR_SHIFT           (0x00000000U)
#define SDL_MSS_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR_RESETVAL        (0x00000000U)
#define SDL_MSS_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR_MAX             (0x00000001U)

#define SDL_MSS_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR_MASK            (0x00000002U)
#define SDL_MSS_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR_SHIFT           (0x00000001U)
#define SDL_MSS_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR_RESETVAL        (0x00000000U)
#define SDL_MSS_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR_MAX             (0x00000001U)

#define SDL_MSS_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_KICK_ERR_MASK            (0x00000004U)
#define SDL_MSS_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_KICK_ERR_SHIFT           (0x00000002U)
#define SDL_MSS_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_KICK_ERR_RESETVAL        (0x00000000U)
#define SDL_MSS_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_KICK_ERR_MAX             (0x00000001U)

#define SDL_MSS_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROXY_ERR_MASK           (0x00000008U)
#define SDL_MSS_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROXY_ERR_SHIFT          (0x00000003U)
#define SDL_MSS_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROXY_ERR_RESETVAL       (0x00000000U)
#define SDL_MSS_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROXY_ERR_MAX            (0x00000001U)

#define SDL_MSS_RCM_INTR_ENABLED_STATUS_CLEAR_RESETVAL                         (0x00000000U)

/* INTR_ENABLE */

#define SDL_MSS_RCM_INTR_ENABLE_PROT_ERR_EN_MASK                               (0x00000001U)
#define SDL_MSS_RCM_INTR_ENABLE_PROT_ERR_EN_SHIFT                              (0x00000000U)
#define SDL_MSS_RCM_INTR_ENABLE_PROT_ERR_EN_RESETVAL                           (0x00000000U)
#define SDL_MSS_RCM_INTR_ENABLE_PROT_ERR_EN_MAX                                (0x00000001U)

#define SDL_MSS_RCM_INTR_ENABLE_ADDR_ERR_EN_MASK                               (0x00000002U)
#define SDL_MSS_RCM_INTR_ENABLE_ADDR_ERR_EN_SHIFT                              (0x00000001U)
#define SDL_MSS_RCM_INTR_ENABLE_ADDR_ERR_EN_RESETVAL                           (0x00000000U)
#define SDL_MSS_RCM_INTR_ENABLE_ADDR_ERR_EN_MAX                                (0x00000001U)

#define SDL_MSS_RCM_INTR_ENABLE_KICK_ERR_EN_MASK                               (0x00000004U)
#define SDL_MSS_RCM_INTR_ENABLE_KICK_ERR_EN_SHIFT                              (0x00000002U)
#define SDL_MSS_RCM_INTR_ENABLE_KICK_ERR_EN_RESETVAL                           (0x00000000U)
#define SDL_MSS_RCM_INTR_ENABLE_KICK_ERR_EN_MAX                                (0x00000001U)

#define SDL_MSS_RCM_INTR_ENABLE_PROXY_ERR_EN_MASK                              (0x00000008U)
#define SDL_MSS_RCM_INTR_ENABLE_PROXY_ERR_EN_SHIFT                             (0x00000003U)
#define SDL_MSS_RCM_INTR_ENABLE_PROXY_ERR_EN_RESETVAL                          (0x00000000U)
#define SDL_MSS_RCM_INTR_ENABLE_PROXY_ERR_EN_MAX                               (0x00000001U)

#define SDL_MSS_RCM_INTR_ENABLE_RESETVAL                                       (0x00000000U)

/* INTR_ENABLE_CLEAR */

#define SDL_MSS_RCM_INTR_ENABLE_CLEAR_PROT_ERR_EN_CLR_MASK                     (0x00000001U)
#define SDL_MSS_RCM_INTR_ENABLE_CLEAR_PROT_ERR_EN_CLR_SHIFT                    (0x00000000U)
#define SDL_MSS_RCM_INTR_ENABLE_CLEAR_PROT_ERR_EN_CLR_RESETVAL                 (0x00000000U)
#define SDL_MSS_RCM_INTR_ENABLE_CLEAR_PROT_ERR_EN_CLR_MAX                      (0x00000001U)

#define SDL_MSS_RCM_INTR_ENABLE_CLEAR_ADDR_ERR_EN_CLR_MASK                     (0x00000002U)
#define SDL_MSS_RCM_INTR_ENABLE_CLEAR_ADDR_ERR_EN_CLR_SHIFT                    (0x00000001U)
#define SDL_MSS_RCM_INTR_ENABLE_CLEAR_ADDR_ERR_EN_CLR_RESETVAL                 (0x00000000U)
#define SDL_MSS_RCM_INTR_ENABLE_CLEAR_ADDR_ERR_EN_CLR_MAX                      (0x00000001U)

#define SDL_MSS_RCM_INTR_ENABLE_CLEAR_KICK_ERR_EN_CLR_MASK                     (0x00000004U)
#define SDL_MSS_RCM_INTR_ENABLE_CLEAR_KICK_ERR_EN_CLR_SHIFT                    (0x00000002U)
#define SDL_MSS_RCM_INTR_ENABLE_CLEAR_KICK_ERR_EN_CLR_RESETVAL                 (0x00000000U)
#define SDL_MSS_RCM_INTR_ENABLE_CLEAR_KICK_ERR_EN_CLR_MAX                      (0x00000001U)

#define SDL_MSS_RCM_INTR_ENABLE_CLEAR_PROXY_ERR_EN_CLR_MASK                    (0x00000008U)
#define SDL_MSS_RCM_INTR_ENABLE_CLEAR_PROXY_ERR_EN_CLR_SHIFT                   (0x00000003U)
#define SDL_MSS_RCM_INTR_ENABLE_CLEAR_PROXY_ERR_EN_CLR_RESETVAL                (0x00000000U)
#define SDL_MSS_RCM_INTR_ENABLE_CLEAR_PROXY_ERR_EN_CLR_MAX                     (0x00000001U)

#define SDL_MSS_RCM_INTR_ENABLE_CLEAR_RESETVAL                                 (0x00000000U)

/* EOI */

#define SDL_MSS_RCM_EOI_EOI_VECTOR_MASK                                        (0x000000FFU)
#define SDL_MSS_RCM_EOI_EOI_VECTOR_SHIFT                                       (0x00000000U)
#define SDL_MSS_RCM_EOI_EOI_VECTOR_RESETVAL                                    (0x00000000U)
#define SDL_MSS_RCM_EOI_EOI_VECTOR_MAX                                         (0x000000FFU)

#define SDL_MSS_RCM_EOI_RESETVAL                                               (0x00000000U)

/* FAULT_ADDRESS */

#define SDL_MSS_RCM_FAULT_ADDRESS_FAULT_ADDR_MASK                              (0xFFFFFFFFU)
#define SDL_MSS_RCM_FAULT_ADDRESS_FAULT_ADDR_SHIFT                             (0x00000000U)
#define SDL_MSS_RCM_FAULT_ADDRESS_FAULT_ADDR_RESETVAL                          (0x00000000U)
#define SDL_MSS_RCM_FAULT_ADDRESS_FAULT_ADDR_MAX                               (0xFFFFFFFFU)

#define SDL_MSS_RCM_FAULT_ADDRESS_RESETVAL                                     (0x00000000U)

/* FAULT_TYPE_STATUS */

#define SDL_MSS_RCM_FAULT_TYPE_STATUS_FAULT_TYPE_MASK                          (0x0000003FU)
#define SDL_MSS_RCM_FAULT_TYPE_STATUS_FAULT_TYPE_SHIFT                         (0x00000000U)
#define SDL_MSS_RCM_FAULT_TYPE_STATUS_FAULT_TYPE_RESETVAL                      (0x00000000U)
#define SDL_MSS_RCM_FAULT_TYPE_STATUS_FAULT_TYPE_MAX                           (0x0000003FU)

#define SDL_MSS_RCM_FAULT_TYPE_STATUS_FAULT_NS_MASK                            (0x00000040U)
#define SDL_MSS_RCM_FAULT_TYPE_STATUS_FAULT_NS_SHIFT                           (0x00000006U)
#define SDL_MSS_RCM_FAULT_TYPE_STATUS_FAULT_NS_RESETVAL                        (0x00000000U)
#define SDL_MSS_RCM_FAULT_TYPE_STATUS_FAULT_NS_MAX                             (0x00000001U)

#define SDL_MSS_RCM_FAULT_TYPE_STATUS_RESETVAL                                 (0x00000000U)

/* FAULT_ATTR_STATUS */

#define SDL_MSS_RCM_FAULT_ATTR_STATUS_FAULT_PRIVID_MASK                        (0x000000FFU)
#define SDL_MSS_RCM_FAULT_ATTR_STATUS_FAULT_PRIVID_SHIFT                       (0x00000000U)
#define SDL_MSS_RCM_FAULT_ATTR_STATUS_FAULT_PRIVID_RESETVAL                    (0x00000000U)
#define SDL_MSS_RCM_FAULT_ATTR_STATUS_FAULT_PRIVID_MAX                         (0x000000FFU)

#define SDL_MSS_RCM_FAULT_ATTR_STATUS_FAULT_ROUTEID_MASK                       (0x000FFF00U)
#define SDL_MSS_RCM_FAULT_ATTR_STATUS_FAULT_ROUTEID_SHIFT                      (0x00000008U)
#define SDL_MSS_RCM_FAULT_ATTR_STATUS_FAULT_ROUTEID_RESETVAL                   (0x00000000U)
#define SDL_MSS_RCM_FAULT_ATTR_STATUS_FAULT_ROUTEID_MAX                        (0x00000FFFU)

#define SDL_MSS_RCM_FAULT_ATTR_STATUS_FAULT_XID_MASK                           (0xFFF00000U)
#define SDL_MSS_RCM_FAULT_ATTR_STATUS_FAULT_XID_SHIFT                          (0x00000014U)
#define SDL_MSS_RCM_FAULT_ATTR_STATUS_FAULT_XID_RESETVAL                       (0x00000000U)
#define SDL_MSS_RCM_FAULT_ATTR_STATUS_FAULT_XID_MAX                            (0x00000FFFU)

#define SDL_MSS_RCM_FAULT_ATTR_STATUS_RESETVAL                                 (0x00000000U)

/* FAULT_CLEAR */

#define SDL_MSS_RCM_FAULT_CLEAR_FAULT_CLR_MASK                                 (0x00000001U)
#define SDL_MSS_RCM_FAULT_CLEAR_FAULT_CLR_SHIFT                                (0x00000000U)
#define SDL_MSS_RCM_FAULT_CLEAR_FAULT_CLR_RESETVAL                             (0x00000000U)
#define SDL_MSS_RCM_FAULT_CLEAR_FAULT_CLR_MAX                                  (0x00000001U)

#define SDL_MSS_RCM_FAULT_CLEAR_RESETVAL                                       (0x00000000U)

#ifdef __cplusplus
}
#endif
#endif
