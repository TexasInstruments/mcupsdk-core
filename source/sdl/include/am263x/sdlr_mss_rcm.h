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

#ifdef __cplusplus
extern "C"
{
#endif
#include <sdl/sdlr.h>
#include <stdint.h>

/**************************************************************************
* Module Base Offset Values
**************************************************************************/

#define SDL_MSS_RCM_REGS_BASE                                             (0x00000000U)


/**************************************************************************
* Hardware Region  : MMRs in region 0
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t PID;                       /* PID register */
    volatile uint8_t  Resv_16[12];
    volatile uint32_t R5SS0_RST_STATUS;
    volatile uint32_t R5SS0_RST_CAUSE_CLR;
    volatile uint32_t R5SS0_DBG_RST_EN;
    volatile uint32_t R5SS0_RST_ASSERDLY;
    volatile uint32_t R5SS0_RST2ASSERTDLY;
    volatile uint32_t R5SS0_RST_WFICHECK;
    volatile uint8_t  Resv_48[8];
    volatile uint32_t R5SS1_RST_STATUS;
    volatile uint32_t R5SS1_RST_CAUSE_CLR;
    volatile uint32_t R5SS1_DBG_RST_EN;
    volatile uint32_t R5SS1_RST_ASSERDLY;
    volatile uint32_t R5SS1_RST2ASSERTDLY;
    volatile uint32_t R5SS1_RST_WFICHECK;
    volatile uint8_t  Resv_256[184];
    volatile uint32_t MCAN0_CLK_SRC_SEL;
    volatile uint32_t MCAN1_CLK_SRC_SEL;
    volatile uint32_t MCAN2_CLK_SRC_SEL;
    volatile uint32_t MCAN3_CLK_SRC_SEL;
    volatile uint32_t QSPI0_CLK_SRC_SEL;
    volatile uint32_t RTI0_CLK_SRC_SEL;
    volatile uint32_t RTI1_CLK_SRC_SEL;
    volatile uint32_t RTI2_CLK_SRC_SEL;
    volatile uint32_t RTI3_CLK_SRC_SEL;
    volatile uint8_t  Resv_296[4];
    volatile uint32_t WDT0_CLK_SRC_SEL;
    volatile uint32_t WDT1_CLK_SRC_SEL;
    volatile uint32_t WDT2_CLK_SRC_SEL;
    volatile uint32_t WDT3_CLK_SRC_SEL;
    volatile uint8_t  Resv_316[4];
    volatile uint32_t MCSPI0_CLK_SRC_SEL;
    volatile uint32_t MCSPI1_CLK_SRC_SEL;
    volatile uint32_t MCSPI2_CLK_SRC_SEL;
    volatile uint32_t MCSPI3_CLK_SRC_SEL;
    volatile uint32_t MCSPI4_CLK_SRC_SEL;
    volatile uint32_t MMC0_CLK_SRC_SEL;
    volatile uint32_t ICSSM0_UART0_CLK_SRC_SEL;
    volatile uint32_t CPTS_CLK_SRC_SEL;
    volatile uint32_t GPMC_CLK_SRC_SEL;
    volatile uint32_t CONTROLSS_PLL_CLK_SRC_SEL;
    volatile uint32_t I2C_CLK_SRC_SEL;
    volatile uint8_t  Resv_372[12];
    volatile uint32_t LIN0_UART0_CLK_SRC_SEL;
    volatile uint32_t LIN1_UART1_CLK_SRC_SEL;
    volatile uint32_t LIN2_UART2_CLK_SRC_SEL;
    volatile uint32_t LIN3_UART3_CLK_SRC_SEL;
    volatile uint32_t LIN4_UART4_CLK_SRC_SEL;
    volatile uint32_t LIN5_UART5_CLK_SRC_SEL;
    volatile uint8_t  Resv_512[116];
    volatile uint32_t MCAN0_CLK_DIV_VAL;
    volatile uint32_t MCAN1_CLK_DIV_VAL;
    volatile uint32_t MCAN2_CLK_DIV_VAL;
    volatile uint32_t MCAN3_CLK_DIV_VAL;
    volatile uint32_t QSPI0_CLK_DIV_VAL;
    volatile uint32_t RTI0_CLK_DIV_VAL;
    volatile uint32_t RTI1_CLK_DIV_VAL;
    volatile uint32_t RTI2_CLK_DIV_VAL;
    volatile uint32_t RTI3_CLK_DIV_VAL;
    volatile uint8_t  Resv_552[4];
    volatile uint32_t WDT0_CLK_DIV_VAL;
    volatile uint32_t WDT1_CLK_DIV_VAL;
    volatile uint32_t WDT2_CLK_DIV_VAL;
    volatile uint32_t WDT3_CLK_DIV_VAL;
    volatile uint8_t  Resv_572[4];
    volatile uint32_t MCSPI0_CLK_DIV_VAL;
    volatile uint32_t MCSPI1_CLK_DIV_VAL;
    volatile uint32_t MCSPI2_CLK_DIV_VAL;
    volatile uint32_t MCSPI3_CLK_DIV_VAL;
    volatile uint32_t MCSPI4_CLK_DIV_VAL;
    volatile uint32_t MMC0_CLK_DIV_VAL;
    volatile uint32_t ICSSM0_UART_CLK_DIV_VAL;
    volatile uint32_t CPTS_CLK_DIV_VAL;
    volatile uint32_t GPMC_CLK_DIV_VAL;
    volatile uint32_t CONTROLSS_PLL_CLK_DIV_VAL;
    volatile uint32_t I2C_CLK_DIV_VAL;
    volatile uint8_t  Resv_628[12];
    volatile uint32_t LIN0_UART0_CLK_DIV_VAL;
    volatile uint32_t LIN1_UART1_CLK_DIV_VAL;
    volatile uint32_t LIN2_UART2_CLK_DIV_VAL;
    volatile uint32_t LIN3_UART3_CLK_DIV_VAL;
    volatile uint32_t LIN4_UART4_CLK_DIV_VAL;
    volatile uint32_t LIN5_UART5_CLK_DIV_VAL;
    volatile uint32_t RGMII_250_CLK_DIV_VAL;
    volatile uint32_t RGMII_50_CLK_DIV_VAL;
    volatile uint32_t RGMII_5_CLK_DIV_VAL;
    volatile uint32_t XTAL_MMC_32K_CLK_DIV_VAL;
    volatile uint32_t XTAL_TEMPSENSE_32K_CLK_DIV_VAL;
    volatile uint32_t MSS_ELM_CLK_DIV_VAL;
    volatile uint8_t  Resv_768[92];
    volatile uint32_t MCAN0_CLK_GATE;
    volatile uint32_t MCAN1_CLK_GATE;
    volatile uint32_t MCAN2_CLK_GATE;
    volatile uint32_t MCAN3_CLK_GATE;
    volatile uint32_t QSPI0_CLK_GATE;
    volatile uint32_t RTI0_CLK_GATE;
    volatile uint32_t RTI1_CLK_GATE;
    volatile uint32_t RTI2_CLK_GATE;
    volatile uint32_t RTI3_CLK_GATE;
    volatile uint8_t  Resv_808[4];
    volatile uint32_t WDT0_CLK_GATE;
    volatile uint32_t WDT1_CLK_GATE;
    volatile uint32_t WDT2_CLK_GATE;
    volatile uint32_t WDT3_CLK_GATE;
    volatile uint8_t  Resv_828[4];
    volatile uint32_t MCSPI0_CLK_GATE;
    volatile uint32_t MCSPI1_CLK_GATE;
    volatile uint32_t MCSPI2_CLK_GATE;
    volatile uint32_t MCSPI3_CLK_GATE;
    volatile uint32_t MCSPI4_CLK_GATE;
    volatile uint32_t MMC0_CLK_GATE;
    volatile uint32_t ICSSM0_UART_CLK_GATE;
    volatile uint32_t CPTS_CLK_GATE;
    volatile uint32_t GPMC_CLK_GATE;
    volatile uint32_t CONTROLSS_PLL_CLK_GATE;
    volatile uint32_t I2C0_CLK_GATE;
    volatile uint32_t I2C1_CLK_GATE;
    volatile uint32_t I2C2_CLK_GATE;
    volatile uint32_t I2C3_CLK_GATE;
    volatile uint32_t LIN0_CLK_GATE;
    volatile uint32_t LIN1_CLK_GATE;
    volatile uint32_t LIN2_CLK_GATE;
    volatile uint32_t LIN3_CLK_GATE;
    volatile uint32_t LIN4_CLK_GATE;
    volatile uint8_t  Resv_908[4];
    volatile uint32_t UART0_CLK_GATE;
    volatile uint32_t UART1_CLK_GATE;
    volatile uint32_t UART2_CLK_GATE;
    volatile uint32_t UART3_CLK_GATE;
    volatile uint32_t UART4_CLK_GATE;
    volatile uint32_t UART5_CLK_GATE;
    volatile uint32_t RGMII_250_CLK_GATE;
    volatile uint32_t RGMII_50_CLK_GATE;
    volatile uint32_t RGMII_5_CLK_GATE;
    volatile uint32_t MMC0_32K_CLK_GATE;
    volatile uint32_t TEMPSENSE_32K_CLK_GATE;
    volatile uint32_t CPSW_CLK_GATE;
    volatile uint32_t ICSSM0_IEP_CLK_GATE;
    volatile uint32_t ICSSM0_CORE_CLK_GATE;
    volatile uint32_t MSS_ICSSM_SYS_CLK_GATE;
    volatile uint32_t MSS_ELM_CLK_GATE;
    volatile uint32_t R5SS0_CORE0_GATE;
    volatile uint32_t R5SS1_CORE0_GATE;
    volatile uint32_t R5SS0_CORE1_GATE;
    volatile uint32_t R5SS1_CORE1_GATE;
    volatile uint8_t  Resv_1024[36];
    volatile uint32_t MCAN0_CLK_STATUS;
    volatile uint32_t MCAN1_CLK_STATUS;
    volatile uint32_t MCAN2_CLK_STATUS;
    volatile uint32_t MCAN3_CLK_STATUS;
    volatile uint32_t QSPI0_CLK_STATUS;
    volatile uint32_t RTI0_CLK_STATUS;
    volatile uint32_t RTI1_CLK_STATUS;
    volatile uint32_t RTI2_CLK_STATUS;
    volatile uint32_t RTI3_CLK_STATUS;
    volatile uint8_t  Resv_1064[4];
    volatile uint32_t WDT0_CLK_STATUS;
    volatile uint32_t WDT1_CLK_STATUS;
    volatile uint32_t WDT2_CLK_STATUS;
    volatile uint32_t WDT3_CLK_STATUS;
    volatile uint8_t  Resv_1084[4];
    volatile uint32_t MCSPI0_CLK_STATUS;
    volatile uint32_t MCSPI1_CLK_STATUS;
    volatile uint32_t MCSPI2_CLK_STATUS;
    volatile uint32_t MCSPI3_CLK_STATUS;
    volatile uint32_t MCSPI4_CLK_STATUS;
    volatile uint32_t MMC0_CLK_STATUS;
    volatile uint32_t ICSSM0_UART_CLK_STATUS;
    volatile uint32_t CPTS_CLK_STATUS;
    volatile uint32_t GPMC_CLK_STATUS;
    volatile uint32_t CONTROLSS_PLL_CLK_STATUS;
    volatile uint32_t I2C_CLK_STATUS;
    volatile uint8_t  Resv_1140[12];
    volatile uint32_t LIN0_UART0_CLK_STATUS;
    volatile uint32_t LIN1_UART1_CLK_STATUS;
    volatile uint32_t LIN2_UART2_CLK_STATUS;
    volatile uint32_t LIN3_UART3_CLK_STATUS;
    volatile uint32_t LIN4_UART4_CLK_STATUS;
    volatile uint32_t LIN5_UART5_CLK_STATUS;
    volatile uint32_t RGMII_250_CLK_STATUS;
    volatile uint32_t RGMII_50_CLK_STATUS;
    volatile uint32_t RGMII_5_CLK_STATUS;
    volatile uint8_t  Resv_1180[4];
    volatile uint32_t MMC0_32K_CLK_STATUS;
    volatile uint32_t TEMPSENSE_32K_CLK_STATUS;
    volatile uint32_t MSS_ELM_CLK_STATUS;
    volatile uint8_t  Resv_1280[88];
    volatile uint32_t R5SS0_POR_RST_CTRL;
    volatile uint32_t R5SS1_POR_RST_CTRL;
    volatile uint32_t R5SS0_CORE0_GRST_CTRL;
    volatile uint32_t R5SS1_CORE0_GRST_CTRL;
    volatile uint32_t R5SS0_CORE1_GRST_CTRL;
    volatile uint32_t R5SS1_CORE1_GRST_CTRL;
    volatile uint32_t R5SS0_CORE0_LRST_CTRL;
    volatile uint32_t R5SS1_CORE0_LRST_CTRL;
    volatile uint32_t R5SS0_CORE1_LRST_CTRL;
    volatile uint32_t R5SS1_CORE1_LRST_CTRL;
    volatile uint32_t R5SS0_VIM0_RST_CTRL;
    volatile uint32_t R5SS1_VIM0_RST_CTRL;
    volatile uint32_t R5SS0_VIM1_RST_CTRL;
    volatile uint32_t R5SS1_VIM1_RST_CTRL;
    volatile uint32_t MCRC0_RST_CTRL;
    volatile uint32_t RTI0_RST_CTRL;
    volatile uint32_t RTI1_RST_CTRL;
    volatile uint32_t RTI2_RST_CTRL;
    volatile uint32_t RTI3_RST_CTRL;
    volatile uint32_t WDT0_RST_CTRL;
    volatile uint32_t WDT1_RST_CTRL;
    volatile uint32_t WDT2_RST_CTRL;
    volatile uint32_t WDT3_RST_CTRL;
    volatile uint32_t TOP_ESM_RST_CTRL;
    volatile uint32_t DCC0_RST_CTRL;
    volatile uint32_t DCC1_RST_CTRL;
    volatile uint32_t DCC2_RST_CTRL;
    volatile uint32_t DCC3_RST_CTRL;
    volatile uint32_t MCSPI0_RST_CTRL;
    volatile uint32_t MCSPI1_RST_CTRL;
    volatile uint32_t MCSPI2_RST_CTRL;
    volatile uint32_t MCSPI3_RST_CTRL;
    volatile uint32_t MCSPI4_RST_CTRL;
    volatile uint32_t QSPI0_RST_CTRL;
    volatile uint32_t MCAN0_RST_CTRL;
    volatile uint32_t MCAN1_RST_CTRL;
    volatile uint32_t MCAN2_RST_CTRL;
    volatile uint32_t MCAN3_RST_CTRL;
    volatile uint32_t I2C0_RST_CTRL;
    volatile uint32_t I2C1_RST_CTRL;
    volatile uint32_t I2C2_RST_CTRL;
    volatile uint32_t I2C3_RST_CTRL;
    volatile uint32_t UART0_RST_CTRL;
    volatile uint32_t UART1_RST_CTRL;
    volatile uint32_t UART2_RST_CTRL;
    volatile uint32_t UART3_RST_CTRL;
    volatile uint32_t UART4_RST_CTRL;
    volatile uint32_t UART5_RST_CTRL;
    volatile uint32_t LIN0_RST_CTRL;
    volatile uint32_t LIN1_RST_CTRL;
    volatile uint32_t LIN2_RST_CTRL;
    volatile uint32_t LIN3_RST_CTRL;
    volatile uint32_t LIN4_RST_CTRL;
    volatile uint8_t  Resv_1496[4];
    volatile uint32_t EDMA_RST_CTRL;
    volatile uint32_t INFRA_RST_CTRL;
    volatile uint32_t CPSW_RST_CTRL;
    volatile uint32_t ICSSM0_RST_CTRL;
    volatile uint32_t MMC0_RST_CTRL;
    volatile uint32_t GPIO0_RST_CTRL;
    volatile uint32_t GPIO1_RST_CTRL;
    volatile uint32_t GPIO2_RST_CTRL;
    volatile uint32_t GPIO3_RST_CTRL;
    volatile uint32_t SPINLOCK0_RST_CTRL;
    volatile uint32_t GPMC_RST_CTRL;
    volatile uint32_t TEMPSENSE_32K_RST_CTRL;
    volatile uint32_t MSS_ELM_RST_CTRL;
    volatile uint8_t  Resv_1792[244];
    volatile uint32_t L2OCRAM_BANK0_PD_CTRL;
    volatile uint32_t L2OCRAM_BANK1_PD_CTRL;
    volatile uint32_t L2OCRAM_BANK2_PD_CTRL;
    volatile uint32_t L2OCRAM_BANK3_PD_CTRL;
    volatile uint32_t L2OCRAM_BANK0_PD_STATUS;
    volatile uint32_t L2OCRAM_BANK1_PD_STATUS;
    volatile uint32_t L2OCRAM_BANK2_PD_STATUS;
    volatile uint32_t L2OCRAM_BANK3_PD_STATUS;
    volatile uint32_t HW_REG0;
    volatile uint32_t HW_REG1;
    volatile uint32_t HW_REG2;
    volatile uint32_t HW_REG3;
    volatile uint8_t  Resv_2048[208];
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
    volatile uint8_t  Resv_4048[1920];
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
    volatile uint32_t LOCK0_KICK0;               /*  - KICK0 component */
    volatile uint32_t LOCK0_KICK1;               /*  - KICK1 component */
    volatile uint32_t INTR_RAW_STATUS;           /* Interrupt Raw Status/Set Register */
    volatile uint32_t INTR_ENABLED_STATUS_CLEAR;   /* Interrupt Enabled Status/Clear register */
    volatile uint32_t INTR_ENABLE;               /* Interrupt Enable register */
    volatile uint32_t INTR_ENABLE_CLEAR;         /* Interrupt Enable Clear register */
    volatile uint32_t EOI;                       /* EOI register */
    volatile uint32_t FAULT_ADDRESS;             /* Fault Address register */
    volatile uint32_t FAULT_TYPE_STATUS;         /* Fault Type Status register */
    volatile uint32_t FAULT_ATTR_STATUS;         /* Fault Attribute Status register */
    volatile uint32_t FAULT_CLEAR;               /* Fault Clear register */
} SDL_mss_rcmRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define SDL_MSS_RCM_PID                                                   (0x00000000U)
#define SDL_MSS_RCM_R5SS0_RST_STATUS                                      (0x00000010U)
#define SDL_MSS_RCM_R5SS0_RST_CAUSE_CLR                                   (0x00000014U)
#define SDL_MSS_RCM_R5SS0_DBG_RST_EN                                      (0x00000018U)
#define SDL_MSS_RCM_R5SS0_RST_ASSERDLY                                    (0x0000001CU)
#define SDL_MSS_RCM_R5SS0_RST2ASSERTDLY                                   (0x00000020U)
#define SDL_MSS_RCM_R5SS0_RST_WFICHECK                                    (0x00000024U)
#define SDL_MSS_RCM_R5SS1_RST_STATUS                                      (0x00000030U)
#define SDL_MSS_RCM_R5SS1_RST_CAUSE_CLR                                   (0x00000034U)
#define SDL_MSS_RCM_R5SS1_DBG_RST_EN                                      (0x00000038U)
#define SDL_MSS_RCM_R5SS1_RST_ASSERDLY                                    (0x0000003CU)
#define SDL_MSS_RCM_R5SS1_RST2ASSERTDLY                                   (0x00000040U)
#define SDL_MSS_RCM_R5SS1_RST_WFICHECK                                    (0x00000044U)
#define SDL_MSS_RCM_MCAN0_CLK_SRC_SEL                                     (0x00000100U)
#define SDL_MSS_RCM_MCAN1_CLK_SRC_SEL                                     (0x00000104U)
#define SDL_MSS_RCM_MCAN2_CLK_SRC_SEL                                     (0x00000108U)
#define SDL_MSS_RCM_MCAN3_CLK_SRC_SEL                                     (0x0000010CU)
#define SDL_MSS_RCM_QSPI0_CLK_SRC_SEL                                     (0x00000110U)
#define SDL_MSS_RCM_RTI0_CLK_SRC_SEL                                      (0x00000114U)
#define SDL_MSS_RCM_RTI1_CLK_SRC_SEL                                      (0x00000118U)
#define SDL_MSS_RCM_RTI2_CLK_SRC_SEL                                      (0x0000011CU)
#define SDL_MSS_RCM_RTI3_CLK_SRC_SEL                                      (0x00000120U)
#define SDL_MSS_RCM_WDT0_CLK_SRC_SEL                                      (0x00000128U)
#define SDL_MSS_RCM_WDT1_CLK_SRC_SEL                                      (0x0000012CU)
#define SDL_MSS_RCM_WDT2_CLK_SRC_SEL                                      (0x00000130U)
#define SDL_MSS_RCM_WDT3_CLK_SRC_SEL                                      (0x00000134U)
#define SDL_MSS_RCM_MCSPI0_CLK_SRC_SEL                                    (0x0000013CU)
#define SDL_MSS_RCM_MCSPI1_CLK_SRC_SEL                                    (0x00000140U)
#define SDL_MSS_RCM_MCSPI2_CLK_SRC_SEL                                    (0x00000144U)
#define SDL_MSS_RCM_MCSPI3_CLK_SRC_SEL                                    (0x00000148U)
#define SDL_MSS_RCM_MCSPI4_CLK_SRC_SEL                                    (0x0000014CU)
#define SDL_MSS_RCM_MMC0_CLK_SRC_SEL                                      (0x00000150U)
#define SDL_MSS_RCM_ICSSM0_UART0_CLK_SRC_SEL                              (0x00000154U)
#define SDL_MSS_RCM_CPTS_CLK_SRC_SEL                                      (0x00000158U)
#define SDL_MSS_RCM_GPMC_CLK_SRC_SEL                                      (0x0000015CU)
#define SDL_MSS_RCM_CONTROLSS_PLL_CLK_SRC_SEL                             (0x00000160U)
#define SDL_MSS_RCM_I2C_CLK_SRC_SEL                                       (0x00000164U)
#define SDL_MSS_RCM_LIN0_UART0_CLK_SRC_SEL                                (0x00000174U)
#define SDL_MSS_RCM_LIN1_UART1_CLK_SRC_SEL                                (0x00000178U)
#define SDL_MSS_RCM_LIN2_UART2_CLK_SRC_SEL                                (0x0000017CU)
#define SDL_MSS_RCM_LIN3_UART3_CLK_SRC_SEL                                (0x00000180U)
#define SDL_MSS_RCM_LIN4_UART4_CLK_SRC_SEL                                (0x00000184U)
#define SDL_MSS_RCM_LIN5_UART5_CLK_SRC_SEL                                (0x00000188U)
#define SDL_MSS_RCM_MCAN0_CLK_DIV_VAL                                     (0x00000200U)
#define SDL_MSS_RCM_MCAN1_CLK_DIV_VAL                                     (0x00000204U)
#define SDL_MSS_RCM_MCAN2_CLK_DIV_VAL                                     (0x00000208U)
#define SDL_MSS_RCM_MCAN3_CLK_DIV_VAL                                     (0x0000020CU)
#define SDL_MSS_RCM_QSPI0_CLK_DIV_VAL                                     (0x00000210U)
#define SDL_MSS_RCM_RTI0_CLK_DIV_VAL                                      (0x00000214U)
#define SDL_MSS_RCM_RTI1_CLK_DIV_VAL                                      (0x00000218U)
#define SDL_MSS_RCM_RTI2_CLK_DIV_VAL                                      (0x0000021CU)
#define SDL_MSS_RCM_RTI3_CLK_DIV_VAL                                      (0x00000220U)
#define SDL_MSS_RCM_WDT0_CLK_DIV_VAL                                      (0x00000228U)
#define SDL_MSS_RCM_WDT1_CLK_DIV_VAL                                      (0x0000022CU)
#define SDL_MSS_RCM_WDT2_CLK_DIV_VAL                                      (0x00000230U)
#define SDL_MSS_RCM_WDT3_CLK_DIV_VAL                                      (0x00000234U)
#define SDL_MSS_RCM_MCSPI0_CLK_DIV_VAL                                    (0x0000023CU)
#define SDL_MSS_RCM_MCSPI1_CLK_DIV_VAL                                    (0x00000240U)
#define SDL_MSS_RCM_MCSPI2_CLK_DIV_VAL                                    (0x00000244U)
#define SDL_MSS_RCM_MCSPI3_CLK_DIV_VAL                                    (0x00000248U)
#define SDL_MSS_RCM_MCSPI4_CLK_DIV_VAL                                    (0x0000024CU)
#define SDL_MSS_RCM_MMC0_CLK_DIV_VAL                                      (0x00000250U)
#define SDL_MSS_RCM_ICSSM0_UART_CLK_DIV_VAL                               (0x00000254U)
#define SDL_MSS_RCM_CPTS_CLK_DIV_VAL                                      (0x00000258U)
#define SDL_MSS_RCM_GPMC_CLK_DIV_VAL                                      (0x0000025CU)
#define SDL_MSS_RCM_CONTROLSS_PLL_CLK_DIV_VAL                             (0x00000260U)
#define SDL_MSS_RCM_I2C_CLK_DIV_VAL                                       (0x00000264U)
#define SDL_MSS_RCM_LIN0_UART0_CLK_DIV_VAL                                (0x00000274U)
#define SDL_MSS_RCM_LIN1_UART1_CLK_DIV_VAL                                (0x00000278U)
#define SDL_MSS_RCM_LIN2_UART2_CLK_DIV_VAL                                (0x0000027CU)
#define SDL_MSS_RCM_LIN3_UART3_CLK_DIV_VAL                                (0x00000280U)
#define SDL_MSS_RCM_LIN4_UART4_CLK_DIV_VAL                                (0x00000284U)
#define SDL_MSS_RCM_LIN5_UART5_CLK_DIV_VAL                                (0x00000288U)
#define SDL_MSS_RCM_RGMII_250_CLK_DIV_VAL                                 (0x0000028CU)
#define SDL_MSS_RCM_RGMII_50_CLK_DIV_VAL                                  (0x00000290U)
#define SDL_MSS_RCM_RGMII_5_CLK_DIV_VAL                                   (0x00000294U)
#define SDL_MSS_RCM_XTAL_MMC_32K_CLK_DIV_VAL                              (0x00000298U)
#define SDL_MSS_RCM_XTAL_TEMPSENSE_32K_CLK_DIV_VAL                        (0x0000029CU)
#define SDL_MSS_RCM_MSS_ELM_CLK_DIV_VAL                                   (0x000002A0U)
#define SDL_MSS_RCM_MCAN0_CLK_GATE                                        (0x00000300U)
#define SDL_MSS_RCM_MCAN1_CLK_GATE                                        (0x00000304U)
#define SDL_MSS_RCM_MCAN2_CLK_GATE                                        (0x00000308U)
#define SDL_MSS_RCM_MCAN3_CLK_GATE                                        (0x0000030CU)
#define SDL_MSS_RCM_QSPI0_CLK_GATE                                        (0x00000310U)
#define SDL_MSS_RCM_RTI0_CLK_GATE                                         (0x00000314U)
#define SDL_MSS_RCM_RTI1_CLK_GATE                                         (0x00000318U)
#define SDL_MSS_RCM_RTI2_CLK_GATE                                         (0x0000031CU)
#define SDL_MSS_RCM_RTI3_CLK_GATE                                         (0x00000320U)
#define SDL_MSS_RCM_WDT0_CLK_GATE                                         (0x00000328U)
#define SDL_MSS_RCM_WDT1_CLK_GATE                                         (0x0000032CU)
#define SDL_MSS_RCM_WDT2_CLK_GATE                                         (0x00000330U)
#define SDL_MSS_RCM_WDT3_CLK_GATE                                         (0x00000334U)
#define SDL_MSS_RCM_MCSPI0_CLK_GATE                                       (0x0000033CU)
#define SDL_MSS_RCM_MCSPI1_CLK_GATE                                       (0x00000340U)
#define SDL_MSS_RCM_MCSPI2_CLK_GATE                                       (0x00000344U)
#define SDL_MSS_RCM_MCSPI3_CLK_GATE                                       (0x00000348U)
#define SDL_MSS_RCM_MCSPI4_CLK_GATE                                       (0x0000034CU)
#define SDL_MSS_RCM_MMC0_CLK_GATE                                         (0x00000350U)
#define SDL_MSS_RCM_ICSSM0_UART_CLK_GATE                                  (0x00000354U)
#define SDL_MSS_RCM_CPTS_CLK_GATE                                         (0x00000358U)
#define SDL_MSS_RCM_GPMC_CLK_GATE                                         (0x0000035CU)
#define SDL_MSS_RCM_CONTROLSS_PLL_CLK_GATE                                (0x00000360U)
#define SDL_MSS_RCM_I2C0_CLK_GATE                                         (0x00000364U)
#define SDL_MSS_RCM_I2C1_CLK_GATE                                         (0x00000368U)
#define SDL_MSS_RCM_I2C2_CLK_GATE                                         (0x0000036CU)
#define SDL_MSS_RCM_I2C3_CLK_GATE                                         (0x00000370U)
#define SDL_MSS_RCM_LIN0_CLK_GATE                                         (0x00000374U)
#define SDL_MSS_RCM_LIN1_CLK_GATE                                         (0x00000378U)
#define SDL_MSS_RCM_LIN2_CLK_GATE                                         (0x0000037CU)
#define SDL_MSS_RCM_LIN3_CLK_GATE                                         (0x00000380U)
#define SDL_MSS_RCM_LIN4_CLK_GATE                                         (0x00000384U)
#define SDL_MSS_RCM_UART0_CLK_GATE                                        (0x0000038CU)
#define SDL_MSS_RCM_UART1_CLK_GATE                                        (0x00000390U)
#define SDL_MSS_RCM_UART2_CLK_GATE                                        (0x00000394U)
#define SDL_MSS_RCM_UART3_CLK_GATE                                        (0x00000398U)
#define SDL_MSS_RCM_UART4_CLK_GATE                                        (0x0000039CU)
#define SDL_MSS_RCM_UART5_CLK_GATE                                        (0x000003A0U)
#define SDL_MSS_RCM_RGMII_250_CLK_GATE                                    (0x000003A4U)
#define SDL_MSS_RCM_RGMII_50_CLK_GATE                                     (0x000003A8U)
#define SDL_MSS_RCM_RGMII_5_CLK_GATE                                      (0x000003ACU)
#define SDL_MSS_RCM_MMC0_32K_CLK_GATE                                     (0x000003B0U)
#define SDL_MSS_RCM_TEMPSENSE_32K_CLK_GATE                                (0x000003B4U)
#define SDL_MSS_RCM_CPSW_CLK_GATE                                         (0x000003B8U)
#define SDL_MSS_RCM_ICSSM0_IEP_CLK_GATE                                   (0x000003BCU)
#define SDL_MSS_RCM_ICSSM0_CORE_CLK_GATE                                  (0x000003C0U)
#define SDL_MSS_RCM_MSS_ICSSM_SYS_CLK_GATE                                (0x000003C4U)
#define SDL_MSS_RCM_MSS_ELM_CLK_GATE                                      (0x000003C8U)
#define SDL_MSS_RCM_R5SS0_CORE0_GATE                                      (0x000003CCU)
#define SDL_MSS_RCM_R5SS1_CORE0_GATE                                      (0x000003D0U)
#define SDL_MSS_RCM_R5SS0_CORE1_GATE                                      (0x000003D4U)
#define SDL_MSS_RCM_R5SS1_CORE1_GATE                                      (0x000003D8U)
#define SDL_MSS_RCM_MCAN0_CLK_STATUS                                      (0x00000400U)
#define SDL_MSS_RCM_MCAN1_CLK_STATUS                                      (0x00000404U)
#define SDL_MSS_RCM_MCAN2_CLK_STATUS                                      (0x00000408U)
#define SDL_MSS_RCM_MCAN3_CLK_STATUS                                      (0x0000040CU)
#define SDL_MSS_RCM_QSPI0_CLK_STATUS                                      (0x00000410U)
#define SDL_MSS_RCM_RTI0_CLK_STATUS                                       (0x00000414U)
#define SDL_MSS_RCM_RTI1_CLK_STATUS                                       (0x00000418U)
#define SDL_MSS_RCM_RTI2_CLK_STATUS                                       (0x0000041CU)
#define SDL_MSS_RCM_RTI3_CLK_STATUS                                       (0x00000420U)
#define SDL_MSS_RCM_WDT0_CLK_STATUS                                       (0x00000428U)
#define SDL_MSS_RCM_WDT1_CLK_STATUS                                       (0x0000042CU)
#define SDL_MSS_RCM_WDT2_CLK_STATUS                                       (0x00000430U)
#define SDL_MSS_RCM_WDT3_CLK_STATUS                                       (0x00000434U)
#define SDL_MSS_RCM_MCSPI0_CLK_STATUS                                     (0x0000043CU)
#define SDL_MSS_RCM_MCSPI1_CLK_STATUS                                     (0x00000440U)
#define SDL_MSS_RCM_MCSPI2_CLK_STATUS                                     (0x00000444U)
#define SDL_MSS_RCM_MCSPI3_CLK_STATUS                                     (0x00000448U)
#define SDL_MSS_RCM_MCSPI4_CLK_STATUS                                     (0x0000044CU)
#define SDL_MSS_RCM_MMC0_CLK_STATUS                                       (0x00000450U)
#define SDL_MSS_RCM_ICSSM0_UART_CLK_STATUS                                (0x00000454U)
#define SDL_MSS_RCM_CPTS_CLK_STATUS                                       (0x00000458U)
#define SDL_MSS_RCM_GPMC_CLK_STATUS                                       (0x0000045CU)
#define SDL_MSS_RCM_CONTROLSS_PLL_CLK_STATUS                              (0x00000460U)
#define SDL_MSS_RCM_I2C_CLK_STATUS                                        (0x00000464U)
#define SDL_MSS_RCM_LIN0_UART0_CLK_STATUS                                 (0x00000474U)
#define SDL_MSS_RCM_LIN1_UART1_CLK_STATUS                                 (0x00000478U)
#define SDL_MSS_RCM_LIN2_UART2_CLK_STATUS                                 (0x0000047CU)
#define SDL_MSS_RCM_LIN3_UART3_CLK_STATUS                                 (0x00000480U)
#define SDL_MSS_RCM_LIN4_UART4_CLK_STATUS                                 (0x00000484U)
#define SDL_MSS_RCM_LIN5_UART5_CLK_STATUS                                 (0x00000488U)
#define SDL_MSS_RCM_RGMII_250_CLK_STATUS                                  (0x0000048CU)
#define SDL_MSS_RCM_RGMII_50_CLK_STATUS                                   (0x00000490U)
#define SDL_MSS_RCM_RGMII_5_CLK_STATUS                                    (0x00000494U)
#define SDL_MSS_RCM_MMC0_32K_CLK_STATUS                                   (0x0000049CU)
#define SDL_MSS_RCM_TEMPSENSE_32K_CLK_STATUS                              (0x000004A0U)
#define SDL_MSS_RCM_MSS_ELM_CLK_STATUS                                    (0x000004A4U)
#define SDL_MSS_RCM_R5SS0_POR_RST_CTRL                                    (0x00000500U)
#define SDL_MSS_RCM_R5SS1_POR_RST_CTRL                                    (0x00000504U)
#define SDL_MSS_RCM_R5SS0_CORE0_GRST_CTRL                                 (0x00000508U)
#define SDL_MSS_RCM_R5SS1_CORE0_GRST_CTRL                                 (0x0000050CU)
#define SDL_MSS_RCM_R5SS0_CORE1_GRST_CTRL                                 (0x00000510U)
#define SDL_MSS_RCM_R5SS1_CORE1_GRST_CTRL                                 (0x00000514U)
#define SDL_MSS_RCM_R5SS0_CORE0_LRST_CTRL                                 (0x00000518U)
#define SDL_MSS_RCM_R5SS1_CORE0_LRST_CTRL                                 (0x0000051CU)
#define SDL_MSS_RCM_R5SS0_CORE1_LRST_CTRL                                 (0x00000520U)
#define SDL_MSS_RCM_R5SS1_CORE1_LRST_CTRL                                 (0x00000524U)
#define SDL_MSS_RCM_R5SS0_VIM0_RST_CTRL                                   (0x00000528U)
#define SDL_MSS_RCM_R5SS1_VIM0_RST_CTRL                                   (0x0000052CU)
#define SDL_MSS_RCM_R5SS0_VIM1_RST_CTRL                                   (0x00000530U)
#define SDL_MSS_RCM_R5SS1_VIM1_RST_CTRL                                   (0x00000534U)
#define SDL_MSS_RCM_MCRC0_RST_CTRL                                        (0x00000538U)
#define SDL_MSS_RCM_RTI0_RST_CTRL                                         (0x0000053CU)
#define SDL_MSS_RCM_RTI1_RST_CTRL                                         (0x00000540U)
#define SDL_MSS_RCM_RTI2_RST_CTRL                                         (0x00000544U)
#define SDL_MSS_RCM_RTI3_RST_CTRL                                         (0x00000548U)
#define SDL_MSS_RCM_WDT0_RST_CTRL                                         (0x0000054CU)
#define SDL_MSS_RCM_WDT1_RST_CTRL                                         (0x00000550U)
#define SDL_MSS_RCM_WDT2_RST_CTRL                                         (0x00000554U)
#define SDL_MSS_RCM_WDT3_RST_CTRL                                         (0x00000558U)
#define SDL_MSS_RCM_TOP_ESM_RST_CTRL                                      (0x0000055CU)
#define SDL_MSS_RCM_DCC0_RST_CTRL                                         (0x00000560U)
#define SDL_MSS_RCM_DCC1_RST_CTRL                                         (0x00000564U)
#define SDL_MSS_RCM_DCC2_RST_CTRL                                         (0x00000568U)
#define SDL_MSS_RCM_DCC3_RST_CTRL                                         (0x0000056CU)
#define SDL_MSS_RCM_MCSPI0_RST_CTRL                                       (0x00000570U)
#define SDL_MSS_RCM_MCSPI1_RST_CTRL                                       (0x00000574U)
#define SDL_MSS_RCM_MCSPI2_RST_CTRL                                       (0x00000578U)
#define SDL_MSS_RCM_MCSPI3_RST_CTRL                                       (0x0000057CU)
#define SDL_MSS_RCM_MCSPI4_RST_CTRL                                       (0x00000580U)
#define SDL_MSS_RCM_QSPI0_RST_CTRL                                        (0x00000584U)
#define SDL_MSS_RCM_MCAN0_RST_CTRL                                        (0x00000588U)
#define SDL_MSS_RCM_MCAN1_RST_CTRL                                        (0x0000058CU)
#define SDL_MSS_RCM_MCAN2_RST_CTRL                                        (0x00000590U)
#define SDL_MSS_RCM_MCAN3_RST_CTRL                                        (0x00000594U)
#define SDL_MSS_RCM_I2C0_RST_CTRL                                         (0x00000598U)
#define SDL_MSS_RCM_I2C1_RST_CTRL                                         (0x0000059CU)
#define SDL_MSS_RCM_I2C2_RST_CTRL                                         (0x000005A0U)
#define SDL_MSS_RCM_I2C3_RST_CTRL                                         (0x000005A4U)
#define SDL_MSS_RCM_UART0_RST_CTRL                                        (0x000005A8U)
#define SDL_MSS_RCM_UART1_RST_CTRL                                        (0x000005ACU)
#define SDL_MSS_RCM_UART2_RST_CTRL                                        (0x000005B0U)
#define SDL_MSS_RCM_UART3_RST_CTRL                                        (0x000005B4U)
#define SDL_MSS_RCM_UART4_RST_CTRL                                        (0x000005B8U)
#define SDL_MSS_RCM_UART5_RST_CTRL                                        (0x000005BCU)
#define SDL_MSS_RCM_LIN0_RST_CTRL                                         (0x000005C0U)
#define SDL_MSS_RCM_LIN1_RST_CTRL                                         (0x000005C4U)
#define SDL_MSS_RCM_LIN2_RST_CTRL                                         (0x000005C8U)
#define SDL_MSS_RCM_LIN3_RST_CTRL                                         (0x000005CCU)
#define SDL_MSS_RCM_LIN4_RST_CTRL                                         (0x000005D0U)
#define SDL_MSS_RCM_EDMA_RST_CTRL                                         (0x000005D8U)
#define SDL_MSS_RCM_INFRA_RST_CTRL                                        (0x000005DCU)
#define SDL_MSS_RCM_CPSW_RST_CTRL                                         (0x000005E0U)
#define SDL_MSS_RCM_ICSSM0_RST_CTRL                                       (0x000005E4U)
#define SDL_MSS_RCM_MMC0_RST_CTRL                                         (0x000005E8U)
#define SDL_MSS_RCM_GPIO0_RST_CTRL                                        (0x000005ECU)
#define SDL_MSS_RCM_GPIO1_RST_CTRL                                        (0x000005F0U)
#define SDL_MSS_RCM_GPIO2_RST_CTRL                                        (0x000005F4U)
#define SDL_MSS_RCM_GPIO3_RST_CTRL                                        (0x000005F8U)
#define SDL_MSS_RCM_SPINLOCK0_RST_CTRL                                    (0x000005FCU)
#define SDL_MSS_RCM_GPMC_RST_CTRL                                         (0x00000600U)
#define SDL_MSS_RCM_TEMPSENSE_32K_RST_CTRL                                (0x00000604U)
#define SDL_MSS_RCM_MSS_ELM_RST_CTRL                                      (0x00000608U)
#define SDL_MSS_RCM_L2OCRAM_BANK0_PD_CTRL                                 (0x00000700U)
#define SDL_MSS_RCM_L2OCRAM_BANK1_PD_CTRL                                 (0x00000704U)
#define SDL_MSS_RCM_L2OCRAM_BANK2_PD_CTRL                                 (0x00000708U)
#define SDL_MSS_RCM_L2OCRAM_BANK3_PD_CTRL                                 (0x0000070CU)
#define SDL_MSS_RCM_L2OCRAM_BANK0_PD_STATUS                               (0x00000710U)
#define SDL_MSS_RCM_L2OCRAM_BANK1_PD_STATUS                               (0x00000714U)
#define SDL_MSS_RCM_L2OCRAM_BANK2_PD_STATUS                               (0x00000718U)
#define SDL_MSS_RCM_L2OCRAM_BANK3_PD_STATUS                               (0x0000071CU)
#define SDL_MSS_RCM_HW_REG0                                               (0x00000720U)
#define SDL_MSS_RCM_HW_REG1                                               (0x00000724U)
#define SDL_MSS_RCM_HW_REG2                                               (0x00000728U)
#define SDL_MSS_RCM_HW_REG3                                               (0x0000072CU)
#define SDL_MSS_RCM_HSM_RTIA_CLK_SRC_SEL                                  (0x00000800U)
#define SDL_MSS_RCM_HSM_WDT_CLK_SRC_SEL                                   (0x00000804U)
#define SDL_MSS_RCM_HSM_RTC_CLK_SRC_SEL                                   (0x00000808U)
#define SDL_MSS_RCM_HSM_DMTA_CLK_SRC_SEL                                  (0x0000080CU)
#define SDL_MSS_RCM_HSM_DMTB_CLK_SRC_SEL                                  (0x00000810U)
#define SDL_MSS_RCM_HSM_RTI_CLK_DIV_VAL                                   (0x00000814U)
#define SDL_MSS_RCM_HSM_WDT_CLK_DIV_VAL                                   (0x00000818U)
#define SDL_MSS_RCM_HSM_RTC_CLK_DIV_VAL                                   (0x0000081CU)
#define SDL_MSS_RCM_HSM_DMTA_CLK_DIV_VAL                                  (0x00000820U)
#define SDL_MSS_RCM_HSM_DMTB_CLK_DIV_VAL                                  (0x00000824U)
#define SDL_MSS_RCM_HSM_RTI_CLK_GATE                                      (0x00000828U)
#define SDL_MSS_RCM_HSM_WDT_CLK_GATE                                      (0x0000082CU)
#define SDL_MSS_RCM_HSM_RTC_CLK_GATE                                      (0x00000830U)
#define SDL_MSS_RCM_HSM_DMTA_CLK_GATE                                     (0x00000834U)
#define SDL_MSS_RCM_HSM_DMTB_CLK_GATE                                     (0x00000838U)
#define SDL_MSS_RCM_HSM_RTI_CLK_STATUS                                    (0x0000083CU)
#define SDL_MSS_RCM_HSM_WDT_CLK_STATUS                                    (0x00000840U)
#define SDL_MSS_RCM_HSM_RTC_CLK_STATUS                                    (0x00000844U)
#define SDL_MSS_RCM_HSM_DMTA_CLK_STATUS                                   (0x00000848U)
#define SDL_MSS_RCM_HSM_DMTB_CLK_STATUS                                   (0x0000084CU)
#define SDL_MSS_RCM_HW_SPARE_RW0                                          (0x00000FD0U)
#define SDL_MSS_RCM_HW_SPARE_RW1                                          (0x00000FD4U)
#define SDL_MSS_RCM_HW_SPARE_RW2                                          (0x00000FD8U)
#define SDL_MSS_RCM_HW_SPARE_RW3                                          (0x00000FDCU)
#define SDL_MSS_RCM_HW_SPARE_RO0                                          (0x00000FE0U)
#define SDL_MSS_RCM_HW_SPARE_RO1                                          (0x00000FE4U)
#define SDL_MSS_RCM_HW_SPARE_RO2                                          (0x00000FE8U)
#define SDL_MSS_RCM_HW_SPARE_RO3                                          (0x00000FECU)
#define SDL_MSS_RCM_HW_SPARE_WPH                                          (0x00000FF0U)
#define SDL_MSS_RCM_HW_SPARE_REC                                          (0x00000FF4U)
#define SDL_MSS_RCM_LOCK0_KICK0                                           (0x00001008U)
#define SDL_MSS_RCM_LOCK0_KICK1                                           (0x0000100CU)
#define SDL_MSS_RCM_INTR_RAW_STATUS                                       (0x00001010U)
#define SDL_MSS_RCM_INTR_ENABLED_STATUS_CLEAR                             (0x00001014U)
#define SDL_MSS_RCM_INTR_ENABLE                                           (0x00001018U)
#define SDL_MSS_RCM_INTR_ENABLE_CLEAR                                     (0x0000101CU)
#define SDL_MSS_RCM_EOI                                                   (0x00001020U)
#define SDL_MSS_RCM_FAULT_ADDRESS                                         (0x00001024U)
#define SDL_MSS_RCM_FAULT_TYPE_STATUS                                     (0x00001028U)
#define SDL_MSS_RCM_FAULT_ATTR_STATUS                                     (0x0000102CU)
#define SDL_MSS_RCM_FAULT_CLEAR                                           (0x00001030U)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* PID */

#define SDL_MSS_RCM_PID_PID_MINOR_MASK                                    (0x0000003FU)
#define SDL_MSS_RCM_PID_PID_MINOR_SHIFT                                   (0x00000000U)
#define SDL_MSS_RCM_PID_PID_MINOR_RESETVAL                                (0x00000014U)
#define SDL_MSS_RCM_PID_PID_MINOR_MAX                                     (0x0000003FU)

#define SDL_MSS_RCM_PID_PID_CUSTOM_MASK                                   (0x000000C0U)
#define SDL_MSS_RCM_PID_PID_CUSTOM_SHIFT                                  (0x00000006U)
#define SDL_MSS_RCM_PID_PID_CUSTOM_RESETVAL                               (0x00000000U)
#define SDL_MSS_RCM_PID_PID_CUSTOM_MAX                                    (0x00000003U)

#define SDL_MSS_RCM_PID_PID_MAJOR_MASK                                    (0x00000700U)
#define SDL_MSS_RCM_PID_PID_MAJOR_SHIFT                                   (0x00000008U)
#define SDL_MSS_RCM_PID_PID_MAJOR_RESETVAL                                (0x00000002U)
#define SDL_MSS_RCM_PID_PID_MAJOR_MAX                                     (0x00000007U)

#define SDL_MSS_RCM_PID_PID_MISC_MASK                                     (0x0000F800U)
#define SDL_MSS_RCM_PID_PID_MISC_SHIFT                                    (0x0000000BU)
#define SDL_MSS_RCM_PID_PID_MISC_RESETVAL                                 (0x00000000U)
#define SDL_MSS_RCM_PID_PID_MISC_MAX                                      (0x0000001FU)

#define SDL_MSS_RCM_PID_PID_MSB16_MASK                                    (0xFFFF0000U)
#define SDL_MSS_RCM_PID_PID_MSB16_SHIFT                                   (0x00000010U)
#define SDL_MSS_RCM_PID_PID_MSB16_RESETVAL                                (0x00006180U)
#define SDL_MSS_RCM_PID_PID_MSB16_MAX                                     (0x0000FFFFU)

#define SDL_MSS_RCM_PID_RESETVAL                                          (0x61800214U)

/* R5SS0_RST_STATUS */

#define SDL_MSS_RCM_R5SS0_RST_STATUS_CAUSE_MASK                           (0x000007FFU)
#define SDL_MSS_RCM_R5SS0_RST_STATUS_CAUSE_SHIFT                          (0x00000000U)
#define SDL_MSS_RCM_R5SS0_RST_STATUS_CAUSE_RESETVAL                       (0x00000003U)
#define SDL_MSS_RCM_R5SS0_RST_STATUS_CAUSE_MAX                            (0x000007FFU)

#define SDL_MSS_RCM_R5SS0_RST_STATUS_RESETVAL                             (0x00000003U)

/* R5SS0_RST_CAUSE_CLR */

#define SDL_MSS_RCM_R5SS0_RST_CAUSE_CLR_CLR_MASK                          (0x00000007U)
#define SDL_MSS_RCM_R5SS0_RST_CAUSE_CLR_CLR_SHIFT                         (0x00000000U)
#define SDL_MSS_RCM_R5SS0_RST_CAUSE_CLR_CLR_RESETVAL                      (0x00000000U)
#define SDL_MSS_RCM_R5SS0_RST_CAUSE_CLR_CLR_MAX                           (0x00000007U)

#define SDL_MSS_RCM_R5SS0_RST_CAUSE_CLR_RESETVAL                          (0x00000000U)

/* R5SS0_DBG_RST_EN */

#define SDL_MSS_RCM_R5SS0_DBG_RST_EN_EN_CORE0_MASK                        (0x00000007U)
#define SDL_MSS_RCM_R5SS0_DBG_RST_EN_EN_CORE0_SHIFT                       (0x00000000U)
#define SDL_MSS_RCM_R5SS0_DBG_RST_EN_EN_CORE0_RESETVAL                    (0x00000000U)
#define SDL_MSS_RCM_R5SS0_DBG_RST_EN_EN_CORE0_MAX                         (0x00000007U)

#define SDL_MSS_RCM_R5SS0_DBG_RST_EN_EN_CORE1_MASK                        (0x00070000U)
#define SDL_MSS_RCM_R5SS0_DBG_RST_EN_EN_CORE1_SHIFT                       (0x00000010U)
#define SDL_MSS_RCM_R5SS0_DBG_RST_EN_EN_CORE1_RESETVAL                    (0x00000000U)
#define SDL_MSS_RCM_R5SS0_DBG_RST_EN_EN_CORE1_MAX                         (0x00000007U)

#define SDL_MSS_RCM_R5SS0_DBG_RST_EN_RESETVAL                             (0x00000000U)

/* R5SS0_RST_ASSERDLY */

#define SDL_MSS_RCM_R5SS0_RST_ASSERDLY_COUNT_MASK                         (0x000000FFU)
#define SDL_MSS_RCM_R5SS0_RST_ASSERDLY_COUNT_SHIFT                        (0x00000000U)
#define SDL_MSS_RCM_R5SS0_RST_ASSERDLY_COUNT_RESETVAL                     (0x0000000FU)
#define SDL_MSS_RCM_R5SS0_RST_ASSERDLY_COUNT_MAX                          (0x000000FFU)

#define SDL_MSS_RCM_R5SS0_RST_ASSERDLY_RESETVAL                           (0x0000000FU)

/* R5SS0_RST2ASSERTDLY */

#define SDL_MSS_RCM_R5SS0_RST2ASSERTDLY_R5SS_CORE0_COUNT_MASK             (0x000000FFU)
#define SDL_MSS_RCM_R5SS0_RST2ASSERTDLY_R5SS_CORE0_COUNT_SHIFT            (0x00000000U)
#define SDL_MSS_RCM_R5SS0_RST2ASSERTDLY_R5SS_CORE0_COUNT_RESETVAL         (0x00000000U)
#define SDL_MSS_RCM_R5SS0_RST2ASSERTDLY_R5SS_CORE0_COUNT_MAX              (0x000000FFU)

#define SDL_MSS_RCM_R5SS0_RST2ASSERTDLY_R5SS_CORE1_COUNT_MASK             (0x0000FF00U)
#define SDL_MSS_RCM_R5SS0_RST2ASSERTDLY_R5SS_CORE1_COUNT_SHIFT            (0x00000008U)
#define SDL_MSS_RCM_R5SS0_RST2ASSERTDLY_R5SS_CORE1_COUNT_RESETVAL         (0x00000000U)
#define SDL_MSS_RCM_R5SS0_RST2ASSERTDLY_R5SS_CORE1_COUNT_MAX              (0x000000FFU)

#define SDL_MSS_RCM_R5SS0_RST2ASSERTDLY_R5_CORE0_COUNT_MASK               (0x00FF0000U)
#define SDL_MSS_RCM_R5SS0_RST2ASSERTDLY_R5_CORE0_COUNT_SHIFT              (0x00000010U)
#define SDL_MSS_RCM_R5SS0_RST2ASSERTDLY_R5_CORE0_COUNT_RESETVAL           (0x00000000U)
#define SDL_MSS_RCM_R5SS0_RST2ASSERTDLY_R5_CORE0_COUNT_MAX                (0x000000FFU)

#define SDL_MSS_RCM_R5SS0_RST2ASSERTDLY_R5_CORE1_COUNT_MASK               (0xFF000000U)
#define SDL_MSS_RCM_R5SS0_RST2ASSERTDLY_R5_CORE1_COUNT_SHIFT              (0x00000018U)
#define SDL_MSS_RCM_R5SS0_RST2ASSERTDLY_R5_CORE1_COUNT_RESETVAL           (0x00000000U)
#define SDL_MSS_RCM_R5SS0_RST2ASSERTDLY_R5_CORE1_COUNT_MAX                (0x000000FFU)

#define SDL_MSS_RCM_R5SS0_RST2ASSERTDLY_RESETVAL                          (0x00000000U)

/* R5SS0_RST_WFICHECK */

#define SDL_MSS_RCM_R5SS0_RST_WFICHECK_EN_R5SS_CORE0_MASK                 (0x00000007U)
#define SDL_MSS_RCM_R5SS0_RST_WFICHECK_EN_R5SS_CORE0_SHIFT                (0x00000000U)
#define SDL_MSS_RCM_R5SS0_RST_WFICHECK_EN_R5SS_CORE0_RESETVAL             (0x00000007U)
#define SDL_MSS_RCM_R5SS0_RST_WFICHECK_EN_R5SS_CORE0_MAX                  (0x00000007U)

#define SDL_MSS_RCM_R5SS0_RST_WFICHECK_EN_R5SS_CORE1_MASK                 (0x00000700U)
#define SDL_MSS_RCM_R5SS0_RST_WFICHECK_EN_R5SS_CORE1_SHIFT                (0x00000008U)
#define SDL_MSS_RCM_R5SS0_RST_WFICHECK_EN_R5SS_CORE1_RESETVAL             (0x00000007U)
#define SDL_MSS_RCM_R5SS0_RST_WFICHECK_EN_R5SS_CORE1_MAX                  (0x00000007U)

#define SDL_MSS_RCM_R5SS0_RST_WFICHECK_EN_R5_CORE0_MASK                   (0x00070000U)
#define SDL_MSS_RCM_R5SS0_RST_WFICHECK_EN_R5_CORE0_SHIFT                  (0x00000010U)
#define SDL_MSS_RCM_R5SS0_RST_WFICHECK_EN_R5_CORE0_RESETVAL               (0x00000007U)
#define SDL_MSS_RCM_R5SS0_RST_WFICHECK_EN_R5_CORE0_MAX                    (0x00000007U)

#define SDL_MSS_RCM_R5SS0_RST_WFICHECK_EN_R5_CORE1_MASK                   (0x07000000U)
#define SDL_MSS_RCM_R5SS0_RST_WFICHECK_EN_R5_CORE1_SHIFT                  (0x00000018U)
#define SDL_MSS_RCM_R5SS0_RST_WFICHECK_EN_R5_CORE1_RESETVAL               (0x00000007U)
#define SDL_MSS_RCM_R5SS0_RST_WFICHECK_EN_R5_CORE1_MAX                    (0x00000007U)

#define SDL_MSS_RCM_R5SS0_RST_WFICHECK_RESETVAL                           (0x07070707U)

/* R5SS1_RST_STATUS */

#define SDL_MSS_RCM_R5SS1_RST_STATUS_CAUSE_MASK                           (0x000007FFU)
#define SDL_MSS_RCM_R5SS1_RST_STATUS_CAUSE_SHIFT                          (0x00000000U)
#define SDL_MSS_RCM_R5SS1_RST_STATUS_CAUSE_RESETVAL                       (0x00000003U)
#define SDL_MSS_RCM_R5SS1_RST_STATUS_CAUSE_MAX                            (0x000007FFU)

#define SDL_MSS_RCM_R5SS1_RST_STATUS_RESETVAL                             (0x00000003U)

/* R5SS1_RST_CAUSE_CLR */

#define SDL_MSS_RCM_R5SS1_RST_CAUSE_CLR_CLR_MASK                          (0x00000007U)
#define SDL_MSS_RCM_R5SS1_RST_CAUSE_CLR_CLR_SHIFT                         (0x00000000U)
#define SDL_MSS_RCM_R5SS1_RST_CAUSE_CLR_CLR_RESETVAL                      (0x00000000U)
#define SDL_MSS_RCM_R5SS1_RST_CAUSE_CLR_CLR_MAX                           (0x00000007U)

#define SDL_MSS_RCM_R5SS1_RST_CAUSE_CLR_RESETVAL                          (0x00000000U)

/* R5SS1_DBG_RST_EN */

#define SDL_MSS_RCM_R5SS1_DBG_RST_EN_EN_CORE0_MASK                        (0x00000007U)
#define SDL_MSS_RCM_R5SS1_DBG_RST_EN_EN_CORE0_SHIFT                       (0x00000000U)
#define SDL_MSS_RCM_R5SS1_DBG_RST_EN_EN_CORE0_RESETVAL                    (0x00000000U)
#define SDL_MSS_RCM_R5SS1_DBG_RST_EN_EN_CORE0_MAX                         (0x00000007U)

#define SDL_MSS_RCM_R5SS1_DBG_RST_EN_EN_CORE1_MASK                        (0x00070000U)
#define SDL_MSS_RCM_R5SS1_DBG_RST_EN_EN_CORE1_SHIFT                       (0x00000010U)
#define SDL_MSS_RCM_R5SS1_DBG_RST_EN_EN_CORE1_RESETVAL                    (0x00000000U)
#define SDL_MSS_RCM_R5SS1_DBG_RST_EN_EN_CORE1_MAX                         (0x00000007U)

#define SDL_MSS_RCM_R5SS1_DBG_RST_EN_RESETVAL                             (0x00000000U)

/* R5SS1_RST_ASSERDLY */

#define SDL_MSS_RCM_R5SS1_RST_ASSERDLY_COUNT_MASK                         (0x000000FFU)
#define SDL_MSS_RCM_R5SS1_RST_ASSERDLY_COUNT_SHIFT                        (0x00000000U)
#define SDL_MSS_RCM_R5SS1_RST_ASSERDLY_COUNT_RESETVAL                     (0x0000000FU)
#define SDL_MSS_RCM_R5SS1_RST_ASSERDLY_COUNT_MAX                          (0x000000FFU)

#define SDL_MSS_RCM_R5SS1_RST_ASSERDLY_RESETVAL                           (0x0000000FU)

/* R5SS1_RST2ASSERTDLY */

#define SDL_MSS_RCM_R5SS1_RST2ASSERTDLY_R5SS_CORE0_COUNT_MASK             (0x000000FFU)
#define SDL_MSS_RCM_R5SS1_RST2ASSERTDLY_R5SS_CORE0_COUNT_SHIFT            (0x00000000U)
#define SDL_MSS_RCM_R5SS1_RST2ASSERTDLY_R5SS_CORE0_COUNT_RESETVAL         (0x00000000U)
#define SDL_MSS_RCM_R5SS1_RST2ASSERTDLY_R5SS_CORE0_COUNT_MAX              (0x000000FFU)

#define SDL_MSS_RCM_R5SS1_RST2ASSERTDLY_R5SS_CORE1_COUNT_MASK             (0x0000FF00U)
#define SDL_MSS_RCM_R5SS1_RST2ASSERTDLY_R5SS_CORE1_COUNT_SHIFT            (0x00000008U)
#define SDL_MSS_RCM_R5SS1_RST2ASSERTDLY_R5SS_CORE1_COUNT_RESETVAL         (0x00000000U)
#define SDL_MSS_RCM_R5SS1_RST2ASSERTDLY_R5SS_CORE1_COUNT_MAX              (0x000000FFU)

#define SDL_MSS_RCM_R5SS1_RST2ASSERTDLY_R5_CORE0_COUNT_MASK               (0x00FF0000U)
#define SDL_MSS_RCM_R5SS1_RST2ASSERTDLY_R5_CORE0_COUNT_SHIFT              (0x00000010U)
#define SDL_MSS_RCM_R5SS1_RST2ASSERTDLY_R5_CORE0_COUNT_RESETVAL           (0x00000000U)
#define SDL_MSS_RCM_R5SS1_RST2ASSERTDLY_R5_CORE0_COUNT_MAX                (0x000000FFU)

#define SDL_MSS_RCM_R5SS1_RST2ASSERTDLY_R5_CORE1_COUNT_MASK               (0xFF000000U)
#define SDL_MSS_RCM_R5SS1_RST2ASSERTDLY_R5_CORE1_COUNT_SHIFT              (0x00000018U)
#define SDL_MSS_RCM_R5SS1_RST2ASSERTDLY_R5_CORE1_COUNT_RESETVAL           (0x00000000U)
#define SDL_MSS_RCM_R5SS1_RST2ASSERTDLY_R5_CORE1_COUNT_MAX                (0x000000FFU)

#define SDL_MSS_RCM_R5SS1_RST2ASSERTDLY_RESETVAL                          (0x00000000U)

/* R5SS1_RST_WFICHECK */

#define SDL_MSS_RCM_R5SS1_RST_WFICHECK_EN_R5SS_CORE0_MASK                 (0x00000007U)
#define SDL_MSS_RCM_R5SS1_RST_WFICHECK_EN_R5SS_CORE0_SHIFT                (0x00000000U)
#define SDL_MSS_RCM_R5SS1_RST_WFICHECK_EN_R5SS_CORE0_RESETVAL             (0x00000007U)
#define SDL_MSS_RCM_R5SS1_RST_WFICHECK_EN_R5SS_CORE0_MAX                  (0x00000007U)

#define SDL_MSS_RCM_R5SS1_RST_WFICHECK_EN_R5SS_CORE1_MASK                 (0x00000700U)
#define SDL_MSS_RCM_R5SS1_RST_WFICHECK_EN_R5SS_CORE1_SHIFT                (0x00000008U)
#define SDL_MSS_RCM_R5SS1_RST_WFICHECK_EN_R5SS_CORE1_RESETVAL             (0x00000007U)
#define SDL_MSS_RCM_R5SS1_RST_WFICHECK_EN_R5SS_CORE1_MAX                  (0x00000007U)

#define SDL_MSS_RCM_R5SS1_RST_WFICHECK_EN_R5_CORE0_MASK                   (0x00070000U)
#define SDL_MSS_RCM_R5SS1_RST_WFICHECK_EN_R5_CORE0_SHIFT                  (0x00000010U)
#define SDL_MSS_RCM_R5SS1_RST_WFICHECK_EN_R5_CORE0_RESETVAL               (0x00000007U)
#define SDL_MSS_RCM_R5SS1_RST_WFICHECK_EN_R5_CORE0_MAX                    (0x00000007U)

#define SDL_MSS_RCM_R5SS1_RST_WFICHECK_EN_R5_CORE1_MASK                   (0x07000000U)
#define SDL_MSS_RCM_R5SS1_RST_WFICHECK_EN_R5_CORE1_SHIFT                  (0x00000018U)
#define SDL_MSS_RCM_R5SS1_RST_WFICHECK_EN_R5_CORE1_RESETVAL               (0x00000007U)
#define SDL_MSS_RCM_R5SS1_RST_WFICHECK_EN_R5_CORE1_MAX                    (0x00000007U)

#define SDL_MSS_RCM_R5SS1_RST_WFICHECK_RESETVAL                           (0x07070707U)

/* MCAN0_CLK_SRC_SEL */

#define SDL_MSS_RCM_MCAN0_CLK_SRC_SEL_CLKSRCSEL_MASK                      (0x00000FFFU)
#define SDL_MSS_RCM_MCAN0_CLK_SRC_SEL_CLKSRCSEL_SHIFT                     (0x00000000U)
#define SDL_MSS_RCM_MCAN0_CLK_SRC_SEL_CLKSRCSEL_RESETVAL                  (0x00000000U)
#define SDL_MSS_RCM_MCAN0_CLK_SRC_SEL_CLKSRCSEL_MAX                       (0x00000FFFU)

#define SDL_MSS_RCM_MCAN0_CLK_SRC_SEL_RESETVAL                            (0x00000000U)

/* MCAN1_CLK_SRC_SEL */

#define SDL_MSS_RCM_MCAN1_CLK_SRC_SEL_CLKSRCSEL_MASK                      (0x00000FFFU)
#define SDL_MSS_RCM_MCAN1_CLK_SRC_SEL_CLKSRCSEL_SHIFT                     (0x00000000U)
#define SDL_MSS_RCM_MCAN1_CLK_SRC_SEL_CLKSRCSEL_RESETVAL                  (0x00000000U)
#define SDL_MSS_RCM_MCAN1_CLK_SRC_SEL_CLKSRCSEL_MAX                       (0x00000FFFU)

#define SDL_MSS_RCM_MCAN1_CLK_SRC_SEL_RESETVAL                            (0x00000000U)

/* MCAN2_CLK_SRC_SEL */

#define SDL_MSS_RCM_MCAN2_CLK_SRC_SEL_CLKSRCSEL_MASK                      (0x00000FFFU)
#define SDL_MSS_RCM_MCAN2_CLK_SRC_SEL_CLKSRCSEL_SHIFT                     (0x00000000U)
#define SDL_MSS_RCM_MCAN2_CLK_SRC_SEL_CLKSRCSEL_RESETVAL                  (0x00000000U)
#define SDL_MSS_RCM_MCAN2_CLK_SRC_SEL_CLKSRCSEL_MAX                       (0x00000FFFU)

#define SDL_MSS_RCM_MCAN2_CLK_SRC_SEL_RESETVAL                            (0x00000000U)

/* MCAN3_CLK_SRC_SEL */

#define SDL_MSS_RCM_MCAN3_CLK_SRC_SEL_CLKSRCSEL_MASK                      (0x00000FFFU)
#define SDL_MSS_RCM_MCAN3_CLK_SRC_SEL_CLKSRCSEL_SHIFT                     (0x00000000U)
#define SDL_MSS_RCM_MCAN3_CLK_SRC_SEL_CLKSRCSEL_RESETVAL                  (0x00000000U)
#define SDL_MSS_RCM_MCAN3_CLK_SRC_SEL_CLKSRCSEL_MAX                       (0x00000FFFU)

#define SDL_MSS_RCM_MCAN3_CLK_SRC_SEL_RESETVAL                            (0x00000000U)

/* QSPI0_CLK_SRC_SEL */

#define SDL_MSS_RCM_QSPI0_CLK_SRC_SEL_CLKSRCSEL_MASK                      (0x00000FFFU)
#define SDL_MSS_RCM_QSPI0_CLK_SRC_SEL_CLKSRCSEL_SHIFT                     (0x00000000U)
#define SDL_MSS_RCM_QSPI0_CLK_SRC_SEL_CLKSRCSEL_RESETVAL                  (0x00000000U)
#define SDL_MSS_RCM_QSPI0_CLK_SRC_SEL_CLKSRCSEL_MAX                       (0x00000FFFU)

#define SDL_MSS_RCM_QSPI0_CLK_SRC_SEL_RESETVAL                            (0x00000000U)

/* RTI0_CLK_SRC_SEL */

#define SDL_MSS_RCM_RTI0_CLK_SRC_SEL_CLKSRCSEL_MASK                       (0x00000FFFU)
#define SDL_MSS_RCM_RTI0_CLK_SRC_SEL_CLKSRCSEL_SHIFT                      (0x00000000U)
#define SDL_MSS_RCM_RTI0_CLK_SRC_SEL_CLKSRCSEL_RESETVAL                   (0x00000000U)
#define SDL_MSS_RCM_RTI0_CLK_SRC_SEL_CLKSRCSEL_MAX                        (0x00000FFFU)

#define SDL_MSS_RCM_RTI0_CLK_SRC_SEL_RESETVAL                             (0x00000000U)

/* RTI1_CLK_SRC_SEL */

#define SDL_MSS_RCM_RTI1_CLK_SRC_SEL_CLKSRCSEL_MASK                       (0x00000FFFU)
#define SDL_MSS_RCM_RTI1_CLK_SRC_SEL_CLKSRCSEL_SHIFT                      (0x00000000U)
#define SDL_MSS_RCM_RTI1_CLK_SRC_SEL_CLKSRCSEL_RESETVAL                   (0x00000000U)
#define SDL_MSS_RCM_RTI1_CLK_SRC_SEL_CLKSRCSEL_MAX                        (0x00000FFFU)

#define SDL_MSS_RCM_RTI1_CLK_SRC_SEL_RESETVAL                             (0x00000000U)

/* RTI2_CLK_SRC_SEL */

#define SDL_MSS_RCM_RTI2_CLK_SRC_SEL_CLKSRCSEL_MASK                       (0x00000FFFU)
#define SDL_MSS_RCM_RTI2_CLK_SRC_SEL_CLKSRCSEL_SHIFT                      (0x00000000U)
#define SDL_MSS_RCM_RTI2_CLK_SRC_SEL_CLKSRCSEL_RESETVAL                   (0x00000000U)
#define SDL_MSS_RCM_RTI2_CLK_SRC_SEL_CLKSRCSEL_MAX                        (0x00000FFFU)

#define SDL_MSS_RCM_RTI2_CLK_SRC_SEL_RESETVAL                             (0x00000000U)

/* RTI3_CLK_SRC_SEL */

#define SDL_MSS_RCM_RTI3_CLK_SRC_SEL_CLKSRCSEL_MASK                       (0x00000FFFU)
#define SDL_MSS_RCM_RTI3_CLK_SRC_SEL_CLKSRCSEL_SHIFT                      (0x00000000U)
#define SDL_MSS_RCM_RTI3_CLK_SRC_SEL_CLKSRCSEL_RESETVAL                   (0x00000000U)
#define SDL_MSS_RCM_RTI3_CLK_SRC_SEL_CLKSRCSEL_MAX                        (0x00000FFFU)

#define SDL_MSS_RCM_RTI3_CLK_SRC_SEL_RESETVAL                             (0x00000000U)

/* WDT0_CLK_SRC_SEL */

#define SDL_MSS_RCM_WDT0_CLK_SRC_SEL_CLKSRCSEL_MASK                       (0x00000FFFU)
#define SDL_MSS_RCM_WDT0_CLK_SRC_SEL_CLKSRCSEL_SHIFT                      (0x00000000U)
#define SDL_MSS_RCM_WDT0_CLK_SRC_SEL_CLKSRCSEL_RESETVAL                   (0x00000000U)
#define SDL_MSS_RCM_WDT0_CLK_SRC_SEL_CLKSRCSEL_MAX                        (0x00000FFFU)

#define SDL_MSS_RCM_WDT0_CLK_SRC_SEL_RESETVAL                             (0x00000000U)

/* WDT1_CLK_SRC_SEL */

#define SDL_MSS_RCM_WDT1_CLK_SRC_SEL_CLKSRCSEL_MASK                       (0x00000FFFU)
#define SDL_MSS_RCM_WDT1_CLK_SRC_SEL_CLKSRCSEL_SHIFT                      (0x00000000U)
#define SDL_MSS_RCM_WDT1_CLK_SRC_SEL_CLKSRCSEL_RESETVAL                   (0x00000000U)
#define SDL_MSS_RCM_WDT1_CLK_SRC_SEL_CLKSRCSEL_MAX                        (0x00000FFFU)

#define SDL_MSS_RCM_WDT1_CLK_SRC_SEL_RESETVAL                             (0x00000000U)

/* WDT2_CLK_SRC_SEL */

#define SDL_MSS_RCM_WDT2_CLK_SRC_SEL_CLKSRCSEL_MASK                       (0x00000FFFU)
#define SDL_MSS_RCM_WDT2_CLK_SRC_SEL_CLKSRCSEL_SHIFT                      (0x00000000U)
#define SDL_MSS_RCM_WDT2_CLK_SRC_SEL_CLKSRCSEL_RESETVAL                   (0x00000000U)
#define SDL_MSS_RCM_WDT2_CLK_SRC_SEL_CLKSRCSEL_MAX                        (0x00000FFFU)

#define SDL_MSS_RCM_WDT2_CLK_SRC_SEL_RESETVAL                             (0x00000000U)

/* WDT3_CLK_SRC_SEL */

#define SDL_MSS_RCM_WDT3_CLK_SRC_SEL_CLKSRCSEL_MASK                       (0x00000FFFU)
#define SDL_MSS_RCM_WDT3_CLK_SRC_SEL_CLKSRCSEL_SHIFT                      (0x00000000U)
#define SDL_MSS_RCM_WDT3_CLK_SRC_SEL_CLKSRCSEL_RESETVAL                   (0x00000000U)
#define SDL_MSS_RCM_WDT3_CLK_SRC_SEL_CLKSRCSEL_MAX                        (0x00000FFFU)

#define SDL_MSS_RCM_WDT3_CLK_SRC_SEL_RESETVAL                             (0x00000000U)

/* MCSPI0_CLK_SRC_SEL */

#define SDL_MSS_RCM_MCSPI0_CLK_SRC_SEL_CLKSRCSEL_MASK                     (0x00000FFFU)
#define SDL_MSS_RCM_MCSPI0_CLK_SRC_SEL_CLKSRCSEL_SHIFT                    (0x00000000U)
#define SDL_MSS_RCM_MCSPI0_CLK_SRC_SEL_CLKSRCSEL_RESETVAL                 (0x00000000U)
#define SDL_MSS_RCM_MCSPI0_CLK_SRC_SEL_CLKSRCSEL_MAX                      (0x00000FFFU)

#define SDL_MSS_RCM_MCSPI0_CLK_SRC_SEL_RESETVAL                           (0x00000000U)

/* MCSPI1_CLK_SRC_SEL */

#define SDL_MSS_RCM_MCSPI1_CLK_SRC_SEL_CLKSRCSEL_MASK                     (0x00000FFFU)
#define SDL_MSS_RCM_MCSPI1_CLK_SRC_SEL_CLKSRCSEL_SHIFT                    (0x00000000U)
#define SDL_MSS_RCM_MCSPI1_CLK_SRC_SEL_CLKSRCSEL_RESETVAL                 (0x00000000U)
#define SDL_MSS_RCM_MCSPI1_CLK_SRC_SEL_CLKSRCSEL_MAX                      (0x00000FFFU)

#define SDL_MSS_RCM_MCSPI1_CLK_SRC_SEL_RESETVAL                           (0x00000000U)

/* MCSPI2_CLK_SRC_SEL */

#define SDL_MSS_RCM_MCSPI2_CLK_SRC_SEL_CLKSRCSEL_MASK                     (0x00000FFFU)
#define SDL_MSS_RCM_MCSPI2_CLK_SRC_SEL_CLKSRCSEL_SHIFT                    (0x00000000U)
#define SDL_MSS_RCM_MCSPI2_CLK_SRC_SEL_CLKSRCSEL_RESETVAL                 (0x00000000U)
#define SDL_MSS_RCM_MCSPI2_CLK_SRC_SEL_CLKSRCSEL_MAX                      (0x00000FFFU)

#define SDL_MSS_RCM_MCSPI2_CLK_SRC_SEL_RESETVAL                           (0x00000000U)

/* MCSPI3_CLK_SRC_SEL */

#define SDL_MSS_RCM_MCSPI3_CLK_SRC_SEL_CLKSRCSEL_MASK                     (0x00000FFFU)
#define SDL_MSS_RCM_MCSPI3_CLK_SRC_SEL_CLKSRCSEL_SHIFT                    (0x00000000U)
#define SDL_MSS_RCM_MCSPI3_CLK_SRC_SEL_CLKSRCSEL_RESETVAL                 (0x00000000U)
#define SDL_MSS_RCM_MCSPI3_CLK_SRC_SEL_CLKSRCSEL_MAX                      (0x00000FFFU)

#define SDL_MSS_RCM_MCSPI3_CLK_SRC_SEL_RESETVAL                           (0x00000000U)

/* MCSPI4_CLK_SRC_SEL */

#define SDL_MSS_RCM_MCSPI4_CLK_SRC_SEL_CLKSRCSEL_MASK                     (0x00000FFFU)
#define SDL_MSS_RCM_MCSPI4_CLK_SRC_SEL_CLKSRCSEL_SHIFT                    (0x00000000U)
#define SDL_MSS_RCM_MCSPI4_CLK_SRC_SEL_CLKSRCSEL_RESETVAL                 (0x00000000U)
#define SDL_MSS_RCM_MCSPI4_CLK_SRC_SEL_CLKSRCSEL_MAX                      (0x00000FFFU)

#define SDL_MSS_RCM_MCSPI4_CLK_SRC_SEL_RESETVAL                           (0x00000000U)

/* MMC0_CLK_SRC_SEL */

#define SDL_MSS_RCM_MMC0_CLK_SRC_SEL_CLKSRCSEL_MASK                       (0x00000FFFU)
#define SDL_MSS_RCM_MMC0_CLK_SRC_SEL_CLKSRCSEL_SHIFT                      (0x00000000U)
#define SDL_MSS_RCM_MMC0_CLK_SRC_SEL_CLKSRCSEL_RESETVAL                   (0x00000000U)
#define SDL_MSS_RCM_MMC0_CLK_SRC_SEL_CLKSRCSEL_MAX                        (0x00000FFFU)

#define SDL_MSS_RCM_MMC0_CLK_SRC_SEL_RESETVAL                             (0x00000000U)

/* ICSSM0_UART0_CLK_SRC_SEL */

#define SDL_MSS_RCM_ICSSM0_UART0_CLK_SRC_SEL_CLKSRCSEL_MASK               (0x00000FFFU)
#define SDL_MSS_RCM_ICSSM0_UART0_CLK_SRC_SEL_CLKSRCSEL_SHIFT              (0x00000000U)
#define SDL_MSS_RCM_ICSSM0_UART0_CLK_SRC_SEL_CLKSRCSEL_RESETVAL           (0x00000000U)
#define SDL_MSS_RCM_ICSSM0_UART0_CLK_SRC_SEL_CLKSRCSEL_MAX                (0x00000FFFU)

#define SDL_MSS_RCM_ICSSM0_UART0_CLK_SRC_SEL_RESETVAL                     (0x00000000U)

/* CPTS_CLK_SRC_SEL */

#define SDL_MSS_RCM_CPTS_CLK_SRC_SEL_CLKSRCSEL_MASK                       (0x00000FFFU)
#define SDL_MSS_RCM_CPTS_CLK_SRC_SEL_CLKSRCSEL_SHIFT                      (0x00000000U)
#define SDL_MSS_RCM_CPTS_CLK_SRC_SEL_CLKSRCSEL_RESETVAL                   (0x00000000U)
#define SDL_MSS_RCM_CPTS_CLK_SRC_SEL_CLKSRCSEL_MAX                        (0x00000FFFU)

#define SDL_MSS_RCM_CPTS_CLK_SRC_SEL_RESETVAL                             (0x00000000U)

/* GPMC_CLK_SRC_SEL */

#define SDL_MSS_RCM_GPMC_CLK_SRC_SEL_GPMC_CLK_SRC_SEL_MASK                (0x00000FFFU)
#define SDL_MSS_RCM_GPMC_CLK_SRC_SEL_GPMC_CLK_SRC_SEL_SHIFT               (0x00000000U)
#define SDL_MSS_RCM_GPMC_CLK_SRC_SEL_GPMC_CLK_SRC_SEL_RESETVAL            (0x00000000U)
#define SDL_MSS_RCM_GPMC_CLK_SRC_SEL_GPMC_CLK_SRC_SEL_MAX                 (0x00000FFFU)

#define SDL_MSS_RCM_GPMC_CLK_SRC_SEL_RESETVAL                             (0x00000000U)

/* CONTROLSS_PLL_CLK_SRC_SEL */

#define SDL_MSS_RCM_CONTROLSS_PLL_CLK_SRC_SEL_CLKSRCSEL_MASK              (0x00000FFFU)
#define SDL_MSS_RCM_CONTROLSS_PLL_CLK_SRC_SEL_CLKSRCSEL_SHIFT             (0x00000000U)
#define SDL_MSS_RCM_CONTROLSS_PLL_CLK_SRC_SEL_CLKSRCSEL_RESETVAL          (0x00000000U)
#define SDL_MSS_RCM_CONTROLSS_PLL_CLK_SRC_SEL_CLKSRCSEL_MAX               (0x00000FFFU)

#define SDL_MSS_RCM_CONTROLSS_PLL_CLK_SRC_SEL_RESETVAL                    (0x00000000U)

/* I2C_CLK_SRC_SEL */

#define SDL_MSS_RCM_I2C_CLK_SRC_SEL_CLKSRCSEL_MASK                        (0x00000FFFU)
#define SDL_MSS_RCM_I2C_CLK_SRC_SEL_CLKSRCSEL_SHIFT                       (0x00000000U)
#define SDL_MSS_RCM_I2C_CLK_SRC_SEL_CLKSRCSEL_RESETVAL                    (0x00000000U)
#define SDL_MSS_RCM_I2C_CLK_SRC_SEL_CLKSRCSEL_MAX                         (0x00000FFFU)

#define SDL_MSS_RCM_I2C_CLK_SRC_SEL_RESETVAL                              (0x00000000U)

/* LIN0_UART0_CLK_SRC_SEL */

#define SDL_MSS_RCM_LIN0_UART0_CLK_SRC_SEL_CLKSRCSEL_MASK                 (0x00000FFFU)
#define SDL_MSS_RCM_LIN0_UART0_CLK_SRC_SEL_CLKSRCSEL_SHIFT                (0x00000000U)
#define SDL_MSS_RCM_LIN0_UART0_CLK_SRC_SEL_CLKSRCSEL_RESETVAL             (0x00000000U)
#define SDL_MSS_RCM_LIN0_UART0_CLK_SRC_SEL_CLKSRCSEL_MAX                  (0x00000FFFU)

#define SDL_MSS_RCM_LIN0_UART0_CLK_SRC_SEL_RESETVAL                       (0x00000000U)

/* LIN1_UART1_CLK_SRC_SEL */

#define SDL_MSS_RCM_LIN1_UART1_CLK_SRC_SEL_CLKSRCSEL_MASK                 (0x00000FFFU)
#define SDL_MSS_RCM_LIN1_UART1_CLK_SRC_SEL_CLKSRCSEL_SHIFT                (0x00000000U)
#define SDL_MSS_RCM_LIN1_UART1_CLK_SRC_SEL_CLKSRCSEL_RESETVAL             (0x00000000U)
#define SDL_MSS_RCM_LIN1_UART1_CLK_SRC_SEL_CLKSRCSEL_MAX                  (0x00000FFFU)

#define SDL_MSS_RCM_LIN1_UART1_CLK_SRC_SEL_RESETVAL                       (0x00000000U)

/* LIN2_UART2_CLK_SRC_SEL */

#define SDL_MSS_RCM_LIN2_UART2_CLK_SRC_SEL_CLKSRCSEL_MASK                 (0x00000FFFU)
#define SDL_MSS_RCM_LIN2_UART2_CLK_SRC_SEL_CLKSRCSEL_SHIFT                (0x00000000U)
#define SDL_MSS_RCM_LIN2_UART2_CLK_SRC_SEL_CLKSRCSEL_RESETVAL             (0x00000000U)
#define SDL_MSS_RCM_LIN2_UART2_CLK_SRC_SEL_CLKSRCSEL_MAX                  (0x00000FFFU)

#define SDL_MSS_RCM_LIN2_UART2_CLK_SRC_SEL_RESETVAL                       (0x00000000U)

/* LIN3_UART3_CLK_SRC_SEL */

#define SDL_MSS_RCM_LIN3_UART3_CLK_SRC_SEL_CLKSRCSEL_MASK                 (0x00000FFFU)
#define SDL_MSS_RCM_LIN3_UART3_CLK_SRC_SEL_CLKSRCSEL_SHIFT                (0x00000000U)
#define SDL_MSS_RCM_LIN3_UART3_CLK_SRC_SEL_CLKSRCSEL_RESETVAL             (0x00000000U)
#define SDL_MSS_RCM_LIN3_UART3_CLK_SRC_SEL_CLKSRCSEL_MAX                  (0x00000FFFU)

#define SDL_MSS_RCM_LIN3_UART3_CLK_SRC_SEL_RESETVAL                       (0x00000000U)

/* LIN4_UART4_CLK_SRC_SEL */

#define SDL_MSS_RCM_LIN4_UART4_CLK_SRC_SEL_CLKSRCSEL_MASK                 (0x00000FFFU)
#define SDL_MSS_RCM_LIN4_UART4_CLK_SRC_SEL_CLKSRCSEL_SHIFT                (0x00000000U)
#define SDL_MSS_RCM_LIN4_UART4_CLK_SRC_SEL_CLKSRCSEL_RESETVAL             (0x00000000U)
#define SDL_MSS_RCM_LIN4_UART4_CLK_SRC_SEL_CLKSRCSEL_MAX                  (0x00000FFFU)

#define SDL_MSS_RCM_LIN4_UART4_CLK_SRC_SEL_RESETVAL                       (0x00000000U)

/* LIN5_UART5_CLK_SRC_SEL */

#define SDL_MSS_RCM_LIN5_UART5_CLK_SRC_SEL_CLKSRCSEL_MASK                 (0x00000FFFU)
#define SDL_MSS_RCM_LIN5_UART5_CLK_SRC_SEL_CLKSRCSEL_SHIFT                (0x00000000U)
#define SDL_MSS_RCM_LIN5_UART5_CLK_SRC_SEL_CLKSRCSEL_RESETVAL             (0x00000000U)
#define SDL_MSS_RCM_LIN5_UART5_CLK_SRC_SEL_CLKSRCSEL_MAX                  (0x00000FFFU)

#define SDL_MSS_RCM_LIN5_UART5_CLK_SRC_SEL_RESETVAL                       (0x00000000U)

/* MCAN0_CLK_DIV_VAL */

#define SDL_MSS_RCM_MCAN0_CLK_DIV_VAL_CLKDIVR_MASK                        (0x00000FFFU)
#define SDL_MSS_RCM_MCAN0_CLK_DIV_VAL_CLKDIVR_SHIFT                       (0x00000000U)
#define SDL_MSS_RCM_MCAN0_CLK_DIV_VAL_CLKDIVR_RESETVAL                    (0x00000000U)
#define SDL_MSS_RCM_MCAN0_CLK_DIV_VAL_CLKDIVR_MAX                         (0x00000FFFU)

#define SDL_MSS_RCM_MCAN0_CLK_DIV_VAL_RESETVAL                            (0x00000000U)

/* MCAN1_CLK_DIV_VAL */

#define SDL_MSS_RCM_MCAN1_CLK_DIV_VAL_CLKDIVR_MASK                        (0x00000FFFU)
#define SDL_MSS_RCM_MCAN1_CLK_DIV_VAL_CLKDIVR_SHIFT                       (0x00000000U)
#define SDL_MSS_RCM_MCAN1_CLK_DIV_VAL_CLKDIVR_RESETVAL                    (0x00000000U)
#define SDL_MSS_RCM_MCAN1_CLK_DIV_VAL_CLKDIVR_MAX                         (0x00000FFFU)

#define SDL_MSS_RCM_MCAN1_CLK_DIV_VAL_RESETVAL                            (0x00000000U)

/* MCAN2_CLK_DIV_VAL */

#define SDL_MSS_RCM_MCAN2_CLK_DIV_VAL_CLKDIVR_MASK                        (0x00000FFFU)
#define SDL_MSS_RCM_MCAN2_CLK_DIV_VAL_CLKDIVR_SHIFT                       (0x00000000U)
#define SDL_MSS_RCM_MCAN2_CLK_DIV_VAL_CLKDIVR_RESETVAL                    (0x00000000U)
#define SDL_MSS_RCM_MCAN2_CLK_DIV_VAL_CLKDIVR_MAX                         (0x00000FFFU)

#define SDL_MSS_RCM_MCAN2_CLK_DIV_VAL_RESETVAL                            (0x00000000U)

/* MCAN3_CLK_DIV_VAL */

#define SDL_MSS_RCM_MCAN3_CLK_DIV_VAL_CLKDIVR_MASK                        (0x00000FFFU)
#define SDL_MSS_RCM_MCAN3_CLK_DIV_VAL_CLKDIVR_SHIFT                       (0x00000000U)
#define SDL_MSS_RCM_MCAN3_CLK_DIV_VAL_CLKDIVR_RESETVAL                    (0x00000000U)
#define SDL_MSS_RCM_MCAN3_CLK_DIV_VAL_CLKDIVR_MAX                         (0x00000FFFU)

#define SDL_MSS_RCM_MCAN3_CLK_DIV_VAL_RESETVAL                            (0x00000000U)

/* QSPI0_CLK_DIV_VAL */

#define SDL_MSS_RCM_QSPI0_CLK_DIV_VAL_CLKDIVR_MASK                        (0x00000FFFU)
#define SDL_MSS_RCM_QSPI0_CLK_DIV_VAL_CLKDIVR_SHIFT                       (0x00000000U)
#define SDL_MSS_RCM_QSPI0_CLK_DIV_VAL_CLKDIVR_RESETVAL                    (0x00000000U)
#define SDL_MSS_RCM_QSPI0_CLK_DIV_VAL_CLKDIVR_MAX                         (0x00000FFFU)

#define SDL_MSS_RCM_QSPI0_CLK_DIV_VAL_RESETVAL                            (0x00000000U)

/* RTI0_CLK_DIV_VAL */

#define SDL_MSS_RCM_RTI0_CLK_DIV_VAL_CLKDIVR_MASK                         (0x00000FFFU)
#define SDL_MSS_RCM_RTI0_CLK_DIV_VAL_CLKDIVR_SHIFT                        (0x00000000U)
#define SDL_MSS_RCM_RTI0_CLK_DIV_VAL_CLKDIVR_RESETVAL                     (0x00000000U)
#define SDL_MSS_RCM_RTI0_CLK_DIV_VAL_CLKDIVR_MAX                          (0x00000FFFU)

#define SDL_MSS_RCM_RTI0_CLK_DIV_VAL_RESETVAL                             (0x00000000U)

/* RTI1_CLK_DIV_VAL */

#define SDL_MSS_RCM_RTI1_CLK_DIV_VAL_CLKDIVR_MASK                         (0x00000FFFU)
#define SDL_MSS_RCM_RTI1_CLK_DIV_VAL_CLKDIVR_SHIFT                        (0x00000000U)
#define SDL_MSS_RCM_RTI1_CLK_DIV_VAL_CLKDIVR_RESETVAL                     (0x00000000U)
#define SDL_MSS_RCM_RTI1_CLK_DIV_VAL_CLKDIVR_MAX                          (0x00000FFFU)

#define SDL_MSS_RCM_RTI1_CLK_DIV_VAL_RESETVAL                             (0x00000000U)

/* RTI2_CLK_DIV_VAL */

#define SDL_MSS_RCM_RTI2_CLK_DIV_VAL_CLKDIVR_MASK                         (0x00000FFFU)
#define SDL_MSS_RCM_RTI2_CLK_DIV_VAL_CLKDIVR_SHIFT                        (0x00000000U)
#define SDL_MSS_RCM_RTI2_CLK_DIV_VAL_CLKDIVR_RESETVAL                     (0x00000000U)
#define SDL_MSS_RCM_RTI2_CLK_DIV_VAL_CLKDIVR_MAX                          (0x00000FFFU)

#define SDL_MSS_RCM_RTI2_CLK_DIV_VAL_RESETVAL                             (0x00000000U)

/* RTI3_CLK_DIV_VAL */

#define SDL_MSS_RCM_RTI3_CLK_DIV_VAL_CLKDIVR_MASK                         (0x00000FFFU)
#define SDL_MSS_RCM_RTI3_CLK_DIV_VAL_CLKDIVR_SHIFT                        (0x00000000U)
#define SDL_MSS_RCM_RTI3_CLK_DIV_VAL_CLKDIVR_RESETVAL                     (0x00000000U)
#define SDL_MSS_RCM_RTI3_CLK_DIV_VAL_CLKDIVR_MAX                          (0x00000FFFU)

#define SDL_MSS_RCM_RTI3_CLK_DIV_VAL_RESETVAL                             (0x00000000U)

/* WDT0_CLK_DIV_VAL */

#define SDL_MSS_RCM_WDT0_CLK_DIV_VAL_CLKDIVR_MASK                         (0x00000FFFU)
#define SDL_MSS_RCM_WDT0_CLK_DIV_VAL_CLKDIVR_SHIFT                        (0x00000000U)
#define SDL_MSS_RCM_WDT0_CLK_DIV_VAL_CLKDIVR_RESETVAL                     (0x00000000U)
#define SDL_MSS_RCM_WDT0_CLK_DIV_VAL_CLKDIVR_MAX                          (0x00000FFFU)

#define SDL_MSS_RCM_WDT0_CLK_DIV_VAL_RESETVAL                             (0x00000000U)

/* WDT1_CLK_DIV_VAL */

#define SDL_MSS_RCM_WDT1_CLK_DIV_VAL_CLKDIVR_MASK                         (0x00000FFFU)
#define SDL_MSS_RCM_WDT1_CLK_DIV_VAL_CLKDIVR_SHIFT                        (0x00000000U)
#define SDL_MSS_RCM_WDT1_CLK_DIV_VAL_CLKDIVR_RESETVAL                     (0x00000000U)
#define SDL_MSS_RCM_WDT1_CLK_DIV_VAL_CLKDIVR_MAX                          (0x00000FFFU)

#define SDL_MSS_RCM_WDT1_CLK_DIV_VAL_RESETVAL                             (0x00000000U)

/* WDT2_CLK_DIV_VAL */

#define SDL_MSS_RCM_WDT2_CLK_DIV_VAL_CLKDIVR_MASK                         (0x00000FFFU)
#define SDL_MSS_RCM_WDT2_CLK_DIV_VAL_CLKDIVR_SHIFT                        (0x00000000U)
#define SDL_MSS_RCM_WDT2_CLK_DIV_VAL_CLKDIVR_RESETVAL                     (0x00000000U)
#define SDL_MSS_RCM_WDT2_CLK_DIV_VAL_CLKDIVR_MAX                          (0x00000FFFU)

#define SDL_MSS_RCM_WDT2_CLK_DIV_VAL_RESETVAL                             (0x00000000U)

/* WDT3_CLK_DIV_VAL */

#define SDL_MSS_RCM_WDT3_CLK_DIV_VAL_CLKDIVR_MASK                         (0x00000FFFU)
#define SDL_MSS_RCM_WDT3_CLK_DIV_VAL_CLKDIVR_SHIFT                        (0x00000000U)
#define SDL_MSS_RCM_WDT3_CLK_DIV_VAL_CLKDIVR_RESETVAL                     (0x00000000U)
#define SDL_MSS_RCM_WDT3_CLK_DIV_VAL_CLKDIVR_MAX                          (0x00000FFFU)

#define SDL_MSS_RCM_WDT3_CLK_DIV_VAL_RESETVAL                             (0x00000000U)

/* MCSPI0_CLK_DIV_VAL */

#define SDL_MSS_RCM_MCSPI0_CLK_DIV_VAL_CLKDIVR_MASK                       (0x00000FFFU)
#define SDL_MSS_RCM_MCSPI0_CLK_DIV_VAL_CLKDIVR_SHIFT                      (0x00000000U)
#define SDL_MSS_RCM_MCSPI0_CLK_DIV_VAL_CLKDIVR_RESETVAL                   (0x00000000U)
#define SDL_MSS_RCM_MCSPI0_CLK_DIV_VAL_CLKDIVR_MAX                        (0x00000FFFU)

#define SDL_MSS_RCM_MCSPI0_CLK_DIV_VAL_RESETVAL                           (0x00000000U)

/* MCSPI1_CLK_DIV_VAL */

#define SDL_MSS_RCM_MCSPI1_CLK_DIV_VAL_CLKDIVR_MASK                       (0x00000FFFU)
#define SDL_MSS_RCM_MCSPI1_CLK_DIV_VAL_CLKDIVR_SHIFT                      (0x00000000U)
#define SDL_MSS_RCM_MCSPI1_CLK_DIV_VAL_CLKDIVR_RESETVAL                   (0x00000000U)
#define SDL_MSS_RCM_MCSPI1_CLK_DIV_VAL_CLKDIVR_MAX                        (0x00000FFFU)

#define SDL_MSS_RCM_MCSPI1_CLK_DIV_VAL_RESETVAL                           (0x00000000U)

/* MCSPI2_CLK_DIV_VAL */

#define SDL_MSS_RCM_MCSPI2_CLK_DIV_VAL_CLKDIVR_MASK                       (0x00000FFFU)
#define SDL_MSS_RCM_MCSPI2_CLK_DIV_VAL_CLKDIVR_SHIFT                      (0x00000000U)
#define SDL_MSS_RCM_MCSPI2_CLK_DIV_VAL_CLKDIVR_RESETVAL                   (0x00000000U)
#define SDL_MSS_RCM_MCSPI2_CLK_DIV_VAL_CLKDIVR_MAX                        (0x00000FFFU)

#define SDL_MSS_RCM_MCSPI2_CLK_DIV_VAL_RESETVAL                           (0x00000000U)

/* MCSPI3_CLK_DIV_VAL */

#define SDL_MSS_RCM_MCSPI3_CLK_DIV_VAL_CLKDIVR_MASK                       (0x00000FFFU)
#define SDL_MSS_RCM_MCSPI3_CLK_DIV_VAL_CLKDIVR_SHIFT                      (0x00000000U)
#define SDL_MSS_RCM_MCSPI3_CLK_DIV_VAL_CLKDIVR_RESETVAL                   (0x00000000U)
#define SDL_MSS_RCM_MCSPI3_CLK_DIV_VAL_CLKDIVR_MAX                        (0x00000FFFU)

#define SDL_MSS_RCM_MCSPI3_CLK_DIV_VAL_RESETVAL                           (0x00000000U)

/* MCSPI4_CLK_DIV_VAL */

#define SDL_MSS_RCM_MCSPI4_CLK_DIV_VAL_CLKDIVR_MASK                       (0x00000FFFU)
#define SDL_MSS_RCM_MCSPI4_CLK_DIV_VAL_CLKDIVR_SHIFT                      (0x00000000U)
#define SDL_MSS_RCM_MCSPI4_CLK_DIV_VAL_CLKDIVR_RESETVAL                   (0x00000000U)
#define SDL_MSS_RCM_MCSPI4_CLK_DIV_VAL_CLKDIVR_MAX                        (0x00000FFFU)

#define SDL_MSS_RCM_MCSPI4_CLK_DIV_VAL_RESETVAL                           (0x00000000U)

/* MMC0_CLK_DIV_VAL */

#define SDL_MSS_RCM_MMC0_CLK_DIV_VAL_CLKDIVR_MASK                         (0x00000FFFU)
#define SDL_MSS_RCM_MMC0_CLK_DIV_VAL_CLKDIVR_SHIFT                        (0x00000000U)
#define SDL_MSS_RCM_MMC0_CLK_DIV_VAL_CLKDIVR_RESETVAL                     (0x00000000U)
#define SDL_MSS_RCM_MMC0_CLK_DIV_VAL_CLKDIVR_MAX                          (0x00000FFFU)

#define SDL_MSS_RCM_MMC0_CLK_DIV_VAL_RESETVAL                             (0x00000000U)

/* ICSSM0_UART_CLK_DIV_VAL */

#define SDL_MSS_RCM_ICSSM0_UART_CLK_DIV_VAL_CLKDIVR_MASK                  (0x00000FFFU)
#define SDL_MSS_RCM_ICSSM0_UART_CLK_DIV_VAL_CLKDIVR_SHIFT                 (0x00000000U)
#define SDL_MSS_RCM_ICSSM0_UART_CLK_DIV_VAL_CLKDIVR_RESETVAL              (0x00000000U)
#define SDL_MSS_RCM_ICSSM0_UART_CLK_DIV_VAL_CLKDIVR_MAX                   (0x00000FFFU)

#define SDL_MSS_RCM_ICSSM0_UART_CLK_DIV_VAL_RESETVAL                      (0x00000000U)

/* CPTS_CLK_DIV_VAL */

#define SDL_MSS_RCM_CPTS_CLK_DIV_VAL_CLKDIVR_MASK                         (0x00000FFFU)
#define SDL_MSS_RCM_CPTS_CLK_DIV_VAL_CLKDIVR_SHIFT                        (0x00000000U)
#define SDL_MSS_RCM_CPTS_CLK_DIV_VAL_CLKDIVR_RESETVAL                     (0x00000000U)
#define SDL_MSS_RCM_CPTS_CLK_DIV_VAL_CLKDIVR_MAX                          (0x00000FFFU)

#define SDL_MSS_RCM_CPTS_CLK_DIV_VAL_RESETVAL                             (0x00000000U)

/* GPMC_CLK_DIV_VAL */

#define SDL_MSS_RCM_GPMC_CLK_DIV_VAL_CLKDIVR_MASK                         (0x00000FFFU)
#define SDL_MSS_RCM_GPMC_CLK_DIV_VAL_CLKDIVR_SHIFT                        (0x00000000U)
#define SDL_MSS_RCM_GPMC_CLK_DIV_VAL_CLKDIVR_RESETVAL                     (0x00000000U)
#define SDL_MSS_RCM_GPMC_CLK_DIV_VAL_CLKDIVR_MAX                          (0x00000FFFU)

#define SDL_MSS_RCM_GPMC_CLK_DIV_VAL_RESETVAL                             (0x00000000U)

/* CONTROLSS_PLL_CLK_DIV_VAL */

#define SDL_MSS_RCM_CONTROLSS_PLL_CLK_DIV_VAL_CLKDIVR_MASK                (0x00000FFFU)
#define SDL_MSS_RCM_CONTROLSS_PLL_CLK_DIV_VAL_CLKDIVR_SHIFT               (0x00000000U)
#define SDL_MSS_RCM_CONTROLSS_PLL_CLK_DIV_VAL_CLKDIVR_RESETVAL            (0x00000000U)
#define SDL_MSS_RCM_CONTROLSS_PLL_CLK_DIV_VAL_CLKDIVR_MAX                 (0x00000FFFU)

#define SDL_MSS_RCM_CONTROLSS_PLL_CLK_DIV_VAL_RESETVAL                    (0x00000000U)

/* I2C_CLK_DIV_VAL */

#define SDL_MSS_RCM_I2C_CLK_DIV_VAL_CLKDIVR_MASK                          (0x00000FFFU)
#define SDL_MSS_RCM_I2C_CLK_DIV_VAL_CLKDIVR_SHIFT                         (0x00000000U)
#define SDL_MSS_RCM_I2C_CLK_DIV_VAL_CLKDIVR_RESETVAL                      (0x00000000U)
#define SDL_MSS_RCM_I2C_CLK_DIV_VAL_CLKDIVR_MAX                           (0x00000FFFU)

#define SDL_MSS_RCM_I2C_CLK_DIV_VAL_RESETVAL                              (0x00000000U)

/* LIN0_UART0_CLK_DIV_VAL */

#define SDL_MSS_RCM_LIN0_UART0_CLK_DIV_VAL_CLKDIVR_MASK                   (0x00000FFFU)
#define SDL_MSS_RCM_LIN0_UART0_CLK_DIV_VAL_CLKDIVR_SHIFT                  (0x00000000U)
#define SDL_MSS_RCM_LIN0_UART0_CLK_DIV_VAL_CLKDIVR_RESETVAL               (0x00000000U)
#define SDL_MSS_RCM_LIN0_UART0_CLK_DIV_VAL_CLKDIVR_MAX                    (0x00000FFFU)

#define SDL_MSS_RCM_LIN0_UART0_CLK_DIV_VAL_RESETVAL                       (0x00000000U)

/* LIN1_UART1_CLK_DIV_VAL */

#define SDL_MSS_RCM_LIN1_UART1_CLK_DIV_VAL_CLKDIVR_MASK                   (0x00000FFFU)
#define SDL_MSS_RCM_LIN1_UART1_CLK_DIV_VAL_CLKDIVR_SHIFT                  (0x00000000U)
#define SDL_MSS_RCM_LIN1_UART1_CLK_DIV_VAL_CLKDIVR_RESETVAL               (0x00000000U)
#define SDL_MSS_RCM_LIN1_UART1_CLK_DIV_VAL_CLKDIVR_MAX                    (0x00000FFFU)

#define SDL_MSS_RCM_LIN1_UART1_CLK_DIV_VAL_RESETVAL                       (0x00000000U)

/* LIN2_UART2_CLK_DIV_VAL */

#define SDL_MSS_RCM_LIN2_UART2_CLK_DIV_VAL_CLKDIVR_MASK                   (0x00000FFFU)
#define SDL_MSS_RCM_LIN2_UART2_CLK_DIV_VAL_CLKDIVR_SHIFT                  (0x00000000U)
#define SDL_MSS_RCM_LIN2_UART2_CLK_DIV_VAL_CLKDIVR_RESETVAL               (0x00000000U)
#define SDL_MSS_RCM_LIN2_UART2_CLK_DIV_VAL_CLKDIVR_MAX                    (0x00000FFFU)

#define SDL_MSS_RCM_LIN2_UART2_CLK_DIV_VAL_RESETVAL                       (0x00000000U)

/* LIN3_UART3_CLK_DIV_VAL */

#define SDL_MSS_RCM_LIN3_UART3_CLK_DIV_VAL_CLKDIVR_MASK                   (0x00000FFFU)
#define SDL_MSS_RCM_LIN3_UART3_CLK_DIV_VAL_CLKDIVR_SHIFT                  (0x00000000U)
#define SDL_MSS_RCM_LIN3_UART3_CLK_DIV_VAL_CLKDIVR_RESETVAL               (0x00000000U)
#define SDL_MSS_RCM_LIN3_UART3_CLK_DIV_VAL_CLKDIVR_MAX                    (0x00000FFFU)

#define SDL_MSS_RCM_LIN3_UART3_CLK_DIV_VAL_RESETVAL                       (0x00000000U)

/* LIN4_UART4_CLK_DIV_VAL */

#define SDL_MSS_RCM_LIN4_UART4_CLK_DIV_VAL_CLKDIVR_MASK                   (0x00000FFFU)
#define SDL_MSS_RCM_LIN4_UART4_CLK_DIV_VAL_CLKDIVR_SHIFT                  (0x00000000U)
#define SDL_MSS_RCM_LIN4_UART4_CLK_DIV_VAL_CLKDIVR_RESETVAL               (0x00000000U)
#define SDL_MSS_RCM_LIN4_UART4_CLK_DIV_VAL_CLKDIVR_MAX                    (0x00000FFFU)

#define SDL_MSS_RCM_LIN4_UART4_CLK_DIV_VAL_RESETVAL                       (0x00000000U)

/* LIN5_UART5_CLK_DIV_VAL */

#define SDL_MSS_RCM_LIN5_UART5_CLK_DIV_VAL_CLKDIVR_MASK                   (0x00000FFFU)
#define SDL_MSS_RCM_LIN5_UART5_CLK_DIV_VAL_CLKDIVR_SHIFT                  (0x00000000U)
#define SDL_MSS_RCM_LIN5_UART5_CLK_DIV_VAL_CLKDIVR_RESETVAL               (0x00000000U)
#define SDL_MSS_RCM_LIN5_UART5_CLK_DIV_VAL_CLKDIVR_MAX                    (0x00000FFFU)

#define SDL_MSS_RCM_LIN5_UART5_CLK_DIV_VAL_RESETVAL                       (0x00000000U)

/* RGMII_250_CLK_DIV_VAL */

#define SDL_MSS_RCM_RGMII_250_CLK_DIV_VAL_CLKDIVR_MASK                    (0x00000FFFU)
#define SDL_MSS_RCM_RGMII_250_CLK_DIV_VAL_CLKDIVR_SHIFT                   (0x00000000U)
#define SDL_MSS_RCM_RGMII_250_CLK_DIV_VAL_CLKDIVR_RESETVAL                (0x00000111U)
#define SDL_MSS_RCM_RGMII_250_CLK_DIV_VAL_CLKDIVR_MAX                     (0x00000FFFU)

#define SDL_MSS_RCM_RGMII_250_CLK_DIV_VAL_RESETVAL                        (0x00000111U)

/* RGMII_50_CLK_DIV_VAL */

#define SDL_MSS_RCM_RGMII_50_CLK_DIV_VAL_CLKDIVR_MASK                     (0x00000FFFU)
#define SDL_MSS_RCM_RGMII_50_CLK_DIV_VAL_CLKDIVR_SHIFT                    (0x00000000U)
#define SDL_MSS_RCM_RGMII_50_CLK_DIV_VAL_CLKDIVR_RESETVAL                 (0x00000999U)
#define SDL_MSS_RCM_RGMII_50_CLK_DIV_VAL_CLKDIVR_MAX                      (0x00000FFFU)

#define SDL_MSS_RCM_RGMII_50_CLK_DIV_VAL_RESETVAL                         (0x00000999U)

/* RGMII_5_CLK_DIV_VAL */

#define SDL_MSS_RCM_RGMII_5_CLK_DIV_VAL_CLKDIVR_MASK                      (0x00FFFFFFU)
#define SDL_MSS_RCM_RGMII_5_CLK_DIV_VAL_CLKDIVR_SHIFT                     (0x00000000U)
#define SDL_MSS_RCM_RGMII_5_CLK_DIV_VAL_CLKDIVR_RESETVAL                  (0x00636363U)
#define SDL_MSS_RCM_RGMII_5_CLK_DIV_VAL_CLKDIVR_MAX                       (0x00FFFFFFU)

#define SDL_MSS_RCM_RGMII_5_CLK_DIV_VAL_RESETVAL                          (0x00636363U)

/* XTAL_MMC_32K_CLK_DIV_VAL */

#define SDL_MSS_RCM_XTAL_MMC_32K_CLK_DIV_VAL_CLKDIVR_MASK                 (0x3FFFFFFFU)
#define SDL_MSS_RCM_XTAL_MMC_32K_CLK_DIV_VAL_CLKDIVR_SHIFT                (0x00000000U)
#define SDL_MSS_RCM_XTAL_MMC_32K_CLK_DIV_VAL_CLKDIVR_RESETVAL             (0x30CC330CU)
#define SDL_MSS_RCM_XTAL_MMC_32K_CLK_DIV_VAL_CLKDIVR_MAX                  (0x3FFFFFFFU)

#define SDL_MSS_RCM_XTAL_MMC_32K_CLK_DIV_VAL_RESETVAL                     (0x30CC330CU)

/* XTAL_TEMPSENSE_32K_CLK_DIV_VAL */

#define SDL_MSS_RCM_XTAL_TEMPSENSE_32K_CLK_DIV_VAL_CLKDIVR_MASK           (0x3FFFFFFFU)
#define SDL_MSS_RCM_XTAL_TEMPSENSE_32K_CLK_DIV_VAL_CLKDIVR_SHIFT          (0x00000000U)
#define SDL_MSS_RCM_XTAL_TEMPSENSE_32K_CLK_DIV_VAL_CLKDIVR_RESETVAL       (0x30CC330CU)
#define SDL_MSS_RCM_XTAL_TEMPSENSE_32K_CLK_DIV_VAL_CLKDIVR_MAX            (0x3FFFFFFFU)

#define SDL_MSS_RCM_XTAL_TEMPSENSE_32K_CLK_DIV_VAL_RESETVAL               (0x30CC330CU)

/* MSS_ELM_CLK_DIV_VAL */

#define SDL_MSS_RCM_MSS_ELM_CLK_DIV_VAL_CLKDIVR_MASK                      (0x00000FFFU)
#define SDL_MSS_RCM_MSS_ELM_CLK_DIV_VAL_CLKDIVR_SHIFT                     (0x00000000U)
#define SDL_MSS_RCM_MSS_ELM_CLK_DIV_VAL_CLKDIVR_RESETVAL                  (0x00000333U)
#define SDL_MSS_RCM_MSS_ELM_CLK_DIV_VAL_CLKDIVR_MAX                       (0x00000FFFU)

#define SDL_MSS_RCM_MSS_ELM_CLK_DIV_VAL_RESETVAL                          (0x00000333U)

/* MCAN0_CLK_GATE */

#define SDL_MSS_RCM_MCAN0_CLK_GATE_GATED_MASK                             (0x00000007U)
#define SDL_MSS_RCM_MCAN0_CLK_GATE_GATED_SHIFT                            (0x00000000U)
#define SDL_MSS_RCM_MCAN0_CLK_GATE_GATED_RESETVAL                         (0x00000000U)
#define SDL_MSS_RCM_MCAN0_CLK_GATE_GATED_MAX                              (0x00000007U)

#define SDL_MSS_RCM_MCAN0_CLK_GATE_RESETVAL                               (0x00000000U)

/* MCAN1_CLK_GATE */

#define SDL_MSS_RCM_MCAN1_CLK_GATE_GATED_MASK                             (0x00000007U)
#define SDL_MSS_RCM_MCAN1_CLK_GATE_GATED_SHIFT                            (0x00000000U)
#define SDL_MSS_RCM_MCAN1_CLK_GATE_GATED_RESETVAL                         (0x00000000U)
#define SDL_MSS_RCM_MCAN1_CLK_GATE_GATED_MAX                              (0x00000007U)

#define SDL_MSS_RCM_MCAN1_CLK_GATE_RESETVAL                               (0x00000000U)

/* MCAN2_CLK_GATE */

#define SDL_MSS_RCM_MCAN2_CLK_GATE_GATED_MASK                             (0x00000007U)
#define SDL_MSS_RCM_MCAN2_CLK_GATE_GATED_SHIFT                            (0x00000000U)
#define SDL_MSS_RCM_MCAN2_CLK_GATE_GATED_RESETVAL                         (0x00000000U)
#define SDL_MSS_RCM_MCAN2_CLK_GATE_GATED_MAX                              (0x00000007U)

#define SDL_MSS_RCM_MCAN2_CLK_GATE_RESETVAL                               (0x00000000U)

/* MCAN3_CLK_GATE */

#define SDL_MSS_RCM_MCAN3_CLK_GATE_GATED_MASK                             (0x00000007U)
#define SDL_MSS_RCM_MCAN3_CLK_GATE_GATED_SHIFT                            (0x00000000U)
#define SDL_MSS_RCM_MCAN3_CLK_GATE_GATED_RESETVAL                         (0x00000000U)
#define SDL_MSS_RCM_MCAN3_CLK_GATE_GATED_MAX                              (0x00000007U)

#define SDL_MSS_RCM_MCAN3_CLK_GATE_RESETVAL                               (0x00000000U)

/* QSPI0_CLK_GATE */

#define SDL_MSS_RCM_QSPI0_CLK_GATE_GATED_MASK                             (0x00000007U)
#define SDL_MSS_RCM_QSPI0_CLK_GATE_GATED_SHIFT                            (0x00000000U)
#define SDL_MSS_RCM_QSPI0_CLK_GATE_GATED_RESETVAL                         (0x00000000U)
#define SDL_MSS_RCM_QSPI0_CLK_GATE_GATED_MAX                              (0x00000007U)

#define SDL_MSS_RCM_QSPI0_CLK_GATE_RESETVAL                               (0x00000000U)

/* RTI0_CLK_GATE */

#define SDL_MSS_RCM_RTI0_CLK_GATE_GATED_MASK                              (0x00000007U)
#define SDL_MSS_RCM_RTI0_CLK_GATE_GATED_SHIFT                             (0x00000000U)
#define SDL_MSS_RCM_RTI0_CLK_GATE_GATED_RESETVAL                          (0x00000000U)
#define SDL_MSS_RCM_RTI0_CLK_GATE_GATED_MAX                               (0x00000007U)

#define SDL_MSS_RCM_RTI0_CLK_GATE_RESETVAL                                (0x00000000U)

/* RTI1_CLK_GATE */

#define SDL_MSS_RCM_RTI1_CLK_GATE_GATED_MASK                              (0x00000007U)
#define SDL_MSS_RCM_RTI1_CLK_GATE_GATED_SHIFT                             (0x00000000U)
#define SDL_MSS_RCM_RTI1_CLK_GATE_GATED_RESETVAL                          (0x00000000U)
#define SDL_MSS_RCM_RTI1_CLK_GATE_GATED_MAX                               (0x00000007U)

#define SDL_MSS_RCM_RTI1_CLK_GATE_RESETVAL                                (0x00000000U)

/* RTI2_CLK_GATE */

#define SDL_MSS_RCM_RTI2_CLK_GATE_GATED_MASK                              (0x00000007U)
#define SDL_MSS_RCM_RTI2_CLK_GATE_GATED_SHIFT                             (0x00000000U)
#define SDL_MSS_RCM_RTI2_CLK_GATE_GATED_RESETVAL                          (0x00000000U)
#define SDL_MSS_RCM_RTI2_CLK_GATE_GATED_MAX                               (0x00000007U)

#define SDL_MSS_RCM_RTI2_CLK_GATE_RESETVAL                                (0x00000000U)

/* RTI3_CLK_GATE */

#define SDL_MSS_RCM_RTI3_CLK_GATE_GATED_MASK                              (0x00000007U)
#define SDL_MSS_RCM_RTI3_CLK_GATE_GATED_SHIFT                             (0x00000000U)
#define SDL_MSS_RCM_RTI3_CLK_GATE_GATED_RESETVAL                          (0x00000000U)
#define SDL_MSS_RCM_RTI3_CLK_GATE_GATED_MAX                               (0x00000007U)

#define SDL_MSS_RCM_RTI3_CLK_GATE_RESETVAL                                (0x00000000U)

/* WDT0_CLK_GATE */

#define SDL_MSS_RCM_WDT0_CLK_GATE_GATED_MASK                              (0x00000007U)
#define SDL_MSS_RCM_WDT0_CLK_GATE_GATED_SHIFT                             (0x00000000U)
#define SDL_MSS_RCM_WDT0_CLK_GATE_GATED_RESETVAL                          (0x00000000U)
#define SDL_MSS_RCM_WDT0_CLK_GATE_GATED_MAX                               (0x00000007U)

#define SDL_MSS_RCM_WDT0_CLK_GATE_RESETVAL                                (0x00000000U)

/* WDT1_CLK_GATE */

#define SDL_MSS_RCM_WDT1_CLK_GATE_GATED_MASK                              (0x00000007U)
#define SDL_MSS_RCM_WDT1_CLK_GATE_GATED_SHIFT                             (0x00000000U)
#define SDL_MSS_RCM_WDT1_CLK_GATE_GATED_RESETVAL                          (0x00000000U)
#define SDL_MSS_RCM_WDT1_CLK_GATE_GATED_MAX                               (0x00000007U)

#define SDL_MSS_RCM_WDT1_CLK_GATE_RESETVAL                                (0x00000000U)

/* WDT2_CLK_GATE */

#define SDL_MSS_RCM_WDT2_CLK_GATE_GATED_MASK                              (0x00000007U)
#define SDL_MSS_RCM_WDT2_CLK_GATE_GATED_SHIFT                             (0x00000000U)
#define SDL_MSS_RCM_WDT2_CLK_GATE_GATED_RESETVAL                          (0x00000000U)
#define SDL_MSS_RCM_WDT2_CLK_GATE_GATED_MAX                               (0x00000007U)

#define SDL_MSS_RCM_WDT2_CLK_GATE_RESETVAL                                (0x00000000U)

/* WDT3_CLK_GATE */

#define SDL_MSS_RCM_WDT3_CLK_GATE_GATED_MASK                              (0x00000007U)
#define SDL_MSS_RCM_WDT3_CLK_GATE_GATED_SHIFT                             (0x00000000U)
#define SDL_MSS_RCM_WDT3_CLK_GATE_GATED_RESETVAL                          (0x00000000U)
#define SDL_MSS_RCM_WDT3_CLK_GATE_GATED_MAX                               (0x00000007U)

#define SDL_MSS_RCM_WDT3_CLK_GATE_RESETVAL                                (0x00000000U)

/* MCSPI0_CLK_GATE */

#define SDL_MSS_RCM_MCSPI0_CLK_GATE_GATED_MASK                            (0x00000007U)
#define SDL_MSS_RCM_MCSPI0_CLK_GATE_GATED_SHIFT                           (0x00000000U)
#define SDL_MSS_RCM_MCSPI0_CLK_GATE_GATED_RESETVAL                        (0x00000000U)
#define SDL_MSS_RCM_MCSPI0_CLK_GATE_GATED_MAX                             (0x00000007U)

#define SDL_MSS_RCM_MCSPI0_CLK_GATE_RESETVAL                              (0x00000000U)

/* MCSPI1_CLK_GATE */

#define SDL_MSS_RCM_MCSPI1_CLK_GATE_GATED_MASK                            (0x00000007U)
#define SDL_MSS_RCM_MCSPI1_CLK_GATE_GATED_SHIFT                           (0x00000000U)
#define SDL_MSS_RCM_MCSPI1_CLK_GATE_GATED_RESETVAL                        (0x00000000U)
#define SDL_MSS_RCM_MCSPI1_CLK_GATE_GATED_MAX                             (0x00000007U)

#define SDL_MSS_RCM_MCSPI1_CLK_GATE_RESETVAL                              (0x00000000U)

/* MCSPI2_CLK_GATE */

#define SDL_MSS_RCM_MCSPI2_CLK_GATE_GATED_MASK                            (0x00000007U)
#define SDL_MSS_RCM_MCSPI2_CLK_GATE_GATED_SHIFT                           (0x00000000U)
#define SDL_MSS_RCM_MCSPI2_CLK_GATE_GATED_RESETVAL                        (0x00000000U)
#define SDL_MSS_RCM_MCSPI2_CLK_GATE_GATED_MAX                             (0x00000007U)

#define SDL_MSS_RCM_MCSPI2_CLK_GATE_RESETVAL                              (0x00000000U)

/* MCSPI3_CLK_GATE */

#define SDL_MSS_RCM_MCSPI3_CLK_GATE_GATED_MASK                            (0x00000007U)
#define SDL_MSS_RCM_MCSPI3_CLK_GATE_GATED_SHIFT                           (0x00000000U)
#define SDL_MSS_RCM_MCSPI3_CLK_GATE_GATED_RESETVAL                        (0x00000000U)
#define SDL_MSS_RCM_MCSPI3_CLK_GATE_GATED_MAX                             (0x00000007U)

#define SDL_MSS_RCM_MCSPI3_CLK_GATE_RESETVAL                              (0x00000000U)

/* MCSPI4_CLK_GATE */

#define SDL_MSS_RCM_MCSPI4_CLK_GATE_GATED_MASK                            (0x00000007U)
#define SDL_MSS_RCM_MCSPI4_CLK_GATE_GATED_SHIFT                           (0x00000000U)
#define SDL_MSS_RCM_MCSPI4_CLK_GATE_GATED_RESETVAL                        (0x00000000U)
#define SDL_MSS_RCM_MCSPI4_CLK_GATE_GATED_MAX                             (0x00000007U)

#define SDL_MSS_RCM_MCSPI4_CLK_GATE_RESETVAL                              (0x00000000U)

/* MMC0_CLK_GATE */

#define SDL_MSS_RCM_MMC0_CLK_GATE_GATED_MASK                              (0x00000007U)
#define SDL_MSS_RCM_MMC0_CLK_GATE_GATED_SHIFT                             (0x00000000U)
#define SDL_MSS_RCM_MMC0_CLK_GATE_GATED_RESETVAL                          (0x00000000U)
#define SDL_MSS_RCM_MMC0_CLK_GATE_GATED_MAX                               (0x00000007U)

#define SDL_MSS_RCM_MMC0_CLK_GATE_RESETVAL                                (0x00000000U)

/* ICSSM0_UART_CLK_GATE */

#define SDL_MSS_RCM_ICSSM0_UART_CLK_GATE_GATED_MASK                       (0x00000007U)
#define SDL_MSS_RCM_ICSSM0_UART_CLK_GATE_GATED_SHIFT                      (0x00000000U)
#define SDL_MSS_RCM_ICSSM0_UART_CLK_GATE_GATED_RESETVAL                   (0x00000000U)
#define SDL_MSS_RCM_ICSSM0_UART_CLK_GATE_GATED_MAX                        (0x00000007U)

#define SDL_MSS_RCM_ICSSM0_UART_CLK_GATE_RESETVAL                         (0x00000000U)

/* CPTS_CLK_GATE */

#define SDL_MSS_RCM_CPTS_CLK_GATE_GATED_MASK                              (0x00000007U)
#define SDL_MSS_RCM_CPTS_CLK_GATE_GATED_SHIFT                             (0x00000000U)
#define SDL_MSS_RCM_CPTS_CLK_GATE_GATED_RESETVAL                          (0x00000000U)
#define SDL_MSS_RCM_CPTS_CLK_GATE_GATED_MAX                               (0x00000007U)

#define SDL_MSS_RCM_CPTS_CLK_GATE_RESETVAL                                (0x00000000U)

/* GPMC_CLK_GATE */

#define SDL_MSS_RCM_GPMC_CLK_GATE_GATED_MASK                              (0x00000007U)
#define SDL_MSS_RCM_GPMC_CLK_GATE_GATED_SHIFT                             (0x00000000U)
#define SDL_MSS_RCM_GPMC_CLK_GATE_GATED_RESETVAL                          (0x00000000U)
#define SDL_MSS_RCM_GPMC_CLK_GATE_GATED_MAX                               (0x00000007U)

#define SDL_MSS_RCM_GPMC_CLK_GATE_RESETVAL                                (0x00000000U)

/* CONTROLSS_PLL_CLK_GATE */

#define SDL_MSS_RCM_CONTROLSS_PLL_CLK_GATE_GATED_MASK                     (0x00000007U)
#define SDL_MSS_RCM_CONTROLSS_PLL_CLK_GATE_GATED_SHIFT                    (0x00000000U)
#define SDL_MSS_RCM_CONTROLSS_PLL_CLK_GATE_GATED_RESETVAL                 (0x00000000U)
#define SDL_MSS_RCM_CONTROLSS_PLL_CLK_GATE_GATED_MAX                      (0x00000007U)

#define SDL_MSS_RCM_CONTROLSS_PLL_CLK_GATE_RESETVAL                       (0x00000000U)

/* I2C0_CLK_GATE */

#define SDL_MSS_RCM_I2C0_CLK_GATE_GATED_MASK                              (0x00000007U)
#define SDL_MSS_RCM_I2C0_CLK_GATE_GATED_SHIFT                             (0x00000000U)
#define SDL_MSS_RCM_I2C0_CLK_GATE_GATED_RESETVAL                          (0x00000000U)
#define SDL_MSS_RCM_I2C0_CLK_GATE_GATED_MAX                               (0x00000007U)

#define SDL_MSS_RCM_I2C0_CLK_GATE_RESETVAL                                (0x00000000U)

/* I2C1_CLK_GATE */

#define SDL_MSS_RCM_I2C1_CLK_GATE_GATED_MASK                              (0x00000007U)
#define SDL_MSS_RCM_I2C1_CLK_GATE_GATED_SHIFT                             (0x00000000U)
#define SDL_MSS_RCM_I2C1_CLK_GATE_GATED_RESETVAL                          (0x00000000U)
#define SDL_MSS_RCM_I2C1_CLK_GATE_GATED_MAX                               (0x00000007U)

#define SDL_MSS_RCM_I2C1_CLK_GATE_RESETVAL                                (0x00000000U)

/* I2C2_CLK_GATE */

#define SDL_MSS_RCM_I2C2_CLK_GATE_GATED_MASK                              (0x00000007U)
#define SDL_MSS_RCM_I2C2_CLK_GATE_GATED_SHIFT                             (0x00000000U)
#define SDL_MSS_RCM_I2C2_CLK_GATE_GATED_RESETVAL                          (0x00000000U)
#define SDL_MSS_RCM_I2C2_CLK_GATE_GATED_MAX                               (0x00000007U)

#define SDL_MSS_RCM_I2C2_CLK_GATE_RESETVAL                                (0x00000000U)

/* I2C3_CLK_GATE */

#define SDL_MSS_RCM_I2C3_CLK_GATE_GATED_MASK                              (0x00000007U)
#define SDL_MSS_RCM_I2C3_CLK_GATE_GATED_SHIFT                             (0x00000000U)
#define SDL_MSS_RCM_I2C3_CLK_GATE_GATED_RESETVAL                          (0x00000000U)
#define SDL_MSS_RCM_I2C3_CLK_GATE_GATED_MAX                               (0x00000007U)

#define SDL_MSS_RCM_I2C3_CLK_GATE_RESETVAL                                (0x00000000U)

/* LIN0_CLK_GATE */

#define SDL_MSS_RCM_LIN0_CLK_GATE_GATED_MASK                              (0x00000007U)
#define SDL_MSS_RCM_LIN0_CLK_GATE_GATED_SHIFT                             (0x00000000U)
#define SDL_MSS_RCM_LIN0_CLK_GATE_GATED_RESETVAL                          (0x00000000U)
#define SDL_MSS_RCM_LIN0_CLK_GATE_GATED_MAX                               (0x00000007U)

#define SDL_MSS_RCM_LIN0_CLK_GATE_RESETVAL                                (0x00000000U)

/* LIN1_CLK_GATE */

#define SDL_MSS_RCM_LIN1_CLK_GATE_GATED_MASK                              (0x00000007U)
#define SDL_MSS_RCM_LIN1_CLK_GATE_GATED_SHIFT                             (0x00000000U)
#define SDL_MSS_RCM_LIN1_CLK_GATE_GATED_RESETVAL                          (0x00000000U)
#define SDL_MSS_RCM_LIN1_CLK_GATE_GATED_MAX                               (0x00000007U)

#define SDL_MSS_RCM_LIN1_CLK_GATE_RESETVAL                                (0x00000000U)

/* LIN2_CLK_GATE */

#define SDL_MSS_RCM_LIN2_CLK_GATE_GATED_MASK                              (0x00000007U)
#define SDL_MSS_RCM_LIN2_CLK_GATE_GATED_SHIFT                             (0x00000000U)
#define SDL_MSS_RCM_LIN2_CLK_GATE_GATED_RESETVAL                          (0x00000000U)
#define SDL_MSS_RCM_LIN2_CLK_GATE_GATED_MAX                               (0x00000007U)

#define SDL_MSS_RCM_LIN2_CLK_GATE_RESETVAL                                (0x00000000U)

/* LIN3_CLK_GATE */

#define SDL_MSS_RCM_LIN3_CLK_GATE_GATED_MASK                              (0x00000007U)
#define SDL_MSS_RCM_LIN3_CLK_GATE_GATED_SHIFT                             (0x00000000U)
#define SDL_MSS_RCM_LIN3_CLK_GATE_GATED_RESETVAL                          (0x00000000U)
#define SDL_MSS_RCM_LIN3_CLK_GATE_GATED_MAX                               (0x00000007U)

#define SDL_MSS_RCM_LIN3_CLK_GATE_RESETVAL                                (0x00000000U)

/* LIN4_CLK_GATE */

#define SDL_MSS_RCM_LIN4_CLK_GATE_GATED_MASK                              (0x00000007U)
#define SDL_MSS_RCM_LIN4_CLK_GATE_GATED_SHIFT                             (0x00000000U)
#define SDL_MSS_RCM_LIN4_CLK_GATE_GATED_RESETVAL                          (0x00000000U)
#define SDL_MSS_RCM_LIN4_CLK_GATE_GATED_MAX                               (0x00000007U)

#define SDL_MSS_RCM_LIN4_CLK_GATE_RESETVAL                                (0x00000000U)

/* UART0_CLK_GATE */

#define SDL_MSS_RCM_UART0_CLK_GATE_GATED_MASK                             (0x00000007U)
#define SDL_MSS_RCM_UART0_CLK_GATE_GATED_SHIFT                            (0x00000000U)
#define SDL_MSS_RCM_UART0_CLK_GATE_GATED_RESETVAL                         (0x00000000U)
#define SDL_MSS_RCM_UART0_CLK_GATE_GATED_MAX                              (0x00000007U)

#define SDL_MSS_RCM_UART0_CLK_GATE_RESETVAL                               (0x00000000U)

/* UART1_CLK_GATE */

#define SDL_MSS_RCM_UART1_CLK_GATE_GATED_MASK                             (0x00000007U)
#define SDL_MSS_RCM_UART1_CLK_GATE_GATED_SHIFT                            (0x00000000U)
#define SDL_MSS_RCM_UART1_CLK_GATE_GATED_RESETVAL                         (0x00000000U)
#define SDL_MSS_RCM_UART1_CLK_GATE_GATED_MAX                              (0x00000007U)

#define SDL_MSS_RCM_UART1_CLK_GATE_RESETVAL                               (0x00000000U)

/* UART2_CLK_GATE */

#define SDL_MSS_RCM_UART2_CLK_GATE_GATED_MASK                             (0x00000007U)
#define SDL_MSS_RCM_UART2_CLK_GATE_GATED_SHIFT                            (0x00000000U)
#define SDL_MSS_RCM_UART2_CLK_GATE_GATED_RESETVAL                         (0x00000000U)
#define SDL_MSS_RCM_UART2_CLK_GATE_GATED_MAX                              (0x00000007U)

#define SDL_MSS_RCM_UART2_CLK_GATE_RESETVAL                               (0x00000000U)

/* UART3_CLK_GATE */

#define SDL_MSS_RCM_UART3_CLK_GATE_GATED_MASK                             (0x00000007U)
#define SDL_MSS_RCM_UART3_CLK_GATE_GATED_SHIFT                            (0x00000000U)
#define SDL_MSS_RCM_UART3_CLK_GATE_GATED_RESETVAL                         (0x00000000U)
#define SDL_MSS_RCM_UART3_CLK_GATE_GATED_MAX                              (0x00000007U)

#define SDL_MSS_RCM_UART3_CLK_GATE_RESETVAL                               (0x00000000U)

/* UART4_CLK_GATE */

#define SDL_MSS_RCM_UART4_CLK_GATE_GATED_MASK                             (0x00000007U)
#define SDL_MSS_RCM_UART4_CLK_GATE_GATED_SHIFT                            (0x00000000U)
#define SDL_MSS_RCM_UART4_CLK_GATE_GATED_RESETVAL                         (0x00000000U)
#define SDL_MSS_RCM_UART4_CLK_GATE_GATED_MAX                              (0x00000007U)

#define SDL_MSS_RCM_UART4_CLK_GATE_RESETVAL                               (0x00000000U)

/* UART5_CLK_GATE */

#define SDL_MSS_RCM_UART5_CLK_GATE_GATED_MASK                             (0x00000007U)
#define SDL_MSS_RCM_UART5_CLK_GATE_GATED_SHIFT                            (0x00000000U)
#define SDL_MSS_RCM_UART5_CLK_GATE_GATED_RESETVAL                         (0x00000000U)
#define SDL_MSS_RCM_UART5_CLK_GATE_GATED_MAX                              (0x00000007U)

#define SDL_MSS_RCM_UART5_CLK_GATE_RESETVAL                               (0x00000000U)

/* RGMII_250_CLK_GATE */

#define SDL_MSS_RCM_RGMII_250_CLK_GATE_GATED_MASK                         (0x00000007U)
#define SDL_MSS_RCM_RGMII_250_CLK_GATE_GATED_SHIFT                        (0x00000000U)
#define SDL_MSS_RCM_RGMII_250_CLK_GATE_GATED_RESETVAL                     (0x00000000U)
#define SDL_MSS_RCM_RGMII_250_CLK_GATE_GATED_MAX                          (0x00000007U)

#define SDL_MSS_RCM_RGMII_250_CLK_GATE_RESETVAL                           (0x00000000U)

/* RGMII_50_CLK_GATE */

#define SDL_MSS_RCM_RGMII_50_CLK_GATE_GATED_MASK                          (0x00000007U)
#define SDL_MSS_RCM_RGMII_50_CLK_GATE_GATED_SHIFT                         (0x00000000U)
#define SDL_MSS_RCM_RGMII_50_CLK_GATE_GATED_RESETVAL                      (0x00000000U)
#define SDL_MSS_RCM_RGMII_50_CLK_GATE_GATED_MAX                           (0x00000007U)

#define SDL_MSS_RCM_RGMII_50_CLK_GATE_RESETVAL                            (0x00000000U)

/* RGMII_5_CLK_GATE */

#define SDL_MSS_RCM_RGMII_5_CLK_GATE_GATED_MASK                           (0x00000007U)
#define SDL_MSS_RCM_RGMII_5_CLK_GATE_GATED_SHIFT                          (0x00000000U)
#define SDL_MSS_RCM_RGMII_5_CLK_GATE_GATED_RESETVAL                       (0x00000000U)
#define SDL_MSS_RCM_RGMII_5_CLK_GATE_GATED_MAX                            (0x00000007U)

#define SDL_MSS_RCM_RGMII_5_CLK_GATE_RESETVAL                             (0x00000000U)

/* MMC0_32K_CLK_GATE */

#define SDL_MSS_RCM_MMC0_32K_CLK_GATE_GATED_MASK                          (0x00000007U)
#define SDL_MSS_RCM_MMC0_32K_CLK_GATE_GATED_SHIFT                         (0x00000000U)
#define SDL_MSS_RCM_MMC0_32K_CLK_GATE_GATED_RESETVAL                      (0x00000000U)
#define SDL_MSS_RCM_MMC0_32K_CLK_GATE_GATED_MAX                           (0x00000007U)

#define SDL_MSS_RCM_MMC0_32K_CLK_GATE_RESETVAL                            (0x00000000U)

/* TEMPSENSE_32K_CLK_GATE */

#define SDL_MSS_RCM_TEMPSENSE_32K_CLK_GATE_GATED_MASK                     (0x00000007U)
#define SDL_MSS_RCM_TEMPSENSE_32K_CLK_GATE_GATED_SHIFT                    (0x00000000U)
#define SDL_MSS_RCM_TEMPSENSE_32K_CLK_GATE_GATED_RESETVAL                 (0x00000000U)
#define SDL_MSS_RCM_TEMPSENSE_32K_CLK_GATE_GATED_MAX                      (0x00000007U)

#define SDL_MSS_RCM_TEMPSENSE_32K_CLK_GATE_RESETVAL                       (0x00000000U)

/* CPSW_CLK_GATE */

#define SDL_MSS_RCM_CPSW_CLK_GATE_GATED_MASK                              (0x00000007U)
#define SDL_MSS_RCM_CPSW_CLK_GATE_GATED_SHIFT                             (0x00000000U)
#define SDL_MSS_RCM_CPSW_CLK_GATE_GATED_RESETVAL                          (0x00000000U)
#define SDL_MSS_RCM_CPSW_CLK_GATE_GATED_MAX                               (0x00000007U)

#define SDL_MSS_RCM_CPSW_CLK_GATE_RESETVAL                                (0x00000000U)

/* ICSSM0_IEP_CLK_GATE */

#define SDL_MSS_RCM_ICSSM0_IEP_CLK_GATE_GATED_MASK                        (0x00000007U)
#define SDL_MSS_RCM_ICSSM0_IEP_CLK_GATE_GATED_SHIFT                       (0x00000000U)
#define SDL_MSS_RCM_ICSSM0_IEP_CLK_GATE_GATED_RESETVAL                    (0x00000000U)
#define SDL_MSS_RCM_ICSSM0_IEP_CLK_GATE_GATED_MAX                         (0x00000007U)

#define SDL_MSS_RCM_ICSSM0_IEP_CLK_GATE_RESETVAL                          (0x00000000U)

/* ICSSM0_CORE_CLK_GATE */

#define SDL_MSS_RCM_ICSSM0_CORE_CLK_GATE_GATED_MASK                       (0x00000007U)
#define SDL_MSS_RCM_ICSSM0_CORE_CLK_GATE_GATED_SHIFT                      (0x00000000U)
#define SDL_MSS_RCM_ICSSM0_CORE_CLK_GATE_GATED_RESETVAL                   (0x00000000U)
#define SDL_MSS_RCM_ICSSM0_CORE_CLK_GATE_GATED_MAX                        (0x00000007U)

#define SDL_MSS_RCM_ICSSM0_CORE_CLK_GATE_RESETVAL                         (0x00000000U)

/* MSS_ICSSM_SYS_CLK_GATE */

#define SDL_MSS_RCM_MSS_ICSSM_SYS_CLK_GATE_GATED_MASK                     (0x00000007U)
#define SDL_MSS_RCM_MSS_ICSSM_SYS_CLK_GATE_GATED_SHIFT                    (0x00000000U)
#define SDL_MSS_RCM_MSS_ICSSM_SYS_CLK_GATE_GATED_RESETVAL                 (0x00000000U)
#define SDL_MSS_RCM_MSS_ICSSM_SYS_CLK_GATE_GATED_MAX                      (0x00000007U)

#define SDL_MSS_RCM_MSS_ICSSM_SYS_CLK_GATE_RESETVAL                       (0x00000000U)

/* MSS_ELM_CLK_GATE */

#define SDL_MSS_RCM_MSS_ELM_CLK_GATE_GATED_MASK                           (0x00000007U)
#define SDL_MSS_RCM_MSS_ELM_CLK_GATE_GATED_SHIFT                          (0x00000000U)
#define SDL_MSS_RCM_MSS_ELM_CLK_GATE_GATED_RESETVAL                       (0x00000000U)
#define SDL_MSS_RCM_MSS_ELM_CLK_GATE_GATED_MAX                            (0x00000007U)

#define SDL_MSS_RCM_MSS_ELM_CLK_GATE_RESETVAL                             (0x00000000U)

/* R5SS0_CORE0_GATE */

#define SDL_MSS_RCM_R5SS0_CORE0_GATE_CLKGATE_MASK                         (0x00000007U)
#define SDL_MSS_RCM_R5SS0_CORE0_GATE_CLKGATE_SHIFT                        (0x00000000U)
#define SDL_MSS_RCM_R5SS0_CORE0_GATE_CLKGATE_RESETVAL                     (0x00000000U)
#define SDL_MSS_RCM_R5SS0_CORE0_GATE_CLKGATE_MAX                          (0x00000007U)

#define SDL_MSS_RCM_R5SS0_CORE0_GATE_RESETVAL                             (0x00000000U)

/* R5SS1_CORE0_GATE */

#define SDL_MSS_RCM_R5SS1_CORE0_GATE_CLKGATE_MASK                         (0x00000007U)
#define SDL_MSS_RCM_R5SS1_CORE0_GATE_CLKGATE_SHIFT                        (0x00000000U)
#define SDL_MSS_RCM_R5SS1_CORE0_GATE_CLKGATE_RESETVAL                     (0x00000000U)
#define SDL_MSS_RCM_R5SS1_CORE0_GATE_CLKGATE_MAX                          (0x00000007U)

#define SDL_MSS_RCM_R5SS1_CORE0_GATE_RESETVAL                             (0x00000000U)

/* R5SS0_CORE1_GATE */

#define SDL_MSS_RCM_R5SS0_CORE1_GATE_CLKGATE_MASK                         (0x00000007U)
#define SDL_MSS_RCM_R5SS0_CORE1_GATE_CLKGATE_SHIFT                        (0x00000000U)
#define SDL_MSS_RCM_R5SS0_CORE1_GATE_CLKGATE_RESETVAL                     (0x00000000U)
#define SDL_MSS_RCM_R5SS0_CORE1_GATE_CLKGATE_MAX                          (0x00000007U)

#define SDL_MSS_RCM_R5SS0_CORE1_GATE_RESETVAL                             (0x00000000U)

/* R5SS1_CORE1_GATE */

#define SDL_MSS_RCM_R5SS1_CORE1_GATE_CLKGATE_MASK                         (0x00000007U)
#define SDL_MSS_RCM_R5SS1_CORE1_GATE_CLKGATE_SHIFT                        (0x00000000U)
#define SDL_MSS_RCM_R5SS1_CORE1_GATE_CLKGATE_RESETVAL                     (0x00000000U)
#define SDL_MSS_RCM_R5SS1_CORE1_GATE_CLKGATE_MAX                          (0x00000007U)

#define SDL_MSS_RCM_R5SS1_CORE1_GATE_RESETVAL                             (0x00000000U)

/* MCAN0_CLK_STATUS */

#define SDL_MSS_RCM_MCAN0_CLK_STATUS_CLKINUSE_MASK                        (0x000000FFU)
#define SDL_MSS_RCM_MCAN0_CLK_STATUS_CLKINUSE_SHIFT                       (0x00000000U)
#define SDL_MSS_RCM_MCAN0_CLK_STATUS_CLKINUSE_RESETVAL                    (0x00000001U)
#define SDL_MSS_RCM_MCAN0_CLK_STATUS_CLKINUSE_MAX                         (0x000000FFU)

#define SDL_MSS_RCM_MCAN0_CLK_STATUS_CURRDIVIDER_MASK                     (0x0000FF00U)
#define SDL_MSS_RCM_MCAN0_CLK_STATUS_CURRDIVIDER_SHIFT                    (0x00000008U)
#define SDL_MSS_RCM_MCAN0_CLK_STATUS_CURRDIVIDER_RESETVAL                 (0x00000000U)
#define SDL_MSS_RCM_MCAN0_CLK_STATUS_CURRDIVIDER_MAX                      (0x000000FFU)

#define SDL_MSS_RCM_MCAN0_CLK_STATUS_RESETVAL                             (0x00000001U)

/* MCAN1_CLK_STATUS */

#define SDL_MSS_RCM_MCAN1_CLK_STATUS_CLKINUSE_MASK                        (0x000000FFU)
#define SDL_MSS_RCM_MCAN1_CLK_STATUS_CLKINUSE_SHIFT                       (0x00000000U)
#define SDL_MSS_RCM_MCAN1_CLK_STATUS_CLKINUSE_RESETVAL                    (0x00000001U)
#define SDL_MSS_RCM_MCAN1_CLK_STATUS_CLKINUSE_MAX                         (0x000000FFU)

#define SDL_MSS_RCM_MCAN1_CLK_STATUS_CURRDIVIDER_MASK                     (0x0000FF00U)
#define SDL_MSS_RCM_MCAN1_CLK_STATUS_CURRDIVIDER_SHIFT                    (0x00000008U)
#define SDL_MSS_RCM_MCAN1_CLK_STATUS_CURRDIVIDER_RESETVAL                 (0x00000000U)
#define SDL_MSS_RCM_MCAN1_CLK_STATUS_CURRDIVIDER_MAX                      (0x000000FFU)

#define SDL_MSS_RCM_MCAN1_CLK_STATUS_RESETVAL                             (0x00000001U)

/* MCAN2_CLK_STATUS */

#define SDL_MSS_RCM_MCAN2_CLK_STATUS_CLKINUSE_MASK                        (0x000000FFU)
#define SDL_MSS_RCM_MCAN2_CLK_STATUS_CLKINUSE_SHIFT                       (0x00000000U)
#define SDL_MSS_RCM_MCAN2_CLK_STATUS_CLKINUSE_RESETVAL                    (0x00000001U)
#define SDL_MSS_RCM_MCAN2_CLK_STATUS_CLKINUSE_MAX                         (0x000000FFU)

#define SDL_MSS_RCM_MCAN2_CLK_STATUS_CURRDIVIDER_MASK                     (0x0000FF00U)
#define SDL_MSS_RCM_MCAN2_CLK_STATUS_CURRDIVIDER_SHIFT                    (0x00000008U)
#define SDL_MSS_RCM_MCAN2_CLK_STATUS_CURRDIVIDER_RESETVAL                 (0x00000000U)
#define SDL_MSS_RCM_MCAN2_CLK_STATUS_CURRDIVIDER_MAX                      (0x000000FFU)

#define SDL_MSS_RCM_MCAN2_CLK_STATUS_RESETVAL                             (0x00000001U)

/* MCAN3_CLK_STATUS */

#define SDL_MSS_RCM_MCAN3_CLK_STATUS_CLKINUSE_MASK                        (0x000000FFU)
#define SDL_MSS_RCM_MCAN3_CLK_STATUS_CLKINUSE_SHIFT                       (0x00000000U)
#define SDL_MSS_RCM_MCAN3_CLK_STATUS_CLKINUSE_RESETVAL                    (0x00000001U)
#define SDL_MSS_RCM_MCAN3_CLK_STATUS_CLKINUSE_MAX                         (0x000000FFU)

#define SDL_MSS_RCM_MCAN3_CLK_STATUS_CURRDIVIDER_MASK                     (0x0000FF00U)
#define SDL_MSS_RCM_MCAN3_CLK_STATUS_CURRDIVIDER_SHIFT                    (0x00000008U)
#define SDL_MSS_RCM_MCAN3_CLK_STATUS_CURRDIVIDER_RESETVAL                 (0x00000000U)
#define SDL_MSS_RCM_MCAN3_CLK_STATUS_CURRDIVIDER_MAX                      (0x000000FFU)

#define SDL_MSS_RCM_MCAN3_CLK_STATUS_RESETVAL                             (0x00000001U)

/* QSPI0_CLK_STATUS */

#define SDL_MSS_RCM_QSPI0_CLK_STATUS_CLKINUSE_MASK                        (0x000000FFU)
#define SDL_MSS_RCM_QSPI0_CLK_STATUS_CLKINUSE_SHIFT                       (0x00000000U)
#define SDL_MSS_RCM_QSPI0_CLK_STATUS_CLKINUSE_RESETVAL                    (0x00000001U)
#define SDL_MSS_RCM_QSPI0_CLK_STATUS_CLKINUSE_MAX                         (0x000000FFU)

#define SDL_MSS_RCM_QSPI0_CLK_STATUS_CURRDIVIDER_MASK                     (0x0000FF00U)
#define SDL_MSS_RCM_QSPI0_CLK_STATUS_CURRDIVIDER_SHIFT                    (0x00000008U)
#define SDL_MSS_RCM_QSPI0_CLK_STATUS_CURRDIVIDER_RESETVAL                 (0x00000000U)
#define SDL_MSS_RCM_QSPI0_CLK_STATUS_CURRDIVIDER_MAX                      (0x000000FFU)

#define SDL_MSS_RCM_QSPI0_CLK_STATUS_RESETVAL                             (0x00000001U)

/* RTI0_CLK_STATUS */

#define SDL_MSS_RCM_RTI0_CLK_STATUS_CLKINUSE_MASK                         (0x000000FFU)
#define SDL_MSS_RCM_RTI0_CLK_STATUS_CLKINUSE_SHIFT                        (0x00000000U)
#define SDL_MSS_RCM_RTI0_CLK_STATUS_CLKINUSE_RESETVAL                     (0x00000001U)
#define SDL_MSS_RCM_RTI0_CLK_STATUS_CLKINUSE_MAX                          (0x000000FFU)

#define SDL_MSS_RCM_RTI0_CLK_STATUS_CURRDIVIDER_MASK                      (0x0000FF00U)
#define SDL_MSS_RCM_RTI0_CLK_STATUS_CURRDIVIDER_SHIFT                     (0x00000008U)
#define SDL_MSS_RCM_RTI0_CLK_STATUS_CURRDIVIDER_RESETVAL                  (0x00000000U)
#define SDL_MSS_RCM_RTI0_CLK_STATUS_CURRDIVIDER_MAX                       (0x000000FFU)

#define SDL_MSS_RCM_RTI0_CLK_STATUS_RESETVAL                              (0x00000001U)

/* RTI1_CLK_STATUS */

#define SDL_MSS_RCM_RTI1_CLK_STATUS_CLKINUSE_MASK                         (0x000000FFU)
#define SDL_MSS_RCM_RTI1_CLK_STATUS_CLKINUSE_SHIFT                        (0x00000000U)
#define SDL_MSS_RCM_RTI1_CLK_STATUS_CLKINUSE_RESETVAL                     (0x00000001U)
#define SDL_MSS_RCM_RTI1_CLK_STATUS_CLKINUSE_MAX                          (0x000000FFU)

#define SDL_MSS_RCM_RTI1_CLK_STATUS_CURRDIVIDER_MASK                      (0x0000FF00U)
#define SDL_MSS_RCM_RTI1_CLK_STATUS_CURRDIVIDER_SHIFT                     (0x00000008U)
#define SDL_MSS_RCM_RTI1_CLK_STATUS_CURRDIVIDER_RESETVAL                  (0x00000000U)
#define SDL_MSS_RCM_RTI1_CLK_STATUS_CURRDIVIDER_MAX                       (0x000000FFU)

#define SDL_MSS_RCM_RTI1_CLK_STATUS_RESETVAL                              (0x00000001U)

/* RTI2_CLK_STATUS */

#define SDL_MSS_RCM_RTI2_CLK_STATUS_CLKINUSE_MASK                         (0x000000FFU)
#define SDL_MSS_RCM_RTI2_CLK_STATUS_CLKINUSE_SHIFT                        (0x00000000U)
#define SDL_MSS_RCM_RTI2_CLK_STATUS_CLKINUSE_RESETVAL                     (0x00000001U)
#define SDL_MSS_RCM_RTI2_CLK_STATUS_CLKINUSE_MAX                          (0x000000FFU)

#define SDL_MSS_RCM_RTI2_CLK_STATUS_CURRDIVIDER_MASK                      (0x0000FF00U)
#define SDL_MSS_RCM_RTI2_CLK_STATUS_CURRDIVIDER_SHIFT                     (0x00000008U)
#define SDL_MSS_RCM_RTI2_CLK_STATUS_CURRDIVIDER_RESETVAL                  (0x00000000U)
#define SDL_MSS_RCM_RTI2_CLK_STATUS_CURRDIVIDER_MAX                       (0x000000FFU)

#define SDL_MSS_RCM_RTI2_CLK_STATUS_RESETVAL                              (0x00000001U)

/* RTI3_CLK_STATUS */

#define SDL_MSS_RCM_RTI3_CLK_STATUS_CLKINUSE_MASK                         (0x000000FFU)
#define SDL_MSS_RCM_RTI3_CLK_STATUS_CLKINUSE_SHIFT                        (0x00000000U)
#define SDL_MSS_RCM_RTI3_CLK_STATUS_CLKINUSE_RESETVAL                     (0x00000001U)
#define SDL_MSS_RCM_RTI3_CLK_STATUS_CLKINUSE_MAX                          (0x000000FFU)

#define SDL_MSS_RCM_RTI3_CLK_STATUS_CURRDIVIDER_MASK                      (0x0000FF00U)
#define SDL_MSS_RCM_RTI3_CLK_STATUS_CURRDIVIDER_SHIFT                     (0x00000008U)
#define SDL_MSS_RCM_RTI3_CLK_STATUS_CURRDIVIDER_RESETVAL                  (0x00000000U)
#define SDL_MSS_RCM_RTI3_CLK_STATUS_CURRDIVIDER_MAX                       (0x000000FFU)

#define SDL_MSS_RCM_RTI3_CLK_STATUS_RESETVAL                              (0x00000001U)

/* WDT0_CLK_STATUS */

#define SDL_MSS_RCM_WDT0_CLK_STATUS_CLKINUSE_MASK                         (0x000000FFU)
#define SDL_MSS_RCM_WDT0_CLK_STATUS_CLKINUSE_SHIFT                        (0x00000000U)
#define SDL_MSS_RCM_WDT0_CLK_STATUS_CLKINUSE_RESETVAL                     (0x00000001U)
#define SDL_MSS_RCM_WDT0_CLK_STATUS_CLKINUSE_MAX                          (0x000000FFU)

#define SDL_MSS_RCM_WDT0_CLK_STATUS_CURRDIVIDER_MASK                      (0x0000FF00U)
#define SDL_MSS_RCM_WDT0_CLK_STATUS_CURRDIVIDER_SHIFT                     (0x00000008U)
#define SDL_MSS_RCM_WDT0_CLK_STATUS_CURRDIVIDER_RESETVAL                  (0x00000000U)
#define SDL_MSS_RCM_WDT0_CLK_STATUS_CURRDIVIDER_MAX                       (0x000000FFU)

#define SDL_MSS_RCM_WDT0_CLK_STATUS_RESETVAL                              (0x00000001U)

/* WDT1_CLK_STATUS */

#define SDL_MSS_RCM_WDT1_CLK_STATUS_CLKINUSE_MASK                         (0x000000FFU)
#define SDL_MSS_RCM_WDT1_CLK_STATUS_CLKINUSE_SHIFT                        (0x00000000U)
#define SDL_MSS_RCM_WDT1_CLK_STATUS_CLKINUSE_RESETVAL                     (0x00000001U)
#define SDL_MSS_RCM_WDT1_CLK_STATUS_CLKINUSE_MAX                          (0x000000FFU)

#define SDL_MSS_RCM_WDT1_CLK_STATUS_CURRDIVIDER_MASK                      (0x0000FF00U)
#define SDL_MSS_RCM_WDT1_CLK_STATUS_CURRDIVIDER_SHIFT                     (0x00000008U)
#define SDL_MSS_RCM_WDT1_CLK_STATUS_CURRDIVIDER_RESETVAL                  (0x00000000U)
#define SDL_MSS_RCM_WDT1_CLK_STATUS_CURRDIVIDER_MAX                       (0x000000FFU)

#define SDL_MSS_RCM_WDT1_CLK_STATUS_RESETVAL                              (0x00000001U)

/* WDT2_CLK_STATUS */

#define SDL_MSS_RCM_WDT2_CLK_STATUS_CLKINUSE_MASK                         (0x000000FFU)
#define SDL_MSS_RCM_WDT2_CLK_STATUS_CLKINUSE_SHIFT                        (0x00000000U)
#define SDL_MSS_RCM_WDT2_CLK_STATUS_CLKINUSE_RESETVAL                     (0x00000001U)
#define SDL_MSS_RCM_WDT2_CLK_STATUS_CLKINUSE_MAX                          (0x000000FFU)

#define SDL_MSS_RCM_WDT2_CLK_STATUS_CURRDIVIDER_MASK                      (0x0000FF00U)
#define SDL_MSS_RCM_WDT2_CLK_STATUS_CURRDIVIDER_SHIFT                     (0x00000008U)
#define SDL_MSS_RCM_WDT2_CLK_STATUS_CURRDIVIDER_RESETVAL                  (0x00000000U)
#define SDL_MSS_RCM_WDT2_CLK_STATUS_CURRDIVIDER_MAX                       (0x000000FFU)

#define SDL_MSS_RCM_WDT2_CLK_STATUS_RESETVAL                              (0x00000001U)

/* WDT3_CLK_STATUS */

#define SDL_MSS_RCM_WDT3_CLK_STATUS_CLKINUSE_MASK                         (0x000000FFU)
#define SDL_MSS_RCM_WDT3_CLK_STATUS_CLKINUSE_SHIFT                        (0x00000000U)
#define SDL_MSS_RCM_WDT3_CLK_STATUS_CLKINUSE_RESETVAL                     (0x00000001U)
#define SDL_MSS_RCM_WDT3_CLK_STATUS_CLKINUSE_MAX                          (0x000000FFU)

#define SDL_MSS_RCM_WDT3_CLK_STATUS_CURRDIVIDER_MASK                      (0x0000FF00U)
#define SDL_MSS_RCM_WDT3_CLK_STATUS_CURRDIVIDER_SHIFT                     (0x00000008U)
#define SDL_MSS_RCM_WDT3_CLK_STATUS_CURRDIVIDER_RESETVAL                  (0x00000000U)
#define SDL_MSS_RCM_WDT3_CLK_STATUS_CURRDIVIDER_MAX                       (0x000000FFU)

#define SDL_MSS_RCM_WDT3_CLK_STATUS_RESETVAL                              (0x00000001U)

/* MCSPI0_CLK_STATUS */

#define SDL_MSS_RCM_MCSPI0_CLK_STATUS_CLKINUSE_MASK                       (0x000000FFU)
#define SDL_MSS_RCM_MCSPI0_CLK_STATUS_CLKINUSE_SHIFT                      (0x00000000U)
#define SDL_MSS_RCM_MCSPI0_CLK_STATUS_CLKINUSE_RESETVAL                   (0x00000001U)
#define SDL_MSS_RCM_MCSPI0_CLK_STATUS_CLKINUSE_MAX                        (0x000000FFU)

#define SDL_MSS_RCM_MCSPI0_CLK_STATUS_CURRDIVIDER_MASK                    (0x0000FF00U)
#define SDL_MSS_RCM_MCSPI0_CLK_STATUS_CURRDIVIDER_SHIFT                   (0x00000008U)
#define SDL_MSS_RCM_MCSPI0_CLK_STATUS_CURRDIVIDER_RESETVAL                (0x00000000U)
#define SDL_MSS_RCM_MCSPI0_CLK_STATUS_CURRDIVIDER_MAX                     (0x000000FFU)

#define SDL_MSS_RCM_MCSPI0_CLK_STATUS_RESETVAL                            (0x00000001U)

/* MCSPI1_CLK_STATUS */

#define SDL_MSS_RCM_MCSPI1_CLK_STATUS_CLKINUSE_MASK                       (0x000000FFU)
#define SDL_MSS_RCM_MCSPI1_CLK_STATUS_CLKINUSE_SHIFT                      (0x00000000U)
#define SDL_MSS_RCM_MCSPI1_CLK_STATUS_CLKINUSE_RESETVAL                   (0x00000001U)
#define SDL_MSS_RCM_MCSPI1_CLK_STATUS_CLKINUSE_MAX                        (0x000000FFU)

#define SDL_MSS_RCM_MCSPI1_CLK_STATUS_CURRDIVIDER_MASK                    (0x0000FF00U)
#define SDL_MSS_RCM_MCSPI1_CLK_STATUS_CURRDIVIDER_SHIFT                   (0x00000008U)
#define SDL_MSS_RCM_MCSPI1_CLK_STATUS_CURRDIVIDER_RESETVAL                (0x00000000U)
#define SDL_MSS_RCM_MCSPI1_CLK_STATUS_CURRDIVIDER_MAX                     (0x000000FFU)

#define SDL_MSS_RCM_MCSPI1_CLK_STATUS_RESETVAL                            (0x00000001U)

/* MCSPI2_CLK_STATUS */

#define SDL_MSS_RCM_MCSPI2_CLK_STATUS_CLKINUSE_MASK                       (0x000000FFU)
#define SDL_MSS_RCM_MCSPI2_CLK_STATUS_CLKINUSE_SHIFT                      (0x00000000U)
#define SDL_MSS_RCM_MCSPI2_CLK_STATUS_CLKINUSE_RESETVAL                   (0x00000001U)
#define SDL_MSS_RCM_MCSPI2_CLK_STATUS_CLKINUSE_MAX                        (0x000000FFU)

#define SDL_MSS_RCM_MCSPI2_CLK_STATUS_CURRDIVIDER_MASK                    (0x0000FF00U)
#define SDL_MSS_RCM_MCSPI2_CLK_STATUS_CURRDIVIDER_SHIFT                   (0x00000008U)
#define SDL_MSS_RCM_MCSPI2_CLK_STATUS_CURRDIVIDER_RESETVAL                (0x00000000U)
#define SDL_MSS_RCM_MCSPI2_CLK_STATUS_CURRDIVIDER_MAX                     (0x000000FFU)

#define SDL_MSS_RCM_MCSPI2_CLK_STATUS_RESETVAL                            (0x00000001U)

/* MCSPI3_CLK_STATUS */

#define SDL_MSS_RCM_MCSPI3_CLK_STATUS_CLKINUSE_MASK                       (0x000000FFU)
#define SDL_MSS_RCM_MCSPI3_CLK_STATUS_CLKINUSE_SHIFT                      (0x00000000U)
#define SDL_MSS_RCM_MCSPI3_CLK_STATUS_CLKINUSE_RESETVAL                   (0x00000001U)
#define SDL_MSS_RCM_MCSPI3_CLK_STATUS_CLKINUSE_MAX                        (0x000000FFU)

#define SDL_MSS_RCM_MCSPI3_CLK_STATUS_CURRDIVIDER_MASK                    (0x0000FF00U)
#define SDL_MSS_RCM_MCSPI3_CLK_STATUS_CURRDIVIDER_SHIFT                   (0x00000008U)
#define SDL_MSS_RCM_MCSPI3_CLK_STATUS_CURRDIVIDER_RESETVAL                (0x00000000U)
#define SDL_MSS_RCM_MCSPI3_CLK_STATUS_CURRDIVIDER_MAX                     (0x000000FFU)

#define SDL_MSS_RCM_MCSPI3_CLK_STATUS_RESETVAL                            (0x00000001U)

/* MCSPI4_CLK_STATUS */

#define SDL_MSS_RCM_MCSPI4_CLK_STATUS_CLKINUSE_MASK                       (0x000000FFU)
#define SDL_MSS_RCM_MCSPI4_CLK_STATUS_CLKINUSE_SHIFT                      (0x00000000U)
#define SDL_MSS_RCM_MCSPI4_CLK_STATUS_CLKINUSE_RESETVAL                   (0x00000001U)
#define SDL_MSS_RCM_MCSPI4_CLK_STATUS_CLKINUSE_MAX                        (0x000000FFU)

#define SDL_MSS_RCM_MCSPI4_CLK_STATUS_CURRDIVIDER_MASK                    (0x0000FF00U)
#define SDL_MSS_RCM_MCSPI4_CLK_STATUS_CURRDIVIDER_SHIFT                   (0x00000008U)
#define SDL_MSS_RCM_MCSPI4_CLK_STATUS_CURRDIVIDER_RESETVAL                (0x00000000U)
#define SDL_MSS_RCM_MCSPI4_CLK_STATUS_CURRDIVIDER_MAX                     (0x000000FFU)

#define SDL_MSS_RCM_MCSPI4_CLK_STATUS_RESETVAL                            (0x00000001U)

/* MMC0_CLK_STATUS */

#define SDL_MSS_RCM_MMC0_CLK_STATUS_CLKINUSE_MASK                         (0x000000FFU)
#define SDL_MSS_RCM_MMC0_CLK_STATUS_CLKINUSE_SHIFT                        (0x00000000U)
#define SDL_MSS_RCM_MMC0_CLK_STATUS_CLKINUSE_RESETVAL                     (0x00000001U)
#define SDL_MSS_RCM_MMC0_CLK_STATUS_CLKINUSE_MAX                          (0x000000FFU)

#define SDL_MSS_RCM_MMC0_CLK_STATUS_CURRDIVIDER_MASK                      (0x0000FF00U)
#define SDL_MSS_RCM_MMC0_CLK_STATUS_CURRDIVIDER_SHIFT                     (0x00000008U)
#define SDL_MSS_RCM_MMC0_CLK_STATUS_CURRDIVIDER_RESETVAL                  (0x00000000U)
#define SDL_MSS_RCM_MMC0_CLK_STATUS_CURRDIVIDER_MAX                       (0x000000FFU)

#define SDL_MSS_RCM_MMC0_CLK_STATUS_RESETVAL                              (0x00000001U)

/* ICSSM0_UART_CLK_STATUS */

#define SDL_MSS_RCM_ICSSM0_UART_CLK_STATUS_CLKINUSE_MASK                  (0x000000FFU)
#define SDL_MSS_RCM_ICSSM0_UART_CLK_STATUS_CLKINUSE_SHIFT                 (0x00000000U)
#define SDL_MSS_RCM_ICSSM0_UART_CLK_STATUS_CLKINUSE_RESETVAL              (0x00000001U)
#define SDL_MSS_RCM_ICSSM0_UART_CLK_STATUS_CLKINUSE_MAX                   (0x000000FFU)

#define SDL_MSS_RCM_ICSSM0_UART_CLK_STATUS_CURRDIVIDER_MASK               (0x0000FF00U)
#define SDL_MSS_RCM_ICSSM0_UART_CLK_STATUS_CURRDIVIDER_SHIFT              (0x00000008U)
#define SDL_MSS_RCM_ICSSM0_UART_CLK_STATUS_CURRDIVIDER_RESETVAL           (0x00000000U)
#define SDL_MSS_RCM_ICSSM0_UART_CLK_STATUS_CURRDIVIDER_MAX                (0x000000FFU)

#define SDL_MSS_RCM_ICSSM0_UART_CLK_STATUS_RESETVAL                       (0x00000001U)

/* CPTS_CLK_STATUS */

#define SDL_MSS_RCM_CPTS_CLK_STATUS_CLKINUSE_MASK                         (0x000000FFU)
#define SDL_MSS_RCM_CPTS_CLK_STATUS_CLKINUSE_SHIFT                        (0x00000000U)
#define SDL_MSS_RCM_CPTS_CLK_STATUS_CLKINUSE_RESETVAL                     (0x00000001U)
#define SDL_MSS_RCM_CPTS_CLK_STATUS_CLKINUSE_MAX                          (0x000000FFU)

#define SDL_MSS_RCM_CPTS_CLK_STATUS_CURRDIVIDER_MASK                      (0x0000FF00U)
#define SDL_MSS_RCM_CPTS_CLK_STATUS_CURRDIVIDER_SHIFT                     (0x00000008U)
#define SDL_MSS_RCM_CPTS_CLK_STATUS_CURRDIVIDER_RESETVAL                  (0x00000000U)
#define SDL_MSS_RCM_CPTS_CLK_STATUS_CURRDIVIDER_MAX                       (0x000000FFU)

#define SDL_MSS_RCM_CPTS_CLK_STATUS_RESETVAL                              (0x00000001U)

/* GPMC_CLK_STATUS */

#define SDL_MSS_RCM_GPMC_CLK_STATUS_CLKINUSE_MASK                         (0x000000FFU)
#define SDL_MSS_RCM_GPMC_CLK_STATUS_CLKINUSE_SHIFT                        (0x00000000U)
#define SDL_MSS_RCM_GPMC_CLK_STATUS_CLKINUSE_RESETVAL                     (0x00000001U)
#define SDL_MSS_RCM_GPMC_CLK_STATUS_CLKINUSE_MAX                          (0x000000FFU)

#define SDL_MSS_RCM_GPMC_CLK_STATUS_CURRDIVIDER_MASK                      (0x0000FF00U)
#define SDL_MSS_RCM_GPMC_CLK_STATUS_CURRDIVIDER_SHIFT                     (0x00000008U)
#define SDL_MSS_RCM_GPMC_CLK_STATUS_CURRDIVIDER_RESETVAL                  (0x00000000U)
#define SDL_MSS_RCM_GPMC_CLK_STATUS_CURRDIVIDER_MAX                       (0x000000FFU)

#define SDL_MSS_RCM_GPMC_CLK_STATUS_RESETVAL                              (0x00000001U)

/* CONTROLSS_PLL_CLK_STATUS */

#define SDL_MSS_RCM_CONTROLSS_PLL_CLK_STATUS_CLKINUSE_MASK                (0x000000FFU)
#define SDL_MSS_RCM_CONTROLSS_PLL_CLK_STATUS_CLKINUSE_SHIFT               (0x00000000U)
#define SDL_MSS_RCM_CONTROLSS_PLL_CLK_STATUS_CLKINUSE_RESETVAL            (0x00000001U)
#define SDL_MSS_RCM_CONTROLSS_PLL_CLK_STATUS_CLKINUSE_MAX                 (0x000000FFU)

#define SDL_MSS_RCM_CONTROLSS_PLL_CLK_STATUS_CURRDIVIDER_MASK             (0x0000FF00U)
#define SDL_MSS_RCM_CONTROLSS_PLL_CLK_STATUS_CURRDIVIDER_SHIFT            (0x00000008U)
#define SDL_MSS_RCM_CONTROLSS_PLL_CLK_STATUS_CURRDIVIDER_RESETVAL         (0x00000000U)
#define SDL_MSS_RCM_CONTROLSS_PLL_CLK_STATUS_CURRDIVIDER_MAX              (0x000000FFU)

#define SDL_MSS_RCM_CONTROLSS_PLL_CLK_STATUS_RESETVAL                     (0x00000001U)

/* I2C_CLK_STATUS */

#define SDL_MSS_RCM_I2C_CLK_STATUS_CLKINUSE_MASK                          (0x000000FFU)
#define SDL_MSS_RCM_I2C_CLK_STATUS_CLKINUSE_SHIFT                         (0x00000000U)
#define SDL_MSS_RCM_I2C_CLK_STATUS_CLKINUSE_RESETVAL                      (0x00000001U)
#define SDL_MSS_RCM_I2C_CLK_STATUS_CLKINUSE_MAX                           (0x000000FFU)

#define SDL_MSS_RCM_I2C_CLK_STATUS_CURRDIVIDER_MASK                       (0x0000FF00U)
#define SDL_MSS_RCM_I2C_CLK_STATUS_CURRDIVIDER_SHIFT                      (0x00000008U)
#define SDL_MSS_RCM_I2C_CLK_STATUS_CURRDIVIDER_RESETVAL                   (0x00000000U)
#define SDL_MSS_RCM_I2C_CLK_STATUS_CURRDIVIDER_MAX                        (0x000000FFU)

#define SDL_MSS_RCM_I2C_CLK_STATUS_RESETVAL                               (0x00000001U)

/* LIN0_UART0_CLK_STATUS */

#define SDL_MSS_RCM_LIN0_UART0_CLK_STATUS_CLKINUSE_MASK                   (0x000000FFU)
#define SDL_MSS_RCM_LIN0_UART0_CLK_STATUS_CLKINUSE_SHIFT                  (0x00000000U)
#define SDL_MSS_RCM_LIN0_UART0_CLK_STATUS_CLKINUSE_RESETVAL               (0x00000001U)
#define SDL_MSS_RCM_LIN0_UART0_CLK_STATUS_CLKINUSE_MAX                    (0x000000FFU)

#define SDL_MSS_RCM_LIN0_UART0_CLK_STATUS_CURRDIVIDER_MASK                (0x0000FF00U)
#define SDL_MSS_RCM_LIN0_UART0_CLK_STATUS_CURRDIVIDER_SHIFT               (0x00000008U)
#define SDL_MSS_RCM_LIN0_UART0_CLK_STATUS_CURRDIVIDER_RESETVAL            (0x00000000U)
#define SDL_MSS_RCM_LIN0_UART0_CLK_STATUS_CURRDIVIDER_MAX                 (0x000000FFU)

#define SDL_MSS_RCM_LIN0_UART0_CLK_STATUS_RESETVAL                        (0x00000001U)

/* LIN1_UART1_CLK_STATUS */

#define SDL_MSS_RCM_LIN1_UART1_CLK_STATUS_CLKINUSE_MASK                   (0x000000FFU)
#define SDL_MSS_RCM_LIN1_UART1_CLK_STATUS_CLKINUSE_SHIFT                  (0x00000000U)
#define SDL_MSS_RCM_LIN1_UART1_CLK_STATUS_CLKINUSE_RESETVAL               (0x00000001U)
#define SDL_MSS_RCM_LIN1_UART1_CLK_STATUS_CLKINUSE_MAX                    (0x000000FFU)

#define SDL_MSS_RCM_LIN1_UART1_CLK_STATUS_CURRDIVIDER_MASK                (0x0000FF00U)
#define SDL_MSS_RCM_LIN1_UART1_CLK_STATUS_CURRDIVIDER_SHIFT               (0x00000008U)
#define SDL_MSS_RCM_LIN1_UART1_CLK_STATUS_CURRDIVIDER_RESETVAL            (0x00000000U)
#define SDL_MSS_RCM_LIN1_UART1_CLK_STATUS_CURRDIVIDER_MAX                 (0x000000FFU)

#define SDL_MSS_RCM_LIN1_UART1_CLK_STATUS_RESETVAL                        (0x00000001U)

/* LIN2_UART2_CLK_STATUS */

#define SDL_MSS_RCM_LIN2_UART2_CLK_STATUS_CLKINUSE_MASK                   (0x000000FFU)
#define SDL_MSS_RCM_LIN2_UART2_CLK_STATUS_CLKINUSE_SHIFT                  (0x00000000U)
#define SDL_MSS_RCM_LIN2_UART2_CLK_STATUS_CLKINUSE_RESETVAL               (0x00000001U)
#define SDL_MSS_RCM_LIN2_UART2_CLK_STATUS_CLKINUSE_MAX                    (0x000000FFU)

#define SDL_MSS_RCM_LIN2_UART2_CLK_STATUS_CURRDIVIDER_MASK                (0x0000FF00U)
#define SDL_MSS_RCM_LIN2_UART2_CLK_STATUS_CURRDIVIDER_SHIFT               (0x00000008U)
#define SDL_MSS_RCM_LIN2_UART2_CLK_STATUS_CURRDIVIDER_RESETVAL            (0x00000000U)
#define SDL_MSS_RCM_LIN2_UART2_CLK_STATUS_CURRDIVIDER_MAX                 (0x000000FFU)

#define SDL_MSS_RCM_LIN2_UART2_CLK_STATUS_RESETVAL                        (0x00000001U)

/* LIN3_UART3_CLK_STATUS */

#define SDL_MSS_RCM_LIN3_UART3_CLK_STATUS_CLKINUSE_MASK                   (0x000000FFU)
#define SDL_MSS_RCM_LIN3_UART3_CLK_STATUS_CLKINUSE_SHIFT                  (0x00000000U)
#define SDL_MSS_RCM_LIN3_UART3_CLK_STATUS_CLKINUSE_RESETVAL               (0x00000001U)
#define SDL_MSS_RCM_LIN3_UART3_CLK_STATUS_CLKINUSE_MAX                    (0x000000FFU)

#define SDL_MSS_RCM_LIN3_UART3_CLK_STATUS_CURRDIVIDER_MASK                (0x0000FF00U)
#define SDL_MSS_RCM_LIN3_UART3_CLK_STATUS_CURRDIVIDER_SHIFT               (0x00000008U)
#define SDL_MSS_RCM_LIN3_UART3_CLK_STATUS_CURRDIVIDER_RESETVAL            (0x00000000U)
#define SDL_MSS_RCM_LIN3_UART3_CLK_STATUS_CURRDIVIDER_MAX                 (0x000000FFU)

#define SDL_MSS_RCM_LIN3_UART3_CLK_STATUS_RESETVAL                        (0x00000001U)

/* LIN4_UART4_CLK_STATUS */

#define SDL_MSS_RCM_LIN4_UART4_CLK_STATUS_CLKINUSE_MASK                   (0x000000FFU)
#define SDL_MSS_RCM_LIN4_UART4_CLK_STATUS_CLKINUSE_SHIFT                  (0x00000000U)
#define SDL_MSS_RCM_LIN4_UART4_CLK_STATUS_CLKINUSE_RESETVAL               (0x00000001U)
#define SDL_MSS_RCM_LIN4_UART4_CLK_STATUS_CLKINUSE_MAX                    (0x000000FFU)

#define SDL_MSS_RCM_LIN4_UART4_CLK_STATUS_CURRDIVIDER_MASK                (0x0000FF00U)
#define SDL_MSS_RCM_LIN4_UART4_CLK_STATUS_CURRDIVIDER_SHIFT               (0x00000008U)
#define SDL_MSS_RCM_LIN4_UART4_CLK_STATUS_CURRDIVIDER_RESETVAL            (0x00000000U)
#define SDL_MSS_RCM_LIN4_UART4_CLK_STATUS_CURRDIVIDER_MAX                 (0x000000FFU)

#define SDL_MSS_RCM_LIN4_UART4_CLK_STATUS_RESETVAL                        (0x00000001U)

/* LIN5_UART5_CLK_STATUS */

#define SDL_MSS_RCM_LIN5_UART5_CLK_STATUS_CLKINUSE_MASK                   (0x000000FFU)
#define SDL_MSS_RCM_LIN5_UART5_CLK_STATUS_CLKINUSE_SHIFT                  (0x00000000U)
#define SDL_MSS_RCM_LIN5_UART5_CLK_STATUS_CLKINUSE_RESETVAL               (0x00000001U)
#define SDL_MSS_RCM_LIN5_UART5_CLK_STATUS_CLKINUSE_MAX                    (0x000000FFU)

#define SDL_MSS_RCM_LIN5_UART5_CLK_STATUS_CURRDIVIDER_MASK                (0x0000FF00U)
#define SDL_MSS_RCM_LIN5_UART5_CLK_STATUS_CURRDIVIDER_SHIFT               (0x00000008U)
#define SDL_MSS_RCM_LIN5_UART5_CLK_STATUS_CURRDIVIDER_RESETVAL            (0x00000000U)
#define SDL_MSS_RCM_LIN5_UART5_CLK_STATUS_CURRDIVIDER_MAX                 (0x000000FFU)

#define SDL_MSS_RCM_LIN5_UART5_CLK_STATUS_RESETVAL                        (0x00000001U)

/* RGMII_250_CLK_STATUS */

#define SDL_MSS_RCM_RGMII_250_CLK_STATUS_CURRDIVIDER_MASK                 (0x0000FF00U)
#define SDL_MSS_RCM_RGMII_250_CLK_STATUS_CURRDIVIDER_SHIFT                (0x00000008U)
#define SDL_MSS_RCM_RGMII_250_CLK_STATUS_CURRDIVIDER_RESETVAL             (0x00000001U)
#define SDL_MSS_RCM_RGMII_250_CLK_STATUS_CURRDIVIDER_MAX                  (0x000000FFU)

#define SDL_MSS_RCM_RGMII_250_CLK_STATUS_RESETVAL                         (0x00000100U)

/* RGMII_50_CLK_STATUS */

#define SDL_MSS_RCM_RGMII_50_CLK_STATUS_CURRDIVIDER_MASK                  (0x0000FF00U)
#define SDL_MSS_RCM_RGMII_50_CLK_STATUS_CURRDIVIDER_SHIFT                 (0x00000008U)
#define SDL_MSS_RCM_RGMII_50_CLK_STATUS_CURRDIVIDER_RESETVAL              (0x00000009U)
#define SDL_MSS_RCM_RGMII_50_CLK_STATUS_CURRDIVIDER_MAX                   (0x000000FFU)

#define SDL_MSS_RCM_RGMII_50_CLK_STATUS_RESETVAL                          (0x00000900U)

/* RGMII_5_CLK_STATUS */

#define SDL_MSS_RCM_RGMII_5_CLK_STATUS_CURRDIVIDER_MASK                   (0x0000FF00U)
#define SDL_MSS_RCM_RGMII_5_CLK_STATUS_CURRDIVIDER_SHIFT                  (0x00000008U)
#define SDL_MSS_RCM_RGMII_5_CLK_STATUS_CURRDIVIDER_RESETVAL               (0x00000063U)
#define SDL_MSS_RCM_RGMII_5_CLK_STATUS_CURRDIVIDER_MAX                    (0x000000FFU)

#define SDL_MSS_RCM_RGMII_5_CLK_STATUS_RESETVAL                           (0x00006300U)

/* MMC0_32K_CLK_STATUS */

#define SDL_MSS_RCM_MMC0_32K_CLK_STATUS_CURRDIVIDER_MASK                  (0x0003FF00U)
#define SDL_MSS_RCM_MMC0_32K_CLK_STATUS_CURRDIVIDER_SHIFT                 (0x00000008U)
#define SDL_MSS_RCM_MMC0_32K_CLK_STATUS_CURRDIVIDER_RESETVAL              (0x0000030CU)
#define SDL_MSS_RCM_MMC0_32K_CLK_STATUS_CURRDIVIDER_MAX                   (0x000003FFU)

#define SDL_MSS_RCM_MMC0_32K_CLK_STATUS_RESETVAL                          (0x00030C00U)

/* TEMPSENSE_32K_CLK_STATUS */

#define SDL_MSS_RCM_TEMPSENSE_32K_CLK_STATUS_CURRDIVIDER_MASK             (0x0003FF00U)
#define SDL_MSS_RCM_TEMPSENSE_32K_CLK_STATUS_CURRDIVIDER_SHIFT            (0x00000008U)
#define SDL_MSS_RCM_TEMPSENSE_32K_CLK_STATUS_CURRDIVIDER_RESETVAL         (0x0000030CU)
#define SDL_MSS_RCM_TEMPSENSE_32K_CLK_STATUS_CURRDIVIDER_MAX              (0x000003FFU)

#define SDL_MSS_RCM_TEMPSENSE_32K_CLK_STATUS_RESETVAL                     (0x00030C00U)

/* MSS_ELM_CLK_STATUS */

#define SDL_MSS_RCM_MSS_ELM_CLK_STATUS_CURRDIVIDER_MASK                   (0x0000FF00U)
#define SDL_MSS_RCM_MSS_ELM_CLK_STATUS_CURRDIVIDER_SHIFT                  (0x00000008U)
#define SDL_MSS_RCM_MSS_ELM_CLK_STATUS_CURRDIVIDER_RESETVAL               (0x00000003U)
#define SDL_MSS_RCM_MSS_ELM_CLK_STATUS_CURRDIVIDER_MAX                    (0x000000FFU)

#define SDL_MSS_RCM_MSS_ELM_CLK_STATUS_RESETVAL                           (0x00000300U)

/* R5SS0_POR_RST_CTRL */

#define SDL_MSS_RCM_R5SS0_POR_RST_CTRL_ASSERT_MASK                        (0x00000007U)
#define SDL_MSS_RCM_R5SS0_POR_RST_CTRL_ASSERT_SHIFT                       (0x00000000U)
#define SDL_MSS_RCM_R5SS0_POR_RST_CTRL_ASSERT_RESETVAL                    (0x00000000U)
#define SDL_MSS_RCM_R5SS0_POR_RST_CTRL_ASSERT_MAX                         (0x00000007U)

#define SDL_MSS_RCM_R5SS0_POR_RST_CTRL_RESETVAL                           (0x00000000U)

/* R5SS1_POR_RST_CTRL */

#define SDL_MSS_RCM_R5SS1_POR_RST_CTRL_ASSERT_MASK                        (0x00000007U)
#define SDL_MSS_RCM_R5SS1_POR_RST_CTRL_ASSERT_SHIFT                       (0x00000000U)
#define SDL_MSS_RCM_R5SS1_POR_RST_CTRL_ASSERT_RESETVAL                    (0x00000000U)
#define SDL_MSS_RCM_R5SS1_POR_RST_CTRL_ASSERT_MAX                         (0x00000007U)

#define SDL_MSS_RCM_R5SS1_POR_RST_CTRL_RESETVAL                           (0x00000000U)

/* R5SS0_CORE0_GRST_CTRL */

#define SDL_MSS_RCM_R5SS0_CORE0_GRST_CTRL_ASSERT_MASK                     (0x00000007U)
#define SDL_MSS_RCM_R5SS0_CORE0_GRST_CTRL_ASSERT_SHIFT                    (0x00000000U)
#define SDL_MSS_RCM_R5SS0_CORE0_GRST_CTRL_ASSERT_RESETVAL                 (0x00000000U)
#define SDL_MSS_RCM_R5SS0_CORE0_GRST_CTRL_ASSERT_MAX                      (0x00000007U)

#define SDL_MSS_RCM_R5SS0_CORE0_GRST_CTRL_RESETVAL                        (0x00000000U)

/* R5SS1_CORE0_GRST_CTRL */

#define SDL_MSS_RCM_R5SS1_CORE0_GRST_CTRL_ASSERT_MASK                     (0x00000007U)
#define SDL_MSS_RCM_R5SS1_CORE0_GRST_CTRL_ASSERT_SHIFT                    (0x00000000U)
#define SDL_MSS_RCM_R5SS1_CORE0_GRST_CTRL_ASSERT_RESETVAL                 (0x00000000U)
#define SDL_MSS_RCM_R5SS1_CORE0_GRST_CTRL_ASSERT_MAX                      (0x00000007U)

#define SDL_MSS_RCM_R5SS1_CORE0_GRST_CTRL_RESETVAL                        (0x00000000U)

/* R5SS0_CORE1_GRST_CTRL */

#define SDL_MSS_RCM_R5SS0_CORE1_GRST_CTRL_ASSERT_MASK                     (0x00000007U)
#define SDL_MSS_RCM_R5SS0_CORE1_GRST_CTRL_ASSERT_SHIFT                    (0x00000000U)
#define SDL_MSS_RCM_R5SS0_CORE1_GRST_CTRL_ASSERT_RESETVAL                 (0x00000000U)
#define SDL_MSS_RCM_R5SS0_CORE1_GRST_CTRL_ASSERT_MAX                      (0x00000007U)

#define SDL_MSS_RCM_R5SS0_CORE1_GRST_CTRL_RESETVAL                        (0x00000000U)

/* R5SS1_CORE1_GRST_CTRL */

#define SDL_MSS_RCM_R5SS1_CORE1_GRST_CTRL_ASSERT_MASK                     (0x00000007U)
#define SDL_MSS_RCM_R5SS1_CORE1_GRST_CTRL_ASSERT_SHIFT                    (0x00000000U)
#define SDL_MSS_RCM_R5SS1_CORE1_GRST_CTRL_ASSERT_RESETVAL                 (0x00000000U)
#define SDL_MSS_RCM_R5SS1_CORE1_GRST_CTRL_ASSERT_MAX                      (0x00000007U)

#define SDL_MSS_RCM_R5SS1_CORE1_GRST_CTRL_RESETVAL                        (0x00000000U)

/* R5SS0_CORE0_LRST_CTRL */

#define SDL_MSS_RCM_R5SS0_CORE0_LRST_CTRL_ASSERT_MASK                     (0x00000007U)
#define SDL_MSS_RCM_R5SS0_CORE0_LRST_CTRL_ASSERT_SHIFT                    (0x00000000U)
#define SDL_MSS_RCM_R5SS0_CORE0_LRST_CTRL_ASSERT_RESETVAL                 (0x00000000U)
#define SDL_MSS_RCM_R5SS0_CORE0_LRST_CTRL_ASSERT_MAX                      (0x00000007U)

#define SDL_MSS_RCM_R5SS0_CORE0_LRST_CTRL_RESETVAL                        (0x00000000U)

/* R5SS1_CORE0_LRST_CTRL */

#define SDL_MSS_RCM_R5SS1_CORE0_LRST_CTRL_ASSERT_MASK                     (0x00000007U)
#define SDL_MSS_RCM_R5SS1_CORE0_LRST_CTRL_ASSERT_SHIFT                    (0x00000000U)
#define SDL_MSS_RCM_R5SS1_CORE0_LRST_CTRL_ASSERT_RESETVAL                 (0x00000000U)
#define SDL_MSS_RCM_R5SS1_CORE0_LRST_CTRL_ASSERT_MAX                      (0x00000007U)

#define SDL_MSS_RCM_R5SS1_CORE0_LRST_CTRL_RESETVAL                        (0x00000000U)

/* R5SS0_CORE1_LRST_CTRL */

#define SDL_MSS_RCM_R5SS0_CORE1_LRST_CTRL_ASSERT_MASK                     (0x00000007U)
#define SDL_MSS_RCM_R5SS0_CORE1_LRST_CTRL_ASSERT_SHIFT                    (0x00000000U)
#define SDL_MSS_RCM_R5SS0_CORE1_LRST_CTRL_ASSERT_RESETVAL                 (0x00000000U)
#define SDL_MSS_RCM_R5SS0_CORE1_LRST_CTRL_ASSERT_MAX                      (0x00000007U)

#define SDL_MSS_RCM_R5SS0_CORE1_LRST_CTRL_RESETVAL                        (0x00000000U)

/* R5SS1_CORE1_LRST_CTRL */

#define SDL_MSS_RCM_R5SS1_CORE1_LRST_CTRL_ASSERT_MASK                     (0x00000007U)
#define SDL_MSS_RCM_R5SS1_CORE1_LRST_CTRL_ASSERT_SHIFT                    (0x00000000U)
#define SDL_MSS_RCM_R5SS1_CORE1_LRST_CTRL_ASSERT_RESETVAL                 (0x00000000U)
#define SDL_MSS_RCM_R5SS1_CORE1_LRST_CTRL_ASSERT_MAX                      (0x00000007U)

#define SDL_MSS_RCM_R5SS1_CORE1_LRST_CTRL_RESETVAL                        (0x00000000U)

/* R5SS0_VIM0_RST_CTRL */

#define SDL_MSS_RCM_R5SS0_VIM0_RST_CTRL_ASSERT_MASK                       (0x00000007U)
#define SDL_MSS_RCM_R5SS0_VIM0_RST_CTRL_ASSERT_SHIFT                      (0x00000000U)
#define SDL_MSS_RCM_R5SS0_VIM0_RST_CTRL_ASSERT_RESETVAL                   (0x00000000U)
#define SDL_MSS_RCM_R5SS0_VIM0_RST_CTRL_ASSERT_MAX                        (0x00000007U)

#define SDL_MSS_RCM_R5SS0_VIM0_RST_CTRL_RESETVAL                          (0x00000000U)

/* R5SS1_VIM0_RST_CTRL */

#define SDL_MSS_RCM_R5SS1_VIM0_RST_CTRL_ASSERT_MASK                       (0x00000007U)
#define SDL_MSS_RCM_R5SS1_VIM0_RST_CTRL_ASSERT_SHIFT                      (0x00000000U)
#define SDL_MSS_RCM_R5SS1_VIM0_RST_CTRL_ASSERT_RESETVAL                   (0x00000000U)
#define SDL_MSS_RCM_R5SS1_VIM0_RST_CTRL_ASSERT_MAX                        (0x00000007U)

#define SDL_MSS_RCM_R5SS1_VIM0_RST_CTRL_RESETVAL                          (0x00000000U)

/* R5SS0_VIM1_RST_CTRL */

#define SDL_MSS_RCM_R5SS0_VIM1_RST_CTRL_ASSERT_MASK                       (0x00000007U)
#define SDL_MSS_RCM_R5SS0_VIM1_RST_CTRL_ASSERT_SHIFT                      (0x00000000U)
#define SDL_MSS_RCM_R5SS0_VIM1_RST_CTRL_ASSERT_RESETVAL                   (0x00000000U)
#define SDL_MSS_RCM_R5SS0_VIM1_RST_CTRL_ASSERT_MAX                        (0x00000007U)

#define SDL_MSS_RCM_R5SS0_VIM1_RST_CTRL_RESETVAL                          (0x00000000U)

/* R5SS1_VIM1_RST_CTRL */

#define SDL_MSS_RCM_R5SS1_VIM1_RST_CTRL_ASSERT_MASK                       (0x00000007U)
#define SDL_MSS_RCM_R5SS1_VIM1_RST_CTRL_ASSERT_SHIFT                      (0x00000000U)
#define SDL_MSS_RCM_R5SS1_VIM1_RST_CTRL_ASSERT_RESETVAL                   (0x00000000U)
#define SDL_MSS_RCM_R5SS1_VIM1_RST_CTRL_ASSERT_MAX                        (0x00000007U)

#define SDL_MSS_RCM_R5SS1_VIM1_RST_CTRL_RESETVAL                          (0x00000000U)

/* MCRC0_RST_CTRL */

#define SDL_MSS_RCM_MCRC0_RST_CTRL_ASSERT_MASK                            (0x00000007U)
#define SDL_MSS_RCM_MCRC0_RST_CTRL_ASSERT_SHIFT                           (0x00000000U)
#define SDL_MSS_RCM_MCRC0_RST_CTRL_ASSERT_RESETVAL                        (0x00000000U)
#define SDL_MSS_RCM_MCRC0_RST_CTRL_ASSERT_MAX                             (0x00000007U)

#define SDL_MSS_RCM_MCRC0_RST_CTRL_RESETVAL                               (0x00000000U)

/* RTI0_RST_CTRL */

#define SDL_MSS_RCM_RTI0_RST_CTRL_ASSERT_MASK                             (0x00000007U)
#define SDL_MSS_RCM_RTI0_RST_CTRL_ASSERT_SHIFT                            (0x00000000U)
#define SDL_MSS_RCM_RTI0_RST_CTRL_ASSERT_RESETVAL                         (0x00000000U)
#define SDL_MSS_RCM_RTI0_RST_CTRL_ASSERT_MAX                              (0x00000007U)

#define SDL_MSS_RCM_RTI0_RST_CTRL_RESETVAL                                (0x00000000U)

/* RTI1_RST_CTRL */

#define SDL_MSS_RCM_RTI1_RST_CTRL_ASSERT_MASK                             (0x00000007U)
#define SDL_MSS_RCM_RTI1_RST_CTRL_ASSERT_SHIFT                            (0x00000000U)
#define SDL_MSS_RCM_RTI1_RST_CTRL_ASSERT_RESETVAL                         (0x00000000U)
#define SDL_MSS_RCM_RTI1_RST_CTRL_ASSERT_MAX                              (0x00000007U)

#define SDL_MSS_RCM_RTI1_RST_CTRL_RESETVAL                                (0x00000000U)

/* RTI2_RST_CTRL */

#define SDL_MSS_RCM_RTI2_RST_CTRL_ASSERT_MASK                             (0x00000007U)
#define SDL_MSS_RCM_RTI2_RST_CTRL_ASSERT_SHIFT                            (0x00000000U)
#define SDL_MSS_RCM_RTI2_RST_CTRL_ASSERT_RESETVAL                         (0x00000000U)
#define SDL_MSS_RCM_RTI2_RST_CTRL_ASSERT_MAX                              (0x00000007U)

#define SDL_MSS_RCM_RTI2_RST_CTRL_RESETVAL                                (0x00000000U)

/* RTI3_RST_CTRL */

#define SDL_MSS_RCM_RTI3_RST_CTRL_ASSERT_MASK                             (0x00000007U)
#define SDL_MSS_RCM_RTI3_RST_CTRL_ASSERT_SHIFT                            (0x00000000U)
#define SDL_MSS_RCM_RTI3_RST_CTRL_ASSERT_RESETVAL                         (0x00000000U)
#define SDL_MSS_RCM_RTI3_RST_CTRL_ASSERT_MAX                              (0x00000007U)

#define SDL_MSS_RCM_RTI3_RST_CTRL_RESETVAL                                (0x00000000U)

/* WDT0_RST_CTRL */

#define SDL_MSS_RCM_WDT0_RST_CTRL_ASSERT_MASK                             (0x00000007U)
#define SDL_MSS_RCM_WDT0_RST_CTRL_ASSERT_SHIFT                            (0x00000000U)
#define SDL_MSS_RCM_WDT0_RST_CTRL_ASSERT_RESETVAL                         (0x00000000U)
#define SDL_MSS_RCM_WDT0_RST_CTRL_ASSERT_MAX                              (0x00000007U)

#define SDL_MSS_RCM_WDT0_RST_CTRL_RESETVAL                                (0x00000000U)

/* WDT1_RST_CTRL */

#define SDL_MSS_RCM_WDT1_RST_CTRL_ASSERT_MASK                             (0x00000007U)
#define SDL_MSS_RCM_WDT1_RST_CTRL_ASSERT_SHIFT                            (0x00000000U)
#define SDL_MSS_RCM_WDT1_RST_CTRL_ASSERT_RESETVAL                         (0x00000000U)
#define SDL_MSS_RCM_WDT1_RST_CTRL_ASSERT_MAX                              (0x00000007U)

#define SDL_MSS_RCM_WDT1_RST_CTRL_RESETVAL                                (0x00000000U)

/* WDT2_RST_CTRL */

#define SDL_MSS_RCM_WDT2_RST_CTRL_ASSERT_MASK                             (0x00000007U)
#define SDL_MSS_RCM_WDT2_RST_CTRL_ASSERT_SHIFT                            (0x00000000U)
#define SDL_MSS_RCM_WDT2_RST_CTRL_ASSERT_RESETVAL                         (0x00000000U)
#define SDL_MSS_RCM_WDT2_RST_CTRL_ASSERT_MAX                              (0x00000007U)

#define SDL_MSS_RCM_WDT2_RST_CTRL_RESETVAL                                (0x00000000U)

/* WDT3_RST_CTRL */

#define SDL_MSS_RCM_WDT3_RST_CTRL_ASSERT_MASK                             (0x00000007U)
#define SDL_MSS_RCM_WDT3_RST_CTRL_ASSERT_SHIFT                            (0x00000000U)
#define SDL_MSS_RCM_WDT3_RST_CTRL_ASSERT_RESETVAL                         (0x00000000U)
#define SDL_MSS_RCM_WDT3_RST_CTRL_ASSERT_MAX                              (0x00000007U)

#define SDL_MSS_RCM_WDT3_RST_CTRL_RESETVAL                                (0x00000000U)

/* TOP_ESM_RST_CTRL */

#define SDL_MSS_RCM_TOP_ESM_RST_CTRL_ASSERT_MASK                          (0x00000007U)
#define SDL_MSS_RCM_TOP_ESM_RST_CTRL_ASSERT_SHIFT                         (0x00000000U)
#define SDL_MSS_RCM_TOP_ESM_RST_CTRL_ASSERT_RESETVAL                      (0x00000000U)
#define SDL_MSS_RCM_TOP_ESM_RST_CTRL_ASSERT_MAX                           (0x00000007U)

#define SDL_MSS_RCM_TOP_ESM_RST_CTRL_RESETVAL                             (0x00000000U)

/* DCC0_RST_CTRL */

#define SDL_MSS_RCM_DCC0_RST_CTRL_ASSERT_MASK                             (0x00000007U)
#define SDL_MSS_RCM_DCC0_RST_CTRL_ASSERT_SHIFT                            (0x00000000U)
#define SDL_MSS_RCM_DCC0_RST_CTRL_ASSERT_RESETVAL                         (0x00000000U)
#define SDL_MSS_RCM_DCC0_RST_CTRL_ASSERT_MAX                              (0x00000007U)

#define SDL_MSS_RCM_DCC0_RST_CTRL_RESETVAL                                (0x00000000U)

/* DCC1_RST_CTRL */

#define SDL_MSS_RCM_DCC1_RST_CTRL_ASSERT_MASK                             (0x00000007U)
#define SDL_MSS_RCM_DCC1_RST_CTRL_ASSERT_SHIFT                            (0x00000000U)
#define SDL_MSS_RCM_DCC1_RST_CTRL_ASSERT_RESETVAL                         (0x00000000U)
#define SDL_MSS_RCM_DCC1_RST_CTRL_ASSERT_MAX                              (0x00000007U)

#define SDL_MSS_RCM_DCC1_RST_CTRL_RESETVAL                                (0x00000000U)

/* DCC2_RST_CTRL */

#define SDL_MSS_RCM_DCC2_RST_CTRL_ASSERT_MASK                             (0x00000007U)
#define SDL_MSS_RCM_DCC2_RST_CTRL_ASSERT_SHIFT                            (0x00000000U)
#define SDL_MSS_RCM_DCC2_RST_CTRL_ASSERT_RESETVAL                         (0x00000000U)
#define SDL_MSS_RCM_DCC2_RST_CTRL_ASSERT_MAX                              (0x00000007U)

#define SDL_MSS_RCM_DCC2_RST_CTRL_RESETVAL                                (0x00000000U)

/* DCC3_RST_CTRL */

#define SDL_MSS_RCM_DCC3_RST_CTRL_ASSERT_MASK                             (0x00000007U)
#define SDL_MSS_RCM_DCC3_RST_CTRL_ASSERT_SHIFT                            (0x00000000U)
#define SDL_MSS_RCM_DCC3_RST_CTRL_ASSERT_RESETVAL                         (0x00000000U)
#define SDL_MSS_RCM_DCC3_RST_CTRL_ASSERT_MAX                              (0x00000007U)

#define SDL_MSS_RCM_DCC3_RST_CTRL_RESETVAL                                (0x00000000U)

/* MCSPI0_RST_CTRL */

#define SDL_MSS_RCM_MCSPI0_RST_CTRL_ASSERT_MASK                           (0x00000007U)
#define SDL_MSS_RCM_MCSPI0_RST_CTRL_ASSERT_SHIFT                          (0x00000000U)
#define SDL_MSS_RCM_MCSPI0_RST_CTRL_ASSERT_RESETVAL                       (0x00000000U)
#define SDL_MSS_RCM_MCSPI0_RST_CTRL_ASSERT_MAX                            (0x00000007U)

#define SDL_MSS_RCM_MCSPI0_RST_CTRL_RESETVAL                              (0x00000000U)

/* MCSPI1_RST_CTRL */

#define SDL_MSS_RCM_MCSPI1_RST_CTRL_ASSERT_MASK                           (0x00000007U)
#define SDL_MSS_RCM_MCSPI1_RST_CTRL_ASSERT_SHIFT                          (0x00000000U)
#define SDL_MSS_RCM_MCSPI1_RST_CTRL_ASSERT_RESETVAL                       (0x00000000U)
#define SDL_MSS_RCM_MCSPI1_RST_CTRL_ASSERT_MAX                            (0x00000007U)

#define SDL_MSS_RCM_MCSPI1_RST_CTRL_RESETVAL                              (0x00000000U)

/* MCSPI2_RST_CTRL */

#define SDL_MSS_RCM_MCSPI2_RST_CTRL_ASSERT_MASK                           (0x00000007U)
#define SDL_MSS_RCM_MCSPI2_RST_CTRL_ASSERT_SHIFT                          (0x00000000U)
#define SDL_MSS_RCM_MCSPI2_RST_CTRL_ASSERT_RESETVAL                       (0x00000000U)
#define SDL_MSS_RCM_MCSPI2_RST_CTRL_ASSERT_MAX                            (0x00000007U)

#define SDL_MSS_RCM_MCSPI2_RST_CTRL_RESETVAL                              (0x00000000U)

/* MCSPI3_RST_CTRL */

#define SDL_MSS_RCM_MCSPI3_RST_CTRL_ASSERT_MASK                           (0x00000007U)
#define SDL_MSS_RCM_MCSPI3_RST_CTRL_ASSERT_SHIFT                          (0x00000000U)
#define SDL_MSS_RCM_MCSPI3_RST_CTRL_ASSERT_RESETVAL                       (0x00000000U)
#define SDL_MSS_RCM_MCSPI3_RST_CTRL_ASSERT_MAX                            (0x00000007U)

#define SDL_MSS_RCM_MCSPI3_RST_CTRL_RESETVAL                              (0x00000000U)

/* MCSPI4_RST_CTRL */

#define SDL_MSS_RCM_MCSPI4_RST_CTRL_ASSERT_MASK                           (0x00000007U)
#define SDL_MSS_RCM_MCSPI4_RST_CTRL_ASSERT_SHIFT                          (0x00000000U)
#define SDL_MSS_RCM_MCSPI4_RST_CTRL_ASSERT_RESETVAL                       (0x00000000U)
#define SDL_MSS_RCM_MCSPI4_RST_CTRL_ASSERT_MAX                            (0x00000007U)

#define SDL_MSS_RCM_MCSPI4_RST_CTRL_RESETVAL                              (0x00000000U)

/* QSPI0_RST_CTRL */

#define SDL_MSS_RCM_QSPI0_RST_CTRL_ASSERT_MASK                            (0x00000007U)
#define SDL_MSS_RCM_QSPI0_RST_CTRL_ASSERT_SHIFT                           (0x00000000U)
#define SDL_MSS_RCM_QSPI0_RST_CTRL_ASSERT_RESETVAL                        (0x00000000U)
#define SDL_MSS_RCM_QSPI0_RST_CTRL_ASSERT_MAX                             (0x00000007U)

#define SDL_MSS_RCM_QSPI0_RST_CTRL_RESETVAL                               (0x00000000U)

/* MCAN0_RST_CTRL */

#define SDL_MSS_RCM_MCAN0_RST_CTRL_ASSERT_MASK                            (0x00000007U)
#define SDL_MSS_RCM_MCAN0_RST_CTRL_ASSERT_SHIFT                           (0x00000000U)
#define SDL_MSS_RCM_MCAN0_RST_CTRL_ASSERT_RESETVAL                        (0x00000000U)
#define SDL_MSS_RCM_MCAN0_RST_CTRL_ASSERT_MAX                             (0x00000007U)

#define SDL_MSS_RCM_MCAN0_RST_CTRL_RESETVAL                               (0x00000000U)

/* MCAN1_RST_CTRL */

#define SDL_MSS_RCM_MCAN1_RST_CTRL_ASSERT_MASK                            (0x00000007U)
#define SDL_MSS_RCM_MCAN1_RST_CTRL_ASSERT_SHIFT                           (0x00000000U)
#define SDL_MSS_RCM_MCAN1_RST_CTRL_ASSERT_RESETVAL                        (0x00000000U)
#define SDL_MSS_RCM_MCAN1_RST_CTRL_ASSERT_MAX                             (0x00000007U)

#define SDL_MSS_RCM_MCAN1_RST_CTRL_RESETVAL                               (0x00000000U)

/* MCAN2_RST_CTRL */

#define SDL_MSS_RCM_MCAN2_RST_CTRL_ASSERT_MASK                            (0x00000007U)
#define SDL_MSS_RCM_MCAN2_RST_CTRL_ASSERT_SHIFT                           (0x00000000U)
#define SDL_MSS_RCM_MCAN2_RST_CTRL_ASSERT_RESETVAL                        (0x00000000U)
#define SDL_MSS_RCM_MCAN2_RST_CTRL_ASSERT_MAX                             (0x00000007U)

#define SDL_MSS_RCM_MCAN2_RST_CTRL_RESETVAL                               (0x00000000U)

/* MCAN3_RST_CTRL */

#define SDL_MSS_RCM_MCAN3_RST_CTRL_ASSERT_MASK                            (0x00000007U)
#define SDL_MSS_RCM_MCAN3_RST_CTRL_ASSERT_SHIFT                           (0x00000000U)
#define SDL_MSS_RCM_MCAN3_RST_CTRL_ASSERT_RESETVAL                        (0x00000000U)
#define SDL_MSS_RCM_MCAN3_RST_CTRL_ASSERT_MAX                             (0x00000007U)

#define SDL_MSS_RCM_MCAN3_RST_CTRL_RESETVAL                               (0x00000000U)

/* I2C0_RST_CTRL */

#define SDL_MSS_RCM_I2C0_RST_CTRL_ASSERT_MASK                             (0x00000007U)
#define SDL_MSS_RCM_I2C0_RST_CTRL_ASSERT_SHIFT                            (0x00000000U)
#define SDL_MSS_RCM_I2C0_RST_CTRL_ASSERT_RESETVAL                         (0x00000000U)
#define SDL_MSS_RCM_I2C0_RST_CTRL_ASSERT_MAX                              (0x00000007U)

#define SDL_MSS_RCM_I2C0_RST_CTRL_RESETVAL                                (0x00000000U)

/* I2C1_RST_CTRL */

#define SDL_MSS_RCM_I2C1_RST_CTRL_ASSERT_MASK                             (0x00000007U)
#define SDL_MSS_RCM_I2C1_RST_CTRL_ASSERT_SHIFT                            (0x00000000U)
#define SDL_MSS_RCM_I2C1_RST_CTRL_ASSERT_RESETVAL                         (0x00000000U)
#define SDL_MSS_RCM_I2C1_RST_CTRL_ASSERT_MAX                              (0x00000007U)

#define SDL_MSS_RCM_I2C1_RST_CTRL_RESETVAL                                (0x00000000U)

/* I2C2_RST_CTRL */

#define SDL_MSS_RCM_I2C2_RST_CTRL_ASSERT_MASK                             (0x00000007U)
#define SDL_MSS_RCM_I2C2_RST_CTRL_ASSERT_SHIFT                            (0x00000000U)
#define SDL_MSS_RCM_I2C2_RST_CTRL_ASSERT_RESETVAL                         (0x00000000U)
#define SDL_MSS_RCM_I2C2_RST_CTRL_ASSERT_MAX                              (0x00000007U)

#define SDL_MSS_RCM_I2C2_RST_CTRL_RESETVAL                                (0x00000000U)

/* I2C3_RST_CTRL */

#define SDL_MSS_RCM_I2C3_RST_CTRL_ASSERT_MASK                             (0x00000007U)
#define SDL_MSS_RCM_I2C3_RST_CTRL_ASSERT_SHIFT                            (0x00000000U)
#define SDL_MSS_RCM_I2C3_RST_CTRL_ASSERT_RESETVAL                         (0x00000000U)
#define SDL_MSS_RCM_I2C3_RST_CTRL_ASSERT_MAX                              (0x00000007U)

#define SDL_MSS_RCM_I2C3_RST_CTRL_RESETVAL                                (0x00000000U)

/* UART0_RST_CTRL */

#define SDL_MSS_RCM_UART0_RST_CTRL_ASSERT_MASK                            (0x00000007U)
#define SDL_MSS_RCM_UART0_RST_CTRL_ASSERT_SHIFT                           (0x00000000U)
#define SDL_MSS_RCM_UART0_RST_CTRL_ASSERT_RESETVAL                        (0x00000000U)
#define SDL_MSS_RCM_UART0_RST_CTRL_ASSERT_MAX                             (0x00000007U)

#define SDL_MSS_RCM_UART0_RST_CTRL_RESETVAL                               (0x00000000U)

/* UART1_RST_CTRL */

#define SDL_MSS_RCM_UART1_RST_CTRL_ASSERT_MASK                            (0x00000007U)
#define SDL_MSS_RCM_UART1_RST_CTRL_ASSERT_SHIFT                           (0x00000000U)
#define SDL_MSS_RCM_UART1_RST_CTRL_ASSERT_RESETVAL                        (0x00000000U)
#define SDL_MSS_RCM_UART1_RST_CTRL_ASSERT_MAX                             (0x00000007U)

#define SDL_MSS_RCM_UART1_RST_CTRL_RESETVAL                               (0x00000000U)

/* UART2_RST_CTRL */

#define SDL_MSS_RCM_UART2_RST_CTRL_ASSERT_MASK                            (0x00000007U)
#define SDL_MSS_RCM_UART2_RST_CTRL_ASSERT_SHIFT                           (0x00000000U)
#define SDL_MSS_RCM_UART2_RST_CTRL_ASSERT_RESETVAL                        (0x00000000U)
#define SDL_MSS_RCM_UART2_RST_CTRL_ASSERT_MAX                             (0x00000007U)

#define SDL_MSS_RCM_UART2_RST_CTRL_RESETVAL                               (0x00000000U)

/* UART3_RST_CTRL */

#define SDL_MSS_RCM_UART3_RST_CTRL_ASSERT_MASK                            (0x00000007U)
#define SDL_MSS_RCM_UART3_RST_CTRL_ASSERT_SHIFT                           (0x00000000U)
#define SDL_MSS_RCM_UART3_RST_CTRL_ASSERT_RESETVAL                        (0x00000000U)
#define SDL_MSS_RCM_UART3_RST_CTRL_ASSERT_MAX                             (0x00000007U)

#define SDL_MSS_RCM_UART3_RST_CTRL_RESETVAL                               (0x00000000U)

/* UART4_RST_CTRL */

#define SDL_MSS_RCM_UART4_RST_CTRL_ASSERT_MASK                            (0x00000007U)
#define SDL_MSS_RCM_UART4_RST_CTRL_ASSERT_SHIFT                           (0x00000000U)
#define SDL_MSS_RCM_UART4_RST_CTRL_ASSERT_RESETVAL                        (0x00000000U)
#define SDL_MSS_RCM_UART4_RST_CTRL_ASSERT_MAX                             (0x00000007U)

#define SDL_MSS_RCM_UART4_RST_CTRL_RESETVAL                               (0x00000000U)

/* UART5_RST_CTRL */

#define SDL_MSS_RCM_UART5_RST_CTRL_ASSERT_MASK                            (0x00000007U)
#define SDL_MSS_RCM_UART5_RST_CTRL_ASSERT_SHIFT                           (0x00000000U)
#define SDL_MSS_RCM_UART5_RST_CTRL_ASSERT_RESETVAL                        (0x00000000U)
#define SDL_MSS_RCM_UART5_RST_CTRL_ASSERT_MAX                             (0x00000007U)

#define SDL_MSS_RCM_UART5_RST_CTRL_RESETVAL                               (0x00000000U)

/* LIN0_RST_CTRL */

#define SDL_MSS_RCM_LIN0_RST_CTRL_ASSERT_MASK                             (0x00000007U)
#define SDL_MSS_RCM_LIN0_RST_CTRL_ASSERT_SHIFT                            (0x00000000U)
#define SDL_MSS_RCM_LIN0_RST_CTRL_ASSERT_RESETVAL                         (0x00000000U)
#define SDL_MSS_RCM_LIN0_RST_CTRL_ASSERT_MAX                              (0x00000007U)

#define SDL_MSS_RCM_LIN0_RST_CTRL_RESETVAL                                (0x00000000U)

/* LIN1_RST_CTRL */

#define SDL_MSS_RCM_LIN1_RST_CTRL_ASSERT_MASK                             (0x00000007U)
#define SDL_MSS_RCM_LIN1_RST_CTRL_ASSERT_SHIFT                            (0x00000000U)
#define SDL_MSS_RCM_LIN1_RST_CTRL_ASSERT_RESETVAL                         (0x00000000U)
#define SDL_MSS_RCM_LIN1_RST_CTRL_ASSERT_MAX                              (0x00000007U)

#define SDL_MSS_RCM_LIN1_RST_CTRL_RESETVAL                                (0x00000000U)

/* LIN2_RST_CTRL */

#define SDL_MSS_RCM_LIN2_RST_CTRL_ASSERT_MASK                             (0x00000007U)
#define SDL_MSS_RCM_LIN2_RST_CTRL_ASSERT_SHIFT                            (0x00000000U)
#define SDL_MSS_RCM_LIN2_RST_CTRL_ASSERT_RESETVAL                         (0x00000000U)
#define SDL_MSS_RCM_LIN2_RST_CTRL_ASSERT_MAX                              (0x00000007U)

#define SDL_MSS_RCM_LIN2_RST_CTRL_RESETVAL                                (0x00000000U)

/* LIN3_RST_CTRL */

#define SDL_MSS_RCM_LIN3_RST_CTRL_ASSERT_MASK                             (0x00000007U)
#define SDL_MSS_RCM_LIN3_RST_CTRL_ASSERT_SHIFT                            (0x00000000U)
#define SDL_MSS_RCM_LIN3_RST_CTRL_ASSERT_RESETVAL                         (0x00000000U)
#define SDL_MSS_RCM_LIN3_RST_CTRL_ASSERT_MAX                              (0x00000007U)

#define SDL_MSS_RCM_LIN3_RST_CTRL_RESETVAL                                (0x00000000U)

/* LIN4_RST_CTRL */

#define SDL_MSS_RCM_LIN4_RST_CTRL_ASSERT_MASK                             (0x00000007U)
#define SDL_MSS_RCM_LIN4_RST_CTRL_ASSERT_SHIFT                            (0x00000000U)
#define SDL_MSS_RCM_LIN4_RST_CTRL_ASSERT_RESETVAL                         (0x00000000U)
#define SDL_MSS_RCM_LIN4_RST_CTRL_ASSERT_MAX                              (0x00000007U)

#define SDL_MSS_RCM_LIN4_RST_CTRL_RESETVAL                                (0x00000000U)

/* EDMA_RST_CTRL */

#define SDL_MSS_RCM_EDMA_RST_CTRL_ASSERT_MASK                             (0x00000007U)
#define SDL_MSS_RCM_EDMA_RST_CTRL_ASSERT_SHIFT                            (0x00000000U)
#define SDL_MSS_RCM_EDMA_RST_CTRL_ASSERT_RESETVAL                         (0x00000000U)
#define SDL_MSS_RCM_EDMA_RST_CTRL_ASSERT_MAX                              (0x00000007U)

#define SDL_MSS_RCM_EDMA_RST_CTRL_TPCCA_ASSERT_MASK                       (0x00000070U)
#define SDL_MSS_RCM_EDMA_RST_CTRL_TPCCA_ASSERT_SHIFT                      (0x00000004U)
#define SDL_MSS_RCM_EDMA_RST_CTRL_TPCCA_ASSERT_RESETVAL                   (0x00000000U)
#define SDL_MSS_RCM_EDMA_RST_CTRL_TPCCA_ASSERT_MAX                        (0x00000007U)

#define SDL_MSS_RCM_EDMA_RST_CTRL_TPTCA0_ASSERT_MASK                      (0x00000700U)
#define SDL_MSS_RCM_EDMA_RST_CTRL_TPTCA0_ASSERT_SHIFT                     (0x00000008U)
#define SDL_MSS_RCM_EDMA_RST_CTRL_TPTCA0_ASSERT_RESETVAL                  (0x00000000U)
#define SDL_MSS_RCM_EDMA_RST_CTRL_TPTCA0_ASSERT_MAX                       (0x00000007U)

#define SDL_MSS_RCM_EDMA_RST_CTRL_TPTCA1_ASSERT_MASK                      (0x00007000U)
#define SDL_MSS_RCM_EDMA_RST_CTRL_TPTCA1_ASSERT_SHIFT                     (0x0000000CU)
#define SDL_MSS_RCM_EDMA_RST_CTRL_TPTCA1_ASSERT_RESETVAL                  (0x00000000U)
#define SDL_MSS_RCM_EDMA_RST_CTRL_TPTCA1_ASSERT_MAX                       (0x00000007U)

#define SDL_MSS_RCM_EDMA_RST_CTRL_RESETVAL                                (0x00000000U)

/* INFRA_RST_CTRL */

#define SDL_MSS_RCM_INFRA_RST_CTRL_ASSERT_MASK                            (0x00000007U)
#define SDL_MSS_RCM_INFRA_RST_CTRL_ASSERT_SHIFT                           (0x00000000U)
#define SDL_MSS_RCM_INFRA_RST_CTRL_ASSERT_RESETVAL                        (0x00000000U)
#define SDL_MSS_RCM_INFRA_RST_CTRL_ASSERT_MAX                             (0x00000007U)

#define SDL_MSS_RCM_INFRA_RST_CTRL_RESETVAL                               (0x00000000U)

/* CPSW_RST_CTRL */

#define SDL_MSS_RCM_CPSW_RST_CTRL_ASSERT_MASK                             (0x00000007U)
#define SDL_MSS_RCM_CPSW_RST_CTRL_ASSERT_SHIFT                            (0x00000000U)
#define SDL_MSS_RCM_CPSW_RST_CTRL_ASSERT_RESETVAL                         (0x00000000U)
#define SDL_MSS_RCM_CPSW_RST_CTRL_ASSERT_MAX                              (0x00000007U)

#define SDL_MSS_RCM_CPSW_RST_CTRL_RESETVAL                                (0x00000000U)

/* ICSSM0_RST_CTRL */

#define SDL_MSS_RCM_ICSSM0_RST_CTRL_ASSERT_MASK                           (0x00000007U)
#define SDL_MSS_RCM_ICSSM0_RST_CTRL_ASSERT_SHIFT                          (0x00000000U)
#define SDL_MSS_RCM_ICSSM0_RST_CTRL_ASSERT_RESETVAL                       (0x00000000U)
#define SDL_MSS_RCM_ICSSM0_RST_CTRL_ASSERT_MAX                            (0x00000007U)

#define SDL_MSS_RCM_ICSSM0_RST_CTRL_RESETVAL                              (0x00000000U)

/* MMC0_RST_CTRL */

#define SDL_MSS_RCM_MMC0_RST_CTRL_ASSERT_MASK                             (0x00000007U)
#define SDL_MSS_RCM_MMC0_RST_CTRL_ASSERT_SHIFT                            (0x00000000U)
#define SDL_MSS_RCM_MMC0_RST_CTRL_ASSERT_RESETVAL                         (0x00000000U)
#define SDL_MSS_RCM_MMC0_RST_CTRL_ASSERT_MAX                              (0x00000007U)

#define SDL_MSS_RCM_MMC0_RST_CTRL_RESETVAL                                (0x00000000U)

/* GPIO0_RST_CTRL */

#define SDL_MSS_RCM_GPIO0_RST_CTRL_ASSERT_MASK                            (0x00000007U)
#define SDL_MSS_RCM_GPIO0_RST_CTRL_ASSERT_SHIFT                           (0x00000000U)
#define SDL_MSS_RCM_GPIO0_RST_CTRL_ASSERT_RESETVAL                        (0x00000000U)
#define SDL_MSS_RCM_GPIO0_RST_CTRL_ASSERT_MAX                             (0x00000007U)

#define SDL_MSS_RCM_GPIO0_RST_CTRL_RESETVAL                               (0x00000000U)

/* GPIO1_RST_CTRL */

#define SDL_MSS_RCM_GPIO1_RST_CTRL_ASSERT_MASK                            (0x00000007U)
#define SDL_MSS_RCM_GPIO1_RST_CTRL_ASSERT_SHIFT                           (0x00000000U)
#define SDL_MSS_RCM_GPIO1_RST_CTRL_ASSERT_RESETVAL                        (0x00000000U)
#define SDL_MSS_RCM_GPIO1_RST_CTRL_ASSERT_MAX                             (0x00000007U)

#define SDL_MSS_RCM_GPIO1_RST_CTRL_RESETVAL                               (0x00000000U)

/* GPIO2_RST_CTRL */

#define SDL_MSS_RCM_GPIO2_RST_CTRL_ASSERT_MASK                            (0x00000007U)
#define SDL_MSS_RCM_GPIO2_RST_CTRL_ASSERT_SHIFT                           (0x00000000U)
#define SDL_MSS_RCM_GPIO2_RST_CTRL_ASSERT_RESETVAL                        (0x00000000U)
#define SDL_MSS_RCM_GPIO2_RST_CTRL_ASSERT_MAX                             (0x00000007U)

#define SDL_MSS_RCM_GPIO2_RST_CTRL_RESETVAL                               (0x00000000U)

/* GPIO3_RST_CTRL */

#define SDL_MSS_RCM_GPIO3_RST_CTRL_ASSERT_MASK                            (0x00000007U)
#define SDL_MSS_RCM_GPIO3_RST_CTRL_ASSERT_SHIFT                           (0x00000000U)
#define SDL_MSS_RCM_GPIO3_RST_CTRL_ASSERT_RESETVAL                        (0x00000000U)
#define SDL_MSS_RCM_GPIO3_RST_CTRL_ASSERT_MAX                             (0x00000007U)

#define SDL_MSS_RCM_GPIO3_RST_CTRL_RESETVAL                               (0x00000000U)

/* SPINLOCK0_RST_CTRL */

#define SDL_MSS_RCM_SPINLOCK0_RST_CTRL_ASSERT_MASK                        (0x00000007U)
#define SDL_MSS_RCM_SPINLOCK0_RST_CTRL_ASSERT_SHIFT                       (0x00000000U)
#define SDL_MSS_RCM_SPINLOCK0_RST_CTRL_ASSERT_RESETVAL                    (0x00000000U)
#define SDL_MSS_RCM_SPINLOCK0_RST_CTRL_ASSERT_MAX                         (0x00000007U)

#define SDL_MSS_RCM_SPINLOCK0_RST_CTRL_RESETVAL                           (0x00000000U)

/* GPMC_RST_CTRL */

#define SDL_MSS_RCM_GPMC_RST_CTRL_ASSERT_MASK                             (0x00000007U)
#define SDL_MSS_RCM_GPMC_RST_CTRL_ASSERT_SHIFT                            (0x00000000U)
#define SDL_MSS_RCM_GPMC_RST_CTRL_ASSERT_RESETVAL                         (0x00000000U)
#define SDL_MSS_RCM_GPMC_RST_CTRL_ASSERT_MAX                              (0x00000007U)

#define SDL_MSS_RCM_GPMC_RST_CTRL_RESETVAL                                (0x00000000U)

/* TEMPSENSE_32K_RST_CTRL */

#define SDL_MSS_RCM_TEMPSENSE_32K_RST_CTRL_ASSERT_MASK                    (0x00000007U)
#define SDL_MSS_RCM_TEMPSENSE_32K_RST_CTRL_ASSERT_SHIFT                   (0x00000000U)
#define SDL_MSS_RCM_TEMPSENSE_32K_RST_CTRL_ASSERT_RESETVAL                (0x00000000U)
#define SDL_MSS_RCM_TEMPSENSE_32K_RST_CTRL_ASSERT_MAX                     (0x00000007U)

#define SDL_MSS_RCM_TEMPSENSE_32K_RST_CTRL_RESETVAL                       (0x00000000U)

/* MSS_ELM_RST_CTRL */

#define SDL_MSS_RCM_MSS_ELM_RST_CTRL_ASSERT_MASK                          (0x00000007U)
#define SDL_MSS_RCM_MSS_ELM_RST_CTRL_ASSERT_SHIFT                         (0x00000000U)
#define SDL_MSS_RCM_MSS_ELM_RST_CTRL_ASSERT_RESETVAL                      (0x00000000U)
#define SDL_MSS_RCM_MSS_ELM_RST_CTRL_ASSERT_MAX                           (0x00000007U)

#define SDL_MSS_RCM_MSS_ELM_RST_CTRL_RESETVAL                             (0x00000000U)

/* L2OCRAM_BANK0_PD_CTRL */

#define SDL_MSS_RCM_L2OCRAM_BANK0_PD_CTRL_ISO_MASK                        (0x00000007U)
#define SDL_MSS_RCM_L2OCRAM_BANK0_PD_CTRL_ISO_SHIFT                       (0x00000000U)
#define SDL_MSS_RCM_L2OCRAM_BANK0_PD_CTRL_ISO_RESETVAL                    (0x00000000U)
#define SDL_MSS_RCM_L2OCRAM_BANK0_PD_CTRL_ISO_MAX                         (0x00000007U)

#define SDL_MSS_RCM_L2OCRAM_BANK0_PD_CTRL_AONIN_MASK                      (0x00000070U)
#define SDL_MSS_RCM_L2OCRAM_BANK0_PD_CTRL_AONIN_SHIFT                     (0x00000004U)
#define SDL_MSS_RCM_L2OCRAM_BANK0_PD_CTRL_AONIN_RESETVAL                  (0x00000007U)
#define SDL_MSS_RCM_L2OCRAM_BANK0_PD_CTRL_AONIN_MAX                       (0x00000007U)

#define SDL_MSS_RCM_L2OCRAM_BANK0_PD_CTRL_AGOODIN_MASK                    (0x00000700U)
#define SDL_MSS_RCM_L2OCRAM_BANK0_PD_CTRL_AGOODIN_SHIFT                   (0x00000008U)
#define SDL_MSS_RCM_L2OCRAM_BANK0_PD_CTRL_AGOODIN_RESETVAL                (0x00000007U)
#define SDL_MSS_RCM_L2OCRAM_BANK0_PD_CTRL_AGOODIN_MAX                     (0x00000007U)

#define SDL_MSS_RCM_L2OCRAM_BANK0_PD_CTRL_RESETVAL                        (0x00000770U)

/* L2OCRAM_BANK1_PD_CTRL */

#define SDL_MSS_RCM_L2OCRAM_BANK1_PD_CTRL_ISO_MASK                        (0x00000007U)
#define SDL_MSS_RCM_L2OCRAM_BANK1_PD_CTRL_ISO_SHIFT                       (0x00000000U)
#define SDL_MSS_RCM_L2OCRAM_BANK1_PD_CTRL_ISO_RESETVAL                    (0x00000000U)
#define SDL_MSS_RCM_L2OCRAM_BANK1_PD_CTRL_ISO_MAX                         (0x00000007U)

#define SDL_MSS_RCM_L2OCRAM_BANK1_PD_CTRL_AONIN_MASK                      (0x00000070U)
#define SDL_MSS_RCM_L2OCRAM_BANK1_PD_CTRL_AONIN_SHIFT                     (0x00000004U)
#define SDL_MSS_RCM_L2OCRAM_BANK1_PD_CTRL_AONIN_RESETVAL                  (0x00000007U)
#define SDL_MSS_RCM_L2OCRAM_BANK1_PD_CTRL_AONIN_MAX                       (0x00000007U)

#define SDL_MSS_RCM_L2OCRAM_BANK1_PD_CTRL_AGOODIN_MASK                    (0x00000700U)
#define SDL_MSS_RCM_L2OCRAM_BANK1_PD_CTRL_AGOODIN_SHIFT                   (0x00000008U)
#define SDL_MSS_RCM_L2OCRAM_BANK1_PD_CTRL_AGOODIN_RESETVAL                (0x00000007U)
#define SDL_MSS_RCM_L2OCRAM_BANK1_PD_CTRL_AGOODIN_MAX                     (0x00000007U)

#define SDL_MSS_RCM_L2OCRAM_BANK1_PD_CTRL_RESETVAL                        (0x00000770U)

/* L2OCRAM_BANK2_PD_CTRL */

#define SDL_MSS_RCM_L2OCRAM_BANK2_PD_CTRL_ISO_MASK                        (0x00000007U)
#define SDL_MSS_RCM_L2OCRAM_BANK2_PD_CTRL_ISO_SHIFT                       (0x00000000U)
#define SDL_MSS_RCM_L2OCRAM_BANK2_PD_CTRL_ISO_RESETVAL                    (0x00000000U)
#define SDL_MSS_RCM_L2OCRAM_BANK2_PD_CTRL_ISO_MAX                         (0x00000007U)

#define SDL_MSS_RCM_L2OCRAM_BANK2_PD_CTRL_AONIN_MASK                      (0x00000070U)
#define SDL_MSS_RCM_L2OCRAM_BANK2_PD_CTRL_AONIN_SHIFT                     (0x00000004U)
#define SDL_MSS_RCM_L2OCRAM_BANK2_PD_CTRL_AONIN_RESETVAL                  (0x00000007U)
#define SDL_MSS_RCM_L2OCRAM_BANK2_PD_CTRL_AONIN_MAX                       (0x00000007U)

#define SDL_MSS_RCM_L2OCRAM_BANK2_PD_CTRL_AGOODIN_MASK                    (0x00000700U)
#define SDL_MSS_RCM_L2OCRAM_BANK2_PD_CTRL_AGOODIN_SHIFT                   (0x00000008U)
#define SDL_MSS_RCM_L2OCRAM_BANK2_PD_CTRL_AGOODIN_RESETVAL                (0x00000007U)
#define SDL_MSS_RCM_L2OCRAM_BANK2_PD_CTRL_AGOODIN_MAX                     (0x00000007U)

#define SDL_MSS_RCM_L2OCRAM_BANK2_PD_CTRL_RESETVAL                        (0x00000770U)

/* L2OCRAM_BANK3_PD_CTRL */

#define SDL_MSS_RCM_L2OCRAM_BANK3_PD_CTRL_ISO_MASK                        (0x00000007U)
#define SDL_MSS_RCM_L2OCRAM_BANK3_PD_CTRL_ISO_SHIFT                       (0x00000000U)
#define SDL_MSS_RCM_L2OCRAM_BANK3_PD_CTRL_ISO_RESETVAL                    (0x00000000U)
#define SDL_MSS_RCM_L2OCRAM_BANK3_PD_CTRL_ISO_MAX                         (0x00000007U)

#define SDL_MSS_RCM_L2OCRAM_BANK3_PD_CTRL_AONIN_MASK                      (0x00000070U)
#define SDL_MSS_RCM_L2OCRAM_BANK3_PD_CTRL_AONIN_SHIFT                     (0x00000004U)
#define SDL_MSS_RCM_L2OCRAM_BANK3_PD_CTRL_AONIN_RESETVAL                  (0x00000007U)
#define SDL_MSS_RCM_L2OCRAM_BANK3_PD_CTRL_AONIN_MAX                       (0x00000007U)

#define SDL_MSS_RCM_L2OCRAM_BANK3_PD_CTRL_AGOODIN_MASK                    (0x00000700U)
#define SDL_MSS_RCM_L2OCRAM_BANK3_PD_CTRL_AGOODIN_SHIFT                   (0x00000008U)
#define SDL_MSS_RCM_L2OCRAM_BANK3_PD_CTRL_AGOODIN_RESETVAL                (0x00000007U)
#define SDL_MSS_RCM_L2OCRAM_BANK3_PD_CTRL_AGOODIN_MAX                     (0x00000007U)

#define SDL_MSS_RCM_L2OCRAM_BANK3_PD_CTRL_RESETVAL                        (0x00000770U)

/* L2OCRAM_BANK0_PD_STATUS */

#define SDL_MSS_RCM_L2OCRAM_BANK0_PD_STATUS_AONOUT_MASK                   (0x00000001U)
#define SDL_MSS_RCM_L2OCRAM_BANK0_PD_STATUS_AONOUT_SHIFT                  (0x00000000U)
#define SDL_MSS_RCM_L2OCRAM_BANK0_PD_STATUS_AONOUT_RESETVAL               (0x00000001U)
#define SDL_MSS_RCM_L2OCRAM_BANK0_PD_STATUS_AONOUT_MAX                    (0x00000001U)

#define SDL_MSS_RCM_L2OCRAM_BANK0_PD_STATUS_AGOODOUT_MASK                 (0x00000002U)
#define SDL_MSS_RCM_L2OCRAM_BANK0_PD_STATUS_AGOODOUT_SHIFT                (0x00000001U)
#define SDL_MSS_RCM_L2OCRAM_BANK0_PD_STATUS_AGOODOUT_RESETVAL             (0x00000001U)
#define SDL_MSS_RCM_L2OCRAM_BANK0_PD_STATUS_AGOODOUT_MAX                  (0x00000001U)

#define SDL_MSS_RCM_L2OCRAM_BANK0_PD_STATUS_RESETVAL                      (0x00000003U)

/* L2OCRAM_BANK1_PD_STATUS */

#define SDL_MSS_RCM_L2OCRAM_BANK1_PD_STATUS_AONOUT_MASK                   (0x00000001U)
#define SDL_MSS_RCM_L2OCRAM_BANK1_PD_STATUS_AONOUT_SHIFT                  (0x00000000U)
#define SDL_MSS_RCM_L2OCRAM_BANK1_PD_STATUS_AONOUT_RESETVAL               (0x00000001U)
#define SDL_MSS_RCM_L2OCRAM_BANK1_PD_STATUS_AONOUT_MAX                    (0x00000001U)

#define SDL_MSS_RCM_L2OCRAM_BANK1_PD_STATUS_AGOODOUT_MASK                 (0x00000002U)
#define SDL_MSS_RCM_L2OCRAM_BANK1_PD_STATUS_AGOODOUT_SHIFT                (0x00000001U)
#define SDL_MSS_RCM_L2OCRAM_BANK1_PD_STATUS_AGOODOUT_RESETVAL             (0x00000001U)
#define SDL_MSS_RCM_L2OCRAM_BANK1_PD_STATUS_AGOODOUT_MAX                  (0x00000001U)

#define SDL_MSS_RCM_L2OCRAM_BANK1_PD_STATUS_RESETVAL                      (0x00000003U)

/* L2OCRAM_BANK2_PD_STATUS */

#define SDL_MSS_RCM_L2OCRAM_BANK2_PD_STATUS_AONOUT_MASK                   (0x00000001U)
#define SDL_MSS_RCM_L2OCRAM_BANK2_PD_STATUS_AONOUT_SHIFT                  (0x00000000U)
#define SDL_MSS_RCM_L2OCRAM_BANK2_PD_STATUS_AONOUT_RESETVAL               (0x00000001U)
#define SDL_MSS_RCM_L2OCRAM_BANK2_PD_STATUS_AONOUT_MAX                    (0x00000001U)

#define SDL_MSS_RCM_L2OCRAM_BANK2_PD_STATUS_AGOODOUT_MASK                 (0x00000002U)
#define SDL_MSS_RCM_L2OCRAM_BANK2_PD_STATUS_AGOODOUT_SHIFT                (0x00000001U)
#define SDL_MSS_RCM_L2OCRAM_BANK2_PD_STATUS_AGOODOUT_RESETVAL             (0x00000001U)
#define SDL_MSS_RCM_L2OCRAM_BANK2_PD_STATUS_AGOODOUT_MAX                  (0x00000001U)

#define SDL_MSS_RCM_L2OCRAM_BANK2_PD_STATUS_RESETVAL                      (0x00000003U)

/* L2OCRAM_BANK3_PD_STATUS */

#define SDL_MSS_RCM_L2OCRAM_BANK3_PD_STATUS_AONOUT_MASK                   (0x00000001U)
#define SDL_MSS_RCM_L2OCRAM_BANK3_PD_STATUS_AONOUT_SHIFT                  (0x00000000U)
#define SDL_MSS_RCM_L2OCRAM_BANK3_PD_STATUS_AONOUT_RESETVAL               (0x00000001U)
#define SDL_MSS_RCM_L2OCRAM_BANK3_PD_STATUS_AONOUT_MAX                    (0x00000001U)

#define SDL_MSS_RCM_L2OCRAM_BANK3_PD_STATUS_AGOODOUT_MASK                 (0x00000002U)
#define SDL_MSS_RCM_L2OCRAM_BANK3_PD_STATUS_AGOODOUT_SHIFT                (0x00000001U)
#define SDL_MSS_RCM_L2OCRAM_BANK3_PD_STATUS_AGOODOUT_RESETVAL             (0x00000001U)
#define SDL_MSS_RCM_L2OCRAM_BANK3_PD_STATUS_AGOODOUT_MAX                  (0x00000001U)

#define SDL_MSS_RCM_L2OCRAM_BANK3_PD_STATUS_RESETVAL                      (0x00000003U)

/* HW_REG0 */

#define SDL_MSS_RCM_HW_REG0_HWREG_MASK                                    (0xFFFFFFFFU)
#define SDL_MSS_RCM_HW_REG0_HWREG_SHIFT                                   (0x00000000U)
#define SDL_MSS_RCM_HW_REG0_HWREG_RESETVAL                                (0x00000000U)
#define SDL_MSS_RCM_HW_REG0_HWREG_MAX                                     (0xFFFFFFFFU)

#define SDL_MSS_RCM_HW_REG0_RESETVAL                                      (0x00000000U)

/* HW_REG1 */

#define SDL_MSS_RCM_HW_REG1_HWREG_MASK                                    (0xFFFFFFFFU)
#define SDL_MSS_RCM_HW_REG1_HWREG_SHIFT                                   (0x00000000U)
#define SDL_MSS_RCM_HW_REG1_HWREG_RESETVAL                                (0x00000000U)
#define SDL_MSS_RCM_HW_REG1_HWREG_MAX                                     (0xFFFFFFFFU)

#define SDL_MSS_RCM_HW_REG1_RESETVAL                                      (0x00000000U)

/* HW_REG2 */

#define SDL_MSS_RCM_HW_REG2_HWREG_MASK                                    (0xFFFFFFFFU)
#define SDL_MSS_RCM_HW_REG2_HWREG_SHIFT                                   (0x00000000U)
#define SDL_MSS_RCM_HW_REG2_HWREG_RESETVAL                                (0x00000000U)
#define SDL_MSS_RCM_HW_REG2_HWREG_MAX                                     (0xFFFFFFFFU)

#define SDL_MSS_RCM_HW_REG2_RESETVAL                                      (0x00000000U)

/* HW_REG3 */

#define SDL_MSS_RCM_HW_REG3_HWREG_MASK                                    (0xFFFFFFFFU)
#define SDL_MSS_RCM_HW_REG3_HWREG_SHIFT                                   (0x00000000U)
#define SDL_MSS_RCM_HW_REG3_HWREG_RESETVAL                                (0x00000000U)
#define SDL_MSS_RCM_HW_REG3_HWREG_MAX                                     (0xFFFFFFFFU)

#define SDL_MSS_RCM_HW_REG3_RESETVAL                                      (0x00000000U)

/* HSM_RTIA_CLK_SRC_SEL */

#define SDL_MSS_RCM_HSM_RTIA_CLK_SRC_SEL_HSM_RTI0_CLK_SRC_SEL_MASK        (0x00000FFFU)
#define SDL_MSS_RCM_HSM_RTIA_CLK_SRC_SEL_HSM_RTI0_CLK_SRC_SEL_SHIFT       (0x00000000U)
#define SDL_MSS_RCM_HSM_RTIA_CLK_SRC_SEL_HSM_RTI0_CLK_SRC_SEL_RESETVAL    (0x00000000U)
#define SDL_MSS_RCM_HSM_RTIA_CLK_SRC_SEL_HSM_RTI0_CLK_SRC_SEL_MAX         (0x00000FFFU)

#define SDL_MSS_RCM_HSM_RTIA_CLK_SRC_SEL_RESETVAL                         (0x00000000U)

/* HSM_WDT_CLK_SRC_SEL */

#define SDL_MSS_RCM_HSM_WDT_CLK_SRC_SEL_HSM_WDT0_CLK_SRC_SEL_MASK         (0x00000FFFU)
#define SDL_MSS_RCM_HSM_WDT_CLK_SRC_SEL_HSM_WDT0_CLK_SRC_SEL_SHIFT        (0x00000000U)
#define SDL_MSS_RCM_HSM_WDT_CLK_SRC_SEL_HSM_WDT0_CLK_SRC_SEL_RESETVAL     (0x00000000U)
#define SDL_MSS_RCM_HSM_WDT_CLK_SRC_SEL_HSM_WDT0_CLK_SRC_SEL_MAX          (0x00000FFFU)

#define SDL_MSS_RCM_HSM_WDT_CLK_SRC_SEL_RESETVAL                          (0x00000000U)

/* HSM_RTC_CLK_SRC_SEL */

#define SDL_MSS_RCM_HSM_RTC_CLK_SRC_SEL_HSM_RTC0_CLK_SRC_SEL_MASK         (0x00000FFFU)
#define SDL_MSS_RCM_HSM_RTC_CLK_SRC_SEL_HSM_RTC0_CLK_SRC_SEL_SHIFT        (0x00000000U)
#define SDL_MSS_RCM_HSM_RTC_CLK_SRC_SEL_HSM_RTC0_CLK_SRC_SEL_RESETVAL     (0x00000777U)
#define SDL_MSS_RCM_HSM_RTC_CLK_SRC_SEL_HSM_RTC0_CLK_SRC_SEL_MAX          (0x00000FFFU)

#define SDL_MSS_RCM_HSM_RTC_CLK_SRC_SEL_RESETVAL                          (0x00000777U)

/* HSM_DMTA_CLK_SRC_SEL */

#define SDL_MSS_RCM_HSM_DMTA_CLK_SRC_SEL_HSM_DTM0_CLK_SRC_SEL_MASK        (0x00000FFFU)
#define SDL_MSS_RCM_HSM_DMTA_CLK_SRC_SEL_HSM_DTM0_CLK_SRC_SEL_SHIFT       (0x00000000U)
#define SDL_MSS_RCM_HSM_DMTA_CLK_SRC_SEL_HSM_DTM0_CLK_SRC_SEL_RESETVAL    (0x00000000U)
#define SDL_MSS_RCM_HSM_DMTA_CLK_SRC_SEL_HSM_DTM0_CLK_SRC_SEL_MAX         (0x00000FFFU)

#define SDL_MSS_RCM_HSM_DMTA_CLK_SRC_SEL_RESETVAL                         (0x00000000U)

/* HSM_DMTB_CLK_SRC_SEL */

#define SDL_MSS_RCM_HSM_DMTB_CLK_SRC_SEL_HSM_DTM1_CLK_SRC_SEL_MASK        (0x00000FFFU)
#define SDL_MSS_RCM_HSM_DMTB_CLK_SRC_SEL_HSM_DTM1_CLK_SRC_SEL_SHIFT       (0x00000000U)
#define SDL_MSS_RCM_HSM_DMTB_CLK_SRC_SEL_HSM_DTM1_CLK_SRC_SEL_RESETVAL    (0x00000000U)
#define SDL_MSS_RCM_HSM_DMTB_CLK_SRC_SEL_HSM_DTM1_CLK_SRC_SEL_MAX         (0x00000FFFU)

#define SDL_MSS_RCM_HSM_DMTB_CLK_SRC_SEL_RESETVAL                         (0x00000000U)

/* HSM_RTI_CLK_DIV_VAL */

#define SDL_MSS_RCM_HSM_RTI_CLK_DIV_VAL_CLKDIVR_MASK                      (0x00000FFFU)
#define SDL_MSS_RCM_HSM_RTI_CLK_DIV_VAL_CLKDIVR_SHIFT                     (0x00000000U)
#define SDL_MSS_RCM_HSM_RTI_CLK_DIV_VAL_CLKDIVR_RESETVAL                  (0x00000000U)
#define SDL_MSS_RCM_HSM_RTI_CLK_DIV_VAL_CLKDIVR_MAX                       (0x00000FFFU)

#define SDL_MSS_RCM_HSM_RTI_CLK_DIV_VAL_RESETVAL                          (0x00000000U)

/* HSM_WDT_CLK_DIV_VAL */

#define SDL_MSS_RCM_HSM_WDT_CLK_DIV_VAL_CLKDIVR_MASK                      (0x00000FFFU)
#define SDL_MSS_RCM_HSM_WDT_CLK_DIV_VAL_CLKDIVR_SHIFT                     (0x00000000U)
#define SDL_MSS_RCM_HSM_WDT_CLK_DIV_VAL_CLKDIVR_RESETVAL                  (0x00000000U)
#define SDL_MSS_RCM_HSM_WDT_CLK_DIV_VAL_CLKDIVR_MAX                       (0x00000FFFU)

#define SDL_MSS_RCM_HSM_WDT_CLK_DIV_VAL_RESETVAL                          (0x00000000U)

/* HSM_RTC_CLK_DIV_VAL */

#define SDL_MSS_RCM_HSM_RTC_CLK_DIV_VAL_CLKDIVR_MASK                      (0x00000FFFU)
#define SDL_MSS_RCM_HSM_RTC_CLK_DIV_VAL_CLKDIVR_SHIFT                     (0x00000000U)
#define SDL_MSS_RCM_HSM_RTC_CLK_DIV_VAL_CLKDIVR_RESETVAL                  (0x00000000U)
#define SDL_MSS_RCM_HSM_RTC_CLK_DIV_VAL_CLKDIVR_MAX                       (0x00000FFFU)

#define SDL_MSS_RCM_HSM_RTC_CLK_DIV_VAL_RESETVAL                          (0x00000000U)

/* HSM_DMTA_CLK_DIV_VAL */

#define SDL_MSS_RCM_HSM_DMTA_CLK_DIV_VAL_CLKDIVR_MASK                     (0x00000FFFU)
#define SDL_MSS_RCM_HSM_DMTA_CLK_DIV_VAL_CLKDIVR_SHIFT                    (0x00000000U)
#define SDL_MSS_RCM_HSM_DMTA_CLK_DIV_VAL_CLKDIVR_RESETVAL                 (0x00000000U)
#define SDL_MSS_RCM_HSM_DMTA_CLK_DIV_VAL_CLKDIVR_MAX                      (0x00000FFFU)

#define SDL_MSS_RCM_HSM_DMTA_CLK_DIV_VAL_RESETVAL                         (0x00000000U)

/* HSM_DMTB_CLK_DIV_VAL */

#define SDL_MSS_RCM_HSM_DMTB_CLK_DIV_VAL_CLKDIVR_MASK                     (0x00000FFFU)
#define SDL_MSS_RCM_HSM_DMTB_CLK_DIV_VAL_CLKDIVR_SHIFT                    (0x00000000U)
#define SDL_MSS_RCM_HSM_DMTB_CLK_DIV_VAL_CLKDIVR_RESETVAL                 (0x00000000U)
#define SDL_MSS_RCM_HSM_DMTB_CLK_DIV_VAL_CLKDIVR_MAX                      (0x00000FFFU)

#define SDL_MSS_RCM_HSM_DMTB_CLK_DIV_VAL_RESETVAL                         (0x00000000U)

/* HSM_RTI_CLK_GATE */

#define SDL_MSS_RCM_HSM_RTI_CLK_GATE_GATED_MASK                           (0x00000007U)
#define SDL_MSS_RCM_HSM_RTI_CLK_GATE_GATED_SHIFT                          (0x00000000U)
#define SDL_MSS_RCM_HSM_RTI_CLK_GATE_GATED_RESETVAL                       (0x00000000U)
#define SDL_MSS_RCM_HSM_RTI_CLK_GATE_GATED_MAX                            (0x00000007U)

#define SDL_MSS_RCM_HSM_RTI_CLK_GATE_RESETVAL                             (0x00000000U)

/* HSM_WDT_CLK_GATE */

#define SDL_MSS_RCM_HSM_WDT_CLK_GATE_GATED_MASK                           (0x00000007U)
#define SDL_MSS_RCM_HSM_WDT_CLK_GATE_GATED_SHIFT                          (0x00000000U)
#define SDL_MSS_RCM_HSM_WDT_CLK_GATE_GATED_RESETVAL                       (0x00000000U)
#define SDL_MSS_RCM_HSM_WDT_CLK_GATE_GATED_MAX                            (0x00000007U)

#define SDL_MSS_RCM_HSM_WDT_CLK_GATE_RESETVAL                             (0x00000000U)

/* HSM_RTC_CLK_GATE */

#define SDL_MSS_RCM_HSM_RTC_CLK_GATE_GATED_MASK                           (0x00000007U)
#define SDL_MSS_RCM_HSM_RTC_CLK_GATE_GATED_SHIFT                          (0x00000000U)
#define SDL_MSS_RCM_HSM_RTC_CLK_GATE_GATED_RESETVAL                       (0x00000000U)
#define SDL_MSS_RCM_HSM_RTC_CLK_GATE_GATED_MAX                            (0x00000007U)

#define SDL_MSS_RCM_HSM_RTC_CLK_GATE_RESETVAL                             (0x00000000U)

/* HSM_DMTA_CLK_GATE */

#define SDL_MSS_RCM_HSM_DMTA_CLK_GATE_GATED_MASK                          (0x00000007U)
#define SDL_MSS_RCM_HSM_DMTA_CLK_GATE_GATED_SHIFT                         (0x00000000U)
#define SDL_MSS_RCM_HSM_DMTA_CLK_GATE_GATED_RESETVAL                      (0x00000000U)
#define SDL_MSS_RCM_HSM_DMTA_CLK_GATE_GATED_MAX                           (0x00000007U)

#define SDL_MSS_RCM_HSM_DMTA_CLK_GATE_RESETVAL                            (0x00000000U)

/* HSM_DMTB_CLK_GATE */

#define SDL_MSS_RCM_HSM_DMTB_CLK_GATE_GATED_MASK                          (0x00000007U)
#define SDL_MSS_RCM_HSM_DMTB_CLK_GATE_GATED_SHIFT                         (0x00000000U)
#define SDL_MSS_RCM_HSM_DMTB_CLK_GATE_GATED_RESETVAL                      (0x00000000U)
#define SDL_MSS_RCM_HSM_DMTB_CLK_GATE_GATED_MAX                           (0x00000007U)

#define SDL_MSS_RCM_HSM_DMTB_CLK_GATE_RESETVAL                            (0x00000000U)

/* HSM_RTI_CLK_STATUS */

#define SDL_MSS_RCM_HSM_RTI_CLK_STATUS_CLKINUSE_MASK                      (0x000000FFU)
#define SDL_MSS_RCM_HSM_RTI_CLK_STATUS_CLKINUSE_SHIFT                     (0x00000000U)
#define SDL_MSS_RCM_HSM_RTI_CLK_STATUS_CLKINUSE_RESETVAL                  (0x00000001U)
#define SDL_MSS_RCM_HSM_RTI_CLK_STATUS_CLKINUSE_MAX                       (0x000000FFU)

#define SDL_MSS_RCM_HSM_RTI_CLK_STATUS_CURRDIVIDER_MASK                   (0x0000FF00U)
#define SDL_MSS_RCM_HSM_RTI_CLK_STATUS_CURRDIVIDER_SHIFT                  (0x00000008U)
#define SDL_MSS_RCM_HSM_RTI_CLK_STATUS_CURRDIVIDER_RESETVAL               (0x00000000U)
#define SDL_MSS_RCM_HSM_RTI_CLK_STATUS_CURRDIVIDER_MAX                    (0x000000FFU)

#define SDL_MSS_RCM_HSM_RTI_CLK_STATUS_RESETVAL                           (0x00000001U)

/* HSM_WDT_CLK_STATUS */

#define SDL_MSS_RCM_HSM_WDT_CLK_STATUS_CLKINUSE_MASK                      (0x000000FFU)
#define SDL_MSS_RCM_HSM_WDT_CLK_STATUS_CLKINUSE_SHIFT                     (0x00000000U)
#define SDL_MSS_RCM_HSM_WDT_CLK_STATUS_CLKINUSE_RESETVAL                  (0x00000001U)
#define SDL_MSS_RCM_HSM_WDT_CLK_STATUS_CLKINUSE_MAX                       (0x000000FFU)

#define SDL_MSS_RCM_HSM_WDT_CLK_STATUS_CURRDIVIDER_MASK                   (0x0000FF00U)
#define SDL_MSS_RCM_HSM_WDT_CLK_STATUS_CURRDIVIDER_SHIFT                  (0x00000008U)
#define SDL_MSS_RCM_HSM_WDT_CLK_STATUS_CURRDIVIDER_RESETVAL               (0x00000000U)
#define SDL_MSS_RCM_HSM_WDT_CLK_STATUS_CURRDIVIDER_MAX                    (0x000000FFU)

#define SDL_MSS_RCM_HSM_WDT_CLK_STATUS_RESETVAL                           (0x00000001U)

/* HSM_RTC_CLK_STATUS */

#define SDL_MSS_RCM_HSM_RTC_CLK_STATUS_CLKINUSE_MASK                      (0x000000FFU)
#define SDL_MSS_RCM_HSM_RTC_CLK_STATUS_CLKINUSE_SHIFT                     (0x00000000U)
#define SDL_MSS_RCM_HSM_RTC_CLK_STATUS_CLKINUSE_RESETVAL                  (0x00000080U)
#define SDL_MSS_RCM_HSM_RTC_CLK_STATUS_CLKINUSE_MAX                       (0x000000FFU)

#define SDL_MSS_RCM_HSM_RTC_CLK_STATUS_CURRDIVIDER_MASK                   (0x0000FF00U)
#define SDL_MSS_RCM_HSM_RTC_CLK_STATUS_CURRDIVIDER_SHIFT                  (0x00000008U)
#define SDL_MSS_RCM_HSM_RTC_CLK_STATUS_CURRDIVIDER_RESETVAL               (0x00000000U)
#define SDL_MSS_RCM_HSM_RTC_CLK_STATUS_CURRDIVIDER_MAX                    (0x000000FFU)

#define SDL_MSS_RCM_HSM_RTC_CLK_STATUS_RESETVAL                           (0x00000080U)

/* HSM_DMTA_CLK_STATUS */

#define SDL_MSS_RCM_HSM_DMTA_CLK_STATUS_CLKINUSE_MASK                     (0x000000FFU)
#define SDL_MSS_RCM_HSM_DMTA_CLK_STATUS_CLKINUSE_SHIFT                    (0x00000000U)
#define SDL_MSS_RCM_HSM_DMTA_CLK_STATUS_CLKINUSE_RESETVAL                 (0x00000001U)
#define SDL_MSS_RCM_HSM_DMTA_CLK_STATUS_CLKINUSE_MAX                      (0x000000FFU)

#define SDL_MSS_RCM_HSM_DMTA_CLK_STATUS_CURRDIVIDER_MASK                  (0x0000FF00U)
#define SDL_MSS_RCM_HSM_DMTA_CLK_STATUS_CURRDIVIDER_SHIFT                 (0x00000008U)
#define SDL_MSS_RCM_HSM_DMTA_CLK_STATUS_CURRDIVIDER_RESETVAL              (0x00000000U)
#define SDL_MSS_RCM_HSM_DMTA_CLK_STATUS_CURRDIVIDER_MAX                   (0x000000FFU)

#define SDL_MSS_RCM_HSM_DMTA_CLK_STATUS_RESETVAL                          (0x00000001U)

/* HSM_DMTB_CLK_STATUS */

#define SDL_MSS_RCM_HSM_DMTB_CLK_STATUS_CLKINUSE_MASK                     (0x000000FFU)
#define SDL_MSS_RCM_HSM_DMTB_CLK_STATUS_CLKINUSE_SHIFT                    (0x00000000U)
#define SDL_MSS_RCM_HSM_DMTB_CLK_STATUS_CLKINUSE_RESETVAL                 (0x00000001U)
#define SDL_MSS_RCM_HSM_DMTB_CLK_STATUS_CLKINUSE_MAX                      (0x000000FFU)

#define SDL_MSS_RCM_HSM_DMTB_CLK_STATUS_CURRDIVIDER_MASK                  (0x0000FF00U)
#define SDL_MSS_RCM_HSM_DMTB_CLK_STATUS_CURRDIVIDER_SHIFT                 (0x00000008U)
#define SDL_MSS_RCM_HSM_DMTB_CLK_STATUS_CURRDIVIDER_RESETVAL              (0x00000000U)
#define SDL_MSS_RCM_HSM_DMTB_CLK_STATUS_CURRDIVIDER_MAX                   (0x000000FFU)

#define SDL_MSS_RCM_HSM_DMTB_CLK_STATUS_RESETVAL                          (0x00000001U)

/* HW_SPARE_RW0 */

#define SDL_MSS_RCM_HW_SPARE_RW0_HW_SPARE_RW0_MASK                        (0xFFFFFFFFU)
#define SDL_MSS_RCM_HW_SPARE_RW0_HW_SPARE_RW0_SHIFT                       (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_RW0_HW_SPARE_RW0_RESETVAL                    (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_RW0_HW_SPARE_RW0_MAX                         (0xFFFFFFFFU)

#define SDL_MSS_RCM_HW_SPARE_RW0_RESETVAL                                 (0x00000000U)

/* HW_SPARE_RW1 */

#define SDL_MSS_RCM_HW_SPARE_RW1_HW_SPARE_RW1_MASK                        (0xFFFFFFFFU)
#define SDL_MSS_RCM_HW_SPARE_RW1_HW_SPARE_RW1_SHIFT                       (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_RW1_HW_SPARE_RW1_RESETVAL                    (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_RW1_HW_SPARE_RW1_MAX                         (0xFFFFFFFFU)

#define SDL_MSS_RCM_HW_SPARE_RW1_RESETVAL                                 (0x00000000U)

/* HW_SPARE_RW2 */

#define SDL_MSS_RCM_HW_SPARE_RW2_HW_SPARE_RW2_MASK                        (0xFFFFFFFFU)
#define SDL_MSS_RCM_HW_SPARE_RW2_HW_SPARE_RW2_SHIFT                       (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_RW2_HW_SPARE_RW2_RESETVAL                    (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_RW2_HW_SPARE_RW2_MAX                         (0xFFFFFFFFU)

#define SDL_MSS_RCM_HW_SPARE_RW2_RESETVAL                                 (0x00000000U)

/* HW_SPARE_RW3 */

#define SDL_MSS_RCM_HW_SPARE_RW3_HW_SPARE_RW3_MASK                        (0xFFFFFFFFU)
#define SDL_MSS_RCM_HW_SPARE_RW3_HW_SPARE_RW3_SHIFT                       (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_RW3_HW_SPARE_RW3_RESETVAL                    (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_RW3_HW_SPARE_RW3_MAX                         (0xFFFFFFFFU)

#define SDL_MSS_RCM_HW_SPARE_RW3_RESETVAL                                 (0x00000000U)

/* HW_SPARE_RO0 */

#define SDL_MSS_RCM_HW_SPARE_RO0_HW_SPARE_RO0_MASK                        (0xFFFFFFFFU)
#define SDL_MSS_RCM_HW_SPARE_RO0_HW_SPARE_RO0_SHIFT                       (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_RO0_HW_SPARE_RO0_RESETVAL                    (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_RO0_HW_SPARE_RO0_MAX                         (0xFFFFFFFFU)

#define SDL_MSS_RCM_HW_SPARE_RO0_RESETVAL                                 (0x00000000U)

/* HW_SPARE_RO1 */

#define SDL_MSS_RCM_HW_SPARE_RO1_HW_SPARE_RO1_MASK                        (0xFFFFFFFFU)
#define SDL_MSS_RCM_HW_SPARE_RO1_HW_SPARE_RO1_SHIFT                       (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_RO1_HW_SPARE_RO1_RESETVAL                    (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_RO1_HW_SPARE_RO1_MAX                         (0xFFFFFFFFU)

#define SDL_MSS_RCM_HW_SPARE_RO1_RESETVAL                                 (0x00000000U)

/* HW_SPARE_RO2 */

#define SDL_MSS_RCM_HW_SPARE_RO2_HW_SPARE_RO2_MASK                        (0xFFFFFFFFU)
#define SDL_MSS_RCM_HW_SPARE_RO2_HW_SPARE_RO2_SHIFT                       (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_RO2_HW_SPARE_RO2_RESETVAL                    (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_RO2_HW_SPARE_RO2_MAX                         (0xFFFFFFFFU)

#define SDL_MSS_RCM_HW_SPARE_RO2_RESETVAL                                 (0x00000000U)

/* HW_SPARE_RO3 */

#define SDL_MSS_RCM_HW_SPARE_RO3_HW_SPARE_RO3_MASK                        (0xFFFFFFFFU)
#define SDL_MSS_RCM_HW_SPARE_RO3_HW_SPARE_RO3_SHIFT                       (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_RO3_HW_SPARE_RO3_RESETVAL                    (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_RO3_HW_SPARE_RO3_MAX                         (0xFFFFFFFFU)

#define SDL_MSS_RCM_HW_SPARE_RO3_RESETVAL                                 (0x00000000U)

/* HW_SPARE_WPH */

#define SDL_MSS_RCM_HW_SPARE_WPH_HW_SPARE_WPH_MASK                        (0xFFFFFFFFU)
#define SDL_MSS_RCM_HW_SPARE_WPH_HW_SPARE_WPH_SHIFT                       (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_WPH_HW_SPARE_WPH_RESETVAL                    (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_WPH_HW_SPARE_WPH_MAX                         (0xFFFFFFFFU)

#define SDL_MSS_RCM_HW_SPARE_WPH_RESETVAL                                 (0x00000000U)

/* HW_SPARE_REC */

#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC0_MASK                       (0x00000001U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC0_SHIFT                      (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC0_RESETVAL                   (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC0_MAX                        (0x00000001U)

#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC1_MASK                       (0x00000002U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC1_SHIFT                      (0x00000001U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC1_RESETVAL                   (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC1_MAX                        (0x00000001U)

#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC2_MASK                       (0x00000004U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC2_SHIFT                      (0x00000002U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC2_RESETVAL                   (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC2_MAX                        (0x00000001U)

#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC3_MASK                       (0x00000008U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC3_SHIFT                      (0x00000003U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC3_RESETVAL                   (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC3_MAX                        (0x00000001U)

#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC4_MASK                       (0x00000010U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC4_SHIFT                      (0x00000004U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC4_RESETVAL                   (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC4_MAX                        (0x00000001U)

#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC5_MASK                       (0x00000020U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC5_SHIFT                      (0x00000005U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC5_RESETVAL                   (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC5_MAX                        (0x00000001U)

#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC6_MASK                       (0x00000040U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC6_SHIFT                      (0x00000006U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC6_RESETVAL                   (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC6_MAX                        (0x00000001U)

#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC7_MASK                       (0x00000080U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC7_SHIFT                      (0x00000007U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC7_RESETVAL                   (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC7_MAX                        (0x00000001U)

#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC8_MASK                       (0x00000100U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC8_SHIFT                      (0x00000008U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC8_RESETVAL                   (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC8_MAX                        (0x00000001U)

#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC9_MASK                       (0x00000200U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC9_SHIFT                      (0x00000009U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC9_RESETVAL                   (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC9_MAX                        (0x00000001U)

#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC10_MASK                      (0x00000400U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC10_SHIFT                     (0x0000000AU)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC10_RESETVAL                  (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC10_MAX                       (0x00000001U)

#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC11_MASK                      (0x00000800U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC11_SHIFT                     (0x0000000BU)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC11_RESETVAL                  (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC11_MAX                       (0x00000001U)

#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC12_MASK                      (0x00001000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC12_SHIFT                     (0x0000000CU)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC12_RESETVAL                  (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC12_MAX                       (0x00000001U)

#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC13_MASK                      (0x00002000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC13_SHIFT                     (0x0000000DU)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC13_RESETVAL                  (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC13_MAX                       (0x00000001U)

#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC14_MASK                      (0x00004000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC14_SHIFT                     (0x0000000EU)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC14_RESETVAL                  (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC14_MAX                       (0x00000001U)

#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC15_MASK                      (0x00008000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC15_SHIFT                     (0x0000000FU)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC15_RESETVAL                  (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC15_MAX                       (0x00000001U)

#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC16_MASK                      (0x00010000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC16_SHIFT                     (0x00000010U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC16_RESETVAL                  (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC16_MAX                       (0x00000001U)

#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC17_MASK                      (0x00020000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC17_SHIFT                     (0x00000011U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC17_RESETVAL                  (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC17_MAX                       (0x00000001U)

#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC18_MASK                      (0x00040000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC18_SHIFT                     (0x00000012U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC18_RESETVAL                  (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC18_MAX                       (0x00000001U)

#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC19_MASK                      (0x00080000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC19_SHIFT                     (0x00000013U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC19_RESETVAL                  (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC19_MAX                       (0x00000001U)

#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC20_MASK                      (0x00100000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC20_SHIFT                     (0x00000014U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC20_RESETVAL                  (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC20_MAX                       (0x00000001U)

#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC21_MASK                      (0x00200000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC21_SHIFT                     (0x00000015U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC21_RESETVAL                  (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC21_MAX                       (0x00000001U)

#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC22_MASK                      (0x00400000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC22_SHIFT                     (0x00000016U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC22_RESETVAL                  (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC22_MAX                       (0x00000001U)

#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC23_MASK                      (0x00800000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC23_SHIFT                     (0x00000017U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC23_RESETVAL                  (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC23_MAX                       (0x00000001U)

#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC24_MASK                      (0x01000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC24_SHIFT                     (0x00000018U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC24_RESETVAL                  (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC24_MAX                       (0x00000001U)

#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC25_MASK                      (0x02000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC25_SHIFT                     (0x00000019U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC25_RESETVAL                  (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC25_MAX                       (0x00000001U)

#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC26_MASK                      (0x04000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC26_SHIFT                     (0x0000001AU)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC26_RESETVAL                  (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC26_MAX                       (0x00000001U)

#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC27_MASK                      (0x08000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC27_SHIFT                     (0x0000001BU)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC27_RESETVAL                  (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC27_MAX                       (0x00000001U)

#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC28_MASK                      (0x10000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC28_SHIFT                     (0x0000001CU)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC28_RESETVAL                  (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC28_MAX                       (0x00000001U)

#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC29_MASK                      (0x20000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC29_SHIFT                     (0x0000001DU)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC29_RESETVAL                  (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC29_MAX                       (0x00000001U)

#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC30_MASK                      (0x40000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC30_SHIFT                     (0x0000001EU)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC30_RESETVAL                  (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC30_MAX                       (0x00000001U)

#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC31_MASK                      (0x80000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC31_SHIFT                     (0x0000001FU)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC31_RESETVAL                  (0x00000000U)
#define SDL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC31_MAX                       (0x00000001U)

#define SDL_MSS_RCM_HW_SPARE_REC_RESETVAL                                 (0x00000000U)

/* LOCK0_KICK0 */

#define SDL_MSS_RCM_LOCK0_KICK0_LOCK0_KICK0_MASK                          (0xFFFFFFFFU)
#define SDL_MSS_RCM_LOCK0_KICK0_LOCK0_KICK0_SHIFT                         (0x00000000U)
#define SDL_MSS_RCM_LOCK0_KICK0_LOCK0_KICK0_RESETVAL                      (0x00000000U)
#define SDL_MSS_RCM_LOCK0_KICK0_LOCK0_KICK0_MAX                           (0xFFFFFFFFU)

#define SDL_MSS_RCM_LOCK0_KICK0_RESETVAL                                  (0x00000000U)

/* LOCK0_KICK1 */

#define SDL_MSS_RCM_LOCK0_KICK1_LOCK0_KICK1_MASK                          (0xFFFFFFFFU)
#define SDL_MSS_RCM_LOCK0_KICK1_LOCK0_KICK1_SHIFT                         (0x00000000U)
#define SDL_MSS_RCM_LOCK0_KICK1_LOCK0_KICK1_RESETVAL                      (0x00000000U)
#define SDL_MSS_RCM_LOCK0_KICK1_LOCK0_KICK1_MAX                           (0xFFFFFFFFU)

#define SDL_MSS_RCM_LOCK0_KICK1_RESETVAL                                  (0x00000000U)

/* INTR_RAW_STATUS */

#define SDL_MSS_RCM_INTR_RAW_STATUS_PROT_ERR_MASK                         (0x00000001U)
#define SDL_MSS_RCM_INTR_RAW_STATUS_PROT_ERR_SHIFT                        (0x00000000U)
#define SDL_MSS_RCM_INTR_RAW_STATUS_PROT_ERR_RESETVAL                     (0x00000000U)
#define SDL_MSS_RCM_INTR_RAW_STATUS_PROT_ERR_MAX                          (0x00000001U)

#define SDL_MSS_RCM_INTR_RAW_STATUS_ADDR_ERR_MASK                         (0x00000002U)
#define SDL_MSS_RCM_INTR_RAW_STATUS_ADDR_ERR_SHIFT                        (0x00000001U)
#define SDL_MSS_RCM_INTR_RAW_STATUS_ADDR_ERR_RESETVAL                     (0x00000000U)
#define SDL_MSS_RCM_INTR_RAW_STATUS_ADDR_ERR_MAX                          (0x00000001U)

#define SDL_MSS_RCM_INTR_RAW_STATUS_KICK_ERR_MASK                         (0x00000004U)
#define SDL_MSS_RCM_INTR_RAW_STATUS_KICK_ERR_SHIFT                        (0x00000002U)
#define SDL_MSS_RCM_INTR_RAW_STATUS_KICK_ERR_RESETVAL                     (0x00000000U)
#define SDL_MSS_RCM_INTR_RAW_STATUS_KICK_ERR_MAX                          (0x00000001U)

#define SDL_MSS_RCM_INTR_RAW_STATUS_PROXY_ERR_MASK                        (0x00000008U)
#define SDL_MSS_RCM_INTR_RAW_STATUS_PROXY_ERR_SHIFT                       (0x00000003U)
#define SDL_MSS_RCM_INTR_RAW_STATUS_PROXY_ERR_RESETVAL                    (0x00000000U)
#define SDL_MSS_RCM_INTR_RAW_STATUS_PROXY_ERR_MAX                         (0x00000001U)

#define SDL_MSS_RCM_INTR_RAW_STATUS_RESETVAL                              (0x00000000U)

/* INTR_ENABLED_STATUS_CLEAR */

#define SDL_MSS_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR_MASK       (0x00000001U)
#define SDL_MSS_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR_SHIFT      (0x00000000U)
#define SDL_MSS_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR_RESETVAL   (0x00000000U)
#define SDL_MSS_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR_MAX        (0x00000001U)

#define SDL_MSS_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR_MASK       (0x00000002U)
#define SDL_MSS_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR_SHIFT      (0x00000001U)
#define SDL_MSS_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR_RESETVAL   (0x00000000U)
#define SDL_MSS_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR_MAX        (0x00000001U)

#define SDL_MSS_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_KICK_ERR_MASK       (0x00000004U)
#define SDL_MSS_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_KICK_ERR_SHIFT      (0x00000002U)
#define SDL_MSS_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_KICK_ERR_RESETVAL   (0x00000000U)
#define SDL_MSS_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_KICK_ERR_MAX        (0x00000001U)

#define SDL_MSS_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROXY_ERR_MASK      (0x00000008U)
#define SDL_MSS_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROXY_ERR_SHIFT     (0x00000003U)
#define SDL_MSS_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROXY_ERR_RESETVAL  (0x00000000U)
#define SDL_MSS_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROXY_ERR_MAX       (0x00000001U)

#define SDL_MSS_RCM_INTR_ENABLED_STATUS_CLEAR_RESETVAL                    (0x00000000U)

/* INTR_ENABLE */

#define SDL_MSS_RCM_INTR_ENABLE_PROT_ERR_EN_MASK                          (0x00000001U)
#define SDL_MSS_RCM_INTR_ENABLE_PROT_ERR_EN_SHIFT                         (0x00000000U)
#define SDL_MSS_RCM_INTR_ENABLE_PROT_ERR_EN_RESETVAL                      (0x00000000U)
#define SDL_MSS_RCM_INTR_ENABLE_PROT_ERR_EN_MAX                           (0x00000001U)

#define SDL_MSS_RCM_INTR_ENABLE_ADDR_ERR_EN_MASK                          (0x00000002U)
#define SDL_MSS_RCM_INTR_ENABLE_ADDR_ERR_EN_SHIFT                         (0x00000001U)
#define SDL_MSS_RCM_INTR_ENABLE_ADDR_ERR_EN_RESETVAL                      (0x00000000U)
#define SDL_MSS_RCM_INTR_ENABLE_ADDR_ERR_EN_MAX                           (0x00000001U)

#define SDL_MSS_RCM_INTR_ENABLE_KICK_ERR_EN_MASK                          (0x00000004U)
#define SDL_MSS_RCM_INTR_ENABLE_KICK_ERR_EN_SHIFT                         (0x00000002U)
#define SDL_MSS_RCM_INTR_ENABLE_KICK_ERR_EN_RESETVAL                      (0x00000000U)
#define SDL_MSS_RCM_INTR_ENABLE_KICK_ERR_EN_MAX                           (0x00000001U)

#define SDL_MSS_RCM_INTR_ENABLE_PROXY_ERR_EN_MASK                         (0x00000008U)
#define SDL_MSS_RCM_INTR_ENABLE_PROXY_ERR_EN_SHIFT                        (0x00000003U)
#define SDL_MSS_RCM_INTR_ENABLE_PROXY_ERR_EN_RESETVAL                     (0x00000000U)
#define SDL_MSS_RCM_INTR_ENABLE_PROXY_ERR_EN_MAX                          (0x00000001U)

#define SDL_MSS_RCM_INTR_ENABLE_RESETVAL                                  (0x00000000U)

/* INTR_ENABLE_CLEAR */

#define SDL_MSS_RCM_INTR_ENABLE_CLEAR_PROT_ERR_EN_CLR_MASK                (0x00000001U)
#define SDL_MSS_RCM_INTR_ENABLE_CLEAR_PROT_ERR_EN_CLR_SHIFT               (0x00000000U)
#define SDL_MSS_RCM_INTR_ENABLE_CLEAR_PROT_ERR_EN_CLR_RESETVAL            (0x00000000U)
#define SDL_MSS_RCM_INTR_ENABLE_CLEAR_PROT_ERR_EN_CLR_MAX                 (0x00000001U)

#define SDL_MSS_RCM_INTR_ENABLE_CLEAR_ADDR_ERR_EN_CLR_MASK                (0x00000002U)
#define SDL_MSS_RCM_INTR_ENABLE_CLEAR_ADDR_ERR_EN_CLR_SHIFT               (0x00000001U)
#define SDL_MSS_RCM_INTR_ENABLE_CLEAR_ADDR_ERR_EN_CLR_RESETVAL            (0x00000000U)
#define SDL_MSS_RCM_INTR_ENABLE_CLEAR_ADDR_ERR_EN_CLR_MAX                 (0x00000001U)

#define SDL_MSS_RCM_INTR_ENABLE_CLEAR_KICK_ERR_EN_CLR_MASK                (0x00000004U)
#define SDL_MSS_RCM_INTR_ENABLE_CLEAR_KICK_ERR_EN_CLR_SHIFT               (0x00000002U)
#define SDL_MSS_RCM_INTR_ENABLE_CLEAR_KICK_ERR_EN_CLR_RESETVAL            (0x00000000U)
#define SDL_MSS_RCM_INTR_ENABLE_CLEAR_KICK_ERR_EN_CLR_MAX                 (0x00000001U)

#define SDL_MSS_RCM_INTR_ENABLE_CLEAR_PROXY_ERR_EN_CLR_MASK               (0x00000008U)
#define SDL_MSS_RCM_INTR_ENABLE_CLEAR_PROXY_ERR_EN_CLR_SHIFT              (0x00000003U)
#define SDL_MSS_RCM_INTR_ENABLE_CLEAR_PROXY_ERR_EN_CLR_RESETVAL           (0x00000000U)
#define SDL_MSS_RCM_INTR_ENABLE_CLEAR_PROXY_ERR_EN_CLR_MAX                (0x00000001U)

#define SDL_MSS_RCM_INTR_ENABLE_CLEAR_RESETVAL                            (0x00000000U)

/* EOI */

#define SDL_MSS_RCM_EOI_EOI_VECTOR_MASK                                   (0x000000FFU)
#define SDL_MSS_RCM_EOI_EOI_VECTOR_SHIFT                                  (0x00000000U)
#define SDL_MSS_RCM_EOI_EOI_VECTOR_RESETVAL                               (0x00000000U)
#define SDL_MSS_RCM_EOI_EOI_VECTOR_MAX                                    (0x000000FFU)

#define SDL_MSS_RCM_EOI_RESETVAL                                          (0x00000000U)

/* FAULT_ADDRESS */

#define SDL_MSS_RCM_FAULT_ADDRESS_FAULT_ADDR_MASK                         (0xFFFFFFFFU)
#define SDL_MSS_RCM_FAULT_ADDRESS_FAULT_ADDR_SHIFT                        (0x00000000U)
#define SDL_MSS_RCM_FAULT_ADDRESS_FAULT_ADDR_RESETVAL                     (0x00000000U)
#define SDL_MSS_RCM_FAULT_ADDRESS_FAULT_ADDR_MAX                          (0xFFFFFFFFU)

#define SDL_MSS_RCM_FAULT_ADDRESS_RESETVAL                                (0x00000000U)

/* FAULT_TYPE_STATUS */

#define SDL_MSS_RCM_FAULT_TYPE_STATUS_FAULT_TYPE_MASK                     (0x0000003FU)
#define SDL_MSS_RCM_FAULT_TYPE_STATUS_FAULT_TYPE_SHIFT                    (0x00000000U)
#define SDL_MSS_RCM_FAULT_TYPE_STATUS_FAULT_TYPE_RESETVAL                 (0x00000000U)
#define SDL_MSS_RCM_FAULT_TYPE_STATUS_FAULT_TYPE_MAX                      (0x0000003FU)

#define SDL_MSS_RCM_FAULT_TYPE_STATUS_FAULT_NS_MASK                       (0x00000040U)
#define SDL_MSS_RCM_FAULT_TYPE_STATUS_FAULT_NS_SHIFT                      (0x00000006U)
#define SDL_MSS_RCM_FAULT_TYPE_STATUS_FAULT_NS_RESETVAL                   (0x00000000U)
#define SDL_MSS_RCM_FAULT_TYPE_STATUS_FAULT_NS_MAX                        (0x00000001U)

#define SDL_MSS_RCM_FAULT_TYPE_STATUS_RESETVAL                            (0x00000000U)

/* FAULT_ATTR_STATUS */

#define SDL_MSS_RCM_FAULT_ATTR_STATUS_FAULT_PRIVID_MASK                   (0x000000FFU)
#define SDL_MSS_RCM_FAULT_ATTR_STATUS_FAULT_PRIVID_SHIFT                  (0x00000000U)
#define SDL_MSS_RCM_FAULT_ATTR_STATUS_FAULT_PRIVID_RESETVAL               (0x00000000U)
#define SDL_MSS_RCM_FAULT_ATTR_STATUS_FAULT_PRIVID_MAX                    (0x000000FFU)

#define SDL_MSS_RCM_FAULT_ATTR_STATUS_FAULT_ROUTEID_MASK                  (0x000FFF00U)
#define SDL_MSS_RCM_FAULT_ATTR_STATUS_FAULT_ROUTEID_SHIFT                 (0x00000008U)
#define SDL_MSS_RCM_FAULT_ATTR_STATUS_FAULT_ROUTEID_RESETVAL              (0x00000000U)
#define SDL_MSS_RCM_FAULT_ATTR_STATUS_FAULT_ROUTEID_MAX                   (0x00000FFFU)

#define SDL_MSS_RCM_FAULT_ATTR_STATUS_FAULT_XID_MASK                      (0xFFF00000U)
#define SDL_MSS_RCM_FAULT_ATTR_STATUS_FAULT_XID_SHIFT                     (0x00000014U)
#define SDL_MSS_RCM_FAULT_ATTR_STATUS_FAULT_XID_RESETVAL                  (0x00000000U)
#define SDL_MSS_RCM_FAULT_ATTR_STATUS_FAULT_XID_MAX                       (0x00000FFFU)

#define SDL_MSS_RCM_FAULT_ATTR_STATUS_RESETVAL                            (0x00000000U)

/* FAULT_CLEAR */

#define SDL_MSS_RCM_FAULT_CLEAR_FAULT_CLR_MASK                            (0x00000001U)
#define SDL_MSS_RCM_FAULT_CLEAR_FAULT_CLR_SHIFT                           (0x00000000U)
#define SDL_MSS_RCM_FAULT_CLEAR_FAULT_CLR_RESETVAL                        (0x00000000U)
#define SDL_MSS_RCM_FAULT_CLEAR_FAULT_CLR_MAX                             (0x00000001U)

#define SDL_MSS_RCM_FAULT_CLEAR_RESETVAL                                  (0x00000000U)

#ifdef __cplusplus
}
#endif
#endif
