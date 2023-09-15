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
 *  Name        : cslr_mss_rcm.h
*/
#ifndef CSLR_MSS_RCM_H_
#define CSLR_MSS_RCM_H_

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
    volatile uint32_t PID;
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
    volatile uint8_t  Resv_128[56];
    volatile uint32_t CPSW_5_50_250_CLK_MUX_CTRL;
    volatile uint8_t  Resv_256[124];
    volatile uint32_t MCAN0_CLK_SRC_SEL;
    volatile uint32_t MCAN1_CLK_SRC_SEL;
    volatile uint32_t MCAN2_CLK_SRC_SEL;
    volatile uint32_t MCAN3_CLK_SRC_SEL;
    volatile uint32_t OSPI0_CLK_SRC_SEL;
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
    volatile uint8_t  Resv_352[4];
    volatile uint32_t CONTROLSS_PLL_CLK_SRC_SEL;
    volatile uint32_t I2C_CLK_SRC_SEL;
    volatile uint8_t  Resv_372[12];
    volatile uint32_t LIN0_UART0_CLK_SRC_SEL;
    volatile uint32_t LIN1_UART1_CLK_SRC_SEL;
    volatile uint32_t LIN2_UART2_CLK_SRC_SEL;
    volatile uint32_t LIN3_UART3_CLK_SRC_SEL;
    volatile uint32_t LIN4_UART4_CLK_SRC_SEL;
    volatile uint32_t LIN5_UART5_CLK_SRC_SEL;
    volatile uint32_t MCAN4_CLK_SRC_SEL;
    volatile uint32_t MCAN5_CLK_SRC_SEL;
    volatile uint32_t MCAN6_CLK_SRC_SEL;
    volatile uint32_t MCAN7_CLK_SRC_SEL;
    volatile uint32_t RTI4_CLK_SRC_SEL;
    volatile uint32_t RTI5_CLK_SRC_SEL;
    volatile uint32_t RTI6_CLK_SRC_SEL;
    volatile uint32_t RTI7_CLK_SRC_SEL;
    volatile uint32_t MCSPI5_CLK_SRC_SEL;
    volatile uint32_t MCSPI6_CLK_SRC_SEL;
    volatile uint32_t MCSPI7_CLK_SRC_SEL;
    volatile uint8_t  Resv_512[72];
    volatile uint32_t MCAN0_CLK_DIV_VAL;
    volatile uint32_t MCAN1_CLK_DIV_VAL;
    volatile uint32_t MCAN2_CLK_DIV_VAL;
    volatile uint32_t MCAN3_CLK_DIV_VAL;
    volatile uint32_t OSPI0_CLK_DIV_VAL;
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
    volatile uint8_t  Resv_608[4];
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
    volatile uint8_t  Resv_676[4];
    volatile uint32_t MCAN4_CLK_DIV_VAL;
    volatile uint32_t MCAN5_CLK_DIV_VAL;
    volatile uint32_t MCAN6_CLK_DIV_VAL;
    volatile uint32_t MCAN7_CLK_DIV_VAL;
    volatile uint32_t RTI4_CLK_DIV_VAL;
    volatile uint32_t RTI5_CLK_DIV_VAL;
    volatile uint32_t RTI6_CLK_DIV_VAL;
    volatile uint32_t RTI7_CLK_DIV_VAL;
    volatile uint32_t MCSPI5_CLK_DIV_VAL;
    volatile uint32_t MCSPI6_CLK_DIV_VAL;
    volatile uint32_t MCSPI7_CLK_DIV_VAL;
    volatile uint8_t  Resv_768[48];
    volatile uint32_t MCAN0_CLK_GATE;
    volatile uint32_t MCAN1_CLK_GATE;
    volatile uint32_t MCAN2_CLK_GATE;
    volatile uint32_t MCAN3_CLK_GATE;
    volatile uint32_t OSPI_CLK_GATE;
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
    volatile uint8_t  Resv_864[4];
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
    volatile uint8_t  Resv_972[4];
    volatile uint32_t R5SS0_CORE0_GATE;
    volatile uint32_t R5SS1_CORE0_GATE;
    volatile uint32_t R5SS0_CORE1_GATE;
    volatile uint32_t R5SS1_CORE1_GATE;
    volatile uint32_t MCAN4_CLK_GATE;
    volatile uint32_t MCAN5_CLK_GATE;
    volatile uint32_t MCAN6_CLK_GATE;
    volatile uint32_t MCAN7_CLK_GATE;
    volatile uint32_t RTI4_CLK_GATE;
    volatile uint32_t RTI5_CLK_GATE;
    volatile uint32_t RTI6_CLK_GATE;
    volatile uint32_t RTI7_CLK_GATE;
    volatile uint8_t  Resv_1024[4];
    volatile uint32_t MCAN0_CLK_STATUS;
    volatile uint32_t MCAN1_CLK_STATUS;
    volatile uint32_t MCAN2_CLK_STATUS;
    volatile uint32_t MCAN3_CLK_STATUS;
    volatile uint32_t OSPI_CLK_STATUS;
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
    volatile uint8_t  Resv_1120[4];
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
    volatile uint8_t  Resv_1192[4];
    volatile uint32_t MCAN4_CLK_STATUS;
    volatile uint32_t MCAN5_CLK_STATUS;
    volatile uint32_t MCAN6_CLK_STATUS;
    volatile uint32_t MCAN7_CLK_STATUS;
    volatile uint32_t RTI4_CLK_STATUS;
    volatile uint32_t RTI5_CLK_STATUS;
    volatile uint32_t RTI6_CLK_STATUS;
    volatile uint32_t RTI7_CLK_STATUS;
    volatile uint32_t MCSPI5_CLK_STATUS;
    volatile uint32_t MCSPI6_CLK_STATUS;
    volatile uint32_t MCSPI7_CLK_STATUS;
    volatile uint8_t  Resv_1280[44];
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
    volatile uint32_t OSPI_RST_CTRL;
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
    volatile uint8_t  Resv_1540[4];
    volatile uint32_t TEMPSENSE_32K_RST_CTRL;
    volatile uint8_t  Resv_1548[4];
    volatile uint32_t MCAN4_RST_CTRL;
    volatile uint32_t MCAN5_RST_CTRL;
    volatile uint32_t MCAN6_RST_CTRL;
    volatile uint32_t MCAN7_RST_CTRL;
    volatile uint32_t RTI4_RST_CTRL;
    volatile uint32_t RTI5_RST_CTRL;
    volatile uint32_t RTI6_RST_CTRL;
    volatile uint32_t RTI7_RST_CTRL;
    volatile uint32_t MCSPI5_RST_CTRL;
    volatile uint32_t MCSPI6_RST_CTRL;
    volatile uint32_t MCSPI7_RST_CTRL;
    volatile uint8_t  Resv_1792[200];
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
    volatile uint32_t L2OCRAM_BANK4_PD_CTRL;
    volatile uint32_t L2OCRAM_BANK5_PD_CTRL;
    volatile uint32_t L2OCRAM_BANK4_PD_STATUS;
    volatile uint32_t L2OCRAM_BANK5_PD_STATUS;
    volatile uint8_t  Resv_2048[192];
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
    volatile uint8_t  Resv_2304[176];
    volatile uint32_t MCSPI5_CLK_GATE;
    volatile uint32_t MCSPI6_CLK_GATE;
    volatile uint32_t MCSPI7_CLK_GATE;
    volatile uint8_t  Resv_4048[1732];
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
} CSL_mss_rcmRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_MSS_RCM_PID                                                        (0x00000000U)
#define CSL_MSS_RCM_R5SS0_RST_STATUS                                           (0x00000010U)
#define CSL_MSS_RCM_R5SS0_RST_CAUSE_CLR                                        (0x00000014U)
#define CSL_MSS_RCM_R5SS0_DBG_RST_EN                                           (0x00000018U)
#define CSL_MSS_RCM_R5SS0_RST_ASSERDLY                                         (0x0000001CU)
#define CSL_MSS_RCM_R5SS0_RST2ASSERTDLY                                        (0x00000020U)
#define CSL_MSS_RCM_R5SS0_RST_WFICHECK                                         (0x00000024U)
#define CSL_MSS_RCM_R5SS1_RST_STATUS                                           (0x00000030U)
#define CSL_MSS_RCM_R5SS1_RST_CAUSE_CLR                                        (0x00000034U)
#define CSL_MSS_RCM_R5SS1_DBG_RST_EN                                           (0x00000038U)
#define CSL_MSS_RCM_R5SS1_RST_ASSERDLY                                         (0x0000003CU)
#define CSL_MSS_RCM_R5SS1_RST2ASSERTDLY                                        (0x00000040U)
#define CSL_MSS_RCM_R5SS1_RST_WFICHECK                                         (0x00000044U)
#define CSL_MSS_RCM_CPSW_5_50_250_CLK_MUX_CTRL                                 (0x00000080U)
#define CSL_MSS_RCM_MCAN0_CLK_SRC_SEL                                          (0x00000100U)
#define CSL_MSS_RCM_MCAN1_CLK_SRC_SEL                                          (0x00000104U)
#define CSL_MSS_RCM_MCAN2_CLK_SRC_SEL                                          (0x00000108U)
#define CSL_MSS_RCM_MCAN3_CLK_SRC_SEL                                          (0x0000010CU)
#define CSL_MSS_RCM_OSPI0_CLK_SRC_SEL                                          (0x00000110U)
#define CSL_MSS_RCM_RTI0_CLK_SRC_SEL                                           (0x00000114U)
#define CSL_MSS_RCM_RTI1_CLK_SRC_SEL                                           (0x00000118U)
#define CSL_MSS_RCM_RTI2_CLK_SRC_SEL                                           (0x0000011CU)
#define CSL_MSS_RCM_RTI3_CLK_SRC_SEL                                           (0x00000120U)
#define CSL_MSS_RCM_WDT0_CLK_SRC_SEL                                           (0x00000128U)
#define CSL_MSS_RCM_WDT1_CLK_SRC_SEL                                           (0x0000012CU)
#define CSL_MSS_RCM_WDT2_CLK_SRC_SEL                                           (0x00000130U)
#define CSL_MSS_RCM_WDT3_CLK_SRC_SEL                                           (0x00000134U)
#define CSL_MSS_RCM_MCSPI0_CLK_SRC_SEL                                         (0x0000013CU)
#define CSL_MSS_RCM_MCSPI1_CLK_SRC_SEL                                         (0x00000140U)
#define CSL_MSS_RCM_MCSPI2_CLK_SRC_SEL                                         (0x00000144U)
#define CSL_MSS_RCM_MCSPI3_CLK_SRC_SEL                                         (0x00000148U)
#define CSL_MSS_RCM_MCSPI4_CLK_SRC_SEL                                         (0x0000014CU)
#define CSL_MSS_RCM_MMC0_CLK_SRC_SEL                                           (0x00000150U)
#define CSL_MSS_RCM_ICSSM0_UART0_CLK_SRC_SEL                                   (0x00000154U)
#define CSL_MSS_RCM_CPTS_CLK_SRC_SEL                                           (0x00000158U)
#define CSL_MSS_RCM_CONTROLSS_PLL_CLK_SRC_SEL                                  (0x00000160U)
#define CSL_MSS_RCM_I2C_CLK_SRC_SEL                                            (0x00000164U)
#define CSL_MSS_RCM_LIN0_UART0_CLK_SRC_SEL                                     (0x00000174U)
#define CSL_MSS_RCM_LIN1_UART1_CLK_SRC_SEL                                     (0x00000178U)
#define CSL_MSS_RCM_LIN2_UART2_CLK_SRC_SEL                                     (0x0000017CU)
#define CSL_MSS_RCM_LIN3_UART3_CLK_SRC_SEL                                     (0x00000180U)
#define CSL_MSS_RCM_LIN4_UART4_CLK_SRC_SEL                                     (0x00000184U)
#define CSL_MSS_RCM_LIN5_UART5_CLK_SRC_SEL                                     (0x00000188U)
#define CSL_MSS_RCM_MCAN4_CLK_SRC_SEL                                          (0x0000018CU)
#define CSL_MSS_RCM_MCAN5_CLK_SRC_SEL                                          (0x00000190U)
#define CSL_MSS_RCM_MCAN6_CLK_SRC_SEL                                          (0x00000194U)
#define CSL_MSS_RCM_MCAN7_CLK_SRC_SEL                                          (0x00000198U)
#define CSL_MSS_RCM_RTI4_CLK_SRC_SEL                                           (0x0000019CU)
#define CSL_MSS_RCM_RTI5_CLK_SRC_SEL                                           (0x000001A0U)
#define CSL_MSS_RCM_RTI6_CLK_SRC_SEL                                           (0x000001A4U)
#define CSL_MSS_RCM_RTI7_CLK_SRC_SEL                                           (0x000001A8U)
#define CSL_MSS_RCM_MCSPI5_CLK_SRC_SEL                                         (0x000001ACU)
#define CSL_MSS_RCM_MCSPI6_CLK_SRC_SEL                                         (0x000001B0U)
#define CSL_MSS_RCM_MCSPI7_CLK_SRC_SEL                                         (0x000001B4U)
#define CSL_MSS_RCM_MCAN0_CLK_DIV_VAL                                          (0x00000200U)
#define CSL_MSS_RCM_MCAN1_CLK_DIV_VAL                                          (0x00000204U)
#define CSL_MSS_RCM_MCAN2_CLK_DIV_VAL                                          (0x00000208U)
#define CSL_MSS_RCM_MCAN3_CLK_DIV_VAL                                          (0x0000020CU)
#define CSL_MSS_RCM_OSPI0_CLK_DIV_VAL                                          (0x00000210U)
#define CSL_MSS_RCM_RTI0_CLK_DIV_VAL                                           (0x00000214U)
#define CSL_MSS_RCM_RTI1_CLK_DIV_VAL                                           (0x00000218U)
#define CSL_MSS_RCM_RTI2_CLK_DIV_VAL                                           (0x0000021CU)
#define CSL_MSS_RCM_RTI3_CLK_DIV_VAL                                           (0x00000220U)
#define CSL_MSS_RCM_WDT0_CLK_DIV_VAL                                           (0x00000228U)
#define CSL_MSS_RCM_WDT1_CLK_DIV_VAL                                           (0x0000022CU)
#define CSL_MSS_RCM_WDT2_CLK_DIV_VAL                                           (0x00000230U)
#define CSL_MSS_RCM_WDT3_CLK_DIV_VAL                                           (0x00000234U)
#define CSL_MSS_RCM_MCSPI0_CLK_DIV_VAL                                         (0x0000023CU)
#define CSL_MSS_RCM_MCSPI1_CLK_DIV_VAL                                         (0x00000240U)
#define CSL_MSS_RCM_MCSPI2_CLK_DIV_VAL                                         (0x00000244U)
#define CSL_MSS_RCM_MCSPI3_CLK_DIV_VAL                                         (0x00000248U)
#define CSL_MSS_RCM_MCSPI4_CLK_DIV_VAL                                         (0x0000024CU)
#define CSL_MSS_RCM_MMC0_CLK_DIV_VAL                                           (0x00000250U)
#define CSL_MSS_RCM_ICSSM0_UART_CLK_DIV_VAL                                    (0x00000254U)
#define CSL_MSS_RCM_CPTS_CLK_DIV_VAL                                           (0x00000258U)
#define CSL_MSS_RCM_CONTROLSS_PLL_CLK_DIV_VAL                                  (0x00000260U)
#define CSL_MSS_RCM_I2C_CLK_DIV_VAL                                            (0x00000264U)
#define CSL_MSS_RCM_LIN0_UART0_CLK_DIV_VAL                                     (0x00000274U)
#define CSL_MSS_RCM_LIN1_UART1_CLK_DIV_VAL                                     (0x00000278U)
#define CSL_MSS_RCM_LIN2_UART2_CLK_DIV_VAL                                     (0x0000027CU)
#define CSL_MSS_RCM_LIN3_UART3_CLK_DIV_VAL                                     (0x00000280U)
#define CSL_MSS_RCM_LIN4_UART4_CLK_DIV_VAL                                     (0x00000284U)
#define CSL_MSS_RCM_LIN5_UART5_CLK_DIV_VAL                                     (0x00000288U)
#define CSL_MSS_RCM_RGMII_250_CLK_DIV_VAL                                      (0x0000028CU)
#define CSL_MSS_RCM_RGMII_50_CLK_DIV_VAL                                       (0x00000290U)
#define CSL_MSS_RCM_RGMII_5_CLK_DIV_VAL                                        (0x00000294U)
#define CSL_MSS_RCM_XTAL_MMC_32K_CLK_DIV_VAL                                   (0x00000298U)
#define CSL_MSS_RCM_XTAL_TEMPSENSE_32K_CLK_DIV_VAL                             (0x0000029CU)
#define CSL_MSS_RCM_MCAN4_CLK_DIV_VAL                                          (0x000002A4U)
#define CSL_MSS_RCM_MCAN5_CLK_DIV_VAL                                          (0x000002A8U)
#define CSL_MSS_RCM_MCAN6_CLK_DIV_VAL                                          (0x000002ACU)
#define CSL_MSS_RCM_MCAN7_CLK_DIV_VAL                                          (0x000002B0U)
#define CSL_MSS_RCM_RTI4_CLK_DIV_VAL                                           (0x000002B4U)
#define CSL_MSS_RCM_RTI5_CLK_DIV_VAL                                           (0x000002B8U)
#define CSL_MSS_RCM_RTI6_CLK_DIV_VAL                                           (0x000002BCU)
#define CSL_MSS_RCM_RTI7_CLK_DIV_VAL                                           (0x000002C0U)
#define CSL_MSS_RCM_MCSPI5_CLK_DIV_VAL                                         (0x000002C4U)
#define CSL_MSS_RCM_MCSPI6_CLK_DIV_VAL                                         (0x000002C8U)
#define CSL_MSS_RCM_MCSPI7_CLK_DIV_VAL                                         (0x000002CCU)
#define CSL_MSS_RCM_MCAN0_CLK_GATE                                             (0x00000300U)
#define CSL_MSS_RCM_MCAN1_CLK_GATE                                             (0x00000304U)
#define CSL_MSS_RCM_MCAN2_CLK_GATE                                             (0x00000308U)
#define CSL_MSS_RCM_MCAN3_CLK_GATE                                             (0x0000030CU)
#define CSL_MSS_RCM_OSPI_CLK_GATE                                              (0x00000310U)
#define CSL_MSS_RCM_RTI0_CLK_GATE                                              (0x00000314U)
#define CSL_MSS_RCM_RTI1_CLK_GATE                                              (0x00000318U)
#define CSL_MSS_RCM_RTI2_CLK_GATE                                              (0x0000031CU)
#define CSL_MSS_RCM_RTI3_CLK_GATE                                              (0x00000320U)
#define CSL_MSS_RCM_WDT0_CLK_GATE                                              (0x00000328U)
#define CSL_MSS_RCM_WDT1_CLK_GATE                                              (0x0000032CU)
#define CSL_MSS_RCM_WDT2_CLK_GATE                                              (0x00000330U)
#define CSL_MSS_RCM_WDT3_CLK_GATE                                              (0x00000334U)
#define CSL_MSS_RCM_MCSPI0_CLK_GATE                                            (0x0000033CU)
#define CSL_MSS_RCM_MCSPI1_CLK_GATE                                            (0x00000340U)
#define CSL_MSS_RCM_MCSPI2_CLK_GATE                                            (0x00000344U)
#define CSL_MSS_RCM_MCSPI3_CLK_GATE                                            (0x00000348U)
#define CSL_MSS_RCM_MCSPI4_CLK_GATE                                            (0x0000034CU)
#define CSL_MSS_RCM_MMC0_CLK_GATE                                              (0x00000350U)
#define CSL_MSS_RCM_ICSSM0_UART_CLK_GATE                                       (0x00000354U)
#define CSL_MSS_RCM_CPTS_CLK_GATE                                              (0x00000358U)
#define CSL_MSS_RCM_CONTROLSS_PLL_CLK_GATE                                     (0x00000360U)
#define CSL_MSS_RCM_I2C0_CLK_GATE                                              (0x00000364U)
#define CSL_MSS_RCM_I2C1_CLK_GATE                                              (0x00000368U)
#define CSL_MSS_RCM_I2C2_CLK_GATE                                              (0x0000036CU)
#define CSL_MSS_RCM_I2C3_CLK_GATE                                              (0x00000370U)
#define CSL_MSS_RCM_LIN0_CLK_GATE                                              (0x00000374U)
#define CSL_MSS_RCM_LIN1_CLK_GATE                                              (0x00000378U)
#define CSL_MSS_RCM_LIN2_CLK_GATE                                              (0x0000037CU)
#define CSL_MSS_RCM_LIN3_CLK_GATE                                              (0x00000380U)
#define CSL_MSS_RCM_LIN4_CLK_GATE                                              (0x00000384U)
#define CSL_MSS_RCM_UART0_CLK_GATE                                             (0x0000038CU)
#define CSL_MSS_RCM_UART1_CLK_GATE                                             (0x00000390U)
#define CSL_MSS_RCM_UART2_CLK_GATE                                             (0x00000394U)
#define CSL_MSS_RCM_UART3_CLK_GATE                                             (0x00000398U)
#define CSL_MSS_RCM_UART4_CLK_GATE                                             (0x0000039CU)
#define CSL_MSS_RCM_UART5_CLK_GATE                                             (0x000003A0U)
#define CSL_MSS_RCM_RGMII_250_CLK_GATE                                         (0x000003A4U)
#define CSL_MSS_RCM_RGMII_50_CLK_GATE                                          (0x000003A8U)
#define CSL_MSS_RCM_RGMII_5_CLK_GATE                                           (0x000003ACU)
#define CSL_MSS_RCM_MMC0_32K_CLK_GATE                                          (0x000003B0U)
#define CSL_MSS_RCM_TEMPSENSE_32K_CLK_GATE                                     (0x000003B4U)
#define CSL_MSS_RCM_CPSW_CLK_GATE                                              (0x000003B8U)
#define CSL_MSS_RCM_ICSSM0_IEP_CLK_GATE                                        (0x000003BCU)
#define CSL_MSS_RCM_ICSSM0_CORE_CLK_GATE                                       (0x000003C0U)
#define CSL_MSS_RCM_MSS_ICSSM_SYS_CLK_GATE                                     (0x000003C4U)
#define CSL_MSS_RCM_R5SS0_CORE0_GATE                                           (0x000003CCU)
#define CSL_MSS_RCM_R5SS1_CORE0_GATE                                           (0x000003D0U)
#define CSL_MSS_RCM_R5SS0_CORE1_GATE                                           (0x000003D4U)
#define CSL_MSS_RCM_R5SS1_CORE1_GATE                                           (0x000003D8U)
#define CSL_MSS_RCM_MCAN4_CLK_GATE                                             (0x000003DCU)
#define CSL_MSS_RCM_MCAN5_CLK_GATE                                             (0x000003E0U)
#define CSL_MSS_RCM_MCAN6_CLK_GATE                                             (0x000003E4U)
#define CSL_MSS_RCM_MCAN7_CLK_GATE                                             (0x000003E8U)
#define CSL_MSS_RCM_RTI4_CLK_GATE                                              (0x000003ECU)
#define CSL_MSS_RCM_RTI5_CLK_GATE                                              (0x000003F0U)
#define CSL_MSS_RCM_RTI6_CLK_GATE                                              (0x000003F4U)
#define CSL_MSS_RCM_RTI7_CLK_GATE                                              (0x000003F8U)
#define CSL_MSS_RCM_MCAN0_CLK_STATUS                                           (0x00000400U)
#define CSL_MSS_RCM_MCAN1_CLK_STATUS                                           (0x00000404U)
#define CSL_MSS_RCM_MCAN2_CLK_STATUS                                           (0x00000408U)
#define CSL_MSS_RCM_MCAN3_CLK_STATUS                                           (0x0000040CU)
#define CSL_MSS_RCM_OSPI_CLK_STATUS                                            (0x00000410U)
#define CSL_MSS_RCM_RTI0_CLK_STATUS                                            (0x00000414U)
#define CSL_MSS_RCM_RTI1_CLK_STATUS                                            (0x00000418U)
#define CSL_MSS_RCM_RTI2_CLK_STATUS                                            (0x0000041CU)
#define CSL_MSS_RCM_RTI3_CLK_STATUS                                            (0x00000420U)
#define CSL_MSS_RCM_WDT0_CLK_STATUS                                            (0x00000428U)
#define CSL_MSS_RCM_WDT1_CLK_STATUS                                            (0x0000042CU)
#define CSL_MSS_RCM_WDT2_CLK_STATUS                                            (0x00000430U)
#define CSL_MSS_RCM_WDT3_CLK_STATUS                                            (0x00000434U)
#define CSL_MSS_RCM_MCSPI0_CLK_STATUS                                          (0x0000043CU)
#define CSL_MSS_RCM_MCSPI1_CLK_STATUS                                          (0x00000440U)
#define CSL_MSS_RCM_MCSPI2_CLK_STATUS                                          (0x00000444U)
#define CSL_MSS_RCM_MCSPI3_CLK_STATUS                                          (0x00000448U)
#define CSL_MSS_RCM_MCSPI4_CLK_STATUS                                          (0x0000044CU)
#define CSL_MSS_RCM_MMC0_CLK_STATUS                                            (0x00000450U)
#define CSL_MSS_RCM_ICSSM0_UART_CLK_STATUS                                     (0x00000454U)
#define CSL_MSS_RCM_CPTS_CLK_STATUS                                            (0x00000458U)
#define CSL_MSS_RCM_CONTROLSS_PLL_CLK_STATUS                                   (0x00000460U)
#define CSL_MSS_RCM_I2C_CLK_STATUS                                             (0x00000464U)
#define CSL_MSS_RCM_LIN0_UART0_CLK_STATUS                                      (0x00000474U)
#define CSL_MSS_RCM_LIN1_UART1_CLK_STATUS                                      (0x00000478U)
#define CSL_MSS_RCM_LIN2_UART2_CLK_STATUS                                      (0x0000047CU)
#define CSL_MSS_RCM_LIN3_UART3_CLK_STATUS                                      (0x00000480U)
#define CSL_MSS_RCM_LIN4_UART4_CLK_STATUS                                      (0x00000484U)
#define CSL_MSS_RCM_LIN5_UART5_CLK_STATUS                                      (0x00000488U)
#define CSL_MSS_RCM_RGMII_250_CLK_STATUS                                       (0x0000048CU)
#define CSL_MSS_RCM_RGMII_50_CLK_STATUS                                        (0x00000490U)
#define CSL_MSS_RCM_RGMII_5_CLK_STATUS                                         (0x00000494U)
#define CSL_MSS_RCM_MMC0_32K_CLK_STATUS                                        (0x0000049CU)
#define CSL_MSS_RCM_TEMPSENSE_32K_CLK_STATUS                                   (0x000004A0U)
#define CSL_MSS_RCM_MCAN4_CLK_STATUS                                           (0x000004A8U)
#define CSL_MSS_RCM_MCAN5_CLK_STATUS                                           (0x000004ACU)
#define CSL_MSS_RCM_MCAN6_CLK_STATUS                                           (0x000004B0U)
#define CSL_MSS_RCM_MCAN7_CLK_STATUS                                           (0x000004B4U)
#define CSL_MSS_RCM_RTI4_CLK_STATUS                                            (0x000004B8U)
#define CSL_MSS_RCM_RTI5_CLK_STATUS                                            (0x000004BCU)
#define CSL_MSS_RCM_RTI6_CLK_STATUS                                            (0x000004C0U)
#define CSL_MSS_RCM_RTI7_CLK_STATUS                                            (0x000004C4U)
#define CSL_MSS_RCM_MCSPI5_CLK_STATUS                                          (0x000004C8U)
#define CSL_MSS_RCM_MCSPI6_CLK_STATUS                                          (0x000004CCU)
#define CSL_MSS_RCM_MCSPI7_CLK_STATUS                                          (0x000004D0U)
#define CSL_MSS_RCM_R5SS0_POR_RST_CTRL                                         (0x00000500U)
#define CSL_MSS_RCM_R5SS1_POR_RST_CTRL                                         (0x00000504U)
#define CSL_MSS_RCM_R5SS0_CORE0_GRST_CTRL                                      (0x00000508U)
#define CSL_MSS_RCM_R5SS1_CORE0_GRST_CTRL                                      (0x0000050CU)
#define CSL_MSS_RCM_R5SS0_CORE1_GRST_CTRL                                      (0x00000510U)
#define CSL_MSS_RCM_R5SS1_CORE1_GRST_CTRL                                      (0x00000514U)
#define CSL_MSS_RCM_R5SS0_CORE0_LRST_CTRL                                      (0x00000518U)
#define CSL_MSS_RCM_R5SS1_CORE0_LRST_CTRL                                      (0x0000051CU)
#define CSL_MSS_RCM_R5SS0_CORE1_LRST_CTRL                                      (0x00000520U)
#define CSL_MSS_RCM_R5SS1_CORE1_LRST_CTRL                                      (0x00000524U)
#define CSL_MSS_RCM_R5SS0_VIM0_RST_CTRL                                        (0x00000528U)
#define CSL_MSS_RCM_R5SS1_VIM0_RST_CTRL                                        (0x0000052CU)
#define CSL_MSS_RCM_R5SS0_VIM1_RST_CTRL                                        (0x00000530U)
#define CSL_MSS_RCM_R5SS1_VIM1_RST_CTRL                                        (0x00000534U)
#define CSL_MSS_RCM_MCRC0_RST_CTRL                                             (0x00000538U)
#define CSL_MSS_RCM_RTI0_RST_CTRL                                              (0x0000053CU)
#define CSL_MSS_RCM_RTI1_RST_CTRL                                              (0x00000540U)
#define CSL_MSS_RCM_RTI2_RST_CTRL                                              (0x00000544U)
#define CSL_MSS_RCM_RTI3_RST_CTRL                                              (0x00000548U)
#define CSL_MSS_RCM_WDT0_RST_CTRL                                              (0x0000054CU)
#define CSL_MSS_RCM_WDT1_RST_CTRL                                              (0x00000550U)
#define CSL_MSS_RCM_WDT2_RST_CTRL                                              (0x00000554U)
#define CSL_MSS_RCM_WDT3_RST_CTRL                                              (0x00000558U)
#define CSL_MSS_RCM_TOP_ESM_RST_CTRL                                           (0x0000055CU)
#define CSL_MSS_RCM_DCC0_RST_CTRL                                              (0x00000560U)
#define CSL_MSS_RCM_DCC1_RST_CTRL                                              (0x00000564U)
#define CSL_MSS_RCM_DCC2_RST_CTRL                                              (0x00000568U)
#define CSL_MSS_RCM_DCC3_RST_CTRL                                              (0x0000056CU)
#define CSL_MSS_RCM_MCSPI0_RST_CTRL                                            (0x00000570U)
#define CSL_MSS_RCM_MCSPI1_RST_CTRL                                            (0x00000574U)
#define CSL_MSS_RCM_MCSPI2_RST_CTRL                                            (0x00000578U)
#define CSL_MSS_RCM_MCSPI3_RST_CTRL                                            (0x0000057CU)
#define CSL_MSS_RCM_MCSPI4_RST_CTRL                                            (0x00000580U)
#define CSL_MSS_RCM_OSPI_RST_CTRL                                              (0x00000584U)
#define CSL_MSS_RCM_MCAN0_RST_CTRL                                             (0x00000588U)
#define CSL_MSS_RCM_MCAN1_RST_CTRL                                             (0x0000058CU)
#define CSL_MSS_RCM_MCAN2_RST_CTRL                                             (0x00000590U)
#define CSL_MSS_RCM_MCAN3_RST_CTRL                                             (0x00000594U)
#define CSL_MSS_RCM_I2C0_RST_CTRL                                              (0x00000598U)
#define CSL_MSS_RCM_I2C1_RST_CTRL                                              (0x0000059CU)
#define CSL_MSS_RCM_I2C2_RST_CTRL                                              (0x000005A0U)
#define CSL_MSS_RCM_I2C3_RST_CTRL                                              (0x000005A4U)
#define CSL_MSS_RCM_UART0_RST_CTRL                                             (0x000005A8U)
#define CSL_MSS_RCM_UART1_RST_CTRL                                             (0x000005ACU)
#define CSL_MSS_RCM_UART2_RST_CTRL                                             (0x000005B0U)
#define CSL_MSS_RCM_UART3_RST_CTRL                                             (0x000005B4U)
#define CSL_MSS_RCM_UART4_RST_CTRL                                             (0x000005B8U)
#define CSL_MSS_RCM_UART5_RST_CTRL                                             (0x000005BCU)
#define CSL_MSS_RCM_LIN0_RST_CTRL                                              (0x000005C0U)
#define CSL_MSS_RCM_LIN1_RST_CTRL                                              (0x000005C4U)
#define CSL_MSS_RCM_LIN2_RST_CTRL                                              (0x000005C8U)
#define CSL_MSS_RCM_LIN3_RST_CTRL                                              (0x000005CCU)
#define CSL_MSS_RCM_LIN4_RST_CTRL                                              (0x000005D0U)
#define CSL_MSS_RCM_EDMA_RST_CTRL                                              (0x000005D8U)
#define CSL_MSS_RCM_INFRA_RST_CTRL                                             (0x000005DCU)
#define CSL_MSS_RCM_CPSW_RST_CTRL                                              (0x000005E0U)
#define CSL_MSS_RCM_ICSSM0_RST_CTRL                                            (0x000005E4U)
#define CSL_MSS_RCM_MMC0_RST_CTRL                                              (0x000005E8U)
#define CSL_MSS_RCM_GPIO0_RST_CTRL                                             (0x000005ECU)
#define CSL_MSS_RCM_GPIO1_RST_CTRL                                             (0x000005F0U)
#define CSL_MSS_RCM_GPIO2_RST_CTRL                                             (0x000005F4U)
#define CSL_MSS_RCM_GPIO3_RST_CTRL                                             (0x000005F8U)
#define CSL_MSS_RCM_SPINLOCK0_RST_CTRL                                         (0x000005FCU)
#define CSL_MSS_RCM_TEMPSENSE_32K_RST_CTRL                                     (0x00000604U)
#define CSL_MSS_RCM_MCAN4_RST_CTRL                                             (0x0000060CU)
#define CSL_MSS_RCM_MCAN5_RST_CTRL                                             (0x00000610U)
#define CSL_MSS_RCM_MCAN6_RST_CTRL                                             (0x00000614U)
#define CSL_MSS_RCM_MCAN7_RST_CTRL                                             (0x00000618U)
#define CSL_MSS_RCM_RTI4_RST_CTRL                                              (0x0000061CU)
#define CSL_MSS_RCM_RTI5_RST_CTRL                                              (0x00000620U)
#define CSL_MSS_RCM_RTI6_RST_CTRL                                              (0x00000624U)
#define CSL_MSS_RCM_RTI7_RST_CTRL                                              (0x00000628U)
#define CSL_MSS_RCM_MCSPI5_RST_CTRL                                            (0x0000062CU)
#define CSL_MSS_RCM_MCSPI6_RST_CTRL                                            (0x00000630U)
#define CSL_MSS_RCM_MCSPI7_RST_CTRL                                            (0x00000634U)
#define CSL_MSS_RCM_L2OCRAM_BANK0_PD_CTRL                                      (0x00000700U)
#define CSL_MSS_RCM_L2OCRAM_BANK1_PD_CTRL                                      (0x00000704U)
#define CSL_MSS_RCM_L2OCRAM_BANK2_PD_CTRL                                      (0x00000708U)
#define CSL_MSS_RCM_L2OCRAM_BANK3_PD_CTRL                                      (0x0000070CU)
#define CSL_MSS_RCM_L2OCRAM_BANK0_PD_STATUS                                    (0x00000710U)
#define CSL_MSS_RCM_L2OCRAM_BANK1_PD_STATUS                                    (0x00000714U)
#define CSL_MSS_RCM_L2OCRAM_BANK2_PD_STATUS                                    (0x00000718U)
#define CSL_MSS_RCM_L2OCRAM_BANK3_PD_STATUS                                    (0x0000071CU)
#define CSL_MSS_RCM_HW_REG0                                                    (0x00000720U)
#define CSL_MSS_RCM_HW_REG1                                                    (0x00000724U)
#define CSL_MSS_RCM_HW_REG2                                                    (0x00000728U)
#define CSL_MSS_RCM_HW_REG3                                                    (0x0000072CU)
#define CSL_MSS_RCM_L2OCRAM_BANK4_PD_CTRL                                      (0x00000730U)
#define CSL_MSS_RCM_L2OCRAM_BANK5_PD_CTRL                                      (0x00000734U)
#define CSL_MSS_RCM_L2OCRAM_BANK4_PD_STATUS                                    (0x00000738U)
#define CSL_MSS_RCM_L2OCRAM_BANK5_PD_STATUS                                    (0x0000073CU)
#define CSL_MSS_RCM_HSM_RTIA_CLK_SRC_SEL                                       (0x00000800U)
#define CSL_MSS_RCM_HSM_WDT_CLK_SRC_SEL                                        (0x00000804U)
#define CSL_MSS_RCM_HSM_RTC_CLK_SRC_SEL                                        (0x00000808U)
#define CSL_MSS_RCM_HSM_DMTA_CLK_SRC_SEL                                       (0x0000080CU)
#define CSL_MSS_RCM_HSM_DMTB_CLK_SRC_SEL                                       (0x00000810U)
#define CSL_MSS_RCM_HSM_RTI_CLK_DIV_VAL                                        (0x00000814U)
#define CSL_MSS_RCM_HSM_WDT_CLK_DIV_VAL                                        (0x00000818U)
#define CSL_MSS_RCM_HSM_RTC_CLK_DIV_VAL                                        (0x0000081CU)
#define CSL_MSS_RCM_HSM_DMTA_CLK_DIV_VAL                                       (0x00000820U)
#define CSL_MSS_RCM_HSM_DMTB_CLK_DIV_VAL                                       (0x00000824U)
#define CSL_MSS_RCM_HSM_RTI_CLK_GATE                                           (0x00000828U)
#define CSL_MSS_RCM_HSM_WDT_CLK_GATE                                           (0x0000082CU)
#define CSL_MSS_RCM_HSM_RTC_CLK_GATE                                           (0x00000830U)
#define CSL_MSS_RCM_HSM_DMTA_CLK_GATE                                          (0x00000834U)
#define CSL_MSS_RCM_HSM_DMTB_CLK_GATE                                          (0x00000838U)
#define CSL_MSS_RCM_HSM_RTI_CLK_STATUS                                         (0x0000083CU)
#define CSL_MSS_RCM_HSM_WDT_CLK_STATUS                                         (0x00000840U)
#define CSL_MSS_RCM_HSM_RTC_CLK_STATUS                                         (0x00000844U)
#define CSL_MSS_RCM_HSM_DMTA_CLK_STATUS                                        (0x00000848U)
#define CSL_MSS_RCM_HSM_DMTB_CLK_STATUS                                        (0x0000084CU)
#define CSL_MSS_RCM_MCSPI5_CLK_GATE                                            (0x00000900U)
#define CSL_MSS_RCM_MCSPI6_CLK_GATE                                            (0x00000904U)
#define CSL_MSS_RCM_MCSPI7_CLK_GATE                                            (0x00000908U)
#define CSL_MSS_RCM_HW_SPARE_RW0                                               (0x00000FD0U)
#define CSL_MSS_RCM_HW_SPARE_RW1                                               (0x00000FD4U)
#define CSL_MSS_RCM_HW_SPARE_RW2                                               (0x00000FD8U)
#define CSL_MSS_RCM_HW_SPARE_RW3                                               (0x00000FDCU)
#define CSL_MSS_RCM_HW_SPARE_RO0                                               (0x00000FE0U)
#define CSL_MSS_RCM_HW_SPARE_RO1                                               (0x00000FE4U)
#define CSL_MSS_RCM_HW_SPARE_RO2                                               (0x00000FE8U)
#define CSL_MSS_RCM_HW_SPARE_RO3                                               (0x00000FECU)
#define CSL_MSS_RCM_HW_SPARE_WPH                                               (0x00000FF0U)
#define CSL_MSS_RCM_HW_SPARE_REC                                               (0x00000FF4U)
#define CSL_MSS_RCM_LOCK0_KICK0                                                (0x00001008U)
#define CSL_MSS_RCM_LOCK0_KICK1                                                (0x0000100CU)
#define CSL_MSS_RCM_INTR_RAW_STATUS                                            (0x00001010U)
#define CSL_MSS_RCM_INTR_ENABLED_STATUS_CLEAR                                  (0x00001014U)
#define CSL_MSS_RCM_INTR_ENABLE                                                (0x00001018U)
#define CSL_MSS_RCM_INTR_ENABLE_CLEAR                                          (0x0000101CU)
#define CSL_MSS_RCM_EOI                                                        (0x00001020U)
#define CSL_MSS_RCM_FAULT_ADDRESS                                              (0x00001024U)
#define CSL_MSS_RCM_FAULT_TYPE_STATUS                                          (0x00001028U)
#define CSL_MSS_RCM_FAULT_ATTR_STATUS                                          (0x0000102CU)
#define CSL_MSS_RCM_FAULT_CLEAR                                                (0x00001030U)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* PID */

#define CSL_MSS_RCM_PID_PID_MINOR_MASK                                         (0x0000003FU)
#define CSL_MSS_RCM_PID_PID_MINOR_SHIFT                                        (0x00000000U)
#define CSL_MSS_RCM_PID_PID_MINOR_RESETVAL                                     (0x00000014U)
#define CSL_MSS_RCM_PID_PID_MINOR_MAX                                          (0x0000003FU)

#define CSL_MSS_RCM_PID_PID_CUSTOM_MASK                                        (0x000000C0U)
#define CSL_MSS_RCM_PID_PID_CUSTOM_SHIFT                                       (0x00000006U)
#define CSL_MSS_RCM_PID_PID_CUSTOM_RESETVAL                                    (0x00000000U)
#define CSL_MSS_RCM_PID_PID_CUSTOM_MAX                                         (0x00000003U)

#define CSL_MSS_RCM_PID_PID_MAJOR_MASK                                         (0x00000700U)
#define CSL_MSS_RCM_PID_PID_MAJOR_SHIFT                                        (0x00000008U)
#define CSL_MSS_RCM_PID_PID_MAJOR_RESETVAL                                     (0x00000002U)
#define CSL_MSS_RCM_PID_PID_MAJOR_MAX                                          (0x00000007U)

#define CSL_MSS_RCM_PID_PID_MISC_MASK                                          (0x0000F800U)
#define CSL_MSS_RCM_PID_PID_MISC_SHIFT                                         (0x0000000BU)
#define CSL_MSS_RCM_PID_PID_MISC_RESETVAL                                      (0x00000000U)
#define CSL_MSS_RCM_PID_PID_MISC_MAX                                           (0x0000001FU)

#define CSL_MSS_RCM_PID_PID_MSB16_MASK                                         (0xFFFF0000U)
#define CSL_MSS_RCM_PID_PID_MSB16_SHIFT                                        (0x00000010U)
#define CSL_MSS_RCM_PID_PID_MSB16_RESETVAL                                     (0x00006180U)
#define CSL_MSS_RCM_PID_PID_MSB16_MAX                                          (0x0000FFFFU)

#define CSL_MSS_RCM_PID_RESETVAL                                               (0x61800214U)

/* R5SS0_RST_STATUS */

#define CSL_MSS_RCM_R5SS0_RST_STATUS_CAUSE_MASK                                (0x000007FFU)
#define CSL_MSS_RCM_R5SS0_RST_STATUS_CAUSE_SHIFT                               (0x00000000U)
#define CSL_MSS_RCM_R5SS0_RST_STATUS_CAUSE_RESETVAL                            (0x00000003U)
#define CSL_MSS_RCM_R5SS0_RST_STATUS_CAUSE_MAX                                 (0x000007FFU)

#define CSL_MSS_RCM_R5SS0_RST_STATUS_RESETVAL                                  (0x00000003U)

/* R5SS0_RST_CAUSE_CLR */

#define CSL_MSS_RCM_R5SS0_RST_CAUSE_CLR_CLR_MASK                               (0x00000007U)
#define CSL_MSS_RCM_R5SS0_RST_CAUSE_CLR_CLR_SHIFT                              (0x00000000U)
#define CSL_MSS_RCM_R5SS0_RST_CAUSE_CLR_CLR_RESETVAL                           (0x00000000U)
#define CSL_MSS_RCM_R5SS0_RST_CAUSE_CLR_CLR_MAX                                (0x00000007U)

#define CSL_MSS_RCM_R5SS0_RST_CAUSE_CLR_RESETVAL                               (0x00000000U)

/* R5SS0_DBG_RST_EN */

#define CSL_MSS_RCM_R5SS0_DBG_RST_EN_EN_CORE0_MASK                             (0x00000007U)
#define CSL_MSS_RCM_R5SS0_DBG_RST_EN_EN_CORE0_SHIFT                            (0x00000000U)
#define CSL_MSS_RCM_R5SS0_DBG_RST_EN_EN_CORE0_RESETVAL                         (0x00000000U)
#define CSL_MSS_RCM_R5SS0_DBG_RST_EN_EN_CORE0_MAX                              (0x00000007U)

#define CSL_MSS_RCM_R5SS0_DBG_RST_EN_EN_CORE1_MASK                             (0x00070000U)
#define CSL_MSS_RCM_R5SS0_DBG_RST_EN_EN_CORE1_SHIFT                            (0x00000010U)
#define CSL_MSS_RCM_R5SS0_DBG_RST_EN_EN_CORE1_RESETVAL                         (0x00000000U)
#define CSL_MSS_RCM_R5SS0_DBG_RST_EN_EN_CORE1_MAX                              (0x00000007U)

#define CSL_MSS_RCM_R5SS0_DBG_RST_EN_RESETVAL                                  (0x00000000U)

/* R5SS0_RST_ASSERDLY */

#define CSL_MSS_RCM_R5SS0_RST_ASSERDLY_COUNT_MASK                              (0x000000FFU)
#define CSL_MSS_RCM_R5SS0_RST_ASSERDLY_COUNT_SHIFT                             (0x00000000U)
#define CSL_MSS_RCM_R5SS0_RST_ASSERDLY_COUNT_RESETVAL                          (0x0000000FU)
#define CSL_MSS_RCM_R5SS0_RST_ASSERDLY_COUNT_MAX                               (0x000000FFU)

#define CSL_MSS_RCM_R5SS0_RST_ASSERDLY_RESETVAL                                (0x0000000FU)

/* R5SS0_RST2ASSERTDLY */

#define CSL_MSS_RCM_R5SS0_RST2ASSERTDLY_R5SS_CORE0_COUNT_MASK                  (0x000000FFU)
#define CSL_MSS_RCM_R5SS0_RST2ASSERTDLY_R5SS_CORE0_COUNT_SHIFT                 (0x00000000U)
#define CSL_MSS_RCM_R5SS0_RST2ASSERTDLY_R5SS_CORE0_COUNT_RESETVAL              (0x00000000U)
#define CSL_MSS_RCM_R5SS0_RST2ASSERTDLY_R5SS_CORE0_COUNT_MAX                   (0x000000FFU)

#define CSL_MSS_RCM_R5SS0_RST2ASSERTDLY_R5SS_CORE1_COUNT_MASK                  (0x0000FF00U)
#define CSL_MSS_RCM_R5SS0_RST2ASSERTDLY_R5SS_CORE1_COUNT_SHIFT                 (0x00000008U)
#define CSL_MSS_RCM_R5SS0_RST2ASSERTDLY_R5SS_CORE1_COUNT_RESETVAL              (0x00000000U)
#define CSL_MSS_RCM_R5SS0_RST2ASSERTDLY_R5SS_CORE1_COUNT_MAX                   (0x000000FFU)

#define CSL_MSS_RCM_R5SS0_RST2ASSERTDLY_R5_CORE0_COUNT_MASK                    (0x00FF0000U)
#define CSL_MSS_RCM_R5SS0_RST2ASSERTDLY_R5_CORE0_COUNT_SHIFT                   (0x00000010U)
#define CSL_MSS_RCM_R5SS0_RST2ASSERTDLY_R5_CORE0_COUNT_RESETVAL                (0x00000000U)
#define CSL_MSS_RCM_R5SS0_RST2ASSERTDLY_R5_CORE0_COUNT_MAX                     (0x000000FFU)

#define CSL_MSS_RCM_R5SS0_RST2ASSERTDLY_R5_CORE1_COUNT_MASK                    (0xFF000000U)
#define CSL_MSS_RCM_R5SS0_RST2ASSERTDLY_R5_CORE1_COUNT_SHIFT                   (0x00000018U)
#define CSL_MSS_RCM_R5SS0_RST2ASSERTDLY_R5_CORE1_COUNT_RESETVAL                (0x00000000U)
#define CSL_MSS_RCM_R5SS0_RST2ASSERTDLY_R5_CORE1_COUNT_MAX                     (0x000000FFU)

#define CSL_MSS_RCM_R5SS0_RST2ASSERTDLY_RESETVAL                               (0x00000000U)

/* R5SS0_RST_WFICHECK */

#define CSL_MSS_RCM_R5SS0_RST_WFICHECK_EN_R5SS_CORE0_MASK                      (0x00000007U)
#define CSL_MSS_RCM_R5SS0_RST_WFICHECK_EN_R5SS_CORE0_SHIFT                     (0x00000000U)
#define CSL_MSS_RCM_R5SS0_RST_WFICHECK_EN_R5SS_CORE0_RESETVAL                  (0x00000007U)
#define CSL_MSS_RCM_R5SS0_RST_WFICHECK_EN_R5SS_CORE0_MAX                       (0x00000007U)

#define CSL_MSS_RCM_R5SS0_RST_WFICHECK_EN_R5SS_CORE1_MASK                      (0x00000700U)
#define CSL_MSS_RCM_R5SS0_RST_WFICHECK_EN_R5SS_CORE1_SHIFT                     (0x00000008U)
#define CSL_MSS_RCM_R5SS0_RST_WFICHECK_EN_R5SS_CORE1_RESETVAL                  (0x00000007U)
#define CSL_MSS_RCM_R5SS0_RST_WFICHECK_EN_R5SS_CORE1_MAX                       (0x00000007U)

#define CSL_MSS_RCM_R5SS0_RST_WFICHECK_EN_R5_CORE0_MASK                        (0x00070000U)
#define CSL_MSS_RCM_R5SS0_RST_WFICHECK_EN_R5_CORE0_SHIFT                       (0x00000010U)
#define CSL_MSS_RCM_R5SS0_RST_WFICHECK_EN_R5_CORE0_RESETVAL                    (0x00000007U)
#define CSL_MSS_RCM_R5SS0_RST_WFICHECK_EN_R5_CORE0_MAX                         (0x00000007U)

#define CSL_MSS_RCM_R5SS0_RST_WFICHECK_EN_R5_CORE1_MASK                        (0x07000000U)
#define CSL_MSS_RCM_R5SS0_RST_WFICHECK_EN_R5_CORE1_SHIFT                       (0x00000018U)
#define CSL_MSS_RCM_R5SS0_RST_WFICHECK_EN_R5_CORE1_RESETVAL                    (0x00000007U)
#define CSL_MSS_RCM_R5SS0_RST_WFICHECK_EN_R5_CORE1_MAX                         (0x00000007U)

#define CSL_MSS_RCM_R5SS0_RST_WFICHECK_RESETVAL                                (0x07070707U)

/* R5SS1_RST_STATUS */

#define CSL_MSS_RCM_R5SS1_RST_STATUS_CAUSE_MASK                                (0x000007FFU)
#define CSL_MSS_RCM_R5SS1_RST_STATUS_CAUSE_SHIFT                               (0x00000000U)
#define CSL_MSS_RCM_R5SS1_RST_STATUS_CAUSE_RESETVAL                            (0x00000003U)
#define CSL_MSS_RCM_R5SS1_RST_STATUS_CAUSE_MAX                                 (0x000007FFU)

#define CSL_MSS_RCM_R5SS1_RST_STATUS_RESETVAL                                  (0x00000003U)

/* R5SS1_RST_CAUSE_CLR */

#define CSL_MSS_RCM_R5SS1_RST_CAUSE_CLR_CLR_MASK                               (0x00000007U)
#define CSL_MSS_RCM_R5SS1_RST_CAUSE_CLR_CLR_SHIFT                              (0x00000000U)
#define CSL_MSS_RCM_R5SS1_RST_CAUSE_CLR_CLR_RESETVAL                           (0x00000000U)
#define CSL_MSS_RCM_R5SS1_RST_CAUSE_CLR_CLR_MAX                                (0x00000007U)

#define CSL_MSS_RCM_R5SS1_RST_CAUSE_CLR_RESETVAL                               (0x00000000U)

/* R5SS1_DBG_RST_EN */

#define CSL_MSS_RCM_R5SS1_DBG_RST_EN_EN_CORE0_MASK                             (0x00000007U)
#define CSL_MSS_RCM_R5SS1_DBG_RST_EN_EN_CORE0_SHIFT                            (0x00000000U)
#define CSL_MSS_RCM_R5SS1_DBG_RST_EN_EN_CORE0_RESETVAL                         (0x00000000U)
#define CSL_MSS_RCM_R5SS1_DBG_RST_EN_EN_CORE0_MAX                              (0x00000007U)

#define CSL_MSS_RCM_R5SS1_DBG_RST_EN_EN_CORE1_MASK                             (0x00070000U)
#define CSL_MSS_RCM_R5SS1_DBG_RST_EN_EN_CORE1_SHIFT                            (0x00000010U)
#define CSL_MSS_RCM_R5SS1_DBG_RST_EN_EN_CORE1_RESETVAL                         (0x00000000U)
#define CSL_MSS_RCM_R5SS1_DBG_RST_EN_EN_CORE1_MAX                              (0x00000007U)

#define CSL_MSS_RCM_R5SS1_DBG_RST_EN_RESETVAL                                  (0x00000000U)

/* R5SS1_RST_ASSERDLY */

#define CSL_MSS_RCM_R5SS1_RST_ASSERDLY_COUNT_MASK                              (0x000000FFU)
#define CSL_MSS_RCM_R5SS1_RST_ASSERDLY_COUNT_SHIFT                             (0x00000000U)
#define CSL_MSS_RCM_R5SS1_RST_ASSERDLY_COUNT_RESETVAL                          (0x0000000FU)
#define CSL_MSS_RCM_R5SS1_RST_ASSERDLY_COUNT_MAX                               (0x000000FFU)

#define CSL_MSS_RCM_R5SS1_RST_ASSERDLY_RESETVAL                                (0x0000000FU)

/* R5SS1_RST2ASSERTDLY */

#define CSL_MSS_RCM_R5SS1_RST2ASSERTDLY_R5SS_CORE0_COUNT_MASK                  (0x000000FFU)
#define CSL_MSS_RCM_R5SS1_RST2ASSERTDLY_R5SS_CORE0_COUNT_SHIFT                 (0x00000000U)
#define CSL_MSS_RCM_R5SS1_RST2ASSERTDLY_R5SS_CORE0_COUNT_RESETVAL              (0x00000000U)
#define CSL_MSS_RCM_R5SS1_RST2ASSERTDLY_R5SS_CORE0_COUNT_MAX                   (0x000000FFU)

#define CSL_MSS_RCM_R5SS1_RST2ASSERTDLY_R5SS_CORE1_COUNT_MASK                  (0x0000FF00U)
#define CSL_MSS_RCM_R5SS1_RST2ASSERTDLY_R5SS_CORE1_COUNT_SHIFT                 (0x00000008U)
#define CSL_MSS_RCM_R5SS1_RST2ASSERTDLY_R5SS_CORE1_COUNT_RESETVAL              (0x00000000U)
#define CSL_MSS_RCM_R5SS1_RST2ASSERTDLY_R5SS_CORE1_COUNT_MAX                   (0x000000FFU)

#define CSL_MSS_RCM_R5SS1_RST2ASSERTDLY_R5_CORE0_COUNT_MASK                    (0x00FF0000U)
#define CSL_MSS_RCM_R5SS1_RST2ASSERTDLY_R5_CORE0_COUNT_SHIFT                   (0x00000010U)
#define CSL_MSS_RCM_R5SS1_RST2ASSERTDLY_R5_CORE0_COUNT_RESETVAL                (0x00000000U)
#define CSL_MSS_RCM_R5SS1_RST2ASSERTDLY_R5_CORE0_COUNT_MAX                     (0x000000FFU)

#define CSL_MSS_RCM_R5SS1_RST2ASSERTDLY_R5_CORE1_COUNT_MASK                    (0xFF000000U)
#define CSL_MSS_RCM_R5SS1_RST2ASSERTDLY_R5_CORE1_COUNT_SHIFT                   (0x00000018U)
#define CSL_MSS_RCM_R5SS1_RST2ASSERTDLY_R5_CORE1_COUNT_RESETVAL                (0x00000000U)
#define CSL_MSS_RCM_R5SS1_RST2ASSERTDLY_R5_CORE1_COUNT_MAX                     (0x000000FFU)

#define CSL_MSS_RCM_R5SS1_RST2ASSERTDLY_RESETVAL                               (0x00000000U)

/* R5SS1_RST_WFICHECK */

#define CSL_MSS_RCM_R5SS1_RST_WFICHECK_EN_R5SS_CORE0_MASK                      (0x00000007U)
#define CSL_MSS_RCM_R5SS1_RST_WFICHECK_EN_R5SS_CORE0_SHIFT                     (0x00000000U)
#define CSL_MSS_RCM_R5SS1_RST_WFICHECK_EN_R5SS_CORE0_RESETVAL                  (0x00000007U)
#define CSL_MSS_RCM_R5SS1_RST_WFICHECK_EN_R5SS_CORE0_MAX                       (0x00000007U)

#define CSL_MSS_RCM_R5SS1_RST_WFICHECK_EN_R5SS_CORE1_MASK                      (0x00000700U)
#define CSL_MSS_RCM_R5SS1_RST_WFICHECK_EN_R5SS_CORE1_SHIFT                     (0x00000008U)
#define CSL_MSS_RCM_R5SS1_RST_WFICHECK_EN_R5SS_CORE1_RESETVAL                  (0x00000007U)
#define CSL_MSS_RCM_R5SS1_RST_WFICHECK_EN_R5SS_CORE1_MAX                       (0x00000007U)

#define CSL_MSS_RCM_R5SS1_RST_WFICHECK_EN_R5_CORE0_MASK                        (0x00070000U)
#define CSL_MSS_RCM_R5SS1_RST_WFICHECK_EN_R5_CORE0_SHIFT                       (0x00000010U)
#define CSL_MSS_RCM_R5SS1_RST_WFICHECK_EN_R5_CORE0_RESETVAL                    (0x00000007U)
#define CSL_MSS_RCM_R5SS1_RST_WFICHECK_EN_R5_CORE0_MAX                         (0x00000007U)

#define CSL_MSS_RCM_R5SS1_RST_WFICHECK_EN_R5_CORE1_MASK                        (0x07000000U)
#define CSL_MSS_RCM_R5SS1_RST_WFICHECK_EN_R5_CORE1_SHIFT                       (0x00000018U)
#define CSL_MSS_RCM_R5SS1_RST_WFICHECK_EN_R5_CORE1_RESETVAL                    (0x00000007U)
#define CSL_MSS_RCM_R5SS1_RST_WFICHECK_EN_R5_CORE1_MAX                         (0x00000007U)

#define CSL_MSS_RCM_R5SS1_RST_WFICHECK_RESETVAL                                (0x07070707U)

/* CPSW_5_50_250_CLK_MUX_CTRL */

#define CSL_MSS_RCM_CPSW_5_50_250_CLK_MUX_CTRL_CLKSRCSEL_MASK                  (0x00000007U)
#define CSL_MSS_RCM_CPSW_5_50_250_CLK_MUX_CTRL_CLKSRCSEL_SHIFT                 (0x00000000U)
#define CSL_MSS_RCM_CPSW_5_50_250_CLK_MUX_CTRL_CLKSRCSEL_RESETVAL              (0x00000000U)
#define CSL_MSS_RCM_CPSW_5_50_250_CLK_MUX_CTRL_CLKSRCSEL_MAX                   (0x00000007U)

#define CSL_MSS_RCM_CPSW_5_50_250_CLK_MUX_CTRL_RESETVAL                        (0x00000000U)

/* MCAN0_CLK_SRC_SEL */

#define CSL_MSS_RCM_MCAN0_CLK_SRC_SEL_CLKSRCSEL_MASK                           (0x00000FFFU)
#define CSL_MSS_RCM_MCAN0_CLK_SRC_SEL_CLKSRCSEL_SHIFT                          (0x00000000U)
#define CSL_MSS_RCM_MCAN0_CLK_SRC_SEL_CLKSRCSEL_RESETVAL                       (0x00000000U)
#define CSL_MSS_RCM_MCAN0_CLK_SRC_SEL_CLKSRCSEL_MAX                            (0x00000FFFU)

#define CSL_MSS_RCM_MCAN0_CLK_SRC_SEL_RESETVAL                                 (0x00000000U)

/* MCAN1_CLK_SRC_SEL */

#define CSL_MSS_RCM_MCAN1_CLK_SRC_SEL_CLKSRCSEL_MASK                           (0x00000FFFU)
#define CSL_MSS_RCM_MCAN1_CLK_SRC_SEL_CLKSRCSEL_SHIFT                          (0x00000000U)
#define CSL_MSS_RCM_MCAN1_CLK_SRC_SEL_CLKSRCSEL_RESETVAL                       (0x00000000U)
#define CSL_MSS_RCM_MCAN1_CLK_SRC_SEL_CLKSRCSEL_MAX                            (0x00000FFFU)

#define CSL_MSS_RCM_MCAN1_CLK_SRC_SEL_RESETVAL                                 (0x00000000U)

/* MCAN2_CLK_SRC_SEL */

#define CSL_MSS_RCM_MCAN2_CLK_SRC_SEL_CLKSRCSEL_MASK                           (0x00000FFFU)
#define CSL_MSS_RCM_MCAN2_CLK_SRC_SEL_CLKSRCSEL_SHIFT                          (0x00000000U)
#define CSL_MSS_RCM_MCAN2_CLK_SRC_SEL_CLKSRCSEL_RESETVAL                       (0x00000000U)
#define CSL_MSS_RCM_MCAN2_CLK_SRC_SEL_CLKSRCSEL_MAX                            (0x00000FFFU)

#define CSL_MSS_RCM_MCAN2_CLK_SRC_SEL_RESETVAL                                 (0x00000000U)

/* MCAN3_CLK_SRC_SEL */

#define CSL_MSS_RCM_MCAN3_CLK_SRC_SEL_CLKSRCSEL_MASK                           (0x00000FFFU)
#define CSL_MSS_RCM_MCAN3_CLK_SRC_SEL_CLKSRCSEL_SHIFT                          (0x00000000U)
#define CSL_MSS_RCM_MCAN3_CLK_SRC_SEL_CLKSRCSEL_RESETVAL                       (0x00000000U)
#define CSL_MSS_RCM_MCAN3_CLK_SRC_SEL_CLKSRCSEL_MAX                            (0x00000FFFU)

#define CSL_MSS_RCM_MCAN3_CLK_SRC_SEL_RESETVAL                                 (0x00000000U)

/* OSPI0_CLK_SRC_SEL */

#define CSL_MSS_RCM_OSPI0_CLK_SRC_SEL_CLKSRCSEL_MASK                           (0x00000FFFU)
#define CSL_MSS_RCM_OSPI0_CLK_SRC_SEL_CLKSRCSEL_SHIFT                          (0x00000000U)
#define CSL_MSS_RCM_OSPI0_CLK_SRC_SEL_CLKSRCSEL_RESETVAL                       (0x00000000U)
#define CSL_MSS_RCM_OSPI0_CLK_SRC_SEL_CLKSRCSEL_MAX                            (0x00000FFFU)

#define CSL_MSS_RCM_OSPI0_CLK_SRC_SEL_RESETVAL                                 (0x00000000U)

/* RTI0_CLK_SRC_SEL */

#define CSL_MSS_RCM_RTI0_CLK_SRC_SEL_CLKSRCSEL_MASK                            (0x00000FFFU)
#define CSL_MSS_RCM_RTI0_CLK_SRC_SEL_CLKSRCSEL_SHIFT                           (0x00000000U)
#define CSL_MSS_RCM_RTI0_CLK_SRC_SEL_CLKSRCSEL_RESETVAL                        (0x00000000U)
#define CSL_MSS_RCM_RTI0_CLK_SRC_SEL_CLKSRCSEL_MAX                             (0x00000FFFU)

#define CSL_MSS_RCM_RTI0_CLK_SRC_SEL_RESETVAL                                  (0x00000000U)

/* RTI1_CLK_SRC_SEL */

#define CSL_MSS_RCM_RTI1_CLK_SRC_SEL_CLKSRCSEL_MASK                            (0x00000FFFU)
#define CSL_MSS_RCM_RTI1_CLK_SRC_SEL_CLKSRCSEL_SHIFT                           (0x00000000U)
#define CSL_MSS_RCM_RTI1_CLK_SRC_SEL_CLKSRCSEL_RESETVAL                        (0x00000000U)
#define CSL_MSS_RCM_RTI1_CLK_SRC_SEL_CLKSRCSEL_MAX                             (0x00000FFFU)

#define CSL_MSS_RCM_RTI1_CLK_SRC_SEL_RESETVAL                                  (0x00000000U)

/* RTI2_CLK_SRC_SEL */

#define CSL_MSS_RCM_RTI2_CLK_SRC_SEL_CLKSRCSEL_MASK                            (0x00000FFFU)
#define CSL_MSS_RCM_RTI2_CLK_SRC_SEL_CLKSRCSEL_SHIFT                           (0x00000000U)
#define CSL_MSS_RCM_RTI2_CLK_SRC_SEL_CLKSRCSEL_RESETVAL                        (0x00000000U)
#define CSL_MSS_RCM_RTI2_CLK_SRC_SEL_CLKSRCSEL_MAX                             (0x00000FFFU)

#define CSL_MSS_RCM_RTI2_CLK_SRC_SEL_RESETVAL                                  (0x00000000U)

/* RTI3_CLK_SRC_SEL */

#define CSL_MSS_RCM_RTI3_CLK_SRC_SEL_CLKSRCSEL_MASK                            (0x00000FFFU)
#define CSL_MSS_RCM_RTI3_CLK_SRC_SEL_CLKSRCSEL_SHIFT                           (0x00000000U)
#define CSL_MSS_RCM_RTI3_CLK_SRC_SEL_CLKSRCSEL_RESETVAL                        (0x00000000U)
#define CSL_MSS_RCM_RTI3_CLK_SRC_SEL_CLKSRCSEL_MAX                             (0x00000FFFU)

#define CSL_MSS_RCM_RTI3_CLK_SRC_SEL_RESETVAL                                  (0x00000000U)

/* WDT0_CLK_SRC_SEL */

#define CSL_MSS_RCM_WDT0_CLK_SRC_SEL_CLKSRCSEL_MASK                            (0x00000FFFU)
#define CSL_MSS_RCM_WDT0_CLK_SRC_SEL_CLKSRCSEL_SHIFT                           (0x00000000U)
#define CSL_MSS_RCM_WDT0_CLK_SRC_SEL_CLKSRCSEL_RESETVAL                        (0x00000000U)
#define CSL_MSS_RCM_WDT0_CLK_SRC_SEL_CLKSRCSEL_MAX                             (0x00000FFFU)

#define CSL_MSS_RCM_WDT0_CLK_SRC_SEL_RESETVAL                                  (0x00000000U)

/* WDT1_CLK_SRC_SEL */

#define CSL_MSS_RCM_WDT1_CLK_SRC_SEL_CLKSRCSEL_MASK                            (0x00000FFFU)
#define CSL_MSS_RCM_WDT1_CLK_SRC_SEL_CLKSRCSEL_SHIFT                           (0x00000000U)
#define CSL_MSS_RCM_WDT1_CLK_SRC_SEL_CLKSRCSEL_RESETVAL                        (0x00000000U)
#define CSL_MSS_RCM_WDT1_CLK_SRC_SEL_CLKSRCSEL_MAX                             (0x00000FFFU)

#define CSL_MSS_RCM_WDT1_CLK_SRC_SEL_RESETVAL                                  (0x00000000U)

/* WDT2_CLK_SRC_SEL */

#define CSL_MSS_RCM_WDT2_CLK_SRC_SEL_CLKSRCSEL_MASK                            (0x00000FFFU)
#define CSL_MSS_RCM_WDT2_CLK_SRC_SEL_CLKSRCSEL_SHIFT                           (0x00000000U)
#define CSL_MSS_RCM_WDT2_CLK_SRC_SEL_CLKSRCSEL_RESETVAL                        (0x00000000U)
#define CSL_MSS_RCM_WDT2_CLK_SRC_SEL_CLKSRCSEL_MAX                             (0x00000FFFU)

#define CSL_MSS_RCM_WDT2_CLK_SRC_SEL_RESETVAL                                  (0x00000000U)

/* WDT3_CLK_SRC_SEL */

#define CSL_MSS_RCM_WDT3_CLK_SRC_SEL_CLKSRCSEL_MASK                            (0x00000FFFU)
#define CSL_MSS_RCM_WDT3_CLK_SRC_SEL_CLKSRCSEL_SHIFT                           (0x00000000U)
#define CSL_MSS_RCM_WDT3_CLK_SRC_SEL_CLKSRCSEL_RESETVAL                        (0x00000000U)
#define CSL_MSS_RCM_WDT3_CLK_SRC_SEL_CLKSRCSEL_MAX                             (0x00000FFFU)

#define CSL_MSS_RCM_WDT3_CLK_SRC_SEL_RESETVAL                                  (0x00000000U)

/* MCSPI0_CLK_SRC_SEL */

#define CSL_MSS_RCM_MCSPI0_CLK_SRC_SEL_CLKSRCSEL_MASK                          (0x00000FFFU)
#define CSL_MSS_RCM_MCSPI0_CLK_SRC_SEL_CLKSRCSEL_SHIFT                         (0x00000000U)
#define CSL_MSS_RCM_MCSPI0_CLK_SRC_SEL_CLKSRCSEL_RESETVAL                      (0x00000000U)
#define CSL_MSS_RCM_MCSPI0_CLK_SRC_SEL_CLKSRCSEL_MAX                           (0x00000FFFU)

#define CSL_MSS_RCM_MCSPI0_CLK_SRC_SEL_RESETVAL                                (0x00000000U)

/* MCSPI1_CLK_SRC_SEL */

#define CSL_MSS_RCM_MCSPI1_CLK_SRC_SEL_CLKSRCSEL_MASK                          (0x00000FFFU)
#define CSL_MSS_RCM_MCSPI1_CLK_SRC_SEL_CLKSRCSEL_SHIFT                         (0x00000000U)
#define CSL_MSS_RCM_MCSPI1_CLK_SRC_SEL_CLKSRCSEL_RESETVAL                      (0x00000000U)
#define CSL_MSS_RCM_MCSPI1_CLK_SRC_SEL_CLKSRCSEL_MAX                           (0x00000FFFU)

#define CSL_MSS_RCM_MCSPI1_CLK_SRC_SEL_RESETVAL                                (0x00000000U)

/* MCSPI2_CLK_SRC_SEL */

#define CSL_MSS_RCM_MCSPI2_CLK_SRC_SEL_CLKSRCSEL_MASK                          (0x00000FFFU)
#define CSL_MSS_RCM_MCSPI2_CLK_SRC_SEL_CLKSRCSEL_SHIFT                         (0x00000000U)
#define CSL_MSS_RCM_MCSPI2_CLK_SRC_SEL_CLKSRCSEL_RESETVAL                      (0x00000000U)
#define CSL_MSS_RCM_MCSPI2_CLK_SRC_SEL_CLKSRCSEL_MAX                           (0x00000FFFU)

#define CSL_MSS_RCM_MCSPI2_CLK_SRC_SEL_RESETVAL                                (0x00000000U)

/* MCSPI3_CLK_SRC_SEL */

#define CSL_MSS_RCM_MCSPI3_CLK_SRC_SEL_CLKSRCSEL_MASK                          (0x00000FFFU)
#define CSL_MSS_RCM_MCSPI3_CLK_SRC_SEL_CLKSRCSEL_SHIFT                         (0x00000000U)
#define CSL_MSS_RCM_MCSPI3_CLK_SRC_SEL_CLKSRCSEL_RESETVAL                      (0x00000000U)
#define CSL_MSS_RCM_MCSPI3_CLK_SRC_SEL_CLKSRCSEL_MAX                           (0x00000FFFU)

#define CSL_MSS_RCM_MCSPI3_CLK_SRC_SEL_RESETVAL                                (0x00000000U)

/* MCSPI4_CLK_SRC_SEL */

#define CSL_MSS_RCM_MCSPI4_CLK_SRC_SEL_CLKSRCSEL_MASK                          (0x00000FFFU)
#define CSL_MSS_RCM_MCSPI4_CLK_SRC_SEL_CLKSRCSEL_SHIFT                         (0x00000000U)
#define CSL_MSS_RCM_MCSPI4_CLK_SRC_SEL_CLKSRCSEL_RESETVAL                      (0x00000000U)
#define CSL_MSS_RCM_MCSPI4_CLK_SRC_SEL_CLKSRCSEL_MAX                           (0x00000FFFU)

#define CSL_MSS_RCM_MCSPI4_CLK_SRC_SEL_RESETVAL                                (0x00000000U)

/* MMC0_CLK_SRC_SEL */

#define CSL_MSS_RCM_MMC0_CLK_SRC_SEL_CLKSRCSEL_MASK                            (0x00000FFFU)
#define CSL_MSS_RCM_MMC0_CLK_SRC_SEL_CLKSRCSEL_SHIFT                           (0x00000000U)
#define CSL_MSS_RCM_MMC0_CLK_SRC_SEL_CLKSRCSEL_RESETVAL                        (0x00000000U)
#define CSL_MSS_RCM_MMC0_CLK_SRC_SEL_CLKSRCSEL_MAX                             (0x00000FFFU)

#define CSL_MSS_RCM_MMC0_CLK_SRC_SEL_RESETVAL                                  (0x00000000U)

/* ICSSM0_UART0_CLK_SRC_SEL */

#define CSL_MSS_RCM_ICSSM0_UART0_CLK_SRC_SEL_CLKSRCSEL_MASK                    (0x00000FFFU)
#define CSL_MSS_RCM_ICSSM0_UART0_CLK_SRC_SEL_CLKSRCSEL_SHIFT                   (0x00000000U)
#define CSL_MSS_RCM_ICSSM0_UART0_CLK_SRC_SEL_CLKSRCSEL_RESETVAL                (0x00000000U)
#define CSL_MSS_RCM_ICSSM0_UART0_CLK_SRC_SEL_CLKSRCSEL_MAX                     (0x00000FFFU)

#define CSL_MSS_RCM_ICSSM0_UART0_CLK_SRC_SEL_RESETVAL                          (0x00000000U)

/* CPTS_CLK_SRC_SEL */

#define CSL_MSS_RCM_CPTS_CLK_SRC_SEL_CLKSRCSEL_MASK                            (0x00000FFFU)
#define CSL_MSS_RCM_CPTS_CLK_SRC_SEL_CLKSRCSEL_SHIFT                           (0x00000000U)
#define CSL_MSS_RCM_CPTS_CLK_SRC_SEL_CLKSRCSEL_RESETVAL                        (0x00000000U)
#define CSL_MSS_RCM_CPTS_CLK_SRC_SEL_CLKSRCSEL_MAX                             (0x00000FFFU)

#define CSL_MSS_RCM_CPTS_CLK_SRC_SEL_RESETVAL                                  (0x00000000U)

/* CONTROLSS_PLL_CLK_SRC_SEL */

#define CSL_MSS_RCM_CONTROLSS_PLL_CLK_SRC_SEL_CLKSRCSEL_MASK                   (0x00000FFFU)
#define CSL_MSS_RCM_CONTROLSS_PLL_CLK_SRC_SEL_CLKSRCSEL_SHIFT                  (0x00000000U)
#define CSL_MSS_RCM_CONTROLSS_PLL_CLK_SRC_SEL_CLKSRCSEL_RESETVAL               (0x00000000U)
#define CSL_MSS_RCM_CONTROLSS_PLL_CLK_SRC_SEL_CLKSRCSEL_MAX                    (0x00000FFFU)

#define CSL_MSS_RCM_CONTROLSS_PLL_CLK_SRC_SEL_RESETVAL                         (0x00000000U)

/* I2C_CLK_SRC_SEL */

#define CSL_MSS_RCM_I2C_CLK_SRC_SEL_CLKSRCSEL_MASK                             (0x00000FFFU)
#define CSL_MSS_RCM_I2C_CLK_SRC_SEL_CLKSRCSEL_SHIFT                            (0x00000000U)
#define CSL_MSS_RCM_I2C_CLK_SRC_SEL_CLKSRCSEL_RESETVAL                         (0x00000000U)
#define CSL_MSS_RCM_I2C_CLK_SRC_SEL_CLKSRCSEL_MAX                              (0x00000FFFU)

#define CSL_MSS_RCM_I2C_CLK_SRC_SEL_RESETVAL                                   (0x00000000U)

/* LIN0_UART0_CLK_SRC_SEL */

#define CSL_MSS_RCM_LIN0_UART0_CLK_SRC_SEL_CLKSRCSEL_MASK                      (0x00000FFFU)
#define CSL_MSS_RCM_LIN0_UART0_CLK_SRC_SEL_CLKSRCSEL_SHIFT                     (0x00000000U)
#define CSL_MSS_RCM_LIN0_UART0_CLK_SRC_SEL_CLKSRCSEL_RESETVAL                  (0x00000000U)
#define CSL_MSS_RCM_LIN0_UART0_CLK_SRC_SEL_CLKSRCSEL_MAX                       (0x00000FFFU)

#define CSL_MSS_RCM_LIN0_UART0_CLK_SRC_SEL_RESETVAL                            (0x00000000U)

/* LIN1_UART1_CLK_SRC_SEL */

#define CSL_MSS_RCM_LIN1_UART1_CLK_SRC_SEL_CLKSRCSEL_MASK                      (0x00000FFFU)
#define CSL_MSS_RCM_LIN1_UART1_CLK_SRC_SEL_CLKSRCSEL_SHIFT                     (0x00000000U)
#define CSL_MSS_RCM_LIN1_UART1_CLK_SRC_SEL_CLKSRCSEL_RESETVAL                  (0x00000000U)
#define CSL_MSS_RCM_LIN1_UART1_CLK_SRC_SEL_CLKSRCSEL_MAX                       (0x00000FFFU)

#define CSL_MSS_RCM_LIN1_UART1_CLK_SRC_SEL_RESETVAL                            (0x00000000U)

/* LIN2_UART2_CLK_SRC_SEL */

#define CSL_MSS_RCM_LIN2_UART2_CLK_SRC_SEL_CLKSRCSEL_MASK                      (0x00000FFFU)
#define CSL_MSS_RCM_LIN2_UART2_CLK_SRC_SEL_CLKSRCSEL_SHIFT                     (0x00000000U)
#define CSL_MSS_RCM_LIN2_UART2_CLK_SRC_SEL_CLKSRCSEL_RESETVAL                  (0x00000000U)
#define CSL_MSS_RCM_LIN2_UART2_CLK_SRC_SEL_CLKSRCSEL_MAX                       (0x00000FFFU)

#define CSL_MSS_RCM_LIN2_UART2_CLK_SRC_SEL_RESETVAL                            (0x00000000U)

/* LIN3_UART3_CLK_SRC_SEL */

#define CSL_MSS_RCM_LIN3_UART3_CLK_SRC_SEL_CLKSRCSEL_MASK                      (0x00000FFFU)
#define CSL_MSS_RCM_LIN3_UART3_CLK_SRC_SEL_CLKSRCSEL_SHIFT                     (0x00000000U)
#define CSL_MSS_RCM_LIN3_UART3_CLK_SRC_SEL_CLKSRCSEL_RESETVAL                  (0x00000000U)
#define CSL_MSS_RCM_LIN3_UART3_CLK_SRC_SEL_CLKSRCSEL_MAX                       (0x00000FFFU)

#define CSL_MSS_RCM_LIN3_UART3_CLK_SRC_SEL_RESETVAL                            (0x00000000U)

/* LIN4_UART4_CLK_SRC_SEL */

#define CSL_MSS_RCM_LIN4_UART4_CLK_SRC_SEL_CLKSRCSEL_MASK                      (0x00000FFFU)
#define CSL_MSS_RCM_LIN4_UART4_CLK_SRC_SEL_CLKSRCSEL_SHIFT                     (0x00000000U)
#define CSL_MSS_RCM_LIN4_UART4_CLK_SRC_SEL_CLKSRCSEL_RESETVAL                  (0x00000000U)
#define CSL_MSS_RCM_LIN4_UART4_CLK_SRC_SEL_CLKSRCSEL_MAX                       (0x00000FFFU)

#define CSL_MSS_RCM_LIN4_UART4_CLK_SRC_SEL_RESETVAL                            (0x00000000U)

/* LIN5_UART5_CLK_SRC_SEL */

#define CSL_MSS_RCM_LIN5_UART5_CLK_SRC_SEL_CLKSRCSEL_MASK                      (0x00000FFFU)
#define CSL_MSS_RCM_LIN5_UART5_CLK_SRC_SEL_CLKSRCSEL_SHIFT                     (0x00000000U)
#define CSL_MSS_RCM_LIN5_UART5_CLK_SRC_SEL_CLKSRCSEL_RESETVAL                  (0x00000000U)
#define CSL_MSS_RCM_LIN5_UART5_CLK_SRC_SEL_CLKSRCSEL_MAX                       (0x00000FFFU)

#define CSL_MSS_RCM_LIN5_UART5_CLK_SRC_SEL_RESETVAL                            (0x00000000U)

/* MCAN4_CLK_SRC_SEL */

#define CSL_MSS_RCM_MCAN4_CLK_SRC_SEL_CLKSRCSEL_MASK                           (0x00000FFFU)
#define CSL_MSS_RCM_MCAN4_CLK_SRC_SEL_CLKSRCSEL_SHIFT                          (0x00000000U)
#define CSL_MSS_RCM_MCAN4_CLK_SRC_SEL_CLKSRCSEL_RESETVAL                       (0x00000000U)
#define CSL_MSS_RCM_MCAN4_CLK_SRC_SEL_CLKSRCSEL_MAX                            (0x00000FFFU)

#define CSL_MSS_RCM_MCAN4_CLK_SRC_SEL_RESETVAL                                 (0x00000000U)

/* MCAN5_CLK_SRC_SEL */

#define CSL_MSS_RCM_MCAN5_CLK_SRC_SEL_CLKSRCSEL_MASK                           (0x00000FFFU)
#define CSL_MSS_RCM_MCAN5_CLK_SRC_SEL_CLKSRCSEL_SHIFT                          (0x00000000U)
#define CSL_MSS_RCM_MCAN5_CLK_SRC_SEL_CLKSRCSEL_RESETVAL                       (0x00000000U)
#define CSL_MSS_RCM_MCAN5_CLK_SRC_SEL_CLKSRCSEL_MAX                            (0x00000FFFU)

#define CSL_MSS_RCM_MCAN5_CLK_SRC_SEL_RESETVAL                                 (0x00000000U)

/* MCAN6_CLK_SRC_SEL */

#define CSL_MSS_RCM_MCAN6_CLK_SRC_SEL_CLKSRCSEL_MASK                           (0x00000FFFU)
#define CSL_MSS_RCM_MCAN6_CLK_SRC_SEL_CLKSRCSEL_SHIFT                          (0x00000000U)
#define CSL_MSS_RCM_MCAN6_CLK_SRC_SEL_CLKSRCSEL_RESETVAL                       (0x00000000U)
#define CSL_MSS_RCM_MCAN6_CLK_SRC_SEL_CLKSRCSEL_MAX                            (0x00000FFFU)

#define CSL_MSS_RCM_MCAN6_CLK_SRC_SEL_RESETVAL                                 (0x00000000U)

/* MCAN7_CLK_SRC_SEL */

#define CSL_MSS_RCM_MCAN7_CLK_SRC_SEL_CLKSRCSEL_MASK                           (0x00000FFFU)
#define CSL_MSS_RCM_MCAN7_CLK_SRC_SEL_CLKSRCSEL_SHIFT                          (0x00000000U)
#define CSL_MSS_RCM_MCAN7_CLK_SRC_SEL_CLKSRCSEL_RESETVAL                       (0x00000000U)
#define CSL_MSS_RCM_MCAN7_CLK_SRC_SEL_CLKSRCSEL_MAX                            (0x00000FFFU)

#define CSL_MSS_RCM_MCAN7_CLK_SRC_SEL_RESETVAL                                 (0x00000000U)

/* RTI4_CLK_SRC_SEL */

#define CSL_MSS_RCM_RTI4_CLK_SRC_SEL_CLKSRCSEL_MASK                            (0x00000FFFU)
#define CSL_MSS_RCM_RTI4_CLK_SRC_SEL_CLKSRCSEL_SHIFT                           (0x00000000U)
#define CSL_MSS_RCM_RTI4_CLK_SRC_SEL_CLKSRCSEL_RESETVAL                        (0x00000000U)
#define CSL_MSS_RCM_RTI4_CLK_SRC_SEL_CLKSRCSEL_MAX                             (0x00000FFFU)

#define CSL_MSS_RCM_RTI4_CLK_SRC_SEL_RESETVAL                                  (0x00000000U)

/* RTI5_CLK_SRC_SEL */

#define CSL_MSS_RCM_RTI5_CLK_SRC_SEL_CLKSRCSEL_MASK                            (0x00000FFFU)
#define CSL_MSS_RCM_RTI5_CLK_SRC_SEL_CLKSRCSEL_SHIFT                           (0x00000000U)
#define CSL_MSS_RCM_RTI5_CLK_SRC_SEL_CLKSRCSEL_RESETVAL                        (0x00000000U)
#define CSL_MSS_RCM_RTI5_CLK_SRC_SEL_CLKSRCSEL_MAX                             (0x00000FFFU)

#define CSL_MSS_RCM_RTI5_CLK_SRC_SEL_RESETVAL                                  (0x00000000U)

/* RTI6_CLK_SRC_SEL */

#define CSL_MSS_RCM_RTI6_CLK_SRC_SEL_CLKSRCSEL_MASK                            (0x00000FFFU)
#define CSL_MSS_RCM_RTI6_CLK_SRC_SEL_CLKSRCSEL_SHIFT                           (0x00000000U)
#define CSL_MSS_RCM_RTI6_CLK_SRC_SEL_CLKSRCSEL_RESETVAL                        (0x00000000U)
#define CSL_MSS_RCM_RTI6_CLK_SRC_SEL_CLKSRCSEL_MAX                             (0x00000FFFU)

#define CSL_MSS_RCM_RTI6_CLK_SRC_SEL_RESETVAL                                  (0x00000000U)

/* RTI7_CLK_SRC_SEL */

#define CSL_MSS_RCM_RTI7_CLK_SRC_SEL_CLKSRCSEL_MASK                            (0x00000FFFU)
#define CSL_MSS_RCM_RTI7_CLK_SRC_SEL_CLKSRCSEL_SHIFT                           (0x00000000U)
#define CSL_MSS_RCM_RTI7_CLK_SRC_SEL_CLKSRCSEL_RESETVAL                        (0x00000000U)
#define CSL_MSS_RCM_RTI7_CLK_SRC_SEL_CLKSRCSEL_MAX                             (0x00000FFFU)

#define CSL_MSS_RCM_RTI7_CLK_SRC_SEL_RESETVAL                                  (0x00000000U)

/* MCSPI5_CLK_SRC_SEL */

#define CSL_MSS_RCM_MCSPI5_CLK_SRC_SEL_CLKSRCSEL_MASK                          (0x00000FFFU)
#define CSL_MSS_RCM_MCSPI5_CLK_SRC_SEL_CLKSRCSEL_SHIFT                         (0x00000000U)
#define CSL_MSS_RCM_MCSPI5_CLK_SRC_SEL_CLKSRCSEL_RESETVAL                      (0x00000000U)
#define CSL_MSS_RCM_MCSPI5_CLK_SRC_SEL_CLKSRCSEL_MAX                           (0x00000FFFU)

#define CSL_MSS_RCM_MCSPI5_CLK_SRC_SEL_RESETVAL                                (0x00000000U)

/* MCSPI6_CLK_SRC_SEL */

#define CSL_MSS_RCM_MCSPI6_CLK_SRC_SEL_CLKSRCSEL_MASK                          (0x00000FFFU)
#define CSL_MSS_RCM_MCSPI6_CLK_SRC_SEL_CLKSRCSEL_SHIFT                         (0x00000000U)
#define CSL_MSS_RCM_MCSPI6_CLK_SRC_SEL_CLKSRCSEL_RESETVAL                      (0x00000000U)
#define CSL_MSS_RCM_MCSPI6_CLK_SRC_SEL_CLKSRCSEL_MAX                           (0x00000FFFU)

#define CSL_MSS_RCM_MCSPI6_CLK_SRC_SEL_RESETVAL                                (0x00000000U)

/* MCSPI7_CLK_SRC_SEL */

#define CSL_MSS_RCM_MCSPI7_CLK_SRC_SEL_CLKSRCSEL_MASK                          (0x00000FFFU)
#define CSL_MSS_RCM_MCSPI7_CLK_SRC_SEL_CLKSRCSEL_SHIFT                         (0x00000000U)
#define CSL_MSS_RCM_MCSPI7_CLK_SRC_SEL_CLKSRCSEL_RESETVAL                      (0x00000000U)
#define CSL_MSS_RCM_MCSPI7_CLK_SRC_SEL_CLKSRCSEL_MAX                           (0x00000FFFU)

#define CSL_MSS_RCM_MCSPI7_CLK_SRC_SEL_RESETVAL                                (0x00000000U)

/* MCAN0_CLK_DIV_VAL */

#define CSL_MSS_RCM_MCAN0_CLK_DIV_VAL_CLKDIVR_MASK                             (0x00000FFFU)
#define CSL_MSS_RCM_MCAN0_CLK_DIV_VAL_CLKDIVR_SHIFT                            (0x00000000U)
#define CSL_MSS_RCM_MCAN0_CLK_DIV_VAL_CLKDIVR_RESETVAL                         (0x00000000U)
#define CSL_MSS_RCM_MCAN0_CLK_DIV_VAL_CLKDIVR_MAX                              (0x00000FFFU)

#define CSL_MSS_RCM_MCAN0_CLK_DIV_VAL_RESETVAL                                 (0x00000000U)

/* MCAN1_CLK_DIV_VAL */

#define CSL_MSS_RCM_MCAN1_CLK_DIV_VAL_CLKDIVR_MASK                             (0x00000FFFU)
#define CSL_MSS_RCM_MCAN1_CLK_DIV_VAL_CLKDIVR_SHIFT                            (0x00000000U)
#define CSL_MSS_RCM_MCAN1_CLK_DIV_VAL_CLKDIVR_RESETVAL                         (0x00000000U)
#define CSL_MSS_RCM_MCAN1_CLK_DIV_VAL_CLKDIVR_MAX                              (0x00000FFFU)

#define CSL_MSS_RCM_MCAN1_CLK_DIV_VAL_RESETVAL                                 (0x00000000U)

/* MCAN2_CLK_DIV_VAL */

#define CSL_MSS_RCM_MCAN2_CLK_DIV_VAL_CLKDIVR_MASK                             (0x00000FFFU)
#define CSL_MSS_RCM_MCAN2_CLK_DIV_VAL_CLKDIVR_SHIFT                            (0x00000000U)
#define CSL_MSS_RCM_MCAN2_CLK_DIV_VAL_CLKDIVR_RESETVAL                         (0x00000000U)
#define CSL_MSS_RCM_MCAN2_CLK_DIV_VAL_CLKDIVR_MAX                              (0x00000FFFU)

#define CSL_MSS_RCM_MCAN2_CLK_DIV_VAL_RESETVAL                                 (0x00000000U)

/* MCAN3_CLK_DIV_VAL */

#define CSL_MSS_RCM_MCAN3_CLK_DIV_VAL_CLKDIVR_MASK                             (0x00000FFFU)
#define CSL_MSS_RCM_MCAN3_CLK_DIV_VAL_CLKDIVR_SHIFT                            (0x00000000U)
#define CSL_MSS_RCM_MCAN3_CLK_DIV_VAL_CLKDIVR_RESETVAL                         (0x00000000U)
#define CSL_MSS_RCM_MCAN3_CLK_DIV_VAL_CLKDIVR_MAX                              (0x00000FFFU)

#define CSL_MSS_RCM_MCAN3_CLK_DIV_VAL_RESETVAL                                 (0x00000000U)

/* OSPI0_CLK_DIV_VAL */

#define CSL_MSS_RCM_OSPI0_CLK_DIV_VAL_CLKDIVR_MASK                             (0x00000FFFU)
#define CSL_MSS_RCM_OSPI0_CLK_DIV_VAL_CLKDIVR_SHIFT                            (0x00000000U)
#define CSL_MSS_RCM_OSPI0_CLK_DIV_VAL_CLKDIVR_RESETVAL                         (0x00000000U)
#define CSL_MSS_RCM_OSPI0_CLK_DIV_VAL_CLKDIVR_MAX                              (0x00000FFFU)

#define CSL_MSS_RCM_OSPI0_CLK_DIV_VAL_RESETVAL                                 (0x00000000U)

/* RTI0_CLK_DIV_VAL */

#define CSL_MSS_RCM_RTI0_CLK_DIV_VAL_CLKDIVR_MASK                              (0x00000FFFU)
#define CSL_MSS_RCM_RTI0_CLK_DIV_VAL_CLKDIVR_SHIFT                             (0x00000000U)
#define CSL_MSS_RCM_RTI0_CLK_DIV_VAL_CLKDIVR_RESETVAL                          (0x00000000U)
#define CSL_MSS_RCM_RTI0_CLK_DIV_VAL_CLKDIVR_MAX                               (0x00000FFFU)

#define CSL_MSS_RCM_RTI0_CLK_DIV_VAL_RESETVAL                                  (0x00000000U)

/* RTI1_CLK_DIV_VAL */

#define CSL_MSS_RCM_RTI1_CLK_DIV_VAL_CLKDIVR_MASK                              (0x00000FFFU)
#define CSL_MSS_RCM_RTI1_CLK_DIV_VAL_CLKDIVR_SHIFT                             (0x00000000U)
#define CSL_MSS_RCM_RTI1_CLK_DIV_VAL_CLKDIVR_RESETVAL                          (0x00000000U)
#define CSL_MSS_RCM_RTI1_CLK_DIV_VAL_CLKDIVR_MAX                               (0x00000FFFU)

#define CSL_MSS_RCM_RTI1_CLK_DIV_VAL_RESETVAL                                  (0x00000000U)

/* RTI2_CLK_DIV_VAL */

#define CSL_MSS_RCM_RTI2_CLK_DIV_VAL_CLKDIVR_MASK                              (0x00000FFFU)
#define CSL_MSS_RCM_RTI2_CLK_DIV_VAL_CLKDIVR_SHIFT                             (0x00000000U)
#define CSL_MSS_RCM_RTI2_CLK_DIV_VAL_CLKDIVR_RESETVAL                          (0x00000000U)
#define CSL_MSS_RCM_RTI2_CLK_DIV_VAL_CLKDIVR_MAX                               (0x00000FFFU)

#define CSL_MSS_RCM_RTI2_CLK_DIV_VAL_RESETVAL                                  (0x00000000U)

/* RTI3_CLK_DIV_VAL */

#define CSL_MSS_RCM_RTI3_CLK_DIV_VAL_CLKDIVR_MASK                              (0x00000FFFU)
#define CSL_MSS_RCM_RTI3_CLK_DIV_VAL_CLKDIVR_SHIFT                             (0x00000000U)
#define CSL_MSS_RCM_RTI3_CLK_DIV_VAL_CLKDIVR_RESETVAL                          (0x00000000U)
#define CSL_MSS_RCM_RTI3_CLK_DIV_VAL_CLKDIVR_MAX                               (0x00000FFFU)

#define CSL_MSS_RCM_RTI3_CLK_DIV_VAL_RESETVAL                                  (0x00000000U)

/* WDT0_CLK_DIV_VAL */

#define CSL_MSS_RCM_WDT0_CLK_DIV_VAL_CLKDIVR_MASK                              (0x00000FFFU)
#define CSL_MSS_RCM_WDT0_CLK_DIV_VAL_CLKDIVR_SHIFT                             (0x00000000U)
#define CSL_MSS_RCM_WDT0_CLK_DIV_VAL_CLKDIVR_RESETVAL                          (0x00000000U)
#define CSL_MSS_RCM_WDT0_CLK_DIV_VAL_CLKDIVR_MAX                               (0x00000FFFU)

#define CSL_MSS_RCM_WDT0_CLK_DIV_VAL_RESETVAL                                  (0x00000000U)

/* WDT1_CLK_DIV_VAL */

#define CSL_MSS_RCM_WDT1_CLK_DIV_VAL_CLKDIVR_MASK                              (0x00000FFFU)
#define CSL_MSS_RCM_WDT1_CLK_DIV_VAL_CLKDIVR_SHIFT                             (0x00000000U)
#define CSL_MSS_RCM_WDT1_CLK_DIV_VAL_CLKDIVR_RESETVAL                          (0x00000000U)
#define CSL_MSS_RCM_WDT1_CLK_DIV_VAL_CLKDIVR_MAX                               (0x00000FFFU)

#define CSL_MSS_RCM_WDT1_CLK_DIV_VAL_RESETVAL                                  (0x00000000U)

/* WDT2_CLK_DIV_VAL */

#define CSL_MSS_RCM_WDT2_CLK_DIV_VAL_CLKDIVR_MASK                              (0x00000FFFU)
#define CSL_MSS_RCM_WDT2_CLK_DIV_VAL_CLKDIVR_SHIFT                             (0x00000000U)
#define CSL_MSS_RCM_WDT2_CLK_DIV_VAL_CLKDIVR_RESETVAL                          (0x00000000U)
#define CSL_MSS_RCM_WDT2_CLK_DIV_VAL_CLKDIVR_MAX                               (0x00000FFFU)

#define CSL_MSS_RCM_WDT2_CLK_DIV_VAL_RESETVAL                                  (0x00000000U)

/* WDT3_CLK_DIV_VAL */

#define CSL_MSS_RCM_WDT3_CLK_DIV_VAL_CLKDIVR_MASK                              (0x00000FFFU)
#define CSL_MSS_RCM_WDT3_CLK_DIV_VAL_CLKDIVR_SHIFT                             (0x00000000U)
#define CSL_MSS_RCM_WDT3_CLK_DIV_VAL_CLKDIVR_RESETVAL                          (0x00000000U)
#define CSL_MSS_RCM_WDT3_CLK_DIV_VAL_CLKDIVR_MAX                               (0x00000FFFU)

#define CSL_MSS_RCM_WDT3_CLK_DIV_VAL_RESETVAL                                  (0x00000000U)

/* MCSPI0_CLK_DIV_VAL */

#define CSL_MSS_RCM_MCSPI0_CLK_DIV_VAL_CLKDIVR_MASK                            (0x00000FFFU)
#define CSL_MSS_RCM_MCSPI0_CLK_DIV_VAL_CLKDIVR_SHIFT                           (0x00000000U)
#define CSL_MSS_RCM_MCSPI0_CLK_DIV_VAL_CLKDIVR_RESETVAL                        (0x00000000U)
#define CSL_MSS_RCM_MCSPI0_CLK_DIV_VAL_CLKDIVR_MAX                             (0x00000FFFU)

#define CSL_MSS_RCM_MCSPI0_CLK_DIV_VAL_RESETVAL                                (0x00000000U)

/* MCSPI1_CLK_DIV_VAL */

#define CSL_MSS_RCM_MCSPI1_CLK_DIV_VAL_CLKDIVR_MASK                            (0x00000FFFU)
#define CSL_MSS_RCM_MCSPI1_CLK_DIV_VAL_CLKDIVR_SHIFT                           (0x00000000U)
#define CSL_MSS_RCM_MCSPI1_CLK_DIV_VAL_CLKDIVR_RESETVAL                        (0x00000000U)
#define CSL_MSS_RCM_MCSPI1_CLK_DIV_VAL_CLKDIVR_MAX                             (0x00000FFFU)

#define CSL_MSS_RCM_MCSPI1_CLK_DIV_VAL_RESETVAL                                (0x00000000U)

/* MCSPI2_CLK_DIV_VAL */

#define CSL_MSS_RCM_MCSPI2_CLK_DIV_VAL_CLKDIVR_MASK                            (0x00000FFFU)
#define CSL_MSS_RCM_MCSPI2_CLK_DIV_VAL_CLKDIVR_SHIFT                           (0x00000000U)
#define CSL_MSS_RCM_MCSPI2_CLK_DIV_VAL_CLKDIVR_RESETVAL                        (0x00000000U)
#define CSL_MSS_RCM_MCSPI2_CLK_DIV_VAL_CLKDIVR_MAX                             (0x00000FFFU)

#define CSL_MSS_RCM_MCSPI2_CLK_DIV_VAL_RESETVAL                                (0x00000000U)

/* MCSPI3_CLK_DIV_VAL */

#define CSL_MSS_RCM_MCSPI3_CLK_DIV_VAL_CLKDIVR_MASK                            (0x00000FFFU)
#define CSL_MSS_RCM_MCSPI3_CLK_DIV_VAL_CLKDIVR_SHIFT                           (0x00000000U)
#define CSL_MSS_RCM_MCSPI3_CLK_DIV_VAL_CLKDIVR_RESETVAL                        (0x00000000U)
#define CSL_MSS_RCM_MCSPI3_CLK_DIV_VAL_CLKDIVR_MAX                             (0x00000FFFU)

#define CSL_MSS_RCM_MCSPI3_CLK_DIV_VAL_RESETVAL                                (0x00000000U)

/* MCSPI4_CLK_DIV_VAL */

#define CSL_MSS_RCM_MCSPI4_CLK_DIV_VAL_CLKDIVR_MASK                            (0x00000FFFU)
#define CSL_MSS_RCM_MCSPI4_CLK_DIV_VAL_CLKDIVR_SHIFT                           (0x00000000U)
#define CSL_MSS_RCM_MCSPI4_CLK_DIV_VAL_CLKDIVR_RESETVAL                        (0x00000000U)
#define CSL_MSS_RCM_MCSPI4_CLK_DIV_VAL_CLKDIVR_MAX                             (0x00000FFFU)

#define CSL_MSS_RCM_MCSPI4_CLK_DIV_VAL_RESETVAL                                (0x00000000U)

/* MMC0_CLK_DIV_VAL */

#define CSL_MSS_RCM_MMC0_CLK_DIV_VAL_CLKDIVR_MASK                              (0x00000FFFU)
#define CSL_MSS_RCM_MMC0_CLK_DIV_VAL_CLKDIVR_SHIFT                             (0x00000000U)
#define CSL_MSS_RCM_MMC0_CLK_DIV_VAL_CLKDIVR_RESETVAL                          (0x00000000U)
#define CSL_MSS_RCM_MMC0_CLK_DIV_VAL_CLKDIVR_MAX                               (0x00000FFFU)

#define CSL_MSS_RCM_MMC0_CLK_DIV_VAL_RESETVAL                                  (0x00000000U)

/* ICSSM0_UART_CLK_DIV_VAL */

#define CSL_MSS_RCM_ICSSM0_UART_CLK_DIV_VAL_CLKDIVR_MASK                       (0x00000FFFU)
#define CSL_MSS_RCM_ICSSM0_UART_CLK_DIV_VAL_CLKDIVR_SHIFT                      (0x00000000U)
#define CSL_MSS_RCM_ICSSM0_UART_CLK_DIV_VAL_CLKDIVR_RESETVAL                   (0x00000000U)
#define CSL_MSS_RCM_ICSSM0_UART_CLK_DIV_VAL_CLKDIVR_MAX                        (0x00000FFFU)

#define CSL_MSS_RCM_ICSSM0_UART_CLK_DIV_VAL_RESETVAL                           (0x00000000U)

/* CPTS_CLK_DIV_VAL */

#define CSL_MSS_RCM_CPTS_CLK_DIV_VAL_CLKDIVR_MASK                              (0x00000FFFU)
#define CSL_MSS_RCM_CPTS_CLK_DIV_VAL_CLKDIVR_SHIFT                             (0x00000000U)
#define CSL_MSS_RCM_CPTS_CLK_DIV_VAL_CLKDIVR_RESETVAL                          (0x00000000U)
#define CSL_MSS_RCM_CPTS_CLK_DIV_VAL_CLKDIVR_MAX                               (0x00000FFFU)

#define CSL_MSS_RCM_CPTS_CLK_DIV_VAL_RESETVAL                                  (0x00000000U)

/* CONTROLSS_PLL_CLK_DIV_VAL */

#define CSL_MSS_RCM_CONTROLSS_PLL_CLK_DIV_VAL_CLKDIVR_MASK                     (0x00000FFFU)
#define CSL_MSS_RCM_CONTROLSS_PLL_CLK_DIV_VAL_CLKDIVR_SHIFT                    (0x00000000U)
#define CSL_MSS_RCM_CONTROLSS_PLL_CLK_DIV_VAL_CLKDIVR_RESETVAL                 (0x00000000U)
#define CSL_MSS_RCM_CONTROLSS_PLL_CLK_DIV_VAL_CLKDIVR_MAX                      (0x00000FFFU)

#define CSL_MSS_RCM_CONTROLSS_PLL_CLK_DIV_VAL_RESETVAL                         (0x00000000U)

/* I2C_CLK_DIV_VAL */

#define CSL_MSS_RCM_I2C_CLK_DIV_VAL_CLKDIVR_MASK                               (0x00000FFFU)
#define CSL_MSS_RCM_I2C_CLK_DIV_VAL_CLKDIVR_SHIFT                              (0x00000000U)
#define CSL_MSS_RCM_I2C_CLK_DIV_VAL_CLKDIVR_RESETVAL                           (0x00000000U)
#define CSL_MSS_RCM_I2C_CLK_DIV_VAL_CLKDIVR_MAX                                (0x00000FFFU)

#define CSL_MSS_RCM_I2C_CLK_DIV_VAL_RESETVAL                                   (0x00000000U)

/* LIN0_UART0_CLK_DIV_VAL */

#define CSL_MSS_RCM_LIN0_UART0_CLK_DIV_VAL_CLKDIVR_MASK                        (0x00000FFFU)
#define CSL_MSS_RCM_LIN0_UART0_CLK_DIV_VAL_CLKDIVR_SHIFT                       (0x00000000U)
#define CSL_MSS_RCM_LIN0_UART0_CLK_DIV_VAL_CLKDIVR_RESETVAL                    (0x00000000U)
#define CSL_MSS_RCM_LIN0_UART0_CLK_DIV_VAL_CLKDIVR_MAX                         (0x00000FFFU)

#define CSL_MSS_RCM_LIN0_UART0_CLK_DIV_VAL_RESETVAL                            (0x00000000U)

/* LIN1_UART1_CLK_DIV_VAL */

#define CSL_MSS_RCM_LIN1_UART1_CLK_DIV_VAL_CLKDIVR_MASK                        (0x00000FFFU)
#define CSL_MSS_RCM_LIN1_UART1_CLK_DIV_VAL_CLKDIVR_SHIFT                       (0x00000000U)
#define CSL_MSS_RCM_LIN1_UART1_CLK_DIV_VAL_CLKDIVR_RESETVAL                    (0x00000000U)
#define CSL_MSS_RCM_LIN1_UART1_CLK_DIV_VAL_CLKDIVR_MAX                         (0x00000FFFU)

#define CSL_MSS_RCM_LIN1_UART1_CLK_DIV_VAL_RESETVAL                            (0x00000000U)

/* LIN2_UART2_CLK_DIV_VAL */

#define CSL_MSS_RCM_LIN2_UART2_CLK_DIV_VAL_CLKDIVR_MASK                        (0x00000FFFU)
#define CSL_MSS_RCM_LIN2_UART2_CLK_DIV_VAL_CLKDIVR_SHIFT                       (0x00000000U)
#define CSL_MSS_RCM_LIN2_UART2_CLK_DIV_VAL_CLKDIVR_RESETVAL                    (0x00000000U)
#define CSL_MSS_RCM_LIN2_UART2_CLK_DIV_VAL_CLKDIVR_MAX                         (0x00000FFFU)

#define CSL_MSS_RCM_LIN2_UART2_CLK_DIV_VAL_RESETVAL                            (0x00000000U)

/* LIN3_UART3_CLK_DIV_VAL */

#define CSL_MSS_RCM_LIN3_UART3_CLK_DIV_VAL_CLKDIVR_MASK                        (0x00000FFFU)
#define CSL_MSS_RCM_LIN3_UART3_CLK_DIV_VAL_CLKDIVR_SHIFT                       (0x00000000U)
#define CSL_MSS_RCM_LIN3_UART3_CLK_DIV_VAL_CLKDIVR_RESETVAL                    (0x00000000U)
#define CSL_MSS_RCM_LIN3_UART3_CLK_DIV_VAL_CLKDIVR_MAX                         (0x00000FFFU)

#define CSL_MSS_RCM_LIN3_UART3_CLK_DIV_VAL_RESETVAL                            (0x00000000U)

/* LIN4_UART4_CLK_DIV_VAL */

#define CSL_MSS_RCM_LIN4_UART4_CLK_DIV_VAL_CLKDIVR_MASK                        (0x00000FFFU)
#define CSL_MSS_RCM_LIN4_UART4_CLK_DIV_VAL_CLKDIVR_SHIFT                       (0x00000000U)
#define CSL_MSS_RCM_LIN4_UART4_CLK_DIV_VAL_CLKDIVR_RESETVAL                    (0x00000000U)
#define CSL_MSS_RCM_LIN4_UART4_CLK_DIV_VAL_CLKDIVR_MAX                         (0x00000FFFU)

#define CSL_MSS_RCM_LIN4_UART4_CLK_DIV_VAL_RESETVAL                            (0x00000000U)

/* LIN5_UART5_CLK_DIV_VAL */

#define CSL_MSS_RCM_LIN5_UART5_CLK_DIV_VAL_CLKDIVR_MASK                        (0x00000FFFU)
#define CSL_MSS_RCM_LIN5_UART5_CLK_DIV_VAL_CLKDIVR_SHIFT                       (0x00000000U)
#define CSL_MSS_RCM_LIN5_UART5_CLK_DIV_VAL_CLKDIVR_RESETVAL                    (0x00000000U)
#define CSL_MSS_RCM_LIN5_UART5_CLK_DIV_VAL_CLKDIVR_MAX                         (0x00000FFFU)

#define CSL_MSS_RCM_LIN5_UART5_CLK_DIV_VAL_RESETVAL                            (0x00000000U)

/* RGMII_250_CLK_DIV_VAL */

#define CSL_MSS_RCM_RGMII_250_CLK_DIV_VAL_CLKDIVR_MASK                         (0x00000FFFU)
#define CSL_MSS_RCM_RGMII_250_CLK_DIV_VAL_CLKDIVR_SHIFT                        (0x00000000U)
#define CSL_MSS_RCM_RGMII_250_CLK_DIV_VAL_CLKDIVR_RESETVAL                     (0x00000111U)
#define CSL_MSS_RCM_RGMII_250_CLK_DIV_VAL_CLKDIVR_MAX                          (0x00000FFFU)

#define CSL_MSS_RCM_RGMII_250_CLK_DIV_VAL_RESETVAL                             (0x00000111U)

/* RGMII_50_CLK_DIV_VAL */

#define CSL_MSS_RCM_RGMII_50_CLK_DIV_VAL_CLKDIVR_MASK                          (0x00000FFFU)
#define CSL_MSS_RCM_RGMII_50_CLK_DIV_VAL_CLKDIVR_SHIFT                         (0x00000000U)
#define CSL_MSS_RCM_RGMII_50_CLK_DIV_VAL_CLKDIVR_RESETVAL                      (0x00000999U)
#define CSL_MSS_RCM_RGMII_50_CLK_DIV_VAL_CLKDIVR_MAX                           (0x00000FFFU)

#define CSL_MSS_RCM_RGMII_50_CLK_DIV_VAL_RESETVAL                              (0x00000999U)

/* RGMII_5_CLK_DIV_VAL */

#define CSL_MSS_RCM_RGMII_5_CLK_DIV_VAL_CLKDIVR_MASK                           (0x00FFFFFFU)
#define CSL_MSS_RCM_RGMII_5_CLK_DIV_VAL_CLKDIVR_SHIFT                          (0x00000000U)
#define CSL_MSS_RCM_RGMII_5_CLK_DIV_VAL_CLKDIVR_RESETVAL                       (0x00636363U)
#define CSL_MSS_RCM_RGMII_5_CLK_DIV_VAL_CLKDIVR_MAX                            (0x00FFFFFFU)

#define CSL_MSS_RCM_RGMII_5_CLK_DIV_VAL_RESETVAL                               (0x00636363U)

/* XTAL_MMC_32K_CLK_DIV_VAL */

#define CSL_MSS_RCM_XTAL_MMC_32K_CLK_DIV_VAL_CLKDIVR_MASK                      (0x3FFFFFFFU)
#define CSL_MSS_RCM_XTAL_MMC_32K_CLK_DIV_VAL_CLKDIVR_SHIFT                     (0x00000000U)
#define CSL_MSS_RCM_XTAL_MMC_32K_CLK_DIV_VAL_CLKDIVR_RESETVAL                  (0x30CC330CU)
#define CSL_MSS_RCM_XTAL_MMC_32K_CLK_DIV_VAL_CLKDIVR_MAX                       (0x3FFFFFFFU)

#define CSL_MSS_RCM_XTAL_MMC_32K_CLK_DIV_VAL_RESETVAL                          (0x30CC330CU)

/* XTAL_TEMPSENSE_32K_CLK_DIV_VAL */

#define CSL_MSS_RCM_XTAL_TEMPSENSE_32K_CLK_DIV_VAL_CLKDIVR_MASK                (0x3FFFFFFFU)
#define CSL_MSS_RCM_XTAL_TEMPSENSE_32K_CLK_DIV_VAL_CLKDIVR_SHIFT               (0x00000000U)
#define CSL_MSS_RCM_XTAL_TEMPSENSE_32K_CLK_DIV_VAL_CLKDIVR_RESETVAL            (0x30CC330CU)
#define CSL_MSS_RCM_XTAL_TEMPSENSE_32K_CLK_DIV_VAL_CLKDIVR_MAX                 (0x3FFFFFFFU)

#define CSL_MSS_RCM_XTAL_TEMPSENSE_32K_CLK_DIV_VAL_RESETVAL                    (0x30CC330CU)

/* MCAN4_CLK_DIV_VAL */

#define CSL_MSS_RCM_MCAN4_CLK_DIV_VAL_CLKDIVR_MASK                             (0x00000FFFU)
#define CSL_MSS_RCM_MCAN4_CLK_DIV_VAL_CLKDIVR_SHIFT                            (0x00000000U)
#define CSL_MSS_RCM_MCAN4_CLK_DIV_VAL_CLKDIVR_RESETVAL                         (0x00000000U)
#define CSL_MSS_RCM_MCAN4_CLK_DIV_VAL_CLKDIVR_MAX                              (0x00000FFFU)

#define CSL_MSS_RCM_MCAN4_CLK_DIV_VAL_RESETVAL                                 (0x00000000U)

/* MCAN5_CLK_DIV_VAL */

#define CSL_MSS_RCM_MCAN5_CLK_DIV_VAL_CLKDIVR_MASK                             (0x00000FFFU)
#define CSL_MSS_RCM_MCAN5_CLK_DIV_VAL_CLKDIVR_SHIFT                            (0x00000000U)
#define CSL_MSS_RCM_MCAN5_CLK_DIV_VAL_CLKDIVR_RESETVAL                         (0x00000000U)
#define CSL_MSS_RCM_MCAN5_CLK_DIV_VAL_CLKDIVR_MAX                              (0x00000FFFU)

#define CSL_MSS_RCM_MCAN5_CLK_DIV_VAL_RESETVAL                                 (0x00000000U)

/* MCAN6_CLK_DIV_VAL */

#define CSL_MSS_RCM_MCAN6_CLK_DIV_VAL_CLKDIVR_MASK                             (0x00000FFFU)
#define CSL_MSS_RCM_MCAN6_CLK_DIV_VAL_CLKDIVR_SHIFT                            (0x00000000U)
#define CSL_MSS_RCM_MCAN6_CLK_DIV_VAL_CLKDIVR_RESETVAL                         (0x00000000U)
#define CSL_MSS_RCM_MCAN6_CLK_DIV_VAL_CLKDIVR_MAX                              (0x00000FFFU)

#define CSL_MSS_RCM_MCAN6_CLK_DIV_VAL_RESETVAL                                 (0x00000000U)

/* MCAN7_CLK_DIV_VAL */

#define CSL_MSS_RCM_MCAN7_CLK_DIV_VAL_CLKDIVR_MASK                             (0x00000FFFU)
#define CSL_MSS_RCM_MCAN7_CLK_DIV_VAL_CLKDIVR_SHIFT                            (0x00000000U)
#define CSL_MSS_RCM_MCAN7_CLK_DIV_VAL_CLKDIVR_RESETVAL                         (0x00000000U)
#define CSL_MSS_RCM_MCAN7_CLK_DIV_VAL_CLKDIVR_MAX                              (0x00000FFFU)

#define CSL_MSS_RCM_MCAN7_CLK_DIV_VAL_RESETVAL                                 (0x00000000U)

/* RTI4_CLK_DIV_VAL */

#define CSL_MSS_RCM_RTI4_CLK_DIV_VAL_CLKDIVR_MASK                              (0x00000FFFU)
#define CSL_MSS_RCM_RTI4_CLK_DIV_VAL_CLKDIVR_SHIFT                             (0x00000000U)
#define CSL_MSS_RCM_RTI4_CLK_DIV_VAL_CLKDIVR_RESETVAL                          (0x00000000U)
#define CSL_MSS_RCM_RTI4_CLK_DIV_VAL_CLKDIVR_MAX                               (0x00000FFFU)

#define CSL_MSS_RCM_RTI4_CLK_DIV_VAL_RESETVAL                                  (0x00000000U)

/* RTI5_CLK_DIV_VAL */

#define CSL_MSS_RCM_RTI5_CLK_DIV_VAL_CLKDIVR_MASK                              (0x00000FFFU)
#define CSL_MSS_RCM_RTI5_CLK_DIV_VAL_CLKDIVR_SHIFT                             (0x00000000U)
#define CSL_MSS_RCM_RTI5_CLK_DIV_VAL_CLKDIVR_RESETVAL                          (0x00000000U)
#define CSL_MSS_RCM_RTI5_CLK_DIV_VAL_CLKDIVR_MAX                               (0x00000FFFU)

#define CSL_MSS_RCM_RTI5_CLK_DIV_VAL_RESETVAL                                  (0x00000000U)

/* RTI6_CLK_DIV_VAL */

#define CSL_MSS_RCM_RTI6_CLK_DIV_VAL_CLKDIVR_MASK                              (0x00000FFFU)
#define CSL_MSS_RCM_RTI6_CLK_DIV_VAL_CLKDIVR_SHIFT                             (0x00000000U)
#define CSL_MSS_RCM_RTI6_CLK_DIV_VAL_CLKDIVR_RESETVAL                          (0x00000000U)
#define CSL_MSS_RCM_RTI6_CLK_DIV_VAL_CLKDIVR_MAX                               (0x00000FFFU)

#define CSL_MSS_RCM_RTI6_CLK_DIV_VAL_RESETVAL                                  (0x00000000U)

/* RTI7_CLK_DIV_VAL */

#define CSL_MSS_RCM_RTI7_CLK_DIV_VAL_CLKDIVR_MASK                              (0x00000FFFU)
#define CSL_MSS_RCM_RTI7_CLK_DIV_VAL_CLKDIVR_SHIFT                             (0x00000000U)
#define CSL_MSS_RCM_RTI7_CLK_DIV_VAL_CLKDIVR_RESETVAL                          (0x00000000U)
#define CSL_MSS_RCM_RTI7_CLK_DIV_VAL_CLKDIVR_MAX                               (0x00000FFFU)

#define CSL_MSS_RCM_RTI7_CLK_DIV_VAL_RESETVAL                                  (0x00000000U)

/* MCSPI5_CLK_DIV_VAL */

#define CSL_MSS_RCM_MCSPI5_CLK_DIV_VAL_CLKDIVR_MASK                            (0x00000FFFU)
#define CSL_MSS_RCM_MCSPI5_CLK_DIV_VAL_CLKDIVR_SHIFT                           (0x00000000U)
#define CSL_MSS_RCM_MCSPI5_CLK_DIV_VAL_CLKDIVR_RESETVAL                        (0x00000000U)
#define CSL_MSS_RCM_MCSPI5_CLK_DIV_VAL_CLKDIVR_MAX                             (0x00000FFFU)

#define CSL_MSS_RCM_MCSPI5_CLK_DIV_VAL_RESETVAL                                (0x00000000U)

/* MCSPI6_CLK_DIV_VAL */

#define CSL_MSS_RCM_MCSPI6_CLK_DIV_VAL_CLKDIVR_MASK                            (0x00000FFFU)
#define CSL_MSS_RCM_MCSPI6_CLK_DIV_VAL_CLKDIVR_SHIFT                           (0x00000000U)
#define CSL_MSS_RCM_MCSPI6_CLK_DIV_VAL_CLKDIVR_RESETVAL                        (0x00000000U)
#define CSL_MSS_RCM_MCSPI6_CLK_DIV_VAL_CLKDIVR_MAX                             (0x00000FFFU)

#define CSL_MSS_RCM_MCSPI6_CLK_DIV_VAL_RESETVAL                                (0x00000000U)

/* MCSPI7_CLK_DIV_VAL */

#define CSL_MSS_RCM_MCSPI7_CLK_DIV_VAL_CLKDIVR_MASK                            (0x00000FFFU)
#define CSL_MSS_RCM_MCSPI7_CLK_DIV_VAL_CLKDIVR_SHIFT                           (0x00000000U)
#define CSL_MSS_RCM_MCSPI7_CLK_DIV_VAL_CLKDIVR_RESETVAL                        (0x00000000U)
#define CSL_MSS_RCM_MCSPI7_CLK_DIV_VAL_CLKDIVR_MAX                             (0x00000FFFU)

#define CSL_MSS_RCM_MCSPI7_CLK_DIV_VAL_RESETVAL                                (0x00000000U)

/* MCAN0_CLK_GATE */

#define CSL_MSS_RCM_MCAN0_CLK_GATE_GATED_MASK                                  (0x00000007U)
#define CSL_MSS_RCM_MCAN0_CLK_GATE_GATED_SHIFT                                 (0x00000000U)
#define CSL_MSS_RCM_MCAN0_CLK_GATE_GATED_RESETVAL                              (0x00000000U)
#define CSL_MSS_RCM_MCAN0_CLK_GATE_GATED_MAX                                   (0x00000007U)

#define CSL_MSS_RCM_MCAN0_CLK_GATE_RESETVAL                                    (0x00000000U)

/* MCAN1_CLK_GATE */

#define CSL_MSS_RCM_MCAN1_CLK_GATE_GATED_MASK                                  (0x00000007U)
#define CSL_MSS_RCM_MCAN1_CLK_GATE_GATED_SHIFT                                 (0x00000000U)
#define CSL_MSS_RCM_MCAN1_CLK_GATE_GATED_RESETVAL                              (0x00000000U)
#define CSL_MSS_RCM_MCAN1_CLK_GATE_GATED_MAX                                   (0x00000007U)

#define CSL_MSS_RCM_MCAN1_CLK_GATE_RESETVAL                                    (0x00000000U)

/* MCAN2_CLK_GATE */

#define CSL_MSS_RCM_MCAN2_CLK_GATE_GATED_MASK                                  (0x00000007U)
#define CSL_MSS_RCM_MCAN2_CLK_GATE_GATED_SHIFT                                 (0x00000000U)
#define CSL_MSS_RCM_MCAN2_CLK_GATE_GATED_RESETVAL                              (0x00000000U)
#define CSL_MSS_RCM_MCAN2_CLK_GATE_GATED_MAX                                   (0x00000007U)

#define CSL_MSS_RCM_MCAN2_CLK_GATE_RESETVAL                                    (0x00000000U)

/* MCAN3_CLK_GATE */

#define CSL_MSS_RCM_MCAN3_CLK_GATE_GATED_MASK                                  (0x00000007U)
#define CSL_MSS_RCM_MCAN3_CLK_GATE_GATED_SHIFT                                 (0x00000000U)
#define CSL_MSS_RCM_MCAN3_CLK_GATE_GATED_RESETVAL                              (0x00000000U)
#define CSL_MSS_RCM_MCAN3_CLK_GATE_GATED_MAX                                   (0x00000007U)

#define CSL_MSS_RCM_MCAN3_CLK_GATE_RESETVAL                                    (0x00000000U)

/* OSPI_CLK_GATE */

#define CSL_MSS_RCM_OSPI_CLK_GATE_OSPI_CLK_GATE_GATED_MASK                     (0x00000007U)
#define CSL_MSS_RCM_OSPI_CLK_GATE_OSPI_CLK_GATE_GATED_SHIFT                    (0x00000000U)
#define CSL_MSS_RCM_OSPI_CLK_GATE_OSPI_CLK_GATE_GATED_RESETVAL                 (0x00000000U)
#define CSL_MSS_RCM_OSPI_CLK_GATE_OSPI_CLK_GATE_GATED_MAX                      (0x00000007U)

#define CSL_MSS_RCM_OSPI_CLK_GATE_RESETVAL                                     (0x00000000U)

/* RTI0_CLK_GATE */

#define CSL_MSS_RCM_RTI0_CLK_GATE_GATED_MASK                                   (0x00000007U)
#define CSL_MSS_RCM_RTI0_CLK_GATE_GATED_SHIFT                                  (0x00000000U)
#define CSL_MSS_RCM_RTI0_CLK_GATE_GATED_RESETVAL                               (0x00000000U)
#define CSL_MSS_RCM_RTI0_CLK_GATE_GATED_MAX                                    (0x00000007U)

#define CSL_MSS_RCM_RTI0_CLK_GATE_RESETVAL                                     (0x00000000U)

/* RTI1_CLK_GATE */

#define CSL_MSS_RCM_RTI1_CLK_GATE_GATED_MASK                                   (0x00000007U)
#define CSL_MSS_RCM_RTI1_CLK_GATE_GATED_SHIFT                                  (0x00000000U)
#define CSL_MSS_RCM_RTI1_CLK_GATE_GATED_RESETVAL                               (0x00000000U)
#define CSL_MSS_RCM_RTI1_CLK_GATE_GATED_MAX                                    (0x00000007U)

#define CSL_MSS_RCM_RTI1_CLK_GATE_RESETVAL                                     (0x00000000U)

/* RTI2_CLK_GATE */

#define CSL_MSS_RCM_RTI2_CLK_GATE_GATED_MASK                                   (0x00000007U)
#define CSL_MSS_RCM_RTI2_CLK_GATE_GATED_SHIFT                                  (0x00000000U)
#define CSL_MSS_RCM_RTI2_CLK_GATE_GATED_RESETVAL                               (0x00000000U)
#define CSL_MSS_RCM_RTI2_CLK_GATE_GATED_MAX                                    (0x00000007U)

#define CSL_MSS_RCM_RTI2_CLK_GATE_RESETVAL                                     (0x00000000U)

/* RTI3_CLK_GATE */

#define CSL_MSS_RCM_RTI3_CLK_GATE_GATED_MASK                                   (0x00000007U)
#define CSL_MSS_RCM_RTI3_CLK_GATE_GATED_SHIFT                                  (0x00000000U)
#define CSL_MSS_RCM_RTI3_CLK_GATE_GATED_RESETVAL                               (0x00000000U)
#define CSL_MSS_RCM_RTI3_CLK_GATE_GATED_MAX                                    (0x00000007U)

#define CSL_MSS_RCM_RTI3_CLK_GATE_RESETVAL                                     (0x00000000U)

/* WDT0_CLK_GATE */

#define CSL_MSS_RCM_WDT0_CLK_GATE_GATED_MASK                                   (0x00000007U)
#define CSL_MSS_RCM_WDT0_CLK_GATE_GATED_SHIFT                                  (0x00000000U)
#define CSL_MSS_RCM_WDT0_CLK_GATE_GATED_RESETVAL                               (0x00000000U)
#define CSL_MSS_RCM_WDT0_CLK_GATE_GATED_MAX                                    (0x00000007U)

#define CSL_MSS_RCM_WDT0_CLK_GATE_RESETVAL                                     (0x00000000U)

/* WDT1_CLK_GATE */

#define CSL_MSS_RCM_WDT1_CLK_GATE_GATED_MASK                                   (0x00000007U)
#define CSL_MSS_RCM_WDT1_CLK_GATE_GATED_SHIFT                                  (0x00000000U)
#define CSL_MSS_RCM_WDT1_CLK_GATE_GATED_RESETVAL                               (0x00000000U)
#define CSL_MSS_RCM_WDT1_CLK_GATE_GATED_MAX                                    (0x00000007U)

#define CSL_MSS_RCM_WDT1_CLK_GATE_RESETVAL                                     (0x00000000U)

/* WDT2_CLK_GATE */

#define CSL_MSS_RCM_WDT2_CLK_GATE_GATED_MASK                                   (0x00000007U)
#define CSL_MSS_RCM_WDT2_CLK_GATE_GATED_SHIFT                                  (0x00000000U)
#define CSL_MSS_RCM_WDT2_CLK_GATE_GATED_RESETVAL                               (0x00000000U)
#define CSL_MSS_RCM_WDT2_CLK_GATE_GATED_MAX                                    (0x00000007U)

#define CSL_MSS_RCM_WDT2_CLK_GATE_RESETVAL                                     (0x00000000U)

/* WDT3_CLK_GATE */

#define CSL_MSS_RCM_WDT3_CLK_GATE_GATED_MASK                                   (0x00000007U)
#define CSL_MSS_RCM_WDT3_CLK_GATE_GATED_SHIFT                                  (0x00000000U)
#define CSL_MSS_RCM_WDT3_CLK_GATE_GATED_RESETVAL                               (0x00000000U)
#define CSL_MSS_RCM_WDT3_CLK_GATE_GATED_MAX                                    (0x00000007U)

#define CSL_MSS_RCM_WDT3_CLK_GATE_RESETVAL                                     (0x00000000U)

/* MCSPI0_CLK_GATE */

#define CSL_MSS_RCM_MCSPI0_CLK_GATE_GATED_MASK                                 (0x00000007U)
#define CSL_MSS_RCM_MCSPI0_CLK_GATE_GATED_SHIFT                                (0x00000000U)
#define CSL_MSS_RCM_MCSPI0_CLK_GATE_GATED_RESETVAL                             (0x00000000U)
#define CSL_MSS_RCM_MCSPI0_CLK_GATE_GATED_MAX                                  (0x00000007U)

#define CSL_MSS_RCM_MCSPI0_CLK_GATE_RESETVAL                                   (0x00000000U)

/* MCSPI1_CLK_GATE */

#define CSL_MSS_RCM_MCSPI1_CLK_GATE_GATED_MASK                                 (0x00000007U)
#define CSL_MSS_RCM_MCSPI1_CLK_GATE_GATED_SHIFT                                (0x00000000U)
#define CSL_MSS_RCM_MCSPI1_CLK_GATE_GATED_RESETVAL                             (0x00000000U)
#define CSL_MSS_RCM_MCSPI1_CLK_GATE_GATED_MAX                                  (0x00000007U)

#define CSL_MSS_RCM_MCSPI1_CLK_GATE_RESETVAL                                   (0x00000000U)

/* MCSPI2_CLK_GATE */

#define CSL_MSS_RCM_MCSPI2_CLK_GATE_GATED_MASK                                 (0x00000007U)
#define CSL_MSS_RCM_MCSPI2_CLK_GATE_GATED_SHIFT                                (0x00000000U)
#define CSL_MSS_RCM_MCSPI2_CLK_GATE_GATED_RESETVAL                             (0x00000000U)
#define CSL_MSS_RCM_MCSPI2_CLK_GATE_GATED_MAX                                  (0x00000007U)

#define CSL_MSS_RCM_MCSPI2_CLK_GATE_RESETVAL                                   (0x00000000U)

/* MCSPI3_CLK_GATE */

#define CSL_MSS_RCM_MCSPI3_CLK_GATE_GATED_MASK                                 (0x00000007U)
#define CSL_MSS_RCM_MCSPI3_CLK_GATE_GATED_SHIFT                                (0x00000000U)
#define CSL_MSS_RCM_MCSPI3_CLK_GATE_GATED_RESETVAL                             (0x00000000U)
#define CSL_MSS_RCM_MCSPI3_CLK_GATE_GATED_MAX                                  (0x00000007U)

#define CSL_MSS_RCM_MCSPI3_CLK_GATE_RESETVAL                                   (0x00000000U)

/* MCSPI4_CLK_GATE */

#define CSL_MSS_RCM_MCSPI4_CLK_GATE_GATED_MASK                                 (0x00000007U)
#define CSL_MSS_RCM_MCSPI4_CLK_GATE_GATED_SHIFT                                (0x00000000U)
#define CSL_MSS_RCM_MCSPI4_CLK_GATE_GATED_RESETVAL                             (0x00000000U)
#define CSL_MSS_RCM_MCSPI4_CLK_GATE_GATED_MAX                                  (0x00000007U)

#define CSL_MSS_RCM_MCSPI4_CLK_GATE_RESETVAL                                   (0x00000000U)

/* MMC0_CLK_GATE */

#define CSL_MSS_RCM_MMC0_CLK_GATE_GATED_MASK                                   (0x00000007U)
#define CSL_MSS_RCM_MMC0_CLK_GATE_GATED_SHIFT                                  (0x00000000U)
#define CSL_MSS_RCM_MMC0_CLK_GATE_GATED_RESETVAL                               (0x00000000U)
#define CSL_MSS_RCM_MMC0_CLK_GATE_GATED_MAX                                    (0x00000007U)

#define CSL_MSS_RCM_MMC0_CLK_GATE_RESETVAL                                     (0x00000000U)

/* ICSSM0_UART_CLK_GATE */

#define CSL_MSS_RCM_ICSSM0_UART_CLK_GATE_GATED_MASK                            (0x00000007U)
#define CSL_MSS_RCM_ICSSM0_UART_CLK_GATE_GATED_SHIFT                           (0x00000000U)
#define CSL_MSS_RCM_ICSSM0_UART_CLK_GATE_GATED_RESETVAL                        (0x00000000U)
#define CSL_MSS_RCM_ICSSM0_UART_CLK_GATE_GATED_MAX                             (0x00000007U)

#define CSL_MSS_RCM_ICSSM0_UART_CLK_GATE_RESETVAL                              (0x00000000U)

/* CPTS_CLK_GATE */

#define CSL_MSS_RCM_CPTS_CLK_GATE_GATED_MASK                                   (0x00000007U)
#define CSL_MSS_RCM_CPTS_CLK_GATE_GATED_SHIFT                                  (0x00000000U)
#define CSL_MSS_RCM_CPTS_CLK_GATE_GATED_RESETVAL                               (0x00000000U)
#define CSL_MSS_RCM_CPTS_CLK_GATE_GATED_MAX                                    (0x00000007U)

#define CSL_MSS_RCM_CPTS_CLK_GATE_RESETVAL                                     (0x00000000U)

/* CONTROLSS_PLL_CLK_GATE */

#define CSL_MSS_RCM_CONTROLSS_PLL_CLK_GATE_GATED_MASK                          (0x00000007U)
#define CSL_MSS_RCM_CONTROLSS_PLL_CLK_GATE_GATED_SHIFT                         (0x00000000U)
#define CSL_MSS_RCM_CONTROLSS_PLL_CLK_GATE_GATED_RESETVAL                      (0x00000000U)
#define CSL_MSS_RCM_CONTROLSS_PLL_CLK_GATE_GATED_MAX                           (0x00000007U)

#define CSL_MSS_RCM_CONTROLSS_PLL_CLK_GATE_RESETVAL                            (0x00000000U)

/* I2C0_CLK_GATE */

#define CSL_MSS_RCM_I2C0_CLK_GATE_GATED_MASK                                   (0x00000007U)
#define CSL_MSS_RCM_I2C0_CLK_GATE_GATED_SHIFT                                  (0x00000000U)
#define CSL_MSS_RCM_I2C0_CLK_GATE_GATED_RESETVAL                               (0x00000000U)
#define CSL_MSS_RCM_I2C0_CLK_GATE_GATED_MAX                                    (0x00000007U)

#define CSL_MSS_RCM_I2C0_CLK_GATE_RESETVAL                                     (0x00000000U)

/* I2C1_CLK_GATE */

#define CSL_MSS_RCM_I2C1_CLK_GATE_GATED_MASK                                   (0x00000007U)
#define CSL_MSS_RCM_I2C1_CLK_GATE_GATED_SHIFT                                  (0x00000000U)
#define CSL_MSS_RCM_I2C1_CLK_GATE_GATED_RESETVAL                               (0x00000000U)
#define CSL_MSS_RCM_I2C1_CLK_GATE_GATED_MAX                                    (0x00000007U)

#define CSL_MSS_RCM_I2C1_CLK_GATE_RESETVAL                                     (0x00000000U)

/* I2C2_CLK_GATE */

#define CSL_MSS_RCM_I2C2_CLK_GATE_GATED_MASK                                   (0x00000007U)
#define CSL_MSS_RCM_I2C2_CLK_GATE_GATED_SHIFT                                  (0x00000000U)
#define CSL_MSS_RCM_I2C2_CLK_GATE_GATED_RESETVAL                               (0x00000000U)
#define CSL_MSS_RCM_I2C2_CLK_GATE_GATED_MAX                                    (0x00000007U)

#define CSL_MSS_RCM_I2C2_CLK_GATE_RESETVAL                                     (0x00000000U)

/* I2C3_CLK_GATE */

#define CSL_MSS_RCM_I2C3_CLK_GATE_GATED_MASK                                   (0x00000007U)
#define CSL_MSS_RCM_I2C3_CLK_GATE_GATED_SHIFT                                  (0x00000000U)
#define CSL_MSS_RCM_I2C3_CLK_GATE_GATED_RESETVAL                               (0x00000000U)
#define CSL_MSS_RCM_I2C3_CLK_GATE_GATED_MAX                                    (0x00000007U)

#define CSL_MSS_RCM_I2C3_CLK_GATE_RESETVAL                                     (0x00000000U)

/* LIN0_CLK_GATE */

#define CSL_MSS_RCM_LIN0_CLK_GATE_GATED_MASK                                   (0x00000007U)
#define CSL_MSS_RCM_LIN0_CLK_GATE_GATED_SHIFT                                  (0x00000000U)
#define CSL_MSS_RCM_LIN0_CLK_GATE_GATED_RESETVAL                               (0x00000000U)
#define CSL_MSS_RCM_LIN0_CLK_GATE_GATED_MAX                                    (0x00000007U)

#define CSL_MSS_RCM_LIN0_CLK_GATE_RESETVAL                                     (0x00000000U)

/* LIN1_CLK_GATE */

#define CSL_MSS_RCM_LIN1_CLK_GATE_GATED_MASK                                   (0x00000007U)
#define CSL_MSS_RCM_LIN1_CLK_GATE_GATED_SHIFT                                  (0x00000000U)
#define CSL_MSS_RCM_LIN1_CLK_GATE_GATED_RESETVAL                               (0x00000000U)
#define CSL_MSS_RCM_LIN1_CLK_GATE_GATED_MAX                                    (0x00000007U)

#define CSL_MSS_RCM_LIN1_CLK_GATE_RESETVAL                                     (0x00000000U)

/* LIN2_CLK_GATE */

#define CSL_MSS_RCM_LIN2_CLK_GATE_GATED_MASK                                   (0x00000007U)
#define CSL_MSS_RCM_LIN2_CLK_GATE_GATED_SHIFT                                  (0x00000000U)
#define CSL_MSS_RCM_LIN2_CLK_GATE_GATED_RESETVAL                               (0x00000000U)
#define CSL_MSS_RCM_LIN2_CLK_GATE_GATED_MAX                                    (0x00000007U)

#define CSL_MSS_RCM_LIN2_CLK_GATE_RESETVAL                                     (0x00000000U)

/* LIN3_CLK_GATE */

#define CSL_MSS_RCM_LIN3_CLK_GATE_GATED_MASK                                   (0x00000007U)
#define CSL_MSS_RCM_LIN3_CLK_GATE_GATED_SHIFT                                  (0x00000000U)
#define CSL_MSS_RCM_LIN3_CLK_GATE_GATED_RESETVAL                               (0x00000000U)
#define CSL_MSS_RCM_LIN3_CLK_GATE_GATED_MAX                                    (0x00000007U)

#define CSL_MSS_RCM_LIN3_CLK_GATE_RESETVAL                                     (0x00000000U)

/* LIN4_CLK_GATE */

#define CSL_MSS_RCM_LIN4_CLK_GATE_GATED_MASK                                   (0x00000007U)
#define CSL_MSS_RCM_LIN4_CLK_GATE_GATED_SHIFT                                  (0x00000000U)
#define CSL_MSS_RCM_LIN4_CLK_GATE_GATED_RESETVAL                               (0x00000000U)
#define CSL_MSS_RCM_LIN4_CLK_GATE_GATED_MAX                                    (0x00000007U)

#define CSL_MSS_RCM_LIN4_CLK_GATE_RESETVAL                                     (0x00000000U)

/* UART0_CLK_GATE */

#define CSL_MSS_RCM_UART0_CLK_GATE_GATED_MASK                                  (0x00000007U)
#define CSL_MSS_RCM_UART0_CLK_GATE_GATED_SHIFT                                 (0x00000000U)
#define CSL_MSS_RCM_UART0_CLK_GATE_GATED_RESETVAL                              (0x00000000U)
#define CSL_MSS_RCM_UART0_CLK_GATE_GATED_MAX                                   (0x00000007U)

#define CSL_MSS_RCM_UART0_CLK_GATE_RESETVAL                                    (0x00000000U)

/* UART1_CLK_GATE */

#define CSL_MSS_RCM_UART1_CLK_GATE_GATED_MASK                                  (0x00000007U)
#define CSL_MSS_RCM_UART1_CLK_GATE_GATED_SHIFT                                 (0x00000000U)
#define CSL_MSS_RCM_UART1_CLK_GATE_GATED_RESETVAL                              (0x00000000U)
#define CSL_MSS_RCM_UART1_CLK_GATE_GATED_MAX                                   (0x00000007U)

#define CSL_MSS_RCM_UART1_CLK_GATE_RESETVAL                                    (0x00000000U)

/* UART2_CLK_GATE */

#define CSL_MSS_RCM_UART2_CLK_GATE_GATED_MASK                                  (0x00000007U)
#define CSL_MSS_RCM_UART2_CLK_GATE_GATED_SHIFT                                 (0x00000000U)
#define CSL_MSS_RCM_UART2_CLK_GATE_GATED_RESETVAL                              (0x00000000U)
#define CSL_MSS_RCM_UART2_CLK_GATE_GATED_MAX                                   (0x00000007U)

#define CSL_MSS_RCM_UART2_CLK_GATE_RESETVAL                                    (0x00000000U)

/* UART3_CLK_GATE */

#define CSL_MSS_RCM_UART3_CLK_GATE_GATED_MASK                                  (0x00000007U)
#define CSL_MSS_RCM_UART3_CLK_GATE_GATED_SHIFT                                 (0x00000000U)
#define CSL_MSS_RCM_UART3_CLK_GATE_GATED_RESETVAL                              (0x00000000U)
#define CSL_MSS_RCM_UART3_CLK_GATE_GATED_MAX                                   (0x00000007U)

#define CSL_MSS_RCM_UART3_CLK_GATE_RESETVAL                                    (0x00000000U)

/* UART4_CLK_GATE */

#define CSL_MSS_RCM_UART4_CLK_GATE_GATED_MASK                                  (0x00000007U)
#define CSL_MSS_RCM_UART4_CLK_GATE_GATED_SHIFT                                 (0x00000000U)
#define CSL_MSS_RCM_UART4_CLK_GATE_GATED_RESETVAL                              (0x00000000U)
#define CSL_MSS_RCM_UART4_CLK_GATE_GATED_MAX                                   (0x00000007U)

#define CSL_MSS_RCM_UART4_CLK_GATE_RESETVAL                                    (0x00000000U)

/* UART5_CLK_GATE */

#define CSL_MSS_RCM_UART5_CLK_GATE_GATED_MASK                                  (0x00000007U)
#define CSL_MSS_RCM_UART5_CLK_GATE_GATED_SHIFT                                 (0x00000000U)
#define CSL_MSS_RCM_UART5_CLK_GATE_GATED_RESETVAL                              (0x00000000U)
#define CSL_MSS_RCM_UART5_CLK_GATE_GATED_MAX                                   (0x00000007U)

#define CSL_MSS_RCM_UART5_CLK_GATE_RESETVAL                                    (0x00000000U)

/* RGMII_250_CLK_GATE */

#define CSL_MSS_RCM_RGMII_250_CLK_GATE_GATED_MASK                              (0x00000007U)
#define CSL_MSS_RCM_RGMII_250_CLK_GATE_GATED_SHIFT                             (0x00000000U)
#define CSL_MSS_RCM_RGMII_250_CLK_GATE_GATED_RESETVAL                          (0x00000000U)
#define CSL_MSS_RCM_RGMII_250_CLK_GATE_GATED_MAX                               (0x00000007U)

#define CSL_MSS_RCM_RGMII_250_CLK_GATE_RESETVAL                                (0x00000000U)

/* RGMII_50_CLK_GATE */

#define CSL_MSS_RCM_RGMII_50_CLK_GATE_GATED_MASK                               (0x00000007U)
#define CSL_MSS_RCM_RGMII_50_CLK_GATE_GATED_SHIFT                              (0x00000000U)
#define CSL_MSS_RCM_RGMII_50_CLK_GATE_GATED_RESETVAL                           (0x00000000U)
#define CSL_MSS_RCM_RGMII_50_CLK_GATE_GATED_MAX                                (0x00000007U)

#define CSL_MSS_RCM_RGMII_50_CLK_GATE_RESETVAL                                 (0x00000000U)

/* RGMII_5_CLK_GATE */

#define CSL_MSS_RCM_RGMII_5_CLK_GATE_GATED_MASK                                (0x00000007U)
#define CSL_MSS_RCM_RGMII_5_CLK_GATE_GATED_SHIFT                               (0x00000000U)
#define CSL_MSS_RCM_RGMII_5_CLK_GATE_GATED_RESETVAL                            (0x00000000U)
#define CSL_MSS_RCM_RGMII_5_CLK_GATE_GATED_MAX                                 (0x00000007U)

#define CSL_MSS_RCM_RGMII_5_CLK_GATE_RESETVAL                                  (0x00000000U)

/* MMC0_32K_CLK_GATE */

#define CSL_MSS_RCM_MMC0_32K_CLK_GATE_GATED_MASK                               (0x00000007U)
#define CSL_MSS_RCM_MMC0_32K_CLK_GATE_GATED_SHIFT                              (0x00000000U)
#define CSL_MSS_RCM_MMC0_32K_CLK_GATE_GATED_RESETVAL                           (0x00000000U)
#define CSL_MSS_RCM_MMC0_32K_CLK_GATE_GATED_MAX                                (0x00000007U)

#define CSL_MSS_RCM_MMC0_32K_CLK_GATE_RESETVAL                                 (0x00000000U)

/* TEMPSENSE_32K_CLK_GATE */

#define CSL_MSS_RCM_TEMPSENSE_32K_CLK_GATE_GATED_MASK                          (0x00000007U)
#define CSL_MSS_RCM_TEMPSENSE_32K_CLK_GATE_GATED_SHIFT                         (0x00000000U)
#define CSL_MSS_RCM_TEMPSENSE_32K_CLK_GATE_GATED_RESETVAL                      (0x00000000U)
#define CSL_MSS_RCM_TEMPSENSE_32K_CLK_GATE_GATED_MAX                           (0x00000007U)

#define CSL_MSS_RCM_TEMPSENSE_32K_CLK_GATE_RESETVAL                            (0x00000000U)

/* CPSW_CLK_GATE */

#define CSL_MSS_RCM_CPSW_CLK_GATE_GATED_MASK                                   (0x00000007U)
#define CSL_MSS_RCM_CPSW_CLK_GATE_GATED_SHIFT                                  (0x00000000U)
#define CSL_MSS_RCM_CPSW_CLK_GATE_GATED_RESETVAL                               (0x00000000U)
#define CSL_MSS_RCM_CPSW_CLK_GATE_GATED_MAX                                    (0x00000007U)

#define CSL_MSS_RCM_CPSW_CLK_GATE_RESETVAL                                     (0x00000000U)

/* ICSSM0_IEP_CLK_GATE */

#define CSL_MSS_RCM_ICSSM0_IEP_CLK_GATE_GATED_MASK                             (0x00000007U)
#define CSL_MSS_RCM_ICSSM0_IEP_CLK_GATE_GATED_SHIFT                            (0x00000000U)
#define CSL_MSS_RCM_ICSSM0_IEP_CLK_GATE_GATED_RESETVAL                         (0x00000000U)
#define CSL_MSS_RCM_ICSSM0_IEP_CLK_GATE_GATED_MAX                              (0x00000007U)

#define CSL_MSS_RCM_ICSSM0_IEP_CLK_GATE_RESETVAL                               (0x00000000U)

/* ICSSM0_CORE_CLK_GATE */

#define CSL_MSS_RCM_ICSSM0_CORE_CLK_GATE_GATED_MASK                            (0x00000007U)
#define CSL_MSS_RCM_ICSSM0_CORE_CLK_GATE_GATED_SHIFT                           (0x00000000U)
#define CSL_MSS_RCM_ICSSM0_CORE_CLK_GATE_GATED_RESETVAL                        (0x00000000U)
#define CSL_MSS_RCM_ICSSM0_CORE_CLK_GATE_GATED_MAX                             (0x00000007U)

#define CSL_MSS_RCM_ICSSM0_CORE_CLK_GATE_RESETVAL                              (0x00000000U)

/* MSS_ICSSM_SYS_CLK_GATE */

#define CSL_MSS_RCM_MSS_ICSSM_SYS_CLK_GATE_GATED_MASK                          (0x00000007U)
#define CSL_MSS_RCM_MSS_ICSSM_SYS_CLK_GATE_GATED_SHIFT                         (0x00000000U)
#define CSL_MSS_RCM_MSS_ICSSM_SYS_CLK_GATE_GATED_RESETVAL                      (0x00000000U)
#define CSL_MSS_RCM_MSS_ICSSM_SYS_CLK_GATE_GATED_MAX                           (0x00000007U)

#define CSL_MSS_RCM_MSS_ICSSM_SYS_CLK_GATE_RESETVAL                            (0x00000000U)

/* R5SS0_CORE0_GATE */

#define CSL_MSS_RCM_R5SS0_CORE0_GATE_CLKGATE_MASK                              (0x00000007U)
#define CSL_MSS_RCM_R5SS0_CORE0_GATE_CLKGATE_SHIFT                             (0x00000000U)
#define CSL_MSS_RCM_R5SS0_CORE0_GATE_CLKGATE_RESETVAL                          (0x00000000U)
#define CSL_MSS_RCM_R5SS0_CORE0_GATE_CLKGATE_MAX                               (0x00000007U)

#define CSL_MSS_RCM_R5SS0_CORE0_GATE_RESETVAL                                  (0x00000000U)

/* R5SS1_CORE0_GATE */

#define CSL_MSS_RCM_R5SS1_CORE0_GATE_CLKGATE_MASK                              (0x00000007U)
#define CSL_MSS_RCM_R5SS1_CORE0_GATE_CLKGATE_SHIFT                             (0x00000000U)
#define CSL_MSS_RCM_R5SS1_CORE0_GATE_CLKGATE_RESETVAL                          (0x00000000U)
#define CSL_MSS_RCM_R5SS1_CORE0_GATE_CLKGATE_MAX                               (0x00000007U)

#define CSL_MSS_RCM_R5SS1_CORE0_GATE_RESETVAL                                  (0x00000000U)

/* R5SS0_CORE1_GATE */

#define CSL_MSS_RCM_R5SS0_CORE1_GATE_CLKGATE_MASK                              (0x00000007U)
#define CSL_MSS_RCM_R5SS0_CORE1_GATE_CLKGATE_SHIFT                             (0x00000000U)
#define CSL_MSS_RCM_R5SS0_CORE1_GATE_CLKGATE_RESETVAL                          (0x00000000U)
#define CSL_MSS_RCM_R5SS0_CORE1_GATE_CLKGATE_MAX                               (0x00000007U)

#define CSL_MSS_RCM_R5SS0_CORE1_GATE_RESETVAL                                  (0x00000000U)

/* R5SS1_CORE1_GATE */

#define CSL_MSS_RCM_R5SS1_CORE1_GATE_CLKGATE_MASK                              (0x00000007U)
#define CSL_MSS_RCM_R5SS1_CORE1_GATE_CLKGATE_SHIFT                             (0x00000000U)
#define CSL_MSS_RCM_R5SS1_CORE1_GATE_CLKGATE_RESETVAL                          (0x00000000U)
#define CSL_MSS_RCM_R5SS1_CORE1_GATE_CLKGATE_MAX                               (0x00000007U)

#define CSL_MSS_RCM_R5SS1_CORE1_GATE_RESETVAL                                  (0x00000000U)

/* MCAN4_CLK_GATE */

#define CSL_MSS_RCM_MCAN4_CLK_GATE_GATED_MASK                                  (0x00000007U)
#define CSL_MSS_RCM_MCAN4_CLK_GATE_GATED_SHIFT                                 (0x00000000U)
#define CSL_MSS_RCM_MCAN4_CLK_GATE_GATED_RESETVAL                              (0x00000000U)
#define CSL_MSS_RCM_MCAN4_CLK_GATE_GATED_MAX                                   (0x00000007U)

#define CSL_MSS_RCM_MCAN4_CLK_GATE_RESETVAL                                    (0x00000000U)

/* MCAN5_CLK_GATE */

#define CSL_MSS_RCM_MCAN5_CLK_GATE_GATED_MASK                                  (0x00000007U)
#define CSL_MSS_RCM_MCAN5_CLK_GATE_GATED_SHIFT                                 (0x00000000U)
#define CSL_MSS_RCM_MCAN5_CLK_GATE_GATED_RESETVAL                              (0x00000000U)
#define CSL_MSS_RCM_MCAN5_CLK_GATE_GATED_MAX                                   (0x00000007U)

#define CSL_MSS_RCM_MCAN5_CLK_GATE_RESETVAL                                    (0x00000000U)

/* MCAN6_CLK_GATE */

#define CSL_MSS_RCM_MCAN6_CLK_GATE_GATED_MASK                                  (0x00000007U)
#define CSL_MSS_RCM_MCAN6_CLK_GATE_GATED_SHIFT                                 (0x00000000U)
#define CSL_MSS_RCM_MCAN6_CLK_GATE_GATED_RESETVAL                              (0x00000000U)
#define CSL_MSS_RCM_MCAN6_CLK_GATE_GATED_MAX                                   (0x00000007U)

#define CSL_MSS_RCM_MCAN6_CLK_GATE_RESETVAL                                    (0x00000000U)

/* MCAN7_CLK_GATE */

#define CSL_MSS_RCM_MCAN7_CLK_GATE_GATED_MASK                                  (0x00000007U)
#define CSL_MSS_RCM_MCAN7_CLK_GATE_GATED_SHIFT                                 (0x00000000U)
#define CSL_MSS_RCM_MCAN7_CLK_GATE_GATED_RESETVAL                              (0x00000000U)
#define CSL_MSS_RCM_MCAN7_CLK_GATE_GATED_MAX                                   (0x00000007U)

#define CSL_MSS_RCM_MCAN7_CLK_GATE_RESETVAL                                    (0x00000000U)

/* RTI4_CLK_GATE */

#define CSL_MSS_RCM_RTI4_CLK_GATE_GATED_MASK                                   (0x00000007U)
#define CSL_MSS_RCM_RTI4_CLK_GATE_GATED_SHIFT                                  (0x00000000U)
#define CSL_MSS_RCM_RTI4_CLK_GATE_GATED_RESETVAL                               (0x00000000U)
#define CSL_MSS_RCM_RTI4_CLK_GATE_GATED_MAX                                    (0x00000007U)

#define CSL_MSS_RCM_RTI4_CLK_GATE_RESETVAL                                     (0x00000000U)

/* RTI5_CLK_GATE */

#define CSL_MSS_RCM_RTI5_CLK_GATE_GATED_MASK                                   (0x00000007U)
#define CSL_MSS_RCM_RTI5_CLK_GATE_GATED_SHIFT                                  (0x00000000U)
#define CSL_MSS_RCM_RTI5_CLK_GATE_GATED_RESETVAL                               (0x00000000U)
#define CSL_MSS_RCM_RTI5_CLK_GATE_GATED_MAX                                    (0x00000007U)

#define CSL_MSS_RCM_RTI5_CLK_GATE_RESETVAL                                     (0x00000000U)

/* RTI6_CLK_GATE */

#define CSL_MSS_RCM_RTI6_CLK_GATE_GATED_MASK                                   (0x00000007U)
#define CSL_MSS_RCM_RTI6_CLK_GATE_GATED_SHIFT                                  (0x00000000U)
#define CSL_MSS_RCM_RTI6_CLK_GATE_GATED_RESETVAL                               (0x00000000U)
#define CSL_MSS_RCM_RTI6_CLK_GATE_GATED_MAX                                    (0x00000007U)

#define CSL_MSS_RCM_RTI6_CLK_GATE_RESETVAL                                     (0x00000000U)

/* RTI7_CLK_GATE */

#define CSL_MSS_RCM_RTI7_CLK_GATE_GATED_MASK                                   (0x00000007U)
#define CSL_MSS_RCM_RTI7_CLK_GATE_GATED_SHIFT                                  (0x00000000U)
#define CSL_MSS_RCM_RTI7_CLK_GATE_GATED_RESETVAL                               (0x00000000U)
#define CSL_MSS_RCM_RTI7_CLK_GATE_GATED_MAX                                    (0x00000007U)

#define CSL_MSS_RCM_RTI7_CLK_GATE_RESETVAL                                     (0x00000000U)

/* MCAN0_CLK_STATUS */

#define CSL_MSS_RCM_MCAN0_CLK_STATUS_CLKINUSE_MASK                             (0x000000FFU)
#define CSL_MSS_RCM_MCAN0_CLK_STATUS_CLKINUSE_SHIFT                            (0x00000000U)
#define CSL_MSS_RCM_MCAN0_CLK_STATUS_CLKINUSE_RESETVAL                         (0x00000001U)
#define CSL_MSS_RCM_MCAN0_CLK_STATUS_CLKINUSE_MAX                              (0x000000FFU)

#define CSL_MSS_RCM_MCAN0_CLK_STATUS_CURRDIVIDER_MASK                          (0x0000FF00U)
#define CSL_MSS_RCM_MCAN0_CLK_STATUS_CURRDIVIDER_SHIFT                         (0x00000008U)
#define CSL_MSS_RCM_MCAN0_CLK_STATUS_CURRDIVIDER_RESETVAL                      (0x00000000U)
#define CSL_MSS_RCM_MCAN0_CLK_STATUS_CURRDIVIDER_MAX                           (0x000000FFU)

#define CSL_MSS_RCM_MCAN0_CLK_STATUS_RESETVAL                                  (0x00000001U)

/* MCAN1_CLK_STATUS */

#define CSL_MSS_RCM_MCAN1_CLK_STATUS_CLKINUSE_MASK                             (0x000000FFU)
#define CSL_MSS_RCM_MCAN1_CLK_STATUS_CLKINUSE_SHIFT                            (0x00000000U)
#define CSL_MSS_RCM_MCAN1_CLK_STATUS_CLKINUSE_RESETVAL                         (0x00000001U)
#define CSL_MSS_RCM_MCAN1_CLK_STATUS_CLKINUSE_MAX                              (0x000000FFU)

#define CSL_MSS_RCM_MCAN1_CLK_STATUS_CURRDIVIDER_MASK                          (0x0000FF00U)
#define CSL_MSS_RCM_MCAN1_CLK_STATUS_CURRDIVIDER_SHIFT                         (0x00000008U)
#define CSL_MSS_RCM_MCAN1_CLK_STATUS_CURRDIVIDER_RESETVAL                      (0x00000000U)
#define CSL_MSS_RCM_MCAN1_CLK_STATUS_CURRDIVIDER_MAX                           (0x000000FFU)

#define CSL_MSS_RCM_MCAN1_CLK_STATUS_RESETVAL                                  (0x00000001U)

/* MCAN2_CLK_STATUS */

#define CSL_MSS_RCM_MCAN2_CLK_STATUS_CLKINUSE_MASK                             (0x000000FFU)
#define CSL_MSS_RCM_MCAN2_CLK_STATUS_CLKINUSE_SHIFT                            (0x00000000U)
#define CSL_MSS_RCM_MCAN2_CLK_STATUS_CLKINUSE_RESETVAL                         (0x00000001U)
#define CSL_MSS_RCM_MCAN2_CLK_STATUS_CLKINUSE_MAX                              (0x000000FFU)

#define CSL_MSS_RCM_MCAN2_CLK_STATUS_CURRDIVIDER_MASK                          (0x0000FF00U)
#define CSL_MSS_RCM_MCAN2_CLK_STATUS_CURRDIVIDER_SHIFT                         (0x00000008U)
#define CSL_MSS_RCM_MCAN2_CLK_STATUS_CURRDIVIDER_RESETVAL                      (0x00000000U)
#define CSL_MSS_RCM_MCAN2_CLK_STATUS_CURRDIVIDER_MAX                           (0x000000FFU)

#define CSL_MSS_RCM_MCAN2_CLK_STATUS_RESETVAL                                  (0x00000001U)

/* MCAN3_CLK_STATUS */

#define CSL_MSS_RCM_MCAN3_CLK_STATUS_CLKINUSE_MASK                             (0x000000FFU)
#define CSL_MSS_RCM_MCAN3_CLK_STATUS_CLKINUSE_SHIFT                            (0x00000000U)
#define CSL_MSS_RCM_MCAN3_CLK_STATUS_CLKINUSE_RESETVAL                         (0x00000001U)
#define CSL_MSS_RCM_MCAN3_CLK_STATUS_CLKINUSE_MAX                              (0x000000FFU)

#define CSL_MSS_RCM_MCAN3_CLK_STATUS_CURRDIVIDER_MASK                          (0x0000FF00U)
#define CSL_MSS_RCM_MCAN3_CLK_STATUS_CURRDIVIDER_SHIFT                         (0x00000008U)
#define CSL_MSS_RCM_MCAN3_CLK_STATUS_CURRDIVIDER_RESETVAL                      (0x00000000U)
#define CSL_MSS_RCM_MCAN3_CLK_STATUS_CURRDIVIDER_MAX                           (0x000000FFU)

#define CSL_MSS_RCM_MCAN3_CLK_STATUS_RESETVAL                                  (0x00000001U)

/* OSPI_CLK_STATUS */

#define CSL_MSS_RCM_OSPI_CLK_STATUS_CLKINUSE_MASK                              (0x000000FFU)
#define CSL_MSS_RCM_OSPI_CLK_STATUS_CLKINUSE_SHIFT                             (0x00000000U)
#define CSL_MSS_RCM_OSPI_CLK_STATUS_CLKINUSE_RESETVAL                          (0x00000001U)
#define CSL_MSS_RCM_OSPI_CLK_STATUS_CLKINUSE_MAX                               (0x000000FFU)

#define CSL_MSS_RCM_OSPI_CLK_STATUS_CURRDIVIDER_MASK                           (0x0000FF00U)
#define CSL_MSS_RCM_OSPI_CLK_STATUS_CURRDIVIDER_SHIFT                          (0x00000008U)
#define CSL_MSS_RCM_OSPI_CLK_STATUS_CURRDIVIDER_RESETVAL                       (0x00000000U)
#define CSL_MSS_RCM_OSPI_CLK_STATUS_CURRDIVIDER_MAX                            (0x000000FFU)

#define CSL_MSS_RCM_OSPI_CLK_STATUS_RESETVAL                                   (0x00000001U)

/* RTI0_CLK_STATUS */

#define CSL_MSS_RCM_RTI0_CLK_STATUS_CLKINUSE_MASK                              (0x000000FFU)
#define CSL_MSS_RCM_RTI0_CLK_STATUS_CLKINUSE_SHIFT                             (0x00000000U)
#define CSL_MSS_RCM_RTI0_CLK_STATUS_CLKINUSE_RESETVAL                          (0x00000001U)
#define CSL_MSS_RCM_RTI0_CLK_STATUS_CLKINUSE_MAX                               (0x000000FFU)

#define CSL_MSS_RCM_RTI0_CLK_STATUS_CURRDIVIDER_MASK                           (0x0000FF00U)
#define CSL_MSS_RCM_RTI0_CLK_STATUS_CURRDIVIDER_SHIFT                          (0x00000008U)
#define CSL_MSS_RCM_RTI0_CLK_STATUS_CURRDIVIDER_RESETVAL                       (0x00000000U)
#define CSL_MSS_RCM_RTI0_CLK_STATUS_CURRDIVIDER_MAX                            (0x000000FFU)

#define CSL_MSS_RCM_RTI0_CLK_STATUS_RESETVAL                                   (0x00000001U)

/* RTI1_CLK_STATUS */

#define CSL_MSS_RCM_RTI1_CLK_STATUS_CLKINUSE_MASK                              (0x000000FFU)
#define CSL_MSS_RCM_RTI1_CLK_STATUS_CLKINUSE_SHIFT                             (0x00000000U)
#define CSL_MSS_RCM_RTI1_CLK_STATUS_CLKINUSE_RESETVAL                          (0x00000001U)
#define CSL_MSS_RCM_RTI1_CLK_STATUS_CLKINUSE_MAX                               (0x000000FFU)

#define CSL_MSS_RCM_RTI1_CLK_STATUS_CURRDIVIDER_MASK                           (0x0000FF00U)
#define CSL_MSS_RCM_RTI1_CLK_STATUS_CURRDIVIDER_SHIFT                          (0x00000008U)
#define CSL_MSS_RCM_RTI1_CLK_STATUS_CURRDIVIDER_RESETVAL                       (0x00000000U)
#define CSL_MSS_RCM_RTI1_CLK_STATUS_CURRDIVIDER_MAX                            (0x000000FFU)

#define CSL_MSS_RCM_RTI1_CLK_STATUS_RESETVAL                                   (0x00000001U)

/* RTI2_CLK_STATUS */

#define CSL_MSS_RCM_RTI2_CLK_STATUS_CLKINUSE_MASK                              (0x000000FFU)
#define CSL_MSS_RCM_RTI2_CLK_STATUS_CLKINUSE_SHIFT                             (0x00000000U)
#define CSL_MSS_RCM_RTI2_CLK_STATUS_CLKINUSE_RESETVAL                          (0x00000001U)
#define CSL_MSS_RCM_RTI2_CLK_STATUS_CLKINUSE_MAX                               (0x000000FFU)

#define CSL_MSS_RCM_RTI2_CLK_STATUS_CURRDIVIDER_MASK                           (0x0000FF00U)
#define CSL_MSS_RCM_RTI2_CLK_STATUS_CURRDIVIDER_SHIFT                          (0x00000008U)
#define CSL_MSS_RCM_RTI2_CLK_STATUS_CURRDIVIDER_RESETVAL                       (0x00000000U)
#define CSL_MSS_RCM_RTI2_CLK_STATUS_CURRDIVIDER_MAX                            (0x000000FFU)

#define CSL_MSS_RCM_RTI2_CLK_STATUS_RESETVAL                                   (0x00000001U)

/* RTI3_CLK_STATUS */

#define CSL_MSS_RCM_RTI3_CLK_STATUS_CLKINUSE_MASK                              (0x000000FFU)
#define CSL_MSS_RCM_RTI3_CLK_STATUS_CLKINUSE_SHIFT                             (0x00000000U)
#define CSL_MSS_RCM_RTI3_CLK_STATUS_CLKINUSE_RESETVAL                          (0x00000001U)
#define CSL_MSS_RCM_RTI3_CLK_STATUS_CLKINUSE_MAX                               (0x000000FFU)

#define CSL_MSS_RCM_RTI3_CLK_STATUS_CURRDIVIDER_MASK                           (0x0000FF00U)
#define CSL_MSS_RCM_RTI3_CLK_STATUS_CURRDIVIDER_SHIFT                          (0x00000008U)
#define CSL_MSS_RCM_RTI3_CLK_STATUS_CURRDIVIDER_RESETVAL                       (0x00000000U)
#define CSL_MSS_RCM_RTI3_CLK_STATUS_CURRDIVIDER_MAX                            (0x000000FFU)

#define CSL_MSS_RCM_RTI3_CLK_STATUS_RESETVAL                                   (0x00000001U)

/* WDT0_CLK_STATUS */

#define CSL_MSS_RCM_WDT0_CLK_STATUS_CLKINUSE_MASK                              (0x000000FFU)
#define CSL_MSS_RCM_WDT0_CLK_STATUS_CLKINUSE_SHIFT                             (0x00000000U)
#define CSL_MSS_RCM_WDT0_CLK_STATUS_CLKINUSE_RESETVAL                          (0x00000001U)
#define CSL_MSS_RCM_WDT0_CLK_STATUS_CLKINUSE_MAX                               (0x000000FFU)

#define CSL_MSS_RCM_WDT0_CLK_STATUS_CURRDIVIDER_MASK                           (0x0000FF00U)
#define CSL_MSS_RCM_WDT0_CLK_STATUS_CURRDIVIDER_SHIFT                          (0x00000008U)
#define CSL_MSS_RCM_WDT0_CLK_STATUS_CURRDIVIDER_RESETVAL                       (0x00000000U)
#define CSL_MSS_RCM_WDT0_CLK_STATUS_CURRDIVIDER_MAX                            (0x000000FFU)

#define CSL_MSS_RCM_WDT0_CLK_STATUS_RESETVAL                                   (0x00000001U)

/* WDT1_CLK_STATUS */

#define CSL_MSS_RCM_WDT1_CLK_STATUS_CLKINUSE_MASK                              (0x000000FFU)
#define CSL_MSS_RCM_WDT1_CLK_STATUS_CLKINUSE_SHIFT                             (0x00000000U)
#define CSL_MSS_RCM_WDT1_CLK_STATUS_CLKINUSE_RESETVAL                          (0x00000001U)
#define CSL_MSS_RCM_WDT1_CLK_STATUS_CLKINUSE_MAX                               (0x000000FFU)

#define CSL_MSS_RCM_WDT1_CLK_STATUS_CURRDIVIDER_MASK                           (0x0000FF00U)
#define CSL_MSS_RCM_WDT1_CLK_STATUS_CURRDIVIDER_SHIFT                          (0x00000008U)
#define CSL_MSS_RCM_WDT1_CLK_STATUS_CURRDIVIDER_RESETVAL                       (0x00000000U)
#define CSL_MSS_RCM_WDT1_CLK_STATUS_CURRDIVIDER_MAX                            (0x000000FFU)

#define CSL_MSS_RCM_WDT1_CLK_STATUS_RESETVAL                                   (0x00000001U)

/* WDT2_CLK_STATUS */

#define CSL_MSS_RCM_WDT2_CLK_STATUS_CLKINUSE_MASK                              (0x000000FFU)
#define CSL_MSS_RCM_WDT2_CLK_STATUS_CLKINUSE_SHIFT                             (0x00000000U)
#define CSL_MSS_RCM_WDT2_CLK_STATUS_CLKINUSE_RESETVAL                          (0x00000001U)
#define CSL_MSS_RCM_WDT2_CLK_STATUS_CLKINUSE_MAX                               (0x000000FFU)

#define CSL_MSS_RCM_WDT2_CLK_STATUS_CURRDIVIDER_MASK                           (0x0000FF00U)
#define CSL_MSS_RCM_WDT2_CLK_STATUS_CURRDIVIDER_SHIFT                          (0x00000008U)
#define CSL_MSS_RCM_WDT2_CLK_STATUS_CURRDIVIDER_RESETVAL                       (0x00000000U)
#define CSL_MSS_RCM_WDT2_CLK_STATUS_CURRDIVIDER_MAX                            (0x000000FFU)

#define CSL_MSS_RCM_WDT2_CLK_STATUS_RESETVAL                                   (0x00000001U)

/* WDT3_CLK_STATUS */

#define CSL_MSS_RCM_WDT3_CLK_STATUS_CLKINUSE_MASK                              (0x000000FFU)
#define CSL_MSS_RCM_WDT3_CLK_STATUS_CLKINUSE_SHIFT                             (0x00000000U)
#define CSL_MSS_RCM_WDT3_CLK_STATUS_CLKINUSE_RESETVAL                          (0x00000001U)
#define CSL_MSS_RCM_WDT3_CLK_STATUS_CLKINUSE_MAX                               (0x000000FFU)

#define CSL_MSS_RCM_WDT3_CLK_STATUS_CURRDIVIDER_MASK                           (0x0000FF00U)
#define CSL_MSS_RCM_WDT3_CLK_STATUS_CURRDIVIDER_SHIFT                          (0x00000008U)
#define CSL_MSS_RCM_WDT3_CLK_STATUS_CURRDIVIDER_RESETVAL                       (0x00000000U)
#define CSL_MSS_RCM_WDT3_CLK_STATUS_CURRDIVIDER_MAX                            (0x000000FFU)

#define CSL_MSS_RCM_WDT3_CLK_STATUS_RESETVAL                                   (0x00000001U)

/* MCSPI0_CLK_STATUS */

#define CSL_MSS_RCM_MCSPI0_CLK_STATUS_CLKINUSE_MASK                            (0x000000FFU)
#define CSL_MSS_RCM_MCSPI0_CLK_STATUS_CLKINUSE_SHIFT                           (0x00000000U)
#define CSL_MSS_RCM_MCSPI0_CLK_STATUS_CLKINUSE_RESETVAL                        (0x00000001U)
#define CSL_MSS_RCM_MCSPI0_CLK_STATUS_CLKINUSE_MAX                             (0x000000FFU)

#define CSL_MSS_RCM_MCSPI0_CLK_STATUS_CURRDIVIDER_MASK                         (0x0000FF00U)
#define CSL_MSS_RCM_MCSPI0_CLK_STATUS_CURRDIVIDER_SHIFT                        (0x00000008U)
#define CSL_MSS_RCM_MCSPI0_CLK_STATUS_CURRDIVIDER_RESETVAL                     (0x00000000U)
#define CSL_MSS_RCM_MCSPI0_CLK_STATUS_CURRDIVIDER_MAX                          (0x000000FFU)

#define CSL_MSS_RCM_MCSPI0_CLK_STATUS_RESETVAL                                 (0x00000001U)

/* MCSPI1_CLK_STATUS */

#define CSL_MSS_RCM_MCSPI1_CLK_STATUS_CLKINUSE_MASK                            (0x000000FFU)
#define CSL_MSS_RCM_MCSPI1_CLK_STATUS_CLKINUSE_SHIFT                           (0x00000000U)
#define CSL_MSS_RCM_MCSPI1_CLK_STATUS_CLKINUSE_RESETVAL                        (0x00000001U)
#define CSL_MSS_RCM_MCSPI1_CLK_STATUS_CLKINUSE_MAX                             (0x000000FFU)

#define CSL_MSS_RCM_MCSPI1_CLK_STATUS_CURRDIVIDER_MASK                         (0x0000FF00U)
#define CSL_MSS_RCM_MCSPI1_CLK_STATUS_CURRDIVIDER_SHIFT                        (0x00000008U)
#define CSL_MSS_RCM_MCSPI1_CLK_STATUS_CURRDIVIDER_RESETVAL                     (0x00000000U)
#define CSL_MSS_RCM_MCSPI1_CLK_STATUS_CURRDIVIDER_MAX                          (0x000000FFU)

#define CSL_MSS_RCM_MCSPI1_CLK_STATUS_RESETVAL                                 (0x00000001U)

/* MCSPI2_CLK_STATUS */

#define CSL_MSS_RCM_MCSPI2_CLK_STATUS_CLKINUSE_MASK                            (0x000000FFU)
#define CSL_MSS_RCM_MCSPI2_CLK_STATUS_CLKINUSE_SHIFT                           (0x00000000U)
#define CSL_MSS_RCM_MCSPI2_CLK_STATUS_CLKINUSE_RESETVAL                        (0x00000001U)
#define CSL_MSS_RCM_MCSPI2_CLK_STATUS_CLKINUSE_MAX                             (0x000000FFU)

#define CSL_MSS_RCM_MCSPI2_CLK_STATUS_CURRDIVIDER_MASK                         (0x0000FF00U)
#define CSL_MSS_RCM_MCSPI2_CLK_STATUS_CURRDIVIDER_SHIFT                        (0x00000008U)
#define CSL_MSS_RCM_MCSPI2_CLK_STATUS_CURRDIVIDER_RESETVAL                     (0x00000000U)
#define CSL_MSS_RCM_MCSPI2_CLK_STATUS_CURRDIVIDER_MAX                          (0x000000FFU)

#define CSL_MSS_RCM_MCSPI2_CLK_STATUS_RESETVAL                                 (0x00000001U)

/* MCSPI3_CLK_STATUS */

#define CSL_MSS_RCM_MCSPI3_CLK_STATUS_CLKINUSE_MASK                            (0x000000FFU)
#define CSL_MSS_RCM_MCSPI3_CLK_STATUS_CLKINUSE_SHIFT                           (0x00000000U)
#define CSL_MSS_RCM_MCSPI3_CLK_STATUS_CLKINUSE_RESETVAL                        (0x00000001U)
#define CSL_MSS_RCM_MCSPI3_CLK_STATUS_CLKINUSE_MAX                             (0x000000FFU)

#define CSL_MSS_RCM_MCSPI3_CLK_STATUS_CURRDIVIDER_MASK                         (0x0000FF00U)
#define CSL_MSS_RCM_MCSPI3_CLK_STATUS_CURRDIVIDER_SHIFT                        (0x00000008U)
#define CSL_MSS_RCM_MCSPI3_CLK_STATUS_CURRDIVIDER_RESETVAL                     (0x00000000U)
#define CSL_MSS_RCM_MCSPI3_CLK_STATUS_CURRDIVIDER_MAX                          (0x000000FFU)

#define CSL_MSS_RCM_MCSPI3_CLK_STATUS_RESETVAL                                 (0x00000001U)

/* MCSPI4_CLK_STATUS */

#define CSL_MSS_RCM_MCSPI4_CLK_STATUS_CLKINUSE_MASK                            (0x000000FFU)
#define CSL_MSS_RCM_MCSPI4_CLK_STATUS_CLKINUSE_SHIFT                           (0x00000000U)
#define CSL_MSS_RCM_MCSPI4_CLK_STATUS_CLKINUSE_RESETVAL                        (0x00000001U)
#define CSL_MSS_RCM_MCSPI4_CLK_STATUS_CLKINUSE_MAX                             (0x000000FFU)

#define CSL_MSS_RCM_MCSPI4_CLK_STATUS_CURRDIVIDER_MASK                         (0x0000FF00U)
#define CSL_MSS_RCM_MCSPI4_CLK_STATUS_CURRDIVIDER_SHIFT                        (0x00000008U)
#define CSL_MSS_RCM_MCSPI4_CLK_STATUS_CURRDIVIDER_RESETVAL                     (0x00000000U)
#define CSL_MSS_RCM_MCSPI4_CLK_STATUS_CURRDIVIDER_MAX                          (0x000000FFU)

#define CSL_MSS_RCM_MCSPI4_CLK_STATUS_RESETVAL                                 (0x00000001U)

/* MMC0_CLK_STATUS */

#define CSL_MSS_RCM_MMC0_CLK_STATUS_CLKINUSE_MASK                              (0x000000FFU)
#define CSL_MSS_RCM_MMC0_CLK_STATUS_CLKINUSE_SHIFT                             (0x00000000U)
#define CSL_MSS_RCM_MMC0_CLK_STATUS_CLKINUSE_RESETVAL                          (0x00000001U)
#define CSL_MSS_RCM_MMC0_CLK_STATUS_CLKINUSE_MAX                               (0x000000FFU)

#define CSL_MSS_RCM_MMC0_CLK_STATUS_CURRDIVIDER_MASK                           (0x0000FF00U)
#define CSL_MSS_RCM_MMC0_CLK_STATUS_CURRDIVIDER_SHIFT                          (0x00000008U)
#define CSL_MSS_RCM_MMC0_CLK_STATUS_CURRDIVIDER_RESETVAL                       (0x00000000U)
#define CSL_MSS_RCM_MMC0_CLK_STATUS_CURRDIVIDER_MAX                            (0x000000FFU)

#define CSL_MSS_RCM_MMC0_CLK_STATUS_RESETVAL                                   (0x00000001U)

/* ICSSM0_UART_CLK_STATUS */

#define CSL_MSS_RCM_ICSSM0_UART_CLK_STATUS_CLKINUSE_MASK                       (0x000000FFU)
#define CSL_MSS_RCM_ICSSM0_UART_CLK_STATUS_CLKINUSE_SHIFT                      (0x00000000U)
#define CSL_MSS_RCM_ICSSM0_UART_CLK_STATUS_CLKINUSE_RESETVAL                   (0x00000001U)
#define CSL_MSS_RCM_ICSSM0_UART_CLK_STATUS_CLKINUSE_MAX                        (0x000000FFU)

#define CSL_MSS_RCM_ICSSM0_UART_CLK_STATUS_CURRDIVIDER_MASK                    (0x0000FF00U)
#define CSL_MSS_RCM_ICSSM0_UART_CLK_STATUS_CURRDIVIDER_SHIFT                   (0x00000008U)
#define CSL_MSS_RCM_ICSSM0_UART_CLK_STATUS_CURRDIVIDER_RESETVAL                (0x00000000U)
#define CSL_MSS_RCM_ICSSM0_UART_CLK_STATUS_CURRDIVIDER_MAX                     (0x000000FFU)

#define CSL_MSS_RCM_ICSSM0_UART_CLK_STATUS_RESETVAL                            (0x00000001U)

/* CPTS_CLK_STATUS */

#define CSL_MSS_RCM_CPTS_CLK_STATUS_CLKINUSE_MASK                              (0x000000FFU)
#define CSL_MSS_RCM_CPTS_CLK_STATUS_CLKINUSE_SHIFT                             (0x00000000U)
#define CSL_MSS_RCM_CPTS_CLK_STATUS_CLKINUSE_RESETVAL                          (0x00000001U)
#define CSL_MSS_RCM_CPTS_CLK_STATUS_CLKINUSE_MAX                               (0x000000FFU)

#define CSL_MSS_RCM_CPTS_CLK_STATUS_CURRDIVIDER_MASK                           (0x0000FF00U)
#define CSL_MSS_RCM_CPTS_CLK_STATUS_CURRDIVIDER_SHIFT                          (0x00000008U)
#define CSL_MSS_RCM_CPTS_CLK_STATUS_CURRDIVIDER_RESETVAL                       (0x00000000U)
#define CSL_MSS_RCM_CPTS_CLK_STATUS_CURRDIVIDER_MAX                            (0x000000FFU)

#define CSL_MSS_RCM_CPTS_CLK_STATUS_RESETVAL                                   (0x00000001U)

/* CONTROLSS_PLL_CLK_STATUS */

#define CSL_MSS_RCM_CONTROLSS_PLL_CLK_STATUS_CLKINUSE_MASK                     (0x000000FFU)
#define CSL_MSS_RCM_CONTROLSS_PLL_CLK_STATUS_CLKINUSE_SHIFT                    (0x00000000U)
#define CSL_MSS_RCM_CONTROLSS_PLL_CLK_STATUS_CLKINUSE_RESETVAL                 (0x00000001U)
#define CSL_MSS_RCM_CONTROLSS_PLL_CLK_STATUS_CLKINUSE_MAX                      (0x000000FFU)

#define CSL_MSS_RCM_CONTROLSS_PLL_CLK_STATUS_CURRDIVIDER_MASK                  (0x0000FF00U)
#define CSL_MSS_RCM_CONTROLSS_PLL_CLK_STATUS_CURRDIVIDER_SHIFT                 (0x00000008U)
#define CSL_MSS_RCM_CONTROLSS_PLL_CLK_STATUS_CURRDIVIDER_RESETVAL              (0x00000000U)
#define CSL_MSS_RCM_CONTROLSS_PLL_CLK_STATUS_CURRDIVIDER_MAX                   (0x000000FFU)

#define CSL_MSS_RCM_CONTROLSS_PLL_CLK_STATUS_RESETVAL                          (0x00000001U)

/* I2C_CLK_STATUS */

#define CSL_MSS_RCM_I2C_CLK_STATUS_CLKINUSE_MASK                               (0x000000FFU)
#define CSL_MSS_RCM_I2C_CLK_STATUS_CLKINUSE_SHIFT                              (0x00000000U)
#define CSL_MSS_RCM_I2C_CLK_STATUS_CLKINUSE_RESETVAL                           (0x00000001U)
#define CSL_MSS_RCM_I2C_CLK_STATUS_CLKINUSE_MAX                                (0x000000FFU)

#define CSL_MSS_RCM_I2C_CLK_STATUS_CURRDIVIDER_MASK                            (0x0000FF00U)
#define CSL_MSS_RCM_I2C_CLK_STATUS_CURRDIVIDER_SHIFT                           (0x00000008U)
#define CSL_MSS_RCM_I2C_CLK_STATUS_CURRDIVIDER_RESETVAL                        (0x00000000U)
#define CSL_MSS_RCM_I2C_CLK_STATUS_CURRDIVIDER_MAX                             (0x000000FFU)

#define CSL_MSS_RCM_I2C_CLK_STATUS_RESETVAL                                    (0x00000001U)

/* LIN0_UART0_CLK_STATUS */

#define CSL_MSS_RCM_LIN0_UART0_CLK_STATUS_CLKINUSE_MASK                        (0x000000FFU)
#define CSL_MSS_RCM_LIN0_UART0_CLK_STATUS_CLKINUSE_SHIFT                       (0x00000000U)
#define CSL_MSS_RCM_LIN0_UART0_CLK_STATUS_CLKINUSE_RESETVAL                    (0x00000001U)
#define CSL_MSS_RCM_LIN0_UART0_CLK_STATUS_CLKINUSE_MAX                         (0x000000FFU)

#define CSL_MSS_RCM_LIN0_UART0_CLK_STATUS_CURRDIVIDER_MASK                     (0x0000FF00U)
#define CSL_MSS_RCM_LIN0_UART0_CLK_STATUS_CURRDIVIDER_SHIFT                    (0x00000008U)
#define CSL_MSS_RCM_LIN0_UART0_CLK_STATUS_CURRDIVIDER_RESETVAL                 (0x00000000U)
#define CSL_MSS_RCM_LIN0_UART0_CLK_STATUS_CURRDIVIDER_MAX                      (0x000000FFU)

#define CSL_MSS_RCM_LIN0_UART0_CLK_STATUS_RESETVAL                             (0x00000001U)

/* LIN1_UART1_CLK_STATUS */

#define CSL_MSS_RCM_LIN1_UART1_CLK_STATUS_CLKINUSE_MASK                        (0x000000FFU)
#define CSL_MSS_RCM_LIN1_UART1_CLK_STATUS_CLKINUSE_SHIFT                       (0x00000000U)
#define CSL_MSS_RCM_LIN1_UART1_CLK_STATUS_CLKINUSE_RESETVAL                    (0x00000001U)
#define CSL_MSS_RCM_LIN1_UART1_CLK_STATUS_CLKINUSE_MAX                         (0x000000FFU)

#define CSL_MSS_RCM_LIN1_UART1_CLK_STATUS_CURRDIVIDER_MASK                     (0x0000FF00U)
#define CSL_MSS_RCM_LIN1_UART1_CLK_STATUS_CURRDIVIDER_SHIFT                    (0x00000008U)
#define CSL_MSS_RCM_LIN1_UART1_CLK_STATUS_CURRDIVIDER_RESETVAL                 (0x00000000U)
#define CSL_MSS_RCM_LIN1_UART1_CLK_STATUS_CURRDIVIDER_MAX                      (0x000000FFU)

#define CSL_MSS_RCM_LIN1_UART1_CLK_STATUS_RESETVAL                             (0x00000001U)

/* LIN2_UART2_CLK_STATUS */

#define CSL_MSS_RCM_LIN2_UART2_CLK_STATUS_CLKINUSE_MASK                        (0x000000FFU)
#define CSL_MSS_RCM_LIN2_UART2_CLK_STATUS_CLKINUSE_SHIFT                       (0x00000000U)
#define CSL_MSS_RCM_LIN2_UART2_CLK_STATUS_CLKINUSE_RESETVAL                    (0x00000001U)
#define CSL_MSS_RCM_LIN2_UART2_CLK_STATUS_CLKINUSE_MAX                         (0x000000FFU)

#define CSL_MSS_RCM_LIN2_UART2_CLK_STATUS_CURRDIVIDER_MASK                     (0x0000FF00U)
#define CSL_MSS_RCM_LIN2_UART2_CLK_STATUS_CURRDIVIDER_SHIFT                    (0x00000008U)
#define CSL_MSS_RCM_LIN2_UART2_CLK_STATUS_CURRDIVIDER_RESETVAL                 (0x00000000U)
#define CSL_MSS_RCM_LIN2_UART2_CLK_STATUS_CURRDIVIDER_MAX                      (0x000000FFU)

#define CSL_MSS_RCM_LIN2_UART2_CLK_STATUS_RESETVAL                             (0x00000001U)

/* LIN3_UART3_CLK_STATUS */

#define CSL_MSS_RCM_LIN3_UART3_CLK_STATUS_CLKINUSE_MASK                        (0x000000FFU)
#define CSL_MSS_RCM_LIN3_UART3_CLK_STATUS_CLKINUSE_SHIFT                       (0x00000000U)
#define CSL_MSS_RCM_LIN3_UART3_CLK_STATUS_CLKINUSE_RESETVAL                    (0x00000001U)
#define CSL_MSS_RCM_LIN3_UART3_CLK_STATUS_CLKINUSE_MAX                         (0x000000FFU)

#define CSL_MSS_RCM_LIN3_UART3_CLK_STATUS_CURRDIVIDER_MASK                     (0x0000FF00U)
#define CSL_MSS_RCM_LIN3_UART3_CLK_STATUS_CURRDIVIDER_SHIFT                    (0x00000008U)
#define CSL_MSS_RCM_LIN3_UART3_CLK_STATUS_CURRDIVIDER_RESETVAL                 (0x00000000U)
#define CSL_MSS_RCM_LIN3_UART3_CLK_STATUS_CURRDIVIDER_MAX                      (0x000000FFU)

#define CSL_MSS_RCM_LIN3_UART3_CLK_STATUS_RESETVAL                             (0x00000001U)

/* LIN4_UART4_CLK_STATUS */

#define CSL_MSS_RCM_LIN4_UART4_CLK_STATUS_CLKINUSE_MASK                        (0x000000FFU)
#define CSL_MSS_RCM_LIN4_UART4_CLK_STATUS_CLKINUSE_SHIFT                       (0x00000000U)
#define CSL_MSS_RCM_LIN4_UART4_CLK_STATUS_CLKINUSE_RESETVAL                    (0x00000001U)
#define CSL_MSS_RCM_LIN4_UART4_CLK_STATUS_CLKINUSE_MAX                         (0x000000FFU)

#define CSL_MSS_RCM_LIN4_UART4_CLK_STATUS_CURRDIVIDER_MASK                     (0x0000FF00U)
#define CSL_MSS_RCM_LIN4_UART4_CLK_STATUS_CURRDIVIDER_SHIFT                    (0x00000008U)
#define CSL_MSS_RCM_LIN4_UART4_CLK_STATUS_CURRDIVIDER_RESETVAL                 (0x00000000U)
#define CSL_MSS_RCM_LIN4_UART4_CLK_STATUS_CURRDIVIDER_MAX                      (0x000000FFU)

#define CSL_MSS_RCM_LIN4_UART4_CLK_STATUS_RESETVAL                             (0x00000001U)

/* LIN5_UART5_CLK_STATUS */

#define CSL_MSS_RCM_LIN5_UART5_CLK_STATUS_CLKINUSE_MASK                        (0x000000FFU)
#define CSL_MSS_RCM_LIN5_UART5_CLK_STATUS_CLKINUSE_SHIFT                       (0x00000000U)
#define CSL_MSS_RCM_LIN5_UART5_CLK_STATUS_CLKINUSE_RESETVAL                    (0x00000001U)
#define CSL_MSS_RCM_LIN5_UART5_CLK_STATUS_CLKINUSE_MAX                         (0x000000FFU)

#define CSL_MSS_RCM_LIN5_UART5_CLK_STATUS_CURRDIVIDER_MASK                     (0x0000FF00U)
#define CSL_MSS_RCM_LIN5_UART5_CLK_STATUS_CURRDIVIDER_SHIFT                    (0x00000008U)
#define CSL_MSS_RCM_LIN5_UART5_CLK_STATUS_CURRDIVIDER_RESETVAL                 (0x00000000U)
#define CSL_MSS_RCM_LIN5_UART5_CLK_STATUS_CURRDIVIDER_MAX                      (0x000000FFU)

#define CSL_MSS_RCM_LIN5_UART5_CLK_STATUS_RESETVAL                             (0x00000001U)

/* RGMII_250_CLK_STATUS */

#define CSL_MSS_RCM_RGMII_250_CLK_STATUS_CURRDIVIDER_MASK                      (0x0000FF00U)
#define CSL_MSS_RCM_RGMII_250_CLK_STATUS_CURRDIVIDER_SHIFT                     (0x00000008U)
#define CSL_MSS_RCM_RGMII_250_CLK_STATUS_CURRDIVIDER_RESETVAL                  (0x00000001U)
#define CSL_MSS_RCM_RGMII_250_CLK_STATUS_CURRDIVIDER_MAX                       (0x000000FFU)

#define CSL_MSS_RCM_RGMII_250_CLK_STATUS_RESETVAL                              (0x00000100U)

/* RGMII_50_CLK_STATUS */

#define CSL_MSS_RCM_RGMII_50_CLK_STATUS_CURRDIVIDER_MASK                       (0x0000FF00U)
#define CSL_MSS_RCM_RGMII_50_CLK_STATUS_CURRDIVIDER_SHIFT                      (0x00000008U)
#define CSL_MSS_RCM_RGMII_50_CLK_STATUS_CURRDIVIDER_RESETVAL                   (0x00000009U)
#define CSL_MSS_RCM_RGMII_50_CLK_STATUS_CURRDIVIDER_MAX                        (0x000000FFU)

#define CSL_MSS_RCM_RGMII_50_CLK_STATUS_RESETVAL                               (0x00000900U)

/* RGMII_5_CLK_STATUS */

#define CSL_MSS_RCM_RGMII_5_CLK_STATUS_CURRDIVIDER_MASK                        (0x0000FF00U)
#define CSL_MSS_RCM_RGMII_5_CLK_STATUS_CURRDIVIDER_SHIFT                       (0x00000008U)
#define CSL_MSS_RCM_RGMII_5_CLK_STATUS_CURRDIVIDER_RESETVAL                    (0x00000063U)
#define CSL_MSS_RCM_RGMII_5_CLK_STATUS_CURRDIVIDER_MAX                         (0x000000FFU)

#define CSL_MSS_RCM_RGMII_5_CLK_STATUS_RESETVAL                                (0x00006300U)

/* MMC0_32K_CLK_STATUS */

#define CSL_MSS_RCM_MMC0_32K_CLK_STATUS_CURRDIVIDER_MASK                       (0x0003FF00U)
#define CSL_MSS_RCM_MMC0_32K_CLK_STATUS_CURRDIVIDER_SHIFT                      (0x00000008U)
#define CSL_MSS_RCM_MMC0_32K_CLK_STATUS_CURRDIVIDER_RESETVAL                   (0x0000030CU)
#define CSL_MSS_RCM_MMC0_32K_CLK_STATUS_CURRDIVIDER_MAX                        (0x000003FFU)

#define CSL_MSS_RCM_MMC0_32K_CLK_STATUS_RESETVAL                               (0x00030C00U)

/* TEMPSENSE_32K_CLK_STATUS */

#define CSL_MSS_RCM_TEMPSENSE_32K_CLK_STATUS_CURRDIVIDER_MASK                  (0x0003FF00U)
#define CSL_MSS_RCM_TEMPSENSE_32K_CLK_STATUS_CURRDIVIDER_SHIFT                 (0x00000008U)
#define CSL_MSS_RCM_TEMPSENSE_32K_CLK_STATUS_CURRDIVIDER_RESETVAL              (0x0000030CU)
#define CSL_MSS_RCM_TEMPSENSE_32K_CLK_STATUS_CURRDIVIDER_MAX                   (0x000003FFU)

#define CSL_MSS_RCM_TEMPSENSE_32K_CLK_STATUS_RESETVAL                          (0x00030C00U)

/* MCAN4_CLK_STATUS */

#define CSL_MSS_RCM_MCAN4_CLK_STATUS_CLKINUSE_MASK                             (0x000000FFU)
#define CSL_MSS_RCM_MCAN4_CLK_STATUS_CLKINUSE_SHIFT                            (0x00000000U)
#define CSL_MSS_RCM_MCAN4_CLK_STATUS_CLKINUSE_RESETVAL                         (0x00000001U)
#define CSL_MSS_RCM_MCAN4_CLK_STATUS_CLKINUSE_MAX                              (0x000000FFU)

#define CSL_MSS_RCM_MCAN4_CLK_STATUS_CURRDIVIDER_MASK                          (0x0000FF00U)
#define CSL_MSS_RCM_MCAN4_CLK_STATUS_CURRDIVIDER_SHIFT                         (0x00000008U)
#define CSL_MSS_RCM_MCAN4_CLK_STATUS_CURRDIVIDER_RESETVAL                      (0x00000000U)
#define CSL_MSS_RCM_MCAN4_CLK_STATUS_CURRDIVIDER_MAX                           (0x000000FFU)

#define CSL_MSS_RCM_MCAN4_CLK_STATUS_RESETVAL                                  (0x00000001U)

/* MCAN5_CLK_STATUS */

#define CSL_MSS_RCM_MCAN5_CLK_STATUS_CLKINUSE_MASK                             (0x000000FFU)
#define CSL_MSS_RCM_MCAN5_CLK_STATUS_CLKINUSE_SHIFT                            (0x00000000U)
#define CSL_MSS_RCM_MCAN5_CLK_STATUS_CLKINUSE_RESETVAL                         (0x00000001U)
#define CSL_MSS_RCM_MCAN5_CLK_STATUS_CLKINUSE_MAX                              (0x000000FFU)

#define CSL_MSS_RCM_MCAN5_CLK_STATUS_CURRDIVIDER_MASK                          (0x0000FF00U)
#define CSL_MSS_RCM_MCAN5_CLK_STATUS_CURRDIVIDER_SHIFT                         (0x00000008U)
#define CSL_MSS_RCM_MCAN5_CLK_STATUS_CURRDIVIDER_RESETVAL                      (0x00000000U)
#define CSL_MSS_RCM_MCAN5_CLK_STATUS_CURRDIVIDER_MAX                           (0x000000FFU)

#define CSL_MSS_RCM_MCAN5_CLK_STATUS_RESETVAL                                  (0x00000001U)

/* MCAN6_CLK_STATUS */

#define CSL_MSS_RCM_MCAN6_CLK_STATUS_CLKINUSE_MASK                             (0x000000FFU)
#define CSL_MSS_RCM_MCAN6_CLK_STATUS_CLKINUSE_SHIFT                            (0x00000000U)
#define CSL_MSS_RCM_MCAN6_CLK_STATUS_CLKINUSE_RESETVAL                         (0x00000001U)
#define CSL_MSS_RCM_MCAN6_CLK_STATUS_CLKINUSE_MAX                              (0x000000FFU)

#define CSL_MSS_RCM_MCAN6_CLK_STATUS_CURRDIVIDER_MASK                          (0x0000FF00U)
#define CSL_MSS_RCM_MCAN6_CLK_STATUS_CURRDIVIDER_SHIFT                         (0x00000008U)
#define CSL_MSS_RCM_MCAN6_CLK_STATUS_CURRDIVIDER_RESETVAL                      (0x00000000U)
#define CSL_MSS_RCM_MCAN6_CLK_STATUS_CURRDIVIDER_MAX                           (0x000000FFU)

#define CSL_MSS_RCM_MCAN6_CLK_STATUS_RESETVAL                                  (0x00000001U)

/* MCAN7_CLK_STATUS */

#define CSL_MSS_RCM_MCAN7_CLK_STATUS_CLKINUSE_MASK                             (0x000000FFU)
#define CSL_MSS_RCM_MCAN7_CLK_STATUS_CLKINUSE_SHIFT                            (0x00000000U)
#define CSL_MSS_RCM_MCAN7_CLK_STATUS_CLKINUSE_RESETVAL                         (0x00000001U)
#define CSL_MSS_RCM_MCAN7_CLK_STATUS_CLKINUSE_MAX                              (0x000000FFU)

#define CSL_MSS_RCM_MCAN7_CLK_STATUS_CURRDIVIDER_MASK                          (0x0000FF00U)
#define CSL_MSS_RCM_MCAN7_CLK_STATUS_CURRDIVIDER_SHIFT                         (0x00000008U)
#define CSL_MSS_RCM_MCAN7_CLK_STATUS_CURRDIVIDER_RESETVAL                      (0x00000000U)
#define CSL_MSS_RCM_MCAN7_CLK_STATUS_CURRDIVIDER_MAX                           (0x000000FFU)

#define CSL_MSS_RCM_MCAN7_CLK_STATUS_RESETVAL                                  (0x00000001U)

/* RTI4_CLK_STATUS */

#define CSL_MSS_RCM_RTI4_CLK_STATUS_CLKINUSE_MASK                              (0x000000FFU)
#define CSL_MSS_RCM_RTI4_CLK_STATUS_CLKINUSE_SHIFT                             (0x00000000U)
#define CSL_MSS_RCM_RTI4_CLK_STATUS_CLKINUSE_RESETVAL                          (0x00000001U)
#define CSL_MSS_RCM_RTI4_CLK_STATUS_CLKINUSE_MAX                               (0x000000FFU)

#define CSL_MSS_RCM_RTI4_CLK_STATUS_CURRDIVIDER_MASK                           (0x0000FF00U)
#define CSL_MSS_RCM_RTI4_CLK_STATUS_CURRDIVIDER_SHIFT                          (0x00000008U)
#define CSL_MSS_RCM_RTI4_CLK_STATUS_CURRDIVIDER_RESETVAL                       (0x00000000U)
#define CSL_MSS_RCM_RTI4_CLK_STATUS_CURRDIVIDER_MAX                            (0x000000FFU)

#define CSL_MSS_RCM_RTI4_CLK_STATUS_RESETVAL                                   (0x00000001U)

/* RTI5_CLK_STATUS */

#define CSL_MSS_RCM_RTI5_CLK_STATUS_CLKINUSE_MASK                              (0x000000FFU)
#define CSL_MSS_RCM_RTI5_CLK_STATUS_CLKINUSE_SHIFT                             (0x00000000U)
#define CSL_MSS_RCM_RTI5_CLK_STATUS_CLKINUSE_RESETVAL                          (0x00000001U)
#define CSL_MSS_RCM_RTI5_CLK_STATUS_CLKINUSE_MAX                               (0x000000FFU)

#define CSL_MSS_RCM_RTI5_CLK_STATUS_CURRDIVIDER_MASK                           (0x0000FF00U)
#define CSL_MSS_RCM_RTI5_CLK_STATUS_CURRDIVIDER_SHIFT                          (0x00000008U)
#define CSL_MSS_RCM_RTI5_CLK_STATUS_CURRDIVIDER_RESETVAL                       (0x00000000U)
#define CSL_MSS_RCM_RTI5_CLK_STATUS_CURRDIVIDER_MAX                            (0x000000FFU)

#define CSL_MSS_RCM_RTI5_CLK_STATUS_RESETVAL                                   (0x00000001U)

/* RTI6_CLK_STATUS */

#define CSL_MSS_RCM_RTI6_CLK_STATUS_CLKINUSE_MASK                              (0x000000FFU)
#define CSL_MSS_RCM_RTI6_CLK_STATUS_CLKINUSE_SHIFT                             (0x00000000U)
#define CSL_MSS_RCM_RTI6_CLK_STATUS_CLKINUSE_RESETVAL                          (0x00000001U)
#define CSL_MSS_RCM_RTI6_CLK_STATUS_CLKINUSE_MAX                               (0x000000FFU)

#define CSL_MSS_RCM_RTI6_CLK_STATUS_CURRDIVIDER_MASK                           (0x0000FF00U)
#define CSL_MSS_RCM_RTI6_CLK_STATUS_CURRDIVIDER_SHIFT                          (0x00000008U)
#define CSL_MSS_RCM_RTI6_CLK_STATUS_CURRDIVIDER_RESETVAL                       (0x00000000U)
#define CSL_MSS_RCM_RTI6_CLK_STATUS_CURRDIVIDER_MAX                            (0x000000FFU)

#define CSL_MSS_RCM_RTI6_CLK_STATUS_RESETVAL                                   (0x00000001U)

/* RTI7_CLK_STATUS */

#define CSL_MSS_RCM_RTI7_CLK_STATUS_CLKINUSE_MASK                              (0x000000FFU)
#define CSL_MSS_RCM_RTI7_CLK_STATUS_CLKINUSE_SHIFT                             (0x00000000U)
#define CSL_MSS_RCM_RTI7_CLK_STATUS_CLKINUSE_RESETVAL                          (0x00000001U)
#define CSL_MSS_RCM_RTI7_CLK_STATUS_CLKINUSE_MAX                               (0x000000FFU)

#define CSL_MSS_RCM_RTI7_CLK_STATUS_CURRDIVIDER_MASK                           (0x0000FF00U)
#define CSL_MSS_RCM_RTI7_CLK_STATUS_CURRDIVIDER_SHIFT                          (0x00000008U)
#define CSL_MSS_RCM_RTI7_CLK_STATUS_CURRDIVIDER_RESETVAL                       (0x00000000U)
#define CSL_MSS_RCM_RTI7_CLK_STATUS_CURRDIVIDER_MAX                            (0x000000FFU)

#define CSL_MSS_RCM_RTI7_CLK_STATUS_RESETVAL                                   (0x00000001U)

/* MCSPI5_CLK_STATUS */

#define CSL_MSS_RCM_MCSPI5_CLK_STATUS_CLKINUSE_MASK                            (0x000000FFU)
#define CSL_MSS_RCM_MCSPI5_CLK_STATUS_CLKINUSE_SHIFT                           (0x00000000U)
#define CSL_MSS_RCM_MCSPI5_CLK_STATUS_CLKINUSE_RESETVAL                        (0x00000001U)
#define CSL_MSS_RCM_MCSPI5_CLK_STATUS_CLKINUSE_MAX                             (0x000000FFU)

#define CSL_MSS_RCM_MCSPI5_CLK_STATUS_CURRDIVIDER_MASK                         (0x0000FF00U)
#define CSL_MSS_RCM_MCSPI5_CLK_STATUS_CURRDIVIDER_SHIFT                        (0x00000008U)
#define CSL_MSS_RCM_MCSPI5_CLK_STATUS_CURRDIVIDER_RESETVAL                     (0x00000000U)
#define CSL_MSS_RCM_MCSPI5_CLK_STATUS_CURRDIVIDER_MAX                          (0x000000FFU)

#define CSL_MSS_RCM_MCSPI5_CLK_STATUS_RESETVAL                                 (0x00000001U)

/* MCSPI6_CLK_STATUS */

#define CSL_MSS_RCM_MCSPI6_CLK_STATUS_CLKINUSE_MASK                            (0x000000FFU)
#define CSL_MSS_RCM_MCSPI6_CLK_STATUS_CLKINUSE_SHIFT                           (0x00000000U)
#define CSL_MSS_RCM_MCSPI6_CLK_STATUS_CLKINUSE_RESETVAL                        (0x00000001U)
#define CSL_MSS_RCM_MCSPI6_CLK_STATUS_CLKINUSE_MAX                             (0x000000FFU)

#define CSL_MSS_RCM_MCSPI6_CLK_STATUS_CURRDIVIDER_MASK                         (0x0000FF00U)
#define CSL_MSS_RCM_MCSPI6_CLK_STATUS_CURRDIVIDER_SHIFT                        (0x00000008U)
#define CSL_MSS_RCM_MCSPI6_CLK_STATUS_CURRDIVIDER_RESETVAL                     (0x00000000U)
#define CSL_MSS_RCM_MCSPI6_CLK_STATUS_CURRDIVIDER_MAX                          (0x000000FFU)

#define CSL_MSS_RCM_MCSPI6_CLK_STATUS_RESETVAL                                 (0x00000001U)

/* MCSPI7_CLK_STATUS */

#define CSL_MSS_RCM_MCSPI7_CLK_STATUS_CLKINUSE_MASK                            (0x000000FFU)
#define CSL_MSS_RCM_MCSPI7_CLK_STATUS_CLKINUSE_SHIFT                           (0x00000000U)
#define CSL_MSS_RCM_MCSPI7_CLK_STATUS_CLKINUSE_RESETVAL                        (0x00000001U)
#define CSL_MSS_RCM_MCSPI7_CLK_STATUS_CLKINUSE_MAX                             (0x000000FFU)

#define CSL_MSS_RCM_MCSPI7_CLK_STATUS_CURRDIVIDER_MASK                         (0x0000FF00U)
#define CSL_MSS_RCM_MCSPI7_CLK_STATUS_CURRDIVIDER_SHIFT                        (0x00000008U)
#define CSL_MSS_RCM_MCSPI7_CLK_STATUS_CURRDIVIDER_RESETVAL                     (0x00000000U)
#define CSL_MSS_RCM_MCSPI7_CLK_STATUS_CURRDIVIDER_MAX                          (0x000000FFU)

#define CSL_MSS_RCM_MCSPI7_CLK_STATUS_RESETVAL                                 (0x00000001U)

/* R5SS0_POR_RST_CTRL */

#define CSL_MSS_RCM_R5SS0_POR_RST_CTRL_ASSERT_MASK                             (0x00000007U)
#define CSL_MSS_RCM_R5SS0_POR_RST_CTRL_ASSERT_SHIFT                            (0x00000000U)
#define CSL_MSS_RCM_R5SS0_POR_RST_CTRL_ASSERT_RESETVAL                         (0x00000000U)
#define CSL_MSS_RCM_R5SS0_POR_RST_CTRL_ASSERT_MAX                              (0x00000007U)

#define CSL_MSS_RCM_R5SS0_POR_RST_CTRL_RESETVAL                                (0x00000000U)

/* R5SS1_POR_RST_CTRL */

#define CSL_MSS_RCM_R5SS1_POR_RST_CTRL_ASSERT_MASK                             (0x00000007U)
#define CSL_MSS_RCM_R5SS1_POR_RST_CTRL_ASSERT_SHIFT                            (0x00000000U)
#define CSL_MSS_RCM_R5SS1_POR_RST_CTRL_ASSERT_RESETVAL                         (0x00000000U)
#define CSL_MSS_RCM_R5SS1_POR_RST_CTRL_ASSERT_MAX                              (0x00000007U)

#define CSL_MSS_RCM_R5SS1_POR_RST_CTRL_RESETVAL                                (0x00000000U)

/* R5SS0_CORE0_GRST_CTRL */

#define CSL_MSS_RCM_R5SS0_CORE0_GRST_CTRL_ASSERT_MASK                          (0x00000007U)
#define CSL_MSS_RCM_R5SS0_CORE0_GRST_CTRL_ASSERT_SHIFT                         (0x00000000U)
#define CSL_MSS_RCM_R5SS0_CORE0_GRST_CTRL_ASSERT_RESETVAL                      (0x00000000U)
#define CSL_MSS_RCM_R5SS0_CORE0_GRST_CTRL_ASSERT_MAX                           (0x00000007U)

#define CSL_MSS_RCM_R5SS0_CORE0_GRST_CTRL_RESETVAL                             (0x00000000U)

/* R5SS1_CORE0_GRST_CTRL */

#define CSL_MSS_RCM_R5SS1_CORE0_GRST_CTRL_ASSERT_MASK                          (0x00000007U)
#define CSL_MSS_RCM_R5SS1_CORE0_GRST_CTRL_ASSERT_SHIFT                         (0x00000000U)
#define CSL_MSS_RCM_R5SS1_CORE0_GRST_CTRL_ASSERT_RESETVAL                      (0x00000000U)
#define CSL_MSS_RCM_R5SS1_CORE0_GRST_CTRL_ASSERT_MAX                           (0x00000007U)

#define CSL_MSS_RCM_R5SS1_CORE0_GRST_CTRL_RESETVAL                             (0x00000000U)

/* R5SS0_CORE1_GRST_CTRL */

#define CSL_MSS_RCM_R5SS0_CORE1_GRST_CTRL_ASSERT_MASK                          (0x00000007U)
#define CSL_MSS_RCM_R5SS0_CORE1_GRST_CTRL_ASSERT_SHIFT                         (0x00000000U)
#define CSL_MSS_RCM_R5SS0_CORE1_GRST_CTRL_ASSERT_RESETVAL                      (0x00000000U)
#define CSL_MSS_RCM_R5SS0_CORE1_GRST_CTRL_ASSERT_MAX                           (0x00000007U)

#define CSL_MSS_RCM_R5SS0_CORE1_GRST_CTRL_RESETVAL                             (0x00000000U)

/* R5SS1_CORE1_GRST_CTRL */

#define CSL_MSS_RCM_R5SS1_CORE1_GRST_CTRL_ASSERT_MASK                          (0x00000007U)
#define CSL_MSS_RCM_R5SS1_CORE1_GRST_CTRL_ASSERT_SHIFT                         (0x00000000U)
#define CSL_MSS_RCM_R5SS1_CORE1_GRST_CTRL_ASSERT_RESETVAL                      (0x00000000U)
#define CSL_MSS_RCM_R5SS1_CORE1_GRST_CTRL_ASSERT_MAX                           (0x00000007U)

#define CSL_MSS_RCM_R5SS1_CORE1_GRST_CTRL_RESETVAL                             (0x00000000U)

/* R5SS0_CORE0_LRST_CTRL */

#define CSL_MSS_RCM_R5SS0_CORE0_LRST_CTRL_ASSERT_MASK                          (0x00000007U)
#define CSL_MSS_RCM_R5SS0_CORE0_LRST_CTRL_ASSERT_SHIFT                         (0x00000000U)
#define CSL_MSS_RCM_R5SS0_CORE0_LRST_CTRL_ASSERT_RESETVAL                      (0x00000000U)
#define CSL_MSS_RCM_R5SS0_CORE0_LRST_CTRL_ASSERT_MAX                           (0x00000007U)

#define CSL_MSS_RCM_R5SS0_CORE0_LRST_CTRL_RESETVAL                             (0x00000000U)

/* R5SS1_CORE0_LRST_CTRL */

#define CSL_MSS_RCM_R5SS1_CORE0_LRST_CTRL_ASSERT_MASK                          (0x00000007U)
#define CSL_MSS_RCM_R5SS1_CORE0_LRST_CTRL_ASSERT_SHIFT                         (0x00000000U)
#define CSL_MSS_RCM_R5SS1_CORE0_LRST_CTRL_ASSERT_RESETVAL                      (0x00000000U)
#define CSL_MSS_RCM_R5SS1_CORE0_LRST_CTRL_ASSERT_MAX                           (0x00000007U)

#define CSL_MSS_RCM_R5SS1_CORE0_LRST_CTRL_RESETVAL                             (0x00000000U)

/* R5SS0_CORE1_LRST_CTRL */

#define CSL_MSS_RCM_R5SS0_CORE1_LRST_CTRL_ASSERT_MASK                          (0x00000007U)
#define CSL_MSS_RCM_R5SS0_CORE1_LRST_CTRL_ASSERT_SHIFT                         (0x00000000U)
#define CSL_MSS_RCM_R5SS0_CORE1_LRST_CTRL_ASSERT_RESETVAL                      (0x00000000U)
#define CSL_MSS_RCM_R5SS0_CORE1_LRST_CTRL_ASSERT_MAX                           (0x00000007U)

#define CSL_MSS_RCM_R5SS0_CORE1_LRST_CTRL_RESETVAL                             (0x00000000U)

/* R5SS1_CORE1_LRST_CTRL */

#define CSL_MSS_RCM_R5SS1_CORE1_LRST_CTRL_ASSERT_MASK                          (0x00000007U)
#define CSL_MSS_RCM_R5SS1_CORE1_LRST_CTRL_ASSERT_SHIFT                         (0x00000000U)
#define CSL_MSS_RCM_R5SS1_CORE1_LRST_CTRL_ASSERT_RESETVAL                      (0x00000000U)
#define CSL_MSS_RCM_R5SS1_CORE1_LRST_CTRL_ASSERT_MAX                           (0x00000007U)

#define CSL_MSS_RCM_R5SS1_CORE1_LRST_CTRL_RESETVAL                             (0x00000000U)

/* R5SS0_VIM0_RST_CTRL */

#define CSL_MSS_RCM_R5SS0_VIM0_RST_CTRL_ASSERT_MASK                            (0x00000007U)
#define CSL_MSS_RCM_R5SS0_VIM0_RST_CTRL_ASSERT_SHIFT                           (0x00000000U)
#define CSL_MSS_RCM_R5SS0_VIM0_RST_CTRL_ASSERT_RESETVAL                        (0x00000000U)
#define CSL_MSS_RCM_R5SS0_VIM0_RST_CTRL_ASSERT_MAX                             (0x00000007U)

#define CSL_MSS_RCM_R5SS0_VIM0_RST_CTRL_RESETVAL                               (0x00000000U)

/* R5SS1_VIM0_RST_CTRL */

#define CSL_MSS_RCM_R5SS1_VIM0_RST_CTRL_ASSERT_MASK                            (0x00000007U)
#define CSL_MSS_RCM_R5SS1_VIM0_RST_CTRL_ASSERT_SHIFT                           (0x00000000U)
#define CSL_MSS_RCM_R5SS1_VIM0_RST_CTRL_ASSERT_RESETVAL                        (0x00000000U)
#define CSL_MSS_RCM_R5SS1_VIM0_RST_CTRL_ASSERT_MAX                             (0x00000007U)

#define CSL_MSS_RCM_R5SS1_VIM0_RST_CTRL_RESETVAL                               (0x00000000U)

/* R5SS0_VIM1_RST_CTRL */

#define CSL_MSS_RCM_R5SS0_VIM1_RST_CTRL_ASSERT_MASK                            (0x00000007U)
#define CSL_MSS_RCM_R5SS0_VIM1_RST_CTRL_ASSERT_SHIFT                           (0x00000000U)
#define CSL_MSS_RCM_R5SS0_VIM1_RST_CTRL_ASSERT_RESETVAL                        (0x00000000U)
#define CSL_MSS_RCM_R5SS0_VIM1_RST_CTRL_ASSERT_MAX                             (0x00000007U)

#define CSL_MSS_RCM_R5SS0_VIM1_RST_CTRL_RESETVAL                               (0x00000000U)

/* R5SS1_VIM1_RST_CTRL */

#define CSL_MSS_RCM_R5SS1_VIM1_RST_CTRL_ASSERT_MASK                            (0x00000007U)
#define CSL_MSS_RCM_R5SS1_VIM1_RST_CTRL_ASSERT_SHIFT                           (0x00000000U)
#define CSL_MSS_RCM_R5SS1_VIM1_RST_CTRL_ASSERT_RESETVAL                        (0x00000000U)
#define CSL_MSS_RCM_R5SS1_VIM1_RST_CTRL_ASSERT_MAX                             (0x00000007U)

#define CSL_MSS_RCM_R5SS1_VIM1_RST_CTRL_RESETVAL                               (0x00000000U)

/* MCRC0_RST_CTRL */

#define CSL_MSS_RCM_MCRC0_RST_CTRL_ASSERT_MASK                                 (0x00000007U)
#define CSL_MSS_RCM_MCRC0_RST_CTRL_ASSERT_SHIFT                                (0x00000000U)
#define CSL_MSS_RCM_MCRC0_RST_CTRL_ASSERT_RESETVAL                             (0x00000000U)
#define CSL_MSS_RCM_MCRC0_RST_CTRL_ASSERT_MAX                                  (0x00000007U)

#define CSL_MSS_RCM_MCRC0_RST_CTRL_RESETVAL                                    (0x00000000U)

/* RTI0_RST_CTRL */

#define CSL_MSS_RCM_RTI0_RST_CTRL_ASSERT_MASK                                  (0x00000007U)
#define CSL_MSS_RCM_RTI0_RST_CTRL_ASSERT_SHIFT                                 (0x00000000U)
#define CSL_MSS_RCM_RTI0_RST_CTRL_ASSERT_RESETVAL                              (0x00000000U)
#define CSL_MSS_RCM_RTI0_RST_CTRL_ASSERT_MAX                                   (0x00000007U)

#define CSL_MSS_RCM_RTI0_RST_CTRL_RESETVAL                                     (0x00000000U)

/* RTI1_RST_CTRL */

#define CSL_MSS_RCM_RTI1_RST_CTRL_ASSERT_MASK                                  (0x00000007U)
#define CSL_MSS_RCM_RTI1_RST_CTRL_ASSERT_SHIFT                                 (0x00000000U)
#define CSL_MSS_RCM_RTI1_RST_CTRL_ASSERT_RESETVAL                              (0x00000000U)
#define CSL_MSS_RCM_RTI1_RST_CTRL_ASSERT_MAX                                   (0x00000007U)

#define CSL_MSS_RCM_RTI1_RST_CTRL_RESETVAL                                     (0x00000000U)

/* RTI2_RST_CTRL */

#define CSL_MSS_RCM_RTI2_RST_CTRL_ASSERT_MASK                                  (0x00000007U)
#define CSL_MSS_RCM_RTI2_RST_CTRL_ASSERT_SHIFT                                 (0x00000000U)
#define CSL_MSS_RCM_RTI2_RST_CTRL_ASSERT_RESETVAL                              (0x00000000U)
#define CSL_MSS_RCM_RTI2_RST_CTRL_ASSERT_MAX                                   (0x00000007U)

#define CSL_MSS_RCM_RTI2_RST_CTRL_RESETVAL                                     (0x00000000U)

/* RTI3_RST_CTRL */

#define CSL_MSS_RCM_RTI3_RST_CTRL_ASSERT_MASK                                  (0x00000007U)
#define CSL_MSS_RCM_RTI3_RST_CTRL_ASSERT_SHIFT                                 (0x00000000U)
#define CSL_MSS_RCM_RTI3_RST_CTRL_ASSERT_RESETVAL                              (0x00000000U)
#define CSL_MSS_RCM_RTI3_RST_CTRL_ASSERT_MAX                                   (0x00000007U)

#define CSL_MSS_RCM_RTI3_RST_CTRL_RESETVAL                                     (0x00000000U)

/* WDT0_RST_CTRL */

#define CSL_MSS_RCM_WDT0_RST_CTRL_ASSERT_MASK                                  (0x00000007U)
#define CSL_MSS_RCM_WDT0_RST_CTRL_ASSERT_SHIFT                                 (0x00000000U)
#define CSL_MSS_RCM_WDT0_RST_CTRL_ASSERT_RESETVAL                              (0x00000000U)
#define CSL_MSS_RCM_WDT0_RST_CTRL_ASSERT_MAX                                   (0x00000007U)

#define CSL_MSS_RCM_WDT0_RST_CTRL_RESETVAL                                     (0x00000000U)

/* WDT1_RST_CTRL */

#define CSL_MSS_RCM_WDT1_RST_CTRL_ASSERT_MASK                                  (0x00000007U)
#define CSL_MSS_RCM_WDT1_RST_CTRL_ASSERT_SHIFT                                 (0x00000000U)
#define CSL_MSS_RCM_WDT1_RST_CTRL_ASSERT_RESETVAL                              (0x00000000U)
#define CSL_MSS_RCM_WDT1_RST_CTRL_ASSERT_MAX                                   (0x00000007U)

#define CSL_MSS_RCM_WDT1_RST_CTRL_RESETVAL                                     (0x00000000U)

/* WDT2_RST_CTRL */

#define CSL_MSS_RCM_WDT2_RST_CTRL_ASSERT_MASK                                  (0x00000007U)
#define CSL_MSS_RCM_WDT2_RST_CTRL_ASSERT_SHIFT                                 (0x00000000U)
#define CSL_MSS_RCM_WDT2_RST_CTRL_ASSERT_RESETVAL                              (0x00000000U)
#define CSL_MSS_RCM_WDT2_RST_CTRL_ASSERT_MAX                                   (0x00000007U)

#define CSL_MSS_RCM_WDT2_RST_CTRL_RESETVAL                                     (0x00000000U)

/* WDT3_RST_CTRL */

#define CSL_MSS_RCM_WDT3_RST_CTRL_ASSERT_MASK                                  (0x00000007U)
#define CSL_MSS_RCM_WDT3_RST_CTRL_ASSERT_SHIFT                                 (0x00000000U)
#define CSL_MSS_RCM_WDT3_RST_CTRL_ASSERT_RESETVAL                              (0x00000000U)
#define CSL_MSS_RCM_WDT3_RST_CTRL_ASSERT_MAX                                   (0x00000007U)

#define CSL_MSS_RCM_WDT3_RST_CTRL_RESETVAL                                     (0x00000000U)

/* TOP_ESM_RST_CTRL */

#define CSL_MSS_RCM_TOP_ESM_RST_CTRL_ASSERT_MASK                               (0x00000007U)
#define CSL_MSS_RCM_TOP_ESM_RST_CTRL_ASSERT_SHIFT                              (0x00000000U)
#define CSL_MSS_RCM_TOP_ESM_RST_CTRL_ASSERT_RESETVAL                           (0x00000000U)
#define CSL_MSS_RCM_TOP_ESM_RST_CTRL_ASSERT_MAX                                (0x00000007U)

#define CSL_MSS_RCM_TOP_ESM_RST_CTRL_RESETVAL                                  (0x00000000U)

/* DCC0_RST_CTRL */

#define CSL_MSS_RCM_DCC0_RST_CTRL_ASSERT_MASK                                  (0x00000007U)
#define CSL_MSS_RCM_DCC0_RST_CTRL_ASSERT_SHIFT                                 (0x00000000U)
#define CSL_MSS_RCM_DCC0_RST_CTRL_ASSERT_RESETVAL                              (0x00000000U)
#define CSL_MSS_RCM_DCC0_RST_CTRL_ASSERT_MAX                                   (0x00000007U)

#define CSL_MSS_RCM_DCC0_RST_CTRL_RESETVAL                                     (0x00000000U)

/* DCC1_RST_CTRL */

#define CSL_MSS_RCM_DCC1_RST_CTRL_ASSERT_MASK                                  (0x00000007U)
#define CSL_MSS_RCM_DCC1_RST_CTRL_ASSERT_SHIFT                                 (0x00000000U)
#define CSL_MSS_RCM_DCC1_RST_CTRL_ASSERT_RESETVAL                              (0x00000000U)
#define CSL_MSS_RCM_DCC1_RST_CTRL_ASSERT_MAX                                   (0x00000007U)

#define CSL_MSS_RCM_DCC1_RST_CTRL_RESETVAL                                     (0x00000000U)

/* DCC2_RST_CTRL */

#define CSL_MSS_RCM_DCC2_RST_CTRL_ASSERT_MASK                                  (0x00000007U)
#define CSL_MSS_RCM_DCC2_RST_CTRL_ASSERT_SHIFT                                 (0x00000000U)
#define CSL_MSS_RCM_DCC2_RST_CTRL_ASSERT_RESETVAL                              (0x00000000U)
#define CSL_MSS_RCM_DCC2_RST_CTRL_ASSERT_MAX                                   (0x00000007U)

#define CSL_MSS_RCM_DCC2_RST_CTRL_RESETVAL                                     (0x00000000U)

/* DCC3_RST_CTRL */

#define CSL_MSS_RCM_DCC3_RST_CTRL_ASSERT_MASK                                  (0x00000007U)
#define CSL_MSS_RCM_DCC3_RST_CTRL_ASSERT_SHIFT                                 (0x00000000U)
#define CSL_MSS_RCM_DCC3_RST_CTRL_ASSERT_RESETVAL                              (0x00000000U)
#define CSL_MSS_RCM_DCC3_RST_CTRL_ASSERT_MAX                                   (0x00000007U)

#define CSL_MSS_RCM_DCC3_RST_CTRL_RESETVAL                                     (0x00000000U)

/* MCSPI0_RST_CTRL */

#define CSL_MSS_RCM_MCSPI0_RST_CTRL_ASSERT_MASK                                (0x00000007U)
#define CSL_MSS_RCM_MCSPI0_RST_CTRL_ASSERT_SHIFT                               (0x00000000U)
#define CSL_MSS_RCM_MCSPI0_RST_CTRL_ASSERT_RESETVAL                            (0x00000000U)
#define CSL_MSS_RCM_MCSPI0_RST_CTRL_ASSERT_MAX                                 (0x00000007U)

#define CSL_MSS_RCM_MCSPI0_RST_CTRL_RESETVAL                                   (0x00000000U)

/* MCSPI1_RST_CTRL */

#define CSL_MSS_RCM_MCSPI1_RST_CTRL_ASSERT_MASK                                (0x00000007U)
#define CSL_MSS_RCM_MCSPI1_RST_CTRL_ASSERT_SHIFT                               (0x00000000U)
#define CSL_MSS_RCM_MCSPI1_RST_CTRL_ASSERT_RESETVAL                            (0x00000000U)
#define CSL_MSS_RCM_MCSPI1_RST_CTRL_ASSERT_MAX                                 (0x00000007U)

#define CSL_MSS_RCM_MCSPI1_RST_CTRL_RESETVAL                                   (0x00000000U)

/* MCSPI2_RST_CTRL */

#define CSL_MSS_RCM_MCSPI2_RST_CTRL_ASSERT_MASK                                (0x00000007U)
#define CSL_MSS_RCM_MCSPI2_RST_CTRL_ASSERT_SHIFT                               (0x00000000U)
#define CSL_MSS_RCM_MCSPI2_RST_CTRL_ASSERT_RESETVAL                            (0x00000000U)
#define CSL_MSS_RCM_MCSPI2_RST_CTRL_ASSERT_MAX                                 (0x00000007U)

#define CSL_MSS_RCM_MCSPI2_RST_CTRL_RESETVAL                                   (0x00000000U)

/* MCSPI3_RST_CTRL */

#define CSL_MSS_RCM_MCSPI3_RST_CTRL_ASSERT_MASK                                (0x00000007U)
#define CSL_MSS_RCM_MCSPI3_RST_CTRL_ASSERT_SHIFT                               (0x00000000U)
#define CSL_MSS_RCM_MCSPI3_RST_CTRL_ASSERT_RESETVAL                            (0x00000000U)
#define CSL_MSS_RCM_MCSPI3_RST_CTRL_ASSERT_MAX                                 (0x00000007U)

#define CSL_MSS_RCM_MCSPI3_RST_CTRL_RESETVAL                                   (0x00000000U)

/* MCSPI4_RST_CTRL */

#define CSL_MSS_RCM_MCSPI4_RST_CTRL_ASSERT_MASK                                (0x00000007U)
#define CSL_MSS_RCM_MCSPI4_RST_CTRL_ASSERT_SHIFT                               (0x00000000U)
#define CSL_MSS_RCM_MCSPI4_RST_CTRL_ASSERT_RESETVAL                            (0x00000000U)
#define CSL_MSS_RCM_MCSPI4_RST_CTRL_ASSERT_MAX                                 (0x00000007U)

#define CSL_MSS_RCM_MCSPI4_RST_CTRL_RESETVAL                                   (0x00000000U)

/* OSPI_RST_CTRL */

#define CSL_MSS_RCM_OSPI_RST_CTRL_ASSERT_MASK                                  (0x00000007U)
#define CSL_MSS_RCM_OSPI_RST_CTRL_ASSERT_SHIFT                                 (0x00000000U)
#define CSL_MSS_RCM_OSPI_RST_CTRL_ASSERT_RESETVAL                              (0x00000000U)
#define CSL_MSS_RCM_OSPI_RST_CTRL_ASSERT_MAX                                   (0x00000007U)

#define CSL_MSS_RCM_OSPI_RST_CTRL_RESETVAL                                     (0x00000000U)

/* MCAN0_RST_CTRL */

#define CSL_MSS_RCM_MCAN0_RST_CTRL_ASSERT_MASK                                 (0x00000007U)
#define CSL_MSS_RCM_MCAN0_RST_CTRL_ASSERT_SHIFT                                (0x00000000U)
#define CSL_MSS_RCM_MCAN0_RST_CTRL_ASSERT_RESETVAL                             (0x00000000U)
#define CSL_MSS_RCM_MCAN0_RST_CTRL_ASSERT_MAX                                  (0x00000007U)

#define CSL_MSS_RCM_MCAN0_RST_CTRL_RESETVAL                                    (0x00000000U)

/* MCAN1_RST_CTRL */

#define CSL_MSS_RCM_MCAN1_RST_CTRL_ASSERT_MASK                                 (0x00000007U)
#define CSL_MSS_RCM_MCAN1_RST_CTRL_ASSERT_SHIFT                                (0x00000000U)
#define CSL_MSS_RCM_MCAN1_RST_CTRL_ASSERT_RESETVAL                             (0x00000000U)
#define CSL_MSS_RCM_MCAN1_RST_CTRL_ASSERT_MAX                                  (0x00000007U)

#define CSL_MSS_RCM_MCAN1_RST_CTRL_RESETVAL                                    (0x00000000U)

/* MCAN2_RST_CTRL */

#define CSL_MSS_RCM_MCAN2_RST_CTRL_ASSERT_MASK                                 (0x00000007U)
#define CSL_MSS_RCM_MCAN2_RST_CTRL_ASSERT_SHIFT                                (0x00000000U)
#define CSL_MSS_RCM_MCAN2_RST_CTRL_ASSERT_RESETVAL                             (0x00000000U)
#define CSL_MSS_RCM_MCAN2_RST_CTRL_ASSERT_MAX                                  (0x00000007U)

#define CSL_MSS_RCM_MCAN2_RST_CTRL_RESETVAL                                    (0x00000000U)

/* MCAN3_RST_CTRL */

#define CSL_MSS_RCM_MCAN3_RST_CTRL_ASSERT_MASK                                 (0x00000007U)
#define CSL_MSS_RCM_MCAN3_RST_CTRL_ASSERT_SHIFT                                (0x00000000U)
#define CSL_MSS_RCM_MCAN3_RST_CTRL_ASSERT_RESETVAL                             (0x00000000U)
#define CSL_MSS_RCM_MCAN3_RST_CTRL_ASSERT_MAX                                  (0x00000007U)

#define CSL_MSS_RCM_MCAN3_RST_CTRL_RESETVAL                                    (0x00000000U)

/* I2C0_RST_CTRL */

#define CSL_MSS_RCM_I2C0_RST_CTRL_ASSERT_MASK                                  (0x00000007U)
#define CSL_MSS_RCM_I2C0_RST_CTRL_ASSERT_SHIFT                                 (0x00000000U)
#define CSL_MSS_RCM_I2C0_RST_CTRL_ASSERT_RESETVAL                              (0x00000000U)
#define CSL_MSS_RCM_I2C0_RST_CTRL_ASSERT_MAX                                   (0x00000007U)

#define CSL_MSS_RCM_I2C0_RST_CTRL_RESETVAL                                     (0x00000000U)

/* I2C1_RST_CTRL */

#define CSL_MSS_RCM_I2C1_RST_CTRL_ASSERT_MASK                                  (0x00000007U)
#define CSL_MSS_RCM_I2C1_RST_CTRL_ASSERT_SHIFT                                 (0x00000000U)
#define CSL_MSS_RCM_I2C1_RST_CTRL_ASSERT_RESETVAL                              (0x00000000U)
#define CSL_MSS_RCM_I2C1_RST_CTRL_ASSERT_MAX                                   (0x00000007U)

#define CSL_MSS_RCM_I2C1_RST_CTRL_RESETVAL                                     (0x00000000U)

/* I2C2_RST_CTRL */

#define CSL_MSS_RCM_I2C2_RST_CTRL_ASSERT_MASK                                  (0x00000007U)
#define CSL_MSS_RCM_I2C2_RST_CTRL_ASSERT_SHIFT                                 (0x00000000U)
#define CSL_MSS_RCM_I2C2_RST_CTRL_ASSERT_RESETVAL                              (0x00000000U)
#define CSL_MSS_RCM_I2C2_RST_CTRL_ASSERT_MAX                                   (0x00000007U)

#define CSL_MSS_RCM_I2C2_RST_CTRL_RESETVAL                                     (0x00000000U)

/* I2C3_RST_CTRL */

#define CSL_MSS_RCM_I2C3_RST_CTRL_ASSERT_MASK                                  (0x00000007U)
#define CSL_MSS_RCM_I2C3_RST_CTRL_ASSERT_SHIFT                                 (0x00000000U)
#define CSL_MSS_RCM_I2C3_RST_CTRL_ASSERT_RESETVAL                              (0x00000000U)
#define CSL_MSS_RCM_I2C3_RST_CTRL_ASSERT_MAX                                   (0x00000007U)

#define CSL_MSS_RCM_I2C3_RST_CTRL_RESETVAL                                     (0x00000000U)

/* UART0_RST_CTRL */

#define CSL_MSS_RCM_UART0_RST_CTRL_ASSERT_MASK                                 (0x00000007U)
#define CSL_MSS_RCM_UART0_RST_CTRL_ASSERT_SHIFT                                (0x00000000U)
#define CSL_MSS_RCM_UART0_RST_CTRL_ASSERT_RESETVAL                             (0x00000000U)
#define CSL_MSS_RCM_UART0_RST_CTRL_ASSERT_MAX                                  (0x00000007U)

#define CSL_MSS_RCM_UART0_RST_CTRL_RESETVAL                                    (0x00000000U)

/* UART1_RST_CTRL */

#define CSL_MSS_RCM_UART1_RST_CTRL_ASSERT_MASK                                 (0x00000007U)
#define CSL_MSS_RCM_UART1_RST_CTRL_ASSERT_SHIFT                                (0x00000000U)
#define CSL_MSS_RCM_UART1_RST_CTRL_ASSERT_RESETVAL                             (0x00000000U)
#define CSL_MSS_RCM_UART1_RST_CTRL_ASSERT_MAX                                  (0x00000007U)

#define CSL_MSS_RCM_UART1_RST_CTRL_RESETVAL                                    (0x00000000U)

/* UART2_RST_CTRL */

#define CSL_MSS_RCM_UART2_RST_CTRL_ASSERT_MASK                                 (0x00000007U)
#define CSL_MSS_RCM_UART2_RST_CTRL_ASSERT_SHIFT                                (0x00000000U)
#define CSL_MSS_RCM_UART2_RST_CTRL_ASSERT_RESETVAL                             (0x00000000U)
#define CSL_MSS_RCM_UART2_RST_CTRL_ASSERT_MAX                                  (0x00000007U)

#define CSL_MSS_RCM_UART2_RST_CTRL_RESETVAL                                    (0x00000000U)

/* UART3_RST_CTRL */

#define CSL_MSS_RCM_UART3_RST_CTRL_ASSERT_MASK                                 (0x00000007U)
#define CSL_MSS_RCM_UART3_RST_CTRL_ASSERT_SHIFT                                (0x00000000U)
#define CSL_MSS_RCM_UART3_RST_CTRL_ASSERT_RESETVAL                             (0x00000000U)
#define CSL_MSS_RCM_UART3_RST_CTRL_ASSERT_MAX                                  (0x00000007U)

#define CSL_MSS_RCM_UART3_RST_CTRL_RESETVAL                                    (0x00000000U)

/* UART4_RST_CTRL */

#define CSL_MSS_RCM_UART4_RST_CTRL_ASSERT_MASK                                 (0x00000007U)
#define CSL_MSS_RCM_UART4_RST_CTRL_ASSERT_SHIFT                                (0x00000000U)
#define CSL_MSS_RCM_UART4_RST_CTRL_ASSERT_RESETVAL                             (0x00000000U)
#define CSL_MSS_RCM_UART4_RST_CTRL_ASSERT_MAX                                  (0x00000007U)

#define CSL_MSS_RCM_UART4_RST_CTRL_RESETVAL                                    (0x00000000U)

/* UART5_RST_CTRL */

#define CSL_MSS_RCM_UART5_RST_CTRL_ASSERT_MASK                                 (0x00000007U)
#define CSL_MSS_RCM_UART5_RST_CTRL_ASSERT_SHIFT                                (0x00000000U)
#define CSL_MSS_RCM_UART5_RST_CTRL_ASSERT_RESETVAL                             (0x00000000U)
#define CSL_MSS_RCM_UART5_RST_CTRL_ASSERT_MAX                                  (0x00000007U)

#define CSL_MSS_RCM_UART5_RST_CTRL_RESETVAL                                    (0x00000000U)

/* LIN0_RST_CTRL */

#define CSL_MSS_RCM_LIN0_RST_CTRL_ASSERT_MASK                                  (0x00000007U)
#define CSL_MSS_RCM_LIN0_RST_CTRL_ASSERT_SHIFT                                 (0x00000000U)
#define CSL_MSS_RCM_LIN0_RST_CTRL_ASSERT_RESETVAL                              (0x00000000U)
#define CSL_MSS_RCM_LIN0_RST_CTRL_ASSERT_MAX                                   (0x00000007U)

#define CSL_MSS_RCM_LIN0_RST_CTRL_RESETVAL                                     (0x00000000U)

/* LIN1_RST_CTRL */

#define CSL_MSS_RCM_LIN1_RST_CTRL_ASSERT_MASK                                  (0x00000007U)
#define CSL_MSS_RCM_LIN1_RST_CTRL_ASSERT_SHIFT                                 (0x00000000U)
#define CSL_MSS_RCM_LIN1_RST_CTRL_ASSERT_RESETVAL                              (0x00000000U)
#define CSL_MSS_RCM_LIN1_RST_CTRL_ASSERT_MAX                                   (0x00000007U)

#define CSL_MSS_RCM_LIN1_RST_CTRL_RESETVAL                                     (0x00000000U)

/* LIN2_RST_CTRL */

#define CSL_MSS_RCM_LIN2_RST_CTRL_ASSERT_MASK                                  (0x00000007U)
#define CSL_MSS_RCM_LIN2_RST_CTRL_ASSERT_SHIFT                                 (0x00000000U)
#define CSL_MSS_RCM_LIN2_RST_CTRL_ASSERT_RESETVAL                              (0x00000000U)
#define CSL_MSS_RCM_LIN2_RST_CTRL_ASSERT_MAX                                   (0x00000007U)

#define CSL_MSS_RCM_LIN2_RST_CTRL_RESETVAL                                     (0x00000000U)

/* LIN3_RST_CTRL */

#define CSL_MSS_RCM_LIN3_RST_CTRL_ASSERT_MASK                                  (0x00000007U)
#define CSL_MSS_RCM_LIN3_RST_CTRL_ASSERT_SHIFT                                 (0x00000000U)
#define CSL_MSS_RCM_LIN3_RST_CTRL_ASSERT_RESETVAL                              (0x00000000U)
#define CSL_MSS_RCM_LIN3_RST_CTRL_ASSERT_MAX                                   (0x00000007U)

#define CSL_MSS_RCM_LIN3_RST_CTRL_RESETVAL                                     (0x00000000U)

/* LIN4_RST_CTRL */

#define CSL_MSS_RCM_LIN4_RST_CTRL_ASSERT_MASK                                  (0x00000007U)
#define CSL_MSS_RCM_LIN4_RST_CTRL_ASSERT_SHIFT                                 (0x00000000U)
#define CSL_MSS_RCM_LIN4_RST_CTRL_ASSERT_RESETVAL                              (0x00000000U)
#define CSL_MSS_RCM_LIN4_RST_CTRL_ASSERT_MAX                                   (0x00000007U)

#define CSL_MSS_RCM_LIN4_RST_CTRL_RESETVAL                                     (0x00000000U)

/* EDMA_RST_CTRL */

#define CSL_MSS_RCM_EDMA_RST_CTRL_ASSERT_MASK                                  (0x00000007U)
#define CSL_MSS_RCM_EDMA_RST_CTRL_ASSERT_SHIFT                                 (0x00000000U)
#define CSL_MSS_RCM_EDMA_RST_CTRL_ASSERT_RESETVAL                              (0x00000000U)
#define CSL_MSS_RCM_EDMA_RST_CTRL_ASSERT_MAX                                   (0x00000007U)

#define CSL_MSS_RCM_EDMA_RST_CTRL_TPCCA_ASSERT_MASK                            (0x00000070U)
#define CSL_MSS_RCM_EDMA_RST_CTRL_TPCCA_ASSERT_SHIFT                           (0x00000004U)
#define CSL_MSS_RCM_EDMA_RST_CTRL_TPCCA_ASSERT_RESETVAL                        (0x00000000U)
#define CSL_MSS_RCM_EDMA_RST_CTRL_TPCCA_ASSERT_MAX                             (0x00000007U)

#define CSL_MSS_RCM_EDMA_RST_CTRL_TPTCA0_ASSERT_MASK                           (0x00000700U)
#define CSL_MSS_RCM_EDMA_RST_CTRL_TPTCA0_ASSERT_SHIFT                          (0x00000008U)
#define CSL_MSS_RCM_EDMA_RST_CTRL_TPTCA0_ASSERT_RESETVAL                       (0x00000000U)
#define CSL_MSS_RCM_EDMA_RST_CTRL_TPTCA0_ASSERT_MAX                            (0x00000007U)

#define CSL_MSS_RCM_EDMA_RST_CTRL_TPTCA1_ASSERT_MASK                           (0x00007000U)
#define CSL_MSS_RCM_EDMA_RST_CTRL_TPTCA1_ASSERT_SHIFT                          (0x0000000CU)
#define CSL_MSS_RCM_EDMA_RST_CTRL_TPTCA1_ASSERT_RESETVAL                       (0x00000000U)
#define CSL_MSS_RCM_EDMA_RST_CTRL_TPTCA1_ASSERT_MAX                            (0x00000007U)

#define CSL_MSS_RCM_EDMA_RST_CTRL_RESETVAL                                     (0x00000000U)

/* INFRA_RST_CTRL */

#define CSL_MSS_RCM_INFRA_RST_CTRL_ASSERT_MASK                                 (0x00000007U)
#define CSL_MSS_RCM_INFRA_RST_CTRL_ASSERT_SHIFT                                (0x00000000U)
#define CSL_MSS_RCM_INFRA_RST_CTRL_ASSERT_RESETVAL                             (0x00000000U)
#define CSL_MSS_RCM_INFRA_RST_CTRL_ASSERT_MAX                                  (0x00000007U)

#define CSL_MSS_RCM_INFRA_RST_CTRL_RESETVAL                                    (0x00000000U)

/* CPSW_RST_CTRL */

#define CSL_MSS_RCM_CPSW_RST_CTRL_ASSERT_MASK                                  (0x00000007U)
#define CSL_MSS_RCM_CPSW_RST_CTRL_ASSERT_SHIFT                                 (0x00000000U)
#define CSL_MSS_RCM_CPSW_RST_CTRL_ASSERT_RESETVAL                              (0x00000000U)
#define CSL_MSS_RCM_CPSW_RST_CTRL_ASSERT_MAX                                   (0x00000007U)

#define CSL_MSS_RCM_CPSW_RST_CTRL_RESETVAL                                     (0x00000000U)

/* ICSSM0_RST_CTRL */

#define CSL_MSS_RCM_ICSSM0_RST_CTRL_ASSERT_MASK                                (0x00000007U)
#define CSL_MSS_RCM_ICSSM0_RST_CTRL_ASSERT_SHIFT                               (0x00000000U)
#define CSL_MSS_RCM_ICSSM0_RST_CTRL_ASSERT_RESETVAL                            (0x00000000U)
#define CSL_MSS_RCM_ICSSM0_RST_CTRL_ASSERT_MAX                                 (0x00000007U)

#define CSL_MSS_RCM_ICSSM0_RST_CTRL_RESETVAL                                   (0x00000000U)

/* MMC0_RST_CTRL */

#define CSL_MSS_RCM_MMC0_RST_CTRL_ASSERT_MASK                                  (0x00000007U)
#define CSL_MSS_RCM_MMC0_RST_CTRL_ASSERT_SHIFT                                 (0x00000000U)
#define CSL_MSS_RCM_MMC0_RST_CTRL_ASSERT_RESETVAL                              (0x00000000U)
#define CSL_MSS_RCM_MMC0_RST_CTRL_ASSERT_MAX                                   (0x00000007U)

#define CSL_MSS_RCM_MMC0_RST_CTRL_RESETVAL                                     (0x00000000U)

/* GPIO0_RST_CTRL */

#define CSL_MSS_RCM_GPIO0_RST_CTRL_ASSERT_MASK                                 (0x00000007U)
#define CSL_MSS_RCM_GPIO0_RST_CTRL_ASSERT_SHIFT                                (0x00000000U)
#define CSL_MSS_RCM_GPIO0_RST_CTRL_ASSERT_RESETVAL                             (0x00000000U)
#define CSL_MSS_RCM_GPIO0_RST_CTRL_ASSERT_MAX                                  (0x00000007U)

#define CSL_MSS_RCM_GPIO0_RST_CTRL_RESETVAL                                    (0x00000000U)

/* GPIO1_RST_CTRL */

#define CSL_MSS_RCM_GPIO1_RST_CTRL_ASSERT_MASK                                 (0x00000007U)
#define CSL_MSS_RCM_GPIO1_RST_CTRL_ASSERT_SHIFT                                (0x00000000U)
#define CSL_MSS_RCM_GPIO1_RST_CTRL_ASSERT_RESETVAL                             (0x00000000U)
#define CSL_MSS_RCM_GPIO1_RST_CTRL_ASSERT_MAX                                  (0x00000007U)

#define CSL_MSS_RCM_GPIO1_RST_CTRL_RESETVAL                                    (0x00000000U)

/* GPIO2_RST_CTRL */

#define CSL_MSS_RCM_GPIO2_RST_CTRL_ASSERT_MASK                                 (0x00000007U)
#define CSL_MSS_RCM_GPIO2_RST_CTRL_ASSERT_SHIFT                                (0x00000000U)
#define CSL_MSS_RCM_GPIO2_RST_CTRL_ASSERT_RESETVAL                             (0x00000000U)
#define CSL_MSS_RCM_GPIO2_RST_CTRL_ASSERT_MAX                                  (0x00000007U)

#define CSL_MSS_RCM_GPIO2_RST_CTRL_RESETVAL                                    (0x00000000U)

/* GPIO3_RST_CTRL */

#define CSL_MSS_RCM_GPIO3_RST_CTRL_ASSERT_MASK                                 (0x00000007U)
#define CSL_MSS_RCM_GPIO3_RST_CTRL_ASSERT_SHIFT                                (0x00000000U)
#define CSL_MSS_RCM_GPIO3_RST_CTRL_ASSERT_RESETVAL                             (0x00000000U)
#define CSL_MSS_RCM_GPIO3_RST_CTRL_ASSERT_MAX                                  (0x00000007U)

#define CSL_MSS_RCM_GPIO3_RST_CTRL_RESETVAL                                    (0x00000000U)

/* SPINLOCK0_RST_CTRL */

#define CSL_MSS_RCM_SPINLOCK0_RST_CTRL_ASSERT_MASK                             (0x00000007U)
#define CSL_MSS_RCM_SPINLOCK0_RST_CTRL_ASSERT_SHIFT                            (0x00000000U)
#define CSL_MSS_RCM_SPINLOCK0_RST_CTRL_ASSERT_RESETVAL                         (0x00000000U)
#define CSL_MSS_RCM_SPINLOCK0_RST_CTRL_ASSERT_MAX                              (0x00000007U)

#define CSL_MSS_RCM_SPINLOCK0_RST_CTRL_RESETVAL                                (0x00000000U)

/* TEMPSENSE_32K_RST_CTRL */

#define CSL_MSS_RCM_TEMPSENSE_32K_RST_CTRL_ASSERT_MASK                         (0x00000007U)
#define CSL_MSS_RCM_TEMPSENSE_32K_RST_CTRL_ASSERT_SHIFT                        (0x00000000U)
#define CSL_MSS_RCM_TEMPSENSE_32K_RST_CTRL_ASSERT_RESETVAL                     (0x00000000U)
#define CSL_MSS_RCM_TEMPSENSE_32K_RST_CTRL_ASSERT_MAX                          (0x00000007U)

#define CSL_MSS_RCM_TEMPSENSE_32K_RST_CTRL_RESETVAL                            (0x00000000U)

/* MCAN4_RST_CTRL */

#define CSL_MSS_RCM_MCAN4_RST_CTRL_ASSERT_MASK                                 (0x00000007U)
#define CSL_MSS_RCM_MCAN4_RST_CTRL_ASSERT_SHIFT                                (0x00000000U)
#define CSL_MSS_RCM_MCAN4_RST_CTRL_ASSERT_RESETVAL                             (0x00000000U)
#define CSL_MSS_RCM_MCAN4_RST_CTRL_ASSERT_MAX                                  (0x00000007U)

#define CSL_MSS_RCM_MCAN4_RST_CTRL_RESETVAL                                    (0x00000000U)

/* MCAN5_RST_CTRL */

#define CSL_MSS_RCM_MCAN5_RST_CTRL_ASSERT_MASK                                 (0x00000007U)
#define CSL_MSS_RCM_MCAN5_RST_CTRL_ASSERT_SHIFT                                (0x00000000U)
#define CSL_MSS_RCM_MCAN5_RST_CTRL_ASSERT_RESETVAL                             (0x00000000U)
#define CSL_MSS_RCM_MCAN5_RST_CTRL_ASSERT_MAX                                  (0x00000007U)

#define CSL_MSS_RCM_MCAN5_RST_CTRL_RESETVAL                                    (0x00000000U)

/* MCAN6_RST_CTRL */

#define CSL_MSS_RCM_MCAN6_RST_CTRL_ASSERT_MASK                                 (0x00000007U)
#define CSL_MSS_RCM_MCAN6_RST_CTRL_ASSERT_SHIFT                                (0x00000000U)
#define CSL_MSS_RCM_MCAN6_RST_CTRL_ASSERT_RESETVAL                             (0x00000000U)
#define CSL_MSS_RCM_MCAN6_RST_CTRL_ASSERT_MAX                                  (0x00000007U)

#define CSL_MSS_RCM_MCAN6_RST_CTRL_RESETVAL                                    (0x00000000U)

/* MCAN7_RST_CTRL */

#define CSL_MSS_RCM_MCAN7_RST_CTRL_ASSERT_MASK                                 (0x00000007U)
#define CSL_MSS_RCM_MCAN7_RST_CTRL_ASSERT_SHIFT                                (0x00000000U)
#define CSL_MSS_RCM_MCAN7_RST_CTRL_ASSERT_RESETVAL                             (0x00000000U)
#define CSL_MSS_RCM_MCAN7_RST_CTRL_ASSERT_MAX                                  (0x00000007U)

#define CSL_MSS_RCM_MCAN7_RST_CTRL_RESETVAL                                    (0x00000000U)

/* RTI4_RST_CTRL */

#define CSL_MSS_RCM_RTI4_RST_CTRL_ASSERT_MASK                                  (0x00000007U)
#define CSL_MSS_RCM_RTI4_RST_CTRL_ASSERT_SHIFT                                 (0x00000000U)
#define CSL_MSS_RCM_RTI4_RST_CTRL_ASSERT_RESETVAL                              (0x00000000U)
#define CSL_MSS_RCM_RTI4_RST_CTRL_ASSERT_MAX                                   (0x00000007U)

#define CSL_MSS_RCM_RTI4_RST_CTRL_RESETVAL                                     (0x00000000U)

/* RTI5_RST_CTRL */

#define CSL_MSS_RCM_RTI5_RST_CTRL_ASSERT_MASK                                  (0x00000007U)
#define CSL_MSS_RCM_RTI5_RST_CTRL_ASSERT_SHIFT                                 (0x00000000U)
#define CSL_MSS_RCM_RTI5_RST_CTRL_ASSERT_RESETVAL                              (0x00000000U)
#define CSL_MSS_RCM_RTI5_RST_CTRL_ASSERT_MAX                                   (0x00000007U)

#define CSL_MSS_RCM_RTI5_RST_CTRL_RESETVAL                                     (0x00000000U)

/* RTI6_RST_CTRL */

#define CSL_MSS_RCM_RTI6_RST_CTRL_ASSERT_MASK                                  (0x00000007U)
#define CSL_MSS_RCM_RTI6_RST_CTRL_ASSERT_SHIFT                                 (0x00000000U)
#define CSL_MSS_RCM_RTI6_RST_CTRL_ASSERT_RESETVAL                              (0x00000000U)
#define CSL_MSS_RCM_RTI6_RST_CTRL_ASSERT_MAX                                   (0x00000007U)

#define CSL_MSS_RCM_RTI6_RST_CTRL_RESETVAL                                     (0x00000000U)

/* RTI7_RST_CTRL */

#define CSL_MSS_RCM_RTI7_RST_CTRL_ASSERT_MASK                                  (0x00000007U)
#define CSL_MSS_RCM_RTI7_RST_CTRL_ASSERT_SHIFT                                 (0x00000000U)
#define CSL_MSS_RCM_RTI7_RST_CTRL_ASSERT_RESETVAL                              (0x00000000U)
#define CSL_MSS_RCM_RTI7_RST_CTRL_ASSERT_MAX                                   (0x00000007U)

#define CSL_MSS_RCM_RTI7_RST_CTRL_RESETVAL                                     (0x00000000U)

/* MCSPI5_RST_CTRL */

#define CSL_MSS_RCM_MCSPI5_RST_CTRL_ASSERT_MASK                                (0x00000007U)
#define CSL_MSS_RCM_MCSPI5_RST_CTRL_ASSERT_SHIFT                               (0x00000000U)
#define CSL_MSS_RCM_MCSPI5_RST_CTRL_ASSERT_RESETVAL                            (0x00000000U)
#define CSL_MSS_RCM_MCSPI5_RST_CTRL_ASSERT_MAX                                 (0x00000007U)

#define CSL_MSS_RCM_MCSPI5_RST_CTRL_RESETVAL                                   (0x00000000U)

/* MCSPI6_RST_CTRL */

#define CSL_MSS_RCM_MCSPI6_RST_CTRL_ASSERT_MASK                                (0x00000007U)
#define CSL_MSS_RCM_MCSPI6_RST_CTRL_ASSERT_SHIFT                               (0x00000000U)
#define CSL_MSS_RCM_MCSPI6_RST_CTRL_ASSERT_RESETVAL                            (0x00000000U)
#define CSL_MSS_RCM_MCSPI6_RST_CTRL_ASSERT_MAX                                 (0x00000007U)

#define CSL_MSS_RCM_MCSPI6_RST_CTRL_RESETVAL                                   (0x00000000U)

/* MCSPI7_RST_CTRL */

#define CSL_MSS_RCM_MCSPI7_RST_CTRL_ASSERT_MASK                                (0x00000007U)
#define CSL_MSS_RCM_MCSPI7_RST_CTRL_ASSERT_SHIFT                               (0x00000000U)
#define CSL_MSS_RCM_MCSPI7_RST_CTRL_ASSERT_RESETVAL                            (0x00000000U)
#define CSL_MSS_RCM_MCSPI7_RST_CTRL_ASSERT_MAX                                 (0x00000007U)

#define CSL_MSS_RCM_MCSPI7_RST_CTRL_RESETVAL                                   (0x00000000U)

/* L2OCRAM_BANK0_PD_CTRL */

#define CSL_MSS_RCM_L2OCRAM_BANK0_PD_CTRL_ISO_MASK                             (0x00000007U)
#define CSL_MSS_RCM_L2OCRAM_BANK0_PD_CTRL_ISO_SHIFT                            (0x00000000U)
#define CSL_MSS_RCM_L2OCRAM_BANK0_PD_CTRL_ISO_RESETVAL                         (0x00000000U)
#define CSL_MSS_RCM_L2OCRAM_BANK0_PD_CTRL_ISO_MAX                              (0x00000007U)

#define CSL_MSS_RCM_L2OCRAM_BANK0_PD_CTRL_AONIN_MASK                           (0x00000070U)
#define CSL_MSS_RCM_L2OCRAM_BANK0_PD_CTRL_AONIN_SHIFT                          (0x00000004U)
#define CSL_MSS_RCM_L2OCRAM_BANK0_PD_CTRL_AONIN_RESETVAL                       (0x00000007U)
#define CSL_MSS_RCM_L2OCRAM_BANK0_PD_CTRL_AONIN_MAX                            (0x00000007U)

#define CSL_MSS_RCM_L2OCRAM_BANK0_PD_CTRL_AGOODIN_MASK                         (0x00000700U)
#define CSL_MSS_RCM_L2OCRAM_BANK0_PD_CTRL_AGOODIN_SHIFT                        (0x00000008U)
#define CSL_MSS_RCM_L2OCRAM_BANK0_PD_CTRL_AGOODIN_RESETVAL                     (0x00000007U)
#define CSL_MSS_RCM_L2OCRAM_BANK0_PD_CTRL_AGOODIN_MAX                          (0x00000007U)

#define CSL_MSS_RCM_L2OCRAM_BANK0_PD_CTRL_RESETVAL                             (0x00000770U)

/* L2OCRAM_BANK1_PD_CTRL */

#define CSL_MSS_RCM_L2OCRAM_BANK1_PD_CTRL_ISO_MASK                             (0x00000007U)
#define CSL_MSS_RCM_L2OCRAM_BANK1_PD_CTRL_ISO_SHIFT                            (0x00000000U)
#define CSL_MSS_RCM_L2OCRAM_BANK1_PD_CTRL_ISO_RESETVAL                         (0x00000000U)
#define CSL_MSS_RCM_L2OCRAM_BANK1_PD_CTRL_ISO_MAX                              (0x00000007U)

#define CSL_MSS_RCM_L2OCRAM_BANK1_PD_CTRL_AONIN_MASK                           (0x00000070U)
#define CSL_MSS_RCM_L2OCRAM_BANK1_PD_CTRL_AONIN_SHIFT                          (0x00000004U)
#define CSL_MSS_RCM_L2OCRAM_BANK1_PD_CTRL_AONIN_RESETVAL                       (0x00000007U)
#define CSL_MSS_RCM_L2OCRAM_BANK1_PD_CTRL_AONIN_MAX                            (0x00000007U)

#define CSL_MSS_RCM_L2OCRAM_BANK1_PD_CTRL_AGOODIN_MASK                         (0x00000700U)
#define CSL_MSS_RCM_L2OCRAM_BANK1_PD_CTRL_AGOODIN_SHIFT                        (0x00000008U)
#define CSL_MSS_RCM_L2OCRAM_BANK1_PD_CTRL_AGOODIN_RESETVAL                     (0x00000007U)
#define CSL_MSS_RCM_L2OCRAM_BANK1_PD_CTRL_AGOODIN_MAX                          (0x00000007U)

#define CSL_MSS_RCM_L2OCRAM_BANK1_PD_CTRL_RESETVAL                             (0x00000770U)

/* L2OCRAM_BANK2_PD_CTRL */

#define CSL_MSS_RCM_L2OCRAM_BANK2_PD_CTRL_ISO_MASK                             (0x00000007U)
#define CSL_MSS_RCM_L2OCRAM_BANK2_PD_CTRL_ISO_SHIFT                            (0x00000000U)
#define CSL_MSS_RCM_L2OCRAM_BANK2_PD_CTRL_ISO_RESETVAL                         (0x00000000U)
#define CSL_MSS_RCM_L2OCRAM_BANK2_PD_CTRL_ISO_MAX                              (0x00000007U)

#define CSL_MSS_RCM_L2OCRAM_BANK2_PD_CTRL_AONIN_MASK                           (0x00000070U)
#define CSL_MSS_RCM_L2OCRAM_BANK2_PD_CTRL_AONIN_SHIFT                          (0x00000004U)
#define CSL_MSS_RCM_L2OCRAM_BANK2_PD_CTRL_AONIN_RESETVAL                       (0x00000007U)
#define CSL_MSS_RCM_L2OCRAM_BANK2_PD_CTRL_AONIN_MAX                            (0x00000007U)

#define CSL_MSS_RCM_L2OCRAM_BANK2_PD_CTRL_AGOODIN_MASK                         (0x00000700U)
#define CSL_MSS_RCM_L2OCRAM_BANK2_PD_CTRL_AGOODIN_SHIFT                        (0x00000008U)
#define CSL_MSS_RCM_L2OCRAM_BANK2_PD_CTRL_AGOODIN_RESETVAL                     (0x00000007U)
#define CSL_MSS_RCM_L2OCRAM_BANK2_PD_CTRL_AGOODIN_MAX                          (0x00000007U)

#define CSL_MSS_RCM_L2OCRAM_BANK2_PD_CTRL_RESETVAL                             (0x00000770U)

/* L2OCRAM_BANK3_PD_CTRL */

#define CSL_MSS_RCM_L2OCRAM_BANK3_PD_CTRL_ISO_MASK                             (0x00000007U)
#define CSL_MSS_RCM_L2OCRAM_BANK3_PD_CTRL_ISO_SHIFT                            (0x00000000U)
#define CSL_MSS_RCM_L2OCRAM_BANK3_PD_CTRL_ISO_RESETVAL                         (0x00000000U)
#define CSL_MSS_RCM_L2OCRAM_BANK3_PD_CTRL_ISO_MAX                              (0x00000007U)

#define CSL_MSS_RCM_L2OCRAM_BANK3_PD_CTRL_AONIN_MASK                           (0x00000070U)
#define CSL_MSS_RCM_L2OCRAM_BANK3_PD_CTRL_AONIN_SHIFT                          (0x00000004U)
#define CSL_MSS_RCM_L2OCRAM_BANK3_PD_CTRL_AONIN_RESETVAL                       (0x00000007U)
#define CSL_MSS_RCM_L2OCRAM_BANK3_PD_CTRL_AONIN_MAX                            (0x00000007U)

#define CSL_MSS_RCM_L2OCRAM_BANK3_PD_CTRL_AGOODIN_MASK                         (0x00000700U)
#define CSL_MSS_RCM_L2OCRAM_BANK3_PD_CTRL_AGOODIN_SHIFT                        (0x00000008U)
#define CSL_MSS_RCM_L2OCRAM_BANK3_PD_CTRL_AGOODIN_RESETVAL                     (0x00000007U)
#define CSL_MSS_RCM_L2OCRAM_BANK3_PD_CTRL_AGOODIN_MAX                          (0x00000007U)

#define CSL_MSS_RCM_L2OCRAM_BANK3_PD_CTRL_RESETVAL                             (0x00000770U)

/* L2OCRAM_BANK0_PD_STATUS */

#define CSL_MSS_RCM_L2OCRAM_BANK0_PD_STATUS_AONOUT_MASK                        (0x00000001U)
#define CSL_MSS_RCM_L2OCRAM_BANK0_PD_STATUS_AONOUT_SHIFT                       (0x00000000U)
#define CSL_MSS_RCM_L2OCRAM_BANK0_PD_STATUS_AONOUT_RESETVAL                    (0x00000001U)
#define CSL_MSS_RCM_L2OCRAM_BANK0_PD_STATUS_AONOUT_MAX                         (0x00000001U)

#define CSL_MSS_RCM_L2OCRAM_BANK0_PD_STATUS_AGOODOUT_MASK                      (0x00000002U)
#define CSL_MSS_RCM_L2OCRAM_BANK0_PD_STATUS_AGOODOUT_SHIFT                     (0x00000001U)
#define CSL_MSS_RCM_L2OCRAM_BANK0_PD_STATUS_AGOODOUT_RESETVAL                  (0x00000001U)
#define CSL_MSS_RCM_L2OCRAM_BANK0_PD_STATUS_AGOODOUT_MAX                       (0x00000001U)

#define CSL_MSS_RCM_L2OCRAM_BANK0_PD_STATUS_RESETVAL                           (0x00000003U)

/* L2OCRAM_BANK1_PD_STATUS */

#define CSL_MSS_RCM_L2OCRAM_BANK1_PD_STATUS_AONOUT_MASK                        (0x00000001U)
#define CSL_MSS_RCM_L2OCRAM_BANK1_PD_STATUS_AONOUT_SHIFT                       (0x00000000U)
#define CSL_MSS_RCM_L2OCRAM_BANK1_PD_STATUS_AONOUT_RESETVAL                    (0x00000001U)
#define CSL_MSS_RCM_L2OCRAM_BANK1_PD_STATUS_AONOUT_MAX                         (0x00000001U)

#define CSL_MSS_RCM_L2OCRAM_BANK1_PD_STATUS_AGOODOUT_MASK                      (0x00000002U)
#define CSL_MSS_RCM_L2OCRAM_BANK1_PD_STATUS_AGOODOUT_SHIFT                     (0x00000001U)
#define CSL_MSS_RCM_L2OCRAM_BANK1_PD_STATUS_AGOODOUT_RESETVAL                  (0x00000001U)
#define CSL_MSS_RCM_L2OCRAM_BANK1_PD_STATUS_AGOODOUT_MAX                       (0x00000001U)

#define CSL_MSS_RCM_L2OCRAM_BANK1_PD_STATUS_RESETVAL                           (0x00000003U)

/* L2OCRAM_BANK2_PD_STATUS */

#define CSL_MSS_RCM_L2OCRAM_BANK2_PD_STATUS_AONOUT_MASK                        (0x00000001U)
#define CSL_MSS_RCM_L2OCRAM_BANK2_PD_STATUS_AONOUT_SHIFT                       (0x00000000U)
#define CSL_MSS_RCM_L2OCRAM_BANK2_PD_STATUS_AONOUT_RESETVAL                    (0x00000001U)
#define CSL_MSS_RCM_L2OCRAM_BANK2_PD_STATUS_AONOUT_MAX                         (0x00000001U)

#define CSL_MSS_RCM_L2OCRAM_BANK2_PD_STATUS_AGOODOUT_MASK                      (0x00000002U)
#define CSL_MSS_RCM_L2OCRAM_BANK2_PD_STATUS_AGOODOUT_SHIFT                     (0x00000001U)
#define CSL_MSS_RCM_L2OCRAM_BANK2_PD_STATUS_AGOODOUT_RESETVAL                  (0x00000001U)
#define CSL_MSS_RCM_L2OCRAM_BANK2_PD_STATUS_AGOODOUT_MAX                       (0x00000001U)

#define CSL_MSS_RCM_L2OCRAM_BANK2_PD_STATUS_RESETVAL                           (0x00000003U)

/* L2OCRAM_BANK3_PD_STATUS */

#define CSL_MSS_RCM_L2OCRAM_BANK3_PD_STATUS_AONOUT_MASK                        (0x00000001U)
#define CSL_MSS_RCM_L2OCRAM_BANK3_PD_STATUS_AONOUT_SHIFT                       (0x00000000U)
#define CSL_MSS_RCM_L2OCRAM_BANK3_PD_STATUS_AONOUT_RESETVAL                    (0x00000001U)
#define CSL_MSS_RCM_L2OCRAM_BANK3_PD_STATUS_AONOUT_MAX                         (0x00000001U)

#define CSL_MSS_RCM_L2OCRAM_BANK3_PD_STATUS_AGOODOUT_MASK                      (0x00000002U)
#define CSL_MSS_RCM_L2OCRAM_BANK3_PD_STATUS_AGOODOUT_SHIFT                     (0x00000001U)
#define CSL_MSS_RCM_L2OCRAM_BANK3_PD_STATUS_AGOODOUT_RESETVAL                  (0x00000001U)
#define CSL_MSS_RCM_L2OCRAM_BANK3_PD_STATUS_AGOODOUT_MAX                       (0x00000001U)

#define CSL_MSS_RCM_L2OCRAM_BANK3_PD_STATUS_RESETVAL                           (0x00000003U)

/* HW_REG0 */

#define CSL_MSS_RCM_HW_REG0_HWREG_MASK                                         (0xFFFFFFFFU)
#define CSL_MSS_RCM_HW_REG0_HWREG_SHIFT                                        (0x00000000U)
#define CSL_MSS_RCM_HW_REG0_HWREG_RESETVAL                                     (0x00000000U)
#define CSL_MSS_RCM_HW_REG0_HWREG_MAX                                          (0xFFFFFFFFU)

#define CSL_MSS_RCM_HW_REG0_RESETVAL                                           (0x00000000U)

/* HW_REG1 */

#define CSL_MSS_RCM_HW_REG1_HWREG_MASK                                         (0xFFFFFFFFU)
#define CSL_MSS_RCM_HW_REG1_HWREG_SHIFT                                        (0x00000000U)
#define CSL_MSS_RCM_HW_REG1_HWREG_RESETVAL                                     (0x00000000U)
#define CSL_MSS_RCM_HW_REG1_HWREG_MAX                                          (0xFFFFFFFFU)

#define CSL_MSS_RCM_HW_REG1_RESETVAL                                           (0x00000000U)

/* HW_REG2 */

#define CSL_MSS_RCM_HW_REG2_HWREG_MASK                                         (0xFFFFFFFFU)
#define CSL_MSS_RCM_HW_REG2_HWREG_SHIFT                                        (0x00000000U)
#define CSL_MSS_RCM_HW_REG2_HWREG_RESETVAL                                     (0x00000000U)
#define CSL_MSS_RCM_HW_REG2_HWREG_MAX                                          (0xFFFFFFFFU)

#define CSL_MSS_RCM_HW_REG2_RESETVAL                                           (0x00000000U)

/* HW_REG3 */

#define CSL_MSS_RCM_HW_REG3_HWREG_MASK                                         (0xFFFFFFFFU)
#define CSL_MSS_RCM_HW_REG3_HWREG_SHIFT                                        (0x00000000U)
#define CSL_MSS_RCM_HW_REG3_HWREG_RESETVAL                                     (0x00000000U)
#define CSL_MSS_RCM_HW_REG3_HWREG_MAX                                          (0xFFFFFFFFU)

#define CSL_MSS_RCM_HW_REG3_RESETVAL                                           (0x00000000U)

/* L2OCRAM_BANK4_PD_CTRL */

#define CSL_MSS_RCM_L2OCRAM_BANK4_PD_CTRL_ISO_MASK                             (0x00000007U)
#define CSL_MSS_RCM_L2OCRAM_BANK4_PD_CTRL_ISO_SHIFT                            (0x00000000U)
#define CSL_MSS_RCM_L2OCRAM_BANK4_PD_CTRL_ISO_RESETVAL                         (0x00000000U)
#define CSL_MSS_RCM_L2OCRAM_BANK4_PD_CTRL_ISO_MAX                              (0x00000007U)

#define CSL_MSS_RCM_L2OCRAM_BANK4_PD_CTRL_AONIN_MASK                           (0x00000070U)
#define CSL_MSS_RCM_L2OCRAM_BANK4_PD_CTRL_AONIN_SHIFT                          (0x00000004U)
#define CSL_MSS_RCM_L2OCRAM_BANK4_PD_CTRL_AONIN_RESETVAL                       (0x00000007U)
#define CSL_MSS_RCM_L2OCRAM_BANK4_PD_CTRL_AONIN_MAX                            (0x00000007U)

#define CSL_MSS_RCM_L2OCRAM_BANK4_PD_CTRL_AGOODIN_MASK                         (0x00000700U)
#define CSL_MSS_RCM_L2OCRAM_BANK4_PD_CTRL_AGOODIN_SHIFT                        (0x00000008U)
#define CSL_MSS_RCM_L2OCRAM_BANK4_PD_CTRL_AGOODIN_RESETVAL                     (0x00000007U)
#define CSL_MSS_RCM_L2OCRAM_BANK4_PD_CTRL_AGOODIN_MAX                          (0x00000007U)

#define CSL_MSS_RCM_L2OCRAM_BANK4_PD_CTRL_RESETVAL                             (0x00000770U)

/* L2OCRAM_BANK5_PD_CTRL */

#define CSL_MSS_RCM_L2OCRAM_BANK5_PD_CTRL_ISO_MASK                             (0x00000007U)
#define CSL_MSS_RCM_L2OCRAM_BANK5_PD_CTRL_ISO_SHIFT                            (0x00000000U)
#define CSL_MSS_RCM_L2OCRAM_BANK5_PD_CTRL_ISO_RESETVAL                         (0x00000000U)
#define CSL_MSS_RCM_L2OCRAM_BANK5_PD_CTRL_ISO_MAX                              (0x00000007U)

#define CSL_MSS_RCM_L2OCRAM_BANK5_PD_CTRL_AONIN_MASK                           (0x00000070U)
#define CSL_MSS_RCM_L2OCRAM_BANK5_PD_CTRL_AONIN_SHIFT                          (0x00000004U)
#define CSL_MSS_RCM_L2OCRAM_BANK5_PD_CTRL_AONIN_RESETVAL                       (0x00000007U)
#define CSL_MSS_RCM_L2OCRAM_BANK5_PD_CTRL_AONIN_MAX                            (0x00000007U)

#define CSL_MSS_RCM_L2OCRAM_BANK5_PD_CTRL_AGOODIN_MASK                         (0x00000700U)
#define CSL_MSS_RCM_L2OCRAM_BANK5_PD_CTRL_AGOODIN_SHIFT                        (0x00000008U)
#define CSL_MSS_RCM_L2OCRAM_BANK5_PD_CTRL_AGOODIN_RESETVAL                     (0x00000007U)
#define CSL_MSS_RCM_L2OCRAM_BANK5_PD_CTRL_AGOODIN_MAX                          (0x00000007U)

#define CSL_MSS_RCM_L2OCRAM_BANK5_PD_CTRL_RESETVAL                             (0x00000770U)

/* L2OCRAM_BANK4_PD_STATUS */

#define CSL_MSS_RCM_L2OCRAM_BANK4_PD_STATUS_AONOUT_MASK                        (0x00000001U)
#define CSL_MSS_RCM_L2OCRAM_BANK4_PD_STATUS_AONOUT_SHIFT                       (0x00000000U)
#define CSL_MSS_RCM_L2OCRAM_BANK4_PD_STATUS_AONOUT_RESETVAL                    (0x00000001U)
#define CSL_MSS_RCM_L2OCRAM_BANK4_PD_STATUS_AONOUT_MAX                         (0x00000001U)

#define CSL_MSS_RCM_L2OCRAM_BANK4_PD_STATUS_AGOODOUT_MASK                      (0x00000002U)
#define CSL_MSS_RCM_L2OCRAM_BANK4_PD_STATUS_AGOODOUT_SHIFT                     (0x00000001U)
#define CSL_MSS_RCM_L2OCRAM_BANK4_PD_STATUS_AGOODOUT_RESETVAL                  (0x00000001U)
#define CSL_MSS_RCM_L2OCRAM_BANK4_PD_STATUS_AGOODOUT_MAX                       (0x00000001U)

#define CSL_MSS_RCM_L2OCRAM_BANK4_PD_STATUS_RESETVAL                           (0x00000003U)

/* L2OCRAM_BANK5_PD_STATUS */

#define CSL_MSS_RCM_L2OCRAM_BANK5_PD_STATUS_AONOUT_MASK                        (0x00000001U)
#define CSL_MSS_RCM_L2OCRAM_BANK5_PD_STATUS_AONOUT_SHIFT                       (0x00000000U)
#define CSL_MSS_RCM_L2OCRAM_BANK5_PD_STATUS_AONOUT_RESETVAL                    (0x00000001U)
#define CSL_MSS_RCM_L2OCRAM_BANK5_PD_STATUS_AONOUT_MAX                         (0x00000001U)

#define CSL_MSS_RCM_L2OCRAM_BANK5_PD_STATUS_AGOODOUT_MASK                      (0x00000002U)
#define CSL_MSS_RCM_L2OCRAM_BANK5_PD_STATUS_AGOODOUT_SHIFT                     (0x00000001U)
#define CSL_MSS_RCM_L2OCRAM_BANK5_PD_STATUS_AGOODOUT_RESETVAL                  (0x00000001U)
#define CSL_MSS_RCM_L2OCRAM_BANK5_PD_STATUS_AGOODOUT_MAX                       (0x00000001U)

#define CSL_MSS_RCM_L2OCRAM_BANK5_PD_STATUS_RESETVAL                           (0x00000003U)

/* HSM_RTIA_CLK_SRC_SEL */

#define CSL_MSS_RCM_HSM_RTIA_CLK_SRC_SEL_HSM_RTI0_CLK_SRC_SEL_MASK             (0x00000FFFU)
#define CSL_MSS_RCM_HSM_RTIA_CLK_SRC_SEL_HSM_RTI0_CLK_SRC_SEL_SHIFT            (0x00000000U)
#define CSL_MSS_RCM_HSM_RTIA_CLK_SRC_SEL_HSM_RTI0_CLK_SRC_SEL_RESETVAL         (0x00000000U)
#define CSL_MSS_RCM_HSM_RTIA_CLK_SRC_SEL_HSM_RTI0_CLK_SRC_SEL_MAX              (0x00000FFFU)

#define CSL_MSS_RCM_HSM_RTIA_CLK_SRC_SEL_RESETVAL                              (0x00000000U)

/* HSM_WDT_CLK_SRC_SEL */

#define CSL_MSS_RCM_HSM_WDT_CLK_SRC_SEL_HSM_WDT0_CLK_SRC_SEL_MASK              (0x00000FFFU)
#define CSL_MSS_RCM_HSM_WDT_CLK_SRC_SEL_HSM_WDT0_CLK_SRC_SEL_SHIFT             (0x00000000U)
#define CSL_MSS_RCM_HSM_WDT_CLK_SRC_SEL_HSM_WDT0_CLK_SRC_SEL_RESETVAL          (0x00000555U)
#define CSL_MSS_RCM_HSM_WDT_CLK_SRC_SEL_HSM_WDT0_CLK_SRC_SEL_MAX               (0x00000FFFU)

#define CSL_MSS_RCM_HSM_WDT_CLK_SRC_SEL_RESETVAL                               (0x00000555U)

/* HSM_RTC_CLK_SRC_SEL */

#define CSL_MSS_RCM_HSM_RTC_CLK_SRC_SEL_HSM_RTC0_CLK_SRC_SEL_MASK              (0x00000FFFU)
#define CSL_MSS_RCM_HSM_RTC_CLK_SRC_SEL_HSM_RTC0_CLK_SRC_SEL_SHIFT             (0x00000000U)
#define CSL_MSS_RCM_HSM_RTC_CLK_SRC_SEL_HSM_RTC0_CLK_SRC_SEL_RESETVAL          (0x00000777U)
#define CSL_MSS_RCM_HSM_RTC_CLK_SRC_SEL_HSM_RTC0_CLK_SRC_SEL_MAX               (0x00000FFFU)

#define CSL_MSS_RCM_HSM_RTC_CLK_SRC_SEL_RESETVAL                               (0x00000777U)

/* HSM_DMTA_CLK_SRC_SEL */

#define CSL_MSS_RCM_HSM_DMTA_CLK_SRC_SEL_HSM_DTM0_CLK_SRC_SEL_MASK             (0x00000FFFU)
#define CSL_MSS_RCM_HSM_DMTA_CLK_SRC_SEL_HSM_DTM0_CLK_SRC_SEL_SHIFT            (0x00000000U)
#define CSL_MSS_RCM_HSM_DMTA_CLK_SRC_SEL_HSM_DTM0_CLK_SRC_SEL_RESETVAL         (0x00000000U)
#define CSL_MSS_RCM_HSM_DMTA_CLK_SRC_SEL_HSM_DTM0_CLK_SRC_SEL_MAX              (0x00000FFFU)

#define CSL_MSS_RCM_HSM_DMTA_CLK_SRC_SEL_RESETVAL                              (0x00000000U)

/* HSM_DMTB_CLK_SRC_SEL */

#define CSL_MSS_RCM_HSM_DMTB_CLK_SRC_SEL_HSM_DTM1_CLK_SRC_SEL_MASK             (0x00000FFFU)
#define CSL_MSS_RCM_HSM_DMTB_CLK_SRC_SEL_HSM_DTM1_CLK_SRC_SEL_SHIFT            (0x00000000U)
#define CSL_MSS_RCM_HSM_DMTB_CLK_SRC_SEL_HSM_DTM1_CLK_SRC_SEL_RESETVAL         (0x00000000U)
#define CSL_MSS_RCM_HSM_DMTB_CLK_SRC_SEL_HSM_DTM1_CLK_SRC_SEL_MAX              (0x00000FFFU)

#define CSL_MSS_RCM_HSM_DMTB_CLK_SRC_SEL_RESETVAL                              (0x00000000U)

/* HSM_RTI_CLK_DIV_VAL */

#define CSL_MSS_RCM_HSM_RTI_CLK_DIV_VAL_CLKDIVR_MASK                           (0x00000FFFU)
#define CSL_MSS_RCM_HSM_RTI_CLK_DIV_VAL_CLKDIVR_SHIFT                          (0x00000000U)
#define CSL_MSS_RCM_HSM_RTI_CLK_DIV_VAL_CLKDIVR_RESETVAL                       (0x00000000U)
#define CSL_MSS_RCM_HSM_RTI_CLK_DIV_VAL_CLKDIVR_MAX                            (0x00000FFFU)

#define CSL_MSS_RCM_HSM_RTI_CLK_DIV_VAL_RESETVAL                               (0x00000000U)

/* HSM_WDT_CLK_DIV_VAL */

#define CSL_MSS_RCM_HSM_WDT_CLK_DIV_VAL_CLKDIVR_MASK                           (0x00000FFFU)
#define CSL_MSS_RCM_HSM_WDT_CLK_DIV_VAL_CLKDIVR_SHIFT                          (0x00000000U)
#define CSL_MSS_RCM_HSM_WDT_CLK_DIV_VAL_CLKDIVR_RESETVAL                       (0x00000000U)
#define CSL_MSS_RCM_HSM_WDT_CLK_DIV_VAL_CLKDIVR_MAX                            (0x00000FFFU)

#define CSL_MSS_RCM_HSM_WDT_CLK_DIV_VAL_RESETVAL                               (0x00000000U)

/* HSM_RTC_CLK_DIV_VAL */

#define CSL_MSS_RCM_HSM_RTC_CLK_DIV_VAL_CLKDIVR_MASK                           (0x00000FFFU)
#define CSL_MSS_RCM_HSM_RTC_CLK_DIV_VAL_CLKDIVR_SHIFT                          (0x00000000U)
#define CSL_MSS_RCM_HSM_RTC_CLK_DIV_VAL_CLKDIVR_RESETVAL                       (0x00000000U)
#define CSL_MSS_RCM_HSM_RTC_CLK_DIV_VAL_CLKDIVR_MAX                            (0x00000FFFU)

#define CSL_MSS_RCM_HSM_RTC_CLK_DIV_VAL_RESETVAL                               (0x00000000U)

/* HSM_DMTA_CLK_DIV_VAL */

#define CSL_MSS_RCM_HSM_DMTA_CLK_DIV_VAL_CLKDIVR_MASK                          (0x00000FFFU)
#define CSL_MSS_RCM_HSM_DMTA_CLK_DIV_VAL_CLKDIVR_SHIFT                         (0x00000000U)
#define CSL_MSS_RCM_HSM_DMTA_CLK_DIV_VAL_CLKDIVR_RESETVAL                      (0x00000000U)
#define CSL_MSS_RCM_HSM_DMTA_CLK_DIV_VAL_CLKDIVR_MAX                           (0x00000FFFU)

#define CSL_MSS_RCM_HSM_DMTA_CLK_DIV_VAL_RESETVAL                              (0x00000000U)

/* HSM_DMTB_CLK_DIV_VAL */

#define CSL_MSS_RCM_HSM_DMTB_CLK_DIV_VAL_CLKDIVR_MASK                          (0x00000FFFU)
#define CSL_MSS_RCM_HSM_DMTB_CLK_DIV_VAL_CLKDIVR_SHIFT                         (0x00000000U)
#define CSL_MSS_RCM_HSM_DMTB_CLK_DIV_VAL_CLKDIVR_RESETVAL                      (0x00000000U)
#define CSL_MSS_RCM_HSM_DMTB_CLK_DIV_VAL_CLKDIVR_MAX                           (0x00000FFFU)

#define CSL_MSS_RCM_HSM_DMTB_CLK_DIV_VAL_RESETVAL                              (0x00000000U)

/* HSM_RTI_CLK_GATE */

#define CSL_MSS_RCM_HSM_RTI_CLK_GATE_GATED_MASK                                (0x00000007U)
#define CSL_MSS_RCM_HSM_RTI_CLK_GATE_GATED_SHIFT                               (0x00000000U)
#define CSL_MSS_RCM_HSM_RTI_CLK_GATE_GATED_RESETVAL                            (0x00000000U)
#define CSL_MSS_RCM_HSM_RTI_CLK_GATE_GATED_MAX                                 (0x00000007U)

#define CSL_MSS_RCM_HSM_RTI_CLK_GATE_RESETVAL                                  (0x00000000U)

/* HSM_WDT_CLK_GATE */

#define CSL_MSS_RCM_HSM_WDT_CLK_GATE_GATED_MASK                                (0x00000007U)
#define CSL_MSS_RCM_HSM_WDT_CLK_GATE_GATED_SHIFT                               (0x00000000U)
#define CSL_MSS_RCM_HSM_WDT_CLK_GATE_GATED_RESETVAL                            (0x00000007U)
#define CSL_MSS_RCM_HSM_WDT_CLK_GATE_GATED_MAX                                 (0x00000007U)

#define CSL_MSS_RCM_HSM_WDT_CLK_GATE_RESETVAL                                  (0x00000007U)

/* HSM_RTC_CLK_GATE */

#define CSL_MSS_RCM_HSM_RTC_CLK_GATE_GATED_MASK                                (0x00000007U)
#define CSL_MSS_RCM_HSM_RTC_CLK_GATE_GATED_SHIFT                               (0x00000000U)
#define CSL_MSS_RCM_HSM_RTC_CLK_GATE_GATED_RESETVAL                            (0x00000000U)
#define CSL_MSS_RCM_HSM_RTC_CLK_GATE_GATED_MAX                                 (0x00000007U)

#define CSL_MSS_RCM_HSM_RTC_CLK_GATE_RESETVAL                                  (0x00000000U)

/* HSM_DMTA_CLK_GATE */

#define CSL_MSS_RCM_HSM_DMTA_CLK_GATE_GATED_MASK                               (0x00000007U)
#define CSL_MSS_RCM_HSM_DMTA_CLK_GATE_GATED_SHIFT                              (0x00000000U)
#define CSL_MSS_RCM_HSM_DMTA_CLK_GATE_GATED_RESETVAL                           (0x00000000U)
#define CSL_MSS_RCM_HSM_DMTA_CLK_GATE_GATED_MAX                                (0x00000007U)

#define CSL_MSS_RCM_HSM_DMTA_CLK_GATE_RESETVAL                                 (0x00000000U)

/* HSM_DMTB_CLK_GATE */

#define CSL_MSS_RCM_HSM_DMTB_CLK_GATE_GATED_MASK                               (0x00000007U)
#define CSL_MSS_RCM_HSM_DMTB_CLK_GATE_GATED_SHIFT                              (0x00000000U)
#define CSL_MSS_RCM_HSM_DMTB_CLK_GATE_GATED_RESETVAL                           (0x00000000U)
#define CSL_MSS_RCM_HSM_DMTB_CLK_GATE_GATED_MAX                                (0x00000007U)

#define CSL_MSS_RCM_HSM_DMTB_CLK_GATE_RESETVAL                                 (0x00000000U)

/* HSM_RTI_CLK_STATUS */

#define CSL_MSS_RCM_HSM_RTI_CLK_STATUS_CLKINUSE_MASK                           (0x000000FFU)
#define CSL_MSS_RCM_HSM_RTI_CLK_STATUS_CLKINUSE_SHIFT                          (0x00000000U)
#define CSL_MSS_RCM_HSM_RTI_CLK_STATUS_CLKINUSE_RESETVAL                       (0x00000001U)
#define CSL_MSS_RCM_HSM_RTI_CLK_STATUS_CLKINUSE_MAX                            (0x000000FFU)

#define CSL_MSS_RCM_HSM_RTI_CLK_STATUS_CURRDIVIDER_MASK                        (0x0000FF00U)
#define CSL_MSS_RCM_HSM_RTI_CLK_STATUS_CURRDIVIDER_SHIFT                       (0x00000008U)
#define CSL_MSS_RCM_HSM_RTI_CLK_STATUS_CURRDIVIDER_RESETVAL                    (0x00000000U)
#define CSL_MSS_RCM_HSM_RTI_CLK_STATUS_CURRDIVIDER_MAX                         (0x000000FFU)

#define CSL_MSS_RCM_HSM_RTI_CLK_STATUS_RESETVAL                                (0x00000001U)

/* HSM_WDT_CLK_STATUS */

#define CSL_MSS_RCM_HSM_WDT_CLK_STATUS_CLKINUSE_MASK                           (0x000000FFU)
#define CSL_MSS_RCM_HSM_WDT_CLK_STATUS_CLKINUSE_SHIFT                          (0x00000000U)
#define CSL_MSS_RCM_HSM_WDT_CLK_STATUS_CLKINUSE_RESETVAL                       (0x00000020U)
#define CSL_MSS_RCM_HSM_WDT_CLK_STATUS_CLKINUSE_MAX                            (0x000000FFU)

#define CSL_MSS_RCM_HSM_WDT_CLK_STATUS_CURRDIVIDER_MASK                        (0x0000FF00U)
#define CSL_MSS_RCM_HSM_WDT_CLK_STATUS_CURRDIVIDER_SHIFT                       (0x00000008U)
#define CSL_MSS_RCM_HSM_WDT_CLK_STATUS_CURRDIVIDER_RESETVAL                    (0x00000000U)
#define CSL_MSS_RCM_HSM_WDT_CLK_STATUS_CURRDIVIDER_MAX                         (0x000000FFU)

#define CSL_MSS_RCM_HSM_WDT_CLK_STATUS_RESETVAL                                (0x00000020U)

/* HSM_RTC_CLK_STATUS */

#define CSL_MSS_RCM_HSM_RTC_CLK_STATUS_CLKINUSE_MASK                           (0x000000FFU)
#define CSL_MSS_RCM_HSM_RTC_CLK_STATUS_CLKINUSE_SHIFT                          (0x00000000U)
#define CSL_MSS_RCM_HSM_RTC_CLK_STATUS_CLKINUSE_RESETVAL                       (0x00000080U)
#define CSL_MSS_RCM_HSM_RTC_CLK_STATUS_CLKINUSE_MAX                            (0x000000FFU)

#define CSL_MSS_RCM_HSM_RTC_CLK_STATUS_CURRDIVIDER_MASK                        (0x0000FF00U)
#define CSL_MSS_RCM_HSM_RTC_CLK_STATUS_CURRDIVIDER_SHIFT                       (0x00000008U)
#define CSL_MSS_RCM_HSM_RTC_CLK_STATUS_CURRDIVIDER_RESETVAL                    (0x00000000U)
#define CSL_MSS_RCM_HSM_RTC_CLK_STATUS_CURRDIVIDER_MAX                         (0x000000FFU)

#define CSL_MSS_RCM_HSM_RTC_CLK_STATUS_RESETVAL                                (0x00000080U)

/* HSM_DMTA_CLK_STATUS */

#define CSL_MSS_RCM_HSM_DMTA_CLK_STATUS_CLKINUSE_MASK                          (0x000000FFU)
#define CSL_MSS_RCM_HSM_DMTA_CLK_STATUS_CLKINUSE_SHIFT                         (0x00000000U)
#define CSL_MSS_RCM_HSM_DMTA_CLK_STATUS_CLKINUSE_RESETVAL                      (0x00000001U)
#define CSL_MSS_RCM_HSM_DMTA_CLK_STATUS_CLKINUSE_MAX                           (0x000000FFU)

#define CSL_MSS_RCM_HSM_DMTA_CLK_STATUS_CURRDIVIDER_MASK                       (0x0000FF00U)
#define CSL_MSS_RCM_HSM_DMTA_CLK_STATUS_CURRDIVIDER_SHIFT                      (0x00000008U)
#define CSL_MSS_RCM_HSM_DMTA_CLK_STATUS_CURRDIVIDER_RESETVAL                   (0x00000000U)
#define CSL_MSS_RCM_HSM_DMTA_CLK_STATUS_CURRDIVIDER_MAX                        (0x000000FFU)

#define CSL_MSS_RCM_HSM_DMTA_CLK_STATUS_RESETVAL                               (0x00000001U)

/* HSM_DMTB_CLK_STATUS */

#define CSL_MSS_RCM_HSM_DMTB_CLK_STATUS_CLKINUSE_MASK                          (0x000000FFU)
#define CSL_MSS_RCM_HSM_DMTB_CLK_STATUS_CLKINUSE_SHIFT                         (0x00000000U)
#define CSL_MSS_RCM_HSM_DMTB_CLK_STATUS_CLKINUSE_RESETVAL                      (0x00000001U)
#define CSL_MSS_RCM_HSM_DMTB_CLK_STATUS_CLKINUSE_MAX                           (0x000000FFU)

#define CSL_MSS_RCM_HSM_DMTB_CLK_STATUS_CURRDIVIDER_MASK                       (0x0000FF00U)
#define CSL_MSS_RCM_HSM_DMTB_CLK_STATUS_CURRDIVIDER_SHIFT                      (0x00000008U)
#define CSL_MSS_RCM_HSM_DMTB_CLK_STATUS_CURRDIVIDER_RESETVAL                   (0x00000000U)
#define CSL_MSS_RCM_HSM_DMTB_CLK_STATUS_CURRDIVIDER_MAX                        (0x000000FFU)

#define CSL_MSS_RCM_HSM_DMTB_CLK_STATUS_RESETVAL                               (0x00000001U)

/* MCSPI5_CLK_GATE */

#define CSL_MSS_RCM_MCSPI5_CLK_GATE_GATED_MASK                                 (0x00000007U)
#define CSL_MSS_RCM_MCSPI5_CLK_GATE_GATED_SHIFT                                (0x00000000U)
#define CSL_MSS_RCM_MCSPI5_CLK_GATE_GATED_RESETVAL                             (0x00000000U)
#define CSL_MSS_RCM_MCSPI5_CLK_GATE_GATED_MAX                                  (0x00000007U)

#define CSL_MSS_RCM_MCSPI5_CLK_GATE_RESETVAL                                   (0x00000000U)

/* MCSPI6_CLK_GATE */

#define CSL_MSS_RCM_MCSPI6_CLK_GATE_GATED_MASK                                 (0x00000007U)
#define CSL_MSS_RCM_MCSPI6_CLK_GATE_GATED_SHIFT                                (0x00000000U)
#define CSL_MSS_RCM_MCSPI6_CLK_GATE_GATED_RESETVAL                             (0x00000000U)
#define CSL_MSS_RCM_MCSPI6_CLK_GATE_GATED_MAX                                  (0x00000007U)

#define CSL_MSS_RCM_MCSPI6_CLK_GATE_RESETVAL                                   (0x00000000U)

/* MCSPI7_CLK_GATE */

#define CSL_MSS_RCM_MCSPI7_CLK_GATE_GATED_MASK                                 (0x00000007U)
#define CSL_MSS_RCM_MCSPI7_CLK_GATE_GATED_SHIFT                                (0x00000000U)
#define CSL_MSS_RCM_MCSPI7_CLK_GATE_GATED_RESETVAL                             (0x00000000U)
#define CSL_MSS_RCM_MCSPI7_CLK_GATE_GATED_MAX                                  (0x00000007U)

#define CSL_MSS_RCM_MCSPI7_CLK_GATE_RESETVAL                                   (0x00000000U)

/* HW_SPARE_RW0 */

#define CSL_MSS_RCM_HW_SPARE_RW0_HW_SPARE_RW0_MASK                             (0xFFFFFFFFU)
#define CSL_MSS_RCM_HW_SPARE_RW0_HW_SPARE_RW0_SHIFT                            (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_RW0_HW_SPARE_RW0_RESETVAL                         (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_RW0_HW_SPARE_RW0_MAX                              (0xFFFFFFFFU)

#define CSL_MSS_RCM_HW_SPARE_RW0_RESETVAL                                      (0x00000000U)

/* HW_SPARE_RW1 */

#define CSL_MSS_RCM_HW_SPARE_RW1_HW_SPARE_RW1_MASK                             (0xFFFFFFFFU)
#define CSL_MSS_RCM_HW_SPARE_RW1_HW_SPARE_RW1_SHIFT                            (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_RW1_HW_SPARE_RW1_RESETVAL                         (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_RW1_HW_SPARE_RW1_MAX                              (0xFFFFFFFFU)

#define CSL_MSS_RCM_HW_SPARE_RW1_RESETVAL                                      (0x00000000U)

/* HW_SPARE_RW2 */

#define CSL_MSS_RCM_HW_SPARE_RW2_HW_SPARE_RW2_MASK                             (0xFFFFFFFFU)
#define CSL_MSS_RCM_HW_SPARE_RW2_HW_SPARE_RW2_SHIFT                            (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_RW2_HW_SPARE_RW2_RESETVAL                         (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_RW2_HW_SPARE_RW2_MAX                              (0xFFFFFFFFU)

#define CSL_MSS_RCM_HW_SPARE_RW2_RESETVAL                                      (0x00000000U)

/* HW_SPARE_RW3 */

#define CSL_MSS_RCM_HW_SPARE_RW3_HW_SPARE_RW3_MASK                             (0xFFFFFFFFU)
#define CSL_MSS_RCM_HW_SPARE_RW3_HW_SPARE_RW3_SHIFT                            (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_RW3_HW_SPARE_RW3_RESETVAL                         (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_RW3_HW_SPARE_RW3_MAX                              (0xFFFFFFFFU)

#define CSL_MSS_RCM_HW_SPARE_RW3_RESETVAL                                      (0x00000000U)

/* HW_SPARE_RO0 */

#define CSL_MSS_RCM_HW_SPARE_RO0_HW_SPARE_RO0_MASK                             (0xFFFFFFFFU)
#define CSL_MSS_RCM_HW_SPARE_RO0_HW_SPARE_RO0_SHIFT                            (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_RO0_HW_SPARE_RO0_RESETVAL                         (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_RO0_HW_SPARE_RO0_MAX                              (0xFFFFFFFFU)

#define CSL_MSS_RCM_HW_SPARE_RO0_RESETVAL                                      (0x00000000U)

/* HW_SPARE_RO1 */

#define CSL_MSS_RCM_HW_SPARE_RO1_HW_SPARE_RO1_MASK                             (0xFFFFFFFFU)
#define CSL_MSS_RCM_HW_SPARE_RO1_HW_SPARE_RO1_SHIFT                            (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_RO1_HW_SPARE_RO1_RESETVAL                         (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_RO1_HW_SPARE_RO1_MAX                              (0xFFFFFFFFU)

#define CSL_MSS_RCM_HW_SPARE_RO1_RESETVAL                                      (0x00000000U)

/* HW_SPARE_RO2 */

#define CSL_MSS_RCM_HW_SPARE_RO2_HW_SPARE_RO2_MASK                             (0xFFFFFFFFU)
#define CSL_MSS_RCM_HW_SPARE_RO2_HW_SPARE_RO2_SHIFT                            (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_RO2_HW_SPARE_RO2_RESETVAL                         (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_RO2_HW_SPARE_RO2_MAX                              (0xFFFFFFFFU)

#define CSL_MSS_RCM_HW_SPARE_RO2_RESETVAL                                      (0x00000000U)

/* HW_SPARE_RO3 */

#define CSL_MSS_RCM_HW_SPARE_RO3_HW_SPARE_RO3_MASK                             (0xFFFFFFFFU)
#define CSL_MSS_RCM_HW_SPARE_RO3_HW_SPARE_RO3_SHIFT                            (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_RO3_HW_SPARE_RO3_RESETVAL                         (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_RO3_HW_SPARE_RO3_MAX                              (0xFFFFFFFFU)

#define CSL_MSS_RCM_HW_SPARE_RO3_RESETVAL                                      (0x00000000U)

/* HW_SPARE_WPH */

#define CSL_MSS_RCM_HW_SPARE_WPH_HW_SPARE_WPH_MASK                             (0xFFFFFFFFU)
#define CSL_MSS_RCM_HW_SPARE_WPH_HW_SPARE_WPH_SHIFT                            (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_WPH_HW_SPARE_WPH_RESETVAL                         (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_WPH_HW_SPARE_WPH_MAX                              (0xFFFFFFFFU)

#define CSL_MSS_RCM_HW_SPARE_WPH_RESETVAL                                      (0x00000000U)

/* HW_SPARE_REC */

#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC0_MASK                            (0x00000001U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC0_SHIFT                           (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC0_RESETVAL                        (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC0_MAX                             (0x00000001U)

#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC1_MASK                            (0x00000002U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC1_SHIFT                           (0x00000001U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC1_RESETVAL                        (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC1_MAX                             (0x00000001U)

#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC2_MASK                            (0x00000004U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC2_SHIFT                           (0x00000002U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC2_RESETVAL                        (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC2_MAX                             (0x00000001U)

#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC3_MASK                            (0x00000008U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC3_SHIFT                           (0x00000003U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC3_RESETVAL                        (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC3_MAX                             (0x00000001U)

#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC4_MASK                            (0x00000010U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC4_SHIFT                           (0x00000004U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC4_RESETVAL                        (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC4_MAX                             (0x00000001U)

#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC5_MASK                            (0x00000020U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC5_SHIFT                           (0x00000005U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC5_RESETVAL                        (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC5_MAX                             (0x00000001U)

#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC6_MASK                            (0x00000040U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC6_SHIFT                           (0x00000006U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC6_RESETVAL                        (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC6_MAX                             (0x00000001U)

#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC7_MASK                            (0x00000080U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC7_SHIFT                           (0x00000007U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC7_RESETVAL                        (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC7_MAX                             (0x00000001U)

#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC8_MASK                            (0x00000100U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC8_SHIFT                           (0x00000008U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC8_RESETVAL                        (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC8_MAX                             (0x00000001U)

#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC9_MASK                            (0x00000200U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC9_SHIFT                           (0x00000009U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC9_RESETVAL                        (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC9_MAX                             (0x00000001U)

#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC10_MASK                           (0x00000400U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC10_SHIFT                          (0x0000000AU)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC10_RESETVAL                       (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC10_MAX                            (0x00000001U)

#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC11_MASK                           (0x00000800U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC11_SHIFT                          (0x0000000BU)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC11_RESETVAL                       (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC11_MAX                            (0x00000001U)

#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC12_MASK                           (0x00001000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC12_SHIFT                          (0x0000000CU)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC12_RESETVAL                       (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC12_MAX                            (0x00000001U)

#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC13_MASK                           (0x00002000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC13_SHIFT                          (0x0000000DU)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC13_RESETVAL                       (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC13_MAX                            (0x00000001U)

#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC14_MASK                           (0x00004000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC14_SHIFT                          (0x0000000EU)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC14_RESETVAL                       (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC14_MAX                            (0x00000001U)

#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC15_MASK                           (0x00008000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC15_SHIFT                          (0x0000000FU)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC15_RESETVAL                       (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC15_MAX                            (0x00000001U)

#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC16_MASK                           (0x00010000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC16_SHIFT                          (0x00000010U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC16_RESETVAL                       (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC16_MAX                            (0x00000001U)

#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC17_MASK                           (0x00020000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC17_SHIFT                          (0x00000011U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC17_RESETVAL                       (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC17_MAX                            (0x00000001U)

#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC18_MASK                           (0x00040000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC18_SHIFT                          (0x00000012U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC18_RESETVAL                       (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC18_MAX                            (0x00000001U)

#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC19_MASK                           (0x00080000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC19_SHIFT                          (0x00000013U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC19_RESETVAL                       (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC19_MAX                            (0x00000001U)

#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC20_MASK                           (0x00100000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC20_SHIFT                          (0x00000014U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC20_RESETVAL                       (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC20_MAX                            (0x00000001U)

#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC21_MASK                           (0x00200000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC21_SHIFT                          (0x00000015U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC21_RESETVAL                       (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC21_MAX                            (0x00000001U)

#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC22_MASK                           (0x00400000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC22_SHIFT                          (0x00000016U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC22_RESETVAL                       (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC22_MAX                            (0x00000001U)

#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC23_MASK                           (0x00800000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC23_SHIFT                          (0x00000017U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC23_RESETVAL                       (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC23_MAX                            (0x00000001U)

#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC24_MASK                           (0x01000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC24_SHIFT                          (0x00000018U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC24_RESETVAL                       (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC24_MAX                            (0x00000001U)

#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC25_MASK                           (0x02000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC25_SHIFT                          (0x00000019U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC25_RESETVAL                       (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC25_MAX                            (0x00000001U)

#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC26_MASK                           (0x04000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC26_SHIFT                          (0x0000001AU)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC26_RESETVAL                       (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC26_MAX                            (0x00000001U)

#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC27_MASK                           (0x08000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC27_SHIFT                          (0x0000001BU)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC27_RESETVAL                       (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC27_MAX                            (0x00000001U)

#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC28_MASK                           (0x10000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC28_SHIFT                          (0x0000001CU)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC28_RESETVAL                       (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC28_MAX                            (0x00000001U)

#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC29_MASK                           (0x20000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC29_SHIFT                          (0x0000001DU)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC29_RESETVAL                       (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC29_MAX                            (0x00000001U)

#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC30_MASK                           (0x40000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC30_SHIFT                          (0x0000001EU)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC30_RESETVAL                       (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC30_MAX                            (0x00000001U)

#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC31_MASK                           (0x80000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC31_SHIFT                          (0x0000001FU)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC31_RESETVAL                       (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC31_MAX                            (0x00000001U)

#define CSL_MSS_RCM_HW_SPARE_REC_RESETVAL                                      (0x00000000U)

/* LOCK0_KICK0 */

#define CSL_MSS_RCM_LOCK0_KICK0_LOCK0_KICK0_MASK                               (0xFFFFFFFFU)
#define CSL_MSS_RCM_LOCK0_KICK0_LOCK0_KICK0_SHIFT                              (0x00000000U)
#define CSL_MSS_RCM_LOCK0_KICK0_LOCK0_KICK0_RESETVAL                           (0x00000000U)
#define CSL_MSS_RCM_LOCK0_KICK0_LOCK0_KICK0_MAX                                (0xFFFFFFFFU)

#define CSL_MSS_RCM_LOCK0_KICK0_RESETVAL                                       (0x00000000U)

/* LOCK0_KICK1 */

#define CSL_MSS_RCM_LOCK0_KICK1_LOCK0_KICK1_MASK                               (0xFFFFFFFFU)
#define CSL_MSS_RCM_LOCK0_KICK1_LOCK0_KICK1_SHIFT                              (0x00000000U)
#define CSL_MSS_RCM_LOCK0_KICK1_LOCK0_KICK1_RESETVAL                           (0x00000000U)
#define CSL_MSS_RCM_LOCK0_KICK1_LOCK0_KICK1_MAX                                (0xFFFFFFFFU)

#define CSL_MSS_RCM_LOCK0_KICK1_RESETVAL                                       (0x00000000U)

/* INTR_RAW_STATUS */

#define CSL_MSS_RCM_INTR_RAW_STATUS_PROT_ERR_MASK                              (0x00000001U)
#define CSL_MSS_RCM_INTR_RAW_STATUS_PROT_ERR_SHIFT                             (0x00000000U)
#define CSL_MSS_RCM_INTR_RAW_STATUS_PROT_ERR_RESETVAL                          (0x00000000U)
#define CSL_MSS_RCM_INTR_RAW_STATUS_PROT_ERR_MAX                               (0x00000001U)

#define CSL_MSS_RCM_INTR_RAW_STATUS_ADDR_ERR_MASK                              (0x00000002U)
#define CSL_MSS_RCM_INTR_RAW_STATUS_ADDR_ERR_SHIFT                             (0x00000001U)
#define CSL_MSS_RCM_INTR_RAW_STATUS_ADDR_ERR_RESETVAL                          (0x00000000U)
#define CSL_MSS_RCM_INTR_RAW_STATUS_ADDR_ERR_MAX                               (0x00000001U)

#define CSL_MSS_RCM_INTR_RAW_STATUS_KICK_ERR_MASK                              (0x00000004U)
#define CSL_MSS_RCM_INTR_RAW_STATUS_KICK_ERR_SHIFT                             (0x00000002U)
#define CSL_MSS_RCM_INTR_RAW_STATUS_KICK_ERR_RESETVAL                          (0x00000000U)
#define CSL_MSS_RCM_INTR_RAW_STATUS_KICK_ERR_MAX                               (0x00000001U)

#define CSL_MSS_RCM_INTR_RAW_STATUS_PROXY_ERR_MASK                             (0x00000008U)
#define CSL_MSS_RCM_INTR_RAW_STATUS_PROXY_ERR_SHIFT                            (0x00000003U)
#define CSL_MSS_RCM_INTR_RAW_STATUS_PROXY_ERR_RESETVAL                         (0x00000000U)
#define CSL_MSS_RCM_INTR_RAW_STATUS_PROXY_ERR_MAX                              (0x00000001U)

#define CSL_MSS_RCM_INTR_RAW_STATUS_RESETVAL                                   (0x00000000U)

/* INTR_ENABLED_STATUS_CLEAR */

#define CSL_MSS_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR_MASK            (0x00000001U)
#define CSL_MSS_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR_SHIFT           (0x00000000U)
#define CSL_MSS_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR_RESETVAL        (0x00000000U)
#define CSL_MSS_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR_MAX             (0x00000001U)

#define CSL_MSS_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR_MASK            (0x00000002U)
#define CSL_MSS_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR_SHIFT           (0x00000001U)
#define CSL_MSS_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR_RESETVAL        (0x00000000U)
#define CSL_MSS_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR_MAX             (0x00000001U)

#define CSL_MSS_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_KICK_ERR_MASK            (0x00000004U)
#define CSL_MSS_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_KICK_ERR_SHIFT           (0x00000002U)
#define CSL_MSS_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_KICK_ERR_RESETVAL        (0x00000000U)
#define CSL_MSS_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_KICK_ERR_MAX             (0x00000001U)

#define CSL_MSS_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROXY_ERR_MASK           (0x00000008U)
#define CSL_MSS_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROXY_ERR_SHIFT          (0x00000003U)
#define CSL_MSS_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROXY_ERR_RESETVAL       (0x00000000U)
#define CSL_MSS_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROXY_ERR_MAX            (0x00000001U)

#define CSL_MSS_RCM_INTR_ENABLED_STATUS_CLEAR_RESETVAL                         (0x00000000U)

/* INTR_ENABLE */

#define CSL_MSS_RCM_INTR_ENABLE_PROT_ERR_EN_MASK                               (0x00000001U)
#define CSL_MSS_RCM_INTR_ENABLE_PROT_ERR_EN_SHIFT                              (0x00000000U)
#define CSL_MSS_RCM_INTR_ENABLE_PROT_ERR_EN_RESETVAL                           (0x00000000U)
#define CSL_MSS_RCM_INTR_ENABLE_PROT_ERR_EN_MAX                                (0x00000001U)

#define CSL_MSS_RCM_INTR_ENABLE_ADDR_ERR_EN_MASK                               (0x00000002U)
#define CSL_MSS_RCM_INTR_ENABLE_ADDR_ERR_EN_SHIFT                              (0x00000001U)
#define CSL_MSS_RCM_INTR_ENABLE_ADDR_ERR_EN_RESETVAL                           (0x00000000U)
#define CSL_MSS_RCM_INTR_ENABLE_ADDR_ERR_EN_MAX                                (0x00000001U)

#define CSL_MSS_RCM_INTR_ENABLE_KICK_ERR_EN_MASK                               (0x00000004U)
#define CSL_MSS_RCM_INTR_ENABLE_KICK_ERR_EN_SHIFT                              (0x00000002U)
#define CSL_MSS_RCM_INTR_ENABLE_KICK_ERR_EN_RESETVAL                           (0x00000000U)
#define CSL_MSS_RCM_INTR_ENABLE_KICK_ERR_EN_MAX                                (0x00000001U)

#define CSL_MSS_RCM_INTR_ENABLE_PROXY_ERR_EN_MASK                              (0x00000008U)
#define CSL_MSS_RCM_INTR_ENABLE_PROXY_ERR_EN_SHIFT                             (0x00000003U)
#define CSL_MSS_RCM_INTR_ENABLE_PROXY_ERR_EN_RESETVAL                          (0x00000000U)
#define CSL_MSS_RCM_INTR_ENABLE_PROXY_ERR_EN_MAX                               (0x00000001U)

#define CSL_MSS_RCM_INTR_ENABLE_RESETVAL                                       (0x00000000U)

/* INTR_ENABLE_CLEAR */

#define CSL_MSS_RCM_INTR_ENABLE_CLEAR_PROT_ERR_EN_CLR_MASK                     (0x00000001U)
#define CSL_MSS_RCM_INTR_ENABLE_CLEAR_PROT_ERR_EN_CLR_SHIFT                    (0x00000000U)
#define CSL_MSS_RCM_INTR_ENABLE_CLEAR_PROT_ERR_EN_CLR_RESETVAL                 (0x00000000U)
#define CSL_MSS_RCM_INTR_ENABLE_CLEAR_PROT_ERR_EN_CLR_MAX                      (0x00000001U)

#define CSL_MSS_RCM_INTR_ENABLE_CLEAR_ADDR_ERR_EN_CLR_MASK                     (0x00000002U)
#define CSL_MSS_RCM_INTR_ENABLE_CLEAR_ADDR_ERR_EN_CLR_SHIFT                    (0x00000001U)
#define CSL_MSS_RCM_INTR_ENABLE_CLEAR_ADDR_ERR_EN_CLR_RESETVAL                 (0x00000000U)
#define CSL_MSS_RCM_INTR_ENABLE_CLEAR_ADDR_ERR_EN_CLR_MAX                      (0x00000001U)

#define CSL_MSS_RCM_INTR_ENABLE_CLEAR_KICK_ERR_EN_CLR_MASK                     (0x00000004U)
#define CSL_MSS_RCM_INTR_ENABLE_CLEAR_KICK_ERR_EN_CLR_SHIFT                    (0x00000002U)
#define CSL_MSS_RCM_INTR_ENABLE_CLEAR_KICK_ERR_EN_CLR_RESETVAL                 (0x00000000U)
#define CSL_MSS_RCM_INTR_ENABLE_CLEAR_KICK_ERR_EN_CLR_MAX                      (0x00000001U)

#define CSL_MSS_RCM_INTR_ENABLE_CLEAR_PROXY_ERR_EN_CLR_MASK                    (0x00000008U)
#define CSL_MSS_RCM_INTR_ENABLE_CLEAR_PROXY_ERR_EN_CLR_SHIFT                   (0x00000003U)
#define CSL_MSS_RCM_INTR_ENABLE_CLEAR_PROXY_ERR_EN_CLR_RESETVAL                (0x00000000U)
#define CSL_MSS_RCM_INTR_ENABLE_CLEAR_PROXY_ERR_EN_CLR_MAX                     (0x00000001U)

#define CSL_MSS_RCM_INTR_ENABLE_CLEAR_RESETVAL                                 (0x00000000U)

/* EOI */

#define CSL_MSS_RCM_EOI_EOI_VECTOR_MASK                                        (0x000000FFU)
#define CSL_MSS_RCM_EOI_EOI_VECTOR_SHIFT                                       (0x00000000U)
#define CSL_MSS_RCM_EOI_EOI_VECTOR_RESETVAL                                    (0x00000000U)
#define CSL_MSS_RCM_EOI_EOI_VECTOR_MAX                                         (0x000000FFU)

#define CSL_MSS_RCM_EOI_RESETVAL                                               (0x00000000U)

/* FAULT_ADDRESS */

#define CSL_MSS_RCM_FAULT_ADDRESS_FAULT_ADDR_MASK                              (0xFFFFFFFFU)
#define CSL_MSS_RCM_FAULT_ADDRESS_FAULT_ADDR_SHIFT                             (0x00000000U)
#define CSL_MSS_RCM_FAULT_ADDRESS_FAULT_ADDR_RESETVAL                          (0x00000000U)
#define CSL_MSS_RCM_FAULT_ADDRESS_FAULT_ADDR_MAX                               (0xFFFFFFFFU)

#define CSL_MSS_RCM_FAULT_ADDRESS_RESETVAL                                     (0x00000000U)

/* FAULT_TYPE_STATUS */

#define CSL_MSS_RCM_FAULT_TYPE_STATUS_FAULT_TYPE_MASK                          (0x0000003FU)
#define CSL_MSS_RCM_FAULT_TYPE_STATUS_FAULT_TYPE_SHIFT                         (0x00000000U)
#define CSL_MSS_RCM_FAULT_TYPE_STATUS_FAULT_TYPE_RESETVAL                      (0x00000000U)
#define CSL_MSS_RCM_FAULT_TYPE_STATUS_FAULT_TYPE_MAX                           (0x0000003FU)

#define CSL_MSS_RCM_FAULT_TYPE_STATUS_FAULT_NS_MASK                            (0x00000040U)
#define CSL_MSS_RCM_FAULT_TYPE_STATUS_FAULT_NS_SHIFT                           (0x00000006U)
#define CSL_MSS_RCM_FAULT_TYPE_STATUS_FAULT_NS_RESETVAL                        (0x00000000U)
#define CSL_MSS_RCM_FAULT_TYPE_STATUS_FAULT_NS_MAX                             (0x00000001U)

#define CSL_MSS_RCM_FAULT_TYPE_STATUS_RESETVAL                                 (0x00000000U)

/* FAULT_ATTR_STATUS */

#define CSL_MSS_RCM_FAULT_ATTR_STATUS_FAULT_PRIVID_MASK                        (0x000000FFU)
#define CSL_MSS_RCM_FAULT_ATTR_STATUS_FAULT_PRIVID_SHIFT                       (0x00000000U)
#define CSL_MSS_RCM_FAULT_ATTR_STATUS_FAULT_PRIVID_RESETVAL                    (0x00000000U)
#define CSL_MSS_RCM_FAULT_ATTR_STATUS_FAULT_PRIVID_MAX                         (0x000000FFU)

#define CSL_MSS_RCM_FAULT_ATTR_STATUS_FAULT_ROUTEID_MASK                       (0x000FFF00U)
#define CSL_MSS_RCM_FAULT_ATTR_STATUS_FAULT_ROUTEID_SHIFT                      (0x00000008U)
#define CSL_MSS_RCM_FAULT_ATTR_STATUS_FAULT_ROUTEID_RESETVAL                   (0x00000000U)
#define CSL_MSS_RCM_FAULT_ATTR_STATUS_FAULT_ROUTEID_MAX                        (0x00000FFFU)

#define CSL_MSS_RCM_FAULT_ATTR_STATUS_FAULT_XID_MASK                           (0xFFF00000U)
#define CSL_MSS_RCM_FAULT_ATTR_STATUS_FAULT_XID_SHIFT                          (0x00000014U)
#define CSL_MSS_RCM_FAULT_ATTR_STATUS_FAULT_XID_RESETVAL                       (0x00000000U)
#define CSL_MSS_RCM_FAULT_ATTR_STATUS_FAULT_XID_MAX                            (0x00000FFFU)

#define CSL_MSS_RCM_FAULT_ATTR_STATUS_RESETVAL                                 (0x00000000U)

/* FAULT_CLEAR */

#define CSL_MSS_RCM_FAULT_CLEAR_FAULT_CLR_MASK                                 (0x00000001U)
#define CSL_MSS_RCM_FAULT_CLEAR_FAULT_CLR_SHIFT                                (0x00000000U)
#define CSL_MSS_RCM_FAULT_CLEAR_FAULT_CLR_RESETVAL                             (0x00000000U)
#define CSL_MSS_RCM_FAULT_CLEAR_FAULT_CLR_MAX                                  (0x00000001U)

#define CSL_MSS_RCM_FAULT_CLEAR_RESETVAL                                       (0x00000000U)

#ifdef __cplusplus
}
#endif
#endif
