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
    volatile uint8_t  Resv_32[12];
    volatile uint32_t R5SS0_RST_CAUSE_CLR;
    volatile uint8_t  Resv_48[12];
    volatile uint32_t SYSRST_BY_DBG_RST0;
    volatile uint8_t  Resv_64[12];
    volatile uint32_t RST_ASSERDLY0;
    volatile uint8_t  Resv_80[12];
    volatile uint32_t R5SS0_RST2ASSERTDLY;
    volatile uint8_t  Resv_96[12];
    volatile uint32_t R5SS0_RST_WFICHECK;
    volatile uint8_t  Resv_256[156];
    volatile uint32_t MCAN0_CLK_SRC_SEL;
    volatile uint32_t MCAN1_CLK_SRC_SEL;
    volatile uint8_t  Resv_320[56];
    volatile uint32_t RTI0_CLK_SRC_SEL;
    volatile uint32_t RTI1_CLK_SRC_SEL;
    volatile uint32_t RTI2_CLK_SRC_SEL;
    volatile uint32_t RTI3_CLK_SRC_SEL;
    volatile uint8_t  Resv_384[48];
    volatile uint32_t MCSPI0_CLK_SRC_SEL;
    volatile uint32_t MCSPI1_CLK_SRC_SEL;
    volatile uint32_t MCSPI2_CLK_SRC_SEL;
    volatile uint32_t MCSPI3_CLK_SRC_SEL;
    volatile uint8_t  Resv_448[48];
    volatile uint32_t WDT0_CLK_SRC_SEL;
    volatile uint32_t WDT1_CLK_SRC_SEL;
    volatile uint8_t  Resv_480[24];
    volatile uint32_t ICSSM0_UART0_CLK_SRC_SEL;
    volatile uint32_t ICSSM1_UART0_CLK_SRC_SEL;
    volatile uint8_t  Resv_496[8];
    volatile uint32_t OSPI0_CLK_SRC_SEL;
	volatile uint32_t OSPI1_CLK_SRC_SEL;
    volatile uint32_t CONTROLSS_PLL_CLK_SRC_SEL;
    volatile uint32_t CPTS_CLK_SRC_SEL;
    volatile uint32_t GPMC_CLK_SRC_SEL;
    volatile uint32_t MMC0_CLK_SRC_SEL;
    volatile uint8_t  Resv_544[24];
    volatile uint32_t CPSW_5_50_250_CLK_MUX_CTRL;
    volatile uint32_t I2C_CLK_SRC_SEL;
    volatile uint8_t  Resv_612[60];
    volatile uint32_t LIN0_UART0_CLK_SRC_SEL;
    volatile uint32_t LIN1_UART1_CLK_SRC_SEL;
    volatile uint32_t LIN2_UART2_CLK_SRC_SEL;
    volatile uint32_t LIN3_UART3_CLK_SRC_SEL;
	volatile uint32_t LIN4_UART4_CLK_SRC_SEL;		
	volatile uint32_t LIN5_UART5_CLK_SRC_SEL;		
	volatile uint8_t  Resv_740[104];
	volatile uint32_t ICSSM0_CORE_CLK_SRC_SEL;		
	volatile uint32_t ICSSM1_CORE_CLK_SRC_SEL;		
	volatile uint8_t  Resv_768[20];
    volatile uint32_t MCAN0_CLK_DIV_VAL;
    volatile uint32_t MCAN1_CLK_DIV_VAL;
    volatile uint8_t  Resv_832[56];
    volatile uint32_t RTI0_CLK_DIV_VAL;
    volatile uint32_t RTI1_CLK_DIV_VAL;
    volatile uint32_t RTI2_CLK_DIV_VAL;
    volatile uint32_t RTI3_CLK_DIV_VAL;
    volatile uint8_t  Resv_896[48];
    volatile uint32_t MCSPI0_CLK_DIV_VAL;
    volatile uint32_t MCSPI1_CLK_DIV_VAL;
    volatile uint32_t MCSPI2_CLK_DIV_VAL;
    volatile uint32_t MCSPI3_CLK_DIV_VAL;
    volatile uint8_t  Resv_960[48];
    volatile uint32_t WDT0_CLK_DIV_VAL;
    volatile uint32_t WDT1_CLK_DIV_VAL;
    volatile uint8_t  Resv_992[24];
    volatile uint32_t ICSSM0_UART_CLK_DIV_VAL;
    volatile uint32_t ICSSM1_UART_CLK_DIV_VAL;
    volatile uint8_t  Resv_1008[8];
    volatile uint32_t OSPI0_CLK_DIV_VAL;
	volatile uint32_t OSPI1_CLK_DIV_VAL;
    volatile uint32_t CONTROLSS_PLL_CLK_DIV_VAL;
    volatile uint32_t CPTS_CLK_DIV_VAL;
    volatile uint32_t GPMC_CLK_DIV_VAL;
    volatile uint32_t MMC0_CLK_DIV_VAL;
    volatile uint32_t MSS_ELM_CLK_DIV_VAL;
    volatile uint32_t MII10_CLK_DIV_VAL;
    volatile uint32_t MII100_CLK_DIV_VAL;
    volatile uint32_t RGMII_CLK_DIV_VAL;
    volatile uint32_t XTAL_32K_CLK_DIV_VAL;
    volatile uint32_t XTAL_TEMPSENSE_32K_CLK_DIV_VAL;
    volatile uint8_t  Resv_1060[4];
    volatile uint32_t I2C_CLK_DIV_VAL;
    volatile uint8_t  Resv_1124[60];
    volatile uint32_t LIN0_UART0_CLK_DIV_VAL;
    volatile uint32_t LIN1_UART1_CLK_DIV_VAL;
    volatile uint32_t LIN2_UART2_CLK_DIV_VAL;
    volatile uint32_t LIN3_UART3_CLK_DIV_VAL;
	volatile uint32_t LIN4_UART4_CLK_DIV_VAL;		
	volatile uint32_t LIN5_UART5_CLK_DIV_VAL;		
	volatile uint8_t  Resv_1252[104];
	volatile uint32_t ICSSM0_CORE_CLK_DIV_VAL;		
	volatile uint32_t ICSSM1_CORE_CLK_DIV_VAL;		
	volatile uint8_t  Resv_1280[20];
    volatile uint32_t MCAN0_CLK_GATE;
    volatile uint32_t MCAN1_CLK_GATE;
    volatile uint8_t  Resv_1344[56];
    volatile uint32_t RTI0_CLK_GATE;
    volatile uint32_t RTI1_CLK_GATE;
    volatile uint32_t RTI2_CLK_GATE;
    volatile uint32_t RTI3_CLK_GATE;
    volatile uint8_t  Resv_1408[48];
    volatile uint32_t MCSPI0_CLK_GATE;
    volatile uint32_t MCSPI1_CLK_GATE;
    volatile uint32_t MCSPI2_CLK_GATE;
    volatile uint32_t MCSPI3_CLK_GATE;
    volatile uint8_t  Resv_1472[48];
    volatile uint32_t WDT0_CLK_GATE;
    volatile uint32_t WDT1_CLK_GATE;
    volatile uint8_t  Resv_1504[24];
    volatile uint32_t ICSSM0_UART_CLK_GATE;
    volatile uint32_t ICSSM1_UART_CLK_GATE;
    volatile uint8_t  Resv_1520[8];
    volatile uint32_t OSPI0_CLK_GATE;
	volatile uint32_t OSPI1_CLK_GATE;
    volatile uint32_t CONTROLSS_PLL_CLK_GATE;
    volatile uint32_t CPTS_CLK_GATE;
    volatile uint32_t GPMC_CLK_GATE;
    volatile uint32_t MMC0_CLK_GATE;
    volatile uint32_t MSS_ELM_CLK_GATE;
    volatile uint32_t MII10_CLK_GATE;
    volatile uint32_t MII100_CLK_GATE;
    volatile uint32_t RGMII_CLK_GATE;
    volatile uint32_t MMC0_32K_CLK_GATE;
    volatile uint32_t TEMPSENSE_32K_CLK_GATE;
    volatile uint32_t CPSW_CLK_GATE;
    volatile uint32_t I2C0_CLK_GATE;
    volatile uint32_t I2C1_CLK_GATE;
    volatile uint32_t I2C2_CLK_GATE;
    volatile uint8_t  Resv_1636[52];
    volatile uint32_t LIN0_CLK_GATE;
    volatile uint32_t LIN1_CLK_GATE;
    volatile uint32_t LIN2_CLK_GATE;
    volatile uint8_t  Resv_1700[52];
    volatile uint32_t UART0_CLK_GATE;
    volatile uint32_t UART1_CLK_GATE;
    volatile uint32_t UART2_CLK_GATE;
    volatile uint32_t UART3_CLK_GATE;
	volatile uint32_t UART4_CLK_GATE;		
	volatile uint32_t UART5_CLK_GATE;		
	volatile uint8_t  Resv_1764[40];
	volatile uint32_t ICSSM0_CORE_CLK_GATE;		
	volatile uint32_t ICSSM1_CORE_CLK_GATE;		
	volatile uint8_t  Resv_1792[20];
    volatile uint32_t R5SS0_CORE0_GATE;
    volatile uint8_t  Resv_1808[12];
    volatile uint32_t R5SS0_CORE1_GATE;
    volatile uint8_t  Resv_1824[12];
    volatile uint32_t ICSSM0_IEP_CLK_GATE;
    volatile uint32_t ICSSM1_IEP_CLK_GATE;
	volatile uint8_t  Resv_1840[8];
    volatile uint32_t ICSSM0_SYS_CLK_GATE;
    volatile uint32_t ICSSM1_SYS_CLK_GATE;
	volatile uint8_t  Resv_1856[8];
    volatile uint32_t USB_CLK_GATE;
    volatile uint32_t USB_WKUP_CLK_GATE;
	volatile uint32_t USB_XTAL_CLK_GATE;		
	volatile uint8_t  Resv_2048[180];
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
    volatile uint32_t MCAN0_CLK_STATUS;
    volatile uint32_t MCAN1_CLK_STATUS;
    volatile uint8_t  Resv_2368[56];
    volatile uint32_t RTI0_CLK_STATUS;
    volatile uint32_t RTI1_CLK_STATUS;
    volatile uint32_t RTI2_CLK_STATUS;
    volatile uint32_t RTI3_CLK_STATUS;
    volatile uint8_t  Resv_2432[48];
    volatile uint32_t MCSPI0_CLK_STATUS;
    volatile uint32_t MCSPI1_CLK_STATUS;
    volatile uint32_t MCSPI2_CLK_STATUS;
    volatile uint32_t MCSPI3_CLK_STATUS;
    volatile uint8_t  Resv_2496[48];
    volatile uint32_t WDT0_CLK_STATUS;
    volatile uint32_t WDT1_CLK_STATUS;
    volatile uint8_t  Resv_2528[24];
    volatile uint32_t ICSSM0_UART_CLK_STATUS;
    volatile uint32_t ICSSM1_UART_CLK_STATUS;
    volatile uint8_t  Resv_2544[8];
    volatile uint32_t OSPI0_CLK_STATUS;
	volatile uint32_t OSPI1_CLK_STATUS;
    volatile uint32_t CONTROLSS_PLL_CLK_STATUS;
    volatile uint32_t CPTS_CLK_STATUS;
    volatile uint32_t GPMC_CLK_STATUS;
    volatile uint32_t MMC0_CLK_STATUS;
    volatile uint32_t MSS_ELM_CLK_STATUS;
    volatile uint32_t MII10_CLK_STATUS;
    volatile uint32_t MII100_CLK_STATUS;
    volatile uint32_t RGMII_CLK_STATUS;
    volatile uint32_t MMC0_32K_CLK_STATUS;
    volatile uint32_t TEMPSENSE_32K_CLK_STATUS;
	volatile uint32_t CPSW_5_50_250_CLK_STATUS;
    volatile uint32_t I2C_CLK_STATUS;
    volatile uint8_t  Resv_2660[60];
    volatile uint32_t LIN0_UART0_CLK_STATUS;
    volatile uint32_t LIN1_UART1_CLK_STATUS;
    volatile uint32_t LIN2_UART2_CLK_STATUS;
    volatile uint32_t LIN3_UART3_CLK_STATUS;
	volatile uint32_t LIN4_UART4_CLK_STATUS;		
	volatile uint32_t LIN5_UART5_CLK_STATUS;		
	volatile uint8_t  Resv_2788[104];
	volatile uint32_t ICSSM0_CORE_CLK_STATUS;		
	volatile uint32_t ICSSM1_CORE_CLK_STATUS;		
	volatile uint8_t  Resv_2816[20];
    volatile uint32_t MCAN0_RST_CTRL;
    volatile uint32_t MCAN1_RST_CTRL;
    volatile uint8_t  Resv_2880[56];
    volatile uint32_t RTI0_RST_CTRL;
    volatile uint32_t RTI1_RST_CTRL;
    volatile uint32_t RTI2_RST_CTRL;
    volatile uint32_t RTI3_RST_CTRL;
    volatile uint8_t  Resv_2944[48];
    volatile uint32_t MCSPI0_RST_CTRL;
    volatile uint32_t MCSPI1_RST_CTRL;
    volatile uint32_t MCSPI2_RST_CTRL;
    volatile uint32_t MCSPI3_RST_CTRL;
    volatile uint8_t  Resv_3008[48];
    volatile uint32_t WDT0_RST_CTRL;
    volatile uint32_t WDT1_RST_CTRL;
    volatile uint8_t  Resv_3040[24];
    volatile uint32_t ICSSM0_RST_CTRL;
    volatile uint32_t ICSSM1_RST_CTRL;
    volatile uint8_t  Resv_3056[8];
    volatile uint32_t OSPI0_RST_CTRL;
	volatile uint32_t OSPI1_RST_CTRL;		
	volatile uint8_t  Resv_3072[8];
    volatile uint32_t GPMC_RST_CTRL;
    volatile uint32_t MMC0_RST_CTRL;
    volatile uint32_t MSS_ELM_RST_CTRL;
    volatile uint8_t  Resv_3100[16];
    volatile uint32_t TEMPSENSE_32K_RST_CTRL;
    volatile uint32_t CPSW_RST_CTRL;
    volatile uint32_t I2C0_RST_CTRL;
    volatile uint32_t I2C1_RST_CTRL;
    volatile uint32_t I2C2_RST_CTRL;
    volatile uint8_t  Resv_3172[52];
    volatile uint32_t LIN0_RST_CTRL;
    volatile uint32_t LIN1_RST_CTRL;
    volatile uint32_t LIN2_RST_CTRL;
    volatile uint8_t  Resv_3236[52];
    volatile uint32_t UART0_RST_CTRL;
    volatile uint32_t UART1_RST_CTRL;
    volatile uint32_t UART2_RST_CTRL;
    volatile uint32_t UART3_RST_CTRL;
	volatile uint32_t UART4_RST_CTRL;		
	volatile uint32_t UART5_RST_CTRL;		
	volatile uint8_t  Resv_3328[68];
    volatile uint32_t R5SS0_POR_RST_CTRL;
    volatile uint8_t  Resv_3344[12];
    volatile uint32_t R5SS0_CORE0_GRST_CTRL;
    volatile uint8_t  Resv_3360[12];
    volatile uint32_t R5SS0_CORE1_GRST_CTRL;
    volatile uint8_t  Resv_3376[12];
    volatile uint32_t R5SS0_CORE0_LRST_CTRL;
    volatile uint8_t  Resv_3392[12];
    volatile uint32_t R5SS0_CORE1_LRST_CTRL;
    volatile uint8_t  Resv_3408[12];
    volatile uint32_t VIMA_RST_CTRL0;
    volatile uint8_t  Resv_3424[12];
    volatile uint32_t VIMB_RST_CTRL0;
    volatile uint8_t  Resv_3440[12];
    volatile uint32_t GPIO0_RST_CTRL;
    volatile uint32_t GPIO1_RST_CTRL;
    volatile uint8_t  Resv_3536[88];
    volatile uint32_t EDMA_RST_CTRL;
    volatile uint32_t INFRA_RST_CTRL;
    volatile uint32_t SPINLOCK_RST_CTRL;
    volatile uint32_t USB_RST_CTRL;
    volatile uint32_t CRC_RST_CTRL;
    volatile uint32_t ESM_RST_CTRL;
    volatile uint32_t DCCA_RST_CTRL;
    volatile uint32_t DCCB_RST_CTRL;
    volatile uint32_t DCCC_RST_CTRL;
    volatile uint32_t DCCD_RST_CTRL;
    volatile uint8_t  Resv_3584[8];
    volatile uint32_t L2OCRAM_BANK0_PD_CTRL;
    volatile uint32_t L2OCRAM_BANK1_PD_CTRL;
    volatile uint32_t L2OCRAM_BANK2_PD_CTRL;
    volatile uint8_t  Resv_3616[20];
    volatile uint32_t L2OCRAM_BANK0_PD_STATUS;
    volatile uint32_t L2OCRAM_BANK1_PD_STATUS;
    volatile uint32_t L2OCRAM_BANK2_PD_STATUS;
    volatile uint8_t  Resv_3840[212];
    volatile uint32_t HW_REG0;
    volatile uint32_t HW_REG1;
    volatile uint32_t HW_REG2;
    volatile uint32_t HW_REG3;
    volatile uint8_t  Resv_3904[48];
    volatile uint32_t HW_SPARE_RW0;
    volatile uint32_t HW_SPARE_RW1;
    volatile uint32_t HW_SPARE_RW2;
    volatile uint32_t HW_SPARE_RW3;
    volatile uint8_t  Resv_3968[48];
    volatile uint32_t HW_SPARE_RO0;
    volatile uint32_t HW_SPARE_RO1;
    volatile uint32_t HW_SPARE_RO2;
    volatile uint32_t HW_SPARE_RO3;
    volatile uint8_t  Resv_4032[48];
    volatile uint32_t HW_SPARE_WPH;
    volatile uint32_t HW_SPARE_REC;
    volatile uint8_t  Resv_4104[64];
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

#define CSL_MSS_RCM_PID                                                    (0x00000000U)
#define CSL_MSS_RCM_R5SS0_RST_STATUS                                       (0x00000010U)
#define CSL_MSS_RCM_R5SS0_RST_CAUSE_CLR                                    (0x00000020U)
#define CSL_MSS_RCM_SYSRST_BY_DBG_RST0                                     (0x00000030U)
#define CSL_MSS_RCM_RST_ASSERDLY0                                          (0x00000040U)
#define CSL_MSS_RCM_R5SS0_RST2ASSERTDLY                                    (0x00000050U)
#define CSL_MSS_RCM_R5SS0_RST_WFICHECK                                     (0x00000060U)
#define CSL_MSS_RCM_MCAN0_CLK_SRC_SEL                                      (0x00000100U)
#define CSL_MSS_RCM_MCAN1_CLK_SRC_SEL                                      (0x00000104U)
#define CSL_MSS_RCM_RTI0_CLK_SRC_SEL                                       (0x00000140U)
#define CSL_MSS_RCM_RTI1_CLK_SRC_SEL                                       (0x00000144U)
#define CSL_MSS_RCM_RTI2_CLK_SRC_SEL                                       (0x00000148U)
#define CSL_MSS_RCM_RTI3_CLK_SRC_SEL                                       (0x0000014CU)
#define CSL_MSS_RCM_MCSPI0_CLK_SRC_SEL                                     (0x00000180U)
#define CSL_MSS_RCM_MCSPI1_CLK_SRC_SEL                                     (0x00000184U)
#define CSL_MSS_RCM_MCSPI2_CLK_SRC_SEL                                     (0x00000188U)
#define CSL_MSS_RCM_MCSPI3_CLK_SRC_SEL                                     (0x0000018CU)
#define CSL_MSS_RCM_WDT0_CLK_SRC_SEL                                       (0x000001C0U)
#define CSL_MSS_RCM_WDT1_CLK_SRC_SEL                                       (0x000001C4U)
#define CSL_MSS_RCM_ICSSM0_UART_CLK_SRC_SEL                                (0x000001E0U)
#define CSL_MSS_RCM_ICSSM1_UART_CLK_SRC_SEL                                (0x000001E4U)
#define CSL_MSS_RCM_OSPI0_CLK_SRC_SEL                                      (0x000001F0U)
#define CSL_MSS_RCM_OSPI1_CLK_SRC_SEL                                      (0x000001F4U)
#define CSL_MSS_RCM_CONTROLSS_PLL_CLK_SRC_SEL                              (0x000001F8U)
#define CSL_MSS_RCM_CPTS_CLK_SRC_SEL                                       (0x000001FCU)
#define CSL_MSS_RCM_GPMC_CLK_SRC_SEL                                       (0x00000200U)
#define CSL_MSS_RCM_MMC0_CLK_SRC_SEL                                       (0x00000204U)
#define CSL_MSS_RCM_CPSW_5_50_250_CLK_MUX_CTRL                             (0x00000220U)
#define CSL_MSS_RCM_I2C_CLK_SRC_SEL                                        (0x00000224U)
#define CSL_MSS_RCM_LIN0_UART0_CLK_SRC_SEL                                 (0x00000264U)
#define CSL_MSS_RCM_LIN1_UART1_CLK_SRC_SEL                                 (0x00000268U)
#define CSL_MSS_RCM_LIN2_UART2_CLK_SRC_SEL                                 (0x0000026CU)
#define CSL_MSS_RCM_LIN3_UART3_CLK_SRC_SEL                                 (0x00000270U)
#define CSL_MSS_RCM_LIN4_UART4_CLK_SRC_SEL                                 (0x00000274U)
#define CSL_MSS_RCM_LIN5_UART5_CLK_SRC_SEL                                 (0x00000278U)
#define CSL_MSS_RCM_ICSSM0_CORE_CLK_SRC_SEL                                (0x000002E4U)
#define CSL_MSS_RCM_ICSSM1_CORE_CLK_SRC_SEL                                (0x000002E8U)
#define CSL_MSS_RCM_MCAN0_CLK_DIV_VAL                                      (0x00000300U)
#define CSL_MSS_RCM_MCAN1_CLK_DIV_VAL                                      (0x00000304U)
#define CSL_MSS_RCM_RTI0_CLK_DIV_VAL                                       (0x00000340U)
#define CSL_MSS_RCM_RTI1_CLK_DIV_VAL                                       (0x00000344U)
#define CSL_MSS_RCM_RTI2_CLK_DIV_VAL                                       (0x00000348U)
#define CSL_MSS_RCM_RTI3_CLK_DIV_VAL                                       (0x0000034CU)
#define CSL_MSS_RCM_MCSPI0_CLK_DIV_VAL                                     (0x00000380U)
#define CSL_MSS_RCM_MCSPI1_CLK_DIV_VAL                                     (0x00000384U)
#define CSL_MSS_RCM_MCSPI2_CLK_DIV_VAL                                     (0x00000388U)
#define CSL_MSS_RCM_MCSPI3_CLK_DIV_VAL                                     (0x0000038CU)
#define CSL_MSS_RCM_WDT0_CLK_DIV_VAL                                       (0x000003C0U)
#define CSL_MSS_RCM_WDT1_CLK_DIV_VAL                                       (0x000003C4U)
#define CSL_MSS_RCM_ICSSM0_UART_CLK_DIV_VAL                                (0x000003E0U)
#define CSL_MSS_RCM_ICSSM1_UART_CLK_DIV_VAL                                (0x000003E4U)
#define CSL_MSS_RCM_OSPI0_CLK_DIV_VAL                                      (0x000003F0U)
#define CSL_MSS_RCM_OSPI1_CLK_DIV_VAL                                      (0x000003F4U)
#define CSL_MSS_RCM_CONTROLSS_PLL_CLK_DIV_VAL                              (0x000003F8U)
#define CSL_MSS_RCM_CPTS_CLK_DIV_VAL                                       (0x000003FCU)
#define CSL_MSS_RCM_GPMC_CLK_DIV_VAL                                       (0x00000400U)
#define CSL_MSS_RCM_MMC0_CLK_DIV_VAL                                       (0x00000404U)
#define CSL_MSS_RCM_MSS_ELM_CLK_DIV_VAL                                    (0x00000408U)
#define CSL_MSS_RCM_MII10_CLK_DIV_VAL                                      (0x0000040CU)
#define CSL_MSS_RCM_MII100_CLK_DIV_VAL                                     (0x00000410U)
#define CSL_MSS_RCM_RGMII_CLK_DIV_VAL                                      (0x00000414U)
#define CSL_MSS_RCM_XTAL_32K_CLK_DIV_VAL                                   (0x00000418U)
#define CSL_MSS_RCM_XTAL_TEMPSENSE_32K_CLK_DIV_VAL                         (0x0000041CU)
#define CSL_MSS_RCM_I2C_CLK_DIV_VAL                                        (0x00000424U)
#define CSL_MSS_RCM_LIN0_UART0_CLK_DIV_VAL                                 (0x00000464U)
#define CSL_MSS_RCM_LIN1_UART1_CLK_DIV_VAL                                 (0x00000468U)
#define CSL_MSS_RCM_LIN2_UART2_CLK_DIV_VAL                                 (0x0000046CU)
#define CSL_MSS_RCM_LIN3_UART3_CLK_DIV_VAL                                 (0x00000470U)
#define CSL_MSS_RCM_LIN4_UART4_CLK_DIV_VAL                                 (0x00000474U)
#define CSL_MSS_RCM_LIN5_UART5_CLK_DIV_VAL                                 (0x00000478U)
#define CSL_MSS_RCM_ICSSM0_CORE_CLK_DIV_VAL                                (0x000004E4U)
#define CSL_MSS_RCM_ICSSM1_CORE_CLK_DIV_VAL                                (0x000004E8U)
#define CSL_MSS_RCM_MCAN0_CLK_GATE                                         (0x00000500U)
#define CSL_MSS_RCM_MCAN1_CLK_GATE                                         (0x00000504U)
#define CSL_MSS_RCM_RTI0_CLK_GATE                                          (0x00000540U)
#define CSL_MSS_RCM_RTI1_CLK_GATE                                          (0x00000544U)
#define CSL_MSS_RCM_RTI2_CLK_GATE                                          (0x00000548U)
#define CSL_MSS_RCM_RTI3_CLK_GATE                                          (0x0000054CU)
#define CSL_MSS_RCM_MCSPI0_CLK_GATE                                        (0x00000580U)
#define CSL_MSS_RCM_MCSPI1_CLK_GATE                                        (0x00000584U)
#define CSL_MSS_RCM_MCSPI2_CLK_GATE                                        (0x00000588U)
#define CSL_MSS_RCM_MCSPI3_CLK_GATE                                        (0x0000058CU)
#define CSL_MSS_RCM_WDT0_CLK_GATE                                          (0x000005C0U)
#define CSL_MSS_RCM_WDT1_CLK_GATE                                          (0x000005C4U)
#define CSL_MSS_RCM_ICSSM0_UART_CLK_GATE                                   (0x000005E0U)
#define CSL_MSS_RCM_ICSSM1_UART_CLK_GATE                                   (0x000005E4U)
#define CSL_MSS_RCM_OSPI0_CLK_GATE                                         (0x000005F0U)
#define CSL_MSS_RCM_OSPI1_CLK_GATE                                         (0x000005F4U)
#define CSL_MSS_RCM_CONTROLSS_PLL_CLK_GATE                                 (0x000005F8U)
#define CSL_MSS_RCM_CPTS_CLK_GATE                                          (0x000005FCU)
#define CSL_MSS_RCM_GPMC_CLK_GATE                                          (0x00000600U)
#define CSL_MSS_RCM_MMC0_CLK_GATE                                          (0x00000604U)
#define CSL_MSS_RCM_MSS_ELM_CLK_GATE                                       (0x00000608U)
#define CSL_MSS_RCM_MII10_CLK_GATE                                         (0x0000060CU)
#define CSL_MSS_RCM_MII100_CLK_GATE                                        (0x00000610U)
#define CSL_MSS_RCM_RGMII_CLK_GATE                                         (0x00000614U)
#define CSL_MSS_RCM_MMC0_32K_CLK_GATE                                      (0x00000618U)
#define CSL_MSS_RCM_TEMPSENSE_32K_CLK_GATE                                 (0x0000061CU)
#define CSL_MSS_RCM_CPSW_CLK_GATE                                          (0x00000620U)
#define CSL_MSS_RCM_I2C0_CLK_GATE                                          (0x00000624U)
#define CSL_MSS_RCM_I2C1_CLK_GATE                                          (0x00000628U)
#define CSL_MSS_RCM_I2C2_CLK_GATE                                          (0x0000062CU)
#define CSL_MSS_RCM_LIN0_CLK_GATE                                          (0x00000664U)
#define CSL_MSS_RCM_LIN1_CLK_GATE                                          (0x00000668U)
#define CSL_MSS_RCM_LIN2_CLK_GATE                                          (0x0000066CU)
#define CSL_MSS_RCM_UART0_CLK_GATE                                         (0x000006A4U)
#define CSL_MSS_RCM_UART1_CLK_GATE                                         (0x000006A8U)
#define CSL_MSS_RCM_UART2_CLK_GATE                                         (0x000006ACU)
#define CSL_MSS_RCM_UART3_CLK_GATE                                         (0x000006B0U)
#define CSL_MSS_RCM_UART4_CLK_GATE                                         (0x000006B4U)
#define CSL_MSS_RCM_UART5_CLK_GATE                                         (0x000006B8U)
#define CSL_MSS_RCM_ICSSM0_CORE_CLK_GATE                                   (0x000006E4U)
#define CSL_MSS_RCM_ICSSM1_CORE_CLK_GATE                                   (0x000006E8U)
#define CSL_MSS_RCM_R5SS0_CORE0_GATE                                         (0x00000700U)
#define CSL_MSS_RCM_R5SS0_CORE1_GATE                                         (0x00000710U)
#define CSL_MSS_RCM_ICSSM0_IEP_CLK_GATE                                    (0x00000720U)
#define CSL_MSS_RCM_ICSSM1_IEP_CLK_GATE                                    (0x00000724U)
#define CSL_MSS_RCM_ICSSM0_SYS_CLK_GATE                                    (0x00000730U)
#define CSL_MSS_RCM_ICSSM1_SYS_CLK_GATE                                    (0x00000734U)
#define CSL_MSS_RCM_USB_CLK_GATE                                           (0x00000740U)
#define CSL_MSS_RCM_USB_WKUP_CLK_GATE                                      (0x00000744U)
#define CSL_MSS_RCM_USB_XTAL_CLK_GATE                                      (0x00000748U)
#define CSL_MSS_RCM_HSM_RTIA_CLK_SRC_SEL                                   (0x00000800U)
#define CSL_MSS_RCM_HSM_WDT_CLK_SRC_SEL                                    (0x00000804U)
#define CSL_MSS_RCM_HSM_RTC_CLK_SRC_SEL                                    (0x00000808U)
#define CSL_MSS_RCM_HSM_DMTA_CLK_SRC_SEL                                   (0x0000080CU)
#define CSL_MSS_RCM_HSM_DMTB_CLK_SRC_SEL                                   (0x00000810U)
#define CSL_MSS_RCM_HSM_RTI_CLK_DIV_VAL                                    (0x00000814U)
#define CSL_MSS_RCM_HSM_WDT_CLK_DIV_VAL                                    (0x00000818U)
#define CSL_MSS_RCM_HSM_RTC_CLK_DIV_VAL                                    (0x0000081CU)
#define CSL_MSS_RCM_HSM_DMTA_CLK_DIV_VAL                                   (0x00000820U)
#define CSL_MSS_RCM_HSM_DMTB_CLK_DIV_VAL                                   (0x00000824U)
#define CSL_MSS_RCM_HSM_RTI_CLK_GATE                                       (0x00000828U)
#define CSL_MSS_RCM_HSM_WDT_CLK_GATE                                       (0x0000082CU)
#define CSL_MSS_RCM_HSM_RTC_CLK_GATE                                       (0x00000830U)
#define CSL_MSS_RCM_HSM_DMTA_CLK_GATE                                      (0x00000834U)
#define CSL_MSS_RCM_HSM_DMTB_CLK_GATE                                      (0x00000838U)
#define CSL_MSS_RCM_HSM_RTI_CLK_STATUS                                     (0x0000083CU)
#define CSL_MSS_RCM_HSM_WDT_CLK_STATUS                                     (0x00000840U)
#define CSL_MSS_RCM_HSM_RTC_CLK_STATUS                                     (0x00000844U)
#define CSL_MSS_RCM_HSM_DMTA_CLK_STATUS                                    (0x00000848U)
#define CSL_MSS_RCM_HSM_DMTB_CLK_STATUS                                    (0x0000084CU)
#define CSL_MSS_RCM_MCAN0_CLK_STATUS                                       (0x00000900U)
#define CSL_MSS_RCM_MCAN1_CLK_STATUS                                       (0x00000904U)
#define CSL_MSS_RCM_RTI0_CLK_STATUS                                        (0x00000940U)
#define CSL_MSS_RCM_RTI1_CLK_STATUS                                        (0x00000944U)
#define CSL_MSS_RCM_RTI2_CLK_STATUS                                        (0x00000948U)
#define CSL_MSS_RCM_RTI3_CLK_STATUS                                        (0x0000094CU)
#define CSL_MSS_RCM_MCSPI0_CLK_STATUS                                      (0x00000980U)
#define CSL_MSS_RCM_MCSPI1_CLK_STATUS                                      (0x00000984U)
#define CSL_MSS_RCM_MCSPI2_CLK_STATUS                                      (0x00000988U)
#define CSL_MSS_RCM_MCSPI3_CLK_STATUS                                      (0x0000098CU)
#define CSL_MSS_RCM_WDT0_CLK_STATUS                                        (0x000009C0U)
#define CSL_MSS_RCM_WDT1_CLK_STATUS                                        (0x000009C4U)
#define CSL_MSS_RCM_ICSSM0_UART_CLK_STATUS                                 (0x000009E0U)
#define CSL_MSS_RCM_ICSSM1_UART_CLK_STATUS                                 (0x000009E4U)
#define CSL_MSS_RCM_OSPI0_CLK_STATUS                                       (0x000009F0U)
#define CSL_MSS_RCM_OSPI1_CLK_STATUS                                       (0x000009F4U)
#define CSL_MSS_RCM_CONTROLSS_PLL_CLK_STATUS                               (0x000009F8U)
#define CSL_MSS_RCM_CPTS_CLK_STATUS                                        (0x000009FCU)
#define CSL_MSS_RCM_GPMC_CLK_STATUS                                        (0x00000A00U)
#define CSL_MSS_RCM_MMC0_CLK_STATUS                                        (0x00000A04U)
#define CSL_MSS_RCM_MSS_ELM_CLK_STATUS                                     (0x00000A08U)
#define CSL_MSS_RCM_MII10_CLK_STATUS                                       (0x00000A0CU)
#define CSL_MSS_RCM_MII100_CLK_STATUS                                      (0x00000A10U)
#define CSL_MSS_RCM_RGMII_CLK_STATUS                                       (0x00000A14U)
#define CSL_MSS_RCM_MMC0_32K_CLK_STATUS                                    (0x00000A18U)
#define CSL_MSS_RCM_TEMPSENSE_32K_CLK_STATUS                               (0x00000A1CU)
#define CSL_MSS_RCM_CPSW_5_50_250_CLK_STATUS                               (0x00000A20U)
#define CSL_MSS_RCM_I2C_CLK_STATUS                                         (0x00000A24U)
#define CSL_MSS_RCM_LIN0_UART0_CLK_STATUS                                  (0x00000A64U)
#define CSL_MSS_RCM_LIN1_UART1_CLK_STATUS                                  (0x00000A68U)
#define CSL_MSS_RCM_LIN2_UART2_CLK_STATUS                                  (0x00000A6CU)
#define CSL_MSS_RCM_LIN3_UART3_CLK_STATUS                                  (0x00000A70U)
#define CSL_MSS_RCM_LIN4_UART4_CLK_STATUS                                  (0x00000A74U)
#define CSL_MSS_RCM_LIN5_UART5_CLK_STATUS                                  (0x00000A78U)
#define CSL_MSS_RCM_ICSSM0_CORE_CLK_STATUS                                 (0x00000AE4U)
#define CSL_MSS_RCM_ICSSM1_CORE_CLK_STATUS                                 (0x00000AE8U)
#define CSL_MSS_RCM_MCAN0_RST_CTRL                                         (0x00000B00U)
#define CSL_MSS_RCM_MCAN1_RST_CTRL                                         (0x00000B04U)
#define CSL_MSS_RCM_RTI0_RST_CTRL                                          (0x00000B40U)
#define CSL_MSS_RCM_RTI1_RST_CTRL                                          (0x00000B44U)
#define CSL_MSS_RCM_RTI2_RST_CTRL                                          (0x00000B48U)
#define CSL_MSS_RCM_RTI3_RST_CTRL                                          (0x00000B4CU)
#define CSL_MSS_RCM_MCSPI0_RST_CTRL                                        (0x00000B80U)
#define CSL_MSS_RCM_MCSPI1_RST_CTRL                                        (0x00000B84U)
#define CSL_MSS_RCM_MCSPI2_RST_CTRL                                        (0x00000B88U)
#define CSL_MSS_RCM_MCSPI3_RST_CTRL                                        (0x00000B8CU)
#define CSL_MSS_RCM_WDT0_RST_CTRL                                          (0x00000BC0U)
#define CSL_MSS_RCM_WDT1_RST_CTRL                                          (0x00000BC4U)
#define CSL_MSS_RCM_ICSSM0_RST_CTRL                                        (0x00000BE0U)
#define CSL_MSS_RCM_ICSSM1_RST_CTRL                                        (0x00000BE4U)
#define CSL_MSS_RCM_OSPI0_RST_CTRL                                         (0x00000BF0U)
#define CSL_MSS_RCM_OSPI1_RST_CTRL                                         (0x00000BF4U)
#define CSL_MSS_RCM_GPMC_RST_CTRL                                          (0x00000C00U)
#define CSL_MSS_RCM_MMC0_RST_CTRL                                          (0x00000C04U)
#define CSL_MSS_RCM_MSS_ELM_RST_CTRL                                       (0x00000C08U)
#define CSL_MSS_RCM_TEMPSENSE_32K_RST_CTRL                                 (0x00000C1CU)
#define CSL_MSS_RCM_CPSW_RST_CTRL                                          (0x00000C20U)
#define CSL_MSS_RCM_I2C0_RST_CTRL                                          (0x00000C24U)
#define CSL_MSS_RCM_I2C1_RST_CTRL                                          (0x00000C28U)
#define CSL_MSS_RCM_I2C2_RST_CTRL                                          (0x00000C2CU)
#define CSL_MSS_RCM_LIN0_RST_CTRL                                          (0x00000C64U)
#define CSL_MSS_RCM_LIN1_RST_CTRL                                          (0x00000C68U)
#define CSL_MSS_RCM_LIN2_RST_CTRL                                          (0x00000C6CU)
#define CSL_MSS_RCM_UART0_RST_CTRL                                         (0x00000CA4U)
#define CSL_MSS_RCM_UART1_RST_CTRL                                         (0x00000CA8U)
#define CSL_MSS_RCM_UART2_RST_CTRL                                         (0x00000CACU)
#define CSL_MSS_RCM_UART3_RST_CTRL                                         (0x00000CB0U)
#define CSL_MSS_RCM_UART4_RST_CTRL                                         (0x00000CB4U)
#define CSL_MSS_RCM_UART5_RST_CTRL                                         (0x00000CB8U)
#define CSL_MSS_RCM_R5SS0_POR_RST_CTRL                                    (0x00000D00U)
#define CSL_MSS_RCM_R5SS0_CORE0_GRST_CTRL                                       (0x00000D10U)
#define CSL_MSS_RCM_R5SS0_CORE1_GRST_CTRL                                       (0x00000D20U)
#define CSL_MSS_RCM_R5SS0_CORE0_LRST_CTRL                                         (0x00000D30U)
#define CSL_MSS_RCM_R5SS0_CORE1_LRST_CTRL                                         (0x00000D40U)
#define CSL_MSS_RCM_VIMA_RST_CTRL0                                         (0x00000D50U)
#define CSL_MSS_RCM_VIMB_RST_CTRL0                                         (0x00000D60U)
#define CSL_MSS_RCM_GPIO0_RST_CTRL                                         (0x00000D70U)
#define CSL_MSS_RCM_GPIO1_RST_CTRL                                         (0x00000D74U)
#define CSL_MSS_RCM_EDMA_RST_CTRL                                          (0x00000DD0U)
#define CSL_MSS_RCM_INFRA_RST_CTRL                                         (0x00000DD4U)
#define CSL_MSS_RCM_SPINLOCK_RST_CTRL                                      (0x00000DD8U)
#define CSL_MSS_RCM_USB_RST_CTRL                                           (0x00000DDCU)
#define CSL_MSS_RCM_CRC_RST_CTRL                                           (0x00000DE0U)
#define CSL_MSS_RCM_ESM_RST_CTRL                                           (0x00000DE4U)
#define CSL_MSS_RCM_DCCA_RST_CTRL                                          (0x00000DE8U)
#define CSL_MSS_RCM_DCCB_RST_CTRL                                          (0x00000DECU)
#define CSL_MSS_RCM_DCCC_RST_CTRL                                          (0x00000DF0U)
#define CSL_MSS_RCM_DCCD_RST_CTRL                                          (0x00000DF4U)
#define CSL_MSS_RCM_L2OCRAM_BANK0_PD_CTRL                                       (0x00000E00U)
#define CSL_MSS_RCM_L2OCRAM_BANK1_PD_CTRL                                       (0x00000E04U)
#define CSL_MSS_RCM_L2OCRAM_BANK2_PD_CTRL                                       (0x00000E08U)
#define CSL_MSS_RCM_L2OCRAM_BANK0_PD_STATUS                                     (0x00000E20U)
#define CSL_MSS_RCM_L2OCRAM_BANK1_PD_STATUS                                     (0x00000E24U)
#define CSL_MSS_RCM_L2OCRAM_BANK2_PD_STATUS                                     (0x00000E28U)
#define CSL_MSS_RCM_HW_REG0                                                (0x00000F00U)
#define CSL_MSS_RCM_HW_REG1                                                (0x00000F04U)
#define CSL_MSS_RCM_HW_REG2                                                (0x00000F08U)
#define CSL_MSS_RCM_HW_REG3                                                (0x00000F0CU)
#define CSL_MSS_RCM_HW_SPARE_RW0                                           (0x00000F40U)
#define CSL_MSS_RCM_HW_SPARE_RW1                                           (0x00000F44U)
#define CSL_MSS_RCM_HW_SPARE_RW2                                           (0x00000F48U)
#define CSL_MSS_RCM_HW_SPARE_RW3                                           (0x00000F4CU)
#define CSL_MSS_RCM_HW_SPARE_RO0                                           (0x00000F80U)
#define CSL_MSS_RCM_HW_SPARE_RO1                                           (0x00000F84U)
#define CSL_MSS_RCM_HW_SPARE_RO2                                           (0x00000F88U)
#define CSL_MSS_RCM_HW_SPARE_RO3                                           (0x00000F8CU)
#define CSL_MSS_RCM_HW_SPARE_WPH                                           (0x00000FC0U)
#define CSL_MSS_RCM_HW_SPARE_REC                                           (0x00000FC4U)
#define CSL_MSS_RCM_LOCK0_KICK0                                            (0x00001008U)
#define CSL_MSS_RCM_LOCK0_KICK1                                            (0x0000100CU)
#define CSL_MSS_RCM_INTR_RAW_STATUS                                        (0x00001010U)
#define CSL_MSS_RCM_INTR_ENABLED_STATUS_CLEAR                              (0x00001014U)
#define CSL_MSS_RCM_INTR_ENABLE                                            (0x00001018U)
#define CSL_MSS_RCM_INTR_ENABLE_CLEAR                                      (0x0000101CU)
#define CSL_MSS_RCM_EOI                                                    (0x00001020U)
#define CSL_MSS_RCM_FAULT_ADDRESS                                          (0x00001024U)
#define CSL_MSS_RCM_FAULT_TYPE_STATUS                                      (0x00001028U)
#define CSL_MSS_RCM_FAULT_ATTR_STATUS                                      (0x0000102CU)
#define CSL_MSS_RCM_FAULT_CLEAR                                            (0x00001030U)

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

#define CSL_MSS_RCM_R5SS0_RST_STATUS_CAUSE_MASK               (0x000007FFU)
#define CSL_MSS_RCM_R5SS0_RST_STATUS_CAUSE_SHIFT              (0x00000000U)
#define CSL_MSS_RCM_R5SS0_RST_STATUS_CAUSE_RESETVAL           (0x00000003U)
#define CSL_MSS_RCM_R5SS0_RST_STATUS_CAUSE_MAX                (0x000007FFU)

#define CSL_MSS_RCM_R5SS0_RST_STATUS_RESETVAL                                  (0x00000003U)

/* R5SS0_RST_CAUSE_CLR */

#define CSL_MSS_RCM_R5SS0_RST_CAUSE_CLR_CLR_MASK           (0x00000007U)
#define CSL_MSS_RCM_R5SS0_RST_CAUSE_CLR_CLR_SHIFT          (0x00000000U)
#define CSL_MSS_RCM_R5SS0_RST_CAUSE_CLR_CLR_RESETVAL       (0x00000000U)
#define CSL_MSS_RCM_R5SS0_RST_CAUSE_CLR_CLR_MAX            (0x00000007U)

#define CSL_MSS_RCM_R5SS0_RST_CAUSE_CLR_RESETVAL                               (0x00000000U)

/* SYSRST_BY_DBG_RST0 */

#define CSL_MSS_RCM_SYSRST_BY_DBG_RST0_R5A_MASK             (0x00000007U)
#define CSL_MSS_RCM_SYSRST_BY_DBG_RST0_R5A_SHIFT            (0x00000000U)
#define CSL_MSS_RCM_SYSRST_BY_DBG_RST0_R5A_RESETVAL         (0x00000000U)
#define CSL_MSS_RCM_SYSRST_BY_DBG_RST0_R5A_MAX              (0x00000007U)

#define CSL_MSS_RCM_SYSRST_BY_DBG_RST0_R5B_MASK             (0x00070000U)
#define CSL_MSS_RCM_SYSRST_BY_DBG_RST0_R5B_SHIFT            (0x00000010U)
#define CSL_MSS_RCM_SYSRST_BY_DBG_RST0_R5B_RESETVAL         (0x00000000U)
#define CSL_MSS_RCM_SYSRST_BY_DBG_RST0_R5B_MAX              (0x00000007U)

#define CSL_MSS_RCM_SYSRST_BY_DBG_RST0_RESETVAL                                (0x00000000U)

/* RST_ASSERDLY0 */

#define CSL_MSS_RCM_RST_ASSERDLY0_COMMON_MASK                    (0x000000FFU)
#define CSL_MSS_RCM_RST_ASSERDLY0_COMMON_SHIFT                   (0x00000000U)
#define CSL_MSS_RCM_RST_ASSERDLY0_COMMON_RESETVAL                (0x0000000FU)
#define CSL_MSS_RCM_RST_ASSERDLY0_COMMON_MAX                     (0x000000FFU)

#define CSL_MSS_RCM_RST_ASSERDLY0_RESETVAL                                     (0x0000000FU)

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

/* MSS_MCAN0_CLK_SRC_SEL */

#define CSL_MSS_RCM_MCAN0_CLK_SRC_SEL_CLKSRCSEL_MASK (0x00000FFFU)
#define CSL_MSS_RCM_MCAN0_CLK_SRC_SEL_CLKSRCSEL_SHIFT (0x00000000U)
#define CSL_MSS_RCM_MCAN0_CLK_SRC_SEL_CLKSRCSEL_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_MCAN0_CLK_SRC_SEL_CLKSRCSEL_MAX  (0x00000FFFU)

#define CSL_MSS_RCM_MCAN0_CLK_SRC_SEL_RESETVAL                             (0x00000000U)

/* MSS_MCAN1_CLK_SRC_SEL */

#define CSL_MSS_RCM_MCAN1_CLK_SRC_SEL_CLKSRCSEL_MASK (0x00000FFFU)
#define CSL_MSS_RCM_MCAN1_CLK_SRC_SEL_CLKSRCSEL_SHIFT (0x00000000U)
#define CSL_MSS_RCM_MCAN1_CLK_SRC_SEL_CLKSRCSEL_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_MCAN1_CLK_SRC_SEL_CLKSRCSEL_MAX  (0x00000FFFU)

#define CSL_MSS_RCM_MCAN1_CLK_SRC_SEL_RESETVAL                             (0x00000000U)

/* MSS_RTI0_CLK_SRC_SEL */

#define CSL_MSS_RCM_RTI0_CLK_SRC_SEL_CLKSRCSEL_MASK   (0x00000FFFU)
#define CSL_MSS_RCM_RTI0_CLK_SRC_SEL_CLKSRCSEL_SHIFT  (0x00000000U)
#define CSL_MSS_RCM_RTI0_CLK_SRC_SEL_CLKSRCSEL_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_RTI0_CLK_SRC_SEL_CLKSRCSEL_MAX    (0x00000FFFU)

#define CSL_MSS_RCM_RTI0_CLK_SRC_SEL_RESETVAL                              (0x00000000U)

/* MSS_RTI1_CLK_SRC_SEL */

#define CSL_MSS_RCM_RTI1_CLK_SRC_SEL_CLKSRCSEL_MASK   (0x00000FFFU)
#define CSL_MSS_RCM_RTI1_CLK_SRC_SEL_CLKSRCSEL_SHIFT  (0x00000000U)
#define CSL_MSS_RCM_RTI1_CLK_SRC_SEL_CLKSRCSEL_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_RTI1_CLK_SRC_SEL_CLKSRCSEL_MAX    (0x00000FFFU)

#define CSL_MSS_RCM_RTI1_CLK_SRC_SEL_RESETVAL                              (0x00000000U)

/* MSS_RTI2_CLK_SRC_SEL */

#define CSL_MSS_RCM_RTI2_CLK_SRC_SEL_CLKSRCSEL_MASK   (0x00000FFFU)
#define CSL_MSS_RCM_RTI2_CLK_SRC_SEL_CLKSRCSEL_SHIFT  (0x00000000U)
#define CSL_MSS_RCM_RTI2_CLK_SRC_SEL_CLKSRCSEL_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_RTI2_CLK_SRC_SEL_CLKSRCSEL_MAX    (0x00000FFFU)

#define CSL_MSS_RCM_RTI2_CLK_SRC_SEL_RESETVAL                              (0x00000000U)

/* MSS_RTI3_CLK_SRC_SEL */

#define CSL_MSS_RCM_RTI3_CLK_SRC_SEL_CLKSRCSEL_MASK   (0x00000FFFU)
#define CSL_MSS_RCM_RTI3_CLK_SRC_SEL_CLKSRCSEL_SHIFT  (0x00000000U)
#define CSL_MSS_RCM_RTI3_CLK_SRC_SEL_CLKSRCSEL_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_RTI3_CLK_SRC_SEL_CLKSRCSEL_MAX    (0x00000FFFU)

#define CSL_MSS_RCM_RTI3_CLK_SRC_SEL_RESETVAL                              (0x00000000U)

/* MSS_MCSPI0_CLK_SRC_SEL */

#define CSL_MSS_RCM_MCSPI0_CLK_SRC_SEL_CLKSRCSEL_MASK   (0x00000FFFU)
#define CSL_MSS_RCM_MCSPI0_CLK_SRC_SEL_CLKSRCSEL_SHIFT  (0x00000000U)
#define CSL_MSS_RCM_MCSPI0_CLK_SRC_SEL_CLKSRCSEL_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_MCSPI0_CLK_SRC_SEL_CLKSRCSEL_MAX    (0x00000FFFU)

#define CSL_MSS_RCM_MCSPI0_CLK_SRC_SEL_RESETVAL                              (0x00000000U)

/* MSS_MCSPI1_CLK_SRC_SEL */

#define CSL_MSS_RCM_MCSPI1_CLK_SRC_SEL_CLKSRCSEL_MASK   (0x00000FFFU)
#define CSL_MSS_RCM_MCSPI1_CLK_SRC_SEL_CLKSRCSEL_SHIFT  (0x00000000U)
#define CSL_MSS_RCM_MCSPI1_CLK_SRC_SEL_CLKSRCSEL_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_MCSPI1_CLK_SRC_SEL_CLKSRCSEL_MAX    (0x00000FFFU)

#define CSL_MSS_RCM_MCSPI1_CLK_SRC_SEL_RESETVAL                              (0x00000000U)

/* MSS_MCSPI2_CLK_SRC_SEL */

#define CSL_MSS_RCM_MCSPI2_CLK_SRC_SEL_CLKSRCSEL_MASK   (0x00000FFFU)
#define CSL_MSS_RCM_MCSPI2_CLK_SRC_SEL_CLKSRCSEL_SHIFT  (0x00000000U)
#define CSL_MSS_RCM_MCSPI2_CLK_SRC_SEL_CLKSRCSEL_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_MCSPI2_CLK_SRC_SEL_CLKSRCSEL_MAX    (0x00000FFFU)

#define CSL_MSS_RCM_MCSPI2_CLK_SRC_SEL_RESETVAL                              (0x00000000U)

/* MSS_MCSPI3_CLK_SRC_SEL */

#define CSL_MSS_RCM_MCSPI3_CLK_SRC_SEL_CLKSRCSEL_MASK   (0x00000FFFU)
#define CSL_MSS_RCM_MCSPI3_CLK_SRC_SEL_CLKSRCSEL_SHIFT  (0x00000000U)
#define CSL_MSS_RCM_MCSPI3_CLK_SRC_SEL_CLKSRCSEL_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_MCSPI3_CLK_SRC_SEL_CLKSRCSEL_MAX    (0x00000FFFU)

#define CSL_MSS_RCM_MCSPI3_CLK_SRC_SEL_RESETVAL                              (0x00000000U)

/* MSS_WDT0_CLK_SRC_SEL */

#define CSL_MSS_RCM_WDT0_CLK_SRC_SEL_CLKSRCSEL_MASK   (0x00000FFFU)
#define CSL_MSS_RCM_WDT0_CLK_SRC_SEL_CLKSRCSEL_SHIFT  (0x00000000U)
#define CSL_MSS_RCM_WDT0_CLK_SRC_SEL_CLKSRCSEL_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_WDT0_CLK_SRC_SEL_CLKSRCSEL_MAX    (0x00000FFFU)

#define CSL_MSS_RCM_WDT0_CLK_SRC_SEL_RESETVAL                              (0x00000000U)

/* MSS_WDT1_CLK_SRC_SEL */

#define CSL_MSS_RCM_WDT1_CLK_SRC_SEL_CLKSRCSEL_MASK   (0x00000FFFU)
#define CSL_MSS_RCM_WDT1_CLK_SRC_SEL_CLKSRCSEL_SHIFT  (0x00000000U)
#define CSL_MSS_RCM_WDT1_CLK_SRC_SEL_CLKSRCSEL_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_WDT1_CLK_SRC_SEL_CLKSRCSEL_MAX    (0x00000FFFU)

#define CSL_MSS_RCM_WDT1_CLK_SRC_SEL_RESETVAL                              (0x00000000U)

/* ICSSM0_UART_CLK_SRC_SEL */

#define CSL_MSS_RCM_ICSSM0_UART_CLK_SRC_SEL_CLKSRCSEL_MASK (0x00000FFFU)
#define CSL_MSS_RCM_ICSSM0_UART_CLK_SRC_SEL_CLKSRCSEL_SHIFT (0x00000000U)
#define CSL_MSS_RCM_ICSSM0_UART_CLK_SRC_SEL_CLKSRCSEL_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_ICSSM0_UART_CLK_SRC_SEL_CLKSRCSEL_MAX (0x00000FFFU)

#define CSL_MSS_RCM_ICSSM0_UART_CLK_SRC_SEL_RESETVAL                           (0x00000000U)

/* ICSSM1_UART_CLK_SRC_SEL */

#define CSL_MSS_RCM_ICSSM1_UART_CLK_SRC_SEL_CLKSRCSEL_MASK (0x00000FFFU)
#define CSL_MSS_RCM_ICSSM1_UART_CLK_SRC_SEL_CLKSRCSEL_SHIFT (0x00000000U)
#define CSL_MSS_RCM_ICSSM1_UART_CLK_SRC_SEL_CLKSRCSEL_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_ICSSM1_UART_CLK_SRC_SEL_CLKSRCSEL_MAX (0x00000FFFU)

#define CSL_MSS_RCM_ICSSM1_UART_CLK_SRC_SEL_RESETVAL                           (0x00000000U)

/* OSPI0_CLK_SRC_SEL */

#define CSL_MSS_RCM_OSPI0_CLK_SRC_SEL_CLKSRCSEL_MASK (0x00000FFFU)
#define CSL_MSS_RCM_OSPI0_CLK_SRC_SEL_CLKSRCSEL_SHIFT (0x00000000U)
#define CSL_MSS_RCM_OSPI0_CLK_SRC_SEL_CLKSRCSEL_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_OSPI0_CLK_SRC_SEL_CLKSRCSEL_MAX  (0x00000FFFU)



/* OSPI1_CLK_SRC_SEL */
#define CSL_MSS_RCM_OSPI1_CLK_SRC_SEL_CLKSRCSEL_MASK                        (0x00000FFFU)
#define CSL_MSS_RCM_OSPI1_CLK_SRC_SEL_CLKSRCSEL_SHIFT                       (0x00000000U)
#define CSL_MSS_RCM_OSPI1_CLK_SRC_SEL_CLKSRCSEL_RESETVAL                    (0x00000000U)
#define CSL_MSS_RCM_OSPI1_CLK_SRC_SEL_CLKSRCSEL_MAX                         (0x00000FFFU)

/* CONTROLSS_PLL_CLK_SRC_SEL */

#define CSL_MSS_RCM_CONTROLSS_PLL_CLK_SRC_SEL_CLKSRCSEL_MASK (0x00000FFFU)
#define CSL_MSS_RCM_CONTROLSS_PLL_CLK_SRC_SEL_CLKSRCSEL_SHIFT (0x00000000U)
#define CSL_MSS_RCM_CONTROLSS_PLL_CLK_SRC_SEL_CLKSRCSEL_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_CONTROLSS_PLL_CLK_SRC_SEL_CLKSRCSEL_MAX (0x00000FFFU)

#define CSL_MSS_RCM_CONTROLSS_PLL_CLK_SRC_SEL_RESETVAL                         (0x00000000U)

/* MSS_CPTS_CLK_SRC_SEL */

#define CSL_MSS_RCM_CPTS_CLK_SRC_SEL_CLKSRCSEL_MASK   (0x00000FFFU)
#define CSL_MSS_RCM_CPTS_CLK_SRC_SEL_CLKSRCSEL_SHIFT  (0x00000000U)
#define CSL_MSS_RCM_CPTS_CLK_SRC_SEL_CLKSRCSEL_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_CPTS_CLK_SRC_SEL_CLKSRCSEL_MAX    (0x00000FFFU)

#define CSL_MSS_RCM_CPTS_CLK_SRC_SEL_RESETVAL                              (0x00000000U)

/* GPMC_CLK_SRC_SEL */

#define CSL_MSS_RCM_GPMC_CLK_SRC_SEL_CLKSRCSEL_MASK           (0x00000FFFU)
#define CSL_MSS_RCM_GPMC_CLK_SRC_SEL_CLKSRCSEL_SHIFT          (0x00000000U)
#define CSL_MSS_RCM_GPMC_CLK_SRC_SEL_CLKSRCSEL_RESETVAL       (0x00000000U)
#define CSL_MSS_RCM_GPMC_CLK_SRC_SEL_CLKSRCSEL_MAX            (0x00000FFFU)

#define CSL_MSS_RCM_GPMC_CLK_SRC_SEL_RESETVAL                                  (0x00000000U)

/* MSS_MMC0_CLK_SRC_SEL */

#define CSL_MSS_RCM_MMC0_CLK_SRC_SEL_CLKSRCSEL_MASK (0x00000FFFU)
#define CSL_MSS_RCM_MMC0_CLK_SRC_SEL_CLKSRCSEL_SHIFT (0x00000000U)
#define CSL_MSS_RCM_MMC0_CLK_SRC_SEL_CLKSRCSEL_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_MMC0_CLK_SRC_SEL_CLKSRCSEL_MAX  (0x00000FFFU)

#define CSL_MSS_RCM_MMC0_CLK_SRC_SEL_RESETVAL                             (0x00000000U)

/* CPSW_5_50_250_CLK_MUX_CTRL */

#define CSL_MSS_RCM_CPSW_5_50_250_CLK_MUX_CTRL_CLKSRCSEL_MASK (0x00000FFFU)
#define CSL_MSS_RCM_CPSW_5_50_250_CLK_MUX_CTRL_CLKSRCSEL_SHIFT (0x00000000U)
#define CSL_MSS_RCM_CPSW_5_50_250_CLK_MUX_CTRL_CLKSRCSEL_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_CPSW_5_50_250_CLK_MUX_CTRL_CLKSRCSEL_MAX (0x00000FFFU)

#define CSL_MSS_RCM_CPSW_5_50_250_CLK_MUX_CTRL_RESETVAL                        (0x00000000U)

/* MSS_I2C_CLK_SRC_SEL */

#define CSL_MSS_RCM_I2C_CLK_SRC_SEL_CLKSRCSEL_MASK     (0x00000FFFU)
#define CSL_MSS_RCM_I2C_CLK_SRC_SEL_CLKSRCSEL_SHIFT    (0x00000000U)
#define CSL_MSS_RCM_I2C_CLK_SRC_SEL_CLKSRCSEL_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_I2C_CLK_SRC_SEL_CLKSRCSEL_MAX      (0x00000FFFU)

#define CSL_MSS_RCM_I2C_CLK_SRC_SEL_RESETVAL                               (0x00000000U)

/* LIN0_UART0_CLK_SRC_SEL */

#define CSL_MSS_RCM_LIN0_UART0_CLK_SRC_SEL_CLKSRCSEL_MASK (0x00000FFFU)
#define CSL_MSS_RCM_LIN0_UART0_CLK_SRC_SEL_CLKSRCSEL_SHIFT (0x00000000U)
#define CSL_MSS_RCM_LIN0_UART0_CLK_SRC_SEL_CLKSRCSEL_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_LIN0_UART0_CLK_SRC_SEL_CLKSRCSEL_MAX (0x00000FFFU)

#define CSL_MSS_RCM_LIN0_UART0_CLK_SRC_SEL_RESETVAL                        (0x00000000U)

/* LIN1_UART1_CLK_SRC_SEL */

#define CSL_MSS_RCM_LIN1_UART1_CLK_SRC_SEL_CLKSRCSEL_MASK (0x00000FFFU)
#define CSL_MSS_RCM_LIN1_UART1_CLK_SRC_SEL_CLKSRCSEL_SHIFT (0x00000000U)
#define CSL_MSS_RCM_LIN1_UART1_CLK_SRC_SEL_CLKSRCSEL_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_LIN1_UART1_CLK_SRC_SEL_CLKSRCSEL_MAX (0x00000FFFU)

#define CSL_MSS_RCM_LIN1_UART1_CLK_SRC_SEL_RESETVAL                        (0x00000000U)

/* LIN2_UART2_CLK_SRC_SEL */

#define CSL_MSS_RCM_LIN2_UART2_CLK_SRC_SEL_CLKSRCSEL_MASK (0x00000FFFU)
#define CSL_MSS_RCM_LIN2_UART2_CLK_SRC_SEL_CLKSRCSEL_SHIFT (0x00000000U)
#define CSL_MSS_RCM_LIN2_UART2_CLK_SRC_SEL_CLKSRCSEL_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_LIN2_UART2_CLK_SRC_SEL_CLKSRCSEL_MAX (0x00000FFFU)

#define CSL_MSS_RCM_LIN2_UART2_CLK_SRC_SEL_RESETVAL                        (0x00000000U)

/* LIN3_UART3_CLK_SRC_SEL */

#define CSL_MSS_RCM_LIN3_UART3_CLK_SRC_SEL_CLKSRCSEL_MASK (0x00000FFFU)
#define CSL_MSS_RCM_LIN3_UART3_CLK_SRC_SEL_CLKSRCSEL_SHIFT (0x00000000U)
#define CSL_MSS_RCM_LIN3_UART3_CLK_SRC_SEL_CLKSRCSEL_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_LIN3_UART3_CLK_SRC_SEL_CLKSRCSEL_MAX (0x00000FFFU)

#define CSL_MSS_RCM_LIN3_UART3_CLK_SRC_SEL_RESETVAL                        (0x00000000U)


/* LIN4_UART4_CLK_SRC_SEL */
#define CSL_MSS_RCM_LIN4_UART4_CLK_SRC_SEL_CLKSRCSEL_MASK                   (0x00000FFFU)
#define CSL_MSS_RCM_LIN4_UART4_CLK_SRC_SEL_CLKSRCSEL_SHIFT                  (0x00000000U)
#define CSL_MSS_RCM_LIN4_UART4_CLK_SRC_SEL_CLKSRCSEL_RESETVAL               (0x00000000U)
#define CSL_MSS_RCM_LIN4_UART4_CLK_SRC_SEL_CLKSRCSEL_MAX                    (0x00000FFFU)

#define CSL_MSS_RCM_LIN4_UART4_CLK_SRC_SEL_RESETVAL                        (0x00000000U)


/* LIN5_UART5_CLK_SRC_SEL */
#define CSL_MSS_RCM_LIN5_UART5_CLK_SRC_SEL_CLKSRCSEL_MASK                   (0x00000FFFU)
#define CSL_MSS_RCM_LIN5_UART5_CLK_SRC_SEL_CLKSRCSEL_SHIFT                  (0x00000000U)
#define CSL_MSS_RCM_LIN5_UART5_CLK_SRC_SEL_CLKSRCSEL_RESETVAL               (0x00000000U)
#define CSL_MSS_RCM_LIN5_UART5_CLK_SRC_SEL_CLKSRCSEL_MAX                    (0x00000FFFU)

#define CSL_MSS_RCM_LIN5_UART5_CLK_SRC_SEL_RESETVAL                        (0x00000000U)



/* ICSSM0_CORE_CLK_SRC_SEL */
#define CSL_MSS_RCM_ICSSM0_CORE_CLK_SRC_SEL_CLKSRCSEL_MASK            (0x00000FFFU)
#define CSL_MSS_RCM_ICSSM0_CORE_CLK_SRC_SEL_CLKSRCSEL_SHIFT           (0x00000000U)
#define CSL_MSS_RCM_ICSSM0_CORE_CLK_SRC_SEL_CLKSRCSEL_RESETVAL        (0x00000000U)
#define CSL_MSS_RCM_ICSSM0_CORE_CLK_SRC_SEL_CLKSRCSEL_MAX             (0x00000FFFU)



/* ICSSM1_CORE_CLK_SRC_SEL */
#define CSL_MSS_RCM_ICSSM1_CORE_CLK_SRC_SEL_CLKSRCSEL_MASK            (0x00000FFFU)
#define CSL_MSS_RCM_ICSSM1_CORE_CLK_SRC_SEL_CLKSRCSEL_SHIFT           (0x00000000U)
#define CSL_MSS_RCM_ICSSM1_CORE_CLK_SRC_SEL_CLKSRCSEL_RESETVAL        (0x00000000U)
#define CSL_MSS_RCM_ICSSM1_CORE_CLK_SRC_SEL_CLKSRCSEL_MAX             (0x00000FFFU)



/* MSS_MCAN0_CLK_DIV_VAL */

#define CSL_MSS_RCM_MCAN0_CLK_DIV_VAL_CLKDIVR_MASK   (0x00000FFFU)
#define CSL_MSS_RCM_MCAN0_CLK_DIV_VAL_CLKDIVR_SHIFT  (0x00000000U)
#define CSL_MSS_RCM_MCAN0_CLK_DIV_VAL_CLKDIVR_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_MCAN0_CLK_DIV_VAL_CLKDIVR_MAX    (0x00000FFFU)

#define CSL_MSS_RCM_MCAN0_CLK_DIV_VAL_RESETVAL                             (0x00000000U)

/* MSS_MCAN1_CLK_DIV_VAL */

#define CSL_MSS_RCM_MCAN1_CLK_DIV_VAL_CLKDIVR_MASK   (0x00000FFFU)
#define CSL_MSS_RCM_MCAN1_CLK_DIV_VAL_CLKDIVR_SHIFT  (0x00000000U)
#define CSL_MSS_RCM_MCAN1_CLK_DIV_VAL_CLKDIVR_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_MCAN1_CLK_DIV_VAL_CLKDIVR_MAX    (0x00000FFFU)

#define CSL_MSS_RCM_MCAN1_CLK_DIV_VAL_RESETVAL                             (0x00000000U)

/* MSS_RTI0_CLK_DIV_VAL */

#define CSL_MSS_RCM_RTI0_CLK_DIV_VAL_CLKDIVR_MASK     (0x00000FFFU)
#define CSL_MSS_RCM_RTI0_CLK_DIV_VAL_CLKDIVR_SHIFT    (0x00000000U)
#define CSL_MSS_RCM_RTI0_CLK_DIV_VAL_CLKDIVR_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_RTI0_CLK_DIV_VAL_CLKDIVR_MAX      (0x00000FFFU)

#define CSL_MSS_RCM_RTI0_CLK_DIV_VAL_RESETVAL                              (0x00000000U)

/* MSS_RTI1_CLK_DIV_VAL */

#define CSL_MSS_RCM_RTI1_CLK_DIV_VAL_CLKDIVR_MASK     (0x00000FFFU)
#define CSL_MSS_RCM_RTI1_CLK_DIV_VAL_CLKDIVR_SHIFT    (0x00000000U)
#define CSL_MSS_RCM_RTI1_CLK_DIV_VAL_CLKDIVR_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_RTI1_CLK_DIV_VAL_CLKDIVR_MAX      (0x00000FFFU)

#define CSL_MSS_RCM_RTI1_CLK_DIV_VAL_RESETVAL                              (0x00000000U)

/* MSS_RTI2_CLK_DIV_VAL */

#define CSL_MSS_RCM_RTI2_CLK_DIV_VAL_CLKDIVR_MASK     (0x00000FFFU)
#define CSL_MSS_RCM_RTI2_CLK_DIV_VAL_CLKDIVR_SHIFT    (0x00000000U)
#define CSL_MSS_RCM_RTI2_CLK_DIV_VAL_CLKDIVR_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_RTI2_CLK_DIV_VAL_CLKDIVR_MAX      (0x00000FFFU)

#define CSL_MSS_RCM_RTI2_CLK_DIV_VAL_RESETVAL                              (0x00000000U)

/* MSS_RTI3_CLK_DIV_VAL */

#define CSL_MSS_RCM_RTI3_CLK_DIV_VAL_CLKDIVR_MASK     (0x00000FFFU)
#define CSL_MSS_RCM_RTI3_CLK_DIV_VAL_CLKDIVR_SHIFT    (0x00000000U)
#define CSL_MSS_RCM_RTI3_CLK_DIV_VAL_CLKDIVR_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_RTI3_CLK_DIV_VAL_CLKDIVR_MAX      (0x00000FFFU)

#define CSL_MSS_RCM_RTI3_CLK_DIV_VAL_RESETVAL                              (0x00000000U)

/* MSS_MCSPI0_CLK_DIV_VAL */

#define CSL_MSS_RCM_MCSPI0_CLK_DIV_VAL_CLKDIVR_MASK     (0x00000FFFU)
#define CSL_MSS_RCM_MCSPI0_CLK_DIV_VAL_CLKDIVR_SHIFT    (0x00000000U)
#define CSL_MSS_RCM_MCSPI0_CLK_DIV_VAL_CLKDIVR_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_MCSPI0_CLK_DIV_VAL_CLKDIVR_MAX      (0x00000FFFU)

#define CSL_MSS_RCM_MCSPI0_CLK_DIV_VAL_RESETVAL                              (0x00000000U)

/* MSS_MCSPI1_CLK_DIV_VAL */

#define CSL_MSS_RCM_MCSPI1_CLK_DIV_VAL_CLKDIVR_MASK     (0x00000FFFU)
#define CSL_MSS_RCM_MCSPI1_CLK_DIV_VAL_CLKDIVR_SHIFT    (0x00000000U)
#define CSL_MSS_RCM_MCSPI1_CLK_DIV_VAL_CLKDIVR_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_MCSPI1_CLK_DIV_VAL_CLKDIVR_MAX      (0x00000FFFU)

#define CSL_MSS_RCM_MCSPI1_CLK_DIV_VAL_RESETVAL                              (0x00000000U)

/* MSS_MCSPI2_CLK_DIV_VAL */

#define CSL_MSS_RCM_MCSPI2_CLK_DIV_VAL_CLKDIVR_MASK     (0x00000FFFU)
#define CSL_MSS_RCM_MCSPI2_CLK_DIV_VAL_CLKDIVR_SHIFT    (0x00000000U)
#define CSL_MSS_RCM_MCSPI2_CLK_DIV_VAL_CLKDIVR_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_MCSPI2_CLK_DIV_VAL_CLKDIVR_MAX      (0x00000FFFU)

#define CSL_MSS_RCM_MCSPI2_CLK_DIV_VAL_RESETVAL                              (0x00000000U)

/* MSS_MCSPI3_CLK_DIV_VAL */

#define CSL_MSS_RCM_MCSPI3_CLK_DIV_VAL_CLKDIVR_MASK     (0x00000FFFU)
#define CSL_MSS_RCM_MCSPI3_CLK_DIV_VAL_CLKDIVR_SHIFT    (0x00000000U)
#define CSL_MSS_RCM_MCSPI3_CLK_DIV_VAL_CLKDIVR_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_MCSPI3_CLK_DIV_VAL_CLKDIVR_MAX      (0x00000FFFU)

#define CSL_MSS_RCM_MCSPI3_CLK_DIV_VAL_RESETVAL                              (0x00000000U)

/* MSS_WDT0_CLK_DIV_VAL */

#define CSL_MSS_RCM_WDT0_CLK_DIV_VAL_CLKDIVR_MASK     (0x00000FFFU)
#define CSL_MSS_RCM_WDT0_CLK_DIV_VAL_CLKDIVR_SHIFT    (0x00000000U)
#define CSL_MSS_RCM_WDT0_CLK_DIV_VAL_CLKDIVR_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_WDT0_CLK_DIV_VAL_CLKDIVR_MAX      (0x00000FFFU)

#define CSL_MSS_RCM_WDT0_CLK_DIV_VAL_RESETVAL                              (0x00000000U)

/* MSS_WDT1_CLK_DIV_VAL */

#define CSL_MSS_RCM_WDT1_CLK_DIV_VAL_CLKDIVR_MASK     (0x00000FFFU)
#define CSL_MSS_RCM_WDT1_CLK_DIV_VAL_CLKDIVR_SHIFT    (0x00000000U)
#define CSL_MSS_RCM_WDT1_CLK_DIV_VAL_CLKDIVR_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_WDT1_CLK_DIV_VAL_CLKDIVR_MAX      (0x00000FFFU)

#define CSL_MSS_RCM_WDT1_CLK_DIV_VAL_RESETVAL                              (0x00000000U)

/* ICSSM0_UART_CLK_DIV_VAL */

#define CSL_MSS_RCM_ICSSM0_UART_CLK_DIV_VAL_CLKDIVR_MASK (0x00000FFFU)
#define CSL_MSS_RCM_ICSSM0_UART_CLK_DIV_VAL_CLKDIVR_SHIFT (0x00000000U)
#define CSL_MSS_RCM_ICSSM0_UART_CLK_DIV_VAL_CLKDIVR_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_ICSSM0_UART_CLK_DIV_VAL_CLKDIVR_MAX (0x00000FFFU)

#define CSL_MSS_RCM_ICSSM0_UART_CLK_DIV_VAL_RESETVAL                           (0x00000000U)

/* ICSSM1_UART_CLK_DIV_VAL */

#define CSL_MSS_RCM_ICSSM1_UART_CLK_DIV_VAL_CLKDIVR_MASK (0x00000FFFU)
#define CSL_MSS_RCM_ICSSM1_UART_CLK_DIV_VAL_CLKDIVR_SHIFT (0x00000000U)
#define CSL_MSS_RCM_ICSSM1_UART_CLK_DIV_VAL_CLKDIVR_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_ICSSM1_UART_CLK_DIV_VAL_CLKDIVR_MAX (0x00000FFFU)

#define CSL_MSS_RCM_ICSSM1_UART_CLK_DIV_VAL_RESETVAL                           (0x00000000U)

/* OSPI0_CLK_DIV_VAL */

#define CSL_MSS_RCM_OSPI0_CLK_DIV_VAL_CLKDIVR_MASK   (0x00000FFFU)
#define CSL_MSS_RCM_OSPI0_CLK_DIV_VAL_CLKDIVR_SHIFT  (0x00000000U)
#define CSL_MSS_RCM_OSPI0_CLK_DIV_VAL_CLKDIVR_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_OSPI0_CLK_DIV_VAL_CLKDIVR_MAX    (0x00000FFFU)



/* OSPI1_CLK_DIV_VAL */
#define CSL_MSS_RCM_OSPI1_CLK_DIV_VAL_CLKDIVR_MASK                          (0x00000FFFU)
#define CSL_MSS_RCM_OSPI1_CLK_DIV_VAL_CLKDIVR_SHIFT                         (0x00000000U)
#define CSL_MSS_RCM_OSPI1_CLK_DIV_VAL_CLKDIVR_RESETVAL                      (0x00000000U)
#define CSL_MSS_RCM_OSPI1_CLK_DIV_VAL_CLKDIVR_MAX                           (0x00000FFFU)

/* CONTROLSS_PLL_CLK_DIV_VAL */

#define CSL_MSS_RCM_CONTROLSS_PLL_CLK_DIV_VAL_CLKDIVR_MASK (0x00000FFFU)
#define CSL_MSS_RCM_CONTROLSS_PLL_CLK_DIV_VAL_CLKDIVR_SHIFT (0x00000000U)
#define CSL_MSS_RCM_CONTROLSS_PLL_CLK_DIV_VAL_CLKDIVR_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_CONTROLSS_PLL_CLK_DIV_VAL_CLKDIVR_MAX (0x00000FFFU)

#define CSL_MSS_RCM_CONTROLSS_PLL_CLK_DIV_VAL_RESETVAL                         (0x00000000U)

/* MSS_CPTS_CLK_DIV_VAL */

#define CSL_MSS_RCM_CPTS_CLK_DIV_VAL_CLKDIVR_MASK     (0x00000FFFU)
#define CSL_MSS_RCM_CPTS_CLK_DIV_VAL_CLKDIVR_SHIFT    (0x00000000U)
#define CSL_MSS_RCM_CPTS_CLK_DIV_VAL_CLKDIVR_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_CPTS_CLK_DIV_VAL_CLKDIVR_MAX      (0x00000FFFU)

#define CSL_MSS_RCM_CPTS_CLK_DIV_VAL_RESETVAL                              (0x00000000U)

/* GPMC_CLK_DIV_VAL */

#define CSL_MSS_RCM_GPMC_CLK_DIV_VAL_CLKDIVR_MASK             (0x00000FFFU)
#define CSL_MSS_RCM_GPMC_CLK_DIV_VAL_CLKDIVR_SHIFT            (0x00000000U)
#define CSL_MSS_RCM_GPMC_CLK_DIV_VAL_CLKDIVR_RESETVAL         (0x00000000U)
#define CSL_MSS_RCM_GPMC_CLK_DIV_VAL_CLKDIVR_MAX              (0x00000FFFU)

#define CSL_MSS_RCM_GPMC_CLK_DIV_VAL_RESETVAL                                  (0x00000000U)

/* MSS_MMC0_CLK_DIV_VAL */

#define CSL_MSS_RCM_MMC0_CLK_DIV_VAL_CLKDIVR_MASK   (0x00000FFFU)
#define CSL_MSS_RCM_MMC0_CLK_DIV_VAL_CLKDIVR_SHIFT  (0x00000000U)
#define CSL_MSS_RCM_MMC0_CLK_DIV_VAL_CLKDIVR_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_MMC0_CLK_DIV_VAL_CLKDIVR_MAX    (0x00000FFFU)

#define CSL_MSS_RCM_MMC0_CLK_DIV_VAL_RESETVAL                             (0x00000000U)

/* MSS_ELM_CLK_DIV_VAL */

#define CSL_MSS_RCM_MSS_ELM_CLK_DIV_VAL_CLKDIVR_MASK       (0x00000FFFU)
#define CSL_MSS_RCM_MSS_ELM_CLK_DIV_VAL_CLKDIVR_SHIFT      (0x00000000U)
#define CSL_MSS_RCM_MSS_ELM_CLK_DIV_VAL_CLKDIVR_RESETVAL   (0x00000333U)
#define CSL_MSS_RCM_MSS_ELM_CLK_DIV_VAL_CLKDIVR_MAX        (0x00000FFFU)

#define CSL_MSS_RCM_MSS_ELM_CLK_DIV_VAL_RESETVAL                               (0x00000333U)

/* MSS_MII10_CLK_DIV_VAL */

#define CSL_MSS_RCM_MII10_CLK_DIV_VAL_CLKDIVR_MASK   (0x00FFFFFFU)
#define CSL_MSS_RCM_MII10_CLK_DIV_VAL_CLKDIVR_SHIFT  (0x00000000U)
#define CSL_MSS_RCM_MII10_CLK_DIV_VAL_CLKDIVR_RESETVAL (0x00636363U)
#define CSL_MSS_RCM_MII10_CLK_DIV_VAL_CLKDIVR_MAX    (0x00FFFFFFU)

#define CSL_MSS_RCM_MII10_CLK_DIV_VAL_RESETVAL                             (0x00636363U)

/* MSS_MII100_CLK_DIV_VAL */

#define CSL_MSS_RCM_MII100_CLK_DIV_VAL_CLKDIVR_MASK (0x00000FFFU)
#define CSL_MSS_RCM_MII100_CLK_DIV_VAL_CLKDIVR_SHIFT (0x00000000U)
#define CSL_MSS_RCM_MII100_CLK_DIV_VAL_CLKDIVR_RESETVAL (0x00000999U)
#define CSL_MSS_RCM_MII100_CLK_DIV_VAL_CLKDIVR_MAX  (0x00000FFFU)

#define CSL_MSS_RCM_MII100_CLK_DIV_VAL_RESETVAL                            (0x00000999U)

/* MSS_RGMII_CLK_DIV_VAL */

#define CSL_MSS_RCM_RGMII_CLK_DIV_VAL_CLKDIVR_MASK   (0x00000FFFU)
#define CSL_MSS_RCM_RGMII_CLK_DIV_VAL_CLKDIVR_SHIFT  (0x00000000U)
#define CSL_MSS_RCM_RGMII_CLK_DIV_VAL_CLKDIVR_RESETVAL (0x00000111U)
#define CSL_MSS_RCM_RGMII_CLK_DIV_VAL_CLKDIVR_MAX    (0x00000FFFU)

#define CSL_MSS_RCM_RGMII_CLK_DIV_VAL_RESETVAL                             (0x00000111U)

/* MSS_XTAL_32K_CLK_DIV_VAL */

#define CSL_MSS_RCM_XTAL_32K_CLK_DIV_VAL_CLKDIVR_MASK (0x3FFFFFFFU)
#define CSL_MSS_RCM_XTAL_32K_CLK_DIV_VAL_CLKDIVR_SHIFT (0x00000000U)
#define CSL_MSS_RCM_XTAL_32K_CLK_DIV_VAL_CLKDIVR_RESETVAL (0x30CC330CU)
#define CSL_MSS_RCM_XTAL_32K_CLK_DIV_VAL_CLKDIVR_MAX (0x3FFFFFFFU)

#define CSL_MSS_RCM_XTAL_32K_CLK_DIV_VAL_RESETVAL                          (0x30CC330CU)

/* XTAL_TEMPSENSE_32K_CLK_DIV_VAL */

#define CSL_MSS_RCM_XTAL_TEMPSENSE_32K_CLK_DIV_VAL_CLKDIVR_MASK (0x3FFFFFFFU)
#define CSL_MSS_RCM_XTAL_TEMPSENSE_32K_CLK_DIV_VAL_CLKDIVR_SHIFT (0x00000000U)
#define CSL_MSS_RCM_XTAL_TEMPSENSE_32K_CLK_DIV_VAL_CLKDIVR_RESETVAL (0x30CC330CU)
#define CSL_MSS_RCM_XTAL_TEMPSENSE_32K_CLK_DIV_VAL_CLKDIVR_MAX (0x3FFFFFFFU)

#define CSL_MSS_RCM_XTAL_TEMPSENSE_32K_CLK_DIV_VAL_RESETVAL                    (0x30CC330CU)

/* MSS_I2C_CLK_DIV_VAL */

#define CSL_MSS_RCM_I2C_CLK_DIV_VAL_CLKDIVR_MASK       (0x00000FFFU)
#define CSL_MSS_RCM_I2C_CLK_DIV_VAL_CLKDIVR_SHIFT      (0x00000000U)
#define CSL_MSS_RCM_I2C_CLK_DIV_VAL_CLKDIVR_RESETVAL   (0x00000000U)
#define CSL_MSS_RCM_I2C_CLK_DIV_VAL_CLKDIVR_MAX        (0x00000FFFU)

#define CSL_MSS_RCM_I2C_CLK_DIV_VAL_RESETVAL                               (0x00000000U)

/* LIN0_UART0_CLK_DIV_VAL */

#define CSL_MSS_RCM_LIN0_UART0_CLK_DIV_VAL_CLKDIVR_MASK (0x00000FFFU)
#define CSL_MSS_RCM_LIN0_UART0_CLK_DIV_VAL_CLKDIVR_SHIFT (0x00000000U)
#define CSL_MSS_RCM_LIN0_UART0_CLK_DIV_VAL_CLKDIVR_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_LIN0_UART0_CLK_DIV_VAL_CLKDIVR_MAX (0x00000FFFU)

#define CSL_MSS_RCM_LIN0_UART0_CLK_DIV_VAL_RESETVAL                        (0x00000000U)

/* LIN1_UART1_CLK_DIV_VAL */

#define CSL_MSS_RCM_LIN1_UART1_CLK_DIV_VAL_CLKDIVR_MASK (0x00000FFFU)
#define CSL_MSS_RCM_LIN1_UART1_CLK_DIV_VAL_CLKDIVR_SHIFT (0x00000000U)
#define CSL_MSS_RCM_LIN1_UART1_CLK_DIV_VAL_CLKDIVR_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_LIN1_UART1_CLK_DIV_VAL_CLKDIVR_MAX (0x00000FFFU)

#define CSL_MSS_RCM_LIN1_UART1_CLK_DIV_VAL_RESETVAL                        (0x00000000U)

/* LIN2_UART2_CLK_DIV_VAL */

#define CSL_MSS_RCM_LIN2_UART2_CLK_DIV_VAL_CLKDIVR_MASK (0x00000FFFU)
#define CSL_MSS_RCM_LIN2_UART2_CLK_DIV_VAL_CLKDIVR_SHIFT (0x00000000U)
#define CSL_MSS_RCM_LIN2_UART2_CLK_DIV_VAL_CLKDIVR_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_LIN2_UART2_CLK_DIV_VAL_CLKDIVR_MAX (0x00000FFFU)

#define CSL_MSS_RCM_LIN2_UART2_CLK_DIV_VAL_RESETVAL                        (0x00000000U)

/* LIN3_UART3_CLK_DIV_VAL */

#define CSL_MSS_RCM_LIN3_UART3_CLK_DIV_VAL_CLKDIVR_MASK (0x00000FFFU)
#define CSL_MSS_RCM_LIN3_UART3_CLK_DIV_VAL_CLKDIVR_SHIFT (0x00000000U)
#define CSL_MSS_RCM_LIN3_UART3_CLK_DIV_VAL_CLKDIVR_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_LIN3_UART3_CLK_DIV_VAL_CLKDIVR_MAX (0x00000FFFU)

#define CSL_MSS_RCM_LIN3_UART3_CLK_DIV_VAL_RESETVAL                        (0x00000000U)


/* LIN4_UART4_CLK_DIV_VAL */
#define CSL_MSS_RCM_LIN4_UART4_CLK_DIV_VAL_CLKDIVR_MASK                     (0x00000FFFU)
#define CSL_MSS_RCM_LIN4_UART4_CLK_DIV_VAL_CLKDIVR_SHIFT                    (0x00000000U)
#define CSL_MSS_RCM_LIN4_UART4_CLK_DIV_VAL_CLKDIVR_RESETVAL                 (0x00000000U)
#define CSL_MSS_RCM_LIN4_UART4_CLK_DIV_VAL_CLKDIVR_MAX                      (0x00000FFFU)

#define CSL_MSS_RCM_LIN4_UART4_CLK_DIV_VAL_RESETVAL                        (0x00000000U)


/* LIN5_UART5_CLK_DIV_VAL */
#define CSL_MSS_RCM_LIN5_UART5_CLK_DIV_VAL_CLKDIVR_MASK                     (0x00000FFFU)
#define CSL_MSS_RCM_LIN5_UART5_CLK_DIV_VAL_CLKDIVR_SHIFT                    (0x00000000U)
#define CSL_MSS_RCM_LIN5_UART5_CLK_DIV_VAL_CLKDIVR_RESETVAL                 (0x00000000U)
#define CSL_MSS_RCM_LIN5_UART5_CLK_DIV_VAL_CLKDIVR_MAX                      (0x00000FFFU)

#define CSL_MSS_RCM_LIN5_UART5_CLK_DIV_VAL_RESETVAL                        (0x00000000U)



/* ICSSM0_CORE_CLK_DIV_VAL */
#define CSL_MSS_RCM_ICSSM0_CORE_CLK_DIV_VAL_CLKDIVR_MASK              (0x00000FFFU)
#define CSL_MSS_RCM_ICSSM0_CORE_CLK_DIV_VAL_CLKDIVR_SHIFT             (0x00000000U)
#define CSL_MSS_RCM_ICSSM0_CORE_CLK_DIV_VAL_CLKDIVR_RESETVAL          (0x00000000U)
#define CSL_MSS_RCM_ICSSM0_CORE_CLK_DIV_VAL_CLKDIVR_MAX               (0x00000FFFU)



/* ICSSM1_CORE_CLK_DIV_VAL */
#define CSL_MSS_RCM_ICSSM1_CORE_CLK_DIV_VAL_CLKDIVR_MASK              (0x00000FFFU)
#define CSL_MSS_RCM_ICSSM1_CORE_CLK_DIV_VAL_CLKDIVR_SHIFT             (0x00000000U)
#define CSL_MSS_RCM_ICSSM1_CORE_CLK_DIV_VAL_CLKDIVR_RESETVAL          (0x00000000U)
#define CSL_MSS_RCM_ICSSM1_CORE_CLK_DIV_VAL_CLKDIVR_MAX               (0x00000FFFU)



/* MSS_MCAN0_CLK_GATE */

#define CSL_MSS_RCM_MCAN0_CLK_GATE_GATED_MASK           (0x00000007U)
#define CSL_MSS_RCM_MCAN0_CLK_GATE_GATED_SHIFT          (0x00000000U)
#define CSL_MSS_RCM_MCAN0_CLK_GATE_GATED_RESETVAL       (0x00000000U)
#define CSL_MSS_RCM_MCAN0_CLK_GATE_GATED_MAX            (0x00000007U)

#define CSL_MSS_RCM_MCAN0_CLK_GATE_RESETVAL                                (0x00000000U)

/* MSS_MCAN1_CLK_GATE */

#define CSL_MSS_RCM_MCAN1_CLK_GATE_GATED_MASK           (0x00000007U)
#define CSL_MSS_RCM_MCAN1_CLK_GATE_GATED_SHIFT          (0x00000000U)
#define CSL_MSS_RCM_MCAN1_CLK_GATE_GATED_RESETVAL       (0x00000000U)
#define CSL_MSS_RCM_MCAN1_CLK_GATE_GATED_MAX            (0x00000007U)

#define CSL_MSS_RCM_MCAN1_CLK_GATE_RESETVAL                                (0x00000000U)

/* MSS_RTI0_CLK_GATE */

#define CSL_MSS_RCM_RTI0_CLK_GATE_GATED_MASK             (0x00000007U)
#define CSL_MSS_RCM_RTI0_CLK_GATE_GATED_SHIFT            (0x00000000U)
#define CSL_MSS_RCM_RTI0_CLK_GATE_GATED_RESETVAL         (0x00000000U)
#define CSL_MSS_RCM_RTI0_CLK_GATE_GATED_MAX              (0x00000007U)

#define CSL_MSS_RCM_RTI0_CLK_GATE_RESETVAL                                 (0x00000000U)

/* MSS_RTI1_CLK_GATE */

#define CSL_MSS_RCM_RTI1_CLK_GATE_GATED_MASK             (0x00000007U)
#define CSL_MSS_RCM_RTI1_CLK_GATE_GATED_SHIFT            (0x00000000U)
#define CSL_MSS_RCM_RTI1_CLK_GATE_GATED_RESETVAL         (0x00000000U)
#define CSL_MSS_RCM_RTI1_CLK_GATE_GATED_MAX              (0x00000007U)

#define CSL_MSS_RCM_RTI1_CLK_GATE_RESETVAL                                 (0x00000000U)

/* MSS_RTI2_CLK_GATE */

#define CSL_MSS_RCM_RTI2_CLK_GATE_GATED_MASK             (0x00000007U)
#define CSL_MSS_RCM_RTI2_CLK_GATE_GATED_SHIFT            (0x00000000U)
#define CSL_MSS_RCM_RTI2_CLK_GATE_GATED_RESETVAL         (0x00000000U)
#define CSL_MSS_RCM_RTI2_CLK_GATE_GATED_MAX              (0x00000007U)

#define CSL_MSS_RCM_RTI2_CLK_GATE_RESETVAL                                 (0x00000000U)

/* MSS_RTI3_CLK_GATE */

#define CSL_MSS_RCM_RTI3_CLK_GATE_GATED_MASK             (0x00000007U)
#define CSL_MSS_RCM_RTI3_CLK_GATE_GATED_SHIFT            (0x00000000U)
#define CSL_MSS_RCM_RTI3_CLK_GATE_GATED_RESETVAL         (0x00000000U)
#define CSL_MSS_RCM_RTI3_CLK_GATE_GATED_MAX              (0x00000007U)

#define CSL_MSS_RCM_RTI3_CLK_GATE_RESETVAL                                 (0x00000000U)

/* MSS_MCSPI0_CLK_GATE */

#define CSL_MSS_RCM_MCSPI0_CLK_GATE_GATED_MASK             (0x00000007U)
#define CSL_MSS_RCM_MCSPI0_CLK_GATE_GATED_SHIFT            (0x00000000U)
#define CSL_MSS_RCM_MCSPI0_CLK_GATE_GATED_RESETVAL         (0x00000000U)
#define CSL_MSS_RCM_MCSPI0_CLK_GATE_GATED_MAX              (0x00000007U)

#define CSL_MSS_RCM_MCSPI0_CLK_GATE_RESETVAL                                 (0x00000000U)

/* MSS_MCSPI1_CLK_GATE */

#define CSL_MSS_RCM_MCSPI1_CLK_GATE_GATED_MASK             (0x00000007U)
#define CSL_MSS_RCM_MCSPI1_CLK_GATE_GATED_SHIFT            (0x00000000U)
#define CSL_MSS_RCM_MCSPI1_CLK_GATE_GATED_RESETVAL         (0x00000000U)
#define CSL_MSS_RCM_MCSPI1_CLK_GATE_GATED_MAX              (0x00000007U)

#define CSL_MSS_RCM_MCSPI1_CLK_GATE_RESETVAL                                 (0x00000000U)

/* MSS_MCSPI2_CLK_GATE */

#define CSL_MSS_RCM_MCSPI2_CLK_GATE_GATED_MASK             (0x00000007U)
#define CSL_MSS_RCM_MCSPI2_CLK_GATE_GATED_SHIFT            (0x00000000U)
#define CSL_MSS_RCM_MCSPI2_CLK_GATE_GATED_RESETVAL         (0x00000000U)
#define CSL_MSS_RCM_MCSPI2_CLK_GATE_GATED_MAX              (0x00000007U)

#define CSL_MSS_RCM_MCSPI2_CLK_GATE_RESETVAL                                 (0x00000000U)

/* MSS_MCSPI3_CLK_GATE */

#define CSL_MSS_RCM_MCSPI3_CLK_GATE_GATED_MASK             (0x00000007U)
#define CSL_MSS_RCM_MCSPI3_CLK_GATE_GATED_SHIFT            (0x00000000U)
#define CSL_MSS_RCM_MCSPI3_CLK_GATE_GATED_RESETVAL         (0x00000000U)
#define CSL_MSS_RCM_MCSPI3_CLK_GATE_GATED_MAX              (0x00000007U)

#define CSL_MSS_RCM_MCSPI3_CLK_GATE_RESETVAL                                 (0x00000000U)

/* MSS_WDT0_CLK_GATE */

#define CSL_MSS_RCM_WDT0_CLK_GATE_GATED_MASK             (0x00000007U)
#define CSL_MSS_RCM_WDT0_CLK_GATE_GATED_SHIFT            (0x00000000U)
#define CSL_MSS_RCM_WDT0_CLK_GATE_GATED_RESETVAL         (0x00000000U)
#define CSL_MSS_RCM_WDT0_CLK_GATE_GATED_MAX              (0x00000007U)

#define CSL_MSS_RCM_WDT0_CLK_GATE_RESETVAL                                 (0x00000000U)

/* MSS_WDT1_CLK_GATE */

#define CSL_MSS_RCM_WDT1_CLK_GATE_GATED_MASK             (0x00000007U)
#define CSL_MSS_RCM_WDT1_CLK_GATE_GATED_SHIFT            (0x00000000U)
#define CSL_MSS_RCM_WDT1_CLK_GATE_GATED_RESETVAL         (0x00000000U)
#define CSL_MSS_RCM_WDT1_CLK_GATE_GATED_MAX              (0x00000007U)

#define CSL_MSS_RCM_WDT1_CLK_GATE_RESETVAL                                 (0x00000000U)

/* ICSSM0_UART_CLK_GATE */

#define CSL_MSS_RCM_ICSSM0_UART_CLK_GATE_GATED_MASK       (0x00000007U)
#define CSL_MSS_RCM_ICSSM0_UART_CLK_GATE_GATED_SHIFT      (0x00000000U)
#define CSL_MSS_RCM_ICSSM0_UART_CLK_GATE_GATED_RESETVAL   (0x00000000U)
#define CSL_MSS_RCM_ICSSM0_UART_CLK_GATE_GATED_MAX        (0x00000007U)

#define CSL_MSS_RCM_ICSSM0_UART_CLK_GATE_RESETVAL                              (0x00000000U)

/* ICSSM1_UART_CLK_GATE */

#define CSL_MSS_RCM_ICSSM1_UART_CLK_GATE_GATED_MASK       (0x00000007U)
#define CSL_MSS_RCM_ICSSM1_UART_CLK_GATE_GATED_SHIFT      (0x00000000U)
#define CSL_MSS_RCM_ICSSM1_UART_CLK_GATE_GATED_RESETVAL   (0x00000000U)
#define CSL_MSS_RCM_ICSSM1_UART_CLK_GATE_GATED_MAX        (0x00000007U)

#define CSL_MSS_RCM_ICSSM1_UART_CLK_GATE_RESETVAL                              (0x00000000U)

/* OSPI0_CLK_GATE */

#define CSL_MSS_RCM_OSPI0_CLK_GATE_GATED_MASK           (0x00000007U)
#define CSL_MSS_RCM_OSPI0_CLK_GATE_GATED_SHIFT          (0x00000000U)
#define CSL_MSS_RCM_OSPI0_CLK_GATE_GATED_RESETVAL       (0x00000000U)
#define CSL_MSS_RCM_OSPI0_CLK_GATE_GATED_MAX            (0x00000007U)



/* OSPI1_CLK_GATE */
#define CSL_MSS_RCM_OSPI1_CLK_GATE_GATED_MASK                               (0x00000007U)
#define CSL_MSS_RCM_OSPI1_CLK_GATE_GATED_SHIFT                              (0x00000000U)
#define CSL_MSS_RCM_OSPI1_CLK_GATE_GATED_RESETVAL                           (0x00000000U)
#define CSL_MSS_RCM_OSPI1_CLK_GATE_GATED_MAX                                (0x00000007U)

/* CONTROLSS_PLL_CLK_GATE */

#define CSL_MSS_RCM_CONTROLSS_PLL_CLK_GATE_GATED_MASK   (0x00000007U)
#define CSL_MSS_RCM_CONTROLSS_PLL_CLK_GATE_GATED_SHIFT  (0x00000000U)
#define CSL_MSS_RCM_CONTROLSS_PLL_CLK_GATE_GATED_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_CONTROLSS_PLL_CLK_GATE_GATED_MAX    (0x00000007U)

#define CSL_MSS_RCM_CONTROLSS_PLL_CLK_GATE_RESETVAL                            (0x00000000U)

/* MSS_CPTS_CLK_GATE */

#define CSL_MSS_RCM_CPTS_CLK_GATE_GATED_MASK             (0x00000007U)
#define CSL_MSS_RCM_CPTS_CLK_GATE_GATED_SHIFT            (0x00000000U)
#define CSL_MSS_RCM_CPTS_CLK_GATE_GATED_RESETVAL         (0x00000000U)
#define CSL_MSS_RCM_CPTS_CLK_GATE_GATED_MAX              (0x00000007U)

#define CSL_MSS_RCM_CPTS_CLK_GATE_RESETVAL                                 (0x00000000U)

/* GPMC_CLK_GATE */

#define CSL_MSS_RCM_GPMC_CLK_GATE_GATED_MASK                     (0x00000007U)
#define CSL_MSS_RCM_GPMC_CLK_GATE_GATED_SHIFT                    (0x00000000U)
#define CSL_MSS_RCM_GPMC_CLK_GATE_GATED_RESETVAL                 (0x00000000U)
#define CSL_MSS_RCM_GPMC_CLK_GATE_GATED_MAX                      (0x00000007U)

#define CSL_MSS_RCM_GPMC_CLK_GATE_RESETVAL                                     (0x00000000U)

/* MSS_MMC0_CLK_GATE */

#define CSL_MSS_RCM_MMC0_CLK_GATE_GATED_MASK           (0x00000007U)
#define CSL_MSS_RCM_MMC0_CLK_GATE_GATED_SHIFT          (0x00000000U)
#define CSL_MSS_RCM_MMC0_CLK_GATE_GATED_RESETVAL       (0x00000000U)
#define CSL_MSS_RCM_MMC0_CLK_GATE_GATED_MAX            (0x00000007U)

#define CSL_MSS_RCM_MMC0_CLK_GATE_RESETVAL                                (0x00000000U)

/* MSS_ELM_CLK_GATE */

#define CSL_MSS_RCM_MSS_ELM_CLK_GATE_GATED_MASK               (0x00000007U)
#define CSL_MSS_RCM_MSS_ELM_CLK_GATE_GATED_SHIFT              (0x00000000U)
#define CSL_MSS_RCM_MSS_ELM_CLK_GATE_GATED_RESETVAL           (0x00000000U)
#define CSL_MSS_RCM_MSS_ELM_CLK_GATE_GATED_MAX                (0x00000007U)

#define CSL_MSS_RCM_MSS_ELM_CLK_GATE_RESETVAL                                  (0x00000000U)

/* MSS_MII10_CLK_GATE */

#define CSL_MSS_RCM_MII10_CLK_GATE_GATED_MASK           (0x00000007U)
#define CSL_MSS_RCM_MII10_CLK_GATE_GATED_SHIFT          (0x00000000U)
#define CSL_MSS_RCM_MII10_CLK_GATE_GATED_RESETVAL       (0x00000000U)
#define CSL_MSS_RCM_MII10_CLK_GATE_GATED_MAX            (0x00000007U)

#define CSL_MSS_RCM_MII10_CLK_GATE_RESETVAL                                (0x00000000U)

/* MSS_MII100_CLK_GATE */

#define CSL_MSS_RCM_MII100_CLK_GATE_GATED_MASK         (0x00000007U)
#define CSL_MSS_RCM_MII100_CLK_GATE_GATED_SHIFT        (0x00000000U)
#define CSL_MSS_RCM_MII100_CLK_GATE_GATED_RESETVAL     (0x00000000U)
#define CSL_MSS_RCM_MII100_CLK_GATE_GATED_MAX          (0x00000007U)

#define CSL_MSS_RCM_MII100_CLK_GATE_RESETVAL                               (0x00000000U)

/* MSS_RGMII_CLK_GATE */

#define CSL_MSS_RCM_RGMII_CLK_GATE_GATED_MASK           (0x00000007U)
#define CSL_MSS_RCM_RGMII_CLK_GATE_GATED_SHIFT          (0x00000000U)
#define CSL_MSS_RCM_RGMII_CLK_GATE_GATED_RESETVAL       (0x00000000U)
#define CSL_MSS_RCM_RGMII_CLK_GATE_GATED_MAX            (0x00000007U)

#define CSL_MSS_RCM_RGMII_CLK_GATE_RESETVAL                                (0x00000000U)

/* MSS_MMC0_32K_CLK_GATE */

#define CSL_MSS_RCM_MMC0_32K_CLK_GATE_GATED_MASK   (0x00000007U)
#define CSL_MSS_RCM_MMC0_32K_CLK_GATE_GATED_SHIFT  (0x00000000U)
#define CSL_MSS_RCM_MMC0_32K_CLK_GATE_GATED_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_MMC0_32K_CLK_GATE_GATED_MAX    (0x00000007U)

#define CSL_MSS_RCM_MMC0_32K_CLK_GATE_RESETVAL                            (0x00000000U)

/* MSS_TEMPSENSE_32K_CLK_GATE */

#define CSL_MSS_RCM_TEMPSENSE_32K_CLK_GATE_GATED_MASK (0x00000007U)
#define CSL_MSS_RCM_TEMPSENSE_32K_CLK_GATE_GATED_SHIFT (0x00000000U)
#define CSL_MSS_RCM_TEMPSENSE_32K_CLK_GATE_GATED_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_TEMPSENSE_32K_CLK_GATE_GATED_MAX (0x00000007U)

#define CSL_MSS_RCM_TEMPSENSE_32K_CLK_GATE_RESETVAL                        (0x00000000U)

/* MSS_CPSW_CLK_GATE */

#define CSL_MSS_RCM_CPSW_CLK_GATE_GATED_MASK             (0x00000007U)
#define CSL_MSS_RCM_CPSW_CLK_GATE_GATED_SHIFT            (0x00000000U)
#define CSL_MSS_RCM_CPSW_CLK_GATE_GATED_RESETVAL         (0x00000000U)
#define CSL_MSS_RCM_CPSW_CLK_GATE_GATED_MAX              (0x00000007U)

#define CSL_MSS_RCM_CPSW_CLK_GATE_RESETVAL                                 (0x00000000U)

/* MSS_I2C0_CLK_GATE */

#define CSL_MSS_RCM_I2C0_CLK_GATE_GATED_MASK             (0x00000007U)
#define CSL_MSS_RCM_I2C0_CLK_GATE_GATED_SHIFT            (0x00000000U)
#define CSL_MSS_RCM_I2C0_CLK_GATE_GATED_RESETVAL         (0x00000000U)
#define CSL_MSS_RCM_I2C0_CLK_GATE_GATED_MAX              (0x00000007U)

#define CSL_MSS_RCM_I2C0_CLK_GATE_RESETVAL                                 (0x00000000U)

/* MSS_I2C1_CLK_GATE */

#define CSL_MSS_RCM_I2C1_CLK_GATE_GATED_MASK             (0x00000007U)
#define CSL_MSS_RCM_I2C1_CLK_GATE_GATED_SHIFT            (0x00000000U)
#define CSL_MSS_RCM_I2C1_CLK_GATE_GATED_RESETVAL         (0x00000000U)
#define CSL_MSS_RCM_I2C1_CLK_GATE_GATED_MAX              (0x00000007U)

#define CSL_MSS_RCM_I2C1_CLK_GATE_RESETVAL                                 (0x00000000U)

/* MSS_I2C2_CLK_GATE */

#define CSL_MSS_RCM_I2C2_CLK_GATE_GATED_MASK             (0x00000007U)
#define CSL_MSS_RCM_I2C2_CLK_GATE_GATED_SHIFT            (0x00000000U)
#define CSL_MSS_RCM_I2C2_CLK_GATE_GATED_RESETVAL         (0x00000000U)
#define CSL_MSS_RCM_I2C2_CLK_GATE_GATED_MAX              (0x00000007U)

#define CSL_MSS_RCM_I2C2_CLK_GATE_RESETVAL                                 (0x00000000U)

/* LIN0_CLK_GATE */

#define CSL_MSS_RCM_LIN0_CLK_GATE_GATED_MASK             (0x00000007U)
#define CSL_MSS_RCM_LIN0_CLK_GATE_GATED_SHIFT            (0x00000000U)
#define CSL_MSS_RCM_LIN0_CLK_GATE_GATED_RESETVAL         (0x00000000U)
#define CSL_MSS_RCM_LIN0_CLK_GATE_GATED_MAX              (0x00000007U)

#define CSL_MSS_RCM_LIN0_CLK_GATE_RESETVAL                                 (0x00000000U)

/* LIN1_CLK_GATE */

#define CSL_MSS_RCM_LIN1_CLK_GATE_GATED_MASK             (0x00000007U)
#define CSL_MSS_RCM_LIN1_CLK_GATE_GATED_SHIFT            (0x00000000U)
#define CSL_MSS_RCM_LIN1_CLK_GATE_GATED_RESETVAL         (0x00000000U)
#define CSL_MSS_RCM_LIN1_CLK_GATE_GATED_MAX              (0x00000007U)

#define CSL_MSS_RCM_LIN1_CLK_GATE_RESETVAL                                 (0x00000000U)

/* LIN2_CLK_GATE */

#define CSL_MSS_RCM_LIN2_CLK_GATE_GATED_MASK             (0x00000007U)
#define CSL_MSS_RCM_LIN2_CLK_GATE_GATED_SHIFT            (0x00000000U)
#define CSL_MSS_RCM_LIN2_CLK_GATE_GATED_RESETVAL         (0x00000000U)
#define CSL_MSS_RCM_LIN2_CLK_GATE_GATED_MAX              (0x00000007U)

#define CSL_MSS_RCM_LIN2_CLK_GATE_RESETVAL                                 (0x00000000U)

/* UART0_CLK_GATE */

#define CSL_MSS_RCM_UART0_CLK_GATE_GATED_MASK           (0x00000007U)
#define CSL_MSS_RCM_UART0_CLK_GATE_GATED_SHIFT          (0x00000000U)
#define CSL_MSS_RCM_UART0_CLK_GATE_GATED_RESETVAL       (0x00000000U)
#define CSL_MSS_RCM_UART0_CLK_GATE_GATED_MAX            (0x00000007U)

#define CSL_MSS_RCM_UART0_CLK_GATE_RESETVAL                                (0x00000000U)

/* UART1_CLK_GATE */

#define CSL_MSS_RCM_UART1_CLK_GATE_GATED_MASK           (0x00000007U)
#define CSL_MSS_RCM_UART1_CLK_GATE_GATED_SHIFT          (0x00000000U)
#define CSL_MSS_RCM_UART1_CLK_GATE_GATED_RESETVAL       (0x00000000U)
#define CSL_MSS_RCM_UART1_CLK_GATE_GATED_MAX            (0x00000007U)

#define CSL_MSS_RCM_UART1_CLK_GATE_RESETVAL                                (0x00000000U)

/* UART2_CLK_GATE */

#define CSL_MSS_RCM_UART2_CLK_GATE_GATED_MASK           (0x00000007U)
#define CSL_MSS_RCM_UART2_CLK_GATE_GATED_SHIFT          (0x00000000U)
#define CSL_MSS_RCM_UART2_CLK_GATE_GATED_RESETVAL       (0x00000000U)
#define CSL_MSS_RCM_UART2_CLK_GATE_GATED_MAX            (0x00000007U)

#define CSL_MSS_RCM_UART2_CLK_GATE_RESETVAL                                (0x00000000U)

/* UART3_CLK_GATE */

#define CSL_MSS_RCM_UART3_CLK_GATE_GATED_MASK           (0x00000007U)
#define CSL_MSS_RCM_UART3_CLK_GATE_GATED_SHIFT          (0x00000000U)
#define CSL_MSS_RCM_UART3_CLK_GATE_GATED_RESETVAL       (0x00000000U)
#define CSL_MSS_RCM_UART3_CLK_GATE_GATED_MAX            (0x00000007U)

#define CSL_MSS_RCM_UART3_CLK_GATE_RESETVAL                                (0x00000000U)


/* UART4_CLK_GATE */
#define CSL_MSS_RCM_UART4_CLK_GATE_GATED_MASK                               (0x00000007U)
#define CSL_MSS_RCM_UART4_CLK_GATE_GATED_SHIFT                              (0x00000000U)
#define CSL_MSS_RCM_UART4_CLK_GATE_GATED_RESETVAL                           (0x00000000U)
#define CSL_MSS_RCM_UART4_CLK_GATE_GATED_MAX                                (0x00000007U)

#define CSL_MSS_RCM_UART4_CLK_GATE_RESETVAL                                (0x00000000U)


/* UART5_CLK_GATE */
#define CSL_MSS_RCM_UART5_CLK_GATE_GATED_MASK                               (0x00000007U)
#define CSL_MSS_RCM_UART5_CLK_GATE_GATED_SHIFT                              (0x00000000U)
#define CSL_MSS_RCM_UART5_CLK_GATE_GATED_RESETVAL                           (0x00000000U)
#define CSL_MSS_RCM_UART5_CLK_GATE_GATED_MAX                                (0x00000007U)

#define CSL_MSS_RCM_UART5_CLK_GATE_RESETVAL                                (0x00000000U)


/* ICSSM0_CORE_CLK_GATE */
#define CSL_MSS_RCM_ICSSM0_CORE_CLK_GATE_GATED_MASK               (0x00000007U)
#define CSL_MSS_RCM_ICSSM0_CORE_CLK_GATE_GATED_SHIFT              (0x00000000U)
#define CSL_MSS_RCM_ICSSM0_CORE_CLK_GATE_GATED_RESETVAL           (0x00000000U)
#define CSL_MSS_RCM_ICSSM0_CORE_CLK_GATE_GATED_MAX                (0x00000007U)



/* ICSSM1_CORE_CLK_GATE */
#define CSL_MSS_RCM_ICSSM1_CORE_CLK_GATE_GATED_MASK               (0x00000007U)
#define CSL_MSS_RCM_ICSSM1_CORE_CLK_GATE_GATED_SHIFT              (0x00000000U)
#define CSL_MSS_RCM_ICSSM1_CORE_CLK_GATE_GATED_RESETVAL           (0x00000000U)
#define CSL_MSS_RCM_ICSSM1_CORE_CLK_GATE_GATED_MAX                (0x00000007U)


/* R5SS0_CORE0_GATE */

#define CSL_MSS_RCM_R5SS0_CORE0_GATE_CLKGATE_MASK                 (0x00000007U)
#define CSL_MSS_RCM_R5SS0_CORE0_GATE_CLKGATE_SHIFT                (0x00000000U)
#define CSL_MSS_RCM_R5SS0_CORE0_GATE_CLKGATE_RESETVAL             (0x00000000U)
#define CSL_MSS_RCM_R5SS0_CORE0_GATE_CLKGATE_MAX                  (0x00000007U)

#define CSL_MSS_RCM_R5SS0_CORE0_GATE_RESETVAL                                    (0x00000000U)

/* R5SS0_CORE1_GATE */

#define CSL_MSS_RCM_R5SS0_CORE1_GATE_CLKGATE_MASK                 (0x00000007U)
#define CSL_MSS_RCM_R5SS0_CORE1_GATE_CLKGATE_SHIFT                (0x00000000U)
#define CSL_MSS_RCM_R5SS0_CORE1_GATE_CLKGATE_RESETVAL             (0x00000000U)
#define CSL_MSS_RCM_R5SS0_CORE1_GATE_CLKGATE_MAX                  (0x00000007U)

#define CSL_MSS_RCM_R5SS0_CORE1_GATE_RESETVAL                                    (0x00000000U)

/* ICSSM0_IEP_CLK_GATE */

#define CSL_MSS_RCM_ICSSM0_IEP_CLK_GATE_GATED_MASK (0x00000007U)
#define CSL_MSS_RCM_ICSSM0_IEP_CLK_GATE_GATED_SHIFT (0x00000000U)
#define CSL_MSS_RCM_ICSSM0_IEP_CLK_GATE_GATED_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_ICSSM0_IEP_CLK_GATE_GATED_MAX  (0x00000007U)

#define CSL_MSS_RCM_ICSSM0_IEP_CLK_GATE_RESETVAL                           (0x00000000U)

/* ICSSM1_IEP_CLK_GATE */

#define CSL_MSS_RCM_ICSSM1_IEP_CLK_GATE_GATED_MASK (0x00000007U)
#define CSL_MSS_RCM_ICSSM1_IEP_CLK_GATE_GATED_SHIFT (0x00000000U)
#define CSL_MSS_RCM_ICSSM1_IEP_CLK_GATE_GATED_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_ICSSM1_IEP_CLK_GATE_GATED_MAX  (0x00000007U)

#define CSL_MSS_RCM_ICSSM1_IEP_CLK_GATE_RESETVAL                           (0x00000000U)

/* ICSSM0_SYS_CLK_GATE */

#define CSL_MSS_RCM_ICSSM0_SYS_CLK_GATE_GATED_MASK (0x00000007U)
#define CSL_MSS_RCM_ICSSM0_SYS_CLK_GATE_GATED_SHIFT (0x00000000U)
#define CSL_MSS_RCM_ICSSM0_SYS_CLK_GATE_GATED_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_ICSSM0_SYS_CLK_GATE_GATED_MAX  (0x00000007U)

#define CSL_MSS_RCM_ICSSM0_SYS_CLK_GATE_RESETVAL                           (0x00000000U)

/* ICSSM1_SYS_CLK_GATE */

#define CSL_MSS_RCM_ICSSM1_SYS_CLK_GATE_GATED_MASK (0x00000007U)
#define CSL_MSS_RCM_ICSSM1_SYS_CLK_GATE_GATED_SHIFT (0x00000000U)
#define CSL_MSS_RCM_ICSSM1_SYS_CLK_GATE_GATED_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_ICSSM1_SYS_CLK_GATE_GATED_MAX  (0x00000007U)

#define CSL_MSS_RCM_ICSSM1_SYS_CLK_GATE_RESETVAL                           (0x00000000U)

/* USB_CLK_GATE */

#define CSL_MSS_RCM_USB_CLK_GATE_GATED_MASK                       (0x00000007U)
#define CSL_MSS_RCM_USB_CLK_GATE_GATED_SHIFT                      (0x00000000U)
#define CSL_MSS_RCM_USB_CLK_GATE_GATED_RESETVAL                   (0x00000000U)
#define CSL_MSS_RCM_USB_CLK_GATE_GATED_MAX                        (0x00000007U)

#define CSL_MSS_RCM_USB_CLK_GATE_RESETVAL                                      (0x00000000U)

/* USB_WKUP_CLK_GATE */

#define CSL_MSS_RCM_USB_WKUP_CLK_GATE_GATED_MASK             (0x00000007U)
#define CSL_MSS_RCM_USB_WKUP_CLK_GATE_GATED_SHIFT            (0x00000000U)
#define CSL_MSS_RCM_USB_WKUP_CLK_GATE_GATED_RESETVAL         (0x00000000U)
#define CSL_MSS_RCM_USB_WKUP_CLK_GATE_GATED_MAX              (0x00000007U)

/* USB_XTAL_CLK_GATE */
#define CSL_MSS_RCM_USB_XTAL_CLK_GATE_GATED_MASK                                (0x00000007U)
#define CSL_MSS_RCM_USB_XTAL_CLK_GATE_GATED_SHIFT                               (0x00000000U)
#define CSL_MSS_RCM_USB_XTAL_CLK_GATE_GATED_RESETVAL                            (0x00000000U)
#define CSL_MSS_RCM_USB_XTAL_CLK_GATE_GATED_MAX                                 (0x00000007U)

/* HSM_RTIA_CLK_SRC_SEL */

#define CSL_MSS_RCM_HSM_RTIA_CLK_SRC_SEL_CLKSRCSEL_MASK   (0x00000FFFU)
#define CSL_MSS_RCM_HSM_RTIA_CLK_SRC_SEL_CLKSRCSEL_SHIFT  (0x00000000U)
#define CSL_MSS_RCM_HSM_RTIA_CLK_SRC_SEL_CLKSRCSEL_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_HSM_RTIA_CLK_SRC_SEL_CLKSRCSEL_MAX    (0x00000FFFU)

#define CSL_MSS_RCM_HSM_RTIA_CLK_SRC_SEL_RESETVAL                              (0x00000000U)

/* HSM_WDT_CLK_SRC_SEL */

#define CSL_MSS_RCM_HSM_WDT_CLK_SRC_SEL_CLKSRCSEL_MASK     (0x00000FFFU)
#define CSL_MSS_RCM_HSM_WDT_CLK_SRC_SEL_CLKSRCSEL_SHIFT    (0x00000000U)
#define CSL_MSS_RCM_HSM_WDT_CLK_SRC_SEL_CLKSRCSEL_RESETVAL (0x00000555U)
#define CSL_MSS_RCM_HSM_WDT_CLK_SRC_SEL_CLKSRCSEL_MAX      (0x00000FFFU)

#define CSL_MSS_RCM_HSM_WDT_CLK_SRC_SEL_RESETVAL                               (0x00000555U)

/* HSM_RTC_CLK_SRC_SEL */

#define CSL_MSS_RCM_HSM_RTC_CLK_SRC_SEL_CLKSRCSEL_MASK     (0x00000FFFU)
#define CSL_MSS_RCM_HSM_RTC_CLK_SRC_SEL_CLKSRCSEL_SHIFT    (0x00000000U)
#define CSL_MSS_RCM_HSM_RTC_CLK_SRC_SEL_CLKSRCSEL_RESETVAL (0x00000777U)
#define CSL_MSS_RCM_HSM_RTC_CLK_SRC_SEL_CLKSRCSEL_MAX      (0x00000FFFU)

#define CSL_MSS_RCM_HSM_RTC_CLK_SRC_SEL_RESETVAL                               (0x00000777U)

/* HSM_DMTA_CLK_SRC_SEL */

#define CSL_MSS_RCM_HSM_DMTA_CLK_SRC_SEL_CLKSRCSEL_MASK   (0x00000FFFU)
#define CSL_MSS_RCM_HSM_DMTA_CLK_SRC_SEL_CLKSRCSEL_SHIFT  (0x00000000U)
#define CSL_MSS_RCM_HSM_DMTA_CLK_SRC_SEL_CLKSRCSEL_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_HSM_DMTA_CLK_SRC_SEL_CLKSRCSEL_MAX    (0x00000FFFU)

#define CSL_MSS_RCM_HSM_DMTA_CLK_SRC_SEL_RESETVAL                              (0x00000000U)

/* HSM_DMTB_CLK_SRC_SEL */

#define CSL_MSS_RCM_HSM_DMTB_CLK_SRC_SEL_CLKSRCSEL_MASK   (0x00000FFFU)
#define CSL_MSS_RCM_HSM_DMTB_CLK_SRC_SEL_CLKSRCSEL_SHIFT  (0x00000000U)
#define CSL_MSS_RCM_HSM_DMTB_CLK_SRC_SEL_CLKSRCSEL_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_HSM_DMTB_CLK_SRC_SEL_CLKSRCSEL_MAX    (0x00000FFFU)

#define CSL_MSS_RCM_HSM_DMTB_CLK_SRC_SEL_RESETVAL                              (0x00000000U)

/* HSM_RTI_CLK_DIV_VAL */

#define CSL_MSS_RCM_HSM_RTI_CLK_DIV_VAL_CLKDIVR_MASK       (0x00000FFFU)
#define CSL_MSS_RCM_HSM_RTI_CLK_DIV_VAL_CLKDIVR_SHIFT      (0x00000000U)
#define CSL_MSS_RCM_HSM_RTI_CLK_DIV_VAL_CLKDIVR_RESETVAL   (0x00000000U)
#define CSL_MSS_RCM_HSM_RTI_CLK_DIV_VAL_CLKDIVR_MAX        (0x00000FFFU)

#define CSL_MSS_RCM_HSM_RTI_CLK_DIV_VAL_RESETVAL                               (0x00000000U)

/* HSM_WDT_CLK_DIV_VAL */

#define CSL_MSS_RCM_HSM_WDT_CLK_DIV_VAL_CLKDIVR_MASK       (0x00000FFFU)
#define CSL_MSS_RCM_HSM_WDT_CLK_DIV_VAL_CLKDIVR_SHIFT      (0x00000000U)
#define CSL_MSS_RCM_HSM_WDT_CLK_DIV_VAL_CLKDIVR_RESETVAL   (0x00000000U)
#define CSL_MSS_RCM_HSM_WDT_CLK_DIV_VAL_CLKDIVR_MAX        (0x00000FFFU)

#define CSL_MSS_RCM_HSM_WDT_CLK_DIV_VAL_RESETVAL                               (0x00000000U)

/* HSM_RTC_CLK_DIV_VAL */

#define CSL_MSS_RCM_HSM_RTC_CLK_DIV_VAL_CLKDIVR_MASK       (0x00000FFFU)
#define CSL_MSS_RCM_HSM_RTC_CLK_DIV_VAL_CLKDIVR_SHIFT      (0x00000000U)
#define CSL_MSS_RCM_HSM_RTC_CLK_DIV_VAL_CLKDIVR_RESETVAL   (0x00000000U)
#define CSL_MSS_RCM_HSM_RTC_CLK_DIV_VAL_CLKDIVR_MAX        (0x00000FFFU)

#define CSL_MSS_RCM_HSM_RTC_CLK_DIV_VAL_RESETVAL                               (0x00000000U)

/* HSM_DMTA_CLK_DIV_VAL */

#define CSL_MSS_RCM_HSM_DMTA_CLK_DIV_VAL_CLKDIVR_MASK     (0x00000FFFU)
#define CSL_MSS_RCM_HSM_DMTA_CLK_DIV_VAL_CLKDIVR_SHIFT    (0x00000000U)
#define CSL_MSS_RCM_HSM_DMTA_CLK_DIV_VAL_CLKDIVR_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_HSM_DMTA_CLK_DIV_VAL_CLKDIVR_MAX      (0x00000FFFU)

#define CSL_MSS_RCM_HSM_DMTA_CLK_DIV_VAL_RESETVAL                              (0x00000000U)

/* HSM_DMTB_CLK_DIV_VAL */

#define CSL_MSS_RCM_HSM_DMTB_CLK_DIV_VAL_CLKDIVR_MASK     (0x00000FFFU)
#define CSL_MSS_RCM_HSM_DMTB_CLK_DIV_VAL_CLKDIVR_SHIFT    (0x00000000U)
#define CSL_MSS_RCM_HSM_DMTB_CLK_DIV_VAL_CLKDIVR_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_HSM_DMTB_CLK_DIV_VAL_CLKDIVR_MAX      (0x00000FFFU)

#define CSL_MSS_RCM_HSM_DMTB_CLK_DIV_VAL_RESETVAL                              (0x00000000U)

/* HSM_RTI_CLK_GATE */

#define CSL_MSS_RCM_HSM_RTI_CLK_GATE_GATED_MASK               (0x00000007U)
#define CSL_MSS_RCM_HSM_RTI_CLK_GATE_GATED_SHIFT              (0x00000000U)
#define CSL_MSS_RCM_HSM_RTI_CLK_GATE_GATED_RESETVAL           (0x00000000U)
#define CSL_MSS_RCM_HSM_RTI_CLK_GATE_GATED_MAX                (0x00000007U)

#define CSL_MSS_RCM_HSM_RTI_CLK_GATE_RESETVAL                                  (0x00000000U)

/* HSM_WDT_CLK_GATE */

#define CSL_MSS_RCM_HSM_WDT_CLK_GATE_GATED_MASK               (0x00000007U)
#define CSL_MSS_RCM_HSM_WDT_CLK_GATE_GATED_SHIFT              (0x00000000U)
#define CSL_MSS_RCM_HSM_WDT_CLK_GATE_GATED_RESETVAL           (0x00000007U)
#define CSL_MSS_RCM_HSM_WDT_CLK_GATE_GATED_MAX                (0x00000007U)

#define CSL_MSS_RCM_HSM_WDT_CLK_GATE_RESETVAL                                  (0x00000007U)

/* HSM_RTC_CLK_GATE */

#define CSL_MSS_RCM_HSM_RTC_CLK_GATE_GATED_MASK               (0x00000007U)
#define CSL_MSS_RCM_HSM_RTC_CLK_GATE_GATED_SHIFT              (0x00000000U)
#define CSL_MSS_RCM_HSM_RTC_CLK_GATE_GATED_RESETVAL           (0x00000000U)
#define CSL_MSS_RCM_HSM_RTC_CLK_GATE_GATED_MAX                (0x00000007U)

#define CSL_MSS_RCM_HSM_RTC_CLK_GATE_RESETVAL                                  (0x00000000U)

/* HSM_DMTA_CLK_GATE */

#define CSL_MSS_RCM_HSM_DMTA_CLK_GATE_GATED_MASK             (0x00000007U)
#define CSL_MSS_RCM_HSM_DMTA_CLK_GATE_GATED_SHIFT            (0x00000000U)
#define CSL_MSS_RCM_HSM_DMTA_CLK_GATE_GATED_RESETVAL         (0x00000000U)
#define CSL_MSS_RCM_HSM_DMTA_CLK_GATE_GATED_MAX              (0x00000007U)

#define CSL_MSS_RCM_HSM_DMTA_CLK_GATE_RESETVAL                                 (0x00000000U)

/* HSM_DMTB_CLK_GATE */

#define CSL_MSS_RCM_HSM_DMTB_CLK_GATE_GATED_MASK             (0x00000007U)
#define CSL_MSS_RCM_HSM_DMTB_CLK_GATE_GATED_SHIFT            (0x00000000U)
#define CSL_MSS_RCM_HSM_DMTB_CLK_GATE_GATED_RESETVAL         (0x00000000U)
#define CSL_MSS_RCM_HSM_DMTB_CLK_GATE_GATED_MAX              (0x00000007U)

#define CSL_MSS_RCM_HSM_DMTB_CLK_GATE_RESETVAL                                 (0x00000000U)

/* HSM_RTI_CLK_STATUS */

#define CSL_MSS_RCM_HSM_RTI_CLK_STATUS_CLKINUSE_MASK        (0x000000FFU)
#define CSL_MSS_RCM_HSM_RTI_CLK_STATUS_CLKINUSE_SHIFT       (0x00000000U)
#define CSL_MSS_RCM_HSM_RTI_CLK_STATUS_CLKINUSE_RESETVAL    (0x00000001U)
#define CSL_MSS_RCM_HSM_RTI_CLK_STATUS_CLKINUSE_MAX         (0x000000FFU)

#define CSL_MSS_RCM_HSM_RTI_CLK_STATUS_CURRDIVIDER_MASK     (0x0000FF00U)
#define CSL_MSS_RCM_HSM_RTI_CLK_STATUS_CURRDIVIDER_SHIFT    (0x00000008U)
#define CSL_MSS_RCM_HSM_RTI_CLK_STATUS_CURRDIVIDER_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_HSM_RTI_CLK_STATUS_CURRDIVIDER_MAX      (0x000000FFU)

#define CSL_MSS_RCM_HSM_RTI_CLK_STATUS_RESETVAL                                (0x00000001U)

/* HSM_WDT_CLK_STATUS */

#define CSL_MSS_RCM_HSM_WDT_CLK_STATUS_CLKINUSE_MASK        (0x000000FFU)
#define CSL_MSS_RCM_HSM_WDT_CLK_STATUS_CLKINUSE_SHIFT       (0x00000000U)
#define CSL_MSS_RCM_HSM_WDT_CLK_STATUS_CLKINUSE_RESETVAL    (0x00000020U)
#define CSL_MSS_RCM_HSM_WDT_CLK_STATUS_CLKINUSE_MAX         (0x000000FFU)

#define CSL_MSS_RCM_HSM_WDT_CLK_STATUS_CURRDIVIDER_MASK     (0x0000FF00U)
#define CSL_MSS_RCM_HSM_WDT_CLK_STATUS_CURRDIVIDER_SHIFT    (0x00000008U)
#define CSL_MSS_RCM_HSM_WDT_CLK_STATUS_CURRDIVIDER_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_HSM_WDT_CLK_STATUS_CURRDIVIDER_MAX      (0x000000FFU)

#define CSL_MSS_RCM_HSM_WDT_CLK_STATUS_RESETVAL                                (0x00000020U)

/* HSM_RTC_CLK_STATUS */

#define CSL_MSS_RCM_HSM_RTC_CLK_STATUS_CLKINUSE_MASK        (0x000000FFU)
#define CSL_MSS_RCM_HSM_RTC_CLK_STATUS_CLKINUSE_SHIFT       (0x00000000U)
#define CSL_MSS_RCM_HSM_RTC_CLK_STATUS_CLKINUSE_RESETVAL    (0x00000080U)
#define CSL_MSS_RCM_HSM_RTC_CLK_STATUS_CLKINUSE_MAX         (0x000000FFU)

#define CSL_MSS_RCM_HSM_RTC_CLK_STATUS_CURRDIVIDER_MASK     (0x0000FF00U)
#define CSL_MSS_RCM_HSM_RTC_CLK_STATUS_CURRDIVIDER_SHIFT    (0x00000008U)
#define CSL_MSS_RCM_HSM_RTC_CLK_STATUS_CURRDIVIDER_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_HSM_RTC_CLK_STATUS_CURRDIVIDER_MAX      (0x000000FFU)

#define CSL_MSS_RCM_HSM_RTC_CLK_STATUS_RESETVAL                                (0x00000080U)

/* HSM_DMTA_CLK_STATUS */

#define CSL_MSS_RCM_HSM_DMTA_CLK_STATUS_CLKINUSE_MASK      (0x000000FFU)
#define CSL_MSS_RCM_HSM_DMTA_CLK_STATUS_CLKINUSE_SHIFT     (0x00000000U)
#define CSL_MSS_RCM_HSM_DMTA_CLK_STATUS_CLKINUSE_RESETVAL  (0x00000001U)
#define CSL_MSS_RCM_HSM_DMTA_CLK_STATUS_CLKINUSE_MAX       (0x000000FFU)

#define CSL_MSS_RCM_HSM_DMTA_CLK_STATUS_CURRDIVIDER_MASK   (0x0000FF00U)
#define CSL_MSS_RCM_HSM_DMTA_CLK_STATUS_CURRDIVIDER_SHIFT  (0x00000008U)
#define CSL_MSS_RCM_HSM_DMTA_CLK_STATUS_CURRDIVIDER_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_HSM_DMTA_CLK_STATUS_CURRDIVIDER_MAX    (0x000000FFU)

#define CSL_MSS_RCM_HSM_DMTA_CLK_STATUS_RESETVAL                               (0x00000001U)

/* HSM_DMTB_CLK_STATUS */

#define CSL_MSS_RCM_HSM_DMTB_CLK_STATUS_CLKINUSE_MASK      (0x000000FFU)
#define CSL_MSS_RCM_HSM_DMTB_CLK_STATUS_CLKINUSE_SHIFT     (0x00000000U)
#define CSL_MSS_RCM_HSM_DMTB_CLK_STATUS_CLKINUSE_RESETVAL  (0x00000001U)
#define CSL_MSS_RCM_HSM_DMTB_CLK_STATUS_CLKINUSE_MAX       (0x000000FFU)

#define CSL_MSS_RCM_HSM_DMTB_CLK_STATUS_CURRDIVIDER_MASK   (0x0000FF00U)
#define CSL_MSS_RCM_HSM_DMTB_CLK_STATUS_CURRDIVIDER_SHIFT  (0x00000008U)
#define CSL_MSS_RCM_HSM_DMTB_CLK_STATUS_CURRDIVIDER_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_HSM_DMTB_CLK_STATUS_CURRDIVIDER_MAX    (0x000000FFU)

#define CSL_MSS_RCM_HSM_DMTB_CLK_STATUS_RESETVAL                               (0x00000001U)

/* MSS_MCAN0_CLK_STATUS */

#define CSL_MSS_RCM_MCAN0_CLK_STATUS_CLKINUSE_MASK    (0x000000FFU)
#define CSL_MSS_RCM_MCAN0_CLK_STATUS_CLKINUSE_SHIFT   (0x00000000U)
#define CSL_MSS_RCM_MCAN0_CLK_STATUS_CLKINUSE_RESETVAL (0x00000001U)
#define CSL_MSS_RCM_MCAN0_CLK_STATUS_CLKINUSE_MAX     (0x000000FFU)

#define CSL_MSS_RCM_MCAN0_CLK_STATUS_CURRDIVIDER_MASK (0x0000FF00U)
#define CSL_MSS_RCM_MCAN0_CLK_STATUS_CURRDIVIDER_SHIFT (0x00000008U)
#define CSL_MSS_RCM_MCAN0_CLK_STATUS_CURRDIVIDER_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_MCAN0_CLK_STATUS_CURRDIVIDER_MAX  (0x000000FFU)

#define CSL_MSS_RCM_MCAN0_CLK_STATUS_RESETVAL                              (0x00000001U)

/* MSS_MCAN1_CLK_STATUS */

#define CSL_MSS_RCM_MCAN1_CLK_STATUS_CLKINUSE_MASK    (0x000000FFU)
#define CSL_MSS_RCM_MCAN1_CLK_STATUS_CLKINUSE_SHIFT   (0x00000000U)
#define CSL_MSS_RCM_MCAN1_CLK_STATUS_CLKINUSE_RESETVAL (0x00000001U)
#define CSL_MSS_RCM_MCAN1_CLK_STATUS_CLKINUSE_MAX     (0x000000FFU)

#define CSL_MSS_RCM_MCAN1_CLK_STATUS_CURRDIVIDER_MASK (0x0000FF00U)
#define CSL_MSS_RCM_MCAN1_CLK_STATUS_CURRDIVIDER_SHIFT (0x00000008U)
#define CSL_MSS_RCM_MCAN1_CLK_STATUS_CURRDIVIDER_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_MCAN1_CLK_STATUS_CURRDIVIDER_MAX  (0x000000FFU)

#define CSL_MSS_RCM_MCAN1_CLK_STATUS_RESETVAL                              (0x00000001U)

/* MSS_RTI0_CLK_STATUS */

#define CSL_MSS_RCM_RTI0_CLK_STATUS_CLKINUSE_MASK      (0x000000FFU)
#define CSL_MSS_RCM_RTI0_CLK_STATUS_CLKINUSE_SHIFT     (0x00000000U)
#define CSL_MSS_RCM_RTI0_CLK_STATUS_CLKINUSE_RESETVAL  (0x00000001U)
#define CSL_MSS_RCM_RTI0_CLK_STATUS_CLKINUSE_MAX       (0x000000FFU)

#define CSL_MSS_RCM_RTI0_CLK_STATUS_CURRDIVIDER_MASK   (0x0000FF00U)
#define CSL_MSS_RCM_RTI0_CLK_STATUS_CURRDIVIDER_SHIFT  (0x00000008U)
#define CSL_MSS_RCM_RTI0_CLK_STATUS_CURRDIVIDER_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_RTI0_CLK_STATUS_CURRDIVIDER_MAX    (0x000000FFU)

#define CSL_MSS_RCM_RTI0_CLK_STATUS_RESETVAL                               (0x00000001U)

/* MSS_RTI1_CLK_STATUS */

#define CSL_MSS_RCM_RTI1_CLK_STATUS_CLKINUSE_MASK      (0x000000FFU)
#define CSL_MSS_RCM_RTI1_CLK_STATUS_CLKINUSE_SHIFT     (0x00000000U)
#define CSL_MSS_RCM_RTI1_CLK_STATUS_CLKINUSE_RESETVAL  (0x00000001U)
#define CSL_MSS_RCM_RTI1_CLK_STATUS_CLKINUSE_MAX       (0x000000FFU)

#define CSL_MSS_RCM_RTI1_CLK_STATUS_CURRDIVIDER_MASK   (0x0000FF00U)
#define CSL_MSS_RCM_RTI1_CLK_STATUS_CURRDIVIDER_SHIFT  (0x00000008U)
#define CSL_MSS_RCM_RTI1_CLK_STATUS_CURRDIVIDER_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_RTI1_CLK_STATUS_CURRDIVIDER_MAX    (0x000000FFU)

#define CSL_MSS_RCM_RTI1_CLK_STATUS_RESETVAL                               (0x00000001U)

/* MSS_RTI2_CLK_STATUS */

#define CSL_MSS_RCM_RTI2_CLK_STATUS_CLKINUSE_MASK      (0x000000FFU)
#define CSL_MSS_RCM_RTI2_CLK_STATUS_CLKINUSE_SHIFT     (0x00000000U)
#define CSL_MSS_RCM_RTI2_CLK_STATUS_CLKINUSE_RESETVAL  (0x00000001U)
#define CSL_MSS_RCM_RTI2_CLK_STATUS_CLKINUSE_MAX       (0x000000FFU)

#define CSL_MSS_RCM_RTI2_CLK_STATUS_CURRDIVIDER_MASK   (0x0000FF00U)
#define CSL_MSS_RCM_RTI2_CLK_STATUS_CURRDIVIDER_SHIFT  (0x00000008U)
#define CSL_MSS_RCM_RTI2_CLK_STATUS_CURRDIVIDER_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_RTI2_CLK_STATUS_CURRDIVIDER_MAX    (0x000000FFU)

#define CSL_MSS_RCM_RTI2_CLK_STATUS_RESETVAL                               (0x00000001U)

/* MSS_RTI3_CLK_STATUS */

#define CSL_MSS_RCM_RTI3_CLK_STATUS_CLKINUSE_MASK      (0x000000FFU)
#define CSL_MSS_RCM_RTI3_CLK_STATUS_CLKINUSE_SHIFT     (0x00000000U)
#define CSL_MSS_RCM_RTI3_CLK_STATUS_CLKINUSE_RESETVAL  (0x00000001U)
#define CSL_MSS_RCM_RTI3_CLK_STATUS_CLKINUSE_MAX       (0x000000FFU)

#define CSL_MSS_RCM_RTI3_CLK_STATUS_CURRDIVIDER_MASK   (0x0000FF00U)
#define CSL_MSS_RCM_RTI3_CLK_STATUS_CURRDIVIDER_SHIFT  (0x00000008U)
#define CSL_MSS_RCM_RTI3_CLK_STATUS_CURRDIVIDER_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_RTI3_CLK_STATUS_CURRDIVIDER_MAX    (0x000000FFU)

#define CSL_MSS_RCM_RTI3_CLK_STATUS_RESETVAL                               (0x00000001U)

/* MSS_MCSPI0_CLK_STATUS */

#define CSL_MSS_RCM_MCSPI0_CLK_STATUS_CLKINUSE_MASK      (0x000000FFU)
#define CSL_MSS_RCM_MCSPI0_CLK_STATUS_CLKINUSE_SHIFT     (0x00000000U)
#define CSL_MSS_RCM_MCSPI0_CLK_STATUS_CLKINUSE_RESETVAL  (0x00000001U)
#define CSL_MSS_RCM_MCSPI0_CLK_STATUS_CLKINUSE_MAX       (0x000000FFU)

#define CSL_MSS_RCM_MCSPI0_CLK_STATUS_CURRDIVIDER_MASK   (0x0000FF00U)
#define CSL_MSS_RCM_MCSPI0_CLK_STATUS_CURRDIVIDER_SHIFT  (0x00000008U)
#define CSL_MSS_RCM_MCSPI0_CLK_STATUS_CURRDIVIDER_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_MCSPI0_CLK_STATUS_CURRDIVIDER_MAX    (0x000000FFU)

#define CSL_MSS_RCM_MCSPI0_CLK_STATUS_RESETVAL                               (0x00000001U)

/* MSS_MCSPI1_CLK_STATUS */

#define CSL_MSS_RCM_MCSPI1_CLK_STATUS_CLKINUSE_MASK      (0x000000FFU)
#define CSL_MSS_RCM_MCSPI1_CLK_STATUS_CLKINUSE_SHIFT     (0x00000000U)
#define CSL_MSS_RCM_MCSPI1_CLK_STATUS_CLKINUSE_RESETVAL  (0x00000001U)
#define CSL_MSS_RCM_MCSPI1_CLK_STATUS_CLKINUSE_MAX       (0x000000FFU)

#define CSL_MSS_RCM_MCSPI1_CLK_STATUS_CURRDIVIDER_MASK   (0x0000FF00U)
#define CSL_MSS_RCM_MCSPI1_CLK_STATUS_CURRDIVIDER_SHIFT  (0x00000008U)
#define CSL_MSS_RCM_MCSPI1_CLK_STATUS_CURRDIVIDER_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_MCSPI1_CLK_STATUS_CURRDIVIDER_MAX    (0x000000FFU)

#define CSL_MSS_RCM_MCSPI1_CLK_STATUS_RESETVAL                               (0x00000001U)

/* MSS_MCSPI2_CLK_STATUS */

#define CSL_MSS_RCM_MCSPI2_CLK_STATUS_CLKINUSE_MASK      (0x000000FFU)
#define CSL_MSS_RCM_MCSPI2_CLK_STATUS_CLKINUSE_SHIFT     (0x00000000U)
#define CSL_MSS_RCM_MCSPI2_CLK_STATUS_CLKINUSE_RESETVAL  (0x00000001U)
#define CSL_MSS_RCM_MCSPI2_CLK_STATUS_CLKINUSE_MAX       (0x000000FFU)

#define CSL_MSS_RCM_MCSPI2_CLK_STATUS_CURRDIVIDER_MASK   (0x0000FF00U)
#define CSL_MSS_RCM_MCSPI2_CLK_STATUS_CURRDIVIDER_SHIFT  (0x00000008U)
#define CSL_MSS_RCM_MCSPI2_CLK_STATUS_CURRDIVIDER_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_MCSPI2_CLK_STATUS_CURRDIVIDER_MAX    (0x000000FFU)

#define CSL_MSS_RCM_MCSPI2_CLK_STATUS_RESETVAL                               (0x00000001U)

/* MSS_MCSPI3_CLK_STATUS */

#define CSL_MSS_RCM_MCSPI3_CLK_STATUS_CLKINUSE_MASK      (0x000000FFU)
#define CSL_MSS_RCM_MCSPI3_CLK_STATUS_CLKINUSE_SHIFT     (0x00000000U)
#define CSL_MSS_RCM_MCSPI3_CLK_STATUS_CLKINUSE_RESETVAL  (0x00000001U)
#define CSL_MSS_RCM_MCSPI3_CLK_STATUS_CLKINUSE_MAX       (0x000000FFU)

#define CSL_MSS_RCM_MCSPI3_CLK_STATUS_CURRDIVIDER_MASK   (0x0000FF00U)
#define CSL_MSS_RCM_MCSPI3_CLK_STATUS_CURRDIVIDER_SHIFT  (0x00000008U)
#define CSL_MSS_RCM_MCSPI3_CLK_STATUS_CURRDIVIDER_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_MCSPI3_CLK_STATUS_CURRDIVIDER_MAX    (0x000000FFU)

#define CSL_MSS_RCM_MCSPI3_CLK_STATUS_RESETVAL                               (0x00000001U)

/* MSS_WDT0_CLK_STATUS */

#define CSL_MSS_RCM_WDT0_CLK_STATUS_CLKINUSE_MASK      (0x000000FFU)
#define CSL_MSS_RCM_WDT0_CLK_STATUS_CLKINUSE_SHIFT     (0x00000000U)
#define CSL_MSS_RCM_WDT0_CLK_STATUS_CLKINUSE_RESETVAL  (0x00000001U)
#define CSL_MSS_RCM_WDT0_CLK_STATUS_CLKINUSE_MAX       (0x000000FFU)

#define CSL_MSS_RCM_WDT0_CLK_STATUS_CURRDIVIDER_MASK   (0x0000FF00U)
#define CSL_MSS_RCM_WDT0_CLK_STATUS_CURRDIVIDER_SHIFT  (0x00000008U)
#define CSL_MSS_RCM_WDT0_CLK_STATUS_CURRDIVIDER_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_WDT0_CLK_STATUS_CURRDIVIDER_MAX    (0x000000FFU)

#define CSL_MSS_RCM_WDT0_CLK_STATUS_RESETVAL                               (0x00000001U)

/* MSS_WDT1_CLK_STATUS */

#define CSL_MSS_RCM_WDT1_CLK_STATUS_CLKINUSE_MASK      (0x000000FFU)
#define CSL_MSS_RCM_WDT1_CLK_STATUS_CLKINUSE_SHIFT     (0x00000000U)
#define CSL_MSS_RCM_WDT1_CLK_STATUS_CLKINUSE_RESETVAL  (0x00000001U)
#define CSL_MSS_RCM_WDT1_CLK_STATUS_CLKINUSE_MAX       (0x000000FFU)

#define CSL_MSS_RCM_WDT1_CLK_STATUS_CURRDIVIDER_MASK   (0x0000FF00U)
#define CSL_MSS_RCM_WDT1_CLK_STATUS_CURRDIVIDER_SHIFT  (0x00000008U)
#define CSL_MSS_RCM_WDT1_CLK_STATUS_CURRDIVIDER_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_WDT1_CLK_STATUS_CURRDIVIDER_MAX    (0x000000FFU)

#define CSL_MSS_RCM_WDT1_CLK_STATUS_RESETVAL                               (0x00000001U)

/* ICSSM0_UART_CLK_STATUS */

#define CSL_MSS_RCM_ICSSM0_UART_CLK_STATUS_CLKINUSE_MASK (0x000000FFU)
#define CSL_MSS_RCM_ICSSM0_UART_CLK_STATUS_CLKINUSE_SHIFT (0x00000000U)
#define CSL_MSS_RCM_ICSSM0_UART_CLK_STATUS_CLKINUSE_RESETVAL (0x00000001U)
#define CSL_MSS_RCM_ICSSM0_UART_CLK_STATUS_CLKINUSE_MAX (0x000000FFU)

#define CSL_MSS_RCM_ICSSM0_UART_CLK_STATUS_CURRDIVIDER_MASK (0x0000FF00U)
#define CSL_MSS_RCM_ICSSM0_UART_CLK_STATUS_CURRDIVIDER_SHIFT (0x00000008U)
#define CSL_MSS_RCM_ICSSM0_UART_CLK_STATUS_CURRDIVIDER_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_ICSSM0_UART_CLK_STATUS_CURRDIVIDER_MAX (0x000000FFU)

#define CSL_MSS_RCM_ICSSM0_UART_CLK_STATUS_RESETVAL                            (0x00000001U)

/* ICSSM1_UART_CLK_STATUS */

#define CSL_MSS_RCM_ICSSM1_UART_CLK_STATUS_CLKINUSE_MASK (0x000000FFU)
#define CSL_MSS_RCM_ICSSM1_UART_CLK_STATUS_CLKINUSE_SHIFT (0x00000000U)
#define CSL_MSS_RCM_ICSSM1_UART_CLK_STATUS_CLKINUSE_RESETVAL (0x00000001U)
#define CSL_MSS_RCM_ICSSM1_UART_CLK_STATUS_CLKINUSE_MAX (0x000000FFU)

#define CSL_MSS_RCM_ICSSM1_UART_CLK_STATUS_CURRDIVIDER_MASK (0x0000FF00U)
#define CSL_MSS_RCM_ICSSM1_UART_CLK_STATUS_CURRDIVIDER_SHIFT (0x00000008U)
#define CSL_MSS_RCM_ICSSM1_UART_CLK_STATUS_CURRDIVIDER_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_ICSSM1_UART_CLK_STATUS_CURRDIVIDER_MAX (0x000000FFU)

#define CSL_MSS_RCM_ICSSM1_UART_CLK_STATUS_RESETVAL                            (0x00000001U)

/* OSPI0_CLK_STATUS */

#define CSL_MSS_RCM_OSPI0_CLK_STATUS_CLKINUSE_MASK    (0x000000FFU)
#define CSL_MSS_RCM_OSPI0_CLK_STATUS_CLKINUSE_SHIFT   (0x00000000U)
#define CSL_MSS_RCM_OSPI0_CLK_STATUS_CLKINUSE_RESETVAL (0x00000001U)
#define CSL_MSS_RCM_OSPI0_CLK_STATUS_CLKINUSE_MAX     (0x000000FFU)

#define CSL_MSS_RCM_OSPI0_CLK_STATUS_CURRDIVIDER_MASK (0x0000FF00U)
#define CSL_MSS_RCM_OSPI0_CLK_STATUS_CURRDIVIDER_SHIFT (0x00000008U)
#define CSL_MSS_RCM_OSPI0_CLK_STATUS_CURRDIVIDER_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_OSPI0_CLK_STATUS_CURRDIVIDER_MAX  (0x000000FFU)


/* OSPI1_CLK_STATUS */
#define CSL_MSS_RCM_OSPI1_CLK_STATUS_CLKINUSE_MASK                          (0x000000FFU)
#define CSL_MSS_RCM_OSPI1_CLK_STATUS_CLKINUSE_SHIFT                         (0x00000000U)
#define CSL_MSS_RCM_OSPI1_CLK_STATUS_CLKINUSE_RESETVAL                      (0x00000001U)
#define CSL_MSS_RCM_OSPI1_CLK_STATUS_CLKINUSE_MAX                           (0x000000FFU)


#define CSL_MSS_RCM_OSPI1_CLK_STATUS_CURRDIVIDER_MASK                       (0x0000FF00U)
#define CSL_MSS_RCM_OSPI1_CLK_STATUS_CURRDIVIDER_SHIFT                      (0x00000008U)
#define CSL_MSS_RCM_OSPI1_CLK_STATUS_CURRDIVIDER_RESETVAL                   (0x00000000U)
#define CSL_MSS_RCM_OSPI1_CLK_STATUS_CURRDIVIDER_MAX                        (0x000000FFU)


/* CONTROLSS_PLL_CLK_STATUS */

#define CSL_MSS_RCM_CONTROLSS_PLL_CLK_STATUS_CLKINUSE_MASK (0x000000FFU)
#define CSL_MSS_RCM_CONTROLSS_PLL_CLK_STATUS_CLKINUSE_SHIFT (0x00000000U)
#define CSL_MSS_RCM_CONTROLSS_PLL_CLK_STATUS_CLKINUSE_RESETVAL (0x00000001U)
#define CSL_MSS_RCM_CONTROLSS_PLL_CLK_STATUS_CLKINUSE_MAX (0x000000FFU)

#define CSL_MSS_RCM_CONTROLSS_PLL_CLK_STATUS_CURRDIVIDER_MASK (0x0000FF00U)
#define CSL_MSS_RCM_CONTROLSS_PLL_CLK_STATUS_CURRDIVIDER_SHIFT (0x00000008U)
#define CSL_MSS_RCM_CONTROLSS_PLL_CLK_STATUS_CURRDIVIDER_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_CONTROLSS_PLL_CLK_STATUS_CURRDIVIDER_MAX (0x000000FFU)

#define CSL_MSS_RCM_CONTROLSS_PLL_CLK_STATUS_RESETVAL                          (0x00000001U)

/* MSS_CPTS_CLK_STATUS */

#define CSL_MSS_RCM_CPTS_CLK_STATUS_CLKINUSE_MASK      (0x000000FFU)
#define CSL_MSS_RCM_CPTS_CLK_STATUS_CLKINUSE_SHIFT     (0x00000000U)
#define CSL_MSS_RCM_CPTS_CLK_STATUS_CLKINUSE_RESETVAL  (0x00000001U)
#define CSL_MSS_RCM_CPTS_CLK_STATUS_CLKINUSE_MAX       (0x000000FFU)

#define CSL_MSS_RCM_CPTS_CLK_STATUS_CURRDIVIDER_MASK   (0x0000FF00U)
#define CSL_MSS_RCM_CPTS_CLK_STATUS_CURRDIVIDER_SHIFT  (0x00000008U)
#define CSL_MSS_RCM_CPTS_CLK_STATUS_CURRDIVIDER_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_CPTS_CLK_STATUS_CURRDIVIDER_MAX    (0x000000FFU)

#define CSL_MSS_RCM_CPTS_CLK_STATUS_RESETVAL                               (0x00000001U)

/* GPMC_CLK_STATUS */

#define CSL_MSS_RCM_GPMC_CLK_STATUS_CLKINUSE_MASK              (0x000000FFU)
#define CSL_MSS_RCM_GPMC_CLK_STATUS_CLKINUSE_SHIFT             (0x00000000U)
#define CSL_MSS_RCM_GPMC_CLK_STATUS_CLKINUSE_RESETVAL          (0x00000001U)
#define CSL_MSS_RCM_GPMC_CLK_STATUS_CLKINUSE_MAX               (0x000000FFU)

#define CSL_MSS_RCM_GPMC_CLK_STATUS_CURRDIVIDER_MASK           (0x0000FF00U)
#define CSL_MSS_RCM_GPMC_CLK_STATUS_CURRDIVIDER_SHIFT          (0x00000008U)
#define CSL_MSS_RCM_GPMC_CLK_STATUS_CURRDIVIDER_RESETVAL       (0x00000000U)
#define CSL_MSS_RCM_GPMC_CLK_STATUS_CURRDIVIDER_MAX            (0x000000FFU)

#define CSL_MSS_RCM_GPMC_CLK_STATUS_RESETVAL                                   (0x00000001U)

/* MSS_MMC0_CLK_STATUS */

#define CSL_MSS_RCM_MMC0_CLK_STATUS_CLKINUSE_MASK    (0x000000FFU)
#define CSL_MSS_RCM_MMC0_CLK_STATUS_CLKINUSE_SHIFT   (0x00000000U)
#define CSL_MSS_RCM_MMC0_CLK_STATUS_CLKINUSE_RESETVAL (0x00000001U)
#define CSL_MSS_RCM_MMC0_CLK_STATUS_CLKINUSE_MAX     (0x000000FFU)

#define CSL_MSS_RCM_MMC0_CLK_STATUS_CURRDIVIDER_MASK (0x0000FF00U)
#define CSL_MSS_RCM_MMC0_CLK_STATUS_CURRDIVIDER_SHIFT (0x00000008U)
#define CSL_MSS_RCM_MMC0_CLK_STATUS_CURRDIVIDER_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_MMC0_CLK_STATUS_CURRDIVIDER_MAX  (0x000000FFU)

#define CSL_MSS_RCM_MMC0_CLK_STATUS_RESETVAL                              (0x00000001U)

/* MSS_ELM_CLK_STATUS */

#define CSL_MSS_RCM_MSS_ELM_CLK_STATUS_CURRDIVIDER_MASK     (0x0000FF00U)
#define CSL_MSS_RCM_MSS_ELM_CLK_STATUS_CURRDIVIDER_SHIFT    (0x00000008U)
#define CSL_MSS_RCM_MSS_ELM_CLK_STATUS_CURRDIVIDER_RESETVAL (0x00000003U)
#define CSL_MSS_RCM_MSS_ELM_CLK_STATUS_CURRDIVIDER_MAX      (0x000000FFU)

#define CSL_MSS_RCM_MSS_ELM_CLK_STATUS_RESETVAL                                (0x00000300U)

/* MSS_MII10_CLK_STATUS */

#define CSL_MSS_RCM_MII10_CLK_STATUS_CURRDIVIDER_MASK (0x0000FF00U)
#define CSL_MSS_RCM_MII10_CLK_STATUS_CURRDIVIDER_SHIFT (0x00000008U)
#define CSL_MSS_RCM_MII10_CLK_STATUS_CURRDIVIDER_RESETVAL (0x00000063U)
#define CSL_MSS_RCM_MII10_CLK_STATUS_CURRDIVIDER_MAX  (0x000000FFU)

#define CSL_MSS_RCM_MII10_CLK_STATUS_RESETVAL                              (0x00006300U)

/* MSS_MII100_CLK_STATUS */

#define CSL_MSS_RCM_MII100_CLK_STATUS_CURRDIVIDER_MASK (0x0000FF00U)
#define CSL_MSS_RCM_MII100_CLK_STATUS_CURRDIVIDER_SHIFT (0x00000008U)
#define CSL_MSS_RCM_MII100_CLK_STATUS_CURRDIVIDER_RESETVAL (0x00000009U)
#define CSL_MSS_RCM_MII100_CLK_STATUS_CURRDIVIDER_MAX (0x000000FFU)

#define CSL_MSS_RCM_MII100_CLK_STATUS_RESETVAL                             (0x00000900U)

/* MSS_RGMII_CLK_STATUS */

#define CSL_MSS_RCM_RGMII_CLK_STATUS_CURRDIVIDER_MASK (0x0000FF00U)
#define CSL_MSS_RCM_RGMII_CLK_STATUS_CURRDIVIDER_SHIFT (0x00000008U)
#define CSL_MSS_RCM_RGMII_CLK_STATUS_CURRDIVIDER_RESETVAL (0x00000001U)
#define CSL_MSS_RCM_RGMII_CLK_STATUS_CURRDIVIDER_MAX  (0x000000FFU)

#define CSL_MSS_RCM_RGMII_CLK_STATUS_RESETVAL                              (0x00000100U)

/* MMC0_32K_CLK_STATUS */

#define CSL_MSS_RCM_MMC0_32K_CLK_STATUS_CURRDIVIDER_MASK   (0x0003FF00U)
#define CSL_MSS_RCM_MMC0_32K_CLK_STATUS_CURRDIVIDER_SHIFT  (0x00000008U)
#define CSL_MSS_RCM_MMC0_32K_CLK_STATUS_CURRDIVIDER_RESETVAL (0x0000030CU)
#define CSL_MSS_RCM_MMC0_32K_CLK_STATUS_CURRDIVIDER_MAX    (0x000003FFU)

#define CSL_MSS_RCM_MMC0_32K_CLK_STATUS_RESETVAL                               (0x00030C00U)

/* TEMPSENSE_32K_CLK_STATUS */

#define CSL_MSS_RCM_TEMPSENSE_32K_CLK_STATUS_CURRDIVIDER_MASK (0x0003FF00U)
#define CSL_MSS_RCM_TEMPSENSE_32K_CLK_STATUS_CURRDIVIDER_SHIFT (0x00000008U)
#define CSL_MSS_RCM_TEMPSENSE_32K_CLK_STATUS_CURRDIVIDER_RESETVAL (0x0000030CU)
#define CSL_MSS_RCM_TEMPSENSE_32K_CLK_STATUS_CURRDIVIDER_MAX (0x000003FFU)



/* CPSW_5_50_250_CLK_STATUS */
#define CSL_MSS_RCM_CPSW_5_50_250_CLK_STATUS_CLKINUSE_MASK                      (0x000000FFU)
#define CSL_MSS_RCM_CPSW_5_50_250_CLK_STATUS_CLKINUSE_SHIFT                     (0x00000000U)
#define CSL_MSS_RCM_CPSW_5_50_250_CLK_STATUS_CLKINUSE_RESETVAL                  (0x00000001U)
#define CSL_MSS_RCM_CPSW_5_50_250_CLK_STATUS_CLKINUSE_MAX                       (0x000000FFU)

/* MSS_I2C_CLK_STATUS */

#define CSL_MSS_RCM_I2C_CLK_STATUS_CLKINUSE_MASK        (0x000000FFU)
#define CSL_MSS_RCM_I2C_CLK_STATUS_CLKINUSE_SHIFT       (0x00000000U)
#define CSL_MSS_RCM_I2C_CLK_STATUS_CLKINUSE_RESETVAL    (0x00000001U)
#define CSL_MSS_RCM_I2C_CLK_STATUS_CLKINUSE_MAX         (0x000000FFU)

#define CSL_MSS_RCM_I2C_CLK_STATUS_CURRDIVIDER_MASK     (0x0000FF00U)
#define CSL_MSS_RCM_I2C_CLK_STATUS_CURRDIVIDER_SHIFT    (0x00000008U)
#define CSL_MSS_RCM_I2C_CLK_STATUS_CURRDIVIDER_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_I2C_CLK_STATUS_CURRDIVIDER_MAX      (0x000000FFU)

#define CSL_MSS_RCM_I2C_CLK_STATUS_RESETVAL                                (0x00000001U)

/* LIN0_UART0_CLK_STATUS */

#define CSL_MSS_RCM_LIN0_UART0_CLK_STATUS_CLKINUSE_MASK (0x000000FFU)
#define CSL_MSS_RCM_LIN0_UART0_CLK_STATUS_CLKINUSE_SHIFT (0x00000000U)
#define CSL_MSS_RCM_LIN0_UART0_CLK_STATUS_CLKINUSE_RESETVAL (0x00000001U)
#define CSL_MSS_RCM_LIN0_UART0_CLK_STATUS_CLKINUSE_MAX (0x000000FFU)

#define CSL_MSS_RCM_LIN0_UART0_CLK_STATUS_CURRDIVIDER_MASK (0x0000FF00U)
#define CSL_MSS_RCM_LIN0_UART0_CLK_STATUS_CURRDIVIDER_SHIFT (0x00000008U)
#define CSL_MSS_RCM_LIN0_UART0_CLK_STATUS_CURRDIVIDER_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_LIN0_UART0_CLK_STATUS_CURRDIVIDER_MAX (0x000000FFU)

#define CSL_MSS_RCM_LIN0_UART0_CLK_STATUS_RESETVAL                         (0x00000001U)

/* LIN1_UART1_CLK_STATUS */

#define CSL_MSS_RCM_LIN1_UART1_CLK_STATUS_CLKINUSE_MASK (0x000000FFU)
#define CSL_MSS_RCM_LIN1_UART1_CLK_STATUS_CLKINUSE_SHIFT (0x00000000U)
#define CSL_MSS_RCM_LIN1_UART1_CLK_STATUS_CLKINUSE_RESETVAL (0x00000001U)
#define CSL_MSS_RCM_LIN1_UART1_CLK_STATUS_CLKINUSE_MAX (0x000000FFU)

#define CSL_MSS_RCM_LIN1_UART1_CLK_STATUS_CURRDIVIDER_MASK (0x0000FF00U)
#define CSL_MSS_RCM_LIN1_UART1_CLK_STATUS_CURRDIVIDER_SHIFT (0x00000008U)
#define CSL_MSS_RCM_LIN1_UART1_CLK_STATUS_CURRDIVIDER_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_LIN1_UART1_CLK_STATUS_CURRDIVIDER_MAX (0x000000FFU)

#define CSL_MSS_RCM_LIN1_UART1_CLK_STATUS_RESETVAL                         (0x00000001U)

/* LIN2_UART2_CLK_STATUS */

#define CSL_MSS_RCM_LIN2_UART2_CLK_STATUS_CLKINUSE_MASK (0x000000FFU)
#define CSL_MSS_RCM_LIN2_UART2_CLK_STATUS_CLKINUSE_SHIFT (0x00000000U)
#define CSL_MSS_RCM_LIN2_UART2_CLK_STATUS_CLKINUSE_RESETVAL (0x00000001U)
#define CSL_MSS_RCM_LIN2_UART2_CLK_STATUS_CLKINUSE_MAX (0x000000FFU)

#define CSL_MSS_RCM_LIN2_UART2_CLK_STATUS_CURRDIVIDER_MASK (0x0000FF00U)
#define CSL_MSS_RCM_LIN2_UART2_CLK_STATUS_CURRDIVIDER_SHIFT (0x00000008U)
#define CSL_MSS_RCM_LIN2_UART2_CLK_STATUS_CURRDIVIDER_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_LIN2_UART2_CLK_STATUS_CURRDIVIDER_MAX (0x000000FFU)

#define CSL_MSS_RCM_LIN2_UART2_CLK_STATUS_RESETVAL                         (0x00000001U)

/* LIN3_UART3_CLK_STATUS */

#define CSL_MSS_RCM_LIN3_UART3_CLK_STATUS_CLKINUSE_MASK (0x000000FFU)
#define CSL_MSS_RCM_LIN3_UART3_CLK_STATUS_CLKINUSE_SHIFT (0x00000000U)
#define CSL_MSS_RCM_LIN3_UART3_CLK_STATUS_CLKINUSE_RESETVAL (0x00000001U)
#define CSL_MSS_RCM_LIN3_UART3_CLK_STATUS_CLKINUSE_MAX (0x000000FFU)

#define CSL_MSS_RCM_LIN3_UART3_CLK_STATUS_CURRDIVIDER_MASK (0x0000FF00U)
#define CSL_MSS_RCM_LIN3_UART3_CLK_STATUS_CURRDIVIDER_SHIFT (0x00000008U)
#define CSL_MSS_RCM_LIN3_UART3_CLK_STATUS_CURRDIVIDER_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_LIN3_UART3_CLK_STATUS_CURRDIVIDER_MAX (0x000000FFU)

#define CSL_MSS_RCM_LIN3_UART3_CLK_STATUS_RESETVAL                         (0x00000001U)


/* LIN4_UART4_CLK_STATUS */
#define CSL_MSS_RCM_LIN4_UART4_CLK_STATUS_CLKINUSE_MASK                     (0x000000FFU)
#define CSL_MSS_RCM_LIN4_UART4_CLK_STATUS_CLKINUSE_SHIFT                    (0x00000000U)
#define CSL_MSS_RCM_LIN4_UART4_CLK_STATUS_CLKINUSE_RESETVAL                 (0x00000001U)
#define CSL_MSS_RCM_LIN4_UART4_CLK_STATUS_CLKINUSE_MAX                      (0x000000FFU)


#define CSL_MSS_RCM_LIN4_UART4_CLK_STATUS_CURRDIVIDER_MASK                  (0x0000FF00U)
#define CSL_MSS_RCM_LIN4_UART4_CLK_STATUS_CURRDIVIDER_SHIFT                 (0x00000008U)
#define CSL_MSS_RCM_LIN4_UART4_CLK_STATUS_CURRDIVIDER_RESETVAL              (0x00000000U)
#define CSL_MSS_RCM_LIN4_UART4_CLK_STATUS_CURRDIVIDER_MAX                   (0x000000FFU)

#define CSL_MSS_RCM_LIN4_UART4_CLK_STATUS_RESETVAL                         (0x00000001U)



/* LIN5_UART5_CLK_STATUS */
#define CSL_MSS_RCM_LIN5_UART5_CLK_STATUS_CLKINUSE_MASK                     (0x000000FFU)
#define CSL_MSS_RCM_LIN5_UART5_CLK_STATUS_CLKINUSE_SHIFT                    (0x00000000U)
#define CSL_MSS_RCM_LIN5_UART5_CLK_STATUS_CLKINUSE_RESETVAL                 (0x00000001U)
#define CSL_MSS_RCM_LIN5_UART5_CLK_STATUS_CLKINUSE_MAX                      (0x000000FFU)


#define CSL_MSS_RCM_LIN5_UART5_CLK_STATUS_CURRDIVIDER_MASK                  (0x0000FF00U)
#define CSL_MSS_RCM_LIN5_UART5_CLK_STATUS_CURRDIVIDER_SHIFT                 (0x00000008U)
#define CSL_MSS_RCM_LIN5_UART5_CLK_STATUS_CURRDIVIDER_RESETVAL              (0x00000000U)
#define CSL_MSS_RCM_LIN5_UART5_CLK_STATUS_CURRDIVIDER_MAX                   (0x000000FFU)

#define CSL_MSS_RCM_LIN5_UART5_CLK_STATUS_RESETVAL                         (0x00000001U)


/* ICSSM0_CORE_CLK_STATUS */
#define CSL_MSS_RCM_ICSSM0_CORE_CLK_STATUS_CLKINUSE_MASK              (0x000000FFU)
#define CSL_MSS_RCM_ICSSM0_CORE_CLK_STATUS_CLKINUSE_SHIFT             (0x00000000U)
#define CSL_MSS_RCM_ICSSM0_CORE_CLK_STATUS_CLKINUSE_RESETVAL          (0x00000001U)
#define CSL_MSS_RCM_ICSSM0_CORE_CLK_STATUS_CLKINUSE_MAX               (0x000000FFU)


#define CSL_MSS_RCM_ICSSM0_CORE_CLK_STATUS_CURRDIVIDER_MASK           (0x0000FF00U)
#define CSL_MSS_RCM_ICSSM0_CORE_CLK_STATUS_CURRDIVIDER_SHIFT          (0x00000008U)
#define CSL_MSS_RCM_ICSSM0_CORE_CLK_STATUS_CURRDIVIDER_RESETVAL       (0x00000000U)
#define CSL_MSS_RCM_ICSSM0_CORE_CLK_STATUS_CURRDIVIDER_MAX            (0x000000FFU)



/* ICSSM1_CORE_CLK_STATUS */
#define CSL_MSS_RCM_ICSSM1_CORE_CLK_STATUS_CLKINUSE_MASK              (0x000000FFU)
#define CSL_MSS_RCM_ICSSM1_CORE_CLK_STATUS_CLKINUSE_SHIFT             (0x00000000U)
#define CSL_MSS_RCM_ICSSM1_CORE_CLK_STATUS_CLKINUSE_RESETVAL          (0x00000001U)
#define CSL_MSS_RCM_ICSSM1_CORE_CLK_STATUS_CLKINUSE_MAX               (0x000000FFU)


#define CSL_MSS_RCM_ICSSM1_CORE_CLK_STATUS_CURRDIVIDER_MASK           (0x0000FF00U)
#define CSL_MSS_RCM_ICSSM1_CORE_CLK_STATUS_CURRDIVIDER_SHIFT          (0x00000008U)
#define CSL_MSS_RCM_ICSSM1_CORE_CLK_STATUS_CURRDIVIDER_RESETVAL       (0x00000000U)
#define CSL_MSS_RCM_ICSSM1_CORE_CLK_STATUS_CURRDIVIDER_MAX            (0x000000FFU)

/* MSS_MCAN0_RST_CTRL */

#define CSL_MSS_RCM_MCAN0_RST_CTRL_ASSERT_MASK          (0x00000007U)
#define CSL_MSS_RCM_MCAN0_RST_CTRL_ASSERT_SHIFT         (0x00000000U)
#define CSL_MSS_RCM_MCAN0_RST_CTRL_ASSERT_RESETVAL      (0x00000000U)
#define CSL_MSS_RCM_MCAN0_RST_CTRL_ASSERT_MAX           (0x00000007U)

#define CSL_MSS_RCM_MCAN0_RST_CTRL_RESETVAL                                (0x00000000U)

/* MSS_MCAN1_RST_CTRL */

#define CSL_MSS_RCM_MCAN1_RST_CTRL_ASSERT_MASK          (0x00000007U)
#define CSL_MSS_RCM_MCAN1_RST_CTRL_ASSERT_SHIFT         (0x00000000U)
#define CSL_MSS_RCM_MCAN1_RST_CTRL_ASSERT_RESETVAL      (0x00000000U)
#define CSL_MSS_RCM_MCAN1_RST_CTRL_ASSERT_MAX           (0x00000007U)

#define CSL_MSS_RCM_MCAN1_RST_CTRL_RESETVAL                                (0x00000000U)

/* MSS_RTI0_RST_CTRL */

#define CSL_MSS_RCM_RTI0_RST_CTRL_ASSERT_MASK            (0x00000007U)
#define CSL_MSS_RCM_RTI0_RST_CTRL_ASSERT_SHIFT           (0x00000000U)
#define CSL_MSS_RCM_RTI0_RST_CTRL_ASSERT_RESETVAL        (0x00000000U)
#define CSL_MSS_RCM_RTI0_RST_CTRL_ASSERT_MAX             (0x00000007U)

#define CSL_MSS_RCM_RTI0_RST_CTRL_RESETVAL                                 (0x00000000U)

/* MSS_RTI1_RST_CTRL */

#define CSL_MSS_RCM_RTI1_RST_CTRL_ASSERT_MASK            (0x00000007U)
#define CSL_MSS_RCM_RTI1_RST_CTRL_ASSERT_SHIFT           (0x00000000U)
#define CSL_MSS_RCM_RTI1_RST_CTRL_ASSERT_RESETVAL        (0x00000000U)
#define CSL_MSS_RCM_RTI1_RST_CTRL_ASSERT_MAX             (0x00000007U)

#define CSL_MSS_RCM_RTI1_RST_CTRL_RESETVAL                                 (0x00000000U)

/* MSS_RTI2_RST_CTRL */

#define CSL_MSS_RCM_RTI2_RST_CTRL_ASSERT_MASK            (0x00000007U)
#define CSL_MSS_RCM_RTI2_RST_CTRL_ASSERT_SHIFT           (0x00000000U)
#define CSL_MSS_RCM_RTI2_RST_CTRL_ASSERT_RESETVAL        (0x00000000U)
#define CSL_MSS_RCM_RTI2_RST_CTRL_ASSERT_MAX             (0x00000007U)

#define CSL_MSS_RCM_RTI2_RST_CTRL_RESETVAL                                 (0x00000000U)

/* MSS_RTI3_RST_CTRL */

#define CSL_MSS_RCM_RTI3_RST_CTRL_ASSERT_MASK            (0x00000007U)
#define CSL_MSS_RCM_RTI3_RST_CTRL_ASSERT_SHIFT           (0x00000000U)
#define CSL_MSS_RCM_RTI3_RST_CTRL_ASSERT_RESETVAL        (0x00000000U)
#define CSL_MSS_RCM_RTI3_RST_CTRL_ASSERT_MAX             (0x00000007U)

#define CSL_MSS_RCM_RTI3_RST_CTRL_RESETVAL                                 (0x00000000U)

/* MSS_MCSPI0_RST_CTRL */

#define CSL_MSS_RCM_MCSPI0_RST_CTRL_ASSERT_MASK            (0x00000007U)
#define CSL_MSS_RCM_MCSPI0_RST_CTRL_ASSERT_SHIFT           (0x00000000U)
#define CSL_MSS_RCM_MCSPI0_RST_CTRL_ASSERT_RESETVAL        (0x00000000U)
#define CSL_MSS_RCM_MCSPI0_RST_CTRL_ASSERT_MAX             (0x00000007U)

#define CSL_MSS_RCM_MCSPI0_RST_CTRL_RESETVAL                                 (0x00000000U)

/* MSS_MCSPI1_RST_CTRL */

#define CSL_MSS_RCM_MCSPI1_RST_CTRL_ASSERT_MASK            (0x00000007U)
#define CSL_MSS_RCM_MCSPI1_RST_CTRL_ASSERT_SHIFT           (0x00000000U)
#define CSL_MSS_RCM_MCSPI1_RST_CTRL_ASSERT_RESETVAL        (0x00000000U)
#define CSL_MSS_RCM_MCSPI1_RST_CTRL_ASSERT_MAX             (0x00000007U)

#define CSL_MSS_RCM_MCSPI1_RST_CTRL_RESETVAL                                 (0x00000000U)

/* MSS_MCSPI2_RST_CTRL */

#define CSL_MSS_RCM_MCSPI2_RST_CTRL_ASSERT_MASK            (0x00000007U)
#define CSL_MSS_RCM_MCSPI2_RST_CTRL_ASSERT_SHIFT           (0x00000000U)
#define CSL_MSS_RCM_MCSPI2_RST_CTRL_ASSERT_RESETVAL        (0x00000000U)
#define CSL_MSS_RCM_MCSPI2_RST_CTRL_ASSERT_MAX             (0x00000007U)

#define CSL_MSS_RCM_MCSPI2_RST_CTRL_RESETVAL                                 (0x00000000U)

/* MSS_MCSPI3_RST_CTRL */

#define CSL_MSS_RCM_MCSPI3_RST_CTRL_ASSERT_MASK            (0x00000007U)
#define CSL_MSS_RCM_MCSPI3_RST_CTRL_ASSERT_SHIFT           (0x00000000U)
#define CSL_MSS_RCM_MCSPI3_RST_CTRL_ASSERT_RESETVAL        (0x00000000U)
#define CSL_MSS_RCM_MCSPI3_RST_CTRL_ASSERT_MAX             (0x00000007U)

#define CSL_MSS_RCM_MCSPI3_RST_CTRL_RESETVAL                                 (0x00000000U)

/* MSS_WDT0_RST_CTRL */

#define CSL_MSS_RCM_WDT0_RST_CTRL_ASSERT_MASK            (0x00000007U)
#define CSL_MSS_RCM_WDT0_RST_CTRL_ASSERT_SHIFT           (0x00000000U)
#define CSL_MSS_RCM_WDT0_RST_CTRL_ASSERT_RESETVAL        (0x00000000U)
#define CSL_MSS_RCM_WDT0_RST_CTRL_ASSERT_MAX             (0x00000007U)

#define CSL_MSS_RCM_WDT0_RST_CTRL_RESETVAL                                 (0x00000000U)

/* MSS_WDT1_RST_CTRL */

#define CSL_MSS_RCM_WDT1_RST_CTRL_ASSERT_MASK            (0x00000007U)
#define CSL_MSS_RCM_WDT1_RST_CTRL_ASSERT_SHIFT           (0x00000000U)
#define CSL_MSS_RCM_WDT1_RST_CTRL_ASSERT_RESETVAL        (0x00000000U)
#define CSL_MSS_RCM_WDT1_RST_CTRL_ASSERT_MAX             (0x00000007U)

#define CSL_MSS_RCM_WDT1_RST_CTRL_RESETVAL                                 (0x00000000U)

/* ICSSM0_RST_CTRL */

#define CSL_MSS_RCM_ICSSM0_RST_CTRL_ASSERT_MASK        (0x00000007U)
#define CSL_MSS_RCM_ICSSM0_RST_CTRL_ASSERT_SHIFT       (0x00000000U)
#define CSL_MSS_RCM_ICSSM0_RST_CTRL_ASSERT_RESETVAL    (0x00000000U)
#define CSL_MSS_RCM_ICSSM0_RST_CTRL_ASSERT_MAX         (0x00000007U)

#define CSL_MSS_RCM_ICSSM0_RST_CTRL_RESETVAL                               (0x00000000U)

/* ICSSM1_RST_CTRL */

#define CSL_MSS_RCM_ICSSM1_RST_CTRL_ASSERT_MASK        (0x00000007U)
#define CSL_MSS_RCM_ICSSM1_RST_CTRL_ASSERT_SHIFT       (0x00000000U)
#define CSL_MSS_RCM_ICSSM1_RST_CTRL_ASSERT_RESETVAL    (0x00000000U)
#define CSL_MSS_RCM_ICSSM1_RST_CTRL_ASSERT_MAX         (0x00000007U)

#define CSL_MSS_RCM_ICSSM1_RST_CTRL_RESETVAL                               (0x00000000U)

/* OSPI0_RST_CTRL */

#define CSL_MSS_RCM_OSPI0_RST_CTRL_ASSERT_MASK          (0x00000007U)
#define CSL_MSS_RCM_OSPI0_RST_CTRL_ASSERT_SHIFT         (0x00000000U)
#define CSL_MSS_RCM_OSPI0_RST_CTRL_ASSERT_RESETVAL      (0x00000000U)
#define CSL_MSS_RCM_OSPI0_RST_CTRL_ASSERT_MAX           (0x00000007U)



/* OSPI1_RST_CTRL */
#define CSL_MSS_RCM_OSPI1_RST_CTRL_ASSERT_MASK                              (0x00000007U)
#define CSL_MSS_RCM_OSPI1_RST_CTRL_ASSERT_SHIFT                             (0x00000000U)
#define CSL_MSS_RCM_OSPI1_RST_CTRL_ASSERT_RESETVAL                          (0x00000000U)
#define CSL_MSS_RCM_OSPI1_RST_CTRL_ASSERT_MAX                               (0x00000007U)

/* GPMC_RST_CTRL */

#define CSL_MSS_RCM_GPMC_RST_CTRL_ASSERT_MASK                    (0x00000007U)
#define CSL_MSS_RCM_GPMC_RST_CTRL_ASSERT_SHIFT                   (0x00000000U)
#define CSL_MSS_RCM_GPMC_RST_CTRL_ASSERT_RESETVAL                (0x00000000U)
#define CSL_MSS_RCM_GPMC_RST_CTRL_ASSERT_MAX                     (0x00000007U)

#define CSL_MSS_RCM_GPMC_RST_CTRL_RESETVAL                                     (0x00000000U)

/* MSS_MMC0_RST_CTRL */

#define CSL_MSS_RCM_MMC0_RST_CTRL_ASSERT_MASK          (0x00000007U)
#define CSL_MSS_RCM_MMC0_RST_CTRL_ASSERT_SHIFT         (0x00000000U)
#define CSL_MSS_RCM_MMC0_RST_CTRL_ASSERT_RESETVAL      (0x00000000U)
#define CSL_MSS_RCM_MMC0_RST_CTRL_ASSERT_MAX           (0x00000007U)

#define CSL_MSS_RCM_MMC0_RST_CTRL_RESETVAL                                (0x00000000U)

/* MSS_ELM_RST_CTRL */

#define CSL_MSS_RCM_MSS_ELM_RST_CTRL_ASSERT_MASK              (0x00000007U)
#define CSL_MSS_RCM_MSS_ELM_RST_CTRL_ASSERT_SHIFT             (0x00000000U)
#define CSL_MSS_RCM_MSS_ELM_RST_CTRL_ASSERT_RESETVAL          (0x00000000U)
#define CSL_MSS_RCM_MSS_ELM_RST_CTRL_ASSERT_MAX               (0x00000007U)

#define CSL_MSS_RCM_MSS_ELM_RST_CTRL_RESETVAL                                  (0x00000000U)

/* TEMPSENSE_32K_RST_CTRL */

#define CSL_MSS_RCM_TEMPSENSE_32K_RST_CTRL_ASSERT_MASK  (0x00000007U)
#define CSL_MSS_RCM_TEMPSENSE_32K_RST_CTRL_ASSERT_SHIFT (0x00000000U)
#define CSL_MSS_RCM_TEMPSENSE_32K_RST_CTRL_ASSERT_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_TEMPSENSE_32K_RST_CTRL_ASSERT_MAX   (0x00000007U)

#define CSL_MSS_RCM_TEMPSENSE_32K_RST_CTRL_RESETVAL                            (0x00000000U)

/* MSS_CPSW_RST_CTRL */

#define CSL_MSS_RCM_CPSW_RST_CTRL_ASSERT_MASK            (0x00000007U)
#define CSL_MSS_RCM_CPSW_RST_CTRL_ASSERT_SHIFT           (0x00000000U)
#define CSL_MSS_RCM_CPSW_RST_CTRL_ASSERT_RESETVAL        (0x00000000U)
#define CSL_MSS_RCM_CPSW_RST_CTRL_ASSERT_MAX             (0x00000007U)

#define CSL_MSS_RCM_CPSW_RST_CTRL_RESETVAL                                 (0x00000000U)

/* MSS_I2C0_RST_CTRL */

#define CSL_MSS_RCM_I2C0_RST_CTRL_ASSERT_MASK            (0x00000007U)
#define CSL_MSS_RCM_I2C0_RST_CTRL_ASSERT_SHIFT           (0x00000000U)
#define CSL_MSS_RCM_I2C0_RST_CTRL_ASSERT_RESETVAL        (0x00000000U)
#define CSL_MSS_RCM_I2C0_RST_CTRL_ASSERT_MAX             (0x00000007U)

#define CSL_MSS_RCM_I2C0_RST_CTRL_RESETVAL                                 (0x00000000U)

/* MSS_I2C1_RST_CTRL */

#define CSL_MSS_RCM_I2C1_RST_CTRL_ASSERT_MASK            (0x00000007U)
#define CSL_MSS_RCM_I2C1_RST_CTRL_ASSERT_SHIFT           (0x00000000U)
#define CSL_MSS_RCM_I2C1_RST_CTRL_ASSERT_RESETVAL        (0x00000000U)
#define CSL_MSS_RCM_I2C1_RST_CTRL_ASSERT_MAX             (0x00000007U)

#define CSL_MSS_RCM_I2C1_RST_CTRL_RESETVAL                                 (0x00000000U)

/* MSS_I2C2_RST_CTRL */

#define CSL_MSS_RCM_I2C2_RST_CTRL_ASSERT_MASK            (0x00000007U)
#define CSL_MSS_RCM_I2C2_RST_CTRL_ASSERT_SHIFT           (0x00000000U)
#define CSL_MSS_RCM_I2C2_RST_CTRL_ASSERT_RESETVAL        (0x00000000U)
#define CSL_MSS_RCM_I2C2_RST_CTRL_ASSERT_MAX             (0x00000007U)

#define CSL_MSS_RCM_I2C2_RST_CTRL_RESETVAL                                 (0x00000000U)

/* LIN0_RST_CTRL */

#define CSL_MSS_RCM_LIN0_RST_CTRL_ASSERT_MASK            (0x00000007U)
#define CSL_MSS_RCM_LIN0_RST_CTRL_ASSERT_SHIFT           (0x00000000U)
#define CSL_MSS_RCM_LIN0_RST_CTRL_ASSERT_RESETVAL        (0x00000000U)
#define CSL_MSS_RCM_LIN0_RST_CTRL_ASSERT_MAX             (0x00000007U)

#define CSL_MSS_RCM_LIN0_RST_CTRL_RESETVAL                                 (0x00000000U)

/* LIN1_RST_CTRL */

#define CSL_MSS_RCM_LIN1_RST_CTRL_ASSERT_MASK            (0x00000007U)
#define CSL_MSS_RCM_LIN1_RST_CTRL_ASSERT_SHIFT           (0x00000000U)
#define CSL_MSS_RCM_LIN1_RST_CTRL_ASSERT_RESETVAL        (0x00000000U)
#define CSL_MSS_RCM_LIN1_RST_CTRL_ASSERT_MAX             (0x00000007U)

#define CSL_MSS_RCM_LIN1_RST_CTRL_RESETVAL                                 (0x00000000U)

/* LIN2_RST_CTRL */

#define CSL_MSS_RCM_LIN2_RST_CTRL_ASSERT_MASK            (0x00000007U)
#define CSL_MSS_RCM_LIN2_RST_CTRL_ASSERT_SHIFT           (0x00000000U)
#define CSL_MSS_RCM_LIN2_RST_CTRL_ASSERT_RESETVAL        (0x00000000U)
#define CSL_MSS_RCM_LIN2_RST_CTRL_ASSERT_MAX             (0x00000007U)

#define CSL_MSS_RCM_LIN2_RST_CTRL_RESETVAL                                 (0x00000000U)

/* UART0_RST_CTRL */

#define CSL_MSS_RCM_UART0_RST_CTRL_ASSERT_MASK          (0x00000007U)
#define CSL_MSS_RCM_UART0_RST_CTRL_ASSERT_SHIFT         (0x00000000U)
#define CSL_MSS_RCM_UART0_RST_CTRL_ASSERT_RESETVAL      (0x00000000U)
#define CSL_MSS_RCM_UART0_RST_CTRL_ASSERT_MAX           (0x00000007U)

#define CSL_MSS_RCM_UART0_RST_CTRL_RESETVAL                                (0x00000000U)

/* UART1_RST_CTRL */

#define CSL_MSS_RCM_UART1_RST_CTRL_ASSERT_MASK          (0x00000007U)
#define CSL_MSS_RCM_UART1_RST_CTRL_ASSERT_SHIFT         (0x00000000U)
#define CSL_MSS_RCM_UART1_RST_CTRL_ASSERT_RESETVAL      (0x00000000U)
#define CSL_MSS_RCM_UART1_RST_CTRL_ASSERT_MAX           (0x00000007U)

#define CSL_MSS_RCM_UART1_RST_CTRL_RESETVAL                                (0x00000000U)

/* UART2_RST_CTRL */

#define CSL_MSS_RCM_UART2_RST_CTRL_ASSERT_MASK          (0x00000007U)
#define CSL_MSS_RCM_UART2_RST_CTRL_ASSERT_SHIFT         (0x00000000U)
#define CSL_MSS_RCM_UART2_RST_CTRL_ASSERT_RESETVAL      (0x00000000U)
#define CSL_MSS_RCM_UART2_RST_CTRL_ASSERT_MAX           (0x00000007U)

#define CSL_MSS_RCM_UART2_RST_CTRL_RESETVAL                                (0x00000000U)

/* UART3_RST_CTRL */

#define CSL_MSS_RCM_UART3_RST_CTRL_ASSERT_MASK          (0x00000007U)
#define CSL_MSS_RCM_UART3_RST_CTRL_ASSERT_SHIFT         (0x00000000U)
#define CSL_MSS_RCM_UART3_RST_CTRL_ASSERT_RESETVAL      (0x00000000U)
#define CSL_MSS_RCM_UART3_RST_CTRL_ASSERT_MAX           (0x00000007U)

#define CSL_MSS_RCM_UART3_RST_CTRL_RESETVAL                                (0x00000000U)


/* UART4_RST_CTRL */
#define CSL_MSS_RCM_UART4_RST_CTRL_ASSERT_MASK                              (0x00000007U)
#define CSL_MSS_RCM_UART4_RST_CTRL_ASSERT_SHIFT                             (0x00000000U)
#define CSL_MSS_RCM_UART4_RST_CTRL_ASSERT_RESETVAL                          (0x00000000U)
#define CSL_MSS_RCM_UART4_RST_CTRL_ASSERT_MAX                               (0x00000007U)

#define CSL_MSS_RCM_UART4_RST_CTRL_RESETVAL                                (0x00000000U)



/* UART5_RST_CTRL */
#define CSL_MSS_RCM_UART5_RST_CTRL_ASSERT_MASK                              (0x00000007U)
#define CSL_MSS_RCM_UART5_RST_CTRL_ASSERT_SHIFT                             (0x00000000U)
#define CSL_MSS_RCM_UART5_RST_CTRL_ASSERT_RESETVAL                          (0x00000000U)
#define CSL_MSS_RCM_UART5_RST_CTRL_ASSERT_MAX                               (0x00000007U)

#define CSL_MSS_RCM_UART5_RST_CTRL_RESETVAL                                (0x00000000U)

/* MSS_R5SS0_POR_RST_CTRL */

#define CSL_MSS_RCM_R5SS0_POR_RST_CTRL_ASSERT_MASK (0x00000007U)
#define CSL_MSS_RCM_R5SS0_POR_RST_CTRL_ASSERT_SHIFT (0x00000000U)
#define CSL_MSS_RCM_R5SS0_POR_RST_CTRL_ASSERT_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_R5SS0_POR_RST_CTRL_ASSERT_MAX (0x00000007U)

#define CSL_MSS_RCM_R5SS0_POR_RST_CTRL_RESETVAL                           (0x00000000U)

/* MSS_R5SS0_CORE0_GRST_CTRL */

#define CSL_MSS_RCM_R5SS0_CORE0_GRST_CTRL_ASSERT_MASK      (0x00000007U)
#define CSL_MSS_RCM_R5SS0_CORE0_GRST_CTRL_ASSERT_SHIFT     (0x00000000U)
#define CSL_MSS_RCM_R5SS0_CORE0_GRST_CTRL_ASSERT_RESETVAL  (0x00000000U)
#define CSL_MSS_RCM_R5SS0_CORE0_GRST_CTRL_ASSERT_MAX       (0x00000007U)

#define CSL_MSS_RCM_R5SS0_CORE0_GRST_CTRL_RESETVAL                              (0x00000000U)

/* MSS_R5SS0_CORE1_GRST_CTRL */

#define CSL_MSS_RCM_R5SS0_CORE1_GRST_CTRL_ASSERT_MASK      (0x00000007U)
#define CSL_MSS_RCM_R5SS0_CORE1_GRST_CTRL_ASSERT_SHIFT     (0x00000000U)
#define CSL_MSS_RCM_R5SS0_CORE1_GRST_CTRL_ASSERT_RESETVAL  (0x00000000U)
#define CSL_MSS_RCM_R5SS0_CORE1_GRST_CTRL_ASSERT_MAX       (0x00000007U)

#define CSL_MSS_RCM_R5SS0_CORE1_GRST_CTRL_RESETVAL                              (0x00000000U)

/* MSS_R5SS0_CORE0_LRST_CTRL */

#define CSL_MSS_RCM_R5SS0_CORE0_LRST_CTRL_ASSERT_MASK          (0x00000007U)
#define CSL_MSS_RCM_R5SS0_CORE0_LRST_CTRL_ASSERT_SHIFT         (0x00000000U)
#define CSL_MSS_RCM_R5SS0_CORE0_LRST_CTRL_ASSERT_RESETVAL      (0x00000000U)
#define CSL_MSS_RCM_R5SS0_CORE0_LRST_CTRL_ASSERT_MAX           (0x00000007U)

#define CSL_MSS_RCM_R5SS0_CORE0_LRST_CTRL_RESETVAL                                (0x00000000U)

/* MSS_R5SS0_CORE1_LRST_CTRL */

#define CSL_MSS_RCM_R5SS0_CORE1_LRST_CTRL_ASSERT_MASK          (0x00000007U)
#define CSL_MSS_RCM_R5SS0_CORE1_LRST_CTRL_ASSERT_SHIFT         (0x00000000U)
#define CSL_MSS_RCM_R5SS0_CORE1_LRST_CTRL_ASSERT_RESETVAL      (0x00000000U)
#define CSL_MSS_RCM_R5SS0_CORE1_LRST_CTRL_ASSERT_MAX           (0x00000007U)

#define CSL_MSS_RCM_R5SS0_CORE1_LRST_CTRL_RESETVAL                                (0x00000000U)

/* MSS_VIMA_RST_CTRL0 */

#define CSL_MSS_RCM_VIMA_RST_CTRL0_ASSERT_MASK          (0x00000007U)
#define CSL_MSS_RCM_VIMA_RST_CTRL0_ASSERT_SHIFT         (0x00000000U)
#define CSL_MSS_RCM_VIMA_RST_CTRL0_ASSERT_RESETVAL      (0x00000000U)
#define CSL_MSS_RCM_VIMA_RST_CTRL0_ASSERT_MAX           (0x00000007U)

#define CSL_MSS_RCM_VIMA_RST_CTRL0_RESETVAL                                (0x00000000U)

/* MSS_VIMB_RST_CTRL0 */

#define CSL_MSS_RCM_VIMB_RST_CTRL0_ASSERT_MASK          (0x00000007U)
#define CSL_MSS_RCM_VIMB_RST_CTRL0_ASSERT_SHIFT         (0x00000000U)
#define CSL_MSS_RCM_VIMB_RST_CTRL0_ASSERT_RESETVAL      (0x00000000U)
#define CSL_MSS_RCM_VIMB_RST_CTRL0_ASSERT_MAX           (0x00000007U)

#define CSL_MSS_RCM_VIMB_RST_CTRL0_RESETVAL                                (0x00000000U)

/* MSS_GPIO0_RST_CTRL */

#define CSL_MSS_RCM_GPIO0_RST_CTRL_ASSERT_MASK          (0x00000007U)
#define CSL_MSS_RCM_GPIO0_RST_CTRL_ASSERT_SHIFT         (0x00000000U)
#define CSL_MSS_RCM_GPIO0_RST_CTRL_ASSERT_RESETVAL      (0x00000000U)
#define CSL_MSS_RCM_GPIO0_RST_CTRL_ASSERT_MAX           (0x00000007U)

#define CSL_MSS_RCM_GPIO0_RST_CTRL_RESETVAL                                (0x00000000U)

/* MSS_GPIO1_RST_CTRL */

#define CSL_MSS_RCM_GPIO1_RST_CTRL_ASSERT_MASK          (0x00000007U)
#define CSL_MSS_RCM_GPIO1_RST_CTRL_ASSERT_SHIFT         (0x00000000U)
#define CSL_MSS_RCM_GPIO1_RST_CTRL_ASSERT_RESETVAL      (0x00000000U)
#define CSL_MSS_RCM_GPIO1_RST_CTRL_ASSERT_MAX           (0x00000007U)

#define CSL_MSS_RCM_GPIO1_RST_CTRL_RESETVAL                                (0x00000000U)

/* MSS_EDMA_RST_CTRL */

#define CSL_MSS_RCM_EDMA_RST_CTRL_ASSERT_MASK            (0x00000007U)
#define CSL_MSS_RCM_EDMA_RST_CTRL_ASSERT_SHIFT           (0x00000000U)
#define CSL_MSS_RCM_EDMA_RST_CTRL_ASSERT_RESETVAL        (0x00000000U)
#define CSL_MSS_RCM_EDMA_RST_CTRL_ASSERT_MAX             (0x00000007U)

#define CSL_MSS_RCM_EDMA_RST_CTRL_TPCCA_ASSERT_MASK      (0x00000070U)
#define CSL_MSS_RCM_EDMA_RST_CTRL_TPCCA_ASSERT_SHIFT     (0x00000004U)
#define CSL_MSS_RCM_EDMA_RST_CTRL_TPCCA_ASSERT_RESETVAL  (0x00000000U)
#define CSL_MSS_RCM_EDMA_RST_CTRL_TPCCA_ASSERT_MAX       (0x00000007U)

#define CSL_MSS_RCM_EDMA_RST_CTRL_TPTCA0_ASSERT_MASK     (0x00000700U)
#define CSL_MSS_RCM_EDMA_RST_CTRL_TPTCA0_ASSERT_SHIFT    (0x00000008U)
#define CSL_MSS_RCM_EDMA_RST_CTRL_TPTCA0_ASSERT_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_EDMA_RST_CTRL_TPTCA0_ASSERT_MAX      (0x00000007U)

#define CSL_MSS_RCM_EDMA_RST_CTRL_TPTCA1_ASSERT_MASK     (0x00007000U)
#define CSL_MSS_RCM_EDMA_RST_CTRL_TPTCA1_ASSERT_SHIFT    (0x0000000CU)
#define CSL_MSS_RCM_EDMA_RST_CTRL_TPTCA1_ASSERT_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_EDMA_RST_CTRL_TPTCA1_ASSERT_MAX      (0x00000007U)

#define CSL_MSS_RCM_EDMA_RST_CTRL_RESETVAL                                 (0x00000000U)

/* MSS_INFRA_RST_CTRL */

#define CSL_MSS_RCM_INFRA_RST_CTRL_ASSERT_MASK          (0x00000007U)
#define CSL_MSS_RCM_INFRA_RST_CTRL_ASSERT_SHIFT         (0x00000000U)
#define CSL_MSS_RCM_INFRA_RST_CTRL_ASSERT_RESETVAL      (0x00000000U)
#define CSL_MSS_RCM_INFRA_RST_CTRL_ASSERT_MAX           (0x00000007U)

#define CSL_MSS_RCM_INFRA_RST_CTRL_RESETVAL                                (0x00000000U)

/* MSS_SPINLOCK_RST_CTRL */

#define CSL_MSS_RCM_SPINLOCK_RST_CTRL_ASSERT_MASK    (0x00000007U)
#define CSL_MSS_RCM_SPINLOCK_RST_CTRL_ASSERT_SHIFT   (0x00000000U)
#define CSL_MSS_RCM_SPINLOCK_RST_CTRL_ASSERT_RESETVAL (0x00000000U)
#define CSL_MSS_RCM_SPINLOCK_RST_CTRL_ASSERT_MAX     (0x00000007U)

#define CSL_MSS_RCM_SPINLOCK_RST_CTRL_RESETVAL                             (0x00000000U)

/* USB_RST_CTRL */

#define CSL_MSS_RCM_USB_RST_CTRL_ASSERT_MASK                      (0x00000007U)
#define CSL_MSS_RCM_USB_RST_CTRL_ASSERT_SHIFT                     (0x00000000U)
#define CSL_MSS_RCM_USB_RST_CTRL_ASSERT_RESETVAL                  (0x00000000U)
#define CSL_MSS_RCM_USB_RST_CTRL_ASSERT_MAX                       (0x00000007U)

#define CSL_MSS_RCM_USB_RST_CTRL_RESETVAL                                      (0x00000000U)

/* MSS_CRC_RST_CTRL */

#define CSL_MSS_RCM_CRC_RST_CTRL_ASSERT_MASK              (0x00000007U)
#define CSL_MSS_RCM_CRC_RST_CTRL_ASSERT_SHIFT             (0x00000000U)
#define CSL_MSS_RCM_CRC_RST_CTRL_ASSERT_RESETVAL          (0x00000000U)
#define CSL_MSS_RCM_CRC_RST_CTRL_ASSERT_MAX               (0x00000007U)

#define CSL_MSS_RCM_CRC_RST_CTRL_RESETVAL                                  (0x00000000U)

/* MSS_ESM_RST_CTRL */

#define CSL_MSS_RCM_ESM_RST_CTRL_ASSERT_MASK              (0x00000007U)
#define CSL_MSS_RCM_ESM_RST_CTRL_ASSERT_SHIFT             (0x00000000U)
#define CSL_MSS_RCM_ESM_RST_CTRL_ASSERT_RESETVAL          (0x00000000U)
#define CSL_MSS_RCM_ESM_RST_CTRL_ASSERT_MAX               (0x00000007U)

#define CSL_MSS_RCM_ESM_RST_CTRL_RESETVAL                                  (0x00000000U)

/* MSS_DCCA_RST_CTRL */

#define CSL_MSS_RCM_DCCA_RST_CTRL_ASSERT_MASK            (0x00000007U)
#define CSL_MSS_RCM_DCCA_RST_CTRL_ASSERT_SHIFT           (0x00000000U)
#define CSL_MSS_RCM_DCCA_RST_CTRL_ASSERT_RESETVAL        (0x00000000U)
#define CSL_MSS_RCM_DCCA_RST_CTRL_ASSERT_MAX             (0x00000007U)

#define CSL_MSS_RCM_DCCA_RST_CTRL_RESETVAL                                 (0x00000000U)

/* MSS_DCCB_RST_CTRL */

#define CSL_MSS_RCM_DCCB_RST_CTRL_ASSERT_MASK            (0x00000007U)
#define CSL_MSS_RCM_DCCB_RST_CTRL_ASSERT_SHIFT           (0x00000000U)
#define CSL_MSS_RCM_DCCB_RST_CTRL_ASSERT_RESETVAL        (0x00000000U)
#define CSL_MSS_RCM_DCCB_RST_CTRL_ASSERT_MAX             (0x00000007U)

#define CSL_MSS_RCM_DCCB_RST_CTRL_RESETVAL                                 (0x00000000U)

/* MSS_DCCC_RST_CTRL */

#define CSL_MSS_RCM_DCCC_RST_CTRL_ASSERT_MASK            (0x00000007U)
#define CSL_MSS_RCM_DCCC_RST_CTRL_ASSERT_SHIFT           (0x00000000U)
#define CSL_MSS_RCM_DCCC_RST_CTRL_ASSERT_RESETVAL        (0x00000000U)
#define CSL_MSS_RCM_DCCC_RST_CTRL_ASSERT_MAX             (0x00000007U)

#define CSL_MSS_RCM_DCCC_RST_CTRL_RESETVAL                                 (0x00000000U)

/* MSS_DCCD_RST_CTRL */

#define CSL_MSS_RCM_DCCD_RST_CTRL_ASSERT_MASK            (0x00000007U)
#define CSL_MSS_RCM_DCCD_RST_CTRL_ASSERT_SHIFT           (0x00000000U)
#define CSL_MSS_RCM_DCCD_RST_CTRL_ASSERT_RESETVAL        (0x00000000U)
#define CSL_MSS_RCM_DCCD_RST_CTRL_ASSERT_MAX             (0x00000007U)

#define CSL_MSS_RCM_DCCD_RST_CTRL_RESETVAL                                 (0x00000000U)

/* MSS_L2OCRAM_BANK0_PD_CTRL */

#define CSL_MSS_RCM_L2OCRAM_BANK0_PD_CTRL_ISO_MASK         (0x00000007U)
#define CSL_MSS_RCM_L2OCRAM_BANK0_PD_CTRL_ISO_SHIFT        (0x00000000U)
#define CSL_MSS_RCM_L2OCRAM_BANK0_PD_CTRL_ISO_RESETVAL     (0x00000000U)
#define CSL_MSS_RCM_L2OCRAM_BANK0_PD_CTRL_ISO_MAX          (0x00000007U)

#define CSL_MSS_RCM_L2OCRAM_BANK0_PD_CTRL_AONIN_MASK       (0x00000070U)
#define CSL_MSS_RCM_L2OCRAM_BANK0_PD_CTRL_AONIN_SHIFT      (0x00000004U)
#define CSL_MSS_RCM_L2OCRAM_BANK0_PD_CTRL_AONIN_RESETVAL   (0x00000007U)
#define CSL_MSS_RCM_L2OCRAM_BANK0_PD_CTRL_AONIN_MAX        (0x00000007U)

#define CSL_MSS_RCM_L2OCRAM_BANK0_PD_CTRL_AGOODIN_MASK     (0x00000700U)
#define CSL_MSS_RCM_L2OCRAM_BANK0_PD_CTRL_AGOODIN_SHIFT    (0x00000008U)
#define CSL_MSS_RCM_L2OCRAM_BANK0_PD_CTRL_AGOODIN_RESETVAL (0x00000007U)
#define CSL_MSS_RCM_L2OCRAM_BANK0_PD_CTRL_AGOODIN_MAX      (0x00000007U)

#define CSL_MSS_RCM_L2OCRAM_BANK0_PD_CTRL_RESETVAL                              (0x00000770U)

/* MSS_L2OCRAM_BANK1_PD_CTRL */

#define CSL_MSS_RCM_L2OCRAM_BANK1_PD_CTRL_ISO_MASK         (0x00000007U)
#define CSL_MSS_RCM_L2OCRAM_BANK1_PD_CTRL_ISO_SHIFT        (0x00000000U)
#define CSL_MSS_RCM_L2OCRAM_BANK1_PD_CTRL_ISO_RESETVAL     (0x00000000U)
#define CSL_MSS_RCM_L2OCRAM_BANK1_PD_CTRL_ISO_MAX          (0x00000007U)

#define CSL_MSS_RCM_L2OCRAM_BANK1_PD_CTRL_AONIN_MASK       (0x00000070U)
#define CSL_MSS_RCM_L2OCRAM_BANK1_PD_CTRL_AONIN_SHIFT      (0x00000004U)
#define CSL_MSS_RCM_L2OCRAM_BANK1_PD_CTRL_AONIN_RESETVAL   (0x00000007U)
#define CSL_MSS_RCM_L2OCRAM_BANK1_PD_CTRL_AONIN_MAX        (0x00000007U)

#define CSL_MSS_RCM_L2OCRAM_BANK1_PD_CTRL_AGOODIN_MASK     (0x00000700U)
#define CSL_MSS_RCM_L2OCRAM_BANK1_PD_CTRL_AGOODIN_SHIFT    (0x00000008U)
#define CSL_MSS_RCM_L2OCRAM_BANK1_PD_CTRL_AGOODIN_RESETVAL (0x00000007U)
#define CSL_MSS_RCM_L2OCRAM_BANK1_PD_CTRL_AGOODIN_MAX      (0x00000007U)

#define CSL_MSS_RCM_L2OCRAM_BANK1_PD_CTRL_RESETVAL                              (0x00000770U)

/* MSS_L2OCRAM_BANK2_PD_CTRL */

#define CSL_MSS_RCM_L2OCRAM_BANK2_PD_CTRL_ISO_MASK         (0x00000007U)
#define CSL_MSS_RCM_L2OCRAM_BANK2_PD_CTRL_ISO_SHIFT        (0x00000000U)
#define CSL_MSS_RCM_L2OCRAM_BANK2_PD_CTRL_ISO_RESETVAL     (0x00000000U)
#define CSL_MSS_RCM_L2OCRAM_BANK2_PD_CTRL_ISO_MAX          (0x00000007U)

#define CSL_MSS_RCM_L2OCRAM_BANK2_PD_CTRL_AONIN_MASK       (0x00000070U)
#define CSL_MSS_RCM_L2OCRAM_BANK2_PD_CTRL_AONIN_SHIFT      (0x00000004U)
#define CSL_MSS_RCM_L2OCRAM_BANK2_PD_CTRL_AONIN_RESETVAL   (0x00000007U)
#define CSL_MSS_RCM_L2OCRAM_BANK2_PD_CTRL_AONIN_MAX        (0x00000007U)

#define CSL_MSS_RCM_L2OCRAM_BANK2_PD_CTRL_AGOODIN_MASK     (0x00000700U)
#define CSL_MSS_RCM_L2OCRAM_BANK2_PD_CTRL_AGOODIN_SHIFT    (0x00000008U)
#define CSL_MSS_RCM_L2OCRAM_BANK2_PD_CTRL_AGOODIN_RESETVAL (0x00000007U)
#define CSL_MSS_RCM_L2OCRAM_BANK2_PD_CTRL_AGOODIN_MAX      (0x00000007U)

#define CSL_MSS_RCM_L2OCRAM_BANK2_PD_CTRL_RESETVAL                              (0x00000770U)

/* MSS_L2OCRAM_BANK0_PD_STATUS */

#define CSL_MSS_RCM_L2OCRAM_BANK0_PD_STATUS_AONOUT_MASK  (0x00000001U)
#define CSL_MSS_RCM_L2OCRAM_BANK0_PD_STATUS_AONOUT_SHIFT (0x00000000U)
#define CSL_MSS_RCM_L2OCRAM_BANK0_PD_STATUS_AONOUT_RESETVAL (0x00000001U)
#define CSL_MSS_RCM_L2OCRAM_BANK0_PD_STATUS_AONOUT_MAX   (0x00000001U)

#define CSL_MSS_RCM_L2OCRAM_BANK0_PD_STATUS_AGOODOUT_MASK (0x00000002U)
#define CSL_MSS_RCM_L2OCRAM_BANK0_PD_STATUS_AGOODOUT_SHIFT (0x00000001U)
#define CSL_MSS_RCM_L2OCRAM_BANK0_PD_STATUS_AGOODOUT_RESETVAL (0x00000001U)
#define CSL_MSS_RCM_L2OCRAM_BANK0_PD_STATUS_AGOODOUT_MAX (0x00000001U)

#define CSL_MSS_RCM_L2OCRAM_BANK0_PD_STATUS_RESETVAL                            (0x00000003U)

/* MSS_L2OCRAM_BANK1_PD_STATUS */

#define CSL_MSS_RCM_L2OCRAM_BANK1_PD_STATUS_AONOUT_MASK  (0x00000001U)
#define CSL_MSS_RCM_L2OCRAM_BANK1_PD_STATUS_AONOUT_SHIFT (0x00000000U)
#define CSL_MSS_RCM_L2OCRAM_BANK1_PD_STATUS_AONOUT_RESETVAL (0x00000001U)
#define CSL_MSS_RCM_L2OCRAM_BANK1_PD_STATUS_AONOUT_MAX   (0x00000001U)

#define CSL_MSS_RCM_L2OCRAM_BANK1_PD_STATUS_AGOODOUT_MASK (0x00000002U)
#define CSL_MSS_RCM_L2OCRAM_BANK1_PD_STATUS_AGOODOUT_SHIFT (0x00000001U)
#define CSL_MSS_RCM_L2OCRAM_BANK1_PD_STATUS_AGOODOUT_RESETVAL (0x00000001U)
#define CSL_MSS_RCM_L2OCRAM_BANK1_PD_STATUS_AGOODOUT_MAX (0x00000001U)

#define CSL_MSS_RCM_L2OCRAM_BANK1_PD_STATUS_RESETVAL                            (0x00000003U)

/* MSS_L2OCRAM_BANK2_PD_STATUS */

#define CSL_MSS_RCM_L2OCRAM_BANK2_PD_STATUS_AONOUT_MASK  (0x00000001U)
#define CSL_MSS_RCM_L2OCRAM_BANK2_PD_STATUS_AONOUT_SHIFT (0x00000000U)
#define CSL_MSS_RCM_L2OCRAM_BANK2_PD_STATUS_AONOUT_RESETVAL (0x00000001U)
#define CSL_MSS_RCM_L2OCRAM_BANK2_PD_STATUS_AONOUT_MAX   (0x00000001U)

#define CSL_MSS_RCM_L2OCRAM_BANK2_PD_STATUS_AGOODOUT_MASK (0x00000002U)
#define CSL_MSS_RCM_L2OCRAM_BANK2_PD_STATUS_AGOODOUT_SHIFT (0x00000001U)
#define CSL_MSS_RCM_L2OCRAM_BANK2_PD_STATUS_AGOODOUT_RESETVAL (0x00000001U)
#define CSL_MSS_RCM_L2OCRAM_BANK2_PD_STATUS_AGOODOUT_MAX (0x00000001U)

#define CSL_MSS_RCM_L2OCRAM_BANK2_PD_STATUS_RESETVAL                            (0x00000003U)

/* HW_REG0 */

#define CSL_MSS_RCM_HW_REG0_HWREG_MASK                                 (0xFFFFFFFFU)
#define CSL_MSS_RCM_HW_REG0_HWREG_SHIFT                                (0x00000000U)
#define CSL_MSS_RCM_HW_REG0_HWREG_RESETVAL                             (0x00000000U)
#define CSL_MSS_RCM_HW_REG0_HWREG_MAX                                  (0xFFFFFFFFU)

#define CSL_MSS_RCM_HW_REG0_RESETVAL                                           (0x00000000U)

/* HW_REG1 */

#define CSL_MSS_RCM_HW_REG1_HWREG_MASK                                 (0xFFFFFFFFU)
#define CSL_MSS_RCM_HW_REG1_HWREG_SHIFT                                (0x00000000U)
#define CSL_MSS_RCM_HW_REG1_HWREG_RESETVAL                             (0x00000000U)
#define CSL_MSS_RCM_HW_REG1_HWREG_MAX                                  (0xFFFFFFFFU)

#define CSL_MSS_RCM_HW_REG1_RESETVAL                                           (0x00000000U)

/* HW_REG2 */

#define CSL_MSS_RCM_HW_REG2_HWREG_MASK                                 (0xFFFFFFFFU)
#define CSL_MSS_RCM_HW_REG2_HWREG_SHIFT                                (0x00000000U)
#define CSL_MSS_RCM_HW_REG2_HWREG_RESETVAL                             (0x00000000U)
#define CSL_MSS_RCM_HW_REG2_HWREG_MAX                                  (0xFFFFFFFFU)

#define CSL_MSS_RCM_HW_REG2_RESETVAL                                           (0x00000000U)

/* HW_REG3 */

#define CSL_MSS_RCM_HW_REG3_HWREG_MASK                                 (0xFFFFFFFFU)
#define CSL_MSS_RCM_HW_REG3_HWREG_SHIFT                                (0x00000000U)
#define CSL_MSS_RCM_HW_REG3_HWREG_RESETVAL                             (0x00000000U)
#define CSL_MSS_RCM_HW_REG3_HWREG_MAX                                  (0xFFFFFFFFU)

#define CSL_MSS_RCM_HW_REG3_RESETVAL                                           (0x00000000U)

/* HW_SPARE_RW0 */

#define CSL_MSS_RCM_HW_SPARE_RW0_HW_SPARE_RW0_MASK                (0xFFFFFFFFU)
#define CSL_MSS_RCM_HW_SPARE_RW0_HW_SPARE_RW0_SHIFT               (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_RW0_HW_SPARE_RW0_RESETVAL            (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_RW0_HW_SPARE_RW0_MAX                 (0xFFFFFFFFU)

#define CSL_MSS_RCM_HW_SPARE_RW0_RESETVAL                                      (0x00000000U)

/* HW_SPARE_RW1 */

#define CSL_MSS_RCM_HW_SPARE_RW1_HW_SPARE_RW1_MASK                (0xFFFFFFFFU)
#define CSL_MSS_RCM_HW_SPARE_RW1_HW_SPARE_RW1_SHIFT               (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_RW1_HW_SPARE_RW1_RESETVAL            (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_RW1_HW_SPARE_RW1_MAX                 (0xFFFFFFFFU)

#define CSL_MSS_RCM_HW_SPARE_RW1_RESETVAL                                      (0x00000000U)

/* HW_SPARE_RW2 */

#define CSL_MSS_RCM_HW_SPARE_RW2_HW_SPARE_RW2_MASK                (0xFFFFFFFFU)
#define CSL_MSS_RCM_HW_SPARE_RW2_HW_SPARE_RW2_SHIFT               (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_RW2_HW_SPARE_RW2_RESETVAL            (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_RW2_HW_SPARE_RW2_MAX                 (0xFFFFFFFFU)

#define CSL_MSS_RCM_HW_SPARE_RW2_RESETVAL                                      (0x00000000U)

/* HW_SPARE_RW3 */

#define CSL_MSS_RCM_HW_SPARE_RW3_HW_SPARE_RW3_MASK                (0xFFFFFFFFU)
#define CSL_MSS_RCM_HW_SPARE_RW3_HW_SPARE_RW3_SHIFT               (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_RW3_HW_SPARE_RW3_RESETVAL            (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_RW3_HW_SPARE_RW3_MAX                 (0xFFFFFFFFU)

#define CSL_MSS_RCM_HW_SPARE_RW3_RESETVAL                                      (0x00000000U)

/* HW_SPARE_RO0 */

#define CSL_MSS_RCM_HW_SPARE_RO0_HW_SPARE_RO0_MASK                (0xFFFFFFFFU)
#define CSL_MSS_RCM_HW_SPARE_RO0_HW_SPARE_RO0_SHIFT               (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_RO0_HW_SPARE_RO0_RESETVAL            (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_RO0_HW_SPARE_RO0_MAX                 (0xFFFFFFFFU)

#define CSL_MSS_RCM_HW_SPARE_RO0_RESETVAL                                      (0x00000000U)

/* HW_SPARE_RO1 */

#define CSL_MSS_RCM_HW_SPARE_RO1_HW_SPARE_RO1_MASK                (0xFFFFFFFFU)
#define CSL_MSS_RCM_HW_SPARE_RO1_HW_SPARE_RO1_SHIFT               (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_RO1_HW_SPARE_RO1_RESETVAL            (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_RO1_HW_SPARE_RO1_MAX                 (0xFFFFFFFFU)

#define CSL_MSS_RCM_HW_SPARE_RO1_RESETVAL                                      (0x00000000U)

/* HW_SPARE_RO2 */

#define CSL_MSS_RCM_HW_SPARE_RO2_HW_SPARE_RO2_MASK                (0xFFFFFFFFU)
#define CSL_MSS_RCM_HW_SPARE_RO2_HW_SPARE_RO2_SHIFT               (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_RO2_HW_SPARE_RO2_RESETVAL            (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_RO2_HW_SPARE_RO2_MAX                 (0xFFFFFFFFU)

#define CSL_MSS_RCM_HW_SPARE_RO2_RESETVAL                                      (0x00000000U)

/* HW_SPARE_RO3 */

#define CSL_MSS_RCM_HW_SPARE_RO3_HW_SPARE_RO3_MASK                (0xFFFFFFFFU)
#define CSL_MSS_RCM_HW_SPARE_RO3_HW_SPARE_RO3_SHIFT               (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_RO3_HW_SPARE_RO3_RESETVAL            (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_RO3_HW_SPARE_RO3_MAX                 (0xFFFFFFFFU)

#define CSL_MSS_RCM_HW_SPARE_RO3_RESETVAL                                      (0x00000000U)

/* HW_SPARE_WPH */

#define CSL_MSS_RCM_HW_SPARE_WPH_HW_SPARE_WPH_MASK                (0xFFFFFFFFU)
#define CSL_MSS_RCM_HW_SPARE_WPH_HW_SPARE_WPH_SHIFT               (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_WPH_HW_SPARE_WPH_RESETVAL            (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_WPH_HW_SPARE_WPH_MAX                 (0xFFFFFFFFU)

#define CSL_MSS_RCM_HW_SPARE_WPH_RESETVAL                                      (0x00000000U)

/* HW_SPARE_REC */

#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC0_MASK               (0x00000001U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC0_SHIFT              (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC0_RESETVAL           (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC0_MAX                (0x00000001U)

#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC1_MASK               (0x00000002U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC1_SHIFT              (0x00000001U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC1_RESETVAL           (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC1_MAX                (0x00000001U)

#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC2_MASK               (0x00000004U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC2_SHIFT              (0x00000002U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC2_RESETVAL           (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC2_MAX                (0x00000001U)

#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC3_MASK               (0x00000008U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC3_SHIFT              (0x00000003U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC3_RESETVAL           (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC3_MAX                (0x00000001U)

#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC4_MASK               (0x00000010U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC4_SHIFT              (0x00000004U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC4_RESETVAL           (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC4_MAX                (0x00000001U)

#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC5_MASK               (0x00000020U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC5_SHIFT              (0x00000005U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC5_RESETVAL           (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC5_MAX                (0x00000001U)

#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC6_MASK               (0x00000040U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC6_SHIFT              (0x00000006U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC6_RESETVAL           (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC6_MAX                (0x00000001U)

#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC7_MASK               (0x00000080U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC7_SHIFT              (0x00000007U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC7_RESETVAL           (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC7_MAX                (0x00000001U)

#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC8_MASK               (0x00000100U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC8_SHIFT              (0x00000008U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC8_RESETVAL           (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC8_MAX                (0x00000001U)

#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC9_MASK               (0x00000200U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC9_SHIFT              (0x00000009U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC9_RESETVAL           (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC9_MAX                (0x00000001U)

#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC10_MASK              (0x00000400U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC10_SHIFT             (0x0000000AU)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC10_RESETVAL          (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC10_MAX               (0x00000001U)

#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC11_MASK              (0x00000800U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC11_SHIFT             (0x0000000BU)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC11_RESETVAL          (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC11_MAX               (0x00000001U)

#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC12_MASK              (0x00001000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC12_SHIFT             (0x0000000CU)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC12_RESETVAL          (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC12_MAX               (0x00000001U)

#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC13_MASK              (0x00002000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC13_SHIFT             (0x0000000DU)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC13_RESETVAL          (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC13_MAX               (0x00000001U)

#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC14_MASK              (0x00004000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC14_SHIFT             (0x0000000EU)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC14_RESETVAL          (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC14_MAX               (0x00000001U)

#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC15_MASK              (0x00008000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC15_SHIFT             (0x0000000FU)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC15_RESETVAL          (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC15_MAX               (0x00000001U)

#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC16_MASK              (0x00010000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC16_SHIFT             (0x00000010U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC16_RESETVAL          (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC16_MAX               (0x00000001U)

#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC17_MASK              (0x00020000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC17_SHIFT             (0x00000011U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC17_RESETVAL          (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC17_MAX               (0x00000001U)

#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC18_MASK              (0x00040000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC18_SHIFT             (0x00000012U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC18_RESETVAL          (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC18_MAX               (0x00000001U)

#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC19_MASK              (0x00080000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC19_SHIFT             (0x00000013U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC19_RESETVAL          (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC19_MAX               (0x00000001U)

#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC20_MASK              (0x00100000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC20_SHIFT             (0x00000014U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC20_RESETVAL          (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC20_MAX               (0x00000001U)

#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC21_MASK              (0x00200000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC21_SHIFT             (0x00000015U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC21_RESETVAL          (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC21_MAX               (0x00000001U)

#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC22_MASK              (0x00400000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC22_SHIFT             (0x00000016U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC22_RESETVAL          (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC22_MAX               (0x00000001U)

#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC23_MASK              (0x00800000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC23_SHIFT             (0x00000017U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC23_RESETVAL          (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC23_MAX               (0x00000001U)

#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC24_MASK              (0x01000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC24_SHIFT             (0x00000018U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC24_RESETVAL          (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC24_MAX               (0x00000001U)

#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC25_MASK              (0x02000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC25_SHIFT             (0x00000019U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC25_RESETVAL          (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC25_MAX               (0x00000001U)

#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC26_MASK              (0x04000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC26_SHIFT             (0x0000001AU)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC26_RESETVAL          (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC26_MAX               (0x00000001U)

#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC27_MASK              (0x08000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC27_SHIFT             (0x0000001BU)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC27_RESETVAL          (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC27_MAX               (0x00000001U)

#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC28_MASK              (0x10000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC28_SHIFT             (0x0000001CU)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC28_RESETVAL          (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC28_MAX               (0x00000001U)

#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC29_MASK              (0x20000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC29_SHIFT             (0x0000001DU)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC29_RESETVAL          (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC29_MAX               (0x00000001U)

#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC30_MASK              (0x40000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC30_SHIFT             (0x0000001EU)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC30_RESETVAL          (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC30_MAX               (0x00000001U)

#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC31_MASK              (0x80000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC31_SHIFT             (0x0000001FU)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC31_RESETVAL          (0x00000000U)
#define CSL_MSS_RCM_HW_SPARE_REC_HW_SPARE_REC31_MAX               (0x00000001U)

#define CSL_MSS_RCM_HW_SPARE_REC_RESETVAL                                      (0x00000000U)

/* LOCK0_KICK0 */

#define CSL_MSS_RCM_LOCK0_KICK0_MASK                               (0xFFFFFFFFU)
#define CSL_MSS_RCM_LOCK0_KICK0_SHIFT                              (0x00000000U)
#define CSL_MSS_RCM_LOCK0_KICK0_RESETVAL                           (0x00000000U)
#define CSL_MSS_RCM_LOCK0_KICK0_MAX                                (0xFFFFFFFFU)

#define CSL_MSS_RCM_LOCK0_KICK0_RESETVAL                                       (0x00000000U)

/* LOCK0_KICK1 */

#define CSL_MSS_RCM_LOCK0_KICK1_MASK                               (0xFFFFFFFFU)
#define CSL_MSS_RCM_LOCK0_KICK1_SHIFT                              (0x00000000U)
#define CSL_MSS_RCM_LOCK0_KICK1_RESETVAL                           (0x00000000U)
#define CSL_MSS_RCM_LOCK0_KICK1_MAX                                (0xFFFFFFFFU)

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

#define CSL_MSS_RCM_EOI_VECTOR_MASK                                        (0x000000FFU)
#define CSL_MSS_RCM_EOI_VECTOR_SHIFT                                       (0x00000000U)
#define CSL_MSS_RCM_EOI_VECTOR_RESETVAL                                    (0x00000000U)
#define CSL_MSS_RCM_EOI_VECTOR_MAX                                         (0x000000FFU)

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
