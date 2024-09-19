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
 *  Name        : cslr_top_rcm.h
*/
#ifndef CSLR_TOP_RCM_H_
#define CSLR_TOP_RCM_H_

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
	volatile uint32_t	PID;
	volatile uint32_t	WARM_RST_CAUSE;
	volatile uint32_t	WARM_RST_CAUSE_CLR;
	volatile uint32_t	WARM_RESET_REQ;
	volatile uint32_t	WARM_RSTTIME1;
	volatile uint32_t	WARM_RSTTIME2;
	volatile uint32_t	WARM_RSTTIME3;
	volatile uint32_t	WARM_RESET_CONFIG_OV;
	volatile uint32_t	WARM_RESET_CONFIG_UV;
	volatile uint32_t	WARM_RESET_CONFIG_MISC;
	volatile uint32_t	WARM_RESET_CONFIG;
	volatile uint32_t	SW_POR;
	volatile uint8_t	Resv_144[96];
	volatile uint32_t	LIMP_MODE_EN;
	volatile uint32_t	CLK_LOSS_STATUS;
	volatile uint8_t	Resv_256[104];
	volatile uint32_t	RCOSC32K_CTRL;
	volatile uint32_t	PLL_REF_CLK_SRC_SEL;
	volatile uint32_t	PAD_XTAL_CTRL;
	volatile uint32_t	SOP_MODE_VALUE;
	volatile uint8_t	Resv_276[4];
	volatile uint32_t	VMON_CLK_GATE;
	volatile uint8_t	Resv_1024[744];
	volatile uint32_t	PLL_CORE_PWRCTRL;
	volatile uint32_t	PLL_CORE_CLKCTRL;
	volatile uint32_t	PLL_CORE_TENABLE;
	volatile uint32_t	PLL_CORE_TENABLEDIV;
	volatile uint32_t	PLL_CORE_M2NDIV;
	volatile uint32_t	PLL_CORE_MN2DIV;
	volatile uint32_t	PLL_CORE_FRACDIV;
	volatile uint32_t	PLL_CORE_BWCTRL;
	volatile uint32_t	PLL_CORE_FRACCTRL;
	volatile uint32_t	PLL_CORE_STATUS;
	volatile uint32_t	PLL_CORE_HSDIVIDER;
	volatile uint32_t	PLL_CORE_HSDIVIDER_CLKOUT0;
	volatile uint32_t	PLL_CORE_HSDIVIDER_CLKOUT1;
	volatile uint32_t	PLL_CORE_HSDIVIDER_CLKOUT2;
	volatile uint32_t	PLL_CORE_HSDIVIDER_CLKOUT3;
	volatile uint32_t	PLL_CORE_RSTCTRL;
	volatile uint32_t	PLL_CORE_HSDIVIDER_RSTCTRL;
	volatile uint8_t	Resv_1280[188];
	volatile uint32_t	R5SS_CLK_SRC_SEL;
	volatile uint32_t	R5SS_CLK_STATUS;
	volatile uint8_t	Resv_1296[8];
	volatile uint32_t	R5SS0_CLK_DIV_SEL;
	volatile uint8_t	Resv_1304[4];
	volatile uint32_t	R5SS0_CLK_GATE;
	volatile uint8_t	Resv_1312[4];
	volatile uint32_t	SYS_CLK_DIV_VAL;
	volatile uint32_t	SYS_CLK_GATE;
	volatile uint32_t	SYS_CLK_STATUS;
	volatile uint8_t	Resv_2048[724];
	volatile uint32_t	PLL_PER_PWRCTRL;
	volatile uint32_t	PLL_PER_CLKCTRL;
	volatile uint32_t	PLL_PER_TENABLE;
	volatile uint32_t	PLL_PER_TENABLEDIV;
	volatile uint32_t	PLL_PER_M2NDIV;
	volatile uint32_t	PLL_PER_MN2DIV;
	volatile uint32_t	PLL_PER_FRACDIV;
	volatile uint32_t	PLL_PER_BWCTRL;
	volatile uint32_t	PLL_PER_FRACCTRL;
	volatile uint32_t	PLL_PER_STATUS;
	volatile uint32_t	PLL_PER_HSDIVIDER;
	volatile uint32_t	PLL_PER_HSDIVIDER_CLKOUT0;
	volatile uint32_t	PLL_PER_HSDIVIDER_CLKOUT1;
	volatile uint32_t	PLL_PER_HSDIVIDER_CLKOUT2;
	volatile uint32_t	PLL_PER_HSDIVIDER_CLKOUT3;
	volatile uint32_t	PLL_PER_RSTCTRL;
	volatile uint32_t	PLL_PER_HSDIVIDER_RSTCTRL;
	volatile uint8_t	Resv_2304[188];
	volatile uint32_t	PLL_ETH_PWRCTRL;
	volatile uint32_t	PLL_ETH_CLKCTRL;
	volatile uint32_t	PLL_ETH_TENABLE;
	volatile uint32_t	PLL_ETH_TENABLEDIV;
	volatile uint32_t	PLL_ETH_M2NDIV;
	volatile uint32_t	PLL_ETH_MN2DIV;
	volatile uint32_t	PLL_ETH_FRACDIV;
	volatile uint32_t	PLL_ETH_BWCTRL;
	volatile uint32_t	PLL_ETH_FRACCTRL;
	volatile uint32_t	PLL_ETH_STATUS;
	volatile uint32_t	PLL_ETH_HSDIVIDER;
	volatile uint32_t	PLL_ETH_HSDIVIDER_CLKOUT0;
	volatile uint8_t	Resv_2356[4];
	volatile uint32_t	PLL_ETH_HSDIVIDER_CLKOUT2;
	volatile uint8_t	Resv_2364[4];
	volatile uint32_t	PLL_ETH_RSTCTRL;
	volatile uint32_t	PLL_ETH_HSDIVIDER_RSTCTRL;
	volatile uint8_t	Resv_3072[700];
	volatile uint32_t	CLKOUT0_CLK_SRC_SEL;
	volatile uint32_t	CLKOUT1_CLK_SRC_SEL;
	volatile uint32_t	CLKOUT0_DIV_VAL;
	volatile uint32_t	CLKOUT1_DIV_VAL;
	volatile uint32_t	CLKOUT0_CLK_GATE;
	volatile uint32_t	CLKOUT1_CLK_GATE;
	volatile uint32_t	CLKOUT0_CLK_STATUS;
	volatile uint32_t	CLKOUT1_CLK_STATUS;
	volatile uint32_t	TRCCLKOUT_CLK_SRC_SEL;
	volatile uint32_t	TRCCLKOUT_DIV_VAL;
	volatile uint32_t	TRCCLKOUT_CLK_GATE;
	volatile uint32_t	TRCCLKOUT_CLK_STATUS;
	volatile uint32_t	VMON_CLK_DIV_VAL;
	volatile uint32_t	VMON_CLK_STATUS;
	volatile uint8_t	Resv_3328[200];
	volatile uint32_t	DFT_DMLED_EXEC;
	volatile uint32_t	DFT_DMLED_STATUS;
	volatile uint8_t	Resv_3840[504];
	volatile uint32_t	HW_REG0;
	volatile uint32_t	HW_REG1;
	volatile uint32_t	HW_REG2;
	volatile uint32_t	HW_REG3;
	volatile uint8_t	Resv_3904[48];
	volatile uint32_t	HW_SPARE_RW0;
	volatile uint32_t	HW_SPARE_RW1;
	volatile uint32_t	HW_SPARE_RW2;
	volatile uint32_t	HW_SPARE_RW3;
	volatile uint8_t	Resv_3968[48];
	volatile uint32_t	HW_SPARE_RO0;
	volatile uint32_t	HW_SPARE_RO1;
	volatile uint32_t	HW_SPARE_RO2;
	volatile uint32_t	HW_SPARE_RO3;
	volatile uint8_t	Resv_4032[48];
	volatile uint32_t	HW_SPARE_WPH;
	volatile uint32_t	HW_SPARE_REC;
	volatile uint8_t	Resv_4104[64];
	volatile uint32_t	LOCK0_KICK0;
	volatile uint32_t	LOCK0_KICK1;
	volatile uint32_t	INTR_RAW_STATUS;
	volatile uint32_t	INTR_ENABLED_STATUS_CLEAR;
	volatile uint32_t	INTR_ENABLE;
	volatile uint32_t	INTR_ENABLE_CLEAR;
	volatile uint32_t	EOI;
	volatile uint32_t	FAULT_ADDRESS;
	volatile uint32_t	FAULT_TYPE_STATUS;
	volatile uint32_t	FAULT_ATTR_STATUS;
	volatile uint32_t	FAULT_CLEAR;
} CSL_top_rcmRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

/*--------TOP_RCM_--------*/
#define CSL_TOP_RCM_PID                                                         (0x00000000U)
#define CSL_TOP_RCM_WARM_RST_CAUSE                                              (0x00000004U)
#define CSL_TOP_RCM_WARM_RST_CAUSE_CLR                                          (0x00000008U)
#define CSL_TOP_RCM_WARM_RESET_REQ                                              (0x0000000CU)
#define CSL_TOP_RCM_WARM_RSTTIME1                                               (0x00000010U)
#define CSL_TOP_RCM_WARM_RSTTIME2                                               (0x00000014U)
#define CSL_TOP_RCM_WARM_RSTTIME3                                               (0x00000018U)
#define CSL_TOP_RCM_WARM_RESET_CONFIG_OV                                        (0x0000001CU)
#define CSL_TOP_RCM_WARM_RESET_CONFIG_UV                                        (0x00000020U)
#define CSL_TOP_RCM_WARM_RESET_CONFIG_MISC                                      (0x00000024U)
#define CSL_TOP_RCM_WARM_RESET_CONFIG                                           (0x00000028U)
#define CSL_TOP_RCM_SW_POR                                                      (0x0000002CU)
#define CSL_TOP_RCM_LIMP_MODE_EN                                                (0x00000090U)
#define CSL_TOP_RCM_CLK_LOSS_STATUS                                             (0x00000094U)
#define CSL_TOP_RCM_RCOSC32K_CTRL                                               (0x00000100U)
#define CSL_TOP_RCM_PLL_REF_CLK_SRC_SEL                                         (0x00000104U)
#define CSL_TOP_RCM_PAD_XTAL_CTRL                                               (0x00000108U)
#define CSL_TOP_RCM_SOP_MODE_VALUE                                              (0x0000010CU)
#define CSL_TOP_RCM_VMON_CLK_GATE                                               (0x00000114U)
#define CSL_TOP_RCM_PLL_CORE_PWRCTRL                                            (0x00000400U)
#define CSL_TOP_RCM_PLL_CORE_CLKCTRL                                            (0x00000404U)
#define CSL_TOP_RCM_PLL_CORE_TENABLE                                            (0x00000408U)
#define CSL_TOP_RCM_PLL_CORE_TENABLEDIV                                         (0x0000040CU)
#define CSL_TOP_RCM_PLL_CORE_M2NDIV                                             (0x00000410U)
#define CSL_TOP_RCM_PLL_CORE_MN2DIV                                             (0x00000414U)
#define CSL_TOP_RCM_PLL_CORE_FRACDIV                                            (0x00000418U)
#define CSL_TOP_RCM_PLL_CORE_BWCTRL                                             (0x0000041CU)
#define CSL_TOP_RCM_PLL_CORE_FRACCTRL                                           (0x00000420U)
#define CSL_TOP_RCM_PLL_CORE_STATUS                                             (0x00000424U)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER                                          (0x00000428U)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT0                                  (0x0000042CU)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT1                                  (0x00000430U)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT2                                  (0x00000434U)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT3                                  (0x00000438U)
#define CSL_TOP_RCM_PLL_CORE_RSTCTRL                                            (0x0000043CU)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_RSTCTRL                                  (0x00000440U)
#define CSL_TOP_RCM_R5SS_CLK_SRC_SEL                                         (0x00000500U)
#define CSL_TOP_RCM_R5SS_CLK_STATUS                                     (0x00000504U)
#define CSL_TOP_RCM_R5SS0_CLK_DIV_SEL                                        (0x00000510U)
#define CSL_TOP_RCM_R5SS0_CLK_GATE                                           (0x00000518U)
#define CSL_TOP_RCM_SYS_CLK_DIV_VAL                                             (0x00000520U)
#define CSL_TOP_RCM_SYS_CLK_GATE                                                (0x00000524U)
#define CSL_TOP_RCM_SYS_CLK_STATUS                                              (0x00000528U)
#define CSL_TOP_RCM_PLL_PER_PWRCTRL                                             (0x00000800U)
#define CSL_TOP_RCM_PLL_PER_CLKCTRL                                             (0x00000804U)
#define CSL_TOP_RCM_PLL_PER_TENABLE                                             (0x00000808U)
#define CSL_TOP_RCM_PLL_PER_TENABLEDIV                                          (0x0000080CU)
#define CSL_TOP_RCM_PLL_PER_M2NDIV                                              (0x00000810U)
#define CSL_TOP_RCM_PLL_PER_MN2DIV                                              (0x00000814U)
#define CSL_TOP_RCM_PLL_PER_FRACDIV                                             (0x00000818U)
#define CSL_TOP_RCM_PLL_PER_BWCTRL                                              (0x0000081CU)
#define CSL_TOP_RCM_PLL_PER_FRACCTRL                                            (0x00000820U)
#define CSL_TOP_RCM_PLL_PER_STATUS                                              (0x00000824U)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER                                           (0x00000828U)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT0                                   (0x0000082CU)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT1                                   (0x00000830U)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT2                                   (0x00000834U)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT3                                   (0x00000838U)
#define CSL_TOP_RCM_PLL_PER_RSTCTRL                                             (0x0000083CU)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_RSTCTRL                                   (0x00000840U)
#define CSL_TOP_RCM_PLL_ETH_PWRCTRL                                             (0x00000900U)
#define CSL_TOP_RCM_PLL_ETH_CLKCTRL                                             (0x00000904U)
#define CSL_TOP_RCM_PLL_ETH_TENABLE                                             (0x00000908U)
#define CSL_TOP_RCM_PLL_ETH_TENABLEDIV                                          (0x0000090CU)
#define CSL_TOP_RCM_PLL_ETH_M2NDIV                                              (0x00000910U)
#define CSL_TOP_RCM_PLL_ETH_MN2DIV                                              (0x00000914U)
#define CSL_TOP_RCM_PLL_ETH_FRACDIV                                             (0x00000918U)
#define CSL_TOP_RCM_PLL_ETH_BWCTRL                                              (0x0000091CU)
#define CSL_TOP_RCM_PLL_ETH_FRACCTRL                                            (0x00000920U)
#define CSL_TOP_RCM_PLL_ETH_STATUS                                              (0x00000924U)
#define CSL_TOP_RCM_PLL_ETH_HSDIVIDER                                           (0x00000928U)
#define CSL_TOP_RCM_PLL_ETH_HSDIVIDER_CLKOUT0                                   (0x0000092CU)
#define CSL_TOP_RCM_PLL_ETH_HSDIVIDER_CLKOUT2                                   (0x00000934U)
#define CSL_TOP_RCM_PLL_ETH_RSTCTRL                                             (0x0000093CU)
#define CSL_TOP_RCM_PLL_ETH_HSDIVIDER_RSTCTRL                                   (0x00000940U)
#define CSL_TOP_RCM_CLKOUT0_CLK_SRC_SEL                                         (0x00000C00U)
#define CSL_TOP_RCM_CLKOUT1_CLK_SRC_SEL                                         (0x00000C04U)
#define CSL_TOP_RCM_CLKOUT0_DIV_VAL                                             (0x00000C08U)
#define CSL_TOP_RCM_CLKOUT1_DIV_VAL                                             (0x00000C0CU)
#define CSL_TOP_RCM_CLKOUT0_CLK_GATE                                            (0x00000C10U)
#define CSL_TOP_RCM_CLKOUT1_CLK_GATE                                            (0x00000C14U)
#define CSL_TOP_RCM_CLKOUT0_CLK_STATUS                                          (0x00000C18U)
#define CSL_TOP_RCM_CLKOUT1_CLK_STATUS                                          (0x00000C1CU)
#define CSL_TOP_RCM_TRCCLKOUT_CLK_SRC_SEL                                       (0x00000C20U)
#define CSL_TOP_RCM_TRCCLKOUT_DIV_VAL                                           (0x00000C24U)
#define CSL_TOP_RCM_TRCCLKOUT_CLK_GATE                                          (0x00000C28U)
#define CSL_TOP_RCM_TRCCLKOUT_CLK_STATUS                                        (0x00000C2CU)
#define CSL_TOP_RCM_VMON_CLK_DIV_VAL                                            (0x00000C30U)
#define CSL_TOP_RCM_VMON_CLK_STATUS                                             (0x00000C34U)
#define CSL_TOP_RCM_DFT_DMLED_EXEC                                              (0x00000D00U)
#define CSL_TOP_RCM_DFT_DMLED_STATUS                                            (0x00000D04U)
#define CSL_TOP_RCM_HW_REG0                                                     (0x00000F00U)
#define CSL_TOP_RCM_HW_REG1                                                     (0x00000F04U)
#define CSL_TOP_RCM_HW_REG2                                                     (0x00000F08U)
#define CSL_TOP_RCM_HW_REG3                                                     (0x00000F0CU)
#define CSL_TOP_RCM_HW_SPARE_RW0                                                (0x00000F40U)
#define CSL_TOP_RCM_HW_SPARE_RW1                                                (0x00000F44U)
#define CSL_TOP_RCM_HW_SPARE_RW2                                                (0x00000F48U)
#define CSL_TOP_RCM_HW_SPARE_RW3                                                (0x00000F4CU)
#define CSL_TOP_RCM_HW_SPARE_RO0                                                (0x00000F80U)
#define CSL_TOP_RCM_HW_SPARE_RO1                                                (0x00000F84U)
#define CSL_TOP_RCM_HW_SPARE_RO2                                                (0x00000F88U)
#define CSL_TOP_RCM_HW_SPARE_RO3                                                (0x00000F8CU)
#define CSL_TOP_RCM_HW_SPARE_WPH                                                (0x00000FC0U)
#define CSL_TOP_RCM_HW_SPARE_REC                                                (0x00000FC4U)
#define CSL_TOP_RCM_LOCK0_KICK0                                                 (0x00001008U)
#define CSL_TOP_RCM_LOCK0_KICK1                                                 (0x0000100CU)
#define CSL_TOP_RCM_INTR_RAW_STATUS                                             (0x00001010U)
#define CSL_TOP_RCM_INTR_ENABLED_STATUS_CLEAR                                   (0x00001014U)
#define CSL_TOP_RCM_INTR_ENABLE                                                 (0x00001018U)
#define CSL_TOP_RCM_INTR_ENABLE_CLEAR                                           (0x0000101CU)
#define CSL_TOP_RCM_EOI                                                         (0x00001020U)
#define CSL_TOP_RCM_FAULT_ADDRESS                                               (0x00001024U)
#define CSL_TOP_RCM_FAULT_TYPE_STATUS                                           (0x00001028U)
#define CSL_TOP_RCM_FAULT_ATTR_STATUS                                           (0x0000102CU)
#define CSL_TOP_RCM_FAULT_CLEAR                                                 (0x00001030U)

/**************************************************************************
* Field Definition Macros
**************************************************************************/

/* PID */
#define CSL_TOP_RCM_PID_PID_CUSTOM_MASK                                         (0x000000C0U)
#define CSL_TOP_RCM_PID_PID_CUSTOM_SHIFT                                        (0x00000006U)
#define CSL_TOP_RCM_PID_PID_CUSTOM_RESETVAL                                     (0x00000000U)
#define CSL_TOP_RCM_PID_PID_CUSTOM_MAX                                          (0x00000003U)


#define CSL_TOP_RCM_PID_PID_MAJOR_MASK                                          (0x00000700U)
#define CSL_TOP_RCM_PID_PID_MAJOR_SHIFT                                         (0x00000008U)
#define CSL_TOP_RCM_PID_PID_MAJOR_RESETVAL                                      (0x00000002U)
#define CSL_TOP_RCM_PID_PID_MAJOR_MAX                                           (0x00000007U)


#define CSL_TOP_RCM_PID_PID_MINOR_MASK                                          (0x0000003FU)
#define CSL_TOP_RCM_PID_PID_MINOR_SHIFT                                         (0x00000000U)
#define CSL_TOP_RCM_PID_PID_MINOR_RESETVAL                                      (0x00000015U)
#define CSL_TOP_RCM_PID_PID_MINOR_MAX                                           (0x0000003FU)


#define CSL_TOP_RCM_PID_PID_MISC_MASK                                           (0x0000F800U)
#define CSL_TOP_RCM_PID_PID_MISC_SHIFT                                          (0x0000000BU)
#define CSL_TOP_RCM_PID_PID_MISC_RESETVAL                                       (0x00000000U)
#define CSL_TOP_RCM_PID_PID_MISC_MAX                                            (0x0000001FU)


#define CSL_TOP_RCM_PID_PID_MSB16_MASK                                          (0xFFFF0000U)
#define CSL_TOP_RCM_PID_PID_MSB16_SHIFT                                         (0x00000010U)
#define CSL_TOP_RCM_PID_PID_MSB16_RESETVAL                                      (0x00006180U)
#define CSL_TOP_RCM_PID_PID_MSB16_MAX                                           (0x0000FFFFU)



/* WARM_RST_CAUSE */
#define CSL_TOP_RCM_WARM_RST_CAUSE_CAUSE_MASK                      (0x07FFFFFFU)
#define CSL_TOP_RCM_WARM_RST_CAUSE_CAUSE_SHIFT                     (0x00000000U)
#define CSL_TOP_RCM_WARM_RST_CAUSE_CAUSE_RESETVAL                  (0x00000041U)
#define CSL_TOP_RCM_WARM_RST_CAUSE_CAUSE_MAX                       (0x07FFFFFFU)



/* WARM_RST_CAUSE_CLR */
#define CSL_TOP_RCM_WARM_RST_CAUSE_CLR_CLEAR_MASK              (0x00000007U)
#define CSL_TOP_RCM_WARM_RST_CAUSE_CLR_CLEAR_SHIFT             (0x00000000U)
#define CSL_TOP_RCM_WARM_RST_CAUSE_CLR_CLEAR_RESETVAL          (0x00000000U)
#define CSL_TOP_RCM_WARM_RST_CAUSE_CLR_CLEAR_MAX               (0x00000007U)



/* WARM_RESET_REQ */
#define CSL_TOP_RCM_WARM_RESET_REQ_SW_RST_MASK                   (0x00000007U)
#define CSL_TOP_RCM_WARM_RESET_REQ_SW_RST_SHIFT                  (0x00000000U)
#define CSL_TOP_RCM_WARM_RESET_REQ_SW_RST_RESETVAL               (0x00000007U)
#define CSL_TOP_RCM_WARM_RESET_REQ_SW_RST_MAX                    (0x00000007U)



/* WARM_RSTTIME1 */
#define CSL_TOP_RCM_WARM_RSTTIME1_DELAY_MASK                      (0x00000FFFU)
#define CSL_TOP_RCM_WARM_RSTTIME1_DELAY_SHIFT                     (0x00000000U)
#define CSL_TOP_RCM_WARM_RSTTIME1_DELAY_RESETVAL                  (0x00000888U)
#define CSL_TOP_RCM_WARM_RSTTIME1_DELAY_MAX                       (0x00000FFFU)



/* WARM_RSTTIME2 */
#define CSL_TOP_RCM_WARM_RSTTIME2_DELAY_MASK                      (0x00000FFFU)
#define CSL_TOP_RCM_WARM_RSTTIME2_DELAY_SHIFT                     (0x00000000U)
#define CSL_TOP_RCM_WARM_RSTTIME2_DELAY_RESETVAL                  (0x00000888U)
#define CSL_TOP_RCM_WARM_RSTTIME2_DELAY_MAX                       (0x00000FFFU)



/* WARM_RSTTIME3 */
#define CSL_TOP_RCM_WARM_RSTTIME3_DELAY_MASK                      (0x00000FFFU)
#define CSL_TOP_RCM_WARM_RSTTIME3_DELAY_SHIFT                     (0x00000000U)
#define CSL_TOP_RCM_WARM_RSTTIME3_DELAY_RESETVAL                  (0x00000111U)
#define CSL_TOP_RCM_WARM_RSTTIME3_DELAY_MAX                       (0x00000FFFU)



/* WARM_RESET_CONFIG_OV */
#define CSL_TOP_RCM_WARM_RESET_CONFIG_OV_VMON_CMP1_OV_RST_EN_MASK (0x00000007U)
#define CSL_TOP_RCM_WARM_RESET_CONFIG_OV_VMON_CMP1_OV_RST_EN_SHIFT (0x00000000U)
#define CSL_TOP_RCM_WARM_RESET_CONFIG_OV_VMON_CMP1_OV_RST_EN_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_WARM_RESET_CONFIG_OV_VMON_CMP1_OV_RST_EN_MAX (0x00000007U)


#define CSL_TOP_RCM_WARM_RESET_CONFIG_OV_VMON_CMP2_OV_RST_EN_MASK (0x00000038U)
#define CSL_TOP_RCM_WARM_RESET_CONFIG_OV_VMON_CMP2_OV_RST_EN_SHIFT (0x00000003U)
#define CSL_TOP_RCM_WARM_RESET_CONFIG_OV_VMON_CMP2_OV_RST_EN_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_WARM_RESET_CONFIG_OV_VMON_CMP2_OV_RST_EN_MAX (0x00000007U)


#define CSL_TOP_RCM_WARM_RESET_CONFIG_OV_VMON_CMP3_OV_RST_EN_MASK (0x000001C0U)
#define CSL_TOP_RCM_WARM_RESET_CONFIG_OV_VMON_CMP3_OV_RST_EN_SHIFT (0x00000006U)
#define CSL_TOP_RCM_WARM_RESET_CONFIG_OV_VMON_CMP3_OV_RST_EN_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_WARM_RESET_CONFIG_OV_VMON_CMP3_OV_RST_EN_MAX (0x00000007U)


#define CSL_TOP_RCM_WARM_RESET_CONFIG_OV_VMON_CMP5_OV_RST_EN_MASK (0x00000E00U)
#define CSL_TOP_RCM_WARM_RESET_CONFIG_OV_VMON_CMP5_OV_RST_EN_SHIFT (0x00000009U)
#define CSL_TOP_RCM_WARM_RESET_CONFIG_OV_VMON_CMP5_OV_RST_EN_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_WARM_RESET_CONFIG_OV_VMON_CMP5_OV_RST_EN_MAX (0x00000007U)



/* WARM_RESET_CONFIG_UV */
#define CSL_TOP_RCM_WARM_RESET_CONFIG_UV_VMON_CMP0_UV_RST_EN_MASK (0x00000007U)
#define CSL_TOP_RCM_WARM_RESET_CONFIG_UV_VMON_CMP0_UV_RST_EN_SHIFT (0x00000000U)
#define CSL_TOP_RCM_WARM_RESET_CONFIG_UV_VMON_CMP0_UV_RST_EN_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_WARM_RESET_CONFIG_UV_VMON_CMP0_UV_RST_EN_MAX (0x00000007U)


#define CSL_TOP_RCM_WARM_RESET_CONFIG_UV_VMON_CMP1_UV_RST_EN_MASK (0x00000038U)
#define CSL_TOP_RCM_WARM_RESET_CONFIG_UV_VMON_CMP1_UV_RST_EN_SHIFT (0x00000003U)
#define CSL_TOP_RCM_WARM_RESET_CONFIG_UV_VMON_CMP1_UV_RST_EN_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_WARM_RESET_CONFIG_UV_VMON_CMP1_UV_RST_EN_MAX (0x00000007U)


#define CSL_TOP_RCM_WARM_RESET_CONFIG_UV_VMON_CMP2_UV_RST_EN_MASK (0x000001C0U)
#define CSL_TOP_RCM_WARM_RESET_CONFIG_UV_VMON_CMP2_UV_RST_EN_SHIFT (0x00000006U)
#define CSL_TOP_RCM_WARM_RESET_CONFIG_UV_VMON_CMP2_UV_RST_EN_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_WARM_RESET_CONFIG_UV_VMON_CMP2_UV_RST_EN_MAX (0x00000007U)


#define CSL_TOP_RCM_WARM_RESET_CONFIG_UV_VMON_CMP3_UV_RST_EN_MASK (0x00000E00U)
#define CSL_TOP_RCM_WARM_RESET_CONFIG_UV_VMON_CMP3_UV_RST_EN_SHIFT (0x00000009U)
#define CSL_TOP_RCM_WARM_RESET_CONFIG_UV_VMON_CMP3_UV_RST_EN_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_WARM_RESET_CONFIG_UV_VMON_CMP3_UV_RST_EN_MAX (0x00000007U)


#define CSL_TOP_RCM_WARM_RESET_CONFIG_UV_VMON_CMP5_UV_RST_EN_MASK (0x00007000U)
#define CSL_TOP_RCM_WARM_RESET_CONFIG_UV_VMON_CMP5_UV_RST_EN_SHIFT (0x0000000CU)
#define CSL_TOP_RCM_WARM_RESET_CONFIG_UV_VMON_CMP5_UV_RST_EN_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_WARM_RESET_CONFIG_UV_VMON_CMP5_UV_RST_EN_MAX (0x00000007U)


#define CSL_TOP_RCM_WARM_RESET_CONFIG_UV_VMON_CMP7_UV_RST_EN_MASK (0x00038000U)
#define CSL_TOP_RCM_WARM_RESET_CONFIG_UV_VMON_CMP7_UV_RST_EN_SHIFT (0x0000000FU)
#define CSL_TOP_RCM_WARM_RESET_CONFIG_UV_VMON_CMP7_UV_RST_EN_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_WARM_RESET_CONFIG_UV_VMON_CMP7_UV_RST_EN_MAX (0x00000007U)


#define CSL_TOP_RCM_WARM_RESET_CONFIG_UV_VMON_CMP8_UV_RST_EN_MASK (0x001C0000U)
#define CSL_TOP_RCM_WARM_RESET_CONFIG_UV_VMON_CMP8_UV_RST_EN_SHIFT (0x00000012U)
#define CSL_TOP_RCM_WARM_RESET_CONFIG_UV_VMON_CMP8_UV_RST_EN_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_WARM_RESET_CONFIG_UV_VMON_CMP8_UV_RST_EN_MAX (0x00000007U)



/* WARM_RESET_CONFIG_MISC */
#define CSL_TOP_RCM_WARM_RESET_CONFIG_MISC_CLK_LOSS_SYS_CLK_RST_EN_MASK (0x00000038U)
#define CSL_TOP_RCM_WARM_RESET_CONFIG_MISC_CLK_LOSS_SYS_CLK_RST_EN_SHIFT (0x00000003U)
#define CSL_TOP_RCM_WARM_RESET_CONFIG_MISC_CLK_LOSS_SYS_CLK_RST_EN_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_WARM_RESET_CONFIG_MISC_CLK_LOSS_SYS_CLK_RST_EN_MAX (0x00000007U)


#define CSL_TOP_RCM_WARM_RESET_CONFIG_MISC_ESM_CRI_INTR_RST_EN_MASK (0x00000007U)
#define CSL_TOP_RCM_WARM_RESET_CONFIG_MISC_ESM_CRI_INTR_RST_EN_SHIFT (0x00000000U)
#define CSL_TOP_RCM_WARM_RESET_CONFIG_MISC_ESM_CRI_INTR_RST_EN_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_WARM_RESET_CONFIG_MISC_ESM_CRI_INTR_RST_EN_MAX (0x00000007U)


#define CSL_TOP_RCM_WARM_RESET_CONFIG_MISC_ESM_ERR_PIN_INTR_RST_EN_MASK (0x00000E00U)
#define CSL_TOP_RCM_WARM_RESET_CONFIG_MISC_ESM_ERR_PIN_INTR_RST_EN_SHIFT (0x00000009U)
#define CSL_TOP_RCM_WARM_RESET_CONFIG_MISC_ESM_ERR_PIN_INTR_RST_EN_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_WARM_RESET_CONFIG_MISC_ESM_ERR_PIN_INTR_RST_EN_MAX (0x00000007U)


#define CSL_TOP_RCM_WARM_RESET_CONFIG_MISC_ESM_HIGH_PRI_WD_INTR_RST_EN_MASK (0x000001C0U)
#define CSL_TOP_RCM_WARM_RESET_CONFIG_MISC_ESM_HIGH_PRI_WD_INTR_RST_EN_SHIFT (0x00000006U)
#define CSL_TOP_RCM_WARM_RESET_CONFIG_MISC_ESM_HIGH_PRI_WD_INTR_RST_EN_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_WARM_RESET_CONFIG_MISC_ESM_HIGH_PRI_WD_INTR_RST_EN_MAX (0x00000007U)


/* WARM_RESET_CONFIG */
#define CSL_TOP_RCM_WARM_RESET_CONFIG_PAD_BYPASS_MASK         (0x00000007U)
#define CSL_TOP_RCM_WARM_RESET_CONFIG_PAD_BYPASS_SHIFT        (0x00000000U)
#define CSL_TOP_RCM_WARM_RESET_CONFIG_PAD_BYPASS_RESETVAL     (0x00000007U)
#define CSL_TOP_RCM_WARM_RESET_CONFIG_PAD_BYPASS_MAX          (0x00000007U)

#define CSL_TOP_RCM_WARM_RESET_CONFIG_DEBUGSS_RST_EN_MASK    (0x00000070U)
#define CSL_TOP_RCM_WARM_RESET_CONFIG_DEBUGSS_RST_EN_SHIFT   (0x00000004U)
#define CSL_TOP_RCM_WARM_RESET_CONFIG_DEBUGSS_RST_EN_RESETVAL (0x00000007U)
#define CSL_TOP_RCM_WARM_RESET_CONFIG_DEBUGSS_RST_EN_MAX     (0x00000007U)

#define CSL_TOP_RCM_WARM_RESET_CONFIG_TSENSE0_RST_EN_MASK     (0x00000700U)
#define CSL_TOP_RCM_WARM_RESET_CONFIG_TSENSE0_RST_EN_SHIFT    (0x00000008U)
#define CSL_TOP_RCM_WARM_RESET_CONFIG_TSENSE0_RST_EN_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_WARM_RESET_CONFIG_TSENSE0_RST_EN_MAX      (0x00000007U)


#define CSL_TOP_RCM_WARM_RESET_CONFIG_TSENSE1_RST_EN_MASK     (0x00007000U)
#define CSL_TOP_RCM_WARM_RESET_CONFIG_TSENSE1_RST_EN_SHIFT    (0x0000000CU)
#define CSL_TOP_RCM_WARM_RESET_CONFIG_TSENSE1_RST_EN_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_WARM_RESET_CONFIG_TSENSE1_RST_EN_MAX      (0x00000007U)


#define CSL_TOP_RCM_WARM_RESET_CONFIG_WDOG0_RST_EN_MASK       (0x00070000U)
#define CSL_TOP_RCM_WARM_RESET_CONFIG_WDOG0_RST_EN_SHIFT      (0x00000010U)
#define CSL_TOP_RCM_WARM_RESET_CONFIG_WDOG0_RST_EN_RESETVAL   (0x00000007U)
#define CSL_TOP_RCM_WARM_RESET_CONFIG_WDOG0_RST_EN_MAX        (0x00000007U)


#define CSL_TOP_RCM_WARM_RESET_CONFIG_WDOG1_RST_EN_MASK       (0x00700000U)
#define CSL_TOP_RCM_WARM_RESET_CONFIG_WDOG1_RST_EN_SHIFT      (0x00000014U)
#define CSL_TOP_RCM_WARM_RESET_CONFIG_WDOG1_RST_EN_RESETVAL   (0x00000007U)
#define CSL_TOP_RCM_WARM_RESET_CONFIG_WDOG1_RST_EN_MAX        (0x00000007U)



/* SW_POR */
#define CSL_TOP_RCM_SW_POR_SW_POR_ASSERT_MASK                                   (0x00000007U)
#define CSL_TOP_RCM_SW_POR_SW_POR_ASSERT_SHIFT                                  (0x00000000U)
#define CSL_TOP_RCM_SW_POR_SW_POR_ASSERT_RESETVAL                               (0x00000000U)
#define CSL_TOP_RCM_SW_POR_SW_POR_ASSERT_MAX                                    (0x00000007U)



/* LIMP_MODE_EN */
#define CSL_TOP_RCM_LIMP_MODE_EN_COREPLL_LOSS_EN_MASK              (0x00000700U)
#define CSL_TOP_RCM_LIMP_MODE_EN_COREPLL_LOSS_EN_SHIFT             (0x00000008U)
#define CSL_TOP_RCM_LIMP_MODE_EN_COREPLL_LOSS_EN_RESETVAL          (0x00000000U)
#define CSL_TOP_RCM_LIMP_MODE_EN_COREPLL_LOSS_EN_MAX               (0x00000007U)


#define CSL_TOP_RCM_LIMP_MODE_EN_DCC0_ERROR_EN_MASK                (0x00000007U)
#define CSL_TOP_RCM_LIMP_MODE_EN_DCC0_ERROR_EN_SHIFT               (0x00000000U)
#define CSL_TOP_RCM_LIMP_MODE_EN_DCC0_ERROR_EN_RESETVAL            (0x00000000U)
#define CSL_TOP_RCM_LIMP_MODE_EN_DCC0_ERROR_EN_MAX                 (0x00000007U)


#define CSL_TOP_RCM_LIMP_MODE_EN_XTALCLK_LOSS_EN_MASK              (0x00000070U)
#define CSL_TOP_RCM_LIMP_MODE_EN_XTALCLK_LOSS_EN_SHIFT             (0x00000004U)
#define CSL_TOP_RCM_LIMP_MODE_EN_XTALCLK_LOSS_EN_RESETVAL          (0x00000000U)
#define CSL_TOP_RCM_LIMP_MODE_EN_XTALCLK_LOSS_EN_MAX               (0x00000007U)



/* CLK_LOSS_STATUS */
#define CSL_TOP_RCM_CLK_LOSS_STATUS_CRYSTAL_CLOCK_LOSS_MASK     (0x00000001U)
#define CSL_TOP_RCM_CLK_LOSS_STATUS_CRYSTAL_CLOCK_LOSS_SHIFT    (0x00000000U)
#define CSL_TOP_RCM_CLK_LOSS_STATUS_CRYSTAL_CLOCK_LOSS_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_CLK_LOSS_STATUS_CRYSTAL_CLOCK_LOSS_MAX      (0x00000001U)


#define CSL_TOP_RCM_CLK_LOSS_STATUS_RC_CLOCK_LOSS_MASK          (0x00000010U)
#define CSL_TOP_RCM_CLK_LOSS_STATUS_RC_CLOCK_LOSS_SHIFT         (0x00000004U)
#define CSL_TOP_RCM_CLK_LOSS_STATUS_RC_CLOCK_LOSS_RESETVAL      (0x00000000U)
#define CSL_TOP_RCM_CLK_LOSS_STATUS_RC_CLOCK_LOSS_MAX           (0x00000001U)


#define CSL_TOP_RCM_CLK_LOSS_STATUS_RC_GOOD_BOOT_MASK           (0x00000100U)
#define CSL_TOP_RCM_CLK_LOSS_STATUS_RC_GOOD_BOOT_SHIFT          (0x00000008U)
#define CSL_TOP_RCM_CLK_LOSS_STATUS_RC_GOOD_BOOT_RESETVAL       (0x00000001U)
#define CSL_TOP_RCM_CLK_LOSS_STATUS_RC_GOOD_BOOT_MAX            (0x00000001U)



/* RCOSC32K_CTRL */
#define CSL_TOP_RCM_RCOSC32K_CTRL_RCOSC32K_CTRL_STOPOSC_MASK                    (0x00000007U)
#define CSL_TOP_RCM_RCOSC32K_CTRL_RCOSC32K_CTRL_STOPOSC_SHIFT                   (0x00000000U)
#define CSL_TOP_RCM_RCOSC32K_CTRL_RCOSC32K_CTRL_STOPOSC_RESETVAL                (0x00000000U)
#define CSL_TOP_RCM_RCOSC32K_CTRL_RCOSC32K_CTRL_STOPOSC_MAX                     (0x00000007U)



/* PLL_REF_CLK_SRC_SEL */
#define CSL_TOP_RCM_PLL_REF_CLK_SRC_SEL_PLL_CORE_REF_CLK_SRC_SEL_MASK (0x00000007U)
#define CSL_TOP_RCM_PLL_REF_CLK_SRC_SEL_PLL_CORE_REF_CLK_SRC_SEL_SHIFT (0x00000000U)
#define CSL_TOP_RCM_PLL_REF_CLK_SRC_SEL_PLL_CORE_REF_CLK_SRC_SEL_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_PLL_REF_CLK_SRC_SEL_PLL_CORE_REF_CLK_SRC_SEL_MAX (0x00000007U)


#define CSL_TOP_RCM_PLL_REF_CLK_SRC_SEL_PLL_ETH_REF_CLK_SRC_SEL_MASK (0x00000700U)
#define CSL_TOP_RCM_PLL_REF_CLK_SRC_SEL_PLL_ETH_REF_CLK_SRC_SEL_SHIFT (0x00000008U)
#define CSL_TOP_RCM_PLL_REF_CLK_SRC_SEL_PLL_ETH_REF_CLK_SRC_SEL_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_PLL_REF_CLK_SRC_SEL_PLL_ETH_REF_CLK_SRC_SEL_MAX (0x00000007U)


#define CSL_TOP_RCM_PLL_REF_CLK_SRC_SEL_PLL_PERI_REF_CLK_SRC_SEL_MASK (0x00000070U)
#define CSL_TOP_RCM_PLL_REF_CLK_SRC_SEL_PLL_PERI_REF_CLK_SRC_SEL_SHIFT (0x00000004U)
#define CSL_TOP_RCM_PLL_REF_CLK_SRC_SEL_PLL_PERI_REF_CLK_SRC_SEL_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_PLL_REF_CLK_SRC_SEL_PLL_PERI_REF_CLK_SRC_SEL_MAX (0x00000007U)



/* PAD_XTAL_CTRL */
#define CSL_TOP_RCM_PAD_XTAL_CTRL_PAD_XTAL_CTRL_XTAL_XI_OE_N_MASK               (0x00000008U)
#define CSL_TOP_RCM_PAD_XTAL_CTRL_PAD_XTAL_CTRL_XTAL_XI_OE_N_SHIFT              (0x00000003U)
#define CSL_TOP_RCM_PAD_XTAL_CTRL_PAD_XTAL_CTRL_XTAL_XI_OE_N_RESETVAL           (0x00000000U)
#define CSL_TOP_RCM_PAD_XTAL_CTRL_PAD_XTAL_CTRL_XTAL_XI_OE_N_MAX                (0x00000001U)


#define CSL_TOP_RCM_PAD_XTAL_CTRL_PAD_XTAL_CTRL_XTAL_XO_RESSELECT_MASK          (0x00000004U)
#define CSL_TOP_RCM_PAD_XTAL_CTRL_PAD_XTAL_CTRL_XTAL_XO_RESSELECT_SHIFT         (0x00000002U)
#define CSL_TOP_RCM_PAD_XTAL_CTRL_PAD_XTAL_CTRL_XTAL_XO_RESSELECT_RESETVAL      (0x00000000U)
#define CSL_TOP_RCM_PAD_XTAL_CTRL_PAD_XTAL_CTRL_XTAL_XO_RESSELECT_MAX           (0x00000001U)


#define CSL_TOP_RCM_PAD_XTAL_CTRL_PAD_XTAL_CTRL_XTAL_XO_SW1_MASK                (0x00000001U)
#define CSL_TOP_RCM_PAD_XTAL_CTRL_PAD_XTAL_CTRL_XTAL_XO_SW1_SHIFT               (0x00000000U)
#define CSL_TOP_RCM_PAD_XTAL_CTRL_PAD_XTAL_CTRL_XTAL_XO_SW1_RESETVAL            (0x00000001U)
#define CSL_TOP_RCM_PAD_XTAL_CTRL_PAD_XTAL_CTRL_XTAL_XO_SW1_MAX                 (0x00000001U)


#define CSL_TOP_RCM_PAD_XTAL_CTRL_PAD_XTAL_CTRL_XTAL_XO_SW2_MASK                (0x00000002U)
#define CSL_TOP_RCM_PAD_XTAL_CTRL_PAD_XTAL_CTRL_XTAL_XO_SW2_SHIFT               (0x00000001U)
#define CSL_TOP_RCM_PAD_XTAL_CTRL_PAD_XTAL_CTRL_XTAL_XO_SW2_RESETVAL            (0x00000000U)
#define CSL_TOP_RCM_PAD_XTAL_CTRL_PAD_XTAL_CTRL_XTAL_XO_SW2_MAX                 (0x00000001U)



/* SOP_MODE_VALUE */
#define CSL_TOP_RCM_SOP_MODE_VALUE_SOP_MODE_VALUE_VAL_MASK                      (0xFFFFFFFFU)
#define CSL_TOP_RCM_SOP_MODE_VALUE_SOP_MODE_VALUE_VAL_SHIFT                     (0x00000000U)
#define CSL_TOP_RCM_SOP_MODE_VALUE_SOP_MODE_VALUE_VAL_RESETVAL                  (0x00000000U)
#define CSL_TOP_RCM_SOP_MODE_VALUE_SOP_MODE_VALUE_VAL_MAX                       (0xFFFFFFFFU)



/* VMON_CLK_GATE */
#define CSL_TOP_RCM_VMON_CLK_GATE_VMON_CLK_GATE_ENABLE_MASK                     (0x00000007U)
#define CSL_TOP_RCM_VMON_CLK_GATE_VMON_CLK_GATE_ENABLE_SHIFT                    (0x00000000U)
#define CSL_TOP_RCM_VMON_CLK_GATE_VMON_CLK_GATE_ENABLE_RESETVAL                 (0x00000000U)
#define CSL_TOP_RCM_VMON_CLK_GATE_VMON_CLK_GATE_ENABLE_MAX                      (0x00000007U)



/* PLL_CORE_PWRCTRL */
#define CSL_TOP_RCM_PLL_CORE_PWRCTRL_PLL_CORE_PWRCTRL_ISORET_MASK               (0x00000004U)
#define CSL_TOP_RCM_PLL_CORE_PWRCTRL_PLL_CORE_PWRCTRL_ISORET_SHIFT              (0x00000002U)
#define CSL_TOP_RCM_PLL_CORE_PWRCTRL_PLL_CORE_PWRCTRL_ISORET_RESETVAL           (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_PWRCTRL_PLL_CORE_PWRCTRL_ISORET_MAX                (0x00000001U)


#define CSL_TOP_RCM_PLL_CORE_PWRCTRL_PLL_CORE_PWRCTRL_ISOSCAN_MASK              (0x00000002U)
#define CSL_TOP_RCM_PLL_CORE_PWRCTRL_PLL_CORE_PWRCTRL_ISOSCAN_SHIFT             (0x00000001U)
#define CSL_TOP_RCM_PLL_CORE_PWRCTRL_PLL_CORE_PWRCTRL_ISOSCAN_RESETVAL          (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_PWRCTRL_PLL_CORE_PWRCTRL_ISOSCAN_MAX               (0x00000001U)


#define CSL_TOP_RCM_PLL_CORE_PWRCTRL_PLL_CORE_PWRCTRL_OFFMODE_MASK              (0x00000001U)
#define CSL_TOP_RCM_PLL_CORE_PWRCTRL_PLL_CORE_PWRCTRL_OFFMODE_SHIFT             (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_PWRCTRL_PLL_CORE_PWRCTRL_OFFMODE_RESETVAL          (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_PWRCTRL_PLL_CORE_PWRCTRL_OFFMODE_MAX               (0x00000001U)


#define CSL_TOP_RCM_PLL_CORE_PWRCTRL_PLL_CORE_PWRCTRL_PGOODIN_MASK              (0x00000010U)
#define CSL_TOP_RCM_PLL_CORE_PWRCTRL_PLL_CORE_PWRCTRL_PGOODIN_SHIFT             (0x00000004U)
#define CSL_TOP_RCM_PLL_CORE_PWRCTRL_PLL_CORE_PWRCTRL_PGOODIN_RESETVAL          (0x00000001U)
#define CSL_TOP_RCM_PLL_CORE_PWRCTRL_PLL_CORE_PWRCTRL_PGOODIN_MAX               (0x00000001U)


#define CSL_TOP_RCM_PLL_CORE_PWRCTRL_PLL_CORE_PWRCTRL_PONIN_MASK                (0x00000020U)
#define CSL_TOP_RCM_PLL_CORE_PWRCTRL_PLL_CORE_PWRCTRL_PONIN_SHIFT               (0x00000005U)
#define CSL_TOP_RCM_PLL_CORE_PWRCTRL_PLL_CORE_PWRCTRL_PONIN_RESETVAL            (0x00000001U)
#define CSL_TOP_RCM_PLL_CORE_PWRCTRL_PLL_CORE_PWRCTRL_PONIN_MAX                 (0x00000001U)


#define CSL_TOP_RCM_PLL_CORE_PWRCTRL_PLL_CORE_PWRCTRL_RET_MASK                  (0x00000008U)
#define CSL_TOP_RCM_PLL_CORE_PWRCTRL_PLL_CORE_PWRCTRL_RET_SHIFT                 (0x00000003U)
#define CSL_TOP_RCM_PLL_CORE_PWRCTRL_PLL_CORE_PWRCTRL_RET_RESETVAL              (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_PWRCTRL_PLL_CORE_PWRCTRL_RET_MAX                   (0x00000001U)



/* PLL_CORE_CLKCTRL */
#define CSL_TOP_RCM_PLL_CORE_CLKCTRL_BYPASSACKZ_MASK           (0x00400000U)
#define CSL_TOP_RCM_PLL_CORE_CLKCTRL_BYPASSACKZ_SHIFT          (0x00000016U)
#define CSL_TOP_RCM_PLL_CORE_CLKCTRL_BYPASSACKZ_RESETVAL       (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_CLKCTRL_BYPASSACKZ_MAX            (0x00000001U)


#define CSL_TOP_RCM_PLL_CORE_CLKCTRL_CLKDCOLDOEN_MASK          (0x20000000U)
#define CSL_TOP_RCM_PLL_CORE_CLKCTRL_CLKDCOLDOEN_SHIFT         (0x0000001DU)
#define CSL_TOP_RCM_PLL_CORE_CLKCTRL_CLKDCOLDOEN_RESETVAL      (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_CLKCTRL_CLKDCOLDOEN_MAX           (0x00000001U)


#define CSL_TOP_RCM_PLL_CORE_CLKCTRL_CLKDCOLDOPWDNZ_MASK       (0x00020000U)
#define CSL_TOP_RCM_PLL_CORE_CLKCTRL_CLKDCOLDOPWDNZ_SHIFT      (0x00000011U)
#define CSL_TOP_RCM_PLL_CORE_CLKCTRL_CLKDCOLDOPWDNZ_RESETVAL   (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_CLKCTRL_CLKDCOLDOPWDNZ_MAX        (0x00000001U)


#define CSL_TOP_RCM_PLL_CORE_CLKCTRL_CLKOUTEN_MASK             (0x00100000U)
#define CSL_TOP_RCM_PLL_CORE_CLKCTRL_CLKOUTEN_SHIFT            (0x00000014U)
#define CSL_TOP_RCM_PLL_CORE_CLKCTRL_CLKOUTEN_RESETVAL         (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_CLKCTRL_CLKOUTEN_MAX              (0x00000001U)


#define CSL_TOP_RCM_PLL_CORE_CLKCTRL_CLKOUTLDOEN_MASK          (0x00080000U)
#define CSL_TOP_RCM_PLL_CORE_CLKCTRL_CLKOUTLDOEN_SHIFT         (0x00000013U)
#define CSL_TOP_RCM_PLL_CORE_CLKCTRL_CLKOUTLDOEN_RESETVAL      (0x00000001U)
#define CSL_TOP_RCM_PLL_CORE_CLKCTRL_CLKOUTLDOEN_MAX           (0x00000001U)


#define CSL_TOP_RCM_PLL_CORE_CLKCTRL_CYCLESLIPEN_MASK          (0x80000000U)
#define CSL_TOP_RCM_PLL_CORE_CLKCTRL_CYCLESLIPEN_SHIFT         (0x0000001FU)
#define CSL_TOP_RCM_PLL_CORE_CLKCTRL_CYCLESLIPEN_RESETVAL      (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_CLKCTRL_CYCLESLIPEN_MAX           (0x00000001U)


#define CSL_TOP_RCM_PLL_CORE_CLKCTRL_ENSSC_MASK                (0x40000000U)
#define CSL_TOP_RCM_PLL_CORE_CLKCTRL_ENSSC_SHIFT               (0x0000001EU)
#define CSL_TOP_RCM_PLL_CORE_CLKCTRL_ENSSC_RESETVAL            (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_CLKCTRL_ENSSC_MAX                 (0x00000001U)


#define CSL_TOP_RCM_PLL_CORE_CLKCTRL_IDLE_MASK                 (0x00800000U)
#define CSL_TOP_RCM_PLL_CORE_CLKCTRL_IDLE_SHIFT                (0x00000017U)
#define CSL_TOP_RCM_PLL_CORE_CLKCTRL_IDLE_RESETVAL             (0x00000001U)
#define CSL_TOP_RCM_PLL_CORE_CLKCTRL_IDLE_MAX                  (0x00000001U)


#define CSL_TOP_RCM_PLL_CORE_CLKCTRL_M2PWDNZ_MASK              (0x00010000U)
#define CSL_TOP_RCM_PLL_CORE_CLKCTRL_M2PWDNZ_SHIFT             (0x00000010U)
#define CSL_TOP_RCM_PLL_CORE_CLKCTRL_M2PWDNZ_RESETVAL          (0x00000001U)
#define CSL_TOP_RCM_PLL_CORE_CLKCTRL_M2PWDNZ_MAX               (0x00000001U)


#define CSL_TOP_RCM_PLL_CORE_CLKCTRL_RELAXED_LOCK_MASK         (0x00000100U)
#define CSL_TOP_RCM_PLL_CORE_CLKCTRL_RELAXED_LOCK_SHIFT        (0x00000008U)
#define CSL_TOP_RCM_PLL_CORE_CLKCTRL_RELAXED_LOCK_RESETVAL     (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_CLKCTRL_RELAXED_LOCK_MAX          (0x00000001U)


#define CSL_TOP_RCM_PLL_CORE_CLKCTRL_SELFREQDCO_MASK           (0x00001C00U)
#define CSL_TOP_RCM_PLL_CORE_CLKCTRL_SELFREQDCO_SHIFT          (0x0000000AU)
#define CSL_TOP_RCM_PLL_CORE_CLKCTRL_SELFREQDCO_RESETVAL       (0x00000004U)
#define CSL_TOP_RCM_PLL_CORE_CLKCTRL_SELFREQDCO_MAX            (0x00000007U)


#define CSL_TOP_RCM_PLL_CORE_CLKCTRL_SSCTYPE_MASK              (0x00000002U)
#define CSL_TOP_RCM_PLL_CORE_CLKCTRL_SSCTYPE_SHIFT             (0x00000001U)
#define CSL_TOP_RCM_PLL_CORE_CLKCTRL_SSCTYPE_RESETVAL          (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_CLKCTRL_SSCTYPE_MAX               (0x00000001U)


#define CSL_TOP_RCM_PLL_CORE_CLKCTRL_STBYRET_MASK              (0x00200000U)
#define CSL_TOP_RCM_PLL_CORE_CLKCTRL_STBYRET_SHIFT             (0x00000015U)
#define CSL_TOP_RCM_PLL_CORE_CLKCTRL_STBYRET_RESETVAL          (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_CLKCTRL_STBYRET_MAX               (0x00000001U)


#define CSL_TOP_RCM_PLL_CORE_CLKCTRL_STOPMODE_MASK             (0x00004000U)
#define CSL_TOP_RCM_PLL_CORE_CLKCTRL_STOPMODE_SHIFT            (0x0000000EU)
#define CSL_TOP_RCM_PLL_CORE_CLKCTRL_STOPMODE_RESETVAL         (0x00000001U)
#define CSL_TOP_RCM_PLL_CORE_CLKCTRL_STOPMODE_MAX              (0x00000001U)


#define CSL_TOP_RCM_PLL_CORE_CLKCTRL_TINTZ_MASK                (0x00000001U)
#define CSL_TOP_RCM_PLL_CORE_CLKCTRL_TINTZ_SHIFT               (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_CLKCTRL_TINTZ_RESETVAL            (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_CLKCTRL_TINTZ_MAX                 (0x00000001U)


#define CSL_TOP_RCM_PLL_CORE_CLKCTRL_ULOWCLKEN_MASK            (0x00040000U)
#define CSL_TOP_RCM_PLL_CORE_CLKCTRL_ULOWCLKEN_SHIFT           (0x00000012U)
#define CSL_TOP_RCM_PLL_CORE_CLKCTRL_ULOWCLKEN_RESETVAL        (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_CLKCTRL_ULOWCLKEN_MAX             (0x00000001U)



/* PLL_CORE_TENABLE */
#define CSL_TOP_RCM_PLL_CORE_TENABLE_PLL_CORE_TENABLE_TENABLE_MASK              (0x00000001U)
#define CSL_TOP_RCM_PLL_CORE_TENABLE_PLL_CORE_TENABLE_TENABLE_SHIFT             (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_TENABLE_PLL_CORE_TENABLE_TENABLE_RESETVAL          (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_TENABLE_PLL_CORE_TENABLE_TENABLE_MAX               (0x00000001U)



/* PLL_CORE_TENABLEDIV */
#define CSL_TOP_RCM_PLL_CORE_TENABLEDIV_TENABLEDIV_MASK     (0x00000001U)
#define CSL_TOP_RCM_PLL_CORE_TENABLEDIV_TENABLEDIV_SHIFT    (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_TENABLEDIV_TENABLEDIV_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_TENABLEDIV_TENABLEDIV_MAX      (0x00000001U)



/* PLL_CORE_M2NDIV */
#define CSL_TOP_RCM_PLL_CORE_M2NDIV_M2_MASK                     (0x007F0000U)
#define CSL_TOP_RCM_PLL_CORE_M2NDIV_M2_SHIFT                    (0x00000010U)
#define CSL_TOP_RCM_PLL_CORE_M2NDIV_M2_RESETVAL                 (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_M2NDIV_M2_MAX                      (0x0000007FU)


#define CSL_TOP_RCM_PLL_CORE_M2NDIV_N_MASK                      (0x000000FFU)
#define CSL_TOP_RCM_PLL_CORE_M2NDIV_N_SHIFT                     (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_M2NDIV_N_RESETVAL                  (0x00000013U)
#define CSL_TOP_RCM_PLL_CORE_M2NDIV_N_MAX                       (0x000000FFU)



/* PLL_CORE_MN2DIV */
#define CSL_TOP_RCM_PLL_CORE_MN2DIV_M_MASK                      (0x00000FFFU)
#define CSL_TOP_RCM_PLL_CORE_MN2DIV_M_SHIFT                     (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_MN2DIV_M_RESETVAL                  (0x00000640U)
#define CSL_TOP_RCM_PLL_CORE_MN2DIV_M_MAX                       (0x00000FFFU)


#define CSL_TOP_RCM_PLL_CORE_MN2DIV_N2_MASK                     (0x000F0000U)
#define CSL_TOP_RCM_PLL_CORE_MN2DIV_N2_SHIFT                    (0x00000010U)
#define CSL_TOP_RCM_PLL_CORE_MN2DIV_N2_RESETVAL                 (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_MN2DIV_N2_MAX                      (0x0000000FU)



/* PLL_CORE_FRACDIV */
#define CSL_TOP_RCM_PLL_CORE_FRACDIV_FRACTIONALM_MASK          (0x0003FFFFU)
#define CSL_TOP_RCM_PLL_CORE_FRACDIV_FRACTIONALM_SHIFT         (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_FRACDIV_FRACTIONALM_RESETVAL      (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_FRACDIV_FRACTIONALM_MAX           (0x0003FFFFU)


#define CSL_TOP_RCM_PLL_CORE_FRACDIV_REGSD_MASK                (0xFF000000U)
#define CSL_TOP_RCM_PLL_CORE_FRACDIV_REGSD_SHIFT               (0x00000018U)
#define CSL_TOP_RCM_PLL_CORE_FRACDIV_REGSD_RESETVAL            (0x00000008U)
#define CSL_TOP_RCM_PLL_CORE_FRACDIV_REGSD_MAX                 (0x000000FFU)



/* PLL_CORE_BWCTRL */
#define CSL_TOP_RCM_PLL_CORE_BWCTRL_BWCONTROL_MASK              (0x00000006U)
#define CSL_TOP_RCM_PLL_CORE_BWCTRL_BWCONTROL_SHIFT             (0x00000001U)
#define CSL_TOP_RCM_PLL_CORE_BWCTRL_BWCONTROL_RESETVAL          (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_BWCTRL_BWCONTROL_MAX               (0x00000003U)


#define CSL_TOP_RCM_PLL_CORE_BWCTRL_BW_INCR_DECRZ_MASK          (0x00000001U)
#define CSL_TOP_RCM_PLL_CORE_BWCTRL_BW_INCR_DECRZ_SHIFT         (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_BWCTRL_BW_INCR_DECRZ_RESETVAL      (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_BWCTRL_BW_INCR_DECRZ_MAX           (0x00000001U)



/* PLL_CORE_FRACCTRL */
#define CSL_TOP_RCM_PLL_CORE_FRACCTRL_DOWNSPREAD_MASK         (0x80000000U)
#define CSL_TOP_RCM_PLL_CORE_FRACCTRL_DOWNSPREAD_SHIFT        (0x0000001FU)
#define CSL_TOP_RCM_PLL_CORE_FRACCTRL_DOWNSPREAD_RESETVAL     (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_FRACCTRL_DOWNSPREAD_MAX          (0x00000001U)


#define CSL_TOP_RCM_PLL_CORE_FRACCTRL_DELTAMSTEPFRACTION_MASK (0x0003FFFFU)
#define CSL_TOP_RCM_PLL_CORE_FRACCTRL_DELTAMSTEPFRACTION_SHIFT (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_FRACCTRL_DELTAMSTEPFRACTION_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_FRACCTRL_DELTAMSTEPFRACTION_MAX  (0x0003FFFFU)


#define CSL_TOP_RCM_PLL_CORE_FRACCTRL_DELTAMSTEPINTEGER_MASK  (0x001C0000U)
#define CSL_TOP_RCM_PLL_CORE_FRACCTRL_DELTAMSTEPINTEGER_SHIFT (0x00000012U)
#define CSL_TOP_RCM_PLL_CORE_FRACCTRL_DELTAMSTEPINTEGER_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_FRACCTRL_DELTAMSTEPINTEGER_MAX   (0x00000007U)


#define CSL_TOP_RCM_PLL_CORE_FRACCTRL_MODFREQDIVIDEREXPONENT_MASK (0x70000000U)
#define CSL_TOP_RCM_PLL_CORE_FRACCTRL_MODFREQDIVIDEREXPONENT_SHIFT (0x0000001CU)
#define CSL_TOP_RCM_PLL_CORE_FRACCTRL_MODFREQDIVIDEREXPONENT_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_FRACCTRL_MODFREQDIVIDEREXPONENT_MAX (0x00000007U)


#define CSL_TOP_RCM_PLL_CORE_FRACCTRL_MODFREQDIVIDERMANTISSA_MASK (0x0FE00000U)
#define CSL_TOP_RCM_PLL_CORE_FRACCTRL_MODFREQDIVIDERMANTISSA_SHIFT (0x00000015U)
#define CSL_TOP_RCM_PLL_CORE_FRACCTRL_MODFREQDIVIDERMANTISSA_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_FRACCTRL_MODFREQDIVIDERMANTISSA_MAX (0x0000007FU)



/* PLL_CORE_STATUS */
#define CSL_TOP_RCM_PLL_CORE_STATUS_BYPASS_MASK                 (0x00000001U)
#define CSL_TOP_RCM_PLL_CORE_STATUS_BYPASS_SHIFT                (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_STATUS_BYPASS_RESETVAL             (0x00000001U)
#define CSL_TOP_RCM_PLL_CORE_STATUS_BYPASS_MAX                  (0x00000001U)


#define CSL_TOP_RCM_PLL_CORE_STATUS_BYPASSACK_MASK              (0x00000100U)
#define CSL_TOP_RCM_PLL_CORE_STATUS_BYPASSACK_SHIFT             (0x00000008U)
#define CSL_TOP_RCM_PLL_CORE_STATUS_BYPASSACK_RESETVAL          (0x00000001U)
#define CSL_TOP_RCM_PLL_CORE_STATUS_BYPASSACK_MAX               (0x00000001U)


#define CSL_TOP_RCM_PLL_CORE_STATUS_CLKDCOLDOACK_MASK           (0x00000800U)
#define CSL_TOP_RCM_PLL_CORE_STATUS_CLKDCOLDOACK_SHIFT          (0x0000000BU)
#define CSL_TOP_RCM_PLL_CORE_STATUS_CLKDCOLDOACK_RESETVAL       (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_STATUS_CLKDCOLDOACK_MAX            (0x00000001U)


#define CSL_TOP_RCM_PLL_CORE_STATUS_CLKOUTENACK_MASK            (0x00000020U)
#define CSL_TOP_RCM_PLL_CORE_STATUS_CLKOUTENACK_SHIFT           (0x00000005U)
#define CSL_TOP_RCM_PLL_CORE_STATUS_CLKOUTENACK_RESETVAL        (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_STATUS_CLKOUTENACK_MAX             (0x00000001U)


#define CSL_TOP_RCM_PLL_CORE_STATUS_CLKOUTLDOENACK_MASK         (0x00001000U)
#define CSL_TOP_RCM_PLL_CORE_STATUS_CLKOUTLDOENACK_SHIFT        (0x0000000CU)
#define CSL_TOP_RCM_PLL_CORE_STATUS_CLKOUTLDOENACK_RESETVAL     (0x00000001U)
#define CSL_TOP_RCM_PLL_CORE_STATUS_CLKOUTLDOENACK_MAX          (0x00000001U)


#define CSL_TOP_RCM_PLL_CORE_STATUS_FREQLOCK_MASK               (0x00000200U)
#define CSL_TOP_RCM_PLL_CORE_STATUS_FREQLOCK_SHIFT              (0x00000009U)
#define CSL_TOP_RCM_PLL_CORE_STATUS_FREQLOCK_RESETVAL           (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_STATUS_FREQLOCK_MAX                (0x00000001U)


#define CSL_TOP_RCM_PLL_CORE_STATUS_HIGHJITTER_MASK             (0x00000002U)
#define CSL_TOP_RCM_PLL_CORE_STATUS_HIGHJITTER_SHIFT            (0x00000001U)
#define CSL_TOP_RCM_PLL_CORE_STATUS_HIGHJITTER_RESETVAL         (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_STATUS_HIGHJITTER_MAX              (0x00000001U)


#define CSL_TOP_RCM_PLL_CORE_STATUS_LDOPWDN_MASK                (0x20000000U)
#define CSL_TOP_RCM_PLL_CORE_STATUS_LDOPWDN_SHIFT               (0x0000001DU)
#define CSL_TOP_RCM_PLL_CORE_STATUS_LDOPWDN_RESETVAL            (0x00000001U)
#define CSL_TOP_RCM_PLL_CORE_STATUS_LDOPWDN_MAX                 (0x00000001U)


#define CSL_TOP_RCM_PLL_CORE_STATUS_LOCK2_MASK                  (0x00000010U)
#define CSL_TOP_RCM_PLL_CORE_STATUS_LOCK2_SHIFT                 (0x00000004U)
#define CSL_TOP_RCM_PLL_CORE_STATUS_LOCK2_RESETVAL              (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_STATUS_LOCK2_MAX                   (0x00000001U)


#define CSL_TOP_RCM_PLL_CORE_STATUS_LOSSREF_MASK                (0x00000040U)
#define CSL_TOP_RCM_PLL_CORE_STATUS_LOSSREF_SHIFT               (0x00000006U)
#define CSL_TOP_RCM_PLL_CORE_STATUS_LOSSREF_RESETVAL            (0x00000001U)
#define CSL_TOP_RCM_PLL_CORE_STATUS_LOSSREF_MAX                 (0x00000001U)


#define CSL_TOP_RCM_PLL_CORE_STATUS_M2CHANGEACK_MASK            (0x00000008U)
#define CSL_TOP_RCM_PLL_CORE_STATUS_M2CHANGEACK_SHIFT           (0x00000003U)
#define CSL_TOP_RCM_PLL_CORE_STATUS_M2CHANGEACK_RESETVAL        (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_STATUS_M2CHANGEACK_MAX             (0x00000001U)


#define CSL_TOP_RCM_PLL_CORE_STATUS_PGOODOUT_MASK               (0x40000000U)
#define CSL_TOP_RCM_PLL_CORE_STATUS_PGOODOUT_SHIFT              (0x0000001EU)
#define CSL_TOP_RCM_PLL_CORE_STATUS_PGOODOUT_RESETVAL           (0x00000001U)
#define CSL_TOP_RCM_PLL_CORE_STATUS_PGOODOUT_MAX                (0x00000001U)


#define CSL_TOP_RCM_PLL_CORE_STATUS_PHASELOCK_MASK              (0x00000400U)
#define CSL_TOP_RCM_PLL_CORE_STATUS_PHASELOCK_SHIFT             (0x0000000AU)
#define CSL_TOP_RCM_PLL_CORE_STATUS_PHASELOCK_RESETVAL          (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_STATUS_PHASELOCK_MAX               (0x00000001U)


#define CSL_TOP_RCM_PLL_CORE_STATUS_PONOUT_MASK                 (0x80000000U)
#define CSL_TOP_RCM_PLL_CORE_STATUS_PONOUT_SHIFT                (0x0000001FU)
#define CSL_TOP_RCM_PLL_CORE_STATUS_PONOUT_RESETVAL             (0x00000001U)
#define CSL_TOP_RCM_PLL_CORE_STATUS_PONOUT_MAX                  (0x00000001U)


#define CSL_TOP_RCM_PLL_CORE_STATUS_RECAL_BSTATUS3_MASK         (0x10000000U)
#define CSL_TOP_RCM_PLL_CORE_STATUS_RECAL_BSTATUS3_SHIFT        (0x0000001CU)
#define CSL_TOP_RCM_PLL_CORE_STATUS_RECAL_BSTATUS3_RESETVAL     (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_STATUS_RECAL_BSTATUS3_MAX          (0x00000001U)


#define CSL_TOP_RCM_PLL_CORE_STATUS_RECAL_OPPIN_MASK            (0x08000000U)
#define CSL_TOP_RCM_PLL_CORE_STATUS_RECAL_OPPIN_SHIFT           (0x0000001BU)
#define CSL_TOP_RCM_PLL_CORE_STATUS_RECAL_OPPIN_RESETVAL        (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_STATUS_RECAL_OPPIN_MAX             (0x00000001U)


#define CSL_TOP_RCM_PLL_CORE_STATUS_SSCACK_MASK                 (0x00000004U)
#define CSL_TOP_RCM_PLL_CORE_STATUS_SSCACK_SHIFT                (0x00000002U)
#define CSL_TOP_RCM_PLL_CORE_STATUS_SSCACK_RESETVAL             (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_STATUS_SSCACK_MAX                  (0x00000001U)


#define CSL_TOP_RCM_PLL_CORE_STATUS_STBYRETACK_MASK             (0x00000080U)
#define CSL_TOP_RCM_PLL_CORE_STATUS_STBYRETACK_SHIFT            (0x00000007U)
#define CSL_TOP_RCM_PLL_CORE_STATUS_STBYRETACK_RESETVAL         (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_STATUS_STBYRETACK_MAX              (0x00000001U)



/* PLL_CORE_HSDIVIDER */
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_BYPASS_MASK           (0x00000001U)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_BYPASS_SHIFT          (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_BYPASS_RESETVAL       (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_BYPASS_MAX            (0x00000001U)


#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_BYPASSACKZ_MASK       (0x00010000U)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_BYPASSACKZ_SHIFT      (0x00000010U)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_BYPASSACKZ_RESETVAL   (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_BYPASSACKZ_MAX        (0x00000001U)


#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_LDOPWDN_MASK          (0x00000002U)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_LDOPWDN_SHIFT         (0x00000001U)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_LDOPWDN_RESETVAL      (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_LDOPWDN_MAX           (0x00000001U)


#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_LDOPWDNACK_MASK       (0x00020000U)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_LDOPWDNACK_SHIFT      (0x00000011U)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_LDOPWDNACK_RESETVAL   (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_LDOPWDNACK_MAX        (0x00000001U)


#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_TENABLEDIV_MASK       (0x00000004U)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_TENABLEDIV_SHIFT      (0x00000002U)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_TENABLEDIV_RESETVAL   (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_TENABLEDIV_MAX        (0x00000001U)



/* PLL_CORE_HSDIVIDER_CLKOUT0 */
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT0_DIV_MASK (0x0000001FU)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT0_DIV_SHIFT (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT0_DIV_RESETVAL (0x00000004U)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT0_DIV_MAX (0x0000001FU)


#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT0_DIVCHACK_MASK (0x00000020U)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT0_DIVCHACK_SHIFT (0x00000005U)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT0_DIVCHACK_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT0_DIVCHACK_MAX (0x00000001U)


#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT0_GATE_CTRL_MASK (0x00000100U)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT0_GATE_CTRL_SHIFT (0x00000008U)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT0_GATE_CTRL_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT0_GATE_CTRL_MAX (0x00000001U)


#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT0_PWDN_MASK (0x00001000U)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT0_PWDN_SHIFT (0x0000000CU)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT0_PWDN_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT0_PWDN_MAX (0x00000001U)


#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT0_STATUS_MASK (0x00000200U)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT0_STATUS_SHIFT (0x00000009U)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT0_STATUS_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT0_STATUS_MAX (0x00000001U)



/* PLL_CORE_HSDIVIDER_CLKOUT1 */
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT1_DIV_MASK (0x0000001FU)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT1_DIV_SHIFT (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT1_DIV_RESETVAL (0x00000003U)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT1_DIV_MAX (0x0000001FU)


#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT1_DIVCHACK_MASK (0x00000020U)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT1_DIVCHACK_SHIFT (0x00000005U)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT1_DIVCHACK_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT1_DIVCHACK_MAX (0x00000001U)


#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT1_GATE_CTRL_MASK (0x00000100U)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT1_GATE_CTRL_SHIFT (0x00000008U)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT1_GATE_CTRL_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT1_GATE_CTRL_MAX (0x00000001U)


#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT1_PWDN_MASK (0x00001000U)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT1_PWDN_SHIFT (0x0000000CU)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT1_PWDN_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT1_PWDN_MAX (0x00000001U)


#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT1_STATUS_MASK (0x00000200U)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT1_STATUS_SHIFT (0x00000009U)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT1_STATUS_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT1_STATUS_MAX (0x00000001U)



/* PLL_CORE_HSDIVIDER_CLKOUT2 */
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT2_DIV_MASK (0x0000001FU)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT2_DIV_SHIFT (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT2_DIV_RESETVAL (0x00000004U)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT2_DIV_MAX (0x0000001FU)


#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT2_DIVCHACK_MASK (0x00000020U)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT2_DIVCHACK_SHIFT (0x00000005U)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT2_DIVCHACK_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT2_DIVCHACK_MAX (0x00000001U)


#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT2_GATE_CTRL_MASK (0x00000100U)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT2_GATE_CTRL_SHIFT (0x00000008U)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT2_GATE_CTRL_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT2_GATE_CTRL_MAX (0x00000001U)


#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT2_PWDN_MASK (0x00001000U)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT2_PWDN_SHIFT (0x0000000CU)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT2_PWDN_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT2_PWDN_MAX (0x00000001U)


#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT2_STATUS_MASK (0x00000200U)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT2_STATUS_SHIFT (0x00000009U)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT2_STATUS_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT2_STATUS_MAX (0x00000001U)



/* PLL_CORE_HSDIVIDER_CLKOUT3 */
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT3_DIV_MASK (0x0000001FU)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT3_DIV_SHIFT (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT3_DIV_RESETVAL (0x00000014U)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT3_DIV_MAX (0x0000001FU)


#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT3_DIVCHACK_MASK (0x00000020U)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT3_DIVCHACK_SHIFT (0x00000005U)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT3_DIVCHACK_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT3_DIVCHACK_MAX (0x00000001U)


#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT3_GATE_CTRL_MASK (0x00000100U)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT3_GATE_CTRL_SHIFT (0x00000008U)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT3_GATE_CTRL_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT3_GATE_CTRL_MAX (0x00000001U)


#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT3_PWDN_MASK (0x00001000U)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT3_PWDN_SHIFT (0x0000000CU)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT3_PWDN_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT3_PWDN_MAX (0x00000001U)


#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT3_STATUS_MASK (0x00000200U)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT3_STATUS_SHIFT (0x00000009U)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT3_STATUS_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT3_STATUS_MAX (0x00000001U)



/* PLL_CORE_RSTCTRL */
#define CSL_TOP_RCM_PLL_CORE_RSTCTRL_ASSERT_MASK               (0x00000007U)
#define CSL_TOP_RCM_PLL_CORE_RSTCTRL_ASSERT_SHIFT              (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_RSTCTRL_ASSERT_RESETVAL           (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_RSTCTRL_ASSERT_MAX                (0x00000007U)



/* PLL_CORE_HSDIVIDER_RSTCTRL */
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_RSTCTRL_ASSERT_MASK (0x00000007U)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_RSTCTRL_ASSERT_SHIFT (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_RSTCTRL_ASSERT_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_PLL_CORE_HSDIVIDER_RSTCTRL_ASSERT_MAX (0x00000007U)



/* R5SS_CLK_SRC_SEL */
#define CSL_TOP_RCM_R5SS_CLK_SRC_SEL_CLKSRCSEL_MASK      (0x00000FFFU)
#define CSL_TOP_RCM_R5SS_CLK_SRC_SEL_CLKSRCSEL_SHIFT     (0x00000000U)
#define CSL_TOP_RCM_R5SS_CLK_SRC_SEL_CLKSRCSEL_RESETVAL  (0x00000000U)
#define CSL_TOP_RCM_R5SS_CLK_SRC_SEL_CLKSRCSEL_MAX       (0x00000FFFU)



/* R5SS_CLK_STATUS */
#define CSL_TOP_RCM_R5SS_CLK_STATUS_CLKINUSE_MASK (0x000000FFU)
#define CSL_TOP_RCM_R5SS_CLK_STATUS_CLKINUSE_SHIFT (0x00000000U)
#define CSL_TOP_RCM_R5SS_CLK_STATUS_CLKINUSE_RESETVAL (0x00000001U)
#define CSL_TOP_RCM_R5SS_CLK_STATUS_CLKINUSE_MAX (0x000000FFU)



/* R5SS0_CLK_DIV_SEL */
#define CSL_TOP_RCM_R5SS0_CLK_DIV_SEL_CLKSRCSEL_MASK    (0x00000007U)
#define CSL_TOP_RCM_R5SS0_CLK_DIV_SEL_CLKSRCSEL_SHIFT   (0x00000000U)
#define CSL_TOP_RCM_R5SS0_CLK_DIV_SEL_CLKSRCSEL_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_R5SS0_CLK_DIV_SEL_CLKSRCSEL_MAX     (0x00000007U)



/* R5SS0_CLK_GATE */
#define CSL_TOP_RCM_R5SS0_CLK_GATE_GATED_MASK              (0x00000007U)
#define CSL_TOP_RCM_R5SS0_CLK_GATE_GATED_SHIFT             (0x00000000U)
#define CSL_TOP_RCM_R5SS0_CLK_GATE_GATED_RESETVAL          (0x00000000U)
#define CSL_TOP_RCM_R5SS0_CLK_GATE_GATED_MAX               (0x00000007U)



/* SYS_CLK_DIV_VAL */
#define CSL_TOP_RCM_SYS_CLK_DIV_VAL_CLKDIV_MASK                 (0x00000FFFU)
#define CSL_TOP_RCM_SYS_CLK_DIV_VAL_CLKDIV_SHIFT                (0x00000000U)
#define CSL_TOP_RCM_SYS_CLK_DIV_VAL_CLKDIV_RESETVAL             (0x00000000U)
#define CSL_TOP_RCM_SYS_CLK_DIV_VAL_CLKDIV_MAX                  (0x00000FFFU)



/* SYS_CLK_GATE */
#define CSL_TOP_RCM_SYS_CLK_GATE_GATED_MASK                        (0x00000007U)
#define CSL_TOP_RCM_SYS_CLK_GATE_GATED_SHIFT                       (0x00000000U)
#define CSL_TOP_RCM_SYS_CLK_GATE_GATED_RESETVAL                    (0x00000000U)
#define CSL_TOP_RCM_SYS_CLK_GATE_GATED_MAX                         (0x00000007U)



/* SYS_CLK_STATUS */
#define CSL_TOP_RCM_SYS_CLK_STATUS_CURRDIVIDER_MASK              (0x0000FF00U)
#define CSL_TOP_RCM_SYS_CLK_STATUS_CURRDIVIDER_SHIFT             (0x00000008U)
#define CSL_TOP_RCM_SYS_CLK_STATUS_CURRDIVIDER_RESETVAL          (0x00000000U)
#define CSL_TOP_RCM_SYS_CLK_STATUS_CURRDIVIDER_MAX               (0x000000FFU)



/* PLL_PER_PWRCTRL */
#define CSL_TOP_RCM_PLL_PER_PWRCTRL_ISORET_MASK                 (0x00000004U)
#define CSL_TOP_RCM_PLL_PER_PWRCTRL_ISORET_SHIFT                (0x00000002U)
#define CSL_TOP_RCM_PLL_PER_PWRCTRL_ISORET_RESETVAL             (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_PWRCTRL_ISORET_MAX                  (0x00000001U)


#define CSL_TOP_RCM_PLL_PER_PWRCTRL_ISOSCAN_MASK                (0x00000002U)
#define CSL_TOP_RCM_PLL_PER_PWRCTRL_ISOSCAN_SHIFT               (0x00000001U)
#define CSL_TOP_RCM_PLL_PER_PWRCTRL_ISOSCAN_RESETVAL            (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_PWRCTRL_ISOSCAN_MAX                 (0x00000001U)


#define CSL_TOP_RCM_PLL_PER_PWRCTRL_OFFMODE_MASK                (0x00000001U)
#define CSL_TOP_RCM_PLL_PER_PWRCTRL_OFFMODE_SHIFT               (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_PWRCTRL_OFFMODE_RESETVAL            (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_PWRCTRL_OFFMODE_MAX                 (0x00000001U)


#define CSL_TOP_RCM_PLL_PER_PWRCTRL_PGOODIN_MASK                (0x00000010U)
#define CSL_TOP_RCM_PLL_PER_PWRCTRL_PGOODIN_SHIFT               (0x00000004U)
#define CSL_TOP_RCM_PLL_PER_PWRCTRL_PGOODIN_RESETVAL            (0x00000001U)
#define CSL_TOP_RCM_PLL_PER_PWRCTRL_PGOODIN_MAX                 (0x00000001U)


#define CSL_TOP_RCM_PLL_PER_PWRCTRL_PONIN_MASK                  (0x00000020U)
#define CSL_TOP_RCM_PLL_PER_PWRCTRL_PONIN_SHIFT                 (0x00000005U)
#define CSL_TOP_RCM_PLL_PER_PWRCTRL_PONIN_RESETVAL              (0x00000001U)
#define CSL_TOP_RCM_PLL_PER_PWRCTRL_PONIN_MAX                   (0x00000001U)


#define CSL_TOP_RCM_PLL_PER_PWRCTRL_RET_MASK                    (0x00000008U)
#define CSL_TOP_RCM_PLL_PER_PWRCTRL_RET_SHIFT                   (0x00000003U)
#define CSL_TOP_RCM_PLL_PER_PWRCTRL_RET_RESETVAL                (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_PWRCTRL_RET_MAX                     (0x00000001U)



/* PLL_PER_CLKCTRL */
#define CSL_TOP_RCM_PLL_PER_CLKCTRL_BYPASSACKZ_MASK             (0x00400000U)
#define CSL_TOP_RCM_PLL_PER_CLKCTRL_BYPASSACKZ_SHIFT            (0x00000016U)
#define CSL_TOP_RCM_PLL_PER_CLKCTRL_BYPASSACKZ_RESETVAL         (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_CLKCTRL_BYPASSACKZ_MAX              (0x00000001U)


#define CSL_TOP_RCM_PLL_PER_CLKCTRL_CLKDCOLDOEN_MASK            (0x20000000U)
#define CSL_TOP_RCM_PLL_PER_CLKCTRL_CLKDCOLDOEN_SHIFT           (0x0000001DU)
#define CSL_TOP_RCM_PLL_PER_CLKCTRL_CLKDCOLDOEN_RESETVAL        (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_CLKCTRL_CLKDCOLDOEN_MAX             (0x00000001U)


#define CSL_TOP_RCM_PLL_PER_CLKCTRL_CLKDCOLDOPWDNZ_MASK         (0x00020000U)
#define CSL_TOP_RCM_PLL_PER_CLKCTRL_CLKDCOLDOPWDNZ_SHIFT        (0x00000011U)
#define CSL_TOP_RCM_PLL_PER_CLKCTRL_CLKDCOLDOPWDNZ_RESETVAL     (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_CLKCTRL_CLKDCOLDOPWDNZ_MAX          (0x00000001U)


#define CSL_TOP_RCM_PLL_PER_CLKCTRL_CLKOUTEN_MASK               (0x00100000U)
#define CSL_TOP_RCM_PLL_PER_CLKCTRL_CLKOUTEN_SHIFT              (0x00000014U)
#define CSL_TOP_RCM_PLL_PER_CLKCTRL_CLKOUTEN_RESETVAL           (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_CLKCTRL_CLKOUTEN_MAX                (0x00000001U)


#define CSL_TOP_RCM_PLL_PER_CLKCTRL_CLKOUTLDOEN_MASK            (0x00080000U)
#define CSL_TOP_RCM_PLL_PER_CLKCTRL_CLKOUTLDOEN_SHIFT           (0x00000013U)
#define CSL_TOP_RCM_PLL_PER_CLKCTRL_CLKOUTLDOEN_RESETVAL        (0x00000001U)
#define CSL_TOP_RCM_PLL_PER_CLKCTRL_CLKOUTLDOEN_MAX             (0x00000001U)


#define CSL_TOP_RCM_PLL_PER_CLKCTRL_CYCLESLIPEN_MASK            (0x80000000U)
#define CSL_TOP_RCM_PLL_PER_CLKCTRL_CYCLESLIPEN_SHIFT           (0x0000001FU)
#define CSL_TOP_RCM_PLL_PER_CLKCTRL_CYCLESLIPEN_RESETVAL        (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_CLKCTRL_CYCLESLIPEN_MAX             (0x00000001U)


#define CSL_TOP_RCM_PLL_PER_CLKCTRL_ENSSC_MASK                  (0x40000000U)
#define CSL_TOP_RCM_PLL_PER_CLKCTRL_ENSSC_SHIFT                 (0x0000001EU)
#define CSL_TOP_RCM_PLL_PER_CLKCTRL_ENSSC_RESETVAL              (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_CLKCTRL_ENSSC_MAX                   (0x00000001U)


#define CSL_TOP_RCM_PLL_PER_CLKCTRL_IDLE_MASK                   (0x00800000U)
#define CSL_TOP_RCM_PLL_PER_CLKCTRL_IDLE_SHIFT                  (0x00000017U)
#define CSL_TOP_RCM_PLL_PER_CLKCTRL_IDLE_RESETVAL               (0x00000001U)
#define CSL_TOP_RCM_PLL_PER_CLKCTRL_IDLE_MAX                    (0x00000001U)


#define CSL_TOP_RCM_PLL_PER_CLKCTRL_M2PWDNZ_MASK                (0x00010000U)
#define CSL_TOP_RCM_PLL_PER_CLKCTRL_M2PWDNZ_SHIFT               (0x00000010U)
#define CSL_TOP_RCM_PLL_PER_CLKCTRL_M2PWDNZ_RESETVAL            (0x00000001U)
#define CSL_TOP_RCM_PLL_PER_CLKCTRL_M2PWDNZ_MAX                 (0x00000001U)


#define CSL_TOP_RCM_PLL_PER_CLKCTRL_RELAXED_LOCK_MASK           (0x00000100U)
#define CSL_TOP_RCM_PLL_PER_CLKCTRL_RELAXED_LOCK_SHIFT          (0x00000008U)
#define CSL_TOP_RCM_PLL_PER_CLKCTRL_RELAXED_LOCK_RESETVAL       (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_CLKCTRL_RELAXED_LOCK_MAX            (0x00000001U)


#define CSL_TOP_RCM_PLL_PER_CLKCTRL_SELFREQDCO_MASK             (0x00001C00U)
#define CSL_TOP_RCM_PLL_PER_CLKCTRL_SELFREQDCO_SHIFT            (0x0000000AU)
#define CSL_TOP_RCM_PLL_PER_CLKCTRL_SELFREQDCO_RESETVAL         (0x00000004U)
#define CSL_TOP_RCM_PLL_PER_CLKCTRL_SELFREQDCO_MAX              (0x00000007U)


#define CSL_TOP_RCM_PLL_PER_CLKCTRL_SSCTYPE_MASK                (0x00000002U)
#define CSL_TOP_RCM_PLL_PER_CLKCTRL_SSCTYPE_SHIFT               (0x00000001U)
#define CSL_TOP_RCM_PLL_PER_CLKCTRL_SSCTYPE_RESETVAL            (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_CLKCTRL_SSCTYPE_MAX                 (0x00000001U)


#define CSL_TOP_RCM_PLL_PER_CLKCTRL_STBYRET_MASK                (0x00200000U)
#define CSL_TOP_RCM_PLL_PER_CLKCTRL_STBYRET_SHIFT               (0x00000015U)
#define CSL_TOP_RCM_PLL_PER_CLKCTRL_STBYRET_RESETVAL            (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_CLKCTRL_STBYRET_MAX                 (0x00000001U)


#define CSL_TOP_RCM_PLL_PER_CLKCTRL_STOPMODE_MASK               (0x00004000U)
#define CSL_TOP_RCM_PLL_PER_CLKCTRL_STOPMODE_SHIFT              (0x0000000EU)
#define CSL_TOP_RCM_PLL_PER_CLKCTRL_STOPMODE_RESETVAL           (0x00000001U)
#define CSL_TOP_RCM_PLL_PER_CLKCTRL_STOPMODE_MAX                (0x00000001U)


#define CSL_TOP_RCM_PLL_PER_CLKCTRL_TINTZ_MASK                  (0x00000001U)
#define CSL_TOP_RCM_PLL_PER_CLKCTRL_TINTZ_SHIFT                 (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_CLKCTRL_TINTZ_RESETVAL              (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_CLKCTRL_TINTZ_MAX                   (0x00000001U)


#define CSL_TOP_RCM_PLL_PER_CLKCTRL_ULOWCLKEN_MASK              (0x00040000U)
#define CSL_TOP_RCM_PLL_PER_CLKCTRL_ULOWCLKEN_SHIFT             (0x00000012U)
#define CSL_TOP_RCM_PLL_PER_CLKCTRL_ULOWCLKEN_RESETVAL          (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_CLKCTRL_ULOWCLKEN_MAX               (0x00000001U)



/* PLL_PER_TENABLE */
#define CSL_TOP_RCM_PLL_PER_TENABLE_TENABLE_MASK                (0x00000001U)
#define CSL_TOP_RCM_PLL_PER_TENABLE_TENABLE_SHIFT               (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_TENABLE_TENABLE_RESETVAL            (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_TENABLE_TENABLE_MAX                 (0x00000001U)



/* PLL_PER_TENABLEDIV */
#define CSL_TOP_RCM_PLL_PER_TENABLEDIV_TENABLEDIV_MASK       (0x00000001U)
#define CSL_TOP_RCM_PLL_PER_TENABLEDIV_TENABLEDIV_SHIFT      (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_TENABLEDIV_TENABLEDIV_RESETVAL   (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_TENABLEDIV_TENABLEDIV_MAX        (0x00000001U)



/* PLL_PER_M2NDIV */
#define CSL_TOP_RCM_PLL_PER_M2NDIV_M2_MASK                       (0x007F0000U)
#define CSL_TOP_RCM_PLL_PER_M2NDIV_M2_SHIFT                      (0x00000010U)
#define CSL_TOP_RCM_PLL_PER_M2NDIV_M2_RESETVAL                   (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_M2NDIV_M2_MAX                        (0x0000007FU)


#define CSL_TOP_RCM_PLL_PER_M2NDIV_N_MASK                        (0x000000FFU)
#define CSL_TOP_RCM_PLL_PER_M2NDIV_N_SHIFT                       (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_M2NDIV_N_RESETVAL                    (0x00000013U)
#define CSL_TOP_RCM_PLL_PER_M2NDIV_N_MAX                         (0x000000FFU)



/* PLL_PER_MN2DIV */
#define CSL_TOP_RCM_PLL_PER_MN2DIV_M_MASK                        (0x00000FFFU)
#define CSL_TOP_RCM_PLL_PER_MN2DIV_M_SHIFT                       (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_MN2DIV_M_RESETVAL                    (0x00000300U)
#define CSL_TOP_RCM_PLL_PER_MN2DIV_M_MAX                         (0x00000FFFU)


#define CSL_TOP_RCM_PLL_PER_MN2DIV_N2_MASK                       (0x000F0000U)
#define CSL_TOP_RCM_PLL_PER_MN2DIV_N2_SHIFT                      (0x00000010U)
#define CSL_TOP_RCM_PLL_PER_MN2DIV_N2_RESETVAL                   (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_MN2DIV_N2_MAX                        (0x0000000FU)



/* PLL_PER_FRACDIV */
#define CSL_TOP_RCM_PLL_PER_FRACDIV_FRACTIONALM_MASK            (0x0003FFFFU)
#define CSL_TOP_RCM_PLL_PER_FRACDIV_FRACTIONALM_SHIFT           (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_FRACDIV_FRACTIONALM_RESETVAL        (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_FRACDIV_FRACTIONALM_MAX             (0x0003FFFFU)


#define CSL_TOP_RCM_PLL_PER_FRACDIV_REGSD_MASK                  (0xFF000000U)
#define CSL_TOP_RCM_PLL_PER_FRACDIV_REGSD_SHIFT                 (0x00000018U)
#define CSL_TOP_RCM_PLL_PER_FRACDIV_REGSD_RESETVAL              (0x00000008U)
#define CSL_TOP_RCM_PLL_PER_FRACDIV_REGSD_MAX                   (0x000000FFU)



/* PLL_PER_BWCTRL */
#define CSL_TOP_RCM_PLL_PER_BWCTRL_BWCONTROL_MASK                (0x00000006U)
#define CSL_TOP_RCM_PLL_PER_BWCTRL_BWCONTROL_SHIFT               (0x00000001U)
#define CSL_TOP_RCM_PLL_PER_BWCTRL_BWCONTROL_RESETVAL            (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_BWCTRL_BWCONTROL_MAX                 (0x00000003U)


#define CSL_TOP_RCM_PLL_PER_BWCTRL_BW_INCR_DECRZ_MASK            (0x00000001U)
#define CSL_TOP_RCM_PLL_PER_BWCTRL_BW_INCR_DECRZ_SHIFT           (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_BWCTRL_BW_INCR_DECRZ_RESETVAL        (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_BWCTRL_BW_INCR_DECRZ_MAX             (0x00000001U)



/* PLL_PER_FRACCTRL */
#define CSL_TOP_RCM_PLL_PER_FRACCTRL_DOWNSPREAD_MASK           (0x80000000U)
#define CSL_TOP_RCM_PLL_PER_FRACCTRL_DOWNSPREAD_SHIFT          (0x0000001FU)
#define CSL_TOP_RCM_PLL_PER_FRACCTRL_DOWNSPREAD_RESETVAL       (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_FRACCTRL_DOWNSPREAD_MAX            (0x00000001U)


#define CSL_TOP_RCM_PLL_PER_FRACCTRL_DELTAMSTEPFRACTION_MASK   (0x0003FFFFU)
#define CSL_TOP_RCM_PLL_PER_FRACCTRL_DELTAMSTEPFRACTION_SHIFT  (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_FRACCTRL_DELTAMSTEPFRACTION_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_FRACCTRL_DELTAMSTEPFRACTION_MAX    (0x0003FFFFU)


#define CSL_TOP_RCM_PLL_PER_FRACCTRL_DELTAMSTEPINTEGER_MASK    (0x001C0000U)
#define CSL_TOP_RCM_PLL_PER_FRACCTRL_DELTAMSTEPINTEGER_SHIFT   (0x00000012U)
#define CSL_TOP_RCM_PLL_PER_FRACCTRL_DELTAMSTEPINTEGER_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_FRACCTRL_DELTAMSTEPINTEGER_MAX     (0x00000007U)


#define CSL_TOP_RCM_PLL_PER_FRACCTRL_MODFREQDIVIDEREXPONENT_MASK (0x70000000U)
#define CSL_TOP_RCM_PLL_PER_FRACCTRL_MODFREQDIVIDEREXPONENT_SHIFT (0x0000001CU)
#define CSL_TOP_RCM_PLL_PER_FRACCTRL_MODFREQDIVIDEREXPONENT_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_FRACCTRL_MODFREQDIVIDEREXPONENT_MAX (0x00000007U)


#define CSL_TOP_RCM_PLL_PER_FRACCTRL_MODFREQDIVIDERMANTISSA_MASK (0x0FE00000U)
#define CSL_TOP_RCM_PLL_PER_FRACCTRL_MODFREQDIVIDERMANTISSA_SHIFT (0x00000015U)
#define CSL_TOP_RCM_PLL_PER_FRACCTRL_MODFREQDIVIDERMANTISSA_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_FRACCTRL_MODFREQDIVIDERMANTISSA_MAX (0x0000007FU)



/* PLL_PER_STATUS */
#define CSL_TOP_RCM_PLL_PER_STATUS_BYPASS_MASK                   (0x00000001U)
#define CSL_TOP_RCM_PLL_PER_STATUS_BYPASS_SHIFT                  (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_STATUS_BYPASS_RESETVAL               (0x00000001U)
#define CSL_TOP_RCM_PLL_PER_STATUS_BYPASS_MAX                    (0x00000001U)


#define CSL_TOP_RCM_PLL_PER_STATUS_BYPASSACK_MASK                (0x00000100U)
#define CSL_TOP_RCM_PLL_PER_STATUS_BYPASSACK_SHIFT               (0x00000008U)
#define CSL_TOP_RCM_PLL_PER_STATUS_BYPASSACK_RESETVAL            (0x00000001U)
#define CSL_TOP_RCM_PLL_PER_STATUS_BYPASSACK_MAX                 (0x00000001U)


#define CSL_TOP_RCM_PLL_PER_STATUS_CLKDCOLDOACK_MASK             (0x00000800U)
#define CSL_TOP_RCM_PLL_PER_STATUS_CLKDCOLDOACK_SHIFT            (0x0000000BU)
#define CSL_TOP_RCM_PLL_PER_STATUS_CLKDCOLDOACK_RESETVAL         (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_STATUS_CLKDCOLDOACK_MAX              (0x00000001U)


#define CSL_TOP_RCM_PLL_PER_STATUS_CLKOUTENACK_MASK              (0x00000020U)
#define CSL_TOP_RCM_PLL_PER_STATUS_CLKOUTENACK_SHIFT             (0x00000005U)
#define CSL_TOP_RCM_PLL_PER_STATUS_CLKOUTENACK_RESETVAL          (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_STATUS_CLKOUTENACK_MAX               (0x00000001U)


#define CSL_TOP_RCM_PLL_PER_STATUS_CLKOUTLDOENACK_MASK           (0x00001000U)
#define CSL_TOP_RCM_PLL_PER_STATUS_CLKOUTLDOENACK_SHIFT          (0x0000000CU)
#define CSL_TOP_RCM_PLL_PER_STATUS_CLKOUTLDOENACK_RESETVAL       (0x00000001U)
#define CSL_TOP_RCM_PLL_PER_STATUS_CLKOUTLDOENACK_MAX            (0x00000001U)


#define CSL_TOP_RCM_PLL_PER_STATUS_FREQLOCK_MASK                 (0x00000200U)
#define CSL_TOP_RCM_PLL_PER_STATUS_FREQLOCK_SHIFT                (0x00000009U)
#define CSL_TOP_RCM_PLL_PER_STATUS_FREQLOCK_RESETVAL             (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_STATUS_FREQLOCK_MAX                  (0x00000001U)


#define CSL_TOP_RCM_PLL_PER_STATUS_HIGHJITTER_MASK               (0x00000002U)
#define CSL_TOP_RCM_PLL_PER_STATUS_HIGHJITTER_SHIFT              (0x00000001U)
#define CSL_TOP_RCM_PLL_PER_STATUS_HIGHJITTER_RESETVAL           (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_STATUS_HIGHJITTER_MAX                (0x00000001U)


#define CSL_TOP_RCM_PLL_PER_STATUS_LDOPWDN_MASK                  (0x20000000U)
#define CSL_TOP_RCM_PLL_PER_STATUS_LDOPWDN_SHIFT                 (0x0000001DU)
#define CSL_TOP_RCM_PLL_PER_STATUS_LDOPWDN_RESETVAL              (0x00000001U)
#define CSL_TOP_RCM_PLL_PER_STATUS_LDOPWDN_MAX                   (0x00000001U)


#define CSL_TOP_RCM_PLL_PER_STATUS_LOCK2_MASK                    (0x00000010U)
#define CSL_TOP_RCM_PLL_PER_STATUS_LOCK2_SHIFT                   (0x00000004U)
#define CSL_TOP_RCM_PLL_PER_STATUS_LOCK2_RESETVAL                (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_STATUS_LOCK2_MAX                     (0x00000001U)


#define CSL_TOP_RCM_PLL_PER_STATUS_LOSSREF_MASK                  (0x00000040U)
#define CSL_TOP_RCM_PLL_PER_STATUS_LOSSREF_SHIFT                 (0x00000006U)
#define CSL_TOP_RCM_PLL_PER_STATUS_LOSSREF_RESETVAL              (0x00000001U)
#define CSL_TOP_RCM_PLL_PER_STATUS_LOSSREF_MAX                   (0x00000001U)


#define CSL_TOP_RCM_PLL_PER_STATUS_M2CHANGEACK_MASK              (0x00000008U)
#define CSL_TOP_RCM_PLL_PER_STATUS_M2CHANGEACK_SHIFT             (0x00000003U)
#define CSL_TOP_RCM_PLL_PER_STATUS_M2CHANGEACK_RESETVAL          (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_STATUS_M2CHANGEACK_MAX               (0x00000001U)


#define CSL_TOP_RCM_PLL_PER_STATUS_PGOODOUT_MASK                 (0x40000000U)
#define CSL_TOP_RCM_PLL_PER_STATUS_PGOODOUT_SHIFT                (0x0000001EU)
#define CSL_TOP_RCM_PLL_PER_STATUS_PGOODOUT_RESETVAL             (0x00000001U)
#define CSL_TOP_RCM_PLL_PER_STATUS_PGOODOUT_MAX                  (0x00000001U)


#define CSL_TOP_RCM_PLL_PER_STATUS_PHASELOCK_MASK                (0x00000400U)
#define CSL_TOP_RCM_PLL_PER_STATUS_PHASELOCK_SHIFT               (0x0000000AU)
#define CSL_TOP_RCM_PLL_PER_STATUS_PHASELOCK_RESETVAL            (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_STATUS_PHASELOCK_MAX                 (0x00000001U)


#define CSL_TOP_RCM_PLL_PER_STATUS_PONOUT_MASK                   (0x80000000U)
#define CSL_TOP_RCM_PLL_PER_STATUS_PONOUT_SHIFT                  (0x0000001FU)
#define CSL_TOP_RCM_PLL_PER_STATUS_PONOUT_RESETVAL               (0x00000001U)
#define CSL_TOP_RCM_PLL_PER_STATUS_PONOUT_MAX                    (0x00000001U)


#define CSL_TOP_RCM_PLL_PER_STATUS_RECAL_BSTATUS3_MASK           (0x10000000U)
#define CSL_TOP_RCM_PLL_PER_STATUS_RECAL_BSTATUS3_SHIFT          (0x0000001CU)
#define CSL_TOP_RCM_PLL_PER_STATUS_RECAL_BSTATUS3_RESETVAL       (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_STATUS_RECAL_BSTATUS3_MAX            (0x00000001U)


#define CSL_TOP_RCM_PLL_PER_STATUS_RECAL_OPPIN_MASK              (0x08000000U)
#define CSL_TOP_RCM_PLL_PER_STATUS_RECAL_OPPIN_SHIFT             (0x0000001BU)
#define CSL_TOP_RCM_PLL_PER_STATUS_RECAL_OPPIN_RESETVAL          (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_STATUS_RECAL_OPPIN_MAX               (0x00000001U)


#define CSL_TOP_RCM_PLL_PER_STATUS_SSCACK_MASK                   (0x00000004U)
#define CSL_TOP_RCM_PLL_PER_STATUS_SSCACK_SHIFT                  (0x00000002U)
#define CSL_TOP_RCM_PLL_PER_STATUS_SSCACK_RESETVAL               (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_STATUS_SSCACK_MAX                    (0x00000001U)


#define CSL_TOP_RCM_PLL_PER_STATUS_STBYRETACK_MASK               (0x00000080U)
#define CSL_TOP_RCM_PLL_PER_STATUS_STBYRETACK_SHIFT              (0x00000007U)
#define CSL_TOP_RCM_PLL_PER_STATUS_STBYRETACK_RESETVAL           (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_STATUS_STBYRETACK_MAX                (0x00000001U)



/* PLL_PER_HSDIVIDER */
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_BYPASS_MASK             (0x00000001U)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_BYPASS_SHIFT            (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_BYPASS_RESETVAL         (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_BYPASS_MAX              (0x00000001U)


#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_BYPASSACKZ_MASK         (0x00010000U)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_BYPASSACKZ_SHIFT        (0x00000010U)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_BYPASSACKZ_RESETVAL     (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_BYPASSACKZ_MAX          (0x00000001U)


#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_LDOPWDN_MASK            (0x00000002U)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_LDOPWDN_SHIFT           (0x00000001U)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_LDOPWDN_RESETVAL        (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_LDOPWDN_MAX             (0x00000001U)


#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_LDOPWDNACK_MASK         (0x00020000U)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_LDOPWDNACK_SHIFT        (0x00000011U)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_LDOPWDNACK_RESETVAL     (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_LDOPWDNACK_MAX          (0x00000001U)


#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_TENABLEDIV_MASK         (0x00000004U)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_TENABLEDIV_SHIFT        (0x00000002U)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_TENABLEDIV_RESETVAL     (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_TENABLEDIV_MAX          (0x00000001U)



/* PLL_PER_HSDIVIDER_CLKOUT0 */
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT0_DIV_MASK (0x0000001FU)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT0_DIV_SHIFT (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT0_DIV_RESETVAL (0x00000004U)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT0_DIV_MAX (0x0000001FU)


#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT0_DIVCHACK_MASK (0x00000020U)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT0_DIVCHACK_SHIFT (0x00000005U)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT0_DIVCHACK_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT0_DIVCHACK_MAX (0x00000001U)


#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT0_GATE_CTRL_MASK (0x00000100U)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT0_GATE_CTRL_SHIFT (0x00000008U)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT0_GATE_CTRL_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT0_GATE_CTRL_MAX (0x00000001U)


#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT0_PWDN_MASK (0x00001000U)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT0_PWDN_SHIFT (0x0000000CU)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT0_PWDN_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT0_PWDN_MAX (0x00000001U)


#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT0_STATUS_MASK (0x00000200U)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT0_STATUS_SHIFT (0x00000009U)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT0_STATUS_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT0_STATUS_MAX (0x00000001U)



/* PLL_PER_HSDIVIDER_CLKOUT1 */
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT1_DIV_MASK (0x0000001FU)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT1_DIV_SHIFT (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT1_DIV_RESETVAL (0x00000001U)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT1_DIV_MAX (0x0000001FU)


#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT1_DIVCHACK_MASK (0x00000020U)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT1_DIVCHACK_SHIFT (0x00000005U)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT1_DIVCHACK_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT1_DIVCHACK_MAX (0x00000001U)


#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT1_GATE_CTRL_MASK (0x00000100U)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT1_GATE_CTRL_SHIFT (0x00000008U)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT1_GATE_CTRL_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT1_GATE_CTRL_MAX (0x00000001U)


#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT1_PWDN_MASK (0x00001000U)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT1_PWDN_SHIFT (0x0000000CU)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT1_PWDN_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT1_PWDN_MAX (0x00000001U)


#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT1_STATUS_MASK (0x00000200U)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT1_STATUS_SHIFT (0x00000009U)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT1_STATUS_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT1_STATUS_MAX (0x00000001U)



/* PLL_PER_HSDIVIDER_CLKOUT2 */
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT2_DIV_MASK (0x0000001FU)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT2_DIV_SHIFT (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT2_DIV_RESETVAL (0x00000005U)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT2_DIV_MAX (0x0000001FU)


#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT2_DIVCHACK_MASK (0x00000020U)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT2_DIVCHACK_SHIFT (0x00000005U)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT2_DIVCHACK_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT2_DIVCHACK_MAX (0x00000001U)


#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT2_GATE_CTRL_MASK (0x00000100U)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT2_GATE_CTRL_SHIFT (0x00000008U)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT2_GATE_CTRL_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT2_GATE_CTRL_MAX (0x00000001U)


#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT2_PWDN_MASK (0x00001000U)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT2_PWDN_SHIFT (0x0000000CU)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT2_PWDN_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT2_PWDN_MAX (0x00000001U)


#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT2_STATUS_MASK (0x00000200U)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT2_STATUS_SHIFT (0x00000009U)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT2_STATUS_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT2_STATUS_MAX (0x00000001U)



/* PLL_PER_HSDIVIDER_CLKOUT3 */
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT3_DIV_MASK (0x0000001FU)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT3_DIV_SHIFT (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT3_DIV_RESETVAL (0x00000007U)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT3_DIV_MAX (0x0000001FU)


#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT3_DIVCHACK_MASK (0x00000020U)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT3_DIVCHACK_SHIFT (0x00000005U)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT3_DIVCHACK_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT3_DIVCHACK_MAX (0x00000001U)


#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT3_GATE_CTRL_MASK (0x00000100U)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT3_GATE_CTRL_SHIFT (0x00000008U)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT3_GATE_CTRL_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT3_GATE_CTRL_MAX (0x00000001U)


#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT3_PWDN_MASK (0x00001000U)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT3_PWDN_SHIFT (0x0000000CU)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT3_PWDN_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT3_PWDN_MAX (0x00000001U)


#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT3_STATUS_MASK (0x00000200U)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT3_STATUS_SHIFT (0x00000009U)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT3_STATUS_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT3_STATUS_MAX (0x00000001U)



/* PLL_PER_RSTCTRL */
#define CSL_TOP_RCM_PLL_PER_RSTCTRL_ASSERT_MASK                 (0x00000007U)
#define CSL_TOP_RCM_PLL_PER_RSTCTRL_ASSERT_SHIFT                (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_RSTCTRL_ASSERT_RESETVAL             (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_RSTCTRL_ASSERT_MAX                  (0x00000007U)



/* PLL_PER_HSDIVIDER_RSTCTRL */
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_RSTCTRL_ASSERT_MASK (0x00000007U)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_RSTCTRL_ASSERT_SHIFT (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_RSTCTRL_ASSERT_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_PLL_PER_HSDIVIDER_RSTCTRL_ASSERT_MAX (0x00000007U)



/* PLL_ETH_PWRCTRL */
#define CSL_TOP_RCM_PLL_ETH_PWRCTRL_ISORET_MASK                 (0x00000004U)
#define CSL_TOP_RCM_PLL_ETH_PWRCTRL_ISORET_SHIFT                (0x00000002U)
#define CSL_TOP_RCM_PLL_ETH_PWRCTRL_ISORET_RESETVAL             (0x00000000U)
#define CSL_TOP_RCM_PLL_ETH_PWRCTRL_ISORET_MAX                  (0x00000001U)


#define CSL_TOP_RCM_PLL_ETH_PWRCTRL_ISOSCAN_MASK                (0x00000002U)
#define CSL_TOP_RCM_PLL_ETH_PWRCTRL_ISOSCAN_SHIFT               (0x00000001U)
#define CSL_TOP_RCM_PLL_ETH_PWRCTRL_ISOSCAN_RESETVAL            (0x00000000U)
#define CSL_TOP_RCM_PLL_ETH_PWRCTRL_ISOSCAN_MAX                 (0x00000001U)


#define CSL_TOP_RCM_PLL_ETH_PWRCTRL_OFFMODE_MASK                (0x00000001U)
#define CSL_TOP_RCM_PLL_ETH_PWRCTRL_OFFMODE_SHIFT               (0x00000000U)
#define CSL_TOP_RCM_PLL_ETH_PWRCTRL_OFFMODE_RESETVAL            (0x00000000U)
#define CSL_TOP_RCM_PLL_ETH_PWRCTRL_OFFMODE_MAX                 (0x00000001U)


#define CSL_TOP_RCM_PLL_ETH_PWRCTRL_PGOODIN_MASK                (0x00000010U)
#define CSL_TOP_RCM_PLL_ETH_PWRCTRL_PGOODIN_SHIFT               (0x00000004U)
#define CSL_TOP_RCM_PLL_ETH_PWRCTRL_PGOODIN_RESETVAL            (0x00000001U)
#define CSL_TOP_RCM_PLL_ETH_PWRCTRL_PGOODIN_MAX                 (0x00000001U)


#define CSL_TOP_RCM_PLL_ETH_PWRCTRL_PONIN_MASK                  (0x00000020U)
#define CSL_TOP_RCM_PLL_ETH_PWRCTRL_PONIN_SHIFT                 (0x00000005U)
#define CSL_TOP_RCM_PLL_ETH_PWRCTRL_PONIN_RESETVAL              (0x00000001U)
#define CSL_TOP_RCM_PLL_ETH_PWRCTRL_PONIN_MAX                   (0x00000001U)


#define CSL_TOP_RCM_PLL_ETH_PWRCTRL_RET_MASK                    (0x00000008U)
#define CSL_TOP_RCM_PLL_ETH_PWRCTRL_RET_SHIFT                   (0x00000003U)
#define CSL_TOP_RCM_PLL_ETH_PWRCTRL_RET_RESETVAL                (0x00000000U)
#define CSL_TOP_RCM_PLL_ETH_PWRCTRL_RET_MAX                     (0x00000001U)



/* PLL_ETH_CLKCTRL */
#define CSL_TOP_RCM_PLL_ETH_CLKCTRL_BYPASSACKZ_MASK             (0x00400000U)
#define CSL_TOP_RCM_PLL_ETH_CLKCTRL_BYPASSACKZ_SHIFT            (0x00000016U)
#define CSL_TOP_RCM_PLL_ETH_CLKCTRL_BYPASSACKZ_RESETVAL         (0x00000000U)
#define CSL_TOP_RCM_PLL_ETH_CLKCTRL_BYPASSACKZ_MAX              (0x00000001U)


#define CSL_TOP_RCM_PLL_ETH_CLKCTRL_CLKDCOLDOEN_MASK            (0x20000000U)
#define CSL_TOP_RCM_PLL_ETH_CLKCTRL_CLKDCOLDOEN_SHIFT           (0x0000001DU)
#define CSL_TOP_RCM_PLL_ETH_CLKCTRL_CLKDCOLDOEN_RESETVAL        (0x00000000U)
#define CSL_TOP_RCM_PLL_ETH_CLKCTRL_CLKDCOLDOEN_MAX             (0x00000001U)


#define CSL_TOP_RCM_PLL_ETH_CLKCTRL_CLKDCOLDOPWDNZ_MASK         (0x00020000U)
#define CSL_TOP_RCM_PLL_ETH_CLKCTRL_CLKDCOLDOPWDNZ_SHIFT        (0x00000011U)
#define CSL_TOP_RCM_PLL_ETH_CLKCTRL_CLKDCOLDOPWDNZ_RESETVAL     (0x00000000U)
#define CSL_TOP_RCM_PLL_ETH_CLKCTRL_CLKDCOLDOPWDNZ_MAX          (0x00000001U)


#define CSL_TOP_RCM_PLL_ETH_CLKCTRL_CLKOUTEN_MASK               (0x00100000U)
#define CSL_TOP_RCM_PLL_ETH_CLKCTRL_CLKOUTEN_SHIFT              (0x00000014U)
#define CSL_TOP_RCM_PLL_ETH_CLKCTRL_CLKOUTEN_RESETVAL           (0x00000000U)
#define CSL_TOP_RCM_PLL_ETH_CLKCTRL_CLKOUTEN_MAX                (0x00000001U)


#define CSL_TOP_RCM_PLL_ETH_CLKCTRL_CLKOUTLDOEN_MASK            (0x00080000U)
#define CSL_TOP_RCM_PLL_ETH_CLKCTRL_CLKOUTLDOEN_SHIFT           (0x00000013U)
#define CSL_TOP_RCM_PLL_ETH_CLKCTRL_CLKOUTLDOEN_RESETVAL        (0x00000001U)
#define CSL_TOP_RCM_PLL_ETH_CLKCTRL_CLKOUTLDOEN_MAX             (0x00000001U)


#define CSL_TOP_RCM_PLL_ETH_CLKCTRL_CYCLESLIPEN_MASK            (0x80000000U)
#define CSL_TOP_RCM_PLL_ETH_CLKCTRL_CYCLESLIPEN_SHIFT           (0x0000001FU)
#define CSL_TOP_RCM_PLL_ETH_CLKCTRL_CYCLESLIPEN_RESETVAL        (0x00000000U)
#define CSL_TOP_RCM_PLL_ETH_CLKCTRL_CYCLESLIPEN_MAX             (0x00000001U)


#define CSL_TOP_RCM_PLL_ETH_CLKCTRL_ENSSC_MASK                  (0x40000000U)
#define CSL_TOP_RCM_PLL_ETH_CLKCTRL_ENSSC_SHIFT                 (0x0000001EU)
#define CSL_TOP_RCM_PLL_ETH_CLKCTRL_ENSSC_RESETVAL              (0x00000000U)
#define CSL_TOP_RCM_PLL_ETH_CLKCTRL_ENSSC_MAX                   (0x00000001U)


#define CSL_TOP_RCM_PLL_ETH_CLKCTRL_IDLE_MASK                   (0x00800000U)
#define CSL_TOP_RCM_PLL_ETH_CLKCTRL_IDLE_SHIFT                  (0x00000017U)
#define CSL_TOP_RCM_PLL_ETH_CLKCTRL_IDLE_RESETVAL               (0x00000001U)
#define CSL_TOP_RCM_PLL_ETH_CLKCTRL_IDLE_MAX                    (0x00000001U)


#define CSL_TOP_RCM_PLL_ETH_CLKCTRL_M2PWDNZ_MASK                (0x00010000U)
#define CSL_TOP_RCM_PLL_ETH_CLKCTRL_M2PWDNZ_SHIFT               (0x00000010U)
#define CSL_TOP_RCM_PLL_ETH_CLKCTRL_M2PWDNZ_RESETVAL            (0x00000001U)
#define CSL_TOP_RCM_PLL_ETH_CLKCTRL_M2PWDNZ_MAX                 (0x00000001U)


#define CSL_TOP_RCM_PLL_ETH_CLKCTRL_RELAXED_LOCK_MASK           (0x00000100U)
#define CSL_TOP_RCM_PLL_ETH_CLKCTRL_RELAXED_LOCK_SHIFT          (0x00000008U)
#define CSL_TOP_RCM_PLL_ETH_CLKCTRL_RELAXED_LOCK_RESETVAL       (0x00000000U)
#define CSL_TOP_RCM_PLL_ETH_CLKCTRL_RELAXED_LOCK_MAX            (0x00000001U)


#define CSL_TOP_RCM_PLL_ETH_CLKCTRL_SELFREQDCO_MASK             (0x00001C00U)
#define CSL_TOP_RCM_PLL_ETH_CLKCTRL_SELFREQDCO_SHIFT            (0x0000000AU)
#define CSL_TOP_RCM_PLL_ETH_CLKCTRL_SELFREQDCO_RESETVAL         (0x00000004U)
#define CSL_TOP_RCM_PLL_ETH_CLKCTRL_SELFREQDCO_MAX              (0x00000007U)


#define CSL_TOP_RCM_PLL_ETH_CLKCTRL_SSCTYPE_MASK                (0x00000002U)
#define CSL_TOP_RCM_PLL_ETH_CLKCTRL_SSCTYPE_SHIFT               (0x00000001U)
#define CSL_TOP_RCM_PLL_ETH_CLKCTRL_SSCTYPE_RESETVAL            (0x00000000U)
#define CSL_TOP_RCM_PLL_ETH_CLKCTRL_SSCTYPE_MAX                 (0x00000001U)


#define CSL_TOP_RCM_PLL_ETH_CLKCTRL_STBYRET_MASK                (0x00200000U)
#define CSL_TOP_RCM_PLL_ETH_CLKCTRL_STBYRET_SHIFT               (0x00000015U)
#define CSL_TOP_RCM_PLL_ETH_CLKCTRL_STBYRET_RESETVAL            (0x00000000U)
#define CSL_TOP_RCM_PLL_ETH_CLKCTRL_STBYRET_MAX                 (0x00000001U)


#define CSL_TOP_RCM_PLL_ETH_CLKCTRL_STOPMODE_MASK               (0x00004000U)
#define CSL_TOP_RCM_PLL_ETH_CLKCTRL_STOPMODE_SHIFT              (0x0000000EU)
#define CSL_TOP_RCM_PLL_ETH_CLKCTRL_STOPMODE_RESETVAL           (0x00000001U)
#define CSL_TOP_RCM_PLL_ETH_CLKCTRL_STOPMODE_MAX                (0x00000001U)


#define CSL_TOP_RCM_PLL_ETH_CLKCTRL_TINTZ_MASK                  (0x00000001U)
#define CSL_TOP_RCM_PLL_ETH_CLKCTRL_TINTZ_SHIFT                 (0x00000000U)
#define CSL_TOP_RCM_PLL_ETH_CLKCTRL_TINTZ_RESETVAL              (0x00000000U)
#define CSL_TOP_RCM_PLL_ETH_CLKCTRL_TINTZ_MAX                   (0x00000001U)


#define CSL_TOP_RCM_PLL_ETH_CLKCTRL_ULOWCLKEN_MASK              (0x00040000U)
#define CSL_TOP_RCM_PLL_ETH_CLKCTRL_ULOWCLKEN_SHIFT             (0x00000012U)
#define CSL_TOP_RCM_PLL_ETH_CLKCTRL_ULOWCLKEN_RESETVAL          (0x00000000U)
#define CSL_TOP_RCM_PLL_ETH_CLKCTRL_ULOWCLKEN_MAX               (0x00000001U)



/* PLL_ETH_TENABLE */
#define CSL_TOP_RCM_PLL_ETH_TENABLE_TENABLE_MASK                (0x00000001U)
#define CSL_TOP_RCM_PLL_ETH_TENABLE_TENABLE_SHIFT               (0x00000000U)
#define CSL_TOP_RCM_PLL_ETH_TENABLE_TENABLE_RESETVAL            (0x00000000U)
#define CSL_TOP_RCM_PLL_ETH_TENABLE_TENABLE_MAX                 (0x00000001U)



/* PLL_ETH_TENABLEDIV */
#define CSL_TOP_RCM_PLL_ETH_TENABLEDIV_TENABLEDIV_MASK       (0x00000001U)
#define CSL_TOP_RCM_PLL_ETH_TENABLEDIV_TENABLEDIV_SHIFT      (0x00000000U)
#define CSL_TOP_RCM_PLL_ETH_TENABLEDIV_TENABLEDIV_RESETVAL   (0x00000000U)
#define CSL_TOP_RCM_PLL_ETH_TENABLEDIV_TENABLEDIV_MAX        (0x00000001U)



/* PLL_ETH_M2NDIV */
#define CSL_TOP_RCM_PLL_ETH_M2NDIV_M2_MASK                       (0x007F0000U)
#define CSL_TOP_RCM_PLL_ETH_M2NDIV_M2_SHIFT                      (0x00000010U)
#define CSL_TOP_RCM_PLL_ETH_M2NDIV_M2_RESETVAL                   (0x00000000U)
#define CSL_TOP_RCM_PLL_ETH_M2NDIV_M2_MAX                        (0x0000007FU)


#define CSL_TOP_RCM_PLL_ETH_M2NDIV_N_MASK                        (0x000000FFU)
#define CSL_TOP_RCM_PLL_ETH_M2NDIV_N_SHIFT                       (0x00000000U)
#define CSL_TOP_RCM_PLL_ETH_M2NDIV_N_RESETVAL                    (0x00000013U)
#define CSL_TOP_RCM_PLL_ETH_M2NDIV_N_MAX                         (0x000000FFU)



/* PLL_ETH_MN2DIV */
#define CSL_TOP_RCM_PLL_ETH_MN2DIV_M_MASK                        (0x00000FFFU)
#define CSL_TOP_RCM_PLL_ETH_MN2DIV_M_SHIFT                       (0x00000000U)
#define CSL_TOP_RCM_PLL_ETH_MN2DIV_M_RESETVAL                    (0x000002D0U)
#define CSL_TOP_RCM_PLL_ETH_MN2DIV_M_MAX                         (0x00000FFFU)


#define CSL_TOP_RCM_PLL_ETH_MN2DIV_N2_MASK                       (0x000F0000U)
#define CSL_TOP_RCM_PLL_ETH_MN2DIV_N2_SHIFT                      (0x00000010U)
#define CSL_TOP_RCM_PLL_ETH_MN2DIV_N2_RESETVAL                   (0x00000000U)
#define CSL_TOP_RCM_PLL_ETH_MN2DIV_N2_MAX                        (0x0000000FU)



/* PLL_ETH_FRACDIV */
#define CSL_TOP_RCM_PLL_ETH_FRACDIV_FRACTIONALM_MASK            (0x0003FFFFU)
#define CSL_TOP_RCM_PLL_ETH_FRACDIV_FRACTIONALM_SHIFT           (0x00000000U)
#define CSL_TOP_RCM_PLL_ETH_FRACDIV_FRACTIONALM_RESETVAL        (0x00000000U)
#define CSL_TOP_RCM_PLL_ETH_FRACDIV_FRACTIONALM_MAX             (0x0003FFFFU)


#define CSL_TOP_RCM_PLL_ETH_FRACDIV_REGSD_MASK                  (0xFF000000U)
#define CSL_TOP_RCM_PLL_ETH_FRACDIV_REGSD_SHIFT                 (0x00000018U)
#define CSL_TOP_RCM_PLL_ETH_FRACDIV_REGSD_RESETVAL              (0x00000008U)
#define CSL_TOP_RCM_PLL_ETH_FRACDIV_REGSD_MAX                   (0x000000FFU)



/* PLL_ETH_BWCTRL */
#define CSL_TOP_RCM_PLL_ETH_BWCTRL_BWCONTROL_MASK                (0x00000006U)
#define CSL_TOP_RCM_PLL_ETH_BWCTRL_BWCONTROL_SHIFT               (0x00000001U)
#define CSL_TOP_RCM_PLL_ETH_BWCTRL_BWCONTROL_RESETVAL            (0x00000000U)
#define CSL_TOP_RCM_PLL_ETH_BWCTRL_BWCONTROL_MAX                 (0x00000003U)


#define CSL_TOP_RCM_PLL_ETH_BWCTRL_BW_INCR_DECRZ_MASK            (0x00000001U)
#define CSL_TOP_RCM_PLL_ETH_BWCTRL_BW_INCR_DECRZ_SHIFT           (0x00000000U)
#define CSL_TOP_RCM_PLL_ETH_BWCTRL_BW_INCR_DECRZ_RESETVAL        (0x00000000U)
#define CSL_TOP_RCM_PLL_ETH_BWCTRL_BW_INCR_DECRZ_MAX             (0x00000001U)



/* PLL_ETH_FRACCTRL */
#define CSL_TOP_RCM_PLL_ETH_FRACCTRL_DOWNSPREAD_MASK           (0x80000000U)
#define CSL_TOP_RCM_PLL_ETH_FRACCTRL_DOWNSPREAD_SHIFT          (0x0000001FU)
#define CSL_TOP_RCM_PLL_ETH_FRACCTRL_DOWNSPREAD_RESETVAL       (0x00000000U)
#define CSL_TOP_RCM_PLL_ETH_FRACCTRL_DOWNSPREAD_MAX            (0x00000001U)


#define CSL_TOP_RCM_PLL_ETH_FRACCTRL_DELTAMSTEPFRACTION_MASK   (0x0003FFFFU)
#define CSL_TOP_RCM_PLL_ETH_FRACCTRL_DELTAMSTEPFRACTION_SHIFT  (0x00000000U)
#define CSL_TOP_RCM_PLL_ETH_FRACCTRL_DELTAMSTEPFRACTION_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_PLL_ETH_FRACCTRL_DELTAMSTEPFRACTION_MAX    (0x0003FFFFU)


#define CSL_TOP_RCM_PLL_ETH_FRACCTRL_DELTAMSTEPINTEGER_MASK    (0x001C0000U)
#define CSL_TOP_RCM_PLL_ETH_FRACCTRL_DELTAMSTEPINTEGER_SHIFT   (0x00000012U)
#define CSL_TOP_RCM_PLL_ETH_FRACCTRL_DELTAMSTEPINTEGER_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_PLL_ETH_FRACCTRL_DELTAMSTEPINTEGER_MAX     (0x00000007U)


#define CSL_TOP_RCM_PLL_ETH_FRACCTRL_MODFREQDIVIDEREXPONENT_MASK (0x70000000U)
#define CSL_TOP_RCM_PLL_ETH_FRACCTRL_MODFREQDIVIDEREXPONENT_SHIFT (0x0000001CU)
#define CSL_TOP_RCM_PLL_ETH_FRACCTRL_MODFREQDIVIDEREXPONENT_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_PLL_ETH_FRACCTRL_MODFREQDIVIDEREXPONENT_MAX (0x00000007U)


#define CSL_TOP_RCM_PLL_ETH_FRACCTRL_MODFREQDIVIDERMANTISSA_MASK (0x0FE00000U)
#define CSL_TOP_RCM_PLL_ETH_FRACCTRL_MODFREQDIVIDERMANTISSA_SHIFT (0x00000015U)
#define CSL_TOP_RCM_PLL_ETH_FRACCTRL_MODFREQDIVIDERMANTISSA_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_PLL_ETH_FRACCTRL_MODFREQDIVIDERMANTISSA_MAX (0x0000007FU)



/* PLL_ETH_STATUS */
#define CSL_TOP_RCM_PLL_ETH_STATUS_BYPASS_MASK                   (0x00000001U)
#define CSL_TOP_RCM_PLL_ETH_STATUS_BYPASS_SHIFT                  (0x00000000U)
#define CSL_TOP_RCM_PLL_ETH_STATUS_BYPASS_RESETVAL               (0x00000001U)
#define CSL_TOP_RCM_PLL_ETH_STATUS_BYPASS_MAX                    (0x00000001U)


#define CSL_TOP_RCM_PLL_ETH_STATUS_BYPASSACK_MASK                (0x00000100U)
#define CSL_TOP_RCM_PLL_ETH_STATUS_BYPASSACK_SHIFT               (0x00000008U)
#define CSL_TOP_RCM_PLL_ETH_STATUS_BYPASSACK_RESETVAL            (0x00000001U)
#define CSL_TOP_RCM_PLL_ETH_STATUS_BYPASSACK_MAX                 (0x00000001U)


#define CSL_TOP_RCM_PLL_ETH_STATUS_CLKDCOLDOACK_MASK             (0x00000800U)
#define CSL_TOP_RCM_PLL_ETH_STATUS_CLKDCOLDOACK_SHIFT            (0x0000000BU)
#define CSL_TOP_RCM_PLL_ETH_STATUS_CLKDCOLDOACK_RESETVAL         (0x00000000U)
#define CSL_TOP_RCM_PLL_ETH_STATUS_CLKDCOLDOACK_MAX              (0x00000001U)


#define CSL_TOP_RCM_PLL_ETH_STATUS_CLKOUTENACK_MASK              (0x00000020U)
#define CSL_TOP_RCM_PLL_ETH_STATUS_CLKOUTENACK_SHIFT             (0x00000005U)
#define CSL_TOP_RCM_PLL_ETH_STATUS_CLKOUTENACK_RESETVAL          (0x00000000U)
#define CSL_TOP_RCM_PLL_ETH_STATUS_CLKOUTENACK_MAX               (0x00000001U)


#define CSL_TOP_RCM_PLL_ETH_STATUS_CLKOUTLDOENACK_MASK           (0x00001000U)
#define CSL_TOP_RCM_PLL_ETH_STATUS_CLKOUTLDOENACK_SHIFT          (0x0000000CU)
#define CSL_TOP_RCM_PLL_ETH_STATUS_CLKOUTLDOENACK_RESETVAL       (0x00000001U)
#define CSL_TOP_RCM_PLL_ETH_STATUS_CLKOUTLDOENACK_MAX            (0x00000001U)


#define CSL_TOP_RCM_PLL_ETH_STATUS_FREQLOCK_MASK                 (0x00000200U)
#define CSL_TOP_RCM_PLL_ETH_STATUS_FREQLOCK_SHIFT                (0x00000009U)
#define CSL_TOP_RCM_PLL_ETH_STATUS_FREQLOCK_RESETVAL             (0x00000000U)
#define CSL_TOP_RCM_PLL_ETH_STATUS_FREQLOCK_MAX                  (0x00000001U)


#define CSL_TOP_RCM_PLL_ETH_STATUS_HIGHJITTER_MASK               (0x00000002U)
#define CSL_TOP_RCM_PLL_ETH_STATUS_HIGHJITTER_SHIFT              (0x00000001U)
#define CSL_TOP_RCM_PLL_ETH_STATUS_HIGHJITTER_RESETVAL           (0x00000000U)
#define CSL_TOP_RCM_PLL_ETH_STATUS_HIGHJITTER_MAX                (0x00000001U)


#define CSL_TOP_RCM_PLL_ETH_STATUS_LDOPWDN_MASK                  (0x20000000U)
#define CSL_TOP_RCM_PLL_ETH_STATUS_LDOPWDN_SHIFT                 (0x0000001DU)
#define CSL_TOP_RCM_PLL_ETH_STATUS_LDOPWDN_RESETVAL              (0x00000001U)
#define CSL_TOP_RCM_PLL_ETH_STATUS_LDOPWDN_MAX                   (0x00000001U)


#define CSL_TOP_RCM_PLL_ETH_STATUS_LOCK2_MASK                    (0x00000010U)
#define CSL_TOP_RCM_PLL_ETH_STATUS_LOCK2_SHIFT                   (0x00000004U)
#define CSL_TOP_RCM_PLL_ETH_STATUS_LOCK2_RESETVAL                (0x00000000U)
#define CSL_TOP_RCM_PLL_ETH_STATUS_LOCK2_MAX                     (0x00000001U)


#define CSL_TOP_RCM_PLL_ETH_STATUS_LOSSREF_MASK                  (0x00000040U)
#define CSL_TOP_RCM_PLL_ETH_STATUS_LOSSREF_SHIFT                 (0x00000006U)
#define CSL_TOP_RCM_PLL_ETH_STATUS_LOSSREF_RESETVAL              (0x00000001U)
#define CSL_TOP_RCM_PLL_ETH_STATUS_LOSSREF_MAX                   (0x00000001U)


#define CSL_TOP_RCM_PLL_ETH_STATUS_M2CHANGEACK_MASK              (0x00000008U)
#define CSL_TOP_RCM_PLL_ETH_STATUS_M2CHANGEACK_SHIFT             (0x00000003U)
#define CSL_TOP_RCM_PLL_ETH_STATUS_M2CHANGEACK_RESETVAL          (0x00000000U)
#define CSL_TOP_RCM_PLL_ETH_STATUS_M2CHANGEACK_MAX               (0x00000001U)


#define CSL_TOP_RCM_PLL_ETH_STATUS_PGOODOUT_MASK                 (0x40000000U)
#define CSL_TOP_RCM_PLL_ETH_STATUS_PGOODOUT_SHIFT                (0x0000001EU)
#define CSL_TOP_RCM_PLL_ETH_STATUS_PGOODOUT_RESETVAL             (0x00000001U)
#define CSL_TOP_RCM_PLL_ETH_STATUS_PGOODOUT_MAX                  (0x00000001U)


#define CSL_TOP_RCM_PLL_ETH_STATUS_PHASELOCK_MASK                (0x00000400U)
#define CSL_TOP_RCM_PLL_ETH_STATUS_PHASELOCK_SHIFT               (0x0000000AU)
#define CSL_TOP_RCM_PLL_ETH_STATUS_PHASELOCK_RESETVAL            (0x00000000U)
#define CSL_TOP_RCM_PLL_ETH_STATUS_PHASELOCK_MAX                 (0x00000001U)


#define CSL_TOP_RCM_PLL_ETH_STATUS_PONOUT_MASK                   (0x80000000U)
#define CSL_TOP_RCM_PLL_ETH_STATUS_PONOUT_SHIFT                  (0x0000001FU)
#define CSL_TOP_RCM_PLL_ETH_STATUS_PONOUT_RESETVAL               (0x00000001U)
#define CSL_TOP_RCM_PLL_ETH_STATUS_PONOUT_MAX                    (0x00000001U)


#define CSL_TOP_RCM_PLL_ETH_STATUS_RECAL_BSTATUS3_MASK           (0x10000000U)
#define CSL_TOP_RCM_PLL_ETH_STATUS_RECAL_BSTATUS3_SHIFT          (0x0000001CU)
#define CSL_TOP_RCM_PLL_ETH_STATUS_RECAL_BSTATUS3_RESETVAL       (0x00000000U)
#define CSL_TOP_RCM_PLL_ETH_STATUS_RECAL_BSTATUS3_MAX            (0x00000001U)


#define CSL_TOP_RCM_PLL_ETH_STATUS_RECAL_OPPIN_MASK              (0x08000000U)
#define CSL_TOP_RCM_PLL_ETH_STATUS_RECAL_OPPIN_SHIFT             (0x0000001BU)
#define CSL_TOP_RCM_PLL_ETH_STATUS_RECAL_OPPIN_RESETVAL          (0x00000000U)
#define CSL_TOP_RCM_PLL_ETH_STATUS_RECAL_OPPIN_MAX               (0x00000001U)


#define CSL_TOP_RCM_PLL_ETH_STATUS_SSCACK_MASK                   (0x00000004U)
#define CSL_TOP_RCM_PLL_ETH_STATUS_SSCACK_SHIFT                  (0x00000002U)
#define CSL_TOP_RCM_PLL_ETH_STATUS_SSCACK_RESETVAL               (0x00000000U)
#define CSL_TOP_RCM_PLL_ETH_STATUS_SSCACK_MAX                    (0x00000001U)


#define CSL_TOP_RCM_PLL_ETH_STATUS_STBYRETACK_MASK               (0x00000080U)
#define CSL_TOP_RCM_PLL_ETH_STATUS_STBYRETACK_SHIFT              (0x00000007U)
#define CSL_TOP_RCM_PLL_ETH_STATUS_STBYRETACK_RESETVAL           (0x00000000U)
#define CSL_TOP_RCM_PLL_ETH_STATUS_STBYRETACK_MAX                (0x00000001U)



/* PLL_ETH_HSDIVIDER */
#define CSL_TOP_RCM_PLL_ETH_HSDIVIDER_BYPASS_MASK             (0x00000001U)
#define CSL_TOP_RCM_PLL_ETH_HSDIVIDER_BYPASS_SHIFT            (0x00000000U)
#define CSL_TOP_RCM_PLL_ETH_HSDIVIDER_BYPASS_RESETVAL         (0x00000000U)
#define CSL_TOP_RCM_PLL_ETH_HSDIVIDER_BYPASS_MAX              (0x00000001U)


#define CSL_TOP_RCM_PLL_ETH_HSDIVIDER_BYPASSACKZ_MASK         (0x00010000U)
#define CSL_TOP_RCM_PLL_ETH_HSDIVIDER_BYPASSACKZ_SHIFT        (0x00000010U)
#define CSL_TOP_RCM_PLL_ETH_HSDIVIDER_BYPASSACKZ_RESETVAL     (0x00000000U)
#define CSL_TOP_RCM_PLL_ETH_HSDIVIDER_BYPASSACKZ_MAX          (0x00000001U)


#define CSL_TOP_RCM_PLL_ETH_HSDIVIDER_LDOPWDN_MASK            (0x00000002U)
#define CSL_TOP_RCM_PLL_ETH_HSDIVIDER_LDOPWDN_SHIFT           (0x00000001U)
#define CSL_TOP_RCM_PLL_ETH_HSDIVIDER_LDOPWDN_RESETVAL        (0x00000000U)
#define CSL_TOP_RCM_PLL_ETH_HSDIVIDER_LDOPWDN_MAX             (0x00000001U)


#define CSL_TOP_RCM_PLL_ETH_HSDIVIDER_LDOPWDNACK_MASK         (0x00020000U)
#define CSL_TOP_RCM_PLL_ETH_HSDIVIDER_LDOPWDNACK_SHIFT        (0x00000011U)
#define CSL_TOP_RCM_PLL_ETH_HSDIVIDER_LDOPWDNACK_RESETVAL     (0x00000000U)
#define CSL_TOP_RCM_PLL_ETH_HSDIVIDER_LDOPWDNACK_MAX          (0x00000001U)


#define CSL_TOP_RCM_PLL_ETH_HSDIVIDER_TENABLEDIV_MASK         (0x00000004U)
#define CSL_TOP_RCM_PLL_ETH_HSDIVIDER_TENABLEDIV_SHIFT        (0x00000002U)
#define CSL_TOP_RCM_PLL_ETH_HSDIVIDER_TENABLEDIV_RESETVAL     (0x00000000U)
#define CSL_TOP_RCM_PLL_ETH_HSDIVIDER_TENABLEDIV_MAX          (0x00000001U)



/* PLL_ETH_HSDIVIDER_CLKOUT0 */
#define CSL_TOP_RCM_PLL_ETH_HSDIVIDER_CLKOUT0_DIV_MASK (0x0000001FU)
#define CSL_TOP_RCM_PLL_ETH_HSDIVIDER_CLKOUT0_DIV_SHIFT (0x00000000U)
#define CSL_TOP_RCM_PLL_ETH_HSDIVIDER_CLKOUT0_DIV_RESETVAL (0x00000001U)
#define CSL_TOP_RCM_PLL_ETH_HSDIVIDER_CLKOUT0_DIV_MAX (0x0000001FU)


#define CSL_TOP_RCM_PLL_ETH_HSDIVIDER_CLKOUT0_DIVCHACK_MASK (0x00000020U)
#define CSL_TOP_RCM_PLL_ETH_HSDIVIDER_CLKOUT0_DIVCHACK_SHIFT (0x00000005U)
#define CSL_TOP_RCM_PLL_ETH_HSDIVIDER_CLKOUT0_DIVCHACK_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_PLL_ETH_HSDIVIDER_CLKOUT0_DIVCHACK_MAX (0x00000001U)


#define CSL_TOP_RCM_PLL_ETH_HSDIVIDER_CLKOUT0_GATE_CTRL_MASK (0x00000100U)
#define CSL_TOP_RCM_PLL_ETH_HSDIVIDER_CLKOUT0_GATE_CTRL_SHIFT (0x00000008U)
#define CSL_TOP_RCM_PLL_ETH_HSDIVIDER_CLKOUT0_GATE_CTRL_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_PLL_ETH_HSDIVIDER_CLKOUT0_GATE_CTRL_MAX (0x00000001U)


#define CSL_TOP_RCM_PLL_ETH_HSDIVIDER_CLKOUT0_PWDN_MASK (0x00001000U)
#define CSL_TOP_RCM_PLL_ETH_HSDIVIDER_CLKOUT0_PWDN_SHIFT (0x0000000CU)
#define CSL_TOP_RCM_PLL_ETH_HSDIVIDER_CLKOUT0_PWDN_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_PLL_ETH_HSDIVIDER_CLKOUT0_PWDN_MAX (0x00000001U)


#define CSL_TOP_RCM_PLL_ETH_HSDIVIDER_CLKOUT0_STATUS_MASK (0x00000200U)
#define CSL_TOP_RCM_PLL_ETH_HSDIVIDER_CLKOUT0_STATUS_SHIFT (0x00000009U)
#define CSL_TOP_RCM_PLL_ETH_HSDIVIDER_CLKOUT0_STATUS_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_PLL_ETH_HSDIVIDER_CLKOUT0_STATUS_MAX (0x00000001U)



/* PLL_ETH_HSDIVIDER_CLKOUT2 */
#define CSL_TOP_RCM_PLL_ETH_HSDIVIDER_CLKOUT2_DIV_MASK (0x0000001FU)
#define CSL_TOP_RCM_PLL_ETH_HSDIVIDER_CLKOUT2_DIV_SHIFT (0x00000000U)
#define CSL_TOP_RCM_PLL_ETH_HSDIVIDER_CLKOUT2_DIV_RESETVAL (0x00000005U)
#define CSL_TOP_RCM_PLL_ETH_HSDIVIDER_CLKOUT2_DIV_MAX (0x0000001FU)


#define CSL_TOP_RCM_PLL_ETH_HSDIVIDER_CLKOUT2_DIVCHACK_MASK (0x00000020U)
#define CSL_TOP_RCM_PLL_ETH_HSDIVIDER_CLKOUT2_DIVCHACK_SHIFT (0x00000005U)
#define CSL_TOP_RCM_PLL_ETH_HSDIVIDER_CLKOUT2_DIVCHACK_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_PLL_ETH_HSDIVIDER_CLKOUT2_DIVCHACK_MAX (0x00000001U)


#define CSL_TOP_RCM_PLL_ETH_HSDIVIDER_CLKOUT2_GATE_CTRL_MASK (0x00000100U)
#define CSL_TOP_RCM_PLL_ETH_HSDIVIDER_CLKOUT2_GATE_CTRL_SHIFT (0x00000008U)
#define CSL_TOP_RCM_PLL_ETH_HSDIVIDER_CLKOUT2_GATE_CTRL_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_PLL_ETH_HSDIVIDER_CLKOUT2_GATE_CTRL_MAX (0x00000001U)


#define CSL_TOP_RCM_PLL_ETH_HSDIVIDER_CLKOUT2_PWDN_MASK (0x00001000U)
#define CSL_TOP_RCM_PLL_ETH_HSDIVIDER_CLKOUT2_PWDN_SHIFT (0x0000000CU)
#define CSL_TOP_RCM_PLL_ETH_HSDIVIDER_CLKOUT2_PWDN_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_PLL_ETH_HSDIVIDER_CLKOUT2_PWDN_MAX (0x00000001U)


#define CSL_TOP_RCM_PLL_ETH_HSDIVIDER_CLKOUT2_STATUS_MASK (0x00000200U)
#define CSL_TOP_RCM_PLL_ETH_HSDIVIDER_CLKOUT2_STATUS_SHIFT (0x00000009U)
#define CSL_TOP_RCM_PLL_ETH_HSDIVIDER_CLKOUT2_STATUS_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_PLL_ETH_HSDIVIDER_CLKOUT2_STATUS_MAX (0x00000001U)



/* PLL_ETH_RSTCTRL */
#define CSL_TOP_RCM_PLL_ETH_RSTCTRL_ASSERT_MASK                 (0x00000007U)
#define CSL_TOP_RCM_PLL_ETH_RSTCTRL_ASSERT_SHIFT                (0x00000000U)
#define CSL_TOP_RCM_PLL_ETH_RSTCTRL_ASSERT_RESETVAL             (0x00000000U)
#define CSL_TOP_RCM_PLL_ETH_RSTCTRL_ASSERT_MAX                  (0x00000007U)



/* PLL_ETH_HSDIVIDER_RSTCTRL */
#define CSL_TOP_RCM_PLL_ETH_HSDIVIDER_RSTCTRL_ASSERT_MASK (0x00000007U)
#define CSL_TOP_RCM_PLL_ETH_HSDIVIDER_RSTCTRL_ASSERT_SHIFT (0x00000000U)
#define CSL_TOP_RCM_PLL_ETH_HSDIVIDER_RSTCTRL_ASSERT_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_PLL_ETH_HSDIVIDER_RSTCTRL_ASSERT_MAX (0x00000007U)



/* CLKOUT0_CLK_SRC_SEL */
#define CSL_TOP_RCM_CLKOUT0_CLK_SRC_SEL_CLKSRCSEL_MASK      (0x00000FFFU)
#define CSL_TOP_RCM_CLKOUT0_CLK_SRC_SEL_CLKSRCSEL_SHIFT     (0x00000000U)
#define CSL_TOP_RCM_CLKOUT0_CLK_SRC_SEL_CLKSRCSEL_RESETVAL  (0x00000000U)
#define CSL_TOP_RCM_CLKOUT0_CLK_SRC_SEL_CLKSRCSEL_MAX       (0x00000FFFU)



/* CLKOUT1_CLK_SRC_SEL */
#define CSL_TOP_RCM_CLKOUT1_CLK_SRC_SEL_CLKSRCSEL_MASK      (0x00000FFFU)
#define CSL_TOP_RCM_CLKOUT1_CLK_SRC_SEL_CLKSRCSEL_SHIFT     (0x00000000U)
#define CSL_TOP_RCM_CLKOUT1_CLK_SRC_SEL_CLKSRCSEL_RESETVAL  (0x00000000U)
#define CSL_TOP_RCM_CLKOUT1_CLK_SRC_SEL_CLKSRCSEL_MAX       (0x00000FFFU)



/* CLKOUT0_DIV_VAL */
#define CSL_TOP_RCM_CLKOUT0_DIV_VAL_CLKDIV_MASK                 (0x00000FFFU)
#define CSL_TOP_RCM_CLKOUT0_DIV_VAL_CLKDIV_SHIFT                (0x00000000U)
#define CSL_TOP_RCM_CLKOUT0_DIV_VAL_CLKDIV_RESETVAL             (0x00000000U)
#define CSL_TOP_RCM_CLKOUT0_DIV_VAL_CLKDIV_MAX                  (0x00000FFFU)



/* CLKOUT1_DIV_VAL */
#define CSL_TOP_RCM_CLKOUT1_DIV_VAL_CLKDIV_MASK                 (0x00000FFFU)
#define CSL_TOP_RCM_CLKOUT1_DIV_VAL_CLKDIV_SHIFT                (0x00000000U)
#define CSL_TOP_RCM_CLKOUT1_DIV_VAL_CLKDIV_RESETVAL             (0x00000000U)
#define CSL_TOP_RCM_CLKOUT1_DIV_VAL_CLKDIV_MAX                  (0x00000FFFU)



/* CLKOUT0_CLK_GATE */
#define CSL_TOP_RCM_CLKOUT0_CLK_GATE_GATED_MASK                (0x00000007U)
#define CSL_TOP_RCM_CLKOUT0_CLK_GATE_GATED_SHIFT               (0x00000000U)
#define CSL_TOP_RCM_CLKOUT0_CLK_GATE_GATED_RESETVAL            (0x00000000U)
#define CSL_TOP_RCM_CLKOUT0_CLK_GATE_GATED_MAX                 (0x00000007U)



/* CLKOUT1_CLK_GATE */
#define CSL_TOP_RCM_CLKOUT1_CLK_GATE_GATED_MASK                (0x00000007U)
#define CSL_TOP_RCM_CLKOUT1_CLK_GATE_GATED_SHIFT               (0x00000000U)
#define CSL_TOP_RCM_CLKOUT1_CLK_GATE_GATED_RESETVAL            (0x00000000U)
#define CSL_TOP_RCM_CLKOUT1_CLK_GATE_GATED_MAX                 (0x00000007U)



/* CLKOUT0_CLK_STATUS */
#define CSL_TOP_RCM_CLKOUT0_CLK_STATUS_CLKINUSE_MASK         (0x000000FFU)
#define CSL_TOP_RCM_CLKOUT0_CLK_STATUS_CLKINUSE_SHIFT        (0x00000000U)
#define CSL_TOP_RCM_CLKOUT0_CLK_STATUS_CLKINUSE_RESETVAL     (0x00000001U)
#define CSL_TOP_RCM_CLKOUT0_CLK_STATUS_CLKINUSE_MAX          (0x000000FFU)


#define CSL_TOP_RCM_CLKOUT0_CLK_STATUS_CURRDIVIDER_MASK      (0x0000FF00U)
#define CSL_TOP_RCM_CLKOUT0_CLK_STATUS_CURRDIVIDER_SHIFT     (0x00000008U)
#define CSL_TOP_RCM_CLKOUT0_CLK_STATUS_CURRDIVIDER_RESETVAL  (0x00000000U)
#define CSL_TOP_RCM_CLKOUT0_CLK_STATUS_CURRDIVIDER_MAX       (0x000000FFU)



/* CLKOUT1_CLK_STATUS */
#define CSL_TOP_RCM_CLKOUT1_CLK_STATUS_CLKINUSE_MASK         (0x000000FFU)
#define CSL_TOP_RCM_CLKOUT1_CLK_STATUS_CLKINUSE_SHIFT        (0x00000000U)
#define CSL_TOP_RCM_CLKOUT1_CLK_STATUS_CLKINUSE_RESETVAL     (0x00000001U)
#define CSL_TOP_RCM_CLKOUT1_CLK_STATUS_CLKINUSE_MAX          (0x000000FFU)


#define CSL_TOP_RCM_CLKOUT1_CLK_STATUS_CURRDIVIDER_MASK      (0x0000FF00U)
#define CSL_TOP_RCM_CLKOUT1_CLK_STATUS_CURRDIVIDER_SHIFT     (0x00000008U)
#define CSL_TOP_RCM_CLKOUT1_CLK_STATUS_CURRDIVIDER_RESETVAL  (0x00000000U)
#define CSL_TOP_RCM_CLKOUT1_CLK_STATUS_CURRDIVIDER_MAX       (0x000000FFU)



/* TRCCLKOUT_CLK_SRC_SEL */
#define CSL_TOP_RCM_TRCCLKOUT_CLK_SRC_SEL_CLKSRCSEL_MASK  (0x00000FFFU)
#define CSL_TOP_RCM_TRCCLKOUT_CLK_SRC_SEL_CLKSRCSEL_SHIFT (0x00000000U)
#define CSL_TOP_RCM_TRCCLKOUT_CLK_SRC_SEL_CLKSRCSEL_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_TRCCLKOUT_CLK_SRC_SEL_CLKSRCSEL_MAX   (0x00000FFFU)



/* TRCCLKOUT_DIV_VAL */
#define CSL_TOP_RCM_TRCCLKOUT_DIV_VAL_CLKDIV_MASK             (0x00000FFFU)
#define CSL_TOP_RCM_TRCCLKOUT_DIV_VAL_CLKDIV_SHIFT            (0x00000000U)
#define CSL_TOP_RCM_TRCCLKOUT_DIV_VAL_CLKDIV_RESETVAL         (0x00000000U)
#define CSL_TOP_RCM_TRCCLKOUT_DIV_VAL_CLKDIV_MAX              (0x00000FFFU)



/* TRCCLKOUT_CLK_GATE */
#define CSL_TOP_RCM_TRCCLKOUT_CLK_GATE_GATED_MASK            (0x00000007U)
#define CSL_TOP_RCM_TRCCLKOUT_CLK_GATE_GATED_SHIFT           (0x00000000U)
#define CSL_TOP_RCM_TRCCLKOUT_CLK_GATE_GATED_RESETVAL        (0x00000000U)
#define CSL_TOP_RCM_TRCCLKOUT_CLK_GATE_GATED_MAX             (0x00000007U)



/* TRCCLKOUT_CLK_STATUS */
#define CSL_TOP_RCM_TRCCLKOUT_CLK_STATUS_CLKINUSE_MASK     (0x000000FFU)
#define CSL_TOP_RCM_TRCCLKOUT_CLK_STATUS_CLKINUSE_SHIFT    (0x00000000U)
#define CSL_TOP_RCM_TRCCLKOUT_CLK_STATUS_CLKINUSE_RESETVAL (0x00000001U)
#define CSL_TOP_RCM_TRCCLKOUT_CLK_STATUS_CLKINUSE_MAX      (0x000000FFU)


#define CSL_TOP_RCM_TRCCLKOUT_CLK_STATUS_CURRDIVIDER_MASK  (0x0000FF00U)
#define CSL_TOP_RCM_TRCCLKOUT_CLK_STATUS_CURRDIVIDER_SHIFT (0x00000008U)
#define CSL_TOP_RCM_TRCCLKOUT_CLK_STATUS_CURRDIVIDER_RESETVAL (0x00000000U)
#define CSL_TOP_RCM_TRCCLKOUT_CLK_STATUS_CURRDIVIDER_MAX   (0x000000FFU)



/* VMON_CLK_DIV_VAL */
#define CSL_TOP_RCM_VMON_CLK_DIV_VAL_CLKDIV_MASK               (0x00FFFFFFU)
#define CSL_TOP_RCM_VMON_CLK_DIV_VAL_CLKDIV_SHIFT              (0x00000000U)
#define CSL_TOP_RCM_VMON_CLK_DIV_VAL_CLKDIV_RESETVAL           (0x00181818U)
#define CSL_TOP_RCM_VMON_CLK_DIV_VAL_CLKDIV_MAX                (0x00FFFFFFU)



/* VMON_CLK_STATUS */
#define CSL_TOP_RCM_VMON_CLK_STATUS_CURRDIVIDER_MASK            (0x0000FF00U)
#define CSL_TOP_RCM_VMON_CLK_STATUS_CURRDIVIDER_SHIFT           (0x00000008U)
#define CSL_TOP_RCM_VMON_CLK_STATUS_CURRDIVIDER_RESETVAL        (0x00000018U)
#define CSL_TOP_RCM_VMON_CLK_STATUS_CURRDIVIDER_MAX             (0x000000FFU)



/* DFT_DMLED_EXEC */
#define CSL_TOP_RCM_DFT_DMLED_EXEC_VAL_MASK                      (0xFFFFFFFFU)
#define CSL_TOP_RCM_DFT_DMLED_EXEC_VAL_SHIFT                     (0x00000000U)
#define CSL_TOP_RCM_DFT_DMLED_EXEC_VAL_RESETVAL                  (0x00000000U)
#define CSL_TOP_RCM_DFT_DMLED_EXEC_VAL_MAX                       (0xFFFFFFFFU)



/* DFT_DMLED_STATUS */
#define CSL_TOP_RCM_DFT_DMLED_STATUS_VAL_MASK                  (0xFFFFFFFFU)
#define CSL_TOP_RCM_DFT_DMLED_STATUS_VAL_SHIFT                 (0x00000000U)
#define CSL_TOP_RCM_DFT_DMLED_STATUS_VAL_RESETVAL              (0x00000000U)
#define CSL_TOP_RCM_DFT_DMLED_STATUS_VAL_MAX                   (0xFFFFFFFFU)



/* HW_REG0 */

#define CSL_TOP_RCM_HW_REG0_HWREG_MASK                                         (0xFFFFFFFFU)
#define CSL_TOP_RCM_HW_REG0_HWREG_SHIFT                                        (0x00000000U)
#define CSL_TOP_RCM_HW_REG0_HWREG_RESETVAL                                     (0x00000000U)
#define CSL_TOP_RCM_HW_REG0_HWREG_MAX                                          (0xFFFFFFFFU)

/* HW_REG1 */

#define CSL_TOP_RCM_HW_REG1_HWREG_MASK                                         (0xFFFFFFFFU)
#define CSL_TOP_RCM_HW_REG1_HWREG_SHIFT                                        (0x00000000U)
#define CSL_TOP_RCM_HW_REG1_HWREG_RESETVAL                                     (0x00000000U)
#define CSL_TOP_RCM_HW_REG1_HWREG_MAX                                          (0xFFFFFFFFU)

/* HW_REG2 */

#define CSL_TOP_RCM_HW_REG2_HWREG_MASK                                         (0xFFFFFFFFU)
#define CSL_TOP_RCM_HW_REG2_HWREG_SHIFT                                        (0x00000000U)
#define CSL_TOP_RCM_HW_REG2_HWREG_RESETVAL                                     (0x00000000U)
#define CSL_TOP_RCM_HW_REG2_HWREG_MAX                                          (0xFFFFFFFFU)


/* HW_REG3 */

#define CSL_TOP_RCM_HW_REG3_HWREG_MASK                                         (0xFFFFFFFFU)
#define CSL_TOP_RCM_HW_REG3_HWREG_SHIFT                                        (0x00000000U)
#define CSL_TOP_RCM_HW_REG3_HWREG_RESETVAL                                     (0x00000000U)
#define CSL_TOP_RCM_HW_REG3_HWREG_MAX                                          (0xFFFFFFFFU)


/* HW_SPARE_RW0 */

#define CSL_TOP_RCM_HW_SPARE_RW0_HW_SPARE_RW0_MASK                             (0xFFFFFFFFU)
#define CSL_TOP_RCM_HW_SPARE_RW0_HW_SPARE_RW0_SHIFT                            (0x00000000U)
#define CSL_TOP_RCM_HW_SPARE_RW0_HW_SPARE_RW0_RESETVAL                         (0x00000000U)
#define CSL_TOP_RCM_HW_SPARE_RW0_HW_SPARE_RW0_MAX                              (0xFFFFFFFFU)

/* HW_SPARE_RW1 */

#define CSL_TOP_RCM_HW_SPARE_RW1_HW_SPARE_RW1_MASK                             (0xFFFFFFFFU)
#define CSL_TOP_RCM_HW_SPARE_RW1_HW_SPARE_RW1_SHIFT                            (0x00000000U)
#define CSL_TOP_RCM_HW_SPARE_RW1_HW_SPARE_RW1_RESETVAL                         (0x00000000U)
#define CSL_TOP_RCM_HW_SPARE_RW1_HW_SPARE_RW1_MAX                              (0xFFFFFFFFU)

/* HW_SPARE_RW2 */

#define CSL_TOP_RCM_HW_SPARE_RW2_HW_SPARE_RW2_MASK                             (0xFFFFFFFFU)
#define CSL_TOP_RCM_HW_SPARE_RW2_HW_SPARE_RW2_SHIFT                            (0x00000000U)
#define CSL_TOP_RCM_HW_SPARE_RW2_HW_SPARE_RW2_RESETVAL                         (0x00000000U)
#define CSL_TOP_RCM_HW_SPARE_RW2_HW_SPARE_RW2_MAX                              (0xFFFFFFFFU)

/* HW_SPARE_RW3 */

#define CSL_TOP_RCM_HW_SPARE_RW3_HW_SPARE_RW3_MASK                             (0xFFFFFFFFU)
#define CSL_TOP_RCM_HW_SPARE_RW3_HW_SPARE_RW3_SHIFT                            (0x00000000U)
#define CSL_TOP_RCM_HW_SPARE_RW3_HW_SPARE_RW3_RESETVAL                         (0x00000000U)
#define CSL_TOP_RCM_HW_SPARE_RW3_HW_SPARE_RW3_MAX                              (0xFFFFFFFFU)


/* HW_SPARE_RO0 */

#define CSL_TOP_RCM_HW_SPARE_RO0_HW_SPARE_RO0_MASK                             (0xFFFFFFFFU)
#define CSL_TOP_RCM_HW_SPARE_RO0_HW_SPARE_RO0_SHIFT                            (0x00000000U)
#define CSL_TOP_RCM_HW_SPARE_RO0_HW_SPARE_RO0_RESETVAL                         (0x00000000U)
#define CSL_TOP_RCM_HW_SPARE_RO0_HW_SPARE_RO0_MAX                              (0xFFFFFFFFU)


/* HW_SPARE_RO1 */

#define CSL_TOP_RCM_HW_SPARE_RO1_HW_SPARE_RO1_MASK                             (0xFFFFFFFFU)
#define CSL_TOP_RCM_HW_SPARE_RO1_HW_SPARE_RO1_SHIFT                            (0x00000000U)
#define CSL_TOP_RCM_HW_SPARE_RO1_HW_SPARE_RO1_RESETVAL                         (0x00000000U)
#define CSL_TOP_RCM_HW_SPARE_RO1_HW_SPARE_RO1_MAX                              (0xFFFFFFFFU)


/* HW_SPARE_RO2 */

#define CSL_TOP_RCM_HW_SPARE_RO2_HW_SPARE_RO2_MASK                             (0xFFFFFFFFU)
#define CSL_TOP_RCM_HW_SPARE_RO2_HW_SPARE_RO2_SHIFT                            (0x00000000U)
#define CSL_TOP_RCM_HW_SPARE_RO2_HW_SPARE_RO2_RESETVAL                         (0x00000000U)
#define CSL_TOP_RCM_HW_SPARE_RO2_HW_SPARE_RO2_MAX                              (0xFFFFFFFFU)


/* HW_SPARE_RO3 */

#define CSL_TOP_RCM_HW_SPARE_RO3_HW_SPARE_RO3_MASK                             (0xFFFFFFFFU)
#define CSL_TOP_RCM_HW_SPARE_RO3_HW_SPARE_RO3_SHIFT                            (0x00000000U)
#define CSL_TOP_RCM_HW_SPARE_RO3_HW_SPARE_RO3_RESETVAL                         (0x00000000U)
#define CSL_TOP_RCM_HW_SPARE_RO3_HW_SPARE_RO3_MAX                              (0xFFFFFFFFU)


/* HW_SPARE_WPH */

#define CSL_TOP_RCM_HW_SPARE_WPH_HW_SPARE_WPH_MASK                             (0xFFFFFFFFU)
#define CSL_TOP_RCM_HW_SPARE_WPH_HW_SPARE_WPH_SHIFT                            (0x00000000U)
#define CSL_TOP_RCM_HW_SPARE_WPH_HW_SPARE_WPH_RESETVAL                         (0x00000000U)
#define CSL_TOP_RCM_HW_SPARE_WPH_HW_SPARE_WPH_MAX                              (0xFFFFFFFFU)


/* HW_SPARE_REC */
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC0_MASK               				(0x00000001U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC0_SHIFT              				(0x00000000U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC0_RESETVAL           				(0x00000000U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC0_MAX                				(0x00000001U)

#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC1_MASK               				(0x00000002U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC1_SHIFT              				(0x00000001U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC1_RESETVAL           				(0x00000000U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC1_MAX                				(0x00000001U)

#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC2_MASK               				(0x00000004U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC2_SHIFT              				(0x00000002U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC2_RESETVAL           				(0x00000000U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC2_MAX                				(0x00000001U)

#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC3_MASK               				(0x00000008U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC3_SHIFT              				(0x00000003U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC3_RESETVAL           				(0x00000000U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC3_MAX                				(0x00000001U)

#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC4_MASK               				(0x00000010U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC4_SHIFT              				(0x00000004U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC4_RESETVAL           				(0x00000000U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC4_MAX                				(0x00000001U)

#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC5_MASK               				(0x00000020U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC5_SHIFT              				(0x00000005U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC5_RESETVAL           				(0x00000000U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC5_MAX                				(0x00000001U)

#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC6_MASK               				(0x00000040U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC6_SHIFT              				(0x00000006U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC6_RESETVAL           				(0x00000000U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC6_MAX                				(0x00000001U)

#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC7_MASK               				(0x00000080U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC7_SHIFT              				(0x00000007U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC7_RESETVAL           				(0x00000000U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC7_MAX                				(0x00000001U)

#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC8_MASK               				(0x00000100U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC8_SHIFT              				(0x00000008U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC8_RESETVAL           				(0x00000000U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC8_MAX                				(0x00000001U)

#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC9_MASK               				(0x00000200U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC9_SHIFT              				(0x00000009U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC9_RESETVAL           				(0x00000000U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC9_MAX                				(0x00000001U)

#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC10_MASK              				(0x00000400U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC10_SHIFT             				(0x0000000AU)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC10_RESETVAL          				(0x00000000U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC10_MAX               				(0x00000001U)


#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC11_MASK               				(0x00000800U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC11_SHIFT              				(0x0000000BU)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC11_RESETVAL           				(0x00000000U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC11_MAX                				(0x00000001U)


#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC12_MASK               				(0x00001000U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC12_SHIFT              				(0x0000000CU)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC12_RESETVAL           				(0x00000000U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC12_MAX                				(0x00000001U)


#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC13_MASK               				(0x00002000U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC13_SHIFT              				(0x0000000DU)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC13_RESETVAL           				(0x00000000U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC13_MAX                				(0x00000001U)


#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC14_MASK               				(0x00004000U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC14_SHIFT              				(0x0000000EU)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC14_RESETVAL           				(0x00000000U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC14_MAX                				(0x00000001U)


#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC15_MASK               				(0x00008000U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC15_SHIFT              				(0x0000000FU)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC15_RESETVAL           				(0x00000000U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC15_MAX                				(0x00000001U)


#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC16_MASK               				(0x00010000U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC16_SHIFT              				(0x00000010U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC16_RESETVAL           				(0x00000000U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC16_MAX                				(0x00000001U)


#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC17_MASK               				(0x00020000U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC17_SHIFT              				(0x00000011U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC17_RESETVAL           				(0x00000000U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC17_MAX                				(0x00000001U)


#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC18_MASK               				(0x00040000U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC18_SHIFT              				(0x00000012U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC18_RESETVAL           				(0x00000000U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC18_MAX                				(0x00000001U)


#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC19_MASK               				(0x00080000U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC19_SHIFT              				(0x00000013U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC19_RESETVAL           				(0x00000000U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC19_MAX                				(0x00000001U)

#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC20_MASK              				(0x00100000U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC20_SHIFT             				(0x00000014U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC20_RESETVAL          				(0x00000000U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC20_MAX               				(0x00000001U)


#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC21_MASK               				(0x00200000U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC21_SHIFT              				(0x00000015U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC21_RESETVAL           				(0x00000000U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC21_MAX                				(0x00000001U)


#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC22_MASK               				(0x00400000U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC22_SHIFT              				(0x00000016U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC22_RESETVAL           				(0x00000000U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC22_MAX                				(0x00000001U)


#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC23_MASK               				(0x00800000U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC23_SHIFT              				(0x00000017U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC23_RESETVAL           				(0x00000000U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC23_MAX                				(0x00000001U)


#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC24_MASK               				(0x01000000U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC24_SHIFT              				(0x00000018U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC24_RESETVAL           				(0x00000000U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC24_MAX                				(0x00000001U)


#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC25_MASK               				(0x02000000U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC25_SHIFT              				(0x00000019U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC25_RESETVAL           				(0x00000000U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC25_MAX                				(0x00000001U)


#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC26_MASK               				(0x04000000U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC26_SHIFT              				(0x0000001AU)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC26_RESETVAL           				(0x00000000U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC26_MAX                				(0x00000001U)


#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC27_MASK               				(0x08000000U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC27_SHIFT              				(0x0000001BU)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC27_RESETVAL           				(0x00000000U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC27_MAX                				(0x00000001U)


#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC28_MASK               				(0x10000000U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC28_SHIFT              				(0x0000001CU)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC28_RESETVAL           				(0x00000000U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC28_MAX                				(0x00000001U)


#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC29_MASK               				(0x20000000U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC29_SHIFT              				(0x0000001DU)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC29_RESETVAL           				(0x00000000U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC29_MAX                				(0x00000001U)

#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC30_MASK                 			(0x40000000U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC30_SHIFT                			(0x0000001EU)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC30_RESETVAL             			(0x00000000U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC30_MAX                  			(0x00000001U)


#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC31_MASK               				(0x80000000U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC31_SHIFT              				(0x0000001FU)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC31_RESETVAL           				(0x00000000U)
#define CSL_TOP_RCM_HW_SPARE_REC_HW_SPARE_REC31_MAX                				(0x00000001U)


/* LOCK0_KICK0 */
#define CSL_TOP_RCM_LOCK0_KICK0_LOCK0_KICK0_MASK                                (0xFFFFFFFFU)
#define CSL_TOP_RCM_LOCK0_KICK0_LOCK0_KICK0_SHIFT                               (0x00000000U)
#define CSL_TOP_RCM_LOCK0_KICK0_LOCK0_KICK0_RESETVAL                            (0x00000000U)
#define CSL_TOP_RCM_LOCK0_KICK0_LOCK0_KICK0_MAX                                 (0xFFFFFFFFU)



/* LOCK0_KICK1 */
#define CSL_TOP_RCM_LOCK0_KICK1_LOCK0_KICK1_MASK                                (0xFFFFFFFFU)
#define CSL_TOP_RCM_LOCK0_KICK1_LOCK0_KICK1_SHIFT                               (0x00000000U)
#define CSL_TOP_RCM_LOCK0_KICK1_LOCK0_KICK1_RESETVAL                            (0x00000000U)
#define CSL_TOP_RCM_LOCK0_KICK1_LOCK0_KICK1_MAX                                 (0xFFFFFFFFU)



/* INTR_RAW_STATUS */
#define CSL_TOP_RCM_INTR_RAW_STATUS_ADDR_ERR_MASK                               (0x00000002U)
#define CSL_TOP_RCM_INTR_RAW_STATUS_ADDR_ERR_SHIFT                              (0x00000001U)
#define CSL_TOP_RCM_INTR_RAW_STATUS_ADDR_ERR_RESETVAL                           (0x00000000U)
#define CSL_TOP_RCM_INTR_RAW_STATUS_ADDR_ERR_MAX                                (0x00000001U)


#define CSL_TOP_RCM_INTR_RAW_STATUS_KICK_ERR_MASK                               (0x00000004U)
#define CSL_TOP_RCM_INTR_RAW_STATUS_KICK_ERR_SHIFT                              (0x00000002U)
#define CSL_TOP_RCM_INTR_RAW_STATUS_KICK_ERR_RESETVAL                           (0x00000000U)
#define CSL_TOP_RCM_INTR_RAW_STATUS_KICK_ERR_MAX                                (0x00000001U)


#define CSL_TOP_RCM_INTR_RAW_STATUS_PROT_ERR_MASK                               (0x00000001U)
#define CSL_TOP_RCM_INTR_RAW_STATUS_PROT_ERR_SHIFT                              (0x00000000U)
#define CSL_TOP_RCM_INTR_RAW_STATUS_PROT_ERR_RESETVAL                           (0x00000000U)
#define CSL_TOP_RCM_INTR_RAW_STATUS_PROT_ERR_MAX                                (0x00000001U)


#define CSL_TOP_RCM_INTR_RAW_STATUS_PROXY_ERR_MASK                              (0x00000008U)
#define CSL_TOP_RCM_INTR_RAW_STATUS_PROXY_ERR_SHIFT                             (0x00000003U)
#define CSL_TOP_RCM_INTR_RAW_STATUS_PROXY_ERR_RESETVAL                          (0x00000000U)
#define CSL_TOP_RCM_INTR_RAW_STATUS_PROXY_ERR_MAX                               (0x00000001U)



/* INTR_ENABLED_STATUS_CLEAR */
#define CSL_TOP_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR_MASK             (0x00000002U)
#define CSL_TOP_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR_SHIFT            (0x00000001U)
#define CSL_TOP_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR_RESETVAL         (0x00000000U)
#define CSL_TOP_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR_MAX              (0x00000001U)


#define CSL_TOP_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_KICK_ERR_MASK             (0x00000004U)
#define CSL_TOP_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_KICK_ERR_SHIFT            (0x00000002U)
#define CSL_TOP_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_KICK_ERR_RESETVAL         (0x00000000U)
#define CSL_TOP_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_KICK_ERR_MAX              (0x00000001U)


#define CSL_TOP_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR_MASK             (0x00000001U)
#define CSL_TOP_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR_SHIFT            (0x00000000U)
#define CSL_TOP_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR_RESETVAL         (0x00000000U)
#define CSL_TOP_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR_MAX              (0x00000001U)


#define CSL_TOP_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROXY_ERR_MASK            (0x00000008U)
#define CSL_TOP_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROXY_ERR_SHIFT           (0x00000003U)
#define CSL_TOP_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROXY_ERR_RESETVAL        (0x00000000U)
#define CSL_TOP_RCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROXY_ERR_MAX             (0x00000001U)



/* INTR_ENABLE */
#define CSL_TOP_RCM_INTR_ENABLE_ADDR_ERR_EN_MASK                                (0x00000002U)
#define CSL_TOP_RCM_INTR_ENABLE_ADDR_ERR_EN_SHIFT                               (0x00000001U)
#define CSL_TOP_RCM_INTR_ENABLE_ADDR_ERR_EN_RESETVAL                            (0x00000000U)
#define CSL_TOP_RCM_INTR_ENABLE_ADDR_ERR_EN_MAX                                 (0x00000001U)


#define CSL_TOP_RCM_INTR_ENABLE_KICK_ERR_EN_MASK                                (0x00000004U)
#define CSL_TOP_RCM_INTR_ENABLE_KICK_ERR_EN_SHIFT                               (0x00000002U)
#define CSL_TOP_RCM_INTR_ENABLE_KICK_ERR_EN_RESETVAL                            (0x00000000U)
#define CSL_TOP_RCM_INTR_ENABLE_KICK_ERR_EN_MAX                                 (0x00000001U)


#define CSL_TOP_RCM_INTR_ENABLE_PROT_ERR_EN_MASK                                (0x00000001U)
#define CSL_TOP_RCM_INTR_ENABLE_PROT_ERR_EN_SHIFT                               (0x00000000U)
#define CSL_TOP_RCM_INTR_ENABLE_PROT_ERR_EN_RESETVAL                            (0x00000000U)
#define CSL_TOP_RCM_INTR_ENABLE_PROT_ERR_EN_MAX                                 (0x00000001U)


#define CSL_TOP_RCM_INTR_ENABLE_PROXY_ERR_EN_MASK                               (0x00000008U)
#define CSL_TOP_RCM_INTR_ENABLE_PROXY_ERR_EN_SHIFT                              (0x00000003U)
#define CSL_TOP_RCM_INTR_ENABLE_PROXY_ERR_EN_RESETVAL                           (0x00000000U)
#define CSL_TOP_RCM_INTR_ENABLE_PROXY_ERR_EN_MAX                                (0x00000001U)



/* INTR_ENABLE_CLEAR */
#define CSL_TOP_RCM_INTR_ENABLE_CLEAR_ADDR_ERR_EN_CLR_MASK                      (0x00000002U)
#define CSL_TOP_RCM_INTR_ENABLE_CLEAR_ADDR_ERR_EN_CLR_SHIFT                     (0x00000001U)
#define CSL_TOP_RCM_INTR_ENABLE_CLEAR_ADDR_ERR_EN_CLR_RESETVAL                  (0x00000000U)
#define CSL_TOP_RCM_INTR_ENABLE_CLEAR_ADDR_ERR_EN_CLR_MAX                       (0x00000001U)


#define CSL_TOP_RCM_INTR_ENABLE_CLEAR_KICK_ERR_EN_CLR_MASK                      (0x00000004U)
#define CSL_TOP_RCM_INTR_ENABLE_CLEAR_KICK_ERR_EN_CLR_SHIFT                     (0x00000002U)
#define CSL_TOP_RCM_INTR_ENABLE_CLEAR_KICK_ERR_EN_CLR_RESETVAL                  (0x00000000U)
#define CSL_TOP_RCM_INTR_ENABLE_CLEAR_KICK_ERR_EN_CLR_MAX                       (0x00000001U)


#define CSL_TOP_RCM_INTR_ENABLE_CLEAR_PROT_ERR_EN_CLR_MASK                      (0x00000001U)
#define CSL_TOP_RCM_INTR_ENABLE_CLEAR_PROT_ERR_EN_CLR_SHIFT                     (0x00000000U)
#define CSL_TOP_RCM_INTR_ENABLE_CLEAR_PROT_ERR_EN_CLR_RESETVAL                  (0x00000000U)
#define CSL_TOP_RCM_INTR_ENABLE_CLEAR_PROT_ERR_EN_CLR_MAX                       (0x00000001U)


#define CSL_TOP_RCM_INTR_ENABLE_CLEAR_PROXY_ERR_EN_CLR_MASK                     (0x00000008U)
#define CSL_TOP_RCM_INTR_ENABLE_CLEAR_PROXY_ERR_EN_CLR_SHIFT                    (0x00000003U)
#define CSL_TOP_RCM_INTR_ENABLE_CLEAR_PROXY_ERR_EN_CLR_RESETVAL                 (0x00000000U)
#define CSL_TOP_RCM_INTR_ENABLE_CLEAR_PROXY_ERR_EN_CLR_MAX                      (0x00000001U)



/* EOI */
#define CSL_TOP_RCM_EOI_EOI_VECTOR_MASK                                         (0x000000FFU)
#define CSL_TOP_RCM_EOI_EOI_VECTOR_SHIFT                                        (0x00000000U)
#define CSL_TOP_RCM_EOI_EOI_VECTOR_RESETVAL                                     (0x00000000U)
#define CSL_TOP_RCM_EOI_EOI_VECTOR_MAX                                          (0x000000FFU)



/* FAULT_ADDRESS */
#define CSL_TOP_RCM_FAULT_ADDRESS_FAULT_ADDR_MASK                               (0xFFFFFFFFU)
#define CSL_TOP_RCM_FAULT_ADDRESS_FAULT_ADDR_SHIFT                              (0x00000000U)
#define CSL_TOP_RCM_FAULT_ADDRESS_FAULT_ADDR_RESETVAL                           (0x00000000U)
#define CSL_TOP_RCM_FAULT_ADDRESS_FAULT_ADDR_MAX                                (0xFFFFFFFFU)



/* FAULT_TYPE_STATUS */
#define CSL_TOP_RCM_FAULT_TYPE_STATUS_FAULT_NS_MASK                             (0x00000040U)
#define CSL_TOP_RCM_FAULT_TYPE_STATUS_FAULT_NS_SHIFT                            (0x00000006U)
#define CSL_TOP_RCM_FAULT_TYPE_STATUS_FAULT_NS_RESETVAL                         (0x00000000U)
#define CSL_TOP_RCM_FAULT_TYPE_STATUS_FAULT_NS_MAX                              (0x00000001U)


#define CSL_TOP_RCM_FAULT_TYPE_STATUS_FAULT_TYPE_MASK                           (0x0000003FU)
#define CSL_TOP_RCM_FAULT_TYPE_STATUS_FAULT_TYPE_SHIFT                          (0x00000000U)
#define CSL_TOP_RCM_FAULT_TYPE_STATUS_FAULT_TYPE_RESETVAL                       (0x00000000U)
#define CSL_TOP_RCM_FAULT_TYPE_STATUS_FAULT_TYPE_MAX                            (0x0000003FU)



/* FAULT_ATTR_STATUS */
#define CSL_TOP_RCM_FAULT_ATTR_STATUS_FAULT_PRIVID_MASK                         (0x000000FFU)
#define CSL_TOP_RCM_FAULT_ATTR_STATUS_FAULT_PRIVID_SHIFT                        (0x00000000U)
#define CSL_TOP_RCM_FAULT_ATTR_STATUS_FAULT_PRIVID_RESETVAL                     (0x00000000U)
#define CSL_TOP_RCM_FAULT_ATTR_STATUS_FAULT_PRIVID_MAX                          (0x000000FFU)


#define CSL_TOP_RCM_FAULT_ATTR_STATUS_FAULT_ROUTEID_MASK                        (0x000FFF00U)
#define CSL_TOP_RCM_FAULT_ATTR_STATUS_FAULT_ROUTEID_SHIFT                       (0x00000008U)
#define CSL_TOP_RCM_FAULT_ATTR_STATUS_FAULT_ROUTEID_RESETVAL                    (0x00000000U)
#define CSL_TOP_RCM_FAULT_ATTR_STATUS_FAULT_ROUTEID_MAX                         (0x00000FFFU)


#define CSL_TOP_RCM_FAULT_ATTR_STATUS_FAULT_XID_MASK                            (0xFFF00000U)
#define CSL_TOP_RCM_FAULT_ATTR_STATUS_FAULT_XID_SHIFT                           (0x00000014U)
#define CSL_TOP_RCM_FAULT_ATTR_STATUS_FAULT_XID_RESETVAL                        (0x00000000U)
#define CSL_TOP_RCM_FAULT_ATTR_STATUS_FAULT_XID_MAX                             (0x00000FFFU)



/* FAULT_CLEAR */
#define CSL_TOP_RCM_FAULT_CLEAR_FAULT_CLR_MASK                                  (0x00000001U)
#define CSL_TOP_RCM_FAULT_CLEAR_FAULT_CLR_SHIFT                                 (0x00000000U)
#define CSL_TOP_RCM_FAULT_CLEAR_FAULT_CLR_RESETVAL                              (0x00000000U)
#define CSL_TOP_RCM_FAULT_CLEAR_FAULT_CLR_MAX                                   (0x00000001U)



#ifdef __cplusplus
}
#endif
#endif