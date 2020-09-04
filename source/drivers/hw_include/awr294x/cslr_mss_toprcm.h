/********************************************************************
 * Copyright (C) 2021 Texas Instruments Incorporated.
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
 *  Name        : cslr_mss_toprcm.h
*/
#ifndef CSLR_MSS_TOPRCM_H_
#define CSLR_MSS_TOPRCM_H_

#include <drivers/hw_include/cslr.h>
#include <drivers/hw_include/tistdtypes.h>

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
    volatile uint32_t HSI_CLK_SRC_SEL;
    volatile uint32_t CSIRX_CLK_SRC_SEL;
    volatile uint32_t MCUCLKOUT_CLK_SRC_SEL;
    volatile uint32_t PMICCLKOUT_CLK_SRC_SEL;
    volatile uint32_t OBSCLKOUT_CLK_SRC_SEL;
    volatile uint32_t TRCCLKOUT_CLK_SRC_SEL;
    volatile uint8_t  Resv_68[20];
    volatile uint32_t HSI_DIV_VAL;
    volatile uint32_t CSIRX_DIV_VAL;
    volatile uint32_t MCUCLKOUT_DIV_VAL;
    volatile uint32_t PMICCLKOUT_DIV_VAL;
    volatile uint32_t OBSCLKOUT_DIV_VAL;
    volatile uint32_t TRCCLKOUT_DIV_VAL;
    volatile uint8_t  Resv_132[40];
    volatile uint32_t HSI_CLK_GATE;
    volatile uint32_t CSIRX_CLK_GATE;
    volatile uint32_t MCUCLKOUT_CLK_GATE;
    volatile uint32_t PMICCLKOUT_CLK_GATE;
    volatile uint32_t OBSCLKOUT_CLK_GATE;
    volatile uint32_t TRCCLKOUT_CLK_GATE;
    volatile uint32_t DSS_CLK_GATE;
    volatile uint8_t  Resv_192[36];
    volatile uint32_t HSI_CLK_STATUS;
    volatile uint32_t CSIRX_CLK_STATUS;
    volatile uint32_t MCUCLKOUT_CLK_STATUS;
    volatile uint32_t PMICCLKOUT_CLK_STATUS;
    volatile uint32_t OBSCLKOUT_CLK_STATUS;
    volatile uint32_t TRCCLKOUT_CLK_STATUS;
    volatile uint8_t  Resv_256[40];
    volatile uint32_t WARM_RESET_CONFIG;
    volatile uint32_t SYS_RST_CAUSE;
    volatile uint32_t SYS_RST_CAUSE_CLR;
    volatile uint32_t DSS_RST_CTRL;
    volatile uint8_t  Resv_516[244];
    volatile uint32_t RS232_BITINTERVAL;
    volatile uint32_t LVDS_PAD_CTRL0;
    volatile uint32_t LVDS_PAD_CTRL1;
    volatile uint32_t DFT_DMLED_EXEC;
    volatile uint32_t DFT_DMLED_STATUS;
    volatile uint32_t LIMP_MODE_EN;
    volatile uint32_t PMICCLKOUT_DCDC_CTRL;
    volatile uint32_t PMICCLKOUT_DCDC_SLOPE;
    volatile uint32_t RCOSC32K_CTRL;
    volatile uint32_t ANA_HSI2DIGCLK_GATE;
    volatile uint8_t  Resv_1024[468];
    volatile uint32_t PLL_CORE_PWRCTRL;
    volatile uint32_t PLL_CORE_CLKCTRL;
    volatile uint32_t PLL_CORE_TENABLE;
    volatile uint32_t PLL_CORE_TENABLEDIV;
    volatile uint32_t PLL_CORE_M2NDIV;
    volatile uint32_t PLL_CORE_MN2DIV;
    volatile uint32_t PLL_CORE_FRACDIV;
    volatile uint32_t PLL_CORE_BWCTRL;
    volatile uint32_t PLL_CORE_FRACCTRL;
    volatile uint32_t PLL_CORE_STATUS;
    volatile uint32_t PLL_CORE_HSDIVIDER;
    volatile uint32_t PLL_CORE_HSDIVIDER_CLKOUT0;
    volatile uint32_t PLL_CORE_HSDIVIDER_CLKOUT1;
    volatile uint32_t PLL_CORE_HSDIVIDER_CLKOUT2;
    volatile uint32_t PLL_CORE_HSDIVIDER_CLKOUT3;
    volatile uint32_t MSS_CR5_CLK_SRC_SEL;
    volatile uint32_t MSS_CR5_DIV_VAL;
    volatile uint32_t SYS_CLK_DIV_VAL;
    volatile uint32_t MSS_CR5_CLK_GATE;
    volatile uint32_t SYS_CLK_GATE;
    volatile uint32_t SYS_CLK_STATUS;
    volatile uint32_t MSS_CR5_CLK_STATUS;
    volatile uint32_t PLL_CORE_RSTCTRL;
    volatile uint32_t PLL_CORE_HSDIVIDER_RSTCTRL;
    volatile uint32_t RSS_CLK_SRC_SEL;
    volatile uint32_t PLLC_CLK2_SRC_SEL;
    volatile uint32_t PLLD_CLK1_SRC_SEL;
    volatile uint32_t PLLD_CLK2_SRC_SEL;
    volatile uint32_t PLLP_CLK1_SRC_SEL;
    volatile uint32_t RSS_DIV_VAL;
    volatile uint32_t RSS_CLK_GATE;
    volatile uint32_t PLLC_CLK2_GATE;
    volatile uint32_t PLLD_CLK1_GATE;
    volatile uint32_t PLLD_CLK2_GATE;
    volatile uint32_t PLLP_CLK1_GATE;
    volatile uint32_t RSS_CLK_STATUS;
    volatile uint32_t PLLC_CLK2_STATUS;
    volatile uint32_t PLLD_CLK1_STATUS;
    volatile uint32_t PLLD_CLK2_STATUS;
    volatile uint32_t PLLP_CLK1_STATUS;
    volatile uint32_t PLL_1P2_HSDIVIDER;
    volatile uint32_t PLL_1P2_HSDIVIDER_CLKOUT0;
    volatile uint32_t PLL_1P2_HSDIVIDER_CLKOUT1;
    volatile uint32_t PLL_1P2_HSDIVIDER_CLKOUT2;
    volatile uint32_t PLL_1P2_HSDIVIDER_CLKOUT3;
    volatile uint32_t PLL_1P2_HSDIVIDER_RSTCTRL;
    volatile uint32_t PLL_1P8_HSDIVIDER;
    volatile uint32_t PLL_1P8_HSDIVIDER_CLKOUT0;
    volatile uint32_t PLL_1P8_HSDIVIDER_CLKOUT1;
    volatile uint32_t PLL_1P8_HSDIVIDER_CLKOUT2;
    volatile uint32_t PLL_1P8_HSDIVIDER_CLKOUT3;
    volatile uint32_t PLL_1P8_HSDIVIDER_RSTCTRL;
    volatile uint8_t  Resv_2048[816];
    volatile uint32_t PLL_DSP_PWRCTRL;
    volatile uint32_t PLL_DSP_CLKCTRL;
    volatile uint32_t PLL_DSP_TENABLE;
    volatile uint32_t PLL_DSP_TENABLEDIV;
    volatile uint32_t PLL_DSP_M2NDIV;
    volatile uint32_t PLL_DSP_MN2DIV;
    volatile uint32_t PLL_DSP_FRACDIV;
    volatile uint32_t PLL_DSP_BWCTRL;
    volatile uint32_t PLL_DSP_FRACCTRL;
    volatile uint32_t PLL_DSP_STATUS;
    volatile uint32_t PLL_DSP_HSDIVIDER;
    volatile uint32_t PLL_DSP_HSDIVIDER_CLKOUT0;
    volatile uint32_t PLL_DSP_HSDIVIDER_CLKOUT1;
    volatile uint32_t PLL_DSP_HSDIVIDER_CLKOUT2;
    volatile uint32_t PLL_DSP_HSDIVIDER_CLKOUT3;
    volatile uint32_t PLL_PER_PWRCTRL;
    volatile uint32_t PLL_PER_CLKCTRL;
    volatile uint32_t PLL_PER_TENABLE;
    volatile uint32_t PLL_PER_TENABLEDIV;
    volatile uint32_t PLL_PER_M2NDIV;
    volatile uint32_t PLL_PER_MN2DIV;
    volatile uint32_t PLL_PER_FRACDIV;
    volatile uint32_t PLL_PER_BWCTRL;
    volatile uint32_t PLL_PER_FRACCTRL;
    volatile uint32_t PLL_PER_STATUS;
    volatile uint32_t PLL_PER_HSDIVIDER;
    volatile uint32_t PLL_PER_HSDIVIDER_CLKOUT0;
    volatile uint32_t PLL_PER_HSDIVIDER_CLKOUT1;
    volatile uint32_t PLL_PER_HSDIVIDER_CLKOUT2;
    volatile uint32_t PLL_PER_HSDIVIDER_CLKOUT3;
    volatile uint32_t PLL_DSP_RSTCTRL;
    volatile uint32_t PLL_DSP_HSDIVIDER_RSTCTRL;
    volatile uint32_t PLL_PER_RSTCTRL;
    volatile uint32_t PLL_PER_HSDIVIDER_RSTCTRL;
    volatile uint8_t  Resv_3072[888];
    volatile uint32_t ANA_REG_CLK_CTRL_REG1_XO_SLICER;
    volatile uint32_t ANA_REG_CLK_CTRL_REG1_CLKTOP;
    volatile uint32_t ANA_REG_CLK_CTRL_REG2_CLKTOP;
    volatile uint32_t ANA_REG_CLK_CTRL_REG1_LDO_CLKTOP;
    volatile uint32_t ANA_REG_CLK_CTRL_REG2_LDO_CLKTOP;
    volatile uint8_t  Resv_3096[4];
    volatile uint32_t ANA_REG_CLK_STATUS_REG;
    volatile uint32_t ANA_REG_REFSYS_CTRL_REG_LOWV;
    volatile uint32_t ANA_REG_REFSYS_TMUX_CTRL_LOWV;
    volatile uint32_t ANA_REG_REFSYS_SPARE_REG_LOWV;
    volatile uint32_t ANA_REG_WU_CTRL_REG_LOWV;
    volatile uint32_t ANA_REG_WU_TMUX_CTRL_LOWV;
    volatile uint32_t ANA_REG_TW_CTRL_REG_LOWV;
    volatile uint32_t ANA_REG_TW_ANA_TMUX_CTRL_LOWV;
    volatile uint8_t  Resv_3132[4];
    volatile uint32_t ANA_REG_WU_MODE_REG_LOWV;
    volatile uint32_t ANA_REG_WU_STATUS_REG_LOWV;
    volatile uint32_t ANA_REG_WU_SPARE_OUT_LOWV;
    volatile uint8_t  Resv_4048[904];
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
} CSL_mss_toprcmRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_MSS_TOPRCM_PID                                                     (0x00000000U)
#define CSL_MSS_TOPRCM_HW_REG0                                                 (0x00000004U)
#define CSL_MSS_TOPRCM_HW_REG1                                                 (0x00000008U)
#define CSL_MSS_TOPRCM_HW_REG2                                                 (0x0000000CU)
#define CSL_MSS_TOPRCM_HW_REG3                                                 (0x00000010U)
#define CSL_MSS_TOPRCM_HSI_CLK_SRC_SEL                                         (0x00000014U)
#define CSL_MSS_TOPRCM_CSIRX_CLK_SRC_SEL                                       (0x00000018U)
#define CSL_MSS_TOPRCM_MCUCLKOUT_CLK_SRC_SEL                                   (0x0000001CU)
#define CSL_MSS_TOPRCM_PMICCLKOUT_CLK_SRC_SEL                                  (0x00000020U)
#define CSL_MSS_TOPRCM_OBSCLKOUT_CLK_SRC_SEL                                   (0x00000024U)
#define CSL_MSS_TOPRCM_TRCCLKOUT_CLK_SRC_SEL                                   (0x00000028U)
#define CSL_MSS_TOPRCM_HSI_DIV_VAL                                             (0x00000040U)
#define CSL_MSS_TOPRCM_CSIRX_DIV_VAL                                           (0x00000044U)
#define CSL_MSS_TOPRCM_MCUCLKOUT_DIV_VAL                                       (0x00000048U)
#define CSL_MSS_TOPRCM_PMICCLKOUT_DIV_VAL                                      (0x0000004CU)
#define CSL_MSS_TOPRCM_OBSCLKOUT_DIV_VAL                                       (0x00000050U)
#define CSL_MSS_TOPRCM_TRCCLKOUT_DIV_VAL                                       (0x00000054U)
#define CSL_MSS_TOPRCM_HSI_CLK_GATE                                            (0x00000080U)
#define CSL_MSS_TOPRCM_CSIRX_CLK_GATE                                          (0x00000084U)
#define CSL_MSS_TOPRCM_MCUCLKOUT_CLK_GATE                                      (0x00000088U)
#define CSL_MSS_TOPRCM_PMICCLKOUT_CLK_GATE                                     (0x0000008CU)
#define CSL_MSS_TOPRCM_OBSCLKOUT_CLK_GATE                                      (0x00000090U)
#define CSL_MSS_TOPRCM_TRCCLKOUT_CLK_GATE                                      (0x00000094U)
#define CSL_MSS_TOPRCM_DSS_CLK_GATE                                            (0x00000098U)
#define CSL_MSS_TOPRCM_HSI_CLK_STATUS                                          (0x000000C0U)
#define CSL_MSS_TOPRCM_CSIRX_CLK_STATUS                                        (0x000000C4U)
#define CSL_MSS_TOPRCM_MCUCLKOUT_CLK_STATUS                                    (0x000000C8U)
#define CSL_MSS_TOPRCM_PMICCLKOUT_CLK_STATUS                                   (0x000000CCU)
#define CSL_MSS_TOPRCM_OBSCLKOUT_CLK_STATUS                                    (0x000000D0U)
#define CSL_MSS_TOPRCM_TRCCLKOUT_CLK_STATUS                                    (0x000000D4U)
#define CSL_MSS_TOPRCM_WARM_RESET_CONFIG                                       (0x00000100U)
#define CSL_MSS_TOPRCM_SYS_RST_CAUSE                                           (0x00000104U)
#define CSL_MSS_TOPRCM_SYS_RST_CAUSE_CLR                                       (0x00000108U)
#define CSL_MSS_TOPRCM_DSS_RST_CTRL                                            (0x0000010CU)
#define CSL_MSS_TOPRCM_RS232_BITINTERVAL                                       (0x00000204U)
#define CSL_MSS_TOPRCM_LVDS_PAD_CTRL0                                          (0x00000208U)
#define CSL_MSS_TOPRCM_LVDS_PAD_CTRL1                                          (0x0000020CU)
#define CSL_MSS_TOPRCM_DFT_DMLED_EXEC                                          (0x00000210U)
#define CSL_MSS_TOPRCM_DFT_DMLED_STATUS                                        (0x00000214U)
#define CSL_MSS_TOPRCM_LIMP_MODE_EN                                            (0x00000218U)
#define CSL_MSS_TOPRCM_PMICCLKOUT_DCDC_CTRL                                    (0x0000021CU)
#define CSL_MSS_TOPRCM_PMICCLKOUT_DCDC_SLOPE                                   (0x00000220U)
#define CSL_MSS_TOPRCM_RCOSC32K_CTRL                                           (0x00000224U)
#define CSL_MSS_TOPRCM_ANA_HSI2DIGCLK_GATE                                     (0x00000228U)
#define CSL_MSS_TOPRCM_PLL_CORE_PWRCTRL                                        (0x00000400U)
#define CSL_MSS_TOPRCM_PLL_CORE_CLKCTRL                                        (0x00000404U)
#define CSL_MSS_TOPRCM_PLL_CORE_TENABLE                                        (0x00000408U)
#define CSL_MSS_TOPRCM_PLL_CORE_TENABLEDIV                                     (0x0000040CU)
#define CSL_MSS_TOPRCM_PLL_CORE_M2NDIV                                         (0x00000410U)
#define CSL_MSS_TOPRCM_PLL_CORE_MN2DIV                                         (0x00000414U)
#define CSL_MSS_TOPRCM_PLL_CORE_FRACDIV                                        (0x00000418U)
#define CSL_MSS_TOPRCM_PLL_CORE_BWCTRL                                         (0x0000041CU)
#define CSL_MSS_TOPRCM_PLL_CORE_FRACCTRL                                       (0x00000420U)
#define CSL_MSS_TOPRCM_PLL_CORE_STATUS                                         (0x00000424U)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER                                      (0x00000428U)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT0                              (0x0000042CU)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT1                              (0x00000430U)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT2                              (0x00000434U)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT3                              (0x00000438U)
#define CSL_MSS_TOPRCM_MSS_CR5_CLK_SRC_SEL                                     (0x0000043CU)
#define CSL_MSS_TOPRCM_MSS_CR5_DIV_VAL                                         (0x00000440U)
#define CSL_MSS_TOPRCM_SYS_CLK_DIV_VAL                                         (0x00000444U)
#define CSL_MSS_TOPRCM_MSS_CR5_CLK_GATE                                        (0x00000448U)
#define CSL_MSS_TOPRCM_SYS_CLK_GATE                                            (0x0000044CU)
#define CSL_MSS_TOPRCM_SYS_CLK_STATUS                                          (0x00000450U)
#define CSL_MSS_TOPRCM_MSS_CR5_CLK_STATUS                                      (0x00000454U)
#define CSL_MSS_TOPRCM_PLL_CORE_RSTCTRL                                        (0x00000458U)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_RSTCTRL                              (0x0000045CU)
#define CSL_MSS_TOPRCM_RSS_CLK_SRC_SEL                                         (0x00000460U)
#define CSL_MSS_TOPRCM_PLLC_CLK2_SRC_SEL                                       (0x00000464U)
#define CSL_MSS_TOPRCM_PLLD_CLK1_SRC_SEL                                       (0x00000468U)
#define CSL_MSS_TOPRCM_PLLD_CLK2_SRC_SEL                                       (0x0000046CU)
#define CSL_MSS_TOPRCM_PLLP_CLK1_SRC_SEL                                       (0x00000470U)
#define CSL_MSS_TOPRCM_RSS_DIV_VAL                                             (0x00000474U)
#define CSL_MSS_TOPRCM_RSS_CLK_GATE                                            (0x00000478U)
#define CSL_MSS_TOPRCM_PLLC_CLK2_GATE                                          (0x0000047CU)
#define CSL_MSS_TOPRCM_PLLD_CLK1_GATE                                          (0x00000480U)
#define CSL_MSS_TOPRCM_PLLD_CLK2_GATE                                          (0x00000484U)
#define CSL_MSS_TOPRCM_PLLP_CLK1_GATE                                          (0x00000488U)
#define CSL_MSS_TOPRCM_RSS_CLK_STATUS                                          (0x0000048CU)
#define CSL_MSS_TOPRCM_PLLC_CLK2_STATUS                                        (0x00000490U)
#define CSL_MSS_TOPRCM_PLLD_CLK1_STATUS                                        (0x00000494U)
#define CSL_MSS_TOPRCM_PLLD_CLK2_STATUS                                        (0x00000498U)
#define CSL_MSS_TOPRCM_PLLP_CLK1_STATUS                                        (0x0000049CU)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER                                       (0x000004A0U)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT0                               (0x000004A4U)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT1                               (0x000004A8U)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT2                               (0x000004ACU)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT3                               (0x000004B0U)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_RSTCTRL                               (0x000004B4U)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER                                       (0x000004B8U)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT0                               (0x000004BCU)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT1                               (0x000004C0U)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT2                               (0x000004C4U)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT3                               (0x000004C8U)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_RSTCTRL                               (0x000004CCU)
#define CSL_MSS_TOPRCM_PLL_DSP_PWRCTRL                                         (0x00000800U)
#define CSL_MSS_TOPRCM_PLL_DSP_CLKCTRL                                         (0x00000804U)
#define CSL_MSS_TOPRCM_PLL_DSP_TENABLE                                         (0x00000808U)
#define CSL_MSS_TOPRCM_PLL_DSP_TENABLEDIV                                      (0x0000080CU)
#define CSL_MSS_TOPRCM_PLL_DSP_M2NDIV                                          (0x00000810U)
#define CSL_MSS_TOPRCM_PLL_DSP_MN2DIV                                          (0x00000814U)
#define CSL_MSS_TOPRCM_PLL_DSP_FRACDIV                                         (0x00000818U)
#define CSL_MSS_TOPRCM_PLL_DSP_BWCTRL                                          (0x0000081CU)
#define CSL_MSS_TOPRCM_PLL_DSP_FRACCTRL                                        (0x00000820U)
#define CSL_MSS_TOPRCM_PLL_DSP_STATUS                                          (0x00000824U)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER                                       (0x00000828U)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT0                               (0x0000082CU)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT1                               (0x00000830U)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT2                               (0x00000834U)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT3                               (0x00000838U)
#define CSL_MSS_TOPRCM_PLL_PER_PWRCTRL                                         (0x0000083CU)
#define CSL_MSS_TOPRCM_PLL_PER_CLKCTRL                                         (0x00000840U)
#define CSL_MSS_TOPRCM_PLL_PER_TENABLE                                         (0x00000844U)
#define CSL_MSS_TOPRCM_PLL_PER_TENABLEDIV                                      (0x00000848U)
#define CSL_MSS_TOPRCM_PLL_PER_M2NDIV                                          (0x0000084CU)
#define CSL_MSS_TOPRCM_PLL_PER_MN2DIV                                          (0x00000850U)
#define CSL_MSS_TOPRCM_PLL_PER_FRACDIV                                         (0x00000854U)
#define CSL_MSS_TOPRCM_PLL_PER_BWCTRL                                          (0x00000858U)
#define CSL_MSS_TOPRCM_PLL_PER_FRACCTRL                                        (0x0000085CU)
#define CSL_MSS_TOPRCM_PLL_PER_STATUS                                          (0x00000860U)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER                                       (0x00000864U)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT0                               (0x00000868U)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT1                               (0x0000086CU)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT2                               (0x00000870U)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT3                               (0x00000874U)
#define CSL_MSS_TOPRCM_PLL_DSP_RSTCTRL                                         (0x00000878U)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_RSTCTRL                               (0x0000087CU)
#define CSL_MSS_TOPRCM_PLL_PER_RSTCTRL                                         (0x00000880U)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_RSTCTRL                               (0x00000884U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG1_XO_SLICER                         (0x00000C00U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG1_CLKTOP                            (0x00000C04U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG2_CLKTOP                            (0x00000C08U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG1_LDO_CLKTOP                        (0x00000C0CU)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG2_LDO_CLKTOP                        (0x00000C10U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_STATUS_REG                                  (0x00000C18U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_CTRL_REG_LOWV                            (0x00000C1CU)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_TMUX_CTRL_LOWV                           (0x00000C20U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV                           (0x00000C24U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV                                (0x00000C28U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV                               (0x00000C2CU)
#define CSL_MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV                                (0x00000C30U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV                           (0x00000C34U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_MODE_REG_LOWV                                (0x00000C3CU)
#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV                              (0x00000C40U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_SPARE_OUT_LOWV                               (0x00000C44U)
#define CSL_MSS_TOPRCM_HW_SPARE_RW0                                            (0x00000FD0U)
#define CSL_MSS_TOPRCM_HW_SPARE_RW1                                            (0x00000FD4U)
#define CSL_MSS_TOPRCM_HW_SPARE_RW2                                            (0x00000FD8U)
#define CSL_MSS_TOPRCM_HW_SPARE_RW3                                            (0x00000FDCU)
#define CSL_MSS_TOPRCM_HW_SPARE_RO0                                            (0x00000FE0U)
#define CSL_MSS_TOPRCM_HW_SPARE_RO1                                            (0x00000FE4U)
#define CSL_MSS_TOPRCM_HW_SPARE_RO2                                            (0x00000FE8U)
#define CSL_MSS_TOPRCM_HW_SPARE_RO3                                            (0x00000FECU)
#define CSL_MSS_TOPRCM_HW_SPARE_WPH                                            (0x00000FF0U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC                                            (0x00000FF4U)
#define CSL_MSS_TOPRCM_LOCK0_KICK0                                             (0x00001008U)
#define CSL_MSS_TOPRCM_LOCK0_KICK1                                             (0x0000100CU)
#define CSL_MSS_TOPRCM_INTR_RAW_STATUS                                         (0x00001010U)
#define CSL_MSS_TOPRCM_INTR_ENABLED_STATUS_CLEAR                               (0x00001014U)
#define CSL_MSS_TOPRCM_INTR_ENABLE                                             (0x00001018U)
#define CSL_MSS_TOPRCM_INTR_ENABLE_CLEAR                                       (0x0000101CU)
#define CSL_MSS_TOPRCM_EOI                                                     (0x00001020U)
#define CSL_MSS_TOPRCM_FAULT_ADDRESS                                           (0x00001024U)
#define CSL_MSS_TOPRCM_FAULT_TYPE_STATUS                                       (0x00001028U)
#define CSL_MSS_TOPRCM_FAULT_ATTR_STATUS                                       (0x0000102CU)
#define CSL_MSS_TOPRCM_FAULT_CLEAR                                             (0x00001030U)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* PID */

#define CSL_MSS_TOPRCM_PID_PID_MINOR_MASK                                      (0x0000003FU)
#define CSL_MSS_TOPRCM_PID_PID_MINOR_SHIFT                                     (0x00000000U)
#define CSL_MSS_TOPRCM_PID_PID_MINOR_RESETVAL                                  (0x00000014U)
#define CSL_MSS_TOPRCM_PID_PID_MINOR_MAX                                       (0x0000003FU)

#define CSL_MSS_TOPRCM_PID_PID_CUSTOM_MASK                                     (0x000000C0U)
#define CSL_MSS_TOPRCM_PID_PID_CUSTOM_SHIFT                                    (0x00000006U)
#define CSL_MSS_TOPRCM_PID_PID_CUSTOM_RESETVAL                                 (0x00000000U)
#define CSL_MSS_TOPRCM_PID_PID_CUSTOM_MAX                                      (0x00000003U)

#define CSL_MSS_TOPRCM_PID_PID_MAJOR_MASK                                      (0x00000700U)
#define CSL_MSS_TOPRCM_PID_PID_MAJOR_SHIFT                                     (0x00000008U)
#define CSL_MSS_TOPRCM_PID_PID_MAJOR_RESETVAL                                  (0x00000002U)
#define CSL_MSS_TOPRCM_PID_PID_MAJOR_MAX                                       (0x00000007U)

#define CSL_MSS_TOPRCM_PID_PID_MISC_MASK                                       (0x0000F800U)
#define CSL_MSS_TOPRCM_PID_PID_MISC_SHIFT                                      (0x0000000BU)
#define CSL_MSS_TOPRCM_PID_PID_MISC_RESETVAL                                   (0x00000000U)
#define CSL_MSS_TOPRCM_PID_PID_MISC_MAX                                        (0x0000001FU)

#define CSL_MSS_TOPRCM_PID_PID_MSB16_MASK                                      (0xFFFF0000U)
#define CSL_MSS_TOPRCM_PID_PID_MSB16_SHIFT                                     (0x00000010U)
#define CSL_MSS_TOPRCM_PID_PID_MSB16_RESETVAL                                  (0x00006180U)
#define CSL_MSS_TOPRCM_PID_PID_MSB16_MAX                                       (0x0000FFFFU)

#define CSL_MSS_TOPRCM_PID_RESETVAL                                            (0x61800214U)

/* HW_REG0 */

#define CSL_MSS_TOPRCM_HW_REG0_HW_REG0_HWREG_MASK                              (0xFFFFFFFFU)
#define CSL_MSS_TOPRCM_HW_REG0_HW_REG0_HWREG_SHIFT                             (0x00000000U)
#define CSL_MSS_TOPRCM_HW_REG0_HW_REG0_HWREG_RESETVAL                          (0x00000000U)
#define CSL_MSS_TOPRCM_HW_REG0_HW_REG0_HWREG_MAX                               (0xFFFFFFFFU)

#define CSL_MSS_TOPRCM_HW_REG0_RESETVAL                                        (0x00000000U)

/* HW_REG1 */

#define CSL_MSS_TOPRCM_HW_REG1_HW_REG1_HWREG_MASK                              (0xFFFFFFFFU)
#define CSL_MSS_TOPRCM_HW_REG1_HW_REG1_HWREG_SHIFT                             (0x00000000U)
#define CSL_MSS_TOPRCM_HW_REG1_HW_REG1_HWREG_RESETVAL                          (0x00000000U)
#define CSL_MSS_TOPRCM_HW_REG1_HW_REG1_HWREG_MAX                               (0xFFFFFFFFU)

#define CSL_MSS_TOPRCM_HW_REG1_RESETVAL                                        (0x00000000U)

/* HW_REG2 */

#define CSL_MSS_TOPRCM_HW_REG2_HW_REG2_HWREG_MASK                              (0xFFFFFFFFU)
#define CSL_MSS_TOPRCM_HW_REG2_HW_REG2_HWREG_SHIFT                             (0x00000000U)
#define CSL_MSS_TOPRCM_HW_REG2_HW_REG2_HWREG_RESETVAL                          (0x00000000U)
#define CSL_MSS_TOPRCM_HW_REG2_HW_REG2_HWREG_MAX                               (0xFFFFFFFFU)

#define CSL_MSS_TOPRCM_HW_REG2_RESETVAL                                        (0x00000000U)

/* HW_REG3 */

#define CSL_MSS_TOPRCM_HW_REG3_HW_REG3_HWREG_MASK                              (0xFFFFFFFFU)
#define CSL_MSS_TOPRCM_HW_REG3_HW_REG3_HWREG_SHIFT                             (0x00000000U)
#define CSL_MSS_TOPRCM_HW_REG3_HW_REG3_HWREG_RESETVAL                          (0x00000000U)
#define CSL_MSS_TOPRCM_HW_REG3_HW_REG3_HWREG_MAX                               (0xFFFFFFFFU)

#define CSL_MSS_TOPRCM_HW_REG3_RESETVAL                                        (0x00000000U)

/* HSI_CLK_SRC_SEL */

#define CSL_MSS_TOPRCM_HSI_CLK_SRC_SEL_HSI_CLK_SRC_SEL_CLKSRCSEL_MASK          (0x00000FFFU)
#define CSL_MSS_TOPRCM_HSI_CLK_SRC_SEL_HSI_CLK_SRC_SEL_CLKSRCSEL_SHIFT         (0x00000000U)
#define CSL_MSS_TOPRCM_HSI_CLK_SRC_SEL_HSI_CLK_SRC_SEL_CLKSRCSEL_RESETVAL      (0x00000555U)
#define CSL_MSS_TOPRCM_HSI_CLK_SRC_SEL_HSI_CLK_SRC_SEL_CLKSRCSEL_MAX           (0x00000FFFU)

#define CSL_MSS_TOPRCM_HSI_CLK_SRC_SEL_RESETVAL                                (0x00000555U)

/* CSIRX_CLK_SRC_SEL */

#define CSL_MSS_TOPRCM_CSIRX_CLK_SRC_SEL_CSIRX_CLK_SRC_SEL_CLKSRCSEL_MASK      (0x00000FFFU)
#define CSL_MSS_TOPRCM_CSIRX_CLK_SRC_SEL_CSIRX_CLK_SRC_SEL_CLKSRCSEL_SHIFT     (0x00000000U)
#define CSL_MSS_TOPRCM_CSIRX_CLK_SRC_SEL_CSIRX_CLK_SRC_SEL_CLKSRCSEL_RESETVAL  (0x00000000U)
#define CSL_MSS_TOPRCM_CSIRX_CLK_SRC_SEL_CSIRX_CLK_SRC_SEL_CLKSRCSEL_MAX       (0x00000FFFU)

#define CSL_MSS_TOPRCM_CSIRX_CLK_SRC_SEL_RESETVAL                              (0x00000000U)

/* MCUCLKOUT_CLK_SRC_SEL */

#define CSL_MSS_TOPRCM_MCUCLKOUT_CLK_SRC_SEL_MCUCLKOUT_CLK_SRC_SEL_CLKSRCSEL_MASK (0x00000FFFU)
#define CSL_MSS_TOPRCM_MCUCLKOUT_CLK_SRC_SEL_MCUCLKOUT_CLK_SRC_SEL_CLKSRCSEL_SHIFT (0x00000000U)
#define CSL_MSS_TOPRCM_MCUCLKOUT_CLK_SRC_SEL_MCUCLKOUT_CLK_SRC_SEL_CLKSRCSEL_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_MCUCLKOUT_CLK_SRC_SEL_MCUCLKOUT_CLK_SRC_SEL_CLKSRCSEL_MAX (0x00000FFFU)

#define CSL_MSS_TOPRCM_MCUCLKOUT_CLK_SRC_SEL_RESETVAL                          (0x00000000U)

/* PMICCLKOUT_CLK_SRC_SEL */

#define CSL_MSS_TOPRCM_PMICCLKOUT_CLK_SRC_SEL_PMICCLKOUT_CLK_SRC_SEL_CLKSRCSEL_MASK (0x00000FFFU)
#define CSL_MSS_TOPRCM_PMICCLKOUT_CLK_SRC_SEL_PMICCLKOUT_CLK_SRC_SEL_CLKSRCSEL_SHIFT (0x00000000U)
#define CSL_MSS_TOPRCM_PMICCLKOUT_CLK_SRC_SEL_PMICCLKOUT_CLK_SRC_SEL_CLKSRCSEL_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PMICCLKOUT_CLK_SRC_SEL_PMICCLKOUT_CLK_SRC_SEL_CLKSRCSEL_MAX (0x00000FFFU)

#define CSL_MSS_TOPRCM_PMICCLKOUT_CLK_SRC_SEL_RESETVAL                         (0x00000000U)

/* OBSCLKOUT_CLK_SRC_SEL */

#define CSL_MSS_TOPRCM_OBSCLKOUT_CLK_SRC_SEL_OBSCLKOUT_CLK_SRC_SEL_CLKSRCSEL_MASK (0x00000FFFU)
#define CSL_MSS_TOPRCM_OBSCLKOUT_CLK_SRC_SEL_OBSCLKOUT_CLK_SRC_SEL_CLKSRCSEL_SHIFT (0x00000000U)
#define CSL_MSS_TOPRCM_OBSCLKOUT_CLK_SRC_SEL_OBSCLKOUT_CLK_SRC_SEL_CLKSRCSEL_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_OBSCLKOUT_CLK_SRC_SEL_OBSCLKOUT_CLK_SRC_SEL_CLKSRCSEL_MAX (0x00000FFFU)

#define CSL_MSS_TOPRCM_OBSCLKOUT_CLK_SRC_SEL_RESETVAL                          (0x00000000U)

/* TRCCLKOUT_CLK_SRC_SEL */

#define CSL_MSS_TOPRCM_TRCCLKOUT_CLK_SRC_SEL_TRCCLKOUT_CLK_SRC_SEL_CLKSRCSEL_MASK (0x00000FFFU)
#define CSL_MSS_TOPRCM_TRCCLKOUT_CLK_SRC_SEL_TRCCLKOUT_CLK_SRC_SEL_CLKSRCSEL_SHIFT (0x00000000U)
#define CSL_MSS_TOPRCM_TRCCLKOUT_CLK_SRC_SEL_TRCCLKOUT_CLK_SRC_SEL_CLKSRCSEL_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_TRCCLKOUT_CLK_SRC_SEL_TRCCLKOUT_CLK_SRC_SEL_CLKSRCSEL_MAX (0x00000FFFU)

#define CSL_MSS_TOPRCM_TRCCLKOUT_CLK_SRC_SEL_RESETVAL                          (0x00000000U)

/* HSI_DIV_VAL */

#define CSL_MSS_TOPRCM_HSI_DIV_VAL_CSIRX_DIV_VAL_CLKDIV_MASK                   (0x00000FFFU)
#define CSL_MSS_TOPRCM_HSI_DIV_VAL_CSIRX_DIV_VAL_CLKDIV_SHIFT                  (0x00000000U)
#define CSL_MSS_TOPRCM_HSI_DIV_VAL_CSIRX_DIV_VAL_CLKDIV_RESETVAL               (0x00000000U)
#define CSL_MSS_TOPRCM_HSI_DIV_VAL_CSIRX_DIV_VAL_CLKDIV_MAX                    (0x00000FFFU)

#define CSL_MSS_TOPRCM_HSI_DIV_VAL_RESETVAL                                    (0x00000000U)

/* CSIRX_DIV_VAL */

#define CSL_MSS_TOPRCM_CSIRX_DIV_VAL_CSIRX_DIV_VAL_CLKDIV_MASK                 (0x00000FFFU)
#define CSL_MSS_TOPRCM_CSIRX_DIV_VAL_CSIRX_DIV_VAL_CLKDIV_SHIFT                (0x00000000U)
#define CSL_MSS_TOPRCM_CSIRX_DIV_VAL_CSIRX_DIV_VAL_CLKDIV_RESETVAL             (0x00000000U)
#define CSL_MSS_TOPRCM_CSIRX_DIV_VAL_CSIRX_DIV_VAL_CLKDIV_MAX                  (0x00000FFFU)

#define CSL_MSS_TOPRCM_CSIRX_DIV_VAL_RESETVAL                                  (0x00000000U)

/* MCUCLKOUT_DIV_VAL */

#define CSL_MSS_TOPRCM_MCUCLKOUT_DIV_VAL_MCUCLKOUT_DIV_VAL_CLKDIV_MASK         (0x00000FFFU)
#define CSL_MSS_TOPRCM_MCUCLKOUT_DIV_VAL_MCUCLKOUT_DIV_VAL_CLKDIV_SHIFT        (0x00000000U)
#define CSL_MSS_TOPRCM_MCUCLKOUT_DIV_VAL_MCUCLKOUT_DIV_VAL_CLKDIV_RESETVAL     (0x00000000U)
#define CSL_MSS_TOPRCM_MCUCLKOUT_DIV_VAL_MCUCLKOUT_DIV_VAL_CLKDIV_MAX          (0x00000FFFU)

#define CSL_MSS_TOPRCM_MCUCLKOUT_DIV_VAL_RESETVAL                              (0x00000000U)

/* PMICCLKOUT_DIV_VAL */

#define CSL_MSS_TOPRCM_PMICCLKOUT_DIV_VAL_PMICCLKOUT_DIV_VAL_CLKDIV_MASK       (0x00000FFFU)
#define CSL_MSS_TOPRCM_PMICCLKOUT_DIV_VAL_PMICCLKOUT_DIV_VAL_CLKDIV_SHIFT      (0x00000000U)
#define CSL_MSS_TOPRCM_PMICCLKOUT_DIV_VAL_PMICCLKOUT_DIV_VAL_CLKDIV_RESETVAL   (0x00000000U)
#define CSL_MSS_TOPRCM_PMICCLKOUT_DIV_VAL_PMICCLKOUT_DIV_VAL_CLKDIV_MAX        (0x00000FFFU)

#define CSL_MSS_TOPRCM_PMICCLKOUT_DIV_VAL_RESETVAL                             (0x00000000U)

/* OBSCLKOUT_DIV_VAL */

#define CSL_MSS_TOPRCM_OBSCLKOUT_DIV_VAL_OBSCLKOUT_DIV_VAL_CLKDIV_MASK         (0x00000FFFU)
#define CSL_MSS_TOPRCM_OBSCLKOUT_DIV_VAL_OBSCLKOUT_DIV_VAL_CLKDIV_SHIFT        (0x00000000U)
#define CSL_MSS_TOPRCM_OBSCLKOUT_DIV_VAL_OBSCLKOUT_DIV_VAL_CLKDIV_RESETVAL     (0x00000000U)
#define CSL_MSS_TOPRCM_OBSCLKOUT_DIV_VAL_OBSCLKOUT_DIV_VAL_CLKDIV_MAX          (0x00000FFFU)

#define CSL_MSS_TOPRCM_OBSCLKOUT_DIV_VAL_RESETVAL                              (0x00000000U)

/* TRCCLKOUT_DIV_VAL */

#define CSL_MSS_TOPRCM_TRCCLKOUT_DIV_VAL_TRCCLKOUT_DIV_VAL_CLKDIV_MASK         (0x00000FFFU)
#define CSL_MSS_TOPRCM_TRCCLKOUT_DIV_VAL_TRCCLKOUT_DIV_VAL_CLKDIV_SHIFT        (0x00000000U)
#define CSL_MSS_TOPRCM_TRCCLKOUT_DIV_VAL_TRCCLKOUT_DIV_VAL_CLKDIV_RESETVAL     (0x00000000U)
#define CSL_MSS_TOPRCM_TRCCLKOUT_DIV_VAL_TRCCLKOUT_DIV_VAL_CLKDIV_MAX          (0x00000FFFU)

#define CSL_MSS_TOPRCM_TRCCLKOUT_DIV_VAL_RESETVAL                              (0x00000000U)

/* HSI_CLK_GATE */

#define CSL_MSS_TOPRCM_HSI_CLK_GATE_CSIRX_CLK_GATE_GATED_MASK                  (0x00000007U)
#define CSL_MSS_TOPRCM_HSI_CLK_GATE_CSIRX_CLK_GATE_GATED_SHIFT                 (0x00000000U)
#define CSL_MSS_TOPRCM_HSI_CLK_GATE_CSIRX_CLK_GATE_GATED_RESETVAL              (0x00000000U)
#define CSL_MSS_TOPRCM_HSI_CLK_GATE_CSIRX_CLK_GATE_GATED_MAX                   (0x00000007U)

#define CSL_MSS_TOPRCM_HSI_CLK_GATE_RESETVAL                                   (0x00000000U)

/* CSIRX_CLK_GATE */

#define CSL_MSS_TOPRCM_CSIRX_CLK_GATE_CSIRX_CLK_GATE_GATED_MASK                (0x00000007U)
#define CSL_MSS_TOPRCM_CSIRX_CLK_GATE_CSIRX_CLK_GATE_GATED_SHIFT               (0x00000000U)
#define CSL_MSS_TOPRCM_CSIRX_CLK_GATE_CSIRX_CLK_GATE_GATED_RESETVAL            (0x00000000U)
#define CSL_MSS_TOPRCM_CSIRX_CLK_GATE_CSIRX_CLK_GATE_GATED_MAX                 (0x00000007U)

#define CSL_MSS_TOPRCM_CSIRX_CLK_GATE_RESETVAL                                 (0x00000000U)

/* MCUCLKOUT_CLK_GATE */

#define CSL_MSS_TOPRCM_MCUCLKOUT_CLK_GATE_MCUCLKOUT_CLK_GATE_GATED_MASK        (0x00000007U)
#define CSL_MSS_TOPRCM_MCUCLKOUT_CLK_GATE_MCUCLKOUT_CLK_GATE_GATED_SHIFT       (0x00000000U)
#define CSL_MSS_TOPRCM_MCUCLKOUT_CLK_GATE_MCUCLKOUT_CLK_GATE_GATED_RESETVAL    (0x00000007U)
#define CSL_MSS_TOPRCM_MCUCLKOUT_CLK_GATE_MCUCLKOUT_CLK_GATE_GATED_MAX         (0x00000007U)

#define CSL_MSS_TOPRCM_MCUCLKOUT_CLK_GATE_RESETVAL                             (0x00000007U)

/* PMICCLKOUT_CLK_GATE */

#define CSL_MSS_TOPRCM_PMICCLKOUT_CLK_GATE_PMICCLKOUT_CLK_GATE_GATED_MASK      (0x00000007U)
#define CSL_MSS_TOPRCM_PMICCLKOUT_CLK_GATE_PMICCLKOUT_CLK_GATE_GATED_SHIFT     (0x00000000U)
#define CSL_MSS_TOPRCM_PMICCLKOUT_CLK_GATE_PMICCLKOUT_CLK_GATE_GATED_RESETVAL  (0x00000007U)
#define CSL_MSS_TOPRCM_PMICCLKOUT_CLK_GATE_PMICCLKOUT_CLK_GATE_GATED_MAX       (0x00000007U)

#define CSL_MSS_TOPRCM_PMICCLKOUT_CLK_GATE_RESETVAL                            (0x00000007U)

/* OBSCLKOUT_CLK_GATE */

#define CSL_MSS_TOPRCM_OBSCLKOUT_CLK_GATE_OBSCLKOUT_CLK_GATE_GATED_MASK        (0x00000007U)
#define CSL_MSS_TOPRCM_OBSCLKOUT_CLK_GATE_OBSCLKOUT_CLK_GATE_GATED_SHIFT       (0x00000000U)
#define CSL_MSS_TOPRCM_OBSCLKOUT_CLK_GATE_OBSCLKOUT_CLK_GATE_GATED_RESETVAL    (0x00000000U)
#define CSL_MSS_TOPRCM_OBSCLKOUT_CLK_GATE_OBSCLKOUT_CLK_GATE_GATED_MAX         (0x00000007U)

#define CSL_MSS_TOPRCM_OBSCLKOUT_CLK_GATE_RESETVAL                             (0x00000000U)

/* TRCCLKOUT_CLK_GATE */

#define CSL_MSS_TOPRCM_TRCCLKOUT_CLK_GATE_TRCCLKOUT_CLK_GATE_GATED_MASK        (0x00000007U)
#define CSL_MSS_TOPRCM_TRCCLKOUT_CLK_GATE_TRCCLKOUT_CLK_GATE_GATED_SHIFT       (0x00000000U)
#define CSL_MSS_TOPRCM_TRCCLKOUT_CLK_GATE_TRCCLKOUT_CLK_GATE_GATED_RESETVAL    (0x00000000U)
#define CSL_MSS_TOPRCM_TRCCLKOUT_CLK_GATE_TRCCLKOUT_CLK_GATE_GATED_MAX         (0x00000007U)

#define CSL_MSS_TOPRCM_TRCCLKOUT_CLK_GATE_RESETVAL                             (0x00000000U)

/* DSS_CLK_GATE */

#define CSL_MSS_TOPRCM_DSS_CLK_GATE_DSS_CLK_GATE_GATED_MASK                    (0x00000007U)
#define CSL_MSS_TOPRCM_DSS_CLK_GATE_DSS_CLK_GATE_GATED_SHIFT                   (0x00000000U)
#define CSL_MSS_TOPRCM_DSS_CLK_GATE_DSS_CLK_GATE_GATED_RESETVAL                (0x00000000U)
#define CSL_MSS_TOPRCM_DSS_CLK_GATE_DSS_CLK_GATE_GATED_MAX                     (0x00000007U)

#define CSL_MSS_TOPRCM_DSS_CLK_GATE_RESETVAL                                   (0x00000000U)

/* HSI_CLK_STATUS */

#define CSL_MSS_TOPRCM_HSI_CLK_STATUS_HSI_CLK_STATUS_CLKINUSE_MASK             (0x000000FFU)
#define CSL_MSS_TOPRCM_HSI_CLK_STATUS_HSI_CLK_STATUS_CLKINUSE_SHIFT            (0x00000000U)
#define CSL_MSS_TOPRCM_HSI_CLK_STATUS_HSI_CLK_STATUS_CLKINUSE_RESETVAL         (0x00000002U)
#define CSL_MSS_TOPRCM_HSI_CLK_STATUS_HSI_CLK_STATUS_CLKINUSE_MAX              (0x000000FFU)

#define CSL_MSS_TOPRCM_HSI_CLK_STATUS_RESETVAL                                 (0x00000002U)

/* CSIRX_CLK_STATUS */

#define CSL_MSS_TOPRCM_CSIRX_CLK_STATUS_CSIRX_CLK_STATUS_CLKINUSE_MASK         (0x000000FFU)
#define CSL_MSS_TOPRCM_CSIRX_CLK_STATUS_CSIRX_CLK_STATUS_CLKINUSE_SHIFT        (0x00000000U)
#define CSL_MSS_TOPRCM_CSIRX_CLK_STATUS_CSIRX_CLK_STATUS_CLKINUSE_RESETVAL     (0x00000001U)
#define CSL_MSS_TOPRCM_CSIRX_CLK_STATUS_CSIRX_CLK_STATUS_CLKINUSE_MAX          (0x000000FFU)

#define CSL_MSS_TOPRCM_CSIRX_CLK_STATUS_CSIRX_CLK_STATUS_CURRDIVIDER_MASK      (0x0000FF00U)
#define CSL_MSS_TOPRCM_CSIRX_CLK_STATUS_CSIRX_CLK_STATUS_CURRDIVIDER_SHIFT     (0x00000008U)
#define CSL_MSS_TOPRCM_CSIRX_CLK_STATUS_CSIRX_CLK_STATUS_CURRDIVIDER_RESETVAL  (0x00000000U)
#define CSL_MSS_TOPRCM_CSIRX_CLK_STATUS_CSIRX_CLK_STATUS_CURRDIVIDER_MAX       (0x000000FFU)

#define CSL_MSS_TOPRCM_CSIRX_CLK_STATUS_RESETVAL                               (0x00000001U)

/* MCUCLKOUT_CLK_STATUS */

#define CSL_MSS_TOPRCM_MCUCLKOUT_CLK_STATUS_MCUCLKOUT_CLK_STATUS_CLKINUSE_MASK (0x000000FFU)
#define CSL_MSS_TOPRCM_MCUCLKOUT_CLK_STATUS_MCUCLKOUT_CLK_STATUS_CLKINUSE_SHIFT (0x00000000U)
#define CSL_MSS_TOPRCM_MCUCLKOUT_CLK_STATUS_MCUCLKOUT_CLK_STATUS_CLKINUSE_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_MCUCLKOUT_CLK_STATUS_MCUCLKOUT_CLK_STATUS_CLKINUSE_MAX  (0x000000FFU)

#define CSL_MSS_TOPRCM_MCUCLKOUT_CLK_STATUS_MCUCLKOUT_CLK_STATUS_CURRDIVIDER_MASK (0x0000FF00U)
#define CSL_MSS_TOPRCM_MCUCLKOUT_CLK_STATUS_MCUCLKOUT_CLK_STATUS_CURRDIVIDER_SHIFT (0x00000008U)
#define CSL_MSS_TOPRCM_MCUCLKOUT_CLK_STATUS_MCUCLKOUT_CLK_STATUS_CURRDIVIDER_RESETVAL (0x00000001U)
#define CSL_MSS_TOPRCM_MCUCLKOUT_CLK_STATUS_MCUCLKOUT_CLK_STATUS_CURRDIVIDER_MAX (0x000000FFU)

#define CSL_MSS_TOPRCM_MCUCLKOUT_CLK_STATUS_RESETVAL                           (0x00000100U)

/* PMICCLKOUT_CLK_STATUS */

#define CSL_MSS_TOPRCM_PMICCLKOUT_CLK_STATUS_PMICCLKOUT_CLK_STATUS_CLKINUSE_MASK (0x000000FFU)
#define CSL_MSS_TOPRCM_PMICCLKOUT_CLK_STATUS_PMICCLKOUT_CLK_STATUS_CLKINUSE_SHIFT (0x00000000U)
#define CSL_MSS_TOPRCM_PMICCLKOUT_CLK_STATUS_PMICCLKOUT_CLK_STATUS_CLKINUSE_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PMICCLKOUT_CLK_STATUS_PMICCLKOUT_CLK_STATUS_CLKINUSE_MAX (0x000000FFU)

#define CSL_MSS_TOPRCM_PMICCLKOUT_CLK_STATUS_PMICCLKOUT_CLK_STATUS_CURRDIVIDER_MASK (0x0000FF00U)
#define CSL_MSS_TOPRCM_PMICCLKOUT_CLK_STATUS_PMICCLKOUT_CLK_STATUS_CURRDIVIDER_SHIFT (0x00000008U)
#define CSL_MSS_TOPRCM_PMICCLKOUT_CLK_STATUS_PMICCLKOUT_CLK_STATUS_CURRDIVIDER_RESETVAL (0x00000001U)
#define CSL_MSS_TOPRCM_PMICCLKOUT_CLK_STATUS_PMICCLKOUT_CLK_STATUS_CURRDIVIDER_MAX (0x000000FFU)

#define CSL_MSS_TOPRCM_PMICCLKOUT_CLK_STATUS_RESETVAL                          (0x00000100U)

/* OBSCLKOUT_CLK_STATUS */

#define CSL_MSS_TOPRCM_OBSCLKOUT_CLK_STATUS_OBSCLKOUT_CLK_STATUS_CLKINUSE_MASK (0x000000FFU)
#define CSL_MSS_TOPRCM_OBSCLKOUT_CLK_STATUS_OBSCLKOUT_CLK_STATUS_CLKINUSE_SHIFT (0x00000000U)
#define CSL_MSS_TOPRCM_OBSCLKOUT_CLK_STATUS_OBSCLKOUT_CLK_STATUS_CLKINUSE_RESETVAL (0x00000001U)
#define CSL_MSS_TOPRCM_OBSCLKOUT_CLK_STATUS_OBSCLKOUT_CLK_STATUS_CLKINUSE_MAX  (0x000000FFU)

#define CSL_MSS_TOPRCM_OBSCLKOUT_CLK_STATUS_OBSCLKOUT_CLK_STATUS_CURRDIVIDER_MASK (0x0000FF00U)
#define CSL_MSS_TOPRCM_OBSCLKOUT_CLK_STATUS_OBSCLKOUT_CLK_STATUS_CURRDIVIDER_SHIFT (0x00000008U)
#define CSL_MSS_TOPRCM_OBSCLKOUT_CLK_STATUS_OBSCLKOUT_CLK_STATUS_CURRDIVIDER_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_OBSCLKOUT_CLK_STATUS_OBSCLKOUT_CLK_STATUS_CURRDIVIDER_MAX (0x000000FFU)

#define CSL_MSS_TOPRCM_OBSCLKOUT_CLK_STATUS_RESETVAL                           (0x00000001U)

/* TRCCLKOUT_CLK_STATUS */

#define CSL_MSS_TOPRCM_TRCCLKOUT_CLK_STATUS_TRCCLKOUT_CLK_STATUS_CLKINUSE_MASK (0x000000FFU)
#define CSL_MSS_TOPRCM_TRCCLKOUT_CLK_STATUS_TRCCLKOUT_CLK_STATUS_CLKINUSE_SHIFT (0x00000000U)
#define CSL_MSS_TOPRCM_TRCCLKOUT_CLK_STATUS_TRCCLKOUT_CLK_STATUS_CLKINUSE_RESETVAL (0x00000001U)
#define CSL_MSS_TOPRCM_TRCCLKOUT_CLK_STATUS_TRCCLKOUT_CLK_STATUS_CLKINUSE_MAX  (0x000000FFU)

#define CSL_MSS_TOPRCM_TRCCLKOUT_CLK_STATUS_TRCCLKOUT_CLK_STATUS_CURRDIVIDER_MASK (0x0000FF00U)
#define CSL_MSS_TOPRCM_TRCCLKOUT_CLK_STATUS_TRCCLKOUT_CLK_STATUS_CURRDIVIDER_SHIFT (0x00000008U)
#define CSL_MSS_TOPRCM_TRCCLKOUT_CLK_STATUS_TRCCLKOUT_CLK_STATUS_CURRDIVIDER_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_TRCCLKOUT_CLK_STATUS_TRCCLKOUT_CLK_STATUS_CURRDIVIDER_MAX (0x000000FFU)

#define CSL_MSS_TOPRCM_TRCCLKOUT_CLK_STATUS_RESETVAL                           (0x00000001U)

/* WARM_RESET_CONFIG */

#define CSL_MSS_TOPRCM_WARM_RESET_CONFIG_WARM_RESET_CONFIG_PAD_BYPASS_MASK     (0x00000007U)
#define CSL_MSS_TOPRCM_WARM_RESET_CONFIG_WARM_RESET_CONFIG_PAD_BYPASS_SHIFT    (0x00000000U)
#define CSL_MSS_TOPRCM_WARM_RESET_CONFIG_WARM_RESET_CONFIG_PAD_BYPASS_RESETVAL (0x00000007U)
#define CSL_MSS_TOPRCM_WARM_RESET_CONFIG_WARM_RESET_CONFIG_PAD_BYPASS_MAX      (0x00000007U)

#define CSL_MSS_TOPRCM_WARM_RESET_CONFIG_WARM_RESET_CONFIG_SW_RST_MASK         (0x00000700U)
#define CSL_MSS_TOPRCM_WARM_RESET_CONFIG_WARM_RESET_CONFIG_SW_RST_SHIFT        (0x00000008U)
#define CSL_MSS_TOPRCM_WARM_RESET_CONFIG_WARM_RESET_CONFIG_SW_RST_RESETVAL     (0x00000007U)
#define CSL_MSS_TOPRCM_WARM_RESET_CONFIG_WARM_RESET_CONFIG_SW_RST_MAX          (0x00000007U)

#define CSL_MSS_TOPRCM_WARM_RESET_CONFIG_WARM_RESET_CONFIG_WDOG_RST_EN_MASK    (0x00070000U)
#define CSL_MSS_TOPRCM_WARM_RESET_CONFIG_WARM_RESET_CONFIG_WDOG_RST_EN_SHIFT   (0x00000010U)
#define CSL_MSS_TOPRCM_WARM_RESET_CONFIG_WARM_RESET_CONFIG_WDOG_RST_EN_RESETVAL (0x00000007U)
#define CSL_MSS_TOPRCM_WARM_RESET_CONFIG_WARM_RESET_CONFIG_WDOG_RST_EN_MAX     (0x00000007U)

#define CSL_MSS_TOPRCM_WARM_RESET_CONFIG_RESETVAL                              (0x00070707U)

/* SYS_RST_CAUSE */

#define CSL_MSS_TOPRCM_SYS_RST_CAUSE_SYS_RST_CAUSE_CAUSE_MASK                  (0x0000001FU)
#define CSL_MSS_TOPRCM_SYS_RST_CAUSE_SYS_RST_CAUSE_CAUSE_SHIFT                 (0x00000000U)
#define CSL_MSS_TOPRCM_SYS_RST_CAUSE_SYS_RST_CAUSE_CAUSE_RESETVAL              (0x00000009U)
#define CSL_MSS_TOPRCM_SYS_RST_CAUSE_SYS_RST_CAUSE_CAUSE_MAX                   (0x0000001FU)

#define CSL_MSS_TOPRCM_SYS_RST_CAUSE_RESETVAL                                  (0x00000009U)

/* SYS_RST_CAUSE_CLR */

#define CSL_MSS_TOPRCM_SYS_RST_CAUSE_CLR_SYS_RST_CAUSE_CLR_CLEAR_MASK          (0x00000001U)
#define CSL_MSS_TOPRCM_SYS_RST_CAUSE_CLR_SYS_RST_CAUSE_CLR_CLEAR_SHIFT         (0x00000000U)
#define CSL_MSS_TOPRCM_SYS_RST_CAUSE_CLR_SYS_RST_CAUSE_CLR_CLEAR_RESETVAL      (0x00000000U)
#define CSL_MSS_TOPRCM_SYS_RST_CAUSE_CLR_SYS_RST_CAUSE_CLR_CLEAR_MAX           (0x00000001U)

#define CSL_MSS_TOPRCM_SYS_RST_CAUSE_CLR_RESETVAL                              (0x00000000U)

/* DSS_RST_CTRL */

#define CSL_MSS_TOPRCM_DSS_RST_CTRL_DSS_RST_CTRL_ASSERT_MASK                   (0x00000007U)
#define CSL_MSS_TOPRCM_DSS_RST_CTRL_DSS_RST_CTRL_ASSERT_SHIFT                  (0x00000000U)
#define CSL_MSS_TOPRCM_DSS_RST_CTRL_DSS_RST_CTRL_ASSERT_RESETVAL               (0x00000000U)
#define CSL_MSS_TOPRCM_DSS_RST_CTRL_DSS_RST_CTRL_ASSERT_MAX                    (0x00000007U)

#define CSL_MSS_TOPRCM_DSS_RST_CTRL_RESETVAL                                   (0x00000000U)

/* RS232_BITINTERVAL */

#define CSL_MSS_TOPRCM_RS232_BITINTERVAL_RS232_BITINTERVAL_BITINTERVAL_MASK    (0xFFFFFFFFU)
#define CSL_MSS_TOPRCM_RS232_BITINTERVAL_RS232_BITINTERVAL_BITINTERVAL_SHIFT   (0x00000000U)
#define CSL_MSS_TOPRCM_RS232_BITINTERVAL_RS232_BITINTERVAL_BITINTERVAL_RESETVAL (0x6C815D5BU)
#define CSL_MSS_TOPRCM_RS232_BITINTERVAL_RS232_BITINTERVAL_BITINTERVAL_MAX     (0xFFFFFFFFU)

#define CSL_MSS_TOPRCM_RS232_BITINTERVAL_RESETVAL                              (0x6C815D5BU)

/* LVDS_PAD_CTRL0 */

#define CSL_MSS_TOPRCM_LVDS_PAD_CTRL0_LVDS_PAD_CTRL0_CTRL_MASK                 (0xFFFFFFFFU)
#define CSL_MSS_TOPRCM_LVDS_PAD_CTRL0_LVDS_PAD_CTRL0_CTRL_SHIFT                (0x00000000U)
#define CSL_MSS_TOPRCM_LVDS_PAD_CTRL0_LVDS_PAD_CTRL0_CTRL_RESETVAL             (0x01010101U)
#define CSL_MSS_TOPRCM_LVDS_PAD_CTRL0_LVDS_PAD_CTRL0_CTRL_MAX                  (0xFFFFFFFFU)

#define CSL_MSS_TOPRCM_LVDS_PAD_CTRL0_RESETVAL                                 (0x01010101U)

/* LVDS_PAD_CTRL1 */

#define CSL_MSS_TOPRCM_LVDS_PAD_CTRL1_LVDS_PAD_CTRL1_CTLR_MASK                 (0xFFFFFFFFU)
#define CSL_MSS_TOPRCM_LVDS_PAD_CTRL1_LVDS_PAD_CTRL1_CTLR_SHIFT                (0x00000000U)
#define CSL_MSS_TOPRCM_LVDS_PAD_CTRL1_LVDS_PAD_CTRL1_CTLR_RESETVAL             (0x00000101U)
#define CSL_MSS_TOPRCM_LVDS_PAD_CTRL1_LVDS_PAD_CTRL1_CTLR_MAX                  (0xFFFFFFFFU)

#define CSL_MSS_TOPRCM_LVDS_PAD_CTRL1_RESETVAL                                 (0x00000101U)

/* DFT_DMLED_EXEC */

#define CSL_MSS_TOPRCM_DFT_DMLED_EXEC_DFT_DMLED_EXEC_VAL_MASK                  (0xFFFFFFFFU)
#define CSL_MSS_TOPRCM_DFT_DMLED_EXEC_DFT_DMLED_EXEC_VAL_SHIFT                 (0x00000000U)
#define CSL_MSS_TOPRCM_DFT_DMLED_EXEC_DFT_DMLED_EXEC_VAL_RESETVAL              (0x00000000U)
#define CSL_MSS_TOPRCM_DFT_DMLED_EXEC_DFT_DMLED_EXEC_VAL_MAX                   (0xFFFFFFFFU)

#define CSL_MSS_TOPRCM_DFT_DMLED_EXEC_RESETVAL                                 (0x00000000U)

/* DFT_DMLED_STATUS */

#define CSL_MSS_TOPRCM_DFT_DMLED_STATUS_DFT_DMLED_STATUS_VAL_MASK              (0xFFFFFFFFU)
#define CSL_MSS_TOPRCM_DFT_DMLED_STATUS_DFT_DMLED_STATUS_VAL_SHIFT             (0x00000000U)
#define CSL_MSS_TOPRCM_DFT_DMLED_STATUS_DFT_DMLED_STATUS_VAL_RESETVAL          (0x00000000U)
#define CSL_MSS_TOPRCM_DFT_DMLED_STATUS_DFT_DMLED_STATUS_VAL_MAX               (0xFFFFFFFFU)

#define CSL_MSS_TOPRCM_DFT_DMLED_STATUS_RESETVAL                               (0x00000000U)

/* LIMP_MODE_EN */

#define CSL_MSS_TOPRCM_LIMP_MODE_EN_LIMP_MODE_EN_DCCA_EN_MASK                  (0x00000007U)
#define CSL_MSS_TOPRCM_LIMP_MODE_EN_LIMP_MODE_EN_DCCA_EN_SHIFT                 (0x00000000U)
#define CSL_MSS_TOPRCM_LIMP_MODE_EN_LIMP_MODE_EN_DCCA_EN_RESETVAL              (0x00000000U)
#define CSL_MSS_TOPRCM_LIMP_MODE_EN_LIMP_MODE_EN_DCCA_EN_MAX                   (0x00000007U)

#define CSL_MSS_TOPRCM_LIMP_MODE_EN_LIMP_MODE_EN_CCCA_EN_MASK                  (0x00000070U)
#define CSL_MSS_TOPRCM_LIMP_MODE_EN_LIMP_MODE_EN_CCCA_EN_SHIFT                 (0x00000004U)
#define CSL_MSS_TOPRCM_LIMP_MODE_EN_LIMP_MODE_EN_CCCA_EN_RESETVAL              (0x00000000U)
#define CSL_MSS_TOPRCM_LIMP_MODE_EN_LIMP_MODE_EN_CCCA_EN_MAX                   (0x00000007U)

#define CSL_MSS_TOPRCM_LIMP_MODE_EN_LIMP_MODE_EN_FORCE_RCCLK_EN_MASK           (0x00000700U)
#define CSL_MSS_TOPRCM_LIMP_MODE_EN_LIMP_MODE_EN_FORCE_RCCLK_EN_SHIFT          (0x00000008U)
#define CSL_MSS_TOPRCM_LIMP_MODE_EN_LIMP_MODE_EN_FORCE_RCCLK_EN_RESETVAL       (0x00000000U)
#define CSL_MSS_TOPRCM_LIMP_MODE_EN_LIMP_MODE_EN_FORCE_RCCLK_EN_MAX            (0x00000007U)

#define CSL_MSS_TOPRCM_LIMP_MODE_EN_RESETVAL                                   (0x00000000U)

/* PMICCLKOUT_DCDC_CTRL */

#define CSL_MSS_TOPRCM_PMICCLKOUT_DCDC_CTRL_PMICCLKOUT_DCDC_CTRL_DCDC_CLK_EN_MASK (0x00000001U)
#define CSL_MSS_TOPRCM_PMICCLKOUT_DCDC_CTRL_PMICCLKOUT_DCDC_CTRL_DCDC_CLK_EN_SHIFT (0x00000000U)
#define CSL_MSS_TOPRCM_PMICCLKOUT_DCDC_CTRL_PMICCLKOUT_DCDC_CTRL_DCDC_CLK_EN_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PMICCLKOUT_DCDC_CTRL_PMICCLKOUT_DCDC_CTRL_DCDC_CLK_EN_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PMICCLKOUT_DCDC_CTRL_PMICCLKOUT_DCDC_CTRL_DITHER_EN_MASK (0x00000002U)
#define CSL_MSS_TOPRCM_PMICCLKOUT_DCDC_CTRL_PMICCLKOUT_DCDC_CTRL_DITHER_EN_SHIFT (0x00000001U)
#define CSL_MSS_TOPRCM_PMICCLKOUT_DCDC_CTRL_PMICCLKOUT_DCDC_CTRL_DITHER_EN_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PMICCLKOUT_DCDC_CTRL_PMICCLKOUT_DCDC_CTRL_DITHER_EN_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PMICCLKOUT_DCDC_CTRL_PMICCLKOUT_DCDC_CTRL_FREQ_ACC_MODE_MASK (0x00000004U)
#define CSL_MSS_TOPRCM_PMICCLKOUT_DCDC_CTRL_PMICCLKOUT_DCDC_CTRL_FREQ_ACC_MODE_SHIFT (0x00000002U)
#define CSL_MSS_TOPRCM_PMICCLKOUT_DCDC_CTRL_PMICCLKOUT_DCDC_CTRL_FREQ_ACC_MODE_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PMICCLKOUT_DCDC_CTRL_PMICCLKOUT_DCDC_CTRL_FREQ_ACC_MODE_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PMICCLKOUT_DCDC_CTRL_PMICCLKOUT_DCDC_CTRL_RESET_ASSERT_MASK (0x00000070U)
#define CSL_MSS_TOPRCM_PMICCLKOUT_DCDC_CTRL_PMICCLKOUT_DCDC_CTRL_RESET_ASSERT_SHIFT (0x00000004U)
#define CSL_MSS_TOPRCM_PMICCLKOUT_DCDC_CTRL_PMICCLKOUT_DCDC_CTRL_RESET_ASSERT_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PMICCLKOUT_DCDC_CTRL_PMICCLKOUT_DCDC_CTRL_RESET_ASSERT_MAX (0x00000007U)

#define CSL_MSS_TOPRCM_PMICCLKOUT_DCDC_CTRL_PMICCLKOUT_DCDC_CTRL_MIN_FREQ_THR_MASK (0x0000FF00U)
#define CSL_MSS_TOPRCM_PMICCLKOUT_DCDC_CTRL_PMICCLKOUT_DCDC_CTRL_MIN_FREQ_THR_SHIFT (0x00000008U)
#define CSL_MSS_TOPRCM_PMICCLKOUT_DCDC_CTRL_PMICCLKOUT_DCDC_CTRL_MIN_FREQ_THR_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PMICCLKOUT_DCDC_CTRL_PMICCLKOUT_DCDC_CTRL_MIN_FREQ_THR_MAX (0x000000FFU)

#define CSL_MSS_TOPRCM_PMICCLKOUT_DCDC_CTRL_PMICCLKOUT_DCDC_CTRL_MAX_FREQ_THR_MASK (0x00FF0000U)
#define CSL_MSS_TOPRCM_PMICCLKOUT_DCDC_CTRL_PMICCLKOUT_DCDC_CTRL_MAX_FREQ_THR_SHIFT (0x00000010U)
#define CSL_MSS_TOPRCM_PMICCLKOUT_DCDC_CTRL_PMICCLKOUT_DCDC_CTRL_MAX_FREQ_THR_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PMICCLKOUT_DCDC_CTRL_PMICCLKOUT_DCDC_CTRL_MAX_FREQ_THR_MAX (0x000000FFU)

#define CSL_MSS_TOPRCM_PMICCLKOUT_DCDC_CTRL_RESETVAL                           (0x00000000U)

/* PMICCLKOUT_DCDC_SLOPE */

#define CSL_MSS_TOPRCM_PMICCLKOUT_DCDC_SLOPE_PMICCLKOUT_DCDC_SLOPE_SLOPE_VAL_MASK (0x07FFFFFFU)
#define CSL_MSS_TOPRCM_PMICCLKOUT_DCDC_SLOPE_PMICCLKOUT_DCDC_SLOPE_SLOPE_VAL_SHIFT (0x00000000U)
#define CSL_MSS_TOPRCM_PMICCLKOUT_DCDC_SLOPE_PMICCLKOUT_DCDC_SLOPE_SLOPE_VAL_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PMICCLKOUT_DCDC_SLOPE_PMICCLKOUT_DCDC_SLOPE_SLOPE_VAL_MAX (0x07FFFFFFU)

#define CSL_MSS_TOPRCM_PMICCLKOUT_DCDC_SLOPE_RESETVAL                          (0x00000000U)

/* RCOSC32K_CTRL */

#define CSL_MSS_TOPRCM_RCOSC32K_CTRL_RCOSC32K_CTRL_STOPOSC_MASK                (0x00000007U)
#define CSL_MSS_TOPRCM_RCOSC32K_CTRL_RCOSC32K_CTRL_STOPOSC_SHIFT               (0x00000000U)
#define CSL_MSS_TOPRCM_RCOSC32K_CTRL_RCOSC32K_CTRL_STOPOSC_RESETVAL            (0x00000000U)
#define CSL_MSS_TOPRCM_RCOSC32K_CTRL_RCOSC32K_CTRL_STOPOSC_MAX                 (0x00000007U)

#define CSL_MSS_TOPRCM_RCOSC32K_CTRL_RESETVAL                                  (0x00000000U)

/* ANA_HSI2DIGCLK_GATE */

#define CSL_MSS_TOPRCM_ANA_HSI2DIGCLK_GATE_ANA_HSI2DIGCLK_GATE_GATED_MASK      (0x00000007U)
#define CSL_MSS_TOPRCM_ANA_HSI2DIGCLK_GATE_ANA_HSI2DIGCLK_GATE_GATED_SHIFT     (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_HSI2DIGCLK_GATE_ANA_HSI2DIGCLK_GATE_GATED_RESETVAL  (0x00000007U)
#define CSL_MSS_TOPRCM_ANA_HSI2DIGCLK_GATE_ANA_HSI2DIGCLK_GATE_GATED_MAX       (0x00000007U)

#define CSL_MSS_TOPRCM_ANA_HSI2DIGCLK_GATE_RESETVAL                            (0x00000007U)

/* PLL_CORE_PWRCTRL */

#define CSL_MSS_TOPRCM_PLL_CORE_PWRCTRL_PLL_CORE_PWRCTRL_OFFMODE_MASK          (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_CORE_PWRCTRL_PLL_CORE_PWRCTRL_OFFMODE_SHIFT         (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_PWRCTRL_PLL_CORE_PWRCTRL_OFFMODE_RESETVAL      (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_PWRCTRL_PLL_CORE_PWRCTRL_OFFMODE_MAX           (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_CORE_PWRCTRL_PLL_CORE_PWRCTRL_ISOSCAN_MASK          (0x00000002U)
#define CSL_MSS_TOPRCM_PLL_CORE_PWRCTRL_PLL_CORE_PWRCTRL_ISOSCAN_SHIFT         (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_CORE_PWRCTRL_PLL_CORE_PWRCTRL_ISOSCAN_RESETVAL      (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_PWRCTRL_PLL_CORE_PWRCTRL_ISOSCAN_MAX           (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_CORE_PWRCTRL_PLL_CORE_PWRCTRL_ISORET_MASK           (0x00000004U)
#define CSL_MSS_TOPRCM_PLL_CORE_PWRCTRL_PLL_CORE_PWRCTRL_ISORET_SHIFT          (0x00000002U)
#define CSL_MSS_TOPRCM_PLL_CORE_PWRCTRL_PLL_CORE_PWRCTRL_ISORET_RESETVAL       (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_PWRCTRL_PLL_CORE_PWRCTRL_ISORET_MAX            (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_CORE_PWRCTRL_PLL_CORE_PWRCTRL_RET_MASK              (0x00000008U)
#define CSL_MSS_TOPRCM_PLL_CORE_PWRCTRL_PLL_CORE_PWRCTRL_RET_SHIFT             (0x00000003U)
#define CSL_MSS_TOPRCM_PLL_CORE_PWRCTRL_PLL_CORE_PWRCTRL_RET_RESETVAL          (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_PWRCTRL_PLL_CORE_PWRCTRL_RET_MAX               (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_CORE_PWRCTRL_PLL_CORE_PWRCTRL_PGOODIN_MASK          (0x00000010U)
#define CSL_MSS_TOPRCM_PLL_CORE_PWRCTRL_PLL_CORE_PWRCTRL_PGOODIN_SHIFT         (0x00000004U)
#define CSL_MSS_TOPRCM_PLL_CORE_PWRCTRL_PLL_CORE_PWRCTRL_PGOODIN_RESETVAL      (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_CORE_PWRCTRL_PLL_CORE_PWRCTRL_PGOODIN_MAX           (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_CORE_PWRCTRL_PLL_CORE_PWRCTRL_PONIN_MASK            (0x00000020U)
#define CSL_MSS_TOPRCM_PLL_CORE_PWRCTRL_PLL_CORE_PWRCTRL_PONIN_SHIFT           (0x00000005U)
#define CSL_MSS_TOPRCM_PLL_CORE_PWRCTRL_PLL_CORE_PWRCTRL_PONIN_RESETVAL        (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_CORE_PWRCTRL_PLL_CORE_PWRCTRL_PONIN_MAX             (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_CORE_PWRCTRL_RESETVAL                               (0x00000030U)

/* PLL_CORE_CLKCTRL */

#define CSL_MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_TINTZ_MASK            (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_TINTZ_SHIFT           (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_TINTZ_RESETVAL        (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_TINTZ_MAX             (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_SSCTYPE_MASK          (0x00000002U)
#define CSL_MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_SSCTYPE_SHIFT         (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_SSCTYPE_RESETVAL      (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_SSCTYPE_MAX           (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_RELAXED_LOCK_MASK     (0x00000100U)
#define CSL_MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_RELAXED_LOCK_SHIFT    (0x00000008U)
#define CSL_MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_RELAXED_LOCK_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_RELAXED_LOCK_MAX      (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_SELFREQDCO_MASK       (0x00001C00U)
#define CSL_MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_SELFREQDCO_SHIFT      (0x0000000AU)
#define CSL_MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_SELFREQDCO_RESETVAL   (0x00000002U)
#define CSL_MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_SELFREQDCO_MAX        (0x00000007U)

#define CSL_MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_STOPMODE_MASK         (0x00004000U)
#define CSL_MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_STOPMODE_SHIFT        (0x0000000EU)
#define CSL_MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_STOPMODE_RESETVAL     (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_STOPMODE_MAX          (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_M2PWDNZ_MASK          (0x00010000U)
#define CSL_MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_M2PWDNZ_SHIFT         (0x00000010U)
#define CSL_MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_M2PWDNZ_RESETVAL      (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_M2PWDNZ_MAX           (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_CLKDCOLDOPWDNZ_MASK   (0x00020000U)
#define CSL_MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_CLKDCOLDOPWDNZ_SHIFT  (0x00000011U)
#define CSL_MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_CLKDCOLDOPWDNZ_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_CLKDCOLDOPWDNZ_MAX    (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_ULOWCLKEN_MASK        (0x00040000U)
#define CSL_MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_ULOWCLKEN_SHIFT       (0x00000012U)
#define CSL_MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_ULOWCLKEN_RESETVAL    (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_ULOWCLKEN_MAX         (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_CLKOUTLDOEN_MASK      (0x00080000U)
#define CSL_MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_CLKOUTLDOEN_SHIFT     (0x00000013U)
#define CSL_MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_CLKOUTLDOEN_RESETVAL  (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_CLKOUTLDOEN_MAX       (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_CLKOUTEN_MASK         (0x00100000U)
#define CSL_MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_CLKOUTEN_SHIFT        (0x00000014U)
#define CSL_MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_CLKOUTEN_RESETVAL     (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_CLKOUTEN_MAX          (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_STBYRET_MASK          (0x00200000U)
#define CSL_MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_STBYRET_SHIFT         (0x00000015U)
#define CSL_MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_STBYRET_RESETVAL      (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_STBYRET_MAX           (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_BYPASSACKZ_MASK       (0x00400000U)
#define CSL_MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_BYPASSACKZ_SHIFT      (0x00000016U)
#define CSL_MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_BYPASSACKZ_RESETVAL   (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_BYPASSACKZ_MAX        (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_IDLE_MASK             (0x00800000U)
#define CSL_MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_IDLE_SHIFT            (0x00000017U)
#define CSL_MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_IDLE_RESETVAL         (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_IDLE_MAX              (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_NWELLTRIM_MASK        (0x1F000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_NWELLTRIM_SHIFT       (0x00000018U)
#define CSL_MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_NWELLTRIM_RESETVAL    (0x00000009U)
#define CSL_MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_NWELLTRIM_MAX         (0x0000001FU)

#define CSL_MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_CLKDCOLDOEN_MASK      (0x20000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_CLKDCOLDOEN_SHIFT     (0x0000001DU)
#define CSL_MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_CLKDCOLDOEN_RESETVAL  (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_CLKDCOLDOEN_MAX       (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_ENSSC_MASK            (0x40000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_ENSSC_SHIFT           (0x0000001EU)
#define CSL_MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_ENSSC_RESETVAL        (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_ENSSC_MAX             (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_CYCLESLIPEN_MASK      (0x80000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_CYCLESLIPEN_SHIFT     (0x0000001FU)
#define CSL_MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_CYCLESLIPEN_RESETVAL  (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_CYCLESLIPEN_MAX       (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_CORE_CLKCTRL_RESETVAL                               (0x09914800U)

/* PLL_CORE_TENABLE */

#define CSL_MSS_TOPRCM_PLL_CORE_TENABLE_PLL_CORE_TENABLE_TENABLE_MASK          (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_CORE_TENABLE_PLL_CORE_TENABLE_TENABLE_SHIFT         (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_TENABLE_PLL_CORE_TENABLE_TENABLE_RESETVAL      (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_TENABLE_PLL_CORE_TENABLE_TENABLE_MAX           (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_CORE_TENABLE_RESETVAL                               (0x00000000U)

/* PLL_CORE_TENABLEDIV */

#define CSL_MSS_TOPRCM_PLL_CORE_TENABLEDIV_PLL_CORE_TENABLEDIV_TENABLEDIV_MASK (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_CORE_TENABLEDIV_PLL_CORE_TENABLEDIV_TENABLEDIV_SHIFT (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_TENABLEDIV_PLL_CORE_TENABLEDIV_TENABLEDIV_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_TENABLEDIV_PLL_CORE_TENABLEDIV_TENABLEDIV_MAX  (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_CORE_TENABLEDIV_RESETVAL                            (0x00000000U)

/* PLL_CORE_M2NDIV */

#define CSL_MSS_TOPRCM_PLL_CORE_M2NDIV_PLL_CORE_M2NDIV_N_MASK                  (0x000000FFU)
#define CSL_MSS_TOPRCM_PLL_CORE_M2NDIV_PLL_CORE_M2NDIV_N_SHIFT                 (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_M2NDIV_PLL_CORE_M2NDIV_N_RESETVAL              (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_M2NDIV_PLL_CORE_M2NDIV_N_MAX                   (0x000000FFU)

#define CSL_MSS_TOPRCM_PLL_CORE_M2NDIV_PLL_CORE_M2NDIV_M2_MASK                 (0x007F0000U)
#define CSL_MSS_TOPRCM_PLL_CORE_M2NDIV_PLL_CORE_M2NDIV_M2_SHIFT                (0x00000010U)
#define CSL_MSS_TOPRCM_PLL_CORE_M2NDIV_PLL_CORE_M2NDIV_M2_RESETVAL             (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_M2NDIV_PLL_CORE_M2NDIV_M2_MAX                  (0x0000007FU)

#define CSL_MSS_TOPRCM_PLL_CORE_M2NDIV_RESETVAL                                (0x00000000U)

/* PLL_CORE_MN2DIV */

#define CSL_MSS_TOPRCM_PLL_CORE_MN2DIV_PLL_CORE_MN2DIV_M_MASK                  (0x00000FFFU)
#define CSL_MSS_TOPRCM_PLL_CORE_MN2DIV_PLL_CORE_MN2DIV_M_SHIFT                 (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_MN2DIV_PLL_CORE_MN2DIV_M_RESETVAL              (0x00000174U)
#define CSL_MSS_TOPRCM_PLL_CORE_MN2DIV_PLL_CORE_MN2DIV_M_MAX                   (0x00000FFFU)

#define CSL_MSS_TOPRCM_PLL_CORE_MN2DIV_PLL_CORE_MN2DIV_N2_MASK                 (0x000F0000U)
#define CSL_MSS_TOPRCM_PLL_CORE_MN2DIV_PLL_CORE_MN2DIV_N2_SHIFT                (0x00000010U)
#define CSL_MSS_TOPRCM_PLL_CORE_MN2DIV_PLL_CORE_MN2DIV_N2_RESETVAL             (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_MN2DIV_PLL_CORE_MN2DIV_N2_MAX                  (0x0000000FU)

#define CSL_MSS_TOPRCM_PLL_CORE_MN2DIV_RESETVAL                                (0x00000174U)

/* PLL_CORE_FRACDIV */

#define CSL_MSS_TOPRCM_PLL_CORE_FRACDIV_PLL_CORE_FRACDIV_FRACTIONALM_MASK      (0x0003FFFFU)
#define CSL_MSS_TOPRCM_PLL_CORE_FRACDIV_PLL_CORE_FRACDIV_FRACTIONALM_SHIFT     (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_FRACDIV_PLL_CORE_FRACDIV_FRACTIONALM_RESETVAL  (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_FRACDIV_PLL_CORE_FRACDIV_FRACTIONALM_MAX       (0x0003FFFFU)

#define CSL_MSS_TOPRCM_PLL_CORE_FRACDIV_PLL_CORE_FRACDIV_REGSD_MASK            (0xFF000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_FRACDIV_PLL_CORE_FRACDIV_REGSD_SHIFT           (0x00000018U)
#define CSL_MSS_TOPRCM_PLL_CORE_FRACDIV_PLL_CORE_FRACDIV_REGSD_RESETVAL        (0x00000008U)
#define CSL_MSS_TOPRCM_PLL_CORE_FRACDIV_PLL_CORE_FRACDIV_REGSD_MAX             (0x000000FFU)

#define CSL_MSS_TOPRCM_PLL_CORE_FRACDIV_RESETVAL                               (0x08000000U)

/* PLL_CORE_BWCTRL */

#define CSL_MSS_TOPRCM_PLL_CORE_BWCTRL_PLL_CORE_BWCTRL_BW_INCR_DECRZ_MASK      (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_CORE_BWCTRL_PLL_CORE_BWCTRL_BW_INCR_DECRZ_SHIFT     (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_BWCTRL_PLL_CORE_BWCTRL_BW_INCR_DECRZ_RESETVAL  (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_BWCTRL_PLL_CORE_BWCTRL_BW_INCR_DECRZ_MAX       (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_CORE_BWCTRL_PLL_CORE_BWCTRL_BWCONTROL_MASK          (0x00000006U)
#define CSL_MSS_TOPRCM_PLL_CORE_BWCTRL_PLL_CORE_BWCTRL_BWCONTROL_SHIFT         (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_CORE_BWCTRL_PLL_CORE_BWCTRL_BWCONTROL_RESETVAL      (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_BWCTRL_PLL_CORE_BWCTRL_BWCONTROL_MAX           (0x00000003U)

#define CSL_MSS_TOPRCM_PLL_CORE_BWCTRL_RESETVAL                                (0x00000000U)

/* PLL_CORE_FRACCTRL */

#define CSL_MSS_TOPRCM_PLL_CORE_FRACCTRL_PLL_CORE_FRACCTRL_DELTAMSTEPFRACTION_MASK (0x0003FFFFU)
#define CSL_MSS_TOPRCM_PLL_CORE_FRACCTRL_PLL_CORE_FRACCTRL_DELTAMSTEPFRACTION_SHIFT (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_FRACCTRL_PLL_CORE_FRACCTRL_DELTAMSTEPFRACTION_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_FRACCTRL_PLL_CORE_FRACCTRL_DELTAMSTEPFRACTION_MAX (0x0003FFFFU)

#define CSL_MSS_TOPRCM_PLL_CORE_FRACCTRL_PLL_CORE_FRACCTRL_DELTAMSTEPINTEGER_MASK (0x001C0000U)
#define CSL_MSS_TOPRCM_PLL_CORE_FRACCTRL_PLL_CORE_FRACCTRL_DELTAMSTEPINTEGER_SHIFT (0x00000012U)
#define CSL_MSS_TOPRCM_PLL_CORE_FRACCTRL_PLL_CORE_FRACCTRL_DELTAMSTEPINTEGER_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_FRACCTRL_PLL_CORE_FRACCTRL_DELTAMSTEPINTEGER_MAX (0x00000007U)

#define CSL_MSS_TOPRCM_PLL_CORE_FRACCTRL_PLL_CORE_FRACCTRL_MODFREQDIVIDERMANTISSA_MASK (0x0FE00000U)
#define CSL_MSS_TOPRCM_PLL_CORE_FRACCTRL_PLL_CORE_FRACCTRL_MODFREQDIVIDERMANTISSA_SHIFT (0x00000015U)
#define CSL_MSS_TOPRCM_PLL_CORE_FRACCTRL_PLL_CORE_FRACCTRL_MODFREQDIVIDERMANTISSA_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_FRACCTRL_PLL_CORE_FRACCTRL_MODFREQDIVIDERMANTISSA_MAX (0x0000007FU)

#define CSL_MSS_TOPRCM_PLL_CORE_FRACCTRL_PLL_CORE_FRACCTRL_MODFREQDIVIDEREXPONENT_MASK (0x70000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_FRACCTRL_PLL_CORE_FRACCTRL_MODFREQDIVIDEREXPONENT_SHIFT (0x0000001CU)
#define CSL_MSS_TOPRCM_PLL_CORE_FRACCTRL_PLL_CORE_FRACCTRL_MODFREQDIVIDEREXPONENT_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_FRACCTRL_PLL_CORE_FRACCTRL_MODFREQDIVIDEREXPONENT_MAX (0x00000007U)

#define CSL_MSS_TOPRCM_PLL_CORE_FRACCTRL_PLL_CORE_FRACCTRL_DOWNSPREAD_MASK     (0x80000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_FRACCTRL_PLL_CORE_FRACCTRL_DOWNSPREAD_SHIFT    (0x0000001FU)
#define CSL_MSS_TOPRCM_PLL_CORE_FRACCTRL_PLL_CORE_FRACCTRL_DOWNSPREAD_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_FRACCTRL_PLL_CORE_FRACCTRL_DOWNSPREAD_MAX      (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_CORE_FRACCTRL_RESETVAL                              (0x00000000U)

/* PLL_CORE_STATUS */

#define CSL_MSS_TOPRCM_PLL_CORE_STATUS_PLL_CORE_STATUS_BYPASS_MASK             (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_CORE_STATUS_PLL_CORE_STATUS_BYPASS_SHIFT            (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_STATUS_PLL_CORE_STATUS_BYPASS_RESETVAL         (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_CORE_STATUS_PLL_CORE_STATUS_BYPASS_MAX              (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_CORE_STATUS_PLL_CORE_STATUS_HIGHJITTER_MASK         (0x00000002U)
#define CSL_MSS_TOPRCM_PLL_CORE_STATUS_PLL_CORE_STATUS_HIGHJITTER_SHIFT        (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_CORE_STATUS_PLL_CORE_STATUS_HIGHJITTER_RESETVAL     (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_STATUS_PLL_CORE_STATUS_HIGHJITTER_MAX          (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_CORE_STATUS_PLL_CORE_STATUS_SSCACK_MASK             (0x00000004U)
#define CSL_MSS_TOPRCM_PLL_CORE_STATUS_PLL_CORE_STATUS_SSCACK_SHIFT            (0x00000002U)
#define CSL_MSS_TOPRCM_PLL_CORE_STATUS_PLL_CORE_STATUS_SSCACK_RESETVAL         (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_STATUS_PLL_CORE_STATUS_SSCACK_MAX              (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_CORE_STATUS_PLL_CORE_STATUS_M2CHANGEACK_MASK        (0x00000008U)
#define CSL_MSS_TOPRCM_PLL_CORE_STATUS_PLL_CORE_STATUS_M2CHANGEACK_SHIFT       (0x00000003U)
#define CSL_MSS_TOPRCM_PLL_CORE_STATUS_PLL_CORE_STATUS_M2CHANGEACK_RESETVAL    (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_STATUS_PLL_CORE_STATUS_M2CHANGEACK_MAX         (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_CORE_STATUS_PLL_CORE_STATUS_LOCK2_MASK              (0x00000010U)
#define CSL_MSS_TOPRCM_PLL_CORE_STATUS_PLL_CORE_STATUS_LOCK2_SHIFT             (0x00000004U)
#define CSL_MSS_TOPRCM_PLL_CORE_STATUS_PLL_CORE_STATUS_LOCK2_RESETVAL          (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_STATUS_PLL_CORE_STATUS_LOCK2_MAX               (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_CORE_STATUS_PLL_CORE_STATUS_CLKOUTENACK_MASK        (0x00000020U)
#define CSL_MSS_TOPRCM_PLL_CORE_STATUS_PLL_CORE_STATUS_CLKOUTENACK_SHIFT       (0x00000005U)
#define CSL_MSS_TOPRCM_PLL_CORE_STATUS_PLL_CORE_STATUS_CLKOUTENACK_RESETVAL    (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_CORE_STATUS_PLL_CORE_STATUS_CLKOUTENACK_MAX         (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_CORE_STATUS_PLL_CORE_STATUS_LOSSREF_MASK            (0x00000040U)
#define CSL_MSS_TOPRCM_PLL_CORE_STATUS_PLL_CORE_STATUS_LOSSREF_SHIFT           (0x00000006U)
#define CSL_MSS_TOPRCM_PLL_CORE_STATUS_PLL_CORE_STATUS_LOSSREF_RESETVAL        (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_CORE_STATUS_PLL_CORE_STATUS_LOSSREF_MAX             (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_CORE_STATUS_PLL_CORE_STATUS_STBYRETACK_MASK         (0x00000080U)
#define CSL_MSS_TOPRCM_PLL_CORE_STATUS_PLL_CORE_STATUS_STBYRETACK_SHIFT        (0x00000007U)
#define CSL_MSS_TOPRCM_PLL_CORE_STATUS_PLL_CORE_STATUS_STBYRETACK_RESETVAL     (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_STATUS_PLL_CORE_STATUS_STBYRETACK_MAX          (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_CORE_STATUS_PLL_CORE_STATUS_BYPASSACK_MASK          (0x00000100U)
#define CSL_MSS_TOPRCM_PLL_CORE_STATUS_PLL_CORE_STATUS_BYPASSACK_SHIFT         (0x00000008U)
#define CSL_MSS_TOPRCM_PLL_CORE_STATUS_PLL_CORE_STATUS_BYPASSACK_RESETVAL      (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_CORE_STATUS_PLL_CORE_STATUS_BYPASSACK_MAX           (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_CORE_STATUS_PLL_CORE_STATUS_FREQLOCK_MASK           (0x00000200U)
#define CSL_MSS_TOPRCM_PLL_CORE_STATUS_PLL_CORE_STATUS_FREQLOCK_SHIFT          (0x00000009U)
#define CSL_MSS_TOPRCM_PLL_CORE_STATUS_PLL_CORE_STATUS_FREQLOCK_RESETVAL       (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_STATUS_PLL_CORE_STATUS_FREQLOCK_MAX            (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_CORE_STATUS_PLL_CORE_STATUS_PHASELOCK_MASK          (0x00000400U)
#define CSL_MSS_TOPRCM_PLL_CORE_STATUS_PLL_CORE_STATUS_PHASELOCK_SHIFT         (0x0000000AU)
#define CSL_MSS_TOPRCM_PLL_CORE_STATUS_PLL_CORE_STATUS_PHASELOCK_RESETVAL      (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_STATUS_PLL_CORE_STATUS_PHASELOCK_MAX           (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_CORE_STATUS_PLL_CORE_STATUS_CLKDCOLDOACK_MASK       (0x00000800U)
#define CSL_MSS_TOPRCM_PLL_CORE_STATUS_PLL_CORE_STATUS_CLKDCOLDOACK_SHIFT      (0x0000000BU)
#define CSL_MSS_TOPRCM_PLL_CORE_STATUS_PLL_CORE_STATUS_CLKDCOLDOACK_RESETVAL   (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_STATUS_PLL_CORE_STATUS_CLKDCOLDOACK_MAX        (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_CORE_STATUS_PLL_CORE_STATUS_RECAL_OPPIN_MASK        (0x08000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_STATUS_PLL_CORE_STATUS_RECAL_OPPIN_SHIFT       (0x0000001BU)
#define CSL_MSS_TOPRCM_PLL_CORE_STATUS_PLL_CORE_STATUS_RECAL_OPPIN_RESETVAL    (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_STATUS_PLL_CORE_STATUS_RECAL_OPPIN_MAX         (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_CORE_STATUS_PLL_CORE_STATUS_RECAL_BSTATUS3_MASK     (0x10000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_STATUS_PLL_CORE_STATUS_RECAL_BSTATUS3_SHIFT    (0x0000001CU)
#define CSL_MSS_TOPRCM_PLL_CORE_STATUS_PLL_CORE_STATUS_RECAL_BSTATUS3_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_STATUS_PLL_CORE_STATUS_RECAL_BSTATUS3_MAX      (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_CORE_STATUS_PLL_CORE_STATUS_LDOPWDN_MASK            (0x20000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_STATUS_PLL_CORE_STATUS_LDOPWDN_SHIFT           (0x0000001DU)
#define CSL_MSS_TOPRCM_PLL_CORE_STATUS_PLL_CORE_STATUS_LDOPWDN_RESETVAL        (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_CORE_STATUS_PLL_CORE_STATUS_LDOPWDN_MAX             (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_CORE_STATUS_PLL_CORE_STATUS_PGOODOUT_MASK           (0x40000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_STATUS_PLL_CORE_STATUS_PGOODOUT_SHIFT          (0x0000001EU)
#define CSL_MSS_TOPRCM_PLL_CORE_STATUS_PLL_CORE_STATUS_PGOODOUT_RESETVAL       (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_CORE_STATUS_PLL_CORE_STATUS_PGOODOUT_MAX            (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_CORE_STATUS_PLL_CORE_STATUS_PONOUT_MASK             (0x80000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_STATUS_PLL_CORE_STATUS_PONOUT_SHIFT            (0x0000001FU)
#define CSL_MSS_TOPRCM_PLL_CORE_STATUS_PLL_CORE_STATUS_PONOUT_RESETVAL         (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_CORE_STATUS_PLL_CORE_STATUS_PONOUT_MAX              (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_CORE_STATUS_RESETVAL                                (0xE0000161U)

/* PLL_CORE_HSDIVIDER */

#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_PLL_CORE_HSDIVIDER_BYPASS_MASK       (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_PLL_CORE_HSDIVIDER_BYPASS_SHIFT      (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_PLL_CORE_HSDIVIDER_BYPASS_RESETVAL   (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_PLL_CORE_HSDIVIDER_BYPASS_MAX        (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_PLL_CORE_HSDIVIDER_LDOPWDN_MASK      (0x00000002U)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_PLL_CORE_HSDIVIDER_LDOPWDN_SHIFT     (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_PLL_CORE_HSDIVIDER_LDOPWDN_RESETVAL  (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_PLL_CORE_HSDIVIDER_LDOPWDN_MAX       (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_PLL_CORE_HSDIVIDER_TENABLEDIV_MASK   (0x00000004U)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_PLL_CORE_HSDIVIDER_TENABLEDIV_SHIFT  (0x00000002U)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_PLL_CORE_HSDIVIDER_TENABLEDIV_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_PLL_CORE_HSDIVIDER_TENABLEDIV_MAX    (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_PLL_CORE_HSDIVIDER_BYPASSACKZ_MASK   (0x00010000U)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_PLL_CORE_HSDIVIDER_BYPASSACKZ_SHIFT  (0x00000010U)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_PLL_CORE_HSDIVIDER_BYPASSACKZ_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_PLL_CORE_HSDIVIDER_BYPASSACKZ_MAX    (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_PLL_CORE_HSDIVIDER_LDOPWDNACK_MASK   (0x00020000U)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_PLL_CORE_HSDIVIDER_LDOPWDNACK_SHIFT  (0x00000011U)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_PLL_CORE_HSDIVIDER_LDOPWDNACK_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_PLL_CORE_HSDIVIDER_LDOPWDNACK_MAX    (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_RESETVAL                             (0x00000000U)

/* PLL_CORE_HSDIVIDER_CLKOUT0 */

#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT0_PLL_CORE_HSDIVIDER_CLKOUT0_DIV_MASK (0x0000001FU)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT0_PLL_CORE_HSDIVIDER_CLKOUT0_DIV_SHIFT (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT0_PLL_CORE_HSDIVIDER_CLKOUT0_DIV_RESETVAL (0x00000004U)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT0_PLL_CORE_HSDIVIDER_CLKOUT0_DIV_MAX (0x0000001FU)

#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT0_PLL_CORE_HSDIVIDER_CLKOUT0_DIVCHACK_MASK (0x00000020U)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT0_PLL_CORE_HSDIVIDER_CLKOUT0_DIVCHACK_SHIFT (0x00000005U)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT0_PLL_CORE_HSDIVIDER_CLKOUT0_DIVCHACK_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT0_PLL_CORE_HSDIVIDER_CLKOUT0_DIVCHACK_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT0_PLL_CORE_HSDIVIDER_CLKOUT0_GATE_CTRL_MASK (0x00000100U)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT0_PLL_CORE_HSDIVIDER_CLKOUT0_GATE_CTRL_SHIFT (0x00000008U)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT0_PLL_CORE_HSDIVIDER_CLKOUT0_GATE_CTRL_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT0_PLL_CORE_HSDIVIDER_CLKOUT0_GATE_CTRL_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT0_PLL_CORE_HSDIVIDER_CLKOUT0_STATUS_MASK (0x00000200U)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT0_PLL_CORE_HSDIVIDER_CLKOUT0_STATUS_SHIFT (0x00000009U)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT0_PLL_CORE_HSDIVIDER_CLKOUT0_STATUS_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT0_PLL_CORE_HSDIVIDER_CLKOUT0_STATUS_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT0_PLL_CORE_HSDIVIDER_CLKOUT0_PWDN_MASK (0x00001000U)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT0_PLL_CORE_HSDIVIDER_CLKOUT0_PWDN_SHIFT (0x0000000CU)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT0_PLL_CORE_HSDIVIDER_CLKOUT0_PWDN_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT0_PLL_CORE_HSDIVIDER_CLKOUT0_PWDN_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT0_RESETVAL                     (0x00000004U)

/* PLL_CORE_HSDIVIDER_CLKOUT1 */

#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT1_PLL_CORE_HSDIVIDER_CLKOUT1_DIV_MASK (0x0000001FU)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT1_PLL_CORE_HSDIVIDER_CLKOUT1_DIV_SHIFT (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT1_PLL_CORE_HSDIVIDER_CLKOUT1_DIV_RESETVAL (0x00000004U)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT1_PLL_CORE_HSDIVIDER_CLKOUT1_DIV_MAX (0x0000001FU)

#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT1_PLL_CORE_HSDIVIDER_CLKOUT1_DIVCHACK_MASK (0x00000020U)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT1_PLL_CORE_HSDIVIDER_CLKOUT1_DIVCHACK_SHIFT (0x00000005U)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT1_PLL_CORE_HSDIVIDER_CLKOUT1_DIVCHACK_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT1_PLL_CORE_HSDIVIDER_CLKOUT1_DIVCHACK_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT1_PLL_CORE_HSDIVIDER_CLKOUT1_GATE_CTRL_MASK (0x00000100U)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT1_PLL_CORE_HSDIVIDER_CLKOUT1_GATE_CTRL_SHIFT (0x00000008U)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT1_PLL_CORE_HSDIVIDER_CLKOUT1_GATE_CTRL_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT1_PLL_CORE_HSDIVIDER_CLKOUT1_GATE_CTRL_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT1_PLL_CORE_HSDIVIDER_CLKOUT1_STATUS_MASK (0x00000200U)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT1_PLL_CORE_HSDIVIDER_CLKOUT1_STATUS_SHIFT (0x00000009U)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT1_PLL_CORE_HSDIVIDER_CLKOUT1_STATUS_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT1_PLL_CORE_HSDIVIDER_CLKOUT1_STATUS_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT1_PLL_CORE_HSDIVIDER_CLKOUT1_PWDN_MASK (0x00001000U)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT1_PLL_CORE_HSDIVIDER_CLKOUT1_PWDN_SHIFT (0x0000000CU)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT1_PLL_CORE_HSDIVIDER_CLKOUT1_PWDN_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT1_PLL_CORE_HSDIVIDER_CLKOUT1_PWDN_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT1_RESETVAL                     (0x00000004U)

/* PLL_CORE_HSDIVIDER_CLKOUT2 */

#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT2_PLL_CORE_HSDIVIDER_CLKOUT2_DIV_MASK (0x0000001FU)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT2_PLL_CORE_HSDIVIDER_CLKOUT2_DIV_SHIFT (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT2_PLL_CORE_HSDIVIDER_CLKOUT2_DIV_RESETVAL (0x00000004U)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT2_PLL_CORE_HSDIVIDER_CLKOUT2_DIV_MAX (0x0000001FU)

#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT2_PLL_CORE_HSDIVIDER_CLKOUT2_DIVCHACK_MASK (0x00000020U)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT2_PLL_CORE_HSDIVIDER_CLKOUT2_DIVCHACK_SHIFT (0x00000005U)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT2_PLL_CORE_HSDIVIDER_CLKOUT2_DIVCHACK_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT2_PLL_CORE_HSDIVIDER_CLKOUT2_DIVCHACK_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT2_PLL_CORE_HSDIVIDER_CLKOUT2_GATE_CTRL_MASK (0x00000100U)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT2_PLL_CORE_HSDIVIDER_CLKOUT2_GATE_CTRL_SHIFT (0x00000008U)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT2_PLL_CORE_HSDIVIDER_CLKOUT2_GATE_CTRL_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT2_PLL_CORE_HSDIVIDER_CLKOUT2_GATE_CTRL_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT2_PLL_CORE_HSDIVIDER_CLKOUT2_STATUS_MASK (0x00000200U)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT2_PLL_CORE_HSDIVIDER_CLKOUT2_STATUS_SHIFT (0x00000009U)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT2_PLL_CORE_HSDIVIDER_CLKOUT2_STATUS_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT2_PLL_CORE_HSDIVIDER_CLKOUT2_STATUS_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT2_PLL_CORE_HSDIVIDER_CLKOUT2_PWDN_MASK (0x00001000U)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT2_PLL_CORE_HSDIVIDER_CLKOUT2_PWDN_SHIFT (0x0000000CU)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT2_PLL_CORE_HSDIVIDER_CLKOUT2_PWDN_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT2_PLL_CORE_HSDIVIDER_CLKOUT2_PWDN_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT2_RESETVAL                     (0x00000004U)

/* PLL_CORE_HSDIVIDER_CLKOUT3 */

#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT3_PLL_CORE_HSDIVIDER_CLKOUT3_DIV_MASK (0x0000001FU)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT3_PLL_CORE_HSDIVIDER_CLKOUT3_DIV_SHIFT (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT3_PLL_CORE_HSDIVIDER_CLKOUT3_DIV_RESETVAL (0x00000004U)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT3_PLL_CORE_HSDIVIDER_CLKOUT3_DIV_MAX (0x0000001FU)

#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT3_PLL_CORE_HSDIVIDER_CLKOUT3_DIVCHACK_MASK (0x00000020U)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT3_PLL_CORE_HSDIVIDER_CLKOUT3_DIVCHACK_SHIFT (0x00000005U)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT3_PLL_CORE_HSDIVIDER_CLKOUT3_DIVCHACK_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT3_PLL_CORE_HSDIVIDER_CLKOUT3_DIVCHACK_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT3_PLL_CORE_HSDIVIDER_CLKOUT3_GATE_CTRL_MASK (0x00000100U)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT3_PLL_CORE_HSDIVIDER_CLKOUT3_GATE_CTRL_SHIFT (0x00000008U)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT3_PLL_CORE_HSDIVIDER_CLKOUT3_GATE_CTRL_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT3_PLL_CORE_HSDIVIDER_CLKOUT3_GATE_CTRL_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT3_PLL_CORE_HSDIVIDER_CLKOUT3_STATUS_MASK (0x00000200U)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT3_PLL_CORE_HSDIVIDER_CLKOUT3_STATUS_SHIFT (0x00000009U)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT3_PLL_CORE_HSDIVIDER_CLKOUT3_STATUS_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT3_PLL_CORE_HSDIVIDER_CLKOUT3_STATUS_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT3_PLL_CORE_HSDIVIDER_CLKOUT3_PWDN_MASK (0x00001000U)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT3_PLL_CORE_HSDIVIDER_CLKOUT3_PWDN_SHIFT (0x0000000CU)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT3_PLL_CORE_HSDIVIDER_CLKOUT3_PWDN_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT3_PLL_CORE_HSDIVIDER_CLKOUT3_PWDN_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT3_RESETVAL                     (0x00000004U)

/* MSS_CR5_CLK_SRC_SEL */

#define CSL_MSS_TOPRCM_MSS_CR5_CLK_SRC_SEL_MSS_CR5_CLK_SRC_SEL_CLKSRCSEL_MASK  (0x00000FFFU)
#define CSL_MSS_TOPRCM_MSS_CR5_CLK_SRC_SEL_MSS_CR5_CLK_SRC_SEL_CLKSRCSEL_SHIFT (0x00000000U)
#define CSL_MSS_TOPRCM_MSS_CR5_CLK_SRC_SEL_MSS_CR5_CLK_SRC_SEL_CLKSRCSEL_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_MSS_CR5_CLK_SRC_SEL_MSS_CR5_CLK_SRC_SEL_CLKSRCSEL_MAX   (0x00000FFFU)

#define CSL_MSS_TOPRCM_MSS_CR5_CLK_SRC_SEL_RESETVAL                            (0x00000000U)

/* MSS_CR5_DIV_VAL */

#define CSL_MSS_TOPRCM_MSS_CR5_DIV_VAL_MSS_CR5_DIV_VAL_CLKDIV_MASK             (0x00000FFFU)
#define CSL_MSS_TOPRCM_MSS_CR5_DIV_VAL_MSS_CR5_DIV_VAL_CLKDIV_SHIFT            (0x00000000U)
#define CSL_MSS_TOPRCM_MSS_CR5_DIV_VAL_MSS_CR5_DIV_VAL_CLKDIV_RESETVAL         (0x00000000U)
#define CSL_MSS_TOPRCM_MSS_CR5_DIV_VAL_MSS_CR5_DIV_VAL_CLKDIV_MAX              (0x00000FFFU)

#define CSL_MSS_TOPRCM_MSS_CR5_DIV_VAL_RESETVAL                                (0x00000000U)

/* SYS_CLK_DIV_VAL */

#define CSL_MSS_TOPRCM_SYS_CLK_DIV_VAL_SYS_CLK_DIV_VAL_CLKDIV_MASK             (0x00000FFFU)
#define CSL_MSS_TOPRCM_SYS_CLK_DIV_VAL_SYS_CLK_DIV_VAL_CLKDIV_SHIFT            (0x00000000U)
#define CSL_MSS_TOPRCM_SYS_CLK_DIV_VAL_SYS_CLK_DIV_VAL_CLKDIV_RESETVAL         (0x00000000U)
#define CSL_MSS_TOPRCM_SYS_CLK_DIV_VAL_SYS_CLK_DIV_VAL_CLKDIV_MAX              (0x00000FFFU)

#define CSL_MSS_TOPRCM_SYS_CLK_DIV_VAL_RESETVAL                                (0x00000000U)

/* MSS_CR5_CLK_GATE */

#define CSL_MSS_TOPRCM_MSS_CR5_CLK_GATE_MSS_CR5_CLK_GATE_GATED_MASK            (0x00000007U)
#define CSL_MSS_TOPRCM_MSS_CR5_CLK_GATE_MSS_CR5_CLK_GATE_GATED_SHIFT           (0x00000000U)
#define CSL_MSS_TOPRCM_MSS_CR5_CLK_GATE_MSS_CR5_CLK_GATE_GATED_RESETVAL        (0x00000000U)
#define CSL_MSS_TOPRCM_MSS_CR5_CLK_GATE_MSS_CR5_CLK_GATE_GATED_MAX             (0x00000007U)

#define CSL_MSS_TOPRCM_MSS_CR5_CLK_GATE_RESETVAL                               (0x00000000U)

/* SYS_CLK_GATE */

#define CSL_MSS_TOPRCM_SYS_CLK_GATE_SYS_CLK_GATE_GATED_MASK                    (0x00000007U)
#define CSL_MSS_TOPRCM_SYS_CLK_GATE_SYS_CLK_GATE_GATED_SHIFT                   (0x00000000U)
#define CSL_MSS_TOPRCM_SYS_CLK_GATE_SYS_CLK_GATE_GATED_RESETVAL                (0x00000000U)
#define CSL_MSS_TOPRCM_SYS_CLK_GATE_SYS_CLK_GATE_GATED_MAX                     (0x00000007U)

#define CSL_MSS_TOPRCM_SYS_CLK_GATE_RESETVAL                                   (0x00000000U)

/* SYS_CLK_STATUS */

#define CSL_MSS_TOPRCM_SYS_CLK_STATUS_SYS_CLK_STATUS_CURRDIVIDER_MASK          (0x0000FF00U)
#define CSL_MSS_TOPRCM_SYS_CLK_STATUS_SYS_CLK_STATUS_CURRDIVIDER_SHIFT         (0x00000008U)
#define CSL_MSS_TOPRCM_SYS_CLK_STATUS_SYS_CLK_STATUS_CURRDIVIDER_RESETVAL      (0x00000000U)
#define CSL_MSS_TOPRCM_SYS_CLK_STATUS_SYS_CLK_STATUS_CURRDIVIDER_MAX           (0x000000FFU)

#define CSL_MSS_TOPRCM_SYS_CLK_STATUS_RESETVAL                                 (0x00000000U)

/* MSS_CR5_CLK_STATUS */

#define CSL_MSS_TOPRCM_MSS_CR5_CLK_STATUS_MSS_CR5_CLK_STATUS_CLKINUSE_MASK     (0x000000FFU)
#define CSL_MSS_TOPRCM_MSS_CR5_CLK_STATUS_MSS_CR5_CLK_STATUS_CLKINUSE_SHIFT    (0x00000000U)
#define CSL_MSS_TOPRCM_MSS_CR5_CLK_STATUS_MSS_CR5_CLK_STATUS_CLKINUSE_RESETVAL (0x00000001U)
#define CSL_MSS_TOPRCM_MSS_CR5_CLK_STATUS_MSS_CR5_CLK_STATUS_CLKINUSE_MAX      (0x000000FFU)

#define CSL_MSS_TOPRCM_MSS_CR5_CLK_STATUS_MSS_CR5_CLK_STATUS_CURRDIVIDER_MASK  (0x0000FF00U)
#define CSL_MSS_TOPRCM_MSS_CR5_CLK_STATUS_MSS_CR5_CLK_STATUS_CURRDIVIDER_SHIFT (0x00000008U)
#define CSL_MSS_TOPRCM_MSS_CR5_CLK_STATUS_MSS_CR5_CLK_STATUS_CURRDIVIDER_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_MSS_CR5_CLK_STATUS_MSS_CR5_CLK_STATUS_CURRDIVIDER_MAX   (0x000000FFU)

#define CSL_MSS_TOPRCM_MSS_CR5_CLK_STATUS_RESETVAL                             (0x00000001U)

/* PLL_CORE_RSTCTRL */

#define CSL_MSS_TOPRCM_PLL_CORE_RSTCTRL_PLL_CORE_RSTCTRL_ASSERT_MASK           (0x00000007U)
#define CSL_MSS_TOPRCM_PLL_CORE_RSTCTRL_PLL_CORE_RSTCTRL_ASSERT_SHIFT          (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_RSTCTRL_PLL_CORE_RSTCTRL_ASSERT_RESETVAL       (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_RSTCTRL_PLL_CORE_RSTCTRL_ASSERT_MAX            (0x00000007U)

#define CSL_MSS_TOPRCM_PLL_CORE_RSTCTRL_RESETVAL                               (0x00000000U)

/* PLL_CORE_HSDIVIDER_RSTCTRL */

#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_RSTCTRL_PLL_CORE_HSDIVIDER_RSTCTRL_ASSERT_MASK (0x00000007U)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_RSTCTRL_PLL_CORE_HSDIVIDER_RSTCTRL_ASSERT_SHIFT (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_RSTCTRL_PLL_CORE_HSDIVIDER_RSTCTRL_ASSERT_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_RSTCTRL_PLL_CORE_HSDIVIDER_RSTCTRL_ASSERT_MAX (0x00000007U)

#define CSL_MSS_TOPRCM_PLL_CORE_HSDIVIDER_RSTCTRL_RESETVAL                     (0x00000000U)

/* RSS_CLK_SRC_SEL */

#define CSL_MSS_TOPRCM_RSS_CLK_SRC_SEL_RSS_CLK_SRC_SEL_CLKSRCSEL_MASK          (0x00000FFFU)
#define CSL_MSS_TOPRCM_RSS_CLK_SRC_SEL_RSS_CLK_SRC_SEL_CLKSRCSEL_SHIFT         (0x00000000U)
#define CSL_MSS_TOPRCM_RSS_CLK_SRC_SEL_RSS_CLK_SRC_SEL_CLKSRCSEL_RESETVAL      (0x00000000U)
#define CSL_MSS_TOPRCM_RSS_CLK_SRC_SEL_RSS_CLK_SRC_SEL_CLKSRCSEL_MAX           (0x00000FFFU)

#define CSL_MSS_TOPRCM_RSS_CLK_SRC_SEL_RESETVAL                                (0x00000000U)

/* PLLC_CLK2_SRC_SEL */

#define CSL_MSS_TOPRCM_PLLC_CLK2_SRC_SEL_PLLC_CLK2_SRC_SEL_CLKSRCSEL_MASK      (0x00000FFFU)
#define CSL_MSS_TOPRCM_PLLC_CLK2_SRC_SEL_PLLC_CLK2_SRC_SEL_CLKSRCSEL_SHIFT     (0x00000000U)
#define CSL_MSS_TOPRCM_PLLC_CLK2_SRC_SEL_PLLC_CLK2_SRC_SEL_CLKSRCSEL_RESETVAL  (0x00000000U)
#define CSL_MSS_TOPRCM_PLLC_CLK2_SRC_SEL_PLLC_CLK2_SRC_SEL_CLKSRCSEL_MAX       (0x00000FFFU)

#define CSL_MSS_TOPRCM_PLLC_CLK2_SRC_SEL_RESETVAL                              (0x00000000U)

/* PLLD_CLK1_SRC_SEL */

#define CSL_MSS_TOPRCM_PLLD_CLK1_SRC_SEL_PLLD_CLK1_SRC_SEL_CLKSRCSEL_MASK      (0x00000FFFU)
#define CSL_MSS_TOPRCM_PLLD_CLK1_SRC_SEL_PLLD_CLK1_SRC_SEL_CLKSRCSEL_SHIFT     (0x00000000U)
#define CSL_MSS_TOPRCM_PLLD_CLK1_SRC_SEL_PLLD_CLK1_SRC_SEL_CLKSRCSEL_RESETVAL  (0x00000000U)
#define CSL_MSS_TOPRCM_PLLD_CLK1_SRC_SEL_PLLD_CLK1_SRC_SEL_CLKSRCSEL_MAX       (0x00000FFFU)

#define CSL_MSS_TOPRCM_PLLD_CLK1_SRC_SEL_RESETVAL                              (0x00000000U)

/* PLLD_CLK2_SRC_SEL */

#define CSL_MSS_TOPRCM_PLLD_CLK2_SRC_SEL_PLLD_CLK2_SRC_SEL_CLKSRCSEL_MASK      (0x00000FFFU)
#define CSL_MSS_TOPRCM_PLLD_CLK2_SRC_SEL_PLLD_CLK2_SRC_SEL_CLKSRCSEL_SHIFT     (0x00000000U)
#define CSL_MSS_TOPRCM_PLLD_CLK2_SRC_SEL_PLLD_CLK2_SRC_SEL_CLKSRCSEL_RESETVAL  (0x00000000U)
#define CSL_MSS_TOPRCM_PLLD_CLK2_SRC_SEL_PLLD_CLK2_SRC_SEL_CLKSRCSEL_MAX       (0x00000FFFU)

#define CSL_MSS_TOPRCM_PLLD_CLK2_SRC_SEL_RESETVAL                              (0x00000000U)

/* PLLP_CLK1_SRC_SEL */

#define CSL_MSS_TOPRCM_PLLP_CLK1_SRC_SEL_PLLP_CLK1_SRC_SEL_CLKSRCSEL_MASK      (0x00000FFFU)
#define CSL_MSS_TOPRCM_PLLP_CLK1_SRC_SEL_PLLP_CLK1_SRC_SEL_CLKSRCSEL_SHIFT     (0x00000000U)
#define CSL_MSS_TOPRCM_PLLP_CLK1_SRC_SEL_PLLP_CLK1_SRC_SEL_CLKSRCSEL_RESETVAL  (0x00000000U)
#define CSL_MSS_TOPRCM_PLLP_CLK1_SRC_SEL_PLLP_CLK1_SRC_SEL_CLKSRCSEL_MAX       (0x00000FFFU)

#define CSL_MSS_TOPRCM_PLLP_CLK1_SRC_SEL_RESETVAL                              (0x00000000U)

/* RSS_DIV_VAL */

#define CSL_MSS_TOPRCM_RSS_DIV_VAL_RSS_DIV_VAL_CLKDIV_MASK                     (0x00000FFFU)
#define CSL_MSS_TOPRCM_RSS_DIV_VAL_RSS_DIV_VAL_CLKDIV_SHIFT                    (0x00000000U)
#define CSL_MSS_TOPRCM_RSS_DIV_VAL_RSS_DIV_VAL_CLKDIV_RESETVAL                 (0x00000000U)
#define CSL_MSS_TOPRCM_RSS_DIV_VAL_RSS_DIV_VAL_CLKDIV_MAX                      (0x00000FFFU)

#define CSL_MSS_TOPRCM_RSS_DIV_VAL_RESETVAL                                    (0x00000000U)

/* RSS_CLK_GATE */

#define CSL_MSS_TOPRCM_RSS_CLK_GATE_RSS_CLK_GATE_GATED_MASK                    (0x00000007U)
#define CSL_MSS_TOPRCM_RSS_CLK_GATE_RSS_CLK_GATE_GATED_SHIFT                   (0x00000000U)
#define CSL_MSS_TOPRCM_RSS_CLK_GATE_RSS_CLK_GATE_GATED_RESETVAL                (0x00000000U)
#define CSL_MSS_TOPRCM_RSS_CLK_GATE_RSS_CLK_GATE_GATED_MAX                     (0x00000007U)

#define CSL_MSS_TOPRCM_RSS_CLK_GATE_RESETVAL                                   (0x00000000U)

/* PLLC_CLK2_GATE */

#define CSL_MSS_TOPRCM_PLLC_CLK2_GATE_PLLC_CLK2_GATE_GATED_MASK                (0x00000007U)
#define CSL_MSS_TOPRCM_PLLC_CLK2_GATE_PLLC_CLK2_GATE_GATED_SHIFT               (0x00000000U)
#define CSL_MSS_TOPRCM_PLLC_CLK2_GATE_PLLC_CLK2_GATE_GATED_RESETVAL            (0x00000000U)
#define CSL_MSS_TOPRCM_PLLC_CLK2_GATE_PLLC_CLK2_GATE_GATED_MAX                 (0x00000007U)

#define CSL_MSS_TOPRCM_PLLC_CLK2_GATE_RESETVAL                                 (0x00000000U)

/* PLLD_CLK1_GATE */

#define CSL_MSS_TOPRCM_PLLD_CLK1_GATE_PLLD_CLK1_GATE_GATED_MASK                (0x00000007U)
#define CSL_MSS_TOPRCM_PLLD_CLK1_GATE_PLLD_CLK1_GATE_GATED_SHIFT               (0x00000000U)
#define CSL_MSS_TOPRCM_PLLD_CLK1_GATE_PLLD_CLK1_GATE_GATED_RESETVAL            (0x00000000U)
#define CSL_MSS_TOPRCM_PLLD_CLK1_GATE_PLLD_CLK1_GATE_GATED_MAX                 (0x00000007U)

#define CSL_MSS_TOPRCM_PLLD_CLK1_GATE_RESETVAL                                 (0x00000000U)

/* PLLD_CLK2_GATE */

#define CSL_MSS_TOPRCM_PLLD_CLK2_GATE_PLLD_CLK2_GATE_GATED_MASK                (0x00000007U)
#define CSL_MSS_TOPRCM_PLLD_CLK2_GATE_PLLD_CLK2_GATE_GATED_SHIFT               (0x00000000U)
#define CSL_MSS_TOPRCM_PLLD_CLK2_GATE_PLLD_CLK2_GATE_GATED_RESETVAL            (0x00000000U)
#define CSL_MSS_TOPRCM_PLLD_CLK2_GATE_PLLD_CLK2_GATE_GATED_MAX                 (0x00000007U)

#define CSL_MSS_TOPRCM_PLLD_CLK2_GATE_RESETVAL                                 (0x00000000U)

/* PLLP_CLK1_GATE */

#define CSL_MSS_TOPRCM_PLLP_CLK1_GATE_PLLP_CLK1_GATE_GATED_MASK                (0x00000007U)
#define CSL_MSS_TOPRCM_PLLP_CLK1_GATE_PLLP_CLK1_GATE_GATED_SHIFT               (0x00000000U)
#define CSL_MSS_TOPRCM_PLLP_CLK1_GATE_PLLP_CLK1_GATE_GATED_RESETVAL            (0x00000000U)
#define CSL_MSS_TOPRCM_PLLP_CLK1_GATE_PLLP_CLK1_GATE_GATED_MAX                 (0x00000007U)

#define CSL_MSS_TOPRCM_PLLP_CLK1_GATE_RESETVAL                                 (0x00000000U)

/* RSS_CLK_STATUS */

#define CSL_MSS_TOPRCM_RSS_CLK_STATUS_RSS_CLK_STATUS_CLKINUSE_MASK             (0x000000FFU)
#define CSL_MSS_TOPRCM_RSS_CLK_STATUS_RSS_CLK_STATUS_CLKINUSE_SHIFT            (0x00000000U)
#define CSL_MSS_TOPRCM_RSS_CLK_STATUS_RSS_CLK_STATUS_CLKINUSE_RESETVAL         (0x00000001U)
#define CSL_MSS_TOPRCM_RSS_CLK_STATUS_RSS_CLK_STATUS_CLKINUSE_MAX              (0x000000FFU)

#define CSL_MSS_TOPRCM_RSS_CLK_STATUS_RSS_CLK_STATUS_CURRDIVIDER_MASK          (0x0000FF00U)
#define CSL_MSS_TOPRCM_RSS_CLK_STATUS_RSS_CLK_STATUS_CURRDIVIDER_SHIFT         (0x00000008U)
#define CSL_MSS_TOPRCM_RSS_CLK_STATUS_RSS_CLK_STATUS_CURRDIVIDER_RESETVAL      (0x00000000U)
#define CSL_MSS_TOPRCM_RSS_CLK_STATUS_RSS_CLK_STATUS_CURRDIVIDER_MAX           (0x000000FFU)

#define CSL_MSS_TOPRCM_RSS_CLK_STATUS_RESETVAL                                 (0x00000001U)

/* PLLC_CLK2_STATUS */

#define CSL_MSS_TOPRCM_PLLC_CLK2_STATUS_PLLC_CLK2_STATUS_CLKINUSE_MASK         (0x000000FFU)
#define CSL_MSS_TOPRCM_PLLC_CLK2_STATUS_PLLC_CLK2_STATUS_CLKINUSE_SHIFT        (0x00000000U)
#define CSL_MSS_TOPRCM_PLLC_CLK2_STATUS_PLLC_CLK2_STATUS_CLKINUSE_RESETVAL     (0x00000001U)
#define CSL_MSS_TOPRCM_PLLC_CLK2_STATUS_PLLC_CLK2_STATUS_CLKINUSE_MAX          (0x000000FFU)

#define CSL_MSS_TOPRCM_PLLC_CLK2_STATUS_RESETVAL                               (0x00000001U)

/* PLLD_CLK1_STATUS */

#define CSL_MSS_TOPRCM_PLLD_CLK1_STATUS_PLLD_CLK1_STATUS_CLKINUSE_MASK         (0x000000FFU)
#define CSL_MSS_TOPRCM_PLLD_CLK1_STATUS_PLLD_CLK1_STATUS_CLKINUSE_SHIFT        (0x00000000U)
#define CSL_MSS_TOPRCM_PLLD_CLK1_STATUS_PLLD_CLK1_STATUS_CLKINUSE_RESETVAL     (0x00000001U)
#define CSL_MSS_TOPRCM_PLLD_CLK1_STATUS_PLLD_CLK1_STATUS_CLKINUSE_MAX          (0x000000FFU)

#define CSL_MSS_TOPRCM_PLLD_CLK1_STATUS_RESETVAL                               (0x00000001U)

/* PLLD_CLK2_STATUS */

#define CSL_MSS_TOPRCM_PLLD_CLK2_STATUS_PLLD_CLK2_STATUS_CLKINUSE_MASK         (0x000000FFU)
#define CSL_MSS_TOPRCM_PLLD_CLK2_STATUS_PLLD_CLK2_STATUS_CLKINUSE_SHIFT        (0x00000000U)
#define CSL_MSS_TOPRCM_PLLD_CLK2_STATUS_PLLD_CLK2_STATUS_CLKINUSE_RESETVAL     (0x00000001U)
#define CSL_MSS_TOPRCM_PLLD_CLK2_STATUS_PLLD_CLK2_STATUS_CLKINUSE_MAX          (0x000000FFU)

#define CSL_MSS_TOPRCM_PLLD_CLK2_STATUS_RESETVAL                               (0x00000001U)

/* PLLP_CLK1_STATUS */

#define CSL_MSS_TOPRCM_PLLP_CLK1_STATUS_PLLP_CLK1_STATUS_CLKINUSE_MASK         (0x000000FFU)
#define CSL_MSS_TOPRCM_PLLP_CLK1_STATUS_PLLP_CLK1_STATUS_CLKINUSE_SHIFT        (0x00000000U)
#define CSL_MSS_TOPRCM_PLLP_CLK1_STATUS_PLLP_CLK1_STATUS_CLKINUSE_RESETVAL     (0x00000001U)
#define CSL_MSS_TOPRCM_PLLP_CLK1_STATUS_PLLP_CLK1_STATUS_CLKINUSE_MAX          (0x000000FFU)

#define CSL_MSS_TOPRCM_PLLP_CLK1_STATUS_RESETVAL                               (0x00000001U)

/* PLL_1P2_HSDIVIDER */

#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_PLL_1P2_HSDIVIDER_BYPASS_MASK         (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_PLL_1P2_HSDIVIDER_BYPASS_SHIFT        (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_PLL_1P2_HSDIVIDER_BYPASS_RESETVAL     (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_PLL_1P2_HSDIVIDER_BYPASS_MAX          (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_PLL_1P2_HSDIVIDER_LDOPWDN_MASK        (0x00000002U)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_PLL_1P2_HSDIVIDER_LDOPWDN_SHIFT       (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_PLL_1P2_HSDIVIDER_LDOPWDN_RESETVAL    (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_PLL_1P2_HSDIVIDER_LDOPWDN_MAX         (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_PLL_1P2_HSDIVIDER_TENABLEDIV_MASK     (0x00000004U)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_PLL_1P2_HSDIVIDER_TENABLEDIV_SHIFT    (0x00000002U)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_PLL_1P2_HSDIVIDER_TENABLEDIV_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_PLL_1P2_HSDIVIDER_TENABLEDIV_MAX      (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_PLL_1P2_HSDIVIDER_BYPASSACKZ_MASK     (0x00010000U)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_PLL_1P2_HSDIVIDER_BYPASSACKZ_SHIFT    (0x00000010U)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_PLL_1P2_HSDIVIDER_BYPASSACKZ_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_PLL_1P2_HSDIVIDER_BYPASSACKZ_MAX      (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_PLL_1P2_HSDIVIDER_LDOPWDNACK_MASK     (0x00020000U)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_PLL_1P2_HSDIVIDER_LDOPWDNACK_SHIFT    (0x00000011U)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_PLL_1P2_HSDIVIDER_LDOPWDNACK_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_PLL_1P2_HSDIVIDER_LDOPWDNACK_MAX      (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_RESETVAL                              (0x00000000U)

/* PLL_1P2_HSDIVIDER_CLKOUT0 */

#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT0_PLL_1P2_HSDIVIDER_CLKOUT0_DIV_MASK (0x0000001FU)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT0_PLL_1P2_HSDIVIDER_CLKOUT0_DIV_SHIFT (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT0_PLL_1P2_HSDIVIDER_CLKOUT0_DIV_RESETVAL (0x00000004U)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT0_PLL_1P2_HSDIVIDER_CLKOUT0_DIV_MAX (0x0000001FU)

#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT0_PLL_1P2_HSDIVIDER_CLKOUT0_DIVCHACK_MASK (0x00000020U)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT0_PLL_1P2_HSDIVIDER_CLKOUT0_DIVCHACK_SHIFT (0x00000005U)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT0_PLL_1P2_HSDIVIDER_CLKOUT0_DIVCHACK_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT0_PLL_1P2_HSDIVIDER_CLKOUT0_DIVCHACK_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT0_PLL_1P2_HSDIVIDER_CLKOUT0_GATE_CTRL_MASK (0x00000100U)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT0_PLL_1P2_HSDIVIDER_CLKOUT0_GATE_CTRL_SHIFT (0x00000008U)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT0_PLL_1P2_HSDIVIDER_CLKOUT0_GATE_CTRL_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT0_PLL_1P2_HSDIVIDER_CLKOUT0_GATE_CTRL_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT0_PLL_1P2_HSDIVIDER_CLKOUT0_STATUS_MASK (0x00000200U)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT0_PLL_1P2_HSDIVIDER_CLKOUT0_STATUS_SHIFT (0x00000009U)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT0_PLL_1P2_HSDIVIDER_CLKOUT0_STATUS_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT0_PLL_1P2_HSDIVIDER_CLKOUT0_STATUS_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT0_PLL_1P2_HSDIVIDER_CLKOUT0_PWDN_MASK (0x00001000U)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT0_PLL_1P2_HSDIVIDER_CLKOUT0_PWDN_SHIFT (0x0000000CU)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT0_PLL_1P2_HSDIVIDER_CLKOUT0_PWDN_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT0_PLL_1P2_HSDIVIDER_CLKOUT0_PWDN_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT0_RESETVAL                      (0x00000004U)

/* PLL_1P2_HSDIVIDER_CLKOUT1 */

#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT1_PLL_1P2_HSDIVIDER_CLKOUT1_DIV_MASK (0x0000001FU)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT1_PLL_1P2_HSDIVIDER_CLKOUT1_DIV_SHIFT (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT1_PLL_1P2_HSDIVIDER_CLKOUT1_DIV_RESETVAL (0x00000004U)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT1_PLL_1P2_HSDIVIDER_CLKOUT1_DIV_MAX (0x0000001FU)

#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT1_PLL_1P2_HSDIVIDER_CLKOUT1_DIVCHACK_MASK (0x00000020U)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT1_PLL_1P2_HSDIVIDER_CLKOUT1_DIVCHACK_SHIFT (0x00000005U)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT1_PLL_1P2_HSDIVIDER_CLKOUT1_DIVCHACK_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT1_PLL_1P2_HSDIVIDER_CLKOUT1_DIVCHACK_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT1_PLL_1P2_HSDIVIDER_CLKOUT1_GATE_CTRL_MASK (0x00000100U)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT1_PLL_1P2_HSDIVIDER_CLKOUT1_GATE_CTRL_SHIFT (0x00000008U)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT1_PLL_1P2_HSDIVIDER_CLKOUT1_GATE_CTRL_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT1_PLL_1P2_HSDIVIDER_CLKOUT1_GATE_CTRL_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT1_PLL_1P2_HSDIVIDER_CLKOUT1_STATUS_MASK (0x00000200U)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT1_PLL_1P2_HSDIVIDER_CLKOUT1_STATUS_SHIFT (0x00000009U)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT1_PLL_1P2_HSDIVIDER_CLKOUT1_STATUS_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT1_PLL_1P2_HSDIVIDER_CLKOUT1_STATUS_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT1_PLL_1P2_HSDIVIDER_CLKOUT1_PWDN_MASK (0x00001000U)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT1_PLL_1P2_HSDIVIDER_CLKOUT1_PWDN_SHIFT (0x0000000CU)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT1_PLL_1P2_HSDIVIDER_CLKOUT1_PWDN_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT1_PLL_1P2_HSDIVIDER_CLKOUT1_PWDN_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT1_RESETVAL                      (0x00000004U)

/* PLL_1P2_HSDIVIDER_CLKOUT2 */

#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT2_PLL_1P2_HSDIVIDER_CLKOUT2_DIV_MASK (0x0000001FU)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT2_PLL_1P2_HSDIVIDER_CLKOUT2_DIV_SHIFT (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT2_PLL_1P2_HSDIVIDER_CLKOUT2_DIV_RESETVAL (0x00000004U)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT2_PLL_1P2_HSDIVIDER_CLKOUT2_DIV_MAX (0x0000001FU)

#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT2_PLL_1P2_HSDIVIDER_CLKOUT2_DIVCHACK_MASK (0x00000020U)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT2_PLL_1P2_HSDIVIDER_CLKOUT2_DIVCHACK_SHIFT (0x00000005U)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT2_PLL_1P2_HSDIVIDER_CLKOUT2_DIVCHACK_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT2_PLL_1P2_HSDIVIDER_CLKOUT2_DIVCHACK_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT2_PLL_1P2_HSDIVIDER_CLKOUT2_GATE_CTRL_MASK (0x00000100U)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT2_PLL_1P2_HSDIVIDER_CLKOUT2_GATE_CTRL_SHIFT (0x00000008U)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT2_PLL_1P2_HSDIVIDER_CLKOUT2_GATE_CTRL_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT2_PLL_1P2_HSDIVIDER_CLKOUT2_GATE_CTRL_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT2_PLL_1P2_HSDIVIDER_CLKOUT2_STATUS_MASK (0x00000200U)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT2_PLL_1P2_HSDIVIDER_CLKOUT2_STATUS_SHIFT (0x00000009U)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT2_PLL_1P2_HSDIVIDER_CLKOUT2_STATUS_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT2_PLL_1P2_HSDIVIDER_CLKOUT2_STATUS_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT2_PLL_1P2_HSDIVIDER_CLKOUT2_PWDN_MASK (0x00001000U)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT2_PLL_1P2_HSDIVIDER_CLKOUT2_PWDN_SHIFT (0x0000000CU)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT2_PLL_1P2_HSDIVIDER_CLKOUT2_PWDN_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT2_PLL_1P2_HSDIVIDER_CLKOUT2_PWDN_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT2_RESETVAL                      (0x00000004U)

/* PLL_1P2_HSDIVIDER_CLKOUT3 */

#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT3_PLL_1P2_HSDIVIDER_CLKOUT3_DIV_MASK (0x0000001FU)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT3_PLL_1P2_HSDIVIDER_CLKOUT3_DIV_SHIFT (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT3_PLL_1P2_HSDIVIDER_CLKOUT3_DIV_RESETVAL (0x00000004U)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT3_PLL_1P2_HSDIVIDER_CLKOUT3_DIV_MAX (0x0000001FU)

#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT3_PLL_1P2_HSDIVIDER_CLKOUT3_DIVCHACK_MASK (0x00000020U)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT3_PLL_1P2_HSDIVIDER_CLKOUT3_DIVCHACK_SHIFT (0x00000005U)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT3_PLL_1P2_HSDIVIDER_CLKOUT3_DIVCHACK_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT3_PLL_1P2_HSDIVIDER_CLKOUT3_DIVCHACK_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT3_PLL_1P2_HSDIVIDER_CLKOUT3_GATE_CTRL_MASK (0x00000100U)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT3_PLL_1P2_HSDIVIDER_CLKOUT3_GATE_CTRL_SHIFT (0x00000008U)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT3_PLL_1P2_HSDIVIDER_CLKOUT3_GATE_CTRL_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT3_PLL_1P2_HSDIVIDER_CLKOUT3_GATE_CTRL_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT3_PLL_1P2_HSDIVIDER_CLKOUT3_STATUS_MASK (0x00000200U)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT3_PLL_1P2_HSDIVIDER_CLKOUT3_STATUS_SHIFT (0x00000009U)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT3_PLL_1P2_HSDIVIDER_CLKOUT3_STATUS_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT3_PLL_1P2_HSDIVIDER_CLKOUT3_STATUS_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT3_PLL_1P2_HSDIVIDER_CLKOUT3_PWDN_MASK (0x00001000U)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT3_PLL_1P2_HSDIVIDER_CLKOUT3_PWDN_SHIFT (0x0000000CU)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT3_PLL_1P2_HSDIVIDER_CLKOUT3_PWDN_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT3_PLL_1P2_HSDIVIDER_CLKOUT3_PWDN_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT3_RESETVAL                      (0x00000004U)

/* PLL_1P2_HSDIVIDER_RSTCTRL */

#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_RSTCTRL_PLL_1P2_HSDIVIDER_RSTCTRL_ASSERT_MASK (0x00000007U)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_RSTCTRL_PLL_1P2_HSDIVIDER_RSTCTRL_ASSERT_SHIFT (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_RSTCTRL_PLL_1P2_HSDIVIDER_RSTCTRL_ASSERT_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_RSTCTRL_PLL_1P2_HSDIVIDER_RSTCTRL_ASSERT_MAX (0x00000007U)

#define CSL_MSS_TOPRCM_PLL_1P2_HSDIVIDER_RSTCTRL_RESETVAL                      (0x00000000U)

/* PLL_1P8_HSDIVIDER */

#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_PLL_1P8_HSDIVIDER_BYPASS_MASK         (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_PLL_1P8_HSDIVIDER_BYPASS_SHIFT        (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_PLL_1P8_HSDIVIDER_BYPASS_RESETVAL     (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_PLL_1P8_HSDIVIDER_BYPASS_MAX          (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_PLL_1P8_HSDIVIDER_LDOPWDN_MASK        (0x00000002U)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_PLL_1P8_HSDIVIDER_LDOPWDN_SHIFT       (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_PLL_1P8_HSDIVIDER_LDOPWDN_RESETVAL    (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_PLL_1P8_HSDIVIDER_LDOPWDN_MAX         (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_PLL_1P8_HSDIVIDER_TENABLEDIV_MASK     (0x00000004U)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_PLL_1P8_HSDIVIDER_TENABLEDIV_SHIFT    (0x00000002U)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_PLL_1P8_HSDIVIDER_TENABLEDIV_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_PLL_1P8_HSDIVIDER_TENABLEDIV_MAX      (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_PLL_1P8_HSDIVIDER_BYPASSACKZ_MASK     (0x00010000U)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_PLL_1P8_HSDIVIDER_BYPASSACKZ_SHIFT    (0x00000010U)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_PLL_1P8_HSDIVIDER_BYPASSACKZ_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_PLL_1P8_HSDIVIDER_BYPASSACKZ_MAX      (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_PLL_1P8_HSDIVIDER_LDOPWDNACK_MASK     (0x00020000U)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_PLL_1P8_HSDIVIDER_LDOPWDNACK_SHIFT    (0x00000011U)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_PLL_1P8_HSDIVIDER_LDOPWDNACK_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_PLL_1P8_HSDIVIDER_LDOPWDNACK_MAX      (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_RESETVAL                              (0x00000000U)

/* PLL_1P8_HSDIVIDER_CLKOUT0 */

#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT0_PLL_1P8_HSDIVIDER_CLKOUT0_DIV_MASK (0x0000001FU)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT0_PLL_1P8_HSDIVIDER_CLKOUT0_DIV_SHIFT (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT0_PLL_1P8_HSDIVIDER_CLKOUT0_DIV_RESETVAL (0x00000004U)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT0_PLL_1P8_HSDIVIDER_CLKOUT0_DIV_MAX (0x0000001FU)

#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT0_PLL_1P8_HSDIVIDER_CLKOUT0_DIVCHACK_MASK (0x00000020U)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT0_PLL_1P8_HSDIVIDER_CLKOUT0_DIVCHACK_SHIFT (0x00000005U)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT0_PLL_1P8_HSDIVIDER_CLKOUT0_DIVCHACK_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT0_PLL_1P8_HSDIVIDER_CLKOUT0_DIVCHACK_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT0_PLL_1P8_HSDIVIDER_CLKOUT0_GATE_CTRL_MASK (0x00000100U)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT0_PLL_1P8_HSDIVIDER_CLKOUT0_GATE_CTRL_SHIFT (0x00000008U)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT0_PLL_1P8_HSDIVIDER_CLKOUT0_GATE_CTRL_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT0_PLL_1P8_HSDIVIDER_CLKOUT0_GATE_CTRL_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT0_PLL_1P8_HSDIVIDER_CLKOUT0_STATUS_MASK (0x00000200U)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT0_PLL_1P8_HSDIVIDER_CLKOUT0_STATUS_SHIFT (0x00000009U)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT0_PLL_1P8_HSDIVIDER_CLKOUT0_STATUS_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT0_PLL_1P8_HSDIVIDER_CLKOUT0_STATUS_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT0_PLL_1P8_HSDIVIDER_CLKOUT0_PWDN_MASK (0x00001000U)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT0_PLL_1P8_HSDIVIDER_CLKOUT0_PWDN_SHIFT (0x0000000CU)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT0_PLL_1P8_HSDIVIDER_CLKOUT0_PWDN_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT0_PLL_1P8_HSDIVIDER_CLKOUT0_PWDN_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT0_RESETVAL                      (0x00000004U)

/* PLL_1P8_HSDIVIDER_CLKOUT1 */

#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT1_PLL_1P8_HSDIVIDER_CLKOUT1_DIV_MASK (0x0000001FU)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT1_PLL_1P8_HSDIVIDER_CLKOUT1_DIV_SHIFT (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT1_PLL_1P8_HSDIVIDER_CLKOUT1_DIV_RESETVAL (0x00000004U)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT1_PLL_1P8_HSDIVIDER_CLKOUT1_DIV_MAX (0x0000001FU)

#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT1_PLL_1P8_HSDIVIDER_CLKOUT1_DIVCHACK_MASK (0x00000020U)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT1_PLL_1P8_HSDIVIDER_CLKOUT1_DIVCHACK_SHIFT (0x00000005U)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT1_PLL_1P8_HSDIVIDER_CLKOUT1_DIVCHACK_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT1_PLL_1P8_HSDIVIDER_CLKOUT1_DIVCHACK_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT1_PLL_1P8_HSDIVIDER_CLKOUT1_GATE_CTRL_MASK (0x00000100U)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT1_PLL_1P8_HSDIVIDER_CLKOUT1_GATE_CTRL_SHIFT (0x00000008U)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT1_PLL_1P8_HSDIVIDER_CLKOUT1_GATE_CTRL_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT1_PLL_1P8_HSDIVIDER_CLKOUT1_GATE_CTRL_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT1_PLL_1P8_HSDIVIDER_CLKOUT1_STATUS_MASK (0x00000200U)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT1_PLL_1P8_HSDIVIDER_CLKOUT1_STATUS_SHIFT (0x00000009U)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT1_PLL_1P8_HSDIVIDER_CLKOUT1_STATUS_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT1_PLL_1P8_HSDIVIDER_CLKOUT1_STATUS_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT1_PLL_1P8_HSDIVIDER_CLKOUT1_PWDN_MASK (0x00001000U)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT1_PLL_1P8_HSDIVIDER_CLKOUT1_PWDN_SHIFT (0x0000000CU)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT1_PLL_1P8_HSDIVIDER_CLKOUT1_PWDN_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT1_PLL_1P8_HSDIVIDER_CLKOUT1_PWDN_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT1_RESETVAL                      (0x00000004U)

/* PLL_1P8_HSDIVIDER_CLKOUT2 */

#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT2_PLL_1P8_HSDIVIDER_CLKOUT2_DIV_MASK (0x0000001FU)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT2_PLL_1P8_HSDIVIDER_CLKOUT2_DIV_SHIFT (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT2_PLL_1P8_HSDIVIDER_CLKOUT2_DIV_RESETVAL (0x00000004U)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT2_PLL_1P8_HSDIVIDER_CLKOUT2_DIV_MAX (0x0000001FU)

#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT2_PLL_1P8_HSDIVIDER_CLKOUT2_DIVCHACK_MASK (0x00000020U)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT2_PLL_1P8_HSDIVIDER_CLKOUT2_DIVCHACK_SHIFT (0x00000005U)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT2_PLL_1P8_HSDIVIDER_CLKOUT2_DIVCHACK_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT2_PLL_1P8_HSDIVIDER_CLKOUT2_DIVCHACK_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT2_PLL_1P8_HSDIVIDER_CLKOUT2_GATE_CTRL_MASK (0x00000100U)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT2_PLL_1P8_HSDIVIDER_CLKOUT2_GATE_CTRL_SHIFT (0x00000008U)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT2_PLL_1P8_HSDIVIDER_CLKOUT2_GATE_CTRL_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT2_PLL_1P8_HSDIVIDER_CLKOUT2_GATE_CTRL_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT2_PLL_1P8_HSDIVIDER_CLKOUT2_STATUS_MASK (0x00000200U)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT2_PLL_1P8_HSDIVIDER_CLKOUT2_STATUS_SHIFT (0x00000009U)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT2_PLL_1P8_HSDIVIDER_CLKOUT2_STATUS_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT2_PLL_1P8_HSDIVIDER_CLKOUT2_STATUS_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT2_PLL_1P8_HSDIVIDER_CLKOUT2_PWDN_MASK (0x00001000U)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT2_PLL_1P8_HSDIVIDER_CLKOUT2_PWDN_SHIFT (0x0000000CU)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT2_PLL_1P8_HSDIVIDER_CLKOUT2_PWDN_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT2_PLL_1P8_HSDIVIDER_CLKOUT2_PWDN_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT2_RESETVAL                      (0x00000004U)

/* PLL_1P8_HSDIVIDER_CLKOUT3 */

#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT3_PLL_1P8_HSDIVIDER_CLKOUT3_DIV_MASK (0x0000001FU)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT3_PLL_1P8_HSDIVIDER_CLKOUT3_DIV_SHIFT (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT3_PLL_1P8_HSDIVIDER_CLKOUT3_DIV_RESETVAL (0x00000004U)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT3_PLL_1P8_HSDIVIDER_CLKOUT3_DIV_MAX (0x0000001FU)

#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT3_PLL_1P8_HSDIVIDER_CLKOUT3_DIVCHACK_MASK (0x00000020U)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT3_PLL_1P8_HSDIVIDER_CLKOUT3_DIVCHACK_SHIFT (0x00000005U)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT3_PLL_1P8_HSDIVIDER_CLKOUT3_DIVCHACK_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT3_PLL_1P8_HSDIVIDER_CLKOUT3_DIVCHACK_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT3_PLL_1P8_HSDIVIDER_CLKOUT3_GATE_CTRL_MASK (0x00000100U)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT3_PLL_1P8_HSDIVIDER_CLKOUT3_GATE_CTRL_SHIFT (0x00000008U)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT3_PLL_1P8_HSDIVIDER_CLKOUT3_GATE_CTRL_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT3_PLL_1P8_HSDIVIDER_CLKOUT3_GATE_CTRL_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT3_PLL_1P8_HSDIVIDER_CLKOUT3_STATUS_MASK (0x00000200U)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT3_PLL_1P8_HSDIVIDER_CLKOUT3_STATUS_SHIFT (0x00000009U)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT3_PLL_1P8_HSDIVIDER_CLKOUT3_STATUS_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT3_PLL_1P8_HSDIVIDER_CLKOUT3_STATUS_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT3_PLL_1P8_HSDIVIDER_CLKOUT3_PWDN_MASK (0x00001000U)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT3_PLL_1P8_HSDIVIDER_CLKOUT3_PWDN_SHIFT (0x0000000CU)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT3_PLL_1P8_HSDIVIDER_CLKOUT3_PWDN_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT3_PLL_1P8_HSDIVIDER_CLKOUT3_PWDN_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT3_RESETVAL                      (0x00000004U)

/* PLL_1P8_HSDIVIDER_RSTCTRL */

#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_RSTCTRL_PLL_1P8_HSDIVIDER_RSTCTRL_ASSERT_MASK (0x00000007U)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_RSTCTRL_PLL_1P8_HSDIVIDER_RSTCTRL_ASSERT_SHIFT (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_RSTCTRL_PLL_1P8_HSDIVIDER_RSTCTRL_ASSERT_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_RSTCTRL_PLL_1P8_HSDIVIDER_RSTCTRL_ASSERT_MAX (0x00000007U)

#define CSL_MSS_TOPRCM_PLL_1P8_HSDIVIDER_RSTCTRL_RESETVAL                      (0x00000000U)

/* PLL_DSP_PWRCTRL */

#define CSL_MSS_TOPRCM_PLL_DSP_PWRCTRL_PLL_DSP_PWRCTRL_OFFMODE_MASK            (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_DSP_PWRCTRL_PLL_DSP_PWRCTRL_OFFMODE_SHIFT           (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_PWRCTRL_PLL_DSP_PWRCTRL_OFFMODE_RESETVAL        (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_PWRCTRL_PLL_DSP_PWRCTRL_OFFMODE_MAX             (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_DSP_PWRCTRL_PLL_DSP_PWRCTRL_ISOSCAN_MASK            (0x00000002U)
#define CSL_MSS_TOPRCM_PLL_DSP_PWRCTRL_PLL_DSP_PWRCTRL_ISOSCAN_SHIFT           (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_DSP_PWRCTRL_PLL_DSP_PWRCTRL_ISOSCAN_RESETVAL        (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_PWRCTRL_PLL_DSP_PWRCTRL_ISOSCAN_MAX             (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_DSP_PWRCTRL_PLL_DSP_PWRCTRL_ISORET_MASK             (0x00000004U)
#define CSL_MSS_TOPRCM_PLL_DSP_PWRCTRL_PLL_DSP_PWRCTRL_ISORET_SHIFT            (0x00000002U)
#define CSL_MSS_TOPRCM_PLL_DSP_PWRCTRL_PLL_DSP_PWRCTRL_ISORET_RESETVAL         (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_PWRCTRL_PLL_DSP_PWRCTRL_ISORET_MAX              (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_DSP_PWRCTRL_PLL_DSP_PWRCTRL_RET_MASK                (0x00000008U)
#define CSL_MSS_TOPRCM_PLL_DSP_PWRCTRL_PLL_DSP_PWRCTRL_RET_SHIFT               (0x00000003U)
#define CSL_MSS_TOPRCM_PLL_DSP_PWRCTRL_PLL_DSP_PWRCTRL_RET_RESETVAL            (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_PWRCTRL_PLL_DSP_PWRCTRL_RET_MAX                 (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_DSP_PWRCTRL_PLL_DSP_PWRCTRL_PGOODIN_MASK            (0x00000010U)
#define CSL_MSS_TOPRCM_PLL_DSP_PWRCTRL_PLL_DSP_PWRCTRL_PGOODIN_SHIFT           (0x00000004U)
#define CSL_MSS_TOPRCM_PLL_DSP_PWRCTRL_PLL_DSP_PWRCTRL_PGOODIN_RESETVAL        (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_DSP_PWRCTRL_PLL_DSP_PWRCTRL_PGOODIN_MAX             (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_DSP_PWRCTRL_PLL_DSP_PWRCTRL_PONIN_MASK              (0x00000020U)
#define CSL_MSS_TOPRCM_PLL_DSP_PWRCTRL_PLL_DSP_PWRCTRL_PONIN_SHIFT             (0x00000005U)
#define CSL_MSS_TOPRCM_PLL_DSP_PWRCTRL_PLL_DSP_PWRCTRL_PONIN_RESETVAL          (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_DSP_PWRCTRL_PLL_DSP_PWRCTRL_PONIN_MAX               (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_DSP_PWRCTRL_RESETVAL                                (0x00000030U)

/* PLL_DSP_CLKCTRL */

#define CSL_MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_TINTZ_MASK              (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_TINTZ_SHIFT             (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_TINTZ_RESETVAL          (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_TINTZ_MAX               (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_SSCTYPE_MASK            (0x00000002U)
#define CSL_MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_SSCTYPE_SHIFT           (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_SSCTYPE_RESETVAL        (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_SSCTYPE_MAX             (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_RELAXED_LOCK_MASK       (0x00000100U)
#define CSL_MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_RELAXED_LOCK_SHIFT      (0x00000008U)
#define CSL_MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_RELAXED_LOCK_RESETVAL   (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_RELAXED_LOCK_MAX        (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_SELFREQDCO_MASK         (0x00001C00U)
#define CSL_MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_SELFREQDCO_SHIFT        (0x0000000AU)
#define CSL_MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_SELFREQDCO_RESETVAL     (0x00000002U)
#define CSL_MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_SELFREQDCO_MAX          (0x00000007U)

#define CSL_MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_STOPMODE_MASK           (0x00004000U)
#define CSL_MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_STOPMODE_SHIFT          (0x0000000EU)
#define CSL_MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_STOPMODE_RESETVAL       (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_STOPMODE_MAX            (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_M2PWDNZ_MASK            (0x00010000U)
#define CSL_MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_M2PWDNZ_SHIFT           (0x00000010U)
#define CSL_MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_M2PWDNZ_RESETVAL        (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_M2PWDNZ_MAX             (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_CLKDCOLDOPWDNZ_MASK     (0x00020000U)
#define CSL_MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_CLKDCOLDOPWDNZ_SHIFT    (0x00000011U)
#define CSL_MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_CLKDCOLDOPWDNZ_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_CLKDCOLDOPWDNZ_MAX      (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_ULOWCLKEN_MASK          (0x00040000U)
#define CSL_MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_ULOWCLKEN_SHIFT         (0x00000012U)
#define CSL_MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_ULOWCLKEN_RESETVAL      (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_ULOWCLKEN_MAX           (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_CLKOUTLDOEN_MASK        (0x00080000U)
#define CSL_MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_CLKOUTLDOEN_SHIFT       (0x00000013U)
#define CSL_MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_CLKOUTLDOEN_RESETVAL    (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_CLKOUTLDOEN_MAX         (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_CLKOUTEN_MASK           (0x00100000U)
#define CSL_MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_CLKOUTEN_SHIFT          (0x00000014U)
#define CSL_MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_CLKOUTEN_RESETVAL       (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_CLKOUTEN_MAX            (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_STBYRET_MASK            (0x00200000U)
#define CSL_MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_STBYRET_SHIFT           (0x00000015U)
#define CSL_MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_STBYRET_RESETVAL        (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_STBYRET_MAX             (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_BYPASSACKZ_MASK         (0x00400000U)
#define CSL_MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_BYPASSACKZ_SHIFT        (0x00000016U)
#define CSL_MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_BYPASSACKZ_RESETVAL     (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_BYPASSACKZ_MAX          (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_IDLE_MASK               (0x00800000U)
#define CSL_MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_IDLE_SHIFT              (0x00000017U)
#define CSL_MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_IDLE_RESETVAL           (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_IDLE_MAX                (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_NWELLTRIM_MASK          (0x1F000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_NWELLTRIM_SHIFT         (0x00000018U)
#define CSL_MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_NWELLTRIM_RESETVAL      (0x00000009U)
#define CSL_MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_NWELLTRIM_MAX           (0x0000001FU)

#define CSL_MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_CLKDCOLDOEN_MASK        (0x20000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_CLKDCOLDOEN_SHIFT       (0x0000001DU)
#define CSL_MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_CLKDCOLDOEN_RESETVAL    (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_CLKDCOLDOEN_MAX         (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_ENSSC_MASK              (0x40000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_ENSSC_SHIFT             (0x0000001EU)
#define CSL_MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_ENSSC_RESETVAL          (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_ENSSC_MAX               (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_CYCLESLIPEN_MASK        (0x80000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_CYCLESLIPEN_SHIFT       (0x0000001FU)
#define CSL_MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_CYCLESLIPEN_RESETVAL    (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_CYCLESLIPEN_MAX         (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_DSP_CLKCTRL_RESETVAL                                (0x09914800U)

/* PLL_DSP_TENABLE */

#define CSL_MSS_TOPRCM_PLL_DSP_TENABLE_PLL_DSP_TENABLE_TENABLE_MASK            (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_DSP_TENABLE_PLL_DSP_TENABLE_TENABLE_SHIFT           (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_TENABLE_PLL_DSP_TENABLE_TENABLE_RESETVAL        (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_TENABLE_PLL_DSP_TENABLE_TENABLE_MAX             (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_DSP_TENABLE_RESETVAL                                (0x00000000U)

/* PLL_DSP_TENABLEDIV */

#define CSL_MSS_TOPRCM_PLL_DSP_TENABLEDIV_PLL_DSP_TENABLEDIV_TENABLEDIV_MASK   (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_DSP_TENABLEDIV_PLL_DSP_TENABLEDIV_TENABLEDIV_SHIFT  (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_TENABLEDIV_PLL_DSP_TENABLEDIV_TENABLEDIV_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_TENABLEDIV_PLL_DSP_TENABLEDIV_TENABLEDIV_MAX    (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_DSP_TENABLEDIV_RESETVAL                             (0x00000000U)

/* PLL_DSP_M2NDIV */

#define CSL_MSS_TOPRCM_PLL_DSP_M2NDIV_PLL_DSP_M2NDIV_N_MASK                    (0x000000FFU)
#define CSL_MSS_TOPRCM_PLL_DSP_M2NDIV_PLL_DSP_M2NDIV_N_SHIFT                   (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_M2NDIV_PLL_DSP_M2NDIV_N_RESETVAL                (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_M2NDIV_PLL_DSP_M2NDIV_N_MAX                     (0x000000FFU)

#define CSL_MSS_TOPRCM_PLL_DSP_M2NDIV_PLL_DSP_M2NDIV_M2_MASK                   (0x007F0000U)
#define CSL_MSS_TOPRCM_PLL_DSP_M2NDIV_PLL_DSP_M2NDIV_M2_SHIFT                  (0x00000010U)
#define CSL_MSS_TOPRCM_PLL_DSP_M2NDIV_PLL_DSP_M2NDIV_M2_RESETVAL               (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_M2NDIV_PLL_DSP_M2NDIV_M2_MAX                    (0x0000007FU)

#define CSL_MSS_TOPRCM_PLL_DSP_M2NDIV_RESETVAL                                 (0x00000000U)

/* PLL_DSP_MN2DIV */

#define CSL_MSS_TOPRCM_PLL_DSP_MN2DIV_PLL_DSP_MN2DIV_M_MASK                    (0x00000FFFU)
#define CSL_MSS_TOPRCM_PLL_DSP_MN2DIV_PLL_DSP_MN2DIV_M_SHIFT                   (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_MN2DIV_PLL_DSP_MN2DIV_M_RESETVAL                (0x00000174U)
#define CSL_MSS_TOPRCM_PLL_DSP_MN2DIV_PLL_DSP_MN2DIV_M_MAX                     (0x00000FFFU)

#define CSL_MSS_TOPRCM_PLL_DSP_MN2DIV_PLL_DSP_MN2DIV_N2_MASK                   (0x000F0000U)
#define CSL_MSS_TOPRCM_PLL_DSP_MN2DIV_PLL_DSP_MN2DIV_N2_SHIFT                  (0x00000010U)
#define CSL_MSS_TOPRCM_PLL_DSP_MN2DIV_PLL_DSP_MN2DIV_N2_RESETVAL               (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_MN2DIV_PLL_DSP_MN2DIV_N2_MAX                    (0x0000000FU)

#define CSL_MSS_TOPRCM_PLL_DSP_MN2DIV_RESETVAL                                 (0x00000174U)

/* PLL_DSP_FRACDIV */

#define CSL_MSS_TOPRCM_PLL_DSP_FRACDIV_PLL_DSP_FRACDIV_FRACTIONALM_MASK        (0x0003FFFFU)
#define CSL_MSS_TOPRCM_PLL_DSP_FRACDIV_PLL_DSP_FRACDIV_FRACTIONALM_SHIFT       (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_FRACDIV_PLL_DSP_FRACDIV_FRACTIONALM_RESETVAL    (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_FRACDIV_PLL_DSP_FRACDIV_FRACTIONALM_MAX         (0x0003FFFFU)

#define CSL_MSS_TOPRCM_PLL_DSP_FRACDIV_PLL_DSP_FRACDIV_REGSD_MASK              (0xFF000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_FRACDIV_PLL_DSP_FRACDIV_REGSD_SHIFT             (0x00000018U)
#define CSL_MSS_TOPRCM_PLL_DSP_FRACDIV_PLL_DSP_FRACDIV_REGSD_RESETVAL          (0x00000008U)
#define CSL_MSS_TOPRCM_PLL_DSP_FRACDIV_PLL_DSP_FRACDIV_REGSD_MAX               (0x000000FFU)

#define CSL_MSS_TOPRCM_PLL_DSP_FRACDIV_RESETVAL                                (0x08000000U)

/* PLL_DSP_BWCTRL */

#define CSL_MSS_TOPRCM_PLL_DSP_BWCTRL_PLL_DSP_BWCTRL_BW_INCR_DECRZ_MASK        (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_DSP_BWCTRL_PLL_DSP_BWCTRL_BW_INCR_DECRZ_SHIFT       (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_BWCTRL_PLL_DSP_BWCTRL_BW_INCR_DECRZ_RESETVAL    (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_BWCTRL_PLL_DSP_BWCTRL_BW_INCR_DECRZ_MAX         (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_DSP_BWCTRL_PLL_DSP_BWCTRL_BWCONTROL_MASK            (0x00000006U)
#define CSL_MSS_TOPRCM_PLL_DSP_BWCTRL_PLL_DSP_BWCTRL_BWCONTROL_SHIFT           (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_DSP_BWCTRL_PLL_DSP_BWCTRL_BWCONTROL_RESETVAL        (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_BWCTRL_PLL_DSP_BWCTRL_BWCONTROL_MAX             (0x00000003U)

#define CSL_MSS_TOPRCM_PLL_DSP_BWCTRL_RESETVAL                                 (0x00000000U)

/* PLL_DSP_FRACCTRL */

#define CSL_MSS_TOPRCM_PLL_DSP_FRACCTRL_PLL_DSP_FRACCTRL_DELTAMSTEPFRACTION_MASK (0x0003FFFFU)
#define CSL_MSS_TOPRCM_PLL_DSP_FRACCTRL_PLL_DSP_FRACCTRL_DELTAMSTEPFRACTION_SHIFT (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_FRACCTRL_PLL_DSP_FRACCTRL_DELTAMSTEPFRACTION_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_FRACCTRL_PLL_DSP_FRACCTRL_DELTAMSTEPFRACTION_MAX (0x0003FFFFU)

#define CSL_MSS_TOPRCM_PLL_DSP_FRACCTRL_PLL_DSP_FRACCTRL_DELTAMSTEPINTEGER_MASK (0x001C0000U)
#define CSL_MSS_TOPRCM_PLL_DSP_FRACCTRL_PLL_DSP_FRACCTRL_DELTAMSTEPINTEGER_SHIFT (0x00000012U)
#define CSL_MSS_TOPRCM_PLL_DSP_FRACCTRL_PLL_DSP_FRACCTRL_DELTAMSTEPINTEGER_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_FRACCTRL_PLL_DSP_FRACCTRL_DELTAMSTEPINTEGER_MAX (0x00000007U)

#define CSL_MSS_TOPRCM_PLL_DSP_FRACCTRL_PLL_DSP_FRACCTRL_MODFREQDIVIDERMANTISSA_MASK (0x0FE00000U)
#define CSL_MSS_TOPRCM_PLL_DSP_FRACCTRL_PLL_DSP_FRACCTRL_MODFREQDIVIDERMANTISSA_SHIFT (0x00000015U)
#define CSL_MSS_TOPRCM_PLL_DSP_FRACCTRL_PLL_DSP_FRACCTRL_MODFREQDIVIDERMANTISSA_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_FRACCTRL_PLL_DSP_FRACCTRL_MODFREQDIVIDERMANTISSA_MAX (0x0000007FU)

#define CSL_MSS_TOPRCM_PLL_DSP_FRACCTRL_PLL_DSP_FRACCTRL_MODFREQDIVIDEREXPONENT_MASK (0x70000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_FRACCTRL_PLL_DSP_FRACCTRL_MODFREQDIVIDEREXPONENT_SHIFT (0x0000001CU)
#define CSL_MSS_TOPRCM_PLL_DSP_FRACCTRL_PLL_DSP_FRACCTRL_MODFREQDIVIDEREXPONENT_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_FRACCTRL_PLL_DSP_FRACCTRL_MODFREQDIVIDEREXPONENT_MAX (0x00000007U)

#define CSL_MSS_TOPRCM_PLL_DSP_FRACCTRL_PLL_DSP_FRACCTRL_DOWNSPREAD_MASK       (0x80000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_FRACCTRL_PLL_DSP_FRACCTRL_DOWNSPREAD_SHIFT      (0x0000001FU)
#define CSL_MSS_TOPRCM_PLL_DSP_FRACCTRL_PLL_DSP_FRACCTRL_DOWNSPREAD_RESETVAL   (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_FRACCTRL_PLL_DSP_FRACCTRL_DOWNSPREAD_MAX        (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_DSP_FRACCTRL_RESETVAL                               (0x00000000U)

/* PLL_DSP_STATUS */

#define CSL_MSS_TOPRCM_PLL_DSP_STATUS_PLL_DSP_STATUS_BYPASS_MASK               (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_DSP_STATUS_PLL_DSP_STATUS_BYPASS_SHIFT              (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_STATUS_PLL_DSP_STATUS_BYPASS_RESETVAL           (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_DSP_STATUS_PLL_DSP_STATUS_BYPASS_MAX                (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_DSP_STATUS_PLL_DSP_STATUS_HIGHJITTER_MASK           (0x00000002U)
#define CSL_MSS_TOPRCM_PLL_DSP_STATUS_PLL_DSP_STATUS_HIGHJITTER_SHIFT          (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_DSP_STATUS_PLL_DSP_STATUS_HIGHJITTER_RESETVAL       (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_STATUS_PLL_DSP_STATUS_HIGHJITTER_MAX            (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_DSP_STATUS_PLL_DSP_STATUS_SSCACK_MASK               (0x00000004U)
#define CSL_MSS_TOPRCM_PLL_DSP_STATUS_PLL_DSP_STATUS_SSCACK_SHIFT              (0x00000002U)
#define CSL_MSS_TOPRCM_PLL_DSP_STATUS_PLL_DSP_STATUS_SSCACK_RESETVAL           (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_STATUS_PLL_DSP_STATUS_SSCACK_MAX                (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_DSP_STATUS_PLL_DSP_STATUS_M2CHANGEACK_MASK          (0x00000008U)
#define CSL_MSS_TOPRCM_PLL_DSP_STATUS_PLL_DSP_STATUS_M2CHANGEACK_SHIFT         (0x00000003U)
#define CSL_MSS_TOPRCM_PLL_DSP_STATUS_PLL_DSP_STATUS_M2CHANGEACK_RESETVAL      (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_STATUS_PLL_DSP_STATUS_M2CHANGEACK_MAX           (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_DSP_STATUS_PLL_DSP_STATUS_LOCK2_MASK                (0x00000010U)
#define CSL_MSS_TOPRCM_PLL_DSP_STATUS_PLL_DSP_STATUS_LOCK2_SHIFT               (0x00000004U)
#define CSL_MSS_TOPRCM_PLL_DSP_STATUS_PLL_DSP_STATUS_LOCK2_RESETVAL            (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_STATUS_PLL_DSP_STATUS_LOCK2_MAX                 (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_DSP_STATUS_PLL_DSP_STATUS_CLKOUTENACK_MASK          (0x00000020U)
#define CSL_MSS_TOPRCM_PLL_DSP_STATUS_PLL_DSP_STATUS_CLKOUTENACK_SHIFT         (0x00000005U)
#define CSL_MSS_TOPRCM_PLL_DSP_STATUS_PLL_DSP_STATUS_CLKOUTENACK_RESETVAL      (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_DSP_STATUS_PLL_DSP_STATUS_CLKOUTENACK_MAX           (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_DSP_STATUS_PLL_DSP_STATUS_LOSSREF_MASK              (0x00000040U)
#define CSL_MSS_TOPRCM_PLL_DSP_STATUS_PLL_DSP_STATUS_LOSSREF_SHIFT             (0x00000006U)
#define CSL_MSS_TOPRCM_PLL_DSP_STATUS_PLL_DSP_STATUS_LOSSREF_RESETVAL          (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_DSP_STATUS_PLL_DSP_STATUS_LOSSREF_MAX               (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_DSP_STATUS_PLL_DSP_STATUS_STBYRETACK_MASK           (0x00000080U)
#define CSL_MSS_TOPRCM_PLL_DSP_STATUS_PLL_DSP_STATUS_STBYRETACK_SHIFT          (0x00000007U)
#define CSL_MSS_TOPRCM_PLL_DSP_STATUS_PLL_DSP_STATUS_STBYRETACK_RESETVAL       (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_STATUS_PLL_DSP_STATUS_STBYRETACK_MAX            (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_DSP_STATUS_PLL_DSP_STATUS_BYPASSACK_MASK            (0x00000100U)
#define CSL_MSS_TOPRCM_PLL_DSP_STATUS_PLL_DSP_STATUS_BYPASSACK_SHIFT           (0x00000008U)
#define CSL_MSS_TOPRCM_PLL_DSP_STATUS_PLL_DSP_STATUS_BYPASSACK_RESETVAL        (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_DSP_STATUS_PLL_DSP_STATUS_BYPASSACK_MAX             (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_DSP_STATUS_PLL_DSP_STATUS_FREQLOCK_MASK             (0x00000200U)
#define CSL_MSS_TOPRCM_PLL_DSP_STATUS_PLL_DSP_STATUS_FREQLOCK_SHIFT            (0x00000009U)
#define CSL_MSS_TOPRCM_PLL_DSP_STATUS_PLL_DSP_STATUS_FREQLOCK_RESETVAL         (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_STATUS_PLL_DSP_STATUS_FREQLOCK_MAX              (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_DSP_STATUS_PLL_DSP_STATUS_PHASELOCK_MASK            (0x00000400U)
#define CSL_MSS_TOPRCM_PLL_DSP_STATUS_PLL_DSP_STATUS_PHASELOCK_SHIFT           (0x0000000AU)
#define CSL_MSS_TOPRCM_PLL_DSP_STATUS_PLL_DSP_STATUS_PHASELOCK_RESETVAL        (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_STATUS_PLL_DSP_STATUS_PHASELOCK_MAX             (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_DSP_STATUS_PLL_DSP_STATUS_CLKDCOLDOACK_MASK         (0x00000800U)
#define CSL_MSS_TOPRCM_PLL_DSP_STATUS_PLL_DSP_STATUS_CLKDCOLDOACK_SHIFT        (0x0000000BU)
#define CSL_MSS_TOPRCM_PLL_DSP_STATUS_PLL_DSP_STATUS_CLKDCOLDOACK_RESETVAL     (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_STATUS_PLL_DSP_STATUS_CLKDCOLDOACK_MAX          (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_DSP_STATUS_PLL_DSP_STATUS_RECAL_OPPIN_MASK          (0x08000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_STATUS_PLL_DSP_STATUS_RECAL_OPPIN_SHIFT         (0x0000001BU)
#define CSL_MSS_TOPRCM_PLL_DSP_STATUS_PLL_DSP_STATUS_RECAL_OPPIN_RESETVAL      (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_STATUS_PLL_DSP_STATUS_RECAL_OPPIN_MAX           (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_DSP_STATUS_PLL_DSP_STATUS_RECAL_BSTATUS3_MASK       (0x10000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_STATUS_PLL_DSP_STATUS_RECAL_BSTATUS3_SHIFT      (0x0000001CU)
#define CSL_MSS_TOPRCM_PLL_DSP_STATUS_PLL_DSP_STATUS_RECAL_BSTATUS3_RESETVAL   (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_STATUS_PLL_DSP_STATUS_RECAL_BSTATUS3_MAX        (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_DSP_STATUS_PLL_DSP_STATUS_LDOPWDN_MASK              (0x20000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_STATUS_PLL_DSP_STATUS_LDOPWDN_SHIFT             (0x0000001DU)
#define CSL_MSS_TOPRCM_PLL_DSP_STATUS_PLL_DSP_STATUS_LDOPWDN_RESETVAL          (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_DSP_STATUS_PLL_DSP_STATUS_LDOPWDN_MAX               (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_DSP_STATUS_PLL_DSP_STATUS_PGOODOUT_MASK             (0x40000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_STATUS_PLL_DSP_STATUS_PGOODOUT_SHIFT            (0x0000001EU)
#define CSL_MSS_TOPRCM_PLL_DSP_STATUS_PLL_DSP_STATUS_PGOODOUT_RESETVAL         (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_DSP_STATUS_PLL_DSP_STATUS_PGOODOUT_MAX              (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_DSP_STATUS_PLL_DSP_STATUS_PONOUT_MASK               (0x80000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_STATUS_PLL_DSP_STATUS_PONOUT_SHIFT              (0x0000001FU)
#define CSL_MSS_TOPRCM_PLL_DSP_STATUS_PLL_DSP_STATUS_PONOUT_RESETVAL           (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_DSP_STATUS_PLL_DSP_STATUS_PONOUT_MAX                (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_DSP_STATUS_RESETVAL                                 (0xE0000161U)

/* PLL_DSP_HSDIVIDER */

#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_PLL_DSP_HSDIVIDER_BYPASS_MASK         (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_PLL_DSP_HSDIVIDER_BYPASS_SHIFT        (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_PLL_DSP_HSDIVIDER_BYPASS_RESETVAL     (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_PLL_DSP_HSDIVIDER_BYPASS_MAX          (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_PLL_DSP_HSDIVIDER_LDOPWDN_MASK        (0x00000002U)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_PLL_DSP_HSDIVIDER_LDOPWDN_SHIFT       (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_PLL_DSP_HSDIVIDER_LDOPWDN_RESETVAL    (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_PLL_DSP_HSDIVIDER_LDOPWDN_MAX         (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_PLL_DSP_HSDIVIDER_TENABLEDIV_MASK     (0x00000004U)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_PLL_DSP_HSDIVIDER_TENABLEDIV_SHIFT    (0x00000002U)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_PLL_DSP_HSDIVIDER_TENABLEDIV_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_PLL_DSP_HSDIVIDER_TENABLEDIV_MAX      (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_PLL_DSP_HSDIVIDER_BYPASSACKZ_MASK     (0x00010000U)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_PLL_DSP_HSDIVIDER_BYPASSACKZ_SHIFT    (0x00000010U)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_PLL_DSP_HSDIVIDER_BYPASSACKZ_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_PLL_DSP_HSDIVIDER_BYPASSACKZ_MAX      (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_PLL_DSP_HSDIVIDER_LDOPWDNACK_MASK     (0x00020000U)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_PLL_DSP_HSDIVIDER_LDOPWDNACK_SHIFT    (0x00000011U)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_PLL_DSP_HSDIVIDER_LDOPWDNACK_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_PLL_DSP_HSDIVIDER_LDOPWDNACK_MAX      (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_RESETVAL                              (0x00000000U)

/* PLL_DSP_HSDIVIDER_CLKOUT0 */

#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT0_PLL_DSP_HSDIVIDER_CLKOUT0_DIV_MASK (0x0000001FU)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT0_PLL_DSP_HSDIVIDER_CLKOUT0_DIV_SHIFT (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT0_PLL_DSP_HSDIVIDER_CLKOUT0_DIV_RESETVAL (0x00000004U)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT0_PLL_DSP_HSDIVIDER_CLKOUT0_DIV_MAX (0x0000001FU)

#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT0_PLL_DSP_HSDIVIDER_CLKOUT0_DIVCHACK_MASK (0x00000020U)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT0_PLL_DSP_HSDIVIDER_CLKOUT0_DIVCHACK_SHIFT (0x00000005U)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT0_PLL_DSP_HSDIVIDER_CLKOUT0_DIVCHACK_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT0_PLL_DSP_HSDIVIDER_CLKOUT0_DIVCHACK_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT0_PLL_DSP_HSDIVIDER_CLKOUT0_GATE_CTRL_MASK (0x00000100U)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT0_PLL_DSP_HSDIVIDER_CLKOUT0_GATE_CTRL_SHIFT (0x00000008U)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT0_PLL_DSP_HSDIVIDER_CLKOUT0_GATE_CTRL_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT0_PLL_DSP_HSDIVIDER_CLKOUT0_GATE_CTRL_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT0_PLL_DSP_HSDIVIDER_CLKOUT0_STATUS_MASK (0x00000200U)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT0_PLL_DSP_HSDIVIDER_CLKOUT0_STATUS_SHIFT (0x00000009U)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT0_PLL_DSP_HSDIVIDER_CLKOUT0_STATUS_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT0_PLL_DSP_HSDIVIDER_CLKOUT0_STATUS_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT0_PLL_DSP_HSDIVIDER_CLKOUT0_PWDN_MASK (0x00001000U)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT0_PLL_DSP_HSDIVIDER_CLKOUT0_PWDN_SHIFT (0x0000000CU)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT0_PLL_DSP_HSDIVIDER_CLKOUT0_PWDN_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT0_PLL_DSP_HSDIVIDER_CLKOUT0_PWDN_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT0_RESETVAL                      (0x00000004U)

/* PLL_DSP_HSDIVIDER_CLKOUT1 */

#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT1_PLL_DSP_HSDIVIDER_CLKOUT1_DIV_MASK (0x0000001FU)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT1_PLL_DSP_HSDIVIDER_CLKOUT1_DIV_SHIFT (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT1_PLL_DSP_HSDIVIDER_CLKOUT1_DIV_RESETVAL (0x00000004U)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT1_PLL_DSP_HSDIVIDER_CLKOUT1_DIV_MAX (0x0000001FU)

#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT1_PLL_DSP_HSDIVIDER_CLKOUT1_DIVCHACK_MASK (0x00000020U)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT1_PLL_DSP_HSDIVIDER_CLKOUT1_DIVCHACK_SHIFT (0x00000005U)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT1_PLL_DSP_HSDIVIDER_CLKOUT1_DIVCHACK_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT1_PLL_DSP_HSDIVIDER_CLKOUT1_DIVCHACK_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT1_PLL_DSP_HSDIVIDER_CLKOUT1_GATE_CTRL_MASK (0x00000100U)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT1_PLL_DSP_HSDIVIDER_CLKOUT1_GATE_CTRL_SHIFT (0x00000008U)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT1_PLL_DSP_HSDIVIDER_CLKOUT1_GATE_CTRL_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT1_PLL_DSP_HSDIVIDER_CLKOUT1_GATE_CTRL_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT1_PLL_DSP_HSDIVIDER_CLKOUT1_STATUS_MASK (0x00000200U)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT1_PLL_DSP_HSDIVIDER_CLKOUT1_STATUS_SHIFT (0x00000009U)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT1_PLL_DSP_HSDIVIDER_CLKOUT1_STATUS_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT1_PLL_DSP_HSDIVIDER_CLKOUT1_STATUS_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT1_PLL_DSP_HSDIVIDER_CLKOUT1_PWDN_MASK (0x00001000U)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT1_PLL_DSP_HSDIVIDER_CLKOUT1_PWDN_SHIFT (0x0000000CU)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT1_PLL_DSP_HSDIVIDER_CLKOUT1_PWDN_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT1_PLL_DSP_HSDIVIDER_CLKOUT1_PWDN_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT1_RESETVAL                      (0x00000004U)

/* PLL_DSP_HSDIVIDER_CLKOUT2 */

#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT2_PLL_DSP_HSDIVIDER_CLKOUT2_DIV_MASK (0x0000001FU)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT2_PLL_DSP_HSDIVIDER_CLKOUT2_DIV_SHIFT (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT2_PLL_DSP_HSDIVIDER_CLKOUT2_DIV_RESETVAL (0x00000004U)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT2_PLL_DSP_HSDIVIDER_CLKOUT2_DIV_MAX (0x0000001FU)

#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT2_PLL_DSP_HSDIVIDER_CLKOUT2_DIVCHACK_MASK (0x00000020U)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT2_PLL_DSP_HSDIVIDER_CLKOUT2_DIVCHACK_SHIFT (0x00000005U)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT2_PLL_DSP_HSDIVIDER_CLKOUT2_DIVCHACK_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT2_PLL_DSP_HSDIVIDER_CLKOUT2_DIVCHACK_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT2_PLL_DSP_HSDIVIDER_CLKOUT2_GATE_CTRL_MASK (0x00000100U)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT2_PLL_DSP_HSDIVIDER_CLKOUT2_GATE_CTRL_SHIFT (0x00000008U)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT2_PLL_DSP_HSDIVIDER_CLKOUT2_GATE_CTRL_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT2_PLL_DSP_HSDIVIDER_CLKOUT2_GATE_CTRL_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT2_PLL_DSP_HSDIVIDER_CLKOUT2_STATUS_MASK (0x00000200U)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT2_PLL_DSP_HSDIVIDER_CLKOUT2_STATUS_SHIFT (0x00000009U)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT2_PLL_DSP_HSDIVIDER_CLKOUT2_STATUS_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT2_PLL_DSP_HSDIVIDER_CLKOUT2_STATUS_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT2_PLL_DSP_HSDIVIDER_CLKOUT2_PWDN_MASK (0x00001000U)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT2_PLL_DSP_HSDIVIDER_CLKOUT2_PWDN_SHIFT (0x0000000CU)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT2_PLL_DSP_HSDIVIDER_CLKOUT2_PWDN_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT2_PLL_DSP_HSDIVIDER_CLKOUT2_PWDN_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT2_RESETVAL                      (0x00000004U)

/* PLL_DSP_HSDIVIDER_CLKOUT3 */

#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT3_PLL_DSP_HSDIVIDER_CLKOUT3_DIV_MASK (0x0000001FU)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT3_PLL_DSP_HSDIVIDER_CLKOUT3_DIV_SHIFT (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT3_PLL_DSP_HSDIVIDER_CLKOUT3_DIV_RESETVAL (0x00000004U)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT3_PLL_DSP_HSDIVIDER_CLKOUT3_DIV_MAX (0x0000001FU)

#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT3_PLL_DSP_HSDIVIDER_CLKOUT3_DIVCHACK_MASK (0x00000020U)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT3_PLL_DSP_HSDIVIDER_CLKOUT3_DIVCHACK_SHIFT (0x00000005U)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT3_PLL_DSP_HSDIVIDER_CLKOUT3_DIVCHACK_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT3_PLL_DSP_HSDIVIDER_CLKOUT3_DIVCHACK_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT3_PLL_DSP_HSDIVIDER_CLKOUT3_GATE_CTRL_MASK (0x00000100U)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT3_PLL_DSP_HSDIVIDER_CLKOUT3_GATE_CTRL_SHIFT (0x00000008U)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT3_PLL_DSP_HSDIVIDER_CLKOUT3_GATE_CTRL_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT3_PLL_DSP_HSDIVIDER_CLKOUT3_GATE_CTRL_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT3_PLL_DSP_HSDIVIDER_CLKOUT3_STATUS_MASK (0x00000200U)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT3_PLL_DSP_HSDIVIDER_CLKOUT3_STATUS_SHIFT (0x00000009U)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT3_PLL_DSP_HSDIVIDER_CLKOUT3_STATUS_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT3_PLL_DSP_HSDIVIDER_CLKOUT3_STATUS_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT3_PLL_DSP_HSDIVIDER_CLKOUT3_PWDN_MASK (0x00001000U)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT3_PLL_DSP_HSDIVIDER_CLKOUT3_PWDN_SHIFT (0x0000000CU)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT3_PLL_DSP_HSDIVIDER_CLKOUT3_PWDN_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT3_PLL_DSP_HSDIVIDER_CLKOUT3_PWDN_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT3_RESETVAL                      (0x00000004U)

/* PLL_PER_PWRCTRL */

#define CSL_MSS_TOPRCM_PLL_PER_PWRCTRL_PLL_PER_PWRCTRL_OFFMODE_MASK            (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_PER_PWRCTRL_PLL_PER_PWRCTRL_OFFMODE_SHIFT           (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_PWRCTRL_PLL_PER_PWRCTRL_OFFMODE_RESETVAL        (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_PWRCTRL_PLL_PER_PWRCTRL_OFFMODE_MAX             (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_PER_PWRCTRL_PLL_PER_PWRCTRL_ISOSCAN_MASK            (0x00000002U)
#define CSL_MSS_TOPRCM_PLL_PER_PWRCTRL_PLL_PER_PWRCTRL_ISOSCAN_SHIFT           (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_PER_PWRCTRL_PLL_PER_PWRCTRL_ISOSCAN_RESETVAL        (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_PWRCTRL_PLL_PER_PWRCTRL_ISOSCAN_MAX             (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_PER_PWRCTRL_PLL_PER_PWRCTRL_ISORET_MASK             (0x00000004U)
#define CSL_MSS_TOPRCM_PLL_PER_PWRCTRL_PLL_PER_PWRCTRL_ISORET_SHIFT            (0x00000002U)
#define CSL_MSS_TOPRCM_PLL_PER_PWRCTRL_PLL_PER_PWRCTRL_ISORET_RESETVAL         (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_PWRCTRL_PLL_PER_PWRCTRL_ISORET_MAX              (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_PER_PWRCTRL_PLL_PER_PWRCTRL_RET_MASK                (0x00000008U)
#define CSL_MSS_TOPRCM_PLL_PER_PWRCTRL_PLL_PER_PWRCTRL_RET_SHIFT               (0x00000003U)
#define CSL_MSS_TOPRCM_PLL_PER_PWRCTRL_PLL_PER_PWRCTRL_RET_RESETVAL            (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_PWRCTRL_PLL_PER_PWRCTRL_RET_MAX                 (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_PER_PWRCTRL_PLL_PER_PWRCTRL_PGOODIN_MASK            (0x00000010U)
#define CSL_MSS_TOPRCM_PLL_PER_PWRCTRL_PLL_PER_PWRCTRL_PGOODIN_SHIFT           (0x00000004U)
#define CSL_MSS_TOPRCM_PLL_PER_PWRCTRL_PLL_PER_PWRCTRL_PGOODIN_RESETVAL        (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_PER_PWRCTRL_PLL_PER_PWRCTRL_PGOODIN_MAX             (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_PER_PWRCTRL_PLL_PER_PWRCTRL_PONIN_MASK              (0x00000020U)
#define CSL_MSS_TOPRCM_PLL_PER_PWRCTRL_PLL_PER_PWRCTRL_PONIN_SHIFT             (0x00000005U)
#define CSL_MSS_TOPRCM_PLL_PER_PWRCTRL_PLL_PER_PWRCTRL_PONIN_RESETVAL          (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_PER_PWRCTRL_PLL_PER_PWRCTRL_PONIN_MAX               (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_PER_PWRCTRL_RESETVAL                                (0x00000030U)

/* PLL_PER_CLKCTRL */

#define CSL_MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_TINTZ_MASK              (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_TINTZ_SHIFT             (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_TINTZ_RESETVAL          (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_TINTZ_MAX               (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_SSCTYPE_MASK            (0x00000002U)
#define CSL_MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_SSCTYPE_SHIFT           (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_SSCTYPE_RESETVAL        (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_SSCTYPE_MAX             (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_RELAXED_LOCK_MASK       (0x00000100U)
#define CSL_MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_RELAXED_LOCK_SHIFT      (0x00000008U)
#define CSL_MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_RELAXED_LOCK_RESETVAL   (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_RELAXED_LOCK_MAX        (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_SELFREQDCO_MASK         (0x00001C00U)
#define CSL_MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_SELFREQDCO_SHIFT        (0x0000000AU)
#define CSL_MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_SELFREQDCO_RESETVAL     (0x00000002U)
#define CSL_MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_SELFREQDCO_MAX          (0x00000007U)

#define CSL_MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_STOPMODE_MASK           (0x00004000U)
#define CSL_MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_STOPMODE_SHIFT          (0x0000000EU)
#define CSL_MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_STOPMODE_RESETVAL       (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_STOPMODE_MAX            (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_M2PWDNZ_MASK            (0x00010000U)
#define CSL_MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_M2PWDNZ_SHIFT           (0x00000010U)
#define CSL_MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_M2PWDNZ_RESETVAL        (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_M2PWDNZ_MAX             (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_CLKDCOLDOPWDNZ_MASK     (0x00020000U)
#define CSL_MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_CLKDCOLDOPWDNZ_SHIFT    (0x00000011U)
#define CSL_MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_CLKDCOLDOPWDNZ_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_CLKDCOLDOPWDNZ_MAX      (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_ULOWCLKEN_MASK          (0x00040000U)
#define CSL_MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_ULOWCLKEN_SHIFT         (0x00000012U)
#define CSL_MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_ULOWCLKEN_RESETVAL      (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_ULOWCLKEN_MAX           (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_CLKOUTLDOEN_MASK        (0x00080000U)
#define CSL_MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_CLKOUTLDOEN_SHIFT       (0x00000013U)
#define CSL_MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_CLKOUTLDOEN_RESETVAL    (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_CLKOUTLDOEN_MAX         (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_CLKOUTEN_MASK           (0x00100000U)
#define CSL_MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_CLKOUTEN_SHIFT          (0x00000014U)
#define CSL_MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_CLKOUTEN_RESETVAL       (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_CLKOUTEN_MAX            (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_STBYRET_MASK            (0x00200000U)
#define CSL_MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_STBYRET_SHIFT           (0x00000015U)
#define CSL_MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_STBYRET_RESETVAL        (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_STBYRET_MAX             (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_BYPASSACKZ_MASK         (0x00400000U)
#define CSL_MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_BYPASSACKZ_SHIFT        (0x00000016U)
#define CSL_MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_BYPASSACKZ_RESETVAL     (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_BYPASSACKZ_MAX          (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_IDLE_MASK               (0x00800000U)
#define CSL_MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_IDLE_SHIFT              (0x00000017U)
#define CSL_MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_IDLE_RESETVAL           (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_IDLE_MAX                (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_NWELLTRIM_MASK          (0x1F000000U)
#define CSL_MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_NWELLTRIM_SHIFT         (0x00000018U)
#define CSL_MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_NWELLTRIM_RESETVAL      (0x00000009U)
#define CSL_MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_NWELLTRIM_MAX           (0x0000001FU)

#define CSL_MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_CLKDCOLDOEN_MASK        (0x20000000U)
#define CSL_MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_CLKDCOLDOEN_SHIFT       (0x0000001DU)
#define CSL_MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_CLKDCOLDOEN_RESETVAL    (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_CLKDCOLDOEN_MAX         (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_ENSSC_MASK              (0x40000000U)
#define CSL_MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_ENSSC_SHIFT             (0x0000001EU)
#define CSL_MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_ENSSC_RESETVAL          (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_ENSSC_MAX               (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_CYCLESLIPEN_MASK        (0x80000000U)
#define CSL_MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_CYCLESLIPEN_SHIFT       (0x0000001FU)
#define CSL_MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_CYCLESLIPEN_RESETVAL    (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_CYCLESLIPEN_MAX         (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_PER_CLKCTRL_RESETVAL                                (0x09914800U)

/* PLL_PER_TENABLE */

#define CSL_MSS_TOPRCM_PLL_PER_TENABLE_PLL_PER_TENABLE_TENABLE_MASK            (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_PER_TENABLE_PLL_PER_TENABLE_TENABLE_SHIFT           (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_TENABLE_PLL_PER_TENABLE_TENABLE_RESETVAL        (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_TENABLE_PLL_PER_TENABLE_TENABLE_MAX             (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_PER_TENABLE_RESETVAL                                (0x00000000U)

/* PLL_PER_TENABLEDIV */

#define CSL_MSS_TOPRCM_PLL_PER_TENABLEDIV_PLL_PER_TENABLEDIV_TENABLEDIV_MASK   (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_PER_TENABLEDIV_PLL_PER_TENABLEDIV_TENABLEDIV_SHIFT  (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_TENABLEDIV_PLL_PER_TENABLEDIV_TENABLEDIV_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_TENABLEDIV_PLL_PER_TENABLEDIV_TENABLEDIV_MAX    (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_PER_TENABLEDIV_RESETVAL                             (0x00000000U)

/* PLL_PER_M2NDIV */

#define CSL_MSS_TOPRCM_PLL_PER_M2NDIV_PLL_PER_M2NDIV_N_MASK                    (0x000000FFU)
#define CSL_MSS_TOPRCM_PLL_PER_M2NDIV_PLL_PER_M2NDIV_N_SHIFT                   (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_M2NDIV_PLL_PER_M2NDIV_N_RESETVAL                (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_M2NDIV_PLL_PER_M2NDIV_N_MAX                     (0x000000FFU)

#define CSL_MSS_TOPRCM_PLL_PER_M2NDIV_PLL_PER_M2NDIV_M2_MASK                   (0x007F0000U)
#define CSL_MSS_TOPRCM_PLL_PER_M2NDIV_PLL_PER_M2NDIV_M2_SHIFT                  (0x00000010U)
#define CSL_MSS_TOPRCM_PLL_PER_M2NDIV_PLL_PER_M2NDIV_M2_RESETVAL               (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_M2NDIV_PLL_PER_M2NDIV_M2_MAX                    (0x0000007FU)

#define CSL_MSS_TOPRCM_PLL_PER_M2NDIV_RESETVAL                                 (0x00000000U)

/* PLL_PER_MN2DIV */

#define CSL_MSS_TOPRCM_PLL_PER_MN2DIV_PLL_PER_MN2DIV_M_MASK                    (0x00000FFFU)
#define CSL_MSS_TOPRCM_PLL_PER_MN2DIV_PLL_PER_MN2DIV_M_SHIFT                   (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_MN2DIV_PLL_PER_MN2DIV_M_RESETVAL                (0x00000174U)
#define CSL_MSS_TOPRCM_PLL_PER_MN2DIV_PLL_PER_MN2DIV_M_MAX                     (0x00000FFFU)

#define CSL_MSS_TOPRCM_PLL_PER_MN2DIV_PLL_PER_MN2DIV_N2_MASK                   (0x000F0000U)
#define CSL_MSS_TOPRCM_PLL_PER_MN2DIV_PLL_PER_MN2DIV_N2_SHIFT                  (0x00000010U)
#define CSL_MSS_TOPRCM_PLL_PER_MN2DIV_PLL_PER_MN2DIV_N2_RESETVAL               (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_MN2DIV_PLL_PER_MN2DIV_N2_MAX                    (0x0000000FU)

#define CSL_MSS_TOPRCM_PLL_PER_MN2DIV_RESETVAL                                 (0x00000174U)

/* PLL_PER_FRACDIV */

#define CSL_MSS_TOPRCM_PLL_PER_FRACDIV_PLL_PER_FRACDIV_FRACTIONALM_MASK        (0x0003FFFFU)
#define CSL_MSS_TOPRCM_PLL_PER_FRACDIV_PLL_PER_FRACDIV_FRACTIONALM_SHIFT       (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_FRACDIV_PLL_PER_FRACDIV_FRACTIONALM_RESETVAL    (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_FRACDIV_PLL_PER_FRACDIV_FRACTIONALM_MAX         (0x0003FFFFU)

#define CSL_MSS_TOPRCM_PLL_PER_FRACDIV_PLL_PER_FRACDIV_REGSD_MASK              (0xFF000000U)
#define CSL_MSS_TOPRCM_PLL_PER_FRACDIV_PLL_PER_FRACDIV_REGSD_SHIFT             (0x00000018U)
#define CSL_MSS_TOPRCM_PLL_PER_FRACDIV_PLL_PER_FRACDIV_REGSD_RESETVAL          (0x00000008U)
#define CSL_MSS_TOPRCM_PLL_PER_FRACDIV_PLL_PER_FRACDIV_REGSD_MAX               (0x000000FFU)

#define CSL_MSS_TOPRCM_PLL_PER_FRACDIV_RESETVAL                                (0x08000000U)

/* PLL_PER_BWCTRL */

#define CSL_MSS_TOPRCM_PLL_PER_BWCTRL_PLL_PER_BWCTRL_BW_INCR_DECRZ_MASK        (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_PER_BWCTRL_PLL_PER_BWCTRL_BW_INCR_DECRZ_SHIFT       (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_BWCTRL_PLL_PER_BWCTRL_BW_INCR_DECRZ_RESETVAL    (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_BWCTRL_PLL_PER_BWCTRL_BW_INCR_DECRZ_MAX         (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_PER_BWCTRL_PLL_PER_BWCTRL_BWCONTROL_MASK            (0x00000006U)
#define CSL_MSS_TOPRCM_PLL_PER_BWCTRL_PLL_PER_BWCTRL_BWCONTROL_SHIFT           (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_PER_BWCTRL_PLL_PER_BWCTRL_BWCONTROL_RESETVAL        (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_BWCTRL_PLL_PER_BWCTRL_BWCONTROL_MAX             (0x00000003U)

#define CSL_MSS_TOPRCM_PLL_PER_BWCTRL_RESETVAL                                 (0x00000000U)

/* PLL_PER_FRACCTRL */

#define CSL_MSS_TOPRCM_PLL_PER_FRACCTRL_PLL_PER_FRACCTRL_DELTAMSTEPFRACTION_MASK (0x0003FFFFU)
#define CSL_MSS_TOPRCM_PLL_PER_FRACCTRL_PLL_PER_FRACCTRL_DELTAMSTEPFRACTION_SHIFT (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_FRACCTRL_PLL_PER_FRACCTRL_DELTAMSTEPFRACTION_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_FRACCTRL_PLL_PER_FRACCTRL_DELTAMSTEPFRACTION_MAX (0x0003FFFFU)

#define CSL_MSS_TOPRCM_PLL_PER_FRACCTRL_PLL_PER_FRACCTRL_DELTAMSTEPINTEGER_MASK (0x001C0000U)
#define CSL_MSS_TOPRCM_PLL_PER_FRACCTRL_PLL_PER_FRACCTRL_DELTAMSTEPINTEGER_SHIFT (0x00000012U)
#define CSL_MSS_TOPRCM_PLL_PER_FRACCTRL_PLL_PER_FRACCTRL_DELTAMSTEPINTEGER_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_FRACCTRL_PLL_PER_FRACCTRL_DELTAMSTEPINTEGER_MAX (0x00000007U)

#define CSL_MSS_TOPRCM_PLL_PER_FRACCTRL_PLL_PER_FRACCTRL_MODFREQDIVIDERMANTISSA_MASK (0x0FE00000U)
#define CSL_MSS_TOPRCM_PLL_PER_FRACCTRL_PLL_PER_FRACCTRL_MODFREQDIVIDERMANTISSA_SHIFT (0x00000015U)
#define CSL_MSS_TOPRCM_PLL_PER_FRACCTRL_PLL_PER_FRACCTRL_MODFREQDIVIDERMANTISSA_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_FRACCTRL_PLL_PER_FRACCTRL_MODFREQDIVIDERMANTISSA_MAX (0x0000007FU)

#define CSL_MSS_TOPRCM_PLL_PER_FRACCTRL_PLL_PER_FRACCTRL_MODFREQDIVIDEREXPONENT_MASK (0x70000000U)
#define CSL_MSS_TOPRCM_PLL_PER_FRACCTRL_PLL_PER_FRACCTRL_MODFREQDIVIDEREXPONENT_SHIFT (0x0000001CU)
#define CSL_MSS_TOPRCM_PLL_PER_FRACCTRL_PLL_PER_FRACCTRL_MODFREQDIVIDEREXPONENT_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_FRACCTRL_PLL_PER_FRACCTRL_MODFREQDIVIDEREXPONENT_MAX (0x00000007U)

#define CSL_MSS_TOPRCM_PLL_PER_FRACCTRL_PLL_PER_FRACCTRL_DOWNSPREAD_MASK       (0x80000000U)
#define CSL_MSS_TOPRCM_PLL_PER_FRACCTRL_PLL_PER_FRACCTRL_DOWNSPREAD_SHIFT      (0x0000001FU)
#define CSL_MSS_TOPRCM_PLL_PER_FRACCTRL_PLL_PER_FRACCTRL_DOWNSPREAD_RESETVAL   (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_FRACCTRL_PLL_PER_FRACCTRL_DOWNSPREAD_MAX        (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_PER_FRACCTRL_RESETVAL                               (0x00000000U)

/* PLL_PER_STATUS */

#define CSL_MSS_TOPRCM_PLL_PER_STATUS_PLL_PER_STATUS_BYPASS_MASK               (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_PER_STATUS_PLL_PER_STATUS_BYPASS_SHIFT              (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_STATUS_PLL_PER_STATUS_BYPASS_RESETVAL           (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_PER_STATUS_PLL_PER_STATUS_BYPASS_MAX                (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_PER_STATUS_PLL_PER_STATUS_HIGHJITTER_MASK           (0x00000002U)
#define CSL_MSS_TOPRCM_PLL_PER_STATUS_PLL_PER_STATUS_HIGHJITTER_SHIFT          (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_PER_STATUS_PLL_PER_STATUS_HIGHJITTER_RESETVAL       (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_STATUS_PLL_PER_STATUS_HIGHJITTER_MAX            (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_PER_STATUS_PLL_PER_STATUS_SSCACK_MASK               (0x00000004U)
#define CSL_MSS_TOPRCM_PLL_PER_STATUS_PLL_PER_STATUS_SSCACK_SHIFT              (0x00000002U)
#define CSL_MSS_TOPRCM_PLL_PER_STATUS_PLL_PER_STATUS_SSCACK_RESETVAL           (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_STATUS_PLL_PER_STATUS_SSCACK_MAX                (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_PER_STATUS_PLL_PER_STATUS_M2CHANGEACK_MASK          (0x00000008U)
#define CSL_MSS_TOPRCM_PLL_PER_STATUS_PLL_PER_STATUS_M2CHANGEACK_SHIFT         (0x00000003U)
#define CSL_MSS_TOPRCM_PLL_PER_STATUS_PLL_PER_STATUS_M2CHANGEACK_RESETVAL      (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_STATUS_PLL_PER_STATUS_M2CHANGEACK_MAX           (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_PER_STATUS_PLL_PER_STATUS_LOCK2_MASK                (0x00000010U)
#define CSL_MSS_TOPRCM_PLL_PER_STATUS_PLL_PER_STATUS_LOCK2_SHIFT               (0x00000004U)
#define CSL_MSS_TOPRCM_PLL_PER_STATUS_PLL_PER_STATUS_LOCK2_RESETVAL            (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_STATUS_PLL_PER_STATUS_LOCK2_MAX                 (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_PER_STATUS_PLL_PER_STATUS_CLKOUTENACK_MASK          (0x00000020U)
#define CSL_MSS_TOPRCM_PLL_PER_STATUS_PLL_PER_STATUS_CLKOUTENACK_SHIFT         (0x00000005U)
#define CSL_MSS_TOPRCM_PLL_PER_STATUS_PLL_PER_STATUS_CLKOUTENACK_RESETVAL      (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_PER_STATUS_PLL_PER_STATUS_CLKOUTENACK_MAX           (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_PER_STATUS_PLL_PER_STATUS_LOSSREF_MASK              (0x00000040U)
#define CSL_MSS_TOPRCM_PLL_PER_STATUS_PLL_PER_STATUS_LOSSREF_SHIFT             (0x00000006U)
#define CSL_MSS_TOPRCM_PLL_PER_STATUS_PLL_PER_STATUS_LOSSREF_RESETVAL          (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_PER_STATUS_PLL_PER_STATUS_LOSSREF_MAX               (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_PER_STATUS_PLL_PER_STATUS_STBYRETACK_MASK           (0x00000080U)
#define CSL_MSS_TOPRCM_PLL_PER_STATUS_PLL_PER_STATUS_STBYRETACK_SHIFT          (0x00000007U)
#define CSL_MSS_TOPRCM_PLL_PER_STATUS_PLL_PER_STATUS_STBYRETACK_RESETVAL       (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_STATUS_PLL_PER_STATUS_STBYRETACK_MAX            (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_PER_STATUS_PLL_PER_STATUS_BYPASSACK_MASK            (0x00000100U)
#define CSL_MSS_TOPRCM_PLL_PER_STATUS_PLL_PER_STATUS_BYPASSACK_SHIFT           (0x00000008U)
#define CSL_MSS_TOPRCM_PLL_PER_STATUS_PLL_PER_STATUS_BYPASSACK_RESETVAL        (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_PER_STATUS_PLL_PER_STATUS_BYPASSACK_MAX             (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_PER_STATUS_PLL_PER_STATUS_FREQLOCK_MASK             (0x00000200U)
#define CSL_MSS_TOPRCM_PLL_PER_STATUS_PLL_PER_STATUS_FREQLOCK_SHIFT            (0x00000009U)
#define CSL_MSS_TOPRCM_PLL_PER_STATUS_PLL_PER_STATUS_FREQLOCK_RESETVAL         (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_STATUS_PLL_PER_STATUS_FREQLOCK_MAX              (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_PER_STATUS_PLL_PER_STATUS_PHASELOCK_MASK            (0x00000400U)
#define CSL_MSS_TOPRCM_PLL_PER_STATUS_PLL_PER_STATUS_PHASELOCK_SHIFT           (0x0000000AU)
#define CSL_MSS_TOPRCM_PLL_PER_STATUS_PLL_PER_STATUS_PHASELOCK_RESETVAL        (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_STATUS_PLL_PER_STATUS_PHASELOCK_MAX             (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_PER_STATUS_PLL_PER_STATUS_CLKDCOLDOACK_MASK         (0x00000800U)
#define CSL_MSS_TOPRCM_PLL_PER_STATUS_PLL_PER_STATUS_CLKDCOLDOACK_SHIFT        (0x0000000BU)
#define CSL_MSS_TOPRCM_PLL_PER_STATUS_PLL_PER_STATUS_CLKDCOLDOACK_RESETVAL     (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_STATUS_PLL_PER_STATUS_CLKDCOLDOACK_MAX          (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_PER_STATUS_PLL_PER_STATUS_RECAL_OPPIN_MASK          (0x08000000U)
#define CSL_MSS_TOPRCM_PLL_PER_STATUS_PLL_PER_STATUS_RECAL_OPPIN_SHIFT         (0x0000001BU)
#define CSL_MSS_TOPRCM_PLL_PER_STATUS_PLL_PER_STATUS_RECAL_OPPIN_RESETVAL      (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_STATUS_PLL_PER_STATUS_RECAL_OPPIN_MAX           (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_PER_STATUS_PLL_PER_STATUS_RECAL_BSTATUS3_MASK       (0x10000000U)
#define CSL_MSS_TOPRCM_PLL_PER_STATUS_PLL_PER_STATUS_RECAL_BSTATUS3_SHIFT      (0x0000001CU)
#define CSL_MSS_TOPRCM_PLL_PER_STATUS_PLL_PER_STATUS_RECAL_BSTATUS3_RESETVAL   (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_STATUS_PLL_PER_STATUS_RECAL_BSTATUS3_MAX        (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_PER_STATUS_PLL_PER_STATUS_LDOPWDN_MASK              (0x20000000U)
#define CSL_MSS_TOPRCM_PLL_PER_STATUS_PLL_PER_STATUS_LDOPWDN_SHIFT             (0x0000001DU)
#define CSL_MSS_TOPRCM_PLL_PER_STATUS_PLL_PER_STATUS_LDOPWDN_RESETVAL          (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_PER_STATUS_PLL_PER_STATUS_LDOPWDN_MAX               (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_PER_STATUS_PLL_PER_STATUS_PGOODOUT_MASK             (0x40000000U)
#define CSL_MSS_TOPRCM_PLL_PER_STATUS_PLL_PER_STATUS_PGOODOUT_SHIFT            (0x0000001EU)
#define CSL_MSS_TOPRCM_PLL_PER_STATUS_PLL_PER_STATUS_PGOODOUT_RESETVAL         (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_PER_STATUS_PLL_PER_STATUS_PGOODOUT_MAX              (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_PER_STATUS_PLL_PER_STATUS_PONOUT_MASK               (0x80000000U)
#define CSL_MSS_TOPRCM_PLL_PER_STATUS_PLL_PER_STATUS_PONOUT_SHIFT              (0x0000001FU)
#define CSL_MSS_TOPRCM_PLL_PER_STATUS_PLL_PER_STATUS_PONOUT_RESETVAL           (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_PER_STATUS_PLL_PER_STATUS_PONOUT_MAX                (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_PER_STATUS_RESETVAL                                 (0xE0000161U)

/* PLL_PER_HSDIVIDER */

#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_PLL_PER_HSDIVIDER_BYPASS_MASK         (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_PLL_PER_HSDIVIDER_BYPASS_SHIFT        (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_PLL_PER_HSDIVIDER_BYPASS_RESETVAL     (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_PLL_PER_HSDIVIDER_BYPASS_MAX          (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_PLL_PER_HSDIVIDER_LDOPWDN_MASK        (0x00000002U)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_PLL_PER_HSDIVIDER_LDOPWDN_SHIFT       (0x00000001U)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_PLL_PER_HSDIVIDER_LDOPWDN_RESETVAL    (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_PLL_PER_HSDIVIDER_LDOPWDN_MAX         (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_PLL_PER_HSDIVIDER_TENABLEDIV_MASK     (0x00000004U)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_PLL_PER_HSDIVIDER_TENABLEDIV_SHIFT    (0x00000002U)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_PLL_PER_HSDIVIDER_TENABLEDIV_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_PLL_PER_HSDIVIDER_TENABLEDIV_MAX      (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_PLL_PER_HSDIVIDER_BYPASSACKZ_MASK     (0x00010000U)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_PLL_PER_HSDIVIDER_BYPASSACKZ_SHIFT    (0x00000010U)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_PLL_PER_HSDIVIDER_BYPASSACKZ_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_PLL_PER_HSDIVIDER_BYPASSACKZ_MAX      (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_PLL_PER_HSDIVIDER_LDOPWDNACK_MASK     (0x00020000U)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_PLL_PER_HSDIVIDER_LDOPWDNACK_SHIFT    (0x00000011U)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_PLL_PER_HSDIVIDER_LDOPWDNACK_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_PLL_PER_HSDIVIDER_LDOPWDNACK_MAX      (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_RESETVAL                              (0x00000000U)

/* PLL_PER_HSDIVIDER_CLKOUT0 */

#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT0_PLL_PER_HSDIVIDER_CLKOUT0_DIV_MASK (0x0000001FU)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT0_PLL_PER_HSDIVIDER_CLKOUT0_DIV_SHIFT (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT0_PLL_PER_HSDIVIDER_CLKOUT0_DIV_RESETVAL (0x00000004U)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT0_PLL_PER_HSDIVIDER_CLKOUT0_DIV_MAX (0x0000001FU)

#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT0_PLL_PER_HSDIVIDER_CLKOUT0_DIVCHACK_MASK (0x00000020U)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT0_PLL_PER_HSDIVIDER_CLKOUT0_DIVCHACK_SHIFT (0x00000005U)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT0_PLL_PER_HSDIVIDER_CLKOUT0_DIVCHACK_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT0_PLL_PER_HSDIVIDER_CLKOUT0_DIVCHACK_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT0_PLL_PER_HSDIVIDER_CLKOUT0_GATE_CTRL_MASK (0x00000100U)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT0_PLL_PER_HSDIVIDER_CLKOUT0_GATE_CTRL_SHIFT (0x00000008U)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT0_PLL_PER_HSDIVIDER_CLKOUT0_GATE_CTRL_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT0_PLL_PER_HSDIVIDER_CLKOUT0_GATE_CTRL_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT0_PLL_PER_HSDIVIDER_CLKOUT0_STATUS_MASK (0x00000200U)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT0_PLL_PER_HSDIVIDER_CLKOUT0_STATUS_SHIFT (0x00000009U)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT0_PLL_PER_HSDIVIDER_CLKOUT0_STATUS_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT0_PLL_PER_HSDIVIDER_CLKOUT0_STATUS_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT0_PLL_PER_HSDIVIDER_CLKOUT0_PWDN_MASK (0x00001000U)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT0_PLL_PER_HSDIVIDER_CLKOUT0_PWDN_SHIFT (0x0000000CU)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT0_PLL_PER_HSDIVIDER_CLKOUT0_PWDN_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT0_PLL_PER_HSDIVIDER_CLKOUT0_PWDN_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT0_RESETVAL                      (0x00000004U)

/* PLL_PER_HSDIVIDER_CLKOUT1 */

#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT1_PLL_PER_HSDIVIDER_CLKOUT1_DIV_MASK (0x0000001FU)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT1_PLL_PER_HSDIVIDER_CLKOUT1_DIV_SHIFT (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT1_PLL_PER_HSDIVIDER_CLKOUT1_DIV_RESETVAL (0x00000004U)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT1_PLL_PER_HSDIVIDER_CLKOUT1_DIV_MAX (0x0000001FU)

#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT1_PLL_PER_HSDIVIDER_CLKOUT1_DIVCHACK_MASK (0x00000020U)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT1_PLL_PER_HSDIVIDER_CLKOUT1_DIVCHACK_SHIFT (0x00000005U)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT1_PLL_PER_HSDIVIDER_CLKOUT1_DIVCHACK_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT1_PLL_PER_HSDIVIDER_CLKOUT1_DIVCHACK_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT1_PLL_PER_HSDIVIDER_CLKOUT1_GATE_CTRL_MASK (0x00000100U)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT1_PLL_PER_HSDIVIDER_CLKOUT1_GATE_CTRL_SHIFT (0x00000008U)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT1_PLL_PER_HSDIVIDER_CLKOUT1_GATE_CTRL_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT1_PLL_PER_HSDIVIDER_CLKOUT1_GATE_CTRL_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT1_PLL_PER_HSDIVIDER_CLKOUT1_STATUS_MASK (0x00000200U)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT1_PLL_PER_HSDIVIDER_CLKOUT1_STATUS_SHIFT (0x00000009U)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT1_PLL_PER_HSDIVIDER_CLKOUT1_STATUS_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT1_PLL_PER_HSDIVIDER_CLKOUT1_STATUS_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT1_PLL_PER_HSDIVIDER_CLKOUT1_PWDN_MASK (0x00001000U)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT1_PLL_PER_HSDIVIDER_CLKOUT1_PWDN_SHIFT (0x0000000CU)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT1_PLL_PER_HSDIVIDER_CLKOUT1_PWDN_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT1_PLL_PER_HSDIVIDER_CLKOUT1_PWDN_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT1_RESETVAL                      (0x00000004U)

/* PLL_PER_HSDIVIDER_CLKOUT2 */

#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT2_PLL_PER_HSDIVIDER_CLKOUT2_DIV_MASK (0x0000001FU)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT2_PLL_PER_HSDIVIDER_CLKOUT2_DIV_SHIFT (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT2_PLL_PER_HSDIVIDER_CLKOUT2_DIV_RESETVAL (0x00000004U)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT2_PLL_PER_HSDIVIDER_CLKOUT2_DIV_MAX (0x0000001FU)

#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT2_PLL_PER_HSDIVIDER_CLKOUT2_DIVCHACK_MASK (0x00000020U)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT2_PLL_PER_HSDIVIDER_CLKOUT2_DIVCHACK_SHIFT (0x00000005U)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT2_PLL_PER_HSDIVIDER_CLKOUT2_DIVCHACK_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT2_PLL_PER_HSDIVIDER_CLKOUT2_DIVCHACK_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT2_PLL_PER_HSDIVIDER_CLKOUT2_GATE_CTRL_MASK (0x00000100U)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT2_PLL_PER_HSDIVIDER_CLKOUT2_GATE_CTRL_SHIFT (0x00000008U)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT2_PLL_PER_HSDIVIDER_CLKOUT2_GATE_CTRL_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT2_PLL_PER_HSDIVIDER_CLKOUT2_GATE_CTRL_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT2_PLL_PER_HSDIVIDER_CLKOUT2_STATUS_MASK (0x00000200U)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT2_PLL_PER_HSDIVIDER_CLKOUT2_STATUS_SHIFT (0x00000009U)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT2_PLL_PER_HSDIVIDER_CLKOUT2_STATUS_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT2_PLL_PER_HSDIVIDER_CLKOUT2_STATUS_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT2_PLL_PER_HSDIVIDER_CLKOUT2_PWDN_MASK (0x00001000U)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT2_PLL_PER_HSDIVIDER_CLKOUT2_PWDN_SHIFT (0x0000000CU)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT2_PLL_PER_HSDIVIDER_CLKOUT2_PWDN_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT2_PLL_PER_HSDIVIDER_CLKOUT2_PWDN_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT2_RESETVAL                      (0x00000004U)

/* PLL_PER_HSDIVIDER_CLKOUT3 */

#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT3_PLL_PER_HSDIVIDER_CLKOUT3_DIV_MASK (0x0000001FU)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT3_PLL_PER_HSDIVIDER_CLKOUT3_DIV_SHIFT (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT3_PLL_PER_HSDIVIDER_CLKOUT3_DIV_RESETVAL (0x00000004U)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT3_PLL_PER_HSDIVIDER_CLKOUT3_DIV_MAX (0x0000001FU)

#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT3_PLL_PER_HSDIVIDER_CLKOUT3_DIVCHACK_MASK (0x00000020U)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT3_PLL_PER_HSDIVIDER_CLKOUT3_DIVCHACK_SHIFT (0x00000005U)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT3_PLL_PER_HSDIVIDER_CLKOUT3_DIVCHACK_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT3_PLL_PER_HSDIVIDER_CLKOUT3_DIVCHACK_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT3_PLL_PER_HSDIVIDER_CLKOUT3_GATE_CTRL_MASK (0x00000100U)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT3_PLL_PER_HSDIVIDER_CLKOUT3_GATE_CTRL_SHIFT (0x00000008U)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT3_PLL_PER_HSDIVIDER_CLKOUT3_GATE_CTRL_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT3_PLL_PER_HSDIVIDER_CLKOUT3_GATE_CTRL_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT3_PLL_PER_HSDIVIDER_CLKOUT3_STATUS_MASK (0x00000200U)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT3_PLL_PER_HSDIVIDER_CLKOUT3_STATUS_SHIFT (0x00000009U)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT3_PLL_PER_HSDIVIDER_CLKOUT3_STATUS_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT3_PLL_PER_HSDIVIDER_CLKOUT3_STATUS_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT3_PLL_PER_HSDIVIDER_CLKOUT3_PWDN_MASK (0x00001000U)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT3_PLL_PER_HSDIVIDER_CLKOUT3_PWDN_SHIFT (0x0000000CU)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT3_PLL_PER_HSDIVIDER_CLKOUT3_PWDN_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT3_PLL_PER_HSDIVIDER_CLKOUT3_PWDN_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT3_RESETVAL                      (0x00000004U)

/* PLL_DSP_RSTCTRL */

#define CSL_MSS_TOPRCM_PLL_DSP_RSTCTRL_PLL_DSP_RSTCTRL_ASSERT_MASK             (0x00000007U)
#define CSL_MSS_TOPRCM_PLL_DSP_RSTCTRL_PLL_DSP_RSTCTRL_ASSERT_SHIFT            (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_RSTCTRL_PLL_DSP_RSTCTRL_ASSERT_RESETVAL         (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_RSTCTRL_PLL_DSP_RSTCTRL_ASSERT_MAX              (0x00000007U)

#define CSL_MSS_TOPRCM_PLL_DSP_RSTCTRL_RESETVAL                                (0x00000000U)

/* PLL_DSP_HSDIVIDER_RSTCTRL */

#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_RSTCTRL_PLL_DSP_HSDIVIDER_RSTCTRL_ASSERT_MASK (0x00000007U)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_RSTCTRL_PLL_DSP_HSDIVIDER_RSTCTRL_ASSERT_SHIFT (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_RSTCTRL_PLL_DSP_HSDIVIDER_RSTCTRL_ASSERT_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_RSTCTRL_PLL_DSP_HSDIVIDER_RSTCTRL_ASSERT_MAX (0x00000007U)

#define CSL_MSS_TOPRCM_PLL_DSP_HSDIVIDER_RSTCTRL_RESETVAL                      (0x00000000U)

/* PLL_PER_RSTCTRL */

#define CSL_MSS_TOPRCM_PLL_PER_RSTCTRL_PLL_PER_RSTCTRL_ASSERT_MASK             (0x00000007U)
#define CSL_MSS_TOPRCM_PLL_PER_RSTCTRL_PLL_PER_RSTCTRL_ASSERT_SHIFT            (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_RSTCTRL_PLL_PER_RSTCTRL_ASSERT_RESETVAL         (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_RSTCTRL_PLL_PER_RSTCTRL_ASSERT_MAX              (0x00000007U)

#define CSL_MSS_TOPRCM_PLL_PER_RSTCTRL_RESETVAL                                (0x00000000U)

/* PLL_PER_HSDIVIDER_RSTCTRL */

#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_RSTCTRL_PLL_PER_HSDIVIDER_RSTCTRL_ASSERT_MASK (0x00000007U)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_RSTCTRL_PLL_PER_HSDIVIDER_RSTCTRL_ASSERT_SHIFT (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_RSTCTRL_PLL_PER_HSDIVIDER_RSTCTRL_ASSERT_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_RSTCTRL_PLL_PER_HSDIVIDER_RSTCTRL_ASSERT_MAX (0x00000007U)

#define CSL_MSS_TOPRCM_PLL_PER_HSDIVIDER_RSTCTRL_RESETVAL                      (0x00000000U)

/* ANA_REG_CLK_CTRL_REG1_XO_SLICER */

#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG1_XO_SLICER_RTRIM_BIAS_XO_SLICER_MASK (0x0000000FU)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG1_XO_SLICER_RTRIM_BIAS_XO_SLICER_SHIFT (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG1_XO_SLICER_RTRIM_BIAS_XO_SLICER_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG1_XO_SLICER_RTRIM_BIAS_XO_SLICER_MAX (0x0000000FU)

#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG1_XO_SLICER_XOSC_DRIVE_XO_SLICER_MASK (0x000001F0U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG1_XO_SLICER_XOSC_DRIVE_XO_SLICER_SHIFT (0x00000004U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG1_XO_SLICER_XOSC_DRIVE_XO_SLICER_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG1_XO_SLICER_XOSC_DRIVE_XO_SLICER_MAX (0x0000001FU)

#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG1_XO_SLICER_FASTCHARGEZ_BIAS_XO_SLICER_MASK (0x00000200U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG1_XO_SLICER_FASTCHARGEZ_BIAS_XO_SLICER_SHIFT (0x00000009U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG1_XO_SLICER_FASTCHARGEZ_BIAS_XO_SLICER_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG1_XO_SLICER_FASTCHARGEZ_BIAS_XO_SLICER_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG1_XO_SLICER_SLICER_HIPWR_XO_SLICER_MASK (0x00000400U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG1_XO_SLICER_SLICER_HIPWR_XO_SLICER_SHIFT (0x0000000AU)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG1_XO_SLICER_SLICER_HIPWR_XO_SLICER_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG1_XO_SLICER_SLICER_HIPWR_XO_SLICER_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG1_XO_SLICER_SLICER_DCCPL_XO_SLICER_MASK (0x00000800U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG1_XO_SLICER_SLICER_DCCPL_XO_SLICER_SHIFT (0x0000000BU)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG1_XO_SLICER_SLICER_DCCPL_XO_SLICER_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG1_XO_SLICER_SLICER_DCCPL_XO_SLICER_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG1_XO_SLICER_XTAL_DETECT_XO_SLICER_MASK (0x00001000U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG1_XO_SLICER_XTAL_DETECT_XO_SLICER_SHIFT (0x0000000CU)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG1_XO_SLICER_XTAL_DETECT_XO_SLICER_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG1_XO_SLICER_XTAL_DETECT_XO_SLICER_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG1_XO_SLICER_SLICER_APLL_BYPASS_MASK (0x00002000U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG1_XO_SLICER_SLICER_APLL_BYPASS_SHIFT (0x0000000DU)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG1_XO_SLICER_SLICER_APLL_BYPASS_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG1_XO_SLICER_SLICER_APLL_BYPASS_MAX  (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG1_XO_SLICER_SLICER_APLL_BYPASS_DRV_MASK (0x00004000U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG1_XO_SLICER_SLICER_APLL_BYPASS_DRV_SHIFT (0x0000000EU)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG1_XO_SLICER_SLICER_APLL_BYPASS_DRV_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG1_XO_SLICER_SLICER_APLL_BYPASS_DRV_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG1_XO_SLICER_RESERVED0_MASK          (0xFFFF8000U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG1_XO_SLICER_RESERVED0_SHIFT         (0x0000000FU)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG1_XO_SLICER_RESERVED0_RESETVAL      (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG1_XO_SLICER_RESERVED0_MAX           (0x0001FFFFU)

#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG1_XO_SLICER_RESETVAL                (0x00000000U)

/* ANA_REG_CLK_CTRL_REG1_CLKTOP */

#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG1_CLKTOP_ENABLE_BIAS_XO_SLICER_MASK (0x00000001U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG1_CLKTOP_ENABLE_BIAS_XO_SLICER_SHIFT (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG1_CLKTOP_ENABLE_BIAS_XO_SLICER_RESETVAL (0x00000001U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG1_CLKTOP_ENABLE_BIAS_XO_SLICER_MAX  (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG1_CLKTOP_ENABLE_SLICER_CLKP_MASK    (0x00000002U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG1_CLKTOP_ENABLE_SLICER_CLKP_SHIFT   (0x00000001U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG1_CLKTOP_ENABLE_SLICER_CLKP_RESETVAL (0x00000001U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG1_CLKTOP_ENABLE_SLICER_CLKP_MAX     (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG1_CLKTOP_ENABLE_XOSC_MASK           (0x00000004U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG1_CLKTOP_ENABLE_XOSC_SHIFT          (0x00000002U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG1_CLKTOP_ENABLE_XOSC_RESETVAL       (0x00000001U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG1_CLKTOP_ENABLE_XOSC_MAX            (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG1_CLKTOP_RESERVED0_MASK             (0xFFFFFFF8U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG1_CLKTOP_RESERVED0_SHIFT            (0x00000003U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG1_CLKTOP_RESERVED0_RESETVAL         (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG1_CLKTOP_RESERVED0_MAX              (0x1FFFFFFFU)

#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG1_CLKTOP_RESETVAL                   (0x00000007U)

/* ANA_REG_CLK_CTRL_REG2_CLKTOP */

#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG2_CLKTOP_RESERVED0_MASK             (0xFFFFFFFFU)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG2_CLKTOP_RESERVED0_SHIFT            (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG2_CLKTOP_RESERVED0_RESETVAL         (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG2_CLKTOP_RESERVED0_MAX              (0xFFFFFFFFU)

#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG2_CLKTOP_RESETVAL                   (0x00000000U)

/* ANA_REG_CLK_CTRL_REG1_LDO_CLKTOP */

#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG1_LDO_CLKTOP_EN_SLICER_LDO_MASK     (0x00000001U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG1_LDO_CLKTOP_EN_SLICER_LDO_SHIFT    (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG1_LDO_CLKTOP_EN_SLICER_LDO_RESETVAL (0x00000001U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG1_LDO_CLKTOP_EN_SLICER_LDO_MAX      (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG1_LDO_CLKTOP_RESERVED0_MASK         (0x000001FEU)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG1_LDO_CLKTOP_RESERVED0_SHIFT        (0x00000001U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG1_LDO_CLKTOP_RESERVED0_RESETVAL     (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG1_LDO_CLKTOP_RESERVED0_MAX          (0x000000FFU)

#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG1_LDO_CLKTOP_CLK_BIST_DISABLE_LDO_MASK (0x00000200U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG1_LDO_CLKTOP_CLK_BIST_DISABLE_LDO_SHIFT (0x00000009U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG1_LDO_CLKTOP_CLK_BIST_DISABLE_LDO_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG1_LDO_CLKTOP_CLK_BIST_DISABLE_LDO_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG1_LDO_CLKTOP_RESERVED1_MASK         (0xFFFFFC00U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG1_LDO_CLKTOP_RESERVED1_SHIFT        (0x0000000AU)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG1_LDO_CLKTOP_RESERVED1_RESETVAL     (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG1_LDO_CLKTOP_RESERVED1_MAX          (0x003FFFFFU)

#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG1_LDO_CLKTOP_RESETVAL               (0x00000001U)

/* ANA_REG_CLK_CTRL_REG2_LDO_CLKTOP */

#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG2_LDO_CLKTOP_LDO_VOUT_CTRL_MASK     (0x0000000FU)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG2_LDO_CLKTOP_LDO_VOUT_CTRL_SHIFT    (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG2_LDO_CLKTOP_LDO_VOUT_CTRL_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG2_LDO_CLKTOP_LDO_VOUT_CTRL_MAX      (0x0000000FU)

#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG2_LDO_CLKTOP_ENZ_LOW_BW_CAP_MASK    (0x00000010U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG2_LDO_CLKTOP_ENZ_LOW_BW_CAP_SHIFT   (0x00000004U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG2_LDO_CLKTOP_ENZ_LOW_BW_CAP_RESETVAL (0x00000001U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG2_LDO_CLKTOP_ENZ_LOW_BW_CAP_MAX     (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG2_LDO_CLKTOP_EN_TEST_MODE_MASK      (0x00000020U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG2_LDO_CLKTOP_EN_TEST_MODE_SHIFT     (0x00000005U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG2_LDO_CLKTOP_EN_TEST_MODE_RESETVAL  (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG2_LDO_CLKTOP_EN_TEST_MODE_MAX       (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG2_LDO_CLKTOP_EN_SHRT_CKT_MASK       (0x00000040U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG2_LDO_CLKTOP_EN_SHRT_CKT_SHIFT      (0x00000006U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG2_LDO_CLKTOP_EN_SHRT_CKT_RESETVAL   (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG2_LDO_CLKTOP_EN_SHRT_CKT_MAX        (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG2_LDO_CLKTOP_EN_BYPASS_MASK         (0x00000080U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG2_LDO_CLKTOP_EN_BYPASS_SHIFT        (0x00000007U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG2_LDO_CLKTOP_EN_BYPASS_RESETVAL     (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG2_LDO_CLKTOP_EN_BYPASS_MAX          (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG2_LDO_CLKTOP_LDO_BW_CTRL_MASK       (0x00000700U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG2_LDO_CLKTOP_LDO_BW_CTRL_SHIFT      (0x00000008U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG2_LDO_CLKTOP_LDO_BW_CTRL_RESETVAL   (0x00000007U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG2_LDO_CLKTOP_LDO_BW_CTRL_MAX        (0x00000007U)

#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG2_LDO_CLKTOP_SCPRT_IBIAS_CTRL_MASK  (0x00000800U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG2_LDO_CLKTOP_SCPRT_IBIAS_CTRL_SHIFT (0x0000000BU)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG2_LDO_CLKTOP_SCPRT_IBIAS_CTRL_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG2_LDO_CLKTOP_SCPRT_IBIAS_CTRL_MAX   (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG2_LDO_CLKTOP_ENABLE_PMOS_PULLDOWN_MASK (0x00001000U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG2_LDO_CLKTOP_ENABLE_PMOS_PULLDOWN_SHIFT (0x0000000CU)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG2_LDO_CLKTOP_ENABLE_PMOS_PULLDOWN_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG2_LDO_CLKTOP_ENABLE_PMOS_PULLDOWN_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG2_LDO_CLKTOP_TLOAD_CTRL_MASK        (0x0000E000U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG2_LDO_CLKTOP_TLOAD_CTRL_SHIFT       (0x0000000DU)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG2_LDO_CLKTOP_TLOAD_CTRL_RESETVAL    (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG2_LDO_CLKTOP_TLOAD_CTRL_MAX         (0x00000007U)

#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG2_LDO_CLKTOP_TESTMUX_CTRL_MASK      (0x000F0000U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG2_LDO_CLKTOP_TESTMUX_CTRL_SHIFT     (0x00000010U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG2_LDO_CLKTOP_TESTMUX_CTRL_RESETVAL  (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG2_LDO_CLKTOP_TESTMUX_CTRL_MAX       (0x0000000FU)

#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG2_LDO_CLKTOP_BISTMUX_CTRL_MASK      (0x00F00000U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG2_LDO_CLKTOP_BISTMUX_CTRL_SHIFT     (0x00000014U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG2_LDO_CLKTOP_BISTMUX_CTRL_RESETVAL  (0x00000004U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG2_LDO_CLKTOP_BISTMUX_CTRL_MAX       (0x0000000FU)

#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG2_LDO_CLKTOP_RESERVED0_MASK         (0xFF000000U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG2_LDO_CLKTOP_RESERVED0_SHIFT        (0x00000018U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG2_LDO_CLKTOP_RESERVED0_RESETVAL     (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG2_LDO_CLKTOP_RESERVED0_MAX          (0x000000FFU)

#define CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG2_LDO_CLKTOP_RESETVAL               (0x00400710U)

/* ANA_REG_CLK_STATUS_REG */

#define CSL_MSS_TOPRCM_ANA_REG_CLK_STATUS_REG_SLICER_LDO_SC_OUT_MASK           (0x00000001U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_STATUS_REG_SLICER_LDO_SC_OUT_SHIFT          (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_STATUS_REG_SLICER_LDO_SC_OUT_RESETVAL       (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_STATUS_REG_SLICER_LDO_SC_OUT_MAX            (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_CLK_STATUS_REG_APLL_VCO_LDO_SC_OUT_MASK         (0x00000002U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_STATUS_REG_APLL_VCO_LDO_SC_OUT_SHIFT        (0x00000001U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_STATUS_REG_APLL_VCO_LDO_SC_OUT_RESETVAL     (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_STATUS_REG_APLL_VCO_LDO_SC_OUT_MAX          (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_CLK_STATUS_REG_CLKTOP_IOBUF_APLL_LDO_SC_OUT_MASK (0x00000004U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_STATUS_REG_CLKTOP_IOBUF_APLL_LDO_SC_OUT_SHIFT (0x00000002U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_STATUS_REG_CLKTOP_IOBUF_APLL_LDO_SC_OUT_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_STATUS_REG_CLKTOP_IOBUF_APLL_LDO_SC_OUT_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_CLK_STATUS_REG_SDM_LDO_SC_OUT_MASK              (0x00000008U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_STATUS_REG_SDM_LDO_SC_OUT_SHIFT             (0x00000003U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_STATUS_REG_SDM_LDO_SC_OUT_RESETVAL          (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_STATUS_REG_SDM_LDO_SC_OUT_MAX               (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_CLK_STATUS_REG_SYNTH_VCO_LDO_SC_OUT_MASK        (0x00000010U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_STATUS_REG_SYNTH_VCO_LDO_SC_OUT_SHIFT       (0x00000004U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_STATUS_REG_SYNTH_VCO_LDO_SC_OUT_RESETVAL    (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_STATUS_REG_SYNTH_VCO_LDO_SC_OUT_MAX         (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_CLK_STATUS_REG_SYNTH_DIV_LDO_SC_OUT_MASK        (0x00000020U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_STATUS_REG_SYNTH_DIV_LDO_SC_OUT_SHIFT       (0x00000005U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_STATUS_REG_SYNTH_DIV_LDO_SC_OUT_RESETVAL    (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_STATUS_REG_SYNTH_DIV_LDO_SC_OUT_MAX         (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_CLK_STATUS_REG_CLKTOP_IOBUF_ROUTE_LDO_SC_OUT_MASK (0x00000040U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_STATUS_REG_CLKTOP_IOBUF_ROUTE_LDO_SC_OUT_SHIFT (0x00000006U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_STATUS_REG_CLKTOP_IOBUF_ROUTE_LDO_SC_OUT_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_STATUS_REG_CLKTOP_IOBUF_ROUTE_LDO_SC_OUT_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_CLK_STATUS_REG_SYNC_20G_LDO_SC_OUT_MASK         (0x00000080U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_STATUS_REG_SYNC_20G_LDO_SC_OUT_SHIFT        (0x00000007U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_STATUS_REG_SYNC_20G_LDO_SC_OUT_RESETVAL     (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_STATUS_REG_SYNC_20G_LDO_SC_OUT_MAX          (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_CLK_STATUS_REG_CLK_TEST_PATH_LDO_SC_OUT_MASK    (0x00000100U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_STATUS_REG_CLK_TEST_PATH_LDO_SC_OUT_SHIFT   (0x00000008U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_STATUS_REG_CLK_TEST_PATH_LDO_SC_OUT_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_STATUS_REG_CLK_TEST_PATH_LDO_SC_OUT_MAX     (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_CLK_STATUS_REG_RESERVED0_MASK                   (0xFFFFFE00U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_STATUS_REG_RESERVED0_SHIFT                  (0x00000009U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_STATUS_REG_RESERVED0_RESETVAL               (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_CLK_STATUS_REG_RESERVED0_MAX                    (0x007FFFFFU)

#define CSL_MSS_TOPRCM_ANA_REG_CLK_STATUS_REG_RESETVAL                         (0x00000000U)

/* ANA_REG_REFSYS_CTRL_REG_LOWV */

#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_CTRL_REG_LOWV_REFSYS_BGAP_EN_CTRL_MASK   (0x00000001U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_CTRL_REG_LOWV_REFSYS_BGAP_EN_CTRL_SHIFT  (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_CTRL_REG_LOWV_REFSYS_BGAP_EN_CTRL_RESETVAL (0x00000001U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_CTRL_REG_LOWV_REFSYS_BGAP_EN_CTRL_MAX    (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_CTRL_REG_LOWV_REFSYS_V2I_EN_CTRL_MASK    (0x00000002U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_CTRL_REG_LOWV_REFSYS_V2I_EN_CTRL_SHIFT   (0x00000001U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_CTRL_REG_LOWV_REFSYS_V2I_EN_CTRL_RESETVAL (0x00000001U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_CTRL_REG_LOWV_REFSYS_V2I_EN_CTRL_MAX     (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_CTRL_REG_LOWV_REFSYS_CAP_SW_CTRLZ_MASK   (0x00000004U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_CTRL_REG_LOWV_REFSYS_CAP_SW_CTRLZ_SHIFT  (0x00000002U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_CTRL_REG_LOWV_REFSYS_CAP_SW_CTRLZ_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_CTRL_REG_LOWV_REFSYS_CAP_SW_CTRLZ_MAX    (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_CTRL_REG_LOWV_REFSYS_PRE_CHARGE_MASK     (0x00000008U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_CTRL_REG_LOWV_REFSYS_PRE_CHARGE_SHIFT    (0x00000003U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_CTRL_REG_LOWV_REFSYS_PRE_CHARGE_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_CTRL_REG_LOWV_REFSYS_PRE_CHARGE_MAX      (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_CTRL_REG_LOWV_SLOPE_TRIM_4_0_MASK        (0x000001F0U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_CTRL_REG_LOWV_SLOPE_TRIM_4_0_SHIFT       (0x00000004U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_CTRL_REG_LOWV_SLOPE_TRIM_4_0_RESETVAL    (0x0000000DU)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_CTRL_REG_LOWV_SLOPE_TRIM_4_0_MAX         (0x0000001FU)

#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_CTRL_REG_LOWV_MAG_TRIM_4_0_MASK          (0x00003E00U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_CTRL_REG_LOWV_MAG_TRIM_4_0_SHIFT         (0x00000009U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_CTRL_REG_LOWV_MAG_TRIM_4_0_RESETVAL      (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_CTRL_REG_LOWV_MAG_TRIM_4_0_MAX           (0x0000001FU)

#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_CTRL_REG_LOWV_IREF_TRIM_4_0_MASK         (0x0007C000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_CTRL_REG_LOWV_IREF_TRIM_4_0_SHIFT        (0x0000000EU)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_CTRL_REG_LOWV_IREF_TRIM_4_0_RESETVAL     (0x00000002U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_CTRL_REG_LOWV_IREF_TRIM_4_0_MAX          (0x0000001FU)

#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_CTRL_REG_LOWV_BGAP_ISW_MASK              (0x00080000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_CTRL_REG_LOWV_BGAP_ISW_SHIFT             (0x00000013U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_CTRL_REG_LOWV_BGAP_ISW_RESETVAL          (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_CTRL_REG_LOWV_BGAP_ISW_MAX               (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_CTRL_REG_LOWV_V2I_STARTUP_MASK           (0x00100000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_CTRL_REG_LOWV_V2I_STARTUP_SHIFT          (0x00000014U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_CTRL_REG_LOWV_V2I_STARTUP_RESETVAL       (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_CTRL_REG_LOWV_V2I_STARTUP_MAX            (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_CTRL_REG_LOWV_CLKTOP_IBIAS_EN_MASK       (0x00200000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_CTRL_REG_LOWV_CLKTOP_IBIAS_EN_SHIFT      (0x00000015U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_CTRL_REG_LOWV_CLKTOP_IBIAS_EN_RESETVAL   (0x00000001U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_CTRL_REG_LOWV_CLKTOP_IBIAS_EN_MAX        (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_CTRL_REG_LOWV_DO_NOT_USE0_MASK           (0x00400000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_CTRL_REG_LOWV_DO_NOT_USE0_SHIFT          (0x00000016U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_CTRL_REG_LOWV_DO_NOT_USE0_RESETVAL       (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_CTRL_REG_LOWV_DO_NOT_USE0_MAX            (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_CTRL_REG_LOWV_DO_NOT_USE1_MASK           (0x00800000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_CTRL_REG_LOWV_DO_NOT_USE1_SHIFT          (0x00000017U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_CTRL_REG_LOWV_DO_NOT_USE1_RESETVAL       (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_CTRL_REG_LOWV_DO_NOT_USE1_MAX            (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_CTRL_REG_LOWV_REFSYS_BYPASS_EN_MASK      (0x01000000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_CTRL_REG_LOWV_REFSYS_BYPASS_EN_SHIFT     (0x00000018U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_CTRL_REG_LOWV_REFSYS_BYPASS_EN_RESETVAL  (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_CTRL_REG_LOWV_REFSYS_BYPASS_EN_MAX       (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_CTRL_REG_LOWV_DO_NOT_USE2_MASK           (0x02000000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_CTRL_REG_LOWV_DO_NOT_USE2_SHIFT          (0x00000019U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_CTRL_REG_LOWV_DO_NOT_USE2_RESETVAL       (0x00000001U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_CTRL_REG_LOWV_DO_NOT_USE2_MAX            (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_CTRL_REG_LOWV_DO_NOT_USE3_MASK           (0x04000000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_CTRL_REG_LOWV_DO_NOT_USE3_SHIFT          (0x0000001AU)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_CTRL_REG_LOWV_DO_NOT_USE3_RESETVAL       (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_CTRL_REG_LOWV_DO_NOT_USE3_MAX            (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_CTRL_REG_LOWV_FTRIM_3_0_MASK             (0x78000000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_CTRL_REG_LOWV_FTRIM_3_0_SHIFT            (0x0000001BU)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_CTRL_REG_LOWV_FTRIM_3_0_RESETVAL         (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_CTRL_REG_LOWV_FTRIM_3_0_MAX              (0x0000000FU)

#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_CTRL_REG_LOWV_RESERVED0_MASK             (0x80000000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_CTRL_REG_LOWV_RESERVED0_SHIFT            (0x0000001FU)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_CTRL_REG_LOWV_RESERVED0_RESETVAL         (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_CTRL_REG_LOWV_RESERVED0_MAX              (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_CTRL_REG_LOWV_RESETVAL                   (0x022080D3U)

/* ANA_REG_REFSYS_TMUX_CTRL_LOWV */

#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_TMUX_CTRL_LOWV_VREF_0P45V_MASK           (0x00000001U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_TMUX_CTRL_LOWV_VREF_0P45V_SHIFT          (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_TMUX_CTRL_LOWV_VREF_0P45V_RESETVAL       (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_TMUX_CTRL_LOWV_VREF_0P45V_MAX            (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_TMUX_CTRL_LOWV_VREF_0P9V_MASK            (0x00000002U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_TMUX_CTRL_LOWV_VREF_0P9V_SHIFT           (0x00000001U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_TMUX_CTRL_LOWV_VREF_0P9V_RESETVAL        (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_TMUX_CTRL_LOWV_VREF_0P9V_MAX             (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_TMUX_CTRL_LOWV_VBG_1P22V_MASK            (0x00000004U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_TMUX_CTRL_LOWV_VBG_1P22V_SHIFT           (0x00000002U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_TMUX_CTRL_LOWV_VBG_1P22V_RESETVAL        (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_TMUX_CTRL_LOWV_VBG_1P22V_MAX             (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_TMUX_CTRL_LOWV_RX_IBIASP_20U_MASK        (0x00000008U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_TMUX_CTRL_LOWV_RX_IBIASP_20U_SHIFT       (0x00000003U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_TMUX_CTRL_LOWV_RX_IBIASP_20U_RESETVAL    (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_TMUX_CTRL_LOWV_RX_IBIASP_20U_MAX         (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_TMUX_CTRL_LOWV_VBE_WEAK_MASK             (0x00000010U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_TMUX_CTRL_LOWV_VBE_WEAK_SHIFT            (0x00000004U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_TMUX_CTRL_LOWV_VBE_WEAK_RESETVAL         (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_TMUX_CTRL_LOWV_VBE_WEAK_MAX              (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_TMUX_CTRL_LOWV_RESERVED0_MASK            (0x00000020U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_TMUX_CTRL_LOWV_RESERVED0_SHIFT           (0x00000005U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_TMUX_CTRL_LOWV_RESERVED0_RESETVAL        (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_TMUX_CTRL_LOWV_RESERVED0_MAX             (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_TMUX_CTRL_LOWV_IBIASP_20U_MASK           (0x00000040U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_TMUX_CTRL_LOWV_IBIASP_20U_SHIFT          (0x00000006U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_TMUX_CTRL_LOWV_IBIASP_20U_RESETVAL       (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_TMUX_CTRL_LOWV_IBIASP_20U_MAX            (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_TMUX_CTRL_LOWV_IBIASP_TS_6U_MASK         (0x00000080U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_TMUX_CTRL_LOWV_IBIASP_TS_6U_SHIFT        (0x00000007U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_TMUX_CTRL_LOWV_IBIASP_TS_6U_RESETVAL     (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_TMUX_CTRL_LOWV_IBIASP_TS_6U_MAX          (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_TMUX_CTRL_LOWV_RESERVED1_MASK            (0x00000100U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_TMUX_CTRL_LOWV_RESERVED1_SHIFT           (0x00000008U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_TMUX_CTRL_LOWV_RESERVED1_RESETVAL        (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_TMUX_CTRL_LOWV_RESERVED1_MAX             (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_TMUX_CTRL_LOWV_IDIODEP_100U_MASK         (0x00000200U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_TMUX_CTRL_LOWV_IDIODEP_100U_SHIFT        (0x00000009U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_TMUX_CTRL_LOWV_IDIODEP_100U_RESETVAL     (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_TMUX_CTRL_LOWV_IDIODEP_100U_MAX          (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_TMUX_CTRL_LOWV_IREFP_10UA_MASK           (0x00000400U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_TMUX_CTRL_LOWV_IREFP_10UA_SHIFT          (0x0000000AU)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_TMUX_CTRL_LOWV_IREFP_10UA_RESETVAL       (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_TMUX_CTRL_LOWV_IREFP_10UA_MAX            (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_TMUX_CTRL_LOWV_RESERVED2_MASK            (0x00000800U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_TMUX_CTRL_LOWV_RESERVED2_SHIFT           (0x0000000BU)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_TMUX_CTRL_LOWV_RESERVED2_RESETVAL        (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_TMUX_CTRL_LOWV_RESERVED2_MAX             (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_TMUX_CTRL_LOWV_I2V_SENSE_MASK            (0x00001000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_TMUX_CTRL_LOWV_I2V_SENSE_SHIFT           (0x0000000CU)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_TMUX_CTRL_LOWV_I2V_SENSE_RESETVAL        (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_TMUX_CTRL_LOWV_I2V_SENSE_MAX             (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_TMUX_CTRL_LOWV_BYPASS_MIRR_VPBIAS_MASK   (0x00002000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_TMUX_CTRL_LOWV_BYPASS_MIRR_VPBIAS_SHIFT  (0x0000000DU)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_TMUX_CTRL_LOWV_BYPASS_MIRR_VPBIAS_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_TMUX_CTRL_LOWV_BYPASS_MIRR_VPBIAS_MAX    (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_TMUX_CTRL_LOWV_TX_IBIASP_20U_MASK        (0x00004000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_TMUX_CTRL_LOWV_TX_IBIASP_20U_SHIFT       (0x0000000EU)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_TMUX_CTRL_LOWV_TX_IBIASP_20U_RESETVAL    (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_TMUX_CTRL_LOWV_TX_IBIASP_20U_MAX         (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_TMUX_CTRL_LOWV_LO_IBIASP_20U_MASK        (0x00008000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_TMUX_CTRL_LOWV_LO_IBIASP_20U_SHIFT       (0x0000000FU)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_TMUX_CTRL_LOWV_LO_IBIASP_20U_RESETVAL    (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_TMUX_CTRL_LOWV_LO_IBIASP_20U_MAX         (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_TMUX_CTRL_LOWV_RESERVED3_MASK            (0x7FFF0000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_TMUX_CTRL_LOWV_RESERVED3_SHIFT           (0x00000010U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_TMUX_CTRL_LOWV_RESERVED3_RESETVAL        (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_TMUX_CTRL_LOWV_RESERVED3_MAX             (0x00007FFFU)

#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_TMUX_CTRL_LOWV_REFSYS_CTRL_8_MASK        (0x80000000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_TMUX_CTRL_LOWV_REFSYS_CTRL_8_SHIFT       (0x0000001FU)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_TMUX_CTRL_LOWV_REFSYS_CTRL_8_RESETVAL    (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_TMUX_CTRL_LOWV_REFSYS_CTRL_8_MAX         (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_TMUX_CTRL_LOWV_RESETVAL                  (0x00000000U)

/* ANA_REG_REFSYS_SPARE_REG_LOWV */

#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_RESERVED0_MASK            (0x00000001U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_RESERVED0_SHIFT           (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_RESERVED0_RESETVAL        (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_RESERVED0_MAX             (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_RESERVED1_MASK            (0x0000003EU)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_RESERVED1_SHIFT           (0x00000001U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_RESERVED1_RESETVAL        (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_RESERVED1_MAX             (0x0000001FU)

#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDD_SR_SEL_MASK           (0x000000C0U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDD_SR_SEL_SHIFT          (0x00000006U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDD_SR_SEL_RESETVAL       (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDD_SR_SEL_MAX            (0x00000003U)

#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDD_UV_SELF_TEST_SEL_MASK (0x00000100U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDD_UV_SELF_TEST_SEL_SHIFT (0x00000008U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDD_UV_SELF_TEST_SEL_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDD_UV_SELF_TEST_SEL_MAX  (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDD_OV_SELF_TEST_SEL_MASK (0x00000200U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDD_OV_SELF_TEST_SEL_SHIFT (0x00000009U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDD_OV_SELF_TEST_SEL_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDD_OV_SELF_TEST_SEL_MAX  (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDDA_OSC_UV_SELF_TEST_SEL_MASK (0x00000400U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDDA_OSC_UV_SELF_TEST_SEL_SHIFT (0x0000000AU)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDDA_OSC_UV_SELF_TEST_SEL_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDDA_OSC_UV_SELF_TEST_SEL_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_RESERVED2_MASK            (0x00000800U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_RESERVED2_SHIFT           (0x0000000BU)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_RESERVED2_RESETVAL        (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_RESERVED2_MAX             (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDDS_3P3V_UV_SELF_TEST_SEL_MASK (0x00001000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDDS_3P3V_UV_SELF_TEST_SEL_SHIFT (0x0000000CU)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDDS_3P3V_UV_SELF_TEST_SEL_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDDS_3P3V_UV_SELF_TEST_SEL_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_RESERVED3_MASK            (0x00002000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_RESERVED3_SHIFT           (0x0000000DU)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_RESERVED3_RESETVAL        (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_RESERVED3_MAX             (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDD_OV_IR_DROP_COMP_SEL_MASK (0x0000C000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDD_OV_IR_DROP_COMP_SEL_SHIFT (0x0000000EU)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDD_OV_IR_DROP_COMP_SEL_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDD_OV_IR_DROP_COMP_SEL_MAX (0x00000003U)

#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDD_OV_SR_SEL_MASK        (0x00030000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDD_OV_SR_SEL_SHIFT       (0x00000010U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDD_OV_SR_SEL_RESETVAL    (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDD_OV_SR_SEL_MAX         (0x00000003U)

#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VIOIN_UV_RSET_MASK_MASK   (0x00040000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VIOIN_UV_RSET_MASK_SHIFT  (0x00000012U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VIOIN_UV_RSET_MASK_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VIOIN_UV_RSET_MASK_MAX    (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDDA_OSC_UV_RSET_MASK_MASK (0x00080000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDDA_OSC_UV_RSET_MASK_SHIFT (0x00000013U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDDA_OSC_UV_RSET_MASK_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDDA_OSC_UV_RSET_MASK_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDD_UV_RSET_MASK_MASK     (0x00100000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDD_UV_RSET_MASK_SHIFT    (0x00000014U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDD_UV_RSET_MASK_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDD_UV_RSET_MASK_MAX      (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDD_OV_RSET_MASK_MASK     (0x00200000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDD_OV_RSET_MASK_SHIFT    (0x00000015U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDD_OV_RSET_MASK_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDD_OV_RSET_MASK_MAX      (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDD_IR_DROP_COMP_SEL_MASK (0x00C00000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDD_IR_DROP_COMP_SEL_SHIFT (0x00000016U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDD_IR_DROP_COMP_SEL_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDD_IR_DROP_COMP_SEL_MAX  (0x00000003U)

#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDDS_3P3V_IR_DROP_COMP_SEL_MASK (0x03000000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDDS_3P3V_IR_DROP_COMP_SEL_SHIFT (0x00000018U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDDS_3P3V_IR_DROP_COMP_SEL_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDDS_3P3V_IR_DROP_COMP_SEL_MAX (0x00000003U)

#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDDA_OSC_IR_DROP_COMP_SEL_MASK (0x0C000000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDDA_OSC_IR_DROP_COMP_SEL_SHIFT (0x0000001AU)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDDA_OSC_IR_DROP_COMP_SEL_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDDA_OSC_IR_DROP_COMP_SEL_MAX (0x00000003U)

#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_RESERVED5_MASK            (0x70000000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_RESERVED5_SHIFT           (0x0000001CU)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_RESERVED5_RESETVAL        (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_RESERVED5_MAX             (0x00000007U)

#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_RESERVED6_MASK            (0x80000000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_RESERVED6_SHIFT           (0x0000001FU)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_RESERVED6_RESETVAL        (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_RESERVED6_MAX             (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_RESETVAL                  (0x00000000U)

/* ANA_REG_WU_CTRL_REG_LOWV */

#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_INT_CLK_EN_MASK                (0x00000001U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_INT_CLK_EN_SHIFT               (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_INT_CLK_EN_RESETVAL            (0x00000001U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_INT_CLK_EN_MAX                 (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_INT_CLK_STOP_MASK              (0x00000002U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_INT_CLK_STOP_SHIFT             (0x00000001U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_INT_CLK_STOP_RESETVAL          (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_INT_CLK_STOP_MAX               (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_INT_CLK_SW_SEL_MASK            (0x00000004U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_INT_CLK_SW_SEL_SHIFT           (0x00000002U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_INT_CLK_SW_SEL_RESETVAL        (0x00000001U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_INT_CLK_SW_SEL_MAX             (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_INT_CLK_TRIM_6_0_MASK          (0x000003F8U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_INT_CLK_TRIM_6_0_SHIFT         (0x00000003U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_INT_CLK_TRIM_6_0_RESETVAL      (0x0000004BU)
#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_INT_CLK_TRIM_6_0_MAX           (0x0000007FU)

#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_ECO_SLICER_CLK_DLY_DIS_MASK    (0x00000400U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_ECO_SLICER_CLK_DLY_DIS_SHIFT   (0x0000000AU)
#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_ECO_SLICER_CLK_DLY_DIS_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_ECO_SLICER_CLK_DLY_DIS_MAX     (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_INT_CLK_FREQ_SEL_3_0_MASK      (0x00007800U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_INT_CLK_FREQ_SEL_3_0_SHIFT     (0x0000000BU)
#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_INT_CLK_FREQ_SEL_3_0_RESETVAL  (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_INT_CLK_FREQ_SEL_3_0_MAX       (0x0000000FU)

#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_CPU_CLK_CTRL_MASK           (0x00008000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_CPU_CLK_CTRL_SHIFT          (0x0000000FU)
#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_CPU_CLK_CTRL_RESETVAL       (0x00000001U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_CPU_CLK_CTRL_MAX            (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_XTAL_EN_OVERRIDE_MASK          (0x00010000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_XTAL_EN_OVERRIDE_SHIFT         (0x00000010U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_XTAL_EN_OVERRIDE_RESETVAL      (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_XTAL_EN_OVERRIDE_MAX           (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_UV_DET_CTRL_MASK            (0x00020000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_UV_DET_CTRL_SHIFT           (0x00000011U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_UV_DET_CTRL_RESETVAL        (0x00000001U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_UV_DET_CTRL_MAX             (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_OV_DET_CTRL_MASK            (0x00040000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_OV_DET_CTRL_SHIFT           (0x00000012U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_OV_DET_CTRL_RESETVAL        (0x00000001U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_OV_DET_CTRL_MAX             (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_XTAL_DLY_CTRL_MASK          (0x00080000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_XTAL_DLY_CTRL_SHIFT         (0x00000013U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_XTAL_DLY_CTRL_RESETVAL      (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_XTAL_DLY_CTRL_MAX           (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_SUPP_VMON_EN_MASK           (0x00100000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_SUPP_VMON_EN_SHIFT          (0x00000014U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_SUPP_VMON_EN_RESETVAL       (0x00000001U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_SUPP_VMON_EN_MAX            (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_VRAM_VMON_EN_MASK           (0x00200000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_VRAM_VMON_EN_SHIFT          (0x00000015U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_VRAM_VMON_EN_RESETVAL       (0x00000001U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_VRAM_VMON_EN_MAX            (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_SUPP_DET_CTRL_MASK          (0x00400000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_SUPP_DET_CTRL_SHIFT         (0x00000016U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_SUPP_DET_CTRL_RESETVAL      (0x00000001U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_SUPP_DET_CTRL_MAX           (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_SPARE_IN_MASK               (0x01800000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_SPARE_IN_SHIFT              (0x00000017U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_SPARE_IN_RESETVAL           (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_SPARE_IN_MAX                (0x00000003U)

#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_VDDS_3P3V_UV_VMON_EN_MASK   (0x02000000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_VDDS_3P3V_UV_VMON_EN_SHIFT  (0x00000019U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_VDDS_3P3V_UV_VMON_EN_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_VDDS_3P3V_UV_VMON_EN_MAX    (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_VDDA_OSC_UV_VMON_EN_MASK    (0x04000000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_VDDA_OSC_UV_VMON_EN_SHIFT   (0x0000001AU)
#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_VDDA_OSC_UV_VMON_EN_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_VDDA_OSC_UV_VMON_EN_MAX     (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_VDD_UV_VMON_EN_MASK         (0x08000000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_VDD_UV_VMON_EN_SHIFT        (0x0000001BU)
#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_VDD_UV_VMON_EN_RESETVAL     (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_VDD_UV_VMON_EN_MAX          (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_VDD_OV_VMON_EN_MASK         (0x10000000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_VDD_OV_VMON_EN_SHIFT        (0x0000001CU)
#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_VDD_OV_VMON_EN_RESETVAL     (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_VDD_OV_VMON_EN_MAX          (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_SPARE_IN_2_MASK             (0x60000000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_SPARE_IN_2_SHIFT            (0x0000001DU)
#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_SPARE_IN_2_RESETVAL         (0x00000003U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_SPARE_IN_2_MAX              (0x00000003U)

#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_RESERVED0_MASK                 (0x80000000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_RESERVED0_SHIFT                (0x0000001FU)
#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_RESERVED0_RESETVAL             (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_RESERVED0_MAX                  (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_RESETVAL                       (0x6076825DU)

/* ANA_REG_WU_TMUX_CTRL_LOWV */

#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_SCALED_VDDA_OSC_DIV3_MASK     (0x00000001U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_SCALED_VDDA_OSC_DIV3_SHIFT    (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_SCALED_VDDA_OSC_DIV3_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_SCALED_VDDA_OSC_DIV3_MAX      (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_SCALED_VIO3318_MASK           (0x00000002U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_SCALED_VIO3318_SHIFT          (0x00000001U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_SCALED_VIO3318_RESETVAL       (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_SCALED_VIO3318_MAX            (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_SCALED_VDDA18BB_MASK          (0x00000004U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_SCALED_VDDA18BB_SHIFT         (0x00000002U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_SCALED_VDDA18BB_RESETVAL      (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_SCALED_VDDA18BB_MAX           (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_VFB_0P6V_MASK                 (0x00000008U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_VFB_0P6V_SHIFT                (0x00000003U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_VFB_0P6V_RESETVAL             (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_VFB_0P6V_MAX                  (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_SCALED_VDDA18VCO_MASK         (0x00000010U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_SCALED_VDDA18VCO_SHIFT        (0x00000004U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_SCALED_VDDA18VCO_RESETVAL     (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_SCALED_VDDA18VCO_MAX          (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_SCALED_VDDA10RF1_MASK         (0x00000020U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_SCALED_VDDA10RF1_SHIFT        (0x00000005U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_SCALED_VDDA10RF1_RESETVAL     (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_SCALED_VDDA10RF1_MAX          (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_SCALED_VDDA10RF2_MASK         (0x00000040U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_SCALED_VDDA10RF2_SHIFT        (0x00000006U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_SCALED_VDDA10RF2_RESETVAL     (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_SCALED_VDDA10RF2_MAX          (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_VREF_0P9V_MASK                (0x00000080U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_VREF_0P9V_SHIFT               (0x00000007U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_VREF_0P9V_RESETVAL            (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_VREF_0P9V_MAX                 (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_VDDA_OSC_UV_VREF_MASK         (0x00000100U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_VDDA_OSC_UV_VREF_SHIFT        (0x00000008U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_VDDA_OSC_UV_VREF_RESETVAL     (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_VDDA_OSC_UV_VREF_MAX          (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_VIOIN33_UV_VREF_MASK          (0x00000200U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_VIOIN33_UV_VREF_SHIFT         (0x00000009U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_VIOIN33_UV_VREF_RESETVAL      (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_VIOIN33_UV_VREF_MAX           (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_VDDA18BB_UV_VREF_MASK         (0x00000400U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_VDDA18BB_UV_VREF_SHIFT        (0x0000000AU)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_VDDA18BB_UV_VREF_RESETVAL     (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_VDDA18BB_UV_VREF_MAX          (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_VDD_OV_VREF_MASK              (0x00000800U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_VDD_OV_VREF_SHIFT             (0x0000000BU)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_VDD_OV_VREF_RESETVAL          (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_VDD_OV_VREF_MAX               (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_VDD_UV_VREF_MASK              (0x00001000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_VDD_UV_VREF_SHIFT             (0x0000000CU)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_VDD_UV_VREF_RESETVAL          (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_VDD_UV_VREF_MAX               (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_VDDA18VCO_UV_VREF_MASK        (0x00002000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_VDDA18VCO_UV_VREF_SHIFT       (0x0000000DU)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_VDDA18VCO_UV_VREF_RESETVAL    (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_VDDA18VCO_UV_VREF_MAX         (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_VDDA10RF1_UV_VREF_MASK        (0x00004000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_VDDA10RF1_UV_VREF_SHIFT       (0x0000000EU)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_VDDA10RF1_UV_VREF_RESETVAL    (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_VDDA10RF1_UV_VREF_MAX         (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_VDDA10RF2_UV_VREF_MASK        (0x00008000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_VDDA10RF2_UV_VREF_SHIFT       (0x0000000FU)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_VDDA10RF2_UV_VREF_RESETVAL    (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_VDDA10RF2_UV_VREF_MAX         (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_VT_ANA_SIG_MASK               (0x00010000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_VT_ANA_SIG_SHIFT              (0x00000010U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_VT_ANA_SIG_RESETVAL           (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_VT_ANA_SIG_MAX                (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_VT_DIG_SIG_UV_MASK            (0x00020000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_VT_DIG_SIG_UV_SHIFT           (0x00000011U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_VT_DIG_SIG_UV_RESETVAL        (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_VT_DIG_SIG_UV_MAX             (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_VT_DIG_SIG_OV_MASK            (0x00040000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_VT_DIG_SIG_OV_SHIFT           (0x00000012U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_VT_DIG_SIG_OV_RESETVAL        (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_VT_DIG_SIG_OV_MAX             (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_SCALED_VDDA_OSC_DIV22_39_MASK (0x00080000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_SCALED_VDDA_OSC_DIV22_39_SHIFT (0x00000013U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_SCALED_VDDA_OSC_DIV22_39_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_SCALED_VDDA_OSC_DIV22_39_MAX  (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_VDDSINT18_MASK                (0x00100000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_VDDSINT18_SHIFT               (0x00000014U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_VDDSINT18_RESETVAL            (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_VDDSINT18_MAX                 (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_VFB_0P85V_MASK                (0x00200000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_VFB_0P85V_SHIFT               (0x00000015U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_VFB_0P85V_RESETVAL            (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_VFB_0P85V_MAX                 (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_SCALED_VDDA_LVDS_1P8V_1P2_MASK (0x00400000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_SCALED_VDDA_LVDS_1P8V_1P2_SHIFT (0x00000016U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_SCALED_VDDA_LVDS_1P8V_1P2_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_SCALED_VDDA_LVDS_1P8V_1P2_MAX (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_SCALED_VDDA_LVDS_1P8V_MASK    (0x00800000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_SCALED_VDDA_LVDS_1P8V_SHIFT   (0x00000017U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_SCALED_VDDA_LVDS_1P8V_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_SCALED_VDDA_LVDS_1P8V_MAX     (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_SCALED_VDDS33_MASK            (0x01000000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_SCALED_VDDS33_SHIFT           (0x00000018U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_SCALED_VDDS33_RESETVAL        (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_SCALED_VDDS33_MAX             (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_RESERVED0_MASK                (0x7E000000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_RESERVED0_SHIFT               (0x00000019U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_RESERVED0_RESETVAL            (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_RESERVED0_MAX                 (0x0000003FU)

#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_WU_TMUX_EN_MASK               (0x80000000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_WU_TMUX_EN_SHIFT              (0x0000001FU)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_WU_TMUX_EN_RESETVAL           (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_WU_TMUX_EN_MAX                (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_WU_TMUX_CTRL_LOWV_RESETVAL                      (0x00000000U)

/* ANA_REG_TW_CTRL_REG_LOWV */

#define CSL_MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_ADC_EN_MASK                    (0x00000001U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_ADC_EN_SHIFT                   (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_ADC_EN_RESETVAL                (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_ADC_EN_MAX                     (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_RESERVED_MASK                  (0x00000002U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_RESERVED_SHIFT                 (0x00000001U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_RESERVED_RESETVAL              (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_RESERVED_MAX                   (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_ADC_RESET_MASK                 (0x00000004U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_ADC_RESET_SHIFT                (0x00000002U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_ADC_RESET_RESETVAL             (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_ADC_RESET_MAX                  (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_ADC_INP_BUF_EN_MASK            (0x00000008U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_ADC_INP_BUF_EN_SHIFT           (0x00000003U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_ADC_INP_BUF_EN_RESETVAL        (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_ADC_INP_BUF_EN_MAX             (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_ADC_REF_BUF_EN_MASK            (0x00000010U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_ADC_REF_BUF_EN_SHIFT           (0x00000004U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_ADC_REF_BUF_EN_RESETVAL        (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_ADC_REF_BUF_EN_MAX             (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_ADC_REF_SEL_2_0_MASK           (0x000000E0U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_ADC_REF_SEL_2_0_SHIFT          (0x00000005U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_ADC_REF_SEL_2_0_RESETVAL       (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_ADC_REF_SEL_2_0_MAX            (0x00000007U)

#define CSL_MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_TS_DIFF_INP_BUF_EN_MASK        (0x00000100U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_TS_DIFF_INP_BUF_EN_SHIFT       (0x00000008U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_TS_DIFF_INP_BUF_EN_RESETVAL    (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_TS_DIFF_INP_BUF_EN_MAX         (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_TS_SE_INP_BUF_EN_MASK          (0x00000200U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_TS_SE_INP_BUF_EN_SHIFT         (0x00000009U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_TS_SE_INP_BUF_EN_RESETVAL      (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_TS_SE_INP_BUF_EN_MAX           (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_IFORCE_EXT_CTRL_MASK           (0x00000400U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_IFORCE_EXT_CTRL_SHIFT          (0x0000000AU)
#define CSL_MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_IFORCE_EXT_CTRL_RESETVAL       (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_IFORCE_EXT_CTRL_MAX            (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_VREF_EXT_CTRL_MASK             (0x00000800U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_VREF_EXT_CTRL_SHIFT            (0x0000000BU)
#define CSL_MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_VREF_EXT_CTRL_RESETVAL         (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_VREF_EXT_CTRL_MAX              (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_VIN_EXT_CTRL_MASK              (0x00001000U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_VIN_EXT_CTRL_SHIFT             (0x0000000CU)
#define CSL_MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_VIN_EXT_CTRL_RESETVAL          (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_VIN_EXT_CTRL_MAX               (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_ANA_TMUX_BUF_BYPASS_MASK       (0x00002000U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_ANA_TMUX_BUF_BYPASS_SHIFT      (0x0000000DU)
#define CSL_MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_ANA_TMUX_BUF_BYPASS_RESETVAL   (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_ANA_TMUX_BUF_BYPASS_MAX        (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_ANA_TMUX_BUF_EN_MASK           (0x00004000U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_ANA_TMUX_BUF_EN_SHIFT          (0x0000000EU)
#define CSL_MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_ANA_TMUX_BUF_EN_RESETVAL       (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_ANA_TMUX_BUF_EN_MAX            (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_RESERVED0_MASK                 (0xFFFF8000U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_RESERVED0_SHIFT                (0x0000000FU)
#define CSL_MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_RESERVED0_RESETVAL             (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_RESERVED0_MAX                  (0x0001FFFFU)

#define CSL_MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_RESETVAL                       (0x00000000U)

/* ANA_REG_TW_ANA_TMUX_CTRL_LOWV */

#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE0_MASK          (0x00000001U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE0_SHIFT         (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE0_RESETVAL      (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE0_MAX           (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE1_MASK          (0x00000002U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE1_SHIFT         (0x00000001U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE1_RESETVAL      (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE1_MAX           (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE2_MASK          (0x00000004U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE2_SHIFT         (0x00000002U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE2_RESETVAL      (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE2_MAX           (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE3_MASK          (0x00000008U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE3_SHIFT         (0x00000003U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE3_RESETVAL      (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE3_MAX           (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE4_MASK          (0x00000010U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE4_SHIFT         (0x00000004U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE4_RESETVAL      (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE4_MAX           (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE5_MASK          (0x00000020U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE5_SHIFT         (0x00000005U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE5_RESETVAL      (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE5_MAX           (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE6_MASK          (0x00000040U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE6_SHIFT         (0x00000006U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE6_RESETVAL      (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE6_MAX           (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE7_MASK          (0x00000080U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE7_SHIFT         (0x00000007U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE7_RESETVAL      (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE7_MAX           (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE8_MASK          (0x00000100U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE8_SHIFT         (0x00000008U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE8_RESETVAL      (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE8_MAX           (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE9_MASK          (0x00000200U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE9_SHIFT         (0x00000009U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE9_RESETVAL      (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE9_MAX           (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE10_MASK         (0x00000400U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE10_SHIFT        (0x0000000AU)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE10_RESETVAL     (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE10_MAX          (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE11_MASK         (0x00000800U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE11_SHIFT        (0x0000000BU)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE11_RESETVAL     (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE11_MAX          (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE12_MASK         (0x00001000U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE12_SHIFT        (0x0000000CU)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE12_RESETVAL     (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE12_MAX          (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE13_MASK         (0x00002000U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE13_SHIFT        (0x0000000DU)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE13_RESETVAL     (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE13_MAX          (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE14_MASK         (0x00004000U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE14_SHIFT        (0x0000000EU)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE14_RESETVAL     (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE14_MAX          (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE15_MASK         (0x00008000U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE15_SHIFT        (0x0000000FU)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE15_RESETVAL     (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE15_MAX          (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE16_MASK         (0x00010000U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE16_SHIFT        (0x00000010U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE16_RESETVAL     (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE16_MAX          (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE17_MASK         (0x00020000U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE17_SHIFT        (0x00000011U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE17_RESETVAL     (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE17_MAX          (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE18_MASK         (0x00040000U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE18_SHIFT        (0x00000012U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE18_RESETVAL     (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE18_MAX          (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_BIST_MUX_OUT_1P8V_MASK    (0x00080000U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_BIST_MUX_OUT_1P8V_SHIFT   (0x00000013U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_BIST_MUX_OUT_1P8V_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_BIST_MUX_OUT_1P8V_MAX     (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_ADC_BUF_OUT_1P8V_MASK     (0x00100000U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_ADC_BUF_OUT_1P8V_SHIFT    (0x00000014U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_ADC_BUF_OUT_1P8V_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_ADC_BUF_OUT_1P8V_MAX      (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_ADC_REF_BUF_OUT_MASK      (0x00200000U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_ADC_REF_BUF_OUT_SHIFT     (0x00000015U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_ADC_REF_BUF_OUT_RESETVAL  (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_ADC_REF_BUF_OUT_MAX       (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE19_MASK         (0x00400000U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE19_SHIFT        (0x00000016U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE19_RESETVAL     (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE19_MAX          (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_RESERVED0_MASK            (0x3F800000U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_RESERVED0_SHIFT           (0x00000017U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_RESERVED0_RESETVAL        (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_RESERVED0_MAX             (0x0000007FU)

#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE20_MASK         (0x40000000U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE20_SHIFT        (0x0000001EU)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE20_RESETVAL     (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE20_MAX          (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE21_MASK         (0x80000000U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE21_SHIFT        (0x0000001FU)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE21_RESETVAL     (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_DO_NOT_USE21_MAX          (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_RESETVAL                  (0x00000000U)

/* ANA_REG_WU_MODE_REG_LOWV */

#define CSL_MSS_TOPRCM_ANA_REG_WU_MODE_REG_LOWV_FUNC_TEST_DET_SYNC_MASK        (0x00000001U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_MODE_REG_LOWV_FUNC_TEST_DET_SYNC_SHIFT       (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_MODE_REG_LOWV_FUNC_TEST_DET_SYNC_RESETVAL    (0x00000001U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_MODE_REG_LOWV_FUNC_TEST_DET_SYNC_MAX         (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_WU_MODE_REG_LOWV_TEST_MODE_DET_SYNC_MASK        (0x00000002U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_MODE_REG_LOWV_TEST_MODE_DET_SYNC_SHIFT       (0x00000001U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_MODE_REG_LOWV_TEST_MODE_DET_SYNC_RESETVAL    (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_MODE_REG_LOWV_TEST_MODE_DET_SYNC_MAX         (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_WU_MODE_REG_LOWV_SOP_MODE_LAT_4_0_MASK          (0x0000007CU)
#define CSL_MSS_TOPRCM_ANA_REG_WU_MODE_REG_LOWV_SOP_MODE_LAT_4_0_SHIFT         (0x00000002U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_MODE_REG_LOWV_SOP_MODE_LAT_4_0_RESETVAL      (0x00000001U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_MODE_REG_LOWV_SOP_MODE_LAT_4_0_MAX           (0x0000001FU)

#define CSL_MSS_TOPRCM_ANA_REG_WU_MODE_REG_LOWV_RESERVED0_MASK                 (0xFFFFFF80U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_MODE_REG_LOWV_RESERVED0_SHIFT                (0x00000007U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_MODE_REG_LOWV_RESERVED0_RESETVAL             (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_MODE_REG_LOWV_RESERVED0_MAX                  (0x01FFFFFFU)

#define CSL_MSS_TOPRCM_ANA_REG_WU_MODE_REG_LOWV_RESETVAL                       (0x00000005U)

/* ANA_REG_WU_STATUS_REG_LOWV */

#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_CORE_OVDET_LAT_MASK          (0x00000001U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_CORE_OVDET_LAT_SHIFT         (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_CORE_OVDET_LAT_RESETVAL      (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_CORE_OVDET_LAT_MAX           (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_CORE_UVDET_LAT_MASK          (0x00000002U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_CORE_UVDET_LAT_SHIFT         (0x00000001U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_CORE_UVDET_LAT_RESETVAL      (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_CORE_UVDET_LAT_MAX           (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_VDDA18BB_UV_DET_LAT_MASK     (0x00000004U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_VDDA18BB_UV_DET_LAT_SHIFT    (0x00000002U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_VDDA18BB_UV_DET_LAT_RESETVAL (0x00000001U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_VDDA18BB_UV_DET_LAT_MAX      (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_SUPP_OK_CLK18_MASK           (0x00000008U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_SUPP_OK_CLK18_SHIFT          (0x00000003U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_SUPP_OK_CLK18_RESETVAL       (0x00000001U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_SUPP_OK_CLK18_MAX            (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_SUPP_OK_IO18_MASK            (0x00000010U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_SUPP_OK_IO18_SHIFT           (0x00000004U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_SUPP_OK_IO18_RESETVAL        (0x00000001U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_SUPP_OK_IO18_MAX             (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_SUPP_OK_IO33_MASK            (0x00000020U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_SUPP_OK_IO33_SHIFT           (0x00000005U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_SUPP_OK_IO33_RESETVAL        (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_SUPP_OK_IO33_MAX             (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_SUPP_OK_RF10_MASK            (0x00000040U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_SUPP_OK_RF10_SHIFT           (0x00000006U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_SUPP_OK_RF10_RESETVAL        (0x00000001U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_SUPP_OK_RF10_MAX             (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_VDDA10RF1_UVDET_LAT_MASK     (0x00000080U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_VDDA10RF1_UVDET_LAT_SHIFT    (0x00000007U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_VDDA10RF1_UVDET_LAT_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_VDDA10RF1_UVDET_LAT_MAX      (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_VDDA10RF2_UVDET_LAT_MASK     (0x00000100U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_VDDA10RF2_UVDET_LAT_SHIFT    (0x00000008U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_VDDA10RF2_UVDET_LAT_RESETVAL (0x00000001U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_VDDA10RF2_UVDET_LAT_MAX      (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_SUPP_OK_SRAM12_MASK          (0x00000200U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_SUPP_OK_SRAM12_SHIFT         (0x00000009U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_SUPP_OK_SRAM12_RESETVAL      (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_SUPP_OK_SRAM12_MAX           (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_SUPP_OK_VDDD18_MASK          (0x00000400U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_SUPP_OK_VDDD18_SHIFT         (0x0000000AU)
#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_SUPP_OK_VDDD18_RESETVAL      (0x00000001U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_SUPP_OK_VDDD18_MAX           (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_REF_CLK_STATUS_MASK          (0x00000800U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_REF_CLK_STATUS_SHIFT         (0x0000000BU)
#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_REF_CLK_STATUS_RESETVAL      (0x00000001U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_REF_CLK_STATUS_MAX           (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_RCOSC_CLK_STATUS_MASK        (0x00001000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_RCOSC_CLK_STATUS_SHIFT       (0x0000000CU)
#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_RCOSC_CLK_STATUS_RESETVAL    (0x00000001U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_RCOSC_CLK_STATUS_MAX         (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_XTAL_DET_STATUS_MASK         (0x00002000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_XTAL_DET_STATUS_SHIFT        (0x0000000DU)
#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_XTAL_DET_STATUS_RESETVAL     (0x00000001U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_XTAL_DET_STATUS_MAX          (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_LIMP_MODE_STATUS_MASK        (0x00004000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_LIMP_MODE_STATUS_SHIFT       (0x0000000EU)
#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_LIMP_MODE_STATUS_RESETVAL    (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_LIMP_MODE_STATUS_MAX         (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_HVMODE_MASK                  (0x00008000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_HVMODE_SHIFT                 (0x0000000FU)
#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_HVMODE_RESETVAL              (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_HVMODE_MAX                   (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_APLLVCO18_UVDET_LAT_MASK     (0x00010000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_APLLVCO18_UVDET_LAT_SHIFT    (0x00000010U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_APLLVCO18_UVDET_LAT_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_APLLVCO18_UVDET_LAT_MAX      (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_VDDA_OSC_UVDET_LAT_MASK      (0x00020000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_VDDA_OSC_UVDET_LAT_SHIFT     (0x00000011U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_VDDA_OSC_UVDET_LAT_RESETVAL  (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_VDDA_OSC_UVDET_LAT_MAX       (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_VDDS_3P3V_UVDET_LAT_MASK     (0x00040000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_VDDS_3P3V_UVDET_LAT_SHIFT    (0x00000012U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_VDDS_3P3V_UVDET_LAT_RESETVAL (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_VDDS_3P3V_UVDET_LAT_MAX      (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_RESERVED0_MASK               (0xFFF80000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_RESERVED0_SHIFT              (0x00000013U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_RESERVED0_RESETVAL           (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_RESERVED0_MAX                (0x00001FFFU)

#define CSL_MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_RESETVAL                     (0x00003D5CU)

/* ANA_REG_WU_SPARE_OUT_LOWV */

#define CSL_MSS_TOPRCM_ANA_REG_WU_SPARE_OUT_LOWV_VDDCLK18DET_MASK              (0x00000001U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_SPARE_OUT_LOWV_VDDCLK18DET_SHIFT             (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_SPARE_OUT_LOWV_VDDCLK18DET_RESETVAL          (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_SPARE_OUT_LOWV_VDDCLK18DET_MAX               (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_WU_SPARE_OUT_LOWV_VDDARF_DET_MASK               (0x00000002U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_SPARE_OUT_LOWV_VDDARF_DET_SHIFT              (0x00000001U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_SPARE_OUT_LOWV_VDDARF_DET_RESETVAL           (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_SPARE_OUT_LOWV_VDDARF_DET_MAX                (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_WU_SPARE_OUT_LOWV_VDDS18DET_MASK                (0x00000004U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_SPARE_OUT_LOWV_VDDS18DET_SHIFT               (0x00000002U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_SPARE_OUT_LOWV_VDDS18DET_RESETVAL            (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_SPARE_OUT_LOWV_VDDS18DET_MAX                 (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_WU_SPARE_OUT_LOWV_HVMODE_MASK                   (0x00000008U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_SPARE_OUT_LOWV_HVMODE_SHIFT                  (0x00000003U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_SPARE_OUT_LOWV_HVMODE_RESETVAL               (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_SPARE_OUT_LOWV_HVMODE_MAX                    (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_WU_SPARE_OUT_LOWV_SUPPDET_OV_CTRL_MASK          (0x00000010U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_SPARE_OUT_LOWV_SUPPDET_OV_CTRL_SHIFT         (0x00000004U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_SPARE_OUT_LOWV_SUPPDET_OV_CTRL_RESETVAL      (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_SPARE_OUT_LOWV_SUPPDET_OV_CTRL_MAX           (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_WU_SPARE_OUT_LOWV_INT_OSC_CTRL_MASK             (0x00000020U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_SPARE_OUT_LOWV_INT_OSC_CTRL_SHIFT            (0x00000005U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_SPARE_OUT_LOWV_INT_OSC_CTRL_RESETVAL         (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_SPARE_OUT_LOWV_INT_OSC_CTRL_MAX              (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_WU_SPARE_OUT_LOWV_CORE_OVDET_LOWV_MASK          (0x00000040U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_SPARE_OUT_LOWV_CORE_OVDET_LOWV_SHIFT         (0x00000006U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_SPARE_OUT_LOWV_CORE_OVDET_LOWV_RESETVAL      (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_SPARE_OUT_LOWV_CORE_OVDET_LOWV_MAX           (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_WU_SPARE_OUT_LOWV_CORE_UVDET_LOWV_MASK          (0x00000080U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_SPARE_OUT_LOWV_CORE_UVDET_LOWV_SHIFT         (0x00000007U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_SPARE_OUT_LOWV_CORE_UVDET_LOWV_RESETVAL      (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_SPARE_OUT_LOWV_CORE_UVDET_LOWV_MAX           (0x00000001U)

#define CSL_MSS_TOPRCM_ANA_REG_WU_SPARE_OUT_LOWV_RESERVED0_MASK                (0xFFFFFF00U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_SPARE_OUT_LOWV_RESERVED0_SHIFT               (0x00000008U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_SPARE_OUT_LOWV_RESERVED0_RESETVAL            (0x00000000U)
#define CSL_MSS_TOPRCM_ANA_REG_WU_SPARE_OUT_LOWV_RESERVED0_MAX                 (0x00FFFFFFU)

#define CSL_MSS_TOPRCM_ANA_REG_WU_SPARE_OUT_LOWV_RESETVAL                      (0x00000000U)

/* HW_SPARE_RW0 */

#define CSL_MSS_TOPRCM_HW_SPARE_RW0_HW_SPARE_RW0_HW_SPARE_RW0_MASK             (0xFFFFFFFFU)
#define CSL_MSS_TOPRCM_HW_SPARE_RW0_HW_SPARE_RW0_HW_SPARE_RW0_SHIFT            (0x00000000U)
#define CSL_MSS_TOPRCM_HW_SPARE_RW0_HW_SPARE_RW0_HW_SPARE_RW0_RESETVAL         (0x00000000U)
#define CSL_MSS_TOPRCM_HW_SPARE_RW0_HW_SPARE_RW0_HW_SPARE_RW0_MAX              (0xFFFFFFFFU)

#define CSL_MSS_TOPRCM_HW_SPARE_RW0_RESETVAL                                   (0x00000000U)

/* HW_SPARE_RW1 */

#define CSL_MSS_TOPRCM_HW_SPARE_RW1_HW_SPARE_RW1_HW_SPARE_RW1_MASK             (0xFFFFFFFFU)
#define CSL_MSS_TOPRCM_HW_SPARE_RW1_HW_SPARE_RW1_HW_SPARE_RW1_SHIFT            (0x00000000U)
#define CSL_MSS_TOPRCM_HW_SPARE_RW1_HW_SPARE_RW1_HW_SPARE_RW1_RESETVAL         (0x00000000U)
#define CSL_MSS_TOPRCM_HW_SPARE_RW1_HW_SPARE_RW1_HW_SPARE_RW1_MAX              (0xFFFFFFFFU)

#define CSL_MSS_TOPRCM_HW_SPARE_RW1_RESETVAL                                   (0x00000000U)

/* HW_SPARE_RW2 */

#define CSL_MSS_TOPRCM_HW_SPARE_RW2_HW_SPARE_RW2_HW_SPARE_RW2_MASK             (0xFFFFFFFFU)
#define CSL_MSS_TOPRCM_HW_SPARE_RW2_HW_SPARE_RW2_HW_SPARE_RW2_SHIFT            (0x00000000U)
#define CSL_MSS_TOPRCM_HW_SPARE_RW2_HW_SPARE_RW2_HW_SPARE_RW2_RESETVAL         (0x00000000U)
#define CSL_MSS_TOPRCM_HW_SPARE_RW2_HW_SPARE_RW2_HW_SPARE_RW2_MAX              (0xFFFFFFFFU)

#define CSL_MSS_TOPRCM_HW_SPARE_RW2_RESETVAL                                   (0x00000000U)

/* HW_SPARE_RW3 */

#define CSL_MSS_TOPRCM_HW_SPARE_RW3_HW_SPARE_RW3_HW_SPARE_RW3_MASK             (0xFFFFFFFFU)
#define CSL_MSS_TOPRCM_HW_SPARE_RW3_HW_SPARE_RW3_HW_SPARE_RW3_SHIFT            (0x00000000U)
#define CSL_MSS_TOPRCM_HW_SPARE_RW3_HW_SPARE_RW3_HW_SPARE_RW3_RESETVAL         (0x00000000U)
#define CSL_MSS_TOPRCM_HW_SPARE_RW3_HW_SPARE_RW3_HW_SPARE_RW3_MAX              (0xFFFFFFFFU)

#define CSL_MSS_TOPRCM_HW_SPARE_RW3_RESETVAL                                   (0x00000000U)

/* HW_SPARE_RO0 */

#define CSL_MSS_TOPRCM_HW_SPARE_RO0_HW_SPARE_RO0_HW_SPARE_RO0_MASK             (0xFFFFFFFFU)
#define CSL_MSS_TOPRCM_HW_SPARE_RO0_HW_SPARE_RO0_HW_SPARE_RO0_SHIFT            (0x00000000U)
#define CSL_MSS_TOPRCM_HW_SPARE_RO0_HW_SPARE_RO0_HW_SPARE_RO0_RESETVAL         (0x00000000U)
#define CSL_MSS_TOPRCM_HW_SPARE_RO0_HW_SPARE_RO0_HW_SPARE_RO0_MAX              (0xFFFFFFFFU)

#define CSL_MSS_TOPRCM_HW_SPARE_RO0_RESETVAL                                   (0x00000000U)

/* HW_SPARE_RO1 */

#define CSL_MSS_TOPRCM_HW_SPARE_RO1_HW_SPARE_RO1_HW_SPARE_RO1_MASK             (0xFFFFFFFFU)
#define CSL_MSS_TOPRCM_HW_SPARE_RO1_HW_SPARE_RO1_HW_SPARE_RO1_SHIFT            (0x00000000U)
#define CSL_MSS_TOPRCM_HW_SPARE_RO1_HW_SPARE_RO1_HW_SPARE_RO1_RESETVAL         (0x00000000U)
#define CSL_MSS_TOPRCM_HW_SPARE_RO1_HW_SPARE_RO1_HW_SPARE_RO1_MAX              (0xFFFFFFFFU)

#define CSL_MSS_TOPRCM_HW_SPARE_RO1_RESETVAL                                   (0x00000000U)

/* HW_SPARE_RO2 */

#define CSL_MSS_TOPRCM_HW_SPARE_RO2_HW_SPARE_RO2_HW_SPARE_RO2_MASK             (0xFFFFFFFFU)
#define CSL_MSS_TOPRCM_HW_SPARE_RO2_HW_SPARE_RO2_HW_SPARE_RO2_SHIFT            (0x00000000U)
#define CSL_MSS_TOPRCM_HW_SPARE_RO2_HW_SPARE_RO2_HW_SPARE_RO2_RESETVAL         (0x00000000U)
#define CSL_MSS_TOPRCM_HW_SPARE_RO2_HW_SPARE_RO2_HW_SPARE_RO2_MAX              (0xFFFFFFFFU)

#define CSL_MSS_TOPRCM_HW_SPARE_RO2_RESETVAL                                   (0x00000000U)

/* HW_SPARE_RO3 */

#define CSL_MSS_TOPRCM_HW_SPARE_RO3_HW_SPARE_RO3_HW_SPARE_RO3_MASK             (0xFFFFFFFFU)
#define CSL_MSS_TOPRCM_HW_SPARE_RO3_HW_SPARE_RO3_HW_SPARE_RO3_SHIFT            (0x00000000U)
#define CSL_MSS_TOPRCM_HW_SPARE_RO3_HW_SPARE_RO3_HW_SPARE_RO3_RESETVAL         (0x00000000U)
#define CSL_MSS_TOPRCM_HW_SPARE_RO3_HW_SPARE_RO3_HW_SPARE_RO3_MAX              (0xFFFFFFFFU)

#define CSL_MSS_TOPRCM_HW_SPARE_RO3_RESETVAL                                   (0x00000000U)

/* HW_SPARE_WPH */

#define CSL_MSS_TOPRCM_HW_SPARE_WPH_HW_SPARE_WPH_HW_SPARE_WPH_MASK             (0xFFFFFFFFU)
#define CSL_MSS_TOPRCM_HW_SPARE_WPH_HW_SPARE_WPH_HW_SPARE_WPH_SHIFT            (0x00000000U)
#define CSL_MSS_TOPRCM_HW_SPARE_WPH_HW_SPARE_WPH_HW_SPARE_WPH_RESETVAL         (0x00000000U)
#define CSL_MSS_TOPRCM_HW_SPARE_WPH_HW_SPARE_WPH_HW_SPARE_WPH_MAX              (0xFFFFFFFFU)

#define CSL_MSS_TOPRCM_HW_SPARE_WPH_RESETVAL                                   (0x00000000U)

/* HW_SPARE_REC */

#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC0_MASK            (0x00000001U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC0_SHIFT           (0x00000000U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC0_RESETVAL        (0x00000000U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC0_MAX             (0x00000001U)

#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC1_MASK            (0x00000002U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC1_SHIFT           (0x00000001U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC1_RESETVAL        (0x00000000U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC1_MAX             (0x00000001U)

#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC2_MASK            (0x00000004U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC2_SHIFT           (0x00000002U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC2_RESETVAL        (0x00000000U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC2_MAX             (0x00000001U)

#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC3_MASK            (0x00000008U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC3_SHIFT           (0x00000003U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC3_RESETVAL        (0x00000000U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC3_MAX             (0x00000001U)

#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC4_MASK            (0x00000010U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC4_SHIFT           (0x00000004U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC4_RESETVAL        (0x00000000U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC4_MAX             (0x00000001U)

#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC5_MASK            (0x00000020U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC5_SHIFT           (0x00000005U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC5_RESETVAL        (0x00000000U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC5_MAX             (0x00000001U)

#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC6_MASK            (0x00000040U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC6_SHIFT           (0x00000006U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC6_RESETVAL        (0x00000000U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC6_MAX             (0x00000001U)

#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC7_MASK            (0x00000080U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC7_SHIFT           (0x00000007U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC7_RESETVAL        (0x00000000U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC7_MAX             (0x00000001U)

#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC8_MASK            (0x00000100U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC8_SHIFT           (0x00000008U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC8_RESETVAL        (0x00000000U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC8_MAX             (0x00000001U)

#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC9_MASK            (0x00000200U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC9_SHIFT           (0x00000009U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC9_RESETVAL        (0x00000000U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC9_MAX             (0x00000001U)

#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC10_MASK           (0x00000400U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC10_SHIFT          (0x0000000AU)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC10_RESETVAL       (0x00000000U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC10_MAX            (0x00000001U)

#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC11_MASK           (0x00000800U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC11_SHIFT          (0x0000000BU)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC11_RESETVAL       (0x00000000U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC11_MAX            (0x00000001U)

#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC12_MASK           (0x00001000U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC12_SHIFT          (0x0000000CU)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC12_RESETVAL       (0x00000000U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC12_MAX            (0x00000001U)

#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC13_MASK           (0x00002000U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC13_SHIFT          (0x0000000DU)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC13_RESETVAL       (0x00000000U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC13_MAX            (0x00000001U)

#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC14_MASK           (0x00004000U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC14_SHIFT          (0x0000000EU)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC14_RESETVAL       (0x00000000U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC14_MAX            (0x00000001U)

#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC15_MASK           (0x00008000U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC15_SHIFT          (0x0000000FU)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC15_RESETVAL       (0x00000000U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC15_MAX            (0x00000001U)

#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC16_MASK           (0x00010000U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC16_SHIFT          (0x00000010U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC16_RESETVAL       (0x00000000U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC16_MAX            (0x00000001U)

#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC17_MASK           (0x00020000U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC17_SHIFT          (0x00000011U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC17_RESETVAL       (0x00000000U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC17_MAX            (0x00000001U)

#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC18_MASK           (0x00040000U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC18_SHIFT          (0x00000012U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC18_RESETVAL       (0x00000000U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC18_MAX            (0x00000001U)

#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC19_MASK           (0x00080000U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC19_SHIFT          (0x00000013U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC19_RESETVAL       (0x00000000U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC19_MAX            (0x00000001U)

#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC20_MASK           (0x00100000U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC20_SHIFT          (0x00000014U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC20_RESETVAL       (0x00000000U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC20_MAX            (0x00000001U)

#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC21_MASK           (0x00200000U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC21_SHIFT          (0x00000015U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC21_RESETVAL       (0x00000000U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC21_MAX            (0x00000001U)

#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC22_MASK           (0x00400000U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC22_SHIFT          (0x00000016U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC22_RESETVAL       (0x00000000U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC22_MAX            (0x00000001U)

#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC23_MASK           (0x00800000U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC23_SHIFT          (0x00000017U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC23_RESETVAL       (0x00000000U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC23_MAX            (0x00000001U)

#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC24_MASK           (0x01000000U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC24_SHIFT          (0x00000018U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC24_RESETVAL       (0x00000000U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC24_MAX            (0x00000001U)

#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC25_MASK           (0x02000000U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC25_SHIFT          (0x00000019U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC25_RESETVAL       (0x00000000U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC25_MAX            (0x00000001U)

#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC26_MASK           (0x04000000U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC26_SHIFT          (0x0000001AU)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC26_RESETVAL       (0x00000000U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC26_MAX            (0x00000001U)

#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC27_MASK           (0x08000000U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC27_SHIFT          (0x0000001BU)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC27_RESETVAL       (0x00000000U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC27_MAX            (0x00000001U)

#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC28_MASK           (0x10000000U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC28_SHIFT          (0x0000001CU)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC28_RESETVAL       (0x00000000U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC28_MAX            (0x00000001U)

#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC29_MASK           (0x20000000U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC29_SHIFT          (0x0000001DU)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC29_RESETVAL       (0x00000000U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC29_MAX            (0x00000001U)

#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC30_MASK           (0x40000000U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC30_SHIFT          (0x0000001EU)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC30_RESETVAL       (0x00000000U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC30_MAX            (0x00000001U)

#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC31_MASK           (0x80000000U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC31_SHIFT          (0x0000001FU)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC31_RESETVAL       (0x00000000U)
#define CSL_MSS_TOPRCM_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC31_MAX            (0x00000001U)

#define CSL_MSS_TOPRCM_HW_SPARE_REC_RESETVAL                                   (0x00000000U)

/* LOCK0_KICK0 */

#define CSL_MSS_TOPRCM_LOCK0_KICK0_LOCK0_KICK0_MASK                            (0xFFFFFFFFU)
#define CSL_MSS_TOPRCM_LOCK0_KICK0_LOCK0_KICK0_SHIFT                           (0x00000000U)
#define CSL_MSS_TOPRCM_LOCK0_KICK0_LOCK0_KICK0_RESETVAL                        (0x00000000U)
#define CSL_MSS_TOPRCM_LOCK0_KICK0_LOCK0_KICK0_MAX                             (0xFFFFFFFFU)

#define CSL_MSS_TOPRCM_LOCK0_KICK0_RESETVAL                                    (0x00000000U)

/* LOCK0_KICK1 */

#define CSL_MSS_TOPRCM_LOCK0_KICK1_LOCK0_KICK1_MASK                            (0xFFFFFFFFU)
#define CSL_MSS_TOPRCM_LOCK0_KICK1_LOCK0_KICK1_SHIFT                           (0x00000000U)
#define CSL_MSS_TOPRCM_LOCK0_KICK1_LOCK0_KICK1_RESETVAL                        (0x00000000U)
#define CSL_MSS_TOPRCM_LOCK0_KICK1_LOCK0_KICK1_MAX                             (0xFFFFFFFFU)

#define CSL_MSS_TOPRCM_LOCK0_KICK1_RESETVAL                                    (0x00000000U)

/* INTR_RAW_STATUS */

#define CSL_MSS_TOPRCM_INTR_RAW_STATUS_PROT_ERR_MASK                           (0x00000001U)
#define CSL_MSS_TOPRCM_INTR_RAW_STATUS_PROT_ERR_SHIFT                          (0x00000000U)
#define CSL_MSS_TOPRCM_INTR_RAW_STATUS_PROT_ERR_RESETVAL                       (0x00000000U)
#define CSL_MSS_TOPRCM_INTR_RAW_STATUS_PROT_ERR_MAX                            (0x00000001U)

#define CSL_MSS_TOPRCM_INTR_RAW_STATUS_ADDR_ERR_MASK                           (0x00000002U)
#define CSL_MSS_TOPRCM_INTR_RAW_STATUS_ADDR_ERR_SHIFT                          (0x00000001U)
#define CSL_MSS_TOPRCM_INTR_RAW_STATUS_ADDR_ERR_RESETVAL                       (0x00000000U)
#define CSL_MSS_TOPRCM_INTR_RAW_STATUS_ADDR_ERR_MAX                            (0x00000001U)

#define CSL_MSS_TOPRCM_INTR_RAW_STATUS_KICK_ERR_MASK                           (0x00000004U)
#define CSL_MSS_TOPRCM_INTR_RAW_STATUS_KICK_ERR_SHIFT                          (0x00000002U)
#define CSL_MSS_TOPRCM_INTR_RAW_STATUS_KICK_ERR_RESETVAL                       (0x00000000U)
#define CSL_MSS_TOPRCM_INTR_RAW_STATUS_KICK_ERR_MAX                            (0x00000001U)

#define CSL_MSS_TOPRCM_INTR_RAW_STATUS_PROXY_ERR_MASK                          (0x00000008U)
#define CSL_MSS_TOPRCM_INTR_RAW_STATUS_PROXY_ERR_SHIFT                         (0x00000003U)
#define CSL_MSS_TOPRCM_INTR_RAW_STATUS_PROXY_ERR_RESETVAL                      (0x00000000U)
#define CSL_MSS_TOPRCM_INTR_RAW_STATUS_PROXY_ERR_MAX                           (0x00000001U)

#define CSL_MSS_TOPRCM_INTR_RAW_STATUS_RESETVAL                                (0x00000000U)

/* INTR_ENABLED_STATUS_CLEAR */

#define CSL_MSS_TOPRCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR_MASK         (0x00000001U)
#define CSL_MSS_TOPRCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR_SHIFT        (0x00000000U)
#define CSL_MSS_TOPRCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR_RESETVAL     (0x00000000U)
#define CSL_MSS_TOPRCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR_MAX          (0x00000001U)

#define CSL_MSS_TOPRCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR_MASK         (0x00000002U)
#define CSL_MSS_TOPRCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR_SHIFT        (0x00000001U)
#define CSL_MSS_TOPRCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR_RESETVAL     (0x00000000U)
#define CSL_MSS_TOPRCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR_MAX          (0x00000001U)

#define CSL_MSS_TOPRCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_KICK_ERR_MASK         (0x00000004U)
#define CSL_MSS_TOPRCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_KICK_ERR_SHIFT        (0x00000002U)
#define CSL_MSS_TOPRCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_KICK_ERR_RESETVAL     (0x00000000U)
#define CSL_MSS_TOPRCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_KICK_ERR_MAX          (0x00000001U)

#define CSL_MSS_TOPRCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROXY_ERR_MASK        (0x00000008U)
#define CSL_MSS_TOPRCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROXY_ERR_SHIFT       (0x00000003U)
#define CSL_MSS_TOPRCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROXY_ERR_RESETVAL    (0x00000000U)
#define CSL_MSS_TOPRCM_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROXY_ERR_MAX         (0x00000001U)

#define CSL_MSS_TOPRCM_INTR_ENABLED_STATUS_CLEAR_RESETVAL                      (0x00000000U)

/* INTR_ENABLE */

#define CSL_MSS_TOPRCM_INTR_ENABLE_PROT_ERR_EN_MASK                            (0x00000001U)
#define CSL_MSS_TOPRCM_INTR_ENABLE_PROT_ERR_EN_SHIFT                           (0x00000000U)
#define CSL_MSS_TOPRCM_INTR_ENABLE_PROT_ERR_EN_RESETVAL                        (0x00000000U)
#define CSL_MSS_TOPRCM_INTR_ENABLE_PROT_ERR_EN_MAX                             (0x00000001U)

#define CSL_MSS_TOPRCM_INTR_ENABLE_ADDR_ERR_EN_MASK                            (0x00000002U)
#define CSL_MSS_TOPRCM_INTR_ENABLE_ADDR_ERR_EN_SHIFT                           (0x00000001U)
#define CSL_MSS_TOPRCM_INTR_ENABLE_ADDR_ERR_EN_RESETVAL                        (0x00000000U)
#define CSL_MSS_TOPRCM_INTR_ENABLE_ADDR_ERR_EN_MAX                             (0x00000001U)

#define CSL_MSS_TOPRCM_INTR_ENABLE_KICK_ERR_EN_MASK                            (0x00000004U)
#define CSL_MSS_TOPRCM_INTR_ENABLE_KICK_ERR_EN_SHIFT                           (0x00000002U)
#define CSL_MSS_TOPRCM_INTR_ENABLE_KICK_ERR_EN_RESETVAL                        (0x00000000U)
#define CSL_MSS_TOPRCM_INTR_ENABLE_KICK_ERR_EN_MAX                             (0x00000001U)

#define CSL_MSS_TOPRCM_INTR_ENABLE_PROXY_ERR_EN_MASK                           (0x00000008U)
#define CSL_MSS_TOPRCM_INTR_ENABLE_PROXY_ERR_EN_SHIFT                          (0x00000003U)
#define CSL_MSS_TOPRCM_INTR_ENABLE_PROXY_ERR_EN_RESETVAL                       (0x00000000U)
#define CSL_MSS_TOPRCM_INTR_ENABLE_PROXY_ERR_EN_MAX                            (0x00000001U)

#define CSL_MSS_TOPRCM_INTR_ENABLE_RESETVAL                                    (0x00000000U)

/* INTR_ENABLE_CLEAR */

#define CSL_MSS_TOPRCM_INTR_ENABLE_CLEAR_PROT_ERR_EN_CLR_MASK                  (0x00000001U)
#define CSL_MSS_TOPRCM_INTR_ENABLE_CLEAR_PROT_ERR_EN_CLR_SHIFT                 (0x00000000U)
#define CSL_MSS_TOPRCM_INTR_ENABLE_CLEAR_PROT_ERR_EN_CLR_RESETVAL              (0x00000000U)
#define CSL_MSS_TOPRCM_INTR_ENABLE_CLEAR_PROT_ERR_EN_CLR_MAX                   (0x00000001U)

#define CSL_MSS_TOPRCM_INTR_ENABLE_CLEAR_ADDR_ERR_EN_CLR_MASK                  (0x00000002U)
#define CSL_MSS_TOPRCM_INTR_ENABLE_CLEAR_ADDR_ERR_EN_CLR_SHIFT                 (0x00000001U)
#define CSL_MSS_TOPRCM_INTR_ENABLE_CLEAR_ADDR_ERR_EN_CLR_RESETVAL              (0x00000000U)
#define CSL_MSS_TOPRCM_INTR_ENABLE_CLEAR_ADDR_ERR_EN_CLR_MAX                   (0x00000001U)

#define CSL_MSS_TOPRCM_INTR_ENABLE_CLEAR_KICK_ERR_EN_CLR_MASK                  (0x00000004U)
#define CSL_MSS_TOPRCM_INTR_ENABLE_CLEAR_KICK_ERR_EN_CLR_SHIFT                 (0x00000002U)
#define CSL_MSS_TOPRCM_INTR_ENABLE_CLEAR_KICK_ERR_EN_CLR_RESETVAL              (0x00000000U)
#define CSL_MSS_TOPRCM_INTR_ENABLE_CLEAR_KICK_ERR_EN_CLR_MAX                   (0x00000001U)

#define CSL_MSS_TOPRCM_INTR_ENABLE_CLEAR_PROXY_ERR_EN_CLR_MASK                 (0x00000008U)
#define CSL_MSS_TOPRCM_INTR_ENABLE_CLEAR_PROXY_ERR_EN_CLR_SHIFT                (0x00000003U)
#define CSL_MSS_TOPRCM_INTR_ENABLE_CLEAR_PROXY_ERR_EN_CLR_RESETVAL             (0x00000000U)
#define CSL_MSS_TOPRCM_INTR_ENABLE_CLEAR_PROXY_ERR_EN_CLR_MAX                  (0x00000001U)

#define CSL_MSS_TOPRCM_INTR_ENABLE_CLEAR_RESETVAL                              (0x00000000U)

/* EOI */

#define CSL_MSS_TOPRCM_EOI_EOI_VECTOR_MASK                                     (0x000000FFU)
#define CSL_MSS_TOPRCM_EOI_EOI_VECTOR_SHIFT                                    (0x00000000U)
#define CSL_MSS_TOPRCM_EOI_EOI_VECTOR_RESETVAL                                 (0x00000000U)
#define CSL_MSS_TOPRCM_EOI_EOI_VECTOR_MAX                                      (0x000000FFU)

#define CSL_MSS_TOPRCM_EOI_RESETVAL                                            (0x00000000U)

/* FAULT_ADDRESS */

#define CSL_MSS_TOPRCM_FAULT_ADDRESS_FAULT_ADDR_MASK                           (0xFFFFFFFFU)
#define CSL_MSS_TOPRCM_FAULT_ADDRESS_FAULT_ADDR_SHIFT                          (0x00000000U)
#define CSL_MSS_TOPRCM_FAULT_ADDRESS_FAULT_ADDR_RESETVAL                       (0x00000000U)
#define CSL_MSS_TOPRCM_FAULT_ADDRESS_FAULT_ADDR_MAX                            (0xFFFFFFFFU)

#define CSL_MSS_TOPRCM_FAULT_ADDRESS_RESETVAL                                  (0x00000000U)

/* FAULT_TYPE_STATUS */

#define CSL_MSS_TOPRCM_FAULT_TYPE_STATUS_FAULT_TYPE_MASK                       (0x0000003FU)
#define CSL_MSS_TOPRCM_FAULT_TYPE_STATUS_FAULT_TYPE_SHIFT                      (0x00000000U)
#define CSL_MSS_TOPRCM_FAULT_TYPE_STATUS_FAULT_TYPE_RESETVAL                   (0x00000000U)
#define CSL_MSS_TOPRCM_FAULT_TYPE_STATUS_FAULT_TYPE_MAX                        (0x0000003FU)

#define CSL_MSS_TOPRCM_FAULT_TYPE_STATUS_FAULT_NS_MASK                         (0x00000040U)
#define CSL_MSS_TOPRCM_FAULT_TYPE_STATUS_FAULT_NS_SHIFT                        (0x00000006U)
#define CSL_MSS_TOPRCM_FAULT_TYPE_STATUS_FAULT_NS_RESETVAL                     (0x00000000U)
#define CSL_MSS_TOPRCM_FAULT_TYPE_STATUS_FAULT_NS_MAX                          (0x00000001U)

#define CSL_MSS_TOPRCM_FAULT_TYPE_STATUS_RESETVAL                              (0x00000000U)

/* FAULT_ATTR_STATUS */

#define CSL_MSS_TOPRCM_FAULT_ATTR_STATUS_FAULT_PRIVID_MASK                     (0x000000FFU)
#define CSL_MSS_TOPRCM_FAULT_ATTR_STATUS_FAULT_PRIVID_SHIFT                    (0x00000000U)
#define CSL_MSS_TOPRCM_FAULT_ATTR_STATUS_FAULT_PRIVID_RESETVAL                 (0x00000000U)
#define CSL_MSS_TOPRCM_FAULT_ATTR_STATUS_FAULT_PRIVID_MAX                      (0x000000FFU)

#define CSL_MSS_TOPRCM_FAULT_ATTR_STATUS_FAULT_ROUTEID_MASK                    (0x000FFF00U)
#define CSL_MSS_TOPRCM_FAULT_ATTR_STATUS_FAULT_ROUTEID_SHIFT                   (0x00000008U)
#define CSL_MSS_TOPRCM_FAULT_ATTR_STATUS_FAULT_ROUTEID_RESETVAL                (0x00000000U)
#define CSL_MSS_TOPRCM_FAULT_ATTR_STATUS_FAULT_ROUTEID_MAX                     (0x00000FFFU)

#define CSL_MSS_TOPRCM_FAULT_ATTR_STATUS_FAULT_XID_MASK                        (0xFFF00000U)
#define CSL_MSS_TOPRCM_FAULT_ATTR_STATUS_FAULT_XID_SHIFT                       (0x00000014U)
#define CSL_MSS_TOPRCM_FAULT_ATTR_STATUS_FAULT_XID_RESETVAL                    (0x00000000U)
#define CSL_MSS_TOPRCM_FAULT_ATTR_STATUS_FAULT_XID_MAX                         (0x00000FFFU)

#define CSL_MSS_TOPRCM_FAULT_ATTR_STATUS_RESETVAL                              (0x00000000U)

/* FAULT_CLEAR */

#define CSL_MSS_TOPRCM_FAULT_CLEAR_FAULT_CLR_MASK                              (0x00000001U)
#define CSL_MSS_TOPRCM_FAULT_CLEAR_FAULT_CLR_SHIFT                             (0x00000000U)
#define CSL_MSS_TOPRCM_FAULT_CLEAR_FAULT_CLR_RESETVAL                          (0x00000000U)
#define CSL_MSS_TOPRCM_FAULT_CLEAR_FAULT_CLR_MAX                               (0x00000001U)

#define CSL_MSS_TOPRCM_FAULT_CLEAR_RESETVAL                                    (0x00000000U)

#ifdef __cplusplus
}
#endif
#endif
