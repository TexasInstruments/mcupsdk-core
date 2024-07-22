/********************************************************************
 * Copyright (C) 2024 Texas Instruments Incorporated.
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
 *  Name        : cslr_iomux.h
*/
#ifndef CSLR_IOMUX_H_
#define CSLR_IOMUX_H_

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
    volatile uint32_t QSPI0_CSN0_CFG_REG;
    volatile uint32_t QSPI0_CSN1_CFG_REG;
    volatile uint32_t QSPI0_CLK_CFG_REG;
    volatile uint32_t QSPI0_D0_CFG_REG;
    volatile uint32_t QSPI0_D1_CFG_REG;
    volatile uint32_t QSPI0_D2_CFG_REG;
    volatile uint32_t QSPI0_D3_CFG_REG;
    volatile uint32_t MCAN0_RX_CFG_REG;
    volatile uint32_t MCAN0_TX_CFG_REG;
    volatile uint32_t MCAN1_RX_CFG_REG;
    volatile uint32_t MCAN1_TX_CFG_REG;
    volatile uint32_t SPI0_CS0_CFG_REG;
    volatile uint32_t SPI0_CLK_CFG_REG;
    volatile uint32_t SPI0_D0_CFG_REG;
    volatile uint32_t SPI0_D1_CFG_REG;
    volatile uint32_t SPI1_CS0_CFG_REG;
    volatile uint32_t SPI1_CLK_CFG_REG;
    volatile uint32_t SPI1_D0_CFG_REG;
    volatile uint32_t SPI1_D1_CFG_REG;
    volatile uint32_t LIN1_RXD_CFG_REG;
    volatile uint32_t LIN1_TXD_CFG_REG;
    volatile uint32_t LIN2_RXD_CFG_REG;
    volatile uint32_t LIN2_TXD_CFG_REG;
    volatile uint32_t I2C1_SCL_CFG_REG;
    volatile uint32_t I2C1_SDA_CFG_REG;
    volatile uint32_t UART0_RTSN_CFG_REG;
    volatile uint32_t UART0_CTSN_CFG_REG;
    volatile uint32_t UART0_RXD_CFG_REG;
    volatile uint32_t UART0_TXD_CFG_REG;
    volatile uint32_t RGMII1_RXC_CFG_REG;
    volatile uint32_t RGMII1_RX_CTL_CFG_REG;
    volatile uint32_t RGMII1_RD0_CFG_REG;
    volatile uint32_t RGMII1_RD1_CFG_REG;
    volatile uint32_t RGMII1_RD2_CFG_REG;
    volatile uint32_t RGMII1_RD3_CFG_REG;
    volatile uint32_t RGMII1_TXC_CFG_REG;
    volatile uint32_t RGMII1_TX_CTL_CFG_REG;
    volatile uint32_t RGMII1_TD0_CFG_REG;
    volatile uint32_t RGMII1_TD1_CFG_REG;
    volatile uint32_t RGMII1_TD2_CFG_REG;
    volatile uint32_t RGMII1_TD3_CFG_REG;
    volatile uint32_t MDIO0_MDIO_CFG_REG;
    volatile uint32_t MDIO0_MDC_CFG_REG;
    volatile uint32_t EPWM0_A_CFG_REG;
    volatile uint32_t EPWM0_B_CFG_REG;
    volatile uint32_t EPWM1_A_CFG_REG;
    volatile uint32_t EPWM1_B_CFG_REG;
    volatile uint32_t EPWM2_A_CFG_REG;
    volatile uint32_t EPWM2_B_CFG_REG;
    volatile uint32_t EPWM3_A_CFG_REG;
    volatile uint32_t EPWM3_B_CFG_REG;
    volatile uint32_t EPWM4_A_CFG_REG;
    volatile uint32_t EPWM4_B_CFG_REG;
    volatile uint32_t EPWM5_A_CFG_REG;
    volatile uint32_t EPWM5_B_CFG_REG;
    volatile uint32_t EPWM6_A_CFG_REG;
    volatile uint32_t EPWM6_B_CFG_REG;
    volatile uint32_t EPWM7_A_CFG_REG;
    volatile uint32_t EPWM7_B_CFG_REG;
    volatile uint32_t EPWM8_A_CFG_REG;
    volatile uint32_t EPWM8_B_CFG_REG;
    volatile uint32_t EPWM9_A_CFG_REG;
    volatile uint32_t EPWM9_B_CFG_REG;
	volatile uint32_t GPIO63_CFG_REG;		
	volatile uint32_t GPIO64_CFG_REG;		
	volatile uint32_t GPIO65_CFG_REG;		
	volatile uint32_t GPIO66_CFG_REG;		
	volatile uint32_t PR1_PRU0_GPO0_CFG_REG;		
	volatile uint32_t PR1_PRU0_GPO1_CFG_REG;		
	volatile uint32_t PR1_PRU0_GPO2_CFG_REG;		
	volatile uint32_t PR1_PRU0_GPO9_CFG_REG;		
	volatile uint32_t PR1_PRU1_GPO0_CFG_REG;		
	volatile uint32_t PR1_PRU1_GPO1_CFG_REG;		
	volatile uint32_t PR1_PRU1_GPO2_CFG_REG;		
	volatile uint32_t PR1_PRU1_GPO9_CFG_REG;		
    volatile uint32_t UART1_RXD_CFG_REG;
    volatile uint32_t UART1_TXD_CFG_REG;
    volatile uint32_t MMC0_CLK_CFG_REG;
    volatile uint32_t MMC0_CMD_CFG_REG;
    volatile uint32_t MMC0_D0_CFG_REG;
    volatile uint32_t MMC0_D1_CFG_REG;
    volatile uint32_t MMC0_D2_CFG_REG;
    volatile uint32_t MMC0_D3_CFG_REG;
    volatile uint32_t MMC0_WP_CFG_REG;
    volatile uint32_t MMC0_CD_CFG_REG;
    volatile uint32_t PR0_MDIO0_MDIO_CFG_REG;
    volatile uint32_t PR0_MDIO0_MDC_CFG_REG;
    volatile uint32_t PR0_PRU0_GPO5_CFG_REG;
    volatile uint32_t PR0_PRU0_GPO9_CFG_REG;
    volatile uint32_t PR0_PRU0_GPO10_CFG_REG;
    volatile uint32_t PR0_PRU0_GPO8_CFG_REG;
    volatile uint32_t PR0_PRU0_GPO6_CFG_REG;
    volatile uint32_t PR0_PRU0_GPO4_CFG_REG;
    volatile uint32_t PR0_PRU0_GPO0_CFG_REG;
    volatile uint32_t PR0_PRU0_GPO1_CFG_REG;
    volatile uint32_t PR0_PRU0_GPO2_CFG_REG;
    volatile uint32_t PR0_PRU0_GPO3_CFG_REG;
    volatile uint32_t PR0_PRU0_GPO16_CFG_REG;
    volatile uint32_t PR0_PRU0_GPO15_CFG_REG;
    volatile uint32_t PR0_PRU0_GPO11_CFG_REG;
    volatile uint32_t PR0_PRU0_GPO12_CFG_REG;
    volatile uint32_t PR0_PRU0_GPO13_CFG_REG;
    volatile uint32_t PR0_PRU0_GPO14_CFG_REG;
    volatile uint32_t PR0_PRU1_GPO5_CFG_REG;
    volatile uint32_t PR0_PRU1_GPO9_CFG_REG;
    volatile uint32_t PR0_PRU1_GPO10_CFG_REG;
    volatile uint32_t PR0_PRU1_GPO8_CFG_REG;
    volatile uint32_t PR0_PRU1_GPO6_CFG_REG;
    volatile uint32_t PR0_PRU1_GPO4_CFG_REG;
    volatile uint32_t PR0_PRU1_GPO0_CFG_REG;
    volatile uint32_t PR0_PRU1_GPO1_CFG_REG;
    volatile uint32_t PR0_PRU1_GPO2_CFG_REG;
    volatile uint32_t PR0_PRU1_GPO3_CFG_REG;
    volatile uint32_t PR0_PRU1_GPO16_CFG_REG;
    volatile uint32_t PR0_PRU1_GPO15_CFG_REG;
    volatile uint32_t PR0_PRU1_GPO11_CFG_REG;
    volatile uint32_t PR0_PRU1_GPO12_CFG_REG;
    volatile uint32_t PR0_PRU1_GPO13_CFG_REG;
    volatile uint32_t PR0_PRU1_GPO14_CFG_REG;
    volatile uint32_t PR0_PRU1_GPO19_CFG_REG;
    volatile uint32_t PR0_PRU1_GPO18_CFG_REG;
    volatile uint32_t EXT_REFCLK0_CFG_REG;
    volatile uint32_t SDFM0_CLK0_CFG_REG;
    volatile uint32_t SDFM0_D0_CFG_REG;
    volatile uint32_t SDFM0_CLK1_CFG_REG;
    volatile uint32_t SDFM0_D1_CFG_REG;
    volatile uint32_t SDFM0_CLK2_CFG_REG;
    volatile uint32_t SDFM0_D2_CFG_REG;
    volatile uint32_t SDFM0_CLK3_CFG_REG;
    volatile uint32_t SDFM0_D3_CFG_REG;
    volatile uint32_t EQEP0_A_CFG_REG;
    volatile uint32_t EQEP0_B_CFG_REG;
    volatile uint32_t EQEP0_STROBE_CFG_REG;
    volatile uint32_t EQEP0_INDEX_CFG_REG;
    volatile uint32_t I2C0_SDA_CFG_REG;
    volatile uint32_t I2C0_SCL_CFG_REG;
	volatile uint32_t GPIO136_CFG_REG;		
	volatile uint32_t GPIO137_CFG_REG;		
    volatile uint32_t CLKOUT0_CFG_REG;
	volatile uint32_t USB0_DP_CFG_REG;		
	volatile uint32_t USB0_DM_CFG_REG;		
    volatile uint32_t WARMRSTN_CFG_REG;
    volatile uint32_t SAFETY_ERRORN_CFG_REG;
    volatile uint32_t TDI_CFG_REG;
    volatile uint32_t TDO_CFG_REG;
    volatile uint32_t TMS_CFG_REG;
    volatile uint32_t TCK_CFG_REG;
    volatile uint32_t OSPI0_CLKLB_CFG_REG;
	volatile uint32_t OSPI1_CLKLB_CFG_REG;		
    volatile uint32_t QUAL_GRP_0_CFG_REG;
    volatile uint32_t QUAL_GRP_1_CFG_REG;
    volatile uint32_t QUAL_GRP_2_CFG_REG;
    volatile uint32_t QUAL_GRP_3_CFG_REG;
    volatile uint32_t QUAL_GRP_4_CFG_REG;
    volatile uint32_t QUAL_GRP_5_CFG_REG;
    volatile uint32_t QUAL_GRP_6_CFG_REG;
    volatile uint32_t QUAL_GRP_7_CFG_REG;
    volatile uint32_t QUAL_GRP_8_CFG_REG;
    volatile uint32_t QUAL_GRP_9_CFG_REG;
    volatile uint32_t QUAL_GRP_10_CFG_REG;
    volatile uint32_t QUAL_GRP_11_CFG_REG;
    volatile uint32_t QUAL_GRP_12_CFG_REG;
    volatile uint32_t QUAL_GRP_13_CFG_REG;
    volatile uint32_t QUAL_GRP_14_CFG_REG;
    volatile uint32_t QUAL_GRP_15_CFG_REG;
    volatile uint32_t QUAL_GRP_16_CFG_REG;
    volatile uint32_t QUAL_GRP_17_CFG_REG;
    volatile uint32_t USER_MODE_EN;
    volatile uint32_t PADGLBL_CFG_REG;
    volatile uint32_t IO_CFG_KICK0;
    volatile uint32_t IO_CFG_KICK1;
} CSL_iomuxRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_IOMUX_QSPI0_CSN0_CFG_REG                                           (0x00000000U)
#define CSL_IOMUX_QSPI0_CSN1_CFG_REG                                           (0x00000004U)
#define CSL_IOMUX_QSPI0_CLK_CFG_REG                                            (0x00000008U)
#define CSL_IOMUX_QSPI0_D0_CFG_REG                                             (0x0000000CU)
#define CSL_IOMUX_QSPI0_D1_CFG_REG                                             (0x00000010U)
#define CSL_IOMUX_QSPI0_D2_CFG_REG                                             (0x00000014U)
#define CSL_IOMUX_QSPI0_D3_CFG_REG                                             (0x00000018U)
#define CSL_IOMUX_MCAN0_RX_CFG_REG                                             (0x0000001CU)
#define CSL_IOMUX_MCAN0_TX_CFG_REG                                             (0x00000020U)
#define CSL_IOMUX_MCAN1_RX_CFG_REG                                             (0x00000024U)
#define CSL_IOMUX_MCAN1_TX_CFG_REG                                             (0x00000028U)
#define CSL_IOMUX_SPI0_CS0_CFG_REG                                             (0x0000002CU)
#define CSL_IOMUX_SPI0_CLK_CFG_REG                                             (0x00000030U)
#define CSL_IOMUX_SPI0_D0_CFG_REG                                              (0x00000034U)
#define CSL_IOMUX_SPI0_D1_CFG_REG                                              (0x00000038U)
#define CSL_IOMUX_SPI1_CS0_CFG_REG                                             (0x0000003CU)
#define CSL_IOMUX_SPI1_CLK_CFG_REG                                             (0x00000040U)
#define CSL_IOMUX_SPI1_D0_CFG_REG                                              (0x00000044U)
#define CSL_IOMUX_SPI1_D1_CFG_REG                                              (0x00000048U)
#define CSL_IOMUX_LIN1_RXD_CFG_REG                                             (0x0000004CU)
#define CSL_IOMUX_LIN1_TXD_CFG_REG                                             (0x00000050U)
#define CSL_IOMUX_LIN2_RXD_CFG_REG                                             (0x00000054U)
#define CSL_IOMUX_LIN2_TXD_CFG_REG                                             (0x00000058U)
#define CSL_IOMUX_I2C1_SCL_CFG_REG                                             (0x0000005CU)
#define CSL_IOMUX_I2C1_SDA_CFG_REG                                             (0x00000060U)
#define CSL_IOMUX_UART0_RTSN_CFG_REG                                           (0x00000064U)
#define CSL_IOMUX_UART0_CTSN_CFG_REG                                           (0x00000068U)
#define CSL_IOMUX_UART0_RXD_CFG_REG                                            (0x0000006CU)
#define CSL_IOMUX_UART0_TXD_CFG_REG                                            (0x00000070U)
#define CSL_IOMUX_RGMII1_RXC_CFG_REG                                           (0x00000074U)
#define CSL_IOMUX_RGMII1_RX_CTL_CFG_REG                                        (0x00000078U)
#define CSL_IOMUX_RGMII1_RD0_CFG_REG                                           (0x0000007CU)
#define CSL_IOMUX_RGMII1_RD1_CFG_REG                                           (0x00000080U)
#define CSL_IOMUX_RGMII1_RD2_CFG_REG                                           (0x00000084U)
#define CSL_IOMUX_RGMII1_RD3_CFG_REG                                           (0x00000088U)
#define CSL_IOMUX_RGMII1_TXC_CFG_REG                                           (0x0000008CU)
#define CSL_IOMUX_RGMII1_TX_CTL_CFG_REG                                        (0x00000090U)
#define CSL_IOMUX_RGMII1_TD0_CFG_REG                                           (0x00000094U)
#define CSL_IOMUX_RGMII1_TD1_CFG_REG                                           (0x00000098U)
#define CSL_IOMUX_RGMII1_TD2_CFG_REG                                           (0x0000009CU)
#define CSL_IOMUX_RGMII1_TD3_CFG_REG                                           (0x000000A0U)
#define CSL_IOMUX_MDIO0_MDIO_CFG_REG                                           (0x000000A4U)
#define CSL_IOMUX_MDIO0_MDC_CFG_REG                                            (0x000000A8U)
#define CSL_IOMUX_EPWM0_A_CFG_REG                                              (0x000000ACU)
#define CSL_IOMUX_EPWM0_B_CFG_REG                                              (0x000000B0U)
#define CSL_IOMUX_EPWM1_A_CFG_REG                                              (0x000000B4U)
#define CSL_IOMUX_EPWM1_B_CFG_REG                                              (0x000000B8U)
#define CSL_IOMUX_EPWM2_A_CFG_REG                                              (0x000000BCU)
#define CSL_IOMUX_EPWM2_B_CFG_REG                                              (0x000000C0U)
#define CSL_IOMUX_EPWM3_A_CFG_REG                                              (0x000000C4U)
#define CSL_IOMUX_EPWM3_B_CFG_REG                                              (0x000000C8U)
#define CSL_IOMUX_EPWM4_A_CFG_REG                                              (0x000000CCU)
#define CSL_IOMUX_EPWM4_B_CFG_REG                                              (0x000000D0U)
#define CSL_IOMUX_EPWM5_A_CFG_REG                                              (0x000000D4U)
#define CSL_IOMUX_EPWM5_B_CFG_REG                                              (0x000000D8U)
#define CSL_IOMUX_EPWM6_A_CFG_REG                                              (0x000000DCU)
#define CSL_IOMUX_EPWM6_B_CFG_REG                                              (0x000000E0U)
#define CSL_IOMUX_EPWM7_A_CFG_REG                                              (0x000000E4U)
#define CSL_IOMUX_EPWM7_B_CFG_REG                                              (0x000000E8U)
#define CSL_IOMUX_EPWM8_A_CFG_REG                                              (0x000000ECU)
#define CSL_IOMUX_EPWM8_B_CFG_REG                                              (0x000000F0U)
#define CSL_IOMUX_EPWM9_A_CFG_REG                                              (0x000000F4U)
#define CSL_IOMUX_EPWM9_B_CFG_REG                                              (0x000000F8U)
#define CSL_IOMUX_GPIO63_CFG_REG                                                (0x000000FCU)
#define CSL_IOMUX_GPIO64_CFG_REG                                                (0x00000100U)
#define CSL_IOMUX_GPIO65_CFG_REG                                                (0x00000104U)
#define CSL_IOMUX_GPIO66_CFG_REG                                                (0x00000108U)
#define CSL_IOMUX_PR1_PRU0_GPO0_CFG_REG                                         (0x0000010CU)
#define CSL_IOMUX_PR1_PRU0_GPO1_CFG_REG                                         (0x00000110U)
#define CSL_IOMUX_PR1_PRU0_GPO2_CFG_REG                                         (0x00000114U)
#define CSL_IOMUX_PR1_PRU0_GPO9_CFG_REG                                         (0x00000118U)
#define CSL_IOMUX_PR1_PRU1_GPO0_CFG_REG                                         (0x0000011CU)
#define CSL_IOMUX_PR1_PRU1_GPO1_CFG_REG                                         (0x00000120U)
#define CSL_IOMUX_PR1_PRU1_GPO2_CFG_REG                                         (0x00000124U)
#define CSL_IOMUX_PR1_PRU1_GPO9_CFG_REG                                         (0x00000128U)
#define CSL_IOMUX_UART1_RXD_CFG_REG                                            (0x0000012CU)
#define CSL_IOMUX_UART1_TXD_CFG_REG                                            (0x00000130U)
#define CSL_IOMUX_MMC0_CLK_CFG_REG                                             (0x00000134U)
#define CSL_IOMUX_MMC0_CMD_CFG_REG                                             (0x00000138U)
#define CSL_IOMUX_MMC0_D0_CFG_REG                                              (0x0000013CU)
#define CSL_IOMUX_MMC0_D1_CFG_REG                                              (0x00000140U)
#define CSL_IOMUX_MMC0_D2_CFG_REG                                              (0x00000144U)
#define CSL_IOMUX_MMC0_D3_CFG_REG                                              (0x00000148U)
#define CSL_IOMUX_MMC0_WP_CFG_REG                                              (0x0000014CU)
#define CSL_IOMUX_MMC0_CD_CFG_REG                                              (0x00000150U)
#define CSL_IOMUX_PR0_MDIO0_MDIO_CFG_REG                                       (0x00000154U)
#define CSL_IOMUX_PR0_MDIO0_MDC_CFG_REG                                        (0x00000158U)
#define CSL_IOMUX_PR0_PRU0_GPO5_CFG_REG                                        (0x0000015CU)
#define CSL_IOMUX_PR0_PRU0_GPO9_CFG_REG                                        (0x00000160U)
#define CSL_IOMUX_PR0_PRU0_GPO10_CFG_REG                                       (0x00000164U)
#define CSL_IOMUX_PR0_PRU0_GPO8_CFG_REG                                        (0x00000168U)
#define CSL_IOMUX_PR0_PRU0_GPO6_CFG_REG                                        (0x0000016CU)
#define CSL_IOMUX_PR0_PRU0_GPO4_CFG_REG                                        (0x00000170U)
#define CSL_IOMUX_PR0_PRU0_GPO0_CFG_REG                                        (0x00000174U)
#define CSL_IOMUX_PR0_PRU0_GPO1_CFG_REG                                        (0x00000178U)
#define CSL_IOMUX_PR0_PRU0_GPO2_CFG_REG                                        (0x0000017CU)
#define CSL_IOMUX_PR0_PRU0_GPO3_CFG_REG                                        (0x00000180U)
#define CSL_IOMUX_PR0_PRU0_GPO16_CFG_REG                                       (0x00000184U)
#define CSL_IOMUX_PR0_PRU0_GPO15_CFG_REG                                       (0x00000188U)
#define CSL_IOMUX_PR0_PRU0_GPO11_CFG_REG                                       (0x0000018CU)
#define CSL_IOMUX_PR0_PRU0_GPO12_CFG_REG                                       (0x00000190U)
#define CSL_IOMUX_PR0_PRU0_GPO13_CFG_REG                                       (0x00000194U)
#define CSL_IOMUX_PR0_PRU0_GPO14_CFG_REG                                       (0x00000198U)
#define CSL_IOMUX_PR0_PRU1_GPO5_CFG_REG                                        (0x0000019CU)
#define CSL_IOMUX_PR0_PRU1_GPO9_CFG_REG                                        (0x000001A0U)
#define CSL_IOMUX_PR0_PRU1_GPO10_CFG_REG                                       (0x000001A4U)
#define CSL_IOMUX_PR0_PRU1_GPO8_CFG_REG                                        (0x000001A8U)
#define CSL_IOMUX_PR0_PRU1_GPO6_CFG_REG                                        (0x000001ACU)
#define CSL_IOMUX_PR0_PRU1_GPO4_CFG_REG                                        (0x000001B0U)
#define CSL_IOMUX_PR0_PRU1_GPO0_CFG_REG                                        (0x000001B4U)
#define CSL_IOMUX_PR0_PRU1_GPO1_CFG_REG                                        (0x000001B8U)
#define CSL_IOMUX_PR0_PRU1_GPO2_CFG_REG                                        (0x000001BCU)
#define CSL_IOMUX_PR0_PRU1_GPO3_CFG_REG                                        (0x000001C0U)
#define CSL_IOMUX_PR0_PRU1_GPO16_CFG_REG                                       (0x000001C4U)
#define CSL_IOMUX_PR0_PRU1_GPO15_CFG_REG                                       (0x000001C8U)
#define CSL_IOMUX_PR0_PRU1_GPO11_CFG_REG                                       (0x000001CCU)
#define CSL_IOMUX_PR0_PRU1_GPO12_CFG_REG                                       (0x000001D0U)
#define CSL_IOMUX_PR0_PRU1_GPO13_CFG_REG                                       (0x000001D4U)
#define CSL_IOMUX_PR0_PRU1_GPO14_CFG_REG                                       (0x000001D8U)
#define CSL_IOMUX_PR0_PRU1_GPO19_CFG_REG                                       (0x000001DCU)
#define CSL_IOMUX_PR0_PRU1_GPO18_CFG_REG                                       (0x000001E0U)
#define CSL_IOMUX_EXT_REFCLK0_CFG_REG                                          (0x000001E4U)
#define CSL_IOMUX_SDFM0_CLK0_CFG_REG                                           (0x000001E8U)
#define CSL_IOMUX_SDFM0_D0_CFG_REG                                             (0x000001ECU)
#define CSL_IOMUX_SDFM0_CLK1_CFG_REG                                           (0x000001F0U)
#define CSL_IOMUX_SDFM0_D1_CFG_REG                                             (0x000001F4U)
#define CSL_IOMUX_SDFM0_CLK2_CFG_REG                                           (0x000001F8U)
#define CSL_IOMUX_SDFM0_D2_CFG_REG                                             (0x000001FCU)
#define CSL_IOMUX_SDFM0_CLK3_CFG_REG                                           (0x00000200U)
#define CSL_IOMUX_SDFM0_D3_CFG_REG                                             (0x00000204U)
#define CSL_IOMUX_EQEP0_A_CFG_REG                                              (0x00000208U)
#define CSL_IOMUX_EQEP0_B_CFG_REG                                              (0x0000020CU)
#define CSL_IOMUX_EQEP0_STROBE_CFG_REG                                         (0x00000210U)
#define CSL_IOMUX_EQEP0_INDEX_CFG_REG                                          (0x00000214U)
#define CSL_IOMUX_I2C0_SDA_CFG_REG                                             (0x00000218U)
#define CSL_IOMUX_I2C0_SCL_CFG_REG                                             (0x0000021CU)
#define CSL_IOMUX_GPIO136_CFG_REG                                               (0x00000220U)
#define CSL_IOMUX_GPIO137_CFG_REG                                               (0x00000224U)
#define CSL_IOMUX_CLKOUT0_CFG_REG                                              (0x00000228U)
#define CSL_IOMUX_USB0_DP_CFG_REG                                               (0x0000022CU)
#define CSL_IOMUX_USB0_DM_CFG_REG                                               (0x00000230U)
#define CSL_IOMUX_WARMRSTN_CFG_REG                                              (0x00000234U)
#define CSL_IOMUX_SAFETY_ERRORN_CFG_REG                                         (0x00000238U)
#define CSL_IOMUX_TDI_CFG_REG                                                   (0x0000023CU)
#define CSL_IOMUX_TDO_CFG_REG                                                   (0x00000240U)
#define CSL_IOMUX_TMS_CFG_REG                                                   (0x00000244U)
#define CSL_IOMUX_TCK_CFG_REG                                                   (0x00000248U)
#define CSL_IOMUX_OSPI0_CLKLB_CFG_REG                                           (0x0000024CU)
#define CSL_IOMUX_OSPI1_CLKLB_CFG_REG                                           (0x00000250U)
#define CSL_IOMUX_QUAL_GRP_0_CFG_REG                                            (0x00000254U)
#define CSL_IOMUX_QUAL_GRP_1_CFG_REG                                            (0x00000258U)
#define CSL_IOMUX_QUAL_GRP_2_CFG_REG                                            (0x0000025CU)
#define CSL_IOMUX_QUAL_GRP_3_CFG_REG                                            (0x00000260U)
#define CSL_IOMUX_QUAL_GRP_4_CFG_REG                                            (0x00000264U)
#define CSL_IOMUX_QUAL_GRP_5_CFG_REG                                            (0x00000268U)
#define CSL_IOMUX_QUAL_GRP_6_CFG_REG                                            (0x0000026CU)
#define CSL_IOMUX_QUAL_GRP_7_CFG_REG                                            (0x00000270U)
#define CSL_IOMUX_QUAL_GRP_8_CFG_REG                                            (0x00000274U)
#define CSL_IOMUX_QUAL_GRP_9_CFG_REG                                            (0x00000278U)
#define CSL_IOMUX_QUAL_GRP_10_CFG_REG                                           (0x0000027CU)
#define CSL_IOMUX_QUAL_GRP_11_CFG_REG                                           (0x00000280U)
#define CSL_IOMUX_QUAL_GRP_12_CFG_REG                                           (0x00000284U)
#define CSL_IOMUX_QUAL_GRP_13_CFG_REG                                           (0x00000288U)
#define CSL_IOMUX_QUAL_GRP_14_CFG_REG                                           (0x0000028CU)
#define CSL_IOMUX_QUAL_GRP_15_CFG_REG                                           (0x00000290U)
#define CSL_IOMUX_QUAL_GRP_16_CFG_REG                                           (0x00000294U)
#define CSL_IOMUX_QUAL_GRP_17_CFG_REG                                           (0x00000298U)
#define CSL_IOMUX_USER_MODE_EN                                                  (0x0000029CU)
#define CSL_IOMUX_PADGLBL_CFG_REG                                               (0x000002A0U)
#define CSL_IOMUX_IO_CFG_KICK0                                                  (0x000002A4U)
#define CSL_IOMUX_IO_CFG_KICK1                                                  (0x000002A8U)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* QSPI0_CSN0_CFG_REG */

#define CSL_IOMUX_QSPI0_CSN0_CFG_REG_FUNC_SEL_MASK                             (0x0000000FU)
#define CSL_IOMUX_QSPI0_CSN0_CFG_REG_FUNC_SEL_SHIFT                            (0x00000000U)
#define CSL_IOMUX_QSPI0_CSN0_CFG_REG_FUNC_SEL_RESETVAL                         (0x00000007U)
#define CSL_IOMUX_QSPI0_CSN0_CFG_REG_FUNC_SEL_MAX                              (0x0000000FU)

#define CSL_IOMUX_QSPI0_CSN0_CFG_REG_IE_OVERRIDE_CTRL_MASK                     (0x00000010U)
#define CSL_IOMUX_QSPI0_CSN0_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                    (0x00000004U)
#define CSL_IOMUX_QSPI0_CSN0_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_IOMUX_QSPI0_CSN0_CFG_REG_IE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_IOMUX_QSPI0_CSN0_CFG_REG_IE_OVERRIDE_MASK                          (0x00000020U)
#define CSL_IOMUX_QSPI0_CSN0_CFG_REG_IE_OVERRIDE_SHIFT                         (0x00000005U)
#define CSL_IOMUX_QSPI0_CSN0_CFG_REG_IE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_IOMUX_QSPI0_CSN0_CFG_REG_IE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_IOMUX_QSPI0_CSN0_CFG_REG_OE_OVERRIDE_CTRL_MASK                     (0x00000040U)
#define CSL_IOMUX_QSPI0_CSN0_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                    (0x00000006U)
#define CSL_IOMUX_QSPI0_CSN0_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_IOMUX_QSPI0_CSN0_CFG_REG_OE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_IOMUX_QSPI0_CSN0_CFG_REG_OE_OVERRIDE_MASK                          (0x00000080U)
#define CSL_IOMUX_QSPI0_CSN0_CFG_REG_OE_OVERRIDE_SHIFT                         (0x00000007U)
#define CSL_IOMUX_QSPI0_CSN0_CFG_REG_OE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_IOMUX_QSPI0_CSN0_CFG_REG_OE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_IOMUX_QSPI0_CSN0_CFG_REG_PI_MASK                                   (0x00000100U)
#define CSL_IOMUX_QSPI0_CSN0_CFG_REG_PI_SHIFT                                  (0x00000008U)
#define CSL_IOMUX_QSPI0_CSN0_CFG_REG_PI_RESETVAL                               (0x00000001U)
#define CSL_IOMUX_QSPI0_CSN0_CFG_REG_PI_MAX                                    (0x00000001U)

#define CSL_IOMUX_QSPI0_CSN0_CFG_REG_PUPDSEL_MASK                              (0x00000200U)
#define CSL_IOMUX_QSPI0_CSN0_CFG_REG_PUPDSEL_SHIFT                             (0x00000009U)
#define CSL_IOMUX_QSPI0_CSN0_CFG_REG_PUPDSEL_RESETVAL                          (0x00000000U)
#define CSL_IOMUX_QSPI0_CSN0_CFG_REG_PUPDSEL_MAX                               (0x00000001U)

#define CSL_IOMUX_QSPI0_CSN0_CFG_REG_SC1_MASK                                  (0x00000400U)
#define CSL_IOMUX_QSPI0_CSN0_CFG_REG_SC1_SHIFT                                 (0x0000000AU)
#define CSL_IOMUX_QSPI0_CSN0_CFG_REG_SC1_RESETVAL                              (0x00000001U)
#define CSL_IOMUX_QSPI0_CSN0_CFG_REG_SC1_MAX                                   (0x00000001U)

#define CSL_IOMUX_QSPI0_CSN0_CFG_REG_GPIO_SEL_MASK                             (0x00030000U)
#define CSL_IOMUX_QSPI0_CSN0_CFG_REG_GPIO_SEL_SHIFT                            (0x00000010U)
#define CSL_IOMUX_QSPI0_CSN0_CFG_REG_GPIO_SEL_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_QSPI0_CSN0_CFG_REG_GPIO_SEL_MAX                              (0x00000003U)

#define CSL_IOMUX_QSPI0_CSN0_CFG_REG_QUAL_SEL_MASK                             (0x000C0000U)
#define CSL_IOMUX_QSPI0_CSN0_CFG_REG_QUAL_SEL_SHIFT                            (0x00000012U)
#define CSL_IOMUX_QSPI0_CSN0_CFG_REG_QUAL_SEL_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_QSPI0_CSN0_CFG_REG_QUAL_SEL_MAX                              (0x00000003U)

#define CSL_IOMUX_QSPI0_CSN0_CFG_REG_INP_INV_SEL_MASK                          (0x00100000U)
#define CSL_IOMUX_QSPI0_CSN0_CFG_REG_INP_INV_SEL_SHIFT                         (0x00000014U)
#define CSL_IOMUX_QSPI0_CSN0_CFG_REG_INP_INV_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_QSPI0_CSN0_CFG_REG_INP_INV_SEL_MAX                           (0x00000001U)

#define CSL_IOMUX_QSPI0_CSN0_CFG_REG_ICSSM_GPIO_SEL_MASK                       (0x00200000U)
#define CSL_IOMUX_QSPI0_CSN0_CFG_REG_ICSSM_GPIO_SEL_SHIFT                      (0x00000015U)
#define CSL_IOMUX_QSPI0_CSN0_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                   (0x00000000U)
#define CSL_IOMUX_QSPI0_CSN0_CFG_REG_ICSSM_GPIO_SEL_MAX                        (0x00000001U)

#define CSL_IOMUX_QSPI0_CSN0_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                  (0x00400000U)
#define CSL_IOMUX_QSPI0_CSN0_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                 (0x00000016U)
#define CSL_IOMUX_QSPI0_CSN0_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL              (0x00000000U)
#define CSL_IOMUX_QSPI0_CSN0_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                   (0x00000001U)

#define CSL_IOMUX_QSPI0_CSN0_CFG_REG_HSMODE_MASK                               (0x40000000U)
#define CSL_IOMUX_QSPI0_CSN0_CFG_REG_HSMODE_SHIFT                              (0x0000001EU)
#define CSL_IOMUX_QSPI0_CSN0_CFG_REG_HSMODE_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_QSPI0_CSN0_CFG_REG_HSMODE_MAX                                (0x00000001U)

#define CSL_IOMUX_QSPI0_CSN0_CFG_REG_HSMASTER_MASK                             (0x80000000U)
#define CSL_IOMUX_QSPI0_CSN0_CFG_REG_HSMASTER_SHIFT                            (0x0000001FU)
#define CSL_IOMUX_QSPI0_CSN0_CFG_REG_HSMASTER_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_QSPI0_CSN0_CFG_REG_HSMASTER_MAX                              (0x00000001U)

#define CSL_IOMUX_QSPI0_CSN0_CFG_REG_RESETVAL                                  (0x000005F7U)

/* QSPI0_CSN1_CFG_REG */

#define CSL_IOMUX_QSPI0_CSN1_CFG_REG_FUNC_SEL_MASK                             (0x0000000FU)
#define CSL_IOMUX_QSPI0_CSN1_CFG_REG_FUNC_SEL_SHIFT                            (0x00000000U)
#define CSL_IOMUX_QSPI0_CSN1_CFG_REG_FUNC_SEL_RESETVAL                         (0x00000007U)
#define CSL_IOMUX_QSPI0_CSN1_CFG_REG_FUNC_SEL_MAX                              (0x0000000FU)

#define CSL_IOMUX_QSPI0_CSN1_CFG_REG_IE_OVERRIDE_CTRL_MASK                     (0x00000010U)
#define CSL_IOMUX_QSPI0_CSN1_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                    (0x00000004U)
#define CSL_IOMUX_QSPI0_CSN1_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_IOMUX_QSPI0_CSN1_CFG_REG_IE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_IOMUX_QSPI0_CSN1_CFG_REG_IE_OVERRIDE_MASK                          (0x00000020U)
#define CSL_IOMUX_QSPI0_CSN1_CFG_REG_IE_OVERRIDE_SHIFT                         (0x00000005U)
#define CSL_IOMUX_QSPI0_CSN1_CFG_REG_IE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_IOMUX_QSPI0_CSN1_CFG_REG_IE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_IOMUX_QSPI0_CSN1_CFG_REG_OE_OVERRIDE_CTRL_MASK                     (0x00000040U)
#define CSL_IOMUX_QSPI0_CSN1_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                    (0x00000006U)
#define CSL_IOMUX_QSPI0_CSN1_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_IOMUX_QSPI0_CSN1_CFG_REG_OE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_IOMUX_QSPI0_CSN1_CFG_REG_OE_OVERRIDE_MASK                          (0x00000080U)
#define CSL_IOMUX_QSPI0_CSN1_CFG_REG_OE_OVERRIDE_SHIFT                         (0x00000007U)
#define CSL_IOMUX_QSPI0_CSN1_CFG_REG_OE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_IOMUX_QSPI0_CSN1_CFG_REG_OE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_IOMUX_QSPI0_CSN1_CFG_REG_PI_MASK                                   (0x00000100U)
#define CSL_IOMUX_QSPI0_CSN1_CFG_REG_PI_SHIFT                                  (0x00000008U)
#define CSL_IOMUX_QSPI0_CSN1_CFG_REG_PI_RESETVAL                               (0x00000001U)
#define CSL_IOMUX_QSPI0_CSN1_CFG_REG_PI_MAX                                    (0x00000001U)

#define CSL_IOMUX_QSPI0_CSN1_CFG_REG_PUPDSEL_MASK                              (0x00000200U)
#define CSL_IOMUX_QSPI0_CSN1_CFG_REG_PUPDSEL_SHIFT                             (0x00000009U)
#define CSL_IOMUX_QSPI0_CSN1_CFG_REG_PUPDSEL_RESETVAL                          (0x00000000U)
#define CSL_IOMUX_QSPI0_CSN1_CFG_REG_PUPDSEL_MAX                               (0x00000001U)

#define CSL_IOMUX_QSPI0_CSN1_CFG_REG_SC1_MASK                                  (0x00000400U)
#define CSL_IOMUX_QSPI0_CSN1_CFG_REG_SC1_SHIFT                                 (0x0000000AU)
#define CSL_IOMUX_QSPI0_CSN1_CFG_REG_SC1_RESETVAL                              (0x00000001U)
#define CSL_IOMUX_QSPI0_CSN1_CFG_REG_SC1_MAX                                   (0x00000001U)

#define CSL_IOMUX_QSPI0_CSN1_CFG_REG_GPIO_SEL_MASK                             (0x00030000U)
#define CSL_IOMUX_QSPI0_CSN1_CFG_REG_GPIO_SEL_SHIFT                            (0x00000010U)
#define CSL_IOMUX_QSPI0_CSN1_CFG_REG_GPIO_SEL_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_QSPI0_CSN1_CFG_REG_GPIO_SEL_MAX                              (0x00000003U)

#define CSL_IOMUX_QSPI0_CSN1_CFG_REG_QUAL_SEL_MASK                             (0x000C0000U)
#define CSL_IOMUX_QSPI0_CSN1_CFG_REG_QUAL_SEL_SHIFT                            (0x00000012U)
#define CSL_IOMUX_QSPI0_CSN1_CFG_REG_QUAL_SEL_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_QSPI0_CSN1_CFG_REG_QUAL_SEL_MAX                              (0x00000003U)

#define CSL_IOMUX_QSPI0_CSN1_CFG_REG_INP_INV_SEL_MASK                          (0x00100000U)
#define CSL_IOMUX_QSPI0_CSN1_CFG_REG_INP_INV_SEL_SHIFT                         (0x00000014U)
#define CSL_IOMUX_QSPI0_CSN1_CFG_REG_INP_INV_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_QSPI0_CSN1_CFG_REG_INP_INV_SEL_MAX                           (0x00000001U)

#define CSL_IOMUX_QSPI0_CSN1_CFG_REG_ICSSM_GPIO_SEL_MASK                       (0x00200000U)
#define CSL_IOMUX_QSPI0_CSN1_CFG_REG_ICSSM_GPIO_SEL_SHIFT                      (0x00000015U)
#define CSL_IOMUX_QSPI0_CSN1_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                   (0x00000000U)
#define CSL_IOMUX_QSPI0_CSN1_CFG_REG_ICSSM_GPIO_SEL_MAX                        (0x00000001U)

#define CSL_IOMUX_QSPI0_CSN1_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                  (0x00400000U)
#define CSL_IOMUX_QSPI0_CSN1_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                 (0x00000016U)
#define CSL_IOMUX_QSPI0_CSN1_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL              (0x00000000U)
#define CSL_IOMUX_QSPI0_CSN1_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                   (0x00000001U)

#define CSL_IOMUX_QSPI0_CSN1_CFG_REG_HSMODE_MASK                               (0x40000000U)
#define CSL_IOMUX_QSPI0_CSN1_CFG_REG_HSMODE_SHIFT                              (0x0000001EU)
#define CSL_IOMUX_QSPI0_CSN1_CFG_REG_HSMODE_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_QSPI0_CSN1_CFG_REG_HSMODE_MAX                                (0x00000001U)

#define CSL_IOMUX_QSPI0_CSN1_CFG_REG_HSMASTER_MASK                             (0x80000000U)
#define CSL_IOMUX_QSPI0_CSN1_CFG_REG_HSMASTER_SHIFT                            (0x0000001FU)
#define CSL_IOMUX_QSPI0_CSN1_CFG_REG_HSMASTER_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_QSPI0_CSN1_CFG_REG_HSMASTER_MAX                              (0x00000001U)

#define CSL_IOMUX_QSPI0_CSN1_CFG_REG_RESETVAL                                  (0x000005F7U)

/* QSPI0_CLK_CFG_REG */

#define CSL_IOMUX_QSPI0_CLK_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_IOMUX_QSPI0_CLK_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_IOMUX_QSPI0_CLK_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000007U)
#define CSL_IOMUX_QSPI0_CLK_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_IOMUX_QSPI0_CLK_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_IOMUX_QSPI0_CLK_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_IOMUX_QSPI0_CLK_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_IOMUX_QSPI0_CLK_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_IOMUX_QSPI0_CLK_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_IOMUX_QSPI0_CLK_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_IOMUX_QSPI0_CLK_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_IOMUX_QSPI0_CLK_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_IOMUX_QSPI0_CLK_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_IOMUX_QSPI0_CLK_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_IOMUX_QSPI0_CLK_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_IOMUX_QSPI0_CLK_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_IOMUX_QSPI0_CLK_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_IOMUX_QSPI0_CLK_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_IOMUX_QSPI0_CLK_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_IOMUX_QSPI0_CLK_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_IOMUX_QSPI0_CLK_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_IOMUX_QSPI0_CLK_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_IOMUX_QSPI0_CLK_CFG_REG_PI_RESETVAL                                (0x00000001U)
#define CSL_IOMUX_QSPI0_CLK_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_IOMUX_QSPI0_CLK_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_IOMUX_QSPI0_CLK_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_IOMUX_QSPI0_CLK_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_QSPI0_CLK_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_IOMUX_QSPI0_CLK_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_IOMUX_QSPI0_CLK_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_IOMUX_QSPI0_CLK_CFG_REG_SC1_RESETVAL                               (0x00000001U)
#define CSL_IOMUX_QSPI0_CLK_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_IOMUX_QSPI0_CLK_CFG_REG_GPIO_SEL_MASK                              (0x00030000U)
#define CSL_IOMUX_QSPI0_CLK_CFG_REG_GPIO_SEL_SHIFT                             (0x00000010U)
#define CSL_IOMUX_QSPI0_CLK_CFG_REG_GPIO_SEL_RESETVAL                          (0x00000000U)
#define CSL_IOMUX_QSPI0_CLK_CFG_REG_GPIO_SEL_MAX                               (0x00000003U)

#define CSL_IOMUX_QSPI0_CLK_CFG_REG_QUAL_SEL_MASK                              (0x000C0000U)
#define CSL_IOMUX_QSPI0_CLK_CFG_REG_QUAL_SEL_SHIFT                             (0x00000012U)
#define CSL_IOMUX_QSPI0_CLK_CFG_REG_QUAL_SEL_RESETVAL                          (0x00000000U)
#define CSL_IOMUX_QSPI0_CLK_CFG_REG_QUAL_SEL_MAX                               (0x00000003U)

#define CSL_IOMUX_QSPI0_CLK_CFG_REG_INP_INV_SEL_MASK                           (0x00100000U)
#define CSL_IOMUX_QSPI0_CLK_CFG_REG_INP_INV_SEL_SHIFT                          (0x00000014U)
#define CSL_IOMUX_QSPI0_CLK_CFG_REG_INP_INV_SEL_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_QSPI0_CLK_CFG_REG_INP_INV_SEL_MAX                            (0x00000001U)

#define CSL_IOMUX_QSPI0_CLK_CFG_REG_ICSSM_GPIO_SEL_MASK                        (0x00200000U)
#define CSL_IOMUX_QSPI0_CLK_CFG_REG_ICSSM_GPIO_SEL_SHIFT                       (0x00000015U)
#define CSL_IOMUX_QSPI0_CLK_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                    (0x00000000U)
#define CSL_IOMUX_QSPI0_CLK_CFG_REG_ICSSM_GPIO_SEL_MAX                         (0x00000001U)

#define CSL_IOMUX_QSPI0_CLK_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                   (0x00400000U)
#define CSL_IOMUX_QSPI0_CLK_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                  (0x00000016U)
#define CSL_IOMUX_QSPI0_CLK_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL               (0x00000000U)
#define CSL_IOMUX_QSPI0_CLK_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                    (0x00000001U)

#define CSL_IOMUX_QSPI0_CLK_CFG_REG_HSMODE_MASK                                (0x40000000U)
#define CSL_IOMUX_QSPI0_CLK_CFG_REG_HSMODE_SHIFT                               (0x0000001EU)
#define CSL_IOMUX_QSPI0_CLK_CFG_REG_HSMODE_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_QSPI0_CLK_CFG_REG_HSMODE_MAX                                 (0x00000001U)

#define CSL_IOMUX_QSPI0_CLK_CFG_REG_HSMASTER_MASK                              (0x80000000U)
#define CSL_IOMUX_QSPI0_CLK_CFG_REG_HSMASTER_SHIFT                             (0x0000001FU)
#define CSL_IOMUX_QSPI0_CLK_CFG_REG_HSMASTER_RESETVAL                          (0x00000000U)
#define CSL_IOMUX_QSPI0_CLK_CFG_REG_HSMASTER_MAX                               (0x00000001U)

#define CSL_IOMUX_QSPI0_CLK_CFG_REG_RESETVAL                                   (0x000005F7U)

/* QSPI0_D0_CFG_REG */

#define CSL_IOMUX_QSPI0_D0_CFG_REG_FUNC_SEL_MASK                               (0x0000000FU)
#define CSL_IOMUX_QSPI0_D0_CFG_REG_FUNC_SEL_SHIFT                              (0x00000000U)
#define CSL_IOMUX_QSPI0_D0_CFG_REG_FUNC_SEL_RESETVAL                           (0x00000007U)
#define CSL_IOMUX_QSPI0_D0_CFG_REG_FUNC_SEL_MAX                                (0x0000000FU)

#define CSL_IOMUX_QSPI0_D0_CFG_REG_IE_OVERRIDE_CTRL_MASK                       (0x00000010U)
#define CSL_IOMUX_QSPI0_D0_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                      (0x00000004U)
#define CSL_IOMUX_QSPI0_D0_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_QSPI0_D0_CFG_REG_IE_OVERRIDE_CTRL_MAX                        (0x00000001U)

#define CSL_IOMUX_QSPI0_D0_CFG_REG_IE_OVERRIDE_MASK                            (0x00000020U)
#define CSL_IOMUX_QSPI0_D0_CFG_REG_IE_OVERRIDE_SHIFT                           (0x00000005U)
#define CSL_IOMUX_QSPI0_D0_CFG_REG_IE_OVERRIDE_RESETVAL                        (0x00000000U)
#define CSL_IOMUX_QSPI0_D0_CFG_REG_IE_OVERRIDE_MAX                             (0x00000001U)

#define CSL_IOMUX_QSPI0_D0_CFG_REG_OE_OVERRIDE_CTRL_MASK                       (0x00000040U)
#define CSL_IOMUX_QSPI0_D0_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                      (0x00000006U)
#define CSL_IOMUX_QSPI0_D0_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_QSPI0_D0_CFG_REG_OE_OVERRIDE_CTRL_MAX                        (0x00000001U)

#define CSL_IOMUX_QSPI0_D0_CFG_REG_OE_OVERRIDE_MASK                            (0x00000080U)
#define CSL_IOMUX_QSPI0_D0_CFG_REG_OE_OVERRIDE_SHIFT                           (0x00000007U)
#define CSL_IOMUX_QSPI0_D0_CFG_REG_OE_OVERRIDE_RESETVAL                        (0x00000001U)
#define CSL_IOMUX_QSPI0_D0_CFG_REG_OE_OVERRIDE_MAX                             (0x00000001U)

#define CSL_IOMUX_QSPI0_D0_CFG_REG_PI_MASK                                     (0x00000100U)
#define CSL_IOMUX_QSPI0_D0_CFG_REG_PI_SHIFT                                    (0x00000008U)
#define CSL_IOMUX_QSPI0_D0_CFG_REG_PI_RESETVAL                                 (0x00000001U)
#define CSL_IOMUX_QSPI0_D0_CFG_REG_PI_MAX                                      (0x00000001U)

#define CSL_IOMUX_QSPI0_D0_CFG_REG_PUPDSEL_MASK                                (0x00000200U)
#define CSL_IOMUX_QSPI0_D0_CFG_REG_PUPDSEL_SHIFT                               (0x00000009U)
#define CSL_IOMUX_QSPI0_D0_CFG_REG_PUPDSEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_QSPI0_D0_CFG_REG_PUPDSEL_MAX                                 (0x00000001U)

#define CSL_IOMUX_QSPI0_D0_CFG_REG_SC1_MASK                                    (0x00000400U)
#define CSL_IOMUX_QSPI0_D0_CFG_REG_SC1_SHIFT                                   (0x0000000AU)
#define CSL_IOMUX_QSPI0_D0_CFG_REG_SC1_RESETVAL                                (0x00000001U)
#define CSL_IOMUX_QSPI0_D0_CFG_REG_SC1_MAX                                     (0x00000001U)

#define CSL_IOMUX_QSPI0_D0_CFG_REG_GPIO_SEL_MASK                               (0x00030000U)
#define CSL_IOMUX_QSPI0_D0_CFG_REG_GPIO_SEL_SHIFT                              (0x00000010U)
#define CSL_IOMUX_QSPI0_D0_CFG_REG_GPIO_SEL_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_QSPI0_D0_CFG_REG_GPIO_SEL_MAX                                (0x00000003U)

#define CSL_IOMUX_QSPI0_D0_CFG_REG_QUAL_SEL_MASK                               (0x000C0000U)
#define CSL_IOMUX_QSPI0_D0_CFG_REG_QUAL_SEL_SHIFT                              (0x00000012U)
#define CSL_IOMUX_QSPI0_D0_CFG_REG_QUAL_SEL_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_QSPI0_D0_CFG_REG_QUAL_SEL_MAX                                (0x00000003U)

#define CSL_IOMUX_QSPI0_D0_CFG_REG_INP_INV_SEL_MASK                            (0x00100000U)
#define CSL_IOMUX_QSPI0_D0_CFG_REG_INP_INV_SEL_SHIFT                           (0x00000014U)
#define CSL_IOMUX_QSPI0_D0_CFG_REG_INP_INV_SEL_RESETVAL                        (0x00000000U)
#define CSL_IOMUX_QSPI0_D0_CFG_REG_INP_INV_SEL_MAX                             (0x00000001U)

#define CSL_IOMUX_QSPI0_D0_CFG_REG_ICSSM_GPIO_SEL_MASK                         (0x00200000U)
#define CSL_IOMUX_QSPI0_D0_CFG_REG_ICSSM_GPIO_SEL_SHIFT                        (0x00000015U)
#define CSL_IOMUX_QSPI0_D0_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_QSPI0_D0_CFG_REG_ICSSM_GPIO_SEL_MAX                          (0x00000001U)

#define CSL_IOMUX_QSPI0_D0_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                    (0x00400000U)
#define CSL_IOMUX_QSPI0_D0_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                   (0x00000016U)
#define CSL_IOMUX_QSPI0_D0_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL                (0x00000000U)
#define CSL_IOMUX_QSPI0_D0_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                     (0x00000001U)

#define CSL_IOMUX_QSPI0_D0_CFG_REG_HSMODE_MASK                                 (0x40000000U)
#define CSL_IOMUX_QSPI0_D0_CFG_REG_HSMODE_SHIFT                                (0x0000001EU)
#define CSL_IOMUX_QSPI0_D0_CFG_REG_HSMODE_RESETVAL                             (0x00000000U)
#define CSL_IOMUX_QSPI0_D0_CFG_REG_HSMODE_MAX                                  (0x00000001U)

#define CSL_IOMUX_QSPI0_D0_CFG_REG_HSMASTER_MASK                               (0x80000000U)
#define CSL_IOMUX_QSPI0_D0_CFG_REG_HSMASTER_SHIFT                              (0x0000001FU)
#define CSL_IOMUX_QSPI0_D0_CFG_REG_HSMASTER_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_QSPI0_D0_CFG_REG_HSMASTER_MAX                                (0x00000001U)

#define CSL_IOMUX_QSPI0_D0_CFG_REG_RESETVAL                                    (0x000005D7U)

/* QSPI0_D1_CFG_REG */

#define CSL_IOMUX_QSPI0_D1_CFG_REG_FUNC_SEL_MASK                               (0x0000000FU)
#define CSL_IOMUX_QSPI0_D1_CFG_REG_FUNC_SEL_SHIFT                              (0x00000000U)
#define CSL_IOMUX_QSPI0_D1_CFG_REG_FUNC_SEL_RESETVAL                           (0x00000007U)
#define CSL_IOMUX_QSPI0_D1_CFG_REG_FUNC_SEL_MAX                                (0x0000000FU)

#define CSL_IOMUX_QSPI0_D1_CFG_REG_IE_OVERRIDE_CTRL_MASK                       (0x00000010U)
#define CSL_IOMUX_QSPI0_D1_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                      (0x00000004U)
#define CSL_IOMUX_QSPI0_D1_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_QSPI0_D1_CFG_REG_IE_OVERRIDE_CTRL_MAX                        (0x00000001U)

#define CSL_IOMUX_QSPI0_D1_CFG_REG_IE_OVERRIDE_MASK                            (0x00000020U)
#define CSL_IOMUX_QSPI0_D1_CFG_REG_IE_OVERRIDE_SHIFT                           (0x00000005U)
#define CSL_IOMUX_QSPI0_D1_CFG_REG_IE_OVERRIDE_RESETVAL                        (0x00000000U)
#define CSL_IOMUX_QSPI0_D1_CFG_REG_IE_OVERRIDE_MAX                             (0x00000001U)

#define CSL_IOMUX_QSPI0_D1_CFG_REG_OE_OVERRIDE_CTRL_MASK                       (0x00000040U)
#define CSL_IOMUX_QSPI0_D1_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                      (0x00000006U)
#define CSL_IOMUX_QSPI0_D1_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_QSPI0_D1_CFG_REG_OE_OVERRIDE_CTRL_MAX                        (0x00000001U)

#define CSL_IOMUX_QSPI0_D1_CFG_REG_OE_OVERRIDE_MASK                            (0x00000080U)
#define CSL_IOMUX_QSPI0_D1_CFG_REG_OE_OVERRIDE_SHIFT                           (0x00000007U)
#define CSL_IOMUX_QSPI0_D1_CFG_REG_OE_OVERRIDE_RESETVAL                        (0x00000001U)
#define CSL_IOMUX_QSPI0_D1_CFG_REG_OE_OVERRIDE_MAX                             (0x00000001U)

#define CSL_IOMUX_QSPI0_D1_CFG_REG_PI_MASK                                     (0x00000100U)
#define CSL_IOMUX_QSPI0_D1_CFG_REG_PI_SHIFT                                    (0x00000008U)
#define CSL_IOMUX_QSPI0_D1_CFG_REG_PI_RESETVAL                                 (0x00000001U)
#define CSL_IOMUX_QSPI0_D1_CFG_REG_PI_MAX                                      (0x00000001U)

#define CSL_IOMUX_QSPI0_D1_CFG_REG_PUPDSEL_MASK                                (0x00000200U)
#define CSL_IOMUX_QSPI0_D1_CFG_REG_PUPDSEL_SHIFT                               (0x00000009U)
#define CSL_IOMUX_QSPI0_D1_CFG_REG_PUPDSEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_QSPI0_D1_CFG_REG_PUPDSEL_MAX                                 (0x00000001U)

#define CSL_IOMUX_QSPI0_D1_CFG_REG_SC1_MASK                                    (0x00000400U)
#define CSL_IOMUX_QSPI0_D1_CFG_REG_SC1_SHIFT                                   (0x0000000AU)
#define CSL_IOMUX_QSPI0_D1_CFG_REG_SC1_RESETVAL                                (0x00000001U)
#define CSL_IOMUX_QSPI0_D1_CFG_REG_SC1_MAX                                     (0x00000001U)

#define CSL_IOMUX_QSPI0_D1_CFG_REG_GPIO_SEL_MASK                               (0x00030000U)
#define CSL_IOMUX_QSPI0_D1_CFG_REG_GPIO_SEL_SHIFT                              (0x00000010U)
#define CSL_IOMUX_QSPI0_D1_CFG_REG_GPIO_SEL_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_QSPI0_D1_CFG_REG_GPIO_SEL_MAX                                (0x00000003U)

#define CSL_IOMUX_QSPI0_D1_CFG_REG_QUAL_SEL_MASK                               (0x000C0000U)
#define CSL_IOMUX_QSPI0_D1_CFG_REG_QUAL_SEL_SHIFT                              (0x00000012U)
#define CSL_IOMUX_QSPI0_D1_CFG_REG_QUAL_SEL_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_QSPI0_D1_CFG_REG_QUAL_SEL_MAX                                (0x00000003U)

#define CSL_IOMUX_QSPI0_D1_CFG_REG_INP_INV_SEL_MASK                            (0x00100000U)
#define CSL_IOMUX_QSPI0_D1_CFG_REG_INP_INV_SEL_SHIFT                           (0x00000014U)
#define CSL_IOMUX_QSPI0_D1_CFG_REG_INP_INV_SEL_RESETVAL                        (0x00000000U)
#define CSL_IOMUX_QSPI0_D1_CFG_REG_INP_INV_SEL_MAX                             (0x00000001U)

#define CSL_IOMUX_QSPI0_D1_CFG_REG_ICSSM_GPIO_SEL_MASK                         (0x00200000U)
#define CSL_IOMUX_QSPI0_D1_CFG_REG_ICSSM_GPIO_SEL_SHIFT                        (0x00000015U)
#define CSL_IOMUX_QSPI0_D1_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_QSPI0_D1_CFG_REG_ICSSM_GPIO_SEL_MAX                          (0x00000001U)

#define CSL_IOMUX_QSPI0_D1_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                    (0x00400000U)
#define CSL_IOMUX_QSPI0_D1_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                   (0x00000016U)
#define CSL_IOMUX_QSPI0_D1_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL                (0x00000000U)
#define CSL_IOMUX_QSPI0_D1_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                     (0x00000001U)

#define CSL_IOMUX_QSPI0_D1_CFG_REG_HSMODE_MASK                                 (0x40000000U)
#define CSL_IOMUX_QSPI0_D1_CFG_REG_HSMODE_SHIFT                                (0x0000001EU)
#define CSL_IOMUX_QSPI0_D1_CFG_REG_HSMODE_RESETVAL                             (0x00000000U)
#define CSL_IOMUX_QSPI0_D1_CFG_REG_HSMODE_MAX                                  (0x00000001U)

#define CSL_IOMUX_QSPI0_D1_CFG_REG_HSMASTER_MASK                               (0x80000000U)
#define CSL_IOMUX_QSPI0_D1_CFG_REG_HSMASTER_SHIFT                              (0x0000001FU)
#define CSL_IOMUX_QSPI0_D1_CFG_REG_HSMASTER_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_QSPI0_D1_CFG_REG_HSMASTER_MAX                                (0x00000001U)

#define CSL_IOMUX_QSPI0_D1_CFG_REG_RESETVAL                                    (0x000005D7U)

/* QSPI0_D2_CFG_REG */

#define CSL_IOMUX_QSPI0_D2_CFG_REG_FUNC_SEL_MASK                               (0x0000000FU)
#define CSL_IOMUX_QSPI0_D2_CFG_REG_FUNC_SEL_SHIFT                              (0x00000000U)
#define CSL_IOMUX_QSPI0_D2_CFG_REG_FUNC_SEL_RESETVAL                           (0x00000007U)
#define CSL_IOMUX_QSPI0_D2_CFG_REG_FUNC_SEL_MAX                                (0x0000000FU)

#define CSL_IOMUX_QSPI0_D2_CFG_REG_IE_OVERRIDE_CTRL_MASK                       (0x00000010U)
#define CSL_IOMUX_QSPI0_D2_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                      (0x00000004U)
#define CSL_IOMUX_QSPI0_D2_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_QSPI0_D2_CFG_REG_IE_OVERRIDE_CTRL_MAX                        (0x00000001U)

#define CSL_IOMUX_QSPI0_D2_CFG_REG_IE_OVERRIDE_MASK                            (0x00000020U)
#define CSL_IOMUX_QSPI0_D2_CFG_REG_IE_OVERRIDE_SHIFT                           (0x00000005U)
#define CSL_IOMUX_QSPI0_D2_CFG_REG_IE_OVERRIDE_RESETVAL                        (0x00000001U)
#define CSL_IOMUX_QSPI0_D2_CFG_REG_IE_OVERRIDE_MAX                             (0x00000001U)

#define CSL_IOMUX_QSPI0_D2_CFG_REG_OE_OVERRIDE_CTRL_MASK                       (0x00000040U)
#define CSL_IOMUX_QSPI0_D2_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                      (0x00000006U)
#define CSL_IOMUX_QSPI0_D2_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_QSPI0_D2_CFG_REG_OE_OVERRIDE_CTRL_MAX                        (0x00000001U)

#define CSL_IOMUX_QSPI0_D2_CFG_REG_OE_OVERRIDE_MASK                            (0x00000080U)
#define CSL_IOMUX_QSPI0_D2_CFG_REG_OE_OVERRIDE_SHIFT                           (0x00000007U)
#define CSL_IOMUX_QSPI0_D2_CFG_REG_OE_OVERRIDE_RESETVAL                        (0x00000001U)
#define CSL_IOMUX_QSPI0_D2_CFG_REG_OE_OVERRIDE_MAX                             (0x00000001U)

#define CSL_IOMUX_QSPI0_D2_CFG_REG_PI_MASK                                     (0x00000100U)
#define CSL_IOMUX_QSPI0_D2_CFG_REG_PI_SHIFT                                    (0x00000008U)
#define CSL_IOMUX_QSPI0_D2_CFG_REG_PI_RESETVAL                                 (0x00000001U)
#define CSL_IOMUX_QSPI0_D2_CFG_REG_PI_MAX                                      (0x00000001U)

#define CSL_IOMUX_QSPI0_D2_CFG_REG_PUPDSEL_MASK                                (0x00000200U)
#define CSL_IOMUX_QSPI0_D2_CFG_REG_PUPDSEL_SHIFT                               (0x00000009U)
#define CSL_IOMUX_QSPI0_D2_CFG_REG_PUPDSEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_QSPI0_D2_CFG_REG_PUPDSEL_MAX                                 (0x00000001U)

#define CSL_IOMUX_QSPI0_D2_CFG_REG_SC1_MASK                                    (0x00000400U)
#define CSL_IOMUX_QSPI0_D2_CFG_REG_SC1_SHIFT                                   (0x0000000AU)
#define CSL_IOMUX_QSPI0_D2_CFG_REG_SC1_RESETVAL                                (0x00000001U)
#define CSL_IOMUX_QSPI0_D2_CFG_REG_SC1_MAX                                     (0x00000001U)

#define CSL_IOMUX_QSPI0_D2_CFG_REG_GPIO_SEL_MASK                               (0x00030000U)
#define CSL_IOMUX_QSPI0_D2_CFG_REG_GPIO_SEL_SHIFT                              (0x00000010U)
#define CSL_IOMUX_QSPI0_D2_CFG_REG_GPIO_SEL_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_QSPI0_D2_CFG_REG_GPIO_SEL_MAX                                (0x00000003U)

#define CSL_IOMUX_QSPI0_D2_CFG_REG_QUAL_SEL_MASK                               (0x000C0000U)
#define CSL_IOMUX_QSPI0_D2_CFG_REG_QUAL_SEL_SHIFT                              (0x00000012U)
#define CSL_IOMUX_QSPI0_D2_CFG_REG_QUAL_SEL_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_QSPI0_D2_CFG_REG_QUAL_SEL_MAX                                (0x00000003U)

#define CSL_IOMUX_QSPI0_D2_CFG_REG_INP_INV_SEL_MASK                            (0x00100000U)
#define CSL_IOMUX_QSPI0_D2_CFG_REG_INP_INV_SEL_SHIFT                           (0x00000014U)
#define CSL_IOMUX_QSPI0_D2_CFG_REG_INP_INV_SEL_RESETVAL                        (0x00000000U)
#define CSL_IOMUX_QSPI0_D2_CFG_REG_INP_INV_SEL_MAX                             (0x00000001U)

#define CSL_IOMUX_QSPI0_D2_CFG_REG_ICSSM_GPIO_SEL_MASK                         (0x00200000U)
#define CSL_IOMUX_QSPI0_D2_CFG_REG_ICSSM_GPIO_SEL_SHIFT                        (0x00000015U)
#define CSL_IOMUX_QSPI0_D2_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_QSPI0_D2_CFG_REG_ICSSM_GPIO_SEL_MAX                          (0x00000001U)

#define CSL_IOMUX_QSPI0_D2_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                    (0x00400000U)
#define CSL_IOMUX_QSPI0_D2_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                   (0x00000016U)
#define CSL_IOMUX_QSPI0_D2_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL                (0x00000000U)
#define CSL_IOMUX_QSPI0_D2_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                     (0x00000001U)

#define CSL_IOMUX_QSPI0_D2_CFG_REG_HSMODE_MASK                                 (0x40000000U)
#define CSL_IOMUX_QSPI0_D2_CFG_REG_HSMODE_SHIFT                                (0x0000001EU)
#define CSL_IOMUX_QSPI0_D2_CFG_REG_HSMODE_RESETVAL                             (0x00000000U)
#define CSL_IOMUX_QSPI0_D2_CFG_REG_HSMODE_MAX                                  (0x00000001U)

#define CSL_IOMUX_QSPI0_D2_CFG_REG_HSMASTER_MASK                               (0x80000000U)
#define CSL_IOMUX_QSPI0_D2_CFG_REG_HSMASTER_SHIFT                              (0x0000001FU)
#define CSL_IOMUX_QSPI0_D2_CFG_REG_HSMASTER_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_QSPI0_D2_CFG_REG_HSMASTER_MAX                                (0x00000001U)

#define CSL_IOMUX_QSPI0_D2_CFG_REG_RESETVAL                                    (0x000005F7U)

/* QSPI0_D3_CFG_REG */

#define CSL_IOMUX_QSPI0_D3_CFG_REG_FUNC_SEL_MASK                               (0x0000000FU)
#define CSL_IOMUX_QSPI0_D3_CFG_REG_FUNC_SEL_SHIFT                              (0x00000000U)
#define CSL_IOMUX_QSPI0_D3_CFG_REG_FUNC_SEL_RESETVAL                           (0x00000007U)
#define CSL_IOMUX_QSPI0_D3_CFG_REG_FUNC_SEL_MAX                                (0x0000000FU)

#define CSL_IOMUX_QSPI0_D3_CFG_REG_IE_OVERRIDE_CTRL_MASK                       (0x00000010U)
#define CSL_IOMUX_QSPI0_D3_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                      (0x00000004U)
#define CSL_IOMUX_QSPI0_D3_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_QSPI0_D3_CFG_REG_IE_OVERRIDE_CTRL_MAX                        (0x00000001U)

#define CSL_IOMUX_QSPI0_D3_CFG_REG_IE_OVERRIDE_MASK                            (0x00000020U)
#define CSL_IOMUX_QSPI0_D3_CFG_REG_IE_OVERRIDE_SHIFT                           (0x00000005U)
#define CSL_IOMUX_QSPI0_D3_CFG_REG_IE_OVERRIDE_RESETVAL                        (0x00000001U)
#define CSL_IOMUX_QSPI0_D3_CFG_REG_IE_OVERRIDE_MAX                             (0x00000001U)

#define CSL_IOMUX_QSPI0_D3_CFG_REG_OE_OVERRIDE_CTRL_MASK                       (0x00000040U)
#define CSL_IOMUX_QSPI0_D3_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                      (0x00000006U)
#define CSL_IOMUX_QSPI0_D3_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_QSPI0_D3_CFG_REG_OE_OVERRIDE_CTRL_MAX                        (0x00000001U)

#define CSL_IOMUX_QSPI0_D3_CFG_REG_OE_OVERRIDE_MASK                            (0x00000080U)
#define CSL_IOMUX_QSPI0_D3_CFG_REG_OE_OVERRIDE_SHIFT                           (0x00000007U)
#define CSL_IOMUX_QSPI0_D3_CFG_REG_OE_OVERRIDE_RESETVAL                        (0x00000001U)
#define CSL_IOMUX_QSPI0_D3_CFG_REG_OE_OVERRIDE_MAX                             (0x00000001U)

#define CSL_IOMUX_QSPI0_D3_CFG_REG_PI_MASK                                     (0x00000100U)
#define CSL_IOMUX_QSPI0_D3_CFG_REG_PI_SHIFT                                    (0x00000008U)
#define CSL_IOMUX_QSPI0_D3_CFG_REG_PI_RESETVAL                                 (0x00000001U)
#define CSL_IOMUX_QSPI0_D3_CFG_REG_PI_MAX                                      (0x00000001U)

#define CSL_IOMUX_QSPI0_D3_CFG_REG_PUPDSEL_MASK                                (0x00000200U)
#define CSL_IOMUX_QSPI0_D3_CFG_REG_PUPDSEL_SHIFT                               (0x00000009U)
#define CSL_IOMUX_QSPI0_D3_CFG_REG_PUPDSEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_QSPI0_D3_CFG_REG_PUPDSEL_MAX                                 (0x00000001U)

#define CSL_IOMUX_QSPI0_D3_CFG_REG_SC1_MASK                                    (0x00000400U)
#define CSL_IOMUX_QSPI0_D3_CFG_REG_SC1_SHIFT                                   (0x0000000AU)
#define CSL_IOMUX_QSPI0_D3_CFG_REG_SC1_RESETVAL                                (0x00000001U)
#define CSL_IOMUX_QSPI0_D3_CFG_REG_SC1_MAX                                     (0x00000001U)

#define CSL_IOMUX_QSPI0_D3_CFG_REG_GPIO_SEL_MASK                               (0x00030000U)
#define CSL_IOMUX_QSPI0_D3_CFG_REG_GPIO_SEL_SHIFT                              (0x00000010U)
#define CSL_IOMUX_QSPI0_D3_CFG_REG_GPIO_SEL_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_QSPI0_D3_CFG_REG_GPIO_SEL_MAX                                (0x00000003U)

#define CSL_IOMUX_QSPI0_D3_CFG_REG_QUAL_SEL_MASK                               (0x000C0000U)
#define CSL_IOMUX_QSPI0_D3_CFG_REG_QUAL_SEL_SHIFT                              (0x00000012U)
#define CSL_IOMUX_QSPI0_D3_CFG_REG_QUAL_SEL_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_QSPI0_D3_CFG_REG_QUAL_SEL_MAX                                (0x00000003U)

#define CSL_IOMUX_QSPI0_D3_CFG_REG_INP_INV_SEL_MASK                            (0x00100000U)
#define CSL_IOMUX_QSPI0_D3_CFG_REG_INP_INV_SEL_SHIFT                           (0x00000014U)
#define CSL_IOMUX_QSPI0_D3_CFG_REG_INP_INV_SEL_RESETVAL                        (0x00000000U)
#define CSL_IOMUX_QSPI0_D3_CFG_REG_INP_INV_SEL_MAX                             (0x00000001U)

#define CSL_IOMUX_QSPI0_D3_CFG_REG_ICSSM_GPIO_SEL_MASK                         (0x00200000U)
#define CSL_IOMUX_QSPI0_D3_CFG_REG_ICSSM_GPIO_SEL_SHIFT                        (0x00000015U)
#define CSL_IOMUX_QSPI0_D3_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_QSPI0_D3_CFG_REG_ICSSM_GPIO_SEL_MAX                          (0x00000001U)

#define CSL_IOMUX_QSPI0_D3_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                    (0x00400000U)
#define CSL_IOMUX_QSPI0_D3_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                   (0x00000016U)
#define CSL_IOMUX_QSPI0_D3_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL                (0x00000000U)
#define CSL_IOMUX_QSPI0_D3_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                     (0x00000001U)

#define CSL_IOMUX_QSPI0_D3_CFG_REG_HSMODE_MASK                                 (0x40000000U)
#define CSL_IOMUX_QSPI0_D3_CFG_REG_HSMODE_SHIFT                                (0x0000001EU)
#define CSL_IOMUX_QSPI0_D3_CFG_REG_HSMODE_RESETVAL                             (0x00000000U)
#define CSL_IOMUX_QSPI0_D3_CFG_REG_HSMODE_MAX                                  (0x00000001U)

#define CSL_IOMUX_QSPI0_D3_CFG_REG_HSMASTER_MASK                               (0x80000000U)
#define CSL_IOMUX_QSPI0_D3_CFG_REG_HSMASTER_SHIFT                              (0x0000001FU)
#define CSL_IOMUX_QSPI0_D3_CFG_REG_HSMASTER_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_QSPI0_D3_CFG_REG_HSMASTER_MAX                                (0x00000001U)

#define CSL_IOMUX_QSPI0_D3_CFG_REG_RESETVAL                                    (0x000005F7U)

/* MCAN0_RX_CFG_REG */

#define CSL_IOMUX_MCAN0_RX_CFG_REG_FUNC_SEL_MASK                               (0x0000000FU)
#define CSL_IOMUX_MCAN0_RX_CFG_REG_FUNC_SEL_SHIFT                              (0x00000000U)
#define CSL_IOMUX_MCAN0_RX_CFG_REG_FUNC_SEL_RESETVAL                           (0x00000007U)
#define CSL_IOMUX_MCAN0_RX_CFG_REG_FUNC_SEL_MAX                                (0x0000000FU)

#define CSL_IOMUX_MCAN0_RX_CFG_REG_IE_OVERRIDE_CTRL_MASK                       (0x00000010U)
#define CSL_IOMUX_MCAN0_RX_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                      (0x00000004U)
#define CSL_IOMUX_MCAN0_RX_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_MCAN0_RX_CFG_REG_IE_OVERRIDE_CTRL_MAX                        (0x00000001U)

#define CSL_IOMUX_MCAN0_RX_CFG_REG_IE_OVERRIDE_MASK                            (0x00000020U)
#define CSL_IOMUX_MCAN0_RX_CFG_REG_IE_OVERRIDE_SHIFT                           (0x00000005U)
#define CSL_IOMUX_MCAN0_RX_CFG_REG_IE_OVERRIDE_RESETVAL                        (0x00000001U)
#define CSL_IOMUX_MCAN0_RX_CFG_REG_IE_OVERRIDE_MAX                             (0x00000001U)

#define CSL_IOMUX_MCAN0_RX_CFG_REG_OE_OVERRIDE_CTRL_MASK                       (0x00000040U)
#define CSL_IOMUX_MCAN0_RX_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                      (0x00000006U)
#define CSL_IOMUX_MCAN0_RX_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_MCAN0_RX_CFG_REG_OE_OVERRIDE_CTRL_MAX                        (0x00000001U)

#define CSL_IOMUX_MCAN0_RX_CFG_REG_OE_OVERRIDE_MASK                            (0x00000080U)
#define CSL_IOMUX_MCAN0_RX_CFG_REG_OE_OVERRIDE_SHIFT                           (0x00000007U)
#define CSL_IOMUX_MCAN0_RX_CFG_REG_OE_OVERRIDE_RESETVAL                        (0x00000001U)
#define CSL_IOMUX_MCAN0_RX_CFG_REG_OE_OVERRIDE_MAX                             (0x00000001U)

#define CSL_IOMUX_MCAN0_RX_CFG_REG_PI_MASK                                     (0x00000100U)
#define CSL_IOMUX_MCAN0_RX_CFG_REG_PI_SHIFT                                    (0x00000008U)
#define CSL_IOMUX_MCAN0_RX_CFG_REG_PI_RESETVAL                                 (0x00000001U)
#define CSL_IOMUX_MCAN0_RX_CFG_REG_PI_MAX                                      (0x00000001U)

#define CSL_IOMUX_MCAN0_RX_CFG_REG_PUPDSEL_MASK                                (0x00000200U)
#define CSL_IOMUX_MCAN0_RX_CFG_REG_PUPDSEL_SHIFT                               (0x00000009U)
#define CSL_IOMUX_MCAN0_RX_CFG_REG_PUPDSEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_MCAN0_RX_CFG_REG_PUPDSEL_MAX                                 (0x00000001U)

#define CSL_IOMUX_MCAN0_RX_CFG_REG_SC1_MASK                                    (0x00000400U)
#define CSL_IOMUX_MCAN0_RX_CFG_REG_SC1_SHIFT                                   (0x0000000AU)
#define CSL_IOMUX_MCAN0_RX_CFG_REG_SC1_RESETVAL                                (0x00000001U)
#define CSL_IOMUX_MCAN0_RX_CFG_REG_SC1_MAX                                     (0x00000001U)

#define CSL_IOMUX_MCAN0_RX_CFG_REG_GPIO_SEL_MASK                               (0x00030000U)
#define CSL_IOMUX_MCAN0_RX_CFG_REG_GPIO_SEL_SHIFT                              (0x00000010U)
#define CSL_IOMUX_MCAN0_RX_CFG_REG_GPIO_SEL_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_MCAN0_RX_CFG_REG_GPIO_SEL_MAX                                (0x00000003U)

#define CSL_IOMUX_MCAN0_RX_CFG_REG_QUAL_SEL_MASK                               (0x000C0000U)
#define CSL_IOMUX_MCAN0_RX_CFG_REG_QUAL_SEL_SHIFT                              (0x00000012U)
#define CSL_IOMUX_MCAN0_RX_CFG_REG_QUAL_SEL_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_MCAN0_RX_CFG_REG_QUAL_SEL_MAX                                (0x00000003U)

#define CSL_IOMUX_MCAN0_RX_CFG_REG_INP_INV_SEL_MASK                            (0x00100000U)
#define CSL_IOMUX_MCAN0_RX_CFG_REG_INP_INV_SEL_SHIFT                           (0x00000014U)
#define CSL_IOMUX_MCAN0_RX_CFG_REG_INP_INV_SEL_RESETVAL                        (0x00000000U)
#define CSL_IOMUX_MCAN0_RX_CFG_REG_INP_INV_SEL_MAX                             (0x00000001U)

#define CSL_IOMUX_MCAN0_RX_CFG_REG_ICSSM_GPIO_SEL_MASK                         (0x00200000U)
#define CSL_IOMUX_MCAN0_RX_CFG_REG_ICSSM_GPIO_SEL_SHIFT                        (0x00000015U)
#define CSL_IOMUX_MCAN0_RX_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_MCAN0_RX_CFG_REG_ICSSM_GPIO_SEL_MAX                          (0x00000001U)

#define CSL_IOMUX_MCAN0_RX_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                    (0x00400000U)
#define CSL_IOMUX_MCAN0_RX_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                   (0x00000016U)
#define CSL_IOMUX_MCAN0_RX_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL                (0x00000000U)
#define CSL_IOMUX_MCAN0_RX_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                     (0x00000001U)

#define CSL_IOMUX_MCAN0_RX_CFG_REG_HSMODE_MASK                                 (0x40000000U)
#define CSL_IOMUX_MCAN0_RX_CFG_REG_HSMODE_SHIFT                                (0x0000001EU)
#define CSL_IOMUX_MCAN0_RX_CFG_REG_HSMODE_RESETVAL                             (0x00000000U)
#define CSL_IOMUX_MCAN0_RX_CFG_REG_HSMODE_MAX                                  (0x00000001U)

#define CSL_IOMUX_MCAN0_RX_CFG_REG_HSMASTER_MASK                               (0x80000000U)
#define CSL_IOMUX_MCAN0_RX_CFG_REG_HSMASTER_SHIFT                              (0x0000001FU)
#define CSL_IOMUX_MCAN0_RX_CFG_REG_HSMASTER_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_MCAN0_RX_CFG_REG_HSMASTER_MAX                                (0x00000001U)

#define CSL_IOMUX_MCAN0_RX_CFG_REG_RESETVAL                                    (0x000005F7U)

/* MCAN0_TX_CFG_REG */

#define CSL_IOMUX_MCAN0_TX_CFG_REG_FUNC_SEL_MASK                               (0x0000000FU)
#define CSL_IOMUX_MCAN0_TX_CFG_REG_FUNC_SEL_SHIFT                              (0x00000000U)
#define CSL_IOMUX_MCAN0_TX_CFG_REG_FUNC_SEL_RESETVAL                           (0x00000007U)
#define CSL_IOMUX_MCAN0_TX_CFG_REG_FUNC_SEL_MAX                                (0x0000000FU)

#define CSL_IOMUX_MCAN0_TX_CFG_REG_IE_OVERRIDE_CTRL_MASK                       (0x00000010U)
#define CSL_IOMUX_MCAN0_TX_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                      (0x00000004U)
#define CSL_IOMUX_MCAN0_TX_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_MCAN0_TX_CFG_REG_IE_OVERRIDE_CTRL_MAX                        (0x00000001U)

#define CSL_IOMUX_MCAN0_TX_CFG_REG_IE_OVERRIDE_MASK                            (0x00000020U)
#define CSL_IOMUX_MCAN0_TX_CFG_REG_IE_OVERRIDE_SHIFT                           (0x00000005U)
#define CSL_IOMUX_MCAN0_TX_CFG_REG_IE_OVERRIDE_RESETVAL                        (0x00000001U)
#define CSL_IOMUX_MCAN0_TX_CFG_REG_IE_OVERRIDE_MAX                             (0x00000001U)

#define CSL_IOMUX_MCAN0_TX_CFG_REG_OE_OVERRIDE_CTRL_MASK                       (0x00000040U)
#define CSL_IOMUX_MCAN0_TX_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                      (0x00000006U)
#define CSL_IOMUX_MCAN0_TX_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_MCAN0_TX_CFG_REG_OE_OVERRIDE_CTRL_MAX                        (0x00000001U)

#define CSL_IOMUX_MCAN0_TX_CFG_REG_OE_OVERRIDE_MASK                            (0x00000080U)
#define CSL_IOMUX_MCAN0_TX_CFG_REG_OE_OVERRIDE_SHIFT                           (0x00000007U)
#define CSL_IOMUX_MCAN0_TX_CFG_REG_OE_OVERRIDE_RESETVAL                        (0x00000001U)
#define CSL_IOMUX_MCAN0_TX_CFG_REG_OE_OVERRIDE_MAX                             (0x00000001U)

#define CSL_IOMUX_MCAN0_TX_CFG_REG_PI_MASK                                     (0x00000100U)
#define CSL_IOMUX_MCAN0_TX_CFG_REG_PI_SHIFT                                    (0x00000008U)
#define CSL_IOMUX_MCAN0_TX_CFG_REG_PI_RESETVAL                                 (0x00000001U)
#define CSL_IOMUX_MCAN0_TX_CFG_REG_PI_MAX                                      (0x00000001U)

#define CSL_IOMUX_MCAN0_TX_CFG_REG_PUPDSEL_MASK                                (0x00000200U)
#define CSL_IOMUX_MCAN0_TX_CFG_REG_PUPDSEL_SHIFT                               (0x00000009U)
#define CSL_IOMUX_MCAN0_TX_CFG_REG_PUPDSEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_MCAN0_TX_CFG_REG_PUPDSEL_MAX                                 (0x00000001U)

#define CSL_IOMUX_MCAN0_TX_CFG_REG_SC1_MASK                                    (0x00000400U)
#define CSL_IOMUX_MCAN0_TX_CFG_REG_SC1_SHIFT                                   (0x0000000AU)
#define CSL_IOMUX_MCAN0_TX_CFG_REG_SC1_RESETVAL                                (0x00000001U)
#define CSL_IOMUX_MCAN0_TX_CFG_REG_SC1_MAX                                     (0x00000001U)

#define CSL_IOMUX_MCAN0_TX_CFG_REG_GPIO_SEL_MASK                               (0x00030000U)
#define CSL_IOMUX_MCAN0_TX_CFG_REG_GPIO_SEL_SHIFT                              (0x00000010U)
#define CSL_IOMUX_MCAN0_TX_CFG_REG_GPIO_SEL_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_MCAN0_TX_CFG_REG_GPIO_SEL_MAX                                (0x00000003U)

#define CSL_IOMUX_MCAN0_TX_CFG_REG_QUAL_SEL_MASK                               (0x000C0000U)
#define CSL_IOMUX_MCAN0_TX_CFG_REG_QUAL_SEL_SHIFT                              (0x00000012U)
#define CSL_IOMUX_MCAN0_TX_CFG_REG_QUAL_SEL_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_MCAN0_TX_CFG_REG_QUAL_SEL_MAX                                (0x00000003U)

#define CSL_IOMUX_MCAN0_TX_CFG_REG_INP_INV_SEL_MASK                            (0x00100000U)
#define CSL_IOMUX_MCAN0_TX_CFG_REG_INP_INV_SEL_SHIFT                           (0x00000014U)
#define CSL_IOMUX_MCAN0_TX_CFG_REG_INP_INV_SEL_RESETVAL                        (0x00000000U)
#define CSL_IOMUX_MCAN0_TX_CFG_REG_INP_INV_SEL_MAX                             (0x00000001U)

#define CSL_IOMUX_MCAN0_TX_CFG_REG_ICSSM_GPIO_SEL_MASK                         (0x00200000U)
#define CSL_IOMUX_MCAN0_TX_CFG_REG_ICSSM_GPIO_SEL_SHIFT                        (0x00000015U)
#define CSL_IOMUX_MCAN0_TX_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_MCAN0_TX_CFG_REG_ICSSM_GPIO_SEL_MAX                          (0x00000001U)

#define CSL_IOMUX_MCAN0_TX_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                    (0x00400000U)
#define CSL_IOMUX_MCAN0_TX_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                   (0x00000016U)
#define CSL_IOMUX_MCAN0_TX_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL                (0x00000000U)
#define CSL_IOMUX_MCAN0_TX_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                     (0x00000001U)

#define CSL_IOMUX_MCAN0_TX_CFG_REG_HSMODE_MASK                                 (0x40000000U)
#define CSL_IOMUX_MCAN0_TX_CFG_REG_HSMODE_SHIFT                                (0x0000001EU)
#define CSL_IOMUX_MCAN0_TX_CFG_REG_HSMODE_RESETVAL                             (0x00000000U)
#define CSL_IOMUX_MCAN0_TX_CFG_REG_HSMODE_MAX                                  (0x00000001U)

#define CSL_IOMUX_MCAN0_TX_CFG_REG_HSMASTER_MASK                               (0x80000000U)
#define CSL_IOMUX_MCAN0_TX_CFG_REG_HSMASTER_SHIFT                              (0x0000001FU)
#define CSL_IOMUX_MCAN0_TX_CFG_REG_HSMASTER_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_MCAN0_TX_CFG_REG_HSMASTER_MAX                                (0x00000001U)

#define CSL_IOMUX_MCAN0_TX_CFG_REG_RESETVAL                                    (0x000005F7U)

/* MCAN1_RX_CFG_REG */

#define CSL_IOMUX_MCAN1_RX_CFG_REG_FUNC_SEL_MASK                               (0x0000000FU)
#define CSL_IOMUX_MCAN1_RX_CFG_REG_FUNC_SEL_SHIFT                              (0x00000000U)
#define CSL_IOMUX_MCAN1_RX_CFG_REG_FUNC_SEL_RESETVAL                           (0x00000007U)
#define CSL_IOMUX_MCAN1_RX_CFG_REG_FUNC_SEL_MAX                                (0x0000000FU)

#define CSL_IOMUX_MCAN1_RX_CFG_REG_IE_OVERRIDE_CTRL_MASK                       (0x00000010U)
#define CSL_IOMUX_MCAN1_RX_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                      (0x00000004U)
#define CSL_IOMUX_MCAN1_RX_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_MCAN1_RX_CFG_REG_IE_OVERRIDE_CTRL_MAX                        (0x00000001U)

#define CSL_IOMUX_MCAN1_RX_CFG_REG_IE_OVERRIDE_MASK                            (0x00000020U)
#define CSL_IOMUX_MCAN1_RX_CFG_REG_IE_OVERRIDE_SHIFT                           (0x00000005U)
#define CSL_IOMUX_MCAN1_RX_CFG_REG_IE_OVERRIDE_RESETVAL                        (0x00000001U)
#define CSL_IOMUX_MCAN1_RX_CFG_REG_IE_OVERRIDE_MAX                             (0x00000001U)

#define CSL_IOMUX_MCAN1_RX_CFG_REG_OE_OVERRIDE_CTRL_MASK                       (0x00000040U)
#define CSL_IOMUX_MCAN1_RX_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                      (0x00000006U)
#define CSL_IOMUX_MCAN1_RX_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_MCAN1_RX_CFG_REG_OE_OVERRIDE_CTRL_MAX                        (0x00000001U)

#define CSL_IOMUX_MCAN1_RX_CFG_REG_OE_OVERRIDE_MASK                            (0x00000080U)
#define CSL_IOMUX_MCAN1_RX_CFG_REG_OE_OVERRIDE_SHIFT                           (0x00000007U)
#define CSL_IOMUX_MCAN1_RX_CFG_REG_OE_OVERRIDE_RESETVAL                        (0x00000001U)
#define CSL_IOMUX_MCAN1_RX_CFG_REG_OE_OVERRIDE_MAX                             (0x00000001U)

#define CSL_IOMUX_MCAN1_RX_CFG_REG_PI_MASK                                     (0x00000100U)
#define CSL_IOMUX_MCAN1_RX_CFG_REG_PI_SHIFT                                    (0x00000008U)
#define CSL_IOMUX_MCAN1_RX_CFG_REG_PI_RESETVAL                                 (0x00000001U)
#define CSL_IOMUX_MCAN1_RX_CFG_REG_PI_MAX                                      (0x00000001U)

#define CSL_IOMUX_MCAN1_RX_CFG_REG_PUPDSEL_MASK                                (0x00000200U)
#define CSL_IOMUX_MCAN1_RX_CFG_REG_PUPDSEL_SHIFT                               (0x00000009U)
#define CSL_IOMUX_MCAN1_RX_CFG_REG_PUPDSEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_MCAN1_RX_CFG_REG_PUPDSEL_MAX                                 (0x00000001U)

#define CSL_IOMUX_MCAN1_RX_CFG_REG_SC1_MASK                                    (0x00000400U)
#define CSL_IOMUX_MCAN1_RX_CFG_REG_SC1_SHIFT                                   (0x0000000AU)
#define CSL_IOMUX_MCAN1_RX_CFG_REG_SC1_RESETVAL                                (0x00000001U)
#define CSL_IOMUX_MCAN1_RX_CFG_REG_SC1_MAX                                     (0x00000001U)

#define CSL_IOMUX_MCAN1_RX_CFG_REG_GPIO_SEL_MASK                               (0x00030000U)
#define CSL_IOMUX_MCAN1_RX_CFG_REG_GPIO_SEL_SHIFT                              (0x00000010U)
#define CSL_IOMUX_MCAN1_RX_CFG_REG_GPIO_SEL_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_MCAN1_RX_CFG_REG_GPIO_SEL_MAX                                (0x00000003U)

#define CSL_IOMUX_MCAN1_RX_CFG_REG_QUAL_SEL_MASK                               (0x000C0000U)
#define CSL_IOMUX_MCAN1_RX_CFG_REG_QUAL_SEL_SHIFT                              (0x00000012U)
#define CSL_IOMUX_MCAN1_RX_CFG_REG_QUAL_SEL_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_MCAN1_RX_CFG_REG_QUAL_SEL_MAX                                (0x00000003U)

#define CSL_IOMUX_MCAN1_RX_CFG_REG_INP_INV_SEL_MASK                            (0x00100000U)
#define CSL_IOMUX_MCAN1_RX_CFG_REG_INP_INV_SEL_SHIFT                           (0x00000014U)
#define CSL_IOMUX_MCAN1_RX_CFG_REG_INP_INV_SEL_RESETVAL                        (0x00000000U)
#define CSL_IOMUX_MCAN1_RX_CFG_REG_INP_INV_SEL_MAX                             (0x00000001U)

#define CSL_IOMUX_MCAN1_RX_CFG_REG_ICSSM_GPIO_SEL_MASK                         (0x00200000U)
#define CSL_IOMUX_MCAN1_RX_CFG_REG_ICSSM_GPIO_SEL_SHIFT                        (0x00000015U)
#define CSL_IOMUX_MCAN1_RX_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_MCAN1_RX_CFG_REG_ICSSM_GPIO_SEL_MAX                          (0x00000001U)

#define CSL_IOMUX_MCAN1_RX_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                    (0x00400000U)
#define CSL_IOMUX_MCAN1_RX_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                   (0x00000016U)
#define CSL_IOMUX_MCAN1_RX_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL                (0x00000000U)
#define CSL_IOMUX_MCAN1_RX_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                     (0x00000001U)

#define CSL_IOMUX_MCAN1_RX_CFG_REG_HSMODE_MASK                                 (0x40000000U)
#define CSL_IOMUX_MCAN1_RX_CFG_REG_HSMODE_SHIFT                                (0x0000001EU)
#define CSL_IOMUX_MCAN1_RX_CFG_REG_HSMODE_RESETVAL                             (0x00000000U)
#define CSL_IOMUX_MCAN1_RX_CFG_REG_HSMODE_MAX                                  (0x00000001U)

#define CSL_IOMUX_MCAN1_RX_CFG_REG_HSMASTER_MASK                               (0x80000000U)
#define CSL_IOMUX_MCAN1_RX_CFG_REG_HSMASTER_SHIFT                              (0x0000001FU)
#define CSL_IOMUX_MCAN1_RX_CFG_REG_HSMASTER_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_MCAN1_RX_CFG_REG_HSMASTER_MAX                                (0x00000001U)

#define CSL_IOMUX_MCAN1_RX_CFG_REG_RESETVAL                                    (0x000005F7U)

/* MCAN1_TX_CFG_REG */

#define CSL_IOMUX_MCAN1_TX_CFG_REG_FUNC_SEL_MASK                               (0x0000000FU)
#define CSL_IOMUX_MCAN1_TX_CFG_REG_FUNC_SEL_SHIFT                              (0x00000000U)
#define CSL_IOMUX_MCAN1_TX_CFG_REG_FUNC_SEL_RESETVAL                           (0x00000007U)
#define CSL_IOMUX_MCAN1_TX_CFG_REG_FUNC_SEL_MAX                                (0x0000000FU)

#define CSL_IOMUX_MCAN1_TX_CFG_REG_IE_OVERRIDE_CTRL_MASK                       (0x00000010U)
#define CSL_IOMUX_MCAN1_TX_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                      (0x00000004U)
#define CSL_IOMUX_MCAN1_TX_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_MCAN1_TX_CFG_REG_IE_OVERRIDE_CTRL_MAX                        (0x00000001U)

#define CSL_IOMUX_MCAN1_TX_CFG_REG_IE_OVERRIDE_MASK                            (0x00000020U)
#define CSL_IOMUX_MCAN1_TX_CFG_REG_IE_OVERRIDE_SHIFT                           (0x00000005U)
#define CSL_IOMUX_MCAN1_TX_CFG_REG_IE_OVERRIDE_RESETVAL                        (0x00000001U)
#define CSL_IOMUX_MCAN1_TX_CFG_REG_IE_OVERRIDE_MAX                             (0x00000001U)

#define CSL_IOMUX_MCAN1_TX_CFG_REG_OE_OVERRIDE_CTRL_MASK                       (0x00000040U)
#define CSL_IOMUX_MCAN1_TX_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                      (0x00000006U)
#define CSL_IOMUX_MCAN1_TX_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_MCAN1_TX_CFG_REG_OE_OVERRIDE_CTRL_MAX                        (0x00000001U)

#define CSL_IOMUX_MCAN1_TX_CFG_REG_OE_OVERRIDE_MASK                            (0x00000080U)
#define CSL_IOMUX_MCAN1_TX_CFG_REG_OE_OVERRIDE_SHIFT                           (0x00000007U)
#define CSL_IOMUX_MCAN1_TX_CFG_REG_OE_OVERRIDE_RESETVAL                        (0x00000001U)
#define CSL_IOMUX_MCAN1_TX_CFG_REG_OE_OVERRIDE_MAX                             (0x00000001U)

#define CSL_IOMUX_MCAN1_TX_CFG_REG_PI_MASK                                     (0x00000100U)
#define CSL_IOMUX_MCAN1_TX_CFG_REG_PI_SHIFT                                    (0x00000008U)
#define CSL_IOMUX_MCAN1_TX_CFG_REG_PI_RESETVAL                                 (0x00000001U)
#define CSL_IOMUX_MCAN1_TX_CFG_REG_PI_MAX                                      (0x00000001U)

#define CSL_IOMUX_MCAN1_TX_CFG_REG_PUPDSEL_MASK                                (0x00000200U)
#define CSL_IOMUX_MCAN1_TX_CFG_REG_PUPDSEL_SHIFT                               (0x00000009U)
#define CSL_IOMUX_MCAN1_TX_CFG_REG_PUPDSEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_MCAN1_TX_CFG_REG_PUPDSEL_MAX                                 (0x00000001U)

#define CSL_IOMUX_MCAN1_TX_CFG_REG_SC1_MASK                                    (0x00000400U)
#define CSL_IOMUX_MCAN1_TX_CFG_REG_SC1_SHIFT                                   (0x0000000AU)
#define CSL_IOMUX_MCAN1_TX_CFG_REG_SC1_RESETVAL                                (0x00000001U)
#define CSL_IOMUX_MCAN1_TX_CFG_REG_SC1_MAX                                     (0x00000001U)

#define CSL_IOMUX_MCAN1_TX_CFG_REG_GPIO_SEL_MASK                               (0x00030000U)
#define CSL_IOMUX_MCAN1_TX_CFG_REG_GPIO_SEL_SHIFT                              (0x00000010U)
#define CSL_IOMUX_MCAN1_TX_CFG_REG_GPIO_SEL_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_MCAN1_TX_CFG_REG_GPIO_SEL_MAX                                (0x00000003U)

#define CSL_IOMUX_MCAN1_TX_CFG_REG_QUAL_SEL_MASK                               (0x000C0000U)
#define CSL_IOMUX_MCAN1_TX_CFG_REG_QUAL_SEL_SHIFT                              (0x00000012U)
#define CSL_IOMUX_MCAN1_TX_CFG_REG_QUAL_SEL_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_MCAN1_TX_CFG_REG_QUAL_SEL_MAX                                (0x00000003U)

#define CSL_IOMUX_MCAN1_TX_CFG_REG_INP_INV_SEL_MASK                            (0x00100000U)
#define CSL_IOMUX_MCAN1_TX_CFG_REG_INP_INV_SEL_SHIFT                           (0x00000014U)
#define CSL_IOMUX_MCAN1_TX_CFG_REG_INP_INV_SEL_RESETVAL                        (0x00000000U)
#define CSL_IOMUX_MCAN1_TX_CFG_REG_INP_INV_SEL_MAX                             (0x00000001U)

#define CSL_IOMUX_MCAN1_TX_CFG_REG_ICSSM_GPIO_SEL_MASK                         (0x00200000U)
#define CSL_IOMUX_MCAN1_TX_CFG_REG_ICSSM_GPIO_SEL_SHIFT                        (0x00000015U)
#define CSL_IOMUX_MCAN1_TX_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_MCAN1_TX_CFG_REG_ICSSM_GPIO_SEL_MAX                          (0x00000001U)

#define CSL_IOMUX_MCAN1_TX_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                    (0x00400000U)
#define CSL_IOMUX_MCAN1_TX_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                   (0x00000016U)
#define CSL_IOMUX_MCAN1_TX_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL                (0x00000000U)
#define CSL_IOMUX_MCAN1_TX_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                     (0x00000001U)

#define CSL_IOMUX_MCAN1_TX_CFG_REG_HSMODE_MASK                                 (0x40000000U)
#define CSL_IOMUX_MCAN1_TX_CFG_REG_HSMODE_SHIFT                                (0x0000001EU)
#define CSL_IOMUX_MCAN1_TX_CFG_REG_HSMODE_RESETVAL                             (0x00000000U)
#define CSL_IOMUX_MCAN1_TX_CFG_REG_HSMODE_MAX                                  (0x00000001U)

#define CSL_IOMUX_MCAN1_TX_CFG_REG_HSMASTER_MASK                               (0x80000000U)
#define CSL_IOMUX_MCAN1_TX_CFG_REG_HSMASTER_SHIFT                              (0x0000001FU)
#define CSL_IOMUX_MCAN1_TX_CFG_REG_HSMASTER_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_MCAN1_TX_CFG_REG_HSMASTER_MAX                                (0x00000001U)

#define CSL_IOMUX_MCAN1_TX_CFG_REG_RESETVAL                                    (0x000005F7U)

/* SPI0_CS0_CFG_REG */

#define CSL_IOMUX_SPI0_CS0_CFG_REG_FUNC_SEL_MASK                               (0x0000000FU)
#define CSL_IOMUX_SPI0_CS0_CFG_REG_FUNC_SEL_SHIFT                              (0x00000000U)
#define CSL_IOMUX_SPI0_CS0_CFG_REG_FUNC_SEL_RESETVAL                           (0x00000007U)
#define CSL_IOMUX_SPI0_CS0_CFG_REG_FUNC_SEL_MAX                                (0x0000000FU)

#define CSL_IOMUX_SPI0_CS0_CFG_REG_IE_OVERRIDE_CTRL_MASK                       (0x00000010U)
#define CSL_IOMUX_SPI0_CS0_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                      (0x00000004U)
#define CSL_IOMUX_SPI0_CS0_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_SPI0_CS0_CFG_REG_IE_OVERRIDE_CTRL_MAX                        (0x00000001U)

#define CSL_IOMUX_SPI0_CS0_CFG_REG_IE_OVERRIDE_MASK                            (0x00000020U)
#define CSL_IOMUX_SPI0_CS0_CFG_REG_IE_OVERRIDE_SHIFT                           (0x00000005U)
#define CSL_IOMUX_SPI0_CS0_CFG_REG_IE_OVERRIDE_RESETVAL                        (0x00000001U)
#define CSL_IOMUX_SPI0_CS0_CFG_REG_IE_OVERRIDE_MAX                             (0x00000001U)

#define CSL_IOMUX_SPI0_CS0_CFG_REG_OE_OVERRIDE_CTRL_MASK                       (0x00000040U)
#define CSL_IOMUX_SPI0_CS0_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                      (0x00000006U)
#define CSL_IOMUX_SPI0_CS0_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_SPI0_CS0_CFG_REG_OE_OVERRIDE_CTRL_MAX                        (0x00000001U)

#define CSL_IOMUX_SPI0_CS0_CFG_REG_OE_OVERRIDE_MASK                            (0x00000080U)
#define CSL_IOMUX_SPI0_CS0_CFG_REG_OE_OVERRIDE_SHIFT                           (0x00000007U)
#define CSL_IOMUX_SPI0_CS0_CFG_REG_OE_OVERRIDE_RESETVAL                        (0x00000001U)
#define CSL_IOMUX_SPI0_CS0_CFG_REG_OE_OVERRIDE_MAX                             (0x00000001U)

#define CSL_IOMUX_SPI0_CS0_CFG_REG_PI_MASK                                     (0x00000100U)
#define CSL_IOMUX_SPI0_CS0_CFG_REG_PI_SHIFT                                    (0x00000008U)
#define CSL_IOMUX_SPI0_CS0_CFG_REG_PI_RESETVAL                                 (0x00000001U)
#define CSL_IOMUX_SPI0_CS0_CFG_REG_PI_MAX                                      (0x00000001U)

#define CSL_IOMUX_SPI0_CS0_CFG_REG_PUPDSEL_MASK                                (0x00000200U)
#define CSL_IOMUX_SPI0_CS0_CFG_REG_PUPDSEL_SHIFT                               (0x00000009U)
#define CSL_IOMUX_SPI0_CS0_CFG_REG_PUPDSEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_SPI0_CS0_CFG_REG_PUPDSEL_MAX                                 (0x00000001U)

#define CSL_IOMUX_SPI0_CS0_CFG_REG_SC1_MASK                                    (0x00000400U)
#define CSL_IOMUX_SPI0_CS0_CFG_REG_SC1_SHIFT                                   (0x0000000AU)
#define CSL_IOMUX_SPI0_CS0_CFG_REG_SC1_RESETVAL                                (0x00000001U)
#define CSL_IOMUX_SPI0_CS0_CFG_REG_SC1_MAX                                     (0x00000001U)

#define CSL_IOMUX_SPI0_CS0_CFG_REG_GPIO_SEL_MASK                               (0x00030000U)
#define CSL_IOMUX_SPI0_CS0_CFG_REG_GPIO_SEL_SHIFT                              (0x00000010U)
#define CSL_IOMUX_SPI0_CS0_CFG_REG_GPIO_SEL_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_SPI0_CS0_CFG_REG_GPIO_SEL_MAX                                (0x00000003U)

#define CSL_IOMUX_SPI0_CS0_CFG_REG_QUAL_SEL_MASK                               (0x000C0000U)
#define CSL_IOMUX_SPI0_CS0_CFG_REG_QUAL_SEL_SHIFT                              (0x00000012U)
#define CSL_IOMUX_SPI0_CS0_CFG_REG_QUAL_SEL_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_SPI0_CS0_CFG_REG_QUAL_SEL_MAX                                (0x00000003U)

#define CSL_IOMUX_SPI0_CS0_CFG_REG_INP_INV_SEL_MASK                            (0x00100000U)
#define CSL_IOMUX_SPI0_CS0_CFG_REG_INP_INV_SEL_SHIFT                           (0x00000014U)
#define CSL_IOMUX_SPI0_CS0_CFG_REG_INP_INV_SEL_RESETVAL                        (0x00000000U)
#define CSL_IOMUX_SPI0_CS0_CFG_REG_INP_INV_SEL_MAX                             (0x00000001U)

#define CSL_IOMUX_SPI0_CS0_CFG_REG_ICSSM_GPIO_SEL_MASK                         (0x00200000U)
#define CSL_IOMUX_SPI0_CS0_CFG_REG_ICSSM_GPIO_SEL_SHIFT                        (0x00000015U)
#define CSL_IOMUX_SPI0_CS0_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_SPI0_CS0_CFG_REG_ICSSM_GPIO_SEL_MAX                          (0x00000001U)

#define CSL_IOMUX_SPI0_CS0_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                    (0x00400000U)
#define CSL_IOMUX_SPI0_CS0_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                   (0x00000016U)
#define CSL_IOMUX_SPI0_CS0_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL                (0x00000000U)
#define CSL_IOMUX_SPI0_CS0_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                     (0x00000001U)

#define CSL_IOMUX_SPI0_CS0_CFG_REG_HSMODE_MASK                                 (0x40000000U)
#define CSL_IOMUX_SPI0_CS0_CFG_REG_HSMODE_SHIFT                                (0x0000001EU)
#define CSL_IOMUX_SPI0_CS0_CFG_REG_HSMODE_RESETVAL                             (0x00000000U)
#define CSL_IOMUX_SPI0_CS0_CFG_REG_HSMODE_MAX                                  (0x00000001U)

#define CSL_IOMUX_SPI0_CS0_CFG_REG_HSMASTER_MASK                               (0x80000000U)
#define CSL_IOMUX_SPI0_CS0_CFG_REG_HSMASTER_SHIFT                              (0x0000001FU)
#define CSL_IOMUX_SPI0_CS0_CFG_REG_HSMASTER_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_SPI0_CS0_CFG_REG_HSMASTER_MAX                                (0x00000001U)

#define CSL_IOMUX_SPI0_CS0_CFG_REG_RESETVAL                                    (0x000005F7U)

/* SPI0_CLK_CFG_REG */

#define CSL_IOMUX_SPI0_CLK_CFG_REG_FUNC_SEL_MASK                               (0x0000000FU)
#define CSL_IOMUX_SPI0_CLK_CFG_REG_FUNC_SEL_SHIFT                              (0x00000000U)
#define CSL_IOMUX_SPI0_CLK_CFG_REG_FUNC_SEL_RESETVAL                           (0x00000007U)
#define CSL_IOMUX_SPI0_CLK_CFG_REG_FUNC_SEL_MAX                                (0x0000000FU)

#define CSL_IOMUX_SPI0_CLK_CFG_REG_IE_OVERRIDE_CTRL_MASK                       (0x00000010U)
#define CSL_IOMUX_SPI0_CLK_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                      (0x00000004U)
#define CSL_IOMUX_SPI0_CLK_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_SPI0_CLK_CFG_REG_IE_OVERRIDE_CTRL_MAX                        (0x00000001U)

#define CSL_IOMUX_SPI0_CLK_CFG_REG_IE_OVERRIDE_MASK                            (0x00000020U)
#define CSL_IOMUX_SPI0_CLK_CFG_REG_IE_OVERRIDE_SHIFT                           (0x00000005U)
#define CSL_IOMUX_SPI0_CLK_CFG_REG_IE_OVERRIDE_RESETVAL                        (0x00000000U)
#define CSL_IOMUX_SPI0_CLK_CFG_REG_IE_OVERRIDE_MAX                             (0x00000001U)

#define CSL_IOMUX_SPI0_CLK_CFG_REG_OE_OVERRIDE_CTRL_MASK                       (0x00000040U)
#define CSL_IOMUX_SPI0_CLK_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                      (0x00000006U)
#define CSL_IOMUX_SPI0_CLK_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_SPI0_CLK_CFG_REG_OE_OVERRIDE_CTRL_MAX                        (0x00000001U)

#define CSL_IOMUX_SPI0_CLK_CFG_REG_OE_OVERRIDE_MASK                            (0x00000080U)
#define CSL_IOMUX_SPI0_CLK_CFG_REG_OE_OVERRIDE_SHIFT                           (0x00000007U)
#define CSL_IOMUX_SPI0_CLK_CFG_REG_OE_OVERRIDE_RESETVAL                        (0x00000001U)
#define CSL_IOMUX_SPI0_CLK_CFG_REG_OE_OVERRIDE_MAX                             (0x00000001U)

#define CSL_IOMUX_SPI0_CLK_CFG_REG_PI_MASK                                     (0x00000100U)
#define CSL_IOMUX_SPI0_CLK_CFG_REG_PI_SHIFT                                    (0x00000008U)
#define CSL_IOMUX_SPI0_CLK_CFG_REG_PI_RESETVAL                                 (0x00000001U)
#define CSL_IOMUX_SPI0_CLK_CFG_REG_PI_MAX                                      (0x00000001U)

#define CSL_IOMUX_SPI0_CLK_CFG_REG_PUPDSEL_MASK                                (0x00000200U)
#define CSL_IOMUX_SPI0_CLK_CFG_REG_PUPDSEL_SHIFT                               (0x00000009U)
#define CSL_IOMUX_SPI0_CLK_CFG_REG_PUPDSEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_SPI0_CLK_CFG_REG_PUPDSEL_MAX                                 (0x00000001U)

#define CSL_IOMUX_SPI0_CLK_CFG_REG_SC1_MASK                                    (0x00000400U)
#define CSL_IOMUX_SPI0_CLK_CFG_REG_SC1_SHIFT                                   (0x0000000AU)
#define CSL_IOMUX_SPI0_CLK_CFG_REG_SC1_RESETVAL                                (0x00000001U)
#define CSL_IOMUX_SPI0_CLK_CFG_REG_SC1_MAX                                     (0x00000001U)

#define CSL_IOMUX_SPI0_CLK_CFG_REG_GPIO_SEL_MASK                               (0x00030000U)
#define CSL_IOMUX_SPI0_CLK_CFG_REG_GPIO_SEL_SHIFT                              (0x00000010U)
#define CSL_IOMUX_SPI0_CLK_CFG_REG_GPIO_SEL_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_SPI0_CLK_CFG_REG_GPIO_SEL_MAX                                (0x00000003U)

#define CSL_IOMUX_SPI0_CLK_CFG_REG_QUAL_SEL_MASK                               (0x000C0000U)
#define CSL_IOMUX_SPI0_CLK_CFG_REG_QUAL_SEL_SHIFT                              (0x00000012U)
#define CSL_IOMUX_SPI0_CLK_CFG_REG_QUAL_SEL_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_SPI0_CLK_CFG_REG_QUAL_SEL_MAX                                (0x00000003U)

#define CSL_IOMUX_SPI0_CLK_CFG_REG_INP_INV_SEL_MASK                            (0x00100000U)
#define CSL_IOMUX_SPI0_CLK_CFG_REG_INP_INV_SEL_SHIFT                           (0x00000014U)
#define CSL_IOMUX_SPI0_CLK_CFG_REG_INP_INV_SEL_RESETVAL                        (0x00000000U)
#define CSL_IOMUX_SPI0_CLK_CFG_REG_INP_INV_SEL_MAX                             (0x00000001U)

#define CSL_IOMUX_SPI0_CLK_CFG_REG_ICSSM_GPIO_SEL_MASK                         (0x00200000U)
#define CSL_IOMUX_SPI0_CLK_CFG_REG_ICSSM_GPIO_SEL_SHIFT                        (0x00000015U)
#define CSL_IOMUX_SPI0_CLK_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_SPI0_CLK_CFG_REG_ICSSM_GPIO_SEL_MAX                          (0x00000001U)

#define CSL_IOMUX_SPI0_CLK_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                    (0x00400000U)
#define CSL_IOMUX_SPI0_CLK_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                   (0x00000016U)
#define CSL_IOMUX_SPI0_CLK_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL                (0x00000000U)
#define CSL_IOMUX_SPI0_CLK_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                     (0x00000001U)

#define CSL_IOMUX_SPI0_CLK_CFG_REG_HSMODE_MASK                                 (0x40000000U)
#define CSL_IOMUX_SPI0_CLK_CFG_REG_HSMODE_SHIFT                                (0x0000001EU)
#define CSL_IOMUX_SPI0_CLK_CFG_REG_HSMODE_RESETVAL                             (0x00000000U)
#define CSL_IOMUX_SPI0_CLK_CFG_REG_HSMODE_MAX                                  (0x00000001U)

#define CSL_IOMUX_SPI0_CLK_CFG_REG_HSMASTER_MASK                               (0x80000000U)
#define CSL_IOMUX_SPI0_CLK_CFG_REG_HSMASTER_SHIFT                              (0x0000001FU)
#define CSL_IOMUX_SPI0_CLK_CFG_REG_HSMASTER_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_SPI0_CLK_CFG_REG_HSMASTER_MAX                                (0x00000001U)

#define CSL_IOMUX_SPI0_CLK_CFG_REG_RESETVAL                                    (0x000005D7U)

/* SPI0_D0_CFG_REG */

#define CSL_IOMUX_SPI0_D0_CFG_REG_FUNC_SEL_MASK                                (0x0000000FU)
#define CSL_IOMUX_SPI0_D0_CFG_REG_FUNC_SEL_SHIFT                               (0x00000000U)
#define CSL_IOMUX_SPI0_D0_CFG_REG_FUNC_SEL_RESETVAL                            (0x00000007U)
#define CSL_IOMUX_SPI0_D0_CFG_REG_FUNC_SEL_MAX                                 (0x0000000FU)

#define CSL_IOMUX_SPI0_D0_CFG_REG_IE_OVERRIDE_CTRL_MASK                        (0x00000010U)
#define CSL_IOMUX_SPI0_D0_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                       (0x00000004U)
#define CSL_IOMUX_SPI0_D0_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_SPI0_D0_CFG_REG_IE_OVERRIDE_CTRL_MAX                         (0x00000001U)

#define CSL_IOMUX_SPI0_D0_CFG_REG_IE_OVERRIDE_MASK                             (0x00000020U)
#define CSL_IOMUX_SPI0_D0_CFG_REG_IE_OVERRIDE_SHIFT                            (0x00000005U)
#define CSL_IOMUX_SPI0_D0_CFG_REG_IE_OVERRIDE_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_SPI0_D0_CFG_REG_IE_OVERRIDE_MAX                              (0x00000001U)

#define CSL_IOMUX_SPI0_D0_CFG_REG_OE_OVERRIDE_CTRL_MASK                        (0x00000040U)
#define CSL_IOMUX_SPI0_D0_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                       (0x00000006U)
#define CSL_IOMUX_SPI0_D0_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_SPI0_D0_CFG_REG_OE_OVERRIDE_CTRL_MAX                         (0x00000001U)

#define CSL_IOMUX_SPI0_D0_CFG_REG_OE_OVERRIDE_MASK                             (0x00000080U)
#define CSL_IOMUX_SPI0_D0_CFG_REG_OE_OVERRIDE_SHIFT                            (0x00000007U)
#define CSL_IOMUX_SPI0_D0_CFG_REG_OE_OVERRIDE_RESETVAL                         (0x00000001U)
#define CSL_IOMUX_SPI0_D0_CFG_REG_OE_OVERRIDE_MAX                              (0x00000001U)

#define CSL_IOMUX_SPI0_D0_CFG_REG_PI_MASK                                      (0x00000100U)
#define CSL_IOMUX_SPI0_D0_CFG_REG_PI_SHIFT                                     (0x00000008U)
#define CSL_IOMUX_SPI0_D0_CFG_REG_PI_RESETVAL                                  (0x00000001U)
#define CSL_IOMUX_SPI0_D0_CFG_REG_PI_MAX                                       (0x00000001U)

#define CSL_IOMUX_SPI0_D0_CFG_REG_PUPDSEL_MASK                                 (0x00000200U)
#define CSL_IOMUX_SPI0_D0_CFG_REG_PUPDSEL_SHIFT                                (0x00000009U)
#define CSL_IOMUX_SPI0_D0_CFG_REG_PUPDSEL_RESETVAL                             (0x00000000U)
#define CSL_IOMUX_SPI0_D0_CFG_REG_PUPDSEL_MAX                                  (0x00000001U)

#define CSL_IOMUX_SPI0_D0_CFG_REG_SC1_MASK                                     (0x00000400U)
#define CSL_IOMUX_SPI0_D0_CFG_REG_SC1_SHIFT                                    (0x0000000AU)
#define CSL_IOMUX_SPI0_D0_CFG_REG_SC1_RESETVAL                                 (0x00000001U)
#define CSL_IOMUX_SPI0_D0_CFG_REG_SC1_MAX                                      (0x00000001U)

#define CSL_IOMUX_SPI0_D0_CFG_REG_GPIO_SEL_MASK                                (0x00030000U)
#define CSL_IOMUX_SPI0_D0_CFG_REG_GPIO_SEL_SHIFT                               (0x00000010U)
#define CSL_IOMUX_SPI0_D0_CFG_REG_GPIO_SEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_SPI0_D0_CFG_REG_GPIO_SEL_MAX                                 (0x00000003U)

#define CSL_IOMUX_SPI0_D0_CFG_REG_QUAL_SEL_MASK                                (0x000C0000U)
#define CSL_IOMUX_SPI0_D0_CFG_REG_QUAL_SEL_SHIFT                               (0x00000012U)
#define CSL_IOMUX_SPI0_D0_CFG_REG_QUAL_SEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_SPI0_D0_CFG_REG_QUAL_SEL_MAX                                 (0x00000003U)

#define CSL_IOMUX_SPI0_D0_CFG_REG_INP_INV_SEL_MASK                             (0x00100000U)
#define CSL_IOMUX_SPI0_D0_CFG_REG_INP_INV_SEL_SHIFT                            (0x00000014U)
#define CSL_IOMUX_SPI0_D0_CFG_REG_INP_INV_SEL_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_SPI0_D0_CFG_REG_INP_INV_SEL_MAX                              (0x00000001U)

#define CSL_IOMUX_SPI0_D0_CFG_REG_ICSSM_GPIO_SEL_MASK                          (0x00200000U)
#define CSL_IOMUX_SPI0_D0_CFG_REG_ICSSM_GPIO_SEL_SHIFT                         (0x00000015U)
#define CSL_IOMUX_SPI0_D0_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_SPI0_D0_CFG_REG_ICSSM_GPIO_SEL_MAX                           (0x00000001U)

#define CSL_IOMUX_SPI0_D0_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                     (0x00400000U)
#define CSL_IOMUX_SPI0_D0_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                    (0x00000016U)
#define CSL_IOMUX_SPI0_D0_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL                 (0x00000000U)
#define CSL_IOMUX_SPI0_D0_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                      (0x00000001U)

#define CSL_IOMUX_SPI0_D0_CFG_REG_HSMODE_MASK                                  (0x40000000U)
#define CSL_IOMUX_SPI0_D0_CFG_REG_HSMODE_SHIFT                                 (0x0000001EU)
#define CSL_IOMUX_SPI0_D0_CFG_REG_HSMODE_RESETVAL                              (0x00000000U)
#define CSL_IOMUX_SPI0_D0_CFG_REG_HSMODE_MAX                                   (0x00000001U)

#define CSL_IOMUX_SPI0_D0_CFG_REG_HSMASTER_MASK                                (0x80000000U)
#define CSL_IOMUX_SPI0_D0_CFG_REG_HSMASTER_SHIFT                               (0x0000001FU)
#define CSL_IOMUX_SPI0_D0_CFG_REG_HSMASTER_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_SPI0_D0_CFG_REG_HSMASTER_MAX                                 (0x00000001U)

#define CSL_IOMUX_SPI0_D0_CFG_REG_RESETVAL                                     (0x000005D7U)

/* SPI0_D1_CFG_REG */

#define CSL_IOMUX_SPI0_D1_CFG_REG_FUNC_SEL_MASK                                (0x0000000FU)
#define CSL_IOMUX_SPI0_D1_CFG_REG_FUNC_SEL_SHIFT                               (0x00000000U)
#define CSL_IOMUX_SPI0_D1_CFG_REG_FUNC_SEL_RESETVAL                            (0x00000007U)
#define CSL_IOMUX_SPI0_D1_CFG_REG_FUNC_SEL_MAX                                 (0x0000000FU)

#define CSL_IOMUX_SPI0_D1_CFG_REG_IE_OVERRIDE_CTRL_MASK                        (0x00000010U)
#define CSL_IOMUX_SPI0_D1_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                       (0x00000004U)
#define CSL_IOMUX_SPI0_D1_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_SPI0_D1_CFG_REG_IE_OVERRIDE_CTRL_MAX                         (0x00000001U)

#define CSL_IOMUX_SPI0_D1_CFG_REG_IE_OVERRIDE_MASK                             (0x00000020U)
#define CSL_IOMUX_SPI0_D1_CFG_REG_IE_OVERRIDE_SHIFT                            (0x00000005U)
#define CSL_IOMUX_SPI0_D1_CFG_REG_IE_OVERRIDE_RESETVAL                         (0x00000001U)
#define CSL_IOMUX_SPI0_D1_CFG_REG_IE_OVERRIDE_MAX                              (0x00000001U)

#define CSL_IOMUX_SPI0_D1_CFG_REG_OE_OVERRIDE_CTRL_MASK                        (0x00000040U)
#define CSL_IOMUX_SPI0_D1_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                       (0x00000006U)
#define CSL_IOMUX_SPI0_D1_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_SPI0_D1_CFG_REG_OE_OVERRIDE_CTRL_MAX                         (0x00000001U)

#define CSL_IOMUX_SPI0_D1_CFG_REG_OE_OVERRIDE_MASK                             (0x00000080U)
#define CSL_IOMUX_SPI0_D1_CFG_REG_OE_OVERRIDE_SHIFT                            (0x00000007U)
#define CSL_IOMUX_SPI0_D1_CFG_REG_OE_OVERRIDE_RESETVAL                         (0x00000001U)
#define CSL_IOMUX_SPI0_D1_CFG_REG_OE_OVERRIDE_MAX                              (0x00000001U)

#define CSL_IOMUX_SPI0_D1_CFG_REG_PI_MASK                                      (0x00000100U)
#define CSL_IOMUX_SPI0_D1_CFG_REG_PI_SHIFT                                     (0x00000008U)
#define CSL_IOMUX_SPI0_D1_CFG_REG_PI_RESETVAL                                  (0x00000001U)
#define CSL_IOMUX_SPI0_D1_CFG_REG_PI_MAX                                       (0x00000001U)

#define CSL_IOMUX_SPI0_D1_CFG_REG_PUPDSEL_MASK                                 (0x00000200U)
#define CSL_IOMUX_SPI0_D1_CFG_REG_PUPDSEL_SHIFT                                (0x00000009U)
#define CSL_IOMUX_SPI0_D1_CFG_REG_PUPDSEL_RESETVAL                             (0x00000000U)
#define CSL_IOMUX_SPI0_D1_CFG_REG_PUPDSEL_MAX                                  (0x00000001U)

#define CSL_IOMUX_SPI0_D1_CFG_REG_SC1_MASK                                     (0x00000400U)
#define CSL_IOMUX_SPI0_D1_CFG_REG_SC1_SHIFT                                    (0x0000000AU)
#define CSL_IOMUX_SPI0_D1_CFG_REG_SC1_RESETVAL                                 (0x00000001U)
#define CSL_IOMUX_SPI0_D1_CFG_REG_SC1_MAX                                      (0x00000001U)

#define CSL_IOMUX_SPI0_D1_CFG_REG_GPIO_SEL_MASK                                (0x00030000U)
#define CSL_IOMUX_SPI0_D1_CFG_REG_GPIO_SEL_SHIFT                               (0x00000010U)
#define CSL_IOMUX_SPI0_D1_CFG_REG_GPIO_SEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_SPI0_D1_CFG_REG_GPIO_SEL_MAX                                 (0x00000003U)

#define CSL_IOMUX_SPI0_D1_CFG_REG_QUAL_SEL_MASK                                (0x000C0000U)
#define CSL_IOMUX_SPI0_D1_CFG_REG_QUAL_SEL_SHIFT                               (0x00000012U)
#define CSL_IOMUX_SPI0_D1_CFG_REG_QUAL_SEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_SPI0_D1_CFG_REG_QUAL_SEL_MAX                                 (0x00000003U)

#define CSL_IOMUX_SPI0_D1_CFG_REG_INP_INV_SEL_MASK                             (0x00100000U)
#define CSL_IOMUX_SPI0_D1_CFG_REG_INP_INV_SEL_SHIFT                            (0x00000014U)
#define CSL_IOMUX_SPI0_D1_CFG_REG_INP_INV_SEL_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_SPI0_D1_CFG_REG_INP_INV_SEL_MAX                              (0x00000001U)

#define CSL_IOMUX_SPI0_D1_CFG_REG_ICSSM_GPIO_SEL_MASK                          (0x00200000U)
#define CSL_IOMUX_SPI0_D1_CFG_REG_ICSSM_GPIO_SEL_SHIFT                         (0x00000015U)
#define CSL_IOMUX_SPI0_D1_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_SPI0_D1_CFG_REG_ICSSM_GPIO_SEL_MAX                           (0x00000001U)

#define CSL_IOMUX_SPI0_D1_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                     (0x00400000U)
#define CSL_IOMUX_SPI0_D1_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                    (0x00000016U)
#define CSL_IOMUX_SPI0_D1_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL                 (0x00000000U)
#define CSL_IOMUX_SPI0_D1_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                      (0x00000001U)

#define CSL_IOMUX_SPI0_D1_CFG_REG_HSMODE_MASK                                  (0x40000000U)
#define CSL_IOMUX_SPI0_D1_CFG_REG_HSMODE_SHIFT                                 (0x0000001EU)
#define CSL_IOMUX_SPI0_D1_CFG_REG_HSMODE_RESETVAL                              (0x00000000U)
#define CSL_IOMUX_SPI0_D1_CFG_REG_HSMODE_MAX                                   (0x00000001U)

#define CSL_IOMUX_SPI0_D1_CFG_REG_HSMASTER_MASK                                (0x80000000U)
#define CSL_IOMUX_SPI0_D1_CFG_REG_HSMASTER_SHIFT                               (0x0000001FU)
#define CSL_IOMUX_SPI0_D1_CFG_REG_HSMASTER_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_SPI0_D1_CFG_REG_HSMASTER_MAX                                 (0x00000001U)

#define CSL_IOMUX_SPI0_D1_CFG_REG_RESETVAL                                     (0x000005F7U)

/* SPI1_CS0_CFG_REG */

#define CSL_IOMUX_SPI1_CS0_CFG_REG_FUNC_SEL_MASK                               (0x0000000FU)
#define CSL_IOMUX_SPI1_CS0_CFG_REG_FUNC_SEL_SHIFT                              (0x00000000U)
#define CSL_IOMUX_SPI1_CS0_CFG_REG_FUNC_SEL_RESETVAL                           (0x00000007U)
#define CSL_IOMUX_SPI1_CS0_CFG_REG_FUNC_SEL_MAX                                (0x0000000FU)

#define CSL_IOMUX_SPI1_CS0_CFG_REG_IE_OVERRIDE_CTRL_MASK                       (0x00000010U)
#define CSL_IOMUX_SPI1_CS0_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                      (0x00000004U)
#define CSL_IOMUX_SPI1_CS0_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_SPI1_CS0_CFG_REG_IE_OVERRIDE_CTRL_MAX                        (0x00000001U)

#define CSL_IOMUX_SPI1_CS0_CFG_REG_IE_OVERRIDE_MASK                            (0x00000020U)
#define CSL_IOMUX_SPI1_CS0_CFG_REG_IE_OVERRIDE_SHIFT                           (0x00000005U)
#define CSL_IOMUX_SPI1_CS0_CFG_REG_IE_OVERRIDE_RESETVAL                        (0x00000001U)
#define CSL_IOMUX_SPI1_CS0_CFG_REG_IE_OVERRIDE_MAX                             (0x00000001U)

#define CSL_IOMUX_SPI1_CS0_CFG_REG_OE_OVERRIDE_CTRL_MASK                       (0x00000040U)
#define CSL_IOMUX_SPI1_CS0_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                      (0x00000006U)
#define CSL_IOMUX_SPI1_CS0_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_SPI1_CS0_CFG_REG_OE_OVERRIDE_CTRL_MAX                        (0x00000001U)

#define CSL_IOMUX_SPI1_CS0_CFG_REG_OE_OVERRIDE_MASK                            (0x00000080U)
#define CSL_IOMUX_SPI1_CS0_CFG_REG_OE_OVERRIDE_SHIFT                           (0x00000007U)
#define CSL_IOMUX_SPI1_CS0_CFG_REG_OE_OVERRIDE_RESETVAL                        (0x00000001U)
#define CSL_IOMUX_SPI1_CS0_CFG_REG_OE_OVERRIDE_MAX                             (0x00000001U)

#define CSL_IOMUX_SPI1_CS0_CFG_REG_PI_MASK                                     (0x00000100U)
#define CSL_IOMUX_SPI1_CS0_CFG_REG_PI_SHIFT                                    (0x00000008U)
#define CSL_IOMUX_SPI1_CS0_CFG_REG_PI_RESETVAL                                 (0x00000001U)
#define CSL_IOMUX_SPI1_CS0_CFG_REG_PI_MAX                                      (0x00000001U)

#define CSL_IOMUX_SPI1_CS0_CFG_REG_PUPDSEL_MASK                                (0x00000200U)
#define CSL_IOMUX_SPI1_CS0_CFG_REG_PUPDSEL_SHIFT                               (0x00000009U)
#define CSL_IOMUX_SPI1_CS0_CFG_REG_PUPDSEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_SPI1_CS0_CFG_REG_PUPDSEL_MAX                                 (0x00000001U)

#define CSL_IOMUX_SPI1_CS0_CFG_REG_SC1_MASK                                    (0x00000400U)
#define CSL_IOMUX_SPI1_CS0_CFG_REG_SC1_SHIFT                                   (0x0000000AU)
#define CSL_IOMUX_SPI1_CS0_CFG_REG_SC1_RESETVAL                                (0x00000001U)
#define CSL_IOMUX_SPI1_CS0_CFG_REG_SC1_MAX                                     (0x00000001U)

#define CSL_IOMUX_SPI1_CS0_CFG_REG_GPIO_SEL_MASK                               (0x00030000U)
#define CSL_IOMUX_SPI1_CS0_CFG_REG_GPIO_SEL_SHIFT                              (0x00000010U)
#define CSL_IOMUX_SPI1_CS0_CFG_REG_GPIO_SEL_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_SPI1_CS0_CFG_REG_GPIO_SEL_MAX                                (0x00000003U)

#define CSL_IOMUX_SPI1_CS0_CFG_REG_QUAL_SEL_MASK                               (0x000C0000U)
#define CSL_IOMUX_SPI1_CS0_CFG_REG_QUAL_SEL_SHIFT                              (0x00000012U)
#define CSL_IOMUX_SPI1_CS0_CFG_REG_QUAL_SEL_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_SPI1_CS0_CFG_REG_QUAL_SEL_MAX                                (0x00000003U)

#define CSL_IOMUX_SPI1_CS0_CFG_REG_INP_INV_SEL_MASK                            (0x00100000U)
#define CSL_IOMUX_SPI1_CS0_CFG_REG_INP_INV_SEL_SHIFT                           (0x00000014U)
#define CSL_IOMUX_SPI1_CS0_CFG_REG_INP_INV_SEL_RESETVAL                        (0x00000000U)
#define CSL_IOMUX_SPI1_CS0_CFG_REG_INP_INV_SEL_MAX                             (0x00000001U)

#define CSL_IOMUX_SPI1_CS0_CFG_REG_ICSSM_GPIO_SEL_MASK                         (0x00200000U)
#define CSL_IOMUX_SPI1_CS0_CFG_REG_ICSSM_GPIO_SEL_SHIFT                        (0x00000015U)
#define CSL_IOMUX_SPI1_CS0_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_SPI1_CS0_CFG_REG_ICSSM_GPIO_SEL_MAX                          (0x00000001U)

#define CSL_IOMUX_SPI1_CS0_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                    (0x00400000U)
#define CSL_IOMUX_SPI1_CS0_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                   (0x00000016U)
#define CSL_IOMUX_SPI1_CS0_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL                (0x00000000U)
#define CSL_IOMUX_SPI1_CS0_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                     (0x00000001U)

#define CSL_IOMUX_SPI1_CS0_CFG_REG_HSMODE_MASK                                 (0x40000000U)
#define CSL_IOMUX_SPI1_CS0_CFG_REG_HSMODE_SHIFT                                (0x0000001EU)
#define CSL_IOMUX_SPI1_CS0_CFG_REG_HSMODE_RESETVAL                             (0x00000000U)
#define CSL_IOMUX_SPI1_CS0_CFG_REG_HSMODE_MAX                                  (0x00000001U)

#define CSL_IOMUX_SPI1_CS0_CFG_REG_HSMASTER_MASK                               (0x80000000U)
#define CSL_IOMUX_SPI1_CS0_CFG_REG_HSMASTER_SHIFT                              (0x0000001FU)
#define CSL_IOMUX_SPI1_CS0_CFG_REG_HSMASTER_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_SPI1_CS0_CFG_REG_HSMASTER_MAX                                (0x00000001U)

#define CSL_IOMUX_SPI1_CS0_CFG_REG_RESETVAL                                    (0x000005F7U)

/* SPI1_CLK_CFG_REG */

#define CSL_IOMUX_SPI1_CLK_CFG_REG_FUNC_SEL_MASK                               (0x0000000FU)
#define CSL_IOMUX_SPI1_CLK_CFG_REG_FUNC_SEL_SHIFT                              (0x00000000U)
#define CSL_IOMUX_SPI1_CLK_CFG_REG_FUNC_SEL_RESETVAL                           (0x00000007U)
#define CSL_IOMUX_SPI1_CLK_CFG_REG_FUNC_SEL_MAX                                (0x0000000FU)

#define CSL_IOMUX_SPI1_CLK_CFG_REG_IE_OVERRIDE_CTRL_MASK                       (0x00000010U)
#define CSL_IOMUX_SPI1_CLK_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                      (0x00000004U)
#define CSL_IOMUX_SPI1_CLK_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_SPI1_CLK_CFG_REG_IE_OVERRIDE_CTRL_MAX                        (0x00000001U)

#define CSL_IOMUX_SPI1_CLK_CFG_REG_IE_OVERRIDE_MASK                            (0x00000020U)
#define CSL_IOMUX_SPI1_CLK_CFG_REG_IE_OVERRIDE_SHIFT                           (0x00000005U)
#define CSL_IOMUX_SPI1_CLK_CFG_REG_IE_OVERRIDE_RESETVAL                        (0x00000001U)
#define CSL_IOMUX_SPI1_CLK_CFG_REG_IE_OVERRIDE_MAX                             (0x00000001U)

#define CSL_IOMUX_SPI1_CLK_CFG_REG_OE_OVERRIDE_CTRL_MASK                       (0x00000040U)
#define CSL_IOMUX_SPI1_CLK_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                      (0x00000006U)
#define CSL_IOMUX_SPI1_CLK_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_SPI1_CLK_CFG_REG_OE_OVERRIDE_CTRL_MAX                        (0x00000001U)

#define CSL_IOMUX_SPI1_CLK_CFG_REG_OE_OVERRIDE_MASK                            (0x00000080U)
#define CSL_IOMUX_SPI1_CLK_CFG_REG_OE_OVERRIDE_SHIFT                           (0x00000007U)
#define CSL_IOMUX_SPI1_CLK_CFG_REG_OE_OVERRIDE_RESETVAL                        (0x00000001U)
#define CSL_IOMUX_SPI1_CLK_CFG_REG_OE_OVERRIDE_MAX                             (0x00000001U)

#define CSL_IOMUX_SPI1_CLK_CFG_REG_PI_MASK                                     (0x00000100U)
#define CSL_IOMUX_SPI1_CLK_CFG_REG_PI_SHIFT                                    (0x00000008U)
#define CSL_IOMUX_SPI1_CLK_CFG_REG_PI_RESETVAL                                 (0x00000001U)
#define CSL_IOMUX_SPI1_CLK_CFG_REG_PI_MAX                                      (0x00000001U)

#define CSL_IOMUX_SPI1_CLK_CFG_REG_PUPDSEL_MASK                                (0x00000200U)
#define CSL_IOMUX_SPI1_CLK_CFG_REG_PUPDSEL_SHIFT                               (0x00000009U)
#define CSL_IOMUX_SPI1_CLK_CFG_REG_PUPDSEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_SPI1_CLK_CFG_REG_PUPDSEL_MAX                                 (0x00000001U)

#define CSL_IOMUX_SPI1_CLK_CFG_REG_SC1_MASK                                    (0x00000400U)
#define CSL_IOMUX_SPI1_CLK_CFG_REG_SC1_SHIFT                                   (0x0000000AU)
#define CSL_IOMUX_SPI1_CLK_CFG_REG_SC1_RESETVAL                                (0x00000001U)
#define CSL_IOMUX_SPI1_CLK_CFG_REG_SC1_MAX                                     (0x00000001U)

#define CSL_IOMUX_SPI1_CLK_CFG_REG_GPIO_SEL_MASK                               (0x00030000U)
#define CSL_IOMUX_SPI1_CLK_CFG_REG_GPIO_SEL_SHIFT                              (0x00000010U)
#define CSL_IOMUX_SPI1_CLK_CFG_REG_GPIO_SEL_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_SPI1_CLK_CFG_REG_GPIO_SEL_MAX                                (0x00000003U)

#define CSL_IOMUX_SPI1_CLK_CFG_REG_QUAL_SEL_MASK                               (0x000C0000U)
#define CSL_IOMUX_SPI1_CLK_CFG_REG_QUAL_SEL_SHIFT                              (0x00000012U)
#define CSL_IOMUX_SPI1_CLK_CFG_REG_QUAL_SEL_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_SPI1_CLK_CFG_REG_QUAL_SEL_MAX                                (0x00000003U)

#define CSL_IOMUX_SPI1_CLK_CFG_REG_INP_INV_SEL_MASK                            (0x00100000U)
#define CSL_IOMUX_SPI1_CLK_CFG_REG_INP_INV_SEL_SHIFT                           (0x00000014U)
#define CSL_IOMUX_SPI1_CLK_CFG_REG_INP_INV_SEL_RESETVAL                        (0x00000000U)
#define CSL_IOMUX_SPI1_CLK_CFG_REG_INP_INV_SEL_MAX                             (0x00000001U)

#define CSL_IOMUX_SPI1_CLK_CFG_REG_ICSSM_GPIO_SEL_MASK                         (0x00200000U)
#define CSL_IOMUX_SPI1_CLK_CFG_REG_ICSSM_GPIO_SEL_SHIFT                        (0x00000015U)
#define CSL_IOMUX_SPI1_CLK_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_SPI1_CLK_CFG_REG_ICSSM_GPIO_SEL_MAX                          (0x00000001U)

#define CSL_IOMUX_SPI1_CLK_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                    (0x00400000U)
#define CSL_IOMUX_SPI1_CLK_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                   (0x00000016U)
#define CSL_IOMUX_SPI1_CLK_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL                (0x00000000U)
#define CSL_IOMUX_SPI1_CLK_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                     (0x00000001U)

#define CSL_IOMUX_SPI1_CLK_CFG_REG_HSMODE_MASK                                 (0x40000000U)
#define CSL_IOMUX_SPI1_CLK_CFG_REG_HSMODE_SHIFT                                (0x0000001EU)
#define CSL_IOMUX_SPI1_CLK_CFG_REG_HSMODE_RESETVAL                             (0x00000000U)
#define CSL_IOMUX_SPI1_CLK_CFG_REG_HSMODE_MAX                                  (0x00000001U)

#define CSL_IOMUX_SPI1_CLK_CFG_REG_HSMASTER_MASK                               (0x80000000U)
#define CSL_IOMUX_SPI1_CLK_CFG_REG_HSMASTER_SHIFT                              (0x0000001FU)
#define CSL_IOMUX_SPI1_CLK_CFG_REG_HSMASTER_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_SPI1_CLK_CFG_REG_HSMASTER_MAX                                (0x00000001U)

#define CSL_IOMUX_SPI1_CLK_CFG_REG_RESETVAL                                    (0x000005F7U)

/* SPI1_D0_CFG_REG */

#define CSL_IOMUX_SPI1_D0_CFG_REG_FUNC_SEL_MASK                                (0x0000000FU)
#define CSL_IOMUX_SPI1_D0_CFG_REG_FUNC_SEL_SHIFT                               (0x00000000U)
#define CSL_IOMUX_SPI1_D0_CFG_REG_FUNC_SEL_RESETVAL                            (0x00000007U)
#define CSL_IOMUX_SPI1_D0_CFG_REG_FUNC_SEL_MAX                                 (0x0000000FU)

#define CSL_IOMUX_SPI1_D0_CFG_REG_IE_OVERRIDE_CTRL_MASK                        (0x00000010U)
#define CSL_IOMUX_SPI1_D0_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                       (0x00000004U)
#define CSL_IOMUX_SPI1_D0_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_SPI1_D0_CFG_REG_IE_OVERRIDE_CTRL_MAX                         (0x00000001U)

#define CSL_IOMUX_SPI1_D0_CFG_REG_IE_OVERRIDE_MASK                             (0x00000020U)
#define CSL_IOMUX_SPI1_D0_CFG_REG_IE_OVERRIDE_SHIFT                            (0x00000005U)
#define CSL_IOMUX_SPI1_D0_CFG_REG_IE_OVERRIDE_RESETVAL                         (0x00000001U)
#define CSL_IOMUX_SPI1_D0_CFG_REG_IE_OVERRIDE_MAX                              (0x00000001U)

#define CSL_IOMUX_SPI1_D0_CFG_REG_OE_OVERRIDE_CTRL_MASK                        (0x00000040U)
#define CSL_IOMUX_SPI1_D0_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                       (0x00000006U)
#define CSL_IOMUX_SPI1_D0_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_SPI1_D0_CFG_REG_OE_OVERRIDE_CTRL_MAX                         (0x00000001U)

#define CSL_IOMUX_SPI1_D0_CFG_REG_OE_OVERRIDE_MASK                             (0x00000080U)
#define CSL_IOMUX_SPI1_D0_CFG_REG_OE_OVERRIDE_SHIFT                            (0x00000007U)
#define CSL_IOMUX_SPI1_D0_CFG_REG_OE_OVERRIDE_RESETVAL                         (0x00000001U)
#define CSL_IOMUX_SPI1_D0_CFG_REG_OE_OVERRIDE_MAX                              (0x00000001U)

#define CSL_IOMUX_SPI1_D0_CFG_REG_PI_MASK                                      (0x00000100U)
#define CSL_IOMUX_SPI1_D0_CFG_REG_PI_SHIFT                                     (0x00000008U)
#define CSL_IOMUX_SPI1_D0_CFG_REG_PI_RESETVAL                                  (0x00000001U)
#define CSL_IOMUX_SPI1_D0_CFG_REG_PI_MAX                                       (0x00000001U)

#define CSL_IOMUX_SPI1_D0_CFG_REG_PUPDSEL_MASK                                 (0x00000200U)
#define CSL_IOMUX_SPI1_D0_CFG_REG_PUPDSEL_SHIFT                                (0x00000009U)
#define CSL_IOMUX_SPI1_D0_CFG_REG_PUPDSEL_RESETVAL                             (0x00000000U)
#define CSL_IOMUX_SPI1_D0_CFG_REG_PUPDSEL_MAX                                  (0x00000001U)

#define CSL_IOMUX_SPI1_D0_CFG_REG_SC1_MASK                                     (0x00000400U)
#define CSL_IOMUX_SPI1_D0_CFG_REG_SC1_SHIFT                                    (0x0000000AU)
#define CSL_IOMUX_SPI1_D0_CFG_REG_SC1_RESETVAL                                 (0x00000001U)
#define CSL_IOMUX_SPI1_D0_CFG_REG_SC1_MAX                                      (0x00000001U)

#define CSL_IOMUX_SPI1_D0_CFG_REG_GPIO_SEL_MASK                                (0x00030000U)
#define CSL_IOMUX_SPI1_D0_CFG_REG_GPIO_SEL_SHIFT                               (0x00000010U)
#define CSL_IOMUX_SPI1_D0_CFG_REG_GPIO_SEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_SPI1_D0_CFG_REG_GPIO_SEL_MAX                                 (0x00000003U)

#define CSL_IOMUX_SPI1_D0_CFG_REG_QUAL_SEL_MASK                                (0x000C0000U)
#define CSL_IOMUX_SPI1_D0_CFG_REG_QUAL_SEL_SHIFT                               (0x00000012U)
#define CSL_IOMUX_SPI1_D0_CFG_REG_QUAL_SEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_SPI1_D0_CFG_REG_QUAL_SEL_MAX                                 (0x00000003U)

#define CSL_IOMUX_SPI1_D0_CFG_REG_INP_INV_SEL_MASK                             (0x00100000U)
#define CSL_IOMUX_SPI1_D0_CFG_REG_INP_INV_SEL_SHIFT                            (0x00000014U)
#define CSL_IOMUX_SPI1_D0_CFG_REG_INP_INV_SEL_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_SPI1_D0_CFG_REG_INP_INV_SEL_MAX                              (0x00000001U)

#define CSL_IOMUX_SPI1_D0_CFG_REG_ICSSM_GPIO_SEL_MASK                          (0x00200000U)
#define CSL_IOMUX_SPI1_D0_CFG_REG_ICSSM_GPIO_SEL_SHIFT                         (0x00000015U)
#define CSL_IOMUX_SPI1_D0_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_SPI1_D0_CFG_REG_ICSSM_GPIO_SEL_MAX                           (0x00000001U)

#define CSL_IOMUX_SPI1_D0_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                     (0x00400000U)
#define CSL_IOMUX_SPI1_D0_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                    (0x00000016U)
#define CSL_IOMUX_SPI1_D0_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL                 (0x00000000U)
#define CSL_IOMUX_SPI1_D0_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                      (0x00000001U)

#define CSL_IOMUX_SPI1_D0_CFG_REG_HSMODE_MASK                                  (0x40000000U)
#define CSL_IOMUX_SPI1_D0_CFG_REG_HSMODE_SHIFT                                 (0x0000001EU)
#define CSL_IOMUX_SPI1_D0_CFG_REG_HSMODE_RESETVAL                              (0x00000000U)
#define CSL_IOMUX_SPI1_D0_CFG_REG_HSMODE_MAX                                   (0x00000001U)

#define CSL_IOMUX_SPI1_D0_CFG_REG_HSMASTER_MASK                                (0x80000000U)
#define CSL_IOMUX_SPI1_D0_CFG_REG_HSMASTER_SHIFT                               (0x0000001FU)
#define CSL_IOMUX_SPI1_D0_CFG_REG_HSMASTER_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_SPI1_D0_CFG_REG_HSMASTER_MAX                                 (0x00000001U)

#define CSL_IOMUX_SPI1_D0_CFG_REG_RESETVAL                                     (0x000005F7U)

/* SPI1_D1_CFG_REG */

#define CSL_IOMUX_SPI1_D1_CFG_REG_FUNC_SEL_MASK                                (0x0000000FU)
#define CSL_IOMUX_SPI1_D1_CFG_REG_FUNC_SEL_SHIFT                               (0x00000000U)
#define CSL_IOMUX_SPI1_D1_CFG_REG_FUNC_SEL_RESETVAL                            (0x00000007U)
#define CSL_IOMUX_SPI1_D1_CFG_REG_FUNC_SEL_MAX                                 (0x0000000FU)

#define CSL_IOMUX_SPI1_D1_CFG_REG_IE_OVERRIDE_CTRL_MASK                        (0x00000010U)
#define CSL_IOMUX_SPI1_D1_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                       (0x00000004U)
#define CSL_IOMUX_SPI1_D1_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_SPI1_D1_CFG_REG_IE_OVERRIDE_CTRL_MAX                         (0x00000001U)

#define CSL_IOMUX_SPI1_D1_CFG_REG_IE_OVERRIDE_MASK                             (0x00000020U)
#define CSL_IOMUX_SPI1_D1_CFG_REG_IE_OVERRIDE_SHIFT                            (0x00000005U)
#define CSL_IOMUX_SPI1_D1_CFG_REG_IE_OVERRIDE_RESETVAL                         (0x00000001U)
#define CSL_IOMUX_SPI1_D1_CFG_REG_IE_OVERRIDE_MAX                              (0x00000001U)

#define CSL_IOMUX_SPI1_D1_CFG_REG_OE_OVERRIDE_CTRL_MASK                        (0x00000040U)
#define CSL_IOMUX_SPI1_D1_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                       (0x00000006U)
#define CSL_IOMUX_SPI1_D1_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_SPI1_D1_CFG_REG_OE_OVERRIDE_CTRL_MAX                         (0x00000001U)

#define CSL_IOMUX_SPI1_D1_CFG_REG_OE_OVERRIDE_MASK                             (0x00000080U)
#define CSL_IOMUX_SPI1_D1_CFG_REG_OE_OVERRIDE_SHIFT                            (0x00000007U)
#define CSL_IOMUX_SPI1_D1_CFG_REG_OE_OVERRIDE_RESETVAL                         (0x00000001U)
#define CSL_IOMUX_SPI1_D1_CFG_REG_OE_OVERRIDE_MAX                              (0x00000001U)

#define CSL_IOMUX_SPI1_D1_CFG_REG_PI_MASK                                      (0x00000100U)
#define CSL_IOMUX_SPI1_D1_CFG_REG_PI_SHIFT                                     (0x00000008U)
#define CSL_IOMUX_SPI1_D1_CFG_REG_PI_RESETVAL                                  (0x00000001U)
#define CSL_IOMUX_SPI1_D1_CFG_REG_PI_MAX                                       (0x00000001U)

#define CSL_IOMUX_SPI1_D1_CFG_REG_PUPDSEL_MASK                                 (0x00000200U)
#define CSL_IOMUX_SPI1_D1_CFG_REG_PUPDSEL_SHIFT                                (0x00000009U)
#define CSL_IOMUX_SPI1_D1_CFG_REG_PUPDSEL_RESETVAL                             (0x00000000U)
#define CSL_IOMUX_SPI1_D1_CFG_REG_PUPDSEL_MAX                                  (0x00000001U)

#define CSL_IOMUX_SPI1_D1_CFG_REG_SC1_MASK                                     (0x00000400U)
#define CSL_IOMUX_SPI1_D1_CFG_REG_SC1_SHIFT                                    (0x0000000AU)
#define CSL_IOMUX_SPI1_D1_CFG_REG_SC1_RESETVAL                                 (0x00000001U)
#define CSL_IOMUX_SPI1_D1_CFG_REG_SC1_MAX                                      (0x00000001U)

#define CSL_IOMUX_SPI1_D1_CFG_REG_GPIO_SEL_MASK                                (0x00030000U)
#define CSL_IOMUX_SPI1_D1_CFG_REG_GPIO_SEL_SHIFT                               (0x00000010U)
#define CSL_IOMUX_SPI1_D1_CFG_REG_GPIO_SEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_SPI1_D1_CFG_REG_GPIO_SEL_MAX                                 (0x00000003U)

#define CSL_IOMUX_SPI1_D1_CFG_REG_QUAL_SEL_MASK                                (0x000C0000U)
#define CSL_IOMUX_SPI1_D1_CFG_REG_QUAL_SEL_SHIFT                               (0x00000012U)
#define CSL_IOMUX_SPI1_D1_CFG_REG_QUAL_SEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_SPI1_D1_CFG_REG_QUAL_SEL_MAX                                 (0x00000003U)

#define CSL_IOMUX_SPI1_D1_CFG_REG_INP_INV_SEL_MASK                             (0x00100000U)
#define CSL_IOMUX_SPI1_D1_CFG_REG_INP_INV_SEL_SHIFT                            (0x00000014U)
#define CSL_IOMUX_SPI1_D1_CFG_REG_INP_INV_SEL_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_SPI1_D1_CFG_REG_INP_INV_SEL_MAX                              (0x00000001U)

#define CSL_IOMUX_SPI1_D1_CFG_REG_ICSSM_GPIO_SEL_MASK                          (0x00200000U)
#define CSL_IOMUX_SPI1_D1_CFG_REG_ICSSM_GPIO_SEL_SHIFT                         (0x00000015U)
#define CSL_IOMUX_SPI1_D1_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_SPI1_D1_CFG_REG_ICSSM_GPIO_SEL_MAX                           (0x00000001U)

#define CSL_IOMUX_SPI1_D1_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                     (0x00400000U)
#define CSL_IOMUX_SPI1_D1_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                    (0x00000016U)
#define CSL_IOMUX_SPI1_D1_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL                 (0x00000000U)
#define CSL_IOMUX_SPI1_D1_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                      (0x00000001U)

#define CSL_IOMUX_SPI1_D1_CFG_REG_HSMODE_MASK                                  (0x40000000U)
#define CSL_IOMUX_SPI1_D1_CFG_REG_HSMODE_SHIFT                                 (0x0000001EU)
#define CSL_IOMUX_SPI1_D1_CFG_REG_HSMODE_RESETVAL                              (0x00000000U)
#define CSL_IOMUX_SPI1_D1_CFG_REG_HSMODE_MAX                                   (0x00000001U)

#define CSL_IOMUX_SPI1_D1_CFG_REG_HSMASTER_MASK                                (0x80000000U)
#define CSL_IOMUX_SPI1_D1_CFG_REG_HSMASTER_SHIFT                               (0x0000001FU)
#define CSL_IOMUX_SPI1_D1_CFG_REG_HSMASTER_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_SPI1_D1_CFG_REG_HSMASTER_MAX                                 (0x00000001U)

#define CSL_IOMUX_SPI1_D1_CFG_REG_RESETVAL                                     (0x000005F7U)

/* LIN1_RXD_CFG_REG */

#define CSL_IOMUX_LIN1_RXD_CFG_REG_FUNC_SEL_MASK                               (0x0000000FU)
#define CSL_IOMUX_LIN1_RXD_CFG_REG_FUNC_SEL_SHIFT                              (0x00000000U)
#define CSL_IOMUX_LIN1_RXD_CFG_REG_FUNC_SEL_RESETVAL                           (0x00000007U)
#define CSL_IOMUX_LIN1_RXD_CFG_REG_FUNC_SEL_MAX                                (0x0000000FU)

#define CSL_IOMUX_LIN1_RXD_CFG_REG_IE_OVERRIDE_CTRL_MASK                       (0x00000010U)
#define CSL_IOMUX_LIN1_RXD_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                      (0x00000004U)
#define CSL_IOMUX_LIN1_RXD_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_LIN1_RXD_CFG_REG_IE_OVERRIDE_CTRL_MAX                        (0x00000001U)

#define CSL_IOMUX_LIN1_RXD_CFG_REG_IE_OVERRIDE_MASK                            (0x00000020U)
#define CSL_IOMUX_LIN1_RXD_CFG_REG_IE_OVERRIDE_SHIFT                           (0x00000005U)
#define CSL_IOMUX_LIN1_RXD_CFG_REG_IE_OVERRIDE_RESETVAL                        (0x00000001U)
#define CSL_IOMUX_LIN1_RXD_CFG_REG_IE_OVERRIDE_MAX                             (0x00000001U)

#define CSL_IOMUX_LIN1_RXD_CFG_REG_OE_OVERRIDE_CTRL_MASK                       (0x00000040U)
#define CSL_IOMUX_LIN1_RXD_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                      (0x00000006U)
#define CSL_IOMUX_LIN1_RXD_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_LIN1_RXD_CFG_REG_OE_OVERRIDE_CTRL_MAX                        (0x00000001U)

#define CSL_IOMUX_LIN1_RXD_CFG_REG_OE_OVERRIDE_MASK                            (0x00000080U)
#define CSL_IOMUX_LIN1_RXD_CFG_REG_OE_OVERRIDE_SHIFT                           (0x00000007U)
#define CSL_IOMUX_LIN1_RXD_CFG_REG_OE_OVERRIDE_RESETVAL                        (0x00000001U)
#define CSL_IOMUX_LIN1_RXD_CFG_REG_OE_OVERRIDE_MAX                             (0x00000001U)

#define CSL_IOMUX_LIN1_RXD_CFG_REG_PI_MASK                                     (0x00000100U)
#define CSL_IOMUX_LIN1_RXD_CFG_REG_PI_SHIFT                                    (0x00000008U)
#define CSL_IOMUX_LIN1_RXD_CFG_REG_PI_RESETVAL                                 (0x00000001U)
#define CSL_IOMUX_LIN1_RXD_CFG_REG_PI_MAX                                      (0x00000001U)

#define CSL_IOMUX_LIN1_RXD_CFG_REG_PUPDSEL_MASK                                (0x00000200U)
#define CSL_IOMUX_LIN1_RXD_CFG_REG_PUPDSEL_SHIFT                               (0x00000009U)
#define CSL_IOMUX_LIN1_RXD_CFG_REG_PUPDSEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_LIN1_RXD_CFG_REG_PUPDSEL_MAX                                 (0x00000001U)

#define CSL_IOMUX_LIN1_RXD_CFG_REG_SC1_MASK                                    (0x00000400U)
#define CSL_IOMUX_LIN1_RXD_CFG_REG_SC1_SHIFT                                   (0x0000000AU)
#define CSL_IOMUX_LIN1_RXD_CFG_REG_SC1_RESETVAL                                (0x00000001U)
#define CSL_IOMUX_LIN1_RXD_CFG_REG_SC1_MAX                                     (0x00000001U)

#define CSL_IOMUX_LIN1_RXD_CFG_REG_GPIO_SEL_MASK                               (0x00030000U)
#define CSL_IOMUX_LIN1_RXD_CFG_REG_GPIO_SEL_SHIFT                              (0x00000010U)
#define CSL_IOMUX_LIN1_RXD_CFG_REG_GPIO_SEL_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_LIN1_RXD_CFG_REG_GPIO_SEL_MAX                                (0x00000003U)

#define CSL_IOMUX_LIN1_RXD_CFG_REG_QUAL_SEL_MASK                               (0x000C0000U)
#define CSL_IOMUX_LIN1_RXD_CFG_REG_QUAL_SEL_SHIFT                              (0x00000012U)
#define CSL_IOMUX_LIN1_RXD_CFG_REG_QUAL_SEL_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_LIN1_RXD_CFG_REG_QUAL_SEL_MAX                                (0x00000003U)

#define CSL_IOMUX_LIN1_RXD_CFG_REG_INP_INV_SEL_MASK                            (0x00100000U)
#define CSL_IOMUX_LIN1_RXD_CFG_REG_INP_INV_SEL_SHIFT                           (0x00000014U)
#define CSL_IOMUX_LIN1_RXD_CFG_REG_INP_INV_SEL_RESETVAL                        (0x00000000U)
#define CSL_IOMUX_LIN1_RXD_CFG_REG_INP_INV_SEL_MAX                             (0x00000001U)

#define CSL_IOMUX_LIN1_RXD_CFG_REG_ICSSM_GPIO_SEL_MASK                         (0x00200000U)
#define CSL_IOMUX_LIN1_RXD_CFG_REG_ICSSM_GPIO_SEL_SHIFT                        (0x00000015U)
#define CSL_IOMUX_LIN1_RXD_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_LIN1_RXD_CFG_REG_ICSSM_GPIO_SEL_MAX                          (0x00000001U)

#define CSL_IOMUX_LIN1_RXD_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                    (0x00400000U)
#define CSL_IOMUX_LIN1_RXD_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                   (0x00000016U)
#define CSL_IOMUX_LIN1_RXD_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL                (0x00000000U)
#define CSL_IOMUX_LIN1_RXD_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                     (0x00000001U)

#define CSL_IOMUX_LIN1_RXD_CFG_REG_HSMODE_MASK                                 (0x40000000U)
#define CSL_IOMUX_LIN1_RXD_CFG_REG_HSMODE_SHIFT                                (0x0000001EU)
#define CSL_IOMUX_LIN1_RXD_CFG_REG_HSMODE_RESETVAL                             (0x00000000U)
#define CSL_IOMUX_LIN1_RXD_CFG_REG_HSMODE_MAX                                  (0x00000001U)

#define CSL_IOMUX_LIN1_RXD_CFG_REG_HSMASTER_MASK                               (0x80000000U)
#define CSL_IOMUX_LIN1_RXD_CFG_REG_HSMASTER_SHIFT                              (0x0000001FU)
#define CSL_IOMUX_LIN1_RXD_CFG_REG_HSMASTER_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_LIN1_RXD_CFG_REG_HSMASTER_MAX                                (0x00000001U)

#define CSL_IOMUX_LIN1_RXD_CFG_REG_RESETVAL                                    (0x000005F7U)

/* LIN1_TXD_CFG_REG */

#define CSL_IOMUX_LIN1_TXD_CFG_REG_FUNC_SEL_MASK                               (0x0000000FU)
#define CSL_IOMUX_LIN1_TXD_CFG_REG_FUNC_SEL_SHIFT                              (0x00000000U)
#define CSL_IOMUX_LIN1_TXD_CFG_REG_FUNC_SEL_RESETVAL                           (0x00000007U)
#define CSL_IOMUX_LIN1_TXD_CFG_REG_FUNC_SEL_MAX                                (0x0000000FU)

#define CSL_IOMUX_LIN1_TXD_CFG_REG_IE_OVERRIDE_CTRL_MASK                       (0x00000010U)
#define CSL_IOMUX_LIN1_TXD_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                      (0x00000004U)
#define CSL_IOMUX_LIN1_TXD_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_LIN1_TXD_CFG_REG_IE_OVERRIDE_CTRL_MAX                        (0x00000001U)

#define CSL_IOMUX_LIN1_TXD_CFG_REG_IE_OVERRIDE_MASK                            (0x00000020U)
#define CSL_IOMUX_LIN1_TXD_CFG_REG_IE_OVERRIDE_SHIFT                           (0x00000005U)
#define CSL_IOMUX_LIN1_TXD_CFG_REG_IE_OVERRIDE_RESETVAL                        (0x00000001U)
#define CSL_IOMUX_LIN1_TXD_CFG_REG_IE_OVERRIDE_MAX                             (0x00000001U)

#define CSL_IOMUX_LIN1_TXD_CFG_REG_OE_OVERRIDE_CTRL_MASK                       (0x00000040U)
#define CSL_IOMUX_LIN1_TXD_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                      (0x00000006U)
#define CSL_IOMUX_LIN1_TXD_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_LIN1_TXD_CFG_REG_OE_OVERRIDE_CTRL_MAX                        (0x00000001U)

#define CSL_IOMUX_LIN1_TXD_CFG_REG_OE_OVERRIDE_MASK                            (0x00000080U)
#define CSL_IOMUX_LIN1_TXD_CFG_REG_OE_OVERRIDE_SHIFT                           (0x00000007U)
#define CSL_IOMUX_LIN1_TXD_CFG_REG_OE_OVERRIDE_RESETVAL                        (0x00000001U)
#define CSL_IOMUX_LIN1_TXD_CFG_REG_OE_OVERRIDE_MAX                             (0x00000001U)

#define CSL_IOMUX_LIN1_TXD_CFG_REG_PI_MASK                                     (0x00000100U)
#define CSL_IOMUX_LIN1_TXD_CFG_REG_PI_SHIFT                                    (0x00000008U)
#define CSL_IOMUX_LIN1_TXD_CFG_REG_PI_RESETVAL                                 (0x00000001U)
#define CSL_IOMUX_LIN1_TXD_CFG_REG_PI_MAX                                      (0x00000001U)

#define CSL_IOMUX_LIN1_TXD_CFG_REG_PUPDSEL_MASK                                (0x00000200U)
#define CSL_IOMUX_LIN1_TXD_CFG_REG_PUPDSEL_SHIFT                               (0x00000009U)
#define CSL_IOMUX_LIN1_TXD_CFG_REG_PUPDSEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_LIN1_TXD_CFG_REG_PUPDSEL_MAX                                 (0x00000001U)

#define CSL_IOMUX_LIN1_TXD_CFG_REG_SC1_MASK                                    (0x00000400U)
#define CSL_IOMUX_LIN1_TXD_CFG_REG_SC1_SHIFT                                   (0x0000000AU)
#define CSL_IOMUX_LIN1_TXD_CFG_REG_SC1_RESETVAL                                (0x00000001U)
#define CSL_IOMUX_LIN1_TXD_CFG_REG_SC1_MAX                                     (0x00000001U)

#define CSL_IOMUX_LIN1_TXD_CFG_REG_GPIO_SEL_MASK                               (0x00030000U)
#define CSL_IOMUX_LIN1_TXD_CFG_REG_GPIO_SEL_SHIFT                              (0x00000010U)
#define CSL_IOMUX_LIN1_TXD_CFG_REG_GPIO_SEL_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_LIN1_TXD_CFG_REG_GPIO_SEL_MAX                                (0x00000003U)

#define CSL_IOMUX_LIN1_TXD_CFG_REG_QUAL_SEL_MASK                               (0x000C0000U)
#define CSL_IOMUX_LIN1_TXD_CFG_REG_QUAL_SEL_SHIFT                              (0x00000012U)
#define CSL_IOMUX_LIN1_TXD_CFG_REG_QUAL_SEL_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_LIN1_TXD_CFG_REG_QUAL_SEL_MAX                                (0x00000003U)

#define CSL_IOMUX_LIN1_TXD_CFG_REG_INP_INV_SEL_MASK                            (0x00100000U)
#define CSL_IOMUX_LIN1_TXD_CFG_REG_INP_INV_SEL_SHIFT                           (0x00000014U)
#define CSL_IOMUX_LIN1_TXD_CFG_REG_INP_INV_SEL_RESETVAL                        (0x00000000U)
#define CSL_IOMUX_LIN1_TXD_CFG_REG_INP_INV_SEL_MAX                             (0x00000001U)

#define CSL_IOMUX_LIN1_TXD_CFG_REG_ICSSM_GPIO_SEL_MASK                         (0x00200000U)
#define CSL_IOMUX_LIN1_TXD_CFG_REG_ICSSM_GPIO_SEL_SHIFT                        (0x00000015U)
#define CSL_IOMUX_LIN1_TXD_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_LIN1_TXD_CFG_REG_ICSSM_GPIO_SEL_MAX                          (0x00000001U)

#define CSL_IOMUX_LIN1_TXD_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                    (0x00400000U)
#define CSL_IOMUX_LIN1_TXD_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                   (0x00000016U)
#define CSL_IOMUX_LIN1_TXD_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL                (0x00000000U)
#define CSL_IOMUX_LIN1_TXD_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                     (0x00000001U)

#define CSL_IOMUX_LIN1_TXD_CFG_REG_HSMODE_MASK                                 (0x40000000U)
#define CSL_IOMUX_LIN1_TXD_CFG_REG_HSMODE_SHIFT                                (0x0000001EU)
#define CSL_IOMUX_LIN1_TXD_CFG_REG_HSMODE_RESETVAL                             (0x00000000U)
#define CSL_IOMUX_LIN1_TXD_CFG_REG_HSMODE_MAX                                  (0x00000001U)

#define CSL_IOMUX_LIN1_TXD_CFG_REG_HSMASTER_MASK                               (0x80000000U)
#define CSL_IOMUX_LIN1_TXD_CFG_REG_HSMASTER_SHIFT                              (0x0000001FU)
#define CSL_IOMUX_LIN1_TXD_CFG_REG_HSMASTER_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_LIN1_TXD_CFG_REG_HSMASTER_MAX                                (0x00000001U)

#define CSL_IOMUX_LIN1_TXD_CFG_REG_RESETVAL                                    (0x000005F7U)

/* LIN2_RXD_CFG_REG */

#define CSL_IOMUX_LIN2_RXD_CFG_REG_FUNC_SEL_MASK                               (0x0000000FU)
#define CSL_IOMUX_LIN2_RXD_CFG_REG_FUNC_SEL_SHIFT                              (0x00000000U)
#define CSL_IOMUX_LIN2_RXD_CFG_REG_FUNC_SEL_RESETVAL                           (0x00000007U)
#define CSL_IOMUX_LIN2_RXD_CFG_REG_FUNC_SEL_MAX                                (0x0000000FU)

#define CSL_IOMUX_LIN2_RXD_CFG_REG_IE_OVERRIDE_CTRL_MASK                       (0x00000010U)
#define CSL_IOMUX_LIN2_RXD_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                      (0x00000004U)
#define CSL_IOMUX_LIN2_RXD_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_LIN2_RXD_CFG_REG_IE_OVERRIDE_CTRL_MAX                        (0x00000001U)

#define CSL_IOMUX_LIN2_RXD_CFG_REG_IE_OVERRIDE_MASK                            (0x00000020U)
#define CSL_IOMUX_LIN2_RXD_CFG_REG_IE_OVERRIDE_SHIFT                           (0x00000005U)
#define CSL_IOMUX_LIN2_RXD_CFG_REG_IE_OVERRIDE_RESETVAL                        (0x00000001U)
#define CSL_IOMUX_LIN2_RXD_CFG_REG_IE_OVERRIDE_MAX                             (0x00000001U)

#define CSL_IOMUX_LIN2_RXD_CFG_REG_OE_OVERRIDE_CTRL_MASK                       (0x00000040U)
#define CSL_IOMUX_LIN2_RXD_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                      (0x00000006U)
#define CSL_IOMUX_LIN2_RXD_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_LIN2_RXD_CFG_REG_OE_OVERRIDE_CTRL_MAX                        (0x00000001U)

#define CSL_IOMUX_LIN2_RXD_CFG_REG_OE_OVERRIDE_MASK                            (0x00000080U)
#define CSL_IOMUX_LIN2_RXD_CFG_REG_OE_OVERRIDE_SHIFT                           (0x00000007U)
#define CSL_IOMUX_LIN2_RXD_CFG_REG_OE_OVERRIDE_RESETVAL                        (0x00000001U)
#define CSL_IOMUX_LIN2_RXD_CFG_REG_OE_OVERRIDE_MAX                             (0x00000001U)

#define CSL_IOMUX_LIN2_RXD_CFG_REG_PI_MASK                                     (0x00000100U)
#define CSL_IOMUX_LIN2_RXD_CFG_REG_PI_SHIFT                                    (0x00000008U)
#define CSL_IOMUX_LIN2_RXD_CFG_REG_PI_RESETVAL                                 (0x00000001U)
#define CSL_IOMUX_LIN2_RXD_CFG_REG_PI_MAX                                      (0x00000001U)

#define CSL_IOMUX_LIN2_RXD_CFG_REG_PUPDSEL_MASK                                (0x00000200U)
#define CSL_IOMUX_LIN2_RXD_CFG_REG_PUPDSEL_SHIFT                               (0x00000009U)
#define CSL_IOMUX_LIN2_RXD_CFG_REG_PUPDSEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_LIN2_RXD_CFG_REG_PUPDSEL_MAX                                 (0x00000001U)

#define CSL_IOMUX_LIN2_RXD_CFG_REG_SC1_MASK                                    (0x00000400U)
#define CSL_IOMUX_LIN2_RXD_CFG_REG_SC1_SHIFT                                   (0x0000000AU)
#define CSL_IOMUX_LIN2_RXD_CFG_REG_SC1_RESETVAL                                (0x00000001U)
#define CSL_IOMUX_LIN2_RXD_CFG_REG_SC1_MAX                                     (0x00000001U)

#define CSL_IOMUX_LIN2_RXD_CFG_REG_GPIO_SEL_MASK                               (0x00030000U)
#define CSL_IOMUX_LIN2_RXD_CFG_REG_GPIO_SEL_SHIFT                              (0x00000010U)
#define CSL_IOMUX_LIN2_RXD_CFG_REG_GPIO_SEL_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_LIN2_RXD_CFG_REG_GPIO_SEL_MAX                                (0x00000003U)

#define CSL_IOMUX_LIN2_RXD_CFG_REG_QUAL_SEL_MASK                               (0x000C0000U)
#define CSL_IOMUX_LIN2_RXD_CFG_REG_QUAL_SEL_SHIFT                              (0x00000012U)
#define CSL_IOMUX_LIN2_RXD_CFG_REG_QUAL_SEL_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_LIN2_RXD_CFG_REG_QUAL_SEL_MAX                                (0x00000003U)

#define CSL_IOMUX_LIN2_RXD_CFG_REG_INP_INV_SEL_MASK                            (0x00100000U)
#define CSL_IOMUX_LIN2_RXD_CFG_REG_INP_INV_SEL_SHIFT                           (0x00000014U)
#define CSL_IOMUX_LIN2_RXD_CFG_REG_INP_INV_SEL_RESETVAL                        (0x00000000U)
#define CSL_IOMUX_LIN2_RXD_CFG_REG_INP_INV_SEL_MAX                             (0x00000001U)

#define CSL_IOMUX_LIN2_RXD_CFG_REG_ICSSM_GPIO_SEL_MASK                         (0x00200000U)
#define CSL_IOMUX_LIN2_RXD_CFG_REG_ICSSM_GPIO_SEL_SHIFT                        (0x00000015U)
#define CSL_IOMUX_LIN2_RXD_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_LIN2_RXD_CFG_REG_ICSSM_GPIO_SEL_MAX                          (0x00000001U)

#define CSL_IOMUX_LIN2_RXD_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                    (0x00400000U)
#define CSL_IOMUX_LIN2_RXD_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                   (0x00000016U)
#define CSL_IOMUX_LIN2_RXD_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL                (0x00000000U)
#define CSL_IOMUX_LIN2_RXD_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                     (0x00000001U)

#define CSL_IOMUX_LIN2_RXD_CFG_REG_HSMODE_MASK                                 (0x40000000U)
#define CSL_IOMUX_LIN2_RXD_CFG_REG_HSMODE_SHIFT                                (0x0000001EU)
#define CSL_IOMUX_LIN2_RXD_CFG_REG_HSMODE_RESETVAL                             (0x00000000U)
#define CSL_IOMUX_LIN2_RXD_CFG_REG_HSMODE_MAX                                  (0x00000001U)

#define CSL_IOMUX_LIN2_RXD_CFG_REG_HSMASTER_MASK                               (0x80000000U)
#define CSL_IOMUX_LIN2_RXD_CFG_REG_HSMASTER_SHIFT                              (0x0000001FU)
#define CSL_IOMUX_LIN2_RXD_CFG_REG_HSMASTER_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_LIN2_RXD_CFG_REG_HSMASTER_MAX                                (0x00000001U)

#define CSL_IOMUX_LIN2_RXD_CFG_REG_RESETVAL                                    (0x000005F7U)

/* LIN2_TXD_CFG_REG */

#define CSL_IOMUX_LIN2_TXD_CFG_REG_FUNC_SEL_MASK                               (0x0000000FU)
#define CSL_IOMUX_LIN2_TXD_CFG_REG_FUNC_SEL_SHIFT                              (0x00000000U)
#define CSL_IOMUX_LIN2_TXD_CFG_REG_FUNC_SEL_RESETVAL                           (0x00000007U)
#define CSL_IOMUX_LIN2_TXD_CFG_REG_FUNC_SEL_MAX                                (0x0000000FU)

#define CSL_IOMUX_LIN2_TXD_CFG_REG_IE_OVERRIDE_CTRL_MASK                       (0x00000010U)
#define CSL_IOMUX_LIN2_TXD_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                      (0x00000004U)
#define CSL_IOMUX_LIN2_TXD_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_LIN2_TXD_CFG_REG_IE_OVERRIDE_CTRL_MAX                        (0x00000001U)

#define CSL_IOMUX_LIN2_TXD_CFG_REG_IE_OVERRIDE_MASK                            (0x00000020U)
#define CSL_IOMUX_LIN2_TXD_CFG_REG_IE_OVERRIDE_SHIFT                           (0x00000005U)
#define CSL_IOMUX_LIN2_TXD_CFG_REG_IE_OVERRIDE_RESETVAL                        (0x00000001U)
#define CSL_IOMUX_LIN2_TXD_CFG_REG_IE_OVERRIDE_MAX                             (0x00000001U)

#define CSL_IOMUX_LIN2_TXD_CFG_REG_OE_OVERRIDE_CTRL_MASK                       (0x00000040U)
#define CSL_IOMUX_LIN2_TXD_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                      (0x00000006U)
#define CSL_IOMUX_LIN2_TXD_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_LIN2_TXD_CFG_REG_OE_OVERRIDE_CTRL_MAX                        (0x00000001U)

#define CSL_IOMUX_LIN2_TXD_CFG_REG_OE_OVERRIDE_MASK                            (0x00000080U)
#define CSL_IOMUX_LIN2_TXD_CFG_REG_OE_OVERRIDE_SHIFT                           (0x00000007U)
#define CSL_IOMUX_LIN2_TXD_CFG_REG_OE_OVERRIDE_RESETVAL                        (0x00000001U)
#define CSL_IOMUX_LIN2_TXD_CFG_REG_OE_OVERRIDE_MAX                             (0x00000001U)

#define CSL_IOMUX_LIN2_TXD_CFG_REG_PI_MASK                                     (0x00000100U)
#define CSL_IOMUX_LIN2_TXD_CFG_REG_PI_SHIFT                                    (0x00000008U)
#define CSL_IOMUX_LIN2_TXD_CFG_REG_PI_RESETVAL                                 (0x00000001U)
#define CSL_IOMUX_LIN2_TXD_CFG_REG_PI_MAX                                      (0x00000001U)

#define CSL_IOMUX_LIN2_TXD_CFG_REG_PUPDSEL_MASK                                (0x00000200U)
#define CSL_IOMUX_LIN2_TXD_CFG_REG_PUPDSEL_SHIFT                               (0x00000009U)
#define CSL_IOMUX_LIN2_TXD_CFG_REG_PUPDSEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_LIN2_TXD_CFG_REG_PUPDSEL_MAX                                 (0x00000001U)

#define CSL_IOMUX_LIN2_TXD_CFG_REG_SC1_MASK                                    (0x00000400U)
#define CSL_IOMUX_LIN2_TXD_CFG_REG_SC1_SHIFT                                   (0x0000000AU)
#define CSL_IOMUX_LIN2_TXD_CFG_REG_SC1_RESETVAL                                (0x00000001U)
#define CSL_IOMUX_LIN2_TXD_CFG_REG_SC1_MAX                                     (0x00000001U)

#define CSL_IOMUX_LIN2_TXD_CFG_REG_GPIO_SEL_MASK                               (0x00030000U)
#define CSL_IOMUX_LIN2_TXD_CFG_REG_GPIO_SEL_SHIFT                              (0x00000010U)
#define CSL_IOMUX_LIN2_TXD_CFG_REG_GPIO_SEL_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_LIN2_TXD_CFG_REG_GPIO_SEL_MAX                                (0x00000003U)

#define CSL_IOMUX_LIN2_TXD_CFG_REG_QUAL_SEL_MASK                               (0x000C0000U)
#define CSL_IOMUX_LIN2_TXD_CFG_REG_QUAL_SEL_SHIFT                              (0x00000012U)
#define CSL_IOMUX_LIN2_TXD_CFG_REG_QUAL_SEL_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_LIN2_TXD_CFG_REG_QUAL_SEL_MAX                                (0x00000003U)

#define CSL_IOMUX_LIN2_TXD_CFG_REG_INP_INV_SEL_MASK                            (0x00100000U)
#define CSL_IOMUX_LIN2_TXD_CFG_REG_INP_INV_SEL_SHIFT                           (0x00000014U)
#define CSL_IOMUX_LIN2_TXD_CFG_REG_INP_INV_SEL_RESETVAL                        (0x00000000U)
#define CSL_IOMUX_LIN2_TXD_CFG_REG_INP_INV_SEL_MAX                             (0x00000001U)

#define CSL_IOMUX_LIN2_TXD_CFG_REG_ICSSM_GPIO_SEL_MASK                         (0x00200000U)
#define CSL_IOMUX_LIN2_TXD_CFG_REG_ICSSM_GPIO_SEL_SHIFT                        (0x00000015U)
#define CSL_IOMUX_LIN2_TXD_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_LIN2_TXD_CFG_REG_ICSSM_GPIO_SEL_MAX                          (0x00000001U)

#define CSL_IOMUX_LIN2_TXD_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                    (0x00400000U)
#define CSL_IOMUX_LIN2_TXD_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                   (0x00000016U)
#define CSL_IOMUX_LIN2_TXD_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL                (0x00000000U)
#define CSL_IOMUX_LIN2_TXD_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                     (0x00000001U)

#define CSL_IOMUX_LIN2_TXD_CFG_REG_HSMODE_MASK                                 (0x40000000U)
#define CSL_IOMUX_LIN2_TXD_CFG_REG_HSMODE_SHIFT                                (0x0000001EU)
#define CSL_IOMUX_LIN2_TXD_CFG_REG_HSMODE_RESETVAL                             (0x00000000U)
#define CSL_IOMUX_LIN2_TXD_CFG_REG_HSMODE_MAX                                  (0x00000001U)

#define CSL_IOMUX_LIN2_TXD_CFG_REG_HSMASTER_MASK                               (0x80000000U)
#define CSL_IOMUX_LIN2_TXD_CFG_REG_HSMASTER_SHIFT                              (0x0000001FU)
#define CSL_IOMUX_LIN2_TXD_CFG_REG_HSMASTER_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_LIN2_TXD_CFG_REG_HSMASTER_MAX                                (0x00000001U)

#define CSL_IOMUX_LIN2_TXD_CFG_REG_RESETVAL                                    (0x000005F7U)

/* I2C1_SCL_CFG_REG */

#define CSL_IOMUX_I2C1_SCL_CFG_REG_FUNC_SEL_MASK                               (0x0000000FU)
#define CSL_IOMUX_I2C1_SCL_CFG_REG_FUNC_SEL_SHIFT                              (0x00000000U)
#define CSL_IOMUX_I2C1_SCL_CFG_REG_FUNC_SEL_RESETVAL                           (0x00000007U)
#define CSL_IOMUX_I2C1_SCL_CFG_REG_FUNC_SEL_MAX                                (0x0000000FU)

#define CSL_IOMUX_I2C1_SCL_CFG_REG_IE_OVERRIDE_CTRL_MASK                       (0x00000010U)
#define CSL_IOMUX_I2C1_SCL_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                      (0x00000004U)
#define CSL_IOMUX_I2C1_SCL_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_I2C1_SCL_CFG_REG_IE_OVERRIDE_CTRL_MAX                        (0x00000001U)

#define CSL_IOMUX_I2C1_SCL_CFG_REG_IE_OVERRIDE_MASK                            (0x00000020U)
#define CSL_IOMUX_I2C1_SCL_CFG_REG_IE_OVERRIDE_SHIFT                           (0x00000005U)
#define CSL_IOMUX_I2C1_SCL_CFG_REG_IE_OVERRIDE_RESETVAL                        (0x00000001U)
#define CSL_IOMUX_I2C1_SCL_CFG_REG_IE_OVERRIDE_MAX                             (0x00000001U)

#define CSL_IOMUX_I2C1_SCL_CFG_REG_OE_OVERRIDE_CTRL_MASK                       (0x00000040U)
#define CSL_IOMUX_I2C1_SCL_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                      (0x00000006U)
#define CSL_IOMUX_I2C1_SCL_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_I2C1_SCL_CFG_REG_OE_OVERRIDE_CTRL_MAX                        (0x00000001U)

#define CSL_IOMUX_I2C1_SCL_CFG_REG_OE_OVERRIDE_MASK                            (0x00000080U)
#define CSL_IOMUX_I2C1_SCL_CFG_REG_OE_OVERRIDE_SHIFT                           (0x00000007U)
#define CSL_IOMUX_I2C1_SCL_CFG_REG_OE_OVERRIDE_RESETVAL                        (0x00000001U)
#define CSL_IOMUX_I2C1_SCL_CFG_REG_OE_OVERRIDE_MAX                             (0x00000001U)

#define CSL_IOMUX_I2C1_SCL_CFG_REG_PI_MASK                                     (0x00000100U)
#define CSL_IOMUX_I2C1_SCL_CFG_REG_PI_SHIFT                                    (0x00000008U)
#define CSL_IOMUX_I2C1_SCL_CFG_REG_PI_RESETVAL                                 (0x00000001U)
#define CSL_IOMUX_I2C1_SCL_CFG_REG_PI_MAX                                      (0x00000001U)

#define CSL_IOMUX_I2C1_SCL_CFG_REG_PUPDSEL_MASK                                (0x00000200U)
#define CSL_IOMUX_I2C1_SCL_CFG_REG_PUPDSEL_SHIFT                               (0x00000009U)
#define CSL_IOMUX_I2C1_SCL_CFG_REG_PUPDSEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_I2C1_SCL_CFG_REG_PUPDSEL_MAX                                 (0x00000001U)

#define CSL_IOMUX_I2C1_SCL_CFG_REG_SC1_MASK                                    (0x00000400U)
#define CSL_IOMUX_I2C1_SCL_CFG_REG_SC1_SHIFT                                   (0x0000000AU)
#define CSL_IOMUX_I2C1_SCL_CFG_REG_SC1_RESETVAL                                (0x00000001U)
#define CSL_IOMUX_I2C1_SCL_CFG_REG_SC1_MAX                                     (0x00000001U)

#define CSL_IOMUX_I2C1_SCL_CFG_REG_GPIO_SEL_MASK                               (0x00030000U)
#define CSL_IOMUX_I2C1_SCL_CFG_REG_GPIO_SEL_SHIFT                              (0x00000010U)
#define CSL_IOMUX_I2C1_SCL_CFG_REG_GPIO_SEL_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_I2C1_SCL_CFG_REG_GPIO_SEL_MAX                                (0x00000003U)

#define CSL_IOMUX_I2C1_SCL_CFG_REG_QUAL_SEL_MASK                               (0x000C0000U)
#define CSL_IOMUX_I2C1_SCL_CFG_REG_QUAL_SEL_SHIFT                              (0x00000012U)
#define CSL_IOMUX_I2C1_SCL_CFG_REG_QUAL_SEL_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_I2C1_SCL_CFG_REG_QUAL_SEL_MAX                                (0x00000003U)

#define CSL_IOMUX_I2C1_SCL_CFG_REG_INP_INV_SEL_MASK                            (0x00100000U)
#define CSL_IOMUX_I2C1_SCL_CFG_REG_INP_INV_SEL_SHIFT                           (0x00000014U)
#define CSL_IOMUX_I2C1_SCL_CFG_REG_INP_INV_SEL_RESETVAL                        (0x00000000U)
#define CSL_IOMUX_I2C1_SCL_CFG_REG_INP_INV_SEL_MAX                             (0x00000001U)

#define CSL_IOMUX_I2C1_SCL_CFG_REG_ICSSM_GPIO_SEL_MASK                         (0x00200000U)
#define CSL_IOMUX_I2C1_SCL_CFG_REG_ICSSM_GPIO_SEL_SHIFT                        (0x00000015U)
#define CSL_IOMUX_I2C1_SCL_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_I2C1_SCL_CFG_REG_ICSSM_GPIO_SEL_MAX                          (0x00000001U)

#define CSL_IOMUX_I2C1_SCL_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                    (0x00400000U)
#define CSL_IOMUX_I2C1_SCL_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                   (0x00000016U)
#define CSL_IOMUX_I2C1_SCL_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL                (0x00000000U)
#define CSL_IOMUX_I2C1_SCL_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                     (0x00000001U)

#define CSL_IOMUX_I2C1_SCL_CFG_REG_HSMODE_MASK                                 (0x40000000U)
#define CSL_IOMUX_I2C1_SCL_CFG_REG_HSMODE_SHIFT                                (0x0000001EU)
#define CSL_IOMUX_I2C1_SCL_CFG_REG_HSMODE_RESETVAL                             (0x00000000U)
#define CSL_IOMUX_I2C1_SCL_CFG_REG_HSMODE_MAX                                  (0x00000001U)

#define CSL_IOMUX_I2C1_SCL_CFG_REG_HSMASTER_MASK                               (0x80000000U)
#define CSL_IOMUX_I2C1_SCL_CFG_REG_HSMASTER_SHIFT                              (0x0000001FU)
#define CSL_IOMUX_I2C1_SCL_CFG_REG_HSMASTER_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_I2C1_SCL_CFG_REG_HSMASTER_MAX                                (0x00000001U)

#define CSL_IOMUX_I2C1_SCL_CFG_REG_RESETVAL                                    (0x000005F7U)

/* I2C1_SDA_CFG_REG */

#define CSL_IOMUX_I2C1_SDA_CFG_REG_FUNC_SEL_MASK                               (0x0000000FU)
#define CSL_IOMUX_I2C1_SDA_CFG_REG_FUNC_SEL_SHIFT                              (0x00000000U)
#define CSL_IOMUX_I2C1_SDA_CFG_REG_FUNC_SEL_RESETVAL                           (0x00000007U)
#define CSL_IOMUX_I2C1_SDA_CFG_REG_FUNC_SEL_MAX                                (0x0000000FU)

#define CSL_IOMUX_I2C1_SDA_CFG_REG_IE_OVERRIDE_CTRL_MASK                       (0x00000010U)
#define CSL_IOMUX_I2C1_SDA_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                      (0x00000004U)
#define CSL_IOMUX_I2C1_SDA_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_I2C1_SDA_CFG_REG_IE_OVERRIDE_CTRL_MAX                        (0x00000001U)

#define CSL_IOMUX_I2C1_SDA_CFG_REG_IE_OVERRIDE_MASK                            (0x00000020U)
#define CSL_IOMUX_I2C1_SDA_CFG_REG_IE_OVERRIDE_SHIFT                           (0x00000005U)
#define CSL_IOMUX_I2C1_SDA_CFG_REG_IE_OVERRIDE_RESETVAL                        (0x00000001U)
#define CSL_IOMUX_I2C1_SDA_CFG_REG_IE_OVERRIDE_MAX                             (0x00000001U)

#define CSL_IOMUX_I2C1_SDA_CFG_REG_OE_OVERRIDE_CTRL_MASK                       (0x00000040U)
#define CSL_IOMUX_I2C1_SDA_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                      (0x00000006U)
#define CSL_IOMUX_I2C1_SDA_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_I2C1_SDA_CFG_REG_OE_OVERRIDE_CTRL_MAX                        (0x00000001U)

#define CSL_IOMUX_I2C1_SDA_CFG_REG_OE_OVERRIDE_MASK                            (0x00000080U)
#define CSL_IOMUX_I2C1_SDA_CFG_REG_OE_OVERRIDE_SHIFT                           (0x00000007U)
#define CSL_IOMUX_I2C1_SDA_CFG_REG_OE_OVERRIDE_RESETVAL                        (0x00000001U)
#define CSL_IOMUX_I2C1_SDA_CFG_REG_OE_OVERRIDE_MAX                             (0x00000001U)

#define CSL_IOMUX_I2C1_SDA_CFG_REG_PI_MASK                                     (0x00000100U)
#define CSL_IOMUX_I2C1_SDA_CFG_REG_PI_SHIFT                                    (0x00000008U)
#define CSL_IOMUX_I2C1_SDA_CFG_REG_PI_RESETVAL                                 (0x00000001U)
#define CSL_IOMUX_I2C1_SDA_CFG_REG_PI_MAX                                      (0x00000001U)

#define CSL_IOMUX_I2C1_SDA_CFG_REG_PUPDSEL_MASK                                (0x00000200U)
#define CSL_IOMUX_I2C1_SDA_CFG_REG_PUPDSEL_SHIFT                               (0x00000009U)
#define CSL_IOMUX_I2C1_SDA_CFG_REG_PUPDSEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_I2C1_SDA_CFG_REG_PUPDSEL_MAX                                 (0x00000001U)

#define CSL_IOMUX_I2C1_SDA_CFG_REG_SC1_MASK                                    (0x00000400U)
#define CSL_IOMUX_I2C1_SDA_CFG_REG_SC1_SHIFT                                   (0x0000000AU)
#define CSL_IOMUX_I2C1_SDA_CFG_REG_SC1_RESETVAL                                (0x00000001U)
#define CSL_IOMUX_I2C1_SDA_CFG_REG_SC1_MAX                                     (0x00000001U)

#define CSL_IOMUX_I2C1_SDA_CFG_REG_GPIO_SEL_MASK                               (0x00030000U)
#define CSL_IOMUX_I2C1_SDA_CFG_REG_GPIO_SEL_SHIFT                              (0x00000010U)
#define CSL_IOMUX_I2C1_SDA_CFG_REG_GPIO_SEL_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_I2C1_SDA_CFG_REG_GPIO_SEL_MAX                                (0x00000003U)

#define CSL_IOMUX_I2C1_SDA_CFG_REG_QUAL_SEL_MASK                               (0x000C0000U)
#define CSL_IOMUX_I2C1_SDA_CFG_REG_QUAL_SEL_SHIFT                              (0x00000012U)
#define CSL_IOMUX_I2C1_SDA_CFG_REG_QUAL_SEL_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_I2C1_SDA_CFG_REG_QUAL_SEL_MAX                                (0x00000003U)

#define CSL_IOMUX_I2C1_SDA_CFG_REG_INP_INV_SEL_MASK                            (0x00100000U)
#define CSL_IOMUX_I2C1_SDA_CFG_REG_INP_INV_SEL_SHIFT                           (0x00000014U)
#define CSL_IOMUX_I2C1_SDA_CFG_REG_INP_INV_SEL_RESETVAL                        (0x00000000U)
#define CSL_IOMUX_I2C1_SDA_CFG_REG_INP_INV_SEL_MAX                             (0x00000001U)

#define CSL_IOMUX_I2C1_SDA_CFG_REG_ICSSM_GPIO_SEL_MASK                         (0x00200000U)
#define CSL_IOMUX_I2C1_SDA_CFG_REG_ICSSM_GPIO_SEL_SHIFT                        (0x00000015U)
#define CSL_IOMUX_I2C1_SDA_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_I2C1_SDA_CFG_REG_ICSSM_GPIO_SEL_MAX                          (0x00000001U)

#define CSL_IOMUX_I2C1_SDA_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                    (0x00400000U)
#define CSL_IOMUX_I2C1_SDA_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                   (0x00000016U)
#define CSL_IOMUX_I2C1_SDA_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL                (0x00000000U)
#define CSL_IOMUX_I2C1_SDA_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                     (0x00000001U)

#define CSL_IOMUX_I2C1_SDA_CFG_REG_HSMODE_MASK                                 (0x40000000U)
#define CSL_IOMUX_I2C1_SDA_CFG_REG_HSMODE_SHIFT                                (0x0000001EU)
#define CSL_IOMUX_I2C1_SDA_CFG_REG_HSMODE_RESETVAL                             (0x00000000U)
#define CSL_IOMUX_I2C1_SDA_CFG_REG_HSMODE_MAX                                  (0x00000001U)

#define CSL_IOMUX_I2C1_SDA_CFG_REG_HSMASTER_MASK                               (0x80000000U)
#define CSL_IOMUX_I2C1_SDA_CFG_REG_HSMASTER_SHIFT                              (0x0000001FU)
#define CSL_IOMUX_I2C1_SDA_CFG_REG_HSMASTER_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_I2C1_SDA_CFG_REG_HSMASTER_MAX                                (0x00000001U)

#define CSL_IOMUX_I2C1_SDA_CFG_REG_RESETVAL                                    (0x000005F7U)

/* UART0_RTSN_CFG_REG */

#define CSL_IOMUX_UART0_RTSN_CFG_REG_FUNC_SEL_MASK                             (0x0000000FU)
#define CSL_IOMUX_UART0_RTSN_CFG_REG_FUNC_SEL_SHIFT                            (0x00000000U)
#define CSL_IOMUX_UART0_RTSN_CFG_REG_FUNC_SEL_RESETVAL                         (0x00000007U)
#define CSL_IOMUX_UART0_RTSN_CFG_REG_FUNC_SEL_MAX                              (0x0000000FU)

#define CSL_IOMUX_UART0_RTSN_CFG_REG_IE_OVERRIDE_CTRL_MASK                     (0x00000010U)
#define CSL_IOMUX_UART0_RTSN_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                    (0x00000004U)
#define CSL_IOMUX_UART0_RTSN_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_IOMUX_UART0_RTSN_CFG_REG_IE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_IOMUX_UART0_RTSN_CFG_REG_IE_OVERRIDE_MASK                          (0x00000020U)
#define CSL_IOMUX_UART0_RTSN_CFG_REG_IE_OVERRIDE_SHIFT                         (0x00000005U)
#define CSL_IOMUX_UART0_RTSN_CFG_REG_IE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_IOMUX_UART0_RTSN_CFG_REG_IE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_IOMUX_UART0_RTSN_CFG_REG_OE_OVERRIDE_CTRL_MASK                     (0x00000040U)
#define CSL_IOMUX_UART0_RTSN_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                    (0x00000006U)
#define CSL_IOMUX_UART0_RTSN_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_IOMUX_UART0_RTSN_CFG_REG_OE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_IOMUX_UART0_RTSN_CFG_REG_OE_OVERRIDE_MASK                          (0x00000080U)
#define CSL_IOMUX_UART0_RTSN_CFG_REG_OE_OVERRIDE_SHIFT                         (0x00000007U)
#define CSL_IOMUX_UART0_RTSN_CFG_REG_OE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_IOMUX_UART0_RTSN_CFG_REG_OE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_IOMUX_UART0_RTSN_CFG_REG_PI_MASK                                   (0x00000100U)
#define CSL_IOMUX_UART0_RTSN_CFG_REG_PI_SHIFT                                  (0x00000008U)
#define CSL_IOMUX_UART0_RTSN_CFG_REG_PI_RESETVAL                               (0x00000001U)
#define CSL_IOMUX_UART0_RTSN_CFG_REG_PI_MAX                                    (0x00000001U)

#define CSL_IOMUX_UART0_RTSN_CFG_REG_PUPDSEL_MASK                              (0x00000200U)
#define CSL_IOMUX_UART0_RTSN_CFG_REG_PUPDSEL_SHIFT                             (0x00000009U)
#define CSL_IOMUX_UART0_RTSN_CFG_REG_PUPDSEL_RESETVAL                          (0x00000000U)
#define CSL_IOMUX_UART0_RTSN_CFG_REG_PUPDSEL_MAX                               (0x00000001U)

#define CSL_IOMUX_UART0_RTSN_CFG_REG_SC1_MASK                                  (0x00000400U)
#define CSL_IOMUX_UART0_RTSN_CFG_REG_SC1_SHIFT                                 (0x0000000AU)
#define CSL_IOMUX_UART0_RTSN_CFG_REG_SC1_RESETVAL                              (0x00000001U)
#define CSL_IOMUX_UART0_RTSN_CFG_REG_SC1_MAX                                   (0x00000001U)

#define CSL_IOMUX_UART0_RTSN_CFG_REG_GPIO_SEL_MASK                             (0x00030000U)
#define CSL_IOMUX_UART0_RTSN_CFG_REG_GPIO_SEL_SHIFT                            (0x00000010U)
#define CSL_IOMUX_UART0_RTSN_CFG_REG_GPIO_SEL_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_UART0_RTSN_CFG_REG_GPIO_SEL_MAX                              (0x00000003U)

#define CSL_IOMUX_UART0_RTSN_CFG_REG_QUAL_SEL_MASK                             (0x000C0000U)
#define CSL_IOMUX_UART0_RTSN_CFG_REG_QUAL_SEL_SHIFT                            (0x00000012U)
#define CSL_IOMUX_UART0_RTSN_CFG_REG_QUAL_SEL_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_UART0_RTSN_CFG_REG_QUAL_SEL_MAX                              (0x00000003U)

#define CSL_IOMUX_UART0_RTSN_CFG_REG_INP_INV_SEL_MASK                          (0x00100000U)
#define CSL_IOMUX_UART0_RTSN_CFG_REG_INP_INV_SEL_SHIFT                         (0x00000014U)
#define CSL_IOMUX_UART0_RTSN_CFG_REG_INP_INV_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_UART0_RTSN_CFG_REG_INP_INV_SEL_MAX                           (0x00000001U)

#define CSL_IOMUX_UART0_RTSN_CFG_REG_ICSSM_GPIO_SEL_MASK                       (0x00200000U)
#define CSL_IOMUX_UART0_RTSN_CFG_REG_ICSSM_GPIO_SEL_SHIFT                      (0x00000015U)
#define CSL_IOMUX_UART0_RTSN_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                   (0x00000000U)
#define CSL_IOMUX_UART0_RTSN_CFG_REG_ICSSM_GPIO_SEL_MAX                        (0x00000001U)

#define CSL_IOMUX_UART0_RTSN_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                  (0x00400000U)
#define CSL_IOMUX_UART0_RTSN_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                 (0x00000016U)
#define CSL_IOMUX_UART0_RTSN_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL              (0x00000000U)
#define CSL_IOMUX_UART0_RTSN_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                   (0x00000001U)

#define CSL_IOMUX_UART0_RTSN_CFG_REG_HSMODE_MASK                               (0x40000000U)
#define CSL_IOMUX_UART0_RTSN_CFG_REG_HSMODE_SHIFT                              (0x0000001EU)
#define CSL_IOMUX_UART0_RTSN_CFG_REG_HSMODE_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_UART0_RTSN_CFG_REG_HSMODE_MAX                                (0x00000001U)

#define CSL_IOMUX_UART0_RTSN_CFG_REG_HSMASTER_MASK                             (0x80000000U)
#define CSL_IOMUX_UART0_RTSN_CFG_REG_HSMASTER_SHIFT                            (0x0000001FU)
#define CSL_IOMUX_UART0_RTSN_CFG_REG_HSMASTER_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_UART0_RTSN_CFG_REG_HSMASTER_MAX                              (0x00000001U)

#define CSL_IOMUX_UART0_RTSN_CFG_REG_RESETVAL                                  (0x000005F7U)

/* UART0_CTSN_CFG_REG */

#define CSL_IOMUX_UART0_CTSN_CFG_REG_FUNC_SEL_MASK                             (0x0000000FU)
#define CSL_IOMUX_UART0_CTSN_CFG_REG_FUNC_SEL_SHIFT                            (0x00000000U)
#define CSL_IOMUX_UART0_CTSN_CFG_REG_FUNC_SEL_RESETVAL                         (0x00000007U)
#define CSL_IOMUX_UART0_CTSN_CFG_REG_FUNC_SEL_MAX                              (0x0000000FU)

#define CSL_IOMUX_UART0_CTSN_CFG_REG_IE_OVERRIDE_CTRL_MASK                     (0x00000010U)
#define CSL_IOMUX_UART0_CTSN_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                    (0x00000004U)
#define CSL_IOMUX_UART0_CTSN_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_IOMUX_UART0_CTSN_CFG_REG_IE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_IOMUX_UART0_CTSN_CFG_REG_IE_OVERRIDE_MASK                          (0x00000020U)
#define CSL_IOMUX_UART0_CTSN_CFG_REG_IE_OVERRIDE_SHIFT                         (0x00000005U)
#define CSL_IOMUX_UART0_CTSN_CFG_REG_IE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_IOMUX_UART0_CTSN_CFG_REG_IE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_IOMUX_UART0_CTSN_CFG_REG_OE_OVERRIDE_CTRL_MASK                     (0x00000040U)
#define CSL_IOMUX_UART0_CTSN_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                    (0x00000006U)
#define CSL_IOMUX_UART0_CTSN_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_IOMUX_UART0_CTSN_CFG_REG_OE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_IOMUX_UART0_CTSN_CFG_REG_OE_OVERRIDE_MASK                          (0x00000080U)
#define CSL_IOMUX_UART0_CTSN_CFG_REG_OE_OVERRIDE_SHIFT                         (0x00000007U)
#define CSL_IOMUX_UART0_CTSN_CFG_REG_OE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_IOMUX_UART0_CTSN_CFG_REG_OE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_IOMUX_UART0_CTSN_CFG_REG_PI_MASK                                   (0x00000100U)
#define CSL_IOMUX_UART0_CTSN_CFG_REG_PI_SHIFT                                  (0x00000008U)
#define CSL_IOMUX_UART0_CTSN_CFG_REG_PI_RESETVAL                               (0x00000001U)
#define CSL_IOMUX_UART0_CTSN_CFG_REG_PI_MAX                                    (0x00000001U)

#define CSL_IOMUX_UART0_CTSN_CFG_REG_PUPDSEL_MASK                              (0x00000200U)
#define CSL_IOMUX_UART0_CTSN_CFG_REG_PUPDSEL_SHIFT                             (0x00000009U)
#define CSL_IOMUX_UART0_CTSN_CFG_REG_PUPDSEL_RESETVAL                          (0x00000000U)
#define CSL_IOMUX_UART0_CTSN_CFG_REG_PUPDSEL_MAX                               (0x00000001U)

#define CSL_IOMUX_UART0_CTSN_CFG_REG_SC1_MASK                                  (0x00000400U)
#define CSL_IOMUX_UART0_CTSN_CFG_REG_SC1_SHIFT                                 (0x0000000AU)
#define CSL_IOMUX_UART0_CTSN_CFG_REG_SC1_RESETVAL                              (0x00000001U)
#define CSL_IOMUX_UART0_CTSN_CFG_REG_SC1_MAX                                   (0x00000001U)

#define CSL_IOMUX_UART0_CTSN_CFG_REG_GPIO_SEL_MASK                             (0x00030000U)
#define CSL_IOMUX_UART0_CTSN_CFG_REG_GPIO_SEL_SHIFT                            (0x00000010U)
#define CSL_IOMUX_UART0_CTSN_CFG_REG_GPIO_SEL_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_UART0_CTSN_CFG_REG_GPIO_SEL_MAX                              (0x00000003U)

#define CSL_IOMUX_UART0_CTSN_CFG_REG_QUAL_SEL_MASK                             (0x000C0000U)
#define CSL_IOMUX_UART0_CTSN_CFG_REG_QUAL_SEL_SHIFT                            (0x00000012U)
#define CSL_IOMUX_UART0_CTSN_CFG_REG_QUAL_SEL_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_UART0_CTSN_CFG_REG_QUAL_SEL_MAX                              (0x00000003U)

#define CSL_IOMUX_UART0_CTSN_CFG_REG_INP_INV_SEL_MASK                          (0x00100000U)
#define CSL_IOMUX_UART0_CTSN_CFG_REG_INP_INV_SEL_SHIFT                         (0x00000014U)
#define CSL_IOMUX_UART0_CTSN_CFG_REG_INP_INV_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_UART0_CTSN_CFG_REG_INP_INV_SEL_MAX                           (0x00000001U)

#define CSL_IOMUX_UART0_CTSN_CFG_REG_ICSSM_GPIO_SEL_MASK                       (0x00200000U)
#define CSL_IOMUX_UART0_CTSN_CFG_REG_ICSSM_GPIO_SEL_SHIFT                      (0x00000015U)
#define CSL_IOMUX_UART0_CTSN_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                   (0x00000000U)
#define CSL_IOMUX_UART0_CTSN_CFG_REG_ICSSM_GPIO_SEL_MAX                        (0x00000001U)

#define CSL_IOMUX_UART0_CTSN_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                  (0x00400000U)
#define CSL_IOMUX_UART0_CTSN_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                 (0x00000016U)
#define CSL_IOMUX_UART0_CTSN_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL              (0x00000000U)
#define CSL_IOMUX_UART0_CTSN_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                   (0x00000001U)

#define CSL_IOMUX_UART0_CTSN_CFG_REG_HSMODE_MASK                               (0x40000000U)
#define CSL_IOMUX_UART0_CTSN_CFG_REG_HSMODE_SHIFT                              (0x0000001EU)
#define CSL_IOMUX_UART0_CTSN_CFG_REG_HSMODE_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_UART0_CTSN_CFG_REG_HSMODE_MAX                                (0x00000001U)

#define CSL_IOMUX_UART0_CTSN_CFG_REG_HSMASTER_MASK                             (0x80000000U)
#define CSL_IOMUX_UART0_CTSN_CFG_REG_HSMASTER_SHIFT                            (0x0000001FU)
#define CSL_IOMUX_UART0_CTSN_CFG_REG_HSMASTER_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_UART0_CTSN_CFG_REG_HSMASTER_MAX                              (0x00000001U)

#define CSL_IOMUX_UART0_CTSN_CFG_REG_RESETVAL                                  (0x000005F7U)

/* UART0_RXD_CFG_REG */

#define CSL_IOMUX_UART0_RXD_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_IOMUX_UART0_RXD_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_IOMUX_UART0_RXD_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000007U)
#define CSL_IOMUX_UART0_RXD_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_IOMUX_UART0_RXD_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_IOMUX_UART0_RXD_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_IOMUX_UART0_RXD_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_IOMUX_UART0_RXD_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_IOMUX_UART0_RXD_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_IOMUX_UART0_RXD_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_IOMUX_UART0_RXD_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_IOMUX_UART0_RXD_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_IOMUX_UART0_RXD_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_IOMUX_UART0_RXD_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_IOMUX_UART0_RXD_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_IOMUX_UART0_RXD_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_IOMUX_UART0_RXD_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_IOMUX_UART0_RXD_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_IOMUX_UART0_RXD_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_IOMUX_UART0_RXD_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_IOMUX_UART0_RXD_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_IOMUX_UART0_RXD_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_IOMUX_UART0_RXD_CFG_REG_PI_RESETVAL                                (0x00000001U)
#define CSL_IOMUX_UART0_RXD_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_IOMUX_UART0_RXD_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_IOMUX_UART0_RXD_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_IOMUX_UART0_RXD_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_UART0_RXD_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_IOMUX_UART0_RXD_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_IOMUX_UART0_RXD_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_IOMUX_UART0_RXD_CFG_REG_SC1_RESETVAL                               (0x00000001U)
#define CSL_IOMUX_UART0_RXD_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_IOMUX_UART0_RXD_CFG_REG_GPIO_SEL_MASK                              (0x00030000U)
#define CSL_IOMUX_UART0_RXD_CFG_REG_GPIO_SEL_SHIFT                             (0x00000010U)
#define CSL_IOMUX_UART0_RXD_CFG_REG_GPIO_SEL_RESETVAL                          (0x00000000U)
#define CSL_IOMUX_UART0_RXD_CFG_REG_GPIO_SEL_MAX                               (0x00000003U)

#define CSL_IOMUX_UART0_RXD_CFG_REG_QUAL_SEL_MASK                              (0x000C0000U)
#define CSL_IOMUX_UART0_RXD_CFG_REG_QUAL_SEL_SHIFT                             (0x00000012U)
#define CSL_IOMUX_UART0_RXD_CFG_REG_QUAL_SEL_RESETVAL                          (0x00000000U)
#define CSL_IOMUX_UART0_RXD_CFG_REG_QUAL_SEL_MAX                               (0x00000003U)

#define CSL_IOMUX_UART0_RXD_CFG_REG_INP_INV_SEL_MASK                           (0x00100000U)
#define CSL_IOMUX_UART0_RXD_CFG_REG_INP_INV_SEL_SHIFT                          (0x00000014U)
#define CSL_IOMUX_UART0_RXD_CFG_REG_INP_INV_SEL_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_UART0_RXD_CFG_REG_INP_INV_SEL_MAX                            (0x00000001U)

#define CSL_IOMUX_UART0_RXD_CFG_REG_ICSSM_GPIO_SEL_MASK                        (0x00200000U)
#define CSL_IOMUX_UART0_RXD_CFG_REG_ICSSM_GPIO_SEL_SHIFT                       (0x00000015U)
#define CSL_IOMUX_UART0_RXD_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                    (0x00000000U)
#define CSL_IOMUX_UART0_RXD_CFG_REG_ICSSM_GPIO_SEL_MAX                         (0x00000001U)

#define CSL_IOMUX_UART0_RXD_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                   (0x00400000U)
#define CSL_IOMUX_UART0_RXD_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                  (0x00000016U)
#define CSL_IOMUX_UART0_RXD_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL               (0x00000000U)
#define CSL_IOMUX_UART0_RXD_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                    (0x00000001U)

#define CSL_IOMUX_UART0_RXD_CFG_REG_HSMODE_MASK                                (0x40000000U)
#define CSL_IOMUX_UART0_RXD_CFG_REG_HSMODE_SHIFT                               (0x0000001EU)
#define CSL_IOMUX_UART0_RXD_CFG_REG_HSMODE_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_UART0_RXD_CFG_REG_HSMODE_MAX                                 (0x00000001U)

#define CSL_IOMUX_UART0_RXD_CFG_REG_HSMASTER_MASK                              (0x80000000U)
#define CSL_IOMUX_UART0_RXD_CFG_REG_HSMASTER_SHIFT                             (0x0000001FU)
#define CSL_IOMUX_UART0_RXD_CFG_REG_HSMASTER_RESETVAL                          (0x00000000U)
#define CSL_IOMUX_UART0_RXD_CFG_REG_HSMASTER_MAX                               (0x00000001U)

#define CSL_IOMUX_UART0_RXD_CFG_REG_RESETVAL                                   (0x000005F7U)

/* UART0_TXD_CFG_REG */

#define CSL_IOMUX_UART0_TXD_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_IOMUX_UART0_TXD_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_IOMUX_UART0_TXD_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000007U)
#define CSL_IOMUX_UART0_TXD_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_IOMUX_UART0_TXD_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_IOMUX_UART0_TXD_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_IOMUX_UART0_TXD_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_IOMUX_UART0_TXD_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_IOMUX_UART0_TXD_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_IOMUX_UART0_TXD_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_IOMUX_UART0_TXD_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_IOMUX_UART0_TXD_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_IOMUX_UART0_TXD_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_IOMUX_UART0_TXD_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_IOMUX_UART0_TXD_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_IOMUX_UART0_TXD_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_IOMUX_UART0_TXD_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_IOMUX_UART0_TXD_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_IOMUX_UART0_TXD_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_IOMUX_UART0_TXD_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_IOMUX_UART0_TXD_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_IOMUX_UART0_TXD_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_IOMUX_UART0_TXD_CFG_REG_PI_RESETVAL                                (0x00000001U)
#define CSL_IOMUX_UART0_TXD_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_IOMUX_UART0_TXD_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_IOMUX_UART0_TXD_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_IOMUX_UART0_TXD_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_UART0_TXD_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_IOMUX_UART0_TXD_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_IOMUX_UART0_TXD_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_IOMUX_UART0_TXD_CFG_REG_SC1_RESETVAL                               (0x00000001U)
#define CSL_IOMUX_UART0_TXD_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_IOMUX_UART0_TXD_CFG_REG_GPIO_SEL_MASK                              (0x00030000U)
#define CSL_IOMUX_UART0_TXD_CFG_REG_GPIO_SEL_SHIFT                             (0x00000010U)
#define CSL_IOMUX_UART0_TXD_CFG_REG_GPIO_SEL_RESETVAL                          (0x00000000U)
#define CSL_IOMUX_UART0_TXD_CFG_REG_GPIO_SEL_MAX                               (0x00000003U)

#define CSL_IOMUX_UART0_TXD_CFG_REG_QUAL_SEL_MASK                              (0x000C0000U)
#define CSL_IOMUX_UART0_TXD_CFG_REG_QUAL_SEL_SHIFT                             (0x00000012U)
#define CSL_IOMUX_UART0_TXD_CFG_REG_QUAL_SEL_RESETVAL                          (0x00000000U)
#define CSL_IOMUX_UART0_TXD_CFG_REG_QUAL_SEL_MAX                               (0x00000003U)

#define CSL_IOMUX_UART0_TXD_CFG_REG_INP_INV_SEL_MASK                           (0x00100000U)
#define CSL_IOMUX_UART0_TXD_CFG_REG_INP_INV_SEL_SHIFT                          (0x00000014U)
#define CSL_IOMUX_UART0_TXD_CFG_REG_INP_INV_SEL_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_UART0_TXD_CFG_REG_INP_INV_SEL_MAX                            (0x00000001U)

#define CSL_IOMUX_UART0_TXD_CFG_REG_ICSSM_GPIO_SEL_MASK                        (0x00200000U)
#define CSL_IOMUX_UART0_TXD_CFG_REG_ICSSM_GPIO_SEL_SHIFT                       (0x00000015U)
#define CSL_IOMUX_UART0_TXD_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                    (0x00000000U)
#define CSL_IOMUX_UART0_TXD_CFG_REG_ICSSM_GPIO_SEL_MAX                         (0x00000001U)

#define CSL_IOMUX_UART0_TXD_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                   (0x00400000U)
#define CSL_IOMUX_UART0_TXD_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                  (0x00000016U)
#define CSL_IOMUX_UART0_TXD_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL               (0x00000000U)
#define CSL_IOMUX_UART0_TXD_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                    (0x00000001U)

#define CSL_IOMUX_UART0_TXD_CFG_REG_HSMODE_MASK                                (0x40000000U)
#define CSL_IOMUX_UART0_TXD_CFG_REG_HSMODE_SHIFT                               (0x0000001EU)
#define CSL_IOMUX_UART0_TXD_CFG_REG_HSMODE_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_UART0_TXD_CFG_REG_HSMODE_MAX                                 (0x00000001U)

#define CSL_IOMUX_UART0_TXD_CFG_REG_HSMASTER_MASK                              (0x80000000U)
#define CSL_IOMUX_UART0_TXD_CFG_REG_HSMASTER_SHIFT                             (0x0000001FU)
#define CSL_IOMUX_UART0_TXD_CFG_REG_HSMASTER_RESETVAL                          (0x00000000U)
#define CSL_IOMUX_UART0_TXD_CFG_REG_HSMASTER_MAX                               (0x00000001U)

#define CSL_IOMUX_UART0_TXD_CFG_REG_RESETVAL                                   (0x000005F7U)

/* RGMII1_RXC_CFG_REG */

#define CSL_IOMUX_RGMII1_RXC_CFG_REG_FUNC_SEL_MASK                             (0x0000000FU)
#define CSL_IOMUX_RGMII1_RXC_CFG_REG_FUNC_SEL_SHIFT                            (0x00000000U)
#define CSL_IOMUX_RGMII1_RXC_CFG_REG_FUNC_SEL_RESETVAL                         (0x00000007U)
#define CSL_IOMUX_RGMII1_RXC_CFG_REG_FUNC_SEL_MAX                              (0x0000000FU)

#define CSL_IOMUX_RGMII1_RXC_CFG_REG_IE_OVERRIDE_CTRL_MASK                     (0x00000010U)
#define CSL_IOMUX_RGMII1_RXC_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                    (0x00000004U)
#define CSL_IOMUX_RGMII1_RXC_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_IOMUX_RGMII1_RXC_CFG_REG_IE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_IOMUX_RGMII1_RXC_CFG_REG_IE_OVERRIDE_MASK                          (0x00000020U)
#define CSL_IOMUX_RGMII1_RXC_CFG_REG_IE_OVERRIDE_SHIFT                         (0x00000005U)
#define CSL_IOMUX_RGMII1_RXC_CFG_REG_IE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_IOMUX_RGMII1_RXC_CFG_REG_IE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_IOMUX_RGMII1_RXC_CFG_REG_OE_OVERRIDE_CTRL_MASK                     (0x00000040U)
#define CSL_IOMUX_RGMII1_RXC_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                    (0x00000006U)
#define CSL_IOMUX_RGMII1_RXC_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_IOMUX_RGMII1_RXC_CFG_REG_OE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_IOMUX_RGMII1_RXC_CFG_REG_OE_OVERRIDE_MASK                          (0x00000080U)
#define CSL_IOMUX_RGMII1_RXC_CFG_REG_OE_OVERRIDE_SHIFT                         (0x00000007U)
#define CSL_IOMUX_RGMII1_RXC_CFG_REG_OE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_IOMUX_RGMII1_RXC_CFG_REG_OE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_IOMUX_RGMII1_RXC_CFG_REG_PI_MASK                                   (0x00000100U)
#define CSL_IOMUX_RGMII1_RXC_CFG_REG_PI_SHIFT                                  (0x00000008U)
#define CSL_IOMUX_RGMII1_RXC_CFG_REG_PI_RESETVAL                               (0x00000001U)
#define CSL_IOMUX_RGMII1_RXC_CFG_REG_PI_MAX                                    (0x00000001U)

#define CSL_IOMUX_RGMII1_RXC_CFG_REG_PUPDSEL_MASK                              (0x00000200U)
#define CSL_IOMUX_RGMII1_RXC_CFG_REG_PUPDSEL_SHIFT                             (0x00000009U)
#define CSL_IOMUX_RGMII1_RXC_CFG_REG_PUPDSEL_RESETVAL                          (0x00000000U)
#define CSL_IOMUX_RGMII1_RXC_CFG_REG_PUPDSEL_MAX                               (0x00000001U)

#define CSL_IOMUX_RGMII1_RXC_CFG_REG_SC1_MASK                                  (0x00000400U)
#define CSL_IOMUX_RGMII1_RXC_CFG_REG_SC1_SHIFT                                 (0x0000000AU)
#define CSL_IOMUX_RGMII1_RXC_CFG_REG_SC1_RESETVAL                              (0x00000001U)
#define CSL_IOMUX_RGMII1_RXC_CFG_REG_SC1_MAX                                   (0x00000001U)

#define CSL_IOMUX_RGMII1_RXC_CFG_REG_GPIO_SEL_MASK                             (0x00030000U)
#define CSL_IOMUX_RGMII1_RXC_CFG_REG_GPIO_SEL_SHIFT                            (0x00000010U)
#define CSL_IOMUX_RGMII1_RXC_CFG_REG_GPIO_SEL_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_RGMII1_RXC_CFG_REG_GPIO_SEL_MAX                              (0x00000003U)

#define CSL_IOMUX_RGMII1_RXC_CFG_REG_QUAL_SEL_MASK                             (0x000C0000U)
#define CSL_IOMUX_RGMII1_RXC_CFG_REG_QUAL_SEL_SHIFT                            (0x00000012U)
#define CSL_IOMUX_RGMII1_RXC_CFG_REG_QUAL_SEL_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_RGMII1_RXC_CFG_REG_QUAL_SEL_MAX                              (0x00000003U)

#define CSL_IOMUX_RGMII1_RXC_CFG_REG_INP_INV_SEL_MASK                          (0x00100000U)
#define CSL_IOMUX_RGMII1_RXC_CFG_REG_INP_INV_SEL_SHIFT                         (0x00000014U)
#define CSL_IOMUX_RGMII1_RXC_CFG_REG_INP_INV_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_RGMII1_RXC_CFG_REG_INP_INV_SEL_MAX                           (0x00000001U)

#define CSL_IOMUX_RGMII1_RXC_CFG_REG_ICSSM_GPIO_SEL_MASK                       (0x00200000U)
#define CSL_IOMUX_RGMII1_RXC_CFG_REG_ICSSM_GPIO_SEL_SHIFT                      (0x00000015U)
#define CSL_IOMUX_RGMII1_RXC_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                   (0x00000000U)
#define CSL_IOMUX_RGMII1_RXC_CFG_REG_ICSSM_GPIO_SEL_MAX                        (0x00000001U)

#define CSL_IOMUX_RGMII1_RXC_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                  (0x00400000U)
#define CSL_IOMUX_RGMII1_RXC_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                 (0x00000016U)
#define CSL_IOMUX_RGMII1_RXC_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL              (0x00000000U)
#define CSL_IOMUX_RGMII1_RXC_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                   (0x00000001U)

#define CSL_IOMUX_RGMII1_RXC_CFG_REG_HSMODE_MASK                               (0x40000000U)
#define CSL_IOMUX_RGMII1_RXC_CFG_REG_HSMODE_SHIFT                              (0x0000001EU)
#define CSL_IOMUX_RGMII1_RXC_CFG_REG_HSMODE_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_RGMII1_RXC_CFG_REG_HSMODE_MAX                                (0x00000001U)

#define CSL_IOMUX_RGMII1_RXC_CFG_REG_HSMASTER_MASK                             (0x80000000U)
#define CSL_IOMUX_RGMII1_RXC_CFG_REG_HSMASTER_SHIFT                            (0x0000001FU)
#define CSL_IOMUX_RGMII1_RXC_CFG_REG_HSMASTER_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_RGMII1_RXC_CFG_REG_HSMASTER_MAX                              (0x00000001U)

#define CSL_IOMUX_RGMII1_RXC_CFG_REG_RESETVAL                                  (0x000005F7U)

/* RGMII1_RX_CTL_CFG_REG */

#define CSL_IOMUX_RGMII1_RX_CTL_CFG_REG_FUNC_SEL_MASK                          (0x0000000FU)
#define CSL_IOMUX_RGMII1_RX_CTL_CFG_REG_FUNC_SEL_SHIFT                         (0x00000000U)
#define CSL_IOMUX_RGMII1_RX_CTL_CFG_REG_FUNC_SEL_RESETVAL                      (0x00000007U)
#define CSL_IOMUX_RGMII1_RX_CTL_CFG_REG_FUNC_SEL_MAX                           (0x0000000FU)

#define CSL_IOMUX_RGMII1_RX_CTL_CFG_REG_IE_OVERRIDE_CTRL_MASK                  (0x00000010U)
#define CSL_IOMUX_RGMII1_RX_CTL_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                 (0x00000004U)
#define CSL_IOMUX_RGMII1_RX_CTL_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL              (0x00000001U)
#define CSL_IOMUX_RGMII1_RX_CTL_CFG_REG_IE_OVERRIDE_CTRL_MAX                   (0x00000001U)

#define CSL_IOMUX_RGMII1_RX_CTL_CFG_REG_IE_OVERRIDE_MASK                       (0x00000020U)
#define CSL_IOMUX_RGMII1_RX_CTL_CFG_REG_IE_OVERRIDE_SHIFT                      (0x00000005U)
#define CSL_IOMUX_RGMII1_RX_CTL_CFG_REG_IE_OVERRIDE_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_RGMII1_RX_CTL_CFG_REG_IE_OVERRIDE_MAX                        (0x00000001U)

#define CSL_IOMUX_RGMII1_RX_CTL_CFG_REG_OE_OVERRIDE_CTRL_MASK                  (0x00000040U)
#define CSL_IOMUX_RGMII1_RX_CTL_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                 (0x00000006U)
#define CSL_IOMUX_RGMII1_RX_CTL_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL              (0x00000001U)
#define CSL_IOMUX_RGMII1_RX_CTL_CFG_REG_OE_OVERRIDE_CTRL_MAX                   (0x00000001U)

#define CSL_IOMUX_RGMII1_RX_CTL_CFG_REG_OE_OVERRIDE_MASK                       (0x00000080U)
#define CSL_IOMUX_RGMII1_RX_CTL_CFG_REG_OE_OVERRIDE_SHIFT                      (0x00000007U)
#define CSL_IOMUX_RGMII1_RX_CTL_CFG_REG_OE_OVERRIDE_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_RGMII1_RX_CTL_CFG_REG_OE_OVERRIDE_MAX                        (0x00000001U)

#define CSL_IOMUX_RGMII1_RX_CTL_CFG_REG_PI_MASK                                (0x00000100U)
#define CSL_IOMUX_RGMII1_RX_CTL_CFG_REG_PI_SHIFT                               (0x00000008U)
#define CSL_IOMUX_RGMII1_RX_CTL_CFG_REG_PI_RESETVAL                            (0x00000001U)
#define CSL_IOMUX_RGMII1_RX_CTL_CFG_REG_PI_MAX                                 (0x00000001U)

#define CSL_IOMUX_RGMII1_RX_CTL_CFG_REG_PUPDSEL_MASK                           (0x00000200U)
#define CSL_IOMUX_RGMII1_RX_CTL_CFG_REG_PUPDSEL_SHIFT                          (0x00000009U)
#define CSL_IOMUX_RGMII1_RX_CTL_CFG_REG_PUPDSEL_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_RGMII1_RX_CTL_CFG_REG_PUPDSEL_MAX                            (0x00000001U)

#define CSL_IOMUX_RGMII1_RX_CTL_CFG_REG_SC1_MASK                               (0x00000400U)
#define CSL_IOMUX_RGMII1_RX_CTL_CFG_REG_SC1_SHIFT                              (0x0000000AU)
#define CSL_IOMUX_RGMII1_RX_CTL_CFG_REG_SC1_RESETVAL                           (0x00000001U)
#define CSL_IOMUX_RGMII1_RX_CTL_CFG_REG_SC1_MAX                                (0x00000001U)

#define CSL_IOMUX_RGMII1_RX_CTL_CFG_REG_GPIO_SEL_MASK                          (0x00030000U)
#define CSL_IOMUX_RGMII1_RX_CTL_CFG_REG_GPIO_SEL_SHIFT                         (0x00000010U)
#define CSL_IOMUX_RGMII1_RX_CTL_CFG_REG_GPIO_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_RGMII1_RX_CTL_CFG_REG_GPIO_SEL_MAX                           (0x00000003U)

#define CSL_IOMUX_RGMII1_RX_CTL_CFG_REG_QUAL_SEL_MASK                          (0x000C0000U)
#define CSL_IOMUX_RGMII1_RX_CTL_CFG_REG_QUAL_SEL_SHIFT                         (0x00000012U)
#define CSL_IOMUX_RGMII1_RX_CTL_CFG_REG_QUAL_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_RGMII1_RX_CTL_CFG_REG_QUAL_SEL_MAX                           (0x00000003U)

#define CSL_IOMUX_RGMII1_RX_CTL_CFG_REG_INP_INV_SEL_MASK                       (0x00100000U)
#define CSL_IOMUX_RGMII1_RX_CTL_CFG_REG_INP_INV_SEL_SHIFT                      (0x00000014U)
#define CSL_IOMUX_RGMII1_RX_CTL_CFG_REG_INP_INV_SEL_RESETVAL                   (0x00000000U)
#define CSL_IOMUX_RGMII1_RX_CTL_CFG_REG_INP_INV_SEL_MAX                        (0x00000001U)

#define CSL_IOMUX_RGMII1_RX_CTL_CFG_REG_ICSSM_GPIO_SEL_MASK                    (0x00200000U)
#define CSL_IOMUX_RGMII1_RX_CTL_CFG_REG_ICSSM_GPIO_SEL_SHIFT                   (0x00000015U)
#define CSL_IOMUX_RGMII1_RX_CTL_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                (0x00000000U)
#define CSL_IOMUX_RGMII1_RX_CTL_CFG_REG_ICSSM_GPIO_SEL_MAX                     (0x00000001U)

#define CSL_IOMUX_RGMII1_RX_CTL_CFG_REG_SAFETY_OVERRIDE_SEL_MASK               (0x00400000U)
#define CSL_IOMUX_RGMII1_RX_CTL_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT              (0x00000016U)
#define CSL_IOMUX_RGMII1_RX_CTL_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL           (0x00000000U)
#define CSL_IOMUX_RGMII1_RX_CTL_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                (0x00000001U)

#define CSL_IOMUX_RGMII1_RX_CTL_CFG_REG_HSMODE_MASK                            (0x40000000U)
#define CSL_IOMUX_RGMII1_RX_CTL_CFG_REG_HSMODE_SHIFT                           (0x0000001EU)
#define CSL_IOMUX_RGMII1_RX_CTL_CFG_REG_HSMODE_RESETVAL                        (0x00000000U)
#define CSL_IOMUX_RGMII1_RX_CTL_CFG_REG_HSMODE_MAX                             (0x00000001U)

#define CSL_IOMUX_RGMII1_RX_CTL_CFG_REG_HSMASTER_MASK                          (0x80000000U)
#define CSL_IOMUX_RGMII1_RX_CTL_CFG_REG_HSMASTER_SHIFT                         (0x0000001FU)
#define CSL_IOMUX_RGMII1_RX_CTL_CFG_REG_HSMASTER_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_RGMII1_RX_CTL_CFG_REG_HSMASTER_MAX                           (0x00000001U)

#define CSL_IOMUX_RGMII1_RX_CTL_CFG_REG_RESETVAL                               (0x000005F7U)

/* RGMII1_RD0_CFG_REG */

#define CSL_IOMUX_RGMII1_RD0_CFG_REG_FUNC_SEL_MASK                             (0x0000000FU)
#define CSL_IOMUX_RGMII1_RD0_CFG_REG_FUNC_SEL_SHIFT                            (0x00000000U)
#define CSL_IOMUX_RGMII1_RD0_CFG_REG_FUNC_SEL_RESETVAL                         (0x00000007U)
#define CSL_IOMUX_RGMII1_RD0_CFG_REG_FUNC_SEL_MAX                              (0x0000000FU)

#define CSL_IOMUX_RGMII1_RD0_CFG_REG_IE_OVERRIDE_CTRL_MASK                     (0x00000010U)
#define CSL_IOMUX_RGMII1_RD0_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                    (0x00000004U)
#define CSL_IOMUX_RGMII1_RD0_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_IOMUX_RGMII1_RD0_CFG_REG_IE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_IOMUX_RGMII1_RD0_CFG_REG_IE_OVERRIDE_MASK                          (0x00000020U)
#define CSL_IOMUX_RGMII1_RD0_CFG_REG_IE_OVERRIDE_SHIFT                         (0x00000005U)
#define CSL_IOMUX_RGMII1_RD0_CFG_REG_IE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_IOMUX_RGMII1_RD0_CFG_REG_IE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_IOMUX_RGMII1_RD0_CFG_REG_OE_OVERRIDE_CTRL_MASK                     (0x00000040U)
#define CSL_IOMUX_RGMII1_RD0_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                    (0x00000006U)
#define CSL_IOMUX_RGMII1_RD0_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_IOMUX_RGMII1_RD0_CFG_REG_OE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_IOMUX_RGMII1_RD0_CFG_REG_OE_OVERRIDE_MASK                          (0x00000080U)
#define CSL_IOMUX_RGMII1_RD0_CFG_REG_OE_OVERRIDE_SHIFT                         (0x00000007U)
#define CSL_IOMUX_RGMII1_RD0_CFG_REG_OE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_IOMUX_RGMII1_RD0_CFG_REG_OE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_IOMUX_RGMII1_RD0_CFG_REG_PI_MASK                                   (0x00000100U)
#define CSL_IOMUX_RGMII1_RD0_CFG_REG_PI_SHIFT                                  (0x00000008U)
#define CSL_IOMUX_RGMII1_RD0_CFG_REG_PI_RESETVAL                               (0x00000001U)
#define CSL_IOMUX_RGMII1_RD0_CFG_REG_PI_MAX                                    (0x00000001U)

#define CSL_IOMUX_RGMII1_RD0_CFG_REG_PUPDSEL_MASK                              (0x00000200U)
#define CSL_IOMUX_RGMII1_RD0_CFG_REG_PUPDSEL_SHIFT                             (0x00000009U)
#define CSL_IOMUX_RGMII1_RD0_CFG_REG_PUPDSEL_RESETVAL                          (0x00000000U)
#define CSL_IOMUX_RGMII1_RD0_CFG_REG_PUPDSEL_MAX                               (0x00000001U)

#define CSL_IOMUX_RGMII1_RD0_CFG_REG_SC1_MASK                                  (0x00000400U)
#define CSL_IOMUX_RGMII1_RD0_CFG_REG_SC1_SHIFT                                 (0x0000000AU)
#define CSL_IOMUX_RGMII1_RD0_CFG_REG_SC1_RESETVAL                              (0x00000001U)
#define CSL_IOMUX_RGMII1_RD0_CFG_REG_SC1_MAX                                   (0x00000001U)

#define CSL_IOMUX_RGMII1_RD0_CFG_REG_GPIO_SEL_MASK                             (0x00030000U)
#define CSL_IOMUX_RGMII1_RD0_CFG_REG_GPIO_SEL_SHIFT                            (0x00000010U)
#define CSL_IOMUX_RGMII1_RD0_CFG_REG_GPIO_SEL_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_RGMII1_RD0_CFG_REG_GPIO_SEL_MAX                              (0x00000003U)

#define CSL_IOMUX_RGMII1_RD0_CFG_REG_QUAL_SEL_MASK                             (0x000C0000U)
#define CSL_IOMUX_RGMII1_RD0_CFG_REG_QUAL_SEL_SHIFT                            (0x00000012U)
#define CSL_IOMUX_RGMII1_RD0_CFG_REG_QUAL_SEL_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_RGMII1_RD0_CFG_REG_QUAL_SEL_MAX                              (0x00000003U)

#define CSL_IOMUX_RGMII1_RD0_CFG_REG_INP_INV_SEL_MASK                          (0x00100000U)
#define CSL_IOMUX_RGMII1_RD0_CFG_REG_INP_INV_SEL_SHIFT                         (0x00000014U)
#define CSL_IOMUX_RGMII1_RD0_CFG_REG_INP_INV_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_RGMII1_RD0_CFG_REG_INP_INV_SEL_MAX                           (0x00000001U)

#define CSL_IOMUX_RGMII1_RD0_CFG_REG_ICSSM_GPIO_SEL_MASK                       (0x00200000U)
#define CSL_IOMUX_RGMII1_RD0_CFG_REG_ICSSM_GPIO_SEL_SHIFT                      (0x00000015U)
#define CSL_IOMUX_RGMII1_RD0_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                   (0x00000000U)
#define CSL_IOMUX_RGMII1_RD0_CFG_REG_ICSSM_GPIO_SEL_MAX                        (0x00000001U)

#define CSL_IOMUX_RGMII1_RD0_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                  (0x00400000U)
#define CSL_IOMUX_RGMII1_RD0_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                 (0x00000016U)
#define CSL_IOMUX_RGMII1_RD0_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL              (0x00000000U)
#define CSL_IOMUX_RGMII1_RD0_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                   (0x00000001U)

#define CSL_IOMUX_RGMII1_RD0_CFG_REG_HSMODE_MASK                               (0x40000000U)
#define CSL_IOMUX_RGMII1_RD0_CFG_REG_HSMODE_SHIFT                              (0x0000001EU)
#define CSL_IOMUX_RGMII1_RD0_CFG_REG_HSMODE_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_RGMII1_RD0_CFG_REG_HSMODE_MAX                                (0x00000001U)

#define CSL_IOMUX_RGMII1_RD0_CFG_REG_HSMASTER_MASK                             (0x80000000U)
#define CSL_IOMUX_RGMII1_RD0_CFG_REG_HSMASTER_SHIFT                            (0x0000001FU)
#define CSL_IOMUX_RGMII1_RD0_CFG_REG_HSMASTER_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_RGMII1_RD0_CFG_REG_HSMASTER_MAX                              (0x00000001U)

#define CSL_IOMUX_RGMII1_RD0_CFG_REG_RESETVAL                                  (0x000005F7U)

/* RGMII1_RD1_CFG_REG */

#define CSL_IOMUX_RGMII1_RD1_CFG_REG_FUNC_SEL_MASK                             (0x0000000FU)
#define CSL_IOMUX_RGMII1_RD1_CFG_REG_FUNC_SEL_SHIFT                            (0x00000000U)
#define CSL_IOMUX_RGMII1_RD1_CFG_REG_FUNC_SEL_RESETVAL                         (0x00000007U)
#define CSL_IOMUX_RGMII1_RD1_CFG_REG_FUNC_SEL_MAX                              (0x0000000FU)

#define CSL_IOMUX_RGMII1_RD1_CFG_REG_IE_OVERRIDE_CTRL_MASK                     (0x00000010U)
#define CSL_IOMUX_RGMII1_RD1_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                    (0x00000004U)
#define CSL_IOMUX_RGMII1_RD1_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_IOMUX_RGMII1_RD1_CFG_REG_IE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_IOMUX_RGMII1_RD1_CFG_REG_IE_OVERRIDE_MASK                          (0x00000020U)
#define CSL_IOMUX_RGMII1_RD1_CFG_REG_IE_OVERRIDE_SHIFT                         (0x00000005U)
#define CSL_IOMUX_RGMII1_RD1_CFG_REG_IE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_IOMUX_RGMII1_RD1_CFG_REG_IE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_IOMUX_RGMII1_RD1_CFG_REG_OE_OVERRIDE_CTRL_MASK                     (0x00000040U)
#define CSL_IOMUX_RGMII1_RD1_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                    (0x00000006U)
#define CSL_IOMUX_RGMII1_RD1_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_IOMUX_RGMII1_RD1_CFG_REG_OE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_IOMUX_RGMII1_RD1_CFG_REG_OE_OVERRIDE_MASK                          (0x00000080U)
#define CSL_IOMUX_RGMII1_RD1_CFG_REG_OE_OVERRIDE_SHIFT                         (0x00000007U)
#define CSL_IOMUX_RGMII1_RD1_CFG_REG_OE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_IOMUX_RGMII1_RD1_CFG_REG_OE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_IOMUX_RGMII1_RD1_CFG_REG_PI_MASK                                   (0x00000100U)
#define CSL_IOMUX_RGMII1_RD1_CFG_REG_PI_SHIFT                                  (0x00000008U)
#define CSL_IOMUX_RGMII1_RD1_CFG_REG_PI_RESETVAL                               (0x00000001U)
#define CSL_IOMUX_RGMII1_RD1_CFG_REG_PI_MAX                                    (0x00000001U)

#define CSL_IOMUX_RGMII1_RD1_CFG_REG_PUPDSEL_MASK                              (0x00000200U)
#define CSL_IOMUX_RGMII1_RD1_CFG_REG_PUPDSEL_SHIFT                             (0x00000009U)
#define CSL_IOMUX_RGMII1_RD1_CFG_REG_PUPDSEL_RESETVAL                          (0x00000000U)
#define CSL_IOMUX_RGMII1_RD1_CFG_REG_PUPDSEL_MAX                               (0x00000001U)

#define CSL_IOMUX_RGMII1_RD1_CFG_REG_SC1_MASK                                  (0x00000400U)
#define CSL_IOMUX_RGMII1_RD1_CFG_REG_SC1_SHIFT                                 (0x0000000AU)
#define CSL_IOMUX_RGMII1_RD1_CFG_REG_SC1_RESETVAL                              (0x00000001U)
#define CSL_IOMUX_RGMII1_RD1_CFG_REG_SC1_MAX                                   (0x00000001U)

#define CSL_IOMUX_RGMII1_RD1_CFG_REG_GPIO_SEL_MASK                             (0x00030000U)
#define CSL_IOMUX_RGMII1_RD1_CFG_REG_GPIO_SEL_SHIFT                            (0x00000010U)
#define CSL_IOMUX_RGMII1_RD1_CFG_REG_GPIO_SEL_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_RGMII1_RD1_CFG_REG_GPIO_SEL_MAX                              (0x00000003U)

#define CSL_IOMUX_RGMII1_RD1_CFG_REG_QUAL_SEL_MASK                             (0x000C0000U)
#define CSL_IOMUX_RGMII1_RD1_CFG_REG_QUAL_SEL_SHIFT                            (0x00000012U)
#define CSL_IOMUX_RGMII1_RD1_CFG_REG_QUAL_SEL_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_RGMII1_RD1_CFG_REG_QUAL_SEL_MAX                              (0x00000003U)

#define CSL_IOMUX_RGMII1_RD1_CFG_REG_INP_INV_SEL_MASK                          (0x00100000U)
#define CSL_IOMUX_RGMII1_RD1_CFG_REG_INP_INV_SEL_SHIFT                         (0x00000014U)
#define CSL_IOMUX_RGMII1_RD1_CFG_REG_INP_INV_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_RGMII1_RD1_CFG_REG_INP_INV_SEL_MAX                           (0x00000001U)

#define CSL_IOMUX_RGMII1_RD1_CFG_REG_ICSSM_GPIO_SEL_MASK                       (0x00200000U)
#define CSL_IOMUX_RGMII1_RD1_CFG_REG_ICSSM_GPIO_SEL_SHIFT                      (0x00000015U)
#define CSL_IOMUX_RGMII1_RD1_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                   (0x00000000U)
#define CSL_IOMUX_RGMII1_RD1_CFG_REG_ICSSM_GPIO_SEL_MAX                        (0x00000001U)

#define CSL_IOMUX_RGMII1_RD1_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                  (0x00400000U)
#define CSL_IOMUX_RGMII1_RD1_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                 (0x00000016U)
#define CSL_IOMUX_RGMII1_RD1_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL              (0x00000000U)
#define CSL_IOMUX_RGMII1_RD1_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                   (0x00000001U)

#define CSL_IOMUX_RGMII1_RD1_CFG_REG_HSMODE_MASK                               (0x40000000U)
#define CSL_IOMUX_RGMII1_RD1_CFG_REG_HSMODE_SHIFT                              (0x0000001EU)
#define CSL_IOMUX_RGMII1_RD1_CFG_REG_HSMODE_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_RGMII1_RD1_CFG_REG_HSMODE_MAX                                (0x00000001U)

#define CSL_IOMUX_RGMII1_RD1_CFG_REG_HSMASTER_MASK                             (0x80000000U)
#define CSL_IOMUX_RGMII1_RD1_CFG_REG_HSMASTER_SHIFT                            (0x0000001FU)
#define CSL_IOMUX_RGMII1_RD1_CFG_REG_HSMASTER_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_RGMII1_RD1_CFG_REG_HSMASTER_MAX                              (0x00000001U)

#define CSL_IOMUX_RGMII1_RD1_CFG_REG_RESETVAL                                  (0x000005F7U)

/* RGMII1_RD2_CFG_REG */

#define CSL_IOMUX_RGMII1_RD2_CFG_REG_FUNC_SEL_MASK                             (0x0000000FU)
#define CSL_IOMUX_RGMII1_RD2_CFG_REG_FUNC_SEL_SHIFT                            (0x00000000U)
#define CSL_IOMUX_RGMII1_RD2_CFG_REG_FUNC_SEL_RESETVAL                         (0x00000007U)
#define CSL_IOMUX_RGMII1_RD2_CFG_REG_FUNC_SEL_MAX                              (0x0000000FU)

#define CSL_IOMUX_RGMII1_RD2_CFG_REG_IE_OVERRIDE_CTRL_MASK                     (0x00000010U)
#define CSL_IOMUX_RGMII1_RD2_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                    (0x00000004U)
#define CSL_IOMUX_RGMII1_RD2_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_IOMUX_RGMII1_RD2_CFG_REG_IE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_IOMUX_RGMII1_RD2_CFG_REG_IE_OVERRIDE_MASK                          (0x00000020U)
#define CSL_IOMUX_RGMII1_RD2_CFG_REG_IE_OVERRIDE_SHIFT                         (0x00000005U)
#define CSL_IOMUX_RGMII1_RD2_CFG_REG_IE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_IOMUX_RGMII1_RD2_CFG_REG_IE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_IOMUX_RGMII1_RD2_CFG_REG_OE_OVERRIDE_CTRL_MASK                     (0x00000040U)
#define CSL_IOMUX_RGMII1_RD2_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                    (0x00000006U)
#define CSL_IOMUX_RGMII1_RD2_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_IOMUX_RGMII1_RD2_CFG_REG_OE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_IOMUX_RGMII1_RD2_CFG_REG_OE_OVERRIDE_MASK                          (0x00000080U)
#define CSL_IOMUX_RGMII1_RD2_CFG_REG_OE_OVERRIDE_SHIFT                         (0x00000007U)
#define CSL_IOMUX_RGMII1_RD2_CFG_REG_OE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_IOMUX_RGMII1_RD2_CFG_REG_OE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_IOMUX_RGMII1_RD2_CFG_REG_PI_MASK                                   (0x00000100U)
#define CSL_IOMUX_RGMII1_RD2_CFG_REG_PI_SHIFT                                  (0x00000008U)
#define CSL_IOMUX_RGMII1_RD2_CFG_REG_PI_RESETVAL                               (0x00000001U)
#define CSL_IOMUX_RGMII1_RD2_CFG_REG_PI_MAX                                    (0x00000001U)

#define CSL_IOMUX_RGMII1_RD2_CFG_REG_PUPDSEL_MASK                              (0x00000200U)
#define CSL_IOMUX_RGMII1_RD2_CFG_REG_PUPDSEL_SHIFT                             (0x00000009U)
#define CSL_IOMUX_RGMII1_RD2_CFG_REG_PUPDSEL_RESETVAL                          (0x00000000U)
#define CSL_IOMUX_RGMII1_RD2_CFG_REG_PUPDSEL_MAX                               (0x00000001U)

#define CSL_IOMUX_RGMII1_RD2_CFG_REG_SC1_MASK                                  (0x00000400U)
#define CSL_IOMUX_RGMII1_RD2_CFG_REG_SC1_SHIFT                                 (0x0000000AU)
#define CSL_IOMUX_RGMII1_RD2_CFG_REG_SC1_RESETVAL                              (0x00000001U)
#define CSL_IOMUX_RGMII1_RD2_CFG_REG_SC1_MAX                                   (0x00000001U)

#define CSL_IOMUX_RGMII1_RD2_CFG_REG_GPIO_SEL_MASK                             (0x00030000U)
#define CSL_IOMUX_RGMII1_RD2_CFG_REG_GPIO_SEL_SHIFT                            (0x00000010U)
#define CSL_IOMUX_RGMII1_RD2_CFG_REG_GPIO_SEL_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_RGMII1_RD2_CFG_REG_GPIO_SEL_MAX                              (0x00000003U)

#define CSL_IOMUX_RGMII1_RD2_CFG_REG_QUAL_SEL_MASK                             (0x000C0000U)
#define CSL_IOMUX_RGMII1_RD2_CFG_REG_QUAL_SEL_SHIFT                            (0x00000012U)
#define CSL_IOMUX_RGMII1_RD2_CFG_REG_QUAL_SEL_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_RGMII1_RD2_CFG_REG_QUAL_SEL_MAX                              (0x00000003U)

#define CSL_IOMUX_RGMII1_RD2_CFG_REG_INP_INV_SEL_MASK                          (0x00100000U)
#define CSL_IOMUX_RGMII1_RD2_CFG_REG_INP_INV_SEL_SHIFT                         (0x00000014U)
#define CSL_IOMUX_RGMII1_RD2_CFG_REG_INP_INV_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_RGMII1_RD2_CFG_REG_INP_INV_SEL_MAX                           (0x00000001U)

#define CSL_IOMUX_RGMII1_RD2_CFG_REG_ICSSM_GPIO_SEL_MASK                       (0x00200000U)
#define CSL_IOMUX_RGMII1_RD2_CFG_REG_ICSSM_GPIO_SEL_SHIFT                      (0x00000015U)
#define CSL_IOMUX_RGMII1_RD2_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                   (0x00000000U)
#define CSL_IOMUX_RGMII1_RD2_CFG_REG_ICSSM_GPIO_SEL_MAX                        (0x00000001U)

#define CSL_IOMUX_RGMII1_RD2_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                  (0x00400000U)
#define CSL_IOMUX_RGMII1_RD2_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                 (0x00000016U)
#define CSL_IOMUX_RGMII1_RD2_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL              (0x00000000U)
#define CSL_IOMUX_RGMII1_RD2_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                   (0x00000001U)

#define CSL_IOMUX_RGMII1_RD2_CFG_REG_HSMODE_MASK                               (0x40000000U)
#define CSL_IOMUX_RGMII1_RD2_CFG_REG_HSMODE_SHIFT                              (0x0000001EU)
#define CSL_IOMUX_RGMII1_RD2_CFG_REG_HSMODE_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_RGMII1_RD2_CFG_REG_HSMODE_MAX                                (0x00000001U)

#define CSL_IOMUX_RGMII1_RD2_CFG_REG_HSMASTER_MASK                             (0x80000000U)
#define CSL_IOMUX_RGMII1_RD2_CFG_REG_HSMASTER_SHIFT                            (0x0000001FU)
#define CSL_IOMUX_RGMII1_RD2_CFG_REG_HSMASTER_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_RGMII1_RD2_CFG_REG_HSMASTER_MAX                              (0x00000001U)

#define CSL_IOMUX_RGMII1_RD2_CFG_REG_RESETVAL                                  (0x000005F7U)

/* RGMII1_RD3_CFG_REG */

#define CSL_IOMUX_RGMII1_RD3_CFG_REG_FUNC_SEL_MASK                             (0x0000000FU)
#define CSL_IOMUX_RGMII1_RD3_CFG_REG_FUNC_SEL_SHIFT                            (0x00000000U)
#define CSL_IOMUX_RGMII1_RD3_CFG_REG_FUNC_SEL_RESETVAL                         (0x00000007U)
#define CSL_IOMUX_RGMII1_RD3_CFG_REG_FUNC_SEL_MAX                              (0x0000000FU)

#define CSL_IOMUX_RGMII1_RD3_CFG_REG_IE_OVERRIDE_CTRL_MASK                     (0x00000010U)
#define CSL_IOMUX_RGMII1_RD3_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                    (0x00000004U)
#define CSL_IOMUX_RGMII1_RD3_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_IOMUX_RGMII1_RD3_CFG_REG_IE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_IOMUX_RGMII1_RD3_CFG_REG_IE_OVERRIDE_MASK                          (0x00000020U)
#define CSL_IOMUX_RGMII1_RD3_CFG_REG_IE_OVERRIDE_SHIFT                         (0x00000005U)
#define CSL_IOMUX_RGMII1_RD3_CFG_REG_IE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_IOMUX_RGMII1_RD3_CFG_REG_IE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_IOMUX_RGMII1_RD3_CFG_REG_OE_OVERRIDE_CTRL_MASK                     (0x00000040U)
#define CSL_IOMUX_RGMII1_RD3_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                    (0x00000006U)
#define CSL_IOMUX_RGMII1_RD3_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_IOMUX_RGMII1_RD3_CFG_REG_OE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_IOMUX_RGMII1_RD3_CFG_REG_OE_OVERRIDE_MASK                          (0x00000080U)
#define CSL_IOMUX_RGMII1_RD3_CFG_REG_OE_OVERRIDE_SHIFT                         (0x00000007U)
#define CSL_IOMUX_RGMII1_RD3_CFG_REG_OE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_IOMUX_RGMII1_RD3_CFG_REG_OE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_IOMUX_RGMII1_RD3_CFG_REG_PI_MASK                                   (0x00000100U)
#define CSL_IOMUX_RGMII1_RD3_CFG_REG_PI_SHIFT                                  (0x00000008U)
#define CSL_IOMUX_RGMII1_RD3_CFG_REG_PI_RESETVAL                               (0x00000001U)
#define CSL_IOMUX_RGMII1_RD3_CFG_REG_PI_MAX                                    (0x00000001U)

#define CSL_IOMUX_RGMII1_RD3_CFG_REG_PUPDSEL_MASK                              (0x00000200U)
#define CSL_IOMUX_RGMII1_RD3_CFG_REG_PUPDSEL_SHIFT                             (0x00000009U)
#define CSL_IOMUX_RGMII1_RD3_CFG_REG_PUPDSEL_RESETVAL                          (0x00000000U)
#define CSL_IOMUX_RGMII1_RD3_CFG_REG_PUPDSEL_MAX                               (0x00000001U)

#define CSL_IOMUX_RGMII1_RD3_CFG_REG_SC1_MASK                                  (0x00000400U)
#define CSL_IOMUX_RGMII1_RD3_CFG_REG_SC1_SHIFT                                 (0x0000000AU)
#define CSL_IOMUX_RGMII1_RD3_CFG_REG_SC1_RESETVAL                              (0x00000001U)
#define CSL_IOMUX_RGMII1_RD3_CFG_REG_SC1_MAX                                   (0x00000001U)

#define CSL_IOMUX_RGMII1_RD3_CFG_REG_GPIO_SEL_MASK                             (0x00030000U)
#define CSL_IOMUX_RGMII1_RD3_CFG_REG_GPIO_SEL_SHIFT                            (0x00000010U)
#define CSL_IOMUX_RGMII1_RD3_CFG_REG_GPIO_SEL_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_RGMII1_RD3_CFG_REG_GPIO_SEL_MAX                              (0x00000003U)

#define CSL_IOMUX_RGMII1_RD3_CFG_REG_QUAL_SEL_MASK                             (0x000C0000U)
#define CSL_IOMUX_RGMII1_RD3_CFG_REG_QUAL_SEL_SHIFT                            (0x00000012U)
#define CSL_IOMUX_RGMII1_RD3_CFG_REG_QUAL_SEL_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_RGMII1_RD3_CFG_REG_QUAL_SEL_MAX                              (0x00000003U)

#define CSL_IOMUX_RGMII1_RD3_CFG_REG_INP_INV_SEL_MASK                          (0x00100000U)
#define CSL_IOMUX_RGMII1_RD3_CFG_REG_INP_INV_SEL_SHIFT                         (0x00000014U)
#define CSL_IOMUX_RGMII1_RD3_CFG_REG_INP_INV_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_RGMII1_RD3_CFG_REG_INP_INV_SEL_MAX                           (0x00000001U)

#define CSL_IOMUX_RGMII1_RD3_CFG_REG_ICSSM_GPIO_SEL_MASK                       (0x00200000U)
#define CSL_IOMUX_RGMII1_RD3_CFG_REG_ICSSM_GPIO_SEL_SHIFT                      (0x00000015U)
#define CSL_IOMUX_RGMII1_RD3_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                   (0x00000000U)
#define CSL_IOMUX_RGMII1_RD3_CFG_REG_ICSSM_GPIO_SEL_MAX                        (0x00000001U)

#define CSL_IOMUX_RGMII1_RD3_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                  (0x00400000U)
#define CSL_IOMUX_RGMII1_RD3_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                 (0x00000016U)
#define CSL_IOMUX_RGMII1_RD3_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL              (0x00000000U)
#define CSL_IOMUX_RGMII1_RD3_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                   (0x00000001U)

#define CSL_IOMUX_RGMII1_RD3_CFG_REG_HSMODE_MASK                               (0x40000000U)
#define CSL_IOMUX_RGMII1_RD3_CFG_REG_HSMODE_SHIFT                              (0x0000001EU)
#define CSL_IOMUX_RGMII1_RD3_CFG_REG_HSMODE_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_RGMII1_RD3_CFG_REG_HSMODE_MAX                                (0x00000001U)

#define CSL_IOMUX_RGMII1_RD3_CFG_REG_HSMASTER_MASK                             (0x80000000U)
#define CSL_IOMUX_RGMII1_RD3_CFG_REG_HSMASTER_SHIFT                            (0x0000001FU)
#define CSL_IOMUX_RGMII1_RD3_CFG_REG_HSMASTER_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_RGMII1_RD3_CFG_REG_HSMASTER_MAX                              (0x00000001U)

#define CSL_IOMUX_RGMII1_RD3_CFG_REG_RESETVAL                                  (0x000005F7U)

/* RGMII1_TXC_CFG_REG */

#define CSL_IOMUX_RGMII1_TXC_CFG_REG_FUNC_SEL_MASK                             (0x0000000FU)
#define CSL_IOMUX_RGMII1_TXC_CFG_REG_FUNC_SEL_SHIFT                            (0x00000000U)
#define CSL_IOMUX_RGMII1_TXC_CFG_REG_FUNC_SEL_RESETVAL                         (0x00000007U)
#define CSL_IOMUX_RGMII1_TXC_CFG_REG_FUNC_SEL_MAX                              (0x0000000FU)

#define CSL_IOMUX_RGMII1_TXC_CFG_REG_IE_OVERRIDE_CTRL_MASK                     (0x00000010U)
#define CSL_IOMUX_RGMII1_TXC_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                    (0x00000004U)
#define CSL_IOMUX_RGMII1_TXC_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_IOMUX_RGMII1_TXC_CFG_REG_IE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_IOMUX_RGMII1_TXC_CFG_REG_IE_OVERRIDE_MASK                          (0x00000020U)
#define CSL_IOMUX_RGMII1_TXC_CFG_REG_IE_OVERRIDE_SHIFT                         (0x00000005U)
#define CSL_IOMUX_RGMII1_TXC_CFG_REG_IE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_IOMUX_RGMII1_TXC_CFG_REG_IE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_IOMUX_RGMII1_TXC_CFG_REG_OE_OVERRIDE_CTRL_MASK                     (0x00000040U)
#define CSL_IOMUX_RGMII1_TXC_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                    (0x00000006U)
#define CSL_IOMUX_RGMII1_TXC_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_IOMUX_RGMII1_TXC_CFG_REG_OE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_IOMUX_RGMII1_TXC_CFG_REG_OE_OVERRIDE_MASK                          (0x00000080U)
#define CSL_IOMUX_RGMII1_TXC_CFG_REG_OE_OVERRIDE_SHIFT                         (0x00000007U)
#define CSL_IOMUX_RGMII1_TXC_CFG_REG_OE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_IOMUX_RGMII1_TXC_CFG_REG_OE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_IOMUX_RGMII1_TXC_CFG_REG_PI_MASK                                   (0x00000100U)
#define CSL_IOMUX_RGMII1_TXC_CFG_REG_PI_SHIFT                                  (0x00000008U)
#define CSL_IOMUX_RGMII1_TXC_CFG_REG_PI_RESETVAL                               (0x00000001U)
#define CSL_IOMUX_RGMII1_TXC_CFG_REG_PI_MAX                                    (0x00000001U)

#define CSL_IOMUX_RGMII1_TXC_CFG_REG_PUPDSEL_MASK                              (0x00000200U)
#define CSL_IOMUX_RGMII1_TXC_CFG_REG_PUPDSEL_SHIFT                             (0x00000009U)
#define CSL_IOMUX_RGMII1_TXC_CFG_REG_PUPDSEL_RESETVAL                          (0x00000000U)
#define CSL_IOMUX_RGMII1_TXC_CFG_REG_PUPDSEL_MAX                               (0x00000001U)

#define CSL_IOMUX_RGMII1_TXC_CFG_REG_SC1_MASK                                  (0x00000400U)
#define CSL_IOMUX_RGMII1_TXC_CFG_REG_SC1_SHIFT                                 (0x0000000AU)
#define CSL_IOMUX_RGMII1_TXC_CFG_REG_SC1_RESETVAL                              (0x00000001U)
#define CSL_IOMUX_RGMII1_TXC_CFG_REG_SC1_MAX                                   (0x00000001U)

#define CSL_IOMUX_RGMII1_TXC_CFG_REG_GPIO_SEL_MASK                             (0x00030000U)
#define CSL_IOMUX_RGMII1_TXC_CFG_REG_GPIO_SEL_SHIFT                            (0x00000010U)
#define CSL_IOMUX_RGMII1_TXC_CFG_REG_GPIO_SEL_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_RGMII1_TXC_CFG_REG_GPIO_SEL_MAX                              (0x00000003U)

#define CSL_IOMUX_RGMII1_TXC_CFG_REG_QUAL_SEL_MASK                             (0x000C0000U)
#define CSL_IOMUX_RGMII1_TXC_CFG_REG_QUAL_SEL_SHIFT                            (0x00000012U)
#define CSL_IOMUX_RGMII1_TXC_CFG_REG_QUAL_SEL_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_RGMII1_TXC_CFG_REG_QUAL_SEL_MAX                              (0x00000003U)

#define CSL_IOMUX_RGMII1_TXC_CFG_REG_INP_INV_SEL_MASK                          (0x00100000U)
#define CSL_IOMUX_RGMII1_TXC_CFG_REG_INP_INV_SEL_SHIFT                         (0x00000014U)
#define CSL_IOMUX_RGMII1_TXC_CFG_REG_INP_INV_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_RGMII1_TXC_CFG_REG_INP_INV_SEL_MAX                           (0x00000001U)

#define CSL_IOMUX_RGMII1_TXC_CFG_REG_ICSSM_GPIO_SEL_MASK                       (0x00200000U)
#define CSL_IOMUX_RGMII1_TXC_CFG_REG_ICSSM_GPIO_SEL_SHIFT                      (0x00000015U)
#define CSL_IOMUX_RGMII1_TXC_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                   (0x00000000U)
#define CSL_IOMUX_RGMII1_TXC_CFG_REG_ICSSM_GPIO_SEL_MAX                        (0x00000001U)

#define CSL_IOMUX_RGMII1_TXC_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                  (0x00400000U)
#define CSL_IOMUX_RGMII1_TXC_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                 (0x00000016U)
#define CSL_IOMUX_RGMII1_TXC_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL              (0x00000000U)
#define CSL_IOMUX_RGMII1_TXC_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                   (0x00000001U)

#define CSL_IOMUX_RGMII1_TXC_CFG_REG_HSMODE_MASK                               (0x40000000U)
#define CSL_IOMUX_RGMII1_TXC_CFG_REG_HSMODE_SHIFT                              (0x0000001EU)
#define CSL_IOMUX_RGMII1_TXC_CFG_REG_HSMODE_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_RGMII1_TXC_CFG_REG_HSMODE_MAX                                (0x00000001U)

#define CSL_IOMUX_RGMII1_TXC_CFG_REG_HSMASTER_MASK                             (0x80000000U)
#define CSL_IOMUX_RGMII1_TXC_CFG_REG_HSMASTER_SHIFT                            (0x0000001FU)
#define CSL_IOMUX_RGMII1_TXC_CFG_REG_HSMASTER_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_RGMII1_TXC_CFG_REG_HSMASTER_MAX                              (0x00000001U)

#define CSL_IOMUX_RGMII1_TXC_CFG_REG_RESETVAL                                  (0x000005F7U)

/* RGMII1_TX_CTL_CFG_REG */

#define CSL_IOMUX_RGMII1_TX_CTL_CFG_REG_FUNC_SEL_MASK                          (0x0000000FU)
#define CSL_IOMUX_RGMII1_TX_CTL_CFG_REG_FUNC_SEL_SHIFT                         (0x00000000U)
#define CSL_IOMUX_RGMII1_TX_CTL_CFG_REG_FUNC_SEL_RESETVAL                      (0x00000007U)
#define CSL_IOMUX_RGMII1_TX_CTL_CFG_REG_FUNC_SEL_MAX                           (0x0000000FU)

#define CSL_IOMUX_RGMII1_TX_CTL_CFG_REG_IE_OVERRIDE_CTRL_MASK                  (0x00000010U)
#define CSL_IOMUX_RGMII1_TX_CTL_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                 (0x00000004U)
#define CSL_IOMUX_RGMII1_TX_CTL_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL              (0x00000001U)
#define CSL_IOMUX_RGMII1_TX_CTL_CFG_REG_IE_OVERRIDE_CTRL_MAX                   (0x00000001U)

#define CSL_IOMUX_RGMII1_TX_CTL_CFG_REG_IE_OVERRIDE_MASK                       (0x00000020U)
#define CSL_IOMUX_RGMII1_TX_CTL_CFG_REG_IE_OVERRIDE_SHIFT                      (0x00000005U)
#define CSL_IOMUX_RGMII1_TX_CTL_CFG_REG_IE_OVERRIDE_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_RGMII1_TX_CTL_CFG_REG_IE_OVERRIDE_MAX                        (0x00000001U)

#define CSL_IOMUX_RGMII1_TX_CTL_CFG_REG_OE_OVERRIDE_CTRL_MASK                  (0x00000040U)
#define CSL_IOMUX_RGMII1_TX_CTL_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                 (0x00000006U)
#define CSL_IOMUX_RGMII1_TX_CTL_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL              (0x00000001U)
#define CSL_IOMUX_RGMII1_TX_CTL_CFG_REG_OE_OVERRIDE_CTRL_MAX                   (0x00000001U)

#define CSL_IOMUX_RGMII1_TX_CTL_CFG_REG_OE_OVERRIDE_MASK                       (0x00000080U)
#define CSL_IOMUX_RGMII1_TX_CTL_CFG_REG_OE_OVERRIDE_SHIFT                      (0x00000007U)
#define CSL_IOMUX_RGMII1_TX_CTL_CFG_REG_OE_OVERRIDE_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_RGMII1_TX_CTL_CFG_REG_OE_OVERRIDE_MAX                        (0x00000001U)

#define CSL_IOMUX_RGMII1_TX_CTL_CFG_REG_PI_MASK                                (0x00000100U)
#define CSL_IOMUX_RGMII1_TX_CTL_CFG_REG_PI_SHIFT                               (0x00000008U)
#define CSL_IOMUX_RGMII1_TX_CTL_CFG_REG_PI_RESETVAL                            (0x00000001U)
#define CSL_IOMUX_RGMII1_TX_CTL_CFG_REG_PI_MAX                                 (0x00000001U)

#define CSL_IOMUX_RGMII1_TX_CTL_CFG_REG_PUPDSEL_MASK                           (0x00000200U)
#define CSL_IOMUX_RGMII1_TX_CTL_CFG_REG_PUPDSEL_SHIFT                          (0x00000009U)
#define CSL_IOMUX_RGMII1_TX_CTL_CFG_REG_PUPDSEL_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_RGMII1_TX_CTL_CFG_REG_PUPDSEL_MAX                            (0x00000001U)

#define CSL_IOMUX_RGMII1_TX_CTL_CFG_REG_SC1_MASK                               (0x00000400U)
#define CSL_IOMUX_RGMII1_TX_CTL_CFG_REG_SC1_SHIFT                              (0x0000000AU)
#define CSL_IOMUX_RGMII1_TX_CTL_CFG_REG_SC1_RESETVAL                           (0x00000001U)
#define CSL_IOMUX_RGMII1_TX_CTL_CFG_REG_SC1_MAX                                (0x00000001U)

#define CSL_IOMUX_RGMII1_TX_CTL_CFG_REG_GPIO_SEL_MASK                          (0x00030000U)
#define CSL_IOMUX_RGMII1_TX_CTL_CFG_REG_GPIO_SEL_SHIFT                         (0x00000010U)
#define CSL_IOMUX_RGMII1_TX_CTL_CFG_REG_GPIO_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_RGMII1_TX_CTL_CFG_REG_GPIO_SEL_MAX                           (0x00000003U)

#define CSL_IOMUX_RGMII1_TX_CTL_CFG_REG_QUAL_SEL_MASK                          (0x000C0000U)
#define CSL_IOMUX_RGMII1_TX_CTL_CFG_REG_QUAL_SEL_SHIFT                         (0x00000012U)
#define CSL_IOMUX_RGMII1_TX_CTL_CFG_REG_QUAL_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_RGMII1_TX_CTL_CFG_REG_QUAL_SEL_MAX                           (0x00000003U)

#define CSL_IOMUX_RGMII1_TX_CTL_CFG_REG_INP_INV_SEL_MASK                       (0x00100000U)
#define CSL_IOMUX_RGMII1_TX_CTL_CFG_REG_INP_INV_SEL_SHIFT                      (0x00000014U)
#define CSL_IOMUX_RGMII1_TX_CTL_CFG_REG_INP_INV_SEL_RESETVAL                   (0x00000000U)
#define CSL_IOMUX_RGMII1_TX_CTL_CFG_REG_INP_INV_SEL_MAX                        (0x00000001U)

#define CSL_IOMUX_RGMII1_TX_CTL_CFG_REG_ICSSM_GPIO_SEL_MASK                    (0x00200000U)
#define CSL_IOMUX_RGMII1_TX_CTL_CFG_REG_ICSSM_GPIO_SEL_SHIFT                   (0x00000015U)
#define CSL_IOMUX_RGMII1_TX_CTL_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                (0x00000000U)
#define CSL_IOMUX_RGMII1_TX_CTL_CFG_REG_ICSSM_GPIO_SEL_MAX                     (0x00000001U)

#define CSL_IOMUX_RGMII1_TX_CTL_CFG_REG_SAFETY_OVERRIDE_SEL_MASK               (0x00400000U)
#define CSL_IOMUX_RGMII1_TX_CTL_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT              (0x00000016U)
#define CSL_IOMUX_RGMII1_TX_CTL_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL           (0x00000000U)
#define CSL_IOMUX_RGMII1_TX_CTL_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                (0x00000001U)

#define CSL_IOMUX_RGMII1_TX_CTL_CFG_REG_HSMODE_MASK                            (0x40000000U)
#define CSL_IOMUX_RGMII1_TX_CTL_CFG_REG_HSMODE_SHIFT                           (0x0000001EU)
#define CSL_IOMUX_RGMII1_TX_CTL_CFG_REG_HSMODE_RESETVAL                        (0x00000000U)
#define CSL_IOMUX_RGMII1_TX_CTL_CFG_REG_HSMODE_MAX                             (0x00000001U)

#define CSL_IOMUX_RGMII1_TX_CTL_CFG_REG_HSMASTER_MASK                          (0x80000000U)
#define CSL_IOMUX_RGMII1_TX_CTL_CFG_REG_HSMASTER_SHIFT                         (0x0000001FU)
#define CSL_IOMUX_RGMII1_TX_CTL_CFG_REG_HSMASTER_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_RGMII1_TX_CTL_CFG_REG_HSMASTER_MAX                           (0x00000001U)

#define CSL_IOMUX_RGMII1_TX_CTL_CFG_REG_RESETVAL                               (0x000005F7U)

/* RGMII1_TD0_CFG_REG */

#define CSL_IOMUX_RGMII1_TD0_CFG_REG_FUNC_SEL_MASK                             (0x0000000FU)
#define CSL_IOMUX_RGMII1_TD0_CFG_REG_FUNC_SEL_SHIFT                            (0x00000000U)
#define CSL_IOMUX_RGMII1_TD0_CFG_REG_FUNC_SEL_RESETVAL                         (0x00000007U)
#define CSL_IOMUX_RGMII1_TD0_CFG_REG_FUNC_SEL_MAX                              (0x0000000FU)

#define CSL_IOMUX_RGMII1_TD0_CFG_REG_IE_OVERRIDE_CTRL_MASK                     (0x00000010U)
#define CSL_IOMUX_RGMII1_TD0_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                    (0x00000004U)
#define CSL_IOMUX_RGMII1_TD0_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_IOMUX_RGMII1_TD0_CFG_REG_IE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_IOMUX_RGMII1_TD0_CFG_REG_IE_OVERRIDE_MASK                          (0x00000020U)
#define CSL_IOMUX_RGMII1_TD0_CFG_REG_IE_OVERRIDE_SHIFT                         (0x00000005U)
#define CSL_IOMUX_RGMII1_TD0_CFG_REG_IE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_IOMUX_RGMII1_TD0_CFG_REG_IE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_IOMUX_RGMII1_TD0_CFG_REG_OE_OVERRIDE_CTRL_MASK                     (0x00000040U)
#define CSL_IOMUX_RGMII1_TD0_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                    (0x00000006U)
#define CSL_IOMUX_RGMII1_TD0_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_IOMUX_RGMII1_TD0_CFG_REG_OE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_IOMUX_RGMII1_TD0_CFG_REG_OE_OVERRIDE_MASK                          (0x00000080U)
#define CSL_IOMUX_RGMII1_TD0_CFG_REG_OE_OVERRIDE_SHIFT                         (0x00000007U)
#define CSL_IOMUX_RGMII1_TD0_CFG_REG_OE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_IOMUX_RGMII1_TD0_CFG_REG_OE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_IOMUX_RGMII1_TD0_CFG_REG_PI_MASK                                   (0x00000100U)
#define CSL_IOMUX_RGMII1_TD0_CFG_REG_PI_SHIFT                                  (0x00000008U)
#define CSL_IOMUX_RGMII1_TD0_CFG_REG_PI_RESETVAL                               (0x00000001U)
#define CSL_IOMUX_RGMII1_TD0_CFG_REG_PI_MAX                                    (0x00000001U)

#define CSL_IOMUX_RGMII1_TD0_CFG_REG_PUPDSEL_MASK                              (0x00000200U)
#define CSL_IOMUX_RGMII1_TD0_CFG_REG_PUPDSEL_SHIFT                             (0x00000009U)
#define CSL_IOMUX_RGMII1_TD0_CFG_REG_PUPDSEL_RESETVAL                          (0x00000000U)
#define CSL_IOMUX_RGMII1_TD0_CFG_REG_PUPDSEL_MAX                               (0x00000001U)

#define CSL_IOMUX_RGMII1_TD0_CFG_REG_SC1_MASK                                  (0x00000400U)
#define CSL_IOMUX_RGMII1_TD0_CFG_REG_SC1_SHIFT                                 (0x0000000AU)
#define CSL_IOMUX_RGMII1_TD0_CFG_REG_SC1_RESETVAL                              (0x00000001U)
#define CSL_IOMUX_RGMII1_TD0_CFG_REG_SC1_MAX                                   (0x00000001U)

#define CSL_IOMUX_RGMII1_TD0_CFG_REG_GPIO_SEL_MASK                             (0x00030000U)
#define CSL_IOMUX_RGMII1_TD0_CFG_REG_GPIO_SEL_SHIFT                            (0x00000010U)
#define CSL_IOMUX_RGMII1_TD0_CFG_REG_GPIO_SEL_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_RGMII1_TD0_CFG_REG_GPIO_SEL_MAX                              (0x00000003U)

#define CSL_IOMUX_RGMII1_TD0_CFG_REG_QUAL_SEL_MASK                             (0x000C0000U)
#define CSL_IOMUX_RGMII1_TD0_CFG_REG_QUAL_SEL_SHIFT                            (0x00000012U)
#define CSL_IOMUX_RGMII1_TD0_CFG_REG_QUAL_SEL_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_RGMII1_TD0_CFG_REG_QUAL_SEL_MAX                              (0x00000003U)

#define CSL_IOMUX_RGMII1_TD0_CFG_REG_INP_INV_SEL_MASK                          (0x00100000U)
#define CSL_IOMUX_RGMII1_TD0_CFG_REG_INP_INV_SEL_SHIFT                         (0x00000014U)
#define CSL_IOMUX_RGMII1_TD0_CFG_REG_INP_INV_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_RGMII1_TD0_CFG_REG_INP_INV_SEL_MAX                           (0x00000001U)

#define CSL_IOMUX_RGMII1_TD0_CFG_REG_ICSSM_GPIO_SEL_MASK                       (0x00200000U)
#define CSL_IOMUX_RGMII1_TD0_CFG_REG_ICSSM_GPIO_SEL_SHIFT                      (0x00000015U)
#define CSL_IOMUX_RGMII1_TD0_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                   (0x00000000U)
#define CSL_IOMUX_RGMII1_TD0_CFG_REG_ICSSM_GPIO_SEL_MAX                        (0x00000001U)

#define CSL_IOMUX_RGMII1_TD0_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                  (0x00400000U)
#define CSL_IOMUX_RGMII1_TD0_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                 (0x00000016U)
#define CSL_IOMUX_RGMII1_TD0_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL              (0x00000000U)
#define CSL_IOMUX_RGMII1_TD0_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                   (0x00000001U)

#define CSL_IOMUX_RGMII1_TD0_CFG_REG_HSMODE_MASK                               (0x40000000U)
#define CSL_IOMUX_RGMII1_TD0_CFG_REG_HSMODE_SHIFT                              (0x0000001EU)
#define CSL_IOMUX_RGMII1_TD0_CFG_REG_HSMODE_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_RGMII1_TD0_CFG_REG_HSMODE_MAX                                (0x00000001U)

#define CSL_IOMUX_RGMII1_TD0_CFG_REG_HSMASTER_MASK                             (0x80000000U)
#define CSL_IOMUX_RGMII1_TD0_CFG_REG_HSMASTER_SHIFT                            (0x0000001FU)
#define CSL_IOMUX_RGMII1_TD0_CFG_REG_HSMASTER_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_RGMII1_TD0_CFG_REG_HSMASTER_MAX                              (0x00000001U)

#define CSL_IOMUX_RGMII1_TD0_CFG_REG_RESETVAL                                  (0x000005F7U)

/* RGMII1_TD1_CFG_REG */

#define CSL_IOMUX_RGMII1_TD1_CFG_REG_FUNC_SEL_MASK                             (0x0000000FU)
#define CSL_IOMUX_RGMII1_TD1_CFG_REG_FUNC_SEL_SHIFT                            (0x00000000U)
#define CSL_IOMUX_RGMII1_TD1_CFG_REG_FUNC_SEL_RESETVAL                         (0x00000007U)
#define CSL_IOMUX_RGMII1_TD1_CFG_REG_FUNC_SEL_MAX                              (0x0000000FU)

#define CSL_IOMUX_RGMII1_TD1_CFG_REG_IE_OVERRIDE_CTRL_MASK                     (0x00000010U)
#define CSL_IOMUX_RGMII1_TD1_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                    (0x00000004U)
#define CSL_IOMUX_RGMII1_TD1_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_IOMUX_RGMII1_TD1_CFG_REG_IE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_IOMUX_RGMII1_TD1_CFG_REG_IE_OVERRIDE_MASK                          (0x00000020U)
#define CSL_IOMUX_RGMII1_TD1_CFG_REG_IE_OVERRIDE_SHIFT                         (0x00000005U)
#define CSL_IOMUX_RGMII1_TD1_CFG_REG_IE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_IOMUX_RGMII1_TD1_CFG_REG_IE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_IOMUX_RGMII1_TD1_CFG_REG_OE_OVERRIDE_CTRL_MASK                     (0x00000040U)
#define CSL_IOMUX_RGMII1_TD1_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                    (0x00000006U)
#define CSL_IOMUX_RGMII1_TD1_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_IOMUX_RGMII1_TD1_CFG_REG_OE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_IOMUX_RGMII1_TD1_CFG_REG_OE_OVERRIDE_MASK                          (0x00000080U)
#define CSL_IOMUX_RGMII1_TD1_CFG_REG_OE_OVERRIDE_SHIFT                         (0x00000007U)
#define CSL_IOMUX_RGMII1_TD1_CFG_REG_OE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_IOMUX_RGMII1_TD1_CFG_REG_OE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_IOMUX_RGMII1_TD1_CFG_REG_PI_MASK                                   (0x00000100U)
#define CSL_IOMUX_RGMII1_TD1_CFG_REG_PI_SHIFT                                  (0x00000008U)
#define CSL_IOMUX_RGMII1_TD1_CFG_REG_PI_RESETVAL                               (0x00000001U)
#define CSL_IOMUX_RGMII1_TD1_CFG_REG_PI_MAX                                    (0x00000001U)

#define CSL_IOMUX_RGMII1_TD1_CFG_REG_PUPDSEL_MASK                              (0x00000200U)
#define CSL_IOMUX_RGMII1_TD1_CFG_REG_PUPDSEL_SHIFT                             (0x00000009U)
#define CSL_IOMUX_RGMII1_TD1_CFG_REG_PUPDSEL_RESETVAL                          (0x00000000U)
#define CSL_IOMUX_RGMII1_TD1_CFG_REG_PUPDSEL_MAX                               (0x00000001U)

#define CSL_IOMUX_RGMII1_TD1_CFG_REG_SC1_MASK                                  (0x00000400U)
#define CSL_IOMUX_RGMII1_TD1_CFG_REG_SC1_SHIFT                                 (0x0000000AU)
#define CSL_IOMUX_RGMII1_TD1_CFG_REG_SC1_RESETVAL                              (0x00000001U)
#define CSL_IOMUX_RGMII1_TD1_CFG_REG_SC1_MAX                                   (0x00000001U)

#define CSL_IOMUX_RGMII1_TD1_CFG_REG_GPIO_SEL_MASK                             (0x00030000U)
#define CSL_IOMUX_RGMII1_TD1_CFG_REG_GPIO_SEL_SHIFT                            (0x00000010U)
#define CSL_IOMUX_RGMII1_TD1_CFG_REG_GPIO_SEL_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_RGMII1_TD1_CFG_REG_GPIO_SEL_MAX                              (0x00000003U)

#define CSL_IOMUX_RGMII1_TD1_CFG_REG_QUAL_SEL_MASK                             (0x000C0000U)
#define CSL_IOMUX_RGMII1_TD1_CFG_REG_QUAL_SEL_SHIFT                            (0x00000012U)
#define CSL_IOMUX_RGMII1_TD1_CFG_REG_QUAL_SEL_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_RGMII1_TD1_CFG_REG_QUAL_SEL_MAX                              (0x00000003U)

#define CSL_IOMUX_RGMII1_TD1_CFG_REG_INP_INV_SEL_MASK                          (0x00100000U)
#define CSL_IOMUX_RGMII1_TD1_CFG_REG_INP_INV_SEL_SHIFT                         (0x00000014U)
#define CSL_IOMUX_RGMII1_TD1_CFG_REG_INP_INV_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_RGMII1_TD1_CFG_REG_INP_INV_SEL_MAX                           (0x00000001U)

#define CSL_IOMUX_RGMII1_TD1_CFG_REG_ICSSM_GPIO_SEL_MASK                       (0x00200000U)
#define CSL_IOMUX_RGMII1_TD1_CFG_REG_ICSSM_GPIO_SEL_SHIFT                      (0x00000015U)
#define CSL_IOMUX_RGMII1_TD1_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                   (0x00000000U)
#define CSL_IOMUX_RGMII1_TD1_CFG_REG_ICSSM_GPIO_SEL_MAX                        (0x00000001U)

#define CSL_IOMUX_RGMII1_TD1_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                  (0x00400000U)
#define CSL_IOMUX_RGMII1_TD1_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                 (0x00000016U)
#define CSL_IOMUX_RGMII1_TD1_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL              (0x00000000U)
#define CSL_IOMUX_RGMII1_TD1_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                   (0x00000001U)

#define CSL_IOMUX_RGMII1_TD1_CFG_REG_HSMODE_MASK                               (0x40000000U)
#define CSL_IOMUX_RGMII1_TD1_CFG_REG_HSMODE_SHIFT                              (0x0000001EU)
#define CSL_IOMUX_RGMII1_TD1_CFG_REG_HSMODE_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_RGMII1_TD1_CFG_REG_HSMODE_MAX                                (0x00000001U)

#define CSL_IOMUX_RGMII1_TD1_CFG_REG_HSMASTER_MASK                             (0x80000000U)
#define CSL_IOMUX_RGMII1_TD1_CFG_REG_HSMASTER_SHIFT                            (0x0000001FU)
#define CSL_IOMUX_RGMII1_TD1_CFG_REG_HSMASTER_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_RGMII1_TD1_CFG_REG_HSMASTER_MAX                              (0x00000001U)

#define CSL_IOMUX_RGMII1_TD1_CFG_REG_RESETVAL                                  (0x000005F7U)

/* RGMII1_TD2_CFG_REG */

#define CSL_IOMUX_RGMII1_TD2_CFG_REG_FUNC_SEL_MASK                             (0x0000000FU)
#define CSL_IOMUX_RGMII1_TD2_CFG_REG_FUNC_SEL_SHIFT                            (0x00000000U)
#define CSL_IOMUX_RGMII1_TD2_CFG_REG_FUNC_SEL_RESETVAL                         (0x00000007U)
#define CSL_IOMUX_RGMII1_TD2_CFG_REG_FUNC_SEL_MAX                              (0x0000000FU)

#define CSL_IOMUX_RGMII1_TD2_CFG_REG_IE_OVERRIDE_CTRL_MASK                     (0x00000010U)
#define CSL_IOMUX_RGMII1_TD2_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                    (0x00000004U)
#define CSL_IOMUX_RGMII1_TD2_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_IOMUX_RGMII1_TD2_CFG_REG_IE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_IOMUX_RGMII1_TD2_CFG_REG_IE_OVERRIDE_MASK                          (0x00000020U)
#define CSL_IOMUX_RGMII1_TD2_CFG_REG_IE_OVERRIDE_SHIFT                         (0x00000005U)
#define CSL_IOMUX_RGMII1_TD2_CFG_REG_IE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_IOMUX_RGMII1_TD2_CFG_REG_IE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_IOMUX_RGMII1_TD2_CFG_REG_OE_OVERRIDE_CTRL_MASK                     (0x00000040U)
#define CSL_IOMUX_RGMII1_TD2_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                    (0x00000006U)
#define CSL_IOMUX_RGMII1_TD2_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_IOMUX_RGMII1_TD2_CFG_REG_OE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_IOMUX_RGMII1_TD2_CFG_REG_OE_OVERRIDE_MASK                          (0x00000080U)
#define CSL_IOMUX_RGMII1_TD2_CFG_REG_OE_OVERRIDE_SHIFT                         (0x00000007U)
#define CSL_IOMUX_RGMII1_TD2_CFG_REG_OE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_IOMUX_RGMII1_TD2_CFG_REG_OE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_IOMUX_RGMII1_TD2_CFG_REG_PI_MASK                                   (0x00000100U)
#define CSL_IOMUX_RGMII1_TD2_CFG_REG_PI_SHIFT                                  (0x00000008U)
#define CSL_IOMUX_RGMII1_TD2_CFG_REG_PI_RESETVAL                               (0x00000001U)
#define CSL_IOMUX_RGMII1_TD2_CFG_REG_PI_MAX                                    (0x00000001U)

#define CSL_IOMUX_RGMII1_TD2_CFG_REG_PUPDSEL_MASK                              (0x00000200U)
#define CSL_IOMUX_RGMII1_TD2_CFG_REG_PUPDSEL_SHIFT                             (0x00000009U)
#define CSL_IOMUX_RGMII1_TD2_CFG_REG_PUPDSEL_RESETVAL                          (0x00000000U)
#define CSL_IOMUX_RGMII1_TD2_CFG_REG_PUPDSEL_MAX                               (0x00000001U)

#define CSL_IOMUX_RGMII1_TD2_CFG_REG_SC1_MASK                                  (0x00000400U)
#define CSL_IOMUX_RGMII1_TD2_CFG_REG_SC1_SHIFT                                 (0x0000000AU)
#define CSL_IOMUX_RGMII1_TD2_CFG_REG_SC1_RESETVAL                              (0x00000001U)
#define CSL_IOMUX_RGMII1_TD2_CFG_REG_SC1_MAX                                   (0x00000001U)

#define CSL_IOMUX_RGMII1_TD2_CFG_REG_GPIO_SEL_MASK                             (0x00030000U)
#define CSL_IOMUX_RGMII1_TD2_CFG_REG_GPIO_SEL_SHIFT                            (0x00000010U)
#define CSL_IOMUX_RGMII1_TD2_CFG_REG_GPIO_SEL_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_RGMII1_TD2_CFG_REG_GPIO_SEL_MAX                              (0x00000003U)

#define CSL_IOMUX_RGMII1_TD2_CFG_REG_QUAL_SEL_MASK                             (0x000C0000U)
#define CSL_IOMUX_RGMII1_TD2_CFG_REG_QUAL_SEL_SHIFT                            (0x00000012U)
#define CSL_IOMUX_RGMII1_TD2_CFG_REG_QUAL_SEL_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_RGMII1_TD2_CFG_REG_QUAL_SEL_MAX                              (0x00000003U)

#define CSL_IOMUX_RGMII1_TD2_CFG_REG_INP_INV_SEL_MASK                          (0x00100000U)
#define CSL_IOMUX_RGMII1_TD2_CFG_REG_INP_INV_SEL_SHIFT                         (0x00000014U)
#define CSL_IOMUX_RGMII1_TD2_CFG_REG_INP_INV_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_RGMII1_TD2_CFG_REG_INP_INV_SEL_MAX                           (0x00000001U)

#define CSL_IOMUX_RGMII1_TD2_CFG_REG_ICSSM_GPIO_SEL_MASK                       (0x00200000U)
#define CSL_IOMUX_RGMII1_TD2_CFG_REG_ICSSM_GPIO_SEL_SHIFT                      (0x00000015U)
#define CSL_IOMUX_RGMII1_TD2_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                   (0x00000000U)
#define CSL_IOMUX_RGMII1_TD2_CFG_REG_ICSSM_GPIO_SEL_MAX                        (0x00000001U)

#define CSL_IOMUX_RGMII1_TD2_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                  (0x00400000U)
#define CSL_IOMUX_RGMII1_TD2_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                 (0x00000016U)
#define CSL_IOMUX_RGMII1_TD2_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL              (0x00000000U)
#define CSL_IOMUX_RGMII1_TD2_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                   (0x00000001U)

#define CSL_IOMUX_RGMII1_TD2_CFG_REG_HSMODE_MASK                               (0x40000000U)
#define CSL_IOMUX_RGMII1_TD2_CFG_REG_HSMODE_SHIFT                              (0x0000001EU)
#define CSL_IOMUX_RGMII1_TD2_CFG_REG_HSMODE_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_RGMII1_TD2_CFG_REG_HSMODE_MAX                                (0x00000001U)

#define CSL_IOMUX_RGMII1_TD2_CFG_REG_HSMASTER_MASK                             (0x80000000U)
#define CSL_IOMUX_RGMII1_TD2_CFG_REG_HSMASTER_SHIFT                            (0x0000001FU)
#define CSL_IOMUX_RGMII1_TD2_CFG_REG_HSMASTER_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_RGMII1_TD2_CFG_REG_HSMASTER_MAX                              (0x00000001U)

#define CSL_IOMUX_RGMII1_TD2_CFG_REG_RESETVAL                                  (0x000005F7U)

/* RGMII1_TD3_CFG_REG */

#define CSL_IOMUX_RGMII1_TD3_CFG_REG_FUNC_SEL_MASK                             (0x0000000FU)
#define CSL_IOMUX_RGMII1_TD3_CFG_REG_FUNC_SEL_SHIFT                            (0x00000000U)
#define CSL_IOMUX_RGMII1_TD3_CFG_REG_FUNC_SEL_RESETVAL                         (0x00000007U)
#define CSL_IOMUX_RGMII1_TD3_CFG_REG_FUNC_SEL_MAX                              (0x0000000FU)

#define CSL_IOMUX_RGMII1_TD3_CFG_REG_IE_OVERRIDE_CTRL_MASK                     (0x00000010U)
#define CSL_IOMUX_RGMII1_TD3_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                    (0x00000004U)
#define CSL_IOMUX_RGMII1_TD3_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_IOMUX_RGMII1_TD3_CFG_REG_IE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_IOMUX_RGMII1_TD3_CFG_REG_IE_OVERRIDE_MASK                          (0x00000020U)
#define CSL_IOMUX_RGMII1_TD3_CFG_REG_IE_OVERRIDE_SHIFT                         (0x00000005U)
#define CSL_IOMUX_RGMII1_TD3_CFG_REG_IE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_IOMUX_RGMII1_TD3_CFG_REG_IE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_IOMUX_RGMII1_TD3_CFG_REG_OE_OVERRIDE_CTRL_MASK                     (0x00000040U)
#define CSL_IOMUX_RGMII1_TD3_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                    (0x00000006U)
#define CSL_IOMUX_RGMII1_TD3_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_IOMUX_RGMII1_TD3_CFG_REG_OE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_IOMUX_RGMII1_TD3_CFG_REG_OE_OVERRIDE_MASK                          (0x00000080U)
#define CSL_IOMUX_RGMII1_TD3_CFG_REG_OE_OVERRIDE_SHIFT                         (0x00000007U)
#define CSL_IOMUX_RGMII1_TD3_CFG_REG_OE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_IOMUX_RGMII1_TD3_CFG_REG_OE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_IOMUX_RGMII1_TD3_CFG_REG_PI_MASK                                   (0x00000100U)
#define CSL_IOMUX_RGMII1_TD3_CFG_REG_PI_SHIFT                                  (0x00000008U)
#define CSL_IOMUX_RGMII1_TD3_CFG_REG_PI_RESETVAL                               (0x00000001U)
#define CSL_IOMUX_RGMII1_TD3_CFG_REG_PI_MAX                                    (0x00000001U)

#define CSL_IOMUX_RGMII1_TD3_CFG_REG_PUPDSEL_MASK                              (0x00000200U)
#define CSL_IOMUX_RGMII1_TD3_CFG_REG_PUPDSEL_SHIFT                             (0x00000009U)
#define CSL_IOMUX_RGMII1_TD3_CFG_REG_PUPDSEL_RESETVAL                          (0x00000000U)
#define CSL_IOMUX_RGMII1_TD3_CFG_REG_PUPDSEL_MAX                               (0x00000001U)

#define CSL_IOMUX_RGMII1_TD3_CFG_REG_SC1_MASK                                  (0x00000400U)
#define CSL_IOMUX_RGMII1_TD3_CFG_REG_SC1_SHIFT                                 (0x0000000AU)
#define CSL_IOMUX_RGMII1_TD3_CFG_REG_SC1_RESETVAL                              (0x00000001U)
#define CSL_IOMUX_RGMII1_TD3_CFG_REG_SC1_MAX                                   (0x00000001U)

#define CSL_IOMUX_RGMII1_TD3_CFG_REG_GPIO_SEL_MASK                             (0x00030000U)
#define CSL_IOMUX_RGMII1_TD3_CFG_REG_GPIO_SEL_SHIFT                            (0x00000010U)
#define CSL_IOMUX_RGMII1_TD3_CFG_REG_GPIO_SEL_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_RGMII1_TD3_CFG_REG_GPIO_SEL_MAX                              (0x00000003U)

#define CSL_IOMUX_RGMII1_TD3_CFG_REG_QUAL_SEL_MASK                             (0x000C0000U)
#define CSL_IOMUX_RGMII1_TD3_CFG_REG_QUAL_SEL_SHIFT                            (0x00000012U)
#define CSL_IOMUX_RGMII1_TD3_CFG_REG_QUAL_SEL_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_RGMII1_TD3_CFG_REG_QUAL_SEL_MAX                              (0x00000003U)

#define CSL_IOMUX_RGMII1_TD3_CFG_REG_INP_INV_SEL_MASK                          (0x00100000U)
#define CSL_IOMUX_RGMII1_TD3_CFG_REG_INP_INV_SEL_SHIFT                         (0x00000014U)
#define CSL_IOMUX_RGMII1_TD3_CFG_REG_INP_INV_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_RGMII1_TD3_CFG_REG_INP_INV_SEL_MAX                           (0x00000001U)

#define CSL_IOMUX_RGMII1_TD3_CFG_REG_ICSSM_GPIO_SEL_MASK                       (0x00200000U)
#define CSL_IOMUX_RGMII1_TD3_CFG_REG_ICSSM_GPIO_SEL_SHIFT                      (0x00000015U)
#define CSL_IOMUX_RGMII1_TD3_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                   (0x00000000U)
#define CSL_IOMUX_RGMII1_TD3_CFG_REG_ICSSM_GPIO_SEL_MAX                        (0x00000001U)

#define CSL_IOMUX_RGMII1_TD3_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                  (0x00400000U)
#define CSL_IOMUX_RGMII1_TD3_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                 (0x00000016U)
#define CSL_IOMUX_RGMII1_TD3_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL              (0x00000000U)
#define CSL_IOMUX_RGMII1_TD3_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                   (0x00000001U)

#define CSL_IOMUX_RGMII1_TD3_CFG_REG_HSMODE_MASK                               (0x40000000U)
#define CSL_IOMUX_RGMII1_TD3_CFG_REG_HSMODE_SHIFT                              (0x0000001EU)
#define CSL_IOMUX_RGMII1_TD3_CFG_REG_HSMODE_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_RGMII1_TD3_CFG_REG_HSMODE_MAX                                (0x00000001U)

#define CSL_IOMUX_RGMII1_TD3_CFG_REG_HSMASTER_MASK                             (0x80000000U)
#define CSL_IOMUX_RGMII1_TD3_CFG_REG_HSMASTER_SHIFT                            (0x0000001FU)
#define CSL_IOMUX_RGMII1_TD3_CFG_REG_HSMASTER_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_RGMII1_TD3_CFG_REG_HSMASTER_MAX                              (0x00000001U)

#define CSL_IOMUX_RGMII1_TD3_CFG_REG_RESETVAL                                  (0x000005F7U)

/* MDIO0_MDIO_CFG_REG */

#define CSL_IOMUX_MDIO0_MDIO_CFG_REG_FUNC_SEL_MASK                             (0x0000000FU)
#define CSL_IOMUX_MDIO0_MDIO_CFG_REG_FUNC_SEL_SHIFT                            (0x00000000U)
#define CSL_IOMUX_MDIO0_MDIO_CFG_REG_FUNC_SEL_RESETVAL                         (0x00000007U)
#define CSL_IOMUX_MDIO0_MDIO_CFG_REG_FUNC_SEL_MAX                              (0x0000000FU)

#define CSL_IOMUX_MDIO0_MDIO_CFG_REG_IE_OVERRIDE_CTRL_MASK                     (0x00000010U)
#define CSL_IOMUX_MDIO0_MDIO_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                    (0x00000004U)
#define CSL_IOMUX_MDIO0_MDIO_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_IOMUX_MDIO0_MDIO_CFG_REG_IE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_IOMUX_MDIO0_MDIO_CFG_REG_IE_OVERRIDE_MASK                          (0x00000020U)
#define CSL_IOMUX_MDIO0_MDIO_CFG_REG_IE_OVERRIDE_SHIFT                         (0x00000005U)
#define CSL_IOMUX_MDIO0_MDIO_CFG_REG_IE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_IOMUX_MDIO0_MDIO_CFG_REG_IE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_IOMUX_MDIO0_MDIO_CFG_REG_OE_OVERRIDE_CTRL_MASK                     (0x00000040U)
#define CSL_IOMUX_MDIO0_MDIO_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                    (0x00000006U)
#define CSL_IOMUX_MDIO0_MDIO_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_IOMUX_MDIO0_MDIO_CFG_REG_OE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_IOMUX_MDIO0_MDIO_CFG_REG_OE_OVERRIDE_MASK                          (0x00000080U)
#define CSL_IOMUX_MDIO0_MDIO_CFG_REG_OE_OVERRIDE_SHIFT                         (0x00000007U)
#define CSL_IOMUX_MDIO0_MDIO_CFG_REG_OE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_IOMUX_MDIO0_MDIO_CFG_REG_OE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_IOMUX_MDIO0_MDIO_CFG_REG_PI_MASK                                   (0x00000100U)
#define CSL_IOMUX_MDIO0_MDIO_CFG_REG_PI_SHIFT                                  (0x00000008U)
#define CSL_IOMUX_MDIO0_MDIO_CFG_REG_PI_RESETVAL                               (0x00000001U)
#define CSL_IOMUX_MDIO0_MDIO_CFG_REG_PI_MAX                                    (0x00000001U)

#define CSL_IOMUX_MDIO0_MDIO_CFG_REG_PUPDSEL_MASK                              (0x00000200U)
#define CSL_IOMUX_MDIO0_MDIO_CFG_REG_PUPDSEL_SHIFT                             (0x00000009U)
#define CSL_IOMUX_MDIO0_MDIO_CFG_REG_PUPDSEL_RESETVAL                          (0x00000000U)
#define CSL_IOMUX_MDIO0_MDIO_CFG_REG_PUPDSEL_MAX                               (0x00000001U)

#define CSL_IOMUX_MDIO0_MDIO_CFG_REG_SC1_MASK                                  (0x00000400U)
#define CSL_IOMUX_MDIO0_MDIO_CFG_REG_SC1_SHIFT                                 (0x0000000AU)
#define CSL_IOMUX_MDIO0_MDIO_CFG_REG_SC1_RESETVAL                              (0x00000001U)
#define CSL_IOMUX_MDIO0_MDIO_CFG_REG_SC1_MAX                                   (0x00000001U)

#define CSL_IOMUX_MDIO0_MDIO_CFG_REG_GPIO_SEL_MASK                             (0x00030000U)
#define CSL_IOMUX_MDIO0_MDIO_CFG_REG_GPIO_SEL_SHIFT                            (0x00000010U)
#define CSL_IOMUX_MDIO0_MDIO_CFG_REG_GPIO_SEL_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_MDIO0_MDIO_CFG_REG_GPIO_SEL_MAX                              (0x00000003U)

#define CSL_IOMUX_MDIO0_MDIO_CFG_REG_QUAL_SEL_MASK                             (0x000C0000U)
#define CSL_IOMUX_MDIO0_MDIO_CFG_REG_QUAL_SEL_SHIFT                            (0x00000012U)
#define CSL_IOMUX_MDIO0_MDIO_CFG_REG_QUAL_SEL_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_MDIO0_MDIO_CFG_REG_QUAL_SEL_MAX                              (0x00000003U)

#define CSL_IOMUX_MDIO0_MDIO_CFG_REG_INP_INV_SEL_MASK                          (0x00100000U)
#define CSL_IOMUX_MDIO0_MDIO_CFG_REG_INP_INV_SEL_SHIFT                         (0x00000014U)
#define CSL_IOMUX_MDIO0_MDIO_CFG_REG_INP_INV_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_MDIO0_MDIO_CFG_REG_INP_INV_SEL_MAX                           (0x00000001U)

#define CSL_IOMUX_MDIO0_MDIO_CFG_REG_ICSSM_GPIO_SEL_MASK                       (0x00200000U)
#define CSL_IOMUX_MDIO0_MDIO_CFG_REG_ICSSM_GPIO_SEL_SHIFT                      (0x00000015U)
#define CSL_IOMUX_MDIO0_MDIO_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                   (0x00000000U)
#define CSL_IOMUX_MDIO0_MDIO_CFG_REG_ICSSM_GPIO_SEL_MAX                        (0x00000001U)

#define CSL_IOMUX_MDIO0_MDIO_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                  (0x00400000U)
#define CSL_IOMUX_MDIO0_MDIO_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                 (0x00000016U)
#define CSL_IOMUX_MDIO0_MDIO_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL              (0x00000000U)
#define CSL_IOMUX_MDIO0_MDIO_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                   (0x00000001U)

#define CSL_IOMUX_MDIO0_MDIO_CFG_REG_HSMODE_MASK                               (0x40000000U)
#define CSL_IOMUX_MDIO0_MDIO_CFG_REG_HSMODE_SHIFT                              (0x0000001EU)
#define CSL_IOMUX_MDIO0_MDIO_CFG_REG_HSMODE_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_MDIO0_MDIO_CFG_REG_HSMODE_MAX                                (0x00000001U)

#define CSL_IOMUX_MDIO0_MDIO_CFG_REG_HSMASTER_MASK                             (0x80000000U)
#define CSL_IOMUX_MDIO0_MDIO_CFG_REG_HSMASTER_SHIFT                            (0x0000001FU)
#define CSL_IOMUX_MDIO0_MDIO_CFG_REG_HSMASTER_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_MDIO0_MDIO_CFG_REG_HSMASTER_MAX                              (0x00000001U)

#define CSL_IOMUX_MDIO0_MDIO_CFG_REG_RESETVAL                                  (0x000005F7U)

/* MDIO0_MDC_CFG_REG */

#define CSL_IOMUX_MDIO0_MDC_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_IOMUX_MDIO0_MDC_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_IOMUX_MDIO0_MDC_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000007U)
#define CSL_IOMUX_MDIO0_MDC_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_IOMUX_MDIO0_MDC_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_IOMUX_MDIO0_MDC_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_IOMUX_MDIO0_MDC_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_IOMUX_MDIO0_MDC_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_IOMUX_MDIO0_MDC_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_IOMUX_MDIO0_MDC_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_IOMUX_MDIO0_MDC_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_IOMUX_MDIO0_MDC_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_IOMUX_MDIO0_MDC_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_IOMUX_MDIO0_MDC_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_IOMUX_MDIO0_MDC_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_IOMUX_MDIO0_MDC_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_IOMUX_MDIO0_MDC_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_IOMUX_MDIO0_MDC_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_IOMUX_MDIO0_MDC_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_IOMUX_MDIO0_MDC_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_IOMUX_MDIO0_MDC_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_IOMUX_MDIO0_MDC_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_IOMUX_MDIO0_MDC_CFG_REG_PI_RESETVAL                                (0x00000001U)
#define CSL_IOMUX_MDIO0_MDC_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_IOMUX_MDIO0_MDC_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_IOMUX_MDIO0_MDC_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_IOMUX_MDIO0_MDC_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_MDIO0_MDC_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_IOMUX_MDIO0_MDC_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_IOMUX_MDIO0_MDC_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_IOMUX_MDIO0_MDC_CFG_REG_SC1_RESETVAL                               (0x00000001U)
#define CSL_IOMUX_MDIO0_MDC_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_IOMUX_MDIO0_MDC_CFG_REG_GPIO_SEL_MASK                              (0x00030000U)
#define CSL_IOMUX_MDIO0_MDC_CFG_REG_GPIO_SEL_SHIFT                             (0x00000010U)
#define CSL_IOMUX_MDIO0_MDC_CFG_REG_GPIO_SEL_RESETVAL                          (0x00000000U)
#define CSL_IOMUX_MDIO0_MDC_CFG_REG_GPIO_SEL_MAX                               (0x00000003U)

#define CSL_IOMUX_MDIO0_MDC_CFG_REG_QUAL_SEL_MASK                              (0x000C0000U)
#define CSL_IOMUX_MDIO0_MDC_CFG_REG_QUAL_SEL_SHIFT                             (0x00000012U)
#define CSL_IOMUX_MDIO0_MDC_CFG_REG_QUAL_SEL_RESETVAL                          (0x00000000U)
#define CSL_IOMUX_MDIO0_MDC_CFG_REG_QUAL_SEL_MAX                               (0x00000003U)

#define CSL_IOMUX_MDIO0_MDC_CFG_REG_INP_INV_SEL_MASK                           (0x00100000U)
#define CSL_IOMUX_MDIO0_MDC_CFG_REG_INP_INV_SEL_SHIFT                          (0x00000014U)
#define CSL_IOMUX_MDIO0_MDC_CFG_REG_INP_INV_SEL_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_MDIO0_MDC_CFG_REG_INP_INV_SEL_MAX                            (0x00000001U)

#define CSL_IOMUX_MDIO0_MDC_CFG_REG_ICSSM_GPIO_SEL_MASK                        (0x00200000U)
#define CSL_IOMUX_MDIO0_MDC_CFG_REG_ICSSM_GPIO_SEL_SHIFT                       (0x00000015U)
#define CSL_IOMUX_MDIO0_MDC_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                    (0x00000000U)
#define CSL_IOMUX_MDIO0_MDC_CFG_REG_ICSSM_GPIO_SEL_MAX                         (0x00000001U)

#define CSL_IOMUX_MDIO0_MDC_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                   (0x00400000U)
#define CSL_IOMUX_MDIO0_MDC_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                  (0x00000016U)
#define CSL_IOMUX_MDIO0_MDC_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL               (0x00000000U)
#define CSL_IOMUX_MDIO0_MDC_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                    (0x00000001U)

#define CSL_IOMUX_MDIO0_MDC_CFG_REG_HSMODE_MASK                                (0x40000000U)
#define CSL_IOMUX_MDIO0_MDC_CFG_REG_HSMODE_SHIFT                               (0x0000001EU)
#define CSL_IOMUX_MDIO0_MDC_CFG_REG_HSMODE_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_MDIO0_MDC_CFG_REG_HSMODE_MAX                                 (0x00000001U)

#define CSL_IOMUX_MDIO0_MDC_CFG_REG_HSMASTER_MASK                              (0x80000000U)
#define CSL_IOMUX_MDIO0_MDC_CFG_REG_HSMASTER_SHIFT                             (0x0000001FU)
#define CSL_IOMUX_MDIO0_MDC_CFG_REG_HSMASTER_RESETVAL                          (0x00000000U)
#define CSL_IOMUX_MDIO0_MDC_CFG_REG_HSMASTER_MAX                               (0x00000001U)

#define CSL_IOMUX_MDIO0_MDC_CFG_REG_RESETVAL                                   (0x000005F7U)

/* EPWM0_A_CFG_REG */

#define CSL_IOMUX_EPWM0_A_CFG_REG_FUNC_SEL_MASK                                (0x0000000FU)
#define CSL_IOMUX_EPWM0_A_CFG_REG_FUNC_SEL_SHIFT                               (0x00000000U)
#define CSL_IOMUX_EPWM0_A_CFG_REG_FUNC_SEL_RESETVAL                            (0x00000007U)
#define CSL_IOMUX_EPWM0_A_CFG_REG_FUNC_SEL_MAX                                 (0x0000000FU)

#define CSL_IOMUX_EPWM0_A_CFG_REG_IE_OVERRIDE_CTRL_MASK                        (0x00000010U)
#define CSL_IOMUX_EPWM0_A_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                       (0x00000004U)
#define CSL_IOMUX_EPWM0_A_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_EPWM0_A_CFG_REG_IE_OVERRIDE_CTRL_MAX                         (0x00000001U)

#define CSL_IOMUX_EPWM0_A_CFG_REG_IE_OVERRIDE_MASK                             (0x00000020U)
#define CSL_IOMUX_EPWM0_A_CFG_REG_IE_OVERRIDE_SHIFT                            (0x00000005U)
#define CSL_IOMUX_EPWM0_A_CFG_REG_IE_OVERRIDE_RESETVAL                         (0x00000001U)
#define CSL_IOMUX_EPWM0_A_CFG_REG_IE_OVERRIDE_MAX                              (0x00000001U)

#define CSL_IOMUX_EPWM0_A_CFG_REG_OE_OVERRIDE_CTRL_MASK                        (0x00000040U)
#define CSL_IOMUX_EPWM0_A_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                       (0x00000006U)
#define CSL_IOMUX_EPWM0_A_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_EPWM0_A_CFG_REG_OE_OVERRIDE_CTRL_MAX                         (0x00000001U)

#define CSL_IOMUX_EPWM0_A_CFG_REG_OE_OVERRIDE_MASK                             (0x00000080U)
#define CSL_IOMUX_EPWM0_A_CFG_REG_OE_OVERRIDE_SHIFT                            (0x00000007U)
#define CSL_IOMUX_EPWM0_A_CFG_REG_OE_OVERRIDE_RESETVAL                         (0x00000001U)
#define CSL_IOMUX_EPWM0_A_CFG_REG_OE_OVERRIDE_MAX                              (0x00000001U)

#define CSL_IOMUX_EPWM0_A_CFG_REG_PI_MASK                                      (0x00000100U)
#define CSL_IOMUX_EPWM0_A_CFG_REG_PI_SHIFT                                     (0x00000008U)
#define CSL_IOMUX_EPWM0_A_CFG_REG_PI_RESETVAL                                  (0x00000001U)
#define CSL_IOMUX_EPWM0_A_CFG_REG_PI_MAX                                       (0x00000001U)

#define CSL_IOMUX_EPWM0_A_CFG_REG_PUPDSEL_MASK                                 (0x00000200U)
#define CSL_IOMUX_EPWM0_A_CFG_REG_PUPDSEL_SHIFT                                (0x00000009U)
#define CSL_IOMUX_EPWM0_A_CFG_REG_PUPDSEL_RESETVAL                             (0x00000000U)
#define CSL_IOMUX_EPWM0_A_CFG_REG_PUPDSEL_MAX                                  (0x00000001U)

#define CSL_IOMUX_EPWM0_A_CFG_REG_SC1_MASK                                     (0x00000400U)
#define CSL_IOMUX_EPWM0_A_CFG_REG_SC1_SHIFT                                    (0x0000000AU)
#define CSL_IOMUX_EPWM0_A_CFG_REG_SC1_RESETVAL                                 (0x00000001U)
#define CSL_IOMUX_EPWM0_A_CFG_REG_SC1_MAX                                      (0x00000001U)

#define CSL_IOMUX_EPWM0_A_CFG_REG_GPIO_SEL_MASK                                (0x00030000U)
#define CSL_IOMUX_EPWM0_A_CFG_REG_GPIO_SEL_SHIFT                               (0x00000010U)
#define CSL_IOMUX_EPWM0_A_CFG_REG_GPIO_SEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_EPWM0_A_CFG_REG_GPIO_SEL_MAX                                 (0x00000003U)

#define CSL_IOMUX_EPWM0_A_CFG_REG_QUAL_SEL_MASK                                (0x000C0000U)
#define CSL_IOMUX_EPWM0_A_CFG_REG_QUAL_SEL_SHIFT                               (0x00000012U)
#define CSL_IOMUX_EPWM0_A_CFG_REG_QUAL_SEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_EPWM0_A_CFG_REG_QUAL_SEL_MAX                                 (0x00000003U)

#define CSL_IOMUX_EPWM0_A_CFG_REG_INP_INV_SEL_MASK                             (0x00100000U)
#define CSL_IOMUX_EPWM0_A_CFG_REG_INP_INV_SEL_SHIFT                            (0x00000014U)
#define CSL_IOMUX_EPWM0_A_CFG_REG_INP_INV_SEL_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_EPWM0_A_CFG_REG_INP_INV_SEL_MAX                              (0x00000001U)

#define CSL_IOMUX_EPWM0_A_CFG_REG_ICSSM_GPIO_SEL_MASK                          (0x00200000U)
#define CSL_IOMUX_EPWM0_A_CFG_REG_ICSSM_GPIO_SEL_SHIFT                         (0x00000015U)
#define CSL_IOMUX_EPWM0_A_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_EPWM0_A_CFG_REG_ICSSM_GPIO_SEL_MAX                           (0x00000001U)

#define CSL_IOMUX_EPWM0_A_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                     (0x00400000U)
#define CSL_IOMUX_EPWM0_A_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                    (0x00000016U)
#define CSL_IOMUX_EPWM0_A_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL                 (0x00000000U)
#define CSL_IOMUX_EPWM0_A_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                      (0x00000001U)

#define CSL_IOMUX_EPWM0_A_CFG_REG_HSMODE_MASK                                  (0x40000000U)
#define CSL_IOMUX_EPWM0_A_CFG_REG_HSMODE_SHIFT                                 (0x0000001EU)
#define CSL_IOMUX_EPWM0_A_CFG_REG_HSMODE_RESETVAL                              (0x00000000U)
#define CSL_IOMUX_EPWM0_A_CFG_REG_HSMODE_MAX                                   (0x00000001U)

#define CSL_IOMUX_EPWM0_A_CFG_REG_HSMASTER_MASK                                (0x80000000U)
#define CSL_IOMUX_EPWM0_A_CFG_REG_HSMASTER_SHIFT                               (0x0000001FU)
#define CSL_IOMUX_EPWM0_A_CFG_REG_HSMASTER_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_EPWM0_A_CFG_REG_HSMASTER_MAX                                 (0x00000001U)

#define CSL_IOMUX_EPWM0_A_CFG_REG_RESETVAL                                     (0x000005F7U)

/* EPWM0_B_CFG_REG */

#define CSL_IOMUX_EPWM0_B_CFG_REG_FUNC_SEL_MASK                                (0x0000000FU)
#define CSL_IOMUX_EPWM0_B_CFG_REG_FUNC_SEL_SHIFT                               (0x00000000U)
#define CSL_IOMUX_EPWM0_B_CFG_REG_FUNC_SEL_RESETVAL                            (0x00000007U)
#define CSL_IOMUX_EPWM0_B_CFG_REG_FUNC_SEL_MAX                                 (0x0000000FU)

#define CSL_IOMUX_EPWM0_B_CFG_REG_IE_OVERRIDE_CTRL_MASK                        (0x00000010U)
#define CSL_IOMUX_EPWM0_B_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                       (0x00000004U)
#define CSL_IOMUX_EPWM0_B_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_EPWM0_B_CFG_REG_IE_OVERRIDE_CTRL_MAX                         (0x00000001U)

#define CSL_IOMUX_EPWM0_B_CFG_REG_IE_OVERRIDE_MASK                             (0x00000020U)
#define CSL_IOMUX_EPWM0_B_CFG_REG_IE_OVERRIDE_SHIFT                            (0x00000005U)
#define CSL_IOMUX_EPWM0_B_CFG_REG_IE_OVERRIDE_RESETVAL                         (0x00000001U)
#define CSL_IOMUX_EPWM0_B_CFG_REG_IE_OVERRIDE_MAX                              (0x00000001U)

#define CSL_IOMUX_EPWM0_B_CFG_REG_OE_OVERRIDE_CTRL_MASK                        (0x00000040U)
#define CSL_IOMUX_EPWM0_B_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                       (0x00000006U)
#define CSL_IOMUX_EPWM0_B_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_EPWM0_B_CFG_REG_OE_OVERRIDE_CTRL_MAX                         (0x00000001U)

#define CSL_IOMUX_EPWM0_B_CFG_REG_OE_OVERRIDE_MASK                             (0x00000080U)
#define CSL_IOMUX_EPWM0_B_CFG_REG_OE_OVERRIDE_SHIFT                            (0x00000007U)
#define CSL_IOMUX_EPWM0_B_CFG_REG_OE_OVERRIDE_RESETVAL                         (0x00000001U)
#define CSL_IOMUX_EPWM0_B_CFG_REG_OE_OVERRIDE_MAX                              (0x00000001U)

#define CSL_IOMUX_EPWM0_B_CFG_REG_PI_MASK                                      (0x00000100U)
#define CSL_IOMUX_EPWM0_B_CFG_REG_PI_SHIFT                                     (0x00000008U)
#define CSL_IOMUX_EPWM0_B_CFG_REG_PI_RESETVAL                                  (0x00000001U)
#define CSL_IOMUX_EPWM0_B_CFG_REG_PI_MAX                                       (0x00000001U)

#define CSL_IOMUX_EPWM0_B_CFG_REG_PUPDSEL_MASK                                 (0x00000200U)
#define CSL_IOMUX_EPWM0_B_CFG_REG_PUPDSEL_SHIFT                                (0x00000009U)
#define CSL_IOMUX_EPWM0_B_CFG_REG_PUPDSEL_RESETVAL                             (0x00000000U)
#define CSL_IOMUX_EPWM0_B_CFG_REG_PUPDSEL_MAX                                  (0x00000001U)

#define CSL_IOMUX_EPWM0_B_CFG_REG_SC1_MASK                                     (0x00000400U)
#define CSL_IOMUX_EPWM0_B_CFG_REG_SC1_SHIFT                                    (0x0000000AU)
#define CSL_IOMUX_EPWM0_B_CFG_REG_SC1_RESETVAL                                 (0x00000001U)
#define CSL_IOMUX_EPWM0_B_CFG_REG_SC1_MAX                                      (0x00000001U)

#define CSL_IOMUX_EPWM0_B_CFG_REG_GPIO_SEL_MASK                                (0x00030000U)
#define CSL_IOMUX_EPWM0_B_CFG_REG_GPIO_SEL_SHIFT                               (0x00000010U)
#define CSL_IOMUX_EPWM0_B_CFG_REG_GPIO_SEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_EPWM0_B_CFG_REG_GPIO_SEL_MAX                                 (0x00000003U)

#define CSL_IOMUX_EPWM0_B_CFG_REG_QUAL_SEL_MASK                                (0x000C0000U)
#define CSL_IOMUX_EPWM0_B_CFG_REG_QUAL_SEL_SHIFT                               (0x00000012U)
#define CSL_IOMUX_EPWM0_B_CFG_REG_QUAL_SEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_EPWM0_B_CFG_REG_QUAL_SEL_MAX                                 (0x00000003U)

#define CSL_IOMUX_EPWM0_B_CFG_REG_INP_INV_SEL_MASK                             (0x00100000U)
#define CSL_IOMUX_EPWM0_B_CFG_REG_INP_INV_SEL_SHIFT                            (0x00000014U)
#define CSL_IOMUX_EPWM0_B_CFG_REG_INP_INV_SEL_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_EPWM0_B_CFG_REG_INP_INV_SEL_MAX                              (0x00000001U)

#define CSL_IOMUX_EPWM0_B_CFG_REG_ICSSM_GPIO_SEL_MASK                          (0x00200000U)
#define CSL_IOMUX_EPWM0_B_CFG_REG_ICSSM_GPIO_SEL_SHIFT                         (0x00000015U)
#define CSL_IOMUX_EPWM0_B_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_EPWM0_B_CFG_REG_ICSSM_GPIO_SEL_MAX                           (0x00000001U)

#define CSL_IOMUX_EPWM0_B_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                     (0x00400000U)
#define CSL_IOMUX_EPWM0_B_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                    (0x00000016U)
#define CSL_IOMUX_EPWM0_B_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL                 (0x00000000U)
#define CSL_IOMUX_EPWM0_B_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                      (0x00000001U)

#define CSL_IOMUX_EPWM0_B_CFG_REG_HSMODE_MASK                                  (0x40000000U)
#define CSL_IOMUX_EPWM0_B_CFG_REG_HSMODE_SHIFT                                 (0x0000001EU)
#define CSL_IOMUX_EPWM0_B_CFG_REG_HSMODE_RESETVAL                              (0x00000000U)
#define CSL_IOMUX_EPWM0_B_CFG_REG_HSMODE_MAX                                   (0x00000001U)

#define CSL_IOMUX_EPWM0_B_CFG_REG_HSMASTER_MASK                                (0x80000000U)
#define CSL_IOMUX_EPWM0_B_CFG_REG_HSMASTER_SHIFT                               (0x0000001FU)
#define CSL_IOMUX_EPWM0_B_CFG_REG_HSMASTER_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_EPWM0_B_CFG_REG_HSMASTER_MAX                                 (0x00000001U)

#define CSL_IOMUX_EPWM0_B_CFG_REG_RESETVAL                                     (0x000005F7U)

/* EPWM1_A_CFG_REG */

#define CSL_IOMUX_EPWM1_A_CFG_REG_FUNC_SEL_MASK                                (0x0000000FU)
#define CSL_IOMUX_EPWM1_A_CFG_REG_FUNC_SEL_SHIFT                               (0x00000000U)
#define CSL_IOMUX_EPWM1_A_CFG_REG_FUNC_SEL_RESETVAL                            (0x00000007U)
#define CSL_IOMUX_EPWM1_A_CFG_REG_FUNC_SEL_MAX                                 (0x0000000FU)

#define CSL_IOMUX_EPWM1_A_CFG_REG_IE_OVERRIDE_CTRL_MASK                        (0x00000010U)
#define CSL_IOMUX_EPWM1_A_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                       (0x00000004U)
#define CSL_IOMUX_EPWM1_A_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_EPWM1_A_CFG_REG_IE_OVERRIDE_CTRL_MAX                         (0x00000001U)

#define CSL_IOMUX_EPWM1_A_CFG_REG_IE_OVERRIDE_MASK                             (0x00000020U)
#define CSL_IOMUX_EPWM1_A_CFG_REG_IE_OVERRIDE_SHIFT                            (0x00000005U)
#define CSL_IOMUX_EPWM1_A_CFG_REG_IE_OVERRIDE_RESETVAL                         (0x00000001U)
#define CSL_IOMUX_EPWM1_A_CFG_REG_IE_OVERRIDE_MAX                              (0x00000001U)

#define CSL_IOMUX_EPWM1_A_CFG_REG_OE_OVERRIDE_CTRL_MASK                        (0x00000040U)
#define CSL_IOMUX_EPWM1_A_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                       (0x00000006U)
#define CSL_IOMUX_EPWM1_A_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_EPWM1_A_CFG_REG_OE_OVERRIDE_CTRL_MAX                         (0x00000001U)

#define CSL_IOMUX_EPWM1_A_CFG_REG_OE_OVERRIDE_MASK                             (0x00000080U)
#define CSL_IOMUX_EPWM1_A_CFG_REG_OE_OVERRIDE_SHIFT                            (0x00000007U)
#define CSL_IOMUX_EPWM1_A_CFG_REG_OE_OVERRIDE_RESETVAL                         (0x00000001U)
#define CSL_IOMUX_EPWM1_A_CFG_REG_OE_OVERRIDE_MAX                              (0x00000001U)

#define CSL_IOMUX_EPWM1_A_CFG_REG_PI_MASK                                      (0x00000100U)
#define CSL_IOMUX_EPWM1_A_CFG_REG_PI_SHIFT                                     (0x00000008U)
#define CSL_IOMUX_EPWM1_A_CFG_REG_PI_RESETVAL                                  (0x00000001U)
#define CSL_IOMUX_EPWM1_A_CFG_REG_PI_MAX                                       (0x00000001U)

#define CSL_IOMUX_EPWM1_A_CFG_REG_PUPDSEL_MASK                                 (0x00000200U)
#define CSL_IOMUX_EPWM1_A_CFG_REG_PUPDSEL_SHIFT                                (0x00000009U)
#define CSL_IOMUX_EPWM1_A_CFG_REG_PUPDSEL_RESETVAL                             (0x00000000U)
#define CSL_IOMUX_EPWM1_A_CFG_REG_PUPDSEL_MAX                                  (0x00000001U)

#define CSL_IOMUX_EPWM1_A_CFG_REG_SC1_MASK                                     (0x00000400U)
#define CSL_IOMUX_EPWM1_A_CFG_REG_SC1_SHIFT                                    (0x0000000AU)
#define CSL_IOMUX_EPWM1_A_CFG_REG_SC1_RESETVAL                                 (0x00000001U)
#define CSL_IOMUX_EPWM1_A_CFG_REG_SC1_MAX                                      (0x00000001U)

#define CSL_IOMUX_EPWM1_A_CFG_REG_GPIO_SEL_MASK                                (0x00030000U)
#define CSL_IOMUX_EPWM1_A_CFG_REG_GPIO_SEL_SHIFT                               (0x00000010U)
#define CSL_IOMUX_EPWM1_A_CFG_REG_GPIO_SEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_EPWM1_A_CFG_REG_GPIO_SEL_MAX                                 (0x00000003U)

#define CSL_IOMUX_EPWM1_A_CFG_REG_QUAL_SEL_MASK                                (0x000C0000U)
#define CSL_IOMUX_EPWM1_A_CFG_REG_QUAL_SEL_SHIFT                               (0x00000012U)
#define CSL_IOMUX_EPWM1_A_CFG_REG_QUAL_SEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_EPWM1_A_CFG_REG_QUAL_SEL_MAX                                 (0x00000003U)

#define CSL_IOMUX_EPWM1_A_CFG_REG_INP_INV_SEL_MASK                             (0x00100000U)
#define CSL_IOMUX_EPWM1_A_CFG_REG_INP_INV_SEL_SHIFT                            (0x00000014U)
#define CSL_IOMUX_EPWM1_A_CFG_REG_INP_INV_SEL_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_EPWM1_A_CFG_REG_INP_INV_SEL_MAX                              (0x00000001U)

#define CSL_IOMUX_EPWM1_A_CFG_REG_ICSSM_GPIO_SEL_MASK                          (0x00200000U)
#define CSL_IOMUX_EPWM1_A_CFG_REG_ICSSM_GPIO_SEL_SHIFT                         (0x00000015U)
#define CSL_IOMUX_EPWM1_A_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_EPWM1_A_CFG_REG_ICSSM_GPIO_SEL_MAX                           (0x00000001U)

#define CSL_IOMUX_EPWM1_A_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                     (0x00400000U)
#define CSL_IOMUX_EPWM1_A_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                    (0x00000016U)
#define CSL_IOMUX_EPWM1_A_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL                 (0x00000000U)
#define CSL_IOMUX_EPWM1_A_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                      (0x00000001U)

#define CSL_IOMUX_EPWM1_A_CFG_REG_HSMODE_MASK                                  (0x40000000U)
#define CSL_IOMUX_EPWM1_A_CFG_REG_HSMODE_SHIFT                                 (0x0000001EU)
#define CSL_IOMUX_EPWM1_A_CFG_REG_HSMODE_RESETVAL                              (0x00000000U)
#define CSL_IOMUX_EPWM1_A_CFG_REG_HSMODE_MAX                                   (0x00000001U)

#define CSL_IOMUX_EPWM1_A_CFG_REG_HSMASTER_MASK                                (0x80000000U)
#define CSL_IOMUX_EPWM1_A_CFG_REG_HSMASTER_SHIFT                               (0x0000001FU)
#define CSL_IOMUX_EPWM1_A_CFG_REG_HSMASTER_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_EPWM1_A_CFG_REG_HSMASTER_MAX                                 (0x00000001U)

#define CSL_IOMUX_EPWM1_A_CFG_REG_RESETVAL                                     (0x000005F7U)

/* EPWM1_B_CFG_REG */

#define CSL_IOMUX_EPWM1_B_CFG_REG_FUNC_SEL_MASK                                (0x0000000FU)
#define CSL_IOMUX_EPWM1_B_CFG_REG_FUNC_SEL_SHIFT                               (0x00000000U)
#define CSL_IOMUX_EPWM1_B_CFG_REG_FUNC_SEL_RESETVAL                            (0x00000007U)
#define CSL_IOMUX_EPWM1_B_CFG_REG_FUNC_SEL_MAX                                 (0x0000000FU)

#define CSL_IOMUX_EPWM1_B_CFG_REG_IE_OVERRIDE_CTRL_MASK                        (0x00000010U)
#define CSL_IOMUX_EPWM1_B_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                       (0x00000004U)
#define CSL_IOMUX_EPWM1_B_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_EPWM1_B_CFG_REG_IE_OVERRIDE_CTRL_MAX                         (0x00000001U)

#define CSL_IOMUX_EPWM1_B_CFG_REG_IE_OVERRIDE_MASK                             (0x00000020U)
#define CSL_IOMUX_EPWM1_B_CFG_REG_IE_OVERRIDE_SHIFT                            (0x00000005U)
#define CSL_IOMUX_EPWM1_B_CFG_REG_IE_OVERRIDE_RESETVAL                         (0x00000001U)
#define CSL_IOMUX_EPWM1_B_CFG_REG_IE_OVERRIDE_MAX                              (0x00000001U)

#define CSL_IOMUX_EPWM1_B_CFG_REG_OE_OVERRIDE_CTRL_MASK                        (0x00000040U)
#define CSL_IOMUX_EPWM1_B_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                       (0x00000006U)
#define CSL_IOMUX_EPWM1_B_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_EPWM1_B_CFG_REG_OE_OVERRIDE_CTRL_MAX                         (0x00000001U)

#define CSL_IOMUX_EPWM1_B_CFG_REG_OE_OVERRIDE_MASK                             (0x00000080U)
#define CSL_IOMUX_EPWM1_B_CFG_REG_OE_OVERRIDE_SHIFT                            (0x00000007U)
#define CSL_IOMUX_EPWM1_B_CFG_REG_OE_OVERRIDE_RESETVAL                         (0x00000001U)
#define CSL_IOMUX_EPWM1_B_CFG_REG_OE_OVERRIDE_MAX                              (0x00000001U)

#define CSL_IOMUX_EPWM1_B_CFG_REG_PI_MASK                                      (0x00000100U)
#define CSL_IOMUX_EPWM1_B_CFG_REG_PI_SHIFT                                     (0x00000008U)
#define CSL_IOMUX_EPWM1_B_CFG_REG_PI_RESETVAL                                  (0x00000001U)
#define CSL_IOMUX_EPWM1_B_CFG_REG_PI_MAX                                       (0x00000001U)

#define CSL_IOMUX_EPWM1_B_CFG_REG_PUPDSEL_MASK                                 (0x00000200U)
#define CSL_IOMUX_EPWM1_B_CFG_REG_PUPDSEL_SHIFT                                (0x00000009U)
#define CSL_IOMUX_EPWM1_B_CFG_REG_PUPDSEL_RESETVAL                             (0x00000000U)
#define CSL_IOMUX_EPWM1_B_CFG_REG_PUPDSEL_MAX                                  (0x00000001U)

#define CSL_IOMUX_EPWM1_B_CFG_REG_SC1_MASK                                     (0x00000400U)
#define CSL_IOMUX_EPWM1_B_CFG_REG_SC1_SHIFT                                    (0x0000000AU)
#define CSL_IOMUX_EPWM1_B_CFG_REG_SC1_RESETVAL                                 (0x00000001U)
#define CSL_IOMUX_EPWM1_B_CFG_REG_SC1_MAX                                      (0x00000001U)

#define CSL_IOMUX_EPWM1_B_CFG_REG_GPIO_SEL_MASK                                (0x00030000U)
#define CSL_IOMUX_EPWM1_B_CFG_REG_GPIO_SEL_SHIFT                               (0x00000010U)
#define CSL_IOMUX_EPWM1_B_CFG_REG_GPIO_SEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_EPWM1_B_CFG_REG_GPIO_SEL_MAX                                 (0x00000003U)

#define CSL_IOMUX_EPWM1_B_CFG_REG_QUAL_SEL_MASK                                (0x000C0000U)
#define CSL_IOMUX_EPWM1_B_CFG_REG_QUAL_SEL_SHIFT                               (0x00000012U)
#define CSL_IOMUX_EPWM1_B_CFG_REG_QUAL_SEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_EPWM1_B_CFG_REG_QUAL_SEL_MAX                                 (0x00000003U)

#define CSL_IOMUX_EPWM1_B_CFG_REG_INP_INV_SEL_MASK                             (0x00100000U)
#define CSL_IOMUX_EPWM1_B_CFG_REG_INP_INV_SEL_SHIFT                            (0x00000014U)
#define CSL_IOMUX_EPWM1_B_CFG_REG_INP_INV_SEL_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_EPWM1_B_CFG_REG_INP_INV_SEL_MAX                              (0x00000001U)

#define CSL_IOMUX_EPWM1_B_CFG_REG_ICSSM_GPIO_SEL_MASK                          (0x00200000U)
#define CSL_IOMUX_EPWM1_B_CFG_REG_ICSSM_GPIO_SEL_SHIFT                         (0x00000015U)
#define CSL_IOMUX_EPWM1_B_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_EPWM1_B_CFG_REG_ICSSM_GPIO_SEL_MAX                           (0x00000001U)

#define CSL_IOMUX_EPWM1_B_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                     (0x00400000U)
#define CSL_IOMUX_EPWM1_B_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                    (0x00000016U)
#define CSL_IOMUX_EPWM1_B_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL                 (0x00000000U)
#define CSL_IOMUX_EPWM1_B_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                      (0x00000001U)

#define CSL_IOMUX_EPWM1_B_CFG_REG_HSMODE_MASK                                  (0x40000000U)
#define CSL_IOMUX_EPWM1_B_CFG_REG_HSMODE_SHIFT                                 (0x0000001EU)
#define CSL_IOMUX_EPWM1_B_CFG_REG_HSMODE_RESETVAL                              (0x00000000U)
#define CSL_IOMUX_EPWM1_B_CFG_REG_HSMODE_MAX                                   (0x00000001U)

#define CSL_IOMUX_EPWM1_B_CFG_REG_HSMASTER_MASK                                (0x80000000U)
#define CSL_IOMUX_EPWM1_B_CFG_REG_HSMASTER_SHIFT                               (0x0000001FU)
#define CSL_IOMUX_EPWM1_B_CFG_REG_HSMASTER_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_EPWM1_B_CFG_REG_HSMASTER_MAX                                 (0x00000001U)

#define CSL_IOMUX_EPWM1_B_CFG_REG_RESETVAL                                     (0x000005F7U)

/* EPWM2_A_CFG_REG */

#define CSL_IOMUX_EPWM2_A_CFG_REG_FUNC_SEL_MASK                                (0x0000000FU)
#define CSL_IOMUX_EPWM2_A_CFG_REG_FUNC_SEL_SHIFT                               (0x00000000U)
#define CSL_IOMUX_EPWM2_A_CFG_REG_FUNC_SEL_RESETVAL                            (0x00000007U)
#define CSL_IOMUX_EPWM2_A_CFG_REG_FUNC_SEL_MAX                                 (0x0000000FU)

#define CSL_IOMUX_EPWM2_A_CFG_REG_IE_OVERRIDE_CTRL_MASK                        (0x00000010U)
#define CSL_IOMUX_EPWM2_A_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                       (0x00000004U)
#define CSL_IOMUX_EPWM2_A_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_EPWM2_A_CFG_REG_IE_OVERRIDE_CTRL_MAX                         (0x00000001U)

#define CSL_IOMUX_EPWM2_A_CFG_REG_IE_OVERRIDE_MASK                             (0x00000020U)
#define CSL_IOMUX_EPWM2_A_CFG_REG_IE_OVERRIDE_SHIFT                            (0x00000005U)
#define CSL_IOMUX_EPWM2_A_CFG_REG_IE_OVERRIDE_RESETVAL                         (0x00000001U)
#define CSL_IOMUX_EPWM2_A_CFG_REG_IE_OVERRIDE_MAX                              (0x00000001U)

#define CSL_IOMUX_EPWM2_A_CFG_REG_OE_OVERRIDE_CTRL_MASK                        (0x00000040U)
#define CSL_IOMUX_EPWM2_A_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                       (0x00000006U)
#define CSL_IOMUX_EPWM2_A_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_EPWM2_A_CFG_REG_OE_OVERRIDE_CTRL_MAX                         (0x00000001U)

#define CSL_IOMUX_EPWM2_A_CFG_REG_OE_OVERRIDE_MASK                             (0x00000080U)
#define CSL_IOMUX_EPWM2_A_CFG_REG_OE_OVERRIDE_SHIFT                            (0x00000007U)
#define CSL_IOMUX_EPWM2_A_CFG_REG_OE_OVERRIDE_RESETVAL                         (0x00000001U)
#define CSL_IOMUX_EPWM2_A_CFG_REG_OE_OVERRIDE_MAX                              (0x00000001U)

#define CSL_IOMUX_EPWM2_A_CFG_REG_PI_MASK                                      (0x00000100U)
#define CSL_IOMUX_EPWM2_A_CFG_REG_PI_SHIFT                                     (0x00000008U)
#define CSL_IOMUX_EPWM2_A_CFG_REG_PI_RESETVAL                                  (0x00000001U)
#define CSL_IOMUX_EPWM2_A_CFG_REG_PI_MAX                                       (0x00000001U)

#define CSL_IOMUX_EPWM2_A_CFG_REG_PUPDSEL_MASK                                 (0x00000200U)
#define CSL_IOMUX_EPWM2_A_CFG_REG_PUPDSEL_SHIFT                                (0x00000009U)
#define CSL_IOMUX_EPWM2_A_CFG_REG_PUPDSEL_RESETVAL                             (0x00000000U)
#define CSL_IOMUX_EPWM2_A_CFG_REG_PUPDSEL_MAX                                  (0x00000001U)

#define CSL_IOMUX_EPWM2_A_CFG_REG_SC1_MASK                                     (0x00000400U)
#define CSL_IOMUX_EPWM2_A_CFG_REG_SC1_SHIFT                                    (0x0000000AU)
#define CSL_IOMUX_EPWM2_A_CFG_REG_SC1_RESETVAL                                 (0x00000001U)
#define CSL_IOMUX_EPWM2_A_CFG_REG_SC1_MAX                                      (0x00000001U)

#define CSL_IOMUX_EPWM2_A_CFG_REG_GPIO_SEL_MASK                                (0x00030000U)
#define CSL_IOMUX_EPWM2_A_CFG_REG_GPIO_SEL_SHIFT                               (0x00000010U)
#define CSL_IOMUX_EPWM2_A_CFG_REG_GPIO_SEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_EPWM2_A_CFG_REG_GPIO_SEL_MAX                                 (0x00000003U)

#define CSL_IOMUX_EPWM2_A_CFG_REG_QUAL_SEL_MASK                                (0x000C0000U)
#define CSL_IOMUX_EPWM2_A_CFG_REG_QUAL_SEL_SHIFT                               (0x00000012U)
#define CSL_IOMUX_EPWM2_A_CFG_REG_QUAL_SEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_EPWM2_A_CFG_REG_QUAL_SEL_MAX                                 (0x00000003U)

#define CSL_IOMUX_EPWM2_A_CFG_REG_INP_INV_SEL_MASK                             (0x00100000U)
#define CSL_IOMUX_EPWM2_A_CFG_REG_INP_INV_SEL_SHIFT                            (0x00000014U)
#define CSL_IOMUX_EPWM2_A_CFG_REG_INP_INV_SEL_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_EPWM2_A_CFG_REG_INP_INV_SEL_MAX                              (0x00000001U)

#define CSL_IOMUX_EPWM2_A_CFG_REG_ICSSM_GPIO_SEL_MASK                          (0x00200000U)
#define CSL_IOMUX_EPWM2_A_CFG_REG_ICSSM_GPIO_SEL_SHIFT                         (0x00000015U)
#define CSL_IOMUX_EPWM2_A_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_EPWM2_A_CFG_REG_ICSSM_GPIO_SEL_MAX                           (0x00000001U)

#define CSL_IOMUX_EPWM2_A_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                     (0x00400000U)
#define CSL_IOMUX_EPWM2_A_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                    (0x00000016U)
#define CSL_IOMUX_EPWM2_A_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL                 (0x00000000U)
#define CSL_IOMUX_EPWM2_A_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                      (0x00000001U)

#define CSL_IOMUX_EPWM2_A_CFG_REG_HSMODE_MASK                                  (0x40000000U)
#define CSL_IOMUX_EPWM2_A_CFG_REG_HSMODE_SHIFT                                 (0x0000001EU)
#define CSL_IOMUX_EPWM2_A_CFG_REG_HSMODE_RESETVAL                              (0x00000000U)
#define CSL_IOMUX_EPWM2_A_CFG_REG_HSMODE_MAX                                   (0x00000001U)

#define CSL_IOMUX_EPWM2_A_CFG_REG_HSMASTER_MASK                                (0x80000000U)
#define CSL_IOMUX_EPWM2_A_CFG_REG_HSMASTER_SHIFT                               (0x0000001FU)
#define CSL_IOMUX_EPWM2_A_CFG_REG_HSMASTER_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_EPWM2_A_CFG_REG_HSMASTER_MAX                                 (0x00000001U)

#define CSL_IOMUX_EPWM2_A_CFG_REG_RESETVAL                                     (0x000005F7U)

/* EPWM2_B_CFG_REG */

#define CSL_IOMUX_EPWM2_B_CFG_REG_FUNC_SEL_MASK                                (0x0000000FU)
#define CSL_IOMUX_EPWM2_B_CFG_REG_FUNC_SEL_SHIFT                               (0x00000000U)
#define CSL_IOMUX_EPWM2_B_CFG_REG_FUNC_SEL_RESETVAL                            (0x00000007U)
#define CSL_IOMUX_EPWM2_B_CFG_REG_FUNC_SEL_MAX                                 (0x0000000FU)

#define CSL_IOMUX_EPWM2_B_CFG_REG_IE_OVERRIDE_CTRL_MASK                        (0x00000010U)
#define CSL_IOMUX_EPWM2_B_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                       (0x00000004U)
#define CSL_IOMUX_EPWM2_B_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_EPWM2_B_CFG_REG_IE_OVERRIDE_CTRL_MAX                         (0x00000001U)

#define CSL_IOMUX_EPWM2_B_CFG_REG_IE_OVERRIDE_MASK                             (0x00000020U)
#define CSL_IOMUX_EPWM2_B_CFG_REG_IE_OVERRIDE_SHIFT                            (0x00000005U)
#define CSL_IOMUX_EPWM2_B_CFG_REG_IE_OVERRIDE_RESETVAL                         (0x00000001U)
#define CSL_IOMUX_EPWM2_B_CFG_REG_IE_OVERRIDE_MAX                              (0x00000001U)

#define CSL_IOMUX_EPWM2_B_CFG_REG_OE_OVERRIDE_CTRL_MASK                        (0x00000040U)
#define CSL_IOMUX_EPWM2_B_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                       (0x00000006U)
#define CSL_IOMUX_EPWM2_B_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_EPWM2_B_CFG_REG_OE_OVERRIDE_CTRL_MAX                         (0x00000001U)

#define CSL_IOMUX_EPWM2_B_CFG_REG_OE_OVERRIDE_MASK                             (0x00000080U)
#define CSL_IOMUX_EPWM2_B_CFG_REG_OE_OVERRIDE_SHIFT                            (0x00000007U)
#define CSL_IOMUX_EPWM2_B_CFG_REG_OE_OVERRIDE_RESETVAL                         (0x00000001U)
#define CSL_IOMUX_EPWM2_B_CFG_REG_OE_OVERRIDE_MAX                              (0x00000001U)

#define CSL_IOMUX_EPWM2_B_CFG_REG_PI_MASK                                      (0x00000100U)
#define CSL_IOMUX_EPWM2_B_CFG_REG_PI_SHIFT                                     (0x00000008U)
#define CSL_IOMUX_EPWM2_B_CFG_REG_PI_RESETVAL                                  (0x00000001U)
#define CSL_IOMUX_EPWM2_B_CFG_REG_PI_MAX                                       (0x00000001U)

#define CSL_IOMUX_EPWM2_B_CFG_REG_PUPDSEL_MASK                                 (0x00000200U)
#define CSL_IOMUX_EPWM2_B_CFG_REG_PUPDSEL_SHIFT                                (0x00000009U)
#define CSL_IOMUX_EPWM2_B_CFG_REG_PUPDSEL_RESETVAL                             (0x00000000U)
#define CSL_IOMUX_EPWM2_B_CFG_REG_PUPDSEL_MAX                                  (0x00000001U)

#define CSL_IOMUX_EPWM2_B_CFG_REG_SC1_MASK                                     (0x00000400U)
#define CSL_IOMUX_EPWM2_B_CFG_REG_SC1_SHIFT                                    (0x0000000AU)
#define CSL_IOMUX_EPWM2_B_CFG_REG_SC1_RESETVAL                                 (0x00000001U)
#define CSL_IOMUX_EPWM2_B_CFG_REG_SC1_MAX                                      (0x00000001U)

#define CSL_IOMUX_EPWM2_B_CFG_REG_GPIO_SEL_MASK                                (0x00030000U)
#define CSL_IOMUX_EPWM2_B_CFG_REG_GPIO_SEL_SHIFT                               (0x00000010U)
#define CSL_IOMUX_EPWM2_B_CFG_REG_GPIO_SEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_EPWM2_B_CFG_REG_GPIO_SEL_MAX                                 (0x00000003U)

#define CSL_IOMUX_EPWM2_B_CFG_REG_QUAL_SEL_MASK                                (0x000C0000U)
#define CSL_IOMUX_EPWM2_B_CFG_REG_QUAL_SEL_SHIFT                               (0x00000012U)
#define CSL_IOMUX_EPWM2_B_CFG_REG_QUAL_SEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_EPWM2_B_CFG_REG_QUAL_SEL_MAX                                 (0x00000003U)

#define CSL_IOMUX_EPWM2_B_CFG_REG_INP_INV_SEL_MASK                             (0x00100000U)
#define CSL_IOMUX_EPWM2_B_CFG_REG_INP_INV_SEL_SHIFT                            (0x00000014U)
#define CSL_IOMUX_EPWM2_B_CFG_REG_INP_INV_SEL_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_EPWM2_B_CFG_REG_INP_INV_SEL_MAX                              (0x00000001U)

#define CSL_IOMUX_EPWM2_B_CFG_REG_ICSSM_GPIO_SEL_MASK                          (0x00200000U)
#define CSL_IOMUX_EPWM2_B_CFG_REG_ICSSM_GPIO_SEL_SHIFT                         (0x00000015U)
#define CSL_IOMUX_EPWM2_B_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_EPWM2_B_CFG_REG_ICSSM_GPIO_SEL_MAX                           (0x00000001U)

#define CSL_IOMUX_EPWM2_B_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                     (0x00400000U)
#define CSL_IOMUX_EPWM2_B_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                    (0x00000016U)
#define CSL_IOMUX_EPWM2_B_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL                 (0x00000000U)
#define CSL_IOMUX_EPWM2_B_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                      (0x00000001U)

#define CSL_IOMUX_EPWM2_B_CFG_REG_HSMODE_MASK                                  (0x40000000U)
#define CSL_IOMUX_EPWM2_B_CFG_REG_HSMODE_SHIFT                                 (0x0000001EU)
#define CSL_IOMUX_EPWM2_B_CFG_REG_HSMODE_RESETVAL                              (0x00000000U)
#define CSL_IOMUX_EPWM2_B_CFG_REG_HSMODE_MAX                                   (0x00000001U)

#define CSL_IOMUX_EPWM2_B_CFG_REG_HSMASTER_MASK                                (0x80000000U)
#define CSL_IOMUX_EPWM2_B_CFG_REG_HSMASTER_SHIFT                               (0x0000001FU)
#define CSL_IOMUX_EPWM2_B_CFG_REG_HSMASTER_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_EPWM2_B_CFG_REG_HSMASTER_MAX                                 (0x00000001U)

#define CSL_IOMUX_EPWM2_B_CFG_REG_RESETVAL                                     (0x000005F7U)

/* EPWM3_A_CFG_REG */

#define CSL_IOMUX_EPWM3_A_CFG_REG_FUNC_SEL_MASK                                (0x0000000FU)
#define CSL_IOMUX_EPWM3_A_CFG_REG_FUNC_SEL_SHIFT                               (0x00000000U)
#define CSL_IOMUX_EPWM3_A_CFG_REG_FUNC_SEL_RESETVAL                            (0x00000007U)
#define CSL_IOMUX_EPWM3_A_CFG_REG_FUNC_SEL_MAX                                 (0x0000000FU)

#define CSL_IOMUX_EPWM3_A_CFG_REG_IE_OVERRIDE_CTRL_MASK                        (0x00000010U)
#define CSL_IOMUX_EPWM3_A_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                       (0x00000004U)
#define CSL_IOMUX_EPWM3_A_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_EPWM3_A_CFG_REG_IE_OVERRIDE_CTRL_MAX                         (0x00000001U)

#define CSL_IOMUX_EPWM3_A_CFG_REG_IE_OVERRIDE_MASK                             (0x00000020U)
#define CSL_IOMUX_EPWM3_A_CFG_REG_IE_OVERRIDE_SHIFT                            (0x00000005U)
#define CSL_IOMUX_EPWM3_A_CFG_REG_IE_OVERRIDE_RESETVAL                         (0x00000001U)
#define CSL_IOMUX_EPWM3_A_CFG_REG_IE_OVERRIDE_MAX                              (0x00000001U)

#define CSL_IOMUX_EPWM3_A_CFG_REG_OE_OVERRIDE_CTRL_MASK                        (0x00000040U)
#define CSL_IOMUX_EPWM3_A_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                       (0x00000006U)
#define CSL_IOMUX_EPWM3_A_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_EPWM3_A_CFG_REG_OE_OVERRIDE_CTRL_MAX                         (0x00000001U)

#define CSL_IOMUX_EPWM3_A_CFG_REG_OE_OVERRIDE_MASK                             (0x00000080U)
#define CSL_IOMUX_EPWM3_A_CFG_REG_OE_OVERRIDE_SHIFT                            (0x00000007U)
#define CSL_IOMUX_EPWM3_A_CFG_REG_OE_OVERRIDE_RESETVAL                         (0x00000001U)
#define CSL_IOMUX_EPWM3_A_CFG_REG_OE_OVERRIDE_MAX                              (0x00000001U)

#define CSL_IOMUX_EPWM3_A_CFG_REG_PI_MASK                                      (0x00000100U)
#define CSL_IOMUX_EPWM3_A_CFG_REG_PI_SHIFT                                     (0x00000008U)
#define CSL_IOMUX_EPWM3_A_CFG_REG_PI_RESETVAL                                  (0x00000001U)
#define CSL_IOMUX_EPWM3_A_CFG_REG_PI_MAX                                       (0x00000001U)

#define CSL_IOMUX_EPWM3_A_CFG_REG_PUPDSEL_MASK                                 (0x00000200U)
#define CSL_IOMUX_EPWM3_A_CFG_REG_PUPDSEL_SHIFT                                (0x00000009U)
#define CSL_IOMUX_EPWM3_A_CFG_REG_PUPDSEL_RESETVAL                             (0x00000000U)
#define CSL_IOMUX_EPWM3_A_CFG_REG_PUPDSEL_MAX                                  (0x00000001U)

#define CSL_IOMUX_EPWM3_A_CFG_REG_SC1_MASK                                     (0x00000400U)
#define CSL_IOMUX_EPWM3_A_CFG_REG_SC1_SHIFT                                    (0x0000000AU)
#define CSL_IOMUX_EPWM3_A_CFG_REG_SC1_RESETVAL                                 (0x00000001U)
#define CSL_IOMUX_EPWM3_A_CFG_REG_SC1_MAX                                      (0x00000001U)

#define CSL_IOMUX_EPWM3_A_CFG_REG_GPIO_SEL_MASK                                (0x00030000U)
#define CSL_IOMUX_EPWM3_A_CFG_REG_GPIO_SEL_SHIFT                               (0x00000010U)
#define CSL_IOMUX_EPWM3_A_CFG_REG_GPIO_SEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_EPWM3_A_CFG_REG_GPIO_SEL_MAX                                 (0x00000003U)

#define CSL_IOMUX_EPWM3_A_CFG_REG_QUAL_SEL_MASK                                (0x000C0000U)
#define CSL_IOMUX_EPWM3_A_CFG_REG_QUAL_SEL_SHIFT                               (0x00000012U)
#define CSL_IOMUX_EPWM3_A_CFG_REG_QUAL_SEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_EPWM3_A_CFG_REG_QUAL_SEL_MAX                                 (0x00000003U)

#define CSL_IOMUX_EPWM3_A_CFG_REG_INP_INV_SEL_MASK                             (0x00100000U)
#define CSL_IOMUX_EPWM3_A_CFG_REG_INP_INV_SEL_SHIFT                            (0x00000014U)
#define CSL_IOMUX_EPWM3_A_CFG_REG_INP_INV_SEL_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_EPWM3_A_CFG_REG_INP_INV_SEL_MAX                              (0x00000001U)

#define CSL_IOMUX_EPWM3_A_CFG_REG_ICSSM_GPIO_SEL_MASK                          (0x00200000U)
#define CSL_IOMUX_EPWM3_A_CFG_REG_ICSSM_GPIO_SEL_SHIFT                         (0x00000015U)
#define CSL_IOMUX_EPWM3_A_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_EPWM3_A_CFG_REG_ICSSM_GPIO_SEL_MAX                           (0x00000001U)

#define CSL_IOMUX_EPWM3_A_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                     (0x00400000U)
#define CSL_IOMUX_EPWM3_A_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                    (0x00000016U)
#define CSL_IOMUX_EPWM3_A_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL                 (0x00000000U)
#define CSL_IOMUX_EPWM3_A_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                      (0x00000001U)

#define CSL_IOMUX_EPWM3_A_CFG_REG_HSMODE_MASK                                  (0x40000000U)
#define CSL_IOMUX_EPWM3_A_CFG_REG_HSMODE_SHIFT                                 (0x0000001EU)
#define CSL_IOMUX_EPWM3_A_CFG_REG_HSMODE_RESETVAL                              (0x00000000U)
#define CSL_IOMUX_EPWM3_A_CFG_REG_HSMODE_MAX                                   (0x00000001U)

#define CSL_IOMUX_EPWM3_A_CFG_REG_HSMASTER_MASK                                (0x80000000U)
#define CSL_IOMUX_EPWM3_A_CFG_REG_HSMASTER_SHIFT                               (0x0000001FU)
#define CSL_IOMUX_EPWM3_A_CFG_REG_HSMASTER_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_EPWM3_A_CFG_REG_HSMASTER_MAX                                 (0x00000001U)

#define CSL_IOMUX_EPWM3_A_CFG_REG_RESETVAL                                     (0x000005F7U)

/* EPWM3_B_CFG_REG */

#define CSL_IOMUX_EPWM3_B_CFG_REG_FUNC_SEL_MASK                                (0x0000000FU)
#define CSL_IOMUX_EPWM3_B_CFG_REG_FUNC_SEL_SHIFT                               (0x00000000U)
#define CSL_IOMUX_EPWM3_B_CFG_REG_FUNC_SEL_RESETVAL                            (0x00000007U)
#define CSL_IOMUX_EPWM3_B_CFG_REG_FUNC_SEL_MAX                                 (0x0000000FU)

#define CSL_IOMUX_EPWM3_B_CFG_REG_IE_OVERRIDE_CTRL_MASK                        (0x00000010U)
#define CSL_IOMUX_EPWM3_B_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                       (0x00000004U)
#define CSL_IOMUX_EPWM3_B_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_EPWM3_B_CFG_REG_IE_OVERRIDE_CTRL_MAX                         (0x00000001U)

#define CSL_IOMUX_EPWM3_B_CFG_REG_IE_OVERRIDE_MASK                             (0x00000020U)
#define CSL_IOMUX_EPWM3_B_CFG_REG_IE_OVERRIDE_SHIFT                            (0x00000005U)
#define CSL_IOMUX_EPWM3_B_CFG_REG_IE_OVERRIDE_RESETVAL                         (0x00000001U)
#define CSL_IOMUX_EPWM3_B_CFG_REG_IE_OVERRIDE_MAX                              (0x00000001U)

#define CSL_IOMUX_EPWM3_B_CFG_REG_OE_OVERRIDE_CTRL_MASK                        (0x00000040U)
#define CSL_IOMUX_EPWM3_B_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                       (0x00000006U)
#define CSL_IOMUX_EPWM3_B_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_EPWM3_B_CFG_REG_OE_OVERRIDE_CTRL_MAX                         (0x00000001U)

#define CSL_IOMUX_EPWM3_B_CFG_REG_OE_OVERRIDE_MASK                             (0x00000080U)
#define CSL_IOMUX_EPWM3_B_CFG_REG_OE_OVERRIDE_SHIFT                            (0x00000007U)
#define CSL_IOMUX_EPWM3_B_CFG_REG_OE_OVERRIDE_RESETVAL                         (0x00000001U)
#define CSL_IOMUX_EPWM3_B_CFG_REG_OE_OVERRIDE_MAX                              (0x00000001U)

#define CSL_IOMUX_EPWM3_B_CFG_REG_PI_MASK                                      (0x00000100U)
#define CSL_IOMUX_EPWM3_B_CFG_REG_PI_SHIFT                                     (0x00000008U)
#define CSL_IOMUX_EPWM3_B_CFG_REG_PI_RESETVAL                                  (0x00000001U)
#define CSL_IOMUX_EPWM3_B_CFG_REG_PI_MAX                                       (0x00000001U)

#define CSL_IOMUX_EPWM3_B_CFG_REG_PUPDSEL_MASK                                 (0x00000200U)
#define CSL_IOMUX_EPWM3_B_CFG_REG_PUPDSEL_SHIFT                                (0x00000009U)
#define CSL_IOMUX_EPWM3_B_CFG_REG_PUPDSEL_RESETVAL                             (0x00000000U)
#define CSL_IOMUX_EPWM3_B_CFG_REG_PUPDSEL_MAX                                  (0x00000001U)

#define CSL_IOMUX_EPWM3_B_CFG_REG_SC1_MASK                                     (0x00000400U)
#define CSL_IOMUX_EPWM3_B_CFG_REG_SC1_SHIFT                                    (0x0000000AU)
#define CSL_IOMUX_EPWM3_B_CFG_REG_SC1_RESETVAL                                 (0x00000001U)
#define CSL_IOMUX_EPWM3_B_CFG_REG_SC1_MAX                                      (0x00000001U)

#define CSL_IOMUX_EPWM3_B_CFG_REG_GPIO_SEL_MASK                                (0x00030000U)
#define CSL_IOMUX_EPWM3_B_CFG_REG_GPIO_SEL_SHIFT                               (0x00000010U)
#define CSL_IOMUX_EPWM3_B_CFG_REG_GPIO_SEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_EPWM3_B_CFG_REG_GPIO_SEL_MAX                                 (0x00000003U)

#define CSL_IOMUX_EPWM3_B_CFG_REG_QUAL_SEL_MASK                                (0x000C0000U)
#define CSL_IOMUX_EPWM3_B_CFG_REG_QUAL_SEL_SHIFT                               (0x00000012U)
#define CSL_IOMUX_EPWM3_B_CFG_REG_QUAL_SEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_EPWM3_B_CFG_REG_QUAL_SEL_MAX                                 (0x00000003U)

#define CSL_IOMUX_EPWM3_B_CFG_REG_INP_INV_SEL_MASK                             (0x00100000U)
#define CSL_IOMUX_EPWM3_B_CFG_REG_INP_INV_SEL_SHIFT                            (0x00000014U)
#define CSL_IOMUX_EPWM3_B_CFG_REG_INP_INV_SEL_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_EPWM3_B_CFG_REG_INP_INV_SEL_MAX                              (0x00000001U)

#define CSL_IOMUX_EPWM3_B_CFG_REG_ICSSM_GPIO_SEL_MASK                          (0x00200000U)
#define CSL_IOMUX_EPWM3_B_CFG_REG_ICSSM_GPIO_SEL_SHIFT                         (0x00000015U)
#define CSL_IOMUX_EPWM3_B_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_EPWM3_B_CFG_REG_ICSSM_GPIO_SEL_MAX                           (0x00000001U)

#define CSL_IOMUX_EPWM3_B_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                     (0x00400000U)
#define CSL_IOMUX_EPWM3_B_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                    (0x00000016U)
#define CSL_IOMUX_EPWM3_B_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL                 (0x00000000U)
#define CSL_IOMUX_EPWM3_B_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                      (0x00000001U)

#define CSL_IOMUX_EPWM3_B_CFG_REG_HSMODE_MASK                                  (0x40000000U)
#define CSL_IOMUX_EPWM3_B_CFG_REG_HSMODE_SHIFT                                 (0x0000001EU)
#define CSL_IOMUX_EPWM3_B_CFG_REG_HSMODE_RESETVAL                              (0x00000000U)
#define CSL_IOMUX_EPWM3_B_CFG_REG_HSMODE_MAX                                   (0x00000001U)

#define CSL_IOMUX_EPWM3_B_CFG_REG_HSMASTER_MASK                                (0x80000000U)
#define CSL_IOMUX_EPWM3_B_CFG_REG_HSMASTER_SHIFT                               (0x0000001FU)
#define CSL_IOMUX_EPWM3_B_CFG_REG_HSMASTER_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_EPWM3_B_CFG_REG_HSMASTER_MAX                                 (0x00000001U)

#define CSL_IOMUX_EPWM3_B_CFG_REG_RESETVAL                                     (0x000005F7U)

/* EPWM4_A_CFG_REG */

#define CSL_IOMUX_EPWM4_A_CFG_REG_FUNC_SEL_MASK                                (0x0000000FU)
#define CSL_IOMUX_EPWM4_A_CFG_REG_FUNC_SEL_SHIFT                               (0x00000000U)
#define CSL_IOMUX_EPWM4_A_CFG_REG_FUNC_SEL_RESETVAL                            (0x00000007U)
#define CSL_IOMUX_EPWM4_A_CFG_REG_FUNC_SEL_MAX                                 (0x0000000FU)

#define CSL_IOMUX_EPWM4_A_CFG_REG_IE_OVERRIDE_CTRL_MASK                        (0x00000010U)
#define CSL_IOMUX_EPWM4_A_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                       (0x00000004U)
#define CSL_IOMUX_EPWM4_A_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_EPWM4_A_CFG_REG_IE_OVERRIDE_CTRL_MAX                         (0x00000001U)

#define CSL_IOMUX_EPWM4_A_CFG_REG_IE_OVERRIDE_MASK                             (0x00000020U)
#define CSL_IOMUX_EPWM4_A_CFG_REG_IE_OVERRIDE_SHIFT                            (0x00000005U)
#define CSL_IOMUX_EPWM4_A_CFG_REG_IE_OVERRIDE_RESETVAL                         (0x00000001U)
#define CSL_IOMUX_EPWM4_A_CFG_REG_IE_OVERRIDE_MAX                              (0x00000001U)

#define CSL_IOMUX_EPWM4_A_CFG_REG_OE_OVERRIDE_CTRL_MASK                        (0x00000040U)
#define CSL_IOMUX_EPWM4_A_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                       (0x00000006U)
#define CSL_IOMUX_EPWM4_A_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_EPWM4_A_CFG_REG_OE_OVERRIDE_CTRL_MAX                         (0x00000001U)

#define CSL_IOMUX_EPWM4_A_CFG_REG_OE_OVERRIDE_MASK                             (0x00000080U)
#define CSL_IOMUX_EPWM4_A_CFG_REG_OE_OVERRIDE_SHIFT                            (0x00000007U)
#define CSL_IOMUX_EPWM4_A_CFG_REG_OE_OVERRIDE_RESETVAL                         (0x00000001U)
#define CSL_IOMUX_EPWM4_A_CFG_REG_OE_OVERRIDE_MAX                              (0x00000001U)

#define CSL_IOMUX_EPWM4_A_CFG_REG_PI_MASK                                      (0x00000100U)
#define CSL_IOMUX_EPWM4_A_CFG_REG_PI_SHIFT                                     (0x00000008U)
#define CSL_IOMUX_EPWM4_A_CFG_REG_PI_RESETVAL                                  (0x00000001U)
#define CSL_IOMUX_EPWM4_A_CFG_REG_PI_MAX                                       (0x00000001U)

#define CSL_IOMUX_EPWM4_A_CFG_REG_PUPDSEL_MASK                                 (0x00000200U)
#define CSL_IOMUX_EPWM4_A_CFG_REG_PUPDSEL_SHIFT                                (0x00000009U)
#define CSL_IOMUX_EPWM4_A_CFG_REG_PUPDSEL_RESETVAL                             (0x00000000U)
#define CSL_IOMUX_EPWM4_A_CFG_REG_PUPDSEL_MAX                                  (0x00000001U)

#define CSL_IOMUX_EPWM4_A_CFG_REG_SC1_MASK                                     (0x00000400U)
#define CSL_IOMUX_EPWM4_A_CFG_REG_SC1_SHIFT                                    (0x0000000AU)
#define CSL_IOMUX_EPWM4_A_CFG_REG_SC1_RESETVAL                                 (0x00000001U)
#define CSL_IOMUX_EPWM4_A_CFG_REG_SC1_MAX                                      (0x00000001U)

#define CSL_IOMUX_EPWM4_A_CFG_REG_GPIO_SEL_MASK                                (0x00030000U)
#define CSL_IOMUX_EPWM4_A_CFG_REG_GPIO_SEL_SHIFT                               (0x00000010U)
#define CSL_IOMUX_EPWM4_A_CFG_REG_GPIO_SEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_EPWM4_A_CFG_REG_GPIO_SEL_MAX                                 (0x00000003U)

#define CSL_IOMUX_EPWM4_A_CFG_REG_QUAL_SEL_MASK                                (0x000C0000U)
#define CSL_IOMUX_EPWM4_A_CFG_REG_QUAL_SEL_SHIFT                               (0x00000012U)
#define CSL_IOMUX_EPWM4_A_CFG_REG_QUAL_SEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_EPWM4_A_CFG_REG_QUAL_SEL_MAX                                 (0x00000003U)

#define CSL_IOMUX_EPWM4_A_CFG_REG_INP_INV_SEL_MASK                             (0x00100000U)
#define CSL_IOMUX_EPWM4_A_CFG_REG_INP_INV_SEL_SHIFT                            (0x00000014U)
#define CSL_IOMUX_EPWM4_A_CFG_REG_INP_INV_SEL_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_EPWM4_A_CFG_REG_INP_INV_SEL_MAX                              (0x00000001U)

#define CSL_IOMUX_EPWM4_A_CFG_REG_ICSSM_GPIO_SEL_MASK                          (0x00200000U)
#define CSL_IOMUX_EPWM4_A_CFG_REG_ICSSM_GPIO_SEL_SHIFT                         (0x00000015U)
#define CSL_IOMUX_EPWM4_A_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_EPWM4_A_CFG_REG_ICSSM_GPIO_SEL_MAX                           (0x00000001U)

#define CSL_IOMUX_EPWM4_A_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                     (0x00400000U)
#define CSL_IOMUX_EPWM4_A_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                    (0x00000016U)
#define CSL_IOMUX_EPWM4_A_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL                 (0x00000000U)
#define CSL_IOMUX_EPWM4_A_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                      (0x00000001U)

#define CSL_IOMUX_EPWM4_A_CFG_REG_HSMODE_MASK                                  (0x40000000U)
#define CSL_IOMUX_EPWM4_A_CFG_REG_HSMODE_SHIFT                                 (0x0000001EU)
#define CSL_IOMUX_EPWM4_A_CFG_REG_HSMODE_RESETVAL                              (0x00000000U)
#define CSL_IOMUX_EPWM4_A_CFG_REG_HSMODE_MAX                                   (0x00000001U)

#define CSL_IOMUX_EPWM4_A_CFG_REG_HSMASTER_MASK                                (0x80000000U)
#define CSL_IOMUX_EPWM4_A_CFG_REG_HSMASTER_SHIFT                               (0x0000001FU)
#define CSL_IOMUX_EPWM4_A_CFG_REG_HSMASTER_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_EPWM4_A_CFG_REG_HSMASTER_MAX                                 (0x00000001U)

#define CSL_IOMUX_EPWM4_A_CFG_REG_RESETVAL                                     (0x000005F7U)

/* EPWM4_B_CFG_REG */

#define CSL_IOMUX_EPWM4_B_CFG_REG_FUNC_SEL_MASK                                (0x0000000FU)
#define CSL_IOMUX_EPWM4_B_CFG_REG_FUNC_SEL_SHIFT                               (0x00000000U)
#define CSL_IOMUX_EPWM4_B_CFG_REG_FUNC_SEL_RESETVAL                            (0x00000007U)
#define CSL_IOMUX_EPWM4_B_CFG_REG_FUNC_SEL_MAX                                 (0x0000000FU)

#define CSL_IOMUX_EPWM4_B_CFG_REG_IE_OVERRIDE_CTRL_MASK                        (0x00000010U)
#define CSL_IOMUX_EPWM4_B_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                       (0x00000004U)
#define CSL_IOMUX_EPWM4_B_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_EPWM4_B_CFG_REG_IE_OVERRIDE_CTRL_MAX                         (0x00000001U)

#define CSL_IOMUX_EPWM4_B_CFG_REG_IE_OVERRIDE_MASK                             (0x00000020U)
#define CSL_IOMUX_EPWM4_B_CFG_REG_IE_OVERRIDE_SHIFT                            (0x00000005U)
#define CSL_IOMUX_EPWM4_B_CFG_REG_IE_OVERRIDE_RESETVAL                         (0x00000001U)
#define CSL_IOMUX_EPWM4_B_CFG_REG_IE_OVERRIDE_MAX                              (0x00000001U)

#define CSL_IOMUX_EPWM4_B_CFG_REG_OE_OVERRIDE_CTRL_MASK                        (0x00000040U)
#define CSL_IOMUX_EPWM4_B_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                       (0x00000006U)
#define CSL_IOMUX_EPWM4_B_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_EPWM4_B_CFG_REG_OE_OVERRIDE_CTRL_MAX                         (0x00000001U)

#define CSL_IOMUX_EPWM4_B_CFG_REG_OE_OVERRIDE_MASK                             (0x00000080U)
#define CSL_IOMUX_EPWM4_B_CFG_REG_OE_OVERRIDE_SHIFT                            (0x00000007U)
#define CSL_IOMUX_EPWM4_B_CFG_REG_OE_OVERRIDE_RESETVAL                         (0x00000001U)
#define CSL_IOMUX_EPWM4_B_CFG_REG_OE_OVERRIDE_MAX                              (0x00000001U)

#define CSL_IOMUX_EPWM4_B_CFG_REG_PI_MASK                                      (0x00000100U)
#define CSL_IOMUX_EPWM4_B_CFG_REG_PI_SHIFT                                     (0x00000008U)
#define CSL_IOMUX_EPWM4_B_CFG_REG_PI_RESETVAL                                  (0x00000001U)
#define CSL_IOMUX_EPWM4_B_CFG_REG_PI_MAX                                       (0x00000001U)

#define CSL_IOMUX_EPWM4_B_CFG_REG_PUPDSEL_MASK                                 (0x00000200U)
#define CSL_IOMUX_EPWM4_B_CFG_REG_PUPDSEL_SHIFT                                (0x00000009U)
#define CSL_IOMUX_EPWM4_B_CFG_REG_PUPDSEL_RESETVAL                             (0x00000000U)
#define CSL_IOMUX_EPWM4_B_CFG_REG_PUPDSEL_MAX                                  (0x00000001U)

#define CSL_IOMUX_EPWM4_B_CFG_REG_SC1_MASK                                     (0x00000400U)
#define CSL_IOMUX_EPWM4_B_CFG_REG_SC1_SHIFT                                    (0x0000000AU)
#define CSL_IOMUX_EPWM4_B_CFG_REG_SC1_RESETVAL                                 (0x00000001U)
#define CSL_IOMUX_EPWM4_B_CFG_REG_SC1_MAX                                      (0x00000001U)

#define CSL_IOMUX_EPWM4_B_CFG_REG_GPIO_SEL_MASK                                (0x00030000U)
#define CSL_IOMUX_EPWM4_B_CFG_REG_GPIO_SEL_SHIFT                               (0x00000010U)
#define CSL_IOMUX_EPWM4_B_CFG_REG_GPIO_SEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_EPWM4_B_CFG_REG_GPIO_SEL_MAX                                 (0x00000003U)

#define CSL_IOMUX_EPWM4_B_CFG_REG_QUAL_SEL_MASK                                (0x000C0000U)
#define CSL_IOMUX_EPWM4_B_CFG_REG_QUAL_SEL_SHIFT                               (0x00000012U)
#define CSL_IOMUX_EPWM4_B_CFG_REG_QUAL_SEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_EPWM4_B_CFG_REG_QUAL_SEL_MAX                                 (0x00000003U)

#define CSL_IOMUX_EPWM4_B_CFG_REG_INP_INV_SEL_MASK                             (0x00100000U)
#define CSL_IOMUX_EPWM4_B_CFG_REG_INP_INV_SEL_SHIFT                            (0x00000014U)
#define CSL_IOMUX_EPWM4_B_CFG_REG_INP_INV_SEL_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_EPWM4_B_CFG_REG_INP_INV_SEL_MAX                              (0x00000001U)

#define CSL_IOMUX_EPWM4_B_CFG_REG_ICSSM_GPIO_SEL_MASK                          (0x00200000U)
#define CSL_IOMUX_EPWM4_B_CFG_REG_ICSSM_GPIO_SEL_SHIFT                         (0x00000015U)
#define CSL_IOMUX_EPWM4_B_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_EPWM4_B_CFG_REG_ICSSM_GPIO_SEL_MAX                           (0x00000001U)

#define CSL_IOMUX_EPWM4_B_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                     (0x00400000U)
#define CSL_IOMUX_EPWM4_B_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                    (0x00000016U)
#define CSL_IOMUX_EPWM4_B_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL                 (0x00000000U)
#define CSL_IOMUX_EPWM4_B_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                      (0x00000001U)

#define CSL_IOMUX_EPWM4_B_CFG_REG_HSMODE_MASK                                  (0x40000000U)
#define CSL_IOMUX_EPWM4_B_CFG_REG_HSMODE_SHIFT                                 (0x0000001EU)
#define CSL_IOMUX_EPWM4_B_CFG_REG_HSMODE_RESETVAL                              (0x00000000U)
#define CSL_IOMUX_EPWM4_B_CFG_REG_HSMODE_MAX                                   (0x00000001U)

#define CSL_IOMUX_EPWM4_B_CFG_REG_HSMASTER_MASK                                (0x80000000U)
#define CSL_IOMUX_EPWM4_B_CFG_REG_HSMASTER_SHIFT                               (0x0000001FU)
#define CSL_IOMUX_EPWM4_B_CFG_REG_HSMASTER_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_EPWM4_B_CFG_REG_HSMASTER_MAX                                 (0x00000001U)

#define CSL_IOMUX_EPWM4_B_CFG_REG_RESETVAL                                     (0x000005F7U)

/* EPWM5_A_CFG_REG */

#define CSL_IOMUX_EPWM5_A_CFG_REG_FUNC_SEL_MASK                                (0x0000000FU)
#define CSL_IOMUX_EPWM5_A_CFG_REG_FUNC_SEL_SHIFT                               (0x00000000U)
#define CSL_IOMUX_EPWM5_A_CFG_REG_FUNC_SEL_RESETVAL                            (0x00000007U)
#define CSL_IOMUX_EPWM5_A_CFG_REG_FUNC_SEL_MAX                                 (0x0000000FU)

#define CSL_IOMUX_EPWM5_A_CFG_REG_IE_OVERRIDE_CTRL_MASK                        (0x00000010U)
#define CSL_IOMUX_EPWM5_A_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                       (0x00000004U)
#define CSL_IOMUX_EPWM5_A_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_EPWM5_A_CFG_REG_IE_OVERRIDE_CTRL_MAX                         (0x00000001U)

#define CSL_IOMUX_EPWM5_A_CFG_REG_IE_OVERRIDE_MASK                             (0x00000020U)
#define CSL_IOMUX_EPWM5_A_CFG_REG_IE_OVERRIDE_SHIFT                            (0x00000005U)
#define CSL_IOMUX_EPWM5_A_CFG_REG_IE_OVERRIDE_RESETVAL                         (0x00000001U)
#define CSL_IOMUX_EPWM5_A_CFG_REG_IE_OVERRIDE_MAX                              (0x00000001U)

#define CSL_IOMUX_EPWM5_A_CFG_REG_OE_OVERRIDE_CTRL_MASK                        (0x00000040U)
#define CSL_IOMUX_EPWM5_A_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                       (0x00000006U)
#define CSL_IOMUX_EPWM5_A_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_EPWM5_A_CFG_REG_OE_OVERRIDE_CTRL_MAX                         (0x00000001U)

#define CSL_IOMUX_EPWM5_A_CFG_REG_OE_OVERRIDE_MASK                             (0x00000080U)
#define CSL_IOMUX_EPWM5_A_CFG_REG_OE_OVERRIDE_SHIFT                            (0x00000007U)
#define CSL_IOMUX_EPWM5_A_CFG_REG_OE_OVERRIDE_RESETVAL                         (0x00000001U)
#define CSL_IOMUX_EPWM5_A_CFG_REG_OE_OVERRIDE_MAX                              (0x00000001U)

#define CSL_IOMUX_EPWM5_A_CFG_REG_PI_MASK                                      (0x00000100U)
#define CSL_IOMUX_EPWM5_A_CFG_REG_PI_SHIFT                                     (0x00000008U)
#define CSL_IOMUX_EPWM5_A_CFG_REG_PI_RESETVAL                                  (0x00000001U)
#define CSL_IOMUX_EPWM5_A_CFG_REG_PI_MAX                                       (0x00000001U)

#define CSL_IOMUX_EPWM5_A_CFG_REG_PUPDSEL_MASK                                 (0x00000200U)
#define CSL_IOMUX_EPWM5_A_CFG_REG_PUPDSEL_SHIFT                                (0x00000009U)
#define CSL_IOMUX_EPWM5_A_CFG_REG_PUPDSEL_RESETVAL                             (0x00000000U)
#define CSL_IOMUX_EPWM5_A_CFG_REG_PUPDSEL_MAX                                  (0x00000001U)

#define CSL_IOMUX_EPWM5_A_CFG_REG_SC1_MASK                                     (0x00000400U)
#define CSL_IOMUX_EPWM5_A_CFG_REG_SC1_SHIFT                                    (0x0000000AU)
#define CSL_IOMUX_EPWM5_A_CFG_REG_SC1_RESETVAL                                 (0x00000001U)
#define CSL_IOMUX_EPWM5_A_CFG_REG_SC1_MAX                                      (0x00000001U)

#define CSL_IOMUX_EPWM5_A_CFG_REG_GPIO_SEL_MASK                                (0x00030000U)
#define CSL_IOMUX_EPWM5_A_CFG_REG_GPIO_SEL_SHIFT                               (0x00000010U)
#define CSL_IOMUX_EPWM5_A_CFG_REG_GPIO_SEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_EPWM5_A_CFG_REG_GPIO_SEL_MAX                                 (0x00000003U)

#define CSL_IOMUX_EPWM5_A_CFG_REG_QUAL_SEL_MASK                                (0x000C0000U)
#define CSL_IOMUX_EPWM5_A_CFG_REG_QUAL_SEL_SHIFT                               (0x00000012U)
#define CSL_IOMUX_EPWM5_A_CFG_REG_QUAL_SEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_EPWM5_A_CFG_REG_QUAL_SEL_MAX                                 (0x00000003U)

#define CSL_IOMUX_EPWM5_A_CFG_REG_INP_INV_SEL_MASK                             (0x00100000U)
#define CSL_IOMUX_EPWM5_A_CFG_REG_INP_INV_SEL_SHIFT                            (0x00000014U)
#define CSL_IOMUX_EPWM5_A_CFG_REG_INP_INV_SEL_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_EPWM5_A_CFG_REG_INP_INV_SEL_MAX                              (0x00000001U)

#define CSL_IOMUX_EPWM5_A_CFG_REG_ICSSM_GPIO_SEL_MASK                          (0x00200000U)
#define CSL_IOMUX_EPWM5_A_CFG_REG_ICSSM_GPIO_SEL_SHIFT                         (0x00000015U)
#define CSL_IOMUX_EPWM5_A_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_EPWM5_A_CFG_REG_ICSSM_GPIO_SEL_MAX                           (0x00000001U)

#define CSL_IOMUX_EPWM5_A_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                     (0x00400000U)
#define CSL_IOMUX_EPWM5_A_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                    (0x00000016U)
#define CSL_IOMUX_EPWM5_A_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL                 (0x00000000U)
#define CSL_IOMUX_EPWM5_A_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                      (0x00000001U)

#define CSL_IOMUX_EPWM5_A_CFG_REG_HSMODE_MASK                                  (0x40000000U)
#define CSL_IOMUX_EPWM5_A_CFG_REG_HSMODE_SHIFT                                 (0x0000001EU)
#define CSL_IOMUX_EPWM5_A_CFG_REG_HSMODE_RESETVAL                              (0x00000000U)
#define CSL_IOMUX_EPWM5_A_CFG_REG_HSMODE_MAX                                   (0x00000001U)

#define CSL_IOMUX_EPWM5_A_CFG_REG_HSMASTER_MASK                                (0x80000000U)
#define CSL_IOMUX_EPWM5_A_CFG_REG_HSMASTER_SHIFT                               (0x0000001FU)
#define CSL_IOMUX_EPWM5_A_CFG_REG_HSMASTER_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_EPWM5_A_CFG_REG_HSMASTER_MAX                                 (0x00000001U)

#define CSL_IOMUX_EPWM5_A_CFG_REG_RESETVAL                                     (0x000005F7U)

/* EPWM5_B_CFG_REG */

#define CSL_IOMUX_EPWM5_B_CFG_REG_FUNC_SEL_MASK                                (0x0000000FU)
#define CSL_IOMUX_EPWM5_B_CFG_REG_FUNC_SEL_SHIFT                               (0x00000000U)
#define CSL_IOMUX_EPWM5_B_CFG_REG_FUNC_SEL_RESETVAL                            (0x00000007U)
#define CSL_IOMUX_EPWM5_B_CFG_REG_FUNC_SEL_MAX                                 (0x0000000FU)

#define CSL_IOMUX_EPWM5_B_CFG_REG_IE_OVERRIDE_CTRL_MASK                        (0x00000010U)
#define CSL_IOMUX_EPWM5_B_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                       (0x00000004U)
#define CSL_IOMUX_EPWM5_B_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_EPWM5_B_CFG_REG_IE_OVERRIDE_CTRL_MAX                         (0x00000001U)

#define CSL_IOMUX_EPWM5_B_CFG_REG_IE_OVERRIDE_MASK                             (0x00000020U)
#define CSL_IOMUX_EPWM5_B_CFG_REG_IE_OVERRIDE_SHIFT                            (0x00000005U)
#define CSL_IOMUX_EPWM5_B_CFG_REG_IE_OVERRIDE_RESETVAL                         (0x00000001U)
#define CSL_IOMUX_EPWM5_B_CFG_REG_IE_OVERRIDE_MAX                              (0x00000001U)

#define CSL_IOMUX_EPWM5_B_CFG_REG_OE_OVERRIDE_CTRL_MASK                        (0x00000040U)
#define CSL_IOMUX_EPWM5_B_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                       (0x00000006U)
#define CSL_IOMUX_EPWM5_B_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_EPWM5_B_CFG_REG_OE_OVERRIDE_CTRL_MAX                         (0x00000001U)

#define CSL_IOMUX_EPWM5_B_CFG_REG_OE_OVERRIDE_MASK                             (0x00000080U)
#define CSL_IOMUX_EPWM5_B_CFG_REG_OE_OVERRIDE_SHIFT                            (0x00000007U)
#define CSL_IOMUX_EPWM5_B_CFG_REG_OE_OVERRIDE_RESETVAL                         (0x00000001U)
#define CSL_IOMUX_EPWM5_B_CFG_REG_OE_OVERRIDE_MAX                              (0x00000001U)

#define CSL_IOMUX_EPWM5_B_CFG_REG_PI_MASK                                      (0x00000100U)
#define CSL_IOMUX_EPWM5_B_CFG_REG_PI_SHIFT                                     (0x00000008U)
#define CSL_IOMUX_EPWM5_B_CFG_REG_PI_RESETVAL                                  (0x00000001U)
#define CSL_IOMUX_EPWM5_B_CFG_REG_PI_MAX                                       (0x00000001U)

#define CSL_IOMUX_EPWM5_B_CFG_REG_PUPDSEL_MASK                                 (0x00000200U)
#define CSL_IOMUX_EPWM5_B_CFG_REG_PUPDSEL_SHIFT                                (0x00000009U)
#define CSL_IOMUX_EPWM5_B_CFG_REG_PUPDSEL_RESETVAL                             (0x00000000U)
#define CSL_IOMUX_EPWM5_B_CFG_REG_PUPDSEL_MAX                                  (0x00000001U)

#define CSL_IOMUX_EPWM5_B_CFG_REG_SC1_MASK                                     (0x00000400U)
#define CSL_IOMUX_EPWM5_B_CFG_REG_SC1_SHIFT                                    (0x0000000AU)
#define CSL_IOMUX_EPWM5_B_CFG_REG_SC1_RESETVAL                                 (0x00000001U)
#define CSL_IOMUX_EPWM5_B_CFG_REG_SC1_MAX                                      (0x00000001U)

#define CSL_IOMUX_EPWM5_B_CFG_REG_GPIO_SEL_MASK                                (0x00030000U)
#define CSL_IOMUX_EPWM5_B_CFG_REG_GPIO_SEL_SHIFT                               (0x00000010U)
#define CSL_IOMUX_EPWM5_B_CFG_REG_GPIO_SEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_EPWM5_B_CFG_REG_GPIO_SEL_MAX                                 (0x00000003U)

#define CSL_IOMUX_EPWM5_B_CFG_REG_QUAL_SEL_MASK                                (0x000C0000U)
#define CSL_IOMUX_EPWM5_B_CFG_REG_QUAL_SEL_SHIFT                               (0x00000012U)
#define CSL_IOMUX_EPWM5_B_CFG_REG_QUAL_SEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_EPWM5_B_CFG_REG_QUAL_SEL_MAX                                 (0x00000003U)

#define CSL_IOMUX_EPWM5_B_CFG_REG_INP_INV_SEL_MASK                             (0x00100000U)
#define CSL_IOMUX_EPWM5_B_CFG_REG_INP_INV_SEL_SHIFT                            (0x00000014U)
#define CSL_IOMUX_EPWM5_B_CFG_REG_INP_INV_SEL_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_EPWM5_B_CFG_REG_INP_INV_SEL_MAX                              (0x00000001U)

#define CSL_IOMUX_EPWM5_B_CFG_REG_ICSSM_GPIO_SEL_MASK                          (0x00200000U)
#define CSL_IOMUX_EPWM5_B_CFG_REG_ICSSM_GPIO_SEL_SHIFT                         (0x00000015U)
#define CSL_IOMUX_EPWM5_B_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_EPWM5_B_CFG_REG_ICSSM_GPIO_SEL_MAX                           (0x00000001U)

#define CSL_IOMUX_EPWM5_B_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                     (0x00400000U)
#define CSL_IOMUX_EPWM5_B_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                    (0x00000016U)
#define CSL_IOMUX_EPWM5_B_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL                 (0x00000000U)
#define CSL_IOMUX_EPWM5_B_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                      (0x00000001U)

#define CSL_IOMUX_EPWM5_B_CFG_REG_HSMODE_MASK                                  (0x40000000U)
#define CSL_IOMUX_EPWM5_B_CFG_REG_HSMODE_SHIFT                                 (0x0000001EU)
#define CSL_IOMUX_EPWM5_B_CFG_REG_HSMODE_RESETVAL                              (0x00000000U)
#define CSL_IOMUX_EPWM5_B_CFG_REG_HSMODE_MAX                                   (0x00000001U)

#define CSL_IOMUX_EPWM5_B_CFG_REG_HSMASTER_MASK                                (0x80000000U)
#define CSL_IOMUX_EPWM5_B_CFG_REG_HSMASTER_SHIFT                               (0x0000001FU)
#define CSL_IOMUX_EPWM5_B_CFG_REG_HSMASTER_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_EPWM5_B_CFG_REG_HSMASTER_MAX                                 (0x00000001U)

#define CSL_IOMUX_EPWM5_B_CFG_REG_RESETVAL                                     (0x000005F7U)

/* EPWM6_A_CFG_REG */

#define CSL_IOMUX_EPWM6_A_CFG_REG_FUNC_SEL_MASK                                (0x0000000FU)
#define CSL_IOMUX_EPWM6_A_CFG_REG_FUNC_SEL_SHIFT                               (0x00000000U)
#define CSL_IOMUX_EPWM6_A_CFG_REG_FUNC_SEL_RESETVAL                            (0x00000007U)
#define CSL_IOMUX_EPWM6_A_CFG_REG_FUNC_SEL_MAX                                 (0x0000000FU)

#define CSL_IOMUX_EPWM6_A_CFG_REG_IE_OVERRIDE_CTRL_MASK                        (0x00000010U)
#define CSL_IOMUX_EPWM6_A_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                       (0x00000004U)
#define CSL_IOMUX_EPWM6_A_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_EPWM6_A_CFG_REG_IE_OVERRIDE_CTRL_MAX                         (0x00000001U)

#define CSL_IOMUX_EPWM6_A_CFG_REG_IE_OVERRIDE_MASK                             (0x00000020U)
#define CSL_IOMUX_EPWM6_A_CFG_REG_IE_OVERRIDE_SHIFT                            (0x00000005U)
#define CSL_IOMUX_EPWM6_A_CFG_REG_IE_OVERRIDE_RESETVAL                         (0x00000001U)
#define CSL_IOMUX_EPWM6_A_CFG_REG_IE_OVERRIDE_MAX                              (0x00000001U)

#define CSL_IOMUX_EPWM6_A_CFG_REG_OE_OVERRIDE_CTRL_MASK                        (0x00000040U)
#define CSL_IOMUX_EPWM6_A_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                       (0x00000006U)
#define CSL_IOMUX_EPWM6_A_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_EPWM6_A_CFG_REG_OE_OVERRIDE_CTRL_MAX                         (0x00000001U)

#define CSL_IOMUX_EPWM6_A_CFG_REG_OE_OVERRIDE_MASK                             (0x00000080U)
#define CSL_IOMUX_EPWM6_A_CFG_REG_OE_OVERRIDE_SHIFT                            (0x00000007U)
#define CSL_IOMUX_EPWM6_A_CFG_REG_OE_OVERRIDE_RESETVAL                         (0x00000001U)
#define CSL_IOMUX_EPWM6_A_CFG_REG_OE_OVERRIDE_MAX                              (0x00000001U)

#define CSL_IOMUX_EPWM6_A_CFG_REG_PI_MASK                                      (0x00000100U)
#define CSL_IOMUX_EPWM6_A_CFG_REG_PI_SHIFT                                     (0x00000008U)
#define CSL_IOMUX_EPWM6_A_CFG_REG_PI_RESETVAL                                  (0x00000001U)
#define CSL_IOMUX_EPWM6_A_CFG_REG_PI_MAX                                       (0x00000001U)

#define CSL_IOMUX_EPWM6_A_CFG_REG_PUPDSEL_MASK                                 (0x00000200U)
#define CSL_IOMUX_EPWM6_A_CFG_REG_PUPDSEL_SHIFT                                (0x00000009U)
#define CSL_IOMUX_EPWM6_A_CFG_REG_PUPDSEL_RESETVAL                             (0x00000000U)
#define CSL_IOMUX_EPWM6_A_CFG_REG_PUPDSEL_MAX                                  (0x00000001U)

#define CSL_IOMUX_EPWM6_A_CFG_REG_SC1_MASK                                     (0x00000400U)
#define CSL_IOMUX_EPWM6_A_CFG_REG_SC1_SHIFT                                    (0x0000000AU)
#define CSL_IOMUX_EPWM6_A_CFG_REG_SC1_RESETVAL                                 (0x00000001U)
#define CSL_IOMUX_EPWM6_A_CFG_REG_SC1_MAX                                      (0x00000001U)

#define CSL_IOMUX_EPWM6_A_CFG_REG_GPIO_SEL_MASK                                (0x00030000U)
#define CSL_IOMUX_EPWM6_A_CFG_REG_GPIO_SEL_SHIFT                               (0x00000010U)
#define CSL_IOMUX_EPWM6_A_CFG_REG_GPIO_SEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_EPWM6_A_CFG_REG_GPIO_SEL_MAX                                 (0x00000003U)

#define CSL_IOMUX_EPWM6_A_CFG_REG_QUAL_SEL_MASK                                (0x000C0000U)
#define CSL_IOMUX_EPWM6_A_CFG_REG_QUAL_SEL_SHIFT                               (0x00000012U)
#define CSL_IOMUX_EPWM6_A_CFG_REG_QUAL_SEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_EPWM6_A_CFG_REG_QUAL_SEL_MAX                                 (0x00000003U)

#define CSL_IOMUX_EPWM6_A_CFG_REG_INP_INV_SEL_MASK                             (0x00100000U)
#define CSL_IOMUX_EPWM6_A_CFG_REG_INP_INV_SEL_SHIFT                            (0x00000014U)
#define CSL_IOMUX_EPWM6_A_CFG_REG_INP_INV_SEL_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_EPWM6_A_CFG_REG_INP_INV_SEL_MAX                              (0x00000001U)

#define CSL_IOMUX_EPWM6_A_CFG_REG_ICSSM_GPIO_SEL_MASK                          (0x00200000U)
#define CSL_IOMUX_EPWM6_A_CFG_REG_ICSSM_GPIO_SEL_SHIFT                         (0x00000015U)
#define CSL_IOMUX_EPWM6_A_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_EPWM6_A_CFG_REG_ICSSM_GPIO_SEL_MAX                           (0x00000001U)

#define CSL_IOMUX_EPWM6_A_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                     (0x00400000U)
#define CSL_IOMUX_EPWM6_A_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                    (0x00000016U)
#define CSL_IOMUX_EPWM6_A_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL                 (0x00000000U)
#define CSL_IOMUX_EPWM6_A_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                      (0x00000001U)

#define CSL_IOMUX_EPWM6_A_CFG_REG_HSMODE_MASK                                  (0x40000000U)
#define CSL_IOMUX_EPWM6_A_CFG_REG_HSMODE_SHIFT                                 (0x0000001EU)
#define CSL_IOMUX_EPWM6_A_CFG_REG_HSMODE_RESETVAL                              (0x00000000U)
#define CSL_IOMUX_EPWM6_A_CFG_REG_HSMODE_MAX                                   (0x00000001U)

#define CSL_IOMUX_EPWM6_A_CFG_REG_HSMASTER_MASK                                (0x80000000U)
#define CSL_IOMUX_EPWM6_A_CFG_REG_HSMASTER_SHIFT                               (0x0000001FU)
#define CSL_IOMUX_EPWM6_A_CFG_REG_HSMASTER_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_EPWM6_A_CFG_REG_HSMASTER_MAX                                 (0x00000001U)

#define CSL_IOMUX_EPWM6_A_CFG_REG_RESETVAL                                     (0x000005F7U)

/* EPWM6_B_CFG_REG */

#define CSL_IOMUX_EPWM6_B_CFG_REG_FUNC_SEL_MASK                                (0x0000000FU)
#define CSL_IOMUX_EPWM6_B_CFG_REG_FUNC_SEL_SHIFT                               (0x00000000U)
#define CSL_IOMUX_EPWM6_B_CFG_REG_FUNC_SEL_RESETVAL                            (0x00000007U)
#define CSL_IOMUX_EPWM6_B_CFG_REG_FUNC_SEL_MAX                                 (0x0000000FU)

#define CSL_IOMUX_EPWM6_B_CFG_REG_IE_OVERRIDE_CTRL_MASK                        (0x00000010U)
#define CSL_IOMUX_EPWM6_B_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                       (0x00000004U)
#define CSL_IOMUX_EPWM6_B_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_EPWM6_B_CFG_REG_IE_OVERRIDE_CTRL_MAX                         (0x00000001U)

#define CSL_IOMUX_EPWM6_B_CFG_REG_IE_OVERRIDE_MASK                             (0x00000020U)
#define CSL_IOMUX_EPWM6_B_CFG_REG_IE_OVERRIDE_SHIFT                            (0x00000005U)
#define CSL_IOMUX_EPWM6_B_CFG_REG_IE_OVERRIDE_RESETVAL                         (0x00000001U)
#define CSL_IOMUX_EPWM6_B_CFG_REG_IE_OVERRIDE_MAX                              (0x00000001U)

#define CSL_IOMUX_EPWM6_B_CFG_REG_OE_OVERRIDE_CTRL_MASK                        (0x00000040U)
#define CSL_IOMUX_EPWM6_B_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                       (0x00000006U)
#define CSL_IOMUX_EPWM6_B_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_EPWM6_B_CFG_REG_OE_OVERRIDE_CTRL_MAX                         (0x00000001U)

#define CSL_IOMUX_EPWM6_B_CFG_REG_OE_OVERRIDE_MASK                             (0x00000080U)
#define CSL_IOMUX_EPWM6_B_CFG_REG_OE_OVERRIDE_SHIFT                            (0x00000007U)
#define CSL_IOMUX_EPWM6_B_CFG_REG_OE_OVERRIDE_RESETVAL                         (0x00000001U)
#define CSL_IOMUX_EPWM6_B_CFG_REG_OE_OVERRIDE_MAX                              (0x00000001U)

#define CSL_IOMUX_EPWM6_B_CFG_REG_PI_MASK                                      (0x00000100U)
#define CSL_IOMUX_EPWM6_B_CFG_REG_PI_SHIFT                                     (0x00000008U)
#define CSL_IOMUX_EPWM6_B_CFG_REG_PI_RESETVAL                                  (0x00000001U)
#define CSL_IOMUX_EPWM6_B_CFG_REG_PI_MAX                                       (0x00000001U)

#define CSL_IOMUX_EPWM6_B_CFG_REG_PUPDSEL_MASK                                 (0x00000200U)
#define CSL_IOMUX_EPWM6_B_CFG_REG_PUPDSEL_SHIFT                                (0x00000009U)
#define CSL_IOMUX_EPWM6_B_CFG_REG_PUPDSEL_RESETVAL                             (0x00000000U)
#define CSL_IOMUX_EPWM6_B_CFG_REG_PUPDSEL_MAX                                  (0x00000001U)

#define CSL_IOMUX_EPWM6_B_CFG_REG_SC1_MASK                                     (0x00000400U)
#define CSL_IOMUX_EPWM6_B_CFG_REG_SC1_SHIFT                                    (0x0000000AU)
#define CSL_IOMUX_EPWM6_B_CFG_REG_SC1_RESETVAL                                 (0x00000001U)
#define CSL_IOMUX_EPWM6_B_CFG_REG_SC1_MAX                                      (0x00000001U)

#define CSL_IOMUX_EPWM6_B_CFG_REG_GPIO_SEL_MASK                                (0x00030000U)
#define CSL_IOMUX_EPWM6_B_CFG_REG_GPIO_SEL_SHIFT                               (0x00000010U)
#define CSL_IOMUX_EPWM6_B_CFG_REG_GPIO_SEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_EPWM6_B_CFG_REG_GPIO_SEL_MAX                                 (0x00000003U)

#define CSL_IOMUX_EPWM6_B_CFG_REG_QUAL_SEL_MASK                                (0x000C0000U)
#define CSL_IOMUX_EPWM6_B_CFG_REG_QUAL_SEL_SHIFT                               (0x00000012U)
#define CSL_IOMUX_EPWM6_B_CFG_REG_QUAL_SEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_EPWM6_B_CFG_REG_QUAL_SEL_MAX                                 (0x00000003U)

#define CSL_IOMUX_EPWM6_B_CFG_REG_INP_INV_SEL_MASK                             (0x00100000U)
#define CSL_IOMUX_EPWM6_B_CFG_REG_INP_INV_SEL_SHIFT                            (0x00000014U)
#define CSL_IOMUX_EPWM6_B_CFG_REG_INP_INV_SEL_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_EPWM6_B_CFG_REG_INP_INV_SEL_MAX                              (0x00000001U)

#define CSL_IOMUX_EPWM6_B_CFG_REG_ICSSM_GPIO_SEL_MASK                          (0x00200000U)
#define CSL_IOMUX_EPWM6_B_CFG_REG_ICSSM_GPIO_SEL_SHIFT                         (0x00000015U)
#define CSL_IOMUX_EPWM6_B_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_EPWM6_B_CFG_REG_ICSSM_GPIO_SEL_MAX                           (0x00000001U)

#define CSL_IOMUX_EPWM6_B_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                     (0x00400000U)
#define CSL_IOMUX_EPWM6_B_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                    (0x00000016U)
#define CSL_IOMUX_EPWM6_B_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL                 (0x00000000U)
#define CSL_IOMUX_EPWM6_B_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                      (0x00000001U)

#define CSL_IOMUX_EPWM6_B_CFG_REG_HSMODE_MASK                                  (0x40000000U)
#define CSL_IOMUX_EPWM6_B_CFG_REG_HSMODE_SHIFT                                 (0x0000001EU)
#define CSL_IOMUX_EPWM6_B_CFG_REG_HSMODE_RESETVAL                              (0x00000000U)
#define CSL_IOMUX_EPWM6_B_CFG_REG_HSMODE_MAX                                   (0x00000001U)

#define CSL_IOMUX_EPWM6_B_CFG_REG_HSMASTER_MASK                                (0x80000000U)
#define CSL_IOMUX_EPWM6_B_CFG_REG_HSMASTER_SHIFT                               (0x0000001FU)
#define CSL_IOMUX_EPWM6_B_CFG_REG_HSMASTER_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_EPWM6_B_CFG_REG_HSMASTER_MAX                                 (0x00000001U)

#define CSL_IOMUX_EPWM6_B_CFG_REG_RESETVAL                                     (0x000005F7U)

/* EPWM7_A_CFG_REG */

#define CSL_IOMUX_EPWM7_A_CFG_REG_FUNC_SEL_MASK                                (0x0000000FU)
#define CSL_IOMUX_EPWM7_A_CFG_REG_FUNC_SEL_SHIFT                               (0x00000000U)
#define CSL_IOMUX_EPWM7_A_CFG_REG_FUNC_SEL_RESETVAL                            (0x00000007U)
#define CSL_IOMUX_EPWM7_A_CFG_REG_FUNC_SEL_MAX                                 (0x0000000FU)

#define CSL_IOMUX_EPWM7_A_CFG_REG_IE_OVERRIDE_CTRL_MASK                        (0x00000010U)
#define CSL_IOMUX_EPWM7_A_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                       (0x00000004U)
#define CSL_IOMUX_EPWM7_A_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_EPWM7_A_CFG_REG_IE_OVERRIDE_CTRL_MAX                         (0x00000001U)

#define CSL_IOMUX_EPWM7_A_CFG_REG_IE_OVERRIDE_MASK                             (0x00000020U)
#define CSL_IOMUX_EPWM7_A_CFG_REG_IE_OVERRIDE_SHIFT                            (0x00000005U)
#define CSL_IOMUX_EPWM7_A_CFG_REG_IE_OVERRIDE_RESETVAL                         (0x00000001U)
#define CSL_IOMUX_EPWM7_A_CFG_REG_IE_OVERRIDE_MAX                              (0x00000001U)

#define CSL_IOMUX_EPWM7_A_CFG_REG_OE_OVERRIDE_CTRL_MASK                        (0x00000040U)
#define CSL_IOMUX_EPWM7_A_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                       (0x00000006U)
#define CSL_IOMUX_EPWM7_A_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_EPWM7_A_CFG_REG_OE_OVERRIDE_CTRL_MAX                         (0x00000001U)

#define CSL_IOMUX_EPWM7_A_CFG_REG_OE_OVERRIDE_MASK                             (0x00000080U)
#define CSL_IOMUX_EPWM7_A_CFG_REG_OE_OVERRIDE_SHIFT                            (0x00000007U)
#define CSL_IOMUX_EPWM7_A_CFG_REG_OE_OVERRIDE_RESETVAL                         (0x00000001U)
#define CSL_IOMUX_EPWM7_A_CFG_REG_OE_OVERRIDE_MAX                              (0x00000001U)

#define CSL_IOMUX_EPWM7_A_CFG_REG_PI_MASK                                      (0x00000100U)
#define CSL_IOMUX_EPWM7_A_CFG_REG_PI_SHIFT                                     (0x00000008U)
#define CSL_IOMUX_EPWM7_A_CFG_REG_PI_RESETVAL                                  (0x00000001U)
#define CSL_IOMUX_EPWM7_A_CFG_REG_PI_MAX                                       (0x00000001U)

#define CSL_IOMUX_EPWM7_A_CFG_REG_PUPDSEL_MASK                                 (0x00000200U)
#define CSL_IOMUX_EPWM7_A_CFG_REG_PUPDSEL_SHIFT                                (0x00000009U)
#define CSL_IOMUX_EPWM7_A_CFG_REG_PUPDSEL_RESETVAL                             (0x00000000U)
#define CSL_IOMUX_EPWM7_A_CFG_REG_PUPDSEL_MAX                                  (0x00000001U)

#define CSL_IOMUX_EPWM7_A_CFG_REG_SC1_MASK                                     (0x00000400U)
#define CSL_IOMUX_EPWM7_A_CFG_REG_SC1_SHIFT                                    (0x0000000AU)
#define CSL_IOMUX_EPWM7_A_CFG_REG_SC1_RESETVAL                                 (0x00000001U)
#define CSL_IOMUX_EPWM7_A_CFG_REG_SC1_MAX                                      (0x00000001U)

#define CSL_IOMUX_EPWM7_A_CFG_REG_GPIO_SEL_MASK                                (0x00030000U)
#define CSL_IOMUX_EPWM7_A_CFG_REG_GPIO_SEL_SHIFT                               (0x00000010U)
#define CSL_IOMUX_EPWM7_A_CFG_REG_GPIO_SEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_EPWM7_A_CFG_REG_GPIO_SEL_MAX                                 (0x00000003U)

#define CSL_IOMUX_EPWM7_A_CFG_REG_QUAL_SEL_MASK                                (0x000C0000U)
#define CSL_IOMUX_EPWM7_A_CFG_REG_QUAL_SEL_SHIFT                               (0x00000012U)
#define CSL_IOMUX_EPWM7_A_CFG_REG_QUAL_SEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_EPWM7_A_CFG_REG_QUAL_SEL_MAX                                 (0x00000003U)

#define CSL_IOMUX_EPWM7_A_CFG_REG_INP_INV_SEL_MASK                             (0x00100000U)
#define CSL_IOMUX_EPWM7_A_CFG_REG_INP_INV_SEL_SHIFT                            (0x00000014U)
#define CSL_IOMUX_EPWM7_A_CFG_REG_INP_INV_SEL_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_EPWM7_A_CFG_REG_INP_INV_SEL_MAX                              (0x00000001U)

#define CSL_IOMUX_EPWM7_A_CFG_REG_ICSSM_GPIO_SEL_MASK                          (0x00200000U)
#define CSL_IOMUX_EPWM7_A_CFG_REG_ICSSM_GPIO_SEL_SHIFT                         (0x00000015U)
#define CSL_IOMUX_EPWM7_A_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_EPWM7_A_CFG_REG_ICSSM_GPIO_SEL_MAX                           (0x00000001U)

#define CSL_IOMUX_EPWM7_A_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                     (0x00400000U)
#define CSL_IOMUX_EPWM7_A_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                    (0x00000016U)
#define CSL_IOMUX_EPWM7_A_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL                 (0x00000000U)
#define CSL_IOMUX_EPWM7_A_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                      (0x00000001U)

#define CSL_IOMUX_EPWM7_A_CFG_REG_HSMODE_MASK                                  (0x40000000U)
#define CSL_IOMUX_EPWM7_A_CFG_REG_HSMODE_SHIFT                                 (0x0000001EU)
#define CSL_IOMUX_EPWM7_A_CFG_REG_HSMODE_RESETVAL                              (0x00000000U)
#define CSL_IOMUX_EPWM7_A_CFG_REG_HSMODE_MAX                                   (0x00000001U)

#define CSL_IOMUX_EPWM7_A_CFG_REG_HSMASTER_MASK                                (0x80000000U)
#define CSL_IOMUX_EPWM7_A_CFG_REG_HSMASTER_SHIFT                               (0x0000001FU)
#define CSL_IOMUX_EPWM7_A_CFG_REG_HSMASTER_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_EPWM7_A_CFG_REG_HSMASTER_MAX                                 (0x00000001U)

#define CSL_IOMUX_EPWM7_A_CFG_REG_RESETVAL                                     (0x000005F7U)

/* EPWM7_B_CFG_REG */

#define CSL_IOMUX_EPWM7_B_CFG_REG_FUNC_SEL_MASK                                (0x0000000FU)
#define CSL_IOMUX_EPWM7_B_CFG_REG_FUNC_SEL_SHIFT                               (0x00000000U)
#define CSL_IOMUX_EPWM7_B_CFG_REG_FUNC_SEL_RESETVAL                            (0x00000007U)
#define CSL_IOMUX_EPWM7_B_CFG_REG_FUNC_SEL_MAX                                 (0x0000000FU)

#define CSL_IOMUX_EPWM7_B_CFG_REG_IE_OVERRIDE_CTRL_MASK                        (0x00000010U)
#define CSL_IOMUX_EPWM7_B_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                       (0x00000004U)
#define CSL_IOMUX_EPWM7_B_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_EPWM7_B_CFG_REG_IE_OVERRIDE_CTRL_MAX                         (0x00000001U)

#define CSL_IOMUX_EPWM7_B_CFG_REG_IE_OVERRIDE_MASK                             (0x00000020U)
#define CSL_IOMUX_EPWM7_B_CFG_REG_IE_OVERRIDE_SHIFT                            (0x00000005U)
#define CSL_IOMUX_EPWM7_B_CFG_REG_IE_OVERRIDE_RESETVAL                         (0x00000001U)
#define CSL_IOMUX_EPWM7_B_CFG_REG_IE_OVERRIDE_MAX                              (0x00000001U)

#define CSL_IOMUX_EPWM7_B_CFG_REG_OE_OVERRIDE_CTRL_MASK                        (0x00000040U)
#define CSL_IOMUX_EPWM7_B_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                       (0x00000006U)
#define CSL_IOMUX_EPWM7_B_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_EPWM7_B_CFG_REG_OE_OVERRIDE_CTRL_MAX                         (0x00000001U)

#define CSL_IOMUX_EPWM7_B_CFG_REG_OE_OVERRIDE_MASK                             (0x00000080U)
#define CSL_IOMUX_EPWM7_B_CFG_REG_OE_OVERRIDE_SHIFT                            (0x00000007U)
#define CSL_IOMUX_EPWM7_B_CFG_REG_OE_OVERRIDE_RESETVAL                         (0x00000001U)
#define CSL_IOMUX_EPWM7_B_CFG_REG_OE_OVERRIDE_MAX                              (0x00000001U)

#define CSL_IOMUX_EPWM7_B_CFG_REG_PI_MASK                                      (0x00000100U)
#define CSL_IOMUX_EPWM7_B_CFG_REG_PI_SHIFT                                     (0x00000008U)
#define CSL_IOMUX_EPWM7_B_CFG_REG_PI_RESETVAL                                  (0x00000001U)
#define CSL_IOMUX_EPWM7_B_CFG_REG_PI_MAX                                       (0x00000001U)

#define CSL_IOMUX_EPWM7_B_CFG_REG_PUPDSEL_MASK                                 (0x00000200U)
#define CSL_IOMUX_EPWM7_B_CFG_REG_PUPDSEL_SHIFT                                (0x00000009U)
#define CSL_IOMUX_EPWM7_B_CFG_REG_PUPDSEL_RESETVAL                             (0x00000000U)
#define CSL_IOMUX_EPWM7_B_CFG_REG_PUPDSEL_MAX                                  (0x00000001U)

#define CSL_IOMUX_EPWM7_B_CFG_REG_SC1_MASK                                     (0x00000400U)
#define CSL_IOMUX_EPWM7_B_CFG_REG_SC1_SHIFT                                    (0x0000000AU)
#define CSL_IOMUX_EPWM7_B_CFG_REG_SC1_RESETVAL                                 (0x00000001U)
#define CSL_IOMUX_EPWM7_B_CFG_REG_SC1_MAX                                      (0x00000001U)

#define CSL_IOMUX_EPWM7_B_CFG_REG_GPIO_SEL_MASK                                (0x00030000U)
#define CSL_IOMUX_EPWM7_B_CFG_REG_GPIO_SEL_SHIFT                               (0x00000010U)
#define CSL_IOMUX_EPWM7_B_CFG_REG_GPIO_SEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_EPWM7_B_CFG_REG_GPIO_SEL_MAX                                 (0x00000003U)

#define CSL_IOMUX_EPWM7_B_CFG_REG_QUAL_SEL_MASK                                (0x000C0000U)
#define CSL_IOMUX_EPWM7_B_CFG_REG_QUAL_SEL_SHIFT                               (0x00000012U)
#define CSL_IOMUX_EPWM7_B_CFG_REG_QUAL_SEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_EPWM7_B_CFG_REG_QUAL_SEL_MAX                                 (0x00000003U)

#define CSL_IOMUX_EPWM7_B_CFG_REG_INP_INV_SEL_MASK                             (0x00100000U)
#define CSL_IOMUX_EPWM7_B_CFG_REG_INP_INV_SEL_SHIFT                            (0x00000014U)
#define CSL_IOMUX_EPWM7_B_CFG_REG_INP_INV_SEL_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_EPWM7_B_CFG_REG_INP_INV_SEL_MAX                              (0x00000001U)

#define CSL_IOMUX_EPWM7_B_CFG_REG_ICSSM_GPIO_SEL_MASK                          (0x00200000U)
#define CSL_IOMUX_EPWM7_B_CFG_REG_ICSSM_GPIO_SEL_SHIFT                         (0x00000015U)
#define CSL_IOMUX_EPWM7_B_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_EPWM7_B_CFG_REG_ICSSM_GPIO_SEL_MAX                           (0x00000001U)

#define CSL_IOMUX_EPWM7_B_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                     (0x00400000U)
#define CSL_IOMUX_EPWM7_B_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                    (0x00000016U)
#define CSL_IOMUX_EPWM7_B_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL                 (0x00000000U)
#define CSL_IOMUX_EPWM7_B_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                      (0x00000001U)

#define CSL_IOMUX_EPWM7_B_CFG_REG_HSMODE_MASK                                  (0x40000000U)
#define CSL_IOMUX_EPWM7_B_CFG_REG_HSMODE_SHIFT                                 (0x0000001EU)
#define CSL_IOMUX_EPWM7_B_CFG_REG_HSMODE_RESETVAL                              (0x00000000U)
#define CSL_IOMUX_EPWM7_B_CFG_REG_HSMODE_MAX                                   (0x00000001U)

#define CSL_IOMUX_EPWM7_B_CFG_REG_HSMASTER_MASK                                (0x80000000U)
#define CSL_IOMUX_EPWM7_B_CFG_REG_HSMASTER_SHIFT                               (0x0000001FU)
#define CSL_IOMUX_EPWM7_B_CFG_REG_HSMASTER_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_EPWM7_B_CFG_REG_HSMASTER_MAX                                 (0x00000001U)

#define CSL_IOMUX_EPWM7_B_CFG_REG_RESETVAL                                     (0x000005F7U)

/* EPWM8_A_CFG_REG */

#define CSL_IOMUX_EPWM8_A_CFG_REG_FUNC_SEL_MASK                                (0x0000000FU)
#define CSL_IOMUX_EPWM8_A_CFG_REG_FUNC_SEL_SHIFT                               (0x00000000U)
#define CSL_IOMUX_EPWM8_A_CFG_REG_FUNC_SEL_RESETVAL                            (0x00000007U)
#define CSL_IOMUX_EPWM8_A_CFG_REG_FUNC_SEL_MAX                                 (0x0000000FU)

#define CSL_IOMUX_EPWM8_A_CFG_REG_IE_OVERRIDE_CTRL_MASK                        (0x00000010U)
#define CSL_IOMUX_EPWM8_A_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                       (0x00000004U)
#define CSL_IOMUX_EPWM8_A_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_EPWM8_A_CFG_REG_IE_OVERRIDE_CTRL_MAX                         (0x00000001U)

#define CSL_IOMUX_EPWM8_A_CFG_REG_IE_OVERRIDE_MASK                             (0x00000020U)
#define CSL_IOMUX_EPWM8_A_CFG_REG_IE_OVERRIDE_SHIFT                            (0x00000005U)
#define CSL_IOMUX_EPWM8_A_CFG_REG_IE_OVERRIDE_RESETVAL                         (0x00000001U)
#define CSL_IOMUX_EPWM8_A_CFG_REG_IE_OVERRIDE_MAX                              (0x00000001U)

#define CSL_IOMUX_EPWM8_A_CFG_REG_OE_OVERRIDE_CTRL_MASK                        (0x00000040U)
#define CSL_IOMUX_EPWM8_A_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                       (0x00000006U)
#define CSL_IOMUX_EPWM8_A_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_EPWM8_A_CFG_REG_OE_OVERRIDE_CTRL_MAX                         (0x00000001U)

#define CSL_IOMUX_EPWM8_A_CFG_REG_OE_OVERRIDE_MASK                             (0x00000080U)
#define CSL_IOMUX_EPWM8_A_CFG_REG_OE_OVERRIDE_SHIFT                            (0x00000007U)
#define CSL_IOMUX_EPWM8_A_CFG_REG_OE_OVERRIDE_RESETVAL                         (0x00000001U)
#define CSL_IOMUX_EPWM8_A_CFG_REG_OE_OVERRIDE_MAX                              (0x00000001U)

#define CSL_IOMUX_EPWM8_A_CFG_REG_PI_MASK                                      (0x00000100U)
#define CSL_IOMUX_EPWM8_A_CFG_REG_PI_SHIFT                                     (0x00000008U)
#define CSL_IOMUX_EPWM8_A_CFG_REG_PI_RESETVAL                                  (0x00000001U)
#define CSL_IOMUX_EPWM8_A_CFG_REG_PI_MAX                                       (0x00000001U)

#define CSL_IOMUX_EPWM8_A_CFG_REG_PUPDSEL_MASK                                 (0x00000200U)
#define CSL_IOMUX_EPWM8_A_CFG_REG_PUPDSEL_SHIFT                                (0x00000009U)
#define CSL_IOMUX_EPWM8_A_CFG_REG_PUPDSEL_RESETVAL                             (0x00000000U)
#define CSL_IOMUX_EPWM8_A_CFG_REG_PUPDSEL_MAX                                  (0x00000001U)

#define CSL_IOMUX_EPWM8_A_CFG_REG_SC1_MASK                                     (0x00000400U)
#define CSL_IOMUX_EPWM8_A_CFG_REG_SC1_SHIFT                                    (0x0000000AU)
#define CSL_IOMUX_EPWM8_A_CFG_REG_SC1_RESETVAL                                 (0x00000001U)
#define CSL_IOMUX_EPWM8_A_CFG_REG_SC1_MAX                                      (0x00000001U)

#define CSL_IOMUX_EPWM8_A_CFG_REG_GPIO_SEL_MASK                                (0x00030000U)
#define CSL_IOMUX_EPWM8_A_CFG_REG_GPIO_SEL_SHIFT                               (0x00000010U)
#define CSL_IOMUX_EPWM8_A_CFG_REG_GPIO_SEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_EPWM8_A_CFG_REG_GPIO_SEL_MAX                                 (0x00000003U)

#define CSL_IOMUX_EPWM8_A_CFG_REG_QUAL_SEL_MASK                                (0x000C0000U)
#define CSL_IOMUX_EPWM8_A_CFG_REG_QUAL_SEL_SHIFT                               (0x00000012U)
#define CSL_IOMUX_EPWM8_A_CFG_REG_QUAL_SEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_EPWM8_A_CFG_REG_QUAL_SEL_MAX                                 (0x00000003U)

#define CSL_IOMUX_EPWM8_A_CFG_REG_INP_INV_SEL_MASK                             (0x00100000U)
#define CSL_IOMUX_EPWM8_A_CFG_REG_INP_INV_SEL_SHIFT                            (0x00000014U)
#define CSL_IOMUX_EPWM8_A_CFG_REG_INP_INV_SEL_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_EPWM8_A_CFG_REG_INP_INV_SEL_MAX                              (0x00000001U)

#define CSL_IOMUX_EPWM8_A_CFG_REG_ICSSM_GPIO_SEL_MASK                          (0x00200000U)
#define CSL_IOMUX_EPWM8_A_CFG_REG_ICSSM_GPIO_SEL_SHIFT                         (0x00000015U)
#define CSL_IOMUX_EPWM8_A_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_EPWM8_A_CFG_REG_ICSSM_GPIO_SEL_MAX                           (0x00000001U)

#define CSL_IOMUX_EPWM8_A_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                     (0x00400000U)
#define CSL_IOMUX_EPWM8_A_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                    (0x00000016U)
#define CSL_IOMUX_EPWM8_A_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL                 (0x00000000U)
#define CSL_IOMUX_EPWM8_A_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                      (0x00000001U)

#define CSL_IOMUX_EPWM8_A_CFG_REG_HSMODE_MASK                                  (0x40000000U)
#define CSL_IOMUX_EPWM8_A_CFG_REG_HSMODE_SHIFT                                 (0x0000001EU)
#define CSL_IOMUX_EPWM8_A_CFG_REG_HSMODE_RESETVAL                              (0x00000000U)
#define CSL_IOMUX_EPWM8_A_CFG_REG_HSMODE_MAX                                   (0x00000001U)

#define CSL_IOMUX_EPWM8_A_CFG_REG_HSMASTER_MASK                                (0x80000000U)
#define CSL_IOMUX_EPWM8_A_CFG_REG_HSMASTER_SHIFT                               (0x0000001FU)
#define CSL_IOMUX_EPWM8_A_CFG_REG_HSMASTER_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_EPWM8_A_CFG_REG_HSMASTER_MAX                                 (0x00000001U)

#define CSL_IOMUX_EPWM8_A_CFG_REG_RESETVAL                                     (0x000005F7U)

/* EPWM8_B_CFG_REG */

#define CSL_IOMUX_EPWM8_B_CFG_REG_FUNC_SEL_MASK                                (0x0000000FU)
#define CSL_IOMUX_EPWM8_B_CFG_REG_FUNC_SEL_SHIFT                               (0x00000000U)
#define CSL_IOMUX_EPWM8_B_CFG_REG_FUNC_SEL_RESETVAL                            (0x00000007U)
#define CSL_IOMUX_EPWM8_B_CFG_REG_FUNC_SEL_MAX                                 (0x0000000FU)

#define CSL_IOMUX_EPWM8_B_CFG_REG_IE_OVERRIDE_CTRL_MASK                        (0x00000010U)
#define CSL_IOMUX_EPWM8_B_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                       (0x00000004U)
#define CSL_IOMUX_EPWM8_B_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_EPWM8_B_CFG_REG_IE_OVERRIDE_CTRL_MAX                         (0x00000001U)

#define CSL_IOMUX_EPWM8_B_CFG_REG_IE_OVERRIDE_MASK                             (0x00000020U)
#define CSL_IOMUX_EPWM8_B_CFG_REG_IE_OVERRIDE_SHIFT                            (0x00000005U)
#define CSL_IOMUX_EPWM8_B_CFG_REG_IE_OVERRIDE_RESETVAL                         (0x00000001U)
#define CSL_IOMUX_EPWM8_B_CFG_REG_IE_OVERRIDE_MAX                              (0x00000001U)

#define CSL_IOMUX_EPWM8_B_CFG_REG_OE_OVERRIDE_CTRL_MASK                        (0x00000040U)
#define CSL_IOMUX_EPWM8_B_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                       (0x00000006U)
#define CSL_IOMUX_EPWM8_B_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_EPWM8_B_CFG_REG_OE_OVERRIDE_CTRL_MAX                         (0x00000001U)

#define CSL_IOMUX_EPWM8_B_CFG_REG_OE_OVERRIDE_MASK                             (0x00000080U)
#define CSL_IOMUX_EPWM8_B_CFG_REG_OE_OVERRIDE_SHIFT                            (0x00000007U)
#define CSL_IOMUX_EPWM8_B_CFG_REG_OE_OVERRIDE_RESETVAL                         (0x00000001U)
#define CSL_IOMUX_EPWM8_B_CFG_REG_OE_OVERRIDE_MAX                              (0x00000001U)

#define CSL_IOMUX_EPWM8_B_CFG_REG_PI_MASK                                      (0x00000100U)
#define CSL_IOMUX_EPWM8_B_CFG_REG_PI_SHIFT                                     (0x00000008U)
#define CSL_IOMUX_EPWM8_B_CFG_REG_PI_RESETVAL                                  (0x00000001U)
#define CSL_IOMUX_EPWM8_B_CFG_REG_PI_MAX                                       (0x00000001U)

#define CSL_IOMUX_EPWM8_B_CFG_REG_PUPDSEL_MASK                                 (0x00000200U)
#define CSL_IOMUX_EPWM8_B_CFG_REG_PUPDSEL_SHIFT                                (0x00000009U)
#define CSL_IOMUX_EPWM8_B_CFG_REG_PUPDSEL_RESETVAL                             (0x00000000U)
#define CSL_IOMUX_EPWM8_B_CFG_REG_PUPDSEL_MAX                                  (0x00000001U)

#define CSL_IOMUX_EPWM8_B_CFG_REG_SC1_MASK                                     (0x00000400U)
#define CSL_IOMUX_EPWM8_B_CFG_REG_SC1_SHIFT                                    (0x0000000AU)
#define CSL_IOMUX_EPWM8_B_CFG_REG_SC1_RESETVAL                                 (0x00000001U)
#define CSL_IOMUX_EPWM8_B_CFG_REG_SC1_MAX                                      (0x00000001U)

#define CSL_IOMUX_EPWM8_B_CFG_REG_GPIO_SEL_MASK                                (0x00030000U)
#define CSL_IOMUX_EPWM8_B_CFG_REG_GPIO_SEL_SHIFT                               (0x00000010U)
#define CSL_IOMUX_EPWM8_B_CFG_REG_GPIO_SEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_EPWM8_B_CFG_REG_GPIO_SEL_MAX                                 (0x00000003U)

#define CSL_IOMUX_EPWM8_B_CFG_REG_QUAL_SEL_MASK                                (0x000C0000U)
#define CSL_IOMUX_EPWM8_B_CFG_REG_QUAL_SEL_SHIFT                               (0x00000012U)
#define CSL_IOMUX_EPWM8_B_CFG_REG_QUAL_SEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_EPWM8_B_CFG_REG_QUAL_SEL_MAX                                 (0x00000003U)

#define CSL_IOMUX_EPWM8_B_CFG_REG_INP_INV_SEL_MASK                             (0x00100000U)
#define CSL_IOMUX_EPWM8_B_CFG_REG_INP_INV_SEL_SHIFT                            (0x00000014U)
#define CSL_IOMUX_EPWM8_B_CFG_REG_INP_INV_SEL_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_EPWM8_B_CFG_REG_INP_INV_SEL_MAX                              (0x00000001U)

#define CSL_IOMUX_EPWM8_B_CFG_REG_ICSSM_GPIO_SEL_MASK                          (0x00200000U)
#define CSL_IOMUX_EPWM8_B_CFG_REG_ICSSM_GPIO_SEL_SHIFT                         (0x00000015U)
#define CSL_IOMUX_EPWM8_B_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_EPWM8_B_CFG_REG_ICSSM_GPIO_SEL_MAX                           (0x00000001U)

#define CSL_IOMUX_EPWM8_B_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                     (0x00400000U)
#define CSL_IOMUX_EPWM8_B_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                    (0x00000016U)
#define CSL_IOMUX_EPWM8_B_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL                 (0x00000000U)
#define CSL_IOMUX_EPWM8_B_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                      (0x00000001U)

#define CSL_IOMUX_EPWM8_B_CFG_REG_HSMODE_MASK                                  (0x40000000U)
#define CSL_IOMUX_EPWM8_B_CFG_REG_HSMODE_SHIFT                                 (0x0000001EU)
#define CSL_IOMUX_EPWM8_B_CFG_REG_HSMODE_RESETVAL                              (0x00000000U)
#define CSL_IOMUX_EPWM8_B_CFG_REG_HSMODE_MAX                                   (0x00000001U)

#define CSL_IOMUX_EPWM8_B_CFG_REG_HSMASTER_MASK                                (0x80000000U)
#define CSL_IOMUX_EPWM8_B_CFG_REG_HSMASTER_SHIFT                               (0x0000001FU)
#define CSL_IOMUX_EPWM8_B_CFG_REG_HSMASTER_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_EPWM8_B_CFG_REG_HSMASTER_MAX                                 (0x00000001U)

#define CSL_IOMUX_EPWM8_B_CFG_REG_RESETVAL                                     (0x000005F7U)

/* EPWM9_A_CFG_REG */

#define CSL_IOMUX_EPWM9_A_CFG_REG_FUNC_SEL_MASK                                (0x0000000FU)
#define CSL_IOMUX_EPWM9_A_CFG_REG_FUNC_SEL_SHIFT                               (0x00000000U)
#define CSL_IOMUX_EPWM9_A_CFG_REG_FUNC_SEL_RESETVAL                            (0x00000007U)
#define CSL_IOMUX_EPWM9_A_CFG_REG_FUNC_SEL_MAX                                 (0x0000000FU)

#define CSL_IOMUX_EPWM9_A_CFG_REG_IE_OVERRIDE_CTRL_MASK                        (0x00000010U)
#define CSL_IOMUX_EPWM9_A_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                       (0x00000004U)
#define CSL_IOMUX_EPWM9_A_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_EPWM9_A_CFG_REG_IE_OVERRIDE_CTRL_MAX                         (0x00000001U)

#define CSL_IOMUX_EPWM9_A_CFG_REG_IE_OVERRIDE_MASK                             (0x00000020U)
#define CSL_IOMUX_EPWM9_A_CFG_REG_IE_OVERRIDE_SHIFT                            (0x00000005U)
#define CSL_IOMUX_EPWM9_A_CFG_REG_IE_OVERRIDE_RESETVAL                         (0x00000001U)
#define CSL_IOMUX_EPWM9_A_CFG_REG_IE_OVERRIDE_MAX                              (0x00000001U)

#define CSL_IOMUX_EPWM9_A_CFG_REG_OE_OVERRIDE_CTRL_MASK                        (0x00000040U)
#define CSL_IOMUX_EPWM9_A_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                       (0x00000006U)
#define CSL_IOMUX_EPWM9_A_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_EPWM9_A_CFG_REG_OE_OVERRIDE_CTRL_MAX                         (0x00000001U)

#define CSL_IOMUX_EPWM9_A_CFG_REG_OE_OVERRIDE_MASK                             (0x00000080U)
#define CSL_IOMUX_EPWM9_A_CFG_REG_OE_OVERRIDE_SHIFT                            (0x00000007U)
#define CSL_IOMUX_EPWM9_A_CFG_REG_OE_OVERRIDE_RESETVAL                         (0x00000001U)
#define CSL_IOMUX_EPWM9_A_CFG_REG_OE_OVERRIDE_MAX                              (0x00000001U)

#define CSL_IOMUX_EPWM9_A_CFG_REG_PI_MASK                                      (0x00000100U)
#define CSL_IOMUX_EPWM9_A_CFG_REG_PI_SHIFT                                     (0x00000008U)
#define CSL_IOMUX_EPWM9_A_CFG_REG_PI_RESETVAL                                  (0x00000001U)
#define CSL_IOMUX_EPWM9_A_CFG_REG_PI_MAX                                       (0x00000001U)

#define CSL_IOMUX_EPWM9_A_CFG_REG_PUPDSEL_MASK                                 (0x00000200U)
#define CSL_IOMUX_EPWM9_A_CFG_REG_PUPDSEL_SHIFT                                (0x00000009U)
#define CSL_IOMUX_EPWM9_A_CFG_REG_PUPDSEL_RESETVAL                             (0x00000000U)
#define CSL_IOMUX_EPWM9_A_CFG_REG_PUPDSEL_MAX                                  (0x00000001U)

#define CSL_IOMUX_EPWM9_A_CFG_REG_SC1_MASK                                     (0x00000400U)
#define CSL_IOMUX_EPWM9_A_CFG_REG_SC1_SHIFT                                    (0x0000000AU)
#define CSL_IOMUX_EPWM9_A_CFG_REG_SC1_RESETVAL                                 (0x00000001U)
#define CSL_IOMUX_EPWM9_A_CFG_REG_SC1_MAX                                      (0x00000001U)

#define CSL_IOMUX_EPWM9_A_CFG_REG_GPIO_SEL_MASK                                (0x00030000U)
#define CSL_IOMUX_EPWM9_A_CFG_REG_GPIO_SEL_SHIFT                               (0x00000010U)
#define CSL_IOMUX_EPWM9_A_CFG_REG_GPIO_SEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_EPWM9_A_CFG_REG_GPIO_SEL_MAX                                 (0x00000003U)

#define CSL_IOMUX_EPWM9_A_CFG_REG_QUAL_SEL_MASK                                (0x000C0000U)
#define CSL_IOMUX_EPWM9_A_CFG_REG_QUAL_SEL_SHIFT                               (0x00000012U)
#define CSL_IOMUX_EPWM9_A_CFG_REG_QUAL_SEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_EPWM9_A_CFG_REG_QUAL_SEL_MAX                                 (0x00000003U)

#define CSL_IOMUX_EPWM9_A_CFG_REG_INP_INV_SEL_MASK                             (0x00100000U)
#define CSL_IOMUX_EPWM9_A_CFG_REG_INP_INV_SEL_SHIFT                            (0x00000014U)
#define CSL_IOMUX_EPWM9_A_CFG_REG_INP_INV_SEL_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_EPWM9_A_CFG_REG_INP_INV_SEL_MAX                              (0x00000001U)

#define CSL_IOMUX_EPWM9_A_CFG_REG_ICSSM_GPIO_SEL_MASK                          (0x00200000U)
#define CSL_IOMUX_EPWM9_A_CFG_REG_ICSSM_GPIO_SEL_SHIFT                         (0x00000015U)
#define CSL_IOMUX_EPWM9_A_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_EPWM9_A_CFG_REG_ICSSM_GPIO_SEL_MAX                           (0x00000001U)

#define CSL_IOMUX_EPWM9_A_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                     (0x00400000U)
#define CSL_IOMUX_EPWM9_A_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                    (0x00000016U)
#define CSL_IOMUX_EPWM9_A_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL                 (0x00000000U)
#define CSL_IOMUX_EPWM9_A_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                      (0x00000001U)

#define CSL_IOMUX_EPWM9_A_CFG_REG_HSMODE_MASK                                  (0x40000000U)
#define CSL_IOMUX_EPWM9_A_CFG_REG_HSMODE_SHIFT                                 (0x0000001EU)
#define CSL_IOMUX_EPWM9_A_CFG_REG_HSMODE_RESETVAL                              (0x00000000U)
#define CSL_IOMUX_EPWM9_A_CFG_REG_HSMODE_MAX                                   (0x00000001U)

#define CSL_IOMUX_EPWM9_A_CFG_REG_HSMASTER_MASK                                (0x80000000U)
#define CSL_IOMUX_EPWM9_A_CFG_REG_HSMASTER_SHIFT                               (0x0000001FU)
#define CSL_IOMUX_EPWM9_A_CFG_REG_HSMASTER_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_EPWM9_A_CFG_REG_HSMASTER_MAX                                 (0x00000001U)

#define CSL_IOMUX_EPWM9_A_CFG_REG_RESETVAL                                     (0x000005F7U)

/* EPWM9_B_CFG_REG */

#define CSL_IOMUX_EPWM9_B_CFG_REG_FUNC_SEL_MASK                                (0x0000000FU)
#define CSL_IOMUX_EPWM9_B_CFG_REG_FUNC_SEL_SHIFT                               (0x00000000U)
#define CSL_IOMUX_EPWM9_B_CFG_REG_FUNC_SEL_RESETVAL                            (0x00000007U)
#define CSL_IOMUX_EPWM9_B_CFG_REG_FUNC_SEL_MAX                                 (0x0000000FU)

#define CSL_IOMUX_EPWM9_B_CFG_REG_IE_OVERRIDE_CTRL_MASK                        (0x00000010U)
#define CSL_IOMUX_EPWM9_B_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                       (0x00000004U)
#define CSL_IOMUX_EPWM9_B_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_EPWM9_B_CFG_REG_IE_OVERRIDE_CTRL_MAX                         (0x00000001U)

#define CSL_IOMUX_EPWM9_B_CFG_REG_IE_OVERRIDE_MASK                             (0x00000020U)
#define CSL_IOMUX_EPWM9_B_CFG_REG_IE_OVERRIDE_SHIFT                            (0x00000005U)
#define CSL_IOMUX_EPWM9_B_CFG_REG_IE_OVERRIDE_RESETVAL                         (0x00000001U)
#define CSL_IOMUX_EPWM9_B_CFG_REG_IE_OVERRIDE_MAX                              (0x00000001U)

#define CSL_IOMUX_EPWM9_B_CFG_REG_OE_OVERRIDE_CTRL_MASK                        (0x00000040U)
#define CSL_IOMUX_EPWM9_B_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                       (0x00000006U)
#define CSL_IOMUX_EPWM9_B_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_EPWM9_B_CFG_REG_OE_OVERRIDE_CTRL_MAX                         (0x00000001U)

#define CSL_IOMUX_EPWM9_B_CFG_REG_OE_OVERRIDE_MASK                             (0x00000080U)
#define CSL_IOMUX_EPWM9_B_CFG_REG_OE_OVERRIDE_SHIFT                            (0x00000007U)
#define CSL_IOMUX_EPWM9_B_CFG_REG_OE_OVERRIDE_RESETVAL                         (0x00000001U)
#define CSL_IOMUX_EPWM9_B_CFG_REG_OE_OVERRIDE_MAX                              (0x00000001U)

#define CSL_IOMUX_EPWM9_B_CFG_REG_PI_MASK                                      (0x00000100U)
#define CSL_IOMUX_EPWM9_B_CFG_REG_PI_SHIFT                                     (0x00000008U)
#define CSL_IOMUX_EPWM9_B_CFG_REG_PI_RESETVAL                                  (0x00000001U)
#define CSL_IOMUX_EPWM9_B_CFG_REG_PI_MAX                                       (0x00000001U)

#define CSL_IOMUX_EPWM9_B_CFG_REG_PUPDSEL_MASK                                 (0x00000200U)
#define CSL_IOMUX_EPWM9_B_CFG_REG_PUPDSEL_SHIFT                                (0x00000009U)
#define CSL_IOMUX_EPWM9_B_CFG_REG_PUPDSEL_RESETVAL                             (0x00000000U)
#define CSL_IOMUX_EPWM9_B_CFG_REG_PUPDSEL_MAX                                  (0x00000001U)

#define CSL_IOMUX_EPWM9_B_CFG_REG_SC1_MASK                                     (0x00000400U)
#define CSL_IOMUX_EPWM9_B_CFG_REG_SC1_SHIFT                                    (0x0000000AU)
#define CSL_IOMUX_EPWM9_B_CFG_REG_SC1_RESETVAL                                 (0x00000001U)
#define CSL_IOMUX_EPWM9_B_CFG_REG_SC1_MAX                                      (0x00000001U)

#define CSL_IOMUX_EPWM9_B_CFG_REG_GPIO_SEL_MASK                                (0x00030000U)
#define CSL_IOMUX_EPWM9_B_CFG_REG_GPIO_SEL_SHIFT                               (0x00000010U)
#define CSL_IOMUX_EPWM9_B_CFG_REG_GPIO_SEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_EPWM9_B_CFG_REG_GPIO_SEL_MAX                                 (0x00000003U)

#define CSL_IOMUX_EPWM9_B_CFG_REG_QUAL_SEL_MASK                                (0x000C0000U)
#define CSL_IOMUX_EPWM9_B_CFG_REG_QUAL_SEL_SHIFT                               (0x00000012U)
#define CSL_IOMUX_EPWM9_B_CFG_REG_QUAL_SEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_EPWM9_B_CFG_REG_QUAL_SEL_MAX                                 (0x00000003U)

#define CSL_IOMUX_EPWM9_B_CFG_REG_INP_INV_SEL_MASK                             (0x00100000U)
#define CSL_IOMUX_EPWM9_B_CFG_REG_INP_INV_SEL_SHIFT                            (0x00000014U)
#define CSL_IOMUX_EPWM9_B_CFG_REG_INP_INV_SEL_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_EPWM9_B_CFG_REG_INP_INV_SEL_MAX                              (0x00000001U)

#define CSL_IOMUX_EPWM9_B_CFG_REG_ICSSM_GPIO_SEL_MASK                          (0x00200000U)
#define CSL_IOMUX_EPWM9_B_CFG_REG_ICSSM_GPIO_SEL_SHIFT                         (0x00000015U)
#define CSL_IOMUX_EPWM9_B_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_EPWM9_B_CFG_REG_ICSSM_GPIO_SEL_MAX                           (0x00000001U)

#define CSL_IOMUX_EPWM9_B_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                     (0x00400000U)
#define CSL_IOMUX_EPWM9_B_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                    (0x00000016U)
#define CSL_IOMUX_EPWM9_B_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL                 (0x00000000U)
#define CSL_IOMUX_EPWM9_B_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                      (0x00000001U)

#define CSL_IOMUX_EPWM9_B_CFG_REG_HSMODE_MASK                                  (0x40000000U)
#define CSL_IOMUX_EPWM9_B_CFG_REG_HSMODE_SHIFT                                 (0x0000001EU)
#define CSL_IOMUX_EPWM9_B_CFG_REG_HSMODE_RESETVAL                              (0x00000000U)
#define CSL_IOMUX_EPWM9_B_CFG_REG_HSMODE_MAX                                   (0x00000001U)

#define CSL_IOMUX_EPWM9_B_CFG_REG_HSMASTER_MASK                                (0x80000000U)
#define CSL_IOMUX_EPWM9_B_CFG_REG_HSMASTER_SHIFT                               (0x0000001FU)
#define CSL_IOMUX_EPWM9_B_CFG_REG_HSMASTER_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_EPWM9_B_CFG_REG_HSMASTER_MAX                                 (0x00000001U)

#define CSL_IOMUX_EPWM9_B_CFG_REG_RESETVAL                                     (0x000005F7U)

/* GPIO63_CFG_REG */
#define CSL_IOMUX_GPIO63_CFG_REG_FUNC_SEL_MASK                                  (0x0000000FU)
#define CSL_IOMUX_GPIO63_CFG_REG_FUNC_SEL_SHIFT                                 (0x00000000U)
#define CSL_IOMUX_GPIO63_CFG_REG_FUNC_SEL_RESETVAL                              (0x00000007U)
#define CSL_IOMUX_GPIO63_CFG_REG_FUNC_SEL_MAX                                   (0x0000000FU)


#define CSL_IOMUX_GPIO63_CFG_REG_GPIO_SEL_MASK                                  (0x00010000U)
#define CSL_IOMUX_GPIO63_CFG_REG_GPIO_SEL_SHIFT                                 (0x00000010U)
#define CSL_IOMUX_GPIO63_CFG_REG_GPIO_SEL_RESETVAL                              (0x00000000U)
#define CSL_IOMUX_GPIO63_CFG_REG_GPIO_SEL_MAX                                   (0x00000001U)


#define CSL_IOMUX_GPIO63_CFG_REG_HSMASTER_MASK                                  (0x80000000U)
#define CSL_IOMUX_GPIO63_CFG_REG_HSMASTER_SHIFT                                  (0x80000000U)
#define CSL_IOMUX_GPIO63_CFG_REG_HSMASTER_RESETVAL                              (0x00000000U)
#define CSL_IOMUX_GPIO63_CFG_REG_HSMASTER_MAX                                   (0x00000001U)


#define CSL_IOMUX_GPIO63_CFG_REG_HSMODE_MASK                                  (0x40000000U)
#define CSL_IOMUX_GPIO63_CFG_REG_HSMODE_SHIFT                                 (0x0000001EU)
#define CSL_IOMUX_GPIO63_CFG_REG_HSMODE_RESETVAL                              (0x00000000U)
#define CSL_IOMUX_GPIO63_CFG_REG_HSMODE_MAX                                   (0x00000001U)


#define CSL_IOMUX_GPIO63_CFG_REG_HVMODE_STATUS_MASK                             (0x01000000U)
#define CSL_IOMUX_GPIO63_CFG_REG_HVMODE_STATUS_SHIFT                            (0x00000018U)
#define CSL_IOMUX_GPIO63_CFG_REG_HVMODE_STATUS_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_GPIO63_CFG_REG_HVMODE_STATUS_MAX                              (0x00000001U)


#define CSL_IOMUX_GPIO63_CFG_REG_ICSSM_GPIO_SEL_MASK                            (0x00200000U)
#define CSL_IOMUX_GPIO63_CFG_REG_ICSSM_GPIO_SEL_SHIFT                           (0x00000015U)
#define CSL_IOMUX_GPIO63_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                        (0x00000000U)
#define CSL_IOMUX_GPIO63_CFG_REG_ICSSM_GPIO_SEL_MAX                             (0x00000001U)


#define CSL_IOMUX_GPIO63_CFG_REG_IE_OVERRIDE_MASK                               (0x00000020U)
#define CSL_IOMUX_GPIO63_CFG_REG_IE_OVERRIDE_SHIFT                              (0x00000005U)
#define CSL_IOMUX_GPIO63_CFG_REG_IE_OVERRIDE_RESETVAL                           (0x00000001U)
#define CSL_IOMUX_GPIO63_CFG_REG_IE_OVERRIDE_MAX                                (0x00000001U)


#define CSL_IOMUX_GPIO63_CFG_REG_IE_OVERRIDE_CTRL_MASK                          (0x00000010U)
#define CSL_IOMUX_GPIO63_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                         (0x00000004U)
#define CSL_IOMUX_GPIO63_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                      (0x00000001U)
#define CSL_IOMUX_GPIO63_CFG_REG_IE_OVERRIDE_CTRL_MAX                           (0x00000001U)


#define CSL_IOMUX_GPIO63_CFG_REG_INP_INV_SEL_MASK                               (0x00100000U)
#define CSL_IOMUX_GPIO63_CFG_REG_INP_INV_SEL_SHIFT                              (0x00000014U)
#define CSL_IOMUX_GPIO63_CFG_REG_INP_INV_SEL_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_GPIO63_CFG_REG_INP_INV_SEL_MAX                                (0x00000001U)


#define CSL_IOMUX_GPIO63_CFG_REG_OE_OVERRIDE_MASK                               (0x00000080U)
#define CSL_IOMUX_GPIO63_CFG_REG_OE_OVERRIDE_SHIFT                              (0x00000007U)
#define CSL_IOMUX_GPIO63_CFG_REG_OE_OVERRIDE_RESETVAL                           (0x00000001U)
#define CSL_IOMUX_GPIO63_CFG_REG_OE_OVERRIDE_MAX                                (0x00000001U)


#define CSL_IOMUX_GPIO63_CFG_REG_OE_OVERRIDE_CTRL_MASK                          (0x00000040U)
#define CSL_IOMUX_GPIO63_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                         (0x00000006U)
#define CSL_IOMUX_GPIO63_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                      (0x00000001U)
#define CSL_IOMUX_GPIO63_CFG_REG_OE_OVERRIDE_CTRL_MAX                           (0x00000001U)


#define CSL_IOMUX_GPIO63_CFG_REG_PE_MASK                                        (0x00000100U)
#define CSL_IOMUX_GPIO63_CFG_REG_PE_SHIFT                                       (0x00000008U)
#define CSL_IOMUX_GPIO63_CFG_REG_PE_RESETVAL                                    (0x00000001U)
#define CSL_IOMUX_GPIO63_CFG_REG_PE_MAX                                         (0x00000001U)


#define CSL_IOMUX_GPIO63_CFG_REG_PUPDSEL_MASK                                   (0x00000200U)
#define CSL_IOMUX_GPIO63_CFG_REG_PUPDSEL_SHIFT                                  (0x00000009U)
#define CSL_IOMUX_GPIO63_CFG_REG_PUPDSEL_RESETVAL                               (0x00000000U)
#define CSL_IOMUX_GPIO63_CFG_REG_PUPDSEL_MAX                                    (0x00000001U)


#define CSL_IOMUX_GPIO63_CFG_REG_QUAL_SEL_MASK                                  (0x000C0000U)
#define CSL_IOMUX_GPIO63_CFG_REG_QUAL_SEL_SHIFT                                 (0x00000012U)
#define CSL_IOMUX_GPIO63_CFG_REG_QUAL_SEL_RESETVAL                              (0x00000000U)
#define CSL_IOMUX_GPIO63_CFG_REG_QUAL_SEL_MAX                                   (0x00000003U)


#define CSL_IOMUX_GPIO63_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                       (0x00400000U)
#define CSL_IOMUX_GPIO63_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                      (0x00000016U)
#define CSL_IOMUX_GPIO63_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL                   (0x00000000U)
#define CSL_IOMUX_GPIO63_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                        (0x00000001U)


#define CSL_IOMUX_GPIO63_CFG_REG_SC1_MASK                                       (0x00000400U)
#define CSL_IOMUX_GPIO63_CFG_REG_SC1_SHIFT                                      (0x0000000AU)
#define CSL_IOMUX_GPIO63_CFG_REG_SC1_RESETVAL                                   (0x00000001U)
#define CSL_IOMUX_GPIO63_CFG_REG_SC1_MAX                                        (0x00000001U)



/* GPIO64_CFG_REG */
#define CSL_IOMUX_GPIO64_CFG_REG_FUNC_SEL_MASK                                  (0x0000000FU)
#define CSL_IOMUX_GPIO64_CFG_REG_FUNC_SEL_SHIFT                                 (0x00000000U)
#define CSL_IOMUX_GPIO64_CFG_REG_FUNC_SEL_RESETVAL                              (0x00000007U)
#define CSL_IOMUX_GPIO64_CFG_REG_FUNC_SEL_MAX                                   (0x0000000FU)


#define CSL_IOMUX_GPIO64_CFG_REG_GPIO_SEL_MASK                                  (0x00010000U)
#define CSL_IOMUX_GPIO64_CFG_REG_GPIO_SEL_SHIFT                                 (0x00000010U)
#define CSL_IOMUX_GPIO64_CFG_REG_GPIO_SEL_RESETVAL                              (0x00000000U)
#define CSL_IOMUX_GPIO64_CFG_REG_GPIO_SEL_MAX                                   (0x00000001U)


#define CSL_IOMUX_GPIO64_CFG_REG_HSMASTER_MASK                                  (0x80000000U)
#define CSL_IOMUX_GPIO64_CFG_REG_HSMASTER_SHIFT                                  (0x80000000U)
#define CSL_IOMUX_GPIO64_CFG_REG_HSMASTER_RESETVAL                              (0x00000000U)
#define CSL_IOMUX_GPIO64_CFG_REG_HSMASTER_MAX                                   (0x00000001U)


#define CSL_IOMUX_GPIO64_CFG_REG_HSMODE_MASK                                  (0x40000000U)
#define CSL_IOMUX_GPIO64_CFG_REG_HSMODE_SHIFT                                 (0x0000001EU)
#define CSL_IOMUX_GPIO64_CFG_REG_HSMODE_RESETVAL                              (0x00000000U)
#define CSL_IOMUX_GPIO64_CFG_REG_HSMODE_MAX                                   (0x00000001U)


#define CSL_IOMUX_GPIO64_CFG_REG_HVMODE_STATUS_MASK                             (0x01000000U)
#define CSL_IOMUX_GPIO64_CFG_REG_HVMODE_STATUS_SHIFT                            (0x00000018U)
#define CSL_IOMUX_GPIO64_CFG_REG_HVMODE_STATUS_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_GPIO64_CFG_REG_HVMODE_STATUS_MAX                              (0x00000001U)


#define CSL_IOMUX_GPIO64_CFG_REG_ICSSM_GPIO_SEL_MASK                            (0x00200000U)
#define CSL_IOMUX_GPIO64_CFG_REG_ICSSM_GPIO_SEL_SHIFT                           (0x00000015U)
#define CSL_IOMUX_GPIO64_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                        (0x00000000U)
#define CSL_IOMUX_GPIO64_CFG_REG_ICSSM_GPIO_SEL_MAX                             (0x00000001U)


#define CSL_IOMUX_GPIO64_CFG_REG_IE_OVERRIDE_MASK                               (0x00000020U)
#define CSL_IOMUX_GPIO64_CFG_REG_IE_OVERRIDE_SHIFT                              (0x00000005U)
#define CSL_IOMUX_GPIO64_CFG_REG_IE_OVERRIDE_RESETVAL                           (0x00000001U)
#define CSL_IOMUX_GPIO64_CFG_REG_IE_OVERRIDE_MAX                                (0x00000001U)


#define CSL_IOMUX_GPIO64_CFG_REG_IE_OVERRIDE_CTRL_MASK                          (0x00000010U)
#define CSL_IOMUX_GPIO64_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                         (0x00000004U)
#define CSL_IOMUX_GPIO64_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                      (0x00000001U)
#define CSL_IOMUX_GPIO64_CFG_REG_IE_OVERRIDE_CTRL_MAX                           (0x00000001U)


#define CSL_IOMUX_GPIO64_CFG_REG_INP_INV_SEL_MASK                               (0x00100000U)
#define CSL_IOMUX_GPIO64_CFG_REG_INP_INV_SEL_SHIFT                              (0x00000014U)
#define CSL_IOMUX_GPIO64_CFG_REG_INP_INV_SEL_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_GPIO64_CFG_REG_INP_INV_SEL_MAX                                (0x00000001U)


#define CSL_IOMUX_GPIO64_CFG_REG_OE_OVERRIDE_MASK                               (0x00000080U)
#define CSL_IOMUX_GPIO64_CFG_REG_OE_OVERRIDE_SHIFT                              (0x00000007U)
#define CSL_IOMUX_GPIO64_CFG_REG_OE_OVERRIDE_RESETVAL                           (0x00000001U)
#define CSL_IOMUX_GPIO64_CFG_REG_OE_OVERRIDE_MAX                                (0x00000001U)


#define CSL_IOMUX_GPIO64_CFG_REG_OE_OVERRIDE_CTRL_MASK                          (0x00000040U)
#define CSL_IOMUX_GPIO64_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                         (0x00000006U)
#define CSL_IOMUX_GPIO64_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                      (0x00000001U)
#define CSL_IOMUX_GPIO64_CFG_REG_OE_OVERRIDE_CTRL_MAX                           (0x00000001U)


#define CSL_IOMUX_GPIO64_CFG_REG_PE_MASK                                        (0x00000100U)
#define CSL_IOMUX_GPIO64_CFG_REG_PE_SHIFT                                       (0x00000008U)
#define CSL_IOMUX_GPIO64_CFG_REG_PE_RESETVAL                                    (0x00000001U)
#define CSL_IOMUX_GPIO64_CFG_REG_PE_MAX                                         (0x00000001U)


#define CSL_IOMUX_GPIO64_CFG_REG_PUPDSEL_MASK                                   (0x00000200U)
#define CSL_IOMUX_GPIO64_CFG_REG_PUPDSEL_SHIFT                                  (0x00000009U)
#define CSL_IOMUX_GPIO64_CFG_REG_PUPDSEL_RESETVAL                               (0x00000000U)
#define CSL_IOMUX_GPIO64_CFG_REG_PUPDSEL_MAX                                    (0x00000001U)


#define CSL_IOMUX_GPIO64_CFG_REG_QUAL_SEL_MASK                                  (0x000C0000U)
#define CSL_IOMUX_GPIO64_CFG_REG_QUAL_SEL_SHIFT                                 (0x00000012U)
#define CSL_IOMUX_GPIO64_CFG_REG_QUAL_SEL_RESETVAL                              (0x00000000U)
#define CSL_IOMUX_GPIO64_CFG_REG_QUAL_SEL_MAX                                   (0x00000003U)


#define CSL_IOMUX_GPIO64_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                       (0x00400000U)
#define CSL_IOMUX_GPIO64_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                      (0x00000016U)
#define CSL_IOMUX_GPIO64_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL                   (0x00000000U)
#define CSL_IOMUX_GPIO64_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                        (0x00000001U)


#define CSL_IOMUX_GPIO64_CFG_REG_SC1_MASK                                       (0x00000400U)
#define CSL_IOMUX_GPIO64_CFG_REG_SC1_SHIFT                                      (0x0000000AU)
#define CSL_IOMUX_GPIO64_CFG_REG_SC1_RESETVAL                                   (0x00000001U)
#define CSL_IOMUX_GPIO64_CFG_REG_SC1_MAX                                        (0x00000001U)



/* GPIO65_CFG_REG */
#define CSL_IOMUX_GPIO65_CFG_REG_FUNC_SEL_MASK                                  (0x0000000FU)
#define CSL_IOMUX_GPIO65_CFG_REG_FUNC_SEL_SHIFT                                 (0x00000000U)
#define CSL_IOMUX_GPIO65_CFG_REG_FUNC_SEL_RESETVAL                              (0x00000007U)
#define CSL_IOMUX_GPIO65_CFG_REG_FUNC_SEL_MAX                                   (0x0000000FU)


#define CSL_IOMUX_GPIO65_CFG_REG_GPIO_SEL_MASK                                  (0x00010000U)
#define CSL_IOMUX_GPIO65_CFG_REG_GPIO_SEL_SHIFT                                 (0x00000010U)
#define CSL_IOMUX_GPIO65_CFG_REG_GPIO_SEL_RESETVAL                              (0x00000000U)
#define CSL_IOMUX_GPIO65_CFG_REG_GPIO_SEL_MAX                                   (0x00000001U)


#define CSL_IOMUX_GPIO65_CFG_REG_HSMASTER_MASK                                  (0x80000000U)
#define CSL_IOMUX_GPIO65_CFG_REG_HSMASTER_SHIFT                                  (0x80000000U)
#define CSL_IOMUX_GPIO65_CFG_REG_HSMASTER_RESETVAL                              (0x00000000U)
#define CSL_IOMUX_GPIO65_CFG_REG_HSMASTER_MAX                                   (0x00000001U)


#define CSL_IOMUX_GPIO65_CFG_REG_HSMODE_MASK                                  (0x40000000U)
#define CSL_IOMUX_GPIO65_CFG_REG_HSMODE_SHIFT                                 (0x0000001EU)
#define CSL_IOMUX_GPIO65_CFG_REG_HSMODE_RESETVAL                              (0x00000000U)
#define CSL_IOMUX_GPIO65_CFG_REG_HSMODE_MAX                                   (0x00000001U)


#define CSL_IOMUX_GPIO65_CFG_REG_HVMODE_STATUS_MASK                             (0x01000000U)
#define CSL_IOMUX_GPIO65_CFG_REG_HVMODE_STATUS_SHIFT                            (0x00000018U)
#define CSL_IOMUX_GPIO65_CFG_REG_HVMODE_STATUS_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_GPIO65_CFG_REG_HVMODE_STATUS_MAX                              (0x00000001U)


#define CSL_IOMUX_GPIO65_CFG_REG_ICSSM_GPIO_SEL_MASK                            (0x00200000U)
#define CSL_IOMUX_GPIO65_CFG_REG_ICSSM_GPIO_SEL_SHIFT                           (0x00000015U)
#define CSL_IOMUX_GPIO65_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                        (0x00000000U)
#define CSL_IOMUX_GPIO65_CFG_REG_ICSSM_GPIO_SEL_MAX                             (0x00000001U)


#define CSL_IOMUX_GPIO65_CFG_REG_IE_OVERRIDE_MASK                               (0x00000020U)
#define CSL_IOMUX_GPIO65_CFG_REG_IE_OVERRIDE_SHIFT                              (0x00000005U)
#define CSL_IOMUX_GPIO65_CFG_REG_IE_OVERRIDE_RESETVAL                           (0x00000001U)
#define CSL_IOMUX_GPIO65_CFG_REG_IE_OVERRIDE_MAX                                (0x00000001U)


#define CSL_IOMUX_GPIO65_CFG_REG_IE_OVERRIDE_CTRL_MASK                          (0x00000010U)
#define CSL_IOMUX_GPIO65_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                         (0x00000004U)
#define CSL_IOMUX_GPIO65_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                      (0x00000001U)
#define CSL_IOMUX_GPIO65_CFG_REG_IE_OVERRIDE_CTRL_MAX                           (0x00000001U)


#define CSL_IOMUX_GPIO65_CFG_REG_INP_INV_SEL_MASK                               (0x00100000U)
#define CSL_IOMUX_GPIO65_CFG_REG_INP_INV_SEL_SHIFT                              (0x00000014U)
#define CSL_IOMUX_GPIO65_CFG_REG_INP_INV_SEL_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_GPIO65_CFG_REG_INP_INV_SEL_MAX                                (0x00000001U)


#define CSL_IOMUX_GPIO65_CFG_REG_OE_OVERRIDE_MASK                               (0x00000080U)
#define CSL_IOMUX_GPIO65_CFG_REG_OE_OVERRIDE_SHIFT                              (0x00000007U)
#define CSL_IOMUX_GPIO65_CFG_REG_OE_OVERRIDE_RESETVAL                           (0x00000001U)
#define CSL_IOMUX_GPIO65_CFG_REG_OE_OVERRIDE_MAX                                (0x00000001U)


#define CSL_IOMUX_GPIO65_CFG_REG_OE_OVERRIDE_CTRL_MASK                          (0x00000040U)
#define CSL_IOMUX_GPIO65_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                         (0x00000006U)
#define CSL_IOMUX_GPIO65_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                      (0x00000001U)
#define CSL_IOMUX_GPIO65_CFG_REG_OE_OVERRIDE_CTRL_MAX                           (0x00000001U)


#define CSL_IOMUX_GPIO65_CFG_REG_PE_MASK                                        (0x00000100U)
#define CSL_IOMUX_GPIO65_CFG_REG_PE_SHIFT                                       (0x00000008U)
#define CSL_IOMUX_GPIO65_CFG_REG_PE_RESETVAL                                    (0x00000001U)
#define CSL_IOMUX_GPIO65_CFG_REG_PE_MAX                                         (0x00000001U)


#define CSL_IOMUX_GPIO65_CFG_REG_PUPDSEL_MASK                                   (0x00000200U)
#define CSL_IOMUX_GPIO65_CFG_REG_PUPDSEL_SHIFT                                  (0x00000009U)
#define CSL_IOMUX_GPIO65_CFG_REG_PUPDSEL_RESETVAL                               (0x00000000U)
#define CSL_IOMUX_GPIO65_CFG_REG_PUPDSEL_MAX                                    (0x00000001U)


#define CSL_IOMUX_GPIO65_CFG_REG_QUAL_SEL_MASK                                  (0x000C0000U)
#define CSL_IOMUX_GPIO65_CFG_REG_QUAL_SEL_SHIFT                                 (0x00000012U)
#define CSL_IOMUX_GPIO65_CFG_REG_QUAL_SEL_RESETVAL                              (0x00000000U)
#define CSL_IOMUX_GPIO65_CFG_REG_QUAL_SEL_MAX                                   (0x00000003U)


#define CSL_IOMUX_GPIO65_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                       (0x00400000U)
#define CSL_IOMUX_GPIO65_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                      (0x00000016U)
#define CSL_IOMUX_GPIO65_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL                   (0x00000000U)
#define CSL_IOMUX_GPIO65_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                        (0x00000001U)


#define CSL_IOMUX_GPIO65_CFG_REG_SC1_MASK                                       (0x00000400U)
#define CSL_IOMUX_GPIO65_CFG_REG_SC1_SHIFT                                      (0x0000000AU)
#define CSL_IOMUX_GPIO65_CFG_REG_SC1_RESETVAL                                   (0x00000001U)
#define CSL_IOMUX_GPIO65_CFG_REG_SC1_MAX                                        (0x00000001U)



/* GPIO66_CFG_REG */
#define CSL_IOMUX_GPIO66_CFG_REG_FUNC_SEL_MASK                                  (0x0000000FU)
#define CSL_IOMUX_GPIO66_CFG_REG_FUNC_SEL_SHIFT                                 (0x00000000U)
#define CSL_IOMUX_GPIO66_CFG_REG_FUNC_SEL_RESETVAL                              (0x00000007U)
#define CSL_IOMUX_GPIO66_CFG_REG_FUNC_SEL_MAX                                   (0x0000000FU)


#define CSL_IOMUX_GPIO66_CFG_REG_GPIO_SEL_MASK                                  (0x00010000U)
#define CSL_IOMUX_GPIO66_CFG_REG_GPIO_SEL_SHIFT                                 (0x00000010U)
#define CSL_IOMUX_GPIO66_CFG_REG_GPIO_SEL_RESETVAL                              (0x00000000U)
#define CSL_IOMUX_GPIO66_CFG_REG_GPIO_SEL_MAX                                   (0x00000001U)


#define CSL_IOMUX_GPIO66_CFG_REG_HSMASTER_MASK                                  (0x80000000U)
#define CSL_IOMUX_GPIO66_CFG_REG_HSMASTER_SHIFT                                  (0x80000000U)
#define CSL_IOMUX_GPIO66_CFG_REG_HSMASTER_RESETVAL                              (0x00000000U)
#define CSL_IOMUX_GPIO66_CFG_REG_HSMASTER_MAX                                   (0x00000001U)


#define CSL_IOMUX_GPIO66_CFG_REG_HSMODE_MASK                                  (0x40000000U)
#define CSL_IOMUX_GPIO66_CFG_REG_HSMODE_SHIFT                                 (0x0000001EU)
#define CSL_IOMUX_GPIO66_CFG_REG_HSMODE_RESETVAL                              (0x00000000U)
#define CSL_IOMUX_GPIO66_CFG_REG_HSMODE_MAX                                   (0x00000001U)


#define CSL_IOMUX_GPIO66_CFG_REG_HVMODE_STATUS_MASK                             (0x01000000U)
#define CSL_IOMUX_GPIO66_CFG_REG_HVMODE_STATUS_SHIFT                            (0x00000018U)
#define CSL_IOMUX_GPIO66_CFG_REG_HVMODE_STATUS_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_GPIO66_CFG_REG_HVMODE_STATUS_MAX                              (0x00000001U)


#define CSL_IOMUX_GPIO66_CFG_REG_ICSSM_GPIO_SEL_MASK                            (0x00200000U)
#define CSL_IOMUX_GPIO66_CFG_REG_ICSSM_GPIO_SEL_SHIFT                           (0x00000015U)
#define CSL_IOMUX_GPIO66_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                        (0x00000000U)
#define CSL_IOMUX_GPIO66_CFG_REG_ICSSM_GPIO_SEL_MAX                             (0x00000001U)


#define CSL_IOMUX_GPIO66_CFG_REG_IE_OVERRIDE_MASK                               (0x00000020U)
#define CSL_IOMUX_GPIO66_CFG_REG_IE_OVERRIDE_SHIFT                              (0x00000005U)
#define CSL_IOMUX_GPIO66_CFG_REG_IE_OVERRIDE_RESETVAL                           (0x00000001U)
#define CSL_IOMUX_GPIO66_CFG_REG_IE_OVERRIDE_MAX                                (0x00000001U)


#define CSL_IOMUX_GPIO66_CFG_REG_IE_OVERRIDE_CTRL_MASK                          (0x00000010U)
#define CSL_IOMUX_GPIO66_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                         (0x00000004U)
#define CSL_IOMUX_GPIO66_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                      (0x00000001U)
#define CSL_IOMUX_GPIO66_CFG_REG_IE_OVERRIDE_CTRL_MAX                           (0x00000001U)


#define CSL_IOMUX_GPIO66_CFG_REG_INP_INV_SEL_MASK                               (0x00100000U)
#define CSL_IOMUX_GPIO66_CFG_REG_INP_INV_SEL_SHIFT                              (0x00000014U)
#define CSL_IOMUX_GPIO66_CFG_REG_INP_INV_SEL_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_GPIO66_CFG_REG_INP_INV_SEL_MAX                                (0x00000001U)


#define CSL_IOMUX_GPIO66_CFG_REG_OE_OVERRIDE_MASK                               (0x00000080U)
#define CSL_IOMUX_GPIO66_CFG_REG_OE_OVERRIDE_SHIFT                              (0x00000007U)
#define CSL_IOMUX_GPIO66_CFG_REG_OE_OVERRIDE_RESETVAL                           (0x00000001U)
#define CSL_IOMUX_GPIO66_CFG_REG_OE_OVERRIDE_MAX                                (0x00000001U)


#define CSL_IOMUX_GPIO66_CFG_REG_OE_OVERRIDE_CTRL_MASK                          (0x00000040U)
#define CSL_IOMUX_GPIO66_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                         (0x00000006U)
#define CSL_IOMUX_GPIO66_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                      (0x00000001U)
#define CSL_IOMUX_GPIO66_CFG_REG_OE_OVERRIDE_CTRL_MAX                           (0x00000001U)


#define CSL_IOMUX_GPIO66_CFG_REG_PE_MASK                                        (0x00000100U)
#define CSL_IOMUX_GPIO66_CFG_REG_PE_SHIFT                                       (0x00000008U)
#define CSL_IOMUX_GPIO66_CFG_REG_PE_RESETVAL                                    (0x00000001U)
#define CSL_IOMUX_GPIO66_CFG_REG_PE_MAX                                         (0x00000001U)


#define CSL_IOMUX_GPIO66_CFG_REG_PUPDSEL_MASK                                   (0x00000200U)
#define CSL_IOMUX_GPIO66_CFG_REG_PUPDSEL_SHIFT                                  (0x00000009U)
#define CSL_IOMUX_GPIO66_CFG_REG_PUPDSEL_RESETVAL                               (0x00000000U)
#define CSL_IOMUX_GPIO66_CFG_REG_PUPDSEL_MAX                                    (0x00000001U)


#define CSL_IOMUX_GPIO66_CFG_REG_QUAL_SEL_MASK                                  (0x000C0000U)
#define CSL_IOMUX_GPIO66_CFG_REG_QUAL_SEL_SHIFT                                 (0x00000012U)
#define CSL_IOMUX_GPIO66_CFG_REG_QUAL_SEL_RESETVAL                              (0x00000000U)
#define CSL_IOMUX_GPIO66_CFG_REG_QUAL_SEL_MAX                                   (0x00000003U)


#define CSL_IOMUX_GPIO66_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                       (0x00400000U)
#define CSL_IOMUX_GPIO66_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                      (0x00000016U)
#define CSL_IOMUX_GPIO66_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL                   (0x00000000U)
#define CSL_IOMUX_GPIO66_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                        (0x00000001U)


#define CSL_IOMUX_GPIO66_CFG_REG_SC1_MASK                                       (0x00000400U)
#define CSL_IOMUX_GPIO66_CFG_REG_SC1_SHIFT                                      (0x0000000AU)
#define CSL_IOMUX_GPIO66_CFG_REG_SC1_RESETVAL                                   (0x00000001U)
#define CSL_IOMUX_GPIO66_CFG_REG_SC1_MAX                                        (0x00000001U)



/* PR1_PRU0_GPO0_CFG_REG */
#define CSL_IOMUX_PR1_PRU0_GPO0_CFG_REG_FUNC_SEL_MASK                           (0x0000000FU)
#define CSL_IOMUX_PR1_PRU0_GPO0_CFG_REG_FUNC_SEL_SHIFT                          (0x00000000U)
#define CSL_IOMUX_PR1_PRU0_GPO0_CFG_REG_FUNC_SEL_RESETVAL                       (0x00000007U)
#define CSL_IOMUX_PR1_PRU0_GPO0_CFG_REG_FUNC_SEL_MAX                            (0x0000000FU)


#define CSL_IOMUX_PR1_PRU0_GPO0_CFG_REG_GPIO_SEL_MASK                           (0x00010000U)
#define CSL_IOMUX_PR1_PRU0_GPO0_CFG_REG_GPIO_SEL_SHIFT                          (0x00000010U)
#define CSL_IOMUX_PR1_PRU0_GPO0_CFG_REG_GPIO_SEL_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_PR1_PRU0_GPO0_CFG_REG_GPIO_SEL_MAX                            (0x00000001U)


#define CSL_IOMUX_PR1_PRU0_GPO0_CFG_REG_HSMASTER_MASK                           (0x80000000U)
#define CSL_IOMUX_PR1_PRU0_GPO0_CFG_REG_HSMASTER_SHIFT                          (0x0000001FU)
#define CSL_IOMUX_PR1_PRU0_GPO0_CFG_REG_HSMASTER_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_PR1_PRU0_GPO0_CFG_REG_HSMASTER_MAX                            (0x00000001U)


#define CSL_IOMUX_PR1_PRU0_GPO0_CFG_REG_HSMODE_MASK                           (0x40000000U)
#define CSL_IOMUX_PR1_PRU0_GPO0_CFG_REG_HSMODE_SHIFT                          (0x0000001EU)
#define CSL_IOMUX_PR1_PRU0_GPO0_CFG_REG_HSMODE_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_PR1_PRU0_GPO0_CFG_REG_HSMODE_MAX                            (0x00000001U)


#define CSL_IOMUX_PR1_PRU0_GPO0_CFG_REG_HVMODE_STATUS_MASK                      (0x01000000U)
#define CSL_IOMUX_PR1_PRU0_GPO0_CFG_REG_HVMODE_STATUS_SHIFT                     (0x00000018U)
#define CSL_IOMUX_PR1_PRU0_GPO0_CFG_REG_HVMODE_STATUS_RESETVAL                  (0x00000000U)
#define CSL_IOMUX_PR1_PRU0_GPO0_CFG_REG_HVMODE_STATUS_MAX                       (0x00000001U)


#define CSL_IOMUX_PR1_PRU0_GPO0_CFG_REG_ICSSM_GPIO_SEL_MASK                     (0x00200000U)
#define CSL_IOMUX_PR1_PRU0_GPO0_CFG_REG_ICSSM_GPIO_SEL_SHIFT                    (0x00000015U)
#define CSL_IOMUX_PR1_PRU0_GPO0_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                 (0x00000000U)
#define CSL_IOMUX_PR1_PRU0_GPO0_CFG_REG_ICSSM_GPIO_SEL_MAX                      (0x00000001U)


#define CSL_IOMUX_PR1_PRU0_GPO0_CFG_REG_IE_OVERRIDE_MASK                        (0x00000020U)
#define CSL_IOMUX_PR1_PRU0_GPO0_CFG_REG_IE_OVERRIDE_SHIFT                       (0x00000005U)
#define CSL_IOMUX_PR1_PRU0_GPO0_CFG_REG_IE_OVERRIDE_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_PR1_PRU0_GPO0_CFG_REG_IE_OVERRIDE_MAX                         (0x00000001U)


#define CSL_IOMUX_PR1_PRU0_GPO0_CFG_REG_IE_OVERRIDE_CTRL_MASK                   (0x00000010U)
#define CSL_IOMUX_PR1_PRU0_GPO0_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                  (0x00000004U)
#define CSL_IOMUX_PR1_PRU0_GPO0_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL               (0x00000001U)
#define CSL_IOMUX_PR1_PRU0_GPO0_CFG_REG_IE_OVERRIDE_CTRL_MAX                    (0x00000001U)


#define CSL_IOMUX_PR1_PRU0_GPO0_CFG_REG_INP_INV_SEL_MASK                        (0x00100000U)
#define CSL_IOMUX_PR1_PRU0_GPO0_CFG_REG_INP_INV_SEL_SHIFT                       (0x00000014U)
#define CSL_IOMUX_PR1_PRU0_GPO0_CFG_REG_INP_INV_SEL_RESETVAL                    (0x00000000U)
#define CSL_IOMUX_PR1_PRU0_GPO0_CFG_REG_INP_INV_SEL_MAX                         (0x00000001U)


#define CSL_IOMUX_PR1_PRU0_GPO0_CFG_REG_OE_OVERRIDE_MASK                        (0x00000080U)
#define CSL_IOMUX_PR1_PRU0_GPO0_CFG_REG_OE_OVERRIDE_SHIFT                       (0x00000007U)
#define CSL_IOMUX_PR1_PRU0_GPO0_CFG_REG_OE_OVERRIDE_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_PR1_PRU0_GPO0_CFG_REG_OE_OVERRIDE_MAX                         (0x00000001U)


#define CSL_IOMUX_PR1_PRU0_GPO0_CFG_REG_OE_OVERRIDE_CTRL_MASK                   (0x00000040U)
#define CSL_IOMUX_PR1_PRU0_GPO0_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                  (0x00000006U)
#define CSL_IOMUX_PR1_PRU0_GPO0_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL               (0x00000001U)
#define CSL_IOMUX_PR1_PRU0_GPO0_CFG_REG_OE_OVERRIDE_CTRL_MAX                    (0x00000001U)


#define CSL_IOMUX_PR1_PRU0_GPO0_CFG_REG_PE_MASK                                 (0x00000100U)
#define CSL_IOMUX_PR1_PRU0_GPO0_CFG_REG_PE_SHIFT                                (0x00000008U)
#define CSL_IOMUX_PR1_PRU0_GPO0_CFG_REG_PE_RESETVAL                             (0x00000001U)
#define CSL_IOMUX_PR1_PRU0_GPO0_CFG_REG_PE_MAX                                  (0x00000001U)


#define CSL_IOMUX_PR1_PRU0_GPO0_CFG_REG_PUPDSEL_MASK                            (0x00000200U)
#define CSL_IOMUX_PR1_PRU0_GPO0_CFG_REG_PUPDSEL_SHIFT                           (0x00000009U)
#define CSL_IOMUX_PR1_PRU0_GPO0_CFG_REG_PUPDSEL_RESETVAL                        (0x00000000U)
#define CSL_IOMUX_PR1_PRU0_GPO0_CFG_REG_PUPDSEL_MAX                             (0x00000001U)


#define CSL_IOMUX_PR1_PRU0_GPO0_CFG_REG_QUAL_SEL_MASK                           (0x000C0000U)
#define CSL_IOMUX_PR1_PRU0_GPO0_CFG_REG_QUAL_SEL_SHIFT                          (0x00000012U)
#define CSL_IOMUX_PR1_PRU0_GPO0_CFG_REG_QUAL_SEL_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_PR1_PRU0_GPO0_CFG_REG_QUAL_SEL_MAX                            (0x00000003U)


#define CSL_IOMUX_PR1_PRU0_GPO0_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                (0x00400000U)
#define CSL_IOMUX_PR1_PRU0_GPO0_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT               (0x00000016U)
#define CSL_IOMUX_PR1_PRU0_GPO0_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL            (0x00000000U)
#define CSL_IOMUX_PR1_PRU0_GPO0_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                 (0x00000001U)


#define CSL_IOMUX_PR1_PRU0_GPO0_CFG_REG_SC1_MASK                                (0x00000400U)
#define CSL_IOMUX_PR1_PRU0_GPO0_CFG_REG_SC1_SHIFT                               (0x0000000AU)
#define CSL_IOMUX_PR1_PRU0_GPO0_CFG_REG_SC1_RESETVAL                            (0x00000001U)
#define CSL_IOMUX_PR1_PRU0_GPO0_CFG_REG_SC1_MAX                                 (0x00000001U)



/* PR1_PRU0_GPO1_CFG_REG */
#define CSL_IOMUX_PR1_PRU0_GPO1_CFG_REG_FUNC_SEL_MASK                           (0x0000000FU)
#define CSL_IOMUX_PR1_PRU0_GPO1_CFG_REG_FUNC_SEL_SHIFT                          (0x00000000U)
#define CSL_IOMUX_PR1_PRU0_GPO1_CFG_REG_FUNC_SEL_RESETVAL                       (0x00000007U)
#define CSL_IOMUX_PR1_PRU0_GPO1_CFG_REG_FUNC_SEL_MAX                            (0x0000000FU)


#define CSL_IOMUX_PR1_PRU0_GPO1_CFG_REG_GPIO_SEL_MASK                           (0x00010000U)
#define CSL_IOMUX_PR1_PRU0_GPO1_CFG_REG_GPIO_SEL_SHIFT                          (0x00000010U)
#define CSL_IOMUX_PR1_PRU0_GPO1_CFG_REG_GPIO_SEL_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_PR1_PRU0_GPO1_CFG_REG_GPIO_SEL_MAX                            (0x00000001U)


#define CSL_IOMUX_PR1_PRU0_GPO1_CFG_REG_HSMASTER_MASK                           (0x80000000U)
#define CSL_IOMUX_PR1_PRU0_GPO1_CFG_REG_HSMASTER_SHIFT                          (0x0000001FU)
#define CSL_IOMUX_PR1_PRU0_GPO1_CFG_REG_HSMASTER_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_PR1_PRU0_GPO1_CFG_REG_HSMASTER_MAX                            (0x00000001U)


#define CSL_IOMUX_PR1_PRU0_GPO1_CFG_REG_HSMODE_MASK                           (0x40000000U)
#define CSL_IOMUX_PR1_PRU0_GPO1_CFG_REG_HSMODE_SHIFT                          (0x0000001EU)
#define CSL_IOMUX_PR1_PRU0_GPO1_CFG_REG_HSMODE_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_PR1_PRU0_GPO1_CFG_REG_HSMODE_MAX                            (0x00000001U)


#define CSL_IOMUX_PR1_PRU0_GPO1_CFG_REG_HVMODE_STATUS_MASK                      (0x01000000U)
#define CSL_IOMUX_PR1_PRU0_GPO1_CFG_REG_HVMODE_STATUS_SHIFT                     (0x00000018U)
#define CSL_IOMUX_PR1_PRU0_GPO1_CFG_REG_HVMODE_STATUS_RESETVAL                  (0x00000000U)
#define CSL_IOMUX_PR1_PRU0_GPO1_CFG_REG_HVMODE_STATUS_MAX                       (0x00000001U)


#define CSL_IOMUX_PR1_PRU0_GPO1_CFG_REG_ICSSM_GPIO_SEL_MASK                     (0x00200000U)
#define CSL_IOMUX_PR1_PRU0_GPO1_CFG_REG_ICSSM_GPIO_SEL_SHIFT                    (0x00000015U)
#define CSL_IOMUX_PR1_PRU0_GPO1_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                 (0x00000000U)
#define CSL_IOMUX_PR1_PRU0_GPO1_CFG_REG_ICSSM_GPIO_SEL_MAX                      (0x00000001U)


#define CSL_IOMUX_PR1_PRU0_GPO1_CFG_REG_IE_OVERRIDE_MASK                        (0x00000020U)
#define CSL_IOMUX_PR1_PRU0_GPO1_CFG_REG_IE_OVERRIDE_SHIFT                       (0x00000005U)
#define CSL_IOMUX_PR1_PRU0_GPO1_CFG_REG_IE_OVERRIDE_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_PR1_PRU0_GPO1_CFG_REG_IE_OVERRIDE_MAX                         (0x00000001U)


#define CSL_IOMUX_PR1_PRU0_GPO1_CFG_REG_IE_OVERRIDE_CTRL_MASK                   (0x00000010U)
#define CSL_IOMUX_PR1_PRU0_GPO1_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                  (0x00000004U)
#define CSL_IOMUX_PR1_PRU0_GPO1_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL               (0x00000001U)
#define CSL_IOMUX_PR1_PRU0_GPO1_CFG_REG_IE_OVERRIDE_CTRL_MAX                    (0x00000001U)


#define CSL_IOMUX_PR1_PRU0_GPO1_CFG_REG_INP_INV_SEL_MASK                        (0x00100000U)
#define CSL_IOMUX_PR1_PRU0_GPO1_CFG_REG_INP_INV_SEL_SHIFT                       (0x00000014U)
#define CSL_IOMUX_PR1_PRU0_GPO1_CFG_REG_INP_INV_SEL_RESETVAL                    (0x00000000U)
#define CSL_IOMUX_PR1_PRU0_GPO1_CFG_REG_INP_INV_SEL_MAX                         (0x00000001U)


#define CSL_IOMUX_PR1_PRU0_GPO1_CFG_REG_OE_OVERRIDE_MASK                        (0x00000080U)
#define CSL_IOMUX_PR1_PRU0_GPO1_CFG_REG_OE_OVERRIDE_SHIFT                       (0x00000007U)
#define CSL_IOMUX_PR1_PRU0_GPO1_CFG_REG_OE_OVERRIDE_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_PR1_PRU0_GPO1_CFG_REG_OE_OVERRIDE_MAX                         (0x00000001U)


#define CSL_IOMUX_PR1_PRU0_GPO1_CFG_REG_OE_OVERRIDE_CTRL_MASK                   (0x00000040U)
#define CSL_IOMUX_PR1_PRU0_GPO1_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                  (0x00000006U)
#define CSL_IOMUX_PR1_PRU0_GPO1_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL               (0x00000001U)
#define CSL_IOMUX_PR1_PRU0_GPO1_CFG_REG_OE_OVERRIDE_CTRL_MAX                    (0x00000001U)


#define CSL_IOMUX_PR1_PRU0_GPO1_CFG_REG_PE_MASK                                 (0x00000100U)
#define CSL_IOMUX_PR1_PRU0_GPO1_CFG_REG_PE_SHIFT                                (0x00000008U)
#define CSL_IOMUX_PR1_PRU0_GPO1_CFG_REG_PE_RESETVAL                             (0x00000001U)
#define CSL_IOMUX_PR1_PRU0_GPO1_CFG_REG_PE_MAX                                  (0x00000001U)


#define CSL_IOMUX_PR1_PRU0_GPO1_CFG_REG_PUPDSEL_MASK                            (0x00000200U)
#define CSL_IOMUX_PR1_PRU0_GPO1_CFG_REG_PUPDSEL_SHIFT                           (0x00000009U)
#define CSL_IOMUX_PR1_PRU0_GPO1_CFG_REG_PUPDSEL_RESETVAL                        (0x00000000U)
#define CSL_IOMUX_PR1_PRU0_GPO1_CFG_REG_PUPDSEL_MAX                             (0x00000001U)


#define CSL_IOMUX_PR1_PRU0_GPO1_CFG_REG_QUAL_SEL_MASK                           (0x000C0000U)
#define CSL_IOMUX_PR1_PRU0_GPO1_CFG_REG_QUAL_SEL_SHIFT                          (0x00000012U)
#define CSL_IOMUX_PR1_PRU0_GPO1_CFG_REG_QUAL_SEL_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_PR1_PRU0_GPO1_CFG_REG_QUAL_SEL_MAX                            (0x00000003U)


#define CSL_IOMUX_PR1_PRU0_GPO1_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                (0x00400000U)
#define CSL_IOMUX_PR1_PRU0_GPO1_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT               (0x00000016U)
#define CSL_IOMUX_PR1_PRU0_GPO1_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL            (0x00000000U)
#define CSL_IOMUX_PR1_PRU0_GPO1_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                 (0x00000001U)


#define CSL_IOMUX_PR1_PRU0_GPO1_CFG_REG_SC1_MASK                                (0x00000400U)
#define CSL_IOMUX_PR1_PRU0_GPO1_CFG_REG_SC1_SHIFT                               (0x0000000AU)
#define CSL_IOMUX_PR1_PRU0_GPO1_CFG_REG_SC1_RESETVAL                            (0x00000001U)
#define CSL_IOMUX_PR1_PRU0_GPO1_CFG_REG_SC1_MAX                                 (0x00000001U)



/* PR1_PRU0_GPO2_CFG_REG */
#define CSL_IOMUX_PR1_PRU0_GPO2_CFG_REG_FUNC_SEL_MASK                           (0x0000000FU)
#define CSL_IOMUX_PR1_PRU0_GPO2_CFG_REG_FUNC_SEL_SHIFT                          (0x00000000U)
#define CSL_IOMUX_PR1_PRU0_GPO2_CFG_REG_FUNC_SEL_RESETVAL                       (0x00000007U)
#define CSL_IOMUX_PR1_PRU0_GPO2_CFG_REG_FUNC_SEL_MAX                            (0x0000000FU)


#define CSL_IOMUX_PR1_PRU0_GPO2_CFG_REG_GPIO_SEL_MASK                           (0x00010000U)
#define CSL_IOMUX_PR1_PRU0_GPO2_CFG_REG_GPIO_SEL_SHIFT                          (0x00000010U)
#define CSL_IOMUX_PR1_PRU0_GPO2_CFG_REG_GPIO_SEL_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_PR1_PRU0_GPO2_CFG_REG_GPIO_SEL_MAX                            (0x00000001U)


#define CSL_IOMUX_PR1_PRU0_GPO2_CFG_REG_HSMASTER_MASK                           (0x80000000U)
#define CSL_IOMUX_PR1_PRU0_GPO2_CFG_REG_HSMASTER_SHIFT                          (0x0000001FU)
#define CSL_IOMUX_PR1_PRU0_GPO2_CFG_REG_HSMASTER_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_PR1_PRU0_GPO2_CFG_REG_HSMASTER_MAX                            (0x00000001U)


#define CSL_IOMUX_PR1_PRU0_GPO2_CFG_REG_HSMODE_MASK                           (0x40000000U)
#define CSL_IOMUX_PR1_PRU0_GPO2_CFG_REG_HSMODE_SHIFT                          (0x0000001EU)
#define CSL_IOMUX_PR1_PRU0_GPO2_CFG_REG_HSMODE_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_PR1_PRU0_GPO2_CFG_REG_HSMODE_MAX                            (0x00000001U)


#define CSL_IOMUX_PR1_PRU0_GPO2_CFG_REG_HVMODE_STATUS_MASK                      (0x01000000U)
#define CSL_IOMUX_PR1_PRU0_GPO2_CFG_REG_HVMODE_STATUS_SHIFT                     (0x00000018U)
#define CSL_IOMUX_PR1_PRU0_GPO2_CFG_REG_HVMODE_STATUS_RESETVAL                  (0x00000000U)
#define CSL_IOMUX_PR1_PRU0_GPO2_CFG_REG_HVMODE_STATUS_MAX                       (0x00000001U)


#define CSL_IOMUX_PR1_PRU0_GPO2_CFG_REG_ICSSM_GPIO_SEL_MASK                     (0x00200000U)
#define CSL_IOMUX_PR1_PRU0_GPO2_CFG_REG_ICSSM_GPIO_SEL_SHIFT                    (0x00000015U)
#define CSL_IOMUX_PR1_PRU0_GPO2_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                 (0x00000000U)
#define CSL_IOMUX_PR1_PRU0_GPO2_CFG_REG_ICSSM_GPIO_SEL_MAX                      (0x00000001U)


#define CSL_IOMUX_PR1_PRU0_GPO2_CFG_REG_IE_OVERRIDE_MASK                        (0x00000020U)
#define CSL_IOMUX_PR1_PRU0_GPO2_CFG_REG_IE_OVERRIDE_SHIFT                       (0x00000005U)
#define CSL_IOMUX_PR1_PRU0_GPO2_CFG_REG_IE_OVERRIDE_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_PR1_PRU0_GPO2_CFG_REG_IE_OVERRIDE_MAX                         (0x00000001U)


#define CSL_IOMUX_PR1_PRU0_GPO2_CFG_REG_IE_OVERRIDE_CTRL_MASK                   (0x00000010U)
#define CSL_IOMUX_PR1_PRU0_GPO2_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                  (0x00000004U)
#define CSL_IOMUX_PR1_PRU0_GPO2_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL               (0x00000001U)
#define CSL_IOMUX_PR1_PRU0_GPO2_CFG_REG_IE_OVERRIDE_CTRL_MAX                    (0x00000001U)


#define CSL_IOMUX_PR1_PRU0_GPO2_CFG_REG_INP_INV_SEL_MASK                        (0x00100000U)
#define CSL_IOMUX_PR1_PRU0_GPO2_CFG_REG_INP_INV_SEL_SHIFT                       (0x00000014U)
#define CSL_IOMUX_PR1_PRU0_GPO2_CFG_REG_INP_INV_SEL_RESETVAL                    (0x00000000U)
#define CSL_IOMUX_PR1_PRU0_GPO2_CFG_REG_INP_INV_SEL_MAX                         (0x00000001U)


#define CSL_IOMUX_PR1_PRU0_GPO2_CFG_REG_OE_OVERRIDE_MASK                        (0x00000080U)
#define CSL_IOMUX_PR1_PRU0_GPO2_CFG_REG_OE_OVERRIDE_SHIFT                       (0x00000007U)
#define CSL_IOMUX_PR1_PRU0_GPO2_CFG_REG_OE_OVERRIDE_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_PR1_PRU0_GPO2_CFG_REG_OE_OVERRIDE_MAX                         (0x00000001U)


#define CSL_IOMUX_PR1_PRU0_GPO2_CFG_REG_OE_OVERRIDE_CTRL_MASK                   (0x00000040U)
#define CSL_IOMUX_PR1_PRU0_GPO2_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                  (0x00000006U)
#define CSL_IOMUX_PR1_PRU0_GPO2_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL               (0x00000001U)
#define CSL_IOMUX_PR1_PRU0_GPO2_CFG_REG_OE_OVERRIDE_CTRL_MAX                    (0x00000001U)


#define CSL_IOMUX_PR1_PRU0_GPO2_CFG_REG_PE_MASK                                 (0x00000100U)
#define CSL_IOMUX_PR1_PRU0_GPO2_CFG_REG_PE_SHIFT                                (0x00000008U)
#define CSL_IOMUX_PR1_PRU0_GPO2_CFG_REG_PE_RESETVAL                             (0x00000001U)
#define CSL_IOMUX_PR1_PRU0_GPO2_CFG_REG_PE_MAX                                  (0x00000001U)


#define CSL_IOMUX_PR1_PRU0_GPO2_CFG_REG_PUPDSEL_MASK                            (0x00000200U)
#define CSL_IOMUX_PR1_PRU0_GPO2_CFG_REG_PUPDSEL_SHIFT                           (0x00000009U)
#define CSL_IOMUX_PR1_PRU0_GPO2_CFG_REG_PUPDSEL_RESETVAL                        (0x00000000U)
#define CSL_IOMUX_PR1_PRU0_GPO2_CFG_REG_PUPDSEL_MAX                             (0x00000001U)


#define CSL_IOMUX_PR1_PRU0_GPO2_CFG_REG_QUAL_SEL_MASK                           (0x000C0000U)
#define CSL_IOMUX_PR1_PRU0_GPO2_CFG_REG_QUAL_SEL_SHIFT                          (0x00000012U)
#define CSL_IOMUX_PR1_PRU0_GPO2_CFG_REG_QUAL_SEL_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_PR1_PRU0_GPO2_CFG_REG_QUAL_SEL_MAX                            (0x00000003U)


#define CSL_IOMUX_PR1_PRU0_GPO2_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                (0x00400000U)
#define CSL_IOMUX_PR1_PRU0_GPO2_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT               (0x00000016U)
#define CSL_IOMUX_PR1_PRU0_GPO2_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL            (0x00000000U)
#define CSL_IOMUX_PR1_PRU0_GPO2_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                 (0x00000001U)


#define CSL_IOMUX_PR1_PRU0_GPO2_CFG_REG_SC1_MASK                                (0x00000400U)
#define CSL_IOMUX_PR1_PRU0_GPO2_CFG_REG_SC1_SHIFT                               (0x0000000AU)
#define CSL_IOMUX_PR1_PRU0_GPO2_CFG_REG_SC1_RESETVAL                            (0x00000001U)
#define CSL_IOMUX_PR1_PRU0_GPO2_CFG_REG_SC1_MAX                                 (0x00000001U)



/* PR1_PRU0_GPO9_CFG_REG */
#define CSL_IOMUX_PR1_PRU0_GPO9_CFG_REG_FUNC_SEL_MASK                           (0x0000000FU)
#define CSL_IOMUX_PR1_PRU0_GPO9_CFG_REG_FUNC_SEL_SHIFT                          (0x00000000U)
#define CSL_IOMUX_PR1_PRU0_GPO9_CFG_REG_FUNC_SEL_RESETVAL                       (0x00000007U)
#define CSL_IOMUX_PR1_PRU0_GPO9_CFG_REG_FUNC_SEL_MAX                            (0x0000000FU)


#define CSL_IOMUX_PR1_PRU0_GPO9_CFG_REG_GPIO_SEL_MASK                           (0x00010000U)
#define CSL_IOMUX_PR1_PRU0_GPO9_CFG_REG_GPIO_SEL_SHIFT                          (0x00000010U)
#define CSL_IOMUX_PR1_PRU0_GPO9_CFG_REG_GPIO_SEL_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_PR1_PRU0_GPO9_CFG_REG_GPIO_SEL_MAX                            (0x00000001U)


#define CSL_IOMUX_PR1_PRU0_GPO9_CFG_REG_HSMASTER_MASK                           (0x80000000U)
#define CSL_IOMUX_PR1_PRU0_GPO9_CFG_REG_HSMASTER_SHIFT                          (0x0000001FU)
#define CSL_IOMUX_PR1_PRU0_GPO9_CFG_REG_HSMASTER_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_PR1_PRU0_GPO9_CFG_REG_HSMASTER_MAX                            (0x00000001U)


#define CSL_IOMUX_PR1_PRU0_GPO9_CFG_REG_HSMODE_MASK                           (0x40000000U)
#define CSL_IOMUX_PR1_PRU0_GPO9_CFG_REG_HSMODE_SHIFT                          (0x0000001EU)
#define CSL_IOMUX_PR1_PRU0_GPO9_CFG_REG_HSMODE_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_PR1_PRU0_GPO9_CFG_REG_HSMODE_MAX                            (0x00000001U)


#define CSL_IOMUX_PR1_PRU0_GPO9_CFG_REG_HVMODE_STATUS_MASK                      (0x01000000U)
#define CSL_IOMUX_PR1_PRU0_GPO9_CFG_REG_HVMODE_STATUS_SHIFT                     (0x00000018U)
#define CSL_IOMUX_PR1_PRU0_GPO9_CFG_REG_HVMODE_STATUS_RESETVAL                  (0x00000000U)
#define CSL_IOMUX_PR1_PRU0_GPO9_CFG_REG_HVMODE_STATUS_MAX                       (0x00000001U)


#define CSL_IOMUX_PR1_PRU0_GPO9_CFG_REG_ICSSM_GPIO_SEL_MASK                     (0x00200000U)
#define CSL_IOMUX_PR1_PRU0_GPO9_CFG_REG_ICSSM_GPIO_SEL_SHIFT                    (0x00000015U)
#define CSL_IOMUX_PR1_PRU0_GPO9_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                 (0x00000000U)
#define CSL_IOMUX_PR1_PRU0_GPO9_CFG_REG_ICSSM_GPIO_SEL_MAX                      (0x00000001U)


#define CSL_IOMUX_PR1_PRU0_GPO9_CFG_REG_IE_OVERRIDE_MASK                        (0x00000020U)
#define CSL_IOMUX_PR1_PRU0_GPO9_CFG_REG_IE_OVERRIDE_SHIFT                       (0x00000005U)
#define CSL_IOMUX_PR1_PRU0_GPO9_CFG_REG_IE_OVERRIDE_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_PR1_PRU0_GPO9_CFG_REG_IE_OVERRIDE_MAX                         (0x00000001U)


#define CSL_IOMUX_PR1_PRU0_GPO9_CFG_REG_IE_OVERRIDE_CTRL_MASK                   (0x00000010U)
#define CSL_IOMUX_PR1_PRU0_GPO9_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                  (0x00000004U)
#define CSL_IOMUX_PR1_PRU0_GPO9_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL               (0x00000001U)
#define CSL_IOMUX_PR1_PRU0_GPO9_CFG_REG_IE_OVERRIDE_CTRL_MAX                    (0x00000001U)


#define CSL_IOMUX_PR1_PRU0_GPO9_CFG_REG_INP_INV_SEL_MASK                        (0x00100000U)
#define CSL_IOMUX_PR1_PRU0_GPO9_CFG_REG_INP_INV_SEL_SHIFT                       (0x00000014U)
#define CSL_IOMUX_PR1_PRU0_GPO9_CFG_REG_INP_INV_SEL_RESETVAL                    (0x00000000U)
#define CSL_IOMUX_PR1_PRU0_GPO9_CFG_REG_INP_INV_SEL_MAX                         (0x00000001U)


#define CSL_IOMUX_PR1_PRU0_GPO9_CFG_REG_OE_OVERRIDE_MASK                        (0x00000080U)
#define CSL_IOMUX_PR1_PRU0_GPO9_CFG_REG_OE_OVERRIDE_SHIFT                       (0x00000007U)
#define CSL_IOMUX_PR1_PRU0_GPO9_CFG_REG_OE_OVERRIDE_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_PR1_PRU0_GPO9_CFG_REG_OE_OVERRIDE_MAX                         (0x00000001U)


#define CSL_IOMUX_PR1_PRU0_GPO9_CFG_REG_OE_OVERRIDE_CTRL_MASK                   (0x00000040U)
#define CSL_IOMUX_PR1_PRU0_GPO9_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                  (0x00000006U)
#define CSL_IOMUX_PR1_PRU0_GPO9_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL               (0x00000001U)
#define CSL_IOMUX_PR1_PRU0_GPO9_CFG_REG_OE_OVERRIDE_CTRL_MAX                    (0x00000001U)


#define CSL_IOMUX_PR1_PRU0_GPO9_CFG_REG_PE_MASK                                 (0x00000100U)
#define CSL_IOMUX_PR1_PRU0_GPO9_CFG_REG_PE_SHIFT                                (0x00000008U)
#define CSL_IOMUX_PR1_PRU0_GPO9_CFG_REG_PE_RESETVAL                             (0x00000001U)
#define CSL_IOMUX_PR1_PRU0_GPO9_CFG_REG_PE_MAX                                  (0x00000001U)


#define CSL_IOMUX_PR1_PRU0_GPO9_CFG_REG_PUPDSEL_MASK                            (0x00000200U)
#define CSL_IOMUX_PR1_PRU0_GPO9_CFG_REG_PUPDSEL_SHIFT                           (0x00000009U)
#define CSL_IOMUX_PR1_PRU0_GPO9_CFG_REG_PUPDSEL_RESETVAL                        (0x00000000U)
#define CSL_IOMUX_PR1_PRU0_GPO9_CFG_REG_PUPDSEL_MAX                             (0x00000001U)


#define CSL_IOMUX_PR1_PRU0_GPO9_CFG_REG_QUAL_SEL_MASK                           (0x000C0000U)
#define CSL_IOMUX_PR1_PRU0_GPO9_CFG_REG_QUAL_SEL_SHIFT                          (0x00000012U)
#define CSL_IOMUX_PR1_PRU0_GPO9_CFG_REG_QUAL_SEL_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_PR1_PRU0_GPO9_CFG_REG_QUAL_SEL_MAX                            (0x00000003U)


#define CSL_IOMUX_PR1_PRU0_GPO9_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                (0x00400000U)
#define CSL_IOMUX_PR1_PRU0_GPO9_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT               (0x00000016U)
#define CSL_IOMUX_PR1_PRU0_GPO9_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL            (0x00000000U)
#define CSL_IOMUX_PR1_PRU0_GPO9_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                 (0x00000001U)


#define CSL_IOMUX_PR1_PRU0_GPO9_CFG_REG_SC1_MASK                                (0x00000400U)
#define CSL_IOMUX_PR1_PRU0_GPO9_CFG_REG_SC1_SHIFT                               (0x0000000AU)
#define CSL_IOMUX_PR1_PRU0_GPO9_CFG_REG_SC1_RESETVAL                            (0x00000001U)
#define CSL_IOMUX_PR1_PRU0_GPO9_CFG_REG_SC1_MAX                                 (0x00000001U)



/* PR1_PRU1_GPO0_CFG_REG */
#define CSL_IOMUX_PR1_PRU1_GPO0_CFG_REG_FUNC_SEL_MASK                           (0x0000000FU)
#define CSL_IOMUX_PR1_PRU1_GPO0_CFG_REG_FUNC_SEL_SHIFT                          (0x00000000U)
#define CSL_IOMUX_PR1_PRU1_GPO0_CFG_REG_FUNC_SEL_RESETVAL                       (0x00000007U)
#define CSL_IOMUX_PR1_PRU1_GPO0_CFG_REG_FUNC_SEL_MAX                            (0x0000000FU)


#define CSL_IOMUX_PR1_PRU1_GPO0_CFG_REG_GPIO_SEL_MASK                           (0x00010000U)
#define CSL_IOMUX_PR1_PRU1_GPO0_CFG_REG_GPIO_SEL_SHIFT                          (0x00000010U)
#define CSL_IOMUX_PR1_PRU1_GPO0_CFG_REG_GPIO_SEL_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_PR1_PRU1_GPO0_CFG_REG_GPIO_SEL_MAX                            (0x00000001U)


#define CSL_IOMUX_PR1_PRU1_GPO0_CFG_REG_HSMASTER_MASK                           (0x80000000U)
#define CSL_IOMUX_PR1_PRU1_GPO0_CFG_REG_HSMASTER_SHIFT                          (0x0000001FU)
#define CSL_IOMUX_PR1_PRU1_GPO0_CFG_REG_HSMASTER_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_PR1_PRU1_GPO0_CFG_REG_HSMASTER_MAX                            (0x00000001U)


#define CSL_IOMUX_PR1_PRU1_GPO0_CFG_REG_HSMODE_MASK                           (0x40000000U)
#define CSL_IOMUX_PR1_PRU1_GPO0_CFG_REG_HSMODE_SHIFT                          (0x0000001EU)
#define CSL_IOMUX_PR1_PRU1_GPO0_CFG_REG_HSMODE_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_PR1_PRU1_GPO0_CFG_REG_HSMODE_MAX                            (0x00000001U)


#define CSL_IOMUX_PR1_PRU1_GPO0_CFG_REG_HVMODE_STATUS_MASK                      (0x01000000U)
#define CSL_IOMUX_PR1_PRU1_GPO0_CFG_REG_HVMODE_STATUS_SHIFT                     (0x00000018U)
#define CSL_IOMUX_PR1_PRU1_GPO0_CFG_REG_HVMODE_STATUS_RESETVAL                  (0x00000000U)
#define CSL_IOMUX_PR1_PRU1_GPO0_CFG_REG_HVMODE_STATUS_MAX                       (0x00000001U)


#define CSL_IOMUX_PR1_PRU1_GPO0_CFG_REG_ICSSM_GPIO_SEL_MASK                     (0x00200000U)
#define CSL_IOMUX_PR1_PRU1_GPO0_CFG_REG_ICSSM_GPIO_SEL_SHIFT                    (0x00000015U)
#define CSL_IOMUX_PR1_PRU1_GPO0_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                 (0x00000000U)
#define CSL_IOMUX_PR1_PRU1_GPO0_CFG_REG_ICSSM_GPIO_SEL_MAX                      (0x00000001U)


#define CSL_IOMUX_PR1_PRU1_GPO0_CFG_REG_IE_OVERRIDE_MASK                        (0x00000020U)
#define CSL_IOMUX_PR1_PRU1_GPO0_CFG_REG_IE_OVERRIDE_SHIFT                       (0x00000005U)
#define CSL_IOMUX_PR1_PRU1_GPO0_CFG_REG_IE_OVERRIDE_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_PR1_PRU1_GPO0_CFG_REG_IE_OVERRIDE_MAX                         (0x00000001U)


#define CSL_IOMUX_PR1_PRU1_GPO0_CFG_REG_IE_OVERRIDE_CTRL_MASK                   (0x00000010U)
#define CSL_IOMUX_PR1_PRU1_GPO0_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                  (0x00000004U)
#define CSL_IOMUX_PR1_PRU1_GPO0_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL               (0x00000001U)
#define CSL_IOMUX_PR1_PRU1_GPO0_CFG_REG_IE_OVERRIDE_CTRL_MAX                    (0x00000001U)


#define CSL_IOMUX_PR1_PRU1_GPO0_CFG_REG_INP_INV_SEL_MASK                        (0x00100000U)
#define CSL_IOMUX_PR1_PRU1_GPO0_CFG_REG_INP_INV_SEL_SHIFT                       (0x00000014U)
#define CSL_IOMUX_PR1_PRU1_GPO0_CFG_REG_INP_INV_SEL_RESETVAL                    (0x00000000U)
#define CSL_IOMUX_PR1_PRU1_GPO0_CFG_REG_INP_INV_SEL_MAX                         (0x00000001U)


#define CSL_IOMUX_PR1_PRU1_GPO0_CFG_REG_OE_OVERRIDE_MASK                        (0x00000080U)
#define CSL_IOMUX_PR1_PRU1_GPO0_CFG_REG_OE_OVERRIDE_SHIFT                       (0x00000007U)
#define CSL_IOMUX_PR1_PRU1_GPO0_CFG_REG_OE_OVERRIDE_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_PR1_PRU1_GPO0_CFG_REG_OE_OVERRIDE_MAX                         (0x00000001U)


#define CSL_IOMUX_PR1_PRU1_GPO0_CFG_REG_OE_OVERRIDE_CTRL_MASK                   (0x00000040U)
#define CSL_IOMUX_PR1_PRU1_GPO0_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                  (0x00000006U)
#define CSL_IOMUX_PR1_PRU1_GPO0_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL               (0x00000001U)
#define CSL_IOMUX_PR1_PRU1_GPO0_CFG_REG_OE_OVERRIDE_CTRL_MAX                    (0x00000001U)


#define CSL_IOMUX_PR1_PRU1_GPO0_CFG_REG_PE_MASK                                 (0x00000100U)
#define CSL_IOMUX_PR1_PRU1_GPO0_CFG_REG_PE_SHIFT                                (0x00000008U)
#define CSL_IOMUX_PR1_PRU1_GPO0_CFG_REG_PE_RESETVAL                             (0x00000001U)
#define CSL_IOMUX_PR1_PRU1_GPO0_CFG_REG_PE_MAX                                  (0x00000001U)


#define CSL_IOMUX_PR1_PRU1_GPO0_CFG_REG_PUPDSEL_MASK                            (0x00000200U)
#define CSL_IOMUX_PR1_PRU1_GPO0_CFG_REG_PUPDSEL_SHIFT                           (0x00000009U)
#define CSL_IOMUX_PR1_PRU1_GPO0_CFG_REG_PUPDSEL_RESETVAL                        (0x00000000U)
#define CSL_IOMUX_PR1_PRU1_GPO0_CFG_REG_PUPDSEL_MAX                             (0x00000001U)


#define CSL_IOMUX_PR1_PRU1_GPO0_CFG_REG_QUAL_SEL_MASK                           (0x000C0000U)
#define CSL_IOMUX_PR1_PRU1_GPO0_CFG_REG_QUAL_SEL_SHIFT                          (0x00000012U)
#define CSL_IOMUX_PR1_PRU1_GPO0_CFG_REG_QUAL_SEL_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_PR1_PRU1_GPO0_CFG_REG_QUAL_SEL_MAX                            (0x00000003U)


#define CSL_IOMUX_PR1_PRU1_GPO0_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                (0x00400000U)
#define CSL_IOMUX_PR1_PRU1_GPO0_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT               (0x00000016U)
#define CSL_IOMUX_PR1_PRU1_GPO0_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL            (0x00000000U)
#define CSL_IOMUX_PR1_PRU1_GPO0_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                 (0x00000001U)


#define CSL_IOMUX_PR1_PRU1_GPO0_CFG_REG_SC1_MASK                                (0x00000400U)
#define CSL_IOMUX_PR1_PRU1_GPO0_CFG_REG_SC1_SHIFT                               (0x0000000AU)
#define CSL_IOMUX_PR1_PRU1_GPO0_CFG_REG_SC1_RESETVAL                            (0x00000001U)
#define CSL_IOMUX_PR1_PRU1_GPO0_CFG_REG_SC1_MAX                                 (0x00000001U)



/* PR1_PRU1_GPO1_CFG_REG */
#define CSL_IOMUX_PR1_PRU1_GPO1_CFG_REG_FUNC_SEL_MASK                           (0x0000000FU)
#define CSL_IOMUX_PR1_PRU1_GPO1_CFG_REG_FUNC_SEL_SHIFT                          (0x00000000U)
#define CSL_IOMUX_PR1_PRU1_GPO1_CFG_REG_FUNC_SEL_RESETVAL                       (0x00000007U)
#define CSL_IOMUX_PR1_PRU1_GPO1_CFG_REG_FUNC_SEL_MAX                            (0x0000000FU)


#define CSL_IOMUX_PR1_PRU1_GPO1_CFG_REG_GPIO_SEL_MASK                           (0x00010000U)
#define CSL_IOMUX_PR1_PRU1_GPO1_CFG_REG_GPIO_SEL_SHIFT                          (0x00000010U)
#define CSL_IOMUX_PR1_PRU1_GPO1_CFG_REG_GPIO_SEL_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_PR1_PRU1_GPO1_CFG_REG_GPIO_SEL_MAX                            (0x00000001U)


#define CSL_IOMUX_PR1_PRU1_GPO1_CFG_REG_HSMASTER_MASK                           (0x80000000U)
#define CSL_IOMUX_PR1_PRU1_GPO1_CFG_REG_HSMASTER_SHIFT                          (0x0000001FU)
#define CSL_IOMUX_PR1_PRU1_GPO1_CFG_REG_HSMASTER_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_PR1_PRU1_GPO1_CFG_REG_HSMASTER_MAX                            (0x00000001U)


#define CSL_IOMUX_PR1_PRU1_GPO1_CFG_REG_HSMODE_MASK                           (0x40000000U)
#define CSL_IOMUX_PR1_PRU1_GPO1_CFG_REG_HSMODE_SHIFT                          (0x0000001EU)
#define CSL_IOMUX_PR1_PRU1_GPO1_CFG_REG_HSMODE_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_PR1_PRU1_GPO1_CFG_REG_HSMODE_MAX                            (0x00000001U)


#define CSL_IOMUX_PR1_PRU1_GPO1_CFG_REG_HVMODE_STATUS_MASK                      (0x01000000U)
#define CSL_IOMUX_PR1_PRU1_GPO1_CFG_REG_HVMODE_STATUS_SHIFT                     (0x00000018U)
#define CSL_IOMUX_PR1_PRU1_GPO1_CFG_REG_HVMODE_STATUS_RESETVAL                  (0x00000000U)
#define CSL_IOMUX_PR1_PRU1_GPO1_CFG_REG_HVMODE_STATUS_MAX                       (0x00000001U)


#define CSL_IOMUX_PR1_PRU1_GPO1_CFG_REG_ICSSM_GPIO_SEL_MASK                     (0x00200000U)
#define CSL_IOMUX_PR1_PRU1_GPO1_CFG_REG_ICSSM_GPIO_SEL_SHIFT                    (0x00000015U)
#define CSL_IOMUX_PR1_PRU1_GPO1_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                 (0x00000000U)
#define CSL_IOMUX_PR1_PRU1_GPO1_CFG_REG_ICSSM_GPIO_SEL_MAX                      (0x00000001U)


#define CSL_IOMUX_PR1_PRU1_GPO1_CFG_REG_IE_OVERRIDE_MASK                        (0x00000020U)
#define CSL_IOMUX_PR1_PRU1_GPO1_CFG_REG_IE_OVERRIDE_SHIFT                       (0x00000005U)
#define CSL_IOMUX_PR1_PRU1_GPO1_CFG_REG_IE_OVERRIDE_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_PR1_PRU1_GPO1_CFG_REG_IE_OVERRIDE_MAX                         (0x00000001U)


#define CSL_IOMUX_PR1_PRU1_GPO1_CFG_REG_IE_OVERRIDE_CTRL_MASK                   (0x00000010U)
#define CSL_IOMUX_PR1_PRU1_GPO1_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                  (0x00000004U)
#define CSL_IOMUX_PR1_PRU1_GPO1_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL               (0x00000001U)
#define CSL_IOMUX_PR1_PRU1_GPO1_CFG_REG_IE_OVERRIDE_CTRL_MAX                    (0x00000001U)


#define CSL_IOMUX_PR1_PRU1_GPO1_CFG_REG_INP_INV_SEL_MASK                        (0x00100000U)
#define CSL_IOMUX_PR1_PRU1_GPO1_CFG_REG_INP_INV_SEL_SHIFT                       (0x00000014U)
#define CSL_IOMUX_PR1_PRU1_GPO1_CFG_REG_INP_INV_SEL_RESETVAL                    (0x00000000U)
#define CSL_IOMUX_PR1_PRU1_GPO1_CFG_REG_INP_INV_SEL_MAX                         (0x00000001U)


#define CSL_IOMUX_PR1_PRU1_GPO1_CFG_REG_OE_OVERRIDE_MASK                        (0x00000080U)
#define CSL_IOMUX_PR1_PRU1_GPO1_CFG_REG_OE_OVERRIDE_SHIFT                       (0x00000007U)
#define CSL_IOMUX_PR1_PRU1_GPO1_CFG_REG_OE_OVERRIDE_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_PR1_PRU1_GPO1_CFG_REG_OE_OVERRIDE_MAX                         (0x00000001U)


#define CSL_IOMUX_PR1_PRU1_GPO1_CFG_REG_OE_OVERRIDE_CTRL_MASK                   (0x00000040U)
#define CSL_IOMUX_PR1_PRU1_GPO1_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                  (0x00000006U)
#define CSL_IOMUX_PR1_PRU1_GPO1_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL               (0x00000001U)
#define CSL_IOMUX_PR1_PRU1_GPO1_CFG_REG_OE_OVERRIDE_CTRL_MAX                    (0x00000001U)


#define CSL_IOMUX_PR1_PRU1_GPO1_CFG_REG_PE_MASK                                 (0x00000100U)
#define CSL_IOMUX_PR1_PRU1_GPO1_CFG_REG_PE_SHIFT                                (0x00000008U)
#define CSL_IOMUX_PR1_PRU1_GPO1_CFG_REG_PE_RESETVAL                             (0x00000001U)
#define CSL_IOMUX_PR1_PRU1_GPO1_CFG_REG_PE_MAX                                  (0x00000001U)


#define CSL_IOMUX_PR1_PRU1_GPO1_CFG_REG_PUPDSEL_MASK                            (0x00000200U)
#define CSL_IOMUX_PR1_PRU1_GPO1_CFG_REG_PUPDSEL_SHIFT                           (0x00000009U)
#define CSL_IOMUX_PR1_PRU1_GPO1_CFG_REG_PUPDSEL_RESETVAL                        (0x00000000U)
#define CSL_IOMUX_PR1_PRU1_GPO1_CFG_REG_PUPDSEL_MAX                             (0x00000001U)


#define CSL_IOMUX_PR1_PRU1_GPO1_CFG_REG_QUAL_SEL_MASK                           (0x000C0000U)
#define CSL_IOMUX_PR1_PRU1_GPO1_CFG_REG_QUAL_SEL_SHIFT                          (0x00000012U)
#define CSL_IOMUX_PR1_PRU1_GPO1_CFG_REG_QUAL_SEL_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_PR1_PRU1_GPO1_CFG_REG_QUAL_SEL_MAX                            (0x00000003U)


#define CSL_IOMUX_PR1_PRU1_GPO1_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                (0x00400000U)
#define CSL_IOMUX_PR1_PRU1_GPO1_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT               (0x00000016U)
#define CSL_IOMUX_PR1_PRU1_GPO1_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL            (0x00000000U)
#define CSL_IOMUX_PR1_PRU1_GPO1_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                 (0x00000001U)


#define CSL_IOMUX_PR1_PRU1_GPO1_CFG_REG_SC1_MASK                                (0x00000400U)
#define CSL_IOMUX_PR1_PRU1_GPO1_CFG_REG_SC1_SHIFT                               (0x0000000AU)
#define CSL_IOMUX_PR1_PRU1_GPO1_CFG_REG_SC1_RESETVAL                            (0x00000001U)
#define CSL_IOMUX_PR1_PRU1_GPO1_CFG_REG_SC1_MAX                                 (0x00000001U)



/* PR1_PRU1_GPO2_CFG_REG */
#define CSL_IOMUX_PR1_PRU1_GPO2_CFG_REG_FUNC_SEL_MASK                           (0x0000000FU)
#define CSL_IOMUX_PR1_PRU1_GPO2_CFG_REG_FUNC_SEL_SHIFT                          (0x00000000U)
#define CSL_IOMUX_PR1_PRU1_GPO2_CFG_REG_FUNC_SEL_RESETVAL                       (0x00000007U)
#define CSL_IOMUX_PR1_PRU1_GPO2_CFG_REG_FUNC_SEL_MAX                            (0x0000000FU)


#define CSL_IOMUX_PR1_PRU1_GPO2_CFG_REG_GPIO_SEL_MASK                           (0x00010000U)
#define CSL_IOMUX_PR1_PRU1_GPO2_CFG_REG_GPIO_SEL_SHIFT                          (0x00000010U)
#define CSL_IOMUX_PR1_PRU1_GPO2_CFG_REG_GPIO_SEL_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_PR1_PRU1_GPO2_CFG_REG_GPIO_SEL_MAX                            (0x00000001U)


#define CSL_IOMUX_PR1_PRU1_GPO2_CFG_REG_HSMASTER_MASK                           (0x80000000U)
#define CSL_IOMUX_PR1_PRU1_GPO2_CFG_REG_HSMASTER_SHIFT                          (0x0000001FU)
#define CSL_IOMUX_PR1_PRU1_GPO2_CFG_REG_HSMASTER_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_PR1_PRU1_GPO2_CFG_REG_HSMASTER_MAX                            (0x00000001U)


#define CSL_IOMUX_PR1_PRU1_GPO2_CFG_REG_HSMODE_MASK                           (0x40000000U)
#define CSL_IOMUX_PR1_PRU1_GPO2_CFG_REG_HSMODE_SHIFT                          (0x0000001EU)
#define CSL_IOMUX_PR1_PRU1_GPO2_CFG_REG_HSMODE_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_PR1_PRU1_GPO2_CFG_REG_HSMODE_MAX                            (0x00000001U)


#define CSL_IOMUX_PR1_PRU1_GPO2_CFG_REG_HVMODE_STATUS_MASK                      (0x01000000U)
#define CSL_IOMUX_PR1_PRU1_GPO2_CFG_REG_HVMODE_STATUS_SHIFT                     (0x00000018U)
#define CSL_IOMUX_PR1_PRU1_GPO2_CFG_REG_HVMODE_STATUS_RESETVAL                  (0x00000000U)
#define CSL_IOMUX_PR1_PRU1_GPO2_CFG_REG_HVMODE_STATUS_MAX                       (0x00000001U)


#define CSL_IOMUX_PR1_PRU1_GPO2_CFG_REG_ICSSM_GPIO_SEL_MASK                     (0x00200000U)
#define CSL_IOMUX_PR1_PRU1_GPO2_CFG_REG_ICSSM_GPIO_SEL_SHIFT                    (0x00000015U)
#define CSL_IOMUX_PR1_PRU1_GPO2_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                 (0x00000000U)
#define CSL_IOMUX_PR1_PRU1_GPO2_CFG_REG_ICSSM_GPIO_SEL_MAX                      (0x00000001U)


#define CSL_IOMUX_PR1_PRU1_GPO2_CFG_REG_IE_OVERRIDE_MASK                        (0x00000020U)
#define CSL_IOMUX_PR1_PRU1_GPO2_CFG_REG_IE_OVERRIDE_SHIFT                       (0x00000005U)
#define CSL_IOMUX_PR1_PRU1_GPO2_CFG_REG_IE_OVERRIDE_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_PR1_PRU1_GPO2_CFG_REG_IE_OVERRIDE_MAX                         (0x00000001U)


#define CSL_IOMUX_PR1_PRU1_GPO2_CFG_REG_IE_OVERRIDE_CTRL_MASK                   (0x00000010U)
#define CSL_IOMUX_PR1_PRU1_GPO2_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                  (0x00000004U)
#define CSL_IOMUX_PR1_PRU1_GPO2_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL               (0x00000001U)
#define CSL_IOMUX_PR1_PRU1_GPO2_CFG_REG_IE_OVERRIDE_CTRL_MAX                    (0x00000001U)


#define CSL_IOMUX_PR1_PRU1_GPO2_CFG_REG_INP_INV_SEL_MASK                        (0x00100000U)
#define CSL_IOMUX_PR1_PRU1_GPO2_CFG_REG_INP_INV_SEL_SHIFT                       (0x00000014U)
#define CSL_IOMUX_PR1_PRU1_GPO2_CFG_REG_INP_INV_SEL_RESETVAL                    (0x00000000U)
#define CSL_IOMUX_PR1_PRU1_GPO2_CFG_REG_INP_INV_SEL_MAX                         (0x00000001U)


#define CSL_IOMUX_PR1_PRU1_GPO2_CFG_REG_OE_OVERRIDE_MASK                        (0x00000080U)
#define CSL_IOMUX_PR1_PRU1_GPO2_CFG_REG_OE_OVERRIDE_SHIFT                       (0x00000007U)
#define CSL_IOMUX_PR1_PRU1_GPO2_CFG_REG_OE_OVERRIDE_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_PR1_PRU1_GPO2_CFG_REG_OE_OVERRIDE_MAX                         (0x00000001U)


#define CSL_IOMUX_PR1_PRU1_GPO2_CFG_REG_OE_OVERRIDE_CTRL_MASK                   (0x00000040U)
#define CSL_IOMUX_PR1_PRU1_GPO2_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                  (0x00000006U)
#define CSL_IOMUX_PR1_PRU1_GPO2_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL               (0x00000001U)
#define CSL_IOMUX_PR1_PRU1_GPO2_CFG_REG_OE_OVERRIDE_CTRL_MAX                    (0x00000001U)


#define CSL_IOMUX_PR1_PRU1_GPO2_CFG_REG_PE_MASK                                 (0x00000100U)
#define CSL_IOMUX_PR1_PRU1_GPO2_CFG_REG_PE_SHIFT                                (0x00000008U)
#define CSL_IOMUX_PR1_PRU1_GPO2_CFG_REG_PE_RESETVAL                             (0x00000001U)
#define CSL_IOMUX_PR1_PRU1_GPO2_CFG_REG_PE_MAX                                  (0x00000001U)


#define CSL_IOMUX_PR1_PRU1_GPO2_CFG_REG_PUPDSEL_MASK                            (0x00000200U)
#define CSL_IOMUX_PR1_PRU1_GPO2_CFG_REG_PUPDSEL_SHIFT                           (0x00000009U)
#define CSL_IOMUX_PR1_PRU1_GPO2_CFG_REG_PUPDSEL_RESETVAL                        (0x00000000U)
#define CSL_IOMUX_PR1_PRU1_GPO2_CFG_REG_PUPDSEL_MAX                             (0x00000001U)


#define CSL_IOMUX_PR1_PRU1_GPO2_CFG_REG_QUAL_SEL_MASK                           (0x000C0000U)
#define CSL_IOMUX_PR1_PRU1_GPO2_CFG_REG_QUAL_SEL_SHIFT                          (0x00000012U)
#define CSL_IOMUX_PR1_PRU1_GPO2_CFG_REG_QUAL_SEL_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_PR1_PRU1_GPO2_CFG_REG_QUAL_SEL_MAX                            (0x00000003U)


#define CSL_IOMUX_PR1_PRU1_GPO2_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                (0x00400000U)
#define CSL_IOMUX_PR1_PRU1_GPO2_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT               (0x00000016U)
#define CSL_IOMUX_PR1_PRU1_GPO2_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL            (0x00000000U)
#define CSL_IOMUX_PR1_PRU1_GPO2_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                 (0x00000001U)


#define CSL_IOMUX_PR1_PRU1_GPO2_CFG_REG_SC1_MASK                                (0x00000400U)
#define CSL_IOMUX_PR1_PRU1_GPO2_CFG_REG_SC1_SHIFT                               (0x0000000AU)
#define CSL_IOMUX_PR1_PRU1_GPO2_CFG_REG_SC1_RESETVAL                            (0x00000001U)
#define CSL_IOMUX_PR1_PRU1_GPO2_CFG_REG_SC1_MAX                                 (0x00000001U)



/* PR1_PRU1_GPO9_CFG_REG */
#define CSL_IOMUX_PR1_PRU1_GPO9_CFG_REG_FUNC_SEL_MASK                           (0x0000000FU)
#define CSL_IOMUX_PR1_PRU1_GPO9_CFG_REG_FUNC_SEL_SHIFT                          (0x00000000U)
#define CSL_IOMUX_PR1_PRU1_GPO9_CFG_REG_FUNC_SEL_RESETVAL                       (0x00000007U)
#define CSL_IOMUX_PR1_PRU1_GPO9_CFG_REG_FUNC_SEL_MAX                            (0x0000000FU)


#define CSL_IOMUX_PR1_PRU1_GPO9_CFG_REG_GPIO_SEL_MASK                           (0x00010000U)
#define CSL_IOMUX_PR1_PRU1_GPO9_CFG_REG_GPIO_SEL_SHIFT                          (0x00000010U)
#define CSL_IOMUX_PR1_PRU1_GPO9_CFG_REG_GPIO_SEL_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_PR1_PRU1_GPO9_CFG_REG_GPIO_SEL_MAX                            (0x00000001U)


#define CSL_IOMUX_PR1_PRU1_GPO9_CFG_REG_HSMASTER_MASK                           (0x80000000U)
#define CSL_IOMUX_PR1_PRU1_GPO9_CFG_REG_HSMASTER_SHIFT                          (0x0000001FU)
#define CSL_IOMUX_PR1_PRU1_GPO9_CFG_REG_HSMASTER_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_PR1_PRU1_GPO9_CFG_REG_HSMASTER_MAX                            (0x00000001U)


#define CSL_IOMUX_PR1_PRU1_GPO9_CFG_REG_HSMODE_MASK                           (0x40000000U)
#define CSL_IOMUX_PR1_PRU1_GPO9_CFG_REG_HSMODE_SHIFT                          (0x0000001EU)
#define CSL_IOMUX_PR1_PRU1_GPO9_CFG_REG_HSMODE_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_PR1_PRU1_GPO9_CFG_REG_HSMODE_MAX                            (0x00000001U)


#define CSL_IOMUX_PR1_PRU1_GPO9_CFG_REG_HVMODE_STATUS_MASK                      (0x01000000U)
#define CSL_IOMUX_PR1_PRU1_GPO9_CFG_REG_HVMODE_STATUS_SHIFT                     (0x00000018U)
#define CSL_IOMUX_PR1_PRU1_GPO9_CFG_REG_HVMODE_STATUS_RESETVAL                  (0x00000000U)
#define CSL_IOMUX_PR1_PRU1_GPO9_CFG_REG_HVMODE_STATUS_MAX                       (0x00000001U)


#define CSL_IOMUX_PR1_PRU1_GPO9_CFG_REG_ICSSM_GPIO_SEL_MASK                     (0x00200000U)
#define CSL_IOMUX_PR1_PRU1_GPO9_CFG_REG_ICSSM_GPIO_SEL_SHIFT                    (0x00000015U)
#define CSL_IOMUX_PR1_PRU1_GPO9_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                 (0x00000000U)
#define CSL_IOMUX_PR1_PRU1_GPO9_CFG_REG_ICSSM_GPIO_SEL_MAX                      (0x00000001U)


#define CSL_IOMUX_PR1_PRU1_GPO9_CFG_REG_IE_OVERRIDE_MASK                        (0x00000020U)
#define CSL_IOMUX_PR1_PRU1_GPO9_CFG_REG_IE_OVERRIDE_SHIFT                       (0x00000005U)
#define CSL_IOMUX_PR1_PRU1_GPO9_CFG_REG_IE_OVERRIDE_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_PR1_PRU1_GPO9_CFG_REG_IE_OVERRIDE_MAX                         (0x00000001U)


#define CSL_IOMUX_PR1_PRU1_GPO9_CFG_REG_IE_OVERRIDE_CTRL_MASK                   (0x00000010U)
#define CSL_IOMUX_PR1_PRU1_GPO9_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                  (0x00000004U)
#define CSL_IOMUX_PR1_PRU1_GPO9_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL               (0x00000001U)
#define CSL_IOMUX_PR1_PRU1_GPO9_CFG_REG_IE_OVERRIDE_CTRL_MAX                    (0x00000001U)


#define CSL_IOMUX_PR1_PRU1_GPO9_CFG_REG_INP_INV_SEL_MASK                        (0x00100000U)
#define CSL_IOMUX_PR1_PRU1_GPO9_CFG_REG_INP_INV_SEL_SHIFT                       (0x00000014U)
#define CSL_IOMUX_PR1_PRU1_GPO9_CFG_REG_INP_INV_SEL_RESETVAL                    (0x00000000U)
#define CSL_IOMUX_PR1_PRU1_GPO9_CFG_REG_INP_INV_SEL_MAX                         (0x00000001U)


#define CSL_IOMUX_PR1_PRU1_GPO9_CFG_REG_OE_OVERRIDE_MASK                        (0x00000080U)
#define CSL_IOMUX_PR1_PRU1_GPO9_CFG_REG_OE_OVERRIDE_SHIFT                       (0x00000007U)
#define CSL_IOMUX_PR1_PRU1_GPO9_CFG_REG_OE_OVERRIDE_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_PR1_PRU1_GPO9_CFG_REG_OE_OVERRIDE_MAX                         (0x00000001U)


#define CSL_IOMUX_PR1_PRU1_GPO9_CFG_REG_OE_OVERRIDE_CTRL_MASK                   (0x00000040U)
#define CSL_IOMUX_PR1_PRU1_GPO9_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                  (0x00000006U)
#define CSL_IOMUX_PR1_PRU1_GPO9_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL               (0x00000001U)
#define CSL_IOMUX_PR1_PRU1_GPO9_CFG_REG_OE_OVERRIDE_CTRL_MAX                    (0x00000001U)


#define CSL_IOMUX_PR1_PRU1_GPO9_CFG_REG_PE_MASK                                 (0x00000100U)
#define CSL_IOMUX_PR1_PRU1_GPO9_CFG_REG_PE_SHIFT                                (0x00000008U)
#define CSL_IOMUX_PR1_PRU1_GPO9_CFG_REG_PE_RESETVAL                             (0x00000001U)
#define CSL_IOMUX_PR1_PRU1_GPO9_CFG_REG_PE_MAX                                  (0x00000001U)


#define CSL_IOMUX_PR1_PRU1_GPO9_CFG_REG_PUPDSEL_MASK                            (0x00000200U)
#define CSL_IOMUX_PR1_PRU1_GPO9_CFG_REG_PUPDSEL_SHIFT                           (0x00000009U)
#define CSL_IOMUX_PR1_PRU1_GPO9_CFG_REG_PUPDSEL_RESETVAL                        (0x00000000U)
#define CSL_IOMUX_PR1_PRU1_GPO9_CFG_REG_PUPDSEL_MAX                             (0x00000001U)


#define CSL_IOMUX_PR1_PRU1_GPO9_CFG_REG_QUAL_SEL_MASK                           (0x000C0000U)
#define CSL_IOMUX_PR1_PRU1_GPO9_CFG_REG_QUAL_SEL_SHIFT                          (0x00000012U)
#define CSL_IOMUX_PR1_PRU1_GPO9_CFG_REG_QUAL_SEL_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_PR1_PRU1_GPO9_CFG_REG_QUAL_SEL_MAX                            (0x00000003U)


#define CSL_IOMUX_PR1_PRU1_GPO9_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                (0x00400000U)
#define CSL_IOMUX_PR1_PRU1_GPO9_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT               (0x00000016U)
#define CSL_IOMUX_PR1_PRU1_GPO9_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL            (0x00000000U)
#define CSL_IOMUX_PR1_PRU1_GPO9_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                 (0x00000001U)


#define CSL_IOMUX_PR1_PRU1_GPO9_CFG_REG_SC1_MASK                                (0x00000400U)
#define CSL_IOMUX_PR1_PRU1_GPO9_CFG_REG_SC1_SHIFT                               (0x0000000AU)
#define CSL_IOMUX_PR1_PRU1_GPO9_CFG_REG_SC1_RESETVAL                            (0x00000001U)
#define CSL_IOMUX_PR1_PRU1_GPO9_CFG_REG_SC1_MAX                                 (0x00000001U)

/* UART1_RXD_CFG_REG */

#define CSL_IOMUX_UART1_RXD_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_IOMUX_UART1_RXD_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_IOMUX_UART1_RXD_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000007U)
#define CSL_IOMUX_UART1_RXD_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_IOMUX_UART1_RXD_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_IOMUX_UART1_RXD_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_IOMUX_UART1_RXD_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_IOMUX_UART1_RXD_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_IOMUX_UART1_RXD_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_IOMUX_UART1_RXD_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_IOMUX_UART1_RXD_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_IOMUX_UART1_RXD_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_IOMUX_UART1_RXD_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_IOMUX_UART1_RXD_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_IOMUX_UART1_RXD_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_IOMUX_UART1_RXD_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_IOMUX_UART1_RXD_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_IOMUX_UART1_RXD_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_IOMUX_UART1_RXD_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_IOMUX_UART1_RXD_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_IOMUX_UART1_RXD_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_IOMUX_UART1_RXD_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_IOMUX_UART1_RXD_CFG_REG_PI_RESETVAL                                (0x00000001U)
#define CSL_IOMUX_UART1_RXD_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_IOMUX_UART1_RXD_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_IOMUX_UART1_RXD_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_IOMUX_UART1_RXD_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_UART1_RXD_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_IOMUX_UART1_RXD_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_IOMUX_UART1_RXD_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_IOMUX_UART1_RXD_CFG_REG_SC1_RESETVAL                               (0x00000001U)
#define CSL_IOMUX_UART1_RXD_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_IOMUX_UART1_RXD_CFG_REG_GPIO_SEL_MASK                              (0x00030000U)
#define CSL_IOMUX_UART1_RXD_CFG_REG_GPIO_SEL_SHIFT                             (0x00000010U)
#define CSL_IOMUX_UART1_RXD_CFG_REG_GPIO_SEL_RESETVAL                          (0x00000000U)
#define CSL_IOMUX_UART1_RXD_CFG_REG_GPIO_SEL_MAX                               (0x00000003U)

#define CSL_IOMUX_UART1_RXD_CFG_REG_QUAL_SEL_MASK                              (0x000C0000U)
#define CSL_IOMUX_UART1_RXD_CFG_REG_QUAL_SEL_SHIFT                             (0x00000012U)
#define CSL_IOMUX_UART1_RXD_CFG_REG_QUAL_SEL_RESETVAL                          (0x00000000U)
#define CSL_IOMUX_UART1_RXD_CFG_REG_QUAL_SEL_MAX                               (0x00000003U)

#define CSL_IOMUX_UART1_RXD_CFG_REG_INP_INV_SEL_MASK                           (0x00100000U)
#define CSL_IOMUX_UART1_RXD_CFG_REG_INP_INV_SEL_SHIFT                          (0x00000014U)
#define CSL_IOMUX_UART1_RXD_CFG_REG_INP_INV_SEL_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_UART1_RXD_CFG_REG_INP_INV_SEL_MAX                            (0x00000001U)

#define CSL_IOMUX_UART1_RXD_CFG_REG_ICSSM_GPIO_SEL_MASK                        (0x00200000U)
#define CSL_IOMUX_UART1_RXD_CFG_REG_ICSSM_GPIO_SEL_SHIFT                       (0x00000015U)
#define CSL_IOMUX_UART1_RXD_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                    (0x00000000U)
#define CSL_IOMUX_UART1_RXD_CFG_REG_ICSSM_GPIO_SEL_MAX                         (0x00000001U)

#define CSL_IOMUX_UART1_RXD_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                   (0x00400000U)
#define CSL_IOMUX_UART1_RXD_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                  (0x00000016U)
#define CSL_IOMUX_UART1_RXD_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL               (0x00000000U)
#define CSL_IOMUX_UART1_RXD_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                    (0x00000001U)

#define CSL_IOMUX_UART1_RXD_CFG_REG_HSMODE_MASK                                (0x40000000U)
#define CSL_IOMUX_UART1_RXD_CFG_REG_HSMODE_SHIFT                               (0x0000001EU)
#define CSL_IOMUX_UART1_RXD_CFG_REG_HSMODE_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_UART1_RXD_CFG_REG_HSMODE_MAX                                 (0x00000001U)

#define CSL_IOMUX_UART1_RXD_CFG_REG_HSMASTER_MASK                              (0x80000000U)
#define CSL_IOMUX_UART1_RXD_CFG_REG_HSMASTER_SHIFT                             (0x0000001FU)
#define CSL_IOMUX_UART1_RXD_CFG_REG_HSMASTER_RESETVAL                          (0x00000000U)
#define CSL_IOMUX_UART1_RXD_CFG_REG_HSMASTER_MAX                               (0x00000001U)

#define CSL_IOMUX_UART1_RXD_CFG_REG_RESETVAL                                   (0x000005F7U)

/* UART1_TXD_CFG_REG */

#define CSL_IOMUX_UART1_TXD_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_IOMUX_UART1_TXD_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_IOMUX_UART1_TXD_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000007U)
#define CSL_IOMUX_UART1_TXD_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_IOMUX_UART1_TXD_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_IOMUX_UART1_TXD_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_IOMUX_UART1_TXD_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_IOMUX_UART1_TXD_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_IOMUX_UART1_TXD_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_IOMUX_UART1_TXD_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_IOMUX_UART1_TXD_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_IOMUX_UART1_TXD_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_IOMUX_UART1_TXD_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_IOMUX_UART1_TXD_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_IOMUX_UART1_TXD_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_IOMUX_UART1_TXD_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_IOMUX_UART1_TXD_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_IOMUX_UART1_TXD_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_IOMUX_UART1_TXD_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_IOMUX_UART1_TXD_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_IOMUX_UART1_TXD_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_IOMUX_UART1_TXD_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_IOMUX_UART1_TXD_CFG_REG_PI_RESETVAL                                (0x00000001U)
#define CSL_IOMUX_UART1_TXD_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_IOMUX_UART1_TXD_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_IOMUX_UART1_TXD_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_IOMUX_UART1_TXD_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_UART1_TXD_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_IOMUX_UART1_TXD_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_IOMUX_UART1_TXD_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_IOMUX_UART1_TXD_CFG_REG_SC1_RESETVAL                               (0x00000001U)
#define CSL_IOMUX_UART1_TXD_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_IOMUX_UART1_TXD_CFG_REG_GPIO_SEL_MASK                              (0x00030000U)
#define CSL_IOMUX_UART1_TXD_CFG_REG_GPIO_SEL_SHIFT                             (0x00000010U)
#define CSL_IOMUX_UART1_TXD_CFG_REG_GPIO_SEL_RESETVAL                          (0x00000000U)
#define CSL_IOMUX_UART1_TXD_CFG_REG_GPIO_SEL_MAX                               (0x00000003U)

#define CSL_IOMUX_UART1_TXD_CFG_REG_QUAL_SEL_MASK                              (0x000C0000U)
#define CSL_IOMUX_UART1_TXD_CFG_REG_QUAL_SEL_SHIFT                             (0x00000012U)
#define CSL_IOMUX_UART1_TXD_CFG_REG_QUAL_SEL_RESETVAL                          (0x00000000U)
#define CSL_IOMUX_UART1_TXD_CFG_REG_QUAL_SEL_MAX                               (0x00000003U)

#define CSL_IOMUX_UART1_TXD_CFG_REG_INP_INV_SEL_MASK                           (0x00100000U)
#define CSL_IOMUX_UART1_TXD_CFG_REG_INP_INV_SEL_SHIFT                          (0x00000014U)
#define CSL_IOMUX_UART1_TXD_CFG_REG_INP_INV_SEL_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_UART1_TXD_CFG_REG_INP_INV_SEL_MAX                            (0x00000001U)

#define CSL_IOMUX_UART1_TXD_CFG_REG_ICSSM_GPIO_SEL_MASK                        (0x00200000U)
#define CSL_IOMUX_UART1_TXD_CFG_REG_ICSSM_GPIO_SEL_SHIFT                       (0x00000015U)
#define CSL_IOMUX_UART1_TXD_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                    (0x00000000U)
#define CSL_IOMUX_UART1_TXD_CFG_REG_ICSSM_GPIO_SEL_MAX                         (0x00000001U)

#define CSL_IOMUX_UART1_TXD_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                   (0x00400000U)
#define CSL_IOMUX_UART1_TXD_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                  (0x00000016U)
#define CSL_IOMUX_UART1_TXD_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL               (0x00000000U)
#define CSL_IOMUX_UART1_TXD_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                    (0x00000001U)

#define CSL_IOMUX_UART1_TXD_CFG_REG_HSMODE_MASK                                (0x40000000U)
#define CSL_IOMUX_UART1_TXD_CFG_REG_HSMODE_SHIFT                               (0x0000001EU)
#define CSL_IOMUX_UART1_TXD_CFG_REG_HSMODE_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_UART1_TXD_CFG_REG_HSMODE_MAX                                 (0x00000001U)

#define CSL_IOMUX_UART1_TXD_CFG_REG_HSMASTER_MASK                              (0x80000000U)
#define CSL_IOMUX_UART1_TXD_CFG_REG_HSMASTER_SHIFT                             (0x0000001FU)
#define CSL_IOMUX_UART1_TXD_CFG_REG_HSMASTER_RESETVAL                          (0x00000000U)
#define CSL_IOMUX_UART1_TXD_CFG_REG_HSMASTER_MAX                               (0x00000001U)

#define CSL_IOMUX_UART1_TXD_CFG_REG_RESETVAL                                   (0x000005F7U)

/* MMC0_CLK_CFG_REG */

#define CSL_IOMUX_MMC0_CLK_CFG_REG_FUNC_SEL_MASK                               (0x0000000FU)
#define CSL_IOMUX_MMC0_CLK_CFG_REG_FUNC_SEL_SHIFT                              (0x00000000U)
#define CSL_IOMUX_MMC0_CLK_CFG_REG_FUNC_SEL_RESETVAL                           (0x00000007U)
#define CSL_IOMUX_MMC0_CLK_CFG_REG_FUNC_SEL_MAX                                (0x0000000FU)

#define CSL_IOMUX_MMC0_CLK_CFG_REG_IE_OVERRIDE_CTRL_MASK                       (0x00000010U)
#define CSL_IOMUX_MMC0_CLK_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                      (0x00000004U)
#define CSL_IOMUX_MMC0_CLK_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_MMC0_CLK_CFG_REG_IE_OVERRIDE_CTRL_MAX                        (0x00000001U)

#define CSL_IOMUX_MMC0_CLK_CFG_REG_IE_OVERRIDE_MASK                            (0x00000020U)
#define CSL_IOMUX_MMC0_CLK_CFG_REG_IE_OVERRIDE_SHIFT                           (0x00000005U)
#define CSL_IOMUX_MMC0_CLK_CFG_REG_IE_OVERRIDE_RESETVAL                        (0x00000001U)
#define CSL_IOMUX_MMC0_CLK_CFG_REG_IE_OVERRIDE_MAX                             (0x00000001U)

#define CSL_IOMUX_MMC0_CLK_CFG_REG_OE_OVERRIDE_CTRL_MASK                       (0x00000040U)
#define CSL_IOMUX_MMC0_CLK_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                      (0x00000006U)
#define CSL_IOMUX_MMC0_CLK_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_MMC0_CLK_CFG_REG_OE_OVERRIDE_CTRL_MAX                        (0x00000001U)

#define CSL_IOMUX_MMC0_CLK_CFG_REG_OE_OVERRIDE_MASK                            (0x00000080U)
#define CSL_IOMUX_MMC0_CLK_CFG_REG_OE_OVERRIDE_SHIFT                           (0x00000007U)
#define CSL_IOMUX_MMC0_CLK_CFG_REG_OE_OVERRIDE_RESETVAL                        (0x00000001U)
#define CSL_IOMUX_MMC0_CLK_CFG_REG_OE_OVERRIDE_MAX                             (0x00000001U)

#define CSL_IOMUX_MMC0_CLK_CFG_REG_PI_MASK                                     (0x00000100U)
#define CSL_IOMUX_MMC0_CLK_CFG_REG_PI_SHIFT                                    (0x00000008U)
#define CSL_IOMUX_MMC0_CLK_CFG_REG_PI_RESETVAL                                 (0x00000001U)
#define CSL_IOMUX_MMC0_CLK_CFG_REG_PI_MAX                                      (0x00000001U)

#define CSL_IOMUX_MMC0_CLK_CFG_REG_PUPDSEL_MASK                                (0x00000200U)
#define CSL_IOMUX_MMC0_CLK_CFG_REG_PUPDSEL_SHIFT                               (0x00000009U)
#define CSL_IOMUX_MMC0_CLK_CFG_REG_PUPDSEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_MMC0_CLK_CFG_REG_PUPDSEL_MAX                                 (0x00000001U)

#define CSL_IOMUX_MMC0_CLK_CFG_REG_SC1_MASK                                    (0x00000400U)
#define CSL_IOMUX_MMC0_CLK_CFG_REG_SC1_SHIFT                                   (0x0000000AU)
#define CSL_IOMUX_MMC0_CLK_CFG_REG_SC1_RESETVAL                                (0x00000001U)
#define CSL_IOMUX_MMC0_CLK_CFG_REG_SC1_MAX                                     (0x00000001U)

#define CSL_IOMUX_MMC0_CLK_CFG_REG_GPIO_SEL_MASK                               (0x00030000U)
#define CSL_IOMUX_MMC0_CLK_CFG_REG_GPIO_SEL_SHIFT                              (0x00000010U)
#define CSL_IOMUX_MMC0_CLK_CFG_REG_GPIO_SEL_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_MMC0_CLK_CFG_REG_GPIO_SEL_MAX                                (0x00000003U)

#define CSL_IOMUX_MMC0_CLK_CFG_REG_QUAL_SEL_MASK                               (0x000C0000U)
#define CSL_IOMUX_MMC0_CLK_CFG_REG_QUAL_SEL_SHIFT                              (0x00000012U)
#define CSL_IOMUX_MMC0_CLK_CFG_REG_QUAL_SEL_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_MMC0_CLK_CFG_REG_QUAL_SEL_MAX                                (0x00000003U)

#define CSL_IOMUX_MMC0_CLK_CFG_REG_INP_INV_SEL_MASK                            (0x00100000U)
#define CSL_IOMUX_MMC0_CLK_CFG_REG_INP_INV_SEL_SHIFT                           (0x00000014U)
#define CSL_IOMUX_MMC0_CLK_CFG_REG_INP_INV_SEL_RESETVAL                        (0x00000000U)
#define CSL_IOMUX_MMC0_CLK_CFG_REG_INP_INV_SEL_MAX                             (0x00000001U)

#define CSL_IOMUX_MMC0_CLK_CFG_REG_ICSSM_GPIO_SEL_MASK                         (0x00200000U)
#define CSL_IOMUX_MMC0_CLK_CFG_REG_ICSSM_GPIO_SEL_SHIFT                        (0x00000015U)
#define CSL_IOMUX_MMC0_CLK_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_MMC0_CLK_CFG_REG_ICSSM_GPIO_SEL_MAX                          (0x00000001U)

#define CSL_IOMUX_MMC0_CLK_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                    (0x00400000U)
#define CSL_IOMUX_MMC0_CLK_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                   (0x00000016U)
#define CSL_IOMUX_MMC0_CLK_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL                (0x00000000U)
#define CSL_IOMUX_MMC0_CLK_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                     (0x00000001U)

#define CSL_IOMUX_MMC0_CLK_CFG_REG_HSMODE_MASK                                 (0x40000000U)
#define CSL_IOMUX_MMC0_CLK_CFG_REG_HSMODE_SHIFT                                (0x0000001EU)
#define CSL_IOMUX_MMC0_CLK_CFG_REG_HSMODE_RESETVAL                             (0x00000000U)
#define CSL_IOMUX_MMC0_CLK_CFG_REG_HSMODE_MAX                                  (0x00000001U)

#define CSL_IOMUX_MMC0_CLK_CFG_REG_HSMASTER_MASK                               (0x80000000U)
#define CSL_IOMUX_MMC0_CLK_CFG_REG_HSMASTER_SHIFT                              (0x0000001FU)
#define CSL_IOMUX_MMC0_CLK_CFG_REG_HSMASTER_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_MMC0_CLK_CFG_REG_HSMASTER_MAX                                (0x00000001U)

#define CSL_IOMUX_MMC0_CLK_CFG_REG_RESETVAL                                    (0x000005F7U)

/* MMC0_CMD_CFG_REG */

#define CSL_IOMUX_MMC0_CMD_CFG_REG_FUNC_SEL_MASK                               (0x0000000FU)
#define CSL_IOMUX_MMC0_CMD_CFG_REG_FUNC_SEL_SHIFT                              (0x00000000U)
#define CSL_IOMUX_MMC0_CMD_CFG_REG_FUNC_SEL_RESETVAL                           (0x00000007U)
#define CSL_IOMUX_MMC0_CMD_CFG_REG_FUNC_SEL_MAX                                (0x0000000FU)

#define CSL_IOMUX_MMC0_CMD_CFG_REG_IE_OVERRIDE_CTRL_MASK                       (0x00000010U)
#define CSL_IOMUX_MMC0_CMD_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                      (0x00000004U)
#define CSL_IOMUX_MMC0_CMD_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_MMC0_CMD_CFG_REG_IE_OVERRIDE_CTRL_MAX                        (0x00000001U)

#define CSL_IOMUX_MMC0_CMD_CFG_REG_IE_OVERRIDE_MASK                            (0x00000020U)
#define CSL_IOMUX_MMC0_CMD_CFG_REG_IE_OVERRIDE_SHIFT                           (0x00000005U)
#define CSL_IOMUX_MMC0_CMD_CFG_REG_IE_OVERRIDE_RESETVAL                        (0x00000001U)
#define CSL_IOMUX_MMC0_CMD_CFG_REG_IE_OVERRIDE_MAX                             (0x00000001U)

#define CSL_IOMUX_MMC0_CMD_CFG_REG_OE_OVERRIDE_CTRL_MASK                       (0x00000040U)
#define CSL_IOMUX_MMC0_CMD_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                      (0x00000006U)
#define CSL_IOMUX_MMC0_CMD_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_MMC0_CMD_CFG_REG_OE_OVERRIDE_CTRL_MAX                        (0x00000001U)

#define CSL_IOMUX_MMC0_CMD_CFG_REG_OE_OVERRIDE_MASK                            (0x00000080U)
#define CSL_IOMUX_MMC0_CMD_CFG_REG_OE_OVERRIDE_SHIFT                           (0x00000007U)
#define CSL_IOMUX_MMC0_CMD_CFG_REG_OE_OVERRIDE_RESETVAL                        (0x00000001U)
#define CSL_IOMUX_MMC0_CMD_CFG_REG_OE_OVERRIDE_MAX                             (0x00000001U)

#define CSL_IOMUX_MMC0_CMD_CFG_REG_PI_MASK                                     (0x00000100U)
#define CSL_IOMUX_MMC0_CMD_CFG_REG_PI_SHIFT                                    (0x00000008U)
#define CSL_IOMUX_MMC0_CMD_CFG_REG_PI_RESETVAL                                 (0x00000001U)
#define CSL_IOMUX_MMC0_CMD_CFG_REG_PI_MAX                                      (0x00000001U)

#define CSL_IOMUX_MMC0_CMD_CFG_REG_PUPDSEL_MASK                                (0x00000200U)
#define CSL_IOMUX_MMC0_CMD_CFG_REG_PUPDSEL_SHIFT                               (0x00000009U)
#define CSL_IOMUX_MMC0_CMD_CFG_REG_PUPDSEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_MMC0_CMD_CFG_REG_PUPDSEL_MAX                                 (0x00000001U)

#define CSL_IOMUX_MMC0_CMD_CFG_REG_SC1_MASK                                    (0x00000400U)
#define CSL_IOMUX_MMC0_CMD_CFG_REG_SC1_SHIFT                                   (0x0000000AU)
#define CSL_IOMUX_MMC0_CMD_CFG_REG_SC1_RESETVAL                                (0x00000001U)
#define CSL_IOMUX_MMC0_CMD_CFG_REG_SC1_MAX                                     (0x00000001U)

#define CSL_IOMUX_MMC0_CMD_CFG_REG_GPIO_SEL_MASK                               (0x00030000U)
#define CSL_IOMUX_MMC0_CMD_CFG_REG_GPIO_SEL_SHIFT                              (0x00000010U)
#define CSL_IOMUX_MMC0_CMD_CFG_REG_GPIO_SEL_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_MMC0_CMD_CFG_REG_GPIO_SEL_MAX                                (0x00000003U)

#define CSL_IOMUX_MMC0_CMD_CFG_REG_QUAL_SEL_MASK                               (0x000C0000U)
#define CSL_IOMUX_MMC0_CMD_CFG_REG_QUAL_SEL_SHIFT                              (0x00000012U)
#define CSL_IOMUX_MMC0_CMD_CFG_REG_QUAL_SEL_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_MMC0_CMD_CFG_REG_QUAL_SEL_MAX                                (0x00000003U)

#define CSL_IOMUX_MMC0_CMD_CFG_REG_INP_INV_SEL_MASK                            (0x00100000U)
#define CSL_IOMUX_MMC0_CMD_CFG_REG_INP_INV_SEL_SHIFT                           (0x00000014U)
#define CSL_IOMUX_MMC0_CMD_CFG_REG_INP_INV_SEL_RESETVAL                        (0x00000000U)
#define CSL_IOMUX_MMC0_CMD_CFG_REG_INP_INV_SEL_MAX                             (0x00000001U)

#define CSL_IOMUX_MMC0_CMD_CFG_REG_ICSSM_GPIO_SEL_MASK                         (0x00200000U)
#define CSL_IOMUX_MMC0_CMD_CFG_REG_ICSSM_GPIO_SEL_SHIFT                        (0x00000015U)
#define CSL_IOMUX_MMC0_CMD_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_MMC0_CMD_CFG_REG_ICSSM_GPIO_SEL_MAX                          (0x00000001U)

#define CSL_IOMUX_MMC0_CMD_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                    (0x00400000U)
#define CSL_IOMUX_MMC0_CMD_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                   (0x00000016U)
#define CSL_IOMUX_MMC0_CMD_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL                (0x00000000U)
#define CSL_IOMUX_MMC0_CMD_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                     (0x00000001U)

#define CSL_IOMUX_MMC0_CMD_CFG_REG_HSMODE_MASK                                 (0x40000000U)
#define CSL_IOMUX_MMC0_CMD_CFG_REG_HSMODE_SHIFT                                (0x0000001EU)
#define CSL_IOMUX_MMC0_CMD_CFG_REG_HSMODE_RESETVAL                             (0x00000000U)
#define CSL_IOMUX_MMC0_CMD_CFG_REG_HSMODE_MAX                                  (0x00000001U)

#define CSL_IOMUX_MMC0_CMD_CFG_REG_HSMASTER_MASK                               (0x80000000U)
#define CSL_IOMUX_MMC0_CMD_CFG_REG_HSMASTER_SHIFT                              (0x0000001FU)
#define CSL_IOMUX_MMC0_CMD_CFG_REG_HSMASTER_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_MMC0_CMD_CFG_REG_HSMASTER_MAX                                (0x00000001U)

#define CSL_IOMUX_MMC0_CMD_CFG_REG_RESETVAL                                    (0x000005F7U)

/* MMC0_D0_CFG_REG */

#define CSL_IOMUX_MMC0_D0_CFG_REG_FUNC_SEL_MASK                                (0x0000000FU)
#define CSL_IOMUX_MMC0_D0_CFG_REG_FUNC_SEL_SHIFT                               (0x00000000U)
#define CSL_IOMUX_MMC0_D0_CFG_REG_FUNC_SEL_RESETVAL                            (0x00000007U)
#define CSL_IOMUX_MMC0_D0_CFG_REG_FUNC_SEL_MAX                                 (0x0000000FU)

#define CSL_IOMUX_MMC0_D0_CFG_REG_IE_OVERRIDE_CTRL_MASK                        (0x00000010U)
#define CSL_IOMUX_MMC0_D0_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                       (0x00000004U)
#define CSL_IOMUX_MMC0_D0_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_MMC0_D0_CFG_REG_IE_OVERRIDE_CTRL_MAX                         (0x00000001U)

#define CSL_IOMUX_MMC0_D0_CFG_REG_IE_OVERRIDE_MASK                             (0x00000020U)
#define CSL_IOMUX_MMC0_D0_CFG_REG_IE_OVERRIDE_SHIFT                            (0x00000005U)
#define CSL_IOMUX_MMC0_D0_CFG_REG_IE_OVERRIDE_RESETVAL                         (0x00000001U)
#define CSL_IOMUX_MMC0_D0_CFG_REG_IE_OVERRIDE_MAX                              (0x00000001U)

#define CSL_IOMUX_MMC0_D0_CFG_REG_OE_OVERRIDE_CTRL_MASK                        (0x00000040U)
#define CSL_IOMUX_MMC0_D0_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                       (0x00000006U)
#define CSL_IOMUX_MMC0_D0_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_MMC0_D0_CFG_REG_OE_OVERRIDE_CTRL_MAX                         (0x00000001U)

#define CSL_IOMUX_MMC0_D0_CFG_REG_OE_OVERRIDE_MASK                             (0x00000080U)
#define CSL_IOMUX_MMC0_D0_CFG_REG_OE_OVERRIDE_SHIFT                            (0x00000007U)
#define CSL_IOMUX_MMC0_D0_CFG_REG_OE_OVERRIDE_RESETVAL                         (0x00000001U)
#define CSL_IOMUX_MMC0_D0_CFG_REG_OE_OVERRIDE_MAX                              (0x00000001U)

#define CSL_IOMUX_MMC0_D0_CFG_REG_PI_MASK                                      (0x00000100U)
#define CSL_IOMUX_MMC0_D0_CFG_REG_PI_SHIFT                                     (0x00000008U)
#define CSL_IOMUX_MMC0_D0_CFG_REG_PI_RESETVAL                                  (0x00000001U)
#define CSL_IOMUX_MMC0_D0_CFG_REG_PI_MAX                                       (0x00000001U)

#define CSL_IOMUX_MMC0_D0_CFG_REG_PUPDSEL_MASK                                 (0x00000200U)
#define CSL_IOMUX_MMC0_D0_CFG_REG_PUPDSEL_SHIFT                                (0x00000009U)
#define CSL_IOMUX_MMC0_D0_CFG_REG_PUPDSEL_RESETVAL                             (0x00000000U)
#define CSL_IOMUX_MMC0_D0_CFG_REG_PUPDSEL_MAX                                  (0x00000001U)

#define CSL_IOMUX_MMC0_D0_CFG_REG_SC1_MASK                                     (0x00000400U)
#define CSL_IOMUX_MMC0_D0_CFG_REG_SC1_SHIFT                                    (0x0000000AU)
#define CSL_IOMUX_MMC0_D0_CFG_REG_SC1_RESETVAL                                 (0x00000001U)
#define CSL_IOMUX_MMC0_D0_CFG_REG_SC1_MAX                                      (0x00000001U)

#define CSL_IOMUX_MMC0_D0_CFG_REG_GPIO_SEL_MASK                                (0x00030000U)
#define CSL_IOMUX_MMC0_D0_CFG_REG_GPIO_SEL_SHIFT                               (0x00000010U)
#define CSL_IOMUX_MMC0_D0_CFG_REG_GPIO_SEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_MMC0_D0_CFG_REG_GPIO_SEL_MAX                                 (0x00000003U)

#define CSL_IOMUX_MMC0_D0_CFG_REG_QUAL_SEL_MASK                                (0x000C0000U)
#define CSL_IOMUX_MMC0_D0_CFG_REG_QUAL_SEL_SHIFT                               (0x00000012U)
#define CSL_IOMUX_MMC0_D0_CFG_REG_QUAL_SEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_MMC0_D0_CFG_REG_QUAL_SEL_MAX                                 (0x00000003U)

#define CSL_IOMUX_MMC0_D0_CFG_REG_INP_INV_SEL_MASK                             (0x00100000U)
#define CSL_IOMUX_MMC0_D0_CFG_REG_INP_INV_SEL_SHIFT                            (0x00000014U)
#define CSL_IOMUX_MMC0_D0_CFG_REG_INP_INV_SEL_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_MMC0_D0_CFG_REG_INP_INV_SEL_MAX                              (0x00000001U)

#define CSL_IOMUX_MMC0_D0_CFG_REG_ICSSM_GPIO_SEL_MASK                          (0x00200000U)
#define CSL_IOMUX_MMC0_D0_CFG_REG_ICSSM_GPIO_SEL_SHIFT                         (0x00000015U)
#define CSL_IOMUX_MMC0_D0_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_MMC0_D0_CFG_REG_ICSSM_GPIO_SEL_MAX                           (0x00000001U)

#define CSL_IOMUX_MMC0_D0_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                     (0x00400000U)
#define CSL_IOMUX_MMC0_D0_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                    (0x00000016U)
#define CSL_IOMUX_MMC0_D0_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL                 (0x00000000U)
#define CSL_IOMUX_MMC0_D0_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                      (0x00000001U)

#define CSL_IOMUX_MMC0_D0_CFG_REG_HSMODE_MASK                                  (0x40000000U)
#define CSL_IOMUX_MMC0_D0_CFG_REG_HSMODE_SHIFT                                 (0x0000001EU)
#define CSL_IOMUX_MMC0_D0_CFG_REG_HSMODE_RESETVAL                              (0x00000000U)
#define CSL_IOMUX_MMC0_D0_CFG_REG_HSMODE_MAX                                   (0x00000001U)

#define CSL_IOMUX_MMC0_D0_CFG_REG_HSMASTER_MASK                                (0x80000000U)
#define CSL_IOMUX_MMC0_D0_CFG_REG_HSMASTER_SHIFT                               (0x0000001FU)
#define CSL_IOMUX_MMC0_D0_CFG_REG_HSMASTER_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_MMC0_D0_CFG_REG_HSMASTER_MAX                                 (0x00000001U)

#define CSL_IOMUX_MMC0_D0_CFG_REG_RESETVAL                                     (0x000005F7U)

/* MMC0_D1_CFG_REG */

#define CSL_IOMUX_MMC0_D1_CFG_REG_FUNC_SEL_MASK                                (0x0000000FU)
#define CSL_IOMUX_MMC0_D1_CFG_REG_FUNC_SEL_SHIFT                               (0x00000000U)
#define CSL_IOMUX_MMC0_D1_CFG_REG_FUNC_SEL_RESETVAL                            (0x00000007U)
#define CSL_IOMUX_MMC0_D1_CFG_REG_FUNC_SEL_MAX                                 (0x0000000FU)

#define CSL_IOMUX_MMC0_D1_CFG_REG_IE_OVERRIDE_CTRL_MASK                        (0x00000010U)
#define CSL_IOMUX_MMC0_D1_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                       (0x00000004U)
#define CSL_IOMUX_MMC0_D1_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_MMC0_D1_CFG_REG_IE_OVERRIDE_CTRL_MAX                         (0x00000001U)

#define CSL_IOMUX_MMC0_D1_CFG_REG_IE_OVERRIDE_MASK                             (0x00000020U)
#define CSL_IOMUX_MMC0_D1_CFG_REG_IE_OVERRIDE_SHIFT                            (0x00000005U)
#define CSL_IOMUX_MMC0_D1_CFG_REG_IE_OVERRIDE_RESETVAL                         (0x00000001U)
#define CSL_IOMUX_MMC0_D1_CFG_REG_IE_OVERRIDE_MAX                              (0x00000001U)

#define CSL_IOMUX_MMC0_D1_CFG_REG_OE_OVERRIDE_CTRL_MASK                        (0x00000040U)
#define CSL_IOMUX_MMC0_D1_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                       (0x00000006U)
#define CSL_IOMUX_MMC0_D1_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_MMC0_D1_CFG_REG_OE_OVERRIDE_CTRL_MAX                         (0x00000001U)

#define CSL_IOMUX_MMC0_D1_CFG_REG_OE_OVERRIDE_MASK                             (0x00000080U)
#define CSL_IOMUX_MMC0_D1_CFG_REG_OE_OVERRIDE_SHIFT                            (0x00000007U)
#define CSL_IOMUX_MMC0_D1_CFG_REG_OE_OVERRIDE_RESETVAL                         (0x00000001U)
#define CSL_IOMUX_MMC0_D1_CFG_REG_OE_OVERRIDE_MAX                              (0x00000001U)

#define CSL_IOMUX_MMC0_D1_CFG_REG_PI_MASK                                      (0x00000100U)
#define CSL_IOMUX_MMC0_D1_CFG_REG_PI_SHIFT                                     (0x00000008U)
#define CSL_IOMUX_MMC0_D1_CFG_REG_PI_RESETVAL                                  (0x00000001U)
#define CSL_IOMUX_MMC0_D1_CFG_REG_PI_MAX                                       (0x00000001U)

#define CSL_IOMUX_MMC0_D1_CFG_REG_PUPDSEL_MASK                                 (0x00000200U)
#define CSL_IOMUX_MMC0_D1_CFG_REG_PUPDSEL_SHIFT                                (0x00000009U)
#define CSL_IOMUX_MMC0_D1_CFG_REG_PUPDSEL_RESETVAL                             (0x00000000U)
#define CSL_IOMUX_MMC0_D1_CFG_REG_PUPDSEL_MAX                                  (0x00000001U)

#define CSL_IOMUX_MMC0_D1_CFG_REG_SC1_MASK                                     (0x00000400U)
#define CSL_IOMUX_MMC0_D1_CFG_REG_SC1_SHIFT                                    (0x0000000AU)
#define CSL_IOMUX_MMC0_D1_CFG_REG_SC1_RESETVAL                                 (0x00000001U)
#define CSL_IOMUX_MMC0_D1_CFG_REG_SC1_MAX                                      (0x00000001U)

#define CSL_IOMUX_MMC0_D1_CFG_REG_GPIO_SEL_MASK                                (0x00030000U)
#define CSL_IOMUX_MMC0_D1_CFG_REG_GPIO_SEL_SHIFT                               (0x00000010U)
#define CSL_IOMUX_MMC0_D1_CFG_REG_GPIO_SEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_MMC0_D1_CFG_REG_GPIO_SEL_MAX                                 (0x00000003U)

#define CSL_IOMUX_MMC0_D1_CFG_REG_QUAL_SEL_MASK                                (0x000C0000U)
#define CSL_IOMUX_MMC0_D1_CFG_REG_QUAL_SEL_SHIFT                               (0x00000012U)
#define CSL_IOMUX_MMC0_D1_CFG_REG_QUAL_SEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_MMC0_D1_CFG_REG_QUAL_SEL_MAX                                 (0x00000003U)

#define CSL_IOMUX_MMC0_D1_CFG_REG_INP_INV_SEL_MASK                             (0x00100000U)
#define CSL_IOMUX_MMC0_D1_CFG_REG_INP_INV_SEL_SHIFT                            (0x00000014U)
#define CSL_IOMUX_MMC0_D1_CFG_REG_INP_INV_SEL_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_MMC0_D1_CFG_REG_INP_INV_SEL_MAX                              (0x00000001U)

#define CSL_IOMUX_MMC0_D1_CFG_REG_ICSSM_GPIO_SEL_MASK                          (0x00200000U)
#define CSL_IOMUX_MMC0_D1_CFG_REG_ICSSM_GPIO_SEL_SHIFT                         (0x00000015U)
#define CSL_IOMUX_MMC0_D1_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_MMC0_D1_CFG_REG_ICSSM_GPIO_SEL_MAX                           (0x00000001U)

#define CSL_IOMUX_MMC0_D1_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                     (0x00400000U)
#define CSL_IOMUX_MMC0_D1_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                    (0x00000016U)
#define CSL_IOMUX_MMC0_D1_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL                 (0x00000000U)
#define CSL_IOMUX_MMC0_D1_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                      (0x00000001U)

#define CSL_IOMUX_MMC0_D1_CFG_REG_HSMODE_MASK                                  (0x40000000U)
#define CSL_IOMUX_MMC0_D1_CFG_REG_HSMODE_SHIFT                                 (0x0000001EU)
#define CSL_IOMUX_MMC0_D1_CFG_REG_HSMODE_RESETVAL                              (0x00000000U)
#define CSL_IOMUX_MMC0_D1_CFG_REG_HSMODE_MAX                                   (0x00000001U)

#define CSL_IOMUX_MMC0_D1_CFG_REG_HSMASTER_MASK                                (0x80000000U)
#define CSL_IOMUX_MMC0_D1_CFG_REG_HSMASTER_SHIFT                               (0x0000001FU)
#define CSL_IOMUX_MMC0_D1_CFG_REG_HSMASTER_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_MMC0_D1_CFG_REG_HSMASTER_MAX                                 (0x00000001U)

#define CSL_IOMUX_MMC0_D1_CFG_REG_RESETVAL                                     (0x000005F7U)

/* MMC0_D2_CFG_REG */

#define CSL_IOMUX_MMC0_D2_CFG_REG_FUNC_SEL_MASK                                (0x0000000FU)
#define CSL_IOMUX_MMC0_D2_CFG_REG_FUNC_SEL_SHIFT                               (0x00000000U)
#define CSL_IOMUX_MMC0_D2_CFG_REG_FUNC_SEL_RESETVAL                            (0x00000007U)
#define CSL_IOMUX_MMC0_D2_CFG_REG_FUNC_SEL_MAX                                 (0x0000000FU)

#define CSL_IOMUX_MMC0_D2_CFG_REG_IE_OVERRIDE_CTRL_MASK                        (0x00000010U)
#define CSL_IOMUX_MMC0_D2_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                       (0x00000004U)
#define CSL_IOMUX_MMC0_D2_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_MMC0_D2_CFG_REG_IE_OVERRIDE_CTRL_MAX                         (0x00000001U)

#define CSL_IOMUX_MMC0_D2_CFG_REG_IE_OVERRIDE_MASK                             (0x00000020U)
#define CSL_IOMUX_MMC0_D2_CFG_REG_IE_OVERRIDE_SHIFT                            (0x00000005U)
#define CSL_IOMUX_MMC0_D2_CFG_REG_IE_OVERRIDE_RESETVAL                         (0x00000001U)
#define CSL_IOMUX_MMC0_D2_CFG_REG_IE_OVERRIDE_MAX                              (0x00000001U)

#define CSL_IOMUX_MMC0_D2_CFG_REG_OE_OVERRIDE_CTRL_MASK                        (0x00000040U)
#define CSL_IOMUX_MMC0_D2_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                       (0x00000006U)
#define CSL_IOMUX_MMC0_D2_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_MMC0_D2_CFG_REG_OE_OVERRIDE_CTRL_MAX                         (0x00000001U)

#define CSL_IOMUX_MMC0_D2_CFG_REG_OE_OVERRIDE_MASK                             (0x00000080U)
#define CSL_IOMUX_MMC0_D2_CFG_REG_OE_OVERRIDE_SHIFT                            (0x00000007U)
#define CSL_IOMUX_MMC0_D2_CFG_REG_OE_OVERRIDE_RESETVAL                         (0x00000001U)
#define CSL_IOMUX_MMC0_D2_CFG_REG_OE_OVERRIDE_MAX                              (0x00000001U)

#define CSL_IOMUX_MMC0_D2_CFG_REG_PI_MASK                                      (0x00000100U)
#define CSL_IOMUX_MMC0_D2_CFG_REG_PI_SHIFT                                     (0x00000008U)
#define CSL_IOMUX_MMC0_D2_CFG_REG_PI_RESETVAL                                  (0x00000001U)
#define CSL_IOMUX_MMC0_D2_CFG_REG_PI_MAX                                       (0x00000001U)

#define CSL_IOMUX_MMC0_D2_CFG_REG_PUPDSEL_MASK                                 (0x00000200U)
#define CSL_IOMUX_MMC0_D2_CFG_REG_PUPDSEL_SHIFT                                (0x00000009U)
#define CSL_IOMUX_MMC0_D2_CFG_REG_PUPDSEL_RESETVAL                             (0x00000000U)
#define CSL_IOMUX_MMC0_D2_CFG_REG_PUPDSEL_MAX                                  (0x00000001U)

#define CSL_IOMUX_MMC0_D2_CFG_REG_SC1_MASK                                     (0x00000400U)
#define CSL_IOMUX_MMC0_D2_CFG_REG_SC1_SHIFT                                    (0x0000000AU)
#define CSL_IOMUX_MMC0_D2_CFG_REG_SC1_RESETVAL                                 (0x00000001U)
#define CSL_IOMUX_MMC0_D2_CFG_REG_SC1_MAX                                      (0x00000001U)

#define CSL_IOMUX_MMC0_D2_CFG_REG_GPIO_SEL_MASK                                (0x00030000U)
#define CSL_IOMUX_MMC0_D2_CFG_REG_GPIO_SEL_SHIFT                               (0x00000010U)
#define CSL_IOMUX_MMC0_D2_CFG_REG_GPIO_SEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_MMC0_D2_CFG_REG_GPIO_SEL_MAX                                 (0x00000003U)

#define CSL_IOMUX_MMC0_D2_CFG_REG_QUAL_SEL_MASK                                (0x000C0000U)
#define CSL_IOMUX_MMC0_D2_CFG_REG_QUAL_SEL_SHIFT                               (0x00000012U)
#define CSL_IOMUX_MMC0_D2_CFG_REG_QUAL_SEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_MMC0_D2_CFG_REG_QUAL_SEL_MAX                                 (0x00000003U)

#define CSL_IOMUX_MMC0_D2_CFG_REG_INP_INV_SEL_MASK                             (0x00100000U)
#define CSL_IOMUX_MMC0_D2_CFG_REG_INP_INV_SEL_SHIFT                            (0x00000014U)
#define CSL_IOMUX_MMC0_D2_CFG_REG_INP_INV_SEL_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_MMC0_D2_CFG_REG_INP_INV_SEL_MAX                              (0x00000001U)

#define CSL_IOMUX_MMC0_D2_CFG_REG_ICSSM_GPIO_SEL_MASK                          (0x00200000U)
#define CSL_IOMUX_MMC0_D2_CFG_REG_ICSSM_GPIO_SEL_SHIFT                         (0x00000015U)
#define CSL_IOMUX_MMC0_D2_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_MMC0_D2_CFG_REG_ICSSM_GPIO_SEL_MAX                           (0x00000001U)

#define CSL_IOMUX_MMC0_D2_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                     (0x00400000U)
#define CSL_IOMUX_MMC0_D2_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                    (0x00000016U)
#define CSL_IOMUX_MMC0_D2_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL                 (0x00000000U)
#define CSL_IOMUX_MMC0_D2_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                      (0x00000001U)

#define CSL_IOMUX_MMC0_D2_CFG_REG_HSMODE_MASK                                  (0x40000000U)
#define CSL_IOMUX_MMC0_D2_CFG_REG_HSMODE_SHIFT                                 (0x0000001EU)
#define CSL_IOMUX_MMC0_D2_CFG_REG_HSMODE_RESETVAL                              (0x00000000U)
#define CSL_IOMUX_MMC0_D2_CFG_REG_HSMODE_MAX                                   (0x00000001U)

#define CSL_IOMUX_MMC0_D2_CFG_REG_HSMASTER_MASK                                (0x80000000U)
#define CSL_IOMUX_MMC0_D2_CFG_REG_HSMASTER_SHIFT                               (0x0000001FU)
#define CSL_IOMUX_MMC0_D2_CFG_REG_HSMASTER_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_MMC0_D2_CFG_REG_HSMASTER_MAX                                 (0x00000001U)

#define CSL_IOMUX_MMC0_D2_CFG_REG_RESETVAL                                     (0x000005F7U)

/* MMC0_D3_CFG_REG */

#define CSL_IOMUX_MMC0_D3_CFG_REG_FUNC_SEL_MASK                                (0x0000000FU)
#define CSL_IOMUX_MMC0_D3_CFG_REG_FUNC_SEL_SHIFT                               (0x00000000U)
#define CSL_IOMUX_MMC0_D3_CFG_REG_FUNC_SEL_RESETVAL                            (0x00000007U)
#define CSL_IOMUX_MMC0_D3_CFG_REG_FUNC_SEL_MAX                                 (0x0000000FU)

#define CSL_IOMUX_MMC0_D3_CFG_REG_IE_OVERRIDE_CTRL_MASK                        (0x00000010U)
#define CSL_IOMUX_MMC0_D3_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                       (0x00000004U)
#define CSL_IOMUX_MMC0_D3_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_MMC0_D3_CFG_REG_IE_OVERRIDE_CTRL_MAX                         (0x00000001U)

#define CSL_IOMUX_MMC0_D3_CFG_REG_IE_OVERRIDE_MASK                             (0x00000020U)
#define CSL_IOMUX_MMC0_D3_CFG_REG_IE_OVERRIDE_SHIFT                            (0x00000005U)
#define CSL_IOMUX_MMC0_D3_CFG_REG_IE_OVERRIDE_RESETVAL                         (0x00000001U)
#define CSL_IOMUX_MMC0_D3_CFG_REG_IE_OVERRIDE_MAX                              (0x00000001U)

#define CSL_IOMUX_MMC0_D3_CFG_REG_OE_OVERRIDE_CTRL_MASK                        (0x00000040U)
#define CSL_IOMUX_MMC0_D3_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                       (0x00000006U)
#define CSL_IOMUX_MMC0_D3_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_MMC0_D3_CFG_REG_OE_OVERRIDE_CTRL_MAX                         (0x00000001U)

#define CSL_IOMUX_MMC0_D3_CFG_REG_OE_OVERRIDE_MASK                             (0x00000080U)
#define CSL_IOMUX_MMC0_D3_CFG_REG_OE_OVERRIDE_SHIFT                            (0x00000007U)
#define CSL_IOMUX_MMC0_D3_CFG_REG_OE_OVERRIDE_RESETVAL                         (0x00000001U)
#define CSL_IOMUX_MMC0_D3_CFG_REG_OE_OVERRIDE_MAX                              (0x00000001U)

#define CSL_IOMUX_MMC0_D3_CFG_REG_PI_MASK                                      (0x00000100U)
#define CSL_IOMUX_MMC0_D3_CFG_REG_PI_SHIFT                                     (0x00000008U)
#define CSL_IOMUX_MMC0_D3_CFG_REG_PI_RESETVAL                                  (0x00000001U)
#define CSL_IOMUX_MMC0_D3_CFG_REG_PI_MAX                                       (0x00000001U)

#define CSL_IOMUX_MMC0_D3_CFG_REG_PUPDSEL_MASK                                 (0x00000200U)
#define CSL_IOMUX_MMC0_D3_CFG_REG_PUPDSEL_SHIFT                                (0x00000009U)
#define CSL_IOMUX_MMC0_D3_CFG_REG_PUPDSEL_RESETVAL                             (0x00000000U)
#define CSL_IOMUX_MMC0_D3_CFG_REG_PUPDSEL_MAX                                  (0x00000001U)

#define CSL_IOMUX_MMC0_D3_CFG_REG_SC1_MASK                                     (0x00000400U)
#define CSL_IOMUX_MMC0_D3_CFG_REG_SC1_SHIFT                                    (0x0000000AU)
#define CSL_IOMUX_MMC0_D3_CFG_REG_SC1_RESETVAL                                 (0x00000001U)
#define CSL_IOMUX_MMC0_D3_CFG_REG_SC1_MAX                                      (0x00000001U)

#define CSL_IOMUX_MMC0_D3_CFG_REG_GPIO_SEL_MASK                                (0x00030000U)
#define CSL_IOMUX_MMC0_D3_CFG_REG_GPIO_SEL_SHIFT                               (0x00000010U)
#define CSL_IOMUX_MMC0_D3_CFG_REG_GPIO_SEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_MMC0_D3_CFG_REG_GPIO_SEL_MAX                                 (0x00000003U)

#define CSL_IOMUX_MMC0_D3_CFG_REG_QUAL_SEL_MASK                                (0x000C0000U)
#define CSL_IOMUX_MMC0_D3_CFG_REG_QUAL_SEL_SHIFT                               (0x00000012U)
#define CSL_IOMUX_MMC0_D3_CFG_REG_QUAL_SEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_MMC0_D3_CFG_REG_QUAL_SEL_MAX                                 (0x00000003U)

#define CSL_IOMUX_MMC0_D3_CFG_REG_INP_INV_SEL_MASK                             (0x00100000U)
#define CSL_IOMUX_MMC0_D3_CFG_REG_INP_INV_SEL_SHIFT                            (0x00000014U)
#define CSL_IOMUX_MMC0_D3_CFG_REG_INP_INV_SEL_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_MMC0_D3_CFG_REG_INP_INV_SEL_MAX                              (0x00000001U)

#define CSL_IOMUX_MMC0_D3_CFG_REG_ICSSM_GPIO_SEL_MASK                          (0x00200000U)
#define CSL_IOMUX_MMC0_D3_CFG_REG_ICSSM_GPIO_SEL_SHIFT                         (0x00000015U)
#define CSL_IOMUX_MMC0_D3_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_MMC0_D3_CFG_REG_ICSSM_GPIO_SEL_MAX                           (0x00000001U)

#define CSL_IOMUX_MMC0_D3_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                     (0x00400000U)
#define CSL_IOMUX_MMC0_D3_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                    (0x00000016U)
#define CSL_IOMUX_MMC0_D3_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL                 (0x00000000U)
#define CSL_IOMUX_MMC0_D3_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                      (0x00000001U)

#define CSL_IOMUX_MMC0_D3_CFG_REG_HSMODE_MASK                                  (0x40000000U)
#define CSL_IOMUX_MMC0_D3_CFG_REG_HSMODE_SHIFT                                 (0x0000001EU)
#define CSL_IOMUX_MMC0_D3_CFG_REG_HSMODE_RESETVAL                              (0x00000000U)
#define CSL_IOMUX_MMC0_D3_CFG_REG_HSMODE_MAX                                   (0x00000001U)

#define CSL_IOMUX_MMC0_D3_CFG_REG_HSMASTER_MASK                                (0x80000000U)
#define CSL_IOMUX_MMC0_D3_CFG_REG_HSMASTER_SHIFT                               (0x0000001FU)
#define CSL_IOMUX_MMC0_D3_CFG_REG_HSMASTER_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_MMC0_D3_CFG_REG_HSMASTER_MAX                                 (0x00000001U)

#define CSL_IOMUX_MMC0_D3_CFG_REG_RESETVAL                                     (0x000005F7U)

/* MMC0_WP_CFG_REG */

#define CSL_IOMUX_MMC0_WP_CFG_REG_FUNC_SEL_MASK                                (0x0000000FU)
#define CSL_IOMUX_MMC0_WP_CFG_REG_FUNC_SEL_SHIFT                               (0x00000000U)
#define CSL_IOMUX_MMC0_WP_CFG_REG_FUNC_SEL_RESETVAL                            (0x00000007U)
#define CSL_IOMUX_MMC0_WP_CFG_REG_FUNC_SEL_MAX                                 (0x0000000FU)

#define CSL_IOMUX_MMC0_WP_CFG_REG_IE_OVERRIDE_CTRL_MASK                        (0x00000010U)
#define CSL_IOMUX_MMC0_WP_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                       (0x00000004U)
#define CSL_IOMUX_MMC0_WP_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_MMC0_WP_CFG_REG_IE_OVERRIDE_CTRL_MAX                         (0x00000001U)

#define CSL_IOMUX_MMC0_WP_CFG_REG_IE_OVERRIDE_MASK                             (0x00000020U)
#define CSL_IOMUX_MMC0_WP_CFG_REG_IE_OVERRIDE_SHIFT                            (0x00000005U)
#define CSL_IOMUX_MMC0_WP_CFG_REG_IE_OVERRIDE_RESETVAL                         (0x00000001U)
#define CSL_IOMUX_MMC0_WP_CFG_REG_IE_OVERRIDE_MAX                              (0x00000001U)

#define CSL_IOMUX_MMC0_WP_CFG_REG_OE_OVERRIDE_CTRL_MASK                        (0x00000040U)
#define CSL_IOMUX_MMC0_WP_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                       (0x00000006U)
#define CSL_IOMUX_MMC0_WP_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_MMC0_WP_CFG_REG_OE_OVERRIDE_CTRL_MAX                         (0x00000001U)

#define CSL_IOMUX_MMC0_WP_CFG_REG_OE_OVERRIDE_MASK                             (0x00000080U)
#define CSL_IOMUX_MMC0_WP_CFG_REG_OE_OVERRIDE_SHIFT                            (0x00000007U)
#define CSL_IOMUX_MMC0_WP_CFG_REG_OE_OVERRIDE_RESETVAL                         (0x00000001U)
#define CSL_IOMUX_MMC0_WP_CFG_REG_OE_OVERRIDE_MAX                              (0x00000001U)

#define CSL_IOMUX_MMC0_WP_CFG_REG_PI_MASK                                      (0x00000100U)
#define CSL_IOMUX_MMC0_WP_CFG_REG_PI_SHIFT                                     (0x00000008U)
#define CSL_IOMUX_MMC0_WP_CFG_REG_PI_RESETVAL                                  (0x00000001U)
#define CSL_IOMUX_MMC0_WP_CFG_REG_PI_MAX                                       (0x00000001U)

#define CSL_IOMUX_MMC0_WP_CFG_REG_PUPDSEL_MASK                                 (0x00000200U)
#define CSL_IOMUX_MMC0_WP_CFG_REG_PUPDSEL_SHIFT                                (0x00000009U)
#define CSL_IOMUX_MMC0_WP_CFG_REG_PUPDSEL_RESETVAL                             (0x00000000U)
#define CSL_IOMUX_MMC0_WP_CFG_REG_PUPDSEL_MAX                                  (0x00000001U)

#define CSL_IOMUX_MMC0_WP_CFG_REG_SC1_MASK                                     (0x00000400U)
#define CSL_IOMUX_MMC0_WP_CFG_REG_SC1_SHIFT                                    (0x0000000AU)
#define CSL_IOMUX_MMC0_WP_CFG_REG_SC1_RESETVAL                                 (0x00000001U)
#define CSL_IOMUX_MMC0_WP_CFG_REG_SC1_MAX                                      (0x00000001U)

#define CSL_IOMUX_MMC0_WP_CFG_REG_GPIO_SEL_MASK                                (0x00030000U)
#define CSL_IOMUX_MMC0_WP_CFG_REG_GPIO_SEL_SHIFT                               (0x00000010U)
#define CSL_IOMUX_MMC0_WP_CFG_REG_GPIO_SEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_MMC0_WP_CFG_REG_GPIO_SEL_MAX                                 (0x00000003U)

#define CSL_IOMUX_MMC0_WP_CFG_REG_QUAL_SEL_MASK                                (0x000C0000U)
#define CSL_IOMUX_MMC0_WP_CFG_REG_QUAL_SEL_SHIFT                               (0x00000012U)
#define CSL_IOMUX_MMC0_WP_CFG_REG_QUAL_SEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_MMC0_WP_CFG_REG_QUAL_SEL_MAX                                 (0x00000003U)

#define CSL_IOMUX_MMC0_WP_CFG_REG_INP_INV_SEL_MASK                             (0x00100000U)
#define CSL_IOMUX_MMC0_WP_CFG_REG_INP_INV_SEL_SHIFT                            (0x00000014U)
#define CSL_IOMUX_MMC0_WP_CFG_REG_INP_INV_SEL_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_MMC0_WP_CFG_REG_INP_INV_SEL_MAX                              (0x00000001U)

#define CSL_IOMUX_MMC0_WP_CFG_REG_ICSSM_GPIO_SEL_MASK                          (0x00200000U)
#define CSL_IOMUX_MMC0_WP_CFG_REG_ICSSM_GPIO_SEL_SHIFT                         (0x00000015U)
#define CSL_IOMUX_MMC0_WP_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_MMC0_WP_CFG_REG_ICSSM_GPIO_SEL_MAX                           (0x00000001U)

#define CSL_IOMUX_MMC0_WP_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                     (0x00400000U)
#define CSL_IOMUX_MMC0_WP_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                    (0x00000016U)
#define CSL_IOMUX_MMC0_WP_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL                 (0x00000000U)
#define CSL_IOMUX_MMC0_WP_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                      (0x00000001U)

#define CSL_IOMUX_MMC0_WP_CFG_REG_HSMODE_MASK                                  (0x40000000U)
#define CSL_IOMUX_MMC0_WP_CFG_REG_HSMODE_SHIFT                                 (0x0000001EU)
#define CSL_IOMUX_MMC0_WP_CFG_REG_HSMODE_RESETVAL                              (0x00000000U)
#define CSL_IOMUX_MMC0_WP_CFG_REG_HSMODE_MAX                                   (0x00000001U)

#define CSL_IOMUX_MMC0_WP_CFG_REG_HSMASTER_MASK                                (0x80000000U)
#define CSL_IOMUX_MMC0_WP_CFG_REG_HSMASTER_SHIFT                               (0x0000001FU)
#define CSL_IOMUX_MMC0_WP_CFG_REG_HSMASTER_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_MMC0_WP_CFG_REG_HSMASTER_MAX                                 (0x00000001U)

#define CSL_IOMUX_MMC0_WP_CFG_REG_RESETVAL                                     (0x000005F7U)

/* MMC0_CD_CFG_REG */

#define CSL_IOMUX_MMC0_CD_CFG_REG_FUNC_SEL_MASK                                (0x0000000FU)
#define CSL_IOMUX_MMC0_CD_CFG_REG_FUNC_SEL_SHIFT                               (0x00000000U)
#define CSL_IOMUX_MMC0_CD_CFG_REG_FUNC_SEL_RESETVAL                            (0x00000007U)
#define CSL_IOMUX_MMC0_CD_CFG_REG_FUNC_SEL_MAX                                 (0x0000000FU)

#define CSL_IOMUX_MMC0_CD_CFG_REG_IE_OVERRIDE_CTRL_MASK                        (0x00000010U)
#define CSL_IOMUX_MMC0_CD_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                       (0x00000004U)
#define CSL_IOMUX_MMC0_CD_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_MMC0_CD_CFG_REG_IE_OVERRIDE_CTRL_MAX                         (0x00000001U)

#define CSL_IOMUX_MMC0_CD_CFG_REG_IE_OVERRIDE_MASK                             (0x00000020U)
#define CSL_IOMUX_MMC0_CD_CFG_REG_IE_OVERRIDE_SHIFT                            (0x00000005U)
#define CSL_IOMUX_MMC0_CD_CFG_REG_IE_OVERRIDE_RESETVAL                         (0x00000001U)
#define CSL_IOMUX_MMC0_CD_CFG_REG_IE_OVERRIDE_MAX                              (0x00000001U)

#define CSL_IOMUX_MMC0_CD_CFG_REG_OE_OVERRIDE_CTRL_MASK                        (0x00000040U)
#define CSL_IOMUX_MMC0_CD_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                       (0x00000006U)
#define CSL_IOMUX_MMC0_CD_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_MMC0_CD_CFG_REG_OE_OVERRIDE_CTRL_MAX                         (0x00000001U)

#define CSL_IOMUX_MMC0_CD_CFG_REG_OE_OVERRIDE_MASK                             (0x00000080U)
#define CSL_IOMUX_MMC0_CD_CFG_REG_OE_OVERRIDE_SHIFT                            (0x00000007U)
#define CSL_IOMUX_MMC0_CD_CFG_REG_OE_OVERRIDE_RESETVAL                         (0x00000001U)
#define CSL_IOMUX_MMC0_CD_CFG_REG_OE_OVERRIDE_MAX                              (0x00000001U)

#define CSL_IOMUX_MMC0_CD_CFG_REG_PI_MASK                                      (0x00000100U)
#define CSL_IOMUX_MMC0_CD_CFG_REG_PI_SHIFT                                     (0x00000008U)
#define CSL_IOMUX_MMC0_CD_CFG_REG_PI_RESETVAL                                  (0x00000001U)
#define CSL_IOMUX_MMC0_CD_CFG_REG_PI_MAX                                       (0x00000001U)

#define CSL_IOMUX_MMC0_CD_CFG_REG_PUPDSEL_MASK                                 (0x00000200U)
#define CSL_IOMUX_MMC0_CD_CFG_REG_PUPDSEL_SHIFT                                (0x00000009U)
#define CSL_IOMUX_MMC0_CD_CFG_REG_PUPDSEL_RESETVAL                             (0x00000000U)
#define CSL_IOMUX_MMC0_CD_CFG_REG_PUPDSEL_MAX                                  (0x00000001U)

#define CSL_IOMUX_MMC0_CD_CFG_REG_SC1_MASK                                     (0x00000400U)
#define CSL_IOMUX_MMC0_CD_CFG_REG_SC1_SHIFT                                    (0x0000000AU)
#define CSL_IOMUX_MMC0_CD_CFG_REG_SC1_RESETVAL                                 (0x00000001U)
#define CSL_IOMUX_MMC0_CD_CFG_REG_SC1_MAX                                      (0x00000001U)

#define CSL_IOMUX_MMC0_CD_CFG_REG_GPIO_SEL_MASK                                (0x00030000U)
#define CSL_IOMUX_MMC0_CD_CFG_REG_GPIO_SEL_SHIFT                               (0x00000010U)
#define CSL_IOMUX_MMC0_CD_CFG_REG_GPIO_SEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_MMC0_CD_CFG_REG_GPIO_SEL_MAX                                 (0x00000003U)

#define CSL_IOMUX_MMC0_CD_CFG_REG_QUAL_SEL_MASK                                (0x000C0000U)
#define CSL_IOMUX_MMC0_CD_CFG_REG_QUAL_SEL_SHIFT                               (0x00000012U)
#define CSL_IOMUX_MMC0_CD_CFG_REG_QUAL_SEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_MMC0_CD_CFG_REG_QUAL_SEL_MAX                                 (0x00000003U)

#define CSL_IOMUX_MMC0_CD_CFG_REG_INP_INV_SEL_MASK                             (0x00100000U)
#define CSL_IOMUX_MMC0_CD_CFG_REG_INP_INV_SEL_SHIFT                            (0x00000014U)
#define CSL_IOMUX_MMC0_CD_CFG_REG_INP_INV_SEL_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_MMC0_CD_CFG_REG_INP_INV_SEL_MAX                              (0x00000001U)

#define CSL_IOMUX_MMC0_CD_CFG_REG_ICSSM_GPIO_SEL_MASK                          (0x00200000U)
#define CSL_IOMUX_MMC0_CD_CFG_REG_ICSSM_GPIO_SEL_SHIFT                         (0x00000015U)
#define CSL_IOMUX_MMC0_CD_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_MMC0_CD_CFG_REG_ICSSM_GPIO_SEL_MAX                           (0x00000001U)

#define CSL_IOMUX_MMC0_CD_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                     (0x00400000U)
#define CSL_IOMUX_MMC0_CD_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                    (0x00000016U)
#define CSL_IOMUX_MMC0_CD_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL                 (0x00000000U)
#define CSL_IOMUX_MMC0_CD_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                      (0x00000001U)

#define CSL_IOMUX_MMC0_CD_CFG_REG_HSMODE_MASK                                  (0x40000000U)
#define CSL_IOMUX_MMC0_CD_CFG_REG_HSMODE_SHIFT                                 (0x0000001EU)
#define CSL_IOMUX_MMC0_CD_CFG_REG_HSMODE_RESETVAL                              (0x00000000U)
#define CSL_IOMUX_MMC0_CD_CFG_REG_HSMODE_MAX                                   (0x00000001U)

#define CSL_IOMUX_MMC0_CD_CFG_REG_HSMASTER_MASK                                (0x80000000U)
#define CSL_IOMUX_MMC0_CD_CFG_REG_HSMASTER_SHIFT                               (0x0000001FU)
#define CSL_IOMUX_MMC0_CD_CFG_REG_HSMASTER_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_MMC0_CD_CFG_REG_HSMASTER_MAX                                 (0x00000001U)

#define CSL_IOMUX_MMC0_CD_CFG_REG_RESETVAL                                     (0x000005F7U)

/* PR0_MDIO0_MDIO_CFG_REG */

#define CSL_IOMUX_PR0_MDIO0_MDIO_CFG_REG_FUNC_SEL_MASK                         (0x0000000FU)
#define CSL_IOMUX_PR0_MDIO0_MDIO_CFG_REG_FUNC_SEL_SHIFT                        (0x00000000U)
#define CSL_IOMUX_PR0_MDIO0_MDIO_CFG_REG_FUNC_SEL_RESETVAL                     (0x00000007U)
#define CSL_IOMUX_PR0_MDIO0_MDIO_CFG_REG_FUNC_SEL_MAX                          (0x0000000FU)

#define CSL_IOMUX_PR0_MDIO0_MDIO_CFG_REG_IE_OVERRIDE_CTRL_MASK                 (0x00000010U)
#define CSL_IOMUX_PR0_MDIO0_MDIO_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                (0x00000004U)
#define CSL_IOMUX_PR0_MDIO0_MDIO_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL             (0x00000001U)
#define CSL_IOMUX_PR0_MDIO0_MDIO_CFG_REG_IE_OVERRIDE_CTRL_MAX                  (0x00000001U)

#define CSL_IOMUX_PR0_MDIO0_MDIO_CFG_REG_IE_OVERRIDE_MASK                      (0x00000020U)
#define CSL_IOMUX_PR0_MDIO0_MDIO_CFG_REG_IE_OVERRIDE_SHIFT                     (0x00000005U)
#define CSL_IOMUX_PR0_MDIO0_MDIO_CFG_REG_IE_OVERRIDE_RESETVAL                  (0x00000001U)
#define CSL_IOMUX_PR0_MDIO0_MDIO_CFG_REG_IE_OVERRIDE_MAX                       (0x00000001U)

#define CSL_IOMUX_PR0_MDIO0_MDIO_CFG_REG_OE_OVERRIDE_CTRL_MASK                 (0x00000040U)
#define CSL_IOMUX_PR0_MDIO0_MDIO_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                (0x00000006U)
#define CSL_IOMUX_PR0_MDIO0_MDIO_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL             (0x00000001U)
#define CSL_IOMUX_PR0_MDIO0_MDIO_CFG_REG_OE_OVERRIDE_CTRL_MAX                  (0x00000001U)

#define CSL_IOMUX_PR0_MDIO0_MDIO_CFG_REG_OE_OVERRIDE_MASK                      (0x00000080U)
#define CSL_IOMUX_PR0_MDIO0_MDIO_CFG_REG_OE_OVERRIDE_SHIFT                     (0x00000007U)
#define CSL_IOMUX_PR0_MDIO0_MDIO_CFG_REG_OE_OVERRIDE_RESETVAL                  (0x00000001U)
#define CSL_IOMUX_PR0_MDIO0_MDIO_CFG_REG_OE_OVERRIDE_MAX                       (0x00000001U)

#define CSL_IOMUX_PR0_MDIO0_MDIO_CFG_REG_PI_MASK                               (0x00000100U)
#define CSL_IOMUX_PR0_MDIO0_MDIO_CFG_REG_PI_SHIFT                              (0x00000008U)
#define CSL_IOMUX_PR0_MDIO0_MDIO_CFG_REG_PI_RESETVAL                           (0x00000001U)
#define CSL_IOMUX_PR0_MDIO0_MDIO_CFG_REG_PI_MAX                                (0x00000001U)

#define CSL_IOMUX_PR0_MDIO0_MDIO_CFG_REG_PUPDSEL_MASK                          (0x00000200U)
#define CSL_IOMUX_PR0_MDIO0_MDIO_CFG_REG_PUPDSEL_SHIFT                         (0x00000009U)
#define CSL_IOMUX_PR0_MDIO0_MDIO_CFG_REG_PUPDSEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_PR0_MDIO0_MDIO_CFG_REG_PUPDSEL_MAX                           (0x00000001U)

#define CSL_IOMUX_PR0_MDIO0_MDIO_CFG_REG_SC1_MASK                              (0x00000400U)
#define CSL_IOMUX_PR0_MDIO0_MDIO_CFG_REG_SC1_SHIFT                             (0x0000000AU)
#define CSL_IOMUX_PR0_MDIO0_MDIO_CFG_REG_SC1_RESETVAL                          (0x00000001U)
#define CSL_IOMUX_PR0_MDIO0_MDIO_CFG_REG_SC1_MAX                               (0x00000001U)

#define CSL_IOMUX_PR0_MDIO0_MDIO_CFG_REG_GPIO_SEL_MASK                         (0x00030000U)
#define CSL_IOMUX_PR0_MDIO0_MDIO_CFG_REG_GPIO_SEL_SHIFT                        (0x00000010U)
#define CSL_IOMUX_PR0_MDIO0_MDIO_CFG_REG_GPIO_SEL_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_PR0_MDIO0_MDIO_CFG_REG_GPIO_SEL_MAX                          (0x00000003U)

#define CSL_IOMUX_PR0_MDIO0_MDIO_CFG_REG_QUAL_SEL_MASK                         (0x000C0000U)
#define CSL_IOMUX_PR0_MDIO0_MDIO_CFG_REG_QUAL_SEL_SHIFT                        (0x00000012U)
#define CSL_IOMUX_PR0_MDIO0_MDIO_CFG_REG_QUAL_SEL_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_PR0_MDIO0_MDIO_CFG_REG_QUAL_SEL_MAX                          (0x00000003U)

#define CSL_IOMUX_PR0_MDIO0_MDIO_CFG_REG_INP_INV_SEL_MASK                      (0x00100000U)
#define CSL_IOMUX_PR0_MDIO0_MDIO_CFG_REG_INP_INV_SEL_SHIFT                     (0x00000014U)
#define CSL_IOMUX_PR0_MDIO0_MDIO_CFG_REG_INP_INV_SEL_RESETVAL                  (0x00000000U)
#define CSL_IOMUX_PR0_MDIO0_MDIO_CFG_REG_INP_INV_SEL_MAX                       (0x00000001U)

#define CSL_IOMUX_PR0_MDIO0_MDIO_CFG_REG_ICSSM_GPIO_SEL_MASK                   (0x00200000U)
#define CSL_IOMUX_PR0_MDIO0_MDIO_CFG_REG_ICSSM_GPIO_SEL_SHIFT                  (0x00000015U)
#define CSL_IOMUX_PR0_MDIO0_MDIO_CFG_REG_ICSSM_GPIO_SEL_RESETVAL               (0x00000000U)
#define CSL_IOMUX_PR0_MDIO0_MDIO_CFG_REG_ICSSM_GPIO_SEL_MAX                    (0x00000001U)

#define CSL_IOMUX_PR0_MDIO0_MDIO_CFG_REG_SAFETY_OVERRIDE_SEL_MASK              (0x00400000U)
#define CSL_IOMUX_PR0_MDIO0_MDIO_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT             (0x00000016U)
#define CSL_IOMUX_PR0_MDIO0_MDIO_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL          (0x00000000U)
#define CSL_IOMUX_PR0_MDIO0_MDIO_CFG_REG_SAFETY_OVERRIDE_SEL_MAX               (0x00000001U)

#define CSL_IOMUX_PR0_MDIO0_MDIO_CFG_REG_HSMODE_MASK                           (0x40000000U)
#define CSL_IOMUX_PR0_MDIO0_MDIO_CFG_REG_HSMODE_SHIFT                          (0x0000001EU)
#define CSL_IOMUX_PR0_MDIO0_MDIO_CFG_REG_HSMODE_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_PR0_MDIO0_MDIO_CFG_REG_HSMODE_MAX                            (0x00000001U)

#define CSL_IOMUX_PR0_MDIO0_MDIO_CFG_REG_HSMASTER_MASK                         (0x80000000U)
#define CSL_IOMUX_PR0_MDIO0_MDIO_CFG_REG_HSMASTER_SHIFT                        (0x0000001FU)
#define CSL_IOMUX_PR0_MDIO0_MDIO_CFG_REG_HSMASTER_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_PR0_MDIO0_MDIO_CFG_REG_HSMASTER_MAX                          (0x00000001U)

#define CSL_IOMUX_PR0_MDIO0_MDIO_CFG_REG_RESETVAL                              (0x000005F7U)

/* PR0_MDIO0_MDC_CFG_REG */

#define CSL_IOMUX_PR0_MDIO0_MDC_CFG_REG_FUNC_SEL_MASK                          (0x0000000FU)
#define CSL_IOMUX_PR0_MDIO0_MDC_CFG_REG_FUNC_SEL_SHIFT                         (0x00000000U)
#define CSL_IOMUX_PR0_MDIO0_MDC_CFG_REG_FUNC_SEL_RESETVAL                      (0x00000007U)
#define CSL_IOMUX_PR0_MDIO0_MDC_CFG_REG_FUNC_SEL_MAX                           (0x0000000FU)

#define CSL_IOMUX_PR0_MDIO0_MDC_CFG_REG_IE_OVERRIDE_CTRL_MASK                  (0x00000010U)
#define CSL_IOMUX_PR0_MDIO0_MDC_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                 (0x00000004U)
#define CSL_IOMUX_PR0_MDIO0_MDC_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL              (0x00000001U)
#define CSL_IOMUX_PR0_MDIO0_MDC_CFG_REG_IE_OVERRIDE_CTRL_MAX                   (0x00000001U)

#define CSL_IOMUX_PR0_MDIO0_MDC_CFG_REG_IE_OVERRIDE_MASK                       (0x00000020U)
#define CSL_IOMUX_PR0_MDIO0_MDC_CFG_REG_IE_OVERRIDE_SHIFT                      (0x00000005U)
#define CSL_IOMUX_PR0_MDIO0_MDC_CFG_REG_IE_OVERRIDE_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_PR0_MDIO0_MDC_CFG_REG_IE_OVERRIDE_MAX                        (0x00000001U)

#define CSL_IOMUX_PR0_MDIO0_MDC_CFG_REG_OE_OVERRIDE_CTRL_MASK                  (0x00000040U)
#define CSL_IOMUX_PR0_MDIO0_MDC_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                 (0x00000006U)
#define CSL_IOMUX_PR0_MDIO0_MDC_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL              (0x00000001U)
#define CSL_IOMUX_PR0_MDIO0_MDC_CFG_REG_OE_OVERRIDE_CTRL_MAX                   (0x00000001U)

#define CSL_IOMUX_PR0_MDIO0_MDC_CFG_REG_OE_OVERRIDE_MASK                       (0x00000080U)
#define CSL_IOMUX_PR0_MDIO0_MDC_CFG_REG_OE_OVERRIDE_SHIFT                      (0x00000007U)
#define CSL_IOMUX_PR0_MDIO0_MDC_CFG_REG_OE_OVERRIDE_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_PR0_MDIO0_MDC_CFG_REG_OE_OVERRIDE_MAX                        (0x00000001U)

#define CSL_IOMUX_PR0_MDIO0_MDC_CFG_REG_PI_MASK                                (0x00000100U)
#define CSL_IOMUX_PR0_MDIO0_MDC_CFG_REG_PI_SHIFT                               (0x00000008U)
#define CSL_IOMUX_PR0_MDIO0_MDC_CFG_REG_PI_RESETVAL                            (0x00000001U)
#define CSL_IOMUX_PR0_MDIO0_MDC_CFG_REG_PI_MAX                                 (0x00000001U)

#define CSL_IOMUX_PR0_MDIO0_MDC_CFG_REG_PUPDSEL_MASK                           (0x00000200U)
#define CSL_IOMUX_PR0_MDIO0_MDC_CFG_REG_PUPDSEL_SHIFT                          (0x00000009U)
#define CSL_IOMUX_PR0_MDIO0_MDC_CFG_REG_PUPDSEL_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_PR0_MDIO0_MDC_CFG_REG_PUPDSEL_MAX                            (0x00000001U)

#define CSL_IOMUX_PR0_MDIO0_MDC_CFG_REG_SC1_MASK                               (0x00000400U)
#define CSL_IOMUX_PR0_MDIO0_MDC_CFG_REG_SC1_SHIFT                              (0x0000000AU)
#define CSL_IOMUX_PR0_MDIO0_MDC_CFG_REG_SC1_RESETVAL                           (0x00000001U)
#define CSL_IOMUX_PR0_MDIO0_MDC_CFG_REG_SC1_MAX                                (0x00000001U)

#define CSL_IOMUX_PR0_MDIO0_MDC_CFG_REG_GPIO_SEL_MASK                          (0x00030000U)
#define CSL_IOMUX_PR0_MDIO0_MDC_CFG_REG_GPIO_SEL_SHIFT                         (0x00000010U)
#define CSL_IOMUX_PR0_MDIO0_MDC_CFG_REG_GPIO_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_PR0_MDIO0_MDC_CFG_REG_GPIO_SEL_MAX                           (0x00000003U)

#define CSL_IOMUX_PR0_MDIO0_MDC_CFG_REG_QUAL_SEL_MASK                          (0x000C0000U)
#define CSL_IOMUX_PR0_MDIO0_MDC_CFG_REG_QUAL_SEL_SHIFT                         (0x00000012U)
#define CSL_IOMUX_PR0_MDIO0_MDC_CFG_REG_QUAL_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_PR0_MDIO0_MDC_CFG_REG_QUAL_SEL_MAX                           (0x00000003U)

#define CSL_IOMUX_PR0_MDIO0_MDC_CFG_REG_INP_INV_SEL_MASK                       (0x00100000U)
#define CSL_IOMUX_PR0_MDIO0_MDC_CFG_REG_INP_INV_SEL_SHIFT                      (0x00000014U)
#define CSL_IOMUX_PR0_MDIO0_MDC_CFG_REG_INP_INV_SEL_RESETVAL                   (0x00000000U)
#define CSL_IOMUX_PR0_MDIO0_MDC_CFG_REG_INP_INV_SEL_MAX                        (0x00000001U)

#define CSL_IOMUX_PR0_MDIO0_MDC_CFG_REG_ICSSM_GPIO_SEL_MASK                    (0x00200000U)
#define CSL_IOMUX_PR0_MDIO0_MDC_CFG_REG_ICSSM_GPIO_SEL_SHIFT                   (0x00000015U)
#define CSL_IOMUX_PR0_MDIO0_MDC_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                (0x00000000U)
#define CSL_IOMUX_PR0_MDIO0_MDC_CFG_REG_ICSSM_GPIO_SEL_MAX                     (0x00000001U)

#define CSL_IOMUX_PR0_MDIO0_MDC_CFG_REG_SAFETY_OVERRIDE_SEL_MASK               (0x00400000U)
#define CSL_IOMUX_PR0_MDIO0_MDC_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT              (0x00000016U)
#define CSL_IOMUX_PR0_MDIO0_MDC_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL           (0x00000000U)
#define CSL_IOMUX_PR0_MDIO0_MDC_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                (0x00000001U)

#define CSL_IOMUX_PR0_MDIO0_MDC_CFG_REG_HSMODE_MASK                            (0x40000000U)
#define CSL_IOMUX_PR0_MDIO0_MDC_CFG_REG_HSMODE_SHIFT                           (0x0000001EU)
#define CSL_IOMUX_PR0_MDIO0_MDC_CFG_REG_HSMODE_RESETVAL                        (0x00000000U)
#define CSL_IOMUX_PR0_MDIO0_MDC_CFG_REG_HSMODE_MAX                             (0x00000001U)

#define CSL_IOMUX_PR0_MDIO0_MDC_CFG_REG_HSMASTER_MASK                          (0x80000000U)
#define CSL_IOMUX_PR0_MDIO0_MDC_CFG_REG_HSMASTER_SHIFT                         (0x0000001FU)
#define CSL_IOMUX_PR0_MDIO0_MDC_CFG_REG_HSMASTER_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_PR0_MDIO0_MDC_CFG_REG_HSMASTER_MAX                           (0x00000001U)

#define CSL_IOMUX_PR0_MDIO0_MDC_CFG_REG_RESETVAL                               (0x000005F7U)

/* PR0_PRU0_GPO5_CFG_REG */

#define CSL_IOMUX_PR0_PRU0_GPO5_CFG_REG_FUNC_SEL_MASK                          (0x0000000FU)
#define CSL_IOMUX_PR0_PRU0_GPO5_CFG_REG_FUNC_SEL_SHIFT                         (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO5_CFG_REG_FUNC_SEL_RESETVAL                      (0x00000007U)
#define CSL_IOMUX_PR0_PRU0_GPO5_CFG_REG_FUNC_SEL_MAX                           (0x0000000FU)

#define CSL_IOMUX_PR0_PRU0_GPO5_CFG_REG_IE_OVERRIDE_CTRL_MASK                  (0x00000010U)
#define CSL_IOMUX_PR0_PRU0_GPO5_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                 (0x00000004U)
#define CSL_IOMUX_PR0_PRU0_GPO5_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL              (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO5_CFG_REG_IE_OVERRIDE_CTRL_MAX                   (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO5_CFG_REG_IE_OVERRIDE_MASK                       (0x00000020U)
#define CSL_IOMUX_PR0_PRU0_GPO5_CFG_REG_IE_OVERRIDE_SHIFT                      (0x00000005U)
#define CSL_IOMUX_PR0_PRU0_GPO5_CFG_REG_IE_OVERRIDE_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO5_CFG_REG_IE_OVERRIDE_MAX                        (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO5_CFG_REG_OE_OVERRIDE_CTRL_MASK                  (0x00000040U)
#define CSL_IOMUX_PR0_PRU0_GPO5_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                 (0x00000006U)
#define CSL_IOMUX_PR0_PRU0_GPO5_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL              (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO5_CFG_REG_OE_OVERRIDE_CTRL_MAX                   (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO5_CFG_REG_OE_OVERRIDE_MASK                       (0x00000080U)
#define CSL_IOMUX_PR0_PRU0_GPO5_CFG_REG_OE_OVERRIDE_SHIFT                      (0x00000007U)
#define CSL_IOMUX_PR0_PRU0_GPO5_CFG_REG_OE_OVERRIDE_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO5_CFG_REG_OE_OVERRIDE_MAX                        (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO5_CFG_REG_PI_MASK                                (0x00000100U)
#define CSL_IOMUX_PR0_PRU0_GPO5_CFG_REG_PI_SHIFT                               (0x00000008U)
#define CSL_IOMUX_PR0_PRU0_GPO5_CFG_REG_PI_RESETVAL                            (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO5_CFG_REG_PI_MAX                                 (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO5_CFG_REG_PUPDSEL_MASK                           (0x00000200U)
#define CSL_IOMUX_PR0_PRU0_GPO5_CFG_REG_PUPDSEL_SHIFT                          (0x00000009U)
#define CSL_IOMUX_PR0_PRU0_GPO5_CFG_REG_PUPDSEL_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO5_CFG_REG_PUPDSEL_MAX                            (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO5_CFG_REG_SC1_MASK                               (0x00000400U)
#define CSL_IOMUX_PR0_PRU0_GPO5_CFG_REG_SC1_SHIFT                              (0x0000000AU)
#define CSL_IOMUX_PR0_PRU0_GPO5_CFG_REG_SC1_RESETVAL                           (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO5_CFG_REG_SC1_MAX                                (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO5_CFG_REG_GPIO_SEL_MASK                          (0x00030000U)
#define CSL_IOMUX_PR0_PRU0_GPO5_CFG_REG_GPIO_SEL_SHIFT                         (0x00000010U)
#define CSL_IOMUX_PR0_PRU0_GPO5_CFG_REG_GPIO_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO5_CFG_REG_GPIO_SEL_MAX                           (0x00000003U)

#define CSL_IOMUX_PR0_PRU0_GPO5_CFG_REG_QUAL_SEL_MASK                          (0x000C0000U)
#define CSL_IOMUX_PR0_PRU0_GPO5_CFG_REG_QUAL_SEL_SHIFT                         (0x00000012U)
#define CSL_IOMUX_PR0_PRU0_GPO5_CFG_REG_QUAL_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO5_CFG_REG_QUAL_SEL_MAX                           (0x00000003U)

#define CSL_IOMUX_PR0_PRU0_GPO5_CFG_REG_INP_INV_SEL_MASK                       (0x00100000U)
#define CSL_IOMUX_PR0_PRU0_GPO5_CFG_REG_INP_INV_SEL_SHIFT                      (0x00000014U)
#define CSL_IOMUX_PR0_PRU0_GPO5_CFG_REG_INP_INV_SEL_RESETVAL                   (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO5_CFG_REG_INP_INV_SEL_MAX                        (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO5_CFG_REG_ICSSM_GPIO_SEL_MASK                    (0x00200000U)
#define CSL_IOMUX_PR0_PRU0_GPO5_CFG_REG_ICSSM_GPIO_SEL_SHIFT                   (0x00000015U)
#define CSL_IOMUX_PR0_PRU0_GPO5_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO5_CFG_REG_ICSSM_GPIO_SEL_MAX                     (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO5_CFG_REG_SAFETY_OVERRIDE_SEL_MASK               (0x00400000U)
#define CSL_IOMUX_PR0_PRU0_GPO5_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT              (0x00000016U)
#define CSL_IOMUX_PR0_PRU0_GPO5_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL           (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO5_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO5_CFG_REG_HSMODE_MASK                            (0x40000000U)
#define CSL_IOMUX_PR0_PRU0_GPO5_CFG_REG_HSMODE_SHIFT                           (0x0000001EU)
#define CSL_IOMUX_PR0_PRU0_GPO5_CFG_REG_HSMODE_RESETVAL                        (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO5_CFG_REG_HSMODE_MAX                             (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO5_CFG_REG_HSMASTER_MASK                          (0x80000000U)
#define CSL_IOMUX_PR0_PRU0_GPO5_CFG_REG_HSMASTER_SHIFT                         (0x0000001FU)
#define CSL_IOMUX_PR0_PRU0_GPO5_CFG_REG_HSMASTER_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO5_CFG_REG_HSMASTER_MAX                           (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO5_CFG_REG_RESETVAL                               (0x000005F7U)

/* PR0_PRU0_GPO9_CFG_REG */

#define CSL_IOMUX_PR0_PRU0_GPO9_CFG_REG_FUNC_SEL_MASK                          (0x0000000FU)
#define CSL_IOMUX_PR0_PRU0_GPO9_CFG_REG_FUNC_SEL_SHIFT                         (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO9_CFG_REG_FUNC_SEL_RESETVAL                      (0x00000007U)
#define CSL_IOMUX_PR0_PRU0_GPO9_CFG_REG_FUNC_SEL_MAX                           (0x0000000FU)

#define CSL_IOMUX_PR0_PRU0_GPO9_CFG_REG_IE_OVERRIDE_CTRL_MASK                  (0x00000010U)
#define CSL_IOMUX_PR0_PRU0_GPO9_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                 (0x00000004U)
#define CSL_IOMUX_PR0_PRU0_GPO9_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL              (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO9_CFG_REG_IE_OVERRIDE_CTRL_MAX                   (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO9_CFG_REG_IE_OVERRIDE_MASK                       (0x00000020U)
#define CSL_IOMUX_PR0_PRU0_GPO9_CFG_REG_IE_OVERRIDE_SHIFT                      (0x00000005U)
#define CSL_IOMUX_PR0_PRU0_GPO9_CFG_REG_IE_OVERRIDE_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO9_CFG_REG_IE_OVERRIDE_MAX                        (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO9_CFG_REG_OE_OVERRIDE_CTRL_MASK                  (0x00000040U)
#define CSL_IOMUX_PR0_PRU0_GPO9_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                 (0x00000006U)
#define CSL_IOMUX_PR0_PRU0_GPO9_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL              (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO9_CFG_REG_OE_OVERRIDE_CTRL_MAX                   (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO9_CFG_REG_OE_OVERRIDE_MASK                       (0x00000080U)
#define CSL_IOMUX_PR0_PRU0_GPO9_CFG_REG_OE_OVERRIDE_SHIFT                      (0x00000007U)
#define CSL_IOMUX_PR0_PRU0_GPO9_CFG_REG_OE_OVERRIDE_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO9_CFG_REG_OE_OVERRIDE_MAX                        (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO9_CFG_REG_PI_MASK                                (0x00000100U)
#define CSL_IOMUX_PR0_PRU0_GPO9_CFG_REG_PI_SHIFT                               (0x00000008U)
#define CSL_IOMUX_PR0_PRU0_GPO9_CFG_REG_PI_RESETVAL                            (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO9_CFG_REG_PI_MAX                                 (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO9_CFG_REG_PUPDSEL_MASK                           (0x00000200U)
#define CSL_IOMUX_PR0_PRU0_GPO9_CFG_REG_PUPDSEL_SHIFT                          (0x00000009U)
#define CSL_IOMUX_PR0_PRU0_GPO9_CFG_REG_PUPDSEL_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO9_CFG_REG_PUPDSEL_MAX                            (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO9_CFG_REG_SC1_MASK                               (0x00000400U)
#define CSL_IOMUX_PR0_PRU0_GPO9_CFG_REG_SC1_SHIFT                              (0x0000000AU)
#define CSL_IOMUX_PR0_PRU0_GPO9_CFG_REG_SC1_RESETVAL                           (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO9_CFG_REG_SC1_MAX                                (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO9_CFG_REG_GPIO_SEL_MASK                          (0x00030000U)
#define CSL_IOMUX_PR0_PRU0_GPO9_CFG_REG_GPIO_SEL_SHIFT                         (0x00000010U)
#define CSL_IOMUX_PR0_PRU0_GPO9_CFG_REG_GPIO_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO9_CFG_REG_GPIO_SEL_MAX                           (0x00000003U)

#define CSL_IOMUX_PR0_PRU0_GPO9_CFG_REG_QUAL_SEL_MASK                          (0x000C0000U)
#define CSL_IOMUX_PR0_PRU0_GPO9_CFG_REG_QUAL_SEL_SHIFT                         (0x00000012U)
#define CSL_IOMUX_PR0_PRU0_GPO9_CFG_REG_QUAL_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO9_CFG_REG_QUAL_SEL_MAX                           (0x00000003U)

#define CSL_IOMUX_PR0_PRU0_GPO9_CFG_REG_INP_INV_SEL_MASK                       (0x00100000U)
#define CSL_IOMUX_PR0_PRU0_GPO9_CFG_REG_INP_INV_SEL_SHIFT                      (0x00000014U)
#define CSL_IOMUX_PR0_PRU0_GPO9_CFG_REG_INP_INV_SEL_RESETVAL                   (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO9_CFG_REG_INP_INV_SEL_MAX                        (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO9_CFG_REG_ICSSM_GPIO_SEL_MASK                    (0x00200000U)
#define CSL_IOMUX_PR0_PRU0_GPO9_CFG_REG_ICSSM_GPIO_SEL_SHIFT                   (0x00000015U)
#define CSL_IOMUX_PR0_PRU0_GPO9_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO9_CFG_REG_ICSSM_GPIO_SEL_MAX                     (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO9_CFG_REG_SAFETY_OVERRIDE_SEL_MASK               (0x00400000U)
#define CSL_IOMUX_PR0_PRU0_GPO9_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT              (0x00000016U)
#define CSL_IOMUX_PR0_PRU0_GPO9_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL           (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO9_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO9_CFG_REG_HSMODE_MASK                            (0x40000000U)
#define CSL_IOMUX_PR0_PRU0_GPO9_CFG_REG_HSMODE_SHIFT                           (0x0000001EU)
#define CSL_IOMUX_PR0_PRU0_GPO9_CFG_REG_HSMODE_RESETVAL                        (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO9_CFG_REG_HSMODE_MAX                             (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO9_CFG_REG_HSMASTER_MASK                          (0x80000000U)
#define CSL_IOMUX_PR0_PRU0_GPO9_CFG_REG_HSMASTER_SHIFT                         (0x0000001FU)
#define CSL_IOMUX_PR0_PRU0_GPO9_CFG_REG_HSMASTER_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO9_CFG_REG_HSMASTER_MAX                           (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO9_CFG_REG_RESETVAL                               (0x000005F7U)

/* PR0_PRU0_GPO10_CFG_REG */

#define CSL_IOMUX_PR0_PRU0_GPO10_CFG_REG_FUNC_SEL_MASK                         (0x0000000FU)
#define CSL_IOMUX_PR0_PRU0_GPO10_CFG_REG_FUNC_SEL_SHIFT                        (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO10_CFG_REG_FUNC_SEL_RESETVAL                     (0x00000007U)
#define CSL_IOMUX_PR0_PRU0_GPO10_CFG_REG_FUNC_SEL_MAX                          (0x0000000FU)

#define CSL_IOMUX_PR0_PRU0_GPO10_CFG_REG_IE_OVERRIDE_CTRL_MASK                 (0x00000010U)
#define CSL_IOMUX_PR0_PRU0_GPO10_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                (0x00000004U)
#define CSL_IOMUX_PR0_PRU0_GPO10_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL             (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO10_CFG_REG_IE_OVERRIDE_CTRL_MAX                  (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO10_CFG_REG_IE_OVERRIDE_MASK                      (0x00000020U)
#define CSL_IOMUX_PR0_PRU0_GPO10_CFG_REG_IE_OVERRIDE_SHIFT                     (0x00000005U)
#define CSL_IOMUX_PR0_PRU0_GPO10_CFG_REG_IE_OVERRIDE_RESETVAL                  (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO10_CFG_REG_IE_OVERRIDE_MAX                       (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO10_CFG_REG_OE_OVERRIDE_CTRL_MASK                 (0x00000040U)
#define CSL_IOMUX_PR0_PRU0_GPO10_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                (0x00000006U)
#define CSL_IOMUX_PR0_PRU0_GPO10_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL             (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO10_CFG_REG_OE_OVERRIDE_CTRL_MAX                  (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO10_CFG_REG_OE_OVERRIDE_MASK                      (0x00000080U)
#define CSL_IOMUX_PR0_PRU0_GPO10_CFG_REG_OE_OVERRIDE_SHIFT                     (0x00000007U)
#define CSL_IOMUX_PR0_PRU0_GPO10_CFG_REG_OE_OVERRIDE_RESETVAL                  (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO10_CFG_REG_OE_OVERRIDE_MAX                       (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO10_CFG_REG_PI_MASK                               (0x00000100U)
#define CSL_IOMUX_PR0_PRU0_GPO10_CFG_REG_PI_SHIFT                              (0x00000008U)
#define CSL_IOMUX_PR0_PRU0_GPO10_CFG_REG_PI_RESETVAL                           (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO10_CFG_REG_PI_MAX                                (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO10_CFG_REG_PUPDSEL_MASK                          (0x00000200U)
#define CSL_IOMUX_PR0_PRU0_GPO10_CFG_REG_PUPDSEL_SHIFT                         (0x00000009U)
#define CSL_IOMUX_PR0_PRU0_GPO10_CFG_REG_PUPDSEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO10_CFG_REG_PUPDSEL_MAX                           (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO10_CFG_REG_SC1_MASK                              (0x00000400U)
#define CSL_IOMUX_PR0_PRU0_GPO10_CFG_REG_SC1_SHIFT                             (0x0000000AU)
#define CSL_IOMUX_PR0_PRU0_GPO10_CFG_REG_SC1_RESETVAL                          (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO10_CFG_REG_SC1_MAX                               (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO10_CFG_REG_GPIO_SEL_MASK                         (0x00030000U)
#define CSL_IOMUX_PR0_PRU0_GPO10_CFG_REG_GPIO_SEL_SHIFT                        (0x00000010U)
#define CSL_IOMUX_PR0_PRU0_GPO10_CFG_REG_GPIO_SEL_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO10_CFG_REG_GPIO_SEL_MAX                          (0x00000003U)

#define CSL_IOMUX_PR0_PRU0_GPO10_CFG_REG_QUAL_SEL_MASK                         (0x000C0000U)
#define CSL_IOMUX_PR0_PRU0_GPO10_CFG_REG_QUAL_SEL_SHIFT                        (0x00000012U)
#define CSL_IOMUX_PR0_PRU0_GPO10_CFG_REG_QUAL_SEL_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO10_CFG_REG_QUAL_SEL_MAX                          (0x00000003U)

#define CSL_IOMUX_PR0_PRU0_GPO10_CFG_REG_INP_INV_SEL_MASK                      (0x00100000U)
#define CSL_IOMUX_PR0_PRU0_GPO10_CFG_REG_INP_INV_SEL_SHIFT                     (0x00000014U)
#define CSL_IOMUX_PR0_PRU0_GPO10_CFG_REG_INP_INV_SEL_RESETVAL                  (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO10_CFG_REG_INP_INV_SEL_MAX                       (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO10_CFG_REG_ICSSM_GPIO_SEL_MASK                   (0x00200000U)
#define CSL_IOMUX_PR0_PRU0_GPO10_CFG_REG_ICSSM_GPIO_SEL_SHIFT                  (0x00000015U)
#define CSL_IOMUX_PR0_PRU0_GPO10_CFG_REG_ICSSM_GPIO_SEL_RESETVAL               (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO10_CFG_REG_ICSSM_GPIO_SEL_MAX                    (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO10_CFG_REG_SAFETY_OVERRIDE_SEL_MASK              (0x00400000U)
#define CSL_IOMUX_PR0_PRU0_GPO10_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT             (0x00000016U)
#define CSL_IOMUX_PR0_PRU0_GPO10_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL          (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO10_CFG_REG_SAFETY_OVERRIDE_SEL_MAX               (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO10_CFG_REG_HSMODE_MASK                           (0x40000000U)
#define CSL_IOMUX_PR0_PRU0_GPO10_CFG_REG_HSMODE_SHIFT                          (0x0000001EU)
#define CSL_IOMUX_PR0_PRU0_GPO10_CFG_REG_HSMODE_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO10_CFG_REG_HSMODE_MAX                            (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO10_CFG_REG_HSMASTER_MASK                         (0x80000000U)
#define CSL_IOMUX_PR0_PRU0_GPO10_CFG_REG_HSMASTER_SHIFT                        (0x0000001FU)
#define CSL_IOMUX_PR0_PRU0_GPO10_CFG_REG_HSMASTER_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO10_CFG_REG_HSMASTER_MAX                          (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO10_CFG_REG_RESETVAL                              (0x000005F7U)

/* PR0_PRU0_GPO8_CFG_REG */

#define CSL_IOMUX_PR0_PRU0_GPO8_CFG_REG_FUNC_SEL_MASK                          (0x0000000FU)
#define CSL_IOMUX_PR0_PRU0_GPO8_CFG_REG_FUNC_SEL_SHIFT                         (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO8_CFG_REG_FUNC_SEL_RESETVAL                      (0x00000007U)
#define CSL_IOMUX_PR0_PRU0_GPO8_CFG_REG_FUNC_SEL_MAX                           (0x0000000FU)

#define CSL_IOMUX_PR0_PRU0_GPO8_CFG_REG_IE_OVERRIDE_CTRL_MASK                  (0x00000010U)
#define CSL_IOMUX_PR0_PRU0_GPO8_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                 (0x00000004U)
#define CSL_IOMUX_PR0_PRU0_GPO8_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL              (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO8_CFG_REG_IE_OVERRIDE_CTRL_MAX                   (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO8_CFG_REG_IE_OVERRIDE_MASK                       (0x00000020U)
#define CSL_IOMUX_PR0_PRU0_GPO8_CFG_REG_IE_OVERRIDE_SHIFT                      (0x00000005U)
#define CSL_IOMUX_PR0_PRU0_GPO8_CFG_REG_IE_OVERRIDE_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO8_CFG_REG_IE_OVERRIDE_MAX                        (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO8_CFG_REG_OE_OVERRIDE_CTRL_MASK                  (0x00000040U)
#define CSL_IOMUX_PR0_PRU0_GPO8_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                 (0x00000006U)
#define CSL_IOMUX_PR0_PRU0_GPO8_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL              (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO8_CFG_REG_OE_OVERRIDE_CTRL_MAX                   (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO8_CFG_REG_OE_OVERRIDE_MASK                       (0x00000080U)
#define CSL_IOMUX_PR0_PRU0_GPO8_CFG_REG_OE_OVERRIDE_SHIFT                      (0x00000007U)
#define CSL_IOMUX_PR0_PRU0_GPO8_CFG_REG_OE_OVERRIDE_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO8_CFG_REG_OE_OVERRIDE_MAX                        (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO8_CFG_REG_PI_MASK                                (0x00000100U)
#define CSL_IOMUX_PR0_PRU0_GPO8_CFG_REG_PI_SHIFT                               (0x00000008U)
#define CSL_IOMUX_PR0_PRU0_GPO8_CFG_REG_PI_RESETVAL                            (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO8_CFG_REG_PI_MAX                                 (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO8_CFG_REG_PUPDSEL_MASK                           (0x00000200U)
#define CSL_IOMUX_PR0_PRU0_GPO8_CFG_REG_PUPDSEL_SHIFT                          (0x00000009U)
#define CSL_IOMUX_PR0_PRU0_GPO8_CFG_REG_PUPDSEL_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO8_CFG_REG_PUPDSEL_MAX                            (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO8_CFG_REG_SC1_MASK                               (0x00000400U)
#define CSL_IOMUX_PR0_PRU0_GPO8_CFG_REG_SC1_SHIFT                              (0x0000000AU)
#define CSL_IOMUX_PR0_PRU0_GPO8_CFG_REG_SC1_RESETVAL                           (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO8_CFG_REG_SC1_MAX                                (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO8_CFG_REG_GPIO_SEL_MASK                          (0x00030000U)
#define CSL_IOMUX_PR0_PRU0_GPO8_CFG_REG_GPIO_SEL_SHIFT                         (0x00000010U)
#define CSL_IOMUX_PR0_PRU0_GPO8_CFG_REG_GPIO_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO8_CFG_REG_GPIO_SEL_MAX                           (0x00000003U)

#define CSL_IOMUX_PR0_PRU0_GPO8_CFG_REG_QUAL_SEL_MASK                          (0x000C0000U)
#define CSL_IOMUX_PR0_PRU0_GPO8_CFG_REG_QUAL_SEL_SHIFT                         (0x00000012U)
#define CSL_IOMUX_PR0_PRU0_GPO8_CFG_REG_QUAL_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO8_CFG_REG_QUAL_SEL_MAX                           (0x00000003U)

#define CSL_IOMUX_PR0_PRU0_GPO8_CFG_REG_INP_INV_SEL_MASK                       (0x00100000U)
#define CSL_IOMUX_PR0_PRU0_GPO8_CFG_REG_INP_INV_SEL_SHIFT                      (0x00000014U)
#define CSL_IOMUX_PR0_PRU0_GPO8_CFG_REG_INP_INV_SEL_RESETVAL                   (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO8_CFG_REG_INP_INV_SEL_MAX                        (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO8_CFG_REG_ICSSM_GPIO_SEL_MASK                    (0x00200000U)
#define CSL_IOMUX_PR0_PRU0_GPO8_CFG_REG_ICSSM_GPIO_SEL_SHIFT                   (0x00000015U)
#define CSL_IOMUX_PR0_PRU0_GPO8_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO8_CFG_REG_ICSSM_GPIO_SEL_MAX                     (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO8_CFG_REG_SAFETY_OVERRIDE_SEL_MASK               (0x00400000U)
#define CSL_IOMUX_PR0_PRU0_GPO8_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT              (0x00000016U)
#define CSL_IOMUX_PR0_PRU0_GPO8_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL           (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO8_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO8_CFG_REG_HSMODE_MASK                            (0x40000000U)
#define CSL_IOMUX_PR0_PRU0_GPO8_CFG_REG_HSMODE_SHIFT                           (0x0000001EU)
#define CSL_IOMUX_PR0_PRU0_GPO8_CFG_REG_HSMODE_RESETVAL                        (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO8_CFG_REG_HSMODE_MAX                             (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO8_CFG_REG_HSMASTER_MASK                          (0x80000000U)
#define CSL_IOMUX_PR0_PRU0_GPO8_CFG_REG_HSMASTER_SHIFT                         (0x0000001FU)
#define CSL_IOMUX_PR0_PRU0_GPO8_CFG_REG_HSMASTER_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO8_CFG_REG_HSMASTER_MAX                           (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO8_CFG_REG_RESETVAL                               (0x000005F7U)

/* PR0_PRU0_GPO6_CFG_REG */

#define CSL_IOMUX_PR0_PRU0_GPO6_CFG_REG_FUNC_SEL_MASK                          (0x0000000FU)
#define CSL_IOMUX_PR0_PRU0_GPO6_CFG_REG_FUNC_SEL_SHIFT                         (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO6_CFG_REG_FUNC_SEL_RESETVAL                      (0x00000007U)
#define CSL_IOMUX_PR0_PRU0_GPO6_CFG_REG_FUNC_SEL_MAX                           (0x0000000FU)

#define CSL_IOMUX_PR0_PRU0_GPO6_CFG_REG_IE_OVERRIDE_CTRL_MASK                  (0x00000010U)
#define CSL_IOMUX_PR0_PRU0_GPO6_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                 (0x00000004U)
#define CSL_IOMUX_PR0_PRU0_GPO6_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL              (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO6_CFG_REG_IE_OVERRIDE_CTRL_MAX                   (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO6_CFG_REG_IE_OVERRIDE_MASK                       (0x00000020U)
#define CSL_IOMUX_PR0_PRU0_GPO6_CFG_REG_IE_OVERRIDE_SHIFT                      (0x00000005U)
#define CSL_IOMUX_PR0_PRU0_GPO6_CFG_REG_IE_OVERRIDE_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO6_CFG_REG_IE_OVERRIDE_MAX                        (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO6_CFG_REG_OE_OVERRIDE_CTRL_MASK                  (0x00000040U)
#define CSL_IOMUX_PR0_PRU0_GPO6_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                 (0x00000006U)
#define CSL_IOMUX_PR0_PRU0_GPO6_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL              (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO6_CFG_REG_OE_OVERRIDE_CTRL_MAX                   (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO6_CFG_REG_OE_OVERRIDE_MASK                       (0x00000080U)
#define CSL_IOMUX_PR0_PRU0_GPO6_CFG_REG_OE_OVERRIDE_SHIFT                      (0x00000007U)
#define CSL_IOMUX_PR0_PRU0_GPO6_CFG_REG_OE_OVERRIDE_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO6_CFG_REG_OE_OVERRIDE_MAX                        (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO6_CFG_REG_PI_MASK                                (0x00000100U)
#define CSL_IOMUX_PR0_PRU0_GPO6_CFG_REG_PI_SHIFT                               (0x00000008U)
#define CSL_IOMUX_PR0_PRU0_GPO6_CFG_REG_PI_RESETVAL                            (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO6_CFG_REG_PI_MAX                                 (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO6_CFG_REG_PUPDSEL_MASK                           (0x00000200U)
#define CSL_IOMUX_PR0_PRU0_GPO6_CFG_REG_PUPDSEL_SHIFT                          (0x00000009U)
#define CSL_IOMUX_PR0_PRU0_GPO6_CFG_REG_PUPDSEL_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO6_CFG_REG_PUPDSEL_MAX                            (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO6_CFG_REG_SC1_MASK                               (0x00000400U)
#define CSL_IOMUX_PR0_PRU0_GPO6_CFG_REG_SC1_SHIFT                              (0x0000000AU)
#define CSL_IOMUX_PR0_PRU0_GPO6_CFG_REG_SC1_RESETVAL                           (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO6_CFG_REG_SC1_MAX                                (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO6_CFG_REG_GPIO_SEL_MASK                          (0x00030000U)
#define CSL_IOMUX_PR0_PRU0_GPO6_CFG_REG_GPIO_SEL_SHIFT                         (0x00000010U)
#define CSL_IOMUX_PR0_PRU0_GPO6_CFG_REG_GPIO_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO6_CFG_REG_GPIO_SEL_MAX                           (0x00000003U)

#define CSL_IOMUX_PR0_PRU0_GPO6_CFG_REG_QUAL_SEL_MASK                          (0x000C0000U)
#define CSL_IOMUX_PR0_PRU0_GPO6_CFG_REG_QUAL_SEL_SHIFT                         (0x00000012U)
#define CSL_IOMUX_PR0_PRU0_GPO6_CFG_REG_QUAL_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO6_CFG_REG_QUAL_SEL_MAX                           (0x00000003U)

#define CSL_IOMUX_PR0_PRU0_GPO6_CFG_REG_INP_INV_SEL_MASK                       (0x00100000U)
#define CSL_IOMUX_PR0_PRU0_GPO6_CFG_REG_INP_INV_SEL_SHIFT                      (0x00000014U)
#define CSL_IOMUX_PR0_PRU0_GPO6_CFG_REG_INP_INV_SEL_RESETVAL                   (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO6_CFG_REG_INP_INV_SEL_MAX                        (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO6_CFG_REG_ICSSM_GPIO_SEL_MASK                    (0x00200000U)
#define CSL_IOMUX_PR0_PRU0_GPO6_CFG_REG_ICSSM_GPIO_SEL_SHIFT                   (0x00000015U)
#define CSL_IOMUX_PR0_PRU0_GPO6_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO6_CFG_REG_ICSSM_GPIO_SEL_MAX                     (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO6_CFG_REG_SAFETY_OVERRIDE_SEL_MASK               (0x00400000U)
#define CSL_IOMUX_PR0_PRU0_GPO6_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT              (0x00000016U)
#define CSL_IOMUX_PR0_PRU0_GPO6_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL           (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO6_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO6_CFG_REG_HSMODE_MASK                            (0x40000000U)
#define CSL_IOMUX_PR0_PRU0_GPO6_CFG_REG_HSMODE_SHIFT                           (0x0000001EU)
#define CSL_IOMUX_PR0_PRU0_GPO6_CFG_REG_HSMODE_RESETVAL                        (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO6_CFG_REG_HSMODE_MAX                             (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO6_CFG_REG_HSMASTER_MASK                          (0x80000000U)
#define CSL_IOMUX_PR0_PRU0_GPO6_CFG_REG_HSMASTER_SHIFT                         (0x0000001FU)
#define CSL_IOMUX_PR0_PRU0_GPO6_CFG_REG_HSMASTER_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO6_CFG_REG_HSMASTER_MAX                           (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO6_CFG_REG_RESETVAL                               (0x000005F7U)

/* PR0_PRU0_GPO4_CFG_REG */

#define CSL_IOMUX_PR0_PRU0_GPO4_CFG_REG_FUNC_SEL_MASK                          (0x0000000FU)
#define CSL_IOMUX_PR0_PRU0_GPO4_CFG_REG_FUNC_SEL_SHIFT                         (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO4_CFG_REG_FUNC_SEL_RESETVAL                      (0x00000007U)
#define CSL_IOMUX_PR0_PRU0_GPO4_CFG_REG_FUNC_SEL_MAX                           (0x0000000FU)

#define CSL_IOMUX_PR0_PRU0_GPO4_CFG_REG_IE_OVERRIDE_CTRL_MASK                  (0x00000010U)
#define CSL_IOMUX_PR0_PRU0_GPO4_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                 (0x00000004U)
#define CSL_IOMUX_PR0_PRU0_GPO4_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL              (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO4_CFG_REG_IE_OVERRIDE_CTRL_MAX                   (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO4_CFG_REG_IE_OVERRIDE_MASK                       (0x00000020U)
#define CSL_IOMUX_PR0_PRU0_GPO4_CFG_REG_IE_OVERRIDE_SHIFT                      (0x00000005U)
#define CSL_IOMUX_PR0_PRU0_GPO4_CFG_REG_IE_OVERRIDE_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO4_CFG_REG_IE_OVERRIDE_MAX                        (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO4_CFG_REG_OE_OVERRIDE_CTRL_MASK                  (0x00000040U)
#define CSL_IOMUX_PR0_PRU0_GPO4_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                 (0x00000006U)
#define CSL_IOMUX_PR0_PRU0_GPO4_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL              (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO4_CFG_REG_OE_OVERRIDE_CTRL_MAX                   (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO4_CFG_REG_OE_OVERRIDE_MASK                       (0x00000080U)
#define CSL_IOMUX_PR0_PRU0_GPO4_CFG_REG_OE_OVERRIDE_SHIFT                      (0x00000007U)
#define CSL_IOMUX_PR0_PRU0_GPO4_CFG_REG_OE_OVERRIDE_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO4_CFG_REG_OE_OVERRIDE_MAX                        (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO4_CFG_REG_PI_MASK                                (0x00000100U)
#define CSL_IOMUX_PR0_PRU0_GPO4_CFG_REG_PI_SHIFT                               (0x00000008U)
#define CSL_IOMUX_PR0_PRU0_GPO4_CFG_REG_PI_RESETVAL                            (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO4_CFG_REG_PI_MAX                                 (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO4_CFG_REG_PUPDSEL_MASK                           (0x00000200U)
#define CSL_IOMUX_PR0_PRU0_GPO4_CFG_REG_PUPDSEL_SHIFT                          (0x00000009U)
#define CSL_IOMUX_PR0_PRU0_GPO4_CFG_REG_PUPDSEL_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO4_CFG_REG_PUPDSEL_MAX                            (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO4_CFG_REG_SC1_MASK                               (0x00000400U)
#define CSL_IOMUX_PR0_PRU0_GPO4_CFG_REG_SC1_SHIFT                              (0x0000000AU)
#define CSL_IOMUX_PR0_PRU0_GPO4_CFG_REG_SC1_RESETVAL                           (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO4_CFG_REG_SC1_MAX                                (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO4_CFG_REG_GPIO_SEL_MASK                          (0x00030000U)
#define CSL_IOMUX_PR0_PRU0_GPO4_CFG_REG_GPIO_SEL_SHIFT                         (0x00000010U)
#define CSL_IOMUX_PR0_PRU0_GPO4_CFG_REG_GPIO_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO4_CFG_REG_GPIO_SEL_MAX                           (0x00000003U)

#define CSL_IOMUX_PR0_PRU0_GPO4_CFG_REG_QUAL_SEL_MASK                          (0x000C0000U)
#define CSL_IOMUX_PR0_PRU0_GPO4_CFG_REG_QUAL_SEL_SHIFT                         (0x00000012U)
#define CSL_IOMUX_PR0_PRU0_GPO4_CFG_REG_QUAL_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO4_CFG_REG_QUAL_SEL_MAX                           (0x00000003U)

#define CSL_IOMUX_PR0_PRU0_GPO4_CFG_REG_INP_INV_SEL_MASK                       (0x00100000U)
#define CSL_IOMUX_PR0_PRU0_GPO4_CFG_REG_INP_INV_SEL_SHIFT                      (0x00000014U)
#define CSL_IOMUX_PR0_PRU0_GPO4_CFG_REG_INP_INV_SEL_RESETVAL                   (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO4_CFG_REG_INP_INV_SEL_MAX                        (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO4_CFG_REG_ICSSM_GPIO_SEL_MASK                    (0x00200000U)
#define CSL_IOMUX_PR0_PRU0_GPO4_CFG_REG_ICSSM_GPIO_SEL_SHIFT                   (0x00000015U)
#define CSL_IOMUX_PR0_PRU0_GPO4_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO4_CFG_REG_ICSSM_GPIO_SEL_MAX                     (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO4_CFG_REG_SAFETY_OVERRIDE_SEL_MASK               (0x00400000U)
#define CSL_IOMUX_PR0_PRU0_GPO4_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT              (0x00000016U)
#define CSL_IOMUX_PR0_PRU0_GPO4_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL           (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO4_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO4_CFG_REG_HSMODE_MASK                            (0x40000000U)
#define CSL_IOMUX_PR0_PRU0_GPO4_CFG_REG_HSMODE_SHIFT                           (0x0000001EU)
#define CSL_IOMUX_PR0_PRU0_GPO4_CFG_REG_HSMODE_RESETVAL                        (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO4_CFG_REG_HSMODE_MAX                             (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO4_CFG_REG_HSMASTER_MASK                          (0x80000000U)
#define CSL_IOMUX_PR0_PRU0_GPO4_CFG_REG_HSMASTER_SHIFT                         (0x0000001FU)
#define CSL_IOMUX_PR0_PRU0_GPO4_CFG_REG_HSMASTER_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO4_CFG_REG_HSMASTER_MAX                           (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO4_CFG_REG_RESETVAL                               (0x000005F7U)

/* PR0_PRU0_GPO0_CFG_REG */

#define CSL_IOMUX_PR0_PRU0_GPO0_CFG_REG_FUNC_SEL_MASK                          (0x0000000FU)
#define CSL_IOMUX_PR0_PRU0_GPO0_CFG_REG_FUNC_SEL_SHIFT                         (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO0_CFG_REG_FUNC_SEL_RESETVAL                      (0x00000007U)
#define CSL_IOMUX_PR0_PRU0_GPO0_CFG_REG_FUNC_SEL_MAX                           (0x0000000FU)

#define CSL_IOMUX_PR0_PRU0_GPO0_CFG_REG_IE_OVERRIDE_CTRL_MASK                  (0x00000010U)
#define CSL_IOMUX_PR0_PRU0_GPO0_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                 (0x00000004U)
#define CSL_IOMUX_PR0_PRU0_GPO0_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL              (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO0_CFG_REG_IE_OVERRIDE_CTRL_MAX                   (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO0_CFG_REG_IE_OVERRIDE_MASK                       (0x00000020U)
#define CSL_IOMUX_PR0_PRU0_GPO0_CFG_REG_IE_OVERRIDE_SHIFT                      (0x00000005U)
#define CSL_IOMUX_PR0_PRU0_GPO0_CFG_REG_IE_OVERRIDE_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO0_CFG_REG_IE_OVERRIDE_MAX                        (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO0_CFG_REG_OE_OVERRIDE_CTRL_MASK                  (0x00000040U)
#define CSL_IOMUX_PR0_PRU0_GPO0_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                 (0x00000006U)
#define CSL_IOMUX_PR0_PRU0_GPO0_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL              (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO0_CFG_REG_OE_OVERRIDE_CTRL_MAX                   (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO0_CFG_REG_OE_OVERRIDE_MASK                       (0x00000080U)
#define CSL_IOMUX_PR0_PRU0_GPO0_CFG_REG_OE_OVERRIDE_SHIFT                      (0x00000007U)
#define CSL_IOMUX_PR0_PRU0_GPO0_CFG_REG_OE_OVERRIDE_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO0_CFG_REG_OE_OVERRIDE_MAX                        (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO0_CFG_REG_PI_MASK                                (0x00000100U)
#define CSL_IOMUX_PR0_PRU0_GPO0_CFG_REG_PI_SHIFT                               (0x00000008U)
#define CSL_IOMUX_PR0_PRU0_GPO0_CFG_REG_PI_RESETVAL                            (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO0_CFG_REG_PI_MAX                                 (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO0_CFG_REG_PUPDSEL_MASK                           (0x00000200U)
#define CSL_IOMUX_PR0_PRU0_GPO0_CFG_REG_PUPDSEL_SHIFT                          (0x00000009U)
#define CSL_IOMUX_PR0_PRU0_GPO0_CFG_REG_PUPDSEL_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO0_CFG_REG_PUPDSEL_MAX                            (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO0_CFG_REG_SC1_MASK                               (0x00000400U)
#define CSL_IOMUX_PR0_PRU0_GPO0_CFG_REG_SC1_SHIFT                              (0x0000000AU)
#define CSL_IOMUX_PR0_PRU0_GPO0_CFG_REG_SC1_RESETVAL                           (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO0_CFG_REG_SC1_MAX                                (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO0_CFG_REG_GPIO_SEL_MASK                          (0x00030000U)
#define CSL_IOMUX_PR0_PRU0_GPO0_CFG_REG_GPIO_SEL_SHIFT                         (0x00000010U)
#define CSL_IOMUX_PR0_PRU0_GPO0_CFG_REG_GPIO_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO0_CFG_REG_GPIO_SEL_MAX                           (0x00000003U)

#define CSL_IOMUX_PR0_PRU0_GPO0_CFG_REG_QUAL_SEL_MASK                          (0x000C0000U)
#define CSL_IOMUX_PR0_PRU0_GPO0_CFG_REG_QUAL_SEL_SHIFT                         (0x00000012U)
#define CSL_IOMUX_PR0_PRU0_GPO0_CFG_REG_QUAL_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO0_CFG_REG_QUAL_SEL_MAX                           (0x00000003U)

#define CSL_IOMUX_PR0_PRU0_GPO0_CFG_REG_INP_INV_SEL_MASK                       (0x00100000U)
#define CSL_IOMUX_PR0_PRU0_GPO0_CFG_REG_INP_INV_SEL_SHIFT                      (0x00000014U)
#define CSL_IOMUX_PR0_PRU0_GPO0_CFG_REG_INP_INV_SEL_RESETVAL                   (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO0_CFG_REG_INP_INV_SEL_MAX                        (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO0_CFG_REG_ICSSM_GPIO_SEL_MASK                    (0x00200000U)
#define CSL_IOMUX_PR0_PRU0_GPO0_CFG_REG_ICSSM_GPIO_SEL_SHIFT                   (0x00000015U)
#define CSL_IOMUX_PR0_PRU0_GPO0_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO0_CFG_REG_ICSSM_GPIO_SEL_MAX                     (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO0_CFG_REG_SAFETY_OVERRIDE_SEL_MASK               (0x00400000U)
#define CSL_IOMUX_PR0_PRU0_GPO0_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT              (0x00000016U)
#define CSL_IOMUX_PR0_PRU0_GPO0_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL           (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO0_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO0_CFG_REG_HSMODE_MASK                            (0x40000000U)
#define CSL_IOMUX_PR0_PRU0_GPO0_CFG_REG_HSMODE_SHIFT                           (0x0000001EU)
#define CSL_IOMUX_PR0_PRU0_GPO0_CFG_REG_HSMODE_RESETVAL                        (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO0_CFG_REG_HSMODE_MAX                             (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO0_CFG_REG_HSMASTER_MASK                          (0x80000000U)
#define CSL_IOMUX_PR0_PRU0_GPO0_CFG_REG_HSMASTER_SHIFT                         (0x0000001FU)
#define CSL_IOMUX_PR0_PRU0_GPO0_CFG_REG_HSMASTER_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO0_CFG_REG_HSMASTER_MAX                           (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO0_CFG_REG_RESETVAL                               (0x000005F7U)

/* PR0_PRU0_GPO1_CFG_REG */

#define CSL_IOMUX_PR0_PRU0_GPO1_CFG_REG_FUNC_SEL_MASK                          (0x0000000FU)
#define CSL_IOMUX_PR0_PRU0_GPO1_CFG_REG_FUNC_SEL_SHIFT                         (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO1_CFG_REG_FUNC_SEL_RESETVAL                      (0x00000007U)
#define CSL_IOMUX_PR0_PRU0_GPO1_CFG_REG_FUNC_SEL_MAX                           (0x0000000FU)

#define CSL_IOMUX_PR0_PRU0_GPO1_CFG_REG_IE_OVERRIDE_CTRL_MASK                  (0x00000010U)
#define CSL_IOMUX_PR0_PRU0_GPO1_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                 (0x00000004U)
#define CSL_IOMUX_PR0_PRU0_GPO1_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL              (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO1_CFG_REG_IE_OVERRIDE_CTRL_MAX                   (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO1_CFG_REG_IE_OVERRIDE_MASK                       (0x00000020U)
#define CSL_IOMUX_PR0_PRU0_GPO1_CFG_REG_IE_OVERRIDE_SHIFT                      (0x00000005U)
#define CSL_IOMUX_PR0_PRU0_GPO1_CFG_REG_IE_OVERRIDE_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO1_CFG_REG_IE_OVERRIDE_MAX                        (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO1_CFG_REG_OE_OVERRIDE_CTRL_MASK                  (0x00000040U)
#define CSL_IOMUX_PR0_PRU0_GPO1_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                 (0x00000006U)
#define CSL_IOMUX_PR0_PRU0_GPO1_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL              (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO1_CFG_REG_OE_OVERRIDE_CTRL_MAX                   (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO1_CFG_REG_OE_OVERRIDE_MASK                       (0x00000080U)
#define CSL_IOMUX_PR0_PRU0_GPO1_CFG_REG_OE_OVERRIDE_SHIFT                      (0x00000007U)
#define CSL_IOMUX_PR0_PRU0_GPO1_CFG_REG_OE_OVERRIDE_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO1_CFG_REG_OE_OVERRIDE_MAX                        (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO1_CFG_REG_PI_MASK                                (0x00000100U)
#define CSL_IOMUX_PR0_PRU0_GPO1_CFG_REG_PI_SHIFT                               (0x00000008U)
#define CSL_IOMUX_PR0_PRU0_GPO1_CFG_REG_PI_RESETVAL                            (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO1_CFG_REG_PI_MAX                                 (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO1_CFG_REG_PUPDSEL_MASK                           (0x00000200U)
#define CSL_IOMUX_PR0_PRU0_GPO1_CFG_REG_PUPDSEL_SHIFT                          (0x00000009U)
#define CSL_IOMUX_PR0_PRU0_GPO1_CFG_REG_PUPDSEL_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO1_CFG_REG_PUPDSEL_MAX                            (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO1_CFG_REG_SC1_MASK                               (0x00000400U)
#define CSL_IOMUX_PR0_PRU0_GPO1_CFG_REG_SC1_SHIFT                              (0x0000000AU)
#define CSL_IOMUX_PR0_PRU0_GPO1_CFG_REG_SC1_RESETVAL                           (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO1_CFG_REG_SC1_MAX                                (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO1_CFG_REG_GPIO_SEL_MASK                          (0x00030000U)
#define CSL_IOMUX_PR0_PRU0_GPO1_CFG_REG_GPIO_SEL_SHIFT                         (0x00000010U)
#define CSL_IOMUX_PR0_PRU0_GPO1_CFG_REG_GPIO_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO1_CFG_REG_GPIO_SEL_MAX                           (0x00000003U)

#define CSL_IOMUX_PR0_PRU0_GPO1_CFG_REG_QUAL_SEL_MASK                          (0x000C0000U)
#define CSL_IOMUX_PR0_PRU0_GPO1_CFG_REG_QUAL_SEL_SHIFT                         (0x00000012U)
#define CSL_IOMUX_PR0_PRU0_GPO1_CFG_REG_QUAL_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO1_CFG_REG_QUAL_SEL_MAX                           (0x00000003U)

#define CSL_IOMUX_PR0_PRU0_GPO1_CFG_REG_INP_INV_SEL_MASK                       (0x00100000U)
#define CSL_IOMUX_PR0_PRU0_GPO1_CFG_REG_INP_INV_SEL_SHIFT                      (0x00000014U)
#define CSL_IOMUX_PR0_PRU0_GPO1_CFG_REG_INP_INV_SEL_RESETVAL                   (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO1_CFG_REG_INP_INV_SEL_MAX                        (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO1_CFG_REG_ICSSM_GPIO_SEL_MASK                    (0x00200000U)
#define CSL_IOMUX_PR0_PRU0_GPO1_CFG_REG_ICSSM_GPIO_SEL_SHIFT                   (0x00000015U)
#define CSL_IOMUX_PR0_PRU0_GPO1_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO1_CFG_REG_ICSSM_GPIO_SEL_MAX                     (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO1_CFG_REG_SAFETY_OVERRIDE_SEL_MASK               (0x00400000U)
#define CSL_IOMUX_PR0_PRU0_GPO1_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT              (0x00000016U)
#define CSL_IOMUX_PR0_PRU0_GPO1_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL           (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO1_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO1_CFG_REG_HSMODE_MASK                            (0x40000000U)
#define CSL_IOMUX_PR0_PRU0_GPO1_CFG_REG_HSMODE_SHIFT                           (0x0000001EU)
#define CSL_IOMUX_PR0_PRU0_GPO1_CFG_REG_HSMODE_RESETVAL                        (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO1_CFG_REG_HSMODE_MAX                             (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO1_CFG_REG_HSMASTER_MASK                          (0x80000000U)
#define CSL_IOMUX_PR0_PRU0_GPO1_CFG_REG_HSMASTER_SHIFT                         (0x0000001FU)
#define CSL_IOMUX_PR0_PRU0_GPO1_CFG_REG_HSMASTER_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO1_CFG_REG_HSMASTER_MAX                           (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO1_CFG_REG_RESETVAL                               (0x000005F7U)

/* PR0_PRU0_GPO2_CFG_REG */

#define CSL_IOMUX_PR0_PRU0_GPO2_CFG_REG_FUNC_SEL_MASK                          (0x0000000FU)
#define CSL_IOMUX_PR0_PRU0_GPO2_CFG_REG_FUNC_SEL_SHIFT                         (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO2_CFG_REG_FUNC_SEL_RESETVAL                      (0x00000007U)
#define CSL_IOMUX_PR0_PRU0_GPO2_CFG_REG_FUNC_SEL_MAX                           (0x0000000FU)

#define CSL_IOMUX_PR0_PRU0_GPO2_CFG_REG_IE_OVERRIDE_CTRL_MASK                  (0x00000010U)
#define CSL_IOMUX_PR0_PRU0_GPO2_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                 (0x00000004U)
#define CSL_IOMUX_PR0_PRU0_GPO2_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL              (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO2_CFG_REG_IE_OVERRIDE_CTRL_MAX                   (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO2_CFG_REG_IE_OVERRIDE_MASK                       (0x00000020U)
#define CSL_IOMUX_PR0_PRU0_GPO2_CFG_REG_IE_OVERRIDE_SHIFT                      (0x00000005U)
#define CSL_IOMUX_PR0_PRU0_GPO2_CFG_REG_IE_OVERRIDE_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO2_CFG_REG_IE_OVERRIDE_MAX                        (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO2_CFG_REG_OE_OVERRIDE_CTRL_MASK                  (0x00000040U)
#define CSL_IOMUX_PR0_PRU0_GPO2_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                 (0x00000006U)
#define CSL_IOMUX_PR0_PRU0_GPO2_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL              (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO2_CFG_REG_OE_OVERRIDE_CTRL_MAX                   (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO2_CFG_REG_OE_OVERRIDE_MASK                       (0x00000080U)
#define CSL_IOMUX_PR0_PRU0_GPO2_CFG_REG_OE_OVERRIDE_SHIFT                      (0x00000007U)
#define CSL_IOMUX_PR0_PRU0_GPO2_CFG_REG_OE_OVERRIDE_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO2_CFG_REG_OE_OVERRIDE_MAX                        (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO2_CFG_REG_PI_MASK                                (0x00000100U)
#define CSL_IOMUX_PR0_PRU0_GPO2_CFG_REG_PI_SHIFT                               (0x00000008U)
#define CSL_IOMUX_PR0_PRU0_GPO2_CFG_REG_PI_RESETVAL                            (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO2_CFG_REG_PI_MAX                                 (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO2_CFG_REG_PUPDSEL_MASK                           (0x00000200U)
#define CSL_IOMUX_PR0_PRU0_GPO2_CFG_REG_PUPDSEL_SHIFT                          (0x00000009U)
#define CSL_IOMUX_PR0_PRU0_GPO2_CFG_REG_PUPDSEL_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO2_CFG_REG_PUPDSEL_MAX                            (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO2_CFG_REG_SC1_MASK                               (0x00000400U)
#define CSL_IOMUX_PR0_PRU0_GPO2_CFG_REG_SC1_SHIFT                              (0x0000000AU)
#define CSL_IOMUX_PR0_PRU0_GPO2_CFG_REG_SC1_RESETVAL                           (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO2_CFG_REG_SC1_MAX                                (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO2_CFG_REG_GPIO_SEL_MASK                          (0x00030000U)
#define CSL_IOMUX_PR0_PRU0_GPO2_CFG_REG_GPIO_SEL_SHIFT                         (0x00000010U)
#define CSL_IOMUX_PR0_PRU0_GPO2_CFG_REG_GPIO_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO2_CFG_REG_GPIO_SEL_MAX                           (0x00000003U)

#define CSL_IOMUX_PR0_PRU0_GPO2_CFG_REG_QUAL_SEL_MASK                          (0x000C0000U)
#define CSL_IOMUX_PR0_PRU0_GPO2_CFG_REG_QUAL_SEL_SHIFT                         (0x00000012U)
#define CSL_IOMUX_PR0_PRU0_GPO2_CFG_REG_QUAL_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO2_CFG_REG_QUAL_SEL_MAX                           (0x00000003U)

#define CSL_IOMUX_PR0_PRU0_GPO2_CFG_REG_INP_INV_SEL_MASK                       (0x00100000U)
#define CSL_IOMUX_PR0_PRU0_GPO2_CFG_REG_INP_INV_SEL_SHIFT                      (0x00000014U)
#define CSL_IOMUX_PR0_PRU0_GPO2_CFG_REG_INP_INV_SEL_RESETVAL                   (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO2_CFG_REG_INP_INV_SEL_MAX                        (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO2_CFG_REG_ICSSM_GPIO_SEL_MASK                    (0x00200000U)
#define CSL_IOMUX_PR0_PRU0_GPO2_CFG_REG_ICSSM_GPIO_SEL_SHIFT                   (0x00000015U)
#define CSL_IOMUX_PR0_PRU0_GPO2_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO2_CFG_REG_ICSSM_GPIO_SEL_MAX                     (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO2_CFG_REG_SAFETY_OVERRIDE_SEL_MASK               (0x00400000U)
#define CSL_IOMUX_PR0_PRU0_GPO2_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT              (0x00000016U)
#define CSL_IOMUX_PR0_PRU0_GPO2_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL           (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO2_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO2_CFG_REG_HSMODE_MASK                            (0x40000000U)
#define CSL_IOMUX_PR0_PRU0_GPO2_CFG_REG_HSMODE_SHIFT                           (0x0000001EU)
#define CSL_IOMUX_PR0_PRU0_GPO2_CFG_REG_HSMODE_RESETVAL                        (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO2_CFG_REG_HSMODE_MAX                             (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO2_CFG_REG_HSMASTER_MASK                          (0x80000000U)
#define CSL_IOMUX_PR0_PRU0_GPO2_CFG_REG_HSMASTER_SHIFT                         (0x0000001FU)
#define CSL_IOMUX_PR0_PRU0_GPO2_CFG_REG_HSMASTER_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO2_CFG_REG_HSMASTER_MAX                           (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO2_CFG_REG_RESETVAL                               (0x000005F7U)

/* PR0_PRU0_GPO3_CFG_REG */

#define CSL_IOMUX_PR0_PRU0_GPO3_CFG_REG_FUNC_SEL_MASK                          (0x0000000FU)
#define CSL_IOMUX_PR0_PRU0_GPO3_CFG_REG_FUNC_SEL_SHIFT                         (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO3_CFG_REG_FUNC_SEL_RESETVAL                      (0x00000007U)
#define CSL_IOMUX_PR0_PRU0_GPO3_CFG_REG_FUNC_SEL_MAX                           (0x0000000FU)

#define CSL_IOMUX_PR0_PRU0_GPO3_CFG_REG_IE_OVERRIDE_CTRL_MASK                  (0x00000010U)
#define CSL_IOMUX_PR0_PRU0_GPO3_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                 (0x00000004U)
#define CSL_IOMUX_PR0_PRU0_GPO3_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL              (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO3_CFG_REG_IE_OVERRIDE_CTRL_MAX                   (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO3_CFG_REG_IE_OVERRIDE_MASK                       (0x00000020U)
#define CSL_IOMUX_PR0_PRU0_GPO3_CFG_REG_IE_OVERRIDE_SHIFT                      (0x00000005U)
#define CSL_IOMUX_PR0_PRU0_GPO3_CFG_REG_IE_OVERRIDE_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO3_CFG_REG_IE_OVERRIDE_MAX                        (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO3_CFG_REG_OE_OVERRIDE_CTRL_MASK                  (0x00000040U)
#define CSL_IOMUX_PR0_PRU0_GPO3_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                 (0x00000006U)
#define CSL_IOMUX_PR0_PRU0_GPO3_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL              (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO3_CFG_REG_OE_OVERRIDE_CTRL_MAX                   (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO3_CFG_REG_OE_OVERRIDE_MASK                       (0x00000080U)
#define CSL_IOMUX_PR0_PRU0_GPO3_CFG_REG_OE_OVERRIDE_SHIFT                      (0x00000007U)
#define CSL_IOMUX_PR0_PRU0_GPO3_CFG_REG_OE_OVERRIDE_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO3_CFG_REG_OE_OVERRIDE_MAX                        (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO3_CFG_REG_PI_MASK                                (0x00000100U)
#define CSL_IOMUX_PR0_PRU0_GPO3_CFG_REG_PI_SHIFT                               (0x00000008U)
#define CSL_IOMUX_PR0_PRU0_GPO3_CFG_REG_PI_RESETVAL                            (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO3_CFG_REG_PI_MAX                                 (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO3_CFG_REG_PUPDSEL_MASK                           (0x00000200U)
#define CSL_IOMUX_PR0_PRU0_GPO3_CFG_REG_PUPDSEL_SHIFT                          (0x00000009U)
#define CSL_IOMUX_PR0_PRU0_GPO3_CFG_REG_PUPDSEL_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO3_CFG_REG_PUPDSEL_MAX                            (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO3_CFG_REG_SC1_MASK                               (0x00000400U)
#define CSL_IOMUX_PR0_PRU0_GPO3_CFG_REG_SC1_SHIFT                              (0x0000000AU)
#define CSL_IOMUX_PR0_PRU0_GPO3_CFG_REG_SC1_RESETVAL                           (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO3_CFG_REG_SC1_MAX                                (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO3_CFG_REG_GPIO_SEL_MASK                          (0x00030000U)
#define CSL_IOMUX_PR0_PRU0_GPO3_CFG_REG_GPIO_SEL_SHIFT                         (0x00000010U)
#define CSL_IOMUX_PR0_PRU0_GPO3_CFG_REG_GPIO_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO3_CFG_REG_GPIO_SEL_MAX                           (0x00000003U)

#define CSL_IOMUX_PR0_PRU0_GPO3_CFG_REG_QUAL_SEL_MASK                          (0x000C0000U)
#define CSL_IOMUX_PR0_PRU0_GPO3_CFG_REG_QUAL_SEL_SHIFT                         (0x00000012U)
#define CSL_IOMUX_PR0_PRU0_GPO3_CFG_REG_QUAL_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO3_CFG_REG_QUAL_SEL_MAX                           (0x00000003U)

#define CSL_IOMUX_PR0_PRU0_GPO3_CFG_REG_INP_INV_SEL_MASK                       (0x00100000U)
#define CSL_IOMUX_PR0_PRU0_GPO3_CFG_REG_INP_INV_SEL_SHIFT                      (0x00000014U)
#define CSL_IOMUX_PR0_PRU0_GPO3_CFG_REG_INP_INV_SEL_RESETVAL                   (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO3_CFG_REG_INP_INV_SEL_MAX                        (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO3_CFG_REG_ICSSM_GPIO_SEL_MASK                    (0x00200000U)
#define CSL_IOMUX_PR0_PRU0_GPO3_CFG_REG_ICSSM_GPIO_SEL_SHIFT                   (0x00000015U)
#define CSL_IOMUX_PR0_PRU0_GPO3_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO3_CFG_REG_ICSSM_GPIO_SEL_MAX                     (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO3_CFG_REG_SAFETY_OVERRIDE_SEL_MASK               (0x00400000U)
#define CSL_IOMUX_PR0_PRU0_GPO3_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT              (0x00000016U)
#define CSL_IOMUX_PR0_PRU0_GPO3_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL           (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO3_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO3_CFG_REG_HSMODE_MASK                            (0x40000000U)
#define CSL_IOMUX_PR0_PRU0_GPO3_CFG_REG_HSMODE_SHIFT                           (0x0000001EU)
#define CSL_IOMUX_PR0_PRU0_GPO3_CFG_REG_HSMODE_RESETVAL                        (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO3_CFG_REG_HSMODE_MAX                             (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO3_CFG_REG_HSMASTER_MASK                          (0x80000000U)
#define CSL_IOMUX_PR0_PRU0_GPO3_CFG_REG_HSMASTER_SHIFT                         (0x0000001FU)
#define CSL_IOMUX_PR0_PRU0_GPO3_CFG_REG_HSMASTER_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO3_CFG_REG_HSMASTER_MAX                           (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO3_CFG_REG_RESETVAL                               (0x000005F7U)

/* PR0_PRU0_GPO16_CFG_REG */

#define CSL_IOMUX_PR0_PRU0_GPO16_CFG_REG_FUNC_SEL_MASK                         (0x0000000FU)
#define CSL_IOMUX_PR0_PRU0_GPO16_CFG_REG_FUNC_SEL_SHIFT                        (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO16_CFG_REG_FUNC_SEL_RESETVAL                     (0x00000007U)
#define CSL_IOMUX_PR0_PRU0_GPO16_CFG_REG_FUNC_SEL_MAX                          (0x0000000FU)

#define CSL_IOMUX_PR0_PRU0_GPO16_CFG_REG_IE_OVERRIDE_CTRL_MASK                 (0x00000010U)
#define CSL_IOMUX_PR0_PRU0_GPO16_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                (0x00000004U)
#define CSL_IOMUX_PR0_PRU0_GPO16_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL             (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO16_CFG_REG_IE_OVERRIDE_CTRL_MAX                  (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO16_CFG_REG_IE_OVERRIDE_MASK                      (0x00000020U)
#define CSL_IOMUX_PR0_PRU0_GPO16_CFG_REG_IE_OVERRIDE_SHIFT                     (0x00000005U)
#define CSL_IOMUX_PR0_PRU0_GPO16_CFG_REG_IE_OVERRIDE_RESETVAL                  (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO16_CFG_REG_IE_OVERRIDE_MAX                       (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO16_CFG_REG_OE_OVERRIDE_CTRL_MASK                 (0x00000040U)
#define CSL_IOMUX_PR0_PRU0_GPO16_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                (0x00000006U)
#define CSL_IOMUX_PR0_PRU0_GPO16_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL             (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO16_CFG_REG_OE_OVERRIDE_CTRL_MAX                  (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO16_CFG_REG_OE_OVERRIDE_MASK                      (0x00000080U)
#define CSL_IOMUX_PR0_PRU0_GPO16_CFG_REG_OE_OVERRIDE_SHIFT                     (0x00000007U)
#define CSL_IOMUX_PR0_PRU0_GPO16_CFG_REG_OE_OVERRIDE_RESETVAL                  (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO16_CFG_REG_OE_OVERRIDE_MAX                       (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO16_CFG_REG_PI_MASK                               (0x00000100U)
#define CSL_IOMUX_PR0_PRU0_GPO16_CFG_REG_PI_SHIFT                              (0x00000008U)
#define CSL_IOMUX_PR0_PRU0_GPO16_CFG_REG_PI_RESETVAL                           (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO16_CFG_REG_PI_MAX                                (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO16_CFG_REG_PUPDSEL_MASK                          (0x00000200U)
#define CSL_IOMUX_PR0_PRU0_GPO16_CFG_REG_PUPDSEL_SHIFT                         (0x00000009U)
#define CSL_IOMUX_PR0_PRU0_GPO16_CFG_REG_PUPDSEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO16_CFG_REG_PUPDSEL_MAX                           (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO16_CFG_REG_SC1_MASK                              (0x00000400U)
#define CSL_IOMUX_PR0_PRU0_GPO16_CFG_REG_SC1_SHIFT                             (0x0000000AU)
#define CSL_IOMUX_PR0_PRU0_GPO16_CFG_REG_SC1_RESETVAL                          (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO16_CFG_REG_SC1_MAX                               (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO16_CFG_REG_GPIO_SEL_MASK                         (0x00030000U)
#define CSL_IOMUX_PR0_PRU0_GPO16_CFG_REG_GPIO_SEL_SHIFT                        (0x00000010U)
#define CSL_IOMUX_PR0_PRU0_GPO16_CFG_REG_GPIO_SEL_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO16_CFG_REG_GPIO_SEL_MAX                          (0x00000003U)

#define CSL_IOMUX_PR0_PRU0_GPO16_CFG_REG_QUAL_SEL_MASK                         (0x000C0000U)
#define CSL_IOMUX_PR0_PRU0_GPO16_CFG_REG_QUAL_SEL_SHIFT                        (0x00000012U)
#define CSL_IOMUX_PR0_PRU0_GPO16_CFG_REG_QUAL_SEL_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO16_CFG_REG_QUAL_SEL_MAX                          (0x00000003U)

#define CSL_IOMUX_PR0_PRU0_GPO16_CFG_REG_INP_INV_SEL_MASK                      (0x00100000U)
#define CSL_IOMUX_PR0_PRU0_GPO16_CFG_REG_INP_INV_SEL_SHIFT                     (0x00000014U)
#define CSL_IOMUX_PR0_PRU0_GPO16_CFG_REG_INP_INV_SEL_RESETVAL                  (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO16_CFG_REG_INP_INV_SEL_MAX                       (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO16_CFG_REG_ICSSM_GPIO_SEL_MASK                   (0x00200000U)
#define CSL_IOMUX_PR0_PRU0_GPO16_CFG_REG_ICSSM_GPIO_SEL_SHIFT                  (0x00000015U)
#define CSL_IOMUX_PR0_PRU0_GPO16_CFG_REG_ICSSM_GPIO_SEL_RESETVAL               (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO16_CFG_REG_ICSSM_GPIO_SEL_MAX                    (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO16_CFG_REG_SAFETY_OVERRIDE_SEL_MASK              (0x00400000U)
#define CSL_IOMUX_PR0_PRU0_GPO16_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT             (0x00000016U)
#define CSL_IOMUX_PR0_PRU0_GPO16_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL          (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO16_CFG_REG_SAFETY_OVERRIDE_SEL_MAX               (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO16_CFG_REG_HSMODE_MASK                           (0x40000000U)
#define CSL_IOMUX_PR0_PRU0_GPO16_CFG_REG_HSMODE_SHIFT                          (0x0000001EU)
#define CSL_IOMUX_PR0_PRU0_GPO16_CFG_REG_HSMODE_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO16_CFG_REG_HSMODE_MAX                            (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO16_CFG_REG_HSMASTER_MASK                         (0x80000000U)
#define CSL_IOMUX_PR0_PRU0_GPO16_CFG_REG_HSMASTER_SHIFT                        (0x0000001FU)
#define CSL_IOMUX_PR0_PRU0_GPO16_CFG_REG_HSMASTER_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO16_CFG_REG_HSMASTER_MAX                          (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO16_CFG_REG_RESETVAL                              (0x000005F7U)

/* PR0_PRU0_GPO15_CFG_REG */

#define CSL_IOMUX_PR0_PRU0_GPO15_CFG_REG_FUNC_SEL_MASK                         (0x0000000FU)
#define CSL_IOMUX_PR0_PRU0_GPO15_CFG_REG_FUNC_SEL_SHIFT                        (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO15_CFG_REG_FUNC_SEL_RESETVAL                     (0x00000007U)
#define CSL_IOMUX_PR0_PRU0_GPO15_CFG_REG_FUNC_SEL_MAX                          (0x0000000FU)

#define CSL_IOMUX_PR0_PRU0_GPO15_CFG_REG_IE_OVERRIDE_CTRL_MASK                 (0x00000010U)
#define CSL_IOMUX_PR0_PRU0_GPO15_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                (0x00000004U)
#define CSL_IOMUX_PR0_PRU0_GPO15_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL             (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO15_CFG_REG_IE_OVERRIDE_CTRL_MAX                  (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO15_CFG_REG_IE_OVERRIDE_MASK                      (0x00000020U)
#define CSL_IOMUX_PR0_PRU0_GPO15_CFG_REG_IE_OVERRIDE_SHIFT                     (0x00000005U)
#define CSL_IOMUX_PR0_PRU0_GPO15_CFG_REG_IE_OVERRIDE_RESETVAL                  (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO15_CFG_REG_IE_OVERRIDE_MAX                       (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO15_CFG_REG_OE_OVERRIDE_CTRL_MASK                 (0x00000040U)
#define CSL_IOMUX_PR0_PRU0_GPO15_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                (0x00000006U)
#define CSL_IOMUX_PR0_PRU0_GPO15_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL             (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO15_CFG_REG_OE_OVERRIDE_CTRL_MAX                  (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO15_CFG_REG_OE_OVERRIDE_MASK                      (0x00000080U)
#define CSL_IOMUX_PR0_PRU0_GPO15_CFG_REG_OE_OVERRIDE_SHIFT                     (0x00000007U)
#define CSL_IOMUX_PR0_PRU0_GPO15_CFG_REG_OE_OVERRIDE_RESETVAL                  (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO15_CFG_REG_OE_OVERRIDE_MAX                       (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO15_CFG_REG_PI_MASK                               (0x00000100U)
#define CSL_IOMUX_PR0_PRU0_GPO15_CFG_REG_PI_SHIFT                              (0x00000008U)
#define CSL_IOMUX_PR0_PRU0_GPO15_CFG_REG_PI_RESETVAL                           (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO15_CFG_REG_PI_MAX                                (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO15_CFG_REG_PUPDSEL_MASK                          (0x00000200U)
#define CSL_IOMUX_PR0_PRU0_GPO15_CFG_REG_PUPDSEL_SHIFT                         (0x00000009U)
#define CSL_IOMUX_PR0_PRU0_GPO15_CFG_REG_PUPDSEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO15_CFG_REG_PUPDSEL_MAX                           (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO15_CFG_REG_SC1_MASK                              (0x00000400U)
#define CSL_IOMUX_PR0_PRU0_GPO15_CFG_REG_SC1_SHIFT                             (0x0000000AU)
#define CSL_IOMUX_PR0_PRU0_GPO15_CFG_REG_SC1_RESETVAL                          (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO15_CFG_REG_SC1_MAX                               (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO15_CFG_REG_GPIO_SEL_MASK                         (0x00030000U)
#define CSL_IOMUX_PR0_PRU0_GPO15_CFG_REG_GPIO_SEL_SHIFT                        (0x00000010U)
#define CSL_IOMUX_PR0_PRU0_GPO15_CFG_REG_GPIO_SEL_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO15_CFG_REG_GPIO_SEL_MAX                          (0x00000003U)

#define CSL_IOMUX_PR0_PRU0_GPO15_CFG_REG_QUAL_SEL_MASK                         (0x000C0000U)
#define CSL_IOMUX_PR0_PRU0_GPO15_CFG_REG_QUAL_SEL_SHIFT                        (0x00000012U)
#define CSL_IOMUX_PR0_PRU0_GPO15_CFG_REG_QUAL_SEL_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO15_CFG_REG_QUAL_SEL_MAX                          (0x00000003U)

#define CSL_IOMUX_PR0_PRU0_GPO15_CFG_REG_INP_INV_SEL_MASK                      (0x00100000U)
#define CSL_IOMUX_PR0_PRU0_GPO15_CFG_REG_INP_INV_SEL_SHIFT                     (0x00000014U)
#define CSL_IOMUX_PR0_PRU0_GPO15_CFG_REG_INP_INV_SEL_RESETVAL                  (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO15_CFG_REG_INP_INV_SEL_MAX                       (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO15_CFG_REG_ICSSM_GPIO_SEL_MASK                   (0x00200000U)
#define CSL_IOMUX_PR0_PRU0_GPO15_CFG_REG_ICSSM_GPIO_SEL_SHIFT                  (0x00000015U)
#define CSL_IOMUX_PR0_PRU0_GPO15_CFG_REG_ICSSM_GPIO_SEL_RESETVAL               (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO15_CFG_REG_ICSSM_GPIO_SEL_MAX                    (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO15_CFG_REG_SAFETY_OVERRIDE_SEL_MASK              (0x00400000U)
#define CSL_IOMUX_PR0_PRU0_GPO15_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT             (0x00000016U)
#define CSL_IOMUX_PR0_PRU0_GPO15_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL          (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO15_CFG_REG_SAFETY_OVERRIDE_SEL_MAX               (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO15_CFG_REG_HSMODE_MASK                           (0x40000000U)
#define CSL_IOMUX_PR0_PRU0_GPO15_CFG_REG_HSMODE_SHIFT                          (0x0000001EU)
#define CSL_IOMUX_PR0_PRU0_GPO15_CFG_REG_HSMODE_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO15_CFG_REG_HSMODE_MAX                            (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO15_CFG_REG_HSMASTER_MASK                         (0x80000000U)
#define CSL_IOMUX_PR0_PRU0_GPO15_CFG_REG_HSMASTER_SHIFT                        (0x0000001FU)
#define CSL_IOMUX_PR0_PRU0_GPO15_CFG_REG_HSMASTER_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO15_CFG_REG_HSMASTER_MAX                          (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO15_CFG_REG_RESETVAL                              (0x000005F7U)

/* PR0_PRU0_GPO11_CFG_REG */

#define CSL_IOMUX_PR0_PRU0_GPO11_CFG_REG_FUNC_SEL_MASK                         (0x0000000FU)
#define CSL_IOMUX_PR0_PRU0_GPO11_CFG_REG_FUNC_SEL_SHIFT                        (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO11_CFG_REG_FUNC_SEL_RESETVAL                     (0x00000007U)
#define CSL_IOMUX_PR0_PRU0_GPO11_CFG_REG_FUNC_SEL_MAX                          (0x0000000FU)

#define CSL_IOMUX_PR0_PRU0_GPO11_CFG_REG_IE_OVERRIDE_CTRL_MASK                 (0x00000010U)
#define CSL_IOMUX_PR0_PRU0_GPO11_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                (0x00000004U)
#define CSL_IOMUX_PR0_PRU0_GPO11_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL             (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO11_CFG_REG_IE_OVERRIDE_CTRL_MAX                  (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO11_CFG_REG_IE_OVERRIDE_MASK                      (0x00000020U)
#define CSL_IOMUX_PR0_PRU0_GPO11_CFG_REG_IE_OVERRIDE_SHIFT                     (0x00000005U)
#define CSL_IOMUX_PR0_PRU0_GPO11_CFG_REG_IE_OVERRIDE_RESETVAL                  (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO11_CFG_REG_IE_OVERRIDE_MAX                       (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO11_CFG_REG_OE_OVERRIDE_CTRL_MASK                 (0x00000040U)
#define CSL_IOMUX_PR0_PRU0_GPO11_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                (0x00000006U)
#define CSL_IOMUX_PR0_PRU0_GPO11_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL             (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO11_CFG_REG_OE_OVERRIDE_CTRL_MAX                  (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO11_CFG_REG_OE_OVERRIDE_MASK                      (0x00000080U)
#define CSL_IOMUX_PR0_PRU0_GPO11_CFG_REG_OE_OVERRIDE_SHIFT                     (0x00000007U)
#define CSL_IOMUX_PR0_PRU0_GPO11_CFG_REG_OE_OVERRIDE_RESETVAL                  (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO11_CFG_REG_OE_OVERRIDE_MAX                       (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO11_CFG_REG_PI_MASK                               (0x00000100U)
#define CSL_IOMUX_PR0_PRU0_GPO11_CFG_REG_PI_SHIFT                              (0x00000008U)
#define CSL_IOMUX_PR0_PRU0_GPO11_CFG_REG_PI_RESETVAL                           (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO11_CFG_REG_PI_MAX                                (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO11_CFG_REG_PUPDSEL_MASK                          (0x00000200U)
#define CSL_IOMUX_PR0_PRU0_GPO11_CFG_REG_PUPDSEL_SHIFT                         (0x00000009U)
#define CSL_IOMUX_PR0_PRU0_GPO11_CFG_REG_PUPDSEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO11_CFG_REG_PUPDSEL_MAX                           (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO11_CFG_REG_SC1_MASK                              (0x00000400U)
#define CSL_IOMUX_PR0_PRU0_GPO11_CFG_REG_SC1_SHIFT                             (0x0000000AU)
#define CSL_IOMUX_PR0_PRU0_GPO11_CFG_REG_SC1_RESETVAL                          (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO11_CFG_REG_SC1_MAX                               (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO11_CFG_REG_GPIO_SEL_MASK                         (0x00030000U)
#define CSL_IOMUX_PR0_PRU0_GPO11_CFG_REG_GPIO_SEL_SHIFT                        (0x00000010U)
#define CSL_IOMUX_PR0_PRU0_GPO11_CFG_REG_GPIO_SEL_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO11_CFG_REG_GPIO_SEL_MAX                          (0x00000003U)

#define CSL_IOMUX_PR0_PRU0_GPO11_CFG_REG_QUAL_SEL_MASK                         (0x000C0000U)
#define CSL_IOMUX_PR0_PRU0_GPO11_CFG_REG_QUAL_SEL_SHIFT                        (0x00000012U)
#define CSL_IOMUX_PR0_PRU0_GPO11_CFG_REG_QUAL_SEL_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO11_CFG_REG_QUAL_SEL_MAX                          (0x00000003U)

#define CSL_IOMUX_PR0_PRU0_GPO11_CFG_REG_INP_INV_SEL_MASK                      (0x00100000U)
#define CSL_IOMUX_PR0_PRU0_GPO11_CFG_REG_INP_INV_SEL_SHIFT                     (0x00000014U)
#define CSL_IOMUX_PR0_PRU0_GPO11_CFG_REG_INP_INV_SEL_RESETVAL                  (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO11_CFG_REG_INP_INV_SEL_MAX                       (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO11_CFG_REG_ICSSM_GPIO_SEL_MASK                   (0x00200000U)
#define CSL_IOMUX_PR0_PRU0_GPO11_CFG_REG_ICSSM_GPIO_SEL_SHIFT                  (0x00000015U)
#define CSL_IOMUX_PR0_PRU0_GPO11_CFG_REG_ICSSM_GPIO_SEL_RESETVAL               (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO11_CFG_REG_ICSSM_GPIO_SEL_MAX                    (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO11_CFG_REG_SAFETY_OVERRIDE_SEL_MASK              (0x00400000U)
#define CSL_IOMUX_PR0_PRU0_GPO11_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT             (0x00000016U)
#define CSL_IOMUX_PR0_PRU0_GPO11_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL          (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO11_CFG_REG_SAFETY_OVERRIDE_SEL_MAX               (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO11_CFG_REG_HSMODE_MASK                           (0x40000000U)
#define CSL_IOMUX_PR0_PRU0_GPO11_CFG_REG_HSMODE_SHIFT                          (0x0000001EU)
#define CSL_IOMUX_PR0_PRU0_GPO11_CFG_REG_HSMODE_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO11_CFG_REG_HSMODE_MAX                            (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO11_CFG_REG_HSMASTER_MASK                         (0x80000000U)
#define CSL_IOMUX_PR0_PRU0_GPO11_CFG_REG_HSMASTER_SHIFT                        (0x0000001FU)
#define CSL_IOMUX_PR0_PRU0_GPO11_CFG_REG_HSMASTER_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO11_CFG_REG_HSMASTER_MAX                          (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO11_CFG_REG_RESETVAL                              (0x000005F7U)

/* PR0_PRU0_GPO12_CFG_REG */

#define CSL_IOMUX_PR0_PRU0_GPO12_CFG_REG_FUNC_SEL_MASK                         (0x0000000FU)
#define CSL_IOMUX_PR0_PRU0_GPO12_CFG_REG_FUNC_SEL_SHIFT                        (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO12_CFG_REG_FUNC_SEL_RESETVAL                     (0x00000007U)
#define CSL_IOMUX_PR0_PRU0_GPO12_CFG_REG_FUNC_SEL_MAX                          (0x0000000FU)

#define CSL_IOMUX_PR0_PRU0_GPO12_CFG_REG_IE_OVERRIDE_CTRL_MASK                 (0x00000010U)
#define CSL_IOMUX_PR0_PRU0_GPO12_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                (0x00000004U)
#define CSL_IOMUX_PR0_PRU0_GPO12_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL             (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO12_CFG_REG_IE_OVERRIDE_CTRL_MAX                  (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO12_CFG_REG_IE_OVERRIDE_MASK                      (0x00000020U)
#define CSL_IOMUX_PR0_PRU0_GPO12_CFG_REG_IE_OVERRIDE_SHIFT                     (0x00000005U)
#define CSL_IOMUX_PR0_PRU0_GPO12_CFG_REG_IE_OVERRIDE_RESETVAL                  (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO12_CFG_REG_IE_OVERRIDE_MAX                       (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO12_CFG_REG_OE_OVERRIDE_CTRL_MASK                 (0x00000040U)
#define CSL_IOMUX_PR0_PRU0_GPO12_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                (0x00000006U)
#define CSL_IOMUX_PR0_PRU0_GPO12_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL             (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO12_CFG_REG_OE_OVERRIDE_CTRL_MAX                  (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO12_CFG_REG_OE_OVERRIDE_MASK                      (0x00000080U)
#define CSL_IOMUX_PR0_PRU0_GPO12_CFG_REG_OE_OVERRIDE_SHIFT                     (0x00000007U)
#define CSL_IOMUX_PR0_PRU0_GPO12_CFG_REG_OE_OVERRIDE_RESETVAL                  (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO12_CFG_REG_OE_OVERRIDE_MAX                       (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO12_CFG_REG_PI_MASK                               (0x00000100U)
#define CSL_IOMUX_PR0_PRU0_GPO12_CFG_REG_PI_SHIFT                              (0x00000008U)
#define CSL_IOMUX_PR0_PRU0_GPO12_CFG_REG_PI_RESETVAL                           (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO12_CFG_REG_PI_MAX                                (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO12_CFG_REG_PUPDSEL_MASK                          (0x00000200U)
#define CSL_IOMUX_PR0_PRU0_GPO12_CFG_REG_PUPDSEL_SHIFT                         (0x00000009U)
#define CSL_IOMUX_PR0_PRU0_GPO12_CFG_REG_PUPDSEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO12_CFG_REG_PUPDSEL_MAX                           (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO12_CFG_REG_SC1_MASK                              (0x00000400U)
#define CSL_IOMUX_PR0_PRU0_GPO12_CFG_REG_SC1_SHIFT                             (0x0000000AU)
#define CSL_IOMUX_PR0_PRU0_GPO12_CFG_REG_SC1_RESETVAL                          (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO12_CFG_REG_SC1_MAX                               (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO12_CFG_REG_GPIO_SEL_MASK                         (0x00030000U)
#define CSL_IOMUX_PR0_PRU0_GPO12_CFG_REG_GPIO_SEL_SHIFT                        (0x00000010U)
#define CSL_IOMUX_PR0_PRU0_GPO12_CFG_REG_GPIO_SEL_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO12_CFG_REG_GPIO_SEL_MAX                          (0x00000003U)

#define CSL_IOMUX_PR0_PRU0_GPO12_CFG_REG_QUAL_SEL_MASK                         (0x000C0000U)
#define CSL_IOMUX_PR0_PRU0_GPO12_CFG_REG_QUAL_SEL_SHIFT                        (0x00000012U)
#define CSL_IOMUX_PR0_PRU0_GPO12_CFG_REG_QUAL_SEL_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO12_CFG_REG_QUAL_SEL_MAX                          (0x00000003U)

#define CSL_IOMUX_PR0_PRU0_GPO12_CFG_REG_INP_INV_SEL_MASK                      (0x00100000U)
#define CSL_IOMUX_PR0_PRU0_GPO12_CFG_REG_INP_INV_SEL_SHIFT                     (0x00000014U)
#define CSL_IOMUX_PR0_PRU0_GPO12_CFG_REG_INP_INV_SEL_RESETVAL                  (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO12_CFG_REG_INP_INV_SEL_MAX                       (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO12_CFG_REG_ICSSM_GPIO_SEL_MASK                   (0x00200000U)
#define CSL_IOMUX_PR0_PRU0_GPO12_CFG_REG_ICSSM_GPIO_SEL_SHIFT                  (0x00000015U)
#define CSL_IOMUX_PR0_PRU0_GPO12_CFG_REG_ICSSM_GPIO_SEL_RESETVAL               (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO12_CFG_REG_ICSSM_GPIO_SEL_MAX                    (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO12_CFG_REG_SAFETY_OVERRIDE_SEL_MASK              (0x00400000U)
#define CSL_IOMUX_PR0_PRU0_GPO12_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT             (0x00000016U)
#define CSL_IOMUX_PR0_PRU0_GPO12_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL          (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO12_CFG_REG_SAFETY_OVERRIDE_SEL_MAX               (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO12_CFG_REG_HSMODE_MASK                           (0x40000000U)
#define CSL_IOMUX_PR0_PRU0_GPO12_CFG_REG_HSMODE_SHIFT                          (0x0000001EU)
#define CSL_IOMUX_PR0_PRU0_GPO12_CFG_REG_HSMODE_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO12_CFG_REG_HSMODE_MAX                            (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO12_CFG_REG_HSMASTER_MASK                         (0x80000000U)
#define CSL_IOMUX_PR0_PRU0_GPO12_CFG_REG_HSMASTER_SHIFT                        (0x0000001FU)
#define CSL_IOMUX_PR0_PRU0_GPO12_CFG_REG_HSMASTER_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO12_CFG_REG_HSMASTER_MAX                          (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO12_CFG_REG_RESETVAL                              (0x000005F7U)

/* PR0_PRU0_GPO13_CFG_REG */

#define CSL_IOMUX_PR0_PRU0_GPO13_CFG_REG_FUNC_SEL_MASK                         (0x0000000FU)
#define CSL_IOMUX_PR0_PRU0_GPO13_CFG_REG_FUNC_SEL_SHIFT                        (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO13_CFG_REG_FUNC_SEL_RESETVAL                     (0x00000007U)
#define CSL_IOMUX_PR0_PRU0_GPO13_CFG_REG_FUNC_SEL_MAX                          (0x0000000FU)

#define CSL_IOMUX_PR0_PRU0_GPO13_CFG_REG_IE_OVERRIDE_CTRL_MASK                 (0x00000010U)
#define CSL_IOMUX_PR0_PRU0_GPO13_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                (0x00000004U)
#define CSL_IOMUX_PR0_PRU0_GPO13_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL             (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO13_CFG_REG_IE_OVERRIDE_CTRL_MAX                  (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO13_CFG_REG_IE_OVERRIDE_MASK                      (0x00000020U)
#define CSL_IOMUX_PR0_PRU0_GPO13_CFG_REG_IE_OVERRIDE_SHIFT                     (0x00000005U)
#define CSL_IOMUX_PR0_PRU0_GPO13_CFG_REG_IE_OVERRIDE_RESETVAL                  (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO13_CFG_REG_IE_OVERRIDE_MAX                       (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO13_CFG_REG_OE_OVERRIDE_CTRL_MASK                 (0x00000040U)
#define CSL_IOMUX_PR0_PRU0_GPO13_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                (0x00000006U)
#define CSL_IOMUX_PR0_PRU0_GPO13_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL             (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO13_CFG_REG_OE_OVERRIDE_CTRL_MAX                  (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO13_CFG_REG_OE_OVERRIDE_MASK                      (0x00000080U)
#define CSL_IOMUX_PR0_PRU0_GPO13_CFG_REG_OE_OVERRIDE_SHIFT                     (0x00000007U)
#define CSL_IOMUX_PR0_PRU0_GPO13_CFG_REG_OE_OVERRIDE_RESETVAL                  (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO13_CFG_REG_OE_OVERRIDE_MAX                       (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO13_CFG_REG_PI_MASK                               (0x00000100U)
#define CSL_IOMUX_PR0_PRU0_GPO13_CFG_REG_PI_SHIFT                              (0x00000008U)
#define CSL_IOMUX_PR0_PRU0_GPO13_CFG_REG_PI_RESETVAL                           (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO13_CFG_REG_PI_MAX                                (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO13_CFG_REG_PUPDSEL_MASK                          (0x00000200U)
#define CSL_IOMUX_PR0_PRU0_GPO13_CFG_REG_PUPDSEL_SHIFT                         (0x00000009U)
#define CSL_IOMUX_PR0_PRU0_GPO13_CFG_REG_PUPDSEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO13_CFG_REG_PUPDSEL_MAX                           (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO13_CFG_REG_SC1_MASK                              (0x00000400U)
#define CSL_IOMUX_PR0_PRU0_GPO13_CFG_REG_SC1_SHIFT                             (0x0000000AU)
#define CSL_IOMUX_PR0_PRU0_GPO13_CFG_REG_SC1_RESETVAL                          (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO13_CFG_REG_SC1_MAX                               (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO13_CFG_REG_GPIO_SEL_MASK                         (0x00030000U)
#define CSL_IOMUX_PR0_PRU0_GPO13_CFG_REG_GPIO_SEL_SHIFT                        (0x00000010U)
#define CSL_IOMUX_PR0_PRU0_GPO13_CFG_REG_GPIO_SEL_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO13_CFG_REG_GPIO_SEL_MAX                          (0x00000003U)

#define CSL_IOMUX_PR0_PRU0_GPO13_CFG_REG_QUAL_SEL_MASK                         (0x000C0000U)
#define CSL_IOMUX_PR0_PRU0_GPO13_CFG_REG_QUAL_SEL_SHIFT                        (0x00000012U)
#define CSL_IOMUX_PR0_PRU0_GPO13_CFG_REG_QUAL_SEL_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO13_CFG_REG_QUAL_SEL_MAX                          (0x00000003U)

#define CSL_IOMUX_PR0_PRU0_GPO13_CFG_REG_INP_INV_SEL_MASK                      (0x00100000U)
#define CSL_IOMUX_PR0_PRU0_GPO13_CFG_REG_INP_INV_SEL_SHIFT                     (0x00000014U)
#define CSL_IOMUX_PR0_PRU0_GPO13_CFG_REG_INP_INV_SEL_RESETVAL                  (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO13_CFG_REG_INP_INV_SEL_MAX                       (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO13_CFG_REG_ICSSM_GPIO_SEL_MASK                   (0x00200000U)
#define CSL_IOMUX_PR0_PRU0_GPO13_CFG_REG_ICSSM_GPIO_SEL_SHIFT                  (0x00000015U)
#define CSL_IOMUX_PR0_PRU0_GPO13_CFG_REG_ICSSM_GPIO_SEL_RESETVAL               (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO13_CFG_REG_ICSSM_GPIO_SEL_MAX                    (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO13_CFG_REG_SAFETY_OVERRIDE_SEL_MASK              (0x00400000U)
#define CSL_IOMUX_PR0_PRU0_GPO13_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT             (0x00000016U)
#define CSL_IOMUX_PR0_PRU0_GPO13_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL          (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO13_CFG_REG_SAFETY_OVERRIDE_SEL_MAX               (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO13_CFG_REG_HSMODE_MASK                           (0x40000000U)
#define CSL_IOMUX_PR0_PRU0_GPO13_CFG_REG_HSMODE_SHIFT                          (0x0000001EU)
#define CSL_IOMUX_PR0_PRU0_GPO13_CFG_REG_HSMODE_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO13_CFG_REG_HSMODE_MAX                            (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO13_CFG_REG_HSMASTER_MASK                         (0x80000000U)
#define CSL_IOMUX_PR0_PRU0_GPO13_CFG_REG_HSMASTER_SHIFT                        (0x0000001FU)
#define CSL_IOMUX_PR0_PRU0_GPO13_CFG_REG_HSMASTER_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO13_CFG_REG_HSMASTER_MAX                          (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO13_CFG_REG_RESETVAL                              (0x000005F7U)

/* PR0_PRU0_GPO14_CFG_REG */

#define CSL_IOMUX_PR0_PRU0_GPO14_CFG_REG_FUNC_SEL_MASK                         (0x0000000FU)
#define CSL_IOMUX_PR0_PRU0_GPO14_CFG_REG_FUNC_SEL_SHIFT                        (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO14_CFG_REG_FUNC_SEL_RESETVAL                     (0x00000007U)
#define CSL_IOMUX_PR0_PRU0_GPO14_CFG_REG_FUNC_SEL_MAX                          (0x0000000FU)

#define CSL_IOMUX_PR0_PRU0_GPO14_CFG_REG_IE_OVERRIDE_CTRL_MASK                 (0x00000010U)
#define CSL_IOMUX_PR0_PRU0_GPO14_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                (0x00000004U)
#define CSL_IOMUX_PR0_PRU0_GPO14_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL             (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO14_CFG_REG_IE_OVERRIDE_CTRL_MAX                  (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO14_CFG_REG_IE_OVERRIDE_MASK                      (0x00000020U)
#define CSL_IOMUX_PR0_PRU0_GPO14_CFG_REG_IE_OVERRIDE_SHIFT                     (0x00000005U)
#define CSL_IOMUX_PR0_PRU0_GPO14_CFG_REG_IE_OVERRIDE_RESETVAL                  (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO14_CFG_REG_IE_OVERRIDE_MAX                       (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO14_CFG_REG_OE_OVERRIDE_CTRL_MASK                 (0x00000040U)
#define CSL_IOMUX_PR0_PRU0_GPO14_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                (0x00000006U)
#define CSL_IOMUX_PR0_PRU0_GPO14_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL             (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO14_CFG_REG_OE_OVERRIDE_CTRL_MAX                  (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO14_CFG_REG_OE_OVERRIDE_MASK                      (0x00000080U)
#define CSL_IOMUX_PR0_PRU0_GPO14_CFG_REG_OE_OVERRIDE_SHIFT                     (0x00000007U)
#define CSL_IOMUX_PR0_PRU0_GPO14_CFG_REG_OE_OVERRIDE_RESETVAL                  (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO14_CFG_REG_OE_OVERRIDE_MAX                       (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO14_CFG_REG_PI_MASK                               (0x00000100U)
#define CSL_IOMUX_PR0_PRU0_GPO14_CFG_REG_PI_SHIFT                              (0x00000008U)
#define CSL_IOMUX_PR0_PRU0_GPO14_CFG_REG_PI_RESETVAL                           (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO14_CFG_REG_PI_MAX                                (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO14_CFG_REG_PUPDSEL_MASK                          (0x00000200U)
#define CSL_IOMUX_PR0_PRU0_GPO14_CFG_REG_PUPDSEL_SHIFT                         (0x00000009U)
#define CSL_IOMUX_PR0_PRU0_GPO14_CFG_REG_PUPDSEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO14_CFG_REG_PUPDSEL_MAX                           (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO14_CFG_REG_SC1_MASK                              (0x00000400U)
#define CSL_IOMUX_PR0_PRU0_GPO14_CFG_REG_SC1_SHIFT                             (0x0000000AU)
#define CSL_IOMUX_PR0_PRU0_GPO14_CFG_REG_SC1_RESETVAL                          (0x00000001U)
#define CSL_IOMUX_PR0_PRU0_GPO14_CFG_REG_SC1_MAX                               (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO14_CFG_REG_GPIO_SEL_MASK                         (0x00030000U)
#define CSL_IOMUX_PR0_PRU0_GPO14_CFG_REG_GPIO_SEL_SHIFT                        (0x00000010U)
#define CSL_IOMUX_PR0_PRU0_GPO14_CFG_REG_GPIO_SEL_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO14_CFG_REG_GPIO_SEL_MAX                          (0x00000003U)

#define CSL_IOMUX_PR0_PRU0_GPO14_CFG_REG_QUAL_SEL_MASK                         (0x000C0000U)
#define CSL_IOMUX_PR0_PRU0_GPO14_CFG_REG_QUAL_SEL_SHIFT                        (0x00000012U)
#define CSL_IOMUX_PR0_PRU0_GPO14_CFG_REG_QUAL_SEL_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO14_CFG_REG_QUAL_SEL_MAX                          (0x00000003U)

#define CSL_IOMUX_PR0_PRU0_GPO14_CFG_REG_INP_INV_SEL_MASK                      (0x00100000U)
#define CSL_IOMUX_PR0_PRU0_GPO14_CFG_REG_INP_INV_SEL_SHIFT                     (0x00000014U)
#define CSL_IOMUX_PR0_PRU0_GPO14_CFG_REG_INP_INV_SEL_RESETVAL                  (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO14_CFG_REG_INP_INV_SEL_MAX                       (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO14_CFG_REG_ICSSM_GPIO_SEL_MASK                   (0x00200000U)
#define CSL_IOMUX_PR0_PRU0_GPO14_CFG_REG_ICSSM_GPIO_SEL_SHIFT                  (0x00000015U)
#define CSL_IOMUX_PR0_PRU0_GPO14_CFG_REG_ICSSM_GPIO_SEL_RESETVAL               (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO14_CFG_REG_ICSSM_GPIO_SEL_MAX                    (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO14_CFG_REG_SAFETY_OVERRIDE_SEL_MASK              (0x00400000U)
#define CSL_IOMUX_PR0_PRU0_GPO14_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT             (0x00000016U)
#define CSL_IOMUX_PR0_PRU0_GPO14_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL          (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO14_CFG_REG_SAFETY_OVERRIDE_SEL_MAX               (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO14_CFG_REG_HSMODE_MASK                           (0x40000000U)
#define CSL_IOMUX_PR0_PRU0_GPO14_CFG_REG_HSMODE_SHIFT                          (0x0000001EU)
#define CSL_IOMUX_PR0_PRU0_GPO14_CFG_REG_HSMODE_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO14_CFG_REG_HSMODE_MAX                            (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO14_CFG_REG_HSMASTER_MASK                         (0x80000000U)
#define CSL_IOMUX_PR0_PRU0_GPO14_CFG_REG_HSMASTER_SHIFT                        (0x0000001FU)
#define CSL_IOMUX_PR0_PRU0_GPO14_CFG_REG_HSMASTER_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_PR0_PRU0_GPO14_CFG_REG_HSMASTER_MAX                          (0x00000001U)

#define CSL_IOMUX_PR0_PRU0_GPO14_CFG_REG_RESETVAL                              (0x000005F7U)

/* PR0_PRU1_GPO5_CFG_REG */

#define CSL_IOMUX_PR0_PRU1_GPO5_CFG_REG_FUNC_SEL_MASK                          (0x0000000FU)
#define CSL_IOMUX_PR0_PRU1_GPO5_CFG_REG_FUNC_SEL_SHIFT                         (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO5_CFG_REG_FUNC_SEL_RESETVAL                      (0x00000007U)
#define CSL_IOMUX_PR0_PRU1_GPO5_CFG_REG_FUNC_SEL_MAX                           (0x0000000FU)

#define CSL_IOMUX_PR0_PRU1_GPO5_CFG_REG_IE_OVERRIDE_CTRL_MASK                  (0x00000010U)
#define CSL_IOMUX_PR0_PRU1_GPO5_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                 (0x00000004U)
#define CSL_IOMUX_PR0_PRU1_GPO5_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL              (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO5_CFG_REG_IE_OVERRIDE_CTRL_MAX                   (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO5_CFG_REG_IE_OVERRIDE_MASK                       (0x00000020U)
#define CSL_IOMUX_PR0_PRU1_GPO5_CFG_REG_IE_OVERRIDE_SHIFT                      (0x00000005U)
#define CSL_IOMUX_PR0_PRU1_GPO5_CFG_REG_IE_OVERRIDE_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO5_CFG_REG_IE_OVERRIDE_MAX                        (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO5_CFG_REG_OE_OVERRIDE_CTRL_MASK                  (0x00000040U)
#define CSL_IOMUX_PR0_PRU1_GPO5_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                 (0x00000006U)
#define CSL_IOMUX_PR0_PRU1_GPO5_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL              (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO5_CFG_REG_OE_OVERRIDE_CTRL_MAX                   (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO5_CFG_REG_OE_OVERRIDE_MASK                       (0x00000080U)
#define CSL_IOMUX_PR0_PRU1_GPO5_CFG_REG_OE_OVERRIDE_SHIFT                      (0x00000007U)
#define CSL_IOMUX_PR0_PRU1_GPO5_CFG_REG_OE_OVERRIDE_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO5_CFG_REG_OE_OVERRIDE_MAX                        (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO5_CFG_REG_PI_MASK                                (0x00000100U)
#define CSL_IOMUX_PR0_PRU1_GPO5_CFG_REG_PI_SHIFT                               (0x00000008U)
#define CSL_IOMUX_PR0_PRU1_GPO5_CFG_REG_PI_RESETVAL                            (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO5_CFG_REG_PI_MAX                                 (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO5_CFG_REG_PUPDSEL_MASK                           (0x00000200U)
#define CSL_IOMUX_PR0_PRU1_GPO5_CFG_REG_PUPDSEL_SHIFT                          (0x00000009U)
#define CSL_IOMUX_PR0_PRU1_GPO5_CFG_REG_PUPDSEL_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO5_CFG_REG_PUPDSEL_MAX                            (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO5_CFG_REG_SC1_MASK                               (0x00000400U)
#define CSL_IOMUX_PR0_PRU1_GPO5_CFG_REG_SC1_SHIFT                              (0x0000000AU)
#define CSL_IOMUX_PR0_PRU1_GPO5_CFG_REG_SC1_RESETVAL                           (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO5_CFG_REG_SC1_MAX                                (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO5_CFG_REG_GPIO_SEL_MASK                          (0x00030000U)
#define CSL_IOMUX_PR0_PRU1_GPO5_CFG_REG_GPIO_SEL_SHIFT                         (0x00000010U)
#define CSL_IOMUX_PR0_PRU1_GPO5_CFG_REG_GPIO_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO5_CFG_REG_GPIO_SEL_MAX                           (0x00000003U)

#define CSL_IOMUX_PR0_PRU1_GPO5_CFG_REG_QUAL_SEL_MASK                          (0x000C0000U)
#define CSL_IOMUX_PR0_PRU1_GPO5_CFG_REG_QUAL_SEL_SHIFT                         (0x00000012U)
#define CSL_IOMUX_PR0_PRU1_GPO5_CFG_REG_QUAL_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO5_CFG_REG_QUAL_SEL_MAX                           (0x00000003U)

#define CSL_IOMUX_PR0_PRU1_GPO5_CFG_REG_INP_INV_SEL_MASK                       (0x00100000U)
#define CSL_IOMUX_PR0_PRU1_GPO5_CFG_REG_INP_INV_SEL_SHIFT                      (0x00000014U)
#define CSL_IOMUX_PR0_PRU1_GPO5_CFG_REG_INP_INV_SEL_RESETVAL                   (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO5_CFG_REG_INP_INV_SEL_MAX                        (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO5_CFG_REG_ICSSM_GPIO_SEL_MASK                    (0x00200000U)
#define CSL_IOMUX_PR0_PRU1_GPO5_CFG_REG_ICSSM_GPIO_SEL_SHIFT                   (0x00000015U)
#define CSL_IOMUX_PR0_PRU1_GPO5_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO5_CFG_REG_ICSSM_GPIO_SEL_MAX                     (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO5_CFG_REG_SAFETY_OVERRIDE_SEL_MASK               (0x00400000U)
#define CSL_IOMUX_PR0_PRU1_GPO5_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT              (0x00000016U)
#define CSL_IOMUX_PR0_PRU1_GPO5_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL           (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO5_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO5_CFG_REG_HSMODE_MASK                            (0x40000000U)
#define CSL_IOMUX_PR0_PRU1_GPO5_CFG_REG_HSMODE_SHIFT                           (0x0000001EU)
#define CSL_IOMUX_PR0_PRU1_GPO5_CFG_REG_HSMODE_RESETVAL                        (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO5_CFG_REG_HSMODE_MAX                             (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO5_CFG_REG_HSMASTER_MASK                          (0x80000000U)
#define CSL_IOMUX_PR0_PRU1_GPO5_CFG_REG_HSMASTER_SHIFT                         (0x0000001FU)
#define CSL_IOMUX_PR0_PRU1_GPO5_CFG_REG_HSMASTER_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO5_CFG_REG_HSMASTER_MAX                           (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO5_CFG_REG_RESETVAL                               (0x000005F7U)

/* PR0_PRU1_GPO9_CFG_REG */

#define CSL_IOMUX_PR0_PRU1_GPO9_CFG_REG_FUNC_SEL_MASK                          (0x0000000FU)
#define CSL_IOMUX_PR0_PRU1_GPO9_CFG_REG_FUNC_SEL_SHIFT                         (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO9_CFG_REG_FUNC_SEL_RESETVAL                      (0x00000007U)
#define CSL_IOMUX_PR0_PRU1_GPO9_CFG_REG_FUNC_SEL_MAX                           (0x0000000FU)

#define CSL_IOMUX_PR0_PRU1_GPO9_CFG_REG_IE_OVERRIDE_CTRL_MASK                  (0x00000010U)
#define CSL_IOMUX_PR0_PRU1_GPO9_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                 (0x00000004U)
#define CSL_IOMUX_PR0_PRU1_GPO9_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL              (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO9_CFG_REG_IE_OVERRIDE_CTRL_MAX                   (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO9_CFG_REG_IE_OVERRIDE_MASK                       (0x00000020U)
#define CSL_IOMUX_PR0_PRU1_GPO9_CFG_REG_IE_OVERRIDE_SHIFT                      (0x00000005U)
#define CSL_IOMUX_PR0_PRU1_GPO9_CFG_REG_IE_OVERRIDE_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO9_CFG_REG_IE_OVERRIDE_MAX                        (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO9_CFG_REG_OE_OVERRIDE_CTRL_MASK                  (0x00000040U)
#define CSL_IOMUX_PR0_PRU1_GPO9_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                 (0x00000006U)
#define CSL_IOMUX_PR0_PRU1_GPO9_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL              (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO9_CFG_REG_OE_OVERRIDE_CTRL_MAX                   (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO9_CFG_REG_OE_OVERRIDE_MASK                       (0x00000080U)
#define CSL_IOMUX_PR0_PRU1_GPO9_CFG_REG_OE_OVERRIDE_SHIFT                      (0x00000007U)
#define CSL_IOMUX_PR0_PRU1_GPO9_CFG_REG_OE_OVERRIDE_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO9_CFG_REG_OE_OVERRIDE_MAX                        (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO9_CFG_REG_PI_MASK                                (0x00000100U)
#define CSL_IOMUX_PR0_PRU1_GPO9_CFG_REG_PI_SHIFT                               (0x00000008U)
#define CSL_IOMUX_PR0_PRU1_GPO9_CFG_REG_PI_RESETVAL                            (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO9_CFG_REG_PI_MAX                                 (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO9_CFG_REG_PUPDSEL_MASK                           (0x00000200U)
#define CSL_IOMUX_PR0_PRU1_GPO9_CFG_REG_PUPDSEL_SHIFT                          (0x00000009U)
#define CSL_IOMUX_PR0_PRU1_GPO9_CFG_REG_PUPDSEL_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO9_CFG_REG_PUPDSEL_MAX                            (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO9_CFG_REG_SC1_MASK                               (0x00000400U)
#define CSL_IOMUX_PR0_PRU1_GPO9_CFG_REG_SC1_SHIFT                              (0x0000000AU)
#define CSL_IOMUX_PR0_PRU1_GPO9_CFG_REG_SC1_RESETVAL                           (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO9_CFG_REG_SC1_MAX                                (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO9_CFG_REG_GPIO_SEL_MASK                          (0x00030000U)
#define CSL_IOMUX_PR0_PRU1_GPO9_CFG_REG_GPIO_SEL_SHIFT                         (0x00000010U)
#define CSL_IOMUX_PR0_PRU1_GPO9_CFG_REG_GPIO_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO9_CFG_REG_GPIO_SEL_MAX                           (0x00000003U)

#define CSL_IOMUX_PR0_PRU1_GPO9_CFG_REG_QUAL_SEL_MASK                          (0x000C0000U)
#define CSL_IOMUX_PR0_PRU1_GPO9_CFG_REG_QUAL_SEL_SHIFT                         (0x00000012U)
#define CSL_IOMUX_PR0_PRU1_GPO9_CFG_REG_QUAL_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO9_CFG_REG_QUAL_SEL_MAX                           (0x00000003U)

#define CSL_IOMUX_PR0_PRU1_GPO9_CFG_REG_INP_INV_SEL_MASK                       (0x00100000U)
#define CSL_IOMUX_PR0_PRU1_GPO9_CFG_REG_INP_INV_SEL_SHIFT                      (0x00000014U)
#define CSL_IOMUX_PR0_PRU1_GPO9_CFG_REG_INP_INV_SEL_RESETVAL                   (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO9_CFG_REG_INP_INV_SEL_MAX                        (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO9_CFG_REG_ICSSM_GPIO_SEL_MASK                    (0x00200000U)
#define CSL_IOMUX_PR0_PRU1_GPO9_CFG_REG_ICSSM_GPIO_SEL_SHIFT                   (0x00000015U)
#define CSL_IOMUX_PR0_PRU1_GPO9_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO9_CFG_REG_ICSSM_GPIO_SEL_MAX                     (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO9_CFG_REG_SAFETY_OVERRIDE_SEL_MASK               (0x00400000U)
#define CSL_IOMUX_PR0_PRU1_GPO9_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT              (0x00000016U)
#define CSL_IOMUX_PR0_PRU1_GPO9_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL           (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO9_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO9_CFG_REG_HSMODE_MASK                            (0x40000000U)
#define CSL_IOMUX_PR0_PRU1_GPO9_CFG_REG_HSMODE_SHIFT                           (0x0000001EU)
#define CSL_IOMUX_PR0_PRU1_GPO9_CFG_REG_HSMODE_RESETVAL                        (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO9_CFG_REG_HSMODE_MAX                             (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO9_CFG_REG_HSMASTER_MASK                          (0x80000000U)
#define CSL_IOMUX_PR0_PRU1_GPO9_CFG_REG_HSMASTER_SHIFT                         (0x0000001FU)
#define CSL_IOMUX_PR0_PRU1_GPO9_CFG_REG_HSMASTER_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO9_CFG_REG_HSMASTER_MAX                           (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO9_CFG_REG_RESETVAL                               (0x000005F7U)

/* PR0_PRU1_GPO10_CFG_REG */

#define CSL_IOMUX_PR0_PRU1_GPO10_CFG_REG_FUNC_SEL_MASK                         (0x0000000FU)
#define CSL_IOMUX_PR0_PRU1_GPO10_CFG_REG_FUNC_SEL_SHIFT                        (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO10_CFG_REG_FUNC_SEL_RESETVAL                     (0x00000007U)
#define CSL_IOMUX_PR0_PRU1_GPO10_CFG_REG_FUNC_SEL_MAX                          (0x0000000FU)

#define CSL_IOMUX_PR0_PRU1_GPO10_CFG_REG_IE_OVERRIDE_CTRL_MASK                 (0x00000010U)
#define CSL_IOMUX_PR0_PRU1_GPO10_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                (0x00000004U)
#define CSL_IOMUX_PR0_PRU1_GPO10_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL             (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO10_CFG_REG_IE_OVERRIDE_CTRL_MAX                  (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO10_CFG_REG_IE_OVERRIDE_MASK                      (0x00000020U)
#define CSL_IOMUX_PR0_PRU1_GPO10_CFG_REG_IE_OVERRIDE_SHIFT                     (0x00000005U)
#define CSL_IOMUX_PR0_PRU1_GPO10_CFG_REG_IE_OVERRIDE_RESETVAL                  (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO10_CFG_REG_IE_OVERRIDE_MAX                       (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO10_CFG_REG_OE_OVERRIDE_CTRL_MASK                 (0x00000040U)
#define CSL_IOMUX_PR0_PRU1_GPO10_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                (0x00000006U)
#define CSL_IOMUX_PR0_PRU1_GPO10_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL             (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO10_CFG_REG_OE_OVERRIDE_CTRL_MAX                  (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO10_CFG_REG_OE_OVERRIDE_MASK                      (0x00000080U)
#define CSL_IOMUX_PR0_PRU1_GPO10_CFG_REG_OE_OVERRIDE_SHIFT                     (0x00000007U)
#define CSL_IOMUX_PR0_PRU1_GPO10_CFG_REG_OE_OVERRIDE_RESETVAL                  (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO10_CFG_REG_OE_OVERRIDE_MAX                       (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO10_CFG_REG_PI_MASK                               (0x00000100U)
#define CSL_IOMUX_PR0_PRU1_GPO10_CFG_REG_PI_SHIFT                              (0x00000008U)
#define CSL_IOMUX_PR0_PRU1_GPO10_CFG_REG_PI_RESETVAL                           (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO10_CFG_REG_PI_MAX                                (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO10_CFG_REG_PUPDSEL_MASK                          (0x00000200U)
#define CSL_IOMUX_PR0_PRU1_GPO10_CFG_REG_PUPDSEL_SHIFT                         (0x00000009U)
#define CSL_IOMUX_PR0_PRU1_GPO10_CFG_REG_PUPDSEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO10_CFG_REG_PUPDSEL_MAX                           (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO10_CFG_REG_SC1_MASK                              (0x00000400U)
#define CSL_IOMUX_PR0_PRU1_GPO10_CFG_REG_SC1_SHIFT                             (0x0000000AU)
#define CSL_IOMUX_PR0_PRU1_GPO10_CFG_REG_SC1_RESETVAL                          (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO10_CFG_REG_SC1_MAX                               (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO10_CFG_REG_GPIO_SEL_MASK                         (0x00030000U)
#define CSL_IOMUX_PR0_PRU1_GPO10_CFG_REG_GPIO_SEL_SHIFT                        (0x00000010U)
#define CSL_IOMUX_PR0_PRU1_GPO10_CFG_REG_GPIO_SEL_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO10_CFG_REG_GPIO_SEL_MAX                          (0x00000003U)

#define CSL_IOMUX_PR0_PRU1_GPO10_CFG_REG_QUAL_SEL_MASK                         (0x000C0000U)
#define CSL_IOMUX_PR0_PRU1_GPO10_CFG_REG_QUAL_SEL_SHIFT                        (0x00000012U)
#define CSL_IOMUX_PR0_PRU1_GPO10_CFG_REG_QUAL_SEL_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO10_CFG_REG_QUAL_SEL_MAX                          (0x00000003U)

#define CSL_IOMUX_PR0_PRU1_GPO10_CFG_REG_INP_INV_SEL_MASK                      (0x00100000U)
#define CSL_IOMUX_PR0_PRU1_GPO10_CFG_REG_INP_INV_SEL_SHIFT                     (0x00000014U)
#define CSL_IOMUX_PR0_PRU1_GPO10_CFG_REG_INP_INV_SEL_RESETVAL                  (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO10_CFG_REG_INP_INV_SEL_MAX                       (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO10_CFG_REG_ICSSM_GPIO_SEL_MASK                   (0x00200000U)
#define CSL_IOMUX_PR0_PRU1_GPO10_CFG_REG_ICSSM_GPIO_SEL_SHIFT                  (0x00000015U)
#define CSL_IOMUX_PR0_PRU1_GPO10_CFG_REG_ICSSM_GPIO_SEL_RESETVAL               (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO10_CFG_REG_ICSSM_GPIO_SEL_MAX                    (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO10_CFG_REG_SAFETY_OVERRIDE_SEL_MASK              (0x00400000U)
#define CSL_IOMUX_PR0_PRU1_GPO10_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT             (0x00000016U)
#define CSL_IOMUX_PR0_PRU1_GPO10_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL          (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO10_CFG_REG_SAFETY_OVERRIDE_SEL_MAX               (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO10_CFG_REG_HSMODE_MASK                           (0x40000000U)
#define CSL_IOMUX_PR0_PRU1_GPO10_CFG_REG_HSMODE_SHIFT                          (0x0000001EU)
#define CSL_IOMUX_PR0_PRU1_GPO10_CFG_REG_HSMODE_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO10_CFG_REG_HSMODE_MAX                            (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO10_CFG_REG_HSMASTER_MASK                         (0x80000000U)
#define CSL_IOMUX_PR0_PRU1_GPO10_CFG_REG_HSMASTER_SHIFT                        (0x0000001FU)
#define CSL_IOMUX_PR0_PRU1_GPO10_CFG_REG_HSMASTER_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO10_CFG_REG_HSMASTER_MAX                          (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO10_CFG_REG_RESETVAL                              (0x000005F7U)

/* PR0_PRU1_GPO8_CFG_REG */

#define CSL_IOMUX_PR0_PRU1_GPO8_CFG_REG_FUNC_SEL_MASK                          (0x0000000FU)
#define CSL_IOMUX_PR0_PRU1_GPO8_CFG_REG_FUNC_SEL_SHIFT                         (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO8_CFG_REG_FUNC_SEL_RESETVAL                      (0x00000007U)
#define CSL_IOMUX_PR0_PRU1_GPO8_CFG_REG_FUNC_SEL_MAX                           (0x0000000FU)

#define CSL_IOMUX_PR0_PRU1_GPO8_CFG_REG_IE_OVERRIDE_CTRL_MASK                  (0x00000010U)
#define CSL_IOMUX_PR0_PRU1_GPO8_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                 (0x00000004U)
#define CSL_IOMUX_PR0_PRU1_GPO8_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL              (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO8_CFG_REG_IE_OVERRIDE_CTRL_MAX                   (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO8_CFG_REG_IE_OVERRIDE_MASK                       (0x00000020U)
#define CSL_IOMUX_PR0_PRU1_GPO8_CFG_REG_IE_OVERRIDE_SHIFT                      (0x00000005U)
#define CSL_IOMUX_PR0_PRU1_GPO8_CFG_REG_IE_OVERRIDE_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO8_CFG_REG_IE_OVERRIDE_MAX                        (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO8_CFG_REG_OE_OVERRIDE_CTRL_MASK                  (0x00000040U)
#define CSL_IOMUX_PR0_PRU1_GPO8_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                 (0x00000006U)
#define CSL_IOMUX_PR0_PRU1_GPO8_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL              (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO8_CFG_REG_OE_OVERRIDE_CTRL_MAX                   (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO8_CFG_REG_OE_OVERRIDE_MASK                       (0x00000080U)
#define CSL_IOMUX_PR0_PRU1_GPO8_CFG_REG_OE_OVERRIDE_SHIFT                      (0x00000007U)
#define CSL_IOMUX_PR0_PRU1_GPO8_CFG_REG_OE_OVERRIDE_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO8_CFG_REG_OE_OVERRIDE_MAX                        (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO8_CFG_REG_PI_MASK                                (0x00000100U)
#define CSL_IOMUX_PR0_PRU1_GPO8_CFG_REG_PI_SHIFT                               (0x00000008U)
#define CSL_IOMUX_PR0_PRU1_GPO8_CFG_REG_PI_RESETVAL                            (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO8_CFG_REG_PI_MAX                                 (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO8_CFG_REG_PUPDSEL_MASK                           (0x00000200U)
#define CSL_IOMUX_PR0_PRU1_GPO8_CFG_REG_PUPDSEL_SHIFT                          (0x00000009U)
#define CSL_IOMUX_PR0_PRU1_GPO8_CFG_REG_PUPDSEL_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO8_CFG_REG_PUPDSEL_MAX                            (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO8_CFG_REG_SC1_MASK                               (0x00000400U)
#define CSL_IOMUX_PR0_PRU1_GPO8_CFG_REG_SC1_SHIFT                              (0x0000000AU)
#define CSL_IOMUX_PR0_PRU1_GPO8_CFG_REG_SC1_RESETVAL                           (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO8_CFG_REG_SC1_MAX                                (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO8_CFG_REG_GPIO_SEL_MASK                          (0x00030000U)
#define CSL_IOMUX_PR0_PRU1_GPO8_CFG_REG_GPIO_SEL_SHIFT                         (0x00000010U)
#define CSL_IOMUX_PR0_PRU1_GPO8_CFG_REG_GPIO_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO8_CFG_REG_GPIO_SEL_MAX                           (0x00000003U)

#define CSL_IOMUX_PR0_PRU1_GPO8_CFG_REG_QUAL_SEL_MASK                          (0x000C0000U)
#define CSL_IOMUX_PR0_PRU1_GPO8_CFG_REG_QUAL_SEL_SHIFT                         (0x00000012U)
#define CSL_IOMUX_PR0_PRU1_GPO8_CFG_REG_QUAL_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO8_CFG_REG_QUAL_SEL_MAX                           (0x00000003U)

#define CSL_IOMUX_PR0_PRU1_GPO8_CFG_REG_INP_INV_SEL_MASK                       (0x00100000U)
#define CSL_IOMUX_PR0_PRU1_GPO8_CFG_REG_INP_INV_SEL_SHIFT                      (0x00000014U)
#define CSL_IOMUX_PR0_PRU1_GPO8_CFG_REG_INP_INV_SEL_RESETVAL                   (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO8_CFG_REG_INP_INV_SEL_MAX                        (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO8_CFG_REG_ICSSM_GPIO_SEL_MASK                    (0x00200000U)
#define CSL_IOMUX_PR0_PRU1_GPO8_CFG_REG_ICSSM_GPIO_SEL_SHIFT                   (0x00000015U)
#define CSL_IOMUX_PR0_PRU1_GPO8_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO8_CFG_REG_ICSSM_GPIO_SEL_MAX                     (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO8_CFG_REG_SAFETY_OVERRIDE_SEL_MASK               (0x00400000U)
#define CSL_IOMUX_PR0_PRU1_GPO8_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT              (0x00000016U)
#define CSL_IOMUX_PR0_PRU1_GPO8_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL           (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO8_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO8_CFG_REG_HSMODE_MASK                            (0x40000000U)
#define CSL_IOMUX_PR0_PRU1_GPO8_CFG_REG_HSMODE_SHIFT                           (0x0000001EU)
#define CSL_IOMUX_PR0_PRU1_GPO8_CFG_REG_HSMODE_RESETVAL                        (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO8_CFG_REG_HSMODE_MAX                             (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO8_CFG_REG_HSMASTER_MASK                          (0x80000000U)
#define CSL_IOMUX_PR0_PRU1_GPO8_CFG_REG_HSMASTER_SHIFT                         (0x0000001FU)
#define CSL_IOMUX_PR0_PRU1_GPO8_CFG_REG_HSMASTER_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO8_CFG_REG_HSMASTER_MAX                           (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO8_CFG_REG_RESETVAL                               (0x000005F7U)

/* PR0_PRU1_GPO6_CFG_REG */

#define CSL_IOMUX_PR0_PRU1_GPO6_CFG_REG_FUNC_SEL_MASK                          (0x0000000FU)
#define CSL_IOMUX_PR0_PRU1_GPO6_CFG_REG_FUNC_SEL_SHIFT                         (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO6_CFG_REG_FUNC_SEL_RESETVAL                      (0x00000007U)
#define CSL_IOMUX_PR0_PRU1_GPO6_CFG_REG_FUNC_SEL_MAX                           (0x0000000FU)

#define CSL_IOMUX_PR0_PRU1_GPO6_CFG_REG_IE_OVERRIDE_CTRL_MASK                  (0x00000010U)
#define CSL_IOMUX_PR0_PRU1_GPO6_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                 (0x00000004U)
#define CSL_IOMUX_PR0_PRU1_GPO6_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL              (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO6_CFG_REG_IE_OVERRIDE_CTRL_MAX                   (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO6_CFG_REG_IE_OVERRIDE_MASK                       (0x00000020U)
#define CSL_IOMUX_PR0_PRU1_GPO6_CFG_REG_IE_OVERRIDE_SHIFT                      (0x00000005U)
#define CSL_IOMUX_PR0_PRU1_GPO6_CFG_REG_IE_OVERRIDE_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO6_CFG_REG_IE_OVERRIDE_MAX                        (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO6_CFG_REG_OE_OVERRIDE_CTRL_MASK                  (0x00000040U)
#define CSL_IOMUX_PR0_PRU1_GPO6_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                 (0x00000006U)
#define CSL_IOMUX_PR0_PRU1_GPO6_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL              (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO6_CFG_REG_OE_OVERRIDE_CTRL_MAX                   (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO6_CFG_REG_OE_OVERRIDE_MASK                       (0x00000080U)
#define CSL_IOMUX_PR0_PRU1_GPO6_CFG_REG_OE_OVERRIDE_SHIFT                      (0x00000007U)
#define CSL_IOMUX_PR0_PRU1_GPO6_CFG_REG_OE_OVERRIDE_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO6_CFG_REG_OE_OVERRIDE_MAX                        (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO6_CFG_REG_PI_MASK                                (0x00000100U)
#define CSL_IOMUX_PR0_PRU1_GPO6_CFG_REG_PI_SHIFT                               (0x00000008U)
#define CSL_IOMUX_PR0_PRU1_GPO6_CFG_REG_PI_RESETVAL                            (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO6_CFG_REG_PI_MAX                                 (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO6_CFG_REG_PUPDSEL_MASK                           (0x00000200U)
#define CSL_IOMUX_PR0_PRU1_GPO6_CFG_REG_PUPDSEL_SHIFT                          (0x00000009U)
#define CSL_IOMUX_PR0_PRU1_GPO6_CFG_REG_PUPDSEL_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO6_CFG_REG_PUPDSEL_MAX                            (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO6_CFG_REG_SC1_MASK                               (0x00000400U)
#define CSL_IOMUX_PR0_PRU1_GPO6_CFG_REG_SC1_SHIFT                              (0x0000000AU)
#define CSL_IOMUX_PR0_PRU1_GPO6_CFG_REG_SC1_RESETVAL                           (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO6_CFG_REG_SC1_MAX                                (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO6_CFG_REG_GPIO_SEL_MASK                          (0x00030000U)
#define CSL_IOMUX_PR0_PRU1_GPO6_CFG_REG_GPIO_SEL_SHIFT                         (0x00000010U)
#define CSL_IOMUX_PR0_PRU1_GPO6_CFG_REG_GPIO_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO6_CFG_REG_GPIO_SEL_MAX                           (0x00000003U)

#define CSL_IOMUX_PR0_PRU1_GPO6_CFG_REG_QUAL_SEL_MASK                          (0x000C0000U)
#define CSL_IOMUX_PR0_PRU1_GPO6_CFG_REG_QUAL_SEL_SHIFT                         (0x00000012U)
#define CSL_IOMUX_PR0_PRU1_GPO6_CFG_REG_QUAL_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO6_CFG_REG_QUAL_SEL_MAX                           (0x00000003U)

#define CSL_IOMUX_PR0_PRU1_GPO6_CFG_REG_INP_INV_SEL_MASK                       (0x00100000U)
#define CSL_IOMUX_PR0_PRU1_GPO6_CFG_REG_INP_INV_SEL_SHIFT                      (0x00000014U)
#define CSL_IOMUX_PR0_PRU1_GPO6_CFG_REG_INP_INV_SEL_RESETVAL                   (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO6_CFG_REG_INP_INV_SEL_MAX                        (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO6_CFG_REG_ICSSM_GPIO_SEL_MASK                    (0x00200000U)
#define CSL_IOMUX_PR0_PRU1_GPO6_CFG_REG_ICSSM_GPIO_SEL_SHIFT                   (0x00000015U)
#define CSL_IOMUX_PR0_PRU1_GPO6_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO6_CFG_REG_ICSSM_GPIO_SEL_MAX                     (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO6_CFG_REG_SAFETY_OVERRIDE_SEL_MASK               (0x00400000U)
#define CSL_IOMUX_PR0_PRU1_GPO6_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT              (0x00000016U)
#define CSL_IOMUX_PR0_PRU1_GPO6_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL           (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO6_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO6_CFG_REG_HSMODE_MASK                            (0x40000000U)
#define CSL_IOMUX_PR0_PRU1_GPO6_CFG_REG_HSMODE_SHIFT                           (0x0000001EU)
#define CSL_IOMUX_PR0_PRU1_GPO6_CFG_REG_HSMODE_RESETVAL                        (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO6_CFG_REG_HSMODE_MAX                             (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO6_CFG_REG_HSMASTER_MASK                          (0x80000000U)
#define CSL_IOMUX_PR0_PRU1_GPO6_CFG_REG_HSMASTER_SHIFT                         (0x0000001FU)
#define CSL_IOMUX_PR0_PRU1_GPO6_CFG_REG_HSMASTER_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO6_CFG_REG_HSMASTER_MAX                           (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO6_CFG_REG_RESETVAL                               (0x000005F7U)

/* PR0_PRU1_GPO4_CFG_REG */

#define CSL_IOMUX_PR0_PRU1_GPO4_CFG_REG_FUNC_SEL_MASK                          (0x0000000FU)
#define CSL_IOMUX_PR0_PRU1_GPO4_CFG_REG_FUNC_SEL_SHIFT                         (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO4_CFG_REG_FUNC_SEL_RESETVAL                      (0x00000007U)
#define CSL_IOMUX_PR0_PRU1_GPO4_CFG_REG_FUNC_SEL_MAX                           (0x0000000FU)

#define CSL_IOMUX_PR0_PRU1_GPO4_CFG_REG_IE_OVERRIDE_CTRL_MASK                  (0x00000010U)
#define CSL_IOMUX_PR0_PRU1_GPO4_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                 (0x00000004U)
#define CSL_IOMUX_PR0_PRU1_GPO4_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL              (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO4_CFG_REG_IE_OVERRIDE_CTRL_MAX                   (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO4_CFG_REG_IE_OVERRIDE_MASK                       (0x00000020U)
#define CSL_IOMUX_PR0_PRU1_GPO4_CFG_REG_IE_OVERRIDE_SHIFT                      (0x00000005U)
#define CSL_IOMUX_PR0_PRU1_GPO4_CFG_REG_IE_OVERRIDE_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO4_CFG_REG_IE_OVERRIDE_MAX                        (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO4_CFG_REG_OE_OVERRIDE_CTRL_MASK                  (0x00000040U)
#define CSL_IOMUX_PR0_PRU1_GPO4_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                 (0x00000006U)
#define CSL_IOMUX_PR0_PRU1_GPO4_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL              (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO4_CFG_REG_OE_OVERRIDE_CTRL_MAX                   (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO4_CFG_REG_OE_OVERRIDE_MASK                       (0x00000080U)
#define CSL_IOMUX_PR0_PRU1_GPO4_CFG_REG_OE_OVERRIDE_SHIFT                      (0x00000007U)
#define CSL_IOMUX_PR0_PRU1_GPO4_CFG_REG_OE_OVERRIDE_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO4_CFG_REG_OE_OVERRIDE_MAX                        (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO4_CFG_REG_PI_MASK                                (0x00000100U)
#define CSL_IOMUX_PR0_PRU1_GPO4_CFG_REG_PI_SHIFT                               (0x00000008U)
#define CSL_IOMUX_PR0_PRU1_GPO4_CFG_REG_PI_RESETVAL                            (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO4_CFG_REG_PI_MAX                                 (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO4_CFG_REG_PUPDSEL_MASK                           (0x00000200U)
#define CSL_IOMUX_PR0_PRU1_GPO4_CFG_REG_PUPDSEL_SHIFT                          (0x00000009U)
#define CSL_IOMUX_PR0_PRU1_GPO4_CFG_REG_PUPDSEL_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO4_CFG_REG_PUPDSEL_MAX                            (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO4_CFG_REG_SC1_MASK                               (0x00000400U)
#define CSL_IOMUX_PR0_PRU1_GPO4_CFG_REG_SC1_SHIFT                              (0x0000000AU)
#define CSL_IOMUX_PR0_PRU1_GPO4_CFG_REG_SC1_RESETVAL                           (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO4_CFG_REG_SC1_MAX                                (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO4_CFG_REG_GPIO_SEL_MASK                          (0x00030000U)
#define CSL_IOMUX_PR0_PRU1_GPO4_CFG_REG_GPIO_SEL_SHIFT                         (0x00000010U)
#define CSL_IOMUX_PR0_PRU1_GPO4_CFG_REG_GPIO_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO4_CFG_REG_GPIO_SEL_MAX                           (0x00000003U)

#define CSL_IOMUX_PR0_PRU1_GPO4_CFG_REG_QUAL_SEL_MASK                          (0x000C0000U)
#define CSL_IOMUX_PR0_PRU1_GPO4_CFG_REG_QUAL_SEL_SHIFT                         (0x00000012U)
#define CSL_IOMUX_PR0_PRU1_GPO4_CFG_REG_QUAL_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO4_CFG_REG_QUAL_SEL_MAX                           (0x00000003U)

#define CSL_IOMUX_PR0_PRU1_GPO4_CFG_REG_INP_INV_SEL_MASK                       (0x00100000U)
#define CSL_IOMUX_PR0_PRU1_GPO4_CFG_REG_INP_INV_SEL_SHIFT                      (0x00000014U)
#define CSL_IOMUX_PR0_PRU1_GPO4_CFG_REG_INP_INV_SEL_RESETVAL                   (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO4_CFG_REG_INP_INV_SEL_MAX                        (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO4_CFG_REG_ICSSM_GPIO_SEL_MASK                    (0x00200000U)
#define CSL_IOMUX_PR0_PRU1_GPO4_CFG_REG_ICSSM_GPIO_SEL_SHIFT                   (0x00000015U)
#define CSL_IOMUX_PR0_PRU1_GPO4_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO4_CFG_REG_ICSSM_GPIO_SEL_MAX                     (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO4_CFG_REG_SAFETY_OVERRIDE_SEL_MASK               (0x00400000U)
#define CSL_IOMUX_PR0_PRU1_GPO4_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT              (0x00000016U)
#define CSL_IOMUX_PR0_PRU1_GPO4_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL           (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO4_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO4_CFG_REG_HSMODE_MASK                            (0x40000000U)
#define CSL_IOMUX_PR0_PRU1_GPO4_CFG_REG_HSMODE_SHIFT                           (0x0000001EU)
#define CSL_IOMUX_PR0_PRU1_GPO4_CFG_REG_HSMODE_RESETVAL                        (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO4_CFG_REG_HSMODE_MAX                             (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO4_CFG_REG_HSMASTER_MASK                          (0x80000000U)
#define CSL_IOMUX_PR0_PRU1_GPO4_CFG_REG_HSMASTER_SHIFT                         (0x0000001FU)
#define CSL_IOMUX_PR0_PRU1_GPO4_CFG_REG_HSMASTER_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO4_CFG_REG_HSMASTER_MAX                           (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO4_CFG_REG_RESETVAL                               (0x000005F7U)

/* PR0_PRU1_GPO0_CFG_REG */

#define CSL_IOMUX_PR0_PRU1_GPO0_CFG_REG_FUNC_SEL_MASK                          (0x0000000FU)
#define CSL_IOMUX_PR0_PRU1_GPO0_CFG_REG_FUNC_SEL_SHIFT                         (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO0_CFG_REG_FUNC_SEL_RESETVAL                      (0x00000007U)
#define CSL_IOMUX_PR0_PRU1_GPO0_CFG_REG_FUNC_SEL_MAX                           (0x0000000FU)

#define CSL_IOMUX_PR0_PRU1_GPO0_CFG_REG_IE_OVERRIDE_CTRL_MASK                  (0x00000010U)
#define CSL_IOMUX_PR0_PRU1_GPO0_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                 (0x00000004U)
#define CSL_IOMUX_PR0_PRU1_GPO0_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL              (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO0_CFG_REG_IE_OVERRIDE_CTRL_MAX                   (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO0_CFG_REG_IE_OVERRIDE_MASK                       (0x00000020U)
#define CSL_IOMUX_PR0_PRU1_GPO0_CFG_REG_IE_OVERRIDE_SHIFT                      (0x00000005U)
#define CSL_IOMUX_PR0_PRU1_GPO0_CFG_REG_IE_OVERRIDE_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO0_CFG_REG_IE_OVERRIDE_MAX                        (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO0_CFG_REG_OE_OVERRIDE_CTRL_MASK                  (0x00000040U)
#define CSL_IOMUX_PR0_PRU1_GPO0_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                 (0x00000006U)
#define CSL_IOMUX_PR0_PRU1_GPO0_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL              (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO0_CFG_REG_OE_OVERRIDE_CTRL_MAX                   (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO0_CFG_REG_OE_OVERRIDE_MASK                       (0x00000080U)
#define CSL_IOMUX_PR0_PRU1_GPO0_CFG_REG_OE_OVERRIDE_SHIFT                      (0x00000007U)
#define CSL_IOMUX_PR0_PRU1_GPO0_CFG_REG_OE_OVERRIDE_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO0_CFG_REG_OE_OVERRIDE_MAX                        (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO0_CFG_REG_PI_MASK                                (0x00000100U)
#define CSL_IOMUX_PR0_PRU1_GPO0_CFG_REG_PI_SHIFT                               (0x00000008U)
#define CSL_IOMUX_PR0_PRU1_GPO0_CFG_REG_PI_RESETVAL                            (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO0_CFG_REG_PI_MAX                                 (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO0_CFG_REG_PUPDSEL_MASK                           (0x00000200U)
#define CSL_IOMUX_PR0_PRU1_GPO0_CFG_REG_PUPDSEL_SHIFT                          (0x00000009U)
#define CSL_IOMUX_PR0_PRU1_GPO0_CFG_REG_PUPDSEL_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO0_CFG_REG_PUPDSEL_MAX                            (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO0_CFG_REG_SC1_MASK                               (0x00000400U)
#define CSL_IOMUX_PR0_PRU1_GPO0_CFG_REG_SC1_SHIFT                              (0x0000000AU)
#define CSL_IOMUX_PR0_PRU1_GPO0_CFG_REG_SC1_RESETVAL                           (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO0_CFG_REG_SC1_MAX                                (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO0_CFG_REG_GPIO_SEL_MASK                          (0x00030000U)
#define CSL_IOMUX_PR0_PRU1_GPO0_CFG_REG_GPIO_SEL_SHIFT                         (0x00000010U)
#define CSL_IOMUX_PR0_PRU1_GPO0_CFG_REG_GPIO_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO0_CFG_REG_GPIO_SEL_MAX                           (0x00000003U)

#define CSL_IOMUX_PR0_PRU1_GPO0_CFG_REG_QUAL_SEL_MASK                          (0x000C0000U)
#define CSL_IOMUX_PR0_PRU1_GPO0_CFG_REG_QUAL_SEL_SHIFT                         (0x00000012U)
#define CSL_IOMUX_PR0_PRU1_GPO0_CFG_REG_QUAL_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO0_CFG_REG_QUAL_SEL_MAX                           (0x00000003U)

#define CSL_IOMUX_PR0_PRU1_GPO0_CFG_REG_INP_INV_SEL_MASK                       (0x00100000U)
#define CSL_IOMUX_PR0_PRU1_GPO0_CFG_REG_INP_INV_SEL_SHIFT                      (0x00000014U)
#define CSL_IOMUX_PR0_PRU1_GPO0_CFG_REG_INP_INV_SEL_RESETVAL                   (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO0_CFG_REG_INP_INV_SEL_MAX                        (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO0_CFG_REG_ICSSM_GPIO_SEL_MASK                    (0x00200000U)
#define CSL_IOMUX_PR0_PRU1_GPO0_CFG_REG_ICSSM_GPIO_SEL_SHIFT                   (0x00000015U)
#define CSL_IOMUX_PR0_PRU1_GPO0_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO0_CFG_REG_ICSSM_GPIO_SEL_MAX                     (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO0_CFG_REG_SAFETY_OVERRIDE_SEL_MASK               (0x00400000U)
#define CSL_IOMUX_PR0_PRU1_GPO0_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT              (0x00000016U)
#define CSL_IOMUX_PR0_PRU1_GPO0_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL           (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO0_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO0_CFG_REG_HSMODE_MASK                            (0x40000000U)
#define CSL_IOMUX_PR0_PRU1_GPO0_CFG_REG_HSMODE_SHIFT                           (0x0000001EU)
#define CSL_IOMUX_PR0_PRU1_GPO0_CFG_REG_HSMODE_RESETVAL                        (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO0_CFG_REG_HSMODE_MAX                             (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO0_CFG_REG_HSMASTER_MASK                          (0x80000000U)
#define CSL_IOMUX_PR0_PRU1_GPO0_CFG_REG_HSMASTER_SHIFT                         (0x0000001FU)
#define CSL_IOMUX_PR0_PRU1_GPO0_CFG_REG_HSMASTER_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO0_CFG_REG_HSMASTER_MAX                           (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO0_CFG_REG_RESETVAL                               (0x000005F7U)

/* PR0_PRU1_GPO1_CFG_REG */

#define CSL_IOMUX_PR0_PRU1_GPO1_CFG_REG_FUNC_SEL_MASK                          (0x0000000FU)
#define CSL_IOMUX_PR0_PRU1_GPO1_CFG_REG_FUNC_SEL_SHIFT                         (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO1_CFG_REG_FUNC_SEL_RESETVAL                      (0x00000007U)
#define CSL_IOMUX_PR0_PRU1_GPO1_CFG_REG_FUNC_SEL_MAX                           (0x0000000FU)

#define CSL_IOMUX_PR0_PRU1_GPO1_CFG_REG_IE_OVERRIDE_CTRL_MASK                  (0x00000010U)
#define CSL_IOMUX_PR0_PRU1_GPO1_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                 (0x00000004U)
#define CSL_IOMUX_PR0_PRU1_GPO1_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL              (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO1_CFG_REG_IE_OVERRIDE_CTRL_MAX                   (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO1_CFG_REG_IE_OVERRIDE_MASK                       (0x00000020U)
#define CSL_IOMUX_PR0_PRU1_GPO1_CFG_REG_IE_OVERRIDE_SHIFT                      (0x00000005U)
#define CSL_IOMUX_PR0_PRU1_GPO1_CFG_REG_IE_OVERRIDE_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO1_CFG_REG_IE_OVERRIDE_MAX                        (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO1_CFG_REG_OE_OVERRIDE_CTRL_MASK                  (0x00000040U)
#define CSL_IOMUX_PR0_PRU1_GPO1_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                 (0x00000006U)
#define CSL_IOMUX_PR0_PRU1_GPO1_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL              (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO1_CFG_REG_OE_OVERRIDE_CTRL_MAX                   (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO1_CFG_REG_OE_OVERRIDE_MASK                       (0x00000080U)
#define CSL_IOMUX_PR0_PRU1_GPO1_CFG_REG_OE_OVERRIDE_SHIFT                      (0x00000007U)
#define CSL_IOMUX_PR0_PRU1_GPO1_CFG_REG_OE_OVERRIDE_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO1_CFG_REG_OE_OVERRIDE_MAX                        (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO1_CFG_REG_PI_MASK                                (0x00000100U)
#define CSL_IOMUX_PR0_PRU1_GPO1_CFG_REG_PI_SHIFT                               (0x00000008U)
#define CSL_IOMUX_PR0_PRU1_GPO1_CFG_REG_PI_RESETVAL                            (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO1_CFG_REG_PI_MAX                                 (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO1_CFG_REG_PUPDSEL_MASK                           (0x00000200U)
#define CSL_IOMUX_PR0_PRU1_GPO1_CFG_REG_PUPDSEL_SHIFT                          (0x00000009U)
#define CSL_IOMUX_PR0_PRU1_GPO1_CFG_REG_PUPDSEL_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO1_CFG_REG_PUPDSEL_MAX                            (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO1_CFG_REG_SC1_MASK                               (0x00000400U)
#define CSL_IOMUX_PR0_PRU1_GPO1_CFG_REG_SC1_SHIFT                              (0x0000000AU)
#define CSL_IOMUX_PR0_PRU1_GPO1_CFG_REG_SC1_RESETVAL                           (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO1_CFG_REG_SC1_MAX                                (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO1_CFG_REG_GPIO_SEL_MASK                          (0x00030000U)
#define CSL_IOMUX_PR0_PRU1_GPO1_CFG_REG_GPIO_SEL_SHIFT                         (0x00000010U)
#define CSL_IOMUX_PR0_PRU1_GPO1_CFG_REG_GPIO_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO1_CFG_REG_GPIO_SEL_MAX                           (0x00000003U)

#define CSL_IOMUX_PR0_PRU1_GPO1_CFG_REG_QUAL_SEL_MASK                          (0x000C0000U)
#define CSL_IOMUX_PR0_PRU1_GPO1_CFG_REG_QUAL_SEL_SHIFT                         (0x00000012U)
#define CSL_IOMUX_PR0_PRU1_GPO1_CFG_REG_QUAL_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO1_CFG_REG_QUAL_SEL_MAX                           (0x00000003U)

#define CSL_IOMUX_PR0_PRU1_GPO1_CFG_REG_INP_INV_SEL_MASK                       (0x00100000U)
#define CSL_IOMUX_PR0_PRU1_GPO1_CFG_REG_INP_INV_SEL_SHIFT                      (0x00000014U)
#define CSL_IOMUX_PR0_PRU1_GPO1_CFG_REG_INP_INV_SEL_RESETVAL                   (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO1_CFG_REG_INP_INV_SEL_MAX                        (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO1_CFG_REG_ICSSM_GPIO_SEL_MASK                    (0x00200000U)
#define CSL_IOMUX_PR0_PRU1_GPO1_CFG_REG_ICSSM_GPIO_SEL_SHIFT                   (0x00000015U)
#define CSL_IOMUX_PR0_PRU1_GPO1_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO1_CFG_REG_ICSSM_GPIO_SEL_MAX                     (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO1_CFG_REG_SAFETY_OVERRIDE_SEL_MASK               (0x00400000U)
#define CSL_IOMUX_PR0_PRU1_GPO1_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT              (0x00000016U)
#define CSL_IOMUX_PR0_PRU1_GPO1_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL           (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO1_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO1_CFG_REG_HSMODE_MASK                            (0x40000000U)
#define CSL_IOMUX_PR0_PRU1_GPO1_CFG_REG_HSMODE_SHIFT                           (0x0000001EU)
#define CSL_IOMUX_PR0_PRU1_GPO1_CFG_REG_HSMODE_RESETVAL                        (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO1_CFG_REG_HSMODE_MAX                             (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO1_CFG_REG_HSMASTER_MASK                          (0x80000000U)
#define CSL_IOMUX_PR0_PRU1_GPO1_CFG_REG_HSMASTER_SHIFT                         (0x0000001FU)
#define CSL_IOMUX_PR0_PRU1_GPO1_CFG_REG_HSMASTER_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO1_CFG_REG_HSMASTER_MAX                           (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO1_CFG_REG_RESETVAL                               (0x000005F7U)

/* PR0_PRU1_GPO2_CFG_REG */

#define CSL_IOMUX_PR0_PRU1_GPO2_CFG_REG_FUNC_SEL_MASK                          (0x0000000FU)
#define CSL_IOMUX_PR0_PRU1_GPO2_CFG_REG_FUNC_SEL_SHIFT                         (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO2_CFG_REG_FUNC_SEL_RESETVAL                      (0x00000007U)
#define CSL_IOMUX_PR0_PRU1_GPO2_CFG_REG_FUNC_SEL_MAX                           (0x0000000FU)

#define CSL_IOMUX_PR0_PRU1_GPO2_CFG_REG_IE_OVERRIDE_CTRL_MASK                  (0x00000010U)
#define CSL_IOMUX_PR0_PRU1_GPO2_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                 (0x00000004U)
#define CSL_IOMUX_PR0_PRU1_GPO2_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL              (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO2_CFG_REG_IE_OVERRIDE_CTRL_MAX                   (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO2_CFG_REG_IE_OVERRIDE_MASK                       (0x00000020U)
#define CSL_IOMUX_PR0_PRU1_GPO2_CFG_REG_IE_OVERRIDE_SHIFT                      (0x00000005U)
#define CSL_IOMUX_PR0_PRU1_GPO2_CFG_REG_IE_OVERRIDE_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO2_CFG_REG_IE_OVERRIDE_MAX                        (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO2_CFG_REG_OE_OVERRIDE_CTRL_MASK                  (0x00000040U)
#define CSL_IOMUX_PR0_PRU1_GPO2_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                 (0x00000006U)
#define CSL_IOMUX_PR0_PRU1_GPO2_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL              (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO2_CFG_REG_OE_OVERRIDE_CTRL_MAX                   (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO2_CFG_REG_OE_OVERRIDE_MASK                       (0x00000080U)
#define CSL_IOMUX_PR0_PRU1_GPO2_CFG_REG_OE_OVERRIDE_SHIFT                      (0x00000007U)
#define CSL_IOMUX_PR0_PRU1_GPO2_CFG_REG_OE_OVERRIDE_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO2_CFG_REG_OE_OVERRIDE_MAX                        (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO2_CFG_REG_PI_MASK                                (0x00000100U)
#define CSL_IOMUX_PR0_PRU1_GPO2_CFG_REG_PI_SHIFT                               (0x00000008U)
#define CSL_IOMUX_PR0_PRU1_GPO2_CFG_REG_PI_RESETVAL                            (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO2_CFG_REG_PI_MAX                                 (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO2_CFG_REG_PUPDSEL_MASK                           (0x00000200U)
#define CSL_IOMUX_PR0_PRU1_GPO2_CFG_REG_PUPDSEL_SHIFT                          (0x00000009U)
#define CSL_IOMUX_PR0_PRU1_GPO2_CFG_REG_PUPDSEL_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO2_CFG_REG_PUPDSEL_MAX                            (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO2_CFG_REG_SC1_MASK                               (0x00000400U)
#define CSL_IOMUX_PR0_PRU1_GPO2_CFG_REG_SC1_SHIFT                              (0x0000000AU)
#define CSL_IOMUX_PR0_PRU1_GPO2_CFG_REG_SC1_RESETVAL                           (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO2_CFG_REG_SC1_MAX                                (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO2_CFG_REG_GPIO_SEL_MASK                          (0x00030000U)
#define CSL_IOMUX_PR0_PRU1_GPO2_CFG_REG_GPIO_SEL_SHIFT                         (0x00000010U)
#define CSL_IOMUX_PR0_PRU1_GPO2_CFG_REG_GPIO_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO2_CFG_REG_GPIO_SEL_MAX                           (0x00000003U)

#define CSL_IOMUX_PR0_PRU1_GPO2_CFG_REG_QUAL_SEL_MASK                          (0x000C0000U)
#define CSL_IOMUX_PR0_PRU1_GPO2_CFG_REG_QUAL_SEL_SHIFT                         (0x00000012U)
#define CSL_IOMUX_PR0_PRU1_GPO2_CFG_REG_QUAL_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO2_CFG_REG_QUAL_SEL_MAX                           (0x00000003U)

#define CSL_IOMUX_PR0_PRU1_GPO2_CFG_REG_INP_INV_SEL_MASK                       (0x00100000U)
#define CSL_IOMUX_PR0_PRU1_GPO2_CFG_REG_INP_INV_SEL_SHIFT                      (0x00000014U)
#define CSL_IOMUX_PR0_PRU1_GPO2_CFG_REG_INP_INV_SEL_RESETVAL                   (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO2_CFG_REG_INP_INV_SEL_MAX                        (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO2_CFG_REG_ICSSM_GPIO_SEL_MASK                    (0x00200000U)
#define CSL_IOMUX_PR0_PRU1_GPO2_CFG_REG_ICSSM_GPIO_SEL_SHIFT                   (0x00000015U)
#define CSL_IOMUX_PR0_PRU1_GPO2_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO2_CFG_REG_ICSSM_GPIO_SEL_MAX                     (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO2_CFG_REG_SAFETY_OVERRIDE_SEL_MASK               (0x00400000U)
#define CSL_IOMUX_PR0_PRU1_GPO2_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT              (0x00000016U)
#define CSL_IOMUX_PR0_PRU1_GPO2_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL           (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO2_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO2_CFG_REG_HSMODE_MASK                            (0x40000000U)
#define CSL_IOMUX_PR0_PRU1_GPO2_CFG_REG_HSMODE_SHIFT                           (0x0000001EU)
#define CSL_IOMUX_PR0_PRU1_GPO2_CFG_REG_HSMODE_RESETVAL                        (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO2_CFG_REG_HSMODE_MAX                             (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO2_CFG_REG_HSMASTER_MASK                          (0x80000000U)
#define CSL_IOMUX_PR0_PRU1_GPO2_CFG_REG_HSMASTER_SHIFT                         (0x0000001FU)
#define CSL_IOMUX_PR0_PRU1_GPO2_CFG_REG_HSMASTER_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO2_CFG_REG_HSMASTER_MAX                           (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO2_CFG_REG_RESETVAL                               (0x000005F7U)

/* PR0_PRU1_GPO3_CFG_REG */

#define CSL_IOMUX_PR0_PRU1_GPO3_CFG_REG_FUNC_SEL_MASK                          (0x0000000FU)
#define CSL_IOMUX_PR0_PRU1_GPO3_CFG_REG_FUNC_SEL_SHIFT                         (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO3_CFG_REG_FUNC_SEL_RESETVAL                      (0x00000007U)
#define CSL_IOMUX_PR0_PRU1_GPO3_CFG_REG_FUNC_SEL_MAX                           (0x0000000FU)

#define CSL_IOMUX_PR0_PRU1_GPO3_CFG_REG_IE_OVERRIDE_CTRL_MASK                  (0x00000010U)
#define CSL_IOMUX_PR0_PRU1_GPO3_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                 (0x00000004U)
#define CSL_IOMUX_PR0_PRU1_GPO3_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL              (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO3_CFG_REG_IE_OVERRIDE_CTRL_MAX                   (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO3_CFG_REG_IE_OVERRIDE_MASK                       (0x00000020U)
#define CSL_IOMUX_PR0_PRU1_GPO3_CFG_REG_IE_OVERRIDE_SHIFT                      (0x00000005U)
#define CSL_IOMUX_PR0_PRU1_GPO3_CFG_REG_IE_OVERRIDE_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO3_CFG_REG_IE_OVERRIDE_MAX                        (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO3_CFG_REG_OE_OVERRIDE_CTRL_MASK                  (0x00000040U)
#define CSL_IOMUX_PR0_PRU1_GPO3_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                 (0x00000006U)
#define CSL_IOMUX_PR0_PRU1_GPO3_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL              (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO3_CFG_REG_OE_OVERRIDE_CTRL_MAX                   (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO3_CFG_REG_OE_OVERRIDE_MASK                       (0x00000080U)
#define CSL_IOMUX_PR0_PRU1_GPO3_CFG_REG_OE_OVERRIDE_SHIFT                      (0x00000007U)
#define CSL_IOMUX_PR0_PRU1_GPO3_CFG_REG_OE_OVERRIDE_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO3_CFG_REG_OE_OVERRIDE_MAX                        (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO3_CFG_REG_PI_MASK                                (0x00000100U)
#define CSL_IOMUX_PR0_PRU1_GPO3_CFG_REG_PI_SHIFT                               (0x00000008U)
#define CSL_IOMUX_PR0_PRU1_GPO3_CFG_REG_PI_RESETVAL                            (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO3_CFG_REG_PI_MAX                                 (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO3_CFG_REG_PUPDSEL_MASK                           (0x00000200U)
#define CSL_IOMUX_PR0_PRU1_GPO3_CFG_REG_PUPDSEL_SHIFT                          (0x00000009U)
#define CSL_IOMUX_PR0_PRU1_GPO3_CFG_REG_PUPDSEL_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO3_CFG_REG_PUPDSEL_MAX                            (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO3_CFG_REG_SC1_MASK                               (0x00000400U)
#define CSL_IOMUX_PR0_PRU1_GPO3_CFG_REG_SC1_SHIFT                              (0x0000000AU)
#define CSL_IOMUX_PR0_PRU1_GPO3_CFG_REG_SC1_RESETVAL                           (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO3_CFG_REG_SC1_MAX                                (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO3_CFG_REG_GPIO_SEL_MASK                          (0x00030000U)
#define CSL_IOMUX_PR0_PRU1_GPO3_CFG_REG_GPIO_SEL_SHIFT                         (0x00000010U)
#define CSL_IOMUX_PR0_PRU1_GPO3_CFG_REG_GPIO_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO3_CFG_REG_GPIO_SEL_MAX                           (0x00000003U)

#define CSL_IOMUX_PR0_PRU1_GPO3_CFG_REG_QUAL_SEL_MASK                          (0x000C0000U)
#define CSL_IOMUX_PR0_PRU1_GPO3_CFG_REG_QUAL_SEL_SHIFT                         (0x00000012U)
#define CSL_IOMUX_PR0_PRU1_GPO3_CFG_REG_QUAL_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO3_CFG_REG_QUAL_SEL_MAX                           (0x00000003U)

#define CSL_IOMUX_PR0_PRU1_GPO3_CFG_REG_INP_INV_SEL_MASK                       (0x00100000U)
#define CSL_IOMUX_PR0_PRU1_GPO3_CFG_REG_INP_INV_SEL_SHIFT                      (0x00000014U)
#define CSL_IOMUX_PR0_PRU1_GPO3_CFG_REG_INP_INV_SEL_RESETVAL                   (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO3_CFG_REG_INP_INV_SEL_MAX                        (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO3_CFG_REG_ICSSM_GPIO_SEL_MASK                    (0x00200000U)
#define CSL_IOMUX_PR0_PRU1_GPO3_CFG_REG_ICSSM_GPIO_SEL_SHIFT                   (0x00000015U)
#define CSL_IOMUX_PR0_PRU1_GPO3_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO3_CFG_REG_ICSSM_GPIO_SEL_MAX                     (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO3_CFG_REG_SAFETY_OVERRIDE_SEL_MASK               (0x00400000U)
#define CSL_IOMUX_PR0_PRU1_GPO3_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT              (0x00000016U)
#define CSL_IOMUX_PR0_PRU1_GPO3_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL           (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO3_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO3_CFG_REG_HSMODE_MASK                            (0x40000000U)
#define CSL_IOMUX_PR0_PRU1_GPO3_CFG_REG_HSMODE_SHIFT                           (0x0000001EU)
#define CSL_IOMUX_PR0_PRU1_GPO3_CFG_REG_HSMODE_RESETVAL                        (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO3_CFG_REG_HSMODE_MAX                             (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO3_CFG_REG_HSMASTER_MASK                          (0x80000000U)
#define CSL_IOMUX_PR0_PRU1_GPO3_CFG_REG_HSMASTER_SHIFT                         (0x0000001FU)
#define CSL_IOMUX_PR0_PRU1_GPO3_CFG_REG_HSMASTER_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO3_CFG_REG_HSMASTER_MAX                           (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO3_CFG_REG_RESETVAL                               (0x000005F7U)

/* PR0_PRU1_GPO16_CFG_REG */

#define CSL_IOMUX_PR0_PRU1_GPO16_CFG_REG_FUNC_SEL_MASK                         (0x0000000FU)
#define CSL_IOMUX_PR0_PRU1_GPO16_CFG_REG_FUNC_SEL_SHIFT                        (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO16_CFG_REG_FUNC_SEL_RESETVAL                     (0x00000007U)
#define CSL_IOMUX_PR0_PRU1_GPO16_CFG_REG_FUNC_SEL_MAX                          (0x0000000FU)

#define CSL_IOMUX_PR0_PRU1_GPO16_CFG_REG_IE_OVERRIDE_CTRL_MASK                 (0x00000010U)
#define CSL_IOMUX_PR0_PRU1_GPO16_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                (0x00000004U)
#define CSL_IOMUX_PR0_PRU1_GPO16_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL             (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO16_CFG_REG_IE_OVERRIDE_CTRL_MAX                  (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO16_CFG_REG_IE_OVERRIDE_MASK                      (0x00000020U)
#define CSL_IOMUX_PR0_PRU1_GPO16_CFG_REG_IE_OVERRIDE_SHIFT                     (0x00000005U)
#define CSL_IOMUX_PR0_PRU1_GPO16_CFG_REG_IE_OVERRIDE_RESETVAL                  (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO16_CFG_REG_IE_OVERRIDE_MAX                       (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO16_CFG_REG_OE_OVERRIDE_CTRL_MASK                 (0x00000040U)
#define CSL_IOMUX_PR0_PRU1_GPO16_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                (0x00000006U)
#define CSL_IOMUX_PR0_PRU1_GPO16_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL             (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO16_CFG_REG_OE_OVERRIDE_CTRL_MAX                  (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO16_CFG_REG_OE_OVERRIDE_MASK                      (0x00000080U)
#define CSL_IOMUX_PR0_PRU1_GPO16_CFG_REG_OE_OVERRIDE_SHIFT                     (0x00000007U)
#define CSL_IOMUX_PR0_PRU1_GPO16_CFG_REG_OE_OVERRIDE_RESETVAL                  (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO16_CFG_REG_OE_OVERRIDE_MAX                       (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO16_CFG_REG_PI_MASK                               (0x00000100U)
#define CSL_IOMUX_PR0_PRU1_GPO16_CFG_REG_PI_SHIFT                              (0x00000008U)
#define CSL_IOMUX_PR0_PRU1_GPO16_CFG_REG_PI_RESETVAL                           (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO16_CFG_REG_PI_MAX                                (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO16_CFG_REG_PUPDSEL_MASK                          (0x00000200U)
#define CSL_IOMUX_PR0_PRU1_GPO16_CFG_REG_PUPDSEL_SHIFT                         (0x00000009U)
#define CSL_IOMUX_PR0_PRU1_GPO16_CFG_REG_PUPDSEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO16_CFG_REG_PUPDSEL_MAX                           (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO16_CFG_REG_SC1_MASK                              (0x00000400U)
#define CSL_IOMUX_PR0_PRU1_GPO16_CFG_REG_SC1_SHIFT                             (0x0000000AU)
#define CSL_IOMUX_PR0_PRU1_GPO16_CFG_REG_SC1_RESETVAL                          (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO16_CFG_REG_SC1_MAX                               (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO16_CFG_REG_GPIO_SEL_MASK                         (0x00030000U)
#define CSL_IOMUX_PR0_PRU1_GPO16_CFG_REG_GPIO_SEL_SHIFT                        (0x00000010U)
#define CSL_IOMUX_PR0_PRU1_GPO16_CFG_REG_GPIO_SEL_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO16_CFG_REG_GPIO_SEL_MAX                          (0x00000003U)

#define CSL_IOMUX_PR0_PRU1_GPO16_CFG_REG_QUAL_SEL_MASK                         (0x000C0000U)
#define CSL_IOMUX_PR0_PRU1_GPO16_CFG_REG_QUAL_SEL_SHIFT                        (0x00000012U)
#define CSL_IOMUX_PR0_PRU1_GPO16_CFG_REG_QUAL_SEL_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO16_CFG_REG_QUAL_SEL_MAX                          (0x00000003U)

#define CSL_IOMUX_PR0_PRU1_GPO16_CFG_REG_INP_INV_SEL_MASK                      (0x00100000U)
#define CSL_IOMUX_PR0_PRU1_GPO16_CFG_REG_INP_INV_SEL_SHIFT                     (0x00000014U)
#define CSL_IOMUX_PR0_PRU1_GPO16_CFG_REG_INP_INV_SEL_RESETVAL                  (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO16_CFG_REG_INP_INV_SEL_MAX                       (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO16_CFG_REG_ICSSM_GPIO_SEL_MASK                   (0x00200000U)
#define CSL_IOMUX_PR0_PRU1_GPO16_CFG_REG_ICSSM_GPIO_SEL_SHIFT                  (0x00000015U)
#define CSL_IOMUX_PR0_PRU1_GPO16_CFG_REG_ICSSM_GPIO_SEL_RESETVAL               (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO16_CFG_REG_ICSSM_GPIO_SEL_MAX                    (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO16_CFG_REG_SAFETY_OVERRIDE_SEL_MASK              (0x00400000U)
#define CSL_IOMUX_PR0_PRU1_GPO16_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT             (0x00000016U)
#define CSL_IOMUX_PR0_PRU1_GPO16_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL          (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO16_CFG_REG_SAFETY_OVERRIDE_SEL_MAX               (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO16_CFG_REG_HSMODE_MASK                           (0x40000000U)
#define CSL_IOMUX_PR0_PRU1_GPO16_CFG_REG_HSMODE_SHIFT                          (0x0000001EU)
#define CSL_IOMUX_PR0_PRU1_GPO16_CFG_REG_HSMODE_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO16_CFG_REG_HSMODE_MAX                            (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO16_CFG_REG_HSMASTER_MASK                         (0x80000000U)
#define CSL_IOMUX_PR0_PRU1_GPO16_CFG_REG_HSMASTER_SHIFT                        (0x0000001FU)
#define CSL_IOMUX_PR0_PRU1_GPO16_CFG_REG_HSMASTER_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO16_CFG_REG_HSMASTER_MAX                          (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO16_CFG_REG_RESETVAL                              (0x000005F7U)

/* PR0_PRU1_GPO15_CFG_REG */

#define CSL_IOMUX_PR0_PRU1_GPO15_CFG_REG_FUNC_SEL_MASK                         (0x0000000FU)
#define CSL_IOMUX_PR0_PRU1_GPO15_CFG_REG_FUNC_SEL_SHIFT                        (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO15_CFG_REG_FUNC_SEL_RESETVAL                     (0x00000007U)
#define CSL_IOMUX_PR0_PRU1_GPO15_CFG_REG_FUNC_SEL_MAX                          (0x0000000FU)

#define CSL_IOMUX_PR0_PRU1_GPO15_CFG_REG_IE_OVERRIDE_CTRL_MASK                 (0x00000010U)
#define CSL_IOMUX_PR0_PRU1_GPO15_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                (0x00000004U)
#define CSL_IOMUX_PR0_PRU1_GPO15_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL             (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO15_CFG_REG_IE_OVERRIDE_CTRL_MAX                  (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO15_CFG_REG_IE_OVERRIDE_MASK                      (0x00000020U)
#define CSL_IOMUX_PR0_PRU1_GPO15_CFG_REG_IE_OVERRIDE_SHIFT                     (0x00000005U)
#define CSL_IOMUX_PR0_PRU1_GPO15_CFG_REG_IE_OVERRIDE_RESETVAL                  (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO15_CFG_REG_IE_OVERRIDE_MAX                       (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO15_CFG_REG_OE_OVERRIDE_CTRL_MASK                 (0x00000040U)
#define CSL_IOMUX_PR0_PRU1_GPO15_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                (0x00000006U)
#define CSL_IOMUX_PR0_PRU1_GPO15_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL             (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO15_CFG_REG_OE_OVERRIDE_CTRL_MAX                  (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO15_CFG_REG_OE_OVERRIDE_MASK                      (0x00000080U)
#define CSL_IOMUX_PR0_PRU1_GPO15_CFG_REG_OE_OVERRIDE_SHIFT                     (0x00000007U)
#define CSL_IOMUX_PR0_PRU1_GPO15_CFG_REG_OE_OVERRIDE_RESETVAL                  (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO15_CFG_REG_OE_OVERRIDE_MAX                       (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO15_CFG_REG_PI_MASK                               (0x00000100U)
#define CSL_IOMUX_PR0_PRU1_GPO15_CFG_REG_PI_SHIFT                              (0x00000008U)
#define CSL_IOMUX_PR0_PRU1_GPO15_CFG_REG_PI_RESETVAL                           (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO15_CFG_REG_PI_MAX                                (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO15_CFG_REG_PUPDSEL_MASK                          (0x00000200U)
#define CSL_IOMUX_PR0_PRU1_GPO15_CFG_REG_PUPDSEL_SHIFT                         (0x00000009U)
#define CSL_IOMUX_PR0_PRU1_GPO15_CFG_REG_PUPDSEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO15_CFG_REG_PUPDSEL_MAX                           (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO15_CFG_REG_SC1_MASK                              (0x00000400U)
#define CSL_IOMUX_PR0_PRU1_GPO15_CFG_REG_SC1_SHIFT                             (0x0000000AU)
#define CSL_IOMUX_PR0_PRU1_GPO15_CFG_REG_SC1_RESETVAL                          (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO15_CFG_REG_SC1_MAX                               (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO15_CFG_REG_GPIO_SEL_MASK                         (0x00030000U)
#define CSL_IOMUX_PR0_PRU1_GPO15_CFG_REG_GPIO_SEL_SHIFT                        (0x00000010U)
#define CSL_IOMUX_PR0_PRU1_GPO15_CFG_REG_GPIO_SEL_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO15_CFG_REG_GPIO_SEL_MAX                          (0x00000003U)

#define CSL_IOMUX_PR0_PRU1_GPO15_CFG_REG_QUAL_SEL_MASK                         (0x000C0000U)
#define CSL_IOMUX_PR0_PRU1_GPO15_CFG_REG_QUAL_SEL_SHIFT                        (0x00000012U)
#define CSL_IOMUX_PR0_PRU1_GPO15_CFG_REG_QUAL_SEL_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO15_CFG_REG_QUAL_SEL_MAX                          (0x00000003U)

#define CSL_IOMUX_PR0_PRU1_GPO15_CFG_REG_INP_INV_SEL_MASK                      (0x00100000U)
#define CSL_IOMUX_PR0_PRU1_GPO15_CFG_REG_INP_INV_SEL_SHIFT                     (0x00000014U)
#define CSL_IOMUX_PR0_PRU1_GPO15_CFG_REG_INP_INV_SEL_RESETVAL                  (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO15_CFG_REG_INP_INV_SEL_MAX                       (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO15_CFG_REG_ICSSM_GPIO_SEL_MASK                   (0x00200000U)
#define CSL_IOMUX_PR0_PRU1_GPO15_CFG_REG_ICSSM_GPIO_SEL_SHIFT                  (0x00000015U)
#define CSL_IOMUX_PR0_PRU1_GPO15_CFG_REG_ICSSM_GPIO_SEL_RESETVAL               (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO15_CFG_REG_ICSSM_GPIO_SEL_MAX                    (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO15_CFG_REG_SAFETY_OVERRIDE_SEL_MASK              (0x00400000U)
#define CSL_IOMUX_PR0_PRU1_GPO15_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT             (0x00000016U)
#define CSL_IOMUX_PR0_PRU1_GPO15_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL          (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO15_CFG_REG_SAFETY_OVERRIDE_SEL_MAX               (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO15_CFG_REG_HSMODE_MASK                           (0x40000000U)
#define CSL_IOMUX_PR0_PRU1_GPO15_CFG_REG_HSMODE_SHIFT                          (0x0000001EU)
#define CSL_IOMUX_PR0_PRU1_GPO15_CFG_REG_HSMODE_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO15_CFG_REG_HSMODE_MAX                            (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO15_CFG_REG_HSMASTER_MASK                         (0x80000000U)
#define CSL_IOMUX_PR0_PRU1_GPO15_CFG_REG_HSMASTER_SHIFT                        (0x0000001FU)
#define CSL_IOMUX_PR0_PRU1_GPO15_CFG_REG_HSMASTER_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO15_CFG_REG_HSMASTER_MAX                          (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO15_CFG_REG_RESETVAL                              (0x000005F7U)

/* PR0_PRU1_GPO11_CFG_REG */

#define CSL_IOMUX_PR0_PRU1_GPO11_CFG_REG_FUNC_SEL_MASK                         (0x0000000FU)
#define CSL_IOMUX_PR0_PRU1_GPO11_CFG_REG_FUNC_SEL_SHIFT                        (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO11_CFG_REG_FUNC_SEL_RESETVAL                     (0x00000007U)
#define CSL_IOMUX_PR0_PRU1_GPO11_CFG_REG_FUNC_SEL_MAX                          (0x0000000FU)

#define CSL_IOMUX_PR0_PRU1_GPO11_CFG_REG_IE_OVERRIDE_CTRL_MASK                 (0x00000010U)
#define CSL_IOMUX_PR0_PRU1_GPO11_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                (0x00000004U)
#define CSL_IOMUX_PR0_PRU1_GPO11_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL             (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO11_CFG_REG_IE_OVERRIDE_CTRL_MAX                  (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO11_CFG_REG_IE_OVERRIDE_MASK                      (0x00000020U)
#define CSL_IOMUX_PR0_PRU1_GPO11_CFG_REG_IE_OVERRIDE_SHIFT                     (0x00000005U)
#define CSL_IOMUX_PR0_PRU1_GPO11_CFG_REG_IE_OVERRIDE_RESETVAL                  (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO11_CFG_REG_IE_OVERRIDE_MAX                       (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO11_CFG_REG_OE_OVERRIDE_CTRL_MASK                 (0x00000040U)
#define CSL_IOMUX_PR0_PRU1_GPO11_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                (0x00000006U)
#define CSL_IOMUX_PR0_PRU1_GPO11_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL             (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO11_CFG_REG_OE_OVERRIDE_CTRL_MAX                  (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO11_CFG_REG_OE_OVERRIDE_MASK                      (0x00000080U)
#define CSL_IOMUX_PR0_PRU1_GPO11_CFG_REG_OE_OVERRIDE_SHIFT                     (0x00000007U)
#define CSL_IOMUX_PR0_PRU1_GPO11_CFG_REG_OE_OVERRIDE_RESETVAL                  (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO11_CFG_REG_OE_OVERRIDE_MAX                       (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO11_CFG_REG_PI_MASK                               (0x00000100U)
#define CSL_IOMUX_PR0_PRU1_GPO11_CFG_REG_PI_SHIFT                              (0x00000008U)
#define CSL_IOMUX_PR0_PRU1_GPO11_CFG_REG_PI_RESETVAL                           (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO11_CFG_REG_PI_MAX                                (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO11_CFG_REG_PUPDSEL_MASK                          (0x00000200U)
#define CSL_IOMUX_PR0_PRU1_GPO11_CFG_REG_PUPDSEL_SHIFT                         (0x00000009U)
#define CSL_IOMUX_PR0_PRU1_GPO11_CFG_REG_PUPDSEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO11_CFG_REG_PUPDSEL_MAX                           (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO11_CFG_REG_SC1_MASK                              (0x00000400U)
#define CSL_IOMUX_PR0_PRU1_GPO11_CFG_REG_SC1_SHIFT                             (0x0000000AU)
#define CSL_IOMUX_PR0_PRU1_GPO11_CFG_REG_SC1_RESETVAL                          (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO11_CFG_REG_SC1_MAX                               (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO11_CFG_REG_GPIO_SEL_MASK                         (0x00030000U)
#define CSL_IOMUX_PR0_PRU1_GPO11_CFG_REG_GPIO_SEL_SHIFT                        (0x00000010U)
#define CSL_IOMUX_PR0_PRU1_GPO11_CFG_REG_GPIO_SEL_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO11_CFG_REG_GPIO_SEL_MAX                          (0x00000003U)

#define CSL_IOMUX_PR0_PRU1_GPO11_CFG_REG_QUAL_SEL_MASK                         (0x000C0000U)
#define CSL_IOMUX_PR0_PRU1_GPO11_CFG_REG_QUAL_SEL_SHIFT                        (0x00000012U)
#define CSL_IOMUX_PR0_PRU1_GPO11_CFG_REG_QUAL_SEL_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO11_CFG_REG_QUAL_SEL_MAX                          (0x00000003U)

#define CSL_IOMUX_PR0_PRU1_GPO11_CFG_REG_INP_INV_SEL_MASK                      (0x00100000U)
#define CSL_IOMUX_PR0_PRU1_GPO11_CFG_REG_INP_INV_SEL_SHIFT                     (0x00000014U)
#define CSL_IOMUX_PR0_PRU1_GPO11_CFG_REG_INP_INV_SEL_RESETVAL                  (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO11_CFG_REG_INP_INV_SEL_MAX                       (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO11_CFG_REG_ICSSM_GPIO_SEL_MASK                   (0x00200000U)
#define CSL_IOMUX_PR0_PRU1_GPO11_CFG_REG_ICSSM_GPIO_SEL_SHIFT                  (0x00000015U)
#define CSL_IOMUX_PR0_PRU1_GPO11_CFG_REG_ICSSM_GPIO_SEL_RESETVAL               (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO11_CFG_REG_ICSSM_GPIO_SEL_MAX                    (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO11_CFG_REG_SAFETY_OVERRIDE_SEL_MASK              (0x00400000U)
#define CSL_IOMUX_PR0_PRU1_GPO11_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT             (0x00000016U)
#define CSL_IOMUX_PR0_PRU1_GPO11_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL          (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO11_CFG_REG_SAFETY_OVERRIDE_SEL_MAX               (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO11_CFG_REG_HSMODE_MASK                           (0x40000000U)
#define CSL_IOMUX_PR0_PRU1_GPO11_CFG_REG_HSMODE_SHIFT                          (0x0000001EU)
#define CSL_IOMUX_PR0_PRU1_GPO11_CFG_REG_HSMODE_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO11_CFG_REG_HSMODE_MAX                            (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO11_CFG_REG_HSMASTER_MASK                         (0x80000000U)
#define CSL_IOMUX_PR0_PRU1_GPO11_CFG_REG_HSMASTER_SHIFT                        (0x0000001FU)
#define CSL_IOMUX_PR0_PRU1_GPO11_CFG_REG_HSMASTER_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO11_CFG_REG_HSMASTER_MAX                          (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO11_CFG_REG_RESETVAL                              (0x000005F7U)

/* PR0_PRU1_GPO12_CFG_REG */

#define CSL_IOMUX_PR0_PRU1_GPO12_CFG_REG_FUNC_SEL_MASK                         (0x0000000FU)
#define CSL_IOMUX_PR0_PRU1_GPO12_CFG_REG_FUNC_SEL_SHIFT                        (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO12_CFG_REG_FUNC_SEL_RESETVAL                     (0x00000007U)
#define CSL_IOMUX_PR0_PRU1_GPO12_CFG_REG_FUNC_SEL_MAX                          (0x0000000FU)

#define CSL_IOMUX_PR0_PRU1_GPO12_CFG_REG_IE_OVERRIDE_CTRL_MASK                 (0x00000010U)
#define CSL_IOMUX_PR0_PRU1_GPO12_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                (0x00000004U)
#define CSL_IOMUX_PR0_PRU1_GPO12_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL             (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO12_CFG_REG_IE_OVERRIDE_CTRL_MAX                  (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO12_CFG_REG_IE_OVERRIDE_MASK                      (0x00000020U)
#define CSL_IOMUX_PR0_PRU1_GPO12_CFG_REG_IE_OVERRIDE_SHIFT                     (0x00000005U)
#define CSL_IOMUX_PR0_PRU1_GPO12_CFG_REG_IE_OVERRIDE_RESETVAL                  (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO12_CFG_REG_IE_OVERRIDE_MAX                       (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO12_CFG_REG_OE_OVERRIDE_CTRL_MASK                 (0x00000040U)
#define CSL_IOMUX_PR0_PRU1_GPO12_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                (0x00000006U)
#define CSL_IOMUX_PR0_PRU1_GPO12_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL             (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO12_CFG_REG_OE_OVERRIDE_CTRL_MAX                  (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO12_CFG_REG_OE_OVERRIDE_MASK                      (0x00000080U)
#define CSL_IOMUX_PR0_PRU1_GPO12_CFG_REG_OE_OVERRIDE_SHIFT                     (0x00000007U)
#define CSL_IOMUX_PR0_PRU1_GPO12_CFG_REG_OE_OVERRIDE_RESETVAL                  (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO12_CFG_REG_OE_OVERRIDE_MAX                       (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO12_CFG_REG_PI_MASK                               (0x00000100U)
#define CSL_IOMUX_PR0_PRU1_GPO12_CFG_REG_PI_SHIFT                              (0x00000008U)
#define CSL_IOMUX_PR0_PRU1_GPO12_CFG_REG_PI_RESETVAL                           (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO12_CFG_REG_PI_MAX                                (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO12_CFG_REG_PUPDSEL_MASK                          (0x00000200U)
#define CSL_IOMUX_PR0_PRU1_GPO12_CFG_REG_PUPDSEL_SHIFT                         (0x00000009U)
#define CSL_IOMUX_PR0_PRU1_GPO12_CFG_REG_PUPDSEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO12_CFG_REG_PUPDSEL_MAX                           (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO12_CFG_REG_SC1_MASK                              (0x00000400U)
#define CSL_IOMUX_PR0_PRU1_GPO12_CFG_REG_SC1_SHIFT                             (0x0000000AU)
#define CSL_IOMUX_PR0_PRU1_GPO12_CFG_REG_SC1_RESETVAL                          (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO12_CFG_REG_SC1_MAX                               (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO12_CFG_REG_GPIO_SEL_MASK                         (0x00030000U)
#define CSL_IOMUX_PR0_PRU1_GPO12_CFG_REG_GPIO_SEL_SHIFT                        (0x00000010U)
#define CSL_IOMUX_PR0_PRU1_GPO12_CFG_REG_GPIO_SEL_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO12_CFG_REG_GPIO_SEL_MAX                          (0x00000003U)

#define CSL_IOMUX_PR0_PRU1_GPO12_CFG_REG_QUAL_SEL_MASK                         (0x000C0000U)
#define CSL_IOMUX_PR0_PRU1_GPO12_CFG_REG_QUAL_SEL_SHIFT                        (0x00000012U)
#define CSL_IOMUX_PR0_PRU1_GPO12_CFG_REG_QUAL_SEL_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO12_CFG_REG_QUAL_SEL_MAX                          (0x00000003U)

#define CSL_IOMUX_PR0_PRU1_GPO12_CFG_REG_INP_INV_SEL_MASK                      (0x00100000U)
#define CSL_IOMUX_PR0_PRU1_GPO12_CFG_REG_INP_INV_SEL_SHIFT                     (0x00000014U)
#define CSL_IOMUX_PR0_PRU1_GPO12_CFG_REG_INP_INV_SEL_RESETVAL                  (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO12_CFG_REG_INP_INV_SEL_MAX                       (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO12_CFG_REG_ICSSM_GPIO_SEL_MASK                   (0x00200000U)
#define CSL_IOMUX_PR0_PRU1_GPO12_CFG_REG_ICSSM_GPIO_SEL_SHIFT                  (0x00000015U)
#define CSL_IOMUX_PR0_PRU1_GPO12_CFG_REG_ICSSM_GPIO_SEL_RESETVAL               (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO12_CFG_REG_ICSSM_GPIO_SEL_MAX                    (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO12_CFG_REG_SAFETY_OVERRIDE_SEL_MASK              (0x00400000U)
#define CSL_IOMUX_PR0_PRU1_GPO12_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT             (0x00000016U)
#define CSL_IOMUX_PR0_PRU1_GPO12_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL          (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO12_CFG_REG_SAFETY_OVERRIDE_SEL_MAX               (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO12_CFG_REG_HSMODE_MASK                           (0x40000000U)
#define CSL_IOMUX_PR0_PRU1_GPO12_CFG_REG_HSMODE_SHIFT                          (0x0000001EU)
#define CSL_IOMUX_PR0_PRU1_GPO12_CFG_REG_HSMODE_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO12_CFG_REG_HSMODE_MAX                            (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO12_CFG_REG_HSMASTER_MASK                         (0x80000000U)
#define CSL_IOMUX_PR0_PRU1_GPO12_CFG_REG_HSMASTER_SHIFT                        (0x0000001FU)
#define CSL_IOMUX_PR0_PRU1_GPO12_CFG_REG_HSMASTER_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO12_CFG_REG_HSMASTER_MAX                          (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO12_CFG_REG_RESETVAL                              (0x000005F7U)

/* PR0_PRU1_GPO13_CFG_REG */

#define CSL_IOMUX_PR0_PRU1_GPO13_CFG_REG_FUNC_SEL_MASK                         (0x0000000FU)
#define CSL_IOMUX_PR0_PRU1_GPO13_CFG_REG_FUNC_SEL_SHIFT                        (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO13_CFG_REG_FUNC_SEL_RESETVAL                     (0x00000007U)
#define CSL_IOMUX_PR0_PRU1_GPO13_CFG_REG_FUNC_SEL_MAX                          (0x0000000FU)

#define CSL_IOMUX_PR0_PRU1_GPO13_CFG_REG_IE_OVERRIDE_CTRL_MASK                 (0x00000010U)
#define CSL_IOMUX_PR0_PRU1_GPO13_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                (0x00000004U)
#define CSL_IOMUX_PR0_PRU1_GPO13_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL             (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO13_CFG_REG_IE_OVERRIDE_CTRL_MAX                  (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO13_CFG_REG_IE_OVERRIDE_MASK                      (0x00000020U)
#define CSL_IOMUX_PR0_PRU1_GPO13_CFG_REG_IE_OVERRIDE_SHIFT                     (0x00000005U)
#define CSL_IOMUX_PR0_PRU1_GPO13_CFG_REG_IE_OVERRIDE_RESETVAL                  (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO13_CFG_REG_IE_OVERRIDE_MAX                       (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO13_CFG_REG_OE_OVERRIDE_CTRL_MASK                 (0x00000040U)
#define CSL_IOMUX_PR0_PRU1_GPO13_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                (0x00000006U)
#define CSL_IOMUX_PR0_PRU1_GPO13_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL             (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO13_CFG_REG_OE_OVERRIDE_CTRL_MAX                  (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO13_CFG_REG_OE_OVERRIDE_MASK                      (0x00000080U)
#define CSL_IOMUX_PR0_PRU1_GPO13_CFG_REG_OE_OVERRIDE_SHIFT                     (0x00000007U)
#define CSL_IOMUX_PR0_PRU1_GPO13_CFG_REG_OE_OVERRIDE_RESETVAL                  (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO13_CFG_REG_OE_OVERRIDE_MAX                       (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO13_CFG_REG_PI_MASK                               (0x00000100U)
#define CSL_IOMUX_PR0_PRU1_GPO13_CFG_REG_PI_SHIFT                              (0x00000008U)
#define CSL_IOMUX_PR0_PRU1_GPO13_CFG_REG_PI_RESETVAL                           (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO13_CFG_REG_PI_MAX                                (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO13_CFG_REG_PUPDSEL_MASK                          (0x00000200U)
#define CSL_IOMUX_PR0_PRU1_GPO13_CFG_REG_PUPDSEL_SHIFT                         (0x00000009U)
#define CSL_IOMUX_PR0_PRU1_GPO13_CFG_REG_PUPDSEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO13_CFG_REG_PUPDSEL_MAX                           (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO13_CFG_REG_SC1_MASK                              (0x00000400U)
#define CSL_IOMUX_PR0_PRU1_GPO13_CFG_REG_SC1_SHIFT                             (0x0000000AU)
#define CSL_IOMUX_PR0_PRU1_GPO13_CFG_REG_SC1_RESETVAL                          (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO13_CFG_REG_SC1_MAX                               (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO13_CFG_REG_GPIO_SEL_MASK                         (0x00030000U)
#define CSL_IOMUX_PR0_PRU1_GPO13_CFG_REG_GPIO_SEL_SHIFT                        (0x00000010U)
#define CSL_IOMUX_PR0_PRU1_GPO13_CFG_REG_GPIO_SEL_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO13_CFG_REG_GPIO_SEL_MAX                          (0x00000003U)

#define CSL_IOMUX_PR0_PRU1_GPO13_CFG_REG_QUAL_SEL_MASK                         (0x000C0000U)
#define CSL_IOMUX_PR0_PRU1_GPO13_CFG_REG_QUAL_SEL_SHIFT                        (0x00000012U)
#define CSL_IOMUX_PR0_PRU1_GPO13_CFG_REG_QUAL_SEL_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO13_CFG_REG_QUAL_SEL_MAX                          (0x00000003U)

#define CSL_IOMUX_PR0_PRU1_GPO13_CFG_REG_INP_INV_SEL_MASK                      (0x00100000U)
#define CSL_IOMUX_PR0_PRU1_GPO13_CFG_REG_INP_INV_SEL_SHIFT                     (0x00000014U)
#define CSL_IOMUX_PR0_PRU1_GPO13_CFG_REG_INP_INV_SEL_RESETVAL                  (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO13_CFG_REG_INP_INV_SEL_MAX                       (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO13_CFG_REG_ICSSM_GPIO_SEL_MASK                   (0x00200000U)
#define CSL_IOMUX_PR0_PRU1_GPO13_CFG_REG_ICSSM_GPIO_SEL_SHIFT                  (0x00000015U)
#define CSL_IOMUX_PR0_PRU1_GPO13_CFG_REG_ICSSM_GPIO_SEL_RESETVAL               (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO13_CFG_REG_ICSSM_GPIO_SEL_MAX                    (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO13_CFG_REG_SAFETY_OVERRIDE_SEL_MASK              (0x00400000U)
#define CSL_IOMUX_PR0_PRU1_GPO13_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT             (0x00000016U)
#define CSL_IOMUX_PR0_PRU1_GPO13_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL          (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO13_CFG_REG_SAFETY_OVERRIDE_SEL_MAX               (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO13_CFG_REG_HSMODE_MASK                           (0x40000000U)
#define CSL_IOMUX_PR0_PRU1_GPO13_CFG_REG_HSMODE_SHIFT                          (0x0000001EU)
#define CSL_IOMUX_PR0_PRU1_GPO13_CFG_REG_HSMODE_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO13_CFG_REG_HSMODE_MAX                            (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO13_CFG_REG_HSMASTER_MASK                         (0x80000000U)
#define CSL_IOMUX_PR0_PRU1_GPO13_CFG_REG_HSMASTER_SHIFT                        (0x0000001FU)
#define CSL_IOMUX_PR0_PRU1_GPO13_CFG_REG_HSMASTER_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO13_CFG_REG_HSMASTER_MAX                          (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO13_CFG_REG_RESETVAL                              (0x000005F7U)

/* PR0_PRU1_GPO14_CFG_REG */

#define CSL_IOMUX_PR0_PRU1_GPO14_CFG_REG_FUNC_SEL_MASK                         (0x0000000FU)
#define CSL_IOMUX_PR0_PRU1_GPO14_CFG_REG_FUNC_SEL_SHIFT                        (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO14_CFG_REG_FUNC_SEL_RESETVAL                     (0x00000007U)
#define CSL_IOMUX_PR0_PRU1_GPO14_CFG_REG_FUNC_SEL_MAX                          (0x0000000FU)

#define CSL_IOMUX_PR0_PRU1_GPO14_CFG_REG_IE_OVERRIDE_CTRL_MASK                 (0x00000010U)
#define CSL_IOMUX_PR0_PRU1_GPO14_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                (0x00000004U)
#define CSL_IOMUX_PR0_PRU1_GPO14_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL             (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO14_CFG_REG_IE_OVERRIDE_CTRL_MAX                  (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO14_CFG_REG_IE_OVERRIDE_MASK                      (0x00000020U)
#define CSL_IOMUX_PR0_PRU1_GPO14_CFG_REG_IE_OVERRIDE_SHIFT                     (0x00000005U)
#define CSL_IOMUX_PR0_PRU1_GPO14_CFG_REG_IE_OVERRIDE_RESETVAL                  (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO14_CFG_REG_IE_OVERRIDE_MAX                       (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO14_CFG_REG_OE_OVERRIDE_CTRL_MASK                 (0x00000040U)
#define CSL_IOMUX_PR0_PRU1_GPO14_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                (0x00000006U)
#define CSL_IOMUX_PR0_PRU1_GPO14_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL             (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO14_CFG_REG_OE_OVERRIDE_CTRL_MAX                  (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO14_CFG_REG_OE_OVERRIDE_MASK                      (0x00000080U)
#define CSL_IOMUX_PR0_PRU1_GPO14_CFG_REG_OE_OVERRIDE_SHIFT                     (0x00000007U)
#define CSL_IOMUX_PR0_PRU1_GPO14_CFG_REG_OE_OVERRIDE_RESETVAL                  (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO14_CFG_REG_OE_OVERRIDE_MAX                       (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO14_CFG_REG_PI_MASK                               (0x00000100U)
#define CSL_IOMUX_PR0_PRU1_GPO14_CFG_REG_PI_SHIFT                              (0x00000008U)
#define CSL_IOMUX_PR0_PRU1_GPO14_CFG_REG_PI_RESETVAL                           (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO14_CFG_REG_PI_MAX                                (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO14_CFG_REG_PUPDSEL_MASK                          (0x00000200U)
#define CSL_IOMUX_PR0_PRU1_GPO14_CFG_REG_PUPDSEL_SHIFT                         (0x00000009U)
#define CSL_IOMUX_PR0_PRU1_GPO14_CFG_REG_PUPDSEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO14_CFG_REG_PUPDSEL_MAX                           (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO14_CFG_REG_SC1_MASK                              (0x00000400U)
#define CSL_IOMUX_PR0_PRU1_GPO14_CFG_REG_SC1_SHIFT                             (0x0000000AU)
#define CSL_IOMUX_PR0_PRU1_GPO14_CFG_REG_SC1_RESETVAL                          (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO14_CFG_REG_SC1_MAX                               (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO14_CFG_REG_GPIO_SEL_MASK                         (0x00030000U)
#define CSL_IOMUX_PR0_PRU1_GPO14_CFG_REG_GPIO_SEL_SHIFT                        (0x00000010U)
#define CSL_IOMUX_PR0_PRU1_GPO14_CFG_REG_GPIO_SEL_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO14_CFG_REG_GPIO_SEL_MAX                          (0x00000003U)

#define CSL_IOMUX_PR0_PRU1_GPO14_CFG_REG_QUAL_SEL_MASK                         (0x000C0000U)
#define CSL_IOMUX_PR0_PRU1_GPO14_CFG_REG_QUAL_SEL_SHIFT                        (0x00000012U)
#define CSL_IOMUX_PR0_PRU1_GPO14_CFG_REG_QUAL_SEL_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO14_CFG_REG_QUAL_SEL_MAX                          (0x00000003U)

#define CSL_IOMUX_PR0_PRU1_GPO14_CFG_REG_INP_INV_SEL_MASK                      (0x00100000U)
#define CSL_IOMUX_PR0_PRU1_GPO14_CFG_REG_INP_INV_SEL_SHIFT                     (0x00000014U)
#define CSL_IOMUX_PR0_PRU1_GPO14_CFG_REG_INP_INV_SEL_RESETVAL                  (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO14_CFG_REG_INP_INV_SEL_MAX                       (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO14_CFG_REG_ICSSM_GPIO_SEL_MASK                   (0x00200000U)
#define CSL_IOMUX_PR0_PRU1_GPO14_CFG_REG_ICSSM_GPIO_SEL_SHIFT                  (0x00000015U)
#define CSL_IOMUX_PR0_PRU1_GPO14_CFG_REG_ICSSM_GPIO_SEL_RESETVAL               (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO14_CFG_REG_ICSSM_GPIO_SEL_MAX                    (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO14_CFG_REG_SAFETY_OVERRIDE_SEL_MASK              (0x00400000U)
#define CSL_IOMUX_PR0_PRU1_GPO14_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT             (0x00000016U)
#define CSL_IOMUX_PR0_PRU1_GPO14_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL          (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO14_CFG_REG_SAFETY_OVERRIDE_SEL_MAX               (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO14_CFG_REG_HSMODE_MASK                           (0x40000000U)
#define CSL_IOMUX_PR0_PRU1_GPO14_CFG_REG_HSMODE_SHIFT                          (0x0000001EU)
#define CSL_IOMUX_PR0_PRU1_GPO14_CFG_REG_HSMODE_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO14_CFG_REG_HSMODE_MAX                            (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO14_CFG_REG_HSMASTER_MASK                         (0x80000000U)
#define CSL_IOMUX_PR0_PRU1_GPO14_CFG_REG_HSMASTER_SHIFT                        (0x0000001FU)
#define CSL_IOMUX_PR0_PRU1_GPO14_CFG_REG_HSMASTER_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO14_CFG_REG_HSMASTER_MAX                          (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO14_CFG_REG_RESETVAL                              (0x000005F7U)

/* PR0_PRU1_GPO19_CFG_REG */

#define CSL_IOMUX_PR0_PRU1_GPO19_CFG_REG_FUNC_SEL_MASK                         (0x0000000FU)
#define CSL_IOMUX_PR0_PRU1_GPO19_CFG_REG_FUNC_SEL_SHIFT                        (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO19_CFG_REG_FUNC_SEL_RESETVAL                     (0x00000007U)
#define CSL_IOMUX_PR0_PRU1_GPO19_CFG_REG_FUNC_SEL_MAX                          (0x0000000FU)

#define CSL_IOMUX_PR0_PRU1_GPO19_CFG_REG_IE_OVERRIDE_CTRL_MASK                 (0x00000010U)
#define CSL_IOMUX_PR0_PRU1_GPO19_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                (0x00000004U)
#define CSL_IOMUX_PR0_PRU1_GPO19_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL             (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO19_CFG_REG_IE_OVERRIDE_CTRL_MAX                  (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO19_CFG_REG_IE_OVERRIDE_MASK                      (0x00000020U)
#define CSL_IOMUX_PR0_PRU1_GPO19_CFG_REG_IE_OVERRIDE_SHIFT                     (0x00000005U)
#define CSL_IOMUX_PR0_PRU1_GPO19_CFG_REG_IE_OVERRIDE_RESETVAL                  (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO19_CFG_REG_IE_OVERRIDE_MAX                       (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO19_CFG_REG_OE_OVERRIDE_CTRL_MASK                 (0x00000040U)
#define CSL_IOMUX_PR0_PRU1_GPO19_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                (0x00000006U)
#define CSL_IOMUX_PR0_PRU1_GPO19_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL             (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO19_CFG_REG_OE_OVERRIDE_CTRL_MAX                  (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO19_CFG_REG_OE_OVERRIDE_MASK                      (0x00000080U)
#define CSL_IOMUX_PR0_PRU1_GPO19_CFG_REG_OE_OVERRIDE_SHIFT                     (0x00000007U)
#define CSL_IOMUX_PR0_PRU1_GPO19_CFG_REG_OE_OVERRIDE_RESETVAL                  (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO19_CFG_REG_OE_OVERRIDE_MAX                       (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO19_CFG_REG_PI_MASK                               (0x00000100U)
#define CSL_IOMUX_PR0_PRU1_GPO19_CFG_REG_PI_SHIFT                              (0x00000008U)
#define CSL_IOMUX_PR0_PRU1_GPO19_CFG_REG_PI_RESETVAL                           (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO19_CFG_REG_PI_MAX                                (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO19_CFG_REG_PUPDSEL_MASK                          (0x00000200U)
#define CSL_IOMUX_PR0_PRU1_GPO19_CFG_REG_PUPDSEL_SHIFT                         (0x00000009U)
#define CSL_IOMUX_PR0_PRU1_GPO19_CFG_REG_PUPDSEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO19_CFG_REG_PUPDSEL_MAX                           (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO19_CFG_REG_SC1_MASK                              (0x00000400U)
#define CSL_IOMUX_PR0_PRU1_GPO19_CFG_REG_SC1_SHIFT                             (0x0000000AU)
#define CSL_IOMUX_PR0_PRU1_GPO19_CFG_REG_SC1_RESETVAL                          (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO19_CFG_REG_SC1_MAX                               (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO19_CFG_REG_GPIO_SEL_MASK                         (0x00030000U)
#define CSL_IOMUX_PR0_PRU1_GPO19_CFG_REG_GPIO_SEL_SHIFT                        (0x00000010U)
#define CSL_IOMUX_PR0_PRU1_GPO19_CFG_REG_GPIO_SEL_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO19_CFG_REG_GPIO_SEL_MAX                          (0x00000003U)

#define CSL_IOMUX_PR0_PRU1_GPO19_CFG_REG_QUAL_SEL_MASK                         (0x000C0000U)
#define CSL_IOMUX_PR0_PRU1_GPO19_CFG_REG_QUAL_SEL_SHIFT                        (0x00000012U)
#define CSL_IOMUX_PR0_PRU1_GPO19_CFG_REG_QUAL_SEL_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO19_CFG_REG_QUAL_SEL_MAX                          (0x00000003U)

#define CSL_IOMUX_PR0_PRU1_GPO19_CFG_REG_INP_INV_SEL_MASK                      (0x00100000U)
#define CSL_IOMUX_PR0_PRU1_GPO19_CFG_REG_INP_INV_SEL_SHIFT                     (0x00000014U)
#define CSL_IOMUX_PR0_PRU1_GPO19_CFG_REG_INP_INV_SEL_RESETVAL                  (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO19_CFG_REG_INP_INV_SEL_MAX                       (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO19_CFG_REG_ICSSM_GPIO_SEL_MASK                   (0x00200000U)
#define CSL_IOMUX_PR0_PRU1_GPO19_CFG_REG_ICSSM_GPIO_SEL_SHIFT                  (0x00000015U)
#define CSL_IOMUX_PR0_PRU1_GPO19_CFG_REG_ICSSM_GPIO_SEL_RESETVAL               (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO19_CFG_REG_ICSSM_GPIO_SEL_MAX                    (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO19_CFG_REG_SAFETY_OVERRIDE_SEL_MASK              (0x00400000U)
#define CSL_IOMUX_PR0_PRU1_GPO19_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT             (0x00000016U)
#define CSL_IOMUX_PR0_PRU1_GPO19_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL          (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO19_CFG_REG_SAFETY_OVERRIDE_SEL_MAX               (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO19_CFG_REG_HSMODE_MASK                           (0x40000000U)
#define CSL_IOMUX_PR0_PRU1_GPO19_CFG_REG_HSMODE_SHIFT                          (0x0000001EU)
#define CSL_IOMUX_PR0_PRU1_GPO19_CFG_REG_HSMODE_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO19_CFG_REG_HSMODE_MAX                            (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO19_CFG_REG_HSMASTER_MASK                         (0x80000000U)
#define CSL_IOMUX_PR0_PRU1_GPO19_CFG_REG_HSMASTER_SHIFT                        (0x0000001FU)
#define CSL_IOMUX_PR0_PRU1_GPO19_CFG_REG_HSMASTER_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO19_CFG_REG_HSMASTER_MAX                          (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO19_CFG_REG_RESETVAL                              (0x000005F7U)

/* PR0_PRU1_GPO18_CFG_REG */

#define CSL_IOMUX_PR0_PRU1_GPO18_CFG_REG_FUNC_SEL_MASK                         (0x0000000FU)
#define CSL_IOMUX_PR0_PRU1_GPO18_CFG_REG_FUNC_SEL_SHIFT                        (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO18_CFG_REG_FUNC_SEL_RESETVAL                     (0x00000007U)
#define CSL_IOMUX_PR0_PRU1_GPO18_CFG_REG_FUNC_SEL_MAX                          (0x0000000FU)

#define CSL_IOMUX_PR0_PRU1_GPO18_CFG_REG_IE_OVERRIDE_CTRL_MASK                 (0x00000010U)
#define CSL_IOMUX_PR0_PRU1_GPO18_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                (0x00000004U)
#define CSL_IOMUX_PR0_PRU1_GPO18_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL             (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO18_CFG_REG_IE_OVERRIDE_CTRL_MAX                  (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO18_CFG_REG_IE_OVERRIDE_MASK                      (0x00000020U)
#define CSL_IOMUX_PR0_PRU1_GPO18_CFG_REG_IE_OVERRIDE_SHIFT                     (0x00000005U)
#define CSL_IOMUX_PR0_PRU1_GPO18_CFG_REG_IE_OVERRIDE_RESETVAL                  (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO18_CFG_REG_IE_OVERRIDE_MAX                       (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO18_CFG_REG_OE_OVERRIDE_CTRL_MASK                 (0x00000040U)
#define CSL_IOMUX_PR0_PRU1_GPO18_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                (0x00000006U)
#define CSL_IOMUX_PR0_PRU1_GPO18_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL             (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO18_CFG_REG_OE_OVERRIDE_CTRL_MAX                  (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO18_CFG_REG_OE_OVERRIDE_MASK                      (0x00000080U)
#define CSL_IOMUX_PR0_PRU1_GPO18_CFG_REG_OE_OVERRIDE_SHIFT                     (0x00000007U)
#define CSL_IOMUX_PR0_PRU1_GPO18_CFG_REG_OE_OVERRIDE_RESETVAL                  (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO18_CFG_REG_OE_OVERRIDE_MAX                       (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO18_CFG_REG_PI_MASK                               (0x00000100U)
#define CSL_IOMUX_PR0_PRU1_GPO18_CFG_REG_PI_SHIFT                              (0x00000008U)
#define CSL_IOMUX_PR0_PRU1_GPO18_CFG_REG_PI_RESETVAL                           (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO18_CFG_REG_PI_MAX                                (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO18_CFG_REG_PUPDSEL_MASK                          (0x00000200U)
#define CSL_IOMUX_PR0_PRU1_GPO18_CFG_REG_PUPDSEL_SHIFT                         (0x00000009U)
#define CSL_IOMUX_PR0_PRU1_GPO18_CFG_REG_PUPDSEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO18_CFG_REG_PUPDSEL_MAX                           (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO18_CFG_REG_SC1_MASK                              (0x00000400U)
#define CSL_IOMUX_PR0_PRU1_GPO18_CFG_REG_SC1_SHIFT                             (0x0000000AU)
#define CSL_IOMUX_PR0_PRU1_GPO18_CFG_REG_SC1_RESETVAL                          (0x00000001U)
#define CSL_IOMUX_PR0_PRU1_GPO18_CFG_REG_SC1_MAX                               (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO18_CFG_REG_GPIO_SEL_MASK                         (0x00030000U)
#define CSL_IOMUX_PR0_PRU1_GPO18_CFG_REG_GPIO_SEL_SHIFT                        (0x00000010U)
#define CSL_IOMUX_PR0_PRU1_GPO18_CFG_REG_GPIO_SEL_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO18_CFG_REG_GPIO_SEL_MAX                          (0x00000003U)

#define CSL_IOMUX_PR0_PRU1_GPO18_CFG_REG_QUAL_SEL_MASK                         (0x000C0000U)
#define CSL_IOMUX_PR0_PRU1_GPO18_CFG_REG_QUAL_SEL_SHIFT                        (0x00000012U)
#define CSL_IOMUX_PR0_PRU1_GPO18_CFG_REG_QUAL_SEL_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO18_CFG_REG_QUAL_SEL_MAX                          (0x00000003U)

#define CSL_IOMUX_PR0_PRU1_GPO18_CFG_REG_INP_INV_SEL_MASK                      (0x00100000U)
#define CSL_IOMUX_PR0_PRU1_GPO18_CFG_REG_INP_INV_SEL_SHIFT                     (0x00000014U)
#define CSL_IOMUX_PR0_PRU1_GPO18_CFG_REG_INP_INV_SEL_RESETVAL                  (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO18_CFG_REG_INP_INV_SEL_MAX                       (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO18_CFG_REG_ICSSM_GPIO_SEL_MASK                   (0x00200000U)
#define CSL_IOMUX_PR0_PRU1_GPO18_CFG_REG_ICSSM_GPIO_SEL_SHIFT                  (0x00000015U)
#define CSL_IOMUX_PR0_PRU1_GPO18_CFG_REG_ICSSM_GPIO_SEL_RESETVAL               (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO18_CFG_REG_ICSSM_GPIO_SEL_MAX                    (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO18_CFG_REG_SAFETY_OVERRIDE_SEL_MASK              (0x00400000U)
#define CSL_IOMUX_PR0_PRU1_GPO18_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT             (0x00000016U)
#define CSL_IOMUX_PR0_PRU1_GPO18_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL          (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO18_CFG_REG_SAFETY_OVERRIDE_SEL_MAX               (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO18_CFG_REG_HSMODE_MASK                           (0x40000000U)
#define CSL_IOMUX_PR0_PRU1_GPO18_CFG_REG_HSMODE_SHIFT                          (0x0000001EU)
#define CSL_IOMUX_PR0_PRU1_GPO18_CFG_REG_HSMODE_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO18_CFG_REG_HSMODE_MAX                            (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO18_CFG_REG_HSMASTER_MASK                         (0x80000000U)
#define CSL_IOMUX_PR0_PRU1_GPO18_CFG_REG_HSMASTER_SHIFT                        (0x0000001FU)
#define CSL_IOMUX_PR0_PRU1_GPO18_CFG_REG_HSMASTER_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_PR0_PRU1_GPO18_CFG_REG_HSMASTER_MAX                          (0x00000001U)

#define CSL_IOMUX_PR0_PRU1_GPO18_CFG_REG_RESETVAL                              (0x000005F7U)

/* EXT_REFCLK0_CFG_REG */

#define CSL_IOMUX_EXT_REFCLK0_CFG_REG_FUNC_SEL_MASK                            (0x0000000FU)
#define CSL_IOMUX_EXT_REFCLK0_CFG_REG_FUNC_SEL_SHIFT                           (0x00000000U)
#define CSL_IOMUX_EXT_REFCLK0_CFG_REG_FUNC_SEL_RESETVAL                        (0x00000007U)
#define CSL_IOMUX_EXT_REFCLK0_CFG_REG_FUNC_SEL_MAX                             (0x0000000FU)

#define CSL_IOMUX_EXT_REFCLK0_CFG_REG_IE_OVERRIDE_CTRL_MASK                    (0x00000010U)
#define CSL_IOMUX_EXT_REFCLK0_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                   (0x00000004U)
#define CSL_IOMUX_EXT_REFCLK0_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                (0x00000001U)
#define CSL_IOMUX_EXT_REFCLK0_CFG_REG_IE_OVERRIDE_CTRL_MAX                     (0x00000001U)

#define CSL_IOMUX_EXT_REFCLK0_CFG_REG_IE_OVERRIDE_MASK                         (0x00000020U)
#define CSL_IOMUX_EXT_REFCLK0_CFG_REG_IE_OVERRIDE_SHIFT                        (0x00000005U)
#define CSL_IOMUX_EXT_REFCLK0_CFG_REG_IE_OVERRIDE_RESETVAL                     (0x00000001U)
#define CSL_IOMUX_EXT_REFCLK0_CFG_REG_IE_OVERRIDE_MAX                          (0x00000001U)

#define CSL_IOMUX_EXT_REFCLK0_CFG_REG_OE_OVERRIDE_CTRL_MASK                    (0x00000040U)
#define CSL_IOMUX_EXT_REFCLK0_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                   (0x00000006U)
#define CSL_IOMUX_EXT_REFCLK0_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                (0x00000001U)
#define CSL_IOMUX_EXT_REFCLK0_CFG_REG_OE_OVERRIDE_CTRL_MAX                     (0x00000001U)

#define CSL_IOMUX_EXT_REFCLK0_CFG_REG_OE_OVERRIDE_MASK                         (0x00000080U)
#define CSL_IOMUX_EXT_REFCLK0_CFG_REG_OE_OVERRIDE_SHIFT                        (0x00000007U)
#define CSL_IOMUX_EXT_REFCLK0_CFG_REG_OE_OVERRIDE_RESETVAL                     (0x00000001U)
#define CSL_IOMUX_EXT_REFCLK0_CFG_REG_OE_OVERRIDE_MAX                          (0x00000001U)

#define CSL_IOMUX_EXT_REFCLK0_CFG_REG_PI_MASK                                  (0x00000100U)
#define CSL_IOMUX_EXT_REFCLK0_CFG_REG_PI_SHIFT                                 (0x00000008U)
#define CSL_IOMUX_EXT_REFCLK0_CFG_REG_PI_RESETVAL                              (0x00000001U)
#define CSL_IOMUX_EXT_REFCLK0_CFG_REG_PI_MAX                                   (0x00000001U)

#define CSL_IOMUX_EXT_REFCLK0_CFG_REG_PUPDSEL_MASK                             (0x00000200U)
#define CSL_IOMUX_EXT_REFCLK0_CFG_REG_PUPDSEL_SHIFT                            (0x00000009U)
#define CSL_IOMUX_EXT_REFCLK0_CFG_REG_PUPDSEL_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_EXT_REFCLK0_CFG_REG_PUPDSEL_MAX                              (0x00000001U)

#define CSL_IOMUX_EXT_REFCLK0_CFG_REG_SC1_MASK                                 (0x00000400U)
#define CSL_IOMUX_EXT_REFCLK0_CFG_REG_SC1_SHIFT                                (0x0000000AU)
#define CSL_IOMUX_EXT_REFCLK0_CFG_REG_SC1_RESETVAL                             (0x00000001U)
#define CSL_IOMUX_EXT_REFCLK0_CFG_REG_SC1_MAX                                  (0x00000001U)

#define CSL_IOMUX_EXT_REFCLK0_CFG_REG_GPIO_SEL_MASK                            (0x00030000U)
#define CSL_IOMUX_EXT_REFCLK0_CFG_REG_GPIO_SEL_SHIFT                           (0x00000010U)
#define CSL_IOMUX_EXT_REFCLK0_CFG_REG_GPIO_SEL_RESETVAL                        (0x00000000U)
#define CSL_IOMUX_EXT_REFCLK0_CFG_REG_GPIO_SEL_MAX                             (0x00000003U)

#define CSL_IOMUX_EXT_REFCLK0_CFG_REG_QUAL_SEL_MASK                            (0x000C0000U)
#define CSL_IOMUX_EXT_REFCLK0_CFG_REG_QUAL_SEL_SHIFT                           (0x00000012U)
#define CSL_IOMUX_EXT_REFCLK0_CFG_REG_QUAL_SEL_RESETVAL                        (0x00000000U)
#define CSL_IOMUX_EXT_REFCLK0_CFG_REG_QUAL_SEL_MAX                             (0x00000003U)

#define CSL_IOMUX_EXT_REFCLK0_CFG_REG_INP_INV_SEL_MASK                         (0x00100000U)
#define CSL_IOMUX_EXT_REFCLK0_CFG_REG_INP_INV_SEL_SHIFT                        (0x00000014U)
#define CSL_IOMUX_EXT_REFCLK0_CFG_REG_INP_INV_SEL_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_EXT_REFCLK0_CFG_REG_INP_INV_SEL_MAX                          (0x00000001U)

#define CSL_IOMUX_EXT_REFCLK0_CFG_REG_ICSSM_GPIO_SEL_MASK                      (0x00200000U)
#define CSL_IOMUX_EXT_REFCLK0_CFG_REG_ICSSM_GPIO_SEL_SHIFT                     (0x00000015U)
#define CSL_IOMUX_EXT_REFCLK0_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                  (0x00000000U)
#define CSL_IOMUX_EXT_REFCLK0_CFG_REG_ICSSM_GPIO_SEL_MAX                       (0x00000001U)

#define CSL_IOMUX_EXT_REFCLK0_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                 (0x00400000U)
#define CSL_IOMUX_EXT_REFCLK0_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                (0x00000016U)
#define CSL_IOMUX_EXT_REFCLK0_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL             (0x00000000U)
#define CSL_IOMUX_EXT_REFCLK0_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                  (0x00000001U)

#define CSL_IOMUX_EXT_REFCLK0_CFG_REG_HSMODE_MASK                              (0x40000000U)
#define CSL_IOMUX_EXT_REFCLK0_CFG_REG_HSMODE_SHIFT                             (0x0000001EU)
#define CSL_IOMUX_EXT_REFCLK0_CFG_REG_HSMODE_RESETVAL                          (0x00000000U)
#define CSL_IOMUX_EXT_REFCLK0_CFG_REG_HSMODE_MAX                               (0x00000001U)

#define CSL_IOMUX_EXT_REFCLK0_CFG_REG_HSMASTER_MASK                            (0x80000000U)
#define CSL_IOMUX_EXT_REFCLK0_CFG_REG_HSMASTER_SHIFT                           (0x0000001FU)
#define CSL_IOMUX_EXT_REFCLK0_CFG_REG_HSMASTER_RESETVAL                        (0x00000000U)
#define CSL_IOMUX_EXT_REFCLK0_CFG_REG_HSMASTER_MAX                             (0x00000001U)

#define CSL_IOMUX_EXT_REFCLK0_CFG_REG_RESETVAL                                 (0x000005F7U)

/* SDFM0_CLK0_CFG_REG */

#define CSL_IOMUX_SDFM0_CLK0_CFG_REG_FUNC_SEL_MASK                             (0x0000000FU)
#define CSL_IOMUX_SDFM0_CLK0_CFG_REG_FUNC_SEL_SHIFT                            (0x00000000U)
#define CSL_IOMUX_SDFM0_CLK0_CFG_REG_FUNC_SEL_RESETVAL                         (0x00000007U)
#define CSL_IOMUX_SDFM0_CLK0_CFG_REG_FUNC_SEL_MAX                              (0x0000000FU)

#define CSL_IOMUX_SDFM0_CLK0_CFG_REG_IE_OVERRIDE_CTRL_MASK                     (0x00000010U)
#define CSL_IOMUX_SDFM0_CLK0_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                    (0x00000004U)
#define CSL_IOMUX_SDFM0_CLK0_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_IOMUX_SDFM0_CLK0_CFG_REG_IE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_IOMUX_SDFM0_CLK0_CFG_REG_IE_OVERRIDE_MASK                          (0x00000020U)
#define CSL_IOMUX_SDFM0_CLK0_CFG_REG_IE_OVERRIDE_SHIFT                         (0x00000005U)
#define CSL_IOMUX_SDFM0_CLK0_CFG_REG_IE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_IOMUX_SDFM0_CLK0_CFG_REG_IE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_IOMUX_SDFM0_CLK0_CFG_REG_OE_OVERRIDE_CTRL_MASK                     (0x00000040U)
#define CSL_IOMUX_SDFM0_CLK0_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                    (0x00000006U)
#define CSL_IOMUX_SDFM0_CLK0_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_IOMUX_SDFM0_CLK0_CFG_REG_OE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_IOMUX_SDFM0_CLK0_CFG_REG_OE_OVERRIDE_MASK                          (0x00000080U)
#define CSL_IOMUX_SDFM0_CLK0_CFG_REG_OE_OVERRIDE_SHIFT                         (0x00000007U)
#define CSL_IOMUX_SDFM0_CLK0_CFG_REG_OE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_IOMUX_SDFM0_CLK0_CFG_REG_OE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_IOMUX_SDFM0_CLK0_CFG_REG_PI_MASK                                   (0x00000100U)
#define CSL_IOMUX_SDFM0_CLK0_CFG_REG_PI_SHIFT                                  (0x00000008U)
#define CSL_IOMUX_SDFM0_CLK0_CFG_REG_PI_RESETVAL                               (0x00000001U)
#define CSL_IOMUX_SDFM0_CLK0_CFG_REG_PI_MAX                                    (0x00000001U)

#define CSL_IOMUX_SDFM0_CLK0_CFG_REG_PUPDSEL_MASK                              (0x00000200U)
#define CSL_IOMUX_SDFM0_CLK0_CFG_REG_PUPDSEL_SHIFT                             (0x00000009U)
#define CSL_IOMUX_SDFM0_CLK0_CFG_REG_PUPDSEL_RESETVAL                          (0x00000000U)
#define CSL_IOMUX_SDFM0_CLK0_CFG_REG_PUPDSEL_MAX                               (0x00000001U)

#define CSL_IOMUX_SDFM0_CLK0_CFG_REG_SC1_MASK                                  (0x00000400U)
#define CSL_IOMUX_SDFM0_CLK0_CFG_REG_SC1_SHIFT                                 (0x0000000AU)
#define CSL_IOMUX_SDFM0_CLK0_CFG_REG_SC1_RESETVAL                              (0x00000001U)
#define CSL_IOMUX_SDFM0_CLK0_CFG_REG_SC1_MAX                                   (0x00000001U)

#define CSL_IOMUX_SDFM0_CLK0_CFG_REG_GPIO_SEL_MASK                             (0x00030000U)
#define CSL_IOMUX_SDFM0_CLK0_CFG_REG_GPIO_SEL_SHIFT                            (0x00000010U)
#define CSL_IOMUX_SDFM0_CLK0_CFG_REG_GPIO_SEL_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_SDFM0_CLK0_CFG_REG_GPIO_SEL_MAX                              (0x00000003U)

#define CSL_IOMUX_SDFM0_CLK0_CFG_REG_QUAL_SEL_MASK                             (0x000C0000U)
#define CSL_IOMUX_SDFM0_CLK0_CFG_REG_QUAL_SEL_SHIFT                            (0x00000012U)
#define CSL_IOMUX_SDFM0_CLK0_CFG_REG_QUAL_SEL_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_SDFM0_CLK0_CFG_REG_QUAL_SEL_MAX                              (0x00000003U)

#define CSL_IOMUX_SDFM0_CLK0_CFG_REG_INP_INV_SEL_MASK                          (0x00100000U)
#define CSL_IOMUX_SDFM0_CLK0_CFG_REG_INP_INV_SEL_SHIFT                         (0x00000014U)
#define CSL_IOMUX_SDFM0_CLK0_CFG_REG_INP_INV_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_SDFM0_CLK0_CFG_REG_INP_INV_SEL_MAX                           (0x00000001U)

#define CSL_IOMUX_SDFM0_CLK0_CFG_REG_ICSSM_GPIO_SEL_MASK                       (0x00200000U)
#define CSL_IOMUX_SDFM0_CLK0_CFG_REG_ICSSM_GPIO_SEL_SHIFT                      (0x00000015U)
#define CSL_IOMUX_SDFM0_CLK0_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                   (0x00000000U)
#define CSL_IOMUX_SDFM0_CLK0_CFG_REG_ICSSM_GPIO_SEL_MAX                        (0x00000001U)

#define CSL_IOMUX_SDFM0_CLK0_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                  (0x00400000U)
#define CSL_IOMUX_SDFM0_CLK0_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                 (0x00000016U)
#define CSL_IOMUX_SDFM0_CLK0_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL              (0x00000000U)
#define CSL_IOMUX_SDFM0_CLK0_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                   (0x00000001U)

#define CSL_IOMUX_SDFM0_CLK0_CFG_REG_HSMODE_MASK                               (0x40000000U)
#define CSL_IOMUX_SDFM0_CLK0_CFG_REG_HSMODE_SHIFT                              (0x0000001EU)
#define CSL_IOMUX_SDFM0_CLK0_CFG_REG_HSMODE_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_SDFM0_CLK0_CFG_REG_HSMODE_MAX                                (0x00000001U)

#define CSL_IOMUX_SDFM0_CLK0_CFG_REG_HSMASTER_MASK                             (0x80000000U)
#define CSL_IOMUX_SDFM0_CLK0_CFG_REG_HSMASTER_SHIFT                            (0x0000001FU)
#define CSL_IOMUX_SDFM0_CLK0_CFG_REG_HSMASTER_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_SDFM0_CLK0_CFG_REG_HSMASTER_MAX                              (0x00000001U)

#define CSL_IOMUX_SDFM0_CLK0_CFG_REG_RESETVAL                                  (0x000005F7U)

/* SDFM0_D0_CFG_REG */

#define CSL_IOMUX_SDFM0_D0_CFG_REG_FUNC_SEL_MASK                               (0x0000000FU)
#define CSL_IOMUX_SDFM0_D0_CFG_REG_FUNC_SEL_SHIFT                              (0x00000000U)
#define CSL_IOMUX_SDFM0_D0_CFG_REG_FUNC_SEL_RESETVAL                           (0x00000007U)
#define CSL_IOMUX_SDFM0_D0_CFG_REG_FUNC_SEL_MAX                                (0x0000000FU)

#define CSL_IOMUX_SDFM0_D0_CFG_REG_IE_OVERRIDE_CTRL_MASK                       (0x00000010U)
#define CSL_IOMUX_SDFM0_D0_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                      (0x00000004U)
#define CSL_IOMUX_SDFM0_D0_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_SDFM0_D0_CFG_REG_IE_OVERRIDE_CTRL_MAX                        (0x00000001U)

#define CSL_IOMUX_SDFM0_D0_CFG_REG_IE_OVERRIDE_MASK                            (0x00000020U)
#define CSL_IOMUX_SDFM0_D0_CFG_REG_IE_OVERRIDE_SHIFT                           (0x00000005U)
#define CSL_IOMUX_SDFM0_D0_CFG_REG_IE_OVERRIDE_RESETVAL                        (0x00000001U)
#define CSL_IOMUX_SDFM0_D0_CFG_REG_IE_OVERRIDE_MAX                             (0x00000001U)

#define CSL_IOMUX_SDFM0_D0_CFG_REG_OE_OVERRIDE_CTRL_MASK                       (0x00000040U)
#define CSL_IOMUX_SDFM0_D0_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                      (0x00000006U)
#define CSL_IOMUX_SDFM0_D0_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_SDFM0_D0_CFG_REG_OE_OVERRIDE_CTRL_MAX                        (0x00000001U)

#define CSL_IOMUX_SDFM0_D0_CFG_REG_OE_OVERRIDE_MASK                            (0x00000080U)
#define CSL_IOMUX_SDFM0_D0_CFG_REG_OE_OVERRIDE_SHIFT                           (0x00000007U)
#define CSL_IOMUX_SDFM0_D0_CFG_REG_OE_OVERRIDE_RESETVAL                        (0x00000001U)
#define CSL_IOMUX_SDFM0_D0_CFG_REG_OE_OVERRIDE_MAX                             (0x00000001U)

#define CSL_IOMUX_SDFM0_D0_CFG_REG_PI_MASK                                     (0x00000100U)
#define CSL_IOMUX_SDFM0_D0_CFG_REG_PI_SHIFT                                    (0x00000008U)
#define CSL_IOMUX_SDFM0_D0_CFG_REG_PI_RESETVAL                                 (0x00000001U)
#define CSL_IOMUX_SDFM0_D0_CFG_REG_PI_MAX                                      (0x00000001U)

#define CSL_IOMUX_SDFM0_D0_CFG_REG_PUPDSEL_MASK                                (0x00000200U)
#define CSL_IOMUX_SDFM0_D0_CFG_REG_PUPDSEL_SHIFT                               (0x00000009U)
#define CSL_IOMUX_SDFM0_D0_CFG_REG_PUPDSEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_SDFM0_D0_CFG_REG_PUPDSEL_MAX                                 (0x00000001U)

#define CSL_IOMUX_SDFM0_D0_CFG_REG_SC1_MASK                                    (0x00000400U)
#define CSL_IOMUX_SDFM0_D0_CFG_REG_SC1_SHIFT                                   (0x0000000AU)
#define CSL_IOMUX_SDFM0_D0_CFG_REG_SC1_RESETVAL                                (0x00000001U)
#define CSL_IOMUX_SDFM0_D0_CFG_REG_SC1_MAX                                     (0x00000001U)

#define CSL_IOMUX_SDFM0_D0_CFG_REG_GPIO_SEL_MASK                               (0x00030000U)
#define CSL_IOMUX_SDFM0_D0_CFG_REG_GPIO_SEL_SHIFT                              (0x00000010U)
#define CSL_IOMUX_SDFM0_D0_CFG_REG_GPIO_SEL_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_SDFM0_D0_CFG_REG_GPIO_SEL_MAX                                (0x00000003U)

#define CSL_IOMUX_SDFM0_D0_CFG_REG_QUAL_SEL_MASK                               (0x000C0000U)
#define CSL_IOMUX_SDFM0_D0_CFG_REG_QUAL_SEL_SHIFT                              (0x00000012U)
#define CSL_IOMUX_SDFM0_D0_CFG_REG_QUAL_SEL_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_SDFM0_D0_CFG_REG_QUAL_SEL_MAX                                (0x00000003U)

#define CSL_IOMUX_SDFM0_D0_CFG_REG_INP_INV_SEL_MASK                            (0x00100000U)
#define CSL_IOMUX_SDFM0_D0_CFG_REG_INP_INV_SEL_SHIFT                           (0x00000014U)
#define CSL_IOMUX_SDFM0_D0_CFG_REG_INP_INV_SEL_RESETVAL                        (0x00000000U)
#define CSL_IOMUX_SDFM0_D0_CFG_REG_INP_INV_SEL_MAX                             (0x00000001U)

#define CSL_IOMUX_SDFM0_D0_CFG_REG_ICSSM_GPIO_SEL_MASK                         (0x00200000U)
#define CSL_IOMUX_SDFM0_D0_CFG_REG_ICSSM_GPIO_SEL_SHIFT                        (0x00000015U)
#define CSL_IOMUX_SDFM0_D0_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_SDFM0_D0_CFG_REG_ICSSM_GPIO_SEL_MAX                          (0x00000001U)

#define CSL_IOMUX_SDFM0_D0_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                    (0x00400000U)
#define CSL_IOMUX_SDFM0_D0_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                   (0x00000016U)
#define CSL_IOMUX_SDFM0_D0_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL                (0x00000000U)
#define CSL_IOMUX_SDFM0_D0_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                     (0x00000001U)

#define CSL_IOMUX_SDFM0_D0_CFG_REG_HSMODE_MASK                                 (0x40000000U)
#define CSL_IOMUX_SDFM0_D0_CFG_REG_HSMODE_SHIFT                                (0x0000001EU)
#define CSL_IOMUX_SDFM0_D0_CFG_REG_HSMODE_RESETVAL                             (0x00000000U)
#define CSL_IOMUX_SDFM0_D0_CFG_REG_HSMODE_MAX                                  (0x00000001U)

#define CSL_IOMUX_SDFM0_D0_CFG_REG_HSMASTER_MASK                               (0x80000000U)
#define CSL_IOMUX_SDFM0_D0_CFG_REG_HSMASTER_SHIFT                              (0x0000001FU)
#define CSL_IOMUX_SDFM0_D0_CFG_REG_HSMASTER_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_SDFM0_D0_CFG_REG_HSMASTER_MAX                                (0x00000001U)

#define CSL_IOMUX_SDFM0_D0_CFG_REG_RESETVAL                                    (0x000005F7U)

/* SDFM0_CLK1_CFG_REG */

#define CSL_IOMUX_SDFM0_CLK1_CFG_REG_FUNC_SEL_MASK                             (0x0000000FU)
#define CSL_IOMUX_SDFM0_CLK1_CFG_REG_FUNC_SEL_SHIFT                            (0x00000000U)
#define CSL_IOMUX_SDFM0_CLK1_CFG_REG_FUNC_SEL_RESETVAL                         (0x00000007U)
#define CSL_IOMUX_SDFM0_CLK1_CFG_REG_FUNC_SEL_MAX                              (0x0000000FU)

#define CSL_IOMUX_SDFM0_CLK1_CFG_REG_IE_OVERRIDE_CTRL_MASK                     (0x00000010U)
#define CSL_IOMUX_SDFM0_CLK1_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                    (0x00000004U)
#define CSL_IOMUX_SDFM0_CLK1_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_IOMUX_SDFM0_CLK1_CFG_REG_IE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_IOMUX_SDFM0_CLK1_CFG_REG_IE_OVERRIDE_MASK                          (0x00000020U)
#define CSL_IOMUX_SDFM0_CLK1_CFG_REG_IE_OVERRIDE_SHIFT                         (0x00000005U)
#define CSL_IOMUX_SDFM0_CLK1_CFG_REG_IE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_IOMUX_SDFM0_CLK1_CFG_REG_IE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_IOMUX_SDFM0_CLK1_CFG_REG_OE_OVERRIDE_CTRL_MASK                     (0x00000040U)
#define CSL_IOMUX_SDFM0_CLK1_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                    (0x00000006U)
#define CSL_IOMUX_SDFM0_CLK1_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_IOMUX_SDFM0_CLK1_CFG_REG_OE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_IOMUX_SDFM0_CLK1_CFG_REG_OE_OVERRIDE_MASK                          (0x00000080U)
#define CSL_IOMUX_SDFM0_CLK1_CFG_REG_OE_OVERRIDE_SHIFT                         (0x00000007U)
#define CSL_IOMUX_SDFM0_CLK1_CFG_REG_OE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_IOMUX_SDFM0_CLK1_CFG_REG_OE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_IOMUX_SDFM0_CLK1_CFG_REG_PI_MASK                                   (0x00000100U)
#define CSL_IOMUX_SDFM0_CLK1_CFG_REG_PI_SHIFT                                  (0x00000008U)
#define CSL_IOMUX_SDFM0_CLK1_CFG_REG_PI_RESETVAL                               (0x00000001U)
#define CSL_IOMUX_SDFM0_CLK1_CFG_REG_PI_MAX                                    (0x00000001U)

#define CSL_IOMUX_SDFM0_CLK1_CFG_REG_PUPDSEL_MASK                              (0x00000200U)
#define CSL_IOMUX_SDFM0_CLK1_CFG_REG_PUPDSEL_SHIFT                             (0x00000009U)
#define CSL_IOMUX_SDFM0_CLK1_CFG_REG_PUPDSEL_RESETVAL                          (0x00000000U)
#define CSL_IOMUX_SDFM0_CLK1_CFG_REG_PUPDSEL_MAX                               (0x00000001U)

#define CSL_IOMUX_SDFM0_CLK1_CFG_REG_SC1_MASK                                  (0x00000400U)
#define CSL_IOMUX_SDFM0_CLK1_CFG_REG_SC1_SHIFT                                 (0x0000000AU)
#define CSL_IOMUX_SDFM0_CLK1_CFG_REG_SC1_RESETVAL                              (0x00000001U)
#define CSL_IOMUX_SDFM0_CLK1_CFG_REG_SC1_MAX                                   (0x00000001U)

#define CSL_IOMUX_SDFM0_CLK1_CFG_REG_GPIO_SEL_MASK                             (0x00030000U)
#define CSL_IOMUX_SDFM0_CLK1_CFG_REG_GPIO_SEL_SHIFT                            (0x00000010U)
#define CSL_IOMUX_SDFM0_CLK1_CFG_REG_GPIO_SEL_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_SDFM0_CLK1_CFG_REG_GPIO_SEL_MAX                              (0x00000003U)

#define CSL_IOMUX_SDFM0_CLK1_CFG_REG_QUAL_SEL_MASK                             (0x000C0000U)
#define CSL_IOMUX_SDFM0_CLK1_CFG_REG_QUAL_SEL_SHIFT                            (0x00000012U)
#define CSL_IOMUX_SDFM0_CLK1_CFG_REG_QUAL_SEL_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_SDFM0_CLK1_CFG_REG_QUAL_SEL_MAX                              (0x00000003U)

#define CSL_IOMUX_SDFM0_CLK1_CFG_REG_INP_INV_SEL_MASK                          (0x00100000U)
#define CSL_IOMUX_SDFM0_CLK1_CFG_REG_INP_INV_SEL_SHIFT                         (0x00000014U)
#define CSL_IOMUX_SDFM0_CLK1_CFG_REG_INP_INV_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_SDFM0_CLK1_CFG_REG_INP_INV_SEL_MAX                           (0x00000001U)

#define CSL_IOMUX_SDFM0_CLK1_CFG_REG_ICSSM_GPIO_SEL_MASK                       (0x00200000U)
#define CSL_IOMUX_SDFM0_CLK1_CFG_REG_ICSSM_GPIO_SEL_SHIFT                      (0x00000015U)
#define CSL_IOMUX_SDFM0_CLK1_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                   (0x00000000U)
#define CSL_IOMUX_SDFM0_CLK1_CFG_REG_ICSSM_GPIO_SEL_MAX                        (0x00000001U)

#define CSL_IOMUX_SDFM0_CLK1_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                  (0x00400000U)
#define CSL_IOMUX_SDFM0_CLK1_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                 (0x00000016U)
#define CSL_IOMUX_SDFM0_CLK1_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL              (0x00000000U)
#define CSL_IOMUX_SDFM0_CLK1_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                   (0x00000001U)

#define CSL_IOMUX_SDFM0_CLK1_CFG_REG_HSMODE_MASK                               (0x40000000U)
#define CSL_IOMUX_SDFM0_CLK1_CFG_REG_HSMODE_SHIFT                              (0x0000001EU)
#define CSL_IOMUX_SDFM0_CLK1_CFG_REG_HSMODE_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_SDFM0_CLK1_CFG_REG_HSMODE_MAX                                (0x00000001U)

#define CSL_IOMUX_SDFM0_CLK1_CFG_REG_HSMASTER_MASK                             (0x80000000U)
#define CSL_IOMUX_SDFM0_CLK1_CFG_REG_HSMASTER_SHIFT                            (0x0000001FU)
#define CSL_IOMUX_SDFM0_CLK1_CFG_REG_HSMASTER_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_SDFM0_CLK1_CFG_REG_HSMASTER_MAX                              (0x00000001U)

#define CSL_IOMUX_SDFM0_CLK1_CFG_REG_RESETVAL                                  (0x000005F7U)

/* SDFM0_D1_CFG_REG */

#define CSL_IOMUX_SDFM0_D1_CFG_REG_FUNC_SEL_MASK                               (0x0000000FU)
#define CSL_IOMUX_SDFM0_D1_CFG_REG_FUNC_SEL_SHIFT                              (0x00000000U)
#define CSL_IOMUX_SDFM0_D1_CFG_REG_FUNC_SEL_RESETVAL                           (0x00000007U)
#define CSL_IOMUX_SDFM0_D1_CFG_REG_FUNC_SEL_MAX                                (0x0000000FU)

#define CSL_IOMUX_SDFM0_D1_CFG_REG_IE_OVERRIDE_CTRL_MASK                       (0x00000010U)
#define CSL_IOMUX_SDFM0_D1_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                      (0x00000004U)
#define CSL_IOMUX_SDFM0_D1_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_SDFM0_D1_CFG_REG_IE_OVERRIDE_CTRL_MAX                        (0x00000001U)

#define CSL_IOMUX_SDFM0_D1_CFG_REG_IE_OVERRIDE_MASK                            (0x00000020U)
#define CSL_IOMUX_SDFM0_D1_CFG_REG_IE_OVERRIDE_SHIFT                           (0x00000005U)
#define CSL_IOMUX_SDFM0_D1_CFG_REG_IE_OVERRIDE_RESETVAL                        (0x00000001U)
#define CSL_IOMUX_SDFM0_D1_CFG_REG_IE_OVERRIDE_MAX                             (0x00000001U)

#define CSL_IOMUX_SDFM0_D1_CFG_REG_OE_OVERRIDE_CTRL_MASK                       (0x00000040U)
#define CSL_IOMUX_SDFM0_D1_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                      (0x00000006U)
#define CSL_IOMUX_SDFM0_D1_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_SDFM0_D1_CFG_REG_OE_OVERRIDE_CTRL_MAX                        (0x00000001U)

#define CSL_IOMUX_SDFM0_D1_CFG_REG_OE_OVERRIDE_MASK                            (0x00000080U)
#define CSL_IOMUX_SDFM0_D1_CFG_REG_OE_OVERRIDE_SHIFT                           (0x00000007U)
#define CSL_IOMUX_SDFM0_D1_CFG_REG_OE_OVERRIDE_RESETVAL                        (0x00000001U)
#define CSL_IOMUX_SDFM0_D1_CFG_REG_OE_OVERRIDE_MAX                             (0x00000001U)

#define CSL_IOMUX_SDFM0_D1_CFG_REG_PI_MASK                                     (0x00000100U)
#define CSL_IOMUX_SDFM0_D1_CFG_REG_PI_SHIFT                                    (0x00000008U)
#define CSL_IOMUX_SDFM0_D1_CFG_REG_PI_RESETVAL                                 (0x00000001U)
#define CSL_IOMUX_SDFM0_D1_CFG_REG_PI_MAX                                      (0x00000001U)

#define CSL_IOMUX_SDFM0_D1_CFG_REG_PUPDSEL_MASK                                (0x00000200U)
#define CSL_IOMUX_SDFM0_D1_CFG_REG_PUPDSEL_SHIFT                               (0x00000009U)
#define CSL_IOMUX_SDFM0_D1_CFG_REG_PUPDSEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_SDFM0_D1_CFG_REG_PUPDSEL_MAX                                 (0x00000001U)

#define CSL_IOMUX_SDFM0_D1_CFG_REG_SC1_MASK                                    (0x00000400U)
#define CSL_IOMUX_SDFM0_D1_CFG_REG_SC1_SHIFT                                   (0x0000000AU)
#define CSL_IOMUX_SDFM0_D1_CFG_REG_SC1_RESETVAL                                (0x00000001U)
#define CSL_IOMUX_SDFM0_D1_CFG_REG_SC1_MAX                                     (0x00000001U)

#define CSL_IOMUX_SDFM0_D1_CFG_REG_GPIO_SEL_MASK                               (0x00030000U)
#define CSL_IOMUX_SDFM0_D1_CFG_REG_GPIO_SEL_SHIFT                              (0x00000010U)
#define CSL_IOMUX_SDFM0_D1_CFG_REG_GPIO_SEL_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_SDFM0_D1_CFG_REG_GPIO_SEL_MAX                                (0x00000003U)

#define CSL_IOMUX_SDFM0_D1_CFG_REG_QUAL_SEL_MASK                               (0x000C0000U)
#define CSL_IOMUX_SDFM0_D1_CFG_REG_QUAL_SEL_SHIFT                              (0x00000012U)
#define CSL_IOMUX_SDFM0_D1_CFG_REG_QUAL_SEL_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_SDFM0_D1_CFG_REG_QUAL_SEL_MAX                                (0x00000003U)

#define CSL_IOMUX_SDFM0_D1_CFG_REG_INP_INV_SEL_MASK                            (0x00100000U)
#define CSL_IOMUX_SDFM0_D1_CFG_REG_INP_INV_SEL_SHIFT                           (0x00000014U)
#define CSL_IOMUX_SDFM0_D1_CFG_REG_INP_INV_SEL_RESETVAL                        (0x00000000U)
#define CSL_IOMUX_SDFM0_D1_CFG_REG_INP_INV_SEL_MAX                             (0x00000001U)

#define CSL_IOMUX_SDFM0_D1_CFG_REG_ICSSM_GPIO_SEL_MASK                         (0x00200000U)
#define CSL_IOMUX_SDFM0_D1_CFG_REG_ICSSM_GPIO_SEL_SHIFT                        (0x00000015U)
#define CSL_IOMUX_SDFM0_D1_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_SDFM0_D1_CFG_REG_ICSSM_GPIO_SEL_MAX                          (0x00000001U)

#define CSL_IOMUX_SDFM0_D1_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                    (0x00400000U)
#define CSL_IOMUX_SDFM0_D1_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                   (0x00000016U)
#define CSL_IOMUX_SDFM0_D1_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL                (0x00000000U)
#define CSL_IOMUX_SDFM0_D1_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                     (0x00000001U)

#define CSL_IOMUX_SDFM0_D1_CFG_REG_HSMODE_MASK                                 (0x40000000U)
#define CSL_IOMUX_SDFM0_D1_CFG_REG_HSMODE_SHIFT                                (0x0000001EU)
#define CSL_IOMUX_SDFM0_D1_CFG_REG_HSMODE_RESETVAL                             (0x00000000U)
#define CSL_IOMUX_SDFM0_D1_CFG_REG_HSMODE_MAX                                  (0x00000001U)

#define CSL_IOMUX_SDFM0_D1_CFG_REG_HSMASTER_MASK                               (0x80000000U)
#define CSL_IOMUX_SDFM0_D1_CFG_REG_HSMASTER_SHIFT                              (0x0000001FU)
#define CSL_IOMUX_SDFM0_D1_CFG_REG_HSMASTER_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_SDFM0_D1_CFG_REG_HSMASTER_MAX                                (0x00000001U)

#define CSL_IOMUX_SDFM0_D1_CFG_REG_RESETVAL                                    (0x000005F7U)

/* SDFM0_CLK2_CFG_REG */

#define CSL_IOMUX_SDFM0_CLK2_CFG_REG_FUNC_SEL_MASK                             (0x0000000FU)
#define CSL_IOMUX_SDFM0_CLK2_CFG_REG_FUNC_SEL_SHIFT                            (0x00000000U)
#define CSL_IOMUX_SDFM0_CLK2_CFG_REG_FUNC_SEL_RESETVAL                         (0x00000007U)
#define CSL_IOMUX_SDFM0_CLK2_CFG_REG_FUNC_SEL_MAX                              (0x0000000FU)

#define CSL_IOMUX_SDFM0_CLK2_CFG_REG_IE_OVERRIDE_CTRL_MASK                     (0x00000010U)
#define CSL_IOMUX_SDFM0_CLK2_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                    (0x00000004U)
#define CSL_IOMUX_SDFM0_CLK2_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_IOMUX_SDFM0_CLK2_CFG_REG_IE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_IOMUX_SDFM0_CLK2_CFG_REG_IE_OVERRIDE_MASK                          (0x00000020U)
#define CSL_IOMUX_SDFM0_CLK2_CFG_REG_IE_OVERRIDE_SHIFT                         (0x00000005U)
#define CSL_IOMUX_SDFM0_CLK2_CFG_REG_IE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_IOMUX_SDFM0_CLK2_CFG_REG_IE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_IOMUX_SDFM0_CLK2_CFG_REG_OE_OVERRIDE_CTRL_MASK                     (0x00000040U)
#define CSL_IOMUX_SDFM0_CLK2_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                    (0x00000006U)
#define CSL_IOMUX_SDFM0_CLK2_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_IOMUX_SDFM0_CLK2_CFG_REG_OE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_IOMUX_SDFM0_CLK2_CFG_REG_OE_OVERRIDE_MASK                          (0x00000080U)
#define CSL_IOMUX_SDFM0_CLK2_CFG_REG_OE_OVERRIDE_SHIFT                         (0x00000007U)
#define CSL_IOMUX_SDFM0_CLK2_CFG_REG_OE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_IOMUX_SDFM0_CLK2_CFG_REG_OE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_IOMUX_SDFM0_CLK2_CFG_REG_PI_MASK                                   (0x00000100U)
#define CSL_IOMUX_SDFM0_CLK2_CFG_REG_PI_SHIFT                                  (0x00000008U)
#define CSL_IOMUX_SDFM0_CLK2_CFG_REG_PI_RESETVAL                               (0x00000001U)
#define CSL_IOMUX_SDFM0_CLK2_CFG_REG_PI_MAX                                    (0x00000001U)

#define CSL_IOMUX_SDFM0_CLK2_CFG_REG_PUPDSEL_MASK                              (0x00000200U)
#define CSL_IOMUX_SDFM0_CLK2_CFG_REG_PUPDSEL_SHIFT                             (0x00000009U)
#define CSL_IOMUX_SDFM0_CLK2_CFG_REG_PUPDSEL_RESETVAL                          (0x00000000U)
#define CSL_IOMUX_SDFM0_CLK2_CFG_REG_PUPDSEL_MAX                               (0x00000001U)

#define CSL_IOMUX_SDFM0_CLK2_CFG_REG_SC1_MASK                                  (0x00000400U)
#define CSL_IOMUX_SDFM0_CLK2_CFG_REG_SC1_SHIFT                                 (0x0000000AU)
#define CSL_IOMUX_SDFM0_CLK2_CFG_REG_SC1_RESETVAL                              (0x00000001U)
#define CSL_IOMUX_SDFM0_CLK2_CFG_REG_SC1_MAX                                   (0x00000001U)

#define CSL_IOMUX_SDFM0_CLK2_CFG_REG_GPIO_SEL_MASK                             (0x00030000U)
#define CSL_IOMUX_SDFM0_CLK2_CFG_REG_GPIO_SEL_SHIFT                            (0x00000010U)
#define CSL_IOMUX_SDFM0_CLK2_CFG_REG_GPIO_SEL_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_SDFM0_CLK2_CFG_REG_GPIO_SEL_MAX                              (0x00000003U)

#define CSL_IOMUX_SDFM0_CLK2_CFG_REG_QUAL_SEL_MASK                             (0x000C0000U)
#define CSL_IOMUX_SDFM0_CLK2_CFG_REG_QUAL_SEL_SHIFT                            (0x00000012U)
#define CSL_IOMUX_SDFM0_CLK2_CFG_REG_QUAL_SEL_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_SDFM0_CLK2_CFG_REG_QUAL_SEL_MAX                              (0x00000003U)

#define CSL_IOMUX_SDFM0_CLK2_CFG_REG_INP_INV_SEL_MASK                          (0x00100000U)
#define CSL_IOMUX_SDFM0_CLK2_CFG_REG_INP_INV_SEL_SHIFT                         (0x00000014U)
#define CSL_IOMUX_SDFM0_CLK2_CFG_REG_INP_INV_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_SDFM0_CLK2_CFG_REG_INP_INV_SEL_MAX                           (0x00000001U)

#define CSL_IOMUX_SDFM0_CLK2_CFG_REG_ICSSM_GPIO_SEL_MASK                       (0x00200000U)
#define CSL_IOMUX_SDFM0_CLK2_CFG_REG_ICSSM_GPIO_SEL_SHIFT                      (0x00000015U)
#define CSL_IOMUX_SDFM0_CLK2_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                   (0x00000000U)
#define CSL_IOMUX_SDFM0_CLK2_CFG_REG_ICSSM_GPIO_SEL_MAX                        (0x00000001U)

#define CSL_IOMUX_SDFM0_CLK2_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                  (0x00400000U)
#define CSL_IOMUX_SDFM0_CLK2_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                 (0x00000016U)
#define CSL_IOMUX_SDFM0_CLK2_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL              (0x00000000U)
#define CSL_IOMUX_SDFM0_CLK2_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                   (0x00000001U)

#define CSL_IOMUX_SDFM0_CLK2_CFG_REG_HSMODE_MASK                               (0x40000000U)
#define CSL_IOMUX_SDFM0_CLK2_CFG_REG_HSMODE_SHIFT                              (0x0000001EU)
#define CSL_IOMUX_SDFM0_CLK2_CFG_REG_HSMODE_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_SDFM0_CLK2_CFG_REG_HSMODE_MAX                                (0x00000001U)

#define CSL_IOMUX_SDFM0_CLK2_CFG_REG_HSMASTER_MASK                             (0x80000000U)
#define CSL_IOMUX_SDFM0_CLK2_CFG_REG_HSMASTER_SHIFT                            (0x0000001FU)
#define CSL_IOMUX_SDFM0_CLK2_CFG_REG_HSMASTER_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_SDFM0_CLK2_CFG_REG_HSMASTER_MAX                              (0x00000001U)

#define CSL_IOMUX_SDFM0_CLK2_CFG_REG_RESETVAL                                  (0x000005F7U)

/* SDFM0_D2_CFG_REG */

#define CSL_IOMUX_SDFM0_D2_CFG_REG_FUNC_SEL_MASK                               (0x0000000FU)
#define CSL_IOMUX_SDFM0_D2_CFG_REG_FUNC_SEL_SHIFT                              (0x00000000U)
#define CSL_IOMUX_SDFM0_D2_CFG_REG_FUNC_SEL_RESETVAL                           (0x00000007U)
#define CSL_IOMUX_SDFM0_D2_CFG_REG_FUNC_SEL_MAX                                (0x0000000FU)

#define CSL_IOMUX_SDFM0_D2_CFG_REG_IE_OVERRIDE_CTRL_MASK                       (0x00000010U)
#define CSL_IOMUX_SDFM0_D2_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                      (0x00000004U)
#define CSL_IOMUX_SDFM0_D2_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_SDFM0_D2_CFG_REG_IE_OVERRIDE_CTRL_MAX                        (0x00000001U)

#define CSL_IOMUX_SDFM0_D2_CFG_REG_IE_OVERRIDE_MASK                            (0x00000020U)
#define CSL_IOMUX_SDFM0_D2_CFG_REG_IE_OVERRIDE_SHIFT                           (0x00000005U)
#define CSL_IOMUX_SDFM0_D2_CFG_REG_IE_OVERRIDE_RESETVAL                        (0x00000001U)
#define CSL_IOMUX_SDFM0_D2_CFG_REG_IE_OVERRIDE_MAX                             (0x00000001U)

#define CSL_IOMUX_SDFM0_D2_CFG_REG_OE_OVERRIDE_CTRL_MASK                       (0x00000040U)
#define CSL_IOMUX_SDFM0_D2_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                      (0x00000006U)
#define CSL_IOMUX_SDFM0_D2_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_SDFM0_D2_CFG_REG_OE_OVERRIDE_CTRL_MAX                        (0x00000001U)

#define CSL_IOMUX_SDFM0_D2_CFG_REG_OE_OVERRIDE_MASK                            (0x00000080U)
#define CSL_IOMUX_SDFM0_D2_CFG_REG_OE_OVERRIDE_SHIFT                           (0x00000007U)
#define CSL_IOMUX_SDFM0_D2_CFG_REG_OE_OVERRIDE_RESETVAL                        (0x00000001U)
#define CSL_IOMUX_SDFM0_D2_CFG_REG_OE_OVERRIDE_MAX                             (0x00000001U)

#define CSL_IOMUX_SDFM0_D2_CFG_REG_PI_MASK                                     (0x00000100U)
#define CSL_IOMUX_SDFM0_D2_CFG_REG_PI_SHIFT                                    (0x00000008U)
#define CSL_IOMUX_SDFM0_D2_CFG_REG_PI_RESETVAL                                 (0x00000001U)
#define CSL_IOMUX_SDFM0_D2_CFG_REG_PI_MAX                                      (0x00000001U)

#define CSL_IOMUX_SDFM0_D2_CFG_REG_PUPDSEL_MASK                                (0x00000200U)
#define CSL_IOMUX_SDFM0_D2_CFG_REG_PUPDSEL_SHIFT                               (0x00000009U)
#define CSL_IOMUX_SDFM0_D2_CFG_REG_PUPDSEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_SDFM0_D2_CFG_REG_PUPDSEL_MAX                                 (0x00000001U)

#define CSL_IOMUX_SDFM0_D2_CFG_REG_SC1_MASK                                    (0x00000400U)
#define CSL_IOMUX_SDFM0_D2_CFG_REG_SC1_SHIFT                                   (0x0000000AU)
#define CSL_IOMUX_SDFM0_D2_CFG_REG_SC1_RESETVAL                                (0x00000001U)
#define CSL_IOMUX_SDFM0_D2_CFG_REG_SC1_MAX                                     (0x00000001U)

#define CSL_IOMUX_SDFM0_D2_CFG_REG_GPIO_SEL_MASK                               (0x00030000U)
#define CSL_IOMUX_SDFM0_D2_CFG_REG_GPIO_SEL_SHIFT                              (0x00000010U)
#define CSL_IOMUX_SDFM0_D2_CFG_REG_GPIO_SEL_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_SDFM0_D2_CFG_REG_GPIO_SEL_MAX                                (0x00000003U)

#define CSL_IOMUX_SDFM0_D2_CFG_REG_QUAL_SEL_MASK                               (0x000C0000U)
#define CSL_IOMUX_SDFM0_D2_CFG_REG_QUAL_SEL_SHIFT                              (0x00000012U)
#define CSL_IOMUX_SDFM0_D2_CFG_REG_QUAL_SEL_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_SDFM0_D2_CFG_REG_QUAL_SEL_MAX                                (0x00000003U)

#define CSL_IOMUX_SDFM0_D2_CFG_REG_INP_INV_SEL_MASK                            (0x00100000U)
#define CSL_IOMUX_SDFM0_D2_CFG_REG_INP_INV_SEL_SHIFT                           (0x00000014U)
#define CSL_IOMUX_SDFM0_D2_CFG_REG_INP_INV_SEL_RESETVAL                        (0x00000000U)
#define CSL_IOMUX_SDFM0_D2_CFG_REG_INP_INV_SEL_MAX                             (0x00000001U)

#define CSL_IOMUX_SDFM0_D2_CFG_REG_ICSSM_GPIO_SEL_MASK                         (0x00200000U)
#define CSL_IOMUX_SDFM0_D2_CFG_REG_ICSSM_GPIO_SEL_SHIFT                        (0x00000015U)
#define CSL_IOMUX_SDFM0_D2_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_SDFM0_D2_CFG_REG_ICSSM_GPIO_SEL_MAX                          (0x00000001U)

#define CSL_IOMUX_SDFM0_D2_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                    (0x00400000U)
#define CSL_IOMUX_SDFM0_D2_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                   (0x00000016U)
#define CSL_IOMUX_SDFM0_D2_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL                (0x00000000U)
#define CSL_IOMUX_SDFM0_D2_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                     (0x00000001U)

#define CSL_IOMUX_SDFM0_D2_CFG_REG_HSMODE_MASK                                 (0x40000000U)
#define CSL_IOMUX_SDFM0_D2_CFG_REG_HSMODE_SHIFT                                (0x0000001EU)
#define CSL_IOMUX_SDFM0_D2_CFG_REG_HSMODE_RESETVAL                             (0x00000000U)
#define CSL_IOMUX_SDFM0_D2_CFG_REG_HSMODE_MAX                                  (0x00000001U)

#define CSL_IOMUX_SDFM0_D2_CFG_REG_HSMASTER_MASK                               (0x80000000U)
#define CSL_IOMUX_SDFM0_D2_CFG_REG_HSMASTER_SHIFT                              (0x0000001FU)
#define CSL_IOMUX_SDFM0_D2_CFG_REG_HSMASTER_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_SDFM0_D2_CFG_REG_HSMASTER_MAX                                (0x00000001U)

#define CSL_IOMUX_SDFM0_D2_CFG_REG_RESETVAL                                    (0x000005F7U)

/* SDFM0_CLK3_CFG_REG */

#define CSL_IOMUX_SDFM0_CLK3_CFG_REG_FUNC_SEL_MASK                             (0x0000000FU)
#define CSL_IOMUX_SDFM0_CLK3_CFG_REG_FUNC_SEL_SHIFT                            (0x00000000U)
#define CSL_IOMUX_SDFM0_CLK3_CFG_REG_FUNC_SEL_RESETVAL                         (0x00000007U)
#define CSL_IOMUX_SDFM0_CLK3_CFG_REG_FUNC_SEL_MAX                              (0x0000000FU)

#define CSL_IOMUX_SDFM0_CLK3_CFG_REG_IE_OVERRIDE_CTRL_MASK                     (0x00000010U)
#define CSL_IOMUX_SDFM0_CLK3_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                    (0x00000004U)
#define CSL_IOMUX_SDFM0_CLK3_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_IOMUX_SDFM0_CLK3_CFG_REG_IE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_IOMUX_SDFM0_CLK3_CFG_REG_IE_OVERRIDE_MASK                          (0x00000020U)
#define CSL_IOMUX_SDFM0_CLK3_CFG_REG_IE_OVERRIDE_SHIFT                         (0x00000005U)
#define CSL_IOMUX_SDFM0_CLK3_CFG_REG_IE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_IOMUX_SDFM0_CLK3_CFG_REG_IE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_IOMUX_SDFM0_CLK3_CFG_REG_OE_OVERRIDE_CTRL_MASK                     (0x00000040U)
#define CSL_IOMUX_SDFM0_CLK3_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                    (0x00000006U)
#define CSL_IOMUX_SDFM0_CLK3_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_IOMUX_SDFM0_CLK3_CFG_REG_OE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_IOMUX_SDFM0_CLK3_CFG_REG_OE_OVERRIDE_MASK                          (0x00000080U)
#define CSL_IOMUX_SDFM0_CLK3_CFG_REG_OE_OVERRIDE_SHIFT                         (0x00000007U)
#define CSL_IOMUX_SDFM0_CLK3_CFG_REG_OE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_IOMUX_SDFM0_CLK3_CFG_REG_OE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_IOMUX_SDFM0_CLK3_CFG_REG_PI_MASK                                   (0x00000100U)
#define CSL_IOMUX_SDFM0_CLK3_CFG_REG_PI_SHIFT                                  (0x00000008U)
#define CSL_IOMUX_SDFM0_CLK3_CFG_REG_PI_RESETVAL                               (0x00000001U)
#define CSL_IOMUX_SDFM0_CLK3_CFG_REG_PI_MAX                                    (0x00000001U)

#define CSL_IOMUX_SDFM0_CLK3_CFG_REG_PUPDSEL_MASK                              (0x00000200U)
#define CSL_IOMUX_SDFM0_CLK3_CFG_REG_PUPDSEL_SHIFT                             (0x00000009U)
#define CSL_IOMUX_SDFM0_CLK3_CFG_REG_PUPDSEL_RESETVAL                          (0x00000000U)
#define CSL_IOMUX_SDFM0_CLK3_CFG_REG_PUPDSEL_MAX                               (0x00000001U)

#define CSL_IOMUX_SDFM0_CLK3_CFG_REG_SC1_MASK                                  (0x00000400U)
#define CSL_IOMUX_SDFM0_CLK3_CFG_REG_SC1_SHIFT                                 (0x0000000AU)
#define CSL_IOMUX_SDFM0_CLK3_CFG_REG_SC1_RESETVAL                              (0x00000001U)
#define CSL_IOMUX_SDFM0_CLK3_CFG_REG_SC1_MAX                                   (0x00000001U)

#define CSL_IOMUX_SDFM0_CLK3_CFG_REG_GPIO_SEL_MASK                             (0x00030000U)
#define CSL_IOMUX_SDFM0_CLK3_CFG_REG_GPIO_SEL_SHIFT                            (0x00000010U)
#define CSL_IOMUX_SDFM0_CLK3_CFG_REG_GPIO_SEL_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_SDFM0_CLK3_CFG_REG_GPIO_SEL_MAX                              (0x00000003U)

#define CSL_IOMUX_SDFM0_CLK3_CFG_REG_QUAL_SEL_MASK                             (0x000C0000U)
#define CSL_IOMUX_SDFM0_CLK3_CFG_REG_QUAL_SEL_SHIFT                            (0x00000012U)
#define CSL_IOMUX_SDFM0_CLK3_CFG_REG_QUAL_SEL_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_SDFM0_CLK3_CFG_REG_QUAL_SEL_MAX                              (0x00000003U)

#define CSL_IOMUX_SDFM0_CLK3_CFG_REG_INP_INV_SEL_MASK                          (0x00100000U)
#define CSL_IOMUX_SDFM0_CLK3_CFG_REG_INP_INV_SEL_SHIFT                         (0x00000014U)
#define CSL_IOMUX_SDFM0_CLK3_CFG_REG_INP_INV_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_SDFM0_CLK3_CFG_REG_INP_INV_SEL_MAX                           (0x00000001U)

#define CSL_IOMUX_SDFM0_CLK3_CFG_REG_ICSSM_GPIO_SEL_MASK                       (0x00200000U)
#define CSL_IOMUX_SDFM0_CLK3_CFG_REG_ICSSM_GPIO_SEL_SHIFT                      (0x00000015U)
#define CSL_IOMUX_SDFM0_CLK3_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                   (0x00000000U)
#define CSL_IOMUX_SDFM0_CLK3_CFG_REG_ICSSM_GPIO_SEL_MAX                        (0x00000001U)

#define CSL_IOMUX_SDFM0_CLK3_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                  (0x00400000U)
#define CSL_IOMUX_SDFM0_CLK3_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                 (0x00000016U)
#define CSL_IOMUX_SDFM0_CLK3_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL              (0x00000000U)
#define CSL_IOMUX_SDFM0_CLK3_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                   (0x00000001U)

#define CSL_IOMUX_SDFM0_CLK3_CFG_REG_HSMODE_MASK                               (0x40000000U)
#define CSL_IOMUX_SDFM0_CLK3_CFG_REG_HSMODE_SHIFT                              (0x0000001EU)
#define CSL_IOMUX_SDFM0_CLK3_CFG_REG_HSMODE_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_SDFM0_CLK3_CFG_REG_HSMODE_MAX                                (0x00000001U)

#define CSL_IOMUX_SDFM0_CLK3_CFG_REG_HSMASTER_MASK                             (0x80000000U)
#define CSL_IOMUX_SDFM0_CLK3_CFG_REG_HSMASTER_SHIFT                            (0x0000001FU)
#define CSL_IOMUX_SDFM0_CLK3_CFG_REG_HSMASTER_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_SDFM0_CLK3_CFG_REG_HSMASTER_MAX                              (0x00000001U)

#define CSL_IOMUX_SDFM0_CLK3_CFG_REG_RESETVAL                                  (0x000005F7U)

/* SDFM0_D3_CFG_REG */

#define CSL_IOMUX_SDFM0_D3_CFG_REG_FUNC_SEL_MASK                               (0x0000000FU)
#define CSL_IOMUX_SDFM0_D3_CFG_REG_FUNC_SEL_SHIFT                              (0x00000000U)
#define CSL_IOMUX_SDFM0_D3_CFG_REG_FUNC_SEL_RESETVAL                           (0x00000007U)
#define CSL_IOMUX_SDFM0_D3_CFG_REG_FUNC_SEL_MAX                                (0x0000000FU)

#define CSL_IOMUX_SDFM0_D3_CFG_REG_IE_OVERRIDE_CTRL_MASK                       (0x00000010U)
#define CSL_IOMUX_SDFM0_D3_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                      (0x00000004U)
#define CSL_IOMUX_SDFM0_D3_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_SDFM0_D3_CFG_REG_IE_OVERRIDE_CTRL_MAX                        (0x00000001U)

#define CSL_IOMUX_SDFM0_D3_CFG_REG_IE_OVERRIDE_MASK                            (0x00000020U)
#define CSL_IOMUX_SDFM0_D3_CFG_REG_IE_OVERRIDE_SHIFT                           (0x00000005U)
#define CSL_IOMUX_SDFM0_D3_CFG_REG_IE_OVERRIDE_RESETVAL                        (0x00000001U)
#define CSL_IOMUX_SDFM0_D3_CFG_REG_IE_OVERRIDE_MAX                             (0x00000001U)

#define CSL_IOMUX_SDFM0_D3_CFG_REG_OE_OVERRIDE_CTRL_MASK                       (0x00000040U)
#define CSL_IOMUX_SDFM0_D3_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                      (0x00000006U)
#define CSL_IOMUX_SDFM0_D3_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_SDFM0_D3_CFG_REG_OE_OVERRIDE_CTRL_MAX                        (0x00000001U)

#define CSL_IOMUX_SDFM0_D3_CFG_REG_OE_OVERRIDE_MASK                            (0x00000080U)
#define CSL_IOMUX_SDFM0_D3_CFG_REG_OE_OVERRIDE_SHIFT                           (0x00000007U)
#define CSL_IOMUX_SDFM0_D3_CFG_REG_OE_OVERRIDE_RESETVAL                        (0x00000001U)
#define CSL_IOMUX_SDFM0_D3_CFG_REG_OE_OVERRIDE_MAX                             (0x00000001U)

#define CSL_IOMUX_SDFM0_D3_CFG_REG_PI_MASK                                     (0x00000100U)
#define CSL_IOMUX_SDFM0_D3_CFG_REG_PI_SHIFT                                    (0x00000008U)
#define CSL_IOMUX_SDFM0_D3_CFG_REG_PI_RESETVAL                                 (0x00000001U)
#define CSL_IOMUX_SDFM0_D3_CFG_REG_PI_MAX                                      (0x00000001U)

#define CSL_IOMUX_SDFM0_D3_CFG_REG_PUPDSEL_MASK                                (0x00000200U)
#define CSL_IOMUX_SDFM0_D3_CFG_REG_PUPDSEL_SHIFT                               (0x00000009U)
#define CSL_IOMUX_SDFM0_D3_CFG_REG_PUPDSEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_SDFM0_D3_CFG_REG_PUPDSEL_MAX                                 (0x00000001U)

#define CSL_IOMUX_SDFM0_D3_CFG_REG_SC1_MASK                                    (0x00000400U)
#define CSL_IOMUX_SDFM0_D3_CFG_REG_SC1_SHIFT                                   (0x0000000AU)
#define CSL_IOMUX_SDFM0_D3_CFG_REG_SC1_RESETVAL                                (0x00000001U)
#define CSL_IOMUX_SDFM0_D3_CFG_REG_SC1_MAX                                     (0x00000001U)

#define CSL_IOMUX_SDFM0_D3_CFG_REG_GPIO_SEL_MASK                               (0x00030000U)
#define CSL_IOMUX_SDFM0_D3_CFG_REG_GPIO_SEL_SHIFT                              (0x00000010U)
#define CSL_IOMUX_SDFM0_D3_CFG_REG_GPIO_SEL_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_SDFM0_D3_CFG_REG_GPIO_SEL_MAX                                (0x00000003U)

#define CSL_IOMUX_SDFM0_D3_CFG_REG_QUAL_SEL_MASK                               (0x000C0000U)
#define CSL_IOMUX_SDFM0_D3_CFG_REG_QUAL_SEL_SHIFT                              (0x00000012U)
#define CSL_IOMUX_SDFM0_D3_CFG_REG_QUAL_SEL_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_SDFM0_D3_CFG_REG_QUAL_SEL_MAX                                (0x00000003U)

#define CSL_IOMUX_SDFM0_D3_CFG_REG_INP_INV_SEL_MASK                            (0x00100000U)
#define CSL_IOMUX_SDFM0_D3_CFG_REG_INP_INV_SEL_SHIFT                           (0x00000014U)
#define CSL_IOMUX_SDFM0_D3_CFG_REG_INP_INV_SEL_RESETVAL                        (0x00000000U)
#define CSL_IOMUX_SDFM0_D3_CFG_REG_INP_INV_SEL_MAX                             (0x00000001U)

#define CSL_IOMUX_SDFM0_D3_CFG_REG_ICSSM_GPIO_SEL_MASK                         (0x00200000U)
#define CSL_IOMUX_SDFM0_D3_CFG_REG_ICSSM_GPIO_SEL_SHIFT                        (0x00000015U)
#define CSL_IOMUX_SDFM0_D3_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_SDFM0_D3_CFG_REG_ICSSM_GPIO_SEL_MAX                          (0x00000001U)

#define CSL_IOMUX_SDFM0_D3_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                    (0x00400000U)
#define CSL_IOMUX_SDFM0_D3_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                   (0x00000016U)
#define CSL_IOMUX_SDFM0_D3_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL                (0x00000000U)
#define CSL_IOMUX_SDFM0_D3_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                     (0x00000001U)

#define CSL_IOMUX_SDFM0_D3_CFG_REG_HSMODE_MASK                                 (0x40000000U)
#define CSL_IOMUX_SDFM0_D3_CFG_REG_HSMODE_SHIFT                                (0x0000001EU)
#define CSL_IOMUX_SDFM0_D3_CFG_REG_HSMODE_RESETVAL                             (0x00000000U)
#define CSL_IOMUX_SDFM0_D3_CFG_REG_HSMODE_MAX                                  (0x00000001U)

#define CSL_IOMUX_SDFM0_D3_CFG_REG_HSMASTER_MASK                               (0x80000000U)
#define CSL_IOMUX_SDFM0_D3_CFG_REG_HSMASTER_SHIFT                              (0x0000001FU)
#define CSL_IOMUX_SDFM0_D3_CFG_REG_HSMASTER_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_SDFM0_D3_CFG_REG_HSMASTER_MAX                                (0x00000001U)

#define CSL_IOMUX_SDFM0_D3_CFG_REG_RESETVAL                                    (0x000005F7U)

/* EQEP0_A_CFG_REG */

#define CSL_IOMUX_EQEP0_A_CFG_REG_FUNC_SEL_MASK                                (0x0000000FU)
#define CSL_IOMUX_EQEP0_A_CFG_REG_FUNC_SEL_SHIFT                               (0x00000000U)
#define CSL_IOMUX_EQEP0_A_CFG_REG_FUNC_SEL_RESETVAL                            (0x00000007U)
#define CSL_IOMUX_EQEP0_A_CFG_REG_FUNC_SEL_MAX                                 (0x0000000FU)

#define CSL_IOMUX_EQEP0_A_CFG_REG_IE_OVERRIDE_CTRL_MASK                        (0x00000010U)
#define CSL_IOMUX_EQEP0_A_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                       (0x00000004U)
#define CSL_IOMUX_EQEP0_A_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_EQEP0_A_CFG_REG_IE_OVERRIDE_CTRL_MAX                         (0x00000001U)

#define CSL_IOMUX_EQEP0_A_CFG_REG_IE_OVERRIDE_MASK                             (0x00000020U)
#define CSL_IOMUX_EQEP0_A_CFG_REG_IE_OVERRIDE_SHIFT                            (0x00000005U)
#define CSL_IOMUX_EQEP0_A_CFG_REG_IE_OVERRIDE_RESETVAL                         (0x00000001U)
#define CSL_IOMUX_EQEP0_A_CFG_REG_IE_OVERRIDE_MAX                              (0x00000001U)

#define CSL_IOMUX_EQEP0_A_CFG_REG_OE_OVERRIDE_CTRL_MASK                        (0x00000040U)
#define CSL_IOMUX_EQEP0_A_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                       (0x00000006U)
#define CSL_IOMUX_EQEP0_A_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_EQEP0_A_CFG_REG_OE_OVERRIDE_CTRL_MAX                         (0x00000001U)

#define CSL_IOMUX_EQEP0_A_CFG_REG_OE_OVERRIDE_MASK                             (0x00000080U)
#define CSL_IOMUX_EQEP0_A_CFG_REG_OE_OVERRIDE_SHIFT                            (0x00000007U)
#define CSL_IOMUX_EQEP0_A_CFG_REG_OE_OVERRIDE_RESETVAL                         (0x00000001U)
#define CSL_IOMUX_EQEP0_A_CFG_REG_OE_OVERRIDE_MAX                              (0x00000001U)

#define CSL_IOMUX_EQEP0_A_CFG_REG_PI_MASK                                      (0x00000100U)
#define CSL_IOMUX_EQEP0_A_CFG_REG_PI_SHIFT                                     (0x00000008U)
#define CSL_IOMUX_EQEP0_A_CFG_REG_PI_RESETVAL                                  (0x00000001U)
#define CSL_IOMUX_EQEP0_A_CFG_REG_PI_MAX                                       (0x00000001U)

#define CSL_IOMUX_EQEP0_A_CFG_REG_PUPDSEL_MASK                                 (0x00000200U)
#define CSL_IOMUX_EQEP0_A_CFG_REG_PUPDSEL_SHIFT                                (0x00000009U)
#define CSL_IOMUX_EQEP0_A_CFG_REG_PUPDSEL_RESETVAL                             (0x00000000U)
#define CSL_IOMUX_EQEP0_A_CFG_REG_PUPDSEL_MAX                                  (0x00000001U)

#define CSL_IOMUX_EQEP0_A_CFG_REG_SC1_MASK                                     (0x00000400U)
#define CSL_IOMUX_EQEP0_A_CFG_REG_SC1_SHIFT                                    (0x0000000AU)
#define CSL_IOMUX_EQEP0_A_CFG_REG_SC1_RESETVAL                                 (0x00000001U)
#define CSL_IOMUX_EQEP0_A_CFG_REG_SC1_MAX                                      (0x00000001U)

#define CSL_IOMUX_EQEP0_A_CFG_REG_GPIO_SEL_MASK                                (0x00030000U)
#define CSL_IOMUX_EQEP0_A_CFG_REG_GPIO_SEL_SHIFT                               (0x00000010U)
#define CSL_IOMUX_EQEP0_A_CFG_REG_GPIO_SEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_EQEP0_A_CFG_REG_GPIO_SEL_MAX                                 (0x00000003U)

#define CSL_IOMUX_EQEP0_A_CFG_REG_QUAL_SEL_MASK                                (0x000C0000U)
#define CSL_IOMUX_EQEP0_A_CFG_REG_QUAL_SEL_SHIFT                               (0x00000012U)
#define CSL_IOMUX_EQEP0_A_CFG_REG_QUAL_SEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_EQEP0_A_CFG_REG_QUAL_SEL_MAX                                 (0x00000003U)

#define CSL_IOMUX_EQEP0_A_CFG_REG_INP_INV_SEL_MASK                             (0x00100000U)
#define CSL_IOMUX_EQEP0_A_CFG_REG_INP_INV_SEL_SHIFT                            (0x00000014U)
#define CSL_IOMUX_EQEP0_A_CFG_REG_INP_INV_SEL_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_EQEP0_A_CFG_REG_INP_INV_SEL_MAX                              (0x00000001U)

#define CSL_IOMUX_EQEP0_A_CFG_REG_ICSSM_GPIO_SEL_MASK                          (0x00200000U)
#define CSL_IOMUX_EQEP0_A_CFG_REG_ICSSM_GPIO_SEL_SHIFT                         (0x00000015U)
#define CSL_IOMUX_EQEP0_A_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_EQEP0_A_CFG_REG_ICSSM_GPIO_SEL_MAX                           (0x00000001U)

#define CSL_IOMUX_EQEP0_A_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                     (0x00400000U)
#define CSL_IOMUX_EQEP0_A_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                    (0x00000016U)
#define CSL_IOMUX_EQEP0_A_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL                 (0x00000000U)
#define CSL_IOMUX_EQEP0_A_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                      (0x00000001U)

#define CSL_IOMUX_EQEP0_A_CFG_REG_HSMODE_MASK                                  (0x40000000U)
#define CSL_IOMUX_EQEP0_A_CFG_REG_HSMODE_SHIFT                                 (0x0000001EU)
#define CSL_IOMUX_EQEP0_A_CFG_REG_HSMODE_RESETVAL                              (0x00000000U)
#define CSL_IOMUX_EQEP0_A_CFG_REG_HSMODE_MAX                                   (0x00000001U)

#define CSL_IOMUX_EQEP0_A_CFG_REG_HSMASTER_MASK                                (0x80000000U)
#define CSL_IOMUX_EQEP0_A_CFG_REG_HSMASTER_SHIFT                               (0x0000001FU)
#define CSL_IOMUX_EQEP0_A_CFG_REG_HSMASTER_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_EQEP0_A_CFG_REG_HSMASTER_MAX                                 (0x00000001U)

#define CSL_IOMUX_EQEP0_A_CFG_REG_RESETVAL                                     (0x000005F7U)

/* EQEP0_B_CFG_REG */

#define CSL_IOMUX_EQEP0_B_CFG_REG_FUNC_SEL_MASK                                (0x0000000FU)
#define CSL_IOMUX_EQEP0_B_CFG_REG_FUNC_SEL_SHIFT                               (0x00000000U)
#define CSL_IOMUX_EQEP0_B_CFG_REG_FUNC_SEL_RESETVAL                            (0x00000007U)
#define CSL_IOMUX_EQEP0_B_CFG_REG_FUNC_SEL_MAX                                 (0x0000000FU)

#define CSL_IOMUX_EQEP0_B_CFG_REG_IE_OVERRIDE_CTRL_MASK                        (0x00000010U)
#define CSL_IOMUX_EQEP0_B_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                       (0x00000004U)
#define CSL_IOMUX_EQEP0_B_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_EQEP0_B_CFG_REG_IE_OVERRIDE_CTRL_MAX                         (0x00000001U)

#define CSL_IOMUX_EQEP0_B_CFG_REG_IE_OVERRIDE_MASK                             (0x00000020U)
#define CSL_IOMUX_EQEP0_B_CFG_REG_IE_OVERRIDE_SHIFT                            (0x00000005U)
#define CSL_IOMUX_EQEP0_B_CFG_REG_IE_OVERRIDE_RESETVAL                         (0x00000001U)
#define CSL_IOMUX_EQEP0_B_CFG_REG_IE_OVERRIDE_MAX                              (0x00000001U)

#define CSL_IOMUX_EQEP0_B_CFG_REG_OE_OVERRIDE_CTRL_MASK                        (0x00000040U)
#define CSL_IOMUX_EQEP0_B_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                       (0x00000006U)
#define CSL_IOMUX_EQEP0_B_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_EQEP0_B_CFG_REG_OE_OVERRIDE_CTRL_MAX                         (0x00000001U)

#define CSL_IOMUX_EQEP0_B_CFG_REG_OE_OVERRIDE_MASK                             (0x00000080U)
#define CSL_IOMUX_EQEP0_B_CFG_REG_OE_OVERRIDE_SHIFT                            (0x00000007U)
#define CSL_IOMUX_EQEP0_B_CFG_REG_OE_OVERRIDE_RESETVAL                         (0x00000001U)
#define CSL_IOMUX_EQEP0_B_CFG_REG_OE_OVERRIDE_MAX                              (0x00000001U)

#define CSL_IOMUX_EQEP0_B_CFG_REG_PI_MASK                                      (0x00000100U)
#define CSL_IOMUX_EQEP0_B_CFG_REG_PI_SHIFT                                     (0x00000008U)
#define CSL_IOMUX_EQEP0_B_CFG_REG_PI_RESETVAL                                  (0x00000001U)
#define CSL_IOMUX_EQEP0_B_CFG_REG_PI_MAX                                       (0x00000001U)

#define CSL_IOMUX_EQEP0_B_CFG_REG_PUPDSEL_MASK                                 (0x00000200U)
#define CSL_IOMUX_EQEP0_B_CFG_REG_PUPDSEL_SHIFT                                (0x00000009U)
#define CSL_IOMUX_EQEP0_B_CFG_REG_PUPDSEL_RESETVAL                             (0x00000000U)
#define CSL_IOMUX_EQEP0_B_CFG_REG_PUPDSEL_MAX                                  (0x00000001U)

#define CSL_IOMUX_EQEP0_B_CFG_REG_SC1_MASK                                     (0x00000400U)
#define CSL_IOMUX_EQEP0_B_CFG_REG_SC1_SHIFT                                    (0x0000000AU)
#define CSL_IOMUX_EQEP0_B_CFG_REG_SC1_RESETVAL                                 (0x00000001U)
#define CSL_IOMUX_EQEP0_B_CFG_REG_SC1_MAX                                      (0x00000001U)

#define CSL_IOMUX_EQEP0_B_CFG_REG_GPIO_SEL_MASK                                (0x00030000U)
#define CSL_IOMUX_EQEP0_B_CFG_REG_GPIO_SEL_SHIFT                               (0x00000010U)
#define CSL_IOMUX_EQEP0_B_CFG_REG_GPIO_SEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_EQEP0_B_CFG_REG_GPIO_SEL_MAX                                 (0x00000003U)

#define CSL_IOMUX_EQEP0_B_CFG_REG_QUAL_SEL_MASK                                (0x000C0000U)
#define CSL_IOMUX_EQEP0_B_CFG_REG_QUAL_SEL_SHIFT                               (0x00000012U)
#define CSL_IOMUX_EQEP0_B_CFG_REG_QUAL_SEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_EQEP0_B_CFG_REG_QUAL_SEL_MAX                                 (0x00000003U)

#define CSL_IOMUX_EQEP0_B_CFG_REG_INP_INV_SEL_MASK                             (0x00100000U)
#define CSL_IOMUX_EQEP0_B_CFG_REG_INP_INV_SEL_SHIFT                            (0x00000014U)
#define CSL_IOMUX_EQEP0_B_CFG_REG_INP_INV_SEL_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_EQEP0_B_CFG_REG_INP_INV_SEL_MAX                              (0x00000001U)

#define CSL_IOMUX_EQEP0_B_CFG_REG_ICSSM_GPIO_SEL_MASK                          (0x00200000U)
#define CSL_IOMUX_EQEP0_B_CFG_REG_ICSSM_GPIO_SEL_SHIFT                         (0x00000015U)
#define CSL_IOMUX_EQEP0_B_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_EQEP0_B_CFG_REG_ICSSM_GPIO_SEL_MAX                           (0x00000001U)

#define CSL_IOMUX_EQEP0_B_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                     (0x00400000U)
#define CSL_IOMUX_EQEP0_B_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                    (0x00000016U)
#define CSL_IOMUX_EQEP0_B_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL                 (0x00000000U)
#define CSL_IOMUX_EQEP0_B_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                      (0x00000001U)

#define CSL_IOMUX_EQEP0_B_CFG_REG_HSMODE_MASK                                  (0x40000000U)
#define CSL_IOMUX_EQEP0_B_CFG_REG_HSMODE_SHIFT                                 (0x0000001EU)
#define CSL_IOMUX_EQEP0_B_CFG_REG_HSMODE_RESETVAL                              (0x00000000U)
#define CSL_IOMUX_EQEP0_B_CFG_REG_HSMODE_MAX                                   (0x00000001U)

#define CSL_IOMUX_EQEP0_B_CFG_REG_HSMASTER_MASK                                (0x80000000U)
#define CSL_IOMUX_EQEP0_B_CFG_REG_HSMASTER_SHIFT                               (0x0000001FU)
#define CSL_IOMUX_EQEP0_B_CFG_REG_HSMASTER_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_EQEP0_B_CFG_REG_HSMASTER_MAX                                 (0x00000001U)

#define CSL_IOMUX_EQEP0_B_CFG_REG_RESETVAL                                     (0x000005F7U)

/* EQEP0_STROBE_CFG_REG */

#define CSL_IOMUX_EQEP0_STROBE_CFG_REG_FUNC_SEL_MASK                           (0x0000000FU)
#define CSL_IOMUX_EQEP0_STROBE_CFG_REG_FUNC_SEL_SHIFT                          (0x00000000U)
#define CSL_IOMUX_EQEP0_STROBE_CFG_REG_FUNC_SEL_RESETVAL                       (0x00000007U)
#define CSL_IOMUX_EQEP0_STROBE_CFG_REG_FUNC_SEL_MAX                            (0x0000000FU)

#define CSL_IOMUX_EQEP0_STROBE_CFG_REG_IE_OVERRIDE_CTRL_MASK                   (0x00000010U)
#define CSL_IOMUX_EQEP0_STROBE_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                  (0x00000004U)
#define CSL_IOMUX_EQEP0_STROBE_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL               (0x00000001U)
#define CSL_IOMUX_EQEP0_STROBE_CFG_REG_IE_OVERRIDE_CTRL_MAX                    (0x00000001U)

#define CSL_IOMUX_EQEP0_STROBE_CFG_REG_IE_OVERRIDE_MASK                        (0x00000020U)
#define CSL_IOMUX_EQEP0_STROBE_CFG_REG_IE_OVERRIDE_SHIFT                       (0x00000005U)
#define CSL_IOMUX_EQEP0_STROBE_CFG_REG_IE_OVERRIDE_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_EQEP0_STROBE_CFG_REG_IE_OVERRIDE_MAX                         (0x00000001U)

#define CSL_IOMUX_EQEP0_STROBE_CFG_REG_OE_OVERRIDE_CTRL_MASK                   (0x00000040U)
#define CSL_IOMUX_EQEP0_STROBE_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                  (0x00000006U)
#define CSL_IOMUX_EQEP0_STROBE_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL               (0x00000001U)
#define CSL_IOMUX_EQEP0_STROBE_CFG_REG_OE_OVERRIDE_CTRL_MAX                    (0x00000001U)

#define CSL_IOMUX_EQEP0_STROBE_CFG_REG_OE_OVERRIDE_MASK                        (0x00000080U)
#define CSL_IOMUX_EQEP0_STROBE_CFG_REG_OE_OVERRIDE_SHIFT                       (0x00000007U)
#define CSL_IOMUX_EQEP0_STROBE_CFG_REG_OE_OVERRIDE_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_EQEP0_STROBE_CFG_REG_OE_OVERRIDE_MAX                         (0x00000001U)

#define CSL_IOMUX_EQEP0_STROBE_CFG_REG_PI_MASK                                 (0x00000100U)
#define CSL_IOMUX_EQEP0_STROBE_CFG_REG_PI_SHIFT                                (0x00000008U)
#define CSL_IOMUX_EQEP0_STROBE_CFG_REG_PI_RESETVAL                             (0x00000001U)
#define CSL_IOMUX_EQEP0_STROBE_CFG_REG_PI_MAX                                  (0x00000001U)

#define CSL_IOMUX_EQEP0_STROBE_CFG_REG_PUPDSEL_MASK                            (0x00000200U)
#define CSL_IOMUX_EQEP0_STROBE_CFG_REG_PUPDSEL_SHIFT                           (0x00000009U)
#define CSL_IOMUX_EQEP0_STROBE_CFG_REG_PUPDSEL_RESETVAL                        (0x00000000U)
#define CSL_IOMUX_EQEP0_STROBE_CFG_REG_PUPDSEL_MAX                             (0x00000001U)

#define CSL_IOMUX_EQEP0_STROBE_CFG_REG_SC1_MASK                                (0x00000400U)
#define CSL_IOMUX_EQEP0_STROBE_CFG_REG_SC1_SHIFT                               (0x0000000AU)
#define CSL_IOMUX_EQEP0_STROBE_CFG_REG_SC1_RESETVAL                            (0x00000001U)
#define CSL_IOMUX_EQEP0_STROBE_CFG_REG_SC1_MAX                                 (0x00000001U)

#define CSL_IOMUX_EQEP0_STROBE_CFG_REG_GPIO_SEL_MASK                           (0x00030000U)
#define CSL_IOMUX_EQEP0_STROBE_CFG_REG_GPIO_SEL_SHIFT                          (0x00000010U)
#define CSL_IOMUX_EQEP0_STROBE_CFG_REG_GPIO_SEL_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_EQEP0_STROBE_CFG_REG_GPIO_SEL_MAX                            (0x00000003U)

#define CSL_IOMUX_EQEP0_STROBE_CFG_REG_QUAL_SEL_MASK                           (0x000C0000U)
#define CSL_IOMUX_EQEP0_STROBE_CFG_REG_QUAL_SEL_SHIFT                          (0x00000012U)
#define CSL_IOMUX_EQEP0_STROBE_CFG_REG_QUAL_SEL_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_EQEP0_STROBE_CFG_REG_QUAL_SEL_MAX                            (0x00000003U)

#define CSL_IOMUX_EQEP0_STROBE_CFG_REG_INP_INV_SEL_MASK                        (0x00100000U)
#define CSL_IOMUX_EQEP0_STROBE_CFG_REG_INP_INV_SEL_SHIFT                       (0x00000014U)
#define CSL_IOMUX_EQEP0_STROBE_CFG_REG_INP_INV_SEL_RESETVAL                    (0x00000000U)
#define CSL_IOMUX_EQEP0_STROBE_CFG_REG_INP_INV_SEL_MAX                         (0x00000001U)

#define CSL_IOMUX_EQEP0_STROBE_CFG_REG_ICSSM_GPIO_SEL_MASK                     (0x00200000U)
#define CSL_IOMUX_EQEP0_STROBE_CFG_REG_ICSSM_GPIO_SEL_SHIFT                    (0x00000015U)
#define CSL_IOMUX_EQEP0_STROBE_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                 (0x00000000U)
#define CSL_IOMUX_EQEP0_STROBE_CFG_REG_ICSSM_GPIO_SEL_MAX                      (0x00000001U)

#define CSL_IOMUX_EQEP0_STROBE_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                (0x00400000U)
#define CSL_IOMUX_EQEP0_STROBE_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT               (0x00000016U)
#define CSL_IOMUX_EQEP0_STROBE_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL            (0x00000000U)
#define CSL_IOMUX_EQEP0_STROBE_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                 (0x00000001U)

#define CSL_IOMUX_EQEP0_STROBE_CFG_REG_HSMODE_MASK                             (0x40000000U)
#define CSL_IOMUX_EQEP0_STROBE_CFG_REG_HSMODE_SHIFT                            (0x0000001EU)
#define CSL_IOMUX_EQEP0_STROBE_CFG_REG_HSMODE_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_EQEP0_STROBE_CFG_REG_HSMODE_MAX                              (0x00000001U)

#define CSL_IOMUX_EQEP0_STROBE_CFG_REG_HSMASTER_MASK                           (0x80000000U)
#define CSL_IOMUX_EQEP0_STROBE_CFG_REG_HSMASTER_SHIFT                          (0x0000001FU)
#define CSL_IOMUX_EQEP0_STROBE_CFG_REG_HSMASTER_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_EQEP0_STROBE_CFG_REG_HSMASTER_MAX                            (0x00000001U)

#define CSL_IOMUX_EQEP0_STROBE_CFG_REG_RESETVAL                                (0x000005F7U)

/* EQEP0_INDEX_CFG_REG */

#define CSL_IOMUX_EQEP0_INDEX_CFG_REG_FUNC_SEL_MASK                            (0x0000000FU)
#define CSL_IOMUX_EQEP0_INDEX_CFG_REG_FUNC_SEL_SHIFT                           (0x00000000U)
#define CSL_IOMUX_EQEP0_INDEX_CFG_REG_FUNC_SEL_RESETVAL                        (0x00000007U)
#define CSL_IOMUX_EQEP0_INDEX_CFG_REG_FUNC_SEL_MAX                             (0x0000000FU)

#define CSL_IOMUX_EQEP0_INDEX_CFG_REG_IE_OVERRIDE_CTRL_MASK                    (0x00000010U)
#define CSL_IOMUX_EQEP0_INDEX_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                   (0x00000004U)
#define CSL_IOMUX_EQEP0_INDEX_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                (0x00000001U)
#define CSL_IOMUX_EQEP0_INDEX_CFG_REG_IE_OVERRIDE_CTRL_MAX                     (0x00000001U)

#define CSL_IOMUX_EQEP0_INDEX_CFG_REG_IE_OVERRIDE_MASK                         (0x00000020U)
#define CSL_IOMUX_EQEP0_INDEX_CFG_REG_IE_OVERRIDE_SHIFT                        (0x00000005U)
#define CSL_IOMUX_EQEP0_INDEX_CFG_REG_IE_OVERRIDE_RESETVAL                     (0x00000001U)
#define CSL_IOMUX_EQEP0_INDEX_CFG_REG_IE_OVERRIDE_MAX                          (0x00000001U)

#define CSL_IOMUX_EQEP0_INDEX_CFG_REG_OE_OVERRIDE_CTRL_MASK                    (0x00000040U)
#define CSL_IOMUX_EQEP0_INDEX_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                   (0x00000006U)
#define CSL_IOMUX_EQEP0_INDEX_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                (0x00000001U)
#define CSL_IOMUX_EQEP0_INDEX_CFG_REG_OE_OVERRIDE_CTRL_MAX                     (0x00000001U)

#define CSL_IOMUX_EQEP0_INDEX_CFG_REG_OE_OVERRIDE_MASK                         (0x00000080U)
#define CSL_IOMUX_EQEP0_INDEX_CFG_REG_OE_OVERRIDE_SHIFT                        (0x00000007U)
#define CSL_IOMUX_EQEP0_INDEX_CFG_REG_OE_OVERRIDE_RESETVAL                     (0x00000001U)
#define CSL_IOMUX_EQEP0_INDEX_CFG_REG_OE_OVERRIDE_MAX                          (0x00000001U)

#define CSL_IOMUX_EQEP0_INDEX_CFG_REG_PI_MASK                                  (0x00000100U)
#define CSL_IOMUX_EQEP0_INDEX_CFG_REG_PI_SHIFT                                 (0x00000008U)
#define CSL_IOMUX_EQEP0_INDEX_CFG_REG_PI_RESETVAL                              (0x00000001U)
#define CSL_IOMUX_EQEP0_INDEX_CFG_REG_PI_MAX                                   (0x00000001U)

#define CSL_IOMUX_EQEP0_INDEX_CFG_REG_PUPDSEL_MASK                             (0x00000200U)
#define CSL_IOMUX_EQEP0_INDEX_CFG_REG_PUPDSEL_SHIFT                            (0x00000009U)
#define CSL_IOMUX_EQEP0_INDEX_CFG_REG_PUPDSEL_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_EQEP0_INDEX_CFG_REG_PUPDSEL_MAX                              (0x00000001U)

#define CSL_IOMUX_EQEP0_INDEX_CFG_REG_SC1_MASK                                 (0x00000400U)
#define CSL_IOMUX_EQEP0_INDEX_CFG_REG_SC1_SHIFT                                (0x0000000AU)
#define CSL_IOMUX_EQEP0_INDEX_CFG_REG_SC1_RESETVAL                             (0x00000001U)
#define CSL_IOMUX_EQEP0_INDEX_CFG_REG_SC1_MAX                                  (0x00000001U)

#define CSL_IOMUX_EQEP0_INDEX_CFG_REG_GPIO_SEL_MASK                            (0x00030000U)
#define CSL_IOMUX_EQEP0_INDEX_CFG_REG_GPIO_SEL_SHIFT                           (0x00000010U)
#define CSL_IOMUX_EQEP0_INDEX_CFG_REG_GPIO_SEL_RESETVAL                        (0x00000000U)
#define CSL_IOMUX_EQEP0_INDEX_CFG_REG_GPIO_SEL_MAX                             (0x00000003U)

#define CSL_IOMUX_EQEP0_INDEX_CFG_REG_QUAL_SEL_MASK                            (0x000C0000U)
#define CSL_IOMUX_EQEP0_INDEX_CFG_REG_QUAL_SEL_SHIFT                           (0x00000012U)
#define CSL_IOMUX_EQEP0_INDEX_CFG_REG_QUAL_SEL_RESETVAL                        (0x00000000U)
#define CSL_IOMUX_EQEP0_INDEX_CFG_REG_QUAL_SEL_MAX                             (0x00000003U)

#define CSL_IOMUX_EQEP0_INDEX_CFG_REG_INP_INV_SEL_MASK                         (0x00100000U)
#define CSL_IOMUX_EQEP0_INDEX_CFG_REG_INP_INV_SEL_SHIFT                        (0x00000014U)
#define CSL_IOMUX_EQEP0_INDEX_CFG_REG_INP_INV_SEL_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_EQEP0_INDEX_CFG_REG_INP_INV_SEL_MAX                          (0x00000001U)

#define CSL_IOMUX_EQEP0_INDEX_CFG_REG_ICSSM_GPIO_SEL_MASK                      (0x00200000U)
#define CSL_IOMUX_EQEP0_INDEX_CFG_REG_ICSSM_GPIO_SEL_SHIFT                     (0x00000015U)
#define CSL_IOMUX_EQEP0_INDEX_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                  (0x00000000U)
#define CSL_IOMUX_EQEP0_INDEX_CFG_REG_ICSSM_GPIO_SEL_MAX                       (0x00000001U)

#define CSL_IOMUX_EQEP0_INDEX_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                 (0x00400000U)
#define CSL_IOMUX_EQEP0_INDEX_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                (0x00000016U)
#define CSL_IOMUX_EQEP0_INDEX_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL             (0x00000000U)
#define CSL_IOMUX_EQEP0_INDEX_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                  (0x00000001U)

#define CSL_IOMUX_EQEP0_INDEX_CFG_REG_HSMODE_MASK                              (0x40000000U)
#define CSL_IOMUX_EQEP0_INDEX_CFG_REG_HSMODE_SHIFT                             (0x0000001EU)
#define CSL_IOMUX_EQEP0_INDEX_CFG_REG_HSMODE_RESETVAL                          (0x00000000U)
#define CSL_IOMUX_EQEP0_INDEX_CFG_REG_HSMODE_MAX                               (0x00000001U)

#define CSL_IOMUX_EQEP0_INDEX_CFG_REG_HSMASTER_MASK                            (0x80000000U)
#define CSL_IOMUX_EQEP0_INDEX_CFG_REG_HSMASTER_SHIFT                           (0x0000001FU)
#define CSL_IOMUX_EQEP0_INDEX_CFG_REG_HSMASTER_RESETVAL                        (0x00000000U)
#define CSL_IOMUX_EQEP0_INDEX_CFG_REG_HSMASTER_MAX                             (0x00000001U)

#define CSL_IOMUX_EQEP0_INDEX_CFG_REG_RESETVAL                                 (0x000005F7U)

/* I2C0_SDA_CFG_REG */

#define CSL_IOMUX_I2C0_SDA_CFG_REG_FUNC_SEL_MASK                               (0x0000000FU)
#define CSL_IOMUX_I2C0_SDA_CFG_REG_FUNC_SEL_SHIFT                              (0x00000000U)
#define CSL_IOMUX_I2C0_SDA_CFG_REG_FUNC_SEL_RESETVAL                           (0x00000007U)
#define CSL_IOMUX_I2C0_SDA_CFG_REG_FUNC_SEL_MAX                                (0x0000000FU)

#define CSL_IOMUX_I2C0_SDA_CFG_REG_IE_OVERRIDE_CTRL_MASK                       (0x00000010U)
#define CSL_IOMUX_I2C0_SDA_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                      (0x00000004U)
#define CSL_IOMUX_I2C0_SDA_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_I2C0_SDA_CFG_REG_IE_OVERRIDE_CTRL_MAX                        (0x00000001U)

#define CSL_IOMUX_I2C0_SDA_CFG_REG_IE_OVERRIDE_MASK                            (0x00000020U)
#define CSL_IOMUX_I2C0_SDA_CFG_REG_IE_OVERRIDE_SHIFT                           (0x00000005U)
#define CSL_IOMUX_I2C0_SDA_CFG_REG_IE_OVERRIDE_RESETVAL                        (0x00000001U)
#define CSL_IOMUX_I2C0_SDA_CFG_REG_IE_OVERRIDE_MAX                             (0x00000001U)

#define CSL_IOMUX_I2C0_SDA_CFG_REG_OE_OVERRIDE_CTRL_MASK                       (0x00000040U)
#define CSL_IOMUX_I2C0_SDA_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                      (0x00000006U)
#define CSL_IOMUX_I2C0_SDA_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_I2C0_SDA_CFG_REG_OE_OVERRIDE_CTRL_MAX                        (0x00000001U)

#define CSL_IOMUX_I2C0_SDA_CFG_REG_OE_OVERRIDE_MASK                            (0x00000080U)
#define CSL_IOMUX_I2C0_SDA_CFG_REG_OE_OVERRIDE_SHIFT                           (0x00000007U)
#define CSL_IOMUX_I2C0_SDA_CFG_REG_OE_OVERRIDE_RESETVAL                        (0x00000001U)
#define CSL_IOMUX_I2C0_SDA_CFG_REG_OE_OVERRIDE_MAX                             (0x00000001U)

#define CSL_IOMUX_I2C0_SDA_CFG_REG_PI_MASK                                     (0x00000100U)
#define CSL_IOMUX_I2C0_SDA_CFG_REG_PI_SHIFT                                    (0x00000008U)
#define CSL_IOMUX_I2C0_SDA_CFG_REG_PI_RESETVAL                                 (0x00000001U)
#define CSL_IOMUX_I2C0_SDA_CFG_REG_PI_MAX                                      (0x00000001U)

#define CSL_IOMUX_I2C0_SDA_CFG_REG_PUPDSEL_MASK                                (0x00000200U)
#define CSL_IOMUX_I2C0_SDA_CFG_REG_PUPDSEL_SHIFT                               (0x00000009U)
#define CSL_IOMUX_I2C0_SDA_CFG_REG_PUPDSEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_I2C0_SDA_CFG_REG_PUPDSEL_MAX                                 (0x00000001U)

#define CSL_IOMUX_I2C0_SDA_CFG_REG_SC1_MASK                                    (0x00000400U)
#define CSL_IOMUX_I2C0_SDA_CFG_REG_SC1_SHIFT                                   (0x0000000AU)
#define CSL_IOMUX_I2C0_SDA_CFG_REG_SC1_RESETVAL                                (0x00000001U)
#define CSL_IOMUX_I2C0_SDA_CFG_REG_SC1_MAX                                     (0x00000001U)

#define CSL_IOMUX_I2C0_SDA_CFG_REG_GPIO_SEL_MASK                               (0x00030000U)
#define CSL_IOMUX_I2C0_SDA_CFG_REG_GPIO_SEL_SHIFT                              (0x00000010U)
#define CSL_IOMUX_I2C0_SDA_CFG_REG_GPIO_SEL_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_I2C0_SDA_CFG_REG_GPIO_SEL_MAX                                (0x00000003U)

#define CSL_IOMUX_I2C0_SDA_CFG_REG_QUAL_SEL_MASK                               (0x000C0000U)
#define CSL_IOMUX_I2C0_SDA_CFG_REG_QUAL_SEL_SHIFT                              (0x00000012U)
#define CSL_IOMUX_I2C0_SDA_CFG_REG_QUAL_SEL_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_I2C0_SDA_CFG_REG_QUAL_SEL_MAX                                (0x00000003U)

#define CSL_IOMUX_I2C0_SDA_CFG_REG_INP_INV_SEL_MASK                            (0x00100000U)
#define CSL_IOMUX_I2C0_SDA_CFG_REG_INP_INV_SEL_SHIFT                           (0x00000014U)
#define CSL_IOMUX_I2C0_SDA_CFG_REG_INP_INV_SEL_RESETVAL                        (0x00000000U)
#define CSL_IOMUX_I2C0_SDA_CFG_REG_INP_INV_SEL_MAX                             (0x00000001U)

#define CSL_IOMUX_I2C0_SDA_CFG_REG_ICSSM_GPIO_SEL_MASK                         (0x00200000U)
#define CSL_IOMUX_I2C0_SDA_CFG_REG_ICSSM_GPIO_SEL_SHIFT                        (0x00000015U)
#define CSL_IOMUX_I2C0_SDA_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_I2C0_SDA_CFG_REG_ICSSM_GPIO_SEL_MAX                          (0x00000001U)

#define CSL_IOMUX_I2C0_SDA_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                    (0x00400000U)
#define CSL_IOMUX_I2C0_SDA_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                   (0x00000016U)
#define CSL_IOMUX_I2C0_SDA_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL                (0x00000000U)
#define CSL_IOMUX_I2C0_SDA_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                     (0x00000001U)

#define CSL_IOMUX_I2C0_SDA_CFG_REG_HSMODE_MASK                                 (0x40000000U)
#define CSL_IOMUX_I2C0_SDA_CFG_REG_HSMODE_SHIFT                                (0x0000001EU)
#define CSL_IOMUX_I2C0_SDA_CFG_REG_HSMODE_RESETVAL                             (0x00000000U)
#define CSL_IOMUX_I2C0_SDA_CFG_REG_HSMODE_MAX                                  (0x00000001U)

#define CSL_IOMUX_I2C0_SDA_CFG_REG_HSMASTER_MASK                               (0x80000000U)
#define CSL_IOMUX_I2C0_SDA_CFG_REG_HSMASTER_SHIFT                              (0x0000001FU)
#define CSL_IOMUX_I2C0_SDA_CFG_REG_HSMASTER_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_I2C0_SDA_CFG_REG_HSMASTER_MAX                                (0x00000001U)

#define CSL_IOMUX_I2C0_SDA_CFG_REG_RESETVAL                                    (0x000005F7U)

/* I2C0_SCL_CFG_REG */

#define CSL_IOMUX_I2C0_SCL_CFG_REG_FUNC_SEL_MASK                               (0x0000000FU)
#define CSL_IOMUX_I2C0_SCL_CFG_REG_FUNC_SEL_SHIFT                              (0x00000000U)
#define CSL_IOMUX_I2C0_SCL_CFG_REG_FUNC_SEL_RESETVAL                           (0x00000007U)
#define CSL_IOMUX_I2C0_SCL_CFG_REG_FUNC_SEL_MAX                                (0x0000000FU)

#define CSL_IOMUX_I2C0_SCL_CFG_REG_IE_OVERRIDE_CTRL_MASK                       (0x00000010U)
#define CSL_IOMUX_I2C0_SCL_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                      (0x00000004U)
#define CSL_IOMUX_I2C0_SCL_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_I2C0_SCL_CFG_REG_IE_OVERRIDE_CTRL_MAX                        (0x00000001U)

#define CSL_IOMUX_I2C0_SCL_CFG_REG_IE_OVERRIDE_MASK                            (0x00000020U)
#define CSL_IOMUX_I2C0_SCL_CFG_REG_IE_OVERRIDE_SHIFT                           (0x00000005U)
#define CSL_IOMUX_I2C0_SCL_CFG_REG_IE_OVERRIDE_RESETVAL                        (0x00000001U)
#define CSL_IOMUX_I2C0_SCL_CFG_REG_IE_OVERRIDE_MAX                             (0x00000001U)

#define CSL_IOMUX_I2C0_SCL_CFG_REG_OE_OVERRIDE_CTRL_MASK                       (0x00000040U)
#define CSL_IOMUX_I2C0_SCL_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                      (0x00000006U)
#define CSL_IOMUX_I2C0_SCL_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                   (0x00000001U)
#define CSL_IOMUX_I2C0_SCL_CFG_REG_OE_OVERRIDE_CTRL_MAX                        (0x00000001U)

#define CSL_IOMUX_I2C0_SCL_CFG_REG_OE_OVERRIDE_MASK                            (0x00000080U)
#define CSL_IOMUX_I2C0_SCL_CFG_REG_OE_OVERRIDE_SHIFT                           (0x00000007U)
#define CSL_IOMUX_I2C0_SCL_CFG_REG_OE_OVERRIDE_RESETVAL                        (0x00000001U)
#define CSL_IOMUX_I2C0_SCL_CFG_REG_OE_OVERRIDE_MAX                             (0x00000001U)

#define CSL_IOMUX_I2C0_SCL_CFG_REG_PI_MASK                                     (0x00000100U)
#define CSL_IOMUX_I2C0_SCL_CFG_REG_PI_SHIFT                                    (0x00000008U)
#define CSL_IOMUX_I2C0_SCL_CFG_REG_PI_RESETVAL                                 (0x00000001U)
#define CSL_IOMUX_I2C0_SCL_CFG_REG_PI_MAX                                      (0x00000001U)

#define CSL_IOMUX_I2C0_SCL_CFG_REG_PUPDSEL_MASK                                (0x00000200U)
#define CSL_IOMUX_I2C0_SCL_CFG_REG_PUPDSEL_SHIFT                               (0x00000009U)
#define CSL_IOMUX_I2C0_SCL_CFG_REG_PUPDSEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_I2C0_SCL_CFG_REG_PUPDSEL_MAX                                 (0x00000001U)

#define CSL_IOMUX_I2C0_SCL_CFG_REG_SC1_MASK                                    (0x00000400U)
#define CSL_IOMUX_I2C0_SCL_CFG_REG_SC1_SHIFT                                   (0x0000000AU)
#define CSL_IOMUX_I2C0_SCL_CFG_REG_SC1_RESETVAL                                (0x00000001U)
#define CSL_IOMUX_I2C0_SCL_CFG_REG_SC1_MAX                                     (0x00000001U)

#define CSL_IOMUX_I2C0_SCL_CFG_REG_GPIO_SEL_MASK                               (0x00030000U)
#define CSL_IOMUX_I2C0_SCL_CFG_REG_GPIO_SEL_SHIFT                              (0x00000010U)
#define CSL_IOMUX_I2C0_SCL_CFG_REG_GPIO_SEL_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_I2C0_SCL_CFG_REG_GPIO_SEL_MAX                                (0x00000003U)

#define CSL_IOMUX_I2C0_SCL_CFG_REG_QUAL_SEL_MASK                               (0x000C0000U)
#define CSL_IOMUX_I2C0_SCL_CFG_REG_QUAL_SEL_SHIFT                              (0x00000012U)
#define CSL_IOMUX_I2C0_SCL_CFG_REG_QUAL_SEL_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_I2C0_SCL_CFG_REG_QUAL_SEL_MAX                                (0x00000003U)

#define CSL_IOMUX_I2C0_SCL_CFG_REG_INP_INV_SEL_MASK                            (0x00100000U)
#define CSL_IOMUX_I2C0_SCL_CFG_REG_INP_INV_SEL_SHIFT                           (0x00000014U)
#define CSL_IOMUX_I2C0_SCL_CFG_REG_INP_INV_SEL_RESETVAL                        (0x00000000U)
#define CSL_IOMUX_I2C0_SCL_CFG_REG_INP_INV_SEL_MAX                             (0x00000001U)

#define CSL_IOMUX_I2C0_SCL_CFG_REG_ICSSM_GPIO_SEL_MASK                         (0x00200000U)
#define CSL_IOMUX_I2C0_SCL_CFG_REG_ICSSM_GPIO_SEL_SHIFT                        (0x00000015U)
#define CSL_IOMUX_I2C0_SCL_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_I2C0_SCL_CFG_REG_ICSSM_GPIO_SEL_MAX                          (0x00000001U)

#define CSL_IOMUX_I2C0_SCL_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                    (0x00400000U)
#define CSL_IOMUX_I2C0_SCL_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                   (0x00000016U)
#define CSL_IOMUX_I2C0_SCL_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL                (0x00000000U)
#define CSL_IOMUX_I2C0_SCL_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                     (0x00000001U)

#define CSL_IOMUX_I2C0_SCL_CFG_REG_HSMODE_MASK                                 (0x40000000U)
#define CSL_IOMUX_I2C0_SCL_CFG_REG_HSMODE_SHIFT                                (0x0000001EU)
#define CSL_IOMUX_I2C0_SCL_CFG_REG_HSMODE_RESETVAL                             (0x00000000U)
#define CSL_IOMUX_I2C0_SCL_CFG_REG_HSMODE_MAX                                  (0x00000001U)

#define CSL_IOMUX_I2C0_SCL_CFG_REG_HSMASTER_MASK                               (0x80000000U)
#define CSL_IOMUX_I2C0_SCL_CFG_REG_HSMASTER_SHIFT                              (0x0000001FU)
#define CSL_IOMUX_I2C0_SCL_CFG_REG_HSMASTER_RESETVAL                           (0x00000000U)
#define CSL_IOMUX_I2C0_SCL_CFG_REG_HSMASTER_MAX                                (0x00000001U)

#define CSL_IOMUX_I2C0_SCL_CFG_REG_RESETVAL                                    (0x000005F7U)

/* GPIO136_CFG_REG */
#define CSL_IOMUX_GPIO136_CFG_REG_FUNC_SEL_MASK                                 (0x0000000FU)
#define CSL_IOMUX_GPIO136_CFG_REG_FUNC_SEL_SHIFT                                (0x00000000U)
#define CSL_IOMUX_GPIO136_CFG_REG_FUNC_SEL_RESETVAL                             (0x00000007U)
#define CSL_IOMUX_GPIO136_CFG_REG_FUNC_SEL_MAX                                  (0x0000000FU)


#define CSL_IOMUX_GPIO136_CFG_REG_GPIO_SEL_MASK                                 (0x00010000U)
#define CSL_IOMUX_GPIO136_CFG_REG_GPIO_SEL_SHIFT                                (0x00000010U)
#define CSL_IOMUX_GPIO136_CFG_REG_GPIO_SEL_RESETVAL                             (0x00000000U)
#define CSL_IOMUX_GPIO136_CFG_REG_GPIO_SEL_MAX                                  (0x00000001U)


#define CSL_IOMUX_GPIO136_CFG_REG_HSMASTER_MASK                                 (0x80000000U)
#define CSL_IOMUX_GPIO136_CFG_REG_HSMASTER_SHIFT                                (0x0000001FU)
#define CSL_IOMUX_GPIO136_CFG_REG_HSMASTER_RESETVAL                             (0x00000000U)
#define CSL_IOMUX_GPIO136_CFG_REG_HSMASTER_MAX                                  (0x00000001U)


#define CSL_IOMUX_GPIO136_CFG_REG_HSMODE_MASK                                 (0x40000000U)
#define CSL_IOMUX_GPIO136_CFG_REG_HSMODE_SHIFT                                (0x0000001EU)
#define CSL_IOMUX_GPIO136_CFG_REG_HSMODE_RESETVAL                             (0x00000000U)
#define CSL_IOMUX_GPIO136_CFG_REG_HSMODE_MAX                                  (0x00000001U)


#define CSL_IOMUX_GPIO136_CFG_REG_HVMODE_STATUS_MASK                            (0x01000000U)
#define CSL_IOMUX_GPIO136_CFG_REG_HVMODE_STATUS_SHIFT                           (0x00000018U)
#define CSL_IOMUX_GPIO136_CFG_REG_HVMODE_STATUS_RESETVAL                        (0x00000000U)
#define CSL_IOMUX_GPIO136_CFG_REG_HVMODE_STATUS_MAX                             (0x00000001U)


#define CSL_IOMUX_GPIO136_CFG_REG_ICSSM_GPIO_SEL_MASK                           (0x00200000U)
#define CSL_IOMUX_GPIO136_CFG_REG_ICSSM_GPIO_SEL_SHIFT                          (0x00000015U)
#define CSL_IOMUX_GPIO136_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_GPIO136_CFG_REG_ICSSM_GPIO_SEL_MAX                            (0x00000001U)


#define CSL_IOMUX_GPIO136_CFG_REG_IE_OVERRIDE_MASK                              (0x00000020U)
#define CSL_IOMUX_GPIO136_CFG_REG_IE_OVERRIDE_SHIFT                             (0x00000005U)
#define CSL_IOMUX_GPIO136_CFG_REG_IE_OVERRIDE_RESETVAL                          (0x00000001U)
#define CSL_IOMUX_GPIO136_CFG_REG_IE_OVERRIDE_MAX                               (0x00000001U)


#define CSL_IOMUX_GPIO136_CFG_REG_IE_OVERRIDE_CTRL_MASK                         (0x00000010U)
#define CSL_IOMUX_GPIO136_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                        (0x00000004U)
#define CSL_IOMUX_GPIO136_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                     (0x00000001U)
#define CSL_IOMUX_GPIO136_CFG_REG_IE_OVERRIDE_CTRL_MAX                          (0x00000001U)


#define CSL_IOMUX_GPIO136_CFG_REG_INP_INV_SEL_MASK                              (0x00100000U)
#define CSL_IOMUX_GPIO136_CFG_REG_INP_INV_SEL_SHIFT                             (0x00000014U)
#define CSL_IOMUX_GPIO136_CFG_REG_INP_INV_SEL_RESETVAL                          (0x00000000U)
#define CSL_IOMUX_GPIO136_CFG_REG_INP_INV_SEL_MAX                               (0x00000001U)


#define CSL_IOMUX_GPIO136_CFG_REG_OE_OVERRIDE_MASK                              (0x00000080U)
#define CSL_IOMUX_GPIO136_CFG_REG_OE_OVERRIDE_SHIFT                             (0x00000007U)
#define CSL_IOMUX_GPIO136_CFG_REG_OE_OVERRIDE_RESETVAL                          (0x00000001U)
#define CSL_IOMUX_GPIO136_CFG_REG_OE_OVERRIDE_MAX                               (0x00000001U)


#define CSL_IOMUX_GPIO136_CFG_REG_OE_OVERRIDE_CTRL_MASK                         (0x00000040U)
#define CSL_IOMUX_GPIO136_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                        (0x00000006U)
#define CSL_IOMUX_GPIO136_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                     (0x00000001U)
#define CSL_IOMUX_GPIO136_CFG_REG_OE_OVERRIDE_CTRL_MAX                          (0x00000001U)


#define CSL_IOMUX_GPIO136_CFG_REG_PE_MASK                                       (0x00000100U)
#define CSL_IOMUX_GPIO136_CFG_REG_PE_SHIFT                                      (0x00000008U)
#define CSL_IOMUX_GPIO136_CFG_REG_PE_RESETVAL                                   (0x00000001U)
#define CSL_IOMUX_GPIO136_CFG_REG_PE_MAX                                        (0x00000001U)


#define CSL_IOMUX_GPIO136_CFG_REG_PUPDSEL_MASK                                  (0x00000200U)
#define CSL_IOMUX_GPIO136_CFG_REG_PUPDSEL_SHIFT                                 (0x00000009U)
#define CSL_IOMUX_GPIO136_CFG_REG_PUPDSEL_RESETVAL                              (0x00000000U)
#define CSL_IOMUX_GPIO136_CFG_REG_PUPDSEL_MAX                                   (0x00000001U)


#define CSL_IOMUX_GPIO136_CFG_REG_QUAL_SEL_MASK                                 (0x000C0000U)
#define CSL_IOMUX_GPIO136_CFG_REG_QUAL_SEL_SHIFT                                (0x00000012U)
#define CSL_IOMUX_GPIO136_CFG_REG_QUAL_SEL_RESETVAL                             (0x00000000U)
#define CSL_IOMUX_GPIO136_CFG_REG_QUAL_SEL_MAX                                  (0x00000003U)


#define CSL_IOMUX_GPIO136_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                      (0x00400000U)
#define CSL_IOMUX_GPIO136_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                     (0x00000016U)
#define CSL_IOMUX_GPIO136_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL                  (0x00000000U)
#define CSL_IOMUX_GPIO136_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                       (0x00000001U)


#define CSL_IOMUX_GPIO136_CFG_REG_SC1_MASK                                      (0x00000400U)
#define CSL_IOMUX_GPIO136_CFG_REG_SC1_SHIFT                                     (0x0000000AU)
#define CSL_IOMUX_GPIO136_CFG_REG_SC1_RESETVAL                                  (0x00000001U)
#define CSL_IOMUX_GPIO136_CFG_REG_SC1_MAX                                       (0x00000001U)



/* GPIO137_CFG_REG */
#define CSL_IOMUX_GPIO137_CFG_REG_FUNC_SEL_MASK                                 (0x0000000FU)
#define CSL_IOMUX_GPIO137_CFG_REG_FUNC_SEL_SHIFT                                (0x00000000U)
#define CSL_IOMUX_GPIO137_CFG_REG_FUNC_SEL_RESETVAL                             (0x00000007U)
#define CSL_IOMUX_GPIO137_CFG_REG_FUNC_SEL_MAX                                  (0x0000000FU)


#define CSL_IOMUX_GPIO137_CFG_REG_GPIO_SEL_MASK                                 (0x00010000U)
#define CSL_IOMUX_GPIO137_CFG_REG_GPIO_SEL_SHIFT                                (0x00000010U)
#define CSL_IOMUX_GPIO137_CFG_REG_GPIO_SEL_RESETVAL                             (0x00000000U)
#define CSL_IOMUX_GPIO137_CFG_REG_GPIO_SEL_MAX                                  (0x00000001U)


#define CSL_IOMUX_GPIO137_CFG_REG_HSMASTER_MASK                                 (0x80000000U)
#define CSL_IOMUX_GPIO137_CFG_REG_HSMASTER_SHIFT                                (0x0000001FU)
#define CSL_IOMUX_GPIO137_CFG_REG_HSMASTER_RESETVAL                             (0x00000000U)
#define CSL_IOMUX_GPIO137_CFG_REG_HSMASTER_MAX                                  (0x00000001U)


#define CSL_IOMUX_GPIO137_CFG_REG_HSMODE_MASK                                 (0x40000000U)
#define CSL_IOMUX_GPIO137_CFG_REG_HSMODE_SHIFT                                (0x0000001EU)
#define CSL_IOMUX_GPIO137_CFG_REG_HSMODE_RESETVAL                             (0x00000000U)
#define CSL_IOMUX_GPIO137_CFG_REG_HSMODE_MAX                                  (0x00000001U)


#define CSL_IOMUX_GPIO137_CFG_REG_HVMODE_STATUS_MASK                            (0x01000000U)
#define CSL_IOMUX_GPIO137_CFG_REG_HVMODE_STATUS_SHIFT                           (0x00000018U)
#define CSL_IOMUX_GPIO137_CFG_REG_HVMODE_STATUS_RESETVAL                        (0x00000000U)
#define CSL_IOMUX_GPIO137_CFG_REG_HVMODE_STATUS_MAX                             (0x00000001U)


#define CSL_IOMUX_GPIO137_CFG_REG_ICSSM_GPIO_SEL_MASK                           (0x00200000U)
#define CSL_IOMUX_GPIO137_CFG_REG_ICSSM_GPIO_SEL_SHIFT                          (0x00000015U)
#define CSL_IOMUX_GPIO137_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_GPIO137_CFG_REG_ICSSM_GPIO_SEL_MAX                            (0x00000001U)


#define CSL_IOMUX_GPIO137_CFG_REG_IE_OVERRIDE_MASK                              (0x00000020U)
#define CSL_IOMUX_GPIO137_CFG_REG_IE_OVERRIDE_SHIFT                             (0x00000005U)
#define CSL_IOMUX_GPIO137_CFG_REG_IE_OVERRIDE_RESETVAL                          (0x00000001U)
#define CSL_IOMUX_GPIO137_CFG_REG_IE_OVERRIDE_MAX                               (0x00000001U)


#define CSL_IOMUX_GPIO137_CFG_REG_IE_OVERRIDE_CTRL_MASK                         (0x00000010U)
#define CSL_IOMUX_GPIO137_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                        (0x00000004U)
#define CSL_IOMUX_GPIO137_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                     (0x00000001U)
#define CSL_IOMUX_GPIO137_CFG_REG_IE_OVERRIDE_CTRL_MAX                          (0x00000001U)


#define CSL_IOMUX_GPIO137_CFG_REG_INP_INV_SEL_MASK                              (0x00100000U)
#define CSL_IOMUX_GPIO137_CFG_REG_INP_INV_SEL_SHIFT                             (0x00000014U)
#define CSL_IOMUX_GPIO137_CFG_REG_INP_INV_SEL_RESETVAL                          (0x00000000U)
#define CSL_IOMUX_GPIO137_CFG_REG_INP_INV_SEL_MAX                               (0x00000001U)


#define CSL_IOMUX_GPIO137_CFG_REG_OE_OVERRIDE_MASK                              (0x00000080U)
#define CSL_IOMUX_GPIO137_CFG_REG_OE_OVERRIDE_SHIFT                             (0x00000007U)
#define CSL_IOMUX_GPIO137_CFG_REG_OE_OVERRIDE_RESETVAL                          (0x00000001U)
#define CSL_IOMUX_GPIO137_CFG_REG_OE_OVERRIDE_MAX                               (0x00000001U)


#define CSL_IOMUX_GPIO137_CFG_REG_OE_OVERRIDE_CTRL_MASK                         (0x00000040U)
#define CSL_IOMUX_GPIO137_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                        (0x00000006U)
#define CSL_IOMUX_GPIO137_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                     (0x00000001U)
#define CSL_IOMUX_GPIO137_CFG_REG_OE_OVERRIDE_CTRL_MAX                          (0x00000001U)


#define CSL_IOMUX_GPIO137_CFG_REG_PE_MASK                                       (0x00000100U)
#define CSL_IOMUX_GPIO137_CFG_REG_PE_SHIFT                                      (0x00000008U)
#define CSL_IOMUX_GPIO137_CFG_REG_PE_RESETVAL                                   (0x00000001U)
#define CSL_IOMUX_GPIO137_CFG_REG_PE_MAX                                        (0x00000001U)


#define CSL_IOMUX_GPIO137_CFG_REG_PUPDSEL_MASK                                  (0x00000200U)
#define CSL_IOMUX_GPIO137_CFG_REG_PUPDSEL_SHIFT                                 (0x00000009U)
#define CSL_IOMUX_GPIO137_CFG_REG_PUPDSEL_RESETVAL                              (0x00000000U)
#define CSL_IOMUX_GPIO137_CFG_REG_PUPDSEL_MAX                                   (0x00000001U)


#define CSL_IOMUX_GPIO137_CFG_REG_QUAL_SEL_MASK                                 (0x000C0000U)
#define CSL_IOMUX_GPIO137_CFG_REG_QUAL_SEL_SHIFT                                (0x00000012U)
#define CSL_IOMUX_GPIO137_CFG_REG_QUAL_SEL_RESETVAL                             (0x00000000U)
#define CSL_IOMUX_GPIO137_CFG_REG_QUAL_SEL_MAX                                  (0x00000003U)


#define CSL_IOMUX_GPIO137_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                      (0x00400000U)
#define CSL_IOMUX_GPIO137_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                     (0x00000016U)
#define CSL_IOMUX_GPIO137_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL                  (0x00000000U)
#define CSL_IOMUX_GPIO137_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                       (0x00000001U)


#define CSL_IOMUX_GPIO137_CFG_REG_SC1_MASK                                      (0x00000400U)
#define CSL_IOMUX_GPIO137_CFG_REG_SC1_SHIFT                                     (0x0000000AU)
#define CSL_IOMUX_GPIO137_CFG_REG_SC1_RESETVAL                                  (0x00000001U)
#define CSL_IOMUX_GPIO137_CFG_REG_SC1_MAX                                       (0x00000001U)

/* CLKOUT0_CFG_REG */

#define CSL_IOMUX_CLKOUT0_CFG_REG_FUNC_SEL_MASK                                (0x0000000FU)
#define CSL_IOMUX_CLKOUT0_CFG_REG_FUNC_SEL_SHIFT                               (0x00000000U)
#define CSL_IOMUX_CLKOUT0_CFG_REG_FUNC_SEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_CLKOUT0_CFG_REG_FUNC_SEL_MAX                                 (0x0000000FU)

#define CSL_IOMUX_CLKOUT0_CFG_REG_IE_OVERRIDE_CTRL_MASK                        (0x00000010U)
#define CSL_IOMUX_CLKOUT0_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                       (0x00000004U)
#define CSL_IOMUX_CLKOUT0_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_CLKOUT0_CFG_REG_IE_OVERRIDE_CTRL_MAX                         (0x00000001U)

#define CSL_IOMUX_CLKOUT0_CFG_REG_IE_OVERRIDE_MASK                             (0x00000020U)
#define CSL_IOMUX_CLKOUT0_CFG_REG_IE_OVERRIDE_SHIFT                            (0x00000005U)
#define CSL_IOMUX_CLKOUT0_CFG_REG_IE_OVERRIDE_RESETVAL                         (0x00000001U)
#define CSL_IOMUX_CLKOUT0_CFG_REG_IE_OVERRIDE_MAX                              (0x00000001U)

#define CSL_IOMUX_CLKOUT0_CFG_REG_OE_OVERRIDE_CTRL_MASK                        (0x00000040U)
#define CSL_IOMUX_CLKOUT0_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                       (0x00000006U)
#define CSL_IOMUX_CLKOUT0_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_CLKOUT0_CFG_REG_OE_OVERRIDE_CTRL_MAX                         (0x00000001U)

#define CSL_IOMUX_CLKOUT0_CFG_REG_OE_OVERRIDE_MASK                             (0x00000080U)
#define CSL_IOMUX_CLKOUT0_CFG_REG_OE_OVERRIDE_SHIFT                            (0x00000007U)
#define CSL_IOMUX_CLKOUT0_CFG_REG_OE_OVERRIDE_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_CLKOUT0_CFG_REG_OE_OVERRIDE_MAX                              (0x00000001U)

#define CSL_IOMUX_CLKOUT0_CFG_REG_PI_MASK                                      (0x00000100U)
#define CSL_IOMUX_CLKOUT0_CFG_REG_PI_SHIFT                                     (0x00000008U)
#define CSL_IOMUX_CLKOUT0_CFG_REG_PI_RESETVAL                                  (0x00000001U)
#define CSL_IOMUX_CLKOUT0_CFG_REG_PI_MAX                                       (0x00000001U)

#define CSL_IOMUX_CLKOUT0_CFG_REG_PUPDSEL_MASK                                 (0x00000200U)
#define CSL_IOMUX_CLKOUT0_CFG_REG_PUPDSEL_SHIFT                                (0x00000009U)
#define CSL_IOMUX_CLKOUT0_CFG_REG_PUPDSEL_RESETVAL                             (0x00000000U)
#define CSL_IOMUX_CLKOUT0_CFG_REG_PUPDSEL_MAX                                  (0x00000001U)

#define CSL_IOMUX_CLKOUT0_CFG_REG_SC1_MASK                                     (0x00000400U)
#define CSL_IOMUX_CLKOUT0_CFG_REG_SC1_SHIFT                                    (0x0000000AU)
#define CSL_IOMUX_CLKOUT0_CFG_REG_SC1_RESETVAL                                 (0x00000001U)
#define CSL_IOMUX_CLKOUT0_CFG_REG_SC1_MAX                                      (0x00000001U)

#define CSL_IOMUX_CLKOUT0_CFG_REG_GPIO_SEL_MASK                                (0x00030000U)
#define CSL_IOMUX_CLKOUT0_CFG_REG_GPIO_SEL_SHIFT                               (0x00000010U)
#define CSL_IOMUX_CLKOUT0_CFG_REG_GPIO_SEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_CLKOUT0_CFG_REG_GPIO_SEL_MAX                                 (0x00000003U)

#define CSL_IOMUX_CLKOUT0_CFG_REG_QUAL_SEL_MASK                                (0x000C0000U)
#define CSL_IOMUX_CLKOUT0_CFG_REG_QUAL_SEL_SHIFT                               (0x00000012U)
#define CSL_IOMUX_CLKOUT0_CFG_REG_QUAL_SEL_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_CLKOUT0_CFG_REG_QUAL_SEL_MAX                                 (0x00000003U)

#define CSL_IOMUX_CLKOUT0_CFG_REG_INP_INV_SEL_MASK                             (0x00100000U)
#define CSL_IOMUX_CLKOUT0_CFG_REG_INP_INV_SEL_SHIFT                            (0x00000014U)
#define CSL_IOMUX_CLKOUT0_CFG_REG_INP_INV_SEL_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_CLKOUT0_CFG_REG_INP_INV_SEL_MAX                              (0x00000001U)

#define CSL_IOMUX_CLKOUT0_CFG_REG_ICSSM_GPIO_SEL_MASK                          (0x00200000U)
#define CSL_IOMUX_CLKOUT0_CFG_REG_ICSSM_GPIO_SEL_SHIFT                         (0x00000015U)
#define CSL_IOMUX_CLKOUT0_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                      (0x00000000U)
#define CSL_IOMUX_CLKOUT0_CFG_REG_ICSSM_GPIO_SEL_MAX                           (0x00000001U)

#define CSL_IOMUX_CLKOUT0_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                     (0x00400000U)
#define CSL_IOMUX_CLKOUT0_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                    (0x00000016U)
#define CSL_IOMUX_CLKOUT0_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL                 (0x00000000U)
#define CSL_IOMUX_CLKOUT0_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                      (0x00000001U)

#define CSL_IOMUX_CLKOUT0_CFG_REG_HSMODE_MASK                                  (0x40000000U)
#define CSL_IOMUX_CLKOUT0_CFG_REG_HSMODE_SHIFT                                 (0x0000001EU)
#define CSL_IOMUX_CLKOUT0_CFG_REG_HSMODE_RESETVAL                              (0x00000000U)
#define CSL_IOMUX_CLKOUT0_CFG_REG_HSMODE_MAX                                   (0x00000001U)

#define CSL_IOMUX_CLKOUT0_CFG_REG_HSMASTER_MASK                                (0x80000000U)
#define CSL_IOMUX_CLKOUT0_CFG_REG_HSMASTER_SHIFT                               (0x0000001FU)
#define CSL_IOMUX_CLKOUT0_CFG_REG_HSMASTER_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_CLKOUT0_CFG_REG_HSMASTER_MAX                                 (0x00000001U)

#define CSL_IOMUX_CLKOUT0_CFG_REG_RESETVAL                                     (0x00000570U)

/* USB0_DP_CFG_REG */
#define CSL_IOMUX_USB0_DP_CFG_REG_FUNC_SEL_MASK                                 (0x0000000FU)
#define CSL_IOMUX_USB0_DP_CFG_REG_FUNC_SEL_SHIFT                                (0x00000000U)
#define CSL_IOMUX_USB0_DP_CFG_REG_FUNC_SEL_RESETVAL                             (0x00000000U)
#define CSL_IOMUX_USB0_DP_CFG_REG_FUNC_SEL_MAX                                  (0x0000000FU)


#define CSL_IOMUX_USB0_DP_CFG_REG_GPIO_SEL_MASK                                 (0x00010000U)
#define CSL_IOMUX_USB0_DP_CFG_REG_GPIO_SEL_SHIFT                                (0x00000010U)
#define CSL_IOMUX_USB0_DP_CFG_REG_GPIO_SEL_RESETVAL                             (0x00000000U)
#define CSL_IOMUX_USB0_DP_CFG_REG_GPIO_SEL_MAX                                  (0x00000001U)


#define CSL_IOMUX_USB0_DP_CFG_REG_ICSSM_GPIO_SEL_MASK                           (0x00200000U)
#define CSL_IOMUX_USB0_DP_CFG_REG_ICSSM_GPIO_SEL_SHIFT                          (0x00000015U)
#define CSL_IOMUX_USB0_DP_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_USB0_DP_CFG_REG_ICSSM_GPIO_SEL_MAX                            (0x00000001U)


#define CSL_IOMUX_USB0_DP_CFG_REG_IE_OVERRIDE_MASK                              (0x00000020U)
#define CSL_IOMUX_USB0_DP_CFG_REG_IE_OVERRIDE_SHIFT                             (0x00000005U)
#define CSL_IOMUX_USB0_DP_CFG_REG_IE_OVERRIDE_RESETVAL                          (0x00000001U)
#define CSL_IOMUX_USB0_DP_CFG_REG_IE_OVERRIDE_MAX                               (0x00000001U)


#define CSL_IOMUX_USB0_DP_CFG_REG_IE_OVERRIDE_CTRL_MASK                         (0x00000010U)
#define CSL_IOMUX_USB0_DP_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                        (0x00000004U)
#define CSL_IOMUX_USB0_DP_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_USB0_DP_CFG_REG_IE_OVERRIDE_CTRL_MAX                          (0x00000001U)


#define CSL_IOMUX_USB0_DP_CFG_REG_INP_INV_SEL_MASK                              (0x00100000U)
#define CSL_IOMUX_USB0_DP_CFG_REG_INP_INV_SEL_SHIFT                             (0x00000014U)
#define CSL_IOMUX_USB0_DP_CFG_REG_INP_INV_SEL_RESETVAL                          (0x00000000U)
#define CSL_IOMUX_USB0_DP_CFG_REG_INP_INV_SEL_MAX                               (0x00000001U)


#define CSL_IOMUX_USB0_DP_CFG_REG_OE_OVERRIDE_MASK                              (0x00000080U)
#define CSL_IOMUX_USB0_DP_CFG_REG_OE_OVERRIDE_SHIFT                             (0x00000007U)
#define CSL_IOMUX_USB0_DP_CFG_REG_OE_OVERRIDE_RESETVAL                          (0x00000000U)
#define CSL_IOMUX_USB0_DP_CFG_REG_OE_OVERRIDE_MAX                               (0x00000001U)


#define CSL_IOMUX_USB0_DP_CFG_REG_OE_OVERRIDE_CTRL_MASK                         (0x00000040U)
#define CSL_IOMUX_USB0_DP_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                        (0x00000006U)
#define CSL_IOMUX_USB0_DP_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                     (0x00000001U)
#define CSL_IOMUX_USB0_DP_CFG_REG_OE_OVERRIDE_CTRL_MAX                          (0x00000001U)


#define CSL_IOMUX_USB0_DP_CFG_REG_PUPDSEL_MASK                                  (0x00000200U)
#define CSL_IOMUX_USB0_DP_CFG_REG_PUPDSEL_SHIFT                                 (0x00000009U)
#define CSL_IOMUX_USB0_DP_CFG_REG_PUPDSEL_RESETVAL                              (0x00000000U)
#define CSL_IOMUX_USB0_DP_CFG_REG_PUPDSEL_MAX                                   (0x00000001U)


#define CSL_IOMUX_USB0_DP_CFG_REG_QUAL_SEL_MASK                                 (0x000C0000U)
#define CSL_IOMUX_USB0_DP_CFG_REG_QUAL_SEL_SHIFT                                (0x00000012U)
#define CSL_IOMUX_USB0_DP_CFG_REG_QUAL_SEL_RESETVAL                             (0x00000000U)
#define CSL_IOMUX_USB0_DP_CFG_REG_QUAL_SEL_MAX                                  (0x00000003U)


#define CSL_IOMUX_USB0_DP_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                      (0x00400000U)
#define CSL_IOMUX_USB0_DP_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                     (0x00000016U)
#define CSL_IOMUX_USB0_DP_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL                  (0x00000000U)
#define CSL_IOMUX_USB0_DP_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                       (0x00000001U)



/* USB0_DM_CFG_REG */
#define CSL_IOMUX_USB0_DM_CFG_REG_FUNC_SEL_MASK                                 (0x0000000FU)
#define CSL_IOMUX_USB0_DM_CFG_REG_FUNC_SEL_SHIFT                                (0x00000000U)
#define CSL_IOMUX_USB0_DM_CFG_REG_FUNC_SEL_RESETVAL                             (0x00000000U)
#define CSL_IOMUX_USB0_DM_CFG_REG_FUNC_SEL_MAX                                  (0x0000000FU)


#define CSL_IOMUX_USB0_DM_CFG_REG_GPIO_SEL_MASK                                 (0x00010000U)
#define CSL_IOMUX_USB0_DM_CFG_REG_GPIO_SEL_SHIFT                                (0x00000010U)
#define CSL_IOMUX_USB0_DM_CFG_REG_GPIO_SEL_RESETVAL                             (0x00000000U)
#define CSL_IOMUX_USB0_DM_CFG_REG_GPIO_SEL_MAX                                  (0x00000001U)


#define CSL_IOMUX_USB0_DM_CFG_REG_ICSSM_GPIO_SEL_MASK                           (0x00200000U)
#define CSL_IOMUX_USB0_DM_CFG_REG_ICSSM_GPIO_SEL_SHIFT                          (0x00000015U)
#define CSL_IOMUX_USB0_DM_CFG_REG_ICSSM_GPIO_SEL_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_USB0_DM_CFG_REG_ICSSM_GPIO_SEL_MAX                            (0x00000001U)


#define CSL_IOMUX_USB0_DM_CFG_REG_IE_OVERRIDE_MASK                              (0x00000020U)
#define CSL_IOMUX_USB0_DM_CFG_REG_IE_OVERRIDE_SHIFT                             (0x00000005U)
#define CSL_IOMUX_USB0_DM_CFG_REG_IE_OVERRIDE_RESETVAL                          (0x00000001U)
#define CSL_IOMUX_USB0_DM_CFG_REG_IE_OVERRIDE_MAX                               (0x00000001U)


#define CSL_IOMUX_USB0_DM_CFG_REG_IE_OVERRIDE_CTRL_MASK                         (0x00000010U)
#define CSL_IOMUX_USB0_DM_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                        (0x00000004U)
#define CSL_IOMUX_USB0_DM_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                     (0x00000000U)
#define CSL_IOMUX_USB0_DM_CFG_REG_IE_OVERRIDE_CTRL_MAX                          (0x00000001U)


#define CSL_IOMUX_USB0_DM_CFG_REG_INP_INV_SEL_MASK                              (0x00100000U)
#define CSL_IOMUX_USB0_DM_CFG_REG_INP_INV_SEL_SHIFT                             (0x00000014U)
#define CSL_IOMUX_USB0_DM_CFG_REG_INP_INV_SEL_RESETVAL                          (0x00000000U)
#define CSL_IOMUX_USB0_DM_CFG_REG_INP_INV_SEL_MAX                               (0x00000001U)


#define CSL_IOMUX_USB0_DM_CFG_REG_OE_OVERRIDE_MASK                              (0x00000080U)
#define CSL_IOMUX_USB0_DM_CFG_REG_OE_OVERRIDE_SHIFT                             (0x00000007U)
#define CSL_IOMUX_USB0_DM_CFG_REG_OE_OVERRIDE_RESETVAL                          (0x00000000U)
#define CSL_IOMUX_USB0_DM_CFG_REG_OE_OVERRIDE_MAX                               (0x00000001U)


#define CSL_IOMUX_USB0_DM_CFG_REG_OE_OVERRIDE_CTRL_MASK                         (0x00000040U)
#define CSL_IOMUX_USB0_DM_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                        (0x00000006U)
#define CSL_IOMUX_USB0_DM_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                     (0x00000001U)
#define CSL_IOMUX_USB0_DM_CFG_REG_OE_OVERRIDE_CTRL_MAX                          (0x00000001U)


#define CSL_IOMUX_USB0_DM_CFG_REG_PUPDSEL_MASK                                  (0x00000200U)
#define CSL_IOMUX_USB0_DM_CFG_REG_PUPDSEL_SHIFT                                 (0x00000009U)
#define CSL_IOMUX_USB0_DM_CFG_REG_PUPDSEL_RESETVAL                              (0x00000000U)
#define CSL_IOMUX_USB0_DM_CFG_REG_PUPDSEL_MAX                                   (0x00000001U)


#define CSL_IOMUX_USB0_DM_CFG_REG_QUAL_SEL_MASK                                 (0x000C0000U)
#define CSL_IOMUX_USB0_DM_CFG_REG_QUAL_SEL_SHIFT                                (0x00000012U)
#define CSL_IOMUX_USB0_DM_CFG_REG_QUAL_SEL_RESETVAL                             (0x00000000U)
#define CSL_IOMUX_USB0_DM_CFG_REG_QUAL_SEL_MAX                                  (0x00000003U)


#define CSL_IOMUX_USB0_DM_CFG_REG_SAFETY_OVERRIDE_SEL_MASK                      (0x00400000U)
#define CSL_IOMUX_USB0_DM_CFG_REG_SAFETY_OVERRIDE_SEL_SHIFT                     (0x00000016U)
#define CSL_IOMUX_USB0_DM_CFG_REG_SAFETY_OVERRIDE_SEL_RESETVAL                  (0x00000000U)
#define CSL_IOMUX_USB0_DM_CFG_REG_SAFETY_OVERRIDE_SEL_MAX                       (0x00000001U)



/* WARMRSTN_CFG_REG */
#define CSL_IOMUX_WARMRSTN_CFG_REG_RESERVED_MASK                                (0x80000000U)
#define CSL_IOMUX_WARMRSTN_CFG_REG_RESERVED_SHIFT                               (0x0000001FU)
#define CSL_IOMUX_WARMRSTN_CFG_REG_RESERVED_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_WARMRSTN_CFG_REG_RESERVED_MAX                                 (0x00000001U)


#define CSL_IOMUX_WARMRSTN_CFG_REG_HVMODE_STATUS_MASK                           (0x01000000U)
#define CSL_IOMUX_WARMRSTN_CFG_REG_HVMODE_STATUS_SHIFT                          (0x00000018U)
#define CSL_IOMUX_WARMRSTN_CFG_REG_HVMODE_STATUS_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_WARMRSTN_CFG_REG_HVMODE_STATUS_MAX                            (0x00000001U)


#define CSL_IOMUX_WARMRSTN_CFG_REG_IE_OVERRIDE_MASK                             (0x00000020U)
#define CSL_IOMUX_WARMRSTN_CFG_REG_IE_OVERRIDE_SHIFT                            (0x00000005U)
#define CSL_IOMUX_WARMRSTN_CFG_REG_IE_OVERRIDE_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_WARMRSTN_CFG_REG_IE_OVERRIDE_MAX                              (0x00000001U)


#define CSL_IOMUX_WARMRSTN_CFG_REG_IE_OVERRIDE_CTRL_MASK                        (0x00000010U)
#define CSL_IOMUX_WARMRSTN_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                       (0x00000004U)
#define CSL_IOMUX_WARMRSTN_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                    (0x00000001U)
#define CSL_IOMUX_WARMRSTN_CFG_REG_IE_OVERRIDE_CTRL_MAX                         (0x00000001U)


#define CSL_IOMUX_WARMRSTN_CFG_REG_OE_OVERRIDE_MASK                             (0x00000080U)
#define CSL_IOMUX_WARMRSTN_CFG_REG_OE_OVERRIDE_SHIFT                            (0x00000007U)
#define CSL_IOMUX_WARMRSTN_CFG_REG_OE_OVERRIDE_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_WARMRSTN_CFG_REG_OE_OVERRIDE_MAX                              (0x00000001U)


#define CSL_IOMUX_WARMRSTN_CFG_REG_OE_OVERRIDE_CTRL_MASK                        (0x00000040U)
#define CSL_IOMUX_WARMRSTN_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                       (0x00000006U)
#define CSL_IOMUX_WARMRSTN_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                    (0x00000000U)
#define CSL_IOMUX_WARMRSTN_CFG_REG_OE_OVERRIDE_CTRL_MAX                         (0x00000001U)


#define CSL_IOMUX_WARMRSTN_CFG_REG_PE_MASK                                      (0x00000100U)
#define CSL_IOMUX_WARMRSTN_CFG_REG_PE_SHIFT                                     (0x00000008U)
#define CSL_IOMUX_WARMRSTN_CFG_REG_PE_RESETVAL                                  (0x00000001U)
#define CSL_IOMUX_WARMRSTN_CFG_REG_PE_MAX                                       (0x00000001U)


#define CSL_IOMUX_WARMRSTN_CFG_REG_PUPDSEL_MASK                                 (0x00000200U)
#define CSL_IOMUX_WARMRSTN_CFG_REG_PUPDSEL_SHIFT                                (0x00000009U)
#define CSL_IOMUX_WARMRSTN_CFG_REG_PUPDSEL_RESETVAL                             (0x00000000U)
#define CSL_IOMUX_WARMRSTN_CFG_REG_PUPDSEL_MAX                                  (0x00000001U)


#define CSL_IOMUX_WARMRSTN_CFG_REG_SC1_MASK                                     (0x00000400U)
#define CSL_IOMUX_WARMRSTN_CFG_REG_SC1_SHIFT                                    (0x0000000AU)
#define CSL_IOMUX_WARMRSTN_CFG_REG_SC1_RESETVAL                                 (0x00000001U)
#define CSL_IOMUX_WARMRSTN_CFG_REG_SC1_MAX                                      (0x00000001U)



/* SAFETY_ERRORN_CFG_REG */
#define CSL_IOMUX_SAFETY_ERRORN_CFG_REG_HSMASTER_MASK                           (0x80000000U)
#define CSL_IOMUX_SAFETY_ERRORN_CFG_REG_HSMASTER_SHIFT                          (0x0000001FU)
#define CSL_IOMUX_SAFETY_ERRORN_CFG_REG_RESERVED_RESETVAL                       (0x00000000U)
#define CSL_IOMUX_SAFETY_ERRORN_CFG_REG_RESERVED_MAX                            (0x00000001U)


#define CSL_IOMUX_SAFETY_ERRORN_CFG_REG_HVMODE_STATUS_MASK                      (0x01000000U)
#define CSL_IOMUX_SAFETY_ERRORN_CFG_REG_HVMODE_STATUS_SHIFT                     (0x00000018U)
#define CSL_IOMUX_SAFETY_ERRORN_CFG_REG_HVMODE_STATUS_RESETVAL                  (0x00000000U)
#define CSL_IOMUX_SAFETY_ERRORN_CFG_REG_HVMODE_STATUS_MAX                       (0x00000001U)


#define CSL_IOMUX_SAFETY_ERRORN_CFG_REG_IE_OVERRIDE_MASK                        (0x00000020U)
#define CSL_IOMUX_SAFETY_ERRORN_CFG_REG_IE_OVERRIDE_SHIFT                       (0x00000005U)
#define CSL_IOMUX_SAFETY_ERRORN_CFG_REG_IE_OVERRIDE_RESETVAL                    (0x00000000U)
#define CSL_IOMUX_SAFETY_ERRORN_CFG_REG_IE_OVERRIDE_MAX                         (0x00000001U)


#define CSL_IOMUX_SAFETY_ERRORN_CFG_REG_IE_OVERRIDE_CTRL_MASK                   (0x00000010U)
#define CSL_IOMUX_SAFETY_ERRORN_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                  (0x00000004U)
#define CSL_IOMUX_SAFETY_ERRORN_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL               (0x00000001U)
#define CSL_IOMUX_SAFETY_ERRORN_CFG_REG_IE_OVERRIDE_CTRL_MAX                    (0x00000001U)


#define CSL_IOMUX_SAFETY_ERRORN_CFG_REG_OE_OVERRIDE_MASK                        (0x00000080U)
#define CSL_IOMUX_SAFETY_ERRORN_CFG_REG_OE_OVERRIDE_SHIFT                       (0x00000007U)
#define CSL_IOMUX_SAFETY_ERRORN_CFG_REG_OE_OVERRIDE_RESETVAL                    (0x00000000U)
#define CSL_IOMUX_SAFETY_ERRORN_CFG_REG_OE_OVERRIDE_MAX                         (0x00000001U)


#define CSL_IOMUX_SAFETY_ERRORN_CFG_REG_OE_OVERRIDE_CTRL_MASK                   (0x00000040U)
#define CSL_IOMUX_SAFETY_ERRORN_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                  (0x00000006U)
#define CSL_IOMUX_SAFETY_ERRORN_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL               (0x00000000U)
#define CSL_IOMUX_SAFETY_ERRORN_CFG_REG_OE_OVERRIDE_CTRL_MAX                    (0x00000001U)


#define CSL_IOMUX_SAFETY_ERRORN_CFG_REG_PE_MASK                                 (0x00000100U)
#define CSL_IOMUX_SAFETY_ERRORN_CFG_REG_PE_SHIFT                                (0x00000008U)
#define CSL_IOMUX_SAFETY_ERRORN_CFG_REG_PE_RESETVAL                             (0x00000000U)
#define CSL_IOMUX_SAFETY_ERRORN_CFG_REG_PE_MAX                                  (0x00000001U)


#define CSL_IOMUX_SAFETY_ERRORN_CFG_REG_PUPDSEL_MASK                            (0x00000200U)
#define CSL_IOMUX_SAFETY_ERRORN_CFG_REG_PUPDSEL_SHIFT                           (0x00000009U)
#define CSL_IOMUX_SAFETY_ERRORN_CFG_REG_PUPDSEL_RESETVAL                        (0x00000000U)
#define CSL_IOMUX_SAFETY_ERRORN_CFG_REG_PUPDSEL_MAX                             (0x00000001U)


#define CSL_IOMUX_SAFETY_ERRORN_CFG_REG_SC1_MASK                                (0x00000400U)
#define CSL_IOMUX_SAFETY_ERRORN_CFG_REG_SC1_SHIFT                               (0x0000000AU)
#define CSL_IOMUX_SAFETY_ERRORN_CFG_REG_SC1_RESETVAL                            (0x00000001U)
#define CSL_IOMUX_SAFETY_ERRORN_CFG_REG_SC1_MAX                                 (0x00000001U)



/* TDI_CFG_REG */
#define CSL_IOMUX_TDI_CFG_REG_RESERVED_MASK                                     (0x80000000U)
#define CSL_IOMUX_TDI_CFG_REG_RESERVED_SHIFT                                    (0x0000001FU)
#define CSL_IOMUX_TDI_CFG_REG_RESERVED_RESETVAL                                 (0x00000000U)
#define CSL_IOMUX_TDI_CFG_REG_RESERVED_MAX                                      (0x00000001U)


#define CSL_IOMUX_TDI_CFG_REG_HVMODE_STATUS_MASK                                (0x01000000U)
#define CSL_IOMUX_TDI_CFG_REG_HVMODE_STATUS_SHIFT                               (0x00000018U)
#define CSL_IOMUX_TDI_CFG_REG_HVMODE_STATUS_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_TDI_CFG_REG_HVMODE_STATUS_MAX                                 (0x00000001U)


#define CSL_IOMUX_TDI_CFG_REG_IE_OVERRIDE_MASK                                  (0x00000020U)
#define CSL_IOMUX_TDI_CFG_REG_IE_OVERRIDE_SHIFT                                 (0x00000005U)
#define CSL_IOMUX_TDI_CFG_REG_IE_OVERRIDE_RESETVAL                              (0x00000000U)
#define CSL_IOMUX_TDI_CFG_REG_IE_OVERRIDE_MAX                                   (0x00000001U)


#define CSL_IOMUX_TDI_CFG_REG_IE_OVERRIDE_CTRL_MASK                             (0x00000010U)
#define CSL_IOMUX_TDI_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                            (0x00000004U)
#define CSL_IOMUX_TDI_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                         (0x00000001U)
#define CSL_IOMUX_TDI_CFG_REG_IE_OVERRIDE_CTRL_MAX                              (0x00000001U)


#define CSL_IOMUX_TDI_CFG_REG_OE_OVERRIDE_MASK                                  (0x00000080U)
#define CSL_IOMUX_TDI_CFG_REG_OE_OVERRIDE_SHIFT                                 (0x00000007U)
#define CSL_IOMUX_TDI_CFG_REG_OE_OVERRIDE_RESETVAL                              (0x00000001U)
#define CSL_IOMUX_TDI_CFG_REG_OE_OVERRIDE_MAX                                   (0x00000001U)


#define CSL_IOMUX_TDI_CFG_REG_OE_OVERRIDE_CTRL_MASK                             (0x00000040U)
#define CSL_IOMUX_TDI_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                            (0x00000006U)
#define CSL_IOMUX_TDI_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                         (0x00000001U)
#define CSL_IOMUX_TDI_CFG_REG_OE_OVERRIDE_CTRL_MAX                              (0x00000001U)


#define CSL_IOMUX_TDI_CFG_REG_PE_MASK                                           (0x00000100U)
#define CSL_IOMUX_TDI_CFG_REG_PE_SHIFT                                          (0x00000008U)
#define CSL_IOMUX_TDI_CFG_REG_PE_RESETVAL                                       (0x00000000U)
#define CSL_IOMUX_TDI_CFG_REG_PE_MAX                                            (0x00000001U)


#define CSL_IOMUX_TDI_CFG_REG_PUPDSEL_MASK                                      (0x00000200U)
#define CSL_IOMUX_TDI_CFG_REG_PUPDSEL_SHIFT                                     (0x00000009U)
#define CSL_IOMUX_TDI_CFG_REG_PUPDSEL_RESETVAL                                  (0x00000001U)
#define CSL_IOMUX_TDI_CFG_REG_PUPDSEL_MAX                                       (0x00000001U)


#define CSL_IOMUX_TDI_CFG_REG_SC1_MASK                                          (0x00000400U)
#define CSL_IOMUX_TDI_CFG_REG_SC1_SHIFT                                         (0x0000000AU)
#define CSL_IOMUX_TDI_CFG_REG_SC1_RESETVAL                                      (0x00000001U)
#define CSL_IOMUX_TDI_CFG_REG_SC1_MAX                                           (0x00000001U)



/* TDO_CFG_REG */
#define CSL_IOMUX_TDO_CFG_REG_RESERVED_MASK                                     (0x80000000U)
#define CSL_IOMUX_TDO_CFG_REG_RESERVED_SHIFT                                    (0x0000001FU)
#define CSL_IOMUX_TDO_CFG_REG_RESERVED_RESETVAL                                 (0x00000000U)
#define CSL_IOMUX_TDO_CFG_REG_RESERVED_MAX                                      (0x00000001U)


#define CSL_IOMUX_TDO_CFG_REG_HVMODE_STATUS_MASK                                (0x01000000U)
#define CSL_IOMUX_TDO_CFG_REG_HVMODE_STATUS_SHIFT                               (0x00000018U)
#define CSL_IOMUX_TDO_CFG_REG_HVMODE_STATUS_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_TDO_CFG_REG_HVMODE_STATUS_MAX                                 (0x00000001U)


#define CSL_IOMUX_TDO_CFG_REG_IE_OVERRIDE_MASK                                  (0x00000020U)
#define CSL_IOMUX_TDO_CFG_REG_IE_OVERRIDE_SHIFT                                 (0x00000005U)
#define CSL_IOMUX_TDO_CFG_REG_IE_OVERRIDE_RESETVAL                              (0x00000001U)
#define CSL_IOMUX_TDO_CFG_REG_IE_OVERRIDE_MAX                                   (0x00000001U)


#define CSL_IOMUX_TDO_CFG_REG_IE_OVERRIDE_CTRL_MASK                             (0x00000010U)
#define CSL_IOMUX_TDO_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                            (0x00000004U)
#define CSL_IOMUX_TDO_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                         (0x00000001U)
#define CSL_IOMUX_TDO_CFG_REG_IE_OVERRIDE_CTRL_MAX                              (0x00000001U)


#define CSL_IOMUX_TDO_CFG_REG_OE_OVERRIDE_MASK                                  (0x00000080U)
#define CSL_IOMUX_TDO_CFG_REG_OE_OVERRIDE_SHIFT                                 (0x00000007U)
#define CSL_IOMUX_TDO_CFG_REG_OE_OVERRIDE_RESETVAL                              (0x00000000U)
#define CSL_IOMUX_TDO_CFG_REG_OE_OVERRIDE_MAX                                   (0x00000001U)


#define CSL_IOMUX_TDO_CFG_REG_OE_OVERRIDE_CTRL_MASK                             (0x00000040U)
#define CSL_IOMUX_TDO_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                            (0x00000006U)
#define CSL_IOMUX_TDO_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_TDO_CFG_REG_OE_OVERRIDE_CTRL_MAX                              (0x00000001U)


#define CSL_IOMUX_TDO_CFG_REG_PE_MASK                                           (0x00000100U)
#define CSL_IOMUX_TDO_CFG_REG_PE_SHIFT                                          (0x00000008U)
#define CSL_IOMUX_TDO_CFG_REG_PE_RESETVAL                                       (0x00000000U)
#define CSL_IOMUX_TDO_CFG_REG_PE_MAX                                            (0x00000001U)


#define CSL_IOMUX_TDO_CFG_REG_PUPDSEL_MASK                                      (0x00000200U)
#define CSL_IOMUX_TDO_CFG_REG_PUPDSEL_SHIFT                                     (0x00000009U)
#define CSL_IOMUX_TDO_CFG_REG_PUPDSEL_RESETVAL                                  (0x00000001U)
#define CSL_IOMUX_TDO_CFG_REG_PUPDSEL_MAX                                       (0x00000001U)


#define CSL_IOMUX_TDO_CFG_REG_SC1_MASK                                          (0x00000400U)
#define CSL_IOMUX_TDO_CFG_REG_SC1_SHIFT                                         (0x0000000AU)
#define CSL_IOMUX_TDO_CFG_REG_SC1_RESETVAL                                      (0x00000001U)
#define CSL_IOMUX_TDO_CFG_REG_SC1_MAX                                           (0x00000001U)



/* TMS_CFG_REG */
#define CSL_IOMUX_TMS_CFG_REG_RESERVED_MASK                                     (0x80000000U)
#define CSL_IOMUX_TMS_CFG_REG_RESERVED_SHIFT                                    (0x0000001FU)
#define CSL_IOMUX_TMS_CFG_REG_RESERVED_RESETVAL                                 (0x00000000U)
#define CSL_IOMUX_TMS_CFG_REG_RESERVED_MAX                                      (0x00000001U)


#define CSL_IOMUX_TMS_CFG_REG_HVMODE_STATUS_MASK                                (0x01000000U)
#define CSL_IOMUX_TMS_CFG_REG_HVMODE_STATUS_SHIFT                               (0x00000018U)
#define CSL_IOMUX_TMS_CFG_REG_HVMODE_STATUS_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_TMS_CFG_REG_HVMODE_STATUS_MAX                                 (0x00000001U)


#define CSL_IOMUX_TMS_CFG_REG_IE_OVERRIDE_MASK                                  (0x00000020U)
#define CSL_IOMUX_TMS_CFG_REG_IE_OVERRIDE_SHIFT                                 (0x00000005U)
#define CSL_IOMUX_TMS_CFG_REG_IE_OVERRIDE_RESETVAL                              (0x00000000U)
#define CSL_IOMUX_TMS_CFG_REG_IE_OVERRIDE_MAX                                   (0x00000001U)


#define CSL_IOMUX_TMS_CFG_REG_IE_OVERRIDE_CTRL_MASK                             (0x00000010U)
#define CSL_IOMUX_TMS_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                            (0x00000004U)
#define CSL_IOMUX_TMS_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                         (0x00000001U)
#define CSL_IOMUX_TMS_CFG_REG_IE_OVERRIDE_CTRL_MAX                              (0x00000001U)


#define CSL_IOMUX_TMS_CFG_REG_OE_OVERRIDE_MASK                                  (0x00000080U)
#define CSL_IOMUX_TMS_CFG_REG_OE_OVERRIDE_SHIFT                                 (0x00000007U)
#define CSL_IOMUX_TMS_CFG_REG_OE_OVERRIDE_RESETVAL                              (0x00000000U)
#define CSL_IOMUX_TMS_CFG_REG_OE_OVERRIDE_MAX                                   (0x00000001U)


#define CSL_IOMUX_TMS_CFG_REG_OE_OVERRIDE_CTRL_MASK                             (0x00000040U)
#define CSL_IOMUX_TMS_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                            (0x00000006U)
#define CSL_IOMUX_TMS_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_TMS_CFG_REG_OE_OVERRIDE_CTRL_MAX                              (0x00000001U)


#define CSL_IOMUX_TMS_CFG_REG_PE_MASK                                           (0x00000100U)
#define CSL_IOMUX_TMS_CFG_REG_PE_SHIFT                                          (0x00000008U)
#define CSL_IOMUX_TMS_CFG_REG_PE_RESETVAL                                       (0x00000000U)
#define CSL_IOMUX_TMS_CFG_REG_PE_MAX                                            (0x00000001U)


#define CSL_IOMUX_TMS_CFG_REG_PUPDSEL_MASK                                      (0x00000200U)
#define CSL_IOMUX_TMS_CFG_REG_PUPDSEL_SHIFT                                     (0x00000009U)
#define CSL_IOMUX_TMS_CFG_REG_PUPDSEL_RESETVAL                                  (0x00000001U)
#define CSL_IOMUX_TMS_CFG_REG_PUPDSEL_MAX                                       (0x00000001U)


#define CSL_IOMUX_TMS_CFG_REG_SC1_MASK                                          (0x00000400U)
#define CSL_IOMUX_TMS_CFG_REG_SC1_SHIFT                                         (0x0000000AU)
#define CSL_IOMUX_TMS_CFG_REG_SC1_RESETVAL                                      (0x00000001U)
#define CSL_IOMUX_TMS_CFG_REG_SC1_MAX                                           (0x00000001U)



/* TCK_CFG_REG */
#define CSL_IOMUX_TCK_CFG_REG_RESERVED_MASK                                     (0x80000000U)
#define CSL_IOMUX_TCK_CFG_REG_RESERVED_SHIFT                                    (0x0000001FU)
#define CSL_IOMUX_TCK_CFG_REG_RESERVED_RESETVAL                                 (0x00000000U)
#define CSL_IOMUX_TCK_CFG_REG_RESERVED_MAX                                      (0x00000001U)


#define CSL_IOMUX_TCK_CFG_REG_HVMODE_STATUS_MASK                                (0x01000000U)
#define CSL_IOMUX_TCK_CFG_REG_HVMODE_STATUS_SHIFT                               (0x00000018U)
#define CSL_IOMUX_TCK_CFG_REG_HVMODE_STATUS_RESETVAL                            (0x00000000U)
#define CSL_IOMUX_TCK_CFG_REG_HVMODE_STATUS_MAX                                 (0x00000001U)


#define CSL_IOMUX_TCK_CFG_REG_IE_OVERRIDE_MASK                                  (0x00000020U)
#define CSL_IOMUX_TCK_CFG_REG_IE_OVERRIDE_SHIFT                                 (0x00000005U)
#define CSL_IOMUX_TCK_CFG_REG_IE_OVERRIDE_RESETVAL                              (0x00000000U)
#define CSL_IOMUX_TCK_CFG_REG_IE_OVERRIDE_MAX                                   (0x00000001U)


#define CSL_IOMUX_TCK_CFG_REG_IE_OVERRIDE_CTRL_MASK                             (0x00000010U)
#define CSL_IOMUX_TCK_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                            (0x00000004U)
#define CSL_IOMUX_TCK_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                         (0x00000001U)
#define CSL_IOMUX_TCK_CFG_REG_IE_OVERRIDE_CTRL_MAX                              (0x00000001U)


#define CSL_IOMUX_TCK_CFG_REG_OE_OVERRIDE_MASK                                  (0x00000080U)
#define CSL_IOMUX_TCK_CFG_REG_OE_OVERRIDE_SHIFT                                 (0x00000007U)
#define CSL_IOMUX_TCK_CFG_REG_OE_OVERRIDE_RESETVAL                              (0x00000000U)
#define CSL_IOMUX_TCK_CFG_REG_OE_OVERRIDE_MAX                                   (0x00000001U)


#define CSL_IOMUX_TCK_CFG_REG_OE_OVERRIDE_CTRL_MASK                             (0x00000040U)
#define CSL_IOMUX_TCK_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                            (0x00000006U)
#define CSL_IOMUX_TCK_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_TCK_CFG_REG_OE_OVERRIDE_CTRL_MAX                              (0x00000001U)


#define CSL_IOMUX_TCK_CFG_REG_PE_MASK                                           (0x00000100U)
#define CSL_IOMUX_TCK_CFG_REG_PE_SHIFT                                          (0x00000008U)
#define CSL_IOMUX_TCK_CFG_REG_PE_RESETVAL                                       (0x00000000U)
#define CSL_IOMUX_TCK_CFG_REG_PE_MAX                                            (0x00000001U)


#define CSL_IOMUX_TCK_CFG_REG_PUPDSEL_MASK                                      (0x00000200U)
#define CSL_IOMUX_TCK_CFG_REG_PUPDSEL_SHIFT                                     (0x00000009U)
#define CSL_IOMUX_TCK_CFG_REG_PUPDSEL_RESETVAL                                  (0x00000001U)
#define CSL_IOMUX_TCK_CFG_REG_PUPDSEL_MAX                                       (0x00000001U)


#define CSL_IOMUX_TCK_CFG_REG_SC1_MASK                                          (0x00000400U)
#define CSL_IOMUX_TCK_CFG_REG_SC1_SHIFT                                         (0x0000000AU)
#define CSL_IOMUX_TCK_CFG_REG_SC1_RESETVAL                                      (0x00000000U)
#define CSL_IOMUX_TCK_CFG_REG_SC1_MAX                                           (0x00000001U)



/* OSPI0_CLKLB_CFG_REG */
#define CSL_IOMUX_OSPI0_CLKLB_CFG_REG_RESERVED_MASK                             (0x80000000U)
#define CSL_IOMUX_OSPI0_CLKLB_CFG_REG_RESERVED_SHIFT                            (0x0000001FU)
#define CSL_IOMUX_OSPI0_CLKLB_CFG_REG_RESERVED_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_OSPI0_CLKLB_CFG_REG_RESERVED_MAX                              (0x00000001U)


#define CSL_IOMUX_OSPI0_CLKLB_CFG_REG_HVMODE_STATUS_MASK                        (0x01000000U)
#define CSL_IOMUX_OSPI0_CLKLB_CFG_REG_HVMODE_STATUS_SHIFT                       (0x00000018U)
#define CSL_IOMUX_OSPI0_CLKLB_CFG_REG_HVMODE_STATUS_RESETVAL                    (0x00000000U)
#define CSL_IOMUX_OSPI0_CLKLB_CFG_REG_HVMODE_STATUS_MAX                         (0x00000001U)


#define CSL_IOMUX_OSPI0_CLKLB_CFG_REG_IE_OVERRIDE_MASK                          (0x00000020U)
#define CSL_IOMUX_OSPI0_CLKLB_CFG_REG_IE_OVERRIDE_SHIFT                         (0x00000005U)
#define CSL_IOMUX_OSPI0_CLKLB_CFG_REG_IE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_IOMUX_OSPI0_CLKLB_CFG_REG_IE_OVERRIDE_MAX                           (0x00000001U)


#define CSL_IOMUX_OSPI0_CLKLB_CFG_REG_IE_OVERRIDE_CTRL_MASK                     (0x00000010U)
#define CSL_IOMUX_OSPI0_CLKLB_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                    (0x00000004U)
#define CSL_IOMUX_OSPI0_CLKLB_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_IOMUX_OSPI0_CLKLB_CFG_REG_IE_OVERRIDE_CTRL_MAX                      (0x00000001U)


#define CSL_IOMUX_OSPI0_CLKLB_CFG_REG_OE_OVERRIDE_MASK                          (0x00000080U)
#define CSL_IOMUX_OSPI0_CLKLB_CFG_REG_OE_OVERRIDE_SHIFT                         (0x00000007U)
#define CSL_IOMUX_OSPI0_CLKLB_CFG_REG_OE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_IOMUX_OSPI0_CLKLB_CFG_REG_OE_OVERRIDE_MAX                           (0x00000001U)


#define CSL_IOMUX_OSPI0_CLKLB_CFG_REG_OE_OVERRIDE_CTRL_MASK                     (0x00000040U)
#define CSL_IOMUX_OSPI0_CLKLB_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                    (0x00000006U)
#define CSL_IOMUX_OSPI0_CLKLB_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_IOMUX_OSPI0_CLKLB_CFG_REG_OE_OVERRIDE_CTRL_MAX                      (0x00000001U)


#define CSL_IOMUX_OSPI0_CLKLB_CFG_REG_PE_MASK                                   (0x00000100U)
#define CSL_IOMUX_OSPI0_CLKLB_CFG_REG_PE_SHIFT                                  (0x00000008U)
#define CSL_IOMUX_OSPI0_CLKLB_CFG_REG_PE_RESETVAL                               (0x00000001U)
#define CSL_IOMUX_OSPI0_CLKLB_CFG_REG_PE_MAX                                    (0x00000001U)


#define CSL_IOMUX_OSPI0_CLKLB_CFG_REG_PUPDSEL_MASK                              (0x00000200U)
#define CSL_IOMUX_OSPI0_CLKLB_CFG_REG_PUPDSEL_SHIFT                             (0x00000009U)
#define CSL_IOMUX_OSPI0_CLKLB_CFG_REG_PUPDSEL_RESETVAL                          (0x00000000U)
#define CSL_IOMUX_OSPI0_CLKLB_CFG_REG_PUPDSEL_MAX                               (0x00000001U)


#define CSL_IOMUX_OSPI0_CLKLB_CFG_REG_SC1_MASK                                  (0x00000400U)
#define CSL_IOMUX_OSPI0_CLKLB_CFG_REG_SC1_SHIFT                                 (0x0000000AU)
#define CSL_IOMUX_OSPI0_CLKLB_CFG_REG_SC1_RESETVAL                              (0x00000001U)
#define CSL_IOMUX_OSPI0_CLKLB_CFG_REG_SC1_MAX                                   (0x00000001U)



/* OSPI1_CLKLB_CFG_REG */
#define CSL_IOMUX_OSPI1_CLKLB_CFG_REG_RESERVED_MASK                             (0x80000000U)
#define CSL_IOMUX_OSPI1_CLKLB_CFG_REG_RESERVED_SHIFT                            (0x0000001FU)
#define CSL_IOMUX_OSPI1_CLKLB_CFG_REG_RESERVED_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_OSPI1_CLKLB_CFG_REG_RESERVED_MAX                              (0x00000001U)


#define CSL_IOMUX_OSPI1_CLKLB_CFG_REG_HVMODE_STATUS_MASK                        (0x01000000U)
#define CSL_IOMUX_OSPI1_CLKLB_CFG_REG_HVMODE_STATUS_SHIFT                       (0x00000018U)
#define CSL_IOMUX_OSPI1_CLKLB_CFG_REG_HVMODE_STATUS_RESETVAL                    (0x00000000U)
#define CSL_IOMUX_OSPI1_CLKLB_CFG_REG_HVMODE_STATUS_MAX                         (0x00000001U)


#define CSL_IOMUX_OSPI1_CLKLB_CFG_REG_IE_OVERRIDE_MASK                          (0x00000020U)
#define CSL_IOMUX_OSPI1_CLKLB_CFG_REG_IE_OVERRIDE_SHIFT                         (0x00000005U)
#define CSL_IOMUX_OSPI1_CLKLB_CFG_REG_IE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_IOMUX_OSPI1_CLKLB_CFG_REG_IE_OVERRIDE_MAX                           (0x00000001U)


#define CSL_IOMUX_OSPI1_CLKLB_CFG_REG_IE_OVERRIDE_CTRL_MASK                     (0x00000010U)
#define CSL_IOMUX_OSPI1_CLKLB_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                    (0x00000004U)
#define CSL_IOMUX_OSPI1_CLKLB_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_IOMUX_OSPI1_CLKLB_CFG_REG_IE_OVERRIDE_CTRL_MAX                      (0x00000001U)


#define CSL_IOMUX_OSPI1_CLKLB_CFG_REG_OE_OVERRIDE_MASK                          (0x00000080U)
#define CSL_IOMUX_OSPI1_CLKLB_CFG_REG_OE_OVERRIDE_SHIFT                         (0x00000007U)
#define CSL_IOMUX_OSPI1_CLKLB_CFG_REG_OE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_IOMUX_OSPI1_CLKLB_CFG_REG_OE_OVERRIDE_MAX                           (0x00000001U)


#define CSL_IOMUX_OSPI1_CLKLB_CFG_REG_OE_OVERRIDE_CTRL_MASK                     (0x00000040U)
#define CSL_IOMUX_OSPI1_CLKLB_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                    (0x00000006U)
#define CSL_IOMUX_OSPI1_CLKLB_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_IOMUX_OSPI1_CLKLB_CFG_REG_OE_OVERRIDE_CTRL_MAX                      (0x00000001U)


#define CSL_IOMUX_OSPI1_CLKLB_CFG_REG_PE_MASK                                   (0x00000100U)
#define CSL_IOMUX_OSPI1_CLKLB_CFG_REG_PE_SHIFT                                  (0x00000008U)
#define CSL_IOMUX_OSPI1_CLKLB_CFG_REG_PE_RESETVAL                               (0x00000001U)
#define CSL_IOMUX_OSPI1_CLKLB_CFG_REG_PE_MAX                                    (0x00000001U)


#define CSL_IOMUX_OSPI1_CLKLB_CFG_REG_PUPDSEL_MASK                              (0x00000200U)
#define CSL_IOMUX_OSPI1_CLKLB_CFG_REG_PUPDSEL_SHIFT                             (0x00000009U)
#define CSL_IOMUX_OSPI1_CLKLB_CFG_REG_PUPDSEL_RESETVAL                          (0x00000000U)
#define CSL_IOMUX_OSPI1_CLKLB_CFG_REG_PUPDSEL_MAX                               (0x00000001U)


#define CSL_IOMUX_OSPI1_CLKLB_CFG_REG_SC1_MASK                                  (0x00000400U)
#define CSL_IOMUX_OSPI1_CLKLB_CFG_REG_SC1_SHIFT                                 (0x0000000AU)
#define CSL_IOMUX_OSPI1_CLKLB_CFG_REG_SC1_RESETVAL                              (0x00000001U)
#define CSL_IOMUX_OSPI1_CLKLB_CFG_REG_SC1_MAX                                   (0x00000001U)



/* QUAL_GRP_0_CFG_REG */
#define CSL_IOMUX_QUAL_GRP_0_CFG_REG_PERIOD_PER_SAMPLE_MASK                     (0x000000FFU)
#define CSL_IOMUX_QUAL_GRP_0_CFG_REG_PERIOD_PER_SAMPLE_SHIFT                    (0x00000000U)
#define CSL_IOMUX_QUAL_GRP_0_CFG_REG_PERIOD_PER_SAMPLE_RESETVAL                 (0x00000000U)
#define CSL_IOMUX_QUAL_GRP_0_CFG_REG_PERIOD_PER_SAMPLE_MAX                      (0x000000FFU)


#define CSL_IOMUX_QUAL_GRP_0_CFG_REG_RESERVED_MASK                              (0x80000000U)
#define CSL_IOMUX_QUAL_GRP_0_CFG_REG_RESERVED_SHIFT                             (0x0000001FU)
#define CSL_IOMUX_QUAL_GRP_0_CFG_REG_RESERVED_RESETVAL                          (0x00000000U)
#define CSL_IOMUX_QUAL_GRP_0_CFG_REG_RESERVED_MAX                               (0x00000001U)



/* QUAL_GRP_1_CFG_REG */
#define CSL_IOMUX_QUAL_GRP_1_CFG_REG_PERIOD_PER_SAMPLE_MASK                     (0x000000FFU)
#define CSL_IOMUX_QUAL_GRP_1_CFG_REG_PERIOD_PER_SAMPLE_SHIFT                    (0x00000000U)
#define CSL_IOMUX_QUAL_GRP_1_CFG_REG_PERIOD_PER_SAMPLE_RESETVAL                 (0x00000000U)
#define CSL_IOMUX_QUAL_GRP_1_CFG_REG_PERIOD_PER_SAMPLE_MAX                      (0x000000FFU)


#define CSL_IOMUX_QUAL_GRP_1_CFG_REG_RESERVED_MASK                              (0x80000000U)
#define CSL_IOMUX_QUAL_GRP_1_CFG_REG_RESERVED_SHIFT                             (0x0000001FU)
#define CSL_IOMUX_QUAL_GRP_1_CFG_REG_RESERVED_RESETVAL                          (0x00000000U)
#define CSL_IOMUX_QUAL_GRP_1_CFG_REG_RESERVED_MAX                               (0x00000001U)



/* QUAL_GRP_2_CFG_REG */
#define CSL_IOMUX_QUAL_GRP_2_CFG_REG_PERIOD_PER_SAMPLE_MASK                     (0x000000FFU)
#define CSL_IOMUX_QUAL_GRP_2_CFG_REG_PERIOD_PER_SAMPLE_SHIFT                    (0x00000000U)
#define CSL_IOMUX_QUAL_GRP_2_CFG_REG_PERIOD_PER_SAMPLE_RESETVAL                 (0x00000000U)
#define CSL_IOMUX_QUAL_GRP_2_CFG_REG_PERIOD_PER_SAMPLE_MAX                      (0x000000FFU)


#define CSL_IOMUX_QUAL_GRP_2_CFG_REG_RESERVED_MASK                              (0x80000000U)
#define CSL_IOMUX_QUAL_GRP_2_CFG_REG_RESERVED_SHIFT                             (0x0000001FU)
#define CSL_IOMUX_QUAL_GRP_2_CFG_REG_RESERVED_RESETVAL                          (0x00000000U)
#define CSL_IOMUX_QUAL_GRP_2_CFG_REG_RESERVED_MAX                               (0x00000001U)



/* QUAL_GRP_3_CFG_REG */
#define CSL_IOMUX_QUAL_GRP_3_CFG_REG_PERIOD_PER_SAMPLE_MASK                     (0x000000FFU)
#define CSL_IOMUX_QUAL_GRP_3_CFG_REG_PERIOD_PER_SAMPLE_SHIFT                    (0x00000000U)
#define CSL_IOMUX_QUAL_GRP_3_CFG_REG_PERIOD_PER_SAMPLE_RESETVAL                 (0x00000000U)
#define CSL_IOMUX_QUAL_GRP_3_CFG_REG_PERIOD_PER_SAMPLE_MAX                      (0x000000FFU)


#define CSL_IOMUX_QUAL_GRP_3_CFG_REG_RESERVED_MASK                              (0x80000000U)
#define CSL_IOMUX_QUAL_GRP_3_CFG_REG_RESERVED_SHIFT                             (0x0000001FU)
#define CSL_IOMUX_QUAL_GRP_3_CFG_REG_RESERVED_RESETVAL                          (0x00000000U)
#define CSL_IOMUX_QUAL_GRP_3_CFG_REG_RESERVED_MAX                               (0x00000001U)



/* QUAL_GRP_4_CFG_REG */
#define CSL_IOMUX_QUAL_GRP_4_CFG_REG_PERIOD_PER_SAMPLE_MASK                     (0x000000FFU)
#define CSL_IOMUX_QUAL_GRP_4_CFG_REG_PERIOD_PER_SAMPLE_SHIFT                    (0x00000000U)
#define CSL_IOMUX_QUAL_GRP_4_CFG_REG_PERIOD_PER_SAMPLE_RESETVAL                 (0x00000000U)
#define CSL_IOMUX_QUAL_GRP_4_CFG_REG_PERIOD_PER_SAMPLE_MAX                      (0x000000FFU)


#define CSL_IOMUX_QUAL_GRP_4_CFG_REG_RESERVED_MASK                              (0x80000000U)
#define CSL_IOMUX_QUAL_GRP_4_CFG_REG_RESERVED_SHIFT                             (0x0000001FU)
#define CSL_IOMUX_QUAL_GRP_4_CFG_REG_RESERVED_RESETVAL                          (0x00000000U)
#define CSL_IOMUX_QUAL_GRP_4_CFG_REG_RESERVED_MAX                               (0x00000001U)



/* QUAL_GRP_5_CFG_REG */
#define CSL_IOMUX_QUAL_GRP_5_CFG_REG_PERIOD_PER_SAMPLE_MASK                     (0x000000FFU)
#define CSL_IOMUX_QUAL_GRP_5_CFG_REG_PERIOD_PER_SAMPLE_SHIFT                    (0x00000000U)
#define CSL_IOMUX_QUAL_GRP_5_CFG_REG_PERIOD_PER_SAMPLE_RESETVAL                 (0x00000000U)
#define CSL_IOMUX_QUAL_GRP_5_CFG_REG_PERIOD_PER_SAMPLE_MAX                      (0x000000FFU)


#define CSL_IOMUX_QUAL_GRP_5_CFG_REG_RESERVED_MASK                              (0x80000000U)
#define CSL_IOMUX_QUAL_GRP_5_CFG_REG_RESERVED_SHIFT                             (0x0000001FU)
#define CSL_IOMUX_QUAL_GRP_5_CFG_REG_RESERVED_RESETVAL                          (0x00000000U)
#define CSL_IOMUX_QUAL_GRP_5_CFG_REG_RESERVED_MAX                               (0x00000001U)



/* QUAL_GRP_6_CFG_REG */
#define CSL_IOMUX_QUAL_GRP_6_CFG_REG_PERIOD_PER_SAMPLE_MASK                     (0x000000FFU)
#define CSL_IOMUX_QUAL_GRP_6_CFG_REG_PERIOD_PER_SAMPLE_SHIFT                    (0x00000000U)
#define CSL_IOMUX_QUAL_GRP_6_CFG_REG_PERIOD_PER_SAMPLE_RESETVAL                 (0x00000000U)
#define CSL_IOMUX_QUAL_GRP_6_CFG_REG_PERIOD_PER_SAMPLE_MAX                      (0x000000FFU)


#define CSL_IOMUX_QUAL_GRP_6_CFG_REG_RESERVED_MASK                              (0x80000000U)
#define CSL_IOMUX_QUAL_GRP_6_CFG_REG_RESERVED_SHIFT                             (0x0000001FU)
#define CSL_IOMUX_QUAL_GRP_6_CFG_REG_RESERVED_RESETVAL                          (0x00000000U)
#define CSL_IOMUX_QUAL_GRP_6_CFG_REG_RESERVED_MAX                               (0x00000001U)



/* QUAL_GRP_7_CFG_REG */
#define CSL_IOMUX_QUAL_GRP_7_CFG_REG_PERIOD_PER_SAMPLE_MASK                     (0x000000FFU)
#define CSL_IOMUX_QUAL_GRP_7_CFG_REG_PERIOD_PER_SAMPLE_SHIFT                    (0x00000000U)
#define CSL_IOMUX_QUAL_GRP_7_CFG_REG_PERIOD_PER_SAMPLE_RESETVAL                 (0x00000000U)
#define CSL_IOMUX_QUAL_GRP_7_CFG_REG_PERIOD_PER_SAMPLE_MAX                      (0x000000FFU)


#define CSL_IOMUX_QUAL_GRP_7_CFG_REG_RESERVED_MASK                              (0x80000000U)
#define CSL_IOMUX_QUAL_GRP_7_CFG_REG_RESERVED_SHIFT                             (0x0000001FU)
#define CSL_IOMUX_QUAL_GRP_7_CFG_REG_RESERVED_RESETVAL                          (0x00000000U)
#define CSL_IOMUX_QUAL_GRP_7_CFG_REG_RESERVED_MAX                               (0x00000001U)



/* QUAL_GRP_8_CFG_REG */
#define CSL_IOMUX_QUAL_GRP_8_CFG_REG_PERIOD_PER_SAMPLE_MASK                     (0x000000FFU)
#define CSL_IOMUX_QUAL_GRP_8_CFG_REG_PERIOD_PER_SAMPLE_SHIFT                    (0x00000000U)
#define CSL_IOMUX_QUAL_GRP_8_CFG_REG_PERIOD_PER_SAMPLE_RESETVAL                 (0x00000000U)
#define CSL_IOMUX_QUAL_GRP_8_CFG_REG_PERIOD_PER_SAMPLE_MAX                      (0x000000FFU)


#define CSL_IOMUX_QUAL_GRP_8_CFG_REG_RESERVED_MASK                              (0x80000000U)
#define CSL_IOMUX_QUAL_GRP_8_CFG_REG_RESERVED_SHIFT                             (0x0000001FU)
#define CSL_IOMUX_QUAL_GRP_8_CFG_REG_RESERVED_RESETVAL                          (0x00000000U)
#define CSL_IOMUX_QUAL_GRP_8_CFG_REG_RESERVED_MAX                               (0x00000001U)



/* QUAL_GRP_9_CFG_REG */
#define CSL_IOMUX_QUAL_GRP_9_CFG_REG_PERIOD_PER_SAMPLE_MASK                     (0x000000FFU)
#define CSL_IOMUX_QUAL_GRP_9_CFG_REG_PERIOD_PER_SAMPLE_SHIFT                    (0x00000000U)
#define CSL_IOMUX_QUAL_GRP_9_CFG_REG_PERIOD_PER_SAMPLE_RESETVAL                 (0x00000000U)
#define CSL_IOMUX_QUAL_GRP_9_CFG_REG_PERIOD_PER_SAMPLE_MAX                      (0x000000FFU)


#define CSL_IOMUX_QUAL_GRP_9_CFG_REG_RESERVED_MASK                              (0x80000000U)
#define CSL_IOMUX_QUAL_GRP_9_CFG_REG_RESERVED_SHIFT                             (0x0000001FU)
#define CSL_IOMUX_QUAL_GRP_9_CFG_REG_RESERVED_RESETVAL                          (0x00000000U)
#define CSL_IOMUX_QUAL_GRP_9_CFG_REG_RESERVED_MAX                               (0x00000001U)



/* QUAL_GRP_10_CFG_REG */
#define CSL_IOMUX_QUAL_GRP_10_CFG_REG_PERIOD_PER_SAMPLE_MASK                    (0x000000FFU)
#define CSL_IOMUX_QUAL_GRP_10_CFG_REG_PERIOD_PER_SAMPLE_SHIFT                   (0x00000000U)
#define CSL_IOMUX_QUAL_GRP_10_CFG_REG_PERIOD_PER_SAMPLE_RESETVAL                (0x00000000U)
#define CSL_IOMUX_QUAL_GRP_10_CFG_REG_PERIOD_PER_SAMPLE_MAX                     (0x000000FFU)


#define CSL_IOMUX_QUAL_GRP_10_CFG_REG_RESERVED_MASK                             (0x80000000U)
#define CSL_IOMUX_QUAL_GRP_10_CFG_REG_RESERVED_SHIFT                            (0x0000001FU)
#define CSL_IOMUX_QUAL_GRP_10_CFG_REG_RESERVED_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_QUAL_GRP_10_CFG_REG_RESERVED_MAX                              (0x00000001U)



/* QUAL_GRP_11_CFG_REG */
#define CSL_IOMUX_QUAL_GRP_11_CFG_REG_PERIOD_PER_SAMPLE_MASK                    (0x000000FFU)
#define CSL_IOMUX_QUAL_GRP_11_CFG_REG_PERIOD_PER_SAMPLE_SHIFT                   (0x00000000U)
#define CSL_IOMUX_QUAL_GRP_11_CFG_REG_PERIOD_PER_SAMPLE_RESETVAL                (0x00000000U)
#define CSL_IOMUX_QUAL_GRP_11_CFG_REG_PERIOD_PER_SAMPLE_MAX                     (0x000000FFU)


#define CSL_IOMUX_QUAL_GRP_11_CFG_REG_RESERVED_MASK                             (0x80000000U)
#define CSL_IOMUX_QUAL_GRP_11_CFG_REG_RESERVED_SHIFT                            (0x0000001FU)
#define CSL_IOMUX_QUAL_GRP_11_CFG_REG_RESERVED_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_QUAL_GRP_11_CFG_REG_RESERVED_MAX                              (0x00000001U)



/* QUAL_GRP_12_CFG_REG */
#define CSL_IOMUX_QUAL_GRP_12_CFG_REG_PERIOD_PER_SAMPLE_MASK                    (0x000000FFU)
#define CSL_IOMUX_QUAL_GRP_12_CFG_REG_PERIOD_PER_SAMPLE_SHIFT                   (0x00000000U)
#define CSL_IOMUX_QUAL_GRP_12_CFG_REG_PERIOD_PER_SAMPLE_RESETVAL                (0x00000000U)
#define CSL_IOMUX_QUAL_GRP_12_CFG_REG_PERIOD_PER_SAMPLE_MAX                     (0x000000FFU)


#define CSL_IOMUX_QUAL_GRP_12_CFG_REG_RESERVED_MASK                             (0x80000000U)
#define CSL_IOMUX_QUAL_GRP_12_CFG_REG_RESERVED_SHIFT                            (0x0000001FU)
#define CSL_IOMUX_QUAL_GRP_12_CFG_REG_RESERVED_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_QUAL_GRP_12_CFG_REG_RESERVED_MAX                              (0x00000001U)



/* QUAL_GRP_13_CFG_REG */
#define CSL_IOMUX_QUAL_GRP_13_CFG_REG_PERIOD_PER_SAMPLE_MASK                    (0x000000FFU)
#define CSL_IOMUX_QUAL_GRP_13_CFG_REG_PERIOD_PER_SAMPLE_SHIFT                   (0x00000000U)
#define CSL_IOMUX_QUAL_GRP_13_CFG_REG_PERIOD_PER_SAMPLE_RESETVAL                (0x00000000U)
#define CSL_IOMUX_QUAL_GRP_13_CFG_REG_PERIOD_PER_SAMPLE_MAX                     (0x000000FFU)


#define CSL_IOMUX_QUAL_GRP_13_CFG_REG_RESERVED_MASK                             (0x80000000U)
#define CSL_IOMUX_QUAL_GRP_13_CFG_REG_RESERVED_SHIFT                            (0x0000001FU)
#define CSL_IOMUX_QUAL_GRP_13_CFG_REG_RESERVED_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_QUAL_GRP_13_CFG_REG_RESERVED_MAX                              (0x00000001U)



/* QUAL_GRP_14_CFG_REG */
#define CSL_IOMUX_QUAL_GRP_14_CFG_REG_PERIOD_PER_SAMPLE_MASK                    (0x000000FFU)
#define CSL_IOMUX_QUAL_GRP_14_CFG_REG_PERIOD_PER_SAMPLE_SHIFT                   (0x00000000U)
#define CSL_IOMUX_QUAL_GRP_14_CFG_REG_PERIOD_PER_SAMPLE_RESETVAL                (0x00000000U)
#define CSL_IOMUX_QUAL_GRP_14_CFG_REG_PERIOD_PER_SAMPLE_MAX                     (0x000000FFU)


#define CSL_IOMUX_QUAL_GRP_14_CFG_REG_RESERVED_MASK                             (0x80000000U)
#define CSL_IOMUX_QUAL_GRP_14_CFG_REG_RESERVED_SHIFT                            (0x0000001FU)
#define CSL_IOMUX_QUAL_GRP_14_CFG_REG_RESERVED_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_QUAL_GRP_14_CFG_REG_RESERVED_MAX                              (0x00000001U)



/* QUAL_GRP_15_CFG_REG */
#define CSL_IOMUX_QUAL_GRP_15_CFG_REG_PERIOD_PER_SAMPLE_MASK                    (0x000000FFU)
#define CSL_IOMUX_QUAL_GRP_15_CFG_REG_PERIOD_PER_SAMPLE_SHIFT                   (0x00000000U)
#define CSL_IOMUX_QUAL_GRP_15_CFG_REG_PERIOD_PER_SAMPLE_RESETVAL                (0x00000000U)
#define CSL_IOMUX_QUAL_GRP_15_CFG_REG_PERIOD_PER_SAMPLE_MAX                     (0x000000FFU)


#define CSL_IOMUX_QUAL_GRP_15_CFG_REG_RESERVED_MASK                             (0x80000000U)
#define CSL_IOMUX_QUAL_GRP_15_CFG_REG_RESERVED_SHIFT                            (0x0000001FU)
#define CSL_IOMUX_QUAL_GRP_15_CFG_REG_RESERVED_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_QUAL_GRP_15_CFG_REG_RESERVED_MAX                              (0x00000001U)



/* QUAL_GRP_16_CFG_REG */
#define CSL_IOMUX_QUAL_GRP_16_CFG_REG_PERIOD_PER_SAMPLE_MASK                    (0x000000FFU)
#define CSL_IOMUX_QUAL_GRP_16_CFG_REG_PERIOD_PER_SAMPLE_SHIFT                   (0x00000000U)
#define CSL_IOMUX_QUAL_GRP_16_CFG_REG_PERIOD_PER_SAMPLE_RESETVAL                (0x00000000U)
#define CSL_IOMUX_QUAL_GRP_16_CFG_REG_PERIOD_PER_SAMPLE_MAX                     (0x000000FFU)


#define CSL_IOMUX_QUAL_GRP_16_CFG_REG_RESERVED_MASK                             (0x80000000U)
#define CSL_IOMUX_QUAL_GRP_16_CFG_REG_RESERVED_SHIFT                            (0x0000001FU)
#define CSL_IOMUX_QUAL_GRP_16_CFG_REG_RESERVED_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_QUAL_GRP_16_CFG_REG_RESERVED_MAX                              (0x00000001U)



/* QUAL_GRP_17_CFG_REG */
#define CSL_IOMUX_QUAL_GRP_17_CFG_REG_PERIOD_PER_SAMPLE_MASK                    (0x000000FFU)
#define CSL_IOMUX_QUAL_GRP_17_CFG_REG_PERIOD_PER_SAMPLE_SHIFT                   (0x00000000U)
#define CSL_IOMUX_QUAL_GRP_17_CFG_REG_PERIOD_PER_SAMPLE_RESETVAL                (0x00000000U)
#define CSL_IOMUX_QUAL_GRP_17_CFG_REG_PERIOD_PER_SAMPLE_MAX                     (0x000000FFU)


#define CSL_IOMUX_QUAL_GRP_17_CFG_REG_RESERVED_MASK                             (0x80000000U)
#define CSL_IOMUX_QUAL_GRP_17_CFG_REG_RESERVED_SHIFT                            (0x0000001FU)
#define CSL_IOMUX_QUAL_GRP_17_CFG_REG_RESERVED_RESETVAL                         (0x00000000U)
#define CSL_IOMUX_QUAL_GRP_17_CFG_REG_RESERVED_MAX                              (0x00000001U)



/* USER_MODE_EN */
#define CSL_IOMUX_USER_MODE_EN_MASK                                             (0xFFFFFFFFU)
#define CSL_IOMUX_USER_MODE_EN_SHIFT                                            (0x00000000U)
#define CSL_IOMUX_USER_MODE_EN_RESETVAL                                         (0x00000000U)
#define CSL_IOMUX_USER_MODE_EN_MAX                                              (0xFFFFFFFFU)



/* PADGLBL_CFG_REG */
#define CSL_IOMUX_PADGLBL_CFG_REG_MASK                                          (0xFFFFFFFFU)
#define CSL_IOMUX_PADGLBL_CFG_REG_SHIFT                                         (0x00000000U)
#define CSL_IOMUX_PADGLBL_CFG_REG_RESETVAL                                      (0x00000000U)
#define CSL_IOMUX_PADGLBL_CFG_REG_MAX                                           (0xFFFFFFFFU)



/* IO_CFG_KICK0 */
#define CSL_IOMUX_IO_CFG_KICK0_MASK                                             (0xFFFFFFFFU)
#define CSL_IOMUX_IO_CFG_KICK0_SHIFT                                            (0x00000000U)
#define CSL_IOMUX_IO_CFG_KICK0_RESETVAL                                         (0x00000000U)
#define CSL_IOMUX_IO_CFG_KICK0_MAX                                              (0xFFFFFFFFU)



/* IO_CFG_KICK1 */
#define CSL_IOMUX_IO_CFG_KICK1_MASK                                             (0xFFFFFFFFU)
#define CSL_IOMUX_IO_CFG_KICK1_SHIFT                                            (0x00000000U)
#define CSL_IOMUX_IO_CFG_KICK1_RESETVAL                                         (0x000000C1U)
#define CSL_IOMUX_IO_CFG_KICK1_MAX                                              (0xFFFFFFFFU)



#ifdef __cplusplus
}
#endif
#endif