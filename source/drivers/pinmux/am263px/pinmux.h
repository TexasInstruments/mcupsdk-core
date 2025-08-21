/*
 * Copyright (C) 2021 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of
 * its contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 *  \defgroup DRV_PINMUX_MODULE APIs for PINMUX
 *  \ingroup DRV_MODULE
 *
 *  This module contains APIs to program and use the pinmux module.
 *
 *  @{
 */

/**
 *  \file pinmux.h
 *
 *  \brief PINMUX Driver API/interface file.
 *
 * \brief  This file contains pad configure register offsets and bit-field
 *         value macros for different configurations,
 *
 *           BIT[20]        INVERT          Invert input (0 = Non Inverted / 1 =  Inverted)
 *           BIT[19:18]     QUAL SEL        Select the input qualifier type
 *                                          (00 : SYNC, 01 : 3 SAMPLE, 10 : 6 SAMPLE, 11 : ASYNC)
 *           BIT[17:16]     GPIO SEL        Select the CPU ownership for GPIO pin
 *                                          (00 : GPIO0, 01 : GPIO1, 10 : GPIO2, 11 : GPIO3)
 *           BIT[10]        SLEWRATE        Set slew rate (0 = High / 1 =  Low)
 *           BIT[9]         PULLTYPESEL     Set the iternal resistor pull direction high or low (if enabled)
 *           BIT[8]         PULLUDEN        Internal resistor disable (0 = enabled / 1 = disabled)
 *           BIT[7:4]       OVERRIDE        Override the default input and output driver from IP
 *           BIT[3:0]       MUXMODE         select the desired function on the given pin
 */

#ifndef PINMUX_AM263PX_H_
#define PINMUX_AM263PX_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/**
 *  \anchor Pinmux_DomainId_t
 *  \name Pinmux Domain ID
 *  @{
 */
#define PINMUX_DOMAIN_ID_MAIN          (0U)
/** @} */

/** \brief Macro to mark end of pinmux config array */
#define PINMUX_END                      (-1)
/** \brief Pin mode - it is at 0th bit. No shift requried */
#define PIN_MODE(mode)                  ((uint32_t) mode)

/** \brief Override IP default to enable input */
#define PIN_FORCE_INPUT_ENABLE          (((uint32_t) 0x1U) << 4U)
/** \brief Override IP default to disable input */
#define PIN_FORCE_INPUT_DISABLE         (((uint32_t) 0x3U) << 4U)
/** \brief Override IP default to enable output */
#define PIN_FORCE_OUTPUT_ENABLE         (((uint32_t) 0x1U) << 6U)
/** \brief Override IP default to disable output */
#define PIN_FORCE_OUTPUT_DISABLE        (((uint32_t) 0x3U) << 6U)

/** \brief Resistor enable */
#define PIN_PULL_DISABLE                (((uint32_t) 0x1U) << 8U)
/** \brief Pull Up */
#define PIN_PULL_UP                     (((uint32_t) 0x1U) << 9U)
/** \brief Pull Down */
#define PIN_PULL_DOWN                   (((uint32_t) 0x0U) << 9U)

/** \brief Slew Rate High */
#define PIN_SLEW_RATE_HIGH              (((uint32_t) 0x0U) << 10U)
/** \brief Slew Rate Low */
#define PIN_SLEW_RATE_LOW               (((uint32_t) 0x1U) << 10U)

/** \brief GPIO Pin CPU ownership - R5SS0_0 */
#define PIN_GPIO_R5SS0_0                (((uint32_t) 0x0U) << 16U)
/** \brief GPIO Pin CPU ownership - R5SS0_1 */
#define PIN_GPIO_R5SS0_1                (((uint32_t) 0x1U) << 16U)
/** \brief GPIO Pin CPU ownership - R5SS1_0 */
#define PIN_GPIO_R5SS1_0                (((uint32_t) 0x2U) << 16U)
/** \brief GPIO Pin CPU ownership - R5SS1_1 */
#define PIN_GPIO_R5SS1_1                (((uint32_t) 0x3U) << 16U)

/** \brief Maximum possible index for GPIO Qual group */
#define PIN_QUAL_GRP_MAX_INDEX          (17U)                              
/** \brief Qual Group config register offset in control module */
#define PIN_QUAL_GROUP_CFG_START        (CSL_IOMUX_QUAL_GRP_0_CFG_REG)  

/** \brief Pin Qualifier - SYNC */
#define PIN_QUAL_SYNC                   (((uint32_t) 0x0U) << 18U)
/** \brief Pin Qualifier - 3 SAMPLE */
#define PIN_QUAL_3SAMPLE                (((uint32_t) 0x1U) << 18U)
/** \brief Pin Qualifier - 6 SAMPLE */
#define PIN_QUAL_6SAMPLE                (((uint32_t) 0x2U) << 18U)
/** \brief Pin Qualifier - ASYNC */
#define PIN_QUAL_ASYNC                  (((uint32_t) 0x3U) << 18U)

/** \brief Pin Invert */
#define PIN_INVERT                      (((uint32_t) 0x1U) << 20U)
/** \brief Pin Non Invert */
#define PIN_NON_INVERT                  (((uint32_t) 0x0U) << 20U)


/**
 *  \anchor Pinmux_Offsets
 *  \name Pad config register offset in control module
 *  @{
 */

#define PIN_OSPI0_CSN0                                          (0x00000000U)
#define PIN_OSPI0_CSN1                                          (0x00000004U)
#define PIN_OSPI0_CLK                                           (0x00000008U)
#define PIN_OSPI0_D0                                            (0x0000000CU)
#define PIN_OSPI0_D1                                            (0x00000010U)
#define PIN_OSPI0_D2                                            (0x00000014U)
#define PIN_OSPI0_D3                                            (0x00000018U)
#define PIN_MCAN0_RX                                            (0x0000001CU)
#define PIN_MCAN0_TX                                            (0x00000020U)
#define PIN_MCAN1_RX                                            (0x00000024U)
#define PIN_MCAN1_TX                                            (0x00000028U)
#define PIN_SPI0_CS0                                            (0x0000002CU)
#define PIN_SPI0_CLK                                            (0x00000030U)
#define PIN_SPI0_D0                                             (0x00000034U)
#define PIN_SPI0_D1                                             (0x00000038U)
#define PIN_SPI1_CS0                                            (0x0000003CU)
#define PIN_SPI1_CLK                                            (0x00000040U)
#define PIN_SPI1_D0                                             (0x00000044U)
#define PIN_SPI1_D1                                             (0x00000048U)
#define PIN_LIN1_RXD                                            (0x0000004CU)
#define PIN_LIN1_TXD                                            (0x00000050U)
#define PIN_LIN2_RXD                                            (0x00000054U)
#define PIN_LIN2_TXD                                            (0x00000058U)
#define PIN_I2C1_SCL                                            (0x0000005CU)
#define PIN_I2C1_SDA                                            (0x00000060U)
#define PIN_UART0_RTSN                                          (0x00000064U)
#define PIN_UART0_CTSN                                          (0x00000068U)
#define PIN_UART0_RXD                                           (0x0000006CU)
#define PIN_UART0_TXD                                           (0x00000070U)
#define PIN_RGMII1_RXC                                          (0x00000074U)
#define PIN_RGMII1_RX_CTL                                       (0x00000078U)
#define PIN_RGMII1_RD0                                          (0x0000007CU)
#define PIN_RGMII1_RD1                                          (0x00000080U)
#define PIN_RGMII1_RD2                                          (0x00000084U)
#define PIN_RGMII1_RD3                                          (0x00000088U)
#define PIN_RGMII1_TXC                                          (0x0000008CU)
#define PIN_RGMII1_TX_CTL                                       (0x00000090U)
#define PIN_RGMII1_TD0                                          (0x00000094U)
#define PIN_RGMII1_TD1                                          (0x00000098U)
#define PIN_RGMII1_TD2                                          (0x0000009CU)
#define PIN_RGMII1_TD3                                          (0x000000A0U)
#define PIN_MDIO_MDIO                                           (0x000000A4U)
#define PIN_MDIO_MDC                                            (0x000000A8U)
#define PIN_EPWM0_A                                             (0x000000ACU)
#define PIN_EPWM0_B                                             (0x000000B0U)
#define PIN_EPWM1_A                                             (0x000000B4U)
#define PIN_EPWM1_B                                             (0x000000B8U)
#define PIN_EPWM2_A                                             (0x000000BCU)
#define PIN_EPWM2_B                                             (0x000000C0U)
#define PIN_EPWM3_A                                             (0x000000C4U)
#define PIN_EPWM3_B                                             (0x000000C8U)
#define PIN_EPWM4_A                                             (0x000000CCU)
#define PIN_EPWM4_B                                             (0x000000D0U)
#define PIN_EPWM5_A                                             (0x000000D4U)
#define PIN_EPWM5_B                                             (0x000000D8U)
#define PIN_EPWM6_A                                             (0x000000DCU)
#define PIN_EPWM6_B                                             (0x000000E0U)
#define PIN_EPWM7_A                                             (0x000000E4U)
#define PIN_EPWM7_B                                             (0x000000E8U)
#define PIN_EPWM8_A                                             (0x000000ECU)
#define PIN_EPWM8_B                                             (0x000000F0U)
#define PIN_EPWM9_A                                             (0x000000F4U)
#define PIN_EPWM9_B                                             (0x000000F8U)
#define PIN_EPWM10_A                                            (0x000000FCU)
#define PIN_EPWM10_B                                            (0x00000100U)
#define PIN_EPWM11_A                                            (0x00000104U)
#define PIN_EPWM11_B                                            (0x00000108U)
#define PIN_EPWM12_A                                            (0x0000010CU)
#define PIN_EPWM12_B                                            (0x00000110U)
#define PIN_EPWM13_A                                            (0x00000114U)
#define PIN_EPWM13_B                                            (0x00000118U)
#define PIN_EPWM14_A                                            (0x0000011CU)
#define PIN_EPWM14_B                                            (0x00000120U)
#define PIN_EPWM15_A                                            (0x00000124U)
#define PIN_EPWM15_B                                            (0x00000128U)
#define PIN_UART1_RXD                                           (0x0000012CU)
#define PIN_UART1_TXD                                           (0x00000130U)
#define PIN_MMC_CLK                                             (0x00000134U)
#define PIN_MMC_CMD                                             (0x00000138U)
#define PIN_MMC_DAT0                                            (0x0000013CU)
#define PIN_MMC_DAT1                                            (0x00000140U)
#define PIN_MMC_DAT2                                            (0x00000144U)
#define PIN_MMC_DAT3                                            (0x00000148U)
#define PIN_MMC_SDWP                                            (0x0000014CU)
#define PIN_MMC_SDCD                                            (0x00000150U)
#define PIN_PR0_MDIO_MDIO                                       (0x00000154U)
#define PIN_PR0_MDIO_MDC                                        (0x00000158U)
#define PIN_PR0_PRU0_GPIO5                                      (0x0000015CU)
#define PIN_PR0_PRU0_GPIO9                                      (0x00000160U)
#define PIN_PR0_PRU0_GPIO10                                     (0x00000164U)
#define PIN_PR0_PRU0_GPIO8                                      (0x00000168U)
#define PIN_PR0_PRU0_GPIO6                                      (0x0000016CU)
#define PIN_PR0_PRU0_GPIO4                                      (0x00000170U)
#define PIN_PR0_PRU0_GPIO0                                      (0x00000174U)
#define PIN_PR0_PRU0_GPIO1                                      (0x00000178U)
#define PIN_PR0_PRU0_GPIO2                                      (0x0000017CU)
#define PIN_PR0_PRU0_GPIO3                                      (0x00000180U)
#define PIN_PR0_PRU0_GPIO16                                     (0x00000184U)
#define PIN_PR0_PRU0_GPIO15                                     (0x00000188U)
#define PIN_PR0_PRU0_GPIO11                                     (0x0000018CU)
#define PIN_PR0_PRU0_GPIO12                                     (0x00000190U)
#define PIN_PR0_PRU0_GPIO13                                     (0x00000194U)
#define PIN_PR0_PRU0_GPIO14                                     (0x00000198U)
#define PIN_PR0_PRU1_GPIO5                                      (0x0000019CU)
#define PIN_PR0_PRU1_GPIO9                                      (0x000001A0U)
#define PIN_PR0_PRU1_GPIO10                                     (0x000001A4U)
#define PIN_PR0_PRU1_GPIO8                                      (0x000001A8U)
#define PIN_PR0_PRU1_GPIO6                                      (0x000001ACU)
#define PIN_PR0_PRU1_GPIO4                                      (0x000001B0U)
#define PIN_PR0_PRU1_GPIO0                                      (0x000001B4U)
#define PIN_PR0_PRU1_GPIO1                                      (0x000001B8U)
#define PIN_PR0_PRU1_GPIO2                                      (0x000001BCU)
#define PIN_PR0_PRU1_GPIO3                                      (0x000001C0U)
#define PIN_PR0_PRU1_GPIO16                                     (0x000001C4U)
#define PIN_PR0_PRU1_GPIO15                                     (0x000001C8U)
#define PIN_PR0_PRU1_GPIO11                                     (0x000001CCU)
#define PIN_PR0_PRU1_GPIO12                                     (0x000001D0U)
#define PIN_PR0_PRU1_GPIO13                                     (0x000001D4U)
#define PIN_PR0_PRU1_GPIO14                                     (0x000001D8U)
#define PIN_PR0_PRU1_GPIO19                                     (0x000001DCU)
#define PIN_PR0_PRU1_GPIO18                                     (0x000001E0U)
#define PIN_EXT_REFCLK0                                         (0x000001E4U)
#define PIN_SDFM0_CLK0                                          (0x000001E8U)
#define PIN_SDFM0_D0                                            (0x000001ECU)
#define PIN_SDFM0_CLK1                                          (0x000001F0U)
#define PIN_SDFM0_D1                                            (0x000001F4U)
#define PIN_SDFM0_CLK2                                          (0x000001F8U)
#define PIN_SDFM0_D2                                            (0x000001FCU)
#define PIN_SDFM0_CLK3                                          (0x00000200U)
#define PIN_SDFM0_D3                                            (0x00000204U)
#define PIN_EQEP0_A                                             (0x00000208U)
#define PIN_EQEP0_B                                             (0x0000020CU)
#define PIN_EQEP0_STROBE                                        (0x00000210U)
#define PIN_EQEP0_INDEX                                         (0x00000214U)
#define PIN_I2C0_SDA                                            (0x00000218U)
#define PIN_I2C0_SCL                                            (0x0000021CU)
#define PIN_MCAN2_TX                                            (0x00000220U)
#define PIN_MCAN2_RX                                            (0x00000224U)
#define PIN_CLKOUT0                                             (0x00000228U)
#define PIN_RESET_REQZ                                          (0x0000022CU)
#define PIN_SAFETY_ERRORN                                       (0x00000230U)
#define PIN_TDI                                                 (0x00000234U)
#define PIN_TDO                                                 (0x00000238U)
#define PIN_TMS                                                 (0x0000023CU)
#define PIN_TCK                                                 (0x00000240U)
#define PIN_QSPI_CLKLB                                          (0x00000244U)

/** @} */

/**
 *  \anchor Pinmux_Offsets_SIP
 *  \name Pad config register offset in control module for SIP package
 *  @{
 */

#define PIN_GPIO0                                               (0x00000000U)
#define PIN_GPIO1                                               (0x00000004U)
#define PIN_GPIO2                                               (0x00000008U)
#define PIN_GPIO3                                               (0x0000000CU)
#define PIN_GPIO4                                               (0x00000010U)
#define PIN_GPIO5                                               (0x00000014U)
#define PIN_GPIO7                                               (0x0000001CU)
#define PIN_GPIO8                                               (0x00000020U)
#define PIN_GPIO9                                               (0x00000024U)
#define PIN_GPIO10                                              (0x00000028U)
#define PIN_GPIO6                                               (0x00000018U)
#define PIN_GPIO64                                              (0x00000100U)
#define PIN_GPIO65                                              (0x00000104U)
#define PIN_GPIO66                                              (0x00000108U)
#define PIN_GPIO67                                              (0x0000010CU)
#define PIN_GPIO68                                              (0x00000110U)
#define PIN_GPIO69                                              (0x00000114U)
#define PIN_GPIO70                                              (0x00000118U)
#define PIN_GPIO75                                              (0x0000012CU)
#define PIN_GPIO76                                              (0x00000130U)
#define PIN_GPIO19                                              (0x0000004CU)
#define PIN_GPIO20                                              (0x00000050U)

/** @} */

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/** \brief Structure defining the pin configuration parameters */
typedef struct Pinmux_PerCfg
{
    int16_t     offset;
    /**< Register offset for configuring the pin.
     *   Refer \ref Pinmux_Offsets.
     *   Set this to #PINMUX_END to demark the end of configuration array */
    uint32_t    settings;
    /**< Value to be configured.
     *   Active mode configurations like mux mode, pull resistor and Slew rate.
     *
     *   To set a value use like   : "| PIN_PULL_DISABLE"
     *   To reset a value use like : "& (~PIN_PULL_DISABLE)"
     *
     *   For example,
     *   PIN_MODE(7) | ((PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW))
     */
} Pinmux_PerCfg_t;

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 *  \brief  This API configures the pinmux based on the domain
 *
 *  \param  pinmuxCfg   Pointer to list of pinmux configuration array.
 *                      This parameter cannot be NULL and the last entry should
 *                      be initialized with #PINMUX_END so that this function
 *                      knows the end of configuration.
 *  \param  domainId    Domain ID to set pinmux configuration.
 *                      Refer \ref Pinmux_DomainId_t
 */
void Pinmux_config(const Pinmux_PerCfg_t *pinmuxCfg, uint32_t domainId);

/**
 *  \brief  This API configures the Qualification period for a specific GPIO Pin group.
 *
 *  \param  qualGroupIndex   Index of GPIO Pin group for which the Period is being configured.
 *                           Range is from 0-17. 

 *  \param  qualPeriod       Qual Period value for the Pin group.
 */
void Pinmux_qualPeriodConfig(uint32_t qualGroupIndex, uint8_t qualPeriod);

/**
 *  \brief  This API enables user mode access to the IOMUX register space.
 */
void Pinmux_enableUserMode(void);

/**
 *  \brief  This API disables user mode access to the IOMUX register space.
 */
void Pinmux_disableUserMode(void);

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
}
#endif

#endif  /* #ifndef PINMUX_AM64X_H_ */

/** @} */