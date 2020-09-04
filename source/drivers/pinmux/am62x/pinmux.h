/*
 * Copyright (C) 2022 Texas Instruments Incorporated
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
 *           BIT[21]        TXDISABLE       disable the pin's output driver
 *           BIT[18]        RXACTIVE        enable the pin's input buffer (typically kept enabled)
 *           BIT[17]        PULLTYPESEL     set the iternal resistor pull direction high or low (if enabled)
 *           BIT[16]        PULLUDEN        internal resistor disable (0 = enabled / 1 = disabled)
 *           BIT[3:0]       MUXMODE         select the desired function on the given pin
 */

#ifndef PINMUX_AM62X_H_
#define PINMUX_AM62X_H_

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
#define PINMUX_DOMAIN_ID_MCU           (1U)
/** @} */

/** \brief Macro to mark end of pinmux config array */
#define PINMUX_END                      (-1)

/** \brief Pin mode - it is at 0th bit. No shift requried */
#define PIN_MODE(mode)                  ((uint32_t) mode)
/** \brief Resistor enable */
#define PIN_PULL_DISABLE                (((uint32_t) 0x1U) << 16U)
/** \brief Pull direction */
#define PIN_PULL_DIRECTION              (((uint32_t) 0x1U) << 17U)
/** \brief Receiver enable */
#define PIN_INPUT_ENABLE                (((uint32_t) 0x1U) << 18U)
/** \brief Driver disable */
#define PIN_OUTPUT_DISABLE              (((uint32_t) 0x1U) << 21U)
/** \brief Wakeup enable */
#define PIN_WAKEUP_ENABLE               (((uint32_t) 0x1U) << 29U)

/** \brief Main domain pad config register offset in control module */
enum Pinmux_MainOffsets
{
    PIN_MMC1_DAT1		 = 0x022C,
	PIN_MMC1_DAT0		 = 0x0230,
	PIN_EXT_REFCLK1		 = 0x01F0,
	PIN_MMC1_DAT3		 = 0x0224,
	PIN_MMC1_DAT2		 = 0x0228,
	PIN_VOUT0_VSYNC		 = 0x0100,
	PIN_VOUT0_HSYNC		 = 0x00F8,
	PIN_VOUT0_PCLK		 = 0x0104,
	PIN_VOUT0_DE		 = 0x00FC,
	PIN_VOUT0_DATA0		 = 0x00B8,
	PIN_VOUT0_DATA1		 = 0x00BC,
	PIN_VOUT0_DATA2		 = 0x00C0,
	PIN_VOUT0_DATA3		 = 0x00C4,
	PIN_VOUT0_DATA4		 = 0x00C8,
	PIN_VOUT0_DATA5		 = 0x00CC,
	PIN_VOUT0_DATA6		 = 0x00D0,
	PIN_VOUT0_DATA7		 = 0x00D4,
	PIN_VOUT0_DATA8		 = 0x00D8,
	PIN_VOUT0_DATA9		 = 0x00DC,
	PIN_VOUT0_DATA10	 = 0x00E0,
	PIN_VOUT0_DATA11	 = 0x00E4,
	PIN_VOUT0_DATA12	 = 0x00E8,
	PIN_VOUT0_DATA13	 = 0x00EC,
	PIN_VOUT0_DATA14	 = 0x00F0,
	PIN_VOUT0_DATA15	 = 0x00F4,
	PIN_GPMC0_AD8		 = 0x005C,
	PIN_GPMC0_AD9		 = 0x0060,
	PIN_GPMC0_AD10		 = 0x0064,
	PIN_GPMC0_AD11		 = 0x0068,
	PIN_GPMC0_AD12		 = 0x006C,
	PIN_GPMC0_AD13		 = 0x0070,
	PIN_GPMC0_AD14		 = 0x0074,
	PIN_GPMC0_AD15		 = 0x0078,
	PIN_GPMC0_WAIT1		 = 0x009C,
	PIN_UART0_TXD		 = 0x01CC,
	PIN_SPI0_CS0		 = 0x01B4,
	PIN_SPI0_CS1		 = 0x01B8,
	PIN_I2C1_SCL		 = 0x01E8,
	PIN_I2C1_SDA		 = 0x01EC,
	PIN_SPI0_CLK		 = 0x01BC,
	PIN_SPI0_D0		     = 0x01C0,
	PIN_UART0_RXD		 = 0x01C8,
	PIN_SPI0_D1		     = 0x01C4,
	PIN_RGMII2_RD2		 = 0x018C,
	PIN_RGMII2_RD3		 = 0x0190,
	PIN_RGMII2_TD2		 = 0x0174,
	PIN_RGMII2_TD3		 = 0x0178,
	PIN_MCASP0_AXR0		 = 0x01A0,
	PIN_GPMC0_CSN3		 = 0x00B4,
	PIN_GPMC0_WPN		 = 0x00A0,
	PIN_GPMC0_AD0		 = 0x003C,
	PIN_GPMC0_AD1		 = 0x0040,
	PIN_GPMC0_AD2		 = 0x0044,
	PIN_GPMC0_AD3		 = 0x0048,
	PIN_GPMC0_AD4		 = 0x004C,
	PIN_GPMC0_AD5		 = 0x0050,
	PIN_GPMC0_AD6		 = 0x0054,
	PIN_GPMC0_AD7		 = 0x0058,
	PIN_GPMC0_WAIT0		 = 0x0098,
	PIN_GPMC0_BE1N		 = 0x0094,
	PIN_GPMC0_CSN0		 = 0x00A8,
	PIN_GPMC0_CLK		 = 0x007C,
	PIN_GPMC0_ADVN_ALE	 = 0x0084,
	PIN_GPMC0_OEN_REN	 = 0x0088,
	PIN_GPMC0_WEN		 = 0x008C,
	PIN_GPMC0_BE0N_CLE	 = 0x0090,
	PIN_UART0_CTSN		 = 0x01D0,
	PIN_UART0_RTSN		 = 0x01D4,
	PIN_TMS		         = 0x0074,
	PIN_MCASP0_ACLKX	 = 0x01A4,
	PIN_MCASP0_AFSX		 = 0x01A8,
	PIN_MCASP0_AXR1		 = 0x019C,
	PIN_OSPI0_D6		 = 0x0024,
	PIN_OSPI0_D7		 = 0x0028,
	PIN_OSPI0_D5		 = 0x0020,
	PIN_RGMII2_RX_CTL	 = 0x017C,
	PIN_MDIO0_MDC		 = 0x0160,
	PIN_MDIO0_MDIO		 = 0x015C,
	PIN_MMC0_CMD		 = 0x0220,
	PIN_MMC0_CLK		 = 0x0218,
	PIN_MMC0_DAT0		 = 0x0214,
	PIN_MMC0_DAT1		 = 0x0210,
	PIN_MMC0_DAT2		 = 0x020C,
	PIN_MMC0_DAT3		 = 0x0208,
	PIN_MMC0_DAT4		 = 0x0204,
	PIN_MMC0_DAT5		 = 0x0200,
	PIN_MMC0_DAT6		 = 0x01FC,
	PIN_MMC0_DAT7		 = 0x01F8,
	PIN_MMC1_CMD		 = 0x023C,
	PIN_MMC1_CLK		 = 0x0234,
	PIN_MMC1_SDCD		 = 0x0240,
	PIN_MMC2_CMD		 = 0x0120,
	PIN_MMC2_CLK		 = 0x0118,
	PIN_MMC2_DAT0		 = 0x0114,
	PIN_MMC2_DAT1		 = 0x0110,
	PIN_MMC2_DAT2		 = 0x010C,
	PIN_MMC2_DAT3		 = 0x0108,
	PIN_MMC2_SDCD		 = 0x0124,
	PIN_MMC2_SDWP		 = 0x0128,
	PIN_OLDI0_A0N		 = 0x0260,
	PIN_OLDI0_A0P		 = 0x025C,
	PIN_OLDI0_A1N		 = 0x0268,
	PIN_OLDI0_A1P		 = 0x0264,
	PIN_OLDI0_A2N		 = 0x0270,
	PIN_OLDI0_A2P		 = 0x026C,
	PIN_OLDI0_A3N		 = 0x0278,
	PIN_OLDI0_A3P		 = 0x0274,
	PIN_OLDI0_A4N		 = 0x0280,
	PIN_OLDI0_A4P		 = 0x027C,
	PIN_OLDI0_A5N		 = 0x0288,
	PIN_OLDI0_A5P		 = 0x0284,
	PIN_OLDI0_A6N		 = 0x0290,
	PIN_OLDI0_A6P		 = 0x028C,
	PIN_OLDI0_A7N		 = 0x0298,
	PIN_OLDI0_A7P		 = 0x0294,
	PIN_OLDI0_CLK0N		 = 0x02A0,
	PIN_OLDI0_CLK0P		 = 0x029C,
	PIN_OLDI0_CLK1N		 = 0x02A8,
	PIN_OLDI0_CLK1P		 = 0x02A4,
	PIN_OSPI0_CLK		 = 0x0000,
	PIN_OSPI0_CSN0		 = 0x002C,
	PIN_OSPI0_CSN1		 = 0x0030,
	PIN_OSPI0_CSN2		 = 0x0034,
	PIN_OSPI0_CSN3		 = 0x0038,
	PIN_OSPI0_D0		 = 0x000C,
	PIN_OSPI0_D1		 = 0x0010,
	PIN_OSPI0_D2		 = 0x0014,
	PIN_OSPI0_D3		 = 0x0018,
	PIN_OSPI0_D4		 = 0x001C,
	PIN_OSPI0_DQS		 = 0x0008,
	PIN_RGMII2_RXC		 = 0x0180,
	PIN_RGMII2_TX_CTL	 = 0x0164,
	PIN_RGMII2_TXC		 = 0x0168,
	PIN_RGMII2_TD0		 = 0x016C,
	PIN_RGMII2_TD1		 = 0x0170,
	PIN_RGMII1_RXC		 = 0x0148,
	PIN_RGMII2_RD0		 = 0x0184,
	PIN_RGMII1_TD2		 = 0x013C,
	PIN_RGMII1_TD3		 = 0x0140,
	PIN_RGMII1_RD0		 = 0x014C,
	PIN_RGMII1_RD1		 = 0x0150,
	PIN_RGMII1_RD2		 = 0x0154,
	PIN_RGMII1_RD3		 = 0x0158,
	PIN_RGMII1_RX_CTL	 = 0x0144,
	PIN_RGMII1_TD0		 = 0x0134,
	PIN_RGMII1_TD1		 = 0x0138,
	PIN_RGMII1_TXC		 = 0x0130,
	PIN_RGMII1_TX_CTL	 = 0x012C,
	PIN_RGMII2_RD1		 = 0x0188,
	PIN_PORZ_OUT		 = 0x0250,
	PIN_RESETSTATZ		 = 0x024C,
	PIN_MMC1_SDWP		 = 0x0244,
	PIN_GPMC0_DIR		 = 0x00A4,
	PIN_GPMC0_CSN1		 = 0x00AC,
	PIN_GPMC0_CSN2		 = 0x00B0,
	PIN_MCASP0_AXR3		 = 0x0194,
	PIN_I2C0_SCL		 = 0x01E0,
	PIN_I2C0_SDA		 = 0x01E4,
	PIN_MCAN0_TX		 = 0x01D8,
	PIN_MCAN0_RX		 = 0x01DC,
	PIN_MCASP0_AXR2		 = 0x0198,
	PIN_MCASP0_AFSR		 = 0x01AC,
	PIN_MCASP0_ACLKR	 = 0x01B0,
	PIN_OSPI0_LBCLKO	 = 0x0004,
	PIN_USB1_DRVVBUS	 = 0x0258,
	PIN_USB0_DRVVBUS	 = 0x0254,
};

/** \brief Wakeup domain pad config register offset in control module */
enum Pinmux_McuOffsets
{
	PIN_EMU0		 = 0x0078,
	PIN_EMU1		 = 0x007C,
	PIN_TCK		     = 0x0064,
	PIN_TDI		     = 0x006C,
	PIN_TDO		     = 0x0070,
	PIN_TRSTN		 = 0x0068,
	PIN_MCU_SPI0_CS1         = 0x0004,
	PIN_MCU_SPI0_CLK         = 0x0008,
	PIN_MCU_SPI0_D0	         = 0x000C,
	PIN_MCU_SPI0_D1	         = 0x0010,
	PIN_MCU_UART0_RXD        = 0x0014,
	PIN_MCU_UART0_TXD		 = 0x0018,
	PIN_MCU_UART0_CTSN		 = 0x001C,
	PIN_MCU_UART0_RTSN		 = 0x0020,
	PIN_MCU_MCAN0_RX		 = 0x0038,
	PIN_MCU_I2C0_SCL		 = 0x0044,
	PIN_MCU_I2C0_SDA		 = 0x0048,
	PIN_MCU_RESETSTATZ		 = 0x005C,
	PIN_MCU_MCAN1_RX		 = 0x0040,
	PIN_MCU_MCAN1_TX		 = 0x003C,
	PIN_MCU_ERRORN	    	 = 0x0060,
	PIN_MCU_PORZ		     = 0x0058,
	PIN_MCU_RESETZ		     = 0x0054,
	PIN_RESET_REQZ		     = 0x0248,
	PIN_WKUP_I2C0_SCL		 = 0x004C,
	PIN_WKUP_I2C0_SDA		 = 0x0050,
	PIN_PMIC_LPM_EN0		 = 0x0080,
	PIN_WKUP_CLKOUT0		 = 0x0084,
	PIN_MCU_SPI0_CS0		 = 0x0000,
	PIN_MCU_MCAN0_TX		 = 0x0034,
	PIN_WKUP_UART0_CTSN		 = 0x002C,
	PIN_WKUP_UART0_RTSN		 = 0x0030,
	PIN_WKUP_UART0_RXD		 = 0x0024,
	PIN_WKUP_UART0_TXD		 = 0x0028,
};


/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/** \brief Structure defining the pin configuration parameters */
typedef struct Pinmux_PerCfg
{
    int16_t     offset;
    /**< Register offset for configuring the pin.
     *   Refer \ref Pinmux_MainOffsets and \ref Pinmux_McuOffsets.
     *   Set this to #PINMUX_END to demark the end of configuration array */
    uint32_t    settings;
    /**< Value to be configured.
     *   Active mode configurations like mux mode, pull resistor and
     *   buffer mode.
     *
     *   To set a value use like   : "| PIN_PULL_DISABLE"
     *   To reset a value use like : "& (~PIN_PULL_DIRECTION)"
     *
     *   For example,
     *   PIN_MODE(7) | ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION)
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

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
}
#endif

#endif  /* #ifndef PINMUX_AM62X_H_ */

/** @} */



