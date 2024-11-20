/*
 *  Copyright (C) 2021 Texas Instruments Incorporated
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
 */

#ifndef ETHPHY_DP83869_H_
#define ETHPHY_DP83869_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <board/ethphy.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/**
 *  \anchor ETHPHY_DP83869_LEDS
 *  \name ETHPHY DP83869 LEDS
 *
 *  Different LEDs present in ETHPHY DP83869
 *
 *  @{
 */
#define ETHPHY_DP83869_LED0                                     (0x0)
#define ETHPHY_DP83869_LED1                                     (0x1)
#define ETHPHY_DP83869_LED2                                     (0x2)
#define ETHPHY_DP83869_LED_GPIO                                 (0x3)
/** @} */

/**
 *  \anchor ETHPHY_DP83869_LED_MODES
 *  \name ETHPHY DP83869 LED MODES
 *
 *  Different modes for LED present in ETHPHY DP83869
 *
 *  @{
 */
#define ETHPHY_DP83869_LED_MODE_LINK_OK                         (0x0)
#define ETHPHY_DP83869_LED_MODE_RX_TX_ACTIVITY                  (0x1)
#define ETHPHY_DP83869_LED_MODE_TX_ACTIVITY                     (0x2)
#define ETHPHY_DP83869_LED_MODE_RX_ACTIVITY                     (0x3)
#define ETHPHY_DP83869_LED_MODE_COLLISION_DETECTED              (0x4)
#define ETHPHY_DP83869_LED_MODE_1000BT_LINK_UP                  (0x5)
#define ETHPHY_DP83869_LED_MODE_100BTX_LINK_UP                  (0x6)
#define ETHPHY_DP83869_LED_MODE_10BT_LINK_UP                    (0x7)
#define ETHPHY_DP83869_LED_MODE_10_OR_100BT_LINK_UP             (0x8)
#define ETHPHY_DP83869_LED_MODE_100_OR_1000BT_LINK_UP           (0x9)
#define ETHPHY_DP83869_LED_MODE_FULL_DUPLEX                     (0xA)
#define ETHPHY_DP83869_LED_MODE_LINK_OK_AND_BLINK_ON_RX_TX      (0xB)
#define ETHPHY_DP83869_LED_MODE_RX_ERROR_OR_TX_ERROR            (0xD)
#define ETHPHY_DP83869_LED_MODE_RX_ERROR                        (0xE)
/** @} */

/**
 *  \anchor ETHPHY_DP83869_LED_BLINK_RATES
 *  \name ETHPHY DP83869 LED BLINK RATES
 *
 *  Different blink rates for LED present in ETHPHY DP83869
 *
 *  @{
 */
#define ETHPHY_DP83869_LED_BLINK_RATE_50_MS                     (0x0)
#define ETHPHY_DP83869_LED_BLINK_RATE_100_MS                    (0x1)
#define ETHPHY_DP83869_LED_BLINK_RATE_200_MS                    (0x2)
#define ETHPHY_DP83869_LED_BLINK_RATE_500_MS                    (0x3)
/** @} */

/**
 *  \anchor ETHPHY_DP83869_FAST_LINKDOWN_MODES
 *  \name ETHPHY DP83869 FAST LINKDOWN MODES
 *
 *  Different modes for fast link down detection in ETHPHY DP83869
 *
 *  @{
 */
#define ETHPHY_DP83869_FAST_LINKDOWN_MODE_ENERGY_LOST           (1u<<0)
#define ETHPHY_DP83869_FAST_LINKDOWN_MODE_MSE                   (1u<<1)
#define ETHPHY_DP83869_FAST_LINKDOWN_MODE_MLT3_ERRORS           (1u<<2)
#define ETHPHY_DP83869_FAST_LINKDOWN_MODE_RX_ERR                (1u<<3)
#define ETHPHY_DP83869_FAST_LINKDOWN_MODE_DESCRAMBLER_SYNC_LOSS (1u<<4)
/** @} */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 * \brief Data structure to ETHPHY_DP83869 Object
 */
typedef struct ETHPHY_DP83869_Object_s {
    uint32_t reserved; /**< reserved for future use */
} ETHPHY_DP83869_Object;

/**
 * \brief Data structure to be passed when calling \ref ETHPHY_command with
 *        \ref ETHPHY_CMD_CONFIGURE_LED_SOURCE for DP83869 PHY
 */
typedef struct ETHPHY_DP83869_LedSourceConfig_s
{
    uint32_t ledNum; /**< LED number from \ref ETHPHY_DP83869_LEDS */
    uint32_t mode;   /**< LED mode from \ref ETHPHY_DP83869_LED_MODES */
} ETHPHY_DP83869_LedSourceConfig;

/**
 * \brief Data structure to be passed when calling \ref ETHPHY_command with
 *        \ref ETHPHY_CMD_CONFIGURE_LED_BLINK_RATE for DP83869 PHY
 */
typedef struct ETHPHY_DP83869_LedBlinkRateConfig_s
{
    uint32_t rate; /**< LED Blink Rate. Allowed values are
                        \ref ETHPHY_DP83869_LED_BLINK_RATES */
} ETHPHY_DP83869_LedBlinkRateConfig;

/**
 * \brief Data structure to be passed when calling \ref ETHPHY_command with
 *        \ref ETHPHY_CMD_ENABLE_FAST_LINK_DOWN_DETECTION for DP83869 PHY
 */
typedef struct ETHPHY_DP83869_FastLinkDownDetectionConfig_s
{
    uint32_t mode; /**< Fast link down detection mode. One or more mode can
                        be passed from \ref ETHPHY_DP83869_FAST_LINKDOWN_MODES.
                        If multiple modes are used, the value of this parameter
                        should be Bitwise OR of individual modes. */
} ETHPHY_DP83869_FastLinkDownDetectionConfig;

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

extern ETHPHY_Fxns gEthPhyFxns_DP83869;

#ifdef __cplusplus
}
#endif

#endif /* #ifndef ETHPHY_DP83869_H_ */

/** @} */
