/*
 *  Copyright (c) Texas Instruments Incorporated 2021
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

/*!
 * \file  dp83869.h
 *
 * \brief This file contains the type definitions and helper macros for the
 *        DP83869 Ethernet PHY.
 */

/*!
 * \ingroup  DRV_ENETPHY
 * \defgroup ENETPHY_DP83869 TI DP83869 PHY
 *
 * TI DP83869 RGMII Ethernet PHY.
 *
 * @{
 */

#ifndef DP83869_H_
#define DP83869_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */

/*! \brief Number of LEDs available in the PHY. */
#define DP83869_LED_NUM                       (4U)

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/*!
 * \brief GPIO_0 mux modes.
 */
typedef enum Dp83869_Gpio0Mode_e
{
    /*! RX_ER */
    DP83869_GPIO0_RXERR     = 0x0U,

    /*! 1588 TX SFD */
    DP83869_GPIO0_1588TXSFD = 0x1U,

    /*! 1588 RX SFD */
    DP83869_GPIO0_1588RXSFD = 0x2U,

    /*! WOL */
    DP83869_GPIO0_WOL       = 0x3U,

    /*! Energy Detect (1000Base-T and 100Base-TX only) */
    DP83869_GPIO0_ENERGYDET = 0x4U,

    /*! LED_3 */
    DP83869_GPIO0_LED3      = 0x6U,

    /*! PRBS Errors / Loss of Sync */
    DP83869_GPIO0_PRBSERR   = 0x7U,

    /*! Constant 0 */
    DP83869_GPIO0_CONSTANT0 = 0x8U,

    /*! Constant 1 */
    DP83869_GPIO0_CONSTANT1 = 0x9U,
} Dp83869_Gpio0Mode;

/*!
 * \brief GPIO_1 mux modes.
 */
typedef enum Dp83869_Gpio1Mode_e
{
    /*! COL */
    DP83869_GPIO1_COL       = 0x0U,

    /*! 1588 TX SFD */
    DP83869_GPIO1_1588TXSFD = 0x1U,

    /*! 1588 RX SFD */
    DP83869_GPIO1_1588RXSFD = 0x2U,

    /*! WOL */
    DP83869_GPIO1_WOL       = 0x3U,

    /*! Energy Detect (1000Base-T and 100Base-TX only) */
    DP83869_GPIO1_ENERGYDET = 0x4U,

    /*! LED_3 */
    DP83869_GPIO1_LED3      = 0x6U,

    /*! PRBS Errors / Loss of Sync */
    DP83869_GPIO1_PRBSERR   = 0x7U,

    /*! Constant 0 */
    DP83869_GPIO1_CONSTANT0 = 0x8U,

    /*! Constant 1 */
    DP83869_GPIO1_CONSTANT1 = 0x9U,
} Dp83869_Gpio1Mode;

/*!
 * \brief LED modes (sources).
 */
typedef enum Dp83869_LedMode_e
{
    /*! Link established */
    DP83869_LED_LINKED           = 0x0U,

    /*! Receive or transmit activity */
    DP83869_LED_RXTXACT          = 0x1U,

    /*! Transmit activity */
    DP83869_LED_TXACT            = 0x2U,

    /*! Receive activity */
    DP83869_LED_RXACT            = 0x3U,

    /*! Collision detected */
    DP83869_LED_COLLDET          = 0x4U,

    /*! 1000BT link established */
    DP83869_LED_LINKED_1000BT    = 0x5U,

    /*! 100 BTX link established */
    DP83869_LED_LINKED_100BTX    = 0x6U,

    /*! 10BT link established */
    DP83869_LED_LINKED_10BT      = 0x7U,

    /*! 10/100BT link established */
    DP83869_LED_LINKED_10100BT   = 0x8U,

    /*! 100/1000BT link established */
    DP83869_LED_LINKED_1001000BT = 0x9U,

    /*! Full duplex */
    DP83869_LED_FULLDUPLEX       = 0xAU,

    /*! Link established, blink for transmit or receive activity */
    DP83869_LED_LINKED_BLINKACT  = 0xBU,

    /*! Receive error or transmit error */
    DP83869_LED_RXTXERR          = 0xDU,

    /*! Receive error */
    DP83869_LED_RXERR            = 0xEU,
} Dp83869_LedMode;

/*!
 * \brief DP83869 PHY configuration parameters.
 */
typedef struct Dp83869_Cfg_s
{
    /*! Enable TX clock shift */
    bool txClkShiftEn;

    /*! Enable RX clock shift */
    bool rxClkShiftEn;

    /*! TX delay value */
    uint32_t txDelayInPs;

    /*! RX delay value */
    uint32_t rxDelayInPs;

    /*! TX FIFO depth */
    uint8_t txFifoDepth;

    /*! Viterbi detector idle count threshold */
    uint32_t idleCntThresh;

    /*! Output impedance in milli-ohms. Ranging from 35 to 70 ohms
     * in steps of 1.129 ohms */
    uint32_t impedanceInMilliOhms;

    /*! Mux mode of the GPIO_0 pin */
    Dp83869_Gpio0Mode gpio0Mode;

    /*! Mux mode of the GPIO_1 pin */
    Dp83869_Gpio1Mode gpio1Mode;

    /*! LED mode (source) configuration */
    Dp83869_LedMode ledMode[DP83869_LED_NUM];
} Dp83869_Cfg;

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/*!
 * \brief Initialize DP83869 PHY specific config params.
 *
 * Initializes the DP83869 PHY specific configuration parameters.
 *
 * \param cfg       DP83869 PHY config structure pointer
 */
void Dp83869ExtPhy_initCfg(Dp83869_Cfg *cfg);

/* ========================================================================== */
/*                        Deprecated Function Declarations                    */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
}
#endif

#endif /* DP83869_H_ */

/*! @} */
