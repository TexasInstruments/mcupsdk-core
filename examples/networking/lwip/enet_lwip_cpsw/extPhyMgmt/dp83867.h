/*
 *  Copyright (c) Texas Instruments Incorporated 2020
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
 * \file  dp83867.h
 *
 * \brief This file contains the type definitions and helper macros for the
 *        DP83867 Ethernet PHY.
 */

/*!
 * \ingroup  DRV_ENETPHY
 * \defgroup ENETPHY_DP83867 TI DP83867 PHY
 *
 * TI DP83867 RGMII Ethernet PHY.
 *
 * @{
 */

#ifndef DP83867_H_
#define DP83867_H_

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
#define DP83867_LED_NUM                       (4U)

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/*!
 * \brief GPIO_0 mux modes.
 */
typedef enum Dp83867_Gpio0Mode_e
{
    /*! RX_ER */
    DP83867_GPIO0_RXERR     = 0x0U,

    /*! 1588 TX SFD */
    DP83867_GPIO0_1588TXSFD = 0x1U,

    /*! 1588 RX SFD */
    DP83867_GPIO0_1588RXSFD = 0x2U,

    /*! WOL */
    DP83867_GPIO0_WOL       = 0x3U,

    /*! Energy Detect (1000Base-T and 100Base-TX only) */
    DP83867_GPIO0_ENERGYDET = 0x4U,

    /*! LED_3 */
    DP83867_GPIO0_LED3      = 0x6U,

    /*! PRBS Errors / Loss of Sync */
    DP83867_GPIO0_PRBSERR   = 0x7U,

    /*! Constant 0 */
    DP83867_GPIO0_CONSTANT0 = 0x8U,

    /*! Constant 1 */
    DP83867_GPIO0_CONSTANT1 = 0x9U,
} Dp83867_Gpio0Mode;

/*!
 * \brief GPIO_1 mux modes.
 */
typedef enum Dp83867_Gpio1Mode_e
{
    /*! COL */
    DP83867_GPIO1_COL       = 0x0U,

    /*! 1588 TX SFD */
    DP83867_GPIO1_1588TXSFD = 0x1U,

    /*! 1588 RX SFD */
    DP83867_GPIO1_1588RXSFD = 0x2U,

    /*! WOL */
    DP83867_GPIO1_WOL       = 0x3U,

    /*! Energy Detect (1000Base-T and 100Base-TX only) */
    DP83867_GPIO1_ENERGYDET = 0x4U,

    /*! LED_3 */
    DP83867_GPIO1_LED3      = 0x6U,

    /*! PRBS Errors / Loss of Sync */
    DP83867_GPIO1_PRBSERR   = 0x7U,

    /*! Constant 0 */
    DP83867_GPIO1_CONSTANT0 = 0x8U,

    /*! Constant 1 */
    DP83867_GPIO1_CONSTANT1 = 0x9U,
} Dp83867_Gpio1Mode;

/*!
 * \brief LED modes (sources).
 */
typedef enum Dp83867_LedMode_e
{
    /*! Link established */
    DP83867_LED_LINKED           = 0x0U,

    /*! Receive or transmit activity */
    DP83867_LED_RXTXACT          = 0x1U,

    /*! Transmit activity */
    DP83867_LED_TXACT            = 0x2U,

    /*! Receive activity */
    DP83867_LED_RXACT            = 0x3U,

    /*! Collision detected */
    DP83867_LED_COLLDET          = 0x4U,

    /*! 1000BT link established */
    DP83867_LED_LINKED_1000BT    = 0x5U,

    /*! 100 BTX link established */
    DP83867_LED_LINKED_100BTX    = 0x6U,

    /*! 10BT link established */
    DP83867_LED_LINKED_10BT      = 0x7U,

    /*! 10/100BT link established */
    DP83867_LED_LINKED_10100BT   = 0x8U,

    /*! 100/1000BT link established */
    DP83867_LED_LINKED_1001000BT = 0x9U,

    /*! Full duplex */
    DP83867_LED_FULLDUPLEX       = 0xAU,

    /*! Link established, blink for transmit or receive activity */
    DP83867_LED_LINKED_BLINKACT  = 0xBU,

    /*! Receive error or transmit error */
    DP83867_LED_RXTXERR          = 0xDU,

    /*! Receive error */
    DP83867_LED_RXERR            = 0xEU,
} Dp83867_LedMode;

/*!
 * \brief DP83867 PHY configuration parameters.
 */
typedef struct Dp83867_Cfg_s
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
    Dp83867_Gpio0Mode gpio0Mode;

    /*! Mux mode of the GPIO_1 pin */
    Dp83867_Gpio1Mode gpio1Mode;

    /*! LED mode (source) configuration */
    Dp83867_LedMode ledMode[DP83867_LED_NUM];
} Dp83867_Cfg;

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/*!
 * \brief Initialize DP83867 PHY specific config params.
 *
 * Initializes the DP83867 PHY specific configuration parameters.
 *
 * \param cfg       DP83867 PHY config structure pointer
 */
void Dp83867ExtPhy_initCfg(Dp83867_Cfg *cfg);

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

#endif /* DP83867_H_ */

/*! @} */
