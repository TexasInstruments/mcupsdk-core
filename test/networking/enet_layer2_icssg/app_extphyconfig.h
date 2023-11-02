/*
 *  Copyright (c) Texas Instruments Incorporated 2023
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
 * \file  app_extphyconfig.h
 *
 * \brief This file contains private type definitions and helper macros for the
 *        DP83826e Ethernet PHY.
 */

#ifndef APP_EXTPHYCONFIG_H_
#define APP_EXTPHYCONFIG_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <drivers/gpio.h>
#include <include/core/enet_utils.h>
#include <priv/core/enet_trace_priv.h>
#include <include/phy/enetphy.h>
#include <kernel/dpl/ClockP.h>
#include <networking/enet/core/src/phy/generic_phy.h>
#include <networking/enet/core/src/phy/enetphy_priv.h>
#include "csl_mdio.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */

/*! \brief PHY Control Register #1 (CR1) */
#define DP83826_CR1                           (0x09U)

/*! \brief PHY Control Register (PHYCR) */
#define DP83826_PHYCR                         (0x19U)

/*! \brief Control Register (CTRL) */
#define DP83826_PHYRCR                        (0x1FU)
/* CTRL register definitions */
#define PHYRCR_SWRESET                        ENETPHY_BIT(15)
#define PHYRCR_SWRESTART                      ENETPHY_BIT(14)

/* CR1 register definitions */
#define CR1_ROBUSTAUTOMDIX                    (0x0020U)

/* PHYCR register definitions */
#define PHYCR_AUTOMDIX_ENABLE                 (0x8000U)
#define PHYCR_FORCEMDIX_MASK                  (0x4000U)
#define PHYCR_FORCEMDIX_MDIX                  (0x4000U)
#define PHYCR_FORCEMDIX_MDI                   (0x0000U)

/* IOMUXCFG register definitions */
#define DP83826_IO_CFG1                       (0x302U)
#define IOCFG_IOIMPEDANCE_MASK                (0xC000U)
#define DP83826_IMPEDENCE_SLOW                (0x00U)
#define DP83826_IMPEDENCE_HIGH                (0x01U)
#define DP83826_IMPEDENCE_VAL(impedance)      (impedance | (impedance<<1))


#define CSL_PRU_ICSSG0_PR1_MDIO_V1P7_MDIO_BASE  (0x30032400UL)

#define ICSS_MDIO_USED                          (CSL_PRU_ICSSG0_PR1_MDIO_V1P7_MDIO_BASE)

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/*!
 * \brief DP83826e PHY configuration parameters.
 */
typedef struct Dp83826_Cfg_s
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

} Dp83826_Cfg;

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/*!
 * \brief Initialize DP83826 PHY specific config params.
 *
 * Initializes the DP83826 PHY specific configuration parameters.
 *
 * \param cfg       DP83826 PHY config structure pointer
 */
void Dp83826_initCfg(Dp83826_Cfg *cfg);

void Dp83826_resetPHYs();

void EnetPhy_configPHYs (void);

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

#endif /* APP_EXTPHYCONFIG_H_ */

/*! @} */


