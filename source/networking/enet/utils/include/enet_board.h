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
 * \file  enet_board.h
 *
 * \brief This file contains the type definitions and helper macros for the
 *        Enet example board utils interface.
 */

#ifndef ENET_BOARD_H_
#define ENET_BOARD_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <enet.h>
#include <include/phy/enetphy.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */

/*!
 * \brief Any board available.
 *
 * It can be used as a wildcard when looking for a port in any board present in
 * the system.
 */
#define ENETBOARD_ANY_ID                      (0xFFFFFFFFU)

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/*!
 * \brief Board related configuration parameters of an Ethernet PHY.
 */
typedef struct EnetBoard_PhyCfg_s
{
    /*! PHY device address */
    uint32_t phyAddr;

    /*! Interface type */
    EnetPhy_Mii mii;

    /*! Whether PHY is strapped or not */
    bool isStrapped;

    /*! Whether to skip PHY-specific extended configuration */
    bool skipExtendedCfg;

    /*! Extended PHY-specific configuration */
    const void *extendedCfg;

    /*! Size of the extended configuration */
    uint32_t extendedCfgSize;
} EnetBoard_PhyCfg;

/*!
 * \brief Ethernet port configuration parameters.
 */
typedef struct EnetBoard_PortCfg_s
{
    /*! Peripheral type connected to */
    Enet_Type enetType;

    /*! Instance Id of the peripheral connected to */
    uint32_t instId;

    /*! MAC port connected to */
    Enet_MacPort macPort;

    /*! MAC port interface */
    EnetMacPort_Interface mii;

    /*! PHY configuration parameters */
    EnetBoard_PhyCfg phyCfg;

    /*! Configuration flags indicating the board features/components that need
     *  to be setup */
    uint32_t flags;
} EnetBoard_PortCfg;

/*!
 * \brief Ethernet port.
 */
typedef struct EnetBoard_EthPort_s
{
    /*! Peripheral type connected to */
    Enet_Type enetType;

    /*! Instance Id of the peripheral connected to */
    uint32_t instId;

    /*! MAC port connected to */
    Enet_MacPort macPort;

    /*! MAC port interface */
    EnetMacPort_Interface mii;

    /*! Board Id where port is physically present */
    uint32_t boardId;
} EnetBoard_EthPort;

/**
 * \enum emac_mode
 *
 * \brief specifies the available emac mode types.
 */
typedef enum
{
    RMII = 1,
    RGMII = 2,
}emac_mode;

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/*!
 * \brief Get configuration info for requested PHY.
 *
 * Gets the configuration information for a given PHY.  The PHY is found in
 * a list of PHYs available in the supported board(s).
 *
 * The search criteria includes peripheral (type and instance), MAC port that
 * the PHY is connected to and the MII interface type.  The search can be done
 * in one or multiple boards (bitmask) via \ref EnetBoard_EthPort::boardId.
 *
 * \param ethPort   Ethernet port to get configuration for
 *
 * \return Pointer to the found PHY configuration info. Otherwise, NULL.
 */
const EnetBoard_PhyCfg *EnetBoard_getPhyCfg(const EnetBoard_EthPort *ethPort);

/*!
 * \brief Initialize board configuration.
 *
 * \return \ref Enet_ErrorCodes
 */
int32_t EnetBoard_init(void);

/*!
 * \brief Initialize board configuration for EthFw use.
 *
 * \return \ref Enet_ErrorCodes
 */
void EnetBoard_initEthFw(void);

/*!
 * \brief Deinitialize board configuration.
 */
void EnetBoard_deinit(void);

/*!
 * \brief Setup board configuration for a given set of ports.
 *
 * \param ethPorts        Array of ports to be setup
 * \param numEthPorts     Size of the ports array
 *
 * \return \ref Enet_ErrorCodes
 */
int32_t EnetBoard_setupPorts(EnetBoard_EthPort *ethPorts,
                             uint32_t numEthPorts);

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

#endif /* ENET_BOARD_H_ */
