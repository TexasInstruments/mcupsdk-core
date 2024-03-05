/*
 *  Copyright (c) Texas Instruments Incorporated 2020-23
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
 * \file  enet_soc_cfg.h
 *
 * \brief This file contains the Enet configuration parameters.
 */

#ifndef ENET_SOC_CFG_H_
#define ENET_SOC_CFG_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */

/* --------------------------------------------------------------------------*/
/*                         Enet generic config options                       */
/* --------------------------------------------------------------------------*/
/*! \brief Maximum number of supported PHYs (allocated PHY objects). */
#define ENET_CFG_ENETPHY_PHY_MAX                    (4U)

/* --------------------------------------------------------------------------*/
/*        CPSW Peripheral and CPSW Module related config options             */
/* --------------------------------------------------------------------------*/

/*! \brief CPSW interVLAN support (requires #ENET_CFG_CPSW_MACPORT_INTERVLAN). */
#define ENET_CFG_CPSW_INTERVLAN                     (ENET_ON)

/*! \brief CPSW 2G (2 Port Switch) support. */
#define ENET_CFG_CPSW_2PORTSWITCH                   (ENET_OFF)

/*! \brief CPDMA Channel Override support. */
#define ENET_CFG_CPDMA_CH_OVERRIDE                  (ENET_OFF)

/*! \brief CPSW IET support. */
#define ENET_CFG_CPSW_IET_INCL                      (ENET_ON)

/*! \brief ALE VLAN MASK MUX support. */
#define ENET_CFG_ALE_VLAN_MASK_MUX                  (ENET_OFF)

/*! \brief CPSW MII support. */
#define ENET_CFG_CPSW_MACPORT_MII                   (ENET_OFF)

/*! \brief CPSW XGMII support. */
#define ENET_CFG_CPSW_XGMII                         (ENET_ON)

/*! \brief CPSW Q/SGMII support (requires #ENET_CFG_CPSW_MACPORT_SGMII). */
#define ENET_CFG_CPSW_SGMII                         (ENET_OFF)

/*! \brief CPSW Q/SGMII support */
#define ENET_CFG_CPSW_MACPORT_SGMII                 (ENET_OFF)

/*! \brief CPSW CPPI Castagnoli CRC support. */
#define ENET_CFG_CPSW_CPPI_CAST                     (ENET_ON)

/*! \brief CPSW Host Traffic Shaping support */
#define ENET_CFG_CPSW_HOSTPORT_TRAFFIC_SHAPING      (ENET_ON)

/*! \brief CPSW Mac Traffic Shaping support */
#define ENET_CFG_CPSW_MACPORT_TRAFFIC_SHAPING       (ENET_ON)

/*! \brief CPSW interVLAN support */
#define ENET_CFG_CPSW_MACPORT_INTERVLAN             (ENET_ON)

/*! \brief Maximum number of remote clients cores */
#define ENET_CFG_REMOTE_CLIENT_CORES_MAX            (1U)

/*! \brief Maximum number of MAC addresses allocated for Resource Manager */
#define ENET_CFG_RM_MAC_ADDR_MAX                    (4U)

/* \brief CPSW Checksum Offload support */
#define ENET_CFG_CPSW_CSUM_OFFLOAD_SUPPORT          (ENET_ON)

/* \brief SOC CPSW Version Info */
#define ENET_SOC_CFG_XGE_CPSW_ID_VER_REG_MAJOR_VER  (1U)

#define ENET_SOC_CFG_XGE_CPSW_ID_VER_REG_MINOR_VER  (3U)

#define ENET_SOC_CFG_XGE_CPSW_ID_VER_REG_RTL_VER    (1U)

/*! \brief Maximum number of TX channels allocated for Resource Manager */
#define ENET_CFG_RM_TX_CH_MAX                       (8U)

/*! \brief Maximum number of RX channels/flows allocated for Resource Manager */
#define ENET_CFG_RM_RX_CH_MAX                       (16U)

/* --------------------------------------------------------------------------*/
/*       ICSS-G Peripheral and CPSW Module related config options            */
/* --------------------------------------------------------------------------*/

/* --------------------------------------------------------------------------*/
/*        GMAC Peripheral and CPSW Module related config options             */
/* --------------------------------------------------------------------------*/

/*! \brief Maximum number of MAC port stats blocks. */
#define CPSW_STATS_MACPORT_MAX              (2U)

/*! \brief Number of switch ports in the subsystem. Number of Mac ports + 1 host port  */
#define CPSW_ALE_NUM_PORTS                          ((CPSW_STATS_MACPORT_MAX) + 1U)

/*! \brief CPSW EST support */
#define ENET_CFG_CPSW_EST                           (ENET_ON)
#define ENET_CFG_CPSW_MACPORT_EST                   (ENET_ON)

/*! \brief Cut-thru switching support */
#define ENET_CFG_CPSW_CUTTHRU                       (ENET_ON)

/*! \brief MDIO Clause-45 frame support. */
#define ENET_CFG_MDIO_CLAUSE45                      (ENET_ON)

/*! \brief Number of CPSW ESTFn modules */
#define ENET_CFG_CPSW_ESTF_NUM                      (CPSW_STATS_MACPORT_MAX)

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* None */

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

#endif /* ENET_SOC_CFG_H_ */
