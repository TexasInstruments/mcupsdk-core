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
/*
 * Auto generated file 
 */
#include "ti_board_config.h"


#include <stdint.h>
#include <enet.h>
#include <networking/enet/core/include/phy/enetphy.h>
#include <networking/enet/core/include/phy/dp83867.h>
#include <networking/enet/core/include/phy/dp83869.h>

#include <networking/enet/utils/include/enet_apputils.h>
#include <kernel/dpl/SystemP.h>
#include <networking/enet/core/src/phy/enetphy_priv.h>
#include <board/eeprom.h>
#include "ti_board_open_close.h"

#if (ENETBOARD_SYSCFG_CUSTOM_BOARD == 1)
/* PHY drivers */
extern EnetPhy_Drv gEnetPhyDrvGeneric;
extern EnetPhy_Drv gEnetPhyDrvDp83869;

/*! \brief All the registered PHY specific drivers. */
static const EnetPhyDrv_Handle gEnetPhyDrvs[] =
{
    &gEnetPhyDrvDp83869,   /* DP83869 */
    &gEnetPhyDrvGeneric,   /* Generic PHY - must be last */
};

const EnetPhy_DrvInfoTbl gEnetPhyDrvTbl =
{
    .numHandles = ENET_ARRAYSIZE(gEnetPhyDrvs),
    .hPhyDrvList = gEnetPhyDrvs,
};


/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static const EnetBoard_PortCfg *EnetBoard_getPortCfg(const EnetBoard_EthPort *ethPort);

static const EnetBoard_PortCfg *EnetBoard_findPortCfg(const EnetBoard_EthPort *ethPort,
                                                      const EnetBoard_PortCfg *ethPortCfgs,
                                                      uint32_t numEthPorts);

static void EnetBoard_setEnetControl(Enet_Type enetType,
                                     Enet_MacPort macPort,
                                     EnetMacPort_Interface *mii);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */


#define ENET_BOARD_NUM_MACADDR_MAX (2U)
#define I2C_EEPROM_MAC_DATA_OFFSET (0x42)


/*!
 * \brief Common Processor Board (CPB) board's DP83869 PHY configuration.
 */
static const Dp83869_Cfg gEnetCpbBoard_dp83869PhyCfg =
{
    .txClkShiftEn         = true,
    .rxClkShiftEn         = true,
    .rxDelayInPs          = 2000U,
    .txDelayInPs          = 750U,
    .txFifoDepth          = 4U,
    .impedanceInMilliOhms = 35000,  /* 35 ohms */
    .idleCntThresh        = 4U,     /* Improves short cable performance */
    .gpio0Mode            = DP83869_GPIO0_LED3,
    .gpio1Mode            = DP83869_GPIO1_COL, /* Unused */
    .ledMode              =
    {
        DP83869_LED_LINKED,         /* Unused */
        DP83869_LED_LINKED_100BTX,
        DP83869_LED_RXTXACT,
        DP83869_LED_LINKED_1000BT,
    },
};

/*
 * am243x-evm board configuration.
 *
 * 1 x RGMII PHY connected to am243x-evm ICSSG
 */
static const EnetBoard_PortCfg gEnetCpbBoard_am243x_evm_EthPort[] =
{
    {    /* "ETH0" (ICSSG0 Dual-MAC port 1) */
        .enetType = ENET_ICSSG_DUALMAC,
        .instId   = 0U,
        .macPort  = ENET_MAC_PORT_1,
        .mii      = { ENET_MAC_LAYER_GMII, ENET_MAC_SUBLAYER_REDUCED },
        .phyCfg   =
        {
            .phyAddr         = 0U,
            .isStrapped      = true,
            .skipExtendedCfg = false,
            .extendedCfg     = &gEnetCpbBoard_dp83869PhyCfg,
            .extendedCfgSize = sizeof(gEnetCpbBoard_dp83869PhyCfg),
        },
        .flags    = 0U,
    },
    {    /* "ETH1" (ICSSG0 Dual-MAC port 2) */
        .enetType = ENET_ICSSG_DUALMAC,
        .instId   = 1U,
        .macPort  = ENET_MAC_PORT_1,
        .mii      = { ENET_MAC_LAYER_GMII, ENET_MAC_SUBLAYER_REDUCED },
        .phyCfg   =
        {
            .phyAddr         = 0U,
            .isStrapped      = true,
            .skipExtendedCfg = false,
            .extendedCfg     = &gEnetCpbBoard_dp83869PhyCfg,
            .extendedCfgSize = sizeof(gEnetCpbBoard_dp83869PhyCfg),
        },
        .flags    = 0U,
    },
    {    /* "ETH2" (ICSSG1 Dual-MAC port 1) */
        .enetType = ENET_ICSSG_DUALMAC,
        .instId   = 2U,
        .macPort  = ENET_MAC_PORT_1,
        .mii      = { ENET_MAC_LAYER_GMII, ENET_MAC_SUBLAYER_REDUCED },
        .phyCfg   =
        {
            .phyAddr         = 0U,
            .isStrapped      = false,
            .skipExtendedCfg = false,
            .extendedCfg     = &gEnetCpbBoard_dp83869PhyCfg,
            .extendedCfgSize = sizeof(gEnetCpbBoard_dp83869PhyCfg),
        },
        .flags    = 0U,
    },
    {    /* "ETH3" (ICSSG1 Dual-MAC port 2) */
        .enetType = ENET_ICSSG_DUALMAC,
        .instId   = 3U,
        .macPort  = ENET_MAC_PORT_1,
        .mii      = { ENET_MAC_LAYER_GMII, ENET_MAC_SUBLAYER_REDUCED },
        .phyCfg   =
        {
            .phyAddr         = 0U,
            .isStrapped      = false,
            .skipExtendedCfg = false,
            .extendedCfg     = &gEnetCpbBoard_dp83869PhyCfg,
            .extendedCfgSize = sizeof(gEnetCpbBoard_dp83869PhyCfg),
        },
        .flags    = 0U,
    },
    {    /* "ETH0" (ICSSG0 Switch port 1) */
        .enetType = ENET_ICSSG_SWITCH,
        .instId   = 0U,
        .macPort  = ENET_MAC_PORT_1,
        .mii      = { ENET_MAC_LAYER_GMII, ENET_MAC_SUBLAYER_REDUCED },
        .phyCfg   =
        {
            .phyAddr         = 0U,
            .isStrapped      = true,
            .skipExtendedCfg = false,
            .extendedCfg     = &gEnetCpbBoard_dp83869PhyCfg,
            .extendedCfgSize = sizeof(gEnetCpbBoard_dp83869PhyCfg),
        },
        .flags    = 0U,
    },
    {    /* "ETH1" (ICSSG0 Switch port 2) */
        .enetType = ENET_ICSSG_SWITCH,
        .instId   = 0U,
        .macPort  = ENET_MAC_PORT_2,
        .mii      = { ENET_MAC_LAYER_GMII, ENET_MAC_SUBLAYER_REDUCED },
        .phyCfg   =
        {
            .phyAddr         = 3U,
            .isStrapped      = true,
            .skipExtendedCfg = false,
            .extendedCfg     = &gEnetCpbBoard_dp83869PhyCfg,
            .extendedCfgSize = sizeof(gEnetCpbBoard_dp83869PhyCfg),
        },
        .flags    = 0U,
    },
    {    /* "ETH2" (ICSSG1 Switch port 1) */
        .enetType = ENET_ICSSG_SWITCH,
        .instId   = 1U,
        .macPort  = ENET_MAC_PORT_1,
        .mii      = { ENET_MAC_LAYER_GMII, ENET_MAC_SUBLAYER_REDUCED },
        .phyCfg   =
        {
            .phyAddr         = 0U,
            .isStrapped      = false,
            .skipExtendedCfg = false,
            .extendedCfg     = &gEnetCpbBoard_dp83869PhyCfg,
            .extendedCfgSize = sizeof(gEnetCpbBoard_dp83869PhyCfg),
        },
        .flags    = 0U,
    },
    {    /* "ETH3" (ICSSG1 Switch port 2) */
        .enetType = ENET_ICSSG_SWITCH,
        .instId   = 1U,
        .macPort  = ENET_MAC_PORT_2,
        .mii      = { ENET_MAC_LAYER_GMII, ENET_MAC_SUBLAYER_REDUCED },
        .phyCfg   =
        {
            .phyAddr         = 3U,
            .isStrapped      = false,
            .skipExtendedCfg = false,
            .extendedCfg     = &gEnetCpbBoard_dp83869PhyCfg,
            .extendedCfgSize = sizeof(gEnetCpbBoard_dp83869PhyCfg),
        },
        .flags    = 0U,
    },
};

/*
 * am243x-evm MII board configuration.
 *
 * 2 x MII PHY connected to am243x-evm ICSSG1 MAC ports.
 */
static const EnetBoard_PortCfg gEnetMiiBoard_am243x_evm_EthPort[] =
{
    {    /* "ETH2" (ICSSG1 Dual-MAC port 1) */
        .enetType = ENET_ICSSG_DUALMAC,
        .instId   = 2U,
        .macPort  = ENET_MAC_PORT_1,
        .mii      = { ENET_MAC_LAYER_MII, ENET_MAC_SUBLAYER_STANDARD },
        .phyCfg   =
        {
            .phyAddr         = 0U,
            .isStrapped      = false,
            .skipExtendedCfg = false,
            .extendedCfg     = &gEnetCpbBoard_dp83869PhyCfg,
            .extendedCfgSize = sizeof(gEnetCpbBoard_dp83869PhyCfg),
        },
        .flags    = 0U,
    },
    {    /* "ETH3" (ICSSG1 Dual-MAC port 2) */
        .enetType = ENET_ICSSG_DUALMAC,
        .instId   = 3U,
        .macPort  = ENET_MAC_PORT_1,
        .mii      = { ENET_MAC_LAYER_MII, ENET_MAC_SUBLAYER_STANDARD },
        .phyCfg   =
        {
            .phyAddr         = 0U,
            .isStrapped      = false,
            .skipExtendedCfg = false,
            .extendedCfg     = &gEnetCpbBoard_dp83869PhyCfg,
            .extendedCfgSize = sizeof(gEnetCpbBoard_dp83869PhyCfg),
        },
        .flags    = 0U,
    },
    {    /* "ETH2" (ICSSG1 Switch port 1) */
        .enetType = ENET_ICSSG_SWITCH,
        .instId   = 1U,
        .macPort  = ENET_MAC_PORT_1,
        .mii      = { ENET_MAC_LAYER_MII, ENET_MAC_SUBLAYER_STANDARD },
        .phyCfg   =
        {
            .phyAddr         = 0U,
            .isStrapped      = false,
            .skipExtendedCfg = false,
            .extendedCfg     = &gEnetCpbBoard_dp83869PhyCfg,
            .extendedCfgSize = sizeof(gEnetCpbBoard_dp83869PhyCfg),
        },
        .flags    = 0U,
    },
    {    /* "ETH3" (ICSSG1 Switch port 2) */
        .enetType = ENET_ICSSG_SWITCH,
        .instId   = 1U,
        .macPort  = ENET_MAC_PORT_2,
        .mii      = { ENET_MAC_LAYER_MII, ENET_MAC_SUBLAYER_STANDARD },
        .phyCfg   =
        {
            .phyAddr         = 3U,
            .isStrapped      = false,
            .skipExtendedCfg = false,
            .extendedCfg     = &gEnetCpbBoard_dp83869PhyCfg,
            .extendedCfgSize = sizeof(gEnetCpbBoard_dp83869PhyCfg),
        },
        .flags    = 0U,
    },
};
/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

static const EnetBoard_PortCfg *EnetBoard_getPortCfg(const EnetBoard_EthPort *ethPort)
{
    const EnetBoard_PortCfg *portCfg = NULL;

    if (ENET_NOT_ZERO(ethPort->boardId & ENETBOARD_CPB_ID))
    {
        portCfg = EnetBoard_findPortCfg(ethPort,
                                        gEnetCpbBoard_am243x_evm_EthPort,
                                        ENETPHY_ARRAYSIZE(gEnetCpbBoard_am243x_evm_EthPort));
    }
    if (ENET_NOT_ZERO(ethPort->boardId & ENETBOARD_MII_ID))
    {
        portCfg = EnetBoard_findPortCfg(ethPort,
                                        gEnetMiiBoard_am243x_evm_EthPort,
                                        ENETPHY_ARRAYSIZE(gEnetMiiBoard_am243x_evm_EthPort));
    }
    return portCfg;
}

static const EnetBoard_PortCfg *EnetBoard_findPortCfg(const EnetBoard_EthPort *ethPort,
                                                      const EnetBoard_PortCfg *ethPortCfgs,
                                                      uint32_t numEthPorts)
{
    const EnetBoard_PortCfg *ethPortCfg = NULL;
    bool found = false;
    uint32_t i;

    for (i = 0U; i < numEthPorts; i++)
    {
        ethPortCfg = &ethPortCfgs[i];

        if ((ethPortCfg->enetType == ethPort->enetType) &&
            (ethPortCfg->instId == ethPort->instId) &&
            (ethPortCfg->macPort == ethPort->macPort) &&
            (ethPortCfg->mii.layerType == ethPort->mii.layerType) &&
            (ethPortCfg->mii.sublayerType == ethPort->mii.sublayerType))
        {
            found = true;
            break;
        }
    }

    return found ? ethPortCfg : NULL;
}

const EnetBoard_PhyCfg *EnetBoard_getPhyCfg(const EnetBoard_EthPort *ethPort)
{
    const EnetBoard_PortCfg *portCfg;

    portCfg = EnetBoard_getPortCfg(ethPort);

    return (portCfg != NULL) ? &portCfg->phyCfg : NULL;
}

int32_t EnetBoard_setupPorts(EnetBoard_EthPort *ethPorts,
                             uint32_t numEthPorts)
{

    /* Nothing else to do */
    return ENET_SOK;
}


void EnetBoard_getMacAddrList(uint8_t macAddr[][ENET_MAC_ADDR_LEN],
                              uint32_t maxMacEntries,
                              uint32_t *pAvailMacEntries)
{
    int32_t status = ENET_SOK;
    uint32_t macAddrCnt;
    uint32_t i;
    uint8_t macAddrBuf[ENET_BOARD_NUM_MACADDR_MAX * ENET_MAC_ADDR_LEN];
    macAddrCnt = EnetUtils_min(ENET_BOARD_NUM_MACADDR_MAX, maxMacEntries);

    EnetAppUtils_assert(pAvailMacEntries != NULL);

    status = EEPROM_read(gEepromHandle[CONFIG_EEPROM0], I2C_EEPROM_MAC_DATA_OFFSET, macAddrBuf, (macAddrCnt * ENET_MAC_ADDR_LEN));
    EnetAppUtils_assert(status == ENET_SOK);

    /* Save only those required to meet the max number of MAC entries */
    /* TODO Read number of mac addresses from the board eeprom */
    for (i = 0U; i < macAddrCnt; i++)
    {
        memcpy(macAddr[i], &macAddrBuf[i * ENET_MAC_ADDR_LEN], ENET_MAC_ADDR_LEN);
    }

    *pAvailMacEntries = macAddrCnt;

    if (macAddrCnt == 0U)
    {
        EnetAppUtils_print("EnetBoard_getMacAddrList Failed - IDK not present\n");
        EnetAppUtils_assert(false);
    }
}



#endif /* #if (ENETBOARD_SYSCFG_CUSTOM_BOARD == 1) */

