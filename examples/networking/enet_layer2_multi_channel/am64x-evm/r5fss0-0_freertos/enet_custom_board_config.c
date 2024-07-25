/*
 *  Copyright (C) 2022 Texas Instruments Incorporated
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

#include <stdint.h>
#include <enet.h>
#include <networking/enet/core/include/phy/enetphy.h>
#include <networking/enet/core/include/phy/dp83867.h>
#include <networking/enet/core/include/phy/dp83869.h>
#include <networking/enet/utils/include/enet_apputils.h>
#include <kernel/dpl/SystemP.h>
#include <kernel/dpl/AddrTranslateP.h>
#include <networking/enet/core/src/phy/enetphy_priv.h>
#include <drivers/gpio.h>
#include <board/eeprom.h>
#include "ti_board_open_close.h"
#include "ti_drivers_config.h"
#include "ti_board_config.h"

#if (ENETBOARD_SYSCFG_CUSTOM_BOARD == 1)

#include <drivers/i2c.h>

#define IO_EXPANDER_PORT0_OUTPUT_REG (0x04U)
#define IO_EXPANDER_PORT0_DIR_REG (0x0CU)
#define IO_EXPANDER_I2C_ADDR (0x22U)
#define ENET_BOARD_NUM_MACADDR_MAX (2U)
#define I2C_EEPROM_MAC_DATA_OFFSET (0x42)

static void EnetBoard_setMacPort2IOExpanderCfg(void);

/* PHY drivers */
extern EnetPhy_Drv gEnetPhyDrvGeneric;
extern EnetPhy_Drv gEnetPhyDrvDp83867;
extern EnetPhy_Drv gEnetPhyDrvDp83869;

/*! \brief All the registered PHY specific drivers. */
static const EnetPhyDrv_Handle gEnetPhyDrvs[] =
{
    &gEnetPhyDrvDp83867,   /* DP83867 */
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

static void EnetBoard_enableExternalMux();

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/*!
 * \brief Common Processor Board (CPB) board's DP83867 PHY configuration.
 */
static const Dp83867_Cfg gEnetCpbBoard_dp83867PhyCfg =
{
/* The delay values are set based on trial and error and not tuned per port of the evm */
    .txClkShiftEn         = true,
    .rxClkShiftEn         = true,
    .txDelayInPs          = 250U,   /* 0.25 ns */
    .rxDelayInPs          = 2000U,  /* 2.00 ns */
    .txFifoDepth          = 4U,
    .impedanceInMilliOhms = 35000,  /* 35 ohms */
    .idleCntThresh        = 4U,     /* Improves short cable performance */
    .gpio0Mode            = DP83867_GPIO0_LED3,
    .gpio1Mode            = DP83867_GPIO1_COL, /* Unused */
    .ledMode              =
    {
        DP83867_LED_LINKED,         /* Unused */
        DP83867_LED_LINKED_100BTX,
        DP83867_LED_RXTXACT,
        DP83867_LED_LINKED_1000BT,
    },
};

/*!
 * \brief Common Processor Board (CPB) board's DP83869 PHY configuration.
 */
static const Dp83869_Cfg gEnetCpbBoard_dp83869PhyCfg =
{
    .txClkShiftEn         = false,
    .rxClkShiftEn         = true,
    .txDelayInPs          = 500U, /* Value in pecosec. Refer to DLL_RX_DELAY_CTRL_SL field in ANA_RGMII_DLL_CTRL register of DP83869 PHY datasheet */
    .rxDelayInPs          = 500U, /* Value in pecosec. Refer to DLL_TX_DELAY_CTRL_SL field in ANA_RGMII_DLL_CTRL register of DP83869 PHY datasheet */
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
 * AM64x board configuration.
 *
 * 1 x RGMII PHY connected to am64x-evm CPSW_3G MAC port.
 */
static const EnetBoard_PortCfg gEnetCpbBoard_am64x_evm_EthPort[] =
{
    {    /* "CPSW3G" */
        .enetType = ENET_CPSW_3G,
        .instId   = 0U,
        .macPort  = ENET_MAC_PORT_1,
        .mii      = { ENET_MAC_LAYER_GMII, ENET_MAC_SUBLAYER_REDUCED },
        .phyCfg   =
        {
            .phyAddr         = 0,
            .isStrapped      = false,
            .skipExtendedCfg = false,
            .extendedCfg     = &gEnetCpbBoard_dp83867PhyCfg,
            .extendedCfgSize = sizeof(gEnetCpbBoard_dp83867PhyCfg),
        },
        .flags    = 0U,
    },
    {    /* "CPSW3G" */
        .enetType = ENET_CPSW_3G,
        .instId   = 0U,
        .macPort  = ENET_MAC_PORT_2,
        .mii      = { ENET_MAC_LAYER_GMII, ENET_MAC_SUBLAYER_REDUCED },
        .phyCfg   =
        {
            .phyAddr         = 3,
            .isStrapped      = false,
            .skipExtendedCfg = false,
            .extendedCfg     = &gEnetCpbBoard_dp83869PhyCfg,
            .extendedCfgSize = sizeof(gEnetCpbBoard_dp83869PhyCfg),
        },
        .flags    = 0U,
    },
};

/*
 * am64x-evm virtual board used for MAC loopback setup.
 */
static const EnetBoard_PortCfg gEnetVirtBoard_am64x_evm_EthPort[] =
{
    {    /* RGMII MAC loopback */
        .enetType = ENET_CPSW_3G,
        .instId   = 0U,
        .macPort  = ENET_MAC_PORT_1,
        .mii      = { ENET_MAC_LAYER_GMII, ENET_MAC_SUBLAYER_REDUCED },
        .phyCfg   =
        {
            .phyAddr = ENETPHY_INVALID_PHYADDR,
        },
        .flags    = 0U,
    },
    {    /* RMII MAC loopback */
        .enetType = ENET_CPSW_3G,
        .instId   = 0U,
        .macPort  = ENET_MAC_PORT_1,
        .mii      = { ENET_MAC_LAYER_MII, ENET_MAC_SUBLAYER_REDUCED },
        .phyCfg   =
        {
            .phyAddr = ENETPHY_INVALID_PHYADDR,
        },
        .flags    = 0U,
    },
};


/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

static void EnetBoard_enableExternalMux()
{
    /* Enable external MUXes, if any, as per the board design */
}

static const EnetBoard_PortCfg *EnetBoard_getPortCfg(const EnetBoard_EthPort *ethPort)
{
    const EnetBoard_PortCfg *portCfg = NULL;

    if (ENET_NOT_ZERO(ethPort->boardId & ENETBOARD_CPB_ID))
    {
        portCfg = EnetBoard_findPortCfg(ethPort,
                                        gEnetCpbBoard_am64x_evm_EthPort,
                                        ENETPHY_ARRAYSIZE(gEnetCpbBoard_am64x_evm_EthPort));
    }
    if ((portCfg == NULL) &&
        ENET_NOT_ZERO(ethPort->boardId & ENETBOARD_LOOPBACK_ID))
    {
        portCfg = EnetBoard_findPortCfg(ethPort,
                                        gEnetVirtBoard_am64x_evm_EthPort,
                                        ENETPHY_ARRAYSIZE(gEnetVirtBoard_am64x_evm_EthPort));
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

static void EnetBoard_setMacPort2IOExpanderCfg(void)
{
    I2C_Transaction i2cTransaction;
    uint8_t buffer[2];

    I2C_Transaction_init(&i2cTransaction);
    i2cTransaction.writeBuf     = buffer;
    i2cTransaction.writeCount   = 2U;
    i2cTransaction.targetAddress = IO_EXPANDER_I2C_ADDR;

    /* Configure MDIO sel pin */
    /* Set output to high */
    buffer[0] = IO_EXPANDER_PORT0_OUTPUT_REG + 1; /* Port 1 */
    buffer[1] = (0x01 << 4); /* Pin 4 */
    I2C_transfer(I2C_getHandle(CONFIG_I2C1), &i2cTransaction);

    /* set pin to output */
    buffer[0] = IO_EXPANDER_PORT0_DIR_REG + 1;
    buffer[1] = ~(0x1 << 4);
    I2C_transfer(I2C_getHandle(CONFIG_I2C1), &i2cTransaction);

}

int32_t EnetBoard_setupPorts(EnetBoard_EthPort *ethPorts,
                             uint32_t numEthPorts)
{
    uint32_t i;

    EnetBoard_enableExternalMux();

    /* Nothing else to do */
    for (i = 0U; i < numEthPorts; i++)
    {
        EnetBoard_EthPort *ethPort = &ethPorts[i];
        if(ethPort->macPort ==ENET_MAC_PORT_2)
        {
               EnetBoard_setMacPort2IOExpanderCfg();
        }
    }
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

/*
 * Get ethernet board id
 */
uint32_t EnetBoard_getId(void)
{
    return ENETBOARD_AM64X_AM243X_EVM;
}

void EnetBoard_getMiiConfig(EnetMacPort_Interface *mii)
{
    mii->layerType      =     ENET_MAC_LAYER_GMII;
    mii->sublayerType   =     ENET_MAC_SUBLAYER_REDUCED;
    mii->variantType    =     ENET_MAC_VARIANT_FORCED;
}


#endif /* #if (ENETBOARD_SYSCFG_CUSTOM_BOARD == 1) */

