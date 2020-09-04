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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <board/ethphy/ethphy_dp83869.h>
#include <drivers/mdio.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define ETHPHY_DP83869_BMCR_REG                                     (0x0000)
#define ETHPHY_DP83869_BMSR_REG                                     (0x0001)
#define ETHPHY_DP83869_PHYIDR1_REG                                  (0x0002)
#define ETHPHY_DP83869_ANER_REG                                     (0x0006)
#define ETHPHY_DP83869_GEN_CFG1_REG                                 (0x0009)
#define ETHPHY_DP83869_PHY_CONTROL_REG                              (0x0010)
#define ETHPHY_DP83869_PHY_STATUS_REG                               (0x0011)
#define ETHPHY_DP83869_LEDS_CFG1_REG                                (0x0018)
#define ETHPHY_DP83869_LEDS_CFG2_REG                                (0x0019)
#define ETHPHY_DP83869_LEDS_CFG3_REG                                (0x001A)
#define ETHPHY_DP83869_GEN_CFG4_REG                                 (0x001E)
#define ETHPHY_DP83869_GEN_CTRL_REG                                 (0x001F)
#define ETHPHY_DP83869_GEN_CFG_FLD_REG                              (0x002D)
#define ETHPHY_DP83869_RGMII_CTRL_REG                               (0x0032)
#define ETHPHY_DP83869_RGMII_CTRL2_REG                              (0x0033)
#define ETHPHY_DP83869_G_100BT_REG0_REG                             (0x0043)
#define ETHPHY_DP83869_OP_MODE_DECODE_REG                           (0x01DF)

#define ETHPHY_DP83869_BMCR_IEEE_POWER_DOWN_ENABLE_MASK             (0x0800)
#define ETHPHY_DP83869_BMCR_AUTO_NEGOTIATE_ENABLE_MASK              (0x1000)
#define ETHPHY_DP83869_BMCR_SPEED_SEL_LSB_MASK                      (0x2000)
#define ETHPHY_DP83869_BMCR_SPEED_SEL_MSB_MASK                      (0x0040)
#define ETHPHY_DP83869_BMCR_DUPLEX_EN_MASK                          (0x0100)
#define ETHPHY_DP83869_BMSR_AUTONEG_COMP_MASK                       (0x0020)
#define ETHPHY_DP83869_ANER_LP_AUTONEG_ABLE_MASK                    (0x0001)
#define ETHPHY_DP83869_GEN_CFG1_G_1000BT_FD_ADV_MASK                (0x0200)
#define ETHPHY_DP83869_GEN_CFG1_G_1000BT_HD_ADV_MASK                (0x0100)
#define ETHPHY_DP83869_PHY_CONTROL_AUTOMDIX_ENABLE_MASK             (0x0040)
#define ETHPHY_DP83869_PHY_STATUS_SPEED_MASK                        (0xC000)
#define ETHPHY_DP83869_PHY_STATUS_DUPLEX_MASK                       (0x2000)
#define ETHPHY_DP83869_LEDS_CFG1_LED_SEL_MASK                       (0x000F)
#define ETHPHY_DP83869_LEDS_CFG3_LEDS_BLINK_RATE_MASK               (0x0003)
#define ETHPHY_DP83869_GEN_CFG4_EXT_FD_ENABLE_MASK                  (0x0800)
#define ETHPHY_DP83869_GEN_CTRL_SW_RESTART_MASK                     (0x4000)
#define ETHPHY_DP83869_GEN_CFG_FLD_FAST_LINK_DOWN_MODES_MASK        (0x001F)
#define ETHPHY_DP83869_RGMII_CTRL_RX_HALF_FULL_THR_LSB_MASK         (0x0060)
#define ETHPHY_DP83869_RGMII_CTRL_TX_HALF_FULL_THR_LSB_MASK         (0x0018)
#define ETHPHY_DP83869_RGMII_CTRL2_RX_HALF_FULL_THR_MSB_MASK        (0x0002)
#define ETHPHY_DP83869_RGMII_CTRL2_TX_HALF_FULL_THR_MSB_MASK        (0x0001)
#define ETHPHY_DP83869_RGMII_CTRL2_LOW_LATENCY_10_100_EN_MASK       (0x0004)
#define ETHPHY_DP83869_G_100BT_REG0_ENHANCED_IPG_DET_ENABLE_MASK    (0x0010)
#define ETHPHY_DP83869_G_100BT_REG0_ODDNIBBLE_DET_ENABLE_MASK       (0x0002)
#define ETHPHY_DP83869_OP_MODE_DECODE_RGMII_MII_SEL_MASK            (0x0020)

#define ETHPHY_DP83869_RGMII_CTRL_RX_HALF_FULL_THR_LSB_SHIFT        (0x0005)
#define ETHPHY_DP83869_RGMII_CTRL_TX_HALF_FULL_THR_LSB_SHIFT        (0x0003)
#define ETHPHY_DP83869_RGMII_CTRL2_RX_HALF_FULL_THR_MSB_SHIFT       (0x0001)
#define ETHPHY_DP83869_RGMII_CTRL2_TX_HALF_FULL_THR_MSB_SHIFT       (0x0000)

#define ETHPHY_DP83869_PHYIDR1_REG_VALUE                            (0x2000)

#define ETHPHY_DP83869_PHY_STATUS_SPEED_10M                         (0x0000)
#define ETHPHY_DP83869_PHY_STATUS_SPEED_100M                        (0x4000)
#define ETHPHY_DP83869_PHY_STATUS_SPEED_1000M                       (0xC000)

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

int32_t ETHPHY_DP83869_open(ETHPHY_Config *config, const ETHPHY_Params *params);

int32_t ETHPHY_DP83869_command(ETHPHY_Config *config,
                               uint32_t command,
                               void *data,
                               uint32_t dataSize);

static int32_t ETHPHY_DP83869_enableMii(ETHPHY_Attrs *attrs);

static int32_t ETHPHY_DP83869_softRestart(ETHPHY_Attrs *attrs);

static int32_t ETHPHY_DP83869_enableAutoMdix(ETHPHY_Attrs *attrs);

static int32_t ETHPHY_DP83869_verifyIdentifierRegister(ETHPHY_Attrs *attrs);

static int32_t ETHPHY_DP83869_disable1000mAdvertisement(ETHPHY_Attrs *attrs);

static int32_t ETHPHY_DP83869_enableFastLinkDownDetection(ETHPHY_Attrs *attrs,
                                                          void         *data,
                                                          uint32_t     dataSize);

static int32_t ETHPHY_DP83869_configureLedSource(ETHPHY_Attrs *attrs,
                                                 void         *data,
                                                 uint32_t     dataSize);

static int32_t ETHPHY_DP83869_configureLedBlinkRate(ETHPHY_Attrs *attrs,
                                                    void         *data,
                                                    uint32_t     dataSize);

static int32_t ETHPHY_DP83869_enableExtendedFdAbility(ETHPHY_Attrs *attrs);

static int32_t ETHPHY_DP83869_enableOddNibbleDetection(ETHPHY_Attrs *attrs);

static int32_t ETHPHY_DP83869_enableEnhancedIpgDetection(ETHPHY_Attrs *attrs);

static int32_t ETHPHY_DP83869_getSpeedDuplex(ETHPHY_Attrs *attrs,
                                                void         *data,
                                                uint32_t     dataSize);

static int32_t ETHPHY_DP83869_setSpeedDuplex(ETHPHY_Attrs *attrs,
                                                void         *data,
                                                uint32_t     dataSize);

static int32_t ETHPHY_DP83869_enableLowLatencyRgmii(ETHPHY_Attrs *attrs);

static int32_t ETHPHY_DP83869_setRxHalfFullThresholdRgmii(ETHPHY_Attrs *attrs,
                                                          void         *data,
                                                          uint32_t     dataSize);

static int32_t ETHPHY_DP83869_setTxHalfFullThresholdRgmii(ETHPHY_Attrs *attrs,
                                                          void         *data,
                                                          uint32_t     dataSize);

static int32_t ETHPHY_DP83869_getAutonegCompleteStatus(ETHPHY_Attrs *attrs,
                                                       void         *data,
                                                       uint32_t     dataSize);


static int32_t ETHPHY_DP83869_getLinkPartnerAutonegAbility(ETHPHY_Attrs *attrs,
                                                           void         *data,
                                                           uint32_t     dataSize);

static int32_t ETHPHY_DP83869_enableIeeePowerDownMode(ETHPHY_Attrs *attrs);

static int32_t ETHPHY_DP83869_disableIeeePowerDownMode(ETHPHY_Attrs *attrs);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

ETHPHY_Fxns gEthPhyFxns_DP83869 =
{
    .openFxn    = ETHPHY_DP83869_open,
    .closeFxn   = NULL,
    .commandFxn = ETHPHY_DP83869_command,
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t ETHPHY_DP83869_open(ETHPHY_Config *config, const ETHPHY_Params *params)
{
    int32_t         status = SystemP_SUCCESS;
    ETHPHY_Attrs    *attrs;

    if((NULL == config) || (NULL == params) || (NULL == config->attrs))
    {
        status = SystemP_FAILURE;
    }
    else
    {
        attrs = config->attrs;
        status = MDIO_initClock(attrs->mdioBaseAddress);
    }

    return status;
}

int32_t ETHPHY_DP83869_command(ETHPHY_Config *config,
                               uint32_t command,
                               void *data,
                               uint32_t dataSize)
{
    int32_t         status = SystemP_SUCCESS;
    ETHPHY_Attrs    *attrs;

    if(NULL == config)
    {
        status = SystemP_FAILURE;
    }

    if(status == SystemP_SUCCESS)
    {
        attrs = config->attrs;
        switch (command)
        {
        case ETHPHY_CMD_ENABLE_MII :
            status = ETHPHY_DP83869_enableMii(attrs);
            break;

        case ETHPHY_CMD_SOFT_RESTART :
            status = ETHPHY_DP83869_softRestart(attrs);
            break;

        case ETHPHY_CMD_ENABLE_AUTO_MDIX :
            status = ETHPHY_DP83869_enableAutoMdix(attrs);
            break;

        case ETHPHY_CMD_VERIFY_IDENTIFIER_REGISTER :
            status = ETHPHY_DP83869_verifyIdentifierRegister(attrs);
            break;

        case ETHPHY_CMD_DISABLE_1000M_ADVERTISEMENT :
            status = ETHPHY_DP83869_disable1000mAdvertisement(attrs);
            break;

        case ETHPHY_CMD_ENABLE_FAST_LINK_DOWN_DETECTION :
            status = ETHPHY_DP83869_enableFastLinkDownDetection(attrs, data, dataSize);
            break;

        case ETHPHY_CMD_CONFIGURE_LED_SOURCE :
            status = ETHPHY_DP83869_configureLedSource(attrs, data, dataSize);
            break;

        case ETHPHY_CMD_CONFIGURE_LED_BLINK_RATE :
            status = ETHPHY_DP83869_configureLedBlinkRate(attrs, data, dataSize);
            break;

        case ETHPHY_CMD_ENABLE_EXTENDED_FD_ABILITY :
            status = ETHPHY_DP83869_enableExtendedFdAbility(attrs);
            break;

        case ETHPHY_CMD_ENABLE_ODD_NIBBLE_DETECTION :
            status = ETHPHY_DP83869_enableOddNibbleDetection(attrs);
            break;

        case ETHPHY_CMD_ENABLE_ENHANCED_IPG_DETECTION :
            status = ETHPHY_DP83869_enableEnhancedIpgDetection(attrs);
            break;

        case ETHPHY_CMD_GET_LINK_STATUS :
            status = MDIO_phyLinkStatus(attrs->mdioBaseAddress, attrs->phyAddress);
            break;

        case ETHPHY_CMD_GET_SPEED_AND_DUPLEX_CONFIG :
            status = ETHPHY_DP83869_getSpeedDuplex(attrs, data, dataSize);
            break;

        case ETHPHY_CMD_SET_SPEED_AND_DUPLEX_CONFIG :
            status = ETHPHY_DP83869_setSpeedDuplex(attrs, data, dataSize);
            break;

        case ETHPHY_CMD_ENABLE_LOW_LATENCY_10M_100M_RGMII:
            status = ETHPHY_DP83869_enableLowLatencyRgmii(attrs);
            break;

        case ETHPHY_CMD_SET_RX_HALF_FULL_THRESHOLD_RGMII:
            status = ETHPHY_DP83869_setRxHalfFullThresholdRgmii(attrs, data, dataSize);
            break;

        case ETHPHY_CMD_SET_TX_HALF_FULL_THRESHOLD_RGMII:
            status = ETHPHY_DP83869_setTxHalfFullThresholdRgmii(attrs, data, dataSize);
            break;

        case ETHPHY_CMD_GET_AUTONEG_COMPLETE_STATUS:
            status = ETHPHY_DP83869_getAutonegCompleteStatus(attrs, data, dataSize);
            break;

        case ETHPHY_CMD_GET_LINK_PARTNER_AUTONEG_ABILITY:
            status = ETHPHY_DP83869_getLinkPartnerAutonegAbility(attrs, data, dataSize);
            break;

        case ETHPHY_CMD_ENABLE_IEEE_POWER_DOWN:
            status = ETHPHY_DP83869_enableIeeePowerDownMode(attrs);
            break;

        case ETHPHY_CMD_DISABLE_IEEE_POWER_DOWN:
            status = ETHPHY_DP83869_disableIeeePowerDownMode(attrs);
            break;

        default:
            status = SystemP_FAILURE;
            break;
        }
    }

    return status;
}

static int32_t ETHPHY_DP83869_enableMii(ETHPHY_Attrs *attrs)
{
    int32_t status = SystemP_FAILURE;
    uint16_t phyRegVal = 0;

    status = MDIO_phyExtRegRead(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83869_OP_MODE_DECODE_REG, &phyRegVal);

    if(status == SystemP_SUCCESS)
    {
        phyRegVal |= ETHPHY_DP83869_OP_MODE_DECODE_RGMII_MII_SEL_MASK;
        status = MDIO_phyExtRegWrite(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83869_OP_MODE_DECODE_REG, phyRegVal);
    }
    return status;
}

static int32_t ETHPHY_DP83869_softRestart(ETHPHY_Attrs *attrs)
{
    int32_t status = SystemP_FAILURE;
    uint16_t phyRegVal = 0;

    status = MDIO_phyRegRead(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83869_GEN_CTRL_REG, &phyRegVal);

    if(status == SystemP_SUCCESS)
    {
        phyRegVal |= ETHPHY_DP83869_GEN_CTRL_SW_RESTART_MASK;
        status = MDIO_phyRegWrite(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83869_GEN_CTRL_REG, phyRegVal);
    }

    return status;
}

static int32_t ETHPHY_DP83869_enableAutoMdix(ETHPHY_Attrs *attrs)
{
    int32_t status = SystemP_FAILURE;
    uint16_t phyRegVal = 0;

    status = MDIO_phyRegRead(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83869_PHY_CONTROL_REG, &phyRegVal);

    if(status == SystemP_SUCCESS)
    {
        phyRegVal |= ETHPHY_DP83869_PHY_CONTROL_AUTOMDIX_ENABLE_MASK;
        status = MDIO_phyRegWrite(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83869_PHY_CONTROL_REG, phyRegVal);
    }

    return status;
}

static int32_t ETHPHY_DP83869_verifyIdentifierRegister(ETHPHY_Attrs *attrs)
{
    int32_t status = SystemP_FAILURE;
    uint16_t phyRegVal = 0;

    status = MDIO_phyRegRead(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83869_PHYIDR1_REG, &phyRegVal);

    if(status == SystemP_SUCCESS)
    {
        if(phyRegVal == ETHPHY_DP83869_PHYIDR1_REG_VALUE)
        {
            status = SystemP_SUCCESS;
        }
        else
        {
            status = SystemP_FAILURE;
        }
    }
    return status;
}

static int32_t ETHPHY_DP83869_disable1000mAdvertisement(ETHPHY_Attrs *attrs)
{
    int32_t status = SystemP_FAILURE;
    uint16_t phyRegVal = 0;

    status = MDIO_phyRegRead(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83869_GEN_CFG1_REG, &phyRegVal);

    if(status == SystemP_SUCCESS)
    {
        phyRegVal &= ~ETHPHY_DP83869_GEN_CFG1_G_1000BT_FD_ADV_MASK;
        phyRegVal &= ~ETHPHY_DP83869_GEN_CFG1_G_1000BT_HD_ADV_MASK;
        status = MDIO_phyRegWrite(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83869_GEN_CFG1_REG, phyRegVal);
    }
    return status;
}

static int32_t ETHPHY_DP83869_enableFastLinkDownDetection(ETHPHY_Attrs *attrs,
                                                          void         *data,
                                                          uint32_t     dataSize)
{
    int32_t status = SystemP_SUCCESS;
    uint16_t phyRegVal = 0;
    uint32_t mode;

    if((data == NULL) || (dataSize != sizeof(ETHPHY_DP83869_FastLinkDownDetectionConfig)))
    {
        status = SystemP_FAILURE;
    }

    if(status == SystemP_SUCCESS)
    {
        status = MDIO_phyExtRegRead(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83869_GEN_CFG_FLD_REG, &phyRegVal);
    }
    if(status == SystemP_SUCCESS)
    {
        mode = ((ETHPHY_DP83869_FastLinkDownDetectionConfig *)data)->mode;

        phyRegVal &= ~(ETHPHY_DP83869_GEN_CFG_FLD_FAST_LINK_DOWN_MODES_MASK);
        phyRegVal |= (mode & ETHPHY_DP83869_GEN_CFG_FLD_FAST_LINK_DOWN_MODES_MASK);
        status = MDIO_phyExtRegWrite(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83869_GEN_CFG_FLD_REG, phyRegVal);
    }
    return status;
}

static int32_t ETHPHY_DP83869_configureLedSource(ETHPHY_Attrs *attrs,
                                                 void         *data,
                                                 uint32_t     dataSize)
{
    int32_t status = SystemP_SUCCESS;
    uint16_t phyRegVal = 0;
    uint32_t ledNum;
    uint32_t mode;

    if((data == NULL) || (dataSize != sizeof(ETHPHY_DP83869_LedSourceConfig)))
    {
        status = SystemP_FAILURE;
    }

    if(status == SystemP_SUCCESS)
    {
        status = MDIO_phyRegRead(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83869_LEDS_CFG1_REG, &phyRegVal);
    }

    if(status == SystemP_SUCCESS)
    {
        ledNum = ((ETHPHY_DP83869_LedSourceConfig *)data)->ledNum;
        mode = ((ETHPHY_DP83869_LedSourceConfig *)data)->mode;

        switch(ledNum)
        {
            case ETHPHY_DP83869_LED0:
                phyRegVal &= ~(ETHPHY_DP83869_LEDS_CFG1_LED_SEL_MASK << 0);
                phyRegVal |= (mode << 0);
                break;

            case ETHPHY_DP83869_LED1:
                phyRegVal &= ~(ETHPHY_DP83869_LEDS_CFG1_LED_SEL_MASK << 4);
                phyRegVal |= (mode << 4);
                break;

            case ETHPHY_DP83869_LED2:
                phyRegVal &= ~(ETHPHY_DP83869_LEDS_CFG1_LED_SEL_MASK << 8);
                phyRegVal |= (mode << 8);
                break;

            case ETHPHY_DP83869_LED_GPIO:
                phyRegVal &= ~(ETHPHY_DP83869_LEDS_CFG1_LED_SEL_MASK << 12);
                phyRegVal |= (mode << 12);
                break;
        }

        status = MDIO_phyRegWrite(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83869_LEDS_CFG1_REG, phyRegVal);
    }

    return status;
}

static int32_t ETHPHY_DP83869_configureLedBlinkRate(ETHPHY_Attrs *attrs,
                                                    void         *data,
                                                    uint32_t     dataSize)
{
    int32_t status = SystemP_SUCCESS;
    uint16_t phyRegVal = 0;
    uint32_t blinkRate;

    if((data == NULL) || (dataSize != sizeof(ETHPHY_DP83869_LedBlinkRateConfig)))
    {
        status = SystemP_FAILURE;
    }

    if(status == SystemP_SUCCESS)
    {
        status = MDIO_phyRegRead(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83869_LEDS_CFG3_REG, &phyRegVal);
    }

    if(status == SystemP_SUCCESS)
    {
        blinkRate = ((ETHPHY_DP83869_LedBlinkRateConfig *)data)->rate;

        phyRegVal &= ~(ETHPHY_DP83869_LEDS_CFG3_LEDS_BLINK_RATE_MASK);
        phyRegVal |= blinkRate;

        status = MDIO_phyRegWrite(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83869_LEDS_CFG3_REG, phyRegVal);
    }
    return status;
}

static int32_t ETHPHY_DP83869_enableExtendedFdAbility(ETHPHY_Attrs *attrs)
{
    int32_t status = SystemP_FAILURE;
    uint16_t phyRegVal = 0;

    status = MDIO_phyRegRead(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83869_GEN_CFG4_REG, &phyRegVal);

    if(status == SystemP_SUCCESS)
    {
        phyRegVal |= ETHPHY_DP83869_GEN_CFG4_EXT_FD_ENABLE_MASK;
        status = MDIO_phyRegWrite(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83869_GEN_CFG4_REG, phyRegVal);
    }
    return status;
}

static int32_t ETHPHY_DP83869_enableOddNibbleDetection(ETHPHY_Attrs *attrs)
{
    int32_t status = SystemP_FAILURE;
    uint16_t phyRegVal = 0;

    status = MDIO_phyExtRegRead(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83869_G_100BT_REG0_REG, &phyRegVal);

    if(status == SystemP_SUCCESS)
    {
        phyRegVal |= (ETHPHY_DP83869_G_100BT_REG0_ODDNIBBLE_DET_ENABLE_MASK);
        status = MDIO_phyExtRegWrite(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83869_G_100BT_REG0_REG, phyRegVal);
    }

    return status;
}

static int32_t ETHPHY_DP83869_enableEnhancedIpgDetection(ETHPHY_Attrs *attrs)
{
    int32_t status = SystemP_FAILURE;
    uint16_t phyRegVal = 0;

    status = MDIO_phyExtRegRead(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83869_G_100BT_REG0_REG, &phyRegVal);

    if(status == SystemP_SUCCESS)
    {
        phyRegVal |= (ETHPHY_DP83869_G_100BT_REG0_ENHANCED_IPG_DET_ENABLE_MASK);
        status = MDIO_phyExtRegWrite(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83869_G_100BT_REG0_REG, phyRegVal);
    }

    return status;
}

static int32_t ETHPHY_DP83869_getSpeedDuplex(ETHPHY_Attrs *attrs,
                                                void         *data,
                                                uint32_t     dataSize)
{
    uint16_t phyRegVal = 0;
    int32_t status = SystemP_SUCCESS;
    ETHPHY_SpeedDuplexConfig *pSpeedDuplex = NULL;

    if((data == NULL) || (dataSize != sizeof(ETHPHY_DP83869_LedBlinkRateConfig)))
    {
        status = SystemP_FAILURE;
    }

    if(status == SystemP_SUCCESS)
    {
        status = MDIO_phyRegRead(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83869_PHY_STATUS_REG, &phyRegVal);
        pSpeedDuplex = (ETHPHY_SpeedDuplexConfig *)data;
    }

    if(status == SystemP_SUCCESS)
    {
        if((phyRegVal & ETHPHY_DP83869_PHY_STATUS_SPEED_MASK) == ETHPHY_DP83869_PHY_STATUS_SPEED_10M)  /*Speed is 10*/
        {
            if(phyRegVal & ETHPHY_DP83869_PHY_STATUS_DUPLEX_MASK)
            {
                pSpeedDuplex->config = ETHPHY_SPEED_DUPLEX_CONFIG_10FD;
            }

            else
            {
                pSpeedDuplex->config = ETHPHY_SPEED_DUPLEX_CONFIG_10HD;
            }
        }
        else if((phyRegVal & ETHPHY_DP83869_PHY_STATUS_SPEED_MASK) == ETHPHY_DP83869_PHY_STATUS_SPEED_100M)/*Speed is 100*/
        {
            if(phyRegVal & ETHPHY_DP83869_PHY_STATUS_DUPLEX_MASK)
            {
                pSpeedDuplex->config = ETHPHY_SPEED_DUPLEX_CONFIG_100FD;
            }

            else
            {
                pSpeedDuplex->config = ETHPHY_SPEED_DUPLEX_CONFIG_100HD;
            }
        }
        else if((phyRegVal & ETHPHY_DP83869_PHY_STATUS_SPEED_MASK) == ETHPHY_DP83869_PHY_STATUS_SPEED_1000M)/*Speed is 1000*/
        {
            if(phyRegVal & ETHPHY_DP83869_PHY_STATUS_DUPLEX_MASK)
            {
                pSpeedDuplex->config = ETHPHY_SPEED_DUPLEX_CONFIG_1000FD;
            }

            else
            {
                pSpeedDuplex->config = ETHPHY_SPEED_DUPLEX_CONFIG_1000HD;
            }
        }
        else
        {
            pSpeedDuplex->config = ETHPHY_SPEED_DUPLEX_CONFIG_INVALID;
        }
    }

    return status;
}

static int32_t ETHPHY_DP83869_setSpeedDuplex(ETHPHY_Attrs *attrs,
                                                void         *data,
                                                uint32_t     dataSize)
{
    uint16_t phyRegVal = 0;
    int32_t status = SystemP_SUCCESS;
    uint32_t config;

    if((data == NULL) || (dataSize != sizeof(ETHPHY_DP83869_LedBlinkRateConfig)))
    {
        status = SystemP_FAILURE;
    }

    if(status == SystemP_SUCCESS)
    {
        status = MDIO_phyRegRead(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83869_BMCR_REG, &phyRegVal);
    }

    if(status == SystemP_SUCCESS)
    {
        config = ((ETHPHY_SpeedDuplexConfig *)data)->config;

        switch(config)
        {
            case ETHPHY_SPEED_DUPLEX_CONFIG_AUTONEG:
                phyRegVal |= ETHPHY_DP83869_BMCR_AUTO_NEGOTIATE_ENABLE_MASK;
                break;

            case ETHPHY_SPEED_DUPLEX_CONFIG_10FD:
                phyRegVal &= ~(ETHPHY_DP83869_BMCR_AUTO_NEGOTIATE_ENABLE_MASK);
                phyRegVal &= ~(ETHPHY_DP83869_BMCR_SPEED_SEL_LSB_MASK);
                phyRegVal &= ~(ETHPHY_DP83869_BMCR_SPEED_SEL_MSB_MASK);
                phyRegVal |= ETHPHY_DP83869_BMCR_DUPLEX_EN_MASK;
                break;

            case ETHPHY_SPEED_DUPLEX_CONFIG_100FD:
                phyRegVal &= ~(ETHPHY_DP83869_BMCR_AUTO_NEGOTIATE_ENABLE_MASK);
                phyRegVal &= ~(ETHPHY_DP83869_BMCR_SPEED_SEL_MSB_MASK);
                phyRegVal |= ETHPHY_DP83869_BMCR_SPEED_SEL_LSB_MASK;
                phyRegVal |= ETHPHY_DP83869_BMCR_DUPLEX_EN_MASK;
                break;

            case ETHPHY_SPEED_DUPLEX_CONFIG_1000FD:
                phyRegVal &= ~(ETHPHY_DP83869_BMCR_AUTO_NEGOTIATE_ENABLE_MASK);
                phyRegVal |= ETHPHY_DP83869_BMCR_SPEED_SEL_MSB_MASK;
                phyRegVal |= ETHPHY_DP83869_BMCR_SPEED_SEL_LSB_MASK;
                phyRegVal |= ETHPHY_DP83869_BMCR_DUPLEX_EN_MASK;
                break;

            case ETHPHY_SPEED_DUPLEX_CONFIG_10HD:
                phyRegVal &= ~(ETHPHY_DP83869_BMCR_AUTO_NEGOTIATE_ENABLE_MASK);
                phyRegVal &= ~(ETHPHY_DP83869_BMCR_SPEED_SEL_LSB_MASK);
                phyRegVal &= ~(ETHPHY_DP83869_BMCR_SPEED_SEL_MSB_MASK);
                phyRegVal &= ~(ETHPHY_DP83869_BMCR_DUPLEX_EN_MASK);
                break;

            case ETHPHY_SPEED_DUPLEX_CONFIG_100HD:
                phyRegVal &= ~(ETHPHY_DP83869_BMCR_AUTO_NEGOTIATE_ENABLE_MASK);
                phyRegVal &= ~(ETHPHY_DP83869_BMCR_SPEED_SEL_MSB_MASK);
                phyRegVal |= ETHPHY_DP83869_BMCR_SPEED_SEL_LSB_MASK;
                phyRegVal &= ~(ETHPHY_DP83869_BMCR_DUPLEX_EN_MASK);
                break;

            case ETHPHY_SPEED_DUPLEX_CONFIG_1000HD:
                phyRegVal &= ~(ETHPHY_DP83869_BMCR_AUTO_NEGOTIATE_ENABLE_MASK);
                phyRegVal |= ETHPHY_DP83869_BMCR_SPEED_SEL_MSB_MASK;
                phyRegVal |= ETHPHY_DP83869_BMCR_SPEED_SEL_LSB_MASK;
                phyRegVal &= ~(ETHPHY_DP83869_BMCR_DUPLEX_EN_MASK);
                break;

            default:
                status = SystemP_FAILURE;
                break;
        }
    }

    if(status == SystemP_SUCCESS)
    {
        status = MDIO_phyRegWrite(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83869_BMCR_REG, phyRegVal);
    }
    return status;
}

static int32_t ETHPHY_DP83869_enableLowLatencyRgmii(ETHPHY_Attrs *attrs)
{
    int32_t status = SystemP_FAILURE;
    uint16_t phyRegVal = 0;

    status = MDIO_phyExtRegRead(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83869_RGMII_CTRL2_REG, &phyRegVal);

    if(status == SystemP_SUCCESS)
    {
        phyRegVal |= (ETHPHY_DP83869_RGMII_CTRL2_LOW_LATENCY_10_100_EN_MASK);
        status = MDIO_phyExtRegWrite(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83869_RGMII_CTRL2_REG, phyRegVal);
    }

    return status;
}

static int32_t ETHPHY_DP83869_setRxHalfFullThresholdRgmii(ETHPHY_Attrs *attrs,
                                                          void         *data,
                                                          uint32_t     dataSize)
{
    int32_t status = SystemP_SUCCESS;
    uint16_t phyRegVal = 0;
    uint16_t phyRegVal2 = 0;
    uint8_t value;
    uint8_t valueLsbs;
    uint8_t valueMsb;

    if((data == NULL) || (dataSize != sizeof(uint8_t)))
    {
        status = SystemP_FAILURE;
    }

    if(status == SystemP_SUCCESS)
    {
        status = MDIO_phyExtRegRead(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83869_RGMII_CTRL_REG, &phyRegVal);
        status = MDIO_phyExtRegRead(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83869_RGMII_CTRL2_REG, &phyRegVal2);
    }

    if(status == SystemP_SUCCESS)
    {
        value = (uint8_t)(*(uint8_t *)data);

        /*Store 2 LSBs from value(Bits[1:0]) into Bits[1:0] of valueLsbs*/
        valueLsbs = (value & 0x03);

        /*Store MSB from value(Bit[2]) into Bit[0] of valueMsb*/
        valueMsb = (value & 0x04);
        valueMsb >>= 2;

        /* Set lower 2 bits in Bits[6:5] of RGMII_CTRL and MSB in Bit[1] of RGMII_CTRL2*/
        phyRegVal &= ~(ETHPHY_DP83869_RGMII_CTRL_RX_HALF_FULL_THR_LSB_MASK);
        phyRegVal |= (valueLsbs << ETHPHY_DP83869_RGMII_CTRL_RX_HALF_FULL_THR_LSB_SHIFT);
        phyRegVal2 &= ~(ETHPHY_DP83869_RGMII_CTRL2_RX_HALF_FULL_THR_MSB_MASK);
        phyRegVal2 |= (valueMsb << ETHPHY_DP83869_RGMII_CTRL2_RX_HALF_FULL_THR_MSB_SHIFT);

        status = MDIO_phyExtRegWrite(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83869_RGMII_CTRL_REG, phyRegVal);
    }

    if(status == SystemP_SUCCESS)
    {
        status = MDIO_phyExtRegWrite(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83869_RGMII_CTRL2_REG, phyRegVal2);
    }

    return status;
}

static int32_t ETHPHY_DP83869_setTxHalfFullThresholdRgmii(ETHPHY_Attrs *attrs,
                                                          void         *data,
                                                          uint32_t     dataSize)
{
    int32_t status = SystemP_SUCCESS;
    uint16_t phyRegVal = 0;
    uint16_t phyRegVal2 = 0;
    uint8_t value;
    uint8_t valueLsbs;
    uint8_t valueMsb;

    if((data == NULL) || (dataSize != sizeof(uint8_t)))
    {
        status = SystemP_FAILURE;
    }

    if(status == SystemP_SUCCESS)
    {
        status = MDIO_phyExtRegRead(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83869_RGMII_CTRL_REG, &phyRegVal);
        status = MDIO_phyExtRegRead(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83869_RGMII_CTRL2_REG, &phyRegVal2);
    }

    if(status == SystemP_SUCCESS)
    {
        value = (uint8_t)(*(uint8_t *)data);

        /*Store 2 LSBs from value(Bits[1:0]) into Bits[1:0] of valueLsbs*/
        valueLsbs = (value & 0x03);

        /*Store MSB from value(Bit[2]) into Bit[0] of valueMsb*/
        valueMsb = (value & 0x04);
        valueMsb >>= 2;

        /* Set lower 2 bits in Bits[4:3] of RGMII_CTRL and MSB in Bit[0] of RGMII_CTRL2*/
        phyRegVal &= ~(ETHPHY_DP83869_RGMII_CTRL_TX_HALF_FULL_THR_LSB_MASK);
        phyRegVal |= (valueLsbs << ETHPHY_DP83869_RGMII_CTRL_TX_HALF_FULL_THR_LSB_SHIFT);
        phyRegVal2 &= ~(ETHPHY_DP83869_RGMII_CTRL2_TX_HALF_FULL_THR_MSB_MASK);
        phyRegVal2 |= (valueMsb << ETHPHY_DP83869_RGMII_CTRL2_TX_HALF_FULL_THR_MSB_SHIFT);

        status = MDIO_phyExtRegWrite(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83869_RGMII_CTRL_REG, phyRegVal);
    }

    if(status == SystemP_SUCCESS)
    {
        status = MDIO_phyExtRegWrite(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83869_RGMII_CTRL2_REG, phyRegVal2);
    }

    return status;
}

static int32_t ETHPHY_DP83869_getAutonegCompleteStatus(ETHPHY_Attrs *attrs,
                                                       void         *data,
                                                       uint32_t     dataSize)
{
    int32_t status = SystemP_SUCCESS;
    uint16_t phyRegVal = 0;
    uint8_t *value = (uint8_t *)data;

    if((data == NULL) || (dataSize != sizeof(uint8_t)))
    {
        status = SystemP_FAILURE;
    }

    if(status == SystemP_SUCCESS)
    {
        status = MDIO_phyRegRead(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83869_BMSR_REG, &phyRegVal);
    }

    if(status == SystemP_SUCCESS)
    {
        if(phyRegVal & ETHPHY_DP83869_BMSR_AUTONEG_COMP_MASK)
        {
            *value = 1;
        }
        else
        {
            *value = 0;
        }
    }

    return status;
}

static int32_t ETHPHY_DP83869_getLinkPartnerAutonegAbility(ETHPHY_Attrs *attrs,
                                                           void         *data,
                                                           uint32_t     dataSize)
{
    int32_t status = SystemP_SUCCESS;
    uint16_t phyRegVal = 0;
    uint8_t *value = (uint8_t *)data;

    if((data == NULL) || (dataSize != sizeof(uint8_t)))
    {
        status = SystemP_FAILURE;
    }

    if(status == SystemP_SUCCESS)
    {
        status = MDIO_phyRegRead(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83869_ANER_REG, &phyRegVal);
    }

    if(status == SystemP_SUCCESS)
    {
        if(phyRegVal & ETHPHY_DP83869_ANER_LP_AUTONEG_ABLE_MASK)
        {
            *value = 1;
        }
        else
        {
            *value = 0;
        }
    }

    return status;
}

static int32_t ETHPHY_DP83869_enableIeeePowerDownMode(ETHPHY_Attrs *attrs)
{
    int32_t status = SystemP_FAILURE;
    uint16_t phyRegVal = 0;

    status = MDIO_phyRegRead(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83869_BMCR_REG, &phyRegVal);

    if(status == SystemP_SUCCESS)
    {
        phyRegVal |= (ETHPHY_DP83869_BMCR_IEEE_POWER_DOWN_ENABLE_MASK);
        status = MDIO_phyRegWrite(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83869_BMCR_REG, phyRegVal);
    }

    return status;
}

static int32_t ETHPHY_DP83869_disableIeeePowerDownMode(ETHPHY_Attrs *attrs)
{
    int32_t status = SystemP_FAILURE;
    uint16_t phyRegVal = 0;

    status = MDIO_phyRegRead(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83869_BMCR_REG, &phyRegVal);

    if(status == SystemP_SUCCESS)
    {
        phyRegVal &= ~(ETHPHY_DP83869_BMCR_IEEE_POWER_DOWN_ENABLE_MASK);
        status = MDIO_phyRegWrite(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83869_BMCR_REG, phyRegVal);
    }

    return status;
}
