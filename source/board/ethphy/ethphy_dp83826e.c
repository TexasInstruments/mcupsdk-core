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

#include <board/ethphy/ethphy_dp83826e.h>
#include <drivers/mdio.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define ETHPHY_DP83826E_BMCR_REG                                        (0x0000)
#define ETHPHY_DP83826E_BMSR_REG                                        (0x0001)
#define ETHPHY_DP83826E_PHYIDR1_REG                                     (0x0002)
#define ETHPHY_DP83826E_ANER_REG                                        (0x0006)
#define ETHPHY_DP83826E_CR2_REG                                         (0x000A)
#define ETHPHY_DP83826E_CR3_REG                                         (0x000B)
#define ETHPHY_DP83826E_PHYSTS_REG                                      (0x0010)
#define ETHPHY_DP83826E_LEDCR_REG                                       (0x0018)
#define ETHPHY_DP83826E_PHYCR_REG                                       (0x0019)
#define ETHPHY_DP83826E_PHYRCR_REG                                      (0x001F)
#define ETHPHY_DP83826E_MLEDCR_REG                                      (0x0025)
#define ETHPHY_DP83826E_LED0_GPIO_CFG_REG                               (0x0303)
#define ETHPHY_DP83826E_LED1_GPIO_CFG_REG                               (0x0304)
#define ETHPHY_DP83826E_LED2_GPIO_CFG_REG                               (0x0305)
#define ETHPHY_DP83826E_LED3_GPIO_CFG_REG                               (0x0306)
#define ETHPHY_DP83826E_LEDCFG_REG                                      (0x0460)

#define ETHPHY_DP83826E_BMCR_IEEE_POWER_DOWN_ENABLE_MASK                (0x0800)
#define ETHPHY_DP83826E_BMCR_AUTO_NEGOTIATE_ENABLE_MASK                 (0x1000)
#define ETHPHY_DP83826E_BMCR_SPEED_SEL_MASK                             (0x2000)
#define ETHPHY_DP83826E_BMCR_DUPLEX_SEL_MASK                            (0x0100)
#define ETHPHY_DP83826E_BMSR_AUTONEG_COMP_MASK                          (0x0020)
#define ETHPHY_DP83826E_ANER_LP_AUTONEG_ABLE_MASK                       (0x0001)
#define ETHPHY_DP83826E_CR2_ENHANCED_IPG_DET_ENABLE_MASK                (0x0004)
#define ETHPHY_DP83826E_CR2_EXT_FD_ENABLE_MASK                          (0x0020)
#define ETHPHY_DP83826E_CR2_ODD_NIBBLE_DETECTION_DISABLE_MASK           (0x0002)
#define ETHPHY_DP83826E_CR3_FAST_LINK_DOWN_MODES_MASK                   (0x040F)
#define ETHPHY_DP83826E_PHYSTS_SPEED_MASK                               (0x0002)
#define ETHPHY_DP83826E_PHYSTS_DUPLEX_MASK                              (0x0004)
#define ETHPHY_DP83826E_LEDCR_LEDS_BLINK_RATE_MASK                      (0x0600)
#define ETHPHY_DP83826E_PHYCR_REG_AUTOMDIX_ENABLE_MASK                  (0x8000)
#define ETHPHY_DP83826E_PHYRCR_SW_RESTART_MASK                          (0x4000)
#define ETHPHY_DP83826E_MLEDCR_LED_SEL_MASK                             (0x000F)
#define ETHPHY_DP83826E_LEDx_GPIO_CFG_LEDx_GPIO_CTRL_MASK               (0x0007)
#define ETHPHY_DP83826E_LEDCFG_LED_SEL_MASK                             (0x000F)

#define ETHPHY_DP83826E_LEDCR_LEDS_BLINK_RATE_SHIFT                     (0x0009)
#define ETHPHY_DP83826E_MLEDCR_LED0_SHIFT                               (0x0003)
#define ETHPHY_DP83826E_LEDCFG_LED1_SHIFT                               (0x000C)
#define ETHPHY_DP83826E_LEDCFG_LED2_SHIFT                               (0x0008)
#define ETHPHY_DP83826E_LEDCFG_LED3_SHIFT                               (0x0004)

#define ETHPHY_DP83826E_PHYIDR1_REG_VALUE                               (0x2000)

#define ETHPHY_DP83826E_PHYSTS_SPEED_10M                                (0x0002)
#define ETHPHY_DP83826E_PHYSTS_SPEED_100M                               (0x0000)

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

int32_t ETHPHY_DP83826E_open(ETHPHY_Config *config, const ETHPHY_Params *params);

int32_t ETHPHY_DP83826E_command(ETHPHY_Config *config,
                               uint32_t command,
                               void *data,
                               uint32_t dataSize);

static int32_t ETHPHY_DP83826E_softRestart(ETHPHY_Attrs *attrs);

static int32_t ETHPHY_DP83826E_enableAutoMdix(ETHPHY_Attrs *attrs);

static int32_t ETHPHY_DP83826E_verifyIdentifierRegister(ETHPHY_Attrs *attrs);

static int32_t ETHPHY_DP83826E_enableFastLinkDownDetection(ETHPHY_Attrs *attrs,
                                                          void         *data,
                                                          uint32_t     dataSize);

static int32_t ETHPHY_DP83826E_configureLedSource(ETHPHY_Attrs *attrs,
                                                 void         *data,
                                                 uint32_t     dataSize);

static int32_t ETHPHY_DP83826E_configureLedBlinkRate(ETHPHY_Attrs *attrs,
                                                    void         *data,
                                                    uint32_t     dataSize);

static int32_t ETHPHY_DP83826E_enableExtendedFdAbility(ETHPHY_Attrs *attrs);

static int32_t ETHPHY_DP83826E_enableOddNibbleDetection(ETHPHY_Attrs *attrs);

static int32_t ETHPHY_DP83826E_enableEnhancedIpgDetection(ETHPHY_Attrs *attrs);

static int32_t ETHPHY_DP83826E_getSpeedDuplex(ETHPHY_Attrs *attrs,
                                                void         *data,
                                                uint32_t     dataSize);

static int32_t ETHPHY_DP83826E_setSpeedDuplex(ETHPHY_Attrs *attrs,
                                                void         *data,
                                                uint32_t     dataSize);

static int32_t ETHPHY_DP83826E_getAutonegCompleteStatus(ETHPHY_Attrs *attrs,
                                                       void         *data,
                                                       uint32_t     dataSize);


static int32_t ETHPHY_DP83826E_getLinkPartnerAutonegAbility(ETHPHY_Attrs *attrs,
                                                           void         *data,
                                                           uint32_t     dataSize);

static int32_t ETHPHY_DP83826E_enableIeeePowerDownMode(ETHPHY_Attrs *attrs);

static int32_t ETHPHY_DP83826E_disableIeeePowerDownMode(ETHPHY_Attrs *attrs);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

ETHPHY_Fxns gEthPhyFxns_DP83826E =
{
    .openFxn    = ETHPHY_DP83826E_open,
    .closeFxn   = NULL,
    .commandFxn = ETHPHY_DP83826E_command,
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t ETHPHY_DP83826E_open(ETHPHY_Config *config, const ETHPHY_Params *params)
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

int32_t ETHPHY_DP83826E_command(ETHPHY_Config *config,
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
            /* This command is not available for DP83826E PHY */
            status = SystemP_FAILURE;
            break;

        case ETHPHY_CMD_SOFT_RESTART :
            status = ETHPHY_DP83826E_softRestart(attrs);
            break;

        case ETHPHY_CMD_ENABLE_AUTO_MDIX :
            status = ETHPHY_DP83826E_enableAutoMdix(attrs);
            break;

        case ETHPHY_CMD_VERIFY_IDENTIFIER_REGISTER :
            status = ETHPHY_DP83826E_verifyIdentifierRegister(attrs);
            break;

        case ETHPHY_CMD_DISABLE_1000M_ADVERTISEMENT :
            /* This command is not available for DP83826E PHY */
            status = SystemP_FAILURE;
            break;

        case ETHPHY_CMD_ENABLE_FAST_LINK_DOWN_DETECTION :
            status = ETHPHY_DP83826E_enableFastLinkDownDetection(attrs, data, dataSize);
            break;

        case ETHPHY_CMD_CONFIGURE_LED_SOURCE :
            status = ETHPHY_DP83826E_configureLedSource(attrs, data, dataSize);
            break;

        case ETHPHY_CMD_CONFIGURE_LED_BLINK_RATE :
            status = ETHPHY_DP83826E_configureLedBlinkRate(attrs, data, dataSize);
            break;

        case ETHPHY_CMD_ENABLE_EXTENDED_FD_ABILITY :
            status = ETHPHY_DP83826E_enableExtendedFdAbility(attrs);
            break;

        case ETHPHY_CMD_ENABLE_ODD_NIBBLE_DETECTION :
            status = ETHPHY_DP83826E_enableOddNibbleDetection(attrs);
            break;

        case ETHPHY_CMD_ENABLE_ENHANCED_IPG_DETECTION :
            status = ETHPHY_DP83826E_enableEnhancedIpgDetection(attrs);
            break;

        case ETHPHY_CMD_GET_LINK_STATUS :
            status = MDIO_phyLinkStatus(attrs->mdioBaseAddress, attrs->phyAddress);
            break;

        case ETHPHY_CMD_GET_SPEED_AND_DUPLEX_CONFIG :
            status = ETHPHY_DP83826E_getSpeedDuplex(attrs, data, dataSize);
            break;

        case ETHPHY_CMD_SET_SPEED_AND_DUPLEX_CONFIG :
            status = ETHPHY_DP83826E_setSpeedDuplex(attrs, data, dataSize);
            break;

        case ETHPHY_CMD_ENABLE_LOW_LATENCY_10M_100M_RGMII:
            /* This command is not available for DP83826E PHY */
            status = SystemP_FAILURE;
            break;

        case ETHPHY_CMD_SET_RX_HALF_FULL_THRESHOLD_RGMII:
            /* This command is not available for DP83826E PHY */
            status = SystemP_FAILURE;
            break;

        case ETHPHY_CMD_SET_TX_HALF_FULL_THRESHOLD_RGMII:
            /* This command is not available for DP83826E PHY */
            status = SystemP_FAILURE;
            break;

        case ETHPHY_CMD_GET_AUTONEG_COMPLETE_STATUS:
            status = ETHPHY_DP83826E_getAutonegCompleteStatus(attrs, data, dataSize);
            break;

        case ETHPHY_CMD_GET_LINK_PARTNER_AUTONEG_ABILITY:
            status = ETHPHY_DP83826E_getLinkPartnerAutonegAbility(attrs, data, dataSize);
            break;

        case ETHPHY_CMD_ENABLE_IEEE_POWER_DOWN:
            status = ETHPHY_DP83826E_enableIeeePowerDownMode(attrs);
            break;

        case ETHPHY_CMD_DISABLE_IEEE_POWER_DOWN:
            status = ETHPHY_DP83826E_disableIeeePowerDownMode(attrs);
            break;

        default:
            status = SystemP_FAILURE;
            break;
        }
    }

    return status;
}

static int32_t ETHPHY_DP83826E_softRestart(ETHPHY_Attrs *attrs)
{
    int32_t status = SystemP_FAILURE;
    uint16_t phyRegVal = 0;

    status = MDIO_phyRegRead(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83826E_PHYRCR_REG, &phyRegVal);

    if(status == SystemP_SUCCESS)
    {
        phyRegVal |= ETHPHY_DP83826E_PHYRCR_SW_RESTART_MASK;
        status = MDIO_phyRegWrite(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83826E_PHYRCR_REG, phyRegVal);
    }

    return status;
}

static int32_t ETHPHY_DP83826E_enableAutoMdix(ETHPHY_Attrs *attrs)
{
    int32_t status = SystemP_FAILURE;
    uint16_t phyRegVal = 0;

    status = MDIO_phyRegRead(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83826E_PHYCR_REG, &phyRegVal);

    if(status == SystemP_SUCCESS)
    {
        phyRegVal |= ETHPHY_DP83826E_PHYCR_REG_AUTOMDIX_ENABLE_MASK;
        status = MDIO_phyRegWrite(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83826E_PHYCR_REG, phyRegVal);
    }

    return status;
}

static int32_t ETHPHY_DP83826E_verifyIdentifierRegister(ETHPHY_Attrs *attrs)
{
    int32_t status = SystemP_FAILURE;
    uint16_t phyRegVal = 0;

    status = MDIO_phyRegRead(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83826E_PHYIDR1_REG, &phyRegVal);

    if(status == SystemP_SUCCESS)
    {
        if(phyRegVal == ETHPHY_DP83826E_PHYIDR1_REG_VALUE)
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

static int32_t ETHPHY_DP83826E_enableFastLinkDownDetection(ETHPHY_Attrs *attrs,
                                                          void         *data,
                                                          uint32_t     dataSize)
{
    int32_t status = SystemP_SUCCESS;
    uint16_t phyRegVal = 0;
    uint32_t mode;

    if((data == NULL) || (dataSize != sizeof(ETHPHY_DP83826E_FastLinkDownDetectionConfig)))
    {
        status = SystemP_FAILURE;
    }

    if(status == SystemP_SUCCESS)
    {
        status = MDIO_phyRegRead(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83826E_CR3_REG, &phyRegVal);
    }
    if(status == SystemP_SUCCESS)
    {
        mode = ((ETHPHY_DP83826E_FastLinkDownDetectionConfig *)data)->mode;

        phyRegVal &= ~(ETHPHY_DP83826E_CR3_FAST_LINK_DOWN_MODES_MASK);
        phyRegVal |= (mode & ETHPHY_DP83826E_CR3_FAST_LINK_DOWN_MODES_MASK);
        status = MDIO_phyRegWrite(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83826E_CR3_REG, phyRegVal);
    }
    return status;
}

static int32_t ETHPHY_DP83826E_configureLedSource(ETHPHY_Attrs *attrs,
                                                 void         *data,
                                                 uint32_t     dataSize)
{
    int32_t status = SystemP_SUCCESS;
    uint16_t phyRegVal = 0;
    uint32_t ledNum;
    uint32_t mode;

    if((data == NULL) || (dataSize != sizeof(ETHPHY_DP83826E_LedSourceConfig)))
    {
        status = SystemP_FAILURE;
    }

    if(status == SystemP_SUCCESS)
    {
        ledNum = ((ETHPHY_DP83826E_LedSourceConfig *)data)->ledNum;
        mode = ((ETHPHY_DP83826E_LedSourceConfig *)data)->mode;

        switch(ledNum)
        {
            case ETHPHY_DP83826E_LED0:
                status = MDIO_phyExtRegRead(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83826E_LED0_GPIO_CFG_REG, &phyRegVal);
                phyRegVal &= ~(ETHPHY_DP83826E_LEDx_GPIO_CFG_LEDx_GPIO_CTRL_MASK);
                break;

            case ETHPHY_DP83826E_LED1:
                status = MDIO_phyExtRegRead(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83826E_LED1_GPIO_CFG_REG, &phyRegVal);
                phyRegVal &= ~(ETHPHY_DP83826E_LEDx_GPIO_CFG_LEDx_GPIO_CTRL_MASK);
                break;

            case ETHPHY_DP83826E_LED2:
                status = MDIO_phyExtRegRead(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83826E_LED2_GPIO_CFG_REG, &phyRegVal);
                phyRegVal &= ~(ETHPHY_DP83826E_LEDx_GPIO_CFG_LEDx_GPIO_CTRL_MASK);
                break;

            case ETHPHY_DP83826E_LED3:
                status = MDIO_phyExtRegRead(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83826E_LED3_GPIO_CFG_REG, &phyRegVal);
                phyRegVal &= ~(ETHPHY_DP83826E_LEDx_GPIO_CFG_LEDx_GPIO_CTRL_MASK);
                break;
        }
    }

    if(status == SystemP_SUCCESS)
    {
        switch(ledNum)
        {
            case ETHPHY_DP83826E_LED0:
                status = MDIO_phyExtRegWrite(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83826E_LED0_GPIO_CFG_REG, phyRegVal);
                break;

            case ETHPHY_DP83826E_LED1:
                status = MDIO_phyExtRegWrite(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83826E_LED1_GPIO_CFG_REG, phyRegVal);
                break;

            case ETHPHY_DP83826E_LED2:
                status = MDIO_phyExtRegWrite(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83826E_LED2_GPIO_CFG_REG, phyRegVal);
                break;

            case ETHPHY_DP83826E_LED3:
                status = MDIO_phyExtRegWrite(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83826E_LED3_GPIO_CFG_REG, phyRegVal);
                break;
        }
    }

    if(status == SystemP_SUCCESS)
    {
        switch(ledNum)
        {
            case ETHPHY_DP83826E_LED0:
                status = MDIO_phyExtRegRead(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83826E_MLEDCR_REG, &phyRegVal);
                phyRegVal &= ~(ETHPHY_DP83826E_MLEDCR_LED_SEL_MASK << ETHPHY_DP83826E_MLEDCR_LED0_SHIFT);
                phyRegVal |= (mode << ETHPHY_DP83826E_MLEDCR_LED0_SHIFT);
                break;

            case ETHPHY_DP83826E_LED1:
                status = MDIO_phyExtRegRead(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83826E_LEDCFG_REG, &phyRegVal);
                phyRegVal &= ~(ETHPHY_DP83826E_LEDCFG_LED_SEL_MASK << ETHPHY_DP83826E_LEDCFG_LED1_SHIFT);
                phyRegVal |= (mode << ETHPHY_DP83826E_LEDCFG_LED1_SHIFT);
                break;

            case ETHPHY_DP83826E_LED2:
                status = MDIO_phyExtRegRead(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83826E_LEDCFG_REG, &phyRegVal);
                phyRegVal &= ~(ETHPHY_DP83826E_LEDCFG_LED_SEL_MASK << ETHPHY_DP83826E_LEDCFG_LED2_SHIFT);
                phyRegVal |= (mode << ETHPHY_DP83826E_LEDCFG_LED2_SHIFT);
                break;

            case ETHPHY_DP83826E_LED3:
                status = MDIO_phyExtRegRead(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83826E_LEDCFG_REG, &phyRegVal);
                phyRegVal &= ~(ETHPHY_DP83826E_LEDCFG_LED_SEL_MASK << ETHPHY_DP83826E_LEDCFG_LED3_SHIFT);
                phyRegVal |= (mode << ETHPHY_DP83826E_LEDCFG_LED3_SHIFT);
                break;
        }
    }

    if(status == SystemP_SUCCESS)
    {
        switch(ledNum)
        {
            case ETHPHY_DP83826E_LED0:
                status = MDIO_phyExtRegWrite(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83826E_MLEDCR_REG, phyRegVal);
                break;

            case ETHPHY_DP83826E_LED1:
            case ETHPHY_DP83826E_LED2:
            case ETHPHY_DP83826E_LED3:
                status = MDIO_phyExtRegWrite(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83826E_LEDCFG_REG, phyRegVal);
                break;
        }
    }
    return status;
}

static int32_t ETHPHY_DP83826E_configureLedBlinkRate(ETHPHY_Attrs *attrs,
                                                    void         *data,
                                                    uint32_t     dataSize)
{
    int32_t status = SystemP_SUCCESS;
    uint16_t phyRegVal = 0;
    uint32_t blinkRate;

    if((data == NULL) || (dataSize != sizeof(ETHPHY_DP83826E_LedBlinkRateConfig)))
    {
        status = SystemP_FAILURE;
    }

    if(status == SystemP_SUCCESS)
    {
        status = MDIO_phyRegRead(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83826E_LEDCR_REG, &phyRegVal);
    }

    if(status == SystemP_SUCCESS)
    {
        blinkRate = ((ETHPHY_DP83826E_LedBlinkRateConfig *)data)->rate;

        phyRegVal &= ~(ETHPHY_DP83826E_LEDCR_LEDS_BLINK_RATE_MASK);
        phyRegVal |= (blinkRate << ETHPHY_DP83826E_LEDCR_LEDS_BLINK_RATE_SHIFT);

        status = MDIO_phyRegWrite(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83826E_LEDCR_REG, phyRegVal);
    }
    return status;
}

static int32_t ETHPHY_DP83826E_enableExtendedFdAbility(ETHPHY_Attrs *attrs)
{
    int32_t status = SystemP_FAILURE;
    uint16_t phyRegVal = 0;

    status = MDIO_phyRegRead(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83826E_CR2_REG, &phyRegVal);

    if(status == SystemP_SUCCESS)
    {
        phyRegVal |= ETHPHY_DP83826E_CR2_EXT_FD_ENABLE_MASK;
        status = MDIO_phyRegWrite(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83826E_CR2_REG, phyRegVal);
    }
    return status;
}

static int32_t ETHPHY_DP83826E_enableOddNibbleDetection(ETHPHY_Attrs *attrs)
{
    int32_t status = SystemP_FAILURE;
    uint16_t phyRegVal = 0;

    status = MDIO_phyRegRead(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83826E_CR2_REG, &phyRegVal);

    if(status == SystemP_SUCCESS)
    {
        phyRegVal &= ~(ETHPHY_DP83826E_CR2_ODD_NIBBLE_DETECTION_DISABLE_MASK);
        status = MDIO_phyRegWrite(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83826E_CR2_REG, phyRegVal);
    }

    return status;
}

static int32_t ETHPHY_DP83826E_enableEnhancedIpgDetection(ETHPHY_Attrs *attrs)
{
    int32_t status = SystemP_FAILURE;
    uint16_t phyRegVal = 0;

    status = MDIO_phyRegRead(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83826E_CR2_REG, &phyRegVal);

    if(status == SystemP_SUCCESS)
    {
        phyRegVal |= (ETHPHY_DP83826E_CR2_ENHANCED_IPG_DET_ENABLE_MASK);
        status = MDIO_phyRegWrite(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83826E_CR2_REG, phyRegVal);
    }

    return status;
}

static int32_t ETHPHY_DP83826E_getSpeedDuplex(ETHPHY_Attrs *attrs,
                                                void         *data,
                                                uint32_t     dataSize)
{
    uint16_t phyRegVal = 0;
    int32_t status = SystemP_SUCCESS;
    ETHPHY_SpeedDuplexConfig *pSpeedDuplex = NULL;

    if((data == NULL) || (dataSize != sizeof(ETHPHY_DP83826E_LedBlinkRateConfig)))
    {
        status = SystemP_FAILURE;
    }

    if(status == SystemP_SUCCESS)
    {
        status = MDIO_phyRegRead(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83826E_PHYSTS_REG, &phyRegVal);
        pSpeedDuplex = (ETHPHY_SpeedDuplexConfig *)data;
    }

    if(status == SystemP_SUCCESS)
    {
        /* Speed is 10M */
        if(phyRegVal & ETHPHY_DP83826E_PHYSTS_SPEED_MASK)
        {
            if(phyRegVal & ETHPHY_DP83826E_PHYSTS_DUPLEX_MASK)
            {
                pSpeedDuplex->config = ETHPHY_SPEED_DUPLEX_CONFIG_10FD;
            }
            else
            {
                pSpeedDuplex->config = ETHPHY_SPEED_DUPLEX_CONFIG_10HD;
            }
        }
        /* Speed is 100M */
        else
        {
            if(phyRegVal & ETHPHY_DP83826E_PHYSTS_DUPLEX_MASK)
            {
                pSpeedDuplex->config = ETHPHY_SPEED_DUPLEX_CONFIG_100FD;
            }
            else
            {
                pSpeedDuplex->config = ETHPHY_SPEED_DUPLEX_CONFIG_100HD;
            }
        }
    }

    return status;
}

static int32_t ETHPHY_DP83826E_setSpeedDuplex(ETHPHY_Attrs *attrs,
                                                void         *data,
                                                uint32_t     dataSize)
{
    uint16_t phyRegVal = 0;
    int32_t status = SystemP_SUCCESS;
    uint32_t config;

    if((data == NULL) || (dataSize != sizeof(ETHPHY_DP83826E_LedBlinkRateConfig)))
    {
        status = SystemP_FAILURE;
    }

    if(status == SystemP_SUCCESS)
    {
        status = MDIO_phyRegRead(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83826E_BMCR_REG, &phyRegVal);
    }

    if(status == SystemP_SUCCESS)
    {
        config = ((ETHPHY_SpeedDuplexConfig *)data)->config;

        switch(config)
        {
            case ETHPHY_SPEED_DUPLEX_CONFIG_AUTONEG:
                phyRegVal |= ETHPHY_DP83826E_BMCR_AUTO_NEGOTIATE_ENABLE_MASK;
                break;

            case ETHPHY_SPEED_DUPLEX_CONFIG_10FD:
                phyRegVal &= ~(ETHPHY_DP83826E_BMCR_AUTO_NEGOTIATE_ENABLE_MASK);
                phyRegVal &= ~(ETHPHY_DP83826E_BMCR_SPEED_SEL_MASK);
                phyRegVal |= ETHPHY_DP83826E_BMCR_DUPLEX_SEL_MASK;
                break;

            case ETHPHY_SPEED_DUPLEX_CONFIG_100FD:
                phyRegVal &= ~(ETHPHY_DP83826E_BMCR_AUTO_NEGOTIATE_ENABLE_MASK);
                phyRegVal |= ETHPHY_DP83826E_BMCR_SPEED_SEL_MASK;
                phyRegVal |= ETHPHY_DP83826E_BMCR_DUPLEX_SEL_MASK;
                break;

            case ETHPHY_SPEED_DUPLEX_CONFIG_10HD:
                phyRegVal &= ~(ETHPHY_DP83826E_BMCR_AUTO_NEGOTIATE_ENABLE_MASK);
                phyRegVal &= ~(ETHPHY_DP83826E_BMCR_SPEED_SEL_MASK);
                phyRegVal &= ~(ETHPHY_DP83826E_BMCR_DUPLEX_SEL_MASK);
                break;

            case ETHPHY_SPEED_DUPLEX_CONFIG_100HD:
                phyRegVal &= ~(ETHPHY_DP83826E_BMCR_AUTO_NEGOTIATE_ENABLE_MASK);
                phyRegVal |= ETHPHY_DP83826E_BMCR_SPEED_SEL_MASK;
                phyRegVal &= ~(ETHPHY_DP83826E_BMCR_DUPLEX_SEL_MASK);
                break;

            default:
                status = SystemP_FAILURE;
                break;
        }
    }

    if(status == SystemP_SUCCESS)
    {
        status = MDIO_phyRegWrite(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83826E_BMCR_REG, phyRegVal);
    }
    return status;
}


static int32_t ETHPHY_DP83826E_getAutonegCompleteStatus(ETHPHY_Attrs *attrs,
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
        status = MDIO_phyRegRead(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83826E_BMSR_REG, &phyRegVal);
    }

    if(status == SystemP_SUCCESS)
    {
        if(phyRegVal & ETHPHY_DP83826E_BMSR_AUTONEG_COMP_MASK)
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

static int32_t ETHPHY_DP83826E_getLinkPartnerAutonegAbility(ETHPHY_Attrs *attrs,
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
        status = MDIO_phyRegRead(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83826E_ANER_REG, &phyRegVal);
    }

    if(status == SystemP_SUCCESS)
    {
        if(phyRegVal & ETHPHY_DP83826E_ANER_LP_AUTONEG_ABLE_MASK)
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

static int32_t ETHPHY_DP83826E_enableIeeePowerDownMode(ETHPHY_Attrs *attrs)
{
    int32_t status = SystemP_FAILURE;
    uint16_t phyRegVal = 0;

    status = MDIO_phyRegRead(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83826E_BMCR_REG, &phyRegVal);

    if(status == SystemP_SUCCESS)
    {
        phyRegVal |= (ETHPHY_DP83826E_BMCR_IEEE_POWER_DOWN_ENABLE_MASK);
        status = MDIO_phyRegWrite(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83826E_BMCR_REG, phyRegVal);
    }

    return status;
}

static int32_t ETHPHY_DP83826E_disableIeeePowerDownMode(ETHPHY_Attrs *attrs)
{
    int32_t status = SystemP_FAILURE;
    uint16_t phyRegVal = 0;

    status = MDIO_phyRegRead(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83826E_BMCR_REG, &phyRegVal);

    if(status == SystemP_SUCCESS)
    {
        phyRegVal &= ~(ETHPHY_DP83826E_BMCR_IEEE_POWER_DOWN_ENABLE_MASK);
        status = MDIO_phyRegWrite(attrs->mdioBaseAddress, NULL, attrs->phyAddress, ETHPHY_DP83826E_BMCR_REG, phyRegVal);
    }

    return status;
}
