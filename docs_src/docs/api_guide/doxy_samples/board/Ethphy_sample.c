#include <stddef.h>
//! [include]
#include <board/ethphy.h>
#include <board/ethphy/icss_emac/ethphy_dp83869.h>
//! [include]

#define CONFIG_ETHPHY0 (0U)
#define CONFIG_ETHPHY_NUM_INSTANCES (1U)

ETHPHY_Handle gEthPhyHandle[CONFIG_ETHPHY_NUM_INSTANCES];

void ethphy_sample_1(void)
{
    int32_t status;
//! [ethphy_sample_1]

    /*Use CONFIG_ETHPHY0 macro as an index for the handle array*/
    /* Select MII mode */
    status = ETHPHY_command(gEthPhyHandle[CONFIG_ETHPHY0], ETHPHY_CMD_ENABLE_MII, NULL, 0);
    (void) status; /* Typically app will take action based on error. Typecast to void to suppress warning
                    * of variable set but not used
                    */
    /* Soft-reset PHY */
    status = ETHPHY_command(gEthPhyHandle[CONFIG_ETHPHY0], ETHPHY_CMD_SOFT_RESTART, NULL, 0);
    (void) status; /* Typically app will take action based on error. Typecast to void to suppress warning
                    * of variable set but not used
                    */
//! [ethphy_sample_1]
}

void ethphy_sample_2(void)
{
    int32_t                         status;

//! [ethphy_sample_2]
    ETHPHY_DP83869_LedSourceConfig  ledConfig;

    /*Use CONFIG_ETHPHY0 macro as an index for the handle array*/
    /* Configure source of PHY LEDs */

    /* PHY pin LED_0 as link for fast link detection */
    ledConfig.ledNum = ETHPHY_DP83869_LED0;
    ledConfig.mode = ETHPHY_DP83869_LED_MODE_LINK_OK;
    status = ETHPHY_command(gEthPhyHandle[CONFIG_ETHPHY0], ETHPHY_CMD_CONFIGURE_LED_SOURCE, (void *)&ledConfig, sizeof(ledConfig));
    (void) status; /* Typically app will take action based on error. Typecast to void to suppress warning
                    * of variable set but not used
                    */
    /* PHY pin LED_1 as 1G link established */
    ledConfig.ledNum = ETHPHY_DP83869_LED1;
    ledConfig.mode = ETHPHY_DP83869_LED_MODE_1000BT_LINK_UP;
    status = ETHPHY_command(gEthPhyHandle[CONFIG_ETHPHY0], ETHPHY_CMD_CONFIGURE_LED_SOURCE, (void *)&ledConfig, sizeof(ledConfig));
    (void) status; /* Typically app will take action based on error. Typecast to void to suppress warning
                    * of variable set but not used
                    */
    /* PHY pin LED_2 as Rx/Tx Activity */
    ledConfig.ledNum = ETHPHY_DP83869_LED2;
    ledConfig.mode = ETHPHY_DP83869_LED_MODE_LINK_OK_AND_BLINK_ON_RX_TX;
    status = ETHPHY_command(gEthPhyHandle[CONFIG_ETHPHY0], ETHPHY_CMD_CONFIGURE_LED_SOURCE, (void *)&ledConfig, sizeof(ledConfig));
    (void) status; /* Typically app will take action based on error. Typecast to void to suppress warning
                    * of variable set but not used
                    */
    /* PHY pin LED_3 as 100M link established */
    ledConfig.ledNum = ETHPHY_DP83869_LED_GPIO;
    ledConfig.mode = ETHPHY_DP83869_LED_MODE_10_OR_100BT_LINK_UP;
    status = ETHPHY_command(gEthPhyHandle[CONFIG_ETHPHY0], ETHPHY_CMD_CONFIGURE_LED_SOURCE, (void *)&ledConfig, sizeof(ledConfig));
    (void) status; /* Typically app will take action based on error. Typecast to void to suppress warning
                    * of variable set but not used
                    */
//! [ethphy_sample_2]
}


