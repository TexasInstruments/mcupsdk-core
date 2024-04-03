/*
 * Copyright (C) 2021 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <test_icss_emac_loopback_utils.h>
#include <firmware/mii/PRU0_bin.h>
#include <firmware/mii/PRU1_bin.h>
#include "pruicss_pinmux.h"
#include <drivers/hw_include/cslr_soc.h>
#include <drivers/hw_include/hw_types.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/**
 *  Value to be set in ICSSM_PRU0_GPIO_OUT_CTRL and ICSSM_PRU1_GPIO_OUT_CTRL registers
 *  when using pins in MII mode.
 *  This configures PRG0_PRUx_GPIO0, PRG0_PRUx_GPIO1, PRG0_PRUx_GPIO2, PRG0_PRUx_GPIO3,
 *  PRG0_PRUx_GPIO4, PRG0_PRUx_GPIO5, PRG0_PRUx_GPIO6, PRG0_PRUx_GPIO8, PRG0_PRUx_GPIO9,
 *  PRG0_PRUx_GPIO10 and PRG0_PRUx_GPIO16 as input pins.
 **/
#define MSS_CTRL_ICSSM_PRU_GPIO_OUT_CTRL_VALUE  (0x0001077F)

#define JIRA_TEST_CASE_ID (320)

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void ICSS_EMAC_testBoardInit(void)
{
    ETHPHY_DP83869_LedSourceConfig ledConfig0;
    ETHPHY_DP83869_LedBlinkRateConfig ledBlinkConfig0;
    ETHPHY_DP83826E_LedSourceConfig ledConfig1;
    ETHPHY_DP83826E_LedBlinkRateConfig ledBlinkConfig1;

    Pinmux_config(gPruicssPinMuxCfg, PINMUX_DOMAIN_ID_MAIN);

    // Set bits for input pins in ICSSM_PRU0_GPIO_OUT_CTRL and ICSSM_PRU1_GPIO_OUT_CTRL registers
    HW_WR_REG32(CSL_MSS_CTRL_U_BASE + CSL_MSS_CTRL_ICSSM_PRU0_GPIO_OUT_CTRL, MSS_CTRL_ICSSM_PRU_GPIO_OUT_CTRL_VALUE);
    HW_WR_REG32(CSL_MSS_CTRL_U_BASE + CSL_MSS_CTRL_ICSSM_PRU1_GPIO_OUT_CTRL, MSS_CTRL_ICSSM_PRU_GPIO_OUT_CTRL_VALUE);

    DebugP_log("MII mode\r\n");

    /* PHY pin LED_0 as link */
    ledConfig0.ledNum = ETHPHY_DP83869_LED0;
    ledConfig0.mode = ETHPHY_DP83869_LED_MODE_100BTX_LINK_UP;
    ledConfig1.ledNum = ETHPHY_DP83826E_LED0;
    ledConfig1.mode = ETHPHY_DP83826E_LED_MODE_MII_LINK_100BT_FD;
    ETHPHY_command(gEthPhyHandle[CONFIG_ETHPHY0], ETHPHY_CMD_CONFIGURE_LED_SOURCE, (void *)&ledConfig0, sizeof(ledConfig0));
    ETHPHY_command(gEthPhyHandle[CONFIG_ETHPHY1], ETHPHY_CMD_CONFIGURE_LED_SOURCE, (void *)&ledConfig1, sizeof(ledConfig1));

    /* PHY pin LED_1 indication is on if 1G link established for PHY0, and if 10M speed id configured for PHY1 */
    ledConfig0.ledNum = ETHPHY_DP83869_LED1;
    ledConfig0.mode = ETHPHY_DP83869_LED_MODE_1000BT_LINK_UP;
    ledConfig1.ledNum = ETHPHY_DP83826E_LED1;
    ledConfig1.mode = ETHPHY_DP83826E_LED_MODE_SPEED_10BT;
    ETHPHY_command(gEthPhyHandle[CONFIG_ETHPHY0], ETHPHY_CMD_CONFIGURE_LED_SOURCE, (void *)&ledConfig0, sizeof(ledConfig0));
    ETHPHY_command(gEthPhyHandle[CONFIG_ETHPHY1], ETHPHY_CMD_CONFIGURE_LED_SOURCE, (void *)&ledConfig1, sizeof(ledConfig1));

    /* PHY pin LED_2 as Rx/Tx Activity */
    ledConfig0.ledNum = ETHPHY_DP83869_LED2;
    ledConfig0.mode = ETHPHY_DP83869_LED_MODE_LINK_OK_AND_BLINK_ON_RX_TX;
    ledConfig1.ledNum = ETHPHY_DP83826E_LED2;
    ledConfig1.mode = ETHPHY_DP83826E_LED_MODE_LINK_OK_AND_BLINK_ON_RX_TX;
    ETHPHY_command(gEthPhyHandle[CONFIG_ETHPHY0], ETHPHY_CMD_CONFIGURE_LED_SOURCE, (void *)&ledConfig0, sizeof(ledConfig0));
    ETHPHY_command(gEthPhyHandle[CONFIG_ETHPHY1], ETHPHY_CMD_CONFIGURE_LED_SOURCE, (void *)&ledConfig1, sizeof(ledConfig1));

    ledBlinkConfig0.rate = ETHPHY_DP83869_LED_BLINK_RATE_200_MS;
    ledBlinkConfig1.rate = ETHPHY_DP83826E_LED_BLINK_RATE_200_MS;
    ETHPHY_command(gEthPhyHandle[CONFIG_ETHPHY0], ETHPHY_CMD_CONFIGURE_LED_BLINK_RATE, (void *)&ledBlinkConfig0, sizeof(ledBlinkConfig0));
    ETHPHY_command(gEthPhyHandle[CONFIG_ETHPHY1], ETHPHY_CMD_CONFIGURE_LED_BLINK_RATE, (void *)&ledBlinkConfig1, sizeof(ledBlinkConfig1));

    /* Enable MII mode for DP83869 PHY */
    ETHPHY_command(gEthPhyHandle[CONFIG_ETHPHY0], ETHPHY_CMD_ENABLE_MII, NULL, 0);

    /* Disable 1G advertisement and sof-reset to restart auto-negotiation in case 1G link was establised */
    ETHPHY_command(gEthPhyHandle[CONFIG_ETHPHY0], ETHPHY_CMD_DISABLE_1000M_ADVERTISEMENT, NULL, 0);
    ETHPHY_command(gEthPhyHandle[CONFIG_ETHPHY0], ETHPHY_CMD_SOFT_RESTART, NULL, 0);

    /*Wait for PHY to come out of reset*/
    ClockP_sleep(1);
}

void ICSS_EMAC_testGetPruFwPtr(uint32_t *pru0FwPtr, uint32_t *pru0FwLength, uint32_t *pru1FwPtr, uint32_t *pru1FwLength)
{
    *pru0FwPtr = (uint32_t)&PRU0_b00[0];
    *pru0FwLength = (uint32_t)sizeof(PRU0_b00);
    *pru1FwPtr = (uint32_t)&PRU1_b00[0];
    *pru1FwLength = (uint32_t)sizeof(PRU1_b00);
}

uint32_t ICSS_EMAC_getTestCaseId(void)
{
    return (uint32_t)JIRA_TEST_CASE_ID;
}
