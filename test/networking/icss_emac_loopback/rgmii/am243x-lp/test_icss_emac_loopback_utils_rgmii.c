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
#include <firmware/rgmii/PRU0_bin.h>
#include <firmware/rgmii/PRU1_bin.h>
#include "pruicss_pinmux.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define JIRA_TEST_CASE_ID (878)

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void ICSS_EMAC_testBoardInit(void)
{
    uint8_t         data;

    Pinmux_config(gPruicssPinMuxCfg, PINMUX_DOMAIN_ID_MAIN);

    DebugP_log("RGMII mode\r\n");

    ETHPHY_command(gEthPhyHandle[CONFIG_ETHPHY0], ETHPHY_CMD_ENABLE_LOW_LATENCY_10M_100M_RGMII, NULL, 0);
    ETHPHY_command(gEthPhyHandle[CONFIG_ETHPHY1], ETHPHY_CMD_ENABLE_LOW_LATENCY_10M_100M_RGMII, NULL, 0);

    data = 0x1;
    ETHPHY_command(gEthPhyHandle[CONFIG_ETHPHY0], ETHPHY_CMD_SET_RX_HALF_FULL_THRESHOLD_RGMII, &data, sizeof(data));
    ETHPHY_command(gEthPhyHandle[CONFIG_ETHPHY1], ETHPHY_CMD_SET_RX_HALF_FULL_THRESHOLD_RGMII, &data, sizeof(data));

    data = 0x1;
    ETHPHY_command(gEthPhyHandle[CONFIG_ETHPHY0], ETHPHY_CMD_SET_TX_HALF_FULL_THRESHOLD_RGMII, &data, sizeof(data));
    ETHPHY_command(gEthPhyHandle[CONFIG_ETHPHY1], ETHPHY_CMD_SET_TX_HALF_FULL_THRESHOLD_RGMII, &data, sizeof(data));

    /* Disable 1G advertisement */
    ETHPHY_command(gEthPhyHandle[CONFIG_ETHPHY0], ETHPHY_CMD_DISABLE_1000M_ADVERTISEMENT, NULL, 0);
    ETHPHY_command(gEthPhyHandle[CONFIG_ETHPHY1], ETHPHY_CMD_DISABLE_1000M_ADVERTISEMENT, NULL, 0);

    /* Soft-reset PHY */
    ETHPHY_command(gEthPhyHandle[CONFIG_ETHPHY0], ETHPHY_CMD_SOFT_RESTART, NULL, 0);
    ETHPHY_command(gEthPhyHandle[CONFIG_ETHPHY1], ETHPHY_CMD_SOFT_RESTART, NULL, 0);

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
