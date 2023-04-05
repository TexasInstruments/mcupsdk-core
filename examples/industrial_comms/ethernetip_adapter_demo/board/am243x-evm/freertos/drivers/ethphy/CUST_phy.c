/*!
 *  \file CUST_phy.c
 *
 *  \brief
 *  Provides initialization of custom PHY's.
 *
 *  \author
 *  KUNBUS GmbH
 *
 *  \date
 *  2022-03-16
 *
 *  \copyright
 *  Copyright (c) 2022, KUNBUS GmbH<br /><br />
 *  All rights reserved.<br />
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:<br />
 *  <ol>
 *  <li>Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimer.</li>
 *  <li>Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.</li>
 *  <li>Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.</li>
 *  </ol>
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <ti_board_open_close.h>

#include <drivers/mdio/v0/mdio.h>

#include <drivers/ethphy/CUST_phy.h>

#include <drivers/pinmux/am64x_am243x/pinmux.h>

#include <drivers/CUST_drivers.h>

#include <CUST_PHY_base.h>

ETHPHY_Fxns gEthPhyFxns_DP83869_0 = { .openFxn    = NULL,
                                      .closeFxn   = NULL,
                                      .commandFxn = NULL};

ETHPHY_Fxns gEthPhyFxns_DP83869_1 = { .openFxn    = NULL,
                                      .closeFxn   = NULL,
                                      .commandFxn = NULL };

/*!
* <!-- Description: -->
*
* \brief
* Custom PHY initialization.
*
* \return     uint32_t                    Error code.
*
* \retval     CUST_PHY_eERR_NOERROR       Success.
* \retval     CUST_PHY_eERR_GENERALERROR  Failed.
*
*/
uint32_t CUST_PHY_init(CUST_PHY_SParams_t* pParams_p)
{
    uint32_t error = (uint32_t) CUST_PHY_eERR_GENERALERROR;
	
    int32_t  status          = SystemP_FAILURE;
    uint32_t mdioBaseAddress = 0;

    if (NULL == pParams_p->pPruIcssCfg)
    {
        goto laError;
    }

    if (NULL == pParams_p->pEthPhy0Cfg)
    {
        goto laError;
    }

    pParams_p->pEthPhy0Cfg->fxns->openFxn    = NULL;
    pParams_p->pEthPhy0Cfg->fxns->closeFxn   = NULL;
    pParams_p->pEthPhy0Cfg->fxns->commandFxn = NULL;

    if (NULL == pParams_p->pEthPhy1Cfg)
    {
        goto laError;
    }

    pParams_p->pEthPhy1Cfg->fxns->openFxn    = NULL;
    pParams_p->pEthPhy1Cfg->fxns->closeFxn   = NULL;
    pParams_p->pEthPhy1Cfg->fxns->commandFxn = NULL;

    if (true == CUST_DRIVERS_getMdioManualMode())
    {
        // Pinmux for the link interrupt pins, we will set it to MLINK later.
        Pinmux_PerCfg_t tempPinMuxCfg[] = {
            /* PR1_MII0_RXLINK -> PRG1_PRU0_GPO8 (W13) */
            { PIN_PRG1_PRU0_GPO8, (PIN_MODE(1) | PIN_INPUT_ENABLE | PIN_PULL_DISABLE) },
            /* PR1_MII1_RXLINK -> PRG1_PRU1_GPO8 (U12) */
            { PIN_PRG1_PRU1_GPO8, (PIN_MODE(1) | PIN_INPUT_ENABLE | PIN_PULL_DISABLE) },
            {PINMUX_END, PINMUX_END}
        };

        Pinmux_config(tempPinMuxCfg, PINMUX_DOMAIN_ID_MAIN);
    }
    else
    {
        mdioBaseAddress = (uint32_t) pParams_p->pPruIcssCfg->hwAttrs->miiMdioRegBase;

        status = MDIO_initClock(mdioBaseAddress);

        if (SystemP_SUCCESS != status)
        {
            goto laError;
        }
    }

    CUST_PHY_CBregisterLibDetect(CUST_PHY_detect, NULL);

    error = (uint32_t) CUST_PHY_eERR_NOERROR;
	
laError:
    return error;
}

/*!
* <!-- Description: -->
*
* \brief
* Custom PHY deinitialization.
*
* \return     uint32_t                    Error code.
*
* \retval     CUST_PHY_eERR_NOERROR       Success.
* \retval     CUST_PHY_eERR_GENERALERROR  Failed.
*
*/
uint32_t CUST_PHY_deInit(void)
{
    return (uint32_t) CUST_PHY_eERR_NOERROR;
}
