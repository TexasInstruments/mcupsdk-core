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

#include <CUST_PHY_base.h>
#include <drivers/ethphy/CUST_phy.h>

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
    int32_t  status          = SystemP_FAILURE;
    uint32_t mdioBaseAddress = 0;

    if (NULL == pParams_p->pPruIcssCfg)
    {
        return CUST_PHY_eERR_GENERALERROR;
    }

    if (NULL == pParams_p->pEthPhy0Cfg)
    {
        return CUST_PHY_eERR_GENERALERROR;
    }

    pParams_p->pEthPhy0Cfg->fxns->openFxn    = NULL;
    pParams_p->pEthPhy0Cfg->fxns->closeFxn   = NULL;
    pParams_p->pEthPhy0Cfg->fxns->commandFxn = NULL;

    if (NULL == pParams_p->pEthPhy1Cfg)
    {
        return CUST_PHY_eERR_GENERALERROR;
    }

    pParams_p->pEthPhy1Cfg->fxns->openFxn    = NULL;
    pParams_p->pEthPhy1Cfg->fxns->closeFxn   = NULL;
    pParams_p->pEthPhy1Cfg->fxns->commandFxn = NULL;

    mdioBaseAddress = (uint32_t) pParams_p->pPruIcssCfg->hwAttrs->miiMdioRegBase;

    status = MDIO_initClock(mdioBaseAddress);

    if (SystemP_SUCCESS != status)
    {
        return CUST_PHY_eERR_GENERALERROR;
    }

    CUST_PHY_CBregisterLibDetect(CUST_PHY_detect, NULL);

    return CUST_PHY_eERR_NOERROR;
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
    return CUST_PHY_eERR_NOERROR;
}
