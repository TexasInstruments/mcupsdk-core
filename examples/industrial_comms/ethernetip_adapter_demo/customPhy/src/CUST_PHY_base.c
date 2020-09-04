/*!
 *  \file CUST_PHY_base.c
 *
 *  \brief
 *  Brief description of purpose and functionality.
 *
 *  \author
 *  KUNBUS GmbH
 *
 *  \date
 *  2022-02-24
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

#include <CUST_PHY_base.h>
#include <CUST_PHY_dp83869.h>

static CUST_PHY_CBextPhyLibDetect_t CUST_PHY_knownImplementation_s [] =
{
    CUST_PHY_DP83869_detect,
    NULL
};

/*! <!-- Description: -->
 *
 *  \brief
 *  Detect Phy Type and setup access structures accordingly
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pPhyLibCtxt_p	Context of External PhyLib.
 *  \param[in]  phyId_p         Phy ID read from hardware
 *  \param[in]  pPhyLibDesc_p   External PhyLib Hooks
 *  \return     0 on success and Phy detected, error code otherwise
 *
 *  <!-- Group: -->
 *
 *  \ingroup APPPHYLIB
 *
 * */
int16_t CUST_PHY_detect(void* pPhyLibCtxt_p, uint32_t phyId_p, CUST_PHY_SPhyDescriptor_t *pPhyLibDesc_p)
{
    int16_t                         retVal      = -CUST_PHY_STATUS_ERROR_UNKNOWN_PHY;

    CUST_PHY_CBextPhyLibDetect_t*   iterator    = CUST_PHY_knownImplementation_s;

    while ((NULL != iterator) && (NULL != iterator[0]))
    {
        retVal = iterator[0](pPhyLibCtxt_p, phyId_p, pPhyLibDesc_p);

        if (0 == retVal)
        {
            break;
        }
        (++iterator);
    }

    return retVal;
}

//*************************************************************************************************
