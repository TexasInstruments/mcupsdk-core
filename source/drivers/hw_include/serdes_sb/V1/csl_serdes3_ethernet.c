/*
 * Copyright (C) 2024 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of
 * its contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
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
 *
 */

/** ============================================================================
 *
 * @defgroup CSL_SERDES_ETHERNET SERDES ETHERNET
 * @ingroup CSL_SERDES_API
 *
 * @section Introduction
 *
 * @subsection xxx Overview
 * This module deals with setting up SERDES configuration for ETHERNET. The API flow should be as follows:   \n
 *
 * CSL_serdesEthernetInit (baseAddr, numLanes, refClock, rate);
 *
 * CSL_serdesLaneEnable (&serdesLaneEnableParams);
 *
 * @subsection References
 *
 * ============================================================================
 */
#include <ti/csl/csl_serdes.h>
#include <ti/csl/csl_serdes_ethernet.h>
#include <ti/csl/src/ip/serdes_sb/V1/csl_wiz8m_sb.h>

CSL_SerdesResult CSL_serdesEthernetInit
(
 uint32_t            baseAddr,
 uint32_t            numLanes,
 CSL_SerdesRefClock  refClock,
 CSL_SerdesLinkRate  rate
)
{

    CSL_SerdesResult result = CSL_SERDES_NO_ERR;

    CSL_serdesDisablePllAndLanes(baseAddr, numLanes);
    CSL_serdesPorReset(baseAddr);

    /* CMU_WAIT has to be set to a lower value for fast sim before loading the config since the CMU ticker kicks off in the config */
    #ifdef CSL_SERDES_FAST_SIM_MODE
	CSL_serdesSetCMUWaitVal(baseAddr, 20);
    #endif

    if (!(refClock == CSL_SERDES_REF_CLOCK_125M || refClock == CSL_SERDES_REF_CLOCK_156p25M))
    {
        result = CSL_SERDES_INVALID_REF_CLOCK;
    }

    if (numLanes == 1)
    {
        if (refClock == CSL_SERDES_REF_CLOCK_125M && rate == CSL_SERDES_LINK_RATE_5G)
	{
            csl_wiz8m_sb_refclk125p0MHz_10b_1p25Gbps_SGMII_1l1c_sr4(baseAddr);
        }
        if (refClock == CSL_SERDES_REF_CLOCK_125M && rate == CSL_SERDES_LINK_RATE_6p25G)
        {
            result = CSL_SERDES_INVALID_LANE_RATE;
        }
        if (refClock == CSL_SERDES_REF_CLOCK_156p25M && rate == CSL_SERDES_LINK_RATE_5G)
        {
            csl_wiz8m_sb_refclk156p25MHz_10b_1p25Gbps_SGMII_1l1c_sr4(baseAddr);
        }
        if (refClock == CSL_SERDES_REF_CLOCK_156p25M && rate == CSL_SERDES_LINK_RATE_6p25G)
        {
            csl_wiz8m_sb_refclk156p25MHz_10b_3p125Gbps_SGMII_1l1c_sr2(baseAddr);
        }
    }
    else
    {
        result = CSL_SERDES_INVALID_NUM_LANES;
    }

    if (!(rate == CSL_SERDES_LINK_RATE_5G || rate == CSL_SERDES_LINK_RATE_6p25G))
    {
        result = CSL_SERDES_INVALID_LANE_RATE;
    }

    /* For Fast Sim mode, enable fast_sim_o after config load */
#ifdef CSL_SERDES_FAST_SIM_MODE
    CSL_serdesFastSimEnable(baseAddr);
#endif

    return result;
}
/* @} */
/* nothing past this point */
