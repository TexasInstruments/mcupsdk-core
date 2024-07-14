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
 * @defgroup CSL_SERDES_USB SERDES USB
 * @ingroup CSL_SERDES_API
 *
 * @section Introduction
 *
 * @subsection xxx Overview
 * This module deals with setting up SERDES configuration for USB. The API flow should be as follows:   \n
 *
 * CSL_serdesUSBInit (baseAddr, numLanes, refClock, rate);
 *
 * CSL_serdesLaneEnable (&serdesLaneEnableParams);
 *
 * @subsection References
 *
 * ============================================================================
 */

#include <ti/csl/csl_serdes.h>
#include <ti/csl/csl_serdes_usb.h>
#include <ti/csl/src/ip/serdes_sb/V1/csl_wiz8m_sb.h>


CSL_SerdesResult CSL_serdesUSBInit
(
        CSL_SerdesLaneEnableParams  *serdesLaneEnableParams
)
{

    CSL_SerdesResult result = CSL_SERDES_NO_ERR;

    CSL_serdesDisablePllAndLanes(serdesLaneEnableParams->baseAddr, serdesLaneEnableParams->numLanes);

	/* CMU_WAIT has to be set to a lower value for fast sim before loading the config since the CMU ticker kicks off in the config */
    /* Enable Fast SIM mode */
    if ((serdesLaneEnableParams->operatingMode == CSL_SERDES_FUNCTIONAL_MODE_FAST_SIM) || (serdesLaneEnableParams->operatingMode == CSL_SERDES_DIAGNOSTIC_MODE_FAST_SIM))
    {
        CSL_serdesRxEqDisable(serdesLaneEnableParams->baseAddr);
        CSL_serdesSetCMUWaitVal(serdesLaneEnableParams->baseAddr, 20);
    }

    if (!(serdesLaneEnableParams->refClock == CSL_SERDES_REF_CLOCK_20M ||
          serdesLaneEnableParams->refClock == CSL_SERDES_REF_CLOCK_100M))
    {
        result = CSL_SERDES_INVALID_REF_CLOCK;
    }

    if (serdesLaneEnableParams->numLanes == 1)
    {
        if (serdesLaneEnableParams->refClock == CSL_SERDES_REF_CLOCK_20M &&
            serdesLaneEnableParams->linkRate == CSL_SERDES_LINK_RATE_5G)
        {
            csl_wiz8m_sb_refclk20p0MHz_16b_5p0Gbps_USB_1l1c_sr(serdesLaneEnableParams->baseAddr);
        }
	if (serdesLaneEnableParams->refClock == CSL_SERDES_REF_CLOCK_100M &&
	    serdesLaneEnableParams->linkRate == CSL_SERDES_LINK_RATE_5G)
        {
            if(serdesLaneEnableParams->sscMode == CSL_SERDES_SSC_ENABLED)
            {
                csl_wiz8m_sb_refclk100p0MHz_16b_5p0Gbps_USBSSC_1l1c_sr(serdesLaneEnableParams->baseAddr);
            }
            else
            {
                csl_wiz8m_sb_refclk100p0MHz_16b_5p0Gbps_USB_1l1c_sr(serdesLaneEnableParams->baseAddr);
            }
        }
    }

    if (serdesLaneEnableParams->linkRate != CSL_SERDES_LINK_RATE_5G)
    {
        result = CSL_SERDES_INVALID_LANE_RATE;
    }

    /* For Fast Sim mode, enable fast_sim_o after config load */
    if ((serdesLaneEnableParams->operatingMode == CSL_SERDES_FUNCTIONAL_MODE_FAST_SIM) || (serdesLaneEnableParams->operatingMode == CSL_SERDES_DIAGNOSTIC_MODE_FAST_SIM))
    {
        CSL_serdesFastSimEnable(serdesLaneEnableParams->baseAddr);
    }

    return result;
}

