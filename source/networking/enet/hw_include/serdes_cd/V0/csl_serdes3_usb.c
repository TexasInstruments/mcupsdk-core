/**
 * @file csl_serdes3_usb.h
 *
 * @brief
 *  Header file for functional layer of CSL SERDES.
 *
 *  It contains the various enumerations, structure definitions and function
 *  declarations
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2017, Texas Instruments, Inc.
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
#include <serdes_cd/V0/csl_wiz16m_cs.h>
#include <serdes_cd/V0/cslr_wiz16b4m4cs.h>


CSL_SerdesResult CSL_serdesUSBInit
(
        CSL_SerdesLaneEnableParams  *serdesLaneEnableParams
)
{

    uint8_t laneNum;
    CSL_SerdesResult result = CSL_SERDES_NO_ERR;

	/* Enable Fast SIM mode */
	#ifdef CSL_SERDES_FAST_SIM_MODE
        CSL_serdesFastSimEnable(serdesLaneEnableParams->baseAddr, serdesLaneEnableParams->numLanes, serdesLaneEnableParams->phyType);
	#endif

	if (!(serdesLaneEnableParams->refClock == CSL_SERDES_REF_CLOCK_100M || serdesLaneEnableParams->refClock == CSL_SERDES_REF_CLOCK_19p2M))
    {
        result = CSL_SERDES_INVALID_REF_CLOCK;
    }

    if (serdesLaneEnableParams->refClock == CSL_SERDES_REF_CLOCK_100M)
    {
        csl_wiz16m_cs_refclk100MHz_20b_USB_cmn(serdesLaneEnableParams->baseAddr);

        if(serdesLaneEnableParams->SSC_mode == CSL_SERDES_EXTERNAL_SSC)
        {
            csl_wiz16m_cs_refclk100MHz_20b_USB_cmn_pll_ext_ssc(serdesLaneEnableParams->baseAddr);

            for(laneNum=0; laneNum< CSL_SERDES_MAX_LANES_SIERRA; laneNum++)
            {
                if ((serdesLaneEnableParams->laneMask & (1<<laneNum))!=0)
                {
                    csl_wiz16m_cs_refclk100MHz_20b_USB_ln_ext_ssc(serdesLaneEnableParams->baseAddr, laneNum);
                }
            }
        }
        else if(serdesLaneEnableParams->SSC_mode == CSL_SERDES_INTERNAL_SSC)
        {
            csl_wiz16m_cs_refclk100MHz_20b_USB_cmn_pll_int_ssc(serdesLaneEnableParams->baseAddr);

            for(laneNum=0; laneNum< CSL_SERDES_MAX_LANES_SIERRA; laneNum++)
            {
                if ((serdesLaneEnableParams->laneMask & (1<<laneNum))!=0)
                {
                    csl_wiz16m_cs_refclk100MHz_20b_USB_ln_int_ssc(serdesLaneEnableParams->baseAddr, laneNum);
                }
            }
        }
        else if(serdesLaneEnableParams->SSC_mode == CSL_SERDES_NO_SSC)
        {
            for(laneNum=0; laneNum< CSL_SERDES_MAX_LANES_SIERRA; laneNum++)
            {
                if ((serdesLaneEnableParams->laneMask & (1<<laneNum))!=0)
                {
                    csl_wiz16m_cs_refclk100MHz_20b_USB_ln_no_ssc(serdesLaneEnableParams->baseAddr, laneNum);
                }
            }
        }
    }

    if (serdesLaneEnableParams->refClock == CSL_SERDES_REF_CLOCK_19p2M)
    {
        csl_wiz16m_cs_refclk19p2MHz_20b_USB_cmn(serdesLaneEnableParams->baseAddr);

        if(serdesLaneEnableParams->SSC_mode == CSL_SERDES_EXTERNAL_SSC)
        {
            csl_wiz16m_cs_refclk19p2MHz_20b_USB_cmn_pll_ext_ssc(serdesLaneEnableParams->baseAddr);

            for(laneNum=0; laneNum< CSL_SERDES_MAX_LANES_SIERRA; laneNum++)
            {
                if ((serdesLaneEnableParams->laneMask & (1<<laneNum))!=0)
                {
                    csl_wiz16m_cs_refclk19p2MHz_20b_USB_ln_ext_ssc(serdesLaneEnableParams->baseAddr, laneNum);
                }
            }
        }
        else if(serdesLaneEnableParams->SSC_mode == CSL_SERDES_INTERNAL_SSC)
        {
            csl_wiz16m_cs_refclk19p2MHz_20b_USB_cmn_pll_int_ssc(serdesLaneEnableParams->baseAddr);

            for(laneNum=0; laneNum< CSL_SERDES_MAX_LANES_SIERRA; laneNum++)
            {
                if ((serdesLaneEnableParams->laneMask & (1<<laneNum))!=0)
                {
                    csl_wiz16m_cs_refclk19p2MHz_20b_USB_ln_int_ssc(serdesLaneEnableParams->baseAddr, laneNum);
                }
            }
        }
        else if(serdesLaneEnableParams->SSC_mode == CSL_SERDES_NO_SSC)
        {
            csl_wiz16m_cs_refclk19p2MHz_20b_USB_cmn_pll_no_ssc(serdesLaneEnableParams->baseAddr);

            for(laneNum=0; laneNum< CSL_SERDES_MAX_LANES_SIERRA; laneNum++)
            {
                if ((serdesLaneEnableParams->laneMask & (1<<laneNum))!=0)
                {
                    csl_wiz16m_cs_refclk19p2MHz_20b_USB_ln_no_ssc(serdesLaneEnableParams->baseAddr, laneNum);
                }
            }
        }
    }

    return result;
}
