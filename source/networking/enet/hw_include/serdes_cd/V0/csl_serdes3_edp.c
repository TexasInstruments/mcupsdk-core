/**
 * @file csl_serdes3_edp.h
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
 * @defgroup CSL_SERDES_EDP SERDES EDP
 * @ingroup CSL_SERDES_API
 *
 * @section Introduction
 *
 * @subsection xxx Overview
 * This module deals with setting up SERDES configuration for eDP. The API flow should be as follows:   \n
 *
 * CSL_serdeseDPInit (baseAddr, numLanes, refClock, rate);
 *
 * CSL_serdesLaneEnable (&serdesLaneEnableParams);
 *
 * @subsection References
 *
 * ============================================================================
 */

#include <ti/csl/csl_serdes.h>
#include <ti/csl/csl_serdes_edp.h>
#include <serdes_cd/V0/csl_wiz16m_ct.h>
#include <serdes_cd/V0/cslr_wiz16b8m4ct.h>

CSL_SerdesResult CSL_serdeseDPInit
(
    CSL_SerdesLaneEnableParams  *serdesLaneEnableParams
)
{

    uint8_t laneNum;
    CSL_SerdesResult result = CSL_SERDES_NO_ERR;

	/* Enable Fast SIM mode */
	#ifdef CSL_SERDES_FAST_SIM_MODE
        CSL_serdesFastSimEnable(serdesLaneEnableParams->baseAddr, serdesLaneEnableParams->numLanes, CSL_SERDES_PHY_TYPE_eDP);
	#endif

        if (!(serdesLaneEnableParams->refClock == CSL_SERDES_REF_CLOCK_100M || serdesLaneEnableParams->refClock == CSL_SERDES_REF_CLOCK_19p2M))
        {
            result = CSL_SERDES_INVALID_REF_CLOCK;
        }

    CSL_wiz16b8m4ctRegs *torrent_sds_reg = (CSL_wiz16b8m4ctRegs *)(uintptr_t)(serdesLaneEnableParams->baseAddr);

    /* PHY_PLL_CFG for using only PLL0 */
    CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->PHY_PCS_CMN_REGISTERS.PHY_AUTO_CFG_SPDUP), 15, 0, (uint32_t)0x0);

	if(serdesLaneEnableParams->serdesInstance == CSL_TORRENT_SERDES0)
    {
        if (serdesLaneEnableParams->refClock == CSL_SERDES_REF_CLOCK_100M && serdesLaneEnableParams->linkRate == CSL_SERDES_LINK_RATE_8p1G)
        {
            for(laneNum=0; laneNum< CSL_SERDES_MAX_LANES_TORRENT; laneNum++)
            {
                if ((serdesLaneEnableParams->laneMask & (1<<laneNum))!=0)
                {
                    /* For only link0 being configured */
                    CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[laneNum].XCVR_DIAG_HSCLK_DIV__XCVR_DIAG_HSCLK_SEL), 15, 0, (uint32_t)0x0000);
                    CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[laneNum].XCVR_DIAG_PLLDRC_CTRL__XCVR_DIAG_XDP_PWRI_STAT), 31, 16, (uint32_t)0x0001);
                    csl_wiz16m_ct_20b_eDP_cmn_8p1G(serdesLaneEnableParams->baseAddr, laneNum);
                }
            }
            csl_wiz16m_ct_refclk100MHz_20b_eDP_cmn_pll_all_vco(serdesLaneEnableParams->baseAddr);
            csl_wiz16m_ct_refclk100MHz_20b_eDP_cmn_pll_8p1G_vco(serdesLaneEnableParams->baseAddr);
            csl_wiz16m_ct_refclk100MHz_20b_eDP_cmn_pll1_all_vco(serdesLaneEnableParams->baseAddr);
            csl_wiz16m_ct_refclk100MHz_20b_eDP_cmn_pll1_8p1G_vco(serdesLaneEnableParams->baseAddr);
        }
        else if (serdesLaneEnableParams->refClock == CSL_SERDES_REF_CLOCK_19p2M && serdesLaneEnableParams->linkRate == CSL_SERDES_LINK_RATE_8p1G)
        {
            for(laneNum=0; laneNum< CSL_SERDES_MAX_LANES_TORRENT; laneNum++)
            {
                if ((serdesLaneEnableParams->laneMask & (1<<laneNum))!=0)
                {
                    /* For only link0 being configured */
                    CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[laneNum].XCVR_DIAG_HSCLK_DIV__XCVR_DIAG_HSCLK_SEL), 15, 0, (uint32_t)0x0000);
                    CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[laneNum].XCVR_DIAG_PLLDRC_CTRL__XCVR_DIAG_XDP_PWRI_STAT), 31, 16, (uint32_t)0x0001);
                    csl_wiz16m_ct_20b_eDP_cmn_8p1G(serdesLaneEnableParams->baseAddr, laneNum);
                    csl_wiz16m_ct_refclk19p2MHz_refclk_related(serdesLaneEnableParams->baseAddr, laneNum);
                }
            }
            csl_wiz16m_ct_refclk19p2MHz_20b_eDP_cmn_pll_all_vco(serdesLaneEnableParams->baseAddr);
            csl_wiz16m_ct_refclk19p2MHz_20b_eDP_cmn_pll_8p1G_vco(serdesLaneEnableParams->baseAddr);
            csl_wiz16m_ct_refclk19p2MHz_20b_eDP_cmn_pll1_all_vco(serdesLaneEnableParams->baseAddr);
            csl_wiz16m_ct_refclk19p2MHz_20b_eDP_cmn_pll1_8p1G_vco(serdesLaneEnableParams->baseAddr);
        }
        else if (serdesLaneEnableParams->refClock == CSL_SERDES_REF_CLOCK_19p2M && serdesLaneEnableParams->linkRate == CSL_SERDES_LINK_RATE_5p4G)
        {
            for(laneNum=0; laneNum< CSL_SERDES_MAX_LANES_TORRENT; laneNum++)
            {
                if ((serdesLaneEnableParams->laneMask & (1<<laneNum))!=0)
                {
                    /* For only link0 being configured */
                    CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[laneNum].XCVR_DIAG_HSCLK_DIV__XCVR_DIAG_HSCLK_SEL), 15, 0, (uint32_t)0x0000);
                    CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[laneNum].XCVR_DIAG_PLLDRC_CTRL__XCVR_DIAG_XDP_PWRI_STAT), 31, 16, (uint32_t)0x0001);
                    csl_wiz16m_ct_20b_eDP_cmn_5p4G(serdesLaneEnableParams->baseAddr, laneNum);
                    csl_wiz16m_ct_refclk19p2MHz_refclk_related(serdesLaneEnableParams->baseAddr, laneNum);
                }
            }
            csl_wiz16m_ct_refclk19p2MHz_20b_eDP_cmn_pll_all_vco(serdesLaneEnableParams->baseAddr);
            csl_wiz16m_ct_refclk19p2MHz_20b_eDP_cmn_pll_10p8G_vco(serdesLaneEnableParams->baseAddr);
            csl_wiz16m_ct_refclk19p2MHz_20b_eDP_cmn_pll1_all_vco(serdesLaneEnableParams->baseAddr);
            csl_wiz16m_ct_refclk19p2MHz_20b_eDP_cmn_pll1_10p8G_vco(serdesLaneEnableParams->baseAddr);
        }
        else if (serdesLaneEnableParams->refClock == CSL_SERDES_REF_CLOCK_19p2M && serdesLaneEnableParams->linkRate == CSL_SERDES_LINK_RATE_2p7G)
                {
                    for(laneNum=0; laneNum< CSL_SERDES_MAX_LANES_TORRENT; laneNum++)
                    {
                        if ((serdesLaneEnableParams->laneMask & (1<<laneNum))!=0)
                        {
                            /* For only link0 being configured */
                            CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[laneNum].XCVR_DIAG_HSCLK_DIV__XCVR_DIAG_HSCLK_SEL), 15, 0, (uint32_t)0x0000);
                            CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[laneNum].XCVR_DIAG_PLLDRC_CTRL__XCVR_DIAG_XDP_PWRI_STAT), 31, 16, (uint32_t)0x0001);
                            csl_wiz16m_ct_20b_eDP_cmn_2p7G(serdesLaneEnableParams->baseAddr, laneNum);
                            csl_wiz16m_ct_refclk19p2MHz_refclk_related(serdesLaneEnableParams->baseAddr, laneNum);
                        }
                    }
                    csl_wiz16m_ct_refclk19p2MHz_20b_eDP_cmn_pll_all_vco(serdesLaneEnableParams->baseAddr);
                    csl_wiz16m_ct_refclk19p2MHz_20b_eDP_cmn_pll_10p8G_vco(serdesLaneEnableParams->baseAddr);
                    csl_wiz16m_ct_refclk19p2MHz_20b_eDP_cmn_pll1_all_vco(serdesLaneEnableParams->baseAddr);
                    csl_wiz16m_ct_refclk19p2MHz_20b_eDP_cmn_pll1_10p8G_vco(serdesLaneEnableParams->baseAddr);
                }
    }
    else
    {
        result = CSL_SERDES_INVALID_PHY_TYPE;
    }
    if (!(serdesLaneEnableParams->linkRate == CSL_SERDES_LINK_RATE_8p1G || serdesLaneEnableParams->linkRate == CSL_SERDES_LINK_RATE_5p4G || serdesLaneEnableParams->linkRate == CSL_SERDES_LINK_RATE_2p7G))
    {
        result = CSL_SERDES_INVALID_LANE_RATE;
    }

    return result;
}
