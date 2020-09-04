/**
 * @file csl_serdes3_ethernet.h
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
#include <serdes_cd/V0/cslr_wiz16b4m4cs.h>
#include <serdes_cd/V0/cslr_wiz16b8m4ct.h>
#include <serdes_cd/V0/csl_wiz16m_cs.h>
#include <serdes_cd/V0/csl_wiz16m_ct.h>

CSL_SerdesResult CSL_serdesEthernetInit
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

    if (!(serdesLaneEnableParams->refClock == CSL_SERDES_REF_CLOCK_19p2M || serdesLaneEnableParams->refClock == CSL_SERDES_REF_CLOCK_100M || serdesLaneEnableParams->refClock == CSL_SERDES_REF_CLOCK_156p25M))
    {
        result |= CSL_SERDES_INVALID_REF_CLOCK;
    }

    if (!(serdesLaneEnableParams->linkRate == CSL_SERDES_LINK_RATE_1p25G ||
            serdesLaneEnableParams->linkRate == CSL_SERDES_LINK_RATE_3p125G ||
            serdesLaneEnableParams->linkRate == CSL_SERDES_LINK_RATE_5G ||
            serdesLaneEnableParams->linkRate == CSL_SERDES_LINK_RATE_10p3125G))
    {
        result |= CSL_SERDES_INVALID_LANE_RATE;
    }

    /* Sierra configs */
    if(!(serdesLaneEnableParams->serdesInstance == CSL_TORRENT_SERDES0))
    {
		/* SGMII 1.25G Baud configs */
        if (serdesLaneEnableParams->phyType == CSL_SERDES_PHY_TYPE_SGMII)
		{
            if(serdesLaneEnableParams->linkRate != CSL_SERDES_LINK_RATE_1p25G)
            {
                result |= CSL_SERDES_INVALID_LANE_RATE;
            }
            if(serdesLaneEnableParams->refClock == CSL_SERDES_REF_CLOCK_19p2M)
            {
                csl_wiz16m_cs_refclk19p2MHz_20b_SGMII_cmn(serdesLaneEnableParams->baseAddr);
                csl_wiz16m_cs_refclk19p2MHz_20b_SGMII_cmn_pll(serdesLaneEnableParams->baseAddr);

                for(laneNum=0; laneNum< CSL_SERDES_MAX_LANES_SIERRA; laneNum++)
                {
                    if ((serdesLaneEnableParams->laneMask & (1<<laneNum))!=0)
                    {
                        csl_wiz16m_cs_refclk19p2MHz_20b_SGMII_ln(serdesLaneEnableParams->baseAddr, laneNum);
                    }
                }
            }
            else if(serdesLaneEnableParams->refClock == CSL_SERDES_REF_CLOCK_100M)
		    {
                csl_wiz16m_cs_refclk100MHz_20b_SGMII_cmn_pll(serdesLaneEnableParams->baseAddr);

                for(laneNum=0; laneNum< CSL_SERDES_MAX_LANES_SIERRA; laneNum++)
                {
                    if ((serdesLaneEnableParams->laneMask & (1<<laneNum))!=0)
                    {
                        csl_wiz16m_cs_refclk100MHz_20b_SGMII_ln(serdesLaneEnableParams->baseAddr, laneNum);
                    }
                }
		    }
		    else if (serdesLaneEnableParams->refClock == CSL_SERDES_REF_CLOCK_156p25M)
		    {
	            csl_wiz16m_cs_refclk156p25MHz_20b_SGMII_cmn(serdesLaneEnableParams->baseAddr);
	            csl_wiz16m_cs_refclk156p25MHz_20b_SGMII_cmn_pll(serdesLaneEnableParams->baseAddr);
	            for(laneNum=0; laneNum< CSL_SERDES_MAX_LANES_SIERRA; laneNum++)
	            {
	                if ((serdesLaneEnableParams->laneMask & (1<<laneNum))!=0)
	                {
	                    csl_wiz16m_cs_refclk156p25MHz_20b_SGMII_ln(serdesLaneEnableParams->baseAddr, laneNum);
	                }
	            }
		    }
		    else
		    {
		        result |= CSL_SERDES_INVALID_REF_CLOCK;
		    }
		}

		/* XAUI 3.125G Baud configs */
		if (serdesLaneEnableParams->phyType == CSL_SERDES_PHY_TYPE_XAUI)
        {
            if(serdesLaneEnableParams->linkRate != CSL_SERDES_LINK_RATE_3p125G)
            {
                result |= CSL_SERDES_INVALID_LANE_RATE;
            }
		    if(serdesLaneEnableParams->refClock == CSL_SERDES_REF_CLOCK_156p25M)
            {
                csl_wiz16m_cs_refclk156p25MHz_20b_XAUI_cmn(serdesLaneEnableParams->baseAddr);
                csl_wiz16m_cs_refclk156p25MHz_20b_XAUI_cmn_pll(serdesLaneEnableParams->baseAddr);

                for(laneNum=0; laneNum< CSL_SERDES_MAX_LANES_SIERRA; laneNum++)
                {
                    if ((serdesLaneEnableParams->laneMask & (1<<laneNum))!=0)
                    {
                        csl_wiz16m_cs_refclk156p25MHz_20b_XAUI_ln(serdesLaneEnableParams->baseAddr, laneNum);
                    }
                }
            }
            else
            {
                result |= CSL_SERDES_INVALID_REF_CLOCK;
            }
        }

		/* QSGMII 5G Baud configs */
		if (serdesLaneEnableParams->phyType == CSL_SERDES_PHY_TYPE_QSGMII)
        {
            if(serdesLaneEnableParams->linkRate != CSL_SERDES_LINK_RATE_5G)
            {
                result |= CSL_SERDES_INVALID_LANE_RATE;
            }
		    if(serdesLaneEnableParams->refClock == CSL_SERDES_REF_CLOCK_100M)
		    {
                csl_wiz16m_cs_refclk100MHz_20b_QSGMII_cmn_pll(serdesLaneEnableParams->baseAddr);

                for(laneNum=0; laneNum< CSL_SERDES_MAX_LANES_SIERRA; laneNum++)
                {
                    if ((serdesLaneEnableParams->laneMask & (1<<laneNum))!=0)
                    {
                        csl_wiz16m_cs_refclk100MHz_20b_QSGMII_ln(serdesLaneEnableParams->baseAddr, laneNum);
                    }
                }
		    }
		    else if (serdesLaneEnableParams->refClock == CSL_SERDES_REF_CLOCK_19p2M)
		    {
	            csl_wiz16m_cs_refclk19p2MHz_20b_QSGMII_cmn(serdesLaneEnableParams->baseAddr);
	            csl_wiz16m_cs_refclk19p2MHz_20b_QSGMII_cmn_pll(serdesLaneEnableParams->baseAddr);

	            for(laneNum=0; laneNum< CSL_SERDES_MAX_LANES_SIERRA; laneNum++)
	            {
	                if ((serdesLaneEnableParams->laneMask & (1<<laneNum))!=0)
	                {
	                    csl_wiz16m_cs_refclk19p2MHz_20b_QSGMII_ln(serdesLaneEnableParams->baseAddr, laneNum);
	                }
	            }
		    }
            else
            {
                result |= CSL_SERDES_INVALID_REF_CLOCK;
            }
        }

		/* USXGMII/XFI 10.3125 Baud configs */
        if (serdesLaneEnableParams->phyType == CSL_SERDES_PHY_TYPE_XFI)
        {
            if(serdesLaneEnableParams->linkRate != CSL_SERDES_LINK_RATE_10p3125G)
            {
                result |= CSL_SERDES_INVALID_LANE_RATE;
            }
            if(serdesLaneEnableParams->refClock == CSL_SERDES_REF_CLOCK_156p25M)
            {
                csl_wiz16m_cs_refclk156p25MHz_32b_XFI_cmn(serdesLaneEnableParams->baseAddr);
                csl_wiz16m_cs_refclk156p25MHz_32b_XFI_cmn_pll(serdesLaneEnableParams->baseAddr);

                for(laneNum=0; laneNum< CSL_SERDES_MAX_LANES_SIERRA; laneNum++)
                {
                    if ((serdesLaneEnableParams->laneMask & (1<<laneNum))!=0)
                    {
                        csl_wiz16m_cs_refclk156p25MHz_32b_XFI_ln(serdesLaneEnableParams->baseAddr, laneNum);
                    }
                }
            }
            else
            {
                result |= CSL_SERDES_INVALID_REF_CLOCK;
            }
        }
    }
    else if(serdesLaneEnableParams->serdesInstance == CSL_TORRENT_SERDES0)
    {
        CSL_wiz16b8m4ctRegs *torrent_sds_reg = (CSL_wiz16b8m4ctRegs *)(uintptr_t)(serdesLaneEnableParams->baseAddr);

        /* PHY_PLL_CFG for using only PLL0 */
        CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->PHY_PCS_CMN_REGISTERS.PHY_AUTO_CFG_SPDUP), 15, 0, (uint32_t)0x0);

        /* SGMII 1.25G Baud configs */
        if (serdesLaneEnableParams->phyType == CSL_SERDES_PHY_TYPE_SGMII)
        {
            if (serdesLaneEnableParams->linkRate != CSL_SERDES_LINK_RATE_1p25G)
            {
                result |= CSL_SERDES_INVALID_LANE_RATE;
            }

            CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CLK_SEL_M0__CMN_PDIAG_PLL0_CTRL_M0),31,16,(uint32_t)0x0601);

            for(laneNum=0; laneNum< CSL_SERDES_MAX_LANES_TORRENT; laneNum++)
            {
                if ((serdesLaneEnableParams->laneMask & (1<<laneNum))!=0)
                {
                    CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[laneNum].XCVR_DIAG_HSCLK_DIV__XCVR_DIAG_HSCLK_SEL), 15, 0, (uint32_t)0x0000);
                    CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[laneNum].XCVR_DIAG_HSCLK_DIV__XCVR_DIAG_HSCLK_SEL), 31, 16, (uint32_t)0x0003);
                    CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[laneNum].XCVR_DIAG_PLLDRC_CTRL__XCVR_DIAG_XDP_PWRI_STAT), 31, 16, (uint32_t)0x0013);
                    csl_wiz16m_ct_20b_SGMII_cmn(serdesLaneEnableParams->baseAddr, laneNum);
                    if(serdesLaneEnableParams->refClock == CSL_SERDES_REF_CLOCK_19p2M)
                    {
                        csl_wiz16m_ct_refclk19p2MHz_refclk_related(serdesLaneEnableParams->baseAddr, laneNum);
                    }
                }
            }
            if(serdesLaneEnableParams->refClock == CSL_SERDES_REF_CLOCK_100M)
            {
                csl_wiz16m_ct_refclk100MHz_20b_SGMII_QSGMII_cmn_pll_all_vco(serdesLaneEnableParams->baseAddr);
                csl_wiz16m_ct_refclk100MHz_20b_SGMII_QSGMII_cmn_pll1_all_vco(serdesLaneEnableParams->baseAddr);
            }
            if(serdesLaneEnableParams->refClock == CSL_SERDES_REF_CLOCK_19p2M)
            {
                csl_wiz16m_ct_refclk19p2MHz_cmn_pll_all_vco(serdesLaneEnableParams->baseAddr);
                csl_wiz16m_ct_refclk19p2MHz_cmn_pll1_all_vco(serdesLaneEnableParams->baseAddr);
                csl_wiz16m_ct_refclk19p2MHz_cmn_pll_10G_vco(serdesLaneEnableParams->baseAddr);
                csl_wiz16m_ct_refclk19p2MHz_cmn_pll1_10G_vco(serdesLaneEnableParams->baseAddr);
            }
        }

        /* XAUI 3.125G Baud configs */
        if (serdesLaneEnableParams->phyType == CSL_SERDES_PHY_TYPE_XAUI)
        {
            if (serdesLaneEnableParams->linkRate != CSL_SERDES_LINK_RATE_3p125G)
            {
                result |= CSL_SERDES_INVALID_LANE_RATE;
            }

            if(serdesLaneEnableParams->refClock == CSL_SERDES_REF_CLOCK_156p25M)
            {
                /* PHY_PLL_CFG for using only PLL0 */
                CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->PHY_PCS_CMN_REGISTERS.PHY_AUTO_CFG_SPDUP), 15, 0, (uint32_t)0x0);
                CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CLK_SEL_M0__CMN_PDIAG_PLL0_CTRL_M0),31,16,(uint32_t)0x0601);
                CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CLK_SEL_M0__CMN_PDIAG_PLL1_CTRL_M0),31,16,(uint32_t)0x0601);

                for(laneNum=0; laneNum< CSL_SERDES_MAX_LANES_TORRENT; laneNum++)
                {
                    if ((serdesLaneEnableParams->laneMask & (1<<laneNum))!=0)
                    {
                        CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[laneNum].XCVR_DIAG_HSCLK_DIV__XCVR_DIAG_HSCLK_SEL), 15, 0, (uint32_t)0x0000);
                        CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[laneNum].XCVR_DIAG_HSCLK_DIV__XCVR_DIAG_HSCLK_SEL), 31, 16, (uint32_t)0x0000);
                        CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[laneNum].XCVR_DIAG_PLLDRC_CTRL__XCVR_DIAG_XDP_PWRI_STAT), 31, 16, (uint32_t)0x0001);
                        csl_wiz16m_ct_refclk156p25MHz_refclk_related(serdesLaneEnableParams->baseAddr, laneNum);
                        csl_wiz16m_ct_20b_XAUI_cmn(serdesLaneEnableParams->baseAddr, laneNum);
                    }
                }
                csl_wiz16m_ct_refclk156p25MHz_20b_XAUI_cmn_pll_all_vco(serdesLaneEnableParams->baseAddr);
                csl_wiz16m_ct_refclk156p25MHz_20b_XAUI_cmn_pll1_all_vco(serdesLaneEnableParams->baseAddr);
                csl_wiz16m_ct_refclk156p25MHz_20b_XAUI_cmn_pll_6p25G_vco(serdesLaneEnableParams->baseAddr);
                csl_wiz16m_ct_refclk156p25MHz_20b_XAUI_cmn_pll1_6p25G_vco(serdesLaneEnableParams->baseAddr);
            }
            else if(serdesLaneEnableParams->refClock == CSL_SERDES_REF_CLOCK_19p2M)
            {
                /* PHY_PLL_CFG for using only PLL0 */
                CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->PHY_PCS_CMN_REGISTERS.PHY_AUTO_CFG_SPDUP), 15, 0, (uint32_t)0x0);
                CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CLK_SEL_M0__CMN_PDIAG_PLL0_CTRL_M0),31,16,(uint32_t)0x0601);
                CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CLK_SEL_M0__CMN_PDIAG_PLL1_CTRL_M0),31,16,(uint32_t)0x0601);

                for(laneNum=0; laneNum< CSL_SERDES_MAX_LANES_TORRENT; laneNum++)
                {
                    if ((serdesLaneEnableParams->laneMask & (1<<laneNum))!=0)
                    {
                        CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[laneNum].XCVR_DIAG_HSCLK_DIV__XCVR_DIAG_HSCLK_SEL), 15, 0, (uint32_t)0x0000);
                        CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[laneNum].XCVR_DIAG_HSCLK_DIV__XCVR_DIAG_HSCLK_SEL), 31, 16, (uint32_t)0x0000);
                        CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[laneNum].XCVR_DIAG_PLLDRC_CTRL__XCVR_DIAG_XDP_PWRI_STAT), 31, 16, (uint32_t)0x0001);
                        csl_wiz16m_ct_refclk19p2MHz_refclk_related(serdesLaneEnableParams->baseAddr, laneNum);
                        csl_wiz16m_ct_20b_XAUI_cmn(serdesLaneEnableParams->baseAddr, laneNum);
                    }
                }
                csl_wiz16m_ct_refclk19p2MHz_cmn_pll_all_vco(serdesLaneEnableParams->baseAddr);
                csl_wiz16m_ct_refclk19p2MHz_cmn_pll1_all_vco(serdesLaneEnableParams->baseAddr);
                csl_wiz16m_ct_refclk19p2MHz_cmn_pll_6p25G_vco(serdesLaneEnableParams->baseAddr);
                csl_wiz16m_ct_refclk19p2MHz_cmn_pll1_6p25G_vco(serdesLaneEnableParams->baseAddr);
            }
            else
            {
                result |= CSL_SERDES_INVALID_REF_CLOCK;
            }
        }

        /* QSGMII 5G Baud configs */
        if (serdesLaneEnableParams->phyType == CSL_SERDES_PHY_TYPE_QSGMII)
        {
            if (serdesLaneEnableParams->linkRate != CSL_SERDES_LINK_RATE_5G)
            {
                result |= CSL_SERDES_INVALID_LANE_RATE;
            }

            if(serdesLaneEnableParams->refClock == CSL_SERDES_REF_CLOCK_100M)
            {
                CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CLK_SEL_M0__CMN_PDIAG_PLL0_CTRL_M0),31,16,(uint32_t)0x0601);
                for(laneNum=0; laneNum< CSL_SERDES_MAX_LANES_TORRENT; laneNum++)
                {
                    if ((serdesLaneEnableParams->laneMask & (1<<laneNum))!=0)
                    {
                        CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[laneNum].XCVR_DIAG_HSCLK_DIV__XCVR_DIAG_HSCLK_SEL), 15, 0, (uint32_t)0x0000);
                        CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[laneNum].XCVR_DIAG_HSCLK_DIV__XCVR_DIAG_HSCLK_SEL), 31, 16, (uint32_t)0x0003);
                        CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[laneNum].XCVR_DIAG_PLLDRC_CTRL__XCVR_DIAG_XDP_PWRI_STAT), 31, 16, (uint32_t)0x0013);
                        csl_wiz16m_ct_20b_QSGMII_cmn(serdesLaneEnableParams->baseAddr, laneNum);
                    }
                }
                csl_wiz16m_ct_refclk100MHz_20b_SGMII_QSGMII_cmn_pll_all_vco(serdesLaneEnableParams->baseAddr);
                csl_wiz16m_ct_refclk100MHz_20b_SGMII_QSGMII_cmn_pll1_all_vco(serdesLaneEnableParams->baseAddr);
            }
            else
            {
                result |= CSL_SERDES_INVALID_REF_CLOCK;
            }
        }
    }

    return result;
}
/*! @} */
/* nothing past this point */
