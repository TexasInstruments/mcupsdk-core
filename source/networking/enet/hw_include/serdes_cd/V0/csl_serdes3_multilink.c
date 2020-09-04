/**
 * @file csl_serdes3_multilink.c
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
 * @defgroup CSL_SERDES_MULTILINK SERDES MULTILINK
 * @ingroup CSL_SERDES_API
 *
 * @section Introduction
 *
 * @subsection xxx Overview
 * This module deals with setting up SERDES configuration for MULTILINK. The API flow should be as follows:   \n
 *
 * CSL_serdesMultiLinkInit (baseAddr, numLanes, refClock, rate);
 *
 * CSL_serdesLaneEnable (&serdesLaneEnableParams);
 *
 * @subsection References
 *
 * ============================================================================
 */
#include <ti/csl/csl_serdes.h>
#include <serdes_cd/V0/csl_serdes3_multilink.h>
#include <serdes_cd/V0/cslr_wiz16b4m4cs.h>
#include <serdes_cd/V0/cslr_wiz16b8m4ct.h>
#include <serdes_cd/V0/csl_wiz16m_cs.h>
#include <serdes_cd/V0/csl_wiz16m_ct.h>

CSL_SerdesResult CSL_serdesMultiLinkInit
(
    CSL_SerdesMultilink         multiLink,
    CSL_SerdesInstance          serdesInstance,
    CSL_SerdesLaneEnableParams  *serdesLaneEnableParams,
    CSL_SerdesLaneEnableParams  *serdesLaneEnableParams1
)
{

    uint8_t laneNum;
    CSL_SerdesResult result = CSL_SERDES_NO_ERR;

    /* Enable Fast SIM mode */
    #ifdef CSL_SERDES_FAST_SIM_MODE
    CSL_serdesFastSimEnable(serdesLaneEnableParams->baseAddr, serdesLaneEnableParams->numLanes, serdesLaneEnableParams->phyType);
    CSL_serdesFastSimEnable(serdesLaneEnableParams1->baseAddr, serdesLaneEnableParams1->numLanes, serdesLaneEnableParams1->phyType);
    #endif

    if(serdesInstance != CSL_TORRENT_SERDES0)
    {
        CSL_wiz16b4m4csRegs *sierra_sds_reg = (CSL_wiz16b4m4csRegs *)(uintptr_t)(serdesLaneEnableParams->baseAddr);

        /* PHY_PLL_CFG for multilink */
        CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&sierra_sds_reg->PHY_PCS_CMN_REGISTERS.PHY_AUTO_CFG_CTRL__PHY_PLL_CFG), 15, 0, (uint32_t)0x2);

        if(multiLink == CSL_SERDES_XAUI_SGMII_MULTILINK)
        {
            if (serdesLaneEnableParams->linkRate == CSL_SERDES_LINK_RATE_3p125G)
            {
                if(serdesLaneEnableParams->refClock == CSL_SERDES_REF_CLOCK_156p25M)
                {
                    csl_wiz16m_cs_refclk156p25MHz_20b_XAUI_cmn_multilink_pll0(serdesLaneEnableParams->baseAddr);
                    csl_wiz16m_cs_refclk156p25MHz_20b_XAUI_cmn_pll_multilink_pll0(serdesLaneEnableParams->baseAddr);

                    for(laneNum=0;laneNum<CSL_SERDES_MAX_LANES;laneNum++)
                    {
                       if ((serdesLaneEnableParams->laneMask & (1<<laneNum))!=0)
                       {
                           csl_wiz16m_cs_refclk156p25MHz_20b_XAUI_ln_multilink_pll0(serdesLaneEnableParams->baseAddr, laneNum);
                       }
               }
            }
                else
                {
                    result = CSL_SERDES_INVALID_REF_CLOCK;
                }
        }

            if (serdesLaneEnableParams1->linkRate == CSL_SERDES_LINK_RATE_1p25G)
            {
                if(serdesLaneEnableParams1->refClock == CSL_SERDES_REF_CLOCK_100M)
                {
                    csl_wiz16m_cs_refclk100MHz_20b_SGMII_cmn_multilink_pll1(serdesLaneEnableParams1->baseAddr);
                    csl_wiz16m_cs_refclk100MHz_20b_SGMII_cmn_pll_multilink_pll1(serdesLaneEnableParams1->baseAddr);

                for(laneNum=0;laneNum<CSL_SERDES_MAX_LANES;laneNum++)
                {
                    if ((serdesLaneEnableParams1->laneMask & (1<<laneNum))!=0)
                    {
                            csl_wiz16m_cs_refclk100MHz_20b_SGMII_ln_multilink_pll1(serdesLaneEnableParams1->baseAddr, laneNum);
                        }
                    }
            }
                else
                {
                    result = CSL_SERDES_INVALID_REF_CLOCK;
                }
            }
        }

        if(multiLink == CSL_SERDES_XAUI_QSGMII_MULTILINK)
        {
            if (serdesLaneEnableParams->linkRate == CSL_SERDES_LINK_RATE_3p125G)
            {
                if(serdesLaneEnableParams->refClock == CSL_SERDES_REF_CLOCK_156p25M)
                {
                    csl_wiz16m_cs_refclk156p25MHz_20b_XAUI_cmn_multilink_pll0(serdesLaneEnableParams->baseAddr);
                    csl_wiz16m_cs_refclk156p25MHz_20b_XAUI_cmn_pll_multilink_pll0(serdesLaneEnableParams->baseAddr);

                    for(laneNum=0;laneNum<CSL_SERDES_MAX_LANES;laneNum++)
                    {
                       if ((serdesLaneEnableParams->laneMask & (1<<laneNum))!=0)
                       {
                           csl_wiz16m_cs_refclk156p25MHz_20b_XAUI_ln_multilink_pll0(serdesLaneEnableParams->baseAddr, laneNum);
                       }
               }
            }
                else
                {
                    result = CSL_SERDES_INVALID_REF_CLOCK;
                }
        }

            if (serdesLaneEnableParams1->linkRate == CSL_SERDES_LINK_RATE_5G)
            {
                if(serdesLaneEnableParams1->refClock == CSL_SERDES_REF_CLOCK_100M)
                {
                    csl_wiz16m_cs_refclk100MHz_20b_QSGMII_cmn_multilink_pll1(serdesLaneEnableParams1->baseAddr);
                    csl_wiz16m_cs_refclk100MHz_20b_QSGMII_cmn_pll_multilink_pll1(serdesLaneEnableParams1->baseAddr);

                for(laneNum=0;laneNum<CSL_SERDES_MAX_LANES;laneNum++)
                {
                    if ((serdesLaneEnableParams1->laneMask & (1<<laneNum))!=0)
                    {
                            csl_wiz16m_cs_refclk100MHz_20b_QSGMII_ln_multilink_pll1(serdesLaneEnableParams1->baseAddr, laneNum);
                        }
                    }
            }
                else
                {
                    result = CSL_SERDES_INVALID_REF_CLOCK;
                }
            }
        }

        if(multiLink == CSL_SERDES_QSGMII_SGMII_MULTILINK)
        {
            if (serdesLaneEnableParams->linkRate == CSL_SERDES_LINK_RATE_5G)
            {
                if(serdesLaneEnableParams->refClock == CSL_SERDES_REF_CLOCK_100M)
                {
                    csl_wiz16m_cs_refclk100MHz_20b_QSGMII_cmn_pll_multilink_pll0(serdesLaneEnableParams->baseAddr);

                    for(laneNum=0;laneNum<CSL_SERDES_MAX_LANES;laneNum++)
                    {
                       if ((serdesLaneEnableParams->laneMask & (1<<laneNum))!=0)
                       {
                           csl_wiz16m_cs_refclk100MHz_20b_QSGMII_ln_multilink_pll0(serdesLaneEnableParams->baseAddr, laneNum);
                       }
               }
            }
                else
                {
                    result = CSL_SERDES_INVALID_REF_CLOCK;
                }
        }

            if (serdesLaneEnableParams1->linkRate == CSL_SERDES_LINK_RATE_1p25G)
            {
                if(serdesLaneEnableParams1->refClock == CSL_SERDES_REF_CLOCK_100M)
                {
                    csl_wiz16m_cs_refclk100MHz_20b_SGMII_cmn_multilink_pll1_opt3(serdesLaneEnableParams1->baseAddr);
                    csl_wiz16m_cs_refclk100MHz_20b_SGMII_cmn_pll_multilink_pll1_opt3(serdesLaneEnableParams1->baseAddr);

                for(laneNum=0;laneNum<CSL_SERDES_MAX_LANES;laneNum++)
                {
                    if ((serdesLaneEnableParams1->laneMask & (1<<laneNum))!=0)
                    {
                            csl_wiz16m_cs_refclk100MHz_20b_SGMII_ln_multilink_pll1_opt3(serdesLaneEnableParams1->baseAddr, laneNum);
                        }
                    }
            }
                else
                {
                    result = CSL_SERDES_INVALID_REF_CLOCK;
                }
            }
        }
        if(multiLink == CSL_SERDES_QSGMII_USB_MULTILINK)
        {
            if (serdesLaneEnableParams->linkRate == CSL_SERDES_LINK_RATE_5G)
            {
                if(serdesLaneEnableParams->refClock == CSL_SERDES_REF_CLOCK_100M)
                {
                    csl_wiz16m_cs_refclk100MHz_20b_QSGMII_cmn_pll_multilink_pll0(serdesLaneEnableParams->baseAddr);

                    for(laneNum=0;laneNum<CSL_SERDES_MAX_LANES;laneNum++)
                    {
                       if ((serdesLaneEnableParams->laneMask & (1<<laneNum))!=0)
                       {
                           csl_wiz16m_cs_refclk100MHz_20b_QSGMII_ln_multilink_pll0(serdesLaneEnableParams->baseAddr, laneNum);
                       }
                    }
                }
                else
                {
                    result = CSL_SERDES_INVALID_REF_CLOCK;
                }
            }

            if (serdesLaneEnableParams1->linkRate == CSL_SERDES_LINK_RATE_5G)
            {
                if(serdesLaneEnableParams1->refClock == CSL_SERDES_REF_CLOCK_100M)
                {
                    if(serdesLaneEnableParams1->SSC_mode == CSL_SERDES_EXTERNAL_SSC)
                    {
                        csl_wiz16m_cs_refclk100MHz_20b_USB_cmn_ext_ssc_multilink_pll1(serdesLaneEnableParams1->baseAddr);
                        csl_wiz16m_cs_refclk100MHz_20b_USB_cmn_pll_ext_ssc_multilink_pll1(serdesLaneEnableParams1->baseAddr);

                        for(laneNum=0;laneNum<CSL_SERDES_MAX_LANES;laneNum++)
                        {
                            if ((serdesLaneEnableParams1->laneMask & (1<<laneNum))!=0)
                            {
                                csl_wiz16m_cs_refclk100MHz_20b_USB_ln_ext_ssc_multilink_pll1(serdesLaneEnableParams1->baseAddr, laneNum);
                            }
                        }
                    }
                    else if(serdesLaneEnableParams1->SSC_mode == CSL_SERDES_INTERNAL_SSC)
                    {

                    }
                }
                else
                {
                    result = CSL_SERDES_INVALID_REF_CLOCK;
                }
            }
        }
        if(multiLink == CSL_SERDES_PCIe_USB_MULTILINK)
        {
            if(serdesLaneEnableParams->refClock == CSL_SERDES_REF_CLOCK_100M)
            {
                if(serdesLaneEnableParams->SSC_mode == CSL_SERDES_EXTERNAL_SSC)
                {
                    csl_wiz16m_cs_refclk100MHz_32b_PCIe_cmn_pll_ext_ssc_multilink_pll0(serdesLaneEnableParams->baseAddr);

                    for(laneNum=0;laneNum<CSL_SERDES_MAX_LANES;laneNum++)
                    {
                       if ((serdesLaneEnableParams->laneMask & (1<<laneNum))!=0)
                       {
                           csl_wiz16m_cs_refclk100MHz_32b_PCIe_ln_ext_ssc_multilink_pll0(serdesLaneEnableParams->baseAddr, laneNum);
                       }
                    }
                }
                else if(serdesLaneEnableParams->SSC_mode == CSL_SERDES_INTERNAL_SSC)
                {
                    csl_wiz16m_cs_refclk100MHz_32b_PCIe_cmn_pll_int_ssc_multilink_pll0(serdesLaneEnableParams->baseAddr);

                    for(laneNum=0;laneNum<CSL_SERDES_MAX_LANES;laneNum++)
                    {
                       if ((serdesLaneEnableParams->laneMask & (1<<laneNum))!=0)
                       {
                           csl_wiz16m_cs_refclk100MHz_32b_PCIe_ln_int_ssc_multilink_pll0(serdesLaneEnableParams->baseAddr, laneNum);
                       }
                    }
                }
                else if(serdesLaneEnableParams->SSC_mode == CSL_SERDES_NO_SSC)
                {
                    csl_wiz16m_cs_refclk100MHz_32b_PCIe_cmn_pll_no_ssc_multilink_pll0(serdesLaneEnableParams->baseAddr);

                    for(laneNum=0;laneNum<CSL_SERDES_MAX_LANES;laneNum++)
                    {
                       if ((serdesLaneEnableParams->laneMask & (1<<laneNum))!=0)
                       {
                           csl_wiz16m_cs_refclk100MHz_32b_PCIe_ln_no_ssc_multilink_pll0(serdesLaneEnableParams->baseAddr, laneNum);
                       }
                    }
                }
            }
            else
            {
                result = CSL_SERDES_INVALID_REF_CLOCK;
            }

            if (serdesLaneEnableParams1->linkRate == CSL_SERDES_LINK_RATE_5G)
            {
                if(serdesLaneEnableParams1->refClock == CSL_SERDES_REF_CLOCK_100M)
                {
                    if(serdesLaneEnableParams1->SSC_mode == CSL_SERDES_EXTERNAL_SSC)
                    {
                        csl_wiz16m_cs_refclk100MHz_20b_USB_cmn_ext_ssc_multilink_pll1(serdesLaneEnableParams1->baseAddr);
                        csl_wiz16m_cs_refclk100MHz_20b_USB_cmn_pll_ext_ssc_multilink_pll1(serdesLaneEnableParams1->baseAddr);

                        for(laneNum=0;laneNum<CSL_SERDES_MAX_LANES;laneNum++)
                        {
                            if ((serdesLaneEnableParams1->laneMask & (1<<laneNum))!=0)
                            {
                                csl_wiz16m_cs_refclk100MHz_20b_USB_ln_ext_ssc_multilink_pll1(serdesLaneEnableParams1->baseAddr, laneNum);
                            }
                        }
                    }
                    else if(serdesLaneEnableParams1->SSC_mode == CSL_SERDES_INTERNAL_SSC)
                    {

                    }
                }
                else
                {
                    result = CSL_SERDES_INVALID_REF_CLOCK;
                }
            }
        }
    }
    else /* Torrent Multilink */
    {
        CSL_wiz16b8m4ctRegs *torrent_sds_reg = (CSL_wiz16b8m4ctRegs *)(uintptr_t)(serdesLaneEnableParams->baseAddr);

        /* PHY_PLL_CFG for using both PLLs as multilink */
        CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->PHY_PCS_CMN_REGISTERS.PHY_AUTO_CFG_SPDUP), 15, 0, (uint32_t)0x2);
    }

    return result;
}
/*! @} */
/* nothing past this point */
