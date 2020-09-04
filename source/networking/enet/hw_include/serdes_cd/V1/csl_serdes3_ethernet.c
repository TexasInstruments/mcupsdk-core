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
 *  @n   (C) Copyright 2019, Texas Instruments, Inc.
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
#include <serdes_cd/V1/cslr_wiz16b8m4ct2.h>
#include <serdes_cd/V1/csl_wiz16m_ct2.h>

CSL_SerdesResult CSL_serdesEthernetInit
(
    CSL_SerdesLaneEnableParams  *serdesLaneEnableParams
)
{

    uint8_t laneNum;
    CSL_SerdesResult result = CSL_SERDES_NO_ERR;

    /* Enable Fast SIM mode */
    if ((serdesLaneEnableParams->operatingMode == CSL_SERDES_FUNCTIONAL_MODE_FAST_SIM) || (serdesLaneEnableParams->operatingMode == CSL_SERDES_DIAGNOSTIC_MODE_FAST_SIM))
    {
        CSL_serdesFastSimEnable(serdesLaneEnableParams->baseAddr);
    }

    if (!(serdesLaneEnableParams->phyType == CSL_SERDES_PHY_TYPE_SGMII
          || serdesLaneEnableParams->phyType == CSL_SERDES_PHY_TYPE_XAUI
          || serdesLaneEnableParams->phyType == CSL_SERDES_PHY_TYPE_QSGMII
          || serdesLaneEnableParams->phyType == CSL_SERDES_PHY_TYPE_USXGMII))
    {
        result |= CSL_SERDES_INVALID_PHY_TYPE;
    }

    if (!(serdesLaneEnableParams->refClock == CSL_SERDES_REF_CLOCK_19p2M ||
	  serdesLaneEnableParams->refClock == CSL_SERDES_REF_CLOCK_20M ||
	  serdesLaneEnableParams->refClock == CSL_SERDES_REF_CLOCK_24M ||
	  serdesLaneEnableParams->refClock == CSL_SERDES_REF_CLOCK_25M ||
	  serdesLaneEnableParams->refClock == CSL_SERDES_REF_CLOCK_26M ||
	  serdesLaneEnableParams->refClock == CSL_SERDES_REF_CLOCK_27M ||
	  serdesLaneEnableParams->refClock == CSL_SERDES_REF_CLOCK_100M || 
	  serdesLaneEnableParams->refClock == CSL_SERDES_REF_CLOCK_125M || 
	  serdesLaneEnableParams->refClock == CSL_SERDES_REF_CLOCK_156p25M))
    {
        result |= CSL_SERDES_INVALID_REF_CLOCK;
    }

    if (!(serdesLaneEnableParams->linkRate == CSL_SERDES_LINK_RATE_1p25G ||
            serdesLaneEnableParams->linkRate == CSL_SERDES_LINK_RATE_3p125G ||
            serdesLaneEnableParams->linkRate == CSL_SERDES_LINK_RATE_5G ||
            serdesLaneEnableParams->linkRate == CSL_SERDES_LINK_RATE_5p15625G ||
            serdesLaneEnableParams->linkRate == CSL_SERDES_LINK_RATE_10p3125G))
    {
        result |= CSL_SERDES_INVALID_LANE_RATE;
    }

    /* Torrent configs */
    if(serdesLaneEnableParams->serdesInstance == CSL_TORRENT_SERDES0)
    {
        /* SGMII 1.25G Baud configs */
        if (serdesLaneEnableParams->phyType == CSL_SERDES_PHY_TYPE_SGMII)
        {
            if (serdesLaneEnableParams->linkRate != CSL_SERDES_LINK_RATE_1p25G)
            {
                result |= CSL_SERDES_INVALID_LANE_RATE;
            }
	    
	    if(serdesLaneEnableParams->refClock == CSL_SERDES_REF_CLOCK_19p2M)
            {
                for(laneNum=0; laneNum< CSL_SERDES_MAX_LANES_TORRENT; laneNum++)
            	{
                    if ((serdesLaneEnableParams->laneMask & (1<<laneNum))!=0)
                    {
                        csl_wiz16m_ct2_refclk19p2MHz_20b_SGMII(serdesLaneEnableParams->baseAddr, laneNum);
                    }
            	}
            } 
	    else if(serdesLaneEnableParams->refClock == CSL_SERDES_REF_CLOCK_20M)
            {
                for(laneNum=0; laneNum< CSL_SERDES_MAX_LANES_TORRENT; laneNum++)
            	{
                    if ((serdesLaneEnableParams->laneMask & (1<<laneNum))!=0)
                    {
                        csl_wiz16m_ct2_refclk20MHz_20b_SGMII(serdesLaneEnableParams->baseAddr, laneNum);
                    }
            	}
            } 
	    else if(serdesLaneEnableParams->refClock == CSL_SERDES_REF_CLOCK_24M)
            {
                for(laneNum=0; laneNum< CSL_SERDES_MAX_LANES_TORRENT; laneNum++)
            	{
                    if ((serdesLaneEnableParams->laneMask & (1<<laneNum))!=0)
                    {
                        csl_wiz16m_ct2_refclk24MHz_20b_SGMII(serdesLaneEnableParams->baseAddr, laneNum);
                    }
            	}
            } 
	    else if(serdesLaneEnableParams->refClock == CSL_SERDES_REF_CLOCK_25M)
            {
                for(laneNum=0; laneNum< CSL_SERDES_MAX_LANES_TORRENT; laneNum++)
            	{
                    if ((serdesLaneEnableParams->laneMask & (1<<laneNum))!=0)
                    {
                        csl_wiz16m_ct2_refclk25MHz_20b_SGMII(serdesLaneEnableParams->baseAddr, laneNum);
                    }
            	}
            } 
	    else if(serdesLaneEnableParams->refClock == CSL_SERDES_REF_CLOCK_26M)
            {
                for(laneNum=0; laneNum< CSL_SERDES_MAX_LANES_TORRENT; laneNum++)
            	{
                    if ((serdesLaneEnableParams->laneMask & (1<<laneNum))!=0)
                    {
                        csl_wiz16m_ct2_refclk26MHz_20b_SGMII(serdesLaneEnableParams->baseAddr, laneNum);
                    }
            	}
            } 	    
	    else if(serdesLaneEnableParams->refClock == CSL_SERDES_REF_CLOCK_27M)
            {
                for(laneNum=0; laneNum< CSL_SERDES_MAX_LANES_TORRENT; laneNum++)
            	{
                    if ((serdesLaneEnableParams->laneMask & (1<<laneNum))!=0)
                    {
                        csl_wiz16m_ct2_refclk27MHz_20b_SGMII(serdesLaneEnableParams->baseAddr, laneNum);
                    }
            	}
            }            
	    else if(serdesLaneEnableParams->refClock == CSL_SERDES_REF_CLOCK_100M)
            {
                for(laneNum=0; laneNum< CSL_SERDES_MAX_LANES_TORRENT; laneNum++)
            	{
                    if ((serdesLaneEnableParams->laneMask & (1<<laneNum))!=0)
                    {
                        csl_wiz16m_ct2_refclk100MHz_20b_SGMII(serdesLaneEnableParams->baseAddr, laneNum);
                    }
            	}
            }
            else if(serdesLaneEnableParams->refClock == CSL_SERDES_REF_CLOCK_125M)
            {
                for(laneNum=0; laneNum< CSL_SERDES_MAX_LANES_TORRENT; laneNum++)
            	{
                    if ((serdesLaneEnableParams->laneMask & (1<<laneNum))!=0)
                    {
                        csl_wiz16m_ct2_refclk125MHz_20b_SGMII(serdesLaneEnableParams->baseAddr, laneNum);
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
            if (serdesLaneEnableParams->linkRate != CSL_SERDES_LINK_RATE_3p125G)
            {
                result |= CSL_SERDES_INVALID_LANE_RATE;
            }

            if(serdesLaneEnableParams->refClock == CSL_SERDES_REF_CLOCK_19p2M)
            {
                for(laneNum=0; laneNum< CSL_SERDES_MAX_LANES_TORRENT; laneNum++)
                {
                    if ((serdesLaneEnableParams->laneMask & (1<<laneNum))!=0)
                    {
                        csl_wiz16m_ct2_refclk19p2MHz_20b_XAUI(serdesLaneEnableParams->baseAddr, laneNum);
                    }
                }
            }
	    else if(serdesLaneEnableParams->refClock == CSL_SERDES_REF_CLOCK_20M)
            {
                for(laneNum=0; laneNum< CSL_SERDES_MAX_LANES_TORRENT; laneNum++)
                {
                    if ((serdesLaneEnableParams->laneMask & (1<<laneNum))!=0)
                    {
                        csl_wiz16m_ct2_refclk20MHz_20b_XAUI(serdesLaneEnableParams->baseAddr, laneNum);
                    }
                }
            }
	    else if(serdesLaneEnableParams->refClock == CSL_SERDES_REF_CLOCK_24M)
            {
                for(laneNum=0; laneNum< CSL_SERDES_MAX_LANES_TORRENT; laneNum++)
                {
                    if ((serdesLaneEnableParams->laneMask & (1<<laneNum))!=0)
                    {
                        csl_wiz16m_ct2_refclk24MHz_20b_XAUI(serdesLaneEnableParams->baseAddr, laneNum);
                    }
                }
            }
            else if(serdesLaneEnableParams->refClock == CSL_SERDES_REF_CLOCK_25M)
            {
                for(laneNum=0; laneNum< CSL_SERDES_MAX_LANES_TORRENT; laneNum++)
                {
                    if ((serdesLaneEnableParams->laneMask & (1<<laneNum))!=0)
                    {
                        csl_wiz16m_ct2_refclk25MHz_20b_XAUI(serdesLaneEnableParams->baseAddr, laneNum);
                    }
                }
            }
            else if(serdesLaneEnableParams->refClock == CSL_SERDES_REF_CLOCK_26M)
            {
                for(laneNum=0; laneNum< CSL_SERDES_MAX_LANES_TORRENT; laneNum++)
                {
                    if ((serdesLaneEnableParams->laneMask & (1<<laneNum))!=0)
                    {
                        csl_wiz16m_ct2_refclk26MHz_20b_XAUI(serdesLaneEnableParams->baseAddr, laneNum);
                    }
                }
            }
            else if(serdesLaneEnableParams->refClock == CSL_SERDES_REF_CLOCK_27M)
            {
                for(laneNum=0; laneNum< CSL_SERDES_MAX_LANES_TORRENT; laneNum++)
                {
                    if ((serdesLaneEnableParams->laneMask & (1<<laneNum))!=0)
                    {
                        csl_wiz16m_ct2_refclk27MHz_20b_XAUI(serdesLaneEnableParams->baseAddr, laneNum);
                    }
                }
            }
	    else if(serdesLaneEnableParams->refClock == CSL_SERDES_REF_CLOCK_156p25M)
            {
                for(laneNum=0; laneNum< CSL_SERDES_MAX_LANES_TORRENT; laneNum++)
                {
                    if ((serdesLaneEnableParams->laneMask & (1<<laneNum))!=0)
                    {
                        csl_wiz16m_ct2_refclk156p25MHz_20b_XAUI(serdesLaneEnableParams->baseAddr, laneNum);
                    }
                }
            }
            else if(serdesLaneEnableParams->refClock == CSL_SERDES_REF_CLOCK_100M)
            {
                for(laneNum=0; laneNum< CSL_SERDES_MAX_LANES_TORRENT; laneNum++)
                {
                    if ((serdesLaneEnableParams->laneMask & (1<<laneNum))!=0)
                    {
                        csl_wiz16m_ct2_refclk100MHz_20b_XAUI(serdesLaneEnableParams->baseAddr, laneNum);
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
            if (serdesLaneEnableParams->linkRate != CSL_SERDES_LINK_RATE_5G)
            {
                result |= CSL_SERDES_INVALID_LANE_RATE;
            }

	    if(serdesLaneEnableParams->refClock == CSL_SERDES_REF_CLOCK_19p2M)
            {
                for(laneNum=0; laneNum< CSL_SERDES_MAX_LANES_TORRENT; laneNum++)
                {
                    if ((serdesLaneEnableParams->laneMask & (1<<laneNum))!=0)
                    {
                        csl_wiz16m_ct2_refclk19p2MHz_20b_QSGMII(serdesLaneEnableParams->baseAddr, laneNum);
                    }
                }
            }
	    else if(serdesLaneEnableParams->refClock == CSL_SERDES_REF_CLOCK_20M)
            {
                for(laneNum=0; laneNum< CSL_SERDES_MAX_LANES_TORRENT; laneNum++)
                {
                    if ((serdesLaneEnableParams->laneMask & (1<<laneNum))!=0)
                    {
                        csl_wiz16m_ct2_refclk20MHz_20b_QSGMII(serdesLaneEnableParams->baseAddr, laneNum);
                    }
                }
            }
	    else if(serdesLaneEnableParams->refClock == CSL_SERDES_REF_CLOCK_24M)
            {
                for(laneNum=0; laneNum< CSL_SERDES_MAX_LANES_TORRENT; laneNum++)
                {
                    if ((serdesLaneEnableParams->laneMask & (1<<laneNum))!=0)
                    {
                        csl_wiz16m_ct2_refclk24MHz_20b_QSGMII(serdesLaneEnableParams->baseAddr, laneNum);
                    }
                }
            }
	    else if(serdesLaneEnableParams->refClock == CSL_SERDES_REF_CLOCK_25M)
            {
                for(laneNum=0; laneNum< CSL_SERDES_MAX_LANES_TORRENT; laneNum++)
                {
                    if ((serdesLaneEnableParams->laneMask & (1<<laneNum))!=0)
                    {
                        csl_wiz16m_ct2_refclk25MHz_20b_QSGMII(serdesLaneEnableParams->baseAddr, laneNum);
                    }
                }
            }
	    else if(serdesLaneEnableParams->refClock == CSL_SERDES_REF_CLOCK_26M)
            {
                for(laneNum=0; laneNum< CSL_SERDES_MAX_LANES_TORRENT; laneNum++)
                {
                    if ((serdesLaneEnableParams->laneMask & (1<<laneNum))!=0)
                    {
                        csl_wiz16m_ct2_refclk26MHz_20b_QSGMII(serdesLaneEnableParams->baseAddr, laneNum);
                    }
                }
            }
	    else if(serdesLaneEnableParams->refClock == CSL_SERDES_REF_CLOCK_27M)
            {
                for(laneNum=0; laneNum< CSL_SERDES_MAX_LANES_TORRENT; laneNum++)
                {
                    if ((serdesLaneEnableParams->laneMask & (1<<laneNum))!=0)
                    {
                        csl_wiz16m_ct2_refclk27MHz_20b_QSGMII(serdesLaneEnableParams->baseAddr, laneNum);
                    }
                }
            }
	    else if(serdesLaneEnableParams->refClock == CSL_SERDES_REF_CLOCK_100M)
            {
                for(laneNum=0; laneNum< CSL_SERDES_MAX_LANES_TORRENT; laneNum++)
                {
                    if ((serdesLaneEnableParams->laneMask & (1<<laneNum))!=0)
                    {
                        csl_wiz16m_ct2_refclk100MHz_20b_QSGMII(serdesLaneEnableParams->baseAddr, laneNum);
                    }
                }
            }
            else if(serdesLaneEnableParams->refClock == CSL_SERDES_REF_CLOCK_125M)
            {
                for(laneNum=0; laneNum< CSL_SERDES_MAX_LANES_TORRENT; laneNum++)
                {
                    if ((serdesLaneEnableParams->laneMask & (1<<laneNum))!=0)
                    {
                        csl_wiz16m_ct2_refclk125MHz_20b_QSGMII(serdesLaneEnableParams->baseAddr, laneNum);
                    }
                }
            }
            else
            {
                result |= CSL_SERDES_INVALID_REF_CLOCK;
            }
        }

        /* USXGMII 5.15625G and 10.3125G Baud configs */
        if (serdesLaneEnableParams->phyType == CSL_SERDES_PHY_TYPE_USXGMII)
        {
            if (!(serdesLaneEnableParams->linkRate == CSL_SERDES_LINK_RATE_5p15625G ||
                  serdesLaneEnableParams->linkRate == CSL_SERDES_LINK_RATE_10p3125G))
            {
                result |= CSL_SERDES_INVALID_LANE_RATE;
            }

            if(serdesLaneEnableParams->refClock == CSL_SERDES_REF_CLOCK_100M)
            {
                for(laneNum=0; laneNum< CSL_SERDES_MAX_LANES_TORRENT; laneNum++)
                {
                    if ((serdesLaneEnableParams->laneMask & (1<<laneNum))!=0)
                    {
                        csl_wiz16m_ct2_refclk100MHz_20b_USXGMII(serdesLaneEnableParams->baseAddr, laneNum);
                    }
                }
            }
            else if(serdesLaneEnableParams->refClock == CSL_SERDES_REF_CLOCK_156p25M)
            {
                for(laneNum=0; laneNum< CSL_SERDES_MAX_LANES_TORRENT; laneNum++)
                {
                    if ((serdesLaneEnableParams->laneMask & (1<<laneNum))!=0)
                    {
                        csl_wiz16m_ct2_refclk156p25MHz_20b_USXGMII(serdesLaneEnableParams->baseAddr, laneNum);
                    }
                }
            }
            else
            {
                result |= CSL_SERDES_INVALID_REF_CLOCK;
            }
        }
    }
    else
    {

    }

    return result;
}
/*! @} */
/* nothing past this point */
