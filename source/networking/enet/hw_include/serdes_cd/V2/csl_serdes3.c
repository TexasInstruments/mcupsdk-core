/**
 * @file  csl_serdes3.c
 *
 * @brief
 *  This is the C implementation for Serdes CSL-FL.
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
#include <ti/csl/csl_serdes.h>
#include <drivers/hw_include/cslr_soc.h>
#include "stdlib.h"
#include <serdes_cd/V2/cslr_wiz16b8m4ct2_nda.h>

CSL_SerdesResult CSL_serdesIPSelect(
        uint32_t                mainCtrlMMRbaseAddr,
        CSL_SerdesPhyType       phyType,
        uint32_t                phyInstanceNum,
        CSL_SerdesInstance      serdesInstance,
        uint32_t                serdeslaneNum
        )
{
    CSL_SerdesResult result = CSL_SERDES_NO_ERR;

    if(serdeslaneNum > 3)
    {
        result |= CSL_SERDES_INVALID_NUM_LANES;
    }

    /* Serdes IP Select in Main Ctrl MMR */
    if(phyType == CSL_SERDES_PHY_TYPE_PCIe)
    {
        if(phyInstanceNum == 0)
        {
            if(serdeslaneNum == 0)
            {
                /* PCIe0 Lane0 */
                CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(mainCtrlMMRbaseAddr + 0x4080),1,0,0x1);
            }
            else if(serdeslaneNum == 1)
            {
                /* PCIe0 Lane1 */
                CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(mainCtrlMMRbaseAddr + 0x4084),1,0,0x1);
            }
            else if(serdeslaneNum == 2)
            {
                /* PCIe0 Lane2 */
                CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(mainCtrlMMRbaseAddr + 0x4088),1,0,0x1);
            }
            else if(serdeslaneNum == 3)
            {
                /* PCIe0 Lane3 */
                CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(mainCtrlMMRbaseAddr + 0x408c),1,0,0x1);
            }
        }
    }

    if(phyType == CSL_SERDES_PHY_TYPE_USB)
    {
            if(phyInstanceNum == 0)
            {
                /* USB3_0 */
                CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(mainCtrlMMRbaseAddr + 0x4084),1,0,0x2);
            }
            if(phyInstanceNum == 1)
            {
                /* USB3_1 */
                CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(mainCtrlMMRbaseAddr + 0x408c),1,0,0x2);
            }
    }

    if(phyType == CSL_SERDES_PHY_TYPE_HYPERLINK)
    {
        if(phyInstanceNum == 0)
        {
            if(serdeslaneNum == 0)
            {
                /* Hyperlink Lane0 */
                CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(mainCtrlMMRbaseAddr + 0x4080),1,0,0x3);
            }
            else if(serdeslaneNum == 1)
            {
                /* Hyperlink Lane1 */
                CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(mainCtrlMMRbaseAddr + 0x4084),1,0,0x3);
            }
            else if(serdeslaneNum == 2)
            {
                /* Hyperlink Lane2 */
                CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(mainCtrlMMRbaseAddr + 0x4088),1,0,0x3);
            }
            else if(serdeslaneNum == 3)
            {
                /* Hyperlink Lane3 */
                CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(mainCtrlMMRbaseAddr + 0x408c),1,0,0x3);
            }
        }
    }

    if(phyType == CSL_SERDES_PHY_TYPE_eDP)
    {
        if(phyInstanceNum == 0)
        {
            if(serdeslaneNum == 0)
            {
                /* eDP Lane0 */
                CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(mainCtrlMMRbaseAddr + 0x4080),1,0,0x0);
            }
            else if(serdeslaneNum == 1)
            {
                /* eDP Lane1 */
                CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(mainCtrlMMRbaseAddr + 0x4084),1,0,0x0);
            }
            else if(serdeslaneNum == 2)
            {
                /* eDP Lane2 */
                CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(mainCtrlMMRbaseAddr + 0x4088),1,0,0x0);
            }
            else if(serdeslaneNum == 3)
            {
                /* eDP Lane3 */
                CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(mainCtrlMMRbaseAddr + 0x408c),1,0,0x0);
            }
        }
    }

    return result;
}

CSL_SerdesResult CSL_serdesRefclkSel(
        uint32_t                mainCtrlMMRbaseAddr,
        uint32_t                baseAddr,
        CSL_SerdesRefClock      refClk,
        CSL_SerdesRefClockSrc   refClkSrc,
        CSL_SerdesInstance      serdesInstance,
        CSL_SerdesPhyType       phyType
        )
{
    CSL_SerdesResult result = CSL_SERDES_NO_ERR;

    CSL_wiz16b8m4ct2Regs *torrent_sds_reg = (CSL_wiz16b8m4ct2Regs *)(uintptr_t)(baseAddr);

    /* PMA_CMN_REFCLK_DIG_DIV */
    if(refClk == CSL_SERDES_REF_CLOCK_100M) /* 100MHz reference clock */
    {
        CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->WIZ_CONFIG.SERDES_TOP_CTRL),27,26,0x2);
    }
    else if(refClk == CSL_SERDES_REF_CLOCK_156p25M) /* 156.25MHz reference clock */
    {
        CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->WIZ_CONFIG.SERDES_TOP_CTRL),27,26,0x3);
    }
    else if(refClk <= CSL_SERDES_REF_CLOCK_27M) /* 19.2MHz-27MHz reference clock */
    {
        CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->WIZ_CONFIG.SERDES_TOP_CTRL),27,26,0x0);
    }

    if(refClkSrc == CSL_SERDES_REF_CLOCK_INT)
    {
        /* PMA_CMN_REFCLK_INT_MODE */
        if(refClk >= CSL_SERDES_REF_CLOCK_100M) /* 100MHz and greater reference clock */
        {
            CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->WIZ_CONFIG.SERDES_TOP_CTRL),29,28,0x1);
        }
        else /* Less than 100MHz reference clock */
        {
            CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->WIZ_CONFIG.SERDES_TOP_CTRL),29,28,0x3);
        }

        /* REFCLK_DIG_SEL = pma_cmn_refclk_int */
        CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->WIZ_CONFIG.SERDES_RST),24,24,0x1);
        /* PLL0_REFCLK_SEL: Selects pma_cmn_refclk_int as reference clock source */
        CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->WIZ_CONFIG.SERDES_RST),28,28,0x1);
        /* PLL1_REFCLK_SEL: Selects pma_cmn_refclk1_int as reference clock source. */
        CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->WIZ_CONFIG.SERDES_RST),29,29,0x1);

        /* Wait 1us before writing to CMN registers */
        CSL_serdesCycleDelay(1000);

        /* Setup MAIN_CTRL_MMR refclk mux for internal refclk and skip setup if mainCtrlMMRbaseAddr is NULL */
        if(mainCtrlMMRbaseAddr != (uintptr_t)NULL)
        {
            if(refClk == CSL_SERDES_REF_CLOCK_156p25M)
            {
                if(serdesInstance == CSL_TORRENT_SERDES0)
                    /*Select core_refclk for serdes0 torrent select MAIN_PLL3_HSDIV4_CLKOUT for 156.25 MHz */
                    CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(mainCtrlMMRbaseAddr + 0x8400),1,0,0x2);
            }
            else if (refClk == CSL_SERDES_REF_CLOCK_100M)
            {
                if(serdesInstance == CSL_TORRENT_SERDES0)
                    /*Select core_refclk for serdes0 torrent select MAIN_PLL2_HSDIV4_CLKOUT for 100 MHz */
                    CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(mainCtrlMMRbaseAddr + 0x8400),1,0,0x3);
            }
            else if (refClk == CSL_SERDES_REF_CLOCK_19p2M)
            {
                if(serdesInstance == CSL_TORRENT_SERDES0)
                    /*Select core_refclk for serdes0 torrent select HFOSC0_CLKOUT for 19.2 MHz */
                    CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(mainCtrlMMRbaseAddr + 0x8400),1,0,0x0);
            }
        }
    }
    else if(refClkSrc == CSL_SERDES_REF_CLOCK_EXT_SSC || CSL_SERDES_REF_CLOCK_EXT_NO_SSC)
    {
        /* PMA_CMN_REFCLK_MODE */
        if(refClk >= CSL_SERDES_REF_CLOCK_100M) /* 100MHz and greater reference clock */
        {
            CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->WIZ_CONFIG.SERDES_TOP_CTRL),31,30,0x0);
        }
        else /* Less than 100MHz reference clock */
        {
            CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->WIZ_CONFIG.SERDES_TOP_CTRL),31,30,0x2);
        }

        /* REFCLK_DIG_SEL = cmn_refclk_<p/m> */
        CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->WIZ_CONFIG.SERDES_RST),24,24,0x0);
        /* PLL0_REFCLK_SEL: Selects cmn_refclk_<m/p> as reference clock source */
        CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->WIZ_CONFIG.SERDES_RST),28,28,0x0);
        /* PLL1_REFCLK_SEL: Selects cmn_refclk_<m/p> as reference clock source */
        CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->WIZ_CONFIG.SERDES_RST),29,29,0x0);

        /* Wait 1us before writing to CMN registers */
        CSL_serdesCycleDelay(1000);
    }

    return result;
}
void CSL_serdesCycleDelay (uint64_t count)
{
  volatile uint64_t sat = 0;
  for(sat=0;sat<count;sat++);
}

void CSL_serdesFastSimEnable(uint32_t baseAddr)
{
    CSL_wiz16b8m4ct2Regs *torrent_sds_reg = (CSL_wiz16b8m4ct2Regs *)(uintptr_t)(baseAddr);

    /* Configure PHY in SPEEDUP */
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PHY_PCS_CMN_REGISTERS.PHY_AUTO_CFG_SPDUP__PHY_PLL_CFG), 17, 17, (uint32_t)0x1);
}

void CSL_serdesPCIeModeSelect(uint32_t baseAddr, CSL_SerdesPCIeGenType  pcieGenType, uint32_t laneNum)
{
    CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(baseAddr + 0x480 + 0x40*laneNum),25, 24, pcieGenType);
}

void CSL_serdesDisablePllAndLanes(uint32_t  baseAddr,
                                  uint32_t  numLanes,
                                  uint8_t   laneMask)
{
     uint32_t laneNum;
     CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(baseAddr + 0x040C),31,31,0x0);

     for(laneNum=0;laneNum<numLanes;laneNum++)
     {
         if ((laneMask & (1<<laneNum))!=0)
         {
             /* disable p0, p1, p2, p3 */
             CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(baseAddr + 0x0480+ (0x40*laneNum)),31, 30, 0x0); /* p0 */
         }
     }
}

void CSL_SerdesInvertLaneTXPolarity(uint32_t baseAddr,
                                  uint32_t laneNum)
{
    /* Invert TX Lane Polarity */
    CSL_FINSR(*(volatile uint32_t *)(baseAddr + 0xf000 + (0x200*laneNum)),8,8,1);
}

void CSL_SerdesInvertLaneRXPolarity(uint32_t baseAddr,
                                  uint32_t laneNum)
{
    /* Invert RX Lane Polarity */
    CSL_FINSR(*(volatile uint32_t *)(baseAddr + 0xf000 + (0x200*laneNum)),0,0,1);
}

void CSL_serdesDisablePLL(uint32_t            baseAddr,
                          CSL_SerdesPhyType   phyType)
{
    CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(baseAddr + 0x040C),31,31,0x0);
}

void CSL_serdesDisableLanes(
        uint32_t    baseAddr,
        uint32_t    laneNum,
        uint8_t     laneMask)
{
    if ((laneMask & (1<<laneNum))!=0) /* control which lanes are affected with bit masking */
    /* disable p0, p1, p2, p3 */
    CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(baseAddr + 0x0480+ (0x40*laneNum)),31, 30, 0x0); /* p0 */
}

void CSL_serdesEnableLanes(uint32_t baseAddr,
                           uint32_t laneNum,
                           CSL_SerdesPhyType   phyType,
                           CSL_SerdesLinkRate  linkRate,
                           CSL_SerdesOperatingMode opMode)
{

    /* Enable P0/P1 depending on laneNum */

    /* PIPE should only enable P0/P1 enable */
    if(phyType == CSL_SERDES_PHY_TYPE_PCIe || phyType == CSL_SERDES_PHY_TYPE_USB)
    {
        CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(baseAddr + 0x480 + (0x40*laneNum)),31, 31, 0x1);
    }
    else if(phyType == CSL_SERDES_PHY_TYPE_HYPERLINK)
    {
        /* RAW mode should enable the force enable. hyperlink shouldnt enable comma align */
        CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(baseAddr + 0x480 + (0x40*laneNum)),31, 28, 0x5);
    }
    else
    {
        /* RAW mode should enable the force enable. SGMII needs comma align and raw auto start */
        CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(baseAddr + 0x480 + (0x40*laneNum)),31, 28, 0x7);
    }

    if(phyType == CSL_SERDES_PHY_TYPE_USXGMII)
    {
        /* Enable cmn_refclk_rcv_out_en */
	    CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(baseAddr + 0xe000),6, 6, 0x1);

	    /* RAW mode should enable the force enable. SGMII needs comma align and raw auto start */
        CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(baseAddr + 0x480 + (0x40*laneNum)),31, 28, 0x5);

        /* Set P0_MAC_SRC_SEL to PMA output cmn_ref_clk_rcv */
        CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(baseAddr + 0x480 + (0x40*laneNum)),21, 20, 0x2);

        /* Set P0_RXFCLK_SEL to rd_div4_clk0 */
        CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(baseAddr + 0x480 + (0x40*laneNum)),7, 6, 0x3);

        if(linkRate == CSL_SERDES_LINK_RATE_10p3125G)
        {
            /* Mode 1 for 10G */
            CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(baseAddr + 0x480 + (0x40*laneNum)),25, 24, 0x1);
            /* Set P0_REFCLK_SEL to pcs_mac_clk_divx0_ln_0 */
            CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(baseAddr + 0x480 + (0x40*laneNum)),19, 18, 0x2);
        }
        else if(linkRate == CSL_SERDES_LINK_RATE_5p15625G)
        {
            /* Mode 0 for 5G */
            CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(baseAddr + 0x480 + (0x40*laneNum)),25, 24, 0x0);
            /* Set P0_REFCLK_SEL to pcs_mac_clk_divx1_ln_0 */
            CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(baseAddr + 0x480 + (0x40*laneNum)),19, 18, 0x3);
        }
    }

    if(phyType == CSL_SERDES_PHY_TYPE_HYPERLINK)
    {
        /* Set MAC clock dividers */
        *(volatile uint32_t *)(uintptr_t)(baseAddr + 0x484 + (0x40*laneNum)) = 0x10002;

        if(linkRate == CSL_SERDES_LINK_RATE_10G)
        {
            /* Mode 1 for 10G */
            CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(baseAddr + 0x480 + (0x40*laneNum)),25, 24, 0x1);
        }
        else if(linkRate == CSL_SERDES_LINK_RATE_5G)
        {
            /* Mode 0 for 5G */
            CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(baseAddr + 0x480 + (0x40*laneNum)),25, 24, 0x0);
        }
    }
	

    if(phyType == CSL_SERDES_PHY_TYPE_SGMII || phyType == CSL_SERDES_PHY_TYPE_SGMII_ICSSG)
    {
        /* Set FULLRT div to /4 to get 125MHz */
        CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(baseAddr + 0x480 + (0x40*laneNum)),23, 22, 0x2);

        /* Enable RX 2x clock enable */
        CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(baseAddr + 0x4000 + (laneNum*0x400) + 0x1d0),31,16, (uint32_t)0x4000);

        /* Set MAC clock dividers */
        *(volatile uint32_t *)(uintptr_t)(baseAddr + 0x484 + (0x40*laneNum)) = 0x10002;
    }

    if(phyType == CSL_SERDES_PHY_TYPE_XAUI)
    {
        /* Set FULLRT div to /2 to get 312.5MHz */
        CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(baseAddr + 0x480 + (0x40*laneNum)),23, 22, 0x0);

        /* Enable RX 2x clock enable */
        CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(baseAddr + 0x4000 + (laneNum*0x400) + 0x1d0),31,16, (uint32_t)0x4000);

        /* Set MAC clock dividers */
        *(volatile uint32_t *)(uintptr_t)(baseAddr + 0x484 + (0x40*laneNum)) = 0x10002;
    }

    if(phyType == CSL_SERDES_PHY_TYPE_QSGMII)
    {
        /* Set standard mode to 1 */
        CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(baseAddr + 0x480 + (0x40*laneNum)),25, 24, 0x1);

        /* Set MAC clock dividers */
        *(volatile uint32_t *)(uintptr_t)(baseAddr + 0x484 + (0x40*laneNum)) = 0x10002;

        /* Enable RX 2x clock enable */
        CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(baseAddr + 0x4000 + (laneNum*0x400) + 0x1d0),31,16, (uint32_t)0x4000);
    }
	
	if(phyType == CSL_SERDES_PHY_TYPE_USXGMII)
	{
        /* Set MAC clock dividers */
        *(volatile uint32_t *)(uintptr_t)(baseAddr + 0x484 + (0x40*laneNum)) = 0x10002;
	}	

    if(phyType == CSL_SERDES_PHY_TYPE_USB)
    {
        /* Enable P0_FULLRT_DIV/P1_FULLRT_DIV to /4 for 125MHz pipe clock */
        CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(baseAddr + 0x480 + laneNum*0x40),23, 22, 0x2);
    }
}

void CSL_serdesRxEqDisable(uint32_t baseAddr)
{
    CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(baseAddr +0x1400),7,0, 0x3E);
}

void CSL_serdesPorReset(uint32_t baseAddr)
{
    CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(baseAddr + 0x404),31,31, 0x1);

    CSL_serdesCycleDelay(1000);

    CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(baseAddr + 0x404),31,31, 0x0);
}

void CSL_serdesSetLoopback
(
 uint32_t baseAddr,
 uint32_t laneNum,
 CSL_SerdesLoopback loopbackMode,
 CSL_SerdesInstance serdesInstance,
 CSL_SerdesPhyType phyType
)
{
    CSL_wiz16b8m4ct2Regs *torrent_sds_reg = (CSL_wiz16b8m4ct2Regs *)(uintptr_t)(baseAddr);

    if (loopbackMode == CSL_SERDES_LOOPBACK_SER)
    {
        if(serdesInstance == CSL_TORRENT_SERDES0) /* Torrent serial loopback */
        {
            CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->PHY_PMA_LANE_REGISTERS[laneNum].PHY_PMA_XCVR_LPBK__PHY_PMA_XCVR_CTRL),16, 16, (uint32_t)0x1);
        }
    }
    else if (loopbackMode == CSL_SERDES_LOOPBACK_LINE)
    {
        if(serdesInstance == CSL_TORRENT_SERDES0) /* Torrent LINE loopback */
        {
            CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->PHY_PMA_LANE_REGISTERS[laneNum].PHY_PMA_XCVR_LPBK__PHY_PMA_XCVR_CTRL),18, 18, (uint32_t)0x1);
        }
    }
}

void CSL_serdesReleaseReset
(
 uint32_t               baseAddr
)
{
    CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(baseAddr + 0x40C),31,31, (uint32_t)0x1);
}

CSL_SerdesStatus CSL_serdesGetPLLStatus
(
 uint32_t baseAddr,
 uint32_t laneMask,
 CSL_SerdesInstance serdesInstance
)
{
    CSL_SerdesStatus retval = CSL_SERDES_STATUS_PLL_LOCKED;
    uint32_t laneNum;

    if(serdesInstance != CSL_TORRENT_SERDES0)
    {
        for(laneNum=0;laneNum<CSL_SERDES_MAX_LANES;laneNum++)
        {
            if ((laneMask & (1<<laneNum))!=0)
            {
                retval &= (CSL_SerdesStatus)CSL_FEXTR(*(volatile uint32_t *)(uintptr_t)(baseAddr + 0x4000 + 0x400*laneNum +0x88), 0, 0);
            }
        }
    }
    else
    {
        retval &= (CSL_SerdesStatus)CSL_FEXTR(*(volatile uint32_t *)(uintptr_t)(baseAddr + 0xE000), 22, 22);
    }
    return retval;
}

CSL_SerdesStatus CSL_serdesGetLaneStatus
(
 uint32_t baseAddr,
 uint32_t numLanes,
 uint8_t laneMask,
 CSL_SerdesPhyType phyType
)
{
    uint32_t laneNum;
    CSL_SerdesStatus retval = CSL_SERDES_STATUS_PLL_LOCKED;

    for (laneNum=0; laneNum < CSL_SERDES_MAX_LANES; laneNum++)
    {
        if ((laneMask & (1<<laneNum))!=0)
        {
		/* Poll for Lane OK State */
		retval &= (CSL_SerdesStatus)CSL_FEXTR(*(volatile uint32_t *)(uintptr_t)(baseAddr + 0x48C + laneNum * 0x40), 0, 0);
        }
    }
    return retval;
}

CSL_SerdesPIPEStatus CSL_serdesGetPIPEClkStatus
(
 uint32_t baseAddr,
 uint8_t laneMask,
 CSL_SerdesPhyType phyType
)
{
    uint32_t laneNum;
    CSL_SerdesPIPEStatus retval = CSL_SERDES_STATUS_PIPE_CLK_VALID;

    for (laneNum=0; laneNum < CSL_SERDES_MAX_LANES; laneNum++)
    {
        if ((laneMask & (1<<laneNum))!=0)
        {
        /* Poll for PIPE Clock Valid State */
        retval |= (CSL_SerdesStatus)CSL_FEXTR(*(volatile uint32_t *)(uintptr_t)(baseAddr + 0xd000 + 0x200*laneNum +0x14), 17, 17);
        }
    }

    if(retval == 0) /* PIPE clock valid is Active low */
    {
        retval = CSL_SERDES_STATUS_PIPE_CLK_VALID;
    }
    else

        retval = CSL_SERDES_STATUS_PIPE_CLK_NOT_VALID;

    return retval;
}

CSL_SerdesLaneEnableStatus CSL_serdesLaneEnable
(
         CSL_SerdesLaneEnableParams *serdesLaneEnableParams
)
{
    uint32_t laneNum;
    CSL_SerdesStatus   pllstat;
    CSL_SerdesLaneEnableStatus status = CSL_SERDES_LANE_ENABLE_NO_ERR;
    CSL_wiz16b8m4ct2Regs *torrent_sds_reg = (CSL_wiz16b8m4ct2Regs *)(uintptr_t)(serdesLaneEnableParams->baseAddr);

    /* Set LANE loopback bits based on the loopback type */
    if(serdesLaneEnableParams->operatingMode != CSL_SERDES_DIAGNOSTIC_MODE)
    {
        for(laneNum=0;laneNum<CSL_SERDES_MAX_LANES;laneNum++)
        {
            if ((serdesLaneEnableParams->laneMask & (1<<laneNum))!=0)
            {
                CSL_serdesSetLoopback(serdesLaneEnableParams->baseAddr, laneNum, serdesLaneEnableParams->loopbackMode[laneNum], serdesLaneEnableParams->serdesInstance, serdesLaneEnableParams->phyType);
            }
        }
    }

    /* Enable Refclk Out when the refclkout flag is set */
    if(serdesLaneEnableParams->refClkOut == CSL_SERDES_REFCLK_OUT_EN)
    {
        if(serdesLaneEnableParams->refClkSrc == CSL_SERDES_REF_CLOCK_INT)
        {
            /* 1. phy_en_refclk = 1'b0 (wiz) */
            CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->WIZ_CONFIG.SERDES_RST),30,30,0x0);
            CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->PHY_PCS_CMN_REGISTERS.PHY_ISO_CMN_CTRL),8,8,0x0);
            /* 2. Write 0x0210 to CMN_CDIAG_REFCLK_DRV0_CTRL to select derived reference cock */
            CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_CDIAG_REFCLK_DRV0_CTRL),15,0,0x0210);
            /* 3. Write 0x0118 to CMN_CDIAG_REFCLK_OVRD to enable derived reference clock */
            CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_CDIAG_REFCLK_TEST__CMN_CDIAG_REFCLK_OVRD),15,0,0x0118);
            /* 4. Write 0x0401 to PHY_PIPE_CMN_CTRL1 to enable phy_en_refclk pin input */
            CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->PHY_PCS_CMN_REGISTERS.PHY_PIPE_CMN_CTRL2__PHY_PIPE_CMN_CTRL1),15,0,0x0401);
            /* 5. Set phy_en_refclk = 1'b1 */
            CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->WIZ_CONFIG.SERDES_RST),30,30,0x1);
            CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->PHY_PCS_CMN_REGISTERS.PHY_ISO_CMN_CTRL),8,8,0x1);
        }
    }

    /* Enable lanes in register */
    for(laneNum=0;laneNum<CSL_SERDES_MAX_LANES;laneNum++)
    {
        if ((serdesLaneEnableParams->laneMask & (1<<laneNum))!=0)
        {
            CSL_serdesEnableLanes(serdesLaneEnableParams->baseAddr, laneNum, serdesLaneEnableParams->phyType, serdesLaneEnableParams->linkRate, serdesLaneEnableParams->operatingMode);
        }
    }

    /* Release PHY Reset*/
    /* If diagnostic mode, reset and lock checks are handled by isolation and test functions */
    if(!(serdesLaneEnableParams->operatingMode == CSL_SERDES_DIAGNOSTIC_MODE))
    {
        CSL_serdesReleaseReset(serdesLaneEnableParams->baseAddr);

        CSL_serdesCycleDelay(100);

        /* Poll for PLL_OK */
        /* Do not poll for FAST SIM or PIPE mode */
        if (!(serdesLaneEnableParams->operatingMode == CSL_SERDES_FUNCTIONAL_MODE_FAST_SIM ||
                serdesLaneEnableParams->operatingMode == CSL_SERDES_DIAGNOSTIC_MODE_FAST_SIM ||
                serdesLaneEnableParams->phyType == CSL_SERDES_PHY_TYPE_USB ||
                serdesLaneEnableParams->phyType == CSL_SERDES_PHY_TYPE_PCIe))
        {
            do
            {
                pllstat = CSL_serdesGetPLLStatus(serdesLaneEnableParams->baseAddr,
                                                  serdesLaneEnableParams->laneMask,
                                                  serdesLaneEnableParams->serdesInstance);
            }while(pllstat == CSL_SERDES_STATUS_PLL_NOT_LOCKED);
        }

        CSL_serdesCycleDelay(100);
    }

	return status;
}

