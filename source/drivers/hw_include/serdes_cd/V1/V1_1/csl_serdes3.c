/**
 * @file  csl_serdes3.c
 *
 * @brief
 *  This is the C implementation for Serdes CSL-FL.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2022, Texas Instruments, Inc.
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

#include <drivers/hw_include/cslr_serdes.h>
#include <drivers/soc.h>
#include "stdlib.h"
#include <drivers/hw_include/serdes_cd/V1/cslr_wiz16b8m4ct2.h>

CSL_SerdesResult CSL_serdesIPSelect
(
        uint32_t                mainCtrlMMRbaseAddr,
        CSL_SerdesPhyType       phyType,
        uint32_t                phyInstanceNum,
        CSL_SerdesInstance      serdesInstance,
        uint32_t                serdeslaneNum
)
{
    CSL_SerdesResult result = CSL_SERDES_NO_ERR;

    if(serdeslaneNum > 0)
    {
        result |= CSL_SERDES_INVALID_NUM_LANES;
    }

    /* Serdes IP Select in Main Ctrl MMR */
    if(phyType != CSL_SERDES_PHY_TYPE_USB &&
       phyType != CSL_SERDES_PHY_TYPE_PCIe)
	{
        result |= CSL_SERDES_INVALID_PHY_TYPE;
    }

    if(phyType == CSL_SERDES_PHY_TYPE_PCIe)
    {
        if(phyInstanceNum == 0)
        {
#if defined(CSL_MAIN_CTRL_MMR_CFG0_SERDES0_LN0_CTRL)
            if(serdeslaneNum == 0)
            {
                /* PCIe0 Lane0 */
                CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(mainCtrlMMRbaseAddr + CSL_MAIN_CTRL_MMR_CFG0_SERDES0_LN0_CTRL),1,0,0x0);
            }
#endif
        }
    }

    if(phyType == CSL_SERDES_PHY_TYPE_USB)
    {

#if defined(CSL_MAIN_CTRL_MMR_CFG0_SERDES0_LN0_CTRL)
            if(phyInstanceNum == 0)
            {
                /* USB3_0 */
                CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(mainCtrlMMRbaseAddr + CSL_MAIN_CTRL_MMR_CFG0_SERDES0_LN0_CTRL),1,0,0x1);
            }
#endif
    }

    return result;
}

CSL_SerdesResult CSL_serdesRefclkSel
(
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
            if(refClk == CSL_SERDES_REF_CLOCK_25M)
            {
                //select HFOSC0_CLKOUT for 25MHz
                CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(mainCtrlMMRbaseAddr + CSL_MAIN_CTRL_MMR_CFG0_SERDES0_CLKSEL),1,0,0x0);
            }
            else if (refClk == CSL_SERDES_REF_CLOCK_100M)
            {
                //select MAIN_PLL2_HSDIV4_CLKOUT for 100 MHz
                CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(mainCtrlMMRbaseAddr + CSL_MAIN_CTRL_MMR_CFG0_SERDES0_CLKSEL),1,0,0x3);
            }
            else if (refClk == CSL_SERDES_REF_CLOCK_125M)
            {
                //select MAIN_PLL0_HSDIV8_CLKOUT for 125 MHz
                CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(mainCtrlMMRbaseAddr + CSL_MAIN_CTRL_MMR_CFG0_SERDES0_CLKSEL),1,0,0x2);
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

        /* Disable refclk termination for PCIe using ext refclk */
        if(phyType == CSL_SERDES_PHY_TYPE_PCIe)
        {
            CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->WIZ_CONFIG.SERDES_RST),27,27,0x1);
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

void CSL_serdesPCIeModeSelect
(
        uint32_t                baseAddr,
        CSL_SerdesPCIeGenType   pcieGenType,
        uint32_t                laneNum
)
{
    CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(baseAddr + 0x480 + 0x40*laneNum),25, 24, pcieGenType);
}

void CSL_serdesDisablePllAndLanes
(
        uint32_t  baseAddr,
        uint32_t  numLanes,
        uint8_t   laneMask
)
{
     uint32_t laneNum;
     CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(baseAddr + 0x040C),31,31,0x0);

     for(laneNum=0;laneNum<numLanes;laneNum++)
     {
         if ((laneMask & (1<<laneNum))!=0)
         {
             /* disable p0, p1, p2, p3 */
             CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(baseAddr + 0x0480+ (0x40*laneNum)),31, 30, 0x0); //p0
         }
     }
}

void CSL_serdesDisablePLL
(
        uint32_t            baseAddr,
        CSL_SerdesPhyType   phyType
)
{
    CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(baseAddr + 0x040C),31,31,0x0);
}

void CSL_serdesDisableLanes
(
        uint32_t    baseAddr,
        uint32_t    laneNum,
        uint8_t     laneMask
)
{
    if ((laneMask & (1<<laneNum))!=0) //control which lanes are affected with bit masking
    //disable p0, p1, p2, p3
    CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(baseAddr + 0x0480+ (0x40*laneNum)),31, 30, 0x0); //p0
}

void CSL_serdesEnableLanes
(
        uint32_t            	baseAddr,
        uint32_t            	laneNum,
        CSL_SerdesPhyType   	phyType,
        CSL_SerdesLinkRate  	linkRate,
		CSL_SerdesOperatingMode opMode
)
{

    /* Enable P0/P1 depending on laneNum */
    CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(baseAddr + 0x480 + (0x40*laneNum)),31, 31, 0x1);

    if(phyType == CSL_SERDES_PHY_TYPE_PCIe)
	{
		if(opMode == CSL_SERDES_FUNCTIONAL_MODE_TB){ //Internal Testbench
		    /* Enable P0_FULLRT_DIV/P1_FULLRT_DIV to /2 for 250MHz pipe clock */
			CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(baseAddr + 0x480 + (0x40*laneNum)),23, 22, 0x0);
		}else{
			/* Enable P0_FULLRT_DIV/P1_FULLRT_DIV to /1 for 250MHz pipe clock */
			CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(baseAddr + 0x480 + (0x40*laneNum)),23, 22, 0x1);
		}
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
        uint32_t            baseAddr,
        uint32_t            laneNum,
        CSL_SerdesLoopback  loopbackMode,
        CSL_SerdesInstance  serdesInstance,
        CSL_SerdesPhyType   phyType
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

void CSL_serdesReleaseReset(uint32_t baseAddr)
{
    CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(baseAddr + 0x40C),31,31, (uint32_t)0x1);
}

CSL_SerdesStatus CSL_serdesGetPLLStatus
(
        uint32_t            baseAddr,
        uint32_t            laneMask,
        CSL_SerdesInstance  serdesInstance
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
        uint32_t            baseAddr,
        uint32_t            numLanes,
        uint8_t             laneMask,
        CSL_SerdesPhyType   phyType
)
{
    uint32_t laneNum;
    CSL_SerdesStatus retval = CSL_SERDES_STATUS_PLL_LOCKED;
    for (laneNum=0; laneNum < numLanes; laneNum++)
    {
        if ((laneMask & (1<<laneNum))!=0)
        {
		/* Poll for Lane OK State */
		retval &= (CSL_SerdesStatus)CSL_FEXTR(*(volatile uint32_t *)(uintptr_t)(baseAddr + 0x48C + laneNum * 0x40), 0, 0);
        }
    }
    return retval;
}

CSL_SerdesStatus CSL_serdesConfigStatus
(
 uint32_t baseAddr
)
{
    CSL_SerdesStatus retval;
    /* Return the PHY_RESET_N status /0 = not configured, /1 = serdes is configured and reset is deasserted */
    retval = (CSL_SerdesStatus)CSL_FEXTR(*(volatile uint32_t *)(uintptr_t)(baseAddr + 0x40c), 31, 31);
    return retval;
}

CSL_SerdesLaneEnableStatus CSL_serdesLaneEnable
(
         CSL_SerdesLaneEnableParams *serdesLaneEnableParams
)
{
    CSL_wiz16b8m4ct2Regs *torrent_sds_reg = (CSL_wiz16b8m4ct2Regs *)(uintptr_t)(serdesLaneEnableParams->baseAddr);
    CSL_SerdesLaneEnableStatus status = CSL_SERDES_LANE_ENABLE_NO_ERR;
    uint32_t laneNum;

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
        if(serdesLaneEnableParams->serdesInstance == CSL_TORRENT_SERDES0)
        {
            if(serdesLaneEnableParams->refClkSrc == CSL_SERDES_REF_CLOCK_INT)
            {
                CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->WIZ_CONFIG.SERDES_RST),30,30,0x0);
                CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->PHY_PCS_CMN_REGISTERS.PHY_ISO_CMN_CTRL),8,8,0x0);
                CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_CDIAG_REFCLK_DRV0_CTRL),15,0,0x0230);
                CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_CDIAG_REFCLK_TEST__CMN_CDIAG_REFCLK_OVRD),15,0,0x0118);
                CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->PHY_PCS_CMN_REGISTERS.PHY_PIPE_CMN_CTRL2__PHY_PIPE_CMN_CTRL1),15,0,0x0401);
                CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->WIZ_CONFIG.SERDES_RST),30,30,0x1);
                CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->PHY_PCS_CMN_REGISTERS.PHY_ISO_CMN_CTRL),8,8,0x1);
            }
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
    CSL_serdesReleaseReset(serdesLaneEnableParams->baseAddr);

    CSL_serdesCycleDelay(100);

	return status;
}

CSL_SerdesPIPEStatus CSL_serdesGetPIPEClkStatus
(
        uint32_t            baseAddr,
        uint8_t             laneMask,
        CSL_SerdesPhyType   phyType
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
	{
        retval = CSL_SERDES_STATUS_PIPE_CLK_NOT_VALID;
	}

    return retval;
}

uint32_t CSL_serdesIsConfigured(uint32_t baseAddr)
{
    return (0u);
}
