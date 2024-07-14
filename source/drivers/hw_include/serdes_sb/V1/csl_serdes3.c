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

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <drivers/hw_include/soc_config.h>
#include <drivers/hw_include/cslr_serdes.h>

void CSL_serdesCycleDelay (uint64_t count)
{
  volatile uint64_t sat = 0;
  for(sat=0;sat<count;sat++);
}

void CSL_serdesDisablePllAndLanes(uint32_t            baseAddr,
                                                uint32_t            numLanes)
{
     uint32_t laneNum;
     CSL_FINSR(*(volatile uint32_t *)(baseAddr +0x1fc0 + 0x34),31,29,(uint32_t)0x04);
     for(laneNum=0;laneNum<numLanes;laneNum++)
     {
         CSL_FINSR(*(volatile uint32_t *)(baseAddr +0x1fc0 + 0x20 + (laneNum*4)),31,29,(uint32_t)0x04);
         CSL_FINSR(*(volatile uint32_t *)(baseAddr +0x1fc0 + 0x20 + (laneNum*4)),15,13,0x04);
     }
}

void CSL_SerdesInvertLanePolarity(uint32_t baseAddr,
                                  uint32_t laneNum)
{
    /* Invert RX Lane Polarity */
    CSL_FINSR(*(volatile uint32_t *)(baseAddr + 0x1fc0 + 0x20 + (laneNum*4)),9,9,1);
    CSL_FINSR(*(volatile uint32_t *)(baseAddr + 0x1fc0 + 0x20 + (laneNum*4)),8,8,1);
}

void CSL_serdesPORResetDefault(uint32_t            baseAddr)
{
     CSL_FINSR(*(volatile uint32_t *)(baseAddr +0x1fc0 + 0x10),29,29,(uint32_t)0x01);
}
void CSL_serdesDisablePLL(uint32_t            baseAddr,
                                        CSL_SerdesPhyType   phyType)
{
     CSL_FINSR(*(volatile uint32_t *)(baseAddr +0x1fc0 + 0x34),31,29,(uint32_t)0x04);
}

void CSL_serdesDisableLanes(uint32_t            baseAddr,
                                         uint32_t            laneNum)
{
     CSL_FINSR(*(volatile uint32_t *)(baseAddr +0x1fc0 + 0x20 + (laneNum*4)),31,29,(uint32_t)0x4);
     CSL_FINSR(*(volatile uint32_t *)(baseAddr +0x1fc0 + 0x20 + (laneNum*4)),15,13,0x4);
}

void CSL_serdesEnableLanes(uint32_t               baseAddr,
                                        uint32_t               laneNum)
{
    CSL_FINSR(*(volatile uint32_t *)(baseAddr +0x1fc0 + 0x20 + (laneNum*4)),31,29,(uint32_t)0x07);
    CSL_FINSR(*(volatile uint32_t *)(baseAddr +0x1fc0 + 0x20 + (laneNum*4)),15,13,0x07);
}

void CSL_serdesWaitForSigDet(uint32_t baseAddr,
		                           uint32_t laneNum)
{
    uint32_t retval = 0;
    while(retval != 1)
    {
        retval = (CSL_SerdesStatus)CSL_FEXTR(*(volatile uint32_t *)
                  (baseAddr + 0x1ff4), (0 + laneNum), (0 + laneNum));
    }
}

void CSL_serdesWaitForCMUOK(uint32_t baseAddr)
{
    uint32_t retval = 0;
    while(retval != 1)
    {
        retval = (CSL_SerdesStatus)CSL_FEXTR(*(volatile uint32_t *)
                  (baseAddr + 0xa00 + 0x194), 19, 19);
    }
}

void CSL_serdesFastSimEnable(uint32_t baseAddr)
{
    CSL_FINSR(*(volatile uint32_t *)(baseAddr +0xa00 + 0x190),16,16,(uint32_t)0x01);
}

void CSL_serdesSetCMUWaitVal(uint32_t baseAddr,
		                           uint32_t val)
{
    CSL_FINSR(*(volatile uint32_t *)(baseAddr +0x1fc0 + 0x3c),16,0,(uint32_t)val);
}

void CSL_serdesSetPhanVal(uint32_t baseAddr)
{
    CSL_FINSR(*(volatile uint32_t *)(baseAddr +0x1fc0 + 0x3c),23,20, 0x1);
}

void CSL_serdesRxEqDisable(uint32_t baseAddr)
{
    CSL_FINSR(*(volatile uint32_t *)(baseAddr +0x1400),7,0, 0x3E);
}

void CSL_serdesPorReset(uint32_t baseAddr)
{
    CSL_FINSR(*(volatile uint32_t *)(baseAddr +0x1fc0 + 0x10),29,29, 0x1);

    CSL_serdesCycleDelay(100000);

    CSL_FINSR(*(volatile uint32_t *)(baseAddr +0x1fc0 + 0x10),29,29, 0x0);
}

void CSL_serdesForceSigDetEn(uint32_t baseAddr, uint32_t laneNum)
{
    CSL_FINSR(*(volatile uint32_t *)(baseAddr + 0x200*(laneNum+1) + 0x13c),17,16,0);
}

void CSL_serdesForceSigDetLow(uint32_t baseAddr, uint32_t laneNum)
{
    CSL_FINSR(*(volatile uint32_t *)(baseAddr + 0x200*(laneNum+1) + 0x13c),17,16,2);
}

void CSL_serdesResetSigDet(uint32_t baseAddr, uint32_t laneNum)
{
    CSL_serdesForceSigDetLow(baseAddr, laneNum);
    CSL_serdesCycleDelay(5000);
    CSL_serdesForceSigDetEn(baseAddr, laneNum);
}

void CSL_serdesForceRxRecalEn(uint32_t baseAddr, uint32_t laneNum)
{
    CSL_FINSR(*(volatile uint32_t *)(baseAddr + 0x1400 + 0x0),6,6,0);
}

void CSL_serdesForceRxRecalLow(uint32_t baseAddr, uint32_t laneNum)
{
    CSL_FINSR(*(volatile uint32_t *)(baseAddr + 0x1400 + 0x0),6,6,1);
}

void CSL_serdesResetRxCal(uint32_t baseAddr, uint32_t laneNum)
{
    CSL_serdesForceRxRecalLow(baseAddr, laneNum);
    CSL_serdesCycleDelay(5000);
    CSL_serdesForceRxRecalEn(baseAddr, laneNum);
}

void CSL_serdesResetReceiver(uint32_t baseAddr, uint32_t laneNum)
{
    CSL_serdesResetSigDet(baseAddr, laneNum);
    CSL_serdesCycleDelay(5000);
    CSL_serdesResetRxCal(baseAddr, laneNum);
}

void CSL_serdesForceSigDetHigh(uint32_t baseAddr, uint32_t laneNum)
{
    CSL_FINSR(*(volatile uint32_t *)(baseAddr + 0x200*(laneNum+1) + 0x13c),17,16,3);
}

void CSL_serdesWriteTbusAddr(uint32_t baseAddr,
                                             int32_t    iSelect,
                                             int32_t    iOffset)
{
    /* For 2 lane macro or 1 lane macro, add 1 to iselect */
    iSelect++;
    CSL_FINSR(*(volatile uint32_t *)(baseAddr+0x084), 10,0,
                                        (uint32_t)((iSelect) << 8) + iOffset); /* Write tbus addr */
}

void CSL_serdesWriteTbusAddrCMU(uint32_t baseAddr,
                                int32_t    iSelect,
                                int32_t    iOffset)
{
    /*CMU0 address is 0x0 */
    CSL_FINSR(*(volatile uint32_t *)(baseAddr+0x084), 10,0,
                                        (uint32_t)((iSelect) << 8) + iOffset); /* Write tbus addr */
}

uint32_t CSL_serdesReadTbusVal(uint32_t baseAddr)
{
    uint32_t iTmp;
    iTmp  = (*(volatile uint32_t *)(baseAddr + 0x80) >> 16) & 0x0fff; /* Read tbusData 11:0 */
    return(iTmp);
}

uint32_t CSL_serdesReadSelectedTbus(uint32_t baseAddr,
                                                    int32_t     iSelect,
                                                    int32_t     iOffset)
{
    CSL_serdesWriteTbusAddr(baseAddr,iSelect,iOffset);
    return(CSL_serdesReadTbusVal(baseAddr));
}

uint32_t CSL_serdesReadSelectedTbusCMU(uint32_t baseAddr,
                                                    int32_t     iSelect,
                                                    int32_t     iOffset)
{
    CSL_serdesWriteTbusAddrCMU(baseAddr,iSelect,iOffset);
    return(CSL_serdesReadTbusVal(baseAddr));
}

void CSL_SerdesConfigCMC1C2(uint32_t            base_addr,
                                                uint32_t            lane_num,
                                                uint32_t            CMcoeff,
                                                uint32_t            C1coeff,
                                                uint32_t            C2coeff,
                                                CSL_SerdesPhyType phy_type)
{
     if(phy_type == CSL_SERDES_PHY_TYPE_PCIe)
     {
         CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0a00 +0xd4),23,20,CMcoeff);
         CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0a00 +0xd4),4,0,C1coeff);
         CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x0a00 +0xd4),11,8,C2coeff);
     }
     if(phy_type == CSL_SERDES_PHY_TYPE_SGMII)
     {
         CSL_FINSR(*(volatile uint32_t *)(base_addr+(lane_num*0x200)+0x200+0x7c),15,12,CMcoeff);
         CSL_FINSR(*(volatile uint32_t *)(base_addr+(lane_num*0x200)+0x200+0x7c),7,3,C1coeff);
         CSL_FINSR(*(volatile uint32_t *)(base_addr+(lane_num*0x200)+0x200+0x7c),11,8,C2coeff);
     }
}


void CSL_serdesTbusLaneDump(uint32_t                 baseAddr,
                        CSL_SerdesTbusDump      *pTbusDump)
{
    uint32_t i,j;
    uint32_t count = 0;

    for(i = 0; i < 1; i++)
    {
        for (j = 0; j <= 220; j++)
        {
            pTbusDump->taddr[count] = j;
            pTbusDump->tbusData[count] = CSL_serdesReadSelectedTbus(baseAddr, i+1, j);
            //printf("TBUS Lane ADDR %d DATA 0x%x\n", pTbusDump->taddr[count], pTbusDump->tbusData[count]);
            count++;
        }
    }

}

void CSL_serdesTbusCMUDump(uint32_t                 baseAddr,
                        CSL_SerdesTbusDump      *pTbusDump)
{
    uint32_t i,j;
    uint32_t count = 0;



    for(i = 0; i < 1; i++)
    {
        for (j = 0; j <= 164; j++)
        {
            pTbusDump->taddr[count] = j;
            pTbusDump->tbusData[count] = CSL_serdesReadSelectedTbusCMU(baseAddr, 0, j);
            //printf("TBUS CMU ADDR %d DATA 0x%x\n", pTbusDump->taddr[count], pTbusDump->tbusData[count]);
            count++;
        }
    }
}

CSL_SerdesLaneEnableStatus CSL_serdesSetLaneRate
(
 uint32_t               baseAddr,
 uint32_t               laneNum,
 CSL_SerdesLaneCtrlRate laneCtrlRate,
 CSL_SerdesLinkRate     linkrate,
 CSL_SerdesPhyType      phyType
)
{
    if (laneCtrlRate == CSL_SERDES_LANE_FULL_RATE)
    {
        /* SGMII should be set to Quarter Rate */
        if(phyType == CSL_SERDES_PHY_TYPE_SGMII)
        {
            return CSL_SERDES_LANE_ENABLE_INVALID_RATE;
        }
        else if(!(phyType == CSL_SERDES_PHY_TYPE_PCIe || phyType == CSL_SERDES_PHY_TYPE_USB))
        {
            /* Set TX0_RATE_VAL and RX0_RATE_VAL to Full Rate */
            CSL_FINSR(*(volatile uint32_t *)(baseAddr + 0x1fe0 + 4*laneNum),28,26, 0x4);
            CSL_FINSR(*(volatile uint32_t *)(baseAddr + 0x1fe0 + 4*laneNum),12,10, 0x4);
        }
    }
    if (laneCtrlRate == CSL_SERDES_LANE_QUARTER_RATE)
    {
        /* DFE, IQN, AIF2 TX should always be set to Full Rate */
        /* Other IPs set TX0_RATE_VAL to Quarter Rate */
        if(!(phyType == CSL_SERDES_PHY_TYPE_DFE || phyType == CSL_SERDES_PHY_TYPE_IQN
	     || phyType == CSL_SERDES_PHY_TYPE_AIF2_B8 || phyType == CSL_SERDES_PHY_TYPE_AIF2_B4))
        {
            CSL_FINSR(*(volatile uint32_t *)(baseAddr + 0x1fe0 + 4*laneNum),28,26, 0x6);
        }
        /* Set RX0_RATE_VAL to Quarter Rate */
        CSL_FINSR(*(volatile uint32_t *)(baseAddr + 0x1fe0 + 4*laneNum),12,10, 0x6);
    }
    if (laneCtrlRate == CSL_SERDES_LANE_HALF_RATE)
    {
        /* DFE, IQN, AIF2 TX should always be set to Full Rate */
        /* Other IPs set TX0_RATE_VAL to Half Rate */
        if(!(phyType == CSL_SERDES_PHY_TYPE_DFE || phyType == CSL_SERDES_PHY_TYPE_IQN
             || phyType == CSL_SERDES_PHY_TYPE_AIF2_B8 || phyType == CSL_SERDES_PHY_TYPE_AIF2_B4))
        {
            CSL_FINSR(*(volatile uint32_t *)(baseAddr + 0x1fe0 + 4*laneNum),28,26, 0x5);
        }
        /* Set RX0_RATE_VAL to Half Rate */
        CSL_FINSR(*(volatile uint32_t *)(baseAddr + 0x1fe0 + 4*laneNum),12,10, 0x5);
    }
    /* Set RX0_ALIGN_VAL to align with comma characters */
    if(phyType == CSL_SERDES_PHY_TYPE_SGMII || phyType == CSL_SERDES_PHY_TYPE_DFE || phyType == CSL_SERDES_PHY_TYPE_SRIO
      || phyType == CSL_SERDES_PHY_TYPE_AIF2_B8 || phyType == CSL_SERDES_PHY_TYPE_AIF2_B4)
    {
        CSL_FINSR(*(volatile uint32_t *)(baseAddr +0x1fc0 + 0x20 + (laneNum*4)),7,6,0x03);
    }
    /* Set TX0_WIDTH_VAL and RX0_WIDTH_VAL to 10 bit mode  for SGMII */
    if(phyType == CSL_SERDES_PHY_TYPE_SGMII)
    {
        CSL_FINSR(*(volatile uint32_t *)(baseAddr +0x1fc0 + 0x20 + (laneNum*4)),23,21,(uint32_t)0x04);
        CSL_FINSR(*(volatile uint32_t *)(baseAddr +0x1fc0 + 0x20 + (laneNum*4)),5,3,0x04);
    }
    /* Set TX0_WIDTH_VAL and RX0_WIDTH_VAL to 40 bit mode  for 10GE */
    else if(phyType == CSL_SERDES_PHY_TYPE_10GE)
    {
        CSL_FINSR(*(volatile uint32_t *)(baseAddr +0x1fc0 + 0x20 + (laneNum*4)),23,21,(uint32_t)0x07);
        CSL_FINSR(*(volatile uint32_t *)(baseAddr +0x1fc0 + 0x20 + (laneNum*4)),5,3,0x07);
    }
    /* For all other IPs, set TX0_WIDTH_VAL and RX0_WIDTH_VAL to 20 bit mode */
    /* TX0_WIDTH_VAL and RX0_WIDTH_VAL are ignored for PCIe and USB PIPE modes */
    else if(!(phyType == CSL_SERDES_PHY_TYPE_PCIe || phyType == CSL_SERDES_PHY_TYPE_USB))
    {
        CSL_FINSR(*(volatile uint32_t *)(baseAddr +0x1fc0 + 0x20 + (laneNum*4)),23,21,(uint32_t)0x06);
        CSL_FINSR(*(volatile uint32_t *)(baseAddr +0x1fc0 + 0x20 + (laneNum*4)),5,3,0x06);
    }
    if(phyType == CSL_SERDES_PHY_TYPE_10GE && linkrate == CSL_SERDES_LINK_RATE_10p3125G)
    {
        CSL_FINSR(*(volatile uint32_t *)(baseAddr + 0x1fe0 + 4*laneNum),23,21, 0x7);
        CSL_FINSR(*(volatile uint32_t *)(baseAddr + 0x1fe0 + 4*laneNum), 5, 3, 0x7);
        CSL_FINSR(*(volatile uint32_t *)(baseAddr + 0x1fe0 + 4*laneNum),16,16, 0x1);
        CSL_FINSR(*(volatile uint32_t *)(baseAddr + 0x1fe0 + 4*laneNum),19,19, 0x1);
    }
    return CSL_SERDES_LANE_ENABLE_NO_ERR;
}

void CSL_serdesSetLoopback
(
 uint32_t baseAddr,
 uint32_t laneNum,
 CSL_SerdesLoopback loopbackMode,
 CSL_SerdesPhyType phyType
)
{
    if (loopbackMode == CSL_SERDES_LOOPBACK_NES)
    {
         CSL_FINSR(*(volatile uint32_t *)(baseAddr + 0x200*(laneNum+1) + 0x54),17,17, (uint32_t)0x1);
    }
    else if (loopbackMode == CSL_SERDES_LOOPBACK_ENCODER)
    {
        /*rx_src_o = 0x1
        ahb_rx_clk_brch1_src_sel_o = 1
        ahb_rx_clk_brch2_src_sel_o = 1
        ahb_rx_clk_brch3_src_sel_o = 3
        ahb_rx_clk_brch4_src_sel_o = 3
        clock_gen_override_o = 1 */

        CSL_FINSR(*(volatile uint32_t *)(baseAddr + 0x200*(laneNum+1) + 0x12c),8,8,1);
        CSL_FINSR(*(volatile uint32_t *)(baseAddr + 0x200*(laneNum+1) + 0x0),10,8,1);
        CSL_FINSR(*(volatile uint32_t *)(baseAddr + 0x200*(laneNum+1) + 0x0),14,12,1);
        CSL_FINSR(*(volatile uint32_t *)(baseAddr + 0x200*(laneNum+1) + 0x0),18,16,3);
        CSL_FINSR(*(volatile uint32_t *)(baseAddr + 0x200*(laneNum+1) + 0x0),22,20,3);
        CSL_FINSR(*(volatile uint32_t *)(baseAddr + 0x200*(laneNum+1) + 0x3c),24,24,1);
    }
}

void CSL_serdesPllEnable
(
 uint32_t               baseAddr,
 CSL_SerdesPhyType      phyType,
 CSL_SerdesLinkRate     link_rate
)
{
    if(phyType == CSL_SERDES_PHY_TYPE_10GE)
    {
        CSL_FINSR(*(volatile uint32_t *)(baseAddr + 0x0a00), 7, 0, (uint32_t)0x1f);
    }
    CSL_FINSR(*(volatile uint32_t *)(baseAddr + 0x1ff4),31,29, (uint32_t)0x7);
    if(phyType== CSL_SERDES_PHY_TYPE_10GE)
    {
        if (link_rate == CSL_SERDES_LINK_RATE_10p3125G)
            CSL_FINSR(*(volatile uint32_t *)(baseAddr + 0x1ff4), 27, 25, 0x7 );
    }
}

CSL_SerdesStatus CSL_serdesGetPLLStatus
(
 uint32_t baseAddr,
 CSL_SerdesPhyType phyType
)
{
    CSL_SerdesStatus retval;
    retval = (CSL_SerdesStatus)CSL_FEXTR(*(volatile uint32_t *)(baseAddr + 0x1ff4), 28, 28);
    return retval;
}

void CSL_serdesDisablePllLanes(uint32_t            baseAddr,
                               uint32_t            numLanes)
{
     uint32_t laneNum;

     /* Disable PLL */
     CSL_FINSR(*(volatile uint32_t *)(baseAddr +0x1fc0 + 0x34),31,29,(uint32_t)0x04);
     /* Disable Lanes */
     for(laneNum=0;laneNum<numLanes;laneNum++)
     {
         CSL_FINSR(*(volatile uint32_t *)(baseAddr +0x1fc0 + 0x20 + (laneNum*4)),31,29,(uint32_t)0x04);
         CSL_FINSR(*(volatile uint32_t *)(baseAddr +0x1fc0 + 0x20 + (laneNum*4)),15,13,0x04);
     }
}



CSL_SerdesStatus CSL_serdesGetSigDetStatus
(
 uint32_t baseAddr,
 uint32_t numLanes,
 uint8_t laneMask,
 CSL_SerdesPhyType phyType
)
{
    uint32_t laneNum;
    CSL_SerdesStatus retval = CSL_SERDES_STATUS_PLL_LOCKED;
    for (laneNum=0; laneNum < numLanes; laneNum++)
    {
        if ((laneMask & (1<<laneNum))!=0)
        {
            retval = (CSL_SerdesStatus)(retval & CSL_FEXTR(*(volatile uint32_t *)(baseAddr + 0x1ff4), (0 + laneNum), (0 + laneNum)));
        }
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

    for (laneNum=0; laneNum < numLanes; laneNum++)
    {
        if ((laneMask & (1<<laneNum))!=0)
        {
            /* Poll for Lane OK State */
            if(!(phyType == CSL_SERDES_PHY_TYPE_PCIe || phyType == CSL_SERDES_PHY_TYPE_USB))
            {
                retval = (CSL_SerdesStatus)(retval & CSL_FEXTR(*(volatile uint32_t *)(baseAddr + 0x1ff4), (8 + laneNum), (8 + laneNum)));
            }
            /* Poll for Signal Detect State for PCIe and USB PIPE mode */
            else
            {
                retval = (CSL_SerdesStatus)(retval & CSL_FEXTR(*(volatile uint32_t *)(baseAddr + 0x1ff4), (0 + laneNum), (0 + laneNum)));
            }
        }
    }
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

    /* Set RX0_RATE_VAL and TX0_RATE_VAL in the LANExCTL_STS register */
    if (!(serdesLaneEnableParams->operatingMode == CSL_SERDES_DIAGNOSTIC_MODE))
    {
        for(laneNum=0;laneNum<CSL_SERDES_MAX_LANES;laneNum++)
        {
            if ((serdesLaneEnableParams->laneMask & (1<<laneNum))!=0)
            {
                status = (CSL_SerdesLaneEnableStatus)(status | CSL_serdesSetLaneRate(serdesLaneEnableParams->baseAddr, laneNum,
                                      serdesLaneEnableParams->laneCtrlRate[laneNum],
                                      serdesLaneEnableParams->linkRate,
                                      serdesLaneEnableParams->phyType));
            }
        }
    }

    /* Set NES bit for functional mode. Diagnostic mode will set NES loopback in serdes diag code */
    if(serdesLaneEnableParams->operatingMode != CSL_SERDES_DIAGNOSTIC_MODE)
    {
        for(laneNum=0;laneNum<CSL_SERDES_MAX_LANES;laneNum++)
        {
            if ((serdesLaneEnableParams->laneMask & (1<<laneNum))!=0)
            {
                CSL_serdesSetLoopback(serdesLaneEnableParams->baseAddr, laneNum, serdesLaneEnableParams->loopbackMode[laneNum], serdesLaneEnableParams->phyType);
            }
        }
    }

    /*Force att and boost if flag is set */
    for(laneNum=0;laneNum<CSL_SERDES_MAX_LANES;laneNum++)
    {
        if ((serdesLaneEnableParams->laneMask & (1<<laneNum))!=0)
        {
            if (serdesLaneEnableParams->forceAttBoost)
            {
                if(serdesLaneEnableParams->phyType == CSL_SERDES_PHY_TYPE_PCIe)
                {
                    CSL_FINSR(*(volatile uint32_t *)(CSL_serdesComRXEQMap(serdesLaneEnableParams->baseAddr, 0xC)), 23,16,0x4C); /* calibration gen3 default FF*/
                    CSL_FINSR(*(volatile uint32_t *)(CSL_serdesComRXEQMap(serdesLaneEnableParams->baseAddr, 0x34)), 18,16,(uint32_t)serdesLaneEnableParams->rxCoeff.forceAttVal[laneNum]); /* att Gen 3 default 2*/ //0
                    CSL_FINSR(*(volatile uint32_t *)(CSL_serdesComRXEQMap(serdesLaneEnableParams->baseAddr, 0x4c)), 3,0,(uint32_t)serdesLaneEnableParams->rxCoeff.forceBoostVal[laneNum]); /* Gen 3 boost start default C*/
                    CSL_FINSR(*(volatile uint32_t *)(CSL_serdesComRXEQMap(serdesLaneEnableParams->baseAddr, 0x8)), 7,0,0x4C); /* csr_rxeq_init_run_rate3_o*/
                }
                else if(serdesLaneEnableParams->phyType == CSL_SERDES_PHY_TYPE_USB)
                {
                    CSL_FINSR(*(volatile uint32_t *)(CSL_serdesComRXEQMap(serdesLaneEnableParams->baseAddr, 0xC)), 7,0,0xC); /* calibration Rate1 default FF*/
                    CSL_FINSR(*(volatile uint32_t *)(CSL_serdesComRXEQMap(serdesLaneEnableParams->baseAddr, 0x34)), 2,0,(uint32_t)serdesLaneEnableParams->rxCoeff.forceAttVal[laneNum]); /* att Rate1 default 2*/ //0
                    CSL_FINSR(*(volatile uint32_t *)(CSL_serdesComRXEQMap(serdesLaneEnableParams->baseAddr, 0x44)), 3,0,(uint32_t)serdesLaneEnableParams->rxCoeff.forceBoostVal[laneNum]); /* Rate1 boost start default C*/
                    CSL_FINSR(*(volatile uint32_t *)(CSL_serdesComRXEQMap(serdesLaneEnableParams->baseAddr, 0x4)), 23,16,0xC); /* csr_rxeq_init_run_rate1_o*/

                    CSL_FINSR(*(volatile uint32_t *)(CSL_serdesComRXEQMap(serdesLaneEnableParams->baseAddr, 0xC)), 15,8,0xC); /* calibration Rate2 default FF*/
                    CSL_FINSR(*(volatile uint32_t *)(CSL_serdesComRXEQMap(serdesLaneEnableParams->baseAddr, 0x34)), 10,8,(uint32_t)serdesLaneEnableParams->rxCoeff.forceAttVal[laneNum]); /* att Rate2 default 2*/ //0
                    CSL_FINSR(*(volatile uint32_t *)(CSL_serdesComRXEQMap(serdesLaneEnableParams->baseAddr, 0x48)), 3,0,(uint32_t)serdesLaneEnableParams->rxCoeff.forceBoostVal[laneNum]); /* Rate2 boost start default C*/
                    CSL_FINSR(*(volatile uint32_t *)(CSL_serdesComRXEQMap(serdesLaneEnableParams->baseAddr, 0x4)), 31,24,0xC); /* csr_rxeq_init_run_rate2_o*/
                }
            }
        }
    }

    /* Enable PLL by setting the PLL_ENABLE_VAL in PLL_CTRL register */
    CSL_serdesPllEnable(serdesLaneEnableParams->baseAddr,serdesLaneEnableParams->phyType,serdesLaneEnableParams->linkRate);

    CSL_serdesWaitForCMUOK(serdesLaneEnableParams->baseAddr);

    CSL_serdesCycleDelay(100);

	/* Poll for PLL_OK */
    /* Do not poll for FAST SIM or PIPE mode */
    if (!(serdesLaneEnableParams->operatingMode == CSL_SERDES_FUNCTIONAL_MODE_FAST_SIM ||
            serdesLaneEnableParams->operatingMode == CSL_SERDES_DIAGNOSTIC_MODE ||
			serdesLaneEnableParams->phyType == CSL_SERDES_PHY_TYPE_PCIe))
	{
		do
		{
			pllstat = CSL_serdesGetPLLStatus(serdesLaneEnableParams->baseAddr,
											 serdesLaneEnableParams->phyType);
		}while(pllstat == CSL_SERDES_STATUS_PLL_NOT_LOCKED);
	}

	/* Enable lanes by setting TX0_ENABLE_VAL and TX0_ENABLE_VAL in LANExCTL_STS register */
	/* Do not enable Lanes yet for Diagnostic BER mode */
    if (!(serdesLaneEnableParams->operatingMode == CSL_SERDES_DIAGNOSTIC_MODE))
    {
        for(laneNum=0;laneNum<CSL_SERDES_MAX_LANES;laneNum++)
        {
            if ((serdesLaneEnableParams->laneMask & (1<<laneNum))!=0)
            {
                CSL_serdesEnableLanes(serdesLaneEnableParams->baseAddr, laneNum);
            }
        }
    }

	/* Poll for LANE OK */
    /* Do not poll for FAST SIM or PIPE mode */
	if (!(serdesLaneEnableParams->operatingMode == CSL_SERDES_FUNCTIONAL_MODE_FAST_SIM ||
	        serdesLaneEnableParams->operatingMode == CSL_SERDES_DIAGNOSTIC_MODE ||
			serdesLaneEnableParams->phyType == CSL_SERDES_PHY_TYPE_PCIe))
	{
		do
		{
			pllstat = CSL_serdesGetLaneStatus(serdesLaneEnableParams->baseAddr,
										  CSL_SERDES_MAX_LANES,
										  serdesLaneEnableParams->laneMask,
										  serdesLaneEnableParams->phyType);
		}while(pllstat == CSL_SERDES_STATUS_PLL_NOT_LOCKED);
	}

	CSL_serdesCycleDelay(100);

	return status;
}
