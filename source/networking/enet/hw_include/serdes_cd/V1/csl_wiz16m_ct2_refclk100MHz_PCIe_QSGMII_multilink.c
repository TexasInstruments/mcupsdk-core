/**
* File: csl_wiz16m_ct2_refclk100MHz_PCIe_QSGMII_multilink.c
*
*  pll0 = pcie, pll1 = qsgmii
*  lane 0, 1 = pcie
*  lane 2, 3 = qsgmii
*  multilink
*
*� ============================================================================
*� (C) Copyright 2019, Texas Instruments, Inc.
*
*� Redistribution and use in source and binary forms, with or without
*� modification, are permitted provided that the following conditions
*� are met:
*
*��� Redistributions of source code must retain the above copyright
*��� notice, this list of conditions and the following disclaimer.
*
*��� Redistributions in binary form must reproduce the above copyright
*��� notice, this list of conditions and the following disclaimer in the
*��� documentation and/or other materials provided with the
*��� distribution.
*
*��� Neither the name of Texas Instruments Incorporated nor the names of
*��� its contributors may be used to endorse or promote products derived
*��� from this software without specific prior written permission.
*
*� THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*� "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*� LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*� A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
*� OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*� SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
*� LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*� DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*� THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*� (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*� OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/
#ifndef CSL_WIZ16M_CT2_REFCLK100MHz_32b_PCIe_QSGMII_C
#define CSL_WIZ16M_CT2_REFCLK100MHz_32b_PCIe_QSGMII_C

#include <stdint.h>
#include <drivers/hw_include/cslr_soc.h>

#include <serdes_cd/V1/cslr_wiz16b8m4ct2.h>

void csl_wiz16m_ct2_refclk100MHz_PCIe_QSGMII_no_ssc_multilink(uint32_t baseAddr)
{
    CSL_wiz16b8m4ct2Regs *torrent_sds_reg = (CSL_wiz16b8m4ct2Regs *)(uintptr_t)(baseAddr);

    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CLK_SEL_M0__CMN_PDIAG_PLL0_CTRL_M0),31,16,(uint32_t)0x0601);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CLK_SEL_M1__CMN_PDIAG_PLL0_CTRL_M1),31,16,(uint32_t)0x0400);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CLK_SEL_M0__CMN_PDIAG_PLL1_CTRL_M0),31,16,(uint32_t)0x0601);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CP_IADJ_M0__CMN_PDIAG_PLL1_CP_PADJ_M0),15,0,(uint32_t)0x0028);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_DSM_FBH_OVRD_M0__CMN_PLL1_DSM_DIAG_M0),31,16,(uint32_t)0x001E);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_DSM_FBL_OVRD_M0),15,0,(uint32_t)0x000C);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_DIAG_BIAS_OVRD1__CMN_DIAG_BANDGAP_OVRD),31,16,(uint32_t)0x3700);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_TXPUCAL_TUNE__CMN_TXPUCAL_START),31,16,(uint32_t)0x0008);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_TXPDCAL_OVRD__CMN_TXPDCAL_CTRL),31,16,(uint32_t)0x0008);    
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[2].TX_TXCC_CPOST_MULT_01__TX_TXCC_CPOST_MULT_00),15,0,(uint32_t)0x0000);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[3].TX_TXCC_CPOST_MULT_01__TX_TXCC_CPOST_MULT_00),15,0,(uint32_t)0x0000);    
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[2].DRV_DIAG_TX_DRV),15,0,(uint32_t)0x0003);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[3].DRV_DIAG_TX_DRV),15,0,(uint32_t)0x0003);    
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[0].XCVR_DIAG_PLLDRC_CTRL__XCVR_DIAG_XDP_PWRI_STAT),31,16,(uint32_t)0x0012);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[1].XCVR_DIAG_PLLDRC_CTRL__XCVR_DIAG_XDP_PWRI_STAT),31,16,(uint32_t)0x0012);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[2].XCVR_DIAG_PLLDRC_CTRL__XCVR_DIAG_XDP_PWRI_STAT),31,16,(uint32_t)0x009B);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[3].XCVR_DIAG_PLLDRC_CTRL__XCVR_DIAG_XDP_PWRI_STAT),31,16,(uint32_t)0x009B);    
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[0].XCVR_DIAG_HSCLK_DIV__XCVR_DIAG_HSCLK_SEL),15,0,(uint32_t)0x0000);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[0].XCVR_DIAG_HSCLK_DIV__XCVR_DIAG_HSCLK_SEL),31,16,(uint32_t)0x0001);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[1].XCVR_DIAG_HSCLK_DIV__XCVR_DIAG_HSCLK_SEL),15,0,(uint32_t)0x0000);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[1].XCVR_DIAG_HSCLK_DIV__XCVR_DIAG_HSCLK_SEL),31,16,(uint32_t)0x0001);    
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[2].XCVR_DIAG_HSCLK_DIV__XCVR_DIAG_HSCLK_SEL),15,0,(uint32_t)0x0011);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[2].XCVR_DIAG_HSCLK_DIV__XCVR_DIAG_HSCLK_SEL),31,16,(uint32_t)0x0003);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[3].XCVR_DIAG_HSCLK_DIV__XCVR_DIAG_HSCLK_SEL),15,0,(uint32_t)0x0011);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[3].XCVR_DIAG_HSCLK_DIV__XCVR_DIAG_HSCLK_SEL),31,16,(uint32_t)0x0003);    
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[2].TX_PSC_A1__TX_PSC_A0),15,0,(uint32_t)0x00F3);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[2].TX_PSC_A3__TX_PSC_A2),15,0,(uint32_t)0x04A2);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[2].TX_PSC_A3__TX_PSC_A2),31,16,(uint32_t)0x04A2);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[2].RX_PSC_A1__RX_PSC_A0),15,0,(uint32_t)0x091D);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[2].RX_PSC_A3__RX_PSC_A2),15,0,(uint32_t)0x0900);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[2].RX_PSC_A3__RX_PSC_A2),31,16,(uint32_t)0x0100);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[3].TX_PSC_A1__TX_PSC_A0),15,0,(uint32_t)0x00F3);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[3].TX_PSC_A3__TX_PSC_A2),15,0,(uint32_t)0x04A2);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[3].TX_PSC_A3__TX_PSC_A2),31,16,(uint32_t)0x04A2);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[3].RX_PSC_A1__RX_PSC_A0),15,0,(uint32_t)0x091D);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[3].RX_PSC_A3__RX_PSC_A2),15,0,(uint32_t)0x0900);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[3].RX_PSC_A3__RX_PSC_A2),31,16,(uint32_t)0x0100);    
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[2].RX_CDRLF_CNFG2__RX_CDRLF_CNFG),15,0,(uint32_t)0x018C);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[3].RX_CDRLF_CNFG2__RX_CDRLF_CNFG),15,0,(uint32_t)0x018C);    
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[2].RX_REE_GCSM1_EQENM_PH1__RX_REE_GCSM1_CTRL),31,16,(uint32_t)0x03C7);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[2].RX_REE_GCSM1_START_TMR__RX_REE_GCSM1_EQENM_PH2),15,0,(uint32_t)0x01C7);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[3].RX_REE_GCSM1_EQENM_PH1__RX_REE_GCSM1_CTRL),31,16,(uint32_t)0x03C7);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[3].RX_REE_GCSM1_START_TMR__RX_REE_GCSM1_EQENM_PH2),15,0,(uint32_t)0x01C7);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[0].RX_REE_TAP1_CLIP__RX_REE_ADDR_CFG),31,16,(uint32_t)0x0019);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[1].RX_REE_TAP1_CLIP__RX_REE_ADDR_CFG),31,16,(uint32_t)0x0019);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[2].RX_REE_TAP1_CLIP__RX_REE_ADDR_CFG),31,16,(uint32_t)0x0019);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[3].RX_REE_TAP1_CLIP__RX_REE_ADDR_CFG),31,16,(uint32_t)0x0019);    
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[0].RX_REE_CTRL_DATA_MASK__RX_REE_TAP2TON_CLIP),15,0,(uint32_t)0x0019);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[1].RX_REE_CTRL_DATA_MASK__RX_REE_TAP2TON_CLIP),15,0,(uint32_t)0x0019);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[2].RX_REE_CTRL_DATA_MASK__RX_REE_TAP2TON_CLIP),15,0,(uint32_t)0x0019);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[3].RX_REE_CTRL_DATA_MASK__RX_REE_TAP2TON_CLIP),15,0,(uint32_t)0x0019);    
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[2].RX_DIAG_DFE_AMP_TUNE__RX_DIAG_DFE_CTRL),15,0,(uint32_t)0x0000);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[3].RX_DIAG_DFE_AMP_TUNE__RX_DIAG_DFE_CTRL),15,0,(uint32_t)0x0000);    
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[2].RX_DIAG_DFE_AMP_TUNE_3__RX_DIAG_DFE_AMP_TUNE_2),15,0,(uint32_t)0x0C01);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[2].RX_DIAG_DFE_AMP_TUNE_3__RX_DIAG_DFE_AMP_TUNE_2),31,16,(uint32_t)0x0000);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[3].RX_DIAG_DFE_AMP_TUNE_3__RX_DIAG_DFE_AMP_TUNE_2),15,0,(uint32_t)0x0C01);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[3].RX_DIAG_DFE_AMP_TUNE_3__RX_DIAG_DFE_AMP_TUNE_2),31,16,(uint32_t)0x0000);    
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[2].RX_DIAG_NQST_CTRL__RX_DIAG_REE_DAC_CTRL),31,16,(uint32_t)0x0098);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[3].RX_DIAG_NQST_CTRL__RX_DIAG_REE_DAC_CTRL),31,16,(uint32_t)0x0098);    
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[2].RX_DIAG_PI_CAP__RX_DIAG_PI_RATE),31,16,(uint32_t)0x0000);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[2].RX_DIAG_PI_CAP__RX_DIAG_PI_RATE),15,0,(uint32_t)0x0010);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[3].RX_DIAG_PI_CAP__RX_DIAG_PI_RATE),31,16,(uint32_t)0x0000);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[3].RX_DIAG_PI_CAP__RX_DIAG_PI_RATE),15,0,(uint32_t)0x0010);    
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[0].RX_DIAG_ACYA__RX_DIAG_DCYA),31,16,(uint32_t)0x0001);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[1].RX_DIAG_ACYA__RX_DIAG_DCYA),31,16,(uint32_t)0x0001);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[2].RX_DIAG_ACYA__RX_DIAG_DCYA),31,16,(uint32_t)0x0001);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[3].RX_DIAG_ACYA__RX_DIAG_DCYA),31,16,(uint32_t)0x0001);    
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PHY_PCS_CMN_REGISTERS.PHY_AUTO_CFG_SPDUP__PHY_PLL_CFG),15,0,(uint32_t)0x0003);
}

#endif

