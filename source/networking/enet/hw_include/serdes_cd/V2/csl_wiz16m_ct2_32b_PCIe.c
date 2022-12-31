/**
* File: csl_wiz16m_ct2_32b_PCIe.c
*
*  using_plllc
*  no multilink
*
*  ============================================================================
*  (C) Copyright 2019, Texas Instruments, Inc.
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

#ifndef CSL_WIZ16M_CT2_32b_PCIe_C
#define CSL_WIZ16M_CT2_32b_PCIe_C

#include <stdint.h>
#include <drivers/hw_include/cslr_soc.h>

#include <serdes_cd/V1/cslr_wiz16b8m4ct2.h>



void csl_wiz16m_ct2_refclk100MHz_32b_PCIe_no_ssc(uint32_t baseAddr, uint32_t laneNum)
{
    CSL_wiz16b8m4ct2Regs *torrent_sds_reg = (CSL_wiz16b8m4ct2Regs *)(uintptr_t)(baseAddr);

    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_OVRD__CMN_PLL0_VCOCAL_TCTRL),15,0,(uint32_t)0x0003);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_OVRD__CMN_PLL1_VCOCAL_TCTRL),15,0,(uint32_t)0x0003);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_REE_TAP1_CLIP__RX_REE_ADDR_CFG),31,16,(uint32_t)0x0019);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_REE_CTRL_DATA_MASK__RX_REE_TAP2TON_CLIP),15,0,(uint32_t)0x0019);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_DIAG_ACYA__RX_DIAG_DCYA),31,16,(uint32_t)0x0001);

}

void csl_wiz16m_ct2_refclk100MHz_32b_PCIe_int_ssc(uint32_t baseAddr, uint32_t laneNum)
{
    CSL_wiz16b8m4ct2Regs *torrent_sds_reg = (CSL_wiz16b8m4ct2Regs *)(uintptr_t)(baseAddr);

    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_OVRD__CMN_PLL0_VCOCAL_TCTRL),15,0,(uint32_t)0x0003);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_REFTIM_START),15,0,(uint32_t)0x0C5E);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_PLLCNT_START),15,0,(uint32_t)0x0C56);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_FRACDIVL_M0__CMN_PLL0_INTDIV_M0),15,0,(uint32_t)0x0064);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_HIGH_THR_M0__CMN_PLL0_FRACDIVH_M0),15,0,(uint32_t)0x0002);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_HIGH_THR_M0__CMN_PLL0_FRACDIVH_M0),31,16,(uint32_t)0x0044);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_DSM_FBH_OVRD_M0__CMN_PLL0_DSM_DIAG_M0),15,0,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_SS_CTRL2_M0__CMN_PLL0_SS_CTRL1_M0),15,0,(uint32_t)0x0001);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_SS_CTRL2_M0__CMN_PLL0_SS_CTRL1_M0),31,16,(uint32_t)0x011B);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_SS_CTRL4_M0__CMN_PLL0_SS_CTRL3_M0),15,0,(uint32_t)0x006E);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_SS_CTRL4_M0__CMN_PLL0_SS_CTRL3_M0),31,16,(uint32_t)0x000E);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_LOCK_REFCNT_IDLE__CMN_PLL0_LOCK_REFCNT_START),15,0,(uint32_t)0x00C7);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_LOCK_PLLCNT_THR__CMN_PLL0_LOCK_PLLCNT_START),15,0,(uint32_t)0x00C7);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_LOCK_PLLCNT_THR__CMN_PLL0_LOCK_PLLCNT_START),31,16,(uint32_t)0x0005);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_FRACDIVL_M1__CMN_PLL0_INTDIV_M1),15,0,(uint32_t)0x0050);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_HIGH_THR_M1__CMN_PLL0_FRACDIVH_M1),15,0,(uint32_t)0x0002);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_HIGH_THR_M1__CMN_PLL0_FRACDIVH_M1),31,16,(uint32_t)0x0036);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_DSM_FBH_OVRD_M1__CMN_PLL0_DSM_DIAG_M1),15,0,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_SS_CTRL2_M1__CMN_PLL0_SS_CTRL1_M1),15,0,(uint32_t)0x0001);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_SS_CTRL2_M1__CMN_PLL0_SS_CTRL1_M1),31,16,(uint32_t)0x011B);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_SS_CTRL4_M1__CMN_PLL0_SS_CTRL3_M1),15,0,(uint32_t)0x0058);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_SS_CTRL4_M1__CMN_PLL0_SS_CTRL3_M1),31,16,(uint32_t)0x0012);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_OVRD__CMN_PLL1_VCOCAL_TCTRL),15,0,(uint32_t)0x0003);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_REFTIM_START),15,0,(uint32_t)0x0C5E);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_PLLCNT_START),15,0,(uint32_t)0x0C56);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_FRACDIVL_M0__CMN_PLL1_INTDIV_M0),15,0,(uint32_t)0x0050);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_HIGH_THR_M0__CMN_PLL1_FRACDIVH_M0),15,0,(uint32_t)0x0002);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_HIGH_THR_M0__CMN_PLL1_FRACDIVH_M0),31,16,(uint32_t)0x0036);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_DSM_FBH_OVRD_M0__CMN_PLL1_DSM_DIAG_M0),15,0,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_SS_CTRL2_M0__CMN_PLL1_SS_CTRL1_M0),15,0,(uint32_t)0x0001);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_SS_CTRL2_M0__CMN_PLL1_SS_CTRL1_M0),31,16,(uint32_t)0x011B);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_SS_CTRL4_M0__CMN_PLL1_SS_CTRL3_M0),15,0,(uint32_t)0x0058);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_SS_CTRL4_M0__CMN_PLL1_SS_CTRL3_M0),31,16,(uint32_t)0x0012);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_LOCK_REFCNT_IDLE__CMN_PLL1_LOCK_REFCNT_START),15,0,(uint32_t)0x00C7);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_LOCK_PLLCNT_THR__CMN_PLL1_LOCK_PLLCNT_START),15,0,(uint32_t)0x00C7);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_LOCK_PLLCNT_THR__CMN_PLL1_LOCK_PLLCNT_START),31,16,(uint32_t)0x0005);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CLK_SEL_M0__CMN_PDIAG_PLL0_CTRL_M0),15,0,(uint32_t)0x0002);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_IADJ_M0__CMN_PDIAG_PLL0_CP_PADJ_M0),15,0,(uint32_t)0x0509);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_IADJ_M0__CMN_PDIAG_PLL0_CP_PADJ_M0),31,16,(uint32_t)0x0F00);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_TUNE_M0__CMN_PDIAG_PLL0_FILT_PADJ_M0),15,0,(uint32_t)0x0F08);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CLK_SEL_M1__CMN_PDIAG_PLL0_CTRL_M1),15,0,(uint32_t)0x0002);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_IADJ_M1__CMN_PDIAG_PLL0_CP_PADJ_M1),15,0,(uint32_t)0x0509);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_IADJ_M1__CMN_PDIAG_PLL0_CP_PADJ_M1),31,16,(uint32_t)0x0F00);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_TUNE_M1__CMN_PDIAG_PLL0_FILT_PADJ_M1),15,0,(uint32_t)0x0F08);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CLK_SEL_M0__CMN_PDIAG_PLL1_CTRL_M0),15,0,(uint32_t)0x0002);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CP_IADJ_M0__CMN_PDIAG_PLL1_CP_PADJ_M0),15,0,(uint32_t)0x0509);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CP_IADJ_M0__CMN_PDIAG_PLL1_CP_PADJ_M0),31,16,(uint32_t)0x0F00);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CP_TUNE_M0__CMN_PDIAG_PLL1_FILT_PADJ_M0),15,0,(uint32_t)0x0F08);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_REE_TAP1_CLIP__RX_REE_ADDR_CFG),31,16,(uint32_t)0x0019);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_REE_CTRL_DATA_MASK__RX_REE_TAP2TON_CLIP),15,0,(uint32_t)0x0019);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_DIAG_ACYA__RX_DIAG_DCYA),31,16,(uint32_t)0x0001);

}

void csl_wiz16m_ct2_refclk100MHz_32b_PCIe_ext_ssc(uint32_t baseAddr, uint32_t laneNum)
{
    CSL_wiz16b8m4ct2Regs *torrent_sds_reg = (CSL_wiz16b8m4ct2Regs *)(uintptr_t)(baseAddr);

    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_OVRD__CMN_PLL0_VCOCAL_TCTRL),15,0,(uint32_t)0x0003);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_OVRD__CMN_PLL1_VCOCAL_TCTRL),15,0,(uint32_t)0x0003);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_REE_TAP1_CLIP__RX_REE_ADDR_CFG),31,16,(uint32_t)0x0019);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_REE_CTRL_DATA_MASK__RX_REE_TAP2TON_CLIP),15,0,(uint32_t)0x0019);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_DIAG_ACYA__RX_DIAG_DCYA),31,16,(uint32_t)0x0001);

}

void csl_wiz16m_ct2_refclk19p2MHz_32b_PCIe_no_ssc(uint32_t baseAddr, uint32_t laneNum)
{
    CSL_wiz16b8m4ct2Regs *torrent_sds_reg = (CSL_wiz16b8m4ct2Regs *)(uintptr_t)(baseAddr);

    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_SSM_BIAS_TMR),15,0,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLLSM0_PLLVREF_TMR__CMN_PLLSM0_PLLPRE_TMR),15,0,(uint32_t)0x0027);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLLSM0_PLLCLKDIS_TMR__CMN_PLLSM0_PLLLOCK_TMR),15,0,(uint32_t)0x00A1);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLLSM1_PLLVREF_TMR__CMN_PLLSM1_PLLPRE_TMR),15,0,(uint32_t)0x0027);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLLSM1_PLLCLKDIS_TMR__CMN_PLLSM1_PLLLOCK_TMR),15,0,(uint32_t)0x00A1);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_BGCAL_ITER_TMR__CMN_BGCAL_INIT_TMR),15,0,(uint32_t)0x0060);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_BGCAL_ITER_TMR__CMN_BGCAL_INIT_TMR),31,16,(uint32_t)0x0060);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_IBCAL_ITER_TMR__CMN_IBCAL_INIT_TMR),15,0,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_OVRD__CMN_PLL0_VCOCAL_TCTRL),15,0,(uint32_t)0x0003);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_ITER_TMR__CMN_PLL0_VCOCAL_INIT_TMR),15,0,(uint32_t)0x00C0);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_ITER_TMR__CMN_PLL0_VCOCAL_INIT_TMR),31,16,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_REFTIM_START),15,0,(uint32_t)0x0261);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_PLLCNT_START),15,0,(uint32_t)0x3DF3);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_FRACDIVL_M0__CMN_PLL0_INTDIV_M0),15,0,(uint32_t)0x0104);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_FRACDIVL_M0__CMN_PLL0_INTDIV_M0),31,16,(uint32_t)0x6AAB);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_HIGH_THR_M0__CMN_PLL0_FRACDIVH_M0),15,0,(uint32_t)0x0002);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_HIGH_THR_M0__CMN_PLL0_FRACDIVH_M0),31,16,(uint32_t)0x00AE);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_DSM_FBH_OVRD_M0__CMN_PLL0_DSM_DIAG_M0),15,0,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_LOCK_REFCNT_IDLE__CMN_PLL0_LOCK_REFCNT_START),15,0,(uint32_t)0x0010);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_LOCK_PLLCNT_THR__CMN_PLL0_LOCK_PLLCNT_START),15,0,(uint32_t)0x01A1);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_LOCK_PLLCNT_THR__CMN_PLL0_LOCK_PLLCNT_START),31,16,(uint32_t)0x0005);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_FRACDIVL_M1__CMN_PLL0_INTDIV_M1),15,0,(uint32_t)0x01A0);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_FRACDIVL_M1__CMN_PLL0_INTDIV_M1),31,16,(uint32_t)0xAAAB);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_HIGH_THR_M1__CMN_PLL0_FRACDIVH_M1),15,0,(uint32_t)0x0002);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_HIGH_THR_M1__CMN_PLL0_FRACDIVH_M1),31,16,(uint32_t)0x0116);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_DSM_FBH_OVRD_M1__CMN_PLL0_DSM_DIAG_M1),15,0,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_OVRD__CMN_PLL1_VCOCAL_TCTRL),15,0,(uint32_t)0x0003);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_ITER_TMR__CMN_PLL1_VCOCAL_INIT_TMR),15,0,(uint32_t)0x00C0);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_ITER_TMR__CMN_PLL1_VCOCAL_INIT_TMR),31,16,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_REFTIM_START),15,0,(uint32_t)0x0261);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_PLLCNT_START),15,0,(uint32_t)0x3DF3);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_FRACDIVL_M0__CMN_PLL1_INTDIV_M0),15,0,(uint32_t)0x01A0);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_FRACDIVL_M0__CMN_PLL1_INTDIV_M0),31,16,(uint32_t)0xAAAB);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_HIGH_THR_M0__CMN_PLL1_FRACDIVH_M0),15,0,(uint32_t)0x0002);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_HIGH_THR_M0__CMN_PLL1_FRACDIVH_M0),31,16,(uint32_t)0x0116);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_DSM_FBH_OVRD_M0__CMN_PLL1_DSM_DIAG_M0),15,0,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_LOCK_REFCNT_IDLE__CMN_PLL1_LOCK_REFCNT_START),15,0,(uint32_t)0x0010);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_LOCK_PLLCNT_THR__CMN_PLL1_LOCK_PLLCNT_START),15,0,(uint32_t)0x01A1);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_LOCK_PLLCNT_THR__CMN_PLL1_LOCK_PLLCNT_START),31,16,(uint32_t)0x0005);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_TXPUCAL_ITER_TMR__CMN_TXPUCAL_INIT_TMR),15,0,(uint32_t)0x0018);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_TXPUCAL_ITER_TMR__CMN_TXPUCAL_INIT_TMR),31,16,(uint32_t)0x0005);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_TXPDCAL_ITER_TMR__CMN_TXPDCAL_INIT_TMR),15,0,(uint32_t)0x0018);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_TXPDCAL_ITER_TMR__CMN_TXPDCAL_INIT_TMR),31,16,(uint32_t)0x0005);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_RXCAL_ITER_TMR__CMN_RXCAL_INIT_TMR),15,0,(uint32_t)0x0240);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_RXCAL_ITER_TMR__CMN_RXCAL_INIT_TMR),31,16,(uint32_t)0x0005);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_SD_CAL_REFTIM_START),15,0,(uint32_t)0x000B);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_SD_CAL_PLLCNT_START),15,0,(uint32_t)0x0137);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CLK_SEL_M0__CMN_PDIAG_PLL0_CTRL_M0),15,0,(uint32_t)0x0032);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_IADJ_M0__CMN_PDIAG_PLL0_CP_PADJ_M0),15,0,(uint32_t)0x0509);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_IADJ_M0__CMN_PDIAG_PLL0_CP_PADJ_M0),31,16,(uint32_t)0x0F00);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_TUNE_M0__CMN_PDIAG_PLL0_FILT_PADJ_M0),15,0,(uint32_t)0x0F08);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CLK_SEL_M1__CMN_PDIAG_PLL0_CTRL_M1),15,0,(uint32_t)0x0022);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_IADJ_M1__CMN_PDIAG_PLL0_CP_PADJ_M1),15,0,(uint32_t)0x0509);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_IADJ_M1__CMN_PDIAG_PLL0_CP_PADJ_M1),31,16,(uint32_t)0x0F00);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_TUNE_M1__CMN_PDIAG_PLL0_FILT_PADJ_M1),15,0,(uint32_t)0x0F08);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CLK_SEL_M0__CMN_PDIAG_PLL1_CTRL_M0),15,0,(uint32_t)0x0022);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CP_IADJ_M0__CMN_PDIAG_PLL1_CP_PADJ_M0),15,0,(uint32_t)0x0509);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CP_IADJ_M0__CMN_PDIAG_PLL1_CP_PADJ_M0),31,16,(uint32_t)0x0F00);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CP_TUNE_M0__CMN_PDIAG_PLL1_FILT_PADJ_M0),15,0,(uint32_t)0x0F08);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[laneNum].DRV_DIAG_LANE_FCM_EN_SWAIT_TMR__DRV_DIAG_LANE_FCM_EN_TO),15,0,(uint32_t)0x0180);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[laneNum].DRV_DIAG_LANE_FCM_EN_TUNE__DRV_DIAG_LANE_FCM_EN_MGN_TMR),15,0,(uint32_t)0x0074);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[laneNum].TX_RCVDET_ST_TMR__TX_RCVDET_EN_TMR),31,16,(uint32_t)0x0780);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_SDCAL0_ITER_TMR__RX_SDCAL0_INIT_TMR),15,0,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_SDCAL0_ITER_TMR__RX_SDCAL0_INIT_TMR),31,16,(uint32_t)0x0060);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_SDCAL1_ITER_TMR__RX_SDCAL1_INIT_TMR),15,0,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_SDCAL1_ITER_TMR__RX_SDCAL1_INIT_TMR),31,16,(uint32_t)0x0060);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_REE_TAP1_CLIP__RX_REE_ADDR_CFG),31,16,(uint32_t)0x0019);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_REE_CTRL_DATA_MASK__RX_REE_TAP2TON_CLIP),15,0,(uint32_t)0x0019);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_DIAG_ACYA__RX_DIAG_DCYA),31,16,(uint32_t)0x0001);

}

void csl_wiz16m_ct2_refclk19p2MHz_32b_PCIe_int_ssc(uint32_t baseAddr, uint32_t laneNum)
{
    CSL_wiz16b8m4ct2Regs *torrent_sds_reg = (CSL_wiz16b8m4ct2Regs *)(uintptr_t)(baseAddr);

    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_SSM_BIAS_TMR),15,0,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLLSM0_PLLVREF_TMR__CMN_PLLSM0_PLLPRE_TMR),15,0,(uint32_t)0x0027);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLLSM0_PLLCLKDIS_TMR__CMN_PLLSM0_PLLLOCK_TMR),15,0,(uint32_t)0x00A1);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLLSM1_PLLVREF_TMR__CMN_PLLSM1_PLLPRE_TMR),15,0,(uint32_t)0x0027);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLLSM1_PLLCLKDIS_TMR__CMN_PLLSM1_PLLLOCK_TMR),15,0,(uint32_t)0x00A1);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_BGCAL_ITER_TMR__CMN_BGCAL_INIT_TMR),15,0,(uint32_t)0x0060);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_BGCAL_ITER_TMR__CMN_BGCAL_INIT_TMR),31,16,(uint32_t)0x0060);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_IBCAL_ITER_TMR__CMN_IBCAL_INIT_TMR),15,0,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_OVRD__CMN_PLL0_VCOCAL_TCTRL),15,0,(uint32_t)0x0003);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_ITER_TMR__CMN_PLL0_VCOCAL_INIT_TMR),15,0,(uint32_t)0x00C0);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_ITER_TMR__CMN_PLL0_VCOCAL_INIT_TMR),31,16,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_REFTIM_START),15,0,(uint32_t)0x0260);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_PLLCNT_START),15,0,(uint32_t)0x3DCB);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_FRACDIVL_M0__CMN_PLL0_INTDIV_M0),15,0,(uint32_t)0x0104);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_FRACDIVL_M0__CMN_PLL0_INTDIV_M0),31,16,(uint32_t)0x6AAB);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_HIGH_THR_M0__CMN_PLL0_FRACDIVH_M0),15,0,(uint32_t)0x0002);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_HIGH_THR_M0__CMN_PLL0_FRACDIVH_M0),31,16,(uint32_t)0x00AE);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_DSM_FBH_OVRD_M0__CMN_PLL0_DSM_DIAG_M0),15,0,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_SS_CTRL2_M0__CMN_PLL0_SS_CTRL1_M0),15,0,(uint32_t)0x0001);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_SS_CTRL2_M0__CMN_PLL0_SS_CTRL1_M0),31,16,(uint32_t)0x0304);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_SS_CTRL4_M0__CMN_PLL0_SS_CTRL3_M0),15,0,(uint32_t)0x0069);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_SS_CTRL4_M0__CMN_PLL0_SS_CTRL3_M0),31,16,(uint32_t)0x0003);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_LOCK_REFCNT_IDLE__CMN_PLL0_LOCK_REFCNT_START),15,0,(uint32_t)0x000F);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_LOCK_PLLCNT_THR__CMN_PLL0_LOCK_PLLCNT_START),15,0,(uint32_t)0x019F);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_LOCK_PLLCNT_THR__CMN_PLL0_LOCK_PLLCNT_START),31,16,(uint32_t)0x0006);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_FRACDIVL_M1__CMN_PLL0_INTDIV_M1),15,0,(uint32_t)0x01A0);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_FRACDIVL_M1__CMN_PLL0_INTDIV_M1),31,16,(uint32_t)0xAAAB);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_HIGH_THR_M1__CMN_PLL0_FRACDIVH_M1),15,0,(uint32_t)0x0002);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_HIGH_THR_M1__CMN_PLL0_FRACDIVH_M1),31,16,(uint32_t)0x0116);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_DSM_FBH_OVRD_M1__CMN_PLL0_DSM_DIAG_M1),15,0,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_SS_CTRL2_M1__CMN_PLL0_SS_CTRL1_M1),15,0,(uint32_t)0x0001);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_SS_CTRL2_M1__CMN_PLL0_SS_CTRL1_M1),31,16,(uint32_t)0x0511);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_SS_CTRL4_M1__CMN_PLL0_SS_CTRL3_M1),15,0,(uint32_t)0x0064);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_SS_CTRL4_M1__CMN_PLL0_SS_CTRL3_M1),31,16,(uint32_t)0x0003);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_OVRD__CMN_PLL1_VCOCAL_TCTRL),15,0,(uint32_t)0x0003);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_ITER_TMR__CMN_PLL1_VCOCAL_INIT_TMR),15,0,(uint32_t)0x00C0);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_ITER_TMR__CMN_PLL1_VCOCAL_INIT_TMR),31,16,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_REFTIM_START),15,0,(uint32_t)0x0260);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_PLLCNT_START),15,0,(uint32_t)0x3DCB);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_FRACDIVL_M0__CMN_PLL1_INTDIV_M0),15,0,(uint32_t)0x01A0);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_FRACDIVL_M0__CMN_PLL1_INTDIV_M0),31,16,(uint32_t)0xAAAB);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_HIGH_THR_M0__CMN_PLL1_FRACDIVH_M0),15,0,(uint32_t)0x0002);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_HIGH_THR_M0__CMN_PLL1_FRACDIVH_M0),31,16,(uint32_t)0x0116);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_DSM_FBH_OVRD_M0__CMN_PLL1_DSM_DIAG_M0),15,0,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_SS_CTRL2_M0__CMN_PLL1_SS_CTRL1_M0),15,0,(uint32_t)0x0001);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_SS_CTRL2_M0__CMN_PLL1_SS_CTRL1_M0),31,16,(uint32_t)0x0511);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_SS_CTRL4_M0__CMN_PLL1_SS_CTRL3_M0),15,0,(uint32_t)0x0064);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_SS_CTRL4_M0__CMN_PLL1_SS_CTRL3_M0),31,16,(uint32_t)0x0003);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_LOCK_REFCNT_IDLE__CMN_PLL1_LOCK_REFCNT_START),15,0,(uint32_t)0x000F);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_LOCK_PLLCNT_THR__CMN_PLL1_LOCK_PLLCNT_START),15,0,(uint32_t)0x019F);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_LOCK_PLLCNT_THR__CMN_PLL1_LOCK_PLLCNT_START),31,16,(uint32_t)0x0006);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_TXPUCAL_ITER_TMR__CMN_TXPUCAL_INIT_TMR),15,0,(uint32_t)0x0018);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_TXPUCAL_ITER_TMR__CMN_TXPUCAL_INIT_TMR),31,16,(uint32_t)0x0005);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_TXPDCAL_ITER_TMR__CMN_TXPDCAL_INIT_TMR),15,0,(uint32_t)0x0018);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_TXPDCAL_ITER_TMR__CMN_TXPDCAL_INIT_TMR),31,16,(uint32_t)0x0005);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_RXCAL_ITER_TMR__CMN_RXCAL_INIT_TMR),15,0,(uint32_t)0x0240);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_RXCAL_ITER_TMR__CMN_RXCAL_INIT_TMR),31,16,(uint32_t)0x0005);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_SD_CAL_REFTIM_START),15,0,(uint32_t)0x000B);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_SD_CAL_PLLCNT_START),15,0,(uint32_t)0x0137);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CLK_SEL_M0__CMN_PDIAG_PLL0_CTRL_M0),15,0,(uint32_t)0x0032);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_IADJ_M0__CMN_PDIAG_PLL0_CP_PADJ_M0),15,0,(uint32_t)0x0509);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_IADJ_M0__CMN_PDIAG_PLL0_CP_PADJ_M0),31,16,(uint32_t)0x0F00);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_TUNE_M0__CMN_PDIAG_PLL0_FILT_PADJ_M0),15,0,(uint32_t)0x0F08);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CLK_SEL_M1__CMN_PDIAG_PLL0_CTRL_M1),15,0,(uint32_t)0x0022);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_IADJ_M1__CMN_PDIAG_PLL0_CP_PADJ_M1),15,0,(uint32_t)0x0509);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_IADJ_M1__CMN_PDIAG_PLL0_CP_PADJ_M1),31,16,(uint32_t)0x0F00);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_TUNE_M1__CMN_PDIAG_PLL0_FILT_PADJ_M1),15,0,(uint32_t)0x0F08);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CLK_SEL_M0__CMN_PDIAG_PLL1_CTRL_M0),15,0,(uint32_t)0x0022);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CP_IADJ_M0__CMN_PDIAG_PLL1_CP_PADJ_M0),15,0,(uint32_t)0x0509);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CP_IADJ_M0__CMN_PDIAG_PLL1_CP_PADJ_M0),31,16,(uint32_t)0x0F00);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CP_TUNE_M0__CMN_PDIAG_PLL1_FILT_PADJ_M0),15,0,(uint32_t)0x0F08);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[laneNum].DRV_DIAG_LANE_FCM_EN_SWAIT_TMR__DRV_DIAG_LANE_FCM_EN_TO),15,0,(uint32_t)0x0180);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[laneNum].DRV_DIAG_LANE_FCM_EN_TUNE__DRV_DIAG_LANE_FCM_EN_MGN_TMR),15,0,(uint32_t)0x0074);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[laneNum].TX_RCVDET_ST_TMR__TX_RCVDET_EN_TMR),31,16,(uint32_t)0x0780);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_SDCAL0_ITER_TMR__RX_SDCAL0_INIT_TMR),15,0,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_SDCAL0_ITER_TMR__RX_SDCAL0_INIT_TMR),31,16,(uint32_t)0x0060);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_SDCAL1_ITER_TMR__RX_SDCAL1_INIT_TMR),15,0,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_SDCAL1_ITER_TMR__RX_SDCAL1_INIT_TMR),31,16,(uint32_t)0x0060);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_REE_TAP1_CLIP__RX_REE_ADDR_CFG),31,16,(uint32_t)0x0019);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_REE_CTRL_DATA_MASK__RX_REE_TAP2TON_CLIP),15,0,(uint32_t)0x0019);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_DIAG_ACYA__RX_DIAG_DCYA),31,16,(uint32_t)0x0001);

}

void csl_wiz16m_ct2_refclk19p2MHz_32b_PCIe_ext_ssc(uint32_t baseAddr, uint32_t laneNum)
{
    CSL_wiz16b8m4ct2Regs *torrent_sds_reg = (CSL_wiz16b8m4ct2Regs *)(uintptr_t)(baseAddr);

    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_SSM_BIAS_TMR),15,0,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLLSM0_PLLVREF_TMR__CMN_PLLSM0_PLLPRE_TMR),15,0,(uint32_t)0x0027);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLLSM0_PLLCLKDIS_TMR__CMN_PLLSM0_PLLLOCK_TMR),15,0,(uint32_t)0x00A1);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLLSM1_PLLVREF_TMR__CMN_PLLSM1_PLLPRE_TMR),15,0,(uint32_t)0x0027);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLLSM1_PLLCLKDIS_TMR__CMN_PLLSM1_PLLLOCK_TMR),15,0,(uint32_t)0x00A1);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_BGCAL_ITER_TMR__CMN_BGCAL_INIT_TMR),15,0,(uint32_t)0x0060);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_BGCAL_ITER_TMR__CMN_BGCAL_INIT_TMR),31,16,(uint32_t)0x0060);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_IBCAL_ITER_TMR__CMN_IBCAL_INIT_TMR),15,0,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_OVRD__CMN_PLL0_VCOCAL_TCTRL),15,0,(uint32_t)0x0003);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_ITER_TMR__CMN_PLL0_VCOCAL_INIT_TMR),15,0,(uint32_t)0x00C0);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_ITER_TMR__CMN_PLL0_VCOCAL_INIT_TMR),31,16,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_REFTIM_START),15,0,(uint32_t)0x0261);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_PLLCNT_START),15,0,(uint32_t)0x3DF3);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_FRACDIVL_M0__CMN_PLL0_INTDIV_M0),15,0,(uint32_t)0x0104);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_FRACDIVL_M0__CMN_PLL0_INTDIV_M0),31,16,(uint32_t)0x6AAB);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_HIGH_THR_M0__CMN_PLL0_FRACDIVH_M0),15,0,(uint32_t)0x0002);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_HIGH_THR_M0__CMN_PLL0_FRACDIVH_M0),31,16,(uint32_t)0x00AE);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_DSM_FBH_OVRD_M0__CMN_PLL0_DSM_DIAG_M0),15,0,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_LOCK_REFCNT_IDLE__CMN_PLL0_LOCK_REFCNT_START),15,0,(uint32_t)0x0010);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_LOCK_PLLCNT_THR__CMN_PLL0_LOCK_PLLCNT_START),15,0,(uint32_t)0x01A1);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_LOCK_PLLCNT_THR__CMN_PLL0_LOCK_PLLCNT_START),31,16,(uint32_t)0x0005);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_FRACDIVL_M1__CMN_PLL0_INTDIV_M1),15,0,(uint32_t)0x01A0);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_FRACDIVL_M1__CMN_PLL0_INTDIV_M1),31,16,(uint32_t)0xAAAB);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_HIGH_THR_M1__CMN_PLL0_FRACDIVH_M1),15,0,(uint32_t)0x0002);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_HIGH_THR_M1__CMN_PLL0_FRACDIVH_M1),31,16,(uint32_t)0x0116);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_DSM_FBH_OVRD_M1__CMN_PLL0_DSM_DIAG_M1),15,0,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_OVRD__CMN_PLL1_VCOCAL_TCTRL),15,0,(uint32_t)0x0003);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_ITER_TMR__CMN_PLL1_VCOCAL_INIT_TMR),15,0,(uint32_t)0x00C0);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_ITER_TMR__CMN_PLL1_VCOCAL_INIT_TMR),31,16,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_REFTIM_START),15,0,(uint32_t)0x0261);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_PLLCNT_START),15,0,(uint32_t)0x3DF3);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_FRACDIVL_M0__CMN_PLL1_INTDIV_M0),15,0,(uint32_t)0x01A0);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_FRACDIVL_M0__CMN_PLL1_INTDIV_M0),31,16,(uint32_t)0xAAAB);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_HIGH_THR_M0__CMN_PLL1_FRACDIVH_M0),15,0,(uint32_t)0x0002);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_HIGH_THR_M0__CMN_PLL1_FRACDIVH_M0),31,16,(uint32_t)0x0116);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_DSM_FBH_OVRD_M0__CMN_PLL1_DSM_DIAG_M0),15,0,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_LOCK_REFCNT_IDLE__CMN_PLL1_LOCK_REFCNT_START),15,0,(uint32_t)0x0010);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_LOCK_PLLCNT_THR__CMN_PLL1_LOCK_PLLCNT_START),15,0,(uint32_t)0x01A1);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_LOCK_PLLCNT_THR__CMN_PLL1_LOCK_PLLCNT_START),31,16,(uint32_t)0x0005);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_TXPUCAL_ITER_TMR__CMN_TXPUCAL_INIT_TMR),15,0,(uint32_t)0x0018);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_TXPUCAL_ITER_TMR__CMN_TXPUCAL_INIT_TMR),31,16,(uint32_t)0x0005);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_TXPDCAL_ITER_TMR__CMN_TXPDCAL_INIT_TMR),15,0,(uint32_t)0x0018);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_TXPDCAL_ITER_TMR__CMN_TXPDCAL_INIT_TMR),31,16,(uint32_t)0x0005);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_RXCAL_ITER_TMR__CMN_RXCAL_INIT_TMR),15,0,(uint32_t)0x0240);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_RXCAL_ITER_TMR__CMN_RXCAL_INIT_TMR),31,16,(uint32_t)0x0005);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_SD_CAL_REFTIM_START),15,0,(uint32_t)0x000B);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_SD_CAL_PLLCNT_START),15,0,(uint32_t)0x0137);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CLK_SEL_M0__CMN_PDIAG_PLL0_CTRL_M0),15,0,(uint32_t)0x0032);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_IADJ_M0__CMN_PDIAG_PLL0_CP_PADJ_M0),15,0,(uint32_t)0x0509);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_IADJ_M0__CMN_PDIAG_PLL0_CP_PADJ_M0),31,16,(uint32_t)0x0F00);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_TUNE_M0__CMN_PDIAG_PLL0_FILT_PADJ_M0),15,0,(uint32_t)0x0F08);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CLK_SEL_M1__CMN_PDIAG_PLL0_CTRL_M1),15,0,(uint32_t)0x0022);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_IADJ_M1__CMN_PDIAG_PLL0_CP_PADJ_M1),15,0,(uint32_t)0x0509);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_IADJ_M1__CMN_PDIAG_PLL0_CP_PADJ_M1),31,16,(uint32_t)0x0F00);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_TUNE_M1__CMN_PDIAG_PLL0_FILT_PADJ_M1),15,0,(uint32_t)0x0F08);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CLK_SEL_M0__CMN_PDIAG_PLL1_CTRL_M0),15,0,(uint32_t)0x0022);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CP_IADJ_M0__CMN_PDIAG_PLL1_CP_PADJ_M0),15,0,(uint32_t)0x0509);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CP_IADJ_M0__CMN_PDIAG_PLL1_CP_PADJ_M0),31,16,(uint32_t)0x0F00);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CP_TUNE_M0__CMN_PDIAG_PLL1_FILT_PADJ_M0),15,0,(uint32_t)0x0F08);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[laneNum].DRV_DIAG_LANE_FCM_EN_SWAIT_TMR__DRV_DIAG_LANE_FCM_EN_TO),15,0,(uint32_t)0x0180);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[laneNum].DRV_DIAG_LANE_FCM_EN_TUNE__DRV_DIAG_LANE_FCM_EN_MGN_TMR),15,0,(uint32_t)0x0074);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[laneNum].TX_RCVDET_ST_TMR__TX_RCVDET_EN_TMR),31,16,(uint32_t)0x0780);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_SDCAL0_ITER_TMR__RX_SDCAL0_INIT_TMR),15,0,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_SDCAL0_ITER_TMR__RX_SDCAL0_INIT_TMR),31,16,(uint32_t)0x0060);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_SDCAL1_ITER_TMR__RX_SDCAL1_INIT_TMR),15,0,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_SDCAL1_ITER_TMR__RX_SDCAL1_INIT_TMR),31,16,(uint32_t)0x0060);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_REE_TAP1_CLIP__RX_REE_ADDR_CFG),31,16,(uint32_t)0x0019);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_REE_CTRL_DATA_MASK__RX_REE_TAP2TON_CLIP),15,0,(uint32_t)0x0019);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_DIAG_ACYA__RX_DIAG_DCYA),31,16,(uint32_t)0x0001);

}

void csl_wiz16m_ct2_refclk20MHz_32b_PCIe_no_ssc(uint32_t baseAddr, uint32_t laneNum)
{
    CSL_wiz16b8m4ct2Regs *torrent_sds_reg = (CSL_wiz16b8m4ct2Regs *)(uintptr_t)(baseAddr);

    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_SSM_BIAS_TMR),15,0,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLLSM0_PLLVREF_TMR__CMN_PLLSM0_PLLPRE_TMR),15,0,(uint32_t)0x0028);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLLSM0_PLLCLKDIS_TMR__CMN_PLLSM0_PLLLOCK_TMR),15,0,(uint32_t)0x00A8);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLLSM1_PLLVREF_TMR__CMN_PLLSM1_PLLPRE_TMR),15,0,(uint32_t)0x0028);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLLSM1_PLLCLKDIS_TMR__CMN_PLLSM1_PLLLOCK_TMR),15,0,(uint32_t)0x00A8);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_BGCAL_ITER_TMR__CMN_BGCAL_INIT_TMR),15,0,(uint32_t)0x0064);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_BGCAL_ITER_TMR__CMN_BGCAL_INIT_TMR),31,16,(uint32_t)0x0064);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_IBCAL_ITER_TMR__CMN_IBCAL_INIT_TMR),15,0,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_OVRD__CMN_PLL0_VCOCAL_TCTRL),15,0,(uint32_t)0x0003);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_ITER_TMR__CMN_PLL0_VCOCAL_INIT_TMR),15,0,(uint32_t)0x00C8);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_ITER_TMR__CMN_PLL0_VCOCAL_INIT_TMR),31,16,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_REFTIM_START),15,0,(uint32_t)0x027A);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_PLLCNT_START),15,0,(uint32_t)0x3DEA);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_FRACDIVL_M0__CMN_PLL0_INTDIV_M0),15,0,(uint32_t)0x01F4);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_HIGH_THR_M0__CMN_PLL0_FRACDIVH_M0),15,0,(uint32_t)0x0002);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_HIGH_THR_M0__CMN_PLL0_FRACDIVH_M0),31,16,(uint32_t)0x014E);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_DSM_FBH_OVRD_M0__CMN_PLL0_DSM_DIAG_M0),15,0,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_LOCK_REFCNT_IDLE__CMN_PLL0_LOCK_REFCNT_START),15,0,(uint32_t)0x0010);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_LOCK_PLLCNT_THR__CMN_PLL0_LOCK_PLLCNT_START),15,0,(uint32_t)0x0190);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_LOCK_PLLCNT_THR__CMN_PLL0_LOCK_PLLCNT_START),31,16,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_FRACDIVL_M1__CMN_PLL0_INTDIV_M1),15,0,(uint32_t)0x0190);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_HIGH_THR_M1__CMN_PLL0_FRACDIVH_M1),15,0,(uint32_t)0x0002);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_HIGH_THR_M1__CMN_PLL0_FRACDIVH_M1),31,16,(uint32_t)0x010C);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_DSM_FBH_OVRD_M1__CMN_PLL0_DSM_DIAG_M1),15,0,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_OVRD__CMN_PLL1_VCOCAL_TCTRL),15,0,(uint32_t)0x0003);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_ITER_TMR__CMN_PLL1_VCOCAL_INIT_TMR),15,0,(uint32_t)0x00C8);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_ITER_TMR__CMN_PLL1_VCOCAL_INIT_TMR),31,16,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_REFTIM_START),15,0,(uint32_t)0x027A);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_PLLCNT_START),15,0,(uint32_t)0x3DEA);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_FRACDIVL_M0__CMN_PLL1_INTDIV_M0),15,0,(uint32_t)0x0190);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_HIGH_THR_M0__CMN_PLL1_FRACDIVH_M0),15,0,(uint32_t)0x0002);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_HIGH_THR_M0__CMN_PLL1_FRACDIVH_M0),31,16,(uint32_t)0x010C);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_DSM_FBH_OVRD_M0__CMN_PLL1_DSM_DIAG_M0),15,0,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_LOCK_REFCNT_IDLE__CMN_PLL1_LOCK_REFCNT_START),15,0,(uint32_t)0x0010);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_LOCK_PLLCNT_THR__CMN_PLL1_LOCK_PLLCNT_START),15,0,(uint32_t)0x0190);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_LOCK_PLLCNT_THR__CMN_PLL1_LOCK_PLLCNT_START),31,16,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_TXPUCAL_ITER_TMR__CMN_TXPUCAL_INIT_TMR),15,0,(uint32_t)0x0018);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_TXPUCAL_ITER_TMR__CMN_TXPUCAL_INIT_TMR),31,16,(uint32_t)0x0005);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_TXPDCAL_ITER_TMR__CMN_TXPDCAL_INIT_TMR),15,0,(uint32_t)0x0018);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_TXPDCAL_ITER_TMR__CMN_TXPDCAL_INIT_TMR),31,16,(uint32_t)0x0005);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_RXCAL_ITER_TMR__CMN_RXCAL_INIT_TMR),15,0,(uint32_t)0x0258);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_RXCAL_ITER_TMR__CMN_RXCAL_INIT_TMR),31,16,(uint32_t)0x0005);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_SD_CAL_REFTIM_START),15,0,(uint32_t)0x000B);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CLK_SEL_M0__CMN_PDIAG_PLL0_CTRL_M0),15,0,(uint32_t)0x0022);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_IADJ_M0__CMN_PDIAG_PLL0_CP_PADJ_M0),15,0,(uint32_t)0x0509);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_IADJ_M0__CMN_PDIAG_PLL0_CP_PADJ_M0),31,16,(uint32_t)0x0F00);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_TUNE_M0__CMN_PDIAG_PLL0_FILT_PADJ_M0),15,0,(uint32_t)0x0F08);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CLK_SEL_M1__CMN_PDIAG_PLL0_CTRL_M1),15,0,(uint32_t)0x0022);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_IADJ_M1__CMN_PDIAG_PLL0_CP_PADJ_M1),15,0,(uint32_t)0x0509);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_IADJ_M1__CMN_PDIAG_PLL0_CP_PADJ_M1),31,16,(uint32_t)0x0F00);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_TUNE_M1__CMN_PDIAG_PLL0_FILT_PADJ_M1),15,0,(uint32_t)0x0F08);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CLK_SEL_M0__CMN_PDIAG_PLL1_CTRL_M0),15,0,(uint32_t)0x0022);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CP_IADJ_M0__CMN_PDIAG_PLL1_CP_PADJ_M0),15,0,(uint32_t)0x0509);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CP_IADJ_M0__CMN_PDIAG_PLL1_CP_PADJ_M0),31,16,(uint32_t)0x0F00);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CP_TUNE_M0__CMN_PDIAG_PLL1_FILT_PADJ_M0),15,0,(uint32_t)0x0F08);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[laneNum].DRV_DIAG_LANE_FCM_EN_SWAIT_TMR__DRV_DIAG_LANE_FCM_EN_TO),15,0,(uint32_t)0x0190);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[laneNum].DRV_DIAG_LANE_FCM_EN_TUNE__DRV_DIAG_LANE_FCM_EN_MGN_TMR),15,0,(uint32_t)0x0078);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[laneNum].TX_RCVDET_ST_TMR__TX_RCVDET_EN_TMR),31,16,(uint32_t)0x07D0);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_SDCAL0_ITER_TMR__RX_SDCAL0_INIT_TMR),15,0,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_SDCAL0_ITER_TMR__RX_SDCAL0_INIT_TMR),31,16,(uint32_t)0x0064);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_SDCAL1_ITER_TMR__RX_SDCAL1_INIT_TMR),15,0,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_SDCAL1_ITER_TMR__RX_SDCAL1_INIT_TMR),31,16,(uint32_t)0x0064);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_REE_TAP1_CLIP__RX_REE_ADDR_CFG),31,16,(uint32_t)0x0019);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_REE_CTRL_DATA_MASK__RX_REE_TAP2TON_CLIP),15,0,(uint32_t)0x0019);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_DIAG_ACYA__RX_DIAG_DCYA),31,16,(uint32_t)0x0001);

}

void csl_wiz16m_ct2_refclk20MHz_32b_PCIe_int_ssc(uint32_t baseAddr, uint32_t laneNum)
{
    CSL_wiz16b8m4ct2Regs *torrent_sds_reg = (CSL_wiz16b8m4ct2Regs *)(uintptr_t)(baseAddr);

    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_SSM_BIAS_TMR),15,0,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLLSM0_PLLVREF_TMR__CMN_PLLSM0_PLLPRE_TMR),15,0,(uint32_t)0x0028);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLLSM0_PLLCLKDIS_TMR__CMN_PLLSM0_PLLLOCK_TMR),15,0,(uint32_t)0x00A8);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLLSM1_PLLVREF_TMR__CMN_PLLSM1_PLLPRE_TMR),15,0,(uint32_t)0x0028);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLLSM1_PLLCLKDIS_TMR__CMN_PLLSM1_PLLLOCK_TMR),15,0,(uint32_t)0x00A8);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_BGCAL_ITER_TMR__CMN_BGCAL_INIT_TMR),15,0,(uint32_t)0x0064);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_BGCAL_ITER_TMR__CMN_BGCAL_INIT_TMR),31,16,(uint32_t)0x0064);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_IBCAL_ITER_TMR__CMN_IBCAL_INIT_TMR),15,0,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_OVRD__CMN_PLL0_VCOCAL_TCTRL),15,0,(uint32_t)0x0003);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_ITER_TMR__CMN_PLL0_VCOCAL_INIT_TMR),15,0,(uint32_t)0x00C8);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_ITER_TMR__CMN_PLL0_VCOCAL_INIT_TMR),31,16,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_REFTIM_START),15,0,(uint32_t)0x0279);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_PLLCNT_START),15,0,(uint32_t)0x3DC1);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_FRACDIVL_M0__CMN_PLL0_INTDIV_M0),15,0,(uint32_t)0x01F4);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_HIGH_THR_M0__CMN_PLL0_FRACDIVH_M0),15,0,(uint32_t)0x0002);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_HIGH_THR_M0__CMN_PLL0_FRACDIVH_M0),31,16,(uint32_t)0x014E);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_DSM_FBH_OVRD_M0__CMN_PLL0_DSM_DIAG_M0),15,0,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_SS_CTRL2_M0__CMN_PLL0_SS_CTRL1_M0),15,0,(uint32_t)0x0001);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_SS_CTRL2_M0__CMN_PLL0_SS_CTRL1_M0),31,16,(uint32_t)0x0587);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_SS_CTRL4_M0__CMN_PLL0_SS_CTRL3_M0),15,0,(uint32_t)0x006E);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_SS_CTRL4_M0__CMN_PLL0_SS_CTRL3_M0),31,16,(uint32_t)0x0003);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_LOCK_REFCNT_IDLE__CMN_PLL0_LOCK_REFCNT_START),15,0,(uint32_t)0x000F);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_LOCK_PLLCNT_THR__CMN_PLL0_LOCK_PLLCNT_START),15,0,(uint32_t)0x018E);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_LOCK_PLLCNT_THR__CMN_PLL0_LOCK_PLLCNT_START),31,16,(uint32_t)0x0005);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_FRACDIVL_M1__CMN_PLL0_INTDIV_M1),15,0,(uint32_t)0x0190);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_HIGH_THR_M1__CMN_PLL0_FRACDIVH_M1),15,0,(uint32_t)0x0002);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_HIGH_THR_M1__CMN_PLL0_FRACDIVH_M1),31,16,(uint32_t)0x010C);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_DSM_FBH_OVRD_M1__CMN_PLL0_DSM_DIAG_M1),15,0,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_SS_CTRL2_M1__CMN_PLL0_SS_CTRL1_M1),15,0,(uint32_t)0x0001);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_SS_CTRL2_M1__CMN_PLL0_SS_CTRL1_M1),31,16,(uint32_t)0x046C);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_SS_CTRL4_M1__CMN_PLL0_SS_CTRL3_M1),15,0,(uint32_t)0x006E);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_SS_CTRL4_M1__CMN_PLL0_SS_CTRL3_M1),31,16,(uint32_t)0x0003);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_OVRD__CMN_PLL1_VCOCAL_TCTRL),15,0,(uint32_t)0x0003);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_ITER_TMR__CMN_PLL1_VCOCAL_INIT_TMR),15,0,(uint32_t)0x00C8);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_ITER_TMR__CMN_PLL1_VCOCAL_INIT_TMR),31,16,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_REFTIM_START),15,0,(uint32_t)0x0279);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_PLLCNT_START),15,0,(uint32_t)0x3DC1);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_FRACDIVL_M0__CMN_PLL1_INTDIV_M0),15,0,(uint32_t)0x0190);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_HIGH_THR_M0__CMN_PLL1_FRACDIVH_M0),15,0,(uint32_t)0x0002);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_HIGH_THR_M0__CMN_PLL1_FRACDIVH_M0),31,16,(uint32_t)0x010C);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_DSM_FBH_OVRD_M0__CMN_PLL1_DSM_DIAG_M0),15,0,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_SS_CTRL2_M0__CMN_PLL1_SS_CTRL1_M0),15,0,(uint32_t)0x0001);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_SS_CTRL2_M0__CMN_PLL1_SS_CTRL1_M0),31,16,(uint32_t)0x046C);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_SS_CTRL4_M0__CMN_PLL1_SS_CTRL3_M0),15,0,(uint32_t)0x006E);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_SS_CTRL4_M0__CMN_PLL1_SS_CTRL3_M0),31,16,(uint32_t)0x0003);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_LOCK_REFCNT_IDLE__CMN_PLL1_LOCK_REFCNT_START),15,0,(uint32_t)0x000F);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_LOCK_PLLCNT_THR__CMN_PLL1_LOCK_PLLCNT_START),15,0,(uint32_t)0x018E);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_LOCK_PLLCNT_THR__CMN_PLL1_LOCK_PLLCNT_START),31,16,(uint32_t)0x0005);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_TXPUCAL_ITER_TMR__CMN_TXPUCAL_INIT_TMR),15,0,(uint32_t)0x0018);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_TXPUCAL_ITER_TMR__CMN_TXPUCAL_INIT_TMR),31,16,(uint32_t)0x0005);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_TXPDCAL_ITER_TMR__CMN_TXPDCAL_INIT_TMR),15,0,(uint32_t)0x0018);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_TXPDCAL_ITER_TMR__CMN_TXPDCAL_INIT_TMR),31,16,(uint32_t)0x0005);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_RXCAL_ITER_TMR__CMN_RXCAL_INIT_TMR),15,0,(uint32_t)0x0258);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_RXCAL_ITER_TMR__CMN_RXCAL_INIT_TMR),31,16,(uint32_t)0x0005);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_SD_CAL_REFTIM_START),15,0,(uint32_t)0x000B);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CLK_SEL_M0__CMN_PDIAG_PLL0_CTRL_M0),15,0,(uint32_t)0x0022);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_IADJ_M0__CMN_PDIAG_PLL0_CP_PADJ_M0),15,0,(uint32_t)0x0509);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_IADJ_M0__CMN_PDIAG_PLL0_CP_PADJ_M0),31,16,(uint32_t)0x0F00);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_TUNE_M0__CMN_PDIAG_PLL0_FILT_PADJ_M0),15,0,(uint32_t)0x0F08);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CLK_SEL_M1__CMN_PDIAG_PLL0_CTRL_M1),15,0,(uint32_t)0x0022);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_IADJ_M1__CMN_PDIAG_PLL0_CP_PADJ_M1),15,0,(uint32_t)0x0509);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_IADJ_M1__CMN_PDIAG_PLL0_CP_PADJ_M1),31,16,(uint32_t)0x0F00);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_TUNE_M1__CMN_PDIAG_PLL0_FILT_PADJ_M1),15,0,(uint32_t)0x0F08);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CLK_SEL_M0__CMN_PDIAG_PLL1_CTRL_M0),15,0,(uint32_t)0x0022);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CP_IADJ_M0__CMN_PDIAG_PLL1_CP_PADJ_M0),15,0,(uint32_t)0x0509);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CP_IADJ_M0__CMN_PDIAG_PLL1_CP_PADJ_M0),31,16,(uint32_t)0x0F00);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CP_TUNE_M0__CMN_PDIAG_PLL1_FILT_PADJ_M0),15,0,(uint32_t)0x0F08);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[laneNum].DRV_DIAG_LANE_FCM_EN_SWAIT_TMR__DRV_DIAG_LANE_FCM_EN_TO),15,0,(uint32_t)0x0190);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[laneNum].DRV_DIAG_LANE_FCM_EN_TUNE__DRV_DIAG_LANE_FCM_EN_MGN_TMR),15,0,(uint32_t)0x0078);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[laneNum].TX_RCVDET_ST_TMR__TX_RCVDET_EN_TMR),31,16,(uint32_t)0x07D0);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_SDCAL0_ITER_TMR__RX_SDCAL0_INIT_TMR),15,0,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_SDCAL0_ITER_TMR__RX_SDCAL0_INIT_TMR),31,16,(uint32_t)0x0064);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_SDCAL1_ITER_TMR__RX_SDCAL1_INIT_TMR),15,0,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_SDCAL1_ITER_TMR__RX_SDCAL1_INIT_TMR),31,16,(uint32_t)0x0064);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_REE_TAP1_CLIP__RX_REE_ADDR_CFG),31,16,(uint32_t)0x0019);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_REE_CTRL_DATA_MASK__RX_REE_TAP2TON_CLIP),15,0,(uint32_t)0x0019);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_DIAG_ACYA__RX_DIAG_DCYA),31,16,(uint32_t)0x0001);

}

void csl_wiz16m_ct2_refclk20MHz_32b_PCIe_ext_ssc(uint32_t baseAddr, uint32_t laneNum)
{
    CSL_wiz16b8m4ct2Regs *torrent_sds_reg = (CSL_wiz16b8m4ct2Regs *)(uintptr_t)(baseAddr);

    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_SSM_BIAS_TMR),15,0,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLLSM0_PLLVREF_TMR__CMN_PLLSM0_PLLPRE_TMR),15,0,(uint32_t)0x0028);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLLSM0_PLLCLKDIS_TMR__CMN_PLLSM0_PLLLOCK_TMR),15,0,(uint32_t)0x00A8);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLLSM1_PLLVREF_TMR__CMN_PLLSM1_PLLPRE_TMR),15,0,(uint32_t)0x0028);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLLSM1_PLLCLKDIS_TMR__CMN_PLLSM1_PLLLOCK_TMR),15,0,(uint32_t)0x00A8);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_BGCAL_ITER_TMR__CMN_BGCAL_INIT_TMR),15,0,(uint32_t)0x0064);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_BGCAL_ITER_TMR__CMN_BGCAL_INIT_TMR),31,16,(uint32_t)0x0064);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_IBCAL_ITER_TMR__CMN_IBCAL_INIT_TMR),15,0,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_OVRD__CMN_PLL0_VCOCAL_TCTRL),15,0,(uint32_t)0x0003);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_ITER_TMR__CMN_PLL0_VCOCAL_INIT_TMR),15,0,(uint32_t)0x00C8);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_ITER_TMR__CMN_PLL0_VCOCAL_INIT_TMR),31,16,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_REFTIM_START),15,0,(uint32_t)0x027A);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_PLLCNT_START),15,0,(uint32_t)0x3DEA);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_FRACDIVL_M0__CMN_PLL0_INTDIV_M0),15,0,(uint32_t)0x01F4);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_HIGH_THR_M0__CMN_PLL0_FRACDIVH_M0),15,0,(uint32_t)0x0002);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_HIGH_THR_M0__CMN_PLL0_FRACDIVH_M0),31,16,(uint32_t)0x014E);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_DSM_FBH_OVRD_M0__CMN_PLL0_DSM_DIAG_M0),15,0,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_LOCK_REFCNT_IDLE__CMN_PLL0_LOCK_REFCNT_START),15,0,(uint32_t)0x0010);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_LOCK_PLLCNT_THR__CMN_PLL0_LOCK_PLLCNT_START),15,0,(uint32_t)0x0190);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_LOCK_PLLCNT_THR__CMN_PLL0_LOCK_PLLCNT_START),31,16,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_FRACDIVL_M1__CMN_PLL0_INTDIV_M1),15,0,(uint32_t)0x0190);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_HIGH_THR_M1__CMN_PLL0_FRACDIVH_M1),15,0,(uint32_t)0x0002);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_HIGH_THR_M1__CMN_PLL0_FRACDIVH_M1),31,16,(uint32_t)0x010C);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_DSM_FBH_OVRD_M1__CMN_PLL0_DSM_DIAG_M1),15,0,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_OVRD__CMN_PLL1_VCOCAL_TCTRL),15,0,(uint32_t)0x0003);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_ITER_TMR__CMN_PLL1_VCOCAL_INIT_TMR),15,0,(uint32_t)0x00C8);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_ITER_TMR__CMN_PLL1_VCOCAL_INIT_TMR),31,16,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_REFTIM_START),15,0,(uint32_t)0x027A);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_PLLCNT_START),15,0,(uint32_t)0x3DEA);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_FRACDIVL_M0__CMN_PLL1_INTDIV_M0),15,0,(uint32_t)0x0190);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_HIGH_THR_M0__CMN_PLL1_FRACDIVH_M0),15,0,(uint32_t)0x0002);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_HIGH_THR_M0__CMN_PLL1_FRACDIVH_M0),31,16,(uint32_t)0x010C);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_DSM_FBH_OVRD_M0__CMN_PLL1_DSM_DIAG_M0),15,0,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_LOCK_REFCNT_IDLE__CMN_PLL1_LOCK_REFCNT_START),15,0,(uint32_t)0x0010);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_LOCK_PLLCNT_THR__CMN_PLL1_LOCK_PLLCNT_START),15,0,(uint32_t)0x0190);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_LOCK_PLLCNT_THR__CMN_PLL1_LOCK_PLLCNT_START),31,16,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_TXPUCAL_ITER_TMR__CMN_TXPUCAL_INIT_TMR),15,0,(uint32_t)0x0018);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_TXPUCAL_ITER_TMR__CMN_TXPUCAL_INIT_TMR),31,16,(uint32_t)0x0005);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_TXPDCAL_ITER_TMR__CMN_TXPDCAL_INIT_TMR),15,0,(uint32_t)0x0018);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_TXPDCAL_ITER_TMR__CMN_TXPDCAL_INIT_TMR),31,16,(uint32_t)0x0005);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_RXCAL_ITER_TMR__CMN_RXCAL_INIT_TMR),15,0,(uint32_t)0x0258);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_RXCAL_ITER_TMR__CMN_RXCAL_INIT_TMR),31,16,(uint32_t)0x0005);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_SD_CAL_REFTIM_START),15,0,(uint32_t)0x000B);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CLK_SEL_M0__CMN_PDIAG_PLL0_CTRL_M0),15,0,(uint32_t)0x0022);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_IADJ_M0__CMN_PDIAG_PLL0_CP_PADJ_M0),15,0,(uint32_t)0x0509);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_IADJ_M0__CMN_PDIAG_PLL0_CP_PADJ_M0),31,16,(uint32_t)0x0F00);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_TUNE_M0__CMN_PDIAG_PLL0_FILT_PADJ_M0),15,0,(uint32_t)0x0F08);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CLK_SEL_M1__CMN_PDIAG_PLL0_CTRL_M1),15,0,(uint32_t)0x0022);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_IADJ_M1__CMN_PDIAG_PLL0_CP_PADJ_M1),15,0,(uint32_t)0x0509);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_IADJ_M1__CMN_PDIAG_PLL0_CP_PADJ_M1),31,16,(uint32_t)0x0F00);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_TUNE_M1__CMN_PDIAG_PLL0_FILT_PADJ_M1),15,0,(uint32_t)0x0F08);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CLK_SEL_M0__CMN_PDIAG_PLL1_CTRL_M0),15,0,(uint32_t)0x0022);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CP_IADJ_M0__CMN_PDIAG_PLL1_CP_PADJ_M0),15,0,(uint32_t)0x0509);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CP_IADJ_M0__CMN_PDIAG_PLL1_CP_PADJ_M0),31,16,(uint32_t)0x0F00);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CP_TUNE_M0__CMN_PDIAG_PLL1_FILT_PADJ_M0),15,0,(uint32_t)0x0F08);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[laneNum].DRV_DIAG_LANE_FCM_EN_SWAIT_TMR__DRV_DIAG_LANE_FCM_EN_TO),15,0,(uint32_t)0x0190);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[laneNum].DRV_DIAG_LANE_FCM_EN_TUNE__DRV_DIAG_LANE_FCM_EN_MGN_TMR),15,0,(uint32_t)0x0078);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[laneNum].TX_RCVDET_ST_TMR__TX_RCVDET_EN_TMR),31,16,(uint32_t)0x07D0);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_SDCAL0_ITER_TMR__RX_SDCAL0_INIT_TMR),15,0,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_SDCAL0_ITER_TMR__RX_SDCAL0_INIT_TMR),31,16,(uint32_t)0x0064);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_SDCAL1_ITER_TMR__RX_SDCAL1_INIT_TMR),15,0,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_SDCAL1_ITER_TMR__RX_SDCAL1_INIT_TMR),31,16,(uint32_t)0x0064);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_REE_TAP1_CLIP__RX_REE_ADDR_CFG),31,16,(uint32_t)0x0019);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_REE_CTRL_DATA_MASK__RX_REE_TAP2TON_CLIP),15,0,(uint32_t)0x0019);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_DIAG_ACYA__RX_DIAG_DCYA),31,16,(uint32_t)0x0001);

}

void csl_wiz16m_ct2_refclk24MHz_32b_PCIe_no_ssc(uint32_t baseAddr, uint32_t laneNum)
{
    CSL_wiz16b8m4ct2Regs *torrent_sds_reg = (CSL_wiz16b8m4ct2Regs *)(uintptr_t)(baseAddr);

    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_SSM_BIAS_TMR),15,0,(uint32_t)0x0018);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLLSM0_PLLVREF_TMR__CMN_PLLSM0_PLLPRE_TMR),15,0,(uint32_t)0x0030);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLLSM0_PLLCLKDIS_TMR__CMN_PLLSM0_PLLLOCK_TMR),15,0,(uint32_t)0x00C9);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLLSM1_PLLVREF_TMR__CMN_PLLSM1_PLLPRE_TMR),15,0,(uint32_t)0x0030);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLLSM1_PLLCLKDIS_TMR__CMN_PLLSM1_PLLLOCK_TMR),15,0,(uint32_t)0x00C9);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_BGCAL_ITER_TMR__CMN_BGCAL_INIT_TMR),15,0,(uint32_t)0x0078);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_BGCAL_ITER_TMR__CMN_BGCAL_INIT_TMR),31,16,(uint32_t)0x0078);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_IBCAL_ITER_TMR__CMN_IBCAL_INIT_TMR),15,0,(uint32_t)0x0018);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_OVRD__CMN_PLL0_VCOCAL_TCTRL),15,0,(uint32_t)0x0003);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_ITER_TMR__CMN_PLL0_VCOCAL_INIT_TMR),15,0,(uint32_t)0x00F0);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_ITER_TMR__CMN_PLL0_VCOCAL_INIT_TMR),31,16,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_REFTIM_START),15,0,(uint32_t)0x02F9);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_PLLCNT_START),15,0,(uint32_t)0x3DEE);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_FRACDIVL_M0__CMN_PLL0_INTDIV_M0),15,0,(uint32_t)0x01A0);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_FRACDIVL_M0__CMN_PLL0_INTDIV_M0),31,16,(uint32_t)0xAAAB);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_HIGH_THR_M0__CMN_PLL0_FRACDIVH_M0),15,0,(uint32_t)0x0002);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_HIGH_THR_M0__CMN_PLL0_FRACDIVH_M0),31,16,(uint32_t)0x0116);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_DSM_FBH_OVRD_M0__CMN_PLL0_DSM_DIAG_M0),15,0,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_LOCK_REFCNT_IDLE__CMN_PLL0_LOCK_REFCNT_START),15,0,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_LOCK_PLLCNT_THR__CMN_PLL0_LOCK_PLLCNT_START),15,0,(uint32_t)0x01A1);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_LOCK_PLLCNT_THR__CMN_PLL0_LOCK_PLLCNT_START),31,16,(uint32_t)0x0005);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_FRACDIVL_M1__CMN_PLL0_INTDIV_M1),15,0,(uint32_t)0x014D);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_FRACDIVL_M1__CMN_PLL0_INTDIV_M1),31,16,(uint32_t)0x5555);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_HIGH_THR_M1__CMN_PLL0_FRACDIVH_M1),15,0,(uint32_t)0x0002);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_HIGH_THR_M1__CMN_PLL0_FRACDIVH_M1),31,16,(uint32_t)0x00E0);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_DSM_FBH_OVRD_M1__CMN_PLL0_DSM_DIAG_M1),15,0,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_OVRD__CMN_PLL1_VCOCAL_TCTRL),15,0,(uint32_t)0x0003);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_ITER_TMR__CMN_PLL1_VCOCAL_INIT_TMR),15,0,(uint32_t)0x00F0);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_ITER_TMR__CMN_PLL1_VCOCAL_INIT_TMR),31,16,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_REFTIM_START),15,0,(uint32_t)0x02F9);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_PLLCNT_START),15,0,(uint32_t)0x3DEE);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_FRACDIVL_M0__CMN_PLL1_INTDIV_M0),15,0,(uint32_t)0x014D);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_FRACDIVL_M0__CMN_PLL1_INTDIV_M0),31,16,(uint32_t)0x5555);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_HIGH_THR_M0__CMN_PLL1_FRACDIVH_M0),15,0,(uint32_t)0x0002);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_HIGH_THR_M0__CMN_PLL1_FRACDIVH_M0),31,16,(uint32_t)0x00E0);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_DSM_FBH_OVRD_M0__CMN_PLL1_DSM_DIAG_M0),15,0,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_LOCK_REFCNT_IDLE__CMN_PLL1_LOCK_REFCNT_START),15,0,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_LOCK_PLLCNT_THR__CMN_PLL1_LOCK_PLLCNT_START),15,0,(uint32_t)0x01A1);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_LOCK_PLLCNT_THR__CMN_PLL1_LOCK_PLLCNT_START),31,16,(uint32_t)0x0005);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_TXPUCAL_ITER_TMR__CMN_TXPUCAL_INIT_TMR),15,0,(uint32_t)0x001D);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_TXPDCAL_ITER_TMR__CMN_TXPDCAL_INIT_TMR),15,0,(uint32_t)0x001D);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_RXCAL_ITER_TMR__CMN_RXCAL_INIT_TMR),15,0,(uint32_t)0x02D0);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_SD_CAL_PLLCNT_START),15,0,(uint32_t)0x0137);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CLK_SEL_M0__CMN_PDIAG_PLL0_CTRL_M0),15,0,(uint32_t)0x0022);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_IADJ_M0__CMN_PDIAG_PLL0_CP_PADJ_M0),15,0,(uint32_t)0x0509);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_IADJ_M0__CMN_PDIAG_PLL0_CP_PADJ_M0),31,16,(uint32_t)0x0F00);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_TUNE_M0__CMN_PDIAG_PLL0_FILT_PADJ_M0),15,0,(uint32_t)0x0F08);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CLK_SEL_M1__CMN_PDIAG_PLL0_CTRL_M1),15,0,(uint32_t)0x0022);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_IADJ_M1__CMN_PDIAG_PLL0_CP_PADJ_M1),15,0,(uint32_t)0x0509);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_IADJ_M1__CMN_PDIAG_PLL0_CP_PADJ_M1),31,16,(uint32_t)0x0F00);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_TUNE_M1__CMN_PDIAG_PLL0_FILT_PADJ_M1),15,0,(uint32_t)0x0F08);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CLK_SEL_M0__CMN_PDIAG_PLL1_CTRL_M0),15,0,(uint32_t)0x0022);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CP_IADJ_M0__CMN_PDIAG_PLL1_CP_PADJ_M0),15,0,(uint32_t)0x0509);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CP_IADJ_M0__CMN_PDIAG_PLL1_CP_PADJ_M0),31,16,(uint32_t)0x0F00);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CP_TUNE_M0__CMN_PDIAG_PLL1_FILT_PADJ_M0),15,0,(uint32_t)0x0F08);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[laneNum].DRV_DIAG_LANE_FCM_EN_SWAIT_TMR__DRV_DIAG_LANE_FCM_EN_TO),15,0,(uint32_t)0x01E0);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[laneNum].DRV_DIAG_LANE_FCM_EN_TUNE__DRV_DIAG_LANE_FCM_EN_MGN_TMR),15,0,(uint32_t)0x0090);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[laneNum].TX_RCVDET_ST_TMR__TX_RCVDET_EN_TMR),31,16,(uint32_t)0x0960);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_SDCAL0_ITER_TMR__RX_SDCAL0_INIT_TMR),15,0,(uint32_t)0x0018);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_SDCAL0_ITER_TMR__RX_SDCAL0_INIT_TMR),31,16,(uint32_t)0x0078);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_SDCAL1_ITER_TMR__RX_SDCAL1_INIT_TMR),15,0,(uint32_t)0x0018);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_SDCAL1_ITER_TMR__RX_SDCAL1_INIT_TMR),31,16,(uint32_t)0x0078);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_REE_TAP1_CLIP__RX_REE_ADDR_CFG),31,16,(uint32_t)0x0019);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_REE_CTRL_DATA_MASK__RX_REE_TAP2TON_CLIP),15,0,(uint32_t)0x0019);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_DIAG_ACYA__RX_DIAG_DCYA),31,16,(uint32_t)0x0001);

}

void csl_wiz16m_ct2_refclk24MHz_32b_PCIe_int_ssc(uint32_t baseAddr, uint32_t laneNum)
{
    CSL_wiz16b8m4ct2Regs *torrent_sds_reg = (CSL_wiz16b8m4ct2Regs *)(uintptr_t)(baseAddr);

    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_SSM_BIAS_TMR),15,0,(uint32_t)0x0018);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLLSM0_PLLVREF_TMR__CMN_PLLSM0_PLLPRE_TMR),15,0,(uint32_t)0x0030);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLLSM0_PLLCLKDIS_TMR__CMN_PLLSM0_PLLLOCK_TMR),15,0,(uint32_t)0x00C9);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLLSM1_PLLVREF_TMR__CMN_PLLSM1_PLLPRE_TMR),15,0,(uint32_t)0x0030);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLLSM1_PLLCLKDIS_TMR__CMN_PLLSM1_PLLLOCK_TMR),15,0,(uint32_t)0x00C9);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_BGCAL_ITER_TMR__CMN_BGCAL_INIT_TMR),15,0,(uint32_t)0x0078);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_BGCAL_ITER_TMR__CMN_BGCAL_INIT_TMR),31,16,(uint32_t)0x0078);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_IBCAL_ITER_TMR__CMN_IBCAL_INIT_TMR),15,0,(uint32_t)0x0018);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_OVRD__CMN_PLL0_VCOCAL_TCTRL),15,0,(uint32_t)0x0003);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_ITER_TMR__CMN_PLL0_VCOCAL_INIT_TMR),15,0,(uint32_t)0x00F0);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_ITER_TMR__CMN_PLL0_VCOCAL_INIT_TMR),31,16,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_REFTIM_START),15,0,(uint32_t)0x02F8);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_PLLCNT_START),15,0,(uint32_t)0x3DC6);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_FRACDIVL_M0__CMN_PLL0_INTDIV_M0),15,0,(uint32_t)0x01A0);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_FRACDIVL_M0__CMN_PLL0_INTDIV_M0),31,16,(uint32_t)0xAAAB);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_HIGH_THR_M0__CMN_PLL0_FRACDIVH_M0),15,0,(uint32_t)0x0002);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_HIGH_THR_M0__CMN_PLL0_FRACDIVH_M0),31,16,(uint32_t)0x0116);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_DSM_FBH_OVRD_M0__CMN_PLL0_DSM_DIAG_M0),15,0,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_SS_CTRL2_M0__CMN_PLL0_SS_CTRL1_M0),15,0,(uint32_t)0x0001);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_SS_CTRL2_M0__CMN_PLL0_SS_CTRL1_M0),31,16,(uint32_t)0x0416);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_SS_CTRL4_M0__CMN_PLL0_SS_CTRL3_M0),15,0,(uint32_t)0x007C);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_SS_CTRL4_M0__CMN_PLL0_SS_CTRL3_M0),31,16,(uint32_t)0x0003);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_LOCK_REFCNT_IDLE__CMN_PLL0_LOCK_REFCNT_START),15,0,(uint32_t)0x0013);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_LOCK_PLLCNT_THR__CMN_PLL0_LOCK_PLLCNT_START),15,0,(uint32_t)0x019F);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_LOCK_PLLCNT_THR__CMN_PLL0_LOCK_PLLCNT_START),31,16,(uint32_t)0x0006);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_FRACDIVL_M1__CMN_PLL0_INTDIV_M1),15,0,(uint32_t)0x014D);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_FRACDIVL_M1__CMN_PLL0_INTDIV_M1),31,16,(uint32_t)0x5555);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_HIGH_THR_M1__CMN_PLL0_FRACDIVH_M1),15,0,(uint32_t)0x0002);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_HIGH_THR_M1__CMN_PLL0_FRACDIVH_M1),31,16,(uint32_t)0x00E0);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_DSM_FBH_OVRD_M1__CMN_PLL0_DSM_DIAG_M1),15,0,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_SS_CTRL2_M1__CMN_PLL0_SS_CTRL1_M1),15,0,(uint32_t)0x0001);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_SS_CTRL2_M1__CMN_PLL0_SS_CTRL1_M1),31,16,(uint32_t)0x0331);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_SS_CTRL4_M1__CMN_PLL0_SS_CTRL3_M1),15,0,(uint32_t)0x007F);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_SS_CTRL4_M1__CMN_PLL0_SS_CTRL3_M1),31,16,(uint32_t)0x0003);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_OVRD__CMN_PLL1_VCOCAL_TCTRL),15,0,(uint32_t)0x0003);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_ITER_TMR__CMN_PLL1_VCOCAL_INIT_TMR),15,0,(uint32_t)0x00F0);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_ITER_TMR__CMN_PLL1_VCOCAL_INIT_TMR),31,16,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_REFTIM_START),15,0,(uint32_t)0x02F8);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_PLLCNT_START),15,0,(uint32_t)0x3DC6);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_FRACDIVL_M0__CMN_PLL1_INTDIV_M0),15,0,(uint32_t)0x014D);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_FRACDIVL_M0__CMN_PLL1_INTDIV_M0),31,16,(uint32_t)0x5555);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_HIGH_THR_M0__CMN_PLL1_FRACDIVH_M0),15,0,(uint32_t)0x0002);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_HIGH_THR_M0__CMN_PLL1_FRACDIVH_M0),31,16,(uint32_t)0x00E0);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_DSM_FBH_OVRD_M0__CMN_PLL1_DSM_DIAG_M0),15,0,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_SS_CTRL2_M0__CMN_PLL1_SS_CTRL1_M0),15,0,(uint32_t)0x0001);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_SS_CTRL2_M0__CMN_PLL1_SS_CTRL1_M0),31,16,(uint32_t)0x0331);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_SS_CTRL4_M0__CMN_PLL1_SS_CTRL3_M0),15,0,(uint32_t)0x007F);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_SS_CTRL4_M0__CMN_PLL1_SS_CTRL3_M0),31,16,(uint32_t)0x0003);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_LOCK_REFCNT_IDLE__CMN_PLL1_LOCK_REFCNT_START),15,0,(uint32_t)0x0013);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_LOCK_PLLCNT_THR__CMN_PLL1_LOCK_PLLCNT_START),15,0,(uint32_t)0x019F);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_LOCK_PLLCNT_THR__CMN_PLL1_LOCK_PLLCNT_START),31,16,(uint32_t)0x0006);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_TXPUCAL_ITER_TMR__CMN_TXPUCAL_INIT_TMR),15,0,(uint32_t)0x001D);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_TXPDCAL_ITER_TMR__CMN_TXPDCAL_INIT_TMR),15,0,(uint32_t)0x001D);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_RXCAL_ITER_TMR__CMN_RXCAL_INIT_TMR),15,0,(uint32_t)0x02D0);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_SD_CAL_PLLCNT_START),15,0,(uint32_t)0x0137);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CLK_SEL_M0__CMN_PDIAG_PLL0_CTRL_M0),15,0,(uint32_t)0x0022);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_IADJ_M0__CMN_PDIAG_PLL0_CP_PADJ_M0),15,0,(uint32_t)0x0509);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_IADJ_M0__CMN_PDIAG_PLL0_CP_PADJ_M0),31,16,(uint32_t)0x0F00);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_TUNE_M0__CMN_PDIAG_PLL0_FILT_PADJ_M0),15,0,(uint32_t)0x0F08);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CLK_SEL_M1__CMN_PDIAG_PLL0_CTRL_M1),15,0,(uint32_t)0x0022);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_IADJ_M1__CMN_PDIAG_PLL0_CP_PADJ_M1),15,0,(uint32_t)0x0509);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_IADJ_M1__CMN_PDIAG_PLL0_CP_PADJ_M1),31,16,(uint32_t)0x0F00);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_TUNE_M1__CMN_PDIAG_PLL0_FILT_PADJ_M1),15,0,(uint32_t)0x0F08);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CLK_SEL_M0__CMN_PDIAG_PLL1_CTRL_M0),15,0,(uint32_t)0x0022);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CP_IADJ_M0__CMN_PDIAG_PLL1_CP_PADJ_M0),15,0,(uint32_t)0x0509);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CP_IADJ_M0__CMN_PDIAG_PLL1_CP_PADJ_M0),31,16,(uint32_t)0x0F00);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CP_TUNE_M0__CMN_PDIAG_PLL1_FILT_PADJ_M0),15,0,(uint32_t)0x0F08);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[laneNum].DRV_DIAG_LANE_FCM_EN_SWAIT_TMR__DRV_DIAG_LANE_FCM_EN_TO),15,0,(uint32_t)0x01E0);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[laneNum].DRV_DIAG_LANE_FCM_EN_TUNE__DRV_DIAG_LANE_FCM_EN_MGN_TMR),15,0,(uint32_t)0x0090);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[laneNum].TX_RCVDET_ST_TMR__TX_RCVDET_EN_TMR),31,16,(uint32_t)0x0960);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_SDCAL0_ITER_TMR__RX_SDCAL0_INIT_TMR),15,0,(uint32_t)0x0018);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_SDCAL0_ITER_TMR__RX_SDCAL0_INIT_TMR),31,16,(uint32_t)0x0078);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_SDCAL1_ITER_TMR__RX_SDCAL1_INIT_TMR),15,0,(uint32_t)0x0018);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_SDCAL1_ITER_TMR__RX_SDCAL1_INIT_TMR),31,16,(uint32_t)0x0078);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_REE_TAP1_CLIP__RX_REE_ADDR_CFG),31,16,(uint32_t)0x0019);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_REE_CTRL_DATA_MASK__RX_REE_TAP2TON_CLIP),15,0,(uint32_t)0x0019);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_DIAG_ACYA__RX_DIAG_DCYA),31,16,(uint32_t)0x0001);

}

void csl_wiz16m_ct2_refclk24MHz_32b_PCIe_ext_ssc(uint32_t baseAddr, uint32_t laneNum)
{
    CSL_wiz16b8m4ct2Regs *torrent_sds_reg = (CSL_wiz16b8m4ct2Regs *)(uintptr_t)(baseAddr);

    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_SSM_BIAS_TMR),15,0,(uint32_t)0x0018);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLLSM0_PLLVREF_TMR__CMN_PLLSM0_PLLPRE_TMR),15,0,(uint32_t)0x0030);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLLSM0_PLLCLKDIS_TMR__CMN_PLLSM0_PLLLOCK_TMR),15,0,(uint32_t)0x00C9);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLLSM1_PLLVREF_TMR__CMN_PLLSM1_PLLPRE_TMR),15,0,(uint32_t)0x0030);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLLSM1_PLLCLKDIS_TMR__CMN_PLLSM1_PLLLOCK_TMR),15,0,(uint32_t)0x00C9);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_BGCAL_ITER_TMR__CMN_BGCAL_INIT_TMR),15,0,(uint32_t)0x0078);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_BGCAL_ITER_TMR__CMN_BGCAL_INIT_TMR),31,16,(uint32_t)0x0078);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_IBCAL_ITER_TMR__CMN_IBCAL_INIT_TMR),15,0,(uint32_t)0x0018);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_OVRD__CMN_PLL0_VCOCAL_TCTRL),15,0,(uint32_t)0x0003);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_ITER_TMR__CMN_PLL0_VCOCAL_INIT_TMR),15,0,(uint32_t)0x00F0);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_ITER_TMR__CMN_PLL0_VCOCAL_INIT_TMR),31,16,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_REFTIM_START),15,0,(uint32_t)0x02F9);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_PLLCNT_START),15,0,(uint32_t)0x3DEE);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_FRACDIVL_M0__CMN_PLL0_INTDIV_M0),15,0,(uint32_t)0x01A0);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_FRACDIVL_M0__CMN_PLL0_INTDIV_M0),31,16,(uint32_t)0xAAAB);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_HIGH_THR_M0__CMN_PLL0_FRACDIVH_M0),15,0,(uint32_t)0x0002);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_HIGH_THR_M0__CMN_PLL0_FRACDIVH_M0),31,16,(uint32_t)0x0116);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_DSM_FBH_OVRD_M0__CMN_PLL0_DSM_DIAG_M0),15,0,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_LOCK_REFCNT_IDLE__CMN_PLL0_LOCK_REFCNT_START),15,0,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_LOCK_PLLCNT_THR__CMN_PLL0_LOCK_PLLCNT_START),15,0,(uint32_t)0x01A1);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_LOCK_PLLCNT_THR__CMN_PLL0_LOCK_PLLCNT_START),31,16,(uint32_t)0x0005);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_FRACDIVL_M1__CMN_PLL0_INTDIV_M1),15,0,(uint32_t)0x014D);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_FRACDIVL_M1__CMN_PLL0_INTDIV_M1),31,16,(uint32_t)0x5555);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_HIGH_THR_M1__CMN_PLL0_FRACDIVH_M1),15,0,(uint32_t)0x0002);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_HIGH_THR_M1__CMN_PLL0_FRACDIVH_M1),31,16,(uint32_t)0x00E0);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_DSM_FBH_OVRD_M1__CMN_PLL0_DSM_DIAG_M1),15,0,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_OVRD__CMN_PLL1_VCOCAL_TCTRL),15,0,(uint32_t)0x0003);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_ITER_TMR__CMN_PLL1_VCOCAL_INIT_TMR),15,0,(uint32_t)0x00F0);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_ITER_TMR__CMN_PLL1_VCOCAL_INIT_TMR),31,16,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_REFTIM_START),15,0,(uint32_t)0x02F9);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_PLLCNT_START),15,0,(uint32_t)0x3DEE);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_FRACDIVL_M0__CMN_PLL1_INTDIV_M0),15,0,(uint32_t)0x014D);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_FRACDIVL_M0__CMN_PLL1_INTDIV_M0),31,16,(uint32_t)0x5555);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_HIGH_THR_M0__CMN_PLL1_FRACDIVH_M0),15,0,(uint32_t)0x0002);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_HIGH_THR_M0__CMN_PLL1_FRACDIVH_M0),31,16,(uint32_t)0x00E0);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_DSM_FBH_OVRD_M0__CMN_PLL1_DSM_DIAG_M0),15,0,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_LOCK_REFCNT_IDLE__CMN_PLL1_LOCK_REFCNT_START),15,0,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_LOCK_PLLCNT_THR__CMN_PLL1_LOCK_PLLCNT_START),15,0,(uint32_t)0x01A1);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_LOCK_PLLCNT_THR__CMN_PLL1_LOCK_PLLCNT_START),31,16,(uint32_t)0x0005);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_TXPUCAL_ITER_TMR__CMN_TXPUCAL_INIT_TMR),15,0,(uint32_t)0x001D);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_TXPDCAL_ITER_TMR__CMN_TXPDCAL_INIT_TMR),15,0,(uint32_t)0x001D);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_RXCAL_ITER_TMR__CMN_RXCAL_INIT_TMR),15,0,(uint32_t)0x02D0);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_SD_CAL_PLLCNT_START),15,0,(uint32_t)0x0137);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CLK_SEL_M0__CMN_PDIAG_PLL0_CTRL_M0),15,0,(uint32_t)0x0022);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_IADJ_M0__CMN_PDIAG_PLL0_CP_PADJ_M0),15,0,(uint32_t)0x0509);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_IADJ_M0__CMN_PDIAG_PLL0_CP_PADJ_M0),31,16,(uint32_t)0x0F00);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_TUNE_M0__CMN_PDIAG_PLL0_FILT_PADJ_M0),15,0,(uint32_t)0x0F08);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CLK_SEL_M1__CMN_PDIAG_PLL0_CTRL_M1),15,0,(uint32_t)0x0022);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_IADJ_M1__CMN_PDIAG_PLL0_CP_PADJ_M1),15,0,(uint32_t)0x0509);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_IADJ_M1__CMN_PDIAG_PLL0_CP_PADJ_M1),31,16,(uint32_t)0x0F00);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_TUNE_M1__CMN_PDIAG_PLL0_FILT_PADJ_M1),15,0,(uint32_t)0x0F08);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CLK_SEL_M0__CMN_PDIAG_PLL1_CTRL_M0),15,0,(uint32_t)0x0022);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CP_IADJ_M0__CMN_PDIAG_PLL1_CP_PADJ_M0),15,0,(uint32_t)0x0509);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CP_IADJ_M0__CMN_PDIAG_PLL1_CP_PADJ_M0),31,16,(uint32_t)0x0F00);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CP_TUNE_M0__CMN_PDIAG_PLL1_FILT_PADJ_M0),15,0,(uint32_t)0x0F08);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[laneNum].DRV_DIAG_LANE_FCM_EN_SWAIT_TMR__DRV_DIAG_LANE_FCM_EN_TO),15,0,(uint32_t)0x01E0);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[laneNum].DRV_DIAG_LANE_FCM_EN_TUNE__DRV_DIAG_LANE_FCM_EN_MGN_TMR),15,0,(uint32_t)0x0090);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[laneNum].TX_RCVDET_ST_TMR__TX_RCVDET_EN_TMR),31,16,(uint32_t)0x0960);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_SDCAL0_ITER_TMR__RX_SDCAL0_INIT_TMR),15,0,(uint32_t)0x0018);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_SDCAL0_ITER_TMR__RX_SDCAL0_INIT_TMR),31,16,(uint32_t)0x0078);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_SDCAL1_ITER_TMR__RX_SDCAL1_INIT_TMR),15,0,(uint32_t)0x0018);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_SDCAL1_ITER_TMR__RX_SDCAL1_INIT_TMR),31,16,(uint32_t)0x0078);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_REE_TAP1_CLIP__RX_REE_ADDR_CFG),31,16,(uint32_t)0x0019);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_REE_CTRL_DATA_MASK__RX_REE_TAP2TON_CLIP),15,0,(uint32_t)0x0019);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_DIAG_ACYA__RX_DIAG_DCYA),31,16,(uint32_t)0x0001);

}

void csl_wiz16m_ct2_refclk25MHz_32b_PCIe_no_ssc(uint32_t baseAddr, uint32_t laneNum)
{
    CSL_wiz16b8m4ct2Regs *torrent_sds_reg = (CSL_wiz16b8m4ct2Regs *)(uintptr_t)(baseAddr);

    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_OVRD__CMN_PLL0_VCOCAL_TCTRL),15,0,(uint32_t)0x0003);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_ITER_TMR__CMN_PLL0_VCOCAL_INIT_TMR),15,0,(uint32_t)0x00FA);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_ITER_TMR__CMN_PLL0_VCOCAL_INIT_TMR),31,16,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_REFTIM_START),15,0,(uint32_t)0x0318);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_PLLCNT_START),15,0,(uint32_t)0x3DE0);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_FRACDIVL_M0__CMN_PLL0_INTDIV_M0),15,0,(uint32_t)0x0190);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_HIGH_THR_M0__CMN_PLL0_FRACDIVH_M0),15,0,(uint32_t)0x0002);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_HIGH_THR_M0__CMN_PLL0_FRACDIVH_M0),31,16,(uint32_t)0x010C);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_DSM_FBH_OVRD_M0__CMN_PLL0_DSM_DIAG_M0),15,0,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_LOCK_REFCNT_IDLE__CMN_PLL0_LOCK_REFCNT_START),15,0,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_LOCK_PLLCNT_THR__CMN_PLL0_LOCK_PLLCNT_START),15,0,(uint32_t)0x0190);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_LOCK_PLLCNT_THR__CMN_PLL0_LOCK_PLLCNT_START),31,16,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_FRACDIVL_M1__CMN_PLL0_INTDIV_M1),15,0,(uint32_t)0x0140);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_HIGH_THR_M1__CMN_PLL0_FRACDIVH_M1),15,0,(uint32_t)0x0002);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_HIGH_THR_M1__CMN_PLL0_FRACDIVH_M1),31,16,(uint32_t)0x00D6);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_DSM_FBH_OVRD_M1__CMN_PLL0_DSM_DIAG_M1),15,0,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_OVRD__CMN_PLL1_VCOCAL_TCTRL),15,0,(uint32_t)0x0003);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_ITER_TMR__CMN_PLL1_VCOCAL_INIT_TMR),15,0,(uint32_t)0x00FA);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_ITER_TMR__CMN_PLL1_VCOCAL_INIT_TMR),31,16,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_REFTIM_START),15,0,(uint32_t)0x0318);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_PLLCNT_START),15,0,(uint32_t)0x3DE0);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_FRACDIVL_M0__CMN_PLL1_INTDIV_M0),15,0,(uint32_t)0x0140);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_HIGH_THR_M0__CMN_PLL1_FRACDIVH_M0),15,0,(uint32_t)0x0002);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_HIGH_THR_M0__CMN_PLL1_FRACDIVH_M0),31,16,(uint32_t)0x00D6);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_DSM_FBH_OVRD_M0__CMN_PLL1_DSM_DIAG_M0),15,0,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_LOCK_REFCNT_IDLE__CMN_PLL1_LOCK_REFCNT_START),15,0,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_LOCK_PLLCNT_THR__CMN_PLL1_LOCK_PLLCNT_START),15,0,(uint32_t)0x0190);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_LOCK_PLLCNT_THR__CMN_PLL1_LOCK_PLLCNT_START),31,16,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CLK_SEL_M0__CMN_PDIAG_PLL0_CTRL_M0),15,0,(uint32_t)0x0022);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_IADJ_M0__CMN_PDIAG_PLL0_CP_PADJ_M0),15,0,(uint32_t)0x0509);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_IADJ_M0__CMN_PDIAG_PLL0_CP_PADJ_M0),31,16,(uint32_t)0x0F00);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_TUNE_M0__CMN_PDIAG_PLL0_FILT_PADJ_M0),15,0,(uint32_t)0x0F08);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CLK_SEL_M1__CMN_PDIAG_PLL0_CTRL_M1),15,0,(uint32_t)0x0022);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_IADJ_M1__CMN_PDIAG_PLL0_CP_PADJ_M1),15,0,(uint32_t)0x0509);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_IADJ_M1__CMN_PDIAG_PLL0_CP_PADJ_M1),31,16,(uint32_t)0x0F00);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_TUNE_M1__CMN_PDIAG_PLL0_FILT_PADJ_M1),15,0,(uint32_t)0x0F08);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CLK_SEL_M0__CMN_PDIAG_PLL1_CTRL_M0),15,0,(uint32_t)0x0022);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CP_IADJ_M0__CMN_PDIAG_PLL1_CP_PADJ_M0),15,0,(uint32_t)0x0509);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CP_IADJ_M0__CMN_PDIAG_PLL1_CP_PADJ_M0),31,16,(uint32_t)0x0F00);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CP_TUNE_M0__CMN_PDIAG_PLL1_FILT_PADJ_M0),15,0,(uint32_t)0x0F08);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_REE_TAP1_CLIP__RX_REE_ADDR_CFG),31,16,(uint32_t)0x0019);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_REE_CTRL_DATA_MASK__RX_REE_TAP2TON_CLIP),15,0,(uint32_t)0x0019);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_DIAG_ACYA__RX_DIAG_DCYA),31,16,(uint32_t)0x0001);

}

void csl_wiz16m_ct2_refclk25MHz_32b_PCIe_int_ssc(uint32_t baseAddr, uint32_t laneNum)
{
    CSL_wiz16b8m4ct2Regs *torrent_sds_reg = (CSL_wiz16b8m4ct2Regs *)(uintptr_t)(baseAddr);

    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_OVRD__CMN_PLL0_VCOCAL_TCTRL),15,0,(uint32_t)0x0003);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_ITER_TMR__CMN_PLL0_VCOCAL_INIT_TMR),15,0,(uint32_t)0x00FA);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_ITER_TMR__CMN_PLL0_VCOCAL_INIT_TMR),31,16,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_REFTIM_START),15,0,(uint32_t)0x0317);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_PLLCNT_START),15,0,(uint32_t)0x3DB7);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_FRACDIVL_M0__CMN_PLL0_INTDIV_M0),15,0,(uint32_t)0x0190);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_HIGH_THR_M0__CMN_PLL0_FRACDIVH_M0),15,0,(uint32_t)0x0002);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_HIGH_THR_M0__CMN_PLL0_FRACDIVH_M0),31,16,(uint32_t)0x010C);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_DSM_FBH_OVRD_M0__CMN_PLL0_DSM_DIAG_M0),15,0,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_SS_CTRL2_M0__CMN_PLL0_SS_CTRL1_M0),15,0,(uint32_t)0x0001);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_SS_CTRL2_M0__CMN_PLL0_SS_CTRL1_M0),31,16,(uint32_t)0x04B9);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_SS_CTRL4_M0__CMN_PLL0_SS_CTRL3_M0),15,0,(uint32_t)0x0067);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_SS_CTRL4_M0__CMN_PLL0_SS_CTRL3_M0),31,16,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_LOCK_REFCNT_IDLE__CMN_PLL0_LOCK_REFCNT_START),15,0,(uint32_t)0x0013);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_LOCK_PLLCNT_THR__CMN_PLL0_LOCK_PLLCNT_START),15,0,(uint32_t)0x018E);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_LOCK_PLLCNT_THR__CMN_PLL0_LOCK_PLLCNT_START),31,16,(uint32_t)0x0005);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_FRACDIVL_M1__CMN_PLL0_INTDIV_M1),15,0,(uint32_t)0x0140);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_HIGH_THR_M1__CMN_PLL0_FRACDIVH_M1),15,0,(uint32_t)0x0002);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_HIGH_THR_M1__CMN_PLL0_FRACDIVH_M1),31,16,(uint32_t)0x00D6);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_DSM_FBH_OVRD_M1__CMN_PLL0_DSM_DIAG_M1),15,0,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_SS_CTRL2_M1__CMN_PLL0_SS_CTRL1_M1),15,0,(uint32_t)0x0001);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_SS_CTRL2_M1__CMN_PLL0_SS_CTRL1_M1),31,16,(uint32_t)0x0403);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_SS_CTRL4_M1__CMN_PLL0_SS_CTRL3_M1),15,0,(uint32_t)0x0061);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_SS_CTRL4_M1__CMN_PLL0_SS_CTRL3_M1),31,16,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_OVRD__CMN_PLL1_VCOCAL_TCTRL),15,0,(uint32_t)0x0003);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_ITER_TMR__CMN_PLL1_VCOCAL_INIT_TMR),15,0,(uint32_t)0x00FA);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_ITER_TMR__CMN_PLL1_VCOCAL_INIT_TMR),31,16,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_REFTIM_START),15,0,(uint32_t)0x0317);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_PLLCNT_START),15,0,(uint32_t)0x3DB7);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_FRACDIVL_M0__CMN_PLL1_INTDIV_M0),15,0,(uint32_t)0x0140);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_HIGH_THR_M0__CMN_PLL1_FRACDIVH_M0),15,0,(uint32_t)0x0002);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_HIGH_THR_M0__CMN_PLL1_FRACDIVH_M0),31,16,(uint32_t)0x00D6);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_DSM_FBH_OVRD_M0__CMN_PLL1_DSM_DIAG_M0),15,0,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_SS_CTRL2_M0__CMN_PLL1_SS_CTRL1_M0),15,0,(uint32_t)0x0001);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_SS_CTRL2_M0__CMN_PLL1_SS_CTRL1_M0),31,16,(uint32_t)0x0403);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_SS_CTRL4_M0__CMN_PLL1_SS_CTRL3_M0),15,0,(uint32_t)0x0061);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_SS_CTRL4_M0__CMN_PLL1_SS_CTRL3_M0),31,16,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_LOCK_REFCNT_IDLE__CMN_PLL1_LOCK_REFCNT_START),15,0,(uint32_t)0x0013);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_LOCK_PLLCNT_THR__CMN_PLL1_LOCK_PLLCNT_START),15,0,(uint32_t)0x018E);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_LOCK_PLLCNT_THR__CMN_PLL1_LOCK_PLLCNT_START),31,16,(uint32_t)0x0005);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CLK_SEL_M0__CMN_PDIAG_PLL0_CTRL_M0),15,0,(uint32_t)0x0022);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_IADJ_M0__CMN_PDIAG_PLL0_CP_PADJ_M0),15,0,(uint32_t)0x0509);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_IADJ_M0__CMN_PDIAG_PLL0_CP_PADJ_M0),31,16,(uint32_t)0x0F00);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_TUNE_M0__CMN_PDIAG_PLL0_FILT_PADJ_M0),15,0,(uint32_t)0x0F08);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CLK_SEL_M1__CMN_PDIAG_PLL0_CTRL_M1),15,0,(uint32_t)0x0022);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_IADJ_M1__CMN_PDIAG_PLL0_CP_PADJ_M1),15,0,(uint32_t)0x0509);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_IADJ_M1__CMN_PDIAG_PLL0_CP_PADJ_M1),31,16,(uint32_t)0x0F00);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_TUNE_M1__CMN_PDIAG_PLL0_FILT_PADJ_M1),15,0,(uint32_t)0x0F08);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CLK_SEL_M0__CMN_PDIAG_PLL1_CTRL_M0),15,0,(uint32_t)0x0022);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CP_IADJ_M0__CMN_PDIAG_PLL1_CP_PADJ_M0),15,0,(uint32_t)0x0509);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CP_IADJ_M0__CMN_PDIAG_PLL1_CP_PADJ_M0),31,16,(uint32_t)0x0F00);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CP_TUNE_M0__CMN_PDIAG_PLL1_FILT_PADJ_M0),15,0,(uint32_t)0x0F08);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_REE_TAP1_CLIP__RX_REE_ADDR_CFG),31,16,(uint32_t)0x0019);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_REE_CTRL_DATA_MASK__RX_REE_TAP2TON_CLIP),15,0,(uint32_t)0x0019);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_DIAG_ACYA__RX_DIAG_DCYA),31,16,(uint32_t)0x0001);

}

void csl_wiz16m_ct2_refclk25MHz_32b_PCIe_ext_ssc(uint32_t baseAddr, uint32_t laneNum)
{
    CSL_wiz16b8m4ct2Regs *torrent_sds_reg = (CSL_wiz16b8m4ct2Regs *)(uintptr_t)(baseAddr);

    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_OVRD__CMN_PLL0_VCOCAL_TCTRL),15,0,(uint32_t)0x0003);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_ITER_TMR__CMN_PLL0_VCOCAL_INIT_TMR),15,0,(uint32_t)0x00FA);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_ITER_TMR__CMN_PLL0_VCOCAL_INIT_TMR),31,16,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_REFTIM_START),15,0,(uint32_t)0x0318);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_PLLCNT_START),15,0,(uint32_t)0x3DE0);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_FRACDIVL_M0__CMN_PLL0_INTDIV_M0),15,0,(uint32_t)0x0190);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_HIGH_THR_M0__CMN_PLL0_FRACDIVH_M0),15,0,(uint32_t)0x0002);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_HIGH_THR_M0__CMN_PLL0_FRACDIVH_M0),31,16,(uint32_t)0x010C);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_DSM_FBH_OVRD_M0__CMN_PLL0_DSM_DIAG_M0),15,0,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_LOCK_REFCNT_IDLE__CMN_PLL0_LOCK_REFCNT_START),15,0,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_LOCK_PLLCNT_THR__CMN_PLL0_LOCK_PLLCNT_START),15,0,(uint32_t)0x0190);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_LOCK_PLLCNT_THR__CMN_PLL0_LOCK_PLLCNT_START),31,16,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_FRACDIVL_M1__CMN_PLL0_INTDIV_M1),15,0,(uint32_t)0x0140);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_HIGH_THR_M1__CMN_PLL0_FRACDIVH_M1),15,0,(uint32_t)0x0002);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_HIGH_THR_M1__CMN_PLL0_FRACDIVH_M1),31,16,(uint32_t)0x00D6);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_DSM_FBH_OVRD_M1__CMN_PLL0_DSM_DIAG_M1),15,0,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_OVRD__CMN_PLL1_VCOCAL_TCTRL),15,0,(uint32_t)0x0003);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_ITER_TMR__CMN_PLL1_VCOCAL_INIT_TMR),15,0,(uint32_t)0x00FA);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_ITER_TMR__CMN_PLL1_VCOCAL_INIT_TMR),31,16,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_REFTIM_START),15,0,(uint32_t)0x0318);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_PLLCNT_START),15,0,(uint32_t)0x3DE0);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_FRACDIVL_M0__CMN_PLL1_INTDIV_M0),15,0,(uint32_t)0x0140);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_HIGH_THR_M0__CMN_PLL1_FRACDIVH_M0),15,0,(uint32_t)0x0002);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_HIGH_THR_M0__CMN_PLL1_FRACDIVH_M0),31,16,(uint32_t)0x00D6);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_DSM_FBH_OVRD_M0__CMN_PLL1_DSM_DIAG_M0),15,0,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_LOCK_REFCNT_IDLE__CMN_PLL1_LOCK_REFCNT_START),15,0,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_LOCK_PLLCNT_THR__CMN_PLL1_LOCK_PLLCNT_START),15,0,(uint32_t)0x0190);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_LOCK_PLLCNT_THR__CMN_PLL1_LOCK_PLLCNT_START),31,16,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CLK_SEL_M0__CMN_PDIAG_PLL0_CTRL_M0),15,0,(uint32_t)0x0022);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_IADJ_M0__CMN_PDIAG_PLL0_CP_PADJ_M0),15,0,(uint32_t)0x0509);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_IADJ_M0__CMN_PDIAG_PLL0_CP_PADJ_M0),31,16,(uint32_t)0x0F00);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_TUNE_M0__CMN_PDIAG_PLL0_FILT_PADJ_M0),15,0,(uint32_t)0x0F08);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CLK_SEL_M1__CMN_PDIAG_PLL0_CTRL_M1),15,0,(uint32_t)0x0022);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_IADJ_M1__CMN_PDIAG_PLL0_CP_PADJ_M1),15,0,(uint32_t)0x0509);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_IADJ_M1__CMN_PDIAG_PLL0_CP_PADJ_M1),31,16,(uint32_t)0x0F00);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_TUNE_M1__CMN_PDIAG_PLL0_FILT_PADJ_M1),15,0,(uint32_t)0x0F08);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CLK_SEL_M0__CMN_PDIAG_PLL1_CTRL_M0),15,0,(uint32_t)0x0022);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CP_IADJ_M0__CMN_PDIAG_PLL1_CP_PADJ_M0),15,0,(uint32_t)0x0509);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CP_IADJ_M0__CMN_PDIAG_PLL1_CP_PADJ_M0),31,16,(uint32_t)0x0F00);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CP_TUNE_M0__CMN_PDIAG_PLL1_FILT_PADJ_M0),15,0,(uint32_t)0x0F08);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_REE_TAP1_CLIP__RX_REE_ADDR_CFG),31,16,(uint32_t)0x0019);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_REE_CTRL_DATA_MASK__RX_REE_TAP2TON_CLIP),15,0,(uint32_t)0x0019);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_DIAG_ACYA__RX_DIAG_DCYA),31,16,(uint32_t)0x0001);

}

void csl_wiz16m_ct2_refclk26MHz_32b_PCIe_no_ssc(uint32_t baseAddr, uint32_t laneNum)
{
    CSL_wiz16b8m4ct2Regs *torrent_sds_reg = (CSL_wiz16b8m4ct2Regs *)(uintptr_t)(baseAddr);

    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_SSM_BIAS_TMR),15,0,(uint32_t)0x001A);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLLSM0_PLLVREF_TMR__CMN_PLLSM0_PLLPRE_TMR),15,0,(uint32_t)0x0034);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLLSM0_PLLCLKDIS_TMR__CMN_PLLSM0_PLLLOCK_TMR),15,0,(uint32_t)0x00DA);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLLSM1_PLLVREF_TMR__CMN_PLLSM1_PLLPRE_TMR),15,0,(uint32_t)0x0034);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLLSM1_PLLCLKDIS_TMR__CMN_PLLSM1_PLLLOCK_TMR),15,0,(uint32_t)0x00DA);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_BGCAL_ITER_TMR__CMN_BGCAL_INIT_TMR),15,0,(uint32_t)0x0082);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_BGCAL_ITER_TMR__CMN_BGCAL_INIT_TMR),31,16,(uint32_t)0x0082);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_IBCAL_ITER_TMR__CMN_IBCAL_INIT_TMR),15,0,(uint32_t)0x001A);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_OVRD__CMN_PLL0_VCOCAL_TCTRL),15,0,(uint32_t)0x0003);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_ITER_TMR__CMN_PLL0_VCOCAL_INIT_TMR),15,0,(uint32_t)0x0104);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_ITER_TMR__CMN_PLL0_VCOCAL_INIT_TMR),31,16,(uint32_t)0x0005);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_REFTIM_START),15,0,(uint32_t)0x0338);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_PLLCNT_START),15,0,(uint32_t)0x3DE6);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_FRACDIVL_M0__CMN_PLL0_INTDIV_M0),15,0,(uint32_t)0x0180);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_FRACDIVL_M0__CMN_PLL0_INTDIV_M0),31,16,(uint32_t)0x9D8A);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_HIGH_THR_M0__CMN_PLL0_FRACDIVH_M0),15,0,(uint32_t)0x0002);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_HIGH_THR_M0__CMN_PLL0_FRACDIVH_M0),31,16,(uint32_t)0x0102);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_DSM_FBH_OVRD_M0__CMN_PLL0_DSM_DIAG_M0),15,0,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_LOCK_REFCNT_IDLE__CMN_PLL0_LOCK_REFCNT_START),15,0,(uint32_t)0x0015);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_LOCK_PLLCNT_THR__CMN_PLL0_LOCK_PLLCNT_START),15,0,(uint32_t)0x0194);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_LOCK_PLLCNT_THR__CMN_PLL0_LOCK_PLLCNT_START),31,16,(uint32_t)0x0005);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_FRACDIVL_M1__CMN_PLL0_INTDIV_M1),15,0,(uint32_t)0x0133);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_FRACDIVL_M1__CMN_PLL0_INTDIV_M1),31,16,(uint32_t)0xB13B);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_HIGH_THR_M1__CMN_PLL0_FRACDIVH_M1),15,0,(uint32_t)0x0002);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_HIGH_THR_M1__CMN_PLL0_FRACDIVH_M1),31,16,(uint32_t)0x00CE);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_DSM_FBH_OVRD_M1__CMN_PLL0_DSM_DIAG_M1),15,0,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_OVRD__CMN_PLL1_VCOCAL_TCTRL),15,0,(uint32_t)0x0003);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_ITER_TMR__CMN_PLL1_VCOCAL_INIT_TMR),15,0,(uint32_t)0x0104);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_ITER_TMR__CMN_PLL1_VCOCAL_INIT_TMR),31,16,(uint32_t)0x0005);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_REFTIM_START),15,0,(uint32_t)0x0338);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_PLLCNT_START),15,0,(uint32_t)0x3DE6);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_FRACDIVL_M0__CMN_PLL1_INTDIV_M0),15,0,(uint32_t)0x0133);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_FRACDIVL_M0__CMN_PLL1_INTDIV_M0),31,16,(uint32_t)0xB13B);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_HIGH_THR_M0__CMN_PLL1_FRACDIVH_M0),15,0,(uint32_t)0x0002);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_HIGH_THR_M0__CMN_PLL1_FRACDIVH_M0),31,16,(uint32_t)0x00CE);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_DSM_FBH_OVRD_M0__CMN_PLL1_DSM_DIAG_M0),15,0,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_LOCK_REFCNT_IDLE__CMN_PLL1_LOCK_REFCNT_START),15,0,(uint32_t)0x0015);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_LOCK_PLLCNT_THR__CMN_PLL1_LOCK_PLLCNT_START),15,0,(uint32_t)0x0194);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_LOCK_PLLCNT_THR__CMN_PLL1_LOCK_PLLCNT_START),31,16,(uint32_t)0x0005);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_TXPUCAL_ITER_TMR__CMN_TXPUCAL_INIT_TMR),15,0,(uint32_t)0x0020);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_TXPUCAL_ITER_TMR__CMN_TXPUCAL_INIT_TMR),31,16,(uint32_t)0x0007);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_TXPDCAL_ITER_TMR__CMN_TXPDCAL_INIT_TMR),15,0,(uint32_t)0x0020);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_TXPDCAL_ITER_TMR__CMN_TXPDCAL_INIT_TMR),31,16,(uint32_t)0x0007);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_RXCAL_ITER_TMR__CMN_RXCAL_INIT_TMR),15,0,(uint32_t)0x030C);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_RXCAL_ITER_TMR__CMN_RXCAL_INIT_TMR),31,16,(uint32_t)0x0007);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_SD_CAL_ITER_TMR__CMN_SD_CAL_INIT_TMR),15,0,(uint32_t)0x0007);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_SD_CAL_ITER_TMR__CMN_SD_CAL_INIT_TMR),31,16,(uint32_t)0x0003);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_SD_CAL_REFTIM_START),15,0,(uint32_t)0x000F);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_SD_CAL_PLLCNT_START),15,0,(uint32_t)0x0132);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CLK_SEL_M0__CMN_PDIAG_PLL0_CTRL_M0),15,0,(uint32_t)0x0022);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_IADJ_M0__CMN_PDIAG_PLL0_CP_PADJ_M0),15,0,(uint32_t)0x0509);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_IADJ_M0__CMN_PDIAG_PLL0_CP_PADJ_M0),31,16,(uint32_t)0x0F00);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_TUNE_M0__CMN_PDIAG_PLL0_FILT_PADJ_M0),15,0,(uint32_t)0x0F08);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CLK_SEL_M1__CMN_PDIAG_PLL0_CTRL_M1),15,0,(uint32_t)0x0022);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_IADJ_M1__CMN_PDIAG_PLL0_CP_PADJ_M1),15,0,(uint32_t)0x0509);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_IADJ_M1__CMN_PDIAG_PLL0_CP_PADJ_M1),31,16,(uint32_t)0x0F00);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_TUNE_M1__CMN_PDIAG_PLL0_FILT_PADJ_M1),15,0,(uint32_t)0x0F08);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CLK_SEL_M0__CMN_PDIAG_PLL1_CTRL_M0),15,0,(uint32_t)0x0022);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CP_IADJ_M0__CMN_PDIAG_PLL1_CP_PADJ_M0),15,0,(uint32_t)0x0509);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CP_IADJ_M0__CMN_PDIAG_PLL1_CP_PADJ_M0),31,16,(uint32_t)0x0F00);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CP_TUNE_M0__CMN_PDIAG_PLL1_FILT_PADJ_M0),15,0,(uint32_t)0x0F08);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[laneNum].DRV_DIAG_LANE_FCM_EN_SWAIT_TMR__DRV_DIAG_LANE_FCM_EN_TO),15,0,(uint32_t)0x0208);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[laneNum].DRV_DIAG_LANE_FCM_EN_TUNE__DRV_DIAG_LANE_FCM_EN_MGN_TMR),15,0,(uint32_t)0x009C);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[laneNum].TX_RCVDET_ST_TMR__TX_RCVDET_EN_TMR),31,16,(uint32_t)0x0A28);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_SDCAL0_ITER_TMR__RX_SDCAL0_INIT_TMR),15,0,(uint32_t)0x001A);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_SDCAL0_ITER_TMR__RX_SDCAL0_INIT_TMR),31,16,(uint32_t)0x0082);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_SDCAL1_ITER_TMR__RX_SDCAL1_INIT_TMR),15,0,(uint32_t)0x001A);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_SDCAL1_ITER_TMR__RX_SDCAL1_INIT_TMR),31,16,(uint32_t)0x0082);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_REE_TAP1_CLIP__RX_REE_ADDR_CFG),31,16,(uint32_t)0x0019);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_REE_CTRL_DATA_MASK__RX_REE_TAP2TON_CLIP),15,0,(uint32_t)0x0019);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_DIAG_ACYA__RX_DIAG_DCYA),31,16,(uint32_t)0x0001);

}

void csl_wiz16m_ct2_refclk26MHz_32b_PCIe_int_ssc(uint32_t baseAddr, uint32_t laneNum)
{
    CSL_wiz16b8m4ct2Regs *torrent_sds_reg = (CSL_wiz16b8m4ct2Regs *)(uintptr_t)(baseAddr);

    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_SSM_BIAS_TMR),15,0,(uint32_t)0x001A);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLLSM0_PLLVREF_TMR__CMN_PLLSM0_PLLPRE_TMR),15,0,(uint32_t)0x0034);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLLSM0_PLLCLKDIS_TMR__CMN_PLLSM0_PLLLOCK_TMR),15,0,(uint32_t)0x00DA);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLLSM1_PLLVREF_TMR__CMN_PLLSM1_PLLPRE_TMR),15,0,(uint32_t)0x0034);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLLSM1_PLLCLKDIS_TMR__CMN_PLLSM1_PLLLOCK_TMR),15,0,(uint32_t)0x00DA);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_BGCAL_ITER_TMR__CMN_BGCAL_INIT_TMR),15,0,(uint32_t)0x0082);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_BGCAL_ITER_TMR__CMN_BGCAL_INIT_TMR),31,16,(uint32_t)0x0082);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_IBCAL_ITER_TMR__CMN_IBCAL_INIT_TMR),15,0,(uint32_t)0x001A);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_OVRD__CMN_PLL0_VCOCAL_TCTRL),15,0,(uint32_t)0x0003);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_ITER_TMR__CMN_PLL0_VCOCAL_INIT_TMR),15,0,(uint32_t)0x0104);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_ITER_TMR__CMN_PLL0_VCOCAL_INIT_TMR),31,16,(uint32_t)0x0005);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_REFTIM_START),15,0,(uint32_t)0x0337);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_PLLCNT_START),15,0,(uint32_t)0x3DBE);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_FRACDIVL_M0__CMN_PLL0_INTDIV_M0),15,0,(uint32_t)0x0180);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_FRACDIVL_M0__CMN_PLL0_INTDIV_M0),31,16,(uint32_t)0x9D8A);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_HIGH_THR_M0__CMN_PLL0_FRACDIVH_M0),15,0,(uint32_t)0x0002);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_HIGH_THR_M0__CMN_PLL0_FRACDIVH_M0),31,16,(uint32_t)0x0102);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_DSM_FBH_OVRD_M0__CMN_PLL0_DSM_DIAG_M0),15,0,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_SS_CTRL2_M0__CMN_PLL0_SS_CTRL1_M0),15,0,(uint32_t)0x0001);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_SS_CTRL2_M0__CMN_PLL0_SS_CTRL1_M0),31,16,(uint32_t)0x045F);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_SS_CTRL4_M0__CMN_PLL0_SS_CTRL3_M0),15,0,(uint32_t)0x006B);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_SS_CTRL4_M0__CMN_PLL0_SS_CTRL3_M0),31,16,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_LOCK_REFCNT_IDLE__CMN_PLL0_LOCK_REFCNT_START),15,0,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_LOCK_PLLCNT_THR__CMN_PLL0_LOCK_PLLCNT_START),15,0,(uint32_t)0x0192);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_LOCK_PLLCNT_THR__CMN_PLL0_LOCK_PLLCNT_START),31,16,(uint32_t)0x0006);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_FRACDIVL_M1__CMN_PLL0_INTDIV_M1),15,0,(uint32_t)0x0133);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_FRACDIVL_M1__CMN_PLL0_INTDIV_M1),31,16,(uint32_t)0xB13B);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_HIGH_THR_M1__CMN_PLL0_FRACDIVH_M1),15,0,(uint32_t)0x0002);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_HIGH_THR_M1__CMN_PLL0_FRACDIVH_M1),31,16,(uint32_t)0x00CE);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_DSM_FBH_OVRD_M1__CMN_PLL0_DSM_DIAG_M1),15,0,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_SS_CTRL2_M1__CMN_PLL0_SS_CTRL1_M1),15,0,(uint32_t)0x0001);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_SS_CTRL2_M1__CMN_PLL0_SS_CTRL1_M1),31,16,(uint32_t)0x0399);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_SS_CTRL4_M1__CMN_PLL0_SS_CTRL3_M1),15,0,(uint32_t)0x0068);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_SS_CTRL4_M1__CMN_PLL0_SS_CTRL3_M1),31,16,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_OVRD__CMN_PLL1_VCOCAL_TCTRL),15,0,(uint32_t)0x0003);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_ITER_TMR__CMN_PLL1_VCOCAL_INIT_TMR),15,0,(uint32_t)0x0104);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_ITER_TMR__CMN_PLL1_VCOCAL_INIT_TMR),31,16,(uint32_t)0x0005);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_REFTIM_START),15,0,(uint32_t)0x0337);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_PLLCNT_START),15,0,(uint32_t)0x3DBE);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_FRACDIVL_M0__CMN_PLL1_INTDIV_M0),15,0,(uint32_t)0x0133);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_FRACDIVL_M0__CMN_PLL1_INTDIV_M0),31,16,(uint32_t)0xB13B);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_HIGH_THR_M0__CMN_PLL1_FRACDIVH_M0),15,0,(uint32_t)0x0002);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_HIGH_THR_M0__CMN_PLL1_FRACDIVH_M0),31,16,(uint32_t)0x00CE);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_DSM_FBH_OVRD_M0__CMN_PLL1_DSM_DIAG_M0),15,0,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_SS_CTRL2_M0__CMN_PLL1_SS_CTRL1_M0),15,0,(uint32_t)0x0001);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_SS_CTRL2_M0__CMN_PLL1_SS_CTRL1_M0),31,16,(uint32_t)0x0399);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_SS_CTRL4_M0__CMN_PLL1_SS_CTRL3_M0),15,0,(uint32_t)0x0068);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_SS_CTRL4_M0__CMN_PLL1_SS_CTRL3_M0),31,16,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_LOCK_REFCNT_IDLE__CMN_PLL1_LOCK_REFCNT_START),15,0,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_LOCK_PLLCNT_THR__CMN_PLL1_LOCK_PLLCNT_START),15,0,(uint32_t)0x0192);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_LOCK_PLLCNT_THR__CMN_PLL1_LOCK_PLLCNT_START),31,16,(uint32_t)0x0006);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_TXPUCAL_ITER_TMR__CMN_TXPUCAL_INIT_TMR),15,0,(uint32_t)0x0020);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_TXPUCAL_ITER_TMR__CMN_TXPUCAL_INIT_TMR),31,16,(uint32_t)0x0007);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_TXPDCAL_ITER_TMR__CMN_TXPDCAL_INIT_TMR),15,0,(uint32_t)0x0020);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_TXPDCAL_ITER_TMR__CMN_TXPDCAL_INIT_TMR),31,16,(uint32_t)0x0007);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_RXCAL_ITER_TMR__CMN_RXCAL_INIT_TMR),15,0,(uint32_t)0x030C);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_RXCAL_ITER_TMR__CMN_RXCAL_INIT_TMR),31,16,(uint32_t)0x0007);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_SD_CAL_ITER_TMR__CMN_SD_CAL_INIT_TMR),15,0,(uint32_t)0x0007);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_SD_CAL_ITER_TMR__CMN_SD_CAL_INIT_TMR),31,16,(uint32_t)0x0003);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_SD_CAL_REFTIM_START),15,0,(uint32_t)0x000F);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_SD_CAL_PLLCNT_START),15,0,(uint32_t)0x0132);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CLK_SEL_M0__CMN_PDIAG_PLL0_CTRL_M0),15,0,(uint32_t)0x0022);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_IADJ_M0__CMN_PDIAG_PLL0_CP_PADJ_M0),15,0,(uint32_t)0x0509);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_IADJ_M0__CMN_PDIAG_PLL0_CP_PADJ_M0),31,16,(uint32_t)0x0F00);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_TUNE_M0__CMN_PDIAG_PLL0_FILT_PADJ_M0),15,0,(uint32_t)0x0F08);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CLK_SEL_M1__CMN_PDIAG_PLL0_CTRL_M1),15,0,(uint32_t)0x0022);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_IADJ_M1__CMN_PDIAG_PLL0_CP_PADJ_M1),15,0,(uint32_t)0x0509);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_IADJ_M1__CMN_PDIAG_PLL0_CP_PADJ_M1),31,16,(uint32_t)0x0F00);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_TUNE_M1__CMN_PDIAG_PLL0_FILT_PADJ_M1),15,0,(uint32_t)0x0F08);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CLK_SEL_M0__CMN_PDIAG_PLL1_CTRL_M0),15,0,(uint32_t)0x0022);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CP_IADJ_M0__CMN_PDIAG_PLL1_CP_PADJ_M0),15,0,(uint32_t)0x0509);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CP_IADJ_M0__CMN_PDIAG_PLL1_CP_PADJ_M0),31,16,(uint32_t)0x0F00);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CP_TUNE_M0__CMN_PDIAG_PLL1_FILT_PADJ_M0),15,0,(uint32_t)0x0F08);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[laneNum].DRV_DIAG_LANE_FCM_EN_SWAIT_TMR__DRV_DIAG_LANE_FCM_EN_TO),15,0,(uint32_t)0x0208);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[laneNum].DRV_DIAG_LANE_FCM_EN_TUNE__DRV_DIAG_LANE_FCM_EN_MGN_TMR),15,0,(uint32_t)0x009C);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[laneNum].TX_RCVDET_ST_TMR__TX_RCVDET_EN_TMR),31,16,(uint32_t)0x0A28);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_SDCAL0_ITER_TMR__RX_SDCAL0_INIT_TMR),15,0,(uint32_t)0x001A);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_SDCAL0_ITER_TMR__RX_SDCAL0_INIT_TMR),31,16,(uint32_t)0x0082);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_SDCAL1_ITER_TMR__RX_SDCAL1_INIT_TMR),15,0,(uint32_t)0x001A);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_SDCAL1_ITER_TMR__RX_SDCAL1_INIT_TMR),31,16,(uint32_t)0x0082);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_REE_TAP1_CLIP__RX_REE_ADDR_CFG),31,16,(uint32_t)0x0019);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_REE_CTRL_DATA_MASK__RX_REE_TAP2TON_CLIP),15,0,(uint32_t)0x0019);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_DIAG_ACYA__RX_DIAG_DCYA),31,16,(uint32_t)0x0001);

}

void csl_wiz16m_ct2_refclk26MHz_32b_PCIe_ext_ssc(uint32_t baseAddr, uint32_t laneNum)
{
    CSL_wiz16b8m4ct2Regs *torrent_sds_reg = (CSL_wiz16b8m4ct2Regs *)(uintptr_t)(baseAddr);

    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_SSM_BIAS_TMR),15,0,(uint32_t)0x001A);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLLSM0_PLLVREF_TMR__CMN_PLLSM0_PLLPRE_TMR),15,0,(uint32_t)0x0034);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLLSM0_PLLCLKDIS_TMR__CMN_PLLSM0_PLLLOCK_TMR),15,0,(uint32_t)0x00DA);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLLSM1_PLLVREF_TMR__CMN_PLLSM1_PLLPRE_TMR),15,0,(uint32_t)0x0034);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLLSM1_PLLCLKDIS_TMR__CMN_PLLSM1_PLLLOCK_TMR),15,0,(uint32_t)0x00DA);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_BGCAL_ITER_TMR__CMN_BGCAL_INIT_TMR),15,0,(uint32_t)0x0082);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_BGCAL_ITER_TMR__CMN_BGCAL_INIT_TMR),31,16,(uint32_t)0x0082);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_IBCAL_ITER_TMR__CMN_IBCAL_INIT_TMR),15,0,(uint32_t)0x001A);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_OVRD__CMN_PLL0_VCOCAL_TCTRL),15,0,(uint32_t)0x0003);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_ITER_TMR__CMN_PLL0_VCOCAL_INIT_TMR),15,0,(uint32_t)0x0104);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_ITER_TMR__CMN_PLL0_VCOCAL_INIT_TMR),31,16,(uint32_t)0x0005);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_REFTIM_START),15,0,(uint32_t)0x0338);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_PLLCNT_START),15,0,(uint32_t)0x3DE6);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_FRACDIVL_M0__CMN_PLL0_INTDIV_M0),15,0,(uint32_t)0x0180);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_FRACDIVL_M0__CMN_PLL0_INTDIV_M0),31,16,(uint32_t)0x9D8A);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_HIGH_THR_M0__CMN_PLL0_FRACDIVH_M0),15,0,(uint32_t)0x0002);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_HIGH_THR_M0__CMN_PLL0_FRACDIVH_M0),31,16,(uint32_t)0x0102);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_DSM_FBH_OVRD_M0__CMN_PLL0_DSM_DIAG_M0),15,0,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_LOCK_REFCNT_IDLE__CMN_PLL0_LOCK_REFCNT_START),15,0,(uint32_t)0x0015);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_LOCK_PLLCNT_THR__CMN_PLL0_LOCK_PLLCNT_START),15,0,(uint32_t)0x0194);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_LOCK_PLLCNT_THR__CMN_PLL0_LOCK_PLLCNT_START),31,16,(uint32_t)0x0005);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_FRACDIVL_M1__CMN_PLL0_INTDIV_M1),15,0,(uint32_t)0x0133);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_FRACDIVL_M1__CMN_PLL0_INTDIV_M1),31,16,(uint32_t)0xB13B);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_HIGH_THR_M1__CMN_PLL0_FRACDIVH_M1),15,0,(uint32_t)0x0002);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_HIGH_THR_M1__CMN_PLL0_FRACDIVH_M1),31,16,(uint32_t)0x00CE);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_DSM_FBH_OVRD_M1__CMN_PLL0_DSM_DIAG_M1),15,0,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_OVRD__CMN_PLL1_VCOCAL_TCTRL),15,0,(uint32_t)0x0003);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_ITER_TMR__CMN_PLL1_VCOCAL_INIT_TMR),15,0,(uint32_t)0x0104);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_ITER_TMR__CMN_PLL1_VCOCAL_INIT_TMR),31,16,(uint32_t)0x0005);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_REFTIM_START),15,0,(uint32_t)0x0338);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_PLLCNT_START),15,0,(uint32_t)0x3DE6);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_FRACDIVL_M0__CMN_PLL1_INTDIV_M0),15,0,(uint32_t)0x0133);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_FRACDIVL_M0__CMN_PLL1_INTDIV_M0),31,16,(uint32_t)0xB13B);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_HIGH_THR_M0__CMN_PLL1_FRACDIVH_M0),15,0,(uint32_t)0x0002);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_HIGH_THR_M0__CMN_PLL1_FRACDIVH_M0),31,16,(uint32_t)0x00CE);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_DSM_FBH_OVRD_M0__CMN_PLL1_DSM_DIAG_M0),15,0,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_LOCK_REFCNT_IDLE__CMN_PLL1_LOCK_REFCNT_START),15,0,(uint32_t)0x0015);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_LOCK_PLLCNT_THR__CMN_PLL1_LOCK_PLLCNT_START),15,0,(uint32_t)0x0194);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_LOCK_PLLCNT_THR__CMN_PLL1_LOCK_PLLCNT_START),31,16,(uint32_t)0x0005);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_TXPUCAL_ITER_TMR__CMN_TXPUCAL_INIT_TMR),15,0,(uint32_t)0x0020);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_TXPUCAL_ITER_TMR__CMN_TXPUCAL_INIT_TMR),31,16,(uint32_t)0x0007);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_TXPDCAL_ITER_TMR__CMN_TXPDCAL_INIT_TMR),15,0,(uint32_t)0x0020);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_TXPDCAL_ITER_TMR__CMN_TXPDCAL_INIT_TMR),31,16,(uint32_t)0x0007);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_RXCAL_ITER_TMR__CMN_RXCAL_INIT_TMR),15,0,(uint32_t)0x030C);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_RXCAL_ITER_TMR__CMN_RXCAL_INIT_TMR),31,16,(uint32_t)0x0007);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_SD_CAL_ITER_TMR__CMN_SD_CAL_INIT_TMR),15,0,(uint32_t)0x0007);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_SD_CAL_ITER_TMR__CMN_SD_CAL_INIT_TMR),31,16,(uint32_t)0x0003);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_SD_CAL_REFTIM_START),15,0,(uint32_t)0x000F);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_SD_CAL_PLLCNT_START),15,0,(uint32_t)0x0132);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CLK_SEL_M0__CMN_PDIAG_PLL0_CTRL_M0),15,0,(uint32_t)0x0022);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_IADJ_M0__CMN_PDIAG_PLL0_CP_PADJ_M0),15,0,(uint32_t)0x0509);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_IADJ_M0__CMN_PDIAG_PLL0_CP_PADJ_M0),31,16,(uint32_t)0x0F00);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_TUNE_M0__CMN_PDIAG_PLL0_FILT_PADJ_M0),15,0,(uint32_t)0x0F08);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CLK_SEL_M1__CMN_PDIAG_PLL0_CTRL_M1),15,0,(uint32_t)0x0022);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_IADJ_M1__CMN_PDIAG_PLL0_CP_PADJ_M1),15,0,(uint32_t)0x0509);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_IADJ_M1__CMN_PDIAG_PLL0_CP_PADJ_M1),31,16,(uint32_t)0x0F00);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_TUNE_M1__CMN_PDIAG_PLL0_FILT_PADJ_M1),15,0,(uint32_t)0x0F08);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CLK_SEL_M0__CMN_PDIAG_PLL1_CTRL_M0),15,0,(uint32_t)0x0022);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CP_IADJ_M0__CMN_PDIAG_PLL1_CP_PADJ_M0),15,0,(uint32_t)0x0509);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CP_IADJ_M0__CMN_PDIAG_PLL1_CP_PADJ_M0),31,16,(uint32_t)0x0F00);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CP_TUNE_M0__CMN_PDIAG_PLL1_FILT_PADJ_M0),15,0,(uint32_t)0x0F08);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[laneNum].DRV_DIAG_LANE_FCM_EN_SWAIT_TMR__DRV_DIAG_LANE_FCM_EN_TO),15,0,(uint32_t)0x0208);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[laneNum].DRV_DIAG_LANE_FCM_EN_TUNE__DRV_DIAG_LANE_FCM_EN_MGN_TMR),15,0,(uint32_t)0x009C);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[laneNum].TX_RCVDET_ST_TMR__TX_RCVDET_EN_TMR),31,16,(uint32_t)0x0A28);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_SDCAL0_ITER_TMR__RX_SDCAL0_INIT_TMR),15,0,(uint32_t)0x001A);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_SDCAL0_ITER_TMR__RX_SDCAL0_INIT_TMR),31,16,(uint32_t)0x0082);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_SDCAL1_ITER_TMR__RX_SDCAL1_INIT_TMR),15,0,(uint32_t)0x001A);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_SDCAL1_ITER_TMR__RX_SDCAL1_INIT_TMR),31,16,(uint32_t)0x0082);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_REE_TAP1_CLIP__RX_REE_ADDR_CFG),31,16,(uint32_t)0x0019);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_REE_CTRL_DATA_MASK__RX_REE_TAP2TON_CLIP),15,0,(uint32_t)0x0019);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_DIAG_ACYA__RX_DIAG_DCYA),31,16,(uint32_t)0x0001);

}

void csl_wiz16m_ct2_refclk27MHz_32b_PCIe_no_ssc(uint32_t baseAddr, uint32_t laneNum)
{
    CSL_wiz16b8m4ct2Regs *torrent_sds_reg = (CSL_wiz16b8m4ct2Regs *)(uintptr_t)(baseAddr);

    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_SSM_BIAS_TMR),15,0,(uint32_t)0x001B);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLLSM0_PLLVREF_TMR__CMN_PLLSM0_PLLPRE_TMR),15,0,(uint32_t)0x0036);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLLSM0_PLLCLKDIS_TMR__CMN_PLLSM0_PLLLOCK_TMR),15,0,(uint32_t)0x00E2);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLLSM1_PLLVREF_TMR__CMN_PLLSM1_PLLPRE_TMR),15,0,(uint32_t)0x0036);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLLSM1_PLLCLKDIS_TMR__CMN_PLLSM1_PLLLOCK_TMR),15,0,(uint32_t)0x00E2);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_BGCAL_ITER_TMR__CMN_BGCAL_INIT_TMR),15,0,(uint32_t)0x0087);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_BGCAL_ITER_TMR__CMN_BGCAL_INIT_TMR),31,16,(uint32_t)0x0087);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_IBCAL_ITER_TMR__CMN_IBCAL_INIT_TMR),15,0,(uint32_t)0x001B);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_OVRD__CMN_PLL0_VCOCAL_TCTRL),15,0,(uint32_t)0x0003);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_ITER_TMR__CMN_PLL0_VCOCAL_INIT_TMR),15,0,(uint32_t)0x010E);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_ITER_TMR__CMN_PLL0_VCOCAL_INIT_TMR),31,16,(uint32_t)0x0005);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_REFTIM_START),15,0,(uint32_t)0x0358);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_PLLCNT_START),15,0,(uint32_t)0x3DEC);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_FRACDIVL_M0__CMN_PLL0_INTDIV_M0),15,0,(uint32_t)0x0172);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_FRACDIVL_M0__CMN_PLL0_INTDIV_M0),31,16,(uint32_t)0x5ED1);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_HIGH_THR_M0__CMN_PLL0_FRACDIVH_M0),15,0,(uint32_t)0x0002);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_HIGH_THR_M0__CMN_PLL0_FRACDIVH_M0),31,16,(uint32_t)0x00F8);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_DSM_FBH_OVRD_M0__CMN_PLL0_DSM_DIAG_M0),15,0,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_LOCK_REFCNT_IDLE__CMN_PLL0_LOCK_REFCNT_START),15,0,(uint32_t)0x0016);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_LOCK_PLLCNT_THR__CMN_PLL0_LOCK_PLLCNT_START),15,0,(uint32_t)0x0197);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_LOCK_PLLCNT_THR__CMN_PLL0_LOCK_PLLCNT_START),31,16,(uint32_t)0x0005);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_FRACDIVL_M1__CMN_PLL0_INTDIV_M1),15,0,(uint32_t)0x0128);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_FRACDIVL_M1__CMN_PLL0_INTDIV_M1),31,16,(uint32_t)0x4BDA);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_HIGH_THR_M1__CMN_PLL0_FRACDIVH_M1),15,0,(uint32_t)0x0002);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_HIGH_THR_M1__CMN_PLL0_FRACDIVH_M1),31,16,(uint32_t)0x00C6);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_DSM_FBH_OVRD_M1__CMN_PLL0_DSM_DIAG_M1),15,0,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_OVRD__CMN_PLL1_VCOCAL_TCTRL),15,0,(uint32_t)0x0003);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_ITER_TMR__CMN_PLL1_VCOCAL_INIT_TMR),15,0,(uint32_t)0x010E);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_ITER_TMR__CMN_PLL1_VCOCAL_INIT_TMR),31,16,(uint32_t)0x0005);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_REFTIM_START),15,0,(uint32_t)0x0358);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_PLLCNT_START),15,0,(uint32_t)0x3DEC);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_FRACDIVL_M0__CMN_PLL1_INTDIV_M0),15,0,(uint32_t)0x0128);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_FRACDIVL_M0__CMN_PLL1_INTDIV_M0),31,16,(uint32_t)0x4BDA);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_HIGH_THR_M0__CMN_PLL1_FRACDIVH_M0),15,0,(uint32_t)0x0002);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_HIGH_THR_M0__CMN_PLL1_FRACDIVH_M0),31,16,(uint32_t)0x00C6);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_DSM_FBH_OVRD_M0__CMN_PLL1_DSM_DIAG_M0),15,0,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_LOCK_REFCNT_IDLE__CMN_PLL1_LOCK_REFCNT_START),15,0,(uint32_t)0x0016);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_LOCK_PLLCNT_THR__CMN_PLL1_LOCK_PLLCNT_START),15,0,(uint32_t)0x0197);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_LOCK_PLLCNT_THR__CMN_PLL1_LOCK_PLLCNT_START),31,16,(uint32_t)0x0005);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_TXPUCAL_ITER_TMR__CMN_TXPUCAL_INIT_TMR),15,0,(uint32_t)0x0021);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_TXPUCAL_ITER_TMR__CMN_TXPUCAL_INIT_TMR),31,16,(uint32_t)0x0007);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_TXPDCAL_ITER_TMR__CMN_TXPDCAL_INIT_TMR),15,0,(uint32_t)0x0021);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_TXPDCAL_ITER_TMR__CMN_TXPDCAL_INIT_TMR),31,16,(uint32_t)0x0007);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_RXCAL_ITER_TMR__CMN_RXCAL_INIT_TMR),15,0,(uint32_t)0x032A);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_RXCAL_ITER_TMR__CMN_RXCAL_INIT_TMR),31,16,(uint32_t)0x0007);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_SD_CAL_ITER_TMR__CMN_SD_CAL_INIT_TMR),15,0,(uint32_t)0x0007);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_SD_CAL_ITER_TMR__CMN_SD_CAL_INIT_TMR),31,16,(uint32_t)0x0003);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_SD_CAL_REFTIM_START),15,0,(uint32_t)0x0010);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_SD_CAL_PLLCNT_START),15,0,(uint32_t)0x0139);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CLK_SEL_M0__CMN_PDIAG_PLL0_CTRL_M0),15,0,(uint32_t)0x0022);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_IADJ_M0__CMN_PDIAG_PLL0_CP_PADJ_M0),15,0,(uint32_t)0x0509);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_IADJ_M0__CMN_PDIAG_PLL0_CP_PADJ_M0),31,16,(uint32_t)0x0F00);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_TUNE_M0__CMN_PDIAG_PLL0_FILT_PADJ_M0),15,0,(uint32_t)0x0F08);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CLK_SEL_M1__CMN_PDIAG_PLL0_CTRL_M1),15,0,(uint32_t)0x0022);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_IADJ_M1__CMN_PDIAG_PLL0_CP_PADJ_M1),15,0,(uint32_t)0x0509);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_IADJ_M1__CMN_PDIAG_PLL0_CP_PADJ_M1),31,16,(uint32_t)0x0F00);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_TUNE_M1__CMN_PDIAG_PLL0_FILT_PADJ_M1),15,0,(uint32_t)0x0F08);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CLK_SEL_M0__CMN_PDIAG_PLL1_CTRL_M0),15,0,(uint32_t)0x0022);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CP_IADJ_M0__CMN_PDIAG_PLL1_CP_PADJ_M0),15,0,(uint32_t)0x0509);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CP_IADJ_M0__CMN_PDIAG_PLL1_CP_PADJ_M0),31,16,(uint32_t)0x0F00);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CP_TUNE_M0__CMN_PDIAG_PLL1_FILT_PADJ_M0),15,0,(uint32_t)0x0F08);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[laneNum].DRV_DIAG_LANE_FCM_EN_SWAIT_TMR__DRV_DIAG_LANE_FCM_EN_TO),15,0,(uint32_t)0x021C);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[laneNum].DRV_DIAG_LANE_FCM_EN_TUNE__DRV_DIAG_LANE_FCM_EN_MGN_TMR),15,0,(uint32_t)0x00A2);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[laneNum].TX_RCVDET_ST_TMR__TX_RCVDET_EN_TMR),31,16,(uint32_t)0x0A8C);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_SDCAL0_ITER_TMR__RX_SDCAL0_INIT_TMR),15,0,(uint32_t)0x001B);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_SDCAL0_ITER_TMR__RX_SDCAL0_INIT_TMR),31,16,(uint32_t)0x0087);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_SDCAL1_ITER_TMR__RX_SDCAL1_INIT_TMR),15,0,(uint32_t)0x001B);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_SDCAL1_ITER_TMR__RX_SDCAL1_INIT_TMR),31,16,(uint32_t)0x0087);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_REE_TAP1_CLIP__RX_REE_ADDR_CFG),31,16,(uint32_t)0x0019);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_REE_CTRL_DATA_MASK__RX_REE_TAP2TON_CLIP),15,0,(uint32_t)0x0019);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_DIAG_ACYA__RX_DIAG_DCYA),31,16,(uint32_t)0x0001);

}

void csl_wiz16m_ct2_refclk27MHz_32b_PCIe_int_ssc(uint32_t baseAddr, uint32_t laneNum)
{
    CSL_wiz16b8m4ct2Regs *torrent_sds_reg = (CSL_wiz16b8m4ct2Regs *)(uintptr_t)(baseAddr);

    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_SSM_BIAS_TMR),15,0,(uint32_t)0x001B);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLLSM0_PLLVREF_TMR__CMN_PLLSM0_PLLPRE_TMR),15,0,(uint32_t)0x0036);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLLSM0_PLLCLKDIS_TMR__CMN_PLLSM0_PLLLOCK_TMR),15,0,(uint32_t)0x00E2);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLLSM1_PLLVREF_TMR__CMN_PLLSM1_PLLPRE_TMR),15,0,(uint32_t)0x0036);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLLSM1_PLLCLKDIS_TMR__CMN_PLLSM1_PLLLOCK_TMR),15,0,(uint32_t)0x00E2);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_BGCAL_ITER_TMR__CMN_BGCAL_INIT_TMR),15,0,(uint32_t)0x0087);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_BGCAL_ITER_TMR__CMN_BGCAL_INIT_TMR),31,16,(uint32_t)0x0087);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_IBCAL_ITER_TMR__CMN_IBCAL_INIT_TMR),15,0,(uint32_t)0x001B);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_OVRD__CMN_PLL0_VCOCAL_TCTRL),15,0,(uint32_t)0x0003);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_ITER_TMR__CMN_PLL0_VCOCAL_INIT_TMR),15,0,(uint32_t)0x010E);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_ITER_TMR__CMN_PLL0_VCOCAL_INIT_TMR),31,16,(uint32_t)0x0005);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_REFTIM_START),15,0,(uint32_t)0x0357);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_PLLCNT_START),15,0,(uint32_t)0x3DC3);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_FRACDIVL_M0__CMN_PLL0_INTDIV_M0),15,0,(uint32_t)0x0172);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_FRACDIVL_M0__CMN_PLL0_INTDIV_M0),31,16,(uint32_t)0x5ED1);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_HIGH_THR_M0__CMN_PLL0_FRACDIVH_M0),15,0,(uint32_t)0x0002);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_HIGH_THR_M0__CMN_PLL0_FRACDIVH_M0),31,16,(uint32_t)0x00F8);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_DSM_FBH_OVRD_M0__CMN_PLL0_DSM_DIAG_M0),15,0,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_SS_CTRL2_M0__CMN_PLL0_SS_CTRL1_M0),15,0,(uint32_t)0x0001);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_SS_CTRL2_M0__CMN_PLL0_SS_CTRL1_M0),31,16,(uint32_t)0x044A);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_SS_CTRL4_M0__CMN_PLL0_SS_CTRL3_M0),15,0,(uint32_t)0x0069);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_SS_CTRL4_M0__CMN_PLL0_SS_CTRL3_M0),31,16,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_LOCK_REFCNT_IDLE__CMN_PLL0_LOCK_REFCNT_START),15,0,(uint32_t)0x0015);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_LOCK_PLLCNT_THR__CMN_PLL0_LOCK_PLLCNT_START),15,0,(uint32_t)0x0195);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_LOCK_PLLCNT_THR__CMN_PLL0_LOCK_PLLCNT_START),31,16,(uint32_t)0x0006);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_FRACDIVL_M1__CMN_PLL0_INTDIV_M1),15,0,(uint32_t)0x0128);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_FRACDIVL_M1__CMN_PLL0_INTDIV_M1),31,16,(uint32_t)0x4BDA);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_HIGH_THR_M1__CMN_PLL0_FRACDIVH_M1),15,0,(uint32_t)0x0002);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_HIGH_THR_M1__CMN_PLL0_FRACDIVH_M1),31,16,(uint32_t)0x00C6);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_DSM_FBH_OVRD_M1__CMN_PLL0_DSM_DIAG_M1),15,0,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_SS_CTRL2_M1__CMN_PLL0_SS_CTRL1_M1),15,0,(uint32_t)0x0001);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_SS_CTRL2_M1__CMN_PLL0_SS_CTRL1_M1),31,16,(uint32_t)0x033F);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_SS_CTRL4_M1__CMN_PLL0_SS_CTRL3_M1),15,0,(uint32_t)0x006F);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_SS_CTRL4_M1__CMN_PLL0_SS_CTRL3_M1),31,16,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_OVRD__CMN_PLL1_VCOCAL_TCTRL),15,0,(uint32_t)0x0003);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_ITER_TMR__CMN_PLL1_VCOCAL_INIT_TMR),15,0,(uint32_t)0x010E);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_ITER_TMR__CMN_PLL1_VCOCAL_INIT_TMR),31,16,(uint32_t)0x0005);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_REFTIM_START),15,0,(uint32_t)0x0357);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_PLLCNT_START),15,0,(uint32_t)0x3DC3);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_FRACDIVL_M0__CMN_PLL1_INTDIV_M0),15,0,(uint32_t)0x0128);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_FRACDIVL_M0__CMN_PLL1_INTDIV_M0),31,16,(uint32_t)0x4BDA);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_HIGH_THR_M0__CMN_PLL1_FRACDIVH_M0),15,0,(uint32_t)0x0002);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_HIGH_THR_M0__CMN_PLL1_FRACDIVH_M0),31,16,(uint32_t)0x00C6);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_DSM_FBH_OVRD_M0__CMN_PLL1_DSM_DIAG_M0),15,0,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_SS_CTRL2_M0__CMN_PLL1_SS_CTRL1_M0),15,0,(uint32_t)0x0001);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_SS_CTRL2_M0__CMN_PLL1_SS_CTRL1_M0),31,16,(uint32_t)0x033F);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_SS_CTRL4_M0__CMN_PLL1_SS_CTRL3_M0),15,0,(uint32_t)0x006F);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_SS_CTRL4_M0__CMN_PLL1_SS_CTRL3_M0),31,16,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_LOCK_REFCNT_IDLE__CMN_PLL1_LOCK_REFCNT_START),15,0,(uint32_t)0x0015);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_LOCK_PLLCNT_THR__CMN_PLL1_LOCK_PLLCNT_START),15,0,(uint32_t)0x0195);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_LOCK_PLLCNT_THR__CMN_PLL1_LOCK_PLLCNT_START),31,16,(uint32_t)0x0006);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_TXPUCAL_ITER_TMR__CMN_TXPUCAL_INIT_TMR),15,0,(uint32_t)0x0021);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_TXPUCAL_ITER_TMR__CMN_TXPUCAL_INIT_TMR),31,16,(uint32_t)0x0007);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_TXPDCAL_ITER_TMR__CMN_TXPDCAL_INIT_TMR),15,0,(uint32_t)0x0021);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_TXPDCAL_ITER_TMR__CMN_TXPDCAL_INIT_TMR),31,16,(uint32_t)0x0007);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_RXCAL_ITER_TMR__CMN_RXCAL_INIT_TMR),15,0,(uint32_t)0x032A);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_RXCAL_ITER_TMR__CMN_RXCAL_INIT_TMR),31,16,(uint32_t)0x0007);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_SD_CAL_ITER_TMR__CMN_SD_CAL_INIT_TMR),15,0,(uint32_t)0x0007);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_SD_CAL_ITER_TMR__CMN_SD_CAL_INIT_TMR),31,16,(uint32_t)0x0003);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_SD_CAL_REFTIM_START),15,0,(uint32_t)0x0010);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_SD_CAL_PLLCNT_START),15,0,(uint32_t)0x0139);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CLK_SEL_M0__CMN_PDIAG_PLL0_CTRL_M0),15,0,(uint32_t)0x0022);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_IADJ_M0__CMN_PDIAG_PLL0_CP_PADJ_M0),15,0,(uint32_t)0x0509);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_IADJ_M0__CMN_PDIAG_PLL0_CP_PADJ_M0),31,16,(uint32_t)0x0F00);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_TUNE_M0__CMN_PDIAG_PLL0_FILT_PADJ_M0),15,0,(uint32_t)0x0F08);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CLK_SEL_M1__CMN_PDIAG_PLL0_CTRL_M1),15,0,(uint32_t)0x0022);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_IADJ_M1__CMN_PDIAG_PLL0_CP_PADJ_M1),15,0,(uint32_t)0x0509);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_IADJ_M1__CMN_PDIAG_PLL0_CP_PADJ_M1),31,16,(uint32_t)0x0F00);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_TUNE_M1__CMN_PDIAG_PLL0_FILT_PADJ_M1),15,0,(uint32_t)0x0F08);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CLK_SEL_M0__CMN_PDIAG_PLL1_CTRL_M0),15,0,(uint32_t)0x0022);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CP_IADJ_M0__CMN_PDIAG_PLL1_CP_PADJ_M0),15,0,(uint32_t)0x0509);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CP_IADJ_M0__CMN_PDIAG_PLL1_CP_PADJ_M0),31,16,(uint32_t)0x0F00);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CP_TUNE_M0__CMN_PDIAG_PLL1_FILT_PADJ_M0),15,0,(uint32_t)0x0F08);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[laneNum].DRV_DIAG_LANE_FCM_EN_SWAIT_TMR__DRV_DIAG_LANE_FCM_EN_TO),15,0,(uint32_t)0x021C);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[laneNum].DRV_DIAG_LANE_FCM_EN_TUNE__DRV_DIAG_LANE_FCM_EN_MGN_TMR),15,0,(uint32_t)0x00A2);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[laneNum].TX_RCVDET_ST_TMR__TX_RCVDET_EN_TMR),31,16,(uint32_t)0x0A8C);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_SDCAL0_ITER_TMR__RX_SDCAL0_INIT_TMR),15,0,(uint32_t)0x001B);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_SDCAL0_ITER_TMR__RX_SDCAL0_INIT_TMR),31,16,(uint32_t)0x0087);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_SDCAL1_ITER_TMR__RX_SDCAL1_INIT_TMR),15,0,(uint32_t)0x001B);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_SDCAL1_ITER_TMR__RX_SDCAL1_INIT_TMR),31,16,(uint32_t)0x0087);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_REE_TAP1_CLIP__RX_REE_ADDR_CFG),31,16,(uint32_t)0x0019);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_REE_CTRL_DATA_MASK__RX_REE_TAP2TON_CLIP),15,0,(uint32_t)0x0019);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_DIAG_ACYA__RX_DIAG_DCYA),31,16,(uint32_t)0x0001);

}

void csl_wiz16m_ct2_refclk27MHz_32b_PCIe_ext_ssc(uint32_t baseAddr, uint32_t laneNum)
{
    CSL_wiz16b8m4ct2Regs *torrent_sds_reg = (CSL_wiz16b8m4ct2Regs *)(uintptr_t)(baseAddr);

    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_SSM_BIAS_TMR),15,0,(uint32_t)0x001B);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLLSM0_PLLVREF_TMR__CMN_PLLSM0_PLLPRE_TMR),15,0,(uint32_t)0x0036);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLLSM0_PLLCLKDIS_TMR__CMN_PLLSM0_PLLLOCK_TMR),15,0,(uint32_t)0x00E2);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLLSM1_PLLVREF_TMR__CMN_PLLSM1_PLLPRE_TMR),15,0,(uint32_t)0x0036);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLLSM1_PLLCLKDIS_TMR__CMN_PLLSM1_PLLLOCK_TMR),15,0,(uint32_t)0x00E2);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_BGCAL_ITER_TMR__CMN_BGCAL_INIT_TMR),15,0,(uint32_t)0x0087);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_BGCAL_ITER_TMR__CMN_BGCAL_INIT_TMR),31,16,(uint32_t)0x0087);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_IBCAL_ITER_TMR__CMN_IBCAL_INIT_TMR),15,0,(uint32_t)0x001B);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_OVRD__CMN_PLL0_VCOCAL_TCTRL),15,0,(uint32_t)0x0003);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_ITER_TMR__CMN_PLL0_VCOCAL_INIT_TMR),15,0,(uint32_t)0x010E);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_ITER_TMR__CMN_PLL0_VCOCAL_INIT_TMR),31,16,(uint32_t)0x0005);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_REFTIM_START),15,0,(uint32_t)0x0358);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_VCOCAL_PLLCNT_START),15,0,(uint32_t)0x3DEC);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_FRACDIVL_M0__CMN_PLL0_INTDIV_M0),15,0,(uint32_t)0x0172);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_FRACDIVL_M0__CMN_PLL0_INTDIV_M0),31,16,(uint32_t)0x5ED1);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_HIGH_THR_M0__CMN_PLL0_FRACDIVH_M0),15,0,(uint32_t)0x0002);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_HIGH_THR_M0__CMN_PLL0_FRACDIVH_M0),31,16,(uint32_t)0x00F8);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_DSM_FBH_OVRD_M0__CMN_PLL0_DSM_DIAG_M0),15,0,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_LOCK_REFCNT_IDLE__CMN_PLL0_LOCK_REFCNT_START),15,0,(uint32_t)0x0016);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_LOCK_PLLCNT_THR__CMN_PLL0_LOCK_PLLCNT_START),15,0,(uint32_t)0x0197);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_LOCK_PLLCNT_THR__CMN_PLL0_LOCK_PLLCNT_START),31,16,(uint32_t)0x0005);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_FRACDIVL_M1__CMN_PLL0_INTDIV_M1),15,0,(uint32_t)0x0128);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_FRACDIVL_M1__CMN_PLL0_INTDIV_M1),31,16,(uint32_t)0x4BDA);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_HIGH_THR_M1__CMN_PLL0_FRACDIVH_M1),15,0,(uint32_t)0x0002);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_HIGH_THR_M1__CMN_PLL0_FRACDIVH_M1),31,16,(uint32_t)0x00C6);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL0_DSM_FBH_OVRD_M1__CMN_PLL0_DSM_DIAG_M1),15,0,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_OVRD__CMN_PLL1_VCOCAL_TCTRL),15,0,(uint32_t)0x0003);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_ITER_TMR__CMN_PLL1_VCOCAL_INIT_TMR),15,0,(uint32_t)0x010E);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_ITER_TMR__CMN_PLL1_VCOCAL_INIT_TMR),31,16,(uint32_t)0x0005);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_REFTIM_START),15,0,(uint32_t)0x0358);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_VCOCAL_PLLCNT_START),15,0,(uint32_t)0x3DEC);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_FRACDIVL_M0__CMN_PLL1_INTDIV_M0),15,0,(uint32_t)0x0128);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_FRACDIVL_M0__CMN_PLL1_INTDIV_M0),31,16,(uint32_t)0x4BDA);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_HIGH_THR_M0__CMN_PLL1_FRACDIVH_M0),15,0,(uint32_t)0x0002);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_HIGH_THR_M0__CMN_PLL1_FRACDIVH_M0),31,16,(uint32_t)0x00C6);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_DSM_FBH_OVRD_M0__CMN_PLL1_DSM_DIAG_M0),15,0,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_LOCK_REFCNT_IDLE__CMN_PLL1_LOCK_REFCNT_START),15,0,(uint32_t)0x0016);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_LOCK_PLLCNT_THR__CMN_PLL1_LOCK_PLLCNT_START),15,0,(uint32_t)0x0197);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLL1_LOCK_PLLCNT_THR__CMN_PLL1_LOCK_PLLCNT_START),31,16,(uint32_t)0x0005);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_TXPUCAL_ITER_TMR__CMN_TXPUCAL_INIT_TMR),15,0,(uint32_t)0x0021);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_TXPUCAL_ITER_TMR__CMN_TXPUCAL_INIT_TMR),31,16,(uint32_t)0x0007);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_TXPDCAL_ITER_TMR__CMN_TXPDCAL_INIT_TMR),15,0,(uint32_t)0x0021);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_TXPDCAL_ITER_TMR__CMN_TXPDCAL_INIT_TMR),31,16,(uint32_t)0x0007);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_RXCAL_ITER_TMR__CMN_RXCAL_INIT_TMR),15,0,(uint32_t)0x032A);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_RXCAL_ITER_TMR__CMN_RXCAL_INIT_TMR),31,16,(uint32_t)0x0007);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_SD_CAL_ITER_TMR__CMN_SD_CAL_INIT_TMR),15,0,(uint32_t)0x0007);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_SD_CAL_ITER_TMR__CMN_SD_CAL_INIT_TMR),31,16,(uint32_t)0x0003);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_SD_CAL_REFTIM_START),15,0,(uint32_t)0x0010);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_SD_CAL_PLLCNT_START),15,0,(uint32_t)0x0139);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CLK_SEL_M0__CMN_PDIAG_PLL0_CTRL_M0),15,0,(uint32_t)0x0022);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_IADJ_M0__CMN_PDIAG_PLL0_CP_PADJ_M0),15,0,(uint32_t)0x0509);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_IADJ_M0__CMN_PDIAG_PLL0_CP_PADJ_M0),31,16,(uint32_t)0x0F00);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_TUNE_M0__CMN_PDIAG_PLL0_FILT_PADJ_M0),15,0,(uint32_t)0x0F08);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CLK_SEL_M1__CMN_PDIAG_PLL0_CTRL_M1),15,0,(uint32_t)0x0022);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_IADJ_M1__CMN_PDIAG_PLL0_CP_PADJ_M1),15,0,(uint32_t)0x0509);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_IADJ_M1__CMN_PDIAG_PLL0_CP_PADJ_M1),31,16,(uint32_t)0x0F00);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL0_CP_TUNE_M1__CMN_PDIAG_PLL0_FILT_PADJ_M1),15,0,(uint32_t)0x0F08);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CLK_SEL_M0__CMN_PDIAG_PLL1_CTRL_M0),15,0,(uint32_t)0x0022);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CP_IADJ_M0__CMN_PDIAG_PLL1_CP_PADJ_M0),15,0,(uint32_t)0x0509);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CP_IADJ_M0__CMN_PDIAG_PLL1_CP_PADJ_M0),31,16,(uint32_t)0x0F00);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PDIAG_PLL1_CP_TUNE_M0__CMN_PDIAG_PLL1_FILT_PADJ_M0),15,0,(uint32_t)0x0F08);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[laneNum].DRV_DIAG_LANE_FCM_EN_SWAIT_TMR__DRV_DIAG_LANE_FCM_EN_TO),15,0,(uint32_t)0x021C);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[laneNum].DRV_DIAG_LANE_FCM_EN_TUNE__DRV_DIAG_LANE_FCM_EN_MGN_TMR),15,0,(uint32_t)0x00A2);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[laneNum].TX_RCVDET_ST_TMR__TX_RCVDET_EN_TMR),31,16,(uint32_t)0x0A8C);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_SDCAL0_ITER_TMR__RX_SDCAL0_INIT_TMR),15,0,(uint32_t)0x001B);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_SDCAL0_ITER_TMR__RX_SDCAL0_INIT_TMR),31,16,(uint32_t)0x0087);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_SDCAL1_ITER_TMR__RX_SDCAL1_INIT_TMR),15,0,(uint32_t)0x001B);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_SDCAL1_ITER_TMR__RX_SDCAL1_INIT_TMR),31,16,(uint32_t)0x0087);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_REE_TAP1_CLIP__RX_REE_ADDR_CFG),31,16,(uint32_t)0x0019);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_REE_CTRL_DATA_MASK__RX_REE_TAP2TON_CLIP),15,0,(uint32_t)0x0019);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_DIAG_ACYA__RX_DIAG_DCYA),31,16,(uint32_t)0x0001);

}

#endif
