/**
* File: csl_wiz16m_ct_refclk156p25MHz_refclk_related.c
*
*  no_pcie
*  refclk_cmn
*
*  ============================================================================
*  (C) Copyright 2018, Texas Instruments, Inc.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*    Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the
*    distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/
#ifndef CSL_WIZ16M_CT_REFCLK156p25MHz_REFCLK_CMN_C
#define CSL_WIZ16M_CT_REFCLK156p25MHz_REFCLK_CMN_C

#include <stdint.h>
#include <drivers/hw_include/cslr_soc.h>

#include <serdes_cd/V0/cslr_wiz16b8m4ct.h>

void csl_wiz16m_ct_refclk156p25MHz_refclk_related(uint32_t baseAddr, uint32_t laneNum)
{
    CSL_wiz16b8m4ctRegs *torrent_sds_reg = (CSL_wiz16b8m4ctRegs *)(uintptr_t)(baseAddr);

    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_SSM_BANDGAP_TMR__CMN_SSM_SM_CTRL),31,16,(uint32_t)0x000A);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_SSM_BIAS_TMR),15,0,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLLSM0_PLLVREF_TMR__CMN_PLLSM0_PLLPRE_TMR),15,0,(uint32_t)0x0028);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLLSM0_PLLCLKDIS_TMR__CMN_PLLSM0_PLLLOCK_TMR),15,0,(uint32_t)0x00A4);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLLSM1_PLLVREF_TMR__CMN_PLLSM1_PLLPRE_TMR),15,0,(uint32_t)0x0028);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_PLLSM1_PLLCLKDIS_TMR__CMN_PLLSM1_PLLLOCK_TMR),15,0,(uint32_t)0x00A4);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_BGCAL_ITER_TMR__CMN_BGCAL_INIT_TMR),15,0,(uint32_t)0x0062);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_BGCAL_ITER_TMR__CMN_BGCAL_INIT_TMR),31,16,(uint32_t)0x0062);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_IBCAL_ITER_TMR__CMN_IBCAL_INIT_TMR),15,0,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_TXPUCAL_ITER_TMR__CMN_TXPUCAL_INIT_TMR),15,0,(uint32_t)0x0018);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_TXPUCAL_ITER_TMR__CMN_TXPUCAL_INIT_TMR),31,16,(uint32_t)0x0005);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_TXPDCAL_ITER_TMR__CMN_TXPDCAL_INIT_TMR),15,0,(uint32_t)0x0018);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_TXPDCAL_ITER_TMR__CMN_TXPDCAL_INIT_TMR),31,16,(uint32_t)0x0005);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_RXCAL_ITER_TMR__CMN_RXCAL_INIT_TMR),15,0,(uint32_t)0x024A);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_RXCAL_ITER_TMR__CMN_RXCAL_INIT_TMR),31,16,(uint32_t)0x0005);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_SD_CAL_ITER_TMR__CMN_SD_CAL_INIT_TMR),15,0,(uint32_t)0x0002);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_SD_CAL_ITER_TMR__CMN_SD_CAL_INIT_TMR),31,16,(uint32_t)0x0002);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_SD_CAL_REFTIM_START),15,0,(uint32_t)0x000B);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_SD_CAL_PLLCNT_START),15,0,(uint32_t)0x0132);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_SDCAL0_ITER_TMR__RX_SDCAL0_INIT_TMR),15,0,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_SDCAL0_ITER_TMR__RX_SDCAL0_INIT_TMR),31,16,(uint32_t)0x0062);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_SDCAL1_ITER_TMR__RX_SDCAL1_INIT_TMR),15,0,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_RX_LANE_REGISTERS[laneNum].RX_SDCAL1_ITER_TMR__RX_SDCAL1_INIT_TMR),31,16,(uint32_t)0x0062);
    CSL_FINSR(*(volatile uint32_t *)(&torrent_sds_reg->PMA_TX_LANE_REGISTERS[laneNum].TX_RCVDET_ST_TMR__TX_RCVDET_EN_TMR),31,16,(uint32_t)0x07A2);
}
#endif

