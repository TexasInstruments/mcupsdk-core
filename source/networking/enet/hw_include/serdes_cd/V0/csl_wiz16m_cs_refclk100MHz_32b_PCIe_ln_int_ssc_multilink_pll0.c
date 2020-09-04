/**
* File: csl_wiz16m_cs_refclk100MHz_32b_PCIe_ln_int_ssc_multilink_pll0.c
*
*  int_ssc
*  gen4_opt2
*  using_plllc
*  cmn_pllcy_anaclk0_1000mhz
*  xcvr_pllclk_fullrt_500mhz
*  multilink
*
*  ============================================================================
*  (C) Copyright 2019, Texas Instruments, Inc.
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
#ifndef CSL_WIZ16M_CS_REFCLK100MHz_32b_PCIe_LN_C
#define CSL_WIZ16M_CS_REFCLK100MHz_32b_PCIe_LN_C

#include <stdint.h>
#include <drivers/hw_include/cslr_soc.h>

#include <serdes_cd/V0/cslr_wiz16b4m4cs.h>

void csl_wiz16m_cs_refclk100MHz_32b_PCIe_ln_int_ssc_multilink_pll0(uint32_t baseAddr, uint32_t laneNum)
{
    CSL_wiz16b4m4csRegs *sierra_sds_reg = (CSL_wiz16b4m4csRegs *)(uintptr_t)(baseAddr);

    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].PSC_LN_A3_PREG__PSC_LN_A2_PREG),31,16,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].PSC_LN_A5_PREG__PSC_LN_A4_PREG),15,0,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].PSC_LN_IDLE_PREG),15,0,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].CLKPATHCTRL_TMR_PREG__CLKPATHCTRL_OVR_PREG),31,16,(uint32_t)0x813E);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].RX_CREQ_FLTR_A_MODE3_PREG),31,16,(uint32_t)0x8047);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].RX_CREQ_FLTR_A_MODE1_PREG__RX_CREQ_FLTR_A_MODE2_PREG),15,0,(uint32_t)0x808F);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].RX_CREQ_FLTR_A_MODE1_PREG__RX_CREQ_FLTR_A_MODE2_PREG),31,16,(uint32_t)0x808F);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].RX_CREQ_FLTR_B_PREG__RX_CREQ_FLTR_A_MODE0_PREG),15,0,(uint32_t)0x808F);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].RX_CTLE_MAINTENANCE_PREG__RX_CTLE_CTRL_PREG),31,16,(uint32_t)0x033C);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].CREQ_EQ_OPEN_EYE_THRESH_PREG__CREQ_SPARE_PREG),31,16,(uint32_t)0x44CC);
}
#endif
