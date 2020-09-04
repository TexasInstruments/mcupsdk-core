/**
* File: csl_wiz16m_cs_refclk100MHz_32b_PCIe_cmn_pll_int_ssc_multilink_pll0.c
*
*  int_ssc
*  using_plllc
*  no_bw_cal
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
#ifndef CSL_WIZ16M_CS_REFCLK100MHz_32b_PCIe_CMN_PLL_C
#define CSL_WIZ16M_CS_REFCLK100MHz_32b_PCIe_CMN_PLL_C

#include <stdint.h>
#include <drivers/hw_include/cslr_soc.h>

#include <serdes_cd/V0/cslr_wiz16b4m4cs.h>

void csl_wiz16m_cs_refclk100MHz_32b_PCIe_cmn_pll_int_ssc_multilink_pll0(uint32_t baseAddr)
{
    CSL_wiz16b4m4csRegs *sierra_sds_reg = (CSL_wiz16b4m4csRegs *)(uintptr_t)(baseAddr);

    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->CMN_CTRL_CDBREG.CMN_PLLLC_LF_COEFF_MODE1_PREG__CMN_PLLLC_MODE_PREG),15,0,(uint32_t)0x000E);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->CMN_CTRL_CDBREG.CMN_PLLLC_LF_COEFF_MODE1_PREG__CMN_PLLLC_MODE_PREG),31,16,(uint32_t)0x4006);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->CMN_CTRL_CDBREG.CMN_PLLLC_LOCK_CNTSTART_PREG__CMN_PLLLC_LF_COEFF_MODE0_PREG),15,0,(uint32_t)0x4006);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->CMN_CTRL_CDBREG.CMN_PLLLC_BWCAL_MODE1_PREG__CMN_PLLLC_CLK0_PREG),31,16,(uint32_t)0x0000);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->CMN_CTRL_CDBREG.CMN_PLLLC_DSMCORR_PREG__CMN_PLLLC_BWCAL_MODE0_PREG),15,0,(uint32_t)0x0000);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->CMN_CTRL_CDBREG.CMN_PLLLC_DSMCORR_PREG__CMN_PLLLC_BWCAL_MODE0_PREG),31,16,(uint32_t)0x0581);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->CMN_CTRL_CDBREG.CMN_PLLLC_SS_AMP_STEP_SIZE_PREG__CMN_PLLLC_SS_PREG),15,0,(uint32_t)0x7F80);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->CMN_CTRL_CDBREG.CMN_PLLLC_SS_AMP_STEP_SIZE_PREG__CMN_PLLLC_SS_PREG),31,16,(uint32_t)0x0041);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->CMN_CTRL_CDBREG.CMN_PLLLC_LF_PROP_OVR_PREG__CMN_PLLLC_SSTWOPT_PREG),15,0,(uint32_t)0x0464);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->CMN_CTRL_CDBREG.CMN_PLLLC_LOCK_DELAY_CTRL_PREG__CMN_PLLLC_SS_TIME_STEPSIZE_MODE_PREG),15,0,(uint32_t)0x0D0D);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->CMN_CTRL_CDBREG.CMN_PLLLC_LOCK_DELAY_CTRL_PREG__CMN_PLLLC_SS_TIME_STEPSIZE_MODE_PREG),31,16,(uint32_t)0x0060);
}
#endif
