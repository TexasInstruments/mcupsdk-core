/**
* File: csl_wiz16m_cs_refclk100MHz_20b_SGMII_ln_multilink_pll1.c
*
*  no_ssc
*  using_plllc1
*  opt_1
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
#ifndef CSL_WIZ16M_CS_REFCLK100MHz_20b_SGMII_LN_C
#define CSL_WIZ16M_CS_REFCLK100MHz_20b_SGMII_LN_C

#include <stdint.h>
#include <drivers/hw_include/cslr_soc.h>

#include <serdes_cd/V0/cslr_wiz16b4m4cs.h>

void csl_wiz16m_cs_refclk100MHz_20b_SGMII_ln_multilink_pll1(uint32_t baseAddr, uint32_t laneNum)
{
    CSL_wiz16b4m4csRegs *sierra_sds_reg = (CSL_wiz16b4m4csRegs *)(uintptr_t)(baseAddr);

    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DET_STANDEC_D_PREG__DET_STANDEC_C_PREG),31,16,(uint32_t)0x688E);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].PSC_LN_IDLE_PREG),15,0,(uint32_t)0x0004);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].PSC_RX_A1_PREG__PSC_RX_A0_PREG),15,0,(uint32_t)0x0FFE);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].PLLCTRL_GEN_A_PREG__PLLCTRL_SUBRATE_PREG),15,0,(uint32_t)0x0013);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].PLLCTRL_GEN_A_PREG__PLLCTRL_SUBRATE_PREG),31,16,(uint32_t)0x0003);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].PLLCTRL_CPGAIN_MODE_PREG__PLLCTRL_GEN_D_PREG),15,0,(uint32_t)0x0106);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DRVCTRL_CM1_CV_PREG__DRVCTRL_ATTEN_PREG),15,0,(uint32_t)0x0000);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].RX_CREQ_FLTR_B_PREG__RX_CREQ_FLTR_A_MODE0_PREG),15,0,(uint32_t)0x0051);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].RX_CTLE_CAL_PREG__CREQ_CCLKDET_MODE01_PREG),15,0,(uint32_t)0x3C0E);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].CREQ_EQ_CTRL_PREG__CREQ_FSMCLK_SEL_PREG),15,0,(uint32_t)0x3220);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].CREQ_EQ_CTRL_PREG__CREQ_FSMCLK_SEL_PREG),31,16,(uint32_t)0x0000);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_PHALIGN_CTRL),15,0,(uint32_t)0x0002);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_GLUT1__DEQ_GLUT0),15,0,(uint32_t)0x0186);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_GLUT1__DEQ_GLUT0),31,16,(uint32_t)0x0186);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_GLUT3__DEQ_GLUT2),15,0,(uint32_t)0x0186);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_GLUT3__DEQ_GLUT2),31,16,(uint32_t)0x0186);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_GLUT5__DEQ_GLUT4),15,0,(uint32_t)0x0186);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_ALUT1__DEQ_ALUT0),15,0,(uint32_t)0x0861);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_ALUT1__DEQ_ALUT0),31,16,(uint32_t)0x07E0);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_ALUT3__DEQ_ALUT2),15,0,(uint32_t)0x079E);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_ALUT3__DEQ_ALUT2),31,16,(uint32_t)0x071D);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_DFETAP0__DEQ_DFETAP_CTRL_PREG),15,0,(uint32_t)0x03F5);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_TAU_CTRL1_FAST_MAINT_PREG),31,16,(uint32_t)0x0C01);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_TAU_CTRL2_PREG__DEQ_TAU_CTRL1_SLOW_MAINT_PREG),15,0,(uint32_t)0x3C40);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_TAU_CTRL2_PREG__DEQ_TAU_CTRL1_SLOW_MAINT_PREG),31,16,(uint32_t)0x1C04);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_PICTRL_PREG__DEQ_PI_OVR_CTRL_PREG),31,16,(uint32_t)0x0033);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].CPI_OUTBUF_RATESEL_PREG),15,0,(uint32_t)0x0000);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].CPI_TRIM_PREG__CPI_RESBIAS_BIN_PREG),15,0,(uint32_t)0x0B6D);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].RXBUFFER_RCDFECTRL_PREG__RXBUFFER_CTLECTRL_PREG),15,0,(uint32_t)0x0102);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].RXBUFFER_RCDFECTRL_PREG__RXBUFFER_CTLECTRL_PREG),31,16,(uint32_t)0x0002);
}
#endif
