/**
* File: csl_wiz16m_cs_refclk100MHz_20b_USB.c
*
*  ext_ssc
*  ss opt1
*  using_plllc
*  no multilink
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
#ifndef CSL_WIZ16M_CS_REFCLK100MHz_20b_USB_C
#define CSL_WIZ16M_CS_REFCLK100MHz_20b_USB_C

#include <stdint.h>
#include <drivers/hw_include/cslr_soc.h>

#include <serdes_cd/V0/cslr_wiz16b4m4cs.h>

void csl_wiz16m_cs_refclk100MHz_20b_USB_cmn(uint32_t baseAddr)
{
    CSL_wiz16b4m4csRegs *sierra_sds_reg = (CSL_wiz16b4m4csRegs *)(uintptr_t)(baseAddr);

    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->PHY_PCS_CMN_REGISTERS.PHY_PIPE_USB3_GEN2_PRE_CFG1__PHY_PIPE_USB3_GEN2_PRE_CFG0),15,0,(uint32_t)0x0A0A);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->PHY_PCS_CMN_REGISTERS.PHY_PIPE_USB3_GEN2_POST_CFG1__PHY_PIPE_USB3_GEN2_POST_CFG0),15,0,(uint32_t)0x1000);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->PHY_PCS_CMN_REGISTERS.PHY_PIPE_USB3_GEN2_POST_CFG1__PHY_PIPE_USB3_GEN2_POST_CFG0),31,16,(uint32_t)0x0010);
}

void csl_wiz16m_cs_refclk100MHz_20b_USB_cmn_pll_ext_ssc(uint32_t baseAddr)
{
    CSL_wiz16b4m4csRegs *sierra_sds_reg = (CSL_wiz16b4m4csRegs *)(uintptr_t)(baseAddr);

    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->CMN_CTRL_CDBREG.CMN_PLLLC_LF_COEFF_MODE1_PREG__CMN_PLLLC_MODE_PREG),31,16,(uint32_t)0x2085);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->CMN_CTRL_CDBREG.CMN_PLLLC_LOCK_CNTSTART_PREG__CMN_PLLLC_LF_COEFF_MODE0_PREG),15,0,(uint32_t)0x2085);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->CMN_CTRL_CDBREG.CMN_PLLLC_BWCAL_MODE1_PREG__CMN_PLLLC_CLK0_PREG),31,16,(uint32_t)0x0000);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->CMN_CTRL_CDBREG.CMN_PLLLC_DSMCORR_PREG__CMN_PLLLC_BWCAL_MODE0_PREG),15,0,(uint32_t)0x0000);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->CMN_CTRL_CDBREG.CMN_PLLLC_LOCK_DELAY_CTRL_PREG__CMN_PLLLC_SS_TIME_STEPSIZE_MODE_PREG),15,0,(uint32_t)0x0000);
}

void csl_wiz16m_cs_refclk100MHz_20b_USB_cmn_pll_int_ssc(uint32_t baseAddr)
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

void csl_wiz16m_cs_refclk100MHz_20b_USB_ln_ext_ssc(uint32_t baseAddr, uint32_t laneNum)
{
    CSL_wiz16b4m4csRegs *sierra_sds_reg = (CSL_wiz16b4m4csRegs *)(uintptr_t)(baseAddr);

    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DET_STANDEC_B_PREG__DET_STANDEC_A_PREG),15,0,(uint32_t)0xFE0A);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DET_STANDEC_B_PREG__DET_STANDEC_A_PREG),31,16,(uint32_t)0x000F);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DET_STANDEC_D_PREG__DET_STANDEC_C_PREG),15,0,(uint32_t)0x00A5);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DET_STANDEC_D_PREG__DET_STANDEC_C_PREG),31,16,(uint32_t)0x69ad);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].FPWRISO_OVRD_PREG__DET_STANDEC_E_PREG),15,0,(uint32_t)0x0241);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].PSM_A0IN_TMR_PREG__PSM_LANECAL_DLY_A1_RESETS_PREG),15,0,(uint32_t)0x0010);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].PSM_A0IN_TMR_PREG__PSM_LANECAL_DLY_A1_RESETS_PREG),31,16,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].PSM_DIAG_PREG),31,16,(uint32_t)0xCF00);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].PSC_TX_A1_PREG__PSC_TX_A0_PREG),15,0,(uint32_t)0x001F);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].PSC_TX_A1_PREG__PSC_TX_A0_PREG),31,16,(uint32_t)0x0007);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].PSC_TX_A3_PREG__PSC_TX_A2_PREG),15,0,(uint32_t)0x0003);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].PSC_TX_A3_PREG__PSC_TX_A2_PREG),31,16,(uint32_t)0x0003);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].PSC_RX_A1_PREG__PSC_RX_A0_PREG),15,0,(uint32_t)0x0FFF);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].PSC_RX_A1_PREG__PSC_RX_A0_PREG),31,16,(uint32_t)0x0619);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].PSC_RX_A3_PREG__PSC_RX_A2_PREG),15,0,(uint32_t)0x0003);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].PSC_RX_A3_PREG__PSC_RX_A2_PREG),31,16,(uint32_t)0x0001);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].PLLCTRL_GEN_A_PREG__PLLCTRL_SUBRATE_PREG),15,0,(uint32_t)0x0001);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].PLLCTRL_CPGAIN_MODE_PREG__PLLCTRL_GEN_D_PREG),15,0,(uint32_t)0x0406);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].PLLCTRL_CPGAIN_MODE_PREG__PLLCTRL_GEN_D_PREG),31,16,(uint32_t)0x5233);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].CLKPATH_BIASTRIM_PREG__RXFE_BIASTRIM_PREG),31,16,(uint32_t)0x00CA);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DFE_BIASTRIM_PREG),15,0,(uint32_t)0x2512);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DRVCTRL_CM1_CV_PREG__DRVCTRL_ATTEN_PREG),15,0,(uint32_t)0x0000);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].CLKPATHCTRL_TMR_PREG__CLKPATHCTRL_OVR_PREG),31,16,(uint32_t)0x873E);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].RX_CREQ_FLTR_A_MODE1_PREG__RX_CREQ_FLTR_A_MODE2_PREG),31,16,(uint32_t)0x03CF);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].RX_CREQ_FLTR_B_PREG__RX_CREQ_FLTR_A_MODE0_PREG),15,0,(uint32_t)0x01CE);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].RX_CTLE_CAL_PREG__CREQ_CCLKDET_MODE01_PREG),15,0,(uint32_t)0x7B3C);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].RX_CTLE_MAINTENANCE_PREG__RX_CTLE_CTRL_PREG),31,16,(uint32_t)0x033F);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].CREQ_EQ_CTRL_PREG__CREQ_FSMCLK_SEL_PREG),15,0,(uint32_t)0x3232);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].CREQ_EQ_CTRL_PREG__CREQ_FSMCLK_SEL_PREG),31,16,(uint32_t)0x0000);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].CREQ_EQ_OPEN_EYE_THRESH_PREG__CREQ_SPARE_PREG),15,0,(uint32_t)0x8000);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].CREQ_EQ_OPEN_EYE_THRESH_PREG__CREQ_SPARE_PREG),31,16,(uint32_t)0xCC44);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].CTLELUT_OVRDCTRL_PREG__CTLELUT_CTRL_PREG),15,0,(uint32_t)0x8453);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DFE_SMP_RATESEL_PREG__DFE_ECMP_RATESEL_PREG),15,0,(uint32_t)0x4110);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DFE_SMP_RATESEL_PREG__DFE_ECMP_RATESEL_PREG),31,16,(uint32_t)0x4110);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_PHALIGN_CTRL),15,0,(uint32_t)0x0002);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_CONCUR_CTRL2_PREG__DEQ_CONCUR_CTRL1_PREG),15,0,(uint32_t)0x3200);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_CONCUR_CTRL2_PREG__DEQ_CONCUR_CTRL1_PREG),31,16,(uint32_t)0x5064);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_EPIPWR_CTRL2_PREG__CONCUR_PREEVAL_MINITER_CTRL_PREG),31,16,(uint32_t)0x0030);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].RX_DEQ_COEF_FIFO_PREG__DEQ_FAST_MAINT_CYCLES_PREG),15,0,(uint32_t)0x0048);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_ERRCMPA_OVR_PREG__DEQ_ERRCMP_CTRL_PREG),15,0,(uint32_t)0x5A5A);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_OFFSET_OVR_CTRL_PREG__DEQ_OFFSET_CTRL_PREG),15,0,(uint32_t)0x02F5);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_VGATUNE_CTRL_PREG__DEQ_GAIN_CTRL_PREG),15,0,(uint32_t)0x02F5);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_VGATUNE_CTRL_PREG__DEQ_GAIN_CTRL_PREG),31,16,(uint32_t)0x9A8A);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_GLUT1__DEQ_GLUT0),15,0,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_GLUT1__DEQ_GLUT0),31,16,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_GLUT3__DEQ_GLUT2),15,0,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_GLUT3__DEQ_GLUT2),31,16,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_GLUT5__DEQ_GLUT4),15,0,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_GLUT5__DEQ_GLUT4),31,16,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_GLUT7__DEQ_GLUT6),15,0,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_GLUT7__DEQ_GLUT6),31,16,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_GLUT9__DEQ_GLUT8),15,0,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_GLUT9__DEQ_GLUT8),31,16,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_GLUT11__DEQ_GLUT10),15,0,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_GLUT11__DEQ_GLUT10),31,16,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_GLUT13__DEQ_GLUT12),15,0,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_GLUT13__DEQ_GLUT12),31,16,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_GLUT15__DEQ_GLUT14),15,0,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_GLUT15__DEQ_GLUT14),31,16,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_GLUT17__DEQ_GLUT16),15,0,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_ALUT1__DEQ_ALUT0),15,0,(uint32_t)0x0BAE);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_ALUT1__DEQ_ALUT0),31,16,(uint32_t)0x0AEB);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_ALUT3__DEQ_ALUT2),15,0,(uint32_t)0x0A28);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_ALUT3__DEQ_ALUT2),31,16,(uint32_t)0x0965);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_ALUT5__DEQ_ALUT4),15,0,(uint32_t)0x08A2);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_ALUT5__DEQ_ALUT4),31,16,(uint32_t)0x07DF);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_ALUT7__DEQ_ALUT6),15,0,(uint32_t)0x071C);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_ALUT7__DEQ_ALUT6),31,16,(uint32_t)0x0659);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_ALUT9__DEQ_ALUT8),15,0,(uint32_t)0x0596);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_ALUT9__DEQ_ALUT8),31,16,(uint32_t)0x0514);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_ALUT11__DEQ_ALUT10),15,0,(uint32_t)0x0492);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_ALUT11__DEQ_ALUT10),31,16,(uint32_t)0x0410);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_ALUT13__DEQ_ALUT12),15,0,(uint32_t)0x038E);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_ALUT13__DEQ_ALUT12),31,16,(uint32_t)0x030C);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_DFETAP0__DEQ_DFETAP_CTRL_PREG),15,0,(uint32_t)0x03F4);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DFE_TRAINING_MASK_PREG__DFE_EN_1010_IGNORE_PREG),15,0,(uint32_t)0x0001);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_TAU_CTRL1_FAST_MAINT_PREG),31,16,(uint32_t)0x3C01);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_TAU_CTRL2_PREG__DEQ_TAU_CTRL1_SLOW_MAINT_PREG),15,0,(uint32_t)0x3C40);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_TAU_CTRL2_PREG__DEQ_TAU_CTRL1_SLOW_MAINT_PREG),31,16,(uint32_t)0x1C08);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_PICTRL_PREG__DEQ_PI_OVR_CTRL_PREG),31,16,(uint32_t)0x0033);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].CPICAL_TMRVAL_MODE0_PREG__CPICAL_TMRVAL_MODE1_PREG),15,0,(uint32_t)0x0400);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].CPICAL_TMRVAL_MODE0_PREG__CPICAL_TMRVAL_MODE1_PREG),31,16,(uint32_t)0x0330);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].CPICAL_PICNT_MODE0_PREG__CPICAL_PICNT_MODE1_PREG),15,0,(uint32_t)0x01FF);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].CPI_OUTBUF_RATESEL_PREG),15,0,(uint32_t)0x0009);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].CPICAL_RES_STARTCODE_MODE23_PREG__CPICAL_INCR_DECR_PREG),31,16,(uint32_t)0x3232);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].LFPSFILT_MD_PREG__LFPSDET_SUPPORT_PREG),15,0,(uint32_t)0x0005);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].LFPSFILT_RD_PREG__LFPSFILT_NS_PREG),15,0,(uint32_t)0x000F);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].LFPSFILT_RD_PREG__LFPSFILT_NS_PREG),31,16,(uint32_t)0x0009);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].LFPSFILT_MP_PREG),15,0,(uint32_t)0x0001);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].SDFILT_H2L_A_PREG__SIGDET_SUPPORT_PREG),15,0,(uint32_t)0x6313);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].SDFILT_H2L_A_PREG__SIGDET_SUPPORT_PREG),31,16,(uint32_t)0x8013);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].SDFILT_L2H_PREG__SDFILT_H2L_B_PREG),31,16,(uint32_t)0x8009);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].RXBUFFER_RCDFECTRL_PREG__RXBUFFER_CTLECTRL_PREG),15,0,(uint32_t)0x0024);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].RXBUFFER_RCDFECTRL_PREG__RXBUFFER_CTLECTRL_PREG),31,16,(uint32_t)0x0020);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].RXBUFFER_DFECTRL_PREG),15,0,(uint32_t)0x4243);
}

void csl_wiz16m_cs_refclk100MHz_20b_USB_ln_int_ssc(uint32_t baseAddr, uint32_t laneNum)
{
    CSL_wiz16b4m4csRegs *sierra_sds_reg = (CSL_wiz16b4m4csRegs *)(uintptr_t)(baseAddr);

    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DET_STANDEC_B_PREG__DET_STANDEC_A_PREG),15,0,(uint32_t)0xFE0A);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DET_STANDEC_B_PREG__DET_STANDEC_A_PREG),31,16,(uint32_t)0x000F);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DET_STANDEC_D_PREG__DET_STANDEC_C_PREG),15,0,(uint32_t)0x00A5);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DET_STANDEC_D_PREG__DET_STANDEC_C_PREG),31,16,(uint32_t)0x69ad);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].FPWRISO_OVRD_PREG__DET_STANDEC_E_PREG),15,0,(uint32_t)0x0241);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].PSM_A0IN_TMR_PREG__PSM_LANECAL_DLY_A1_RESETS_PREG),15,0,(uint32_t)0x0010);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].PSM_A0IN_TMR_PREG__PSM_LANECAL_DLY_A1_RESETS_PREG),31,16,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].PSM_DIAG_PREG),31,16,(uint32_t)0xCF00);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].PSC_TX_A1_PREG__PSC_TX_A0_PREG),15,0,(uint32_t)0x001F);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].PSC_TX_A1_PREG__PSC_TX_A0_PREG),31,16,(uint32_t)0x0007);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].PSC_TX_A3_PREG__PSC_TX_A2_PREG),15,0,(uint32_t)0x0003);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].PSC_TX_A3_PREG__PSC_TX_A2_PREG),31,16,(uint32_t)0x0003);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].PSC_RX_A1_PREG__PSC_RX_A0_PREG),15,0,(uint32_t)0x0FFF);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].PSC_RX_A1_PREG__PSC_RX_A0_PREG),31,16,(uint32_t)0x0619);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].PSC_RX_A3_PREG__PSC_RX_A2_PREG),15,0,(uint32_t)0x0003);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].PSC_RX_A3_PREG__PSC_RX_A2_PREG),31,16,(uint32_t)0x0001);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].PLLCTRL_GEN_A_PREG__PLLCTRL_SUBRATE_PREG),15,0,(uint32_t)0x0001);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].PLLCTRL_CPGAIN_MODE_PREG__PLLCTRL_GEN_D_PREG),15,0,(uint32_t)0x0406);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].PLLCTRL_CPGAIN_MODE_PREG__PLLCTRL_GEN_D_PREG),31,16,(uint32_t)0x5233);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].CLKPATH_BIASTRIM_PREG__RXFE_BIASTRIM_PREG),31,16,(uint32_t)0x00CA);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DFE_BIASTRIM_PREG),15,0,(uint32_t)0x2512);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DRVCTRL_CM1_CV_PREG__DRVCTRL_ATTEN_PREG),15,0,(uint32_t)0x0000);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].CLKPATHCTRL_TMR_PREG__CLKPATHCTRL_OVR_PREG),31,16,(uint32_t)0x873E);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].RX_CREQ_FLTR_A_MODE1_PREG__RX_CREQ_FLTR_A_MODE2_PREG),31,16,(uint32_t)0x03CF);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].RX_CREQ_FLTR_B_PREG__RX_CREQ_FLTR_A_MODE0_PREG),15,0,(uint32_t)0x01CE);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].RX_CTLE_CAL_PREG__CREQ_CCLKDET_MODE01_PREG),15,0,(uint32_t)0x7B3C);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].RX_CTLE_MAINTENANCE_PREG__RX_CTLE_CTRL_PREG),31,16,(uint32_t)0x033F);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].CREQ_EQ_CTRL_PREG__CREQ_FSMCLK_SEL_PREG),15,0,(uint32_t)0x3232);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].CREQ_EQ_CTRL_PREG__CREQ_FSMCLK_SEL_PREG),31,16,(uint32_t)0x0000);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].CREQ_EQ_OPEN_EYE_THRESH_PREG__CREQ_SPARE_PREG),15,0,(uint32_t)0x8000);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].CREQ_EQ_OPEN_EYE_THRESH_PREG__CREQ_SPARE_PREG),31,16,(uint32_t)0xCC44);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].CTLELUT_OVRDCTRL_PREG__CTLELUT_CTRL_PREG),15,0,(uint32_t)0x8453);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DFE_SMP_RATESEL_PREG__DFE_ECMP_RATESEL_PREG),15,0,(uint32_t)0x4110);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DFE_SMP_RATESEL_PREG__DFE_ECMP_RATESEL_PREG),31,16,(uint32_t)0x4110);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_PHALIGN_CTRL),15,0,(uint32_t)0x0002);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_CONCUR_CTRL2_PREG__DEQ_CONCUR_CTRL1_PREG),15,0,(uint32_t)0x3200);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_CONCUR_CTRL2_PREG__DEQ_CONCUR_CTRL1_PREG),31,16,(uint32_t)0x5064);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_EPIPWR_CTRL2_PREG__CONCUR_PREEVAL_MINITER_CTRL_PREG),31,16,(uint32_t)0x0030);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].RX_DEQ_COEF_FIFO_PREG__DEQ_FAST_MAINT_CYCLES_PREG),15,0,(uint32_t)0x0048);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_ERRCMPA_OVR_PREG__DEQ_ERRCMP_CTRL_PREG),15,0,(uint32_t)0x5A5A);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_OFFSET_OVR_CTRL_PREG__DEQ_OFFSET_CTRL_PREG),15,0,(uint32_t)0x02F5);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_VGATUNE_CTRL_PREG__DEQ_GAIN_CTRL_PREG),15,0,(uint32_t)0x02F5);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_VGATUNE_CTRL_PREG__DEQ_GAIN_CTRL_PREG),31,16,(uint32_t)0x9A8A);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_GLUT1__DEQ_GLUT0),15,0,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_GLUT1__DEQ_GLUT0),31,16,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_GLUT3__DEQ_GLUT2),15,0,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_GLUT3__DEQ_GLUT2),31,16,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_GLUT5__DEQ_GLUT4),15,0,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_GLUT5__DEQ_GLUT4),31,16,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_GLUT7__DEQ_GLUT6),15,0,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_GLUT7__DEQ_GLUT6),31,16,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_GLUT9__DEQ_GLUT8),15,0,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_GLUT9__DEQ_GLUT8),31,16,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_GLUT11__DEQ_GLUT10),15,0,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_GLUT11__DEQ_GLUT10),31,16,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_GLUT13__DEQ_GLUT12),15,0,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_GLUT13__DEQ_GLUT12),31,16,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_GLUT15__DEQ_GLUT14),15,0,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_GLUT15__DEQ_GLUT14),31,16,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_GLUT17__DEQ_GLUT16),15,0,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_ALUT1__DEQ_ALUT0),15,0,(uint32_t)0x0BAE);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_ALUT1__DEQ_ALUT0),31,16,(uint32_t)0x0AEB);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_ALUT3__DEQ_ALUT2),15,0,(uint32_t)0x0A28);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_ALUT3__DEQ_ALUT2),31,16,(uint32_t)0x0965);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_ALUT5__DEQ_ALUT4),15,0,(uint32_t)0x08A2);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_ALUT5__DEQ_ALUT4),31,16,(uint32_t)0x07DF);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_ALUT7__DEQ_ALUT6),15,0,(uint32_t)0x071C);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_ALUT7__DEQ_ALUT6),31,16,(uint32_t)0x0659);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_ALUT9__DEQ_ALUT8),15,0,(uint32_t)0x0596);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_ALUT9__DEQ_ALUT8),31,16,(uint32_t)0x0514);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_ALUT11__DEQ_ALUT10),15,0,(uint32_t)0x0492);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_ALUT11__DEQ_ALUT10),31,16,(uint32_t)0x0410);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_ALUT13__DEQ_ALUT12),15,0,(uint32_t)0x038E);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_ALUT13__DEQ_ALUT12),31,16,(uint32_t)0x030C);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_DFETAP0__DEQ_DFETAP_CTRL_PREG),15,0,(uint32_t)0x03F4);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DFE_TRAINING_MASK_PREG__DFE_EN_1010_IGNORE_PREG),15,0,(uint32_t)0x0001);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_TAU_CTRL1_FAST_MAINT_PREG),31,16,(uint32_t)0x3C01);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_TAU_CTRL2_PREG__DEQ_TAU_CTRL1_SLOW_MAINT_PREG),15,0,(uint32_t)0x3C40);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_TAU_CTRL2_PREG__DEQ_TAU_CTRL1_SLOW_MAINT_PREG),31,16,(uint32_t)0x1C08);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_PICTRL_PREG__DEQ_PI_OVR_CTRL_PREG),31,16,(uint32_t)0x0033);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].CPICAL_TMRVAL_MODE0_PREG__CPICAL_TMRVAL_MODE1_PREG),15,0,(uint32_t)0x0200);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].CPICAL_TMRVAL_MODE0_PREG__CPICAL_TMRVAL_MODE1_PREG),31,16,(uint32_t)0x0330);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].CPICAL_PICNT_MODE0_PREG__CPICAL_PICNT_MODE1_PREG),15,0,(uint32_t)0x01FF);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].CPI_OUTBUF_RATESEL_PREG),15,0,(uint32_t)0x0009);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].CPICAL_RES_STARTCODE_MODE23_PREG__CPICAL_INCR_DECR_PREG),31,16,(uint32_t)0x3232);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].LFPSFILT_MD_PREG__LFPSDET_SUPPORT_PREG),15,0,(uint32_t)0x0005);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].LFPSFILT_RD_PREG__LFPSFILT_NS_PREG),15,0,(uint32_t)0x000F);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].LFPSFILT_RD_PREG__LFPSFILT_NS_PREG),31,16,(uint32_t)0x0009);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].LFPSFILT_MP_PREG),15,0,(uint32_t)0x0001);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].SDFILT_H2L_A_PREG__SIGDET_SUPPORT_PREG),15,0,(uint32_t)0x6313);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].SDFILT_H2L_A_PREG__SIGDET_SUPPORT_PREG),31,16,(uint32_t)0x8013);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].SDFILT_L2H_PREG__SDFILT_H2L_B_PREG),31,16,(uint32_t)0x8009);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].RXBUFFER_RCDFECTRL_PREG__RXBUFFER_CTLECTRL_PREG),15,0,(uint32_t)0x0024);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].RXBUFFER_RCDFECTRL_PREG__RXBUFFER_CTLECTRL_PREG),31,16,(uint32_t)0x0020);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].RXBUFFER_DFECTRL_PREG),15,0,(uint32_t)0x4243);
}

void csl_wiz16m_cs_refclk100MHz_20b_USB_ln_no_ssc(uint32_t baseAddr, uint32_t laneNum)
{
    CSL_wiz16b4m4csRegs *sierra_sds_reg = (CSL_wiz16b4m4csRegs *)(uintptr_t)(baseAddr);

    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DET_STANDEC_B_PREG__DET_STANDEC_A_PREG),15,0,(uint32_t)0xFE0A);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DET_STANDEC_B_PREG__DET_STANDEC_A_PREG),31,16,(uint32_t)0x000F);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DET_STANDEC_D_PREG__DET_STANDEC_C_PREG),15,0,(uint32_t)0x00A5);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DET_STANDEC_D_PREG__DET_STANDEC_C_PREG),31,16,(uint32_t)0x69ad);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].FPWRISO_OVRD_PREG__DET_STANDEC_E_PREG),15,0,(uint32_t)0x0241);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].PSM_A0IN_TMR_PREG__PSM_LANECAL_DLY_A1_RESETS_PREG),15,0,(uint32_t)0x0010);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].PSM_A0IN_TMR_PREG__PSM_LANECAL_DLY_A1_RESETS_PREG),31,16,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].PSM_DIAG_PREG),31,16,(uint32_t)0xCF00);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].PSC_TX_A1_PREG__PSC_TX_A0_PREG),15,0,(uint32_t)0x001F);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].PSC_TX_A1_PREG__PSC_TX_A0_PREG),31,16,(uint32_t)0x0007);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].PSC_TX_A3_PREG__PSC_TX_A2_PREG),15,0,(uint32_t)0x0003);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].PSC_TX_A3_PREG__PSC_TX_A2_PREG),31,16,(uint32_t)0x0003);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].PSC_RX_A1_PREG__PSC_RX_A0_PREG),15,0,(uint32_t)0x0FFF);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].PSC_RX_A1_PREG__PSC_RX_A0_PREG),31,16,(uint32_t)0x0619);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].PSC_RX_A3_PREG__PSC_RX_A2_PREG),15,0,(uint32_t)0x0003);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].PSC_RX_A3_PREG__PSC_RX_A2_PREG),31,16,(uint32_t)0x0001);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].PLLCTRL_GEN_A_PREG__PLLCTRL_SUBRATE_PREG),15,0,(uint32_t)0x0001);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].PLLCTRL_CPGAIN_MODE_PREG__PLLCTRL_GEN_D_PREG),15,0,(uint32_t)0x0406);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].PLLCTRL_CPGAIN_MODE_PREG__PLLCTRL_GEN_D_PREG),31,16,(uint32_t)0x5233);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].CLKPATH_BIASTRIM_PREG__RXFE_BIASTRIM_PREG),31,16,(uint32_t)0x00CA);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DFE_BIASTRIM_PREG),15,0,(uint32_t)0x2512);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DRVCTRL_CM1_CV_PREG__DRVCTRL_ATTEN_PREG),15,0,(uint32_t)0x0000);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].CLKPATHCTRL_TMR_PREG__CLKPATHCTRL_OVR_PREG),31,16,(uint32_t)0x873E);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].RX_CREQ_FLTR_A_MODE1_PREG__RX_CREQ_FLTR_A_MODE2_PREG),31,16,(uint32_t)0x03CF);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].RX_CREQ_FLTR_B_PREG__RX_CREQ_FLTR_A_MODE0_PREG),15,0,(uint32_t)0x01CE);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].RX_CTLE_CAL_PREG__CREQ_CCLKDET_MODE01_PREG),15,0,(uint32_t)0x7B3C);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].RX_CTLE_MAINTENANCE_PREG__RX_CTLE_CTRL_PREG),31,16,(uint32_t)0x033F);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].CREQ_EQ_CTRL_PREG__CREQ_FSMCLK_SEL_PREG),15,0,(uint32_t)0x3232);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].CREQ_EQ_CTRL_PREG__CREQ_FSMCLK_SEL_PREG),31,16,(uint32_t)0x0000);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].CREQ_EQ_OPEN_EYE_THRESH_PREG__CREQ_SPARE_PREG),15,0,(uint32_t)0x8000);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].CREQ_EQ_OPEN_EYE_THRESH_PREG__CREQ_SPARE_PREG),31,16,(uint32_t)0xCC44);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].CTLELUT_OVRDCTRL_PREG__CTLELUT_CTRL_PREG),15,0,(uint32_t)0x8453);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DFE_SMP_RATESEL_PREG__DFE_ECMP_RATESEL_PREG),15,0,(uint32_t)0x4110);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DFE_SMP_RATESEL_PREG__DFE_ECMP_RATESEL_PREG),31,16,(uint32_t)0x4110);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_PHALIGN_CTRL),15,0,(uint32_t)0x0002);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_CONCUR_CTRL2_PREG__DEQ_CONCUR_CTRL1_PREG),15,0,(uint32_t)0x3200);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_CONCUR_CTRL2_PREG__DEQ_CONCUR_CTRL1_PREG),31,16,(uint32_t)0x5064);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_EPIPWR_CTRL2_PREG__CONCUR_PREEVAL_MINITER_CTRL_PREG),31,16,(uint32_t)0x0030);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].RX_DEQ_COEF_FIFO_PREG__DEQ_FAST_MAINT_CYCLES_PREG),15,0,(uint32_t)0x0048);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_ERRCMPA_OVR_PREG__DEQ_ERRCMP_CTRL_PREG),15,0,(uint32_t)0x5A5A);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_OFFSET_OVR_CTRL_PREG__DEQ_OFFSET_CTRL_PREG),15,0,(uint32_t)0x02F5);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_VGATUNE_CTRL_PREG__DEQ_GAIN_CTRL_PREG),15,0,(uint32_t)0x02F5);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_VGATUNE_CTRL_PREG__DEQ_GAIN_CTRL_PREG),31,16,(uint32_t)0x9A8A);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_GLUT1__DEQ_GLUT0),15,0,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_GLUT1__DEQ_GLUT0),31,16,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_GLUT3__DEQ_GLUT2),15,0,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_GLUT3__DEQ_GLUT2),31,16,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_GLUT5__DEQ_GLUT4),15,0,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_GLUT5__DEQ_GLUT4),31,16,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_GLUT7__DEQ_GLUT6),15,0,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_GLUT7__DEQ_GLUT6),31,16,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_GLUT9__DEQ_GLUT8),15,0,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_GLUT9__DEQ_GLUT8),31,16,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_GLUT11__DEQ_GLUT10),15,0,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_GLUT11__DEQ_GLUT10),31,16,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_GLUT13__DEQ_GLUT12),15,0,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_GLUT13__DEQ_GLUT12),31,16,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_GLUT15__DEQ_GLUT14),15,0,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_GLUT15__DEQ_GLUT14),31,16,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_GLUT17__DEQ_GLUT16),15,0,(uint32_t)0x0014);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_ALUT1__DEQ_ALUT0),15,0,(uint32_t)0x0BAE);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_ALUT1__DEQ_ALUT0),31,16,(uint32_t)0x0AEB);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_ALUT3__DEQ_ALUT2),15,0,(uint32_t)0x0A28);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_ALUT3__DEQ_ALUT2),31,16,(uint32_t)0x0965);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_ALUT5__DEQ_ALUT4),15,0,(uint32_t)0x08A2);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_ALUT5__DEQ_ALUT4),31,16,(uint32_t)0x07DF);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_ALUT7__DEQ_ALUT6),15,0,(uint32_t)0x071C);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_ALUT7__DEQ_ALUT6),31,16,(uint32_t)0x0659);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_ALUT9__DEQ_ALUT8),15,0,(uint32_t)0x0596);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_ALUT9__DEQ_ALUT8),31,16,(uint32_t)0x0514);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_ALUT11__DEQ_ALUT10),15,0,(uint32_t)0x0492);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_ALUT11__DEQ_ALUT10),31,16,(uint32_t)0x0410);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_ALUT13__DEQ_ALUT12),15,0,(uint32_t)0x038E);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_ALUT13__DEQ_ALUT12),31,16,(uint32_t)0x030C);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_DFETAP0__DEQ_DFETAP_CTRL_PREG),15,0,(uint32_t)0x03F4);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DFE_TRAINING_MASK_PREG__DFE_EN_1010_IGNORE_PREG),15,0,(uint32_t)0x0001);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_TAU_CTRL1_FAST_MAINT_PREG),31,16,(uint32_t)0x3C01);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_TAU_CTRL2_PREG__DEQ_TAU_CTRL1_SLOW_MAINT_PREG),15,0,(uint32_t)0x3C40);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_TAU_CTRL2_PREG__DEQ_TAU_CTRL1_SLOW_MAINT_PREG),31,16,(uint32_t)0x1C08);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_PICTRL_PREG__DEQ_PI_OVR_CTRL_PREG),31,16,(uint32_t)0x0033);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].CPICAL_TMRVAL_MODE0_PREG__CPICAL_TMRVAL_MODE1_PREG),15,0,(uint32_t)0x0400);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].CPICAL_TMRVAL_MODE0_PREG__CPICAL_TMRVAL_MODE1_PREG),31,16,(uint32_t)0x0330);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].CPICAL_PICNT_MODE0_PREG__CPICAL_PICNT_MODE1_PREG),15,0,(uint32_t)0x01FF);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].CPI_OUTBUF_RATESEL_PREG),15,0,(uint32_t)0x0009);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].CPICAL_RES_STARTCODE_MODE23_PREG__CPICAL_INCR_DECR_PREG),31,16,(uint32_t)0x3232);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].LFPSFILT_MD_PREG__LFPSDET_SUPPORT_PREG),15,0,(uint32_t)0x0005);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].LFPSFILT_RD_PREG__LFPSFILT_NS_PREG),15,0,(uint32_t)0x000F);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].LFPSFILT_RD_PREG__LFPSFILT_NS_PREG),31,16,(uint32_t)0x0009);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].LFPSFILT_MP_PREG),15,0,(uint32_t)0x0001);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].SDFILT_H2L_A_PREG__SIGDET_SUPPORT_PREG),15,0,(uint32_t)0x6313);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].SDFILT_H2L_A_PREG__SIGDET_SUPPORT_PREG),31,16,(uint32_t)0x8013);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].SDFILT_L2H_PREG__SDFILT_H2L_B_PREG),31,16,(uint32_t)0x8009);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].RXBUFFER_RCDFECTRL_PREG__RXBUFFER_CTLECTRL_PREG),15,0,(uint32_t)0x0024);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].RXBUFFER_RCDFECTRL_PREG__RXBUFFER_CTLECTRL_PREG),31,16,(uint32_t)0x0020);
    CSL_FINSR(*(volatile uint32_t *)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].RXBUFFER_DFECTRL_PREG),15,0,(uint32_t)0x4243);
}
#endif

