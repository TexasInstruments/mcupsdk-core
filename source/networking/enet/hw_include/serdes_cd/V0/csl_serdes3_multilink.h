/**
 * @file csl_serdes3_multilink.h
 *
 * @brief
 *  Header file for functional layer of CSL SERDES.
 *
 *  It contains the various enumerations, structure definitions and function
 *  declarations
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2017, Texas Instruments, Inc.
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

#ifndef CSL_SERDES_MULTILINK_H_
#define CSL_SERDES_MULTILINK_H_
#ifdef __cplusplus
extern "C" {
#endif
extern CSL_SerdesResult CSL_serdesMultiLinkInit(
    CSL_SerdesMultilink         multiLink,
    CSL_SerdesInstance          serdesInstance,
    CSL_SerdesLaneEnableParams  *serdesLaneEnableParams,
    CSL_SerdesLaneEnableParams  *serdesLaneEnableParams1);

extern void csl_wiz16m_cs_refclk100MHz_20b_QSGMII_cmn_multilink_pll0(uint32_t baseAddr);
extern void csl_wiz16m_cs_refclk100MHz_20b_QSGMII_cmn_multilink_pll1(uint32_t baseAddr);
extern void csl_wiz16m_cs_refclk100MHz_20b_QSGMII_cmn_pll_multilink_pll0(uint32_t baseAddr);
extern void csl_wiz16m_cs_refclk100MHz_20b_QSGMII_cmn_pll_multilink_pll1(uint32_t baseAddr);
extern void csl_wiz16m_cs_refclk100MHz_20b_QSGMII_ln_multilink_pll0(uint32_t baseAddr, uint32_t laneNum);
extern void csl_wiz16m_cs_refclk100MHz_20b_QSGMII_ln_multilink_pll1(uint32_t baseAddr, uint32_t laneNum);
extern void csl_wiz16m_cs_refclk100MHz_20b_SGMII_cmn_multilink_pll1_opt3(uint32_t baseAddr);
extern void csl_wiz16m_cs_refclk100MHz_20b_SGMII_cmn_multilink_pll1(uint32_t baseAddr);
extern void csl_wiz16m_cs_refclk100MHz_20b_SGMII_cmn_pll_multilink_pll1_opt3(uint32_t baseAddr);
extern void csl_wiz16m_cs_refclk100MHz_20b_SGMII_cmn_pll_multilink_pll1(uint32_t baseAddr);
extern void csl_wiz16m_cs_refclk100MHz_20b_SGMII_ln_multilink_pll1_opt3(uint32_t baseAddr, uint32_t laneNum);
extern void csl_wiz16m_cs_refclk100MHz_20b_SGMII_ln_multilink_pll1(uint32_t baseAddr, uint32_t laneNum);
extern void csl_wiz16m_cs_refclk100MHz_20b_USB_cmn_ext_ssc_multilink_pll1(uint32_t baseAddr);
extern void csl_wiz16m_cs_refclk100MHz_20b_USB_cmn_pll_ext_ssc_multilink_pll1(uint32_t baseAddr);
extern void csl_wiz16m_cs_refclk100MHz_20b_USB_ln_ext_ssc_multilink_pll1(uint32_t baseAddr, uint32_t laneNum);
extern void csl_wiz16m_cs_refclk156p25MHz_20b_XAUI_cmn_multilink_pll0(uint32_t baseAddr);
extern void csl_wiz16m_cs_refclk156p25MHz_20b_XAUI_cmn_pll_multilink_pll0(uint32_t baseAddr);
extern void csl_wiz16m_cs_refclk156p25MHz_20b_XAUI_ln_multilink_pll0(uint32_t baseAddr, uint32_t laneNum);
extern void csl_wiz16m_cs_refclk100MHz_32b_PCIe_cmn_pll_ext_ssc_multilink_pll0(uint32_t baseAddr);
extern void csl_wiz16m_cs_refclk100MHz_32b_PCIe_cmn_pll_int_ssc_multilink_pll0(uint32_t baseAddr);
extern void csl_wiz16m_cs_refclk100MHz_32b_PCIe_ln_ext_ssc_multilink_pll0(uint32_t baseAddr, uint32_t laneNum);
extern void csl_wiz16m_cs_refclk100MHz_32b_PCIe_ln_int_ssc_multilink_pll0(uint32_t baseAddr, uint32_t laneNum);
extern void csl_wiz16m_cs_refclk100MHz_32b_PCIe_cmn_pll_no_ssc_multilink_pll0(uint32_t baseAddr);
extern void csl_wiz16m_cs_refclk100MHz_32b_PCIe_ln_no_ssc_multilink_pll0(uint32_t baseAddr, uint32_t laneNum);
#ifdef __cplusplus
}
#endif
#endif
