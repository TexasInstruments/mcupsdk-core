/********************************************************************
* Copyright (C) 2024 Texas Instruments Incorporated.
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
#ifndef _CSL_SOC_FW_H_
#define _CSL_SOC_FW_H_


/*
 * Auto-generated CSL definitions for SoC Firewalls:
*/
#define CSL_FW_SECURITY     (0U)
#define CSL_FW_CHANNEL      (1U)

/* Standard Security Slave-side Firewall Definitions */

/* Properties of firewall protecting slave endpoint: IPSRAM256X32E_MAIN_0.RAM_VB*/
#define CSL_FW_IPSRAM256X32E_MAIN_0_RAM_VB_ID                         1
#define CSL_FW_IPSRAM256X32E_MAIN_0_RAM_VB_TYPE                       CSL_FW_SECURITY
#define CSL_FW_IPSRAM256X32E_MAIN_0_RAM_VB_MMR_BASE                   0x45000400
#define CSL_FW_IPSRAM256X32E_MAIN_0_RAM_VB_NUM_REGIONS                1
#define CSL_FW_IPSRAM256X32E_MAIN_0_RAM_VB_NUM_PRIV_IDS_PER_REGION    3
#define CSL_FW_IPSRAM256X32E_MAIN_0_RAM_VB_NUM_PROTECTED_HW_REGIONS   1
#define CSL_FW_IPSRAM256X32E_MAIN_0_RAM_VB_RAM_START                  0x0
#define CSL_FW_IPSRAM256X32E_MAIN_0_RAM_VB_RAM_END                    0x3ff

/* Properties of firewall protecting slave endpoint: IPSRAM256X32E_MAIN_0.ECCAGGR_VB*/
#define CSL_FW_IPSRAM256X32E_MAIN_0_ECCAGGR_VB_ID                     2
#define CSL_FW_IPSRAM256X32E_MAIN_0_ECCAGGR_VB_TYPE                   CSL_FW_SECURITY
#define CSL_FW_IPSRAM256X32E_MAIN_0_ECCAGGR_VB_MMR_BASE               0x45000800
#define CSL_FW_IPSRAM256X32E_MAIN_0_ECCAGGR_VB_NUM_REGIONS            1
#define CSL_FW_IPSRAM256X32E_MAIN_0_ECCAGGR_VB_NUM_PRIV_IDS_PER_REGION          3
#define CSL_FW_IPSRAM256X32E_MAIN_0_ECCAGGR_VB_NUM_PROTECTED_HW_REGIONS         1
#define CSL_FW_IPSRAM256X32E_MAIN_0_ECCAGGR_VB_REGS_START             0x780000
#define CSL_FW_IPSRAM256X32E_MAIN_0_ECCAGGR_VB_REGS_END               0x7803ff

/* Properties of firewall protecting slave endpoint: IK3_MAIN_PSC_WRAP_MAIN_0.VBUS*/
#define CSL_FW_IK3_MAIN_PSC_WRAP_MAIN_0_VBUS_ID                       5
#define CSL_FW_IK3_MAIN_PSC_WRAP_MAIN_0_VBUS_TYPE                     CSL_FW_SECURITY
#define CSL_FW_IK3_MAIN_PSC_WRAP_MAIN_0_VBUS_MMR_BASE                 0x45001400
#define CSL_FW_IK3_MAIN_PSC_WRAP_MAIN_0_VBUS_NUM_REGIONS              1
#define CSL_FW_IK3_MAIN_PSC_WRAP_MAIN_0_VBUS_NUM_PRIV_IDS_PER_REGION  3
#define CSL_FW_IK3_MAIN_PSC_WRAP_MAIN_0_VBUS_NUM_PROTECTED_HW_REGIONS 1
#define CSL_FW_IK3_MAIN_PSC_WRAP_MAIN_0_VBUS_VBUS_START               0x400000
#define CSL_FW_IK3_MAIN_PSC_WRAP_MAIN_0_VBUS_VBUS_END                 0x400fff

/* Properties of firewall protecting slave endpoint: IK3_PLL_CTRL_WRAP_MAIN_0.VBUS_SLV*/
#define CSL_FW_IK3_PLL_CTRL_WRAP_MAIN_0_VBUS_SLV_ID                   6
#define CSL_FW_IK3_PLL_CTRL_WRAP_MAIN_0_VBUS_SLV_TYPE                 CSL_FW_SECURITY
#define CSL_FW_IK3_PLL_CTRL_WRAP_MAIN_0_VBUS_SLV_MMR_BASE             0x45001800
#define CSL_FW_IK3_PLL_CTRL_WRAP_MAIN_0_VBUS_SLV_NUM_REGIONS          1
#define CSL_FW_IK3_PLL_CTRL_WRAP_MAIN_0_VBUS_SLV_NUM_PRIV_IDS_PER_REGION        3
#define CSL_FW_IK3_PLL_CTRL_WRAP_MAIN_0_VBUS_SLV_NUM_PROTECTED_HW_REGIONS       1
#define CSL_FW_IK3_PLL_CTRL_WRAP_MAIN_0_VBUS_SLV_MEM_START            0x410000
#define CSL_FW_IK3_PLL_CTRL_WRAP_MAIN_0_VBUS_SLV_MEM_END              0x4101ff

/* Properties of firewall protecting slave endpoint: IGTC_R10_MAIN_0.VBUSP*/
#define CSL_FW_IGTC_R10_MAIN_0_VBUSP_ID                               7
#define CSL_FW_IGTC_R10_MAIN_0_VBUSP_TYPE                             CSL_FW_SECURITY
#define CSL_FW_IGTC_R10_MAIN_0_VBUSP_MMR_BASE                         0x45001c00
#define CSL_FW_IGTC_R10_MAIN_0_VBUSP_NUM_REGIONS                      4
#define CSL_FW_IGTC_R10_MAIN_0_VBUSP_NUM_PRIV_IDS_PER_REGION          3
#define CSL_FW_IGTC_R10_MAIN_0_VBUSP_NUM_PROTECTED_HW_REGIONS         4
#define CSL_FW_IGTC_R10_MAIN_0_VBUSP_GTC_CFG0_START                   0xa80000
#define CSL_FW_IGTC_R10_MAIN_0_VBUSP_GTC_CFG0_END                     0xa803ff
#define CSL_FW_IGTC_R10_MAIN_0_VBUSP_GTC_CFG1_START                   0xa90000
#define CSL_FW_IGTC_R10_MAIN_0_VBUSP_GTC_CFG1_END                     0xa93fff
#define CSL_FW_IGTC_R10_MAIN_0_VBUSP_GTC_CFG2_START                   0xaa0000
#define CSL_FW_IGTC_R10_MAIN_0_VBUSP_GTC_CFG2_END                     0xaa3fff
#define CSL_FW_IGTC_R10_MAIN_0_VBUSP_GTC_CFG3_START                   0xab0000
#define CSL_FW_IGTC_R10_MAIN_0_VBUSP_GTC_CFG3_END                     0xab3fff

/* Properties of firewall protecting slave endpoint: IMAIN_PLL_MMR_MAIN_0.VBUSP*/
#define CSL_FW_IMAIN_PLL_MMR_MAIN_0_VBUSP_ID                          8
#define CSL_FW_IMAIN_PLL_MMR_MAIN_0_VBUSP_TYPE                        CSL_FW_SECURITY
#define CSL_FW_IMAIN_PLL_MMR_MAIN_0_VBUSP_MMR_BASE                    0x45002000
#define CSL_FW_IMAIN_PLL_MMR_MAIN_0_VBUSP_NUM_REGIONS                 8
#define CSL_FW_IMAIN_PLL_MMR_MAIN_0_VBUSP_NUM_PRIV_IDS_PER_REGION     3
#define CSL_FW_IMAIN_PLL_MMR_MAIN_0_VBUSP_NUM_PROTECTED_HW_REGIONS    1
#define CSL_FW_IMAIN_PLL_MMR_MAIN_0_VBUSP_CFG_START                   0x680000
#define CSL_FW_IMAIN_PLL_MMR_MAIN_0_VBUSP_CFG_END                     0x687fff

/* Properties of firewall protecting slave endpoint: IMAIN_CTRL_MMR_MAIN_0.VBUSP*/
#define CSL_FW_IMAIN_CTRL_MMR_MAIN_0_VBUSP_ID                         9
#define CSL_FW_IMAIN_CTRL_MMR_MAIN_0_VBUSP_TYPE                       CSL_FW_SECURITY
#define CSL_FW_IMAIN_CTRL_MMR_MAIN_0_VBUSP_MMR_BASE                   0x45002400
#define CSL_FW_IMAIN_CTRL_MMR_MAIN_0_VBUSP_NUM_REGIONS                16
#define CSL_FW_IMAIN_CTRL_MMR_MAIN_0_VBUSP_NUM_PRIV_IDS_PER_REGION    3
#define CSL_FW_IMAIN_CTRL_MMR_MAIN_0_VBUSP_NUM_PROTECTED_HW_REGIONS   1
#define CSL_FW_IMAIN_CTRL_MMR_MAIN_0_VBUSP_CFG0_START                 0x100000
#define CSL_FW_IMAIN_CTRL_MMR_MAIN_0_VBUSP_CFG0_END                   0x11ffff

/* Properties of firewall protecting slave endpoint: IMX_DFTSS_WRAP_MAIN_0.VBUSP_CFG*/
#define CSL_FW_IMX_DFTSS_WRAP_MAIN_0_VBUSP_CFG_ID                     10
#define CSL_FW_IMX_DFTSS_WRAP_MAIN_0_VBUSP_CFG_TYPE                   CSL_FW_SECURITY
#define CSL_FW_IMX_DFTSS_WRAP_MAIN_0_VBUSP_CFG_MMR_BASE               0x45002800
#define CSL_FW_IMX_DFTSS_WRAP_MAIN_0_VBUSP_CFG_NUM_REGIONS            1
#define CSL_FW_IMX_DFTSS_WRAP_MAIN_0_VBUSP_CFG_NUM_PRIV_IDS_PER_REGION          3
#define CSL_FW_IMX_DFTSS_WRAP_MAIN_0_VBUSP_CFG_NUM_PROTECTED_HW_REGIONS         1
#define CSL_FW_IMX_DFTSS_WRAP_MAIN_0_VBUSP_CFG_MEM_START              0x500000
#define CSL_FW_IMX_DFTSS_WRAP_MAIN_0_VBUSP_CFG_MEM_END                0x5003ff

/* Properties of firewall protecting slave endpoint: IK3_MAIN_EFUSE_MAIN_0.SLV*/
#define CSL_FW_IK3_MAIN_EFUSE_MAIN_0_SLV_ID                           11
#define CSL_FW_IK3_MAIN_EFUSE_MAIN_0_SLV_TYPE                         CSL_FW_SECURITY
#define CSL_FW_IK3_MAIN_EFUSE_MAIN_0_SLV_MMR_BASE                     0x45002c00
#define CSL_FW_IK3_MAIN_EFUSE_MAIN_0_SLV_NUM_REGIONS                  1
#define CSL_FW_IK3_MAIN_EFUSE_MAIN_0_SLV_NUM_PRIV_IDS_PER_REGION      3
#define CSL_FW_IK3_MAIN_EFUSE_MAIN_0_SLV_NUM_PROTECTED_HW_REGIONS     1
#define CSL_FW_IK3_MAIN_EFUSE_MAIN_0_SLV_MEM_START                    0x300000
#define CSL_FW_IK3_MAIN_EFUSE_MAIN_0_SLV_MEM_END                      0x3000ff

/* Properties of firewall protecting slave endpoint: IK3_PBIST_4C28P_WRAP_MAIN_0.CFG*/
#define CSL_FW_IK3_PBIST_4C28P_WRAP_MAIN_0_CFG_ID                     12
#define CSL_FW_IK3_PBIST_4C28P_WRAP_MAIN_0_CFG_TYPE                   CSL_FW_SECURITY
#define CSL_FW_IK3_PBIST_4C28P_WRAP_MAIN_0_CFG_MMR_BASE               0x45003000
#define CSL_FW_IK3_PBIST_4C28P_WRAP_MAIN_0_CFG_NUM_REGIONS            1
#define CSL_FW_IK3_PBIST_4C28P_WRAP_MAIN_0_CFG_NUM_PRIV_IDS_PER_REGION          3
#define CSL_FW_IK3_PBIST_4C28P_WRAP_MAIN_0_CFG_NUM_PROTECTED_HW_REGIONS         1
#define CSL_FW_IK3_PBIST_4C28P_WRAP_MAIN_0_CFG_MEM_START              0x420000
#define CSL_FW_IK3_PBIST_4C28P_WRAP_MAIN_0_CFG_MEM_END                0x4203ff

/* Properties of firewall protecting slave endpoint: IK3_PBIST_4C28P_WRAP_MAIN_1.CFG*/
#define CSL_FW_IK3_PBIST_4C28P_WRAP_MAIN_1_CFG_ID                     13
#define CSL_FW_IK3_PBIST_4C28P_WRAP_MAIN_1_CFG_TYPE                   CSL_FW_SECURITY
#define CSL_FW_IK3_PBIST_4C28P_WRAP_MAIN_1_CFG_MMR_BASE               0x45003400
#define CSL_FW_IK3_PBIST_4C28P_WRAP_MAIN_1_CFG_NUM_REGIONS            1
#define CSL_FW_IK3_PBIST_4C28P_WRAP_MAIN_1_CFG_NUM_PRIV_IDS_PER_REGION          3
#define CSL_FW_IK3_PBIST_4C28P_WRAP_MAIN_1_CFG_NUM_PROTECTED_HW_REGIONS         1
#define CSL_FW_IK3_PBIST_4C28P_WRAP_MAIN_1_CFG_MEM_START              0x430000
#define CSL_FW_IK3_PBIST_4C28P_WRAP_MAIN_1_CFG_MEM_END                0x4303ff

/* Properties of firewall protecting slave endpoint: IGPIO_144_MAIN_0.MMR*/
#define CSL_FW_IGPIO_144_MAIN_0_MMR_ID                                16
#define CSL_FW_IGPIO_144_MAIN_0_MMR_TYPE                              CSL_FW_SECURITY
#define CSL_FW_IGPIO_144_MAIN_0_MMR_MMR_BASE                          0x45004000
#define CSL_FW_IGPIO_144_MAIN_0_MMR_NUM_REGIONS                       1
#define CSL_FW_IGPIO_144_MAIN_0_MMR_NUM_PRIV_IDS_PER_REGION           3
#define CSL_FW_IGPIO_144_MAIN_0_MMR_NUM_PROTECTED_HW_REGIONS          1
#define CSL_FW_IGPIO_144_MAIN_0_MMR_MEM_START                         0x600000
#define CSL_FW_IGPIO_144_MAIN_0_MMR_MEM_END                           0x6000ff

/* Properties of firewall protecting slave endpoint: IGPIO_144_MAIN_1.MMR*/
#define CSL_FW_IGPIO_144_MAIN_1_MMR_ID                                17
#define CSL_FW_IGPIO_144_MAIN_1_MMR_TYPE                              CSL_FW_SECURITY
#define CSL_FW_IGPIO_144_MAIN_1_MMR_MMR_BASE                          0x45004400
#define CSL_FW_IGPIO_144_MAIN_1_MMR_NUM_REGIONS                       1
#define CSL_FW_IGPIO_144_MAIN_1_MMR_NUM_PRIV_IDS_PER_REGION           3
#define CSL_FW_IGPIO_144_MAIN_1_MMR_NUM_PROTECTED_HW_REGIONS          1
#define CSL_FW_IGPIO_144_MAIN_1_MMR_MEM_START                         0x601000
#define CSL_FW_IGPIO_144_MAIN_1_MMR_MEM_END                           0x6010ff

/* Properties of firewall protecting slave endpoint: IESM_MAIN_MAIN_0.ESM_CFG*/
#define CSL_FW_IESM_MAIN_MAIN_0_ESM_CFG_ID                            24
#define CSL_FW_IESM_MAIN_MAIN_0_ESM_CFG_TYPE                          CSL_FW_SECURITY
#define CSL_FW_IESM_MAIN_MAIN_0_ESM_CFG_MMR_BASE                      0x45006000
#define CSL_FW_IESM_MAIN_MAIN_0_ESM_CFG_NUM_REGIONS                   1
#define CSL_FW_IESM_MAIN_MAIN_0_ESM_CFG_NUM_PRIV_IDS_PER_REGION       3
#define CSL_FW_IESM_MAIN_MAIN_0_ESM_CFG_NUM_PROTECTED_HW_REGIONS      1
#define CSL_FW_IESM_MAIN_MAIN_0_ESM_CFG_CFG_START                     0x700000
#define CSL_FW_IESM_MAIN_MAIN_0_ESM_CFG_CFG_END                       0x700fff

/* Properties of firewall protecting slave endpoint: IDCC_MAIN_0.DCC_CBA*/
#define CSL_FW_IDCC_MAIN_0_DCC_CBA_ID                                 32
#define CSL_FW_IDCC_MAIN_0_DCC_CBA_TYPE                               CSL_FW_SECURITY
#define CSL_FW_IDCC_MAIN_0_DCC_CBA_MMR_BASE                           0x45008000
#define CSL_FW_IDCC_MAIN_0_DCC_CBA_NUM_REGIONS                        1
#define CSL_FW_IDCC_MAIN_0_DCC_CBA_NUM_PRIV_IDS_PER_REGION            3
#define CSL_FW_IDCC_MAIN_0_DCC_CBA_NUM_PROTECTED_HW_REGIONS           1
#define CSL_FW_IDCC_MAIN_0_DCC_CBA_MEM_START                          0x800000
#define CSL_FW_IDCC_MAIN_0_DCC_CBA_MEM_END                            0x80003f

/* Properties of firewall protecting slave endpoint: IDCC_MAIN_1.DCC_CBA*/
#define CSL_FW_IDCC_MAIN_1_DCC_CBA_ID                                 33
#define CSL_FW_IDCC_MAIN_1_DCC_CBA_TYPE                               CSL_FW_SECURITY
#define CSL_FW_IDCC_MAIN_1_DCC_CBA_MMR_BASE                           0x45008400
#define CSL_FW_IDCC_MAIN_1_DCC_CBA_NUM_REGIONS                        1
#define CSL_FW_IDCC_MAIN_1_DCC_CBA_NUM_PRIV_IDS_PER_REGION            3
#define CSL_FW_IDCC_MAIN_1_DCC_CBA_NUM_PROTECTED_HW_REGIONS           1
#define CSL_FW_IDCC_MAIN_1_DCC_CBA_MEM_START                          0x804000
#define CSL_FW_IDCC_MAIN_1_DCC_CBA_MEM_END                            0x80403f

/* Properties of firewall protecting slave endpoint: IDCC_MAIN_2.DCC_CBA*/
#define CSL_FW_IDCC_MAIN_2_DCC_CBA_ID                                 34
#define CSL_FW_IDCC_MAIN_2_DCC_CBA_TYPE                               CSL_FW_SECURITY
#define CSL_FW_IDCC_MAIN_2_DCC_CBA_MMR_BASE                           0x45008800
#define CSL_FW_IDCC_MAIN_2_DCC_CBA_NUM_REGIONS                        1
#define CSL_FW_IDCC_MAIN_2_DCC_CBA_NUM_PRIV_IDS_PER_REGION            3
#define CSL_FW_IDCC_MAIN_2_DCC_CBA_NUM_PROTECTED_HW_REGIONS           1
#define CSL_FW_IDCC_MAIN_2_DCC_CBA_MEM_START                          0x808000
#define CSL_FW_IDCC_MAIN_2_DCC_CBA_MEM_END                            0x80803f

/* Properties of firewall protecting slave endpoint: IDCC_MAIN_3.DCC_CBA*/
#define CSL_FW_IDCC_MAIN_3_DCC_CBA_ID                                 35
#define CSL_FW_IDCC_MAIN_3_DCC_CBA_TYPE                               CSL_FW_SECURITY
#define CSL_FW_IDCC_MAIN_3_DCC_CBA_MMR_BASE                           0x45008c00
#define CSL_FW_IDCC_MAIN_3_DCC_CBA_NUM_REGIONS                        1
#define CSL_FW_IDCC_MAIN_3_DCC_CBA_NUM_PRIV_IDS_PER_REGION            3
#define CSL_FW_IDCC_MAIN_3_DCC_CBA_NUM_PROTECTED_HW_REGIONS           1
#define CSL_FW_IDCC_MAIN_3_DCC_CBA_MEM_START                          0x80c000
#define CSL_FW_IDCC_MAIN_3_DCC_CBA_MEM_END                            0x80c03f

/* Properties of firewall protecting slave endpoint: IDCC_MAIN_4.DCC_CBA*/
#define CSL_FW_IDCC_MAIN_4_DCC_CBA_ID                                 36
#define CSL_FW_IDCC_MAIN_4_DCC_CBA_TYPE                               CSL_FW_SECURITY
#define CSL_FW_IDCC_MAIN_4_DCC_CBA_MMR_BASE                           0x45009000
#define CSL_FW_IDCC_MAIN_4_DCC_CBA_NUM_REGIONS                        1
#define CSL_FW_IDCC_MAIN_4_DCC_CBA_NUM_PRIV_IDS_PER_REGION            3
#define CSL_FW_IDCC_MAIN_4_DCC_CBA_NUM_PROTECTED_HW_REGIONS           1
#define CSL_FW_IDCC_MAIN_4_DCC_CBA_MEM_START                          0x810000
#define CSL_FW_IDCC_MAIN_4_DCC_CBA_MEM_END                            0x81003f

/* Properties of firewall protecting slave endpoint: IDCC_MAIN_5.DCC_CBA*/
#define CSL_FW_IDCC_MAIN_5_DCC_CBA_ID                                 37
#define CSL_FW_IDCC_MAIN_5_DCC_CBA_TYPE                               CSL_FW_SECURITY
#define CSL_FW_IDCC_MAIN_5_DCC_CBA_MMR_BASE                           0x45009400
#define CSL_FW_IDCC_MAIN_5_DCC_CBA_NUM_REGIONS                        1
#define CSL_FW_IDCC_MAIN_5_DCC_CBA_NUM_PRIV_IDS_PER_REGION            3
#define CSL_FW_IDCC_MAIN_5_DCC_CBA_NUM_PROTECTED_HW_REGIONS           1
#define CSL_FW_IDCC_MAIN_5_DCC_CBA_MEM_START                          0x814000
#define CSL_FW_IDCC_MAIN_5_DCC_CBA_MEM_END                            0x81403f

/* Properties of firewall protecting slave endpoint: IDCC_MAIN_6.DCC_CBA*/
#define CSL_FW_IDCC_MAIN_6_DCC_CBA_ID                                 38
#define CSL_FW_IDCC_MAIN_6_DCC_CBA_TYPE                               CSL_FW_SECURITY
#define CSL_FW_IDCC_MAIN_6_DCC_CBA_MMR_BASE                           0x45009800
#define CSL_FW_IDCC_MAIN_6_DCC_CBA_NUM_REGIONS                        1
#define CSL_FW_IDCC_MAIN_6_DCC_CBA_NUM_PRIV_IDS_PER_REGION            3
#define CSL_FW_IDCC_MAIN_6_DCC_CBA_NUM_PROTECTED_HW_REGIONS           1
#define CSL_FW_IDCC_MAIN_6_DCC_CBA_MEM_START                          0x818000
#define CSL_FW_IDCC_MAIN_6_DCC_CBA_MEM_END                            0x81803f

/* Properties of firewall protecting slave endpoint: IDCC_MAIN_7.DCC_CBA*/
#define CSL_FW_IDCC_MAIN_7_DCC_CBA_ID                                 39
#define CSL_FW_IDCC_MAIN_7_DCC_CBA_TYPE                               CSL_FW_SECURITY
#define CSL_FW_IDCC_MAIN_7_DCC_CBA_MMR_BASE                           0x45009c00
#define CSL_FW_IDCC_MAIN_7_DCC_CBA_NUM_REGIONS                        1
#define CSL_FW_IDCC_MAIN_7_DCC_CBA_NUM_PRIV_IDS_PER_REGION            3
#define CSL_FW_IDCC_MAIN_7_DCC_CBA_NUM_PROTECTED_HW_REGIONS           1
#define CSL_FW_IDCC_MAIN_7_DCC_CBA_MEM_START                          0x81c000
#define CSL_FW_IDCC_MAIN_7_DCC_CBA_MEM_END                            0x81c03f

/* Properties of firewall protecting slave endpoint: IWIZ8B2M4VSB_MAIN_0.SLV*/
#define CSL_FW_IWIZ8B2M4VSB_MAIN_0_SLV_ID                             48
#define CSL_FW_IWIZ8B2M4VSB_MAIN_0_SLV_TYPE                           CSL_FW_SECURITY
#define CSL_FW_IWIZ8B2M4VSB_MAIN_0_SLV_MMR_BASE                       0x4500c000
#define CSL_FW_IWIZ8B2M4VSB_MAIN_0_SLV_NUM_REGIONS                    1
#define CSL_FW_IWIZ8B2M4VSB_MAIN_0_SLV_NUM_PRIV_IDS_PER_REGION        3
#define CSL_FW_IWIZ8B2M4VSB_MAIN_0_SLV_NUM_PROTECTED_HW_REGIONS       1
#define CSL_FW_IWIZ8B2M4VSB_MAIN_0_SLV_WIZ8B2M4VSB_START              0x900000
#define CSL_FW_IWIZ8B2M4VSB_MAIN_0_SLV_WIZ8B2M4VSB_END                0x901fff

/* Properties of firewall protecting slave endpoint: IWIZ8B2M4VSB_MAIN_1.SLV*/
#define CSL_FW_IWIZ8B2M4VSB_MAIN_1_SLV_ID                             49
#define CSL_FW_IWIZ8B2M4VSB_MAIN_1_SLV_TYPE                           CSL_FW_SECURITY
#define CSL_FW_IWIZ8B2M4VSB_MAIN_1_SLV_MMR_BASE                       0x4500c400
#define CSL_FW_IWIZ8B2M4VSB_MAIN_1_SLV_NUM_REGIONS                    1
#define CSL_FW_IWIZ8B2M4VSB_MAIN_1_SLV_NUM_PRIV_IDS_PER_REGION        3
#define CSL_FW_IWIZ8B2M4VSB_MAIN_1_SLV_NUM_PROTECTED_HW_REGIONS       1
#define CSL_FW_IWIZ8B2M4VSB_MAIN_1_SLV_WIZ8B2M4VSB_START              0x910000
#define CSL_FW_IWIZ8B2M4VSB_MAIN_1_SLV_WIZ8B2M4VSB_END                0x911fff

/* Properties of firewall protecting slave endpoint: IMAIN_GPIOMUX_INTROUTER_MAIN_0.CFG*/
#define CSL_FW_IMAIN_GPIOMUX_INTROUTER_MAIN_0_CFG_ID                  56
#define CSL_FW_IMAIN_GPIOMUX_INTROUTER_MAIN_0_CFG_TYPE                CSL_FW_SECURITY
#define CSL_FW_IMAIN_GPIOMUX_INTROUTER_MAIN_0_CFG_MMR_BASE            0x4500e000
#define CSL_FW_IMAIN_GPIOMUX_INTROUTER_MAIN_0_CFG_NUM_REGIONS         1
#define CSL_FW_IMAIN_GPIOMUX_INTROUTER_MAIN_0_CFG_NUM_PRIV_IDS_PER_REGION       3
#define CSL_FW_IMAIN_GPIOMUX_INTROUTER_MAIN_0_CFG_NUM_PROTECTED_HW_REGIONS      1
#define CSL_FW_IMAIN_GPIOMUX_INTROUTER_MAIN_0_CFG_INTR_ROUTER_CFG_START         0xa00000
#define CSL_FW_IMAIN_GPIOMUX_INTROUTER_MAIN_0_CFG_INTR_ROUTER_CFG_END 0xa003ff

/* Properties of firewall protecting slave endpoint: IMAIN2MCU_LVL_INTROUTER_MAIN_0.CFG*/
#define CSL_FW_IMAIN2MCU_LVL_INTROUTER_MAIN_0_CFG_ID                  57
#define CSL_FW_IMAIN2MCU_LVL_INTROUTER_MAIN_0_CFG_TYPE                CSL_FW_SECURITY
#define CSL_FW_IMAIN2MCU_LVL_INTROUTER_MAIN_0_CFG_MMR_BASE            0x4500e400
#define CSL_FW_IMAIN2MCU_LVL_INTROUTER_MAIN_0_CFG_NUM_REGIONS         1
#define CSL_FW_IMAIN2MCU_LVL_INTROUTER_MAIN_0_CFG_NUM_PRIV_IDS_PER_REGION       3
#define CSL_FW_IMAIN2MCU_LVL_INTROUTER_MAIN_0_CFG_NUM_PROTECTED_HW_REGIONS      1
#define CSL_FW_IMAIN2MCU_LVL_INTROUTER_MAIN_0_CFG_INTR_ROUTER_CFG_START         0xa10000
#define CSL_FW_IMAIN2MCU_LVL_INTROUTER_MAIN_0_CFG_INTR_ROUTER_CFG_END 0xa107ff

/* Properties of firewall protecting slave endpoint: IMAIN2MCU_PLS_INTROUTER_MAIN_0.CFG*/
#define CSL_FW_IMAIN2MCU_PLS_INTROUTER_MAIN_0_CFG_ID                  58
#define CSL_FW_IMAIN2MCU_PLS_INTROUTER_MAIN_0_CFG_TYPE                CSL_FW_SECURITY
#define CSL_FW_IMAIN2MCU_PLS_INTROUTER_MAIN_0_CFG_MMR_BASE            0x4500e800
#define CSL_FW_IMAIN2MCU_PLS_INTROUTER_MAIN_0_CFG_NUM_REGIONS         1
#define CSL_FW_IMAIN2MCU_PLS_INTROUTER_MAIN_0_CFG_NUM_PRIV_IDS_PER_REGION       3
#define CSL_FW_IMAIN2MCU_PLS_INTROUTER_MAIN_0_CFG_NUM_PROTECTED_HW_REGIONS      1
#define CSL_FW_IMAIN2MCU_PLS_INTROUTER_MAIN_0_CFG_INTR_ROUTER_CFG_START         0xa20000
#define CSL_FW_IMAIN2MCU_PLS_INTROUTER_MAIN_0_CFG_INTR_ROUTER_CFG_END 0xa207ff

/* Properties of firewall protecting slave endpoint: ICMP_EVENT_INTROUTER_MAIN_0.CFG*/
#define CSL_FW_ICMP_EVENT_INTROUTER_MAIN_0_CFG_ID                     59
#define CSL_FW_ICMP_EVENT_INTROUTER_MAIN_0_CFG_TYPE                   CSL_FW_SECURITY
#define CSL_FW_ICMP_EVENT_INTROUTER_MAIN_0_CFG_MMR_BASE               0x4500ec00
#define CSL_FW_ICMP_EVENT_INTROUTER_MAIN_0_CFG_NUM_REGIONS            1
#define CSL_FW_ICMP_EVENT_INTROUTER_MAIN_0_CFG_NUM_PRIV_IDS_PER_REGION          3
#define CSL_FW_ICMP_EVENT_INTROUTER_MAIN_0_CFG_NUM_PROTECTED_HW_REGIONS         1
#define CSL_FW_ICMP_EVENT_INTROUTER_MAIN_0_CFG_INTR_ROUTER_CFG_START  0xa30000
#define CSL_FW_ICMP_EVENT_INTROUTER_MAIN_0_CFG_INTR_ROUTER_CFG_END    0xa303ff

/* Properties of firewall protecting slave endpoint: ITIMESYNC_EVENT_INTROUTER_MAIN_0.CFG*/
#define CSL_FW_ITIMESYNC_EVENT_INTROUTER_MAIN_0_CFG_ID                60
#define CSL_FW_ITIMESYNC_EVENT_INTROUTER_MAIN_0_CFG_TYPE              CSL_FW_SECURITY
#define CSL_FW_ITIMESYNC_EVENT_INTROUTER_MAIN_0_CFG_MMR_BASE          0x4500f000
#define CSL_FW_ITIMESYNC_EVENT_INTROUTER_MAIN_0_CFG_NUM_REGIONS       1
#define CSL_FW_ITIMESYNC_EVENT_INTROUTER_MAIN_0_CFG_NUM_PRIV_IDS_PER_REGION     3
#define CSL_FW_ITIMESYNC_EVENT_INTROUTER_MAIN_0_CFG_NUM_PROTECTED_HW_REGIONS    1
#define CSL_FW_ITIMESYNC_EVENT_INTROUTER_MAIN_0_CFG_INTR_ROUTER_CFG_START       0xa40000
#define CSL_FW_ITIMESYNC_EVENT_INTROUTER_MAIN_0_CFG_INTR_ROUTER_CFG_END         0xa407ff

/* Properties of firewall protecting slave endpoint: IM4_MAIN_FW_CBASS_MAIN_0.CBASS_ERR_SLV*/
#define CSL_FW_IM4_MAIN_FW_CBASS_MAIN_0_CBASS_ERR_SLV_ID              73
#define CSL_FW_IM4_MAIN_FW_CBASS_MAIN_0_CBASS_ERR_SLV_TYPE            CSL_FW_SECURITY
#define CSL_FW_IM4_MAIN_FW_CBASS_MAIN_0_CBASS_ERR_SLV_MMR_BASE        0x45012400
#define CSL_FW_IM4_MAIN_FW_CBASS_MAIN_0_CBASS_ERR_SLV_NUM_REGIONS     1
#define CSL_FW_IM4_MAIN_FW_CBASS_MAIN_0_CBASS_ERR_SLV_NUM_PRIV_IDS_PER_REGION   3
#define CSL_FW_IM4_MAIN_FW_CBASS_MAIN_0_CBASS_ERR_SLV_NUM_PROTECTED_HW_REGIONS  1
#define CSL_FW_IM4_MAIN_FW_CBASS_MAIN_0_CBASS_ERR_SLV_ERR_REGS_START  0xb01000
#define CSL_FW_IM4_MAIN_FW_CBASS_MAIN_0_CBASS_ERR_SLV_ERR_REGS_END    0xb013ff

/* Properties of firewall protecting slave endpoint: IM4_MAIN_INFRA_CBASS_MAIN_0.CBASS_ERR_SLV*/
#define CSL_FW_IM4_MAIN_INFRA_CBASS_MAIN_0_CBASS_ERR_SLV_ID           74
#define CSL_FW_IM4_MAIN_INFRA_CBASS_MAIN_0_CBASS_ERR_SLV_TYPE         CSL_FW_SECURITY
#define CSL_FW_IM4_MAIN_INFRA_CBASS_MAIN_0_CBASS_ERR_SLV_MMR_BASE     0x45012800
#define CSL_FW_IM4_MAIN_INFRA_CBASS_MAIN_0_CBASS_ERR_SLV_NUM_REGIONS  1
#define CSL_FW_IM4_MAIN_INFRA_CBASS_MAIN_0_CBASS_ERR_SLV_NUM_PRIV_IDS_PER_REGION          3
#define CSL_FW_IM4_MAIN_INFRA_CBASS_MAIN_0_CBASS_ERR_SLV_NUM_PROTECTED_HW_REGIONS         1
#define CSL_FW_IM4_MAIN_INFRA_CBASS_MAIN_0_CBASS_ERR_SLV_ERR_REGS_START         0xb00000
#define CSL_FW_IM4_MAIN_INFRA_CBASS_MAIN_0_CBASS_ERR_SLV_ERR_REGS_END 0xb003ff

/* Properties of firewall protecting slave endpoint: IM4_GTCCLK_ECC_AGGR_MAIN_0.CFG*/
#define CSL_FW_IM4_GTCCLK_ECC_AGGR_MAIN_0_CFG_ID                      75
#define CSL_FW_IM4_GTCCLK_ECC_AGGR_MAIN_0_CFG_TYPE                    CSL_FW_SECURITY
#define CSL_FW_IM4_GTCCLK_ECC_AGGR_MAIN_0_CFG_MMR_BASE                0x45012c00
#define CSL_FW_IM4_GTCCLK_ECC_AGGR_MAIN_0_CFG_NUM_REGIONS             1
#define CSL_FW_IM4_GTCCLK_ECC_AGGR_MAIN_0_CFG_NUM_PRIV_IDS_PER_REGION 3
#define CSL_FW_IM4_GTCCLK_ECC_AGGR_MAIN_0_CFG_NUM_PROTECTED_HW_REGIONS          1
#define CSL_FW_IM4_GTCCLK_ECC_AGGR_MAIN_0_CFG_REGS_START              0xc01000
#define CSL_FW_IM4_GTCCLK_ECC_AGGR_MAIN_0_CFG_REGS_END                0xc013ff

/* Properties of firewall protecting slave endpoint: IM4_MAIN_INFRACLK4_ECC_AGGR_MAIN_0.CFG*/
#define CSL_FW_IM4_MAIN_INFRACLK4_ECC_AGGR_MAIN_0_CFG_ID              76
#define CSL_FW_IM4_MAIN_INFRACLK4_ECC_AGGR_MAIN_0_CFG_TYPE            CSL_FW_SECURITY
#define CSL_FW_IM4_MAIN_INFRACLK4_ECC_AGGR_MAIN_0_CFG_MMR_BASE        0x45013000
#define CSL_FW_IM4_MAIN_INFRACLK4_ECC_AGGR_MAIN_0_CFG_NUM_REGIONS     1
#define CSL_FW_IM4_MAIN_INFRACLK4_ECC_AGGR_MAIN_0_CFG_NUM_PRIV_IDS_PER_REGION   3
#define CSL_FW_IM4_MAIN_INFRACLK4_ECC_AGGR_MAIN_0_CFG_NUM_PROTECTED_HW_REGIONS  1
#define CSL_FW_IM4_MAIN_INFRACLK4_ECC_AGGR_MAIN_0_CFG_REGS_START      0xc00000
#define CSL_FW_IM4_MAIN_INFRACLK4_ECC_AGGR_MAIN_0_CFG_REGS_END        0xc003ff

/* Properties of firewall protecting slave endpoint: IK3_WKUP_PSC_WRAP_WKUP_0.VBUS*/
#define CSL_FW_IK3_WKUP_PSC_WRAP_WKUP_0_VBUS_ID                       129
#define CSL_FW_IK3_WKUP_PSC_WRAP_WKUP_0_VBUS_TYPE                     CSL_FW_SECURITY
#define CSL_FW_IK3_WKUP_PSC_WRAP_WKUP_0_VBUS_MMR_BASE                 0x45020400
#define CSL_FW_IK3_WKUP_PSC_WRAP_WKUP_0_VBUS_NUM_REGIONS              1
#define CSL_FW_IK3_WKUP_PSC_WRAP_WKUP_0_VBUS_NUM_PRIV_IDS_PER_REGION  3
#define CSL_FW_IK3_WKUP_PSC_WRAP_WKUP_0_VBUS_NUM_PROTECTED_HW_REGIONS 1
#define CSL_FW_IK3_WKUP_PSC_WRAP_WKUP_0_VBUS_VBUS_START               0x42000000
#define CSL_FW_IK3_WKUP_PSC_WRAP_WKUP_0_VBUS_VBUS_END                 0x42000fff

/* Properties of firewall protecting slave endpoint: IK3_PLL_CTRL_WRAP_WKUP_0.VBUS_SLV*/
#define CSL_FW_IK3_PLL_CTRL_WRAP_WKUP_0_VBUS_SLV_ID                   130
#define CSL_FW_IK3_PLL_CTRL_WRAP_WKUP_0_VBUS_SLV_TYPE                 CSL_FW_SECURITY
#define CSL_FW_IK3_PLL_CTRL_WRAP_WKUP_0_VBUS_SLV_MMR_BASE             0x45020800
#define CSL_FW_IK3_PLL_CTRL_WRAP_WKUP_0_VBUS_SLV_NUM_REGIONS          1
#define CSL_FW_IK3_PLL_CTRL_WRAP_WKUP_0_VBUS_SLV_NUM_PRIV_IDS_PER_REGION        3
#define CSL_FW_IK3_PLL_CTRL_WRAP_WKUP_0_VBUS_SLV_NUM_PROTECTED_HW_REGIONS       1
#define CSL_FW_IK3_PLL_CTRL_WRAP_WKUP_0_VBUS_SLV_MEM_START            0x42010000
#define CSL_FW_IK3_PLL_CTRL_WRAP_WKUP_0_VBUS_SLV_MEM_END              0x420101ff

/* Properties of firewall protecting slave endpoint: IWKUP_CTRL_MMR_WKUP_0.VBUSP*/
#define CSL_FW_IWKUP_CTRL_MMR_WKUP_0_VBUSP_ID                         131
#define CSL_FW_IWKUP_CTRL_MMR_WKUP_0_VBUSP_TYPE                       CSL_FW_SECURITY
#define CSL_FW_IWKUP_CTRL_MMR_WKUP_0_VBUSP_MMR_BASE                   0x45020c00
#define CSL_FW_IWKUP_CTRL_MMR_WKUP_0_VBUSP_NUM_REGIONS                16
#define CSL_FW_IWKUP_CTRL_MMR_WKUP_0_VBUSP_NUM_PRIV_IDS_PER_REGION    3
#define CSL_FW_IWKUP_CTRL_MMR_WKUP_0_VBUSP_NUM_PROTECTED_HW_REGIONS   1
#define CSL_FW_IWKUP_CTRL_MMR_WKUP_0_VBUSP_CFG0_START                 0x43000000
#define CSL_FW_IWKUP_CTRL_MMR_WKUP_0_VBUSP_CFG0_END                   0x4301ffff

/* Properties of firewall protecting slave endpoint: IGPIO_144_WKUP_0.MMR*/
#define CSL_FW_IGPIO_144_WKUP_0_MMR_ID                                132
#define CSL_FW_IGPIO_144_WKUP_0_MMR_TYPE                              CSL_FW_SECURITY
#define CSL_FW_IGPIO_144_WKUP_0_MMR_MMR_BASE                          0x45021000
#define CSL_FW_IGPIO_144_WKUP_0_MMR_NUM_REGIONS                       1
#define CSL_FW_IGPIO_144_WKUP_0_MMR_NUM_PRIV_IDS_PER_REGION           3
#define CSL_FW_IGPIO_144_WKUP_0_MMR_NUM_PROTECTED_HW_REGIONS          1
#define CSL_FW_IGPIO_144_WKUP_0_MMR_MEM_START                         0x42110000
#define CSL_FW_IGPIO_144_WKUP_0_MMR_MEM_END                           0x421100ff

/* Properties of firewall protecting slave endpoint: IESM_WKUP_WKUP_0.ESM_CFG*/
#define CSL_FW_IESM_WKUP_WKUP_0_ESM_CFG_ID                            133
#define CSL_FW_IESM_WKUP_WKUP_0_ESM_CFG_TYPE                          CSL_FW_SECURITY
#define CSL_FW_IESM_WKUP_WKUP_0_ESM_CFG_MMR_BASE                      0x45021400
#define CSL_FW_IESM_WKUP_WKUP_0_ESM_CFG_NUM_REGIONS                   1
#define CSL_FW_IESM_WKUP_WKUP_0_ESM_CFG_NUM_PRIV_IDS_PER_REGION       3
#define CSL_FW_IESM_WKUP_WKUP_0_ESM_CFG_NUM_PROTECTED_HW_REGIONS      1
#define CSL_FW_IESM_WKUP_WKUP_0_ESM_CFG_CFG_START                     0x42080000
#define CSL_FW_IESM_WKUP_WKUP_0_ESM_CFG_CFG_END                       0x42080fff

/* Properties of firewall protecting slave endpoint: IK3VTM_WKUP_0.VBUSP*/
#define CSL_FW_IK3VTM_WKUP_0_VBUSP_ID                                 135
#define CSL_FW_IK3VTM_WKUP_0_VBUSP_TYPE                               CSL_FW_SECURITY
#define CSL_FW_IK3VTM_WKUP_0_VBUSP_MMR_BASE                           0x45021c00
#define CSL_FW_IK3VTM_WKUP_0_VBUSP_NUM_REGIONS                        2
#define CSL_FW_IK3VTM_WKUP_0_VBUSP_NUM_PRIV_IDS_PER_REGION            3
#define CSL_FW_IK3VTM_WKUP_0_VBUSP_NUM_PROTECTED_HW_REGIONS           2
#define CSL_FW_IK3VTM_WKUP_0_VBUSP_CFG0_START                         0x42040000
#define CSL_FW_IK3VTM_WKUP_0_VBUSP_CFG0_END                           0x420403ff
#define CSL_FW_IK3VTM_WKUP_0_VBUSP_CFG1_START                         0x42050000
#define CSL_FW_IK3VTM_WKUP_0_VBUSP_CFG1_END                           0x420503ff

/* Properties of firewall protecting slave endpoint: IMSHSI2C_WKUP_0.VBUSP*/
#define CSL_FW_IMSHSI2C_WKUP_0_VBUSP_ID                               144
#define CSL_FW_IMSHSI2C_WKUP_0_VBUSP_TYPE                             CSL_FW_SECURITY
#define CSL_FW_IMSHSI2C_WKUP_0_VBUSP_MMR_BASE                         0x45024000
#define CSL_FW_IMSHSI2C_WKUP_0_VBUSP_NUM_REGIONS                      1
#define CSL_FW_IMSHSI2C_WKUP_0_VBUSP_NUM_PRIV_IDS_PER_REGION          3
#define CSL_FW_IMSHSI2C_WKUP_0_VBUSP_NUM_PROTECTED_HW_REGIONS         1
#define CSL_FW_IMSHSI2C_WKUP_0_VBUSP_CFG_START                        0x42120000
#define CSL_FW_IMSHSI2C_WKUP_0_VBUSP_CFG_END                          0x421200ff

/* Properties of firewall protecting slave endpoint: IUSART_WKUP_0.VBUSP_CBA*/
#define CSL_FW_IUSART_WKUP_0_VBUSP_CBA_ID                             160
#define CSL_FW_IUSART_WKUP_0_VBUSP_CBA_TYPE                           CSL_FW_SECURITY
#define CSL_FW_IUSART_WKUP_0_VBUSP_CBA_MMR_BASE                       0x45028000
#define CSL_FW_IUSART_WKUP_0_VBUSP_CBA_NUM_REGIONS                    1
#define CSL_FW_IUSART_WKUP_0_VBUSP_CBA_NUM_PRIV_IDS_PER_REGION        3
#define CSL_FW_IUSART_WKUP_0_VBUSP_CBA_NUM_PROTECTED_HW_REGIONS       1
#define CSL_FW_IUSART_WKUP_0_VBUSP_CBA_MEM_START                      0x42300000
#define CSL_FW_IUSART_WKUP_0_VBUSP_CBA_MEM_END                        0x423001ff

/* Properties of firewall protecting slave endpoint: IWKUP_GPIOMUX_INTROUTER_WKUP_0.CFG*/
#define CSL_FW_IWKUP_GPIOMUX_INTROUTER_WKUP_0_CFG_ID                  168
#define CSL_FW_IWKUP_GPIOMUX_INTROUTER_WKUP_0_CFG_TYPE                CSL_FW_SECURITY
#define CSL_FW_IWKUP_GPIOMUX_INTROUTER_WKUP_0_CFG_MMR_BASE            0x4502a000
#define CSL_FW_IWKUP_GPIOMUX_INTROUTER_WKUP_0_CFG_NUM_REGIONS         1
#define CSL_FW_IWKUP_GPIOMUX_INTROUTER_WKUP_0_CFG_NUM_PRIV_IDS_PER_REGION       3
#define CSL_FW_IWKUP_GPIOMUX_INTROUTER_WKUP_0_CFG_NUM_PROTECTED_HW_REGIONS      1
#define CSL_FW_IWKUP_GPIOMUX_INTROUTER_WKUP_0_CFG_INTR_ROUTER_CFG_START         0x42200000
#define CSL_FW_IWKUP_GPIOMUX_INTROUTER_WKUP_0_CFG_INTR_ROUTER_CFG_END 0x422001ff

/* Properties of firewall protecting slave endpoint: IM4_WKUP_CBASS_WKUP_0.CBASS_ERR_SLV*/
#define CSL_FW_IM4_WKUP_CBASS_WKUP_0_CBASS_ERR_SLV_ID                 176
#define CSL_FW_IM4_WKUP_CBASS_WKUP_0_CBASS_ERR_SLV_TYPE               CSL_FW_SECURITY
#define CSL_FW_IM4_WKUP_CBASS_WKUP_0_CBASS_ERR_SLV_MMR_BASE           0x4502c000
#define CSL_FW_IM4_WKUP_CBASS_WKUP_0_CBASS_ERR_SLV_NUM_REGIONS        1
#define CSL_FW_IM4_WKUP_CBASS_WKUP_0_CBASS_ERR_SLV_NUM_PRIV_IDS_PER_REGION      3
#define CSL_FW_IM4_WKUP_CBASS_WKUP_0_CBASS_ERR_SLV_NUM_PROTECTED_HW_REGIONS     1
#define CSL_FW_IM4_WKUP_CBASS_WKUP_0_CBASS_ERR_SLV_ERR_REGS_START     0x42400000
#define CSL_FW_IM4_WKUP_CBASS_WKUP_0_CBASS_ERR_SLV_ERR_REGS_END       0x424003ff

/* Properties of firewall protecting slave endpoint: IM4_WKUP_FW_CBASS_WKUP_0.CBASS_ERR_SLV*/
#define CSL_FW_IM4_WKUP_FW_CBASS_WKUP_0_CBASS_ERR_SLV_ID              177
#define CSL_FW_IM4_WKUP_FW_CBASS_WKUP_0_CBASS_ERR_SLV_TYPE            CSL_FW_SECURITY
#define CSL_FW_IM4_WKUP_FW_CBASS_WKUP_0_CBASS_ERR_SLV_MMR_BASE        0x4502c400
#define CSL_FW_IM4_WKUP_FW_CBASS_WKUP_0_CBASS_ERR_SLV_NUM_REGIONS     1
#define CSL_FW_IM4_WKUP_FW_CBASS_WKUP_0_CBASS_ERR_SLV_NUM_PRIV_IDS_PER_REGION   3
#define CSL_FW_IM4_WKUP_FW_CBASS_WKUP_0_CBASS_ERR_SLV_NUM_PROTECTED_HW_REGIONS  1
#define CSL_FW_IM4_WKUP_FW_CBASS_WKUP_0_CBASS_ERR_SLV_ERR_REGS_START  0x42404000
#define CSL_FW_IM4_WKUP_FW_CBASS_WKUP_0_CBASS_ERR_SLV_ERR_REGS_END    0x424043ff

/* Properties of firewall protecting slave endpoint: IM4_WKUP_CLK2_ECC_AGGR_WKUP_0.CFG*/
#define CSL_FW_IM4_WKUP_CLK2_ECC_AGGR_WKUP_0_CFG_ID                   178
#define CSL_FW_IM4_WKUP_CLK2_ECC_AGGR_WKUP_0_CFG_TYPE                 CSL_FW_SECURITY
#define CSL_FW_IM4_WKUP_CLK2_ECC_AGGR_WKUP_0_CFG_MMR_BASE             0x4502c800
#define CSL_FW_IM4_WKUP_CLK2_ECC_AGGR_WKUP_0_CFG_NUM_REGIONS          1
#define CSL_FW_IM4_WKUP_CLK2_ECC_AGGR_WKUP_0_CFG_NUM_PRIV_IDS_PER_REGION        3
#define CSL_FW_IM4_WKUP_CLK2_ECC_AGGR_WKUP_0_CFG_NUM_PROTECTED_HW_REGIONS       1
#define CSL_FW_IM4_WKUP_CLK2_ECC_AGGR_WKUP_0_CFG_REGS_START           0x42410000
#define CSL_FW_IM4_WKUP_CLK2_ECC_AGGR_WKUP_0_CFG_REGS_END             0x424103ff

/* Properties of firewall protecting slave endpoint: IDRU0.MMR*/
#define CSL_FW_IDRU0_MMR_ID                                           288
#define CSL_FW_IDRU0_MMR_TYPE                                         CSL_FW_SECURITY
#define CSL_FW_IDRU0_MMR_MMR_BASE                                     0x45048000
#define CSL_FW_IDRU0_MMR_NUM_REGIONS                                  1
#define CSL_FW_IDRU0_MMR_NUM_PRIV_IDS_PER_REGION                      1
#define CSL_FW_IDRU0_MMR_NUM_PROTECTED_HW_REGIONS                     1
#define CSL_FW_IDRU0_MMR_DRU_START                                    0x6d000000
#define CSL_FW_IDRU0_MMR_DRU_END                                      0x6d0dffff

/* Properties of firewall protecting slave endpoint: IROM.SLV*/
#define CSL_FW_IROM_SLV_ID                                            512
#define CSL_FW_IROM_SLV_TYPE                                          CSL_FW_SECURITY
#define CSL_FW_IROM_SLV_MMR_BASE                                      0x45080000
#define CSL_FW_IROM_SLV_NUM_REGIONS                                   2
#define CSL_FW_IROM_SLV_NUM_PRIV_IDS_PER_REGION                       1
#define CSL_FW_IROM_SLV_NUM_PROTECTED_HW_REGIONS                      1
#define CSL_FW_IROM_SLV_ROM_START                                     0x44000000
#define CSL_FW_IROM_SLV_ROM_END                                       0x4403ffff

/* Properties of firewall protecting slave endpoint: IIRAM.SLV*/
#define CSL_FW_IIRAM_SLV_ID                                           513
#define CSL_FW_IIRAM_SLV_TYPE                                         CSL_FW_SECURITY
#define CSL_FW_IIRAM_SLV_MMR_BASE                                     0x45080400
#define CSL_FW_IIRAM_SLV_NUM_REGIONS                                  8
#define CSL_FW_IIRAM_SLV_NUM_PRIV_IDS_PER_REGION                      3
#define CSL_FW_IIRAM_SLV_NUM_PROTECTED_HW_REGIONS                     1
#define CSL_FW_IIRAM_SLV_RAM_START                                    0x44040000
#define CSL_FW_IIRAM_SLV_RAM_END                                      0x4406ffff

/* Properties of firewall protecting slave endpoint: IDRAM0.SLV*/
#define CSL_FW_IDRAM0_SLV_ID                                          514
#define CSL_FW_IDRAM0_SLV_TYPE                                        CSL_FW_SECURITY
#define CSL_FW_IDRAM0_SLV_MMR_BASE                                    0x45080800
#define CSL_FW_IDRAM0_SLV_NUM_REGIONS                                 8
#define CSL_FW_IDRAM0_SLV_NUM_PRIV_IDS_PER_REGION                     3
#define CSL_FW_IDRAM0_SLV_NUM_PROTECTED_HW_REGIONS                    1
#define CSL_FW_IDRAM0_SLV_RAM_START                                   0x44070000
#define CSL_FW_IDRAM0_SLV_RAM_END                                     0x4407ffff

/* Properties of firewall protecting slave endpoint: IDRAM1.SLV*/
#define CSL_FW_IDRAM1_SLV_ID                                          515
#define CSL_FW_IDRAM1_SLV_TYPE                                        CSL_FW_SECURITY
#define CSL_FW_IDRAM1_SLV_MMR_BASE                                    0x45080c00
#define CSL_FW_IDRAM1_SLV_NUM_REGIONS                                 8
#define CSL_FW_IDRAM1_SLV_NUM_PRIV_IDS_PER_REGION                     3
#define CSL_FW_IDRAM1_SLV_NUM_PROTECTED_HW_REGIONS                    1
#define CSL_FW_IDRAM1_SLV_RAM_START                                   0x44080000
#define CSL_FW_IDRAM1_SLV_RAM_END                                     0x44083fff

/* Properties of firewall protecting slave endpoint: IPCFG.SLV*/
#define CSL_FW_IPCFG_SLV_ID                                           528
#define CSL_FW_IPCFG_SLV_TYPE                                         CSL_FW_SECURITY
#define CSL_FW_IPCFG_SLV_MMR_BASE                                     0x45084000
#define CSL_FW_IPCFG_SLV_NUM_REGIONS                                  1
#define CSL_FW_IPCFG_SLV_NUM_PRIV_IDS_PER_REGION                      3
#define CSL_FW_IPCFG_SLV_NUM_PROTECTED_HW_REGIONS                     1
#define CSL_FW_IPCFG_SLV_MEM_START                                    0x44130000
#define CSL_FW_IPCFG_SLV_MEM_END                                      0x441307ff

/* Properties of firewall protecting slave endpoint: ITIMER1.TIMER_CFG_VBP*/
#define CSL_FW_ITIMER1_TIMER_CFG_VBP_ID                               536
#define CSL_FW_ITIMER1_TIMER_CFG_VBP_TYPE                             CSL_FW_SECURITY
#define CSL_FW_ITIMER1_TIMER_CFG_VBP_MMR_BASE                         0x45086000
#define CSL_FW_ITIMER1_TIMER_CFG_VBP_NUM_REGIONS                      1
#define CSL_FW_ITIMER1_TIMER_CFG_VBP_NUM_PRIV_IDS_PER_REGION          3
#define CSL_FW_ITIMER1_TIMER_CFG_VBP_NUM_PROTECTED_HW_REGIONS         1
#define CSL_FW_ITIMER1_TIMER_CFG_VBP_CFG_START                        0x44133000
#define CSL_FW_ITIMER1_TIMER_CFG_VBP_CFG_END                          0x441333ff

/* Properties of firewall protecting slave endpoint: ITIMER2.TIMER_CFG_VBP*/
#define CSL_FW_ITIMER2_TIMER_CFG_VBP_ID                               537
#define CSL_FW_ITIMER2_TIMER_CFG_VBP_TYPE                             CSL_FW_SECURITY
#define CSL_FW_ITIMER2_TIMER_CFG_VBP_MMR_BASE                         0x45086400
#define CSL_FW_ITIMER2_TIMER_CFG_VBP_NUM_REGIONS                      1
#define CSL_FW_ITIMER2_TIMER_CFG_VBP_NUM_PRIV_IDS_PER_REGION          3
#define CSL_FW_ITIMER2_TIMER_CFG_VBP_NUM_PROTECTED_HW_REGIONS         1
#define CSL_FW_ITIMER2_TIMER_CFG_VBP_CFG_START                        0x44134000
#define CSL_FW_ITIMER2_TIMER_CFG_VBP_CFG_END                          0x441343ff

/* Properties of firewall protecting slave endpoint: IWDT.SLV*/
#define CSL_FW_IWDT_SLV_ID                                            544
#define CSL_FW_IWDT_SLV_TYPE                                          CSL_FW_SECURITY
#define CSL_FW_IWDT_SLV_MMR_BASE                                      0x45088000
#define CSL_FW_IWDT_SLV_NUM_REGIONS                                   1
#define CSL_FW_IWDT_SLV_NUM_PRIV_IDS_PER_REGION                       3
#define CSL_FW_IWDT_SLV_NUM_PROTECTED_HW_REGIONS                      1
#define CSL_FW_IWDT_SLV_MEM_START                                     0x44135000
#define CSL_FW_IWDT_SLV_MEM_END                                       0x441350ff

/* Properties of firewall protecting slave endpoint: IWDT_CTRL.SLV*/
#define CSL_FW_IWDT_CTRL_SLV_ID                                       545
#define CSL_FW_IWDT_CTRL_SLV_TYPE                                     CSL_FW_SECURITY
#define CSL_FW_IWDT_CTRL_SLV_MMR_BASE                                 0x45088400
#define CSL_FW_IWDT_CTRL_SLV_NUM_REGIONS                              1
#define CSL_FW_IWDT_CTRL_SLV_NUM_PRIV_IDS_PER_REGION                  3
#define CSL_FW_IWDT_CTRL_SLV_NUM_PROTECTED_HW_REGIONS                 1
#define CSL_FW_IWDT_CTRL_SLV_MEM_START                                0x44135100
#define CSL_FW_IWDT_CTRL_SLV_MEM_END                                  0x441351ff

/* Properties of firewall protecting slave endpoint: IRAT.SLV*/
#define CSL_FW_IRAT_SLV_ID                                            552
#define CSL_FW_IRAT_SLV_TYPE                                          CSL_FW_SECURITY
#define CSL_FW_IRAT_SLV_MMR_BASE                                      0x4508a000
#define CSL_FW_IRAT_SLV_NUM_REGIONS                                   1
#define CSL_FW_IRAT_SLV_NUM_PRIV_IDS_PER_REGION                       3
#define CSL_FW_IRAT_SLV_NUM_PROTECTED_HW_REGIONS                      1
#define CSL_FW_IRAT_SLV_MEM_START                                     0x44200000
#define CSL_FW_IRAT_SLV_MEM_END                                       0x44200fff

/* Properties of firewall protecting slave endpoint: IECC.CFG*/
#define CSL_FW_IECC_CFG_ID                                            562
#define CSL_FW_IECC_CFG_TYPE                                          CSL_FW_SECURITY
#define CSL_FW_IECC_CFG_MMR_BASE                                      0x4508c800
#define CSL_FW_IECC_CFG_NUM_REGIONS                                   2
#define CSL_FW_IECC_CFG_NUM_PRIV_IDS_PER_REGION                       3
#define CSL_FW_IECC_CFG_NUM_PROTECTED_HW_REGIONS                      1
#define CSL_FW_IECC_CFG_REGS_START                                    0x44202000
#define CSL_FW_IECC_CFG_REGS_END                                      0x442023ff

/* Properties of firewall protecting slave endpoint: ISECDBG.SLV*/
#define CSL_FW_ISECDBG_SLV_ID                                         576
#define CSL_FW_ISECDBG_SLV_TYPE                                       CSL_FW_SECURITY
#define CSL_FW_ISECDBG_SLV_MMR_BASE                                   0x45090000
#define CSL_FW_ISECDBG_SLV_NUM_REGIONS                                1
#define CSL_FW_ISECDBG_SLV_NUM_PRIV_IDS_PER_REGION                    3
#define CSL_FW_ISECDBG_SLV_NUM_PROTECTED_HW_REGIONS                   1
#define CSL_FW_ISECDBG_SLV_MEM_START                                  0x44230000
#define CSL_FW_ISECDBG_SLV_MEM_END                                    0x44230fff

/* Properties of firewall protecting slave endpoint: IDBG_AUTH.SLV*/
#define CSL_FW_IDBG_AUTH_SLV_ID                                       578
#define CSL_FW_IDBG_AUTH_SLV_TYPE                                     CSL_FW_SECURITY
#define CSL_FW_IDBG_AUTH_SLV_MMR_BASE                                 0x45090800
#define CSL_FW_IDBG_AUTH_SLV_NUM_REGIONS                              1
#define CSL_FW_IDBG_AUTH_SLV_NUM_PRIV_IDS_PER_REGION                  3
#define CSL_FW_IDBG_AUTH_SLV_NUM_PROTECTED_HW_REGIONS                 1
#define CSL_FW_IDBG_AUTH_SLV_MEM_START                                0x44232000
#define CSL_FW_IDBG_AUTH_SLV_MEM_END                                  0x442320ff

/* Properties of firewall protecting slave endpoint: ISECKEY.SLV*/
#define CSL_FW_ISECKEY_SLV_ID                                         582
#define CSL_FW_ISECKEY_SLV_TYPE                                       CSL_FW_SECURITY
#define CSL_FW_ISECKEY_SLV_MMR_BASE                                   0x45091800
#define CSL_FW_ISECKEY_SLV_NUM_REGIONS                                2
#define CSL_FW_ISECKEY_SLV_NUM_PRIV_IDS_PER_REGION                    3
#define CSL_FW_ISECKEY_SLV_NUM_PROTECTED_HW_REGIONS                   1
#define CSL_FW_ISECKEY_SLV_MEM_START                                  0x44234000
#define CSL_FW_ISECKEY_SLV_MEM_END                                    0x44237fff

/* Properties of firewall protecting slave endpoint: ITIMER3.TIMER_CFG_VBP*/
#define CSL_FW_ITIMER3_TIMER_CFG_VBP_ID                               592
#define CSL_FW_ITIMER3_TIMER_CFG_VBP_TYPE                             CSL_FW_SECURITY
#define CSL_FW_ITIMER3_TIMER_CFG_VBP_MMR_BASE                         0x45094000
#define CSL_FW_ITIMER3_TIMER_CFG_VBP_NUM_REGIONS                      1
#define CSL_FW_ITIMER3_TIMER_CFG_VBP_NUM_PRIV_IDS_PER_REGION          3
#define CSL_FW_ITIMER3_TIMER_CFG_VBP_NUM_PROTECTED_HW_REGIONS         1
#define CSL_FW_ITIMER3_TIMER_CFG_VBP_CFG_START                        0x44238000
#define CSL_FW_ITIMER3_TIMER_CFG_VBP_CFG_END                          0x442383ff

/* Properties of firewall protecting slave endpoint: ITIMER4.TIMER_CFG_VBP*/
#define CSL_FW_ITIMER4_TIMER_CFG_VBP_ID                               593
#define CSL_FW_ITIMER4_TIMER_CFG_VBP_TYPE                             CSL_FW_SECURITY
#define CSL_FW_ITIMER4_TIMER_CFG_VBP_MMR_BASE                         0x45094400
#define CSL_FW_ITIMER4_TIMER_CFG_VBP_NUM_REGIONS                      1
#define CSL_FW_ITIMER4_TIMER_CFG_VBP_NUM_PRIV_IDS_PER_REGION          3
#define CSL_FW_ITIMER4_TIMER_CFG_VBP_NUM_PROTECTED_HW_REGIONS         1
#define CSL_FW_ITIMER4_TIMER_CFG_VBP_CFG_START                        0x44239000
#define CSL_FW_ITIMER4_TIMER_CFG_VBP_CFG_END                          0x442393ff

/* Properties of firewall protecting slave endpoint: IAES.SLV*/
#define CSL_FW_IAES_SLV_ID                                            602
#define CSL_FW_IAES_SLV_TYPE                                          CSL_FW_SECURITY
#define CSL_FW_IAES_SLV_MMR_BASE                                      0x45096800
#define CSL_FW_IAES_SLV_NUM_REGIONS                                   2
#define CSL_FW_IAES_SLV_NUM_PRIV_IDS_PER_REGION                       3
#define CSL_FW_IAES_SLV_NUM_PROTECTED_HW_REGIONS                      1
#define CSL_FW_IAES_SLV_MEM_START                                     0x4423c000
#define CSL_FW_IAES_SLV_MEM_END                                       0x4423dfff

/* Properties of firewall protecting slave endpoint: IDMSC_IA.CFG*/
#define CSL_FW_IDMSC_IA_CFG_ID                                        607
#define CSL_FW_IDMSC_IA_CFG_TYPE                                      CSL_FW_SECURITY
#define CSL_FW_IDMSC_IA_CFG_MMR_BASE                                  0x45097c00
#define CSL_FW_IDMSC_IA_CFG_NUM_REGIONS                               2
#define CSL_FW_IDMSC_IA_CFG_NUM_PRIV_IDS_PER_REGION                   3
#define CSL_FW_IDMSC_IA_CFG_NUM_PROTECTED_HW_REGIONS                  3
#define CSL_FW_IDMSC_IA_CFG_CFG_START                                 0x44410000
#define CSL_FW_IDMSC_IA_CFG_CFG_END                                   0x4441001f
#define CSL_FW_IDMSC_IA_CFG_IMAP_START                                0x44400000
#define CSL_FW_IDMSC_IA_CFG_IMAP_END                                  0x44401fff
#define CSL_FW_IDMSC_IA_CFG_INTR_START                                0x44300000
#define CSL_FW_IDMSC_IA_CFG_INTR_END                                  0x4437ffff

/* Properties of firewall protecting slave endpoint: IFWMGR_INT.SLV*/
#define CSL_FW_IFWMGR_INT_SLV_ID                                      639
#define CSL_FW_IFWMGR_INT_SLV_TYPE                                    CSL_FW_SECURITY
#define CSL_FW_IFWMGR_INT_SLV_MMR_BASE                                0x4509fc00
#define CSL_FW_IFWMGR_INT_SLV_NUM_REGIONS                             8
#define CSL_FW_IFWMGR_INT_SLV_NUM_PRIV_IDS_PER_REGION                 3
#define CSL_FW_IFWMGR_INT_SLV_NUM_PROTECTED_HW_REGIONS                1
#define CSL_FW_IFWMGR_INT_SLV_MEM_START                               0x45000000
#define CSL_FW_IFWMGR_INT_SLV_MEM_END                                 0x45ffffff

/* Properties of firewall protecting slave endpoint: IPULSAR_SL_MCU_0.CPU0_SLV*/
#define CSL_FW_IPULSAR_SL_MCU_0_CPU0_SLV_ID                           1024
#define CSL_FW_IPULSAR_SL_MCU_0_CPU0_SLV_TYPE                         CSL_FW_SECURITY
#define CSL_FW_IPULSAR_SL_MCU_0_CPU0_SLV_MMR_BASE                     0x45100000
#define CSL_FW_IPULSAR_SL_MCU_0_CPU0_SLV_NUM_REGIONS                  4
#define CSL_FW_IPULSAR_SL_MCU_0_CPU0_SLV_NUM_PRIV_IDS_PER_REGION      3
#define CSL_FW_IPULSAR_SL_MCU_0_CPU0_SLV_NUM_PROTECTED_HW_REGIONS     4
#define CSL_FW_IPULSAR_SL_MCU_0_CPU0_SLV_ATCM0_START                  0x41000000
#define CSL_FW_IPULSAR_SL_MCU_0_CPU0_SLV_ATCM0_END                    0x41007fff
#define CSL_FW_IPULSAR_SL_MCU_0_CPU0_SLV_BTCM0_START                  0x41010000
#define CSL_FW_IPULSAR_SL_MCU_0_CPU0_SLV_BTCM0_END                    0x41017fff
#define CSL_FW_IPULSAR_SL_MCU_0_CPU0_SLV_ICACHE0_START                0x5400000000
#define CSL_FW_IPULSAR_SL_MCU_0_CPU0_SLV_ICACHE0_END                  0x54007fffff
#define CSL_FW_IPULSAR_SL_MCU_0_CPU0_SLV_DCACHE0_START                0x5400800000
#define CSL_FW_IPULSAR_SL_MCU_0_CPU0_SLV_DCACHE0_END                  0x5400ffffff

/* Properties of firewall protecting slave endpoint: IPULSAR_SL_MCU_0.CPU0_CFG_SLV*/
#define CSL_FW_IPULSAR_SL_MCU_0_CPU0_CFG_SLV_ID                       1025
#define CSL_FW_IPULSAR_SL_MCU_0_CPU0_CFG_SLV_TYPE                     CSL_FW_SECURITY
#define CSL_FW_IPULSAR_SL_MCU_0_CPU0_CFG_SLV_MMR_BASE                 0x45100400
#define CSL_FW_IPULSAR_SL_MCU_0_CPU0_CFG_SLV_NUM_REGIONS              2
#define CSL_FW_IPULSAR_SL_MCU_0_CPU0_CFG_SLV_NUM_PRIV_IDS_PER_REGION  3
#define CSL_FW_IPULSAR_SL_MCU_0_CPU0_CFG_SLV_NUM_PROTECTED_HW_REGIONS 2
#define CSL_FW_IPULSAR_SL_MCU_0_CPU0_CFG_SLV_CPU0_ECC_AGGR__CFG__REGS_START     0x40080000
#define CSL_FW_IPULSAR_SL_MCU_0_CPU0_CFG_SLV_CPU0_ECC_AGGR__CFG__REGS_END       0x400803ff
#define CSL_FW_IPULSAR_SL_MCU_0_CPU0_CFG_SLV_PULSAR_SL_COMPARE_WRAPPER__CFG__MMRS_START   0x400f0000
#define CSL_FW_IPULSAR_SL_MCU_0_CPU0_CFG_SLV_PULSAR_SL_COMPARE_WRAPPER__CFG__MMRS_END     0x400f00ff

/* Properties of firewall protecting slave endpoint: IRTI_MCU_0.SLV*/
#define CSL_FW_IRTI_MCU_0_SLV_ID                                      1026
#define CSL_FW_IRTI_MCU_0_SLV_TYPE                                    CSL_FW_SECURITY
#define CSL_FW_IRTI_MCU_0_SLV_MMR_BASE                                0x45100800
#define CSL_FW_IRTI_MCU_0_SLV_NUM_REGIONS                             1
#define CSL_FW_IRTI_MCU_0_SLV_NUM_PRIV_IDS_PER_REGION                 3
#define CSL_FW_IRTI_MCU_0_SLV_NUM_PROTECTED_HW_REGIONS                1
#define CSL_FW_IRTI_MCU_0_SLV_CFG_START                               0x40600000
#define CSL_FW_IRTI_MCU_0_SLV_CFG_END                                 0x406000ff

/* Properties of firewall protecting slave endpoint: IPULSAR_SL_MCU_0.CPU1_SLV*/
#define CSL_FW_IPULSAR_SL_MCU_0_CPU1_SLV_ID                           1028
#define CSL_FW_IPULSAR_SL_MCU_0_CPU1_SLV_TYPE                         CSL_FW_SECURITY
#define CSL_FW_IPULSAR_SL_MCU_0_CPU1_SLV_MMR_BASE                     0x45101000
#define CSL_FW_IPULSAR_SL_MCU_0_CPU1_SLV_NUM_REGIONS                  4
#define CSL_FW_IPULSAR_SL_MCU_0_CPU1_SLV_NUM_PRIV_IDS_PER_REGION      3
#define CSL_FW_IPULSAR_SL_MCU_0_CPU1_SLV_NUM_PROTECTED_HW_REGIONS     4
#define CSL_FW_IPULSAR_SL_MCU_0_CPU1_SLV_ATCM1_START                  0x41400000
#define CSL_FW_IPULSAR_SL_MCU_0_CPU1_SLV_ATCM1_END                    0x41407fff
#define CSL_FW_IPULSAR_SL_MCU_0_CPU1_SLV_BTCM1_START                  0x41410000
#define CSL_FW_IPULSAR_SL_MCU_0_CPU1_SLV_BTCM1_END                    0x41417fff
#define CSL_FW_IPULSAR_SL_MCU_0_CPU1_SLV_ICACHE1_START                0x5401000000
#define CSL_FW_IPULSAR_SL_MCU_0_CPU1_SLV_ICACHE1_END                  0x54017fffff
#define CSL_FW_IPULSAR_SL_MCU_0_CPU1_SLV_DCACHE1_START                0x5401800000
#define CSL_FW_IPULSAR_SL_MCU_0_CPU1_SLV_DCACHE1_END                  0x5401ffffff

/* Properties of firewall protecting slave endpoint: IPULSAR_SL_MCU_0.CPU1_CFG_SLV*/
#define CSL_FW_IPULSAR_SL_MCU_0_CPU1_CFG_SLV_ID                       1029
#define CSL_FW_IPULSAR_SL_MCU_0_CPU1_CFG_SLV_TYPE                     CSL_FW_SECURITY
#define CSL_FW_IPULSAR_SL_MCU_0_CPU1_CFG_SLV_MMR_BASE                 0x45101400
#define CSL_FW_IPULSAR_SL_MCU_0_CPU1_CFG_SLV_NUM_REGIONS              1
#define CSL_FW_IPULSAR_SL_MCU_0_CPU1_CFG_SLV_NUM_PRIV_IDS_PER_REGION  3
#define CSL_FW_IPULSAR_SL_MCU_0_CPU1_CFG_SLV_NUM_PROTECTED_HW_REGIONS 1
#define CSL_FW_IPULSAR_SL_MCU_0_CPU1_CFG_SLV_REGS_START               0x400c0000
#define CSL_FW_IPULSAR_SL_MCU_0_CPU1_CFG_SLV_REGS_END                 0x400c03ff

/* Properties of firewall protecting slave endpoint: IRTI_MCU_1.SLV*/
#define CSL_FW_IRTI_MCU_1_SLV_ID                                      1030
#define CSL_FW_IRTI_MCU_1_SLV_TYPE                                    CSL_FW_SECURITY
#define CSL_FW_IRTI_MCU_1_SLV_MMR_BASE                                0x45101800
#define CSL_FW_IRTI_MCU_1_SLV_NUM_REGIONS                             1
#define CSL_FW_IRTI_MCU_1_SLV_NUM_PRIV_IDS_PER_REGION                 3
#define CSL_FW_IRTI_MCU_1_SLV_NUM_PROTECTED_HW_REGIONS                1
#define CSL_FW_IRTI_MCU_1_SLV_CFG_START                               0x40610000
#define CSL_FW_IRTI_MCU_1_SLV_CFG_END                                 0x406100ff

/* Properties of firewall protecting slave endpoint: IFSS_MCU_0.FSS_CFG*/
#define CSL_FW_IFSS_MCU_0_FSS_CFG_ID                                  1032
#define CSL_FW_IFSS_MCU_0_FSS_CFG_TYPE                                CSL_FW_SECURITY
#define CSL_FW_IFSS_MCU_0_FSS_CFG_MMR_BASE                            0x45102000
#define CSL_FW_IFSS_MCU_0_FSS_CFG_NUM_REGIONS                         12
#define CSL_FW_IFSS_MCU_0_FSS_CFG_NUM_PRIV_IDS_PER_REGION             3
#define CSL_FW_IFSS_MCU_0_FSS_CFG_NUM_PROTECTED_HW_REGIONS            12
#define CSL_FW_IFSS_MCU_0_FSS_CFG_FSAS__FSAS_MMR_CFG__FSAS_GENREGS_START        0x47010000
#define CSL_FW_IFSS_MCU_0_FSS_CFG_FSAS__FSAS_MMR_CFG__FSAS_GENREGS_END          0x470100ff
#define CSL_FW_IFSS_MCU_0_FSS_CFG_FSAS__FSAS_OTFA_CFG__FSAS_OTFA_REGS_START     0x47020000
#define CSL_FW_IFSS_MCU_0_FSS_CFG_FSAS__FSAS_OTFA_CFG__FSAS_OTFA_REGS_END       0x47020fff
#define CSL_FW_IFSS_MCU_0_FSS_CFG_FSS_MMR__FSS_MMR_CFG__FSS_GENREGS_START       0x47000000
#define CSL_FW_IFSS_MCU_0_FSS_CFG_FSS_MMR__FSS_MMR_CFG__FSS_GENREGS_END         0x470000ff
#define CSL_FW_IFSS_MCU_0_FSS_CFG_HB__HPB_CFG__SYS__SS_CFG__SS_CFG_REG_START    0x47030000
#define CSL_FW_IFSS_MCU_0_FSS_CFG_HB__HPB_CFG__SYS__SS_CFG__SS_CFG_REG_END      0x470300ff
#define CSL_FW_IFSS_MCU_0_FSS_CFG_HB__HPB_CFG__WRAP__CORE_CFG__CORE_CFG_REG_START         0x47034000
#define CSL_FW_IFSS_MCU_0_FSS_CFG_HB__HPB_CFG__WRAP__CORE_CFG__CORE_CFG_REG_END 0x470340ff
#define CSL_FW_IFSS_MCU_0_FSS_CFG_HB__HPB_CFG__WRAP__ECC_AGGR_CFG__REGS_START   0x47060000
#define CSL_FW_IFSS_MCU_0_FSS_CFG_HB__HPB_CFG__WRAP__ECC_AGGR_CFG__REGS_END     0x470603ff
#define CSL_FW_IFSS_MCU_0_FSS_CFG_OSPI0__OSPI_CFG_VBUSP__MMR__MMRVBP__REGS_START          0x47044000
#define CSL_FW_IFSS_MCU_0_FSS_CFG_OSPI0__OSPI_CFG_VBUSP__MMR__MMRVBP__REGS_END  0x470441ff
#define CSL_FW_IFSS_MCU_0_FSS_CFG_OSPI0__OSPI_CFG_VBUSP__OSPI_WRAP__ECC_AGGR_VBP__REGS_START        0x47068000
#define CSL_FW_IFSS_MCU_0_FSS_CFG_OSPI0__OSPI_CFG_VBUSP__OSPI_WRAP__ECC_AGGR_VBP__REGS_END          0x470683ff
#define CSL_FW_IFSS_MCU_0_FSS_CFG_OSPI0__OSPI_CFG_VBUSP__VBP2APB_WRAP__OSPI_CFG_VBP__OSPI_FLASH_APB_REGS_START          0x47040000
#define CSL_FW_IFSS_MCU_0_FSS_CFG_OSPI0__OSPI_CFG_VBUSP__VBP2APB_WRAP__OSPI_CFG_VBP__OSPI_FLASH_APB_REGS_END  0x470400ff
#define CSL_FW_IFSS_MCU_0_FSS_CFG_OSPI1__OSPI_CFG_VBUSP__MMR__MMRVBP__REGS_START          0x47054000
#define CSL_FW_IFSS_MCU_0_FSS_CFG_OSPI1__OSPI_CFG_VBUSP__MMR__MMRVBP__REGS_END  0x470541ff
#define CSL_FW_IFSS_MCU_0_FSS_CFG_OSPI1__OSPI_CFG_VBUSP__OSPI_WRAP__ECC_AGGR_VBP__REGS_START        0x47064000
#define CSL_FW_IFSS_MCU_0_FSS_CFG_OSPI1__OSPI_CFG_VBUSP__OSPI_WRAP__ECC_AGGR_VBP__REGS_END          0x470643ff
#define CSL_FW_IFSS_MCU_0_FSS_CFG_OSPI1__OSPI_CFG_VBUSP__VBP2APB_WRAP__OSPI_CFG_VBP__OSPI_FLASH_APB_REGS_START          0x47050000
#define CSL_FW_IFSS_MCU_0_FSS_CFG_OSPI1__OSPI_CFG_VBUSP__VBP2APB_WRAP__OSPI_CFG_VBP__OSPI_FLASH_APB_REGS_END  0x470500ff

/* Properties of firewall protecting slave endpoint: IFSS_MCU_0.FSS_S1*/
#define CSL_FW_IFSS_MCU_0_FSS_S1_ID                                   1033
#define CSL_FW_IFSS_MCU_0_FSS_S1_TYPE                                 CSL_FW_SECURITY
#define CSL_FW_IFSS_MCU_0_FSS_S1_MMR_BASE                             0x45102400
#define CSL_FW_IFSS_MCU_0_FSS_S1_NUM_REGIONS                          8
#define CSL_FW_IFSS_MCU_0_FSS_S1_NUM_PRIV_IDS_PER_REGION              3
#define CSL_FW_IFSS_MCU_0_FSS_S1_NUM_PROTECTED_HW_REGIONS             3
#define CSL_FW_IFSS_MCU_0_FSS_S1_OSPI1_OSPI_DATA_VBP_R0_MAP_START     0x600000000
#define CSL_FW_IFSS_MCU_0_FSS_S1_OSPI1_OSPI_DATA_VBP_R0_MAP_END       0x6ffffffff
#define CSL_FW_IFSS_MCU_0_FSS_S1_OSPI1_OSPI_DATA_VBP_R1_MAP_START     0x58000000
#define CSL_FW_IFSS_MCU_0_FSS_S1_OSPI1_OSPI_DATA_VBP_R1_MAP_END       0x5fffffff
#define CSL_FW_IFSS_MCU_0_FSS_S1_OSPI1_OSPI_DATA_VBP_R3_MAP_START     0x700000000
#define CSL_FW_IFSS_MCU_0_FSS_S1_OSPI1_OSPI_DATA_VBP_R3_MAP_END       0x7ffffffff

/* Properties of firewall protecting slave endpoint: IFSS_MCU_0.FSS_S0*/
#define CSL_FW_IFSS_MCU_0_FSS_S0_ID                                   1036
#define CSL_FW_IFSS_MCU_0_FSS_S0_TYPE                                 CSL_FW_SECURITY
#define CSL_FW_IFSS_MCU_0_FSS_S0_MMR_BASE                             0x45103000
#define CSL_FW_IFSS_MCU_0_FSS_S0_NUM_REGIONS                          8
#define CSL_FW_IFSS_MCU_0_FSS_S0_NUM_PRIV_IDS_PER_REGION              3
#define CSL_FW_IFSS_MCU_0_FSS_S0_NUM_PROTECTED_HW_REGIONS             3
#define CSL_FW_IFSS_MCU_0_FSS_S0_DAT_REG0_START                       0x400000000
#define CSL_FW_IFSS_MCU_0_FSS_S0_DAT_REG0_END                         0x4ffffffff
#define CSL_FW_IFSS_MCU_0_FSS_S0_DAT_REG1_START                       0x50000000
#define CSL_FW_IFSS_MCU_0_FSS_S0_DAT_REG1_END                         0x57ffffff
#define CSL_FW_IFSS_MCU_0_FSS_S0_DAT_REG3_START                       0x500000000
#define CSL_FW_IFSS_MCU_0_FSS_S0_DAT_REG3_END                         0x5ffffffff

/* Properties of firewall protecting slave endpoint: IK3_SOC_49152X32_PROM_MCU_0.SLV*/
#define CSL_FW_IK3_SOC_49152X32_PROM_MCU_0_SLV_ID                     1048
#define CSL_FW_IK3_SOC_49152X32_PROM_MCU_0_SLV_TYPE                   CSL_FW_SECURITY
#define CSL_FW_IK3_SOC_49152X32_PROM_MCU_0_SLV_MMR_BASE               0x45106000
#define CSL_FW_IK3_SOC_49152X32_PROM_MCU_0_SLV_NUM_REGIONS            1
#define CSL_FW_IK3_SOC_49152X32_PROM_MCU_0_SLV_NUM_PRIV_IDS_PER_REGION          3
#define CSL_FW_IK3_SOC_49152X32_PROM_MCU_0_SLV_NUM_PROTECTED_HW_REGIONS         1
#define CSL_FW_IK3_SOC_49152X32_PROM_MCU_0_SLV_ROM_START              0x41100000
#define CSL_FW_IK3_SOC_49152X32_PROM_MCU_0_SLV_ROM_END                0x4113ffff

/* Properties of firewall protecting slave endpoint: IMSRAM64KX64E_MCU_0.SLV*/
#define CSL_FW_IMSRAM64KX64E_MCU_0_SLV_ID                             1050
#define CSL_FW_IMSRAM64KX64E_MCU_0_SLV_TYPE                           CSL_FW_SECURITY
#define CSL_FW_IMSRAM64KX64E_MCU_0_SLV_MMR_BASE                       0x45106800
#define CSL_FW_IMSRAM64KX64E_MCU_0_SLV_NUM_REGIONS                    8
#define CSL_FW_IMSRAM64KX64E_MCU_0_SLV_NUM_PRIV_IDS_PER_REGION        3
#define CSL_FW_IMSRAM64KX64E_MCU_0_SLV_NUM_PROTECTED_HW_REGIONS       1
#define CSL_FW_IMSRAM64KX64E_MCU_0_SLV_RAM_START                      0x41c00000
#define CSL_FW_IMSRAM64KX64E_MCU_0_SLV_RAM_END                        0x41c7ffff

/* Properties of firewall protecting slave endpoint: IMSRAM64KX64E_MCU_0.CFG*/
#define CSL_FW_IMSRAM64KX64E_MCU_0_CFG_ID                             1051
#define CSL_FW_IMSRAM64KX64E_MCU_0_CFG_TYPE                           CSL_FW_SECURITY
#define CSL_FW_IMSRAM64KX64E_MCU_0_CFG_MMR_BASE                       0x45106c00
#define CSL_FW_IMSRAM64KX64E_MCU_0_CFG_NUM_REGIONS                    1
#define CSL_FW_IMSRAM64KX64E_MCU_0_CFG_NUM_PRIV_IDS_PER_REGION        3
#define CSL_FW_IMSRAM64KX64E_MCU_0_CFG_NUM_PROTECTED_HW_REGIONS       1
#define CSL_FW_IMSRAM64KX64E_MCU_0_CFG_ECC_AGGR_REGSREGS_START        0x4070b000
#define CSL_FW_IMSRAM64KX64E_MCU_0_CFG_ECC_AGGR_REGSREGS_END          0x4070b3ff

/* Properties of firewall protecting slave endpoint: IPSRAM128X32_MCU_0.RAM_VB*/
#define CSL_FW_IPSRAM128X32_MCU_0_RAM_VB_ID                           1052
#define CSL_FW_IPSRAM128X32_MCU_0_RAM_VB_TYPE                         CSL_FW_SECURITY
#define CSL_FW_IPSRAM128X32_MCU_0_RAM_VB_MMR_BASE                     0x45107000
#define CSL_FW_IPSRAM128X32_MCU_0_RAM_VB_NUM_REGIONS                  1
#define CSL_FW_IPSRAM128X32_MCU_0_RAM_VB_NUM_PRIV_IDS_PER_REGION      3
#define CSL_FW_IPSRAM128X32_MCU_0_RAM_VB_NUM_PROTECTED_HW_REGIONS     1
#define CSL_FW_IPSRAM128X32_MCU_0_RAM_VB_RAM_START                    0x40280000
#define CSL_FW_IPSRAM128X32_MCU_0_RAM_VB_RAM_END                      0x402801ff

/* Properties of firewall protecting slave endpoint: IDMTIMER_DMC1MS_MCU_0.TIMER_CFG_VBP*/
#define CSL_FW_IDMTIMER_DMC1MS_MCU_0_TIMER_CFG_VBP_ID                 1056
#define CSL_FW_IDMTIMER_DMC1MS_MCU_0_TIMER_CFG_VBP_TYPE               CSL_FW_SECURITY
#define CSL_FW_IDMTIMER_DMC1MS_MCU_0_TIMER_CFG_VBP_MMR_BASE           0x45108000
#define CSL_FW_IDMTIMER_DMC1MS_MCU_0_TIMER_CFG_VBP_NUM_REGIONS        1
#define CSL_FW_IDMTIMER_DMC1MS_MCU_0_TIMER_CFG_VBP_NUM_PRIV_IDS_PER_REGION      3
#define CSL_FW_IDMTIMER_DMC1MS_MCU_0_TIMER_CFG_VBP_NUM_PROTECTED_HW_REGIONS     1
#define CSL_FW_IDMTIMER_DMC1MS_MCU_0_TIMER_CFG_VBP_CFG_START          0x40400000
#define CSL_FW_IDMTIMER_DMC1MS_MCU_0_TIMER_CFG_VBP_CFG_END            0x404003ff

/* Properties of firewall protecting slave endpoint: IDMTIMER_DMC1MS_MCU_1.TIMER_CFG_VBP*/
#define CSL_FW_IDMTIMER_DMC1MS_MCU_1_TIMER_CFG_VBP_ID                 1057
#define CSL_FW_IDMTIMER_DMC1MS_MCU_1_TIMER_CFG_VBP_TYPE               CSL_FW_SECURITY
#define CSL_FW_IDMTIMER_DMC1MS_MCU_1_TIMER_CFG_VBP_MMR_BASE           0x45108400
#define CSL_FW_IDMTIMER_DMC1MS_MCU_1_TIMER_CFG_VBP_NUM_REGIONS        1
#define CSL_FW_IDMTIMER_DMC1MS_MCU_1_TIMER_CFG_VBP_NUM_PRIV_IDS_PER_REGION      3
#define CSL_FW_IDMTIMER_DMC1MS_MCU_1_TIMER_CFG_VBP_NUM_PROTECTED_HW_REGIONS     1
#define CSL_FW_IDMTIMER_DMC1MS_MCU_1_TIMER_CFG_VBP_CFG_START          0x40410000
#define CSL_FW_IDMTIMER_DMC1MS_MCU_1_TIMER_CFG_VBP_CFG_END            0x404103ff

/* Properties of firewall protecting slave endpoint: IDMTIMER_DMC1MS_MCU_2.TIMER_CFG_VBP*/
#define CSL_FW_IDMTIMER_DMC1MS_MCU_2_TIMER_CFG_VBP_ID                 1058
#define CSL_FW_IDMTIMER_DMC1MS_MCU_2_TIMER_CFG_VBP_TYPE               CSL_FW_SECURITY
#define CSL_FW_IDMTIMER_DMC1MS_MCU_2_TIMER_CFG_VBP_MMR_BASE           0x45108800
#define CSL_FW_IDMTIMER_DMC1MS_MCU_2_TIMER_CFG_VBP_NUM_REGIONS        1
#define CSL_FW_IDMTIMER_DMC1MS_MCU_2_TIMER_CFG_VBP_NUM_PRIV_IDS_PER_REGION      3
#define CSL_FW_IDMTIMER_DMC1MS_MCU_2_TIMER_CFG_VBP_NUM_PROTECTED_HW_REGIONS     1
#define CSL_FW_IDMTIMER_DMC1MS_MCU_2_TIMER_CFG_VBP_CFG_START          0x40420000
#define CSL_FW_IDMTIMER_DMC1MS_MCU_2_TIMER_CFG_VBP_CFG_END            0x404203ff

/* Properties of firewall protecting slave endpoint: IDMTIMER_DMC1MS_MCU_3.TIMER_CFG_VBP*/
#define CSL_FW_IDMTIMER_DMC1MS_MCU_3_TIMER_CFG_VBP_ID                 1059
#define CSL_FW_IDMTIMER_DMC1MS_MCU_3_TIMER_CFG_VBP_TYPE               CSL_FW_SECURITY
#define CSL_FW_IDMTIMER_DMC1MS_MCU_3_TIMER_CFG_VBP_MMR_BASE           0x45108c00
#define CSL_FW_IDMTIMER_DMC1MS_MCU_3_TIMER_CFG_VBP_NUM_REGIONS        1
#define CSL_FW_IDMTIMER_DMC1MS_MCU_3_TIMER_CFG_VBP_NUM_PRIV_IDS_PER_REGION      3
#define CSL_FW_IDMTIMER_DMC1MS_MCU_3_TIMER_CFG_VBP_NUM_PROTECTED_HW_REGIONS     1
#define CSL_FW_IDMTIMER_DMC1MS_MCU_3_TIMER_CFG_VBP_CFG_START          0x40430000
#define CSL_FW_IDMTIMER_DMC1MS_MCU_3_TIMER_CFG_VBP_CFG_END            0x404303ff

/* Properties of firewall protecting slave endpoint: ISPI_MCU_0.SLV*/
#define CSL_FW_ISPI_MCU_0_SLV_ID                                      1072
#define CSL_FW_ISPI_MCU_0_SLV_TYPE                                    CSL_FW_SECURITY
#define CSL_FW_ISPI_MCU_0_SLV_MMR_BASE                                0x4510c000
#define CSL_FW_ISPI_MCU_0_SLV_NUM_REGIONS                             1
#define CSL_FW_ISPI_MCU_0_SLV_NUM_PRIV_IDS_PER_REGION                 3
#define CSL_FW_ISPI_MCU_0_SLV_NUM_PROTECTED_HW_REGIONS                1
#define CSL_FW_ISPI_MCU_0_SLV_CFG_START                               0x40300000
#define CSL_FW_ISPI_MCU_0_SLV_CFG_END                                 0x403003ff

/* Properties of firewall protecting slave endpoint: ISPI_MCU_1.SLV*/
#define CSL_FW_ISPI_MCU_1_SLV_ID                                      1073
#define CSL_FW_ISPI_MCU_1_SLV_TYPE                                    CSL_FW_SECURITY
#define CSL_FW_ISPI_MCU_1_SLV_MMR_BASE                                0x4510c400
#define CSL_FW_ISPI_MCU_1_SLV_NUM_REGIONS                             1
#define CSL_FW_ISPI_MCU_1_SLV_NUM_PRIV_IDS_PER_REGION                 3
#define CSL_FW_ISPI_MCU_1_SLV_NUM_PROTECTED_HW_REGIONS                1
#define CSL_FW_ISPI_MCU_1_SLV_CFG_START                               0x40310000
#define CSL_FW_ISPI_MCU_1_SLV_CFG_END                                 0x403103ff

/* Properties of firewall protecting slave endpoint: ISPI_MCU_2.SLV*/
#define CSL_FW_ISPI_MCU_2_SLV_ID                                      1074
#define CSL_FW_ISPI_MCU_2_SLV_TYPE                                    CSL_FW_SECURITY
#define CSL_FW_ISPI_MCU_2_SLV_MMR_BASE                                0x4510c800
#define CSL_FW_ISPI_MCU_2_SLV_NUM_REGIONS                             1
#define CSL_FW_ISPI_MCU_2_SLV_NUM_PRIV_IDS_PER_REGION                 3
#define CSL_FW_ISPI_MCU_2_SLV_NUM_PROTECTED_HW_REGIONS                1
#define CSL_FW_ISPI_MCU_2_SLV_CFG_START                               0x40320000
#define CSL_FW_ISPI_MCU_2_SLV_CFG_END                                 0x403203ff

/* Properties of firewall protecting slave endpoint: IDCC_MCU_0.DCC_CBA*/
#define CSL_FW_IDCC_MCU_0_DCC_CBA_ID                                  1088
#define CSL_FW_IDCC_MCU_0_DCC_CBA_TYPE                                CSL_FW_SECURITY
#define CSL_FW_IDCC_MCU_0_DCC_CBA_MMR_BASE                            0x45110000
#define CSL_FW_IDCC_MCU_0_DCC_CBA_NUM_REGIONS                         1
#define CSL_FW_IDCC_MCU_0_DCC_CBA_NUM_PRIV_IDS_PER_REGION             3
#define CSL_FW_IDCC_MCU_0_DCC_CBA_NUM_PROTECTED_HW_REGIONS            1
#define CSL_FW_IDCC_MCU_0_DCC_CBA_MEM_START                           0x40100000
#define CSL_FW_IDCC_MCU_0_DCC_CBA_MEM_END                             0x4010003f

/* Properties of firewall protecting slave endpoint: IDCC_MCU_1.DCC_CBA*/
#define CSL_FW_IDCC_MCU_1_DCC_CBA_ID                                  1089
#define CSL_FW_IDCC_MCU_1_DCC_CBA_TYPE                                CSL_FW_SECURITY
#define CSL_FW_IDCC_MCU_1_DCC_CBA_MMR_BASE                            0x45110400
#define CSL_FW_IDCC_MCU_1_DCC_CBA_NUM_REGIONS                         1
#define CSL_FW_IDCC_MCU_1_DCC_CBA_NUM_PRIV_IDS_PER_REGION             3
#define CSL_FW_IDCC_MCU_1_DCC_CBA_NUM_PROTECTED_HW_REGIONS            1
#define CSL_FW_IDCC_MCU_1_DCC_CBA_MEM_START                           0x40110000
#define CSL_FW_IDCC_MCU_1_DCC_CBA_MEM_END                             0x4011003f

/* Properties of firewall protecting slave endpoint: IDCC_MCU_2.DCC_CBA*/
#define CSL_FW_IDCC_MCU_2_DCC_CBA_ID                                  1090
#define CSL_FW_IDCC_MCU_2_DCC_CBA_TYPE                                CSL_FW_SECURITY
#define CSL_FW_IDCC_MCU_2_DCC_CBA_MMR_BASE                            0x45110800
#define CSL_FW_IDCC_MCU_2_DCC_CBA_NUM_REGIONS                         1
#define CSL_FW_IDCC_MCU_2_DCC_CBA_NUM_PRIV_IDS_PER_REGION             3
#define CSL_FW_IDCC_MCU_2_DCC_CBA_NUM_PROTECTED_HW_REGIONS            1
#define CSL_FW_IDCC_MCU_2_DCC_CBA_MEM_START                           0x40120000
#define CSL_FW_IDCC_MCU_2_DCC_CBA_MEM_END                             0x4012003f

/* Properties of firewall protecting slave endpoint: IADC12_GS80_MCU_0.CBA1_DMA*/
#define CSL_FW_IADC12_GS80_MCU_0_CBA1_DMA_ID                          1104
#define CSL_FW_IADC12_GS80_MCU_0_CBA1_DMA_TYPE                        CSL_FW_SECURITY
#define CSL_FW_IADC12_GS80_MCU_0_CBA1_DMA_MMR_BASE                    0x45114000
#define CSL_FW_IADC12_GS80_MCU_0_CBA1_DMA_NUM_REGIONS                 1
#define CSL_FW_IADC12_GS80_MCU_0_CBA1_DMA_NUM_PRIV_IDS_PER_REGION     3
#define CSL_FW_IADC12_GS80_MCU_0_CBA1_DMA_NUM_PROTECTED_HW_REGIONS    1
#define CSL_FW_IADC12_GS80_MCU_0_CBA1_DMA_ADC12_FIFO_DMA_START        0x40208000
#define CSL_FW_IADC12_GS80_MCU_0_CBA1_DMA_ADC12_FIFO_DMA_END          0x402083ff

/* Properties of firewall protecting slave endpoint: IADC12_GS80_MCU_0.CBA0_CFG*/
#define CSL_FW_IADC12_GS80_MCU_0_CBA0_CFG_ID                          1105
#define CSL_FW_IADC12_GS80_MCU_0_CBA0_CFG_TYPE                        CSL_FW_SECURITY
#define CSL_FW_IADC12_GS80_MCU_0_CBA0_CFG_MMR_BASE                    0x45114400
#define CSL_FW_IADC12_GS80_MCU_0_CBA0_CFG_NUM_REGIONS                 2
#define CSL_FW_IADC12_GS80_MCU_0_CBA0_CFG_NUM_PRIV_IDS_PER_REGION     3
#define CSL_FW_IADC12_GS80_MCU_0_CBA0_CFG_NUM_PROTECTED_HW_REGIONS    2
#define CSL_FW_IADC12_GS80_MCU_0_CBA0_CFG_ADCREGS_START               0x40200000
#define CSL_FW_IADC12_GS80_MCU_0_CBA0_CFG_ADCREGS_END                 0x402003ff
#define CSL_FW_IADC12_GS80_MCU_0_CBA0_CFG_ECCREGS_START               0x40707000
#define CSL_FW_IADC12_GS80_MCU_0_CBA0_CFG_ECCREGS_END                 0x407073ff

/* Properties of firewall protecting slave endpoint: IADC12_GS80_MCU_1.CBA1_DMA*/
#define CSL_FW_IADC12_GS80_MCU_1_CBA1_DMA_ID                          1106
#define CSL_FW_IADC12_GS80_MCU_1_CBA1_DMA_TYPE                        CSL_FW_SECURITY
#define CSL_FW_IADC12_GS80_MCU_1_CBA1_DMA_MMR_BASE                    0x45114800
#define CSL_FW_IADC12_GS80_MCU_1_CBA1_DMA_NUM_REGIONS                 1
#define CSL_FW_IADC12_GS80_MCU_1_CBA1_DMA_NUM_PRIV_IDS_PER_REGION     3
#define CSL_FW_IADC12_GS80_MCU_1_CBA1_DMA_NUM_PROTECTED_HW_REGIONS    1
#define CSL_FW_IADC12_GS80_MCU_1_CBA1_DMA_ADC12_FIFO_DMA_START        0x40218000
#define CSL_FW_IADC12_GS80_MCU_1_CBA1_DMA_ADC12_FIFO_DMA_END          0x402183ff

/* Properties of firewall protecting slave endpoint: IADC12_GS80_MCU_1.CBA0_CFG*/
#define CSL_FW_IADC12_GS80_MCU_1_CBA0_CFG_ID                          1107
#define CSL_FW_IADC12_GS80_MCU_1_CBA0_CFG_TYPE                        CSL_FW_SECURITY
#define CSL_FW_IADC12_GS80_MCU_1_CBA0_CFG_MMR_BASE                    0x45114c00
#define CSL_FW_IADC12_GS80_MCU_1_CBA0_CFG_NUM_REGIONS                 2
#define CSL_FW_IADC12_GS80_MCU_1_CBA0_CFG_NUM_PRIV_IDS_PER_REGION     3
#define CSL_FW_IADC12_GS80_MCU_1_CBA0_CFG_NUM_PROTECTED_HW_REGIONS    2
#define CSL_FW_IADC12_GS80_MCU_1_CBA0_CFG_ADCREGS_START               0x40210000
#define CSL_FW_IADC12_GS80_MCU_1_CBA0_CFG_ADCREGS_END                 0x402103ff
#define CSL_FW_IADC12_GS80_MCU_1_CBA0_CFG_ECCREGS_START               0x40708000
#define CSL_FW_IADC12_GS80_MCU_1_CBA0_CFG_ECCREGS_END                 0x407083ff

/* Properties of firewall protecting slave endpoint: IUSART_MCU_0.VBUSP_CBA*/
#define CSL_FW_IUSART_MCU_0_VBUSP_CBA_ID                              1120
#define CSL_FW_IUSART_MCU_0_VBUSP_CBA_TYPE                            CSL_FW_SECURITY
#define CSL_FW_IUSART_MCU_0_VBUSP_CBA_MMR_BASE                        0x45118000
#define CSL_FW_IUSART_MCU_0_VBUSP_CBA_NUM_REGIONS                     1
#define CSL_FW_IUSART_MCU_0_VBUSP_CBA_NUM_PRIV_IDS_PER_REGION         3
#define CSL_FW_IUSART_MCU_0_VBUSP_CBA_NUM_PROTECTED_HW_REGIONS        1
#define CSL_FW_IUSART_MCU_0_VBUSP_CBA_MEM_START                       0x40a00000
#define CSL_FW_IUSART_MCU_0_VBUSP_CBA_MEM_END                         0x40a001ff

/* Properties of firewall protecting slave endpoint: IMSHSI2C_MCU_0.VBUSP*/
#define CSL_FW_IMSHSI2C_MCU_0_VBUSP_ID                                1152
#define CSL_FW_IMSHSI2C_MCU_0_VBUSP_TYPE                              CSL_FW_SECURITY
#define CSL_FW_IMSHSI2C_MCU_0_VBUSP_MMR_BASE                          0x45120000
#define CSL_FW_IMSHSI2C_MCU_0_VBUSP_NUM_REGIONS                       1
#define CSL_FW_IMSHSI2C_MCU_0_VBUSP_NUM_PRIV_IDS_PER_REGION           3
#define CSL_FW_IMSHSI2C_MCU_0_VBUSP_NUM_PROTECTED_HW_REGIONS          1
#define CSL_FW_IMSHSI2C_MCU_0_VBUSP_CFG_START                         0x40b00000
#define CSL_FW_IMSHSI2C_MCU_0_VBUSP_CFG_END                           0x40b000ff

/* Properties of firewall protecting slave endpoint: IESM_MCU_MCU_0.ESM_CFG*/
#define CSL_FW_IESM_MCU_MCU_0_ESM_CFG_ID                              1168
#define CSL_FW_IESM_MCU_MCU_0_ESM_CFG_TYPE                            CSL_FW_SECURITY
#define CSL_FW_IESM_MCU_MCU_0_ESM_CFG_MMR_BASE                        0x45124000
#define CSL_FW_IESM_MCU_MCU_0_ESM_CFG_NUM_REGIONS                     1
#define CSL_FW_IESM_MCU_MCU_0_ESM_CFG_NUM_PRIV_IDS_PER_REGION         3
#define CSL_FW_IESM_MCU_MCU_0_ESM_CFG_NUM_PROTECTED_HW_REGIONS        1
#define CSL_FW_IESM_MCU_MCU_0_ESM_CFG_CFG_START                       0x40800000
#define CSL_FW_IESM_MCU_MCU_0_ESM_CFG_CFG_END                         0x40800fff

/* Properties of firewall protecting slave endpoint: IMCANSS_MCU_0.MCANSS_VBUSP*/
#define CSL_FW_IMCANSS_MCU_0_MCANSS_VBUSP_ID                          1184
#define CSL_FW_IMCANSS_MCU_0_MCANSS_VBUSP_TYPE                        CSL_FW_SECURITY
#define CSL_FW_IMCANSS_MCU_0_MCANSS_VBUSP_MMR_BASE                    0x45128000
#define CSL_FW_IMCANSS_MCU_0_MCANSS_VBUSP_NUM_REGIONS                 4
#define CSL_FW_IMCANSS_MCU_0_MCANSS_VBUSP_NUM_PRIV_IDS_PER_REGION     3
#define CSL_FW_IMCANSS_MCU_0_MCANSS_VBUSP_NUM_PROTECTED_HW_REGIONS    4
#define CSL_FW_IMCANSS_MCU_0_MCANSS_VBUSP_MMR__MMRVBP__MCANSS_REGS_START        0x40520000
#define CSL_FW_IMCANSS_MCU_0_MCANSS_VBUSP_MMR__MMRVBP__MCANSS_REGS_END          0x405200ff
#define CSL_FW_IMCANSS_MCU_0_MCANSS_VBUSP_MCAN_WRAP__MCAN_CFG_VBP__MCAN_REGS_START        0x40528000
#define CSL_FW_IMCANSS_MCU_0_MCANSS_VBUSP_MCAN_WRAP__MCAN_CFG_VBP__MCAN_REGS_END          0x405281ff
#define CSL_FW_IMCANSS_MCU_0_MCANSS_VBUSP_MSGMEM_WRAP__ECC_AGGR_VBP__REGS_START 0x40700000
#define CSL_FW_IMCANSS_MCU_0_MCANSS_VBUSP_MSGMEM_WRAP__ECC_AGGR_VBP__REGS_END   0x407003ff
#define CSL_FW_IMCANSS_MCU_0_MCANSS_VBUSP_MSGMEM_WRAP__MSGMEM_VBP__RAM_START    0x40500000
#define CSL_FW_IMCANSS_MCU_0_MCANSS_VBUSP_MSGMEM_WRAP__MSGMEM_VBP__RAM_END      0x40507fff

/* Properties of firewall protecting slave endpoint: IMCANSS_MCU_1.MCANSS_VBUSP*/
#define CSL_FW_IMCANSS_MCU_1_MCANSS_VBUSP_ID                          1185
#define CSL_FW_IMCANSS_MCU_1_MCANSS_VBUSP_TYPE                        CSL_FW_SECURITY
#define CSL_FW_IMCANSS_MCU_1_MCANSS_VBUSP_MMR_BASE                    0x45128400
#define CSL_FW_IMCANSS_MCU_1_MCANSS_VBUSP_NUM_REGIONS                 4
#define CSL_FW_IMCANSS_MCU_1_MCANSS_VBUSP_NUM_PRIV_IDS_PER_REGION     3
#define CSL_FW_IMCANSS_MCU_1_MCANSS_VBUSP_NUM_PROTECTED_HW_REGIONS    4
#define CSL_FW_IMCANSS_MCU_1_MCANSS_VBUSP_MMR__MMRVBP__MCANSS_REGS_START        0x40560000
#define CSL_FW_IMCANSS_MCU_1_MCANSS_VBUSP_MMR__MMRVBP__MCANSS_REGS_END          0x405600ff
#define CSL_FW_IMCANSS_MCU_1_MCANSS_VBUSP_MCAN_WRAP__MCAN_CFG_VBP__MCAN_REGS_START        0x40568000
#define CSL_FW_IMCANSS_MCU_1_MCANSS_VBUSP_MCAN_WRAP__MCAN_CFG_VBP__MCAN_REGS_END          0x405681ff
#define CSL_FW_IMCANSS_MCU_1_MCANSS_VBUSP_MSGMEM_WRAP__ECC_AGGR_VBP__REGS_START 0x40701000
#define CSL_FW_IMCANSS_MCU_1_MCANSS_VBUSP_MSGMEM_WRAP__ECC_AGGR_VBP__REGS_END   0x407013ff
#define CSL_FW_IMCANSS_MCU_1_MCANSS_VBUSP_MSGMEM_WRAP__MSGMEM_VBP__RAM_START    0x40540000
#define CSL_FW_IMCANSS_MCU_1_MCANSS_VBUSP_MSGMEM_WRAP__MSGMEM_VBP__RAM_END      0x40547fff

/* Properties of firewall protecting slave endpoint: IMCU_CTRL_MMR_MCU_0.VBUSP*/
#define CSL_FW_IMCU_CTRL_MMR_MCU_0_VBUSP_ID                           1200
#define CSL_FW_IMCU_CTRL_MMR_MCU_0_VBUSP_TYPE                         CSL_FW_SECURITY
#define CSL_FW_IMCU_CTRL_MMR_MCU_0_VBUSP_MMR_BASE                     0x4512c000
#define CSL_FW_IMCU_CTRL_MMR_MCU_0_VBUSP_NUM_REGIONS                  16
#define CSL_FW_IMCU_CTRL_MMR_MCU_0_VBUSP_NUM_PRIV_IDS_PER_REGION      3
#define CSL_FW_IMCU_CTRL_MMR_MCU_0_VBUSP_NUM_PROTECTED_HW_REGIONS     1
#define CSL_FW_IMCU_CTRL_MMR_MCU_0_VBUSP_CFG0_START                   0x40f00000
#define CSL_FW_IMCU_CTRL_MMR_MCU_0_VBUSP_CFG0_END                     0x40f1ffff

/* Properties of firewall protecting slave endpoint: IMCU_PLL_MMR_MCU_0.VBUSP*/
#define CSL_FW_IMCU_PLL_MMR_MCU_0_VBUSP_ID                            1201
#define CSL_FW_IMCU_PLL_MMR_MCU_0_VBUSP_TYPE                          CSL_FW_SECURITY
#define CSL_FW_IMCU_PLL_MMR_MCU_0_VBUSP_MMR_BASE                      0x4512c400
#define CSL_FW_IMCU_PLL_MMR_MCU_0_VBUSP_NUM_REGIONS                   2
#define CSL_FW_IMCU_PLL_MMR_MCU_0_VBUSP_NUM_PRIV_IDS_PER_REGION       3
#define CSL_FW_IMCU_PLL_MMR_MCU_0_VBUSP_NUM_PROTECTED_HW_REGIONS      1
#define CSL_FW_IMCU_PLL_MMR_MCU_0_VBUSP_CFG_START                     0x40d00000
#define CSL_FW_IMCU_PLL_MMR_MCU_0_VBUSP_CFG_END                       0x40d01fff

/* Properties of firewall protecting slave endpoint: IK3_MCU_EFUSE_MCU_0.SLV*/
#define CSL_FW_IK3_MCU_EFUSE_MCU_0_SLV_ID                             1208
#define CSL_FW_IK3_MCU_EFUSE_MCU_0_SLV_TYPE                           CSL_FW_SECURITY
#define CSL_FW_IK3_MCU_EFUSE_MCU_0_SLV_MMR_BASE                       0x4512e000
#define CSL_FW_IK3_MCU_EFUSE_MCU_0_SLV_NUM_REGIONS                    1
#define CSL_FW_IK3_MCU_EFUSE_MCU_0_SLV_NUM_PRIV_IDS_PER_REGION        3
#define CSL_FW_IK3_MCU_EFUSE_MCU_0_SLV_NUM_PROTECTED_HW_REGIONS       1
#define CSL_FW_IK3_MCU_EFUSE_MCU_0_SLV_MEM_START                      0x40c00000
#define CSL_FW_IK3_MCU_EFUSE_MCU_0_SLV_MEM_END                        0x40c000ff

/* Properties of firewall protecting slave endpoint: IK3_PBIST_4C28P_WRAP_MCU_0.CFG*/
#define CSL_FW_IK3_PBIST_4C28P_WRAP_MCU_0_CFG_ID                      1212
#define CSL_FW_IK3_PBIST_4C28P_WRAP_MCU_0_CFG_TYPE                    CSL_FW_SECURITY
#define CSL_FW_IK3_PBIST_4C28P_WRAP_MCU_0_CFG_MMR_BASE                0x4512f000
#define CSL_FW_IK3_PBIST_4C28P_WRAP_MCU_0_CFG_NUM_REGIONS             1
#define CSL_FW_IK3_PBIST_4C28P_WRAP_MCU_0_CFG_NUM_PRIV_IDS_PER_REGION 3
#define CSL_FW_IK3_PBIST_4C28P_WRAP_MCU_0_CFG_NUM_PROTECTED_HW_REGIONS          1
#define CSL_FW_IK3_PBIST_4C28P_WRAP_MCU_0_CFG_MEM_START               0x40e00000
#define CSL_FW_IK3_PBIST_4C28P_WRAP_MCU_0_CFG_MEM_END                 0x40e003ff

/* Properties of firewall protecting slave endpoint: ICPSW_2GUSS_MCU_0.S_VBUSP*/
#define CSL_FW_ICPSW_2GUSS_MCU_0_S_VBUSP_ID                           1220
#define CSL_FW_ICPSW_2GUSS_MCU_0_S_VBUSP_TYPE                         CSL_FW_SECURITY
#define CSL_FW_ICPSW_2GUSS_MCU_0_S_VBUSP_MMR_BASE                     0x45131000
#define CSL_FW_ICPSW_2GUSS_MCU_0_S_VBUSP_NUM_REGIONS                  2
#define CSL_FW_ICPSW_2GUSS_MCU_0_S_VBUSP_NUM_PRIV_IDS_PER_REGION      3
#define CSL_FW_ICPSW_2GUSS_MCU_0_S_VBUSP_NUM_PROTECTED_HW_REGIONS     2
#define CSL_FW_ICPSW_2GUSS_MCU_0_S_VBUSP_CPSW_NUSS_VBUSP_START        0x46000000
#define CSL_FW_ICPSW_2GUSS_MCU_0_S_VBUSP_CPSW_NUSS_VBUSP_END          0x461fffff
#define CSL_FW_ICPSW_2GUSS_MCU_0_S_VBUSP_CPSW_NUSS_VBUSP_ECC_START    0x40709000
#define CSL_FW_ICPSW_2GUSS_MCU_0_S_VBUSP_CPSW_NUSS_VBUSP_ECC_END      0x407093ff

/* Properties of firewall protecting slave endpoint: IPDMA_MCU0_MCU_0.ECCAGGR_CFG*/
#define CSL_FW_IPDMA_MCU0_MCU_0_ECCAGGR_CFG_ID                        1240
#define CSL_FW_IPDMA_MCU0_MCU_0_ECCAGGR_CFG_TYPE                      CSL_FW_SECURITY
#define CSL_FW_IPDMA_MCU0_MCU_0_ECCAGGR_CFG_MMR_BASE                  0x45136000
#define CSL_FW_IPDMA_MCU0_MCU_0_ECCAGGR_CFG_NUM_REGIONS               1
#define CSL_FW_IPDMA_MCU0_MCU_0_ECCAGGR_CFG_NUM_PRIV_IDS_PER_REGION   3
#define CSL_FW_IPDMA_MCU0_MCU_0_ECCAGGR_CFG_NUM_PROTECTED_HW_REGIONS  1
#define CSL_FW_IPDMA_MCU0_MCU_0_ECCAGGR_CFG_REGS_START                0x40710000
#define CSL_FW_IPDMA_MCU0_MCU_0_ECCAGGR_CFG_REGS_END                  0x407103ff

/* Properties of firewall protecting slave endpoint: IPDMA_MCU1_MCU_0.ECCAGGR_CFG*/
#define CSL_FW_IPDMA_MCU1_MCU_0_ECCAGGR_CFG_ID                        1241
#define CSL_FW_IPDMA_MCU1_MCU_0_ECCAGGR_CFG_TYPE                      CSL_FW_SECURITY
#define CSL_FW_IPDMA_MCU1_MCU_0_ECCAGGR_CFG_MMR_BASE                  0x45136400
#define CSL_FW_IPDMA_MCU1_MCU_0_ECCAGGR_CFG_NUM_REGIONS               1
#define CSL_FW_IPDMA_MCU1_MCU_0_ECCAGGR_CFG_NUM_PRIV_IDS_PER_REGION   3
#define CSL_FW_IPDMA_MCU1_MCU_0_ECCAGGR_CFG_NUM_PROTECTED_HW_REGIONS  1
#define CSL_FW_IPDMA_MCU1_MCU_0_ECCAGGR_CFG_REGS_START                0x40711000
#define CSL_FW_IPDMA_MCU1_MCU_0_ECCAGGR_CFG_REGS_END                  0x407113ff

/* Properties of firewall protecting slave endpoint: IM4_MCU_CBASS_MCU_0.CBASS_ERR_SLV*/
#define CSL_FW_IM4_MCU_CBASS_MCU_0_CBASS_ERR_SLV_ID                   1244
#define CSL_FW_IM4_MCU_CBASS_MCU_0_CBASS_ERR_SLV_TYPE                 CSL_FW_SECURITY
#define CSL_FW_IM4_MCU_CBASS_MCU_0_CBASS_ERR_SLV_MMR_BASE             0x45137000
#define CSL_FW_IM4_MCU_CBASS_MCU_0_CBASS_ERR_SLV_NUM_REGIONS          1
#define CSL_FW_IM4_MCU_CBASS_MCU_0_CBASS_ERR_SLV_NUM_PRIV_IDS_PER_REGION        3
#define CSL_FW_IM4_MCU_CBASS_MCU_0_CBASS_ERR_SLV_NUM_PROTECTED_HW_REGIONS       1
#define CSL_FW_IM4_MCU_CBASS_MCU_0_CBASS_ERR_SLV_ERR_REGS_START       0x47100000
#define CSL_FW_IM4_MCU_CBASS_MCU_0_CBASS_ERR_SLV_ERR_REGS_END         0x471003ff

/* Properties of firewall protecting slave endpoint: IM4_MCU_DBG_CBASS_MCU_0.CBASS_ERR_SLV*/
#define CSL_FW_IM4_MCU_DBG_CBASS_MCU_0_CBASS_ERR_SLV_ID               1245
#define CSL_FW_IM4_MCU_DBG_CBASS_MCU_0_CBASS_ERR_SLV_TYPE             CSL_FW_SECURITY
#define CSL_FW_IM4_MCU_DBG_CBASS_MCU_0_CBASS_ERR_SLV_MMR_BASE         0x45137400
#define CSL_FW_IM4_MCU_DBG_CBASS_MCU_0_CBASS_ERR_SLV_NUM_REGIONS      1
#define CSL_FW_IM4_MCU_DBG_CBASS_MCU_0_CBASS_ERR_SLV_NUM_PRIV_IDS_PER_REGION    3
#define CSL_FW_IM4_MCU_DBG_CBASS_MCU_0_CBASS_ERR_SLV_NUM_PROTECTED_HW_REGIONS   1
#define CSL_FW_IM4_MCU_DBG_CBASS_MCU_0_CBASS_ERR_SLV_ERR_REGS_START   0x47104000
#define CSL_FW_IM4_MCU_DBG_CBASS_MCU_0_CBASS_ERR_SLV_ERR_REGS_END     0x471043ff

/* Properties of firewall protecting slave endpoint: IM4_MCU_FW_CBASS_MCU_0.CBASS_ERR_SLV*/
#define CSL_FW_IM4_MCU_FW_CBASS_MCU_0_CBASS_ERR_SLV_ID                1246
#define CSL_FW_IM4_MCU_FW_CBASS_MCU_0_CBASS_ERR_SLV_TYPE              CSL_FW_SECURITY
#define CSL_FW_IM4_MCU_FW_CBASS_MCU_0_CBASS_ERR_SLV_MMR_BASE          0x45137800
#define CSL_FW_IM4_MCU_FW_CBASS_MCU_0_CBASS_ERR_SLV_NUM_REGIONS       1
#define CSL_FW_IM4_MCU_FW_CBASS_MCU_0_CBASS_ERR_SLV_NUM_PRIV_IDS_PER_REGION     3
#define CSL_FW_IM4_MCU_FW_CBASS_MCU_0_CBASS_ERR_SLV_NUM_PROTECTED_HW_REGIONS    1
#define CSL_FW_IM4_MCU_FW_CBASS_MCU_0_CBASS_ERR_SLV_ERR_REGS_START    0x47108000
#define CSL_FW_IM4_MCU_FW_CBASS_MCU_0_CBASS_ERR_SLV_ERR_REGS_END      0x471083ff

/* Properties of firewall protecting slave endpoint: IM4_MCUCLK2_ECC_AGGR_MCU_0.CFG*/
#define CSL_FW_IM4_MCUCLK2_ECC_AGGR_MCU_0_CFG_ID                      1252
#define CSL_FW_IM4_MCUCLK2_ECC_AGGR_MCU_0_CFG_TYPE                    CSL_FW_SECURITY
#define CSL_FW_IM4_MCUCLK2_ECC_AGGR_MCU_0_CFG_MMR_BASE                0x45139000
#define CSL_FW_IM4_MCUCLK2_ECC_AGGR_MCU_0_CFG_NUM_REGIONS             1
#define CSL_FW_IM4_MCUCLK2_ECC_AGGR_MCU_0_CFG_NUM_PRIV_IDS_PER_REGION 3
#define CSL_FW_IM4_MCUCLK2_ECC_AGGR_MCU_0_CFG_NUM_PROTECTED_HW_REGIONS          1
#define CSL_FW_IM4_MCUCLK2_ECC_AGGR_MCU_0_CFG_REGS_START              0x47200000
#define CSL_FW_IM4_MCUCLK2_ECC_AGGR_MCU_0_CFG_REGS_END                0x472003ff

/* Properties of firewall protecting slave endpoint: IM4_MCUCLK4_ECC_AGGR_MCU_0.CFG*/
#define CSL_FW_IM4_MCUCLK4_ECC_AGGR_MCU_0_CFG_ID                      1253
#define CSL_FW_IM4_MCUCLK4_ECC_AGGR_MCU_0_CFG_TYPE                    CSL_FW_SECURITY
#define CSL_FW_IM4_MCUCLK4_ECC_AGGR_MCU_0_CFG_MMR_BASE                0x45139400
#define CSL_FW_IM4_MCUCLK4_ECC_AGGR_MCU_0_CFG_NUM_REGIONS             1
#define CSL_FW_IM4_MCUCLK4_ECC_AGGR_MCU_0_CFG_NUM_PRIV_IDS_PER_REGION 3
#define CSL_FW_IM4_MCUCLK4_ECC_AGGR_MCU_0_CFG_NUM_PROTECTED_HW_REGIONS          1
#define CSL_FW_IM4_MCUCLK4_ECC_AGGR_MCU_0_CFG_REGS_START              0x47204000
#define CSL_FW_IM4_MCUCLK4_ECC_AGGR_MCU_0_CFG_REGS_END                0x472043ff

/* Properties of firewall protecting slave endpoint: IEXPORT_VBUSM_64B_SLV_MCU_0.SLV*/
#define CSL_FW_IEXPORT_VBUSM_64B_SLV_MCU_0_SLV_ID                     1280
#define CSL_FW_IEXPORT_VBUSM_64B_SLV_MCU_0_SLV_TYPE                   CSL_FW_SECURITY
#define CSL_FW_IEXPORT_VBUSM_64B_SLV_MCU_0_SLV_MMR_BASE               0x45140000
#define CSL_FW_IEXPORT_VBUSM_64B_SLV_MCU_0_SLV_NUM_REGIONS            8
#define CSL_FW_IEXPORT_VBUSM_64B_SLV_MCU_0_SLV_NUM_PRIV_IDS_PER_REGION          3
#define CSL_FW_IEXPORT_VBUSM_64B_SLV_MCU_0_SLV_NUM_PROTECTED_HW_REGIONS         1
#define CSL_FW_IEXPORT_VBUSM_64B_SLV_MCU_0_SLV_MMR_START              0x20000000
#define CSL_FW_IEXPORT_VBUSM_64B_SLV_MCU_0_SLV_MMR_END                0x200003ff

/* Properties of firewall protecting slave endpoint: IEXPORT_VBUSM_64B_SLV_MAIN_0.SLV*/
#define CSL_FW_IEXPORT_VBUSM_64B_SLV_MAIN_0_SLV_ID                    2048
#define CSL_FW_IEXPORT_VBUSM_64B_SLV_MAIN_0_SLV_TYPE                  CSL_FW_SECURITY
#define CSL_FW_IEXPORT_VBUSM_64B_SLV_MAIN_0_SLV_MMR_BASE              0x45200000
#define CSL_FW_IEXPORT_VBUSM_64B_SLV_MAIN_0_SLV_NUM_REGIONS           8
#define CSL_FW_IEXPORT_VBUSM_64B_SLV_MAIN_0_SLV_NUM_PRIV_IDS_PER_REGION         3
#define CSL_FW_IEXPORT_VBUSM_64B_SLV_MAIN_0_SLV_NUM_PROTECTED_HW_REGIONS        1
#define CSL_FW_IEXPORT_VBUSM_64B_SLV_MAIN_0_SLV_MMR_START             0x40000000
#define CSL_FW_IEXPORT_VBUSM_64B_SLV_MAIN_0_SLV_MMR_END               0x400003ff

/* Properties of firewall protecting slave endpoint: INAVSS256L_MAIN_0.NBSS_CFG*/
#define CSL_FW_INAVSS256L_MAIN_0_NBSS_CFG_ID                          2049
#define CSL_FW_INAVSS256L_MAIN_0_NBSS_CFG_TYPE                        CSL_FW_SECURITY
#define CSL_FW_INAVSS256L_MAIN_0_NBSS_CFG_MMR_BASE                    0x45200400
#define CSL_FW_INAVSS256L_MAIN_0_NBSS_CFG_NUM_REGIONS                 8
#define CSL_FW_INAVSS256L_MAIN_0_NBSS_CFG_NUM_PRIV_IDS_PER_REGION     3
#define CSL_FW_INAVSS256L_MAIN_0_NBSS_CFG_NUM_PROTECTED_HW_REGIONS    8
#define CSL_FW_INAVSS256L_MAIN_0_NBSS_CFG_NBSS_CFG___ECCAGGR0__REGS_START       0x3801000
#define CSL_FW_INAVSS256L_MAIN_0_NBSS_CFG_NBSS_CFG___ECCAGGR0__REGS_END         0x38013ff
#define CSL_FW_INAVSS256L_MAIN_0_NBSS_CFG_NBSS_CFG_NB0__CFG__MEM_ATTR0__SLV_SF__CFG_START 0x3820000
#define CSL_FW_INAVSS256L_MAIN_0_NBSS_CFG_NBSS_CFG_NB0__CFG__MEM_ATTR0__SLV_SF__CFG_END   0x3827fff
#define CSL_FW_INAVSS256L_MAIN_0_NBSS_CFG_NBSS_CFG_NB0__CFG__MEM_ATTR1__SLV_SF__CFG_START 0x3828000
#define CSL_FW_INAVSS256L_MAIN_0_NBSS_CFG_NBSS_CFG_NB0__CFG__MEM_ATTR1__SLV_SF__CFG_END   0x382ffff
#define CSL_FW_INAVSS256L_MAIN_0_NBSS_CFG_NBSS_CFG_NB0__CFG__CFG__CFG__MMRS_START         0x3802000
#define CSL_FW_INAVSS256L_MAIN_0_NBSS_CFG_NBSS_CFG_NB0__CFG__CFG__CFG__MMRS_END 0x38020ff
#define CSL_FW_INAVSS256L_MAIN_0_NBSS_CFG_NBSS_CFG_NB1__CFG__CFG__CFG__MMRS_START         0x3803000
#define CSL_FW_INAVSS256L_MAIN_0_NBSS_CFG_NBSS_CFG_NB1__CFG__CFG__CFG__MMRS_END 0x38030ff
#define CSL_FW_INAVSS256L_MAIN_0_NBSS_CFG_NBSS_CFG_NB1__CFG__MEM_ATTR0__SLV_SF__CFG_START 0x3840000
#define CSL_FW_INAVSS256L_MAIN_0_NBSS_CFG_NBSS_CFG_NB1__CFG__MEM_ATTR0__SLV_SF__CFG_END   0x384ffff
#define CSL_FW_INAVSS256L_MAIN_0_NBSS_CFG_NBSS_CFG_NB1__CFG__MEM_ATTR1__SLV_SF__CFG_START 0x3850000
#define CSL_FW_INAVSS256L_MAIN_0_NBSS_CFG_NBSS_CFG_NB1__CFG__MEM_ATTR1__SLV_SF__CFG_END   0x385ffff
#define CSL_FW_INAVSS256L_MAIN_0_NBSS_CFG_NBSS_CFG_REGS0__CFG__MMRS_START       0x3800000
#define CSL_FW_INAVSS256L_MAIN_0_NBSS_CFG_NBSS_CFG_REGS0__CFG__MMRS_END         0x38000ff

/* Properties of firewall protecting slave endpoint: ICOMPUTE_CLUSTER_MAXWELL_TB_VDC_MAIN_0.VBUSP_CFG*/
#define CSL_FW_ICOMPUTE_CLUSTER_MAXWELL_TB_VDC_MAIN_0_VBUSP_CFG_ID    2052
#define CSL_FW_ICOMPUTE_CLUSTER_MAXWELL_TB_VDC_MAIN_0_VBUSP_CFG_TYPE  CSL_FW_SECURITY
#define CSL_FW_ICOMPUTE_CLUSTER_MAXWELL_TB_VDC_MAIN_0_VBUSP_CFG_MMR_BASE        0x45201000
#define CSL_FW_ICOMPUTE_CLUSTER_MAXWELL_TB_VDC_MAIN_0_VBUSP_CFG_NUM_REGIONS     3
#define CSL_FW_ICOMPUTE_CLUSTER_MAXWELL_TB_VDC_MAIN_0_VBUSP_CFG_NUM_PRIV_IDS_PER_REGION   3
#define CSL_FW_ICOMPUTE_CLUSTER_MAXWELL_TB_VDC_MAIN_0_VBUSP_CFG_NUM_PROTECTED_HW_REGIONS  8
#define CSL_FW_ICOMPUTE_CLUSTER_MAXWELL_TB_VDC_MAIN_0_VBUSP_CFG___VBUSP_CFG0__CFG_ARM_PBIST0_START  0x4d10010000
#define CSL_FW_ICOMPUTE_CLUSTER_MAXWELL_TB_VDC_MAIN_0_VBUSP_CFG___VBUSP_CFG0__CFG_ARM_PBIST0_END    0x4d1001ffff
#define CSL_FW_ICOMPUTE_CLUSTER_MAXWELL_TB_VDC_MAIN_0_VBUSP_CFG___VBUSP_CFG0__CFG_ARM_ECC0_START    0x4d20010000
#define CSL_FW_ICOMPUTE_CLUSTER_MAXWELL_TB_VDC_MAIN_0_VBUSP_CFG___VBUSP_CFG0__CFG_ARM_ECC0_END      0x4d2001ffff
#define CSL_FW_ICOMPUTE_CLUSTER_MAXWELL_TB_VDC_MAIN_0_VBUSP_CFG___VBUSP_CFG1__CFG_ARM_PBIST1_START  0x4d10020000
#define CSL_FW_ICOMPUTE_CLUSTER_MAXWELL_TB_VDC_MAIN_0_VBUSP_CFG___VBUSP_CFG1__CFG_ARM_PBIST1_END    0x4d1002ffff
#define CSL_FW_ICOMPUTE_CLUSTER_MAXWELL_TB_VDC_MAIN_0_VBUSP_CFG___VBUSP_CFG1__CFG_ARM_ECC1_START    0x4d20020000
#define CSL_FW_ICOMPUTE_CLUSTER_MAXWELL_TB_VDC_MAIN_0_VBUSP_CFG___VBUSP_CFG1__CFG_ARM_ECC1_END      0x4d2002ffff
#define CSL_FW_ICOMPUTE_CLUSTER_MAXWELL_TB_VDC_MAIN_0_VBUSP_CFG___VBUSP_MSMC_ECC_AGGR0__CFG_MSMC_ECC0_START   0x4d20000000
#define CSL_FW_ICOMPUTE_CLUSTER_MAXWELL_TB_VDC_MAIN_0_VBUSP_CFG___VBUSP_MSMC_ECC_AGGR0__CFG_MSMC_ECC0_END     0x4d200003ff
#define CSL_FW_ICOMPUTE_CLUSTER_MAXWELL_TB_VDC_MAIN_0_VBUSP_CFG___VBUSP_MSMC_ECC_AGGR1__CFG_MSMC_ECC1_START   0x4d20000400
#define CSL_FW_ICOMPUTE_CLUSTER_MAXWELL_TB_VDC_MAIN_0_VBUSP_CFG___VBUSP_MSMC_ECC_AGGR1__CFG_MSMC_ECC1_END     0x4d200007ff
#define CSL_FW_ICOMPUTE_CLUSTER_MAXWELL_TB_VDC_MAIN_0_VBUSP_CFG___VBUSP4_CFG_MSMC_PBIST__CFG_MSMC_PBIST_START 0x4d10000000
#define CSL_FW_ICOMPUTE_CLUSTER_MAXWELL_TB_VDC_MAIN_0_VBUSP_CFG___VBUSP4_CFG_MSMC_PBIST__CFG_MSMC_PBIST_END   0x4d1000ffff
#define CSL_FW_ICOMPUTE_CLUSTER_MAXWELL_TB_VDC_MAIN_0_VBUSP_CFG___VBUSP4_CFG_CLEC__CFG_CLEC_START   0x4d00000000
#define CSL_FW_ICOMPUTE_CLUSTER_MAXWELL_TB_VDC_MAIN_0_VBUSP_CFG___VBUSP4_CFG_CLEC__CFG_CLEC_END     0x4d07ffffff

/* Properties of firewall protecting slave endpoint: IEMMC4SD3SS_GS80_MAIN_0.EMMCSDSS_CFG*/
#define CSL_FW_IEMMC4SD3SS_GS80_MAIN_0_EMMCSDSS_CFG_ID                2057
#define CSL_FW_IEMMC4SD3SS_GS80_MAIN_0_EMMCSDSS_CFG_TYPE              CSL_FW_SECURITY
#define CSL_FW_IEMMC4SD3SS_GS80_MAIN_0_EMMCSDSS_CFG_MMR_BASE          0x45202400
#define CSL_FW_IEMMC4SD3SS_GS80_MAIN_0_EMMCSDSS_CFG_NUM_REGIONS       4
#define CSL_FW_IEMMC4SD3SS_GS80_MAIN_0_EMMCSDSS_CFG_NUM_PRIV_IDS_PER_REGION     3
#define CSL_FW_IEMMC4SD3SS_GS80_MAIN_0_EMMCSDSS_CFG_NUM_PROTECTED_HW_REGIONS    4
#define CSL_FW_IEMMC4SD3SS_GS80_MAIN_0_EMMCSDSS_CFG_REGS__SS_CFG__SSCFG_START   0x4f90000
#define CSL_FW_IEMMC4SD3SS_GS80_MAIN_0_EMMCSDSS_CFG_REGS__SS_CFG__SSCFG_END     0x4f903ff
#define CSL_FW_IEMMC4SD3SS_GS80_MAIN_0_EMMCSDSS_CFG_SDHC_WRAP__CTL_CFG__CTLCFG_START      0x4f80000
#define CSL_FW_IEMMC4SD3SS_GS80_MAIN_0_EMMCSDSS_CFG_SDHC_WRAP__CTL_CFG__CTLCFG_END        0x4f80fff
#define CSL_FW_IEMMC4SD3SS_GS80_MAIN_0_EMMCSDSS_CFG_ECC_AGGR_TXMEM__CFG__REGS_START       0x2a25000
#define CSL_FW_IEMMC4SD3SS_GS80_MAIN_0_EMMCSDSS_CFG_ECC_AGGR_TXMEM__CFG__REGS_END         0x2a253ff
#define CSL_FW_IEMMC4SD3SS_GS80_MAIN_0_EMMCSDSS_CFG_ECC_AGGR_RXMEM__CFG__REGS_START       0x2a24000
#define CSL_FW_IEMMC4SD3SS_GS80_MAIN_0_EMMCSDSS_CFG_ECC_AGGR_RXMEM__CFG__REGS_END         0x2a243ff

/* Properties of firewall protecting slave endpoint: IEMMC2SD3SS_GS80_MAIN_0.EMMCSDSS_CFG*/
#define CSL_FW_IEMMC2SD3SS_GS80_MAIN_0_EMMCSDSS_CFG_ID                2058
#define CSL_FW_IEMMC2SD3SS_GS80_MAIN_0_EMMCSDSS_CFG_TYPE              CSL_FW_SECURITY
#define CSL_FW_IEMMC2SD3SS_GS80_MAIN_0_EMMCSDSS_CFG_MMR_BASE          0x45202800
#define CSL_FW_IEMMC2SD3SS_GS80_MAIN_0_EMMCSDSS_CFG_NUM_REGIONS       4
#define CSL_FW_IEMMC2SD3SS_GS80_MAIN_0_EMMCSDSS_CFG_NUM_PRIV_IDS_PER_REGION     3
#define CSL_FW_IEMMC2SD3SS_GS80_MAIN_0_EMMCSDSS_CFG_NUM_PROTECTED_HW_REGIONS    4
#define CSL_FW_IEMMC2SD3SS_GS80_MAIN_0_EMMCSDSS_CFG_REGS__SS_CFG__SSCFG_START   0x4fb0000
#define CSL_FW_IEMMC2SD3SS_GS80_MAIN_0_EMMCSDSS_CFG_REGS__SS_CFG__SSCFG_END     0x4fb03ff
#define CSL_FW_IEMMC2SD3SS_GS80_MAIN_0_EMMCSDSS_CFG_SDHC_WRAP__CTL_CFG__CTLCFG_START      0x4fa0000
#define CSL_FW_IEMMC2SD3SS_GS80_MAIN_0_EMMCSDSS_CFG_SDHC_WRAP__CTL_CFG__CTLCFG_END        0x4fa0fff
#define CSL_FW_IEMMC2SD3SS_GS80_MAIN_0_EMMCSDSS_CFG_ECC_AGGR_TXMEM__CFG__REGS_START       0x2a27000
#define CSL_FW_IEMMC2SD3SS_GS80_MAIN_0_EMMCSDSS_CFG_ECC_AGGR_TXMEM__CFG__REGS_END         0x2a273ff
#define CSL_FW_IEMMC2SD3SS_GS80_MAIN_0_EMMCSDSS_CFG_ECC_AGGR_RXMEM__CFG__REGS_START       0x2a26000
#define CSL_FW_IEMMC2SD3SS_GS80_MAIN_0_EMMCSDSS_CFG_ECC_AGGR_RXMEM__CFG__REGS_END         0x2a263ff

/* Properties of firewall protecting slave endpoint: IICSS_G_MAIN_0.PR1_SLV*/
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_ID                              2064
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_TYPE                            CSL_FW_SECURITY
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_MMR_BASE                        0x45204000
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_NUM_REGIONS                     8
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_NUM_PRIV_IDS_PER_REGION         3
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_NUM_PROTECTED_HW_REGIONS        37
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_BORG_ECC_AGGR__CFG__REGS_START  0xbf00000
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_BORG_ECC_AGGR__CFG__REGS_END    0xbf003ff
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_PA_STAT_WRAP__PA_SLV__CSTAT_START         0xb02c000
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_PA_STAT_WRAP__PA_SLV__CSTAT_END 0xb02cfff
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_PA_STAT_WRAP__PA_SLV__QSTAT_START         0xb027000
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_PA_STAT_WRAP__PA_SLV__QSTAT_END 0xb027fff
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_PA_STAT_WRAP__PA_SLV__REGS_START          0xb03c000
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_PA_STAT_WRAP__PA_SLV__REGS_END  0xb03c0ff
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_PR1_PROTECT__SLV__REGS_START    0xb024c00
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_PR1_PROTECT__SLV__REGS_END      0xb024cff
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_PR1_TASKS_MGR_RTU1__PR1_TASKS_MGR_RTU1_MMR__REGS_START        0xb02a300
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_PR1_TASKS_MGR_RTU1__PR1_TASKS_MGR_RTU1_MMR__REGS_END          0xb02a3ff
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_PR1_TASKS_MGR_RTU0__PR1_TASKS_MGR_RTU0_MMR__REGS_START        0xb02a100
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_PR1_TASKS_MGR_RTU0__PR1_TASKS_MGR_RTU0_MMR__REGS_END          0xb02a1ff
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_PR1_TASKS_MGR_PRU1__PR1_TASKS_MGR_PRU1_MMR__REGS_START        0xb02a200
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_PR1_TASKS_MGR_PRU1__PR1_TASKS_MGR_PRU1_MMR__REGS_END          0xb02a2ff
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_PR1_TASKS_MGR_PRU0__PR1_TASKS_MGR_PRU0_MMR__REGS_START        0xb02a000
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_PR1_TASKS_MGR_PRU0__PR1_TASKS_MGR_PRU0_MMR__REGS_END          0xb02a0ff
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_PR1_MDIO_V1P7__MDIO__REGS_START 0xb032400
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_PR1_MDIO_V1P7__MDIO__REGS_END   0xb0324ff
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_PR1_RTU1__PR1_RTU1_IRAM__RAM_START        0xb006000
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_PR1_RTU1__PR1_RTU1_IRAM__RAM_END          0xb007fff
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_PR1_RTU1__PR1_RTU1_IRAM__REGS_START       0xb023800
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_PR1_RTU1__PR1_RTU1_IRAM__REGS_END         0xb0238ff
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_PR1_RTU1__PR1_RTU1_IRAM__DEBUG_START      0xb023c00
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_PR1_RTU1__PR1_RTU1_IRAM__DEBUG_END        0xb023cff
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_PR1_RTU0__PR1_RTU0_IRAM__RAM_START        0xb004000
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_PR1_RTU0__PR1_RTU0_IRAM__RAM_END          0xb005fff
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_PR1_RTU0__PR1_RTU0_IRAM__REGS_START       0xb023000
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_PR1_RTU0__PR1_RTU0_IRAM__REGS_END         0xb0230ff
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_PR1_RTU0__PR1_RTU0_IRAM__DEBUG_START      0xb023400
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_PR1_RTU0__PR1_RTU0_IRAM__DEBUG_END        0xb0234ff
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_PR1_MII_RT__PR1_SGMII1_CFG__SGMII1_REGS_START       0xb032200
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_PR1_MII_RT__PR1_SGMII1_CFG__SGMII1_REGS_END         0xb0322ff
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_PR1_MII_RT__PR1_SGMII0_CFG__SGMII0_REGS_START       0xb032100
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_PR1_MII_RT__PR1_SGMII0_CFG__SGMII0_REGS_END         0xb0321ff
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_PR1_MII_RT__PR1_MII_RT_CFG__REGS_START    0xb032000
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_PR1_MII_RT__PR1_MII_RT_CFG__REGS_END      0xb0320ff
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_PR1_MII_RT__PR1_MII_RT_G_CFG__REGS_G_START          0xb033000
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_PR1_MII_RT__PR1_MII_RT_G_CFG__REGS_G_END  0xb033fff
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_PR1_CFG__SLV__REGS_START        0xb026000
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_PR1_CFG__SLV__REGS_END          0xb0261ff
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_PR1_PDSP0__IRAM__REGS_START     0xb022000
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_PR1_PDSP0__IRAM__REGS_END       0xb0220ff
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_PR1_PDSP0__IRAM__DEBUG_START    0xb022400
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_PR1_PDSP0__IRAM__DEBUG_END      0xb0224ff
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_PR1_PDSP0__IRAM__RAM_START      0xb034000
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_PR1_PDSP0__IRAM__RAM_END        0xb037fff
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_PR1_PDSP1__IRAM__REGS_START     0xb024000
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_PR1_PDSP1__IRAM__REGS_END       0xb0240ff
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_PR1_PDSP1__IRAM__DEBUG_START    0xb024400
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_PR1_PDSP1__IRAM__DEBUG_END      0xb0244ff
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_PR1_PDSP1__IRAM__RAM_START      0xb038000
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_PR1_PDSP1__IRAM__RAM_END        0xb03bfff
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_RAM__SLV__RAM_START             0xb010000
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_RAM__SLV__RAM_END               0xb01ffff
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_DRAM0__SLV__RAM_START           0xb000000
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_DRAM0__SLV__RAM_END             0xb001fff
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_DRAM1__SLV__RAM_START           0xb002000
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_DRAM1__SLV__RAM_END             0xb003fff
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_PR1_ICSS_ECAP0__ECAP_SLV__REGS_START      0xb030000
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_PR1_ICSS_ECAP0__ECAP_SLV__REGS_END        0xb0300ff
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_PR1_ICSS_INTC__INTC_SLV__REGS_START       0xb020000
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_PR1_ICSS_INTC__INTC_SLV__REGS_END         0xb021fff
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_PR1_ICSS_UART__UART_SLV__REGS_START       0xb028000
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_PR1_ICSS_UART__UART_SLV__REGS_END         0xb02803f
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_PR1_IEP1__SLV__REGS_START       0xb02f000
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_PR1_IEP1__SLV__REGS_END         0xb02ffff
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_PR1_IEP0__SLV__REGS_START       0xb02e000
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_PR1_IEP0__SLV__REGS_END         0xb02efff
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_PR1_RAT_SLICE0__CFG__MMRS_START 0xb008000
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_PR1_RAT_SLICE0__CFG__MMRS_END   0xb008fff
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_PR1_RAT_SLICE1__CFG__MMRS_START 0xb009000
#define CSL_FW_IICSS_G_MAIN_0_PR1_SLV_PR1_RAT_SLICE1__CFG__MMRS_END   0xb009fff

/* Properties of firewall protecting slave endpoint: IICSS_G_MAIN_1.PR1_SLV*/
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_ID                              2065
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_TYPE                            CSL_FW_SECURITY
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_MMR_BASE                        0x45204400
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_NUM_REGIONS                     8
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_NUM_PRIV_IDS_PER_REGION         3
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_NUM_PROTECTED_HW_REGIONS        37
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_BORG_ECC_AGGR__CFG__REGS_START  0xbf01000
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_BORG_ECC_AGGR__CFG__REGS_END    0xbf013ff
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_PA_STAT_WRAP__PA_SLV__CSTAT_START         0xb12c000
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_PA_STAT_WRAP__PA_SLV__CSTAT_END 0xb12cfff
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_PA_STAT_WRAP__PA_SLV__QSTAT_START         0xb127000
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_PA_STAT_WRAP__PA_SLV__QSTAT_END 0xb127fff
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_PA_STAT_WRAP__PA_SLV__REGS_START          0xb13c000
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_PA_STAT_WRAP__PA_SLV__REGS_END  0xb13c0ff
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_PR1_PROTECT__SLV__REGS_START    0xb124c00
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_PR1_PROTECT__SLV__REGS_END      0xb124cff
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_PR1_TASKS_MGR_RTU1__PR1_TASKS_MGR_RTU1_MMR__REGS_START        0xb12a300
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_PR1_TASKS_MGR_RTU1__PR1_TASKS_MGR_RTU1_MMR__REGS_END          0xb12a3ff
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_PR1_TASKS_MGR_RTU0__PR1_TASKS_MGR_RTU0_MMR__REGS_START        0xb12a100
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_PR1_TASKS_MGR_RTU0__PR1_TASKS_MGR_RTU0_MMR__REGS_END          0xb12a1ff
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_PR1_TASKS_MGR_PRU1__PR1_TASKS_MGR_PRU1_MMR__REGS_START        0xb12a200
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_PR1_TASKS_MGR_PRU1__PR1_TASKS_MGR_PRU1_MMR__REGS_END          0xb12a2ff
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_PR1_TASKS_MGR_PRU0__PR1_TASKS_MGR_PRU0_MMR__REGS_START        0xb12a000
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_PR1_TASKS_MGR_PRU0__PR1_TASKS_MGR_PRU0_MMR__REGS_END          0xb12a0ff
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_PR1_MDIO_V1P7__MDIO__REGS_START 0xb132400
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_PR1_MDIO_V1P7__MDIO__REGS_END   0xb1324ff
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_PR1_RTU1__PR1_RTU1_IRAM__RAM_START        0xb106000
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_PR1_RTU1__PR1_RTU1_IRAM__RAM_END          0xb107fff
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_PR1_RTU1__PR1_RTU1_IRAM__REGS_START       0xb123800
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_PR1_RTU1__PR1_RTU1_IRAM__REGS_END         0xb1238ff
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_PR1_RTU1__PR1_RTU1_IRAM__DEBUG_START      0xb123c00
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_PR1_RTU1__PR1_RTU1_IRAM__DEBUG_END        0xb123cff
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_PR1_RTU0__PR1_RTU0_IRAM__RAM_START        0xb104000
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_PR1_RTU0__PR1_RTU0_IRAM__RAM_END          0xb105fff
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_PR1_RTU0__PR1_RTU0_IRAM__REGS_START       0xb123000
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_PR1_RTU0__PR1_RTU0_IRAM__REGS_END         0xb1230ff
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_PR1_RTU0__PR1_RTU0_IRAM__DEBUG_START      0xb123400
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_PR1_RTU0__PR1_RTU0_IRAM__DEBUG_END        0xb1234ff
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_PR1_MII_RT__PR1_SGMII1_CFG__SGMII1_REGS_START       0xb132200
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_PR1_MII_RT__PR1_SGMII1_CFG__SGMII1_REGS_END         0xb1322ff
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_PR1_MII_RT__PR1_SGMII0_CFG__SGMII0_REGS_START       0xb132100
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_PR1_MII_RT__PR1_SGMII0_CFG__SGMII0_REGS_END         0xb1321ff
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_PR1_MII_RT__PR1_MII_RT_CFG__REGS_START    0xb132000
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_PR1_MII_RT__PR1_MII_RT_CFG__REGS_END      0xb1320ff
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_PR1_MII_RT__PR1_MII_RT_G_CFG__REGS_G_START          0xb133000
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_PR1_MII_RT__PR1_MII_RT_G_CFG__REGS_G_END  0xb133fff
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_PR1_CFG__SLV__REGS_START        0xb126000
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_PR1_CFG__SLV__REGS_END          0xb1261ff
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_PR1_PDSP0__IRAM__REGS_START     0xb122000
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_PR1_PDSP0__IRAM__REGS_END       0xb1220ff
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_PR1_PDSP0__IRAM__DEBUG_START    0xb122400
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_PR1_PDSP0__IRAM__DEBUG_END      0xb1224ff
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_PR1_PDSP0__IRAM__RAM_START      0xb134000
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_PR1_PDSP0__IRAM__RAM_END        0xb137fff
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_PR1_PDSP1__IRAM__REGS_START     0xb124000
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_PR1_PDSP1__IRAM__REGS_END       0xb1240ff
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_PR1_PDSP1__IRAM__DEBUG_START    0xb124400
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_PR1_PDSP1__IRAM__DEBUG_END      0xb1244ff
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_PR1_PDSP1__IRAM__RAM_START      0xb138000
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_PR1_PDSP1__IRAM__RAM_END        0xb13bfff
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_RAM__SLV__RAM_START             0xb110000
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_RAM__SLV__RAM_END               0xb11ffff
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_DRAM0__SLV__RAM_START           0xb100000
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_DRAM0__SLV__RAM_END             0xb101fff
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_DRAM1__SLV__RAM_START           0xb102000
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_DRAM1__SLV__RAM_END             0xb103fff
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_PR1_ICSS_ECAP0__ECAP_SLV__REGS_START      0xb130000
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_PR1_ICSS_ECAP0__ECAP_SLV__REGS_END        0xb1300ff
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_PR1_ICSS_INTC__INTC_SLV__REGS_START       0xb120000
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_PR1_ICSS_INTC__INTC_SLV__REGS_END         0xb121fff
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_PR1_ICSS_UART__UART_SLV__REGS_START       0xb128000
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_PR1_ICSS_UART__UART_SLV__REGS_END         0xb12803f
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_PR1_IEP1__SLV__REGS_START       0xb12f000
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_PR1_IEP1__SLV__REGS_END         0xb12ffff
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_PR1_IEP0__SLV__REGS_START       0xb12e000
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_PR1_IEP0__SLV__REGS_END         0xb12efff
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_PR1_RAT_SLICE0__CFG__MMRS_START 0xb108000
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_PR1_RAT_SLICE0__CFG__MMRS_END   0xb108fff
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_PR1_RAT_SLICE1__CFG__MMRS_START 0xb109000
#define CSL_FW_IICSS_G_MAIN_1_PR1_SLV_PR1_RAT_SLICE1__CFG__MMRS_END   0xb109fff

/* Properties of firewall protecting slave endpoint: IICSS_G_MAIN_2.PR1_SLV*/
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_ID                              2066
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_TYPE                            CSL_FW_SECURITY
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_MMR_BASE                        0x45204800
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_NUM_REGIONS                     8
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_NUM_PRIV_IDS_PER_REGION         3
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_NUM_PROTECTED_HW_REGIONS        37
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_BORG_ECC_AGGR__CFG__REGS_START  0xbf02000
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_BORG_ECC_AGGR__CFG__REGS_END    0xbf023ff
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_PA_STAT_WRAP__PA_SLV__CSTAT_START         0xb22c000
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_PA_STAT_WRAP__PA_SLV__CSTAT_END 0xb22cfff
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_PA_STAT_WRAP__PA_SLV__QSTAT_START         0xb227000
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_PA_STAT_WRAP__PA_SLV__QSTAT_END 0xb227fff
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_PA_STAT_WRAP__PA_SLV__REGS_START          0xb23c000
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_PA_STAT_WRAP__PA_SLV__REGS_END  0xb23c0ff
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_PR1_PROTECT__SLV__REGS_START    0xb224c00
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_PR1_PROTECT__SLV__REGS_END      0xb224cff
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_PR1_TASKS_MGR_RTU1__PR1_TASKS_MGR_RTU1_MMR__REGS_START        0xb22a300
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_PR1_TASKS_MGR_RTU1__PR1_TASKS_MGR_RTU1_MMR__REGS_END          0xb22a3ff
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_PR1_TASKS_MGR_RTU0__PR1_TASKS_MGR_RTU0_MMR__REGS_START        0xb22a100
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_PR1_TASKS_MGR_RTU0__PR1_TASKS_MGR_RTU0_MMR__REGS_END          0xb22a1ff
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_PR1_TASKS_MGR_PRU1__PR1_TASKS_MGR_PRU1_MMR__REGS_START        0xb22a200
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_PR1_TASKS_MGR_PRU1__PR1_TASKS_MGR_PRU1_MMR__REGS_END          0xb22a2ff
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_PR1_TASKS_MGR_PRU0__PR1_TASKS_MGR_PRU0_MMR__REGS_START        0xb22a000
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_PR1_TASKS_MGR_PRU0__PR1_TASKS_MGR_PRU0_MMR__REGS_END          0xb22a0ff
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_PR1_MDIO_V1P7__MDIO__REGS_START 0xb232400
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_PR1_MDIO_V1P7__MDIO__REGS_END   0xb2324ff
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_PR1_RTU1__PR1_RTU1_IRAM__RAM_START        0xb206000
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_PR1_RTU1__PR1_RTU1_IRAM__RAM_END          0xb207fff
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_PR1_RTU1__PR1_RTU1_IRAM__REGS_START       0xb223800
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_PR1_RTU1__PR1_RTU1_IRAM__REGS_END         0xb2238ff
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_PR1_RTU1__PR1_RTU1_IRAM__DEBUG_START      0xb223c00
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_PR1_RTU1__PR1_RTU1_IRAM__DEBUG_END        0xb223cff
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_PR1_RTU0__PR1_RTU0_IRAM__RAM_START        0xb204000
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_PR1_RTU0__PR1_RTU0_IRAM__RAM_END          0xb205fff
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_PR1_RTU0__PR1_RTU0_IRAM__REGS_START       0xb223000
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_PR1_RTU0__PR1_RTU0_IRAM__REGS_END         0xb2230ff
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_PR1_RTU0__PR1_RTU0_IRAM__DEBUG_START      0xb223400
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_PR1_RTU0__PR1_RTU0_IRAM__DEBUG_END        0xb2234ff
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_PR1_MII_RT__PR1_SGMII1_CFG__SGMII1_REGS_START       0xb232200
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_PR1_MII_RT__PR1_SGMII1_CFG__SGMII1_REGS_END         0xb2322ff
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_PR1_MII_RT__PR1_SGMII0_CFG__SGMII0_REGS_START       0xb232100
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_PR1_MII_RT__PR1_SGMII0_CFG__SGMII0_REGS_END         0xb2321ff
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_PR1_MII_RT__PR1_MII_RT_CFG__REGS_START    0xb232000
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_PR1_MII_RT__PR1_MII_RT_CFG__REGS_END      0xb2320ff
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_PR1_MII_RT__PR1_MII_RT_G_CFG__REGS_G_START          0xb233000
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_PR1_MII_RT__PR1_MII_RT_G_CFG__REGS_G_END  0xb233fff
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_PR1_CFG__SLV__REGS_START        0xb226000
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_PR1_CFG__SLV__REGS_END          0xb2261ff
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_PR1_PDSP0__IRAM__REGS_START     0xb222000
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_PR1_PDSP0__IRAM__REGS_END       0xb2220ff
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_PR1_PDSP0__IRAM__DEBUG_START    0xb222400
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_PR1_PDSP0__IRAM__DEBUG_END      0xb2224ff
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_PR1_PDSP0__IRAM__RAM_START      0xb234000
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_PR1_PDSP0__IRAM__RAM_END        0xb237fff
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_PR1_PDSP1__IRAM__REGS_START     0xb224000
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_PR1_PDSP1__IRAM__REGS_END       0xb2240ff
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_PR1_PDSP1__IRAM__DEBUG_START    0xb224400
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_PR1_PDSP1__IRAM__DEBUG_END      0xb2244ff
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_PR1_PDSP1__IRAM__RAM_START      0xb238000
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_PR1_PDSP1__IRAM__RAM_END        0xb23bfff
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_RAM__SLV__RAM_START             0xb210000
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_RAM__SLV__RAM_END               0xb21ffff
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_DRAM0__SLV__RAM_START           0xb200000
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_DRAM0__SLV__RAM_END             0xb201fff
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_DRAM1__SLV__RAM_START           0xb202000
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_DRAM1__SLV__RAM_END             0xb203fff
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_PR1_ICSS_ECAP0__ECAP_SLV__REGS_START      0xb230000
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_PR1_ICSS_ECAP0__ECAP_SLV__REGS_END        0xb2300ff
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_PR1_ICSS_INTC__INTC_SLV__REGS_START       0xb220000
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_PR1_ICSS_INTC__INTC_SLV__REGS_END         0xb221fff
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_PR1_ICSS_UART__UART_SLV__REGS_START       0xb228000
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_PR1_ICSS_UART__UART_SLV__REGS_END         0xb22803f
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_PR1_IEP1__SLV__REGS_START       0xb22f000
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_PR1_IEP1__SLV__REGS_END         0xb22ffff
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_PR1_IEP0__SLV__REGS_START       0xb22e000
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_PR1_IEP0__SLV__REGS_END         0xb22efff
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_PR1_RAT_SLICE0__CFG__MMRS_START 0xb208000
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_PR1_RAT_SLICE0__CFG__MMRS_END   0xb208fff
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_PR1_RAT_SLICE1__CFG__MMRS_START 0xb209000
#define CSL_FW_IICSS_G_MAIN_2_PR1_SLV_PR1_RAT_SLICE1__CFG__MMRS_END   0xb209fff

/* Properties of firewall protecting slave endpoint: ICAL_MAIN_0.S_CFG*/
#define CSL_FW_ICAL_MAIN_0_S_CFG_ID                                   2096
#define CSL_FW_ICAL_MAIN_0_S_CFG_TYPE                                 CSL_FW_SECURITY
#define CSL_FW_ICAL_MAIN_0_S_CFG_MMR_BASE                             0x4520c000
#define CSL_FW_ICAL_MAIN_0_S_CFG_NUM_REGIONS                          1
#define CSL_FW_ICAL_MAIN_0_S_CFG_NUM_PRIV_IDS_PER_REGION              3
#define CSL_FW_ICAL_MAIN_0_S_CFG_NUM_PROTECTED_HW_REGIONS             1
#define CSL_FW_ICAL_MAIN_0_S_CFG_CAL_REGS_START                       0x6f00000
#define CSL_FW_ICAL_MAIN_0_S_CFG_CAL_REGS_END                         0x6f03fff

/* Properties of firewall protecting slave endpoint: ISA2_UL_MAIN_0.SA_UL_CFG*/
#define CSL_FW_ISA2_UL_MAIN_0_SA_UL_CFG_ID                            2112
#define CSL_FW_ISA2_UL_MAIN_0_SA_UL_CFG_TYPE                          CSL_FW_SECURITY
#define CSL_FW_ISA2_UL_MAIN_0_SA_UL_CFG_MMR_BASE                      0x45210000
#define CSL_FW_ISA2_UL_MAIN_0_SA_UL_CFG_NUM_REGIONS                   5
#define CSL_FW_ISA2_UL_MAIN_0_SA_UL_CFG_NUM_PRIV_IDS_PER_REGION       3
#define CSL_FW_ISA2_UL_MAIN_0_SA_UL_CFG_NUM_PROTECTED_HW_REGIONS      5
#define CSL_FW_ISA2_UL_MAIN_0_SA_UL_CFG_MMRS_START                    0x4e00000
#define CSL_FW_ISA2_UL_MAIN_0_SA_UL_CFG_MMRS_END                      0x4e00fff
#define CSL_FW_ISA2_UL_MAIN_0_SA_UL_CFG_MMRA_REGS_START               0x4e01000
#define CSL_FW_ISA2_UL_MAIN_0_SA_UL_CFG_MMRA_REGS_END                 0x4e011ff
#define CSL_FW_ISA2_UL_MAIN_0_SA_UL_CFG_ECC_REGS_START                0x2a23000
#define CSL_FW_ISA2_UL_MAIN_0_SA_UL_CFG_ECC_REGS_END                  0x2a233ff
#define CSL_FW_ISA2_UL_MAIN_0_SA_UL_CFG_EIP_76D_8_BCDF_EIP76_REGISTERS_START    0x4e10000
#define CSL_FW_ISA2_UL_MAIN_0_SA_UL_CFG_EIP_76D_8_BCDF_EIP76_REGISTERS_END      0x4e1007f
#define CSL_FW_ISA2_UL_MAIN_0_SA_UL_CFG_EIP_29T2_REGS_START           0x4e20000
#define CSL_FW_ISA2_UL_MAIN_0_SA_UL_CFG_EIP_29T2_REGS_END             0x4e2ffff

/* Properties of firewall protecting slave endpoint: IELM_MAIN_0.ELM_CBA*/
#define CSL_FW_IELM_MAIN_0_ELM_CBA_ID                                 2128
#define CSL_FW_IELM_MAIN_0_ELM_CBA_TYPE                               CSL_FW_SECURITY
#define CSL_FW_IELM_MAIN_0_ELM_CBA_MMR_BASE                           0x45214000
#define CSL_FW_IELM_MAIN_0_ELM_CBA_NUM_REGIONS                        1
#define CSL_FW_IELM_MAIN_0_ELM_CBA_NUM_PRIV_IDS_PER_REGION            3
#define CSL_FW_IELM_MAIN_0_ELM_CBA_NUM_PROTECTED_HW_REGIONS           1
#define CSL_FW_IELM_MAIN_0_ELM_CBA_MEM_START                          0x5380000
#define CSL_FW_IELM_MAIN_0_ELM_CBA_MEM_END                            0x5380fff

/* Properties of firewall protecting slave endpoint: IGPMC_MAIN_0.SLV*/
#define CSL_FW_IGPMC_MAIN_0_SLV_ID                                    2144
#define CSL_FW_IGPMC_MAIN_0_SLV_TYPE                                  CSL_FW_SECURITY
#define CSL_FW_IGPMC_MAIN_0_SLV_MMR_BASE                              0x45218000
#define CSL_FW_IGPMC_MAIN_0_SLV_NUM_REGIONS                           8
#define CSL_FW_IGPMC_MAIN_0_SLV_NUM_PRIV_IDS_PER_REGION               3
#define CSL_FW_IGPMC_MAIN_0_SLV_NUM_PROTECTED_HW_REGIONS              2
#define CSL_FW_IGPMC_MAIN_0_SLV_DATA_START                            0x20000000
#define CSL_FW_IGPMC_MAIN_0_SLV_DATA_END                              0x5fffffff
#define CSL_FW_IGPMC_MAIN_0_SLV_CFG_START                             0x5390000
#define CSL_FW_IGPMC_MAIN_0_SLV_CFG_END                               0x53903ff

/* Properties of firewall protecting slave endpoint: IGIC500SS_MAIN_0.SLV_CFG*/
#define CSL_FW_IGIC500SS_MAIN_0_SLV_CFG_ID                            2160
#define CSL_FW_IGIC500SS_MAIN_0_SLV_CFG_TYPE                          CSL_FW_SECURITY
#define CSL_FW_IGIC500SS_MAIN_0_SLV_CFG_MMR_BASE                      0x4521c000
#define CSL_FW_IGIC500SS_MAIN_0_SLV_CFG_NUM_REGIONS                   2
#define CSL_FW_IGIC500SS_MAIN_0_SLV_CFG_NUM_PRIV_IDS_PER_REGION       3
#define CSL_FW_IGIC500SS_MAIN_0_SLV_CFG_NUM_PROTECTED_HW_REGIONS      2
#define CSL_FW_IGIC500SS_MAIN_0_SLV_CFG_GIC_REGS_START                0x1800000
#define CSL_FW_IGIC500SS_MAIN_0_SLV_CFG_GIC_REGS_END                  0x18fffff
#define CSL_FW_IGIC500SS_MAIN_0_SLV_CFG_GIC_ITS_REGS_START            0x1000000
#define CSL_FW_IGIC500SS_MAIN_0_SLV_CFG_GIC_ITS_REGS_END              0x13fffff

/* Properties of firewall protecting slave endpoint: IGIC500SS_MAIN_0.ECC_AGGR_CFG*/
#define CSL_FW_IGIC500SS_MAIN_0_ECC_AGGR_CFG_ID                       2161
#define CSL_FW_IGIC500SS_MAIN_0_ECC_AGGR_CFG_TYPE                     CSL_FW_SECURITY
#define CSL_FW_IGIC500SS_MAIN_0_ECC_AGGR_CFG_MMR_BASE                 0x4521c400
#define CSL_FW_IGIC500SS_MAIN_0_ECC_AGGR_CFG_NUM_REGIONS              1
#define CSL_FW_IGIC500SS_MAIN_0_ECC_AGGR_CFG_NUM_PRIV_IDS_PER_REGION  3
#define CSL_FW_IGIC500SS_MAIN_0_ECC_AGGR_CFG_NUM_PROTECTED_HW_REGIONS 1
#define CSL_FW_IGIC500SS_MAIN_0_ECC_AGGR_CFG_REGS_START               0x2a22000
#define CSL_FW_IGIC500SS_MAIN_0_ECC_AGGR_CFG_REGS_END                 0x2a223ff

/* Properties of firewall protecting slave endpoint: IUSB3SS2P0_GS80_MAIN_0.SLV0*/
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_0_SLV0_ID                         2176
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_0_SLV0_TYPE                       CSL_FW_SECURITY
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_0_SLV0_MMR_BASE                   0x45220000
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_0_SLV0_NUM_REGIONS                2
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_0_SLV0_NUM_PRIV_IDS_PER_REGION    3
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_0_SLV0_NUM_PROTECTED_HW_REGIONS   16
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_0_SLV0_REGS_START                 0x4000000
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_0_SLV0_REGS_END                   0x4003fff
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_0_SLV0_AHB0_START                 0x4008000
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_0_SLV0_AHB0_END                   0x400ffff
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_0_SLV0_ECC_AGGR_VBP0_START        0x2a30000
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_0_SLV0_ECC_AGGR_VBP0_END          0x2a31fff
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_0_SLV0_USB3_CORE_CAP_START        0x4010000
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_0_SLV0_USB3_CORE_CAP_END          0x401001f
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_0_SLV0_USB3_CORE_OPER_START       0x4010020
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_0_SLV0_USB3_CORE_OPER_END         0x401041f
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_0_SLV0_USB3_CORE_PORT_START       0x4010420
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_0_SLV0_USB3_CORE_PORT_END         0x401043f
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_0_SLV0_USB3_CORE_RUNTIME_START    0x4010440
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_0_SLV0_USB3_CORE_RUNTIME_END      0x401045f
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_0_SLV0_USB3_CORE_INTR_START       0x4010460
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_0_SLV0_USB3_CORE_INTR_END         0x401065f
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_0_SLV0_USB3_CORE_DB_START         0x4010660
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_0_SLV0_USB3_CORE_DB_END           0x401085f
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_0_SLV0_USB3_CORE_EXTCAP_START     0x4010a60
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_0_SLV0_USB3_CORE_EXTCAP_END       0x4010a6f
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_0_SLV0_USB3_CORE_SUPPRTCAP2_START 0x4010a70
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_0_SLV0_USB3_CORE_SUPPRTCAP2_END   0x4010a7f
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_0_SLV0_USB3_CORE_SUPPRTCAP3_START 0x4010a80
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_0_SLV0_USB3_CORE_SUPPRTCAP3_END   0x4010a8f
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_0_SLV0_USB3_CORE_GBL_START        0x401c100
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_0_SLV0_USB3_CORE_GBL_END          0x401c8ff
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_0_SLV0_USB3_CORE_DEV_START        0x401c700
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_0_SLV0_USB3_CORE_DEV_END          0x401d6ff
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_0_SLV0_USB3_CORE_OTG_START        0x401cc00
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_0_SLV0_USB3_CORE_OTG_END          0x401cc3f
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_0_SLV0_USB3_CORE_BC_START         0x401cc30
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_0_SLV0_USB3_CORE_BC_END           0x401cc3f

/* Properties of firewall protecting slave endpoint: IUSB3SS2P0_GS80_MAIN_0.SLV1*/
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_0_SLV1_ID                         2177
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_0_SLV1_TYPE                       CSL_FW_SECURITY
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_0_SLV1_MMR_BASE                   0x45220400
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_0_SLV1_NUM_REGIONS                1
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_0_SLV1_NUM_PRIV_IDS_PER_REGION    3
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_0_SLV1_NUM_PROTECTED_HW_REGIONS   1
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_0_SLV1_PHY2_START                 0x4100000
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_0_SLV1_PHY2_END                   0x410007f

/* Properties of firewall protecting slave endpoint: IUSB3SS2P0_GS80_MAIN_1.SLV0*/
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_1_SLV0_ID                         2178
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_1_SLV0_TYPE                       CSL_FW_SECURITY
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_1_SLV0_MMR_BASE                   0x45220800
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_1_SLV0_NUM_REGIONS                2
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_1_SLV0_NUM_PRIV_IDS_PER_REGION    3
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_1_SLV0_NUM_PROTECTED_HW_REGIONS   16
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_1_SLV0_REGS_START                 0x4020000
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_1_SLV0_REGS_END                   0x4023fff
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_1_SLV0_AHB0_START                 0x4028000
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_1_SLV0_AHB0_END                   0x402ffff
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_1_SLV0_ECC_AGGR_VBP0_START        0x2a32000
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_1_SLV0_ECC_AGGR_VBP0_END          0x2a33fff
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_1_SLV0_USB3_CORE_CAP_START        0x4030000
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_1_SLV0_USB3_CORE_CAP_END          0x403001f
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_1_SLV0_USB3_CORE_OPER_START       0x4030020
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_1_SLV0_USB3_CORE_OPER_END         0x403041f
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_1_SLV0_USB3_CORE_PORT_START       0x4030420
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_1_SLV0_USB3_CORE_PORT_END         0x403043f
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_1_SLV0_USB3_CORE_RUNTIME_START    0x4030440
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_1_SLV0_USB3_CORE_RUNTIME_END      0x403045f
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_1_SLV0_USB3_CORE_INTR_START       0x4030460
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_1_SLV0_USB3_CORE_INTR_END         0x403065f
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_1_SLV0_USB3_CORE_DB_START         0x4030660
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_1_SLV0_USB3_CORE_DB_END           0x403085f
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_1_SLV0_USB3_CORE_EXTCAP_START     0x4030a60
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_1_SLV0_USB3_CORE_EXTCAP_END       0x4030a6f
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_1_SLV0_USB3_CORE_SUPPRTCAP2_START 0x4030a70
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_1_SLV0_USB3_CORE_SUPPRTCAP2_END   0x4030a7f
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_1_SLV0_USB3_CORE_SUPPRTCAP3_START 0x4030a80
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_1_SLV0_USB3_CORE_SUPPRTCAP3_END   0x4030a8f
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_1_SLV0_USB3_CORE_GBL_START        0x403c100
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_1_SLV0_USB3_CORE_GBL_END          0x403c8ff
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_1_SLV0_USB3_CORE_DEV_START        0x403c700
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_1_SLV0_USB3_CORE_DEV_END          0x403d6ff
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_1_SLV0_USB3_CORE_OTG_START        0x403cc00
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_1_SLV0_USB3_CORE_OTG_END          0x403cc3f
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_1_SLV0_USB3_CORE_BC_START         0x403cc30
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_1_SLV0_USB3_CORE_BC_END           0x403cc3f

/* Properties of firewall protecting slave endpoint: IUSB3SS2P0_GS80_MAIN_1.SLV1*/
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_1_SLV1_ID                         2179
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_1_SLV1_TYPE                       CSL_FW_SECURITY
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_1_SLV1_MMR_BASE                   0x45220c00
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_1_SLV1_NUM_REGIONS                1
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_1_SLV1_NUM_PRIV_IDS_PER_REGION    3
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_1_SLV1_NUM_PROTECTED_HW_REGIONS   1
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_1_SLV1_PHY2_START                 0x4110000
#define CSL_FW_IUSB3SS2P0_GS80_MAIN_1_SLV1_PHY2_END                   0x411007f

/* Properties of firewall protecting slave endpoint: IEQEP_MAIN_0.EQEP_VBUSP*/
#define CSL_FW_IEQEP_MAIN_0_EQEP_VBUSP_ID                             2192
#define CSL_FW_IEQEP_MAIN_0_EQEP_VBUSP_TYPE                           CSL_FW_SECURITY
#define CSL_FW_IEQEP_MAIN_0_EQEP_VBUSP_MMR_BASE                       0x45224000
#define CSL_FW_IEQEP_MAIN_0_EQEP_VBUSP_NUM_REGIONS                    1
#define CSL_FW_IEQEP_MAIN_0_EQEP_VBUSP_NUM_PRIV_IDS_PER_REGION        3
#define CSL_FW_IEQEP_MAIN_0_EQEP_VBUSP_NUM_PROTECTED_HW_REGIONS       1
#define CSL_FW_IEQEP_MAIN_0_EQEP_VBUSP_REG_START                      0x3200000
#define CSL_FW_IEQEP_MAIN_0_EQEP_VBUSP_REG_END                        0x32000ff

/* Properties of firewall protecting slave endpoint: IEQEP_MAIN_1.EQEP_VBUSP*/
#define CSL_FW_IEQEP_MAIN_1_EQEP_VBUSP_ID                             2193
#define CSL_FW_IEQEP_MAIN_1_EQEP_VBUSP_TYPE                           CSL_FW_SECURITY
#define CSL_FW_IEQEP_MAIN_1_EQEP_VBUSP_MMR_BASE                       0x45224400
#define CSL_FW_IEQEP_MAIN_1_EQEP_VBUSP_NUM_REGIONS                    1
#define CSL_FW_IEQEP_MAIN_1_EQEP_VBUSP_NUM_PRIV_IDS_PER_REGION        3
#define CSL_FW_IEQEP_MAIN_1_EQEP_VBUSP_NUM_PROTECTED_HW_REGIONS       1
#define CSL_FW_IEQEP_MAIN_1_EQEP_VBUSP_REG_START                      0x3210000
#define CSL_FW_IEQEP_MAIN_1_EQEP_VBUSP_REG_END                        0x32100ff

/* Properties of firewall protecting slave endpoint: IEQEP_MAIN_2.EQEP_VBUSP*/
#define CSL_FW_IEQEP_MAIN_2_EQEP_VBUSP_ID                             2194
#define CSL_FW_IEQEP_MAIN_2_EQEP_VBUSP_TYPE                           CSL_FW_SECURITY
#define CSL_FW_IEQEP_MAIN_2_EQEP_VBUSP_MMR_BASE                       0x45224800
#define CSL_FW_IEQEP_MAIN_2_EQEP_VBUSP_NUM_REGIONS                    1
#define CSL_FW_IEQEP_MAIN_2_EQEP_VBUSP_NUM_PRIV_IDS_PER_REGION        3
#define CSL_FW_IEQEP_MAIN_2_EQEP_VBUSP_NUM_PROTECTED_HW_REGIONS       1
#define CSL_FW_IEQEP_MAIN_2_EQEP_VBUSP_REG_START                      0x3220000
#define CSL_FW_IEQEP_MAIN_2_EQEP_VBUSP_REG_END                        0x32200ff

/* Properties of firewall protecting slave endpoint: IECAP_MAIN_0.ECAP_VBUSP*/
#define CSL_FW_IECAP_MAIN_0_ECAP_VBUSP_ID                             2208
#define CSL_FW_IECAP_MAIN_0_ECAP_VBUSP_TYPE                           CSL_FW_SECURITY
#define CSL_FW_IECAP_MAIN_0_ECAP_VBUSP_MMR_BASE                       0x45228000
#define CSL_FW_IECAP_MAIN_0_ECAP_VBUSP_NUM_REGIONS                    1
#define CSL_FW_IECAP_MAIN_0_ECAP_VBUSP_NUM_PRIV_IDS_PER_REGION        3
#define CSL_FW_IECAP_MAIN_0_ECAP_VBUSP_NUM_PROTECTED_HW_REGIONS       1
#define CSL_FW_IECAP_MAIN_0_ECAP_VBUSP_CTL_STS_START                  0x3100000
#define CSL_FW_IECAP_MAIN_0_ECAP_VBUSP_CTL_STS_END                    0x31000ff

/* Properties of firewall protecting slave endpoint: IEHRPWM_MAIN_0.EPWM_VBUSP*/
#define CSL_FW_IEHRPWM_MAIN_0_EPWM_VBUSP_ID                           2224
#define CSL_FW_IEHRPWM_MAIN_0_EPWM_VBUSP_TYPE                         CSL_FW_SECURITY
#define CSL_FW_IEHRPWM_MAIN_0_EPWM_VBUSP_MMR_BASE                     0x4522c000
#define CSL_FW_IEHRPWM_MAIN_0_EPWM_VBUSP_NUM_REGIONS                  1
#define CSL_FW_IEHRPWM_MAIN_0_EPWM_VBUSP_NUM_PRIV_IDS_PER_REGION      3
#define CSL_FW_IEHRPWM_MAIN_0_EPWM_VBUSP_NUM_PROTECTED_HW_REGIONS     1
#define CSL_FW_IEHRPWM_MAIN_0_EPWM_VBUSP_EPWM_REGS_START              0x3000000
#define CSL_FW_IEHRPWM_MAIN_0_EPWM_VBUSP_EPWM_REGS_END                0x30000ff

/* Properties of firewall protecting slave endpoint: IEHRPWM_MAIN_0.HRPWM_VBUSP*/
#define CSL_FW_IEHRPWM_MAIN_0_HRPWM_VBUSP_ID                          2225
#define CSL_FW_IEHRPWM_MAIN_0_HRPWM_VBUSP_TYPE                        CSL_FW_SECURITY
#define CSL_FW_IEHRPWM_MAIN_0_HRPWM_VBUSP_MMR_BASE                    0x4522c400
#define CSL_FW_IEHRPWM_MAIN_0_HRPWM_VBUSP_NUM_REGIONS                 1
#define CSL_FW_IEHRPWM_MAIN_0_HRPWM_VBUSP_NUM_PRIV_IDS_PER_REGION     3
#define CSL_FW_IEHRPWM_MAIN_0_HRPWM_VBUSP_NUM_PROTECTED_HW_REGIONS    1
#define CSL_FW_IEHRPWM_MAIN_0_HRPWM_VBUSP_EHRPWM_REGS_START           0x3008000
#define CSL_FW_IEHRPWM_MAIN_0_HRPWM_VBUSP_EHRPWM_REGS_END             0x30080ff

/* Properties of firewall protecting slave endpoint: IEHRPWM_MAIN_5.EPWM_VBUSP*/
#define CSL_FW_IEHRPWM_MAIN_5_EPWM_VBUSP_ID                           2226
#define CSL_FW_IEHRPWM_MAIN_5_EPWM_VBUSP_TYPE                         CSL_FW_SECURITY
#define CSL_FW_IEHRPWM_MAIN_5_EPWM_VBUSP_MMR_BASE                     0x4522c800
#define CSL_FW_IEHRPWM_MAIN_5_EPWM_VBUSP_NUM_REGIONS                  1
#define CSL_FW_IEHRPWM_MAIN_5_EPWM_VBUSP_NUM_PRIV_IDS_PER_REGION      3
#define CSL_FW_IEHRPWM_MAIN_5_EPWM_VBUSP_NUM_PROTECTED_HW_REGIONS     1
#define CSL_FW_IEHRPWM_MAIN_5_EPWM_VBUSP_EPWM_REGS_START              0x3050000
#define CSL_FW_IEHRPWM_MAIN_5_EPWM_VBUSP_EPWM_REGS_END                0x30500ff

/* Properties of firewall protecting slave endpoint: IEHRPWM_MAIN_5.HRPWM_VBUSP*/
#define CSL_FW_IEHRPWM_MAIN_5_HRPWM_VBUSP_ID                          2227
#define CSL_FW_IEHRPWM_MAIN_5_HRPWM_VBUSP_TYPE                        CSL_FW_SECURITY
#define CSL_FW_IEHRPWM_MAIN_5_HRPWM_VBUSP_MMR_BASE                    0x4522cc00
#define CSL_FW_IEHRPWM_MAIN_5_HRPWM_VBUSP_NUM_REGIONS                 1
#define CSL_FW_IEHRPWM_MAIN_5_HRPWM_VBUSP_NUM_PRIV_IDS_PER_REGION     3
#define CSL_FW_IEHRPWM_MAIN_5_HRPWM_VBUSP_NUM_PROTECTED_HW_REGIONS    1
#define CSL_FW_IEHRPWM_MAIN_5_HRPWM_VBUSP_EHRPWM_REGS_START           0x3058000
#define CSL_FW_IEHRPWM_MAIN_5_HRPWM_VBUSP_EHRPWM_REGS_END             0x30580ff

/* Properties of firewall protecting slave endpoint: IEHRPWM_MAIN_4.EPWM_VBUSP*/
#define CSL_FW_IEHRPWM_MAIN_4_EPWM_VBUSP_ID                           2228
#define CSL_FW_IEHRPWM_MAIN_4_EPWM_VBUSP_TYPE                         CSL_FW_SECURITY
#define CSL_FW_IEHRPWM_MAIN_4_EPWM_VBUSP_MMR_BASE                     0x4522d000
#define CSL_FW_IEHRPWM_MAIN_4_EPWM_VBUSP_NUM_REGIONS                  1
#define CSL_FW_IEHRPWM_MAIN_4_EPWM_VBUSP_NUM_PRIV_IDS_PER_REGION      3
#define CSL_FW_IEHRPWM_MAIN_4_EPWM_VBUSP_NUM_PROTECTED_HW_REGIONS     1
#define CSL_FW_IEHRPWM_MAIN_4_EPWM_VBUSP_EPWM_REGS_START              0x3040000
#define CSL_FW_IEHRPWM_MAIN_4_EPWM_VBUSP_EPWM_REGS_END                0x30400ff

/* Properties of firewall protecting slave endpoint: IEHRPWM_MAIN_4.HRPWM_VBUSP*/
#define CSL_FW_IEHRPWM_MAIN_4_HRPWM_VBUSP_ID                          2229
#define CSL_FW_IEHRPWM_MAIN_4_HRPWM_VBUSP_TYPE                        CSL_FW_SECURITY
#define CSL_FW_IEHRPWM_MAIN_4_HRPWM_VBUSP_MMR_BASE                    0x4522d400
#define CSL_FW_IEHRPWM_MAIN_4_HRPWM_VBUSP_NUM_REGIONS                 1
#define CSL_FW_IEHRPWM_MAIN_4_HRPWM_VBUSP_NUM_PRIV_IDS_PER_REGION     3
#define CSL_FW_IEHRPWM_MAIN_4_HRPWM_VBUSP_NUM_PROTECTED_HW_REGIONS    1
#define CSL_FW_IEHRPWM_MAIN_4_HRPWM_VBUSP_EHRPWM_REGS_START           0x3048000
#define CSL_FW_IEHRPWM_MAIN_4_HRPWM_VBUSP_EHRPWM_REGS_END             0x30480ff

/* Properties of firewall protecting slave endpoint: IEHRPWM_MAIN_3.EPWM_VBUSP*/
#define CSL_FW_IEHRPWM_MAIN_3_EPWM_VBUSP_ID                           2230
#define CSL_FW_IEHRPWM_MAIN_3_EPWM_VBUSP_TYPE                         CSL_FW_SECURITY
#define CSL_FW_IEHRPWM_MAIN_3_EPWM_VBUSP_MMR_BASE                     0x4522d800
#define CSL_FW_IEHRPWM_MAIN_3_EPWM_VBUSP_NUM_REGIONS                  1
#define CSL_FW_IEHRPWM_MAIN_3_EPWM_VBUSP_NUM_PRIV_IDS_PER_REGION      3
#define CSL_FW_IEHRPWM_MAIN_3_EPWM_VBUSP_NUM_PROTECTED_HW_REGIONS     1
#define CSL_FW_IEHRPWM_MAIN_3_EPWM_VBUSP_EPWM_REGS_START              0x3030000
#define CSL_FW_IEHRPWM_MAIN_3_EPWM_VBUSP_EPWM_REGS_END                0x30300ff

/* Properties of firewall protecting slave endpoint: IEHRPWM_MAIN_3.HRPWM_VBUSP*/
#define CSL_FW_IEHRPWM_MAIN_3_HRPWM_VBUSP_ID                          2231
#define CSL_FW_IEHRPWM_MAIN_3_HRPWM_VBUSP_TYPE                        CSL_FW_SECURITY
#define CSL_FW_IEHRPWM_MAIN_3_HRPWM_VBUSP_MMR_BASE                    0x4522dc00
#define CSL_FW_IEHRPWM_MAIN_3_HRPWM_VBUSP_NUM_REGIONS                 1
#define CSL_FW_IEHRPWM_MAIN_3_HRPWM_VBUSP_NUM_PRIV_IDS_PER_REGION     3
#define CSL_FW_IEHRPWM_MAIN_3_HRPWM_VBUSP_NUM_PROTECTED_HW_REGIONS    1
#define CSL_FW_IEHRPWM_MAIN_3_HRPWM_VBUSP_EHRPWM_REGS_START           0x3038000
#define CSL_FW_IEHRPWM_MAIN_3_HRPWM_VBUSP_EHRPWM_REGS_END             0x30380ff

/* Properties of firewall protecting slave endpoint: IEHRPWM_MAIN_2.EPWM_VBUSP*/
#define CSL_FW_IEHRPWM_MAIN_2_EPWM_VBUSP_ID                           2232
#define CSL_FW_IEHRPWM_MAIN_2_EPWM_VBUSP_TYPE                         CSL_FW_SECURITY
#define CSL_FW_IEHRPWM_MAIN_2_EPWM_VBUSP_MMR_BASE                     0x4522e000
#define CSL_FW_IEHRPWM_MAIN_2_EPWM_VBUSP_NUM_REGIONS                  1
#define CSL_FW_IEHRPWM_MAIN_2_EPWM_VBUSP_NUM_PRIV_IDS_PER_REGION      3
#define CSL_FW_IEHRPWM_MAIN_2_EPWM_VBUSP_NUM_PROTECTED_HW_REGIONS     1
#define CSL_FW_IEHRPWM_MAIN_2_EPWM_VBUSP_EPWM_REGS_START              0x3020000
#define CSL_FW_IEHRPWM_MAIN_2_EPWM_VBUSP_EPWM_REGS_END                0x30200ff

/* Properties of firewall protecting slave endpoint: IEHRPWM_MAIN_2.HRPWM_VBUSP*/
#define CSL_FW_IEHRPWM_MAIN_2_HRPWM_VBUSP_ID                          2233
#define CSL_FW_IEHRPWM_MAIN_2_HRPWM_VBUSP_TYPE                        CSL_FW_SECURITY
#define CSL_FW_IEHRPWM_MAIN_2_HRPWM_VBUSP_MMR_BASE                    0x4522e400
#define CSL_FW_IEHRPWM_MAIN_2_HRPWM_VBUSP_NUM_REGIONS                 1
#define CSL_FW_IEHRPWM_MAIN_2_HRPWM_VBUSP_NUM_PRIV_IDS_PER_REGION     3
#define CSL_FW_IEHRPWM_MAIN_2_HRPWM_VBUSP_NUM_PROTECTED_HW_REGIONS    1
#define CSL_FW_IEHRPWM_MAIN_2_HRPWM_VBUSP_EHRPWM_REGS_START           0x3028000
#define CSL_FW_IEHRPWM_MAIN_2_HRPWM_VBUSP_EHRPWM_REGS_END             0x30280ff

/* Properties of firewall protecting slave endpoint: IEHRPWM_MAIN_1.EPWM_VBUSP*/
#define CSL_FW_IEHRPWM_MAIN_1_EPWM_VBUSP_ID                           2234
#define CSL_FW_IEHRPWM_MAIN_1_EPWM_VBUSP_TYPE                         CSL_FW_SECURITY
#define CSL_FW_IEHRPWM_MAIN_1_EPWM_VBUSP_MMR_BASE                     0x4522e800
#define CSL_FW_IEHRPWM_MAIN_1_EPWM_VBUSP_NUM_REGIONS                  1
#define CSL_FW_IEHRPWM_MAIN_1_EPWM_VBUSP_NUM_PRIV_IDS_PER_REGION      3
#define CSL_FW_IEHRPWM_MAIN_1_EPWM_VBUSP_NUM_PROTECTED_HW_REGIONS     1
#define CSL_FW_IEHRPWM_MAIN_1_EPWM_VBUSP_EPWM_REGS_START              0x3010000
#define CSL_FW_IEHRPWM_MAIN_1_EPWM_VBUSP_EPWM_REGS_END                0x30100ff

/* Properties of firewall protecting slave endpoint: IEHRPWM_MAIN_1.HRPWM_VBUSP*/
#define CSL_FW_IEHRPWM_MAIN_1_HRPWM_VBUSP_ID                          2235
#define CSL_FW_IEHRPWM_MAIN_1_HRPWM_VBUSP_TYPE                        CSL_FW_SECURITY
#define CSL_FW_IEHRPWM_MAIN_1_HRPWM_VBUSP_MMR_BASE                    0x4522ec00
#define CSL_FW_IEHRPWM_MAIN_1_HRPWM_VBUSP_NUM_REGIONS                 1
#define CSL_FW_IEHRPWM_MAIN_1_HRPWM_VBUSP_NUM_PRIV_IDS_PER_REGION     3
#define CSL_FW_IEHRPWM_MAIN_1_HRPWM_VBUSP_NUM_PROTECTED_HW_REGIONS    1
#define CSL_FW_IEHRPWM_MAIN_1_HRPWM_VBUSP_EHRPWM_REGS_START           0x3018000
#define CSL_FW_IEHRPWM_MAIN_1_HRPWM_VBUSP_EHRPWM_REGS_END             0x30180ff

/* Properties of firewall protecting slave endpoint: IDMTIMER_DMC1MS_MAIN_0.TIMER_CFG_VBP*/
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_0_TIMER_CFG_VBP_ID                2240
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_0_TIMER_CFG_VBP_TYPE              CSL_FW_SECURITY
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_0_TIMER_CFG_VBP_MMR_BASE          0x45230000
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_0_TIMER_CFG_VBP_NUM_REGIONS       1
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_0_TIMER_CFG_VBP_NUM_PRIV_IDS_PER_REGION     3
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_0_TIMER_CFG_VBP_NUM_PROTECTED_HW_REGIONS    1
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_0_TIMER_CFG_VBP_CFG_START         0x2400000
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_0_TIMER_CFG_VBP_CFG_END           0x24003ff

/* Properties of firewall protecting slave endpoint: IDMTIMER_DMC1MS_MAIN_1.TIMER_CFG_VBP*/
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_1_TIMER_CFG_VBP_ID                2241
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_1_TIMER_CFG_VBP_TYPE              CSL_FW_SECURITY
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_1_TIMER_CFG_VBP_MMR_BASE          0x45230400
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_1_TIMER_CFG_VBP_NUM_REGIONS       1
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_1_TIMER_CFG_VBP_NUM_PRIV_IDS_PER_REGION     3
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_1_TIMER_CFG_VBP_NUM_PROTECTED_HW_REGIONS    1
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_1_TIMER_CFG_VBP_CFG_START         0x2410000
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_1_TIMER_CFG_VBP_CFG_END           0x24103ff

/* Properties of firewall protecting slave endpoint: IDMTIMER_DMC1MS_MAIN_2.TIMER_CFG_VBP*/
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_2_TIMER_CFG_VBP_ID                2242
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_2_TIMER_CFG_VBP_TYPE              CSL_FW_SECURITY
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_2_TIMER_CFG_VBP_MMR_BASE          0x45230800
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_2_TIMER_CFG_VBP_NUM_REGIONS       1
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_2_TIMER_CFG_VBP_NUM_PRIV_IDS_PER_REGION     3
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_2_TIMER_CFG_VBP_NUM_PROTECTED_HW_REGIONS    1
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_2_TIMER_CFG_VBP_CFG_START         0x2420000
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_2_TIMER_CFG_VBP_CFG_END           0x24203ff

/* Properties of firewall protecting slave endpoint: IDMTIMER_DMC1MS_MAIN_3.TIMER_CFG_VBP*/
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_3_TIMER_CFG_VBP_ID                2243
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_3_TIMER_CFG_VBP_TYPE              CSL_FW_SECURITY
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_3_TIMER_CFG_VBP_MMR_BASE          0x45230c00
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_3_TIMER_CFG_VBP_NUM_REGIONS       1
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_3_TIMER_CFG_VBP_NUM_PRIV_IDS_PER_REGION     3
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_3_TIMER_CFG_VBP_NUM_PROTECTED_HW_REGIONS    1
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_3_TIMER_CFG_VBP_CFG_START         0x2430000
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_3_TIMER_CFG_VBP_CFG_END           0x24303ff

/* Properties of firewall protecting slave endpoint: IDMTIMER_DMC1MS_MAIN_4.TIMER_CFG_VBP*/
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_4_TIMER_CFG_VBP_ID                2244
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_4_TIMER_CFG_VBP_TYPE              CSL_FW_SECURITY
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_4_TIMER_CFG_VBP_MMR_BASE          0x45231000
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_4_TIMER_CFG_VBP_NUM_REGIONS       1
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_4_TIMER_CFG_VBP_NUM_PRIV_IDS_PER_REGION     3
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_4_TIMER_CFG_VBP_NUM_PROTECTED_HW_REGIONS    1
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_4_TIMER_CFG_VBP_CFG_START         0x2440000
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_4_TIMER_CFG_VBP_CFG_END           0x24403ff

/* Properties of firewall protecting slave endpoint: IDMTIMER_DMC1MS_MAIN_5.TIMER_CFG_VBP*/
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_5_TIMER_CFG_VBP_ID                2245
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_5_TIMER_CFG_VBP_TYPE              CSL_FW_SECURITY
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_5_TIMER_CFG_VBP_MMR_BASE          0x45231400
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_5_TIMER_CFG_VBP_NUM_REGIONS       1
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_5_TIMER_CFG_VBP_NUM_PRIV_IDS_PER_REGION     3
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_5_TIMER_CFG_VBP_NUM_PROTECTED_HW_REGIONS    1
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_5_TIMER_CFG_VBP_CFG_START         0x2450000
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_5_TIMER_CFG_VBP_CFG_END           0x24503ff

/* Properties of firewall protecting slave endpoint: IDMTIMER_DMC1MS_MAIN_6.TIMER_CFG_VBP*/
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_6_TIMER_CFG_VBP_ID                2246
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_6_TIMER_CFG_VBP_TYPE              CSL_FW_SECURITY
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_6_TIMER_CFG_VBP_MMR_BASE          0x45231800
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_6_TIMER_CFG_VBP_NUM_REGIONS       1
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_6_TIMER_CFG_VBP_NUM_PRIV_IDS_PER_REGION     3
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_6_TIMER_CFG_VBP_NUM_PROTECTED_HW_REGIONS    1
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_6_TIMER_CFG_VBP_CFG_START         0x2460000
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_6_TIMER_CFG_VBP_CFG_END           0x24603ff

/* Properties of firewall protecting slave endpoint: IDMTIMER_DMC1MS_MAIN_7.TIMER_CFG_VBP*/
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_7_TIMER_CFG_VBP_ID                2247
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_7_TIMER_CFG_VBP_TYPE              CSL_FW_SECURITY
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_7_TIMER_CFG_VBP_MMR_BASE          0x45231c00
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_7_TIMER_CFG_VBP_NUM_REGIONS       1
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_7_TIMER_CFG_VBP_NUM_PRIV_IDS_PER_REGION     3
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_7_TIMER_CFG_VBP_NUM_PROTECTED_HW_REGIONS    1
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_7_TIMER_CFG_VBP_CFG_START         0x2470000
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_7_TIMER_CFG_VBP_CFG_END           0x24703ff

/* Properties of firewall protecting slave endpoint: IDMTIMER_DMC1MS_MAIN_8.TIMER_CFG_VBP*/
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_8_TIMER_CFG_VBP_ID                2248
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_8_TIMER_CFG_VBP_TYPE              CSL_FW_SECURITY
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_8_TIMER_CFG_VBP_MMR_BASE          0x45232000
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_8_TIMER_CFG_VBP_NUM_REGIONS       1
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_8_TIMER_CFG_VBP_NUM_PRIV_IDS_PER_REGION     3
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_8_TIMER_CFG_VBP_NUM_PROTECTED_HW_REGIONS    1
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_8_TIMER_CFG_VBP_CFG_START         0x2480000
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_8_TIMER_CFG_VBP_CFG_END           0x24803ff

/* Properties of firewall protecting slave endpoint: IDMTIMER_DMC1MS_MAIN_9.TIMER_CFG_VBP*/
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_9_TIMER_CFG_VBP_ID                2249
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_9_TIMER_CFG_VBP_TYPE              CSL_FW_SECURITY
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_9_TIMER_CFG_VBP_MMR_BASE          0x45232400
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_9_TIMER_CFG_VBP_NUM_REGIONS       1
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_9_TIMER_CFG_VBP_NUM_PRIV_IDS_PER_REGION     3
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_9_TIMER_CFG_VBP_NUM_PROTECTED_HW_REGIONS    1
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_9_TIMER_CFG_VBP_CFG_START         0x2490000
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_9_TIMER_CFG_VBP_CFG_END           0x24903ff

/* Properties of firewall protecting slave endpoint: IDMTIMER_DMC1MS_MAIN_10.TIMER_CFG_VBP*/
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_10_TIMER_CFG_VBP_ID               2250
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_10_TIMER_CFG_VBP_TYPE             CSL_FW_SECURITY
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_10_TIMER_CFG_VBP_MMR_BASE         0x45232800
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_10_TIMER_CFG_VBP_NUM_REGIONS      1
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_10_TIMER_CFG_VBP_NUM_PRIV_IDS_PER_REGION    3
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_10_TIMER_CFG_VBP_NUM_PROTECTED_HW_REGIONS   1
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_10_TIMER_CFG_VBP_CFG_START        0x24a0000
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_10_TIMER_CFG_VBP_CFG_END          0x24a03ff

/* Properties of firewall protecting slave endpoint: IDMTIMER_DMC1MS_MAIN_11.TIMER_CFG_VBP*/
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_11_TIMER_CFG_VBP_ID               2251
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_11_TIMER_CFG_VBP_TYPE             CSL_FW_SECURITY
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_11_TIMER_CFG_VBP_MMR_BASE         0x45232c00
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_11_TIMER_CFG_VBP_NUM_REGIONS      1
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_11_TIMER_CFG_VBP_NUM_PRIV_IDS_PER_REGION    3
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_11_TIMER_CFG_VBP_NUM_PROTECTED_HW_REGIONS   1
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_11_TIMER_CFG_VBP_CFG_START        0x24b0000
#define CSL_FW_IDMTIMER_DMC1MS_MAIN_11_TIMER_CFG_VBP_CFG_END          0x24b03ff

/* Properties of firewall protecting slave endpoint: IRTI_MAIN_0.SLV*/
#define CSL_FW_IRTI_MAIN_0_SLV_ID                                     2272
#define CSL_FW_IRTI_MAIN_0_SLV_TYPE                                   CSL_FW_SECURITY
#define CSL_FW_IRTI_MAIN_0_SLV_MMR_BASE                               0x45238000
#define CSL_FW_IRTI_MAIN_0_SLV_NUM_REGIONS                            1
#define CSL_FW_IRTI_MAIN_0_SLV_NUM_PRIV_IDS_PER_REGION                3
#define CSL_FW_IRTI_MAIN_0_SLV_NUM_PROTECTED_HW_REGIONS               1
#define CSL_FW_IRTI_MAIN_0_SLV_CFG_START                              0x2200000
#define CSL_FW_IRTI_MAIN_0_SLV_CFG_END                                0x22000ff

/* Properties of firewall protecting slave endpoint: IRTI_MAIN_1.SLV*/
#define CSL_FW_IRTI_MAIN_1_SLV_ID                                     2273
#define CSL_FW_IRTI_MAIN_1_SLV_TYPE                                   CSL_FW_SECURITY
#define CSL_FW_IRTI_MAIN_1_SLV_MMR_BASE                               0x45238400
#define CSL_FW_IRTI_MAIN_1_SLV_NUM_REGIONS                            1
#define CSL_FW_IRTI_MAIN_1_SLV_NUM_PRIV_IDS_PER_REGION                3
#define CSL_FW_IRTI_MAIN_1_SLV_NUM_PROTECTED_HW_REGIONS               1
#define CSL_FW_IRTI_MAIN_1_SLV_CFG_START                              0x2210000
#define CSL_FW_IRTI_MAIN_1_SLV_CFG_END                                0x22100ff

/* Properties of firewall protecting slave endpoint: IRTI_MAIN_2.SLV*/
#define CSL_FW_IRTI_MAIN_2_SLV_ID                                     2274
#define CSL_FW_IRTI_MAIN_2_SLV_TYPE                                   CSL_FW_SECURITY
#define CSL_FW_IRTI_MAIN_2_SLV_MMR_BASE                               0x45238800
#define CSL_FW_IRTI_MAIN_2_SLV_NUM_REGIONS                            1
#define CSL_FW_IRTI_MAIN_2_SLV_NUM_PRIV_IDS_PER_REGION                3
#define CSL_FW_IRTI_MAIN_2_SLV_NUM_PROTECTED_HW_REGIONS               1
#define CSL_FW_IRTI_MAIN_2_SLV_CFG_START                              0x2220000
#define CSL_FW_IRTI_MAIN_2_SLV_CFG_END                                0x22200ff

/* Properties of firewall protecting slave endpoint: IRTI_MAIN_3.SLV*/
#define CSL_FW_IRTI_MAIN_3_SLV_ID                                     2275
#define CSL_FW_IRTI_MAIN_3_SLV_TYPE                                   CSL_FW_SECURITY
#define CSL_FW_IRTI_MAIN_3_SLV_MMR_BASE                               0x45238c00
#define CSL_FW_IRTI_MAIN_3_SLV_NUM_REGIONS                            1
#define CSL_FW_IRTI_MAIN_3_SLV_NUM_PRIV_IDS_PER_REGION                3
#define CSL_FW_IRTI_MAIN_3_SLV_NUM_PROTECTED_HW_REGIONS               1
#define CSL_FW_IRTI_MAIN_3_SLV_CFG_START                              0x2230000
#define CSL_FW_IRTI_MAIN_3_SLV_CFG_END                                0x22300ff

/* Properties of firewall protecting slave endpoint: IDDR39SS_GS80_MAIN_0.DDRSS_CFG*/
#define CSL_FW_IDDR39SS_GS80_MAIN_0_DDRSS_CFG_ID                      2312
#define CSL_FW_IDDR39SS_GS80_MAIN_0_DDRSS_CFG_TYPE                    CSL_FW_SECURITY
#define CSL_FW_IDDR39SS_GS80_MAIN_0_DDRSS_CFG_MMR_BASE                0x45242000
#define CSL_FW_IDDR39SS_GS80_MAIN_0_DDRSS_CFG_NUM_REGIONS             5
#define CSL_FW_IDDR39SS_GS80_MAIN_0_DDRSS_CFG_NUM_PRIV_IDS_PER_REGION 3
#define CSL_FW_IDDR39SS_GS80_MAIN_0_DDRSS_CFG_NUM_PROTECTED_HW_REGIONS          5
#define CSL_FW_IDDR39SS_GS80_MAIN_0_DDRSS_CFG_REGS__SS_CFG__SSCFG_START         0x298e000
#define CSL_FW_IDDR39SS_GS80_MAIN_0_DDRSS_CFG_REGS__SS_CFG__SSCFG_END 0x298e1ff
#define CSL_FW_IDDR39SS_GS80_MAIN_0_DDRSS_CFG_CTLPHY_WRAP__CTL_CFG__CTLCFG_START          0x2980000
#define CSL_FW_IDDR39SS_GS80_MAIN_0_DDRSS_CFG_CTLPHY_WRAP__CTL_CFG__CTLCFG_END  0x2983fff
#define CSL_FW_IDDR39SS_GS80_MAIN_0_DDRSS_CFG_CTLPHY_WRAP__PHY_CFG__PHYCFG_START          0x2988000
#define CSL_FW_IDDR39SS_GS80_MAIN_0_DDRSS_CFG_CTLPHY_WRAP__PHY_CFG__PHYCFG_END  0x2989fff
#define CSL_FW_IDDR39SS_GS80_MAIN_0_DDRSS_CFG_ECC_AGGR_CTL__CFG__REGS_START     0x2a20000
#define CSL_FW_IDDR39SS_GS80_MAIN_0_DDRSS_CFG_ECC_AGGR_CTL__CFG__REGS_END       0x2a203ff
#define CSL_FW_IDDR39SS_GS80_MAIN_0_DDRSS_CFG_ECC_AGGR_VBUS__CFG__REGS_START    0x2a21000
#define CSL_FW_IDDR39SS_GS80_MAIN_0_DDRSS_CFG_ECC_AGGR_VBUS__CFG__REGS_END      0x2a213ff

/* Properties of firewall protecting slave endpoint: IMSHSI2C_MAIN_0.VBUSP*/
#define CSL_FW_IMSHSI2C_MAIN_0_VBUSP_ID                               2320
#define CSL_FW_IMSHSI2C_MAIN_0_VBUSP_TYPE                             CSL_FW_SECURITY
#define CSL_FW_IMSHSI2C_MAIN_0_VBUSP_MMR_BASE                         0x45244000
#define CSL_FW_IMSHSI2C_MAIN_0_VBUSP_NUM_REGIONS                      1
#define CSL_FW_IMSHSI2C_MAIN_0_VBUSP_NUM_PRIV_IDS_PER_REGION          3
#define CSL_FW_IMSHSI2C_MAIN_0_VBUSP_NUM_PROTECTED_HW_REGIONS         1
#define CSL_FW_IMSHSI2C_MAIN_0_VBUSP_CFG_START                        0x2000000
#define CSL_FW_IMSHSI2C_MAIN_0_VBUSP_CFG_END                          0x20000ff

/* Properties of firewall protecting slave endpoint: IMSHSI2C_MAIN_1.VBUSP*/
#define CSL_FW_IMSHSI2C_MAIN_1_VBUSP_ID                               2321
#define CSL_FW_IMSHSI2C_MAIN_1_VBUSP_TYPE                             CSL_FW_SECURITY
#define CSL_FW_IMSHSI2C_MAIN_1_VBUSP_MMR_BASE                         0x45244400
#define CSL_FW_IMSHSI2C_MAIN_1_VBUSP_NUM_REGIONS                      1
#define CSL_FW_IMSHSI2C_MAIN_1_VBUSP_NUM_PRIV_IDS_PER_REGION          3
#define CSL_FW_IMSHSI2C_MAIN_1_VBUSP_NUM_PROTECTED_HW_REGIONS         1
#define CSL_FW_IMSHSI2C_MAIN_1_VBUSP_CFG_START                        0x2010000
#define CSL_FW_IMSHSI2C_MAIN_1_VBUSP_CFG_END                          0x20100ff

/* Properties of firewall protecting slave endpoint: IMSHSI2C_MAIN_2.VBUSP*/
#define CSL_FW_IMSHSI2C_MAIN_2_VBUSP_ID                               2322
#define CSL_FW_IMSHSI2C_MAIN_2_VBUSP_TYPE                             CSL_FW_SECURITY
#define CSL_FW_IMSHSI2C_MAIN_2_VBUSP_MMR_BASE                         0x45244800
#define CSL_FW_IMSHSI2C_MAIN_2_VBUSP_NUM_REGIONS                      1
#define CSL_FW_IMSHSI2C_MAIN_2_VBUSP_NUM_PRIV_IDS_PER_REGION          3
#define CSL_FW_IMSHSI2C_MAIN_2_VBUSP_NUM_PROTECTED_HW_REGIONS         1
#define CSL_FW_IMSHSI2C_MAIN_2_VBUSP_CFG_START                        0x2020000
#define CSL_FW_IMSHSI2C_MAIN_2_VBUSP_CFG_END                          0x20200ff

/* Properties of firewall protecting slave endpoint: IMSHSI2C_MAIN_3.VBUSP*/
#define CSL_FW_IMSHSI2C_MAIN_3_VBUSP_ID                               2323
#define CSL_FW_IMSHSI2C_MAIN_3_VBUSP_TYPE                             CSL_FW_SECURITY
#define CSL_FW_IMSHSI2C_MAIN_3_VBUSP_MMR_BASE                         0x45244c00
#define CSL_FW_IMSHSI2C_MAIN_3_VBUSP_NUM_REGIONS                      1
#define CSL_FW_IMSHSI2C_MAIN_3_VBUSP_NUM_PRIV_IDS_PER_REGION          3
#define CSL_FW_IMSHSI2C_MAIN_3_VBUSP_NUM_PROTECTED_HW_REGIONS         1
#define CSL_FW_IMSHSI2C_MAIN_3_VBUSP_CFG_START                        0x2030000
#define CSL_FW_IMSHSI2C_MAIN_3_VBUSP_CFG_END                          0x20300ff

/* Properties of firewall protecting slave endpoint: IPCIE_G3X2_MAIN_0.PCIE_SLV*/
#define CSL_FW_IPCIE_G3X2_MAIN_0_PCIE_SLV_ID                          2336
#define CSL_FW_IPCIE_G3X2_MAIN_0_PCIE_SLV_TYPE                        CSL_FW_SECURITY
#define CSL_FW_IPCIE_G3X2_MAIN_0_PCIE_SLV_MMR_BASE                    0x45248000
#define CSL_FW_IPCIE_G3X2_MAIN_0_PCIE_SLV_NUM_REGIONS                 8
#define CSL_FW_IPCIE_G3X2_MAIN_0_PCIE_SLV_NUM_PRIV_IDS_PER_REGION     3
#define CSL_FW_IPCIE_G3X2_MAIN_0_PCIE_SLV_NUM_PROTECTED_HW_REGIONS    4
#define CSL_FW_IPCIE_G3X2_MAIN_0_PCIE_SLV_CORE__CORE_DAT_SLV__PCIE_CORE_REG_START         0x5500000
#define CSL_FW_IPCIE_G3X2_MAIN_0_PCIE_SLV_CORE__CORE_DAT_SLV__PCIE_CORE_REG_END 0x55fffff
#define CSL_FW_IPCIE_G3X2_MAIN_0_PCIE_SLV_CORE__CORE_DAT_SLV__PCIE_DAT0_START   0x10000000
#define CSL_FW_IPCIE_G3X2_MAIN_0_PCIE_SLV_CORE__CORE_DAT_SLV__PCIE_DAT0_END     0x17ffffff
#define CSL_FW_IPCIE_G3X2_MAIN_0_PCIE_SLV_CORE__CORE_DAT_SLV__PCIE_DAT1_START   0x4000000000
#define CSL_FW_IPCIE_G3X2_MAIN_0_PCIE_SLV_CORE__CORE_DAT_SLV__PCIE_DAT1_END     0x40ffffffff
#define CSL_FW_IPCIE_G3X2_MAIN_0_PCIE_SLV_CORE__CORE_DAT_SLV__PCIE_DAT2_START   0xunknown
#define CSL_FW_IPCIE_G3X2_MAIN_0_PCIE_SLV_CORE__CORE_DAT_SLV__PCIE_DAT2_END     0xunknown

/* Properties of firewall protecting slave endpoint: IPCIE_G3X2_MAIN_1.PCIE_SLV*/
#define CSL_FW_IPCIE_G3X2_MAIN_1_PCIE_SLV_ID                          2337
#define CSL_FW_IPCIE_G3X2_MAIN_1_PCIE_SLV_TYPE                        CSL_FW_SECURITY
#define CSL_FW_IPCIE_G3X2_MAIN_1_PCIE_SLV_MMR_BASE                    0x45248400
#define CSL_FW_IPCIE_G3X2_MAIN_1_PCIE_SLV_NUM_REGIONS                 8
#define CSL_FW_IPCIE_G3X2_MAIN_1_PCIE_SLV_NUM_PRIV_IDS_PER_REGION     3
#define CSL_FW_IPCIE_G3X2_MAIN_1_PCIE_SLV_NUM_PROTECTED_HW_REGIONS    4
#define CSL_FW_IPCIE_G3X2_MAIN_1_PCIE_SLV_CORE__CORE_DAT_SLV__PCIE_CORE_REG_START         0x5600000
#define CSL_FW_IPCIE_G3X2_MAIN_1_PCIE_SLV_CORE__CORE_DAT_SLV__PCIE_CORE_REG_END 0x56fffff
#define CSL_FW_IPCIE_G3X2_MAIN_1_PCIE_SLV_CORE__CORE_DAT_SLV__PCIE_DAT0_START   0x18000000
#define CSL_FW_IPCIE_G3X2_MAIN_1_PCIE_SLV_CORE__CORE_DAT_SLV__PCIE_DAT0_END     0x1fffffff
#define CSL_FW_IPCIE_G3X2_MAIN_1_PCIE_SLV_CORE__CORE_DAT_SLV__PCIE_DAT1_START   0x4100000000
#define CSL_FW_IPCIE_G3X2_MAIN_1_PCIE_SLV_CORE__CORE_DAT_SLV__PCIE_DAT1_END     0x41ffffffff
#define CSL_FW_IPCIE_G3X2_MAIN_1_PCIE_SLV_CORE__CORE_DAT_SLV__PCIE_DAT2_START   0xunknown
#define CSL_FW_IPCIE_G3X2_MAIN_1_PCIE_SLV_CORE__CORE_DAT_SLV__PCIE_DAT2_END     0xunknown

/* Properties of firewall protecting slave endpoint: ICXSTM500SS_MAIN_0.VBUSM*/
#define CSL_FW_ICXSTM500SS_MAIN_0_VBUSM_ID                            2352
#define CSL_FW_ICXSTM500SS_MAIN_0_VBUSM_TYPE                          CSL_FW_SECURITY
#define CSL_FW_ICXSTM500SS_MAIN_0_VBUSM_MMR_BASE                      0x4524c000
#define CSL_FW_ICXSTM500SS_MAIN_0_VBUSM_NUM_REGIONS                   2
#define CSL_FW_ICXSTM500SS_MAIN_0_VBUSM_NUM_PRIV_IDS_PER_REGION       3
#define CSL_FW_ICXSTM500SS_MAIN_0_VBUSM_NUM_PROTECTED_HW_REGIONS      1
#define CSL_FW_ICXSTM500SS_MAIN_0_VBUSM_STIMULUS_START                0x9000000
#define CSL_FW_ICXSTM500SS_MAIN_0_VBUSM_STIMULUS_END                  0x9ffffff

/* Properties of firewall protecting slave endpoint: IDEBUGSS_K3_WRAP_CV0_MAIN_0.VBUSP_CFG*/
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_ID               2384
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_TYPE             CSL_FW_SECURITY
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_MMR_BASE         0x45254000
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_NUM_REGIONS      1
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_NUM_PRIV_IDS_PER_REGION    3
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_NUM_PROTECTED_HW_REGIONS   56
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_ROM_TABLE_0_0_START        0x4c00000000
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_ROM_TABLE_0_0_END          0x4c00000fff
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_RESV0_0_START    0x4c00001000
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_RESV0_0_END      0x4c00001fff
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_CFGAP_CFG_0_START          0x4c00002000
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_CFGAP_CFG_0_END  0x4c000020ff
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_APBAP_CFG_0_START          0x4c00002100
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_APBAP_CFG_0_END  0x4c000021ff
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_AXIAP_CFG_0_START          0x4c00002200
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_AXIAP_CFG_0_END  0x4c000022ff
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_PWRAP_CFG_0_START          0x4c00002300
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_PWRAP_CFG_0_END  0x4c000023ff
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_PVIEW_CFG_0_START          0x4c00002400
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_PVIEW_CFG_0_END  0x4c000024ff
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_JTAGAP_CFG_0_START         0x4c00002500
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_JTAGAP_CFG_0_END 0x4c000025ff
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_SECAP_CFG_0_START          0x4c00002600
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_SECAP_CFG_0_END  0x4c000026ff
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_CORTEX0_CFG_0_START        0x4c00002700
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_CORTEX0_CFG_0_END          0x4c000027ff
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_CORTEX1_CFG_0_START        0x4c00002800
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_CORTEX1_CFG_0_END          0x4c000028ff
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_CORTEX2_CFG_0_START        0x4c00002900
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_CORTEX2_CFG_0_END          0x4c000029ff
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_CORTEX3_CFG_0_START        0x4c00002a00
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_CORTEX3_CFG_0_END          0x4c00002aff
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_CORTEX4_CFG_0_START        0x4c00002b00
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_CORTEX4_CFG_0_END          0x4c00002bff
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_CORTEX5_CFG_0_START        0x4c00002c00
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_CORTEX5_CFG_0_END          0x4c00002cff
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_CORTEX6_CFG_0_START        0x4c00002d00
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_CORTEX6_CFG_0_END          0x4c00002dff
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_CORTEX7_CFG_0_START        0x4c00002e00
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_CORTEX7_CFG_0_END          0x4c00002eff
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_CORTEX8_CFG_0_START        0x4c00002f00
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_CORTEX8_CFG_0_END          0x4c00002fff
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_RESV1_0_START    0x4c00003000
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_RESV1_0_END      0x4c00003fff
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_RESV2_0_START    0x4c00004000
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_RESV2_0_END      0x4c02003fff
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_ROM_TABLE_1_0_START        0x4c20000000
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_ROM_TABLE_1_0_END          0x4c20000fff
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_CSCTI_CFG_0_START          0x4c20001000
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_CSCTI_CFG_0_END  0x4c20001fff
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_DRM_CFG_0_START  0x4c20002000
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_DRM_CFG_0_END    0x4c20002fff
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_RESV3_0_START    0x4c20003000
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_RESV3_0_END      0x4c20003fff
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_CSTPIU_CFG_0_START         0x4c20004000
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_CSTPIU_CFG_0_END 0x4c20004fff
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_CTF_CFG_0_START  0x4c20005000
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_CTF_CFG_0_END    0x4c20005fff
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_RESV4_0_START    0x4c20006000
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_RESV4_0_END      0x4c21005fff
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_EXT_APB_0_START  0x4c30000000
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_EXT_APB_0_END    0x4c3fffffff
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_ROM_TABLE_0_1_START        0x4c40000000
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_ROM_TABLE_0_1_END          0x4c40000fff
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_RESV0_1_START    0x4c40001000
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_RESV0_1_END      0x4c40001fff
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_CFGAP_CFG_1_START          0x4c40002000
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_CFGAP_CFG_1_END  0x4c400020ff
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_APBAP_CFG_1_START          0x4c40002100
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_APBAP_CFG_1_END  0x4c400021ff
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_AXIAP_CFG_1_START          0x4c40002200
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_AXIAP_CFG_1_END  0x4c400022ff
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_PWRAP_CFG_1_START          0x4c40002300
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_PWRAP_CFG_1_END  0x4c400023ff
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_PVIEW_CFG_1_START          0x4c40002400
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_PVIEW_CFG_1_END  0x4c400024ff
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_JTAGAP_CFG_1_START         0x4c40002500
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_JTAGAP_CFG_1_END 0x4c400025ff
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_SECAP_CFG_1_START          0x4c40002600
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_SECAP_CFG_1_END  0x4c400026ff
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_CORTEX0_CFG_1_START        0x4c40002700
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_CORTEX0_CFG_1_END          0x4c400027ff
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_CORTEX1_CFG_1_START        0x4c40002800
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_CORTEX1_CFG_1_END          0x4c400028ff
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_CORTEX2_CFG_1_START        0x4c40002900
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_CORTEX2_CFG_1_END          0x4c400029ff
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_CORTEX3_CFG_1_START        0x4c40002a00
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_CORTEX3_CFG_1_END          0x4c40002aff
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_CORTEX4_CFG_1_START        0x4c40002b00
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_CORTEX4_CFG_1_END          0x4c40002bff
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_CORTEX5_CFG_1_START        0x4c40002c00
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_CORTEX5_CFG_1_END          0x4c40002cff
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_CORTEX6_CFG_1_START        0x4c40002d00
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_CORTEX6_CFG_1_END          0x4c40002dff
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_CORTEX7_CFG_1_START        0x4c40002e00
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_CORTEX7_CFG_1_END          0x4c40002eff
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_CORTEX8_CFG_1_START        0x4c40002f00
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_CORTEX8_CFG_1_END          0x4c40002fff
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_RESV1_1_START    0x4c40003000
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_RESV1_1_END      0x4c40003fff
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_RESV2_1_START    0x4c40004000
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_RESV2_1_END      0x4c42003fff
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_ROM_TABLE_1_1_START        0x4c60000000
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_ROM_TABLE_1_1_END          0x4c60000fff
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_CSCTI_CFG_1_START          0x4c60001000
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_CSCTI_CFG_1_END  0x4c60001fff
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_DRM_CFG_1_START  0x4c60002000
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_DRM_CFG_1_END    0x4c60002fff
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_RESV3_1_START    0x4c60003000
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_RESV3_1_END      0x4c60003fff
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_CSTPIU_CFG_1_START         0x4c60004000
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_CSTPIU_CFG_1_END 0x4c60004fff
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_CTF_CFG_1_START  0x4c60005000
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_CTF_CFG_1_END    0x4c60005fff
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_RESV4_1_START    0x4c60006000
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_RESV4_1_END      0x4c61005fff
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_EXT_APB_1_START  0x4c70000000
#define CSL_FW_IDEBUGSS_K3_WRAP_CV0_MAIN_0_VBUSP_CFG_EXT_APB_1_END    0x4c7fffffff

/* Properties of firewall protecting slave endpoint: ISPI_MAIN_0.SLV*/
#define CSL_FW_ISPI_MAIN_0_SLV_ID                                     2416
#define CSL_FW_ISPI_MAIN_0_SLV_TYPE                                   CSL_FW_SECURITY
#define CSL_FW_ISPI_MAIN_0_SLV_MMR_BASE                               0x4525c000
#define CSL_FW_ISPI_MAIN_0_SLV_NUM_REGIONS                            1
#define CSL_FW_ISPI_MAIN_0_SLV_NUM_PRIV_IDS_PER_REGION                3
#define CSL_FW_ISPI_MAIN_0_SLV_NUM_PROTECTED_HW_REGIONS               1
#define CSL_FW_ISPI_MAIN_0_SLV_CFG_START                              0x2100000
#define CSL_FW_ISPI_MAIN_0_SLV_CFG_END                                0x21003ff

/* Properties of firewall protecting slave endpoint: ISPI_MAIN_1.SLV*/
#define CSL_FW_ISPI_MAIN_1_SLV_ID                                     2417
#define CSL_FW_ISPI_MAIN_1_SLV_TYPE                                   CSL_FW_SECURITY
#define CSL_FW_ISPI_MAIN_1_SLV_MMR_BASE                               0x4525c400
#define CSL_FW_ISPI_MAIN_1_SLV_NUM_REGIONS                            1
#define CSL_FW_ISPI_MAIN_1_SLV_NUM_PRIV_IDS_PER_REGION                3
#define CSL_FW_ISPI_MAIN_1_SLV_NUM_PROTECTED_HW_REGIONS               1
#define CSL_FW_ISPI_MAIN_1_SLV_CFG_START                              0x2110000
#define CSL_FW_ISPI_MAIN_1_SLV_CFG_END                                0x21103ff

/* Properties of firewall protecting slave endpoint: ISPI_MAIN_2.SLV*/
#define CSL_FW_ISPI_MAIN_2_SLV_ID                                     2418
#define CSL_FW_ISPI_MAIN_2_SLV_TYPE                                   CSL_FW_SECURITY
#define CSL_FW_ISPI_MAIN_2_SLV_MMR_BASE                               0x4525c800
#define CSL_FW_ISPI_MAIN_2_SLV_NUM_REGIONS                            1
#define CSL_FW_ISPI_MAIN_2_SLV_NUM_PRIV_IDS_PER_REGION                3
#define CSL_FW_ISPI_MAIN_2_SLV_NUM_PROTECTED_HW_REGIONS               1
#define CSL_FW_ISPI_MAIN_2_SLV_CFG_START                              0x2120000
#define CSL_FW_ISPI_MAIN_2_SLV_CFG_END                                0x21203ff

/* Properties of firewall protecting slave endpoint: ISPI_MAIN_3.SLV*/
#define CSL_FW_ISPI_MAIN_3_SLV_ID                                     2419
#define CSL_FW_ISPI_MAIN_3_SLV_TYPE                                   CSL_FW_SECURITY
#define CSL_FW_ISPI_MAIN_3_SLV_MMR_BASE                               0x4525cc00
#define CSL_FW_ISPI_MAIN_3_SLV_NUM_REGIONS                            1
#define CSL_FW_ISPI_MAIN_3_SLV_NUM_PRIV_IDS_PER_REGION                3
#define CSL_FW_ISPI_MAIN_3_SLV_NUM_PROTECTED_HW_REGIONS               1
#define CSL_FW_ISPI_MAIN_3_SLV_CFG_START                              0x2130000
#define CSL_FW_ISPI_MAIN_3_SLV_CFG_END                                0x21303ff

/* Properties of firewall protecting slave endpoint: ISPI_MAIN_4.SLV*/
#define CSL_FW_ISPI_MAIN_4_SLV_ID                                     2420
#define CSL_FW_ISPI_MAIN_4_SLV_TYPE                                   CSL_FW_SECURITY
#define CSL_FW_ISPI_MAIN_4_SLV_MMR_BASE                               0x4525d000
#define CSL_FW_ISPI_MAIN_4_SLV_NUM_REGIONS                            1
#define CSL_FW_ISPI_MAIN_4_SLV_NUM_PRIV_IDS_PER_REGION                3
#define CSL_FW_ISPI_MAIN_4_SLV_NUM_PROTECTED_HW_REGIONS               1
#define CSL_FW_ISPI_MAIN_4_SLV_CFG_START                              0x2140000
#define CSL_FW_ISPI_MAIN_4_SLV_CFG_END                                0x21403ff

/* Properties of firewall protecting slave endpoint: IUSART_MAIN_0.VBUSP_CBA*/
#define CSL_FW_IUSART_MAIN_0_VBUSP_CBA_ID                             2432
#define CSL_FW_IUSART_MAIN_0_VBUSP_CBA_TYPE                           CSL_FW_SECURITY
#define CSL_FW_IUSART_MAIN_0_VBUSP_CBA_MMR_BASE                       0x45260000
#define CSL_FW_IUSART_MAIN_0_VBUSP_CBA_NUM_REGIONS                    1
#define CSL_FW_IUSART_MAIN_0_VBUSP_CBA_NUM_PRIV_IDS_PER_REGION        3
#define CSL_FW_IUSART_MAIN_0_VBUSP_CBA_NUM_PROTECTED_HW_REGIONS       1
#define CSL_FW_IUSART_MAIN_0_VBUSP_CBA_MEM_START                      0x2800000
#define CSL_FW_IUSART_MAIN_0_VBUSP_CBA_MEM_END                        0x28001ff

/* Properties of firewall protecting slave endpoint: IUSART_MAIN_1.VBUSP_CBA*/
#define CSL_FW_IUSART_MAIN_1_VBUSP_CBA_ID                             2433
#define CSL_FW_IUSART_MAIN_1_VBUSP_CBA_TYPE                           CSL_FW_SECURITY
#define CSL_FW_IUSART_MAIN_1_VBUSP_CBA_MMR_BASE                       0x45260400
#define CSL_FW_IUSART_MAIN_1_VBUSP_CBA_NUM_REGIONS                    1
#define CSL_FW_IUSART_MAIN_1_VBUSP_CBA_NUM_PRIV_IDS_PER_REGION        3
#define CSL_FW_IUSART_MAIN_1_VBUSP_CBA_NUM_PROTECTED_HW_REGIONS       1
#define CSL_FW_IUSART_MAIN_1_VBUSP_CBA_MEM_START                      0x2810000
#define CSL_FW_IUSART_MAIN_1_VBUSP_CBA_MEM_END                        0x28101ff

/* Properties of firewall protecting slave endpoint: IUSART_MAIN_2.VBUSP_CBA*/
#define CSL_FW_IUSART_MAIN_2_VBUSP_CBA_ID                             2434
#define CSL_FW_IUSART_MAIN_2_VBUSP_CBA_TYPE                           CSL_FW_SECURITY
#define CSL_FW_IUSART_MAIN_2_VBUSP_CBA_MMR_BASE                       0x45260800
#define CSL_FW_IUSART_MAIN_2_VBUSP_CBA_NUM_REGIONS                    1
#define CSL_FW_IUSART_MAIN_2_VBUSP_CBA_NUM_PRIV_IDS_PER_REGION        3
#define CSL_FW_IUSART_MAIN_2_VBUSP_CBA_NUM_PROTECTED_HW_REGIONS       1
#define CSL_FW_IUSART_MAIN_2_VBUSP_CBA_MEM_START                      0x2820000
#define CSL_FW_IUSART_MAIN_2_VBUSP_CBA_MEM_END                        0x28201ff

/* Properties of firewall protecting slave endpoint: IMCASP_MAIN_0.SLV_CFG*/
#define CSL_FW_IMCASP_MAIN_0_SLV_CFG_ID                               2464
#define CSL_FW_IMCASP_MAIN_0_SLV_CFG_TYPE                             CSL_FW_SECURITY
#define CSL_FW_IMCASP_MAIN_0_SLV_CFG_MMR_BASE                         0x45268000
#define CSL_FW_IMCASP_MAIN_0_SLV_CFG_NUM_REGIONS                      1
#define CSL_FW_IMCASP_MAIN_0_SLV_CFG_NUM_PRIV_IDS_PER_REGION          3
#define CSL_FW_IMCASP_MAIN_0_SLV_CFG_NUM_PROTECTED_HW_REGIONS         1
#define CSL_FW_IMCASP_MAIN_0_SLV_CFG_CFG_START                        0x2b00000
#define CSL_FW_IMCASP_MAIN_0_SLV_CFG_CFG_END                          0x2b01fff

/* Properties of firewall protecting slave endpoint: IMCASP_MAIN_0.SLV_DMA*/
#define CSL_FW_IMCASP_MAIN_0_SLV_DMA_ID                               2465
#define CSL_FW_IMCASP_MAIN_0_SLV_DMA_TYPE                             CSL_FW_SECURITY
#define CSL_FW_IMCASP_MAIN_0_SLV_DMA_MMR_BASE                         0x45268400
#define CSL_FW_IMCASP_MAIN_0_SLV_DMA_NUM_REGIONS                      1
#define CSL_FW_IMCASP_MAIN_0_SLV_DMA_NUM_PRIV_IDS_PER_REGION          3
#define CSL_FW_IMCASP_MAIN_0_SLV_DMA_NUM_PROTECTED_HW_REGIONS         1
#define CSL_FW_IMCASP_MAIN_0_SLV_DMA_DMA_START                        0x2b08000
#define CSL_FW_IMCASP_MAIN_0_SLV_DMA_DMA_END                          0x2b083ff

/* Properties of firewall protecting slave endpoint: IMCASP_MAIN_1.SLV_CFG*/
#define CSL_FW_IMCASP_MAIN_1_SLV_CFG_ID                               2466
#define CSL_FW_IMCASP_MAIN_1_SLV_CFG_TYPE                             CSL_FW_SECURITY
#define CSL_FW_IMCASP_MAIN_1_SLV_CFG_MMR_BASE                         0x45268800
#define CSL_FW_IMCASP_MAIN_1_SLV_CFG_NUM_REGIONS                      1
#define CSL_FW_IMCASP_MAIN_1_SLV_CFG_NUM_PRIV_IDS_PER_REGION          3
#define CSL_FW_IMCASP_MAIN_1_SLV_CFG_NUM_PROTECTED_HW_REGIONS         1
#define CSL_FW_IMCASP_MAIN_1_SLV_CFG_CFG_START                        0x2b10000
#define CSL_FW_IMCASP_MAIN_1_SLV_CFG_CFG_END                          0x2b11fff

/* Properties of firewall protecting slave endpoint: IMCASP_MAIN_1.SLV_DMA*/
#define CSL_FW_IMCASP_MAIN_1_SLV_DMA_ID                               2467
#define CSL_FW_IMCASP_MAIN_1_SLV_DMA_TYPE                             CSL_FW_SECURITY
#define CSL_FW_IMCASP_MAIN_1_SLV_DMA_MMR_BASE                         0x45268c00
#define CSL_FW_IMCASP_MAIN_1_SLV_DMA_NUM_REGIONS                      1
#define CSL_FW_IMCASP_MAIN_1_SLV_DMA_NUM_PRIV_IDS_PER_REGION          3
#define CSL_FW_IMCASP_MAIN_1_SLV_DMA_NUM_PROTECTED_HW_REGIONS         1
#define CSL_FW_IMCASP_MAIN_1_SLV_DMA_DMA_START                        0x2b18000
#define CSL_FW_IMCASP_MAIN_1_SLV_DMA_DMA_END                          0x2b183ff

/* Properties of firewall protecting slave endpoint: IMCASP_MAIN_2.SLV_CFG*/
#define CSL_FW_IMCASP_MAIN_2_SLV_CFG_ID                               2468
#define CSL_FW_IMCASP_MAIN_2_SLV_CFG_TYPE                             CSL_FW_SECURITY
#define CSL_FW_IMCASP_MAIN_2_SLV_CFG_MMR_BASE                         0x45269000
#define CSL_FW_IMCASP_MAIN_2_SLV_CFG_NUM_REGIONS                      1
#define CSL_FW_IMCASP_MAIN_2_SLV_CFG_NUM_PRIV_IDS_PER_REGION          3
#define CSL_FW_IMCASP_MAIN_2_SLV_CFG_NUM_PROTECTED_HW_REGIONS         1
#define CSL_FW_IMCASP_MAIN_2_SLV_CFG_CFG_START                        0x2b20000
#define CSL_FW_IMCASP_MAIN_2_SLV_CFG_CFG_END                          0x2b21fff

/* Properties of firewall protecting slave endpoint: IMCASP_MAIN_2.SLV_DMA*/
#define CSL_FW_IMCASP_MAIN_2_SLV_DMA_ID                               2469
#define CSL_FW_IMCASP_MAIN_2_SLV_DMA_TYPE                             CSL_FW_SECURITY
#define CSL_FW_IMCASP_MAIN_2_SLV_DMA_MMR_BASE                         0x45269400
#define CSL_FW_IMCASP_MAIN_2_SLV_DMA_NUM_REGIONS                      1
#define CSL_FW_IMCASP_MAIN_2_SLV_DMA_NUM_PRIV_IDS_PER_REGION          3
#define CSL_FW_IMCASP_MAIN_2_SLV_DMA_NUM_PROTECTED_HW_REGIONS         1
#define CSL_FW_IMCASP_MAIN_2_SLV_DMA_DMA_START                        0x2b28000
#define CSL_FW_IMCASP_MAIN_2_SLV_DMA_DMA_END                          0x2b283ff

/* Properties of firewall protecting slave endpoint: IK3_CC_DEBUG_CELL_MAIN_0.SYS_VBUSP*/
#define CSL_FW_IK3_CC_DEBUG_CELL_MAIN_0_SYS_VBUSP_ID                  2480
#define CSL_FW_IK3_CC_DEBUG_CELL_MAIN_0_SYS_VBUSP_TYPE                CSL_FW_SECURITY
#define CSL_FW_IK3_CC_DEBUG_CELL_MAIN_0_SYS_VBUSP_MMR_BASE            0x4526c000
#define CSL_FW_IK3_CC_DEBUG_CELL_MAIN_0_SYS_VBUSP_NUM_REGIONS         1
#define CSL_FW_IK3_CC_DEBUG_CELL_MAIN_0_SYS_VBUSP_NUM_PRIV_IDS_PER_REGION       3
#define CSL_FW_IK3_CC_DEBUG_CELL_MAIN_0_SYS_VBUSP_NUM_PROTECTED_HW_REGIONS      1
#define CSL_FW_IK3_CC_DEBUG_CELL_MAIN_0_SYS_VBUSP_SYS_REGS_START      0x8008000
#define CSL_FW_IK3_CC_DEBUG_CELL_MAIN_0_SYS_VBUSP_SYS_REGS_END        0x8008fff

/* Properties of firewall protecting slave endpoint: IK3_MAIN_DEBUG_CELL_MAIN_0.SYS_VBUSP*/
#define CSL_FW_IK3_MAIN_DEBUG_CELL_MAIN_0_SYS_VBUSP_ID                2481
#define CSL_FW_IK3_MAIN_DEBUG_CELL_MAIN_0_SYS_VBUSP_TYPE              CSL_FW_SECURITY
#define CSL_FW_IK3_MAIN_DEBUG_CELL_MAIN_0_SYS_VBUSP_MMR_BASE          0x4526c400
#define CSL_FW_IK3_MAIN_DEBUG_CELL_MAIN_0_SYS_VBUSP_NUM_REGIONS       1
#define CSL_FW_IK3_MAIN_DEBUG_CELL_MAIN_0_SYS_VBUSP_NUM_PRIV_IDS_PER_REGION     3
#define CSL_FW_IK3_MAIN_DEBUG_CELL_MAIN_0_SYS_VBUSP_NUM_PROTECTED_HW_REGIONS    1
#define CSL_FW_IK3_MAIN_DEBUG_CELL_MAIN_0_SYS_VBUSP_SYS_REGS_START    0x8004000
#define CSL_FW_IK3_MAIN_DEBUG_CELL_MAIN_0_SYS_VBUSP_SYS_REGS_END      0x8004fff

/* Properties of firewall protecting slave endpoint: IK3_MCU_DEBUG_CELL_MAIN_0.SYS_VBUSP*/
#define CSL_FW_IK3_MCU_DEBUG_CELL_MAIN_0_SYS_VBUSP_ID                 2482
#define CSL_FW_IK3_MCU_DEBUG_CELL_MAIN_0_SYS_VBUSP_TYPE               CSL_FW_SECURITY
#define CSL_FW_IK3_MCU_DEBUG_CELL_MAIN_0_SYS_VBUSP_MMR_BASE           0x4526c800
#define CSL_FW_IK3_MCU_DEBUG_CELL_MAIN_0_SYS_VBUSP_NUM_REGIONS        1
#define CSL_FW_IK3_MCU_DEBUG_CELL_MAIN_0_SYS_VBUSP_NUM_PRIV_IDS_PER_REGION      3
#define CSL_FW_IK3_MCU_DEBUG_CELL_MAIN_0_SYS_VBUSP_NUM_PROTECTED_HW_REGIONS     1
#define CSL_FW_IK3_MCU_DEBUG_CELL_MAIN_0_SYS_VBUSP_SYS_REGS_START     0x8000000
#define CSL_FW_IK3_MCU_DEBUG_CELL_MAIN_0_SYS_VBUSP_SYS_REGS_END       0x8000fff

/* Properties of firewall protecting slave endpoint: IK3_DSS_UL_MAIN_0.VBUSP_CFG*/
#define CSL_FW_IK3_DSS_UL_MAIN_0_VBUSP_CFG_ID                         2608
#define CSL_FW_IK3_DSS_UL_MAIN_0_VBUSP_CFG_TYPE                       CSL_FW_SECURITY
#define CSL_FW_IK3_DSS_UL_MAIN_0_VBUSP_CFG_MMR_BASE                   0x4528c000
#define CSL_FW_IK3_DSS_UL_MAIN_0_VBUSP_CFG_NUM_REGIONS                8
#define CSL_FW_IK3_DSS_UL_MAIN_0_VBUSP_CFG_NUM_PRIV_IDS_PER_REGION    3
#define CSL_FW_IK3_DSS_UL_MAIN_0_VBUSP_CFG_NUM_PROTECTED_HW_REGIONS   8
#define CSL_FW_IK3_DSS_UL_MAIN_0_VBUSP_CFG_COMMON_START               0x4a00000
#define CSL_FW_IK3_DSS_UL_MAIN_0_VBUSP_CFG_COMMON_END                 0x4a00fff
#define CSL_FW_IK3_DSS_UL_MAIN_0_VBUSP_CFG_COMMON1_START              0x4a01000
#define CSL_FW_IK3_DSS_UL_MAIN_0_VBUSP_CFG_COMMON1_END                0x4a01fff
#define CSL_FW_IK3_DSS_UL_MAIN_0_VBUSP_CFG_VIDL1_START                0x4a02000
#define CSL_FW_IK3_DSS_UL_MAIN_0_VBUSP_CFG_VIDL1_END                  0x4a02fff
#define CSL_FW_IK3_DSS_UL_MAIN_0_VBUSP_CFG_VID_START                  0x4a06000
#define CSL_FW_IK3_DSS_UL_MAIN_0_VBUSP_CFG_VID_END                    0x4a06fff
#define CSL_FW_IK3_DSS_UL_MAIN_0_VBUSP_CFG_OVR1_START                 0x4a07000
#define CSL_FW_IK3_DSS_UL_MAIN_0_VBUSP_CFG_OVR1_END                   0x4a07fff
#define CSL_FW_IK3_DSS_UL_MAIN_0_VBUSP_CFG_OVR2_START                 0x4a08000
#define CSL_FW_IK3_DSS_UL_MAIN_0_VBUSP_CFG_OVR2_END                   0x4a08fff
#define CSL_FW_IK3_DSS_UL_MAIN_0_VBUSP_CFG_VP1_START                  0x4a0a000
#define CSL_FW_IK3_DSS_UL_MAIN_0_VBUSP_CFG_VP1_END                    0x4a0afff
#define CSL_FW_IK3_DSS_UL_MAIN_0_VBUSP_CFG_VP2_START                  0x4a0b000
#define CSL_FW_IK3_DSS_UL_MAIN_0_VBUSP_CFG_VP2_END                    0x4a0bfff

/* Properties of firewall protecting slave endpoint: IPCIE_G3X2_MAIN_0.PCIE_CFG*/
#define CSL_FW_IPCIE_G3X2_MAIN_0_PCIE_CFG_ID                          2688
#define CSL_FW_IPCIE_G3X2_MAIN_0_PCIE_CFG_TYPE                        CSL_FW_SECURITY
#define CSL_FW_IPCIE_G3X2_MAIN_0_PCIE_CFG_MMR_BASE                    0x452a0000
#define CSL_FW_IPCIE_G3X2_MAIN_0_PCIE_CFG_NUM_REGIONS                 5
#define CSL_FW_IPCIE_G3X2_MAIN_0_PCIE_CFG_NUM_PRIV_IDS_PER_REGION     3
#define CSL_FW_IPCIE_G3X2_MAIN_0_PCIE_CFG_NUM_PROTECTED_HW_REGIONS    5
#define CSL_FW_IPCIE_G3X2_MAIN_0_PCIE_CFG_CORE__VMAP_HP__MMRS_START   0x2908000
#define CSL_FW_IPCIE_G3X2_MAIN_0_PCIE_CFG_CORE__VMAP_HP__MMRS_END     0x29083ff
#define CSL_FW_IPCIE_G3X2_MAIN_0_PCIE_CFG_CORE__VMAP_LP__MMRS_START   0x2900000
#define CSL_FW_IPCIE_G3X2_MAIN_0_PCIE_CFG_CORE__VMAP_LP__MMRS_END     0x29003ff
#define CSL_FW_IPCIE_G3X2_MAIN_0_PCIE_CFG_CORE__CPTS__CPTS_VBUSP_START          0x2940000
#define CSL_FW_IPCIE_G3X2_MAIN_0_PCIE_CFG_CORE__CPTS__CPTS_VBUSP_END  0x29403ff
#define CSL_FW_IPCIE_G3X2_MAIN_0_PCIE_CFG_CORE__ECC_AGGR0__REGS_START 0x2a28000
#define CSL_FW_IPCIE_G3X2_MAIN_0_PCIE_CFG_CORE__ECC_AGGR0__REGS_END   0x2a283ff
#define CSL_FW_IPCIE_G3X2_MAIN_0_PCIE_CFG_CORE__ECC_AGGR1__REGS_START 0x2a29000
#define CSL_FW_IPCIE_G3X2_MAIN_0_PCIE_CFG_CORE__ECC_AGGR1__REGS_END   0x2a293ff

/* Properties of firewall protecting slave endpoint: IPCIE_G3X2_MAIN_1.PCIE_CFG*/
#define CSL_FW_IPCIE_G3X2_MAIN_1_PCIE_CFG_ID                          2689
#define CSL_FW_IPCIE_G3X2_MAIN_1_PCIE_CFG_TYPE                        CSL_FW_SECURITY
#define CSL_FW_IPCIE_G3X2_MAIN_1_PCIE_CFG_MMR_BASE                    0x452a0400
#define CSL_FW_IPCIE_G3X2_MAIN_1_PCIE_CFG_NUM_REGIONS                 5
#define CSL_FW_IPCIE_G3X2_MAIN_1_PCIE_CFG_NUM_PRIV_IDS_PER_REGION     3
#define CSL_FW_IPCIE_G3X2_MAIN_1_PCIE_CFG_NUM_PROTECTED_HW_REGIONS    5
#define CSL_FW_IPCIE_G3X2_MAIN_1_PCIE_CFG_CORE__VMAP_HP__MMRS_START   0x2918000
#define CSL_FW_IPCIE_G3X2_MAIN_1_PCIE_CFG_CORE__VMAP_HP__MMRS_END     0x29183ff
#define CSL_FW_IPCIE_G3X2_MAIN_1_PCIE_CFG_CORE__VMAP_LP__MMRS_START   0x2910000
#define CSL_FW_IPCIE_G3X2_MAIN_1_PCIE_CFG_CORE__VMAP_LP__MMRS_END     0x29103ff
#define CSL_FW_IPCIE_G3X2_MAIN_1_PCIE_CFG_CORE__CPTS__CPTS_VBUSP_START          0x2950000
#define CSL_FW_IPCIE_G3X2_MAIN_1_PCIE_CFG_CORE__CPTS__CPTS_VBUSP_END  0x29503ff
#define CSL_FW_IPCIE_G3X2_MAIN_1_PCIE_CFG_CORE__ECC_AGGR0__REGS_START 0x2a2a000
#define CSL_FW_IPCIE_G3X2_MAIN_1_PCIE_CFG_CORE__ECC_AGGR0__REGS_END   0x2a2a3ff
#define CSL_FW_IPCIE_G3X2_MAIN_1_PCIE_CFG_CORE__ECC_AGGR1__REGS_START 0x2a2b000
#define CSL_FW_IPCIE_G3X2_MAIN_1_PCIE_CFG_CORE__ECC_AGGR1__REGS_END   0x2a2b3ff

/* Properties of firewall protecting slave endpoint: IK3_BOLTV2_MAIN_0.KLIO_CFG*/
#define CSL_FW_IK3_BOLTV2_MAIN_0_KLIO_CFG_ID                          2704
#define CSL_FW_IK3_BOLTV2_MAIN_0_KLIO_CFG_TYPE                        CSL_FW_SECURITY
#define CSL_FW_IK3_BOLTV2_MAIN_0_KLIO_CFG_MMR_BASE                    0x452a4000
#define CSL_FW_IK3_BOLTV2_MAIN_0_KLIO_CFG_NUM_REGIONS                 3
#define CSL_FW_IK3_BOLTV2_MAIN_0_KLIO_CFG_NUM_PRIV_IDS_PER_REGION     3
#define CSL_FW_IK3_BOLTV2_MAIN_0_KLIO_CFG_NUM_PROTECTED_HW_REGIONS    3
#define CSL_FW_IK3_BOLTV2_MAIN_0_KLIO_CFG_KLIOMP1_HYD_MMRS_START      0x7004000
#define CSL_FW_IK3_BOLTV2_MAIN_0_KLIO_CFG_KLIOMP1_HYD_MMRS_END        0x7007fff
#define CSL_FW_IK3_BOLTV2_MAIN_0_KLIO_CFG_KLIOMP1_CORE0_MMRS_START    0x7008000
#define CSL_FW_IK3_BOLTV2_MAIN_0_KLIO_CFG_KLIOMP1_CORE0_MMRS_END      0x700bfff
#define CSL_FW_IK3_BOLTV2_MAIN_0_KLIO_CFG_KLIOMP1_HL_MMRS_START       0x700fe00
#define CSL_FW_IK3_BOLTV2_MAIN_0_KLIO_CFG_KLIOMP1_HL_MMRS_END         0x700ffff

/* Properties of firewall protecting slave endpoint: IK3_BOLTV2_MAIN_0.RAT_CFG*/
#define CSL_FW_IK3_BOLTV2_MAIN_0_RAT_CFG_ID                           2705
#define CSL_FW_IK3_BOLTV2_MAIN_0_RAT_CFG_TYPE                         CSL_FW_SECURITY
#define CSL_FW_IK3_BOLTV2_MAIN_0_RAT_CFG_MMR_BASE                     0x452a4400
#define CSL_FW_IK3_BOLTV2_MAIN_0_RAT_CFG_NUM_REGIONS                  1
#define CSL_FW_IK3_BOLTV2_MAIN_0_RAT_CFG_NUM_PRIV_IDS_PER_REGION      3
#define CSL_FW_IK3_BOLTV2_MAIN_0_RAT_CFG_NUM_PROTECTED_HW_REGIONS     1
#define CSL_FW_IK3_BOLTV2_MAIN_0_RAT_CFG_MMRS_START                   0x53a0000
#define CSL_FW_IK3_BOLTV2_MAIN_0_RAT_CFG_MMRS_END                     0x53a0fff

/* Properties of firewall protecting slave endpoint: IPDMA_DEBUG_MAIN_0.ECCAGGR_CFG*/
#define CSL_FW_IPDMA_DEBUG_MAIN_0_ECCAGGR_CFG_ID                      2720
#define CSL_FW_IPDMA_DEBUG_MAIN_0_ECCAGGR_CFG_TYPE                    CSL_FW_SECURITY
#define CSL_FW_IPDMA_DEBUG_MAIN_0_ECCAGGR_CFG_MMR_BASE                0x452a8000
#define CSL_FW_IPDMA_DEBUG_MAIN_0_ECCAGGR_CFG_NUM_REGIONS             1
#define CSL_FW_IPDMA_DEBUG_MAIN_0_ECCAGGR_CFG_NUM_PRIV_IDS_PER_REGION 3
#define CSL_FW_IPDMA_DEBUG_MAIN_0_ECCAGGR_CFG_NUM_PROTECTED_HW_REGIONS          1
#define CSL_FW_IPDMA_DEBUG_MAIN_0_ECCAGGR_CFG_REGS_START              0x2a40000
#define CSL_FW_IPDMA_DEBUG_MAIN_0_ECCAGGR_CFG_REGS_END                0x2a403ff

/* Properties of firewall protecting slave endpoint: IPDMA_MAIN0_MAIN_0.ECCAGGR_CFG*/
#define CSL_FW_IPDMA_MAIN0_MAIN_0_ECCAGGR_CFG_ID                      2721
#define CSL_FW_IPDMA_MAIN0_MAIN_0_ECCAGGR_CFG_TYPE                    CSL_FW_SECURITY
#define CSL_FW_IPDMA_MAIN0_MAIN_0_ECCAGGR_CFG_MMR_BASE                0x452a8400
#define CSL_FW_IPDMA_MAIN0_MAIN_0_ECCAGGR_CFG_NUM_REGIONS             1
#define CSL_FW_IPDMA_MAIN0_MAIN_0_ECCAGGR_CFG_NUM_PRIV_IDS_PER_REGION 3
#define CSL_FW_IPDMA_MAIN0_MAIN_0_ECCAGGR_CFG_NUM_PROTECTED_HW_REGIONS          1
#define CSL_FW_IPDMA_MAIN0_MAIN_0_ECCAGGR_CFG_REGS_START              0x2a41000
#define CSL_FW_IPDMA_MAIN0_MAIN_0_ECCAGGR_CFG_REGS_END                0x2a413ff

/* Properties of firewall protecting slave endpoint: IPDMA_MAIN1_MAIN_0.ECCAGGR_CFG*/
#define CSL_FW_IPDMA_MAIN1_MAIN_0_ECCAGGR_CFG_ID                      2722
#define CSL_FW_IPDMA_MAIN1_MAIN_0_ECCAGGR_CFG_TYPE                    CSL_FW_SECURITY
#define CSL_FW_IPDMA_MAIN1_MAIN_0_ECCAGGR_CFG_MMR_BASE                0x452a8800
#define CSL_FW_IPDMA_MAIN1_MAIN_0_ECCAGGR_CFG_NUM_REGIONS             1
#define CSL_FW_IPDMA_MAIN1_MAIN_0_ECCAGGR_CFG_NUM_PRIV_IDS_PER_REGION 3
#define CSL_FW_IPDMA_MAIN1_MAIN_0_ECCAGGR_CFG_NUM_PROTECTED_HW_REGIONS          1
#define CSL_FW_IPDMA_MAIN1_MAIN_0_ECCAGGR_CFG_REGS_START              0x2a42000
#define CSL_FW_IPDMA_MAIN1_MAIN_0_ECCAGGR_CFG_REGS_END                0x2a423ff

/* Properties of firewall protecting slave endpoint: INAVSS256L_MAIN_0.NAV_NB0_BP*/
#define CSL_FW_INAVSS256L_MAIN_0_NAV_NB0_BP_ID                        2808
#define CSL_FW_INAVSS256L_MAIN_0_NAV_NB0_BP_TYPE                      CSL_FW_SECURITY
#define CSL_FW_INAVSS256L_MAIN_0_NAV_NB0_BP_MMR_BASE                  0x452be000
#define CSL_FW_INAVSS256L_MAIN_0_NAV_NB0_BP_NUM_REGIONS               24
#define CSL_FW_INAVSS256L_MAIN_0_NAV_NB0_BP_NUM_PRIV_IDS_PER_REGION   3
#define CSL_FW_INAVSS256L_MAIN_0_NAV_NB0_BP_NUM_PROTECTED_HW_REGIONS  0

/* Properties of firewall protecting slave endpoint: IM4_MAINCLK2_ECC_AGGR_MAIN_0.CFG*/
#define CSL_FW_IM4_MAINCLK2_ECC_AGGR_MAIN_0_CFG_ID                    2816
#define CSL_FW_IM4_MAINCLK2_ECC_AGGR_MAIN_0_CFG_TYPE                  CSL_FW_SECURITY
#define CSL_FW_IM4_MAINCLK2_ECC_AGGR_MAIN_0_CFG_MMR_BASE              0x452c0000
#define CSL_FW_IM4_MAINCLK2_ECC_AGGR_MAIN_0_CFG_NUM_REGIONS           1
#define CSL_FW_IM4_MAINCLK2_ECC_AGGR_MAIN_0_CFG_NUM_PRIV_IDS_PER_REGION         3
#define CSL_FW_IM4_MAINCLK2_ECC_AGGR_MAIN_0_CFG_NUM_PROTECTED_HW_REGIONS        1
#define CSL_FW_IM4_MAINCLK2_ECC_AGGR_MAIN_0_CFG_REGS_START            0x2a50000
#define CSL_FW_IM4_MAINCLK2_ECC_AGGR_MAIN_0_CFG_REGS_END              0x2a503ff

/* Properties of firewall protecting slave endpoint: IM4_MAINCLK4_ECC_AGGR_MAIN_0.CFG*/
#define CSL_FW_IM4_MAINCLK4_ECC_AGGR_MAIN_0_CFG_ID                    2817
#define CSL_FW_IM4_MAINCLK4_ECC_AGGR_MAIN_0_CFG_TYPE                  CSL_FW_SECURITY
#define CSL_FW_IM4_MAINCLK4_ECC_AGGR_MAIN_0_CFG_MMR_BASE              0x452c0400
#define CSL_FW_IM4_MAINCLK4_ECC_AGGR_MAIN_0_CFG_NUM_REGIONS           1
#define CSL_FW_IM4_MAINCLK4_ECC_AGGR_MAIN_0_CFG_NUM_PRIV_IDS_PER_REGION         3
#define CSL_FW_IM4_MAINCLK4_ECC_AGGR_MAIN_0_CFG_NUM_PROTECTED_HW_REGIONS        1
#define CSL_FW_IM4_MAINCLK4_ECC_AGGR_MAIN_0_CFG_REGS_START            0x2a51000
#define CSL_FW_IM4_MAINCLK4_ECC_AGGR_MAIN_0_CFG_REGS_END              0x2a513ff

/* Properties of firewall protecting slave endpoint: IM4_MAIN_DBG_CBASS_MAIN_0.CBASS_ERR_SLV*/
#define CSL_FW_IM4_MAIN_DBG_CBASS_MAIN_0_CBASS_ERR_SLV_ID             2824
#define CSL_FW_IM4_MAIN_DBG_CBASS_MAIN_0_CBASS_ERR_SLV_TYPE           CSL_FW_SECURITY
#define CSL_FW_IM4_MAIN_DBG_CBASS_MAIN_0_CBASS_ERR_SLV_MMR_BASE       0x452c2000
#define CSL_FW_IM4_MAIN_DBG_CBASS_MAIN_0_CBASS_ERR_SLV_NUM_REGIONS    1
#define CSL_FW_IM4_MAIN_DBG_CBASS_MAIN_0_CBASS_ERR_SLV_NUM_PRIV_IDS_PER_REGION  3
#define CSL_FW_IM4_MAIN_DBG_CBASS_MAIN_0_CBASS_ERR_SLV_NUM_PROTECTED_HW_REGIONS 1
#define CSL_FW_IM4_MAIN_DBG_CBASS_MAIN_0_CBASS_ERR_SLV_ERR_REGS_START 0x2a80000
#define CSL_FW_IM4_MAIN_DBG_CBASS_MAIN_0_CBASS_ERR_SLV_ERR_REGS_END   0x2a803ff

/* Properties of firewall protecting slave endpoint: IM4_MAIN_CBASS_MAIN_0.CBASS_ERR_SLV*/
#define CSL_FW_IM4_MAIN_CBASS_MAIN_0_CBASS_ERR_SLV_ID                 2825
#define CSL_FW_IM4_MAIN_CBASS_MAIN_0_CBASS_ERR_SLV_TYPE               CSL_FW_SECURITY
#define CSL_FW_IM4_MAIN_CBASS_MAIN_0_CBASS_ERR_SLV_MMR_BASE           0x452c2400
#define CSL_FW_IM4_MAIN_CBASS_MAIN_0_CBASS_ERR_SLV_NUM_REGIONS        1
#define CSL_FW_IM4_MAIN_CBASS_MAIN_0_CBASS_ERR_SLV_NUM_PRIV_IDS_PER_REGION      3
#define CSL_FW_IM4_MAIN_CBASS_MAIN_0_CBASS_ERR_SLV_NUM_PROTECTED_HW_REGIONS     1
#define CSL_FW_IM4_MAIN_CBASS_MAIN_0_CBASS_ERR_SLV_ERR_REGS_START     0x2a81000
#define CSL_FW_IM4_MAIN_CBASS_MAIN_0_CBASS_ERR_SLV_ERR_REGS_END       0x2a813ff

/* Properties of firewall protecting slave endpoint: MSMC0_SLV_VIRTID.CFG*/
#define CSL_FW_NAVSS256L_MODSS_MSMC0_SLV_VIRTID_CFG_ID                4096
#define CSL_FW_NAVSS256L_MODSS_MSMC0_SLV_VIRTID_CFG_TYPE              CSL_FW_SECURITY
#define CSL_FW_NAVSS256L_MODSS_MSMC0_SLV_VIRTID_CFG_MMR_BASE          0x45400000
#define CSL_FW_NAVSS256L_MODSS_MSMC0_SLV_VIRTID_CFG_NUM_REGIONS       1
#define CSL_FW_NAVSS256L_MODSS_MSMC0_SLV_VIRTID_CFG_NUM_PRIV_IDS_PER_REGION     3
#define CSL_FW_NAVSS256L_MODSS_MSMC0_SLV_VIRTID_CFG_NUM_PROTECTED_HW_REGIONS    1
#define CSL_FW_NAVSS256L_MODSS_MSMC0_SLV_VIRTID_CFG_MMRS_START        0x30a00000
#define CSL_FW_NAVSS256L_MODSS_MSMC0_SLV_VIRTID_CFG_MMRS_END          0x30a000ff

/* Properties of firewall protecting slave endpoint: MSMC1_SLV_VIRTID.CFG*/
#define CSL_FW_NAVSS256L_MODSS_MSMC1_SLV_VIRTID_CFG_ID                4097
#define CSL_FW_NAVSS256L_MODSS_MSMC1_SLV_VIRTID_CFG_TYPE              CSL_FW_SECURITY
#define CSL_FW_NAVSS256L_MODSS_MSMC1_SLV_VIRTID_CFG_MMR_BASE          0x45400400
#define CSL_FW_NAVSS256L_MODSS_MSMC1_SLV_VIRTID_CFG_NUM_REGIONS       1
#define CSL_FW_NAVSS256L_MODSS_MSMC1_SLV_VIRTID_CFG_NUM_PRIV_IDS_PER_REGION     3
#define CSL_FW_NAVSS256L_MODSS_MSMC1_SLV_VIRTID_CFG_NUM_PROTECTED_HW_REGIONS    1
#define CSL_FW_NAVSS256L_MODSS_MSMC1_SLV_VIRTID_CFG_MMRS_START        0x30a01000
#define CSL_FW_NAVSS256L_MODSS_MSMC1_SLV_VIRTID_CFG_MMRS_END          0x30a010ff

/* Properties of firewall protecting slave endpoint: NAV_DDR_LO_VIRTID.CFG*/
#define CSL_FW_NAVSS256L_MODSS_NAV_DDR_LO_VIRTID_CFG_ID               4098
#define CSL_FW_NAVSS256L_MODSS_NAV_DDR_LO_VIRTID_CFG_TYPE             CSL_FW_SECURITY
#define CSL_FW_NAVSS256L_MODSS_NAV_DDR_LO_VIRTID_CFG_MMR_BASE         0x45400800
#define CSL_FW_NAVSS256L_MODSS_NAV_DDR_LO_VIRTID_CFG_NUM_REGIONS      1
#define CSL_FW_NAVSS256L_MODSS_NAV_DDR_LO_VIRTID_CFG_NUM_PRIV_IDS_PER_REGION    3
#define CSL_FW_NAVSS256L_MODSS_NAV_DDR_LO_VIRTID_CFG_NUM_PROTECTED_HW_REGIONS   1
#define CSL_FW_NAVSS256L_MODSS_NAV_DDR_LO_VIRTID_CFG_MMRS_START       0x30a02000
#define CSL_FW_NAVSS256L_MODSS_NAV_DDR_LO_VIRTID_CFG_MMRS_END         0x30a020ff

/* Properties of firewall protecting slave endpoint: NAV_DDR_HI_VIRTID.CFG*/
#define CSL_FW_NAVSS256L_MODSS_NAV_DDR_HI_VIRTID_CFG_ID               4099
#define CSL_FW_NAVSS256L_MODSS_NAV_DDR_HI_VIRTID_CFG_TYPE             CSL_FW_SECURITY
#define CSL_FW_NAVSS256L_MODSS_NAV_DDR_HI_VIRTID_CFG_MMR_BASE         0x45400c00
#define CSL_FW_NAVSS256L_MODSS_NAV_DDR_HI_VIRTID_CFG_NUM_REGIONS      1
#define CSL_FW_NAVSS256L_MODSS_NAV_DDR_HI_VIRTID_CFG_NUM_PRIV_IDS_PER_REGION    3
#define CSL_FW_NAVSS256L_MODSS_NAV_DDR_HI_VIRTID_CFG_NUM_PROTECTED_HW_REGIONS   1
#define CSL_FW_NAVSS256L_MODSS_NAV_DDR_HI_VIRTID_CFG_MMRS_START       0x30a03000
#define CSL_FW_NAVSS256L_MODSS_NAV_DDR_HI_VIRTID_CFG_MMRS_END         0x30a030ff

/* Properties of firewall protecting slave endpoint: CPTS0.S_VBUSP*/
#define CSL_FW_NAVSS256L_MODSS_CPTS0_S_VBUSP_ID                       4100
#define CSL_FW_NAVSS256L_MODSS_CPTS0_S_VBUSP_TYPE                     CSL_FW_SECURITY
#define CSL_FW_NAVSS256L_MODSS_CPTS0_S_VBUSP_MMR_BASE                 0x45401000
#define CSL_FW_NAVSS256L_MODSS_CPTS0_S_VBUSP_NUM_REGIONS              1
#define CSL_FW_NAVSS256L_MODSS_CPTS0_S_VBUSP_NUM_PRIV_IDS_PER_REGION  3
#define CSL_FW_NAVSS256L_MODSS_CPTS0_S_VBUSP_NUM_PROTECTED_HW_REGIONS 1
#define CSL_FW_NAVSS256L_MODSS_CPTS0_S_VBUSP_CPTS_VBUSP_START         0x310d0000
#define CSL_FW_NAVSS256L_MODSS_CPTS0_S_VBUSP_CPTS_VBUSP_END           0x310d03ff

/* Properties of firewall protecting slave endpoint: SPINLOCK0.CFG*/
#define CSL_FW_NAVSS256L_MODSS_SPINLOCK0_CFG_ID                       4101
#define CSL_FW_NAVSS256L_MODSS_SPINLOCK0_CFG_TYPE                     CSL_FW_SECURITY
#define CSL_FW_NAVSS256L_MODSS_SPINLOCK0_CFG_MMR_BASE                 0x45401400
#define CSL_FW_NAVSS256L_MODSS_SPINLOCK0_CFG_NUM_REGIONS              1
#define CSL_FW_NAVSS256L_MODSS_SPINLOCK0_CFG_NUM_PRIV_IDS_PER_REGION  3
#define CSL_FW_NAVSS256L_MODSS_SPINLOCK0_CFG_NUM_PROTECTED_HW_REGIONS 1
#define CSL_FW_NAVSS256L_MODSS_SPINLOCK0_CFG_REGS_START               0x30e00000
#define CSL_FW_NAVSS256L_MODSS_SPINLOCK0_CFG_REGS_END                 0x30e07fff

/* Properties of firewall protecting slave endpoint: MAILBOX0.CFG*/
#define CSL_FW_NAVSS256L_MODSS_MAILBOX0_CFG_ID                        4114
#define CSL_FW_NAVSS256L_MODSS_MAILBOX0_CFG_TYPE                      CSL_FW_SECURITY
#define CSL_FW_NAVSS256L_MODSS_MAILBOX0_CFG_MMR_BASE                  0x45404800
#define CSL_FW_NAVSS256L_MODSS_MAILBOX0_CFG_NUM_REGIONS               12
#define CSL_FW_NAVSS256L_MODSS_MAILBOX0_CFG_NUM_PRIV_IDS_PER_REGION   3
#define CSL_FW_NAVSS256L_MODSS_MAILBOX0_CFG_NUM_PROTECTED_HW_REGIONS  12
#define CSL_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS0_START               0x31f80000
#define CSL_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS0_END                 0x31f801ff
#define CSL_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS1_START               0x31f81000
#define CSL_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS1_END                 0x31f811ff
#define CSL_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS2_START               0x31f82000
#define CSL_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS2_END                 0x31f821ff
#define CSL_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS3_START               0x31f83000
#define CSL_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS3_END                 0x31f831ff
#define CSL_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS4_START               0x31f84000
#define CSL_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS4_END                 0x31f841ff
#define CSL_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS5_START               0x31f85000
#define CSL_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS5_END                 0x31f851ff
#define CSL_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS6_START               0x31f86000
#define CSL_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS6_END                 0x31f861ff
#define CSL_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS7_START               0x31f87000
#define CSL_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS7_END                 0x31f871ff
#define CSL_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS8_START               0x31f88000
#define CSL_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS8_END                 0x31f881ff
#define CSL_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS9_START               0x31f89000
#define CSL_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS9_END                 0x31f891ff
#define CSL_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS10_START              0x31f8a000
#define CSL_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS10_END                0x31f8a1ff
#define CSL_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS11_START              0x31f8b000
#define CSL_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS11_END                0x31f8b1ff

/* Properties of firewall protecting slave endpoint: TIMERMGR0.CFG*/
#define CSL_FW_NAVSS256L_MODSS_TIMERMGR0_CFG_ID                       4120
#define CSL_FW_NAVSS256L_MODSS_TIMERMGR0_CFG_TYPE                     CSL_FW_SECURITY
#define CSL_FW_NAVSS256L_MODSS_TIMERMGR0_CFG_MMR_BASE                 0x45406000
#define CSL_FW_NAVSS256L_MODSS_TIMERMGR0_CFG_NUM_REGIONS              3
#define CSL_FW_NAVSS256L_MODSS_TIMERMGR0_CFG_NUM_PRIV_IDS_PER_REGION  3
#define CSL_FW_NAVSS256L_MODSS_TIMERMGR0_CFG_NUM_PROTECTED_HW_REGIONS 3
#define CSL_FW_NAVSS256L_MODSS_TIMERMGR0_CFG_CFG_START                0x30e80000
#define CSL_FW_NAVSS256L_MODSS_TIMERMGR0_CFG_CFG_END                  0x30e801ff
#define CSL_FW_NAVSS256L_MODSS_TIMERMGR0_CFG_TIMERS_START             0x32200000
#define CSL_FW_NAVSS256L_MODSS_TIMERMGR0_CFG_TIMERS_END               0x3223ffff
#define CSL_FW_NAVSS256L_MODSS_TIMERMGR0_CFG_OES_START                0x30f00000
#define CSL_FW_NAVSS256L_MODSS_TIMERMGR0_CFG_OES_END                  0x30f00fff

/* Properties of firewall protecting slave endpoint: TIMERMGR1.CFG*/
#define CSL_FW_NAVSS256L_MODSS_TIMERMGR1_CFG_ID                       4128
#define CSL_FW_NAVSS256L_MODSS_TIMERMGR1_CFG_TYPE                     CSL_FW_SECURITY
#define CSL_FW_NAVSS256L_MODSS_TIMERMGR1_CFG_MMR_BASE                 0x45408000
#define CSL_FW_NAVSS256L_MODSS_TIMERMGR1_CFG_NUM_REGIONS              3
#define CSL_FW_NAVSS256L_MODSS_TIMERMGR1_CFG_NUM_PRIV_IDS_PER_REGION  3
#define CSL_FW_NAVSS256L_MODSS_TIMERMGR1_CFG_NUM_PROTECTED_HW_REGIONS 3
#define CSL_FW_NAVSS256L_MODSS_TIMERMGR1_CFG_CFG_START                0x30e81000
#define CSL_FW_NAVSS256L_MODSS_TIMERMGR1_CFG_CFG_END                  0x30e811ff
#define CSL_FW_NAVSS256L_MODSS_TIMERMGR1_CFG_TIMERS_START             0x32240000
#define CSL_FW_NAVSS256L_MODSS_TIMERMGR1_CFG_TIMERS_END               0x3227ffff
#define CSL_FW_NAVSS256L_MODSS_TIMERMGR1_CFG_OES_START                0x30f01000
#define CSL_FW_NAVSS256L_MODSS_TIMERMGR1_CFG_OES_END                  0x30f01fff

/* Properties of firewall protecting slave endpoint: MODSS_INTA0.CFG*/
#define CSL_FW_NAVSS256L_MODSS_MODSS_INTA0_CFG_ID                     4136
#define CSL_FW_NAVSS256L_MODSS_MODSS_INTA0_CFG_TYPE                   CSL_FW_SECURITY
#define CSL_FW_NAVSS256L_MODSS_MODSS_INTA0_CFG_MMR_BASE               0x4540a000
#define CSL_FW_NAVSS256L_MODSS_MODSS_INTA0_CFG_NUM_REGIONS            3
#define CSL_FW_NAVSS256L_MODSS_MODSS_INTA0_CFG_NUM_PRIV_IDS_PER_REGION          3
#define CSL_FW_NAVSS256L_MODSS_MODSS_INTA0_CFG_NUM_PROTECTED_HW_REGIONS         3
#define CSL_FW_NAVSS256L_MODSS_MODSS_INTA0_CFG_CFG_START              0x30800000
#define CSL_FW_NAVSS256L_MODSS_MODSS_INTA0_CFG_CFG_END                0x3080001f
#define CSL_FW_NAVSS256L_MODSS_MODSS_INTA0_CFG_IMAP_START             0x30900000
#define CSL_FW_NAVSS256L_MODSS_MODSS_INTA0_CFG_IMAP_END               0x30907fff
#define CSL_FW_NAVSS256L_MODSS_MODSS_INTA0_CFG_INTR_START             0x33c00000
#define CSL_FW_NAVSS256L_MODSS_MODSS_INTA0_CFG_INTR_END               0x33c3ffff

/* Properties of firewall protecting slave endpoint: MODSS_INTA1.CFG*/
#define CSL_FW_NAVSS256L_MODSS_MODSS_INTA1_CFG_ID                     4144
#define CSL_FW_NAVSS256L_MODSS_MODSS_INTA1_CFG_TYPE                   CSL_FW_SECURITY
#define CSL_FW_NAVSS256L_MODSS_MODSS_INTA1_CFG_MMR_BASE               0x4540c000
#define CSL_FW_NAVSS256L_MODSS_MODSS_INTA1_CFG_NUM_REGIONS            3
#define CSL_FW_NAVSS256L_MODSS_MODSS_INTA1_CFG_NUM_PRIV_IDS_PER_REGION          3
#define CSL_FW_NAVSS256L_MODSS_MODSS_INTA1_CFG_NUM_PROTECTED_HW_REGIONS         3
#define CSL_FW_NAVSS256L_MODSS_MODSS_INTA1_CFG_CFG_START              0x30801000
#define CSL_FW_NAVSS256L_MODSS_MODSS_INTA1_CFG_CFG_END                0x3080101f
#define CSL_FW_NAVSS256L_MODSS_MODSS_INTA1_CFG_IMAP_START             0x30908000
#define CSL_FW_NAVSS256L_MODSS_MODSS_INTA1_CFG_IMAP_END               0x3090ffff
#define CSL_FW_NAVSS256L_MODSS_MODSS_INTA1_CFG_INTR_START             0x33c40000
#define CSL_FW_NAVSS256L_MODSS_MODSS_INTA1_CFG_INTR_END               0x33c7ffff

/* Properties of firewall protecting slave endpoint: PROXY0.CFG*/
#define CSL_FW_NAVSS256L_MODSS_PROXY0_CFG_ID                          4152
#define CSL_FW_NAVSS256L_MODSS_PROXY0_CFG_TYPE                        CSL_FW_SECURITY
#define CSL_FW_NAVSS256L_MODSS_PROXY0_CFG_MMR_BASE                    0x4540e000
#define CSL_FW_NAVSS256L_MODSS_PROXY0_CFG_NUM_REGIONS                 3
#define CSL_FW_NAVSS256L_MODSS_PROXY0_CFG_NUM_PRIV_IDS_PER_REGION     3
#define CSL_FW_NAVSS256L_MODSS_PROXY0_CFG_NUM_PROTECTED_HW_REGIONS    3
#define CSL_FW_NAVSS256L_MODSS_PROXY0_CFG_BUF__CFG__GCFG_START        0x31120000
#define CSL_FW_NAVSS256L_MODSS_PROXY0_CFG_BUF__CFG__GCFG_END          0x311200ff
#define CSL_FW_NAVSS256L_MODSS_PROXY0_CFG_BUF__CFG__CFG_START         0x33400000
#define CSL_FW_NAVSS256L_MODSS_PROXY0_CFG_BUF__CFG__CFG_END           0x3343ffff
#define CSL_FW_NAVSS256L_MODSS_PROXY0_CFG_BUF__BUFRAM_SLV__RAM_START  0x31130000
#define CSL_FW_NAVSS256L_MODSS_PROXY0_CFG_BUF__BUFRAM_SLV__RAM_END    0x31133fff

/* Properties of firewall protecting slave endpoint: SEC_PROXY0.CFG*/
#define CSL_FW_NAVSS256L_MODSS_SEC_PROXY0_CFG_ID                      4168
#define CSL_FW_NAVSS256L_MODSS_SEC_PROXY0_CFG_TYPE                    CSL_FW_SECURITY
#define CSL_FW_NAVSS256L_MODSS_SEC_PROXY0_CFG_MMR_BASE                0x45412000
#define CSL_FW_NAVSS256L_MODSS_SEC_PROXY0_CFG_NUM_REGIONS             3
#define CSL_FW_NAVSS256L_MODSS_SEC_PROXY0_CFG_NUM_PRIV_IDS_PER_REGION 3
#define CSL_FW_NAVSS256L_MODSS_SEC_PROXY0_CFG_NUM_PROTECTED_HW_REGIONS          3
#define CSL_FW_NAVSS256L_MODSS_SEC_PROXY0_CFG_MMRS_START              0x31140000
#define CSL_FW_NAVSS256L_MODSS_SEC_PROXY0_CFG_MMRS_END                0x311400ff
#define CSL_FW_NAVSS256L_MODSS_SEC_PROXY0_CFG_SCFG_START              0x32800000
#define CSL_FW_NAVSS256L_MODSS_SEC_PROXY0_CFG_SCFG_END                0x328fffff
#define CSL_FW_NAVSS256L_MODSS_SEC_PROXY0_CFG_RT_START                0x32400000
#define CSL_FW_NAVSS256L_MODSS_SEC_PROXY0_CFG_RT_END                  0x324fffff

/* Properties of firewall protecting slave endpoint: PVU0.CFG*/
#define CSL_FW_NAVSS256L_MODSS_PVU0_CFG_ID                            4192
#define CSL_FW_NAVSS256L_MODSS_PVU0_CFG_TYPE                          CSL_FW_SECURITY
#define CSL_FW_NAVSS256L_MODSS_PVU0_CFG_MMR_BASE                      0x45418000
#define CSL_FW_NAVSS256L_MODSS_PVU0_CFG_NUM_REGIONS                   2
#define CSL_FW_NAVSS256L_MODSS_PVU0_CFG_NUM_PRIV_IDS_PER_REGION       3
#define CSL_FW_NAVSS256L_MODSS_PVU0_CFG_NUM_PROTECTED_HW_REGIONS      2
#define CSL_FW_NAVSS256L_MODSS_PVU0_CFG_CFG__CFG__MMRS_START          0x30f80000
#define CSL_FW_NAVSS256L_MODSS_PVU0_CFG_CFG__CFG__MMRS_END            0x30f80fff
#define CSL_FW_NAVSS256L_MODSS_PVU0_CFG_TLBIF__CFG__TLB_START         0x36000000
#define CSL_FW_NAVSS256L_MODSS_PVU0_CFG_TLBIF__CFG__TLB_END           0x360fffff

/* Properties of firewall protecting slave endpoint: PVU1.CFG*/
#define CSL_FW_NAVSS256L_MODSS_PVU1_CFG_ID                            4224
#define CSL_FW_NAVSS256L_MODSS_PVU1_CFG_TYPE                          CSL_FW_SECURITY
#define CSL_FW_NAVSS256L_MODSS_PVU1_CFG_MMR_BASE                      0x45420000
#define CSL_FW_NAVSS256L_MODSS_PVU1_CFG_NUM_REGIONS                   2
#define CSL_FW_NAVSS256L_MODSS_PVU1_CFG_NUM_PRIV_IDS_PER_REGION       3
#define CSL_FW_NAVSS256L_MODSS_PVU1_CFG_NUM_PROTECTED_HW_REGIONS      2
#define CSL_FW_NAVSS256L_MODSS_PVU1_CFG_CFG__CFG__MMRS_START          0x30f81000
#define CSL_FW_NAVSS256L_MODSS_PVU1_CFG_CFG__CFG__MMRS_END            0x30f81fff
#define CSL_FW_NAVSS256L_MODSS_PVU1_CFG_TLBIF__CFG__TLB_START         0x36100000
#define CSL_FW_NAVSS256L_MODSS_PVU1_CFG_TLBIF__CFG__TLB_END           0x361fffff

/* Properties of firewall protecting slave endpoint: ^+UDMASS.RINGACC0_CFG*/
#define CSL_FW_NAVSS256L_MODSS_RINGACC0_CFG_ID                        4288
#define CSL_FW_NAVSS256L_MODSS_RINGACC0_CFG_TYPE                      CSL_FW_SECURITY
#define CSL_FW_NAVSS256L_MODSS_RINGACC0_CFG_MMR_BASE                  0x45430000
#define CSL_FW_NAVSS256L_MODSS_RINGACC0_CFG_NUM_REGIONS               4
#define CSL_FW_NAVSS256L_MODSS_RINGACC0_CFG_NUM_PRIV_IDS_PER_REGION   3
#define CSL_FW_NAVSS256L_MODSS_RINGACC0_CFG_NUM_PROTECTED_HW_REGIONS  4
#define CSL_FW_NAVSS256L_MODSS_RINGACC0_CFG_UDMASS_RINGACC0_GCFG_START          0x31160000
#define CSL_FW_NAVSS256L_MODSS_RINGACC0_CFG_UDMASS_RINGACC0_GCFG_END  0x311603ff
#define CSL_FW_NAVSS256L_MODSS_RINGACC0_CFG_UDMASS_RINGACC0_CFG_START 0x31080000
#define CSL_FW_NAVSS256L_MODSS_RINGACC0_CFG_UDMASS_RINGACC0_CFG_END   0x310bffff
#define CSL_FW_NAVSS256L_MODSS_RINGACC0_CFG_UDMASS_RINGACC0_RT_START  0x3c000000
#define CSL_FW_NAVSS256L_MODSS_RINGACC0_CFG_UDMASS_RINGACC0_RT_END    0x3c3fffff
#define CSL_FW_NAVSS256L_MODSS_RINGACC0_CFG_UDMASS_RINGACC0_MON_START 0x32000000
#define CSL_FW_NAVSS256L_MODSS_RINGACC0_CFG_UDMASS_RINGACC0_MON_END   0x3201ffff

/* Properties of firewall protecting slave endpoint: ^+UDMASS.UDMASS_INTA0_CFG*/
#define CSL_FW_NAVSS256L_MODSS_UDMASSINTA0_CFG_ID                     4352
#define CSL_FW_NAVSS256L_MODSS_UDMASSINTA0_CFG_TYPE                   CSL_FW_SECURITY
#define CSL_FW_NAVSS256L_MODSS_UDMASSINTA0_CFG_MMR_BASE               0x45440000
#define CSL_FW_NAVSS256L_MODSS_UDMASSINTA0_CFG_NUM_REGIONS            7
#define CSL_FW_NAVSS256L_MODSS_UDMASSINTA0_CFG_NUM_PRIV_IDS_PER_REGION          3
#define CSL_FW_NAVSS256L_MODSS_UDMASSINTA0_CFG_NUM_PROTECTED_HW_REGIONS         7
#define CSL_FW_NAVSS256L_MODSS_UDMASSINTA0_CFG_UDMASS_INTA0_CFG_START 0x30802000
#define CSL_FW_NAVSS256L_MODSS_UDMASSINTA0_CFG_UDMASS_INTA0_CFG_END   0x3080201f
#define CSL_FW_NAVSS256L_MODSS_UDMASSINTA0_CFG_UDMASS_INTA0_IMAP_START          0x30940000
#define CSL_FW_NAVSS256L_MODSS_UDMASSINTA0_CFG_UDMASS_INTA0_IMAP_END  0x3097ffff
#define CSL_FW_NAVSS256L_MODSS_UDMASSINTA0_CFG_UDMASS_INTA0_INTR_START          0x33d00000
#define CSL_FW_NAVSS256L_MODSS_UDMASSINTA0_CFG_UDMASS_INTA0_INTR_END  0x33dfffff
#define CSL_FW_NAVSS256L_MODSS_UDMASSINTA0_CFG_UDMASS_INTA0_L2G_START 0x31100000
#define CSL_FW_NAVSS256L_MODSS_UDMASSINTA0_CFG_UDMASS_INTA0_L2G_END   0x3110007f
#define CSL_FW_NAVSS256L_MODSS_UDMASSINTA0_CFG_UDMASS_INTA0_MCAST_START         0x31110000
#define CSL_FW_NAVSS256L_MODSS_UDMASSINTA0_CFG_UDMASS_INTA0_MCAST_END 0x31113fff
#define CSL_FW_NAVSS256L_MODSS_UDMASSINTA0_CFG_UDMASS_INTA0_GCNTCFG_START       0x31040000
#define CSL_FW_NAVSS256L_MODSS_UDMASSINTA0_CFG_UDMASS_INTA0_GCNTCFG_END         0x31043fff
#define CSL_FW_NAVSS256L_MODSS_UDMASSINTA0_CFG_UDMASS_INTA0_GCNTRTI_START       0x33800000
#define CSL_FW_NAVSS256L_MODSS_UDMASSINTA0_CFG_UDMASS_INTA0_GCNTRTI_END         0x339fffff

/* Properties of firewall protecting slave endpoint: ^+UDMASS.UDMAP0_CFG*/
#define CSL_FW_NAVSS256L_MODSS_UDMAP0_CFG_ID                          4384
#define CSL_FW_NAVSS256L_MODSS_UDMAP0_CFG_TYPE                        CSL_FW_SECURITY
#define CSL_FW_NAVSS256L_MODSS_UDMAP0_CFG_MMR_BASE                    0x45448000
#define CSL_FW_NAVSS256L_MODSS_UDMAP0_CFG_NUM_REGIONS                 6
#define CSL_FW_NAVSS256L_MODSS_UDMAP0_CFG_NUM_PRIV_IDS_PER_REGION     3
#define CSL_FW_NAVSS256L_MODSS_UDMAP0_CFG_NUM_PROTECTED_HW_REGIONS    6
#define CSL_FW_NAVSS256L_MODSS_UDMAP0_CFG_UDMASS_UDMAP0_GCFG_START    0x31150000
#define CSL_FW_NAVSS256L_MODSS_UDMAP0_CFG_UDMASS_UDMAP0_GCFG_END      0x311500ff
#define CSL_FW_NAVSS256L_MODSS_UDMAP0_CFG_UDMASS_UDMAP0_RFLOW_START   0x30d00000
#define CSL_FW_NAVSS256L_MODSS_UDMAP0_CFG_UDMASS_UDMAP0_RFLOW_END     0x30d07fff
#define CSL_FW_NAVSS256L_MODSS_UDMAP0_CFG_UDMASS_UDMAP0_TCHAN_START   0x30b00000
#define CSL_FW_NAVSS256L_MODSS_UDMAP0_CFG_UDMASS_UDMAP0_TCHAN_END     0x30b0ffff
#define CSL_FW_NAVSS256L_MODSS_UDMAP0_CFG_UDMASS_UDMAP0_RCHAN_START   0x30c00000
#define CSL_FW_NAVSS256L_MODSS_UDMAP0_CFG_UDMASS_UDMAP0_RCHAN_END     0x30c0ffff
#define CSL_FW_NAVSS256L_MODSS_UDMAP0_CFG_UDMASS_UDMAP0_TCHANRT_START 0x35000000
#define CSL_FW_NAVSS256L_MODSS_UDMAP0_CFG_UDMASS_UDMAP0_TCHANRT_END   0x350fffff
#define CSL_FW_NAVSS256L_MODSS_UDMAP0_CFG_UDMASS_UDMAP0_RCHANRT_START 0x34000000
#define CSL_FW_NAVSS256L_MODSS_UDMAP0_CFG_UDMASS_UDMAP0_RCHANRT_END   0x340fffff

/* Properties of firewall protecting slave endpoint: .ECCAGGR0*/
#define CSL_FW_NAVSS256L_MODSS_NAVSS256L_MODSS_CBASS_ECCAGGR0_ID      4385
#define CSL_FW_NAVSS256L_MODSS_NAVSS256L_MODSS_CBASS_ECCAGGR0_TYPE    CSL_FW_SECURITY
#define CSL_FW_NAVSS256L_MODSS_NAVSS256L_MODSS_CBASS_ECCAGGR0_MMR_BASE          0x45448400
#define CSL_FW_NAVSS256L_MODSS_NAVSS256L_MODSS_CBASS_ECCAGGR0_NUM_REGIONS       1
#define CSL_FW_NAVSS256L_MODSS_NAVSS256L_MODSS_CBASS_ECCAGGR0_NUM_PRIV_IDS_PER_REGION     3
#define CSL_FW_NAVSS256L_MODSS_NAVSS256L_MODSS_CBASS_ECCAGGR0_NUM_PROTECTED_HW_REGIONS    1
#define CSL_FW_NAVSS256L_MODSS_NAVSS256L_MODSS_CBASS_ECCAGGR0_REGS_START        0x31000000
#define CSL_FW_NAVSS256L_MODSS_NAVSS256L_MODSS_CBASS_ECCAGGR0_REGS_END          0x310003ff

/* Properties of firewall protecting slave endpoint: ^+UDMASS.UDMASS_ECCAGGR0_CFG*/
#define CSL_FW_NAVSS256L_MODSS_UDMASSECCAGGR0_CFG_ID                  4386
#define CSL_FW_NAVSS256L_MODSS_UDMASSECCAGGR0_CFG_TYPE                CSL_FW_SECURITY
#define CSL_FW_NAVSS256L_MODSS_UDMASSECCAGGR0_CFG_MMR_BASE            0x45448800
#define CSL_FW_NAVSS256L_MODSS_UDMASSECCAGGR0_CFG_NUM_REGIONS         1
#define CSL_FW_NAVSS256L_MODSS_UDMASSECCAGGR0_CFG_NUM_PRIV_IDS_PER_REGION       3
#define CSL_FW_NAVSS256L_MODSS_UDMASSECCAGGR0_CFG_NUM_PROTECTED_HW_REGIONS      1
#define CSL_FW_NAVSS256L_MODSS_UDMASSECCAGGR0_CFG_UDMASS_ECCAGGR0_REGS_START    0x31001000
#define CSL_FW_NAVSS256L_MODSS_UDMASSECCAGGR0_CFG_UDMASS_ECCAGGR0_REGS_END      0x310013ff

/* Properties of firewall protecting slave endpoint: INTR0.CFG*/
#define CSL_FW_NAVSS256L_MODSS_INTR0_CFG_ID                           4387
#define CSL_FW_NAVSS256L_MODSS_INTR0_CFG_TYPE                         CSL_FW_SECURITY
#define CSL_FW_NAVSS256L_MODSS_INTR0_CFG_MMR_BASE                     0x45448c00
#define CSL_FW_NAVSS256L_MODSS_INTR0_CFG_NUM_REGIONS                  1
#define CSL_FW_NAVSS256L_MODSS_INTR0_CFG_NUM_PRIV_IDS_PER_REGION      3
#define CSL_FW_NAVSS256L_MODSS_INTR0_CFG_NUM_PROTECTED_HW_REGIONS     1
#define CSL_FW_NAVSS256L_MODSS_INTR0_CFG_INTR_ROUTER_CFG_START        0x310e0000
#define CSL_FW_NAVSS256L_MODSS_INTR0_CFG_INTR_ROUTER_CFG_END          0x310e1fff

/* Properties of firewall protecting slave endpoint: ^+UDMASS.PSILCFG0_CFG*/
#define CSL_FW_NAVSS256L_MODSS_PSILCFG0_CFG_ID                        4388
#define CSL_FW_NAVSS256L_MODSS_PSILCFG0_CFG_TYPE                      CSL_FW_SECURITY
#define CSL_FW_NAVSS256L_MODSS_PSILCFG0_CFG_MMR_BASE                  0x45449000
#define CSL_FW_NAVSS256L_MODSS_PSILCFG0_CFG_NUM_REGIONS               1
#define CSL_FW_NAVSS256L_MODSS_PSILCFG0_CFG_NUM_PRIV_IDS_PER_REGION   3
#define CSL_FW_NAVSS256L_MODSS_PSILCFG0_CFG_NUM_PROTECTED_HW_REGIONS  1
#define CSL_FW_NAVSS256L_MODSS_PSILCFG0_CFG_UDMASS_PSILCFG0_PROXY_START         0x31f78000
#define CSL_FW_NAVSS256L_MODSS_PSILCFG0_CFG_UDMASS_PSILCFG0_PROXY_END 0x31f781ff

/* Properties of firewall protecting slave endpoint: ^+UDMASS.PSILSS0_CFG*/
#define CSL_FW_NAVSS256L_MODSS_PSILSS0_CFG_ID                         4389
#define CSL_FW_NAVSS256L_MODSS_PSILSS0_CFG_TYPE                       CSL_FW_SECURITY
#define CSL_FW_NAVSS256L_MODSS_PSILSS0_CFG_MMR_BASE                   0x45449400
#define CSL_FW_NAVSS256L_MODSS_PSILSS0_CFG_NUM_REGIONS                1
#define CSL_FW_NAVSS256L_MODSS_PSILSS0_CFG_NUM_PRIV_IDS_PER_REGION    3
#define CSL_FW_NAVSS256L_MODSS_PSILSS0_CFG_NUM_PROTECTED_HW_REGIONS   1
#define CSL_FW_NAVSS256L_MODSS_PSILSS0_CFG_UDMASS_PSILSS0_MMRS_START  0x31170000
#define CSL_FW_NAVSS256L_MODSS_PSILSS0_CFG_UDMASS_PSILSS0_MMRS_END    0x31170fff

/* Properties of firewall protecting slave endpoint: MCRC0.S_CFG*/
#define CSL_FW_NAVSS256L_MODSS_MCRC0_S_CFG_ID                         4390
#define CSL_FW_NAVSS256L_MODSS_MCRC0_S_CFG_TYPE                       CSL_FW_SECURITY
#define CSL_FW_NAVSS256L_MODSS_MCRC0_S_CFG_MMR_BASE                   0x45449800
#define CSL_FW_NAVSS256L_MODSS_MCRC0_S_CFG_NUM_REGIONS                1
#define CSL_FW_NAVSS256L_MODSS_MCRC0_S_CFG_NUM_PRIV_IDS_PER_REGION    3
#define CSL_FW_NAVSS256L_MODSS_MCRC0_S_CFG_NUM_PROTECTED_HW_REGIONS   1
#define CSL_FW_NAVSS256L_MODSS_MCRC0_S_CFG_MCRC64_REGS_START          0x31f70000
#define CSL_FW_NAVSS256L_MODSS_MCRC0_S_CFG_MCRC64_REGS_END            0x31f70fff

/* Properties of firewall protecting slave endpoint: PROXY0.SRC*/
#define CSL_FW_NAVSS256L_MODSS_PROXY0_SRC_ID                          4396
#define CSL_FW_NAVSS256L_MODSS_PROXY0_SRC_TYPE                        CSL_FW_SECURITY
#define CSL_FW_NAVSS256L_MODSS_PROXY0_SRC_MMR_BASE                    0x4544b000
#define CSL_FW_NAVSS256L_MODSS_PROXY0_SRC_NUM_REGIONS                 1
#define CSL_FW_NAVSS256L_MODSS_PROXY0_SRC_NUM_PRIV_IDS_PER_REGION     3
#define CSL_FW_NAVSS256L_MODSS_PROXY0_SRC_NUM_PROTECTED_HW_REGIONS    1
#define CSL_FW_NAVSS256L_MODSS_PROXY0_SRC_TARGET0_DATA_START          0x33000000
#define CSL_FW_NAVSS256L_MODSS_PROXY0_SRC_TARGET0_DATA_END            0x3303ffff

/* Properties of firewall protecting slave endpoint: SEC_PROXY0.SRC*/
#define CSL_FW_NAVSS256L_MODSS_SEC_PROXY0_SRC_ID                      4408
#define CSL_FW_NAVSS256L_MODSS_SEC_PROXY0_SRC_TYPE                    CSL_FW_SECURITY
#define CSL_FW_NAVSS256L_MODSS_SEC_PROXY0_SRC_MMR_BASE                0x4544e000
#define CSL_FW_NAVSS256L_MODSS_SEC_PROXY0_SRC_NUM_REGIONS             1
#define CSL_FW_NAVSS256L_MODSS_SEC_PROXY0_SRC_NUM_PRIV_IDS_PER_REGION 3
#define CSL_FW_NAVSS256L_MODSS_SEC_PROXY0_SRC_NUM_PROTECTED_HW_REGIONS          1
#define CSL_FW_NAVSS256L_MODSS_SEC_PROXY0_SRC_TARGET_DATA_START       0x32c00000
#define CSL_FW_NAVSS256L_MODSS_SEC_PROXY0_SRC_TARGET_DATA_END         0x32cfffff

/* Properties of firewall protecting slave endpoint: MSRAM0.SLV*/
#define CSL_FW_NAVSS256L_MODSS_MSRAM0_SLV_ID                          4409
#define CSL_FW_NAVSS256L_MODSS_MSRAM0_SLV_TYPE                        CSL_FW_SECURITY
#define CSL_FW_NAVSS256L_MODSS_MSRAM0_SLV_MMR_BASE                    0x4544e400
#define CSL_FW_NAVSS256L_MODSS_MSRAM0_SLV_NUM_REGIONS                 1
#define CSL_FW_NAVSS256L_MODSS_MSRAM0_SLV_NUM_PRIV_IDS_PER_REGION     3
#define CSL_FW_NAVSS256L_MODSS_MSRAM0_SLV_NUM_PROTECTED_HW_REGIONS    1
#define CSL_FW_NAVSS256L_MODSS_MSRAM0_SLV_RAM_START                   0x30000000
#define CSL_FW_NAVSS256L_MODSS_MSRAM0_SLV_RAM_END                     0x3000ffff

/* Properties of firewall protecting slave endpoint: PVU0.SRC*/
#define CSL_FW_NAVSS256L_MODSS_PVU0_SRC_ID                            4410
#define CSL_FW_NAVSS256L_MODSS_PVU0_SRC_TYPE                          CSL_FW_SECURITY
#define CSL_FW_NAVSS256L_MODSS_PVU0_SRC_MMR_BASE                      0x4544e800
#define CSL_FW_NAVSS256L_MODSS_PVU0_SRC_NUM_REGIONS                   1
#define CSL_FW_NAVSS256L_MODSS_PVU0_SRC_NUM_PRIV_IDS_PER_REGION       3
#define CSL_FW_NAVSS256L_MODSS_PVU0_SRC_NUM_PROTECTED_HW_REGIONS      1
#define CSL_FW_NAVSS256L_MODSS_PVU0_SRC_PVU_START                     0xunknown
#define CSL_FW_NAVSS256L_MODSS_PVU0_SRC_PVU_END                       0xunknown

/* Properties of firewall protecting slave endpoint: PVU1.SRC*/
#define CSL_FW_NAVSS256L_MODSS_PVU1_SRC_ID                            4411
#define CSL_FW_NAVSS256L_MODSS_PVU1_SRC_TYPE                          CSL_FW_SECURITY
#define CSL_FW_NAVSS256L_MODSS_PVU1_SRC_MMR_BASE                      0x4544ec00
#define CSL_FW_NAVSS256L_MODSS_PVU1_SRC_NUM_REGIONS                   1
#define CSL_FW_NAVSS256L_MODSS_PVU1_SRC_NUM_PRIV_IDS_PER_REGION       3
#define CSL_FW_NAVSS256L_MODSS_PVU1_SRC_NUM_PROTECTED_HW_REGIONS      1
#define CSL_FW_NAVSS256L_MODSS_PVU1_SRC_PVU_START                     0xunknown
#define CSL_FW_NAVSS256L_MODSS_PVU1_SRC_PVU_END                       0xunknown

/* Properties of firewall protecting slave endpoint: ^+UDMASS.RINGACC0_SRC*/
#define CSL_FW_NAVSS256L_MODSS_RINGACC0_SRC_ID                        4448
#define CSL_FW_NAVSS256L_MODSS_RINGACC0_SRC_TYPE                      CSL_FW_SECURITY
#define CSL_FW_NAVSS256L_MODSS_RINGACC0_SRC_MMR_BASE                  0x45458000
#define CSL_FW_NAVSS256L_MODSS_RINGACC0_SRC_NUM_REGIONS               1
#define CSL_FW_NAVSS256L_MODSS_RINGACC0_SRC_NUM_PRIV_IDS_PER_REGION   3
#define CSL_FW_NAVSS256L_MODSS_RINGACC0_SRC_NUM_PROTECTED_HW_REGIONS  1
#define CSL_FW_NAVSS256L_MODSS_RINGACC0_SRC_UDMASS_RINGACC0_SRC_FIFOS_START     0x38000000
#define CSL_FW_NAVSS256L_MODSS_RINGACC0_SRC_UDMASS_RINGACC0_SRC_FIFOS_END       0x383fffff

/* Properties of firewall protecting slave endpoint: ^+NBSS.MSMC0_MST_INT*/
#define CSL_FW_NAVSS256L_MODSS_MSMC0MST_INT_ID                        4449
#define CSL_FW_NAVSS256L_MODSS_MSMC0MST_INT_TYPE                      CSL_FW_SECURITY
#define CSL_FW_NAVSS256L_MODSS_MSMC0MST_INT_MMR_BASE                  0x45458400
#define CSL_FW_NAVSS256L_MODSS_MSMC0MST_INT_NUM_REGIONS               24
#define CSL_FW_NAVSS256L_MODSS_MSMC0MST_INT_NUM_PRIV_IDS_PER_REGION   3
#define CSL_FW_NAVSS256L_MODSS_MSMC0MST_INT_NUM_PROTECTED_HW_REGIONS  2
#define CSL_FW_NAVSS256L_MODSS_MSMC0MST_INT_NB0_MSMC0_MST_INT_MEM_START         0x60000000
#define CSL_FW_NAVSS256L_MODSS_MSMC0MST_INT_NB0_MSMC0_MST_INT_MEM_END 0x7fffffff
#define CSL_FW_NAVSS256L_MODSS_MSMC0MST_INT_NB0_MSMC0_MST_INT_MEM1_START        0xunknown
#define CSL_FW_NAVSS256L_MODSS_MSMC0MST_INT_NB0_MSMC0_MST_INT_MEM1_END          0xunknown

/* Properties of firewall protecting slave endpoint: ^+NBSS.MSMC1_DDRLO_INT*/
#define CSL_FW_NAVSS256L_MODSS_MSMC1DDRLO_INT_ID                      4450
#define CSL_FW_NAVSS256L_MODSS_MSMC1DDRLO_INT_TYPE                    CSL_FW_SECURITY
#define CSL_FW_NAVSS256L_MODSS_MSMC1DDRLO_INT_MMR_BASE                0x45458800
#define CSL_FW_NAVSS256L_MODSS_MSMC1DDRLO_INT_NUM_REGIONS             24
#define CSL_FW_NAVSS256L_MODSS_MSMC1DDRLO_INT_NUM_PRIV_IDS_PER_REGION 3
#define CSL_FW_NAVSS256L_MODSS_MSMC1DDRLO_INT_NUM_PROTECTED_HW_REGIONS          2
#define CSL_FW_NAVSS256L_MODSS_MSMC1DDRLO_INT_NB1_MSMC1_DDRLO_INT_MEM_START     0x80000000
#define CSL_FW_NAVSS256L_MODSS_MSMC1DDRLO_INT_NB1_MSMC1_DDRLO_INT_MEM_END       0xffffffff
#define CSL_FW_NAVSS256L_MODSS_MSMC1DDRLO_INT_NB1_MSMC1_DDRLO_INT_MEM1_START    0x800000000
#define CSL_FW_NAVSS256L_MODSS_MSMC1DDRLO_INT_NB1_MSMC1_DDRLO_INT_MEM1_END      0xfffffffff

/* Properties of firewall protecting slave endpoint: ^+NBSS.MSMC1_DDRHI_INT*/
#define CSL_FW_NAVSS256L_MODSS_MSMC1DDRHI_INT_ID                      4451
#define CSL_FW_NAVSS256L_MODSS_MSMC1DDRHI_INT_TYPE                    CSL_FW_SECURITY
#define CSL_FW_NAVSS256L_MODSS_MSMC1DDRHI_INT_MMR_BASE                0x45458c00
#define CSL_FW_NAVSS256L_MODSS_MSMC1DDRHI_INT_NUM_REGIONS             24
#define CSL_FW_NAVSS256L_MODSS_MSMC1DDRHI_INT_NUM_PRIV_IDS_PER_REGION 3
#define CSL_FW_NAVSS256L_MODSS_MSMC1DDRHI_INT_NUM_PROTECTED_HW_REGIONS          2
#define CSL_FW_NAVSS256L_MODSS_MSMC1DDRHI_INT_NB1_MSMC1_DDRHI_INT_MEM_START     0x80000000
#define CSL_FW_NAVSS256L_MODSS_MSMC1DDRHI_INT_NB1_MSMC1_DDRHI_INT_MEM_END       0xffffffff
#define CSL_FW_NAVSS256L_MODSS_MSMC1DDRHI_INT_NB1_MSMC1_DDRHI_INT_MEM1_START    0x800000000
#define CSL_FW_NAVSS256L_MODSS_MSMC1DDRHI_INT_NB1_MSMC1_DDRHI_INT_MEM1_END      0xfffffffff

/* Properties of firewall protecting slave endpoint: .ECCAGGR0*/
#define CSL_FW_NAVSS256L_NBSS_NAVSS256L_NBSS_CBASS_ECCAGGR0_ID        6112
#define CSL_FW_NAVSS256L_NBSS_NAVSS256L_NBSS_CBASS_ECCAGGR0_TYPE      CSL_FW_SECURITY
#define CSL_FW_NAVSS256L_NBSS_NAVSS256L_NBSS_CBASS_ECCAGGR0_MMR_BASE  0x455f8000
#define CSL_FW_NAVSS256L_NBSS_NAVSS256L_NBSS_CBASS_ECCAGGR0_NUM_REGIONS         1
#define CSL_FW_NAVSS256L_NBSS_NAVSS256L_NBSS_CBASS_ECCAGGR0_NUM_PRIV_IDS_PER_REGION       3
#define CSL_FW_NAVSS256L_NBSS_NAVSS256L_NBSS_CBASS_ECCAGGR0_NUM_PROTECTED_HW_REGIONS      1
#define CSL_FW_NAVSS256L_NBSS_NAVSS256L_NBSS_CBASS_ECCAGGR0_REGS_START          0x1000
#define CSL_FW_NAVSS256L_NBSS_NAVSS256L_NBSS_CBASS_ECCAGGR0_REGS_END  0x13ff

/* Properties of firewall protecting slave endpoint: NB0.CFG*/
#define CSL_FW_NAVSS256L_NBSS_NB0_CFG_ID                              6113
#define CSL_FW_NAVSS256L_NBSS_NB0_CFG_TYPE                            CSL_FW_SECURITY
#define CSL_FW_NAVSS256L_NBSS_NB0_CFG_MMR_BASE                        0x455f8400
#define CSL_FW_NAVSS256L_NBSS_NB0_CFG_NUM_REGIONS                     3
#define CSL_FW_NAVSS256L_NBSS_NB0_CFG_NUM_PRIV_IDS_PER_REGION         3
#define CSL_FW_NAVSS256L_NBSS_NB0_CFG_NUM_PROTECTED_HW_REGIONS        3
#define CSL_FW_NAVSS256L_NBSS_NB0_CFG_CFG__CFG__MMRS_START            0x2000
#define CSL_FW_NAVSS256L_NBSS_NB0_CFG_CFG__CFG__MMRS_END              0x20ff
#define CSL_FW_NAVSS256L_NBSS_NB0_CFG_MEM_ATTR0__SLV_SF__CFG_START    0x20000
#define CSL_FW_NAVSS256L_NBSS_NB0_CFG_MEM_ATTR0__SLV_SF__CFG_END      0x27fff
#define CSL_FW_NAVSS256L_NBSS_NB0_CFG_MEM_ATTR1__SLV_SF__CFG_START    0x28000
#define CSL_FW_NAVSS256L_NBSS_NB0_CFG_MEM_ATTR1__SLV_SF__CFG_END      0x2ffff

/* Properties of firewall protecting slave endpoint: NB1.CFG*/
#define CSL_FW_NAVSS256L_NBSS_NB1_CFG_ID                              6114
#define CSL_FW_NAVSS256L_NBSS_NB1_CFG_TYPE                            CSL_FW_SECURITY
#define CSL_FW_NAVSS256L_NBSS_NB1_CFG_MMR_BASE                        0x455f8800
#define CSL_FW_NAVSS256L_NBSS_NB1_CFG_NUM_REGIONS                     3
#define CSL_FW_NAVSS256L_NBSS_NB1_CFG_NUM_PRIV_IDS_PER_REGION         3
#define CSL_FW_NAVSS256L_NBSS_NB1_CFG_NUM_PROTECTED_HW_REGIONS        3
#define CSL_FW_NAVSS256L_NBSS_NB1_CFG_CFG__CFG__MMRS_START            0x3000
#define CSL_FW_NAVSS256L_NBSS_NB1_CFG_CFG__CFG__MMRS_END              0x30ff
#define CSL_FW_NAVSS256L_NBSS_NB1_CFG_MEM_ATTR0__SLV_SF__CFG_START    0x40000
#define CSL_FW_NAVSS256L_NBSS_NB1_CFG_MEM_ATTR0__SLV_SF__CFG_END      0x4ffff
#define CSL_FW_NAVSS256L_NBSS_NB1_CFG_MEM_ATTR1__SLV_SF__CFG_START    0x50000
#define CSL_FW_NAVSS256L_NBSS_NB1_CFG_MEM_ATTR1__SLV_SF__CFG_END      0x5ffff

/* Properties of firewall protecting slave endpoint: PROXY0.CFG*/
#define CSL_FW_NAVSS_MCU_MODSS_PROXY0_CFG_ID                          6148
#define CSL_FW_NAVSS_MCU_MODSS_PROXY0_CFG_TYPE                        CSL_FW_SECURITY
#define CSL_FW_NAVSS_MCU_MODSS_PROXY0_CFG_MMR_BASE                    0x45601000
#define CSL_FW_NAVSS_MCU_MODSS_PROXY0_CFG_NUM_REGIONS                 3
#define CSL_FW_NAVSS_MCU_MODSS_PROXY0_CFG_NUM_PRIV_IDS_PER_REGION     3
#define CSL_FW_NAVSS_MCU_MODSS_PROXY0_CFG_NUM_PROTECTED_HW_REGIONS    3
#define CSL_FW_NAVSS_MCU_MODSS_PROXY0_CFG_BUF__CFG__GCFG_START        0x28590000
#define CSL_FW_NAVSS_MCU_MODSS_PROXY0_CFG_BUF__CFG__GCFG_END          0x285900ff
#define CSL_FW_NAVSS_MCU_MODSS_PROXY0_CFG_BUF__CFG__CFG_START         0x2a580000
#define CSL_FW_NAVSS_MCU_MODSS_PROXY0_CFG_BUF__CFG__CFG_END           0x2a5bffff
#define CSL_FW_NAVSS_MCU_MODSS_PROXY0_CFG_BUF__BUFRAM_SLV__RAM_START  0x285a0000
#define CSL_FW_NAVSS_MCU_MODSS_PROXY0_CFG_BUF__BUFRAM_SLV__RAM_END    0x285a3fff

/* Properties of firewall protecting slave endpoint: SEC_PROXY0.CFG*/
#define CSL_FW_NAVSS_MCU_MODSS_SEC_PROXY0_CFG_ID                      6156
#define CSL_FW_NAVSS_MCU_MODSS_SEC_PROXY0_CFG_TYPE                    CSL_FW_SECURITY
#define CSL_FW_NAVSS_MCU_MODSS_SEC_PROXY0_CFG_MMR_BASE                0x45603000
#define CSL_FW_NAVSS_MCU_MODSS_SEC_PROXY0_CFG_NUM_REGIONS             3
#define CSL_FW_NAVSS_MCU_MODSS_SEC_PROXY0_CFG_NUM_PRIV_IDS_PER_REGION 3
#define CSL_FW_NAVSS_MCU_MODSS_SEC_PROXY0_CFG_NUM_PROTECTED_HW_REGIONS          3
#define CSL_FW_NAVSS_MCU_MODSS_SEC_PROXY0_CFG_MMRS_START              0x285b0000
#define CSL_FW_NAVSS_MCU_MODSS_SEC_PROXY0_CFG_MMRS_END                0x285b00ff
#define CSL_FW_NAVSS_MCU_MODSS_SEC_PROXY0_CFG_SCFG_START              0x2a400000
#define CSL_FW_NAVSS_MCU_MODSS_SEC_PROXY0_CFG_SCFG_END                0x2a47ffff
#define CSL_FW_NAVSS_MCU_MODSS_SEC_PROXY0_CFG_RT_START                0x2a380000
#define CSL_FW_NAVSS_MCU_MODSS_SEC_PROXY0_CFG_RT_END                  0x2a3fffff

/* Properties of firewall protecting slave endpoint: ^+UDMASS.RINGACC0_CFG*/
#define CSL_FW_NAVSS_MCU_MODSS_RINGACC0_CFG_ID                        6176
#define CSL_FW_NAVSS_MCU_MODSS_RINGACC0_CFG_TYPE                      CSL_FW_SECURITY
#define CSL_FW_NAVSS_MCU_MODSS_RINGACC0_CFG_MMR_BASE                  0x45608000
#define CSL_FW_NAVSS_MCU_MODSS_RINGACC0_CFG_NUM_REGIONS               4
#define CSL_FW_NAVSS_MCU_MODSS_RINGACC0_CFG_NUM_PRIV_IDS_PER_REGION   3
#define CSL_FW_NAVSS_MCU_MODSS_RINGACC0_CFG_NUM_PROTECTED_HW_REGIONS  4
#define CSL_FW_NAVSS_MCU_MODSS_RINGACC0_CFG_UDMASS_RINGACC0_GCFG_START          0x285d0000
#define CSL_FW_NAVSS_MCU_MODSS_RINGACC0_CFG_UDMASS_RINGACC0_GCFG_END  0x285d03ff
#define CSL_FW_NAVSS_MCU_MODSS_RINGACC0_CFG_UDMASS_RINGACC0_CFG_START 0x28440000
#define CSL_FW_NAVSS_MCU_MODSS_RINGACC0_CFG_UDMASS_RINGACC0_CFG_END   0x2847ffff
#define CSL_FW_NAVSS_MCU_MODSS_RINGACC0_CFG_UDMASS_RINGACC0_RT_START  0x2b800000
#define CSL_FW_NAVSS_MCU_MODSS_RINGACC0_CFG_UDMASS_RINGACC0_RT_END    0x2bbfffff
#define CSL_FW_NAVSS_MCU_MODSS_RINGACC0_CFG_UDMASS_RINGACC0_MON_START 0x2a280000
#define CSL_FW_NAVSS_MCU_MODSS_RINGACC0_CFG_UDMASS_RINGACC0_MON_END   0x2a29ffff

/* Properties of firewall protecting slave endpoint: ^+UDMASS.UDMASS_INTA0_CFG*/
#define CSL_FW_NAVSS_MCU_MODSS_UDMASSINTA0_CFG_ID                     6240
#define CSL_FW_NAVSS_MCU_MODSS_UDMASSINTA0_CFG_TYPE                   CSL_FW_SECURITY
#define CSL_FW_NAVSS_MCU_MODSS_UDMASSINTA0_CFG_MMR_BASE               0x45618000
#define CSL_FW_NAVSS_MCU_MODSS_UDMASSINTA0_CFG_NUM_REGIONS            7
#define CSL_FW_NAVSS_MCU_MODSS_UDMASSINTA0_CFG_NUM_PRIV_IDS_PER_REGION          3
#define CSL_FW_NAVSS_MCU_MODSS_UDMASSINTA0_CFG_NUM_PROTECTED_HW_REGIONS         7
#define CSL_FW_NAVSS_MCU_MODSS_UDMASSINTA0_CFG_UDMASS_INTA0_CFG_START 0x283c0000
#define CSL_FW_NAVSS_MCU_MODSS_UDMASSINTA0_CFG_UDMASS_INTA0_CFG_END   0x283c001f
#define CSL_FW_NAVSS_MCU_MODSS_UDMASSINTA0_CFG_UDMASS_INTA0_IMAP_START          0x28560000
#define CSL_FW_NAVSS_MCU_MODSS_UDMASSINTA0_CFG_UDMASS_INTA0_IMAP_END  0x2856ffff
#define CSL_FW_NAVSS_MCU_MODSS_UDMASSINTA0_CFG_UDMASS_INTA0_INTR_START          0x2a700000
#define CSL_FW_NAVSS_MCU_MODSS_UDMASSINTA0_CFG_UDMASS_INTA0_INTR_END  0x2a7fffff
#define CSL_FW_NAVSS_MCU_MODSS_UDMASSINTA0_CFG_UDMASS_INTA0_L2G_START 0x28570000
#define CSL_FW_NAVSS_MCU_MODSS_UDMASSINTA0_CFG_UDMASS_INTA0_L2G_END   0x2857007f
#define CSL_FW_NAVSS_MCU_MODSS_UDMASSINTA0_CFG_UDMASS_INTA0_MCAST_START         0x28580000
#define CSL_FW_NAVSS_MCU_MODSS_UDMASSINTA0_CFG_UDMASS_INTA0_MCAST_END 0x28580fff
#define CSL_FW_NAVSS_MCU_MODSS_UDMASSINTA0_CFG_UDMASS_INTA0_GCNTCFG_START       0x28480000
#define CSL_FW_NAVSS_MCU_MODSS_UDMASSINTA0_CFG_UDMASS_INTA0_GCNTCFG_END         0x28481fff
#define CSL_FW_NAVSS_MCU_MODSS_UDMASSINTA0_CFG_UDMASS_INTA0_GCNTRTI_START       0x2a600000
#define CSL_FW_NAVSS_MCU_MODSS_UDMASSINTA0_CFG_UDMASS_INTA0_GCNTRTI_END         0x2a6fffff

/* Properties of firewall protecting slave endpoint: ^+UDMASS.UDMAP0_CFG*/
#define CSL_FW_NAVSS_MCU_MODSS_UDMAP0_CFG_ID                          6248
#define CSL_FW_NAVSS_MCU_MODSS_UDMAP0_CFG_TYPE                        CSL_FW_SECURITY
#define CSL_FW_NAVSS_MCU_MODSS_UDMAP0_CFG_MMR_BASE                    0x4561a000
#define CSL_FW_NAVSS_MCU_MODSS_UDMAP0_CFG_NUM_REGIONS                 6
#define CSL_FW_NAVSS_MCU_MODSS_UDMAP0_CFG_NUM_PRIV_IDS_PER_REGION     3
#define CSL_FW_NAVSS_MCU_MODSS_UDMAP0_CFG_NUM_PROTECTED_HW_REGIONS    6
#define CSL_FW_NAVSS_MCU_MODSS_UDMAP0_CFG_UDMASS_UDMAP0_GCFG_START    0x285c0000
#define CSL_FW_NAVSS_MCU_MODSS_UDMAP0_CFG_UDMASS_UDMAP0_GCFG_END      0x285c00ff
#define CSL_FW_NAVSS_MCU_MODSS_UDMAP0_CFG_UDMASS_UDMAP0_RFLOW_START   0x28400000
#define CSL_FW_NAVSS_MCU_MODSS_UDMAP0_CFG_UDMASS_UDMAP0_RFLOW_END     0x28401fff
#define CSL_FW_NAVSS_MCU_MODSS_UDMAP0_CFG_UDMASS_UDMAP0_TCHAN_START   0x284a0000
#define CSL_FW_NAVSS_MCU_MODSS_UDMAP0_CFG_UDMASS_UDMAP0_TCHAN_END     0x284a3fff
#define CSL_FW_NAVSS_MCU_MODSS_UDMAP0_CFG_UDMASS_UDMAP0_RCHAN_START   0x284c0000
#define CSL_FW_NAVSS_MCU_MODSS_UDMAP0_CFG_UDMASS_UDMAP0_RCHAN_END     0x284c3fff
#define CSL_FW_NAVSS_MCU_MODSS_UDMAP0_CFG_UDMASS_UDMAP0_TCHANRT_START 0x2aa00000
#define CSL_FW_NAVSS_MCU_MODSS_UDMAP0_CFG_UDMASS_UDMAP0_TCHANRT_END   0x2aa3ffff
#define CSL_FW_NAVSS_MCU_MODSS_UDMAP0_CFG_UDMASS_UDMAP0_RCHANRT_START 0x2a800000
#define CSL_FW_NAVSS_MCU_MODSS_UDMAP0_CFG_UDMASS_UDMAP0_RCHANRT_END   0x2a83ffff

/* Properties of firewall protecting slave endpoint: .ECCAGGR0*/
#define CSL_FW_NAVSS_MCU_MODSS_NAVSS_MCU_MODSS_CBASS_ECCAGGR0_ID      6249
#define CSL_FW_NAVSS_MCU_MODSS_NAVSS_MCU_MODSS_CBASS_ECCAGGR0_TYPE    CSL_FW_SECURITY
#define CSL_FW_NAVSS_MCU_MODSS_NAVSS_MCU_MODSS_CBASS_ECCAGGR0_MMR_BASE          0x4561a400
#define CSL_FW_NAVSS_MCU_MODSS_NAVSS_MCU_MODSS_CBASS_ECCAGGR0_NUM_REGIONS       1
#define CSL_FW_NAVSS_MCU_MODSS_NAVSS_MCU_MODSS_CBASS_ECCAGGR0_NUM_PRIV_IDS_PER_REGION     3
#define CSL_FW_NAVSS_MCU_MODSS_NAVSS_MCU_MODSS_CBASS_ECCAGGR0_NUM_PROTECTED_HW_REGIONS    1
#define CSL_FW_NAVSS_MCU_MODSS_NAVSS_MCU_MODSS_CBASS_ECCAGGR0_REGS_START        0x28380000
#define CSL_FW_NAVSS_MCU_MODSS_NAVSS_MCU_MODSS_CBASS_ECCAGGR0_REGS_END          0x283803ff

/* Properties of firewall protecting slave endpoint: ^+UDMASS.UDMASS_ECCAGGR0_CFG*/
#define CSL_FW_NAVSS_MCU_MODSS_UDMASSECCAGGR0_CFG_ID                  6250
#define CSL_FW_NAVSS_MCU_MODSS_UDMASSECCAGGR0_CFG_TYPE                CSL_FW_SECURITY
#define CSL_FW_NAVSS_MCU_MODSS_UDMASSECCAGGR0_CFG_MMR_BASE            0x4561a800
#define CSL_FW_NAVSS_MCU_MODSS_UDMASSECCAGGR0_CFG_NUM_REGIONS         1
#define CSL_FW_NAVSS_MCU_MODSS_UDMASSECCAGGR0_CFG_NUM_PRIV_IDS_PER_REGION       3
#define CSL_FW_NAVSS_MCU_MODSS_UDMASSECCAGGR0_CFG_NUM_PROTECTED_HW_REGIONS      1
#define CSL_FW_NAVSS_MCU_MODSS_UDMASSECCAGGR0_CFG_UDMASS_ECCAGGR0_REGS_START    0x28381000
#define CSL_FW_NAVSS_MCU_MODSS_UDMASSECCAGGR0_CFG_UDMASS_ECCAGGR0_REGS_END      0x283813ff

/* Properties of firewall protecting slave endpoint: INTR0.CFG*/
#define CSL_FW_NAVSS_MCU_MODSS_INTR0_CFG_ID                           6251
#define CSL_FW_NAVSS_MCU_MODSS_INTR0_CFG_TYPE                         CSL_FW_SECURITY
#define CSL_FW_NAVSS_MCU_MODSS_INTR0_CFG_MMR_BASE                     0x4561ac00
#define CSL_FW_NAVSS_MCU_MODSS_INTR0_CFG_NUM_REGIONS                  1
#define CSL_FW_NAVSS_MCU_MODSS_INTR0_CFG_NUM_PRIV_IDS_PER_REGION      3
#define CSL_FW_NAVSS_MCU_MODSS_INTR0_CFG_NUM_PROTECTED_HW_REGIONS     1
#define CSL_FW_NAVSS_MCU_MODSS_INTR0_CFG_INTR_ROUTER_CFG_START        0x28540000
#define CSL_FW_NAVSS_MCU_MODSS_INTR0_CFG_INTR_ROUTER_CFG_END          0x285407ff

/* Properties of firewall protecting slave endpoint: ^+UDMASS.PSILCFG0_CFG*/
#define CSL_FW_NAVSS_MCU_MODSS_PSILCFG0_CFG_ID                        6252
#define CSL_FW_NAVSS_MCU_MODSS_PSILCFG0_CFG_TYPE                      CSL_FW_SECURITY
#define CSL_FW_NAVSS_MCU_MODSS_PSILCFG0_CFG_MMR_BASE                  0x4561b000
#define CSL_FW_NAVSS_MCU_MODSS_PSILCFG0_CFG_NUM_REGIONS               1
#define CSL_FW_NAVSS_MCU_MODSS_PSILCFG0_CFG_NUM_PRIV_IDS_PER_REGION   3
#define CSL_FW_NAVSS_MCU_MODSS_PSILCFG0_CFG_NUM_PROTECTED_HW_REGIONS  1
#define CSL_FW_NAVSS_MCU_MODSS_PSILCFG0_CFG_UDMASS_PSILCFG0_PROXY_START         0x2a268000
#define CSL_FW_NAVSS_MCU_MODSS_PSILCFG0_CFG_UDMASS_PSILCFG0_PROXY_END 0x2a2681ff

/* Properties of firewall protecting slave endpoint: ^+UDMASS.PSILSS0_CFG*/
#define CSL_FW_NAVSS_MCU_MODSS_PSILSS0_CFG_ID                         6253
#define CSL_FW_NAVSS_MCU_MODSS_PSILSS0_CFG_TYPE                       CSL_FW_SECURITY
#define CSL_FW_NAVSS_MCU_MODSS_PSILSS0_CFG_MMR_BASE                   0x4561b400
#define CSL_FW_NAVSS_MCU_MODSS_PSILSS0_CFG_NUM_REGIONS                1
#define CSL_FW_NAVSS_MCU_MODSS_PSILSS0_CFG_NUM_PRIV_IDS_PER_REGION    3
#define CSL_FW_NAVSS_MCU_MODSS_PSILSS0_CFG_NUM_PROTECTED_HW_REGIONS   1
#define CSL_FW_NAVSS_MCU_MODSS_PSILSS0_CFG_UDMASS_PSILSS0_MMRS_START  0x285e0000
#define CSL_FW_NAVSS_MCU_MODSS_PSILSS0_CFG_UDMASS_PSILSS0_MMRS_END    0x285e0fff

/* Properties of firewall protecting slave endpoint: MCRC0.S_CFG*/
#define CSL_FW_NAVSS_MCU_MODSS_MCRC0_S_CFG_ID                         6254
#define CSL_FW_NAVSS_MCU_MODSS_MCRC0_S_CFG_TYPE                       CSL_FW_SECURITY
#define CSL_FW_NAVSS_MCU_MODSS_MCRC0_S_CFG_MMR_BASE                   0x4561b800
#define CSL_FW_NAVSS_MCU_MODSS_MCRC0_S_CFG_NUM_REGIONS                1
#define CSL_FW_NAVSS_MCU_MODSS_MCRC0_S_CFG_NUM_PRIV_IDS_PER_REGION    3
#define CSL_FW_NAVSS_MCU_MODSS_MCRC0_S_CFG_NUM_PROTECTED_HW_REGIONS   1
#define CSL_FW_NAVSS_MCU_MODSS_MCRC0_S_CFG_MCRC64_REGS_START          0x2a264000
#define CSL_FW_NAVSS_MCU_MODSS_MCRC0_S_CFG_MCRC64_REGS_END            0x2a264fff

/* Properties of firewall protecting slave endpoint: PROXY0.SRC*/
#define CSL_FW_NAVSS_MCU_MODSS_PROXY0_SRC_ID                          6260
#define CSL_FW_NAVSS_MCU_MODSS_PROXY0_SRC_TYPE                        CSL_FW_SECURITY
#define CSL_FW_NAVSS_MCU_MODSS_PROXY0_SRC_MMR_BASE                    0x4561d000
#define CSL_FW_NAVSS_MCU_MODSS_PROXY0_SRC_NUM_REGIONS                 1
#define CSL_FW_NAVSS_MCU_MODSS_PROXY0_SRC_NUM_PRIV_IDS_PER_REGION     3
#define CSL_FW_NAVSS_MCU_MODSS_PROXY0_SRC_NUM_PROTECTED_HW_REGIONS    1
#define CSL_FW_NAVSS_MCU_MODSS_PROXY0_SRC_TARGET0_DATA_START          0x2a500000
#define CSL_FW_NAVSS_MCU_MODSS_PROXY0_SRC_TARGET0_DATA_END            0x2a53ffff

/* Properties of firewall protecting slave endpoint: SEC_PROXY0.SRC*/
#define CSL_FW_NAVSS_MCU_MODSS_SEC_PROXY0_SRC_ID                      6268
#define CSL_FW_NAVSS_MCU_MODSS_SEC_PROXY0_SRC_TYPE                    CSL_FW_SECURITY
#define CSL_FW_NAVSS_MCU_MODSS_SEC_PROXY0_SRC_MMR_BASE                0x4561f000
#define CSL_FW_NAVSS_MCU_MODSS_SEC_PROXY0_SRC_NUM_REGIONS             1
#define CSL_FW_NAVSS_MCU_MODSS_SEC_PROXY0_SRC_NUM_PRIV_IDS_PER_REGION 3
#define CSL_FW_NAVSS_MCU_MODSS_SEC_PROXY0_SRC_NUM_PROTECTED_HW_REGIONS          1
#define CSL_FW_NAVSS_MCU_MODSS_SEC_PROXY0_SRC_TARGET_DATA_START       0x2a480000
#define CSL_FW_NAVSS_MCU_MODSS_SEC_PROXY0_SRC_TARGET_DATA_END         0x2a4fffff

/* Properties of firewall protecting slave endpoint: MSRAM0.SLV*/
#define CSL_FW_NAVSS_MCU_MODSS_MSRAM0_SLV_ID                          6269
#define CSL_FW_NAVSS_MCU_MODSS_MSRAM0_SLV_TYPE                        CSL_FW_SECURITY
#define CSL_FW_NAVSS_MCU_MODSS_MSRAM0_SLV_MMR_BASE                    0x4561f400
#define CSL_FW_NAVSS_MCU_MODSS_MSRAM0_SLV_NUM_REGIONS                 1
#define CSL_FW_NAVSS_MCU_MODSS_MSRAM0_SLV_NUM_PRIV_IDS_PER_REGION     3
#define CSL_FW_NAVSS_MCU_MODSS_MSRAM0_SLV_NUM_PROTECTED_HW_REGIONS    1
#define CSL_FW_NAVSS_MCU_MODSS_MSRAM0_SLV_RAM_START                   0x28000000
#define CSL_FW_NAVSS_MCU_MODSS_MSRAM0_SLV_RAM_END                     0x28007fff

/* Properties of firewall protecting slave endpoint: MSRAM1.SLV*/
#define CSL_FW_NAVSS_MCU_MODSS_MSRAM1_SLV_ID                          6270
#define CSL_FW_NAVSS_MCU_MODSS_MSRAM1_SLV_TYPE                        CSL_FW_SECURITY
#define CSL_FW_NAVSS_MCU_MODSS_MSRAM1_SLV_MMR_BASE                    0x4561f800
#define CSL_FW_NAVSS_MCU_MODSS_MSRAM1_SLV_NUM_REGIONS                 1
#define CSL_FW_NAVSS_MCU_MODSS_MSRAM1_SLV_NUM_PRIV_IDS_PER_REGION     3
#define CSL_FW_NAVSS_MCU_MODSS_MSRAM1_SLV_NUM_PROTECTED_HW_REGIONS    1
#define CSL_FW_NAVSS_MCU_MODSS_MSRAM1_SLV_RAM_START                   0x28010000
#define CSL_FW_NAVSS_MCU_MODSS_MSRAM1_SLV_RAM_END                     0x28017fff

/* Properties of firewall protecting slave endpoint: ^+UDMASS.RINGACC0_SRC*/
#define CSL_FW_NAVSS_MCU_MODSS_RINGACC0_SRC_ID                        6288
#define CSL_FW_NAVSS_MCU_MODSS_RINGACC0_SRC_TYPE                      CSL_FW_SECURITY
#define CSL_FW_NAVSS_MCU_MODSS_RINGACC0_SRC_MMR_BASE                  0x45624000
#define CSL_FW_NAVSS_MCU_MODSS_RINGACC0_SRC_NUM_REGIONS               1
#define CSL_FW_NAVSS_MCU_MODSS_RINGACC0_SRC_NUM_PRIV_IDS_PER_REGION   3
#define CSL_FW_NAVSS_MCU_MODSS_RINGACC0_SRC_NUM_PROTECTED_HW_REGIONS  1
#define CSL_FW_NAVSS_MCU_MODSS_RINGACC0_SRC_UDMASS_RINGACC0_SRC_FIFOS_START     0x2b000000
#define CSL_FW_NAVSS_MCU_MODSS_RINGACC0_SRC_UDMASS_RINGACC0_SRC_FIFOS_END       0x2b3fffff
/* Standard Security Master-side Firewall Definitions */

/* Properties of master firewall ID: 257*/
#define CSL_FW_MST0_ID                                                257
#define CSL_FW_MST0_TYPE                                              CSL_FW_SECURITY
#define CSL_FW_MST0_MMR_BASE                                          0x45040400
#define CSL_FW_MST0_NUM_REGIONS                                       8
#define CSL_FW_MST0_NUM_PRIV_IDS_PER_REGION                           1
#define CSL_FW_MST0_NA_START                                          0x0
#define CSL_FW_MST0_NA_END                                            0xffffffffff

/* Properties of master firewall ID: 259*/
#define CSL_FW_MST1_ID                                                259
#define CSL_FW_MST1_TYPE                                              CSL_FW_SECURITY
#define CSL_FW_MST1_MMR_BASE                                          0x45040c00
#define CSL_FW_MST1_NUM_REGIONS                                       8
#define CSL_FW_MST1_NUM_PRIV_IDS_PER_REGION                           1
#define CSL_FW_MST1_NA_START                                          0x0
#define CSL_FW_MST1_NA_END                                            0xffffffffff

/* Properties of master firewall ID: 284*/
#define CSL_FW_MST2_ID                                                284
#define CSL_FW_MST2_TYPE                                              CSL_FW_SECURITY
#define CSL_FW_MST2_MMR_BASE                                          0x45047000
#define CSL_FW_MST2_NUM_REGIONS                                       16
#define CSL_FW_MST2_NUM_PRIV_IDS_PER_REGION                           3
#define CSL_FW_MST2_NA_START                                          0x0
#define CSL_FW_MST2_NA_END                                            0xffffffffff
/* Channelized Firewall Definitions */

/* Properties of channelized firewall : DRU_CHANNEL_CONFIG*/
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_ID                               304
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_TYPE                             CSL_FW_CHANNEL
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_MMR_BASE                         0x4504c000
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NUM_REGIONS                      1
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NUM_PRIV_IDS_PER_REGION          1
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_CH_RT_CFG_CH0_START              0x6d060000
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_CH_RT_CFG_CH0_END                0x6d0600ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_CH_RT_CFG_CH1_START              0x6d060100
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_CH_RT_CFG_CH1_END                0x6d0601ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_CH_RT_CFG_CH2_START              0x6d060200
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_CH_RT_CFG_CH2_END                0x6d0602ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_CH_RT_CFG_CH3_START              0x6d060300
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_CH_RT_CFG_CH3_END                0x6d0603ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_CH_RT_CFG_CH4_START              0x6d060400
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_CH_RT_CFG_CH4_END                0x6d0604ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_CH_RT_CFG_CH5_START              0x6d060500
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_CH_RT_CFG_CH5_END                0x6d0605ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_CH_RT_CFG_CH6_START              0x6d060600
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_CH_RT_CFG_CH6_END                0x6d0606ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_CH_RT_CFG_CH7_START              0x6d060700
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_CH_RT_CFG_CH7_END                0x6d0607ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_CH_RT_CFG_CH8_START              0x6d060800
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_CH_RT_CFG_CH8_END                0x6d0608ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_CH_RT_CFG_CH9_START              0x6d060900
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_CH_RT_CFG_CH9_END                0x6d0609ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_CH_RT_CFG_CH10_START             0x6d060a00
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_CH_RT_CFG_CH10_END               0x6d060aff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_CH_RT_CFG_CH11_START             0x6d060b00
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_CH_RT_CFG_CH11_END               0x6d060bff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_CH_RT_CFG_CH12_START             0x6d060c00
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_CH_RT_CFG_CH12_END               0x6d060cff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_CH_RT_CFG_CH13_START             0x6d060d00
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_CH_RT_CFG_CH13_END               0x6d060dff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_CH_RT_CFG_CH14_START             0x6d060e00
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_CH_RT_CFG_CH14_END               0x6d060eff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_CH_RT_CFG_CH15_START             0x6d060f00
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_CH_RT_CFG_CH15_END               0x6d060fff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_CH_RT_CFG_CH16_START             0x6d061000
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_CH_RT_CFG_CH16_END               0x6d0610ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_CH_RT_CFG_CH17_START             0x6d061100
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_CH_RT_CFG_CH17_END               0x6d0611ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_CH_RT_CFG_CH18_START             0x6d061200
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_CH_RT_CFG_CH18_END               0x6d0612ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_CH_RT_CFG_CH19_START             0x6d061300
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_CH_RT_CFG_CH19_END               0x6d0613ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_CH_RT_CFG_CH20_START             0x6d061400
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_CH_RT_CFG_CH20_END               0x6d0614ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_CH_RT_CFG_CH21_START             0x6d061500
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_CH_RT_CFG_CH21_END               0x6d0615ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_CH_RT_CFG_CH22_START             0x6d061600
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_CH_RT_CFG_CH22_END               0x6d0616ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_CH_RT_CFG_CH23_START             0x6d061700
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_CH_RT_CFG_CH23_END               0x6d0617ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_CH_RT_CFG_CH24_START             0x6d061800
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_CH_RT_CFG_CH24_END               0x6d0618ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_CH_RT_CFG_CH25_START             0x6d061900
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_CH_RT_CFG_CH25_END               0x6d0619ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_CH_RT_CFG_CH26_START             0x6d061a00
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_CH_RT_CFG_CH26_END               0x6d061aff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_CH_RT_CFG_CH27_START             0x6d061b00
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_CH_RT_CFG_CH27_END               0x6d061bff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_CH_RT_CFG_CH28_START             0x6d061c00
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_CH_RT_CFG_CH28_END               0x6d061cff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_CH_RT_CFG_CH29_START             0x6d061d00
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_CH_RT_CFG_CH29_END               0x6d061dff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_CH_RT_CFG_CH30_START             0x6d061e00
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_CH_RT_CFG_CH30_END               0x6d061eff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_CH_RT_CFG_CH31_START             0x6d061f00
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_CH_RT_CFG_CH31_END               0x6d061fff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_SUBMISSION_DBG_CH0_START         0x6d080000
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_SUBMISSION_DBG_CH0_END           0x6d0800ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_SUBMISSION_DBG_CH1_START         0x6d080100
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_SUBMISSION_DBG_CH1_END           0x6d0801ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_SUBMISSION_DBG_CH2_START         0x6d080200
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_SUBMISSION_DBG_CH2_END           0x6d0802ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_SUBMISSION_DBG_CH3_START         0x6d080300
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_SUBMISSION_DBG_CH3_END           0x6d0803ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_SUBMISSION_DBG_CH4_START         0x6d080400
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_SUBMISSION_DBG_CH4_END           0x6d0804ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_SUBMISSION_DBG_CH5_START         0x6d080500
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_SUBMISSION_DBG_CH5_END           0x6d0805ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_SUBMISSION_DBG_CH6_START         0x6d080600
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_SUBMISSION_DBG_CH6_END           0x6d0806ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_SUBMISSION_DBG_CH7_START         0x6d080700
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_SUBMISSION_DBG_CH7_END           0x6d0807ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_SUBMISSION_DBG_CH8_START         0x6d080800
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_SUBMISSION_DBG_CH8_END           0x6d0808ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_SUBMISSION_DBG_CH9_START         0x6d080900
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_SUBMISSION_DBG_CH9_END           0x6d0809ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_SUBMISSION_DBG_CH10_START        0x6d080a00
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_SUBMISSION_DBG_CH10_END          0x6d080aff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_SUBMISSION_DBG_CH11_START        0x6d080b00
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_SUBMISSION_DBG_CH11_END          0x6d080bff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_SUBMISSION_DBG_CH12_START        0x6d080c00
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_SUBMISSION_DBG_CH12_END          0x6d080cff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_SUBMISSION_DBG_CH13_START        0x6d080d00
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_SUBMISSION_DBG_CH13_END          0x6d080dff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_SUBMISSION_DBG_CH14_START        0x6d080e00
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_SUBMISSION_DBG_CH14_END          0x6d080eff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_SUBMISSION_DBG_CH15_START        0x6d080f00
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_SUBMISSION_DBG_CH15_END          0x6d080fff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_SUBMISSION_DBG_CH16_START        0x6d081000
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_SUBMISSION_DBG_CH16_END          0x6d0810ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_SUBMISSION_DBG_CH17_START        0x6d081100
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_SUBMISSION_DBG_CH17_END          0x6d0811ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_SUBMISSION_DBG_CH18_START        0x6d081200
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_SUBMISSION_DBG_CH18_END          0x6d0812ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_SUBMISSION_DBG_CH19_START        0x6d081300
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_SUBMISSION_DBG_CH19_END          0x6d0813ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_SUBMISSION_DBG_CH20_START        0x6d081400
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_SUBMISSION_DBG_CH20_END          0x6d0814ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_SUBMISSION_DBG_CH21_START        0x6d081500
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_SUBMISSION_DBG_CH21_END          0x6d0815ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_SUBMISSION_DBG_CH22_START        0x6d081600
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_SUBMISSION_DBG_CH22_END          0x6d0816ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_SUBMISSION_DBG_CH23_START        0x6d081700
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_SUBMISSION_DBG_CH23_END          0x6d0817ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_SUBMISSION_DBG_CH24_START        0x6d081800
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_SUBMISSION_DBG_CH24_END          0x6d0818ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_SUBMISSION_DBG_CH25_START        0x6d081900
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_SUBMISSION_DBG_CH25_END          0x6d0819ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_SUBMISSION_DBG_CH26_START        0x6d081a00
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_SUBMISSION_DBG_CH26_END          0x6d081aff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_SUBMISSION_DBG_CH27_START        0x6d081b00
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_SUBMISSION_DBG_CH27_END          0x6d081bff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_SUBMISSION_DBG_CH28_START        0x6d081c00
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_SUBMISSION_DBG_CH28_END          0x6d081cff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_SUBMISSION_DBG_CH29_START        0x6d081d00
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_SUBMISSION_DBG_CH29_END          0x6d081dff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_SUBMISSION_DBG_CH30_START        0x6d081e00
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_SUBMISSION_DBG_CH30_END          0x6d081eff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_SUBMISSION_DBG_CH31_START        0x6d081f00
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_SUBMISSION_DBG_CH31_END          0x6d081fff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_CH0_START          0x6d0a0000
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_CH0_END            0x6d0a00ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_CH1_START          0x6d0a0100
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_CH1_END            0x6d0a01ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_CH2_START          0x6d0a0200
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_CH2_END            0x6d0a02ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_CH3_START          0x6d0a0300
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_CH3_END            0x6d0a03ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_CH4_START          0x6d0a0400
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_CH4_END            0x6d0a04ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_CH5_START          0x6d0a0500
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_CH5_END            0x6d0a05ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_CH6_START          0x6d0a0600
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_CH6_END            0x6d0a06ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_CH7_START          0x6d0a0700
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_CH7_END            0x6d0a07ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_CH8_START          0x6d0a0800
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_CH8_END            0x6d0a08ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_CH9_START          0x6d0a0900
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_CH9_END            0x6d0a09ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_CH10_START         0x6d0a0a00
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_CH10_END           0x6d0a0aff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_CH11_START         0x6d0a0b00
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_CH11_END           0x6d0a0bff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_CH12_START         0x6d0a0c00
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_CH12_END           0x6d0a0cff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_CH13_START         0x6d0a0d00
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_CH13_END           0x6d0a0dff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_CH14_START         0x6d0a0e00
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_CH14_END           0x6d0a0eff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_CH15_START         0x6d0a0f00
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_CH15_END           0x6d0a0fff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_CH16_START         0x6d0a1000
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_CH16_END           0x6d0a10ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_CH17_START         0x6d0a1100
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_CH17_END           0x6d0a11ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_CH18_START         0x6d0a1200
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_CH18_END           0x6d0a12ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_CH19_START         0x6d0a1300
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_CH19_END           0x6d0a13ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_CH20_START         0x6d0a1400
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_CH20_END           0x6d0a14ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_CH21_START         0x6d0a1500
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_CH21_END           0x6d0a15ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_CH22_START         0x6d0a1600
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_CH22_END           0x6d0a16ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_CH23_START         0x6d0a1700
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_CH23_END           0x6d0a17ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_CH24_START         0x6d0a1800
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_CH24_END           0x6d0a18ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_CH25_START         0x6d0a1900
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_CH25_END           0x6d0a19ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_CH26_START         0x6d0a1a00
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_CH26_END           0x6d0a1aff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_CH27_START         0x6d0a1b00
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_CH27_END           0x6d0a1bff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_CH28_START         0x6d0a1c00
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_CH28_END           0x6d0a1cff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_CH29_START         0x6d0a1d00
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_CH29_END           0x6d0a1dff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_CH30_START         0x6d0a1e00
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_CH30_END           0x6d0a1eff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_CH31_START         0x6d0a1f00
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_CH31_END           0x6d0a1fff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_HI_CH0_START       0x6d0c0000
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_HI_CH0_END         0x6d0c00ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_HI_CH1_START       0x6d0c0100
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_HI_CH1_END         0x6d0c01ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_HI_CH2_START       0x6d0c0200
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_HI_CH2_END         0x6d0c02ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_HI_CH3_START       0x6d0c0300
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_HI_CH3_END         0x6d0c03ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_HI_CH4_START       0x6d0c0400
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_HI_CH4_END         0x6d0c04ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_HI_CH5_START       0x6d0c0500
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_HI_CH5_END         0x6d0c05ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_HI_CH6_START       0x6d0c0600
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_HI_CH6_END         0x6d0c06ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_HI_CH7_START       0x6d0c0700
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_HI_CH7_END         0x6d0c07ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_HI_CH8_START       0x6d0c0800
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_HI_CH8_END         0x6d0c08ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_HI_CH9_START       0x6d0c0900
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_HI_CH9_END         0x6d0c09ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_HI_CH10_START      0x6d0c0a00
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_HI_CH10_END        0x6d0c0aff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_HI_CH11_START      0x6d0c0b00
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_HI_CH11_END        0x6d0c0bff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_HI_CH12_START      0x6d0c0c00
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_HI_CH12_END        0x6d0c0cff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_HI_CH13_START      0x6d0c0d00
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_HI_CH13_END        0x6d0c0dff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_HI_CH14_START      0x6d0c0e00
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_HI_CH14_END        0x6d0c0eff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_HI_CH15_START      0x6d0c0f00
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_HI_CH15_END        0x6d0c0fff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_HI_CH16_START      0x6d0c1000
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_HI_CH16_END        0x6d0c10ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_HI_CH17_START      0x6d0c1100
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_HI_CH17_END        0x6d0c11ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_HI_CH18_START      0x6d0c1200
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_HI_CH18_END        0x6d0c12ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_HI_CH19_START      0x6d0c1300
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_HI_CH19_END        0x6d0c13ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_HI_CH20_START      0x6d0c1400
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_HI_CH20_END        0x6d0c14ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_HI_CH21_START      0x6d0c1500
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_HI_CH21_END        0x6d0c15ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_HI_CH22_START      0x6d0c1600
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_HI_CH22_END        0x6d0c16ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_HI_CH23_START      0x6d0c1700
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_HI_CH23_END        0x6d0c17ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_HI_CH24_START      0x6d0c1800
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_HI_CH24_END        0x6d0c18ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_HI_CH25_START      0x6d0c1900
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_HI_CH25_END        0x6d0c19ff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_HI_CH26_START      0x6d0c1a00
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_HI_CH26_END        0x6d0c1aff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_HI_CH27_START      0x6d0c1b00
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_HI_CH27_END        0x6d0c1bff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_HI_CH28_START      0x6d0c1c00
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_HI_CH28_END        0x6d0c1cff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_HI_CH29_START      0x6d0c1d00
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_HI_CH29_END        0x6d0c1dff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_HI_CH30_START      0x6d0c1e00
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_HI_CH30_END        0x6d0c1eff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_HI_CH31_START      0x6d0c1f00
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_NA_SUBMISSION_HI_CH31_END        0x6d0c1fff
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_TBD_OFFSET_FIRST_RES             0
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_TBD_NUM_CHANNELS                 32
#define CSL_CH_FW_DRU_CHANNEL_CONFIG_TBD_CHANNEL_SIZE                 256

/* Properties of channelized firewall : navss256l_modss_mailbox0_cfg*/
#define CSL_CH_FW_NAVSS256L_MODSS_MAILBOX0_CFG_ID                     4102
#define CSL_CH_FW_NAVSS256L_MODSS_MAILBOX0_CFG_TYPE                   CSL_FW_CHANNEL
#define CSL_CH_FW_NAVSS256L_MODSS_MAILBOX0_CFG_MMR_BASE               0x45401800
#define CSL_CH_FW_NAVSS256L_MODSS_MAILBOX0_CFG_NUM_REGIONS            12
#define CSL_CH_FW_NAVSS256L_MODSS_MAILBOX0_CFG_NUM_PRIV_IDS_PER_REGION          3
#define CSL_CH_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS0_START            0x31f80000
#define CSL_CH_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS0_END              0x31f801ff
#define CSL_CH_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS1_START            0x31f81000
#define CSL_CH_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS1_END              0x31f811ff
#define CSL_CH_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS2_START            0x31f82000
#define CSL_CH_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS2_END              0x31f821ff
#define CSL_CH_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS3_START            0x31f83000
#define CSL_CH_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS3_END              0x31f831ff
#define CSL_CH_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS4_START            0x31f84000
#define CSL_CH_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS4_END              0x31f841ff
#define CSL_CH_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS5_START            0x31f85000
#define CSL_CH_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS5_END              0x31f851ff
#define CSL_CH_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS6_START            0x31f86000
#define CSL_CH_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS6_END              0x31f861ff
#define CSL_CH_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS7_START            0x31f87000
#define CSL_CH_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS7_END              0x31f871ff
#define CSL_CH_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS8_START            0x31f88000
#define CSL_CH_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS8_END              0x31f881ff
#define CSL_CH_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS9_START            0x31f89000
#define CSL_CH_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS9_END              0x31f891ff
#define CSL_CH_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS10_START           0x31f8a000
#define CSL_CH_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS10_END             0x31f8a1ff
#define CSL_CH_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS11_START           0x31f8b000
#define CSL_CH_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS11_END             0x31f8b1ff
#define CSL_CH_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS0_OFFSET_FIRST_RES 0
#define CSL_CH_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS0_NUM_CHANNELS     1
#define CSL_CH_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS0_CHANNEL_SIZE     4096
#define CSL_CH_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS1_OFFSET_FIRST_RES 0
#define CSL_CH_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS1_NUM_CHANNELS     1
#define CSL_CH_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS1_CHANNEL_SIZE     4096
#define CSL_CH_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS2_OFFSET_FIRST_RES 0
#define CSL_CH_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS2_NUM_CHANNELS     1
#define CSL_CH_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS2_CHANNEL_SIZE     4096
#define CSL_CH_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS3_OFFSET_FIRST_RES 0
#define CSL_CH_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS3_NUM_CHANNELS     1
#define CSL_CH_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS3_CHANNEL_SIZE     4096
#define CSL_CH_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS4_OFFSET_FIRST_RES 0
#define CSL_CH_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS4_NUM_CHANNELS     1
#define CSL_CH_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS4_CHANNEL_SIZE     4096
#define CSL_CH_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS5_OFFSET_FIRST_RES 0
#define CSL_CH_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS5_NUM_CHANNELS     1
#define CSL_CH_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS5_CHANNEL_SIZE     4096
#define CSL_CH_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS6_OFFSET_FIRST_RES 0
#define CSL_CH_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS6_NUM_CHANNELS     1
#define CSL_CH_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS6_CHANNEL_SIZE     4096
#define CSL_CH_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS7_OFFSET_FIRST_RES 0
#define CSL_CH_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS7_NUM_CHANNELS     1
#define CSL_CH_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS7_CHANNEL_SIZE     4096
#define CSL_CH_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS8_OFFSET_FIRST_RES 0
#define CSL_CH_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS8_NUM_CHANNELS     1
#define CSL_CH_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS8_CHANNEL_SIZE     4096
#define CSL_CH_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS9_OFFSET_FIRST_RES 0
#define CSL_CH_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS9_NUM_CHANNELS     1
#define CSL_CH_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS9_CHANNEL_SIZE     4096
#define CSL_CH_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS10_OFFSET_FIRST_RES          0
#define CSL_CH_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS10_NUM_CHANNELS    1
#define CSL_CH_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS10_CHANNEL_SIZE    4096
#define CSL_CH_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS11_OFFSET_FIRST_RES          0
#define CSL_CH_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS11_NUM_CHANNELS    1
#define CSL_CH_FW_NAVSS256L_MODSS_MAILBOX0_CFG_REGS11_CHANNEL_SIZE    4096

/* Properties of channelized firewall : navss256l_modss_timermgr0_cfg*/
#define CSL_CH_FW_NAVSS256L_MODSS_TIMERMGR0_CFG_ID                    4116
#define CSL_CH_FW_NAVSS256L_MODSS_TIMERMGR0_CFG_TYPE                  CSL_FW_CHANNEL
#define CSL_CH_FW_NAVSS256L_MODSS_TIMERMGR0_CFG_MMR_BASE              0x45405000
#define CSL_CH_FW_NAVSS256L_MODSS_TIMERMGR0_CFG_NUM_REGIONS           1
#define CSL_CH_FW_NAVSS256L_MODSS_TIMERMGR0_CFG_NUM_PRIV_IDS_PER_REGION         3
#define CSL_CH_FW_NAVSS256L_MODSS_TIMERMGR0_CFG_CFG_START             0x30e80000
#define CSL_CH_FW_NAVSS256L_MODSS_TIMERMGR0_CFG_CFG_END               0x30e801ff
#define CSL_CH_FW_NAVSS256L_MODSS_TIMERMGR0_CFG_TIMERS_START          0x32200000
#define CSL_CH_FW_NAVSS256L_MODSS_TIMERMGR0_CFG_TIMERS_END            0x3223ffff
#define CSL_CH_FW_NAVSS256L_MODSS_TIMERMGR0_CFG_OES_START             0x30f00000
#define CSL_CH_FW_NAVSS256L_MODSS_TIMERMGR0_CFG_OES_END               0x30f00fff
#define CSL_CH_FW_NAVSS256L_MODSS_TIMERMGR0_CFG_TIMERS_OFFSET_FIRST_RES         0
#define CSL_CH_FW_NAVSS256L_MODSS_TIMERMGR0_CFG_TIMERS_NUM_CHANNELS   64
#define CSL_CH_FW_NAVSS256L_MODSS_TIMERMGR0_CFG_TIMERS_CHANNEL_SIZE   4096

/* Properties of channelized firewall : navss256l_modss_timermgr1_cfg*/
#define CSL_CH_FW_NAVSS256L_MODSS_TIMERMGR1_CFG_ID                    4124
#define CSL_CH_FW_NAVSS256L_MODSS_TIMERMGR1_CFG_TYPE                  CSL_FW_CHANNEL
#define CSL_CH_FW_NAVSS256L_MODSS_TIMERMGR1_CFG_MMR_BASE              0x45407000
#define CSL_CH_FW_NAVSS256L_MODSS_TIMERMGR1_CFG_NUM_REGIONS           1
#define CSL_CH_FW_NAVSS256L_MODSS_TIMERMGR1_CFG_NUM_PRIV_IDS_PER_REGION         3
#define CSL_CH_FW_NAVSS256L_MODSS_TIMERMGR1_CFG_CFG_START             0x30e81000
#define CSL_CH_FW_NAVSS256L_MODSS_TIMERMGR1_CFG_CFG_END               0x30e811ff
#define CSL_CH_FW_NAVSS256L_MODSS_TIMERMGR1_CFG_TIMERS_START          0x32240000
#define CSL_CH_FW_NAVSS256L_MODSS_TIMERMGR1_CFG_TIMERS_END            0x3227ffff
#define CSL_CH_FW_NAVSS256L_MODSS_TIMERMGR1_CFG_OES_START             0x30f01000
#define CSL_CH_FW_NAVSS256L_MODSS_TIMERMGR1_CFG_OES_END               0x30f01fff
#define CSL_CH_FW_NAVSS256L_MODSS_TIMERMGR1_CFG_TIMERS_OFFSET_FIRST_RES         0
#define CSL_CH_FW_NAVSS256L_MODSS_TIMERMGR1_CFG_TIMERS_NUM_CHANNELS   64
#define CSL_CH_FW_NAVSS256L_MODSS_TIMERMGR1_CFG_TIMERS_CHANNEL_SIZE   4096

/* Properties of channelized firewall : navss256l_modss_modss_inta0_cfg*/
#define CSL_CH_FW_NAVSS256L_MODSS_MODSS_INTA0_CFG_ID                  4132
#define CSL_CH_FW_NAVSS256L_MODSS_MODSS_INTA0_CFG_TYPE                CSL_FW_CHANNEL
#define CSL_CH_FW_NAVSS256L_MODSS_MODSS_INTA0_CFG_MMR_BASE            0x45409000
#define CSL_CH_FW_NAVSS256L_MODSS_MODSS_INTA0_CFG_NUM_REGIONS         1
#define CSL_CH_FW_NAVSS256L_MODSS_MODSS_INTA0_CFG_NUM_PRIV_IDS_PER_REGION       3
#define CSL_CH_FW_NAVSS256L_MODSS_MODSS_INTA0_CFG_CFG_START           0x30800000
#define CSL_CH_FW_NAVSS256L_MODSS_MODSS_INTA0_CFG_CFG_END             0x3080001f
#define CSL_CH_FW_NAVSS256L_MODSS_MODSS_INTA0_CFG_IMAP_START          0x30900000
#define CSL_CH_FW_NAVSS256L_MODSS_MODSS_INTA0_CFG_IMAP_END            0x30907fff
#define CSL_CH_FW_NAVSS256L_MODSS_MODSS_INTA0_CFG_INTR_START          0x33c00000
#define CSL_CH_FW_NAVSS256L_MODSS_MODSS_INTA0_CFG_INTR_END            0x33c3ffff
#define CSL_CH_FW_NAVSS256L_MODSS_MODSS_INTA0_CFG_INTR_OFFSET_FIRST_RES         0
#define CSL_CH_FW_NAVSS256L_MODSS_MODSS_INTA0_CFG_INTR_NUM_CHANNELS   64
#define CSL_CH_FW_NAVSS256L_MODSS_MODSS_INTA0_CFG_INTR_CHANNEL_SIZE   4096

/* Properties of channelized firewall : navss256l_modss_modss_inta1_cfg*/
#define CSL_CH_FW_NAVSS256L_MODSS_MODSS_INTA1_CFG_ID                  4140
#define CSL_CH_FW_NAVSS256L_MODSS_MODSS_INTA1_CFG_TYPE                CSL_FW_CHANNEL
#define CSL_CH_FW_NAVSS256L_MODSS_MODSS_INTA1_CFG_MMR_BASE            0x4540b000
#define CSL_CH_FW_NAVSS256L_MODSS_MODSS_INTA1_CFG_NUM_REGIONS         1
#define CSL_CH_FW_NAVSS256L_MODSS_MODSS_INTA1_CFG_NUM_PRIV_IDS_PER_REGION       3
#define CSL_CH_FW_NAVSS256L_MODSS_MODSS_INTA1_CFG_CFG_START           0x30801000
#define CSL_CH_FW_NAVSS256L_MODSS_MODSS_INTA1_CFG_CFG_END             0x3080101f
#define CSL_CH_FW_NAVSS256L_MODSS_MODSS_INTA1_CFG_IMAP_START          0x30908000
#define CSL_CH_FW_NAVSS256L_MODSS_MODSS_INTA1_CFG_IMAP_END            0x3090ffff
#define CSL_CH_FW_NAVSS256L_MODSS_MODSS_INTA1_CFG_INTR_START          0x33c40000
#define CSL_CH_FW_NAVSS256L_MODSS_MODSS_INTA1_CFG_INTR_END            0x33c7ffff
#define CSL_CH_FW_NAVSS256L_MODSS_MODSS_INTA1_CFG_INTR_OFFSET_FIRST_RES         0
#define CSL_CH_FW_NAVSS256L_MODSS_MODSS_INTA1_CFG_INTR_NUM_CHANNELS   64
#define CSL_CH_FW_NAVSS256L_MODSS_MODSS_INTA1_CFG_INTR_CHANNEL_SIZE   4096

/* Properties of channelized firewall : navss256l_modss_proxy0_cfg*/
#define CSL_CH_FW_NAVSS256L_MODSS_PROXY0_CFG_ID                       4148
#define CSL_CH_FW_NAVSS256L_MODSS_PROXY0_CFG_TYPE                     CSL_FW_CHANNEL
#define CSL_CH_FW_NAVSS256L_MODSS_PROXY0_CFG_MMR_BASE                 0x4540d000
#define CSL_CH_FW_NAVSS256L_MODSS_PROXY0_CFG_NUM_REGIONS              1
#define CSL_CH_FW_NAVSS256L_MODSS_PROXY0_CFG_NUM_PRIV_IDS_PER_REGION  3
#define CSL_CH_FW_NAVSS256L_MODSS_PROXY0_CFG_BUF__CFG__GCFG_START     0x31120000
#define CSL_CH_FW_NAVSS256L_MODSS_PROXY0_CFG_BUF__CFG__GCFG_END       0x311200ff
#define CSL_CH_FW_NAVSS256L_MODSS_PROXY0_CFG_BUF__CFG__CFG_START      0x33400000
#define CSL_CH_FW_NAVSS256L_MODSS_PROXY0_CFG_BUF__CFG__CFG_END        0x3343ffff
#define CSL_CH_FW_NAVSS256L_MODSS_PROXY0_CFG_BUF__BUFRAM_SLV__RAM_START         0x31130000
#define CSL_CH_FW_NAVSS256L_MODSS_PROXY0_CFG_BUF__BUFRAM_SLV__RAM_END 0x31133fff
#define CSL_CH_FW_NAVSS256L_MODSS_PROXY0_CFG_BUF__CFG__CFG_OFFSET_FIRST_RES     0
#define CSL_CH_FW_NAVSS256L_MODSS_PROXY0_CFG_BUF__CFG__CFG_NUM_CHANNELS         64
#define CSL_CH_FW_NAVSS256L_MODSS_PROXY0_CFG_BUF__CFG__CFG_CHANNEL_SIZE         4096

/* Properties of channelized firewall : navss256l_modss_sec_proxy0_cfg*/
#define CSL_CH_FW_NAVSS256L_MODSS_SEC_PROXY0_CFG_ID                   4160
#define CSL_CH_FW_NAVSS256L_MODSS_SEC_PROXY0_CFG_TYPE                 CSL_FW_CHANNEL
#define CSL_CH_FW_NAVSS256L_MODSS_SEC_PROXY0_CFG_MMR_BASE             0x45410000
#define CSL_CH_FW_NAVSS256L_MODSS_SEC_PROXY0_CFG_NUM_REGIONS          1
#define CSL_CH_FW_NAVSS256L_MODSS_SEC_PROXY0_CFG_NUM_PRIV_IDS_PER_REGION        3
#define CSL_CH_FW_NAVSS256L_MODSS_SEC_PROXY0_CFG_MMRS_START           0x31140000
#define CSL_CH_FW_NAVSS256L_MODSS_SEC_PROXY0_CFG_MMRS_END             0x311400ff
#define CSL_CH_FW_NAVSS256L_MODSS_SEC_PROXY0_CFG_SCFG_START           0x32800000
#define CSL_CH_FW_NAVSS256L_MODSS_SEC_PROXY0_CFG_SCFG_END             0x328fffff
#define CSL_CH_FW_NAVSS256L_MODSS_SEC_PROXY0_CFG_RT_START             0x32400000
#define CSL_CH_FW_NAVSS256L_MODSS_SEC_PROXY0_CFG_RT_END               0x324fffff
#define CSL_CH_FW_NAVSS256L_MODSS_SEC_PROXY0_CFG_RT_OFFSET_FIRST_RES  0
#define CSL_CH_FW_NAVSS256L_MODSS_SEC_PROXY0_CFG_RT_NUM_CHANNELS      160
#define CSL_CH_FW_NAVSS256L_MODSS_SEC_PROXY0_CFG_RT_CHANNEL_SIZE      4096

/* Properties of channelized firewall : navss256l_modss_pvu0_cfg*/
#define CSL_CH_FW_NAVSS256L_MODSS_PVU0_CFG_ID                         4176
#define CSL_CH_FW_NAVSS256L_MODSS_PVU0_CFG_TYPE                       CSL_FW_CHANNEL
#define CSL_CH_FW_NAVSS256L_MODSS_PVU0_CFG_MMR_BASE                   0x45414000
#define CSL_CH_FW_NAVSS256L_MODSS_PVU0_CFG_NUM_REGIONS                1
#define CSL_CH_FW_NAVSS256L_MODSS_PVU0_CFG_NUM_PRIV_IDS_PER_REGION    3
#define CSL_CH_FW_NAVSS256L_MODSS_PVU0_CFG_CFG__CFG__MMRS_START       0x30f80000
#define CSL_CH_FW_NAVSS256L_MODSS_PVU0_CFG_CFG__CFG__MMRS_END         0x30f80fff
#define CSL_CH_FW_NAVSS256L_MODSS_PVU0_CFG_TLBIF__CFG__TLB_START      0x36000000
#define CSL_CH_FW_NAVSS256L_MODSS_PVU0_CFG_TLBIF__CFG__TLB_END        0x360fffff
#define CSL_CH_FW_NAVSS256L_MODSS_PVU0_CFG_TLBIF__CFG__TLB_OFFSET_FIRST_RES     0
#define CSL_CH_FW_NAVSS256L_MODSS_PVU0_CFG_TLBIF__CFG__TLB_NUM_CHANNELS         256
#define CSL_CH_FW_NAVSS256L_MODSS_PVU0_CFG_TLBIF__CFG__TLB_CHANNEL_SIZE         4096

/* Properties of channelized firewall : navss256l_modss_pvu1_cfg*/
#define CSL_CH_FW_NAVSS256L_MODSS_PVU1_CFG_ID                         4208
#define CSL_CH_FW_NAVSS256L_MODSS_PVU1_CFG_TYPE                       CSL_FW_CHANNEL
#define CSL_CH_FW_NAVSS256L_MODSS_PVU1_CFG_MMR_BASE                   0x4541c000
#define CSL_CH_FW_NAVSS256L_MODSS_PVU1_CFG_NUM_REGIONS                1
#define CSL_CH_FW_NAVSS256L_MODSS_PVU1_CFG_NUM_PRIV_IDS_PER_REGION    3
#define CSL_CH_FW_NAVSS256L_MODSS_PVU1_CFG_CFG__CFG__MMRS_START       0x30f81000
#define CSL_CH_FW_NAVSS256L_MODSS_PVU1_CFG_CFG__CFG__MMRS_END         0x30f81fff
#define CSL_CH_FW_NAVSS256L_MODSS_PVU1_CFG_TLBIF__CFG__TLB_START      0x36100000
#define CSL_CH_FW_NAVSS256L_MODSS_PVU1_CFG_TLBIF__CFG__TLB_END        0x361fffff
#define CSL_CH_FW_NAVSS256L_MODSS_PVU1_CFG_TLBIF__CFG__TLB_OFFSET_FIRST_RES     0
#define CSL_CH_FW_NAVSS256L_MODSS_PVU1_CFG_TLBIF__CFG__TLB_NUM_CHANNELS         256
#define CSL_CH_FW_NAVSS256L_MODSS_PVU1_CFG_TLBIF__CFG__TLB_CHANNEL_SIZE         4096

/* Properties of channelized firewall : navss256l_modss_ringacc0_cfg*/
#define CSL_CH_FW_NAVSS256L_MODSS_RINGACC0_CFG_ID                     4256
#define CSL_CH_FW_NAVSS256L_MODSS_RINGACC0_CFG_TYPE                   CSL_FW_CHANNEL
#define CSL_CH_FW_NAVSS256L_MODSS_RINGACC0_CFG_MMR_BASE               0x45428000
#define CSL_CH_FW_NAVSS256L_MODSS_RINGACC0_CFG_NUM_REGIONS            2
#define CSL_CH_FW_NAVSS256L_MODSS_RINGACC0_CFG_NUM_PRIV_IDS_PER_REGION          3
#define CSL_CH_FW_NAVSS256L_MODSS_RINGACC0_CFG_UDMASS_RINGACC0_GCFG_START       0x31160000
#define CSL_CH_FW_NAVSS256L_MODSS_RINGACC0_CFG_UDMASS_RINGACC0_GCFG_END         0x311603ff
#define CSL_CH_FW_NAVSS256L_MODSS_RINGACC0_CFG_UDMASS_RINGACC0_CFG_START        0x31080000
#define CSL_CH_FW_NAVSS256L_MODSS_RINGACC0_CFG_UDMASS_RINGACC0_CFG_END          0x310bffff
#define CSL_CH_FW_NAVSS256L_MODSS_RINGACC0_CFG_UDMASS_RINGACC0_RT_START         0x3c000000
#define CSL_CH_FW_NAVSS256L_MODSS_RINGACC0_CFG_UDMASS_RINGACC0_RT_END 0x3c3fffff
#define CSL_CH_FW_NAVSS256L_MODSS_RINGACC0_CFG_UDMASS_RINGACC0_MON_START        0x32000000
#define CSL_CH_FW_NAVSS256L_MODSS_RINGACC0_CFG_UDMASS_RINGACC0_MON_END          0x3201ffff
#define CSL_CH_FW_NAVSS256L_MODSS_RINGACC0_CFG_UDMASS_RINGACC0_RT_OFFSET_FIRST_RES        0
#define CSL_CH_FW_NAVSS256L_MODSS_RINGACC0_CFG_UDMASS_RINGACC0_RT_NUM_CHANNELS  818
#define CSL_CH_FW_NAVSS256L_MODSS_RINGACC0_CFG_UDMASS_RINGACC0_RT_CHANNEL_SIZE  4096
#define CSL_CH_FW_NAVSS256L_MODSS_RINGACC0_CFG_UDMASS_RINGACC0_MON_OFFSET_FIRST_RES       0
#define CSL_CH_FW_NAVSS256L_MODSS_RINGACC0_CFG_UDMASS_RINGACC0_MON_NUM_CHANNELS 32
#define CSL_CH_FW_NAVSS256L_MODSS_RINGACC0_CFG_UDMASS_RINGACC0_MON_CHANNEL_SIZE 4096

/* Properties of channelized firewall : navss256l_modss_udmassinta0_cfg*/
#define CSL_CH_FW_NAVSS256L_MODSS_UDMASSINTA0_CFG_ID                  4320
#define CSL_CH_FW_NAVSS256L_MODSS_UDMASSINTA0_CFG_TYPE                CSL_FW_CHANNEL
#define CSL_CH_FW_NAVSS256L_MODSS_UDMASSINTA0_CFG_MMR_BASE            0x45438000
#define CSL_CH_FW_NAVSS256L_MODSS_UDMASSINTA0_CFG_NUM_REGIONS         2
#define CSL_CH_FW_NAVSS256L_MODSS_UDMASSINTA0_CFG_NUM_PRIV_IDS_PER_REGION       3
#define CSL_CH_FW_NAVSS256L_MODSS_UDMASSINTA0_CFG_UDMASS_INTA0_CFG_START        0x30802000
#define CSL_CH_FW_NAVSS256L_MODSS_UDMASSINTA0_CFG_UDMASS_INTA0_CFG_END          0x3080201f
#define CSL_CH_FW_NAVSS256L_MODSS_UDMASSINTA0_CFG_UDMASS_INTA0_IMAP_START       0x30940000
#define CSL_CH_FW_NAVSS256L_MODSS_UDMASSINTA0_CFG_UDMASS_INTA0_IMAP_END         0x3097ffff
#define CSL_CH_FW_NAVSS256L_MODSS_UDMASSINTA0_CFG_UDMASS_INTA0_INTR_START       0x33d00000
#define CSL_CH_FW_NAVSS256L_MODSS_UDMASSINTA0_CFG_UDMASS_INTA0_INTR_END         0x33dfffff
#define CSL_CH_FW_NAVSS256L_MODSS_UDMASSINTA0_CFG_UDMASS_INTA0_L2G_START        0x31100000
#define CSL_CH_FW_NAVSS256L_MODSS_UDMASSINTA0_CFG_UDMASS_INTA0_L2G_END          0x3110007f
#define CSL_CH_FW_NAVSS256L_MODSS_UDMASSINTA0_CFG_UDMASS_INTA0_MCAST_START      0x31110000
#define CSL_CH_FW_NAVSS256L_MODSS_UDMASSINTA0_CFG_UDMASS_INTA0_MCAST_END        0x31113fff
#define CSL_CH_FW_NAVSS256L_MODSS_UDMASSINTA0_CFG_UDMASS_INTA0_GCNTCFG_START    0x31040000
#define CSL_CH_FW_NAVSS256L_MODSS_UDMASSINTA0_CFG_UDMASS_INTA0_GCNTCFG_END      0x31043fff
#define CSL_CH_FW_NAVSS256L_MODSS_UDMASSINTA0_CFG_UDMASS_INTA0_GCNTRTI_START    0x33800000
#define CSL_CH_FW_NAVSS256L_MODSS_UDMASSINTA0_CFG_UDMASS_INTA0_GCNTRTI_END      0x339fffff
#define CSL_CH_FW_NAVSS256L_MODSS_UDMASSINTA0_CFG_UDMASS_INTA0_INTR_OFFSET_FIRST_RES      0
#define CSL_CH_FW_NAVSS256L_MODSS_UDMASSINTA0_CFG_UDMASS_INTA0_INTR_NUM_CHANNELS          256
#define CSL_CH_FW_NAVSS256L_MODSS_UDMASSINTA0_CFG_UDMASS_INTA0_INTR_CHANNEL_SIZE          4096
#define CSL_CH_FW_NAVSS256L_MODSS_UDMASSINTA0_CFG_UDMASS_INTA0_GCNTRTI_OFFSET_FIRST_RES   0
#define CSL_CH_FW_NAVSS256L_MODSS_UDMASSINTA0_CFG_UDMASS_INTA0_GCNTRTI_NUM_CHANNELS       512
#define CSL_CH_FW_NAVSS256L_MODSS_UDMASSINTA0_CFG_UDMASS_INTA0_GCNTRTI_CHANNEL_SIZE       4096

/* Properties of channelized firewall : navss256l_modss_udmap0_cfg*/
#define CSL_CH_FW_NAVSS256L_MODSS_UDMAP0_CFG_ID                       4368
#define CSL_CH_FW_NAVSS256L_MODSS_UDMAP0_CFG_TYPE                     CSL_FW_CHANNEL
#define CSL_CH_FW_NAVSS256L_MODSS_UDMAP0_CFG_MMR_BASE                 0x45444000
#define CSL_CH_FW_NAVSS256L_MODSS_UDMAP0_CFG_NUM_REGIONS              2
#define CSL_CH_FW_NAVSS256L_MODSS_UDMAP0_CFG_NUM_PRIV_IDS_PER_REGION  3
#define CSL_CH_FW_NAVSS256L_MODSS_UDMAP0_CFG_UDMASS_UDMAP0_GCFG_START 0x31150000
#define CSL_CH_FW_NAVSS256L_MODSS_UDMAP0_CFG_UDMASS_UDMAP0_GCFG_END   0x311500ff
#define CSL_CH_FW_NAVSS256L_MODSS_UDMAP0_CFG_UDMASS_UDMAP0_RFLOW_START          0x30d00000
#define CSL_CH_FW_NAVSS256L_MODSS_UDMAP0_CFG_UDMASS_UDMAP0_RFLOW_END  0x30d07fff
#define CSL_CH_FW_NAVSS256L_MODSS_UDMAP0_CFG_UDMASS_UDMAP0_TCHAN_START          0x30b00000
#define CSL_CH_FW_NAVSS256L_MODSS_UDMAP0_CFG_UDMASS_UDMAP0_TCHAN_END  0x30b0ffff
#define CSL_CH_FW_NAVSS256L_MODSS_UDMAP0_CFG_UDMASS_UDMAP0_RCHAN_START          0x30c00000
#define CSL_CH_FW_NAVSS256L_MODSS_UDMAP0_CFG_UDMASS_UDMAP0_RCHAN_END  0x30c0ffff
#define CSL_CH_FW_NAVSS256L_MODSS_UDMAP0_CFG_UDMASS_UDMAP0_TCHANRT_START        0x35000000
#define CSL_CH_FW_NAVSS256L_MODSS_UDMAP0_CFG_UDMASS_UDMAP0_TCHANRT_END          0x350fffff
#define CSL_CH_FW_NAVSS256L_MODSS_UDMAP0_CFG_UDMASS_UDMAP0_RCHANRT_START        0x34000000
#define CSL_CH_FW_NAVSS256L_MODSS_UDMAP0_CFG_UDMASS_UDMAP0_RCHANRT_END          0x340fffff
#define CSL_CH_FW_NAVSS256L_MODSS_UDMAP0_CFG_UDMASS_UDMAP0_TCHANRT_OFFSET_FIRST_RES       0
#define CSL_CH_FW_NAVSS256L_MODSS_UDMAP0_CFG_UDMASS_UDMAP0_TCHANRT_NUM_CHANNELS 152
#define CSL_CH_FW_NAVSS256L_MODSS_UDMAP0_CFG_UDMASS_UDMAP0_TCHANRT_CHANNEL_SIZE 4096
#define CSL_CH_FW_NAVSS256L_MODSS_UDMAP0_CFG_UDMASS_UDMAP0_RCHANRT_OFFSET_FIRST_RES       0
#define CSL_CH_FW_NAVSS256L_MODSS_UDMAP0_CFG_UDMASS_UDMAP0_RCHANRT_NUM_CHANNELS 150
#define CSL_CH_FW_NAVSS256L_MODSS_UDMAP0_CFG_UDMASS_UDMAP0_RCHANRT_CHANNEL_SIZE 4096

/* Properties of channelized firewall : navss256l_modss_proxy0_src*/
#define CSL_CH_FW_NAVSS256L_MODSS_PROXY0_SRC_ID                       4392
#define CSL_CH_FW_NAVSS256L_MODSS_PROXY0_SRC_TYPE                     CSL_FW_CHANNEL
#define CSL_CH_FW_NAVSS256L_MODSS_PROXY0_SRC_MMR_BASE                 0x4544a000
#define CSL_CH_FW_NAVSS256L_MODSS_PROXY0_SRC_NUM_REGIONS              1
#define CSL_CH_FW_NAVSS256L_MODSS_PROXY0_SRC_NUM_PRIV_IDS_PER_REGION  3
#define CSL_CH_FW_NAVSS256L_MODSS_PROXY0_SRC_TARGET0_DATA_START       0x33000000
#define CSL_CH_FW_NAVSS256L_MODSS_PROXY0_SRC_TARGET0_DATA_END         0x3303ffff
#define CSL_CH_FW_NAVSS256L_MODSS_PROXY0_SRC_TARGET0_DATA_OFFSET_FIRST_RES      0
#define CSL_CH_FW_NAVSS256L_MODSS_PROXY0_SRC_TARGET0_DATA_NUM_CHANNELS          64
#define CSL_CH_FW_NAVSS256L_MODSS_PROXY0_SRC_TARGET0_DATA_CHANNEL_SIZE          4096

/* Properties of channelized firewall : navss256l_modss_sec_proxy0_src*/
#define CSL_CH_FW_NAVSS256L_MODSS_SEC_PROXY0_SRC_ID                   4400
#define CSL_CH_FW_NAVSS256L_MODSS_SEC_PROXY0_SRC_TYPE                 CSL_FW_CHANNEL
#define CSL_CH_FW_NAVSS256L_MODSS_SEC_PROXY0_SRC_MMR_BASE             0x4544c000
#define CSL_CH_FW_NAVSS256L_MODSS_SEC_PROXY0_SRC_NUM_REGIONS          1
#define CSL_CH_FW_NAVSS256L_MODSS_SEC_PROXY0_SRC_NUM_PRIV_IDS_PER_REGION        3
#define CSL_CH_FW_NAVSS256L_MODSS_SEC_PROXY0_SRC_TARGET_DATA_START    0x32c00000
#define CSL_CH_FW_NAVSS256L_MODSS_SEC_PROXY0_SRC_TARGET_DATA_END      0x32cfffff
#define CSL_CH_FW_NAVSS256L_MODSS_SEC_PROXY0_SRC_TARGET_DATA_OFFSET_FIRST_RES   0
#define CSL_CH_FW_NAVSS256L_MODSS_SEC_PROXY0_SRC_TARGET_DATA_NUM_CHANNELS       160
#define CSL_CH_FW_NAVSS256L_MODSS_SEC_PROXY0_SRC_TARGET_DATA_CHANNEL_SIZE       4096

/* Properties of channelized firewall : navss256l_modss_ringacc0_src*/
#define CSL_CH_FW_NAVSS256L_MODSS_RINGACC0_SRC_ID                     4416
#define CSL_CH_FW_NAVSS256L_MODSS_RINGACC0_SRC_TYPE                   CSL_FW_CHANNEL
#define CSL_CH_FW_NAVSS256L_MODSS_RINGACC0_SRC_MMR_BASE               0x45450000
#define CSL_CH_FW_NAVSS256L_MODSS_RINGACC0_SRC_NUM_REGIONS            1
#define CSL_CH_FW_NAVSS256L_MODSS_RINGACC0_SRC_NUM_PRIV_IDS_PER_REGION          3
#define CSL_CH_FW_NAVSS256L_MODSS_RINGACC0_SRC_UDMASS_RINGACC0_SRC_FIFOS_START  0x38000000
#define CSL_CH_FW_NAVSS256L_MODSS_RINGACC0_SRC_UDMASS_RINGACC0_SRC_FIFOS_END    0x383fffff
#define CSL_CH_FW_NAVSS256L_MODSS_RINGACC0_SRC_UDMASS_RINGACC0_SRC_FIFOS_OFFSET_FIRST_RES 0
#define CSL_CH_FW_NAVSS256L_MODSS_RINGACC0_SRC_UDMASS_RINGACC0_SRC_FIFOS_NUM_CHANNELS     818
#define CSL_CH_FW_NAVSS256L_MODSS_RINGACC0_SRC_UDMASS_RINGACC0_SRC_FIFOS_CHANNEL_SIZE     4096

/* Properties of channelized firewall : navss_mcu_modss_proxy0_cfg*/
#define CSL_CH_FW_NAVSS_MCU_MODSS_PROXY0_CFG_ID                       6146
#define CSL_CH_FW_NAVSS_MCU_MODSS_PROXY0_CFG_TYPE                     CSL_FW_CHANNEL
#define CSL_CH_FW_NAVSS_MCU_MODSS_PROXY0_CFG_MMR_BASE                 0x45600800
#define CSL_CH_FW_NAVSS_MCU_MODSS_PROXY0_CFG_NUM_REGIONS              1
#define CSL_CH_FW_NAVSS_MCU_MODSS_PROXY0_CFG_NUM_PRIV_IDS_PER_REGION  3
#define CSL_CH_FW_NAVSS_MCU_MODSS_PROXY0_CFG_BUF__CFG__GCFG_START     0x28590000
#define CSL_CH_FW_NAVSS_MCU_MODSS_PROXY0_CFG_BUF__CFG__GCFG_END       0x285900ff
#define CSL_CH_FW_NAVSS_MCU_MODSS_PROXY0_CFG_BUF__CFG__CFG_START      0x2a580000
#define CSL_CH_FW_NAVSS_MCU_MODSS_PROXY0_CFG_BUF__CFG__CFG_END        0x2a5bffff
#define CSL_CH_FW_NAVSS_MCU_MODSS_PROXY0_CFG_BUF__BUFRAM_SLV__RAM_START         0x285a0000
#define CSL_CH_FW_NAVSS_MCU_MODSS_PROXY0_CFG_BUF__BUFRAM_SLV__RAM_END 0x285a3fff
#define CSL_CH_FW_NAVSS_MCU_MODSS_PROXY0_CFG_BUF__CFG__CFG_OFFSET_FIRST_RES     0
#define CSL_CH_FW_NAVSS_MCU_MODSS_PROXY0_CFG_BUF__CFG__CFG_NUM_CHANNELS         64
#define CSL_CH_FW_NAVSS_MCU_MODSS_PROXY0_CFG_BUF__CFG__CFG_CHANNEL_SIZE         4096

/* Properties of channelized firewall : navss_mcu_modss_sec_proxy0_cfg*/
#define CSL_CH_FW_NAVSS_MCU_MODSS_SEC_PROXY0_CFG_ID                   6152
#define CSL_CH_FW_NAVSS_MCU_MODSS_SEC_PROXY0_CFG_TYPE                 CSL_FW_CHANNEL
#define CSL_CH_FW_NAVSS_MCU_MODSS_SEC_PROXY0_CFG_MMR_BASE             0x45602000
#define CSL_CH_FW_NAVSS_MCU_MODSS_SEC_PROXY0_CFG_NUM_REGIONS          1
#define CSL_CH_FW_NAVSS_MCU_MODSS_SEC_PROXY0_CFG_NUM_PRIV_IDS_PER_REGION        3
#define CSL_CH_FW_NAVSS_MCU_MODSS_SEC_PROXY0_CFG_MMRS_START           0x285b0000
#define CSL_CH_FW_NAVSS_MCU_MODSS_SEC_PROXY0_CFG_MMRS_END             0x285b00ff
#define CSL_CH_FW_NAVSS_MCU_MODSS_SEC_PROXY0_CFG_SCFG_START           0x2a400000
#define CSL_CH_FW_NAVSS_MCU_MODSS_SEC_PROXY0_CFG_SCFG_END             0x2a47ffff
#define CSL_CH_FW_NAVSS_MCU_MODSS_SEC_PROXY0_CFG_RT_START             0x2a380000
#define CSL_CH_FW_NAVSS_MCU_MODSS_SEC_PROXY0_CFG_RT_END               0x2a3fffff
#define CSL_CH_FW_NAVSS_MCU_MODSS_SEC_PROXY0_CFG_RT_OFFSET_FIRST_RES  0
#define CSL_CH_FW_NAVSS_MCU_MODSS_SEC_PROXY0_CFG_RT_NUM_CHANNELS      90
#define CSL_CH_FW_NAVSS_MCU_MODSS_SEC_PROXY0_CFG_RT_CHANNEL_SIZE      4096

/* Properties of channelized firewall : navss_mcu_modss_ringacc0_cfg*/
#define CSL_CH_FW_NAVSS_MCU_MODSS_RINGACC0_CFG_ID                     6160
#define CSL_CH_FW_NAVSS_MCU_MODSS_RINGACC0_CFG_TYPE                   CSL_FW_CHANNEL
#define CSL_CH_FW_NAVSS_MCU_MODSS_RINGACC0_CFG_MMR_BASE               0x45604000
#define CSL_CH_FW_NAVSS_MCU_MODSS_RINGACC0_CFG_NUM_REGIONS            2
#define CSL_CH_FW_NAVSS_MCU_MODSS_RINGACC0_CFG_NUM_PRIV_IDS_PER_REGION          3
#define CSL_CH_FW_NAVSS_MCU_MODSS_RINGACC0_CFG_UDMASS_RINGACC0_GCFG_START       0x285d0000
#define CSL_CH_FW_NAVSS_MCU_MODSS_RINGACC0_CFG_UDMASS_RINGACC0_GCFG_END         0x285d03ff
#define CSL_CH_FW_NAVSS_MCU_MODSS_RINGACC0_CFG_UDMASS_RINGACC0_CFG_START        0x28440000
#define CSL_CH_FW_NAVSS_MCU_MODSS_RINGACC0_CFG_UDMASS_RINGACC0_CFG_END          0x2847ffff
#define CSL_CH_FW_NAVSS_MCU_MODSS_RINGACC0_CFG_UDMASS_RINGACC0_RT_START         0x2b800000
#define CSL_CH_FW_NAVSS_MCU_MODSS_RINGACC0_CFG_UDMASS_RINGACC0_RT_END 0x2bbfffff
#define CSL_CH_FW_NAVSS_MCU_MODSS_RINGACC0_CFG_UDMASS_RINGACC0_MON_START        0x2a280000
#define CSL_CH_FW_NAVSS_MCU_MODSS_RINGACC0_CFG_UDMASS_RINGACC0_MON_END          0x2a29ffff
#define CSL_CH_FW_NAVSS_MCU_MODSS_RINGACC0_CFG_UDMASS_RINGACC0_RT_OFFSET_FIRST_RES        0
#define CSL_CH_FW_NAVSS_MCU_MODSS_RINGACC0_CFG_UDMASS_RINGACC0_RT_NUM_CHANNELS  286
#define CSL_CH_FW_NAVSS_MCU_MODSS_RINGACC0_CFG_UDMASS_RINGACC0_RT_CHANNEL_SIZE  4096
#define CSL_CH_FW_NAVSS_MCU_MODSS_RINGACC0_CFG_UDMASS_RINGACC0_MON_OFFSET_FIRST_RES       0
#define CSL_CH_FW_NAVSS_MCU_MODSS_RINGACC0_CFG_UDMASS_RINGACC0_MON_NUM_CHANNELS 32
#define CSL_CH_FW_NAVSS_MCU_MODSS_RINGACC0_CFG_UDMASS_RINGACC0_MON_CHANNEL_SIZE 4096

/* Properties of channelized firewall : navss_mcu_modss_udmassinta0_cfg*/
#define CSL_CH_FW_NAVSS_MCU_MODSS_UDMASSINTA0_CFG_ID                  6208
#define CSL_CH_FW_NAVSS_MCU_MODSS_UDMASSINTA0_CFG_TYPE                CSL_FW_CHANNEL
#define CSL_CH_FW_NAVSS_MCU_MODSS_UDMASSINTA0_CFG_MMR_BASE            0x45610000
#define CSL_CH_FW_NAVSS_MCU_MODSS_UDMASSINTA0_CFG_NUM_REGIONS         2
#define CSL_CH_FW_NAVSS_MCU_MODSS_UDMASSINTA0_CFG_NUM_PRIV_IDS_PER_REGION       3
#define CSL_CH_FW_NAVSS_MCU_MODSS_UDMASSINTA0_CFG_UDMASS_INTA0_CFG_START        0x283c0000
#define CSL_CH_FW_NAVSS_MCU_MODSS_UDMASSINTA0_CFG_UDMASS_INTA0_CFG_END          0x283c001f
#define CSL_CH_FW_NAVSS_MCU_MODSS_UDMASSINTA0_CFG_UDMASS_INTA0_IMAP_START       0x28560000
#define CSL_CH_FW_NAVSS_MCU_MODSS_UDMASSINTA0_CFG_UDMASS_INTA0_IMAP_END         0x2856ffff
#define CSL_CH_FW_NAVSS_MCU_MODSS_UDMASSINTA0_CFG_UDMASS_INTA0_INTR_START       0x2a700000
#define CSL_CH_FW_NAVSS_MCU_MODSS_UDMASSINTA0_CFG_UDMASS_INTA0_INTR_END         0x2a7fffff
#define CSL_CH_FW_NAVSS_MCU_MODSS_UDMASSINTA0_CFG_UDMASS_INTA0_L2G_START        0x28570000
#define CSL_CH_FW_NAVSS_MCU_MODSS_UDMASSINTA0_CFG_UDMASS_INTA0_L2G_END          0x2857007f
#define CSL_CH_FW_NAVSS_MCU_MODSS_UDMASSINTA0_CFG_UDMASS_INTA0_MCAST_START      0x28580000
#define CSL_CH_FW_NAVSS_MCU_MODSS_UDMASSINTA0_CFG_UDMASS_INTA0_MCAST_END        0x28580fff
#define CSL_CH_FW_NAVSS_MCU_MODSS_UDMASSINTA0_CFG_UDMASS_INTA0_GCNTCFG_START    0x28480000
#define CSL_CH_FW_NAVSS_MCU_MODSS_UDMASSINTA0_CFG_UDMASS_INTA0_GCNTCFG_END      0x28481fff
#define CSL_CH_FW_NAVSS_MCU_MODSS_UDMASSINTA0_CFG_UDMASS_INTA0_GCNTRTI_START    0x2a600000
#define CSL_CH_FW_NAVSS_MCU_MODSS_UDMASSINTA0_CFG_UDMASS_INTA0_GCNTRTI_END      0x2a6fffff
#define CSL_CH_FW_NAVSS_MCU_MODSS_UDMASSINTA0_CFG_UDMASS_INTA0_INTR_OFFSET_FIRST_RES      0
#define CSL_CH_FW_NAVSS_MCU_MODSS_UDMASSINTA0_CFG_UDMASS_INTA0_INTR_NUM_CHANNELS          256
#define CSL_CH_FW_NAVSS_MCU_MODSS_UDMASSINTA0_CFG_UDMASS_INTA0_INTR_CHANNEL_SIZE          4096
#define CSL_CH_FW_NAVSS_MCU_MODSS_UDMASSINTA0_CFG_UDMASS_INTA0_GCNTRTI_OFFSET_FIRST_RES   0
#define CSL_CH_FW_NAVSS_MCU_MODSS_UDMASSINTA0_CFG_UDMASS_INTA0_GCNTRTI_NUM_CHANNELS       256
#define CSL_CH_FW_NAVSS_MCU_MODSS_UDMASSINTA0_CFG_UDMASS_INTA0_GCNTRTI_CHANNEL_SIZE       4096

/* Properties of channelized firewall : navss_mcu_modss_udmap0_cfg*/
#define CSL_CH_FW_NAVSS_MCU_MODSS_UDMAP0_CFG_ID                       6244
#define CSL_CH_FW_NAVSS_MCU_MODSS_UDMAP0_CFG_TYPE                     CSL_FW_CHANNEL
#define CSL_CH_FW_NAVSS_MCU_MODSS_UDMAP0_CFG_MMR_BASE                 0x45619000
#define CSL_CH_FW_NAVSS_MCU_MODSS_UDMAP0_CFG_NUM_REGIONS              2
#define CSL_CH_FW_NAVSS_MCU_MODSS_UDMAP0_CFG_NUM_PRIV_IDS_PER_REGION  3
#define CSL_CH_FW_NAVSS_MCU_MODSS_UDMAP0_CFG_UDMASS_UDMAP0_GCFG_START 0x285c0000
#define CSL_CH_FW_NAVSS_MCU_MODSS_UDMAP0_CFG_UDMASS_UDMAP0_GCFG_END   0x285c00ff
#define CSL_CH_FW_NAVSS_MCU_MODSS_UDMAP0_CFG_UDMASS_UDMAP0_RFLOW_START          0x28400000
#define CSL_CH_FW_NAVSS_MCU_MODSS_UDMAP0_CFG_UDMASS_UDMAP0_RFLOW_END  0x28401fff
#define CSL_CH_FW_NAVSS_MCU_MODSS_UDMAP0_CFG_UDMASS_UDMAP0_TCHAN_START          0x284a0000
#define CSL_CH_FW_NAVSS_MCU_MODSS_UDMAP0_CFG_UDMASS_UDMAP0_TCHAN_END  0x284a3fff
#define CSL_CH_FW_NAVSS_MCU_MODSS_UDMAP0_CFG_UDMASS_UDMAP0_RCHAN_START          0x284c0000
#define CSL_CH_FW_NAVSS_MCU_MODSS_UDMAP0_CFG_UDMASS_UDMAP0_RCHAN_END  0x284c3fff
#define CSL_CH_FW_NAVSS_MCU_MODSS_UDMAP0_CFG_UDMASS_UDMAP0_TCHANRT_START        0x2aa00000
#define CSL_CH_FW_NAVSS_MCU_MODSS_UDMAP0_CFG_UDMASS_UDMAP0_TCHANRT_END          0x2aa3ffff
#define CSL_CH_FW_NAVSS_MCU_MODSS_UDMAP0_CFG_UDMASS_UDMAP0_RCHANRT_START        0x2a800000
#define CSL_CH_FW_NAVSS_MCU_MODSS_UDMAP0_CFG_UDMASS_UDMAP0_RCHANRT_END          0x2a83ffff
#define CSL_CH_FW_NAVSS_MCU_MODSS_UDMAP0_CFG_UDMASS_UDMAP0_TCHANRT_OFFSET_FIRST_RES       0
#define CSL_CH_FW_NAVSS_MCU_MODSS_UDMAP0_CFG_UDMASS_UDMAP0_TCHANRT_NUM_CHANNELS 48
#define CSL_CH_FW_NAVSS_MCU_MODSS_UDMAP0_CFG_UDMASS_UDMAP0_TCHANRT_CHANNEL_SIZE 4096
#define CSL_CH_FW_NAVSS_MCU_MODSS_UDMAP0_CFG_UDMASS_UDMAP0_RCHANRT_OFFSET_FIRST_RES       0
#define CSL_CH_FW_NAVSS_MCU_MODSS_UDMAP0_CFG_UDMASS_UDMAP0_RCHANRT_NUM_CHANNELS 48
#define CSL_CH_FW_NAVSS_MCU_MODSS_UDMAP0_CFG_UDMASS_UDMAP0_RCHANRT_CHANNEL_SIZE 4096

/* Properties of channelized firewall : navss_mcu_modss_proxy0_src*/
#define CSL_CH_FW_NAVSS_MCU_MODSS_PROXY0_SRC_ID                       6256
#define CSL_CH_FW_NAVSS_MCU_MODSS_PROXY0_SRC_TYPE                     CSL_FW_CHANNEL
#define CSL_CH_FW_NAVSS_MCU_MODSS_PROXY0_SRC_MMR_BASE                 0x4561c000
#define CSL_CH_FW_NAVSS_MCU_MODSS_PROXY0_SRC_NUM_REGIONS              1
#define CSL_CH_FW_NAVSS_MCU_MODSS_PROXY0_SRC_NUM_PRIV_IDS_PER_REGION  3
#define CSL_CH_FW_NAVSS_MCU_MODSS_PROXY0_SRC_TARGET0_DATA_START       0x2a500000
#define CSL_CH_FW_NAVSS_MCU_MODSS_PROXY0_SRC_TARGET0_DATA_END         0x2a53ffff
#define CSL_CH_FW_NAVSS_MCU_MODSS_PROXY0_SRC_TARGET0_DATA_OFFSET_FIRST_RES      0
#define CSL_CH_FW_NAVSS_MCU_MODSS_PROXY0_SRC_TARGET0_DATA_NUM_CHANNELS          64
#define CSL_CH_FW_NAVSS_MCU_MODSS_PROXY0_SRC_TARGET0_DATA_CHANNEL_SIZE          4096

/* Properties of channelized firewall : navss_mcu_modss_sec_proxy0_src*/
#define CSL_CH_FW_NAVSS_MCU_MODSS_SEC_PROXY0_SRC_ID                   6264
#define CSL_CH_FW_NAVSS_MCU_MODSS_SEC_PROXY0_SRC_TYPE                 CSL_FW_CHANNEL
#define CSL_CH_FW_NAVSS_MCU_MODSS_SEC_PROXY0_SRC_MMR_BASE             0x4561e000
#define CSL_CH_FW_NAVSS_MCU_MODSS_SEC_PROXY0_SRC_NUM_REGIONS          1
#define CSL_CH_FW_NAVSS_MCU_MODSS_SEC_PROXY0_SRC_NUM_PRIV_IDS_PER_REGION        3
#define CSL_CH_FW_NAVSS_MCU_MODSS_SEC_PROXY0_SRC_TARGET_DATA_START    0x2a480000
#define CSL_CH_FW_NAVSS_MCU_MODSS_SEC_PROXY0_SRC_TARGET_DATA_END      0x2a4fffff
#define CSL_CH_FW_NAVSS_MCU_MODSS_SEC_PROXY0_SRC_TARGET_DATA_OFFSET_FIRST_RES   0
#define CSL_CH_FW_NAVSS_MCU_MODSS_SEC_PROXY0_SRC_TARGET_DATA_NUM_CHANNELS       90
#define CSL_CH_FW_NAVSS_MCU_MODSS_SEC_PROXY0_SRC_TARGET_DATA_CHANNEL_SIZE       4096

/* Properties of channelized firewall : navss_mcu_modss_ringacc0_src*/
#define CSL_CH_FW_NAVSS_MCU_MODSS_RINGACC0_SRC_ID                     6272
#define CSL_CH_FW_NAVSS_MCU_MODSS_RINGACC0_SRC_TYPE                   CSL_FW_CHANNEL
#define CSL_CH_FW_NAVSS_MCU_MODSS_RINGACC0_SRC_MMR_BASE               0x45620000
#define CSL_CH_FW_NAVSS_MCU_MODSS_RINGACC0_SRC_NUM_REGIONS            1
#define CSL_CH_FW_NAVSS_MCU_MODSS_RINGACC0_SRC_NUM_PRIV_IDS_PER_REGION          3
#define CSL_CH_FW_NAVSS_MCU_MODSS_RINGACC0_SRC_UDMASS_RINGACC0_SRC_FIFOS_START  0x2b000000
#define CSL_CH_FW_NAVSS_MCU_MODSS_RINGACC0_SRC_UDMASS_RINGACC0_SRC_FIFOS_END    0x2b3fffff
#define CSL_CH_FW_NAVSS_MCU_MODSS_RINGACC0_SRC_UDMASS_RINGACC0_SRC_FIFOS_OFFSET_FIRST_RES 0
#define CSL_CH_FW_NAVSS_MCU_MODSS_RINGACC0_SRC_UDMASS_RINGACC0_SRC_FIFOS_NUM_CHANNELS     286
#define CSL_CH_FW_NAVSS_MCU_MODSS_RINGACC0_SRC_UDMASS_RINGACC0_SRC_FIFOS_CHANNEL_SIZE     4096


#endif /*_CSL_SOC_FW_H_ */
