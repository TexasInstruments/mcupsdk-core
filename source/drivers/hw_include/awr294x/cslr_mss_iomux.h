/********************************************************************
 * Copyright (C) 2021 Texas Instruments Incorporated.
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
 *  Name        : cslr_mss_iomux.h
*/
#ifndef CSLR_MSS_IOMUX_H_
#define CSLR_MSS_IOMUX_H_

#include <drivers/hw_include/cslr.h>
#include <drivers/hw_include/tistdtypes.h>

#ifdef __cplusplus
extern "C"
{
#endif

/**************************************************************************
* Hardware Region  :
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t PADAA_CFG_REG;
    volatile uint32_t PADAB_CFG_REG;
    volatile uint32_t PADAC_CFG_REG;
    volatile uint32_t PADAD_CFG_REG;
    volatile uint32_t PADAE_CFG_REG;
    volatile uint32_t PADAF_CFG_REG;
    volatile uint32_t PADAG_CFG_REG;
    volatile uint32_t PADAH_CFG_REG;
    volatile uint32_t PADAI_CFG_REG;
    volatile uint32_t PADAJ_CFG_REG;
    volatile uint32_t PADAK_CFG_REG;
    volatile uint32_t PADAL_CFG_REG;
    volatile uint32_t PADAM_CFG_REG;
    volatile uint32_t PADAN_CFG_REG;
    volatile uint32_t PADAO_CFG_REG;
    volatile uint32_t PADAP_CFG_REG;
    volatile uint32_t PADAQ_CFG_REG;
    volatile uint32_t PADAR_CFG_REG;
    volatile uint32_t PADAS_CFG_REG;
    volatile uint32_t PADAT_CFG_REG;
    volatile uint32_t PADAU_CFG_REG;
    volatile uint32_t PADAV_CFG_REG;
    volatile uint32_t PADAW_CFG_REG;
    volatile uint32_t PADAX_CFG_REG;
    volatile uint32_t PADAY_CFG_REG;
    volatile uint32_t PADAZ_CFG_REG;
    volatile uint32_t PADBA_CFG_REG;
    volatile uint32_t PADBB_CFG_REG;
    volatile uint32_t PADBC_CFG_REG;
    volatile uint32_t PADBD_CFG_REG;
    volatile uint32_t PADBE_CFG_REG;
    volatile uint32_t PADBF_CFG_REG;
    volatile uint32_t PADBG_CFG_REG;
    volatile uint32_t PADBH_CFG_REG;
    volatile uint32_t PADBI_CFG_REG;
    volatile uint32_t PADBJ_CFG_REG;
    volatile uint32_t PADBK_CFG_REG;
    volatile uint32_t PADBL_CFG_REG;
    volatile uint32_t PADBM_CFG_REG;
    volatile uint32_t PADBN_CFG_REG;
    volatile uint32_t PADBO_CFG_REG;
    volatile uint32_t PADBP_CFG_REG;
    volatile uint32_t PADBQ_CFG_REG;
    volatile uint32_t PADBR_CFG_REG;
    volatile uint32_t PADBS_CFG_REG;
    volatile uint32_t PADBT_CFG_REG;
    volatile uint32_t PADBU_CFG_REG;
    volatile uint32_t PADBV_CFG_REG;
    volatile uint32_t PADBW_CFG_REG;
    volatile uint32_t PADBX_CFG_REG;
    volatile uint32_t PADBY_CFG_REG;
    volatile uint32_t PADBZ_CFG_REG;
    volatile uint32_t PADCA_CFG_REG;
    volatile uint32_t PADCB_CFG_REG;
    volatile uint32_t PADCC_CFG_REG;
    volatile uint32_t PADCD_CFG_REG;
    volatile uint32_t PADCE_CFG_REG;
    volatile uint32_t PADCF_CFG_REG;
    volatile uint32_t PADCG_CFG_REG;
    volatile uint32_t PADCH_CFG_REG;
    volatile uint32_t PADCI_CFG_REG;
    volatile uint32_t PADCJ_CFG_REG;
    volatile uint32_t PADCK_CFG_REG;
    volatile uint32_t PADCL_CFG_REG;
    volatile uint32_t PADCM_CFG_REG;
    volatile uint32_t PADCN_CFG_REG;
    volatile uint32_t PADCO_CFG_REG;
    volatile uint32_t PADCP_CFG_REG;
    volatile uint32_t PADCQ_CFG_REG;
    volatile uint32_t PADCR_CFG_REG;
    volatile uint32_t PADCS_CFG_REG;
    volatile uint32_t PADCT_CFG_REG;
    volatile uint32_t PADCU_CFG_REG;
    volatile uint32_t PADCV_CFG_REG;
    volatile uint32_t PADCW_CFG_REG;
    volatile uint32_t PADCX_CFG_REG;
    volatile uint32_t PADCY_CFG_REG;
    volatile uint32_t PADCZ_CFG_REG;
    volatile uint32_t PADDA_CFG_REG;
    volatile uint32_t PADDB_CFG_REG;
    volatile uint32_t PADDC_CFG_REG;
    volatile uint32_t PADDD_CFG_REG;
    volatile uint32_t PADDE_CFG_REG;
    volatile uint32_t PADDF_CFG_REG;
    volatile uint32_t PADDG_CFG_REG;
    volatile uint32_t PADDH_CFG_REG;
    volatile uint32_t PADDI_CFG_REG;
    volatile uint32_t PADDJ_CFG_REG;
    volatile uint32_t PADDK_CFG_REG;
    volatile uint32_t PADDL_CFG_REG;
    volatile uint32_t PADDM_CFG_REG;
    volatile uint32_t PADDN_CFG_REG;
    volatile uint32_t PADDO_CFG_REG;
    volatile uint32_t PADDP_CFG_REG;
    volatile uint32_t PADDQ_CFG_REG;
    volatile uint32_t PADDR_CFG_REG;
    volatile uint32_t PADDS_CFG_REG;
    volatile uint32_t PADDT_CFG_REG;
    volatile uint32_t PADDU_CFG_REG;
    volatile uint32_t PADDV_CFG_REG;
    volatile uint32_t PADDW_CFG_REG;
    volatile uint32_t PADDX_CFG_REG;
    volatile uint32_t PADDY_CFG_REG;
    volatile uint32_t PADDZ_CFG_REG;
    volatile uint32_t PADEA_CFG_REG;
    volatile uint32_t PADEB_CFG_REG;
    volatile uint32_t PADEC_CFG_REG;
    volatile uint32_t PADED_CFG_REG;
    volatile uint32_t PADEE_CFG_REG;
    volatile uint32_t PADEF_CFG_REG;
    volatile uint32_t PADEG_CFG_REG;
    volatile uint32_t PADEH_CFG_REG;
    volatile uint32_t PADEI_CFG_REG;
    volatile uint32_t PADEJ_CFG_REG;
    volatile uint32_t PADEK_CFG_REG;
    volatile uint32_t PADEL_CFG_REG;
    volatile uint32_t PADEM_CFG_REG;
    volatile uint32_t PADEN_CFG_REG;
    volatile uint32_t PADEO_CFG_REG;
    volatile uint32_t PADEP_CFG_REG;
    volatile uint8_t  Resv_496[16];
    volatile uint32_t USERMODEEN;
    volatile uint32_t PADGLBLCFGREG;
    volatile uint32_t IOCFGKICK0;
    volatile uint32_t IOCFGKICK1;
} CSL_mss_iomuxRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_MSS_IOMUX_PADAA_CFG_REG                                            (0x00000000U)
#define CSL_MSS_IOMUX_PADAB_CFG_REG                                            (0x00000004U)
#define CSL_MSS_IOMUX_PADAC_CFG_REG                                            (0x00000008U)
#define CSL_MSS_IOMUX_PADAD_CFG_REG                                            (0x0000000CU)
#define CSL_MSS_IOMUX_PADAE_CFG_REG                                            (0x00000010U)
#define CSL_MSS_IOMUX_PADAF_CFG_REG                                            (0x00000014U)
#define CSL_MSS_IOMUX_PADAG_CFG_REG                                            (0x00000018U)
#define CSL_MSS_IOMUX_PADAH_CFG_REG                                            (0x0000001CU)
#define CSL_MSS_IOMUX_PADAI_CFG_REG                                            (0x00000020U)
#define CSL_MSS_IOMUX_PADAJ_CFG_REG                                            (0x00000024U)
#define CSL_MSS_IOMUX_PADAK_CFG_REG                                            (0x00000028U)
#define CSL_MSS_IOMUX_PADAL_CFG_REG                                            (0x0000002CU)
#define CSL_MSS_IOMUX_PADAM_CFG_REG                                            (0x00000030U)
#define CSL_MSS_IOMUX_PADAN_CFG_REG                                            (0x00000034U)
#define CSL_MSS_IOMUX_PADAO_CFG_REG                                            (0x00000038U)
#define CSL_MSS_IOMUX_PADAP_CFG_REG                                            (0x0000003CU)
#define CSL_MSS_IOMUX_PADAQ_CFG_REG                                            (0x00000040U)
#define CSL_MSS_IOMUX_PADAR_CFG_REG                                            (0x00000044U)
#define CSL_MSS_IOMUX_PADAS_CFG_REG                                            (0x00000048U)
#define CSL_MSS_IOMUX_PADAT_CFG_REG                                            (0x0000004CU)
#define CSL_MSS_IOMUX_PADAU_CFG_REG                                            (0x00000050U)
#define CSL_MSS_IOMUX_PADAV_CFG_REG                                            (0x00000054U)
#define CSL_MSS_IOMUX_PADAW_CFG_REG                                            (0x00000058U)
#define CSL_MSS_IOMUX_PADAX_CFG_REG                                            (0x0000005CU)
#define CSL_MSS_IOMUX_PADAY_CFG_REG                                            (0x00000060U)
#define CSL_MSS_IOMUX_PADAZ_CFG_REG                                            (0x00000064U)
#define CSL_MSS_IOMUX_PADBA_CFG_REG                                            (0x00000068U)
#define CSL_MSS_IOMUX_PADBB_CFG_REG                                            (0x0000006CU)
#define CSL_MSS_IOMUX_PADBC_CFG_REG                                            (0x00000070U)
#define CSL_MSS_IOMUX_PADBD_CFG_REG                                            (0x00000074U)
#define CSL_MSS_IOMUX_PADBE_CFG_REG                                            (0x00000078U)
#define CSL_MSS_IOMUX_PADBF_CFG_REG                                            (0x0000007CU)
#define CSL_MSS_IOMUX_PADBG_CFG_REG                                            (0x00000080U)
#define CSL_MSS_IOMUX_PADBH_CFG_REG                                            (0x00000084U)
#define CSL_MSS_IOMUX_PADBI_CFG_REG                                            (0x00000088U)
#define CSL_MSS_IOMUX_PADBJ_CFG_REG                                            (0x0000008CU)
#define CSL_MSS_IOMUX_PADBK_CFG_REG                                            (0x00000090U)
#define CSL_MSS_IOMUX_PADBL_CFG_REG                                            (0x00000094U)
#define CSL_MSS_IOMUX_PADBM_CFG_REG                                            (0x00000098U)
#define CSL_MSS_IOMUX_PADBN_CFG_REG                                            (0x0000009CU)
#define CSL_MSS_IOMUX_PADBO_CFG_REG                                            (0x000000A0U)
#define CSL_MSS_IOMUX_PADBP_CFG_REG                                            (0x000000A4U)
#define CSL_MSS_IOMUX_PADBQ_CFG_REG                                            (0x000000A8U)
#define CSL_MSS_IOMUX_PADBR_CFG_REG                                            (0x000000ACU)
#define CSL_MSS_IOMUX_PADBS_CFG_REG                                            (0x000000B0U)
#define CSL_MSS_IOMUX_PADBT_CFG_REG                                            (0x000000B4U)
#define CSL_MSS_IOMUX_PADBU_CFG_REG                                            (0x000000B8U)
#define CSL_MSS_IOMUX_PADBV_CFG_REG                                            (0x000000BCU)
#define CSL_MSS_IOMUX_PADBW_CFG_REG                                            (0x000000C0U)
#define CSL_MSS_IOMUX_PADBX_CFG_REG                                            (0x000000C4U)
#define CSL_MSS_IOMUX_PADBY_CFG_REG                                            (0x000000C8U)
#define CSL_MSS_IOMUX_PADBZ_CFG_REG                                            (0x000000CCU)
#define CSL_MSS_IOMUX_PADCA_CFG_REG                                            (0x000000D0U)
#define CSL_MSS_IOMUX_PADCB_CFG_REG                                            (0x000000D4U)
#define CSL_MSS_IOMUX_PADCC_CFG_REG                                            (0x000000D8U)
#define CSL_MSS_IOMUX_PADCD_CFG_REG                                            (0x000000DCU)
#define CSL_MSS_IOMUX_PADCE_CFG_REG                                            (0x000000E0U)
#define CSL_MSS_IOMUX_PADCF_CFG_REG                                            (0x000000E4U)
#define CSL_MSS_IOMUX_PADCG_CFG_REG                                            (0x000000E8U)
#define CSL_MSS_IOMUX_PADCH_CFG_REG                                            (0x000000ECU)
#define CSL_MSS_IOMUX_PADCI_CFG_REG                                            (0x000000F0U)
#define CSL_MSS_IOMUX_PADCJ_CFG_REG                                            (0x000000F4U)
#define CSL_MSS_IOMUX_PADCK_CFG_REG                                            (0x000000F8U)
#define CSL_MSS_IOMUX_PADCL_CFG_REG                                            (0x000000FCU)
#define CSL_MSS_IOMUX_PADCM_CFG_REG                                            (0x00000100U)
#define CSL_MSS_IOMUX_PADCN_CFG_REG                                            (0x00000104U)
#define CSL_MSS_IOMUX_PADCO_CFG_REG                                            (0x00000108U)
#define CSL_MSS_IOMUX_PADCP_CFG_REG                                            (0x0000010CU)
#define CSL_MSS_IOMUX_PADCQ_CFG_REG                                            (0x00000110U)
#define CSL_MSS_IOMUX_PADCR_CFG_REG                                            (0x00000114U)
#define CSL_MSS_IOMUX_PADCS_CFG_REG                                            (0x00000118U)
#define CSL_MSS_IOMUX_PADCT_CFG_REG                                            (0x0000011CU)
#define CSL_MSS_IOMUX_PADCU_CFG_REG                                            (0x00000120U)
#define CSL_MSS_IOMUX_PADCV_CFG_REG                                            (0x00000124U)
#define CSL_MSS_IOMUX_PADCW_CFG_REG                                            (0x00000128U)
#define CSL_MSS_IOMUX_PADCX_CFG_REG                                            (0x0000012CU)
#define CSL_MSS_IOMUX_PADCY_CFG_REG                                            (0x00000130U)
#define CSL_MSS_IOMUX_PADCZ_CFG_REG                                            (0x00000134U)
#define CSL_MSS_IOMUX_PADDA_CFG_REG                                            (0x00000138U)
#define CSL_MSS_IOMUX_PADDB_CFG_REG                                            (0x0000013CU)
#define CSL_MSS_IOMUX_PADDC_CFG_REG                                            (0x00000140U)
#define CSL_MSS_IOMUX_PADDD_CFG_REG                                            (0x00000144U)
#define CSL_MSS_IOMUX_PADDE_CFG_REG                                            (0x00000148U)
#define CSL_MSS_IOMUX_PADDF_CFG_REG                                            (0x0000014CU)
#define CSL_MSS_IOMUX_PADDG_CFG_REG                                            (0x00000150U)
#define CSL_MSS_IOMUX_PADDH_CFG_REG                                            (0x00000154U)
#define CSL_MSS_IOMUX_PADDI_CFG_REG                                            (0x00000158U)
#define CSL_MSS_IOMUX_PADDJ_CFG_REG                                            (0x0000015CU)
#define CSL_MSS_IOMUX_PADDK_CFG_REG                                            (0x00000160U)
#define CSL_MSS_IOMUX_PADDL_CFG_REG                                            (0x00000164U)
#define CSL_MSS_IOMUX_PADDM_CFG_REG                                            (0x00000168U)
#define CSL_MSS_IOMUX_PADDN_CFG_REG                                            (0x0000016CU)
#define CSL_MSS_IOMUX_PADDO_CFG_REG                                            (0x00000170U)
#define CSL_MSS_IOMUX_PADDP_CFG_REG                                            (0x00000174U)
#define CSL_MSS_IOMUX_PADDQ_CFG_REG                                            (0x00000178U)
#define CSL_MSS_IOMUX_PADDR_CFG_REG                                            (0x0000017CU)
#define CSL_MSS_IOMUX_PADDS_CFG_REG                                            (0x00000180U)
#define CSL_MSS_IOMUX_PADDT_CFG_REG                                            (0x00000184U)
#define CSL_MSS_IOMUX_PADDU_CFG_REG                                            (0x00000188U)
#define CSL_MSS_IOMUX_PADDV_CFG_REG                                            (0x0000018CU)
#define CSL_MSS_IOMUX_PADDW_CFG_REG                                            (0x00000190U)
#define CSL_MSS_IOMUX_PADDX_CFG_REG                                            (0x00000194U)
#define CSL_MSS_IOMUX_PADDY_CFG_REG                                            (0x00000198U)
#define CSL_MSS_IOMUX_PADDZ_CFG_REG                                            (0x0000019CU)
#define CSL_MSS_IOMUX_PADEA_CFG_REG                                            (0x000001A0U)
#define CSL_MSS_IOMUX_PADEB_CFG_REG                                            (0x000001A4U)
#define CSL_MSS_IOMUX_PADEC_CFG_REG                                            (0x000001A8U)
#define CSL_MSS_IOMUX_PADED_CFG_REG                                            (0x000001ACU)
#define CSL_MSS_IOMUX_PADEE_CFG_REG                                            (0x000001B0U)
#define CSL_MSS_IOMUX_PADEF_CFG_REG                                            (0x000001B4U)
#define CSL_MSS_IOMUX_PADEG_CFG_REG                                            (0x000001B8U)
#define CSL_MSS_IOMUX_PADEH_CFG_REG                                            (0x000001BCU)
#define CSL_MSS_IOMUX_PADEI_CFG_REG                                            (0x000001C0U)
#define CSL_MSS_IOMUX_PADEJ_CFG_REG                                            (0x000001C4U)
#define CSL_MSS_IOMUX_PADEK_CFG_REG                                            (0x000001C8U)
#define CSL_MSS_IOMUX_PADEL_CFG_REG                                            (0x000001CCU)
#define CSL_MSS_IOMUX_PADEM_CFG_REG                                            (0x000001D0U)
#define CSL_MSS_IOMUX_PADEN_CFG_REG                                            (0x000001D4U)
#define CSL_MSS_IOMUX_PADEO_CFG_REG                                            (0x000001D8U)
#define CSL_MSS_IOMUX_PADEP_CFG_REG                                            (0x000001DCU)
#define CSL_MSS_IOMUX_USERMODEEN                                               (0x000001F0U)
#define CSL_MSS_IOMUX_PADGLBLCFGREG                                            (0x000001F4U)
#define CSL_MSS_IOMUX_IOCFGKICK0                                               (0x000001F8U)
#define CSL_MSS_IOMUX_IOCFGKICK1                                               (0x000001FCU)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* PADAA_CFG_REG */

#define CSL_MSS_IOMUX_PADAA_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADAA_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADAA_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADAA_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADAA_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADAA_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADAA_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADAA_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADAA_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADAA_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADAA_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADAA_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADAA_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADAA_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADAA_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADAA_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADAA_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADAA_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADAA_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADAA_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADAA_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADAA_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADAA_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADAA_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADAA_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADAA_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADAA_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADAA_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADAA_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADAA_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADAA_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADAA_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADAA_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADAA_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADAA_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADAA_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADAA_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADAB_CFG_REG */

#define CSL_MSS_IOMUX_PADAB_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADAB_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADAB_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADAB_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADAB_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADAB_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADAB_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADAB_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADAB_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADAB_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADAB_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADAB_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADAB_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADAB_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADAB_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADAB_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADAB_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADAB_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADAB_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADAB_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADAB_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADAB_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADAB_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADAB_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADAB_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADAB_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADAB_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADAB_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADAB_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADAB_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADAB_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADAB_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADAB_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADAB_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADAB_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADAB_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADAB_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADAC_CFG_REG */

#define CSL_MSS_IOMUX_PADAC_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADAC_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADAC_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADAC_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADAC_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADAC_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADAC_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADAC_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADAC_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADAC_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADAC_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADAC_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADAC_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADAC_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADAC_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADAC_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADAC_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADAC_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADAC_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADAC_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADAC_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADAC_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADAC_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADAC_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADAC_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADAC_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADAC_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADAC_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADAC_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADAC_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADAC_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADAC_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADAC_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADAC_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADAC_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADAC_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADAC_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADAD_CFG_REG */

#define CSL_MSS_IOMUX_PADAD_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADAD_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADAD_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADAD_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADAD_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADAD_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADAD_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADAD_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADAD_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADAD_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADAD_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADAD_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADAD_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADAD_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADAD_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADAD_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADAD_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADAD_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADAD_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADAD_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADAD_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADAD_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADAD_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADAD_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADAD_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADAD_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADAD_CFG_REG_PUPDSEL_RESETVAL                           (0x00000001U)
#define CSL_MSS_IOMUX_PADAD_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADAD_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADAD_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADAD_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADAD_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADAD_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADAD_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADAD_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADAD_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADAD_CFG_REG_RESETVAL                                   (0x000002C1U)

/* PADAE_CFG_REG */

#define CSL_MSS_IOMUX_PADAE_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADAE_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADAE_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADAE_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADAE_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADAE_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADAE_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADAE_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADAE_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADAE_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADAE_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADAE_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADAE_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADAE_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADAE_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADAE_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADAE_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADAE_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADAE_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADAE_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADAE_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADAE_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADAE_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADAE_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADAE_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADAE_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADAE_CFG_REG_PUPDSEL_RESETVAL                           (0x00000001U)
#define CSL_MSS_IOMUX_PADAE_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADAE_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADAE_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADAE_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADAE_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADAE_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADAE_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADAE_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADAE_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADAE_CFG_REG_RESETVAL                                   (0x000002C1U)

/* PADAF_CFG_REG */

#define CSL_MSS_IOMUX_PADAF_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADAF_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADAF_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADAF_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADAF_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADAF_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADAF_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADAF_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADAF_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADAF_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADAF_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADAF_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADAF_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADAF_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADAF_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADAF_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADAF_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADAF_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADAF_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADAF_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADAF_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADAF_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADAF_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADAF_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADAF_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADAF_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADAF_CFG_REG_PUPDSEL_RESETVAL                           (0x00000001U)
#define CSL_MSS_IOMUX_PADAF_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADAF_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADAF_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADAF_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADAF_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADAF_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADAF_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADAF_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADAF_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADAF_CFG_REG_RESETVAL                                   (0x000002C1U)

/* PADAG_CFG_REG */

#define CSL_MSS_IOMUX_PADAG_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADAG_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADAG_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADAG_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADAG_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADAG_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADAG_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADAG_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADAG_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADAG_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADAG_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADAG_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADAG_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADAG_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADAG_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADAG_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADAG_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADAG_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADAG_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADAG_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADAG_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADAG_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADAG_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADAG_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADAG_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADAG_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADAG_CFG_REG_PUPDSEL_RESETVAL                           (0x00000001U)
#define CSL_MSS_IOMUX_PADAG_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADAG_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADAG_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADAG_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADAG_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADAG_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADAG_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADAG_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADAG_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADAG_CFG_REG_RESETVAL                                   (0x000002C1U)

/* PADAH_CFG_REG */

#define CSL_MSS_IOMUX_PADAH_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADAH_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADAH_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADAH_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADAH_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADAH_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADAH_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADAH_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADAH_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADAH_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADAH_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADAH_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADAH_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADAH_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADAH_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADAH_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADAH_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADAH_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADAH_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADAH_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADAH_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADAH_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADAH_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADAH_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADAH_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADAH_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADAH_CFG_REG_PUPDSEL_RESETVAL                           (0x00000001U)
#define CSL_MSS_IOMUX_PADAH_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADAH_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADAH_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADAH_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADAH_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADAH_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADAH_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADAH_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADAH_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADAH_CFG_REG_RESETVAL                                   (0x000002C1U)

/* PADAI_CFG_REG */

#define CSL_MSS_IOMUX_PADAI_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADAI_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADAI_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADAI_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADAI_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADAI_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADAI_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADAI_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADAI_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADAI_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADAI_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADAI_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADAI_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADAI_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADAI_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADAI_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADAI_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADAI_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADAI_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADAI_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADAI_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADAI_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADAI_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADAI_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADAI_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADAI_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADAI_CFG_REG_PUPDSEL_RESETVAL                           (0x00000001U)
#define CSL_MSS_IOMUX_PADAI_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADAI_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADAI_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADAI_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADAI_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADAI_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADAI_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADAI_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADAI_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADAI_CFG_REG_RESETVAL                                   (0x000002C1U)

/* PADAJ_CFG_REG */

#define CSL_MSS_IOMUX_PADAJ_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADAJ_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADAJ_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADAJ_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADAJ_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADAJ_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADAJ_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADAJ_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADAJ_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADAJ_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADAJ_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADAJ_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADAJ_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADAJ_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADAJ_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADAJ_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADAJ_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADAJ_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADAJ_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADAJ_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADAJ_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADAJ_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADAJ_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADAJ_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADAJ_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADAJ_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADAJ_CFG_REG_PUPDSEL_RESETVAL                           (0x00000001U)
#define CSL_MSS_IOMUX_PADAJ_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADAJ_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADAJ_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADAJ_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADAJ_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADAJ_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADAJ_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADAJ_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADAJ_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADAJ_CFG_REG_RESETVAL                                   (0x000002C1U)

/* PADAK_CFG_REG */

#define CSL_MSS_IOMUX_PADAK_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADAK_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADAK_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADAK_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADAK_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADAK_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADAK_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADAK_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADAK_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADAK_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADAK_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADAK_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADAK_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADAK_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADAK_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADAK_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADAK_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADAK_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADAK_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADAK_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADAK_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADAK_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADAK_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADAK_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADAK_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADAK_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADAK_CFG_REG_PUPDSEL_RESETVAL                           (0x00000001U)
#define CSL_MSS_IOMUX_PADAK_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADAK_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADAK_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADAK_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADAK_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADAK_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADAK_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADAK_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADAK_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADAK_CFG_REG_RESETVAL                                   (0x000002C1U)

/* PADAL_CFG_REG */

#define CSL_MSS_IOMUX_PADAL_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADAL_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADAL_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADAL_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADAL_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADAL_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADAL_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADAL_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADAL_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADAL_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADAL_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADAL_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADAL_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADAL_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADAL_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADAL_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADAL_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADAL_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADAL_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADAL_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADAL_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADAL_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADAL_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADAL_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADAL_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADAL_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADAL_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADAL_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADAL_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADAL_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADAL_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADAL_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADAL_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADAL_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADAL_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADAL_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADAL_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADAM_CFG_REG */

#define CSL_MSS_IOMUX_PADAM_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADAM_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADAM_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADAM_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADAM_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADAM_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADAM_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADAM_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADAM_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADAM_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADAM_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADAM_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADAM_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADAM_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADAM_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADAM_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADAM_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADAM_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADAM_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADAM_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADAM_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADAM_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADAM_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADAM_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADAM_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADAM_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADAM_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADAM_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADAM_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADAM_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADAM_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADAM_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADAM_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADAM_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADAM_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADAM_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADAM_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADAN_CFG_REG */

#define CSL_MSS_IOMUX_PADAN_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADAN_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADAN_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADAN_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADAN_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADAN_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADAN_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADAN_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADAN_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADAN_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADAN_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADAN_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADAN_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADAN_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADAN_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADAN_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADAN_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADAN_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADAN_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADAN_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADAN_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADAN_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADAN_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADAN_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADAN_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADAN_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADAN_CFG_REG_PUPDSEL_RESETVAL                           (0x00000001U)
#define CSL_MSS_IOMUX_PADAN_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADAN_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADAN_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADAN_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADAN_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADAN_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADAN_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADAN_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADAN_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADAN_CFG_REG_RESETVAL                                   (0x000002C1U)

/* PADAO_CFG_REG */

#define CSL_MSS_IOMUX_PADAO_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADAO_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADAO_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADAO_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADAO_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADAO_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADAO_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADAO_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADAO_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADAO_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADAO_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADAO_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADAO_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADAO_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADAO_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADAO_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADAO_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADAO_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADAO_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADAO_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADAO_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADAO_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADAO_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADAO_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADAO_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADAO_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADAO_CFG_REG_PUPDSEL_RESETVAL                           (0x00000001U)
#define CSL_MSS_IOMUX_PADAO_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADAO_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADAO_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADAO_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADAO_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADAO_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADAO_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADAO_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADAO_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADAO_CFG_REG_RESETVAL                                   (0x000002C1U)

/* PADAP_CFG_REG */

#define CSL_MSS_IOMUX_PADAP_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADAP_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADAP_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADAP_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADAP_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADAP_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADAP_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADAP_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADAP_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADAP_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADAP_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADAP_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADAP_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADAP_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADAP_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADAP_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADAP_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADAP_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADAP_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADAP_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADAP_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADAP_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADAP_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADAP_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADAP_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADAP_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADAP_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADAP_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADAP_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADAP_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADAP_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADAP_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADAP_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADAP_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADAP_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADAP_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADAP_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADAQ_CFG_REG */

#define CSL_MSS_IOMUX_PADAQ_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADAQ_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADAQ_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADAQ_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADAQ_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADAQ_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADAQ_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADAQ_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADAQ_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADAQ_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADAQ_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADAQ_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADAQ_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADAQ_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADAQ_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADAQ_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADAQ_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADAQ_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADAQ_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADAQ_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADAQ_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADAQ_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADAQ_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADAQ_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADAQ_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADAQ_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADAQ_CFG_REG_PUPDSEL_RESETVAL                           (0x00000001U)
#define CSL_MSS_IOMUX_PADAQ_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADAQ_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADAQ_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADAQ_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADAQ_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADAQ_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADAQ_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADAQ_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADAQ_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADAQ_CFG_REG_RESETVAL                                   (0x000002C1U)

/* PADAR_CFG_REG */

#define CSL_MSS_IOMUX_PADAR_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADAR_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADAR_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000000U)
#define CSL_MSS_IOMUX_PADAR_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADAR_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADAR_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADAR_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADAR_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADAR_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADAR_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADAR_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADAR_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADAR_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADAR_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADAR_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADAR_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADAR_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADAR_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADAR_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADAR_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADAR_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADAR_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADAR_CFG_REG_PI_RESETVAL                                (0x00000001U)
#define CSL_MSS_IOMUX_PADAR_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADAR_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADAR_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADAR_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADAR_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADAR_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADAR_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADAR_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADAR_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADAR_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADAR_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADAR_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADAR_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADAR_CFG_REG_RESETVAL                                   (0x00000100U)

/* PADAS_CFG_REG */

#define CSL_MSS_IOMUX_PADAS_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADAS_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADAS_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000000U)
#define CSL_MSS_IOMUX_PADAS_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADAS_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADAS_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADAS_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADAS_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADAS_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADAS_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADAS_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADAS_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADAS_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADAS_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADAS_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADAS_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADAS_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADAS_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADAS_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADAS_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADAS_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADAS_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADAS_CFG_REG_PI_RESETVAL                                (0x00000001U)
#define CSL_MSS_IOMUX_PADAS_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADAS_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADAS_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADAS_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADAS_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADAS_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADAS_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADAS_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADAS_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADAS_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADAS_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADAS_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADAS_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADAS_CFG_REG_RESETVAL                                   (0x00000100U)

/* PADAT_CFG_REG */

#define CSL_MSS_IOMUX_PADAT_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADAT_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADAT_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000000U)
#define CSL_MSS_IOMUX_PADAT_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADAT_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADAT_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADAT_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADAT_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADAT_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADAT_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADAT_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADAT_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADAT_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADAT_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADAT_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADAT_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADAT_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADAT_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADAT_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADAT_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADAT_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADAT_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADAT_CFG_REG_PI_RESETVAL                                (0x00000001U)
#define CSL_MSS_IOMUX_PADAT_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADAT_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADAT_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADAT_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADAT_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADAT_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADAT_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADAT_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADAT_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADAT_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADAT_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADAT_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADAT_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADAT_CFG_REG_RESETVAL                                   (0x00000100U)

/* PADAU_CFG_REG */

#define CSL_MSS_IOMUX_PADAU_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADAU_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADAU_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADAU_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADAU_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADAU_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADAU_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADAU_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADAU_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADAU_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADAU_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADAU_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADAU_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADAU_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADAU_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADAU_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADAU_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADAU_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADAU_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADAU_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADAU_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADAU_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADAU_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADAU_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADAU_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADAU_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADAU_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADAU_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADAU_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADAU_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADAU_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADAU_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADAU_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADAU_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADAU_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADAU_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADAU_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADAV_CFG_REG */

#define CSL_MSS_IOMUX_PADAV_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADAV_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADAV_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADAV_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADAV_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADAV_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADAV_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADAV_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADAV_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADAV_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADAV_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADAV_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADAV_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADAV_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADAV_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADAV_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADAV_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADAV_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADAV_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADAV_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADAV_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADAV_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADAV_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADAV_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADAV_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADAV_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADAV_CFG_REG_PUPDSEL_RESETVAL                           (0x00000001U)
#define CSL_MSS_IOMUX_PADAV_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADAV_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADAV_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADAV_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADAV_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADAV_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADAV_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADAV_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADAV_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADAV_CFG_REG_RESETVAL                                   (0x000002C1U)

/* PADAW_CFG_REG */

#define CSL_MSS_IOMUX_PADAW_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADAW_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADAW_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADAW_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADAW_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADAW_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADAW_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADAW_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADAW_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADAW_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADAW_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADAW_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADAW_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADAW_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADAW_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADAW_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADAW_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADAW_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADAW_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADAW_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADAW_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADAW_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADAW_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADAW_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADAW_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADAW_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADAW_CFG_REG_PUPDSEL_RESETVAL                           (0x00000001U)
#define CSL_MSS_IOMUX_PADAW_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADAW_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADAW_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADAW_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADAW_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADAW_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADAW_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADAW_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADAW_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADAW_CFG_REG_RESETVAL                                   (0x000002C1U)

/* PADAX_CFG_REG */

#define CSL_MSS_IOMUX_PADAX_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADAX_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADAX_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADAX_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADAX_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADAX_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADAX_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADAX_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADAX_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADAX_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADAX_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADAX_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADAX_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADAX_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADAX_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADAX_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADAX_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADAX_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADAX_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADAX_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADAX_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADAX_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADAX_CFG_REG_PI_RESETVAL                                (0x00000001U)
#define CSL_MSS_IOMUX_PADAX_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADAX_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADAX_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADAX_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADAX_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADAX_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADAX_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADAX_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADAX_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADAX_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADAX_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADAX_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADAX_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADAX_CFG_REG_RESETVAL                                   (0x00000101U)

/* PADAY_CFG_REG */

#define CSL_MSS_IOMUX_PADAY_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADAY_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADAY_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADAY_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADAY_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADAY_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADAY_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADAY_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADAY_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADAY_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADAY_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADAY_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADAY_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADAY_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADAY_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADAY_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADAY_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADAY_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADAY_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADAY_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADAY_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADAY_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADAY_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADAY_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADAY_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADAY_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADAY_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADAY_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADAY_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADAY_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADAY_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADAY_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADAY_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADAY_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADAY_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADAY_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADAY_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADAZ_CFG_REG */

#define CSL_MSS_IOMUX_PADAZ_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADAZ_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADAZ_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADAZ_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADAZ_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADAZ_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADAZ_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADAZ_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADAZ_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADAZ_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADAZ_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADAZ_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADAZ_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADAZ_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADAZ_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADAZ_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADAZ_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADAZ_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADAZ_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADAZ_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADAZ_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADAZ_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADAZ_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADAZ_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADAZ_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADAZ_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADAZ_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADAZ_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADAZ_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADAZ_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADAZ_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADAZ_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADAZ_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADAZ_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADAZ_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADAZ_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADAZ_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADBA_CFG_REG */

#define CSL_MSS_IOMUX_PADBA_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADBA_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADBA_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADBA_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADBA_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADBA_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADBA_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADBA_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADBA_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADBA_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADBA_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADBA_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADBA_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADBA_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADBA_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADBA_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADBA_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADBA_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADBA_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADBA_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADBA_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADBA_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADBA_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADBA_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADBA_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADBA_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADBA_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADBA_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADBA_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADBA_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADBA_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADBA_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADBA_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADBA_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADBA_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADBA_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADBA_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADBB_CFG_REG */

#define CSL_MSS_IOMUX_PADBB_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADBB_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADBB_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADBB_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADBB_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADBB_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADBB_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADBB_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADBB_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADBB_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADBB_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADBB_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADBB_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADBB_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADBB_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADBB_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADBB_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADBB_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADBB_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADBB_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADBB_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADBB_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADBB_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADBB_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADBB_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADBB_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADBB_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADBB_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADBB_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADBB_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADBB_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADBB_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADBB_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADBB_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADBB_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADBB_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADBB_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADBC_CFG_REG */

#define CSL_MSS_IOMUX_PADBC_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADBC_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADBC_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADBC_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADBC_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADBC_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADBC_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADBC_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADBC_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADBC_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADBC_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADBC_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADBC_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADBC_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADBC_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADBC_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADBC_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADBC_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADBC_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADBC_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADBC_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADBC_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADBC_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADBC_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADBC_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADBC_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADBC_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADBC_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADBC_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADBC_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADBC_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADBC_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADBC_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADBC_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADBC_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADBC_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADBC_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADBD_CFG_REG */

#define CSL_MSS_IOMUX_PADBD_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADBD_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADBD_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADBD_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADBD_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADBD_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADBD_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADBD_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADBD_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADBD_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADBD_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADBD_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADBD_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADBD_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADBD_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADBD_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADBD_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADBD_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADBD_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADBD_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADBD_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADBD_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADBD_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADBD_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADBD_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADBD_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADBD_CFG_REG_PUPDSEL_RESETVAL                           (0x00000001U)
#define CSL_MSS_IOMUX_PADBD_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADBD_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADBD_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADBD_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADBD_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADBD_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADBD_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADBD_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADBD_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADBD_CFG_REG_RESETVAL                                   (0x000002C1U)

/* PADBE_CFG_REG */

#define CSL_MSS_IOMUX_PADBE_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADBE_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADBE_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADBE_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADBE_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADBE_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADBE_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADBE_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADBE_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADBE_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADBE_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADBE_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADBE_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADBE_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADBE_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADBE_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADBE_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADBE_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADBE_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADBE_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADBE_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADBE_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADBE_CFG_REG_PI_RESETVAL                                (0x00000001U)
#define CSL_MSS_IOMUX_PADBE_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADBE_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADBE_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADBE_CFG_REG_PUPDSEL_RESETVAL                           (0x00000001U)
#define CSL_MSS_IOMUX_PADBE_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADBE_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADBE_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADBE_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADBE_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADBE_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADBE_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADBE_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADBE_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADBE_CFG_REG_RESETVAL                                   (0x00000301U)

/* PADBF_CFG_REG */

#define CSL_MSS_IOMUX_PADBF_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADBF_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADBF_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADBF_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADBF_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADBF_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADBF_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADBF_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADBF_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADBF_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADBF_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADBF_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADBF_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADBF_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADBF_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADBF_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADBF_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADBF_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADBF_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADBF_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADBF_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADBF_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADBF_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADBF_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADBF_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADBF_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADBF_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADBF_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADBF_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADBF_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADBF_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADBF_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADBF_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADBF_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADBF_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADBF_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADBF_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADBG_CFG_REG */

#define CSL_MSS_IOMUX_PADBG_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADBG_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADBG_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADBG_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADBG_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADBG_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADBG_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADBG_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADBG_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADBG_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADBG_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADBG_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADBG_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADBG_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADBG_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADBG_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADBG_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADBG_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADBG_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADBG_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADBG_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADBG_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADBG_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADBG_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADBG_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADBG_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADBG_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADBG_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADBG_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADBG_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADBG_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADBG_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADBG_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADBG_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADBG_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADBG_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADBG_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADBH_CFG_REG */

#define CSL_MSS_IOMUX_PADBH_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADBH_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADBH_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADBH_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADBH_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADBH_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADBH_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADBH_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADBH_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADBH_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADBH_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADBH_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADBH_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADBH_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADBH_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADBH_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADBH_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADBH_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADBH_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADBH_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADBH_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADBH_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADBH_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADBH_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADBH_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADBH_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADBH_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADBH_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADBH_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADBH_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADBH_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADBH_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADBH_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADBH_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADBH_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADBH_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADBH_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADBI_CFG_REG */

#define CSL_MSS_IOMUX_PADBI_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADBI_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADBI_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADBI_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADBI_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADBI_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADBI_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADBI_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADBI_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADBI_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADBI_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADBI_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADBI_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADBI_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADBI_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADBI_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADBI_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADBI_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADBI_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADBI_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADBI_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADBI_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADBI_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADBI_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADBI_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADBI_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADBI_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADBI_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADBI_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADBI_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADBI_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADBI_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADBI_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADBI_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADBI_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADBI_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADBI_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADBJ_CFG_REG */

#define CSL_MSS_IOMUX_PADBJ_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADBJ_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADBJ_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADBJ_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADBJ_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADBJ_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADBJ_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADBJ_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADBJ_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADBJ_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADBJ_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADBJ_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADBJ_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADBJ_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADBJ_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADBJ_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADBJ_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADBJ_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADBJ_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADBJ_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADBJ_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADBJ_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADBJ_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADBJ_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADBJ_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADBJ_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADBJ_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADBJ_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADBJ_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADBJ_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADBJ_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADBJ_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADBJ_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADBJ_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADBJ_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADBJ_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADBJ_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADBK_CFG_REG */

#define CSL_MSS_IOMUX_PADBK_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADBK_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADBK_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADBK_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADBK_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADBK_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADBK_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADBK_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADBK_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADBK_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADBK_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADBK_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADBK_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADBK_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADBK_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADBK_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADBK_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADBK_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADBK_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADBK_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADBK_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADBK_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADBK_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADBK_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADBK_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADBK_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADBK_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADBK_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADBK_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADBK_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADBK_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADBK_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADBK_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADBK_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADBK_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADBK_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADBK_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADBL_CFG_REG */

#define CSL_MSS_IOMUX_PADBL_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADBL_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADBL_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADBL_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADBL_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADBL_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADBL_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADBL_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADBL_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADBL_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADBL_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADBL_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADBL_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADBL_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADBL_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADBL_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADBL_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADBL_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADBL_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADBL_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADBL_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADBL_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADBL_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADBL_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADBL_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADBL_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADBL_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADBL_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADBL_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADBL_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADBL_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADBL_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADBL_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADBL_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADBL_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADBL_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADBL_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADBM_CFG_REG */

#define CSL_MSS_IOMUX_PADBM_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADBM_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADBM_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADBM_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADBM_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADBM_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADBM_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADBM_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADBM_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADBM_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADBM_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADBM_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADBM_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADBM_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADBM_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADBM_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADBM_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADBM_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADBM_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADBM_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADBM_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADBM_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADBM_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADBM_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADBM_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADBM_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADBM_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADBM_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADBM_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADBM_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADBM_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADBM_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADBM_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADBM_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADBM_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADBM_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADBM_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADBN_CFG_REG */

#define CSL_MSS_IOMUX_PADBN_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADBN_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADBN_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADBN_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADBN_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADBN_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADBN_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADBN_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADBN_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADBN_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADBN_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADBN_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADBN_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADBN_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADBN_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADBN_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADBN_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADBN_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADBN_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADBN_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADBN_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADBN_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADBN_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADBN_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADBN_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADBN_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADBN_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADBN_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADBN_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADBN_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADBN_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADBN_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADBN_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADBN_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADBN_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADBN_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADBN_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADBO_CFG_REG */

#define CSL_MSS_IOMUX_PADBO_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADBO_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADBO_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADBO_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADBO_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADBO_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADBO_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADBO_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADBO_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADBO_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADBO_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADBO_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADBO_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADBO_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADBO_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADBO_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADBO_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADBO_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADBO_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADBO_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADBO_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADBO_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADBO_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADBO_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADBO_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADBO_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADBO_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADBO_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADBO_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADBO_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADBO_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADBO_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADBO_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADBO_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADBO_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADBO_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADBO_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADBP_CFG_REG */

#define CSL_MSS_IOMUX_PADBP_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADBP_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADBP_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADBP_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADBP_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADBP_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADBP_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADBP_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADBP_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADBP_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADBP_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADBP_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADBP_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADBP_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADBP_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADBP_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADBP_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADBP_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADBP_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADBP_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADBP_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADBP_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADBP_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADBP_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADBP_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADBP_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADBP_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADBP_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADBP_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADBP_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADBP_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADBP_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADBP_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADBP_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADBP_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADBP_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADBP_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADBQ_CFG_REG */

#define CSL_MSS_IOMUX_PADBQ_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADBQ_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADBQ_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADBQ_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADBQ_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADBQ_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADBQ_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADBQ_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADBQ_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADBQ_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADBQ_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADBQ_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADBQ_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADBQ_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADBQ_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADBQ_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADBQ_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADBQ_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADBQ_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADBQ_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADBQ_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADBQ_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADBQ_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADBQ_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADBQ_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADBQ_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADBQ_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADBQ_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADBQ_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADBQ_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADBQ_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADBQ_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADBQ_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADBQ_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADBQ_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADBQ_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADBQ_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADBR_CFG_REG */

#define CSL_MSS_IOMUX_PADBR_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADBR_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADBR_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADBR_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADBR_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADBR_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADBR_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADBR_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADBR_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADBR_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADBR_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADBR_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADBR_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADBR_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADBR_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADBR_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADBR_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADBR_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADBR_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADBR_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADBR_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADBR_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADBR_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADBR_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADBR_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADBR_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADBR_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADBR_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADBR_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADBR_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADBR_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADBR_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADBR_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADBR_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADBR_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADBR_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADBR_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADBS_CFG_REG */

#define CSL_MSS_IOMUX_PADBS_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADBS_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADBS_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADBS_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADBS_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADBS_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADBS_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADBS_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADBS_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADBS_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADBS_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADBS_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADBS_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADBS_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADBS_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADBS_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADBS_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADBS_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADBS_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADBS_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADBS_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADBS_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADBS_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADBS_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADBS_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADBS_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADBS_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADBS_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADBS_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADBS_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADBS_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADBS_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADBS_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADBS_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADBS_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADBS_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADBS_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADBT_CFG_REG */

#define CSL_MSS_IOMUX_PADBT_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADBT_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADBT_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADBT_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADBT_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADBT_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADBT_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADBT_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADBT_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADBT_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADBT_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADBT_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADBT_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADBT_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADBT_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADBT_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADBT_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADBT_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADBT_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADBT_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADBT_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADBT_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADBT_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADBT_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADBT_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADBT_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADBT_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADBT_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADBT_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADBT_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADBT_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADBT_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADBT_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADBT_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADBT_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADBT_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADBT_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADBU_CFG_REG */

#define CSL_MSS_IOMUX_PADBU_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADBU_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADBU_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADBU_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADBU_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADBU_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADBU_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADBU_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADBU_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADBU_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADBU_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADBU_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADBU_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADBU_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADBU_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADBU_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADBU_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADBU_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADBU_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADBU_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADBU_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADBU_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADBU_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADBU_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADBU_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADBU_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADBU_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADBU_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADBU_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADBU_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADBU_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADBU_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADBU_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADBU_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADBU_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADBU_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADBU_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADBV_CFG_REG */

#define CSL_MSS_IOMUX_PADBV_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADBV_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADBV_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADBV_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADBV_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADBV_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADBV_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADBV_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADBV_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADBV_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADBV_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADBV_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADBV_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADBV_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADBV_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADBV_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADBV_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADBV_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADBV_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADBV_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADBV_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADBV_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADBV_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADBV_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADBV_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADBV_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADBV_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADBV_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADBV_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADBV_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADBV_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADBV_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADBV_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADBV_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADBV_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADBV_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADBV_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADBW_CFG_REG */

#define CSL_MSS_IOMUX_PADBW_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADBW_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADBW_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADBW_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADBW_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADBW_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADBW_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADBW_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADBW_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADBW_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADBW_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADBW_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADBW_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADBW_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADBW_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADBW_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADBW_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADBW_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADBW_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADBW_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADBW_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADBW_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADBW_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADBW_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADBW_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADBW_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADBW_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADBW_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADBW_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADBW_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADBW_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADBW_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADBW_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADBW_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADBW_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADBW_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADBW_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADBX_CFG_REG */

#define CSL_MSS_IOMUX_PADBX_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADBX_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADBX_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADBX_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADBX_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADBX_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADBX_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADBX_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADBX_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADBX_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADBX_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADBX_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADBX_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADBX_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADBX_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADBX_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADBX_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADBX_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADBX_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADBX_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADBX_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADBX_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADBX_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADBX_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADBX_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADBX_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADBX_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADBX_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADBX_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADBX_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADBX_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADBX_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADBX_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADBX_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADBX_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADBX_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADBX_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADBY_CFG_REG */

#define CSL_MSS_IOMUX_PADBY_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADBY_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADBY_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADBY_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADBY_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADBY_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADBY_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADBY_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADBY_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADBY_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADBY_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADBY_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADBY_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADBY_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADBY_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADBY_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADBY_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADBY_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADBY_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADBY_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADBY_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADBY_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADBY_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADBY_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADBY_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADBY_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADBY_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADBY_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADBY_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADBY_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADBY_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADBY_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADBY_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADBY_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADBY_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADBY_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADBY_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADBZ_CFG_REG */

#define CSL_MSS_IOMUX_PADBZ_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADBZ_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADBZ_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADBZ_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADBZ_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADBZ_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADBZ_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADBZ_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADBZ_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADBZ_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADBZ_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADBZ_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADBZ_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADBZ_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADBZ_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADBZ_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADBZ_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADBZ_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADBZ_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADBZ_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADBZ_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADBZ_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADBZ_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADBZ_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADBZ_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADBZ_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADBZ_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADBZ_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADBZ_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADBZ_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADBZ_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADBZ_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADBZ_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADBZ_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADBZ_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADBZ_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADBZ_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADCA_CFG_REG */

#define CSL_MSS_IOMUX_PADCA_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADCA_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADCA_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADCA_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADCA_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADCA_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADCA_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADCA_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADCA_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADCA_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADCA_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADCA_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADCA_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADCA_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADCA_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADCA_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADCA_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADCA_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADCA_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADCA_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADCA_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADCA_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADCA_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADCA_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADCA_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADCA_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADCA_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADCA_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADCA_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADCA_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADCA_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADCA_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADCA_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADCA_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADCA_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADCA_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADCA_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADCB_CFG_REG */

#define CSL_MSS_IOMUX_PADCB_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADCB_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADCB_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADCB_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADCB_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADCB_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADCB_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADCB_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADCB_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADCB_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADCB_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADCB_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADCB_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADCB_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADCB_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADCB_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADCB_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADCB_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADCB_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADCB_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADCB_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADCB_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADCB_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADCB_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADCB_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADCB_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADCB_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADCB_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADCB_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADCB_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADCB_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADCB_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADCB_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADCB_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADCB_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADCB_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADCB_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADCC_CFG_REG */

#define CSL_MSS_IOMUX_PADCC_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADCC_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADCC_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADCC_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADCC_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADCC_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADCC_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADCC_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADCC_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADCC_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADCC_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADCC_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADCC_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADCC_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADCC_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADCC_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADCC_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADCC_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADCC_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADCC_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADCC_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADCC_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADCC_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADCC_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADCC_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADCC_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADCC_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADCC_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADCC_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADCC_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADCC_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADCC_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADCC_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADCC_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADCC_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADCC_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADCC_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADCD_CFG_REG */

#define CSL_MSS_IOMUX_PADCD_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADCD_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADCD_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADCD_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADCD_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADCD_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADCD_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADCD_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADCD_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADCD_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADCD_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADCD_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADCD_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADCD_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADCD_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADCD_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADCD_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADCD_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADCD_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADCD_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADCD_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADCD_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADCD_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADCD_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADCD_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADCD_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADCD_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADCD_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADCD_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADCD_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADCD_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADCD_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADCD_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADCD_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADCD_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADCD_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADCD_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADCE_CFG_REG */

#define CSL_MSS_IOMUX_PADCE_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADCE_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADCE_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADCE_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADCE_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADCE_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADCE_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADCE_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADCE_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADCE_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADCE_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADCE_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADCE_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADCE_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADCE_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADCE_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADCE_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADCE_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADCE_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADCE_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADCE_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADCE_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADCE_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADCE_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADCE_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADCE_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADCE_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADCE_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADCE_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADCE_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADCE_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADCE_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADCE_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADCE_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADCE_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADCE_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADCE_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADCF_CFG_REG */

#define CSL_MSS_IOMUX_PADCF_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADCF_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADCF_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADCF_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADCF_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADCF_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADCF_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADCF_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADCF_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADCF_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADCF_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADCF_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADCF_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADCF_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADCF_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADCF_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADCF_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADCF_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADCF_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADCF_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADCF_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADCF_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADCF_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADCF_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADCF_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADCF_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADCF_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADCF_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADCF_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADCF_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADCF_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADCF_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADCF_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADCF_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADCF_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADCF_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADCF_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADCG_CFG_REG */

#define CSL_MSS_IOMUX_PADCG_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADCG_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADCG_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADCG_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADCG_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADCG_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADCG_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADCG_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADCG_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADCG_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADCG_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADCG_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADCG_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADCG_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADCG_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADCG_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADCG_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADCG_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADCG_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADCG_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADCG_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADCG_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADCG_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADCG_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADCG_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADCG_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADCG_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADCG_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADCG_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADCG_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADCG_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADCG_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADCG_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADCG_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADCG_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADCG_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADCG_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADCH_CFG_REG */

#define CSL_MSS_IOMUX_PADCH_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADCH_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADCH_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADCH_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADCH_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADCH_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADCH_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADCH_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADCH_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADCH_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADCH_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADCH_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADCH_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADCH_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADCH_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADCH_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADCH_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADCH_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADCH_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADCH_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADCH_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADCH_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADCH_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADCH_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADCH_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADCH_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADCH_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADCH_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADCH_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADCH_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADCH_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADCH_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADCH_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADCH_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADCH_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADCH_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADCH_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADCI_CFG_REG */

#define CSL_MSS_IOMUX_PADCI_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADCI_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADCI_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADCI_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADCI_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADCI_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADCI_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADCI_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADCI_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADCI_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADCI_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADCI_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADCI_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADCI_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADCI_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADCI_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADCI_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADCI_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADCI_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADCI_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADCI_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADCI_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADCI_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADCI_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADCI_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADCI_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADCI_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADCI_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADCI_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADCI_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADCI_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADCI_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADCI_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADCI_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADCI_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADCI_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADCI_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADCJ_CFG_REG */

#define CSL_MSS_IOMUX_PADCJ_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADCJ_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADCJ_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADCJ_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADCJ_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADCJ_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADCJ_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADCJ_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADCJ_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADCJ_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADCJ_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADCJ_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADCJ_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADCJ_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADCJ_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADCJ_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADCJ_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADCJ_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADCJ_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADCJ_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADCJ_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADCJ_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADCJ_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADCJ_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADCJ_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADCJ_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADCJ_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADCJ_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADCJ_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADCJ_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADCJ_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADCJ_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADCJ_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADCJ_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADCJ_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADCJ_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADCJ_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADCK_CFG_REG */

#define CSL_MSS_IOMUX_PADCK_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADCK_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADCK_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADCK_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADCK_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADCK_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADCK_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADCK_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADCK_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADCK_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADCK_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADCK_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADCK_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADCK_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADCK_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADCK_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADCK_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADCK_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADCK_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADCK_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADCK_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADCK_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADCK_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADCK_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADCK_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADCK_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADCK_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADCK_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADCK_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADCK_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADCK_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADCK_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADCK_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADCK_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADCK_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADCK_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADCK_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADCL_CFG_REG */

#define CSL_MSS_IOMUX_PADCL_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADCL_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADCL_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADCL_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADCL_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADCL_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADCL_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADCL_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADCL_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADCL_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADCL_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADCL_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADCL_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADCL_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADCL_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADCL_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADCL_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADCL_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADCL_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADCL_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADCL_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADCL_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADCL_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADCL_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADCL_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADCL_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADCL_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADCL_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADCL_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADCL_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADCL_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADCL_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADCL_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADCL_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADCL_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADCL_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADCL_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADCM_CFG_REG */

#define CSL_MSS_IOMUX_PADCM_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADCM_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADCM_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADCM_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADCM_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADCM_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADCM_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADCM_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADCM_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADCM_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADCM_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADCM_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADCM_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADCM_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADCM_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADCM_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADCM_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADCM_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADCM_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADCM_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADCM_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADCM_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADCM_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADCM_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADCM_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADCM_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADCM_CFG_REG_PUPDSEL_RESETVAL                           (0x00000001U)
#define CSL_MSS_IOMUX_PADCM_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADCM_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADCM_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADCM_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADCM_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADCM_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADCM_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADCM_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADCM_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADCM_CFG_REG_RESETVAL                                   (0x000002C1U)

/* PADCN_CFG_REG */

#define CSL_MSS_IOMUX_PADCN_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADCN_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADCN_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADCN_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADCN_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADCN_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADCN_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADCN_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADCN_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADCN_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADCN_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADCN_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADCN_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADCN_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADCN_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADCN_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADCN_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADCN_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADCN_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADCN_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADCN_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADCN_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADCN_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADCN_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADCN_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADCN_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADCN_CFG_REG_PUPDSEL_RESETVAL                           (0x00000001U)
#define CSL_MSS_IOMUX_PADCN_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADCN_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADCN_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADCN_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADCN_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADCN_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADCN_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADCN_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADCN_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADCN_CFG_REG_RESETVAL                                   (0x000002C1U)

/* PADCO_CFG_REG */

#define CSL_MSS_IOMUX_PADCO_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADCO_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADCO_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADCO_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADCO_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADCO_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADCO_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADCO_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADCO_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADCO_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADCO_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADCO_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADCO_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADCO_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADCO_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADCO_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADCO_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADCO_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADCO_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADCO_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADCO_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADCO_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADCO_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADCO_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADCO_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADCO_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADCO_CFG_REG_PUPDSEL_RESETVAL                           (0x00000001U)
#define CSL_MSS_IOMUX_PADCO_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADCO_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADCO_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADCO_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADCO_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADCO_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADCO_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADCO_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADCO_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADCO_CFG_REG_RESETVAL                                   (0x000002C1U)

/* PADCP_CFG_REG */

#define CSL_MSS_IOMUX_PADCP_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADCP_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADCP_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADCP_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADCP_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADCP_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADCP_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADCP_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADCP_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADCP_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADCP_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADCP_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADCP_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADCP_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADCP_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADCP_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADCP_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADCP_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADCP_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADCP_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADCP_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADCP_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADCP_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADCP_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADCP_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADCP_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADCP_CFG_REG_PUPDSEL_RESETVAL                           (0x00000001U)
#define CSL_MSS_IOMUX_PADCP_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADCP_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADCP_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADCP_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADCP_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADCP_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADCP_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADCP_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADCP_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADCP_CFG_REG_RESETVAL                                   (0x000002C1U)

/* PADCQ_CFG_REG */

#define CSL_MSS_IOMUX_PADCQ_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADCQ_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADCQ_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADCQ_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADCQ_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADCQ_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADCQ_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADCQ_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADCQ_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADCQ_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADCQ_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADCQ_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADCQ_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADCQ_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADCQ_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADCQ_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADCQ_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADCQ_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADCQ_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADCQ_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADCQ_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADCQ_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADCQ_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADCQ_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADCQ_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADCQ_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADCQ_CFG_REG_PUPDSEL_RESETVAL                           (0x00000001U)
#define CSL_MSS_IOMUX_PADCQ_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADCQ_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADCQ_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADCQ_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADCQ_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADCQ_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADCQ_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADCQ_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADCQ_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADCQ_CFG_REG_RESETVAL                                   (0x000002C1U)

/* PADCR_CFG_REG */

#define CSL_MSS_IOMUX_PADCR_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADCR_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADCR_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADCR_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADCR_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADCR_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADCR_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADCR_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADCR_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADCR_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADCR_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADCR_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADCR_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADCR_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADCR_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADCR_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADCR_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADCR_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADCR_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADCR_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADCR_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADCR_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADCR_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADCR_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADCR_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADCR_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADCR_CFG_REG_PUPDSEL_RESETVAL                           (0x00000001U)
#define CSL_MSS_IOMUX_PADCR_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADCR_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADCR_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADCR_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADCR_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADCR_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADCR_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADCR_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADCR_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADCR_CFG_REG_RESETVAL                                   (0x000002C1U)

/* PADCS_CFG_REG */

#define CSL_MSS_IOMUX_PADCS_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADCS_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADCS_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADCS_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADCS_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADCS_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADCS_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADCS_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADCS_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADCS_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADCS_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADCS_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADCS_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADCS_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADCS_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADCS_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADCS_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADCS_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADCS_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADCS_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADCS_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADCS_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADCS_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADCS_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADCS_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADCS_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADCS_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADCS_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADCS_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADCS_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADCS_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADCS_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADCS_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADCS_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADCS_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADCS_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADCS_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADCT_CFG_REG */

#define CSL_MSS_IOMUX_PADCT_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADCT_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADCT_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADCT_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADCT_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADCT_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADCT_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADCT_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADCT_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADCT_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADCT_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADCT_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADCT_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADCT_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADCT_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADCT_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADCT_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADCT_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADCT_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADCT_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADCT_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADCT_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADCT_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADCT_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADCT_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADCT_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADCT_CFG_REG_PUPDSEL_RESETVAL                           (0x00000001U)
#define CSL_MSS_IOMUX_PADCT_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADCT_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADCT_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADCT_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADCT_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADCT_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADCT_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADCT_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADCT_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADCT_CFG_REG_RESETVAL                                   (0x000002C1U)

/* PADCU_CFG_REG */

#define CSL_MSS_IOMUX_PADCU_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADCU_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADCU_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADCU_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADCU_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADCU_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADCU_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADCU_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADCU_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADCU_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADCU_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADCU_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADCU_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADCU_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADCU_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADCU_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADCU_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADCU_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADCU_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADCU_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADCU_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADCU_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADCU_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADCU_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADCU_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADCU_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADCU_CFG_REG_PUPDSEL_RESETVAL                           (0x00000001U)
#define CSL_MSS_IOMUX_PADCU_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADCU_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADCU_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADCU_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADCU_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADCU_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADCU_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADCU_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADCU_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADCU_CFG_REG_RESETVAL                                   (0x000002C1U)

/* PADCV_CFG_REG */

#define CSL_MSS_IOMUX_PADCV_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADCV_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADCV_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADCV_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADCV_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADCV_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADCV_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADCV_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADCV_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADCV_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADCV_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADCV_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADCV_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADCV_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADCV_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADCV_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADCV_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADCV_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADCV_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADCV_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADCV_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADCV_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADCV_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADCV_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADCV_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADCV_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADCV_CFG_REG_PUPDSEL_RESETVAL                           (0x00000001U)
#define CSL_MSS_IOMUX_PADCV_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADCV_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADCV_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADCV_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADCV_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADCV_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADCV_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADCV_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADCV_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADCV_CFG_REG_RESETVAL                                   (0x000002C1U)

/* PADCW_CFG_REG */

#define CSL_MSS_IOMUX_PADCW_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADCW_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADCW_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADCW_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADCW_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADCW_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADCW_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADCW_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADCW_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADCW_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADCW_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADCW_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADCW_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADCW_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADCW_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADCW_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADCW_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADCW_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADCW_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADCW_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADCW_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADCW_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADCW_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADCW_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADCW_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADCW_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADCW_CFG_REG_PUPDSEL_RESETVAL                           (0x00000001U)
#define CSL_MSS_IOMUX_PADCW_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADCW_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADCW_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADCW_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADCW_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADCW_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADCW_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADCW_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADCW_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADCW_CFG_REG_RESETVAL                                   (0x000002C1U)

/* PADCX_CFG_REG */

#define CSL_MSS_IOMUX_PADCX_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADCX_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADCX_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADCX_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADCX_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADCX_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADCX_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADCX_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADCX_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADCX_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADCX_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADCX_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADCX_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADCX_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADCX_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADCX_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADCX_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADCX_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADCX_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADCX_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADCX_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADCX_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADCX_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADCX_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADCX_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADCX_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADCX_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADCX_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADCX_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADCX_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADCX_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADCX_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADCX_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADCX_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADCX_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADCX_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADCX_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADCY_CFG_REG */

#define CSL_MSS_IOMUX_PADCY_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADCY_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADCY_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADCY_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADCY_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADCY_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADCY_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADCY_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADCY_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADCY_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADCY_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADCY_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADCY_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADCY_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADCY_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADCY_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADCY_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADCY_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADCY_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADCY_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADCY_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADCY_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADCY_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADCY_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADCY_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADCY_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADCY_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADCY_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADCY_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADCY_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADCY_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADCY_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADCY_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADCY_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADCY_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADCY_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADCY_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADCZ_CFG_REG */

#define CSL_MSS_IOMUX_PADCZ_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADCZ_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADCZ_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADCZ_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADCZ_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADCZ_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADCZ_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADCZ_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADCZ_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADCZ_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADCZ_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADCZ_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADCZ_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADCZ_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADCZ_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADCZ_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADCZ_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADCZ_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADCZ_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADCZ_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADCZ_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADCZ_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADCZ_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADCZ_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADCZ_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADCZ_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADCZ_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADCZ_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADCZ_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADCZ_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADCZ_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADCZ_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADCZ_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADCZ_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADCZ_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADCZ_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADCZ_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADDA_CFG_REG */

#define CSL_MSS_IOMUX_PADDA_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADDA_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADDA_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADDA_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADDA_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADDA_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADDA_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADDA_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADDA_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADDA_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADDA_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADDA_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADDA_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADDA_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADDA_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADDA_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADDA_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADDA_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADDA_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADDA_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADDA_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADDA_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADDA_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADDA_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADDA_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADDA_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADDA_CFG_REG_PUPDSEL_RESETVAL                           (0x00000001U)
#define CSL_MSS_IOMUX_PADDA_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADDA_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADDA_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADDA_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADDA_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADDA_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADDA_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADDA_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADDA_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADDA_CFG_REG_RESETVAL                                   (0x000002C1U)

/* PADDB_CFG_REG */

#define CSL_MSS_IOMUX_PADDB_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADDB_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADDB_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADDB_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADDB_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADDB_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADDB_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADDB_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADDB_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADDB_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADDB_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADDB_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADDB_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADDB_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADDB_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADDB_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADDB_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADDB_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADDB_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADDB_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADDB_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADDB_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADDB_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADDB_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADDB_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADDB_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADDB_CFG_REG_PUPDSEL_RESETVAL                           (0x00000001U)
#define CSL_MSS_IOMUX_PADDB_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADDB_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADDB_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADDB_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADDB_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADDB_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADDB_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADDB_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADDB_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADDB_CFG_REG_RESETVAL                                   (0x000002C1U)

/* PADDC_CFG_REG */

#define CSL_MSS_IOMUX_PADDC_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADDC_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADDC_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADDC_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADDC_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADDC_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADDC_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADDC_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADDC_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADDC_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADDC_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADDC_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADDC_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADDC_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADDC_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADDC_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADDC_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADDC_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADDC_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADDC_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADDC_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADDC_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADDC_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADDC_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADDC_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADDC_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADDC_CFG_REG_PUPDSEL_RESETVAL                           (0x00000001U)
#define CSL_MSS_IOMUX_PADDC_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADDC_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADDC_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADDC_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADDC_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADDC_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADDC_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADDC_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADDC_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADDC_CFG_REG_RESETVAL                                   (0x000002C1U)

/* PADDD_CFG_REG */

#define CSL_MSS_IOMUX_PADDD_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADDD_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADDD_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADDD_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADDD_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADDD_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADDD_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADDD_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADDD_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADDD_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADDD_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADDD_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADDD_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADDD_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADDD_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADDD_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADDD_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADDD_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADDD_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADDD_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADDD_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADDD_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADDD_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADDD_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADDD_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADDD_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADDD_CFG_REG_PUPDSEL_RESETVAL                           (0x00000001U)
#define CSL_MSS_IOMUX_PADDD_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADDD_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADDD_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADDD_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADDD_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADDD_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADDD_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADDD_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADDD_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADDD_CFG_REG_RESETVAL                                   (0x000002C1U)

/* PADDE_CFG_REG */

#define CSL_MSS_IOMUX_PADDE_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADDE_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADDE_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADDE_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADDE_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADDE_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADDE_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADDE_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADDE_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADDE_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADDE_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADDE_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADDE_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADDE_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADDE_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADDE_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADDE_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADDE_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADDE_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADDE_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADDE_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADDE_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADDE_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADDE_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADDE_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADDE_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADDE_CFG_REG_PUPDSEL_RESETVAL                           (0x00000001U)
#define CSL_MSS_IOMUX_PADDE_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADDE_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADDE_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADDE_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADDE_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADDE_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADDE_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADDE_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADDE_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADDE_CFG_REG_RESETVAL                                   (0x000002C1U)

/* PADDF_CFG_REG */

#define CSL_MSS_IOMUX_PADDF_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADDF_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADDF_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADDF_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADDF_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADDF_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADDF_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADDF_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADDF_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADDF_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADDF_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADDF_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADDF_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADDF_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADDF_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADDF_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADDF_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADDF_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADDF_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADDF_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADDF_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADDF_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADDF_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADDF_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADDF_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADDF_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADDF_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADDF_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADDF_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADDF_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADDF_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADDF_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADDF_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADDF_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADDF_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADDF_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADDF_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADDG_CFG_REG */

#define CSL_MSS_IOMUX_PADDG_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADDG_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADDG_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADDG_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADDG_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADDG_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADDG_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADDG_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADDG_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADDG_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADDG_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADDG_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADDG_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADDG_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADDG_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADDG_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADDG_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADDG_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADDG_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADDG_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADDG_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADDG_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADDG_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADDG_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADDG_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADDG_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADDG_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADDG_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADDG_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADDG_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADDG_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADDG_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADDG_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADDG_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADDG_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADDG_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADDG_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADDH_CFG_REG */

#define CSL_MSS_IOMUX_PADDH_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADDH_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADDH_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADDH_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADDH_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADDH_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADDH_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADDH_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADDH_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADDH_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADDH_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADDH_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADDH_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADDH_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADDH_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADDH_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADDH_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADDH_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADDH_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADDH_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADDH_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADDH_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADDH_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADDH_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADDH_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADDH_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADDH_CFG_REG_PUPDSEL_RESETVAL                           (0x00000001U)
#define CSL_MSS_IOMUX_PADDH_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADDH_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADDH_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADDH_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADDH_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADDH_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADDH_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADDH_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADDH_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADDH_CFG_REG_RESETVAL                                   (0x000002C1U)

/* PADDI_CFG_REG */

#define CSL_MSS_IOMUX_PADDI_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADDI_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADDI_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADDI_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADDI_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADDI_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADDI_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADDI_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADDI_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADDI_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADDI_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADDI_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADDI_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADDI_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADDI_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADDI_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADDI_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADDI_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADDI_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADDI_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADDI_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADDI_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADDI_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADDI_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADDI_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADDI_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADDI_CFG_REG_PUPDSEL_RESETVAL                           (0x00000001U)
#define CSL_MSS_IOMUX_PADDI_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADDI_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADDI_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADDI_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADDI_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADDI_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADDI_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADDI_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADDI_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADDI_CFG_REG_RESETVAL                                   (0x000002C1U)

/* PADDJ_CFG_REG */

#define CSL_MSS_IOMUX_PADDJ_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADDJ_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADDJ_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADDJ_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADDJ_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADDJ_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADDJ_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADDJ_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADDJ_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADDJ_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADDJ_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADDJ_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADDJ_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADDJ_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADDJ_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADDJ_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADDJ_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADDJ_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADDJ_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADDJ_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADDJ_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADDJ_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADDJ_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADDJ_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADDJ_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADDJ_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADDJ_CFG_REG_PUPDSEL_RESETVAL                           (0x00000001U)
#define CSL_MSS_IOMUX_PADDJ_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADDJ_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADDJ_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADDJ_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADDJ_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADDJ_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADDJ_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADDJ_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADDJ_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADDJ_CFG_REG_RESETVAL                                   (0x000002C1U)

/* PADDK_CFG_REG */

#define CSL_MSS_IOMUX_PADDK_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADDK_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADDK_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADDK_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADDK_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADDK_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADDK_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADDK_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADDK_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADDK_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADDK_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADDK_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADDK_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADDK_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADDK_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADDK_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADDK_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADDK_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADDK_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADDK_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADDK_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADDK_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADDK_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADDK_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADDK_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADDK_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADDK_CFG_REG_PUPDSEL_RESETVAL                           (0x00000001U)
#define CSL_MSS_IOMUX_PADDK_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADDK_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADDK_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADDK_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADDK_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADDK_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADDK_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADDK_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADDK_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADDK_CFG_REG_RESETVAL                                   (0x000002C1U)

/* PADDL_CFG_REG */

#define CSL_MSS_IOMUX_PADDL_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADDL_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADDL_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADDL_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADDL_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADDL_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADDL_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADDL_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADDL_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADDL_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADDL_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADDL_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADDL_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADDL_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADDL_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADDL_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADDL_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADDL_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADDL_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADDL_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADDL_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADDL_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADDL_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADDL_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADDL_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADDL_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADDL_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADDL_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADDL_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADDL_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADDL_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADDL_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADDL_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADDL_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADDL_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADDL_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADDL_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADDM_CFG_REG */

#define CSL_MSS_IOMUX_PADDM_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADDM_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADDM_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADDM_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADDM_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADDM_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADDM_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADDM_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADDM_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADDM_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADDM_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADDM_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADDM_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADDM_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADDM_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADDM_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADDM_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADDM_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADDM_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADDM_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADDM_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADDM_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADDM_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADDM_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADDM_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADDM_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADDM_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADDM_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADDM_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADDM_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADDM_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADDM_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADDM_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADDM_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADDM_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADDM_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADDM_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADDN_CFG_REG */

#define CSL_MSS_IOMUX_PADDN_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADDN_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADDN_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADDN_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADDN_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADDN_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADDN_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADDN_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADDN_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADDN_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADDN_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADDN_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADDN_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADDN_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADDN_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADDN_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADDN_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADDN_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADDN_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADDN_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADDN_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADDN_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADDN_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADDN_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADDN_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADDN_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADDN_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADDN_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADDN_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADDN_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADDN_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADDN_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADDN_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADDN_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADDN_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADDN_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADDN_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADDO_CFG_REG */

#define CSL_MSS_IOMUX_PADDO_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADDO_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADDO_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADDO_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADDO_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADDO_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADDO_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADDO_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADDO_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADDO_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADDO_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADDO_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADDO_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADDO_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADDO_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADDO_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADDO_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADDO_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADDO_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADDO_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADDO_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADDO_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADDO_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADDO_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADDO_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADDO_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADDO_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADDO_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADDO_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADDO_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADDO_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADDO_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADDO_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADDO_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADDO_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADDO_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADDO_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADDP_CFG_REG */

#define CSL_MSS_IOMUX_PADDP_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADDP_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADDP_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADDP_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADDP_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADDP_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADDP_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADDP_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADDP_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADDP_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADDP_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADDP_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADDP_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADDP_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADDP_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADDP_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADDP_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADDP_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADDP_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADDP_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADDP_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADDP_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADDP_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADDP_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADDP_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADDP_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADDP_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADDP_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADDP_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADDP_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADDP_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADDP_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADDP_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADDP_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADDP_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADDP_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADDP_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADDQ_CFG_REG */

#define CSL_MSS_IOMUX_PADDQ_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADDQ_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADDQ_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADDQ_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADDQ_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADDQ_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADDQ_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADDQ_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADDQ_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADDQ_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADDQ_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADDQ_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADDQ_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADDQ_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADDQ_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADDQ_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADDQ_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADDQ_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADDQ_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADDQ_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADDQ_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADDQ_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADDQ_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADDQ_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADDQ_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADDQ_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADDQ_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADDQ_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADDQ_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADDQ_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADDQ_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADDQ_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADDQ_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADDQ_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADDQ_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADDQ_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADDQ_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADDR_CFG_REG */

#define CSL_MSS_IOMUX_PADDR_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADDR_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADDR_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADDR_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADDR_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADDR_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADDR_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADDR_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADDR_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADDR_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADDR_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADDR_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADDR_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADDR_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADDR_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADDR_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADDR_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADDR_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADDR_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADDR_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADDR_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADDR_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADDR_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADDR_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADDR_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADDR_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADDR_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADDR_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADDR_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADDR_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADDR_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADDR_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADDR_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADDR_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADDR_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADDR_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADDR_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADDS_CFG_REG */

#define CSL_MSS_IOMUX_PADDS_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADDS_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADDS_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADDS_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADDS_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADDS_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADDS_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADDS_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADDS_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADDS_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADDS_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADDS_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADDS_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADDS_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADDS_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADDS_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADDS_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADDS_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADDS_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADDS_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADDS_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADDS_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADDS_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADDS_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADDS_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADDS_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADDS_CFG_REG_PUPDSEL_RESETVAL                           (0x00000001U)
#define CSL_MSS_IOMUX_PADDS_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADDS_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADDS_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADDS_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADDS_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADDS_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADDS_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADDS_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADDS_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADDS_CFG_REG_RESETVAL                                   (0x000002C1U)

/* PADDT_CFG_REG */

#define CSL_MSS_IOMUX_PADDT_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADDT_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADDT_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADDT_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADDT_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADDT_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADDT_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADDT_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADDT_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADDT_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADDT_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADDT_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADDT_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADDT_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADDT_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADDT_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADDT_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADDT_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADDT_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADDT_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADDT_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADDT_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADDT_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADDT_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADDT_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADDT_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADDT_CFG_REG_PUPDSEL_RESETVAL                           (0x00000001U)
#define CSL_MSS_IOMUX_PADDT_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADDT_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADDT_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADDT_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADDT_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADDT_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADDT_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADDT_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADDT_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADDT_CFG_REG_RESETVAL                                   (0x000002C1U)

/* PADDU_CFG_REG */

#define CSL_MSS_IOMUX_PADDU_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADDU_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADDU_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADDU_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADDU_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADDU_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADDU_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADDU_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADDU_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADDU_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADDU_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADDU_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADDU_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADDU_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADDU_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADDU_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADDU_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADDU_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADDU_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADDU_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADDU_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADDU_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADDU_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADDU_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADDU_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADDU_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADDU_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADDU_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADDU_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADDU_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADDU_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADDU_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADDU_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADDU_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADDU_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADDU_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADDU_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADDV_CFG_REG */

#define CSL_MSS_IOMUX_PADDV_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADDV_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADDV_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADDV_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADDV_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADDV_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADDV_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADDV_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADDV_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADDV_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADDV_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADDV_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADDV_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADDV_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADDV_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADDV_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADDV_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADDV_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADDV_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADDV_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADDV_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADDV_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADDV_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADDV_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADDV_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADDV_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADDV_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADDV_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADDV_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADDV_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADDV_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADDV_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADDV_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADDV_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADDV_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADDV_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADDV_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADDW_CFG_REG */

#define CSL_MSS_IOMUX_PADDW_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADDW_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADDW_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADDW_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADDW_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADDW_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADDW_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADDW_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADDW_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADDW_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADDW_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADDW_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADDW_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADDW_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADDW_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADDW_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADDW_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADDW_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADDW_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADDW_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADDW_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADDW_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADDW_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADDW_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADDW_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADDW_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADDW_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADDW_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADDW_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADDW_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADDW_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADDW_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADDW_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADDW_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADDW_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADDW_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADDW_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADDX_CFG_REG */

#define CSL_MSS_IOMUX_PADDX_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADDX_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADDX_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADDX_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADDX_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADDX_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADDX_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADDX_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADDX_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADDX_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADDX_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADDX_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADDX_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADDX_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADDX_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADDX_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADDX_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADDX_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADDX_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADDX_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADDX_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADDX_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADDX_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADDX_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADDX_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADDX_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADDX_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADDX_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADDX_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADDX_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADDX_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADDX_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADDX_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADDX_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADDX_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADDX_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADDX_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADDY_CFG_REG */

#define CSL_MSS_IOMUX_PADDY_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADDY_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADDY_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADDY_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADDY_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADDY_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADDY_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADDY_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADDY_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADDY_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADDY_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADDY_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADDY_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADDY_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADDY_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADDY_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADDY_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADDY_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADDY_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADDY_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADDY_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADDY_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADDY_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADDY_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADDY_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADDY_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADDY_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADDY_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADDY_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADDY_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADDY_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADDY_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADDY_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADDY_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADDY_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADDY_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADDY_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADDZ_CFG_REG */

#define CSL_MSS_IOMUX_PADDZ_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADDZ_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADDZ_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADDZ_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADDZ_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADDZ_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADDZ_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADDZ_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADDZ_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADDZ_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADDZ_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADDZ_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADDZ_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADDZ_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADDZ_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADDZ_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADDZ_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADDZ_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADDZ_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADDZ_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADDZ_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADDZ_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADDZ_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADDZ_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADDZ_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADDZ_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADDZ_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADDZ_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADDZ_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADDZ_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADDZ_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADDZ_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADDZ_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADDZ_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADDZ_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADDZ_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADDZ_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADEA_CFG_REG */

#define CSL_MSS_IOMUX_PADEA_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADEA_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADEA_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADEA_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADEA_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADEA_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADEA_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADEA_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADEA_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADEA_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADEA_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADEA_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADEA_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADEA_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADEA_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADEA_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADEA_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADEA_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADEA_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADEA_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADEA_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADEA_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADEA_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADEA_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADEA_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADEA_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADEA_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADEA_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADEA_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADEA_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADEA_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADEA_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADEA_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADEA_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADEA_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADEA_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADEA_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADEB_CFG_REG */

#define CSL_MSS_IOMUX_PADEB_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADEB_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADEB_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADEB_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADEB_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADEB_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADEB_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADEB_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADEB_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADEB_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADEB_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADEB_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADEB_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADEB_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADEB_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADEB_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADEB_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADEB_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADEB_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADEB_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADEB_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADEB_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADEB_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADEB_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADEB_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADEB_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADEB_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADEB_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADEB_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADEB_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADEB_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADEB_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADEB_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADEB_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADEB_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADEB_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADEB_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADEC_CFG_REG */

#define CSL_MSS_IOMUX_PADEC_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADEC_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADEC_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADEC_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADEC_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADEC_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADEC_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADEC_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADEC_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADEC_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADEC_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADEC_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADEC_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADEC_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADEC_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADEC_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADEC_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADEC_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADEC_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADEC_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADEC_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADEC_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADEC_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADEC_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADEC_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADEC_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADEC_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADEC_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADEC_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADEC_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADEC_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADEC_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADEC_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADEC_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADEC_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADEC_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADEC_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADED_CFG_REG */

#define CSL_MSS_IOMUX_PADED_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADED_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADED_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADED_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADED_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADED_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADED_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADED_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADED_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADED_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADED_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADED_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADED_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADED_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADED_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADED_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADED_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADED_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADED_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADED_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADED_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADED_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADED_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADED_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADED_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADED_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADED_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADED_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADED_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADED_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADED_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADED_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADED_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADED_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADED_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADED_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADED_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADEE_CFG_REG */

#define CSL_MSS_IOMUX_PADEE_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADEE_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADEE_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADEE_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADEE_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADEE_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADEE_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADEE_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADEE_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADEE_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADEE_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADEE_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADEE_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADEE_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADEE_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADEE_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADEE_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADEE_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADEE_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADEE_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADEE_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADEE_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADEE_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADEE_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADEE_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADEE_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADEE_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADEE_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADEE_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADEE_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADEE_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADEE_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADEE_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADEE_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADEE_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADEE_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADEE_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADEF_CFG_REG */

#define CSL_MSS_IOMUX_PADEF_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADEF_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADEF_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADEF_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADEF_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADEF_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADEF_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADEF_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADEF_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADEF_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADEF_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADEF_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADEF_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADEF_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADEF_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADEF_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADEF_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADEF_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADEF_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADEF_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADEF_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADEF_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADEF_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADEF_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADEF_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADEF_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADEF_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADEF_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADEF_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADEF_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADEF_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADEF_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADEF_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADEF_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADEF_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADEF_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADEF_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADEG_CFG_REG */

#define CSL_MSS_IOMUX_PADEG_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADEG_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADEG_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADEG_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADEG_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADEG_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADEG_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADEG_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADEG_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADEG_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADEG_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADEG_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADEG_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADEG_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADEG_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADEG_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADEG_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADEG_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADEG_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADEG_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADEG_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADEG_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADEG_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADEG_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADEG_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADEG_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADEG_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADEG_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADEG_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADEG_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADEG_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADEG_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADEG_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADEG_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADEG_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADEG_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADEG_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADEH_CFG_REG */

#define CSL_MSS_IOMUX_PADEH_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADEH_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADEH_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADEH_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADEH_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADEH_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADEH_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADEH_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADEH_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADEH_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADEH_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADEH_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADEH_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADEH_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADEH_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADEH_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADEH_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADEH_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADEH_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADEH_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADEH_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADEH_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADEH_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADEH_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADEH_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADEH_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADEH_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADEH_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADEH_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADEH_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADEH_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADEH_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADEH_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADEH_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADEH_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADEH_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADEH_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADEI_CFG_REG */

#define CSL_MSS_IOMUX_PADEI_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADEI_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADEI_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADEI_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADEI_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADEI_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADEI_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADEI_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADEI_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADEI_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADEI_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADEI_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADEI_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADEI_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADEI_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADEI_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADEI_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADEI_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADEI_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADEI_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADEI_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADEI_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADEI_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADEI_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADEI_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADEI_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADEI_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADEI_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADEI_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADEI_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADEI_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADEI_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADEI_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADEI_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADEI_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADEI_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADEI_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADEJ_CFG_REG */

#define CSL_MSS_IOMUX_PADEJ_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADEJ_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADEJ_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADEJ_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADEJ_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADEJ_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADEJ_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADEJ_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADEJ_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADEJ_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADEJ_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADEJ_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADEJ_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADEJ_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADEJ_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADEJ_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADEJ_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADEJ_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADEJ_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADEJ_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADEJ_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADEJ_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADEJ_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADEJ_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADEJ_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADEJ_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADEJ_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADEJ_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADEJ_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADEJ_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADEJ_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADEJ_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADEJ_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADEJ_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADEJ_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADEJ_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADEJ_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADEK_CFG_REG */

#define CSL_MSS_IOMUX_PADEK_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADEK_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADEK_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADEK_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADEK_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADEK_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADEK_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADEK_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADEK_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADEK_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADEK_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADEK_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADEK_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADEK_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADEK_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADEK_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADEK_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADEK_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADEK_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADEK_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADEK_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADEK_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADEK_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADEK_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADEK_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADEK_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADEK_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADEK_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADEK_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADEK_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADEK_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADEK_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADEK_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADEK_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADEK_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADEK_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADEK_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADEL_CFG_REG */

#define CSL_MSS_IOMUX_PADEL_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADEL_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADEL_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADEL_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADEL_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADEL_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADEL_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADEL_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADEL_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADEL_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADEL_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADEL_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADEL_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADEL_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADEL_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADEL_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADEL_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADEL_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADEL_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADEL_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADEL_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADEL_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADEL_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADEL_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADEL_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADEL_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADEL_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADEL_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADEL_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADEL_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADEL_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADEL_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADEL_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADEL_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADEL_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADEL_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADEL_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADEM_CFG_REG */

#define CSL_MSS_IOMUX_PADEM_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADEM_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADEM_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADEM_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADEM_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADEM_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADEM_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADEM_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADEM_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADEM_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADEM_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADEM_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADEM_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADEM_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADEM_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADEM_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADEM_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADEM_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADEM_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADEM_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADEM_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADEM_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADEM_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADEM_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADEM_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADEM_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADEM_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADEM_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADEM_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADEM_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADEM_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADEM_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADEM_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADEM_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADEM_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADEM_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADEM_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADEN_CFG_REG */

#define CSL_MSS_IOMUX_PADEN_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADEN_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADEN_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADEN_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADEN_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADEN_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADEN_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADEN_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADEN_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADEN_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADEN_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADEN_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADEN_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADEN_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADEN_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADEN_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADEN_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADEN_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADEN_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADEN_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADEN_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADEN_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADEN_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADEN_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADEN_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADEN_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADEN_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADEN_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADEN_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADEN_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADEN_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADEN_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADEN_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADEN_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADEN_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADEN_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADEN_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADEO_CFG_REG */

#define CSL_MSS_IOMUX_PADEO_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADEO_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADEO_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADEO_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADEO_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADEO_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADEO_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADEO_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADEO_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADEO_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADEO_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADEO_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADEO_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADEO_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADEO_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADEO_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADEO_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADEO_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADEO_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADEO_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADEO_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADEO_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADEO_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADEO_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADEO_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADEO_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADEO_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADEO_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADEO_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADEO_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADEO_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADEO_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADEO_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADEO_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADEO_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADEO_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADEO_CFG_REG_RESETVAL                                   (0x000000C1U)

/* PADEP_CFG_REG */

#define CSL_MSS_IOMUX_PADEP_CFG_REG_FUNC_SEL_MASK                              (0x0000000FU)
#define CSL_MSS_IOMUX_PADEP_CFG_REG_FUNC_SEL_SHIFT                             (0x00000000U)
#define CSL_MSS_IOMUX_PADEP_CFG_REG_FUNC_SEL_RESETVAL                          (0x00000001U)
#define CSL_MSS_IOMUX_PADEP_CFG_REG_FUNC_SEL_MAX                               (0x0000000FU)

#define CSL_MSS_IOMUX_PADEP_CFG_REG_IE_OVERRIDE_CTRL_MASK                      (0x00000010U)
#define CSL_MSS_IOMUX_PADEP_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                     (0x00000004U)
#define CSL_MSS_IOMUX_PADEP_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                  (0x00000000U)
#define CSL_MSS_IOMUX_PADEP_CFG_REG_IE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADEP_CFG_REG_IE_OVERRIDE_MASK                           (0x00000020U)
#define CSL_MSS_IOMUX_PADEP_CFG_REG_IE_OVERRIDE_SHIFT                          (0x00000005U)
#define CSL_MSS_IOMUX_PADEP_CFG_REG_IE_OVERRIDE_RESETVAL                       (0x00000000U)
#define CSL_MSS_IOMUX_PADEP_CFG_REG_IE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADEP_CFG_REG_OE_OVERRIDE_CTRL_MASK                      (0x00000040U)
#define CSL_MSS_IOMUX_PADEP_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                     (0x00000006U)
#define CSL_MSS_IOMUX_PADEP_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                  (0x00000001U)
#define CSL_MSS_IOMUX_PADEP_CFG_REG_OE_OVERRIDE_CTRL_MAX                       (0x00000001U)

#define CSL_MSS_IOMUX_PADEP_CFG_REG_OE_OVERRIDE_MASK                           (0x00000080U)
#define CSL_MSS_IOMUX_PADEP_CFG_REG_OE_OVERRIDE_SHIFT                          (0x00000007U)
#define CSL_MSS_IOMUX_PADEP_CFG_REG_OE_OVERRIDE_RESETVAL                       (0x00000001U)
#define CSL_MSS_IOMUX_PADEP_CFG_REG_OE_OVERRIDE_MAX                            (0x00000001U)

#define CSL_MSS_IOMUX_PADEP_CFG_REG_PI_MASK                                    (0x00000100U)
#define CSL_MSS_IOMUX_PADEP_CFG_REG_PI_SHIFT                                   (0x00000008U)
#define CSL_MSS_IOMUX_PADEP_CFG_REG_PI_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADEP_CFG_REG_PI_MAX                                     (0x00000001U)

#define CSL_MSS_IOMUX_PADEP_CFG_REG_PUPDSEL_MASK                               (0x00000200U)
#define CSL_MSS_IOMUX_PADEP_CFG_REG_PUPDSEL_SHIFT                              (0x00000009U)
#define CSL_MSS_IOMUX_PADEP_CFG_REG_PUPDSEL_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_PADEP_CFG_REG_PUPDSEL_MAX                                (0x00000001U)

#define CSL_MSS_IOMUX_PADEP_CFG_REG_SC1_MASK                                   (0x00000400U)
#define CSL_MSS_IOMUX_PADEP_CFG_REG_SC1_SHIFT                                  (0x0000000AU)
#define CSL_MSS_IOMUX_PADEP_CFG_REG_SC1_RESETVAL                               (0x00000000U)
#define CSL_MSS_IOMUX_PADEP_CFG_REG_SC1_MAX                                    (0x00000001U)

#define CSL_MSS_IOMUX_PADEP_CFG_REG_NU_MASK                                    (0xFFFFF800U)
#define CSL_MSS_IOMUX_PADEP_CFG_REG_NU_SHIFT                                   (0x0000000BU)
#define CSL_MSS_IOMUX_PADEP_CFG_REG_NU_RESETVAL                                (0x00000000U)
#define CSL_MSS_IOMUX_PADEP_CFG_REG_NU_MAX                                     (0x001FFFFFU)

#define CSL_MSS_IOMUX_PADEP_CFG_REG_RESETVAL                                   (0x000000C1U)

/* USERMODEEN */

#define CSL_MSS_IOMUX_USERMODEEN_USERMODEEN_MASK                               (0xFFFFFFFFU)
#define CSL_MSS_IOMUX_USERMODEEN_USERMODEEN_SHIFT                              (0x00000000U)
#define CSL_MSS_IOMUX_USERMODEEN_USERMODEEN_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_USERMODEEN_USERMODEEN_MAX                                (0xFFFFFFFFU)

#define CSL_MSS_IOMUX_USERMODEEN_RESETVAL                                      (0x00000000U)

/* PADGLBLCFGREG */

#define CSL_MSS_IOMUX_PADGLBLCFGREG_PADGLBLCFGREG_MASK                         (0xFFFFFFFFU)
#define CSL_MSS_IOMUX_PADGLBLCFGREG_PADGLBLCFGREG_SHIFT                        (0x00000000U)
#define CSL_MSS_IOMUX_PADGLBLCFGREG_PADGLBLCFGREG_RESETVAL                     (0x00000000U)
#define CSL_MSS_IOMUX_PADGLBLCFGREG_PADGLBLCFGREG_MAX                          (0xFFFFFFFFU)

#define CSL_MSS_IOMUX_PADGLBLCFGREG_RESETVAL                                   (0x00000000U)

/* IOCFGKICK0 */

#define CSL_MSS_IOMUX_IOCFGKICK0_IOCFGKICK0_MASK                               (0xFFFFFFFFU)
#define CSL_MSS_IOMUX_IOCFGKICK0_IOCFGKICK0_SHIFT                              (0x00000000U)
#define CSL_MSS_IOMUX_IOCFGKICK0_IOCFGKICK0_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_IOCFGKICK0_IOCFGKICK0_MAX                                (0xFFFFFFFFU)

#define CSL_MSS_IOMUX_IOCFGKICK0_RESETVAL                                      (0x00000000U)

/* IOCFGKICK1 */

#define CSL_MSS_IOMUX_IOCFGKICK1_IOCFGKICK1_MASK                               (0xFFFFFFFFU)
#define CSL_MSS_IOMUX_IOCFGKICK1_IOCFGKICK1_SHIFT                              (0x00000000U)
#define CSL_MSS_IOMUX_IOCFGKICK1_IOCFGKICK1_RESETVAL                           (0x00000000U)
#define CSL_MSS_IOMUX_IOCFGKICK1_IOCFGKICK1_MAX                                (0xFFFFFFFFU)

#define CSL_MSS_IOMUX_IOCFGKICK1_RESETVAL                                      (0x00000000U)

#ifdef __cplusplus
}
#endif
#endif
