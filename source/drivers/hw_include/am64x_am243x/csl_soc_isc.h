/********************************************************************
*
* SOC ISC PROPERTIES. header file
*
* Copyright (C) 2015-2019 Texas Instruments Incorporated.
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
#ifndef CSLR_SOC_ISC_H_
#define CSLR_SOC_ISC_H_

#include <drivers/hw_include/cslr.h>
#include <drivers/hw_include/tistdtypes.h>
#ifdef __cplusplus
extern "C"
{
#endif

/*
* Auto-generated CSL definitions for SoC ISC Instances:
*/

#define CSL_ISC_DEFAULT                                                                            (0U)
#define CSL_ISC_CC                                                                                 (1U)

/* Properties of firewall at master: DMSC0_CBASS_0 */
#define CSL_ISC_DMSC0_CBASS_0_IQUASAR_I_MST_TYPE                                                   (CSL_ISC_DEFAULT)
#define CSL_ISC_DMSC0_CBASS_0_IQUASAR_I_MST_ID                                                     (32U)
#define CSL_ISC_DMSC0_CBASS_0_IQUASAR_I_MST_DEFAULT_PRIV_ID                                        (202U)
#define CSL_ISC_DMSC0_CBASS_0_IQUASAR_I_MST_MMR_BASE                                               (0x45808000ul)
#define CSL_ISC_DMSC0_CBASS_0_IQUASAR_I_MST_REGION_COUNT                                           (1U)
#define CSL_ISC_DMSC0_CBASS_0_IQUASAR_D_MST_TYPE                                                   (CSL_ISC_DEFAULT)
#define CSL_ISC_DMSC0_CBASS_0_IQUASAR_D_MST_ID                                                     (33U)
#define CSL_ISC_DMSC0_CBASS_0_IQUASAR_D_MST_DEFAULT_PRIV_ID                                        (202U)
#define CSL_ISC_DMSC0_CBASS_0_IQUASAR_D_MST_MMR_BASE                                               (0x45808400ul)
#define CSL_ISC_DMSC0_CBASS_0_IQUASAR_D_MST_REGION_COUNT                                           (1U)
#define CSL_ISC_DMSC0_CBASS_0_IQUASAR_S_MST_TYPE                                                   (CSL_ISC_DEFAULT)
#define CSL_ISC_DMSC0_CBASS_0_IQUASAR_S_MST_ID                                                     (34U)
#define CSL_ISC_DMSC0_CBASS_0_IQUASAR_S_MST_DEFAULT_PRIV_ID                                        (202U)
#define CSL_ISC_DMSC0_CBASS_0_IQUASAR_S_MST_MMR_BASE                                               (0x45808800ul)
#define CSL_ISC_DMSC0_CBASS_0_IQUASAR_S_MST_REGION_COUNT                                           (1U)

/* Properties of firewall at master: R5FSS0 */
#define CSL_ISC_R5FSS0_CPU0_PMST_TYPE                                                              (CSL_ISC_DEFAULT)
#define CSL_ISC_R5FSS0_CPU0_PMST_ID                                                                (128U)
#define CSL_ISC_R5FSS0_CPU0_PMST_DEFAULT_PRIV_ID                                                   (212U)
#define CSL_ISC_R5FSS0_CPU0_PMST_MMR_BASE                                                          (0x45820000ul)
#define CSL_ISC_R5FSS0_CPU0_PMST_REGION_COUNT                                                      (1U)
#define CSL_ISC_R5FSS0_CPU1_PMST_TYPE                                                              (CSL_ISC_DEFAULT)
#define CSL_ISC_R5FSS0_CPU1_PMST_ID                                                                (129U)
#define CSL_ISC_R5FSS0_CPU1_PMST_DEFAULT_PRIV_ID                                                   (213U)
#define CSL_ISC_R5FSS0_CPU1_PMST_MMR_BASE                                                          (0x45820400ul)
#define CSL_ISC_R5FSS0_CPU1_PMST_REGION_COUNT                                                      (1U)
#define CSL_ISC_R5FSS0_CPU0_RMST_TYPE                                                              (CSL_ISC_DEFAULT)
#define CSL_ISC_R5FSS0_CPU0_RMST_ID                                                                (522U)
#define CSL_ISC_R5FSS0_CPU0_RMST_DEFAULT_PRIV_ID                                                   (212U)
#define CSL_ISC_R5FSS0_CPU0_RMST_MMR_BASE                                                          (0x45882800ul)
#define CSL_ISC_R5FSS0_CPU0_RMST_REGION_COUNT                                                      (1U)
#define CSL_ISC_R5FSS0_CPU0_WMST_TYPE                                                              (CSL_ISC_DEFAULT)
#define CSL_ISC_R5FSS0_CPU0_WMST_ID                                                                (523U)
#define CSL_ISC_R5FSS0_CPU0_WMST_DEFAULT_PRIV_ID                                                   (212U)
#define CSL_ISC_R5FSS0_CPU0_WMST_MMR_BASE                                                          (0x45882c00ul)
#define CSL_ISC_R5FSS0_CPU0_WMST_REGION_COUNT                                                      (1U)
#define CSL_ISC_R5FSS0_CPU1_RMST_TYPE                                                              (CSL_ISC_DEFAULT)
#define CSL_ISC_R5FSS0_CPU1_RMST_ID                                                                (527U)
#define CSL_ISC_R5FSS0_CPU1_RMST_DEFAULT_PRIV_ID                                                   (213U)
#define CSL_ISC_R5FSS0_CPU1_RMST_MMR_BASE                                                          (0x45883c00ul)
#define CSL_ISC_R5FSS0_CPU1_RMST_REGION_COUNT                                                      (1U)
#define CSL_ISC_R5FSS0_CPU1_WMST_TYPE                                                              (CSL_ISC_DEFAULT)
#define CSL_ISC_R5FSS0_CPU1_WMST_ID                                                                (528U)
#define CSL_ISC_R5FSS0_CPU1_WMST_DEFAULT_PRIV_ID                                                   (213U)
#define CSL_ISC_R5FSS0_CPU1_WMST_MMR_BASE                                                          (0x45884000ul)
#define CSL_ISC_R5FSS0_CPU1_WMST_REGION_COUNT                                                      (1U)

/* Properties of firewall at master: R5FSS1 */
#define CSL_ISC_R5FSS1_CPU0_PMST_TYPE                                                              (CSL_ISC_DEFAULT)
#define CSL_ISC_R5FSS1_CPU0_PMST_ID                                                                (130U)
#define CSL_ISC_R5FSS1_CPU0_PMST_DEFAULT_PRIV_ID                                                   (214U)
#define CSL_ISC_R5FSS1_CPU0_PMST_MMR_BASE                                                          (0x45820800ul)
#define CSL_ISC_R5FSS1_CPU0_PMST_REGION_COUNT                                                      (1U)
#define CSL_ISC_R5FSS1_CPU1_PMST_TYPE                                                              (CSL_ISC_DEFAULT)
#define CSL_ISC_R5FSS1_CPU1_PMST_ID                                                                (131U)
#define CSL_ISC_R5FSS1_CPU1_PMST_DEFAULT_PRIV_ID                                                   (215U)
#define CSL_ISC_R5FSS1_CPU1_PMST_MMR_BASE                                                          (0x45820c00ul)
#define CSL_ISC_R5FSS1_CPU1_PMST_REGION_COUNT                                                      (1U)
#define CSL_ISC_R5FSS1_CPU0_RMST_TYPE                                                              (CSL_ISC_DEFAULT)
#define CSL_ISC_R5FSS1_CPU0_RMST_ID                                                                (532U)
#define CSL_ISC_R5FSS1_CPU0_RMST_DEFAULT_PRIV_ID                                                   (214U)
#define CSL_ISC_R5FSS1_CPU0_RMST_MMR_BASE                                                          (0x45885000ul)
#define CSL_ISC_R5FSS1_CPU0_RMST_REGION_COUNT                                                      (1U)
#define CSL_ISC_R5FSS1_CPU0_WMST_TYPE                                                              (CSL_ISC_DEFAULT)
#define CSL_ISC_R5FSS1_CPU0_WMST_ID                                                                (533U)
#define CSL_ISC_R5FSS1_CPU0_WMST_DEFAULT_PRIV_ID                                                   (214U)
#define CSL_ISC_R5FSS1_CPU0_WMST_MMR_BASE                                                          (0x45885400ul)
#define CSL_ISC_R5FSS1_CPU0_WMST_REGION_COUNT                                                      (1U)
#define CSL_ISC_R5FSS1_CPU1_RMST_TYPE                                                              (CSL_ISC_DEFAULT)
#define CSL_ISC_R5FSS1_CPU1_RMST_ID                                                                (537U)
#define CSL_ISC_R5FSS1_CPU1_RMST_DEFAULT_PRIV_ID                                                   (215U)
#define CSL_ISC_R5FSS1_CPU1_RMST_MMR_BASE                                                          (0x45886400ul)
#define CSL_ISC_R5FSS1_CPU1_RMST_REGION_COUNT                                                      (1U)
#define CSL_ISC_R5FSS1_CPU1_WMST_TYPE                                                              (CSL_ISC_DEFAULT)
#define CSL_ISC_R5FSS1_CPU1_WMST_ID                                                                (538U)
#define CSL_ISC_R5FSS1_CPU1_WMST_DEFAULT_PRIV_ID                                                   (215U)
#define CSL_ISC_R5FSS1_CPU1_WMST_MMR_BASE                                                          (0x45886800ul)
#define CSL_ISC_R5FSS1_CPU1_WMST_REGION_COUNT                                                      (1U)

/* Properties of firewall at master: MCU_M4FSS0 */
#define CSL_ISC_MCU_M4FSS0_VBUSP_M_TYPE                                                            (CSL_ISC_DEFAULT)
#define CSL_ISC_MCU_M4FSS0_VBUSP_M_ID                                                              (192U)
#define CSL_ISC_MCU_M4FSS0_VBUSP_M_DEFAULT_PRIV_ID                                                 (100U)
#define CSL_ISC_MCU_M4FSS0_VBUSP_M_MMR_BASE                                                        (0x45830000ul)
#define CSL_ISC_MCU_M4FSS0_VBUSP_M_REGION_COUNT                                                    (1U)

/* Properties of firewall at master: DMASS0_SEC_PROXY_0 */
#define CSL_ISC_DMASS0_SEC_PROXY_0_DST_TYPE                                                        (CSL_ISC_DEFAULT)
#define CSL_ISC_DMASS0_SEC_PROXY_0_DST_ID                                                          (256U)
#define CSL_ISC_DMASS0_SEC_PROXY_0_DST_DEFAULT_PRIV_ID                                             (64U)
#define CSL_ISC_DMASS0_SEC_PROXY_0_DST_MMR_BASE                                                    (0x45840000ul)
#define CSL_ISC_DMASS0_SEC_PROXY_0_DST_REGION_COUNT                                                (1U)

/* Properties of firewall at master: DMASS0_RINGACC_0 */
#define CSL_ISC_DMASS0_RINGACC_0_DST_TYPE                                                          (CSL_ISC_DEFAULT)
#define CSL_ISC_DMASS0_RINGACC_0_DST_ID                                                            (257U)
#define CSL_ISC_DMASS0_RINGACC_0_DST_DEFAULT_PRIV_ID                                               (65U)
#define CSL_ISC_DMASS0_RINGACC_0_DST_MMR_BASE                                                      (0x45840400ul)
#define CSL_ISC_DMASS0_RINGACC_0_DST_REGION_COUNT                                                  (1U)

/* Properties of firewall at master: COMPUTE_CLUSTER0 */
#define CSL_ISC_COMPUTE_CLUSTER0_A53_DUAL_WRAP_CBA_AXI_R_TYPE                                      (CSL_ISC_DEFAULT)
#define CSL_ISC_COMPUTE_CLUSTER0_A53_DUAL_WRAP_CBA_AXI_R_ID                                        (513U)
#define CSL_ISC_COMPUTE_CLUSTER0_A53_DUAL_WRAP_CBA_AXI_R_DEFAULT_PRIV_ID                           (4U)
#define CSL_ISC_COMPUTE_CLUSTER0_A53_DUAL_WRAP_CBA_AXI_R_MMR_BASE                                  (0x45880400ul)
#define CSL_ISC_COMPUTE_CLUSTER0_A53_DUAL_WRAP_CBA_AXI_R_REGION_COUNT                              (1U)
#define CSL_ISC_COMPUTE_CLUSTER0_A53_DUAL_WRAP_CBA_AXI_W_TYPE                                      (CSL_ISC_DEFAULT)
#define CSL_ISC_COMPUTE_CLUSTER0_A53_DUAL_WRAP_CBA_AXI_W_ID                                        (514U)
#define CSL_ISC_COMPUTE_CLUSTER0_A53_DUAL_WRAP_CBA_AXI_W_DEFAULT_PRIV_ID                           (4U)
#define CSL_ISC_COMPUTE_CLUSTER0_A53_DUAL_WRAP_CBA_AXI_W_MMR_BASE                                  (0x45880800ul)
#define CSL_ISC_COMPUTE_CLUSTER0_A53_DUAL_WRAP_CBA_AXI_W_REGION_COUNT                              (1U)

/* Properties of firewall at master: MMCSD0 */
#define CSL_ISC_MMCSD0_EMMCSS_WR_TYPE                                                              (CSL_ISC_DEFAULT)
#define CSL_ISC_MMCSD0_EMMCSS_WR_ID                                                                (516U)
#define CSL_ISC_MMCSD0_EMMCSS_WR_DEFAULT_PRIV_ID                                                   (128U)
#define CSL_ISC_MMCSD0_EMMCSS_WR_MMR_BASE                                                          (0x45881000ul)
#define CSL_ISC_MMCSD0_EMMCSS_WR_REGION_COUNT                                                      (1U)
#define CSL_ISC_MMCSD0_EMMCSS_RD_TYPE                                                              (CSL_ISC_DEFAULT)
#define CSL_ISC_MMCSD0_EMMCSS_RD_ID                                                                (517U)
#define CSL_ISC_MMCSD0_EMMCSS_RD_DEFAULT_PRIV_ID                                                   (128U)
#define CSL_ISC_MMCSD0_EMMCSS_RD_MMR_BASE                                                          (0x45881400ul)
#define CSL_ISC_MMCSD0_EMMCSS_RD_REGION_COUNT                                                      (1U)

/* Properties of firewall at master: GICSS0 */
#define CSL_ISC_GICSS0_MEM_WR_VBUSM_TYPE                                                           (CSL_ISC_DEFAULT)
#define CSL_ISC_GICSS0_MEM_WR_VBUSM_ID                                                             (520U)
#define CSL_ISC_GICSS0_MEM_WR_VBUSM_DEFAULT_PRIV_ID                                                (154U)
#define CSL_ISC_GICSS0_MEM_WR_VBUSM_MMR_BASE                                                       (0x45882000ul)
#define CSL_ISC_GICSS0_MEM_WR_VBUSM_REGION_COUNT                                                   (1U)
#define CSL_ISC_GICSS0_MEM_RD_VBUSM_TYPE                                                           (CSL_ISC_DEFAULT)
#define CSL_ISC_GICSS0_MEM_RD_VBUSM_ID                                                             (521U)
#define CSL_ISC_GICSS0_MEM_RD_VBUSM_DEFAULT_PRIV_ID                                                (154U)
#define CSL_ISC_GICSS0_MEM_RD_VBUSM_MMR_BASE                                                       (0x45882400ul)
#define CSL_ISC_GICSS0_MEM_RD_VBUSM_REGION_COUNT                                                   (1U)

/* Properties of firewall at master: PRU_ICSSG0 */
#define CSL_ISC_PRU_ICSSG0_PR1_EXT_VBUSM_TYPE                                                      (CSL_ISC_DEFAULT)
#define CSL_ISC_PRU_ICSSG0_PR1_EXT_VBUSM_ID                                                        (542U)
#define CSL_ISC_PRU_ICSSG0_PR1_EXT_VBUSM_DEFAULT_PRIV_ID                                           (136U)
#define CSL_ISC_PRU_ICSSG0_PR1_EXT_VBUSM_MMR_BASE                                                  (0x45887800ul)
#define CSL_ISC_PRU_ICSSG0_PR1_EXT_VBUSM_REGION_COUNT                                              (16U)

/* Properties of firewall at master: PRU_ICSSG1 */
#define CSL_ISC_PRU_ICSSG1_PR1_EXT_VBUSM_TYPE                                                      (CSL_ISC_DEFAULT)
#define CSL_ISC_PRU_ICSSG1_PR1_EXT_VBUSM_ID                                                        (543U)
#define CSL_ISC_PRU_ICSSG1_PR1_EXT_VBUSM_DEFAULT_PRIV_ID                                           (137U)
#define CSL_ISC_PRU_ICSSG1_PR1_EXT_VBUSM_MMR_BASE                                                  (0x45887c00ul)
#define CSL_ISC_PRU_ICSSG1_PR1_EXT_VBUSM_REGION_COUNT                                              (16U)

/* Properties of firewall at master: PCIE0 */
#define CSL_ISC_PCIE0_PCIE_MST_RD_TYPE                                                             (CSL_ISC_DEFAULT)
#define CSL_ISC_PCIE0_PCIE_MST_RD_ID                                                               (544U)
#define CSL_ISC_PCIE0_PCIE_MST_RD_DEFAULT_PRIV_ID                                                  (179U)
#define CSL_ISC_PCIE0_PCIE_MST_RD_MMR_BASE                                                         (0x45888000ul)
#define CSL_ISC_PCIE0_PCIE_MST_RD_REGION_COUNT                                                     (8U)
#define CSL_ISC_PCIE0_PCIE_MST_WR_TYPE                                                             (CSL_ISC_DEFAULT)
#define CSL_ISC_PCIE0_PCIE_MST_WR_ID                                                               (545U)
#define CSL_ISC_PCIE0_PCIE_MST_WR_DEFAULT_PRIV_ID                                                  (179U)
#define CSL_ISC_PCIE0_PCIE_MST_WR_MMR_BASE                                                         (0x45888400ul)
#define CSL_ISC_PCIE0_PCIE_MST_WR_REGION_COUNT                                                     (8U)

/* Properties of firewall at master: MMCSD1 */
#define CSL_ISC_MMCSD1_EMMCSDSS_WR_TYPE                                                            (CSL_ISC_DEFAULT)
#define CSL_ISC_MMCSD1_EMMCSDSS_WR_ID                                                              (546U)
#define CSL_ISC_MMCSD1_EMMCSDSS_WR_DEFAULT_PRIV_ID                                                 (129U)
#define CSL_ISC_MMCSD1_EMMCSDSS_WR_MMR_BASE                                                        (0x45888800ul)
#define CSL_ISC_MMCSD1_EMMCSDSS_WR_REGION_COUNT                                                    (1U)
#define CSL_ISC_MMCSD1_EMMCSDSS_RD_TYPE                                                            (CSL_ISC_DEFAULT)
#define CSL_ISC_MMCSD1_EMMCSDSS_RD_ID                                                              (547U)
#define CSL_ISC_MMCSD1_EMMCSDSS_RD_DEFAULT_PRIV_ID                                                 (129U)
#define CSL_ISC_MMCSD1_EMMCSDSS_RD_MMR_BASE                                                        (0x45888c00ul)
#define CSL_ISC_MMCSD1_EMMCSDSS_RD_REGION_COUNT                                                    (1U)

/* Properties of firewall at master: SA2_UL0 */
#define CSL_ISC_SA2_UL0_CTXCACH_EXT_DMA_TYPE                                                       (CSL_ISC_DEFAULT)
#define CSL_ISC_SA2_UL0_CTXCACH_EXT_DMA_ID                                                         (548U)
#define CSL_ISC_SA2_UL0_CTXCACH_EXT_DMA_DEFAULT_PRIV_ID                                            (152U)
#define CSL_ISC_SA2_UL0_CTXCACH_EXT_DMA_MMR_BASE                                                   (0x45889000ul)
#define CSL_ISC_SA2_UL0_CTXCACH_EXT_DMA_REGION_COUNT                                               (1U)

/* Properties of firewall at master: USB0 */
#define CSL_ISC_USB0_MSTR0_TYPE                                                                    (CSL_ISC_DEFAULT)
#define CSL_ISC_USB0_MSTR0_ID                                                                      (549U)
#define CSL_ISC_USB0_MSTR0_DEFAULT_PRIV_ID                                                         (155U)
#define CSL_ISC_USB0_MSTR0_MMR_BASE                                                                (0x45889400ul)
#define CSL_ISC_USB0_MSTR0_REGION_COUNT                                                            (8U)
#define CSL_ISC_USB0_MSTW0_TYPE                                                                    (CSL_ISC_DEFAULT)
#define CSL_ISC_USB0_MSTW0_ID                                                                      (550U)
#define CSL_ISC_USB0_MSTW0_DEFAULT_PRIV_ID                                                         (155U)
#define CSL_ISC_USB0_MSTW0_MMR_BASE                                                                (0x45889800ul)
#define CSL_ISC_USB0_MSTW0_REGION_COUNT                                                            (8U)

/* Properties of firewall at master: LED0 */
#define CSL_ISC_LED0_VBUSP_TYPE                                                                    (CSL_ISC_DEFAULT)
#define CSL_ISC_LED0_VBUSP_ID                                                                      (551U)
#define CSL_ISC_LED0_VBUSP_DEFAULT_PRIV_ID                                                         (176U)
#define CSL_ISC_LED0_VBUSP_MMR_BASE                                                                (0x45889c00ul)
#define CSL_ISC_LED0_VBUSP_REGION_COUNT                                                            (1U)

/* Properties of firewall at master: DEBUGSS_WRAP0 */
#define CSL_ISC_DEBUGSS_WRAP0_VBUSMR_TYPE                                                          (CSL_ISC_DEFAULT)
#define CSL_ISC_DEBUGSS_WRAP0_VBUSMR_ID                                                            (552U)
#define CSL_ISC_DEBUGSS_WRAP0_VBUSMR_DEFAULT_PRIV_ID                                               (177U)
#define CSL_ISC_DEBUGSS_WRAP0_VBUSMR_MMR_BASE                                                      (0x4588a000ul)
#define CSL_ISC_DEBUGSS_WRAP0_VBUSMR_REGION_COUNT                                                  (1U)
#define CSL_ISC_DEBUGSS_WRAP0_VBUSMW_TYPE                                                          (CSL_ISC_DEFAULT)
#define CSL_ISC_DEBUGSS_WRAP0_VBUSMW_ID                                                            (553U)
#define CSL_ISC_DEBUGSS_WRAP0_VBUSMW_DEFAULT_PRIV_ID                                               (177U)
#define CSL_ISC_DEBUGSS_WRAP0_VBUSMW_MMR_BASE                                                      (0x4588a400ul)
#define CSL_ISC_DEBUGSS_WRAP0_VBUSMW_REGION_COUNT                                                  (1U)

#ifdef __cplusplus
}
#endif
#endif /* CSLR_SOC_ISC_H_ */

