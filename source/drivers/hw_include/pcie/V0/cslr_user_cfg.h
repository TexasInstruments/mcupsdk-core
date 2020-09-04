/********************************************************************
 * Copyright (C) 2022 Texas Instruments Incorporated.
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
 *  Name        : cslr_user_cfg.h
*/
#ifndef CSLR_USER_CFG_H_
#define CSLR_USER_CFG_H_

#ifdef __cplusplus
extern "C"
{
#endif
#include <drivers/hw_include/cslr.h>
#include <stdint.h>

/**************************************************************************
* Hardware Region  :
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t REVID;
    volatile uint32_t CMD_STATUS;
    volatile uint32_t RSTCMD;
    volatile uint32_t INITCFG;
    volatile uint32_t PMCMD;
    volatile uint32_t LINKSTATUS;
    volatile uint32_t LEGACY_INTR_SET;
    volatile uint32_t LEGACY_INT_PENDING;
    volatile uint32_t MSI_STAT;
    volatile uint32_t MSI_VECTOR;
    volatile uint32_t MSI_MASK_PF0;
    volatile uint8_t  Resv_64[20];
    volatile uint32_t MSI_PENDING_STATUS_PF0;
    volatile uint8_t  Resv_164[96];
    volatile uint32_t MSIX_STAT;
    volatile uint32_t MSIX_MASK;
    volatile uint8_t  Resv_180[8];
    volatile uint32_t FLR_DONE;
    volatile uint8_t  Resv_188[4];
    volatile uint32_t PTM_CFG;
    volatile uint32_t PTM_TIMER_LOW;
    volatile uint32_t PTM_TIMER_HIGH;
    volatile uint32_t EOI_VECTOR;
} CSL_user_cfgRegs;

/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_USER_CFG_REVID                                                     (0x00000000U)
#define CSL_USER_CFG_CMD_STATUS                                                (0x00000004U)
#define CSL_USER_CFG_RSTCMD                                                    (0x00000008U)
#define CSL_USER_CFG_INITCFG                                                   (0x0000000CU)
#define CSL_USER_CFG_PMCMD                                                     (0x00000010U)
#define CSL_USER_CFG_LINKSTATUS                                                (0x00000014U)
#define CSL_USER_CFG_LEGACY_INTR_SET                                           (0x00000018U)
#define CSL_USER_CFG_LEGACY_INT_PENDING                                        (0x0000001CU)
#define CSL_USER_CFG_MSI_STAT                                                  (0x00000020U)
#define CSL_USER_CFG_MSI_VECTOR                                                (0x00000024U)
#define CSL_USER_CFG_MSI_MASK_PF0                                              (0x00000028U)
#define CSL_USER_CFG_MSI_MASK_PF1                                              (0x0000002CU)
#define CSL_USER_CFG_MSI_MASK_PF2                                              (0x00000030U)
#define CSL_USER_CFG_MSI_MASK_PF3                                              (0x00000034U)
#define CSL_USER_CFG_MSI_MASK_PF4                                              (0x00000038U)
#define CSL_USER_CFG_MSI_MASK_PF5                                              (0x0000003CU)
#define CSL_USER_CFG_MSI_PENDING_STATUS_PF0                                    (0x00000040U)
#define CSL_USER_CFG_MSI_PENDING_STATUS_PF1                                    (0x00000044U)
#define CSL_USER_CFG_MSI_PENDING_STATUS_PF2                                    (0x00000048U)
#define CSL_USER_CFG_MSI_PENDING_STATUS_PF3                                    (0x0000004CU)
#define CSL_USER_CFG_MSI_PENDING_STATUS_PF4                                    (0x00000050U)
#define CSL_USER_CFG_MSI_PENDING_STATUS_PF5                                    (0x00000054U)
#define CSL_USER_CFG_MSI_STAT_VF                                               (0x00000058U)
#define CSL_USER_CFG_MSI_VECTOR0_VF                                            (0x0000005CU)
#define CSL_USER_CFG_MSI_VECTOR1_VF                                            (0x00000060U)
#define CSL_USER_CFG_MSI_MASK_VF0                                              (0x00000064U)
#define CSL_USER_CFG_MSI_MASK_VF1                                              (0x00000068U)
#define CSL_USER_CFG_MSI_MASK_VF2                                              (0x0000006CU)
#define CSL_USER_CFG_MSI_MASK_VF3                                              (0x00000070U)
#define CSL_USER_CFG_MSI_MASK_VF4                                              (0x00000074U)
#define CSL_USER_CFG_MSI_MASK_VF5                                              (0x00000078U)
#define CSL_USER_CFG_MSI_MASK_VF6                                              (0x0000007CU)
#define CSL_USER_CFG_MSI_MASK_VF7                                              (0x00000080U)
#define CSL_USER_CFG_MSI_MASK_VF8                                              (0x00000084U)
#define CSL_USER_CFG_MSI_MASK_VF9                                              (0x00000088U)
#define CSL_USER_CFG_MSI_MASK_VF10                                             (0x0000008CU)
#define CSL_USER_CFG_MSI_MASK_VF11                                             (0x00000090U)
#define CSL_USER_CFG_MSI_MASK_VF12                                             (0x00000094U)
#define CSL_USER_CFG_MSI_MASK_VF13                                             (0x00000098U)
#define CSL_USER_CFG_MSI_MASK_VF14                                             (0x0000009CU)
#define CSL_USER_CFG_MSI_MASK_VF15                                             (0x000000A0U)
#define CSL_USER_CFG_MSIX_STAT                                                 (0x000000A4U)
#define CSL_USER_CFG_MSIX_MASK                                                 (0x000000A8U)
#define CSL_USER_CFG_MSIX_STAT_VF                                              (0x000000ACU)
#define CSL_USER_CFG_MSIX_MASK_VF                                              (0x000000B0U)
#define CSL_USER_CFG_FLR_DONE                                                  (0x000000B4U)
#define CSL_USER_CFG_VF_FLR_DONE                                               (0x000000B8U)
#define CSL_USER_CFG_PTM                                                       (0x000000BCU)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* REVID */

#define CSL_USER_CFG_REVID_MODID_MASK                                          (0xFFFF0000U)
#define CSL_USER_CFG_REVID_MODID_SHIFT                                         (0x00000010U)
#define CSL_USER_CFG_REVID_MODID_MAX                                           (0x0000FFFFU)

#define CSL_USER_CFG_REVID_REVRTL_MASK                                         (0x0000F800U)
#define CSL_USER_CFG_REVID_REVRTL_SHIFT                                        (0x0000000BU)
#define CSL_USER_CFG_REVID_REVRTL_MAX                                          (0x0000001FU)

#define CSL_USER_CFG_REVID_REVMAJ_MASK                                         (0x00000700U)
#define CSL_USER_CFG_REVID_REVMAJ_SHIFT                                        (0x00000008U)
#define CSL_USER_CFG_REVID_REVMAJ_MAX                                          (0x00000007U)

#define CSL_USER_CFG_REVID_CUSTOM_MASK                                         (0x000000C0U)
#define CSL_USER_CFG_REVID_CUSTOM_SHIFT                                        (0x00000006U)
#define CSL_USER_CFG_REVID_CUSTOM_MAX                                          (0x00000003U)

#define CSL_USER_CFG_REVID_REVMIN_MASK                                         (0x0000003FU)
#define CSL_USER_CFG_REVID_REVMIN_SHIFT                                        (0x00000000U)
#define CSL_USER_CFG_REVID_REVMIN_MAX                                          (0x0000003FU)

/* CMD_STATUS */

#define CSL_USER_CFG_CMD_STATUS_LINK_TRAINING_ENABLE_MASK                      (0x00000001U)
#define CSL_USER_CFG_CMD_STATUS_LINK_TRAINING_ENABLE_SHIFT                     (0x00000000U)
#define CSL_USER_CFG_CMD_STATUS_LINK_TRAINING_ENABLE_MAX                       (0x00000001U)

/* RSTCMD */

#define CSL_USER_CFG_RSTCMD_INIT_HOT_RESET_MASK                                (0x00000001U)
#define CSL_USER_CFG_RSTCMD_INIT_HOT_RESET_SHIFT                               (0x00000000U)
#define CSL_USER_CFG_RSTCMD_INIT_HOT_RESET_MAX                                 (0x00000001U)

/* INITCFG */

#define CSL_USER_CFG_INITCFG_SRIS_ENABLE_MASK                                  (0x00000001U)
#define CSL_USER_CFG_INITCFG_SRIS_ENABLE_SHIFT                                 (0x00000000U)
#define CSL_USER_CFG_INITCFG_SRIS_ENABLE_MAX                                   (0x00000001U)

#define CSL_USER_CFG_INITCFG_DISABLE_GEN3_DC_BALANCE_MASK                      (0x00000002U)
#define CSL_USER_CFG_INITCFG_DISABLE_GEN3_DC_BALANCE_SHIFT                     (0x00000001U)
#define CSL_USER_CFG_INITCFG_DISABLE_GEN3_DC_BALANCE_MAX                       (0x00000001U)

#define CSL_USER_CFG_INITCFG_SUPPORTED_PRESET_MASK                             (0x00001FFCU)
#define CSL_USER_CFG_INITCFG_SUPPORTED_PRESET_SHIFT                            (0x00000002U)
#define CSL_USER_CFG_INITCFG_SUPPORTED_PRESET_MAX                              (0x000007FFU)

#define CSL_USER_CFG_INITCFG_BYPASS_REMOTE_TX_EQUALIZATION_MASK                (0x00002000U)
#define CSL_USER_CFG_INITCFG_BYPASS_REMOTE_TX_EQUALIZATION_SHIFT               (0x0000000DU)
#define CSL_USER_CFG_INITCFG_BYPASS_REMOTE_TX_EQUALIZATION_MAX                 (0x00000001U)

#define CSL_USER_CFG_INITCFG_BYPASS_PHASE23_MASK                               (0x00004000U)
#define CSL_USER_CFG_INITCFG_BYPASS_PHASE23_SHIFT                              (0x0000000EU)
#define CSL_USER_CFG_INITCFG_BYPASS_PHASE23_MAX                                (0x00000001U)

#define CSL_USER_CFG_INITCFG_MAX_EVAL_ITERATION_MASK                           (0x003F8000U)
#define CSL_USER_CFG_INITCFG_MAX_EVAL_ITERATION_SHIFT                          (0x0000000FU)
#define CSL_USER_CFG_INITCFG_MAX_EVAL_ITERATION_MAX                            (0x0000007FU)

#define CSL_USER_CFG_INITCFG_VC_COUNT_MASK                                     (0x00C00000U)
#define CSL_USER_CFG_INITCFG_VC_COUNT_SHIFT                                    (0x00000016U)
#define CSL_USER_CFG_INITCFG_VC_COUNT_MAX                                      (0x00000003U)

#define CSL_USER_CFG_INITCFG_CONFIG_ENABLE_MASK                                (0x01000000U)
#define CSL_USER_CFG_INITCFG_CONFIG_ENABLE_SHIFT                               (0x00000018U)
#define CSL_USER_CFG_INITCFG_CONFIG_ENABLE_MAX                                 (0x00000001U)

/* PMCMD */

#define CSL_USER_CFG_PMCMD_CLIENT_REQ_EXIT_L1_MASK                             (0x00000001U)
#define CSL_USER_CFG_PMCMD_CLIENT_REQ_EXIT_L1_SHIFT                            (0x00000000U)
#define CSL_USER_CFG_PMCMD_CLIENT_REQ_EXIT_L1_MAX                              (0x00000001U)

#define CSL_USER_CFG_PMCMD_CLIENT_REQ_EXIT_L1_SUBSTATE_MASK                    (0x00000002U)
#define CSL_USER_CFG_PMCMD_CLIENT_REQ_EXIT_L1_SUBSTATE_SHIFT                   (0x00000001U)
#define CSL_USER_CFG_PMCMD_CLIENT_REQ_EXIT_L1_SUBSTATE_MAX                     (0x00000001U)

/* LINKSTATUS */

#define CSL_USER_CFG_LINKSTATUS_LINK_STATUS_MASK                               (0x00000003U)
#define CSL_USER_CFG_LINKSTATUS_LINK_STATUS_SHIFT                              (0x00000000U)
#define CSL_USER_CFG_LINKSTATUS_LINK_STATUS_MAX                                (0x00000003U)

#define CSL_USER_CFG_LINKSTATUS_NEGOTIATED_LINK_WIDTH_MASK                     (0x00000004U)
#define CSL_USER_CFG_LINKSTATUS_NEGOTIATED_LINK_WIDTH_SHIFT                    (0x00000002U)
#define CSL_USER_CFG_LINKSTATUS_NEGOTIATED_LINK_WIDTH_MAX                      (0x00000001U)

#define CSL_USER_CFG_LINKSTATUS_NEGOTIATED_SPEED_MASK                          (0x00000018U)
#define CSL_USER_CFG_LINKSTATUS_NEGOTIATED_SPEED_SHIFT                         (0x00000003U)
#define CSL_USER_CFG_LINKSTATUS_NEGOTIATED_SPEED_MAX                           (0x00000003U)

#define CSL_USER_CFG_LINKSTATUS_LINK_POWER_STATE_MASK                          (0x00000F00U)
#define CSL_USER_CFG_LINKSTATUS_LINK_POWER_STATE_SHIFT                         (0x00000008U)
#define CSL_USER_CFG_LINKSTATUS_LINK_POWER_STATE_MAX                           (0x0000000FU)

#define CSL_USER_CFG_LINKSTATUS_L1_PM_SUBSTATE_MASK                            (0x00007000U)
#define CSL_USER_CFG_LINKSTATUS_L1_PM_SUBSTATE_SHIFT                           (0x0000000CU)
#define CSL_USER_CFG_LINKSTATUS_L1_PM_SUBSTATE_MAX                             (0x00000007U)

#define CSL_USER_CFG_LINKSTATUS_LTSSM_STATE_MASK                               (0x3F000000U)
#define CSL_USER_CFG_LINKSTATUS_LTSSM_STATE_SHIFT                              (0x00000018U)
#define CSL_USER_CFG_LINKSTATUS_LTSSM_STATE_MAX                                (0x0000003FU)

/* LEGACY_INTR_SET */

#define CSL_USER_CFG_LEGACY_INTR_SET_INTA_IN_MASK                              (0x00000001U)
#define CSL_USER_CFG_LEGACY_INTR_SET_INTA_IN_SHIFT                             (0x00000000U)
#define CSL_USER_CFG_LEGACY_INTR_SET_INTA_IN_MAX                               (0x00000001U)

#define CSL_USER_CFG_LEGACY_INTR_SET_INTB_IN_MASK                              (0x00000002U)
#define CSL_USER_CFG_LEGACY_INTR_SET_INTB_IN_SHIFT                             (0x00000001U)
#define CSL_USER_CFG_LEGACY_INTR_SET_INTB_IN_MAX                               (0x00000001U)

#define CSL_USER_CFG_LEGACY_INTR_SET_INTC_IN_MASK                              (0x00000004U)
#define CSL_USER_CFG_LEGACY_INTR_SET_INTC_IN_SHIFT                             (0x00000002U)
#define CSL_USER_CFG_LEGACY_INTR_SET_INTC_IN_MAX                               (0x00000001U)

#define CSL_USER_CFG_LEGACY_INTR_SET_INTD_IN_MASK                              (0x00000008U)
#define CSL_USER_CFG_LEGACY_INTR_SET_INTD_IN_SHIFT                             (0x00000003U)
#define CSL_USER_CFG_LEGACY_INTR_SET_INTD_IN_MAX                               (0x00000001U)

/* LEGACY_INT_PENDING */

#define CSL_USER_CFG_LEGACY_INT_PENDING_INT_PENDING_STATUS_MASK                (0x0000003FU)
#define CSL_USER_CFG_LEGACY_INT_PENDING_INT_PENDING_STATUS_SHIFT               (0x00000000U)
#define CSL_USER_CFG_LEGACY_INT_PENDING_INT_PENDING_STATUS_MAX                 (0x0000003FU)

/* MSI_STAT */

#define CSL_USER_CFG_MSI_STAT_MSI_ENABLE_MASK                                  (0x0000003FU)
#define CSL_USER_CFG_MSI_STAT_MSI_ENABLE_SHIFT                                 (0x00000000U)
#define CSL_USER_CFG_MSI_STAT_MSI_ENABLE_MAX                                   (0x0000003FU)

/* MSI_VECTOR */

#define CSL_USER_CFG_MSI_VECTOR_MSI_VECTOR_COUNT_MASK                          (0x0003FFFFU)
#define CSL_USER_CFG_MSI_VECTOR_MSI_VECTOR_COUNT_SHIFT                         (0x00000000U)
#define CSL_USER_CFG_MSI_VECTOR_MSI_VECTOR_COUNT_MAX                           (0x0003FFFFU)

/* MSI_MASK_PF0 */

#define CSL_USER_CFG_MSI_MASK_PF0_MSI_MASK_PF0_MASK                            (0xFFFFFFFFU)
#define CSL_USER_CFG_MSI_MASK_PF0_MSI_MASK_PF0_SHIFT                           (0x00000000U)
#define CSL_USER_CFG_MSI_MASK_PF0_MSI_MASK_PF0_MAX                             (0xFFFFFFFFU)

/* MSI_MASK_PF1 */

#define CSL_USER_CFG_MSI_MASK_PF1_MSI_MASK_PF1_MASK                            (0xFFFFFFFFU)
#define CSL_USER_CFG_MSI_MASK_PF1_MSI_MASK_PF1_SHIFT                           (0x00000000U)
#define CSL_USER_CFG_MSI_MASK_PF1_MSI_MASK_PF1_MAX                             (0xFFFFFFFFU)

/* MSI_MASK_PF2 */

#define CSL_USER_CFG_MSI_MASK_PF2_MSI_MASK_PF2_MASK                            (0xFFFFFFFFU)
#define CSL_USER_CFG_MSI_MASK_PF2_MSI_MASK_PF2_SHIFT                           (0x00000000U)
#define CSL_USER_CFG_MSI_MASK_PF2_MSI_MASK_PF2_MAX                             (0xFFFFFFFFU)

/* MSI_MASK_PF3 */

#define CSL_USER_CFG_MSI_MASK_PF3_MSI_MASK_PF3_MASK                            (0xFFFFFFFFU)
#define CSL_USER_CFG_MSI_MASK_PF3_MSI_MASK_PF3_SHIFT                           (0x00000000U)
#define CSL_USER_CFG_MSI_MASK_PF3_MSI_MASK_PF3_MAX                             (0xFFFFFFFFU)

/* MSI_MASK_PF4 */

#define CSL_USER_CFG_MSI_MASK_PF4_MSI_MASK_PF4_MASK                            (0xFFFFFFFFU)
#define CSL_USER_CFG_MSI_MASK_PF4_MSI_MASK_PF4_SHIFT                           (0x00000000U)
#define CSL_USER_CFG_MSI_MASK_PF4_MSI_MASK_PF4_MAX                             (0xFFFFFFFFU)

/* MSI_MASK_PF5 */

#define CSL_USER_CFG_MSI_MASK_PF5_MSI_MASK_PF5_MASK                            (0xFFFFFFFFU)
#define CSL_USER_CFG_MSI_MASK_PF5_MSI_MASK_PF5_SHIFT                           (0x00000000U)
#define CSL_USER_CFG_MSI_MASK_PF5_MSI_MASK_PF5_MAX                             (0xFFFFFFFFU)

/* MSI_PENDING_STATUS_PF0 */

#define CSL_USER_CFG_MSI_PENDING_STATUS_PF0_MSI_PENDING_STATUS_PF0_MASK        (0xFFFFFFFFU)
#define CSL_USER_CFG_MSI_PENDING_STATUS_PF0_MSI_PENDING_STATUS_PF0_SHIFT       (0x00000000U)
#define CSL_USER_CFG_MSI_PENDING_STATUS_PF0_MSI_PENDING_STATUS_PF0_MAX         (0xFFFFFFFFU)

/* MSI_PENDING_STATUS_PF1 */

#define CSL_USER_CFG_MSI_PENDING_STATUS_PF1_MSI_PENDING_STATUS_PF1_MASK        (0xFFFFFFFFU)
#define CSL_USER_CFG_MSI_PENDING_STATUS_PF1_MSI_PENDING_STATUS_PF1_SHIFT       (0x00000000U)
#define CSL_USER_CFG_MSI_PENDING_STATUS_PF1_MSI_PENDING_STATUS_PF1_MAX         (0xFFFFFFFFU)

/* MSI_PENDING_STATUS_PF2 */

#define CSL_USER_CFG_MSI_PENDING_STATUS_PF2_MSI_PENDING_STATUS_PF2_MASK        (0xFFFFFFFFU)
#define CSL_USER_CFG_MSI_PENDING_STATUS_PF2_MSI_PENDING_STATUS_PF2_SHIFT       (0x00000000U)
#define CSL_USER_CFG_MSI_PENDING_STATUS_PF2_MSI_PENDING_STATUS_PF2_MAX         (0xFFFFFFFFU)

/* MSI_PENDING_STATUS_PF3 */

#define CSL_USER_CFG_MSI_PENDING_STATUS_PF3_MSI_PENDING_STATUS_PF3_MASK        (0xFFFFFFFFU)
#define CSL_USER_CFG_MSI_PENDING_STATUS_PF3_MSI_PENDING_STATUS_PF3_SHIFT       (0x00000000U)
#define CSL_USER_CFG_MSI_PENDING_STATUS_PF3_MSI_PENDING_STATUS_PF3_MAX         (0xFFFFFFFFU)

/* MSI_PENDING_STATUS_PF4 */

#define CSL_USER_CFG_MSI_PENDING_STATUS_PF4_MSI_PENDING_STATUS_PF4_MASK        (0xFFFFFFFFU)
#define CSL_USER_CFG_MSI_PENDING_STATUS_PF4_MSI_PENDING_STATUS_PF4_SHIFT       (0x00000000U)
#define CSL_USER_CFG_MSI_PENDING_STATUS_PF4_MSI_PENDING_STATUS_PF4_MAX         (0xFFFFFFFFU)

/* MSI_PENDING_STATUS_PF5 */

#define CSL_USER_CFG_MSI_PENDING_STATUS_PF5_MSI_PENDING_STATUS_PF5_MASK        (0xFFFFFFFFU)
#define CSL_USER_CFG_MSI_PENDING_STATUS_PF5_MSI_PENDING_STATUS_PF5_SHIFT       (0x00000000U)
#define CSL_USER_CFG_MSI_PENDING_STATUS_PF5_MSI_PENDING_STATUS_PF5_MAX         (0xFFFFFFFFU)

/* MSI_STAT_VF */

#define CSL_USER_CFG_MSI_STAT_VF_VF_MSI_ENABLE_MASK                            (0x0000FFFFU)
#define CSL_USER_CFG_MSI_STAT_VF_VF_MSI_ENABLE_SHIFT                           (0x00000000U)
#define CSL_USER_CFG_MSI_STAT_VF_VF_MSI_ENABLE_MAX                             (0x0000FFFFU)

/* MSI_VECTOR0_VF */

#define CSL_USER_CFG_MSI_VECTOR0_VF_VF_MSI_VECTOR_COUNT0_MASK                  (0x00FFFFFFU)
#define CSL_USER_CFG_MSI_VECTOR0_VF_VF_MSI_VECTOR_COUNT0_SHIFT                 (0x00000000U)
#define CSL_USER_CFG_MSI_VECTOR0_VF_VF_MSI_VECTOR_COUNT0_MAX                   (0x00FFFFFFU)

/* MSI_VECTOR1_VF */

#define CSL_USER_CFG_MSI_VECTOR1_VF_VF_MSI_VECTOR_COUNT1_MASK                  (0x00FFFFFFU)
#define CSL_USER_CFG_MSI_VECTOR1_VF_VF_MSI_VECTOR_COUNT1_SHIFT                 (0x00000000U)
#define CSL_USER_CFG_MSI_VECTOR1_VF_VF_MSI_VECTOR_COUNT1_MAX                   (0x00FFFFFFU)

/* MSI_MASK_VF0 */

#define CSL_USER_CFG_MSI_MASK_VF0_MSI_MASK_VF0_MASK                            (0xFFFFFFFFU)
#define CSL_USER_CFG_MSI_MASK_VF0_MSI_MASK_VF0_SHIFT                           (0x00000000U)
#define CSL_USER_CFG_MSI_MASK_VF0_MSI_MASK_VF0_MAX                             (0xFFFFFFFFU)

/* MSI_MASK_VF1 */

#define CSL_USER_CFG_MSI_MASK_VF1_MSI_MASK_VF1_MASK                            (0xFFFFFFFFU)
#define CSL_USER_CFG_MSI_MASK_VF1_MSI_MASK_VF1_SHIFT                           (0x00000000U)
#define CSL_USER_CFG_MSI_MASK_VF1_MSI_MASK_VF1_MAX                             (0xFFFFFFFFU)

/* MSI_MASK_VF2 */

#define CSL_USER_CFG_MSI_MASK_VF2_MSI_MASK_VF2_MASK                            (0xFFFFFFFFU)
#define CSL_USER_CFG_MSI_MASK_VF2_MSI_MASK_VF2_SHIFT                           (0x00000000U)
#define CSL_USER_CFG_MSI_MASK_VF2_MSI_MASK_VF2_MAX                             (0xFFFFFFFFU)

/* MSI_MASK_VF3 */

#define CSL_USER_CFG_MSI_MASK_VF3_MSI_MASK_VF3_MASK                            (0xFFFFFFFFU)
#define CSL_USER_CFG_MSI_MASK_VF3_MSI_MASK_VF3_SHIFT                           (0x00000000U)
#define CSL_USER_CFG_MSI_MASK_VF3_MSI_MASK_VF3_MAX                             (0xFFFFFFFFU)

/* MSI_MASK_VF4 */

#define CSL_USER_CFG_MSI_MASK_VF4_MSI_MASK_VF4_MASK                            (0xFFFFFFFFU)
#define CSL_USER_CFG_MSI_MASK_VF4_MSI_MASK_VF4_SHIFT                           (0x00000000U)
#define CSL_USER_CFG_MSI_MASK_VF4_MSI_MASK_VF4_MAX                             (0xFFFFFFFFU)

/* MSI_MASK_VF5 */

#define CSL_USER_CFG_MSI_MASK_VF5_MSI_MASK_VF5_MASK                            (0xFFFFFFFFU)
#define CSL_USER_CFG_MSI_MASK_VF5_MSI_MASK_VF5_SHIFT                           (0x00000000U)
#define CSL_USER_CFG_MSI_MASK_VF5_MSI_MASK_VF5_MAX                             (0xFFFFFFFFU)

/* MSI_MASK_VF6 */

#define CSL_USER_CFG_MSI_MASK_VF6_MSI_MASK_VF6_MASK                            (0xFFFFFFFFU)
#define CSL_USER_CFG_MSI_MASK_VF6_MSI_MASK_VF6_SHIFT                           (0x00000000U)
#define CSL_USER_CFG_MSI_MASK_VF6_MSI_MASK_VF6_MAX                             (0xFFFFFFFFU)

/* MSI_MASK_VF7 */

#define CSL_USER_CFG_MSI_MASK_VF7_MSI_MASK_VF7_MASK                            (0xFFFFFFFFU)
#define CSL_USER_CFG_MSI_MASK_VF7_MSI_MASK_VF7_SHIFT                           (0x00000000U)
#define CSL_USER_CFG_MSI_MASK_VF7_MSI_MASK_VF7_MAX                             (0xFFFFFFFFU)

/* MSI_MASK_VF8 */

#define CSL_USER_CFG_MSI_MASK_VF8_MSI_MASK_VF8_MASK                            (0xFFFFFFFFU)
#define CSL_USER_CFG_MSI_MASK_VF8_MSI_MASK_VF8_SHIFT                           (0x00000000U)
#define CSL_USER_CFG_MSI_MASK_VF8_MSI_MASK_VF8_MAX                             (0xFFFFFFFFU)

/* MSI_MASK_VF9 */

#define CSL_USER_CFG_MSI_MASK_VF9_MSI_MASK_VF9_MASK                            (0xFFFFFFFFU)
#define CSL_USER_CFG_MSI_MASK_VF9_MSI_MASK_VF9_SHIFT                           (0x00000000U)
#define CSL_USER_CFG_MSI_MASK_VF9_MSI_MASK_VF9_MAX                             (0xFFFFFFFFU)

/* MSI_MASK_VF10 */

#define CSL_USER_CFG_MSI_MASK_VF10_MSI_MASK_VF10_MASK                          (0xFFFFFFFFU)
#define CSL_USER_CFG_MSI_MASK_VF10_MSI_MASK_VF10_SHIFT                         (0x00000000U)
#define CSL_USER_CFG_MSI_MASK_VF10_MSI_MASK_VF10_MAX                           (0xFFFFFFFFU)

/* MSI_MASK_VF11 */

#define CSL_USER_CFG_MSI_MASK_VF11_MSI_MASK_VF11_MASK                          (0xFFFFFFFFU)
#define CSL_USER_CFG_MSI_MASK_VF11_MSI_MASK_VF11_SHIFT                         (0x00000000U)
#define CSL_USER_CFG_MSI_MASK_VF11_MSI_MASK_VF11_MAX                           (0xFFFFFFFFU)

/* MSI_MASK_VF12 */

#define CSL_USER_CFG_MSI_MASK_VF12_MSI_MASK_VF12_MASK                          (0xFFFFFFFFU)
#define CSL_USER_CFG_MSI_MASK_VF12_MSI_MASK_VF12_SHIFT                         (0x00000000U)
#define CSL_USER_CFG_MSI_MASK_VF12_MSI_MASK_VF12_MAX                           (0xFFFFFFFFU)

/* MSI_MASK_VF13 */

#define CSL_USER_CFG_MSI_MASK_VF13_MSI_MASK_VF13_MASK                          (0xFFFFFFFFU)
#define CSL_USER_CFG_MSI_MASK_VF13_MSI_MASK_VF13_SHIFT                         (0x00000000U)
#define CSL_USER_CFG_MSI_MASK_VF13_MSI_MASK_VF13_MAX                           (0xFFFFFFFFU)

/* MSI_MASK_VF14 */

#define CSL_USER_CFG_MSI_MASK_VF14_MSI_MASK_VF14_MASK                          (0xFFFFFFFFU)
#define CSL_USER_CFG_MSI_MASK_VF14_MSI_MASK_VF14_SHIFT                         (0x00000000U)
#define CSL_USER_CFG_MSI_MASK_VF14_MSI_MASK_VF14_MAX                           (0xFFFFFFFFU)

/* MSI_MASK_VF15 */

#define CSL_USER_CFG_MSI_MASK_VF15_MSI_MASK_VF15_MASK                          (0xFFFFFFFFU)
#define CSL_USER_CFG_MSI_MASK_VF15_MSI_MASK_VF15_SHIFT                         (0x00000000U)
#define CSL_USER_CFG_MSI_MASK_VF15_MSI_MASK_VF15_MAX                           (0xFFFFFFFFU)

/* MSIX_STAT */

#define CSL_USER_CFG_MSIX_STAT_MSIX_ENABLE_MASK                                (0x0000003FU)
#define CSL_USER_CFG_MSIX_STAT_MSIX_ENABLE_SHIFT                               (0x00000000U)
#define CSL_USER_CFG_MSIX_STAT_MSIX_ENABLE_MAX                                 (0x0000003FU)

/* MSIX_MASK */

#define CSL_USER_CFG_MSIX_MASK_MSIX_MASK_MASK                                  (0x0000003FU)
#define CSL_USER_CFG_MSIX_MASK_MSIX_MASK_SHIFT                                 (0x00000000U)
#define CSL_USER_CFG_MSIX_MASK_MSIX_MASK_MAX                                   (0x0000003FU)

/* MSIX_STAT_VF */

#define CSL_USER_CFG_MSIX_STAT_VF_VF_MSIX_ENABLE_MASK                          (0x0000FFFFU)
#define CSL_USER_CFG_MSIX_STAT_VF_VF_MSIX_ENABLE_SHIFT                         (0x00000000U)
#define CSL_USER_CFG_MSIX_STAT_VF_VF_MSIX_ENABLE_MAX                           (0x0000FFFFU)

/* MSIX_MASK_VF */

#define CSL_USER_CFG_MSIX_MASK_VF_VF_MSIX_MASK_MASK                            (0x0000FFFFU)
#define CSL_USER_CFG_MSIX_MASK_VF_VF_MSIX_MASK_SHIFT                           (0x00000000U)
#define CSL_USER_CFG_MSIX_MASK_VF_VF_MSIX_MASK_MAX                             (0x0000FFFFU)

/* FLR_DONE */

#define CSL_USER_CFG_FLR_DONE_FLR_DONE_MASK                                    (0x0000003FU)
#define CSL_USER_CFG_FLR_DONE_FLR_DONE_SHIFT                                   (0x00000000U)
#define CSL_USER_CFG_FLR_DONE_FLR_DONE_MAX                                     (0x0000003FU)

/* VF_FLR_DONE */

#define CSL_USER_CFG_VF_FLR_DONE_VF_FLR_DONE_MASK                              (0x0000FFFFU)
#define CSL_USER_CFG_VF_FLR_DONE_VF_FLR_DONE_SHIFT                             (0x00000000U)
#define CSL_USER_CFG_VF_FLR_DONE_VF_FLR_DONE_MAX                               (0x0000FFFFU)

/* PTM */

#define CSL_USER_CFG_PTM_PTM_CLK_SEL_MASK                                      (0x0000003FU)
#define CSL_USER_CFG_PTM_PTM_CLK_SEL_SHIFT                                     (0x00000000U)
#define CSL_USER_CFG_PTM_PTM_CLK_SEL_MAX                                       (0x0000003FU)

#ifdef __cplusplus
}
#endif
#endif
