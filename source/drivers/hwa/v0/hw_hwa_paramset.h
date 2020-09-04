/*
 *  Copyright (C) 2021 Texas Instruments Incorporated
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

#ifndef HW_HWA_PARAMSET_H_
#define HW_HWA_PARAMSET_H_

/****************************************************************************************
 * INCLUDE FILES
 ****************************************************************************************/
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Definition for the field CPU_INTR2_EN in register HEADER */
#define HEADER_CPU_INTR2_EN_END                              (31U)
#define HEADER_CPU_INTR2_EN_START                           (31U)

/* Definition for the field CPU_INTR1_EN in register HEADER */
#define HEADER_CPU_INTR1_EN_END                              (30U)
#define HEADER_CPU_INTR1_EN_START                           (30U)

/* Definition for the field FORCED_CONTEXTSW_EN in register HEADER */
#define HEADER_FORCED_CONTEXTSW_EN_END                       (20U)
#define HEADER_FORCED_CONTEXTSW_EN_START                    (20U)

/* Definition for the field CONTEXTSW_EN in register HEADER */
#define HEADER_CONTEXTSW_EN_END                             (19U)
#define HEADER_CONTEXTSW_EN_START                          (19U)

/* Definition for the field ACCEL_MODE in register HEADER */
#define HEADER_ACCEL_MODE_END                               (18U)
#define HEADER_ACCEL_MODE_START                             (16U)

/* Definition for the field HWA2DMA_TRIGDST in register HEADER */
#define HEADER_HWA2DMA_TRIGDST_END                           (15U)
#define HEADER_HWA2DMA_TRIGDST_START                        (11U)

/* Definition for the field DMATRIG_EN in register HEADER */
#define HEADER_DMATRIG_EN_END                                (10U)
#define HEADER_DMATRIG_EN_START                             (10U)

/* Definition for the field HWA_TRIGSRC in register HEADER */
#define HEADER_HWA_TRIGSRC_END                               (8U)
#define HEADER_HWA_TRIGSRC_START                            (4U)

/* Definition for the field TRIGMODE in register HEADER */
#define HEADER_TRIGMODE_END                                  (3U)
#define HEADER_TRIGMODE_START                               (0U)

/* Definition for the field SRCCONJ in register SRC */
#define SRC_SRCCONJ_END                                      (31U)
#define SRC_SRCCONJ_START                                   (31U)

/* Definition for the field SRCIQSWAP in register SRC */
#define SRC_SRCIQSWAP_END                                   (30U)
#define SRC_SRCIQSWAP_START                                 (30U)

/* Definition for the field SRCSIGNED in register SRC */
#define SRC_SRCSIGNED_END                                   (29U)
#define SRC_SRCSIGNED_START                                 (29U)

/* Definition for the field SRC16b32b in register SRC */
#define SRC_SRC16b32b_END                                   (28U)
#define SRC_SRC16b32b_START                                 (28U)

/* Definition for the field SRCREAL in register SRC */
#define SRC_SRCREAL_END                                     (27U)
#define SRC_SRCREAL_START                                   (27U)

/* Definition for the field SHUFFLE_AB in register SRC */
#define SRC_SHUFFLE_AB_END                                  (26U)
#define SRC_SHUFFLE_AB_START                                (25U)

/* Definition for the field SRCSCAL in register SRC */
#define SRC_SRCSCAL_END                                     (23U)
#define SRC_SRCSCAL_START                                   (20U)

/* Definition for the field SRCADDR in register SRC */
#define SRC_SRCADDR_END                                     (19U)
#define SRC_SRCADDR_START                                  (0U)

/* Definition for the field SRCACNT in register SRCA */
#define SRCA_SRCACNT_END                                     (31U)
#define SRCA_SRCACNT_START                                  (20U)

/* Definition for the field SRCAINDX in register SRCA */
#define SRCA_SRCAINDX_END                                   (19U)
#define SRCA_SRCAINDX_START                                 (0U)

/* Definition for the field BCNT in register SRCB */
#define SRCB_BCNT_END                                       (31U)
#define SRCB_BCNT_START                                     (20U)

/* Definition for the field SRCBINDX in register SRCB */
#define SRCB_SRCBINDX_END                                   (19U)
#define SRCB_SRCBINDX_START                                (0U)

/* Definition for the field CCNT in register SRCC */
#define SRCC_CCNT_END                                        (31U)
#define SRCC_CCNT_START                                     (20U)

/* Definition for the field SRCCINDX in register SRCC */
#define SRCC_SRCCINDX_END                                   (19U)
#define SRCC_SRCCINDX_START                                (0U)

/* Definition for the field SRCB_CIRCSHIFT in register CIRCSHIFT */
#define CIRCSHIFT_SRCB_CIRCSHIFT_END                         (27U)
#define CIRCSHIFT_SRCB_CIRCSHIFT_START                      (16U)

/* Definition for the field SRCA_CIRCSHIFT in register CIRCSHIFT */
#define CIRCSHIFT_SRCA_CIRCSHIFT_END                        (11U)
#define CIRCSHIFT_SRCA_CIRCSHIFT_START                      (0U)

/* Definition for the field SRC_CIRCSHIFTWRAP3X in register CIRCSHIFT2 */
#define CIRCSHIFT2_SRC_CIRCSHIFTWRAP3X_END                   (30U)
#define CIRCSHIFT2_SRC_CIRCSHIFTWRAP3X_START                (29U)

/* Definition for the field SRCA_CIRCSHIFTWRAP in register CIRCSHIFT2 */
#define CIRCSHIFT2_SRCA_CIRCSHIFTWRAP_END                    (28U)
#define CIRCSHIFT2_SRCA_CIRCSHIFTWRAP_START                 (25U)

/* Definition for the field SRCB_CIRCSHIFTWRAP in register CIRCSHIFT2 */
#define CIRCSHIFT2_SRCB_CIRCSHIFTWRAP_END                    (24U)
#define CIRCSHIFT2_SRCB_CIRCSHIFTWRAP_START                 (21U)

/* Definition for the field DSTCONJ in register DST */
#define DST_DSTCONJ_END                                      (31U)
#define DST_DSTCONJ_START                                   (31U)

/* Definition for the field DSTIQSWAP in register DST */
#define DST_DSTIQSWAP_END                                    (30U)
#define DST_DSTIQSWAP_START                                 (30U)

/* Definition for the field DSTSIGNED in register DST */
#define DST_DSTSIGNED_END                                    (29U)
#define DST_DSTSIGNED_START                                 (29U)

/* Definition for the field DST16b32b in register DST */
#define DST_DST16b32b_END                                    (28U)
#define DST_DST16b32b_START                                 (28U)

/* Definition for the field DSTREAL in register DST */
#define DST_DSTREAL_END                                      (27U)
#define DST_DSTREAL_START                                   (27U)

/* Definition for the field DSTSCAL in register DST */
#define DST_DSTSCAL_END                                      (23U)
#define DST_DSTSCAL_START                                   (20U)

/* Definition for the field DSTADDR in register DST */
#define DST_DSTADDR_END                                     (19U)
#define DST_DSTADDR_START                                   (0U)

/* Definition for the field DSTACNT in register DSTA */
#define DSTA_DSTACNT_END                                     (31U)
#define DSTA_DSTACNT_START                                  (20U)

/* Definition for the field DSTAINDX in register DSTA */
#define DSTA_DSTAINDX_END                                    (19U)
#define DSTA_DSTAINDX_START                                 (0U)

/* Definition for the field DST_SKIP_INIT in register DSTB */
#define DSTB_DST_SKIP_INIT_END                               (31U)
#define DSTB_DST_SKIP_INIT_START                            (20U)

/* Definition for the field DSTBINDX in register DSTB */
#define DSTB_DSTBINDX_END                                   (19U)
#define DSTB_DSTBINDX_START                                 (0U)


/* Definition for the field CFAR_OS_NON_CYC_VARIANT_EN in register CFAREN */
#define CFAREN_CFAR_OS_NON_CYC_VARIANT_EN_END                (11U)
#define CFAREN_CFAR_OS_NON_CYC_VARIANT_EN_START             (11U)

/* Definition for the field CFAR_ABS_MODE in register CFAREN */
#define CFAREN_CFAR_ABS_MODE_END                             (10U)
#define CFAREN_CFAR_ABS_MODE_START                          (9U)

/* Definition for the field CFAR_ADV_OUT_MODE in register CFAREN */
#define CFAREN_CFAR_ADV_OUT_MODE_END                        (8U)
#define CFAREN_CFAR_ADV_OUT_MODE_START                     (8U)

/* Definition for the field CFAR_OUT_MODE in register CFAREN */
#define CFAREN_CFAR_OUT_MODE_END                             (7U)
#define CFAREN_CFAR_OUT_MODE_START                          (6U)

/* Definition for the field CFAR_GROUPING_EN in register CFAREN */
#define CFAREN_CFAR_GROUPING_EN_END                          (5U)
#define CFAREN_CFAR_GROUPING_EN_START                       (5U)

/* Definition for the field CFAR_CYCLIC in register CFAREN */
#define CFAREN_CFAR_CYCLIC_END                               (4U)
#define CFAREN_CFAR_CYCLIC_START                            (4U)

/* Definition for the field CFAR_INP_MODE in register CFAREN */
#define CFAREN_CFAR_INP_MODE_END                             (3U)
#define CFAREN_CFAR_INP_MODE_START                          (3U)

/* Definition for the field CFAR_LOG_MODE in register CFAREN */
#define CFAREN_CFAR_LOG_MODE_END                             (2U)
#define CFAREN_CFAR_LOG_MODE_START                          (2U)

/* Definition for the field CFAR_CA_MODE in register CFAREN */
#define CFAREN_CFAR_CA_MODE_END                              (1U)
#define CFAREN_CFAR_CA_MODE_START                           (0U)

/* Definition for the field CFAR_OS_KVAL in register CFARCFG */
#define CFARCFG_CFAR_OS_KVAL_END                             (30U)
#define CFARCFG_CFAR_OS_KVAL_START                          (24U)

/* Definition for the field CFAR_GUARD_INT in register CFARCFG */
#define CFARCFG_CFAR_GUARD_INT_END                           (18U)
#define CFARCFG_CFAR_GUARD_INT_START                        (16U)

/* Definition for the field CFAR_NOISEDIV in register CFARCFG */
#define CFARCFG_CFAR_NOISEDIV_END                            (15U)
#define CFARCFG_CFAR_NOISEDIV_START                         (12U)

/* Definition for the field CFAR_AVG_LEFT in register CFARCFG */
#define CFARCFG_CFAR_AVG_LEFT_END                           (11U)
#define CFARCFG_CFAR_AVG_LEFT_START                         (6U)

/* Definition for the field CFAR_AVG_RIGHT in register CFARCFG */
#define CFARCFG_CFAR_AVG_RIGHT_END                           (5U)
#define CFARCFG_CFAR_AVG_RIGHT_START                        (0U)

/* Definition for the field BFLY_SCALING_FFT3X in register BFLYFFT */
#define BFLYFFT_BFLY_SCALING_FFT3X_END                       (29U)
#define BFLYFFT_BFLY_SCALING_FFT3X_START                    (28U)

/* Definition for the field BFLY_SCALING in register BFLYFFT */
#define BFLYFFT_BFLY_SCALING_END                             (27U)
#define BFLYFFT_BFLY_SCALING_START                          (16U)

/* Definition for the field FFTSIZE in register BFLYFFT */
#define BFLYFFT_FFTSIZE_END                                  (15U)
#define BFLYFFT_FFTSIZE_START                               (12U)

/* Definition for the field FFTSIZE_DIM2 in register BFLYFFT */
#define BFLYFFT_FFTSIZE_DIM2_END                            (11U)
#define BFLYFFT_FFTSIZE_DIM2_START                          (8U)

/* Definition for the field BPM_PHASE in register BFLYFFT */
#define BFLYFFT_BPM_PHASE_END                                (7U)
#define BFLYFFT_BPM_PHASE_START                             (4U)

/* Definition for the field ZEROINSERT_EN in register BFLYFFT */
#define BFLYFFT_ZEROINSERT_EN_END                            (3U)
#define BFLYFFT_ZEROINSERT_EN_START                         (3U)

/* Definition for the field FFTSIZE3X_EN in register BFLYFFT */
#define BFLYFFT_FFTSIZE3X_EN_END                             (2U)
#define BFLYFFT_FFTSIZE3X_EN_START                          (2U)

/* Definition for the field BPM_EN in register BFLYFFT */
#define BFLYFFT_BPM_EN_END                                   (1U)
#define BFLYFFT_BPM_EN_START                                (1U)

/* Definition for the field FFT_EN in register BFLYFFT */
#define BFLYFFT_FFT_EN_END                                   (0U)
#define BFLYFFT_FFT_EN_START                                (0U)

/* Definition for the field HIST_SCALE_SEL in register POSTPROCWIN */
#define POSTPROCWIN_HIST_SCALE_SEL_END                       (31U)
#define POSTPROCWIN_HIST_SCALE_SEL_START                    (28U)

/* Definition for the field HIST_SIZE_SEL in register POSTPROCWIN */
#define POSTPROCWIN_HIST_SIZE_SEL_END                        (27U)
#define POSTPROCWIN_HIST_SIZE_SEL_START                     (24U)


/* Definition for the field MAX2D_EN in register POSTPROCWIN */
#define POSTPROCWIN_MAX2D_EN_END                             (22U)
#define POSTPROCWIN_MAX2D_EN_START                          (22U)

/* Definition for the field FFT_OUTPUT_MODE in register POSTPROCWIN */
#define POSTPROCWIN_FFT_OUTPUT_MODE_END                      (21U)
#define POSTPROCWIN_FFT_OUTPUT_MODE_START                   (20U)

/* Definition for the field WINDOW_INTERP_FRACTION in register POSTPROCWIN */
#define POSTPROCWIN_WINDOW_INTERP_FRACTION_END               (19U)
#define POSTPROCWIN_WINDOW_INTERP_FRACTION_START            (18U)

/* Definition for the field ABS_EN in register POSTPROCWIN */
#define POSTPROCWIN_ABS_EN_END                               (17U)
#define POSTPROCWIN_ABS_EN_START                            (17U)

/* Definition for the field LOG2_EN in register POSTPROCWIN */
#define POSTPROCWIN_LOG2_EN_END                              (16U)
#define POSTPROCWIN_LOG2_EN_START                           (16U)

/* Definition for the field WINDOW_MODE in register POSTPROCWIN */
#define POSTPROCWIN_WINDOW_MODE_END                          (14U)
#define POSTPROCWIN_WINDOW_MODE_START                       (13U)

/* Definition for the field WINDOW_START in register POSTPROCWIN */
#define POSTPROCWIN_WINDOW_START_END                        (12U)
#define POSTPROCWIN_WINDOW_START_START                      (2U)

/* Definition for the field WINSYMM in register POSTPROCWIN */
#define POSTPROCWIN_WINSYMM_END                              (1U)
#define POSTPROCWIN_WINSYMM_START                           (1U)

/* Definition for the field WINDOW_EN in register POSTPROCWIN */
#define POSTPROCWIN_WINDOW_EN_END                            (0U)
#define POSTPROCWIN_WINDOW_EN_START                         (0U)

/* Definition for the field CMULT_MODE in register PREPROC1 */
#define PREPROC1_CMULT_MODE_END                              (31U)
#define PREPROC1_CMULT_MODE_START                           (28U)

/* Definition for the field CMULT_SCALE_EN in register PREPROC1 */
#define PREPROC1_CMULT_SCALE_EN_END                          (27U)
#define PREPROC1_CMULT_SCALE_EN_START                       (27U)

/* Definition for the field HIST_MODE in register PREPROC1 */
#define PREPROC1_HIST_MODE_END                               (26U)
#define PREPROC1_HIST_MODE_START                            (25U)

/* Definition for the field INTF_MITG_CNT_THRESH in register PREPROC1 */
#define PREPROC1_INTF_MITG_CNT_THRESH_END                    (24U)
#define PREPROC1_INTF_MITG_CNT_THRESH_START                 (20U)

/* Definition for the field INTF_MITG_EN in register PREPROC1 */
#define PREPROC1_INTF_MITG_EN_END                           (19U)
#define PREPROC1_INTF_MITG_EN_START                         (19U)

/* Definition for the field INTF_MITG_PATH_SEL in register PREPROC1 */
#define PREPROC1_INTF_MITG_PATH_SEL_END                     (17U)
#define PREPROC1_INTF_MITG_PATH_SEL_START                   (16U)

/* Definition for the field RECWIN_MODE in register PREPROC1 */
#define PREPROC1_RECWIN_MODE_END                             (14U)
#define PREPROC1_RECWIN_MODE_START                          (14U)

/* Definition for the field INTF_STATS_RESET_MODE in register PREPROC1 */
#define PREPROC1_INTF_STATS_RESET_MODE_END                   (13U)
#define PREPROC1_INTF_STATS_RESET_MODE_START                (12U)

/* Definition for the field INTF_LOC_THRESH_SEL in register PREPROC1 */
#define PREPROC1_INTF_LOC_THRESH_SEL_END                     (11U)
#define PREPROC1_INTF_LOC_THRESH_SEL_START                  (10U)

/* Definition for the field INTF_LOC_THRESH_MODE in register PREPROC1 */
#define PREPROC1_INTF_LOC_THRESH_MODE_END                   (9U)
#define PREPROC1_INTF_LOC_THRESH_MODE_START                 (8U)

/* Definition for the field CHANCOMB_EN in register PREPROC1 */
#define PREPROC1_CHANCOMB_EN_END                             (7U)
#define PREPROC1_CHANCOMB_EN_START                          (7U)

/* Definition for the field INTF_LOC_THRESH_EN in register PREPROC1 */
#define PREPROC1_INTF_LOC_THRESH_EN_END                      (6U)
#define PREPROC1_INTF_LOC_THRESH_EN_START                   (6U)

/* Definition for the field DCSUB_SEL in register PREPROC1 */
#define PREPROC1_DCSUB_SEL_END                               (4U)
#define PREPROC1_DCSUB_SEL_START                            (4U)

/* Definition for the field DCEST_RESET_MODE in register PREPROC1 */
#define PREPROC1_DCEST_RESET_MODE_END                        (3U)
#define PREPROC1_DCEST_RESET_MODE_START                     (2U)

/* Definition for the field DCSUB_EN in register PREPROC1 */
#define PREPROC1_DCSUB_EN_END                                (1U)
#define PREPROC1_DCSUB_EN_START                             (1U)

/* Definition for the field INTF_MITG_LEFT_HYST_ORD in register PREPROC2 */
#define PREPROC2_INTF_MITG_LEFT_HYST_ORD_END                 (31U)
#define PREPROC2_INTF_MITG_LEFT_HYST_ORD_START              (28U)

/* Definition for the field INTF_MITG_RIGHT_HYST_ORD in register PREPROC2 */
#define PREPROC2_INTF_MITG_RIGHT_HYST_ORD_END                (27U)
#define PREPROC2_INTF_MITG_RIGHT_HYST_ORD_START             (24U)

/* Definition for the field TWIDINCR in register PREPROC2 */
#define PREPROC2_TWIDINCR_END                               (13U)
#define PREPROC2_TWIDINCR_START                             (0U)


/* Definition for the field SHUFFLE_INDX_START_OFFSET in register WRAPCOMB */
#define WRAPCOMB_SHUFFLE_INDX_START_OFFSET_END               (23U)
#define WRAPCOMB_SHUFFLE_INDX_START_OFFSET_START            (20U)

/* Definition for the field WRAP_COMB in register WRAPCOMB */
#define WRAPCOMB_WRAP_COMB_END                              (19U)
#define WRAPCOMB_WRAP_COMB_START                            (0U)

/* Definition for the field CMP_SCALEFAC in register CMPDCMP */
#define CMPDCMP_CMP_SCALEFAC_END                             (31U)
#define CMPDCMP_CMP_SCALEFAC_START                          (27U)

/* Definition for the field CMP_EGE_OPT_K_INDX in register CMPDCMP */
#define CMPDCMP_CMP_EGE_OPT_K_INDX_END                       (26U)
#define CMPDCMP_CMP_EGE_OPT_K_INDX_START                    (23U)

/* Definition for the field CMP_PASS_SEL in register CMPDCMP */
#define CMPDCMP_CMP_PASS_SEL_END                             (22U)
#define CMPDCMP_CMP_PASS_SEL_START                          (21U)

/* Definition for the field CMP_HEADER_EN in register CMPDCMP */
#define CMPDCMP_CMP_HEADER_EN_END                            (20U)
#define CMPDCMP_CMP_HEADER_EN_START                         (20U)

/* Definition for the field CMP_SCALEFAC_BW in register CMPDCMP */
#define CMPDCMP_CMP_SCALEFAC_BW_END                          (19U)
#define CMPDCMP_CMP_SCALEFAC_BW_START                       (16U)

/* Definition for the field CMP_BFP_MANTISSA_BW in register CMPDCMP */
#define CMPDCMP_CMP_BFP_MANTISSA_BW_END                      (15U)
#define CMPDCMP_CMP_BFP_MANTISSA_BW_START                   (11U)

/* Definition for the field CMP_EGE_K_ARR_LEN in register CMPDCMP */
#define CMPDCMP_CMP_EGE_K_ARR_LEN_END                       (10U)
#define CMPDCMP_CMP_EGE_K_ARR_LEN_START                     (7U)

/* Definition for the field CMP_METHOD in register CMPDCMP */
#define CMPDCMP_CMP_METHOD_END                               (6U)
#define CMPDCMP_CMP_METHOD_START                            (4U)

/* Definition for the field CMP_DCMP in register CMPDCMP */
#define CMPDCMP_CMP_DCMP_END                                 (3U)
#define CMPDCMP_CMP_DCMP_START                              (3U)

/* Definition for the field CMP_DITHER_EN in register CMPDCMP */
#define CMPDCMP_CMP_DITHER_EN_END                            (2U)
#define CMPDCMP_CMP_DITHER_EN_START                         (2U)

#if defined (SOC_AWR294X)
/* Below set of macros are applicable for AWR294x ES2.0 samples only */
/* Definition for the field REG_CMP_BFP_DECR_IMAG_BITW in register CMPDCMP2 */
#define CMPDCMP2_REG_CMP_BFP_DECR_IMAG_BITW_END              (4U)
#define CMPDCMP2_REG_CMP_BFP_DECR_IMAG_BITW_START            (4U)

/* Definition for the field REG_CMP_ROUND_EN in register CMPDCMP2 */
#define CMPDCMP2_REG_CMP_ROUND_EN_END                        (3U)
#define CMPDCMP2_REG_CMP_ROUND_EN_START                      (3U)

/* Definition for the field REG_CMP_SEL_LFSR in register CMPDCMP2 */
#define CMPDCMP2_REG_CMP_SEL_LFSR_END                        (2U)
#define CMPDCMP2_REG_CMP_SEL_LFSR_START                      (2U)
#endif

/* Definition for the field LM_DIMC_NONCYCLIC in register LOCALMAX */
#define LOCALMAX_LM_DIMC_NONCYCLIC_END                          (13U)
#define LOCALMAX_LM_DIMC_NONCYCLIC_START                       (13U)

/* Definition for the field LM_DIMB_NONCYCLIC in register LOCALMAX */
#define LOCALMAX_LM_DIMB_NONCYCLIC_END                          (12U)
#define LOCALMAX_LM_DIMB_NONCYCLIC_START                       (12U)

/* Definition for the field LM_THRESH_MODE in register LOCALMAX */
#define LOCALMAX_LM_THRESH_MODE_END                          (11U)
#define LOCALMAX_LM_THRESH_MODE_START                       (10U)

/* Definition for the field LM_THRESH_BITMASK in register LOCALMAX */
#define LOCALMAX_LM_THRESH_BITMASK_END                       (9U)
#define LOCALMAX_LM_THRESH_BITMASK_START                    (8U)

/* Definition for the field LM_NEIGH_BITMASK in register LOCALMAX */
#define LOCALMAX_LM_NEIGH_BITMASK_END                        (7U)
#define LOCALMAX_LM_NEIGH_BITMASK_START                     (0U)

/* *
 @struct DSSHWACCPARAMRegs
 * @brief
 *   Module DSS_HW_ACC_PARAM Register Definition
 * @details
 *   This structure is used to access the DSS_HW_ACC_PARAM module registers.
 */

typedef volatile struct DSSHWACCPARAMRegs_t
{
    uint32_t    HEADER;           /* Offset = 0x000 */
    uint32_t    SRC;              /* Offset = 0x004 */
    uint32_t    SRCA;             /* Offset = 0x008 */
    uint32_t    SRCB;             /* Offset = 0x00C */
    uint32_t    SRCC;             /* Offset = 0x010 */
    uint32_t    CIRCSHIFT;        /* Offset = 0x014 */
    uint32_t    CIRCSHIFT2;       /* Offset = 0x018 */
    uint32_t    DST;              /* Offset = 0x01C */
    uint32_t    DSTA;             /* Offset = 0x020 */
    uint32_t    DSTB;             /* Offset = 0x024 */
    uint32_t    RESERVED;          /* Offset = 0x028 */
    union
    {
        struct {
            uint32_t    CFAREN;            /* Offset = 0x02C */
            uint32_t    CFARCFG;           /* Offset = 0x030 */
            uint32_t    RESERVED2;         /* Offset = 0x034 */
            uint32_t    RESERVED3;         /* Offset = 0x038 */
            uint32_t    RESERVED4;         /* Offset = 0x03C */
        }CFARPATH;

        struct {
            uint32_t    BFLYFFT;           /* Offset = 0x02C */
            uint32_t    POSTPROCWIN;       /* Offset = 0x030 */
            uint32_t    PREPROC1;           /* Offset = 0x034 */
            uint32_t    PREPROC2;       /* Offset = 0x038 */
            uint32_t    WRAPCOMB;           /* Offset = 0x03C */
        }FFTPATH;

        struct
        {
            int32_t     CMPDCMP;           /* Offset = 0x02C */
#if defined (SOC_AWR294X)
            /* This field is reserved for AWR294x ES1.0 and applicable for only ES2.0 devices. */
            uint32_t    CMPDCMP2;          /* Offset = 0x030 */
#endif
#if defined (SOC_AM273X)
            uint32_t    RESERVED1;         /* Offset = 0x030 */
#endif
            uint32_t    RESERVED2;         /* Offset = 0x034 */
            uint32_t    RESERVED3;         /* Offset = 0x038 */
            uint32_t    RESERVED4;         /* Offset = 0x03C */

        }CMPDCMPPATH;

        struct
        {
            uint32_t    LOCALMAX;          /* Offset = 0x02C */
            uint32_t    RESERVED1;         /* Offset = 0x030 */
            uint32_t    RESERVED2;         /* Offset = 0x034 */
            uint32_t    RESERVED3;         /* Offset = 0x038 */
            uint32_t    WRAPCOMB;           /* Offset = 0x03C */

        }LOCALMAXPATH;
    }accelModeParam;

} DSSHWACCPARAMRegs;

#ifdef __cplusplus
}
#endif

#endif /* HW_HWA_PARAMSET_H_ */
