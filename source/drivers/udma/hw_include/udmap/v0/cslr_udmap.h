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
 *  Name        : cslr_udmap.h
*/
#ifndef CSLR_UDMAP_H_
#define CSLR_UDMAP_H_

#ifdef __cplusplus
extern "C"
{
#endif
#include <drivers/hw_include/cslr.h>
#include <stdint.h>

/**************************************************************************
* Hardware Region  : UDMA-P Control / Status Registers
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t REVISION;                  /* Revision Register */
    volatile uint32_t PERF_CTRL;                 /* Performance Control Register */
    volatile uint32_t EMU_CTRL;                  /* Emulation Control Register */
    volatile uint8_t  Resv_16[4];
    volatile uint32_t PSIL_TO;                   /* PSI-L Proxy Timeout Register */
    volatile uint8_t  Resv_28[8];
    volatile uint32_t UTC_CTRL;                  /* External UTC Control Register */
    volatile uint32_t CAP0;                      /* Capabilities Register 0 */
    volatile uint32_t CAP1;                      /* Capabilities Register 1 */
    volatile uint32_t CAP2;                      /* Capabilities Register 2 */
    volatile uint32_t CAP3;                      /* Capabilities Register 3 */
    volatile uint8_t  Resv_64[16];
    volatile uint32_t PERF0;                     /* mem0 Port Virtualization Tuning Register */
    volatile uint32_t PERF1;                     /* mem1 Port Virtualization Tuning Register */
    volatile uint32_t PERF2;                     /* memr Port Virtualization Tuning Register */
    volatile uint32_t PERF3;                     /* memw Port Virtualization Tuning Register */
    volatile uint8_t  Resv_96[16];
    volatile uint32_t PM0;                       /* Power Management Register 0 */
    volatile uint32_t PM1;                       /* Power Management Register 1 */
    volatile uint8_t  Resv_120[16];
    volatile uint32_t DBGADDR;                   /* Debug Address Register */
    volatile uint32_t DBGDATA;                   /* Debug Data Register */
    volatile uint32_t RFLOWFWOES;                /* Rx Flow ID Firewall Output Event Steering Register */
    volatile uint8_t  Resv_136[4];
    volatile uint32_t RFLOWFWSTAT;               /* Rx Flow ID Firewall Status Register 0 */
} CSL_udmap_gcfgRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_UDMAP_GCFG_REVISION                                                (0x00000000U)
#define CSL_UDMAP_GCFG_PERF_CTRL                                               (0x00000004U)
#define CSL_UDMAP_GCFG_EMU_CTRL                                                (0x00000008U)
#define CSL_UDMAP_GCFG_PSIL_TO                                                 (0x00000010U)
#define CSL_UDMAP_GCFG_UTC_CTRL                                                (0x0000001CU)
#define CSL_UDMAP_GCFG_CAP0                                                    (0x00000020U)
#define CSL_UDMAP_GCFG_CAP1                                                    (0x00000024U)
#define CSL_UDMAP_GCFG_CAP2                                                    (0x00000028U)
#define CSL_UDMAP_GCFG_CAP3                                                    (0x0000002CU)
#define CSL_UDMAP_GCFG_PERF0                                                   (0x00000040U)
#define CSL_UDMAP_GCFG_PERF1                                                   (0x00000044U)
#define CSL_UDMAP_GCFG_PERF2                                                   (0x00000048U)
#define CSL_UDMAP_GCFG_PERF3                                                   (0x0000004CU)
#define CSL_UDMAP_GCFG_PM0                                                     (0x00000060U)
#define CSL_UDMAP_GCFG_PM1                                                     (0x00000064U)
#define CSL_UDMAP_GCFG_DBGADDR                                                 (0x00000078U)
#define CSL_UDMAP_GCFG_DBGDATA                                                 (0x0000007CU)
#define CSL_UDMAP_GCFG_RFLOWFWOES                                              (0x00000080U)
#define CSL_UDMAP_GCFG_RFLOWFWSTAT                                             (0x00000088U)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* REVISION */

#define CSL_UDMAP_GCFG_REVISION_MODID_MASK                                     (0xFFFF0000U)
#define CSL_UDMAP_GCFG_REVISION_MODID_SHIFT                                    (0x00000010U)
#define CSL_UDMAP_GCFG_REVISION_MODID_MAX                                      (0x0000FFFFU)

#define CSL_UDMAP_GCFG_REVISION_REVRTL_MASK                                    (0x0000F800U)
#define CSL_UDMAP_GCFG_REVISION_REVRTL_SHIFT                                   (0x0000000BU)
#define CSL_UDMAP_GCFG_REVISION_REVRTL_MAX                                     (0x0000001FU)

#define CSL_UDMAP_GCFG_REVISION_REVMAJ_MASK                                    (0x00000700U)
#define CSL_UDMAP_GCFG_REVISION_REVMAJ_SHIFT                                   (0x00000008U)
#define CSL_UDMAP_GCFG_REVISION_REVMAJ_MAX                                     (0x00000007U)

#define CSL_UDMAP_GCFG_REVISION_CUSTOM_MASK                                    (0x000000C0U)
#define CSL_UDMAP_GCFG_REVISION_CUSTOM_SHIFT                                   (0x00000006U)
#define CSL_UDMAP_GCFG_REVISION_CUSTOM_MAX                                     (0x00000003U)

#define CSL_UDMAP_GCFG_REVISION_REVMIN_MASK                                    (0x0000003FU)
#define CSL_UDMAP_GCFG_REVISION_REVMIN_SHIFT                                   (0x00000000U)
#define CSL_UDMAP_GCFG_REVISION_REVMIN_MAX                                     (0x0000003FU)

/* PERF_CTRL */

#define CSL_UDMAP_GCFG_PERF_CTRL_TIMEOUT_CNT_MASK                              (0x0000FFFFU)
#define CSL_UDMAP_GCFG_PERF_CTRL_TIMEOUT_CNT_SHIFT                             (0x00000000U)
#define CSL_UDMAP_GCFG_PERF_CTRL_TIMEOUT_CNT_MAX                               (0x0000FFFFU)

/* EMU_CTRL */

#define CSL_UDMAP_GCFG_EMU_CTRL_SOFT_MASK                                      (0x00000002U)
#define CSL_UDMAP_GCFG_EMU_CTRL_SOFT_SHIFT                                     (0x00000001U)
#define CSL_UDMAP_GCFG_EMU_CTRL_SOFT_MAX                                       (0x00000001U)

#define CSL_UDMAP_GCFG_EMU_CTRL_FREE_MASK                                      (0x00000001U)
#define CSL_UDMAP_GCFG_EMU_CTRL_FREE_SHIFT                                     (0x00000000U)
#define CSL_UDMAP_GCFG_EMU_CTRL_FREE_MAX                                       (0x00000001U)

/* PSIL_TO */

#define CSL_UDMAP_GCFG_PSIL_TO_TOUT_MASK                                       (0x80000000U)
#define CSL_UDMAP_GCFG_PSIL_TO_TOUT_SHIFT                                      (0x0000001FU)
#define CSL_UDMAP_GCFG_PSIL_TO_TOUT_MAX                                        (0x00000001U)

#define CSL_UDMAP_GCFG_PSIL_TO_TOUT_CNT_MASK                                   (0x0000FFFFU)
#define CSL_UDMAP_GCFG_PSIL_TO_TOUT_CNT_SHIFT                                  (0x00000000U)
#define CSL_UDMAP_GCFG_PSIL_TO_TOUT_CNT_MAX                                    (0x0000FFFFU)

/* UTC_CTRL */

#define CSL_UDMAP_GCFG_UTC_CTRL_UTC_CHAN_START_MASK                            (0x0000FFFFU)
#define CSL_UDMAP_GCFG_UTC_CTRL_UTC_CHAN_START_SHIFT                           (0x00000000U)
#define CSL_UDMAP_GCFG_UTC_CTRL_UTC_CHAN_START_MAX                             (0x0000FFFFU)

/* CAP0 */

#define CSL_UDMAP_GCFG_CAP0_GLOBAL_TRIG_MASK                                   (0x00080000U)
#define CSL_UDMAP_GCFG_CAP0_GLOBAL_TRIG_SHIFT                                  (0x00000013U)
#define CSL_UDMAP_GCFG_CAP0_GLOBAL_TRIG_MAX                                    (0x00000001U)

#define CSL_UDMAP_GCFG_CAP0_LOCAL_TRIG_MASK                                    (0x00040000U)
#define CSL_UDMAP_GCFG_CAP0_LOCAL_TRIG_SHIFT                                   (0x00000012U)
#define CSL_UDMAP_GCFG_CAP0_LOCAL_TRIG_MAX                                     (0x00000001U)

#define CSL_UDMAP_GCFG_CAP0_EOL_MASK                                           (0x00020000U)
#define CSL_UDMAP_GCFG_CAP0_EOL_SHIFT                                          (0x00000011U)
#define CSL_UDMAP_GCFG_CAP0_EOL_MAX                                            (0x00000001U)

#define CSL_UDMAP_GCFG_CAP0_STATIC_MASK                                        (0x00010000U)
#define CSL_UDMAP_GCFG_CAP0_STATIC_SHIFT                                       (0x00000010U)
#define CSL_UDMAP_GCFG_CAP0_STATIC_MAX                                         (0x00000001U)

#define CSL_UDMAP_GCFG_CAP0_TYPE15_MASK                                        (0x00008000U)
#define CSL_UDMAP_GCFG_CAP0_TYPE15_SHIFT                                       (0x0000000FU)
#define CSL_UDMAP_GCFG_CAP0_TYPE15_MAX                                         (0x00000001U)

#define CSL_UDMAP_GCFG_CAP0_TYPE14_MASK                                        (0x00004000U)
#define CSL_UDMAP_GCFG_CAP0_TYPE14_SHIFT                                       (0x0000000EU)
#define CSL_UDMAP_GCFG_CAP0_TYPE14_MAX                                         (0x00000001U)

#define CSL_UDMAP_GCFG_CAP0_TYPE13_MASK                                        (0x00002000U)
#define CSL_UDMAP_GCFG_CAP0_TYPE13_SHIFT                                       (0x0000000DU)
#define CSL_UDMAP_GCFG_CAP0_TYPE13_MAX                                         (0x00000001U)

#define CSL_UDMAP_GCFG_CAP0_TYPE12_MASK                                        (0x00001000U)
#define CSL_UDMAP_GCFG_CAP0_TYPE12_SHIFT                                       (0x0000000CU)
#define CSL_UDMAP_GCFG_CAP0_TYPE12_MAX                                         (0x00000001U)

#define CSL_UDMAP_GCFG_CAP0_TYPE11_MASK                                        (0x00000800U)
#define CSL_UDMAP_GCFG_CAP0_TYPE11_SHIFT                                       (0x0000000BU)
#define CSL_UDMAP_GCFG_CAP0_TYPE11_MAX                                         (0x00000001U)

#define CSL_UDMAP_GCFG_CAP0_TYPE10_MASK                                        (0x00000400U)
#define CSL_UDMAP_GCFG_CAP0_TYPE10_SHIFT                                       (0x0000000AU)
#define CSL_UDMAP_GCFG_CAP0_TYPE10_MAX                                         (0x00000001U)

#define CSL_UDMAP_GCFG_CAP0_TYPE9_MASK                                         (0x00000200U)
#define CSL_UDMAP_GCFG_CAP0_TYPE9_SHIFT                                        (0x00000009U)
#define CSL_UDMAP_GCFG_CAP0_TYPE9_MAX                                          (0x00000001U)

#define CSL_UDMAP_GCFG_CAP0_TYPE8_MASK                                         (0x00000100U)
#define CSL_UDMAP_GCFG_CAP0_TYPE8_SHIFT                                        (0x00000008U)
#define CSL_UDMAP_GCFG_CAP0_TYPE8_MAX                                          (0x00000001U)

#define CSL_UDMAP_GCFG_CAP0_TYPE7_MASK                                         (0x00000080U)
#define CSL_UDMAP_GCFG_CAP0_TYPE7_SHIFT                                        (0x00000007U)
#define CSL_UDMAP_GCFG_CAP0_TYPE7_MAX                                          (0x00000001U)

#define CSL_UDMAP_GCFG_CAP0_TYPE6_MASK                                         (0x00000040U)
#define CSL_UDMAP_GCFG_CAP0_TYPE6_SHIFT                                        (0x00000006U)
#define CSL_UDMAP_GCFG_CAP0_TYPE6_MAX                                          (0x00000001U)

#define CSL_UDMAP_GCFG_CAP0_TYPE5_MASK                                         (0x00000020U)
#define CSL_UDMAP_GCFG_CAP0_TYPE5_SHIFT                                        (0x00000005U)
#define CSL_UDMAP_GCFG_CAP0_TYPE5_MAX                                          (0x00000001U)

#define CSL_UDMAP_GCFG_CAP0_TYPE4_MASK                                         (0x00000010U)
#define CSL_UDMAP_GCFG_CAP0_TYPE4_SHIFT                                        (0x00000004U)
#define CSL_UDMAP_GCFG_CAP0_TYPE4_MAX                                          (0x00000001U)

#define CSL_UDMAP_GCFG_CAP0_TYPE3_MASK                                         (0x00000008U)
#define CSL_UDMAP_GCFG_CAP0_TYPE3_SHIFT                                        (0x00000003U)
#define CSL_UDMAP_GCFG_CAP0_TYPE3_MAX                                          (0x00000001U)

#define CSL_UDMAP_GCFG_CAP0_TYPE2_MASK                                         (0x00000004U)
#define CSL_UDMAP_GCFG_CAP0_TYPE2_SHIFT                                        (0x00000002U)
#define CSL_UDMAP_GCFG_CAP0_TYPE2_MAX                                          (0x00000001U)

#define CSL_UDMAP_GCFG_CAP0_TYPE1_MASK                                         (0x00000002U)
#define CSL_UDMAP_GCFG_CAP0_TYPE1_SHIFT                                        (0x00000001U)
#define CSL_UDMAP_GCFG_CAP0_TYPE1_MAX                                          (0x00000001U)

#define CSL_UDMAP_GCFG_CAP0_TYPE0_MASK                                         (0x00000001U)
#define CSL_UDMAP_GCFG_CAP0_TYPE0_SHIFT                                        (0x00000000U)
#define CSL_UDMAP_GCFG_CAP0_TYPE0_MAX                                          (0x00000001U)

/* CAP1 */

#define CSL_UDMAP_GCFG_CAP1_SECTR_MASK                                         (0x00000008U)
#define CSL_UDMAP_GCFG_CAP1_SECTR_SHIFT                                        (0x00000003U)
#define CSL_UDMAP_GCFG_CAP1_SECTR_MAX                                          (0x00000001U)

#define CSL_UDMAP_GCFG_CAP1_DFMT_MASK                                          (0x00000004U)
#define CSL_UDMAP_GCFG_CAP1_DFMT_SHIFT                                         (0x00000002U)
#define CSL_UDMAP_GCFG_CAP1_DFMT_MAX                                           (0x00000001U)

#define CSL_UDMAP_GCFG_CAP1_ELTYPE_MASK                                        (0x00000002U)
#define CSL_UDMAP_GCFG_CAP1_ELTYPE_SHIFT                                       (0x00000001U)
#define CSL_UDMAP_GCFG_CAP1_ELTYPE_MAX                                         (0x00000001U)

#define CSL_UDMAP_GCFG_CAP1_AMODE_MASK                                         (0x00000001U)
#define CSL_UDMAP_GCFG_CAP1_AMODE_SHIFT                                        (0x00000000U)
#define CSL_UDMAP_GCFG_CAP1_AMODE_MAX                                          (0x00000001U)

/* CAP2 */

#define CSL_UDMAP_GCFG_CAP2_RCHAN_CNT_MASK                                     (0x07FC0000U)
#define CSL_UDMAP_GCFG_CAP2_RCHAN_CNT_SHIFT                                    (0x00000012U)
#define CSL_UDMAP_GCFG_CAP2_RCHAN_CNT_MAX                                      (0x000001FFU)

#define CSL_UDMAP_GCFG_CAP2_ECHAN_CNT_MASK                                     (0x0003FE00U)
#define CSL_UDMAP_GCFG_CAP2_ECHAN_CNT_SHIFT                                    (0x00000009U)
#define CSL_UDMAP_GCFG_CAP2_ECHAN_CNT_MAX                                      (0x000001FFU)

#define CSL_UDMAP_GCFG_CAP2_TCHAN_CNT_MASK                                     (0x000001FFU)
#define CSL_UDMAP_GCFG_CAP2_TCHAN_CNT_SHIFT                                    (0x00000000U)
#define CSL_UDMAP_GCFG_CAP2_TCHAN_CNT_MAX                                      (0x000001FFU)

/* CAP3 */

#define CSL_UDMAP_GCFG_CAP3_UCHAN_CNT_MASK                                     (0xFF800000U)
#define CSL_UDMAP_GCFG_CAP3_UCHAN_CNT_SHIFT                                    (0x00000017U)
#define CSL_UDMAP_GCFG_CAP3_UCHAN_CNT_MAX                                      (0x000001FFU)

#define CSL_UDMAP_GCFG_CAP3_HCHAN_CNT_MASK                                     (0x007FC000U)
#define CSL_UDMAP_GCFG_CAP3_HCHAN_CNT_SHIFT                                    (0x0000000EU)
#define CSL_UDMAP_GCFG_CAP3_HCHAN_CNT_MAX                                      (0x000001FFU)

#define CSL_UDMAP_GCFG_CAP3_RFLOW_CNT_MASK                                     (0x00003FFFU)
#define CSL_UDMAP_GCFG_CAP3_RFLOW_CNT_SHIFT                                    (0x00000000U)
#define CSL_UDMAP_GCFG_CAP3_RFLOW_CNT_MAX                                      (0x00003FFFU)

/* PERF0 */

#define CSL_UDMAP_GCFG_PERF0_VRD_THRESH0_MASK                                  (0x00FF0000U)
#define CSL_UDMAP_GCFG_PERF0_VRD_THRESH0_SHIFT                                 (0x00000010U)
#define CSL_UDMAP_GCFG_PERF0_VRD_THRESH0_MAX                                   (0x000000FFU)

#define CSL_UDMAP_GCFG_PERF0_VWR_THRESH0_MASK                                  (0x000000FFU)
#define CSL_UDMAP_GCFG_PERF0_VWR_THRESH0_SHIFT                                 (0x00000000U)
#define CSL_UDMAP_GCFG_PERF0_VWR_THRESH0_MAX                                   (0x000000FFU)

/* PERF1 */

#define CSL_UDMAP_GCFG_PERF1_VRD_THRESH1_MASK                                  (0x00FF0000U)
#define CSL_UDMAP_GCFG_PERF1_VRD_THRESH1_SHIFT                                 (0x00000010U)
#define CSL_UDMAP_GCFG_PERF1_VRD_THRESH1_MAX                                   (0x000000FFU)

#define CSL_UDMAP_GCFG_PERF1_VWR_THRESH1_MASK                                  (0x000000FFU)
#define CSL_UDMAP_GCFG_PERF1_VWR_THRESH1_SHIFT                                 (0x00000000U)
#define CSL_UDMAP_GCFG_PERF1_VWR_THRESH1_MAX                                   (0x000000FFU)

/* PERF2 */

#define CSL_UDMAP_GCFG_PERF2_VRD_THRESH2_MASK                                  (0x00FF0000U)
#define CSL_UDMAP_GCFG_PERF2_VRD_THRESH2_SHIFT                                 (0x00000010U)
#define CSL_UDMAP_GCFG_PERF2_VRD_THRESH2_MAX                                   (0x000000FFU)

/* PERF3 */

#define CSL_UDMAP_GCFG_PERF3_VWR_THRESH3_MASK                                  (0x000000FFU)
#define CSL_UDMAP_GCFG_PERF3_VWR_THRESH3_SHIFT                                 (0x00000000U)
#define CSL_UDMAP_GCFG_PERF3_VWR_THRESH3_MAX                                   (0x000000FFU)

/* PM0 */

#define CSL_UDMAP_GCFG_PM0_NOGATE_LO_MASK                                      (0xFFFFFFFFU)
#define CSL_UDMAP_GCFG_PM0_NOGATE_LO_SHIFT                                     (0x00000000U)
#define CSL_UDMAP_GCFG_PM0_NOGATE_LO_MAX                                       (0xFFFFFFFFU)

/* PM1 */

#define CSL_UDMAP_GCFG_PM1_NOGATE_HI_MASK                                      (0xFFFFFFFFU)
#define CSL_UDMAP_GCFG_PM1_NOGATE_HI_SHIFT                                     (0x00000000U)
#define CSL_UDMAP_GCFG_PM1_NOGATE_HI_MAX                                       (0xFFFFFFFFU)

/* DBGADDR */

#define CSL_UDMAP_GCFG_DBGADDR_DBG_EN_MASK                                     (0x80000000U)
#define CSL_UDMAP_GCFG_DBGADDR_DBG_EN_SHIFT                                    (0x0000001FU)
#define CSL_UDMAP_GCFG_DBGADDR_DBG_EN_MAX                                      (0x00000001U)

#define CSL_UDMAP_GCFG_DBGADDR_DBG_UNIT_MASK                                   (0x0000FF00U)
#define CSL_UDMAP_GCFG_DBGADDR_DBG_UNIT_SHIFT                                  (0x00000008U)
#define CSL_UDMAP_GCFG_DBGADDR_DBG_UNIT_MAX                                    (0x000000FFU)

#define CSL_UDMAP_GCFG_DBGADDR_DBG_ADDR_MASK                                   (0x000000FFU)
#define CSL_UDMAP_GCFG_DBGADDR_DBG_ADDR_SHIFT                                  (0x00000000U)
#define CSL_UDMAP_GCFG_DBGADDR_DBG_ADDR_MAX                                    (0x000000FFU)

/* DBGDATA */

#define CSL_UDMAP_GCFG_DBGDATA_DBG_DATA_MASK                                   (0xFFFFFFFFU)
#define CSL_UDMAP_GCFG_DBGDATA_DBG_DATA_SHIFT                                  (0x00000000U)
#define CSL_UDMAP_GCFG_DBGDATA_DBG_DATA_MAX                                    (0xFFFFFFFFU)

/* RFLOWFWOES */

#define CSL_UDMAP_GCFG_RFLOWFWOES_EVT_NUM_MASK                                 (0x0000FFFFU)
#define CSL_UDMAP_GCFG_RFLOWFWOES_EVT_NUM_SHIFT                                (0x00000000U)
#define CSL_UDMAP_GCFG_RFLOWFWOES_EVT_NUM_MAX                                  (0x0000FFFFU)

/* RFLOWFWSTAT */

#define CSL_UDMAP_GCFG_RFLOWFWSTAT_PEND_MASK                                   (0x80000000U)
#define CSL_UDMAP_GCFG_RFLOWFWSTAT_PEND_SHIFT                                  (0x0000001FU)
#define CSL_UDMAP_GCFG_RFLOWFWSTAT_PEND_MAX                                    (0x00000001U)

#define CSL_UDMAP_GCFG_RFLOWFWSTAT_FLOWID_MASK                                 (0x3FFF0000U)
#define CSL_UDMAP_GCFG_RFLOWFWSTAT_FLOWID_SHIFT                                (0x00000010U)
#define CSL_UDMAP_GCFG_RFLOWFWSTAT_FLOWID_MAX                                  (0x00003FFFU)

#define CSL_UDMAP_GCFG_RFLOWFWSTAT_CHANNEL_MASK                                (0x000001FFU)
#define CSL_UDMAP_GCFG_RFLOWFWSTAT_CHANNEL_SHIFT                               (0x00000000U)
#define CSL_UDMAP_GCFG_RFLOWFWSTAT_CHANNEL_MAX                                 (0x000001FFU)

/**************************************************************************
* Hardware Region  : UDMA-P Rx Flow Table Registers
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t RFA;                       /* Rx Flow Config Register A */
    volatile uint32_t RFB;                       /* Rx Flow Config Register B */
    volatile uint32_t RFC;                       /* Rx Flow Config Register C */
    volatile uint32_t RFD;                       /* Rx Flow Config Register D */
    volatile uint32_t RFE;                       /* Rx Flow Config Register E */
    volatile uint32_t RFF;                       /* Rx Flow Config Register F */
    volatile uint32_t RFG;                       /* Rx Flow Config Register G */
    volatile uint32_t RFH;                       /* Rx Flow Config Register H */
    volatile uint8_t  Resv_64[32];
} CSL_udmap_rxfcfgRegs_flow;


typedef struct {
    CSL_udmap_rxfcfgRegs_flow FLOW[4096];
} CSL_udmap_rxfcfgRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_UDMAP_RXFCFG_FLOW_RFA(FLOW)                                        (0x00000000U+((FLOW)*0x40U))
#define CSL_UDMAP_RXFCFG_FLOW_RFB(FLOW)                                        (0x00000004U+((FLOW)*0x40U))
#define CSL_UDMAP_RXFCFG_FLOW_RFC(FLOW)                                        (0x00000008U+((FLOW)*0x40U))
#define CSL_UDMAP_RXFCFG_FLOW_RFD(FLOW)                                        (0x0000000CU+((FLOW)*0x40U))
#define CSL_UDMAP_RXFCFG_FLOW_RFE(FLOW)                                        (0x00000010U+((FLOW)*0x40U))
#define CSL_UDMAP_RXFCFG_FLOW_RFF(FLOW)                                        (0x00000014U+((FLOW)*0x40U))
#define CSL_UDMAP_RXFCFG_FLOW_RFG(FLOW)                                        (0x00000018U+((FLOW)*0x40U))
#define CSL_UDMAP_RXFCFG_FLOW_RFH(FLOW)                                        (0x0000001CU+((FLOW)*0x40U))

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* RFA */

#define CSL_UDMAP_RXFCFG_FLOW_RFA_EINFO_MASK                                   (0x40000000U)
#define CSL_UDMAP_RXFCFG_FLOW_RFA_EINFO_SHIFT                                  (0x0000001EU)
#define CSL_UDMAP_RXFCFG_FLOW_RFA_EINFO_MAX                                    (0x00000001U)

#define CSL_UDMAP_RXFCFG_FLOW_RFA_PSINFO_MASK                                  (0x20000000U)
#define CSL_UDMAP_RXFCFG_FLOW_RFA_PSINFO_SHIFT                                 (0x0000001DU)
#define CSL_UDMAP_RXFCFG_FLOW_RFA_PSINFO_MAX                                   (0x00000001U)

#define CSL_UDMAP_RXFCFG_FLOW_RFA_ERR_HANDLING_MASK                            (0x10000000U)
#define CSL_UDMAP_RXFCFG_FLOW_RFA_ERR_HANDLING_SHIFT                           (0x0000001CU)
#define CSL_UDMAP_RXFCFG_FLOW_RFA_ERR_HANDLING_MAX                             (0x00000001U)

#define CSL_UDMAP_RXFCFG_FLOW_RFA_DESC_TYPE_MASK                               (0x0C000000U)
#define CSL_UDMAP_RXFCFG_FLOW_RFA_DESC_TYPE_SHIFT                              (0x0000001AU)
#define CSL_UDMAP_RXFCFG_FLOW_RFA_DESC_TYPE_MAX                                (0x00000003U)

#define CSL_UDMAP_RXFCFG_FLOW_RFA_PS_LOC_MASK                                  (0x02000000U)
#define CSL_UDMAP_RXFCFG_FLOW_RFA_PS_LOC_SHIFT                                 (0x00000019U)
#define CSL_UDMAP_RXFCFG_FLOW_RFA_PS_LOC_MAX                                   (0x00000001U)

#define CSL_UDMAP_RXFCFG_FLOW_RFA_SOP_OFF_MASK                                 (0x01FF0000U)
#define CSL_UDMAP_RXFCFG_FLOW_RFA_SOP_OFF_SHIFT                                (0x00000010U)
#define CSL_UDMAP_RXFCFG_FLOW_RFA_SOP_OFF_MAX                                  (0x000001FFU)

#define CSL_UDMAP_RXFCFG_FLOW_RFA_DEST_QNUM_MASK                               (0x0000FFFFU)
#define CSL_UDMAP_RXFCFG_FLOW_RFA_DEST_QNUM_SHIFT                              (0x00000000U)
#define CSL_UDMAP_RXFCFG_FLOW_RFA_DEST_QNUM_MAX                                (0x0000FFFFU)

/* RFB */

#define CSL_UDMAP_RXFCFG_FLOW_RFB_SRCTAG_HI_MASK                               (0xFF000000U)
#define CSL_UDMAP_RXFCFG_FLOW_RFB_SRCTAG_HI_SHIFT                              (0x00000018U)
#define CSL_UDMAP_RXFCFG_FLOW_RFB_SRCTAG_HI_MAX                                (0x000000FFU)

#define CSL_UDMAP_RXFCFG_FLOW_RFB_SRCTAG_LO_MASK                               (0x00FF0000U)
#define CSL_UDMAP_RXFCFG_FLOW_RFB_SRCTAG_LO_SHIFT                              (0x00000010U)
#define CSL_UDMAP_RXFCFG_FLOW_RFB_SRCTAG_LO_MAX                                (0x000000FFU)

#define CSL_UDMAP_RXFCFG_FLOW_RFB_DSTTAG_HI_MASK                               (0x0000FF00U)
#define CSL_UDMAP_RXFCFG_FLOW_RFB_DSTTAG_HI_SHIFT                              (0x00000008U)
#define CSL_UDMAP_RXFCFG_FLOW_RFB_DSTTAG_HI_MAX                                (0x000000FFU)

#define CSL_UDMAP_RXFCFG_FLOW_RFB_DSTTAG_LO_MASK                               (0x000000FFU)
#define CSL_UDMAP_RXFCFG_FLOW_RFB_DSTTAG_LO_SHIFT                              (0x00000000U)
#define CSL_UDMAP_RXFCFG_FLOW_RFB_DSTTAG_LO_MAX                                (0x000000FFU)

/* RFC */

#define CSL_UDMAP_RXFCFG_FLOW_RFC_SRCTAG_HI_SEL_MASK                           (0x70000000U)
#define CSL_UDMAP_RXFCFG_FLOW_RFC_SRCTAG_HI_SEL_SHIFT                          (0x0000001CU)
#define CSL_UDMAP_RXFCFG_FLOW_RFC_SRCTAG_HI_SEL_MAX                            (0x00000007U)

#define CSL_UDMAP_RXFCFG_FLOW_RFC_SRCTAG_LO_SEL_MASK                           (0x07000000U)
#define CSL_UDMAP_RXFCFG_FLOW_RFC_SRCTAG_LO_SEL_SHIFT                          (0x00000018U)
#define CSL_UDMAP_RXFCFG_FLOW_RFC_SRCTAG_LO_SEL_MAX                            (0x00000007U)

#define CSL_UDMAP_RXFCFG_FLOW_RFC_DSTTAG_HI_SEL_MASK                           (0x00700000U)
#define CSL_UDMAP_RXFCFG_FLOW_RFC_DSTTAG_HI_SEL_SHIFT                          (0x00000014U)
#define CSL_UDMAP_RXFCFG_FLOW_RFC_DSTTAG_HI_SEL_MAX                            (0x00000007U)

#define CSL_UDMAP_RXFCFG_FLOW_RFC_DSTTAG_LO_SEL_MASK                           (0x00070000U)
#define CSL_UDMAP_RXFCFG_FLOW_RFC_DSTTAG_LO_SEL_SHIFT                          (0x00000010U)
#define CSL_UDMAP_RXFCFG_FLOW_RFC_DSTTAG_LO_SEL_MAX                            (0x00000007U)

#define CSL_UDMAP_RXFCFG_FLOW_RFC_SIZE_THRESH_EN_MASK                          (0x00000007U)
#define CSL_UDMAP_RXFCFG_FLOW_RFC_SIZE_THRESH_EN_SHIFT                         (0x00000000U)
#define CSL_UDMAP_RXFCFG_FLOW_RFC_SIZE_THRESH_EN_MAX                           (0x00000007U)

/* RFD */

#define CSL_UDMAP_RXFCFG_FLOW_RFD_FDQ0_SZ0_QNUM_MASK                           (0xFFFF0000U)
#define CSL_UDMAP_RXFCFG_FLOW_RFD_FDQ0_SZ0_QNUM_SHIFT                          (0x00000010U)
#define CSL_UDMAP_RXFCFG_FLOW_RFD_FDQ0_SZ0_QNUM_MAX                            (0x0000FFFFU)

#define CSL_UDMAP_RXFCFG_FLOW_RFD_FDQ1_QNUM_MASK                               (0x0000FFFFU)
#define CSL_UDMAP_RXFCFG_FLOW_RFD_FDQ1_QNUM_SHIFT                              (0x00000000U)
#define CSL_UDMAP_RXFCFG_FLOW_RFD_FDQ1_QNUM_MAX                                (0x0000FFFFU)

/* RFE */

#define CSL_UDMAP_RXFCFG_FLOW_RFE_FDQ2_QNUM_MASK                               (0xFFFF0000U)
#define CSL_UDMAP_RXFCFG_FLOW_RFE_FDQ2_QNUM_SHIFT                              (0x00000010U)
#define CSL_UDMAP_RXFCFG_FLOW_RFE_FDQ2_QNUM_MAX                                (0x0000FFFFU)

#define CSL_UDMAP_RXFCFG_FLOW_RFE_FDQ3_QNUM_MASK                               (0x0000FFFFU)
#define CSL_UDMAP_RXFCFG_FLOW_RFE_FDQ3_QNUM_SHIFT                              (0x00000000U)
#define CSL_UDMAP_RXFCFG_FLOW_RFE_FDQ3_QNUM_MAX                                (0x0000FFFFU)

/* RFF */

#define CSL_UDMAP_RXFCFG_FLOW_RFF_SIZE_THRESH0_MASK                            (0xFFFF0000U)
#define CSL_UDMAP_RXFCFG_FLOW_RFF_SIZE_THRESH0_SHIFT                           (0x00000010U)
#define CSL_UDMAP_RXFCFG_FLOW_RFF_SIZE_THRESH0_MAX                             (0x0000FFFFU)

#define CSL_UDMAP_RXFCFG_FLOW_RFF_SIZE_THRESH1_MASK                            (0x0000FFFFU)
#define CSL_UDMAP_RXFCFG_FLOW_RFF_SIZE_THRESH1_SHIFT                           (0x00000000U)
#define CSL_UDMAP_RXFCFG_FLOW_RFF_SIZE_THRESH1_MAX                             (0x0000FFFFU)

/* RFG */

#define CSL_UDMAP_RXFCFG_FLOW_RFG_SIZE_THRESH2_MASK                            (0xFFFF0000U)
#define CSL_UDMAP_RXFCFG_FLOW_RFG_SIZE_THRESH2_SHIFT                           (0x00000010U)
#define CSL_UDMAP_RXFCFG_FLOW_RFG_SIZE_THRESH2_MAX                             (0x0000FFFFU)

#define CSL_UDMAP_RXFCFG_FLOW_RFG_FDQ0_SZ1_QNUM_MASK                           (0x0000FFFFU)
#define CSL_UDMAP_RXFCFG_FLOW_RFG_FDQ0_SZ1_QNUM_SHIFT                          (0x00000000U)
#define CSL_UDMAP_RXFCFG_FLOW_RFG_FDQ0_SZ1_QNUM_MAX                            (0x0000FFFFU)

/* RFH */

#define CSL_UDMAP_RXFCFG_FLOW_RFH_FDQ0_SZ2_QNUM_MASK                           (0xFFFF0000U)
#define CSL_UDMAP_RXFCFG_FLOW_RFH_FDQ0_SZ2_QNUM_SHIFT                          (0x00000010U)
#define CSL_UDMAP_RXFCFG_FLOW_RFH_FDQ0_SZ2_QNUM_MAX                            (0x0000FFFFU)

#define CSL_UDMAP_RXFCFG_FLOW_RFH_FDQ0_SZ3_QNUM_MASK                           (0x0000FFFFU)
#define CSL_UDMAP_RXFCFG_FLOW_RFH_FDQ0_SZ3_QNUM_SHIFT                          (0x00000000U)
#define CSL_UDMAP_RXFCFG_FLOW_RFH_FDQ0_SZ3_QNUM_MAX                            (0x0000FFFFU)

/**************************************************************************
* Hardware Region  : UDMA-P Tx Channel Configuration Registers
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t TCFG;                      /* Tx Channel Configuration Register */
    volatile uint32_t TCREDIT;                   /* Tx Channel Transfer Request Credit Register */
    volatile uint8_t  Resv_20[12];
    volatile uint32_t TCQ;                       /* Tx Channel Completion Queue Register */
    volatile uint8_t  Resv_32[8];
    volatile uint32_t TOES[1];                   /* Tx Channel Output Event Steering Register 0 */
    volatile uint8_t  Resv_96[60];
    volatile uint32_t TEOES[1];                  /* Tx Channel Error Output Event Steering Register 0 */
    volatile uint32_t TPRI_CTRL;                 /* Tx Channel Priority Control Register */
    volatile uint32_t THREAD;                    /* Tx Channel Destination ThreadID Mapping Register */
    volatile uint8_t  Resv_112[4];
    volatile uint32_t TFIFO_DEPTH;               /* Tx Channel FIFO Depth Register */
    volatile uint8_t  Resv_128[12];
    volatile uint32_t TST_SCHED;                 /* Tx Channel Static Scheduler Config Register */
    volatile uint8_t  Resv_256[124];
} CSL_udmap_txccfgRegs_chan;


typedef struct {
    CSL_udmap_txccfgRegs_chan CHAN[1024];
} CSL_udmap_txccfgRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_UDMAP_TXCCFG_CHAN_TCFG(CHAN)                                       (0x00000000U+((CHAN)*0x100U))
#define CSL_UDMAP_TXCCFG_CHAN_TCREDIT(CHAN)                                    (0x00000004U+((CHAN)*0x100U))
#define CSL_UDMAP_TXCCFG_CHAN_TCQ(CHAN)                                        (0x00000014U+((CHAN)*0x100U))
#define CSL_UDMAP_TXCCFG_CHAN_TOES(CHAN,TOES)                                  (0x00000020U+((CHAN)*0x100U)+((TOES)*0x4U))
#define CSL_UDMAP_TXCCFG_CHAN_TEOES(CHAN,TEOES)                                (0x00000060U+((CHAN)*0x100U)+((TEOES)*0x4U))
#define CSL_UDMAP_TXCCFG_CHAN_TPRI_CTRL(CHAN)                                  (0x00000064U+((CHAN)*0x100U))
#define CSL_UDMAP_TXCCFG_CHAN_THREAD(CHAN)                                     (0x00000068U+((CHAN)*0x100U))
#define CSL_UDMAP_TXCCFG_CHAN_TFIFO_DEPTH(CHAN)                                (0x00000070U+((CHAN)*0x100U))
#define CSL_UDMAP_TXCCFG_CHAN_TST_SCHED(CHAN)                                  (0x00000080U+((CHAN)*0x100U))

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* TCFG */

#define CSL_UDMAP_TXCCFG_CHAN_TCFG_PAUSE_ON_ERR_MASK                           (0x80000000U)
#define CSL_UDMAP_TXCCFG_CHAN_TCFG_PAUSE_ON_ERR_SHIFT                          (0x0000001FU)
#define CSL_UDMAP_TXCCFG_CHAN_TCFG_PAUSE_ON_ERR_MAX                            (0x00000001U)

#define CSL_UDMAP_TXCCFG_CHAN_TCFG_FILT_EINFO_MASK                             (0x40000000U)
#define CSL_UDMAP_TXCCFG_CHAN_TCFG_FILT_EINFO_SHIFT                            (0x0000001EU)
#define CSL_UDMAP_TXCCFG_CHAN_TCFG_FILT_EINFO_MAX                              (0x00000001U)

#define CSL_UDMAP_TXCCFG_CHAN_TCFG_FILT_PSWORDS_MASK                           (0x20000000U)
#define CSL_UDMAP_TXCCFG_CHAN_TCFG_FILT_PSWORDS_SHIFT                          (0x0000001DU)
#define CSL_UDMAP_TXCCFG_CHAN_TCFG_FILT_PSWORDS_MAX                            (0x00000001U)

#define CSL_UDMAP_TXCCFG_CHAN_TCFG_ATYPE_MASK                                  (0x03000000U)
#define CSL_UDMAP_TXCCFG_CHAN_TCFG_ATYPE_SHIFT                                 (0x00000018U)
#define CSL_UDMAP_TXCCFG_CHAN_TCFG_ATYPE_MAX                                   (0x00000003U)

#define CSL_UDMAP_TXCCFG_CHAN_TCFG_CHAN_TYPE_MASK                              (0x000F0000U)
#define CSL_UDMAP_TXCCFG_CHAN_TCFG_CHAN_TYPE_SHIFT                             (0x00000010U)
#define CSL_UDMAP_TXCCFG_CHAN_TCFG_CHAN_TYPE_MAX                               (0x0000000FU)

#define CSL_UDMAP_TXCCFG_CHAN_TCFG_BURST_SIZE_MASK                             (0x00000C00U)
#define CSL_UDMAP_TXCCFG_CHAN_TCFG_BURST_SIZE_SHIFT                            (0x0000000AU)
#define CSL_UDMAP_TXCCFG_CHAN_TCFG_BURST_SIZE_MAX                              (0x00000003U)

#define CSL_UDMAP_TXCCFG_CHAN_TCFG_TDTYPE_MASK                                 (0x00000200U)
#define CSL_UDMAP_TXCCFG_CHAN_TCFG_TDTYPE_SHIFT                                (0x00000009U)
#define CSL_UDMAP_TXCCFG_CHAN_TCFG_TDTYPE_MAX                                  (0x00000001U)

#define CSL_UDMAP_TXCCFG_CHAN_TCFG_NOTDPKT_MASK                                (0x00000100U)
#define CSL_UDMAP_TXCCFG_CHAN_TCFG_NOTDPKT_SHIFT                               (0x00000008U)
#define CSL_UDMAP_TXCCFG_CHAN_TCFG_NOTDPKT_MAX                                 (0x00000001U)

#define CSL_UDMAP_TXCCFG_CHAN_TCFG_FETCH_SIZE_MASK                             (0x0000007FU)
#define CSL_UDMAP_TXCCFG_CHAN_TCFG_FETCH_SIZE_SHIFT                            (0x00000000U)
#define CSL_UDMAP_TXCCFG_CHAN_TCFG_FETCH_SIZE_MAX                              (0x0000007FU)

/* TCREDIT */

#define CSL_UDMAP_TXCCFG_CHAN_TCREDIT_COUNT_MASK                               (0x00000007U)
#define CSL_UDMAP_TXCCFG_CHAN_TCREDIT_COUNT_SHIFT                              (0x00000000U)
#define CSL_UDMAP_TXCCFG_CHAN_TCREDIT_COUNT_MAX                                (0x00000007U)

/* TCQ */

#define CSL_UDMAP_TXCCFG_CHAN_TCQ_TXCQ_QNUM_MASK                               (0x0000FFFFU)
#define CSL_UDMAP_TXCCFG_CHAN_TCQ_TXCQ_QNUM_SHIFT                              (0x00000000U)
#define CSL_UDMAP_TXCCFG_CHAN_TCQ_TXCQ_QNUM_MAX                                (0x0000FFFFU)

/* TOES */

#define CSL_UDMAP_TXCCFG_CHAN_TOES_EVT_NUM_MASK                                (0x0000FFFFU)
#define CSL_UDMAP_TXCCFG_CHAN_TOES_EVT_NUM_SHIFT                               (0x00000000U)
#define CSL_UDMAP_TXCCFG_CHAN_TOES_EVT_NUM_MAX                                 (0x0000FFFFU)

/* TEOES */

#define CSL_UDMAP_TXCCFG_CHAN_TEOES_EVT_NUM_MASK                               (0x0000FFFFU)
#define CSL_UDMAP_TXCCFG_CHAN_TEOES_EVT_NUM_SHIFT                              (0x00000000U)
#define CSL_UDMAP_TXCCFG_CHAN_TEOES_EVT_NUM_MAX                                (0x0000FFFFU)

/* TPRI_CTRL */

#define CSL_UDMAP_TXCCFG_CHAN_TPRI_CTRL_PRIORITY_MASK                          (0x70000000U)
#define CSL_UDMAP_TXCCFG_CHAN_TPRI_CTRL_PRIORITY_SHIFT                         (0x0000001CU)
#define CSL_UDMAP_TXCCFG_CHAN_TPRI_CTRL_PRIORITY_MAX                           (0x00000007U)

#define CSL_UDMAP_TXCCFG_CHAN_TPRI_CTRL_QOS_MASK                               (0x00070000U)
#define CSL_UDMAP_TXCCFG_CHAN_TPRI_CTRL_QOS_SHIFT                              (0x00000010U)
#define CSL_UDMAP_TXCCFG_CHAN_TPRI_CTRL_QOS_MAX                                (0x00000007U)

#define CSL_UDMAP_TXCCFG_CHAN_TPRI_CTRL_ORDERID_MASK                           (0x0000000FU)
#define CSL_UDMAP_TXCCFG_CHAN_TPRI_CTRL_ORDERID_SHIFT                          (0x00000000U)
#define CSL_UDMAP_TXCCFG_CHAN_TPRI_CTRL_ORDERID_MAX                            (0x0000000FU)

/* THREAD */

#define CSL_UDMAP_TXCCFG_CHAN_THREAD_ID_MASK                                   (0x0000FFFFU)
#define CSL_UDMAP_TXCCFG_CHAN_THREAD_ID_SHIFT                                  (0x00000000U)
#define CSL_UDMAP_TXCCFG_CHAN_THREAD_ID_MAX                                    (0x0000FFFFU)

/* TFIFO_DEPTH */

#define CSL_UDMAP_TXCCFG_CHAN_TFIFO_DEPTH_FDEPTH_MASK                          (0x000007FFU)
#define CSL_UDMAP_TXCCFG_CHAN_TFIFO_DEPTH_FDEPTH_SHIFT                         (0x00000000U)
#define CSL_UDMAP_TXCCFG_CHAN_TFIFO_DEPTH_FDEPTH_MAX                           (0x000007FFU)

/* TST_SCHED */

#define CSL_UDMAP_TXCCFG_CHAN_TST_SCHED_PRIORITY_MASK                          (0x00000003U)
#define CSL_UDMAP_TXCCFG_CHAN_TST_SCHED_PRIORITY_SHIFT                         (0x00000000U)
#define CSL_UDMAP_TXCCFG_CHAN_TST_SCHED_PRIORITY_MAX                           (0x00000003U)

/**************************************************************************
* Hardware Region  : UDMA-P Rx Channel Configuration Registers
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t RCFG;                      /* Rx Channel Configuration Register */
    volatile uint8_t  Resv_20[16];
    volatile uint32_t RCQ;                       /* Rx Channel Completion Queue Register */
    volatile uint8_t  Resv_32[8];
    volatile uint32_t ROES[1];                   /* Rx Channel Output Event Steering Register 0 */
    volatile uint8_t  Resv_96[60];
    volatile uint32_t REOES[1];                  /* Rx Channel Error Output Event Steering Register 0 */
    volatile uint32_t RPRI_CTRL;                 /* Rx Channel Priority Control Register */
    volatile uint32_t THREAD;                    /* Rx Channel Destination ThreadID Mapping Register */
    volatile uint8_t  Resv_128[20];
    volatile uint32_t RST_SCHED;                 /* Rx Channel Static Scheduler Config Register */
    volatile uint8_t  Resv_240[108];
    volatile uint32_t RFLOW_RNG;                 /* Rx Channel Flow Range Register */
    volatile uint8_t  Resv_256[12];
} CSL_udmap_rxccfgRegs_chan;


typedef struct {
    CSL_udmap_rxccfgRegs_chan CHAN[512];
} CSL_udmap_rxccfgRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_UDMAP_RXCCFG_CHAN_RCFG(CHAN)                                       (0x00000000U+((CHAN)*0x100U))
#define CSL_UDMAP_RXCCFG_CHAN_RCQ(CHAN)                                        (0x00000014U+((CHAN)*0x100U))
#define CSL_UDMAP_RXCCFG_CHAN_ROES(CHAN,ROES)                                  (0x00000020U+((CHAN)*0x100U)+((ROES)*0x4U))
#define CSL_UDMAP_RXCCFG_CHAN_REOES(CHAN,REOES)                                (0x00000060U+((CHAN)*0x100U)+((REOES)*0x4U))
#define CSL_UDMAP_RXCCFG_CHAN_RPRI_CTRL(CHAN)                                  (0x00000064U+((CHAN)*0x100U))
#define CSL_UDMAP_RXCCFG_CHAN_THREAD(CHAN)                                     (0x00000068U+((CHAN)*0x100U))
#define CSL_UDMAP_RXCCFG_CHAN_RST_SCHED(CHAN)                                  (0x00000080U+((CHAN)*0x100U))
#define CSL_UDMAP_RXCCFG_CHAN_RFLOW_RNG(CHAN)                                  (0x000000F0U+((CHAN)*0x100U))

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* RCFG */

#define CSL_UDMAP_RXCCFG_CHAN_RCFG_PAUSE_ON_ERR_MASK                           (0x80000000U)
#define CSL_UDMAP_RXCCFG_CHAN_RCFG_PAUSE_ON_ERR_SHIFT                          (0x0000001FU)
#define CSL_UDMAP_RXCCFG_CHAN_RCFG_PAUSE_ON_ERR_MAX                            (0x00000001U)

#define CSL_UDMAP_RXCCFG_CHAN_RCFG_ATYPE_MASK                                  (0x03000000U)
#define CSL_UDMAP_RXCCFG_CHAN_RCFG_ATYPE_SHIFT                                 (0x00000018U)
#define CSL_UDMAP_RXCCFG_CHAN_RCFG_ATYPE_MAX                                   (0x00000003U)

#define CSL_UDMAP_RXCCFG_CHAN_RCFG_CHAN_TYPE_MASK                              (0x000F0000U)
#define CSL_UDMAP_RXCCFG_CHAN_RCFG_CHAN_TYPE_SHIFT                             (0x00000010U)
#define CSL_UDMAP_RXCCFG_CHAN_RCFG_CHAN_TYPE_MAX                               (0x0000000FU)

#define CSL_UDMAP_RXCCFG_CHAN_RCFG_IGNORE_SHORT_MASK                           (0x00008000U)
#define CSL_UDMAP_RXCCFG_CHAN_RCFG_IGNORE_SHORT_SHIFT                          (0x0000000FU)
#define CSL_UDMAP_RXCCFG_CHAN_RCFG_IGNORE_SHORT_MAX                            (0x00000001U)

#define CSL_UDMAP_RXCCFG_CHAN_RCFG_IGNORE_LONG_MASK                            (0x00004000U)
#define CSL_UDMAP_RXCCFG_CHAN_RCFG_IGNORE_LONG_SHIFT                           (0x0000000EU)
#define CSL_UDMAP_RXCCFG_CHAN_RCFG_IGNORE_LONG_MAX                             (0x00000001U)

#define CSL_UDMAP_RXCCFG_CHAN_RCFG_BURST_SIZE_MASK                             (0x00000C00U)
#define CSL_UDMAP_RXCCFG_CHAN_RCFG_BURST_SIZE_SHIFT                            (0x0000000AU)
#define CSL_UDMAP_RXCCFG_CHAN_RCFG_BURST_SIZE_MAX                              (0x00000003U)

#define CSL_UDMAP_RXCCFG_CHAN_RCFG_FETCH_SIZE_MASK                             (0x0000007FU)
#define CSL_UDMAP_RXCCFG_CHAN_RCFG_FETCH_SIZE_SHIFT                            (0x00000000U)
#define CSL_UDMAP_RXCCFG_CHAN_RCFG_FETCH_SIZE_MAX                              (0x0000007FU)

/* RCQ */

#define CSL_UDMAP_RXCCFG_CHAN_RCQ_RXCQ_QNUM_MASK                               (0x0000FFFFU)
#define CSL_UDMAP_RXCCFG_CHAN_RCQ_RXCQ_QNUM_SHIFT                              (0x00000000U)
#define CSL_UDMAP_RXCCFG_CHAN_RCQ_RXCQ_QNUM_MAX                                (0x0000FFFFU)

/* ROES */

#define CSL_UDMAP_RXCCFG_CHAN_ROES_EVT_NUM_MASK                                (0x0000FFFFU)
#define CSL_UDMAP_RXCCFG_CHAN_ROES_EVT_NUM_SHIFT                               (0x00000000U)
#define CSL_UDMAP_RXCCFG_CHAN_ROES_EVT_NUM_MAX                                 (0x0000FFFFU)

/* REOES */

#define CSL_UDMAP_RXCCFG_CHAN_REOES_EVT_NUM_MASK                               (0x0000FFFFU)
#define CSL_UDMAP_RXCCFG_CHAN_REOES_EVT_NUM_SHIFT                              (0x00000000U)
#define CSL_UDMAP_RXCCFG_CHAN_REOES_EVT_NUM_MAX                                (0x0000FFFFU)

/* RPRI_CTRL */

#define CSL_UDMAP_RXCCFG_CHAN_RPRI_CTRL_PRIORITY_MASK                          (0x70000000U)
#define CSL_UDMAP_RXCCFG_CHAN_RPRI_CTRL_PRIORITY_SHIFT                         (0x0000001CU)
#define CSL_UDMAP_RXCCFG_CHAN_RPRI_CTRL_PRIORITY_MAX                           (0x00000007U)

#define CSL_UDMAP_RXCCFG_CHAN_RPRI_CTRL_QOS_MASK                               (0x00070000U)
#define CSL_UDMAP_RXCCFG_CHAN_RPRI_CTRL_QOS_SHIFT                              (0x00000010U)
#define CSL_UDMAP_RXCCFG_CHAN_RPRI_CTRL_QOS_MAX                                (0x00000007U)

#define CSL_UDMAP_RXCCFG_CHAN_RPRI_CTRL_ORDERID_MASK                           (0x0000000FU)
#define CSL_UDMAP_RXCCFG_CHAN_RPRI_CTRL_ORDERID_SHIFT                          (0x00000000U)
#define CSL_UDMAP_RXCCFG_CHAN_RPRI_CTRL_ORDERID_MAX                            (0x0000000FU)

/* THREAD */

#define CSL_UDMAP_RXCCFG_CHAN_THREAD_ID_MASK                                   (0x0000FFFFU)
#define CSL_UDMAP_RXCCFG_CHAN_THREAD_ID_SHIFT                                  (0x00000000U)
#define CSL_UDMAP_RXCCFG_CHAN_THREAD_ID_MAX                                    (0x0000FFFFU)

/* RST_SCHED */

#define CSL_UDMAP_RXCCFG_CHAN_RST_SCHED_PRIORITY_MASK                          (0x00000003U)
#define CSL_UDMAP_RXCCFG_CHAN_RST_SCHED_PRIORITY_SHIFT                         (0x00000000U)
#define CSL_UDMAP_RXCCFG_CHAN_RST_SCHED_PRIORITY_MAX                           (0x00000003U)

/* RFLOW_RNG */

#define CSL_UDMAP_RXCCFG_CHAN_RFLOW_RNG_FLOWID_CNT_MASK                        (0x7FFF0000U)
#define CSL_UDMAP_RXCCFG_CHAN_RFLOW_RNG_FLOWID_CNT_SHIFT                       (0x00000010U)
#define CSL_UDMAP_RXCCFG_CHAN_RFLOW_RNG_FLOWID_CNT_MAX                         (0x00007FFFU)

#define CSL_UDMAP_RXCCFG_CHAN_RFLOW_RNG_FLOWID_START_MASK                      (0x00003FFFU)
#define CSL_UDMAP_RXCCFG_CHAN_RFLOW_RNG_FLOWID_START_SHIFT                     (0x00000000U)
#define CSL_UDMAP_RXCCFG_CHAN_RFLOW_RNG_FLOWID_START_MAX                       (0x00003FFFU)

/**************************************************************************
* Hardware Region  : UDMA-P Tx Channel Realtime Registers
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t CTL;                       /* Tx Channel Realtime Control Register */
    volatile uint8_t  Resv_8[4];
    volatile uint32_t SWTRIG;                    /* Tx Channel Realtime Software Trigger Register */
    volatile uint8_t  Resv_64[52];
    volatile uint32_t STATUS0;                   /* Tx Channel Realtime Status Register 0 */
    volatile uint32_t STATUS1;                   /* Tx Channel Realtime Status Register 1 */
    volatile uint8_t  Resv_128[56];
    volatile uint32_t STDATA[32];                /* Tx Channel Realtime State Data Register */
    volatile uint8_t  Resv_512[256];
    volatile uint32_t PEER0;                     /* Tx Channel Real-time Remote Peer Register 0 */
    volatile uint32_t PEER1;                     /* Tx Channel Real-time Remote Peer Register 1 */
    volatile uint32_t PEER2;                     /* Tx Channel Real-time Remote Peer Register 2 */
    volatile uint32_t PEER3;                     /* Tx Channel Real-time Remote Peer Register 3 */
    volatile uint32_t PEER4;                     /* Tx Channel Real-time Remote Peer Register 4 */
    volatile uint32_t PEER5;                     /* Tx Channel Real-time Remote Peer Register 5 */
    volatile uint32_t PEER6;                     /* Tx Channel Real-time Remote Peer Register 6 */
    volatile uint32_t PEER7;                     /* Tx Channel Real-time Remote Peer Register 7 */
    volatile uint32_t PEER8;                     /* Tx Channel Real-time Remote Peer Register 8 */
    volatile uint32_t PEER9;                     /* Tx Channel Real-time Remote Peer Register 9 */
    volatile uint32_t PEER10;                    /* Tx Channel Real-time Remote Peer Register 10 */
    volatile uint32_t PEER11;                    /* Tx Channel Real-time Remote Peer Register 11 */
    volatile uint32_t PEER12;                    /* Tx Channel Real-time Remote Peer Register 12 */
    volatile uint32_t PEER13;                    /* Tx Channel Real-time Remote Peer Register 13 */
    volatile uint32_t PEER14;                    /* Tx Channel Real-time Remote Peer Register 14 */
    volatile uint32_t PEER15;                    /* Tx Channel Real-time Remote Peer Register 15 */
    volatile uint8_t  Resv_1024[448];
    volatile uint32_t PCNT;                      /* Tx Channel Real-time Packet Count Statistics Register */
    volatile uint8_t  Resv_1032[4];
    volatile uint32_t BCNT;                      /* Tx Channel Real-time Completed Byte Count Statistics Register */
    volatile uint8_t  Resv_1040[4];
    volatile uint32_t SBCNT;                     /* Tx Channel Real-time Started Byte Count Statistics Register */
    volatile uint8_t  Resv_4096[3052];
} CSL_udmap_txcrtRegs_chan;


typedef struct {
    CSL_udmap_txcrtRegs_chan CHAN[1024];
} CSL_udmap_txcrtRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_UDMAP_TXCRT_CHAN_CTL(CHAN)                                         (0x00000000U+((CHAN)*0x1000U))
#define CSL_UDMAP_TXCRT_CHAN_SWTRIG(CHAN)                                      (0x00000008U+((CHAN)*0x1000U))
#define CSL_UDMAP_TXCRT_CHAN_STATUS0(CHAN)                                     (0x00000040U+((CHAN)*0x1000U))
#define CSL_UDMAP_TXCRT_CHAN_STATUS1(CHAN)                                     (0x00000044U+((CHAN)*0x1000U))
#define CSL_UDMAP_TXCRT_CHAN_STDATA(CHAN,STDATA)                               (0x00000080U+((CHAN)*0x1000U)+((STDATA)*0x4U))
#define CSL_UDMAP_TXCRT_CHAN_PEER0(CHAN)                                       (0x00000200U+((CHAN)*0x1000U))
#define CSL_UDMAP_TXCRT_CHAN_PEER1(CHAN)                                       (0x00000204U+((CHAN)*0x1000U))
#define CSL_UDMAP_TXCRT_CHAN_PEER2(CHAN)                                       (0x00000208U+((CHAN)*0x1000U))
#define CSL_UDMAP_TXCRT_CHAN_PEER3(CHAN)                                       (0x0000020CU+((CHAN)*0x1000U))
#define CSL_UDMAP_TXCRT_CHAN_PEER4(CHAN)                                       (0x00000210U+((CHAN)*0x1000U))
#define CSL_UDMAP_TXCRT_CHAN_PEER5(CHAN)                                       (0x00000214U+((CHAN)*0x1000U))
#define CSL_UDMAP_TXCRT_CHAN_PEER6(CHAN)                                       (0x00000218U+((CHAN)*0x1000U))
#define CSL_UDMAP_TXCRT_CHAN_PEER7(CHAN)                                       (0x0000021CU+((CHAN)*0x1000U))
#define CSL_UDMAP_TXCRT_CHAN_PEER8(CHAN)                                       (0x00000220U+((CHAN)*0x1000U))
#define CSL_UDMAP_TXCRT_CHAN_PEER9(CHAN)                                       (0x00000224U+((CHAN)*0x1000U))
#define CSL_UDMAP_TXCRT_CHAN_PEER10(CHAN)                                      (0x00000228U+((CHAN)*0x1000U))
#define CSL_UDMAP_TXCRT_CHAN_PEER11(CHAN)                                      (0x0000022CU+((CHAN)*0x1000U))
#define CSL_UDMAP_TXCRT_CHAN_PEER12(CHAN)                                      (0x00000230U+((CHAN)*0x1000U))
#define CSL_UDMAP_TXCRT_CHAN_PEER13(CHAN)                                      (0x00000234U+((CHAN)*0x1000U))
#define CSL_UDMAP_TXCRT_CHAN_PEER14(CHAN)                                      (0x00000238U+((CHAN)*0x1000U))
#define CSL_UDMAP_TXCRT_CHAN_PEER15(CHAN)                                      (0x0000023CU+((CHAN)*0x1000U))
#define CSL_UDMAP_TXCRT_CHAN_PCNT(CHAN)                                        (0x00000400U+((CHAN)*0x1000U))
#define CSL_UDMAP_TXCRT_CHAN_BCNT(CHAN)                                        (0x00000408U+((CHAN)*0x1000U))
#define CSL_UDMAP_TXCRT_CHAN_SBCNT(CHAN)                                       (0x00000410U+((CHAN)*0x1000U))

/**************************************************************************
* 64K Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t CTL;                       /* Tx Channel Realtime Control Register */
    volatile uint8_t  Resv_8[4];
    volatile uint32_t SWTRIG;                    /* Tx Channel Realtime Software Trigger Register */
    volatile uint8_t  Resv_64[52];
    volatile uint32_t STATUS0;                   /* Tx Channel Realtime Status Register 0 */
    volatile uint32_t STATUS1;                   /* Tx Channel Realtime Status Register 1 */
    volatile uint8_t  Resv_128[56];
    volatile uint32_t STDATA[32];                /* Tx Channel Realtime State Data Register */
    volatile uint8_t  Resv_512[256];
    volatile uint32_t PEER0;                     /* Tx Channel Real-time Remote Peer Register 0 */
    volatile uint32_t PEER1;                     /* Tx Channel Real-time Remote Peer Register 1 */
    volatile uint32_t PEER2;                     /* Tx Channel Real-time Remote Peer Register 2 */
    volatile uint32_t PEER3;                     /* Tx Channel Real-time Remote Peer Register 3 */
    volatile uint32_t PEER4;                     /* Tx Channel Real-time Remote Peer Register 4 */
    volatile uint32_t PEER5;                     /* Tx Channel Real-time Remote Peer Register 5 */
    volatile uint32_t PEER6;                     /* Tx Channel Real-time Remote Peer Register 6 */
    volatile uint32_t PEER7;                     /* Tx Channel Real-time Remote Peer Register 7 */
    volatile uint32_t PEER8;                     /* Tx Channel Real-time Remote Peer Register 8 */
    volatile uint32_t PEER9;                     /* Tx Channel Real-time Remote Peer Register 9 */
    volatile uint32_t PEER10;                    /* Tx Channel Real-time Remote Peer Register 10 */
    volatile uint32_t PEER11;                    /* Tx Channel Real-time Remote Peer Register 11 */
    volatile uint32_t PEER12;                    /* Tx Channel Real-time Remote Peer Register 12 */
    volatile uint32_t PEER13;                    /* Tx Channel Real-time Remote Peer Register 13 */
    volatile uint32_t PEER14;                    /* Tx Channel Real-time Remote Peer Register 14 */
    volatile uint32_t PEER15;                    /* Tx Channel Real-time Remote Peer Register 15 */
    volatile uint8_t  Resv_1024[448];
    volatile uint32_t PCNT;                      /* Tx Channel Real-time Packet Count Statistics Register */
    volatile uint8_t  Resv_1032[4];
    volatile uint32_t BCNT;                      /* Tx Channel Real-time Completed Byte Count Statistics Register */
    volatile uint8_t  Resv_1040[4];
    volatile uint32_t SBCNT;                     /* Tx Channel Real-time Started Byte Count Statistics Register */
    volatile uint8_t  Resv_65536[64492];
} CSL_udmap_txcrtRegs64_chan;


typedef struct {
    CSL_udmap_txcrtRegs64_chan CHAN[1024];
} CSL_udmap_txcrtRegs64;


/**************************************************************************
* 64K Register Macros
**************************************************************************/

#define CSL_UDMAP_TXCRT_64_CHAN_CTL(CHAN)                                      (0x00000000U+((CHAN)*0x10000U))
#define CSL_UDMAP_TXCRT_64_CHAN_SWTRIG(CHAN)                                   (0x00000008U+((CHAN)*0x10000U))
#define CSL_UDMAP_TXCRT_64_CHAN_STATUS0(CHAN)                                  (0x00000040U+((CHAN)*0x10000U))
#define CSL_UDMAP_TXCRT_64_CHAN_STATUS1(CHAN)                                  (0x00000044U+((CHAN)*0x10000U))
#define CSL_UDMAP_TXCRT_64_CHAN_STDATA(CHAN,STDATA)                            (0x00000080U+((CHAN)*0x10000U)+((STDATA)*0x4U))
#define CSL_UDMAP_TXCRT_64_CHAN_PEER0(CHAN)                                    (0x00000200U+((CHAN)*0x10000U))
#define CSL_UDMAP_TXCRT_64_CHAN_PEER1(CHAN)                                    (0x00000204U+((CHAN)*0x10000U))
#define CSL_UDMAP_TXCRT_64_CHAN_PEER2(CHAN)                                    (0x00000208U+((CHAN)*0x10000U))
#define CSL_UDMAP_TXCRT_64_CHAN_PEER3(CHAN)                                    (0x0000020CU+((CHAN)*0x10000U))
#define CSL_UDMAP_TXCRT_64_CHAN_PEER4(CHAN)                                    (0x00000210U+((CHAN)*0x10000U))
#define CSL_UDMAP_TXCRT_64_CHAN_PEER5(CHAN)                                    (0x00000214U+((CHAN)*0x10000U))
#define CSL_UDMAP_TXCRT_64_CHAN_PEER6(CHAN)                                    (0x00000218U+((CHAN)*0x10000U))
#define CSL_UDMAP_TXCRT_64_CHAN_PEER7(CHAN)                                    (0x0000021CU+((CHAN)*0x10000U))
#define CSL_UDMAP_TXCRT_64_CHAN_PEER8(CHAN)                                    (0x00000220U+((CHAN)*0x10000U))
#define CSL_UDMAP_TXCRT_64_CHAN_PEER9(CHAN)                                    (0x00000224U+((CHAN)*0x10000U))
#define CSL_UDMAP_TXCRT_64_CHAN_PEER10(CHAN)                                   (0x00000228U+((CHAN)*0x10000U))
#define CSL_UDMAP_TXCRT_64_CHAN_PEER11(CHAN)                                   (0x0000022CU+((CHAN)*0x10000U))
#define CSL_UDMAP_TXCRT_64_CHAN_PEER12(CHAN)                                   (0x00000230U+((CHAN)*0x10000U))
#define CSL_UDMAP_TXCRT_64_CHAN_PEER13(CHAN)                                   (0x00000234U+((CHAN)*0x10000U))
#define CSL_UDMAP_TXCRT_64_CHAN_PEER14(CHAN)                                   (0x00000238U+((CHAN)*0x10000U))
#define CSL_UDMAP_TXCRT_64_CHAN_PEER15(CHAN)                                   (0x0000023CU+((CHAN)*0x10000U))
#define CSL_UDMAP_TXCRT_64_CHAN_PCNT(CHAN)                                     (0x00000400U+((CHAN)*0x10000U))
#define CSL_UDMAP_TXCRT_64_CHAN_BCNT(CHAN)                                     (0x00000408U+((CHAN)*0x10000U))
#define CSL_UDMAP_TXCRT_64_CHAN_SBCNT(CHAN)                                    (0x00000410U+((CHAN)*0x10000U))

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* CTL */

#define CSL_UDMAP_TXCRT_CHAN_CTL_EN_MASK                                       (0x80000000U)
#define CSL_UDMAP_TXCRT_CHAN_CTL_EN_SHIFT                                      (0x0000001FU)
#define CSL_UDMAP_TXCRT_CHAN_CTL_EN_MAX                                        (0x00000001U)

#define CSL_UDMAP_TXCRT_CHAN_CTL_TDOWN_MASK                                    (0x40000000U)
#define CSL_UDMAP_TXCRT_CHAN_CTL_TDOWN_SHIFT                                   (0x0000001EU)
#define CSL_UDMAP_TXCRT_CHAN_CTL_TDOWN_MAX                                     (0x00000001U)

#define CSL_UDMAP_TXCRT_CHAN_CTL_PAUSE_MASK                                    (0x20000000U)
#define CSL_UDMAP_TXCRT_CHAN_CTL_PAUSE_SHIFT                                   (0x0000001DU)
#define CSL_UDMAP_TXCRT_CHAN_CTL_PAUSE_MAX                                     (0x00000001U)

#define CSL_UDMAP_TXCRT_CHAN_CTL_FTDOWN_MASK                                   (0x10000000U)
#define CSL_UDMAP_TXCRT_CHAN_CTL_FTDOWN_SHIFT                                  (0x0000001CU)
#define CSL_UDMAP_TXCRT_CHAN_CTL_FTDOWN_MAX                                    (0x00000001U)

#define CSL_UDMAP_TXCRT_CHAN_CTL_ERROR_MASK                                    (0x00000001U)
#define CSL_UDMAP_TXCRT_CHAN_CTL_ERROR_SHIFT                                   (0x00000000U)
#define CSL_UDMAP_TXCRT_CHAN_CTL_ERROR_MAX                                     (0x00000001U)

/* SWTRIG */

#define CSL_UDMAP_TXCRT_CHAN_SWTRIG_TRIGGER_MASK                               (0x00000001U)
#define CSL_UDMAP_TXCRT_CHAN_SWTRIG_TRIGGER_SHIFT                              (0x00000000U)
#define CSL_UDMAP_TXCRT_CHAN_SWTRIG_TRIGGER_MAX                                (0x00000001U)

/* STATUS0 */

/* STATUS1 */

/* STDATA */

#define CSL_UDMAP_TXCRT_CHAN_STDATA_STATE_INFO_MASK                            (0xFFFFFFFFU)
#define CSL_UDMAP_TXCRT_CHAN_STDATA_STATE_INFO_SHIFT                           (0x00000000U)
#define CSL_UDMAP_TXCRT_CHAN_STDATA_STATE_INFO_MAX                             (0xFFFFFFFFU)

/* PEER0 */

#define CSL_UDMAP_TXCRT_CHAN_PEER0_PEER_DATA_MASK                              (0xFFFFFFFFU)
#define CSL_UDMAP_TXCRT_CHAN_PEER0_PEER_DATA_SHIFT                             (0x00000000U)
#define CSL_UDMAP_TXCRT_CHAN_PEER0_PEER_DATA_MAX                               (0xFFFFFFFFU)

/* PEER1 */

#define CSL_UDMAP_TXCRT_CHAN_PEER1_PEER_DATA_MASK                              (0xFFFFFFFFU)
#define CSL_UDMAP_TXCRT_CHAN_PEER1_PEER_DATA_SHIFT                             (0x00000000U)
#define CSL_UDMAP_TXCRT_CHAN_PEER1_PEER_DATA_MAX                               (0xFFFFFFFFU)

/* PEER2 */

#define CSL_UDMAP_TXCRT_CHAN_PEER2_PEER_DATA_MASK                              (0xFFFFFFFFU)
#define CSL_UDMAP_TXCRT_CHAN_PEER2_PEER_DATA_SHIFT                             (0x00000000U)
#define CSL_UDMAP_TXCRT_CHAN_PEER2_PEER_DATA_MAX                               (0xFFFFFFFFU)

/* PEER3 */

#define CSL_UDMAP_TXCRT_CHAN_PEER3_PEER_DATA_MASK                              (0xFFFFFFFFU)
#define CSL_UDMAP_TXCRT_CHAN_PEER3_PEER_DATA_SHIFT                             (0x00000000U)
#define CSL_UDMAP_TXCRT_CHAN_PEER3_PEER_DATA_MAX                               (0xFFFFFFFFU)

/* PEER4 */

#define CSL_UDMAP_TXCRT_CHAN_PEER4_PEER_DATA_MASK                              (0xFFFFFFFFU)
#define CSL_UDMAP_TXCRT_CHAN_PEER4_PEER_DATA_SHIFT                             (0x00000000U)
#define CSL_UDMAP_TXCRT_CHAN_PEER4_PEER_DATA_MAX                               (0xFFFFFFFFU)

/* PEER5 */

#define CSL_UDMAP_TXCRT_CHAN_PEER5_PEER_DATA_MASK                              (0xFFFFFFFFU)
#define CSL_UDMAP_TXCRT_CHAN_PEER5_PEER_DATA_SHIFT                             (0x00000000U)
#define CSL_UDMAP_TXCRT_CHAN_PEER5_PEER_DATA_MAX                               (0xFFFFFFFFU)

/* PEER6 */

#define CSL_UDMAP_TXCRT_CHAN_PEER6_PEER_DATA_MASK                              (0xFFFFFFFFU)
#define CSL_UDMAP_TXCRT_CHAN_PEER6_PEER_DATA_SHIFT                             (0x00000000U)
#define CSL_UDMAP_TXCRT_CHAN_PEER6_PEER_DATA_MAX                               (0xFFFFFFFFU)

/* PEER7 */

#define CSL_UDMAP_TXCRT_CHAN_PEER7_PEER_DATA_MASK                              (0xFFFFFFFFU)
#define CSL_UDMAP_TXCRT_CHAN_PEER7_PEER_DATA_SHIFT                             (0x00000000U)
#define CSL_UDMAP_TXCRT_CHAN_PEER7_PEER_DATA_MAX                               (0xFFFFFFFFU)

/* PEER8 */

#define CSL_UDMAP_TXCRT_CHAN_PEER8_PEER_DATA_MASK                              (0xFFFFFFFFU)
#define CSL_UDMAP_TXCRT_CHAN_PEER8_PEER_DATA_SHIFT                             (0x00000000U)
#define CSL_UDMAP_TXCRT_CHAN_PEER8_PEER_DATA_MAX                               (0xFFFFFFFFU)

/* PEER9 */

#define CSL_UDMAP_TXCRT_CHAN_PEER9_PEER_DATA_MASK                              (0xFFFFFFFFU)
#define CSL_UDMAP_TXCRT_CHAN_PEER9_PEER_DATA_SHIFT                             (0x00000000U)
#define CSL_UDMAP_TXCRT_CHAN_PEER9_PEER_DATA_MAX                               (0xFFFFFFFFU)

/* PEER10 */

#define CSL_UDMAP_TXCRT_CHAN_PEER10_PEER_DATA_MASK                             (0xFFFFFFFFU)
#define CSL_UDMAP_TXCRT_CHAN_PEER10_PEER_DATA_SHIFT                            (0x00000000U)
#define CSL_UDMAP_TXCRT_CHAN_PEER10_PEER_DATA_MAX                              (0xFFFFFFFFU)

/* PEER11 */

#define CSL_UDMAP_TXCRT_CHAN_PEER11_PEER_DATA_MASK                             (0xFFFFFFFFU)
#define CSL_UDMAP_TXCRT_CHAN_PEER11_PEER_DATA_SHIFT                            (0x00000000U)
#define CSL_UDMAP_TXCRT_CHAN_PEER11_PEER_DATA_MAX                              (0xFFFFFFFFU)

/* PEER12 */

#define CSL_UDMAP_TXCRT_CHAN_PEER12_PEER_DATA_MASK                             (0xFFFFFFFFU)
#define CSL_UDMAP_TXCRT_CHAN_PEER12_PEER_DATA_SHIFT                            (0x00000000U)
#define CSL_UDMAP_TXCRT_CHAN_PEER12_PEER_DATA_MAX                              (0xFFFFFFFFU)

/* PEER13 */

#define CSL_UDMAP_TXCRT_CHAN_PEER13_PEER_DATA_MASK                             (0xFFFFFFFFU)
#define CSL_UDMAP_TXCRT_CHAN_PEER13_PEER_DATA_SHIFT                            (0x00000000U)
#define CSL_UDMAP_TXCRT_CHAN_PEER13_PEER_DATA_MAX                              (0xFFFFFFFFU)

/* PEER14 */

#define CSL_UDMAP_TXCRT_CHAN_PEER14_PEER_DATA_MASK                             (0xFFFFFFFFU)
#define CSL_UDMAP_TXCRT_CHAN_PEER14_PEER_DATA_SHIFT                            (0x00000000U)
#define CSL_UDMAP_TXCRT_CHAN_PEER14_PEER_DATA_MAX                              (0xFFFFFFFFU)

/* PEER15 */

#define CSL_UDMAP_TXCRT_CHAN_PEER15_PEER_DATA_MASK                             (0xFFFFFFFFU)
#define CSL_UDMAP_TXCRT_CHAN_PEER15_PEER_DATA_SHIFT                            (0x00000000U)
#define CSL_UDMAP_TXCRT_CHAN_PEER15_PEER_DATA_MAX                              (0xFFFFFFFFU)

/* PCNT */

#define CSL_UDMAP_TXCRT_CHAN_PCNT_PCNT_MASK                                    (0xFFFFFFFFU)
#define CSL_UDMAP_TXCRT_CHAN_PCNT_PCNT_SHIFT                                   (0x00000000U)
#define CSL_UDMAP_TXCRT_CHAN_PCNT_PCNT_MAX                                     (0xFFFFFFFFU)

/* BCNT */

#define CSL_UDMAP_TXCRT_CHAN_BCNT_BCNT_MASK                                    (0xFFFFFFFFU)
#define CSL_UDMAP_TXCRT_CHAN_BCNT_BCNT_SHIFT                                   (0x00000000U)
#define CSL_UDMAP_TXCRT_CHAN_BCNT_BCNT_MAX                                     (0xFFFFFFFFU)

/* SBCNT */

#define CSL_UDMAP_TXCRT_CHAN_SBCNT_SBCNT_MASK                                  (0xFFFFFFFFU)
#define CSL_UDMAP_TXCRT_CHAN_SBCNT_SBCNT_SHIFT                                 (0x00000000U)
#define CSL_UDMAP_TXCRT_CHAN_SBCNT_SBCNT_MAX                                   (0xFFFFFFFFU)

/**************************************************************************
* Hardware Region  : UDMA-P Rx Channel Realtime Registers
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t CTL;                       /* Rx Channel Realtime Control Register */
    volatile uint8_t  Resv_8[4];
    volatile uint32_t SWTRIG;                    /* Rx Channel Realtime Software Trigger Register */
    volatile uint8_t  Resv_64[52];
    volatile uint32_t STATUS0;                   /* Tx Channel Realtime Status Register 0 */
    volatile uint32_t STATUS1;                   /* Tx Channel Realtime Status Register 1 */
    volatile uint8_t  Resv_128[56];
    volatile uint32_t STDATA[32];                /* Rx Channel Realtime State Data Register */
    volatile uint8_t  Resv_512[256];
    volatile uint32_t PEER0;                     /* Rx Channel Real-time Remote Peer Register 0 */
    volatile uint32_t PEER1;                     /* Rx Channel Real-time Remote Peer Register 1 */
    volatile uint32_t PEER2;                     /* Rx Channel Real-time Remote Peer Register 2 */
    volatile uint32_t PEER3;                     /* Rx Channel Real-time Remote Peer Register 3 */
    volatile uint32_t PEER4;                     /* Rx Channel Real-time Remote Peer Register 4 */
    volatile uint32_t PEER5;                     /* Rx Channel Real-time Remote Peer Register 5 */
    volatile uint32_t PEER6;                     /* Rx Channel Real-time Remote Peer Register 6 */
    volatile uint32_t PEER7;                     /* Rx Channel Real-time Remote Peer Register 7 */
    volatile uint32_t PEER8;                     /* Rx Channel Real-time Remote Peer Register 8 */
    volatile uint32_t PEER9;                     /* Rx Channel Real-time Remote Peer Register 9 */
    volatile uint32_t PEER10;                    /* Rx Channel Real-time Remote Peer Register 10 */
    volatile uint32_t PEER11;                    /* Rx Channel Real-time Remote Peer Register 11 */
    volatile uint32_t PEER12;                    /* Rx Channel Real-time Remote Peer Register 12 */
    volatile uint32_t PEER13;                    /* Rx Channel Real-time Remote Peer Register 13 */
    volatile uint32_t PEER14;                    /* Rx Channel Real-time Remote Peer Register 14 */
    volatile uint32_t PEER15;                    /* Rx Channel Real-time Remote Peer Register 15 */
    volatile uint8_t  Resv_1024[448];
    volatile uint32_t PCNT;                      /* Rx Channel Real-time Packet Count Statistics Register */
    volatile uint8_t  Resv_1032[4];
    volatile uint32_t BCNT;                      /* Rx Channel Real-time Completed Byte Count Statistics Register */
    volatile uint8_t  Resv_1040[4];
    volatile uint32_t SBCNT;                     /* Rx Channel Real-time Started Byte Count Statistics Register */
    volatile uint8_t  Resv_4096[3052];
} CSL_udmap_rxcrtRegs_chan;


typedef struct {
    CSL_udmap_rxcrtRegs_chan CHAN[512];
} CSL_udmap_rxcrtRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_UDMAP_RXCRT_CHAN_CTL(CHAN)                                         (0x00000000U+((CHAN)*0x1000U))
#define CSL_UDMAP_RXCRT_CHAN_SWTRIG(CHAN)                                      (0x00000008U+((CHAN)*0x1000U))
#define CSL_UDMAP_RXCRT_CHAN_STATUS0(CHAN)                                     (0x00000040U+((CHAN)*0x1000U))
#define CSL_UDMAP_RXCRT_CHAN_STATUS1(CHAN)                                     (0x00000044U+((CHAN)*0x1000U))
#define CSL_UDMAP_RXCRT_CHAN_STDATA(CHAN,STDATA)                               (0x00000080U+((CHAN)*0x1000U)+((STDATA)*0x4U))
#define CSL_UDMAP_RXCRT_CHAN_PEER0(CHAN)                                       (0x00000200U+((CHAN)*0x1000U))
#define CSL_UDMAP_RXCRT_CHAN_PEER1(CHAN)                                       (0x00000204U+((CHAN)*0x1000U))
#define CSL_UDMAP_RXCRT_CHAN_PEER2(CHAN)                                       (0x00000208U+((CHAN)*0x1000U))
#define CSL_UDMAP_RXCRT_CHAN_PEER3(CHAN)                                       (0x0000020CU+((CHAN)*0x1000U))
#define CSL_UDMAP_RXCRT_CHAN_PEER4(CHAN)                                       (0x00000210U+((CHAN)*0x1000U))
#define CSL_UDMAP_RXCRT_CHAN_PEER5(CHAN)                                       (0x00000214U+((CHAN)*0x1000U))
#define CSL_UDMAP_RXCRT_CHAN_PEER6(CHAN)                                       (0x00000218U+((CHAN)*0x1000U))
#define CSL_UDMAP_RXCRT_CHAN_PEER7(CHAN)                                       (0x0000021CU+((CHAN)*0x1000U))
#define CSL_UDMAP_RXCRT_CHAN_PEER8(CHAN)                                       (0x00000220U+((CHAN)*0x1000U))
#define CSL_UDMAP_RXCRT_CHAN_PEER9(CHAN)                                       (0x00000224U+((CHAN)*0x1000U))
#define CSL_UDMAP_RXCRT_CHAN_PEER10(CHAN)                                      (0x00000228U+((CHAN)*0x1000U))
#define CSL_UDMAP_RXCRT_CHAN_PEER11(CHAN)                                      (0x0000022CU+((CHAN)*0x1000U))
#define CSL_UDMAP_RXCRT_CHAN_PEER12(CHAN)                                      (0x00000230U+((CHAN)*0x1000U))
#define CSL_UDMAP_RXCRT_CHAN_PEER13(CHAN)                                      (0x00000234U+((CHAN)*0x1000U))
#define CSL_UDMAP_RXCRT_CHAN_PEER14(CHAN)                                      (0x00000238U+((CHAN)*0x1000U))
#define CSL_UDMAP_RXCRT_CHAN_PEER15(CHAN)                                      (0x0000023CU+((CHAN)*0x1000U))
#define CSL_UDMAP_RXCRT_CHAN_PCNT(CHAN)                                        (0x00000400U+((CHAN)*0x1000U))
#define CSL_UDMAP_RXCRT_CHAN_BCNT(CHAN)                                        (0x00000408U+((CHAN)*0x1000U))
#define CSL_UDMAP_RXCRT_CHAN_SBCNT(CHAN)                                       (0x00000410U+((CHAN)*0x1000U))

/**************************************************************************
* 64K Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t CTL;                       /* Rx Channel Realtime Control Register */
    volatile uint8_t  Resv_8[4];
    volatile uint32_t SWTRIG;                    /* Rx Channel Realtime Software Trigger Register */
    volatile uint8_t  Resv_64[52];
    volatile uint32_t STATUS0;                   /* Tx Channel Realtime Status Register 0 */
    volatile uint32_t STATUS1;                   /* Tx Channel Realtime Status Register 1 */
    volatile uint8_t  Resv_128[56];
    volatile uint32_t STDATA[32];                /* Rx Channel Realtime State Data Register */
    volatile uint8_t  Resv_512[256];
    volatile uint32_t PEER0;                     /* Rx Channel Real-time Remote Peer Register 0 */
    volatile uint32_t PEER1;                     /* Rx Channel Real-time Remote Peer Register 1 */
    volatile uint32_t PEER2;                     /* Rx Channel Real-time Remote Peer Register 2 */
    volatile uint32_t PEER3;                     /* Rx Channel Real-time Remote Peer Register 3 */
    volatile uint32_t PEER4;                     /* Rx Channel Real-time Remote Peer Register 4 */
    volatile uint32_t PEER5;                     /* Rx Channel Real-time Remote Peer Register 5 */
    volatile uint32_t PEER6;                     /* Rx Channel Real-time Remote Peer Register 6 */
    volatile uint32_t PEER7;                     /* Rx Channel Real-time Remote Peer Register 7 */
    volatile uint32_t PEER8;                     /* Rx Channel Real-time Remote Peer Register 8 */
    volatile uint32_t PEER9;                     /* Rx Channel Real-time Remote Peer Register 9 */
    volatile uint32_t PEER10;                    /* Rx Channel Real-time Remote Peer Register 10 */
    volatile uint32_t PEER11;                    /* Rx Channel Real-time Remote Peer Register 11 */
    volatile uint32_t PEER12;                    /* Rx Channel Real-time Remote Peer Register 12 */
    volatile uint32_t PEER13;                    /* Rx Channel Real-time Remote Peer Register 13 */
    volatile uint32_t PEER14;                    /* Rx Channel Real-time Remote Peer Register 14 */
    volatile uint32_t PEER15;                    /* Rx Channel Real-time Remote Peer Register 15 */
    volatile uint8_t  Resv_1024[448];
    volatile uint32_t PCNT;                      /* Rx Channel Real-time Packet Count Statistics Register */
    volatile uint8_t  Resv_1032[4];
    volatile uint32_t BCNT;                      /* Rx Channel Real-time Completed Byte Count Statistics Register */
    volatile uint8_t  Resv_1040[4];
    volatile uint32_t SBCNT;                     /* Rx Channel Real-time Started Byte Count Statistics Register */
    volatile uint8_t  Resv_65536[64492];
} CSL_udmap_rxcrtRegs64_chan;


typedef struct {
    CSL_udmap_rxcrtRegs64_chan CHAN[512];
} CSL_udmap_rxcrtRegs64;


/**************************************************************************
* 64K Register Macros
**************************************************************************/

#define CSL_UDMAP_RXCRT_64_CHAN_CTL(CHAN)                                      (0x00000000U+((CHAN)*0x10000U))
#define CSL_UDMAP_RXCRT_64_CHAN_SWTRIG(CHAN)                                   (0x00000008U+((CHAN)*0x10000U))
#define CSL_UDMAP_RXCRT_64_CHAN_STATUS0(CHAN)                                  (0x00000040U+((CHAN)*0x10000U))
#define CSL_UDMAP_RXCRT_64_CHAN_STATUS1(CHAN)                                  (0x00000044U+((CHAN)*0x10000U))
#define CSL_UDMAP_RXCRT_64_CHAN_STDATA(CHAN,STDATA)                            (0x00000080U+((CHAN)*0x10000U)+((STDATA)*0x4U))
#define CSL_UDMAP_RXCRT_64_CHAN_PEER0(CHAN)                                    (0x00000200U+((CHAN)*0x10000U))
#define CSL_UDMAP_RXCRT_64_CHAN_PEER1(CHAN)                                    (0x00000204U+((CHAN)*0x10000U))
#define CSL_UDMAP_RXCRT_64_CHAN_PEER2(CHAN)                                    (0x00000208U+((CHAN)*0x10000U))
#define CSL_UDMAP_RXCRT_64_CHAN_PEER3(CHAN)                                    (0x0000020CU+((CHAN)*0x10000U))
#define CSL_UDMAP_RXCRT_64_CHAN_PEER4(CHAN)                                    (0x00000210U+((CHAN)*0x10000U))
#define CSL_UDMAP_RXCRT_64_CHAN_PEER5(CHAN)                                    (0x00000214U+((CHAN)*0x10000U))
#define CSL_UDMAP_RXCRT_64_CHAN_PEER6(CHAN)                                    (0x00000218U+((CHAN)*0x10000U))
#define CSL_UDMAP_RXCRT_64_CHAN_PEER7(CHAN)                                    (0x0000021CU+((CHAN)*0x10000U))
#define CSL_UDMAP_RXCRT_64_CHAN_PEER8(CHAN)                                    (0x00000220U+((CHAN)*0x10000U))
#define CSL_UDMAP_RXCRT_64_CHAN_PEER9(CHAN)                                    (0x00000224U+((CHAN)*0x10000U))
#define CSL_UDMAP_RXCRT_64_CHAN_PEER10(CHAN)                                   (0x00000228U+((CHAN)*0x10000U))
#define CSL_UDMAP_RXCRT_64_CHAN_PEER11(CHAN)                                   (0x0000022CU+((CHAN)*0x10000U))
#define CSL_UDMAP_RXCRT_64_CHAN_PEER12(CHAN)                                   (0x00000230U+((CHAN)*0x10000U))
#define CSL_UDMAP_RXCRT_64_CHAN_PEER13(CHAN)                                   (0x00000234U+((CHAN)*0x10000U))
#define CSL_UDMAP_RXCRT_64_CHAN_PEER14(CHAN)                                   (0x00000238U+((CHAN)*0x10000U))
#define CSL_UDMAP_RXCRT_64_CHAN_PEER15(CHAN)                                   (0x0000023CU+((CHAN)*0x10000U))
#define CSL_UDMAP_RXCRT_64_CHAN_PCNT(CHAN)                                     (0x00000400U+((CHAN)*0x10000U))
#define CSL_UDMAP_RXCRT_64_CHAN_BCNT(CHAN)                                     (0x00000408U+((CHAN)*0x10000U))
#define CSL_UDMAP_RXCRT_64_CHAN_SBCNT(CHAN)                                    (0x00000410U+((CHAN)*0x10000U))

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* CTL */

#define CSL_UDMAP_RXCRT_CHAN_CTL_EN_MASK                                       (0x80000000U)
#define CSL_UDMAP_RXCRT_CHAN_CTL_EN_SHIFT                                      (0x0000001FU)
#define CSL_UDMAP_RXCRT_CHAN_CTL_EN_MAX                                        (0x00000001U)

#define CSL_UDMAP_RXCRT_CHAN_CTL_TDOWN_MASK                                    (0x40000000U)
#define CSL_UDMAP_RXCRT_CHAN_CTL_TDOWN_SHIFT                                   (0x0000001EU)
#define CSL_UDMAP_RXCRT_CHAN_CTL_TDOWN_MAX                                     (0x00000001U)

#define CSL_UDMAP_RXCRT_CHAN_CTL_PAUSE_MASK                                    (0x20000000U)
#define CSL_UDMAP_RXCRT_CHAN_CTL_PAUSE_SHIFT                                   (0x0000001DU)
#define CSL_UDMAP_RXCRT_CHAN_CTL_PAUSE_MAX                                     (0x00000001U)

#define CSL_UDMAP_RXCRT_CHAN_CTL_FTDOWN_MASK                                   (0x10000000U)
#define CSL_UDMAP_RXCRT_CHAN_CTL_FTDOWN_SHIFT                                  (0x0000001CU)
#define CSL_UDMAP_RXCRT_CHAN_CTL_FTDOWN_MAX                                    (0x00000001U)

#define CSL_UDMAP_RXCRT_CHAN_CTL_ERROR_MASK                                    (0x00000001U)
#define CSL_UDMAP_RXCRT_CHAN_CTL_ERROR_SHIFT                                   (0x00000000U)
#define CSL_UDMAP_RXCRT_CHAN_CTL_ERROR_MAX                                     (0x00000001U)

/* SWTRIG */

#define CSL_UDMAP_RXCRT_CHAN_SWTRIG_TRIGGER_MASK                               (0x00000001U)
#define CSL_UDMAP_RXCRT_CHAN_SWTRIG_TRIGGER_SHIFT                              (0x00000000U)
#define CSL_UDMAP_RXCRT_CHAN_SWTRIG_TRIGGER_MAX                                (0x00000001U)

/* STATUS0 */

/* STATUS1 */

/* STDATA */

#define CSL_UDMAP_RXCRT_CHAN_STDATA_STATE_INFO_MASK                            (0xFFFFFFFFU)
#define CSL_UDMAP_RXCRT_CHAN_STDATA_STATE_INFO_SHIFT                           (0x00000000U)
#define CSL_UDMAP_RXCRT_CHAN_STDATA_STATE_INFO_MAX                             (0xFFFFFFFFU)

/* PEER0 */

#define CSL_UDMAP_RXCRT_CHAN_PEER0_PEER_DATA_MASK                              (0xFFFFFFFFU)
#define CSL_UDMAP_RXCRT_CHAN_PEER0_PEER_DATA_SHIFT                             (0x00000000U)
#define CSL_UDMAP_RXCRT_CHAN_PEER0_PEER_DATA_MAX                               (0xFFFFFFFFU)

/* PEER1 */

#define CSL_UDMAP_RXCRT_CHAN_PEER1_PEER_DATA_MASK                              (0xFFFFFFFFU)
#define CSL_UDMAP_RXCRT_CHAN_PEER1_PEER_DATA_SHIFT                             (0x00000000U)
#define CSL_UDMAP_RXCRT_CHAN_PEER1_PEER_DATA_MAX                               (0xFFFFFFFFU)

/* PEER2 */

#define CSL_UDMAP_RXCRT_CHAN_PEER2_PEER_DATA_MASK                              (0xFFFFFFFFU)
#define CSL_UDMAP_RXCRT_CHAN_PEER2_PEER_DATA_SHIFT                             (0x00000000U)
#define CSL_UDMAP_RXCRT_CHAN_PEER2_PEER_DATA_MAX                               (0xFFFFFFFFU)

/* PEER3 */

#define CSL_UDMAP_RXCRT_CHAN_PEER3_PEER_DATA_MASK                              (0xFFFFFFFFU)
#define CSL_UDMAP_RXCRT_CHAN_PEER3_PEER_DATA_SHIFT                             (0x00000000U)
#define CSL_UDMAP_RXCRT_CHAN_PEER3_PEER_DATA_MAX                               (0xFFFFFFFFU)

/* PEER4 */

#define CSL_UDMAP_RXCRT_CHAN_PEER4_PEER_DATA_MASK                              (0xFFFFFFFFU)
#define CSL_UDMAP_RXCRT_CHAN_PEER4_PEER_DATA_SHIFT                             (0x00000000U)
#define CSL_UDMAP_RXCRT_CHAN_PEER4_PEER_DATA_MAX                               (0xFFFFFFFFU)

/* PEER5 */

#define CSL_UDMAP_RXCRT_CHAN_PEER5_PEER_DATA_MASK                              (0xFFFFFFFFU)
#define CSL_UDMAP_RXCRT_CHAN_PEER5_PEER_DATA_SHIFT                             (0x00000000U)
#define CSL_UDMAP_RXCRT_CHAN_PEER5_PEER_DATA_MAX                               (0xFFFFFFFFU)

/* PEER6 */

#define CSL_UDMAP_RXCRT_CHAN_PEER6_PEER_DATA_MASK                              (0xFFFFFFFFU)
#define CSL_UDMAP_RXCRT_CHAN_PEER6_PEER_DATA_SHIFT                             (0x00000000U)
#define CSL_UDMAP_RXCRT_CHAN_PEER6_PEER_DATA_MAX                               (0xFFFFFFFFU)

/* PEER7 */

#define CSL_UDMAP_RXCRT_CHAN_PEER7_PEER_DATA_MASK                              (0xFFFFFFFFU)
#define CSL_UDMAP_RXCRT_CHAN_PEER7_PEER_DATA_SHIFT                             (0x00000000U)
#define CSL_UDMAP_RXCRT_CHAN_PEER7_PEER_DATA_MAX                               (0xFFFFFFFFU)

/* PEER8 */

#define CSL_UDMAP_RXCRT_CHAN_PEER8_PEER_DATA_MASK                              (0xFFFFFFFFU)
#define CSL_UDMAP_RXCRT_CHAN_PEER8_PEER_DATA_SHIFT                             (0x00000000U)
#define CSL_UDMAP_RXCRT_CHAN_PEER8_PEER_DATA_MAX                               (0xFFFFFFFFU)

/* PEER9 */

#define CSL_UDMAP_RXCRT_CHAN_PEER9_PEER_DATA_MASK                              (0xFFFFFFFFU)
#define CSL_UDMAP_RXCRT_CHAN_PEER9_PEER_DATA_SHIFT                             (0x00000000U)
#define CSL_UDMAP_RXCRT_CHAN_PEER9_PEER_DATA_MAX                               (0xFFFFFFFFU)

/* PEER10 */

#define CSL_UDMAP_RXCRT_CHAN_PEER10_PEER_DATA_MASK                             (0xFFFFFFFFU)
#define CSL_UDMAP_RXCRT_CHAN_PEER10_PEER_DATA_SHIFT                            (0x00000000U)
#define CSL_UDMAP_RXCRT_CHAN_PEER10_PEER_DATA_MAX                              (0xFFFFFFFFU)

/* PEER11 */

#define CSL_UDMAP_RXCRT_CHAN_PEER11_PEER_DATA_MASK                             (0xFFFFFFFFU)
#define CSL_UDMAP_RXCRT_CHAN_PEER11_PEER_DATA_SHIFT                            (0x00000000U)
#define CSL_UDMAP_RXCRT_CHAN_PEER11_PEER_DATA_MAX                              (0xFFFFFFFFU)

/* PEER12 */

#define CSL_UDMAP_RXCRT_CHAN_PEER12_PEER_DATA_MASK                             (0xFFFFFFFFU)
#define CSL_UDMAP_RXCRT_CHAN_PEER12_PEER_DATA_SHIFT                            (0x00000000U)
#define CSL_UDMAP_RXCRT_CHAN_PEER12_PEER_DATA_MAX                              (0xFFFFFFFFU)

/* PEER13 */

#define CSL_UDMAP_RXCRT_CHAN_PEER13_PEER_DATA_MASK                             (0xFFFFFFFFU)
#define CSL_UDMAP_RXCRT_CHAN_PEER13_PEER_DATA_SHIFT                            (0x00000000U)
#define CSL_UDMAP_RXCRT_CHAN_PEER13_PEER_DATA_MAX                              (0xFFFFFFFFU)

/* PEER14 */

#define CSL_UDMAP_RXCRT_CHAN_PEER14_PEER_DATA_MASK                             (0xFFFFFFFFU)
#define CSL_UDMAP_RXCRT_CHAN_PEER14_PEER_DATA_SHIFT                            (0x00000000U)
#define CSL_UDMAP_RXCRT_CHAN_PEER14_PEER_DATA_MAX                              (0xFFFFFFFFU)

/* PEER15 */

#define CSL_UDMAP_RXCRT_CHAN_PEER15_PEER_DATA_MASK                             (0xFFFFFFFFU)
#define CSL_UDMAP_RXCRT_CHAN_PEER15_PEER_DATA_SHIFT                            (0x00000000U)
#define CSL_UDMAP_RXCRT_CHAN_PEER15_PEER_DATA_MAX                              (0xFFFFFFFFU)

/* PCNT */

#define CSL_UDMAP_RXCRT_CHAN_PCNT_PCNT_MASK                                    (0xFFFFFFFFU)
#define CSL_UDMAP_RXCRT_CHAN_PCNT_PCNT_SHIFT                                   (0x00000000U)
#define CSL_UDMAP_RXCRT_CHAN_PCNT_PCNT_MAX                                     (0xFFFFFFFFU)

/* BCNT */

#define CSL_UDMAP_RXCRT_CHAN_BCNT_BCNT_MASK                                    (0xFFFFFFFFU)
#define CSL_UDMAP_RXCRT_CHAN_BCNT_BCNT_SHIFT                                   (0x00000000U)
#define CSL_UDMAP_RXCRT_CHAN_BCNT_BCNT_MAX                                     (0xFFFFFFFFU)

/* SBCNT */

#define CSL_UDMAP_RXCRT_CHAN_SBCNT_SBCNT_MASK                                  (0xFFFFFFFFU)
#define CSL_UDMAP_RXCRT_CHAN_SBCNT_SBCNT_SHIFT                                 (0x00000000U)
#define CSL_UDMAP_RXCRT_CHAN_SBCNT_SBCNT_MAX                                   (0xFFFFFFFFU)

#ifdef __cplusplus
}
#endif
#endif
