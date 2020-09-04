/*
 *  Copyright (c) Texas Instruments Incorporated 2019
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
 */

/**
 *  \file udma_test_soc.h
 *
 *  \brief UT SOC specific file
 */

#ifndef UDMA_TEST_SOC_H_
#define UDMA_TEST_SOC_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/** \brief Utility define for Kilobyte, i.e 1024 bytes */
#ifndef KB
#define KB ((uint32_t) 1024U)
#endif

/** \brief Utility define for Megabyte, i.e 1024*1024 bytes */
#ifndef MB
#define MB (KB * KB)
#endif

#define UTILS_MEM_HEAP_SIZE_MSMC        (128U * KB)
#define UTILS_MEM_HEAP_SIZE_DDR         (64U * MB)
#define UTILS_MEM_HEAP_SIZE_OSPI        (16U * MB)
#define UTILS_MEM_HEAP_SIZE_INTERNAL    (32U * KB)

#define UDMA_TEST_SOC_OCMC_MEM_PRESENT  (0U)

#define UDMA_TEST_DEFAULT_UDMA_INST     (UDMA_INST_ID_BCDMA_0)
/*
#define UDMA_TEST_DEFAULT_UDMA_INST     (UDMA_INST_ID_PKTDMA_0)
*/

#define UDMA_TEST_INST_ID_FLOW          (UDMA_INST_ID_PKTDMA_0)

#define UDMA_TEST_RF_SOC                (UDMA_TEST_RF_SOC_AM64X)

#define UDMA_TEST_INST_ID_BCDMA_BC      (UDMA_INST_ID_BCDMA_0)

#define UDMA_TEST_RF_BCDMA_BC_HC        (0U)
#define UDMA_TEST_RF_BCDMA_BC           (UDMA_TEST_RF_SOC | UDMA_TEST_RF_CORE_ALL | UDMA_TEST_RF_CFG_DEF)
#define UDMA_TEST_RF_PKTDMA_BC_HC       (0U)
#define UDMA_TEST_RF_PKTDMA_BC          (0U)

#define UDMA_TEST_RF_BCDMA_BC_INTERNAL_MEM (UDMA_TEST_RF_BCDMA_BC)

#define UDMA_TEST_RF_DRU                (0U)
/* Need at least two channels - so enable only for those cores */
#define UDMA_TEST_RF_CHAIN              (UDMA_TEST_RF_SOC | \
                                         UDMA_TEST_RF_CORE_MPU1_0 | \
                                         UDMA_TEST_RF_CORE_MCU1_0 | \
                                         UDMA_TEST_RF_CORE_MCU1_1 | \
                                         UDMA_TEST_RF_CORE_MCU2_0 | \
                                         UDMA_TEST_RF_CORE_MCU2_1 | \
                                         UDMA_TEST_RF_CFG_DEF)

/* Multipe task testcases - some have only one instance. Doesn't make sense to run from 1 task */
#define UDMA_TEST_RF_BCDMA_BC_HC_MT     (UDMA_TEST_RF_BCDMA_BC_HC)
#define UDMA_TEST_RF_BCDMA_BC_MT        (UDMA_TEST_RF_BCDMA_BC)
#define UDMA_TEST_RF_PKTDMA_BC_HC_MT    (UDMA_TEST_RF_PKTDMA_BC_HC)
#define UDMA_TEST_RF_PKTDMA_BC_MT       (UDMA_TEST_RF_PKTDMA_BC)
#define UDMA_TEST_RF_DRU_MT             (UDMA_TEST_RF_DRU)

#define UDMA_TEST_RF_FLOW               (UDMA_TEST_RF_SOC | UDMA_TEST_RF_CORE_ALL | UDMA_TEST_RF_CFG_DEF)

#define UDMA_TEST_RF_BCDMA_BC_PACING    (UDMA_TEST_RF_BCDMA_BC)
#define UDMA_TEST_RF_BCDMA_BC_PAUSE     (UDMA_TEST_RF_BCDMA_BC)

#define UDMA_TEST_BCDMA_UHC_START       (0U)
#define UDMA_TEST_BCDMA_HC_START        (0U)
#define UDMA_TEST_PKTDMA_UHC_START      (0U)
#define UDMA_TEST_PKTDMA_HC_START       (0U)
#define UDMA_TEST_MAX_BCDMA_BC_UHC_CH   (0U)
#define UDMA_TEST_MAX_BCDMA_BC_HC_CH    (0U)
#define UDMA_TEST_MAX_PKTDMA_BC_UHC_CH  (0U)
#define UDMA_TEST_MAX_PKTDMA_BC_HC_CH   (0U)
#define UDMA_TEST_MAX_PKTDMA_BC_CH      (0U)
#define UDMA_TEST_MAX_DRU_CH            (0U)

//TODO
#define UDMA_TEST_RF_CORE               (UDMA_TEST_RF_CORE_MCU2_0)
#define UDMA_TEST_MAX_BCDMA_BC_CH       (2U)
#if defined (BUILD_MPU1_0)
#define UDMA_TEST_RF_CORE               (UDMA_TEST_RF_CORE_MPU1_0)
#define UDMA_TEST_MAX_BCDMA_BC_CH       (4U)
#endif
#if defined (BUILD_MCU2_0)
#define UDMA_TEST_RF_CORE               (UDMA_TEST_RF_CORE_MCU2_0)
#define UDMA_TEST_MAX_BCDMA_BC_CH       (2U)
#endif
#if defined (BUILD_MCU2_1)
#define UDMA_TEST_RF_CORE               (UDMA_TEST_RF_CORE_MCU2_1)
#define UDMA_TEST_MAX_BCDMA_BC_CH       (2U)
#endif
#if defined (BUILD_MCU1_0)
#define UDMA_TEST_RF_CORE               (UDMA_TEST_RF_CORE_MCU1_0)
#define UDMA_TEST_MAX_BCDMA_BC_CH       (4U)
#endif
#if defined (BUILD_MCU1_1)
#define UDMA_TEST_RF_CORE               (UDMA_TEST_RF_CORE_MCU1_1)
#define UDMA_TEST_MAX_BCDMA_BC_CH       (2U)
#endif
#if defined (BUILD_M4F_0)
#define UDMA_TEST_RF_CORE               (UDMA_TEST_RF_CORE_M4F_0)
#define UDMA_TEST_MAX_BCDMA_BC_CH       (1U)
#endif

#define UDMA_TEST_BCDMA_BC_TCNAME_PREFIX    "BCDMA "

#define UDMA_TEST_LOOP_CNT_MT_SOC               (UDMA_TEST_PERF_LOOP_CNT)

#define UDMA_TEST_RING_MODE_MIN                 (TISCI_MSG_VALUE_RM_RING_MODE_RING)
#define UDMA_TEST_RING_MODE_MAX                 (TISCI_MSG_VALUE_RM_RING_MODE_RING)
#define UDMA_TEST_RING_MODE_DEFAULT_START       (TISCI_MSG_VALUE_RM_RING_MODE_RING)
#define UDMA_TEST_RING_MODE_DEFAULT_STOP        (TISCI_MSG_VALUE_RM_RING_MODE_RING)

#define UDMA_TEST_PKTDMA_UNMAPPED_TX_PEER_CH    (UDMA_PDMA_CH_MAIN0_MCSPI0_CH0_TX)
#define UDMA_TEST_PKTDMA_CPSW_TX_PEER_CH        (UDMA_PSIL_CH_CPSW2_TX)
#define UDMA_TEST_PKTDMA_SAUL_TX_PEER_CH        (UDMA_PSIL_CH_SAUL0_TX)
#define UDMA_TEST_PKTDMA_ICSSG_0_TX_PEER_CH     (UDMA_PSIL_CH_ICSS_G0_TX)
#define UDMA_TEST_PKTDMA_ICSSG_1_TX_PEER_CH     (UDMA_PSIL_CH_ICSS_G1_TX)

#define UDMA_TEST_PKTDMA_UNMAPPED_RX_PEER_CH    (UDMA_PDMA_CH_MAIN0_MCSPI0_CH0_RX)
#define UDMA_TEST_PKTDMA_CPSW_RX_PEER_CH        (UDMA_PSIL_CH_CPSW2_RX)
#define UDMA_TEST_PKTDMA_SAUL_RX_PEER_CH        (UDMA_PSIL_CH_SAUL0_RX)
#define UDMA_TEST_PKTDMA_ICSSG_0_RX_PEER_CH     (UDMA_PSIL_CH_ICSS_G0_RX)
#define UDMA_TEST_PKTDMA_ICSSG_1_RX_PEER_CH     (UDMA_PSIL_CH_ICSS_G1_RX)

/* Redirect for backward compaitibilitly */
#define UDMA_TEST_RF_MAIN_BC                (UDMA_TEST_RF_BCDMA_BC)
#define UDMA_TEST_RF_MCU_BC                 (UDMA_TEST_RF_BCDMA_BC)
#define UDMA_TEST_RF_MCU_BC_INTERNAL_MEM    (UDMA_TEST_RF_BCDMA_BC_INTERNAL_MEM)
#define UDMA_TEST_MAX_MAIN_BC_CH            (UDMA_TEST_MAX_BCDMA_BC_CH)
#define UDMA_TEST_RF_MAIN_BC_MT             (UDMA_TEST_RF_BCDMA_BC_MT)
#define UDMA_TEST_RF_MAIN_BC_PACING         (UDMA_TEST_RF_BCDMA_BC_PACING)
#define UDMA_TEST_RF_MAIN_BC_PAUSE          (UDMA_TEST_RF_BCDMA_BC_PAUSE)
#define UDMA_TEST_INST_ID_MAIN_BC           (UDMA_TEST_INST_ID_BCDMA_BC)
#define UDMA_TEST_INST_ID_MCU_BC            (UDMA_TEST_INST_ID_BCDMA_BC)

#define UDMA_TEST_MAIN_BC_TCNAME_PREFIX     UDMA_TEST_BCDMA_BC_TCNAME_PREFIX
#define UDMA_TEST_MCU_BC_TCNAME_PREFIX      UDMA_TEST_BCDMA_BC_TCNAME_PREFIX

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

uint32_t udmaTestGetMappedRingChNum(Udma_DrvHandle drvHandle, uint32_t mappedRingGrp, uint32_t mappedRingNum);

#ifdef __cplusplus
}
#endif

#endif  /* UDMA_TEST_SOC_H_ */
