/*
 *  Copyright (C) 2020 Texas Instruments Incorporated
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

#ifndef CSLR_SOC_DEFINES_H_
#define CSLR_SOC_DEFINES_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
/** @brief Number of UART instances */
#define CSL_UART_PER_CNT                (4U)

/** @brief Number of SPI instances */
#define CSL_SPI_PER_CNT                (4U)

/** @brief Number of LIN instances */
#define CSL_LIN_PER_CNT                (3U)

/** @brief Number of I2C instances */
#define CSL_I2C_PER_CNT                (2U)

/** @brief Number of MCAN instances */
#define CSL_MCAN_PER_CNT                (2U)

/** @brief Number of ETPWM instances */
#define CSL_ETPWM_PER_CNT                (10U)

/** @brief Number of ECAP instances */
#define CSL_ECAP_PER_CNT                (8U)

/** @brief Number of EQEP instances */
#define CSL_EQEP_PER_CNT                (2U)

/** @brief Number of SDFM instances */
#define CSL_SDFM_PER_CNT                (2U)

/** @brief Number of ADC instances */
#define CSL_ADC_PER_CNT                (3U)

/** @brief Number of CMPSSA instances */
#define CSL_CMPSSA_PER_CNT                (10U)

/** @brief Number of CMPSSB instances */
#define CSL_CMPSSB_PER_CNT                (0U)

/** @brief Number of DMA Channels */
#define SOC_EDMA_NUM_DMACH                 (64U)
/** @brief Number of QDMA Channels */
#define SOC_EDMA_NUM_QDMACH                (8U)
/** @brief Number of PaRAM Sets available */
#define SOC_EDMA_NUM_PARAMSETS             (256U)
/** @brief Number of Event Queues available */
#define SOC_EDMA_NUM_EVQUE                 (2U)
/** @brief Support for Channel to PaRAM Set mapping */
#define SOC_EDMA_CHMAPEXIST                (1U)
/** @brief Number of EDMA Regions */
#define SOC_EDMA_NUM_REGIONS               (8U)
/** @brief Support for Memory Protection */
#define SOC_EDMA_MEMPROTECT                (1U)

#define MCAN_MSG_RAM_MAX_WORD_COUNT     (4352U)
/*! @brief  Maximum number of Rx Dma buffers. */
#define MCAN_MAX_RX_DMA_BUFFERS             (7U)

/*! @brief  Maximum number of Tx Dma buffers. */
#define MCAN_MAX_TX_DMA_BUFFERS             (4U)

/**
 * \anchor CSL_CoreID
 * \name Core ID's of core or CPUs present on this SOC
 *
 * @{
 */
#define CSL_CORE_ID_R5FSS0_0         (0U)
#define CSL_CORE_ID_R5FSS0_1         (1U)
#define CSL_CORE_ID_MAX              (2U)
/** @} */

/**
 * \anchor PrivID
 * \name Priv ID's of core or CPUs present on this SOC
 *
 * @{
 */
#define PRIV_ID_HSM              (1U)
#define PRIV_ID_R5FSS0_0         (4U)
#define PRIV_ID_R5FSS0_1         (5U)
#define PRIV_ID_ICSSM0           (9U)
#define PRIV_ID_ICSSM1           (11U)
#define PRIV_ID_CPSW             (10U)
#define PRIV_ID_USB              (12U)

/** @} */

/***********************************************************************
 * MSS - CLOCK setting
 ***********************************************************************/
 /* Sys_vclk : 200MHz */
#define MSS_SYS_VCLK                  200000000U
#define R5F_CLOCK_MHZ                 400U

//#define EDMA_MSS_TPCC_A_NUM_PARAM_SETS  (128U)
//#define EDMA_MSS_TPCC_A_NUM_DMA_CHANS   (64U)
//#define EDMA_MSS_TPCC_A_NUM_TC          (2U)
//
//#define EDMA_HSM_TPCC_A_NUM_PARAM_SETS  (128U)
//#define EDMA_HSM_TPCC_A_NUM_TC          (2U)

/**
 *  \anchor CSL_ArmR5ClusterGroupID
 *  \name R5 Cluster Group IDs
 *
 *  @{
 */
/** \brief R5 Cluster Group ID0 */
#define CSL_ARM_R5_CLUSTER_GROUP_ID_0                 ((uint32_t) 0x00U)
/** @} */

/**
 *  \anchor CSL_ArmR5CPUID
 *  \name R5 Core IDs
 *
 *  @{
 */
/** \brief R5 Core ID0 */
#define CSL_ARM_R5_CPU_ID_0                          ((uint32_t) 0x00U)
/** \brief R5 Core ID1 */
#define CSL_ARM_R5_CPU_ID_1                          ((uint32_t) 0x01U)
/** @} */

/***********************************************************************
 * Cache line size definitions
 ***********************************************************************/
/* Cache line size definitions	*/
#if (__ARM_ARCH == 7) && (__ARM_ARCH_PROFILE == 'R') /* R5F */
#define CSL_CACHE_L1P_LINESIZE     (32U)
#define CSL_CACHE_L1D_LINESIZE     (32U)
#elif (__ARM_ARCH == 7) && (__ARM_ARCH_PROFILE == 'M') /* M4F */
/* No cache support */
#endif

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
}
#endif

#endif  /* CSLR_SOC_DEFINES_H_ */
