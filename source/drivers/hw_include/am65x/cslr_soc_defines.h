/********************************************************************
*  Copyright (C) 2024 Texas Instruments Incorporated
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

/** \brief number of instances of CAL available */
#define CSL_CAL_PER_CNT                     (1U)
/** \brief Defines the total number of PPI available per CAL */
#define CSL_CAL_PPI_CNT                     (2U)
/** \brief Defines the total number of complex IO available per CAL */
#define CSL_CAL_CMPLXIO_CNT                 (1U)
/** \brief Defines the total number of pixel processing context
 *          available per CAL */
#define CSL_CAL_PIX_PROC_CTX_CNT            (4U)

/** @brief interrupt number for CAL. */
#define CSL_CAL_IRQ_NUM                     (71U)

/** @brief Number of UART instances */
#define CSL_UART_PER_CNT                    (3U)

/** @brief Max Number of McSPI instances in a domain */
#define CSL_MCSPI_PER_CNT                   (8U)

/** @brief Max Number of McSPI channels per instance */
#define CSL_MCSPI_CHAN_CNT                  (4U)

/** @brief Number of domains contating McSPI instances */
#define CSL_MCSPI_DOMAIN_CNT                (2U)

/** @brief Number of McSPI instances in MCU domain */
#define CSL_MCSPI_MCU_CNT                   (3U)

/** @brief Number of McSPI instances in MAIN domain */
#define CSL_MCSPI_MAIN_CNT                  (8U)

/** @brief Number of OSPI instances */
#define CSL_OSPI_PER_CNT                    (2U)

/** @brief Number of domains contating OSPI instances.
    Second domain is not currently supported but kept as place holder
    for future use */
#define CSL_OSPI_DOMAIN_CNT                 (2U)

/** Cache line size definitions	*/
#if defined(__aarch64__) /* A53 */
#define CSL_CACHE_L1P_LINESIZE     (64U)
#define CSL_CACHE_L1D_LINESIZE     (64U)
#define CSL_CACHE_L2_LINESIZE      (64U)
#elif (__ARM_ARCH == 7) && (__ARM_ARCH_PROFILE == 'R') /* R5F */ /* R5F */
#define CSL_CACHE_L1P_LINESIZE     (32U)
#define CSL_CACHE_L1D_LINESIZE     (32U)
#endif

/**
 * \anchor CSL_CoreID
 * \name Core ID's of core or CPUs present on this SOC
 *
 * @{
 */
#define CSL_CORE_ID_R5FSS0_0         (0U)
#define CSL_CORE_ID_R5FSS0_1         (1U)
#define CSL_CORE_ID_A53SS0_0         (2U)
#define CSL_CORE_ID_A53SS0_1         (3U)
#define CSL_CORE_ID_A53SS1_0         (4U)
#define CSL_CORE_ID_A53SS1_1         (5U)
#define CSL_CORE_ID_MAX              (6U)
/** @} */

/**
 *  \anchor CSL_ArmR5ClusterGroupID
 *  \name R5 Cluster Group IDs
 *
 *  @{
 */
/** \brief R5 Cluster Group ID0 */
#define CSL_ARM_R5_CLUSTER_GROUP_ID_0                 ((uint32_t) 0x00U)
/** \brief R5 Cluster Group ID1 */
#define CSL_ARM_R5_CLUSTER_GROUP_ID_1                 ((uint32_t) 0x01U)
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

/** @brief Number of ePWM instances */
#define CSL_EPWM_PER_CNT (9U)

/**
 * \brief  MCAN Maximum Message RAM words
 */
#define MCAN_MSG_RAM_MAX_WORD_COUNT     (4352U)
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
