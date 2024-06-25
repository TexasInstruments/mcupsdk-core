/*
 * SDL ECC
 *
 * Software Diagnostics Library module for ECC
 *
 *  Copyright (c) Texas Instruments Incorporated 2022-2024
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

/**
 * @file  sdl_ecc.h
 *
 * @brief
 *  Header file contains enumerations, structure definitions and function
 *  declarations for SDL ECC interface.
 *  ============================================================================
 */

#ifndef INCLUDE_SDL_ECC_H_
#define INCLUDE_SDL_ECC_H_

#include <stdint.h>
#include <stdbool.h>

#include "sdl_common.h"
#include <sdl/ecc/sdl_ip_ecc.h>
#if defined(SOC_AM263X)
#include <sdl/esm/v0/sdl_esm.h>
#endif
#if defined(SOC_AM263PX) || defined(SOC_AM261X)
#include <sdl/esm/v2/sdl_esm.h>
#endif
#if defined(SOC_AM273X) || defined(SOC_AWR294X)
#include <sdl/esm/v1/sdl_esm.h>
#endif
#if defined(SOC_AM64X) || defined(SOC_AM243X)
#include <sdl/esm/v0/sdl_esm.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 *  @addtogroup SDL_ECC_AGGR_MACROS
    @{
 *
 */
#if defined(SOC_AM263X) || defined(SOC_AM263PX) || defined(SOC_AM261X)

#define SDL_SOC_ECC_AGGR                                            (0U)
#define SDL_R5FSS0_CORE0_ECC_AGGR                                   (1U)
#define SDL_R5FSS0_CORE1_ECC_AGGR                                   (2U)
#define SDL_R5FSS1_CORE0_ECC_AGGR                                   (3U)
#define SDL_R5FSS1_CORE1_ECC_AGGR                                   (4U)
#define SDL_HSM_ECC_AGGR                                            (5U)
#define SDL_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR                         (6U)
#define SDL_MCAN0_MCANSS_MSGMEM_WRAP_ECC_AGGR                       (7U)
#define SDL_MCAN1_MCANSS_MSGMEM_WRAP_ECC_AGGR                       (8U)
#if defined(SOC_AM263X) || defined(SOC_AM263PX)
#define SDL_MCAN2_MCANSS_MSGMEM_WRAP_ECC_AGGR                       (9U)
#define SDL_MCAN3_MCANSS_MSGMEM_WRAP_ECC_AGGR                       (10U)
#endif
#if defined(SOC_AM263PX)
#define SDL_MCAN4_MCANSS_MSGMEM_WRAP_ECC_AGGR                       (11U)
#define SDL_MCAN5_MCANSS_MSGMEM_WRAP_ECC_AGGR                       (12U)
#define SDL_MCAN6_MCANSS_MSGMEM_WRAP_ECC_AGGR                       (13U)
#define SDL_MCAN7_MCANSS_MSGMEM_WRAP_ECC_AGGR                       (14U)
#define SDL_FSS_OSPI_RAM_ECC_AGGR                                   (15U)
#define SDL_FSS_FOTA_8051_RAM_ECC_AGGR                              (16U)
#define SDL_CPSW3GCSS_ECC_AGGR                                      (17U)
#else
#define SDL_CPSW3GCSS_ECC_AGGR                                      (11U)
#endif
#define SDL_ECC_MEMTYPE_MAX                                         (SDL_CPSW3GCSS_ECC_AGGR + 1U)

/* Parity */
#define SDL_R5SS0_CPU0_TCM                                          (0U)
#define SDL_R5SS1_CPU0_TCM                                          (1U)
/* SDL_R5SS0_CPU0_TCM */
#define SDL_R5FSS0_CORE0_ATCM0                                      (1U)
#define SDL_R5FSS0_CORE0_B0TCM0                                     (3U)
#define SDL_R5FSS0_CORE0_B1TCM0                                     (5U)
/* SDL_R5SS0_CPU10_TCM */
#define SDL_R5FSS0_CORE1_ATCM1                                      (2U)
#define SDL_R5FSS0_CORE1_B0TCM1                                     (4U)
#define SDL_R5FSS0_CORE1_B1TCM1                                     (6U)
/* SDL_R5SS1_CPU0_TCM */
#define SDL_R5FSS1_CORE0_ATCM0                                      (7U)
#define SDL_R5FSS1_CORE0_B0TCM0                                     (9U)
#define SDL_R5FSS1_CORE0_B1TCM0                                     (11U)
/* SDL_R5SS1_CPU1_TCM */
#define SDL_R5FSS1_CORE1_ATCM1                                      (8U)
#define SDL_R5FSS1_CORE1_B0TCM1                                     (10U)
#define SDL_R5FSS1_CORE1_B1TCM1                                     (12U)
/* TPCC */
#define SDL_TPCC0                                                   (2)
#endif

#if defined(SOC_AM273X) || defined(SOC_AWR294X)
#define SDL_R5FSS0_CORE0_ECC_AGGR                                   (0U)
#define SDL_R5FSS0_CORE1_ECC_AGGR                                   (1U)
#define SDL_MSS_ECC_AGG_MSS                                         (2U)
#define SDL_DSS_ECC_AGG                                             (3U)
#define SDL_MSS_MCANA_ECC                                           (4U)
#define SDL_MSS_MCANB_ECC                                           (5U)
#define SDL_CPSW3GCSS_ECC_AGGR                                      (6U)
#define SDL_ECC_MEMTYPE_MAX                                         (SDL_CPSW3GCSS_ECC_AGGR + 1U)
/* TCM PARITY */
#define SDL_TCM_PARITY_ATCM0                                        (1U)
#define SDL_TCM_PARITY_ATCM1                                        (2U)
#define SDL_TCM_PARITY_B0TCM0                                       (3U)
#define SDL_TCM_PARITY_B0TCM1                                       (4U)
#define SDL_TCM_PARITY_B1TCM0                                       (5U)
#define SDL_TCM_PARITY_B1TCM1                                       (6U)

/* TPCC */
#define SDL_TPCC0A                                                  (2U)
#define SDL_TPCC0B                                                  (3U)
#define SDL_DSS_TPCCA                                               (4U)
#define SDL_DSS_TPCCB                                               (5U)
#define SDL_DSS_TPCCC                                               (6U)
#endif

#if defined(SOC_AM64X) || defined(SOC_AM243X)
#define SDL_PSRAMECC0_PSRAM256X32E_ECC_AGGR                                                                                 (0u)
#define SDL_MMCSD1_EMMCSD4SS_ECC_AGGR_RXMEM                                                                                 (1u)
#define SDL_ADC0_ADC12_CORE_FIFO_RAM_ECC_AGGR                                                                               (2u)
#define SDL_ECC_AGGR1                                                                                                       (3u)
#define SDL_ECC_AGGR0                                                                                                       (4u)
#define SDL_SA2_UL0_SA2_UL_SA2_UL_ECC_AGGR                                                                                  (5u)
#define SDL_MCAN0_MCANSS_MSGMEM_WRAP_ECC_AGGR                                                                               (6u)
#define SDL_DMASS0_DMSS_AM64_ECCAGGR                                                                                        (7u)
#define SDL_MMCSD1_EMMCSD4SS_ECC_AGGR_TXMEM                                                                                 (8u)
#define SDL_MCAN1_MCANSS_MSGMEM_WRAP_ECC_AGGR                                                                               (9u)
#define SDL_PRU_ICSSG1_ICSS_G_16FF_CORE_BORG_ECC_AGGR                                                                       (10u)
#define SDL_PRU_ICSSG0_ICSS_G_16FF_CORE_BORG_ECC_AGGR                                                                       (11u)
#define SDL_MSRAM_256K2_MSRAM32KX64E_ECC_AGGR                                                                               (12u)
#define SDL_FSS0_FSS_UL_OSPI0_OSPI_WRAP_ECC_AGGR                                                                            (13u)
#define SDL_CPSW0_CPSW_3GUSS_CORE_ECC_CPSW_ECC_AGGR                                                                         (14u)
#define SDL_GICSS0_GIC500SS_1_2_ECC_AGGR                                                                                    (15u)
#define SDL_PCIE0_PCIE_G2X1_64_CORE_AXI_ECC_AGGR                                                                            (16u)
#define SDL_PCIE0_PCIE_G2X1_64_CORE_CORE_ECC_AGGR                                                                           (17u)
#define SDL_USB0_USB3P0SS64_16FFC_USB3P0SS64_CORE_A__ECC_AGGR                                                               (18u)
#define SDL_PDMA1_PDMA_AM64_MAIN1_ECCAGGR                                                                                   (19u)
#define SDL_DMSC0_DMSC_LITE_ECC_AGGR_TXMEM                                                                                  (20u)
#define SDL_MSRAM_256K1_MSRAM32KX64E_ECC_AGGR_TXMEM                                                                         (21u)
#define SDL_MSRAM_256K0_MSRAM32KX64E_ECC_AGGR                                                                               (22u)
#define SDL_MSRAM_256K3_MSRAM32KX64E_ECC_AGGR                                                                               (23u)
#define SDL_MSRAM_256K5_MSRAM32KX64E_ECC_AGGR                                                                               (24u)
#define SDL_MSRAM_256K4_MSRAM32KX64E_ECC_AGGR                                                                               (25u)
#define SDL_MSRAM_256K7_MSRAM32KX64E_ECC_AGGR                                                                               (26u)
#define SDL_MSRAM_256K6_MSRAM32KX64E_ECC_AGGR                                                                               (27u)
#define SDL_MCU_M4FSS0_BLAZAR_ECCAGGR                                                                                       (28u)
#define SDL_PDMA0_PDMA_AM64_MAIN0_ECCAGGR                                                                                   (29u)
#define SDL_MMCSD0_EMMC8SS_16FFC_ECC_AGGR_RXMEM                                                                             (30u)
#define SDL_MMCSD0_EMMC8SS_16FFC_ECC_AGGR_TXMEM                                                                             (31u)
#define SDL_VTM0_K3VTM_N16FFC_ECCAGGR                                                                                       (32u)
#define SDL_R5FSS1_PULSAR_LITE_CPU0_ECC_AGGR                                                                                (33u)
#define SDL_R5FSS1_PULSAR_LITE_CPU1_ECC_AGGR                                                                                (34u)
#define SDL_R5FSS0_PULSAR_LITE_CPU0_ECC_AGGR                                                                                (35u)
#define SDL_R5FSS0_PULSAR_LITE_CPU1_ECC_AGGR                                                                                (36u)
#if defined(SOC_AM64X)
#define SDL_COMPUTE_CLUSTER0_SAM64_A53_256KB_WRAP_A53_DUAL_WRAP_CBA_WRAP_A53_DUAL_WRAP_CBA_COREPAC_ECC_AGGR_CORE0           (37u)
#define SDL_COMPUTE_CLUSTER0_SAM64_A53_256KB_WRAP_A53_DUAL_WRAP_CBA_WRAP_A53_DUAL_WRAP_CBA_COREPAC_ECC_AGGR_COREPAC         (38u)
#define SDL_COMPUTE_CLUSTER0_SAM64_A53_256KB_WRAP_A53_DUAL_WRAP_CBA_WRAP_A53_DUAL_WRAP_CBA_COREPAC_ECC_AGGR_CORE1           (39u)
#define SDL_ECC_MEMTYPE_MAX                                                                                                 (SDL_COMPUTE_CLUSTER0_SAM64_A53_256KB_WRAP_A53_DUAL_WRAP_CBA_WRAP_A53_DUAL_WRAP_CBA_COREPAC_ECC_AGGR_CORE1 + 1U)
#endif
#if defined(SOC_AM243X)
#define SDL_ECC_MEMTYPE_MAX                                                                                                 (SDL_R5FSS0_PULSAR_LITE_CPU1_ECC_AGGR + 1U)
#endif
#endif

/* The following are the memory sub type for Memory type
   SDL_ECC_MEMTYPE_MCU_R5F0_CORE & SDL_ECC_MEMTYPE_MCU_R5F1_CORE */
/* Keeping for backward-compatibility. Recommend to use RAM_ID directly from sdlr_soc_ecc_aggr.h file */
#if defined(SOC_AM273X) || defined(SOC_AWR294X) || defined(SOC_AM263X) || defined(SOC_AM263PX) || defined(SOC_AM261X)
/** \brief Select memory subtype ATCM0 BANK0 */
#define SDL_ECC_R5F_MEM_SUBTYPE_ATCM0_BANK0_VECTOR_ID (SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_ATCM0_BANK0_RAM_ID)
/** \brief Select memory subtype ATCM0 BANK1 */
#define SDL_ECC_R5F_MEM_SUBTYPE_ATCM0_BANK1_VECTOR_ID (SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_ATCM0_BANK1_RAM_ID)
/** \brief Select memory subtype B0TCM0 BANK0 */
#define SDL_ECC_R5F_MEM_SUBTYPE_B0TCM0_BANK0_VECTOR_ID (SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B0TCM0_BANK0_RAM_ID)
/** \brief Select memory subtype B0TCM0 BANK1 */
#define SDL_ECC_R5F_MEM_SUBTYPE_B0TCM0_BANK1_VECTOR_ID (SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B0TCM0_BANK1_RAM_ID)
/** \brief Select memory subtype B1TCM0 BANK0 */
#define SDL_ECC_R5F_MEM_SUBTYPE_B1TCM0_BANK0_VECTOR_ID (SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B1TCM0_BANK0_RAM_ID)
/** \brief Select memory subtype B1TCM0 BANK1 */
#define SDL_ECC_R5F_MEM_SUBTYPE_B1TCM0_BANK1_VECTOR_ID (SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B1TCM0_BANK1_RAM_ID)
/** \brief Select memory subtype VIM RAM */
#define SDL_ECC_R5F_MEM_SUBTYPE_KS_VIM_RAM_VECTOR_ID (SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_KS_VIM_RAMECC_RAM_ID)
#endif

#if defined(SOC_AM64X) || defined(SOC_AM243X)
#define SDL_ECC_R5F_MEM_SUBTYPE_ATCM0_BANK0_VECTOR_ID (SDL_R5FSS0_PULSAR_LITE_CPU0_ECC_AGGR_PULSAR_LITE_ATCM0_BANK0_RAM_ID)
/** \brief Select memory subtype ATCM0 BANK1 */
#define SDL_ECC_R5F_MEM_SUBTYPE_ATCM0_BANK1_VECTOR_ID (SDL_R5FSS0_PULSAR_LITE_CPU0_ECC_AGGR_PULSAR_LITE_ATCM0_BANK1_RAM_ID)
/** \brief Select memory subtype B0TCM0 BANK0 */
#define SDL_ECC_R5F_MEM_SUBTYPE_B0TCM0_BANK0_VECTOR_ID (SDL_R5FSS0_PULSAR_LITE_CPU0_ECC_AGGR_PULSAR_LITE_B0TCM0_BANK0_RAM_ID)
/** \brief Select memory subtype B0TCM0 BANK1 */
#define SDL_ECC_R5F_MEM_SUBTYPE_B0TCM0_BANK1_VECTOR_ID (SDL_R5FSS0_PULSAR_LITE_CPU0_ECC_AGGR_PULSAR_LITE_B0TCM0_BANK1_RAM_ID)
/** \brief Select memory subtype B1TCM0 BANK0 */
#define SDL_ECC_R5F_MEM_SUBTYPE_B1TCM0_BANK0_VECTOR_ID (SDL_R5FSS0_PULSAR_LITE_CPU0_ECC_AGGR_PULSAR_LITE_B1TCM0_BANK0_RAM_ID)
/** \brief Select memory subtype B1TCM0 BANK1 */
#define SDL_ECC_R5F_MEM_SUBTYPE_B1TCM0_BANK1_VECTOR_ID (SDL_R5FSS0_PULSAR_LITE_CPU0_ECC_AGGR_PULSAR_LITE_B1TCM0_BANK1_RAM_ID)
/** \brief Select memory subtype VIM RAM */
#define SDL_ECC_R5F_MEM_SUBTYPE_KS_VIM_RAM_VECTOR_ID (SDL_R5FSS0_PULSAR_LITE_CPU0_ECC_AGGR_CPU0_KS_VIM_RAMECC_RAM_ID)
#endif
/** @} */


/**
 *  @addtogroup SDL_ECC_AGGR_ENUM
    @{
 *
 */
/** ---------------------------------------------------------------------------
 * \brief This enumerator defines the different ECC aggregator types
 * ----------------------------------------------------------------------------
 */
typedef enum {
    SDL_ECC_AGGR_TYPE_INJECT_ONLY = 1,
    /**<  Ecc aggregator inject only */
    SDL_ECC_AGGR_TYPE_FULL_FUNCTION = 2,
    /**<  Ecc aggregator full funtionality */
} SDL_ECC_AggregatorType;


/** ---------------------------------------------------------------------------
 * \brief      ECC Inject error types
 *
 * ----------------------------------------------------------------------------
 */
typedef enum {
    /** No error */
    SDL_INJECT_ECC_NO_ERROR = 0,
    /** 1-Bit ECC Error forcing once */
    SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE = 1,
    /** 2-Bit ECC Error forcing once */
    SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE = 2,
    /** 1-Bit ECC Error Force once on next any Ram read */
    SDL_INJECT_ECC_ERROR_FORCING_1BIT_N_ROW_ONCE = 3,
    /** 2-Bit ECC Error Force once on  next Ram read */
    SDL_INJECT_ECC_ERROR_FORCING_2BIT_N_ROW_ONCE = 4,
    /** 1-Bit ECC Error forcing once */
    SDL_INJECT_ECC_ERROR_FORCING_1BIT_REPEAT = 5,
    /** 2-Bit ECC Error forcing once */
    SDL_INJECT_ECC_ERROR_FORCING_2BIT_REPEAT = 6,
    /** 1-Bit ECC Error Force once on next any Ram read */
    SDL_INJECT_ECC_ERROR_FORCING_1BIT_N_ROW_REPEAT = 7,
    /** 2-Bit ECC Error Force once on  next Ram read */
    SDL_INJECT_ECC_ERROR_FORCING_2BIT_N_ROW_REPEAT = 8,
} SDL_ECC_InjectErrorType;


/** ---------------------------------------------------------------------------
 * \brief This enumerator defines the different ECC RAM ID types
 * ----------------------------------------------------------------------------
 */
typedef enum {
    SDL_ECC_RAM_ID_TYPE_WRAPPER = 0,
    /**<  Ecc RAM ID Wrapper type */
    SDL_ECC_RAM_ID_TYPE_INTERCONNECT = 1,
    /**<  Ecc RAM ID Interconnect/CBASS type */
} SDL_ECC_RamIdType;

/** ---------------------------------------------------------------------------
 * \brief This enumerator indicate ECC memory Sub Type
 *
 * ----------------------------------------------------------------------------
 */
typedef uint32_t SDL_ECC_MemSubType;

/** ---------------------------------------------------------------------------
 * \brief This enumerator indicate ECC memory type
 *
 * ----------------------------------------------------------------------------
 */
typedef uint32_t SDL_ECC_MemType;

/** @} */

/** /brief Format of ECC error Call back function */
typedef void (*SDL_ECC_ErrorCallback_t) (uint32_t errorSrc, uint32_t address);

/** /brief Format of VIM DED vector function */
typedef void (*SDL_ECC_VIMDEDVector_t) (void);

/**
 *  @addtogroup SDL_ECC_AGGR_DATASTRUCT
    @{
 *
 */
/** ---------------------------------------------------------------------------
 * \brief This structure defines the elements of ECC  Init configuration
 * ----------------------------------------------------------------------------
 */
typedef struct SDL_ECC_InitConfig_s
{
    uint32_t numRams;
    /**< Max number of memory sections ECC is enabled on R5F,
         the memory sections include ATCM, VIM, BTCM RAMs */
    SDL_ECC_MemSubType *pMemSubTypeList;
    /**< Pointer to list of Vector ID types of ECC enabled memory sections */
} SDL_ECC_InitConfig_t;

/** ---------------------------------------------------------------------------
 * \brief This structure defines the inject error configuration
 * ----------------------------------------------------------------------------
 */
typedef struct SDL_ECC_InjectErrorConfig_s
{
    uint32_t *pErrMem;
    /**< Address to inject error */
    uint32_t flipBitMask;
    /**< Bit location to flip bits */
    uint32_t chkGrp;
    /**< Group checker (for Interconnect RAM ID's only) */
}  SDL_ECC_InjectErrorConfig_t;

/** ---------------------------------------------------------------------------
 * \brief This structure defines the error status information
 *  ---------------------------------------------------------------------------
 */
typedef struct SDL_ECC_ErrorInfo_s
{
    SDL_ECC_MemType eccMemType;
    /**< ECC Memory type */
    SDL_ECC_MemSubType memSubType;
    /**< Memory subtype */
    SDL_Ecc_AggrIntrSrc intrSrc;
    /**< Interrupt source */
    uint32_t bitErrCnt;
    /**< bit error count for the interrupt source. 0-2 Number of errors, 3 means 3 or more errors. */
    uint32_t injectBitErrCnt;
    /**< inject bit error count for the interrupt source (valid for EDC only). 0-2 Number of errors, 3 means 3 or more errors. */
    uint32_t bitErrorGroup;
    /**< bit error group. Indicates the Checker Group where the error occurred. */
    uint64_t bitErrorOffset;
    /**< bit error offset */
} SDL_ECC_ErrorInfo_t;

/** @} */

/**
 *  @addtogroup SDL_ECC_AGGR_FUNCTION
    @{
 *
 */
/** ============================================================================*
 *
 * \brief   Initializes an  module for usage with ECC module
 *
 * \param   esmInstType: Instance of
 *
 * \return  SDL_PASS : Success; SDL_FAIL for failures
 */
int32_t SDL_ECC_initEsm (const SDL_ESM_Inst esmInstType);

/** ============================================================================*
 *
 * \brief   Initializes ECC module for ECC detection
 *
 * \param  eccMemType ECC memory type
 * \param  pECCInitConfig     Pointer to Ecc init configuration
 *
 * \return  SDL_PASS : Success; SDL_FAIL for failuresn
 */
int32_t SDL_ECC_init (SDL_ECC_MemType eccMemType,
                        const SDL_ECC_InitConfig_t *pECCInitConfig);

/** ============================================================================
 *
 * \brief   Initializes Memory to be ready for ECC error detection.
 *          Assumes ECC is already enabled.
 *
 * \param  eccMemType ECC memory type
 * \param  memSubType: Memory subtype
 *
 * \return  SDL_PASS : Success; SDL_FAIL for failures
 */
int32_t SDL_ECC_initMemory (SDL_ECC_MemType eccMemType,
                               SDL_ECC_MemSubType memSubType);

/** ============================================================================
 *
 * \brief   Runs self test by injecting and error and monitor response
 *          Assumes ECC is already enabled.
 *
 * \param  eccMemType ECC memory type
 * \param  memSubType: Memory subtype
 * \param  errorType: ECC Self test type
 * \param  pECCErrorConfig: Pointer to Error configuration
 * \param  selfTestTimeOut: Number of retries before time out
 *
 * \return  SDL_PASS : Success; SDL_FAIL for failures
 */
int32_t SDL_ECC_selfTest(SDL_ECC_MemType eccMemType,
                            SDL_ECC_MemSubType memSubType,
                            SDL_ECC_InjectErrorType errorType,
                            const SDL_ECC_InjectErrorConfig_t *pECCErrorConfig,
                            uint32_t selfTestTimeOut);

/** ============================================================================
 *
 * \brief   Injects ECC error at specified location
 *          Assumes ECC is already enabled.
 *
 * \param  eccMemType ECC memory type
 * \param  memSubType: Memory subtype
 * \param  errorType: ECC error type
 * \param  pECCErrorConfig: Pointer to Error configuration
 *
 * \return  SDL_PASS : Success; SDL_FAIL for failures
 */
int32_t SDL_ECC_injectError(SDL_ECC_MemType eccMemType,
                               SDL_ECC_MemSubType memSubType,
                               SDL_ECC_InjectErrorType errorType,
                               const SDL_ECC_InjectErrorConfig_t *pECCErrorConfig);

/** ============================================================================
 *
 * \brief   Gets the static registers for the specified ECC instance.
 *
 * \param  eccMemType ECC memory type
 * \param  pStaticRegs: Pointer to Static registers structure
 *
 * \return  SDL_PASS : Success; SDL_FAIL for failures
 */
int32_t SDL_ECC_getStaticRegisters(SDL_ECC_MemType eccMemType,
                                   SDL_ECC_staticRegs *pStaticRegs);

/** ============================================================================
 *
 * \brief   Retrieves the ECC error information for the specified memtype and
 *          interrupt source.
 *
 * \param   eccMemType ECC memory type
 * \param   intrSrc: interrupt source
 * \param   pErrorInfo: Pointer to the Error Information
 *
 * \return  SDL_PASS : Success; SDL_FAIL for failures
 */
int32_t SDL_ECC_getErrorInfo(SDL_ECC_MemType eccMemType,
                                SDL_Ecc_AggrIntrSrc intrSrc,
                                SDL_ECC_ErrorInfo_t *pErrorInfo);

/** ============================================================================
 *
 * \brief   Acknowledge the ECC interrupt
 *
 * \param   eccMemType ECC memory type
 * \param   intrSrc: interrupt source
 *
 * \return  SDL_PASS : Success; SDL_FAIL for failures
 */
int32_t SDL_ECC_ackIntr(SDL_ECC_MemType eccMemType,
                        SDL_Ecc_AggrIntrSrc intrSrc);

/** ============================================================================
 *
 * \brief   Retrieves the ECC error information for the specified ESM error. If
 *          it isn't an ECC error or the ECC error is not supported an error is
 *          returned.
 *
 * \param   instance ESM instance
 * \param   intSrc: ESM interrupt number
 * \param   eccMemType: Pointer to the ECC memory type
 * \param   intrSrcType: Pointer to the interrupt source type
 *
 * \return  SDL_PASS : Success; SDL_FAIL for failures
 */
int32_t SDL_ECC_getESMErrorInfo(SDL_ESM_Inst instance, uint32_t intSrc,
                                SDL_ECC_MemType *eccMemType, SDL_Ecc_AggrIntrSrc *intrSrcType);

/** ============================================================================
 *
 * \brief   Clears N pending interrupts for the specified memtype, subtype and
 *          interrupt source.
 *
 * \param   eccMemType ECC memory type
 * \param   memSubType: Memory subtype
 * \param   intrSrc: interrupt source
 * \param   subType: error subtype (valid for EDC types only)
 * \param   numEvents: number of pending interrupts to clear
 *
 * \return  SDL_PASS : Success; SDL_FAIL for failures
 */
int32_t SDL_ECC_clearNIntrPending(SDL_ECC_MemType eccMemType, SDL_ECC_MemSubType memSubType,
                                  SDL_Ecc_AggrIntrSrc intrSrc,
                                  SDL_Ecc_AggrEDCErrorSubType subType, uint32_t numEvents);

/** ============================================================================
 *
 * \brief   Application provided external callback function for ECC handling
 *          Called inside the reference functions when ECC errors occur.
 *          NOTE: This is application supplied and not part of the SDL
 *          If not supplied by application this will result in an linker error
 *
 * \param  eccMemType: ECC Memory Type
 * \param  errorSrc: Error source for the ECC error event.
 * \param  address: Address at which the ECC error occurred.
 * \param  ramId: RAM ID at which the ECC error occurred.
 * \param  bitErrorOffset: Offset at which the ECC error occurred.
 * \param  bitErrorGroup: group checker that reported the error
 *         (Interconnect ECC type only).
 *
 */
void SDL_ECC_applicationCallbackFunction(SDL_ECC_MemType eccMemType,
                                         uint32_t errorSrc,
                                         uint32_t address,
                                         uint32_t ramId,
                                         uint64_t bitErrorOffset,
                                         uint32_t bitErrorGroup);
#if defined(SOC_AM263X) || defined(SOC_AM263PX) || defined(SOC_AM261X)
/** ============================================================================
 *
 * \brief   Injects ECC TCM Parity error
 *
 * \param1  eccMemType: Memory type for ECC AGGR
 * \param2  memSubType: Memory subtype
 * \param3  bitValue  : Bit Value to set particular register
 *
 * \return  SDL_PASS : Success; SDL_EFAIL for failures
 */
int32_t SDL_ECC_tcmParity(SDL_ECC_MemType eccMemType,
                              SDL_ECC_MemSubType memSubType,
							  uint32_t bitValue);
/** ============================================================================
 *
 * \brief   Clear TCM Parity Status Registers
 *
 * \param1  clearVal  : Value to clear particular register
 *
 */
int32_t SDL_cleartcmStatusRegs(uint32_t clearVal);
#endif
#if defined(SOC_AM273X)|| defined(SOC_AWR294X)
/** ============================================================================
 *
 * \brief   Injects ECC TCM Parity error
 *
 * \param1  memSubType: Memory subtype
 * \param2  bitValue  : Bit Value to set particular register
 *
 * \return  SDL_PASS : Success; SDL_EFAIL for failures
 */
int32_t SDL_ECC_tcmParity(SDL_ECC_MemSubType memSubType,
							  uint32_t bitValue);

/***********************************************************************
 *
 * \brief   DSS L2 parity init
 *
 * \param1  void
 * @return  void
 **********************************************************************/
void SDL_ECC_dss_l2_parity_init(void);

/***********************************************************************
 *
 * \brief   DSS L2 parity error inject
 *
 * \param1  injectError : single bit inject for parity error
 * \param2  injectErrAdd: Inject memory address
 * \param3  value       : Initial value before injecting
 * @return  void
 **********************************************************************/
void SDL_ECC_dss_l2_parity_errorInject(uint32_t injectError, uint32_t injectErrAdd, uint32_t value);

/***********************************************************************
 *
 * \brief   The single-bit error correction and double-bit error
 *          detection errors from the memories of L1 and L2 using EDC
 *          Mask and FLG registers
 *
 * \param1  exception_mask_flag : Register value used to enable
 *                                propagation of particular exceptions
 * @return  void
 **********************************************************************/
void SDL_ECC_DSP_Aggregated_EDC_Errors(uint32_t exception_mask_flag);

/***********************************************************************
 *
 * \brief   EDC Command Enable for L1P memory
 *
 * \param1  void
 * @return  SDL_PASS or SDL_EFAIL
 **********************************************************************/
int32_t SDL_ECC_dss_l1p_edc_CMD_EN(void);

/***********************************************************************
 *
 * \brief   EDC Command Suspend for L1P memory
 *
 * \param1  void
 * @return  SDL_PASS or SDL_EFAIL
 **********************************************************************/
int32_t SDL_ECC_dss_l1p_CMD_SUSP(void);

/***********************************************************************
 *
 * \brief   EDC Command Enable for L2 memory
 *
 * \param1  void
 * @return  SDL_PASS or SDL_EFAIL
 **********************************************************************/
int32_t SDL_ECC_dss_l2_edc_CMD_EN(void);

/***********************************************************************
 *
 * \brief   EDC Command Suspend for L2 memory
 *
 * \param1  void
 * @return  SDL_PASS or SDL_EFAIL
 **********************************************************************/
int32_t SDL_ECC_dss_l2_CMD_SUSP(void);

/***********************************************************************
 *
 * \brief   IDMA 1 Transfer function
 *
 * \param1  srcAddr : Source address of the IDMA 1 transfer
 * \param2  destAddr: Destination address of the IDMA 1 transfer
 *
 * @return  void
 **********************************************************************/
void SDL_ECC_IDMA1_transfer(uint32_t srcAddr, uint32_t destAddr);

#endif


/** ============================================================================
 *
 * \brief   Injects TPCC Parity error
 *
 * \param  eccMemType: Memory type for ECC AGGR
 * \param  bitValue  : Bit Value to set particular register
 * \param  paramregvalue: select param register
 * \param  regval : value to be written into param register
 * \return  SDL_PASS : Success; SDL_EFAIL for failures
 */
int32_t SDL_ECC_tpccParity(SDL_ECC_MemType eccMemType,
							  uint32_t bitValue,
							  uint32_t paramregvalue,
							  uint32_t regval);

#if defined(SOC_AM263PX) || defined(SOC_AM261X)
/** ============================================================================
 *
 * \brief   Enable TMU ROM Parity
 *
 */
void SDL_ECC_enableTMUROMParity(void);

/** ============================================================================
 *
 * \brief   Enables TMU Parity error Force Error
 *
 *
 */
void SDL_ECC_enableTMUROMParityForceError(void);

/** ============================================================================
 *
 * \brief   Disables TMU Parity error
 *
 *
 */
void SDL_ECC_disableTMUROMParity(void);

/** ============================================================================
 *
 * \brief   Disables TMU Parity Force Error
 *
 *
 */
void SDL_ECC_disableTMUROMParityErrorForce(void);

/** ============================================================================
 *
 * \brief    Clear TMU Parity error
 *
 *
 */
void SDL_ECC_clearTMUROMParityError(void);

#endif

/** @} */

#ifdef __cplusplus
}
#endif  /* extern "C" */

#endif
