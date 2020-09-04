/*
 * SDL ECC
 *
 * Software Diagnostics Library module for ECC
 *
 *  Copyright (c) Texas Instruments Incorporated 2022
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
#if defined(SOC_AM273X) || defined(SOC_AWR294X)
#include <sdl/esm/v1/sdl_esm.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

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
 * \brief This enumerator indicate ECC memory type
 *
 * ----------------------------------------------------------------------------
 */
typedef uint32_t SDL_ECC_MemType;

#if defined(SOC_AM263X)

#define SDL_SOC_ECC_AGGR                                            (0)
#define SDL_R5FSS0_CORE0_ECC_AGGR                                   (1)
#define SDL_R5FSS0_CORE1_ECC_AGGR                                   (2)
#define SDL_R5FSS1_CORE0_ECC_AGGR                                   (3)
#define SDL_R5FSS1_CORE1_ECC_AGGR                                   (4)
#define SDL_HSM_ECC_AGGR                                            (5)
#define SDL_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR                         (6)
#define SDL_MCAN0_MCANSS_MSGMEM_WRAP_ECC_AGGR                       (7)
#define SDL_MCAN1_MCANSS_MSGMEM_WRAP_ECC_AGGR                       (8)
#define SDL_MCAN2_MCANSS_MSGMEM_WRAP_ECC_AGGR                       (9)
#define SDL_MCAN3_MCANSS_MSGMEM_WRAP_ECC_AGGR                       (10)
#define SDL_CPSW3GCSS_ECC_AGGR                                      (11)
#define SDL_ECC_MEMTYPE_MAX                                         (SDL_CPSW3GCSS_ECC_AGGR + 1U)
#endif

#if defined(SOC_AM273X) || defined(SOC_AWR294X)

#define SDL_R5FSS0_CORE0_ECC_AGGR                                   (0)
#define SDL_R5FSS0_CORE1_ECC_AGGR                                   (1)
#define SDL_MSS_ECC_AGG_MSS                                         (2)
#define SDL_DSS_ECC_AGG                                             (3)
#define SDL_MSS_MCANA_ECC                                           (4)
#define SDL_MSS_MCANB_ECC                                           (5)
#define SDL_ECC_MEMTYPE_MAX                                         (SDL_MSS_MCANB_ECC + 1U)

#endif
/* The following are the memory sub type for Memory type
   SDL_ECC_MEMTYPE_MCU_R5F0_CORE & SDL_ECC_MEMTYPE_MCU_R5F1_CORE */
/* Keeping for backward-compatibility. Recommend to use RAM_ID directly from sdlr_soc_ecc_aggr.h file */

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


/** ---------------------------------------------------------------------------
 * \brief This enumerator indicate ECC memory Sub Type
 *
 * ----------------------------------------------------------------------------
 */
typedef uint32_t SDL_ECC_MemSubType;

/** /brief Format of ECC error Call back function */
typedef void (*SDL_ECC_ErrorCallback_t) (uint32_t errorSrc, uint32_t address);

/** /brief Format of VIM DED vector function */
typedef void (*SDL_ECC_VIMDEDVector_t) (void);


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

#ifdef __cplusplus
}
#endif  /* extern "C" */

#endif
