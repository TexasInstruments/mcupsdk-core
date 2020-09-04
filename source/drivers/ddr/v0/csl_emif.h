/**
 * @file  csl_emif.h
 *
 * @brief
 *  Header file containing various enumerations, structure definitions and function
 *  declarations for the EMIF IP.
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2021, Texas Instruments, Inc.
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
#ifndef CSL_EMIF_H_
#define CSL_EMIF_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <drivers/ddr/v0/cslr_emif.h>

/** ===========================================================================
 *
 * @defgroup CSL_EMIF_API External Memory Interface API
 *
 *  This is the CSL-FL for the EMIF controller/PUB. It provides the structures
 *  and APIs needed to configure and initialize the EMIF controller/PUB.
 *  Once the EMIF is initialized, additional API functions are available
 *  to implement runtime operations
 *
 *  The following lists the steps required to use this CSL-FL:
 *
 *  1. Allocate and initialize a #CSL_EmifConfig structure. This structure
 *     provides all controller configuration information required to
 *     properly initialize them.
 *
 *  2. Call the #CSL_emifConfig function. This function uses configuration
 *     information provided in the #CSL_EmifConfig structure to initialize
 *     the controller.
 *
 *  3. Call the runtime functions to enable or disable ECC
 *
 *
 *  ===========================================================================
 */
/**
@defgroup CSL_EMIF_DATASTRUCT  EMIF Data Structures
@ingroup CSL_EMIF_API
*/
/**
@defgroup CSL_EMIF_FUNCTION  EMIF Functions
@ingroup CSL_EMIF_API
*/
/**
@defgroup CSL_EMIF_ENUM EMIF Enumerated Data Types
@ingroup CSL_EMIF_API
*/

/**
 *  \addtogroup CSL_EMIF_ENUM
 *  @{
 */

/** ---------------------------------------------------------------------------
 * \brief This enumerator defines the different ECC error types
 * ----------------------------------------------------------------------------
 */
typedef enum {
    CSL_EMIF_ECC_ERROR_TYPE_SINGLE_BIT = 1,
    /**<  Ecc error type single bit correctable */
    CSL_EMIF_ECC_ERROR_TYPE_DOUBLE_BIT = 2,
    /**<  Ecc error type Double bit non-crrectable */
} CSL_EmifECCErrorType;

/** ---------------------------------------------------------------------------
 * @brief This enumerator defines the possible return values from the
 * CSL_emifConfig function
 *
 *  \anchor CSL_EmifConfigReturn
 *  \name EMIF config function return value
 *
 * ----------------------------------------------------------------------------
 */
typedef int32_t CSL_EmifConfigReturn;
    /** Success */
#define CSL_EMIF_CONFIG_RETURN_OK                       ((int32_t) 0)
    /** An invalid value was detected in the specified configuration parameters */
#define CSL_EMIF_CONFIG_RETURN_INVALID_CFG              ((int32_t) -1)

/* @} */

/**
 *  \addtogroup CSL_EMIF_DATASTRUCT
 *  @{
 */

#define CSL_EMIF_ECC_MAX_REGIONS 3

/** \brief Emif ECC configuration structure
 *
 *  This structure provides configuration settings used for ECC configuration
 *
 */
typedef struct
{
    uintptr_t   startAddr[CSL_EMIF_ECC_MAX_REGIONS];   /**< [IN] Start address of ECC regions */
    uintptr_t   endAddr[CSL_EMIF_ECC_MAX_REGIONS];     /**< [IN] End address of ECC regions */
} CSL_EmifMemEccCfg;

/** \brief EMIF configuration structure
 *
 *  This structure provides the configuration settings for the EMIF.
 *  A pointer to this structure is passed to the #CSL_emifConfig function
 *  which performs the proper configuration sequence for the EMIF subsystem.
 *
 */
typedef struct csl_emif_config_t
{
    CSL_EmifMemEccCfg                   pMemEccCfg;             /**< [IN] ECC memory configuration */
    uint32_t                            ECCThreshold;           /**< [IN] ECC threshold */
    bool                                bWriteAlloc;            /**< [IN] Unassigned ECC cache line will be allocated for write */
    bool                                bECCCheck  ;            /**< [IN] Enable ECC verification on read access */
    bool                                bReadModifyWriteEnable; /**< [IN] Enable read modify write for sub-quanta accesses */   
    bool                                bEnableMemoryECC;       /**< [IN] Enable memory ECC */       
} CSL_EmifConfig;

/** \brief Emif ECC Error Information structure
 *
 *  This structure provides information about the ECC error
 *
 */
typedef struct
{
    uintptr_t   singlebitErrorAddress;   /**< One Bit ECC error address */
    uintptr_t   doublebitErrorAddress;   /**< Two Bit ECC error address */
    uint32_t    singlebitErrorCount;     /**< One Bit ECC error count   */
} CSL_ECCErrorInfo;
/* @} */

/**
 *  \addtogroup CSL_EMIF_FUNCTION
 *  @{
 */

/**
 *  \brief Return revision of the EMIF subsystem module.
 *
 *  This function returns the contents of the EMIF subsystem revision register.
 *  Consult the EMIF subsystem module documentation for a description of the
 *  contents of the revision register.
 *
 *  \param pEmifSsRegs      [IN]    Pointer to the EMIF Subsystem register space base.
 *
 *  \return The 32-bit revision register is returned.
 */
uint32_t CSL_emifGetRevision(CSL_emif_sscfgRegs *pEmifSsRegs);

/**
 *  \brief Configure EMIF
 *
 *  This function performs the proper configuration sequence for the SDRAM
 *  controller and PHY per the specified configuration parameters.
 *
 *  \param pEmifSsRegs      [IN]    Pointer to the EMIF Subsystem register space base.
 *  \param pCfg             [IN]    Pointer to the EMIF configuration structure. See #CSL_EmifConfig for details.
 *
 *  \return    0 = success
 *            -1 = An invalid value was detected in the specified configuration parameters
 */
int32_t CSL_emifConfig(CSL_emif_sscfgRegs *pEmifSsRegs, CSL_EmifConfig *pCfg);

/**
 *  \brief Enables ECC
 *
 *  This function enables ECC
 *
 *  \param pEmifSsRegs      [IN]    Pointer to the EMIF Subsystem register space base.
 *
 *  \return    0 = success
 *            -1 = An invalid value was detected in the specified configuration parameters
 */
int32_t CSL_emifEnableECC(CSL_emif_sscfgRegs *pEmifSsRegs);


/**
 *  \brief Disables ECC
 *
 *  This function disables ECC
 *
 *  \param pEmifSsRegs      [IN]    Pointer to the EMIF Subsystem register space base.
 *
 *  \return    0 = success
 *            -1 = An invalid value was detected in the specified configuration parameters
 */
int32_t CSL_emifDisableECC(CSL_emif_sscfgRegs *pEmifSsRegs);

/**
 *  \brief Get ECC error information
 *
 *  This function reads the registers and provides details of the last ECC error
 *
 *  \param pEmifSsRegs      [IN]    Pointer to the EMIF Subsystem register space base.
 *  \param pECCErrorInfo    [OUT]   Pointer to the EMIF ECC Error info structure.
 *
 *  \return    0 = success
 *            -1 = An invalid value was detected in the specified configuration parameters
 */
int32_t CSL_emifGetECCErrorInfo(CSL_emif_sscfgRegs *pEmifSsRegs,
                                       CSL_ECCErrorInfo *pECCErrorInfo);

/**
 *  \brief Clears all ECC errors
 *
 *  This function clears , both single bit and double bit errors in queue
 *
 *  \param pEmifSsRegs      [IN]    Pointer to the EMIF Subsystem register space base.
 *
 *  \return    0 = success
 *            -1 = An invalid value was detected in the specified configuration parameters
 */
int32_t CSL_emifClearAllECCErrors(CSL_emif_sscfgRegs *pEmifSsRegs);

/**
 *  \brief Clears ECC error
 *
 *  This function clear one  ECC error of specified type
 *
 *  \param pEmifSsRegs      [IN]    Pointer to the EMIF Subsystem register space base.
 *
 *  \return    0 = success
 *            -1 = An invalid value was detected in the specified configuration parameters
 */
int32_t CSL_emifClearECCError(CSL_emif_sscfgRegs *pEmifSsRegs,
                              CSL_EmifECCErrorType errorType);
/**
 *  \brief Enables ECC interrupts
 *
 *  This function enables ECC interrupts
 *
 *  \param pEmifSsRegs         [IN]    Pointer to the EMIF Subsystem register space base.
 *  \param eccInterruptBitmap  [IN]    ECC interrupt bitmap to enable interrupt
 *
 *  \return    0 = success
 *            -1 = An invalid value was detected in the specified configuration parameters
 */
int32_t CSL_emifEnableECCInterrupts(CSL_emif_sscfgRegs *pEmifSsRegs,
                                    uint32_t eccInterruptBitmap);

/**
 *  \brief Disables ECC interrupts
 *
 *  This function disables ECC interrupts
 *
 *  \param pEmifSsRegs         [IN]    Pointer to the EMIF Subsystem register space base.
 *  \param eccInterruptBitmap  [IN]    ECC interrupt bitmap to disable interrupt
 *
 *  \return    0 = success
 *            -1 = An invalid value was detected in the specified configuration parameters
 */
int32_t CSL_emifDisableECCInterrupts(CSL_emif_sscfgRegs *pEmifSsRegs,
                                     uint32_t eccInterruptBitmap);
/**
 *  \brief Clear ECC interrupt status
 *
 *  This function clears ECC interrupt status
 *
 *  \param pEmifSsRegs         [IN]    Pointer to the EMIF Subsystem register space base.
 *  \param eccInterruptBitmap  [IN]    ECC interrupt bitmap to clear interrupt status
 *
 *  \return    0 = success
 *            -1 = An invalid value was detected in the specified configuration parameters
 */
int32_t CSL_emifClearECCInterruptStatus(CSL_emif_sscfgRegs *pEmifSsRegs,
                                        uint32_t eccInterruptBitmap);

/* @} */

#ifdef __cplusplus
}
#endif  /* __cplusplus */

#endif  /* CSL_EMIF_H_ */
