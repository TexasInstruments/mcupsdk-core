/*
 *  Copyright (c) 2022-2024 Texas Instruments Incorporated
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
 *  @ingroup SDL_CCM_MODULE
 *  @defgroup SDL_IP_CCM_API CCM API
 *
 *  This module contains the APIs to program and use the CCM module.
 *
 *  @{
 */

/**
 * \file  sdl_ip_ccm.h
 *
 * \brief Header file contains enumerations, structure definitions and function
 *        declarations for SDL CCM interface.
 *
 */

#ifndef SDL_IP_CCM_H_
#define SDL_IP_CCM_H_

#include <stddef.h>
#include <stdbool.h>
#include "sdl/sdl_ccm.h"
#include "sdl/r5/v0/sdlr_vim.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                            Macros                                          */
/* ========================================================================== */
/** This enumerator defines the maximum interrupt groups in VIM.*/
#define SDL_VIM_MAX_INTR_GROUPS     (32U)
/** This enumerator defines the number of interrupts per group in VIM. */
#define SDL_VIM_NUM_INTRS_PER_GROUP (32U)
/* ========================================================================== */
/*                             Enums & Typedefs                                 */
/* ========================================================================== */

/**
 * \brief This enumerator defines the type of CCM Monitor type
 *
 */
typedef enum {
    SDL_CCM_MONITOR_TYPE_NONE = 0,
    /**< CCM Monitor None  */
    SDL_CCM_MONITOR_TYPE_OUTPUT_COMPARE_BLOCK = 1,
    /**< CCM output compare block monitor type */
    SDL_CCM_MONITOR_TYPE_VIM = 2,
    /**< CCM VIM block monitor type */
#if defined (SOC_AM263PX) || (SOC_AM261X)
    SDL_CCM_MONITOR_TYPE_TMU = 3,
    /**< CCM TMU block monitor type */
    SDL_CCM_MONITOR_TYPE_RL2 = 4,
    /**< CCM RL2 block monitor type */
    SDL_CCM_MONITOR_TYPE_INACTIVITY_MONITOR = 5,
    /**< CCM inacitivty monitor type */
    SDL_CCM_MONITOR_TYPE_INVALID = 6,
    /**< CCM invalid type */
#else
    SDL_CCM_MONITOR_TYPE_INACTIVITY_MONITOR = 3,
    /**< CCM inacitivty monitor type */
    SDL_CCM_MONITOR_TYPE_INVALID = 4,
    /**< CCM invalid type */
#endif
} SDL_CCM_MonitorType;

/**
 * \brief This enumerator defines the type of CCM Self-test type
 *
 */
typedef enum {
    SDL_CCM_SELFTEST_TYPE_NORMAL = 1,
    /**< Self-test for a particular monitor type */
    SDL_CCM_SELFTEST_TYPE_ERROR_FORCING = 2,
    /**< Error-forcing self-test for a particular monitor type */
    SDL_CCM_SELFTEST_POLARITY_INVERSION = 3,
    /**< Polarity inversion self-test for a particular monitor type */
    SDL_CCM_SELFTEST_TYPE_INVALID = 4,
    /**< CCM invalid type */
} SDL_CCM_SelfTestType;


/**
 * \brief CCM Static Register
 *
 * This is the structure that is used to return the static registers in the
 * static registers readback function
 */
typedef struct {
	volatile uint32_t CCMKEYR1;          /* MKEY1 value */
	volatile uint32_t CCMKEYR2;          /* MKEY2 value */
	volatile uint32_t CCMKEYR3;          /* MKEY3 value */
	volatile uint32_t CCMPOLCNTRL;       /* POL_INV value */
#if defined (SOC_AM263PX) || (SOC_AM261X)
	volatile uint32_t CCMKEYR5;          /* MKEY5 value */
	volatile uint32_t CCMKEYR6;          /* MKEY6 value */
#endif
} SDL_CCM_staticRegs;

/**
 * \brief Structure for CCM error status
 *
 */
typedef struct SDL_CCM_ErrorStatus_s
{
    bool compareErrorFlag;
    /**< CCM compare error flag: true indicates error */
    bool selfTestErrorFlag;
    /**< CCM self test error flag: true indicates error */
    bool selfTestErrorTypeFlag;
    /**< CCM self test error type flag: true indicates error */

} SDL_CCM_ErrorStatus_t;

/**
 * @brief This enumerator defines the possible output interrupt types. Each
 *        source interrupt can be configured to drive either the IRQ or FIQ
 *        output signal.
 *
 *
 */
typedef uint32_t SDL_VimIntrMap;

/** Output interrupt is mapped to IRQ  */
#define SDL_VIM_INTR_MAP_IRQ    ((uint32_t)0U)

/** Output interrupt is mapped to FIQ  */
#define SDL_VIM_INTR_MAP_FIQ    ((uint32_t)1U)

/** Output interrupt is mapped to IRQ or FIQ */
#define SDL_VIM_INTR_MAP_ALL    ((uint32_t)2U)

/**
 * @brief This enumerator defines the possible interrupt types. Each source
 *        interrupt is either an active high level or active high pulse.
 *
 *
 */
typedef uint32_t SDL_VimIntrType;

/** Interrupt source is a level interrupt type */
#define SDL_VIM_INTR_TYPE_LEVEL ((uint32_t)0U)

/** Interrupt source is a pulse interrupt type */
#define SDL_VIM_INTR_TYPE_PULSE ((uint32_t)1U)

/**
 * @brief This structure defines the group registers identified for VIM static
 * registers
 *
 *
 */
typedef struct {
    volatile uint32_t INTR_EN_SET;               /** Interrupt Enable Set Register */
    volatile uint32_t INTR_EN_CLR;               /** Interrupt Enabled Clear Register */
    volatile uint32_t INTMAP;                    /** Interrupt Map Register */
    volatile uint32_t INTTYPE;                   /** Interrupt Type Register */
} SDL_vimStaticRegs_GRP;
/**
  * @brief This structure defines the static register groups identified for VIM
 *
 *
 */
typedef struct {
    SDL_vimStaticRegs_GRP GRP[SDL_VIM_MAX_INTR_GROUPS]; /** Group Registers */
    SDL_vimRegs_PRI PRI[SDL_VIM_MAX_INTR_GROUPS*SDL_VIM_NUM_INTRS_PER_GROUP]; /** Priority Regs */
    SDL_vimRegs_VEC VEC[SDL_VIM_MAX_INTR_GROUPS*SDL_VIM_NUM_INTRS_PER_GROUP]; /** Vec addr regs */
    volatile uint32_t PID;                       /** Revision Register */
    volatile uint32_t INFO;                      /** Info Register */
    volatile uint32_t IRQVEC;                    /** IRQ Vector Address Register */
    volatile uint32_t FIQVEC;                    /** FIQ Vector Address Register */
    volatile uint32_t DEDVEC;                    /** DED Vector Address Register */
} SDL_vimStaticRegs;

/* ========================================================================== */
/*                            Function Declarations                               */
/* ========================================================================== */

/**
 *
 * \brief   Initialization API for CCM module
 *
 * \param   instance                [IN] CCM Instance
 *
 * \param   index                   [IN] bit map array index value
 * \return The SDL error code for the API
 *                                   SDL_PASS : Success;
 *                                   SDL_EFAIL : if error happened during initialization
 */
int32_t SDL_CCM_init(SDL_CCM_Inst instance, uint32_t index);

/**
 *
 * \brief   Verifies the configuration done as part of SDL_CCM_init is as expected
 *
 * \param   instance                [IN] CCM Instance
 *
 * \return The SDL error code for the API
 *                                   SDL_PASS : Success;
 *                                   SDL_EFAIL : Verification failed
 */
 int32_t SDL_CCM_verifyConfig(SDL_CCM_Inst instance);

 /**
 *
 * \brief   Executes a self-test of the CCM module. The types of self-tests supported
 *          are described in detail in SDL_CCM_SelfTestType
 *
 * \param   instance                [IN] CCM Instance
 *
 * \param   monitorType             [IN] Monitor type for which to run the self-test
 *
 * \param   testType                [IN] Test type to run for the selected monitor type.
 *
 * \param   polarityInversionMask   [IN] Mask used to invert polarity of selected signals.
 *                                        For meaning of signals, refer to hardware manuals.
 *                                        This parameter is used only for the polarity inversion
 *                                        self-test type
 *
 * \param   timeoutCnt              [IN] Number of times to check for self-test completion
 *                                       in a loop before timing out
 *
 * \return The SDL error code for the API
 *                                   SDL_PASS     : Self-test passed
 *                                   SDL_EBADARGS : Returned if input paramter is invalid
 *                                   SDL_EFAIL    : Self-test failed
 */
 int32_t SDL_CCM_selfTest(SDL_CCM_Inst instance, SDL_CCM_MonitorType monitorType, SDL_CCM_SelfTestType testType, \
							uint32_t polarityInversionMask, uint32_t timeoutCnt);

 /**
 *
 * \brief   Forces an error to be generated for the selected monitor type. After injecting the error,
 *          an ESM error event is generated and the application will receive notification via
 *          ESM application callback.
 *
 * \param   instance                [IN] CCM Instance
 *
 * \param   monitorType             [IN] Monitor type for which to inject the error
 *
 * \return The SDL error code for the API
 *                                   SDL_PASS     : Error injection passed
 *                                   SDL_EBADARGS : Returned if input paramter is invalid
 *                                   SDL_EFAIL    : Error injection failed
 */
 int32_t SDL_CCM_injectError(SDL_CCM_Inst instance, SDL_CCM_MonitorType monitorType);

 /**
 *
 * \brief   Retrieves the static register configuration. The values returned in staticRegs
 *          can be saved by the application and periodically checked for changes.
 *          Note: This API should not be called while running any self-test or during error
 *          injection, as this will change the values of the static registers temporarily.
 *          It should be called only when in normal operating mode.
 *
 * \param   instance                [IN] CCM Instance
 *
 * \param   pStaticRegs             [OUT] Pointer to the variable to store the static register
 *                                        information
 *
 * \return The SDL error code for the API
 *                                   SDL_PASS     : Static registers successfully read
 *                                   SDL_EBADARGS : Invalid pointer passed
 */
 int32_t SDL_CCM_getStaticRegisters(SDL_CCM_Inst instance, SDL_CCM_staticRegs *pStaticRegs);

 /**
 *
 * \brief   Gets the monitor type for which the CCM error was generated. Takes the
 *          ESM interrupt source as input
 *
 * \param   instance                [IN] CCM Instance
 *
 * \param   intSrc                  [IN] ESM interrupt source used to check for monitorType
 *                                       that generated the error
 *
 * \param   monitorType             [OUT] Monitor type which generated the error
 *
 * \return The SDL error code for the API
 *                                   SDL_PASS     : Monitor type retrieved
 *                                   SDL_EBADARGS : Invalid parameter
 */
 int32_t SDL_CCM_getErrorType(SDL_CCM_Inst instance, uint32_t intSrc, SDL_CCM_MonitorType *monitorType);

 /**
 *
 * \brief   Clears the compare error for the selected monitor type
 *
 * \param   instance                [IN] CCM Instance
 *
 * \param   monitorType             [IN] Monitor type for which to clear the error
 *
 * \return The SDL error code for the API
 *                                   SDL_PASS     : Event cleared
 *                                   SDL_EBADARGS : Invalid parameter
 *                                   SDL_EFAIL    : Failed to clear the error
 */
 int32_t SDL_CCM_clearError(SDL_CCM_Inst instance, SDL_CCM_MonitorType monitorType);

/**
 *  \brief Configure a source interrupt using VIM
 *
 *  This function configures the specified source interrupt per the provided
 *  parameters.
 *
 *  \param pRegs        [IN]    Pointer to the SDL_vimRegs register structure
 *  \param intrNum      [IN]    Interrupt number
 *  \param pri          [IN]    Interrupt priority (0(highest)..15(lowest))
 *  \param intrMap      [IN]    Interrupt mapping
 *  \param intrType     [IN]    Interrupt type
 *  \param vecAddr      [IN]    32-bit vector address (forced to 32-bit alignment)
 *
 *  \return  0 = success
 *          -1 = intrNum or pri parameters are out of range
 */
int32_t SDL_VIM_cfgIntr(SDL_vimRegs *pRegs, uint32_t intrNum, uint32_t pri, SDL_VimIntrMap intrMap,
						SDL_VimIntrType intrType, uint32_t vecAddr );
/**
 *  \brief Read back Verify Configure a source interrupt
 *
 *  This function verifies the writetn configuration of
 *  interrupt for the specified source interrupt per the provided
 *  parameters.
 *
 *  \param pRegs        [IN]    Pointer to the SDL_vimRegs register structure
 *  \param intrNum      [IN]    Interrupt number
 *  \param pri          [IN]    Interrupt priority (0(highest)..15(lowest))
 *  \param intrMap      [IN]    Interrupt mapping
 *  \param intrType     [IN]    Interrupt type
 *  \param vecAddr      [IN]    32-bit vector address (forced to 32-bit alignment)
 *
 *  \return  0 = success (matches to expected values)
 *          -1 = VIM register configuration is not matching expected value
 */
int32_t SDL_VIM_verifyCfgIntr(SDL_vimRegs     *pRegs,
                                uint32_t         intrNum, uint32_t  pri,
                                SDL_VimIntrMap   intrMap,
                                SDL_VimIntrType  intrType,
                                uint32_t         vecAddr);
/**
 *  \brief Read back of static registers for VIM
 *
 *  The function is used to acknowlege that the specified interrupt type has been
 *  serviced. The VIM will then re-evaluate the new highest priority interrupt.
 *
 *  \param pRegs        [IN]    Pointer to the SDL_vimRegs register structure
 *  \param pStaticRegs  [IN]    Pointer to the SDL_vimStaticRegs structure to hold
 *                              the static register values
 *                              The static register set is defined to be the list
 *                              of registers that hardware does not update by itself
 *
 *  \return  0 = success
 *          -1 = failure (could not read back the static registers)
 */
int32_t SDL_VIM_getStaticRegs( SDL_vimRegs *pRegs, SDL_vimStaticRegs *pStaticRegs);
#ifdef __cplusplus
}
#endif /*extern "C" */
#endif

/** @} */
