/*
 * SDL ESM
 *
 * Software Diagnostics Reference module for Error Signaling Module
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
 *  @addtogroup SDL_ESM_MODULE APIs for SDL ESM
 *  @{
 */

/**
 * \file  sdl_esm.h
 *
 * \brief
 *  Header file contains enumerations, structure definitions and function
 *  declarations for SDL Error Signaling Module interface.
 *
 */

#ifndef ESM_H_
#define ESM_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>
#include <sdl/include/sdl_types.h>
#include <sdl/include/hw_types.h>
#include <sdl/esm/sdlr_esm.h>
#include <sdl/esm/v1/v1_0/sdlr_esm.h>
#include <sdl/esm/v1/esm.h>
#include <sdl/dpl/sdl_dpl.h>
#include <sdl/esm/soc/sdl_esm_soc.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/** \brief ESM Module error base */
#define SDL_ESM_ERRNO_BASE               (-3000)

/** \brief Invalid interrupt number */
#define SDL_ESM_INTNUMBER_INVALID (0xffffffffu)

/** \brief Invalid Esm Instance */
#define SDL_ESM_INST_INVALID      (0xfffffffeu)

/** \brief Address field: Error Address invalid */
#define SDL_ESM_ERRORADDR_INVALID (0xffffffffu)

/** \brief Address field: Error RAM ID invalid */
#define SDL_ESM_ERRORRAMID_INVALID (0xffffffffu)

/** \brief Address field: Error Bit Error Offset invalid */
#define SDL_ESM_ERRORBITOFFSET_INVALID (0xffffffffffffffffu)

/** \brief Address field: Error Bit Error Group invalid */
#define SDL_ESM_ERRORBITGROUP_INVALID (0xffffffffu)

/** \brief Maximum number of EVENT words */
#define SDL_ESM_MAX_EVENT_MAP_NUM_WORDS (32u)

/** 
*   @defgroup ESM_DRIVER_ERROR_CODE    ESM Driver Error Codes
*   @ingroup DRV_ESM_MODULE
*   @{ 
*/
/** \brief   Error Code: Invalid argument  */
#define SDL_ESM_EINVAL                 (SDL_ESM_ERRNO_BASE-1)
/** \brief   Error Code: Operation cannot be done as SDL_ESM_Init is not done */
#define SDL_ESM_ENOINIT                (SDL_ESM_ERRNO_BASE-2)
/** \brief   Error Code: Operation cannot be done as SDL_ESM_Init is already done and re-init is not permitted */
#define SDL_ESM_EREINIT                (SDL_ESM_ERRNO_BASE-3)
/** \brief   Error Code: Out of memory */
#define SDL_ESM_ENOMEM                 (SDL_ESM_ERRNO_BASE-4)
/** @}*/

/** \brief Maximum number of notify callbacks supported */
#define SDL_ESM_MAX_NOTIFIERS           (256U)
/** \brief Maximum ISR supported */
#define SDL_ESM_MAX_ISR_COUNT           (256U)

 #define CONFIG_ESM0           (0U)

typedef int32_t SDL_Result;

/**
 *  \anchor sdlEsmEccErrorSource_t
 *  \name ESM ECC Error source type
 *  @{
 */
/**
 * \brief  Source of the ECC error which maps to the ESM interrupt source
 */
typedef uint32_t sdlEsmEccErrorSource_t;

/** \brief MCU CPU0 detected 1-bit ECC error source */
#define SDL_ESM_ECC_PARAM_MCU_CPU0_SEC_ERROR (1u)
/** \brief MCU CPU0 detected 2-bit ECC error source */
#define SDL_ESM_ECC_PARAM_MCU_CPU0_DED_ERROR (2u)
/** \brief MCU CPU1 detected 1-bit ECC error source */
#define SDL_ESM_ECC_PARAM_MCU_CPU1_SEC_ERROR (3u)
/** \brief MCU CPU1 detected 2-bit ECC error source */
#define SDL_ESM_ECC_PARAM_MCU_CPU1_DED_ERROR (4u)
/** \brief MCU CBASS detected 1-bit ECC error source */
#define SDL_ESM_ECC_PARAM_MCU_CBASS_SEC_ERROR (5u)
/** \brief MCU CBASS detected 2-bit ECC error source */
#define SDL_ESM_ECC_PARAM_MCU_CBASS_DED_ERROR (6u)
/** @} */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 * \brief   Callback function which is invoked by the ESM module if a notify
 *          function is registered using the SDL_ESM_Init() API.
 *
 * @param   instance	: 	ESM Instance
 * @param	grpChannel	:   ESM group channel
 * @param   vecNum      :   Interrupt vector number
 * @param	arg			:  	Argument passed back when the Notify function is invoked.
 *			
 *     
 * @retval
 *          Not applicable
 */
 typedef int32_t (* SDL_ESM_CallBack) (SDL_ESM_Inst instance, int32_t grpChannel, int32_t vecNum, void *arg);
 
/** \brief  Structure to access the status of interrupts belonging to a group.*/
typedef struct 
{
    uint32_t grpIntrStatus[SDL_ESM_NUM_INTR_PER_GROUP / BITS_PER_WORD];
    /**< Contains status for interrupts from a group */
}ESM_GroupIntrStatus;

/** ---------------------------------------------------------------------------
 * \brief ESM error configuration
 *
 * This structure defines the elements ESM error configuration
 * ----------------------------------------------------------------------------
 */
typedef struct SDL_ESM_Errorconfig_s
{
    uint32_t groupNumber;
    /**< Group number of error event  */
    uint32_t eventNumber;
    /**< Bit number within the group  */
} SDL_ESM_ErrorConfig_t;

/**
 * \brief
 *  Parameters used to register the ESM notify function to handle
 *  Group1 and Group2 errors. The notify function will be invoked post
 *  the ESM FIQ interrupt handler processing.
 */
typedef struct 
{
    uint32_t            groupNumber;
    /**< Group number to which the following error number belongs. */
    uint32_t            errorNumber;
    /**< Error number for which the notify function is registered. */
    uint32_t            setIntrPriorityLvl;
    /**< Set the interrupt priority level to high or low. Applicable to Group 1 errors only. */
    bool                enableInfluenceOnErrPin;
    /**< Enable failure influence on ERROR pin. Applicable to Group 1 errors only. */
    void                *arg;
    /**< Argument passed back when the Notify function is invoked. */
    SDL_ESM_CallBack        callBackFunction;
    /**< Notify function called by the ESM Module. */
	SDL_ESM_ErrorConfig_t esmErrorConfig;
	/**< Error event to be used for self test */
} SDL_ESM_NotifyParams;

/**
 *  \brief ESM Open Parameters
 *
 *  ESM Open Parameters are used with the SDL_ESM_Init() call. Default values for
 *  these parameters are set using #SDL_ESM_Params_init().
 *
 *  If NULL is passed for the parameters, SDL_ESM_Init() uses default parameters.
 *
 *  \sa #SDL_ESM_Params_init()
 */
typedef struct
{
    bool bClearErrors;
    /**<  boolean value to indicate if old ESM pending errors should be cleared or not
          This field will be set by SysCfg.        
          value = 0: do not clear
          value = 1: clear all ESM group errors
          hint: If you using TI RTOS, then ESM errors are cleared before
          entering main and this flag can be set to 0. For any other RTOS, check the
          the RTOS implementation or set this flag to 1
    */
} SDL_ESM_OpenParams;

/**
 * \brief
 *  ESM Hardware Attributes
 *
 *  The structure is used to store the hardware specific configuration which is
 *  passed to the Module instance
 *
 *  ESM parameters are used with the SDL_ESM_Init() call.
 */
typedef struct {
	SDL_ESM_ErrorConfig_t esmErrorConfig;
	/**< Error event to be used for self test */
    SDL_esmRegs             *ptrESMRegs;
    /**< ESM Peripheral's base address for the control register space */
    SDL_mss_ctrlRegs        *ptrCtrlRegs;
    /**< SDL MSS control register base address  */
    uint32_t                numGroup1Err; 
    /**< ESM Peripheral's supported number of group1 errors */    
    uint32_t                highPrioIntNum;
    /**< ESM Peripheral's interrupt vector for high priority line */    
    uint32_t                lowPrioIntNum;
    /**< ESM Peripheral's interrupt vector for low priority line */
    uint8_t                 intrHighPriority;
    /**< Interrupt High priority */    
    uint8_t                 intrLowPriority;
    /**< Interrupt Low priority */
} SDL_ESM_Params;

/**
 * \brief
 *  ESM Module Object
 *
 *  The structure is used to hold all the pertinent information with respect
 *  to the ESM Module.
 */
typedef struct 
{
    volatile bool selfTestFlag;
	/**< selfTest Flag */
    uint32_t                    esmBaseAddr;
    /**<   Base address of the ESM address space to be used */
    SDL_ESM_OpenParams              params;
    /**<   ESM Module parameters */
    SDL_ESM_Handle                  esmHandle;
    /**< Instance handle to which this object belongs */
    SDL_ESM_NotifyParams            notifyParams[SDL_ESM_MAX_NOTIFIERS];
    /**<   Registered notify function. Currently only upto 4 notify functions are supported */
    uint32_t                    numGroup1Err;
    /**<    The number of Group1 errors supported */
    void *eccCallBackFunctionArg[SDL_ESM_MAX_ISR_COUNT];
     /**< Store the ECC callback function arg */
     void *ccmCallBackFunctionArg[SDL_ESM_MAX_ISR_COUNT];
    /**< Store the CCM callback function arg */
    SDL_ESM_CallBack eccCallBackFunction[SDL_ESM_MAX_ISR_COUNT];
    /**< Store the ECC callback function */
    SDL_ESM_CallBack ccmCallBackFunction[SDL_ESM_MAX_ISR_COUNT];
	/**< Store the CCM callback function */
    uint32_t eccenableEventBitmap[SDL_ESM_MAX_ISR_COUNT];
    uint32_t ccmenableBitmap[SDL_ESM_MAX_ISR_COUNT];
    /**< Store ECC Event */
    uint32_t                    debugEsmISRCount[SDL_ESM_MAX_ISR_COUNT];
    /**<   DEBUG: to keep track of various ESM interrupts received by the system */
} SDL_ESM_Object;

/**
 *  \brief      ESM Instance configuration
 *
 *  The SDL_ESM_Config structure contains a set of pointers used to
 *  characterize the ESM Module implementation.
 */
typedef struct  {
    SDL_ESM_Params      *esmConfig;
    /**< Pointer to Module specific attributes */
    SDL_ESM_Object           *object;
    /**< Pointer to Module specific data object */
} SDL_ESM_Config;

/*
 * ESM
 */
/* ESM attributes */
static SDL_ESM_Params gEsmParams[SDL_ESM_INST_DSS_ESM] =
{
    {
        .ptrESMRegs             = (SDL_esmRegs *)SDL_MSS_ESM_U_BASE,
        .ptrCtrlRegs            = (SDL_mss_ctrlRegs *)SDL_MSS_CTRL_U_BASE,
        .numGroup1Err           = SDL_ESM_NUM_INTR_PER_GROUP,
        .highPrioIntNum         = SDL_MSS_ESM_HI_INTNO,
        .lowPrioIntNum          = SDL_MSS_ESM_LOW_INTNO,
        .intrHighPriority       = SDL_ESM_EN_KEY_ENBALE_VAL,
        .intrLowPriority        = 8U,
    },
	
	{
        .ptrESMRegs             = (SDL_esmRegs *)SDL_DSS_ESM_U_BASE,
        .ptrCtrlRegs            = (SDL_mss_ctrlRegs *)SDL_DSS_CTRL_U_BASE,
        .numGroup1Err           = SDL_ESM_NUM_INTR_PER_GROUP,
        .highPrioIntNum         = SDL_DSS_ESM_HI_INTNO,
        .lowPrioIntNum          = SDL_DSS_ESM_LOW_INTNO,
        .intrHighPriority       = SDL_ESM_EN_KEY_ENBALE_VAL,
        .intrLowPriority        = 1U,
    },
};

/* ESM objects - initialized by the Module */
static SDL_ESM_Object gEsmObjects[CONFIG_ESM0];

/* ESM Module handles */
extern SDL_ESM_Handle gEsmHandle;

/* ESM Module configuration */
static SDL_ESM_Config gEsmConfig[SDL_ESM_INST_DSS_ESM] =
{
    {
        (&gEsmParams[SDL_ESM_INST_MSS_ESM -1U]),
        &gEsmObjects[CONFIG_ESM0],
    },
	{
        (&gEsmParams[SDL_ESM_INST_DSS_ESM -1U]),
        &gEsmObjects[CONFIG_ESM0],
    },
};

/* ========================================================================== */
/*                           enums                                            */
/* ========================================================================== */
/** ---------------------------------------------------------------------------
 * \brief This enumerator defines the values for ESM interrupt types
 * ----------------------------------------------------------------------------
 */

typedef enum {
   SDL_ESM_INT_TYPE_HI = 1,
    /**< Interrupt type Hi  */
   SDL_ESM_INT_TYPE_LO = 2,
    /**< Interrupt type Lo  */
   SDL_ESM_INT_TYPE_CFG = 3,
    /**< Interrupt type Config  */
   SDL_ESM_INT_TYPE_MAX = 4,
   /**< Interrupt type Max  */
} SDL_ESM_IntType;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/** ============================================================================
 *
 * \brief   SDL ESM API to get the status of the nError pin for the specified
 *          ESM instance
 *
 * \param   instance: ESM Instance
 * \param   pStatus: Pointer to variable to store the status.
 *                   If status is 1, then error pin is not active.
 *                   If status is 0, then error pin is active.
 *
 * \return  SDL_PASS if nError pin status is successfully retrieved.
 *          SDL_EBADARGS if instance or pStatus pointer are invalid.
 *          SDL_EFAIL if fail to read the error pin.
 */
int32_t SDL_ESM_getNErrorStatus(SDL_ESM_Inst instance, uint32_t *pStatus);

/** ============================================================================
 *
 * \brief   SDL ESM API to read the static registers. The API reads and returns
 *          the static register configuration for the ESM module for the specified
 *          instance. This API can be used by the application to read back the
 *          static configuration. Comparision of the static configuration registers
 *          against expected values is the responsibility of the application.
 *
 * \param   instance: ESM Instance
 * \param   pStaticRegs: Pointer to the static config register structure
 *
 * \return  SDL_PASS if registers are successfully read.
 *          SDL_EBADARGS if instance or pStaticRegs are invalid.
 */
int32_t SDL_ESM_getStaticRegisters(SDL_ESM_Inst instance, SDL_ESM_staticRegs *pStaticRegs);

/** ============================================================================
 *
 * \brief   SDL ESM API to clear the nError pin for the specified ESM instance
 *
 * \param   instance: ESM Instance
 *
 * \return  SDL_PASS if nError pin status is successfully cleared.
 *          SDL_EBADARGS if instance is invalid.
 */
int32_t SDL_ESM_clrNError(SDL_ESM_Inst instance);

/** ============================================================================
 *
 * \brief   SDL ESM API to set the nError pin for the specified ESM instance
 *
 * \param   instance: ESM Instance
 *
 * \return  SDL_PASS if nError pin status is successfully set.
 *          SDL_EBADARGS if instance is invalid.
 */
int32_t SDL_ESM_setNError(SDL_ESM_Inst instance);

/** ============================================================================
 *
 * \brief   SDL ESM API to initialize an ESM instance. The API initializes the
 *          specified ESM instance with the provided configuration. The
 *          configuration will allow the application to specify for each event
 *          whether the interrupt is enabled or disabled, the priority of the
 *          event, and whether the nErrorPin assertion is enabled or disabled for
 *          the event.
 *
 * \param   esmInstType		: ESM Instance type
 * \param   params			: Pointer to an Notify parameter ,
 * \param   esmOpenParams	: Pointer to an parameter block, if NULL it will use
 *                        	  default values.
 * \param   arg				: Application argument that will passed to the application when
 *                  		 the application callback is called.
 *
 * \return  SDL_PASS if success.
 *          SDL_EBADARGS if invalid argument is passed.
 *          SDL_EFAIL if other failure.
 */

SDL_Result SDL_ESM_init (const SDL_ESM_Inst esmInstType,
                         SDL_ESM_NotifyParams* params,
						 SDL_ESM_OpenParams *esmOpenParams,
                         void *arg);

/** ============================================================================
 *
 * \brief  There are modules within SDL which will generate ESM errors
 *         intentionally in the course of running self-tests. The ECC module is
 *         one such module. To allow these modules to get the notification when
 *         the ESM error occurs, callback registration APIs are provided. The
 *         following APIs allow registration of a callback for specific events.
 *         This API is used by other SDL modules and not by the application
 *
 * \param   esmInstType	: ESM Instance Type
 * \param   eccEvent	: Bitmap for ESM error event of interest for this callback.
 *                       Array of uint32_t type with each bit representing one
 *                       ESM error event.
 * \param   callBack	: Pointer to the callback to be called by the ESM Handler
 *                       to notify the ECC module of an ESM error event
 * \param   callbackArg	: Argument that will be passed along with the callback.
 *
 * \return  SDL_PASS if success.
 *          SDL_EBADARGS if invalid argument is passed.
 *          SDL_EFAIL if other failure.
 */
int32_t SDL_ESM_registerECCCallback(SDL_ESM_Inst esmInstType,uint32_t eccEvent,
                                    SDL_ESM_CallBack callBack,
                                    void *callbackArg);
									
/** ============================================================================
 *
 * \brief  There are modules within SDL which will generate ESM errors
 *         intentionally in the course of running self-tests. The CCM module is
 *         one such module. To allow these modules to get the notification when
 *         the ESM error occurs, callback registration APIs are provided. The
 *         following APIs allow registration of a callback for specific events.
 *         This API is used by other SDL modules and not by the application
 *
 * \param   esmInstType	: ESM Instance Type
 * \param   eccEvent	: Bitmap for ESM error event of interest for this callback.
 *                       Array of uint32_t type with each bit representing one
 *                       ESM error event.
 * \param   callBack	: Pointer to the callback to be called by the ESM Handler
 *                       to notify the CCM module of an ESM error event
 * \param   callbackArg	: Argument that will be passed along with the callback.
 *
 * \return  SDL_PASS if success.
 *          SDL_EBADARGS if invalid argument is passed.
 *          SDL_EFAIL if other failure.
 */
int32_t SDL_ESM_registerCCMCallback(SDL_ESM_Inst esmInstType,uint32_t eccEvent,
                                    SDL_ESM_CallBack callBack,
                                    void *callbackArg);

/** ============================================================================
 *
 * \brief   SDL ESM API to verify the written configuration of the ESM module.
 *          The API verifies the written config that was done during SDL_ESM_init
 *          against the provided configuration.
 *
 * \param   instance: ESM Instance
 * \param   params: Pointer to the ESM configuration to be used for verification.
 *
 * \return  SDL_PASS if Verification passed
 *          SDL_EBADARGS if instance or pConfig are invalid.
 *          SDL_EFAIL if verification failed.
 */
int32_t SDL_ESM_verifyConfig(SDL_ESM_Inst instance, SDL_ESM_NotifyParams* params);
									
/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */
static inline void SDL_ESM_Params_init(SDL_ESM_OpenParams *openPrms)
{
    if (openPrms != NULL)
    {
        openPrms->bClearErrors  = FALSE;
    }
}

#ifdef __cplusplus
}
#endif

#endif /* #ifndef ESM_H_ */

/** @} */
