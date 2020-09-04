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
 */

/**
 *  \defgroup DRV_ESM_MODULE APIs for ESM
 *  \ingroup DRV_MODULE
 *
 *  This module contains APIs to program and use the ESM module. The APIs
 *  can be used by other drivers to get access to ESM. 
 *
 *  @{
 */

/**
 *  \file v0/esm.h
 *
 *  \brief ESM Driver API/interface file.
 */

#ifndef ESM_H_
#define ESM_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>
#include <kernel/dpl/HwiP.h>
#include <drivers/hw_include/cslr_soc.h>
#include <drivers/hw_include/cslr_esm.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/** \brief A handle that is returned from a #ESM_open() call */
typedef void *ESM_Handle;

/** \brief ESM driver error base */
#define ESM_ERRNO_BASE               (-3000)

/** 
*   @defgroup ESM_DRIVER_ERROR_CODE    ESM Driver Error Codes
*   @ingroup DRV_ESM_MODULE
*   @{ 
*/
/** \brief   Error Code: Invalid argument  */
#define ESM_EINVAL                 (ESM_ERRNO_BASE-1)
/** \brief   Error Code: Operation cannot be done as ESM_open is not done */
#define ESM_ENOINIT                (ESM_ERRNO_BASE-2)
/** \brief   Error Code: Operation cannot be done as ESM_open is already done and re-init is not permitted */
#define ESM_EREINIT                (ESM_ERRNO_BASE-3)
/** \brief   Error Code: Out of memory */
#define ESM_ENOMEM                 (ESM_ERRNO_BASE-4)
/** @}*/

/** \brief Maximum number of notify callbacks supported */
#define ESM_MAX_NOTIFIERS           (4U)
/** \brief Maximum ISR supported */
#define ESM_MAX_ISR_COUNT           (4U)

/* ========================================================================== */
/*                           enums                                            */
/* ========================================================================== */

/** \brief  Enum to select level for interrupt. */
typedef enum 
{
    ESM_INTR_PRIORITY_LEVEL_LOW = 0x0U,
    /**< Configure interrupt to high level interrupt */
    ESM_INTR_PRIORITY_LEVEL_HIGH = 0x1U
    /**< Configure interrupt to low level interrupt */
}ESM_IntrPriorityLvl;

/** \brief  Enum to select the ESM Operation Mode. */
typedef enum 
{
    ESM_OPERATION_MODE_NORMAL = ESM_ESMEKR_EKEY_NORMAL_MODE,
    /**< Configure ESM operation mode to normal mode */
    ESM_OPERATION_MODE_ERROR_FORCE = ESM_ESMEKR_EKEY_ERROR_FORCE_MODE
    /**< Configure ESM operation mode to error force mode */
}ESM_OperationMode;

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 * \brief   Callback function which is invoked by the ESM module if a notify
 *          function is registered using the ESM_registerNotifier() API.
 *
 * @param   arg  Argument passed back when the Notify function is invoked.
 *     
 * @retval
 *          Not applicable
 */
typedef void (*ESM_CallBack)(void* arg);

/** \brief  Structure to access the status of interrupts belonging to a group.*/
typedef struct 
{
    uint32_t grpIntrStatus[ESM_NUM_INTR_PER_GROUP / 32U];
    /**< Contains status for interrupts from a group */
}ESM_GroupIntrStatus;

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
    Bool                enableInfluenceOnErrPin;
    /**< Enable failure influence on ERROR pin. Applicable to Group 1 errors only. */
    void                *arg;
    /**< Argument passed back when the Notify function is invoked. */
    ESM_CallBack        notify;
    /**< Notify function called by the ESM driver. */
} ESM_NotifyParams;

/**
 *  \brief ESM Open Parameters
 *
 *  ESM Open Parameters are used with the #ESM_open() call. Default values for
 *  these parameters are set using #ESM_Params_init().
 *
 *  If NULL is passed for the parameters, #ESM_open() uses default parameters.
 *
 *  \sa #ESM_Params_init()
 */
typedef struct
{
    Bool bClearErrors;
    /**<  boolean value to indicate if old ESM pending errors should be cleared or not
          This field will be set by SysCfg.        
          value = 0: do not clear
          value = 1: clear all ESM group errors
          hint: If you using TI RTOS, then ESM errors are cleared before
          entering main and this flag can be set to 0. For any other RTOS, check the
          the RTOS implementation or set this flag to 1
    */
} ESM_OpenParams;

/**
 * \brief
 *  ESM Hardware Atrributes
 *
 *  The structure is used to store the hardware specific configuration which is
 *  passed to the driver instance
 *
 *  ESM parameters are used with the ESM_open() call.
 */
typedef struct {
    CSL_esmRegs             *ptrESMRegs;
    /**< ESM Peripheral's base address for the control register space */
    CSL_mss_ctrlRegs        *ptrCtrlRegs;
    /**< CSL MSS control register base address  */
    uint32_t                numGroup1Err; 
    /**< ESM Peripheral's supported number of group1 errors */    
    uint32_t                highPrioIntNum;
    /**< ESM Peripheral's interrupt vector for high priority line */    
    uint32_t                lowPrioIntNum;
    /**< ESM Peripheral's interrupt vector for low priority line */
    uint8_t                 intrHighPriority;
    /**< Interrupt Low priority */    
    uint8_t                 intrLowPriority;
    /**< Interrupt Low priority */
} ESM_Attrs;

/**
 * \brief
 *  ESM Driver Object
 *
 *  The structure is used to hold all the pertinent information with respect
 *  to the ESM Driver.
 */
typedef struct 
{
    ESM_Handle                  esmHandle;
    /**< Instance handle to which this object belongs */
    uint32_t                    esmBaseAddr;
    /**<   Base address of the ESM address space to be used */
    ESM_OpenParams              params;
    /**<   ESM driver parameters */
    void                       *hwiHandleHi;
    /**< Interrupt handle for high priority ISR */ 
    HwiP_Object                 hwiHiObj;
    /**< Interrupt object for high priority ISR */
    void                       *hwiHandleLo;
    /**< Interrupt handle for low priority ISR */ 
    HwiP_Object                 hwiLoObj;
    /**< Interrupt object for low priority ISR */
    ESM_NotifyParams            notifyParams[ESM_MAX_NOTIFIERS];
    /**<   Registered notify function. Currently only upto 4 notify functions are supported */
    uint32_t                    numGroup1Err;
    /**<    The number of Group1 errors supported */
    uint32_t                    debugEsmISRCount[ESM_MAX_ISR_COUNT];
    /**<   DEBUG: to keep track of various ESM interrupts received by the system */
} ESM_Object;

/**
 *  \brief      ESM Instance configuration
 *
 *  The ESM_Config structure contains a set of pointers used to
 *  characterize the ESM driver implementation.
 */
typedef struct  {
    const ESM_Attrs      *hwAttrs;
    /**< Pointer to driver specific attributes */
    ESM_Object           *object;
    /**< Pointer to driver specific data object */
} ESM_Config;

/** \brief Externally defined driver configuration array */
extern ESM_Config            gEsmConfig[];
/** \brief Externally defined driver configuration array size */
extern uint32_t              gEsmConfigNum;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 *  \brief The functions initializes the ESM module.
 *
 */
void ESM_init(void);

/**
 *  \brief The functions de-initializes the ESM module.
 *
 */
void ESM_deinit(void);

/**
 *  \brief  Opens a ESM object with the index and parameters specified, and
 *          returns a ESM_Handle.
 *
 *  \param  index         Logical peripheral number for the ESM indexed
 *                        into the ESM_Config table
 *
 *  \param  params        Pointer to an parameter block, if NULL it will use
 *                        default values.
 *
 *  \return A ESM_Handle on success or a NULL on an error.
 * 
 */
ESM_Handle ESM_open(uint32_t index, ESM_OpenParams *params);

/**
 *  \brief  The function closes a ESM peripheral specified by the ESM
 *          handle. 
 * 
 *  @pre    ESM_open() has to be called first.
 *
 *  \param  handle      ESM Handle
 *
 */
void ESM_close(ESM_Handle handle);

/** 
*   \brief Register the notifers. The ESM module will call back if error interrupt is detected.
*
*   @param[in] handle: Handle to the ESM Driver.
*
*   @param[in] params: Notifier error number and callback function.
*
*   @param[out]  errCode
*      Error code populated on error.
*
*   @return On success returns SystemP_SUCCESS. Negative values indicate
*           unsuccessful operations.
*
*/
int32_t ESM_registerNotifier(ESM_Handle handle, ESM_NotifyParams* params, int32_t* errCode);

/** 
*   \brief Deregister the ESM notifers.
*
*   @param[in] handle: Handle to the ESM Driver.
*
*   @param[in] notifyIndex: Notifier index returned when the notifier was registered.
*
*   @param[out]  errCode
*      Error code populated on error.
*
*   @return On success returns SystemP_SUCCESS. Negative values indicate
*           unsuccessful operations.
*
*/
int32_t ESM_deregisterNotifier(ESM_Handle handle, int32_t notifyIndex, int32_t* errCode);

/**
 * \brief   This API is used to configure operation mode of ESM module.
 *
 * \param   baseAddr        Base Address of the ESM Registers.
 *
 * \param   mode            Channel number for which reset is to be done.
 *                          Refer enum #ESM_OperationMode.
 */
void ESM_setMode(uint32_t baseAddr, uint32_t mode);

/**
 * \brief   This API is used to configure the low time counter pre-load value.
 *
 * \param   baseAddr        Base Address of the ESM Registers.
 *
 * \param   lowTime         Time to be configured as LTCP.
 *
 * \return  status          Configuration status.
 */
int32_t ESM_setErrPinLowTimePreload(uint32_t baseAddr, uint32_t lowTime);

/**
 * \brief   This API is used to get the current value of low time counter.
 *
 * \param   baseAddr        Base Address of the ESM Registers.
 *
 * \return  Counter value   Current low time count.
 */
uint32_t ESM_getCurrErrPinLowTimeCnt(uint32_t baseAddr);

/**
 * \brief   This API is used to get the current status of nERROR pin.
 *
 * \param   baseAddr        Base Address of the ESM Registers.
 *
 * \return  status          Current nERROR pin status.
 */
uint32_t ESM_getErrPinStatus(uint32_t baseAddr);

/**
 * \brief   This API is used to reset the nERROR pin.
 *          This will set the nERROR pin to high.
 *
 * \param   baseAddr        Base Address of the ESM Registers.
 */
void ESM_resetErrPin(uint32_t baseAddr);

/**
 * \brief   This API is used to get the interrupt status.
 *
 * \param   baseAddr        Base Address of the ESM Registers.
 *
 * \param   intrSrc         Interrupt for which status to return.
 *
 * \return  status          Interrupt status.
 */
uint32_t ESM_getIntrStatus(uint32_t baseAddr, uint32_t intrSrc);

/**
 * \brief   This API is used to get the interrupt/error status for a group.
 *
 * \param   baseAddr        Base Address of the ESM Registers.
 *
 * \param   grpNum          Group for which status to return.
 *                          It Starts from 1 and should not be more than
 *                          ESM_NUM_GROUP_MAX.
 *
 * \param   intrstatus      Interrupt status.
 *                          Refer struct #ESM_GroupIntrStatus.
 *
 * \return  status          Success/failure.
 */
int32_t ESM_getGroupIntrStatus(uint32_t baseAddr,
                              uint32_t grpNum,
                              ESM_GroupIntrStatus *intrstatus);

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */
static inline void ESM_Params_init(ESM_OpenParams *openPrms)
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
