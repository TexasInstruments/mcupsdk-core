/*
 *  Copyright (C) 2023-2024 Texas Instruments Incorporated
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
 *
 * @ingroup  SDL_ESM_MODULE
 * @defgroup SDL_IP_ESM_API ESM Low-Level API
 *
 * The Error Signaling Module (ESM) aggregates safety-related events and/or errors from throughout the
 * device into one location. It can signal both low and high priority interrupts to a processor to deal with a
 * safety event and/or manipulate an I/O error pin to signal an external hardware that an error has occurred.
 * Therefore an external controller is able to reset the device or keep the system in a safe, known state.
 *
 * The SDL APIs provide APIs to do the following functionalities.
 *
 * -# API to read back ESM registers
 * -# API to set the ESM error forcing mode
 * -# APIs for ESM initialization sequence/configurations
 * -# APIs for ESM Raw error status
 *
 *  @{
 */

/**
 *  \file     esm.h
 *
 *  \brief    This file contains the prototypes of the APIs present in the
 *            device abstraction layer file of ESM.
 *            This also contains some related macros.
 */

#ifndef SDL_ESM_H
#define SDL_ESM_H

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>
#include <stdbool.h>


#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/**
 * \brief  ESM Operation Mode type.
 */

#define ESM_NUMBER_OF_GROUP_REGS          (32u)

/**
 *  \anchor esmOperationMode_t
 *  \name ESM Operation Mode type
 *  @{
 */

/**
 * \brief  ESM Operation Mode type.
 */
typedef uint32_t esmOperationMode_t;

#define ESM_OPERATION_MODE_NORMAL 0x0U
    /**< Configure ESM operation mode to normal mode */
#define ESM_OPERATION_MODE_ERROR_FORCE 0xAu
    /**< Configure ESM operation mode to error force mode */
/** @} */

/**
 *  \anchor esmErrOutMode_t
 *  \name ESM Error Output Mode type
 *  @{
 */

/**
 * \brief  ESM Error Output Mode type.
 */
typedef uint32_t esmErrOutMode_t;

#define SDL_ESM_LVL_PINOUT 0x0u
    /**< Configure ESM error output mode to LEVEL output */
#define SDL_ESM_PWM_PINOUT 0xFu
    /**< Configure ESM error output mode to PWM output */
/** @} */


/**
 *  \anchor esmIntrType_t
 *  \name ESM Interrupt Type to select level for interrupt.
 *  @{
 */

typedef uint32_t esmIntrType_t;

#define ESM_INTR_TYPE_CONFIG_ERROR   (0x0u)
    /**< Configure interrupt to high level interrupt */
#define ESM_INTR_TYPE_LOW_PRIO_ERROR (0x1u)
    /**< Configure interrupt to high level interrupt */
#define ESM_INTR_TYPE_HIGH_PRIO_ERROR (0x2u)
    /**< Configure interrupt to low level interrupt */
/** @} */

/**
 *  \anchor esmIntrPriorityLvl_t
 *  \name ESM Interrupt Priority Levels
 *  @{
 */

typedef uint32_t esmIntrPriorityLvl_t;

#define ESM_INTR_PRIORITY_LEVEL_LOW  (0x0u)
    /**< Configure interrupt to low level interrupt */
#define ESM_INTR_PRIORITY_LEVEL_HIGH (0x1u)
    /**< Configure interrupt to high level interrupt */
/** @} */


/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/**
 * \brief  Structure for accessing Revision ID of ESM module.
 */
typedef struct esmRevisionId
{
    uint32_t scheme;
    /**< Scheme */
    uint32_t func;
    /**< Functional number */
    uint32_t rtlRev;
    /**< RTL revision */
    uint32_t major;
    /**< Major revision */
    uint32_t custom;
    /**< Custom revision */
    uint32_t minor;
    /**< Minor revision */
}esmRevisionId_t;

/**
 * \brief  Structure for accessing information register of ESM module.
 */
typedef struct esmInfo
{
    uint32_t lastRstType;
    /**< Last reset type
     *   0 – Last reset was a Power On Reset
     *   1 – Last reset was a Warm Reset
     */
    uint32_t plsGrpNum;
    /**< Number of event groups that are pulse */
    uint32_t lvlGrpNum;
    /**< Number of event groups that are level */
}esmInfo_t;

/**
 * \brief  Structure to access the status of interrupts belonging to a High or
 *         Low priority interrupt.
 */
typedef struct esmGroupIntrStatus
{
    uint32_t highestPendPlsIntNum;
    /**< Indicates what is the highest priority High Priority interrupt
     *  caused by a pulse number.
     */
    uint32_t highestPendLvlIntNum;
    /**< Indicates what is the highest priority High Priority interrupt
     *  caused by a level number.
     */
    uint32_t grpIntrStatus;
    /**< Indicates which Event Groups have one or more interrupts
     *   pending. This register is bit oriented where bit 0 is for
     *   Event Group 0, bit 1 is for Event Group 1, etc…
     *   (bit N is for Event Group N).
     */
}esmGroupIntrStatus_t;

/**
 * \brief  ESM Error Group static registers list.
 */
typedef struct {
    volatile uint32_t RAW;                       /* Config Error Raw Status/Set Register */
    volatile uint32_t INTR_EN_SET;               /* Level Error Interrutp Enable Set Register */
    volatile uint32_t INTR_EN_CLR;               /* Level Error Interrupt Enabled Clear Register */
    volatile uint32_t INT_PRIO;                  /* Level Error Interrupt Enabled Clear Register */
    volatile uint32_t PIN_EN_SET;                /* Level Error Interrupt Enabled Clear Register */
    volatile uint32_t PIN_EN_CLR;                /* Level Error Interrupt Enabled Clear Register */
} SDL_esmRegs_ERR_GRP_STATIC;

/**
 * \brief  ESM static registers list.
 */
typedef struct {
    SDL_esmRegs_ERR_GRP_STATIC ERR_GRP[ESM_NUMBER_OF_GROUP_REGS];
    volatile uint32_t PID;                       /* Revision Register */
    volatile uint32_t INFO;                      /* Info Register */
    volatile uint32_t EN;                        /* Global Enable Register */
    volatile uint32_t ERR_EN_SET;                /* Config Error Interrutp Enable Set Register */
    volatile uint32_t ERR_EN_CLR;                /* Config Error Interrupt Enabled Clear Register */
    volatile uint32_t LOW_PRI;                   /* Low Priority Prioritized Register */
    volatile uint32_t HI_PRI;                    /* High Priority Prioritized Register */
    volatile uint32_t LOW;                       /* Low Priority Interrupt Status Register */
    volatile uint32_t HI;                        /* High Priority Interrupt Status Register */
    volatile uint32_t PIN_CTRL;                  /* Error Pin Control Register */
    volatile uint32_t PIN_CNTR_PRE;              /* Error Counter Value Pre-Load Register */
    volatile uint32_t PWMH_PIN_CNTR_PRE;         /* Error PWM High Counter Value Pre-Load Register */
    volatile uint32_t PWML_PIN_CNTR_PRE;         /* Error PWM Low Counter Value Pre-Load Register */

}SDL_ESM_staticRegs;


/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
/**
 * \brief   This API is used to configure operation mode of ESM module.
 *
 * \param   baseAddr        Base Address of the ESM Registers.
 *
 * \param   mode            Channel number for which reset is to be done.
 *                          Refer enum #esmOperationMode_t.
 *
 * \return                  SDL_PASS - API success
 * @n                       SDL_EBADARGS - API fails due to bad input arguments
 *
 * \pre
 *   @n  ESM module is reset and initialized for desired operation
 *
 * \post
 *   @n  None
 *
 * @b Example
   @verbatim

       uint32_t            baseAddr = SDL_MCU_ESM0_CFG_BASE;
       esmOperationMode_t  mode     = ESM_OPERATION_MODE_ERROR_FORCE;
       int32_t             sdlRet;

       sdlRet = SDL_ESM_setMode (baseAddr, mode);

   @endverbatim
 *
 */
int32_t SDL_ESM_setMode(uint32_t baseAddr, esmOperationMode_t mode);

/**
 * \brief   This API is used to read operation mode of ESM module.
 *
 * \param   baseAddr        Base Address of the ESM Registers.
 *
 * \param   pMode           pointer to variable to hold ESM operation Mode.
 *                          Refer enum #esmOperationMode_t.
 *
 * \return                  SDL_PASS - API success
 * @n                       SDL_EBADARGS - API fails due to bad input arguments
 *
 * \pre
 *   @n  ESM module is reset and initialized for desired operation
 *
 * \post
 *   @n  None
 *
 * @b Example
   @verbatim

       uint32_t            baseAddr = SDL_MCU_ESM0_CFG_BASE;
       esmOperationMode_t  mode;
       uint32_t            sdlRet;

       sdlRet = SDL_ESM_getPinMode (baseAddr, &mode);

   @endverbatim
 *
 */
int32_t SDL_ESM_getPinMode(uint32_t baseAddr, esmOperationMode_t *pMode);


/**
 * \brief   This API is used to read Error Out mode (LVL or PWM) of ESM module.
 *
 * \param   baseAddr        Base Address of the ESM Registers.
 *
 * \param   pMode           pointer to variable to hold ESM ErrorOut Mode.
 *                          Refer enum #esmErrOutMode_t.
 *
 * \return                  SDL_PASS - API success
 * @n                       SDL_EBADARGS - API fails due to bad input arguments
 *
 * \pre
 *   @n  ESM module is reset and initialized for desired operation
 *
 * \post
 *   @n  None
 *
 * @b Example
   @verbatim

       uint32_t            baseAddr = SDL_MCU_ESM0_CFG_BASE;
       esmOperationMode_t  mode;
       uint32_t            sdlRet;

       sdlRet = SDL_ESM_getErrorOutMode (baseAddr, &mode);

   @endverbatim
 *
 */
int32_t SDL_ESM_getErrorOutMode(uint32_t baseAddr, esmOperationMode_t *pMode);

/**
 * \brief   This API is used to set the influence of interrupt on nERROR pin.
 *
 * \param   baseAddr        Base Address of the ESM Registers.
 *
 * \param   intrSrc         Interrupt source which will influence nERROR pin.
 *
 * \param   enable          true: Enables influence on nERROR pin if it is TRUE.
 *                          false:Disables influence on nERROR pin if it is FALSE.
 *
 * \return                  SDL_PASS - success
 * @n                       SDL_EBADARGS - API fails due to bad input arguments
 *
 * Note: the intrSrc check is only at the IP level max and not done at instance
 *       level, so it is expected to be done at higher layer.
 *
 * \pre
 *   @n  ESM module is reset and initialized for desired operation
 *
 * \post
 *   @n  None
 *
 * @b Example
   @verbatim

       uint32_t            baseAddr = SDL_MCU_ESM0_CFG_BASE;
       uint32_t            intrSrc  = SDL_ESM1_INTR_MCU0_CPU0_ECC_CORRECTED_LEVEL;
       int32_t             sdlRet;
       sdlRet =  SDL_ESM_setInfluenceOnErrPin (baseAddr, intrSrc, TRUE);

   @endverbatim
 *
 */
int32_t SDL_ESM_setInfluenceOnErrPin(uint32_t baseAddr, uint32_t intrSrc,
                                     bool enable);

/**
 * \brief   This API is used to get the influence of interrupt on nERROR pin.
 *
 * \param   baseAddr        Base Address of the ESM Registers.
 *
 * \param   intrSrc         Interrupt source which will influence nERROR pin.
 *
 * \param   pInfluence      pointer to read the influence value
 *                          1: enabled
 *                          0: disabled
 *
 * \return                  SDL_PASS - success
 * @n                       SDL_EBADARGS - API fails due to bad input arguments
 *
 *
 * Note: the intrSrc check is only at the IP level max and not done at instance
 *       level, so it is expected to be done at higher layer.
 *
 * \pre
 *   @n  ESM module is reset and initialized for desired operation
 *
 * \post
 *   @n  None
 *
 * @b Example
   @verbatim

       uint32_t            baseAddr = SDL_MCU_ESM0_CFG_BASE;
       uint32_t            intrSrc  = SDL_ESM1_INTR_MCU0_CPU0_ECC_CORRECTED_LEVEL;
       uint32_t            influence;
       int32_t             sdlRet;
       sdlRet =  SDL_ESM_getInfluenceOnErrPin (baseAddr, intrSrc, &influence);

   @endverbatim
 *
 */
int32_t SDL_ESM_getInfluenceOnErrPin(uint32_t baseAddr, uint32_t intrSrc,
                                     uint32_t *pInfluence);
/**
 * \brief   This API is used to configure the low time counter pre-load value.
 *
 * \param   baseAddr        Base Address of the ESM Registers.
 *
 * \param   lowTime         Time to be configured as LTCP.
 *
 * \return                  SDL_PASS - success
 * @n                       SDL_EBADARGS - API fails due to bad input arguments
 *
 * \pre
 *   @n  ESM module is reset and initialized for desired operation
 *
 * This is the value that will be pre-loaded in to the counter field of the
 * ESM_PIN_CNTR register whenever the ESM enters the
 * ESM_ERROR state from ESM_IDLE. The default value is
 * determined based on the ESM clock frequency, so that there is a
 * minimum low time of 100 micro seconds.
 * This field is only reset by a Power-On-Reset (not warm reset). A
 * global soft reset will set this field to 0h.
 *
 * \post
 *   @n  None
 *
 * @b Example
   @verbatim

       uint32_t            baseAddr = SDL_MCU_ESM0_CFG_BASE;
       uint32_t            pinCntrPre  = 100;
       int32_t             sdlRet;
       sdlRet = SDL_ESM_setErrPinLowTimePreload (baseAddr, pinCntrPre);

   @endverbatim
 *
 */
int32_t SDL_ESM_setErrPinLowTimePreload(uint32_t baseAddr, uint32_t lowTime);

/**
 * \brief   This API is used to configure the low time counter pre-load value for PWM error.
 *
 * \param   baseAddr        Base Address of the ESM Registers.
 *
 * \param   lowTime         Time to be configured as LTCP.
 *
 * \return                  SDL_PASS - success
 * @n                       SDL_EBADARGS - API fails due to bad input arguments
 *
 * \pre
 *   @n  ESM module is reset and initialized for desired operation
 *
 * This is the value that will be loaded in to the counter field of
 * the Error Pin PWM low Counter Value Register whenever the error
 * output pin toggles low.
 *
 * \post
 *   @n  None
 *
 * @b Example
   @verbatim

       uint32_t            baseAddr = SDL_MCU_ESM0_CFG_BASE;
       uint32_t            pinCntrPre  = 100;
       int32_t             sdlRet;
       sdlRet = SDL_ESM_PWML_setErrPinLowTimePreload(baseAddr, pinCntrPre);

   @endverbatim
 *
 */
int32_t SDL_ESM_PWML_setErrPinLowTimePreload(uint32_t baseAddr, uint32_t lowTime);

/**
 * \brief   This API is used to configure the high time counter pre-load value for PWM error.
 *
 * \param   baseAddr        Base Address of the ESM Registers.
 *
 * \param   highTime         Time to be configured as HTCP.
 *
 * \return                  SDL_PASS - success
 * @n                       SDL_EBADARGS - API fails due to bad input arguments
 *
 * \pre
 *   @n  ESM module is reset and initialized for desired operation
 *
 * This is the value that will be loaded in to the counter field of
 * the Error Pin PWM High Counter Value Register whenever the error
 * output pin toggles high.
 *
 * \post
 *   @n  None
 *
 * @b Example
   @verbatim

       uint32_t            baseAddr = SDL_MCU_ESM0_CFG_BASE;
       uint32_t            pinCntrPre  = 100;
       int32_t             sdlRet;
       sdlRet = SDL_ESM_PWMH_setErrPinHighTimePreload(baseAddr, pinCntrPre);

   @endverbatim
 *
 */
int32_t SDL_ESM_PWMH_setErrPinHighTimePreload(uint32_t baseAddr, uint32_t highTime);

/**
 * \brief   This API is used to read the low time counter pre-load value.
 *
 * \param   baseAddr        Base Address of the ESM Registers.
 *
 * \param   pLowTime        pointer to Time to be read as LTCP.
 *
 * \return                  SDL_PASS - success
 * @n                       SDL_EBADARGS - API fails due to bad input arguments
 *
 * \pre
 *   @n  ESM module is reset and initialized for desired operation
 *
 * This is the value that will be pre-loaded in to the counter field of the
 * ESM_PIN_CNTR register whenever the ESM enters the
 * ESM_ERROR state from ESM_IDLE. The default value is
 * determined based on the ESM clock frequency, so that there is a
 * minimum low time of 100 micro seconds.
 * This field is only reset by a Power-On-Reset (not warm reset). A
 * global soft reset will set this field to 0h.
 *
 * \post
 *   @n  None
 *
 * @b Example
   @verbatim

       uint32_t            baseAddr = SDL_MCU_ESM0_CFG_BASE;
       uint32_t            inCntrPre;
       int32_t             sdlRet;
       sdlRet = SDL_ESM_getErrPinLowTimePreload (baseAddr, &inCntrPre);

   @endverbatim
 *
 */
int32_t SDL_ESM_getErrPinLowTimePreload(uint32_t baseAddr, uint32_t *pLowTime);

/**
 * \brief   This API is used to read the low time counter pre-load value for PWM error.
 *
 * \param   baseAddr               Base Address of the ESM Registers.
 *
 * \param   pPinPWMLCntrPre        pointer to Time to be read as LTCP.
 *
 * \return                         SDL_PASS - success
 * @n                              SDL_EBADARGS - API fails due to bad input arguments
 *
 * \pre
 *   @n  ESM module is reset and initialized for desired operation
 *
 * This is the value that will be loaded in to the counter field of
 * the Error Pin PWM Low Counter Value Register whenever the error
 * output pin toggles low.
 *
 * \post
 *   @n  None
 *
 * @b Example
   @verbatim

       uint32_t            baseAddr = SDL_MCU_ESM0_CFG_BASE;
       uint32_t            inCntrPre;
       int32_t             sdlRet;
       sdlRet = SDL_ESM_PWML_getErrPinLowTimePreload(baseAddr, &inCntrPre);

   @endverbatim
 *
 */
int32_t SDL_ESM_PWML_getErrPinLowTimePreload(uint32_t baseAddr, uint32_t *pPinPWMLCntrPre);

/**
 * \brief   This API is used to read the High time counter pre-load value for PWM error.
 *
 * \param   baseAddr               Base Address of the ESM Registers.
 *
 * \param   pPinPWMHCntrPre        pointer to Time to be read as HTCP.
 *
 * \return                         SDL_PASS - success
 * @n                              SDL_EBADARGS - API fails due to bad input arguments
 *
 * \pre
 *   @n  ESM module is reset and initialized for desired operation
 *
 * This is the value that will be loaded in to the counter field of
 * the Error Pin PWM High Counter Value Register whenever the error
 * output pin toggles high.
 *
 * \post
 *   @n  None
 *
 * @b Example
   @verbatim

       uint32_t            baseAddr = SDL_MCU_ESM0_CFG_BASE;
       uint32_t            inCntrPre;
       int32_t             sdlRet;
       sdlRet = SDL_ESM_PWMH_getErrPinHighTimePreload(baseAddr, inCntrPre);

   @endverbatim
 *
 */
int32_t SDL_ESM_PWMH_getErrPinHighTimePreload(uint32_t baseAddr, uint32_t *pPinPWMHCntrPre);


/**
 * \brief   This API is used to get the current value of low time counter.
 *
 * \param   baseAddr        Base Address of the ESM Registers.
 *
 * \param   pPinCntrPre     pointer to Counter value Current low time count.
 *
 * \return                  SDL_PASS - success
 * @n                       SDL_EBADARGS - API fails due to bad input arguments
 *
 * \pre
 *   @n  ESM module is reset and initialized for desired operation
 *
 * \post
 *   @n  None
 *
 * @b Example
   @verbatim

       uint32_t            baseAddr = SDL_MCU_ESM0_CFG_BASE;
       uint32_t            pinCntrPre;
       int32_t             sdlRet;
       sdlRet  = SDL_ESM_getCurrErrPinLowTimeCnt (baseAddr, &pinCntrPre);

   @endverbatim
 *
 */
int32_t SDL_ESM_getCurrErrPinLowTimeCnt(uint32_t baseAddr, uint32_t *pPinCntrPre);

/**
 * \brief   This API is used to get the current value of low time counter for PWM error.
 *
 * \param   baseAddr        Base Address of the ESM Registers.
 *
 * \param   pLowPWMLTime     pointer to Counter value Current low time count for PWM error.
 *
 * \return                  SDL_PASS - success
 * @n                       SDL_EBADARGS - API fails due to bad input arguments
 *
 * \pre
 *   @n  ESM module is reset and initialized for desired operation
 *
 * \post
 *   @n  None
 *
 * @b Example
   @verbatim

       uint32_t            baseAddr = SDL_MCU_ESM0_CFG_BASE;
       uint32_t            pinCntrPre;
       int32_t             sdlRet;
       sdlRet  = SDL_ESM_PWML_getCurrErrPinLowTimeCnt(baseAddr, &pinCntrPre);

   @endverbatim
 *
 */
int32_t SDL_ESM_PWML_getCurrErrPinLowTimeCnt(uint32_t baseAddr, uint32_t *pLowPWMLTime);

/**
 * \brief   This API is used to get the current value of high time counter for PWM error.
 *
 * \param   baseAddr        Base Address of the ESM Registers.
 *
 * \param   pHighPWMHTime    pointer to Counter value Current high time count for PWM error.
 *
 * \return                  SDL_PASS - success
 * @n                       SDL_EBADARGS - API fails due to bad input arguments
 *
 * \pre
 *   @n  ESM module is reset and initialized for desired operation
 *
 * \post
 *   @n  None
 *
 * @b Example
   @verbatim

       uint32_t            baseAddr = SDL_MCU_ESM0_CFG_BASE;
       uint32_t            pinCntrPre;
       int32_t             sdlRet;
       sdlRet  = SDL_ESM_PWMH_getCurrErrPinHighTimeCnt(baseAddr, &pinCntrPre);

   @endverbatim
 *
 */
int32_t SDL_ESM_PWMH_getCurrErrPinHighTimeCnt(uint32_t baseAddr, uint32_t *pHighPWMHTime);

/**
 * \brief   This API is used to get the current status of nERROR pin.
 *
 * \param   baseAddr        Base Address of the ESM Registers.
 *
 * \param   pStatus         pointer to Current nERROR pin status.
 *
 * \return                  SDL_PASS - success
 * @n                       SDL_EBADARGS - API fails due to bad input arguments
 *
 * \pre
 *   @n  ESM module is reset and initialized for desired operation
 *
 * \post
 *   @n  None
 *
 * @b Example
   @verbatim

       uint32_t            baseAddr = SDL_MCU_ESM0_CFG_BASE;
       uint32_t            status;
       uint32_t            sdlRet;
       sdlRet = SDL_ESM_getErrPinStatus (baseAddr, &status);

   @endverbatim
 *
 */
int32_t SDL_ESM_getErrPinStatus(uint32_t baseAddr, uint32_t *pStatus);

/**
 * \brief   This API is used to reset the nERROR pin.
 *
 * \param   baseAddr        Base Address of the ESM Registers.
 *
 * \return                  SDL_PASS - success
 * @n                       SDL_EBADARGS - API fails due to bad input arguments
 *
 * \note    This will set the nERROR pin to high.
 *
 * \pre
 *   @n  ESM module is reset and initialized for desired operation
 *
 * \post
 *   @n  None
 *
 * @b Example
   @verbatim

       uint32_t            baseAddr = SDL_MCU_ESM0_CFG_BASE;
       int32_t             sdlRet;
       sdlRet = SDL_ESM_resetErrPin (baseAddr);

   @endverbatim
 *
 */
int32_t SDL_ESM_resetErrPin(uint32_t baseAddr);

/**
 * \brief   This API is used check if the configuration interrupt for a group is enabled/disabled.
 *
 * \param   baseAddr        Base Address of the ESM Registers.
 *
 * \param   group           Group number for which to check if the interrupt is enabled.
 *
 * \param   pEnStatus       Pointer to status of interrupt enable variable
 *
 * \return                  SDL_PASS - success
 * @n                       SDL_EBADARGS - API fails due to bad input arguments
 *
 *
 * \pre
 *   @n  ESM module is reset and initialized for desired operation
 *
 *
 * Note: the group check is only at the IP level max and not done at instance
 *       level, so it is expected to be done at higher layer.
 *
 * \post
 *   @n  None
 *
 * @b Example
   @verbatim

       uint32_t            baseAddr = SDL_MCU_ESM0_CFG_BASE;
       uint32_t            status, group = 0u;
       int32_t             sdlRet;
       sdlRet = SDL_ESM_isEnableCfgIntr (baseAddr, group, &status);

   @endverbatim
 *
 */
int32_t SDL_ESM_isEnableCfgIntr(uint32_t baseAddr, uint32_t group, uint32_t *pEnStatus);

/**
 * \brief   This API is used check if interrupt is enabled/disabled.
 *
 * \param   baseAddr        Base Address of the ESM Registers.
 *
 * \param   intrSrc         Interrupt to enable.
 *
 * \param   pEnStatus       Pointer to status of interrupt enable variable
 *
 * \return                  SDL_PASS - success
 * @n                       SDL_EBADARGS - API fails due to bad input arguments
 *
 *
 * \pre
 *   @n  ESM module is reset and initialized for desired operation
 *
 *
 * Note: the intrSrc check is only at the IP level max and not done at instance
 *       level, so it is expected to be done at higher layer.
 *
 * \post
 *   @n  None
 *
 * @b Example
   @verbatim

       uint32_t            baseAddr = SDL_MCU_ESM0_CFG_BASE;
       uint32_t            status, intrNum  = 0u;
       int32_t             sdlRet;
       sdlRet = SDL_ESM_isEnableIntr (baseAddr, intrNum, &status);

   @endverbatim
 *
 */
int32_t SDL_ESM_isEnableIntr(uint32_t baseAddr, uint32_t intrSrc, uint32_t *pEnStatus);

/**
 * \brief   This API is used to disable the configuration interrupt.
 *
 * \param   baseAddr        Base Address of the ESM Registers.
 *
 * \param   intrNum         Interrupt Number to diable
 *
 * \return                  SDL_PASS - success
 * @n                       SDL_EBADARGS - API fails due to bad input arguments
 *
 *
 * \pre
 *   @n  None
 *
 * \post
 *   @n  None
 *
 * @b Example
   @verbatim

       uint32_t            baseAddr = SDL_MCU_ESM0_CFG_BASE;
       uint32_t            intrNum = 0u;
       int32_t             sdlRet;
       sdlRet = SDL_ESM_disableCfgIntr (baseAddr, intrNum);

   @endverbatim
 *
 */
int32_t SDL_ESM_disableCfgIntr(uint32_t baseAddr, uint32_t intrNum);

/**
 * \brief   This API is used to enable the configuration interrupt.
 *
 * \param   baseAddr        Base Address of the ESM Registers.
 *
 * \param   group           Group for which to enable configuration interrupt.
 *
 * \return                  SDL_PASS - success
 * @n                       SDL_EBADARGS - API fails due to bad input arguments
 *
 *
 * \pre
 *   @n  ESM module is reset and initialized for desired operation
 *
 * \post
 *   @n  None
 *
 * @b Example
   @verbatim

       uint32_t            baseAddr = SDL_MCU_ESM0_CFG_BASE;
       uint32_t            group = 0u;
       int32_t             status;
       status = SDL_ESM_enableCfgIntr (baseAddr, group);

   @endverbatim
 *
 */
int32_t SDL_ESM_enableCfgIntr(uint32_t baseAddr, uint32_t group);

/**
 * \brief   This API is used to enable interrupt.
 *
 * \param   baseAddr        Base Address of the ESM Registers.
 *
 * \param   intrNum         Interrupt to enable.
 *
 * \return                  SDL_PASS - success
 * @n                       SDL_EBADARGS - API fails due to bad input arguments
 *
 *
 * \pre
 *   @n  ESM module is reset and initialized for desired operation
 *
 * \post
 *   @n  None
 *
 * @b Example
   @verbatim

       uint32_t            baseAddr = SDL_MCU_ESM0_CFG_BASE;
       uint32_t            intrNum  = 0u;
       int32_t             status;
       status = SDL_ESM_enableIntr (baseAddr, intrNum);

   @endverbatim
 *
 */
int32_t SDL_ESM_enableIntr(uint32_t baseAddr, uint32_t intrNum);

/**
 * \brief   This API is used to disable interrupt.
 *
 * \param   baseAddr        Base Address of the ESM Registers.
 *
 * \param   intrNum         Interrupt to disable.
 *
 * \return                  SDL_PASS - success
 * @n                       SDL_EBADARGS - API fails due to bad input arguments
 *
 *
 * \pre
 *   @n  None
 *
 * \post
 *   @n  None
 *
 * @b Example
   @verbatim

       uint32_t            baseAddr = SDL_MCU_ESM0_CFG_BASE;
       uint32_t            intrNum  = 0u;
       int32_t             sdlRet;
       sdlRet = SDL_ESM_setInfluenceOnErrPin (baseAddr, intrNum);

   @endverbatim
 *
 */
int32_t SDL_ESM_disableIntr(uint32_t baseAddr, uint32_t intrNum);

/**
 * \brief   This API is used to set interrupt level.
 *
 * \param   baseAddr        Base Address of the ESM Registers.
 *
 * \param   intrSrc         Interrupt to set the level.
 *
 * \param   intrPriorityLvl Interrupt level to set.
 *                          Refer enum #esmIntrPriorityLvl_t.
 *
 * \return                  SDL_PASS - success
 * @n                       SDL_EBADARGS - API fails due to bad input arguments
 *
 *
 * \pre
 *   @n  ESM module is reset and initialized for desired operation
 *
 *
 * Note: the intrSrc check is only at the IP level max and not done at instance
 *       level, so it is expected to be done at higher layer.
 *
 * \post
 *   @n  None
 *
 * @b Example
   @verbatim

       uint32_t            baseAddr = SDL_MCU_ESM0_CFG_BASE;
       uint32_t            intrSrc  = SDL_ESM1_INTR_MCU0_CPU0_ECC_CORRECTED_LEVEL;
       esmIntrPriorityLvl_t pri     = ESM_INTR_PRIORITY_LEVEL_HIGH;
       int32_t             sdlRet;
       sdlRet = SDL_ESM_setIntrPriorityLvl (baseAddr, intrSrc, pri);

   @endverbatim
 *
 */
int32_t SDL_ESM_setIntrPriorityLvl(uint32_t baseAddr, uint32_t intrSrc,
                                   esmIntrPriorityLvl_t intrPriorityLvl);

/**
 * \brief   This API is used to get interrupt level.
 *
 * \param   baseAddr        Base Address of the ESM Registers.
 *
 * \param   intrSrc         Interrupt to set the level.
 *
 * \param   pIntrPriorityLvl Pointer to Interrupt level to get.
 *                          Refer enum #esmIntrPriorityLvl_t.
 *
 * \return                  SDL_PASS - success
 * @n                       SDL_EBADARGS - API fails due to bad input arguments
 *
 *
 *
 * Note: the intrSrc check is only at the IP level max and not done at instance
 *       level, so it is expected to be done at higher layer.
 *
 * \pre
 *   @n  ESM module is reset and initialized for desired operation
 *
 * \post
 *   @n  None
 *
 * @b Example
   @verbatim

       uint32_t            baseAddr = SDL_MCU_ESM0_CFG_BASE;
       uint32_t            intrSrc  = SDL_ESM1_INTR_MCU0_CPU0_ECC_CORRECTED_LEVEL;
       esmIntrPriorityLvl_t            pri;
       int32_t             sdlRet;
       sdlRet = SDL_ESM_getIntrPriorityLvl (baseAddr, intrSrc, &pri);

   @endverbatim
 *
 */
int32_t SDL_ESM_getIntrPriorityLvl(uint32_t baseAddr, uint32_t intrSrc,
                                   esmIntrPriorityLvl_t *pIntrPriorityLvl);

/**
 * \brief   This API is used to get the configuration interrupt status for a group.
 *
 * \param   baseAddr        Base Address of the ESM Registers.
 *
 * \param   group           Group for which status to return.
 *
 * \param   pStaus          pointer to interrupt status
 *
 * \return                  SDL_PASS - success
 * @n                       SDL_EBADARGS - API fails due to bad input arguments
 *
 *
 * \pre
 *   @n  ESM module is reset and initialized for desired operation
 *
 * \post
 *   @n  None
 *
 *
 * Note: the group check is only at the IP level max and not done at instance
 *       level, so it is expected to be done at higher layer.
 *
 * @b Example
   @verbatim

       uint32_t            baseAddr = SDL_MCU_ESM0_CFG_BASE;
       uint32_t            status, group = 0u;
       int32_t             sdlRet;
       sdlRet = SDL_ESM_getCfgIntrStatus (baseAddr, group, &status);

   @endverbatim
 *
 */
int32_t SDL_ESM_getCfgIntrStatus(uint32_t baseAddr, uint32_t group, uint32_t *pStaus);

/**
 * \brief   This API is used to get the interrupt status.
 *
 * \param   baseAddr        Base Address of the ESM Registers.
 *
 * \param   intrSrc         Interrupt for which status to return.
 *
 * \param   pStaus          pointer to interrupt status
 *
 * \return                  SDL_PASS - success
 * @n                       SDL_EBADARGS - API fails due to bad input arguments
 *
 *
 * \pre
 *   @n  ESM module is reset and initialized for desired operation
 *
 * \post
 *   @n  None
 *
 *
 * Note: the intrSrc check is only at the IP level max and not done at instance
 *       level, so it is expected to be done at higher layer.
 *
 * @b Example
   @verbatim

       uint32_t            baseAddr = SDL_MCU_ESM0_CFG_BASE;
       uint32_t            status, intrSrc  = SDL_ESM1_INTR_MCU0_CPU0_ECC_CORRECTED_LEVEL;
       int32_t             sdlRet;
       sdlRet = SDL_ESM_getIntrStatus (baseAddr, intrSrc, &status);

   @endverbatim
 *
 */
int32_t SDL_ESM_getIntrStatus(uint32_t baseAddr, uint32_t intrSrc, uint32_t *pStaus);

/**
 * \brief   This API is used to set the configuration interrupt RAW status for a group.
 *
 * \param   baseAddr        Base Address of the ESM Registers.
 *
 * \param   group           Group for which status to return.
 *
 *
 * \return                  SDL_PASS - success
 * @n                       SDL_EBADARGS - API fails due to bad input arguments
 *
 * \pre
 *   @n  ESM module is reset and initialized for desired operation
 *
 * \post
 *   @n  None
 *
 *
 * Note: the group check is only at the IP level max and not done at instance
 *       level, so it is expected to be done at higher layer.
 *
 * @b Example
   @verbatim

       uint32_t            baseAddr = SDL_MCU_ESM0_CFG_BASE;
       uint32_t            group = 0u;
       int32_t             sdlRet;
       sdlRet = SDL_ESM_setCfgIntrStatusRAW (baseAddr, group);

   @endverbatim
 *
 */
int32_t SDL_ESM_setCfgIntrStatusRAW(uint32_t baseAddr, uint32_t group);

/**
 * \brief   This API is used to set the interrupt RAW status.
 *
 * \param   baseAddr        Base Address of the ESM Registers.
 *
 * \param   intrSrc         Interrupt for which status to return.
 *
 *
 * \return                  SDL_PASS - success
 * @n                       SDL_EBADARGS - API fails due to bad input arguments
 *
 * \pre
 *   @n  ESM module is reset and initialized for desired operation
 *
 * \post
 *   @n  None
 *
 *
 * Note: the intrSrc check is only at the IP level max and not done at instance
 *       level, so it is expected to be done at higher layer.
 *
 * @b Example
   @verbatim

       uint32_t            baseAddr = SDL_MCU_ESM0_CFG_BASE;
       uint32_t            intrSrc  = SDL_ESM1_INTR_MCU0_CPU0_ECC_CORRECTED_LEVEL;
       int32_t             sdlRet;
       sdlRet = SDL_ESM_setIntrStatusRAW (baseAddr, intrSrc);

   @endverbatim
 *
 */
int32_t SDL_ESM_setIntrStatusRAW(uint32_t baseAddr, uint32_t intrSrc);

/**
 * \brief   This API is used to get the interrupt RAW status.
 *
 * \param   baseAddr        Base Address of the ESM Registers.
 *
 * \param   intrSrc         Interrupt for which status to return.
 *
 * \param   pStatus         pointer to Interrupt status.
 *
 * \return                  SDL_PASS - success
 * @n                       SDL_EBADARGS - API fails due to bad input arguments
 *
 * \pre
 *   @n  ESM module is reset and initialized for desired operation
 *
 * \post
 *   @n  None
 *
 *
 * Note: the intrSrc check is only at the IP level max and not done at instance
 *       level, so it is expected to be done at higher layer.
 *
 * @b Example
   @verbatim

       uint32_t            baseAddr = SDL_MCU_ESM0_CFG_BASE;
       uint32_t            intrSrc  = SDL_ESM1_INTR_MCU0_CPU0_ECC_CORRECTED_LEVEL;
       uitn32_t            status;
       int32_t             sdlRet;
       sdlRet = SDL_ESM_getIntrStatusRAW (baseAddr, intrSrc, &status);

   @endverbatim
 *
 */
int32_t SDL_ESM_getIntrStatusRAW(uint32_t baseAddr, uint32_t intrSrc, uint32_t *pStatus);

/**
 * \brief   This API is used to get the interrupt/error status for a group.
 *          This will also return highest pending interrupt for pulse as well
 *          as for level interrupts.
 *
 * \param   baseAddr        Base Address of the ESM Registers.
 *
 * \param   intrPrioType    Group for which status to return.
 *                          Refer enum #esmIntrPriorityLvl_t.
 *
 * \param   pIntrstatus     pointer to Interrupt status.
 *                          Refer struct #esmGroupIntrStatus_t.
 *
 * \return                  SDL_PASS - success
 * @n                       SDL_EBADARGS - API fails due to bad input arguments
 *
 * \pre
 *   @n  ESM module is reset and initialized for desired operation
 *
 * \post
 *   @n  None
 *
 * @b Example
   @verbatim

       uint32_t            baseAddr = SDL_MCU_ESM0_CFG_BASE;
       esmIntrPriorityLvl  priType  = ESM_INTR_PRIORITY_LEVEL_HIGH;
       esmGroupIntrStatus_t groupIntrStatus;
       int32_t             sdlRet;

       sdlRet = SDL_ESM_getGroupIntrStatus (baseAddr, priType, &groupIntrStatus);

   @endverbatim
 *
 */
int32_t SDL_ESM_getGroupIntrStatus(uint32_t baseAddr, esmIntrPriorityLvl_t intrPrioType,
                                   esmGroupIntrStatus_t *pIntrstatus);

/**
 * \brief   This API is used to clear the configuration interrupt status.
 *
 * \param   baseAddr        Base Address of the ESM Registers.
 *
 * \param   group           Group for which to clear the configuration interrupt to clear status.
 *
 * \return                  SDL_PASS - success
 * @n                       SDL_EBADARGS - API fails due to bad input arguments
 *
 * \pre
 *   @n  ESM module is reset and initialized for desired operation
 *
 * \post
 *   @n  None
 *
 *
 * Note: the group check is only at the IP level max and not done at instance
 *       level, so it is expected to be done at higher layer.
 *
 * @b Example
   @verbatim

       uint32_t            baseAddr = SDL_MCU_ESM0_CFG_BASE;
       uint32_t            group = 0u;
       int32_t             sdlRet;
       sdlRet = SDL_ESM_clearCfgIntrStatus (baseAddr, group);

   @endverbatim
 *
 */
int32_t SDL_ESM_clearCfgIntrStatus(uint32_t baseAddr, uint32_t group);

/**
 * \brief   This API is used to clear the interrupt status.
 *
 * \param   baseAddr        Base Address of the ESM Registers.
 *
 * \param   intrSrc         Interrupt to clear status.
 *
 * \return                  SDL_PASS - success
 * @n                       SDL_EBADARGS - API fails due to bad input arguments
 *
 * \pre
 *   @n  ESM module is reset and initialized for desired operation
 *
 * \post
 *   @n  None
 *
 *
 * Note: the intrSrc check is only at the IP level max and not done at instance
 *       level, so it is expected to be done at higher layer.
 *
 * @b Example
   @verbatim

       uint32_t            baseAddr = SDL_MCU_ESM0_CFG_BASE;
       uint32_t            intrSrc  = SDL_ESM1_INTR_MCU0_CPU0_ECC_CORRECTED_LEVEL;
       int32_t             sdlRet;
       sdlRet = SDL_ESM_clearIntrStatus (baseAddr, intrSrc);

   @endverbatim
 *
 */
int32_t SDL_ESM_clearIntrStatus(uint32_t baseAddr, uint32_t intrSrc);

/**
 * \brief   This API is used to write EOI.
 *
 * \param   baseAddr        Base Address of the ESM Registers.
 *
 * \param   intrType        Type of interrupt for which to write EOI.
 *                          Refer enum #esmIntrType_t.
 *
 * \return                  SDL_PASS - success
 * @n                       SDL_EBADARGS - API fails due to bad input arguments
 *
 * \pre
 *   @n  ESM module is reset and initialized for desired operation
 *
 * \post
 *   @n  None
 *
 * @b Example
   @verbatim

       uint32_t            baseAddr = SDL_MCU_ESM0_CFG_BASE;
       esmIntrType_t       intrType  = ESM_INTR_TYPE_HIGH_PRIO_ERROR;
       int32_t             sdlRet;
       sdlRet = SDL_ESM_writeEOI (baseAddr, intrType);

   @endverbatim
 *
 */
int32_t SDL_ESM_writeEOI(uint32_t baseAddr, esmIntrType_t intrType);

/**
 * \brief   This API is used get the ESM revision ID.
 *
 * \param   baseAddr        Base Address of the ESM Registers.
 *
 * \param   pRevId          Pointer to Revision ID of ESM module..
 *                          Refer struct #esmRevisionId_t.
 *
 * \return                  SDL_PASS - success
 * @n                       SDL_EBADARGS - API fails due to bad input arguments
 *
 * \pre
 *   @n  ESM module is reset and initialized for desired operation
 *
 * \post
 *   @n  None
 *
 * @b Example
   @verbatim

       uint32_t            baseAddr = SDL_MCU_ESM0_CFG_BASE;
       esmRevisionId_t     revId;
       int32_t             sdlRet;
       sdlRet = ESMGetRevisionId (baseAddr, &revId);

   @endverbatim
 *
 */
int32_t SDL_ESM_getRevisionId(uint32_t baseAddr, esmRevisionId_t *pRevId);

/**
 * \brief   This API is used read the ESM information register.
 *
 * \param   baseAddr        Base Address of the ESM Registers.
 *
 * \param   pInfo           pointer to variable that
 *                          Contains information register contents of ESM module..
 *                          Refer struct #esmInfo_t.
 *
 * \return                  SDL_PASS - success
 * @n                       SDL_EBADARGS - API fails due to bad input arguments
 *
 * \pre
 *   @n  None
 *
 * \post
 *   @n  None
 *
 * @b Example
   @verbatim

       uint32_t            baseAddr = SDL_MCU_ESM0_CFG_BASE;
       esmInfo_t           esmInfo;
       int32_t             sdlRet;
       sdlRet = ESMGetInfo (baseAddr, &esmInfo);

   @endverbatim
 *
 */
int32_t SDL_ESM_getInfo(uint32_t baseAddr, esmInfo_t *pInfo);


/**
 * \brief   This API is used read the ESM information register.
 *
 * \param   baseAddr        Base Address of the ESM Registers.
 *
 * \param   pStatus         pointer to is global Interrupt enabled status
 *                          0h: all interrupts are enabled
 *                          Fh: all interrupts are disabled
 *                         -others: interrupts are in invalid state.Software should never write
 *                          these values. If these values are ever read, they indicate that an
 *                          error has occurred. In this state, all interrupts are enabled (biased to
 *                          false enable).
 *
 * \return                  SDL_PASS - success
 * @n                       SDL_EBADARGS - API fails due to bad input arguments
 *
 * \pre
 *   @n  None
 *
 * \post
 *   @n  None
 *
 * @b Example
   @verbatim

       uint32_t            baseAddr = SDL_MCU_ESM0_CFG_BASE;
       uint32_t            enStatus;
       int32_t             sdlRet;
       sdlRet = ESMGetGlobalIntrEnabledStatus (baseAddr, &enStatus);

   @endverbatim
 *
 */
int32_t SDL_ESM_getGlobalIntrEnabledStatus(uint32_t baseAddr, uint32_t *pStatus);


/**
 * \brief   This API is used to enable Global control of interrupt.
 *
 * \param   baseAddr        Base Address of the ESM Registers.
 *
 * \return                  SDL_PASS - success
 * @n                       SDL_EBADARGS - API fails due to bad input arguments
 *
 * \pre
 *   @n  ESM module is reset and initialized for desired operation
 *
 * \post
 *   @n  None
 *
 * @b Example
   @verbatim

       uint32_t            baseAddr = SDL_MCU_ESM0_CFG_BASE;
       int32_t             sdlRet;
       sdlRet = ESMEnableGlobalIntr (baseAddr);

   @endverbatim
 *
 */
int32_t SDL_ESM_enableGlobalIntr(uint32_t baseAddr);

/**
 * \brief   This API is used to disable Global control of interrupt.
 *
 * \param   baseAddr        Base Address of the ESM Registers.
 *
 * \return                  SDL_PASS - success
 * @n                       SDL_EBADARGS - API fails due to bad input arguments
 *
 * \pre
 *   @n  ESM module is reset and initialized for desired operation
 *
 * \post
 *   @n  None
 *
 * @b Example
   @verbatim

       uint32_t            baseAddr = SDL_MCU_ESM0_CFG_BASE;
       uint32_t            sdlRet;
       sdlRet = ESMDisableGlobalIntr (baseAddr);

   @endverbatim
 *
 */
int32_t SDL_ESM_disableGlobalIntr(uint32_t baseAddr);

/**
 * \brief   This API is used to reset ESM module.
 *          Reset is used to reset all enables and raw status bits.
 *
 * \param   baseAddr        Base Address of the ESM Registers.
 *
 * \return                  SDL_PASS - success
 * @n                       SDL_EBADARGS - API fails due to bad input arguments
 *
 * \pre
 *   @n  None
 *
 * \post
 *   @n  None
 *
 * @b Example
   @verbatim

       uint32_t            baseAddr = SDL_MCU_ESM0_CFG_BASE;
       uint32_t            sdlRet;
       sdlRet = ESMReset (baseAddr);

   @endverbatim
 *
 */
int32_t SDL_ESM_reset(uint32_t baseAddr);

/** @} */

#ifdef __cplusplus
}

#endif /*extern "C" */

#endif /*SDL_ESM_H*/