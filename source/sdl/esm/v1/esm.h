/*
 *   Copyright (c) Texas Instruments Incorporated 2022
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
#include <sdl/include/hw_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */


/* Local Defines */
#define BITS_PER_WORD (32u)
#define ESM_INTR_GRP_NUM (32U)
#define GROUP_NUMBER_BIT_SHIFT  (5u)
#define NO_EVENT_VALUE (0xffffu)
#define SDL_ESM_EN_KEY_ENBALE_VAL (0xFU)
#define INVALID_BIT (0u)
#define FLAG_NO (0u)
#define FLAG_YES (1u)

#define ESM_NUM_INTR_PER_GRP              (32U)
#define ESM_MAX_NUM_INTRS                 (1024U)
#define ESM_ESM_PIN_CTRL_KEY_RESET_VAL    (0x5U)
#define ESM_SFT_RST_KEY_RESET_VAL         (0xFU)
#define ESM_EN_KEY_MASK                   (0xFU)
#define ESM_EN_KEY_ENBALE_VAL             (0xFU)
#define ESM_EN_KEY_DISABLE_VAL            (0x0U)


/** \brief Get ESM enable error pin action/response register offset */
#define SDL_ESM_ESMIEPSR(m)            ((uint32_t) ESM_ESMIEPSR1 + \
                                    (((m) / 32U) * 0x40U))
/** \brief Get ESM disable error pin action/response register offset */
#define SDL_ESM_ESMIEPCR(m)            ((uint32_t) ESM_ESMIEPCR1 + \
                                    (((m) / 32U) * 0x40U))
/** \brief Get ESM interrupt enable set/status register offset */
#define SDL_ESM_ESMIESR(m)             ((uint32_t) ESM_ESMIESR1 + \
                                    (((m) / 32U) * 0x40U))
/** \brief Get ESM interrupt enable clear/status register offset */
#define SDL_ESM_ESMIECR(m)             ((uint32_t) ESM_ESMIECR1 + \
                                    (((m) / 32U) * 0x40U))
/** \brief Get ESM interrupt level set/status register offset */
#define SDL_ESM_ESMILSR(m)             ((uint32_t) ESM_ESMILSR1 + \
                                    (((m) / 32U) * 0x40U))
/** \brief Get ESM interrupt level clear/status register offset */
#define SDL_ESM_ESMILCR(m)             ((uint32_t) ESM_ESMILCR1 + \
                                    (((m) / 32U) * 0x40U))
/** \brief Get ESM status register offset */
#define SDL_ESM_ESMSR(m)               ((uint32_t) ESM_ESMSR1 + \
                                    (((m) / 32U) * 0x40U))

/** \brief Maximum number of elements */
#define SDL_ESM_ESMSR_NUM_ELEMS        (4U)
/** \brief ESM gating operation related definitions */
/** \brief Four bits are used for each ESM event, creating a mask of 0xF */
#define ESM_GATING_MASK   		(0xFU)
/** \brief 4-bit Shift from one ESM event to the next one */
#define ESM_GATING_SHIFT  		(0x4U)
/** \brief Each ESM_GATING register handles 8 ESM events */
#define ESM_NUM_EVTS_PER_GATING_REG  	(0x8U)
/** \brief 4 ESM_GATING registers for group 2, followed by the registers for group 3 */
#define ESM_GATING_GROUP  (0x4U)  

#define BITS_PER_WORD (32u)

/**       
 * \brief  ESM Operation Mode type.
 */

#define ESM_NUMBER_OF_GROUP_REGS          (32u)

/**        
 * \brief ESM number of groups 
*/
#define SDL_ESM_NUM_GROUP_MAX                    (3U)
#define SDL_ESM_NUM_INTR_PER_GROUP               (128U)

/**
 *  \anchor esmOperationMode_t
 *  \name ESM Operation Mode type
 *  @{
 */

/**
 * \brief  ESM Operation Mode type.
 */
typedef uint32_t esmOperationMode_t;

#define SDL_ESM_OPERATION_MODE_NORMAL 0x0U
    /**< Configure ESM operation mode to normal mode */
#define SDL_ESM_OPERATION_MODE_ERROR_FORCE 0xAu
    /**< Configure ESM operation mode to error force mode */
/** @} */

/**
 *  \anchor esmIntrType_t
 *  \name ESM Interrupt Type to select level for interrupt.
 *  @{
 */

typedef uint32_t esmIntrType_t;

#define SDL_ESM_INTR_TYPE_LOW_PRIO_ERROR (0x1u)
    /**< Configure interrupt to high level interrupt */
#define SDL_ESM_INTR_TYPE_HIGH_PRIO_ERROR (0x2u)
    /**< Configure interrupt to low level interrupt */
/** @} */

/**
 *  \anchor esmIntrPriorityLvl_t
 *  \name ESM Interrupt Priority Levels
 *  @{
 */

typedef uint32_t esmIntrPriorityLvl_t;

#define SDL_ESM_INTR_PRIORITY_LEVEL_LOW  (0x0u)
    /**< Configure interrupt to low level interrupt */
#define SDL_ESM_INTR_PRIORITY_LEVEL_HIGH (0x1u)
    /**< Configure interrupt to high level interrupt */
/** @} */

/** \brief A handle that is returned from a ESM_Init() call */
typedef void *SDL_ESM_Handle;

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/**
 * \brief  Structure to access the status of interrupts belonging to a High or
 *         Low priority interrupt.
 */
typedef struct 
{
    uint32_t grpIntrStatus[SDL_ESM_NUM_INTR_PER_GROUP / ESM_NUM_INTR_PER_GRP];
    /**< Contains status for interrupts from a group */
}SDL_ESM_GroupIntrStatus;

/**
 * \brief  ESM static registers list.
 */
typedef struct {
    volatile uint32_t ESMIEPSR1;             /* ESM Enable ERROR Pin Action/Response Register 1 */
    volatile uint32_t ESMIEPCR1;             /* ESM Disable ERROR Pin Action/Response Register 1 */
    volatile uint32_t ESMIESR1;              /* ESM Interrupt Enable Set/Status Register 1 */
    volatile uint32_t ESMIECR1;              /* ESM Interrupt Enable Clear/Status Register 1 */
    volatile uint32_t ESMILSR1;              /* Interrupt Level Set/Status Register 1 */
    volatile uint32_t ESMILCR1;              /* Interrupt Level Clear/Status Register 1 */
    volatile uint32_t ESMSR1;                /* ESM Status Register 1 */
    volatile uint32_t ESMSR2;                /* ESM Status Register 2 */
    volatile uint32_t ESMSR3;                /* ESM Status Register 3 */
    volatile uint32_t ESMEPSR;               /* ESM ERROR Pin Status Register */
    volatile uint32_t ESMIOFFHR;             /* ESM Interrupt Offset High Register */
	volatile uint32_t ESMIOFFLR;			 /*	ESM Interrupt Offset Low Register */
	volatile uint32_t ESMLTCR;  			 /* ESM Low-Time Counter Register */
	volatile uint32_t ESMLTCPR;              /* ESM Low-Time Counter Preload Register */
	volatile uint32_t ESMEKR;                /* ESM Error Key Register */
	volatile uint32_t ESMSSR2;               /* ESM Status Shadow Register 2 */
	
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
       esmOperationMode_t  mode     = SDL_ESM_OPERATION_MODE_ERROR_FORCE;
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
 *                          Refer struct #SDL_ESM_GroupIntrStatus.
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
       SDL_ESM_GroupIntrStatus groupIntrStatus;
       int32_t             sdlRet;

       sdlRet = SDL_ESM_getGroupIntrStatus (baseAddr, priType, &groupIntrStatus);

   @endverbatim
 *
 */
int32_t SDL_ESM_getGroupIntrStatus(uint32_t baseAddr, esmIntrPriorityLvl_t intrPrioType,
                                   SDL_ESM_GroupIntrStatus *pIntrstatus);

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
 * \brief   This API is used to clear the interrupt/error status for a group.
 *          This will also return highest pending interrupt for pulse as well
 *          as for level interrupts.
 *
 * \param   baseAddr        Base Address of the ESM Registers.
 *
 * \param   grpNum    Group for which status to return.
 *                          Refer enum #esmIntrPriorityLvl_t.
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
       uint32_t  			grpNum  = GROUP_ONE;
       int32_t             sdlRet;

       sdlRet = SDL_ESM_clearGroupIntrStatus (baseAddr, grpNum);

   @endverbatim
 *
 */

int32_t SDL_ESM_clearGroupIntrStatus(uint32_t baseAddr, uint32_t grpNum);


/**
 * \brief   This API is used to get the low priority level interrupt status.
 *
 * \param   baseAddr        Base Address of the ESM Registers.
 *
 * \param   pstatus         pointer to interrupt status
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
       uint32_t            status;
       int32_t             sdlRet;
       sdlRet = SDL_ESM_getLowPriorityLvlIntrStatus (baseAddr, &status);

   @endverbatim
 *
 */

int32_t SDL_ESM_getLowPriorityLvlIntrStatus(uint32_t baseAddr, uint32_t *pstatus);

/**
 * \brief   This API is used to get the High priority level interrupt status.
 *
 * \param   baseAddr        Base Address of the ESM Registers.
 *
 * \param   pstatus         pointer to interrupt status
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
       uint32_t            status;
       int32_t             sdlRet;
       sdlRet = SDL_ESM_getHighPriorityLvlIntrStatus (baseAddr, &status);

   @endverbatim
 *
 */

int32_t SDL_ESM_getHighPriorityLvlIntrStatus(uint32_t baseAddr, uint32_t *pstatus);

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

/** @} */

#ifdef __cplusplus
}

#endif /*extern "C" */

#endif /*SDL_ESM_H*/
