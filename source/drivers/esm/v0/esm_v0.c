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
 *  \file esm_v0.c
 *
 *  \brief File containing ESM Driver APIs implementation for version v0.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
/* This is needed for memset/memcpy */
#include <string.h>
#include <drivers/esm.h>
#include <kernel/dpl/HwiP.h>
#include <drivers/soc.h>
#include <drivers/hw_include/hw_types.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
/** \brief Get ESM enable error pin action/response register offset */
#define ESM_ESMIEPSR(m)            ((uint32_t) ESM_ESMIEPSR1 + \
                                    (((m) / 32U) * 0x40U))
/** \brief Get ESM disable error pin action/response register offset */
#define ESM_ESMIEPCR(m)            ((uint32_t) ESM_ESMIEPCR1 + \
                                    (((m) / 32U) * 0x40U))
/** \brief Get ESM interrupt enable set/status register offset */
#define ESM_ESMIESR(m)             ((uint32_t) ESM_ESMIESR1 + \
                                    (((m) / 32U) * 0x40U))
/** \brief Get ESM interrupt enable clear/status register offset */
#define ESM_ESMIECR(m)             ((uint32_t) ESM_ESMIECR1 + \
                                    (((m) / 32U) * 0x40U))
/** \brief Get ESM interrupt level set/status register offset */
#define ESM_ESMILSR(m)             ((uint32_t) ESM_ESMILSR1 + \
                                    (((m) / 32U) * 0x40U))
/** \brief Get ESM interrupt level clear/status register offset */
#define ESM_ESMILCR(m)             ((uint32_t) ESM_ESMILCR1 + \
                                    (((m) / 32U) * 0x40U))
/** \brief Get ESM status register offset */
#define ESM_ESMSR(m)               ((uint32_t) ESM_ESMSR1 + \
                                    (((m) / 32U) * 0x40U))
/** \brief Maximum number of elements */
#define ESM_ESMSR_NUM_ELEMS        (4U)
/** \brief ESM gating operation related definitions */
/** \brief Four bits are used for each ESM event, creating a mask of 0xF */
#define ESM_GATING_MASK   		(0xFU)
/** \brief 4-bit Shift from one ESM event to the next one */
#define ESM_GATING_SHIFT  		(0x4U)
/** \brief Each ESM_GATING register handles 8 ESM events */
#define ESM_NUM_EVTS_PER_GATING_REG  	(0x8U)
/** \brief 4 ESM_GATING registers for group 2, followed by the registers for group 3 */
#define ESM_GATING_GROUP  (0x4U)  
/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */


/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
/* Driver internal functions */
static void ESM_highpriority_interrupt(void *args);
static uint32_t ESM_getHighPriorityLvlIntrStatus(uint32_t baseAddr);
static void ESM_setInfluenceOnErrPin(uint32_t baseAddr, 
                                    uint32_t intrSrc,
                                    Bool     influence);
static void ESM_enableIntr(uint32_t baseAddr, 
                          uint32_t intrSrc);  
static void ESM_disableIntr(uint32_t baseAddr, 
                           uint32_t intrSrc);
static void ESM_setIntrPriorityLvl(uint32_t baseAddr, 
                                  uint32_t intrSrc,
                                  uint32_t intrPriorityLvl);
static void ESM_clearIntrStatus(uint32_t baseAddr, 
                               uint32_t intrSrc);
static int32_t ESM_clearGroupIntrStatus(uint32_t baseAddr, 
                                       uint32_t grpNum);
static uint32_t ESM_getLowPriorityLvlIntrStatus(uint32_t baseAddr);
static void ESM_lowpriority_interrupt(void *args);
static void ESM_processInterrupt (void *arg, 
                                  uint32_t vec, 
                                  int32_t  *groupNum, 
                                  int32_t* vecNum);
static int32_t ESM_configErrorGating(ESM_Handle handle, 
                                 uint8_t groupNumber, 
                                 uint8_t errorNumber, 
                                 uint8_t gating);
/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
void ESM_init(void)
{
    return;
}

void ESM_deinit(void)
{
    return;
}

ESM_Handle ESM_open(uint32_t index, ESM_OpenParams *params)
{
    int32_t                status = SystemP_SUCCESS;
    uint32_t               count;
    ESM_Handle             handle = NULL;
    ESM_Config             *ptrESMConfig;
    ESM_Object             *object;
    const ESM_Attrs        *hwAttrs;
    HwiP_Params            hwiPrms;

    if(index >= gEsmConfigNum)
    {
        status = SystemP_FAILURE;
    }
    else
    {
        ptrESMConfig = &gEsmConfig[index];
    }
    
    if(SystemP_SUCCESS == status)
    {
        /* Validate the driver object configuration */
        DebugP_assert (ptrESMConfig->object != NULL);
        object  = ptrESMConfig->object;
        
        /* Initialize the memory */
        memset ((void *)object, 0, sizeof(ESM_Object));
        
        /* Validate the driver object configuration */
        DebugP_assert (ptrESMConfig->hwAttrs != NULL);
        hwAttrs = ptrESMConfig->hwAttrs;
        
        DebugP_assert(hwAttrs->ptrESMRegs != NULL);
        object->esmBaseAddr = (uint32_t)hwAttrs->ptrESMRegs;
        
        handle = (ESM_Handle) ptrESMConfig;
        /* Init state */
        object->esmHandle = (ESM_Handle)ptrESMConfig;
        
        object->numGroup1Err = hwAttrs->numGroup1Err;
        
        if(NULL != params)
        {
            /* copy params into the driver object structure */
            memcpy(&object->params, params, sizeof(ESM_OpenParams));
        }
        else
        {
            /* Init with default if NULL is passed */
            ESM_Params_init(&object->params);
        }

        /* Register ESM_highpriority_interrupt for MSS only. For DSS,
         * the ESM high priority interrupt is an NMI and application
         * needs to populate the NMI exception handler to hook up
         * ESM_highpriority_interrupt by calling the HwiP_construct
         * with hwiPrms.intNum = 1. Driver doesnot register NMI handler
         * as the nmi handler will be common for multiple sources of NMI. */
#if !defined(_TMS320C6X)
        /* Initialize with defaults */
        HwiP_Params_init(&hwiPrms);

        /* Populate the interrupt parameters */
        hwiPrms.callback   = &ESM_highpriority_interrupt;
        hwiPrms.args       = (void*)handle;
        hwiPrms.priority   = hwAttrs->intrHighPriority;
        hwiPrms.intNum     = (int32_t)hwAttrs->highPrioIntNum;
        
        /* Register interrupts */
        status = HwiP_construct(&object->hwiHiObj, &hwiPrms);
        if(SystemP_SUCCESS != status)
        {
            DebugP_log("Debug: ESM Driver Registering HWI(High Priority) ISR [%p] for Interrupt %d\n",
                     (uintptr_t)object->hwiHandleHi, hwAttrs->highPrioIntNum);
        }
        else
        {
            object->hwiHandleHi = &object->hwiHiObj;
        }
#else
        /* register the NMI Handler. */
        status = HwiP_registerNmiHandler(&ESM_highpriority_interrupt, (void*)handle);
        if(SystemP_SUCCESS != status)
        {
            DebugP_log("Debug: ESM Driver Registering NMI ESM_HI Failed. \n");
        }
#endif
        /* Register Low priroty interrupt for mss and dss */
        /* Initialize with defaults */
        HwiP_Params_init(&hwiPrms);

        /* Populate the interrupt parameters */
        hwiPrms.callback    = ESM_lowpriority_interrupt;
        hwiPrms.args        = (void*)handle;
        hwiPrms.priority    = hwAttrs->intrLowPriority;;
        hwiPrms.intNum      = (int32_t)hwAttrs->lowPrioIntNum;
        
        /* Register interrupts */
        status = HwiP_construct(&object->hwiLoObj, &hwiPrms);
        if(SystemP_SUCCESS != status)
        {
            DebugP_log("Debug: ESM Driver Registering HWI(Low Priority) ISR [%p] for Interrupt %d\n",
                     (uintptr_t)object->hwiHandleLo, hwAttrs->lowPrioIntNum);
        }
        else
        {
            object->hwiHandleLo = &object->hwiLoObj;
        }
            
        if (object->params.bClearErrors == TRUE)
        {
            /* Clear ESM Group 1, 2, 3 errors */
            for (count=0; count<ESM_NUM_GROUP_MAX; count++)
            {
                ESM_clearGroupIntrStatus(object->esmBaseAddr, count+1);
            }
        }
    }

    return handle;
}

void ESM_close(ESM_Handle handle)
{
    ESM_Config          *ptrESMConfig;
    ESM_Object          *object;

    /* Sanity check handle */
    DebugP_assert(handle != NULL);
    
    /* Get the pointer to the object and hwAttrs */
    ptrESMConfig = (ESM_Config*)handle;
    object  = (ESM_Object*)ptrESMConfig->object;

#if !defined(_TMS320C6X)
    /* Was the HWI registered?  */
    if (object->hwiHandleHi)
    {
        /* YES: Delete and unregister the interrupt handler. */
        HwiP_destruct(&object->hwiHiObj);
        object->hwiHandleHi = NULL;
    }
#else
        /* unregister the NMI Handler. */
        HwiP_unregisterNmiHandler();
#endif
    /* Was the HWI registered?  */
    if (object->hwiHandleLo)
    {
        /* YES: Delete and unregister the interrupt handler. */
        HwiP_destruct(&object->hwiLoObj);
        object->hwiHandleLo = NULL;
    }

}

int32_t ESM_registerNotifier(ESM_Handle handle, ESM_NotifyParams* params, int32_t* errCode)
{
    ESM_Config          *ptrESMConfig;
    ESM_Object          *object;
    int32_t             retVal = SystemP_SUCCESS;
    uintptr_t           key;
    int32_t             notifyIndex;

    if ((handle == NULL) || (params == NULL))
    {
        *errCode = ESM_EINVAL;
        retVal = SystemP_FAILURE;
    }
    else
    {
        /* Get the pointer to the object */
        ptrESMConfig = (ESM_Config*)handle;
        object  = (ESM_Object*)ptrESMConfig->object;

        /* Critical Section Protection: Notify registration needs to be
         * protected against multiple threads */
        key = HwiP_disable();

        /* Find a free notifier index */
        for (notifyIndex = 0; notifyIndex < ESM_MAX_NOTIFIERS; notifyIndex++)
        {
            if (object->notifyParams[notifyIndex].groupNumber == 0)
            {
                break;
            }
        }

        if (notifyIndex == ESM_MAX_NOTIFIERS)
        {
            /* Max allowed notifiers have already been registered */
            *errCode = ESM_ENOMEM;
            retVal = SystemP_FAILURE;
        }
        else
        {
            /* Check if the notifier handles group 1 or group 2 errors.
               Group 2 errors are enabled by default. Group 1 errors have to be explicitly enabled.
               Also, user can configure the interrupt priority level and influence on ERROR pin for group 1 errors.
               For group 2 errors, the interrupt priority level is always high, and the influence on ERROR pin is on always.
             */
            if (params->groupNumber == 1)
            {
                ESM_enableIntr(object->esmBaseAddr, params->errorNumber);
                /* Configure the interrupt priority level */
                ESM_setIntrPriorityLvl(object->esmBaseAddr, params->errorNumber, params->setIntrPriorityLvl);
                /* Configure the failure influence on ERROR pin */
                ESM_setInfluenceOnErrPin(object->esmBaseAddr, params->errorNumber, params->enableInfluenceOnErrPin);
            }
            /* Unmask Group 2 ESM errors to enable the generation of NMI. */
            if (params->groupNumber == 2)
            {
                retVal = ESM_configErrorGating(handle, params->groupNumber, params->errorNumber, 0);
            }

            memcpy ((void *)&object->notifyParams[notifyIndex], (void *)params, sizeof (ESM_NotifyParams));
        }
        /* Release the critical section: */
        HwiP_restore(key);
    }
    return retVal;
}

int32_t ESM_deregisterNotifier(ESM_Handle handle, int32_t notifyIndex, int32_t* errCode)
{
    ESM_Config          *ptrESMConfig;
    ESM_Object          *object;
    int32_t             retVal = SystemP_SUCCESS;
    uintptr_t           key;
    uint32_t            groupNumber;
    uint32_t            errorNumber;

    if ((handle == NULL) || (notifyIndex >= ESM_MAX_NOTIFIERS))
    {
        *errCode = ESM_EINVAL;
        retVal = SystemP_FAILURE;
    }
    else
    {
        /* Get the pointer to the object */
        ptrESMConfig = (ESM_Config*)handle;
        object  = (ESM_Object*)ptrESMConfig->object;

        /* Critical Section Protection: Notify registration needs to be
         * protected against multiple threads */
        key = HwiP_disable();

        groupNumber = object->notifyParams[notifyIndex].groupNumber;
        errorNumber = object->notifyParams[notifyIndex].errorNumber;

        /* Check if the notifier is to handle group 1 or group 2 errors.
         * Group 2 errors are enabled by default. Group 1 errors was explicitly enabled
         * and now needs to be disabled. Also reset the interrupt priority level to low
         * and disable the influence on ERROR pin.
         */
        if (groupNumber == 1)
        {
            ESM_disableIntr(object->esmBaseAddr, errorNumber);
            ESM_setIntrPriorityLvl(object->esmBaseAddr, errorNumber, 0);
            ESM_setInfluenceOnErrPin(object->esmBaseAddr, errorNumber, 0);
        }
        /* Gating Group 2 ESM errors to disable the generation of NMI. */
        if (groupNumber == 2)
        {
            ESM_configErrorGating(handle, groupNumber, errorNumber, 1);
        }
        memset ((void *)&object->notifyParams[notifyIndex], 0, sizeof (ESM_NotifyParams));

        /* Release the critical section: */
        HwiP_restore(key);
    }
    return retVal;
}

void ESM_setMode(uint32_t baseAddr, uint32_t mode)
{
    HW_WR_FIELD32(baseAddr + ESM_ESMEKR, ESM_ESMEKR_EKEY, mode);
}

int32_t ESM_setErrPinLowTimePreload(uint32_t baseAddr, uint32_t lowTime)
{
    int32_t status = SystemP_FAILURE;

    if (ESM_ESMLTCPR_LTCPR_MAX >= lowTime)
    {
        HW_WR_FIELD32(baseAddr + ESM_ESMLTCPR, ESM_ESMLTCPR_LTCPR, lowTime);
        status = SystemP_SUCCESS;
    }
    return status;
}

uint32_t ESM_getCurrErrPinLowTimeCnt(uint32_t baseAddr)
{
    return HW_RD_FIELD32(baseAddr + ESM_ESMLTCR, ESM_ESMLTCR_LTC);
}

uint32_t ESM_getErrPinStatus(uint32_t baseAddr)
{
    return HW_RD_FIELD32(baseAddr + ESM_ESMEPSR, ESM_ESMEPSR_EPSF);
}

void ESM_resetErrPin(uint32_t baseAddr)
{
    HW_WR_FIELD32(baseAddr + ESM_ESMEKR, ESM_ESMEKR_EKEY,
                  ESM_ESMEKR_EKEY_ERROR_PIN_RESET);
}

uint32_t ESM_getIntrStatus(uint32_t baseAddr, uint32_t intrSrc)
{
    uint32_t regVal;

    regVal  = HW_RD_REG32(baseAddr + ESM_ESMSR(intrSrc));
    regVal &= ((uint32_t) TRUE << (intrSrc % 32U));
    regVal  = (regVal >> (intrSrc % 32U));
    return regVal;
}

int32_t ESM_getGroupIntrStatus(uint32_t baseAddr, 
                              uint32_t grpNum,
                              ESM_GroupIntrStatus *intrstatus)
{
    int32_t  status  = SystemP_FAILURE;
    uint32_t loopCnt = 0;

    if (ESM_NUM_GROUP_MAX < grpNum)
    {
        status = SystemP_FAILURE;
    }
    else
    {
        switch (grpNum)
        {
            case 1U:
                for (loopCnt = 0; loopCnt < ESM_ESMSR_NUM_ELEMS; loopCnt++)
                {
                    intrstatus->grpIntrStatus[loopCnt] =
                        HW_RD_REG32(baseAddr + ESM_ESMSR(loopCnt * 32U));
                }
                status = SystemP_SUCCESS;
                break;
            case 2U:
                status = SystemP_SUCCESS;
                intrstatus->grpIntrStatus[0] = HW_RD_REG32(baseAddr + ESM_ESMSR2);
                break;
            case 3U:
                status = SystemP_FAILURE;
                intrstatus->grpIntrStatus[0] = HW_RD_REG32(baseAddr + ESM_ESMSR3);
                break;
            default:
                status = SystemP_FAILURE;
                break;
        }
    }
    return status;
}

/* ========================================================================== */
/*                          Static Function Definitions                       */
/* ========================================================================== */
static void ESM_highpriority_interrupt(void *args)
{
    uint32_t            esmioffhr;
    uint32_t            vec;
    int32_t             groupNum = -1, vecNum = -1;
    ESM_Config          *ptrESMConfig;
    ESM_Object          *object;

    /* Get the ESM Configuration: */
    ptrESMConfig = (ESM_Config*)args;
    object  = (ESM_Object*)ptrESMConfig->object;

    esmioffhr = ESM_getHighPriorityLvlIntrStatus(object->esmBaseAddr);
    vec = esmioffhr - 1U;

    ESM_processInterrupt(ptrESMConfig, vec, &groupNum, &vecNum);
}

static uint32_t ESM_getHighPriorityLvlIntrStatus(uint32_t baseAddr)
{
    return HW_RD_FIELD32(baseAddr + ESM_ESMIOFFHR, ESM_ESMIOFFHR_INTOFFH);
}

static void ESM_setInfluenceOnErrPin(uint32_t baseAddr, 
                                     uint32_t intrSrc,
                                     Bool     influence)
{
    uint32_t regVal;

    if ((uint32_t) TRUE == influence)
    {
        regVal  = HW_RD_REG32(baseAddr + ESM_ESMIEPSR(intrSrc));
        regVal &= ~((uint32_t) TRUE << (intrSrc % 32U));
        regVal |= ((uint32_t) 0x1U << (intrSrc % 32U));
        HW_WR_REG32(baseAddr + ESM_ESMIEPSR(intrSrc), regVal);
    }
    else
    {
        regVal  = HW_RD_REG32(baseAddr + ESM_ESMIEPCR(intrSrc));
        regVal &= ~((uint32_t) TRUE << (intrSrc % 32U));
        regVal |= ((uint32_t) TRUE << (intrSrc % 32U));
        HW_WR_REG32(baseAddr + ESM_ESMIEPCR(intrSrc), regVal);
    }
}

static void ESM_enableIntr(uint32_t baseAddr, uint32_t intrSrc)
{
    uint32_t regVal;

    regVal  = HW_RD_REG32(baseAddr + ESM_ESMIESR(intrSrc));
    regVal &= ~((uint32_t) TRUE << (intrSrc % 32U));
    regVal |= ((uint32_t) TRUE << (intrSrc % 32U));
    HW_WR_REG32(baseAddr + ESM_ESMIESR(intrSrc), regVal);
}

static void ESM_disableIntr(uint32_t baseAddr, uint32_t intrSrc)
{
    uint32_t regVal;

    regVal  = HW_RD_REG32(baseAddr + ESM_ESMIECR(intrSrc));
    regVal &= ~((uint32_t) TRUE << (intrSrc % 32U));
    regVal |= ((uint32_t) TRUE << (intrSrc % 32U));
    HW_WR_REG32(baseAddr + ESM_ESMIECR(intrSrc), regVal);
}

static void ESM_setIntrPriorityLvl(uint32_t baseAddr, 
                                  uint32_t intrSrc,
                                  uint32_t intrPriorityLvl)
{
    uint32_t regVal;

    if (ESM_INTR_PRIORITY_LEVEL_LOW == intrPriorityLvl)
    {
        regVal  = HW_RD_REG32(baseAddr + ESM_ESMILCR(intrSrc));
        regVal &= ~((uint32_t) TRUE << (intrSrc % 32U));
        regVal |= ((uint32_t) TRUE << (intrSrc % 32U));
        HW_WR_REG32(baseAddr + ESM_ESMILCR(intrSrc), regVal);
    }
    else
    {
        regVal  = HW_RD_REG32(baseAddr + ESM_ESMILSR(intrSrc));
        regVal &= ~((uint32_t) TRUE << (intrSrc % 32U));
        regVal |= ((uint32_t) TRUE << (intrSrc % 32U));
        HW_WR_REG32(baseAddr + ESM_ESMILSR(intrSrc), regVal);
    }
}

static void ESM_clearIntrStatus(uint32_t baseAddr, uint32_t intrSrc)
{
    uint32_t regVal = 0U;

    regVal = ((uint32_t) TRUE << (intrSrc % 32U));
    HW_WR_REG32(baseAddr + ESM_ESMSR(intrSrc), regVal);
}

static int32_t ESM_clearGroupIntrStatus(uint32_t baseAddr, uint32_t grpNum)
{
    int32_t  status  = SystemP_FAILURE;
    uint32_t loopCnt = 0;
    uint32_t regVal  = 0xFFFFFFFFU;

    if (ESM_NUM_GROUP_MAX < grpNum)
    {
        status = SystemP_FAILURE;
    }
    else
    {
        switch (grpNum)
        {
            case 1U:
                for (loopCnt = 0; loopCnt < ESM_ESMSR_NUM_ELEMS; loopCnt++)
                {
                    HW_WR_REG32(baseAddr + ESM_ESMSR(loopCnt * 32U), regVal);
                }
                status = SystemP_SUCCESS;
                break;
            case 2U:
                status = SystemP_SUCCESS;
                HW_WR_REG32(baseAddr + ESM_ESMSR2, regVal);
                break;
            case 3U:
                status = SystemP_SUCCESS;
                HW_WR_REG32(baseAddr + ESM_ESMSR3, regVal);
                break;
            default:
                status = SystemP_FAILURE;
                break;
        }
    }
    return status;
}

static uint32_t ESM_getLowPriorityLvlIntrStatus(uint32_t baseAddr)
{
    return HW_RD_FIELD32(baseAddr + ESM_ESMIOFFLR, ESM_ESMIOFFLR_INTOFFL);
}

static void ESM_lowpriority_interrupt(void *args)
{
    uint32_t            esmioffhr;
    uint32_t            vec;
    int32_t             groupNum = -1, vecNum = -1;
    ESM_Config          *ptrESMConfig;
    ESM_Object          *object;

    /* Get the ESM Configuration: */
    ptrESMConfig = (ESM_Config*)args;
    object  = (ESM_Object*)ptrESMConfig->object;

    esmioffhr = ESM_getLowPriorityLvlIntrStatus(object->esmBaseAddr);
    vec = esmioffhr - 1U;

    ESM_processInterrupt(ptrESMConfig, vec, &groupNum, &vecNum);
}

static void ESM_processInterrupt (void *arg, uint32_t vec, int32_t* groupNum, int32_t *vecNum)
{
    uint32_t            index;
    ESM_Config          *ptrESMConfig;
    ESM_Object          *object;

    /* Get the ESM Configuration: */
    ptrESMConfig = (ESM_Config*)arg;
    object  = (ESM_Object*)ptrESMConfig->object;

    /* Error numbers are extracted from the INTOFFH/INTOFFL depending upon the 
     * processing of high or low priority interrupts
     * below info provides extracting the error number and group from these registers
     * Reg Val : Group
     * 1-32    : GRP1 (0 - 31)
     * 33-64   : GRP2 (0 - 31)
     * 65-96   : GRP1 (32 - 63)
     * 97-128  : Reserved
     * 129-160 : GRP1 (64-95)
     * 161-192 : Reserved
     * 193-224 : GRP1 (96-127)
     * >224    : Reserved
    */
    if (vec < 32U)
    {
        /* group 1 0-31 errors */
        object->debugEsmISRCount[0]++;
        *groupNum = 1;
        *vecNum = vec;
    }
    else if (vec < 64U)
    {
        /* group 2 0-31 errors */
        object->debugEsmISRCount[1]++;
        vec = vec - 32;
        *groupNum = 2;
        *vecNum = vec;
    }
    else if (vec < 96U)
    {
        /* group 1 32-63 errors */
        object->debugEsmISRCount[((vec/32) % ESM_MAX_ISR_COUNT)]++;
        vec = vec - 32;
        *groupNum = 1;
        *vecNum = vec;
    }
    else if (vec < 128U)
    {
        /* Reserved */
        DebugP_assert(0);
    }
    else if (vec < 160U)
    {
        /* group 1 64-95 errors */
        object->debugEsmISRCount[((vec/64) % ESM_MAX_ISR_COUNT)]++;
        vec = vec - 64;
        *groupNum = 1;
        *vecNum = vec;
    }
    else if (vec < 192U)
    {
        /* Reserved */
        DebugP_assert(0);
    }
    else if (vec < 224U)
    {
        /* group 1 96-127 errors */
        object->debugEsmISRCount[((vec/96) % ESM_MAX_ISR_COUNT)]++;
        vec = vec - 96;
        *groupNum = 1;
        *vecNum = vec;
    }
    else
    {
        /* Invalid group errors */ 
        DebugP_assert(0);
    }
    
    if (*groupNum != -1)
    {
        /* Clear the error status flag for group 1 errors. There is no need to clear group 2 errors,
         * since the error status in ESMSR2 has been cleared when reading the appropriate vector
         * in the ESMIOFFHR offset register (via ESM_getHighPriorityLvlIntrStatus()).
         */
        if (*groupNum == 1)
        {
            ESM_clearIntrStatus(object->esmBaseAddr, *vecNum);
        }
        /* Check if notify function was registered? */
        for (index = 0; index < ESM_MAX_NOTIFIERS; index++)
        {
            if ((*vecNum == object->notifyParams[index].errorNumber) &&
                (*groupNum == object->notifyParams[index].groupNumber))
            {
                object->notifyParams[index].notify(object->notifyParams[index].arg);
                break;
            }
        }
    }
}

static int32_t ESM_configErrorGating(ESM_Handle handle, uint8_t groupNumber, uint8_t errorNumber, uint8_t gating)
{
    uint32_t            regVal;
    uint32_t            regIndex;
    uint32_t            regAddr;
    int32_t             retVal = SystemP_SUCCESS;
    ESM_Config          *ptrESMConfig;
    CSL_mss_ctrlRegs    *ptrCtrlRegs;
    const ESM_Attrs     *hwAttrs;

    ptrESMConfig = (ESM_Config*)handle;
    hwAttrs = ptrESMConfig->hwAttrs;
    ptrCtrlRegs = hwAttrs->ptrCtrlRegs;
    
    /* Error gating is for Group 2  and Group 3 errors only */
    if ((groupNumber < 2) || (groupNumber > 3))
    {
        retVal = SystemP_FAILURE;
    }

    /* For Group 2 and Group 3, there are up to 32 errors for each*/
    if (errorNumber > 32)
    {
        retVal = SystemP_FAILURE;
    }

    if (SystemP_SUCCESS == retVal)
    {
        regIndex = errorNumber / ESM_NUM_EVTS_PER_GATING_REG + (groupNumber-2) * ESM_GATING_GROUP;
        regAddr  = (uint32_t)&ptrCtrlRegs->ESM_GATING0 + regIndex * (sizeof(uint32_t));

        regVal = CSL_REG_RD((volatile uint32_t *)regAddr);
        regVal &= ~(ESM_GATING_MASK << (ESM_GATING_SHIFT*(errorNumber % ESM_NUM_EVTS_PER_GATING_REG)));
        if (gating)
        {
            regVal |= (ESM_GATING_MASK << (ESM_GATING_SHIFT*(errorNumber % ESM_NUM_EVTS_PER_GATING_REG)));
        }
        CSL_REG_WR((volatile uint32_t *)regAddr, regVal);
    }

    return retVal;
}
