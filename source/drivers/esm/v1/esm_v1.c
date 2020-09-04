/*
 *  Copyright (C) 2022 Texas Instruments Incorporated
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
 *  \file esm_v1.c
 *
 *  \brief File containing ESM Driver APIs implementation for version v1.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <string.h>
#include <drivers/esm.h>
#include <kernel/dpl/HwiP.h>
#include <drivers/soc.h>
#include <drivers/hw_include/hw_types.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define ESM_NUM_INTR_PER_GRP              (32U)
#define ESM_INTR_GRP_NUM                  (32U)
#define ESM_MAX_NUM_INTRS                 (1024U)
#define ESM_ESM_PIN_CTRL_KEY_RESET_VAL    (0x5U)
#define ESM_SFT_RST_KEY_RESET_VAL         (0xFU)
#define ESM_EN_KEY_MASK                   (0xFU)
#define ESM_EN_KEY_ENBALE_VAL             (0xFU)
#define ESM_EN_KEY_DISABLE_VAL            (0x0U)

#define NO_EVENT_VALUE                    (0xFFFFu)

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

/* Clear group interrupt status */
static int32_t ESM_clearGroupIntrStatus (uint32_t baseAddr, uint32_t grpNum);
/* High priority interrupt Handler */
static void ESM_highPriorityInterrupt (void *args);
/* Low priority interrupt Handler */
static void ESM_lowPriorityInterrupt (void *args);
/* Process error received */
static void ESM_processInterrupt (void * args, uint32_t priority);
/* Enable global interrupt */
static int32_t ESM_enableGlobalIntr(uint32_t baseAddr);
/* Set the influence of interrupt on ERROR pin. */
static int32_t ESM_setInfluenceOnErrPin (uint32_t baseAddr, uint32_t intrSrc,
                             Bool enable);
/* Enable interrupt */
static int32_t ESM_enableIntr(uint32_t baseAddr, uint32_t intrNum);
/* Disable interrupt */
static int32_t ESM_disableIntr(uint32_t baseAddr, uint32_t intrNum);
/* Set priority level for interrupt */
static int32_t ESM_setIntrPriorityLvl(uint32_t baseAddr, uint32_t intrSrc,
                           uint32_t intrPriorityLvl);

static int32_t ESM_clearIntrStatus(uint32_t baseAddr, uint32_t intrSrc);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void ESM_init (void)
{
    return;
}

void ESM_deinit (void)
{
    return;
}

ESM_Handle ESM_open (uint32_t index, ESM_OpenParams *params)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t               count;
    ESM_Handle             handle = NULL;
    ESM_Config             *ptrESMConfig;
    ESM_Object             *object = NULL;
    const ESM_Attrs        *hwAttrs;
    HwiP_Params            hwiPrms;

    if (index >= gEsmConfigNum)
    {
        status = SystemP_FAILURE;
    }
    else
    {
        ptrESMConfig = &gEsmConfig[index];
    }

    if (status == SystemP_SUCCESS)
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

        if (NULL != params)
        {
            memcpy (&object->params, params, sizeof(ESM_OpenParams));
        }
        else
        {
            /* Init with default if NULL is passed */
            ESM_Params_init(&object->params);
        }

        /* Register handlers for High and Low Priority ESM interrupts */
        HwiP_Params_init (&hwiPrms);

        /* Populate the interrupt parameters */
        hwiPrms.callback   = &ESM_highPriorityInterrupt;
        hwiPrms.args       = (void*)handle;
        hwiPrms.priority   = hwAttrs->intrHighPriority;
        hwiPrms.intNum     = (int32_t)hwAttrs->highPrioIntNum;

        /* Register interrupts */
        status = HwiP_construct (&object->hwiHiObj, &hwiPrms);
        if (status != SystemP_SUCCESS)
        {
            DebugP_logError("Debug: ESM Driver Registering HWI(High Priority) ISR [%p] for Interrupt %d\n",
                     (uintptr_t)object->hwiHandleHi, hwAttrs->highPrioIntNum);
        }
        else
        {
            object->hwiHandleHi = &object->hwiHiObj;
        }

        HwiP_Params_init(&hwiPrms);

        /* Populate the interrupt parameters */
        hwiPrms.callback    = ESM_lowPriorityInterrupt;
        hwiPrms.args        = (void*)handle;
        hwiPrms.priority    = hwAttrs->intrLowPriority;;
        hwiPrms.intNum      = (int32_t)hwAttrs->lowPrioIntNum;

        /* Register interrupts */
        status = HwiP_construct(&object->hwiHiObj, &hwiPrms);
        if(SystemP_SUCCESS != status)
        {
            DebugP_logError("Debug: ESM Driver Registering HWI(Low Priority) ISR [%p] for Interrupt %d\n",
                     (uintptr_t)object->hwiHandleLo, hwAttrs->lowPrioIntNum);
        }
        else
        {
            object->hwiHandleLo = &object->hwiLoObj;
        }

        if (status == SystemP_SUCCESS)
        {
            status = ESM_enableGlobalIntr (object->esmBaseAddr);
        }

        if (object->params.bClearErrors == TRUE)
        {
            for (count = 0; count < ESM_INTR_GRP_NUM; count++)
            {
                ESM_clearGroupIntrStatus (object->esmBaseAddr, count);
            }
        }

    }

    return handle;
}

void ESM_close (ESM_Handle handle)
{
    ESM_Config      *ptrESMConfig;
    ESM_Object      *object;

    /* Sanity check handle */
    DebugP_assert(handle != NULL);

    /* Get the pointer to the object and hwAttrs */
    ptrESMConfig = (ESM_Config*)handle;
    object  = (ESM_Object*)ptrESMConfig->object;

    /* Was the HWI registered?  */
    if (object->hwiHandleHi)
    {
        /* YES: Delete and unregister the interrupt handler. */
        HwiP_destruct(&object->hwiHiObj);
        object->hwiHandleHi = NULL;
    }
    /* Was the HWI registered?  */
    if (object->hwiHandleLo)
    {
        /* YES: Delete and unregister the interrupt handler. */
        HwiP_destruct(&object->hwiLoObj);
        object->hwiHandleLo = NULL;
    }
}

int32_t ESM_registerNotifier(ESM_Handle handle, ESM_NotifyParams* params)
{
    ESM_Config          *ptrESMConfig;
    ESM_Object          *object;
    int32_t             status = SystemP_SUCCESS;
    uintptr_t           key;
    uint32_t            notifyIndex;

    if ((handle == NULL) || (params == NULL))
    {
        status = SystemP_FAILURE;
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
            if (object->notifyParams[notifyIndex].notify == NULL)
            {
                break;
            }
        }

        if (notifyIndex == ESM_MAX_NOTIFIERS)
        {
            /* Max allowed notifiers have already been registered */
            status = SystemP_FAILURE;
        }
        else
        {
            /* Enable interrupt */
            status = ESM_enableIntr (object->esmBaseAddr, params->errorNumber);

            if (status == SystemP_SUCCESS)
            {
                /* Set interrupt priority */
                status = ESM_setIntrPriorityLvl (object->esmBaseAddr, params->errorNumber, params->setIntrPriorityLvl);
            }
            if (status == SystemP_SUCCESS)
            {
                /* Configure the failure influence on ERROR pin */
                status = ESM_setInfluenceOnErrPin (object->esmBaseAddr, params->errorNumber, params->enableInfluenceOnErrPin);
            }

            memcpy ((void *)&object->notifyParams[notifyIndex], (void *)params, sizeof (ESM_NotifyParams));
        }

        /* Release the critical section: */
        HwiP_restore(key);
    }

    return status;
}

int32_t ESM_deregisterNotifier(ESM_Handle handle, int32_t notifyIndex)
{
    ESM_Config          *ptrESMConfig;
    ESM_Object          *object;
    int32_t             status = SystemP_SUCCESS;
    uintptr_t           key;
    uint32_t            errorNumber;

    if ((handle == NULL) || (notifyIndex >= ESM_MAX_NOTIFIERS))
    {
        status = SystemP_FAILURE;
    }
    else
    {
        /* Get the pointer to the object */
        ptrESMConfig = (ESM_Config*)handle;
        object  = (ESM_Object*)ptrESMConfig->object;

        /* Critical Section Protection: Notify registration needs to be
         * protected against multiple threads */
        key = HwiP_disable();

        errorNumber = object->notifyParams[notifyIndex].errorNumber;


        ESM_disableIntr(object->esmBaseAddr, errorNumber);
        ESM_setIntrPriorityLvl(object->esmBaseAddr, errorNumber, 0);
        ESM_setInfluenceOnErrPin(object->esmBaseAddr, errorNumber, 0);

        memset ((void *)&object->notifyParams[notifyIndex], 0, sizeof (ESM_NotifyParams));

        /* Release the critical section: */
        HwiP_restore(key);
    }

    return status;
}

int32_t ESM_setMode (uint32_t baseAddr, uint32_t mode)
{
    int32_t    status = SystemP_FAILURE;
    uint32_t   regVal;

    /* Map the mode passed to supported values */
    if (mode == ESM_OPERATION_MODE_NORMAL)
    {
        regVal = ((uint32_t)(ESM_OPERATION_MODE_NORMAL));
    }
    else
    {
        regVal = ((uint32_t)(ESM_OPERATION_MODE_ERROR_FORCE));
    }

    if (baseAddr != ((uint32_t) (0u)))
    {
        HW_WR_FIELD32(baseAddr + CSL_ESM_PIN_CTRL, CSL_ESM_PIN_CTRL_KEY, regVal);
        status = SystemP_SUCCESS;
    }
    return (status);
}

int32_t ESM_setErrPinLowTimePreload(uint32_t baseAddr, uint32_t lowTime)
{
    int32_t status = SystemP_FAILURE;

    if (baseAddr != ((uint32_t) (0u)))
    {
        if (CSL_ESM_PIN_CNTR_PRE_COUNT_MAX >= lowTime)
        {
            HW_WR_FIELD32(baseAddr + CSL_ESM_PIN_CNTR_PRE,
                          CSL_ESM_PIN_CNTR_PRE_COUNT,
                          lowTime);
            status = SystemP_SUCCESS;
        }
    }
    return status;
}

uint32_t ESM_getCurrErrPinLowTimeCnt (uint32_t baseAddr)
{
    return HW_RD_FIELD32(baseAddr + CSL_ESM_PIN_CNTR, CSL_ESM_PIN_CNTR_COUNT);
}

uint32_t ESM_getErrPinStatus (uint32_t baseAddr)
{
    return HW_RD_FIELD32(baseAddr + CSL_ESM_PIN_STS, CSL_ESM_PIN_STS_VAL);
}

void ESM_resetErrPin (uint32_t baseAddr)
{
    HW_WR_FIELD32(baseAddr + CSL_ESM_PIN_CTRL, CSL_ESM_PIN_CTRL_KEY,
                      ESM_ESM_PIN_CTRL_KEY_RESET_VAL);
}

int32_t ESM_setIntrPriorityLvl(uint32_t baseAddr, uint32_t intrSrc,
                           uint32_t intrPriorityLvl)
{
    int32_t  status = SystemP_FAILURE;
    uint32_t regVal;

    if ( (baseAddr != ((uint32_t) (0u)))  &&
         (intrSrc  < ESM_MAX_NUM_INTRS) )
    {
        regVal  = HW_RD_REG32(baseAddr +
                CSL_ESM_ERR_GRP_INT_PRIO(intrSrc / ESM_NUM_INTR_PER_GRP));
        if (intrPriorityLvl == (uint32_t)ESM_INTR_PRIORITY_LEVEL_LOW)
        {
            regVal &= ~((uint32_t) 0x1U << (intrSrc % ESM_NUM_INTR_PER_GRP));
        }
        else
        {
            regVal |= ((uint32_t) 0x1U << (intrSrc % ESM_NUM_INTR_PER_GRP));
        }
        HW_WR_REG32(baseAddr +
                CSL_ESM_ERR_GRP_INT_PRIO(intrSrc / ESM_NUM_INTR_PER_GRP),
                regVal);
        status = SystemP_SUCCESS;
    }
    return (status);
}

uint32_t ESM_getIntrStatus(uint32_t baseAddr, uint32_t intrSrc)
{
    uint32_t regVal;

    regVal  = HW_RD_REG32(baseAddr +
                CSL_ESM_ERR_GRP_STS(intrSrc / ESM_NUM_INTR_PER_GRP));
    regVal &= ((uint32_t) 0x1U << (intrSrc % ESM_NUM_INTR_PER_GRP));

    regVal  = (regVal >> (intrSrc % ESM_NUM_INTR_PER_GRP));
    return regVal;
}

int32_t ESM_getGroupIntrStatus(uint32_t baseAddr, uint32_t intrPrioType,
                           ESM_GroupIntrStatus *pIntrstatus)
{
    int32_t    status = SystemP_FAILURE;

    if ((baseAddr != ((uint32_t) (0u))) &&
        (pIntrstatus != ((void *) 0)))
    {
        if (intrPrioType == ESM_INTR_PRIORITY_LEVEL_LOW)
        {
            pIntrstatus->highestPendPlsIntNum = HW_RD_FIELD32(
                        baseAddr + CSL_ESM_LOW_PRI, CSL_ESM_LOW_PRI_PLS);
            pIntrstatus->highestPendLvlIntNum = HW_RD_FIELD32(
                        baseAddr + CSL_ESM_LOW_PRI, CSL_ESM_LOW_PRI_LVL);
            pIntrstatus->grpIntrStatus = HW_RD_REG32(baseAddr + CSL_ESM_LOW);
        }
        else
        {
            pIntrstatus->highestPendPlsIntNum = HW_RD_FIELD32(
                        baseAddr + CSL_ESM_HI_PRI, CSL_ESM_HI_PRI_PLS);
            pIntrstatus->highestPendLvlIntNum = HW_RD_FIELD32(
                        baseAddr + CSL_ESM_HI_PRI, CSL_ESM_HI_PRI_LVL);
            pIntrstatus->grpIntrStatus = HW_RD_REG32(baseAddr + CSL_ESM_HI);
        }
        status = SystemP_SUCCESS;
    }
    return (status);
}

static int32_t ESM_enableGlobalIntr(uint32_t baseAddr)
{
    int32_t  status = SystemP_FAILURE;
    if (baseAddr != ((uint32_t) (0u)))
    {
        HW_WR_FIELD32(baseAddr + CSL_ESM_EN,
                      CSL_ESM_EN_KEY,
                      ESM_EN_KEY_ENBALE_VAL);
        status = SystemP_SUCCESS;
    }
    return status;
}

static void ESM_processInterrupt (void * args, uint32_t priority)
{
    uint32_t                intSrcPulse, intSrcLevel;
    ESM_Config              *ptrESMConfig;
    ESM_Object              *object;
    ESM_GroupIntrStatus     esmGroupStatus;
    uint32_t                index;

    if (args != NULL)
    {
        ptrESMConfig = (ESM_Config *)args;
        object = (ESM_Object*)ptrESMConfig->object;

        do
        {
            ESM_getGroupIntrStatus (object->esmBaseAddr, priority, &esmGroupStatus);

            intSrcPulse = esmGroupStatus.highestPendPlsIntNum;

            /* Check if notify function was registered? */
            for (index = 0; index < ESM_MAX_NOTIFIERS; index++)
            {
                if ((object->notifyParams[index].errorNumber == intSrcPulse) &&
                    (object->notifyParams[index].notify != NULL))
                {
                    object->notifyParams[index].notify(object->notifyParams[index].arg);
                    break;
                }
            }

            if (intSrcPulse != (NO_EVENT_VALUE))
            {
                /* Clear this error */
                (void)ESM_clearIntrStatus (object->esmBaseAddr, intSrcPulse);
            }

            intSrcLevel = esmGroupStatus.highestPendLvlIntNum;

            /* Check if notify function was registered? */
            for (index = 0; index < ESM_MAX_NOTIFIERS; index++)
            {
                if ((object->notifyParams[index].errorNumber == intSrcLevel) &&
                    (object->notifyParams[index].notify != NULL))
                {
                    object->notifyParams[index].notify(object->notifyParams[index].arg);
                    break;
                }
            }

            if (intSrcLevel != (NO_EVENT_VALUE))
            {
                /* Clear this error */
                (void)ESM_clearIntrStatus (object->esmBaseAddr, intSrcLevel);
            }


        } while ((intSrcPulse != (uint32_t)NO_EVENT_VALUE) || (intSrcPulse != (uint32_t)NO_EVENT_VALUE));
    }
}

static void ESM_highPriorityInterrupt (void *args)
{
    if (args != NULL)
    {
        ESM_processInterrupt (args, ESM_INTR_PRIORITY_LEVEL_HIGH);
    }
}

static void ESM_lowPriorityInterrupt (void *args)
{
    if (args != NULL)
    {
        ESM_processInterrupt (args, ESM_INTR_PRIORITY_LEVEL_LOW);
    }
}

static int32_t ESM_clearGroupIntrStatus (uint32_t baseAddr, uint32_t grpNum)
{
    int32_t status = SystemP_FAILURE;
    uint32_t regVal = 0xFFFFFFFFU;

    if ((baseAddr != (uint32_t)NULL) && (grpNum < ESM_INTR_GRP_NUM))
    {
        HW_WR_REG32 (baseAddr + CSL_ESM_ERR_GRP_STS(grpNum), regVal);
        status = SystemP_SUCCESS;
    }

    return status;
}

static int32_t ESM_enableIntr(uint32_t baseAddr, uint32_t intrSrc)
{
    int32_t  status;
    uint32_t regVal;

    if ( (baseAddr == ((uint32_t) (0u)))   ||
         (intrSrc  >= ESM_MAX_NUM_INTRS) )
    {
        status = SystemP_FAILURE;
    }
    else
    {
        status = SystemP_SUCCESS;
    }

    if (status == SystemP_SUCCESS)
    {
        regVal  = HW_RD_REG32(baseAddr +
                CSL_ESM_ERR_GRP_INTR_EN_SET(intrSrc / ESM_NUM_INTR_PER_GRP));
        regVal &= ~((uint32_t) 0x1U << (intrSrc % ESM_NUM_INTR_PER_GRP));
        regVal |= ((uint32_t) 0x1U << (intrSrc % ESM_NUM_INTR_PER_GRP));
        HW_WR_REG32(baseAddr +
                CSL_ESM_ERR_GRP_INTR_EN_SET(intrSrc / ESM_NUM_INTR_PER_GRP),
                regVal);
    }
    return (status);
}

static int32_t ESM_disableIntr (uint32_t baseAddr, uint32_t intrSrc)
{
    int32_t  status;
    uint32_t regVal;

    if ( (baseAddr == ((uint32_t) (0u)))   ||
         (intrSrc  >= ESM_MAX_NUM_INTRS) )
    {
        status = SystemP_FAILURE;
    }
    else
    {
        status = SystemP_SUCCESS;
    }

    if (status == SystemP_SUCCESS)
    {
        regVal  = HW_RD_REG32(baseAddr +
                CSL_ESM_ERR_GRP_INTR_EN_CLR(intrSrc / ESM_NUM_INTR_PER_GRP));
        regVal &= ~((uint32_t) 0x1U << (intrSrc % ESM_NUM_INTR_PER_GRP));
        regVal |= ((uint32_t) 0x1U << (intrSrc % ESM_NUM_INTR_PER_GRP));
        HW_WR_REG32(baseAddr +
            CSL_ESM_ERR_GRP_INTR_EN_CLR(intrSrc / ESM_NUM_INTR_PER_GRP),
            regVal);
    }

    return status;
}

static int32_t ESM_clearIntrStatus (uint32_t baseAddr, uint32_t intrSrc)
{
    int32_t  status = SystemP_FAILURE;
    uint32_t regVal = 0U;
    if ( (baseAddr != ((uint32_t) (0u))) &&
         (intrSrc  < ESM_MAX_NUM_INTRS))
    {
        regVal = ((uint32_t) 0x1U << (intrSrc % ESM_NUM_INTR_PER_GRP));
        HW_WR_REG32(baseAddr +
                CSL_ESM_ERR_GRP_STS(intrSrc / ESM_NUM_INTR_PER_GRP), regVal);
        status = SystemP_SUCCESS;
    }
    return status;
}

static int32_t ESM_setInfluenceOnErrPin (uint32_t baseAddr, uint32_t intrSrc, Bool enable)
{
    int32_t  status ;
    uint32_t regVal = 0;

    if ( (baseAddr == ((uint32_t) (0u))) ||
         (intrSrc  >= ESM_MAX_NUM_INTRS) )
    {
        status = SystemP_FAILURE;
    }
    else
    {
        status = SystemP_SUCCESS;
    }

    if (status == SystemP_SUCCESS)
    {
        if (1U == enable)
        {
            regVal |= ((uint32_t) 0x1U << (intrSrc % ESM_NUM_INTR_PER_GRP));
            HW_WR_REG32(baseAddr +
                CSL_ESM_ERR_GRP_PIN_EN_SET(intrSrc / ESM_NUM_INTR_PER_GRP),
                regVal);
        }
        else
        {
            regVal |= ((uint32_t) 0x1U << (intrSrc % ESM_NUM_INTR_PER_GRP));
            HW_WR_REG32(baseAddr +
                CSL_ESM_ERR_GRP_PIN_EN_CLR(intrSrc / ESM_NUM_INTR_PER_GRP),
                regVal);
        }
    }
    return (status);
}
