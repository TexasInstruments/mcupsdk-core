/*
 *  Copyright (C) 2024 Texas Instruments Incorporated
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
 *  \file   gp_timer.c
 *
 *  \brief  File containing GPTIMER Driver APIs implementation for V0.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <drivers/gp_timer/v0/gp_timer.h>
#include <kernel/dpl/HwiP.h>
#include <drivers/soc.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* Register Offsets */
#define TIMER_TIDR                          (uint32_t)0x00U
#define TIMER_TIOCP_CFG                     (uint32_t)0x10U
#define TIMER_IRQ_EOI                       (uint32_t)0x20U
#define TIMER_IRQSTATUS_RAW                 (uint32_t)0x24U
#define TIMER_IRQSTATUS                     (uint32_t)0x28U
#define TIMER_IRQSTATUS_SET                 (uint32_t)0x2CU
#define TIMER_IRQSTATUS_CLR                 (uint32_t)0x30U
#define TIMER_IRQWAKEEN                     (uint32_t)0x34U
#define TIMER_TCLR                          (uint32_t)0x38U
#define TIMER_TCRR                          (uint32_t)0x3CU
#define TIMER_TLDR                          (uint32_t)0x40U
#define TIMER_TTGR                          (uint32_t)0x44U
#define TIMER_TWPS                          (uint32_t)0x48U
#define TIMER_TMAR                          (uint32_t)0x4CU
#define TIMER_TCAR1                         (uint32_t)0x50U
#define TIMER_TSICR                         (uint32_t)0x54U
#define TIMER_TCAR2                         (uint32_t)0x58U
#define TIMER_TPIR                          (uint32_t)0x5CU
#define TIMER_TNIR                          (uint32_t)0x60U
#define TIMER_TCVR                          (uint32_t)0x64U
#define TIMER_TOCR                          (uint32_t)0x68U
#define TIMER_TOWR                          (uint32_t)0x6CU

/* Register Descriptions */

/* IRQ MASKS */
#define TIMER_IRQ_TCAR_IT_FLAG_MASK         (uint32_t)0x04U
#define TIMER_IRQ_OVF_IT_FLAG_MASK          (uint32_t)0x02U
#define TIMER_IRQ_MAT_IT_FLAG_MASK          (uint32_t)0x01U

/* TIMER_TIOCP_CFG */
#define TIMER_TIOCP_IDLEMODE_MASK           (uint32_t)0x0CU
#define TIMER_TIOCP_IDLEMODE_SHIFT          (uint32_t)0x02U

#define TIMER_TIOCP_EMUFREE_MASK            (uint32_t)0x02U
#define TIMER_TIOCP_EMUFREE_SHIFT           (uint32_t)0x01U

#define TIMER_TIOCP_SOFTRESET_MASK          (uint32_t)0x01U
#define TIMER_TIOCP_SOFTRESET_SHIFT         (uint32_t)0x00U

/* TIMER_TCLR */
#define TIMER_TCLR_ST_MASK                  (uint32_t)0x01U
#define TIMER_TCLR_ST_SHIFT                 (uint32_t)0x00U

#define TIMER_TCLR_AR_MASK                  (uint32_t)0x02U
#define TIMER_TCLR_AR_SHIFT                 (uint32_t)0x01U

#define TIMER_TCLR_PTV_MASK                 (uint32_t)0x1CU
#define TIMER_TCLR_PTV_SHIFT                (uint32_t)0x02U

#define TIMER_TCLR_PRE_MASK                 (uint32_t)0x20U
#define TIMER_TCLR_PRE_SHIFT                (uint32_t)0x05U

#define TIMER_TCLR_CE_MASK                  (uint32_t)0x40U
#define TIMER_TCLR_CE_SHIFT                 (uint32_t)0x06U

#define TIMER_TCLR_SCPWM_MASK               (uint32_t)0x80U
#define TIMER_TCLR_SCPWM_SHIFT              (uint32_t)0x07U

#define TIMER_TCLR_TCM_MASK                 (uint32_t)0x300U
#define TIMER_TCLR_TCM_SHIFT                (uint32_t)0x08U

#define TIMER_TCLR_TRG_MASK                 (uint32_t)0xC00U
#define TIMER_TCLR_TRG_SHIFT                (uint32_t)0x0AU

#define TIMER_TCLR_PT_MASK                  (uint32_t)0x1000U
#define TIMER_TCLR_PT_SHIFT                 (uint32_t)0x0CU

#define TIMER_TCLR_CAPT_MODE_MASK           (uint32_t)0x2000U
#define TIMER_TCLR_CAPT_MODE_SHIFT          (uint32_t)0x0DU

#define TIMER_TCLR_GPO_CFG_MASK             (uint32_t)0x4000U
#define TIMER_TCLR_GPO_CFG_SHIFT            (uint32_t)0x0EU

/* TIMER_TSICR */
#define TIMER_TSICR_SFT_MASK                (uint32_t)0x02U
#define TIMER_TSICR_SFT_SHIFT               (uint32_t)0x01U

#define TIMER_TSICR_POSTED_MASK             (uint32_t)0x04U
#define TIMER_TSICR_POSTED_SHIFT            (uint32_t)0x02U

#define TIMER_TSICR_READ_MODE_MASK          (uint32_t)0x08U
#define TIMER_TSICR_READ_MODE_SHIFT         (uint32_t)0x03U

#define TIMER_TSICR_READ_AFTER_IDLE_MASK    (uint32_t)0x10U
#define TIMER_TSICR_READ_AFTER_IDLE_SHIFT   (uint32_t)0x04U



/* Reg Configurations */

#define TIMER_IDLE_MODE_FORCE_IDLE          (uint32_t)0x00U
#define TIMER_IDLE_MODE_NO_IDLE             (uint32_t)0x01U
#define TIMER_IDLE_MODE_SMART_IDLE          (uint32_t)0x02U
#define TIMER_IDLE_MODE_SMART_IDLE_WAKE     (uint32_t)0x03U


/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

typedef struct
{
    void                   *lock;
    /**< Driver lock - to protect across open/close */
    SemaphoreP_Object       lockObj;
    /**< Driver lock object */
} GPTIMER_DrvObj;

/* Default GPTIMER parameters structure */
const GPTIMER_Params GPTIMER_defaultParams = {

    false,                  /* Enable Prescaler */
    0U,                     /* Counter Prescaler */
    false,                  /* One Shot Mode */
    0U,                     /* Counter Reload Val */
    0U,                     /* Overflow Mask Count */
    0U,                     /* Counter Initial Value */
};
/* ========================================================================== */
/*                      Internal Function Declarations                        */
/* ========================================================================== */

/* Internal Function for Register Access */
static void GPTIMER_setIdleMode(uint32_t baseAddr, uint32_t idleMode);
static void GPTIMER_softReset(uint32_t baseAddr);

static uint32_t GPTIMER_getIRQStat(uint32_t baseAddr);
static void GPTIMER_setIRQStatusEnable(uint32_t baseAddr, uint32_t mask);
static void GPTIMER_clearIRQStatusEnable(uint32_t baseAddr, uint32_t mask);
static void GPTIMER_clrIRQStatus(uint32_t baseAddr, uint32_t mask);

static void GPTIMER_setGPOConfig(uint32_t baseAddr, uint32_t pinFunction);
static void GPTIMER_setCAPTMode(uint32_t baseAddr, uint32_t capMode);
static void GPTIMER_setModulationMode(uint32_t baseAddr,
                                      uint32_t pwmModulationMode);
static void GPTIMER_setPWMTrigOutputMode(uint32_t baseAddr,
                                         uint32_t pwmTrigOutMode);
static void GPTIMER_setTranCaptureMode(uint32_t baseAddr,
                                       uint32_t tranCaptureMode);
static void GPTIMER_pwmOutDefaultSetting(uint32_t baseAddr,
                                         uint32_t pwmOutDefaultSetting);
static void GPTIMER_setCompareEnableState(uint32_t baseAddr, bool state);
static void GPTIMER_setPrescalerEnableState(uint32_t baseAddr, bool state);
static void GPTIMER_setPrescalerClockTimerVal(uint32_t baseAddr, uint32_t ptv);
static void GPTIMER_setAutoReloadEnableState(uint32_t baseAddr, bool state);
static void GPTIMER_setTimerState(uint32_t baseAddr, bool state);

static uint32_t GPTIMER_getCounterVal(uint32_t baseAddr);
static void GPTIMER_setCounterVal(uint32_t baseAddr, uint32_t counterVal);
static void GPTIMER_setTimerLoadVal(uint32_t baseAddr, uint32_t reloadVal);
static void GPTIMER_setTimerCompareVal(uint32_t baseAddr, uint32_t compareVal);

static uint32_t GPTIMER_getTCAR1Val(uint32_t baseAddr);
static uint32_t GPTIMER_getTCAR2Val(uint32_t baseAddr);

static void GPTIMER_setPostedMode(uint32_t baseAddr, bool state);
static void GPTIMER_setOverFlowMaskCount(uint32_t baseAddr, uint32_t count);

/* HWI Function */
static void GPTIMER_hwiFxn(void *arg);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* Externs */
extern GPTIMER_Config gGpTimerConfig[];
extern uint32_t gGpTimerConfigNum;

/** \brief Driver object */
static GPTIMER_DrvObj gGptimerDrvObj =
{
    .lock = NULL,
};

/* ========================================================================== */
/*                       API Function Definitions                             */
/* ========================================================================== */

void GPTIMER_init(void)
{
    GPTIMER_Handle handle;
    uint32_t i;

    /* Call init function for each config*/
    for(i = 0; i < gGpTimerConfigNum; i++)
    {
        handle = &gGpTimerConfig[i];
        /* Input parameter validation */
        if (handle->object != NULL)
        {
            /* Mark the object as available */
            handle->object->isOpen = (bool)false;
        }
    }
    /* Create driver lock */
    (void)SemaphoreP_constructMutex(&gGptimerDrvObj.lockObj);
    gGptimerDrvObj.lock = &gGptimerDrvObj.lockObj;
}

void GPTIMER_deinit(void)
{
    /* Delete driver lock */
    if(NULL != gGptimerDrvObj.lock)
    {
        SemaphoreP_destruct(&gGptimerDrvObj.lockObj);
        gGptimerDrvObj.lock = NULL;
    }
}

void GPTIMER_Params_init(GPTIMER_Params *params)
{
    /* Input parameter validation */
    if (params != NULL)
    {
        *params = GPTIMER_defaultParams;
    }
}

GPTIMER_Handle GPTIMER_open(uint32_t idx, const GPTIMER_Params *params)
{
    GPTIMER_Handle          handle = NULL;
    GPTIMER_Object          *object = NULL;
    GPTIMER_HwAttrs const   *hwAttrs = NULL;
    int32_t                 status = SystemP_SUCCESS;
    void                    *config = NULL;

    /* Check index */
    if(idx >= gGpTimerConfigNum)
    {
        status = SystemP_FAILURE;
    }
    else
    {
        handle = (GPTIMER_Handle)&(gGpTimerConfig[idx]);
    }

    DebugP_assert(NULL != gGptimerDrvObj.lock);
    (void)SemaphoreP_pend(&gGptimerDrvObj.lockObj, SystemP_WAIT_FOREVER);

    if(status == SystemP_SUCCESS)
    {
        object = (GPTIMER_Object*)handle->object;
        DebugP_assert(NULL != object);
        DebugP_assert(NULL != handle->hwAttrs);
        hwAttrs = (GPTIMER_HwAttrs const *)handle->hwAttrs;
        if(true == object->isOpen)
        {
            /* Handle already opended */
            status = SystemP_FAILURE;
            handle = NULL;
        }
    }

    if (status == SystemP_SUCCESS)
    {
        /* Mark the handle as being used */
        object->isOpen = (bool)true;
        /* Store the GPTIMER parameters */
        if (params == NULL)
        {
            /* No params passed in, so use the defaults */
            GPTIMER_Params_init(&(object->gptimerParams));
        }
        else
        {
            /* Copy the params contents */
            object->gptimerParams = *params;
        }
    }

    if (status == SystemP_SUCCESS)
    {
        /* Register interrupt if interrupt is enabled */
        if (hwAttrs->enableIntr == true)
        {
            HwiP_Params hwiPrms;
            /* Initialize with defaults */
            HwiP_Params_init(&hwiPrms);
            /* Populate the interrupt parameters */
            hwiPrms.args = (void *)handle;
            hwiPrms.callback = &GPTIMER_hwiFxn;
            hwiPrms.eventId = hwAttrs->eventId;
            hwiPrms.intNum = hwAttrs->intNum;
            hwiPrms.isPulse = 0U;
            hwiPrms.priority = hwAttrs->intrPriority;
            hwiPrms.isFIQ = 0U;
            /* Register interrupts */
            status = HwiP_construct(&object->hwiObj,&hwiPrms);
            DebugP_assert(status==SystemP_SUCCESS);
        }

        status = SemaphoreP_constructMutex(&object->mutex);
        DebugP_assert(status==SystemP_SUCCESS);

        switch(object->timerConfigMode)
        {
            case GPTIMER_MODE_CONFIG_FREE_RUN:
            {
                config = NULL;
            }
            break;

            case GPTIMER_MODE_CONFIG_INPUT_CAPTURE:
            {
                config = (void *)(&object->captureConfig);
            }
            break;

            case GPTIMER_MODE_CONFIG_OUTPUT_COMPARE:
            {
                config = (void *)(&object->compareConfig);
            }
            break;

            case GPTIMER_MODE_CONFIG_PWM_GEN:
            {
                config = (void *)(&object->pwmConfig);
            }
            break;

            default:
            break;
        }


        status = GPTIMER_setTimerConfigMode(handle, object->timerConfigMode, config);

        /* Load timer Counter value TCRR */
        GPTIMER_setCounterVal(hwAttrs->baseAddr,
                              object->gptimerParams.counterInitVal);
    }

    /* Close the driver if initialization does not go through */
    if(status != SystemP_SUCCESS)
    {
        GPTIMER_close(handle);
    }

    SemaphoreP_post(&gGptimerDrvObj.lockObj);
    return (handle);
}

void GPTIMER_close(GPTIMER_Handle handle)
{
    GPTIMER_Object          *object = NULL;
    GPTIMER_HwAttrs const   *hwAttrs = NULL;

    if(handle != NULL)
    {
        object = (GPTIMER_Object *)handle->object;
        DebugP_assert(NULL != object);
        hwAttrs = (GPTIMER_HwAttrs const *)handle->hwAttrs;
        DebugP_assert(NULL != hwAttrs);

        DebugP_assert(NULL != gGptimerDrvObj.lock);
        (void)SemaphoreP_pend(&gGptimerDrvObj.lockObj, SystemP_WAIT_FOREVER);

        if (true == hwAttrs->enableIntr)
        {
            /* Destruct the Hwi */
            (void)HwiP_destruct(&object->hwiObj);
        }

        /* Clear all interrupt enable */
        GPTIMER_clearIRQStatusEnable(hwAttrs->baseAddr,
                                     TIMER_IRQ_TCAR_IT_FLAG_MASK |
                                     TIMER_IRQ_OVF_IT_FLAG_MASK  |
                                     TIMER_IRQ_MAT_IT_FLAG_MASK);

        /* Clear all pending interrupts */
        GPTIMER_clrIRQStatus(hwAttrs->baseAddr,
                               TIMER_IRQ_TCAR_IT_FLAG_MASK  |
                               TIMER_IRQ_OVF_IT_FLAG_MASK   |
                               TIMER_IRQ_MAT_IT_FLAG_MASK);

        /* Reset Timer Module */
        GPTIMER_softReset(hwAttrs->baseAddr);

        /* Destruct the instance lock */
        (void)SemaphoreP_destruct(&object->mutex);

        object->isOpen = (bool)false;
        SemaphoreP_post(&gGptimerDrvObj.lockObj);
    }
}

void GPTIMER_start(GPTIMER_Handle handle)
{
    GPTIMER_HwAttrs const *hwAttrs = NULL;

    if(handle != NULL)
    {
        hwAttrs = (GPTIMER_HwAttrs const *)handle->hwAttrs;
        GPTIMER_setTimerState(hwAttrs->baseAddr, true);
    }
}

void GPTIMER_stop(GPTIMER_Handle handle)
{
    GPTIMER_HwAttrs const *hwAttrs = NULL;

    if(handle != NULL)
    {
        hwAttrs = (GPTIMER_HwAttrs const *)handle->hwAttrs;
        GPTIMER_setTimerState(hwAttrs->baseAddr, false);
    }
}

uint32_t GPTIMER_getCount(GPTIMER_Handle handle)
{
    GPTIMER_HwAttrs const *hwAttrs = NULL;
    uint32_t counterVal = 0U;

    if(handle != NULL)
    {
        hwAttrs = (GPTIMER_HwAttrs const *)handle->hwAttrs;
        counterVal = GPTIMER_getCounterVal(hwAttrs->baseAddr);
    }

    return counterVal;
}

void GPTIMER_setCount(GPTIMER_Handle handle, uint32_t value)
{
    GPTIMER_HwAttrs const *hwAttrs = NULL;

    if(handle != NULL)
    {
        hwAttrs = (GPTIMER_HwAttrs const *)handle->hwAttrs;
        GPTIMER_setCounterVal(hwAttrs->baseAddr, value);
    }
}

void GPTIMER_setCompareVal(GPTIMER_Handle handle, uint32_t value)
{
    GPTIMER_HwAttrs const *hwAttrs = NULL;

    if(handle != NULL)
    {
        hwAttrs = (GPTIMER_HwAttrs const *)handle->hwAttrs;
        GPTIMER_setTimerCompareVal(hwAttrs->baseAddr, value);
    }
}

uint32_t GPTIMER_getTimerCaptureVal1(GPTIMER_Handle handle)
{
    GPTIMER_HwAttrs const *hwAttrs = NULL;
    uint32_t captureVal = 0U;

    if(handle != NULL)
    {
        hwAttrs = (GPTIMER_HwAttrs const *)handle->hwAttrs;
        captureVal = GPTIMER_getTCAR1Val(hwAttrs->baseAddr);
    }

    return captureVal;
}

uint32_t GPTIMER_getTimerCaptureVal2(GPTIMER_Handle handle)
{
    GPTIMER_HwAttrs const *hwAttrs = NULL;
    uint32_t captureVal = 0U;

    if(handle != NULL)
    {
        hwAttrs = (GPTIMER_HwAttrs const *)handle->hwAttrs;
        captureVal = GPTIMER_getTCAR2Val(hwAttrs->baseAddr);
    }

    return captureVal;
}

int32_t GPTIMER_setTimerConfigMode( GPTIMER_Handle handle,
                                    uint32_t timerConfigMode, void *config)
{
    GPTIMER_Object *object = NULL;
    GPTIMER_HwAttrs const *hwAttrs = NULL;
    int32_t status = SystemP_SUCCESS;

    if(handle != NULL)
    {
        /* Get the Pointers to the object and attributes */
        object = (GPTIMER_Object*)handle->object;
        hwAttrs = (GPTIMER_HwAttrs const *)handle->hwAttrs;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    if((status == SystemP_SUCCESS) && (object->isOpen == true))
    {
        (void)SemaphoreP_pend(&object->mutex, SystemP_WAIT_FOREVER);

        if (hwAttrs->enableIntr == true)
        {
            (void)HwiP_disableInt(hwAttrs->intNum);
        }

        /* Store timer Config Mode */
        object->timerConfigMode = timerConfigMode;

        /* Timer Module Initialization ---------------------------------------*/

        /* Execute Software Reset */
        GPTIMER_softReset(hwAttrs->baseAddr);
        /* Configure IDLE Mode */
        GPTIMER_setIdleMode(hwAttrs->baseAddr, TIMER_IDLE_MODE_FORCE_IDLE);
        /* Select Posted Mode */
        GPTIMER_setPostedMode(hwAttrs->baseAddr, false);

        /* Timer Mode Configuration ------------------------------------------*/
        switch(object->timerConfigMode)
        {
            case GPTIMER_MODE_CONFIG_FREE_RUN:
            {
                /* Set Auto reload Mode */
                GPTIMER_setAutoReloadEnableState(hwAttrs->baseAddr,
                                        !(object->gptimerParams.oneShotMode));
                /* Set Prescalar Timer Value and enable PSC or dont */
                if(object->gptimerParams.enablePrescaler)
                {
                    GPTIMER_setPrescalerClockTimerVal(hwAttrs->baseAddr,
                                        object->gptimerParams.cntPrescaler);
                    GPTIMER_setPrescalerEnableState(hwAttrs->baseAddr, true);
                }
                else
                {
                    GPTIMER_setPrescalerEnableState(hwAttrs->baseAddr, false);
                }
                /* Load timer Counter value TCRR */
                GPTIMER_setCounterVal(hwAttrs->baseAddr, 0U);

                /* If not One Shot Mode */
                /* Load timer load value TLDR */
                if(!(object->gptimerParams.oneShotMode))
                {
                    GPTIMER_setTimerLoadVal(hwAttrs->baseAddr,
                                        object->gptimerParams.cntReloadVal);
                }

                /* Set Overflow Mask Count */
                GPTIMER_setOverFlowMaskCount(hwAttrs->baseAddr,
                                    object->gptimerParams.overflowMaskCount);
                /* Enable overflow interrupt */
                GPTIMER_setIRQStatusEnable(hwAttrs->baseAddr,
                                           TIMER_IRQ_OVF_IT_FLAG_MASK);
                GPTIMER_clearIRQStatusEnable(hwAttrs->baseAddr,
                                             TIMER_IRQ_TCAR_IT_FLAG_MASK);
                GPTIMER_clearIRQStatusEnable(hwAttrs->baseAddr,
                                             TIMER_IRQ_MAT_IT_FLAG_MASK);
            }
            break;

            case GPTIMER_MODE_CONFIG_INPUT_CAPTURE:
            {
                /* Store the new params in driver object */
                object->captureConfig = (*((GPTIMER_Capture_Config *)config));
                /* Set Auto reload Mode */
                GPTIMER_setAutoReloadEnableState(hwAttrs->baseAddr,
                                        !(object->gptimerParams.oneShotMode));
                /* Set Prescalar Timer Value and enable PSC or dont */
                if(object->gptimerParams.enablePrescaler)
                {
                    GPTIMER_setPrescalerClockTimerVal(hwAttrs->baseAddr,
                                        object->gptimerParams.cntPrescaler);
                    GPTIMER_setPrescalerEnableState(hwAttrs->baseAddr, true);
                }
                else
                {
                    GPTIMER_setPrescalerEnableState(hwAttrs->baseAddr, false);
                }

                if(!(object->gptimerParams.oneShotMode))
                {
                    GPTIMER_setTimerLoadVal(hwAttrs->baseAddr,
                                        object->gptimerParams.cntReloadVal);
                }
                /* Set GPO_CFG bit for input */
                GPTIMER_setGPOConfig(hwAttrs->baseAddr, 1U);
                /* Set Capture Mode CAPT_MODE */
                GPTIMER_setCAPTMode(hwAttrs->baseAddr,
                                    object->captureConfig.captureMode);
                /* Set Transition Capture Mode TCM */
                GPTIMER_setTranCaptureMode(hwAttrs->baseAddr,
                                    object->captureConfig.captureEventMode);
                /* Enable Capture Interrupt */
                GPTIMER_clearIRQStatusEnable(hwAttrs->baseAddr,
                                             TIMER_IRQ_OVF_IT_FLAG_MASK);
                GPTIMER_setIRQStatusEnable(hwAttrs->baseAddr,
                                           TIMER_IRQ_TCAR_IT_FLAG_MASK);
                GPTIMER_clearIRQStatusEnable(hwAttrs->baseAddr,
                                             TIMER_IRQ_MAT_IT_FLAG_MASK);
            }
            break;

            case GPTIMER_MODE_CONFIG_OUTPUT_COMPARE:
            {
                /* Store the new params in driver object */
                object->compareConfig = (*((GPTIMER_Compare_Config *)config));
                /* Set Auto reload Mode */
                GPTIMER_setAutoReloadEnableState(hwAttrs->baseAddr,
                                        !(object->gptimerParams.oneShotMode));
                /* Set Prescalar Timer Value and enable PSC or dont */
                if(object->gptimerParams.enablePrescaler)
                {
                    GPTIMER_setPrescalerClockTimerVal(hwAttrs->baseAddr,
                                        object->gptimerParams.cntPrescaler);
                    GPTIMER_setPrescalerEnableState(hwAttrs->baseAddr, true);
                }
                else
                {
                    GPTIMER_setPrescalerEnableState(hwAttrs->baseAddr, false);
                }
                /* Load timer Counter Value TCRR */
                GPTIMER_setCounterVal(hwAttrs->baseAddr, 0U);
                /* Load timer Compare Value TMAR */
                GPTIMER_setTimerCompareVal(hwAttrs->baseAddr,
                                    object->compareConfig.cntCompareValComp);
                /* Enable Compare Mode */
                GPTIMER_setCompareEnableState(hwAttrs->baseAddr, true);
                /* Enable Match interrupt */
                GPTIMER_clearIRQStatusEnable(hwAttrs->baseAddr,
                                             TIMER_IRQ_OVF_IT_FLAG_MASK);
                GPTIMER_clearIRQStatusEnable(hwAttrs->baseAddr,
                                             TIMER_IRQ_TCAR_IT_FLAG_MASK);
                GPTIMER_setIRQStatusEnable(hwAttrs->baseAddr,
                                           TIMER_IRQ_MAT_IT_FLAG_MASK);
            }
            break;

            case GPTIMER_MODE_CONFIG_PWM_GEN:
            {
                /* Store the new params in driver object */
                object->pwmConfig = (*((GPTIMER_PWM_Config *)config));
                /* Set Auto reload Mode */
                GPTIMER_setAutoReloadEnableState(hwAttrs->baseAddr,
                                        !(object->gptimerParams.oneShotMode));
                /* Set Prescalar Timer Value and enable PSC or dont */
                if(object->gptimerParams.enablePrescaler)
                {
                    GPTIMER_setPrescalerClockTimerVal(hwAttrs->baseAddr,
                                        object->gptimerParams.cntPrescaler);
                    GPTIMER_setPrescalerEnableState(hwAttrs->baseAddr, true);
                }
                else
                {
                    GPTIMER_setPrescalerEnableState(hwAttrs->baseAddr, false);
                }

                if(!(object->gptimerParams.oneShotMode))
                {
                    GPTIMER_setTimerLoadVal(hwAttrs->baseAddr,
                                        object->gptimerParams.cntReloadVal);
                }
                /* Select Trigger output mode TRG */
                GPTIMER_setPWMTrigOutputMode(hwAttrs->baseAddr,
                                        object->pwmConfig.trigOutputPWMMode);
                /* Enable CE(Compare Enable Bit if trig mode is Compare and Overflow) */
                if(object->pwmConfig.trigOutputPWMMode ==
                                    GPTIMER_PWM_OUT_OVERFLOW_MATCH_TRIGGER)
                {
                    GPTIMER_setCompareEnableState(hwAttrs->baseAddr, true);
                }
                else
                {
                    GPTIMER_setCompareEnableState(hwAttrs->baseAddr, false);
                }
                /* Select PWM mode pulse or toggle PT */
                GPTIMER_setModulationMode(hwAttrs->baseAddr,
                                        object->pwmConfig.outputModulationType);
                /* Set GPO_CFG bit for pwm output */
                GPTIMER_setGPOConfig(hwAttrs->baseAddr, 0U);
                /* Configure PWM output pin default Value */
                GPTIMER_pwmOutDefaultSetting(hwAttrs->baseAddr,
                                        object->pwmConfig.defaultPWMOutSetting);
                /* Load timer Counter Value TCRR */
                GPTIMER_setCounterVal(hwAttrs->baseAddr, 0U);
                /* Load timer Compare Value TMAR */
                GPTIMER_setTimerCompareVal(hwAttrs->baseAddr,
                                        object->pwmConfig.cntCompareValPWM);
                /* Enable Comapare */
                GPTIMER_setCompareEnableState(hwAttrs->baseAddr, true);
            }
            break;

            default:
            break;
        }

        if (hwAttrs->enableIntr == true)
        {
            HwiP_enableInt(hwAttrs->intNum);
        }

        /* Release the lock for this particular GPTIMER handle */
        (void)SemaphoreP_post(&object->mutex);
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

void GPTIMER_setCallbackFxn (GPTIMER_Handle handle,
                            GPTIMER_OverflowCallbackFxn overflowCbFxn,
                            GPTIMER_CompareMatchCallbackFxn compMatchCbFxn,
                            GPTIMER_CaptureCallbackFxn captureCbFxn)
{
    GPTIMER_Object *object = NULL;

    if(handle != NULL)
    {
        /* Get the Pointers to the object and attributes */
        object = (GPTIMER_Object*)handle->object;

        object->overflowCallbackFunction = overflowCbFxn;
        object->compareMatchCallbackFunction = compMatchCbFxn;
        object->captureCallbackFunction = captureCbFxn;
    }
}

uint32_t GPTIMER_getIRQStatus(GPTIMER_Handle handle)
{
    GPTIMER_HwAttrs const *hwAttrs = NULL;
    uint32_t irqStat = 0U;

    if(handle != NULL)
    {
        hwAttrs = (GPTIMER_HwAttrs const *)handle->hwAttrs;
        irqStat = GPTIMER_getIRQStat(hwAttrs->baseAddr);
    }

    return irqStat;
}

void GPTIMER_clearIRQStatus(GPTIMER_Handle handle, uint32_t irqMask)
{
    GPTIMER_HwAttrs const   *hwAttrs = NULL;

    if(handle != NULL)
    {
        hwAttrs = (GPTIMER_HwAttrs const *)handle->hwAttrs;

        GPTIMER_clrIRQStatus(hwAttrs->baseAddr, irqMask);
    }
}

void GPTIMER_enableInterruptStatus(GPTIMER_Handle handle, uint32_t irqMask)
{
    GPTIMER_HwAttrs const   *hwAttrs = NULL;

    if(handle != NULL)
    {
        hwAttrs = (GPTIMER_HwAttrs const *)handle->hwAttrs;

        GPTIMER_setIRQStatusEnable(hwAttrs->baseAddr, irqMask);
    }
}

void GPTIMER_disableInterruptStatus(GPTIMER_Handle handle, uint32_t irqMask)
{
    GPTIMER_HwAttrs const   *hwAttrs = NULL;

    if(handle != NULL)
    {
        hwAttrs = (GPTIMER_HwAttrs const *)handle->hwAttrs;

        GPTIMER_clearIRQStatusEnable(hwAttrs->baseAddr, irqMask);
    }
}

/* ========================================================================== */
/*                     API ISR Function Definitions                           */
/* ========================================================================== */
/* HWI Function */
static void GPTIMER_hwiFxn(void *arg)
{
    GPTIMER_Handle          handle = (GPTIMER_Handle)arg;
    GPTIMER_Object          *object = NULL;
    GPTIMER_HwAttrs const   *hwAttrs = NULL;
    uint32_t                isrRegVal = 0U;

    /* Input parameter validation */
    if (handle != NULL)
    {
        /* Get the pointer to the object */
        object = (GPTIMER_Object *)handle->object;
        hwAttrs = (GPTIMER_HwAttrs const *)handle->hwAttrs;
        isrRegVal = GPTIMER_getIRQStat(hwAttrs->baseAddr);

        if((isrRegVal & TIMER_IRQ_TCAR_IT_FLAG_MASK) != 0U)
        {
            object->captureCallbackFunction(handle);
            GPTIMER_clrIRQStatus(hwAttrs->baseAddr,
                                   TIMER_IRQ_TCAR_IT_FLAG_MASK);
        }

        if((isrRegVal & TIMER_IRQ_OVF_IT_FLAG_MASK) != 0U)
        {
            object->overflowCallbackFunction(handle);
            GPTIMER_clrIRQStatus(hwAttrs->baseAddr,
                                   TIMER_IRQ_OVF_IT_FLAG_MASK);
        }

        if((isrRegVal & TIMER_IRQ_MAT_IT_FLAG_MASK) != 0U)
        {
            object->compareMatchCallbackFunction(handle);
            GPTIMER_clrIRQStatus(hwAttrs->baseAddr,
                                   TIMER_IRQ_MAT_IT_FLAG_MASK);
        }
    }
    return;
}

/* ========================================================================== */
/*                      Internal Function Definitions                         */
/* ========================================================================== */

static void GPTIMER_setIdleMode(uint32_t baseAddr, uint32_t idleMode)
{
    uint32_t regVal = 0U;
    volatile uint32_t *addr = (volatile uint32_t *)(baseAddr + TIMER_TIOCP_CFG);
    regVal = *addr;
    regVal &= ~TIMER_TIOCP_IDLEMODE_MASK;
    regVal |= (idleMode << TIMER_TIOCP_IDLEMODE_SHIFT);
    *addr = regVal;
}

static void GPTIMER_softReset(uint32_t baseAddr)
{
    volatile uint32_t *addr = (volatile uint32_t *)(baseAddr + TIMER_TIOCP_CFG);
    *addr |= TIMER_TIOCP_SOFTRESET_MASK;

    while(((*addr) & TIMER_TIOCP_SOFTRESET_MASK) != 0U)
    {
        /* Wait For Reset To happen */
    }
}

static uint32_t GPTIMER_getIRQStat(uint32_t baseAddr)
{
    volatile uint32_t *addr = (volatile uint32_t *)(baseAddr + TIMER_IRQSTATUS);
    return *addr;
}

static void GPTIMER_setIRQStatusEnable(uint32_t baseAddr, uint32_t mask)
{
    volatile uint32_t *addr =
                    (volatile uint32_t *)(baseAddr + TIMER_IRQSTATUS_SET);
    *addr |= mask;
}

static void GPTIMER_clearIRQStatusEnable(uint32_t baseAddr, uint32_t mask)
{
    uint32_t regVal = 0U;
    volatile uint32_t *addr =
                    (volatile uint32_t *)(baseAddr + TIMER_IRQSTATUS_CLR);
    regVal = *addr;
    regVal = (regVal & mask);
    *addr = regVal;
}

static void GPTIMER_clrIRQStatus(uint32_t baseAddr, uint32_t mask)
{
    volatile uint32_t *addr = (volatile uint32_t *)(baseAddr + TIMER_IRQSTATUS);
    *addr |= mask;
}

static void GPTIMER_setGPOConfig(uint32_t baseAddr, uint32_t pinFunction)
{
    uint32_t regVal = 0U;
    volatile uint32_t *addr =
                    (volatile uint32_t *)(baseAddr + TIMER_TCLR);
    regVal = *addr;
    regVal &= ~TIMER_TCLR_GPO_CFG_MASK;
    regVal |= (pinFunction << TIMER_TCLR_GPO_CFG_SHIFT);
    *addr = regVal;
}

static void GPTIMER_setCAPTMode(uint32_t baseAddr, uint32_t capMode)
{
    uint32_t regVal = 0U;
    volatile uint32_t *addr =
                    (volatile uint32_t *)(baseAddr + TIMER_TCLR);
    regVal = *addr;
    regVal &= ~TIMER_TCLR_CAPT_MODE_MASK;
    regVal |= (capMode << TIMER_TCLR_CAPT_MODE_SHIFT);
    *addr = regVal;
}

static void GPTIMER_setModulationMode(uint32_t baseAddr,
                                      uint32_t pwmModulationMode)
{
    uint32_t regVal = 0U;
    volatile uint32_t *addr =
                    (volatile uint32_t *)(baseAddr + TIMER_TCLR);
    regVal = *addr;
    regVal &= ~TIMER_TCLR_PT_MASK;
    regVal |= (pwmModulationMode << TIMER_TCLR_PT_SHIFT);
    *addr = regVal;
}

static void GPTIMER_setPWMTrigOutputMode(uint32_t baseAddr,
                                         uint32_t pwmTrigOutMode)
{
    uint32_t regVal = 0U;
    volatile uint32_t *addr =
                    (volatile uint32_t *)(baseAddr + TIMER_TCLR);
    regVal = *addr;
    regVal &= ~TIMER_TCLR_TRG_MASK;
    regVal |= (pwmTrigOutMode << TIMER_TCLR_TRG_SHIFT);
    *addr = regVal;
}

static void GPTIMER_setTranCaptureMode(uint32_t baseAddr,
                                       uint32_t tranCaptureMode)
{
    uint32_t regVal = 0U;
    volatile uint32_t *addr =
                    (volatile uint32_t *)(baseAddr + TIMER_TCLR);
    regVal = *addr;
    regVal &= ~TIMER_TCLR_TCM_MASK;
    regVal |= (tranCaptureMode << TIMER_TCLR_TCM_SHIFT);
    *addr = regVal;
}

static void GPTIMER_pwmOutDefaultSetting(uint32_t baseAddr,
                                         uint32_t pwmOutDefaultSetting)
{
    uint32_t regVal = 0U;
    volatile uint32_t *addr =
                    (volatile uint32_t *)(baseAddr + TIMER_TCLR);
    regVal = *addr;
    regVal &= ~TIMER_TCLR_SCPWM_MASK;
    regVal |= (pwmOutDefaultSetting << TIMER_TCLR_SCPWM_SHIFT);
    *addr = regVal;
}

static void GPTIMER_setCompareEnableState(uint32_t baseAddr, bool state)
{
    uint32_t regVal = 0U;
    volatile uint32_t *addr =
                    (volatile uint32_t *)(baseAddr + TIMER_TCLR);
    regVal = *addr;
    if(state)
    {
        regVal |= (TIMER_TCLR_CE_MASK);
    }
    else
    {
        regVal &= ~(TIMER_TCLR_CE_MASK);
    }
    *addr = regVal;
}

static void GPTIMER_setPrescalerEnableState(uint32_t baseAddr, bool state)
{
    uint32_t regVal = 0U;
    volatile uint32_t *addr =
                    (volatile uint32_t *)(baseAddr + TIMER_TCLR);
    regVal = *addr;
    if(state)
    {
        regVal |= (TIMER_TCLR_PRE_MASK);
    }
    else
    {
        regVal &= ~(TIMER_TCLR_PRE_MASK);
    }
    *addr = regVal;
}

static void GPTIMER_setPrescalerClockTimerVal(uint32_t baseAddr, uint32_t ptv)
{
    uint32_t regVal = 0U;
    volatile uint32_t *addr =
                    (volatile uint32_t *)(baseAddr + TIMER_TCLR);
    regVal = *addr;
    regVal &= ~TIMER_TCLR_PTV_MASK;
    regVal |= (ptv << TIMER_TCLR_PTV_SHIFT);
    *addr = regVal;
}

static void GPTIMER_setAutoReloadEnableState(uint32_t baseAddr, bool state)
{
    uint32_t regVal = 0U;
    volatile uint32_t *addr =
                    (volatile uint32_t *)(baseAddr + TIMER_TCLR);
    regVal = *addr;
    if(state)
    {
        regVal |= (TIMER_TCLR_AR_MASK);
    }
    else
    {
        regVal &= ~(TIMER_TCLR_AR_MASK);
    }
    *addr = regVal;
}

static void GPTIMER_setTimerState(uint32_t baseAddr, bool state)
{
    uint32_t regVal = 0U;
    volatile uint32_t *addr =
                    (volatile uint32_t *)(baseAddr + TIMER_TCLR);
    regVal = *addr;
    if(state)
    {
        regVal |= (TIMER_TCLR_ST_MASK);
    }
    else
    {
        regVal &= ~(TIMER_TCLR_ST_MASK);
    }
    *addr = regVal;
}

static uint32_t GPTIMER_getCounterVal(uint32_t baseAddr)
{
    volatile uint32_t *addr = (volatile uint32_t *)(baseAddr + TIMER_TCRR);
    return *addr;
}

static void GPTIMER_setCounterVal(uint32_t baseAddr, uint32_t counterVal)
{
    volatile uint32_t *addr = (volatile uint32_t *)(baseAddr + TIMER_TCRR);
    *addr = counterVal;
}

static void GPTIMER_setTimerLoadVal(uint32_t baseAddr, uint32_t reloadVal)
{
    volatile uint32_t *addr = (volatile uint32_t *)(baseAddr + TIMER_TLDR);
    *addr = reloadVal;
}

static void GPTIMER_setTimerCompareVal(uint32_t baseAddr, uint32_t compareVal)
{
    volatile uint32_t *addr = (volatile uint32_t *)(baseAddr + TIMER_TMAR);
    *addr = compareVal;
}

static uint32_t GPTIMER_getTCAR1Val(uint32_t baseAddr)
{
    volatile uint32_t *addr = (volatile uint32_t *)(baseAddr + TIMER_TCAR1);
    return *addr;
}

static uint32_t GPTIMER_getTCAR2Val(uint32_t baseAddr)
{
    volatile uint32_t *addr = (volatile uint32_t *)(baseAddr + TIMER_TCAR2);
    return *addr;
}

static void GPTIMER_setPostedMode(uint32_t baseAddr, bool state)
{
    uint32_t regVal = 0U;
    volatile uint32_t *addr =
                    (volatile uint32_t *)(baseAddr + TIMER_TSICR);
    regVal = *addr;
    if(state)
    {
        regVal |= (TIMER_TSICR_POSTED_MASK);
    }
    else
    {
        regVal &= ~(TIMER_TSICR_POSTED_MASK);
    }
    *addr = regVal;
}

static void GPTIMER_setOverFlowMaskCount(uint32_t baseAddr, uint32_t count)
{
    volatile uint32_t *addr = (volatile uint32_t *)(baseAddr + TIMER_TOWR);
    *addr = (count & 0x00FFFFFFU);
}
