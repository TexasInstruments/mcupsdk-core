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
 *  \file gpmc_v0.c
 *
 *  \brief File containing GPMC Driver APIs implementation for version V0.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <string.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/CacheP.h>
#include <kernel/dpl/ClockP.h>
#include <drivers/hw_include/cslr.h>
#include <drivers/gpmc.h>
#include <drivers/elm.h>
#include <drivers/gpmc/v0/dma/gpmc_dma.h>
#include "gpmc_priv_v0.h"

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

typedef struct
{
    void *openLock;
    /**<  Lock to protect GPMC open*/
    SemaphoreP_Object lockObj;
    /**< Lock object */
} GPMC_DrvObj;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* Internal functions */
static void GPMC_isr(void *arg);
static void GPMC_transferCallback(GPMC_Handle handle, GPMC_Transaction *msg);
static int32_t GPMC_programInstance(GPMC_Config *config);
static void GPMC_waitPinPolaritySelect(GPMC_Handle handle, uint32_t pin,
                                uint32_t polarity);
static int32_t GPMC_moduleResetStatusWaitTimeout(GPMC_Config *config,
                                uint32_t timeOut);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/** \brief Driver object */
static GPMC_DrvObj gGpmcDrvObj =
{
    .openLock      = NULL,
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void GPMC_init(void)
{
    int32_t status = SystemP_SUCCESS;
    GPMC_Object *obj;

    for(uint8_t count =0; count < gGpmcConfigNum; count++)
    {
        /* Init object variables */
        obj = gGpmcConfig[count].object;
        DebugP_assert(NULL != obj);
        memset(obj, 0, sizeof(GPMC_Object));
    }

    /* Create the driver lock */
    status = SemaphoreP_constructMutex(&gGpmcDrvObj.lockObj);
    if(SystemP_SUCCESS == status)
    {
        gGpmcDrvObj.openLock = &gGpmcDrvObj.lockObj;
    }

    return;
}

void GPMC_deinit(void)
{
    if(gGpmcDrvObj.openLock != NULL)
    {
        /* Delete Semaphore. */
        SemaphoreP_destruct(&gGpmcDrvObj.lockObj);
        gGpmcDrvObj.openLock = NULL;
    }

    return;
}

int32_t GPMC_configureTimingParameters(GPMC_Handle handle)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t timeConfig = 0;

    if(handle != NULL)
    {
        const GPMC_HwAttrs *hwAttrs = ((GPMC_Config*)handle)->attrs;
        GPMC_Object *object  = ((GPMC_Config*)handle)->object;
        uint32_t devType = object->params.devType;

        /* CONFIG2 reister timing config, no extra delay */
        timeConfig = GPMC_CS_TIMING_CONFIG(hwAttrs->timingParams.csWrOffTime,
                                            hwAttrs->timingParams.csRdOffTime,
                                            hwAttrs->csExDelay,
                                            hwAttrs->timingParams.csOnTime);

        CSL_REG32_WR(hwAttrs->gpmcBaseAddr + CSL_GPMC_CONFIG2(object->params.chipSel), timeConfig);


        /* CONFIG3 reister timing config, no extra delay */
        timeConfig = GPMC_ADV_TIMING_CONFIG(hwAttrs->timingParams.advAadMuxWrOffTime,
                                            hwAttrs->timingParams.advAadMuxRdOffTime,
                                            hwAttrs->timingParams.advWrOffTime,
                                            hwAttrs->timingParams.advRdOffTime,
                                            CSL_GPMC_CONFIG3_ADVEXTRADELAY_NOTDELAYED,
                                            hwAttrs->timingParams.advAadMuxOnTime,
                                            hwAttrs->timingParams.advOnTime);
        CSL_REG32_WR(hwAttrs->gpmcBaseAddr + CSL_GPMC_CONFIG3(object->params.chipSel), timeConfig);

       /* CONFIG4 reister timing config, no xtra delay */
        timeConfig = GPMC_WE_OE_TIMING_CONFIG(hwAttrs->timingParams.weOffTime,
                                                CSL_GPMC_CONFIG4_WEEXTRADELAY_NOTDELAYED,
                                                hwAttrs->timingParams.weOnTtime,
                                                hwAttrs->timingParams.oeAadMuxOffTime,
                                                hwAttrs->timingParams.oeOffTime,
                                                CSL_GPMC_CONFIG4_OEEXTRADELAY_NOTDELAYED,
                                                hwAttrs->timingParams.oeAadMuxOnTime,
                                                hwAttrs->timingParams.oeOnTime);

        CSL_REG32_WR(hwAttrs->gpmcBaseAddr + CSL_GPMC_CONFIG4(object->params.chipSel), timeConfig);

        /* CONFIG5 reister timing config */
        timeConfig = GPMC_RDACCESS_CYCLETIME_TIMING_CONFIG(hwAttrs->timingParams.rdCycleTime,
                                                            hwAttrs->timingParams.wrCycleTime,
                                                            hwAttrs->timingParams.rdAccessTime,
                                                            hwAttrs->timingParams.pageBurstAccess);
        CSL_REG32_WR(hwAttrs->gpmcBaseAddr + CSL_GPMC_CONFIG5(object->params.chipSel), timeConfig);

//        CSL_REG32_FINS(0x3B000078, GPMC_CONFIG7_CSVALID, CSL_GPMC_CONFIG7_CSVALID_CSENABLED);

        /* CONFIG6 register timing config */
        CSL_REG32_FINS(hwAttrs->gpmcBaseAddr + CSL_GPMC_CONFIG6(object->params.chipSel),
                    GPMC_CONFIG6_WRACCESSTIME, hwAttrs->timingParams.wrAcessTime);
        CSL_REG32_FINS(hwAttrs->gpmcBaseAddr + CSL_GPMC_CONFIG6(object->params.chipSel),
                    GPMC_CONFIG6_WRDATAONADMUXBUS, hwAttrs->timingParams.wrDataOnMuxBusTime);


        timeConfig = GPMC_CYCLE2CYCLE_BUSTURNAROUND_TIMING_CONFIG(hwAttrs->timingParams.cycle2CycleDelay,
                                                                    hwAttrs->timingParams.cycleDelaySameChipSel,
                                                                    hwAttrs->timingParams.cycleDelayDiffChipSel,
                                                                    hwAttrs->timingParams.busTurnAroundTime);

        CSL_REG32_WR(hwAttrs->gpmcBaseAddr + CSL_GPMC_CONFIG6(object->params.chipSel), \
        (CSL_REG32_RD(hwAttrs->gpmcBaseAddr + CSL_GPMC_CONFIG6(object->params.chipSel)) | timeConfig));

        if(devType == CSL_GPMC_CONFIG1_DEVICETYPE_NORLIKE)
        {
            // CSL_REG32_WR(hwAttrs->gpmcBaseAddr + CSL_GPMC_CONFIG7(object->params.chipSel), 0xf10);
            CSL_REG32_FINS(hwAttrs->gpmcBaseAddr + CSL_GPMC_CONFIG7(object->params.chipSel), \
                            GPMC_CONFIG7_CSVALID, CSL_GPMC_CONFIG7_CSVALID_CSENABLED);
        }
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

int32_t GPMC_setDeviceType(GPMC_Handle handle)
{
    int32_t status = SystemP_SUCCESS;
    /* Input parameter validation. */
    if(handle != NULL)
    {
        GPMC_Config *config = (GPMC_Config*)handle;
        const GPMC_HwAttrs *attrs = config->attrs;
        GPMC_Object *object = config->object;
        /* Set Device type interfaced with GPMC. */
        CSL_REG32_FINS(attrs->gpmcBaseAddr + CSL_GPMC_CONFIG1(object->params.chipSel),
                  GPMC_CONFIG1_DEVICETYPE, object->params.devType);

    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

int32_t GPMC_setDeviceSize(GPMC_Handle handle)
{
    int32_t status = SystemP_SUCCESS;

    /* Input parameter validation. */
    if(handle != NULL)
    {
        GPMC_Config *config = (GPMC_Config*)handle;
        const GPMC_HwAttrs *attrs = config->attrs;
        GPMC_Object *object = config->object;
        /* Set device width interfaced with GPMC. */
        CSL_REG32_FINS(attrs->gpmcBaseAddr + CSL_GPMC_CONFIG1(object->params.chipSel),
                  GPMC_CONFIG1_DEVICESIZE, object->params.devSize);

    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

GPMC_Handle GPMC_open(uint32_t index, const GPMC_Params *prms)
{
    int32_t status = SystemP_SUCCESS;
    GPMC_Handle         handle = NULL;
    GPMC_Config         *config = NULL;
    GPMC_Object         *object = NULL;
    const GPMC_HwAttrs  *hwAttrs = NULL;
    HwiP_Params         hwiParams;


    /* Check for valid index */
    if(index >= gGpmcConfigNum)
    {
        status = SystemP_FAILURE;
    }
    else
    {
        config = &gGpmcConfig[index];
    }

    /* Protect this region from a concurrent GPMC_Open */
    DebugP_assert(NULL != gGpmcDrvObj.openLock);
    SemaphoreP_pend(&gGpmcDrvObj.lockObj, SystemP_WAIT_FOREVER);

    if(status == SystemP_SUCCESS)
    {
        object = config->object;
        DebugP_assert(object != NULL);
        DebugP_assert(config->attrs != NULL);
        hwAttrs = config->attrs;
        if(object->isOpen == TRUE)
        {
            status = SystemP_FAILURE;
        }
    }

    if(status == SystemP_SUCCESS)
    {
        object->handle = (GPMC_Handle)config;

        if(prms != NULL)
        {
            memcpy((void*)&object->params, (void*)prms, sizeof(GPMC_Params));
        }
        else
        {
            /* Init with default if NULL is passed */
            GPMC_Params_init(&object->params);
        }

        /* If DMA is enabled, program UDMA block copy channel */
        if(object->params.dmaEnable == TRUE)
        {
            object->gpmcDmaHandle = GPMC_dmaOpen(object->params.gpmcDmaChIndex);
        }
        else
        {
            object->gpmcDmaHandle = NULL;
        }

        if(object->params.transferMode == GPMC_TRANSFER_MODE_BLOCKING)
        {
            if(object->params.intrEnable == TRUE)
            {
                object->operMode = GPMC_OPERATING_MODE_BLOCKING;
            }
            else
            {
                object->operMode = GPMC_OPERATING_MODE_POLLING;
            }
        }
        else
        {
            object->operMode = GPMC_OPERATING_MODE_CALLBACK;
        }

        if(object->params.intrEnable == TRUE)
        {
            HwiP_Params_init(&hwiParams);
            hwiParams.intNum = hwAttrs->intrNum;
            hwiParams.callback = GPMC_isr;
            hwiParams.args     = (void*)config;
            hwiParams.priority = hwAttrs->intrPriority;
            status += HwiP_construct(&object->hwi, &hwiParams);

        }

    }

    if(status == SystemP_SUCCESS)
    {
        status += GPMC_programInstance(config);

        status += SemaphoreP_constructMutex(&object->mutex);

        if (object->operMode == GPMC_OPERATING_MODE_BLOCKING)
        {
            /*
            * Semaphore to cause the waiting task to block for the GPMC to finish.
            */
            status += SemaphoreP_constructBinary(&object->transferComplete, 0);

            /* Store internal callback function */
            object->params.transferCallBckFunc = &GPMC_transferCallback;
        }

        if(object->operMode == GPMC_OPERATING_MODE_CALLBACK)
        {
            /* Currently not supported. */
        }

    }

    if(status == SystemP_SUCCESS)
    {
        object->isOpen = 1;
        handle = (GPMC_Handle)config;
    }

    SemaphoreP_post(&gGpmcDrvObj.lockObj);

    /* Free up resources in case of error. */
    if(SystemP_SUCCESS != status)
    {
        if(NULL != config)
        {
            GPMC_close((GPMC_Handle) config);
        }
    }

    return(handle);
}

void GPMC_close(GPMC_Handle handle)
{
    /* Input parameter validation */
    if (handle != NULL)
    {
        GPMC_Object        *object = NULL;
        const GPMC_HwAttrs *hwAttrs = NULL;
        /* Get the pointer to the object and hwAttrs */
        object = ((GPMC_Config*)handle)->object;
        hwAttrs = ((GPMC_Config*)handle)->attrs;

        /* Disable all interupts associated to GPMC. */
        GPMC_disableInterupt(hwAttrs->gpmcBaseAddr,GPMC_FIFOEVENT_INT);
        GPMC_disableInterupt(hwAttrs->gpmcBaseAddr,GPMC_TERMINALCOUNT_INT);
        GPMC_disableInterupt(hwAttrs->gpmcBaseAddr,GPMC_WAIT0EDGEDETECTION_INT);
        GPMC_disableInterupt(hwAttrs->gpmcBaseAddr,GPMC_WAIT1EDGEDETECTION_INT);

        /* If DMA block copy channel was opened, close it. */
        if(object->params.dmaEnable == TRUE)
        {
            GPMC_dmaClose(object->gpmcDmaHandle);
        }

        /* Destruct the Hwi. */
        if(object->operMode != GPMC_OPERATING_MODE_POLLING)
        {
            HwiP_destruct(&object->hwi);
        }

        /* Destruct the instance lock. */
        SemaphoreP_destruct(&object->mutex);

        /* Destruct the transfer completion lock. */
        if(object->operMode == GPMC_OPERATING_MODE_BLOCKING)
        {
            SemaphoreP_destruct(&object->transferComplete);
        }

        /* Open flag is set false. */
        object->isOpen = 0;
    }

    return;
}

void GPMC_transactionInit(GPMC_Transaction *trans)
{
    trans->Buf = NULL;
    trans->count = 0;
    trans->status = GPMC_TRANSFER_STARTED;
    trans->transType = GPMC_TRANSACTION_TYPE_READ;
    trans->arg = NULL;
    trans->transferTimeout = SystemP_WAIT_FOREVER;
}

GPMC_Handle GPMC_getHandle(uint32_t driverInstanceIndex)
{
    GPMC_Handle         handle = NULL;
    /* Check index */
    if(driverInstanceIndex < gGpmcConfigNum)
    {
        GPMC_Object *obj;
        obj = gGpmcConfig[driverInstanceIndex].object;

        if(obj && (TRUE == obj->isOpen))
        {
            /* valid handle */
            handle = obj->handle;
        }
    }
    return handle;
}

uint32_t GPMC_getInputClk(GPMC_Handle handle)
{
    uint32_t retVal = 0U;

    /* Input parameter validation. */
    if(handle != NULL)
    {
        /* Get GPMC interface clock. */
        const GPMC_HwAttrs* attrs = ((GPMC_Config *)handle)->attrs;
        retVal = attrs->inputClkFreq;
    }
    return retVal;
}

/* ========================================================================== */
/*                     Internal function definitions                          */
/* ========================================================================== */

static void GPMC_isr(void *arg)
{
    /* Currently interupt mode not supported.*/
}

static int32_t GPMC_moduleResetStatusWaitTimeout(GPMC_Config *config, uint32_t timeOut)
{
    int32_t status  =   SystemP_SUCCESS;
    const GPMC_HwAttrs *hwAttrs = config->attrs;
    uint64_t curTime = 0;

    if(timeOut != 0)
    {
        curTime = ClockP_getTimeUsec();
        /* Check for GPMC module reset status with timeout. */
        while((CSL_REG32_FEXT(hwAttrs->gpmcBaseAddr + CSL_GPMC_SYSSTATUS, GPMC_SYSSTATUS_RESETDONE) != CSL_GPMC_SYSSTATUS_RESETDONE_RSTDONE) &&
            ((ClockP_getTimeUsec() - curTime) < timeOut))
        {

        }

        if(CSL_REG32_FEXT(hwAttrs->gpmcBaseAddr + CSL_GPMC_SYSSTATUS, GPMC_SYSSTATUS_RESETDONE) != CSL_GPMC_SYSSTATUS_RESETDONE_RSTDONE)
        {
            return SystemP_FAILURE;
        }

    }
    else
    {
        if(CSL_REG32_FEXT(hwAttrs->gpmcBaseAddr + CSL_GPMC_SYSSTATUS, GPMC_SYSSTATUS_RESETDONE) != CSL_GPMC_SYSSTATUS_RESETDONE_RSTDONE)
        {
            return SystemP_FAILURE;
        }
    }

    return status;

 }

static void GPMC_waitPinPolaritySelect(GPMC_Handle handle, uint32_t pin,
                                uint32_t polarity)
{
    if(handle != NULL)
    {
        const GPMC_HwAttrs *hwAttrs = ((GPMC_Config*)handle)->attrs;
        /* Select WAIT PIN polarity. */
        if (pin == CSL_GPMC_CONFIG1_WAITPINSELECT_W0)
        {
            CSL_REG32_FINS(hwAttrs->gpmcBaseAddr + CSL_GPMC_CONFIG, GPMC_CONFIG_WAIT0PINPOLARITY,
                        polarity);
        }
        else if (pin == CSL_GPMC_CONFIG1_WAITPINSELECT_W1)
        {
            CSL_REG32_FINS(hwAttrs->gpmcBaseAddr + CSL_GPMC_CONFIG, GPMC_CONFIG_WAIT1PINPOLARITY,
                        polarity);
        }
        else
        {
            /*
            * Do nothing. Error will be generated by the hardware
            */
        }
    }
}

static int32_t GPMC_programInstance(GPMC_Config *config)
{
    int32_t status = SystemP_SUCCESS;

    if(config != NULL)
    {
        const GPMC_HwAttrs *hwAttrs = config->attrs;
        GPMC_Object *object  = config->object;
        uint32_t devType = object->params.devType;

        /* Reset GPMC */
        CSL_REG32_FINS(hwAttrs->gpmcBaseAddr + CSL_GPMC_SYSCONFIG, GPMC_SYSCONFIG_SOFTRESET,CSL_GPMC_SYSCONFIG_SOFTRESET_RESET);

        status += GPMC_moduleResetStatusWaitTimeout(config,GPMC_MODULE_RESET_WAIT_TIME_MAX);

        if(status == SystemP_SUCCESS)
        {
            /*set GPMC in NORMAL mode*/
            CSL_REG32_FINS(hwAttrs->gpmcBaseAddr + CSL_GPMC_SYSCONFIG, GPMC_SYSCONFIG_SOFTRESET,CSL_GPMC_SYSCONFIG_SOFTRESET_NORMAL);

            /* Disable all interrupts */
            if(devType == CSL_GPMC_CONFIG1_DEVICETYPE_NANDLIKE)
            {
                GPMC_disableInterupt(hwAttrs->gpmcBaseAddr,GPMC_FIFOEVENT_INT);
                GPMC_disableInterupt(hwAttrs->gpmcBaseAddr,GPMC_TERMINALCOUNT_INT);
                GPMC_disableInterupt(hwAttrs->gpmcBaseAddr,GPMC_WAIT0EDGEDETECTION_INT);
                GPMC_disableInterupt(hwAttrs->gpmcBaseAddr,GPMC_WAIT1EDGEDETECTION_INT);
            }
            /* Disable Chip select*/
            CSL_REG32_FINS(hwAttrs->gpmcBaseAddr + CSL_GPMC_CONFIG7(object->params.chipSel), GPMC_CONFIG7_CSVALID, CSL_GPMC_CONFIG7_CSVALID_CSDISABLED);

            if(devType == CSL_GPMC_CONFIG1_DEVICETYPE_NANDLIKE)
            {
                    /* Timeout control disable */
                CSL_REG32_FINS(hwAttrs->gpmcBaseAddr + CSL_GPMC_TIMEOUT_CONTROL,
                            GPMC_TIMEOUT_CONTROL_TIMEOUTENABLE,
                            CSL_GPMC_TIMEOUT_CONTROL_TIMEOUTENABLE_TODISABLED);
                CSL_REG32_FINS(hwAttrs->gpmcBaseAddr + CSL_GPMC_TIMEOUT_CONTROL,
                            GPMC_TIMEOUT_CONTROL_TIMEOUTSTARTVALUE, 0);
            }
            /* Set CONFIG 1 Parameters*/
            CSL_REG32_FINS(hwAttrs->gpmcBaseAddr + CSL_GPMC_CONFIG1(object->params.chipSel),
                        GPMC_CONFIG1_GPMCFCLKDIVIDER, hwAttrs->clkDivider );
            CSL_REG32_FINS(hwAttrs->gpmcBaseAddr + CSL_GPMC_CONFIG1(object->params.chipSel),
                        GPMC_CONFIG1_MUXADDDATA,  hwAttrs->addrDataMux);

            CSL_REG32_FINS(hwAttrs->gpmcBaseAddr + CSL_GPMC_CONFIG1(object->params.chipSel),
                        GPMC_CONFIG1_WAITMONITORINGTIME, CSL_GPMC_CONFIG1_WAITMONITORINGTIME_ATVALID);

            CSL_REG32_FINS(hwAttrs->gpmcBaseAddr + CSL_GPMC_CONFIG1(object->params.chipSel),
                        GPMC_CONFIG1_WAITREADMONITORING, CSL_GPMC_CONFIG1_WAITREADMONITORING_WNOTMONIT);

            CSL_REG32_FINS(hwAttrs->gpmcBaseAddr + CSL_GPMC_CONFIG1(object->params.chipSel),
                        GPMC_CONFIG1_DEVICESIZE, 0x1);

            if(devType == CSL_GPMC_CONFIG1_DEVICETYPE_NORLIKE)
            {
                // CSL_REG32_WR(hwAttrs->gpmcBaseAddr + CSL_GPMC_CONFIG1(object->params.chipSel), 0x1000);
                CSL_REG32_FINS(hwAttrs->gpmcBaseAddr + CSL_GPMC_CONFIG1(object->params.chipSel),
                        GPMC_CONFIG1_READTYPE, hwAttrs->readType);
            }

            if(devType == CSL_GPMC_CONFIG1_DEVICETYPE_NANDLIKE)
            {
               /* Set the wait pin polarity */
                CSL_REG32_FINS(hwAttrs->gpmcBaseAddr + CSL_GPMC_CONFIG1(object->params.chipSel),
                            GPMC_CONFIG1_WAITPINSELECT,  hwAttrs->waitPinNum);
            }

            /* Set the Time granularity */
            CSL_REG32_FINS(hwAttrs->gpmcBaseAddr + CSL_GPMC_CONFIG1(object->params.chipSel),
                        GPMC_CONFIG1_TIMEPARAGRANULARITY, hwAttrs->timeLatency);

            CSL_REG32_FINS(hwAttrs->gpmcBaseAddr + CSL_GPMC_CONFIG1(object->params.chipSel),
                        GPMC_CONFIG1_READMULTIPLE,  hwAttrs->accessType);

            /* Set chip select base address*/
            CSL_REG32_FINS(hwAttrs->gpmcBaseAddr + CSL_GPMC_CONFIG7(object->params.chipSel),
                        GPMC_CONFIG7_BASEADDRESS, (hwAttrs->chipSelBaseAddr >> GPMC_CS_BASE_ADDR_SHIFT) & 0x3fU);

            if(devType == CSL_GPMC_CONFIG1_DEVICETYPE_NORLIKE)
            {
                CSL_REG32_WR(hwAttrs->gpmcBaseAddr + CSL_GPMC_CONFIG, 0x10);
            }
            /* Set chip select address mask*/
            CSL_REG32_FINS(hwAttrs->gpmcBaseAddr + CSL_GPMC_CONFIG7(object->params.chipSel),
                        GPMC_CONFIG7_MASKADDRESS,  hwAttrs->chipSelAddrSize);

            if(devType == CSL_GPMC_CONFIG1_DEVICETYPE_NANDLIKE)
            /* Set Wait pin polarity*/
            {
                GPMC_waitPinPolaritySelect((GPMC_Handle*)config, hwAttrs->waitPinNum, hwAttrs->waitPinPol);

                GPMC_interuptStatusClear(hwAttrs->gpmcBaseAddr,GPMC_WAIT0EDGEDETECTION_STATUS);
                /* Enable interupt for the WAIT PIN*/
                GPMC_enableInterupt(hwAttrs->gpmcBaseAddr,GPMC_WAIT0EDGEDETECTION_INT);
                /* Enable CHIP SELECT*/
                CSL_REG32_FINS(hwAttrs->gpmcBaseAddr + CSL_GPMC_CONFIG7(object->params.chipSel), \
                                GPMC_CONFIG7_CSVALID, CSL_GPMC_CONFIG7_CSVALID_CSENABLED);

                status += GPMC_waitPinStatusReadyWaitTimeout((GPMC_Handle)config, GPMC_WAIT_PIN_STATUS_WAIT_TIME_MAX);
            }
        }

    }

    return status;
}



static void GPMC_transferCallback(GPMC_Handle handle, GPMC_Transaction *msg)
{
    GPMC_Object   *object; /* GPMC object */

    /* Input parameter validation */
    if (handle != NULL)
    {
        /* Get the pointer to the object. */
        object = ((GPMC_Config*)handle)->object;

        /* Indicate transfer complete. */
        SemaphoreP_post(&object->transferComplete);
    }
}
