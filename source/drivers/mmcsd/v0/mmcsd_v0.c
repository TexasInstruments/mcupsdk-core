/*
 *  Copyright (C) 2021-24 Texas Instruments Incorporated
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
 *  \file mmcsd_v0.c
 *
 *  \brief File containing MMCSD Driver APIs implementation for version V0.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <string.h>
#include <drivers/mmcsd.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/CacheP.h>
#include <kernel/dpl/ClockP.h>
#include <drivers/hw_include/cslr.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

typedef struct
{
    void                *openLock;
    /**<  Lock to protect MMCSD open*/
    SemaphoreP_Object   lockObj;
    /**< Lock object */
} MMCSD_DrvObj;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* HLD ISR */
static void MMCSD_isr(void *arg);
/* LLD Transfer Complete callback functions */
static void MMCSD_LLD_transferCompleteCallback (void *args,
                                                int32_t transferStatus);
/* Complete Current Transfer */
static void MMCSD_completeCurrTransfer(MMCSD_Handle handle, int32_t xferStatus);
/* HLD Transfer Complete callback Functions */
static void MMCSD_transferCallback(MMCSD_Handle handle, int32_t transferStatus);
/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/** \brief Driver object */
static MMCSD_DrvObj gMmcsdDrvObj =
{
    .openLock = NULL,
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void MMCSD_init(void)
{
    int32_t         status;
    uint32_t        count;
    MMCSD_Object    *object;

    /* Init each driver instance object */
    for(count = 0U; count < gMmcsdConfigNum; count++)
    {
        /* Init object variables */
        object = gMmcsdConfig[count].object;
        DebugP_assert(NULL != object);
        (void)memset(object, 0, sizeof(MMCSD_Object));
    }

    /* Create the driver lock */
    status = SemaphoreP_constructMutex(&gMmcsdDrvObj.lockObj);
    if(SystemP_SUCCESS == status)
    {
        gMmcsdDrvObj.openLock = &gMmcsdDrvObj.lockObj;
    }
}

void MMCSD_deinit(void)
{
    /* Delete driver lock */
    if(NULL != gMmcsdDrvObj.openLock)
    {
        SemaphoreP_destruct(&gMmcsdDrvObj.lockObj);
        gMmcsdDrvObj.openLock = NULL;
    }
}

void MMCSD_Params_init(MMCSD_Params *mmcsdParams)
{
    if(mmcsdParams != NULL)
    {
        /* NULL init deviceData */
        mmcsdParams->deviceData = NULL;
    }
}

MMCSD_Handle MMCSD_open(uint32_t index, const MMCSD_Params *openParams)
{
    int32_t             status = SystemP_SUCCESS;
    MMCSD_Handle        handle = NULL;
    MMCSD_Config        *config = NULL;
    MMCSD_Object        *object = NULL;
    MMCSDLLD_Handle     mmcsdLldHandle = NULL;
    HwiP_Params         hwiPrms;
    const MMCSD_Attrs   *attrs;

    /* Check for valid index */
    if(index >= gMmcsdConfigNum)
    {
        status = SystemP_FAILURE;
    }
    else
    {
        config = &gMmcsdConfig[index];
    }

    /* Protect this region from a concurrent OSPI_Open */
    DebugP_assert(NULL != gMmcsdDrvObj.openLock);
    (void)SemaphoreP_pend(&gMmcsdDrvObj.lockObj, SystemP_WAIT_FOREVER);

    if(SystemP_SUCCESS == status)
    {
        object = config->object;
        DebugP_assert(NULL != object);
        DebugP_assert(NULL != config->attrs);
        attrs = config->attrs;
        if(true == object->isOpen)
        {
            /* Handle already opened */
            status = SystemP_FAILURE;
        }
    }

    if(SystemP_SUCCESS == status)
    {
        object->mmcsdLldHandle = &object->mmcsdLldObject;
        mmcsdLldHandle = object->mmcsdLldHandle;
        mmcsdLldHandle->initHandle = &object->mmcsdLldInitObject;

        /* Populate LLD initialization Object */
        mmcsdLldHandle->initHandle->ctrlBaseAddr = attrs->ctrlBaseAddr;
        mmcsdLldHandle->initHandle->ssBaseAddr = attrs->ssBaseAddr;
        mmcsdLldHandle->initHandle->inputClkFreq = attrs->inputClkFreq;
        mmcsdLldHandle->initHandle->cardType = attrs->cardType;
        mmcsdLldHandle->initHandle->autoAssignMaxSpeed =
                                                    attrs->autoAssignMaxSpeed;
        mmcsdLldHandle->initHandle->uaBusSpeed = attrs->uaBusSpeed;
        mmcsdLldHandle->initHandle->tuningType = attrs->tuningType;
        mmcsdLldHandle->initHandle->intrNum =  attrs->intrNum;
        mmcsdLldHandle->initHandle->slotType = attrs->slotType;
        mmcsdLldHandle->initHandle->busWidth = attrs->busWidth;
        mmcsdLldHandle->initHandle->enableDma = attrs->enableDma;

        mmcsdLldHandle->initHandle->phyType = attrs->phyType;
        mmcsdLldHandle->initHandle->pllEnableSD = attrs->pllEnableSD;
        mmcsdLldHandle->initHandle->deviceData = openParams->deviceData;
        mmcsdLldHandle->initHandle->dataBuf = openParams->dataBuf;

        mmcsdLldHandle->initHandle->Clock_getTicks = ClockP_getTicks;
        mmcsdLldHandle->initHandle->Clock_uSleep = ClockP_usleep;

        object->cardType = attrs->cardType;

        /* Register interrupt */
        if(true == attrs->intrEnable)
        {
            mmcsdLldHandle->transferCompleteCallback =
                                            MMCSD_LLD_transferCompleteCallback;

            HwiP_Params_init(&hwiPrms);
            hwiPrms.intNum      = attrs->intrNum;
            hwiPrms.callback    = &MMCSD_isr;
            hwiPrms.args        = (void *)config;
            hwiPrms.eventId     = (uint16_t)attrs->eventId;
            hwiPrms.isPulse     = 0U;
            hwiPrms.priority    = (uint8_t)(attrs->intrPriority);
            hwiPrms.isFIQ       = 0U;
            status += HwiP_construct(&object->hwiObj, &hwiPrms);
            DebugP_assert(status == SystemP_SUCCESS);
        }

        /*
         * Construct thread safe handles for this MMCSD peripheral
         * Semaphore to provide exclusive access to the MMCSD peripheral
         */
        status = SemaphoreP_constructMutex(&object->mutex);
        DebugP_assert(status == SystemP_SUCCESS);

        /* Store Transfer Mode */
        object->transferMode = openParams->transferMode;

        if(true == attrs->intrEnable)
        {
            if(openParams->transferMode == MMCSD_MODE_CALLBACK)
            {
                /* Save the callback function pointer */
                object->txnCallbackFxn = openParams->txnCallbackFxn;
            }
            else if(openParams->transferMode == MMCSD_MODE_BLOCKING)
            {
                status = SemaphoreP_constructBinary(&object->xferCompleteSemObj,
                                                    0U);
                DebugP_assert(status == SystemP_SUCCESS);
                /* Store internal callback function */
                object->txnCallbackFxn = &MMCSD_transferCallback;
            }
            else
            {
                /* No Code */
            }
        }
        /* Initialize LLD driver */
        if(MMCSD_lld_init(mmcsdLldHandle) == MMCSD_STS_SUCCESS)
        {
            status = SystemP_SUCCESS;
            mmcsdLldHandle->args = config;
        }
        else
        {
            status = SystemP_FAILURE;
        }
        DebugP_assert(status == SystemP_SUCCESS);
    }

    if(SystemP_SUCCESS == status)
    {
        object->isOpen = true;
        handle = (MMCSD_Handle)config;
    }

    SemaphoreP_post(&gMmcsdDrvObj.lockObj);

    /* Free up resources in case of error */
    if(SystemP_SUCCESS != status)
    {
        if(NULL != config)
        {
            MMCSD_close((MMCSD_Handle)config);
        }
    }

    return handle;
}

void MMCSD_close(MMCSD_Handle handle)
{
    int32_t             status = SystemP_SUCCESS;

    /* Input parameter validation */
    if (handle != NULL)
    {
        /* Get the pointer to the object */
        MMCSD_Object *object = ((MMCSD_Config *)handle)->object;
        const MMCSD_Attrs *attrs = ((MMCSD_Config *)handle)->attrs;

        DebugP_assert(NULL != gMmcsdDrvObj.openLock);
        (void)SemaphoreP_pend(&gMmcsdDrvObj.lockObj, SystemP_WAIT_FOREVER);

        if(MMCSD_lld_deInit(object->mmcsdLldHandle) == MMCSD_STS_SUCCESS)
        {
            status = SystemP_SUCCESS;
        }
        else
        {
            status = SystemP_FAILURE;
        }
        DebugP_assert(status == SystemP_SUCCESS);

        if (true == attrs->intrEnable)
        {
            /* Destruct the Hwi */
            (void)HwiP_destruct(&object->hwiObj);
        }

        /* Destruct the instance lock */
        (void)SemaphoreP_destruct(&object->mutex);

        if (true == attrs->intrEnable)
        {
            if (MMCSD_MODE_BLOCKING == object->transferMode)
            {
                /* Destruct the transfer completion lock */
                (void)SemaphoreP_destruct(&object->xferCompleteSemObj);
            }
        }

        object->isOpen = (bool)false;

        SemaphoreP_post(&gMmcsdDrvObj.lockObj);
        (void)memset(object, 0, sizeof(MMCSD_Object));
    }
}

MMCSD_Handle MMCSD_getHandle(uint32_t driverInstanceIndex)
{
    MMCSD_Handle         handle = NULL;
    /* Check index */
    if(driverInstanceIndex < gMmcsdConfigNum)
    {
        MMCSD_Object *object;
        object = gMmcsdConfig[driverInstanceIndex].object;

        if(object && (true == object->isOpen))
        {
            /* valid handle */
            handle = (MMCSD_Handle)(&gMmcsdConfig[driverInstanceIndex]);
        }
    }
    return handle;
}

int32_t MMCSD_read(MMCSD_Handle handle, uint8_t *buf,
                   uint32_t startBlk, uint32_t numBlks)
{
    int32_t status = SystemP_SUCCESS;
    MMCSD_Object *object = ((MMCSD_Config *)handle)->object;
    MMCSD_Attrs const *attrs = ((MMCSD_Config *)handle)->attrs;
    MMCSDLLD_Handle mmcsdLldHandle = object->mmcsdLldHandle;

    if(object->isOpen)
    {
        status = SemaphoreP_pend(&object->mutex, SystemP_WAIT_FOREVER);

        if(status == SystemP_SUCCESS)
        {
            /* API is ready for use */
            if(attrs->intrEnable == true)
            {
                (void)HwiP_disableInt(attrs->intrNum);
            }

            if(attrs->cardType == MMCSD_CARD_TYPE_SD)
            {
                if(attrs->intrEnable)
                {
                    status = MMCSD_lld_read_SD_Intr(mmcsdLldHandle, buf,
                                                    startBlk, numBlks);
                }
                else
                {
                    status = MMCSD_lld_read_SD_Poll(mmcsdLldHandle, buf,
                                                    startBlk, numBlks);
                }
            }
            else if(attrs->cardType == MMCSD_CARD_TYPE_EMMC)
            {
                if(attrs->intrEnable)
                {
                    status = MMCSD_lld_read_MMC_Intr(mmcsdLldHandle, buf,
                                                     startBlk, numBlks);
                }
                else
                {
                    status = MMCSD_lld_read_MMC_Poll(mmcsdLldHandle, buf,
                                                     startBlk, numBlks);
                }
            }
            else
            {
                /* No Code */
            }

            if(attrs->intrEnable == true)
            {
                if( (status == MMCSD_STS_SUCCESS) &&
                    (object->transferMode == MMCSD_MODE_BLOCKING))
                {
                    (void)HwiP_enableInt(attrs->intrNum);
                    status = SemaphoreP_pend(&object->xferCompleteSemObj,
                                             SystemP_WAIT_FOREVER);
                }
                else
                {
                    (void)HwiP_enableInt(attrs->intrNum);
                }
            }

            (void)SemaphoreP_post(&object->mutex);
        }
        else
        {
            /* Mutex not Available */
        }
    }

    return status;
}

int32_t MMCSD_write(MMCSD_Handle handle, uint8_t *buf,
                    uint32_t startBlk, uint32_t numBlks)
{
    int32_t status = SystemP_SUCCESS;
    MMCSD_Object *object = ((MMCSD_Config *)handle)->object;
    MMCSD_Attrs const *attrs = ((MMCSD_Config *)handle)->attrs;
    MMCSDLLD_Handle mmcsdLldHandle = object->mmcsdLldHandle;

    if(object->isOpen)
    {
        status = SemaphoreP_pend(&object->mutex, SystemP_WAIT_FOREVER);

        if(status == SystemP_SUCCESS)
        {
            /* API is ready for use */
            if(attrs->intrEnable == true)
            {
                (void)HwiP_disableInt(attrs->intrNum);
            }

            if(attrs->cardType == MMCSD_CARD_TYPE_SD)
            {
                if(attrs->intrEnable)
                {
                    status = MMCSD_lld_write_SD_Intr(mmcsdLldHandle, buf,
                                                     startBlk, numBlks);
                }
                else
                {
                    status = MMCSD_lld_write_SD_Poll(mmcsdLldHandle, buf,
                                                     startBlk, numBlks);
                }
            }
            else if(attrs->cardType == MMCSD_CARD_TYPE_EMMC)
            {
                if(attrs->intrEnable)
                {
                    status = MMCSD_lld_write_MMC_Intr(mmcsdLldHandle, buf,
                                                      startBlk, numBlks);
                }
                else
                {
                    status = MMCSD_lld_write_MMC_Poll(mmcsdLldHandle, buf,
                                                      startBlk, numBlks);
                }
            }
            else
            {
                /* No Code */
            }

            if(attrs->intrEnable == true)
            {
                (void)HwiP_enableInt(attrs->intrNum);

                if((status == MMCSD_STS_SUCCESS) &&
                (object->transferMode == MMCSD_MODE_BLOCKING))
                {
                    (void)SemaphoreP_pend(  &object->xferCompleteSemObj,
                                            SystemP_WAIT_FOREVER);
                }
            }

            (void)SemaphoreP_post(&object->mutex);
        }
        else
        {
            /* Mutex not Available */
        }
    }

    return status;
}

uint32_t MMCSD_getBlockSize(MMCSD_Handle handle)
{
    MMCSD_Object    *object = ((MMCSD_Config *)handle)->object;
    uint32_t        blockSize = 0U;

    blockSize = MMCSD_lld_getBlockSize(object->mmcsdLldHandle);

    return blockSize;
}

uint32_t MMCSD_getBlockCount(MMCSD_Handle handle)
{
    MMCSD_Object    *object = ((MMCSD_Config *)handle)->object;
    uint32_t        blockCount = 0U;

    if(object->cardType == MMCSD_CARD_TYPE_EMMC)
    {
        MMCSD_EmmcDeviceData *emmcDeviceData;
        emmcDeviceData = (MMCSD_EmmcDeviceData *)
                                    (object->mmcsdLldInitObject.deviceData);
        blockCount = emmcDeviceData->blockCount;
    }
    else
    {
        MMCSD_SdDeviceData *sdDeviceData;
        sdDeviceData = (MMCSD_SdDeviceData *)
                                    (object->mmcsdLldInitObject.deviceData);
        blockCount = sdDeviceData->blockCount;
    }

    return blockCount;
}

uint32_t MMCSD_isHC(MMCSD_Handle handle)
{
    MMCSD_Object    *object = ((MMCSD_Config *)handle)->object;
    uint32_t        isHC = 0U;

    if(object->cardType == MMCSD_CARD_TYPE_EMMC)
    {
        MMCSD_EmmcDeviceData *emmcDeviceData;
        emmcDeviceData = (MMCSD_EmmcDeviceData *)
                                    (object->mmcsdLldInitObject.deviceData);
        if(emmcDeviceData->isHC)
        {
            isHC = 1U;
        }
    }
    else
    {
        MMCSD_SdDeviceData *sdDeviceData;
        sdDeviceData = (MMCSD_SdDeviceData *)
                                    (object->mmcsdLldInitObject.deviceData);
        if(sdDeviceData->isHC)
        {
            isHC = 1U;
        }
    }

    return isHC;
}

int32_t MMCSD_change_Bus_Config(MMCSD_Handle handle, uint32_t busSpeed,
                                uint32_t busWidth)
{
    MMCSD_Object *object = ((MMCSD_Config *)handle)->object;
    MMCSDLLD_Handle mmcsdLldHandle = object->mmcsdLldHandle;
    int32_t status = SystemP_SUCCESS;

    status = SemaphoreP_pend(&object->mutex, SystemP_WAIT_FOREVER);

    if(status == SystemP_SUCCESS)
    {
        if(object->cardType == MMCSD_CARD_TYPE_EMMC)
        {
            status = MMCSD_lld_change_Bus_Config_MMC(mmcsdLldHandle, busSpeed,
                                                     busWidth);
        }
        else if(object->cardType == MMCSD_CARD_TYPE_SD)
        {
            status = MMCSD_lld_change_Bus_Config_SD(mmcsdLldHandle, busSpeed);
        }
        else
        {
            /* No Code */
        }

        if(status != SystemP_SUCCESS)
        {
            status = SystemP_FAILURE;
        }

        (void)SemaphoreP_post(&object->mutex);
    }
    else
    {
        /* Mutex not Available */
    }

    return status;
}

int32_t MMCSD_change_Tuning_Type(MMCSD_Handle handle, uint32_t tuningType)
{
    MMCSD_Object *object = ((MMCSD_Config *)handle)->object;
    MMCSDLLD_Handle mmcsdLldHandle = object->mmcsdLldHandle;
    int32_t status = SystemP_SUCCESS;

    status = MMCSD_lld_change_Tuning_Type(mmcsdLldHandle, tuningType);

    if(status != SystemP_SUCCESS)
    {
        status = SystemP_FAILURE;
    }

    return status;
}

int32_t MMCSD_enableBootPartition(MMCSD_Handle handle, uint32_t partitionNum)
{
    int32_t status = SystemP_SUCCESS;
    MMCSD_Object *object = ((MMCSD_Config *)handle)->object;
    MMCSDLLD_Handle mmcsdLldHandle = object->mmcsdLldHandle;

    status = SemaphoreP_pend(&object->mutex, SystemP_WAIT_FOREVER);

    if(status == SystemP_SUCCESS)
    {
        status = MMCSD_lld_enableBootPartition(mmcsdLldHandle, partitionNum);

        (void)SemaphoreP_post(&object->mutex);
    }
    else
    {
        /* Mutex not Available */
    }

    return status;
}

int32_t MMCSD_disableBootPartition(MMCSD_Handle handle)
{
    int32_t status = SystemP_SUCCESS;
    MMCSD_Object *object = ((MMCSD_Config *)handle)->object;
    MMCSDLLD_Handle mmcsdLldHandle = object->mmcsdLldHandle;

    status = SemaphoreP_pend(&object->mutex, SystemP_WAIT_FOREVER);

    if(status == SystemP_SUCCESS)
    {
        status = MMCSD_lld_disableBootPartition(mmcsdLldHandle);
        (void)SemaphoreP_post(&object->mutex);
    }
    else
    {
        /* Mutex not Available */
    }

    return status;
}

/* ========================================================================== */
/*                       ISR Function Definitions                             */
/* ========================================================================== */

static void MMCSD_isr(void *arg)
{
    MMCSD_Handle        handle = (MMCSD_Handle)arg;
    MMCSD_Object        *object = NULL;

    /* Input parameter validation */
    if (handle != NULL)
    {
        /* Get the pointer to the object */
        object = (MMCSD_Object *)(((MMCSD_Config *)handle)->object);
        /* Call LLD ISR */
        MMCSD_lld_Isr(object->mmcsdLldHandle);
    }
}

/* ========================================================================== */
/*                     Internal function definitions                          */
/* ========================================================================== */
static void MMCSD_transferCallback(MMCSD_Handle handle, int32_t transferStatus)
{
    MMCSD_Object *object;

    /* Input parameter validation */
    if ((handle != NULL))
    {
        /* Get the pointer to the object */
        object = (MMCSD_Object *)(((MMCSD_Config *)handle)->object);
        /* Indicate transfer complete */
        SemaphoreP_post(&object->xferCompleteSemObj);
    }
}

static void MMCSD_LLD_transferCompleteCallback (void *args,
                                                int32_t transferStatus)
{
    MMCSDLLD_Handle mmcsdLldHandle = (MMCSDLLD_Handle)args;

    if(NULL_PTR != mmcsdLldHandle)
    {
        MMCSD_Handle mmcsdHldhandle = (MMCSD_Handle)mmcsdLldHandle->args;

        if(NULL != mmcsdHldhandle)
        {
            MMCSD_completeCurrTransfer(mmcsdHldhandle, transferStatus);
        }
    }
}

static void MMCSD_completeCurrTransfer(MMCSD_Handle handle, int32_t xferStatus)
{
    MMCSD_Object *object = (MMCSD_Object *)(((MMCSD_Config *)handle)->object);

    /* Callback to application or post semaphore */
    object->txnCallbackFxn(handle, xferStatus);
}