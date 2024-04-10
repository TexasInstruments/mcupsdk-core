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
 *  \file mcspi_v0.c
 *
 *  \brief File containing MCSPI Driver APIs implementation for version V0.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

/* This is needed for memset/memcpy */
#include <string.h>
#include <drivers/mcspi.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/AddrTranslateP.h>
#include <drivers/mcspi/v0/lld/mcspi_lld.h>
#include <drivers/mcspi/v0/lld/dma/mcspi_dma.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

typedef struct
{
    void                   *lock;
    /**< Driver lock - to protect across open/close */
    SemaphoreP_Object       lockObj;
    /**< Driver lock object */
} MCSPI_DrvObj;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* Driver internal functions */
static int32_t MCSPI_checkOpenParams(const MCSPI_OpenParams *openPrms);
/* Transfer complete callback function */
void MCSPI_transferCallback (void *args, uint32_t transferStatus);
/* Error callback function */
void MCSPI_errorCallback (void  *args, uint32_t transferStatus);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/** \brief Driver object */
static MCSPI_DrvObj     gMcspiDrvObj =
{
    .lock           = NULL,
};

extern uint32_t              gMcspiNumCh[];
extern MCSPI_DmaHandle       gMcspiDmaHandle[];
extern MCSPI_DmaChConfig    *gMcspiDmaChConfig[];
extern MCSPI_ChConfig       *gConfigMcspiChCfg[];

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void MCSPI_init(void)
{
    uint32_t       cnt;
    MCSPI_Object  *obj;
    int32_t        status = SystemP_SUCCESS;

    /* Init each driver instance object */
    for (cnt = 0U; cnt < gMcspiConfigNum; cnt++)
    {
        /* initialize object varibles */
        obj = gMcspiConfig[cnt].object;
        DebugP_assert(NULL_PTR != obj);
        (void)memset(obj, 0, sizeof(MCSPI_Object));
    }

    /* Create driver lock */
    status = SemaphoreP_constructMutex(&gMcspiDrvObj.lockObj);
    if(status == SystemP_SUCCESS)
    {
        gMcspiDrvObj.lock = &gMcspiDrvObj.lockObj;
    }

    return;
}

void MCSPI_deinit(void)
{
    /* Delete driver lock */
    if(NULL != gMcspiDrvObj.lock)
    {
        SemaphoreP_destruct(&gMcspiDrvObj.lockObj);
        gMcspiDrvObj.lock = NULL;
    }

    return;
}

MCSPI_Handle MCSPI_open(uint32_t index, const MCSPI_OpenParams *openPrms)
{
    int32_t              status = SystemP_SUCCESS;
    uint8_t              configNum, chNum;
    MCSPI_Handle         handle = NULL;
    MCSPI_Config        *config = NULL;
    MCSPI_Object        *obj    = NULL;
    HwiP_Params          hwiPrms;
    MCSPILLD_Handle      mcspiLldHandle;
    MCSPILLD_InitHandle  mcspiLldInithandle;
    MCSPI_ChConfig      *lldChCfg, *chConfig;
    MCSPI_DmaChConfig    dmaChConfig;
    const MCSPI_Attrs   *attrs;

    /* Check index */
    if(index >= gMcspiConfigNum)
    {
        status = SystemP_FAILURE;
    }
    else
    {
        config = &gMcspiConfig[index];
    }

    DebugP_assert(NULL_PTR != gMcspiDrvObj.lock);
    status += SemaphoreP_pend(&gMcspiDrvObj.lockObj, SystemP_WAIT_FOREVER);

    if(SystemP_SUCCESS == status)
    {
        obj = config->object;
        DebugP_assert(NULL_PTR != obj);
        DebugP_assert(NULL_PTR != config->attrs);
        attrs = config->attrs;

        /* Init state */
        obj->handle = (MCSPI_Handle) config;
        (void)memset(&obj->chObj, 0, sizeof(obj->chObj));
        obj->baseAddr = (uint32_t) AddrTranslateP_getLocalAddr(attrs->baseAddr);

        if(NULL != openPrms)
        {
            (void)memcpy(&obj->openPrms, openPrms, sizeof(MCSPI_OpenParams));
        }
        else
        {
            /* Init with default if NULL is passed */
            MCSPI_OpenParams_init(&obj->openPrms);
        }

        /*  Mapping HLD parameter with LLD. */
        obj->mcspiLldHandle           = &obj->mcspiLldObject;
        mcspiLldHandle                = obj->mcspiLldHandle;
        mcspiLldHandle->hMcspiInit    = &obj->mcspiLldInitObj;
        mcspiLldInithandle            = mcspiLldHandle->hMcspiInit;

        /* Check open parameters */
        status += MCSPI_checkOpenParams(&obj->openPrms);

        if((status == SystemP_SUCCESS) && (openPrms != NULL))
        {
            /* populating LLD parameters. */
            mcspiLldHandle->baseAddr                 = obj->baseAddr;
            mcspiLldHandle->state                    = MCSPI_STATE_RESET;
            mcspiLldHandle->args                     = (MCSPI_Handle)obj->handle;
            mcspiLldHandle->errorFlag                = 0;
            mcspiLldInithandle->inputClkFreq         = attrs->inputClkFreq;
            mcspiLldInithandle->intrNum              = attrs->intrNum;
            mcspiLldInithandle->operMode             = attrs->operMode;
            mcspiLldInithandle->chMode               = attrs->chMode;
            mcspiLldInithandle->pinMode              = attrs->pinMode;
            mcspiLldInithandle->initDelay            = attrs->initDelay;
            mcspiLldInithandle->multiWordAccess      = attrs->multiWordAccess;
            mcspiLldInithandle->msMode               = openPrms->msMode;
            mcspiLldInithandle->mcspiDmaHandle       = (MCSPI_DmaHandle) gMcspiDmaHandle[0];
            mcspiLldInithandle->clockP_get           = ClockP_getTicks;
            mcspiLldInithandle->transferCallbackFxn  = MCSPI_transferCallback;
            mcspiLldInithandle->errorCallbackFxn     = MCSPI_errorCallback;

            chConfig    = gConfigMcspiChCfg[index];
            dmaChConfig = gMcspiDmaChConfig[index];

            for(configNum = 0; configNum < gMcspiNumCh[index]; configNum++)
            {
                chNum = chConfig[configNum].chNum;
                mcspiLldInithandle->chObj[chNum].chCfg = &chConfig[configNum];
                lldChCfg = mcspiLldInithandle->chObj[chNum].chCfg;

                lldChCfg->chNum         = chNum;
                lldChCfg->frameFormat   = chConfig[configNum].frameFormat;
                lldChCfg->bitRate       = chConfig[configNum].bitRate;
                lldChCfg->csPolarity    = chConfig[configNum].csPolarity;
                lldChCfg->trMode        = chConfig[configNum].trMode;
                lldChCfg->inputSelect   = chConfig[configNum].inputSelect;
                lldChCfg->dpe0          = chConfig[configNum].dpe0;
                lldChCfg->dpe1          = chConfig[configNum].dpe1;
                lldChCfg->slvCsSelect   = chConfig[configNum].slvCsSelect;
                lldChCfg->startBitEnable   = chConfig[configNum].startBitEnable;
                lldChCfg->startBitPolarity = chConfig[configNum].startBitPolarity;
                lldChCfg->turboEnable   = chConfig[configNum].turboEnable;
                lldChCfg->csIdleTime    = chConfig[configNum].csIdleTime;
                lldChCfg->defaultTxData = chConfig[configNum].defaultTxData;
                lldChCfg->txFifoTrigLvl = chConfig[configNum].txFifoTrigLvl;
                lldChCfg->rxFifoTrigLvl = chConfig[configNum].rxFifoTrigLvl;

                mcspiLldInithandle->chObj[chNum].dmaChCfg = dmaChConfig;
                mcspiLldInithandle->chObj[chNum].dmaChConfigNum = configNum;
                mcspiLldInithandle->chEnabled[chNum] = TRUE;
            }
        }

        if(MCSPI_OPER_MODE_DMA == attrs->operMode)
        {
            status = MCSPI_lld_initDma(mcspiLldHandle);
        }
        else
        {
            status = MCSPI_lld_init(mcspiLldHandle);
        }
        if(status == MCSPI_STATUS_SUCCESS)
        {
            status = SystemP_SUCCESS;
        }
        else
        {
            status = SystemP_FAILURE;
        }
    }
    if(SystemP_SUCCESS == status)
    {
        /* Create transfer sync semaphore */
        status = SemaphoreP_constructBinary(&obj->transferSemObj, 0U);
        obj->transferSem = &obj->transferSemObj;
        mcspiLldHandle->transferMutex = obj->transferSem;

        /* Register interrupt */
        if(MCSPI_OPER_MODE_INTERRUPT == attrs->operMode)
        {
            HwiP_Params_init(&hwiPrms);
            hwiPrms.intNum      = attrs->intrNum;
            hwiPrms.priority    = attrs->intrPriority;
            if(MCSPI_MS_MODE_CONTROLLER == obj->openPrms.msMode)
            {
                hwiPrms.callback    = &MCSPI_lld_controllerIsr;
            }
            else
            {
                hwiPrms.callback    = &MCSPI_lld_peripheralIsr;
            }

            hwiPrms.priority    = attrs->intrPriority;
            hwiPrms.args        = (void *) mcspiLldHandle;
            status += HwiP_construct(&obj->hwiObj, &hwiPrms);
            obj->hwiHandle = &obj->hwiObj;
            obj->hwiHandle = &obj->hwiObj;
        }
    }

    if(SystemP_SUCCESS == status)
    {
        obj->isOpen = TRUE;
        handle = (MCSPI_Handle) config;
    }

    SemaphoreP_post(&gMcspiDrvObj.lockObj);

    /* Free-up resources in case of error */
    if ((SystemP_SUCCESS != status) && ((NULL != config)))
    {
        MCSPI_close((MCSPI_Handle) config);
    }

    return (handle);
}

MCSPI_Handle MCSPI_getHandle(uint32_t index)
{
    MCSPI_Handle         handle = NULL;

    /* Check index */
    if(index < gMcspiConfigNum)
    {
        MCSPI_Object *object;

        object = gMcspiConfig[index].object;

        if(object && (TRUE == object->isOpen))
        {
            /* valid handle */
            handle = object->handle;
        }
    }

    return handle;
}

void MCSPI_close(MCSPI_Handle handle)
{
    MCSPI_Config        *config;
    MCSPI_Object        *obj;
    MCSPILLD_Handle      mcspiLldHandle;
    int32_t              status = SystemP_FAILURE;

    if(NULL != handle)
    {
        config = (MCSPI_Config *)handle;
        obj    = config->object;
        DebugP_assert(NULL_PTR != obj);
        DebugP_assert(NULL_PTR != config->attrs);
        DebugP_assert(NULL_PTR != gMcspiDrvObj.lock);

        mcspiLldHandle = obj->mcspiLldHandle;
        status = SemaphoreP_pend(&gMcspiDrvObj.lockObj, SystemP_WAIT_FOREVER);

        if(MCSPI_OPER_MODE_DMA == config->attrs->operMode)
        {
            status += MCSPI_lld_deInitDma(mcspiLldHandle);
            DebugP_assert(SystemP_SUCCESS == status);
        }
        else
        {
            status += MCSPI_lld_deInit(mcspiLldHandle);
            DebugP_assert(SystemP_SUCCESS == status);
        }

        if(NULL != obj->transferSem)
        {
            SemaphoreP_destruct(&obj->transferSemObj);
            obj->transferSem = NULL;
        }
        if(NULL != obj->hwiHandle)
        {
            HwiP_destruct(&obj->hwiObj);
            obj->hwiHandle = NULL;
        }

        obj->isOpen = FALSE;
        SemaphoreP_post(&gMcspiDrvObj.lockObj);
    }

    return;
}

int32_t MCSPI_chConfig(MCSPI_Handle handle, const MCSPI_ChConfig *chCfg)
{
    /* MCSPI channels are already initialized in MCSPI_lld_init().
    *  Here this function is declared in order to avoid HLD flow
    */
    return SystemP_SUCCESS;
}

int32_t MCSPI_dmaChConfig(MCSPI_Handle handle,
                       const MCSPI_ChConfig *chCfg,
                       const MCSPI_DmaChConfig *dmaChCfg)
{
    /* MCSPI dma channels are already initialized in MCSPI_lld_init().
    *  Here this function is declared in order to avoid HLD flow
    */
    return SystemP_SUCCESS;
}

int32_t MCSPI_transfer(MCSPI_Handle handle, MCSPI_Transaction *transaction)
{
    int32_t             status = MCSPI_STATUS_SUCCESS, semStatus;
    int32_t             lldStatus = MCSPI_STATUS_SUCCESS;
    uint32_t            chNum = 0;
    MCSPI_Config       *config;
    MCSPI_Object       *obj;
    const MCSPI_Attrs  *attrs;
    uintptr_t key;

    MCSPILLD_Handle         mcspiLldHandle;
    MCSPILLD_InitHandle     mcspiLldInithandle;
    MCSPI_ExtendedParams    extendedParams;

    /* Check parameters */
    if((NULL == handle) || (NULL == transaction))
    {
        lldStatus = MCSPI_INVALID_PARAM;
    }

    if(MCSPI_STATUS_SUCCESS == lldStatus)
    {
        config = (MCSPI_Config *) handle;
        obj = config->object;
        DebugP_assert(NULL_PTR != obj);
        DebugP_assert(NULL_PTR != config->attrs);
        attrs = config->attrs;

        mcspiLldHandle = obj->mcspiLldHandle;
        mcspiLldInithandle = obj->mcspiLldHandle->hMcspiInit;
        /* populate transaction parameters. */
        extendedParams.channel   = transaction->channel;
        extendedParams.dataSize  = transaction->dataSize;
        extendedParams.csDisable = transaction->csDisable;
        extendedParams.args      = transaction->args;

        /*  update timeout parameter from syscfg  */
        transaction->timeout = obj->openPrms.transferTimeout;
        key = HwiP_disable();

        /* Check if any transaction is in progress */
        if(NULL == obj->transaction)
        {
            /* Start transfer */
            obj->transaction = transaction;
            obj->transaction->status = MCSPI_TRANSFER_STARTED;
            (void)memcpy(&mcspiLldHandle->transaction, obj->transaction,
                         sizeof(mcspiLldHandle->transaction));
            HwiP_restore(key);

            if ((MCSPI_OPER_MODE_INTERRUPT == attrs->operMode) ||
                (MCSPI_OPER_MODE_DMA == attrs->operMode))
            {
                if (MCSPI_OPER_MODE_INTERRUPT == attrs->operMode)
                {
                    lldStatus = MCSPI_lld_readWriteIntr(mcspiLldHandle, transaction->txBuf,
                                                    transaction->rxBuf, transaction->count,
                                                    transaction->timeout, &extendedParams);
                }
                else
                {
                    lldStatus = MCSPI_lld_readWriteDma(mcspiLldHandle, transaction->txBuf,
                                                    transaction->rxBuf, transaction->count,
                                                    transaction->timeout, &extendedParams);
                }
                if (lldStatus == MCSPI_STATUS_SUCCESS)
                {
                    if (obj->openPrms.transferMode == MCSPI_TRANSFER_MODE_BLOCKING)
                    {
                        /* Block on transferSem till the transfer completion. */
                        DebugP_assert(NULL_PTR != obj->transferSem);
                        semStatus = SemaphoreP_pend(&obj->transferSemObj, obj->openPrms.transferTimeout);
                        if (semStatus == SystemP_SUCCESS)
                        {
                            status = SystemP_SUCCESS;
                        }
                        else
                        {
                            MCSPI_stop(mcspiLldHandle, &mcspiLldInithandle->chObj[chNum], chNum);
                            status = SystemP_FAILURE;
                            /* update HLD current transaction status from LLD */
                            (void)memcpy(obj->transaction,
                                         &mcspiLldHandle->transaction,
                                         sizeof(mcspiLldHandle->transaction));

                            transaction->status = MCSPI_TRANSFER_TIMEOUT;
                            obj->transaction = NULL;
                        }
                    }
                }
                else
                {
                    status = SystemP_FAILURE;
                }
            }
            else
            {
                lldStatus = MCSPI_lld_readWrite(mcspiLldHandle, transaction->txBuf,
                                            transaction->rxBuf, transaction->count,
                                            transaction->timeout, &extendedParams);

                if (lldStatus == MCSPI_STATUS_SUCCESS)
                {
                    status = SystemP_SUCCESS;
                    /* update HLD current transaction status from LLD */
                    (void)memcpy(obj->transaction,
                                 &mcspiLldHandle->transaction,
                                 sizeof(mcspiLldHandle->transaction));

                    transaction->status = MCSPI_TRANSFER_COMPLETED;
                }
                else if(lldStatus == MCSPI_TRANSFER_TIMEOUT)
                {
                    status = SystemP_FAILURE;
                    transaction->status = MCSPI_TRANSFER_TIMEOUT;
                }
                else
                {
                    status  = SystemP_FAILURE;
                    transaction->status = MCSPI_TRANSFER_FAILED;
                }

                obj->transaction = NULL;
            }
        }
        else
        {
            HwiP_restore(key);
            /* other transaction is in progress */
            status = SystemP_FAILURE;
            transaction->status = MCSPI_TRANSFER_CANCELLED;
        }
    }

    return (status);
}

int32_t MCSPI_transferCancel(MCSPI_Handle handle)
{
    int32_t             status = MCSPI_STATUS_SUCCESS;
    int32_t             lldStatus = MCSPI_STATUS_SUCCESS;
    MCSPI_Config        *config;
    MCSPI_Object        *obj;
    const MCSPI_Attrs   *attrs;
    MCSPILLD_Handle      mcspiLldHandle;

    /* Check parameters */
    if (NULL == handle)
    {
        status = MCSPI_STATUS_FAILURE;
    }

    if(MCSPI_STATUS_SUCCESS == status)
    {
        config = (MCSPI_Config *) handle;
        obj    = config->object;
        DebugP_assert(NULL_PTR != obj);
        DebugP_assert(NULL_PTR != config->attrs);
        attrs = config->attrs;
        if (obj->transaction == NULL)
        {
            status = MCSPI_STATUS_FAILURE;
        }
    }

    if(MCSPI_STATUS_SUCCESS == status)
    {
        mcspiLldHandle = obj->mcspiLldHandle;
        if (MCSPI_OPER_MODE_DMA != attrs->operMode)
        {
            lldStatus = MCSPI_lld_readWriteCancel(mcspiLldHandle);
        }
        else
        {
            lldStatus = MCSPI_lld_readWriteDmaCancel(mcspiLldHandle);
        }

        obj->transaction->status = MCSPI_TRANSFER_CANCELLED;
        obj->transaction = NULL;
    }
    if ((lldStatus != MCSPI_STATUS_SUCCESS) &&
        (lldStatus == MCSPI_INVALID_PARAM)  &&
        (lldStatus != MCSPI_TRANSFER_CANCELLED))
    {
        status = SystemP_FAILURE;
    }
    else
    {
        status = SystemP_SUCCESS;
    }

    return (status);
}

static int32_t MCSPI_checkOpenParams(const MCSPI_OpenParams *openPrms)
{
    int32_t     status = SystemP_SUCCESS;

    if((MCSPI_TRANSFER_MODE_CALLBACK == openPrms->transferMode) &&
       (NULL == openPrms->transferCallbackFxn))
    {
        DebugP_logError("[MCSPI] Callback should be provided when using callback mode !!!\r\n");
        status = SystemP_FAILURE;
    }

    return (status);
}

void MCSPI_transferCallback (void *args, uint32_t transferStatus)
{
    MCSPI_Config   *config;
    MCSPI_Object   *obj;

    MCSPILLD_Handle hMcspi = (MCSPILLD_Handle)args;
    if(NULL != hMcspi)
    {
        MCSPI_Handle handle = (MCSPI_Handle)hMcspi->args;

        if(NULL != handle)
        {
            config = (MCSPI_Config *) handle;
            obj    = config->object;

            (void)memcpy(obj->transaction,
                         &hMcspi->transaction,
                         sizeof(hMcspi->transaction));

            if(transferStatus == (uint32_t) MCSPI_TRANSFER_CANCELLED)
            {
                obj->transaction->status = MCSPI_TRANSFER_CANCELLED;
            }
            else
            {
                obj->transaction->status = MCSPI_TRANSFER_COMPLETED;
            }

            if((obj->openPrms.transferMode) == MCSPI_TRANSFER_MODE_CALLBACK)
            {
                obj->openPrms.transferCallbackFxn(hMcspi, obj->transaction);
            }
            else
            {
                SemaphoreP_post((SemaphoreP_Object *)hMcspi->transferMutex);
            }
            obj->transaction = NULL;
        }
    }
}

void MCSPI_errorCallback (void *args, uint32_t transferStatus)
{
    MCSPI_Config   *config;
    MCSPI_Object   *obj;

    MCSPILLD_Handle hMcspi = (MCSPILLD_Handle)args;
    if(NULL != hMcspi)
    {
        MCSPI_Handle handle = (MCSPI_Handle)hMcspi->args;

        if(NULL != handle)
        {
            config = (MCSPI_Config *) handle;
            obj    = config->object;
            if((obj->openPrms.transferMode) == MCSPI_TRANSFER_MODE_CALLBACK)
            {
                obj->openPrms.transferCallbackFxn(hMcspi, &hMcspi->transaction);
            }
            else
            {
                SemaphoreP_post((SemaphoreP_Object *)hMcspi->transferMutex);
            }

            (void)memcpy(obj->transaction,
                         &hMcspi->transaction,
                         sizeof(hMcspi->transaction));

            obj->transaction->status = MCSPI_TRANSFER_FAILED;
            obj->transaction = NULL;
        }
    }
}

/* ========================================================================== */
/*                       Advanced Function Definitions                        */
/* ========================================================================== */

uint32_t MCSPI_getBaseAddr(MCSPI_Handle handle)
{
    MCSPI_Config       *config;
    MCSPI_Object       *obj;
    uint32_t            baseAddr;

    /* Check parameters */
    if (NULL == handle)
    {
        baseAddr = 0U;
    }
    else
    {
        config   = (MCSPI_Config *) handle;
        obj      = config->object;
        baseAddr = obj->baseAddr;
    }

    return baseAddr;
}

int32_t MCSPI_reConfigFifo(MCSPI_Handle handle, uint32_t chNum, uint32_t numWordsRxTx)
{
    MCSPI_Config        *config;
    MCSPI_Object        *obj;
    int32_t              status = SystemP_SUCCESS;
    MCSPILLD_Handle      hMcspi;

    /* Check parameters */
    if((NULL == handle) || (chNum >= MCSPI_MAX_NUM_CHANNELS))
    {
        status = SystemP_FAILURE;
    }

    if(SystemP_SUCCESS == status)
    {
        config = (MCSPI_Config *) handle;
        obj = config->object;
        hMcspi = obj->mcspiLldHandle;

        status = MCSPI_lld_reConfigFifo(hMcspi, chNum, numWordsRxTx);
    }

    return status;
}
