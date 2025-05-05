/*
 *  Copyright (C) 2021-2024 Texas Instruments Incorporated
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
 *  \file qspi.c
 *
 *  \brief File containing QSPI Driver APIs implementation.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

/* This is needed for memset/memcpy */
#include <string.h>
#include <drivers/qspi.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/CacheP.h>
#include <drivers/hw_include/cslr.h>
#include <drivers/hw_include/hw_types.h>
#include <drivers/qspi/v0/lld/edma/qspi_edma_lld.h>
#include <kernel/dpl/ClockP.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */


/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

typedef struct
{
    void *openLock;
    /**<  Lock to protect QSPI open*/
    SemaphoreP_Object lockObj;
    /**< Lock object */
} QSPI_DrvObj;

typedef struct
{
    int32_t                count;
    /**< [IN] Number of frames for this transaction */
    void                   *buf;
    /**< [IN] void * to a buffer to receive/send data */
    uint32_t                wlen;
    /**< [IN] word length to be used for this transaction. */
    uint32_t                cmdRegVal;
    /**< [IN] cmd register value to be written. */
} QSPI_ConfigAccess;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
static void QSPI_interruptCallback(void* args);
/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/** \brief Driver object */
static QSPI_DrvObj gQspiDrvObj =
{
    .openLock      = NULL,
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void QSPI_init(void)
{
    uint32_t count;
    QSPI_Object *obj;

    /* Init each driver instance object */
    for(count = 0U; count < gQspiConfigNum; count++)
    {
        /* Init object variables */
        obj = gQspiConfig[count].object;
        DebugP_assert(NULL_PTR != obj);
        (void) memset(obj, 0, sizeof(QSPI_Object));
    }

    /* Create the driver lock */
    (void) SemaphoreP_constructMutex(&gQspiDrvObj.lockObj);
    gQspiDrvObj.openLock = &gQspiDrvObj.lockObj;
}

void QSPI_deinit(void)
{
    /* Delete driver lock */
    if(NULL_PTR != gQspiDrvObj.openLock)
    {
        SemaphoreP_destruct(&gQspiDrvObj.lockObj);
        gQspiDrvObj.openLock = NULL;
    }
}

QSPI_Handle QSPI_open(uint32_t index, const QSPI_Params *openParams)
{
    int32_t status = SystemP_SUCCESS;
    QSPI_Handle handle = NULL;
    QSPI_Config *config = NULL;
    QSPI_Object *obj = NULL;
    HwiP_Params hwiPrms;
    const QSPI_Attrs *attrs;
    uint32_t edmaInterrupt;

    QSPILLD_InitHandle   qspilldInitHandle;
    /* QSPI LLD Init Handle */
    QSPILLD_Handle      qspilldHandle;
    /* QSPI LLD Handle */

    /* Check for valid index */
    if(index >= gQspiConfigNum)
    {
        status = SystemP_FAILURE;
    }
    else
    {
        config = &gQspiConfig[index];
    }

    /* Protect this region from a concurrent QSPI_Open */
    DebugP_assert(NULL_PTR != gQspiDrvObj.openLock);
    status += SemaphoreP_pend(&gQspiDrvObj.lockObj, SystemP_WAIT_FOREVER);

    if(SystemP_SUCCESS == status)
    {
        obj = config->object;
        DebugP_assert(NULL_PTR != obj);
        DebugP_assert(NULL_PTR != config->attrs);
        attrs = config->attrs;
        if(TRUE == obj->isOpen)
        {
            /* Handle already opened */
            status = SystemP_FAILURE;
        }
    }

    if(SystemP_SUCCESS == status)
    {
        obj->handle = (QSPI_Handle)config;

        /*  Mapping HLD parameter with LLD. */
        obj->qspilldHandle                  = &obj->qspilldObject;
        qspilldHandle                       = obj->qspilldHandle;
        qspilldHandle->hQspiInit            = &obj->qspilldInitObject;
        qspilldInitHandle                   = qspilldHandle->hQspiInit;

        /* populating LLD parameters. */
        qspilldHandle->baseAddr             = attrs->baseAddr;
        qspilldHandle->state                = QSPI_STATE_RESET;
        qspilldHandle->args                 = (void *)obj->handle;
        qspilldHandle->interruptCallback    = NULL;
        qspilldInitHandle->memMapBaseAddr   = attrs->memMapBaseAddr;
        qspilldInitHandle->inputClkFreq     = attrs->inputClkFreq;
        qspilldInitHandle->qspiClockDiv     = attrs->baudRateDiv;
        qspilldInitHandle->chipSelect       = attrs->chipSelect;
        qspilldInitHandle->csPol            = attrs->csPol;
        qspilldInitHandle->frmFmt           = attrs->frmFmt;
        qspilldInitHandle->dataDelay        = attrs->dataDelay;
        qspilldInitHandle->wrdLen           = attrs->wrdLen;
        qspilldInitHandle->rxLines          = attrs->rxLines;
        qspilldInitHandle->intrNum          = attrs->intrNum;
        qspilldInitHandle->intrEnable       = attrs->intrEnable;
        qspilldInitHandle->wordIntr         = attrs->wordIntr;
        qspilldInitHandle->frameIntr        = attrs->frameIntr;
        qspilldInitHandle->intrPriority     = attrs->intrPriority;
        qspilldInitHandle->dmaEnable        = attrs->dmaEnable;
        qspilldInitHandle->Clock_getTicks   = ClockP_getTicks;
        qspilldInitHandle->Clock_usecToTicks= ClockP_usecToTicks;
        qspilldHandle->transaction          = NULL;

        if(true == attrs->dmaEnable)
        {
            qspilldInitHandle->qspiDmaHandle    = EDMA_getHandle(openParams->edmaInst);
            qspilldInitHandle->qspiDmaChConfig = (QSPI_DmaChConfig) &gqspiEdmaParam;
            if (NULL != qspilldInitHandle->qspiDmaHandle)
            {
                edmaInterrupt = EDMA_isInterruptEnabled(qspilldInitHandle->qspiDmaHandle);
            }
            else
            {
                edmaInterrupt = FALSE;
            }
            if (edmaInterrupt == TRUE)
            {
                qspilldHandle->readCompleteCallback = &QSPI_interruptCallback;
            }
            else
            {
                qspilldHandle->readCompleteCallback = NULL;
            }

            status = QSPI_lld_initDma(qspilldHandle);
        }
        else
        {
            qspilldInitHandle->qspiDmaHandle = NULL;
            qspilldInitHandle->qspiDmaChConfig = NULL;
            qspilldHandle->readCompleteCallback = NULL;
            status = QSPI_lld_init(qspilldHandle);
        }

        /* Create instance lock */
        status += SemaphoreP_constructMutex(&obj->lockObj);

        /* Create transfer sync semaphore */
        status += SemaphoreP_constructBinary(&obj->transferSemObj, 0U);

        /* Register interrupt */
        if(true == attrs->intrEnable)
        {
            HwiP_Params_init(&hwiPrms);
            hwiPrms.intNum                  = attrs->intrNum;
            hwiPrms.callback                = &QSPI_lld_isr;
            hwiPrms.priority                = attrs->intrPriority;
            hwiPrms.args                    = (void *) qspilldHandle;
            hwiPrms.isPulse                 = 1U;
            status                          += HwiP_construct(&obj->hwiObj, &hwiPrms);
            qspilldHandle->interruptCallback = QSPI_interruptCallback;
        }
    }

    if(SystemP_SUCCESS == status)
    {
        obj->isOpen = 1;
        handle = (QSPI_Handle) config;
    }

    SemaphoreP_post(&gQspiDrvObj.lockObj);

    /* Free up resources in case of error */
    if(SystemP_SUCCESS != status)
    {
        if(NULL != config)
        {
            QSPI_close((QSPI_Handle) config);
        }
    }
    return handle;
}

void QSPI_close(QSPI_Handle handle)
{
    QSPILLD_Handle      qspilldHandle;
    QSPI_Object *obj;
    int32_t status = SystemP_SUCCESS;

    if(handle != NULL)
    {
        obj = ((QSPI_Config *)handle)->object;
        const QSPI_Attrs *attrs = ((QSPI_Config *)handle)->attrs;
        qspilldHandle = obj->qspilldHandle;

        if(true == attrs->dmaEnable)
        {
            status = QSPI_lld_deInitDma(qspilldHandle);
        }
        else
        {
            status = QSPI_lld_deInit(qspilldHandle);
        }

        /* Destruct all locks and Hwi objects */
        if(SystemP_SUCCESS == status)
        {
            SemaphoreP_destruct(&obj->lockObj);
            SemaphoreP_destruct(&obj->transferSemObj);
            HwiP_destruct(&obj->hwiObj);
            obj->isOpen = 0;
            SemaphoreP_post(&gQspiDrvObj.lockObj);
        }
    }
    return;
}

QSPI_Handle QSPI_getHandle(uint32_t driverInstanceIndex)
{
    QSPI_Handle         handle = NULL;
    /* Check index */
    if(driverInstanceIndex < gQspiConfigNum)
    {
        QSPI_Object *obj;
        obj = gQspiConfig[driverInstanceIndex].object;

        if(obj && (TRUE == obj->isOpen))
        {
            /* valid handle */
            handle = obj->handle;
        }
    }
    return handle;
}

uint32_t QSPI_getInputClk(QSPI_Handle handle)
{
    uint32_t retVal = 0U;

    if(handle != NULL)
    {
        const QSPI_Attrs *attrs = ((QSPI_Config *)handle)->attrs;
        retVal = attrs->inputClkFreq;
    }

    return retVal;
}

int32_t QSPI_setPreScaler(QSPI_Handle handle, uint32_t clkDividerVal)
{
    int32_t status = SystemP_SUCCESS;
    QSPILLD_Handle      qspilldHandle;

    if(handle != NULL)
    {
        QSPI_Object *obj = ((QSPI_Config *)handle)->object;
        qspilldHandle    = obj->qspilldHandle;
        status = QSPI_lld_setPreScaler(qspilldHandle,clkDividerVal);
    }
    else
    {
        status = SystemP_FAILURE;
    }
    return status;
}

int32_t QSPI_setMemAddrSpace(QSPI_Handle handle, uint32_t memMappedPortSwitch)
{
    int32_t status = SystemP_SUCCESS;
    const QSPI_Attrs *attrs = ((QSPI_Config *)handle)->attrs;
    const CSL_QspiRegs *pReg = (const CSL_QspiRegs *)attrs->baseAddr;

    CSL_REG32_FINS(&pReg->SPI_SWITCH_REG, QSPI_SPI_SWITCH_REG_MMPT_S,
                    memMappedPortSwitch);

    return status;
}

int32_t QSPI_intEnable(QSPI_Handle handle, uint32_t intFlag)
{
    int32_t status = SystemP_SUCCESS;

    if(handle != NULL)
    {
        uint32_t regVal;
        const QSPI_Attrs *attrs = ((QSPI_Config *)handle)->attrs;
        const CSL_QspiRegs *pReg = (const CSL_QspiRegs *)attrs->baseAddr;

        regVal = CSL_REG32_RD(&pReg->INTR_ENABLE_SET_REG);
        regVal |= intFlag;
        CSL_REG32_WR(&pReg->INTR_ENABLE_SET_REG, regVal);
    }
    else
    {
        status = SystemP_FAILURE;
    }
    return status;
}

int32_t QSPI_intDisable(QSPI_Handle handle, uint32_t intFlag)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t regVal;

    if(handle != NULL)
    {
        const QSPI_Attrs *attrs = ((QSPI_Config *)handle)->attrs;
        const CSL_QspiRegs *pReg = (const CSL_QspiRegs *)attrs->baseAddr;
        regVal = CSL_REG32_RD(&pReg->INTR_ENABLE_CLEAR_REG);
        regVal |= intFlag;
        CSL_REG32_WR(&pReg->INTR_ENABLE_CLEAR_REG, regVal);
    }
    else
    {
        status = SystemP_FAILURE;
    }
    return status;
}

int32_t QSPI_intClear(QSPI_Handle handle, uint32_t intFlag)
{
    int32_t status = SystemP_SUCCESS;

    if(handle != NULL)
    {
        uint32_t regVal;
        const QSPI_Attrs *attrs = ((QSPI_Config *)handle)->attrs;
        const CSL_QspiRegs *pReg = (const CSL_QspiRegs *)attrs->baseAddr;

        regVal = CSL_REG32_RD(&pReg->INTR_STATUS_ENABLED_CLEAR);
        regVal |= intFlag;
        CSL_REG32_WR(&pReg->INTR_STATUS_ENABLED_CLEAR, regVal);
    }
    else
    {
        status = SystemP_FAILURE;
    }
    return status;
}

void QSPI_transaction_init(QSPI_Transaction *trans)
{
    if( trans != NULL)
    {
        trans->count = 0U;
        trans->buf = NULL;
        trans->addrOffset = 0U;
        trans->transferTimeout = SystemP_WAIT_FOREVER;
        trans->status = QSPI_TRANSFER_STARTED;
    }
}

void QSPI_readCmdParams_init(QSPI_ReadCmdParams *rdParams)
{
    if( rdParams != NULL)
    {
        rdParams->cmd = QSPI_CMD_INVALID_OPCODE;
        rdParams->cmdAddr = QSPI_CMD_INVALID_ADDR;
        rdParams->numAddrBytes = 3;
        rdParams->rxDataBuf = NULL;
        rdParams->rxDataLen = 0;
    }
}

void QSPI_writeCmdParams_init(QSPI_WriteCmdParams *wrParams)
{
    if( wrParams != NULL)
    {
        wrParams->cmd = QSPI_CMD_INVALID_OPCODE;
        wrParams->cmdAddr = QSPI_CMD_INVALID_ADDR;
        wrParams->numAddrBytes = 3;
        wrParams->txDataBuf = NULL;
        wrParams->txDataLen = 0;
    }
}

int32_t QSPI_setWriteCmd(QSPI_Handle handle, uint8_t command)
{
    int32_t status = SystemP_SUCCESS;
    QSPILLD_Handle      qspilldHandle;

    if(handle != NULL)
    {
        QSPI_Object *obj = ((QSPI_Config *)handle)->object;
        qspilldHandle               = obj->qspilldHandle;
        qspilldHandle->writeCmd =  command;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

int32_t QSPI_setReadCmd(QSPI_Handle handle, uint8_t command)
{
    int32_t status = SystemP_SUCCESS;
    QSPILLD_Handle      qspilldHandle;

    if(handle != NULL)
    {
        QSPI_Object *obj = ((QSPI_Config *)handle)->object;
        qspilldHandle               = obj->qspilldHandle;
        qspilldHandle->readCmd =  command;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

int32_t QSPI_setAddressByteCount(QSPI_Handle handle, uint32_t count)
{
    int32_t status = SystemP_SUCCESS;
    QSPILLD_Handle      qspilldHandle;

    if(handle != NULL)
    {
        QSPI_Object *obj = ((QSPI_Config *)handle)->object;
        qspilldHandle               = obj->qspilldHandle;
        qspilldHandle->numAddrBytes =  count;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

int32_t QSPI_setDummyBitCount(QSPI_Handle handle, uint32_t count)
{
    int32_t status = SystemP_SUCCESS;
    QSPILLD_Handle      qspilldHandle;

    if(handle != NULL)
    {
        QSPI_Object *obj = ((QSPI_Config *)handle)->object;
        qspilldHandle               = obj->qspilldHandle;
        qspilldHandle->numDummyBits =  count;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

int32_t QSPI_setRxLines(QSPI_Handle handle, uint32_t rxLines)
{
    int32_t status = SystemP_SUCCESS;

    if(handle != NULL)
    {
        QSPI_Object  *object = ((QSPI_Config *)handle)->object;
        object->rxLines = rxLines;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

uint32_t QSPI_getRxLines(QSPI_Handle handle)
{
    uint32_t retVal = 0xFFFFFFFFU;
    if(handle != NULL)
    {
        QSPI_Object  *object = ((QSPI_Config *)handle)->object;
        retVal = object->rxLines;
    }
    return retVal;
}

int32_t QSPI_readMemMapMode(QSPI_Handle handle, QSPI_Transaction *trans)
{
    int32_t status = SystemP_SUCCESS;

    QSPILLD_Handle      qspilldHandle;
    uint32_t            edmaInterrupt;
    /* QSPI LLD Handle */
    if((NULL != handle) && (NULL != trans))
    {
        QSPI_Object *obj = ((QSPI_Config *)handle)->object;
        qspilldHandle               = obj->qspilldHandle;
        if(true == qspilldHandle->hQspiInit->dmaEnable)
        {
            edmaInterrupt = EDMA_isInterruptEnabled(qspilldHandle->hQspiInit->qspiDmaHandle);
            status = QSPI_lld_readDma(qspilldHandle,trans->count,trans->buf,trans->addrOffset,trans->transferTimeout);

            if(status == SystemP_SUCCESS && edmaInterrupt == TRUE && qspilldHandle->state == QSPI_STATE_BLOCK)
            {
                qspilldHandle->state = QSPI_STATE_IDLE;
                /* Pend the semaphore */
                (void) SemaphoreP_pend(&obj->transferSemObj, SystemP_WAIT_FOREVER);
            }
        }
        else
        {
            status = QSPI_lld_read(qspilldHandle,trans->count,trans->buf,trans->addrOffset,trans->transferTimeout);
        }
    }
    return status;
}

int32_t QSPI_readCmd(QSPI_Handle handle, QSPI_ReadCmdParams *rdParams)
{
    int32_t status = SystemP_SUCCESS;

    QSPILLD_Handle      qspilldHandle;
    QSPILLD_WriteCmdParams *msg = (QSPILLD_WriteCmdParams *)rdParams;
    /* QSPI LLD Handle */
    if((NULL != handle) && (NULL != rdParams))
    {
        QSPI_Object *obj = ((QSPI_Config *)handle)->object;
        qspilldHandle               = obj->qspilldHandle;
        status = QSPI_lld_readCmd(qspilldHandle,msg);
    }
    return status;
}

int32_t QSPI_writeCmd(QSPI_Handle handle, QSPI_WriteCmdParams *wrParams)
{
    int32_t status = SystemP_SUCCESS;

    QSPILLD_Handle      qspilldHandle;
    QSPILLD_WriteCmdParams *msg = (QSPILLD_WriteCmdParams *)wrParams;
    /* QSPI LLD Handle */
    if((NULL != handle) && (NULL != wrParams))
    {
        QSPI_Object *obj = ((QSPI_Config *)handle)->object;
        qspilldHandle               = obj->qspilldHandle;
        status = QSPI_lld_writeCmd(qspilldHandle,msg);
    }
    return status;
}

int32_t QSPI_writeConfigMode(QSPI_Handle handle, const QSPI_Transaction *trans)
{
    int32_t status = SystemP_SUCCESS;
    QSPI_WriteCmdParams wrParams;
    QSPILLD_Handle      qspilldHandle;
    /* QSPI LLD Handle */
    if((NULL != handle) && (NULL != trans))
    {
        QSPI_Object *obj = ((QSPI_Config *)handle)->object;
        qspilldHandle               = obj->qspilldHandle;

        wrParams.cmd = qspilldHandle->writeCmd;
        wrParams.cmdAddr = trans->addrOffset;
        wrParams.numAddrBytes = (uint8_t)qspilldHandle->numAddrBytes;
        wrParams.txDataBuf = trans->buf;
        wrParams.txDataLen = trans->count;

        status = QSPI_writeCmd(handle, &wrParams);
    }
    else
    {
        status = SystemP_FAILURE;
    }
    return status;
}

int32_t QSPI_writeConfigModeIntr(QSPI_Handle handle, QSPI_WriteCmdParams *wrParams)
{
    int32_t status = SystemP_SUCCESS;
    QSPILLD_Handle      qspilldHandle;
    QSPILLD_WriteCmdParams *trans = NULL;

    /* QSPI LLD Handle */
    if((NULL != handle) && (NULL != wrParams))
    {
        trans                       = (QSPILLD_WriteCmdParams *) wrParams;
        QSPI_Object *obj            = ((QSPI_Config *)handle)->object;
        qspilldHandle               = obj->qspilldHandle;

        /* Write Trigger */
        status = QSPI_lld_writeCmdIntr(qspilldHandle,trans);
        /* Pend the semaphore */
        (void) SemaphoreP_pend(&obj->transferSemObj, SystemP_WAIT_FOREVER);
    }
    return status;
}

int32_t QSPI_readConfigModeIntr(QSPI_Handle handle, QSPI_ReadCmdParams *rdParams)
{
    int32_t status = SystemP_SUCCESS;
    QSPILLD_Handle      qspilldHandle;
    QSPILLD_WriteCmdParams *trans = NULL;

    /* QSPI LLD Handle */
    if((NULL != handle) && (NULL != rdParams))
    {
        trans                       = (QSPILLD_WriteCmdParams *) rdParams;
        QSPI_Object *obj            = ((QSPI_Config *)handle)->object;
        qspilldHandle               = obj->qspilldHandle;

        /* Read Trigger */
        status = QSPI_lld_readCmdIntr(qspilldHandle,trans);
        /* Pend the semaphore */
        (void) SemaphoreP_pend(&obj->transferSemObj, SystemP_WAIT_FOREVER);
    }
    return status;
}

void QSPI_interruptCallback(void* args)
{
    QSPILLD_Handle handle = (QSPILLD_Handle) args;
    QSPI_Object *obj    = ((QSPI_Config *)handle->args)->object;
    (void) SemaphoreP_post(&obj->transferSemObj);
}

void OSPI_phyGetTuningData(uint32_t *tuningData, uint32_t *tuningDataSize)
{
    /* Dummy function. Turing data not supported for QSPI. */
    *tuningData = (uint32_t) NULL;
    *tuningDataSize = 0;
}