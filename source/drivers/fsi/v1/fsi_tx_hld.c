/*
 * Copyright (C) 2024 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 *  \file     v1/fsi_tx_hld.c
 *
 *  \brief    This file contains the implementation of the APIs present in the
 *            device abstraction layer file of FSI_TX.
 *            This also contains some related macros.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <string.h>
#include <kernel/dpl/AddrTranslateP.h>
#include <drivers/hw_include/csl_types.h>
#include <drivers/hw_include/hw_types.h>
#include <drivers/fsi.h>
#include <drivers/fsi/v1/fsi_tx_hld.h>
#include <drivers/fsi/v1/dma/edma/fsi_dma_edma.h>

/* Need to be removed */
#define CONFIG_FSI_TX0_CLK (500000000U)
/* FSI TXCLK - 50 MHz */
#define FSI_APP_TXCLK_FREQ              (50 * 1000 * 1000)
/* FSI module input clock - 500 MHz */
#define FSI_APP_CLK_FREQ                (CONFIG_FSI_TX0_CLK)
/* FSI TX prescaler value for TXCLKIN of 100 MHz. / 2 is provided as TXCLK = TXCLKIN/2 */
#define FSI_APP_TX_PRESCALER_VAL        (FSI_APP_CLK_FREQ / FSI_APP_TXCLK_FREQ / 2U)

#define FSI_APP_LOOP_COUNT              (100U)
/* User data to be sent with Data frame */
#define FSI_APP_TX_USER_DATA            (0x07U)
/* Configuring Frame - can be between 1-16U */
#define FSI_APP_FRAME_DATA_WORD_SIZE    (16U)
/* 0x0U for 1 lane and 0x1U for two lane */
#define FSI_APP_N_LANES                 (0x0U)
#define FSI_APP_TX_DATA_FRAME_TAG       (0x1U)


/* AD_Review : Need to remove the following macros */
/* Configuring Frame - can be between 1-16U */
#define FSI_APP_FRAME_DATA_WORD_COUNT   (16U)

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

static int32_t FSI_Tx_configInstance(FSI_Tx_Handle hFsiTx);
static int32_t FSI_Tx_deConfigInstance(FSI_Tx_Handle hFsiTx);

/* None */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

typedef struct
{
    void                   *lock;
    /**< Driver lock - to protect across open/close */
    SemaphoreP_Object       lockObj;
    /**< Driver lock object */
} FSI_Tx_DrvObj;

/** \brief Driver object */
static FSI_Tx_DrvObj     gFsiTxDrvObj =
{
    .lock           = NULL,
};

extern uint32_t gFsiTxConfigNum;
extern FSI_Tx_Config gFsiTxConfig[];
extern FSI_Tx_DmaHandle gFsiTxDmaHandle[];
extern FSI_Tx_DmaChConfig gFsiTxDmaChCfg;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void FSI_Tx_init(void)
{
    int32_t       status;
    uint32_t      cnt;
    FSI_Tx_Object    *object;

    /* Init each driver instance object */
    for (cnt = 0U; cnt < gFsiTxConfigNum; cnt++)
    {
        /* initialize object varibles */
        object = gFsiTxConfig[cnt].object;
        DebugP_assert(NULL_PTR != object);
        (void)memset(object, 0, sizeof(FSI_Tx_Object));
        gFsiTxConfig[cnt].attrs->baseAddr = (uint32_t) AddrTranslateP_getLocalAddr((uint64_t)gFsiTxConfig[cnt].attrs->baseAddr);
    }

    /* Create driver lock */
    status = SemaphoreP_constructMutex(&gFsiTxDrvObj.lockObj);
    if(SystemP_SUCCESS == status)
    {
        gFsiTxDrvObj.lock = &gFsiTxDrvObj.lockObj;
    }

    return;
}

void FSI_Tx_deinit(void)
{
    /* Delete driver lock */
    if(NULL != gFsiTxDrvObj.lock)
    {
        SemaphoreP_destruct(&gFsiTxDrvObj.lockObj);
        gFsiTxDrvObj.lock = NULL;
    }

    return;
}

/*
 * Provides delay based on requested count. Needed while performing reset and
 * flush sequence, sufficient delay ensures reliability in operation.
 */
FSI_Tx_Handle FSI_Tx_open(uint32_t index, FSI_Tx_Params *prms)
{
    int32_t               status = SystemP_SUCCESS;
    FSI_Tx_Handle         handle = NULL;
    FSI_Tx_Config        *config = NULL;
    FSI_Tx_Object        *obj = NULL;
    const FSI_Tx_Attrs   *attrs;
    HwiP_Params           hwiPrms;

    /* Check index */
    if((index >= gFsiTxConfigNum) && (prms == NULL))
    {
        status = SystemP_FAILURE;
        DebugP_assert(NULL_PTR != prms);
    }
    else
    {
        config = &gFsiTxConfig[index];
    }

    DebugP_assert(NULL_PTR != gFsiTxDrvObj.lock);
    status += SemaphoreP_pend(&gFsiTxDrvObj.lockObj, SystemP_WAIT_FOREVER);

    if((SystemP_SUCCESS == status) && (config != NULL_PTR))
    {
        obj = config->object;
        DebugP_assert(NULL_PTR != obj);
        DebugP_assert(NULL_PTR != config->attrs);
        attrs = config->attrs;
        obj->params = prms;
        if(attrs->operMode == FSI_TX_OPER_MODE_DMA)
        {
            obj->fsiTxDmaHandle = (FSI_Tx_DmaHandle) gFsiTxDmaHandle[0];
            obj->fsiTxDmaChCfg  = gFsiTxDmaChCfg;
        }
        obj->handle = (FSI_Tx_Handle) config;
        handle = obj->handle;
    
        status += FSI_Tx_configInstance(handle);
        if(status == SystemP_SUCCESS)
        {
            /* Create write transfer sync semaphore */
            status += SemaphoreP_constructBinary(&obj->writeTransferSemObj, 0U);
            obj->writeTransferSem = &obj->writeTransferSemObj;

            /* Register interrupt */
            if(FSI_TX_OPER_MODE_INTERRUPT == attrs->operMode)
            {
                HwiP_Params_init(&hwiPrms);
                hwiPrms.intNum      = attrs->intrNum;
                hwiPrms.priority    = attrs->intrPriority;
                hwiPrms.callback    = &FSI_Tx_Isr;
                hwiPrms.args        = (void *) handle;
                status += HwiP_construct(&obj->hwiObj, &hwiPrms);
                status += FSI_enableTxInterrupt(attrs->baseAddr, FSI_INT1, FSI_TX_EVT_FRAME_DONE);
            }
        }

        SemaphoreP_post(&gFsiTxDrvObj.lockObj);

        /* Free-up resources in case of error */
        if (SystemP_SUCCESS != status)
        {
            FSI_Tx_close((FSI_Tx_Handle) config);
        }
    }

    return (handle);
}

void FSI_Tx_close(FSI_Tx_Handle handle)
{
    FSI_Tx_Config        *config;
    FSI_Tx_Object        *obj;
    int32_t           status = SystemP_FAILURE;

    if(NULL != handle)
    {
        config = (FSI_Tx_Config *)handle;
        obj    = config->object;
        DebugP_assert(NULL_PTR != obj);
        DebugP_assert(NULL_PTR != config->attrs);
        DebugP_assert(NULL_PTR != gFsiTxDrvObj.lock);

        status = SemaphoreP_pend(&gFsiTxDrvObj.lockObj, SystemP_WAIT_FOREVER);
        status += FSI_Tx_deConfigInstance(handle);
        DebugP_assert(SystemP_SUCCESS == status);

        if(NULL != obj->writeTransferSem)
        {
            SemaphoreP_destruct(&obj->writeTransferSemObj);
            obj->writeTransferSem = NULL;
        }
        if(NULL != obj->hwiHandle)
        {
            HwiP_destruct(&obj->hwiObj);
            obj->hwiHandle = NULL;
        }

        SemaphoreP_post(&gFsiTxDrvObj.lockObj);
    }

    return;
}

/**
 *  @b Description
 *  @n
 *      Function initializes the FSITx driver instance with the specified hardware attributes.
 *      It resets and configures the FSITX module.
 *
 *
 *  @param[in]  handle
 *      FSITx handle.
 *  @retval
 *      Success  -   SystemP_SUCCESS
 *  @retval
 *      Failure  -   SystemP_FAILURE
 */

static int32_t FSI_Tx_configInstance(FSI_Tx_Handle handle)
{
    int32_t     status = SystemP_SUCCESS;
    uint32_t    baseAddr;
    const FSI_Tx_Attrs *attrs;
    FSI_Tx_Config      *config = NULL;
    FSI_Tx_Object      *fsiTxObj = NULL;

    if(handle != NULL)
    {
        config = (FSI_Tx_Config *)handle;
        attrs = config->attrs;
        baseAddr = attrs->baseAddr;
        fsiTxObj = config->object;

        /* TX init and reset */
        status = FSI_performTxInitialization(baseAddr, FSI_APP_TX_PRESCALER_VAL);
        status += FSI_resetTxModule(baseAddr, FSI_TX_MAIN_CORE_RESET);
        FSI_clearTxModuleReset(baseAddr, FSI_TX_MAIN_CORE_RESET);

        /* Setting for requested transfer params */
        status += FSI_setTxSoftwareFrameSize(baseAddr, 16);
        status += FSI_setTxDataWidth(baseAddr, 0);

        if(attrs->operMode != FSI_TX_OPER_MODE_DMA)
        {
            /*In case of NON-DMA transmission */
            /* Setting frame config */
            status += FSI_setTxUserDefinedData(baseAddr, FSI_APP_TX_USER_DATA);
            status += FSI_setTxFrameTag(baseAddr, FSI_APP_TX_DATA_FRAME_TAG);
        }

        status += FSI_setTxFrameType(baseAddr, FSI_FRAME_TYPE_NWORD_DATA);

        /* Initialize dma mode if the dma handle is not NULL */
        if (status == SystemP_SUCCESS)
        {
            if (attrs->operMode == FSI_TX_OPER_MODE_DMA)
            {
                status  = FSI_Tx_dmaOpen(handle, fsiTxObj->fsiTxDmaChCfg);
            }
        }
    }

    return status;
}

static int32_t FSI_Tx_deConfigInstance(FSI_Tx_Handle handle)
{
    int32_t                 status = SystemP_FAILURE;
    const FSI_Tx_Attrs      *attrs;
    FSI_Tx_Config           *config = NULL;
    FSI_Tx_Object           *fsiTxObj = NULL;

    if(NULL_PTR != handle)
    {
        /* Get the pointer to the FSI-TX Driver Block */
        config = (FSI_Tx_Config*)handle;
        attrs = config->attrs;
        fsiTxObj = config->object;
        if(FSI_TX_OPER_MODE_DMA == attrs->operMode)
        {
           status = FSI_Tx_dmaClose(handle, fsiTxObj->fsiTxDmaChCfg);
        }
    }

    return status;
}

int32_t FSI_Tx_hld(FSI_Tx_Handle handle, uint16_t *txBufData, uint16_t *txBufTagAndUserData, 
                    uint16_t dataSize, uint16_t bufIdx)
{
    int32_t                 retVal = SystemP_FAILURE;
    const FSI_Tx_Attrs      *attrs;
    FSI_Tx_Config           *config = NULL;
    FSI_Tx_Object           *object = NULL;

    if(NULL_PTR != handle)
    {
        /* Get the pointer to the CAN Driver Block */
        config = (FSI_Tx_Config*)handle;
        attrs = config->attrs;
        object = config->object;
        if((FSI_TX_OPER_MODE_INTERRUPT == attrs->operMode) ||
            (FSI_TX_OPER_MODE_DMA == attrs->operMode))
        {
            if(FSI_TX_OPER_MODE_INTERRUPT == attrs->operMode)
            {
                retVal = FSI_Tx_Intr(handle, txBufData, NULL, dataSize, bufIdx);
            }
            else
            {
                retVal = FSI_Tx_Dma(handle, txBufData, txBufTagAndUserData, dataSize, bufIdx);
            }
            if (retVal == SystemP_SUCCESS)
            {
                if(attrs->operMode == FSI_TX_OPER_MODE_INTERRUPT)
                {
                    if (object->params->transferMode == FSI_TX_TRANSFER_MODE_BLOCKING)
                    {
                        /* Block on transferSem till the transfer completion. */
                        DebugP_assert(NULL_PTR != object->writeTransferSem);
                        retVal = SemaphoreP_pend(&object->writeTransferSemObj, SystemP_WAIT_FOREVER);
                        if (retVal != SystemP_SUCCESS)
                        {
                            retVal = FSI_Tx_deConfigInstance(handle);
                        }
                    }
                }
            }
        }
        else
        {
            retVal = FSI_Tx_Poll(handle, txBufData, NULL, dataSize, bufIdx);
        }
    }

    return retVal;
}

int32_t FSI_Tx_Poll(FSI_Tx_Handle handle, uint16_t *txBufData, uint16_t *txBufTagAndUserData,
                    uint16_t dataSize, uint16_t bufIdx)
{
    int32_t     status = SystemP_SUCCESS;
    uint32_t    baseAddr = 0;
    const FSI_Tx_Attrs *attrs;
    FSI_Tx_Config *config = NULL;
    uint16_t    txEvtSts;

    if(handle != NULL)
    {
        config = (FSI_Tx_Config *)handle;
        attrs = config->attrs;
        baseAddr = attrs->baseAddr;
        /* Transmit data */
        status = FSI_setTxBufferPtr(baseAddr, bufIdx);
        status += FSI_writeTxBuffer(baseAddr, txBufData, dataSize, bufIdx);
        status += FSI_startTxTransmit(baseAddr);
        DebugP_assert(status == SystemP_SUCCESS);
    }
    /* Wait for TX completion */
    while(1)
    {
        FSI_getTxEventStatus(baseAddr, &txEvtSts);
        if(txEvtSts & FSI_TX_EVT_FRAME_DONE)
        {
            FSI_clearTxEvents(baseAddr, FSI_TX_EVT_FRAME_DONE);
            break;
        }
    }
	return status;
}

int32_t FSI_Tx_Intr(FSI_Tx_Handle handle, uint16_t *txBufData, uint16_t *txBufTagAndUserData,
                    uint16_t dataSize, uint16_t bufIdx)
{
    int32_t     status = SystemP_SUCCESS;
    uint32_t    baseAddr = 0;
    const FSI_Tx_Attrs *attrs;
    FSI_Tx_Config *config = NULL;

    if(handle != NULL)
    {
        config = (FSI_Tx_Config *)handle;
        attrs = config->attrs;
        baseAddr = attrs->baseAddr;
        /* Transmit data */
        status = FSI_setTxBufferPtr(baseAddr, bufIdx);
        status += FSI_writeTxBuffer(baseAddr, txBufData, dataSize, bufIdx);
        status += FSI_startTxTransmit(baseAddr);
        DebugP_assert(status == SystemP_SUCCESS);
    }

    return status;
}

int32_t FSI_Tx_Dma(FSI_Tx_Handle handle, uint16_t *txBufData, uint16_t *txBufTagAndUserData, 
                    uint16_t dataSize, uint16_t bufIdx)
{
    int32_t     status = SystemP_SUCCESS;
    uint32_t    baseAddr, regionId, txBufBaseAddr, txFrameTagData;
    const FSI_Tx_Attrs *attrs;
    FSI_Tx_Config *config = NULL;
    FSI_Tx_Object *object = NULL;
    FSI_Tx_EdmaChConfig *edmaChCfg = NULL;
    uint32_t    dmaCh0, dmaCh1;
    uint32_t    param0, param1;
    uint32_t    tccTx;

    if(handle != NULL)
    {
        config = (FSI_Tx_Config *)handle;
        attrs = config->attrs;
        object = config->object;
        edmaChCfg = (FSI_Tx_EdmaChConfig *)object->fsiTxDmaChCfg;
        baseAddr = attrs->baseAddr;
        txBufBaseAddr = baseAddr + CSL_FSI_TX_CFG_TX_BUF_BASE(bufIdx);
        txFrameTagData = baseAddr + CSL_FSI_TX_CFG_TX_FRAME_TAG_UDATA;

        regionId = edmaChCfg->edmaRegionId;
        dmaCh0   = edmaChCfg->edmaTxChId[0];
        param0 = EDMA_RESOURCE_ALLOC_ANY;
        tccTx = EDMA_RESOURCE_ALLOC_ANY;

        dmaCh1  = edmaChCfg->edmaTxChId[1];
        param1 = EDMA_RESOURCE_ALLOC_ANY;

        /* Create a semaphore to signal EDMA transfer completion */
        status = SemaphoreP_constructBinary(&gFsiDmaTxSemObject, 0);
        DebugP_assert(SystemP_SUCCESS == status);
    
        if(status == SystemP_SUCCESS)
        {
            status += FSI_Tx_edmaChInit(object, 0, &param0, NULL);
            status += FSI_Tx_configureDma(object, &dmaCh0, (void *)txBufData,
                        (void *)txBufBaseAddr,
                            NULL, &param0, regionId, sizeof(uint16_t), FSI_APP_FRAME_DATA_WORD_COUNT, 1U,
                            sizeof(uint16_t), sizeof(uint16_t), 0U, 0U, EDMA_TRIG_MODE_MANUAL);

            status += FSI_Tx_edmaChInit(object, 1, &param1, &tccTx);
            status += FSI_Tx_configureDma(object, &dmaCh1, (void *)txBufTagAndUserData,
                            (void *)txFrameTagData,
                            NULL, &param1, regionId, sizeof(uint16_t), 1U, 1U, 0U, 0U, sizeof(uint16_t), 0U, EDMA_TRIG_MODE_MANUAL);

            /* Copy rest of the data with event based */
            status += FSI_Tx_configureDma(object, &dmaCh0, (void *)(txBufData + FSI_APP_FRAME_DATA_WORD_COUNT),
                        (void *)txBufBaseAddr,
                            NULL, &param0, regionId, sizeof(uint16_t), FSI_APP_FRAME_DATA_WORD_COUNT, FSI_APP_LOOP_COUNT - 1U,
                            sizeof(uint16_t), sizeof(uint16_t), sizeof(uint16_t) * FSI_APP_FRAME_DATA_WORD_COUNT, 0U, EDMA_TRIG_MODE_EVENT);
            status += FSI_Tx_configureDma(object, &dmaCh1, (void *)&txBufTagAndUserData[1U],
                            (void *)txFrameTagData,
                            &tccTx, &param1, regionId, sizeof(uint16_t), 1U, FSI_APP_LOOP_COUNT - 1U, 0U, 0U, sizeof(uint16_t), 0U, EDMA_TRIG_MODE_EVENT);
        }

        status = FSI_Tx_edmaIntrInit(object, tccTx);

        if(status == SystemP_SUCCESS)
        {
            /* Restore the structure parameters */
            edmaChCfg->edmaTxChId[0] = dmaCh0;
            edmaChCfg->edmaTxChId[1] = dmaCh1;
            edmaChCfg->edmaTxParam[0] = param0;
            edmaChCfg->edmaTxParam[1] = param1;
            edmaChCfg->edmaTccTx = tccTx;

            /* Transmit data */
            /* Send Flush Sequence to sync, after every rx soft reset */
            status = FSI_executeTxFlushSequence(baseAddr, FSI_APP_TX_PRESCALER_VAL);
            DebugP_assert(status == SystemP_SUCCESS);
            status = FSI_setTxStartMode(baseAddr, FSI_TX_START_FRAME_CTRL_OR_UDATA_TAG);
            FSI_enableTxDMAEvent(baseAddr);
        }
    }

       /* Wait for TX completion */
   // SemaphoreP_pend(&gFsiDmaTxSemObject, SystemP_WAIT_FOREVER);

    return status;
}

void FSI_Tx_Isr(void* args)
{
    const FSI_Tx_Attrs *attrs;
    FSI_Tx_Config *config = NULL;
    FSI_Tx_Object *object = NULL;
    uint32_t baseAddr = 0;

    if(args != NULL)
    {
        config = (FSI_Tx_Config *)args;
        attrs = config->attrs;
        baseAddr = attrs->baseAddr;
        object = config->object;

        FSI_clearTxEvents(baseAddr, FSI_TX_EVT_FRAME_DONE);
        SemaphoreP_post(&object->writeTransferSemObj);
    }
    return;
}

void FSI_Tx_pendDmaCompletion()
{
    SemaphoreP_pend(&gFsiDmaTxSemObject, SystemP_WAIT_FOREVER);
}

void FSI_Tx_DmaCompletionCallback(void *args)
{
    SemaphoreP_post(&gFsiDmaTxSemObject);
}