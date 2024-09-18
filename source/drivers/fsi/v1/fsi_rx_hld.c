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
 *  \file     v1/fsi_rx_hld.c
 *
 *  \brief    This file contains the implementation of the APIs present in the
 *            device abstraction layer file of FSI_RX.
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
#include <drivers/fsi/v1/fsi_rx_hld.h>
#include <drivers/fsi/v1/dma/edma/fsi_dma_edma.h>

/* Need to be removed */
#define CONFIG_FSI_RX0_CLK (500000000U)
/* FSI RXCLK - 50 MHz */
#define FSI_APP_RXCLK_FREQ              (50 * 1000 * 1000)
/* FSI module input clock - 500 MHz */
#define FSI_APP_CLK_FREQ                (CONFIG_FSI_RX0_CLK)
/* FSI RX prescaler value for RXCLKIN of 100 MHz. / 2 is provided as RXCLK = RXCLKIN/2 */
#define FSI_APP_RX_PRESCALER_VAL        (FSI_APP_CLK_FREQ / FSI_APP_RXCLK_FREQ / 2U)

#define FSI_APP_LOOP_COUNT              (100U)
/* User data to be sent with Data frame */
#define FSI_APP_RX_USER_DATA            (0x07U)
/* Configuring Frame - can be between 1-16U */
#define FSI_APP_FRAME_DATA_WORD_SIZE    (16U)
/* 0x0U for 1 lane and 0x1U for two lane */
#define FSI_APP_N_LANES                 (0x0U)
#define FSI_APP_RX_DATA_FRAME_TAG       (0x1U)

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

static int32_t FSI_Rx_configInstance(FSI_Rx_Handle hFsiRx);
static int32_t FSI_Rx_deConfigInstance(FSI_Rx_Handle hFsiRx);

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
} FSI_Rx_DrvObj;

/** \brief Driver object */
static FSI_Rx_DrvObj     gFsiRxDrvObj =
{
    .lock           = NULL,
};

extern uint32_t gFsiRxConfigNum;
extern FSI_Rx_Config gFsiRxConfig[];
extern FSI_Rx_DmaHandle gFsiRxDmaHandle[];
extern FSI_Rx_DmaChConfig gFsiRxDmaChCfg;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void FSI_Rx_init(void)
{
    int32_t       status;
    uint32_t      cnt;
    FSI_Rx_Object    *object;

    /* Init each driver instance object */
    for (cnt = 0U; cnt < gFsiRxConfigNum; cnt++)
    {
        /* initialize object varibles */
        object = gFsiRxConfig[cnt].object;
        DebugP_assert(NULL_PTR != object);
        (void)memset(object, 0, sizeof(FSI_Rx_Object));
        gFsiRxConfig[cnt].attrs->baseAddr = (uint32_t) AddrTranslateP_getLocalAddr((uint64_t)gFsiRxConfig[cnt].attrs->baseAddr);
    }

    /* Create driver lock */
    status = SemaphoreP_constructMutex(&gFsiRxDrvObj.lockObj);
    if(SystemP_SUCCESS == status)
    {
        gFsiRxDrvObj.lock = &gFsiRxDrvObj.lockObj;
    }

    return;
}

void FSI_Rx_deinit(void)
{
    /* Delete driver lock */
    if(NULL != gFsiRxDrvObj.lock)
    {
        SemaphoreP_destruct(&gFsiRxDrvObj.lockObj);
        gFsiRxDrvObj.lock = NULL;
    }

    return;
}

/*
 * Provides delay based on requested count. Needed while performing reset and
 * flush sequence, sufficient delay ensures reliability in operation.
 */
FSI_Rx_Handle FSI_Rx_open(uint32_t index, FSI_Rx_Params *prms)
{
    int32_t               status = SystemP_SUCCESS;
    FSI_Rx_Handle         handle = NULL;
    FSI_Rx_Config        *config = NULL;
    FSI_Rx_Object        *obj = NULL;
    const FSI_Rx_Attrs   *attrs;
    HwiP_Params           hwiPrms;

    /* Check index */
    if((index >= gFsiRxConfigNum) && (prms == NULL))
    {
        status = SystemP_FAILURE;
        DebugP_assert(NULL_PTR != prms);
    }
    else
    {
        config = &gFsiRxConfig[index];
    }

    DebugP_assert(NULL_PTR != gFsiRxDrvObj.lock);
    status += SemaphoreP_pend(&gFsiRxDrvObj.lockObj, SystemP_WAIT_FOREVER);

    if((SystemP_SUCCESS == status) && (config != NULL_PTR))
    {
        obj = config->object;
        DebugP_assert(NULL_PTR != obj);
        DebugP_assert(NULL_PTR != config->attrs);
        attrs = config->attrs;
        obj->params = prms;
        if(attrs->operMode == FSI_RX_OPER_MODE_DMA)
        {
            obj->fsiRxDmaHandle = (FSI_Rx_DmaHandle) gFsiRxDmaHandle[0];
            obj->fsiRxDmaChCfg  = gFsiRxDmaChCfg;
        }
        obj->handle = (FSI_Rx_Handle) config;
        handle = obj->handle;
    
        status += FSI_Rx_configInstance(handle);
        if(status == SystemP_SUCCESS)
        {
            /* Create read transfer sync semaphore */
            status += SemaphoreP_constructBinary(&obj->readTransferSemObj, 0U);
            obj->readTransferSem = &obj->readTransferSemObj;

            /* Register interrupt */
            if(FSI_RX_OPER_MODE_INTERRUPT == attrs->operMode)
            {
                HwiP_Params_init(&hwiPrms);
                hwiPrms.intNum      = attrs->intrNum;
                hwiPrms.priority    = attrs->intrPriority;
                hwiPrms.callback    = &FSI_Rx_Isr;
                hwiPrms.args        = (void *) handle;
                status += HwiP_construct(&obj->hwiObj, &hwiPrms);
                status = FSI_enableRxInterrupt(attrs->baseAddr, FSI_INT1, FSI_RX_EVT_DATA_FRAME);
            }
        }

        SemaphoreP_post(&gFsiRxDrvObj.lockObj);

        /* Free-up resources in case of error */
        if (SystemP_SUCCESS != status)
        {
            FSI_Rx_close((FSI_Rx_Handle) config);
        }
    }

    return (handle);
}

void FSI_Rx_close(FSI_Rx_Handle handle)
{
    FSI_Rx_Config        *config;
    FSI_Rx_Object        *obj;
    int32_t           status = SystemP_FAILURE;

    if(NULL != handle)
    {
        config = (FSI_Rx_Config *)handle;
        obj    = config->object;
        DebugP_assert(NULL_PTR != obj);
        DebugP_assert(NULL_PTR != config->attrs);
        DebugP_assert(NULL_PTR != gFsiRxDrvObj.lock);

        status = SemaphoreP_pend(&gFsiRxDrvObj.lockObj, SystemP_WAIT_FOREVER);
        status += FSI_Rx_deConfigInstance(handle);
        DebugP_assert(SystemP_SUCCESS == status);

        if(NULL != obj->readTransferSem)
        {
            SemaphoreP_destruct(&obj->readTransferSemObj);
            obj->readTransferSem = NULL;
        }
        if(NULL != obj->hwiHandle)
        {
            HwiP_destruct(&obj->hwiObj);
            obj->hwiHandle = NULL;
        }

        SemaphoreP_post(&gFsiRxDrvObj.lockObj);
    }

    return;
}

/**
 *  @b Description
 *  @n
 *      Function initializes the FSIRx driver instance with the specified hardware attributes.
 *      It resets and configures the FSIRX module.
 *
 *
 *  @param[in]  handle
 *      FSIRx handle.
 *  @retval
 *      Success  -   SystemP_SUCCESS
 *  @retval
 *      Failure  -   SystemP_FAILURE
 */

static int32_t FSI_Rx_configInstance(FSI_Rx_Handle handle)
{
    int32_t     status = SystemP_SUCCESS;
    uint32_t    baseAddr;
    const FSI_Rx_Attrs *attrs;
    FSI_Rx_Config      *config = NULL;
    FSI_Rx_Object      *fsiRxObj = NULL;

    if(handle != NULL)
    {
        config = (FSI_Rx_Config *)handle;
        attrs = config->attrs;
        baseAddr = attrs->baseAddr;
        fsiRxObj = config->object;

        /* Rx init and reset */
        status = FSI_performRxInitialization(baseAddr);
        status += FSI_resetRxModule(baseAddr, FSI_RX_MAIN_CORE_RESET);
        FSI_clearRxModuleReset(baseAddr, FSI_RX_MAIN_CORE_RESET);

        /*AD_Review: Need to update */
        /* Setting for requested transfer params */
        status += FSI_setRxSoftwareFrameSize(baseAddr, 16);
        status += FSI_setRxDataWidth(baseAddr, 0);

        if(attrs->operMode != FSI_RX_OPER_MODE_DMA)
        {
            /* Setting frame config */
            status += FSI_setRxBufferPtr(baseAddr, 0U);
        }
        else
        {
            status  = FSI_Rx_dmaOpen(handle, fsiRxObj->fsiRxDmaChCfg);
        }
    }

    return status;
}

static int32_t FSI_Rx_deConfigInstance(FSI_Rx_Handle handle)
{
    return 0;
}

int32_t FSI_Rx_hld(FSI_Rx_Handle handle, uint16_t *rxBufData, uint16_t *rxBufTagAndUserData,
                    uint16_t dataSize, uint16_t bufIdx)
{
    int32_t                 retVal = SystemP_FAILURE;
    const FSI_Rx_Attrs      *attrs;
    FSI_Rx_Config           *config = NULL;
    FSI_Rx_Object           *object = NULL;

    if(NULL_PTR != handle)
    {
        /* Get the pointer to the FSI Driver Block */
        config = (FSI_Rx_Config*)handle;
        attrs = config->attrs;
        object = config->object;
        if((FSI_RX_OPER_MODE_INTERRUPT == attrs->operMode) ||
            (FSI_RX_OPER_MODE_DMA == attrs->operMode))
        {
            if(FSI_RX_OPER_MODE_INTERRUPT == attrs->operMode)
            {
                retVal = FSI_Rx_Intr(handle, rxBufData, NULL, dataSize, bufIdx);
            }
            else
            {
                retVal = FSI_Rx_Dma(handle, rxBufData, rxBufTagAndUserData, dataSize, bufIdx);
            }
            if (retVal == SystemP_SUCCESS)
            {
                if(attrs->operMode == FSI_RX_OPER_MODE_INTERRUPT)
                {
                    if (object->params->transferMode == FSI_RX_TRANSFER_MODE_BLOCKING)
                    {
                        /* Block on transferSem till the transfer completion. */
                        DebugP_assert(NULL_PTR != object->readTransferSem);
                        retVal = SemaphoreP_pend(&object->readTransferSemObj, SystemP_WAIT_FOREVER);
                        if (retVal != SystemP_SUCCESS)
                        {
                            retVal = FSI_Rx_deConfigInstance(handle);
                        }
                    }
                }
            }
        }
        else
        {
            retVal = FSI_Rx_Poll(handle, rxBufData, NULL, dataSize, bufIdx);
        }
    }

    return retVal;
}

int32_t FSI_Rx_Poll(FSI_Rx_Handle handle, uint16_t *rxBufData, uint16_t *rxBufTagAndUserData, 
                    uint16_t dataSize, uint16_t bufIdx)
{
    int32_t     status = SystemP_SUCCESS;
    uint32_t    baseAddr = 0;
    const FSI_Rx_Attrs *attrs;
    FSI_Rx_Config *config = NULL;
    uint16_t    rxEvtSts;

    if(handle != NULL)
    {
        config = (FSI_Rx_Config *)handle;
        attrs = config->attrs;
        baseAddr = attrs->baseAddr;

        /* Wait for RX completion */
        while(1)
        {
            FSI_getRxEventStatus(baseAddr, &rxEvtSts);
            if(rxEvtSts & FSI_RX_EVT_FRAME_DONE)
            {
                FSI_clearRxEvents(baseAddr, FSI_RX_EVT_FRAME_DONE);
                break;
            }
        }

        /* Recieve data */
        status = FSI_readRxBuffer(baseAddr, rxBufData, dataSize, bufIdx);
        DebugP_assert(status == SystemP_SUCCESS);
    }

    return status;
}

int32_t FSI_Rx_Intr(FSI_Rx_Handle handle, uint16_t *rxBufData, uint16_t *rxBufTagAndUserData,
                    uint16_t dataSize, uint16_t bufIdx)
{
    int32_t     status = SystemP_SUCCESS;
    uint32_t    baseAddr = 0;
    const FSI_Rx_Attrs *attrs;
    FSI_Rx_Config *config = NULL;

    if(handle != NULL)
    {
        config = (FSI_Rx_Config *)handle;
        attrs = config->attrs;
        baseAddr = attrs->baseAddr;
        /* Recieve data */
        status = FSI_readRxBuffer(baseAddr, rxBufData, dataSize, bufIdx);
        DebugP_assert(status == SystemP_SUCCESS);
    }

    return status;
}

int32_t FSI_Rx_Dma(FSI_Rx_Handle handle, uint16_t *rxBufData, uint16_t *rxBufTagAndUserData, 
                    uint16_t dataSize, uint16_t bufIdx)
{
    int32_t     status = SystemP_SUCCESS;
    uint32_t    baseAddr, regionId, rxBufBaseAddr, rxFrameTagData;
    const FSI_Rx_Attrs *attrs;
    FSI_Rx_Config *config = NULL;
    FSI_Rx_Object *object = NULL;
    FSI_Rx_EdmaChConfig *edmaChCfg = NULL;
    uint32_t    dmaCh0, dmaCh1;
    uint32_t    param0, param1;
    uint32_t    tccAlloc0;

    if(handle != NULL)
    {
        config = (FSI_Rx_Config *)handle;
        attrs = config->attrs;
        object = config->object;
        edmaChCfg = (FSI_Rx_EdmaChConfig *)object->fsiRxDmaChCfg;
        baseAddr = attrs->baseAddr;
        rxBufBaseAddr = baseAddr + CSL_FSI_RX_CFG_RX_BUF_BASE(bufIdx);
        rxFrameTagData = baseAddr + CSL_FSI_RX_CFG_RX_FRAME_TAG_UDATA;

        regionId = edmaChCfg->edmaRegionId;
        dmaCh0   = edmaChCfg->edmaRxChId[0];
        param0 = EDMA_RESOURCE_ALLOC_ANY;
        tccAlloc0 = EDMA_RESOURCE_ALLOC_ANY;

        dmaCh1  = edmaChCfg->edmaRxChId[1];
        param1 = EDMA_RESOURCE_ALLOC_ANY;

        /* Create a semaphore to signal EDMA transfer completion */
        status = SemaphoreP_constructBinary(&gFsiDmaRxSemObject, 0);
        DebugP_assert(SystemP_SUCCESS == status);

        FSI_Rx_edmaChInit(object, 0, &param0, NULL);
        FSI_Rx_configureDma(object, &dmaCh0, (void *)rxBufBaseAddr,
                            (void *)rxBufData, NULL, &param0, regionId, sizeof(uint16_t), FSI_APP_FRAME_DATA_WORD_COUNT, FSI_APP_LOOP_COUNT,
                         sizeof(uint16_t), sizeof(uint16_t), 0U, sizeof(uint16_t) * FSI_APP_FRAME_DATA_WORD_COUNT, EDMA_TRIG_MODE_EVENT);

        FSI_Rx_edmaChInit(object, 1, &param1, &tccAlloc0);
        FSI_Rx_configureDma(object, &dmaCh1, (void *)rxFrameTagData,
                        (void *)rxBufTagAndUserData,
                        &tccAlloc0, &param1, regionId, sizeof(uint16_t), 1U, FSI_APP_LOOP_COUNT, 0U, 0U, 0U, sizeof(uint16_t), EDMA_TRIG_MODE_EVENT);

        status = FSI_Rx_edmaIntrInit(object, tccAlloc0);
        FSI_enableRxDMAEvent(baseAddr);

    }

    /* Wait for RX completion */
    //SemaphoreP_pend(&gFsiDmaRxSemObject, SystemP_WAIT_FOREVER);

    return status;
}

void FSI_Rx_Isr(void* args)
{
    const FSI_Rx_Attrs *attrs;
    FSI_Rx_Config *config = NULL;
    FSI_Rx_Object *object = NULL;
    uint32_t baseAddr = 0;

    if(args != NULL)
    {
        config = (FSI_Rx_Config *)args;
        attrs = config->attrs;
        baseAddr = attrs->baseAddr;
        object = config->object;

        FSI_clearRxEvents(baseAddr, FSI_RX_EVT_FRAME_DONE);
        SemaphoreP_post(&object->readTransferSemObj);
    }
    return;
}

void FSI_Rx_pendDmaCompletion()
{
    SemaphoreP_pend(&gFsiDmaRxSemObject, SystemP_WAIT_FOREVER);
}

void FSI_Rx_DmaCompletionCallback(void *args)
{
    SemaphoreP_post(&gFsiDmaRxSemObject);
}
