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
 *  \file mcasp.c
 *
 *  \brief File containing MCASP Driver APIs implementation.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

/* This is needed for memset/memcpy */
#include <string.h>
#include <drivers/mcasp.h>
#include <drivers/mcasp/v0/mcasp_priv.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/CacheP.h>
#include <drivers/hw_include/cslr.h>
#include <drivers/hw_include/hw_types.h>

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

typedef struct
{
    void *openLock;
    /**<  Lock to protect MCASP open*/
    SemaphoreP_Object openLockObj;
    /**< Lock object */
} MCASP_DrvObj;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* Internal functions */
static void MCASP_tx_isr(void *args);
static void MCASP_rx_isr(void *args);
static int32_t MCASP_programInstance(MCASP_Config *config);
static int32_t MCASP_bitClearGblCtl(const MCASP_Handle handle, uint32_t bitMask);
static int32_t MCASP_bitSetGblCtl(const MCASP_Handle handle, uint32_t bitMask);
static int32_t MCASP_validateTransaction (MCASP_Transaction *txn);
static int32_t MCASP_getBufferOffset(MCASP_TransferObj *xfrObj, uint8_t serIdx, uint32_t* pOffset);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/** \brief Driver object */
static MCASP_DrvObj gMcaspDrvObj =
{
    .openLock      = NULL,
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void MCASP_init(void)
{
    int32_t status;
    uint32_t count;
    MCASP_Object *obj;

    /* Init each driver instance object */
    for(count = 0U; count < gMcaspConfigNum; count++)
    {
        /* Init object variables */
        obj = gMcaspConfig[count].object;
        DebugP_assert(NULL != obj);
        memset(obj, 0, sizeof(MCASP_Object));
    }

    /* Create the driver lock */
    status = SemaphoreP_constructMutex(&gMcaspDrvObj.openLockObj);
    if(SystemP_SUCCESS == status)
    {
        gMcaspDrvObj.openLock = &gMcaspDrvObj.openLockObj;
    }

    return;
}

void MCASP_deinit(void)
{
    /* Delete driver lock */
    if(NULL != gMcaspDrvObj.openLock)
    {
        SemaphoreP_destruct(&gMcaspDrvObj.openLockObj);
        gMcaspDrvObj.openLock = NULL;
    }

    return;
}

MCASP_Handle MCASP_open(uint32_t index, const MCASP_OpenParams *openParams)
{
    int32_t status = SystemP_SUCCESS;
    MCASP_Handle handle = NULL;
    MCASP_Config *config = NULL;
    MCASP_Object *obj = NULL;
    HwiP_Params hwiPrms;
    const MCASP_Attrs *attrs;

    /* Check for valid index */
    if(index >= gMcaspConfigNum)
    {
        status = SystemP_FAILURE;
    }
    else
    {
        config = &gMcaspConfig[index];
    }

    /* Validate input param */
    if (NULL == openParams)
    {
        status = SystemP_FAILURE;
    }

    /* Protect this region from a concurrent MCASP_open */
    DebugP_assert(NULL != gMcaspDrvObj.openLock);
    SemaphoreP_pend(&gMcaspDrvObj.openLockObj, SystemP_WAIT_FOREVER);

    if(SystemP_SUCCESS == status)
    {
        obj = config->object;
        DebugP_assert(NULL != obj);
        DebugP_assert(NULL != config->attrs);
        attrs = config->attrs;
        if(TRUE == obj->isOpen)
        {
            /* Handle already opened */
            status = SystemP_FAILURE;
        }
    }

    if(SystemP_SUCCESS == status)
    {
        obj->handle = (MCASP_Handle)config;

        /* Create Queues to hold the application buffers. */
        obj->reqQueueHandleTx = QueueP_create(&obj->reqQueueObjTx);
        obj->curentQueueHandleTx = QueueP_create(&obj->curentQueueObjTx);
        obj->reqQueueHandleRx = QueueP_create(&obj->reqQueueObjRx);
        obj->curentQueueHandleRx = QueueP_create(&obj->curentQueueObjRx);

        /* Program MCASP instance according the user config */
        status += MCASP_programInstance(config);

        /* Create instance lock */
        status += SemaphoreP_constructMutex(&obj->lockObj);

        /* Create transfer sync semaphore */
        status += SemaphoreP_constructBinary(&obj->transferSemObj, 0U);

        /* Load driver transfer mode */
        obj->transferMode = openParams->transferMode;

        /* Load transfer configurations */
        obj->XmtObj.serCount = openParams->txSerUsedCount;
        obj->RcvObj.serCount = openParams->rxSerUsedCount;
        obj->XmtObj.serArray = openParams->txSerUsedArray;
        obj->RcvObj.serArray = openParams->rxSerUsedArray;
        obj->XmtObj.slotCount = openParams->txSlotCount;
        obj->RcvObj.slotCount = openParams->rxSlotCount;
        obj->XmtObj.bufferFormat = openParams->txBufferFormat;
        obj->RcvObj.bufferFormat = openParams->rxBufferFormat;
        obj->XmtObj.cbFxn = openParams->txCallbackFxn;
        obj->RcvObj.cbFxn = openParams->rxCallbackFxn;
        obj->XmtObj.loopjobEnable = openParams->txLoopjobEnable;
        obj->XmtObj.txnLoopjob.buf = openParams->txLoopjobBuf;
        obj->XmtObj.txnLoopjob.count = openParams->txLoopjobBufLength;
        obj->RcvObj.loopjobEnable = openParams->rxLoopjobEnable;
        obj->RcvObj.txnLoopjob.buf = openParams->rxLoopjobBuf;
        obj->RcvObj.txnLoopjob.count = openParams->rxLoopjobBufLength;

        /* Register interrupt */
        if(obj->transferMode != MCASP_TRANSFER_MODE_POLLING)
        {
            /* Transmit section */
            HwiP_Params_init(&hwiPrms);
            hwiPrms.intNum      = attrs->intCfgTx.intrNum;
            hwiPrms.callback    = &MCASP_tx_isr;
            hwiPrms.priority    = attrs->intCfgTx.intrPriority;
            hwiPrms.args        = (void *) config;
            status += HwiP_construct(&obj->hwiObjTx, &hwiPrms);

            /* Receive section */
            HwiP_Params_init(&hwiPrms);
            hwiPrms.intNum      = attrs->intCfgRx.intrNum;
            hwiPrms.callback    = &MCASP_rx_isr;
            hwiPrms.priority    = attrs->intCfgRx.intrPriority;
            hwiPrms.args        = (void *) config;
            status += HwiP_construct(&obj->hwiObjRx, &hwiPrms);
        }
        /* Register interrupt */
        if(obj->transferMode == MCASP_TRANSFER_MODE_DMA)
        {
            obj->edmaInst = openParams->edmaInst;
            MCASP_openDma(config, openParams->edmaInst);
        }
    }

    if(SystemP_SUCCESS == status)
    {
        obj->isOpen = 1;
        handle = (MCASP_Handle) config;
    }

    SemaphoreP_post(&gMcaspDrvObj.openLockObj);

    /* Free up resources in case of error */
    if(SystemP_SUCCESS != status)
    {
        if(NULL != config)
        {
            MCASP_close((MCASP_Handle) config);
        }
    }
    return handle;
}

void MCASP_close(MCASP_Handle handle)
{
    if(handle != NULL)
    {
        MCASP_Object *obj = ((MCASP_Config *)handle)->object;

        /* Protect this region from a concurrent MCASP_close */
        DebugP_assert(NULL != gMcaspDrvObj.openLock);
        SemaphoreP_pend(&gMcaspDrvObj.openLockObj, SystemP_WAIT_FOREVER);

        /* Destruct all locks and Hwi objects */
        SemaphoreP_destruct(&obj->lockObj);
        SemaphoreP_destruct(&obj->transferSemObj);
        if(MCASP_TRANSFER_MODE_POLLING != obj->transferMode)
        {
            HwiP_destruct(&obj->hwiObjTx);
            HwiP_destruct(&obj->hwiObjRx);
        }
        /* Register interrupt */
        if(obj->transferMode == MCASP_TRANSFER_MODE_DMA)
        {
            MCASP_closeDma((MCASP_Config *)handle, obj->edmaInst);
        }
        obj->isOpen = 0;
        SemaphoreP_post(&gMcaspDrvObj.openLockObj);
    }

    return;
}

MCASP_Handle MCASP_getHandle(uint32_t index)
{
    MCASP_Handle handle = NULL;
    /* Check index */
    if(index < gMcaspConfigNum)
    {
        MCASP_Object *obj;
        obj = gMcaspConfig[index].object;

        if(obj && (TRUE == obj->isOpen))
        {
            /* valid handle */
            handle = obj->handle;
        }
    }
    return handle;
}

static int32_t MCASP_programInstance(MCASP_Config *config)
{
    int32_t status = SystemP_SUCCESS;
    const MCASP_Attrs *attrs = config->attrs;
    MCASP_Handle handle = (MCASP_Handle) config;
    int16_t serNum = 0;

    const CSL_McaspRegs *pReg = (const CSL_McaspRegs *)attrs->baseAddr;

    /* Reset McASP to default values by setting GBLCTL = 0 */
    status += MCASP_bitClearGblCtl(handle, (uint32_t) 0xFFFFU);

    /* Set the idle mode value */
    CSL_REG32_FINS(&pReg->PWRIDLESYSCONFIG, MCASP_PWRIDLESYSCONFIG_IDLEMODE,
                   CSL_MCASP_PWRIDLESYSCONFIG_IDLEMODE_NOIDLE);

    /* FIFO settings */
    CSL_REG32_WR(&pReg->WFIFOCTL, (attrs->hwCfg.tx.fifoCfg.fifoCtl & 0xFFFF));
    CSL_REG32_WR(&pReg->RFIFOCTL, (attrs->hwCfg.rx.fifoCfg.fifoCtl & 0xFFFF));
    /* FIFO enable */
    CSL_REG32_FINS(&pReg->WFIFOCTL, MCASP_WFIFOCTL_WENA, 1);
    CSL_REG32_FINS(&pReg->RFIFOCTL, MCASP_RFIFOCTL_RENA, 1);

    /* ----- Receive settings ----- */
    /* Configure the Receive format mask */
    CSL_REG32_WR(&pReg->RMASK, attrs->hwCfg.rx.mask);

    /* Configure Receive format */
    CSL_REG32_WR(&pReg->RFMT, attrs->hwCfg.rx.fmt);

    /* Configure frame sync */
    CSL_REG32_WR(&pReg->AFSRCTL, attrs->hwCfg.rx.frSyncCtl);

    /* Configure receive clock settings */
    CSL_REG32_WR(&pReg->AHCLKRCTL, attrs->hwCfg.rx.clk.hiClk);
    CSL_REG32_WR(&pReg->ACLKRCTL, attrs->hwCfg.rx.clk.aClk);

    /* Configure receive TDM time slot */
    CSL_REG32_WR(&pReg->RTDM, attrs->hwCfg.rx.tdm);

    /* Configure clock check */
    CSL_REG32_WR(&pReg->RCLKCHK, attrs->hwCfg.rx.clk.clkChk);

    /* ----- Transmit settings ----- */
    /* Configure the Transmit format mask */
    CSL_REG32_WR(&pReg->XMASK, attrs->hwCfg.tx.mask);

    /* Configure Transmit format */
    CSL_REG32_WR(&pReg->XFMT, attrs->hwCfg.tx.fmt);

    /* Configure frame sync */
    CSL_REG32_WR(&pReg->AFSXCTL, attrs->hwCfg.tx.frSyncCtl);

    /* Configure transmit clock settings */
    CSL_REG32_WR(&pReg->AHCLKXCTL, attrs->hwCfg.tx.clk.hiClk);
    CSL_REG32_WR(&pReg->ACLKXCTL, attrs->hwCfg.tx.clk.aClk);

    /* Configure Transmit TDM time slot */
    CSL_REG32_WR(&pReg->XTDM, attrs->hwCfg.tx.tdm);

    /* Configure clock check */
    CSL_REG32_WR(&pReg->XCLKCHK, attrs->hwCfg.tx.clk.clkChk);

    // Configure serializers
    while (serNum < attrs->numOfSerializers)
    {
        CSL_REG32_WR(&pReg->SRCTL0 + (serNum),
                     attrs->hwCfg.gbl.serSetup[serNum]);
        serNum++;
    }

    /* Configure pin function register */
    CSL_REG32_WR(&pReg->PFUNC, attrs->hwCfg.gbl.pfunc);

    /* Configure pin direction register */
    CSL_REG32_WR(&pReg->PDIR, attrs->hwCfg.gbl.pdir);

    /* Configure TDM mode (disable DIT) */
    CSL_REG32_WR(&pReg->DITCTL, attrs->hwCfg.gbl.ditCtl);

    /* Configure loopback */
    CSL_REG32_WR(&pReg->DLBCTL, attrs->hwCfg.gbl.dlbCtl);

    /* Configure amute */
    CSL_REG32_WR(&pReg->AMUTE, attrs->hwCfg.gbl.amute);

    /* Initialize the global control register */
    /* start high speed clocks first */
    status += MCASP_bitSetGblCtl(handle, (uint32_t) 0x202U);
    /* start serial clocks next */
    status += MCASP_bitSetGblCtl(handle, (uint32_t) 0x101U);

    /* Configure trasnmit and receive status */
    CSL_REG32_WR(&pReg->RSTAT, attrs->hwCfg.rx.stat);
    CSL_REG32_WR(&pReg->XSTAT, attrs->hwCfg.tx.stat);

    /* Configure REVTCTL and XEVTCTL */
    CSL_REG32_WR(&pReg->REVTCTL, attrs->hwCfg.tx.evtCtl);
    CSL_REG32_WR(&pReg->XEVTCTL, attrs->hwCfg.tx.evtCtl);

    /* clear the clk fail bit in status reg*/
    CSL_REG32_FINS(&pReg->XSTAT, MCASP_XSTAT_XCKFAIL,
                            CSL_MCASP_XSTAT_XCKFAIL_YES);
    CSL_REG32_FINS(&pReg->RSTAT, MCASP_RSTAT_RCKFAIL,
                            CSL_MCASP_RSTAT_RCKFAIL_YES);

    return status;
}

static int32_t MCASP_validateTransaction (MCASP_Transaction *txn)
{
    int32_t status = SystemP_SUCCESS;
    if ((txn->buf == NULL) || (txn->count == 0))
    {
        status = SystemP_FAILURE;
    }
    return status;
}

int32_t MCASP_submitTx(MCASP_Handle handle, MCASP_Transaction *txn)
{
    int32_t status = SystemP_SUCCESS;
    if ((NULL == handle) || (NULL == txn))
    {
        status = SystemP_FAILURE;
    }
    if (status == SystemP_SUCCESS)
    {
        status = MCASP_validateTransaction(txn);
    }
    if (status == SystemP_SUCCESS)
    {
        MCASP_Object *object = ((MCASP_Config *)handle)->object;
        QueueP_put(object->reqQueueHandleTx, txn);
    }
    return status;
}

int32_t MCASP_submitRx(MCASP_Handle handle, MCASP_Transaction *txn)
{
    int32_t status = SystemP_SUCCESS;
    if ((NULL == handle) || (NULL == txn))
    {
        status = SystemP_FAILURE;
    }
    if (status == SystemP_SUCCESS)
    {
        status = MCASP_validateTransaction(txn);
    }
    if (status == SystemP_SUCCESS)
    {
        MCASP_Object *object = ((MCASP_Config *)handle)->object;
        QueueP_put(object->reqQueueHandleRx, txn);
    }
    return status;
}

MCASP_Transaction* MCASP_withdrawTx(MCASP_Handle handle)
{

    int32_t status = SystemP_SUCCESS;
    MCASP_Object *object;
    MCASP_Transaction *txn = NULL;
    if (NULL == handle)
    {
        status = SystemP_FAILURE;
    }
    if (status == SystemP_SUCCESS)
    {
        object = ((MCASP_Config *)handle)->object;
        if (NULL == object)
        {
            status = SystemP_FAILURE;
        }
    }
    if ((status == SystemP_SUCCESS) && (object->isTxStarted == 0))
    {
        if (QueueP_isEmpty(object->curentQueueHandleTx) == QueueP_NOTEMPTY)
        {
            txn = QueueP_get(object->curentQueueHandleTx);
        }
        else if (QueueP_isEmpty(object->reqQueueHandleTx) == QueueP_NOTEMPTY)
        {
            txn = QueueP_get(object->reqQueueHandleTx);
        }
        else
        {
            /* Both Queues are empty. return NULL. */
            txn = NULL;
        }
    }
    return txn;
}

MCASP_Transaction* MCASP_withdrawRx(MCASP_Handle handle)
{

    int32_t status = SystemP_SUCCESS;
    MCASP_Object *object;
    MCASP_Transaction *txn = NULL;
    if (NULL == handle)
    {
        status = SystemP_FAILURE;
    }
    if (status == SystemP_SUCCESS)
    {
        object = ((MCASP_Config *)handle)->object;
        if (NULL == object)
        {
            status = SystemP_FAILURE;
        }
    }
    if ((status == SystemP_SUCCESS) && (object->isRxStarted == 0))
    {
        if (QueueP_isEmpty(object->curentQueueHandleRx) == QueueP_NOTEMPTY)
        {
            txn = QueueP_get(object->curentQueueHandleRx);
        }
        else if (QueueP_isEmpty(object->reqQueueHandleRx) == QueueP_NOTEMPTY)
        {
            txn = QueueP_get(object->reqQueueHandleRx);
        }
        else
        {
            /* Both Queues are empty. return NULL. */
            txn = NULL;
        }
    }
    return txn;
}

int32_t MCASP_startTransferTx(MCASP_Handle handle)
{
    int32_t status = SystemP_SUCCESS;
    const MCASP_Attrs *attrs = NULL;
    const CSL_McaspRegs *pReg = NULL;
    MCASP_Object *object = NULL;
    uint32_t regVal;
    int32_t timeout = MCASP_DATA_TIMEOUT;

    if (NULL == handle)
    {
        status = SystemP_FAILURE;
    }
    if (status == SystemP_SUCCESS)
    {
        attrs = ((MCASP_Config *)handle)->attrs;
        if (NULL != attrs)
        {
            pReg = (const CSL_McaspRegs *)attrs->baseAddr;
            object = ((MCASP_Config *)handle)->object;
        }
        if ((NULL == pReg) || (NULL == object))
        {
            status = SystemP_FAILURE;
        }
    }
    if (status == SystemP_SUCCESS)
    {
        object->isTxStarted = 1;
        // case MCASP_TRANSMIT_STATE_TX_RESET:
        if(object->transferMode == MCASP_TRANSFER_MODE_DMA)
        {
            /* Enable DMA requests generation */
            CSL_REG32_FINS(&pReg->XEVTCTL, MCASP_XEVTCTL_XDATDMA,
                CSL_MCASP_XEVTCTL_XDATDMA_ENABLE);
            status = MCASP_enableDmaTx((MCASP_Config *)handle);
            if (status != SystemP_SUCCESS)
            {
                /* Disable DMA requests generation */
                CSL_REG32_FINS(&pReg->XEVTCTL, MCASP_XEVTCTL_XDATDMA,
                    CSL_MCASP_XEVTCTL_XDATDMA_RSV);
            }
            /* Disable the data ready event transmit interrupt */
            CSL_REG32_FINS(&pReg->XINTCTL, MCASP_XINTCTL_XDATA,
                CSL_MCASP_XINTCTL_XDATA_DISABLE);
        }
        else if(object->transferMode == MCASP_TRANSFER_MODE_INTERRUPT)
        {
            /* Get the first buffer queued and store in XmtObj to transfer from ISR. */
            MCASP_Transaction *txn = QueueP_get(object->reqQueueHandleTx);
            if (txn == object->reqQueueHandleTx)
            {
                /* No buffers are queued. */
                if (object->XmtObj.loopjobEnable == true)
                {
                    txn = &object->XmtObj.txnLoopjob;
                }
                else
                {
                    status = SystemP_FAILURE;
                }
            }
            if (status == SystemP_SUCCESS)
            {
                object->XmtObj.transaction = txn;
                object->XmtObj.count = 0;
                object->XmtObj.frameCount = object->XmtObj.transaction->count /
                                (object->XmtObj.slotCount * object->XmtObj.serCount);
                object->XmtObj.frameIndex = 0;
                object->XmtObj.slotIndex = 0;
                /* Enable the data ready event transmit interrupt */
                CSL_REG32_FINS(&pReg->XINTCTL, MCASP_XINTCTL_XDATA,
                    CSL_MCASP_XINTCTL_XDATA_ENABLE);
                /* Disable DMA requests generation */
                CSL_REG32_FINS(&pReg->XEVTCTL, MCASP_XEVTCTL_XDATDMA,
                    CSL_MCASP_XEVTCTL_XDATDMA_RSV);
            }
        }
    }
    if (status == SystemP_SUCCESS)
    {
        /* Clear the transmitter status logic */
        CSL_REG32_WR(&pReg->XSTAT, (uint32_t)0x1FFU);

        // case MCASP_TRANSMIT_STATE_TX_FLUSH:
        /* Flush transmitters buffers to an empty state */
        status += MCASP_bitSetGblCtl(handle, CSL_MCASP_GBLCTL_XSRCLR_MASK);
    }
    if (status == SystemP_SUCCESS)
    {
        if (MCASP_TRANSFER_MODE_INTERRUPT == object->transferMode)
        {
            /* Verify that transmitters buffers are already written in ISR */
            regVal = CSL_REG32_RD(&pReg->XSTAT);
            /* wait for opeartion */
            while ((0U != (regVal & CSL_MCASP_XSTAT_XDATA_MASK)) && (timeout > 0))
            {
                /* update read value */
                regVal = CSL_REG32_RD(&pReg->XSTAT);
                timeout--;
            }
            if (0U == timeout)
            {
                status = SystemP_FAILURE;
            }
        }
    }
    if (status == SystemP_SUCCESS)
    {
        // case MCASP_TRANSMIT_STATE_RELEASE_FROM_RESET:
        /* Release transmit state-machine(TSM) from RESET */
        status += MCASP_bitSetGblCtl(handle, CSL_MCASP_GBLCTL_XSMRST_MASK);

        /* Release frame sync generator from RESET */
        status += MCASP_bitSetGblCtl(handle, CSL_MCASP_GBLCTL_XFRST_MASK);
    }
    return status;
}

int32_t MCASP_startTransferRx(MCASP_Handle handle)
{
    int32_t status = SystemP_SUCCESS;
    const MCASP_Attrs *attrs = NULL;
    const CSL_McaspRegs *pReg = NULL;
    MCASP_Object *object = NULL;
    uint32_t regVal;
    int32_t timeout = MCASP_DATA_TIMEOUT;

    if (NULL == handle)
    {
        status = SystemP_FAILURE;
    }
    if (status == SystemP_SUCCESS)
    {
        attrs = ((MCASP_Config *)handle)->attrs;
        if (NULL != attrs)
        {
            pReg = (const CSL_McaspRegs *)attrs->baseAddr;
            object = ((MCASP_Config *)handle)->object;
        }
        if ((NULL == pReg) || (NULL == object))
        {
            status = SystemP_FAILURE;
        }
    }
    if (status == SystemP_SUCCESS)
    {
        object->isRxStarted = 1;
        // case MCASP_RECEIVE_STATE_RX_RESET:
        if(MCASP_TRANSFER_MODE_DMA == object->transferMode)
        {
            /* Enable DMA requests generation */
            CSL_REG32_FINS(&pReg->REVTCTL, MCASP_REVTCTL_RDATDMA,
                CSL_MCASP_REVTCTL_RDATDMA_ENABLE);
            status = MCASP_enableDmaRx((MCASP_Config *)handle);
            if (status != SystemP_SUCCESS)
            {
                /* Disable DMA requests generation */
                CSL_REG32_FINS(&pReg->REVTCTL, MCASP_REVTCTL_RDATDMA,
                    CSL_MCASP_REVTCTL_RDATDMA_RSV);
            }
            /* Disable the data ready event transmit interrupt */
            CSL_REG32_FINS(&pReg->RINTCTL, MCASP_RINTCTL_RDATA,
                CSL_MCASP_RINTCTL_RDATA_DISABLE);
        }
        else if(MCASP_TRANSFER_MODE_INTERRUPT == object->transferMode)
        {
            /* Get the first buffer queued and store in RcvObj to transfer from ISR. */
            MCASP_Transaction *txn = QueueP_get(object->reqQueueHandleRx);
            if (txn == object->reqQueueHandleRx)
            {
                /* No buffers are queued. */
                if (object->RcvObj.loopjobEnable == true)
                {
                    txn = &object->RcvObj.txnLoopjob;
                }
                else
                {
                    status = SystemP_FAILURE;
                }
            }
            if (status == SystemP_SUCCESS)
            {
                object->RcvObj.transaction = txn;
                object->RcvObj.count = 0;
                object->RcvObj.frameCount = object->RcvObj.transaction->count /
                                (object->RcvObj.slotCount * object->RcvObj.serCount);
                object->RcvObj.frameIndex = 0;
                object->RcvObj.slotIndex = 0;
                /* Enable the data ready event transmit interrupt */
                CSL_REG32_FINS(&pReg->RINTCTL, MCASP_RINTCTL_RDATA,
                    CSL_MCASP_RINTCTL_RDATA_ENABLE);
                /* Disable DMA requests generation */
                CSL_REG32_FINS(&pReg->REVTCTL, MCASP_REVTCTL_RDATDMA,
                    CSL_MCASP_REVTCTL_RDATDMA_RSV);
            }
        }
        if (status == SystemP_SUCCESS)
        {

            /* Clear the receivers status logic */
            CSL_REG32_WR(&pReg->RSTAT, (uint32_t)0x1FFU);

            // case MCASP_RECEIVE_STATE_RX_FLUSH:
            /* Flush receivers buffers to an empty state */
            status += MCASP_bitSetGblCtl(handle, CSL_MCASP_GBLCTL_RSRCLR_MASK);
        }
    }
    if (status == SystemP_SUCCESS)
    {
        if (MCASP_TRANSFER_MODE_INTERRUPT == object->transferMode)
        {
            /* Verify that the receiver buffers are already read in ISR */
            regVal = CSL_REG32_RD(&pReg->RSTAT);
            /* wait for opeartion */
            while ((0U != (regVal & CSL_MCASP_RSTAT_RDATA_MASK)) && (timeout > 0))
            {
                /* update read value */
                regVal = CSL_REG32_RD(&pReg->RSTAT);
                timeout--;
            }
            if (0U == timeout)
            {
                status = SystemP_FAILURE;
            }
        }
    }
    if (status == SystemP_SUCCESS)
    {
        // case MCASP_RECEIVE_STATE_RELEASE_FROM_RESET:
        /* Release receive state-machine(TSM) from RESET */
        status += MCASP_bitSetGblCtl(handle, CSL_MCASP_GBLCTL_RSMRST_MASK);
        /* Release frame sync generator from RESET */
        status += MCASP_bitSetGblCtl(handle, CSL_MCASP_GBLCTL_RFRST_MASK);
    }
    return status;
}

int32_t MCASP_stopTransferTx(MCASP_Handle handle)
{
    int32_t status = SystemP_SUCCESS;
    const MCASP_Attrs *attrs = NULL;
    const CSL_McaspRegs *pReg = NULL;
    MCASP_Object *object = NULL;

    if (NULL == handle)
    {
        status = SystemP_FAILURE;
    }
    if (status == SystemP_SUCCESS)
    {
        attrs = ((MCASP_Config *)handle)->attrs;
        if (NULL != attrs)
        {
            pReg = (const CSL_McaspRegs *)attrs->baseAddr;
            object = ((MCASP_Config *)handle)->object;
        }
        if ((NULL == pReg) || (NULL == object))
        {
            status = SystemP_FAILURE;
        }
    }
    if (status == SystemP_SUCCESS)
    {
        object->isTxStarted = 0;
        if(object->transferMode == MCASP_TRANSFER_MODE_DMA)
        {
            /* Disable DMA requests generation */
            CSL_REG32_FINS(&pReg->XEVTCTL, MCASP_XEVTCTL_XDATDMA,
                CSL_MCASP_XEVTCTL_XDATDMA_RSV);
            MCASP_disableDmaTx((MCASP_Config *)handle);
        }
        else if(object->transferMode == MCASP_TRANSFER_MODE_INTERRUPT)
        {
            /* Disable the data ready event transmit interrupt */
            CSL_REG32_FINS(&pReg->XINTCTL, MCASP_XINTCTL_XDATA,
                CSL_MCASP_XINTCTL_XDATA_DISABLE);
        }
    }
    return status;
}

int32_t MCASP_stopTransferRx(MCASP_Handle handle)
{
    int32_t status = SystemP_SUCCESS;
    const MCASP_Attrs *attrs = NULL;
    const CSL_McaspRegs *pReg = NULL;
    MCASP_Object *object = NULL;

    if (NULL == handle)
    {
        status = SystemP_FAILURE;
    }
    if (status == SystemP_SUCCESS)
    {
        attrs = ((MCASP_Config *)handle)->attrs;
        if (NULL != attrs)
        {
            pReg = (const CSL_McaspRegs *)attrs->baseAddr;
            object = ((MCASP_Config *)handle)->object;
        }
        if ((NULL == pReg) || (NULL == object))
        {
            status = SystemP_FAILURE;
        }
    }
    if (status == SystemP_SUCCESS)
    {
        object->isRxStarted = 0;
        if(object->transferMode == MCASP_TRANSFER_MODE_DMA)
        {
            /* Disable DMA requests generation */
            CSL_REG32_FINS(&pReg->REVTCTL, MCASP_REVTCTL_RDATDMA,
                CSL_MCASP_REVTCTL_RDATDMA_RSV);
            MCASP_disableDmaRx((MCASP_Config *)handle);
        }
        else if(object->transferMode == MCASP_TRANSFER_MODE_INTERRUPT)
        {
            /* Disable the data ready event receive interrupt */
            CSL_REG32_FINS(&pReg->RINTCTL, MCASP_RINTCTL_RDATA,
                CSL_MCASP_RINTCTL_RDATA_DISABLE);
        }
    }
    return status;
}

static int32_t MCASP_bitClearGblCtl(const MCASP_Handle handle, uint32_t bitMask)
{
    int32_t status = SystemP_SUCCESS;
    int32_t timeout = MCASP_GBLCTL_TIMEOUT;

    if(handle != NULL)
    {
        uint32_t regVal;
        const MCASP_Attrs *attrs = ((MCASP_Config *)handle)->attrs;
        const CSL_McaspRegs *pReg = (const CSL_McaspRegs *)attrs->baseAddr;

        regVal = CSL_REG32_RD(&pReg->GBLCTL);

        /* Check if requested bit(s) are already cleared */
        if (0U != (regVal & bitMask))
        {
            regVal &= ~bitMask;
            CSL_REG32_WR(&pReg->GBLCTL, regVal);

            /* Confirm the clear operation */
            regVal = CSL_REG32_RD(&pReg->GBLCTL);

            /* wait for opeartion */
            while ((0U != (regVal & bitMask)) && (timeout > 0))
            {
                /* update read value */
                regVal = CSL_REG32_RD(&pReg->GBLCTL);
                timeout--;
            }

            if (0U == timeout)
            {
                status = SystemP_FAILURE;
            }
        }
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

static int32_t MCASP_bitSetGblCtl(const MCASP_Handle handle, uint32_t bitMask)
{
    int32_t status = SystemP_SUCCESS;
    int32_t timeout = MCASP_GBLCTL_TIMEOUT;

    if(handle != NULL)
    {
        uint32_t regVal;
        const MCASP_Attrs *attrs = ((MCASP_Config *)handle)->attrs;
        const CSL_McaspRegs *pReg = (const CSL_McaspRegs *)attrs->baseAddr;

        regVal = CSL_REG32_RD(&pReg->GBLCTL);

        /* Check if requested bit(s) are already set */
        if (bitMask != (regVal & bitMask))
        {
            regVal |= bitMask;
            CSL_REG32_WR(&pReg->GBLCTL, regVal);

            /* Confirm the clear operation */
            regVal = CSL_REG32_RD(&pReg->GBLCTL);

            /* wait for opeartion */
            while ((bitMask != (regVal & bitMask)) && (timeout > 0))
            {
                /* update read value */
                regVal = CSL_REG32_RD(&pReg->GBLCTL);
                timeout--;
            }

            if (0U == timeout)
            {
                status = SystemP_FAILURE;
            }
        }
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

static int32_t MCASP_getBufferOffset(MCASP_TransferObj *xfrObj, uint8_t serIdx,
                                                             uint32_t* pOffset)
{
    int32_t status = SystemP_SUCCESS;

    switch(xfrObj->bufferFormat)
    {
        case MCASP_AUDBUFF_FORMAT_1SER_MULTISLOT_INTERLEAVED:
        case MCASP_AUDBUFF_FORMAT_MULTISER_MULTISLOT_SEMI_INTERLEAVED_1:
        {
            *pOffset = xfrObj->count;
            break;
        }
        case MCASP_AUDBUFF_FORMAT_1SER_MULTISLOT_NON_INTERLEAVED:
        {
            *pOffset = (xfrObj->frameIndex * xfrObj->frameCount) + xfrObj->slotIndex;
            xfrObj->frameIndex++;
            if (xfrObj->frameIndex == xfrObj->slotCount)
            {
                xfrObj->slotIndex++;
                xfrObj->frameIndex = 0;
            }
            break;
        }
        case MCASP_AUDBUFF_FORMAT_MULTISER_MULTISLOT_SEMI_INTERLEAVED_2:
        {
            *pOffset = (serIdx * xfrObj->frameCount * xfrObj->slotCount) +
                     (xfrObj->frameIndex * xfrObj->slotCount) +
                      xfrObj->slotIndex;
            if (serIdx == xfrObj->serCount - 1)
            {
                xfrObj->slotIndex++;
                if (xfrObj->slotIndex == xfrObj->slotCount)
                {
                    xfrObj->slotIndex = 0;
                    xfrObj->frameIndex++;
                }
            }
            break;
        }
        default:
        {
            status = SystemP_FAILURE;
            break;
        }
    }

    return status;
}

static void MCASP_tx_isr(void *args)
{
    const MCASP_Attrs *attrs = ((MCASP_Config *)args)->attrs;
    const CSL_McaspRegs *pReg = (const CSL_McaspRegs *)attrs->baseAddr;
    MCASP_Object *object = ((MCASP_Config *)args)->object;
    MCASP_TransferObj *xfrObj = &(object->XmtObj);
    MCASP_Transaction *txn = xfrObj->transaction;
    uint32_t regVal, tmpVal, offset;
    int32_t status = SystemP_SUCCESS;
    uint8_t serIdx;

    /* Check if there is a transmitter buffer empty event */
    regVal = CSL_REG32_RD(&pReg->XSTAT);
    if ((CSL_MCASP_XSTAT_XDATA_MASK == (regVal & CSL_MCASP_XSTAT_XDATA_MASK)))
    {
        /* Check if CPU transfer is through the DATA port */
        regVal = CSL_REG32_RD(&pReg->XFMT);
        if ((CSL_MCASP_XFMT_XBUSEL_VBUSP == (regVal & CSL_MCASP_XFMT_XBUSEL_MASK)))
        {
            /* TODO: Write active slot data for transmit channels subsequently
                     using only MCASP_TXBUF0 DATA port physical address */
        }
        else
        {
            /* Explicitly write active slot data to different Tx buffers CFG
               port addresses */
            for (serIdx = 0; serIdx < xfrObj->serCount; serIdx++)
            {
                /* Check if Transmit buffer ready */
                regVal = CSL_REG32_RD(&pReg->SRCTL0 + xfrObj->serArray[serIdx]);
                if (CSL_MCASP_SRCTL0_XRDY_MASK == (regVal & CSL_MCASP_SRCTL0_XRDY_MASK))
                {
                    /* get audio buffer offset based on buffer format */
                    status = MCASP_getBufferOffset(xfrObj, serIdx, &offset);

                    /* Write active slot data */
                    if (status == SystemP_SUCCESS)
                    {
                        tmpVal = *((uint32_t *)txn->buf + offset);
                        CSL_REG32_WR(&pReg->XBUF0 + xfrObj->serArray[serIdx], tmpVal);
                        xfrObj->count++;
                        if (xfrObj->count >= txn->count)
                        {
                            uint32_t callCbFxn = true;
                            if (txn == &object->XmtObj.txnLoopjob)
                            {
                                /* current transaction is from loopjob. */
                                callCbFxn = false;
                            }
                            /* transfer from current buffer is complete. */
                            /* get next buffer and program. */
                            MCASP_Transaction *newTxn = QueueP_get(object->reqQueueHandleTx);
                            if (newTxn == object->reqQueueHandleTx)
                            {
                                /* No new buffers are loaded. */
                                if (object->XmtObj.loopjobEnable == true)
                                {
                                    newTxn = &object->XmtObj.txnLoopjob;
                                }
                                else
                                {
                                    /* repeate the current txn. */
                                    newTxn = txn;
                                    callCbFxn = false;
                                }
                            }
                            if (callCbFxn == true)
                            {
                                /* Give callback to the current transaction. */
                                txn->status = SystemP_SUCCESS;
                                xfrObj->cbFxn((MCASP_Handle *)args, txn);
                            }
                            xfrObj->transaction = newTxn;
                            xfrObj->count = 0;
                            xfrObj->frameCount = newTxn->count / (xfrObj->slotCount * xfrObj->serCount);
                            xfrObj->frameIndex = 0;
                            xfrObj->slotIndex = 0;
                        }
                    }
                }
            }
        }

        if (status == SystemP_SUCCESS)
        {
            /* Check if there is LAST SLOT request event */
            regVal = CSL_REG32_RD(&pReg->XSTAT);
            if (CSL_MCASP_XSTAT_XLAST_MASK == (regVal & CSL_MCASP_XSTAT_XLAST_MASK))
            {
                /* Last slot */
                /* Clear the XLAST bit */
                CSL_REG32_FINS(&pReg->XSTAT, MCASP_XSTAT_XLAST,
                            CSL_MCASP_XSTAT_XLAST_YES);
            }
        }
    }
    else
    {
        if ((CSL_MCASP_XSTAT_XERR_MASK == (regVal & CSL_MCASP_XSTAT_XERR_MASK)))
        {
            /* TODO: Tx Error Handelling */
        }
        else
        {
            /* TODO: Tx Start of frame and odd/even slot processing */
        }
    }
}

static void MCASP_rx_isr(void *args)
{
    const MCASP_Attrs *attrs = ((MCASP_Config *)args)->attrs;
    const CSL_McaspRegs *pReg = (const CSL_McaspRegs *)attrs->baseAddr;
    MCASP_Object *object = ((MCASP_Config *)args)->object;
    MCASP_TransferObj *xfrObj = &(object->RcvObj);
    MCASP_Transaction *txn = xfrObj->transaction;
    uint32_t regVal, offset;
    int32_t status = SystemP_SUCCESS;
    uint8_t serIdx;

    /* Check if there is a receiver buffer full event */
    regVal = CSL_REG32_RD(&pReg->RSTAT);
    if ((CSL_MCASP_RSTAT_RDATA_MASK == (regVal & CSL_MCASP_RSTAT_RDATA_MASK)))
    {
        /* Check if CPU transfer is through the DATA port */
        regVal = CSL_REG32_RD(&pReg->RFMT);
        if ((CSL_MCASP_RFMT_RBUSEL_VBUSP == (regVal & CSL_MCASP_RFMT_RBUSEL_MASK)))
        {
            /* TODO: Read active slot data for receive channels subsequently
                     using only MCASP_RXBUF0 DATA port physical address */
        }
        else
        {
            /* Explicitly read active slot data from different Rx buffers CFG
               port addresses */
            for (serIdx = 0; serIdx < xfrObj->serCount; serIdx++)
            {
                /* Check if Receive buffer ready */
                regVal = CSL_REG32_RD(&pReg->SRCTL0 + xfrObj->serArray[serIdx]);
                if (CSL_MCASP_SRCTL0_RRDY_MASK == (regVal & CSL_MCASP_SRCTL0_RRDY_MASK))
                {
                    /* get audio buffer offset based on buffer format */
                    status = MCASP_getBufferOffset(xfrObj, serIdx, &offset);

                    /* Read active slot data */
                    if (status == SystemP_SUCCESS)
                    {
                        *((uint32_t *)txn->buf + offset) =
                                CSL_REG32_RD(&pReg->RBUF0 + xfrObj->serArray[serIdx]);
                        xfrObj->count++;
                        if (xfrObj->count >= txn->count)
                        {
                            uint32_t callCbFxn = true;
                            if (txn == &object->RcvObj.txnLoopjob)
                            {
                                /* current transaction is from loopjob. */
                                callCbFxn = false;
                            }
                            /* transfer from current buffer is complete. */
                            /* get next buffer and program. */
                            MCASP_Transaction *newTxn = QueueP_get(object->reqQueueHandleRx);
                            if (newTxn == object->reqQueueHandleRx)
                            {
                                /* No new buffers are loaded. */
                                if (object->RcvObj.loopjobEnable == true)
                                {
                                    newTxn = &object->RcvObj.txnLoopjob;
                                }
                                else
                                {
                                    /* repeate the current txn. */
                                    newTxn = txn;
                                    callCbFxn = false;
                                }
                            }
                            if (callCbFxn == true)
                            {
                                /* Give callback to the current transaction. */
                                txn->status = SystemP_SUCCESS;
                                xfrObj->cbFxn((MCASP_Handle *)args, txn);
                            }
                            xfrObj->transaction = newTxn;
                            xfrObj->count = 0;
                            xfrObj->frameCount = newTxn->count / (xfrObj->slotCount * xfrObj->serCount);
                            xfrObj->frameIndex = 0;
                            xfrObj->slotIndex = 0;
                        }
                    }
                }
            }
        }

        if (status == SystemP_SUCCESS)
        {
            /* Check if there is LAST SLOT request event */
            regVal = CSL_REG32_RD(&pReg->RSTAT);
            if (CSL_MCASP_RSTAT_RLAST_MASK == (regVal & CSL_MCASP_RSTAT_RLAST_MASK))
            {
                /* Last slot */
                /* Clear the XLAST bit */
                CSL_REG32_FINS(&pReg->RSTAT, MCASP_RSTAT_RLAST,
                            CSL_MCASP_RSTAT_RLAST_YES);
            }
        }
    }
    else
    {
        if ((CSL_MCASP_RSTAT_RERR_MASK == (regVal & CSL_MCASP_RSTAT_RERR_MASK)))
        {
            /* TODO: Rx Error Handelling */
        }
        else
        {
            /* TODO: Rx Start of frame and odd/even slot processing */
        }
    }
}
