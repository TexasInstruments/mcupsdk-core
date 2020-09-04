
#include <stdio.h>
//! [include]
#include <drivers/mcasp.h>
//! [include]
#include <drivers/hw_include/csl_types.h>
#include <kernel/dpl/SystemP.h>
#include <kernel/dpl/SemaphoreP.h>

#define CONFIG_EDMA0            (0U)
#define CONFIG_MCASP0           (0U)
#define APP_MCASP_MSGSIZE       (128U)
#define APP_MCASP_MSG_COUNT     (2U)

MCASP_Handle        gMcaspHandle;
void mcasp_loopback_txcb (MCASP_Handle handle,
                          MCASP_Transaction *transaction);
void mcasp_loopback_rxcb (MCASP_Handle handle,
                          MCASP_Transaction *transaction);

void open(void)
{
//! [open]
    MCASP_OpenParams     mcaspParams;
    MCASP_openParamsInit(&mcaspParams); /* Initialize mcasp parameters */
    mcaspParams.edmaInst  = 0;
    mcaspParams.edmaInst     = CONFIG_EDMA0,
    mcaspParams.transferMode = MCASP_TRANSFER_MODE_DMA,
    mcaspParams.txCallbackFxn = mcasp_loopback_txcb,
    mcaspParams.rxCallbackFxn = mcasp_loopback_rxcb,

    gMcaspHandle = MCASP_open(CONFIG_MCASP0, &mcaspParams);
    DebugP_assert(gMcaspHandle != NULL);
//! [open]
}

void close(void)
{
//! [close]
    MCASP_close(gMcaspHandle);
//! [close]
}

void transfer_loopback (void)
{
//! [start_transfer_loopback]
    MCASP_Transaction   txnTx[APP_MCASP_MSG_COUNT] = {0};
    MCASP_Transaction   txnRx[APP_MCASP_MSG_COUNT] = {0};
    uint8_t mcaspTxBuffer[APP_MCASP_MSG_COUNT][APP_MCASP_MSGSIZE];
    uint8_t mcaspRxBuffer[APP_MCASP_MSG_COUNT][APP_MCASP_MSGSIZE];
    uint32_t i;

    for (i = 0U; i < APP_MCASP_MSG_COUNT; i++)
    {
        txnTx[i].buf = (void*) &mcaspTxBuffer[i][0];
        txnTx[i].count = APP_MCASP_MSGSIZE/4;
        txnTx[i].timeout = 0xFFFFFF;
        MCASP_submitTx(gMcaspHandle, &txnTx[i]);
    }
    for (i = 0U; i < APP_MCASP_MSG_COUNT; i++)
    {
        txnRx[i].buf = (void*) &mcaspRxBuffer[i][0];
        txnRx[i].count = APP_MCASP_MSGSIZE/4;
        txnRx[i].timeout = 0xFFFFFF;
        MCASP_submitRx(gMcaspHandle,  &txnRx[i]);
    }

    MCASP_startTransferRx(gMcaspHandle);
    MCASP_startTransferTx(gMcaspHandle);
//! [start_transfer_loopback]

//! [stop_transfer_loopback]
    MCASP_stopTransferTx(gMcaspHandle);
    MCASP_stopTransferRx(gMcaspHandle);

    /* withdraw the buffers submitted to driver. */
    {
        MCASP_Transaction *transaction;
        do {
            transaction = MCASP_withdrawRx(gMcaspHandle);
        }while (transaction != NULL);
        do {
            transaction = MCASP_withdrawTx(gMcaspHandle);
        }while (transaction != NULL);
    }
//! [stop_transfer_loopback]
}

//! [mcasp_callback_functions]
void mcasp_loopback_txcb (MCASP_Handle handle,
                          MCASP_Transaction *transaction)
{
    /* Sample Tx callback function. Submit the same buffer again to driver. */
    MCASP_submitTx(handle, transaction);
}

void mcasp_loopback_rxcb (MCASP_Handle handle,
                          MCASP_Transaction *transaction)
{
    /* Sample Rx callback function. Submit the same buffer again to driver. */
    MCASP_submitRx(handle, transaction);
}

//! [mcasp_callback_functions]