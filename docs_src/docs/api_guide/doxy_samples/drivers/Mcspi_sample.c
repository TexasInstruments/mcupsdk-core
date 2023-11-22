
#include <stdio.h>
//! [include]
#include <drivers/mcspi.h>
//! [include]
#include <drivers/hw_include/csl_types.h>
#include <kernel/dpl/SystemP.h>
#include <kernel/dpl/SemaphoreP.h>

#define CONFIG_MCSPI0           (0U)
#define APP_MCSPI_MSGSIZE       (8U)
#define APP_MCSPI_NUM_CH        (1U)

MCSPI_Handle        gMcspiHandle;
SemaphoreP_Object   gMcspiISRDoneSem;
MCSPI_ChConfig      gMcspiChConfig[APP_MCSPI_NUM_CH];

void open(void)
{
//! [open]
    MCSPI_OpenParams    spiParams;

    MCSPI_OpenParams_init(&spiParams);      /* Initialize SPI parameters */
    spiParams.transferMode  = MCSPI_TRANSFER_MODE_BLOCKING;
    gMcspiHandle = MCSPI_open(CONFIG_MCSPI0, &spiParams);
    DebugP_assert(gMcspiHandle != NULL);

//! [open]
}

void close(void)
{
//! [close]
    MCSPI_close(gMcspiHandle);
//! [close]
}

void transfer_blocking(void)
{
//! [transfer_blocking]
    int32_t             transferOK;
    MCSPI_Transaction   spiTransaction;
    uint8_t             transmitBuffer[APP_MCSPI_MSGSIZE];
    uint8_t             receiveBuffer[APP_MCSPI_MSGSIZE];

    /* Fill in transmitBuffer */
    spiTransaction.channel  = 0U;
    spiTransaction.dataSize  = 16U;
    spiTransaction.csDisable = TRUE;
    spiTransaction.count    = APP_MCSPI_MSGSIZE;
    spiTransaction.txBuf    = (void *)transmitBuffer;
    spiTransaction.rxBuf    = (void *)receiveBuffer;
    spiTransaction.args     = NULL;

    /* Initiate transfer */
    transferOK = MCSPI_transfer(gMcspiHandle, &spiTransaction);
    if((SystemP_SUCCESS != transferOK) ||
       (MCSPI_TRANSFER_COMPLETED != spiTransaction.status))
    {
        /* MCSPI transfer failed!! */
        DebugP_assert(FALSE);
    }
//! [transfer_blocking]
}

void chain_transfer_blocking(void)
{
//! [chain_transfer_blocking]
    int32_t             transferOK;
    MCSPI_Transaction   spiTransaction;
    uint8_t             transmitBuffer[APP_MCSPI_MSGSIZE];
    uint8_t             receiveBuffer[APP_MCSPI_MSGSIZE];

    /* Fill in transmitBuffer with commands */
    spiTransaction.channel  = 0U;
    spiTransaction.dataSize  = 8U;
    spiTransaction.csDisable = FALSE;
    spiTransaction.count    = APP_MCSPI_MSGSIZE;
    spiTransaction.txBuf    = (void *)transmitBuffer;
    spiTransaction.rxBuf    = (void *)receiveBuffer;
    spiTransaction.args     = NULL;

    /* Initiate transfer */
    transferOK = MCSPI_transfer(gMcspiHandle, &spiTransaction);
    if((SystemP_SUCCESS != transferOK) ||
       (MCSPI_TRANSFER_COMPLETED != spiTransaction.status))
    {
        /* MCSPI transfer failed!! */
        DebugP_assert(FALSE);
    }

    /* Read Data from peripheral if any */

    /* Fill in transmitBuffer with data */
    spiTransaction.channel  = 0U;
    spiTransaction.dataSize  = 16U;
    spiTransaction.csDisable = TRUE;
    spiTransaction.count    = APP_MCSPI_MSGSIZE;
    spiTransaction.txBuf    = (void *)transmitBuffer;
    spiTransaction.rxBuf    = (void *)receiveBuffer;
    spiTransaction.args     = NULL;

    /* Initiate transfer */
    transferOK = MCSPI_transfer(gMcspiHandle, &spiTransaction);
    if((SystemP_SUCCESS != transferOK) ||
       (MCSPI_TRANSFER_COMPLETED != spiTransaction.status))
    {
        /* MCSPI transfer failed!! */
        DebugP_assert(FALSE);
    }

//! [chain_transfer_blocking]
}

//! [transfer_nonblocking]
void App_callbackFxn(MCSPI_Handle handle,
                     MCSPI_Transaction *transaction)
{
    SemaphoreP_Object *semObj;

    if((NULL != transaction) &&
       (MCSPI_TRANSFER_COMPLETED == transaction->status))
    {
        semObj = (SemaphoreP_Object *) transaction->args;
        if(NULL != semObj)
        {
            SemaphoreP_post(semObj);
        }
    }
}

void transfer_nonblocking(void)
{
    int32_t             status;
    int32_t             transferOK;
    MCSPI_OpenParams    spiParams;
    MCSPI_Transaction   spiTransaction;
    uint8_t             transmitBuffer[APP_MCSPI_MSGSIZE];
    uint8_t             receiveBuffer[APP_MCSPI_MSGSIZE];

    MCSPI_OpenParams_init(&spiParams);      /* Initialize SPI parameters */
    spiParams.transferMode          = MCSPI_TRANSFER_MODE_CALLBACK;
    spiParams.transferCallbackFxn   = &App_callbackFxn;
    gMcspiHandle = MCSPI_open(CONFIG_MCSPI0, &spiParams);
    DebugP_assert(gMcspiHandle != NULL);

    /* Fill in transmitBuffer */
    spiTransaction.channel  = 0U;
    spiTransaction.csDisable = TRUE;
    spiTransaction.dataSize  = 16U;
    spiTransaction.count    = APP_MCSPI_MSGSIZE;
    spiTransaction.txBuf    = (void *)transmitBuffer;
    spiTransaction.rxBuf    = (void *)receiveBuffer;
    spiTransaction.args     = &gMcspiISRDoneSem;    /* Pass semaphore */

    /* Initiate transfer */
    transferOK = MCSPI_transfer(gMcspiHandle, &spiTransaction);
    DebugP_assert(transferOK == SystemP_SUCCESS);

    /* Wait for callback */
    status = SemaphoreP_pend(&gMcspiISRDoneSem, SystemP_WAIT_FOREVER);
    DebugP_assert(status == SystemP_SUCCESS);
}
//! [transfer_nonblocking]
