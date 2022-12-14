
#include <stdio.h>
//! [include]
#include <drivers/mibspi.h>
//! [include]
#include <drivers/hw_include/csl_types.h>
#include <kernel/dpl/SystemP.h>
#include <kernel/dpl/SemaphoreP.h>

#define CONFIG_MIBSPI0           (0U)
#define APP_MIBSPI_MSGSIZE       (128U)

MIBSPI_Handle        gMibspiHandle;
SemaphoreP_Object    gMibspiISRDoneSem;

void open(void)
{
//! [open]
    MIBSPI_OpenParams     spiParams;

    MIBSPI_Params_init(&spiParams);      /* Initialize SPI parameters */
    spiParams.transferMode  = MIBSPI_MODE_BLOCKING;
    gMibspiHandle = MIBSPI_open(CONFIG_MIBSPI0, &spiParams);
    DebugP_assert(gMibspiHandle != NULL);

//! [open]
}

void close(void)
{
//! [close]
    MIBSPI_close(gMibspiHandle);
//! [close]
}

void transfer_blocking(void)
{
//! [transfer_blocking]
    int32_t              transferOK;
    MIBSPI_Transaction   spiTransaction;
    uint8_t              transmitBuffer[APP_MIBSPI_MSGSIZE];
    uint8_t              receiveBuffer[APP_MIBSPI_MSGSIZE];

    /* Fill in transmitBuffer */
    spiTransaction.count        = APP_MIBSPI_MSGSIZE;
    spiTransaction.txBuf        = (void *)transmitBuffer;
    spiTransaction.rxBuf        = (void *)receiveBuffer;
    spiTransaction.peripheralIndex   = 0U;
    spiTransaction.arg          = NULL;

    /* Initiate transfer */
    transferOK = MIBSPI_transfer(gMibspiHandle, &spiTransaction);
    if((SystemP_SUCCESS != transferOK) ||
       (MIBSPI_TRANSFER_COMPLETED != spiTransaction.status))
    {
        /* MIBSPI transfer failed!! */
        DebugP_assert(FALSE);
    }
//! [transfer_blocking]
}

//! [transfer_nonblocking]
void App_callbackFxn(MIBSPI_Handle handle,
                     MIBSPI_Transaction *transaction)
{
    SemaphoreP_Object *semObj;

    if((NULL != transaction) &&
       (MIBSPI_TRANSFER_COMPLETED == transaction->status))
    {
        semObj = (SemaphoreP_Object *) transaction->arg;
        if(NULL != semObj)
        {
            SemaphoreP_post(semObj);
        }
    }
}

void transfer_nonblocking(void)
{
    int32_t              status;
    int32_t              transferOK;
    MIBSPI_OpenParams    spiParams;
    MIBSPI_Transaction   spiTransaction;
    uint8_t              transmitBuffer[APP_MIBSPI_MSGSIZE];
    uint8_t              receiveBuffer[APP_MIBSPI_MSGSIZE];

    MIBSPI_Params_init(&spiParams);      /* Initialize SPI parameters */
    spiParams.transferMode          = MIBSPI_MODE_CALLBACK;
    spiParams.transferCallbackFxn   = &App_callbackFxn;
    gMibspiHandle = MIBSPI_open(CONFIG_MIBSPI0, &spiParams);
    DebugP_assert(gMibspiHandle != NULL);

    /* Fill in transmitBuffer */
    spiTransaction.count        = APP_MIBSPI_MSGSIZE;
    spiTransaction.txBuf        = (void *)transmitBuffer;
    spiTransaction.rxBuf        = (void *)receiveBuffer;
    spiTransaction.peripheralIndex   = 0U;
    spiTransaction.arg          = &gMibspiISRDoneSem;;

    /* Initiate transfer */
    transferOK = MIBSPI_transfer(gMibspiHandle, &spiTransaction);
    DebugP_assert(transferOK == SystemP_SUCCESS);

    /* Wait for callback */
    status = SemaphoreP_pend(&gMibspiISRDoneSem, SystemP_WAIT_FOREVER);
    DebugP_assert(status == SystemP_SUCCESS);
}
//! [transfer_nonblocking]
