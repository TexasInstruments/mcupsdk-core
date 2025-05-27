
#include <stdio.h>
//! [include]
#include <drivers/uart.h>
//! [include]

#define CONFIG_UART0            (0U)
#define APP_UART_MSGSIZE        (8U)

UART_Handle        gUartHandle;
/* Semaphore to indicate Write/Read completion used in callback api's */
SemaphoreP_Object gUartWriteDoneSem;
SemaphoreP_Object gUartReadDoneSem;
/* Variables to get read/write count in callbacks */
volatile uint32_t gNumBytesRead = 0U, gNumBytesWritten = 0U;

void open(void)
{
//! [open]
    UART_Params         params;

    UART_Params_init(&params);      /* Initialize parameters */
    params.baudRate = 115200;
    gUartHandle = UART_open(CONFIG_UART0, &params);
    DebugP_assert(gUartHandle != NULL);
//! [open]
}

void close(void)
{
//! [close]
    UART_close(gUartHandle);
//! [close]
}

void write_transfer_blocking(void)
{
//! [write_transfer_blocking]
    int32_t             transferOK;
    UART_Transaction    transaction;
    uint8_t             txBuffer[APP_UART_MSGSIZE];

    UART_Transaction_init(&transaction);
 
 /* Initiate write */
    transaction.count   = APP_UART_MSGSIZE;
    transaction.buf     = (void *)txBuffer;
    transaction.args    = NULL;
    transferOK = UART_write(gUartHandle, &transaction);
    if((SystemP_SUCCESS != transferOK) ||
       (UART_TRANSFER_STATUS_SUCCESS != transaction.status))
    {
        /* UART transfer failed!! */
        DebugP_assert(FALSE);
    }
//! [write_transfer_blocking]
}

void read_transfer_blocking(void)
{
//! [read_transfer_blocking]
    int32_t             transferOK;
    UART_Transaction    transaction;
    uint8_t             rxBuffer[APP_UART_MSGSIZE];

    UART_Transaction_init(&transaction);

    /* Initiate read */
    transaction.count   = APP_UART_MSGSIZE;
    transaction.buf     = (void *)rxBuffer;
    transaction.args    = NULL;
    transferOK = UART_read(gUartHandle, &transaction);
    if((SystemP_SUCCESS != transferOK) ||
       (UART_TRANSFER_STATUS_SUCCESS != transaction.status))
    {
        /* UART transfer failed!! */
        DebugP_assert(FALSE);
    }
//! [read_transfer_blocking]
}

//! [write_transfer_nonblocking]
void write_callback(UART_Handle handle, UART_Transaction *trans)
{
    DebugP_assertNoLog(UART_TRANSFER_STATUS_SUCCESS == trans->status);
    gNumBytesWritten = trans->count;
    SemaphoreP_post(&gUartWriteDoneSem);

    return;
}

void write_transfer_nonblocking(void)
{
    int32_t             transferOK;
    UART_Transaction    transaction;
    uint8_t             txBuffer[APP_UART_MSGSIZE];

    UART_Transaction_init(&transaction);

    /* Initiate write */
    transaction.count   = APP_UART_MSGSIZE;
    transaction.buf     = (void *)txBuffer;
    transaction.args    = NULL;
    transferOK = UART_write(gUartHandle, &transaction);
    if((SystemP_SUCCESS != transferOK) ||
       (UART_TRANSFER_STATUS_SUCCESS != transaction.status))
    {
        /* UART transfer failed!! */
        DebugP_assert(FALSE);
    }
    else
    {
        /* Wait for callback */
        SemaphoreP_pend(&gUartWriteDoneSem, SystemP_WAIT_FOREVER);
        DebugP_assert(gNumBytesWritten == transaction.count);
    }
}
//! [write_transfer_nonblocking]

//! [read_transfer_nonblocking]
void read_callback(UART_Handle handle, UART_Transaction *trans)
{
    DebugP_assertNoLog(UART_TRANSFER_STATUS_SUCCESS == trans->status);
    gNumBytesRead = trans->count;
    SemaphoreP_post(&gUartReadDoneSem);

    return;
}

void read_transfer_nonblocking(void)
{
    int32_t             transferOK;
    UART_Transaction    transaction;
    uint8_t             rxBuffer[APP_UART_MSGSIZE];

    UART_Transaction_init(&transaction);

    /* Initiate read */
    transaction.count   = APP_UART_MSGSIZE;
    transaction.buf     = (void *)rxBuffer;
    transaction.args    = NULL;
    transferOK = UART_read(gUartHandle, &transaction);
    if((SystemP_SUCCESS != transferOK) ||
       (UART_TRANSFER_STATUS_SUCCESS != transaction.status))
    {
        /* UART transfer failed!! */
        DebugP_assert(FALSE);
    }
    else
    {
        /* Wait for callback */
        SemaphoreP_pend(&gUartReadDoneSem, SystemP_WAIT_FOREVER);
        DebugP_assert(gNumBytesRead == transaction.count);
    }
}
//! [read_transfer_nonblocking]
