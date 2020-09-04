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

#include <string.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/TaskP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/*
 * This example demonstrate the UART low latency API in polling mode.
 * This example receives 8 characters and echos back the same.
 * The application ends when the user types 8 characters.
 * Initially the application sets a buffer to receive data.
 *
 * In the main context, the application checks any data from the UART
 * FIFO and if so, write to the RX buffer and sets the RX buffer count.
 * Application then copies the data to TX buffer and initiate the UART TX (echo).
 *
 */

/** \brief Size of UART buffer to receive and transmit data */
#define APP_UART_BUFSIZE                (8U)

/** \brief App state structure to share data between main fxn and ISR */
typedef struct
{
    /*
     * UART variables - Set once
     */
    uint32_t                baseAddr;
    /*< UART base address */

    /*
     * UART write state variables - set for every transfer
     */
    const uint8_t          *writeBuf;
    /**< Buffer data pointer */
    volatile uint32_t       writeCount;
    /**< Number of Chars sent */
    volatile uint32_t       writeSizeRemaining;
    /**< Chars remaining in buffer */

    /*
     * UART receive state variables - set for every transfer
     */
    uint8_t                *readBuf;
    /**< Read Buffer data pointer */
    volatile uint32_t       readCount;
    /**< Number of Chars read */
    volatile uint32_t       rxOverflow;
    /**< Flag to indicate if num of chars read more than the given size */
} UART_AppPrms;

/** \brief UART RX buffer where received data is put in ISR */
static uint8_t              gAppUartRxBuffer[APP_UART_BUFSIZE];
/** \brief UART TX buffer where TX data is put in Task context */
static uint8_t              gAppUartTxBuffer[APP_UART_BUFSIZE];
/** \brief UART app object */
static UART_AppPrms         gAppPrms;

/* Static Function Declarations */
static void App_uartTx(UART_AppPrms   *appPrms,
                       const uint8_t  *writeBuf,
                       uint32_t        writeSize);
static void App_uartRx(UART_AppPrms *appPrms, uint8_t *readBuf);
static void App_uartInit(UART_AppPrms *appPrms);
static uint32_t App_uartTxFifoWrite(UART_AppPrms   *appPrms,
                                    const uint8_t  *writeBuf,
                                    uint32_t        writeSizeRemaining);
static void App_uartRxFifoRead(UART_AppPrms *appPrms, uint8_t *readBuf);

void uart_echo_low_latency_polling(void *args)
{
    UART_AppPrms            *appPrms = &gAppPrms;
    const char              *printStr = "This is uart low latency test in polling mode, Receives 8 characters then echo's back and exits..\r\n";
    const char              *printExitStr = "\r\nAll tests have passed!!\r\n";
    uint32_t                 numCharsRead, i, charRecvdCnt;

    Drivers_open();
    Board_driversOpen();

    DebugP_log("[UART] Echo Low Latency polling mode example started ...\r\n");

    /* Init App Prms */
    App_uartInit(appPrms);

    /* Send entry string and wait for Tx completion */
    App_uartTx(appPrms, (const uint8_t *) printStr, strlen(printStr));

    charRecvdCnt = 0U;
    /* Echo whatever is read */
    while(1)
    {
        /* Init Rx Prms and wait for Rx input */
        App_uartRx(appPrms, &gAppUartRxBuffer[0U]);
        if(appPrms->readCount > 0U)
        {
            /* copy data from RX buf to TX buf */
            numCharsRead = appPrms->readCount;
            for(i = 0U; i < numCharsRead; i++)
            {
                gAppUartTxBuffer[i] = gAppUartRxBuffer[i];
            }
            appPrms->readCount = 0U; /* Reset variable so that RX can start from first */

            /* Echo the characters received and wait for Tx completion */
            App_uartTx(appPrms, &gAppUartTxBuffer[0U], numCharsRead);

            charRecvdCnt++;
        }

        /* Error check */
        if(appPrms->rxOverflow == TRUE)
        {
           DebugP_log("Rx overflow occurred!!\r\n");
           break;
        }

        /* Error check */
        if (charRecvdCnt >= APP_UART_BUFSIZE)
        {
            break;
        }

        TaskP_yield();
    }

    /* Send exit string and wait for Tx completion */
    App_uartTx(appPrms, (const uint8_t *) printExitStr, strlen(printExitStr));

    DebugP_log("All tests have passed!!\r\n");

    Board_driversClose();
    Drivers_close();

    return;
}

static void App_uartTx(UART_AppPrms   *appPrms,
                       const uint8_t  *writeBuf,
                       uint32_t        writeSize)
{
    uint32_t    baseAddr        = appPrms->baseAddr;
    uint32_t    lineStatus      = 0U, numBytesWritten;

    appPrms->writeBuf           = (const uint8_t *)writeBuf;
    appPrms->writeCount         = 0U;
    appPrms->writeSizeRemaining = writeSize;

    /* Send characters until done */
    while (0U != appPrms->writeSizeRemaining)
    {
        numBytesWritten = App_uartTxFifoWrite(appPrms,
                                     appPrms->writeBuf,
                                     appPrms->writeSizeRemaining);

        appPrms->writeSizeRemaining  -= numBytesWritten;
        appPrms->writeBuf            += numBytesWritten;
        appPrms->writeCount          += numBytesWritten;
    }

    /* Ensure TX FIFO Empty before exiting transfer function */
    if (0U == appPrms->writeSizeRemaining)
    {
        do
        {
            lineStatus = UART_readLineStatus(baseAddr);
        }
        while ((uint32_t) (UART_LSR_TX_FIFO_E_MASK |
                         UART_LSR_TX_SR_E_MASK) !=
               (lineStatus & (uint32_t) (UART_LSR_TX_FIFO_E_MASK |
                                       UART_LSR_TX_SR_E_MASK)));
    }

    return;
}

static uint32_t App_uartTxFifoWrite(UART_AppPrms   *appPrms,
                                    const uint8_t  *writeBuf,
                                    uint32_t        writeSizeRemaining)
{
    uint32_t lineStatus            = 0U;
    uint32_t tempChunksize         = 0U;
    const uint8_t *tmpBuf          = writeBuf;
    int32_t maxTrialCount          = (int32_t) UART_TRANSMITEMPTY_TRIALCOUNT;
    uint32_t writeSize             = writeSizeRemaining;

    /* Load the fifo size  */
    tempChunksize = UART_FIFO_SIZE;

    /* Before we could write no of bytes, we should have
     * no of free buffers. Hence, we check for shiftregister
     * empty (ensure the FIFO is empty) to write num of bytes */
    do
    {
        lineStatus = (uint32_t) UART_readLineStatus(appPrms->baseAddr);
        maxTrialCount--;
    }
    while (((uint32_t) (UART_LSR_TX_SR_E_MASK | UART_LSR_TX_FIFO_E_MASK) !=
            ((uint32_t) (UART_LSR_TX_SR_E_MASK |
                       UART_LSR_TX_FIFO_E_MASK) & lineStatus))
           && (0U < maxTrialCount));

    if (maxTrialCount > 0U)
    {
        while ((tempChunksize > 0U) && (writeSizeRemaining > 0U))
        {
            /* Writing to the FIFO */
            UART_putChar(appPrms->baseAddr, *tmpBuf);
            tmpBuf++;
            writeSizeRemaining--;
            tempChunksize--;
        }
    }

    /* Returns the size actually written */
    return (writeSize - writeSizeRemaining);
}

static void App_uartRxFifoRead(UART_AppPrms *appPrms, uint8_t *readBuf)
{
    uint32_t isRxReady   = (uint32_t) FALSE;

    isRxReady = (uint32_t) UART_checkCharsAvailInFifo(appPrms->baseAddr);
    while ((uint32_t) TRUE == isRxReady)
    {
        /* once the data is ready, read from the FIFO */
        *readBuf = (uint8_t) UART_getCharFifo(appPrms->baseAddr, readBuf);
        readBuf++;
        appPrms->readCount++;
        isRxReady = UART_checkCharsAvailInFifo(appPrms->baseAddr);
    }

    return;
}

static void App_uartRx(UART_AppPrms *appPrms, uint8_t *readBuf)
{
    appPrms->readBuf             = readBuf;
    appPrms->readCount           = 0U;

    /* Read all data from RX FIFO */
    App_uartRxFifoRead(appPrms, appPrms->readBuf);
    if(appPrms->readCount > 0U)
    {
        if(appPrms->readCount > APP_UART_BUFSIZE)
        {
            /* Rx buffer overflow */
            appPrms->rxOverflow = TRUE;
        }
    }

    return;
}

static void App_uartInit(UART_AppPrms *appPrms)
{

    /* Get UART Instance Base Address */
    appPrms->baseAddr = UART_getBaseAddr(gUartHandle[CONFIG_UART_CONSOLE]);
    DebugP_assert(appPrms->baseAddr != 0U); /* UART baseAddr Invalid!! */

    /* Reset other variables */
    appPrms->writeBuf           = NULL;
    appPrms->writeSizeRemaining = 0U;
    appPrms->writeCount         = 0U;
    appPrms->readBuf            = NULL;
    appPrms->readCount          = 0U;
    appPrms->rxOverflow         = FALSE;

    return;
}
