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
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/TaskP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/*
 * This example demonstrate the UART low latency API and user managed interrupt
 * service routine.
 * This example receives 8 characters and echos back the same.
 * The application ends when the user types 8 characters.
 * Initially the application sets a buffer to receive data and enables the
 * RX interrupt. When RX ISR is triggered the ISR reads the data from UART
 * FIFO and write to the RX buffer and sets the RX buffer count.
 *
 * In the main context, the application checks if RX buffer has any data
 * and if so reads the data from it and copy it to TX buffer and initiate the
 * UART TX (echo).
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
    uint32_t                txTrigLvl;
    /**< TX FIFO Trigger Level */
    uint32_t                rxTrigLvl;
    /**< RX FIFO Trigger Level */

    /*
     * UART objects
     */
    HwiP_Object             hwiObject;
    /**< HWI object */
    SemaphoreP_Object       writeDoneSem;
    /**< Write done semaphore */

    /*
     * UART write state variables - set for every transfer
     */
    const uint8_t          *writeBuf;
    /**< Buffer data pointer */
    volatile uint32_t       writeCount;
    /**< Number of Chars sent */
    volatile uint32_t       writeSizeRemaining;
    /**< Chars remaining in buffer */
    volatile uint32_t       txDataSent;
    /**< Flag to indicate all the data written to the TX FIFO */

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
static void App_uartUserISR(void *arg);
static void App_uartInit(UART_AppPrms *appPrms);
static void App_uartDeInit(UART_AppPrms *appPrms);

void uart_echo_low_latency_interrupt(void *args)
{
    UART_AppPrms            *appPrms = &gAppPrms;
    const char              *printStr = "This is uart low latency test in interrupt mode, Receives 8 characters then echo's back and exits..\r\n";
    const char              *printExitStr = "\r\nAll tests have passed!!\r\n";
    uint32_t                 numCharsRead, i, charRecvdCnt;

    Drivers_open();
    Board_driversOpen();

    DebugP_log("[UART] Echo Low Latency interrupt mode example started ...\r\n");

    /* Init App Prms */
    App_uartInit(appPrms);

    /* Send entry string and wait for Tx completion */
    App_uartTx(appPrms, (const uint8_t *) printStr, strlen(printStr));

    /* Init Rx Prms and Enable Rx Interrupt */
    App_uartRx(appPrms, &gAppUartRxBuffer[0U]);

    charRecvdCnt = 0U;
    /* Echo whatever is read */
    while(1)
    {
        if(appPrms->readCount > 0U)
        {
            uintptr_t   key;

            /* Enter Critical section, disable interrupt */
            key = HwiP_disable();

            /* copy data from RX buf to TX buf */
            numCharsRead = appPrms->readCount;
            for(i = 0U; i < numCharsRead; i++)
            {
                gAppUartTxBuffer[i] = gAppUartRxBuffer[i];
            }
            appPrms->readCount = 0U; /* Reset variable so that RX can start from first */

            /* End critical section */
            HwiP_restore(key);

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

        if (charRecvdCnt >= APP_UART_BUFSIZE)
        {
            break;
        }

        TaskP_yield();
    }

    /* Disable Rx Interrupt */
    UART_intrDisable(appPrms->baseAddr, UART_INTR_RHR_CTI);

    /* Send exit string and wait for Tx completion */
    App_uartTx(appPrms, (const uint8_t *) printExitStr, strlen(printExitStr));

    DebugP_log("All tests have passed!!\r\n");

    App_uartDeInit(appPrms);
    Board_driversClose();
    Drivers_close();

    return;
}

static void App_uartTx(UART_AppPrms   *appPrms,
                       const uint8_t  *writeBuf,
                       uint32_t        writeSize)
{
    appPrms->writeBuf           = (const uint8_t *)writeBuf;
    appPrms->writeSizeRemaining = writeSize;
    appPrms->writeCount         = 0U;
    appPrms->txDataSent         = FALSE;

    /* Enable THR interrupt */
    UART_intrEnable(appPrms->baseAddr, UART_INTR_THR);

    SemaphoreP_pend(&appPrms->writeDoneSem, SystemP_WAIT_FOREVER);

    return;
}

static void App_uartRx(UART_AppPrms *appPrms, uint8_t *readBuf)
{
    appPrms->readBuf           = readBuf;
    appPrms->readCount         = 0U;

    /* Enable RHR interrupt */
    UART_intrEnable(appPrms->baseAddr, UART_INTR_RHR_CTI);

    return;
}

static void App_uartUserISR(void *arg)
{
    UART_AppPrms  *appPrms     = (UART_AppPrms *) arg;
    uint32_t       intrType;

    intrType = UART_getIntrIdentityStatus(appPrms->baseAddr);

    /* Check RX FIFO threshold is set */
    if((intrType & UART_INTID_RX_THRES_REACH) == UART_INTID_RX_THRES_REACH)
    {
        uint32_t       readSuccess;
        uint8_t       *readBuf, readChar;

        /* Read all data from RX FIFO */
        readBuf = (uint8_t *)appPrms->readBuf + appPrms->readCount;
        while(1)
        {
            readSuccess = UART_getChar(appPrms->baseAddr, &readChar);
            if(readSuccess == TRUE)
            {
                if(appPrms->readCount >= APP_UART_BUFSIZE)
                {
                    /* Rx buffer overflow */
                    appPrms->rxOverflow = TRUE;
                }
                else
                {
                    /* Store data in buffer */
                    *readBuf = readChar;
                    readBuf++;
                    appPrms->readCount++;
                }
            }
            else
            {
                break;      /* No data left in FIFO */
            }
        }

        if(appPrms->rxOverflow == TRUE)
        {
            /* Stop processing further data */
            UART_intrDisable(appPrms->baseAddr, UART_INTR_RHR_CTI);
        }
    }

    /* Check TX FIFO threshold is set */
    if((intrType & UART_INTID_TX_THRES_REACH) == UART_INTID_TX_THRES_REACH)
    {
        uint32_t       numBytesToTransfer;
        const uint8_t *writeBuf;

        if(appPrms->writeSizeRemaining > 0U)
        {
            numBytesToTransfer = appPrms->writeSizeRemaining;
            if(numBytesToTransfer > appPrms->txTrigLvl)
            {
                /* Write only threshold level of data */
                numBytesToTransfer = appPrms->txTrigLvl;
            }
            appPrms->writeSizeRemaining -= numBytesToTransfer;

            /* Send characters upto FIFO threshold level or until done */
            writeBuf = appPrms->writeBuf + appPrms->writeCount;
            while(numBytesToTransfer != 0U)
            {
                UART_putChar(appPrms->baseAddr, *writeBuf);
                writeBuf++;
                numBytesToTransfer--;
                appPrms->writeCount++;
            }

            if(appPrms->writeSizeRemaining == 0U)
            {
                /* Write complete  - disable TX interrupts */
                appPrms->txDataSent = TRUE;
                UART_intrDisable(appPrms->baseAddr, UART_INTR_THR);
                UART_intr2Enable(appPrms->baseAddr, UART_INTR2_TX_EMPTY);
            }
        }
        else
        {
            /* Disable interrupt */
            UART_intrDisable(appPrms->baseAddr, UART_INTR_THR);
        }
    }

    /* Check if TX FIFO is empty */
    if(appPrms->txDataSent == TRUE)
    {
        intrType = UART_getIntr2Status(appPrms->baseAddr);
        if((intrType & UART_INTR2_TX_EMPTY) != 0U)
        {
            UART_intr2Disable(appPrms->baseAddr, UART_INTR2_TX_EMPTY);
            appPrms->txDataSent = FALSE;
            SemaphoreP_post(&appPrms->writeDoneSem);
        }
    }

    return;
}

static void App_uartInit(UART_AppPrms *appPrms)
{
    int32_t         status;
    HwiP_Params     hwiPrms;

    /* Get UART Instance Base Address */
    appPrms->baseAddr = UART_getBaseAddr(gUartHandle[CONFIG_UART_CONSOLE]);
    DebugP_assert(appPrms->baseAddr != 0U); /* UART baseAddr Invalid!! */
    appPrms->rxTrigLvl          = gUartParams[CONFIG_UART_CONSOLE].rxTrigLvl;
    appPrms->txTrigLvl          = gUartParams[CONFIG_UART_CONSOLE].txTrigLvl;

    /* Reset other variables */
    appPrms->writeBuf           = NULL;
    appPrms->writeSizeRemaining = 0U;
    appPrms->writeCount         = 0U;
    appPrms->txDataSent         = FALSE;
    appPrms->readBuf            = NULL;
    appPrms->readCount          = 0U;
    appPrms->rxOverflow         = FALSE;

    status = SemaphoreP_constructBinary(&appPrms->writeDoneSem, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Register interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = gUartParams[CONFIG_UART_CONSOLE].intrNum;
    hwiPrms.priority    = gUartParams[CONFIG_UART_CONSOLE].intrPriority;
    hwiPrms.callback    = &App_uartUserISR;
    hwiPrms.args        = (void *) appPrms;
    status              = HwiP_construct(&appPrms->hwiObject, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);

    return;
}

static void App_uartDeInit(UART_AppPrms *appPrms)
{
    HwiP_destruct(&appPrms->hwiObject);
    SemaphoreP_destruct(&appPrms->writeDoneSem);

    return;
}