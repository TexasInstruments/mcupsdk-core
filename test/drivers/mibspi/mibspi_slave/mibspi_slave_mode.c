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
 *  \file   mibspi_slave_mode.c
 *
 *  \brief This file contains the Slave  application which demonstrates
 *  data transfer in slave mode. The test case verifies the slave write,
 *  slave read and slave write and read with PC host application
 *
 *  Connect micro USB cable to FTDI_USB port of the board.
 *  The PC side app is always in SPI master mode. So run the Target side
 *  app before running PC Host app.
 *
 *  MibSPI is configured in Slave mode.
 *  Word Length tested is 16 bits with size 12000 bytes.
 *
 * The application supports three categories to verify the slave
 * functionality of the target.Below are the options displayed on the
 * UART console which is connected to the evm.
 * Enter the Testcase to Run
 *  1 : Slave SPI write and PC read test
 *  2 : Slave SPI read and PC write test
 *  3 : Slave SPI write and read test
 *  4 : Exit
 *
 * Below are the options for the PC host application.
 * spi_receiver [Mode] [Num Frames] [Port Num] [Mode]
 *
 * -1 : Slave SPI read test
 * -2 : Slave SPI write test
 * -3 : Slave SPI read and write test
 *
 * [Num Frames] is how many iterations you want to send the data. It is 1 for the evm.
 *
 * [Port Num] Is the dev id number will be 0 for the evm.
 *
 * ex:
 * To execute Slave SPI write and PC read test.
 * Enter 1 on the UART console which is connected to the evm.
 *
 * go to the docs_src\docs\test_code\am273x_awr294x\mibspi_slavemode_test path
 * Enter following command on the command prompt
 *
 * spi_receiver -1 1 0
 *
 */

#include <kernel/dpl/DebugP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <unity.h>
#include <string.h>
#include <stdio.h>
#include <drivers/pinmux.h>

/* Number of Word count */
#define APP_MIBSPI_MSGSIZE                   (12000U)
#define MIBSPI_BLOCK_SIZE                    (MIBSPI_RAM_MAX_ELEM*2)


uint8_t     gMibspiTxBuffer[APP_MIBSPI_MSGSIZE];
uint8_t     gMibspiRxBuffer[APP_MIBSPI_MSGSIZE];

void test_mibspi_slave_read(void *args);
void test_mibspi_slave_write(void *args);
void test_mibspi_slave_write_read(void *args);

/* Define the GPIO's for the MIBSPI interrupt generation.
 * Slave test doesn’t work if we configure only pinmux of mibspi and HOSTIRQ signal.
 * The slave application goes into idle mode waiting for the receive interrupt completion.
 * Configure the pinmux ball no. B2,A2,C1 and B1 (These are dedicated for the CAN module) as GPIO then
 * slave example works fine.
 * configuration of MIBSPIA HOST IRQ signal (ball no. G18) doesn't have any effect on the functionality of the slave mode.
 * Refer MCUSDK-1973
 */
static Pinmux_PerCfg_t gGpioPinmuxData[] =
{
    {
        PIN_PAD_AG,
        (PIN_MODE(0) | PIN_PULL_DISABLE)
    },
    {
        PIN_PAD_AF,
        (PIN_MODE(0) | PIN_PULL_DISABLE)
    },
    {
        PIN_PAD_AD,
        (PIN_MODE(0) | PIN_PULL_DISABLE)
    },
    {
        PIN_PAD_AE,
        (PIN_MODE(0) | PIN_PULL_DISABLE)
    },
    {PINMUX_END, PINMUX_END}
};

void test_mibspi(void *args)
{
    uint32_t testNum;

    Drivers_open();
    Board_driversOpen();

    Pinmux_config(gGpioPinmuxData, PINMUX_DOMAIN_ID_MAIN);

    UNITY_BEGIN();

    while (1)
    {
        DebugP_log("Enter the Testcase to Run:\r\n");
        DebugP_log("1 : Slave SPI write and PC read test \r\n");
        DebugP_log("2 : Slave SPI read and PC write test \r\n");
        DebugP_log("3 : Slave SPI write and read test \r\n");
        DebugP_log("4 : Exit \r\n");

        DebugP_scanf("%u",&testNum);

        if (testNum == 1)
        {
            RUN_TEST(test_mibspi_slave_write, 1951, NULL);
        }
        else if (testNum == 2)
        {
            RUN_TEST(test_mibspi_slave_read, 1969, NULL);
        }
        else if (testNum == 3)
        {
            RUN_TEST(test_mibspi_slave_write_read, 1970, NULL);
        }
        else
        {
            DebugP_log("Exiting App\n");
            break;
        }
    }

    UNITY_END();

    Board_driversClose();
    Drivers_close();

    return;
}

void test_mibspi_slave_write(void *args)
{
    uint32_t            i;
    uint32_t            u32RemainingBytes;
    uint32_t            u32ChunkSize;
    int32_t             transferOK;
    int32_t             status = SystemP_SUCCESS;
    MIBSPI_Transaction  spiTransaction;
    uint8_t             *pu8BuffPtr;

    DebugP_log("[MIBSPI] Slave Write Test Started...\r\n\n");

    /* Memfill buffers */
    for(i = 0U; i < APP_MIBSPI_MSGSIZE; i++)
    {
        gMibspiTxBuffer[i] = i;
    }

    u32RemainingBytes = APP_MIBSPI_MSGSIZE;
    pu8BuffPtr        = &gMibspiTxBuffer[0];

    CacheP_wbInv((void *)pu8BuffPtr, APP_MIBSPI_MSGSIZE, CacheP_TYPE_ALLD);

    spiTransaction.slaveIndex   = 0U;
    spiTransaction.rxBuf        = NULL;
    spiTransaction.arg          = NULL;

    if(SystemP_SUCCESS == status)
    {
        /* Full chunk of SPI transfer */
        while(u32RemainingBytes > 0U)
        {
            if(u32RemainingBytes >= MIBSPI_BLOCK_SIZE)
            {
                u32ChunkSize = MIBSPI_BLOCK_SIZE;
            }
            else
            {
                u32ChunkSize = u32RemainingBytes;
            }
            spiTransaction.count        = u32ChunkSize;
            spiTransaction.txBuf        = (void *)pu8BuffPtr;

            transferOK = MIBSPI_transfer(gMibspiHandle[CONFIG_MIBSPI0], &spiTransaction);
            if((SystemP_SUCCESS != transferOK) ||
               (MIBSPI_TRANSFER_COMPLETED != spiTransaction.status))
            {
                u32RemainingBytes = 0;
                DebugP_assert(FALSE); /* MIBSPI transfer failed!! */
            }
            else
            {
                /* Prepare next chunk */
                pu8BuffPtr         += u32ChunkSize;
                u32RemainingBytes -= u32ChunkSize;
            }
        }
    }

    if(SystemP_SUCCESS == status)
    {
        DebugP_log("All tests have passed!!\r\n");
    }

    return ;
}

void test_mibspi_slave_read(void *args)
{
    uint32_t            i;
    uint32_t            u32RemainingBytes;
    uint32_t            u32ChunkSize;
    uint32_t            u32NbrErr = 0;
    int32_t             transferOK;
    int32_t             status = SystemP_SUCCESS;
    MIBSPI_Transaction  spiTransaction;
    uint8_t             *pu8BuffPtr;

    DebugP_log("[MIBSPI] Slave Read Test Started...\r\n\n");

    /* Memset Rx buffer */
    memset(&gMibspiRxBuffer, 0, APP_MIBSPI_MSGSIZE);

    u32RemainingBytes = APP_MIBSPI_MSGSIZE;
    pu8BuffPtr        = &gMibspiRxBuffer[0];

    CacheP_wbInv((void *)pu8BuffPtr, APP_MIBSPI_MSGSIZE, CacheP_TYPE_ALLD);

    spiTransaction.slaveIndex   = 0U;
    spiTransaction.txBuf        = NULL;
    spiTransaction.arg          = NULL;

    if(SystemP_SUCCESS == status)
    {
        /* Full chunk of SPI transfer */
        while(u32RemainingBytes > 0U)
        {
            if(u32RemainingBytes >= MIBSPI_BLOCK_SIZE)
            {
                u32ChunkSize = MIBSPI_BLOCK_SIZE;
            }
            else
            {
                u32ChunkSize = u32RemainingBytes;
            }
            spiTransaction.count        = u32ChunkSize;
            spiTransaction.rxBuf        = (void *)pu8BuffPtr;

            transferOK = MIBSPI_transfer(gMibspiHandle[CONFIG_MIBSPI0], &spiTransaction);
            if((SystemP_SUCCESS != transferOK) ||
               (MIBSPI_TRANSFER_COMPLETED != spiTransaction.status))
            {
                u32RemainingBytes = 0;
                DebugP_assert(FALSE); /* MIBSPI transfer failed!! */
            }
            else
            {
                /* Prepare next chunk */
                pu8BuffPtr         += u32ChunkSize;
                u32RemainingBytes -= u32ChunkSize;
            }
        }
    }

    for(i = 0U; i < APP_MIBSPI_MSGSIZE; i++)
    {
        if(gMibspiRxBuffer[i] != (i & 0xFF))
        {
            u32NbrErr++;
            status = SystemP_FAILURE;
        }
    }

    if(SystemP_SUCCESS == status)
    {
        DebugP_log("All tests have passed!!\r\n");
    }
    else
    {
        DebugP_log("Some tests have failed!!\r\n");
        DebugP_log("Number of error %u !!\r\n",u32NbrErr);
    }

    return ;
}

void test_mibspi_slave_write_read(void *args)
{
    uint32_t            i;
    uint32_t            u32RemainingBytes;
    uint32_t            u32ChunkSize;
    uint32_t            u32NbrErr = 0;
    int32_t             transferOK;
    int32_t             status = SystemP_SUCCESS;
    MIBSPI_Transaction  spiTransaction;
    uint8_t             *pu8WrBuffPtr;
    uint8_t             *pu8RdBuffPtr;

    DebugP_log("[MIBSPI] Slave Write and Read Test Started...\r\n\n");

    /* Memfill buffers */
    for(i = 0U; i < APP_MIBSPI_MSGSIZE; i++)
    {
        gMibspiTxBuffer[i] = i;
        gMibspiRxBuffer[i] = 0;
    }

    u32RemainingBytes   = APP_MIBSPI_MSGSIZE;
    pu8WrBuffPtr        = &gMibspiTxBuffer[0];
    pu8RdBuffPtr        = &gMibspiRxBuffer[0];

    CacheP_wbInv((void *)pu8WrBuffPtr, APP_MIBSPI_MSGSIZE, CacheP_TYPE_ALLD);
    CacheP_wbInv((void *)pu8RdBuffPtr, APP_MIBSPI_MSGSIZE, CacheP_TYPE_ALLD);

    spiTransaction.slaveIndex   = 0U;
    spiTransaction.txBuf        = NULL;
    spiTransaction.arg          = NULL;

    if(SystemP_SUCCESS == status)
    {
        /* Full chunk of SPI transfer */
        while(u32RemainingBytes > 0U)
        {
            if(u32RemainingBytes >= MIBSPI_BLOCK_SIZE)
            {
                u32ChunkSize = MIBSPI_BLOCK_SIZE;
            }
            else
            {
                u32ChunkSize = u32RemainingBytes;
            }
            spiTransaction.count        = u32ChunkSize;
            spiTransaction.txBuf        = (void *)pu8WrBuffPtr;
            spiTransaction.rxBuf        = (void *)pu8RdBuffPtr;

            transferOK = MIBSPI_transfer(gMibspiHandle[CONFIG_MIBSPI0], &spiTransaction);
            if((SystemP_SUCCESS != transferOK) ||
               (MIBSPI_TRANSFER_COMPLETED != spiTransaction.status))
            {
                u32RemainingBytes = 0;
                DebugP_assert(FALSE); /* MIBSPI transfer failed!! */
            }
            else
            {
                /* Prepare next chunk */
                pu8WrBuffPtr         += u32ChunkSize;
                pu8RdBuffPtr         += u32ChunkSize;
                u32RemainingBytes -= u32ChunkSize;
            }
        }
    }

    for(i = 0U; i < APP_MIBSPI_MSGSIZE; i++)
    {
        if(gMibspiRxBuffer[i] != (i & 0xFF))
        {
            u32NbrErr++;
            status = SystemP_FAILURE;
        }
    }

    if(SystemP_SUCCESS == status)
    {
        DebugP_log("All tests have passed!!\r\n");
    }
    else
    {
        DebugP_log("Some tests have failed!!\r\n");
        DebugP_log("Number of error %u !!\r\n",u32NbrErr);
    }

    return ;
}

void setUp(void)
{
}

void tearDown(void)
{
}
