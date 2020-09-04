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
 *  \file   mibspi_performance_16bit.c
 *
 *  \brief This file contains the Master application which demonstrates
 *         data transfer in master mode with performance measurment.
 *
 *         MibSPI is configured in Digital loopback mode.
 *         Word Length tested is 16 bits.
 *         SPI CLK Frequency used is 40 MHZ.
 *         Number of Words is 1024.
 *
 */

#include <kernel/dpl/DebugP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <unity.h>
#include <string.h>

/* Number of Word count */
#define APP_MIBSPI_MSGSIZE                   (1024U)
#define APP_MIBSPI_TRANSFER_LOOPCOUNT        (10U)


uint8_t     gMibspiTxBuffer[APP_MIBSPI_MSGSIZE];
uint8_t     gMibspiRxBuffer[APP_MIBSPI_MSGSIZE];

void test_mibspi_performance_16bit(void *args);


void test_mibspi(void *args)
{
    Drivers_open();
    Board_driversOpen();

    UNITY_BEGIN();

    RUN_TEST(test_mibspi_performance_16bit, 1895, NULL);

    UNITY_END();

    Board_driversClose();
    Drivers_close();

    return;
}

void test_mibspi_performance_16bit(void *args)
{
    uint32_t            i, j;
    uint32_t            dataLength, dataWidth, bitRate;
    int32_t             transferOK;
    int32_t             status = SystemP_SUCCESS;
    uint64_t            startTimeInUSec = 0, elapsedTimeInUsecs = 0;
    MIBSPI_Transaction  spiTransaction;
    MIBSPI_LoopBackType loopback;

    DebugP_log("[MIBSPI] Performance Example Started...\r\n\n");
     
    /* Enable digital loopback */
    loopback = MIBSPI_LOOPBK_DIGITAL;

    /* Memfill buffers */
    for(i = 0U; i < APP_MIBSPI_MSGSIZE; i++)
    {
        gMibspiTxBuffer[i] = i;
        gMibspiRxBuffer[i] = 0U;
    }
    
    CacheP_wbInv((void *)gMibspiTxBuffer, APP_MIBSPI_MSGSIZE, CacheP_TYPE_ALLD);
    CacheP_wbInv((void *)gMibspiRxBuffer, APP_MIBSPI_MSGSIZE, CacheP_TYPE_ALLD);
    
    /* Initiate transfer */
    spiTransaction.count        = APP_MIBSPI_MSGSIZE;
    spiTransaction.txBuf        = (void *)gMibspiTxBuffer;
    spiTransaction.rxBuf        = (void *)gMibspiRxBuffer;
    spiTransaction.slaveIndex   = 0U;
    spiTransaction.arg          = NULL;
    
     /* Number of Word Count */
    dataWidth  = gMibspiConfig[0U].object->params.dataSize;
    bitRate    = gMibspiConfig[0U].object->params.u.masterParams.bitRate;
    dataLength = APP_MIBSPI_MSGSIZE;
  
    status = MIBSPI_enableLoopback(gMibspiHandle[CONFIG_MIBSPI0],loopback);
    if(SystemP_SUCCESS != status)
    {
        DebugP_log("Check SPI instance mode of operation and loopback type\r\n");
    }
     
    if(SystemP_SUCCESS == status)
    {    
        /* Initiate transfer */
        startTimeInUSec = ClockP_getTimeUsec(); 
        for(j = 0U; j < APP_MIBSPI_TRANSFER_LOOPCOUNT; j++)
        {     
            transferOK = MIBSPI_transfer(gMibspiHandle[CONFIG_MIBSPI0], &spiTransaction);
            if((SystemP_SUCCESS != transferOK) ||
               (MIBSPI_TRANSFER_COMPLETED != spiTransaction.status))
            {
                DebugP_assert(FALSE); /* MIBSPI transfer failed!! */
            }
        }
    }
    
    if(SystemP_SUCCESS == status)
    { 
        elapsedTimeInUsecs = ClockP_getTimeUsec() - startTimeInUSec;
        
        DebugP_log("----------------------------------------------------------\r\n");
        DebugP_log("MibSPI Clock %d Hz\r\n", bitRate);
        DebugP_log("----------------------------------------------------------\r\n");
        DebugP_log("Data Width \tData Length \tTransfer Time (micro sec)\r\n");
        DebugP_log("%u\t\t%u\t\t%5.2f\r\n", dataWidth, dataLength,
                            (float)elapsedTimeInUsecs / APP_MIBSPI_TRANSFER_LOOPCOUNT);
        DebugP_log("----------------------------------------------------------\r\n\n");
        DebugP_log("All tests have passed!!\r\n");
    }

    return ;
 }

void setUp(void)
{
}

void tearDown(void)
{
}
