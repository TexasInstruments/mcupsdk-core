/*
 *  Copyright (C) 2022 Texas Instruments Incorporated
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

/*
 *   This example configures the LIN module in controller mode for internal
 *   loopback without interrupts. The module is setup to perform 8 data
 *   transmissions with different transmit IDs and varying transmit data.
 *   Waits for reception of an ID header. The received data is then checked
 *   for accuracy.
 *
 *   External Connections:
 *   - None.
 *
 *  Watch Variables:
 *   - txData - An array with the data being sent
 *   - rxData - An array with the data that was received
 *   - result - The example completion status (PASS = 0xABCD, FAIL = 0xFFFF)
 *
 */

/* Included Files */
#include <kernel/dpl/DebugP.h>
#include <drivers/lin.h>
#include <ti_drivers_config.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/* Defines */
#define FRAME_LENGTH        (0x8)
#define LIN_ID              (0x10)

#define LIN_PASS            (0xABCD)
#define LIN_FAIL            (0xFFFF)


#define APP_LIN_BASE_ADDR   (CONFIG_LIN1_BASE_ADDR)

/* Globals */
volatile uint32_t level0Count = 0;
volatile uint32_t level1Count = 0;
volatile uint32_t vectorOffset = 0;
uint16_t result;
uint16_t txData[8] = {0x11, 0x34, 0x56, 0x78, 0x9A, 0xAB, 0xCD, 0xEF};
uint16_t rxData[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

/* lin_loopback_polling_main */
void lin_loopback_polling_main(void)
{
    uint32_t                i, dataIndex;
    uint16_t                txID, error = 0;

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    DebugP_log("[LIN] Loopback Polling mode, application started ...\r\n");

    /* Enable parity check */
    LIN_enableParity(APP_LIN_BASE_ADDR);

    /*
     * Perform 8 data transmissions with different transmit IDs and varying
     * number of bytes transmitted. Received data is checked for correctness.
     */
    for(i = 1 ; i <= FRAME_LENGTH; i++)
    {
        vectorOffset = 0;

        /* Create a new transmit ID and update with parity bits */
        txID = (LIN_ID + i);
        txID = LIN_generateParityID(txID);

        /*
         * Increment the value of the first 8-bits of the transmitted
         * message data
         */
        txData[0]++;

        /* Reset values in receive buffer array */
        for(dataIndex=0; dataIndex < 8; dataIndex++)
        {
            rxData[dataIndex] = 0xFF;
        }

        /*
         * Set the frame length (number of bytes to be transmitted)
         */
        LIN_setFrameLength(APP_LIN_BASE_ADDR, i);

        /*
         * This places data into the transmit buffer.
         * No ID or data is placed on the bus and transmitted yet.
         */
        LIN_sendData(APP_LIN_BASE_ADDR, txData);

        /*
         * Set the message ID to initiate a header transmission.
         * This causes the ID to be written to the bus followed by the
         * data in the transmit buffers.
         */
        LIN_setIDByte(APP_LIN_BASE_ADDR, txID);

        /* Wait until Transmit buffer is empty and has completed transmission */
        while(!LIN_isTxBufferEmpty(APP_LIN_BASE_ADDR));

        /* Wait for the Reception */
        while(!LIN_isRxMatch(APP_LIN_BASE_ADDR));

        LIN_clearInterruptStatus(APP_LIN_BASE_ADDR,LIN_INT_ID);

        /* Read the received data in the receive buffers */
        LIN_getData(APP_LIN_BASE_ADDR, rxData);

        /* Verify the transmitted data matches the received data */
        for (dataIndex=0; dataIndex < i; dataIndex++)
        {
            if (rxData[dataIndex] != txData[dataIndex])
            {
                error++;
            }
        }
    }

    /* Check if any data errors occurred */
    if(error == 0)
    {
        result = LIN_PASS;
        DebugP_log("All tests have passed!!\r\n");
    }
    else
    {
        result = LIN_FAIL;
        DebugP_log("Test FAILED!!\r\n");
    }

    Board_driversClose();
    Drivers_close();
}
