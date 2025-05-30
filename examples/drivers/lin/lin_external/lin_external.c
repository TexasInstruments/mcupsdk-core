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
 *   This example configures the LIN module in LIN mode for The LIN module
 *   performs sends a data over LIN_1 which is connected to PC via PLIN_USB.
 *   The data is sent over LIN at 19200 baud rate.
 *
 *   Make sure to move the SW9 to ON. This will turn ON the LIN Transceiver.
 *   I2C_2 is configured to set the level shifter so the LIN_MUX_SEL is set as LOW.
 *   So the Transceiver is set as LIN Mode.
 *
 *   External Connections :
 *    - PLIN-USB connected to Windows Machine.
 *
 *   Watch Variables :
 *
 */

/* Included Files */
#include <kernel/dpl/DebugP.h>
#include <drivers/lin.h>
#include <drivers/i2c.h>
#include <ti_drivers_config.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/* Defines */
#define FRAME_LENGTH        (0x8)
#define LIN_ID              (0x10)

#define LIN_PASS            (0xABCD)
#define LIN_FAIL            (0xFFFF)

#define I2C_TARGET_ADDRESS   (0x20U)
#define I2C_POLARITY_INV    (0x5U)

#define APP_LIN_BASE_ADDR   (CONFIG_LIN1_BASE_ADDR)

/* Globals */
uint16_t result;
uint16_t txData[8] = {0x12, 0x34, 0x56, 0x78, 0x9A, 0xAB, 0xCD, 0xEF};
uint16_t rxData[8] = {0x00};

/* lin_external_main */
void lin_external_main(void)
{
    uint32_t                i;
    uint16_t                txID, error = 0;

#if !defined(SOC_AM261X)
    int32_t                 status;
    uint8_t                 txBuffer[1];
    I2C_Handle              i2cHandle;
    I2C_Transaction         i2cTransaction;
#endif

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

#if !defined(SOC_AM261X)
    i2cHandle = gI2cHandle[CONFIG_I2C2];
#endif

    DebugP_log("[LIN] LIN mode external, application started ...\r\n");

    /* Enable multi-buffer mode */
    LIN_enableMultibufferMode(APP_LIN_BASE_ADDR);

    LIN_setSyncFields(APP_LIN_BASE_ADDR, 5U, 3U);

    /* Enable Fixed baud rate mode */
    LIN_disableAutomaticBaudrate(APP_LIN_BASE_ADDR);

    /* Reaching the Baud of 19200 */
    LIN_setBaudRatePrescaler(APP_LIN_BASE_ADDR, 624U, 0U);
    LIN_setMaximumBaudRate(APP_LIN_BASE_ADDR, 100000000U);

    /* Set Mask ID so TX/RX match will always happen */
    LIN_setRxMask(APP_LIN_BASE_ADDR, 0xFFU);

    /* Enable transfer of data to the shift registers  */
    LIN_enableDataTransmitter(APP_LIN_BASE_ADDR);
    LIN_enableDataReceiver(APP_LIN_BASE_ADDR);

    /* Enable the triggering of checksum compare on extended frames */
    LIN_triggerChecksumCompare(APP_LIN_BASE_ADDR);

#if !defined(SOC_AM261X)
    DebugP_log("[I2C] LIN Volatage Level Shifter started ...\r\n");

    I2C_Transaction_init(&i2cTransaction);
    i2cTransaction.writeBuf   = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.targetAddress = I2C_TARGET_ADDRESS;
    txBuffer[0] = I2C_POLARITY_INV;
    status = I2C_transfer(i2cHandle, &i2cTransaction);
    DebugP_assert(status == SystemP_SUCCESS);
#endif

    /*
     * Perform 8 data transmissions with different transmit IDs and varying
     * number of bytes transmitted. Received data is checked for correctness.
     */
    for(i = 1 ; i <= FRAME_LENGTH; i++)
    {
        /* Create a new transmit ID and update with parity bits */
        txID = (LIN_ID + i);
        txID = LIN_generateParityID(txID);

        /* Set the tx Mask value to txID*/
        LIN_setTxMask(APP_LIN_BASE_ADDR, txID);

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

        /* Poll for Tx completion interrupt */
        while(!( (LIN_getInterruptStatus(APP_LIN_BASE_ADDR)) & LIN_FLAG_TXEMPTY));

        DebugP_log("[LIN] : New Data Sent = %x\r\n", txData[i-1]);
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
