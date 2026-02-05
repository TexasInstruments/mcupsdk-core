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
void lin_responder_lld_main(void)
{
    uint32_t                i;
    uint16_t                txID;

    int32_t                 status;
    uint8_t                 txBuffer[1];
    I2C_Handle              i2cHandle;
    I2C_Transaction         i2cTransaction;

    uint32_t                intrStatus = 0x00U;
    uint32_t                id = 0;

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    i2cHandle = gI2cHandle[CONFIG_I2C2];

    DebugP_log("[LIN]  LIN Responder LLD Application Started ...\r\n");

    // ------------------------------------------------------------------------
    // LIN INIT
    // ------------------------------------------------------------------------

    LIN_initModule(APP_LIN_BASE_ADDR);

    LIN_enterSoftwareReset(APP_LIN_BASE_ADDR);

    LIN_disableGlobalInterrupt(APP_LIN_BASE_ADDR, LIN_INTERRUPT_LINE0);
    LIN_disableGlobalInterrupt(APP_LIN_BASE_ADDR, LIN_INTERRUPT_LINE1);

    LIN_clearGlobalInterruptStatus(APP_LIN_BASE_ADDR, LIN_INTERRUPT_LINE0);
    LIN_clearGlobalInterruptStatus(APP_LIN_BASE_ADDR, LIN_INTERRUPT_LINE1);

    LIN_disableSCIMode(APP_LIN_BASE_ADDR);

    LIN_setLINMode(APP_LIN_BASE_ADDR, LIN_MODE_LIN_RESPONDER);

    /* Enable Fixed baud rate mode */
    LIN_enableAutomaticBaudrate(APP_LIN_BASE_ADDR);

    LIN_setCommMode(APP_LIN_BASE_ADDR, LIN_COMM_LIN_USELENGTHVAL);

    LIN_setDebugSuspendMode(APP_LIN_BASE_ADDR, LIN_DEBUG_COMPLETE);

    LIN_setMessageFiltering(APP_LIN_BASE_ADDR, LIN_MSG_FILTER_IDRESPONDER);

    LIN_setTxMask(APP_LIN_BASE_ADDR, 0xFFU);
    LIN_setRxMask(APP_LIN_BASE_ADDR, 0xFFU);

    LIN_setChecksumType(APP_LIN_BASE_ADDR, LIN_CHECKSUM_ENHANCED);

    LIN_disableIntLoopback(APP_LIN_BASE_ADDR);
    LIN_disableExtLoopback(APP_LIN_BASE_ADDR);

    LIN_enableMultibufferMode(APP_LIN_BASE_ADDR);

    LIN_enableParity(APP_LIN_BASE_ADDR);

    // Clock 96MHz, Baudrate 19200
    LIN_setBaudRatePrescaler(APP_LIN_BASE_ADDR, 311U, 8U);

    LIN_setMaximumBaudRate(APP_LIN_BASE_ADDR, 96000000, 19200);

    LIN_enableDataTransmitter(APP_LIN_BASE_ADDR);
    LIN_enableDataReceiver(APP_LIN_BASE_ADDR);

    LIN_setSyncFields(APP_LIN_BASE_ADDR, 5U, 2U);
    LIN_triggerChecksumCompare(APP_LIN_BASE_ADDR);

    /* Finally exit SW reset and enter LIN ready state */
    LIN_exitSoftwareReset(APP_LIN_BASE_ADDR);

    DebugP_log("[I2C] LIN Volatage Level Shifter started ...\r\n");

    I2C_Transaction_init(&i2cTransaction);
    i2cTransaction.writeBuf   = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.targetAddress = I2C_TARGET_ADDRESS;
    txBuffer[0] = I2C_POLARITY_INV;
    status = I2C_transfer(i2cHandle, &i2cTransaction);
    DebugP_assert(status == SystemP_SUCCESS);

    txID = (LIN_ID + 8);
    txID = LIN_generateParityID(txID);

    LIN_setIDResponderTask(APP_LIN_BASE_ADDR, LIN_generateParityID(txID));
    LIN_setIDByte(APP_LIN_BASE_ADDR, txID);
    
    DebugP_log("[LIN] Responder Write ... !!!\r\n");

    // Wait for an interrupt status indicating a received ID or an error
    while (1) {
        intrStatus = LIN_getInterruptStatus(APP_LIN_BASE_ADDR);

        if (intrStatus & (LIN_FLAG_RXID | LIN_FLAG_FE | LIN_FLAG_PE |
                          LIN_FLAG_PBE | LIN_FLAG_BE | LIN_INT_ISFE)) {
            break;
        }
    }

    if (intrStatus & LIN_FLAG_RXID) {
        LIN_setFrameLength(APP_LIN_BASE_ADDR, FRAME_LENGTH);
        LIN_sendData(APP_LIN_BASE_ADDR, txData);
        
        // Wait until the transmit buffer is empty
        while (!LIN_isTxBufferEmpty(APP_LIN_BASE_ADDR)) {}

        LIN_clearInterruptStatus(APP_LIN_BASE_ADDR, LIN_FLAG_RXRDY);
        status = SystemP_SUCCESS;
    } else {
        status = SystemP_FAILURE;
    }

    if (status == SystemP_SUCCESS) {
        DebugP_log("[LIN] Responder Read ... !!!\r\n");

        // Wait for an interrupt status indicating received data or an error
        while (1) {
            intrStatus = LIN_getInterruptStatus(APP_LIN_BASE_ADDR);

            if (intrStatus & (LIN_FLAG_RXRDY | LIN_FLAG_BE | LIN_FLAG_FE |
                              LIN_FLAG_OE | LIN_FLAG_PE | LIN_FLAG_CE)) {
                break;
            }
        }

        if (intrStatus & LIN_FLAG_RXRDY) {
            // Retrieve the received ID and data
            id = LIN_getRxIdentifier(APP_LIN_BASE_ADDR) & 0x3FU;
            LIN_getData(APP_LIN_BASE_ADDR, rxData);

            DebugP_log("Received Message ID: 0x%x\r\n", id);
            DebugP_log("Received Message Data:\r\n");

            for (i = 0; i < FRAME_LENGTH; i++) {
                DebugP_log("0x%x \r\n", rxData[i]);
            }
            status = SystemP_SUCCESS;
        } else {
            status = SystemP_FAILURE;
            DebugP_log("Error in receiving data\r\n");
        }
    }

    if (status == SystemP_SUCCESS) {
        DebugP_log("All tests have passed!!\r\n");
    } else {
        DebugP_log("Some tests have failed!!\r\n");
    }
    



    Board_driversClose();
    Drivers_close();
}
