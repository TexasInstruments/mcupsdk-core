/*
 *  Copyright (C) 2024 Texas Instruments Incorporated
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
#include <stdio.h>
#include <inttypes.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <drivers/ipc_notify.h>
#include <drivers/i2c.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/* This example shows Controller Peripheral Communication between Two I2C instances.
 *
 * One of the core is in control of the I2C Controller instance
 * and the other core is control of the I2C Peripheral instance
 *
 * The main core in in control of the Controller I2C Instance.
 * The remote cores in in control of the Peripheral I2C Instance.
 *
 * Two transactions are carried out in this example.
 *
 * In the first transaction the Controller Reads two bytes from Peripheral in Blocking Mode.
 * In the second transaction the Controller Writes two bytes to Peripheral in Blocking Mode.
 *
 */

#if defined (SOC_AM263X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
/* main core that starts the message exchange */
uint32_t gMainCoreId = CSL_CORE_ID_R5FSS0_0;
/* remote cores that echo messages from main core, make sure to NOT list main core in this list */
uint32_t gRemoteCoreId = CSL_CORE_ID_R5FSS0_1;

#endif

I2C_Handle      gI2cControllerHandle;
I2C_Transaction gI2cControllerTransaction;

I2C_Handle      gI2cPeripheralHandle;
I2C_Transaction gI2cPeripheralTransaction;

uint8_t         gTxControllerBuffer[2];
uint8_t         gRxControllerBuffer[2];
uint8_t         gTxPeripheralBuffer[2];
uint8_t         gRxPeripheralBuffer[2];

void i2c_peripheral_transfer_Controller_core_start(void)
{
    int32_t         status;
    uint32_t        i2cTargetAddr = 0x2C;

    /* wait for all cores to be ready */
    IpcNotify_syncAll(SystemP_WAIT_FOREVER);

    DebugP_log("[I2C Controller] I2C Controller Peripheral Transaction Started ... !!!\r\n");

    gI2cControllerHandle = gI2cHandle[CONFIG_I2C0];

    /* Set default transaction parameters */
    I2C_Transaction_init(&gI2cControllerTransaction);

    /* Override with required transaction parameters */
    gI2cControllerTransaction.readBuf        = gRxControllerBuffer;
    gI2cControllerTransaction.readCount      = 2U;
    gI2cControllerTransaction.writeBuf       = gTxControllerBuffer;
    gI2cControllerTransaction.writeCount     = 0U;
    gI2cControllerTransaction.targetAddress  = i2cTargetAddr;

    /* Fill TX Buffer */
    gTxControllerBuffer[0] = 0x01U;
    gTxControllerBuffer[1] = 0x02U;

    /* Wait for Peripheral to initiate transaction */
    ClockP_sleep(1);

    /* Start Transfer */
    status = I2C_transfer(gI2cControllerHandle, &gI2cControllerTransaction);

    if(status == SystemP_SUCCESS)
    {
        DebugP_log("[I2C Controller] Received Data %u %u !!!\r\n", gRxControllerBuffer[0], gRxControllerBuffer[1]);

        /* Clear buffer */
        gRxControllerBuffer[0] = 0U;
        gRxControllerBuffer[1] = 0U;
    }

    /* Wait for Peripheral to initiate transaction */
    ClockP_sleep(1);

    /* Update Transfer Parameters */
    gI2cControllerTransaction.readCount      = 0U;
    gI2cControllerTransaction.writeCount     = 2U;

    /* Start Transfer */
    status = I2C_transfer(gI2cControllerHandle, &gI2cControllerTransaction);

    if(status == SystemP_SUCCESS)
    {
        DebugP_log("[I2C Controller] Transmitted Data %u %u !!!\r\n", gTxControllerBuffer[0], gTxControllerBuffer[1]);

        /* Clear buffer */
        gRxControllerBuffer[0] = 0U;
        gRxControllerBuffer[1] = 0U;
    }

    DebugP_log("[I2C Controller] Transaction Complete !!!\r\n");
    DebugP_log("All tests have passed!!\r\n");
}

void i2c_peripheral_transfer_Peripheral_core_start(void)
{
    int32_t         status;

    /* Wait for all cores to be ready */
    IpcNotify_syncAll(SystemP_WAIT_FOREVER);

    DebugP_log("[I2C Peripheral] I2C Controller Peripheral Transaction Started ... !!!\r\n");

    gI2cPeripheralHandle = gI2cHandle[CONFIG_I2C0];

    /* Set default transaction parameters */
    I2C_Transaction_init(&gI2cPeripheralTransaction);

    /* Override with required transaction parameters */
    gI2cPeripheralTransaction.readBuf         = gRxPeripheralBuffer;
    gI2cPeripheralTransaction.readCount       = 0U;
    gI2cPeripheralTransaction.writeBuf        = gTxPeripheralBuffer;
    gI2cPeripheralTransaction.writeCount      = 2U;
    gI2cPeripheralTransaction.controllerMode  = false;

    /* Fill TX Buffer */
    gTxPeripheralBuffer[0] = 0x03U;
    gTxPeripheralBuffer[1] = 0x04U;

    /* Start Transfer */
    status = I2C_transfer(gI2cPeripheralHandle, &gI2cPeripheralTransaction);

    if(status == SystemP_SUCCESS)
    {
        DebugP_log("[I2C Peripheral] Transmitted Data %u %u !!!\r\n", gTxPeripheralBuffer[0], gTxPeripheralBuffer[1]);

        /* Clear buffer */
        gRxPeripheralBuffer[0] = 0U;
        gRxPeripheralBuffer[1] = 0U;
    }

    /* Update Transfer Parameters */
    gI2cPeripheralTransaction.readCount       = 2U;
    gI2cPeripheralTransaction.writeCount      = 0U;

    /* Start Transfer */
    status = I2C_transfer(gI2cPeripheralHandle, &gI2cPeripheralTransaction);

    if(status == SystemP_SUCCESS)
    {
        DebugP_log("[I2C Peripheral] Received Data %u %u !!!\r\n", gRxPeripheralBuffer[0], gRxPeripheralBuffer[1]);
    }

    DebugP_log("[I2C Peripheral] Transaction Complete !!!\r\n");
}

void i2c_peripheral_transfer_main(void *args)
{
    Drivers_open();
    Board_driversOpen();

    if(IpcNotify_getSelfCoreId()==gMainCoreId)
    {
        i2c_peripheral_transfer_Controller_core_start();
    }
    else
    {
        i2c_peripheral_transfer_Peripheral_core_start();
    }

    Board_driversClose();
    Drivers_close();
}
