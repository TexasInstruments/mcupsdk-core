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

#include <drivers/i2c.h>
#include <kernel/dpl/DebugP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

extern uint32_t Board_i2cGetEepromDeviceAddr(void);
#define I2C_READ_LEN                    (1U)

static void i2c_read_error_handler(uint16_t sample, int32_t status);

void i2c_read_main(void *arg0)
{
    uint16_t        sample;
    int32_t         status;
    uint32_t        i2cReadTargetAddr;
    uint8_t         rxBuffer[I2C_READ_LEN];
    I2C_Handle      i2cHandle;
    I2C_Transaction i2cTransaction;

    Drivers_open();
    Board_driversOpen();

    i2cReadTargetAddr     = Board_i2cGetEepromDeviceAddr();
    i2cHandle = gI2cHandle[CONFIG_I2C0];

    DebugP_log("[I2C] Read data ... !!!\r\n");

    /* Set default transaction parameters */
    I2C_Transaction_init(&i2cTransaction);

    /* Override with required transaction parameters */
    i2cTransaction.readBuf      = rxBuffer;
    i2cTransaction.readCount    = I2C_READ_LEN;
    i2cTransaction.targetAddress = i2cReadTargetAddr;

    /* Read 20 samples and log them */
    for(sample = 0; sample < 20; sample++)
    {
        status = I2C_transfer(i2cHandle, &i2cTransaction);
        if(status == I2C_STS_SUCCESS)
        {
            DebugP_log("[I2C] Sample %u: %u\r\n", sample, rxBuffer[0]);
        }
        else
        {
            i2c_read_error_handler(sample, i2cTransaction.status);
        }
    }

    DebugP_log("[I2C] Read data ... DONE !!!\r\n");
    if(status == SystemP_SUCCESS)
    {
        DebugP_log("All tests have passed!!\r\n");
    }
    else
    {
        DebugP_log("Some tests have failed!!\r\n");
    }

    Board_driversClose();
    Drivers_close();

    return;
}

static void i2c_read_error_handler(uint16_t sample, int32_t status)
{
    switch(status)
    {
        case I2C_STS_ERR:
            DebugP_logError("[I2C] Sample %u: Generic error occurred", sample);
            break;
        case I2C_STS_ERR_TIMEOUT:
            DebugP_logError("[I2C] Sample %u: Timeout error occurred", sample);
            break;
        case I2C_STS_ERR_NO_ACK:
            DebugP_logError("[I2C] Sample %u: No acknowledgement received", sample);
            break;
        case I2C_STS_ERR_ARBITRATION_LOST:
            DebugP_logError("[I2C] Sample %u: Arbitration lost", sample);
            break;
        case I2C_STS_ERR_BUS_BUSY:
            DebugP_logError("[I2C] Sample %u: Bus Bus Busy error occurred", sample);
            break;
    }

    return;
}
