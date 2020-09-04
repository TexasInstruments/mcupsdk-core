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

#include <kernel/dpl/DebugP.h>
#include <drivers/i2c.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/* Temperature result registers */
#define TMP10X_RESULT_REG       (0x0000U)

extern uint8_t Board_getSocTemperatureSensorAddr(void);
extern uint8_t Board_getPowerSecTemperatureSensorAddr(void);

void i2c_temperature_main(void *arg0)
{
    uint16_t        sample;
    int16_t         temperature;
    uint8_t         txBuffer[1];
    uint8_t         rxBuffer[2];
    int32_t         status;
    uint8_t         deviceAddress;
    I2C_Handle      i2cHandle;
    I2C_Transaction i2cTransaction;

    Drivers_open();
    Board_driversOpen();

    i2cHandle = gI2cHandle[CONFIG_I2C0];

    /* Determine if I2C sensor is present */
    deviceAddress = Board_getSocTemperatureSensorAddr();
    status = I2C_probe(i2cHandle, deviceAddress);
    if(status == SystemP_SUCCESS)
    {
        DebugP_log("[I2C] Temperature sensor found at device address 0x%02x \r\n", deviceAddress);
    }
    else
    {
        DebugP_logError("[I2C] Temperature sensor not found at device address 0x%02x \r\n", deviceAddress);
    }

    if(status == SystemP_SUCCESS)
    {
        /* found temperature sensor */
        /* Select result register */
        I2C_Transaction_init(&i2cTransaction);
        i2cTransaction.writeBuf   = txBuffer;
        i2cTransaction.writeCount = 1;
        i2cTransaction.slaveAddress = deviceAddress;
        txBuffer[0] = TMP10X_RESULT_REG;
        status = I2C_transfer(i2cHandle, &i2cTransaction);
        if(status == SystemP_SUCCESS)
        {
            /* read the results */
            I2C_Transaction_init(&i2cTransaction);
            i2cTransaction.readBuf = rxBuffer;
            i2cTransaction.readCount = 2;
            i2cTransaction.slaveAddress = deviceAddress;

            /* Take 20 samples and print them out onto the console */
            for(sample = 0; sample < 20; sample++)
            {
                /* clear RX buffer every time we read, to make sure it does not have stale data */
                rxBuffer[0] = rxBuffer[1] = 0;
                status = I2C_transfer(i2cHandle, &i2cTransaction);
                if(status == SystemP_SUCCESS)
                {
                    /* Create 16 bit temperature */
                    temperature = ((uint16_t)rxBuffer[0] << 8) | (rxBuffer[1]);
                    /*
                     * 4 LSBs of temperature are 0 according to datasheet
                     * since temperature is stored in 12 bits. Therefore,
                     * right shift by 4 places
                     */
                    temperature = temperature >> 4;
                    /*
                     * If the 12th bit of temperature is set '1' (equivalent to 8th bit of the first byte read),
                     * then we have a 2's complement negative value which needs to be sign extended
                     */
                    if(rxBuffer[0] & 0x80)
                    {
                        temperature |= 0xF000;
                    }
                    /* Of the 12 bits of temperature, 4 LSBs are for decimal point according to datasheet so divide by 16 */
                    DebugP_log("[I2C] Sample %u: %f (celcius)\r\n", sample, temperature/16.0);
                }
                else
                {
                    DebugP_logError("[I2C] Sample %u read failed\r\n", sample);
                    break;
                }
            }
        }
    }

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
