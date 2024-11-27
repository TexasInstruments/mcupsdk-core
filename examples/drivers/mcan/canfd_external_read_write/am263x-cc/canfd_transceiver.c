/*
 *  Copyright (C) 2023 Texas Instruments Incorporated
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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <drivers/i2c.h>
#include "ti_drivers_open_close.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* Input status register */
#define TCA6416_REG_INPUT0              ((UInt8) 0x00U)
#define TCA6416_REG_INPUT1              ((UInt8) 0x01U)

/* Output register to change state of output BIT set to 1, output set HIGH */
#define TCA6416_REG_OUTPUT0             ((uint8_t) 0x02U)
#define TCA6416_REG_OUTPUT1             ((uint8_t) 0x03U)

/* Configuration register. BIT = '1' sets port to input, BIT = '0' sets
 * port to output */
#define TCA6416_REG_CONFIG0             ((uint8_t) 0x06U)
#define TCA6416_REG_CONFIG1             ((uint8_t) 0x07U)

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
static void SetupI2CTransfer(I2C_Handle handle,  uint32_t targetAddr,
                      uint8_t *writeData, uint32_t numWriteBytes,
                      uint8_t *readData,  uint32_t numReadBytes);

void mcanEnableTransceiver(void)
{
    I2C_Handle      i2cHandle;
    uint8_t         dataToSlave[4];

    i2cHandle = gI2cHandle[CONFIG_I2C0];
    dataToSlave[0] = TCA6416_REG_CONFIG0;
    dataToSlave[1] = 0x0U;
    SetupI2CTransfer(i2cHandle, 0x20, &dataToSlave[0], 1, &dataToSlave[1], 1);
    /* set the P00 to 0 make them output ports. */
    dataToSlave[1] &= ~(0x1U);
    SetupI2CTransfer(i2cHandle, 0x20, &dataToSlave[0], 2, NULL, 0);

    /* Get the port values. */
    dataToSlave[0] = TCA6416_REG_INPUT0;
    dataToSlave[1] = 0x0U;
    SetupI2CTransfer(i2cHandle, 0x20, &dataToSlave[0], 1, &dataToSlave[1], 1);

    /* Set P10 and P11 to 0.
     */
    dataToSlave[0] = TCA6416_REG_OUTPUT0;
    dataToSlave[1] &= ~(0x1);
    SetupI2CTransfer(i2cHandle, 0x20, &dataToSlave[0], 2, NULL, 0);
}

static void SetupI2CTransfer(I2C_Handle handle,  uint32_t targetAddr,
                      uint8_t *writeData, uint32_t numWriteBytes,
                      uint8_t *readData,  uint32_t numReadBytes)
{
    int32_t status;
    I2C_Transaction i2cTransaction;

    /* Enable Transceiver */
    I2C_Transaction_init(&i2cTransaction);
    i2cTransaction.targetAddress = targetAddr;
    i2cTransaction.writeBuf = (uint8_t *)&writeData[0];
    i2cTransaction.writeCount = numWriteBytes;
    i2cTransaction.readBuf = (uint8_t *)&readData[0];
    i2cTransaction.readCount = numReadBytes;
    status = I2C_transfer(handle, &i2cTransaction);
    DebugP_assert(SystemP_SUCCESS == status);
}