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

#include <drivers/i2c.h>
#include <kernel/dpl/DebugP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

extern uint32_t Board_i2cGetEepromDeviceAddr(void);
extern uint16_t Board_i2cGetEepromMemAddr(void);
extern uint8_t Board_i2cGetEepromAddrSize(void);

#define BYTE_COUNT                      (10U)

void i2c_memory_read_main(void *arg)
{
    int32_t         status;
    uint32_t        i2cDeviceAddr;
    uint32_t        i2cEepromMemAddr;
    uint32_t        i2cEepromAddrSize;

    uint8_t         txBuffer[BYTE_COUNT];
    uint8_t         rxBuffer[BYTE_COUNT];

    uint32_t        count = 0;

    I2C_Handle      i2cHandle;
    I2C_Transaction i2cTransaction;
    I2C_Mem_Transaction mem_transaction;

    Drivers_open();
    Board_driversOpen();

    i2cDeviceAddr = Board_i2cGetEepromDeviceAddr();
    i2cEepromMemAddr = (uint32_t)Board_i2cGetEepromMemAddr();
    i2cEepromAddrSize = (uint32_t)Board_i2cGetEepromAddrSize();

    i2cHandle = gI2cHandle[CONFIG_I2C0];

    DebugP_log("[I2C] Writing data to EEPROM ... !!!\r\n");

    /* Set default transaction parameters */
    I2C_Transaction_init(&i2cTransaction);

    /* Override with required transaction parameters */
    i2cTransaction.memTxnEnable = true;
    i2cTransaction.memTransaction = &mem_transaction;
    i2cTransaction.targetAddress = i2cDeviceAddr;

    mem_transaction.memAddr = i2cEepromMemAddr;
    mem_transaction.memAddrSize = i2cEepromAddrSize;
    mem_transaction.buffer = txBuffer;
    mem_transaction.size = (uint32_t)BYTE_COUNT;
    mem_transaction.memDataDir = I2C_MEM_TXN_DIR_TX;

    for(count = 0; count < BYTE_COUNT; count++)
    {
        txBuffer[count] = (uint8_t)count;
    }

    /* Initiate memory write transfer operation */
    status = I2C_transfer(i2cHandle, &i2cTransaction);

    ClockP_usleep(4000);

    if(status == SystemP_SUCCESS)
    {
        DebugP_log("[I2C] Reading data from EEPROM ... !!!\r\n");

        mem_transaction.memAddr = i2cEepromMemAddr;
        mem_transaction.memAddrSize = i2cEepromAddrSize;
        mem_transaction.buffer = rxBuffer;
        mem_transaction.size = (uint32_t)BYTE_COUNT;
        mem_transaction.memDataDir = I2C_MEM_TXN_DIR_RX;

        /* Initiate memory read transfer operation */
        status = I2C_transfer(i2cHandle, &i2cTransaction);
    }

    if(status == SystemP_SUCCESS)
    {
        for(count = 0; count < BYTE_COUNT; count++)
        {
            DebugP_log("[I2C] Data at address 0x%x : 0x%x \r\n", (i2cEepromMemAddr + count), rxBuffer[count]);
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
