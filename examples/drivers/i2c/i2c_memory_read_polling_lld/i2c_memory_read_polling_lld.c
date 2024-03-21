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

#include <kernel/dpl/DebugP.h>
#include <drivers/hw_include/soc_config.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <drivers/hw_include/cslr_soc.h>

/* Number of bytes to write to and read from EEPROM */
#define BYTE_COUNT                      (10U)

I2CLLD_Handle gI2cLldHandle0;
I2C_Memory_ExtendedParams mem_extendedParams_read;
I2C_Memory_ExtendedParams mem_extendedParams_write;

uint8_t txBuf[BYTE_COUNT];
uint8_t rxBuf[BYTE_COUNT];

extern uint32_t Board_i2cGetEepromDeviceAddr(void);
extern uint16_t Board_i2cGetEepromMemAddr(void);
extern uint8_t Board_i2cGetEepromAddrSize(void);

void i2c_memory_read_polling_main(void *arg)
{
    int32_t status = I2C_STS_SUCCESS;
    uint32_t count = 0U;
    uint16_t eepromMemAddr = Board_i2cGetEepromMemAddr();
    uint8_t eepromAddrSize = Board_i2cGetEepromAddrSize();

    Drivers_open();
    Board_driversOpen();

    DebugP_log("[I2C] LLD Memory Read test Started ... !!!\r\n");

    gI2cLldHandle0 = (I2CLLD_Handle)(gI2cLldHandle[0]);

    /* Initialize I2C_Memory_ExtendedParams write object */
    mem_extendedParams_write.memAddr = eepromMemAddr;
    mem_extendedParams_write.memAddrSize = eepromAddrSize;
    mem_extendedParams_write.extendedParams.deviceAddress =
                                Board_i2cGetEepromDeviceAddr();
    mem_extendedParams_write.extendedParams.buffer = txBuf;
    mem_extendedParams_write.extendedParams.size = BYTE_COUNT;
    mem_extendedParams_write.extendedParams.expandSA = false;

    /* Fill write Buffer */
    for(count = 0; count < BYTE_COUNT; count++)
    {
        txBuf[count] = (uint8_t)count;
    }

    /* Initialize I2C_Memory_ExtendedParams read object */
    mem_extendedParams_read.memAddr = eepromMemAddr;
    mem_extendedParams_read.memAddrSize = eepromAddrSize;
    mem_extendedParams_read.extendedParams.deviceAddress =
                                Board_i2cGetEepromDeviceAddr();
    mem_extendedParams_read.extendedParams.buffer = rxBuf;
    mem_extendedParams_read.extendedParams.size = BYTE_COUNT;
    mem_extendedParams_read.extendedParams.expandSA = false;

    DebugP_log("[I2C] Application will write %d consecutive bytes in polling Mode to EEPROM memory address 0x%x ...\r\n", \
                mem_extendedParams_write.extendedParams.size, \
                mem_extendedParams_write.memAddr);

    /* Initiate Memory Write operation */
    status = I2C_lld_mem_write( gI2cLldHandle0, &mem_extendedParams_write,
                                I2C_WAIT_FOREVER);

    /* To ensure that eeprom is ready, added delay of 4ms */
    ClockP_usleep(4000);

    if(status == I2C_STS_SUCCESS)
    {
        DebugP_log("[I2C] Application will Read %d consecutive bytes in polling Mode from EEPROM memory address 0x%x ...\r\n", \
                mem_extendedParams_read.extendedParams.size, \
                mem_extendedParams_read.memAddr);

        /* Initiate Memory Read operation */
        status = I2C_lld_mem_read(  gI2cLldHandle0, &mem_extendedParams_read,
                                    I2C_WAIT_FOREVER);
    }

    if(status == I2C_STS_SUCCESS)
    {
        for(count = 0; count < BYTE_COUNT; count++)
        {
            DebugP_log("[I2C] Data at address 0x%x : 0x%x \r\n", (eepromMemAddr + count), rxBuf[count]);
        }
    }
    else
    {
        DebugP_log("[I2C] Memory read in polling mode Failed ... !!! \r\n");
    }

    if(status == I2C_STS_SUCCESS)
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
