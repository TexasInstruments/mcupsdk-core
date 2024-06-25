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
#include <kernel/dpl/ClockP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

void i2c_io_expander(void *args)
{
    int32_t         status;
    uint32_t        i2cTargetAddr;
    uint8_t         txBuffer[3];
    I2C_Handle      i2cHandle;
    I2C_Transaction i2cTransaction;
    i2cTargetAddr     = 0x20;
    i2cHandle = gI2cHandle[CONFIG_I2C0];
    status = I2C_probe(i2cHandle, i2cTargetAddr);


    if(status == SystemP_SUCCESS)
    {
        DebugP_log("[I2C] IO Expander found at device address 0x%02x \r\n", i2cTargetAddr);
    }
    else
    {
        DebugP_logError("[I2C] IO Expander not found at device address 0x%02x \r\n", i2cTargetAddr);
    }
    if(status == SystemP_SUCCESS)
            {


                I2C_Transaction_init(&i2cTransaction);
                i2cTransaction.writeBuf   = txBuffer;
                i2cTransaction.writeCount = 3;
                i2cTransaction.targetAddress = i2cTargetAddr;
                txBuffer[0] = 0x06; /* configuration register 1 */
                txBuffer[1] = 0xF3; /* ICSSM1_MUX_SEL ,ICSSM2_MUX_SELconfigured as output */
                //txBuffer[2] = 0x07; /* configuration register 2 */
                txBuffer[2] = 0xFE; /* RGMII_MUX_SEL configured as output */


                I2C_transfer(i2cHandle, &i2cTransaction);
                DebugP_log("[I2C] IO Expander Configuration complete. [GPMC][PSRAM] ready for operation. \r\n");
            }
    Drivers_i2cClose();
}