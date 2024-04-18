/*
 *  Copyright (C) 2023-24 Texas Instruments Incorporated
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
#include <stdint.h>
#include <board/ioexp/ioexp_tca6424.h>
#include <board/eeprom.h>
#include <drivers/i2c.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
#define IO_MUX_MCAN_SEL                             (4U)                        /* PORT 0, PIN 4         -> ioIndex : 0*8 + 4 = 4*/
#define IO_MUX_MCAN_STB                             (17U)                       /* PORT 2, PIN 1         -> ioIndex : 2*8 + 1 = 17 */
#define TCA6424_IO_MUX_MCAN_SEL_PORT_LINE_STATE     (TCA6424_OUT_STATE_LOW)     /* MCAN_SEL PIN OUTPUT   -> 0 */
#define TCA6424_IO_MUX_MCAN_STB_PORT_LINE_STATE     (TCA6424_OUT_STATE_HIGH)    /* MCAN_STB PIN OUTPUT   -> 1 */
#define EEPROM_OFFSET_READ_PCB_REV                  (0x0022U)
#define EEPROM_READ_PCB_REV_DATA_LEN                (0x2U)

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
/*                            Global Variables                                */
/* ========================================================================== */

static TCA6424_Config  gTCA6424_Config;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

static int32_t SetupI2CTransfer(I2C_Handle handle,  uint32_t targetAddr,
                      uint8_t *writeData, uint32_t numWriteBytes,
                      uint8_t *readData,  uint32_t numReadBytes)
{
    int32_t status = SystemP_SUCCESS;
    I2C_Transaction i2cTransaction;

    /* Enable Transceiver */
    I2C_Transaction_init(&i2cTransaction);
    i2cTransaction.targetAddress = targetAddr;
    i2cTransaction.writeBuf = (uint8_t *)&writeData[0];
    i2cTransaction.writeCount = numWriteBytes;
    i2cTransaction.readBuf = (uint8_t *)&readData[0];
    i2cTransaction.readCount = numReadBytes;
    status = I2C_transfer(handle, &i2cTransaction);
    return status;
}

int32_t TCA6416_Transceiver(void)
{
    I2C_Handle      i2cHandle;
    uint8_t         dataToSlave[4];
    int32_t status = SystemP_SUCCESS;

    i2cHandle = gI2cHandle[CONFIG_I2C0];
    dataToSlave[0] = TCA6416_REG_CONFIG0;
    dataToSlave[1] = 0x0U;
    status = SetupI2CTransfer(i2cHandle, 0x20, &dataToSlave[0], 1, &dataToSlave[1], 1);
    DebugP_assert(status == SystemP_SUCCESS);

    /* set the P00 to 0 make them output ports. */
    dataToSlave[1] &= ~(0x1U);
    status = SetupI2CTransfer(i2cHandle, 0x20, &dataToSlave[0], 2, NULL, 0);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Get the port values. */
    dataToSlave[0] = TCA6416_REG_INPUT0;
    dataToSlave[1] = 0x0U;
    status = SetupI2CTransfer(i2cHandle, 0x20, &dataToSlave[0], 1, &dataToSlave[1], 1);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Set P10 and P11 to 0.
     */
    dataToSlave[0] = TCA6416_REG_OUTPUT0;
    dataToSlave[1] &= ~(0x1);
    status = SetupI2CTransfer(i2cHandle, 0x20, &dataToSlave[0], 2, NULL, 0);
    DebugP_assert(status == SystemP_SUCCESS);

    return status;
}

int32_t TCA6424_Transceiver(void)
{
    int32_t             status = SystemP_SUCCESS;
    TCA6424_Params      tca6424Params;
    TCA6424_Params_init(&tca6424Params);

    status = TCA6424_open(&gTCA6424_Config, &tca6424Params);
    DebugP_assert(SystemP_SUCCESS == status);

    /* For MCAN_SEL */
    status = TCA6424_setOutput(
                    &gTCA6424_Config,
                    IO_MUX_MCAN_SEL,
                    TCA6424_IO_MUX_MCAN_SEL_PORT_LINE_STATE);
    DebugP_assert(SystemP_SUCCESS == status);

    status += TCA6424_config(
                    &gTCA6424_Config,
                    IO_MUX_MCAN_SEL,
                    TCA6424_MODE_OUTPUT);
    DebugP_assert(SystemP_SUCCESS == status);


    /* For MCAN_STB*/
    status += TCA6424_setOutput(
                    &gTCA6424_Config,
                    IO_MUX_MCAN_STB,
                    TCA6424_IO_MUX_MCAN_STB_PORT_LINE_STATE);
    DebugP_assert(SystemP_SUCCESS == status);

    status += TCA6424_config(
                    &gTCA6424_Config,
                    IO_MUX_MCAN_STB,
                    TCA6424_MODE_OUTPUT);

    if(status != SystemP_SUCCESS)
    {
        DebugP_log("Transceiver Setup Failure !!");
        TCA6424_close(&gTCA6424_Config);
    }

    TCA6424_close(&gTCA6424_Config);
    return status;
}

void mcanEnableTransceiver()
{
    int32_t status = SystemP_SUCCESS;
    uint8_t boardVer[2] = "";

    Board_eepromOpen();

    status = EEPROM_read(gEepromHandle[CONFIG_EEPROM0], EEPROM_OFFSET_READ_PCB_REV, boardVer, EEPROM_READ_PCB_REV_DATA_LEN);
    if(status == SystemP_SUCCESS)
    {
        if(boardVer[1] == '2')
        {
            /* boardVer is E2 */
            status = TCA6424_Transceiver();
        }
        else if(boardVer[1] == '1')
        {
            /* boardVer is E1 */
            status = TCA6416_Transceiver();
        }
        else
        {
            status = TCA6416_Transceiver();
        }
    }

    DebugP_assert(status == SystemP_SUCCESS);
    Board_eepromClose();
}