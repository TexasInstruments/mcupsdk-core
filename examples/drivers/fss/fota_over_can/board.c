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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>
#include <string.h>
#include <board/ioexp/ioexp_tca6416.h>
#include <board/ioexp/ioexp_tca6424.h>
#include <board/eeprom.h>
#include <drivers/i2c.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* PORT 0, PIN 0         -> ioIndex : 0*8 + 0 = 0*/
#define IO_MUX_OSPI_RST_SEL_PORT_LINE               (0U)
/* PORT 0, PIN 4         -> ioIndex : 0*8 + 4 = 4*/
#define IO_MUX_MCAN_SEL                             (4U)
/* PORT 2, PIN 1         -> ioIndex : 2*8 + 1 = 17 */
#define IO_MUX_MCAN_STB                             (17U)
/* MCAN_SEL PIN OUTPUT   -> 0 */
#define TCA6424_IO_MUX_MCAN_SEL_PORT_LINE_STATE     (TCA6424_OUT_STATE_LOW)
/* MCAN_STB PIN OUTPUT   -> 1 */
#define TCA6424_IO_MUX_MCAN_STB_PORT_LINE_STATE     (TCA6424_OUT_STATE_HIGH)

#define EEPROM_OFFSET_READ_PCB_REV                  (0x0022U)
#define EEPROM_READ_PCB_REV_DATA_LEN                (0x2U)
#define BOARD_VERSION_E2                            ('2')

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
uint8_t gBoardVer[2] = "";
static TCA6424_Config  gTCA6424_Config;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t TCA6424_Flash_reset()
{
    static TCA6424_Config  gTCA6424_Config;
    int32_t             status = SystemP_SUCCESS;
    TCA6424_Params      TCA6424Params;

    TCA6424_Params_init(&TCA6424Params);
    status = TCA6424_open(&gTCA6424_Config, &TCA6424Params);

    /* Configure as output  */
    status += TCA6424_config(&gTCA6424_Config,
                    IO_MUX_OSPI_RST_SEL_PORT_LINE,
                    TCA6424_MODE_OUTPUT);

    status = TCA6424_setOutput(&gTCA6424_Config,IO_MUX_OSPI_RST_SEL_PORT_LINE,TCA6424_OUT_STATE_LOW);

    status = TCA6424_setOutput(&gTCA6424_Config,IO_MUX_OSPI_RST_SEL_PORT_LINE,TCA6424_OUT_STATE_HIGH);

    if(status != SystemP_SUCCESS)
    {
        DebugP_log("Failure to reset the Flash!! %d\r\n");
        TCA6424_close(&gTCA6424_Config);
    }

    TCA6424_close(&gTCA6424_Config);

    return status;
}


int32_t TCA6416_Flash_reset()
{
    static TCA6416_Config  gTCA6416_Config;
    int32_t             status = SystemP_SUCCESS;
    TCA6416_Params      tca6416Params;

    TCA6416_Params_init(&tca6416Params);
    status = TCA6416_open(&gTCA6416_Config, &tca6416Params);

    /* Configure as output  */
    status += TCA6416_config(&gTCA6416_Config,
                    IO_MUX_OSPI_RST_SEL_PORT_LINE,
                    TCA6416_MODE_OUTPUT);

    status = TCA6416_setOutput(&gTCA6416_Config,IO_MUX_OSPI_RST_SEL_PORT_LINE,TCA6416_OUT_STATE_LOW);

    status = TCA6416_setOutput(&gTCA6416_Config,IO_MUX_OSPI_RST_SEL_PORT_LINE,TCA6416_OUT_STATE_HIGH);

    if(status != SystemP_SUCCESS)
    {
        DebugP_log("Failure to reset the Flash!! %d\r\n");
        TCA6416_close(&gTCA6416_Config);
    }

    TCA6416_close(&gTCA6416_Config);

    return status;
}

int32_t TCA6424_Mcan_Transceiver(void)
{
    int32_t             status = SystemP_SUCCESS;
    TCA6424_Params      tca6424Params;

    TCA6424_Params_init(&tca6424Params);
    status = TCA6424_open(&gTCA6424_Config, &tca6424Params);
    DebugP_assert(SystemP_SUCCESS == status);

    /* For MCAN_SEL */
    status = TCA6424_setOutput(&gTCA6424_Config,IO_MUX_MCAN_SEL,TCA6424_IO_MUX_MCAN_SEL_PORT_LINE_STATE);
    DebugP_assert(SystemP_SUCCESS == status);

    status += TCA6424_config(&gTCA6424_Config,IO_MUX_MCAN_SEL,TCA6424_MODE_OUTPUT);
    DebugP_assert(SystemP_SUCCESS == status);

    /* For MCAN_STB*/
    status += TCA6424_setOutput(&gTCA6424_Config,IO_MUX_MCAN_STB,TCA6424_IO_MUX_MCAN_STB_PORT_LINE_STATE);
    DebugP_assert(SystemP_SUCCESS == status);

    status += TCA6424_config(&gTCA6424_Config,IO_MUX_MCAN_STB,TCA6424_MODE_OUTPUT);

    if(status != SystemP_SUCCESS)
    {
        DebugP_log("Transceiver Setup Failure !!");
        TCA6424_close(&gTCA6424_Config);
    }

    TCA6424_close(&gTCA6424_Config);
    return status;
}

void i2c_flash_reset(void)
{
    int32_t status = SystemP_SUCCESS;

    Board_eepromOpen();

    status = EEPROM_read(gEepromHandle[CONFIG_EEPROM0], EEPROM_OFFSET_READ_PCB_REV, gBoardVer, EEPROM_READ_PCB_REV_DATA_LEN);
    if(status == SystemP_SUCCESS)
    {
        if(gBoardVer[1] == BOARD_VERSION_E2)
        {
            /* boardVer is E2 */
            status = TCA6424_Flash_reset();
        }
        else
        {
            /* boardVer is E1 */
            status = TCA6416_Flash_reset();
        }
    }

    DebugP_assert(status == SystemP_SUCCESS);
    Board_eepromClose();
}

void mcanEnableTransceiver(void)
{
    int32_t status = SystemP_SUCCESS;

    if(gBoardVer[1] == BOARD_VERSION_E2)
    {
        /* boardVer is E1 */
        status = TCA6424_Mcan_Transceiver();
    }
    else
    {
        /* boardVer is E1 */
        /* MCAN Transceiver is enabled by default in E1*/
    }
    
    DebugP_assert(status == SystemP_SUCCESS);
}