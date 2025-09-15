/*
*  Copyright (C) 2025 Texas Instruments Incorporated
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

#include <stdint.h>
#include <drivers/i2c.h>
#include <board/eeprom.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <board/ioexp/ioexp_tca6424.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
/* Port Number : 0, Pin Number : 0. Line Number : (Port Number * 8) + Pin Number */
#define IO_EXP_SPI1_MUX_SEL_LINE      (10U)        /* PORT 1, PIN 2  -> ioIndex : 1*8 + 2 = 10 */
#define IO_EXP_SPI1_MUX_SEL_STATE     (TCA6424_OUT_STATE_HIGH)

#define EEPROM_OFFSET_READ_PCB_REV         (0x0022U)
#define EEPROM_READ_PCB_REV_DATA_LEN       (0x2U)

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

static TCA6424_Config  gTCA6424_Config;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

static int32_t TCA6424_enable_spi_mux(void);

int32_t mcspi_io_expander_open(void)
{
    int32_t status = SystemP_SUCCESS;
    uint8_t boardVer[2] = "";

    Board_eepromOpen();

    status = EEPROM_read(gEepromHandle[CONFIG_EEPROM0], EEPROM_OFFSET_READ_PCB_REV, boardVer, EEPROM_READ_PCB_REV_DATA_LEN);
    if(status == SystemP_SUCCESS)
    {
        if(boardVer[1] == '1' && boardVer[0] == 'E')
        {
            /* boardVer is E1 */
            /* MCAN Transceiver is enabled by default in E1*/
        }
        else
        {
            /* boardVer is E2 or Rev A or Rev B*/
            status = TCA6424_enable_spi_mux();
        }
    }

    DebugP_assert(status == SystemP_SUCCESS);
    Board_eepromClose();

    return status;     
}

static int32_t TCA6424_enable_spi_mux()
{
    int32_t             status = SystemP_SUCCESS;
    TCA6424_Params      tca6424Params;

    TCA6424_Params_init(&tca6424Params);
    status = TCA6424_open(&gTCA6424_Config, &tca6424Params);
    DebugP_assert(SystemP_SUCCESS == status);

    status = TCA6424_setOutput(&gTCA6424_Config, IO_EXP_SPI1_MUX_SEL_LINE, IO_EXP_SPI1_MUX_SEL_STATE);
    DebugP_assert(SystemP_SUCCESS == status);

    status += TCA6424_config(&gTCA6424_Config, IO_EXP_SPI1_MUX_SEL_LINE, TCA6424_MODE_OUTPUT);
    DebugP_assert(SystemP_SUCCESS == status);

    TCA6424_close(&gTCA6424_Config);

    return status;
}