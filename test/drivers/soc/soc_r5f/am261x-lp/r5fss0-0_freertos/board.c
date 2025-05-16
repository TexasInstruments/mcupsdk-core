/*
 *  Copyright (C) 2022-23 Texas Instruments Incorporated
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
#include <drivers/gpio.h>
#include <kernel/dpl/AddrTranslateP.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

#define PIN_STATE_HIGH      (1U)
#define PIN_STATE_LOW       (0U)

int32_t enableOspiResetSignal(uint16_t enable);

void board_flash_reset(OSPI_Handle oHandle)
{
    int32_t status = SystemP_FAILURE;

    /* Toggle the level translator to enable the ospi reset signal */
    status = enableOspiResetSignal(TRUE);

    if (status == SystemP_SUCCESS)
    {
        /* Now toggle the ospi reset line */
        OSPI_setResetPinStatus(oHandle, PIN_STATE_HIGH);
        OSPI_setResetPinStatus(oHandle, PIN_STATE_LOW);
        
        status = enableOspiResetSignal(FALSE);
        if (status == SystemP_FAILURE)
        {
            DebugP_log("Failed to disable the ospi reset line!\r\n");
        }
    }
    else
    {
        DebugP_log("Failed to enable the ospi reset line!\r\n");

    }
}


/*
 * BP_BO_MUX_EN is set high in Board_driversOpen
 * This API pulls this signal to low once the application is received,
 * to allow OSPI Reset to be possible.
 */

int32_t enableOspiResetSignal(uint16_t enable)
{
    int32_t status = SystemP_SUCCESS;
    static TCA6408_Config gTCA6408_Config;
    TCA6408_Params TCA6408Params;
    TCA6408_Params_init(&TCA6408Params);
    TCA6408Params.i2cAddress = 0x20U;

    status = TCA6408_open(&gTCA6408_Config, &TCA6408Params);

    /* Configure as output  */
    status += TCA6408_config(
                        &gTCA6408_Config,
                        IO_EXP_BP_BO_MUX_EN_LINE,
                        TCA6408_MODE_OUTPUT);

    if (enable == TRUE)
    {
        /* Configure State */
        status = TCA6408_setOutput(
                        &gTCA6408_Config,
                        IO_EXP_BP_BO_MUX_EN_LINE,
                        TCA6408_OUT_STATE_HIGH);
    }
    else
    {
        /* Configure State */
        status = TCA6408_setOutput(
                        &gTCA6408_Config,
                        IO_EXP_BP_BO_MUX_EN_LINE,
                        TCA6408_OUT_STATE_LOW);
    }

    if (status != SystemP_SUCCESS)
    {
        DebugP_log("Failed to enable OSPI Reset Signal\r\n");
        TCA6408_close(&gTCA6408_Config);
    }

    if(SystemP_FAILURE == status)
    {
        /* Exit gracefully */
    }

    return status;
}