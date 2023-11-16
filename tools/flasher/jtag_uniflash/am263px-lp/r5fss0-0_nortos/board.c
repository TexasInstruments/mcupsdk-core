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

#include <board/ioexp/ioexp_tca6424.h>

#define IO_MUX_OSPI_RST_SEL_PORT_LINE (0U)      // PORT 0, PIN 0    -> ioIndex : 0*8 + 0 = 0


void App_TCA6424_Params_init(TCA6424_Params *params)
{
    if(NULL != params)
    {
        params->i2cInstance = 0U;
        params->i2cAddress  = 0x22U;
    }

    return;
}

void i2c_flash_reset()
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
}
