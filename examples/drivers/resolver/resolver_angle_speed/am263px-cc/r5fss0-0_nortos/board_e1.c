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

#include <stdint.h>

#include <board/ioexp/ioexp_tca6416.h>

#define IO_MUX_ADC1_MUX_SEL_PORT_LINE (14)      // PORT 1, PIN 6    -> ioIndex : 1*8 + 6 = 14
#define IO_MUX_ADC1_MUX_SEL_PORT_LINE_STATE (TCA6416_OUT_STATE_HIGH)

#define IO_MUX_ADC2_MUX_SEL_PORT_LINE (15)      // PORT 1, PIN 7    -> ioIndex : 1*8 + 7 = 15
#define IO_MUX_ADC2_MUX_SEL_PORT_LINE_STATE (TCA6416_OUT_STATE_LOW)

#define IO_MUX_ADC3_MUX_SEL_PORT_LINE (5)       // PORT 0, PIN 5    -> ioIndex : 0*8 + 5 = 5
#define IO_MUX_ADC3_MUX_SEL_PORT_LINE_STATE (TCA6416_OUT_STATE_LOW)

#define IO_MUX_ADC4_MUX_SEL_PORT_LINE (6)       // PORT 0, PIN 6    -> ioIndex : 0*8 + 6 = 6
#define IO_MUX_ADC4_MUX_SEL_PORT_LINE_STATE (TCA6416_OUT_STATE_HIGH)

#define IO_MUX_ADC5_MUX_SEL_PORT_LINE (8)       // PORT 1, PIN 0    -> ioIndex : 1*8 + 0 = 8
#define IO_MUX_ADC5_MUX_SEL_PORT_LINE_STATE (TCA6416_OUT_STATE_HIGH)

#define IO_MUX_ICSSM2_MUX_SEL_PORT_LINE (3)      // PORT 0, PIN 3    -> ioIndex : 0*8 + 3 = 3
#define IO_MUX_ICSSM2_MUX_SEL_PORT_LINE_STATE (TCA6416_OUT_STATE_HIGH)

#define IO_MUX_PORT_LINES (6)


void App_TCA6416_Params_init(TCA6416_Params *params)
{
    if(NULL != params)
    {
        params->i2cInstance = 0U;
        params->i2cAddress  = 0x20U;
    }

    return;
}

void i2c_io_expander_resolver_adc()
{
    static TCA6416_Config  gTCA6416_Config;
    int32_t             status = SystemP_SUCCESS;
    TCA6416_Params      tca6416Params;
    App_TCA6416_Params_init(&tca6416Params);
    status = TCA6416_open(&gTCA6416_Config, &tca6416Params);
    uint32_t ioIndex[IO_MUX_PORT_LINES] = {
        IO_MUX_ADC1_MUX_SEL_PORT_LINE,
        IO_MUX_ADC2_MUX_SEL_PORT_LINE,
        IO_MUX_ADC3_MUX_SEL_PORT_LINE,
        IO_MUX_ADC4_MUX_SEL_PORT_LINE,
        IO_MUX_ADC5_MUX_SEL_PORT_LINE,
        IO_MUX_ICSSM2_MUX_SEL_PORT_LINE,
    };
    uint32_t ioIndex_state[IO_MUX_PORT_LINES] = {
        IO_MUX_ADC1_MUX_SEL_PORT_LINE_STATE,
        IO_MUX_ADC2_MUX_SEL_PORT_LINE_STATE,
        IO_MUX_ADC3_MUX_SEL_PORT_LINE_STATE,
        IO_MUX_ADC4_MUX_SEL_PORT_LINE_STATE,
        IO_MUX_ADC5_MUX_SEL_PORT_LINE_STATE,
        IO_MUX_ICSSM2_MUX_SEL_PORT_LINE_STATE,
    };

    for(int iter = 0; iter < IO_MUX_PORT_LINES; iter++ )
    {
        uint32_t index = ioIndex[iter];
        uint32_t state = ioIndex_state[iter];

        if(status == SystemP_SUCCESS)
        {
            DebugP_log("index : %d\r\n", index);
            status = TCA6416_setOutput(
                            &gTCA6416_Config,
                            index,
                            state);
            /* Configure as output  */
            status += TCA6416_config(
                            &gTCA6416_Config,
                            index,
                            TCA6416_MODE_OUTPUT);
        }
        else
        {
            DebugP_log("failure to Select RESOLVER ADC Mux lines, index : %d\r\n", index);
            TCA6416_close(&gTCA6416_Config);
        }
    }

    TCA6416_close(&gTCA6416_Config);
}
