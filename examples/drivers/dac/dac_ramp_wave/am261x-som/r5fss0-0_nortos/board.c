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

#include <stdint.h>

#include <board/ioexp/ioexp_tca6408.h>

#include "ti_board_open_close.h"

#define IO_MUX_BP_BO_MUX_EN_N_PORT_LINE   (6)        // PORT 0, PIN 6    -> ioIndex : 6


/* checks the board version if CC E1/ E2 */
void i2c_io_expander_dac_out();
m
void i2c_io_expander_dac_out_gTCA6408();

void i2c_io_expander_dac_out(void)
{
    // int32_t status = SystemP_SUCCESS;
    // uint32_t boardVer_offset = 0x1AU;
    // uint32_t boardVer_length = 0x2U; // 2 char

    // uint8_t boardVer[2] = "";
    // status = EEPROM_read(gEepromHandle[CONFIG_EEPROM0], boardVer_offset, boardVer, boardVer_length);
    // if(status == SystemP_SUCCESS)
    // {
    //     /* read successful */
    //     if(boardVer[1] == '1')
    //     {
    //         /* boardVer is E1 */
    //         DebugP_log("Detected CC version is E1. Calling TCA6408 Drivers for io expander configurations\r\n");
    //         i2c_io_expander_dac_out_gTCA6408();
    //     }
    //     else if(boardVer[1] == '2')
    //     {
    //         /* boardVer is E2 */
    //         DebugP_log("Detected CC version is E2. Calling TCA6424 Drivers for io expander configurations\r\n");
    //         i2c_io_expander_dac_out_gTCA6424();
    //     }
    //     else
    //     {
    //         DebugP_log("invalid board read. Assuming CC version is E1. Calling TCA6408 Drivers for io expander configurations\r\n");
    //         i2c_io_expander_dac_out_gTCA6408();
    //     }
    // }
    // else
    // {
    //     DebugP_log("EEPROM read failed. aborting\r\n");
    //     DebugP_assert(0);
    // }
    i2c_io_expander_dac_out_gTCA6408();
}

void i2c_io_expander_dac_out_gTCA6408()
{
    static TCA6408_Config  gTCA6408_Config;
    int32_t             status = SystemP_SUCCESS;
    TCA6408_Params      tca6408Params;
    TCA6408_Params_init(&tca6408Params);

    TCA6408_Params.i2cAddress = 0x20U;  
    
    status = TCA6408_open(&gTCA6408_Config, &tca6408Params);
    
    // uint32_t ioIndex[TCA6408_IO_MUX_PORT_LINES] = {
    //     IO_MUX_ADC1_MUX_SEL_PORT_LINE,
    //     IO_MUX_ADC2_MUX_SEL_PORT_LINE,
    //     IO_MUX_ADC3_MUX_SEL_PORT_LINE,
    //     IO_MUX_ADC4_MUX_SEL_PORT_LINE,
    //     IO_MUX_ADC5_MUX_SEL_PORT_LINE,
    //     IO_MUX_ICSSM2_MUX_SEL_PORT_LINE,
    // };
    // uint32_t ioIndex_state[TCA6408_IO_MUX_PORT_LINES] = {
    //     TCA6408_IO_MUX_ADC1_MUX_SEL_PORT_LINE_STATE,
    //     TCA6408_IO_MUX_ADC2_MUX_SEL_PORT_LINE_STATE,
    //     TCA6408_IO_MUX_ADC3_MUX_SEL_PORT_LINE_STATE,
    //     TCA6408_IO_MUX_ADC4_MUX_SEL_PORT_LINE_STATE,
    //     TCA6408_IO_MUX_ADC5_MUX_SEL_PORT_LINE_STATE,
    //     TCA6408_IO_MUX_ICSSM2_MUX_SEL_PORT_LINE_STATE,
    // };

    // for(int iter = 0; iter < TCA6408_IO_MUX_PORT_LINES; iter++ )
    // {
        // uint32_t index = ioIndex[iter];
        // uint32_t state = ioIndex_state[iter];

        DebugP_log("Setting U57 : SN74CB3Q3257RGYR Mux Select Lines, From U23 : TCA6408ARGTR [0x20U addr] Index : 6, State : Low\r\n");

        status = TCA6408_setOutput(
                        &gTCA6408_Config,
                        IO_MUX_BP_BO_MUX_EN_N_PORT_LINE,
                        TCA6408_OUT_STATE_LOW);

        /* Configure as output  */
        status += TCA6408_config(
                        &gTCA6408_Config,
                        index,
                        TCA6408_MODE_OUTPUT);

        if(status != SystemP_SUCCESS)
        {
            DebugP_log("Failure in Setting U57 : SN74CB3Q3257RGYR Mux Select Lines, From U23 : TCA6408ARGTR [0x20U addr] Index : 6, State : Low");
            TCA6408_close(&gTCA6408_Config);
        }
    // }

    TCA6408_close(&gTCA6408_Config);
}
