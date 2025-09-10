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
#include <board/ioexp/ioexp_tca6424.h>

#include "ti_board_open_close.h"

#define IO_MUX_ADC1_MUX_SEL_PORT_LINE   (14)        // PORT 1, PIN 6    -> ioIndex : 1*8 + 6 = 14
#define IO_MUX_ADC2_MUX_SEL_PORT_LINE   (15)        // PORT 1, PIN 7    -> ioIndex : 1*8 + 7 = 15
#define IO_MUX_ADC3_MUX_SEL_PORT_LINE   (5)         // PORT 0, PIN 5    -> ioIndex : 0*8 + 5 = 5
#define IO_MUX_ADC4_MUX_SEL_PORT_LINE   (6)         // PORT 0, PIN 6    -> ioIndex : 0*8 + 6 = 6
#define IO_MUX_ADC5_MUX_SEL_PORT_LINE   (8)         // PORT 1, PIN 0    -> ioIndex : 1*8 + 0 = 8
#define IO_MUX_ICSSM1_MUX_SEL_PORT_LINE (2)         // PORT 0, PIN 2    -> ioIndex : 0*8 + 2 = 2
#define IO_MUX_ICSSM2_MUX_SEL_PORT_LINE (3)         // PORT 0, PIN 3    -> ioIndex : 0*8 + 3 = 3

/* Macros to be used for E1 version of Control Card */
#define TCA6416_IO_MUX_ADC1_MUX_SEL_PORT_LINE_STATE     (TCA6424_OUT_STATE_LOW)        // configures ADC0_AIN0, ADC0_AIN1
#define TCA6416_IO_MUX_ADC2_MUX_SEL_PORT_LINE_STATE     (TCA6416_OUT_STATE_LOW)         // configures ADC_CALx/ADC_R0_AIN(0,1)
#define TCA6416_IO_MUX_ADC3_MUX_SEL_PORT_LINE_STATE     (TCA6416_OUT_STATE_LOW)         // configures ADC_R0_AIN(0,1) against CAL pins
#define TCA6416_IO_MUX_ADC4_MUX_SEL_PORT_LINE_STATE     (TCA6416_OUT_STATE_HIGH)        // configures ADC_R0_AIN(2,3) and ADC_R1_AIN(0,1)
#define TCA6416_IO_MUX_ADC5_MUX_SEL_PORT_LINE_STATE     (TCA6416_OUT_STATE_HIGH)        // configures ADC_R1_AIN(2,3) and RES0_PWMOUT0
#define TCA6416_IO_MUX_ICSSM2_MUX_SEL_PORT_LINE_STATE   (TCA6416_OUT_STATE_HIGH)        // configures RES0_PWMOUT0
#define TCA6416_IO_MUX_PORT_LINES (6)


/* Macros to be used for E2 version of Control Card */
#define TCA6424_IO_MUX_ADC1_MUX_SEL_PORT_LINE_STATE   (TCA6424_OUT_STATE_LOW)           // configures ADC0_AIN0, ADC0_AIN1
#define TCA6424_IO_MUX_ADC2_MUX_SEL_PORT_LINE_STATE   (TCA6424_OUT_STATE_LOW)            // configures ADC_CALx/ADC_R0_AIN(0,1)
#define TCA6424_IO_MUX_ADC3_MUX_SEL_PORT_LINE_STATE   (TCA6424_OUT_STATE_LOW)            // configures ADC_R1_AIN1 ADC_R0_AIN1(0,1)
#define TCA6424_IO_MUX_ADC4_MUX_SEL_PORT_LINE_STATE   (TCA6424_OUT_STATE_HIGH)           // configures ADC_R0_AIN(2,3) and ADC_R1_AIN(2,3)
#define TCA6424_IO_MUX_ADC5_MUX_SEL_PORT_LINE_STATE   (TCA6424_OUT_STATE_HIGH)           // configures ADC_R0_AIN0 ADC_R1_AIN0 and RES0_PWMOUT0
#define TCA6424_IO_MUX_ICSSM1_MUX_SEL_PORT_LINE_STATE (TCA6424_OUT_STATE_HIGH)           // configures RES0_PWMOUT0
#define TCA6424_IO_MUX_PORT_LINES (6)

/* checks the board version if CC E1/ E2 */
void i2c_io_expander_resolver_adc();
void i2c_io_expander_resolver_adc_gTCA6416();
void i2c_io_expander_resolver_adc_gTCA6424();

void i2c_io_expander_resolver_adc(void)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t boardVer_offset = 0x1AU;
    uint32_t boardVer_length = 0x2U; // 2 char

    uint8_t boardVer[2] = "";
    status = EEPROM_read(gEepromHandle[CONFIG_EEPROM0], boardVer_offset, boardVer, boardVer_length);
    if(status == SystemP_SUCCESS)
    {
        /* read successful */
        if(boardVer[1] == '1' && boardVer[0] == 'E')
        {
            /* boardVer is E1 */
            DebugP_log("Calling TCA6416 Drivers for io expander configurations\r\n");
            i2c_io_expander_resolver_adc_gTCA6416();
        }
        else
        {
            /* boardVer is E2 or REV A or Rev B */
            DebugP_log("Calling TCA6424 Drivers for io expander configurations\r\n");
            i2c_io_expander_resolver_adc_gTCA6424();
        }
    }
    else
    {
        DebugP_log("EEPROM read failed. aborting\r\n");
        DebugP_assert(0);
    }
}

void i2c_io_expander_resolver_adc_gTCA6416()
{
    static TCA6416_Config  gTCA6416_Config;
    int32_t             status = SystemP_SUCCESS;
    TCA6416_Params      tca6416Params;
    TCA6416_Params_init(&tca6416Params);
    status = TCA6416_open(&gTCA6416_Config, &tca6416Params);
    uint32_t ioIndex[TCA6416_IO_MUX_PORT_LINES] = {
        IO_MUX_ADC1_MUX_SEL_PORT_LINE,
        IO_MUX_ADC2_MUX_SEL_PORT_LINE,
        IO_MUX_ADC3_MUX_SEL_PORT_LINE,
        IO_MUX_ADC4_MUX_SEL_PORT_LINE,
        IO_MUX_ADC5_MUX_SEL_PORT_LINE,
        IO_MUX_ICSSM2_MUX_SEL_PORT_LINE,
    };
    uint32_t ioIndex_state[TCA6416_IO_MUX_PORT_LINES] = {
        TCA6416_IO_MUX_ADC1_MUX_SEL_PORT_LINE_STATE,
        TCA6416_IO_MUX_ADC2_MUX_SEL_PORT_LINE_STATE,
        TCA6416_IO_MUX_ADC3_MUX_SEL_PORT_LINE_STATE,
        TCA6416_IO_MUX_ADC4_MUX_SEL_PORT_LINE_STATE,
        TCA6416_IO_MUX_ADC5_MUX_SEL_PORT_LINE_STATE,
        TCA6416_IO_MUX_ICSSM2_MUX_SEL_PORT_LINE_STATE,
    };

    for(int iter = 0; iter < TCA6416_IO_MUX_PORT_LINES; iter++ )
    {
        uint32_t index = ioIndex[iter];
        uint32_t state = ioIndex_state[iter];

        DebugP_log("Setting RESOLVER ADC Mux Select Lines, Index : %d, State : %d\r\n", index, state);

        status = TCA6416_setOutput(
                        &gTCA6416_Config,
                        index,
                        state);

        /* Configure as output  */
        status += TCA6416_config(
                        &gTCA6416_Config,
                        index,
                        TCA6416_MODE_OUTPUT);

        if(status != SystemP_SUCCESS)
        {
            DebugP_log("Failure to Set RESOLVER ADC Mux Select lines, at index : %d\r\n", index);
            TCA6416_close(&gTCA6416_Config);
        }
    }

    TCA6416_close(&gTCA6416_Config);
}

void i2c_io_expander_resolver_adc_gTCA6424()
{
    static TCA6424_Config  gTCA6424_Config;
    int32_t             status = SystemP_SUCCESS;
    TCA6424_Params      tca6424Params;
    TCA6424_Params_init(&tca6424Params);
    status = TCA6424_open(&gTCA6424_Config, &tca6424Params);
    uint32_t ioIndex[TCA6424_IO_MUX_PORT_LINES] = {
        IO_MUX_ADC1_MUX_SEL_PORT_LINE,
        IO_MUX_ADC2_MUX_SEL_PORT_LINE,
        IO_MUX_ADC3_MUX_SEL_PORT_LINE,
        IO_MUX_ADC4_MUX_SEL_PORT_LINE,
        IO_MUX_ADC5_MUX_SEL_PORT_LINE,
        IO_MUX_ICSSM1_MUX_SEL_PORT_LINE,
    };
    uint32_t ioIndex_state[TCA6424_IO_MUX_PORT_LINES] = {
        TCA6424_IO_MUX_ADC1_MUX_SEL_PORT_LINE_STATE,
        TCA6424_IO_MUX_ADC2_MUX_SEL_PORT_LINE_STATE,
        TCA6424_IO_MUX_ADC3_MUX_SEL_PORT_LINE_STATE,
        TCA6424_IO_MUX_ADC4_MUX_SEL_PORT_LINE_STATE,
        TCA6424_IO_MUX_ADC5_MUX_SEL_PORT_LINE_STATE,
        TCA6424_IO_MUX_ICSSM1_MUX_SEL_PORT_LINE_STATE,
    };

    for(int iter = 0; iter < TCA6424_IO_MUX_PORT_LINES; iter++ )
    {
        uint32_t index = ioIndex[iter];
        uint32_t state = ioIndex_state[iter];

        DebugP_log("Setting RESOLVER ADC Mux Select Lines, Index : %d, State : %d\r\n", index, state);

        status = TCA6424_setOutput(
                        &gTCA6424_Config,
                        index,
                        state);

        /* Configure as output  */
        status += TCA6424_config(
                        &gTCA6424_Config,
                        index,
                        TCA6424_MODE_OUTPUT);

        if(status != SystemP_SUCCESS)
        {
            DebugP_log("Failure to Set RESOLVER ADC Mux Select lines, at index : %d\r\n", index);
            TCA6424_close(&gTCA6424_Config);
        }
    }

    TCA6424_close(&gTCA6424_Config);
}
