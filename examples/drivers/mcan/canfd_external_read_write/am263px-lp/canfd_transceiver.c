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
#include <board/ioexp/ioexp_tca6416.h>
#include "ti_board_open_close.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
#define IO_MUX_MCAN_STB                             (10U)                       /* PORT 1, PIN 2         -> ioIndex : 1*8 + 2 = 10 */
#define TCA6416_IO_MUX_MCAN_STB_PORT_LINE_STATE     (TCA6416_OUT_STATE_LOW)     /* MCAN_STB PIN OUTPUT   -> 0 */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
static TCA6416_Config  gTCA6416_Config;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
void mcanEnableTransceiver(void)
{
    int32_t             status = SystemP_SUCCESS;
    TCA6416_Params      tca6416Params;
    TCA6416_Params_init(&tca6416Params);

    status = TCA6416_open(&gTCA6416_Config, &tca6416Params);
    DebugP_assert(SystemP_SUCCESS == status);

    status = TCA6416_setOutput(
                    &gTCA6416_Config,
                    IO_MUX_MCAN_STB,
                    TCA6416_IO_MUX_MCAN_STB_PORT_LINE_STATE);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Configure as output  */
    status += TCA6416_config(
                    &gTCA6416_Config,
                    IO_MUX_MCAN_STB,
                    TCA6416_MODE_OUTPUT);

    if(status != SystemP_SUCCESS)
    {
        DebugP_log("Transceiver Setup Failure !!");
        TCA6416_close(&gTCA6416_Config);
    }

    TCA6416_close(&gTCA6416_Config);
}