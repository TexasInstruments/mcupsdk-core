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

#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/AddrTranslateP.h>
#include <kernel/dpl/HwiP.h>
#include <drivers/gpadc.h>
#include <string.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/*
 * This example configures a GPADC configuration for single channel and 
 * triggers the single channel conversion. Upon successful converted adc conversion 
 * data is displayed. Conversion happens based on the parameters provided through syscfg
 */
extern GPADC_ConfigType gCfgPtr;
void gpadc_single_channel_read_main(void *args)
{
    int32_t status = SystemP_SUCCESS;
    uint16_t gpadc_result = 0;
    uint16_t channelCount;
    GPADC_ConvResultType convRes = GPADC_CONV_ERROR;

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    for(channelCount=0; channelCount<CONFIG_GPADC_CHANNEL_NUM_INSTANCES; channelCount++)
    {
        convRes = GPADC_startSingleChannelConversion(gCfgPtr.channelConfig[channelCount].channelID, &gpadc_result);
	
        if(GPADC_CONV_DONE == convRes)
        {
            DebugP_log("Channel adc conversion successful\n");
            status = SystemP_SUCCESS;
        }
        else if(GPADC_CONV_CHANNEL_CONFIG_MISSING == convRes)
        {
            DebugP_log("Channel adc configuration missing\n");
            status = SystemP_FAILURE;
        }
        else
        {
            DebugP_log("Channel adc conversion error\n");
            status = SystemP_FAILURE;
        }

        DebugP_log("GPADC EXT%d  AvgValue %d\r\n\n",(gCfgPtr.channelConfig[channelCount].channelID+1),gpadc_result);
    }

    if(SystemP_SUCCESS == status)
    {
        DebugP_log("All tests have passed!!\r\n");
    }
    else
    {
        DebugP_log("Some tests have failed!!\r\n");
    }

    Board_driversClose();
    Drivers_close();
}

