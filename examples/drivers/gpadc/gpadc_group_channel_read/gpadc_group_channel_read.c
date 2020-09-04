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
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/*
 * This example configures a GPADC configuration for multi channel and triggers 
 * the multi channel conversion. * Upon successful converted adc conversion data 
 * is displayed. Conversion happens based on the parameters provided through syscfg
 */

#define LOWER_REFERENCE                 0
#define UPPER_REFERENCE                 1800
#define ADC_DEF_CHANNEL_RESOLUTION      (10U)

void gpadc_group_channel_read_main(void *args)
{
    int32_t status = SystemP_SUCCESS;
    uint16_t gpadc_result[9];
    GPADC_channelsGroupSelectType channels;
    GPADC_ConvResultType convRes = GPADC_CONV_ERROR;
    uint8_t index =0;
    uint32_t adcInMv;
 
    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    channels.bits.b9_ChannelSelectionBitMap = (CONFIG_GPADC0_CHANNEL_BITMAP & 0x1FF);

	GPADC_setupResultBuffer(&gpadc_result[0]);
	convRes = GPADC_startGroupConversion(channels,MAX_GPADC_MEAS_SOURCES);
	if(GPADC_CONV_DONE == convRes)
	{
		DebugP_log("Channel adc conversion successful\n");
		status = SystemP_SUCCESS;
	}
	else if(GPADC_CONV_CHANNEL_CONFIG_MISSING == convRes)
	{
		DebugP_log("channel adc configuration missing\n");
		status = SystemP_FAILURE;
	}
	else
	{
		DebugP_log("Channel adc conversion error\n");
		status = SystemP_FAILURE;
	}

	DebugP_log(" Channel\tHW_CH\t\tADC Value\tVolt\r\n");

	for(index=0; index < MAX_GPADC_MEAS_SOURCES; index++)
	{

		adcInMv = (gpadc_result[index] * (UPPER_REFERENCE -LOWER_REFERENCE))/
				(1<<ADC_DEF_CHANNEL_RESOLUTION);

		DebugP_log(
				" %4d\t\tADC_IN%d\t0x%08x\t%04dmV\r\n",
				index, index, gpadc_result[index], adcInMv);
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
