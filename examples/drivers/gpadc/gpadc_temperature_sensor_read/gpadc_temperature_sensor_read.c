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
 * This example configures a GPADC configuration for temperature sensors and triggers the
 * group channel conversion. Upon successful conversion temperature values for the sensors is displayed.
 * Conversion happens based on the parameters provided through syscfg
 */

void gpadc_temperature_sensor_read_main(void *args)
{
    int32_t status = SystemP_SUCCESS;
	GPADC_TempSensValueType tempValues;
	uint8_t numAverageSamples;

    Drivers_open();
    Board_driversOpen();

    memset(&tempValues,0,sizeof(tempValues));

    GPADC_initTempMeasurement();
    numAverageSamples = 5U;
    status = GPADC_readTemperature(numAverageSamples, MAX_GPADC_TEMP_SENSORS, &tempValues);

	if(SystemP_SUCCESS == status)
	{
		DebugP_log("Temperature read conversion successful\n");
	}
	else if(SystemP_FAILURE == status)
	{
		DebugP_log("Temperature read conversion unsuccessful\n");
	}

	DebugP_log("DSP Temp Sensor temp value %d\n", tempValues.DigDspTempValue);
	DebugP_log("HWA Temp Sensor temp value %d\n", tempValues.DigHwaTempValue);
	DebugP_log("HSM Temp Sensor temp value %d\n", tempValues.DigHsmTempValue);

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

