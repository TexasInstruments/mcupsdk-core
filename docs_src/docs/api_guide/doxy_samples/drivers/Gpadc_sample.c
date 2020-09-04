
#include <stdio.h>
//! [include]
#include <drivers/gpadc.h>
//! [include]
#include <drivers/hw_include/cslr_soc.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/HwiP.h>

extern GPADC_ConfigType gCfgPtr;
void gpadcSingleBufferRead(void)
{
//! [singleBufferRead]
    int32_t status = SystemP_SUCCESS;
    uint16_t gpadc_result = 0;
    uint16_t channelCount;
    GPADC_ConvResultType convRes = GPADC_CONV_ERROR;
    
	for(channelCount=0; channelCount<CONFIG_GPADC_CHANNEL_NUM_INSTANCES; channelCount++)
    {
        convRes = GPADC_startSingleChannelConversion(gCfgPtr.channelConfig[channelCount].channelID, &gpadc_result);
    }

//! [singleBufferRead]
}

void gpadcGroupBufferRead(void)
{
//! [groupBufferRead]
    int32_t status = SystemP_SUCCESS;
    uint16_t gpadc_result[9];
    GPADC_channelsGroupSelectType channels;
    GPADC_ConvResultType convRes = GPADC_CONV_ERROR;
    uint8_t index =0;
    uint32_t adcInMv;

    channels.bits.b9_ChannelSelectionBitMap = (CONFIG_GPADC0_CHANNEL_BITMAP & 0x1FF);

	GPADC_setupResultBuffer(&gpadc_result[0]);
	convRes = GPADC_startGroupConversion(channels);

	for(index=0; index < MAX_GPADC_MEAS_SOURCES; index++)
	{

		adcInMv = (gpadc_result[index] * (UPPER_REFERENCE -LOWER_REFERENCE))/
				(1<<ADC_DEF_CHANNEL_RESOLUTION);
	}

//! [groupBufferRead]
}


void gpadcTempSensorRead()
{
//! [tempSensorRead]
	int32_t convRes;
	GPADC_TempSensValueType tempValues;
	uint8_t numAverageSamples;

    GPADC_initTempMeasurement();
    numAverageSamples = 5U;
    convRes = GPADC_readTemperature(numAverageSamples, &tempValues);

//! [tempSensorRead]
}
