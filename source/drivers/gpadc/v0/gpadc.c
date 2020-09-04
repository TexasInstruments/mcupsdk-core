/*
 * Copyright (C) 2021 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 *  \file   gpadc.c
 *
 *  \brief  The file implements the Platform specific GPADC Driver Interface
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <string.h>
#include <drivers/gpadc.h>
#include <drivers/hw_include/hw_types.h>
#include <kernel/dpl/CycleCounterP.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

static uint8_t GPADC_convSkipTimeToCode(uint32_t skipSamples);
static void GPADC_convert(uint32_t cfgVal,
                          uint32_t paramVal,
                          GPADC_ResultType *gpAdcResult);
static void GPADC_cfg(uint32_t cfgVal,
                      uint32_t paramVal);
static void GPADC_readSamplingResult(uint8_t numSamples,
                                     GPADC_ResultType *gpAdcResult);
static uint16_t GPADC_efuseExtractTrims (volatile uint32_t reg,
                                        uint8_t msb, uint8_t lsb);
static int32_t GPADC_calculateTemp(uint16_t gpadcTempCode,
                                   uint8_t index);
static void GPADC_SWTrigger(void);
static void GPADC_computeTempSlope(void);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/** \brief GPADC Initialization check flag */
Bool isGPADCInitialized = FALSE;
/** \brief Externally defined GPADC Driver object */
GPADC_DriverObjectType GPADCDrvObj;
/** \brief Temperature sensor trim Slop Values */
GPADC_TempSensTrimType tempSensTrimSlopeValues;
/** \brief Efuse Temperature trim Values */
GPADC_EfuseTempTrimType efuseTempTrimValues;


/** \brief TopRCM Base Address */
CSL_top_ctrlRegs *topCtrlRegs = (CSL_top_ctrlRegs *)CSL_MSS_TOPRCM_U_BASE;
/** \brief Externally defined  GPADC Param Lut*/
extern GPADC_CfgAndParamValuesType GPADC_ConfigParamLuTab[MAX_GPADC_MEAS_SOURCES];
/** \brief Externally defined  GPADC Temperature Sensors Param Lookup Table*/
extern GPADC_TempSensMuxType GPADC_TempSensConfigParamTab[MAX_GPADC_TEMP_SENSORS];

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void GPADC_init(void)
{
    return;
}

void GPADC_deinit(void)
{
    return;
}

int32_t GPADC_open(GPADC_ConfigType *CfgPtr)
{
    uint8_t divider;
    uint32_t profRamAdd;
    uint8_t channelNumber;
    uint32_t regWrSts = 0U;
    uint8_t  channelIndex = 0U;
    uint32_t skipSamples = 0U;
    uint8_t skipSamplesReg = 0U;
    int32_t status = SystemP_SUCCESS;
    GPADC_ConfigType *ConfigPtr;
    
	if(TRUE == isGPADCInitialized)
	{
		 status = SystemP_FAILURE;
	}

    if(NULL == CfgPtr)
    {
		DebugP_assert(NULL == CfgPtr);
    }
    else
    {
        ConfigPtr = CfgPtr;
        memset(&GPADCDrvObj,0,sizeof(GPADCDrvObj));

        GPADCDrvObj.convMode = ConfigPtr->convMode;
        GPADCDrvObj.triggSrc = ConfigPtr->triggSrc;
        GPADCDrvObj.driverStatus = GPADC_IDLE;

        for (channelIndex = 0U; channelIndex < MAX_GPADC_MEAS_SOURCES; channelIndex++)
        {
            if(ConfigPtr->channelConfig[channelIndex].isConfigured)
            {
                channelNumber = ConfigPtr->channelConfig[channelIndex].channelID;
                GPADCDrvObj.driverChannelConfig[channelNumber].isChannelConfigured = TRUE;
                GPADCDrvObj.driverChannelConfig[channelNumber].isChannelBufferedMode = ConfigPtr->channelConfig[channelIndex].isBufferedMode;


                if(ConfigPtr->channelConfig[channelIndex].useLuTable)
                {
                    if(ConfigPtr->channelConfig[channelIndex].isBufferedMode)
                    {
                        GPADCDrvObj.driverChannelConfig[channelNumber].channelConfigValue = GPADC_ConfigParamLuTab[channelNumber].BuffConfigValue;
                    }
                    else
                    {
                        GPADCDrvObj.driverChannelConfig[channelNumber].channelConfigValue = GPADC_ConfigParamLuTab[channelNumber].UnbuffConfigValue;
                    }

                    GPADCDrvObj.driverChannelConfig[channelNumber].channelParamValue.b32_Val = GPADC_ConfigParamLuTab[channelNumber].ParamInfo.b32_Val;
                }
                else
                {
                    if(channelIndex >= MAX_GPADC_TEMP_SENSORS)
                    {
                        break;
                    }
                    GPADCDrvObj.driverChannelConfig[channelIndex].channelConfigValue = GPADC_TempSensConfigParamTab[channelIndex].channelConfigValue;

                    GPADCDrvObj.driverChannelConfig[channelIndex].channelParamValue.bits.b8_ParamValue = GPADC_TempSensConfigParamTab[channelIndex].channelParamValue;

                    GPADCDrvObj.driverChannelConfig[channelIndex].channelParamValue.bits.b8_CollectSamples = GPADC_TempSensConfigParamTab[channelIndex].collectSamples;
                    skipSamples = GPADC_TempSensConfigParamTab[channelIndex].skipSamples;;
                    skipSamplesReg = GPADC_convSkipTimeToCode(skipSamples);
                    GPADCDrvObj.driverChannelConfig[channelIndex].channelParamValue.bits.b7_SkipSamples = (((uint32_t)skipSamplesReg & (uint32_t)0x7FU));
                    GPADCDrvObj.driverChannelConfig[channelIndex].channelParamValue.bits.b9_Reserved = 0U; // Clear reserved 9 bits
                }
            }
        }

        /*
         * Source sysclk 200MHz
         * Configure sys clock divider as 0x000 in multibit which is 200/1
         */
        REG_STRUCT_SWRITE(MSS_GPADC_CLK_DIV_VAL_PTR->b24_Clkdivr, 0x131313U,  regWrSts);

        /*
         * writing '111' will gate clock for MSS GPADC
         */
        REG_STRUCT_SWRITE(MSS_GPADC_CLK_GATE_PTR->b3_Gated, 0U,  regWrSts);

        /*
         * Configure the clock divider for GPADC Module
         * GPADC Clock = 10MHz = source_clock/Divider
         * source_clock can be 200 or xtal clock
         */
        divider = (20U);
        REG_STRUCT_SWRITE(GPADC_REGS_PTR->r_Reg8.b8_GpadcClkDiv, divider,  regWrSts);


        REG_STRUCT_SWRITE(GPADC_REGS_PTR->r_Reg1.b1_GpadcInit, 1U, regWrSts);
        REG_STRUCT_SWRITE(GPADC_REGS_PTR->r_Reg1.b1_GpadcFsmBypass, 0U, regWrSts);

        /* Enable the GPADC FSM clock */
        REG_STRUCT_SWRITE(GPADC_REGS_PTR->r_Reg0.b1_GpadcFsmClkEnable, 1U, regWrSts);

        /* Enable the GPADC clock */    
        REG_STRUCT_SWRITE(GPADC_REGS_PTR->r_Reg8.b1_GpadcClkEnable, 1U, regWrSts);

        /* Write the Mode of operation */
        REG_STRUCT_SWRITE(GPADC_REGS_PTR->r_Reg0.b2_DcbistMode, GPADC_MODE_DISABLE, \
                                                                                        regWrSts);

        /* Write config value for IFM Mode */
        REG_STRUCT_SWRITE(GPADC_REGS_PTR->r_Reg2.b32_ConfigValueIfm, 0U, regWrSts);

        /* Write Skip Samples for IFM Mode */
        REG_STRUCT_SWRITE(GPADC_REGS_PTR->r_Reg3.bits.b7_SkipSamplesIfm, 0U, regWrSts);

        /* Write Collect Samples for IFM Mode */
        REG_STRUCT_SWRITE(GPADC_REGS_PTR->r_Reg3.bits.b8_CollectSamplesIfm, 0U, regWrSts);

        /* Write Param value for IFM Mode */
        REG_STRUCT_SWRITE(GPADC_REGS_PTR->r_Reg3.bits.b8_ParamValIfm, 0U, regWrSts);

        /* Write Packet RAM Base address for CTM Mode */
        REG_STRUCT_SWRITE(GPADC_REGS_PTR->r_PacketRamAdd[0].b32_Reg, 0U, regWrSts);
        REG_STRUCT_SWRITE(GPADC_REGS_PTR->r_PacketRamAdd[1].b32_Reg, 0U, regWrSts);
        REG_STRUCT_SWRITE(GPADC_REGS_PTR->r_PacketRamAdd[2].b32_Reg, 0U, regWrSts);

        /*!
         * Profile 15 Instruction Ram address update with GPADC_MON_INSTR_RAM_ST_IND for monitoring
         */
        profRamAdd = ((uint32_t)GPADC_MON_INSTR_RAM_ST_IND << 24U) & 0xFF000000U;
        REG_STRUCT_SWRITE(GPADC_REGS_PTR->r_PacketRamAdd[3].b32_Reg, profRamAdd, regWrSts);

        if(GPADC_CONTINUOUS_CONV_MODE == GPADCDrvObj.convMode)
        {
            REG_STRUCT_SWRITE(MSS_CTRL_MSS_GPADC_MEM_INIT_PTR->b1_mem0_init, 1U,  regWrSts);
        }

        /* Release Reset for GPADC Analog and Digital Module */
        GPADC_socResetRelease();

        isGPADCInitialized = TRUE;
    }
    return status;
}

GPADC_ConvResultType GPADC_startGroupConversion(GPADC_channelsGroupSelectType channels, uint8_t numChannels )
{
    uint32_t cfgVal;
	uint32_t paramVal;
    uint8_t channelIndex = 0;
    uint32_t regWrSts = 0U;
	GPADC_ConvResultType retVal;
	GPADC_ResultType gpAdcResult;
    GPADCDrvObj.driverStatus = GPADC_BUSY;
    retVal = GPADC_CONV_DONE;

    if(FALSE == isGPADCInitialized)
    {
        retVal = GPADC_CONV_ERROR;
    }

    else if(0U == channels.b16_Val)
    {
        retVal = GPADC_CONV_ERROR;
    }
    else if (NULL == GPADCDrvObj.ResultBufferPtr)
    {
        retVal = GPADC_CONV_ERROR;
    }
    else
    {
        /* Max Check for the GPADC supported measurement list */
        for(channelIndex =0; channelIndex < numChannels; channelIndex++)
        {
            if(channels.b16_Val & (0x01 << channelIndex))
            {
                if(GPADCDrvObj.driverChannelConfig[channelIndex].isChannelConfigured)
                {
                    if(GPADCDrvObj.driverChannelConfig[channelIndex].isChannelBufferedMode)
                    {
                        REG_STRUCT_SWRITE(MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_PTR->b1_AdcInpBufEn, 1U, regWrSts);
                    }
                    else
                    {
                        REG_STRUCT_SWRITE(MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_PTR->b1_AdcInpBufEn, 0U, regWrSts);
                    }
                    /* Check if the GPADC is not in use before using it */
                    DebugP_assert(GPADC_MODE_DISABLE == GPADC_REGS_PTR->r_Reg0.b2_DcbistMode);

                        /* Change the Mode of operation to IFM */
                    REG_STRUCT_SWRITE(GPADC_REGS_PTR->r_Reg0.b2_DcbistMode, GPADC_MODE_IFM, regWrSts);
                    REG_STRUCT_SWRITE(GPADC_REGS_PTR->r_Reg0.b1_GpadcDebugModeEnable, 1U, regWrSts);

                    /* Get the Config Value from DriverObj */
                    cfgVal = GPADCDrvObj.driverChannelConfig[channelIndex].channelConfigValue;
                    /* Get the Param Value from DriverObj */
                    paramVal = GPADCDrvObj.driverChannelConfig[channelIndex].channelParamValue.b32_Val;

                    /* Call the GPADC function to convert the param from ana to dig */
                    GPADC_convert(cfgVal, paramVal, &gpAdcResult);

                    GPADCDrvObj.ResultBufferPtr[channelIndex] = gpAdcResult.Avg;

                }
                else
                {
                    GPADCDrvObj.ResultBufferPtr[channelIndex] = 0U;
                    retVal = GPADC_CONV_CHANNEL_CONFIG_MISSING;
                }
            }
            else
            {
                GPADCDrvObj.ResultBufferPtr[channelIndex] = 0U;
            }
        }

        GPADCDrvObj.driverStatus = GPADC_COMPLETED;
    }

	return retVal;
}

GPADC_ConvResultType GPADC_startSingleChannelConversion(GPADC_MeasExtSrcType channelID, uint16_t *gpadcValue)
{
    uint32_t regWrSts = 0U;
	uint32_t cfgVal;
	uint32_t paramVal;
    GPADC_ConvResultType retVal;
    GPADC_ResultType gpAdcResult;

    if(FALSE == isGPADCInitialized)
    {
        retVal = GPADC_CONV_ERROR;
    }
    else if(MAX_GPADC_MEAS_SOURCES <= channelID)
    {
        retVal = GPADC_CONV_ERROR;
    }
    else if(NULL == gpadcValue)
    {
        retVal = GPADC_CONV_ERROR;
    }
    else
    {
        if(GPADCDrvObj.driverChannelConfig[channelID].isChannelConfigured)
        {
            if(GPADCDrvObj.driverChannelConfig[channelID].isChannelBufferedMode)
            {
                REG_STRUCT_SWRITE(MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_PTR->b1_AdcInpBufEn, 1U, regWrSts);
            }
            else
            {
                REG_STRUCT_SWRITE(MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_PTR->b1_AdcInpBufEn, 0U, regWrSts);
            }
            /* Check if the GPADC is not in use before using it */
            DebugP_assert(GPADC_MODE_DISABLE == GPADC_REGS_PTR->r_Reg0.b2_DcbistMode);

                /* Change the Mode of operation to IFM */
            REG_STRUCT_SWRITE(GPADC_REGS_PTR->r_Reg0.b2_DcbistMode, GPADC_MODE_IFM, regWrSts);
            REG_STRUCT_SWRITE(GPADC_REGS_PTR->r_Reg0.b1_GpadcDebugModeEnable, 1U, regWrSts);

            /* Get the Config Value from DriverObj */
            cfgVal = GPADCDrvObj.driverChannelConfig[channelID].channelConfigValue;
            /* Get the Param Value from DriverObj */
            paramVal = GPADCDrvObj.driverChannelConfig[channelID].channelParamValue.b32_Val;

            /* Call the GPADC function to convert the param from ana to dig */
            GPADC_convert(cfgVal, paramVal, &gpAdcResult);

            *gpadcValue = gpAdcResult.Avg;

            retVal = GPADC_CONV_DONE;
        }
        else
        {
            *gpadcValue = 0U;
            retVal = GPADC_CONV_CHANNEL_CONFIG_MISSING;
        }
     }

	return retVal;
}

int32_t GPADC_stopConversion(void)
{
	int32_t status = SystemP_SUCCESS;
    uint32_t regWrSts = 0U;

    if(FALSE == isGPADCInitialized)
    {
		status = SystemP_FAILURE;
    }
	else if(GPADC_IDLE == GPADCDrvObj.driverStatus)
	{
		status = SystemP_FAILURE;
	}
	else
	{
		/* Disable GPADC after use */
        REG_STRUCT_SWRITE(GPADC_REGS_PTR->r_Reg0.b2_DcbistMode, GPADC_MODE_DISABLE, regWrSts);
        GPADCDrvObj.driverStatus = GPADC_IDLE;
		status = SystemP_SUCCESS;
	}

	return status;
}

int32_t GPADC_setupResultBuffer(uint16_t * ResBufferPtr)
{
    int32_t status = SystemP_SUCCESS;
	if(FALSE == isGPADCInitialized)
	{
		 status = SystemP_FAILURE;
	}
	else if(NULL == ResBufferPtr)
	{
        DebugP_assert(NULL == ResBufferPtr);
        status = SystemP_FAILURE;
	}
	else if(GPADC_IDLE != GPADCDrvObj.driverStatus)
	{
		status = SystemP_FAILURE;
	}
	else
	{
		GPADCDrvObj.ResultBufferPtr = ResBufferPtr;
	}

	return status;
}

int32_t GPADC_readResultBuffer(uint16_t *ResBufferPtr)
{
	uint8_t index;
	int32_t status = SystemP_SUCCESS;

	if(FALSE == isGPADCInitialized)
	{
		status = SystemP_FAILURE;
	}
	else if(NULL == ResBufferPtr)
	{
		status = SystemP_FAILURE;
	}
	else if(GPADC_IDLE == GPADCDrvObj.driverStatus)
	{
		status = SystemP_FAILURE;
	}
	else
	{
	    for (index = 0U; index < MAX_GPADC_MEAS_SOURCES; index++)
	    {
            ResBufferPtr[index] = GPADCDrvObj.ResultBufferPtr[index];
	    }

	    GPADCDrvObj.driverStatus = GPADC_IDLE;
	    status = SystemP_SUCCESS;
	}

    return status;
}

GPADC_StatusType GPADC_getStatus(void)
{
	GPADC_StatusType groupStatus = GPADC_IDLE;

	if(FALSE == isGPADCInitialized)
	{
		DebugP_assert(NULL == isGPADCInitialized);
	}
	else
	{
		groupStatus = GPADCDrvObj.driverStatus;
	}

	return groupStatus;
}

int32_t GPADC_close(void)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t regWrSts = 0U;

    if(FALSE == isGPADCInitialized)
    {
        status = SystemP_FAILURE;
    }
    else if(GPADC_BUSY == GPADCDrvObj.driverStatus)
    {
        status = SystemP_FAILURE;
    }
    else
    {
        /*
         * Release Reset GPADC for digital FSM
         * Writing '111' or '0x07' will reset MSS GPADC
         */
        REG_STRUCT_SWRITE(MSS_GPADC_RST_CTRL_PTR->b3_Assert, GPADC_FSM_ASSERT_RESET, regWrSts);

        REG_STRUCT_SWRITE(MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_PTR->b1_ClkTmuxEsdCtrl, 0U, regWrSts);
        REG_STRUCT_SWRITE(MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_PTR->b1_AnaTestEn, 0U, regWrSts);

        REG_STRUCT_SWRITE(MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_PTR->b1_AnaogTestTmuxEsdCtrl, 0U, regWrSts);

        /* Reset GPADC Ana */
        REG_STRUCT_SWRITE(MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_PTR->b1_AdcReset, GPADC_ASSERT_RESET, regWrSts);
        /* Disable GPADC Ana */
        REG_STRUCT_SWRITE(MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_PTR->b1_AdcEn, GPADC_DISABLE, regWrSts);

        /*! Assert if Register read back test failed */
         DebugP_assert(NULL == regWrSts);

        isGPADCInitialized = FALSE;
        status = SystemP_SUCCESS;
    }

    return status;
}

void GPADC_initTempMeasurement(void)
{
    int16_t trimTemp;
    memset(&efuseTempTrimValues,0,sizeof(efuseTempTrimValues));

    efuseTempTrimValues.FuseROMVer = GPADC_efuseExtractTrims(topCtrlRegs->EFUSE1_ROW_14, \
            EFUSE1_ROW_14_FUSEROM_VER_STOP_BIT, EFUSE1_ROW_14_FUSEROM_VER_START_BIT);
	trimTemp = GPADC_efuseExtractTrims(topCtrlRegs->EFUSE1_ROW_36, \
    		EFUSE1_ROW_36_TRIM_TEMPERATURE_30C_STOP_BIT , EFUSE1_ROW_36_TRIM_TEMPERATURE_30C_START_BIT);

    efuseTempTrimValues.TrimTemp30C = trimTemp;

    trimTemp = GPADC_efuseExtractTrims(topCtrlRegs->EFUSE1_ROW_33, \
            EFUSE1_ROW_33_TRIM_TEMPERATURE_125C_STOP_BIT , EFUSE1_ROW_33_TRIM_TEMPERATURE_125C_START_BIT);

    efuseTempTrimValues.TrimTemp125C = trimTemp;


	if((0U == efuseTempTrimValues.FuseROMVer) && (0U == efuseTempTrimValues.TrimTemp30C) \
			&& (0U == efuseTempTrimValues.TrimTemp125C))
	{
		tempSensTrimSlopeValues.TrimTemp30C = ZERO;
		tempSensTrimSlopeValues.TrimTemp125C = ZERO;

		tempSensTrimSlopeValues.TrimIntercept30C[GPADC_DIG_DSP_TEMP_SENSOR] = ZERO_PT_TRIM_FIXED_DIG_TEMP_SENSOR_TRIM_30C;
		tempSensTrimSlopeValues.TrimIntercept30C[GPADC_DIG_HWA_TEMP_SENSOR] = ZERO_PT_TRIM_FIXED_DIG_TEMP_SENSOR_TRIM_30C;
		tempSensTrimSlopeValues.TrimIntercept30C[GPADC_DIG_HSM_TEMP_SENSOR] = ZERO_PT_TRIM_FIXED_DIG_TEMP_SENSOR_TRIM_30C;

		tempSensTrimSlopeValues.TrimIntercept125C[GPADC_DIG_DSP_TEMP_SENSOR] = ZERO;
		tempSensTrimSlopeValues.TrimIntercept125C[GPADC_DIG_HWA_TEMP_SENSOR] = ZERO;
		tempSensTrimSlopeValues.TrimIntercept125C[GPADC_DIG_HSM_TEMP_SENSOR] = ZERO;

		tempSensTrimSlopeValues.Slope[GPADC_DIG_DSP_TEMP_SENSOR] = ZERO_PT_TRIM_FIXED_SLOPE;
		tempSensTrimSlopeValues.Slope[GPADC_DIG_HWA_TEMP_SENSOR] = ZERO_PT_TRIM_FIXED_SLOPE;
		tempSensTrimSlopeValues.Slope[GPADC_DIG_HSM_TEMP_SENSOR] = ZERO_PT_TRIM_FIXED_SLOPE;

		tempSensTrimSlopeValues.InterceptTemp = ZERO_PT_TRIM_FIXED_TRIM_TEMP;
	}
	else if((1U == efuseTempTrimValues.FuseROMVer) || (0U != efuseTempTrimValues.TrimTemp30C) \
			|| (0U != efuseTempTrimValues.TrimTemp125C))
	{
		tempSensTrimSlopeValues.TrimTemp30C = efuseTempTrimValues.TrimTemp30C;
		tempSensTrimSlopeValues.TrimTemp125C = efuseTempTrimValues.TrimTemp125C;

		efuseTempTrimValues.TrimIntercept30C[GPADC_DIG_DSP_TEMP_SENSOR] = GPADC_efuseExtractTrims(topCtrlRegs->EFUSE1_ROW_37, \
				EFUSE1_ROW_37_DIG_DSP_TEMP_SENSOR_TRIM0_30C_STOP_BIT, EFUSE1_ROW_37_DIG_DSP_TEMP_SENSOR_TRIM0_30C_START_BIT);
		efuseTempTrimValues.TrimIntercept30C[GPADC_DIG_HWA_TEMP_SENSOR] = GPADC_efuseExtractTrims(topCtrlRegs->EFUSE1_ROW_37, \
				EFUSE1_ROW_37_DIG_HWA_TEMP_SENSOR_TRIM1_30C_STOP_BIT, EFUSE1_ROW_37_DIG_HWA_TEMP_SENSOR_TRIM1_30C_START_BIT);
		efuseTempTrimValues.TrimIntercept30C[GPADC_DIG_HSM_TEMP_SENSOR] = GPADC_efuseExtractTrims(topCtrlRegs->EFUSE1_ROW_38, \
				EFUSE1_ROW_38_DIG_HSM_TEMP_SENSOR_TRIM2_30C_STOP_BIT, EFUSE1_ROW_38_DIG_HSM_TEMP_SENSOR_TRIM2_30C_START_BIT);


		efuseTempTrimValues.TrimIntercept125C[GPADC_DIG_DSP_TEMP_SENSOR] = GPADC_efuseExtractTrims(topCtrlRegs->EFUSE1_ROW_34, \
				EFUSE1_ROW_34_DIG_DSP_TEMP_SENSOR_TRIM0_125C_STOP_BIT, EFUSE1_ROW_34_DIG_DSP_TEMP_SENSOR_TRIM0_125C_START_BIT);
		efuseTempTrimValues.TrimIntercept125C[GPADC_DIG_HWA_TEMP_SENSOR] = GPADC_efuseExtractTrims(topCtrlRegs->EFUSE1_ROW_34, \
				EFUSE1_ROW_34_DIG_HWA_TEMP_SENSOR_TRIM1_125C_STOP_BIT, EFUSE1_ROW_34_DIG_HWA_TEMP_SENSOR_TRIM1_125C_START_BIT);
		efuseTempTrimValues.TrimIntercept125C[GPADC_DIG_HSM_TEMP_SENSOR] = GPADC_efuseExtractTrims(topCtrlRegs->EFUSE1_ROW_35, \
				EFUSE1_ROW_35_DIG_HSM_TEMP_SENSOR_TRIM2_125C_STOP_BIT, EFUSE1_ROW_35_DIG_HSM_TEMP_SENSOR_TRIM2_125C_START_BIT);

		tempSensTrimSlopeValues.TrimIntercept30C[GPADC_DIG_DSP_TEMP_SENSOR] = efuseTempTrimValues.TrimIntercept30C[GPADC_DIG_DSP_TEMP_SENSOR];
		tempSensTrimSlopeValues.TrimIntercept30C[GPADC_DIG_HWA_TEMP_SENSOR] = efuseTempTrimValues.TrimIntercept30C[GPADC_DIG_HWA_TEMP_SENSOR];
		tempSensTrimSlopeValues.TrimIntercept30C[GPADC_DIG_HSM_TEMP_SENSOR] = efuseTempTrimValues.TrimIntercept30C[GPADC_DIG_HSM_TEMP_SENSOR];

		tempSensTrimSlopeValues.TrimIntercept125C[GPADC_DIG_DSP_TEMP_SENSOR] = efuseTempTrimValues.TrimIntercept125C[GPADC_DIG_DSP_TEMP_SENSOR];
		tempSensTrimSlopeValues.TrimIntercept125C[GPADC_DIG_HWA_TEMP_SENSOR] = efuseTempTrimValues.TrimIntercept125C[GPADC_DIG_HWA_TEMP_SENSOR];
		tempSensTrimSlopeValues.TrimIntercept125C[GPADC_DIG_HSM_TEMP_SENSOR] = efuseTempTrimValues.TrimIntercept125C[GPADC_DIG_HSM_TEMP_SENSOR];

		tempSensTrimSlopeValues.Slope[GPADC_DIG_DSP_TEMP_SENSOR] = ONE_PT_TRIM_FIXED_SLOPE;
		tempSensTrimSlopeValues.Slope[GPADC_DIG_HWA_TEMP_SENSOR] = ONE_PT_TRIM_FIXED_SLOPE;
		tempSensTrimSlopeValues.Slope[GPADC_DIG_HSM_TEMP_SENSOR] = ONE_PT_TRIM_FIXED_SLOPE;

		tempSensTrimSlopeValues.InterceptTemp = ( tempSensTrimSlopeValues.TrimTemp30C - EFUSE_TRIM_TEMPERATURE_CONST )/ EFUSE_TRIM_TEMPERATURE_DIV_CONST;
	}
	else if(2U == efuseTempTrimValues.FuseROMVer)
	{
		tempSensTrimSlopeValues.TrimTemp30C = efuseTempTrimValues.TrimTemp30C;
		tempSensTrimSlopeValues.TrimTemp125C = efuseTempTrimValues.TrimTemp125C;

		efuseTempTrimValues.TrimIntercept30C[GPADC_DIG_DSP_TEMP_SENSOR] = GPADC_efuseExtractTrims(topCtrlRegs->EFUSE1_ROW_37, \
				EFUSE1_ROW_37_DIG_DSP_TEMP_SENSOR_TRIM0_30C_STOP_BIT, EFUSE1_ROW_37_DIG_DSP_TEMP_SENSOR_TRIM0_30C_START_BIT);
		efuseTempTrimValues.TrimIntercept30C[GPADC_DIG_HWA_TEMP_SENSOR] = GPADC_efuseExtractTrims(topCtrlRegs->EFUSE1_ROW_37, \
				EFUSE1_ROW_37_DIG_HWA_TEMP_SENSOR_TRIM1_30C_STOP_BIT, EFUSE1_ROW_37_DIG_HWA_TEMP_SENSOR_TRIM1_30C_START_BIT);
		efuseTempTrimValues.TrimIntercept30C[GPADC_DIG_HSM_TEMP_SENSOR] = GPADC_efuseExtractTrims(topCtrlRegs->EFUSE1_ROW_38, \
				EFUSE1_ROW_38_DIG_HSM_TEMP_SENSOR_TRIM2_30C_STOP_BIT, EFUSE1_ROW_38_DIG_HSM_TEMP_SENSOR_TRIM2_30C_START_BIT);


		efuseTempTrimValues.TrimIntercept125C[GPADC_DIG_DSP_TEMP_SENSOR] = GPADC_efuseExtractTrims(topCtrlRegs->EFUSE1_ROW_34, \
				EFUSE1_ROW_34_DIG_DSP_TEMP_SENSOR_TRIM0_125C_STOP_BIT, EFUSE1_ROW_34_DIG_DSP_TEMP_SENSOR_TRIM0_125C_START_BIT);
		efuseTempTrimValues.TrimIntercept125C[GPADC_DIG_HWA_TEMP_SENSOR] = GPADC_efuseExtractTrims(topCtrlRegs->EFUSE1_ROW_34, \
				EFUSE1_ROW_34_DIG_HWA_TEMP_SENSOR_TRIM1_125C_STOP_BIT, EFUSE1_ROW_34_DIG_HWA_TEMP_SENSOR_TRIM1_125C_START_BIT);
		efuseTempTrimValues.TrimIntercept125C[GPADC_DIG_HSM_TEMP_SENSOR] = GPADC_efuseExtractTrims(topCtrlRegs->EFUSE1_ROW_35, \
				EFUSE1_ROW_35_DIG_HSM_TEMP_SENSOR_TRIM2_125C_STOP_BIT, EFUSE1_ROW_35_DIG_HSM_TEMP_SENSOR_TRIM2_125C_START_BIT);

		tempSensTrimSlopeValues.TrimIntercept30C[GPADC_DIG_DSP_TEMP_SENSOR] = efuseTempTrimValues.TrimIntercept30C[GPADC_DIG_DSP_TEMP_SENSOR];
		tempSensTrimSlopeValues.TrimIntercept30C[GPADC_DIG_HWA_TEMP_SENSOR] = efuseTempTrimValues.TrimIntercept30C[GPADC_DIG_HWA_TEMP_SENSOR];
		tempSensTrimSlopeValues.TrimIntercept30C[GPADC_DIG_HSM_TEMP_SENSOR] = efuseTempTrimValues.TrimIntercept30C[GPADC_DIG_HSM_TEMP_SENSOR];

		tempSensTrimSlopeValues.TrimIntercept125C[GPADC_DIG_DSP_TEMP_SENSOR] = efuseTempTrimValues.TrimIntercept125C[GPADC_DIG_DSP_TEMP_SENSOR];
		tempSensTrimSlopeValues.TrimIntercept125C[GPADC_DIG_HWA_TEMP_SENSOR] = efuseTempTrimValues.TrimIntercept125C[GPADC_DIG_HWA_TEMP_SENSOR];
		tempSensTrimSlopeValues.TrimIntercept125C[GPADC_DIG_HSM_TEMP_SENSOR] = efuseTempTrimValues.TrimIntercept125C[GPADC_DIG_HSM_TEMP_SENSOR];


		GPADC_computeTempSlope();

		tempSensTrimSlopeValues.InterceptTemp = ( tempSensTrimSlopeValues.TrimTemp30C - EFUSE_TRIM_TEMPERATURE_CONST )/ EFUSE_TRIM_TEMPERATURE_DIV_CONST;

	}
}

static uint8_t GPADC_convSkipTimeToCode(uint32_t skipSamples)
{
    uint32_t result;
    uint8_t  twoPwr = 0U, skipSampProg;

    /*! find the nearest a*2^b values to program to Hw */
    result = skipSamples;
    while (result >= (uint32_t)16U)
    {
        twoPwr++;
        /*Invalid LDRA warning - c_twoPwr can not be zero */
        /*LDRA_INSPECTED 127 D */
        result = (skipSamples + ((uint32_t)1U << (twoPwr - 1U))) / ((uint32_t)1U << twoPwr);

        /*! Max two to the power (b) is 3 bits */
        if (twoPwr == 7U)
        {
            break;
        }
    }

    /*! Max value of a is 4 bits */
    if (result >= 16U)
    {
        result = 15U;
    }

    /*! The programming skip sample values for first parameter */
    skipSampProg = (((twoPwr & 0x07U) << 4U) | ((uint8_t)result & 0xFU));

    return skipSampProg;
}


static void GPADC_convert(uint32_t cfgVal, uint32_t paramVal, GPADC_ResultType *gpAdcResult)
{

    uint8_t numSamples;

    /* Configure the GPADC by writing Config reg and Param reg*/
    GPADC_cfg(cfgVal, paramVal);

    /* Trigger the GPADC for Conversion */
    GPADC_SWTrigger();

    numSamples = (uint8_t)((paramVal & 0xFF00U) >> 8U);

    /* Get the GPADC conversion results */
    GPADC_readSamplingResult(numSamples, gpAdcResult);
}

static void GPADC_cfg(uint32_t cfgVal, uint32_t paramVal)
{
    uint32_t regWrSts = 0U;

    /* Write 32 bit Dynamic value to config reg */
    REG_STRUCT_SWRITE(GPADC_REGS_PTR->r_Reg2.b32_ConfigValueIfm, cfgVal, regWrSts);

    /* Write Param Value, Collect Samples , Skip Samples */
    REG_STRUCT_SWRITE(GPADC_REGS_PTR->r_Reg3.b32_Reg, paramVal, regWrSts);

    /* Raise a fatal error if any of above register writes failed */
    DebugP_assert(NULL == regWrSts);

    return;
}

static void GPADC_SWTrigger(void)
{
    uint32_t pmuCount;
    uint32_t countDelta = 0U;
    uint32_t regWrSts = 0U;

    /* Get the PMU count */
    pmuCount = CycleCounterP_getCount32();
    /* Trigger the ADC Conversion in IFM mode */
    REG_STRUCT_SWRITE(GPADC_REGS_PTR->r_Reg1.b1_GpadcStartBypVal, 0U, regWrSts);

    REG_STRUCT_SWRITE(GPADC_REGS_PTR->r_Reg1.b1_GpadcTrigger, 0U, regWrSts);
    REG_STRUCT_SWRITE(GPADC_REGS_PTR->r_Reg1.b1_GpadcTrigger, 1U, regWrSts);

    /* Wait for GPADC conversion to complete */
    while (countDelta < GPADC_TIMEOUT_MAX)
    {
        /* Break if the Status bit is set to 1 */
        if (1U == GPADC_REGS_PTR->r_Reg17.b1_GpadcIfmDoneStatus)
        {
            break;
        }

        /* Compute the PMU count delta */
        countDelta = CycleCounterP_getCount32() - pmuCount;
    }

    /* Raise an exception if timeout occured */
    DebugP_assert(1U == GPADC_REGS_PTR->r_Reg17.b1_GpadcIfmDoneStatus);

    /* Clear the IFM status bit */
    REG_STRUCT_SCLEAR(GPADC_REGS_PTR->r_Reg18.b1_GpadcIfmDoneClr, 1U, regWrSts);

}

static void GPADC_readSamplingResult(uint8_t numSamples, GPADC_ResultType *gpAdcResult)
{
    uint32_t regWrSts = 0U;
    uint32_t sumReg;

    /* Read the Results and populate the Structure */
    if (NULL != gpAdcResult)
    {
        /* Read the Sum of ADC samples */
        sumReg = (uint32_t)(GPADC_REGS_PTR->r_Reg14.b20_SumIfm);
        gpAdcResult->Sum = sumReg;

        /* Read the Min of ADC samples */
        gpAdcResult->Min = (uint16_t)(GPADC_REGS_PTR->r_Reg15.b10_MinGpadc);

        /* Read the Max of ADC samples */
        gpAdcResult->Max = (uint16_t)(GPADC_REGS_PTR->r_Reg15.b10_MaxGpadc);

        /* Read the Avg of ADC samples */
        if ((uint8_t)0U != numSamples)
        {
            gpAdcResult->Avg = (uint16_t)(sumReg/ (uint32_t)numSamples);
        }
        else
        {
            /* Spin here if number of samples is zero */
          DebugP_assert(NULL == numSamples);
        }
    }
    else
    {
        /* Spin here if Null Pointer */
        DebugP_assert(NULL == gpAdcResult);
    }

    /* Write Param Value to 0 to avoid any contention */
    REG_STRUCT_SWRITE(GPADC_REGS_PTR->r_Reg3.b32_Reg, 0U, regWrSts);

    /* Disable GPADC after use */
    REG_STRUCT_SWRITE(GPADC_REGS_PTR->r_Reg0.b2_DcbistMode, GPADC_MODE_DISABLE, regWrSts);

    return;
}

int32_t GPADC_readTemperature(uint8_t numAverages,uint8_t numChannels, GPADC_TempSensValueType * tempValuesPtr)
{
	uint16_t gpadcTempVal[3] = {0}, gpadcCode;
	int32_t gpadcTempValSum[3] = {0}, tempVal;
	uint8_t index, index2;
	GPADC_channelsGroupSelectType channels;
	GPADC_ConvResultType convRes;
    uint32_t regWrSts = 0U;
    static uint8_t avgSamples = 0;
    int32_t status = SystemP_SUCCESS;
    avgSamples = numAverages;

	channels.bits.b9_ChannelSelectionBitMap = 0x007;

    REG_STRUCT_SWRITE(MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_PTR->b1_TsSeInpBufEn, 1U, regWrSts);
    REG_STRUCT_SWRITE(MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_PTR->b1_TsDiffInpBufEn, 0U, regWrSts);

	GPADC_setupResultBuffer(&gpadcTempVal[0]);

	for( index = 0; index < avgSamples; index++)
	{
		convRes = GPADC_startGroupConversion(channels, numChannels);

		if(GPADC_CONV_DONE == convRes)
		{
			status = SystemP_SUCCESS;
		}
		else if(GPADC_CONV_CHANNEL_CONFIG_MISSING == convRes)
		{
			status = SystemP_FAILURE;
			break;
		}
		else
		{
			status = SystemP_FAILURE;
			break;
		}

		for( index2 = 0; index2 < MAX_GPADC_TEMP_SENSORS; index2++)
		{
			gpadcCode = gpadcTempVal[index2];
			tempVal = GPADC_calculateTemp(gpadcCode, index2);
			gpadcTempValSum[index2] = gpadcTempValSum[index2] + tempVal;
		}
	}

	tempValuesPtr->DigDspTempValue = gpadcTempValSum[GPADC_DIG_DSP_TEMP_SENSOR] / avgSamples;
	tempValuesPtr->DigHwaTempValue = gpadcTempValSum[GPADC_DIG_HWA_TEMP_SENSOR] / avgSamples;
	tempValuesPtr->DigHsmTempValue = gpadcTempValSum[GPADC_DIG_HSM_TEMP_SENSOR] / avgSamples;

	return status;
}

static uint16_t GPADC_efuseExtractTrims (volatile uint32_t reg, uint8_t msb, uint8_t lsb)
{
    uint32_t    mask;
    uint8_t     bits;
    uint16_t    value;

    /* Compute the mask: */
    bits = (msb - lsb + 1U);
    mask = (uint32_t)((uint32_t)1U << bits);
    mask = mask - 1U;

    value = (uint16_t)((reg >> lsb) & mask);
    return value;
}

static void GPADC_computeTempSlope(void)
{
    uint8_t index;

    for( index = 0; index < MAX_GPADC_TEMP_SENSORS; index++)
    {
        tempSensTrimSlopeValues.Slope[index] = (tempSensTrimSlopeValues.TrimIntercept125C[index] - tempSensTrimSlopeValues.TrimIntercept30C[index]) \
            / (tempSensTrimSlopeValues.TrimTemp125C - tempSensTrimSlopeValues.TrimTemp30C);
    }
}

static int32_t GPADC_calculateTemp(uint16_t gpadcTempCode, uint8_t index)
{
	int32_t tempVal;

	tempVal = ((((int32_t)gpadcTempCode - (int32_t)tempSensTrimSlopeValues.TrimIntercept30C[index]) \
			/ (int32_t)tempSensTrimSlopeValues.Slope[index]) \
			+ tempSensTrimSlopeValues.InterceptTemp);
	return tempVal;
}
