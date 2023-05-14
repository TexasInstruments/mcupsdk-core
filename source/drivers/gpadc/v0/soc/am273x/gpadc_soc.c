/*
 *  Copyright (C) 2021-23 Texas Instruments Incorporated.
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
 *
 */

/**
 *  \file gpadc_soc.c
 *
 *  \brief File containing GPADC soc APIs implementation for version V0.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <string.h>
#include <drivers/gpadc.h>

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/** \brief   Zero point TRIM Fixed Slope */
#define ZERO_PT_TRIM_FIXED_SLOPE                                     (-0.988f)

/** \brief   Zero point TRIM Fixed Temperature */
#define ZERO_PT_TRIM_FIXED_TRIM_TEMP                                 (32.0f)

/** \brief   Zero Value */
#define ZERO                                                         (0.0f)

/** \brief   Zero point TRIM Fixed  Digital Temperature */
#define ZERO_PT_TRIM_FIXED_DIG_TEMP_SENSOR_TRIM_30C                  (384U)

/* ========================================================================== */
/*                         Structure                  */
/* ========================================================================== */

/** \brief
 *    The Temperature sensor trim parameters structure
 */
typedef struct
{
    /** \brief Temperature trim value */
   Float32      TrimTemp30C;
   /** \brief Temperature trim value */
   Float32      TrimTemp125C;
   /** \brief Intercept trim value */
   uint16_t     TrimIntercept30C[MAX_GPADC_TEMP_SENSORS];
   /** \brief Intercept trim value */
   uint16_t     TrimIntercept125C[MAX_GPADC_TEMP_SENSORS];
   /** \brief Slope value */
   Float32      Slope[MAX_GPADC_TEMP_SENSORS];
   /** \brief Intercept Temperature value */
   Float32      InterceptTemp;
} GPADC_TempSensTrimType;

/** \brief
 *   GPADC Param Lut
 *   ConfigValue, {ParamValue, CollectSamples, SkipSamples, ChirpBreak, Reserved}, time(10MHz clock)
 */
GPADC_CfgAndParamValuesType GPADC_ConfigParamLuTab[MAX_GPADC_MEAS_SOURCES] = {
    /*! UnBuffConfigVal, BuffConfigVal, {ParamVal, CollectSamples, b4_SkipSamples, Reserved}, time */
    {    0x400U,         0x2U,          {{ 124U,        255U,           0U,            0U }},     74U }, /* #0 M_GPADC_MEAS_EXT_CH1 */
    {    0x800U,         0x4U,          {{ 124U,        255U,           0U,            0U }},     74U }, /* #1 M_GPADC_MEAS_EXT_CH2 */
    {   0x1000U,         0x8U,          {{ 124U,        255U,           0U,            0U }},     74U }, /* #2 M_GPADC_MEAS_EXT_CH3 */
    {   0x2000U,        0x10U,          {{ 124U,        255U,           0U,            0U }},     74U }, /* #3 M_GPADC_MEAS_EXT_CH4 */
    {   0x4000U,     0x10000U,          {{ 124U,        255U,           0U,            0U }},     74U }, /* #4 M_GPADC_MEAS_EXT_CH5 */
    {   0x8000U,    0x200000U,          {{ 124U,        255U,           0U,            0U }},     74U }, /* #5 M_GPADC_MEAS_EXT_CH6 */
    {0x1000000U,    0x400000U,          {{ 124U,        255U,           0U,            0U }},     74U }, /* #6 M_GPADC_MEAS_EXT_CH7 */
    {0x2000000U,    0x800000U,          {{ 124U,        255U,           0U,            0U }},     74U }, /* #7 M_GPADC_MEAS_EXT_CH8 */
    {0x8000000U,   0x4000000U,          {{ 124U,        255U,           0U,            0U }},     74U }, /* #8 M_GPADC_MEAS_EXT_CH9 */
};

/**\brief
 *  GPADC Temperature Sensors Param Lookup Table
 *  ConfigValue, ParamValue, SkipSamples, CollectSamples
 */
GPADC_TempSensMuxType GPADC_TempSensConfigParamTab[MAX_GPADC_TEMP_SENSORS] = {
    /*! ConfigValue, ParamValue, SkipSamples, CollectSamples */
    {    0x40000U,     0x7CU,        0U,           255U},  /* #0 GPADC_DIG_DSP_TEMP_SENSOR */
    {    0x80000U,     0x7CU,        0U,           255U},  /* #1 GPADC_DIG_HWA_TEMP_SENSOR */
    {   0x100000U,     0x7CU,        0U,           255U}   /* #2 GPADC_DIG_HSM_TEMP_SENSOR */
};

/* ========================================================================== */
/*                         Global Variables                        */
/* ========================================================================== */

/** \brief Temperature sensor trim Slop Values */
GPADC_TempSensTrimType tempSensTrimSlopeValues;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static void GPADC_computeTempSlope(void);
static Float32 GPADC_calculateTemp(uint16_t gpadcTempCode, uint8_t index);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void GPADC_socResetRelease(void)
{

    uint32_t regWrSts = 0U;

    /* Release Reset GPADC for digital FSM */
    REG_STRUCT_SWRITE(MSS_GPADC_RST_CTRL_PTR->b3_Assert, GPADC_FSM_DEASSERT_RESET, regWrSts);

    REG_STRUCT_SWRITE(MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_PTR->b1_ClkTmuxEsdCtrl, 1U, regWrSts);
    REG_STRUCT_SWRITE(MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_PTR->b1_AnaTestEn, 1U, regWrSts);

    REG_STRUCT_SWRITE(MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_PTR->b1_AnaogTestTmuxEsdCtrl, 1U, regWrSts);

    REG_STRUCT_SWRITE(MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_PTR->b1_AdcRefBufEn, 1U, regWrSts);

    /* Reset GPADC Ana */
    REG_STRUCT_SWRITE(MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_PTR->b1_AdcReset, GPADC_ASSERT_RESET, regWrSts);
    /* Enable GPADC Ana */
    REG_STRUCT_SWRITE(MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_PTR->b1_AdcEn, GPADC_ENABLE, regWrSts);

    /* Release GPADC Reset Ana */
    REG_STRUCT_SWRITE(MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_PTR->b1_AdcReset, GPADC_DEASSERT_RESET, regWrSts);

    REG_STRUCT_SWRITE(MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_PTR->b3_AdcRefSel_2_0, 1U, regWrSts);

    REG_STRUCT_SWRITE(MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_PTR->b1_AdcStartConv, 1U, regWrSts);

    /*! Assert if Register read back test failed */
    DebugP_assert(NULL == regWrSts);
}

void GPADC_initTempMeasurement(void)
{
    memset(&tempSensTrimSlopeValues, 0, sizeof(GPADC_TempSensTrimType));

    /* Zero-Point Trim */
    tempSensTrimSlopeValues.TrimTemp30C  = ZERO;
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

int32_t GPADC_readTemperature(uint8_t numAverages,uint8_t numChannels, GPADC_TempSensValueType * tempValuesPtr)
{
    uint16_t gpadcTempVal[3] = {0}, gpadcCode;
    Float32 gpadcTempValSum[3] = {0}, tempVal;
    uint8_t index, index2;
    GPADC_channelsGroupSelectType channels;
    GPADC_ConvResultType convRes;
    uint32_t regWrSts = 0U;
    int32_t status = SystemP_SUCCESS;

    channels.bits.b9_ChannelSelectionBitMap = 0x007;

    REG_STRUCT_SWRITE(MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_PTR->b1_TsSeInpBufEn, 1U, regWrSts);
    REG_STRUCT_SWRITE(MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_PTR->b1_TsDiffInpBufEn, 0U, regWrSts);

    GPADC_setupResultBuffer(&gpadcTempVal[0]);

    for( index = 0; index < numAverages; index++)
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

    tempValuesPtr->DigDspTempValue = gpadcTempValSum[GPADC_DIG_DSP_TEMP_SENSOR] / numAverages;
    tempValuesPtr->DigHwaTempValue = gpadcTempValSum[GPADC_DIG_HWA_TEMP_SENSOR] / numAverages;
    tempValuesPtr->DigHsmTempValue = gpadcTempValSum[GPADC_DIG_HSM_TEMP_SENSOR] / numAverages;

    return status;
}

static Float32 GPADC_calculateTemp(uint16_t gpadcTempCode, uint8_t index)
{
    Float32 tempVal;

    tempVal = ((((Float32)gpadcTempCode - (Float32)tempSensTrimSlopeValues.TrimIntercept30C[index]) \
            / tempSensTrimSlopeValues.Slope[index]) \
            + tempSensTrimSlopeValues.InterceptTemp);
    return tempVal;
}
