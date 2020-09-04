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
 *  \defgroup DRV_GPADC_MODULE APIs for GPADC
 *  \ingroup DRV_MODULE
 *
 *  This module contains APIs to program and use the GPADC module.
 *
 *  @{
 */

/**
 *  \file v0/gpadc.h
 *
 *  \brief This file contains the prototypes of the APIs present in the
 *         device abstraction layer file of GPADC.
 */

#ifndef GPADC_H_
#define GPADC_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <drivers/hw_include/cslr_soc.h>
#include <kernel/dpl/HwiP.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/**
 *  \anchor GPADC_Macros
 *  \name GPADC Macros
 *  @{
 */
/** \brief Disable GPADC */
#define GPADC_DISABLE               (0U)
/** \brief Enable GPADC */
#define GPADC_ENABLE                (1U)
/** \brief Reset GPADC */
#define GPADC_ASSERT_RESET          (1U)
/** \brief Release GPADC Reset */
#define GPADC_DEASSERT_RESET        (0U)
 /** \brief Reset GPADC for digital FSM */
#define GPADC_FSM_ASSERT_RESET      (0x7U)
 /** \brief Release Reset GPADC for digital FSM */
#define GPADC_FSM_DEASSERT_RESET    (0U)
/** @} */

/**
 *  \anchor GPADC_Modes
 *  \name GPADC Modes
 *  @{
 */
/** \brief GPADC Disable mode */
#define GPADC_MODE_DISABLE        (0U)
/** \brief GPADC IFM Operation mode */
#define GPADC_MODE_IFM            (1U)
/** \brief GPADC CTM Operation mode */
#define GPADC_MODE_CTM            (2U)
/** @} */

/**
 *  \anchor GPADC MON CTM INSTRUCTION RAM Index
 *  \name Instruction RAM Index
 *  @{
 */
/** \brief GPADC RAM Instruction Index */
#define GPADC_MON_INSTR_RAM_ST_IND    (224U)
/** \brief GPADC MAX RAM Instruction */
#define GPADC_MAX_MON_INSTR_RAM       (32U)
/** @} */

/** \brief Number of measurement parameters for GPADC in CTM mode  */
#define MAX_CTM_GPADC_PARAMS        (1U)

/** \brief
 *  GPADC TIMEOUT
 *  Max Number of Sample collect : 256
 *  Max Time for 256 samples collection : 256 * 16/10MHz = 409.6us
 *  Add 10 % margin : 450us
 *  Add 550us for skip samples
 *  Total timeout: 1ms
 *  Timout in terms of pmu count = 1ms/5ns = 200,000
 */
#define GPADC_TIMEOUT_MAX         (200000U)

/** \brief   Efuse Trime Constant Temperature */
#define EFUSE_TRIM_TEMPERATURE_CONST           523
/** \brief   Efuse Trime Constant Temperature Divider */
#define EFUSE_TRIM_TEMPERATURE_DIV_CONST       10
/** \brief   Zero point TRIM Fixed Temperature */
#define ZERO_PT_TRIM_FIXED_TRIM_TEMP                    110
/** \brief   Zero point TRIM Fixed  Digital Temperature */
#define ZERO_PT_TRIM_FIXED_DIG_TEMP_SENSOR_TRIM_30C     305U
/** \brief   Zero Value */
#define ZERO                                            0

/** \brief   Zero point TRIM Fixed Slope */
#define ZERO_PT_TRIM_FIXED_SLOPE -1.05
/** \brief   One point TRIM Fixed Slope */
#define ONE_PT_TRIM_FIXED_SLOPE -1.05
/** \brief   Efuse Version Start Bit */
#define EFUSE1_ROW_14_FUSEROM_VER_START_BIT    20U
/** \brief   Efuse Version Stop Bit */
#define EFUSE1_ROW_14_FUSEROM_VER_STOP_BIT     24U
/** \brief   Efuse Trim Temperature 30C Start Bit */
#define EFUSE1_ROW_36_TRIM_TEMPERATURE_30C_START_BIT    15U
/** \brief   Efuse Trim Temperature 30C Stop Bit */
#define EFUSE1_ROW_36_TRIM_TEMPERATURE_30C_STOP_BIT     25U
/** \brief   Efuse Trim Temperature 125C Start Bit */
#define EFUSE1_ROW_33_TRIM_TEMPERATURE_125C_START_BIT    0U
/** \brief   Efuse Trim Temperature 125C Stop Bit */
#define EFUSE1_ROW_33_TRIM_TEMPERATURE_125C_STOP_BIT     10U
/** \brief   Efuse Digital DSP Temperature 30C Sensor Start Bit */
#define EFUSE1_ROW_37_DIG_DSP_TEMP_SENSOR_TRIM0_30C_START_BIT  5U
/** \brief   Efuse Digital DSP Temperature 30C Sensor Stop Bit */
#define EFUSE1_ROW_37_DIG_DSP_TEMP_SENSOR_TRIM0_30C_STOP_BIT   14U
/** \brief   Efuse Digital HWA Temperature 30C Sensor Start Bit */
#define EFUSE1_ROW_37_DIG_HWA_TEMP_SENSOR_TRIM1_30C_START_BIT  15U
/** \brief   Efuse Digital HWA Temperature 30C Sensor Stop Bit */
#define EFUSE1_ROW_37_DIG_HWA_TEMP_SENSOR_TRIM1_30C_STOP_BIT   24U
/** \brief   Efuse Digital HSM Temperature 30C Sensor Start Bit */
#define EFUSE1_ROW_38_DIG_HSM_TEMP_SENSOR_TRIM2_30C_START_BIT  0U
/** \brief   Efuse Digital HSM Temperature 30C Sensor Stop Bit */
#define EFUSE1_ROW_38_DIG_HSM_TEMP_SENSOR_TRIM2_30C_STOP_BIT  9U
/** \brief   Efuse Digital DSP Temperature 125C Sensor Start Bit */
#define EFUSE1_ROW_34_DIG_DSP_TEMP_SENSOR_TRIM0_125C_START_BIT  5U
/** \brief   Efuse Digital DSP Temperature 125C Sensor Stop Bit */
#define EFUSE1_ROW_34_DIG_DSP_TEMP_SENSOR_TRIM0_125C_STOP_BIT   14U
/** \brief   Efuse Digital HWA Temperature 125C Sensor Start Bit */
#define EFUSE1_ROW_34_DIG_HWA_TEMP_SENSOR_TRIM1_125C_START_BIT  15U
/** \brief   Efuse Digital HWA Temperature 125C Sensor Stop Bit */
#define EFUSE1_ROW_34_DIG_HWA_TEMP_SENSOR_TRIM1_125C_STOP_BIT   24U
/** \brief   Efuse Digital HSM Temperature 125C Sensor Start Bit */
#define EFUSE1_ROW_35_DIG_HSM_TEMP_SENSOR_TRIM2_125C_START_BIT  0U
/** \brief   Efuse Digital HSM Temperature 125C Sensor Stop Bit */
#define EFUSE1_ROW_35_DIG_HSM_TEMP_SENSOR_TRIM2_125C_STOP_BIT   9U

/** \brief   GPADC Register Base Address */
#define GPADC_REGS_PTR            ((T_GPADC_REGS*)CSL_MSS_GPADC_REG_U_BASE)
/** \brief   GPADCPKTRAM Register Base Address */
#define GPADCPKTRAM_REGS_PTR      ((T_GPADCPKTRAM_REGS*)CSL_MSS_GPADC_PKT_RAM_U_BASE)
/** \brief   GPADCPKTRAM Data RAM Base Address */
#define GPADCOUT_RAM_PTR          ((T_GPADCOUT_RAM*)CSL_MSS_GPADC_DATA_RAM_U_BASE)

/** \brief   GPADC Reset Control Address */
#define MSS_GPADC_RST_CTRL_ADDR     (CSL_MSS_RCM_U_BASE + CSL_MSS_RCM_MSS_GPADC_RST_CTRL)
/** \brief   GPADC Reset Control Pointer */
#define MSS_GPADC_RST_CTRL_PTR      ((MSS_GPADC_RST_CTRL_REG*)MSS_GPADC_RST_CTRL_ADDR)
/** \brief   GPADC Clock Divider value Address */
#define MSS_GPADC_CLK_DIV_VAL_ADDR  (CSL_MSS_RCM_U_BASE + CSL_MSS_RCM_MSS_GPADC_CLK_DIV_VAL)
/** \brief   GPADC Clock Divider value pointer */
#define MSS_GPADC_CLK_DIV_VAL_PTR   ((MSS_GPADC_CLK_DIV_VAL_REG*)MSS_GPADC_CLK_DIV_VAL_ADDR)
/** \brief   GPADC Clock Gate Address */
#define MSS_GPADC_CLK_GATE_ADDR     (CSL_MSS_RCM_U_BASE + CSL_MSS_RCM_MSS_GPADC_CLK_GATE)
/** \brief   GPADC Clock Gate Pointer */
#define MSS_GPADC_CLK_GATE_PTR      ((MSS_GPADC_CLK_GATE_REG*)MSS_GPADC_CLK_GATE_ADDR)
/** \brief   RCM TW Analog TMUX Contol Address */
#define MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_ADDR (CSL_MSS_TOPRCM_U_BASE + CSL_MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV)
/** \brief   RCM TW Analog TMUX Contol Poniter */
#define MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_PTR  ((MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_REG*)MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_ADDR)
/** \brief   RCM Analog Refsys Spare register Address */
#define MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_ADDR (CSL_MSS_TOPRCM_U_BASE + CSL_MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV)
/** \brief   RCM Analog Refsys Spare register Pointer */
#define MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_PTR  ((MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_REG*)MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_ADDR)
/** \brief   RCM Analog TW Control register Address */
#define MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_ADDR (CSL_MSS_TOPRCM_U_BASE + CSL_MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV)
/** \brief   RCM Analog TW Control register pointer */
#define MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_PTR  ((U_MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_REG*)MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_ADDR)
/** \brief   GPADC Memomory Init Address */
#define MSS_CTRL_MSS_GPADC_MEM_INIT_ADDR    (CSL_MSS_CTRL_U_BASE + CSL_MSS_CTRL_MSS_GPADC_MEM_INIT)
/** \brief   GPADC Memomory Init poniter */
#define MSS_CTRL_MSS_GPADC_MEM_INIT_PTR     ((MSS_CTRL_MSS_GPADC_MEM_INIT_REG*)MSS_CTRL_MSS_GPADC_MEM_INIT_ADDR)

/** \brief
 * Register Structure member write and compare macro
 */
#define REG_STRUCT_SWRITE(w_hwRegStruct, w_regVal, w_regWrSts)   \
                do { (w_hwRegStruct) = (w_regVal); \
                     REG32_SCOMPARE((w_hwRegStruct), (w_regVal), (w_regWrSts)); \
                   } while (0)

/** \brief
 * 32 bit register compare macro
 */
#define REG32_SCOMPARE(w_hwVal, w_swVal, w_regWrSts)  \
                do { \
                     (w_regWrSts) |= (((uint32_t)(w_swVal)) ^ ((uint32_t)(w_hwVal))); \
                   } while (0)

/** \brief
 * Register Structure member CLEAR and compare macro
 */
#define REG_STRUCT_SCLEAR(w_hwRegStruct, w_regVal, w_regWrSts)   \
                do { (w_hwRegStruct) = (w_regVal); \
                     (w_regWrSts) |= ((((uint32_t)(w_hwRegStruct)) & (uint32_t)(w_regVal))); \
                   } while (0)

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/**
 * \brief  Enumeration which describes the trigger sources for GPADC CTM mode conversion
 *
 *  Note: Only GPADC_TRIGG_SRC_SW is supported in the AM273x GPADC Driver.
 *        GPADC_TRIG_SRC_MMR_Based_SW_Trigger will be default trigger if CTM mode is configured
 */
typedef enum
{
    /** \brief  0x00 - GPIO_0 */
	GPADC_TRIG_SRC_GPIO_0 = 0,
    /** \brief  0x01 - GPIO_1 */
	GPADC_TRIG_SRC_GPIO_1,
    /** \brief  0x02 - GPIO_2 */
	GPADC_TRIG_SRC_GPIO_2,
    /** \brief  0x03 - GPIO_3 */
	GPADC_TRIG_SRC_GPIO_3,
    /** \brief  0x04 - RSS_CSI2A_EOL_INT */
	GPADC_TRIG_SRC_RSS_CSI2A_EOL_INT,
    /** \brief  0x05 - RSS_CSI2A_SOF_INT0 */
	GPADC_TRIG_SRC_RSS_CSI2A_SOF_INT0,
    /** \brief  0x06 - RSS_CSI2A_SOF_INT1 */
	GPADC_TRIG_SRC_RSS_CSI2A_SOF_INT1,
    /** \brief  0x07 - RSS_CSI2A_SOF_INT */
	GPADC_TRIG_SRC_RSS_CSI2A_SOF_INT,
    /** \brief  0x08 - RSS_CSI2B_SOF_INT */
	GPADC_TRIG_SRC_RSS_CSI2B_SOF_INT,
    /** \brief  0x09 - HW_Sync_FE1 */
	GPADC_TRIG_SRC_HW_Sync_FE1,
    /** \brief  0x0A - HW_Sync_FE2 */
	GPADC_TRIG_SRC_HW_Sync_FE2,
    /** \brief  0x0B - DSS_RTIA_1 */
	GPADC_TRIG_SRC_DSS_RTIA_1,
    /** \brief  0x0C - DSS_RTIB_1 */
	GPADC_TRIG_SRC_DSS_RTIB_1,
    /** \brief  0x0D - MSS_RTIA_INT1 */
	GPADC_TRIG_SRC_MSS_RTIA_INT1,
    /** \brief  0x0E - MSS_RTIB_INT1 */
	GPADC_TRIG_SRC_MSS_RTIB_INT1,
    /** \brief  0x0F - MMR_Based_SW_Trigger */
	GPADC_TRIG_SRC_MMR_Based_SW_Trigger
}GPADC_CtmTrigSrcType;

/**
 *  \brief  Enumeration which lists the conversion modes supported for AM273x GPADC conversion
 *
 *  Note: Only GPADC_ONESHOT_CONV_MODE is supported in the AM273x GPADC Driver.
 */
typedef enum
{
    /** \brief 0x00 - IFM - Inter Frame Monitoring/One Shot Conversion Mode */
    GPADC_ONESHOT_CONV_MODE = 0,

    /** \brief 0x01 - CTM - Continuous Time Monitoring/ Continuous conversion Mode */
	GPADC_CONTINUOUS_CONV_MODE
}GPADC_ChannelConvModeType;

/**
 *  \brief  Enumeration which describes the trigger sources for GPADC CTM mode conversion.
 *
 *  Note: Only GPADC_TRIGG_SRC_SW is supported in the AM273x GPADC Driver.
 */
typedef enum
{
    /** \brief   Conversion is triggered by a software API call */
    GPADC_TRIGG_SRC_SW = 0,
    /** \brief   Conversion is triggered by a hardware event */
    GPADC_TRIGG_SRC_HW
}GPADC_TriggerSourceType;

/**
 * \brief  Enumeration which describes the external sources available for GPADC conversion
 */
typedef enum
{
    /** \brief   0x0 - Channel 1 */
	GPADC_MEAS_EXT_CH1 = 0,
    /** \brief   0x1 - Channel 2 */
	GPADC_MEAS_EXT_CH2,
    /** \brief   0x2 - Channel 3 */
	GPADC_MEAS_EXT_CH3,
    /** \brief   0x3 - Channel 4 */
	GPADC_MEAS_EXT_CH4,
    /** \brief   0x4 - Channel 5 */
	GPADC_MEAS_EXT_CH5,
    /** \brief   0x5 - Channel 6 */
	GPADC_MEAS_EXT_CH6,
    /** \brief   0x6 - Channel 7 */
	GPADC_MEAS_EXT_CH7,
    /** \brief   0x7 - Channel 8 */
	GPADC_MEAS_EXT_CH8,
    /** \brief   0x8 - Channel 9 */
	GPADC_MEAS_EXT_CH9,
    /** \brief   0x9 - MAX_CHANNELS */
	MAX_GPADC_MEAS_SOURCES
}GPADC_MeasExtSrcType;

/**
 * \brief  Enumeration which describes the error types of GPADC conversion
 */
typedef enum
{
    /** \brief   GPADC conversion error */
	GPADC_CONV_ERROR = 0,
    /** \brief   GPADC conversion done */
	GPADC_CONV_DONE,
    /**
     * \brief   GPADC requested channel through the channel bitmap or
     *          index is not configured during the initialization
     */
	GPADC_CONV_CHANNEL_CONFIG_MISSING
}GPADC_ConvResultType;

/**
 *  \brief Current status of the conversion of the requested GPADC HW unit.
 */
typedef enum
{
    /**
     * \brief The conversion of the specified group has not been started.
     *        No result is available
     */
    GPADC_IDLE,
    /**
     * \brief The conversion of the specified group has been started and is still
     *        going on. So far no result is available.
     */
    GPADC_BUSY,
    /**
     * \brief A conversion round of the specified group has been finished.
     *        A result is available for all specified channels of the group.
     */
    GPADC_COMPLETED
}GPADC_StatusType;

/**
 * \brief  Enumeration which describes the temperature sensors available for GPADC measurement
 */
typedef enum
{
    /** \brief   0x0 - DIG_DSP_TEMP_SENSOR */
	GPADC_DIG_DSP_TEMP_SENSOR = 0,
    /** \brief   0x1 - DIG_HWA_TEMP_SENSOR */
	GPADC_DIG_HWA_TEMP_SENSOR ,
    /** \brief   0x2 - DIG_HSM_TEMP_SENSOR */
	GPADC_DIG_HSM_TEMP_SENSOR ,
    /** \brief   0x3 - MAX_TEMP_SENSORS */
	MAX_GPADC_TEMP_SENSORS
}GPADC_TempSensorSrcType;

/**
 * \brief
 *  GPADC Config Type data structure for the mode, trigger source and channel
 *  configuration for all the available external sources <0-8>
 *
 *  Structure containing parameters for GPADC channels configuration.
 *  In term of GPADC hardware, this represents the conversion configuration
 *  for the unit and muxing, sampling parameters for each channel.
 *
 */
typedef struct
{
    /**
     * \brief   Channel number
     *   The hardware channel number from which input is given
     *   Valid values: 0x00 to MAX_GPADC_MEAS_SOURCES
     *   0x0 = Channel 1,
     *   0x1 = Channel 2,
     *   0x2 = Channel 3,
     *   0x3 = Channel 4,
     *   0x4 = Channel 5,
     *   0x5 = Channel 6,
     *   0x6 = Channel 7,
     *   0x7 = Channel 8,
     *   0x8 = Channel 9
     */
	GPADC_MeasExtSrcType  channelID;

    /** \brief  Unique muxing config value per channel */
    uint32_t channelConfigValue;

    /**
     * \brief Channel parameters including channel paramVal(subsystem-type),
     *        collect samples and skip *samples
     */
    uint8_t channelParamValue;

    /**
     * \brief GPADC driver considers channels configuration passed to the
     *        driver by the application only if this flag is set
     *        TRUE: Channel Configured for channelID,
     *        FALSE: Channel is not configured
     */
    Bool     isConfigured;
    /** \brief TRUE: Buffered mode FALSE: Unbuffered/ Full Scale mode */
    Bool     isBufferedMode;
    /**
     * \brief Number of samples to be skipped before collecting samples per
     *        input channel
     */
    uint32_t skipSamples;
    /** \brief Number of samples to be collected for conversion per each input channel */
    uint8_t  collectSamples;
    /**
     * \brief TRUE: Use predefined lookup table to load number of skipSamples and collectSamples
     *        configuration for the specific input channel.
     *
     *        FALSE: Use skipSamples and collectSamples configuration from the data
     *        structure passed by the application for each configured channel
     */
    Bool     useLuTable;
}GPADC_ChannelConfigType;

/**
 * \brief
 *     GPADC Config Type data structure for the mode, trigger source and channel
 *     configuration for all the available external sources <0-8>
 *
 *     Structure containing parameters for GPADC channels configuration.
 *     In term of GPADC hardware, this represents the conversion configuration
 *     for the unit and muxing, sampling parameters for each channel.
 *
 */
typedef struct
{
    /** \brief Conversion mode of the GPADC driver */
	GPADC_ChannelConvModeType        convMode;
    /** \brief Conversion trigger SW/HW trigger selection */
	GPADC_TriggerSourceType          triggSrc;
    /** \brief Channel configuration for all channels <0-8> */
	GPADC_ChannelConfigType   channelConfig[MAX_GPADC_MEAS_SOURCES];
}GPADC_ConfigType;

/** \brief
 *    Configuration Parameters for GPADC in IFM mode
 *    ParamValue        : Value to be programmed in one hot reg
 *    CollectSamples    : Number of samples to collect @625KHz
 *    SkipSamples       : Number of samples to skip @10MHz
 *                        Time = Mantissa[3:0] * 2^(Exponent[6:4]) / 10M
 */
typedef union
{
    /** \brief GPADC_ParamInfoType bits structure */
    struct
    {
        uint32_t b8_ParamValue             : 8;    /*  bits  7:  0  */
        uint32_t b8_CollectSamples         : 8;    /*  bits 15:  8  */
        /* Skip samples = Mantissa[3:0] * 2^(Exponent[6:4]) */
        uint32_t b7_SkipSamples            : 7;    /*  bits 22: 16 - */
        uint32_t b9_Reserved               : 9;    /*  bits 31: 23 - */
    } bits;
    /** \ bits 31: 0 */
    uint32_t b32_Val;
}GPADC_ParamInfoType;

/**
 * \brief  Available <0-8> external sources/channels could be selected by using bitmap
 *      of 9 bits in LSB. Ex: 0x01F is the bitmap for conversion of <0-4> external sources
 *
 *  NOTE: All channels selected in the group bitmap must have channel configuration
 *  available during GPADC initialization for successful conversion without errors.
 */
typedef union
{
    /** \brief Channel selection bitmap bits */
    struct
    {
        /**
         * \brief
         *   Channel selection bitmap for triggering group GPADC conversion and
         *   getting results for the specified channels of the group
         */
        uint16_t b9_ChannelSelectionBitMap             : 9;    /*  bits  8:  0  */
        /** \brief Reserved */
        uint16_t b7_Reserved                           : 7;    /*  bits  15: 9  */
    } bits;
    /** \brief  bits 16: 0 */
    uint16_t b16_Val;
}GPADC_channelsGroupSelectType;

/**
 * \brief
 *  GPADC Driver Channel configuration
 *
 *  The structure holds the channel confiiguration values
 */
typedef struct
{
    /** \brief true if Channel is Buffer mode  */
    Bool                  isChannelBufferedMode;
    /** \brief Channel Config value  */
    uint32_t              channelConfigValue;
    /** \brief Channel Param type */
    GPADC_ParamInfoType   channelParamValue;
    /** \brief true if channel is configured */
    Bool                  isChannelConfigured;
}GPADC_DriverChannelConfigType;

/**
 * \brief
 *  GPADC Driver Object configuration
 *
 *  The structure holds the channel confiiguration structure,operation mode structure 
 */
typedef struct
{
    /** \brief Operation mode of the group */
	GPADC_ChannelConvModeType        convMode;
    /** \brief Trigger Source Type */
	GPADC_TriggerSourceType          triggSrc;
     /** \brief Driver Status */
	GPADC_StatusType                 driverStatus;
	/** \brief  Channel configurations */
	GPADC_DriverChannelConfigType   driverChannelConfig[MAX_GPADC_MEAS_SOURCES];
    /** \brief  Pointer to store conversion results */
	uint16_t                        *ResultBufferPtr;
}GPADC_DriverObjectType;

/**
 * \brief
 *   The Temperature sensor trim parameters structure
 */
typedef struct
{
   /** \brief FuseROM Version */
   uint16_t      FuseROMVer;
   /** \brief Efuse Tim Temperature30C Value */
   uint16_t      TrimTemp30C;
   /** \brief Efuse Tim Temperature125C Value */
   uint16_t      TrimTemp125C;
   /** \brief Efuse Tim Intercept30C Value */
   uint16_t      TrimIntercept30C[MAX_GPADC_TEMP_SENSORS];
   /** \brief Efuse Tim Intercept125C Value */
   uint16_t      TrimIntercept125C[MAX_GPADC_TEMP_SENSORS];
} GPADC_EfuseTempTrimType;

/** \brief
 *    The Temperature sensor trim parameters structure
 */
typedef struct
{
    /** \brief Temperature trim value */
   uint16_t     TrimTemp30C;
   /** \brief Temperature trim value */
   uint16_t     TrimTemp125C;
   /** \brief Intercept trim value */
   uint16_t     TrimIntercept30C[MAX_GPADC_TEMP_SENSORS];
   /** \brief Intercept trim value */
   uint16_t     TrimIntercept125C[MAX_GPADC_TEMP_SENSORS];
   /** \brief Slope value */
   float        Slope[MAX_GPADC_TEMP_SENSORS];
   /** \brief Intercept Temperature value */
   float        InterceptTemp;
} GPADC_TempSensTrimType;

/** \brief
 *   Temperature sensors mux values
 */
typedef struct
{
    /** \brief Unique muxing config value per sensor */
    uint32_t channelConfigValue;
    /**
     * \brief Channel parameters including channel paramVal(subsystem-type), collect samples
     *        and skip samples
     */
    uint8_t channelParamValue;
    /**
     * \brief Number of samples to be skipped before collecting samples per
     *        input channel
     */
    uint32_t skipSamples;
    /** \brief Number of samples to be collected for conversion per each input channel */

    uint8_t  collectSamples;
} GPADC_TempSensMuxType;

/** \brief
 *  The Temperature sensor values structure
 */
typedef struct
{
   /** \brief Digital Dsp Temperature value */
   int16_t     DigDspTempValue;
   /** \brief Digital Hwa Temperature value */
   int16_t     DigHwaTempValue;
   /** \brief Digital Hsm Temperature value */
   int16_t     DigHsmTempValue;
} GPADC_TempSensValueType;

/**
 * \brief
 *  GPADC Reset Control register
 */
typedef struct
{
    uint32_t b3_Assert                    :   3;           /*!< bits   2: 0 */
    uint32_t b29_Nu1                      :  29;         /*!< bits  31: 3 */
} MSS_GPADC_RST_CTRL_REG;

/**
 * \brief
 *  GPADC Clock Divider Value register
 */
typedef struct
{
    /** \brief MSS_GPADC_CLK_DIV_VAL_REG bits structure */
    uint32_t b24_Clkdivr                    :  24;           /*!< bits   23: 0 */
    uint32_t b8_Nu1                         :  8;         /*!< bits  31: 24 */
} MSS_GPADC_CLK_DIV_VAL_REG;

/**
 * \brief
 *  GPADC Clock Gate register
 */
typedef struct
{
    uint32_t b3_Gated                     :   3;           /*!< bits   2: 0 */
    uint32_t b29_Nu1                      :  29;         /*!< bits  31: 3 */
} MSS_GPADC_CLK_GATE_REG;

/**
 * \brief
 *  Analog Mux Control Registers
 */
typedef struct
{
    uint32_t b30_Reserved1                     : 30;           /*!< bits    0: 29 */
    uint32_t b1_ClkTmuxEsdCtrl                 :  1;           /*!< bits   30:  30 */
    uint32_t b1_AnaTestEn                      :  1;           /*!< bits   31:  31 */
} MSS_TOPRCM_ANA_REG_TW_ANA_TMUX_CTRL_LOWV_REG;

/**
 * \brief
 *  Analog Refsys spare Registers
 */
typedef struct
{
    uint32_t b30_Reserved1                     : 31;           /*!< bits    0: 30 */
    uint32_t b1_AnaogTestTmuxEsdCtrl           :  1;           /*!< bits   31:  31 */
} MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_REG;

/**
 * \brief
 *  RCM Analog register TW Control Registers
 */
typedef struct
{
    uint32_t b1_AdcEn                          :  1;           /**< bits   0:  0 */
    uint32_t b1_AdcStartConv                   :  1;           /**< bits   1:  1 */
    uint32_t b1_AdcReset                       :  1;           /**< bits   2:  2 */
    uint32_t b1_AdcInpBufEn                    :  1;           /**< bits   3:  3 */
    uint32_t b1_AdcRefBufEn                    :  1;           /**< bits   4:  4 */
    uint32_t b3_AdcRefSel_2_0                  :  3;           /**< bits   7:  5 */
    uint32_t b1_TsDiffInpBufEn                 :  1;           /**< bits   8:  8 */
    uint32_t b1_TsSeInpBufEn                   :  1;           /**< bits   9:  9 */
    uint32_t b1_IforceExtCtrl                  :  1;           /**< bits  10: 10 */
    uint32_t b1_VrefExtCtrl                    :  1;           /**< bits  11: 11 */
    uint32_t b1_VinExtCtrl                     :  1;           /**< bits  12: 12 */
    uint32_t b1_AnaTmuxBufBypass               :  1;           /**< bits  13: 13 */
    uint32_t b1_AnaTmuxBufEn                   :  1;           /**< bits  14: 14 */
    uint32_t b5_RtrimTw_4_0                    :  5;           /**< bits  19: 15 */
    uint32_t b12_Reserved1                     : 12;           /**< bits  31: 20 */
} U_MSS_TOPRCM_ANA_REG_TW_CTRL_REG_LOWV_REG;


/**
 * \brief
 *  Register Offset 0x000
 */
typedef struct
{
    uint32_t b2_DcbistMode                :  2;           /*!< bits   1:  0 */
    uint32_t b6_Nu1                       :  6;           /*!< bits   7:  2 */
    uint32_t b1_GpadcFsmClkEnable         :  1;           /*!< bits   8:  8 */
    uint32_t b3_Gpadc2adcbufPathEn        :  3;           /*!< bits  11:  9 */
    uint32_t b4_Nu2                       :  4;           /*!< bits  15: 12 */
    uint32_t b1_GpadcDebugModeEnable      :  1;           /*!< bits  16: 16 */
    uint32_t b15_Nu3                      : 15;           /*!< bits  31: 17 */
} GPADCREG_REG0;

/**
 * \brief
 *  Register1 Offset 0x004
 */
typedef struct
{
    uint32_t b1_GpadcTrigger              :  1;           /**< bits   0:  0 */
    uint32_t b7_Nu1                       :  7;           /**< bits   7:  1 */
    uint32_t b1_GpadcInit                 :  1;           /**< bits   8:  8 */
    uint32_t b7_Nu2                       :  7;           /**< bits  15:  9 */
    uint32_t b1_GpadcFsmBypass            :  1;           /**< bits  16: 16 */
    uint32_t b7_Nu3                       :  7;           /**< bits  23: 17 */
    uint32_t b1_GpadcStartBypVal          :  1;           /**< bits  24: 24 */
    uint32_t b7_Nu4                       :  7;           /**< bits  31: 25 */
} GPADCREG_REG1;

/**
 * \brief
 *  Register2 Offset 0x008
 */
typedef struct
{
    uint32_t b32_ConfigValueIfm           : 32;           /**< bits  31:  0 */
} GPADCREG_REG2;

/**
 * \brief
 *  Register3 Offset 0x00C
 */
typedef union
{
    struct
    {
        uint32_t b8_ParamValIfm               :  8;           /**< bits   7:  0 */
        uint32_t b8_CollectSamplesIfm         :  8;           /**< bits  15:  8 */
        uint32_t b7_SkipSamplesIfm            :  7;           /**< bits  22: 16 */
        uint32_t b9_Nu                        :  9;           /**< bits  31: 23 */
    } bits;  /*!<Struct */
    /** \brief  bits  31: 0 */
    uint32_t b32_Reg;  /*!<Union */
} U_GPADCREG_REG3;

/**
 * \brief
 *  Register4 Offset 0x010
 */
typedef union
{
    struct
    {
        uint32_t b8_PktRamBaseAddrCp0         :  8;           /*!< bits   7:  0 */
        uint32_t b8_PktRamBaseAddrCp1         :  8;           /*!< bits  15:  8 */
        uint32_t b8_PktRamBaseAddrCp2         :  8;           /*!< bits  23: 16 */
        uint32_t b8_PktRamBaseAddrCp3         :  8;           /*!< bits  31: 24 */
    } bits;  /*!<Struct */
    uint32_t b32_Reg;  /*!<Union */
} U_GPADCREG_REG4;

/**
 * \brief
 *  Register5 Offset 0x014
 */
typedef struct
{
    uint32_t b8_PktRamBaseAddrCp4         :  8;           /**< bits   7:  0 */
    uint32_t b8_PktRamBaseAddrCp5         :  8;           /**< bits  15:  8 */
    uint32_t b8_PktRamBaseAddrCp6         :  8;           /**< bits  23: 16 */
    uint32_t b8_PktRamBaseAddrCp7         :  8;           /**< bits  31: 24 */
} GPADCREG_REG5;

/**
 * \brief
 *  Register6 Offset 0x018
 */
typedef struct
{
    uint32_t b8_PktRamBaseAddrCp8         :  8;           /**< bits   7:  0 */
    uint32_t b8_PktRamBaseAddrCp9         :  8;           /**< bits  15:  8 */
    uint32_t b8_PktRamBaseAddrCp10        :  8;           /**< bits  23: 16 */
    uint32_t b8_PktRamBaseAddrCp11        :  8;           /**< bits  31: 24 */
} GPADCREG_REG6;

/**
 * \brief
 *  Register7 Offset 0x01C
 */
typedef struct
{
    uint32_t b8_PktRamBaseAddrCp12        :  8;           /**< bits   7:  0 */
    uint32_t b8_PktRamBaseAddrCp13        :  8;           /**< bits  15:  8 */
    uint32_t b8_PktRamBaseAddrCp14        :  8;           /**< bits  23: 16 */
    uint32_t b8_PktRamBaseAddrCp15        :  8;           /**< bits  31: 24 */
} GPADCREG_REG7;

/**
 * \brief
 *  Register8 Offset 0x020
 */
typedef struct
{
    uint32_t b8_GpadcClkDiv               :  8;           /**< bits   7:  0 */
    uint32_t b1_GpadcClkEnable            :  1;           /**< bits   8:  8 */
    uint32_t b23_Nu                       : 23;           /**< bits  31:  9 */
} GPADCREG_REG8;

/**
 * \brief
 *  Register9 Offset 0x024
 */
typedef struct
{
    uint32_t b32_ParamNotUsedTxEna1Off    : 32;           /**< bits  31:  0 */
} GPADCREG_REG9;

/**
 * \brief
 *  Register10 Offset 0x028
 */
typedef struct
{
    uint32_t b32_ParamNotUsedTxEna2Off    : 32;           /**< bits  31:  0 */
} GPADCREG_REG10;

/**
 * \brief
 *  Register11 Offset 0x02C
 */
typedef struct
{
    uint32_t b32_ParamNotUsedTxEna3Off    : 32;           /**< bits  31:  0 */
} GPADCREG_REG11;

/**
 * \brief
 *  Register12 Offset 0x030
 */
typedef struct
{
    uint32_t b1_DramEccEnable             :  1;           /**< bits   0:  0 */
    uint32_t b7_Nu1                       :  7;           /**< bits   7:  1 */
    uint32_t b1_DramEccErrClr             :  1;           /**< bits   8:  8 */
    uint32_t b7_Nu2                       :  7;           /**< bits  15:  9 */
    uint32_t b8_DramEccErrAddr            :  8;           /**< bits  23: 16 */
    uint32_t b8_DramRepairedBit           :  8;           /**< bits  31: 24 */
} GPADCREG_REG12;

/**
 * \brief
 *  Register13 Offset 0x034
 */
typedef struct
{
    uint32_t b32_SpareWr2                 : 32;           /**< bits  31:  0 */
} GPADCREG_REG13;

/**
 * \brief
 *  Register14 Offset 0x038
 */
typedef struct
{
    uint32_t b20_SumIfm                   : 20;           /**< bits  19:  0 */
    uint32_t b12_Nu                       : 12;           /**< bits  31: 20 */
} GPADCREG_REG14;

/**
 * \brief
 *  Register15 Offset 0x03C
 */
typedef struct
{
    uint32_t b10_MinGpadc                 : 10;           /**< bits   9:  0 */
    uint32_t b6_Nu1                       :  6;           /**< bits  15: 10 */
    uint32_t b10_MaxGpadc                 : 10;           /**< bits  25: 16 */
    uint32_t b6_Nu2                       :  6;           /**< bits  31: 26 */
} GPADCREG_REG15;

/**
 * \brief
 *  Register16 Offset 0x040
 */
typedef struct
{
    uint32_t b1_GpadcMemInitDoneStat      :  1;           /**< bits   0:  0 */
    uint32_t b31_Nu                       : 31;           /**< bits  31:  1 */
} GPADCREG_REG16;

/**
 * \brief
 *  Register17 Offset 0x044
 */
typedef struct
{
    uint32_t b1_GpadcIfmDoneStatus        :  1;           /**< bits   0:  0 */
    uint32_t b31_Nu                       : 31;           /**< bits  31:  1 */
} GPADCREG_REG17;

/**
 * \brief
 *  Register19 Offset 0x048
 */
typedef struct
{
    uint32_t b1_GpadcIfmDoneClr           :  1;           /**< bits   0:  0 */
    uint32_t b31_Nu                       : 31;           /**< bits  31:  1 */
} GPADCREG_REG18;

/**
 * \brief
 *  Register20 Offset 0x04C
 */
typedef struct
{
    uint32_t b16_GpadcSamplesFrame        : 16;           /**< bits  15:  0 */
    uint32_t b16_Nu                       : 16;           /**< bits  31: 16 */
} GPADCREG_REG19;

/**
 * \brief
 *  Register21 Offset 0x050
 */
typedef struct
{
    uint32_t b32_SpareRd1                 : 32;           /**< bits  31:  0 */
} GPADCREG_REG20;

/**
 * \brief
 *  Register22 Offset 0x054
 */
typedef struct
{
    uint32_t b32_SpareRd2                 : 32;           /**< bits  31:  0 */
} GPADCREG_REG21;

/**
 * \brief
 *  Register22 Offset 0x058
 */
typedef struct
{
    uint32_t b32_SpareWr1                 : 32;           /**< bits  31:  0 */
} GPADCREG_REG22;


/**
 *  \brief MSS_GPADC_REG_REGS
 */
typedef volatile struct
{
    GPADCREG_REG0                       r_Reg0              ;        /**< Offset = 0x000 */
    GPADCREG_REG1                       r_Reg1              ;        /**< Offset = 0x004 */
    GPADCREG_REG2                       r_Reg2              ;        /**< Offset = 0x008 */
    U_GPADCREG_REG3                     r_Reg3              ;        /**< Offset = 0x00C */
    U_GPADCREG_REG4                     r_PacketRamAdd[4]   ;        /**< Offset = 0x010 - 0x1C */
    GPADCREG_REG8                       r_Reg8              ;        /**< Offset = 0x020 */
    GPADCREG_REG9                       r_Reg9              ;        /**< Offset = 0x024 */
    GPADCREG_REG10                      r_Reg10             ;        /**< Offset = 0x028 */
    GPADCREG_REG11                      r_Reg11             ;        /**< Offset = 0x02C */
    GPADCREG_REG12                      r_Reg12             ;        /**< Offset = 0x030 */
    GPADCREG_REG13                      r_Reg13             ;        /**< Offset = 0x034 */
    GPADCREG_REG14                      r_Reg14             ;        /**< Offset = 0x038 */
    GPADCREG_REG15                      r_Reg15             ;        /**< Offset = 0x03C */
    GPADCREG_REG16                      r_Reg16             ;        /**< Offset = 0x040 */
    GPADCREG_REG17                      r_Reg17             ;        /**< Offset = 0x044 */
    GPADCREG_REG18                      r_Reg18             ;        /**< Offset = 0x048 */
    GPADCREG_REG19                      r_Reg19             ;        /**< Offset = 0x04C */
    GPADCREG_REG20                      r_Reg20             ;        /**< Offset = 0x050 */
    GPADCREG_REG21                      r_Reg21             ;        /**< Offset = 0x054 */
    GPADCREG_REG22                      r_Reg22             ;        /**< Offset = 0x058 */
} T_GPADC_REGS;

/**
 *  \brief GPADC Memory initialize registers
 */
typedef struct
{
    uint32_t b1_mem0_init                 :   1;         /**< bits   0: 0 */
    uint32_t b31_Reserved                 :  31;         /**< bits  31: 1 */
} MSS_CTRL_MSS_GPADC_MEM_INIT_REG;

/** \brief  Configuration Parameters for GPADC LUT in IFM mode
 *          32 bits:  ConfigValue
 *          32 bits:  ParamInfo
 */
typedef struct
{
    /** \brief Unbuff configuration value */
    uint32_t             UnbuffConfigValue;
    /** \brief Buffer configuration value */
    uint32_t             BuffConfigValue;
    /** \brief ParamInfo */
    GPADC_ParamInfoType  ParamInfo;
    /** \brief Time in unit of 100ns = CollectSamples*16 + SkipSamples */
    uint16_t             TotalTime;
}GPADC_CfgAndParamValuesType;

/** \brief
 *  GPADC returning min, avg, max and sum
 */
typedef struct
{
    /** \brief Average of ADC samples */
    uint16_t Avg;
    /** \brief Minimum of ADC samples */
    uint16_t Min;
    /** \brief Maximum of ADC samples */
    uint16_t Max;
    /** \brief Sum of ADC samples */
    uint32_t Sum;
}GPADC_ResultType;


/* ========================================================================== */
/*                  Internal/Private Structure Declarations                   */
/* ========================================================================== */


/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */


/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 *  \brief      This function initializes the GPADC module.
 */
void GPADC_init(void);

/**
 *  \brief      This function de-initializes the GPADC module.
 */
void GPADC_deinit(void);

/**
 *  \brief      Initializes the GPADC Driver with the channels configuration.
 *
 *  \param[in]  CfgPtr      Pointer to the GPADC configuration
 *
 */
int32_t GPADC_open(GPADC_ConfigType *CfgPtr);

/**
 *  \brief      This function closes the GPADC module.
 */
int32_t GPADC_close(void);

/**
 *  \brief      Starts and triggers the multi channel ADC conversion.
 *
 *              Pass the channel selection bitmap for group conversion to the GPADC Driver and
 *              check the result buffer passed to the driver using GPADC_SetupResultBuffer()
 *              for conversion results of all configured channels.
 *
 *              Conversion happens based on the parameters provided through GPADC_Init()
 *              for GPADC HW unit and each channel config like sampling parameters, buffered/ unbuffered.
 *
 *              GPADC_Init() should be done before starting conversion
 *              Channel bitmap selected shouldn't be zero
 *              Result buffer should be setup before starting conversion using GPADC_SetupResultBuffer().
 *
 *  \param[in]  channels
 *  	        Channel selection bitmap to start conversion together
 *  \param[in]  numChannels     
 *              number of channels
 *
 *  \return     GPADC_CONV_ERROR: GPADC Conversion Error
 *              GPADC_CONV_DONE: GPADC Conversion Successfully Done
 *              GPADC_CONV_CHANNEL_CONFIG_MISSING: One or more of the bitmap selected
 *              channel group is missing configuration during initialization
 */
 GPADC_ConvResultType GPADC_startGroupConversion(GPADC_channelsGroupSelectType channels, uint8_t numChannels);

/**
 *  \brief      Starts and triggers the single channel conversion.
 *              Pass the channelID and result address to the driver and
 *              the result will be stored in the address passed.
 *
 *  \param[in]  channelID
 *		        Channel index from <0-8> available external sources
 *
 *  \param[out] gpadcValue
 *              Pointer to the address where the result shall be stored.
 *
 *  \return     GPADC_CONV_ERROR: GPADC Conversion Error
 *              GPADC_CONV_DONE: GPADC Conversion Successfully Done
 *              GPADC_CONV_CHANNEL_CONFIG_MISSING: The specified channel
 *              channel is not configured using GPADC_Init()
 */
GPADC_ConvResultType GPADC_startSingleChannelConversion(GPADC_MeasExtSrcType channelID, uint16_t *gpadcValue);

/**
 *  \brief      Stops the GPADC conversion.
 *
 *              This API should not be called before initialization.\n
 *              This API should not be called before starting group/single channel start conversion\n
 *              To be used when the mode of conversion is chosen as GPADC_CONTINUOUS_CONV_MODE.
 *
 *  return      If stop conversion is successful returns SystemP_SUCCESS, else error on failure
 */
int32_t GPADC_stopConversion(void);

/**
 *  \brief      Initializes GPADC driver with the group specific result
 *              buffer start address where the conversion results will be stored.
 *
 *              The application has to ensure that the application buffer,
 *              where ResBufferPtr points to, can hold all the conversion results
 *              of the specified group. The initialization with GPADC_SetupResultBuffer
 *              is required after GPADC_Init(), before a group conversion can be started.
 *
 *  \param[in]  ResBufferPtr
 *
 *
 *  \return     If Result Buffer Setup is  successful returns SystemP_SUCCESS, else error on failure
 */
int32_t GPADC_setupResultBuffer(uint16_t * ResBufferPtr);

/**
 *  \brief      The function is used to read the result buffer from the GPADC Driver.
 *
 *              Reads the group conversion result of the last completed conversion round
 *              of the requested group and stores the channel values starting at the
 *              ResBufferPtr address.
 *
 *  \param[out]  ResBufferPtr
 *
 *  \return     On Read Result Buffer is successful returns SystemP_SUCCESS , else error on failure
 */
int32_t GPADC_readResultBuffer(uint16_t *ResBufferPtr);

/**
 *  \brief      Gets the status of GPADC Driver HW unit.
 *
 *  \return     GPADC_IDLE: The conversion of the specified group has not been started.
 *                          No result is available.
 *              GPADC_BUSY: The conversion of the specified group has been started and is still going on.
 *                          So far no result is available.
 *              GPADC_COMPLETED: A conversion round of the specified group has been finished.
 *              A result is available for all specified channels of the group.
 *
 */
GPADC_StatusType GPADC_getStatus(void);

/**
 *  \brief      Read the temperature sensor value.
 *
 *  \param[in]  numAverages     number of average samples
 *  \param[in]  numChannels     number of channels
 *  \param[out] tempValuesPtr   Pointer to store the temperature values
 *
 *  \return     If temperature read is successful returns SystemP_SUCCESS, else error on failure
 */
int32_t GPADC_readTemperature(uint8_t numAverages, uint8_t numChannels, GPADC_TempSensValueType * tempValuesPtr);

/**
 *  \brief      Initialize GPADC efuse temperature parameters.
 */
void GPADC_initTempMeasurement(void);

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
}
#endif

#endif  /* #ifndef GPADC_H_ */

/** @} */
