/*
 *  Copyright (C) 2023 Texas Instruments Incorporated.
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

#ifndef SDL_IP_VTM_H
#define SDL_IP_VTM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <sdl/vtm/v0/sdlr_vtm.h>
#if defined (SOC_AM64X)
#include <sdl/vtm/v0/soc/am64x/sdl_soc_vtm.h>
#endif
#if defined (SOC_AM243X)
#include <sdl/vtm/v0/soc/am243x/sdl_soc_vtm.h>
#endif
/**
 *  \defgroup SDL_IP_VTM_API VTM Low-Level API
 *  \ingroup SDL_VTM_API
 *
 *  This module contains the Low-Level APIs to program and use the VTM module.
 *
 *  @{
 */

/**
 * \ingroup SDL_VTM_API
 * \defgroup SDL_IP_VTM_Enum VTM IP Enumerated Data Types
 * @{
 *  Provides the APIs for VTM IP.
 */

/**
 * \brief This enumerator define forVTM VD configuration valid map
 */

typedef uint8_t SDL_VTM_configVdCtrl;
#define SDL_VTM_VD_CONFIG_CTRL_VID_OPP                (1U)
#define SDL_VTM_VD_CONFIG_CTRL_EVT_SEL			      (2U)
#define SDL_VTM_VD_CONFIG_CTRL_GLB_CFG			      (4U)


/**
 * \brief This enumerator define for VTM TS configuration valid map
 *
 *  \anchor SDL_VTM_configTsCtrl
 */

typedef uint8_t SDL_VTM_configTsCtrl;
#define SDL_VTM_VD_CONFIG_CTRL_SET_CTL                (1U)
#define SDL_VTM_VD_CONFIG_CTRL_OUTRNG_ALRT		      (2U)
#define SDL_VTM_VD_CONFIG_CTRL_SET_THR			      (4U)


/**
 * \brief This enumerator defines the possible VID Codes to set various
 *        voltage domain supply voltages
 *
 *  \anchor SDL_VTM_vid_opp
 *
 */

typedef  uint8_t SDL_VTM_vid_opp;
    /** Maximum number of OPP VID Codes */
#define SDL_VTM_VID_OPP_MAX_NUM                   ((uint8_t) 4U)
    /** VID OPP3 Code */
#define SDL_VTM_VID_OPP_3_CODE                    ((uint8_t) 3U)
    /** VID OPP2 Code */
#define SDL_VTM_VID_OPP_2_CODE                    ((uint8_t) 2U)
    /** VID OPP1 Code */
#define SDL_VTM_VID_OPP_1_CODE                    ((uint8_t) 1U)
    /** VID OPP0 Code */
#define SDL_VTM_VID_OPP_0_CODE                    ((uint8_t) 0U)


/**
 * \brief This enumerator defines the core voltage domain mapping of VTM VD
 *
 *  \anchor SDL_VTM_ts_stat_vd_map
 */
typedef  uint8_t SDL_VTM_ts_stat_vd_map;
    /** RTC Voltage Domain map */
#define SDL_VTM_TS_STAT_VD_MAP_RTC                       ((uint32) 0U)
    /** WKUP Voltage Domain map */
#define SDL_VTM_TS_STAT_VD_MAP_WKUP                      ((uint32) 1U)
    /** MCU Voltage Domain map */
#define SDL_VTM_TS_STAT_VD_MAP_MCU                       ((uint32) 2U)
    /** Core Voltage Domain map */
#define SDL_VTM_TS_STAT_VD_MAP_CORE                      ((uint32) 3U)
    /** Voltage Domain map not implemented  */
#define SDL_VTM_TSTAT_VD_MAP_NOT_IMPLEMENTED             ((uint32) 15U)


/**
 * \brief This enumerator define for VTM Voltage domain threshold interrupt control
 *
 *  \anchor SDL_VTM_intrCtrl
 */

typedef uint16_t SDL_VTM_intrCtrl;
#define SDL_VTM_VD_LT_THR0_INTR_RAW_SET                   (1u)
#define SDL_VTM_VD_GT_THR1_INTR_RAW_SET                   (2u)
#define SDL_VTM_VD_GT_THR2_INTR_RAW_SET                   (4u)
#define SDL_VTM_VD_LT_THR0_INTR_RAW_CLR                   (8u)
#define SDL_VTM_VD_GT_THR1_INTR_RAW_CLR                   (16u)
#define SDL_VTM_VD_GT_THR2_INTR_RAW_CLR                   (32u)
#define SDL_VTM_VD_LT_THR0_INTR_EN_SET                    (64u)
#define SDL_VTM_VD_GT_THR1_INTR_EN_SET                   (128u)
#define SDL_VTM_VD_GT_THR2_INTR_EN_SET                   (256u)
#define SDL_VTM_VD_LT_THR0_INTR_EN_CLR                   (512u)
#define SDL_VTM_VD_GT_THR1_INTR_EN_CLR                  (1024u)
#define SDL_VTM_VD_GT_THR2_INTR_EN_CLR                  (2048u)
#define SDL_VTM_VD_INTR_INVALID				            (SDL_VTM_VD_LT_THR0_INTR_RAW_SET |	\
														 SDL_VTM_VD_LT_THR0_INTR_RAW_CLR)


/**
 * \brief This enumerator define for VTM Voltage domain Event selection set
 *
 *  \anchor SDL_VTM_vdEvtSel_set
 */

typedef uint16_t SDL_VTM_vdEvtSel_set;

#define SDL_VTM_VD_EVT_SELECT_TEMP_SENSOR_0            (1u)
#define SDL_VTM_VD_EVT_SELECT_TEMP_SENSOR_1            (2u)
#define SDL_VTM_VD_EVT_SELECT_TEMP_SENSOR_2            (4u)
#define SDL_VTM_VD_EVT_SELECT_TEMP_SENSOR_3            (8u)
#define SDL_VTM_VD_EVT_SELECT_TEMP_SENSOR_4            (16u)
#define SDL_VTM_VD_EVT_SELECT_TEMP_SENSOR_5            (32u)
#define SDL_VTM_VD_EVT_SELECT_TEMP_SENSOR_6            (64u)
#define SDL_VTM_VD_EVT_SELECT_TEMP_SENSOR_7            (128u)

/**
 * \brief This enumerator defines for VTM Temperature sensor id
 *         control update valid maps. This controls the selective
 *         update of the fields in the temperature sensor control field.
 *
 *  \anchor SDL_VTM_tsGlobal_ctrl_valid_map
 *
 */

typedef uint32_t SDL_VTM_tsGlobal_ctrl_valid_map;
#define SDL_VTM_TSGLOBAL_CLK_SEL_VALID                    (1u)
#define SDL_VTM_TSGLOBAL_CLK_DIV_VALID                    (2u)
#define SDL_VTM_TSGLOBAL_ANY_MAXT_OUTRG_ALERT_EN_VALID    (4u)
#define SDL_VTM_TSGLOBAL_MAXT_OUTRG_ALERT_THR0_VALID      (8u)
#define SDL_VTM_TSGLOBAL_MAXT_OUTRG_ALERT_THR_VALID      (16u)
#define SDL_VTM_TSGLOBAL_SAMPLES_PER_CNT_VALID           (32u)


/**
 * \brief This enumerator define for VTM Temperature sensor global control Clock
 *        select options
 *
 *  \anchor SDL_VTM_tsGlobal_clkSel
 *
 */

typedef uint8_t SDL_VTM_tsGlobal_clkSel;
#define SDL_VTM_TSGLOBAL_CLK_CTRL_CLK_SEL_FIX_REF_CLK         (1u)
#define SDL_VTM_TSGLOBAL_CLK_CTRL_CLK_SEL_FIX_REF2_CLK        (2u)



/**
 * \brief This enumerator define for VTM Temperature sensor global control Clock
 *        divide options
 *
 *  \anchor SDL_VTM_tsGlobal_clkDiv
 *
 */

typedef uint8_t SDL_VTM_tsGlobal_clkDiv;
#define SDL_VTM_TSGLOBAL_CLK_CTRL_CLK_DIV_BY_1                   (0u)
#define SDL_VTM_TSGLOBAL_CLK_CTRL_CLK_DIV_BY_2                   (1u)
#define SDL_VTM_TSGLOBAL_CLK_CTRL_CLK_DIV_BY_3                   (2u)
#define SDL_VTM_TSGLOBAL_CLK_CTRL_CLK_DIV_BY_4                   (3u)
#define SDL_VTM_TSGLOBAL_CLK_CTRL_CLK_DIV_BY_5                   (4u)
#define SDL_VTM_TSGLOBAL_CLK_CTRL_CLK_DIV_BY_6                   (5u)
#define SDL_VTM_TSGLOBAL_CLK_CTRL_CLK_DIV_BY_7                   (6u)
#define SDL_VTM_TSGLOBAL_CLK_CTRL_CLK_DIV_BY_8                   (7u)
#define SDL_VTM_TSGLOBAL_CLK_CTRL_CLK_DIV_BY_9                   (8u)
#define SDL_VTM_TSGLOBAL_CLK_CTRL_CLK_DIV_BY_10                  (9u)
#define SDL_VTM_TSGLOBAL_CLK_CTRL_CLK_DIV_BY_11                  (10u)
#define SDL_VTM_TSGLOBAL_CLK_CTRL_CLK_DIV_BY_12                  (11u)
#define SDL_VTM_TSGLOBAL_CLK_CTRL_CLK_DIV_BY_13                  (12u)
#define SDL_VTM_TSGLOBAL_CLK_CTRL_CLK_DIV_BY_14                  (13u)
#define SDL_VTM_TSGLOBAL_CLK_CTRL_CLK_DIV_BY_15                  (14u)
#define SDL_VTM_TSGLOBAL_CLK_CTRL_CLK_DIV_BY_16                  (15u)
#define SDL_VTM_TSGLOBAL_CLK_CTRL_CLK_DIV_BY_17                  (16u)
#define SDL_VTM_TSGLOBAL_CLK_CTRL_CLK_DIV_BY_18                  (17u)
#define SDL_VTM_TSGLOBAL_CLK_CTRL_CLK_DIV_BY_19                  (18u)
#define SDL_VTM_TSGLOBAL_CLK_CTRL_CLK_DIV_BY_20                  (19u)
#define SDL_VTM_TSGLOBAL_CLK_CTRL_CLK_DIV_BY_21                  (20u)
#define SDL_VTM_TSGLOBAL_CLK_CTRL_CLK_DIV_BY_22                  (21u)
#define SDL_VTM_TSGLOBAL_CLK_CTRL_CLK_DIV_BY_23                  (22u)
#define SDL_VTM_TSGLOBAL_CLK_CTRL_CLK_DIV_BY_24                  (23u)
#define SDL_VTM_TSGLOBAL_CLK_CTRL_CLK_DIV_BY_25                  (24u)
#define SDL_VTM_TSGLOBAL_CLK_CTRL_CLK_DIV_BY_26                  (25u)
#define SDL_VTM_TSGLOBAL_CLK_CTRL_CLK_DIV_BY_27                  (26u)
#define SDL_VTM_TSGLOBAL_CLK_CTRL_CLK_DIV_BY_28                  (27u)
#define SDL_VTM_TSGLOBAL_CLK_CTRL_CLK_DIV_BY_29                  (28u)
#define SDL_VTM_TSGLOBAL_CLK_CTRL_CLK_DIV_BY_30                  (29u)
#define SDL_VTM_TSGLOBAL_CLK_CTRL_CLK_DIV_BY_31                  (30u)
#define SDL_VTM_TSGLOBAL_CLK_CTRL_CLK_DIV_BY_32                  (31u)
#define SDL_VTM_TSGLOBAL_CLK_CTRL_CLK_DIV_BY_33                  (32u)
#define SDL_VTM_TSGLOBAL_CLK_CTRL_CLK_DIV_BY_34                  (33u)
#define SDL_VTM_TSGLOBAL_CLK_CTRL_CLK_DIV_BY_35                  (34u)
#define SDL_VTM_TSGLOBAL_CLK_CTRL_CLK_DIV_BY_36                  (35u)
#define SDL_VTM_TSGLOBAL_CLK_CTRL_CLK_DIV_BY_37                  (36u)
#define SDL_VTM_TSGLOBAL_CLK_CTRL_CLK_DIV_BY_38                  (37u)
#define SDL_VTM_TSGLOBAL_CLK_CTRL_CLK_DIV_BY_39                  (38u)
#define SDL_VTM_TSGLOBAL_CLK_CTRL_CLK_DIV_BY_40                  (39u)
#define SDL_VTM_TSGLOBAL_CLK_CTRL_CLK_DIV_BY_41                  (40u)
#define SDL_VTM_TSGLOBAL_CLK_CTRL_CLK_DIV_BY_42                  (41u)
#define SDL_VTM_TSGLOBAL_CLK_CTRL_CLK_DIV_BY_43                  (42u)
#define SDL_VTM_TSGLOBAL_CLK_CTRL_CLK_DIV_BY_44                  (43u)
#define SDL_VTM_TSGLOBAL_CLK_CTRL_CLK_DIV_BY_45                  (44u)
#define SDL_VTM_TSGLOBAL_CLK_CTRL_CLK_DIV_BY_46                  (45u)
#define SDL_VTM_TSGLOBAL_CLK_CTRL_CLK_DIV_BY_47                  (46u)
#define SDL_VTM_TSGLOBAL_CLK_CTRL_CLK_DIV_BY_48                  (47u)
#define SDL_VTM_TSGLOBAL_CLK_CTRL_CLK_DIV_BY_49                  (48u)
#define SDL_VTM_TSGLOBAL_CLK_CTRL_CLK_DIV_BY_50                  (49u)
#define SDL_VTM_TSGLOBAL_CLK_CTRL_CLK_DIV_BY_51                  (50u)
#define SDL_VTM_TSGLOBAL_CLK_CTRL_CLK_DIV_BY_52                  (51u)
#define SDL_VTM_TSGLOBAL_CLK_CTRL_CLK_DIV_BY_53                  (52u)
#define SDL_VTM_TSGLOBAL_CLK_CTRL_CLK_DIV_BY_54                  (53u)
#define SDL_VTM_TSGLOBAL_CLK_CTRL_CLK_DIV_BY_55                  (54u)
#define SDL_VTM_TSGLOBAL_CLK_CTRL_CLK_DIV_BY_56                  (55u)
#define SDL_VTM_TSGLOBAL_CLK_CTRL_CLK_DIV_BY_57                  (56u)
#define SDL_VTM_TSGLOBAL_CLK_CTRL_CLK_DIV_BY_58                  (57u)
#define SDL_VTM_TSGLOBAL_CLK_CTRL_CLK_DIV_BY_59                  (58u)
#define SDL_VTM_TSGLOBAL_CLK_CTRL_CLK_DIV_BY_60                  (59u)
#define SDL_VTM_TSGLOBAL_CLK_CTRL_CLK_DIV_BY_61                  (60u)
#define SDL_VTM_TSGLOBAL_CLK_CTRL_CLK_DIV_BY_62                  (61u)
#define SDL_VTM_TSGLOBAL_CLK_CTRL_CLK_DIV_BY_63                  (62u)
#define SDL_VTM_TSGLOBAL_CLK_CTRL_CLK_DIV_BY_64                  (63u)


/**
 * \brief This enumerator define for VTM Temperature sensor global control any
 *        max temperature alert enable control
 *
 *  \anchor SDL_VTM_tsGlobal_any_maxt_outrg_alert_en
 *
 */

typedef uint8_t  SDL_VTM_tsGlobal_any_maxt_outrg_alert_en;
#define SDL_VTM_TSGLOBAL_ANY_MAXT_OUTRG_ALERT_ENABLE   (1u)
#define SDL_VTM_TSGLOBAL_ANY_MAXT_OUTRG_ALERT_DISABLE  (0u)


/**
 * \brief This enumerator define for VTM Temperature sensor global control
 *        samples per count
 *
 *  \anchor SDL_VTM_tsGlobal_samples_per_count
 */

typedef uint16_t SDL_VTM_tsGlobal_samples_per_count;


/**
 * \brief This enumerator define for VTM Temperature sensor control valid map
 *
 *  \anchor SDL_VTM_tsCtrl_valid_map
 *
 */
typedef uint8_t SDL_VTM_tsCtrl_valid_map;
#define SDL_VTM_TS_CTRL_MAXT_OUTG_ALERT_VALID                (1u)
#define SDL_VTM_TS_CTRL_RESET_CTRL_VALID                     (2u)
#define SDL_VTM_TS_CTRL_SOC_VALID                            (4u)
#define SDL_VTM_TS_CTRL_MODE_VALID                           (8u)


/**
 * \brief This enumerator define for VTM temperature sensor band gap
 *        maximum temperature out of range alert control
 *
 *  \anchor SDL_VTM_tsCtrl_max_outrg_alert
 *
 */

typedef uint8_t  SDL_VTM_tsCtrl_max_outrg_alert;
#define SDL_VTM_TS_CTRL_MAXT_OUTRG_GEN_ALERT         (1u)
#define SDL_VTM_TS_CTRL_MAXT_OUTRG_NO_ALERT          (0u)


/**
 * \brief This enumerator define for
 *        VTM temperature sensor band gap reset  control bits
 *
 *  \anchor SDL_VTM_tsCtrl_resetCtrl
 */

typedef  uint8_t SDL_VTM_tsCtrl_resetCtrl;
#define SDL_VTM_TS_CTRL_SENSOR_RESET                     (0u)
#define SDL_VTM_TS_CTRL_SENSOR_NORM_OP                   (1u)


/**
 * \brief This enumerator define for
 *        VTM temperature sensor mode control bits
 *
 *  \anchor SDL_VTM_tsCtrl_mode
 */

typedef  uint8_t    SDL_VTM_tsCtrl_mode;
#define SDL_VTM_TS_CTRL_SINGLESHOT_MODE                 (0u)
#define SDL_VTM_TS_CTRL_CONTINUOUS_MODE                 (1u)



/**
 * \brief This enumerator define for
 *        VTM temperature sensor band gap single shot mode start of conversion trigger
 *
 *  \anchor SDL_VTM_tsCtrl_singleshot_conv_stat
 */

typedef  uint8_t SDL_VTM_tsCtrl_singleshot_conv_stat;
#define SDL_VTM_TS_CTRL_SINGLESHOT_ADC_CONV_IN_PROGRESS    (1u)
#define SDL_VTM_TS_CTRL_SINGLESHOT_ADC_CONV_COMPLETE       (0u)


/**
 * \brief This enumerator define for
 *        VTM Temperature Sensor thresholds valid bit map
 *
 *  \anchor SDL_VTM_thr_valid_map
 */

typedef uint8_t  SDL_VTM_thr_valid_map;
#define SDL_VTM_GT_TH1_VALID                (1u)
#define SDL_VTM_GT_TH2_VALID                (2u)
#define SDL_VTM_LT_TH0_VALID                (4u)


/**
 * \brief This enumerator define for
 *        VTM temperature sensor STAT read valid map
 *
 *  \anchor SDL_VTM_Stat_read_ctrl
 */

typedef uint8_t SDL_VTM_Stat_read_ctrl;
#define SDL_VTM_TS_READ_VD_MAP_VAL                (1U)
#define SDL_VTM_TS_READ_ALL_THRESHOLD_ALERTS      (2U)
#define SDL_VTM_TS_READ_FIRST_TIME_EOC_BIT        (4U)
#define SDL_VTM_TS_READ_DATA_VALID_BIT            (8U)
#define SDL_VTM_TS_READ_DATA_OUT_VAL             (16U)


/**
 * \brief This enumerator define for
 *        VTM temperature sensor ADC code
 *        This is the data_out value of the temperature sensor stat register
 *
 *  \anchor SDL_VTM_adc_code
 */

typedef  int16_t SDL_VTM_adc_code;


/**
 * \brief This enumerator define for VTM Voltage domain event status
 *
 *  \anchor SDL_VTM_vdEvt_status
 *
 */

typedef uint8_t SDL_VTM_vdEvt_status;

#define SDL_VTM_VD_EVT_STAT_THR_ALERTS_MASK               (7u)
#define SDL_VTM_VD_EVT_STAT_LT_TH0_ALERT                  (4u)
#define SDL_VTM_VD_EVT_STAT_GT_TH1_ALERT                  (1u)
#define SDL_VTM_VD_EVT_STAT_GT_TH2_ALERT                  (2u)

/** @} */
/**
 * \ingroup SDL_VTM_API
 * \defgroup SDL_IP_VTM_DATASTRUCT VTM IP Data Structures
 * @{
 *  Provides the APIs for VTM IP.
 */

/** \brief VTM Global Configuration Registers
 *
 *  This structure contains VTM Global Configuration register.
 *
 */

typedef struct {
    /** valid bit map for temperature sensor global control */
    SDL_VTM_tsGlobal_ctrl_valid_map             validMap;
    /** Temperature sensor clock source selector.
         0 = fix_ref_clk as source.
         1 = fix_ref2_clk as source */
    SDL_VTM_tsGlobal_clkSel                     clkSel;
    /** Temperature sensor clock source divider selector.
        Default set by e-fuse or tie-off.
        Divider uses select reference clock as source.
        0 = 1x divide.  1 = 2x divide.
        ... 15 = 16x divide. ... 63 = 64x divide. */
    SDL_VTM_tsGlobal_clkDiv                     clkDiv;
    /** This bit in VTM_MISC_CTRL register, when enable will cause,
        the VTM’s output therm_maxtemp_outrange_alert to be driven high,
        if any of the sources for the MAXT_OUTRG_ALERT, is set high. */
    SDL_VTM_tsGlobal_any_maxt_outrg_alert_en    any_maxt_outrg_alert_en;
    /** ADC code programmed in VTM_MISC_CTRL2 for the global max temperature
    out of range safe sample value. If the alert is enabled globally
    for the sensor, and the sensor reads a value <= this value, then the alert
    is cleared after being triggered */
    SDL_VTM_adc_code                            maxt_outrg_alert_thr0;
    /** ADC code programmed in VTM_MISC_CTRL2 for the global max temperature
    out of range sample value. If the alert is enabled globally for the
    sensor, and the sensor reads a value >= this value, then
    the alert is triggered */
    SDL_VTM_adc_code                            maxt_outrg_alert_thr;
    /** Temperature sensor sample period count selector,
        programmed in VTM_SAMPLE_CTRL reg */
    SDL_VTM_tsGlobal_samples_per_count          samplesPerCnt;
} SDL_VTM_tsGlobal_cfg;

/** \brief VTM temperature sensor band gap control
 *
 *  This structure contains VTM temperature sensor control.
 *
 */

typedef struct {
    /** Valid control map for temperature sensor control parameters */
    SDL_VTM_tsCtrl_valid_map         valid_map;
    /** Enable out-of-range event. This bit enables generation of the
      alert in case the given temperature sensors generates a temp code
      above a programmed max */
    SDL_VTM_tsCtrl_max_outrg_alert   maxt_outrg_alert_en;
    /** Temp-Monitor control to reset all Temp-monitor digital outputs or
        allow operation of sensor */
    SDL_VTM_tsCtrl_resetCtrl         tsReset;
    /** Temp-Monitor control: ADC Start of Conversion.
    A transition from 0 to 1 starts a new ADC conversion cycle.
    The bit will automatically clear when the conversion has completed.
    This mode is not valid when already in continuous mode. */
    SDL_VTM_tsCtrl_singleshot_conv_stat adc_stat;
    /** Temp-Monitor control: ADC Continuous mode.
    Setting this mode enables the VTM to continuously monitor
    the sensor automatically */
    SDL_VTM_tsCtrl_mode              mode;
} SDL_VTM_Ctrlcfg;

/** \brief VTM temperature sensor threshold values
 *
 *  This structure contains VTM temperature sensor threshold values
 *
 */

typedef struct {
    /** Valid control bit map the threshold */
    SDL_VTM_thr_valid_map    thrValidMap;
    /** Over threshold value 1 */
    SDL_VTM_adc_code         gtTh1;
    /* 0: disable Th1, otherwise: enable Th1 */
    bool                     gtTh1En;
    /** Over threshold value 2 */
    SDL_VTM_adc_code         gtTh2;
    /* 0: disable Th2, otherwise: enable Th2 */
    bool                     gtTh2En;
    /** Under threshld value 0 */
    SDL_VTM_adc_code         ltTh0;
    /* 0: disable Th0, otherwise: enable Th0 */
    bool                     ltTh0En;
} SDL_VTM_tsThrVal;

/** \brief VTM temperature sensor Stat values
 *
 *  This structure contains VTM temperature sensor Stat values
 *
 */

typedef struct {
    /** Indicates the core voltage domain placement of the temperature sensor.
       Device specific field. This field indicates in which core voltage domain,
       cVD, has been physically placed the temp-monitor.
       Valid values: 0x0 to 0xE where: 0x0 = VD_RTC, not present is some SOCs,
       0x1 = VD_WKUP, 0x2 = VD_MCU, 0x3 = VD_CORE etc */
    SDL_VTM_ts_stat_vd_map     vd_map;
    /** This bit will be driven to a level 1 for a given temperature monitor
       if it has  its corresponding bit maxt_outrg_en = 1, and the temperature
       reading is reporting to be outside the max temperature supported,
       temp > programmed value */
    uint8_t                    maxt_outrg_alert;
    /** reflects the status of the lt_th0_alert comparator result. */
    uint8_t                    lt_th0_alert;
    /** reflects the status of the gt_th1_alert comparator result */
    uint8_t                    gt_th1_alert;
    /** reflects the status of the gt_th2_alert comparator result */
    uint8_t                    gt_th2_alert;
    /** ADC conversion status */
    uint8_t                    soc_fc_update;
    /** Data valid bit after the adc is done */
    uint8_t                    data_valid;
    /** Data_out signal value from sensor: Temperature data from the ADC in monitor. */
    SDL_VTM_adc_code           data_out;
} SDL_VTM_Stat_val;

/** @} */

/**
* \defgroup SDL_IP_VTM_FUNCTION  VTM IP Functions
* \ingroup SDL_VTM_API
* @{
*/

/**
 *  \brief get sensor and VD count
 *
 *  \param p_cfg1            [IN]    Pointer to the VTM configuration1 structure
 *
 */
void SDL_VTM_getSensorVDCount(const SDL_VTM_cfg1Regs       *p_cfg1);

/**
 *  \brief select best ADC code
 *
 *  \param c0            [IN]    ADC code 0
 *  \param c1            [IN]    ADC code 1
 *  \param c2            [IN]    ADC code 2

 *  \return The SDL SDL_VTM_adc_code
 */
SDL_VTM_adc_code SDL_VTM_getBestValue(SDL_VTM_adc_code c0,
                                            SDL_VTM_adc_code c1,
                                            SDL_VTM_adc_code c2);

/**
 *  \brief read Temperature sensor ADC code
 *
 *  \param p_sensor            [IN]    Pointer to the sensor code

 *  \return The SDL SDL_VTM_adc_code
 */
SDL_VTM_adc_code SDL_VTM_getAdcCode(const SDL_VTM_cfg1Regs_TMPSENS    *p_sensor);

/**
 *  \brief set the VID OPP Code for VID OPP register
 *
 *            Reset defaults are sourced from efuse for each OPP.
 *            The default reset values will not be necessarily overwritten.
 *            The write capability in the MMR is for having the option to
 *            debug and have software driven adjustments if necessary
 *
 *  \param p_cfg1            [IN]    Pointer to the VTM configuration1 structure
 *  \param instance          [IN]  	 VTM Voltage Domain instance
 *  \param vid_opp           [IN]    which VID OPP to be updated
 *  \param vid_opp_val       [IN]    VID OPP Code value
 *
 *  \return The SDL error code for the API.
 *                                 Success      : SDL_PASS
 *                                 Fail         : SDL_EFAIL
 *                                 Invalid Args : SDL_EBADARGS
 */
int32_t SDL_VTM_vdSetOppVid (const SDL_VTM_cfg1Regs  		*p_cfg1,
									SDL_VTM_InstVd 			instance,
									SDL_VTM_vid_opp         vid_opp,
									uint8_t                 vid_opp_val);

/**
 *  \brief get VTM VID OPP Code from VID OPP register
 *
 *  \param p_cfg1             [IN]    Pointer to the VTM configuration1 structure
 *  \param instance         [IN]  VTM Voltage Domain instance
 *  \param vid_opp           [IN]    which VID OPP to be updated
 *  \param p_vid_opp_val     [OUT]   Pointer to VID OPP Code value
 *
 *  \return The SDL error code for the API.
 *                                 Success      : SDL_PASS
 *                                 Fail         : SDL_EFAIL
 *                                 Invalid Args : SDL_EBADARGS
 */
int32_t SDL_VTM_vdGetOppVid (const SDL_VTM_cfg1Regs  *p_cfg1,
							SDL_VTM_InstVd 			instance,
                            SDL_VTM_vid_opp         vid_opp,
                            uint8_t                 *p_vid_opp_val);

/**
 *  \brief set Voltage domain a event select and control set register.
 *         In this API, select which of the event contributions of the
 *         temp-monitors controlled by this VTM will contribute to generate the
 *         merged event/alerts of this VD. Any combination of them could be selected
 *
 *
 *  \param p_cfg1             [IN]    Pointer to the VTM configuration1 structure
 *  \param instance         [IN]  VTM Voltage Domain instance
 *  \param vd_temp_evts      [IN]    Temperature events to be selected for VD
 *
 *  \return The SDL error code for the API.
 *                                 Success      : SDL_PASS
 *                                 Fail         : SDL_EFAIL
 *                                 Invalid Args : SDL_EBADARGS
 */
int32_t SDL_VTM_vdEvtSelSet (const SDL_VTM_cfg1Regs  *p_cfg1,
							SDL_VTM_InstVd instance,
                            SDL_VTM_vdEvtSel_set   vd_temp_evts);

/**
 *  \brief VTM Temperature Sensor Set Global configuration values
 *
 *  \param p_cfg2             [IN]    Pointer to the VTM configuration2 structure
 *  \param p_tsGlobal_cfg    [IN]    Pointer to temperature global configuration

 *  \return The SDL error code for the API.
 *                                 Success      : SDL_PASS
 *                                 Fail         : SDL_EFAIL
 *                                 Invalid Args : SDL_EBADARGS
 */
int32_t SDL_VTM_tsSetGlobalCfg (const SDL_VTM_cfg2Regs       *p_cfg2,
                               const SDL_VTM_tsGlobal_cfg   *p_tsGlobal_cfg);

/**
 *  \brief VTM Temperature Sensor Get Global configuration values
 *
 *  \param p_cfg2            [IN]    Pointer to the VTM configuration2 structure
 *  \param p_tsGlobal_cfg    [OUT]   Pointer to temperature global configuration
 *
 *  \return The SDL error code for the API.
 *                                 Success      : SDL_PASS
 *                                 Invalid Args : SDL_EBADARGS
 */
int32_t SDL_VTM_tsGetGlobalCfg (const SDL_VTM_cfg2Regs       *p_cfg2,
                               SDL_VTM_tsGlobal_cfg         *p_tsGlobal_cfg);

/**
 *  \brief VTM Temperature Sensor Control
 *
 *  \param p_cfg2            [IN]    Pointer to the VTM configuration2 structure
 *  \param instance         [IN]  VTM Temperature sensor instance
 *  \param p_tsCtrl_cfg      [IN]    Pointer to temperature sensor control configuration

 *  \return The SDL error code for the API.
 *                                 Success      : SDL_PASS
 *                                 Fail         : SDL_EFAIL
 *                                 Invalid Args : SDL_EBADARGS
 */
int32_t SDL_VTM_tsSetCtrl (const SDL_VTM_cfg2Regs  		*p_cfg2,
							SDL_VTM_InstTs 				instance,
							const SDL_VTM_Ctrlcfg   *p_tsCtrl_cfg);

/**
 *  \brief Read VTM Temperature Sensor Control
 *
 *  \param p_cfg2            [IN]    Pointer to the VTM configuration2 structure
 *  \param instance         [IN]  VTM Temperature sensor instance
 *  \param p_tsCtrl_cfg      [IN]    Pointer to temperature sensor control configuration

 *  \return The SDL error code for the API.
 *                                 Success      : SDL_PASS
 *                                 Fail         : SDL_EFAIL
 *                                 Invalid Args : SDL_EBADARGS
 */
int32_t SDL_VTM_tsGetCtrl (const SDL_VTM_cfg2Regs      *p_cfg2,
							SDL_VTM_InstTs 				instance,
                          SDL_VTM_Ctrlcfg         *p_tsCtrl_cfg);

/**
 *  \brief VTM Temperature sensor set, clear threshold values and
 *         enable, disable threshold events
 *
 *  \param p_cfg1             [IN]    Pointer to the VTM configuration1 structure
 *  \param instance         [IN]  VTM Temperature sensor instance
 *  \param p_thr_val         [IN]    Pointer to temperature threshold values
 *
 *  \return The SDL error code for the API.
 *                                 Success      : SDL_PASS
 *                                 Fail         : SDL_EFAIL
 *                                 Invalid Args : SDL_EBADARGS
 */
int32_t SDL_VTM_tsSetThresholds (const SDL_VTM_cfg1Regs		*p_cfg1,
                                SDL_VTM_InstTs 				instance,
                                const SDL_VTM_tsThrVal   	*p_thr_val);

/**
 *  \brief VTM Temperature Sensor get threshold values and threshold enable/disable status
 *
 *  \param p_cfg1             [IN]    Pointer to the VTM configuration1 structure
 *  \param instance         [IN]  VTM Temperature sensor instance
 *  \param p_thr_val         [OUT]   Pointer to temperature threshold values
 *
 *  \return The SDL error code for the API.
 *                                 Success      : SDL_PASS
 *                                 Invalid Args : SDL_EBADARGS
 */
int32_t SDL_VTM_tsGetThresholds (const SDL_VTM_cfg1Regs   *p_cfg1,
                                SDL_VTM_InstTs 				instance,
                                SDL_VTM_tsThrVal         *p_thr_val);

/**
 *  \brief VTM Temperature ADC code to Temperature conversion
 *
 *  \param adc_code                 [IN]   10 Bit ADC code
 *  \param instance         [IN]  VTM Temperature sensor instance
 *  \param p_milli_degree_temp_val  [OUT]  Pointer to Temperature in milli degree celcius
 *
 *  \return The SDL error code for the API.
 *                                 Success      : SDL_PASS
 *                                 Invalid Args : SDL_EBADARGS
 */
int32_t SDL_VTM_tsConvADCToTemp (SDL_VTM_adc_code        adc_code,
                                SDL_VTM_InstTs 				instance,
                                int32_t                 *p_milli_degree_temp_val);

/**
 *  \brief VTM Temperature to ADC code conversion
 *
 *  \param milli_degree_temp_val    [IN]   Temperature in milli degree celcius
 *  \param instance                 [IN]   VTM Temperature sensor instance
 *  \param p_adc_code               [OUT]  Pointer to 10 Bit ADC code
 *
 *  \return The SDL error code for the API.
 *                                 Success      : SDL_PASS
 *                                 Invalid Args : SDL_EBADARGS
 */
int32_t SDL_VTM_tsConvTempToAdc (int32_t milli_degree_temp_val,
                                 SDL_VTM_InstTs instance,
                                 SDL_VTM_adc_code *p_adc_code);

/**
 *  \brief VTM Temperature Sensor Maximum Temperature Out of Range Alert threshold
 *
 *  This function sets the "high temperature threshold" and "low temperature threshold"
 *  alert thresholds for the VTM hardware to use in determining when to apply the device
 *  reset (and when to release it). When the temperatures are above the high threshold,
 *  a SoC reset would be done and gets released after the temperature falls below
 *  the low temperature threshold.
 *  There should not be any ISR (Interrupt Service Routine) need to program for
 *  maximum temperature out of range programming.
 *  The caller should have actively taken necessary cooling actions, prior to
 *  temperature reaching to this maximum value, with the help of GT_THR1 and/or
 *  GT_THR2 alert ISRs.
 *
 *  \param p_cfg2            					[IN]    Pointer to the VTM configuration2 structure
 *  \param instance         					[IN]  VTM Temperature sensor instance
 *  \param high_temp_in_milli_degree_celcius    [IN]   high temperature in milli degree celcius
 *  \param low_temp_in_milli_degree_celcius     [IN]   low temperature in milli degree celcius
 *  \return The CSL error code for the API.
 *                                 Success      : CSL_PASS
 *                                 Invalid Args : CSL_EBADARGS
 */
int32_t SDL_VTM_tsSetMaxTOutRgAlertThr(const SDL_VTM_cfg2Regs    	*p_cfg2,
                                      SDL_VTM_InstTs 				instance,
                                      int32_t               high_temp_in_milli_degree_celcius,
                                      int32_t               low_temp_in_milli_degree_celcius);


/** @} */
/** @} */
#ifdef __cplusplus
}
#endif  /* extern "C" */
#endif  /* end of SDL_IP_VTM_H definition */