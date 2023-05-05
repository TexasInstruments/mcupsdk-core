/*
 * VTM Example Application
 *
 * Voltage and Thermal Monitor (VTM) Example Application
 *
 *  Copyright (c) 2023 Texas Instruments Incorporated
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
/**
 *  \file vtm_example.c
 *
 *  \brief This file contains functions that provide input event triggers
 *         for the Voltage and Thermal Monitor (VTM) application.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <dpl_interface.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/TimerP.h>
#include <drivers/sciclient.h>
#include <sdl/dpl/sdl_dpl.h>
#include <sdl/sdl_vtm.h>
#include <sdl/include/sdl_types.h>
#include <sdl/sdl_esm.h>
#include <sdl/vtm/v0/sdl_ip_vtm.h>
/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define LT_THR0_DEFAULT          (95000)
#define GT_THR1_DEFAULT          (105000)
#define GT_THR2_DEFAULT          (115000)
#define USE_CASE_STATUS_NOT_RUN           (0u)
#define USE_CASE_STATUS_COMPLETED_SUCCESS (1u)
#define USE_CASE_STATUS_COMPLETED_FAILURE (2u)
/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static int32_t vtmTriggerTh(int32_t lt_thr0_offset,
                            int32_t gt_thr1_offset,
                            int32_t gt_thr2_offset);
static int32_t SDL_VTM_config(void);
static void setAllVTMTempThr(SDL_VTM_adc_code lt_thr0_adc_code,
                             SDL_VTM_adc_code gt_thr1_adc_code,
                             SDL_VTM_adc_code gt_thr2_adc_code);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* Completion of Use Case from Input trigger perspective updates these flags */
extern volatile uint32_t    vtmEventInputTrig[2];
/* Current Use case being run */
extern volatile uint8_t     currTestCase;
uint32_t                    pStatus;
SDL_ESM_Inst                currEsmInstance;


/* ========================================================================== */
/*                            External Variables                              */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/*
* This function modifies all the VTM thresholds in a single function to ensure
* appropriate number of ESM events are triggered by the change in thresholds.
*/
static void setAllVTMTempThr(SDL_VTM_adc_code lt_thr0_adc_code,
                             SDL_VTM_adc_code gt_thr1_adc_code,
                             SDL_VTM_adc_code gt_thr2_adc_code)
{

    SDL_VTM_tsThrVal        thr_val;
    SDL_VTM_intrCtrl        ctrl;
    SDL_VTM_configTs cfgTs;
    thr_val.thrValidMap = SDL_VTM_GT_TH1_VALID | \
                          SDL_VTM_GT_TH2_VALID | \
                          SDL_VTM_LT_TH0_VALID;
    thr_val.gtTh2En     = TRUE;
    thr_val.gtTh2       = gt_thr2_adc_code;
    thr_val.gtTh1En     = TRUE;
    thr_val.gtTh1       = gt_thr1_adc_code;
    thr_val.ltTh0En     = TRUE;
    thr_val.ltTh0       = lt_thr0_adc_code;
    cfgTs.thr_val       = thr_val;
    cfgTs.tsCtrl_cfg.valid_map = 0;
    cfgTs.tsCtrl_cfg.maxt_outrg_alert_en = 0;
    cfgTs.tsCtrl_cfg.tsReset = 0;
    cfgTs.tsCtrl_cfg.adc_stat = 0;
    cfgTs.tsCtrl_cfg.mode = 0;
    cfgTs.configTsCtrl = SDL_VTM_VD_CONFIG_CTRL_SET_THR;

    ctrl    = SDL_VTM_VD_GT_THR2_INTR_EN_SET | \
              SDL_VTM_VD_GT_THR1_INTR_EN_SET | \
              SDL_VTM_VD_LT_THR0_INTR_EN_CLR;

    SDL_VTM_configVd cfgVd;
    cfgVd.configVdCtrl = SDL_VTM_VD_CONFIG_CTRL_EVT_SEL;
    cfgVd.vd_temp_evts = SDL_VTM_VD_EVT_SELECT_TEMP_SENSOR_0;
    SDL_VTM_tsGlobal_cfg   tsGlobalCfg;
    tsGlobalCfg.validMap = 0;
    tsGlobalCfg.clkSel = 0;
    tsGlobalCfg.clkDiv = 0;
    tsGlobalCfg.any_maxt_outrg_alert_en = 0;
    tsGlobalCfg.maxt_outrg_alert_thr0 = 0;
    tsGlobalCfg.maxt_outrg_alert_thr = 0;
    tsGlobalCfg.samplesPerCnt = 0;
    cfgVd.tsGlobal_cfg = tsGlobalCfg;
    /* Set the temperature thresholds */

    SDL_VTM_initTs(SDL_VTM_INSTANCE_TS_0 , &cfgTs);

    /* enable the threshold interrupts */
    SDL_VTM_intrCntrl(SDL_VTM_INSTANCE_VD_DOMAIN_0, ctrl);

    /* enable the tracking of temperature events on this VD */
    SDL_VTM_initVd(SDL_VTM_INSTANCE_VD_DOMAIN_0, &cfgVd);
}

/*
* This function reads the current temperature in the VTM module
* and then modifies the VTM thresholds to cause events to trigger.
* Note that in a typical system, the thresholds would be kept the static, and
* changes in temperature would cause the events to occur.
*/
static int32_t vtmTriggerTh(int32_t lt_thr0_offset,
                            int32_t gt_thr1_offset,
                            int32_t gt_thr2_offset)
{
    int32_t             retVal = 0;
    int32_t             temp_milli_degrees_read;
    SDL_VTM_InstTs      insTs = SDL_VTM_INSTANCE_TS_0;
    SDL_VTM_adc_code    adc_code_read;
    SDL_VTM_adc_code    adc_code_lt_thr0, adc_code_gt_thr1, adc_code_gt_thr2;
    int32_t             gt_thr2_val, gt_thr1_val, lt_thr0_val;
    SDL_VTM_intrCtrl  ctrl;
    SDL_VTM_Stat_val statusVal;
    SDL_VTM_Stat_read_ctrl readCtrl;
    SDL_VTM_configTs cfgTs;
    readCtrl = SDL_VTM_TS_READ_DATA_OUT_VAL;

    /* Set temp sensor for continuous mode */
    cfgTs.configTsCtrl = SDL_VTM_VD_CONFIG_CTRL_SET_CTL;
    cfgTs.tsCtrl_cfg.valid_map = SDL_VTM_TS_CTRL_MODE_VALID;
    cfgTs.tsCtrl_cfg.mode = SDL_VTM_TS_CTRL_CONTINUOUS_MODE;
    SDL_VTM_initTs(SDL_VTM_INSTANCE_TS_0 , &cfgTs);

    /* Get current temperature value */
    if(currTestCase == 0)
	{
		SDL_DPL_delay(1);
	}
    SDL_VTM_getSensorStatus(insTs, &readCtrl, &statusVal);
    adc_code_read = statusVal.data_out;
    (void) SDL_VTM_tsConvADCToTemp (adc_code_read, insTs, \
                                   &temp_milli_degrees_read);

    DebugP_log ("sensor id                       : %d \r\n" \
                 "adc_code                        : %d \r\n" \
                 "temp in milli degree celcius    : %d \r\n", \
                 insTs, adc_code_read, temp_milli_degrees_read);


    /* Disable interrupts while changing thresholds */
    ctrl = (SDL_VTM_VD_GT_THR2_INTR_EN_CLR | \
            SDL_VTM_VD_GT_THR2_INTR_RAW_CLR | \
            SDL_VTM_VD_GT_THR1_INTR_EN_CLR | \
            SDL_VTM_VD_GT_THR1_INTR_RAW_CLR | \
            SDL_VTM_VD_LT_THR0_INTR_EN_CLR | \
            SDL_VTM_VD_LT_THR0_INTR_RAW_CLR);
    SDL_VTM_intrCntrl(SDL_VTM_INSTANCE_VD_DOMAIN_0, ctrl);


    /* Change all 3 thresholds relative to the current temperature */
    lt_thr0_val = temp_milli_degrees_read + lt_thr0_offset;
    gt_thr1_val = temp_milli_degrees_read + gt_thr1_offset;
    gt_thr2_val = temp_milli_degrees_read + gt_thr2_offset;

    SDL_VTM_tsConvTempToAdc(lt_thr0_val, insTs,  &adc_code_lt_thr0);
    SDL_VTM_tsConvTempToAdc(gt_thr1_val, insTs,  &adc_code_gt_thr1);
    SDL_VTM_tsConvTempToAdc(gt_thr2_val, insTs,  &adc_code_gt_thr2);


    DebugP_log ("vtmTriggerTh: Setting lt_thr0_val temp to " \
                 "%d millidegrees Celsius, and adc_code_lt_thr0 = %d\r\n",
                 lt_thr0_val,
                 adc_code_lt_thr0);
    DebugP_log ("vtmTriggerTh: Setting gt_thr1_val temp to " \
                 "%d millidegrees Celsius, and adc_code_gt_thr1 = %d\r\n",
                 gt_thr1_val,
                 adc_code_gt_thr1);
    DebugP_log ("vtmTriggerTh: Setting gt_thr2_val temp to " \
                 "%d millidegrees Celsius, and adc_code_gt_thr2 = %d\r\n",
                 gt_thr2_val,
                 adc_code_gt_thr2);


    setAllVTMTempThr(adc_code_lt_thr0, adc_code_gt_thr1, adc_code_gt_thr2);


    DebugP_log("Finished VTM threshold setting\r\n");

    return (retVal);
}

static int32_t SDL_VTM_config(void)
{
    int32_t retVal = 0;
#ifdef DEBUG
    DebugP_log("SDL_VTM_config: starting test with threshold change\r\n");
#endif
    retVal = vtmTriggerTh(-4000, -2000, 5000);

    return (retVal);
}

/*
* This function clears VTM THR0 interrupt (Less Than Temp event)
* for VTM input events to the ESM.
* It resets the VTM module to look for high temperature events again.
*/
void SDL_VTM_IntrruptLtThr0(void)
{
    SDL_VTM_intrCtrl ctrl;

    /* Ack the interrupt, by clearing the pending bit */
    ctrl = (SDL_VTM_VD_LT_THR0_INTR_EN_CLR | \
            SDL_VTM_VD_GT_THR1_INTR_EN_SET | \
            SDL_VTM_VD_GT_THR2_INTR_EN_SET | \
            SDL_VTM_VD_LT_THR0_INTR_RAW_CLR | \
            SDL_VTM_VD_GT_THR1_INTR_RAW_CLR | \
            SDL_VTM_VD_GT_THR2_INTR_RAW_CLR);
    SDL_VTM_intrCntrl(SDL_VTM_INSTANCE_VD_DOMAIN_0, ctrl);
    /* Print output ESM event to the screen */
    DebugP_log ("\r\n    Got ltThr0 interrupt through ESM module\r\n");
    DebugP_log ("\r\n    System at a temperature below the threshold of lt_thr0 \r\n"
                 "    System running at a safe temperature \r\n");
}

/*
* This function clears VTM THR1 interrupts for VTM input event to the ESM.
*/
void SDL_VTM_IntrruptGtThr1(void)
{
    SDL_VTM_intrCtrl ctrl;
    /*
    - disable the gt1 interrupt
    - clear the gt1 interrupt
    - clear the lt0 interrupt
    - enable the lt0 intterupt
    */
    ctrl = (SDL_VTM_VD_GT_THR1_INTR_EN_CLR  |  \
            SDL_VTM_VD_GT_THR1_INTR_RAW_CLR |  \
            SDL_VTM_VD_LT_THR0_INTR_EN_SET  |  \
            SDL_VTM_VD_LT_THR0_INTR_RAW_CLR);

    /* Ack and Re-enable the LT_THR0 interrupt to let system know if cooling
     * took place */
    SDL_VTM_intrCntrl(SDL_VTM_INSTANCE_VD_DOMAIN_0, ctrl);

    /* Print output ESM event to the screen */
    DebugP_log ("\r\n    Got gtThr1 interrupt through ESM module\r\n");
    DebugP_log ("\r\n    Crossed early warning threshold of gt_thr1 \r\n"
                 "    System should take action to implement system cooling \r\n");
}

/*
* This function clears VTM THR2 interrupts for VTM input event to the ESM.
*/
void SDL_VTM_IntrruptGtThr2(void)
{
    SDL_VTM_intrCtrl ctrl;
    /* Ack the interrupt, by clearing the pending bit */
    ctrl = (SDL_VTM_VD_GT_THR2_INTR_EN_CLR | \
            SDL_VTM_VD_GT_THR2_INTR_RAW_CLR);
    SDL_VTM_intrCntrl(SDL_VTM_INSTANCE_VD_DOMAIN_0, ctrl);

    /* Print output ESM event to the screen */
    DebugP_log ("\r\n    Got gtThr2 interrupt through ESM module\r\n");
    DebugP_log ("\r\n    Crossed critical threshold of gt_thr2 \r\n"
                 "    System should take critical action to implement system cooling \r\n");
}


int32_t vtm_setThresholdsForCriticalTrigger(void)
{
    int32_t retVal = 0;
#ifdef DEBUG
    DebugP_log("vtm_setThresholdsForCriticalTrigger: setting thresholds " \
                "to trigger high priority thermal event.\r\n");
#endif
    retVal = vtmTriggerTh(-12000, -8000, -3000);

    return (retVal);
}

/*
* This function resets the VTM thresholds back to some typical default
* values.
*/
int32_t vtm_setNormalThresholds(void)
{
    int32_t          retVal = 0;
    SDL_VTM_InstTs insTs = SDL_VTM_INSTANCE_TS_0;
    SDL_VTM_adc_code adc_code_lt_thr0, adc_code_gt_thr1, adc_code_gt_thr2;
    int32_t          gt_thr2_val, gt_thr1_val, lt_thr0_val;
    SDL_VTM_intrCtrl  ctrl;

#ifdef DEBUG
    DebugP_log("Start changing to normal VTM threshold setting\r\n");
#endif

    /* Disable interrupts while changing thresholds */
    ctrl = (SDL_VTM_VD_GT_THR2_INTR_EN_CLR | \
            SDL_VTM_VD_GT_THR2_INTR_RAW_CLR | \
            SDL_VTM_VD_GT_THR1_INTR_EN_CLR | \
            SDL_VTM_VD_GT_THR1_INTR_EN_CLR | \
            SDL_VTM_VD_LT_THR0_INTR_EN_CLR | \
            SDL_VTM_VD_LT_THR0_INTR_RAW_CLR);

    SDL_VTM_intrCntrl(SDL_VTM_INSTANCE_VD_DOMAIN_0, ctrl);

    /* Set to default values */
    lt_thr0_val = LT_THR0_DEFAULT;
    gt_thr1_val = GT_THR1_DEFAULT;
    gt_thr2_val = GT_THR2_DEFAULT;

    SDL_VTM_tsConvTempToAdc(lt_thr0_val, insTs, &adc_code_lt_thr0);
    SDL_VTM_tsConvTempToAdc(gt_thr1_val, insTs, &adc_code_gt_thr1);
    SDL_VTM_tsConvTempToAdc(gt_thr2_val, insTs, &adc_code_gt_thr2);

#ifdef DEBUG
    DebugP_log ("vtm_setNormalThresholds: Setting lt_thr0_val temp to %d " \
                 "millidegrees Celsius, and adc_code_lt_thr0 = %d\r\n",
                 lt_thr0_val,
                 adc_code_lt_thr0);
    DebugP_log ("vtm_setNormalThresholds: Setting gt_thr1_val temp to %d " \
                 "millidegrees Celsius, and adc_code_gt_thr1 = %d\r\n",
                 gt_thr1_val,
                 adc_code_gt_thr1);
    DebugP_log ("vtm_setNormalThresholds: Setting gt_thr2_val temp to %d " \
                 "millidegrees Celsius, and adc_code_gt_thr2 = %d\r\n",
                 gt_thr2_val,
                 adc_code_gt_thr2);
#endif

    setAllVTMTempThr(adc_code_lt_thr0, adc_code_gt_thr1, adc_code_gt_thr2);

#ifdef DEBUG
    DebugP_log("Finished normal VTM threshold setting\r\n");
#endif

    return (retVal);
}

/* USE CASE FUNCTIONS */

/*
* This function initiates the input trigger event for each use case
*/
int32_t vtm_runTestCaseTrigger(uint8_t useCaseId)
{
    int32_t       retVal = 0;
    DebugP_log("\r\nStarting Use Case %d \r\n", useCaseId);
    switch(useCaseId)
    {
        case 0:
            /* UC-1: Low Priority interrupt on MAIN ESM -
             * VTM less than THR1 */
            currEsmInstance = SDL_ESM_INST_MAIN_ESM0;
            retVal = SDL_VTM_config();
            if (retVal == 0) {
                DebugP_log("case 0 success\r\n");
                vtmEventInputTrig[useCaseId] = USE_CASE_STATUS_COMPLETED_SUCCESS;
            } else{
                DebugP_log("case 0 failure\r\n");
                vtmEventInputTrig[useCaseId] = USE_CASE_STATUS_COMPLETED_FAILURE;
            }
            break;

        case 1:
            /* UC-2: High Priority interrupt on MAIN ESM -
             * VTM greater than THR2 with clearing
             * of MCU_SAFETY_ERRORn pin */

            /* Start the Pin Control and Measurement Timer */
            currEsmInstance = SDL_ESM_INST_MAIN_ESM0;
            retVal = SDL_VTM_config();
            if (retVal == 0) {
                vtmEventInputTrig[useCaseId] = USE_CASE_STATUS_COMPLETED_SUCCESS;
            } else {
                vtmEventInputTrig[useCaseId] = USE_CASE_STATUS_COMPLETED_FAILURE;
            }
            break;

        default:
            DebugP_log("ERR: Invalid Test Case ID %d \r\n", useCaseId);
            retVal = -1;
            break;
    }

    return (retVal);
}

/* Nothing past this point */
