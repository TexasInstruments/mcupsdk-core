/*
 *  Copyright (C) 2021-2023 Texas Instruments Incorporated
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
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unity.h>
#include <drivers/ecap.h>
#include <drivers/epwm.h>
#include <drivers/edma.h>
#include <drivers/adc.h>
#include <drivers/cmpss.h>
#include <drivers/gpio.h>
#include <drivers/pinmux.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

#include "menu.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
#define TEST_ALL_INSTANCES (false)
/* None */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* ========================================================================== */

/* ========================================================================== */

#define PARAMS_RESET_VAL_EMU_MODE                       (0)
#define PARAMS_RESET_VAL_ECAP_MODE_APWM                 (0) // CAPTURE mode
#define PARAMS_RESET_VAL_CAPTURE_MODE_CONTINUOUS        (0) // continuous mode
#define PARAMS_RESET_VAL_WRAP_CAPTURE                   (3) // cap 4
#define PARAMS_RESET_VAL_PRESCALER                      (0) // DIV_by_1
#define PARAMS_RESET_VAL_POLARITY                       (0) // Raising edge
#define PARAMS_RESET_VAL_COUNTER_RESET_CAPTURE          (4) // NONE
#define PARAMS_RESET_VAL_PHASE_SHIFT_COUNT              (0)
#define PARAMS_RESET_VAL_LOAD_COUNTER_ENABLE            (0) // SYNCIN disabled
#define PARAMS_RESET_VAL_SYNC_OUT_MODE                  (0) // only SWSYNC
#define PARAMS_RESET_VAL_SYNC_IN_SOURCE                 (1) // resets to EPWM0_SYNCOUT
#define PARAMS_RESET_VAL_DMA_EVT_SOURCE                 (0) // CAP 1
#define PARAMS_RESET_VAL_SOC_EVT_SOURCE                 (0) // CAP 1
#define PARAMS_RESET_VAL_INPUT_SEL                      (0) // 0XFF (invalid input.)

#define NUM_ECAP_INSTANCES (10)
uint32_t ecap_base_addr[NUM_ECAP_INSTANCES] =
{
    CSL_CONTROLSS_ECAP0_U_BASE,
    CSL_CONTROLSS_ECAP1_U_BASE,
    CSL_CONTROLSS_ECAP2_U_BASE,
    CSL_CONTROLSS_ECAP3_U_BASE,
    CSL_CONTROLSS_ECAP4_U_BASE,
    CSL_CONTROLSS_ECAP5_U_BASE,
    CSL_CONTROLSS_ECAP6_U_BASE,
    CSL_CONTROLSS_ECAP7_U_BASE,
    CSL_CONTROLSS_ECAP8_U_BASE,
    CSL_CONTROLSS_ECAP9_U_BASE,
};

#define NUM_ADC_INSTANCES (5)
uint32_t adc_base_addr[NUM_ADC_INSTANCES] =
{
    CSL_CONTROLSS_ADC0_U_BASE,
    CSL_CONTROLSS_ADC1_U_BASE,
    CSL_CONTROLSS_ADC2_U_BASE,
    CSL_CONTROLSS_ADC3_U_BASE,
    CSL_CONTROLSS_ADC4_U_BASE,
};

#define NO_CHANNEL 99
uint32_t adc_available_channel[5] =
{
    ADC_CH_ADCIN2,
    ADC_CH_ADCIN2,
    ADC_CH_ADCIN2,
    ADC_CH_ADCIN2,
    NO_CHANNEL,
};

#define NUM_CMPSS_INSTANCES (32)
uint32_t cmpss_base_addr[NUM_CMPSS_INSTANCES]=
{
    CSL_CONTROLSS_CMPSSA0_U_BASE,
    CSL_CONTROLSS_CMPSSA1_U_BASE,
    CSL_CONTROLSS_CMPSSA2_U_BASE,
    CSL_CONTROLSS_CMPSSA3_U_BASE,
    CSL_CONTROLSS_CMPSSA4_U_BASE,
    CSL_CONTROLSS_CMPSSA5_U_BASE,
    CSL_CONTROLSS_CMPSSA6_U_BASE,
    CSL_CONTROLSS_CMPSSA7_U_BASE,
    CSL_CONTROLSS_CMPSSA8_U_BASE,
    CSL_CONTROLSS_CMPSSA9_U_BASE,
    CSL_CONTROLSS_CMPSSB0_U_BASE,
    CSL_CONTROLSS_CMPSSB1_U_BASE,
    CSL_CONTROLSS_CMPSSB2_U_BASE,
    CSL_CONTROLSS_CMPSSB3_U_BASE,
    CSL_CONTROLSS_CMPSSB4_U_BASE,
    CSL_CONTROLSS_CMPSSB5_U_BASE,
    CSL_CONTROLSS_CMPSSB6_U_BASE,
    CSL_CONTROLSS_CMPSSB7_U_BASE,
    CSL_CONTROLSS_CMPSSB8_U_BASE,
    CSL_CONTROLSS_CMPSSB9_U_BASE,
};

#define NUM_EPWM_INSTANCES (32)
uint32_t epwm_base_addr[NUM_EPWM_INSTANCES] =
{
    CSL_CONTROLSS_G0_EPWM0_U_BASE,
    CSL_CONTROLSS_G0_EPWM1_U_BASE,
    CSL_CONTROLSS_G0_EPWM2_U_BASE,
    CSL_CONTROLSS_G0_EPWM3_U_BASE,
    CSL_CONTROLSS_G0_EPWM4_U_BASE,
    CSL_CONTROLSS_G0_EPWM5_U_BASE,
    CSL_CONTROLSS_G0_EPWM6_U_BASE,
    CSL_CONTROLSS_G0_EPWM7_U_BASE,
    CSL_CONTROLSS_G0_EPWM8_U_BASE,
    CSL_CONTROLSS_G0_EPWM9_U_BASE,
    CSL_CONTROLSS_G0_EPWM10_U_BASE,
    CSL_CONTROLSS_G0_EPWM11_U_BASE,
    CSL_CONTROLSS_G0_EPWM12_U_BASE,
    CSL_CONTROLSS_G0_EPWM13_U_BASE,
    CSL_CONTROLSS_G0_EPWM14_U_BASE,
    CSL_CONTROLSS_G0_EPWM15_U_BASE,
    CSL_CONTROLSS_G0_EPWM16_U_BASE,
    CSL_CONTROLSS_G0_EPWM17_U_BASE,
    CSL_CONTROLSS_G0_EPWM18_U_BASE,
    CSL_CONTROLSS_G0_EPWM19_U_BASE,
    CSL_CONTROLSS_G0_EPWM20_U_BASE,
    CSL_CONTROLSS_G0_EPWM21_U_BASE,
    CSL_CONTROLSS_G0_EPWM22_U_BASE,
    CSL_CONTROLSS_G0_EPWM23_U_BASE,
    CSL_CONTROLSS_G0_EPWM24_U_BASE,
    CSL_CONTROLSS_G0_EPWM25_U_BASE,
    CSL_CONTROLSS_G0_EPWM26_U_BASE,
    CSL_CONTROLSS_G0_EPWM27_U_BASE,
    CSL_CONTROLSS_G0_EPWM28_U_BASE,
    CSL_CONTROLSS_G0_EPWM29_U_BASE,
    CSL_CONTROLSS_G0_EPWM30_U_BASE,
    CSL_CONTROLSS_G0_EPWM31_U_BASE,
};

ECAP_Events events[4] =
{
    ECAP_EVENT_1,
    ECAP_EVENT_2,
    ECAP_EVENT_3,
    ECAP_EVENT_4,
};

#define INPUT   (1)
#define OUTPUT  (0)
#define RISING  false
#define FALLING  true
#define GOOD     true
#define BAD      false

uint32_t gpio_base_addr = CSL_GPIO0_U_BASE;

/* GPIO59 -> EPWM8_A (G3) */
uint32_t gpio_pin = (PIN_EPWM8_A)>>2;
uint32_t pwm_input = (PIN_EPWM13_A)>>2;


uint32_t inputxbar_base_addr = CSL_CONTROLSS_INPUTXBAR_U_BASE;

typedef struct
{
    uint8_t emu_mode;
    bool ecap_mode_apwm;
    uint32_t apwm_period;
    uint32_t apwm_compare;
    uint16_t apwm_polarity;
    uint32_t input;
    bool capture_mode_continuous;
    uint16_t wrap_capture;
    uint16_t prescaler;
    uint16_t qual_prd;
    bool polarity[4];
    bool int_en;
    uint16_t int_source;
    uint8_t counter_reset_capture[4];
    uint32_t phase_shift_count;
    bool load_counter_en;
    uint8_t sync_out_mode;
    uint16_t sync_in_source;
    bool dma_evt_en;
    uint16_t dma_evt_source;
    bool soc_evt_en;
    uint16_t soc_evt_source;
    bool rearm;
} ECAP_config_t;

ECAP_config_t ECAP_config;

ECAP_config_t* ecap_config_ptr =(ECAP_config_t*) &ECAP_config;

/*
    instance 0 prescaler div_1 all events disabled
    onshot - capture mode - stops at evt 1 (falling)
*/
void util_ecap_config_reset(void)
{
    ecap_config_ptr->emu_mode = PARAMS_RESET_VAL_EMU_MODE;
    /* to capture mode */
    ecap_config_ptr->ecap_mode_apwm = PARAMS_RESET_VAL_ECAP_MODE_APWM;
    /* to oneshot mode */
    ecap_config_ptr->capture_mode_continuous = PARAMS_RESET_VAL_CAPTURE_MODE_CONTINUOUS;
    /* select the reset value of the input*/
    ecap_config_ptr->input = PARAMS_RESET_VAL_INPUT_SEL;
    /* to cap_evet3 */
    ecap_config_ptr->wrap_capture = PARAMS_RESET_VAL_WRAP_CAPTURE;
    /* minimum prescale DIV_BY_1*/
    ecap_config_ptr->prescaler = PARAMS_RESET_VAL_PRESCALER;

    /* reset value of qual period is byPass */
    ecap_config_ptr->qual_prd = ECAP_PULSE_WIDTH_FILTER_BYPASS;

    /* all to the falling edge polarity */
    for (int iter = 0; iter < 4; iter++)
        ecap_config_ptr->polarity[iter] = PARAMS_RESET_VAL_POLARITY;
    /* interrupt disable */
    ecap_config_ptr->int_en = false;
    /* No source*/
    ecap_config_ptr->int_source = 0x0;
    /* No event resets by default*/
    for (int iter = 0; iter < 4; iter++)
        ecap_config_ptr->counter_reset_capture[iter] = false;
    /* 32-bit value. 0 by reset*/
    ecap_config_ptr->phase_shift_count = 0x0;
    /* disabling the load counter */
    ecap_config_ptr->load_counter_en = PARAMS_RESET_VAL_LOAD_COUNTER_ENABLE;
    /* not SWSYNC to load the counter*/
    ecap_config_ptr->sync_out_mode = PARAMS_RESET_VAL_SYNC_OUT_MODE;
    /* disable by default*/
    ecap_config_ptr->sync_in_source = PARAMS_RESET_VAL_SYNC_IN_SOURCE;
    /* disable by default*/
    ecap_config_ptr->dma_evt_en = 0;
    /* setting to evt_1 or value 0 by default*/
    ecap_config_ptr->dma_evt_source = PARAMS_RESET_VAL_DMA_EVT_SOURCE;
    /* disabling by default SOC_evet*/
    ecap_config_ptr->soc_evt_en = false;
    /* setting to evt_1 or value 0 by default*/
    ecap_config_ptr->soc_evt_source = PARAMS_RESET_VAL_SOC_EVT_SOURCE;
    /* disabling by default. reset the counter, start loading on the cap registers*/
    ecap_config_ptr->rearm = false;
}

typedef struct{
    bool is_pulse;
    bool is_active_high;
    bool edges[4];
    bool check_timestamps[4];
    uint32_t timestamps[4];
} Input_params_t;

Input_params_t input_params_;

Input_params_t* input_params = (Input_params_t*) (&input_params_) ;

void util_input_params_init(void)
{
    input_params->is_pulse = 0;
    input_params->is_active_high = 0;
    for(int i = 0; i < 4; i++)
    {
        input_params->edges[i] = FALLING;
        input_params->check_timestamps[i] = false;
        input_params->timestamps[i] = 0;
    }
    return;
}

/*
Info: Procedure to configure the peripheral modes and interrupts
- Disable global interrupts
- Stop eCAP counter
- Disable eCAP interrupts
- Configure peripheral registers
- Clear spurious eCAP interrupt flags
- Enable eCAP interrupts
- Start eCAP counter
- Enable global interrupts
*/

/* Testcases */
static void ECAP_setEventPrescalerApiCheck(void *args);
static void ECAP_selection_of_sync_in_source(void *args);
static void ECAP_high_resolution_functions_for_ecap(void *args);
static void ECAP_input_evaluation_block(void *args);
static void ECAP_capture_mode_tests(void *args);
static void ECAP_add_additional_input_trigger_sources(void *args);
static void ECAP_support_interrupt_generation_on_either_of_4_events(void *args);
static void ECAP_selection_of_soc_trigger_source(void *args);
static void ECAP_selection_of_dma_trigger_source(void *args);

int32_t test_ecap_cases(uint8_t in);

/* ECAP util functions */
void util_ecap_reset(uint16_t instance);
void util_ecap_reset_all(void);
void util_ecap_start_timestamp(uint16_t instance);
void util_ecap_stop_timestamp(uint16_t instance);
void util_ecap_configure(uint16_t instance);
void util_ecap_deconfigure_and_reset(uint16_t instance);
bool util_ecap_wait_for_interrupt(uint16_t instance);
bool util_ecap_configure_input(void);
bool util_activate_input_pulses(void);
bool util_compare_in_range(int value1, int value2, int delta);
int util_validate_input_events(uint16_t instance, bool check_flags, int delta);

/* ADC util funcitons */
void util_adc_clear_interrupts(void);
bool util_adc_wait_for_interrupts(void);
bool util_adc_wait_for_interrupt(uint16_t adc_instance);
void util_adc_init_configure_soc_source(uint16_t adc_instance,uint16_t ecap_instance);
void util_adc_setup_ppbevt(uint16_t adc_instance, uint16_t event_number);
void util_adc_reset(uint16_t adc_instance);

/* CMPSS util functions */
void util_cmpss_init(uint16_t instance);
bool util_cmpss_set_status(uint16_t instance, uint16_t trip_signal);
void util_cmpss_deinit(uint16_t instance);
/* CMPSS global Variables */
uint16_t gTrip_high_type = 0;
uint16_t gTrip_low_type = 0;

/* GPIO util funcitons */
void util_gpio_configure(bool input);
void util_gpio_toggle(uint16_t times, uint32_t uSec_delay);
void util_gpio_pwm_input_configure(void);

/* EDMA util funcitons */
void util_edma_configure(uint16_t ecap_instance);
void util_edma_deconfigure(void);

/* EPWM util functions */
void util_epwm_enable_all(void);
void util_epwm_enable(uint16_t epwm_instance);
void util_epwm_setup(uint16_t epwm_instance, uint16_t ON_value, uint16_t period);
void util_epwm_enable_soc_trigger(uint16_t epwm_instance, EPWM_ADCStartOfConversionType epwm_soc_type);
bool util_epwm_wait_on_soc_trigger(uint16_t epwm_instance, EPWM_ADCStartOfConversionType epwm_soc_type);
void util_epwm_reset(int8_t epwm_instance);
void util_epwm_sync(uint16_t epwm_instance, uint16_t main_pwm_instance, uint16_t phase_shift_value);
void util_epwm_disable_all(void);
void util_xbar_configure_sync_dut(uint32_t epwm_instance);

/* test implementing functions */
int32_t AM263x_ECAP_BTR_001(uint16_t instance);
int32_t AM263x_ECAP_BTR_002(uint16_t instance);
int32_t AM263x_ECAP_BTR_003(uint16_t instance);
int32_t AM263x_ECAP_BTR_004(uint16_t instance);
int32_t AM263x_ECAP_BTR_005(uint16_t instance);
int32_t AM263x_ECAP_BTR_006(void);
int32_t AM263x_ECAP_BTR_007(uint16_t instance);


/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* ========================================================================== */
/*                            EDMA Related variabls                           */

#define CONFIG_EDMA0 (0U)
#define CONFIG_EDMA_NUM_INSTANCES (1U)
/* Event queue to be used for EDMA transfer */
#define EDMA_TEST_EVT_QUEUE_NO  0U

static EDMA_Object gEdmaObjects[CONFIG_EDMA_NUM_INSTANCES];

/* EDMA Driver handles */
EDMA_Handle gEdmaHandle[CONFIG_EDMA_NUM_INSTANCES];

/* EDMA Driver Open Parameters */
EDMA_Params gEdmaParams[CONFIG_EDMA_NUM_INSTANCES] =
{
    {
        .intrEnable = TRUE,
    },
};
static EDMA_Attrs gEdmaAttrs[CONFIG_EDMA_NUM_INSTANCES] =
{
    {

        .baseAddr           = CSL_TPCC0_U_BASE,
        .compIntrNumber     = CSLR_R5FSS0_CORE0_INTR_TPCC0_INTAGGR,
        .intrAggEnableAddr  = CSL_MSS_CTRL_U_BASE + CSL_MSS_CTRL_TPCC0_INTAGG_MASK,
        .intrAggEnableMask  = 0x1FF & (~(2U << 0)),
        .intrAggStatusAddr  = CSL_MSS_CTRL_U_BASE + CSL_MSS_CTRL_TPCC0_INTAGG_STATUS,
        .intrAggClearMask   = (2U << 0),
        .initPrms           =
        {
            .regionId     = 0,
            .queNum       = 0,
            .initParamSet = FALSE,
            .ownResource    =
            {
                .qdmaCh      = 0x03U,
                .dmaCh[0]    = 0xFFFFFFFFU,
                .dmaCh[1]    = 0x000000FFU,
                .tcc[0]      = 0xFFFFFFFFU,
                .tcc[1]      = 0x000000FFU,
                .paramSet[0] = 0xFFFFFFFFU,
                .paramSet[1] = 0xFFFFFFFFU,
                .paramSet[2] = 0xFFFFFFFFU,
                .paramSet[3] = 0xFFFFFFFFU,
                .paramSet[4] = 0xFFFFFFFFU,
                .paramSet[5] = 0xFFFFFFFFU,
                .paramSet[6] = 0xFFFFFFFFU,
                .paramSet[7] = 0x000007FFU,
            },
            .reservedDmaCh[0]    = 0x00000000U,
            .reservedDmaCh[1]    = 0x00000000U,
        },
    },
};
EDMA_Config gEdmaConfig[CONFIG_EDMA_NUM_INSTANCES] =
{
    {
        &gEdmaAttrs[CONFIG_EDMA0],
        &gEdmaObjects[CONFIG_EDMA0],
    },
};

/* EDMA global variables */
uint32_t gEdmaConfigNum = CONFIG_EDMA_NUM_INSTANCES;
EDMACCPaRAMEntry    edmaParam;
uint32_t            edma_base_addr, regionId;
uint32_t            dmaCh, tcc, param;


void util_edma_configure(uint16_t ecap_instance)
{
    /* configuring the ECAP DMA INT To the dmaxbar and trigxbar.*/
    SOC_xbarSelectEdmaTrigXbarInputSource(CSL_EDMA_TRIG_XBAR_U_BASE, DMA_TRIG_XBAR_EDMA_MODULE_0, DMA_TRIG_XBAR_DMA_XBAR_OUT_0);
    SOC_xbarSelectDMAXBarInputSource(CSL_CONTROLSS_DMAXBAR_U_BASE, 0, 5, 0, 0, 0, 0, 0, ecap_instance);

    int32_t  status     = SystemP_SUCCESS;

    gEdmaHandle[0] = NULL;   /* Init to NULL so that we can exit gracefully */

    /* Open all instances */
    gEdmaHandle[0] = EDMA_open(0, &gEdmaParams[0]);
    if(NULL == gEdmaHandle[0])
    {
        DebugP_logError("EDMA open failed for instance %d !!!\r\n", 0);
        status = SystemP_FAILURE;
    }

    if(SystemP_FAILURE == status)
    {
        util_edma_deconfigure();   /* Exit gracefully */
    }

/* -----------------------------------EDMA Setup------------------------------------ */
    int32_t testStatus = SystemP_SUCCESS;

    edma_base_addr = EDMA_getBaseAddr(gEdmaHandle[0]);
    DebugP_assert(edma_base_addr != 0);

    regionId = EDMA_getRegionId(gEdmaHandle[0]);
    DebugP_assert(regionId < SOC_EDMA_NUM_REGIONS);

    dmaCh = DMA_TRIG_XBAR_EDMA_MODULE_0;
    testStatus = EDMA_allocDmaChannel(gEdmaHandle[0], &dmaCh);
    DebugP_assert(testStatus == SystemP_SUCCESS);

    tcc = EDMA_RESOURCE_ALLOC_ANY;
    testStatus = EDMA_allocTcc(gEdmaHandle[0], &tcc);
    DebugP_assert(testStatus == SystemP_SUCCESS);

    param = EDMA_RESOURCE_ALLOC_ANY;
    testStatus = EDMA_allocParam(gEdmaHandle[0], &param);
    DebugP_assert(testStatus == SystemP_SUCCESS);

    EDMA_configureChannelRegion(edma_base_addr, regionId, EDMA_CHANNEL_TYPE_DMA,
        dmaCh, tcc, param, EDMA_TEST_EVT_QUEUE_NO);

    EDMA_ccPaRAMEntry_init(&edmaParam);
    edmaParam.srcAddr       = (uint32_t) SOC_virtToPhy((void *)(0x501c0000));
    edmaParam.destAddr      = (uint32_t) SOC_virtToPhy((void *) 0x50260000);
    edmaParam.aCnt          = (uint16_t) 1;
    edmaParam.bCnt          = (uint16_t) 4;
    edmaParam.cCnt          = (uint16_t) 1;
    edmaParam.bCntReload    = (uint16_t) 0;
    edmaParam.srcBIdx       = (int16_t) EDMA_PARAM_BIDX(0);
    edmaParam.destBIdx      = (int16_t) EDMA_PARAM_BIDX(0);
    edmaParam.srcCIdx       = (int16_t) 0;
    edmaParam.destCIdx      = (int16_t) 0;
    edmaParam.linkAddr      = 0xFFFFU;
    edmaParam.srcBIdxExt    = (int8_t) EDMA_PARAM_BIDX_EXT(0);
    edmaParam.destBIdxExt   = (int8_t) EDMA_PARAM_BIDX_EXT(0);
    edmaParam.opt           = (EDMA_OPT_TCINTEN_MASK | EDMA_OPT_ITCINTEN_MASK |
                            ((((uint32_t)tcc) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));

    EDMA_setPaRAM(edma_base_addr, param, &edmaParam);

    EDMA_enableTransferRegion(edma_base_addr, regionId, dmaCh,
                            EDMA_TRIG_MODE_EVENT);
/* -----------------------------EDMA Setup Complete-------------------------------- */

    return;
}

void util_edma_deconfigure(void)
{
    /* Close all instances that are open */

    int32_t testStatus = SystemP_SUCCESS;

    EDMA_clrIntrRegion(edma_base_addr, regionId, tcc);

    EDMA_freeChannelRegion(edma_base_addr, regionId, EDMA_CHANNEL_TYPE_DMA,
        dmaCh, EDMA_TRIG_MODE_EVENT, tcc, EDMA_TEST_EVT_QUEUE_NO);

    testStatus = EDMA_freeDmaChannel(gEdmaHandle[0], &dmaCh);
    DebugP_assert(testStatus == SystemP_SUCCESS);
    testStatus = EDMA_freeTcc(gEdmaHandle[0], &tcc);
    DebugP_assert(testStatus == SystemP_SUCCESS);
    testStatus = EDMA_freeParam(gEdmaHandle[0], &param);
    DebugP_assert(testStatus == SystemP_SUCCESS);

    if(gEdmaHandle[0] != NULL)
    {
        EDMA_close(gEdmaHandle[0]);
        gEdmaHandle[0] = NULL;
    }
    /*configuring the DMA xbar, trig xbar to default values*/
    return;
}

/* ========================================================================== */

/* None */
/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

#define TOTAL_TEST_CASES (9)

void test_main(void *args)
{
    /* Open drivers */
    Drivers_open();
    Board_driversOpen();

    tester_init();
    UNITY_BEGIN();
    char test_title[] = "----------------------ECAP-TEST-CASES----------------------";

    menu_input test_list_input_independent[TOTAL_TEST_CASES] =
    {
        {0, 3330,  ECAP_setEventPrescalerApiCheck,                         "ECAP_setEventPrescalerApiCheck"},
        {0, 9476,  ECAP_selection_of_dma_trigger_source,                   "ECAP_selection_of_dma_trigger_source"},
        {0, 7906,  ECAP_selection_of_soc_trigger_source,                   "ECAP_selection_of_soc_trigger_source"},
        {0, 3403,  ECAP_input_evaluation_block,                            "ECAP_input_evaluation_block"},
        {0, 3404,  ECAP_capture_mode_tests,                                "ECAP_capture_mode_tests"},
        {0, 3406,  ECAP_add_additional_input_trigger_sources,              "ECAP_add_additional_input_trigger_sources"},
        {0, 3407,  ECAP_support_interrupt_generation_on_either_of_4_events,"ECAP_support_interrupt_generation_on_either_of_4_events"},
        {0, 3402,  ECAP_high_resolution_functions_for_ecap,                "ECAP_high_resolution_functions_for_ecap"},
        {1, 3401,  ECAP_selection_of_sync_in_source,                       "ECAP_selection_of_sync_in_source"},
    };

    menu(TOTAL_TEST_CASES, test_list_input_independent, test_title);

    if(enableLog)
    {
        DebugP_logZoneEnable(DebugP_LOG_ZONE_ERROR);
        DebugP_logZoneEnable(DebugP_LOG_ZONE_INFO);
    }
    UNITY_END();

    /* Close drivers */
    Board_driversClose();
    Drivers_close();

    return;
}

/* Unity framework required information */
void setUp(void)
{
    util_ecap_reset_all();
}

void tearDown(void)
{
    util_ecap_reset_all();
}

void util_ecap_reset(uint16_t instance)
{
    SOC_generateEcapReset(instance);
}
void util_ecap_reset_all(void)
{
    for (uint32_t instance = 0; instance < 10; instance++)
    {
        SOC_generateEcapReset(instance);
    }
}

/* resets ecap. sets the config_ptr to given configuration */
void util_ecap_configure(uint16_t instance)
{   
    util_ecap_reset(instance);
    uint32_t base = ecap_base_addr[instance];

    /* Disables time stamp capture */
	ECAP_disableTimeStampCapture(base);
    /* Stops Time stamp counter */
	ECAP_stopCounter(base);


    /* Disable ,clear all capture flags and interrupts */
    /* TODO : Add Signal Monitoring events */
    if(ecap_config_ptr->int_en)
    {
        ECAP_enableInterrupt(base, ecap_config_ptr->int_source);
    	ECAP_clearInterrupt(base, ecap_config_ptr->int_source);
    }else
    {
        ECAP_disableInterrupt(base, ECAP_ISR_SOURCE_ALL);
        ECAP_clearInterrupt(base, ECAP_ISR_SOURCE_ALL);
    }

    if(ecap_config_ptr->ecap_mode_apwm)
    {
	    /* Sets eCAP in PWM mode. */
        ECAP_enableAPWMMode(base);
        /* Set eCAP APWM period */
        ECAP_setAPWMPeriod(base, ecap_config_ptr->apwm_period);
        /* Set eCAP APWM on or off time count */
        ECAP_setAPWMCompare(base, ecap_config_ptr->apwm_compare);
        /* Set eCAP APWM polarity */
        ECAP_setAPWMPolarity(base, ecap_config_ptr->apwm_polarity);
    }
    else
    {
        /* ECAP is in Capture mode by reset i.e., ECAP_enableCaptureMode(base);*/
        ECAP_enableCaptureMode(base);

	    /* Sets the capture mode (continuous or oneshot),*/
	    ECAP_setCaptureMode(base, ecap_config_ptr->capture_mode_continuous, ecap_config_ptr->wrap_capture);
	    /* Sets the Capture event polarity */
	    ECAP_setEventPrescaler(base, ecap_config_ptr->prescaler);

        /* Sets the Capture event polarity and Configure counter reset on events */
        for(int event = 0; event < 4; event++)
        {
            ECAP_setEventPolarity(base, event, ecap_config_ptr->polarity[event]);

            if(ecap_config_ptr->counter_reset_capture[event])
            {
                ECAP_enableCounterResetOnEvent(base, event);
            }
            else
            {
                ECAP_disableCounterResetOnEvent(base, event);
            }
        }

        /* Select eCAP input */
        ECAP_selectECAPInput(base, ecap_config_ptr->input);
        ECAP_selectQualPeriod(base, ecap_config_ptr->qual_prd);
    }
    /* Sets a phase shift value count */
    ECAP_setPhaseShiftCount(base, ecap_config_ptr->phase_shift_count);

    uint16_t syncout_source = (ecap_config_ptr->sync_out_mode) >> 3;
    /* Configures Sync out signal mode */
    ECAP_setSyncOutMode(base, syncout_source);

    /* Configures emulation mode */
    ECAP_setEmulationMode(base, ecap_config_ptr->emu_mode);

    /* Set up the source for sync-in pulse */
    ECAP_setSyncInPulseSource(base, ecap_config_ptr->sync_in_source);

    if((ecap_config_ptr->load_counter_en) == 0)
    {
        /* Disable counter loading with phase shift value when selected sync-in event occurs*/
        ECAP_disableLoadCounter(base);
    }
    else
    {
        ECAP_enableLoadCounter(base);
    }
    /* Resetting the Modulo counter, Counter, Event flags */
    ECAP_resetCounters(base);
    /* Start Time stamp counter for instance  and enable Timestamp capture separately
        the following may be used
        - ECAP_startCounter(base);
        - ECAP_enableTimeStampCapture(base); */
    if(ecap_config_ptr->rearm)
    {
        ECAP_reArm(base);
    }
    return;
}

void util_ecap_start_timestamp(uint16_t instance)
{
    uint32_t base = ecap_base_addr[instance];
    ECAP_enableTimeStampCapture(base);
    ECAP_startCounter(base);
    return;
}

void util_ecap_stop_timestamp(uint16_t instance)
{
    uint32_t base = ecap_base_addr[instance];
    ECAP_disableTimeStampCapture(base);
    ECAP_stopCounter(base);
    return;
}
/* resets given instance */
void util_ecap_deconfigure_and_reset(uint16_t instance)
{
    SOC_generateEcapReset(instance);
    util_ecap_config_reset();
    util_ecap_configure(instance);
    return;
}
/* returns 1 if wait is successful i.e., interrupt flag set */
bool util_ecap_wait_for_interrupt(uint16_t instance)
{
    int counter_max = 100;
    int count = 0;
    uint32_t base = ecap_base_addr[instance];
    while(count < counter_max)
    {
        if(ECAP_getGlobalInterruptStatus(base) == 1)
        {
            /* interrupt occured wait successful*/
            return 1;
        }
        count++;
    }
    /* interrupt did not occur. wait unsuccessful*/
    return 0;
}
/* clears all the ADC interrupt ADC_INT1 */
void util_adc_clear_interrupts(void)
{
    for(int adc_instance = 0; adc_instance < 5; adc_instance++)
    {
        uint32_t adc_base = adc_base_addr[adc_instance];
        ADC_clearInterruptStatus(adc_base, ADC_INT_NUMBER1);
    }
    return;
}
/* returns 1 if wait is successful i.e., interrupt flag set */
bool util_adc_wait_for_interrupts(void)
{
    int counter_max = 1000;
    int errors = 0;
    for(int adc_instance = 0; adc_instance < 5; adc_instance++)
    {
        int count = 0;
        uint32_t adc_base = adc_base_addr[adc_instance];
        while(count < counter_max)
        {
            if(ADC_getInterruptStatus(adc_base, ADC_INT_NUMBER1) == true)
            {
                break;
            }
            count++;
        }

        if(count >= counter_max)
        {
            /* wait failed.*/
            DebugP_logError("failed at %d\r\n",adc_instance);
            errors++;
        }
    }
    if(errors > 0 )
    {
        return 0;
    }
    /* all waits successful */
    return 1;
}

/* returns 1 if wait is successful i.e., interrupt flag set */
bool util_adc_wait_for_interrupt(uint16_t adc_instance)
{
    int count = 0;
    int errors = 0;
    int counter_max = 1000;

    uint32_t adc_base = adc_base_addr[adc_instance];
    while(count < counter_max)
    {
        if(ADC_getInterruptStatus(adc_base, ADC_INT_NUMBER1) == true)
        {
            break;
        }
        count++;
    }

    if(count >= counter_max)
    {
        /* wait failed.*/
        DebugP_logError("failed at %d\r\n", adc_instance);
        errors++;
    }

    if(errors > 0 )
    {
        return 0;
    }
    /* all waits successful */
    return 1;
}
/* returns 1 if wait is successful i.e., interrupt flag set */
bool util_dma_wait_for_interrupts(void)
{
    int counter_max = 1000;
    int count = 0;

    int errors = 0;

    while(count < counter_max)
    {
        if(EDMA_readIntrStatusRegion(edma_base_addr, regionId, tcc) == true)
        {
            break;
        }
        count++;
    }

    if(count >= counter_max)
    {
        /* wait failed.*/
        DebugP_logError("wait failed\r\n");
        errors++;
    }

    if(errors > 0 )
    {
        return 0;
    }
    /* all waits successful */
    return 1;
}
/* configures the given adc instance with the ECAP trigger */
void util_adc_init_configure_soc_source(uint16_t adc_instance, uint16_t ecap_instance)
{
    uint32_t adc_base = adc_base_addr[adc_instance];

    SOC_enableAdcReference((adc_base & 0x0000F000)>>12);
    /*
     * this util function is meant to..
     * Set Prescalar to ADC_CLK_DIV_3_0
     * Set Mode to signal_mode
     * Set interrupt Pulse Mode to interrupt_pulse_mode
     */
    ADC_setPrescaler(adc_base, ADC_CLK_DIV_3_0);
    ADC_setMode(adc_base, ADC_RESOLUTION_12BIT, ADC_MODE_SINGLE_ENDED);
    ADC_setInterruptPulseMode(adc_base, ADC_PULSE_END_OF_CONV);
    ADC_setSOCPriority(adc_base, ADC_PRI_ALL_ROUND_ROBIN);
    ADC_enableConverter(adc_base);
    ClockP_usleep(500);

    /* disabling and enabling the interrupt, setting the intterupt source as the ecap soc signal*/
    ADC_disableInterrupt(adc_base, ADC_INT_NUMBER1);
    ADC_enableInterrupt(adc_base, ADC_INT_NUMBER1);
    ADC_setInterruptSource(adc_base, ADC_INT_NUMBER1, ADC_SOC_NUMBER0);
    ADC_clearInterruptStatus(adc_base, ADC_INT_NUMBER1);
    ADC_clearInterruptOverflowStatus(adc_base, ADC_INT_NUMBER1);

    /* Setting up the SOC configuration for the given ecap_instance SOC signal.*/
    uint16_t ecap_soc_trigger = ADC_TRIGGER_ECAP0_SOCEVT + ecap_instance;
    ADC_setupSOC(adc_base, ADC_SOC_NUMBER0, ecap_soc_trigger, ADC_CH_ADCIN0, 16);

    return ;
}

void util_adc_setup_ppbevt(uint16_t adc_instance, uint16_t event_number)
{
    uint32_t adc_base = adc_base_addr[adc_instance];
    uint16_t ppb_number = event_number;
    SOC_enableAdcReference(adc_instance);
    ADC_setPrescaler(adc_base, ADC_CLK_DIV_4_0);
    ADC_setMode(adc_base, ADC_RESOLUTION_12BIT, ADC_MODE_SINGLE_ENDED);
    ADC_setInterruptPulseMode(adc_base, ADC_PULSE_END_OF_CONV);
    ADC_setSOCPriority(adc_base, ADC_PRI_ALL_ROUND_ROBIN);
    ADC_enableConverter(adc_base);
    ClockP_usleep(500);

    ADC_setupSOC(adc_base, ADC_SOC_NUMBER1, ADC_TRIGGER_SW_ONLY, adc_available_channel[adc_instance], 20);
    ADC_enableInterrupt(adc_base, ADC_INT_NUMBER1);
    ADC_setInterruptSource(adc_base,ADC_INT_NUMBER1, ADC_SOC_NUMBER1);
    ADC_clearInterruptStatus(adc_base, ADC_INT_NUMBER1);
    ADC_setupPPB(adc_base, ppb_number, ADC_SOC_NUMBER1);
    ADC_enablePPBEvent(adc_base, ppb_number, ADC_EVT_TRIPHI);
    ADC_clearPPBEventStatus(adc_base, ppb_number, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO | ADC_EVT_ZERO));
    ADC_setPPBTripLimits(adc_base, ppb_number, 3000, 1000);
    ADC_setPPBReferenceOffset(adc_base, ppb_number, 0);
    ADC_enablePPBEventCBCClear(adc_base, ppb_number);
    /* lowering input voltage on ADC for no accidental pulses */
    char test_command[CMD_SIZE];
    sprintf(test_command, "provide analog voltage of %.4fV on ADC %d Channel %d", 0.0, 0, 0);
    tester_command(test_command);
    sprintf(test_command, "provide analog voltage of %.4fV on ADC %d Channel %d", 0.0, 0, 1);
    tester_command(test_command);

    /* forcing one SOC for clearing the ADC SOC/PPB result registers and events.*/
    ADC_forceSOC(adc_base, ADC_SOC_NUMBER1);
    (void)util_adc_wait_for_interrupt(adc_instance);

    ADC_clearInterruptStatus(adc_base, ADC_INT_NUMBER1);
    ADC_clearInterruptOverflowStatus(adc_base, ADC_INT_NUMBER1);

    return;
}
/* resets the ADC instance */
void util_adc_reset(uint16_t adc_instance)
{
    SOC_generateAdcReset(adc_instance);
    return;
}

void util_cmpss_init(uint16_t instance)
{
    uint32_t cmpss_base = cmpss_base_addr[instance];
    uint16_t cmpss_instance_a;
    uint16_t cmpss_instance_b;

    DebugP_logInfo("instance : %d\r\n", instance);
    if(instance > 9)
    {
        cmpss_instance_b = 9 - instance;
        SOC_generateCmpssbReset(cmpss_instance_b);
    }
    else
    {
        cmpss_instance_a = instance;
        SOC_generateCmpssaReset(cmpss_instance_a);
    }

    CMPSS_enableModule(cmpss_base);
    CMPSS_configHighComparator(cmpss_base, CMPSS_INSRC_PIN);
    CMPSS_configOutputsHigh(cmpss_base, CMPSS_TRIP_ASYNC_COMP | CMPSS_TRIPOUT_ASYNC_COMP);
    CMPSS_configHighComparator(cmpss_base, CMPSS_INV_INVERTED | CMPSS_INSRC_PIN);

    CMPSS_configLowComparator(cmpss_base, CMPSS_INSRC_PIN);
    CMPSS_configOutputsLow(cmpss_base, CMPSS_TRIP_ASYNC_COMP | CMPSS_TRIPOUT_ASYNC_COMP);
    CMPSS_configLowComparator(cmpss_base, CMPSS_INV_INVERTED | CMPSS_INSRC_PIN);

    ClockP_usleep(5);
    uint16_t trip_high_status;
    uint16_t trip_low_status ;

    trip_high_status = CMPSS_getStatus(cmpss_base) & CMPSS_STS_HI_FILTOUT;
    trip_low_status = CMPSS_getStatus(cmpss_base) & CMPSS_STS_LO_FILTOUT;

    DebugP_logInfo("cmpss trip_high_status : %x trip_low_status : %x \r\n",trip_high_status ,trip_low_status);

    if(trip_high_status != 0)
    {
        CMPSS_configHighComparator(cmpss_base, 0 | CMPSS_INSRC_PIN);
        ClockP_usleep(5);
        gTrip_high_type = 1;
        trip_high_status = CMPSS_getStatus(cmpss_base) & CMPSS_STS_HI_FILTOUT;
        DebugP_logInfo("cmpss trip_high_status : %x \r\n",trip_high_status);
    }
    else
    {
        gTrip_high_type = 0;
    }
    if(trip_low_status != 0)
    {
        CMPSS_configLowComparator(cmpss_base, 0 | CMPSS_INSRC_PIN);
        ClockP_usleep(5);
        gTrip_low_type = 1;
        trip_low_status = CMPSS_getStatus(cmpss_base) & CMPSS_STS_LO_FILTOUT;
        DebugP_logInfo("cmpss trip_low_status : %x \r\n",trip_low_status);

    }
    else
    {
        gTrip_low_type = 0;
    }

    /* Assume that the tripHi and tripLow signals are Low now.*/
}

bool util_cmpss_set_status(uint16_t instance, uint16_t trip_signal)
{
    uint32_t cmpss_base = cmpss_base_addr[instance];
    uint16_t trip_low = 0;
    uint16_t trip_high = 1;

    bool status = GOOD;

    uint16_t trip_high_status;
    uint16_t trip_low_status;

    trip_high_status =     CMPSS_getStatus(cmpss_base) & CMPSS_STS_HI_FILTOUT;
    trip_low_status  =     CMPSS_getStatus(cmpss_base) & CMPSS_STS_LO_FILTOUT;

    DebugP_logInfo("tripHigh : %x\r\n", trip_high_status);
    DebugP_logInfo("tripLow : %x\r\n", trip_low_status);

    if(trip_signal == trip_high)
    {
        if(gTrip_high_type)
        {
            /* not inverted state. move to inverted state to get rising edge*/
            CMPSS_configHighComparator(cmpss_base, CMPSS_INV_INVERTED | CMPSS_INSRC_PIN);
            ClockP_usleep(5);
        }
        else
        {
            /* inverted state. move to not inverted state to get rising edge*/
            CMPSS_configHighComparator(cmpss_base, 0 | CMPSS_INSRC_PIN);
            ClockP_usleep(5);
        }

        trip_high_status =  CMPSS_getStatus(cmpss_base) & CMPSS_STS_HI_FILTOUT;

        if(trip_high_status == 0)
        {
            status = BAD;
            DebugP_logError("tripHigh : %x\r\n", trip_high_status);
        }
    }
    else
    if(trip_signal == trip_low)
    {
        if(gTrip_low_type)
        {
            /* not inverted state. move to inverted state to get rising edge*/
            CMPSS_configLowComparator(cmpss_base, CMPSS_INV_INVERTED | CMPSS_INSRC_PIN);
            ClockP_usleep(5);
        }
        else
        {
            /* inverted state. move to not inverted state to get rising edge*/
            CMPSS_configLowComparator(cmpss_base, 0 | CMPSS_INSRC_PIN);
            ClockP_usleep(5);
        }

        trip_low_status = CMPSS_getStatus(cmpss_base) & CMPSS_STS_LO_FILTOUT;

        if(trip_low_status == 0)
        {
            status = BAD;
            DebugP_logError("tripLow : %x\r\n", trip_low_status);
        }
    }
    DebugP_logInfo("tripHigh : %x\r\n", trip_high_status);
    DebugP_logInfo("tripLow : %x\r\n", trip_low_status);

    return status;
}
void util_cmpss_deinit(uint16_t instance)
{
    uint16_t cmpss_instance_a;
    uint16_t cmpss_instance_b;
    if(instance > 9)
    {
        cmpss_instance_b = 9 - instance;
        SOC_generateCmpssbReset(cmpss_instance_b);
    }
    else
    {
        cmpss_instance_a = instance;
        SOC_generateCmpssaReset(cmpss_instance_a);
    }
}

/* configures GPIO (declared global) as input or output */
void util_gpio_configure(bool input)
{
    /* Setting EPWM PIN mux (overwriting if already if existed)*/
    Pinmux_PerCfg_t gPinMuxMainDomainCfg[] = {
        /* GPIO59 -> EPWM8_A (G3) */
        {
            PIN_EPWM8_A,
            ( PIN_MODE(7) | PIN_PULL_DISABLE | PIN_SLEW_RATE_HIGH )
        },

        {PINMUX_END, PINMUX_END}
    };
    Pinmux_config(gPinMuxMainDomainCfg, PINMUX_DOMAIN_ID_MAIN);

    /*setting the GPIO direction*/
    if(input)
    {
        GPIO_setDirMode(gpio_base_addr, gpio_pin, INPUT);
    }
    else
    {
        GPIO_setDirMode(gpio_base_addr, gpio_pin, OUTPUT);
    }
    return;
}
/* toggles the GPIO (declared global) for \b times with \b uSecs delay*/
void util_gpio_toggle(uint16_t times, uint32_t uSec_delay)
{
    DebugP_testLog("toggling GPIO %d\r\n", gpio_pin);
    /* if uSec_delay is <=0 then only toggle without any delay*/
    if(uSec_delay == 0)
    {
        for(int iter = 0; iter < times; iter++)
        {
            GPIO_pinWriteHigh(gpio_base_addr, gpio_pin);
            GPIO_pinWriteLow(gpio_base_addr, gpio_pin);
        }
        return;
    }

    uint32_t uSec_delay_by_2 = uSec_delay << 1;

    for(int iter = 0; iter < times; iter++)
    {
        GPIO_pinWriteHigh(gpio_base_addr, gpio_pin);
        ClockP_usleep(uSec_delay_by_2);
        GPIO_pinWriteLow(gpio_base_addr, gpio_pin);
        ClockP_usleep(uSec_delay_by_2);
    }

    return;
}

/* Configures the Input Pinmux and sets as GPIO input for PWM input */
void util_gpio_pwm_input_configure(void)
{

    Pinmux_PerCfg_t gPinMuxMainDomainCfg[] = {
                /* GPIO0 pin config */
        /* GPIO69 -> EPWM13_A (K4) */
        {
            PIN_EPWM13_A,
            ( PIN_MODE(7) | PIN_PULL_DISABLE | PIN_SLEW_RATE_HIGH )
        },

        {
            PIN_I2C1_SDA,
        ( PIN_MODE(7) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW | PIN_QUAL_SYNC | PIN_GPIO_R5SS0_0 )
        },

        {PINMUX_END, PINMUX_END}
    };
    Pinmux_config(gPinMuxMainDomainCfg, PINMUX_DOMAIN_ID_MAIN);

    GPIO_setDirMode(gpio_base_addr, pwm_input, INPUT);
    GPIO_setDirMode(gpio_base_addr, (PIN_I2C1_SDA)>>2, INPUT);
}

void util_epwm_enable_all(void)
{
    for(int i = 0; i<=31 ; i++)
    {
        SOC_setEpwmTbClk(i, TRUE);
        SOC_setEpwmGroup(i, 0);
    }

    DebugP_testLog("Enabled TB CLK for all EPWM, set group as 0\r\n");
}

void util_epwm_enable(uint16_t epwm_instance)
{
    SOC_setEpwmTbClk(epwm_instance, TRUE);
    SOC_setEpwmGroup(epwm_instance, 0);
}

/* sets up the given epwm to given period, ON_value values, enables syncout pulse by default at counter = 0 */
void util_epwm_setup(uint16_t epwm_instance, uint16_t ON_value, uint16_t period)
{
    uint32_t epwm_base = epwm_base_addr[epwm_instance];
    /* reset before setting up the configuration */
    util_epwm_reset(epwm_instance);
    EPWM_setClockPrescaler(epwm_base, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);
    EPWM_setTimeBasePeriod(epwm_base, period);
    EPWM_disablePhaseShiftLoad(epwm_base);
    EPWM_setTimeBaseCounter(epwm_base, 0);
    EPWM_setTimeBaseCounterMode(epwm_base, EPWM_COUNTER_MODE_STOP_FREEZE);

    EPWM_setCounterCompareShadowLoadMode(epwm_base, EPWM_COUNTER_COMPARE_A, EPWM_COMP_LOAD_ON_CNTR_ZERO);
    EPWM_setCounterCompareValue(epwm_base, EPWM_COUNTER_COMPARE_A, ON_value);
    EPWM_setActionQualifierAction(epwm_base, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(epwm_base, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);

    EPWM_setCounterCompareShadowLoadMode(epwm_base, EPWM_COUNTER_COMPARE_B, EPWM_COMP_LOAD_ON_CNTR_ZERO);
    EPWM_setCounterCompareValue(epwm_base, EPWM_COUNTER_COMPARE_B, ON_value);
    EPWM_setActionQualifierAction(epwm_base, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(epwm_base, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);

    EPWM_enableSyncOutPulseSource(epwm_base, EPWM_SYNC_OUT_PULSE_ON_CNTR_ZERO);

    DebugP_testLog("\tPWM %d is set for Period : %d, ON_value : %d\r\n", epwm_instance, period, ON_value);
}

void util_epwm_enable_soc_trigger(uint16_t epwm_instance, EPWM_ADCStartOfConversionType epwm_soc_type)
{
    uint32_t epwm_base = epwm_base_addr[epwm_instance];

    EPWM_enableADCTrigger(epwm_base, epwm_soc_type);
    EPWM_setADCTriggerSource(epwm_base, epwm_soc_type, EPWM_SOC_TBCTR_U_CMPA, 0);
	EPWM_setADCTriggerEventPrescale(epwm_base, epwm_soc_type, 1);
	EPWM_disableADCTriggerEventCountInit(epwm_base, epwm_soc_type);
	EPWM_setADCTriggerEventCountInitValue(epwm_base, epwm_soc_type, 0);
}

bool util_epwm_wait_on_soc_trigger(uint16_t epwm_instance, EPWM_ADCStartOfConversionType epwm_soc_type)
{
    uint32_t epwm_base = epwm_base_addr[epwm_instance];
    int counter_max = 100;
    int count = 0;

    while(count < counter_max)
    {
        if(EPWM_getADCTriggerFlagStatus(epwm_base, epwm_soc_type) == 1)
        {
            /* interrupt occured wait successful*/
            return 1;
        }
        count++;
    }
    /* interrupt did not occur. wait unsuccessful*/
    return 0;

}

void util_epwm_reset(int8_t epwm_instance)
{
    SOC_generateEpwmReset(epwm_instance);
}

void util_epwm_sync(uint16_t epwm_instance, uint16_t main_pwm_instance, uint16_t phase_shift_value)
{
    EPWM_SyncInPulseSource sync_in_source = EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM0 + main_pwm_instance;
    uint32_t epwm_base_address = epwm_base_addr[epwm_instance];
	EPWM_setSyncInPulseSource(epwm_base_address, sync_in_source);
	EPWM_enablePhaseShiftLoad(epwm_base_address);
	EPWM_setPhaseShift(epwm_base_address, phase_shift_value);
}

void util_epwm_disable_all(void)
{
    for (int i = 0; i <= 31; i++)
    {
        SOC_setEpwmTbClk(i, FALSE);
    }
}

/* Configures xbar instance to route the syncout signal from given EPWM to GPIO to sync at TESTER */
void util_xbar_configure_sync_dut(uint32_t epwm_instance)
{

    util_epwm_enable(epwm_instance);

    HW_WR_REG32(CSL_IOMUX_U_BASE + CSL_IOMUX_IO_CFG_KICK0,0x83E70B13);
    HW_WR_REG32(CSL_IOMUX_U_BASE + CSL_IOMUX_IO_CFG_KICK1,0x95A4F1E0);
    HW_WR_REG32(CSL_IOMUX_U_BASE + CSL_IOMUX_PR0_PRU1_GPO18_CFG_REG, 0x5);

    SOC_xbarSelectOutputXBarInputSource(CSL_CONTROLSS_OUTPUTXBAR_U_BASE, 14, 0, 0, 0, 0, 0, 0, 0, 0, 0, OUTPUT_XBAR_EPWM_SYNCOUT_XBAR0, 0);
    SOC_xbarSelectLatchOutputXBarOutputSignal(CSL_CONTROLSS_OUTPUTXBAR_U_BASE, 0);
    SOC_xbarSelectStretchedPulseOutputXBarOutputSignal(CSL_CONTROLSS_OUTPUTXBAR_U_BASE, 0);
    SOC_xbarSelectPWMSyncOutXBarInput(CSL_CONTROLSS_PWMSYNCOUTXBAR_U_BASE, 0, (1<<epwm_instance));
}

/* Testcase 1 - Check the ECAP_setEventPrescaler API */
static void ECAP_setEventPrescalerApiCheck(void *args)
{
    DebugP_testLog("Test : Prescaler API Check\r\n");
    TEST_ASSERT_EQUAL_INT32(test_ecap_cases(0), 0);
}
static void ECAP_support_interrupt_generation_on_either_of_4_events(void *args)
{
    DebugP_testLog("Test : Interrupt generation on various events\r\n");
    TEST_ASSERT_EQUAL_INT32(test_ecap_cases(1), 0);
}
static void ECAP_selection_of_soc_trigger_source(void *args)
{
    DebugP_testLog("Test : ADC_SOC event generation on various events\r\n");
    TEST_ASSERT_EQUAL_INT32(test_ecap_cases(2), 0);
}
static void ECAP_selection_of_dma_trigger_source(void *args)
{
    DebugP_testLog("Test : DMA event generation on various events\r\n");
    TEST_ASSERT_EQUAL_INT32(test_ecap_cases(3), 0);
}
static void ECAP_add_additional_input_trigger_sources(void *args)
{
    DebugP_testLog("Test : Input Sources to ECAP\r\n");
    TEST_ASSERT_EQUAL_INT32(test_ecap_cases(4), 0);
}
static void ECAP_capture_mode_tests(void *args)
{
    DebugP_testLog("Test : Capture Mode tests. Tests the mode of captures and counter reset on each capture.\r\n");
    TEST_ASSERT_EQUAL_INT32(test_ecap_cases(5), 0);
}
static void ECAP_selection_of_sync_in_source(void *args)
{
    DebugP_testLog("Test : Sync In Sources from EPWM tests\r\n");
    TEST_ASSERT_EQUAL_INT32(test_ecap_cases(6), 0);
}
static void ECAP_input_evaluation_block(void *args)
{
    DebugP_testLog("Test : Input Evaluation block tests - glitch filter and Prescaler checks\r\n");
    TEST_ASSERT_EQUAL_INT32(test_ecap_cases(7), 0);
}
static void ECAP_high_resolution_functions_for_ecap(void *args)
{
    UNITY_TEST_IGNORE("","TODO");
}
/*
configure and test the interrupt sources.
interrupt sources for ECAP are the following
- ECAP_ISR_SOURCE_CAPTURE_EVENT_1   - Event 1 generates interrupt
- ECAP_ISR_SOURCE_CAPTURE_EVENT_2   - Event 2 generates interrupt
- ECAP_ISR_SOURCE_CAPTURE_EVENT_3   - Event 3 generates interrupt
- ECAP_ISR_SOURCE_CAPTURE_EVENT_4   - Event 4 generates interrupt
- ECAP_ISR_SOURCE_COUNTER_OVERFLOW  - Counter overflow generates interrupt
- ECAP_ISR_SOURCE_COUNTER_PERIOD    - Counter equal period generates
                                      interrupt
- ECAP_ISR_SOURCE_COUNTER_COMPARE   - Counter equal compare generates
                                      interrupt

TODO : test the Signal monitoring events as sources and test.
*/
int32_t AM263x_ECAP_BTR_001(uint16_t instance)
{
    DebugP_testLog("------------- instance : %d-------------\r\n", instance);

    int errors = 0;
    uint32_t base = ecap_base_addr[instance];
    uint32_t interrupt_event_flag = 0;
    uint32_t interrupt_source[7] = {
        ECAP_ISR_SOURCE_CAPTURE_EVENT_1,
        ECAP_ISR_SOURCE_CAPTURE_EVENT_2,
        ECAP_ISR_SOURCE_CAPTURE_EVENT_3,
        ECAP_ISR_SOURCE_CAPTURE_EVENT_4,
        ECAP_ISR_SOURCE_COUNTER_OVERFLOW,
        ECAP_ISR_SOURCE_COUNTER_PERIOD,
        ECAP_ISR_SOURCE_COUNTER_COMPARE,
    };
    /* reset the current ecap */
    util_ecap_deconfigure_and_reset(instance);

    for(int iter = 0; iter < 7; iter++)
    {
        if((interrupt_source[iter] == ECAP_ISR_SOURCE_COUNTER_PERIOD) ||
           (interrupt_source[iter] == ECAP_ISR_SOURCE_COUNTER_COMPARE))
        {
            /* these are active only in pwm mode.*/
            ECAP_enableAPWMMode(base);
        }
        else
        {
            /* these are active only in capture mode.*/
            ECAP_enableCaptureMode(base);
        }

        /* clearing the interrupt flags from all sources*/
        ECAP_clearInterrupt(base, ECAP_ISR_SOURCE_ALL);

        /* Interrupt source selection */
        ECAP_enableInterrupt(base, interrupt_source[iter]);

        /* force interrupt event*/
        ECAP_forceInterrupt(base, interrupt_source[iter]);

        if(util_ecap_wait_for_interrupt(instance) == false)
        {
            errors++;
            DebugP_logError("wait failed for source : %x\r\n", interrupt_source[iter]);
        }
        else
        {
            interrupt_event_flag = ECAP_getInterruptSource(base);
            if(interrupt_event_flag != interrupt_source[iter])
            {
                errors++;
                DebugP_logError("incorrect interrupt source failed\r\n");
            }
        }
    }

    if(errors > 0)
    {
        /* Test fail */
        return 1;
    }
    /* test pass */
    return 0;
}

/* configures the trigger sources and cycles through events */
int16_t check_soc_dma_triggers(uint16_t instance, int event_out)
{
    int soc_trigger_check = 0;

    int errors = 0;
    bool wait_success = false;

    uint32_t event_source[7] = {
        ECAP_CAP_MODE_SOC_TRIGGER_SRC_CEVT1,
        ECAP_CAP_MODE_SOC_TRIGGER_SRC_CEVT2,
        ECAP_CAP_MODE_SOC_TRIGGER_SRC_CEVT3,
        ECAP_CAP_MODE_SOC_TRIGGER_SRC_CEVT4,
        ECAP_APWM_MODE_SOC_TRIGGER_SRC_PRD,
        ECAP_APWM_MODE_SOC_TRIGGER_SRC_CMP,
        ECAP_APWM_MODE_SOC_TRIGGER_SRC_PRD_CMP,
    };

    uint32_t base = ecap_base_addr[instance];

    /* set GPIOx as inputxbar for the usage in this test case */
    util_gpio_configure(OUTPUT);

    /* configure input Xbar to a gpio pin.*/
    SOC_xbarSelectInputXBarInputSource(inputxbar_base_addr, 0, 0, gpio_pin, 0);

    for(int iter = 0; iter < 7; iter++)
    {
        uint16_t source = event_source[iter];

        ECAP_clearInterrupt(base, ECAP_ISR_SOURCE_ALL);
            ECAP_stopCounter(base);

        /* Observation :
        ECAP_forceInterrupt(base, event); doesn't help generating an SOC signal to ADC */
        if(soc_trigger_check == event_out)
        {
            ECAP_selectSocTriggerSource(base, source);
        }
        else
        {
            ECAP_setDMASource(base,source);
        }
        if( (source == ECAP_APWM_MODE_SOC_TRIGGER_SRC_PRD) ||
            (source == ECAP_APWM_MODE_SOC_TRIGGER_SRC_CMP) ||
            (source == ECAP_APWM_MODE_SOC_TRIGGER_SRC_PRD_CMP))
        {
            /* these work in the pwm mode*/
            ECAP_enableAPWMMode(base);
            /* stop the couter to avoid accidental triggers*/
            ECAP_stopCounter(base);
            /* reset the counter*/
            ECAP_resetCounters(base);

            if(source == ECAP_APWM_MODE_SOC_TRIGGER_SRC_PRD_CMP)
            {

                /* setting cmp > prd to verify CMP generating the event*/
                ECAP_setAPWMCompare(base, 10000);
                ECAP_setAPWMPeriod(base, 10);

                /*starting the counter*/
                ECAP_startCounter(base);

                if(event_out == soc_trigger_check)
                {
                    wait_success = util_adc_wait_for_interrupts();
                }
                else
                {
                    wait_success = util_dma_wait_for_interrupts();
                }
                if(wait_success == false)
                {
                    /*Interrupts did not occur. i.e., trigger failed*/
                    errors++;
                    DebugP_logError("wait failed. for source %d \r\n", source);
                }

                /* stopping the couter to avoid accidental triggers*/
                ECAP_stopCounter(base);

                /* resetting the counter*/
                ECAP_resetCounters(base);

                /* Clear Interrupts*/
                if(event_out == soc_trigger_check)
                {
                    util_adc_clear_interrupts();
                }
                else
                {
                    /*clear DMA interrupts*/
                    EDMA_clrIntrRegion(edma_base_addr, regionId, tcc);
                    util_edma_deconfigure();
                    util_edma_configure(instance);
                }
                ECAP_clearInterrupt(base, ECAP_ISR_SOURCE_ALL);

                /* setting cmp < prd to verify PRD generating the event
                CMP is already verified above*/

                ECAP_setAPWMCompare(base, 10);
                ECAP_setAPWMPeriod(base, 10000);

                /*starting the counter*/
                ECAP_startCounter(base);

                if(event_out == soc_trigger_check)
                {
                    wait_success = util_adc_wait_for_interrupts();
                }
                else
                {
                    wait_success = util_dma_wait_for_interrupts();
                }
                if(wait_success == false)
                {
                    /*Interrupts did not occur. i.e., trigger failed*/
                    errors++;
                    DebugP_logError("wait failed. for source %d \r\n", source);
                }

                /* stopping the couter to avoid accidental triggers*/
                ECAP_stopCounter(base);
                /* resetting the counter*/
                ECAP_resetCounters(base);

                /* Clear Interrupts*/
                if(event_out == soc_trigger_check)
                {
                    util_adc_clear_interrupts();
                }
                else
                {
                    /*clear DMA interrupts*/
                    EDMA_clrIntrRegion(edma_base_addr, regionId, tcc);
                    util_edma_deconfigure();
                    util_edma_configure(instance);

                }
                ECAP_clearInterrupt(base, ECAP_ISR_SOURCE_ALL);

            }
            else
            {
                ECAP_stopCounter(base);
                ECAP_resetCounters(base);

                if(source == ECAP_APWM_MODE_SOC_TRIGGER_SRC_CMP)
                {
                    ECAP_setAPWMCompare(base, 10);
                    ECAP_setAPWMPeriod(base, 10000);
                }
                else
                {
                    ECAP_setAPWMCompare(base, 10000);
                    ECAP_setAPWMPeriod(base, 10);
                }
                ECAP_startCounter(base);

                if(event_out == soc_trigger_check)
                {
                    wait_success = util_adc_wait_for_interrupts();
                }
                else
                {
                    wait_success = util_dma_wait_for_interrupts();
                }
                if(wait_success == false)
                {
                    /*Interrupts did not occur. i.e., trigger failed*/
                    errors++;
                    DebugP_logError("wait failed. for source %d \r\n", source);
                }

                /* stopping the couter to avoid accidental triggers*/
                ECAP_stopCounter(base);
                /* resetting the counter*/
                ECAP_resetCounters(base);

                /* Clear Interrupts*/
                if(event_out == soc_trigger_check)
                {
                    util_adc_clear_interrupts();
                }
                else
                {
                    /*clear DMA interrupts*/
                    EDMA_clrIntrRegion(edma_base_addr, regionId, tcc);
                    util_edma_deconfigure();
                    util_edma_configure(instance);
                }
                ECAP_clearInterrupt(base, ECAP_ISR_SOURCE_ALL);
            }
        }
        else
        {
            /* these work in the Capture mode.*/
            ECAP_enableCaptureMode(base);
            ECAP_reArm(base);

            /* set ecap input as inputxbar */
            ECAP_selectECAPInput(base, ECAP_INPUT_INPUTXBAR0);

            /* set the CAP events to have Raising edge */
            for(int iter = ECAP_EVENT_1; iter <= ECAP_EVENT_4; iter++)
            {
                ECAP_setEventPolarity(base, iter, ECAP_EVNT_RISING_EDGE);
            }

            ECAP_startCounter(base);
            ECAP_enableTimeStampCapture(base);
            /* toggle the GPIO --> inputxbar x number of times. to generate event.*/
            int num_pulses = source+1;      /* +1 to satify indexing */
            uint32_t uSec_delay = 10;       /* arbitary */

            util_gpio_toggle(num_pulses, uSec_delay);

            if(event_out == soc_trigger_check)
            {
                wait_success = util_adc_wait_for_interrupts();
            }
            else
            {
                wait_success = util_dma_wait_for_interrupts();
            }
            if(wait_success == false)
            {
                /*Interrupts did not occur. i.e., trigger failed*/
                errors++;
                DebugP_logError("wait failed. for source %d \r\n", source);
            }

            /* Clear Interrupts*/
            if(event_out == soc_trigger_check)
            {
                util_adc_clear_interrupts();
            }
            else
            {
                /*clear DMA interrupts*/
                EDMA_clrIntrRegion(edma_base_addr, regionId, tcc);
                util_edma_deconfigure();
                util_edma_configure(instance);
            }

        }
    }
    /* unset GPIOx as inputxbar for the usage in this test case */
    SOC_xbarSelectInputXBarInputSource(inputxbar_base_addr, 0, 0, 0, 0);

    return errors;
}

/* ADC_SOC event generation from ECAP
the following can be used to trigger ADC_SOC:

    ECAP_CAP_MODE_SOC_TRIGGER_SRC_CEVT1 --> reset value.
    ECAP_CAP_MODE_SOC_TRIGGER_SRC_CEVT2
    ECAP_CAP_MODE_SOC_TRIGGER_SRC_CEVT3
    ECAP_CAP_MODE_SOC_TRIGGER_SRC_CEVT4
    ECAP_APWM_MODE_SOC_TRIGGER_SRC_PRD
    ECAP_APWM_MODE_SOC_TRIGGER_SRC_CMP
    ECAP_APWM_MODE_SOC_TRIGGER_SRC_PRD_CMP

to disable :
    ECAP_APWM_MODE_SOC_TRIGGER_SRC_DISABLED
*/
int32_t AM263x_ECAP_BTR_002(uint16_t instance)
{
    DebugP_testLog("------------- instance : %d-------------\r\n", instance);

    int errors = 0;
    int soc_trigger_check = 0;

    /* configuring ADCs */
    for(int adc_instance = 0; adc_instance < 5; adc_instance++)
    {
        util_adc_reset(adc_instance);
        util_adc_init_configure_soc_source(adc_instance, instance);
    }

    errors = check_soc_dma_triggers(instance, soc_trigger_check);

    for(int adc_instance = 0; adc_instance < 5; adc_instance++)
    {
        util_adc_reset(adc_instance);
    }

    if(errors > 0 )
    {
        /* test fail */
        return 1;
    }
    return 0;
}

/* DMA event generation from ECAP
the following can be used to trigger ADC_SOC:

    ECAP_CAP_MODE_SOC_TRIGGER_SRC_CEVT1 --> reset value.
    ECAP_CAP_MODE_SOC_TRIGGER_SRC_CEVT2
    ECAP_CAP_MODE_SOC_TRIGGER_SRC_CEVT3
    ECAP_CAP_MODE_SOC_TRIGGER_SRC_CEVT4
    ECAP_APWM_MODE_SOC_TRIGGER_SRC_PRD
    ECAP_APWM_MODE_SOC_TRIGGER_SRC_CMP
    ECAP_APWM_MODE_SOC_TRIGGER_SRC_PRD_CMP

to disable :
    ECAP_APWM_MODE_SOC_TRIGGER_SRC_DISABLED
*/

int32_t AM263x_ECAP_BTR_003(uint16_t instance)
{
    DebugP_testLog("------------- instance : %d-------------\r\n", instance);

    int errors = 0;
    int dma_trigger_check = 1;
    uint16_t ecap_instance = instance;

    /* configuring DMA */
    util_edma_configure(ecap_instance);

    errors = check_soc_dma_triggers(instance, dma_trigger_check);

    /* deconfigure DMAs*/
    util_edma_deconfigure();

    if(errors > 0 )
    {
        /* test fail */
        return 1;
    }
    return 0;
}

/* returns status if configurations are set properly*/
bool util_ecap_configure_input(void)
{
    bool status = GOOD;
    ECAP_InputCaptureSignals input = ecap_config_ptr->input;

    input_params->check_timestamps[0] = false;
    input_params->check_timestamps[1] = false;
    input_params->check_timestamps[2] = false;
    input_params->check_timestamps[3] = false;

    if( (input >= ECAP_INPUT_INPUTXBAR0) && (input <= ECAP_INPUT_INPUTXBAR31) )
    {
        /* InputXbar type. expect four pulses so the stop/wrap capture should be event 4*/
        /* input source is xbar instance */
        /* set the input event count to be 4 and timestamps to be of 1 uSec */
        util_gpio_configure(OUTPUT);

        uint8_t inputxbar_instance = (uint8_t) (input - ECAP_INPUT_INPUTXBAR0);

        SOC_xbarSelectInputXBarInputSource(inputxbar_base_addr, inputxbar_instance, 0, gpio_pin, 0);

        ecap_config_ptr->wrap_capture = ECAP_EVENT_4;
        for(int iter = 0; iter <= ecap_config_ptr->wrap_capture; iter++)
        {
        /* setting all rising edges */
            ecap_config_ptr->polarity[iter] = RISING;
            ecap_config_ptr->counter_reset_capture[iter] = true;
            input_params->check_timestamps[iter] = false;
            input_params->edges[iter] = RISING;
        }

    }
    if ((input >= ECAP_INPUT_EPWM0_SOCA) && (input <= ECAP_INPUT_EPWM31_SOCB))
    {
        /* Setting to capture only one pulse.*/
        ecap_config_ptr->wrap_capture = ECAP_EVENT_2;
        ecap_config_ptr->counter_reset_capture[0] = true;
        ecap_config_ptr->polarity[0] = RISING;
        ecap_config_ptr->polarity[1] = FALLING;

        /* setting up the input */
        /* ECAP_INPUT_EPWM0_SOCA to ECAP_INPUT_EPWM31_SOCA go from 54 to 85*/
        /* ECAP_INPUT_EPWM0_SOCB to ECAP_INPUT_EPWM31_SOCB go from 86 to 117*/
        uint16_t epwm_instance;
        uint16_t epwm_soc;
        uint16_t silly_period_value = 200;
        uint16_t silly_on_value  = (silly_period_value)>>1;

        if(input >= ECAP_INPUT_EPWM0_SOCB)
        {
            epwm_instance = input - ECAP_INPUT_EPWM0_SOCB;
            epwm_soc = EPWM_SOC_B;
        }
        else
        {
            epwm_instance = input - ECAP_INPUT_EPWM0_SOCA;
            epwm_soc = EPWM_SOC_A;
        }
        util_epwm_enable(epwm_instance);
        util_epwm_setup(epwm_instance, silly_on_value, silly_period_value);
        util_epwm_enable_soc_trigger(epwm_instance, epwm_soc);
    }

    if((input >= ECAP_INPUT_ADC0_EVT0) && (input <= ECAP_INPUT_ADC4_EVT3))
    {
        uint16_t number_of_events_per_instance = 4;

        uint16_t adc_instance = (input - ECAP_INPUT_ADC0_EVT0)/number_of_events_per_instance;
        uint16_t event = (input - ECAP_INPUT_ADC0_EVT0)%number_of_events_per_instance;

        util_adc_setup_ppbevt(adc_instance, event);

        ecap_config_ptr->wrap_capture = ECAP_EVENT_1;
        ecap_config_ptr->polarity[0] = RISING;
    }

    if((input >= ECAP_INPUT_CMPSSA0_CTRIP_LOW) && (input <= ECAP_INPUT_CMPSSB9_CTRIP_HIGH))
    {
        uint16_t number_of_events_per_instance = 2;
        uint16_t cmpss_instance = (input - ECAP_INPUT_CMPSSA0_CTRIP_LOW) / number_of_events_per_instance;

        DebugP_logInfo("instance : %d\r\n", cmpss_instance);

        util_cmpss_init(cmpss_instance);

        /* the util_cmpss_set_status will generate a rising edge */
        ecap_config_ptr->wrap_capture = ECAP_EVENT_1;
        ecap_config_ptr->polarity[0] = RISING;
    }
    return status;
}

bool util_activate_input_pulses(void)
{
    bool status = GOOD;
    ECAP_InputCaptureSignals input = ecap_config_ptr->input;

    if( (input >= ECAP_INPUT_INPUTXBAR0) && (input <= ECAP_INPUT_INPUTXBAR31) )
    {
        /* input source is xbar instance */
        /* toggle the GPIO 4 times with a delay of 1 uSec*/
        int num_pulses = 4;      /* +1 to satify indexing */
        util_gpio_toggle(num_pulses, 0);
    }
    if( (input >= ECAP_INPUT_EPWM0_SOCA) && (input <= ECAP_INPUT_EPWM31_SOCB) )
    {
        uint16_t epwm_instance;
        uint16_t epwm_soc;

        if(input >= ECAP_INPUT_EPWM0_SOCB)
        {
            epwm_instance = input - ECAP_INPUT_EPWM0_SOCB;
            epwm_soc = EPWM_SOC_B;
        }
        else
        {
            epwm_instance = input - ECAP_INPUT_EPWM0_SOCA;
            epwm_soc = EPWM_SOC_A;
        }
        /* running the Time base counter of epwm to generete the EPMW SOC triggers*/
        EPWM_setTimeBaseCounterMode(epwm_base_addr[epwm_instance], EPWM_COUNTER_MODE_UP);
        /* the EPWM SOC flag need not be cleared. this pulse keeps occuring even if SOC flag is not cleared.*/
        /* wait until atleast one flag has occured.*/
        (void)util_epwm_wait_on_soc_trigger(epwm_instance, epwm_soc);
        /* stopping the time base counter */
        EPWM_setTimeBaseCounterMode(epwm_base_addr[epwm_instance], EPWM_COUNTER_MODE_STOP_FREEZE);
        util_epwm_reset(epwm_instance);
        util_epwm_disable_all();
    }
    if( (input >= ECAP_INPUT_ADC0_EVT0) && (input <= ECAP_INPUT_ADC4_EVT3) )
    {
        uint16_t number_of_events_per_instance = 4;

        uint16_t adc_instance = (input - ECAP_INPUT_ADC0_EVT0)/number_of_events_per_instance;
        uint16_t event = (input - ECAP_INPUT_ADC0_EVT0)%number_of_events_per_instance;
        uint16_t ppb_number = event;
        if(adc_available_channel[adc_instance] != NO_CHANNEL)
        {
            char test_command[CMD_SIZE];
            sprintf(test_command, "provide analog voltage of %.4fV on ADC %d Channel %d", 2.5, 0, 0);
            tester_command(test_command);
            sprintf(test_command, "provide analog voltage of %.4fV on ADC %d Channel %d", 2.5, 0, 1);
            tester_command(test_command);
            /* force conversion for event generation */
            ADC_forceSOC(adc_base_addr[adc_instance], ADC_SOC_NUMBER1);
            status = util_adc_wait_for_interrupt(adc_instance);

            if(status == BAD)
            {
                return status;
            }
            uint32_t ppb_event_status = ADC_getPPBEventStatus(adc_base_addr[adc_instance], (ADC_PPBNumber) ppb_number);
            ppb_event_status &= ADC_EVT_TRIPHI;
            if(ppb_event_status != ADC_EVT_TRIPHI)
            {
                status = BAD;
                DebugP_logError("PPB event status : %d\r\n", ppb_event_status);
            }

        }
    }
    if((input >= ECAP_INPUT_CMPSSA0_CTRIP_LOW) && (input <= ECAP_INPUT_CMPSSB9_CTRIP_HIGH))
    {
        uint16_t number_of_events_per_instance = 2;
        uint16_t cmpss_instance = (input - ECAP_INPUT_CMPSSA0_CTRIP_LOW) / number_of_events_per_instance;
        uint16_t event = input % number_of_events_per_instance;

        /* the util_cmpss_set_status will generate a rising edge */
        status = util_cmpss_set_status(cmpss_instance, event);
        if(status == BAD)
        {
            DebugP_logError("cmpss set status BAD\r\n");
        }
        util_cmpss_deinit(cmpss_instance);
    }
    return status;
}

bool util_compare_in_range(int value1, int value2, int delta)
{
    int max_value = (value1 > value2)? value1 : value2;
    int min_value = (value1 > value2)? value2 : value1;

    if((max_value - min_value) >= delta)
    {
        DebugP_logError("values differ out of range\r\n");
        return false;
    }
    return true;
}

int util_validate_input_events(uint16_t instance, bool check_flags, int delta)
{
    int errors = 0;
    uint32_t ecap_base = ecap_base_addr[instance];

    ECAP_InputCaptureSignals input = ecap_config_ptr->input;

    uint16_t flags = ECAP_getInterruptSource(ecap_base);
    /* for expected events are each bits. */
    uint16_t expected_flags = (1<<((ecap_config_ptr->wrap_capture)+1))-1;

    if((input >= ECAP_INPUT_ADC4_EVT0) && (input <= ECAP_INPUT_ADC4_EVT3))
    {
        return errors;
    }
    if(check_flags && ((flags>>1)!= (expected_flags)))
    {
        errors++;
        DebugP_logError("ERROR: expected number of events did not occur.\r\n");
        DebugP_testLog("\t\tExpected flags: %x\r\n",expected_flags);
        DebugP_testLog("\r\n\t\tSet flags: %x\r\n", flags>>1);
       
    }
    if( ((input >= ECAP_INPUT_INPUTXBAR0) && (input <= ECAP_INPUT_INPUTXBAR31))
        ||
        ((input >= ECAP_INPUT_EPWM0_SOCA) && (input <= ECAP_INPUT_EPWM31_SOCB))
        ||
        ((input >= ECAP_INPUT_ADC0_EVT0) && (input <= ECAP_INPUT_ADC4_EVT3))
        ||
        ((input >= ECAP_INPUT_CMPSSA0_CTRIP_LOW) && (input <= ECAP_INPUT_CMPSSB9_CTRIP_HIGH)))
    {
        /* input source is xbar instance, the GPIO toggle for 4 times is already validated above.
           hence, this can be skipped. */

        for (int iter = 0; iter <= ecap_config_ptr->wrap_capture; iter++)
        {

            if(!(input_params->check_timestamps[iter]))
            {
                continue;
            }
            uint32_t timestamp = ECAP_getEventTimeStamp(ecap_base, events[iter]);
            
            DebugP_testLog("Timestamp %d is %d\r\n", iter, timestamp);

            if(util_compare_in_range(input_params->timestamps[iter], timestamp, delta))
            {
                continue;
            }
            errors++;
            DebugP_logError("\tTimeStamps for event %d donot match\r\n",iter);
            DebugP_testLog("\t\tExpected: %d\r\n", input_params->timestamps[iter]);
            DebugP_testLog("\t\tReturned: %d\r\n", timestamp);
    
        }

    }

    else
    {
        /* invalid input source */
        errors++;
        DebugP_logError("invalid input source\r\n");
    }
    return errors;
}
/* Tests the input sources to the ECAP */
int32_t AM263x_ECAP_BTR_004(uint16_t instance)
{
    DebugP_testLog("------------- instance : %d-------------\r\n", instance);

    int errors = 0;
    /* Add additional signals too*/
    ECAP_InputCaptureSignals input;

    for(input = ECAP_INPUT_FSI_RX0_TRIG_0;
        input <= ECAP_INPUT_INPUTXBAR31;
        input++)
    {   if(((input >= ECAP_INPUT_INPUTXBAR0) && (input <= ECAP_INPUT_INPUTXBAR31))
        ||
          ((input >= ECAP_INPUT_EPWM0_SOCA) && (input <= ECAP_INPUT_EPWM31_SOCB))
        ||
          ((input >= ECAP_INPUT_ADC0_EVT0) && (input <= ECAP_INPUT_ADC4_EVT3))
        ||
          ((input >= ECAP_INPUT_CMPSSA0_CTRIP_LOW) && (input <= ECAP_INPUT_CMPSSB9_CTRIP_HIGH)) )
        {
            bool status;

            DebugP_testLog("Selected input %d\r\n",input);
            /* configure ecap for the oneshot mode to capture a max of 4 events. */
            ecap_config_ptr->capture_mode_continuous = false;
            /* configure input and select input source for ecap */
            ecap_config_ptr->input = input;
            /* setting rearm configuration ecap */
            ecap_config_ptr->rearm = true;
            /* wrap capture poit will be set inside the input configuration based on the input */
            status = util_ecap_configure_input();
            if(status == BAD)
            {
                errors++;
            }
            /* configure the ecap with the settings updated */
            util_ecap_configure(instance);
            /* start the counter in the ecap and enable capture timestamps */
            util_ecap_start_timestamp(instance);
            /* activate input pulses */
            status = util_activate_input_pulses();
            if(status == BAD)
            {
                errors++;
            }
            /* check the ecap cap_registers, event flags to verify the input to expected output */
            errors += util_validate_input_events(instance, true, 50);
        }
    }

    if(errors > 0)
    {
        /* fail criteria*/
        return 1;
    }
    /* Pass criteria */
    return 0;
}


static HwiP_Object  gEcapHwiObject1;
volatile uint8_t      ecap_isr_count;
volatile uint32_t gbase = 0;
void ECAP_CAPTURE_MODE_ISR(void* args)
{
    ecap_isr_count++;
    ECAP_clearInterrupt(gbase, ECAP_ISR_SOURCE_ALL);
    ECAP_clearGlobalInterrupt(gbase);
}

/* tests Continuous or oneshot mode of the captures in ECAP and counter reset events */
int32_t AM263x_ECAP_BTR_005(uint16_t instance)
{
    DebugP_testLog("------------- instance : %d-------------\r\n", instance);

    int errors = 0;
    ECAP_CaptureMode capture_mode;

    util_gpio_configure(OUTPUT);
    SOC_xbarSelectInputXBarInputSource(inputxbar_base_addr, 0, 0, gpio_pin, 0);

    uint32_t reset_events[4][4]={
        {true, false, false, true},
        {true, true,  false, true},
        {true, false, true,  true},
        {true, true,  true,  true},
    };

    /* this array might need updation based on the GPIO toggling speeds */
    /* Observed data :
        a gpio rising edge to next rising edge (toggle period)
        is seen to be around 210 (in release build and 440 in debug build)
        ecap timestamp counts on prescaled value of 1
    */

    uint32_t gpio_toggle = 210;
#ifdef _DEBUG_
    gpio_toggle = 440;     // for debug mode
#endif
    uint32_t timestamps_array[4][4]={
        {0, 1*gpio_toggle, 2*gpio_toggle, 3*gpio_toggle},
        {0, 1*gpio_toggle, 1*gpio_toggle, 2*gpio_toggle},
        {0, 1*gpio_toggle, 2*gpio_toggle, 1*gpio_toggle},
        {0, 1*gpio_toggle, 1*gpio_toggle, 1*gpio_toggle},
    };

    gbase = ecap_base_addr[instance];
    DebugP_testLog("%x is the base\r\n",gbase);
    volatile uint32_t intxbar_input = 1<<(instance);
    SOC_xbarSelectInterruptXBarInputSource(CSL_CONTROLSS_INTXBAR_U_BASE, 0, 0, 0, 0, 0, 0, ( intxbar_input ), 0);

    for(capture_mode =  ECAP_CONTINUOUS_CAPTURE_MODE; capture_mode <= ECAP_ONE_SHOT_CAPTURE_MODE; capture_mode++)
    {
        for(int iter=0; iter<4; iter++)
        {
            ECAP_clearGlobalInterrupt(gbase);
            ecap_isr_count = 0;
            /* registering the ISR */
            int32_t status;
            HwiP_Params     hwiPrms;
            HwiP_Params_init(&hwiPrms);
            hwiPrms.intNum      = CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_0;
            hwiPrms.callback    = &ECAP_CAPTURE_MODE_ISR;
            hwiPrms.isPulse     = 0;
            status              = HwiP_construct(&gEcapHwiObject1, &hwiPrms);
            DebugP_assert(status == SystemP_SUCCESS);
            /*
            set the reset events to be (1,4) (1,2,4) (1,3,4) (1,2,3,4)
                generate 8 gpio toggles with no delay in between
                for reset on (1,4) -> the timestamps will be

            */
            util_input_params_init();
            util_ecap_config_reset();

            ecap_config_ptr->input = ECAP_INPUT_INPUTXBAR0;
            ecap_config_ptr->capture_mode_continuous = capture_mode;
            ecap_config_ptr->rearm = true;
            ecap_config_ptr->int_en = true;
            ecap_config_ptr->int_source = ( ECAP_ISR_SOURCE_CAPTURE_EVENT_1 | ECAP_ISR_SOURCE_CAPTURE_EVENT_2 | ECAP_ISR_SOURCE_CAPTURE_EVENT_3 | ECAP_ISR_SOURCE_CAPTURE_EVENT_4 );

            input_params->check_timestamps[iter] = true;
            input_params->check_timestamps[0] = false;
            for(int element = 0; element <= 4; element++)
            {
                ecap_config_ptr->counter_reset_capture[element] = reset_events[iter][element];
                input_params->timestamps[element] = timestamps_array[iter][element];
            }

            util_ecap_configure(instance);
            util_ecap_start_timestamp(instance);

            /*
            set ISR for 4 events and toggle GPIO 8 times.
            the ISR shall read 8 for continuous mode and only 4 for oneshot mode.
            ISR shall stop the ecap counter at 8th ocurrance.
            this confirms the continuous/oneshot mode.
            */

            int num_times = 8;
            util_gpio_toggle(num_times, 0);
            util_ecap_stop_timestamp(instance);

            HwiP_destruct(&gEcapHwiObject1);

            if(capture_mode == ECAP_CONTINUOUS_CAPTURE_MODE)
            {
                if(ecap_isr_count != num_times)
                {
                    errors++;
                    DebugP_logError("ISR count didn't match. Expected %d recieved %d\r\n", num_times, ecap_isr_count);
                }
            }
            else
            {
                if(ecap_isr_count != 4)
                {
                    errors++;
                    DebugP_logError("ISR count didn't match. Expected 4 recieved %d\r\n", ecap_isr_count);
                }
            }
            errors += util_validate_input_events(instance, false, 50);
        }

    }

    if(errors > 0)
    {
        /* fail criteria*/
        return 1;
    }
    /* Pass criteria */
    return 0;
}

/* tests Sync In Pulse */
int32_t AM263x_ECAP_BTR_006(void)
{
    int errors = 0;
    uint32_t pwm_period = 32000;
    uint32_t pwm_ON_value = (pwm_period)>>1;    /* 50% duty cycle */

    uint16_t input_duty_cycle = 5;             /* irrespective of sync PWM duty cycle */
    uint16_t input_period = (pwm_period)>>5;

    char     tester_cmd_start_input[CMD_SIZE];
    char     tester_cmd_stop_input[CMD_SIZE] = "halt pwm input";

    sprintf(tester_cmd_start_input, "generate pwm with duty cycle %02d period %05d on GPIO ", input_duty_cycle, input_period);
    /* test description :
        1. set up a Main PWM to sync all the PWMs
        2. set up a fixed phase shift between all the PWMs
        3. set all the 10 ECAPs with consecutive PWMs to sync their Counters.
            Absolute timestamps in continuous mode.
        4. All the ECAPs will be capturing a single input that is at least 4 times faster than the Main PWM
        5. the timestamps should differ from uniformly across the ECAPs.
    */
    /*
        10 ecaps to be testing 32 pwms syncouts.
        32 epwms -> 4* (set of 8 pwms)
        10 ecaps -> 8 to sync with 8 pwms. 2 redundant ones.
    */
    uint32_t pwm_phase_shifts[8];
    pwm_phase_shifts[0] = 0;
    for(int iter = 1; iter<8; iter++)
    {
        pwm_phase_shifts[iter] = pwm_period - (iter)*(pwm_period/8);
    }

    util_epwm_enable_all();

    util_gpio_pwm_input_configure();
    SOC_xbarSelectInputXBarInputSource(inputxbar_base_addr, 0, 0, pwm_input, 0);

    DebugP_testLog("pwm_input gpio is %d\r\n", pwm_input);
    /* enabling the sync pulse from EPWM0 to xbar to sync external input. */

    for(int iter = 0; iter < 4; iter++)
    {
        uint16_t ecap_instance;
        for(int pwm_instance = (iter*8); pwm_instance < ((iter+1)*8); pwm_instance++)
        {
            /* ecap instances 0 till 7 are configured here */
            ecap_instance = (pwm_instance%8);
            uint16_t sync_in_pwm = ECAP_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM0 + pwm_instance;

            uint16_t epwm_sync_instance = (iter*8);

            util_epwm_setup(epwm_sync_instance, pwm_ON_value, pwm_period);
            util_epwm_enable(epwm_sync_instance);
            util_xbar_configure_sync_dut(epwm_sync_instance);
            EPWM_setTimeBaseCounterMode(epwm_base_addr[epwm_sync_instance], EPWM_COUNTER_MODE_UP);

            util_epwm_setup(pwm_instance, pwm_ON_value, pwm_period);
            util_epwm_enable(pwm_instance);

            util_epwm_sync(pwm_instance, epwm_sync_instance, pwm_period - pwm_phase_shifts[(pwm_instance%8)]);

            DebugP_testLog("\tPWM Instance %d has phase shift value of %d syncs ecap %d \r\n", pwm_instance, pwm_phase_shifts[(pwm_instance%8)], ecap_instance);

            util_ecap_config_reset();

            ecap_config_ptr->load_counter_en = true;
            ecap_config_ptr->sync_in_source = sync_in_pwm;
            ecap_config_ptr->phase_shift_count = 0;
            ecap_config_ptr->input = ECAP_INPUT_INPUTXBAR0;

            ecap_config_ptr->polarity[0] = FALLING;
            ecap_config_ptr->polarity[1] = FALLING;
            ecap_config_ptr->polarity[2] = FALLING;
            ecap_config_ptr->polarity[3] = FALLING;

            util_ecap_configure(ecap_instance);
            ClockP_usleep(5);
        }
        /*
            the other two ecaps, i.e., 8,9 are configured here, with phase shift at ecap instance.
            these are statically, phase shifted with 100 and 200 with ecap 0 to validate the timestamp differences
        */
        ecap_config_ptr->sync_in_source = (iter*8) + ECAP_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM0;

        uint32_t ecap_phase_8 = pwm_period>>2;
        uint32_t ecap_phase_9 = pwm_period>>4;

        ecap_instance = 8;
        ecap_config_ptr->phase_shift_count = ecap_phase_8;

        util_ecap_configure(ecap_instance);
        DebugP_testLog("ecap %d is configured and is synced by pwm_instance %d \r\n",ecap_instance, iter*8);

        ecap_instance = 9;
        ecap_config_ptr->phase_shift_count = ecap_phase_9;
        util_ecap_configure(ecap_instance);

        DebugP_testLog("ecap %d is configured and is synced by pwm_instance %d \r\n",ecap_instance, iter*8);


        /* now, run the PWMs, start capturing the timestamps. */
        for(int pwm_instance = (iter*8); pwm_instance < ((iter+1)*8); pwm_instance++)
        {
            EPWM_setTimeBaseCounterMode(epwm_base_addr[pwm_instance], EPWM_COUNTER_MODE_UP);
        }

        /* start the captures for all the ecaps */
        for(int ecap_instance = 0; ecap_instance < 10; ecap_instance++)
        {
            util_ecap_start_timestamp(ecap_instance);
        }

        /* requesting the input */
        tester_command(tester_cmd_start_input);

        /* wait for some amount of time, stop all the ecap captures and EPWMs. */
        ClockP_usleep(5);

        uint32_t flags_set = ECAP_getInterruptSource(ecap_base_addr[0]);
        if(ECAP_getInterruptSource(ecap_base_addr[0]) > 0)
        {
            DebugP_testLog("flags set ! : %b\r\n", flags_set);
        }
        else
        {
            DebugP_logError("Error : no flags set  %b\r\n", flags_set);
        }
        util_ecap_stop_timestamp(ecap_instance);

        /* Halt the input PWM */
        tester_command(tester_cmd_stop_input);

        util_epwm_disable_all();

        for(int ecap_instance = 0; ecap_instance < 10; ecap_instance++)
        {
            ECAP_stopCounter(ecap_base_addr[ecap_instance]);
        }

        /*
            validate the timestamps of all the ecaps with respect to each other.
            Note :
            1. the consecutive ECAPs(0-7) timestamp[0] should differ by phase shift of the coresponding PWM phase shifts.
            2. the ECAPs(8,9) should have the timestamp[0] should differ by 100, 200 with respect to ECAP0
        */
        volatile uint32_t ecap_0_timestamp_0;
        volatile uint32_t ecap_8_timestamp_0;
        volatile uint32_t ecap_9_timestamp_0;


        ecap_0_timestamp_0 = ECAP_getEventTimeStamp(ecap_base_addr[0], ECAP_EVENT_1);
        ecap_8_timestamp_0 = ECAP_getEventTimeStamp(ecap_base_addr[8], ECAP_EVENT_1);
        ecap_9_timestamp_0 = ECAP_getEventTimeStamp(ecap_base_addr[9], ECAP_EVENT_1);

        for(int ecap_instance=1; ecap_instance<8; ecap_instance++)
        {
            volatile uint32_t ecap_iter_timestamp_0 = ECAP_getEventTimeStamp(ecap_base_addr[ecap_instance], ECAP_EVENT_1);

            /*
            the timestamp diff is,
                (ecap_0_timestamp + pwm_period - phase_diff) % pwm_period = ecap_iter_timestamp
            */
            uint32_t check_timestamp = (ecap_0_timestamp_0 + pwm_period - pwm_phase_shifts[ecap_instance]) % pwm_period;
            if(util_compare_in_range(check_timestamp, ecap_iter_timestamp_0, 50))
            {
                DebugP_testLog("PASS\r\n");
            }
            else
            {
                errors++;
                DebugP_logError("ERROR: timestamps didnt match.\tecap_0_timestamp_0 : %d\tecap_%d_timestamp_0 : %d\tphase_diff_expected : %d\r\n",
                            ecap_0_timestamp_0, ecap_instance, ecap_iter_timestamp_0, pwm_period - pwm_phase_shifts[ecap_instance]);
            }

        }
        /*
        input period is 4000, phase shift is 100 (or 200).
            so, the timestamp diff should be 4000 - 100 (or 200)
        */
            uint32_t check_timestamp_8 = (ecap_0_timestamp_0 + ecap_phase_8) % pwm_period;
            uint32_t check_timestamp_9 = (ecap_0_timestamp_0 + ecap_phase_9) % pwm_period;
            if( util_compare_in_range(check_timestamp_8, ecap_8_timestamp_0, 50) &&
                util_compare_in_range(check_timestamp_9, ecap_9_timestamp_0, 50))
            {
                DebugP_testLog("PASS\r\n");
            }
            else
            {
                DebugP_logError("ERROR: timestamps didnt match.\tecap_0_timestamp_0 : %d\tecap_8_timestamp_0 : %d\tphase_diff_expected : %d\r\n",
                                ecap_0_timestamp_0, ecap_8_timestamp_0, pwm_period -  ecap_phase_8);

                DebugP_logError("ERROR: timestamps didnt match.\tecap_0_timestamp_0 : %d\tecap_9_timestamp_0 : %d\tphase_diff_expected : %d\r\n",
                                ecap_0_timestamp_0, ecap_9_timestamp_0,pwm_period -  ecap_phase_9);
            }
    util_ecap_reset_all();
    util_epwm_disable_all();
    }

    if(errors > 0)
    {
        /* fail criteria*/
        return 1;
    }
    /* Pass criteria */
    return 0;
}


bool prescaler_tests(uint16_t instance, uint16_t input_duty_cycle, uint16_t input_period, ECAP_QualPeriodSelect qual_prd)
{
    bool status = GOOD;

    uint32_t base = ecap_base_addr[instance];
    int errors = 0;
    /* toggle x times and expect x/prescaler number of events and their timestamps should be stretched accordingly */

    uint32_t pwm_on = 0;
    uint32_t pwm_period = 0;

    char     tester_cmd_stop_input[CMD_SIZE] = "halt pwm input";
    char     tester_cmd_start_input[CMD_SIZE];

    sprintf(tester_cmd_start_input, "generate pwm with duty cycle %02d period %05d on GPIO ", input_duty_cycle, input_period);

    util_gpio_pwm_input_configure();
    SOC_xbarSelectInputXBarInputSource(inputxbar_base_addr, 0, 0, pwm_input, 0);
    
    util_input_params_init();
    input_params->check_timestamps[1] = true;
    input_params->check_timestamps[2] = true;
    input_params->timestamps[1] = 0;
    input_params->timestamps[2] = 0;

    util_ecap_config_reset();
    ecap_config_ptr->input = ECAP_INPUT_INPUTXBAR0;
    ecap_config_ptr->capture_mode_continuous = false;
    ecap_config_ptr->qual_prd = ECAP_PULSE_WIDTH_FILTER_BYPASS;

    ecap_config_ptr->polarity[0] = RISING;
    ecap_config_ptr->polarity[1] = FALLING;
    ecap_config_ptr->polarity[2] = RISING;
    ecap_config_ptr->polarity[3] = FALLING;

    ecap_config_ptr->rearm = true;


    ecap_config_ptr->counter_reset_capture[0] = true;
    ecap_config_ptr->counter_reset_capture[1] = false;
    ecap_config_ptr->counter_reset_capture[2] = false;
    ecap_config_ptr->counter_reset_capture[3] = false;
    
    int while_breaker_count = 5;
    do
    {
        tester_command(tester_cmd_stop_input);
        tester_command(tester_cmd_start_input);

        util_ecap_configure(instance);
        util_ecap_start_timestamp(instance);
        
        /* small delay */
        ClockP_usleep(5);

        util_ecap_stop_timestamp(instance);

        tester_command(tester_cmd_stop_input);
        
        pwm_on = ECAP_getEventTimeStamp(base, events[1]) + 1;
        pwm_period = ECAP_getEventTimeStamp(base, events[2]) + 1;

        if(while_breaker_count > 1)
        {
            while_breaker_count--;
        }
        else
        {
            break;
            DebugP_testLog("input reception fail. check input PWM\r\n");
            return BAD;
        }
    }while(pwm_on != (input_duty_cycle + 1));
    
    ecap_config_ptr->qual_prd = qual_prd;
    
    DebugP_testLog("\tpwm_on %d, pwm_period %d\r\n", pwm_on, pwm_period);

    for(int iter = 0; iter < 4; iter++)
    {
        input_params->timestamps[iter] = ECAP_getEventTimeStamp(base, events[iter]);
    }

    DebugP_testLog("pwm_on\t:\t%d\r\n",pwm_on);
    DebugP_testLog("pwm_period\t:\t%d\r\n",pwm_period);

    
    for(uint16_t prescale_value = 1; prescale_value < ECAP_MAX_PRESCALER_VALUE; prescale_value++)
    {
        DebugP_testLog("\r\nqual_period : %d\r\n", qual_prd);
        DebugP_testLog("\r\nprescale_value : %d\r\n", prescale_value);
        
        ecap_config_ptr->prescaler = prescale_value;
        
        input_params->timestamps[1] = (pwm_period * prescale_value) - 1 ;
        input_params->timestamps[2] = (pwm_period * 2 * prescale_value) - 1;
        
        DebugP_testLog("expected timestamps[1] = (pwm_period * prescale_value) - 1 : %d\r\n",input_params->timestamps[1]);
        DebugP_testLog("expected timestamps[2] = (pwm_period * 2 * prescale_value) - 1 : %d\r\n",input_params->timestamps[2]);


        tester_command(tester_cmd_start_input);

        util_ecap_configure(instance);
        util_ecap_start_timestamp(instance);
    
        /* small delay */
            ClockP_usleep(10);
        util_ecap_stop_timestamp(instance);
        
        tester_command(tester_cmd_stop_input);
        
        DebugP_testLog("\treturned timestamps\r\n\t%d\t%d\t%d\t%d\r\n",
                        ECAP_getEventTimeStamp(base, events[0]),
                        ECAP_getEventTimeStamp(base, events[1]),
                        ECAP_getEventTimeStamp(base, events[2]),
                        ECAP_getEventTimeStamp(base, events[3])
                        );
        errors += util_validate_input_events(instance, false, 2);
    }
    
    if (errors > 0)
    {
        status = BAD;
    }

    return status;
}

bool qual_period_tests(uint16_t instance)
{
    bool status = GOOD;
    uint16_t event_flags = 0;
    /* Negative tests */
    /* 
        Set qual_prd to a value and send in the pwm wave with pulses smaller than the qual_prd 
        if the pulses are detected then its a fail.
    */
    uint16_t input_duty_cycle = 0;
    uint16_t input_period = 100;        
    /* 100 -> 101 TBCLKs -> 101 sysclk or 200MHz/101 is the frequency
        each TBCLK is */
    
    char     tester_cmd_stop_input[CMD_SIZE] = "halt pwm input";
    char     tester_cmd_start_input[CMD_SIZE];


    util_gpio_pwm_input_configure();
    SOC_xbarSelectInputXBarInputSource(inputxbar_base_addr, 0, 0, pwm_input, 0);
    
    util_ecap_config_reset();
    ecap_config_ptr->rearm = true;
    ecap_config_ptr->capture_mode_continuous = false;
    ecap_config_ptr->input = ECAP_INPUT_INPUTXBAR0;
    
    ecap_config_ptr->polarity[0] = RISING;
    ecap_config_ptr->polarity[1] = RISING;
    ecap_config_ptr->polarity[2] = RISING;
    ecap_config_ptr->polarity[3] = RISING;

    for(uint16_t qual_period = ECAP_PULSE_WIDTH_FILTER_CYCLE1; qual_period <= ECAP_PULSE_WIDTH_FILTER_CYCLE15;  qual_period++ )
    {
        ecap_config_ptr->qual_prd = qual_period;
        /* setting a pwm_pulse just less than the qual_period*/

        input_duty_cycle = qual_period-1;

        DebugP_testLog("qual_prd : %d\tinput duty cycle : %d\r\n", qual_period, input_duty_cycle);

        sprintf(tester_cmd_start_input, "generate pwm with duty cycle %02d period %05d on GPIO ", input_duty_cycle, input_period);
        
        tester_command(tester_cmd_stop_input);
        tester_command(tester_cmd_start_input);
        
        util_ecap_configure(instance);
        util_ecap_start_timestamp(instance);
        /* small delay */
        ClockP_usleep(2);
        util_ecap_stop_timestamp(instance);
        event_flags = ECAP_getInterruptSource(ecap_base_addr[instance]);

        DebugP_testLog( 
                        "\t%x\t%x\t%x\t%x\r\n",
                        ECAP_getEventTimeStamp(ecap_base_addr[instance],events[0]),
                        ECAP_getEventTimeStamp(ecap_base_addr[instance],events[1]),
                        ECAP_getEventTimeStamp(ecap_base_addr[instance],events[2]),
                        ECAP_getEventTimeStamp(ecap_base_addr[instance],events[3])
                    );

        /* expecting there shouldn't be any event happening other than the counter overflow. 
            so let's check the event flags :)*/
        event_flags &= ~ECAP_ISR_SOURCE_COUNTER_OVERFLOW;
        tester_command(tester_cmd_stop_input);

        DebugP_testLog("%x\r\n", event_flags);
        if(event_flags > 0)
        {
            status = BAD;
            DebugP_testLog("tests failed at level 1, qual_period value : %d\tinput_duty_cycle : %d\r\n", qual_period, input_duty_cycle);
            break;
        }
    }
    if(status == BAD)
    {   
        return status;
    }
    /* negative tests with prescaler included.*/
    for(uint16_t qual_period = ECAP_PULSE_WIDTH_FILTER_CYCLE1; qual_period <= ECAP_PULSE_WIDTH_FILTER_CYCLE15;  qual_period++ )
    {
        
        if(qual_period < 1)
        {
            continue;
        }

        input_duty_cycle = qual_period - 1;
        /* same as above. But here, the tests run with the prescalers set combinations and are expected to fail */
        status = prescaler_tests(   
                                instance, 
                                input_duty_cycle, 
                                input_period, 
                                qual_period
                                );
        if(status == !BAD)
        {
            DebugP_logError("Negative Prescale Test With Glitch Filter Fail For Input-Duty-Cycle : %d \r\n",input_duty_cycle);
            return BAD;
        }
        else
        {
            status = GOOD;
        }
    }

    return status;
}

/* Input evaluation tests */
int32_t AM263x_ECAP_BTR_007(uint16_t instance)
{   
    /* the input evaluation tests 
        1. Prescaler tests
            - API void ECAP_setEventPrescaler(uint32_t base, uint16_t preScalerValue)
            - this API can take the inputs from 0 and till < ECAP_MAX_PRESCALER_VALUE 
            - input will be prescaled for such number of the values and will be taken for consideration 
                on the event polarity later.
        2. Glitch filter tests
            - filters out pulses with width less than given number of system cycles.
    */
    bool status;
    int errors = 0;
    uint16_t qual_prd = 0; 
    uint16_t input_period = 100;
    uint16_t input_duty_cycle = 50;

    status = prescaler_tests(   
                            instance, 
                            input_duty_cycle, 
                            input_period, 
                            qual_prd
                            );
    
    status = qual_period_tests(instance);

    if(status == BAD)
    {
        errors++;
    }
    if(errors > 0)
    {
        /* fail criteria*/
        return 1;
    }
    /* Pass criteria */
    return 0;
}

int32_t test_ecap_cases(uint8_t in)
{
    int32_t failcount = 0;
    uint16_t instance;
    bool single_instance_test = false;
    for (instance = 0; instance < NUM_ECAP_INSTANCES; instance++)
    {
        switch (in)
        {
        case 0:
        {
            uint32_t base = ecap_base_addr[instance];
            /* API checks are done here.*/
            uint16_t val = 31;

            /* Call the ECAP_setEventPrescaler API */
            ECAP_setEventPrescaler(base, val);

            /* Check if the value was written correctly */
            if((HW_RD_REG16(base + CSL_ECAP_ECCTL1) & CSL_ECAP_ECCTL1_PRESCALE_MASK) >> CSL_ECAP_ECCTL1_PRESCALE_SHIFT != val)
            {
                failcount++;
            }
            break;
        }
        case 1:
        {
            /* Interrupt Generation on ECAP*/
            failcount += AM263x_ECAP_BTR_001(instance);
            break;
        }
        case 2:
        {
            failcount += AM263x_ECAP_BTR_002(instance);
            break;
        }
        case 3:
        {
            failcount += AM263x_ECAP_BTR_003(instance);
            break;
        }
        case 4:
        {
            failcount += AM263x_ECAP_BTR_004(instance);
            break;
        }
        case 5:
        {
            failcount += AM263x_ECAP_BTR_005(instance);
            break;
        }
        case 6:
        {
            failcount += AM263x_ECAP_BTR_006();
            single_instance_test = true;
            break;
        }
        case 7:
        {
            failcount += AM263x_ECAP_BTR_007(instance);
            single_instance_test = true;
            break;
        }
        // case n:
        // {
        //     failcount += error returning
        // }
        }

        if((!TEST_ALL_INSTANCES) || single_instance_test)
        {
            break;
        }
    }

    DebugP_testLog("%d is the failcount FYI\r\n", failcount);

    if (failcount != 0)
    {
        return 1;
    }

    else
    {
        return 0;
    }
}