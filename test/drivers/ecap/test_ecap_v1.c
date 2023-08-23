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

volatile uint32_t epwm_pinmux_restore_values[NUM_EPWM_INSTANCES*2] = {0};

ECAP_Events events[4] =
{
    ECAP_EVENT_1,
    ECAP_EVENT_2,
    ECAP_EVENT_3,
    ECAP_EVENT_4,
};

int XBAR_ecap_output_selects[10]=
{
    OUTPUT_XBAR_ECAP0_OUT,
    OUTPUT_XBAR_ECAP1_OUT,
    OUTPUT_XBAR_ECAP2_OUT,
    OUTPUT_XBAR_ECAP3_OUT,
    OUTPUT_XBAR_ECAP4_OUT,
    OUTPUT_XBAR_ECAP5_OUT,
    OUTPUT_XBAR_ECAP6_OUT,
    OUTPUT_XBAR_ECAP7_OUT,
    OUTPUT_XBAR_ECAP8_OUT,
    OUTPUT_XBAR_ECAP9_OUT,
};

#define INPUT   (1)
#define OUTPUT  (0)
#define RISING  false
#define FALLING  true
#define GOOD     true
#define BAD      false
#define IOMUX_OFFSET (PIN_EPWM0_A-PIN_EPWM1_A)
#define IOMUX_EPWM_OUTPUT_OFFSET (PIN_EPWM0_A-PIN_EPWM0_B)

#if defined (SOC_AM263PX)
uint32_t epwm_pins[] = {
    PIN_EPWM0_A,
    PIN_EPWM0_B,
    PIN_EPWM1_A,
    PIN_EPWM1_B,
    PIN_EPWM2_A,
    PIN_EPWM2_B,
    PIN_EPWM3_A,
    PIN_EPWM3_B,
    PIN_EPWM4_A,
    PIN_EPWM4_B,
    PIN_EPWM5_A,
    PIN_EPWM5_B,
    PIN_EPWM6_A,
    PIN_EPWM6_B,
    PIN_EPWM7_A,
    PIN_EPWM7_B,
    PIN_EPWM8_A,
    PIN_EPWM8_B,
    PIN_EPWM9_A,
    PIN_EPWM9_B,
    PIN_EPWM10_A,
    PIN_EPWM10_B,
    PIN_EPWM11_A,
    PIN_EPWM11_B,
    PIN_EPWM12_A,
    PIN_EPWM12_B,
    PIN_EPWM13_A,
    PIN_EPWM13_B,
    PIN_EPWM14_A,
    PIN_EPWM14_B,
    PIN_EPWM15_A,
    PIN_EPWM15_B,
    PIN_UART1_RXD,
    PIN_UART1_TXD,
    PIN_MMC_CLK,
    PIN_MMC_CMD,
    PIN_MMC_DAT0,
    PIN_MMC_DAT1,
    PIN_MMC_DAT2,
    PIN_MMC_DAT3,
    PIN_MMC_SDWP,
    PIN_MMC_SDCD,
    PIN_PR0_MDIO0_MDIO,
    PIN_PR0_MDIO0_MDC,
    PIN_PR0_PRU0_GPIO5,
    PIN_PR0_PRU0_GPIO9,
    PIN_PR0_PRU0_GPIO10,
    PIN_PR0_PRU0_GPIO8,
    PIN_PR0_PRU0_GPIO6,
    PIN_PR0_PRU0_GPIO4,
    PIN_PR0_PRU0_GPIO0,
    PIN_PR0_PRU0_GPIO1,
    PIN_PR0_PRU0_GPIO2,
    PIN_PR0_PRU0_GPIO3,
    PIN_PR0_PRU0_GPIO16,
    PIN_PR0_PRU0_GPIO15,
    PIN_PR0_PRU0_GPIO11,
    PIN_PR0_PRU0_GPIO12,
    PIN_PR0_PRU0_GPIO13,
    PIN_PR0_PRU0_GPIO14,
    PIN_PR0_PRU1_GPIO5,
    PIN_PR0_PRU1_GPIO9,
    PIN_PR0_PRU1_GPIO10,
    PIN_PR0_PRU1_GPIO8,
};
#else
uint32_t epwm_pins[] = {
    PIN_EPWM0_A,
    PIN_EPWM0_B,
    PIN_EPWM1_A,
    PIN_EPWM1_B,
    PIN_EPWM2_A,
    PIN_EPWM2_B,
    PIN_EPWM3_A,
    PIN_EPWM3_B,
    PIN_EPWM4_A,
    PIN_EPWM4_B,
    PIN_EPWM5_A,
    PIN_EPWM5_B,
    PIN_EPWM6_A,
    PIN_EPWM6_B,
    PIN_EPWM7_A,
    PIN_EPWM7_B,
    PIN_EPWM8_A,
    PIN_EPWM8_B,
    PIN_EPWM9_A,
    PIN_EPWM9_B,
    PIN_EPWM10_A,
    PIN_EPWM10_B,
    PIN_EPWM11_A,
    PIN_EPWM11_B,
    PIN_EPWM12_A,
    PIN_EPWM12_B,
    PIN_EPWM13_A,
    PIN_EPWM13_B,
    PIN_EPWM14_A,
    PIN_EPWM14_B,
    PIN_EPWM15_A,
    PIN_EPWM15_B,
    PIN_UART1_RXD,
    PIN_UART1_TXD,
    PIN_MMC_CLK,
    PIN_MMC_CMD,
    PIN_MMC_DAT0,
    PIN_MMC_DAT1,
    PIN_MMC_DAT2,
    PIN_MMC_DAT3,
    PIN_MMC_SDWP,
    PIN_MMC_SDCD,
    PIN_PR0_MDIO_MDIO,
    PIN_PR0_MDIO_MDC,
    PIN_PR0_PRU0_GPIO5,
    PIN_PR0_PRU0_GPIO9,
    PIN_PR0_PRU0_GPIO10,
    PIN_PR0_PRU0_GPIO8,
    PIN_PR0_PRU0_GPIO6,
    PIN_PR0_PRU0_GPIO4,
    PIN_PR0_PRU0_GPIO0,
    PIN_PR0_PRU0_GPIO1,
    PIN_PR0_PRU0_GPIO2,
    PIN_PR0_PRU0_GPIO3,
    PIN_PR0_PRU0_GPIO16,
    PIN_PR0_PRU0_GPIO15,
    PIN_PR0_PRU0_GPIO11,
    PIN_PR0_PRU0_GPIO12,
    PIN_PR0_PRU0_GPIO13,
    PIN_PR0_PRU0_GPIO14,
    PIN_PR0_PRU1_GPIO5,
    PIN_PR0_PRU1_GPIO9,
    PIN_PR0_PRU1_GPIO10,
    PIN_PR0_PRU1_GPIO8,
};
#endif


uint32_t gpio_base_addr = CSL_GPIO0_U_BASE;

/* GPIO59 -> EPWM8_A (G3) */
volatile uint32_t gpio_pin = (PIN_EPWM8_A)>>2;
volatile uint32_t pwm_input = (PIN_EPWM13_A)>>2;


volatile uint32_t inputxbar_base_addr = CSL_CONTROLSS_INPUTXBAR_U_BASE;

typedef struct
{
    bool enable;
    uint16_t monitoring_type;
    uint32_t monitor_min;
    uint32_t monitor_max;
    bool sync_in_enable;
    bool force_copy_from_shadow;
    uint16_t shadow_load_mode;
    uint32_t monitor_min_shadow;
    uint32_t monitor_max_shadow;
    bool debug_mode_enable;
} signal_monitoring_uint;
typedef struct
{
    uint8_t emu_mode;
    bool ecap_mode_apwm;
    bool apwm_shadow_enable;
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
    uint32_t signal_monitor_global_trip;
    uint32_t signal_monitor_global_load_strobe;
    signal_monitoring_uint* munit1_ptr;
    signal_monitoring_uint* munit2_ptr;
} ECAP_config_t;

ECAP_config_t ECAP_config;
/* for used for configuration holding and initialising the ecap_config_ptr->munit1_ptr and ecap_config_ptr->munit2_ptr*/
signal_monitoring_uint munit1, munit2;

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
    /* shadow mode */
    ecap_config_ptr->apwm_shadow_enable = false;
    ecap_config_ptr->apwm_period = 0;
    ecap_config_ptr->apwm_compare = 0;
    ecap_config_ptr->apwm_polarity = ECAP_APWM_ACTIVE_HIGH;
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
    {
            ecap_config_ptr->polarity[iter] = PARAMS_RESET_VAL_POLARITY;
    }
    /* interrupt disable */
    ecap_config_ptr->int_en = false;
    /* No source*/
    ecap_config_ptr->int_source = 0x0;
    /* No event resets by default*/
    for (int iter = 0; iter < 4; iter++)
    {
            ecap_config_ptr->counter_reset_capture[iter] = false;
    }
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

    ecap_config_ptr->signal_monitor_global_load_strobe = ECAP_MUNIT_GLDSTRB_DISABLED;
    ecap_config_ptr->signal_monitor_global_trip = ECAP_MUNIT_TRIP_DISABLED;
}

void util_ecap_signal_monitoring_config_reset(void)
{
    /* assigning the munit1 struct to the signal monitoring unit pointer in ecap_config*/
    ecap_config_ptr->munit1_ptr = &munit1;
    ecap_config_ptr->munit2_ptr = &munit2;

    /* disabling by default */
    ecap_config_ptr->munit1_ptr->enable = false;
    ecap_config_ptr->munit2_ptr->enable = false;

    /* copying reset values for the following parameters */
    ecap_config_ptr->munit1_ptr->monitoring_type = ECAP_MUNIT_HIGH_PULSE_WIDTH;
    ecap_config_ptr->munit2_ptr->monitoring_type = ECAP_MUNIT_HIGH_PULSE_WIDTH;

    ecap_config_ptr->munit1_ptr->monitor_min = 0;
    ecap_config_ptr->munit2_ptr->monitor_min = 0;

    ecap_config_ptr->munit1_ptr->monitor_max = 0;
    ecap_config_ptr->munit2_ptr->monitor_max = 0;

    ecap_config_ptr->munit1_ptr->sync_in_enable = false;
    ecap_config_ptr->munit2_ptr->sync_in_enable = false;

    ecap_config_ptr->munit1_ptr->force_copy_from_shadow = false;
    ecap_config_ptr->munit2_ptr->force_copy_from_shadow = false;

    ecap_config_ptr->munit1_ptr->shadow_load_mode = ECAP_ACTIVE_LOAD_SYNC_EVT;
    ecap_config_ptr->munit2_ptr->shadow_load_mode = ECAP_ACTIVE_LOAD_SYNC_EVT;

    ecap_config_ptr->munit1_ptr->monitor_min_shadow = 0;
    ecap_config_ptr->munit2_ptr->monitor_min_shadow = 0;

    ecap_config_ptr->munit1_ptr->monitor_max_shadow = 0;
    ecap_config_ptr->munit2_ptr->monitor_max_shadow = 0;

    ecap_config_ptr->munit1_ptr->debug_mode_enable = false;
    ecap_config_ptr->munit2_ptr->debug_mode_enable = false;
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
static void ECAP_pwm_mode_features(void *args);
static void ECAP_input_evaluation_block(void *args);
static void ECAP_capture_mode_tests(void *args);
static void ECAP_add_additional_input_trigger_sources(void *args);
static void ECAP_support_interrupt_generation_on_either_of_4_events(void *args);
static void ECAP_selection_of_soc_trigger_source(void *args);
static void ECAP_selection_of_dma_trigger_source(void *args);
static void ECAP_signal_monitoring_tests(void *args);

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
void util_setup_capture_module(uint16_t instance, uint32_t input, bool continuous_mode);


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
void util_outputXbar_configure_loopback(uint16_t instance);

/* EDMA util funcitons */
void util_edma_configure(uint16_t ecap_instance);
void util_edma_deconfigure(void);

/* EPWM util functions */
void util_epwm_enable_all(void);
void util_epwm_enable(uint16_t epwm_instance);
void util_epwm_setup(uint16_t epwm_instance, uint16_t ON_value, uint16_t period);
void util_epwm_setup_pulse(uint16_t epwm_instance, uint16_t ON_value, uint16_t OFF_value, uint16_t period);
void util_epwm_enable_soc_trigger(uint16_t epwm_instance, EPWM_ADCStartOfConversionType epwm_soc_type);
void util_epwm_enable_global_strobe(uint16_t epwm_instance, uint16_t old_period, uint16_t new_period);
bool util_epwm_wait_on_soc_trigger(uint16_t epwm_instance, EPWM_ADCStartOfConversionType epwm_soc_type);
void util_epwm_reset(int8_t epwm_instance);
void util_epwm_sync(uint16_t epwm_instance, uint16_t main_pwm_instance, uint16_t phase_shift_value);
void util_epwm_disable_all(void);
void util_epmw_pinmux_configure(uint32_t epwm_instance);
void util_epmw_pinmux_restore(uint32_t epwm_instance);
void util_xbar_configure_sync_dut(uint32_t epwm_instance);

/* test implementing functions */
int32_t AM263x_ECAP_BTR_001(uint16_t instance);
int32_t AM263x_ECAP_BTR_002(uint16_t instance);
int32_t AM263x_ECAP_BTR_003(uint16_t instance);
int32_t AM263x_ECAP_BTR_004(uint16_t instance);
int32_t AM263x_ECAP_BTR_005(uint16_t instance);
int32_t AM263x_ECAP_BTR_006(void);
int32_t AM263x_ECAP_BTR_007(uint16_t instance);
int32_t AM263x_ECAP_BTR_008(uint16_t instance);
int32_t AM263x_ECAP_BTR_009(uint16_t instance);


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

#define TOTAL_TEST_CASES (10)

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
        {0, 3330,  ECAP_setEventPrescalerApiCheck,                          "ECAP_setEventPrescalerApiCheck"},
        {1, 9476,  ECAP_selection_of_dma_trigger_source,                    "ECAP_selection_of_dma_trigger_source"},
        {1, 7906,  ECAP_selection_of_soc_trigger_source,                    "ECAP_selection_of_soc_trigger_source"},
        {1, 3407,  ECAP_support_interrupt_generation_on_either_of_4_events, "ECAP_support_interrupt_generation_on_either_of_4_events"},
        {2, 3406,  ECAP_add_additional_input_trigger_sources,               "ECAP_add_additional_input_trigger_sources"},
        {2, 3403,  ECAP_input_evaluation_block,                             "ECAP_input_evaluation_block"},
        {3, 3401,  ECAP_selection_of_sync_in_source,                        "ECAP_selection_of_sync_in_source"},
        {4, 3404,  ECAP_capture_mode_tests,                                 "ECAP_capture_mode_tests"},
        {4, 9423,  ECAP_pwm_mode_features,                                  "ECAP_pwm_mode_features"},
        {5, 9422,  ECAP_signal_monitoring_tests,                            "ECAP_signal_monitoring_tests"},
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
    	ECAP_clearInterrupt(base, ECAP_ISR_SOURCE_ALL);
    }else
    {
        ECAP_disableInterrupt(base, ECAP_ISR_SOURCE_ALL);
        ECAP_clearInterrupt(base, ECAP_ISR_SOURCE_ALL);
    }

    if(ecap_config_ptr->ecap_mode_apwm)
    {
	    /* Sets eCAP in PWM mode. */
        ECAP_enableAPWMMode(base);
        if(ecap_config_ptr->apwm_shadow_enable)
        {
            /* Set eCAP APWM period shadow */
            ECAP_setAPWMShadowPeriod(base, ecap_config_ptr->apwm_period);
            /* Set eCAP APWM on or off time count */
            ECAP_setAPWMShadowCompare(base, ecap_config_ptr->apwm_compare);
        }
        else
        {
            /* Set eCAP APWM period */
            ECAP_setAPWMPeriod(base, ecap_config_ptr->apwm_period);
            /* Set eCAP APWM on or off time count */
            ECAP_setAPWMCompare(base, ecap_config_ptr->apwm_compare);
        }
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

    uint16_t syncout_source = (ecap_config_ptr->sync_out_mode);
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

    /* ------- signal monitoring init ------- */
    /* Selecting Trip input */
    ECAP_selectTripSignal(base, (ECAP_MunitTripInputSelect) ecap_config_ptr->signal_monitor_global_trip);
    /* Selecting the Global strobe load (signal to load Shadow to active regs in the monitoring units) */
    ECAP_selectGlobalLoadStrobe(base, (ECAP_MunitGlobalStrobeSelect) ecap_config_ptr->signal_monitor_global_load_strobe);


    /* ------- signal monitoring unit 1 ------- */

    if(ecap_config_ptr->munit1_ptr->enable)
    {
        ECAP_enableSignalMonitoringUnit(base, ECAP_MONITORING_UNIT_1);
        ECAP_selectMonitoringType(base, ECAP_MONITORING_UNIT_1, ecap_config_ptr->munit1_ptr->monitoring_type);
        ECAP_configureMinValue(base, ECAP_MONITORING_UNIT_1, ecap_config_ptr->munit1_ptr->monitor_min);
        ECAP_configureMaxValue(base, ECAP_MONITORING_UNIT_1, ecap_config_ptr->munit1_ptr->monitor_max);

        if(ecap_config_ptr->munit1_ptr->sync_in_enable)
        {
            ECAP_enableShadowMinMaxRegisters(base, ECAP_MONITORING_UNIT_1);
            ECAP_selectShadowLoadMode(base, ECAP_MONITORING_UNIT_1, ecap_config_ptr->munit1_ptr->shadow_load_mode);
            ECAP_configureShadowMinValue(base, ECAP_MONITORING_UNIT_1, ecap_config_ptr->munit1_ptr->monitor_min_shadow);
            ECAP_configureShadowMaxValue(base, ECAP_MONITORING_UNIT_1, ecap_config_ptr->munit1_ptr->monitor_max_shadow);
            if(ecap_config_ptr->munit1_ptr->force_copy_from_shadow)
            {
                ECAP_enableSoftwareSync(base, ECAP_MONITORING_UNIT_1);
            }

        }
        else
        {
            ECAP_disableShadowMinMaxRegisters(base, ECAP_MONITORING_UNIT_1);
        }

        if(ecap_config_ptr->munit1_ptr->debug_mode_enable)
        {
            ECAP_enableDebugRange(base, ECAP_MONITORING_UNIT_1);
        }
        else
        {
            ECAP_disableDebugRange(base, ECAP_MONITORING_UNIT_1);
        }
    }
    else
    {
        ECAP_disableSignalMonitoringUnit(base, ECAP_MONITORING_UNIT_1);
    }

    /* ------- signal monitoring unit 2 ------- */

    if(ecap_config_ptr->munit2_ptr->enable)
    {
        ECAP_enableSignalMonitoringUnit(base, ECAP_MONITORING_UNIT_2);
        ECAP_selectMonitoringType(base, ECAP_MONITORING_UNIT_2, ecap_config_ptr->munit2_ptr->monitoring_type);
        ECAP_configureMinValue(base, ECAP_MONITORING_UNIT_2, ecap_config_ptr->munit2_ptr->monitor_min);
        ECAP_configureMaxValue(base, ECAP_MONITORING_UNIT_2, ecap_config_ptr->munit2_ptr->monitor_max);

        if(ecap_config_ptr->munit2_ptr->sync_in_enable)
        {
            ECAP_enableShadowMinMaxRegisters(base, ECAP_MONITORING_UNIT_2);
            ECAP_selectShadowLoadMode(base, ECAP_MONITORING_UNIT_2, ecap_config_ptr->munit2_ptr->shadow_load_mode);
            ECAP_configureShadowMinValue(base, ECAP_MONITORING_UNIT_2, ecap_config_ptr->munit2_ptr->monitor_min_shadow);
            ECAP_configureShadowMaxValue(base, ECAP_MONITORING_UNIT_2, ecap_config_ptr->munit2_ptr->monitor_max_shadow);

            if(ecap_config_ptr->munit2_ptr->force_copy_from_shadow)
            {
                ECAP_enableSoftwareSync(base, ECAP_MONITORING_UNIT_2);
            }

        }
        else
        {
            ECAP_disableShadowMinMaxRegisters(base, ECAP_MONITORING_UNIT_2);
        }

        if(ecap_config_ptr->munit2_ptr->debug_mode_enable)
        {
            ECAP_enableDebugRange(base, ECAP_MONITORING_UNIT_2);
        }
        else
        {
            ECAP_disableDebugRange(base, ECAP_MONITORING_UNIT_2);
        }
    }
    else
    {
        ECAP_disableSignalMonitoringUnit(base, ECAP_MONITORING_UNIT_2);
    }

    return;
}

void util_ecap_start_timestamp(uint16_t instance)
{
    uint32_t base = ecap_base_addr[instance];
    ECAP_reArm(base);
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
    util_ecap_signal_monitoring_config_reset();
    util_ecap_configure(instance);
    return;
}
/* returns 1 if wait is successful i.e., interrupt flag set */
bool util_ecap_wait_for_interrupt(uint16_t instance)
{
    int32_t counter_max = 100;
    int32_t count = 0;
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


/* configures the XbarOut3 to be the ECAPx output and configures the pinMux and loopsback to inputXbar-0 via GPIO [no external LoopBack]*/
void util_outputXbar_configure_loopback(uint16_t instance)
{
    /* OUTPUT XBAR */
    SOC_xbarSelectOutputXBarInputSource(CSL_CONTROLSS_OUTPUTXBAR_U_BASE, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, ( XBAR_ecap_output_selects[instance] ), 0);
    SOC_xbarInvertOutputXBarOutputSignalBeforeLatch(CSL_CONTROLSS_OUTPUTXBAR_U_BASE, 0);
    SOC_xbarInvertOutputXBarOutputSignal(CSL_CONTROLSS_OUTPUTXBAR_U_BASE, 0);
    SOC_xbarSelectLatchOutputXBarOutputSignal(CSL_CONTROLSS_OUTPUTXBAR_U_BASE, 0);
    SOC_xbarSelectStretchedPulseOutputXBarOutputSignal(CSL_CONTROLSS_OUTPUTXBAR_U_BASE, 0);
    SOC_xbarSelectStretchedPulseLengthOutputXBarOutputSignal(CSL_CONTROLSS_OUTPUTXBAR_U_BASE, 0);

    Pinmux_PerCfg_t gPinMuxMainDomainCfg[] = {
            /* OUTPUTXBAR3 pin config */
    /* OUTPUTXBAR3 -> SPI1_D0 (B10) */
    {
        PIN_SPI1_D0,
        ( PIN_MODE(5) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },

    {PINMUX_END, PINMUX_END}
    };
    Pinmux_config(gPinMuxMainDomainCfg, PINMUX_DOMAIN_ID_MAIN);

    /* configuring the inputXbar0 to take in the GPIO 17 that reads the outputxbar 3*/
    GPIO_setDirMode(gpio_base_addr, (PIN_SPI1_D0 >> 2), OUTPUT);

    SOC_xbarSelectInputXBarInputSource(inputxbar_base_addr, 0, 0, (PIN_SPI1_D0 >> 2), 0);
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
    EPWM_disableGlobalLoad(epwm_base);
    EPWM_setClockPrescaler(epwm_base, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);
    EPWM_setPeriodLoadMode(epwm_base, EPWM_PERIOD_DIRECT_LOAD);
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

    // DebugP_testLog("\tPWM %d is set for Period : %d, ON_value : %d\r\n", epwm_instance, period, ON_value);
}

void util_epwm_setup_pulse(uint16_t epwm_instance, uint16_t ON_value, uint16_t OFF_value, uint16_t period)
{
    uint32_t epwm_base = epwm_base_addr[epwm_instance];
    EPWM_setTimeBaseCounter(epwm_base, 0);
    EPWM_setTimeBasePeriod(epwm_base, period);
    EPWM_setCounterCompareValue(epwm_base, EPWM_COUNTER_COMPARE_A, ON_value);
    EPWM_setCounterCompareValue(epwm_base, EPWM_COUNTER_COMPARE_B, OFF_value);

    EPWM_setActionQualifierAction(epwm_base, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(epwm_base, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(epwm_base, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);

    return;
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
void util_epwm_enable_global_strobe(uint16_t epwm_instance, uint16_t old_period, uint16_t new_period)
{
    uint32_t epwm_base = epwm_base_addr[epwm_instance];
    /* use the following to force the global load event.*/
    // EPWM_forceGlobalLoadOneShotEvent(epwm_base);
	EPWM_enableGlobalLoadRegisters(epwm_base, EPWM_GL_REGISTER_TBPRD_TBPRDHR);
	EPWM_setPeriodLoadMode(epwm_base, EPWM_PERIOD_SHADOW_LOAD);

    /* as shown in the configurations : */
    EPWM_enableGlobalLoad(epwm_base);
	EPWM_setGlobalLoadTrigger(epwm_base, EPWM_GL_LOAD_PULSE_GLOBAL_FORCE);
	EPWM_setGlobalLoadEventPrescale(epwm_base, 1);
    EPWM_disableGlobalLoadOneShotMode(epwm_base);

    EPWM_setTimeBasePeriod(epwm_base, old_period);  /* old_period to be loaded when global strobe occurs */
    EPWM_forceGlobalLoadOneShotEvent(epwm_base);

    EPWM_setTimeBasePeriod(epwm_base, new_period);  /* new_period to be loaded when global strobe occurs */

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

void util_epmw_pinmux_configure(uint32_t epwm_instance)
{
    HW_WR_REG32(CSL_IOMUX_U_BASE + CSL_IOMUX_IO_CFG_KICK0,0x83E70B13);
    HW_WR_REG32(CSL_IOMUX_U_BASE + CSL_IOMUX_IO_CFG_KICK1,0x95A4F1E0);

    uint32_t baseA = CSL_IOMUX_EPWM0_A_CFG_REG + 8*epwm_instance;
    uint32_t baseB = CSL_IOMUX_EPWM0_B_CFG_REG + 8*epwm_instance;

    epwm_pinmux_restore_values[2*epwm_instance] = HW_RD_REG32(CSL_IOMUX_U_BASE + baseA);
    epwm_pinmux_restore_values[2*epwm_instance + 1] = HW_RD_REG32(CSL_IOMUX_U_BASE + baseB);

    if(epwm_instance < 16)
    {
        HW_WR_REG32(CSL_IOMUX_U_BASE + baseA, 0xC500) ;
        HW_WR_REG32(CSL_IOMUX_U_BASE + baseB, 0xC500) ;
    }
    else
    {
        HW_WR_REG32(CSL_IOMUX_U_BASE + baseA, 0xC505) ;
        HW_WR_REG32(CSL_IOMUX_U_BASE + baseB, 0xC505) ;
    }
}

void util_epmw_pinmux_restore(uint32_t epwm_instance)
{
    if( (epwm_pinmux_restore_values[2*epwm_instance] != 0) &&
        (epwm_pinmux_restore_values[2*epwm_instance + 1] !=0))
    {
        uint32_t baseA = CSL_IOMUX_EPWM0_A_CFG_REG + 8*epwm_instance;
        uint32_t baseB = CSL_IOMUX_EPWM0_B_CFG_REG + 8*epwm_instance;

        HW_WR_REG32(CSL_IOMUX_U_BASE + CSL_IOMUX_IO_CFG_KICK0,0x83E70B13);
        HW_WR_REG32(CSL_IOMUX_U_BASE + CSL_IOMUX_IO_CFG_KICK1,0x95A4F1E0);

        HW_WR_REG32(CSL_IOMUX_U_BASE + baseA, epwm_pinmux_restore_values[2*epwm_instance]) ;
        HW_WR_REG32(CSL_IOMUX_U_BASE + baseB, epwm_pinmux_restore_values[2*epwm_instance + 1]) ;

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
static void ECAP_pwm_mode_features(void *args)
{
    DebugP_testLog("Test : PWM mode features of the ECAP\r\n");
    TEST_ASSERT_EQUAL_INT32(test_ecap_cases(8), 0);
}

static void ECAP_signal_monitoring_tests(void *args)
{
    DebugP_testLog("Test : Signal Monitoring features of the ECAP\r\n");
    TEST_ASSERT_EQUAL_INT32(test_ecap_cases(9), 0);
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

the Signal monitoring events as sources and tests are done in AM263x_ECAP_BTR_009.

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

    bool is_constant_voltage_set = false;
    char test_command[CMD_SIZE];
    sprintf(test_command, "provide analog voltage of %.4fV on ADC %d Channel %d", 1.5, 0, 0);
    /* expected to fail for the CMPSS instances that do not have Channel Outputs available on the EVM*/
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
            if ((is_constant_voltage_set != true) || ((input >= ECAP_INPUT_CMPSSA0_CTRIP_LOW) && (input <= ECAP_INPUT_CMPSSB9_CTRIP_HIGH)))
            {
                /* need a constant voltage on the CMPSS channels */
                tester_command(test_command);
                is_constant_voltage_set = true;
            }
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

bool active_shadow_tests(uint16_t instance)
{
    bool status = GOOD;
    uint32_t active_period = 1000;
    uint32_t active_cmp = 100;
    uint32_t shadow_period = 10000;
    uint32_t shadow_cmp = 500;

    /* shadow register tests*/
    /* Cap 1 : active APRD */
    /* Cap 2 : active ACMP */
    /* Cap 3 : shadow APRD */
    /* Cap 4 : shadow ACMP */

    util_outputXbar_configure_loopback(instance);


    util_ecap_config_reset();
    ecap_config_ptr->ecap_mode_apwm = true;
    ecap_config_ptr->apwm_period = 0;
    ecap_config_ptr->apwm_compare = 0;
    util_ecap_configure(instance);

    ECAP_setAPWMPeriod(ecap_base_addr[instance], active_period);
    if(active_period != ECAP_getEventTimeStamp(ecap_base_addr[instance], events[0]))
    {
        status = BAD;
        DebugP_logError("Active Period register load failed\r\n");
    }
    ECAP_setAPWMCompare(ecap_base_addr[instance], active_cmp);
    if(active_cmp != ECAP_getEventTimeStamp(ecap_base_addr[instance], events[1]))
    {
        status = BAD;
        DebugP_logError("Active Compare register load failed\r\n");
    }

    ECAP_setAPWMShadowPeriod(ecap_base_addr[instance], shadow_period);
    if(shadow_period != ECAP_getEventTimeStamp(ecap_base_addr[instance], events[2]))
    {
        status = BAD;
        DebugP_logError("Shadow Period register load fail!\r\n");
    }
    else
    {
        active_period = ECAP_getEventTimeStamp(ecap_base_addr[instance], events[0]);
        if(active_period == shadow_period)
        {   status = BAD;
            DebugP_logError("Active Period register loaded with shadow!\r\n");
        }
    }

    ECAP_setAPWMShadowCompare(ecap_base_addr[instance], shadow_cmp);
    if(shadow_cmp != ECAP_getEventTimeStamp(ecap_base_addr[instance], events[3]))
    {
        status = BAD;
        DebugP_logError("Shadow Compare register load fail!\r\n");
    }
    else
    {
        active_cmp = ECAP_getEventTimeStamp(ecap_base_addr[instance], events[1]);
        if(active_cmp == shadow_cmp)
        {   status = BAD;
            DebugP_logError("Active Compare register loaded with shadow!\r\n");
        }
    }

    /*run the counter, wait until the Period had occured */
    ECAP_startCounter(ecap_base_addr[instance]);

    volatile uint16_t flags = 0;
    do
    {
        flags = 0;
        flags = (ECAP_ISR_SOURCE_COUNTER_PERIOD & ECAP_getInterruptSource(ecap_base_addr[instance]));
    } while(flags != ECAP_ISR_SOURCE_COUNTER_PERIOD);

    DebugP_testLog("flags : %x\r\n",ECAP_ISR_SOURCE_COUNTER_PERIOD & ECAP_getInterruptSource(ecap_base_addr[instance]));
    /* stop the counter to make sure only one period occured*/
    ECAP_stopCounter(ecap_base_addr[instance]);


    /* check shadow to active load*/
    active_period = ECAP_getEventTimeStamp(ecap_base_addr[instance], events[0]);
    active_cmp = ECAP_getEventTimeStamp(ecap_base_addr[instance], events[1]);
    if((shadow_period != active_period) || (shadow_cmp != active_cmp))
    {
        status = BAD;
        DebugP_logError("Shadow to Active load failed at the Counter = Period event\r\n");
        DebugP_logInfo("active period : %d\r\n", active_period);
        DebugP_logInfo("shadow period : %d\r\n", shadow_period);
        DebugP_logInfo("active cmp : %d\r\n", active_cmp);
        DebugP_logInfo("shadow cmp : %d\r\n", shadow_cmp);
    }

    util_ecap_deconfigure_and_reset(instance);
    return status;
}

void util_setup_capture_module(uint16_t instance, uint32_t input, bool continuous_mode)
{
    util_ecap_config_reset();
    ecap_config_ptr->rearm = true;
    ecap_config_ptr->capture_mode_continuous = continuous_mode;
    ecap_config_ptr->input = input;

    ecap_config_ptr->polarity[0] = RISING;
    ecap_config_ptr->polarity[1] = RISING;
    ecap_config_ptr->polarity[2] = FALLING;
    ecap_config_ptr->polarity[3] = RISING;

    for(int iter = 0; iter < 4; iter++)
    {
        ecap_config_ptr->counter_reset_capture[iter] = true;
    }
    util_ecap_configure(instance);
    util_ecap_config_reset();

    return;
}

bool apwm_generation_test(uint16_t instance)
{
    bool status = GOOD;
    uint32_t period = 2000;     /* 2000*5ns : 10000nS or 10 uS*/
    uint32_t compare = 500;
    /*
    - setup the ECAP in the APWM mode
    - setup the OutputXbar to take the ECAP output
    - setup inputXbar to take in the InputXbar that loopsback the Xbar output via GPIO
    */
    util_outputXbar_configure_loopback(instance);

    /*
    - setup ecap 9 to take in the inputxbar 0 to validate the PWM output
    */

    ECAP_APWMPolarity test_runs[2] = {ECAP_APWM_ACTIVE_HIGH, ECAP_APWM_ACTIVE_LOW};
    for(int iter = 0; iter < 2; iter++)
    {
        util_setup_capture_module(9, ECAP_INPUT_INPUTXBAR0, false);
        util_ecap_config_reset();
        ecap_config_ptr->ecap_mode_apwm = true;
        ecap_config_ptr->apwm_period = period;
        ecap_config_ptr->apwm_compare = compare;
        ecap_config_ptr->apwm_polarity = test_runs[iter];

        util_ecap_configure(instance);

        /* start capturing on the ecap 9*/
        util_ecap_start_timestamp(9);

        /* start PWM on the given instance*/
        ECAP_startCounter(ecap_base_addr[instance]);

        /* some delay */
        ClockP_usleep(30);

        /*stop the timestamping*/
        util_ecap_stop_timestamp(9);

        /*stop the PWM*/
        ECAP_stopCounter(ecap_base_addr[instance]);

        /* timestamp 2 -> ON time */
        /* timestamp 3 -> OFF time */
        uint32_t on_time = ECAP_getEventTimeStamp(ecap_base_addr[9], events[2]);
        uint32_t off_time = ECAP_getEventTimeStamp(ecap_base_addr[9], events[3]);

        DebugP_testLog("\tpolarity : %x\ton_time : %d\toff_time : %d\tcompare : %d\r\n", test_runs[iter], on_time, off_time, compare);

        status = util_compare_in_range(on_time + off_time, period, 5);
        if(status == BAD)
        {
            break;
        }

        if(test_runs[iter] == ECAP_APWM_ACTIVE_HIGH)
        {
            status = util_compare_in_range(on_time, compare, 5);
        }
        else if(test_runs[iter] == ECAP_APWM_ACTIVE_LOW)
        {
            status = util_compare_in_range(off_time, compare, 5);
        }


        util_ecap_deconfigure_and_reset(instance);
    }

    /* deconfiguring and resetting instance 9 for future use.*/
    util_ecap_deconfigure_and_reset(9);

return status;
}

bool apwm_sync_in_sync_out_tests (uint16_t instance)
{
    /*
    setup an EPWM, say pwm_to_sync with long period=1000 that goes high at ctr=0 and low at cmp=10
    get a delta capture on RRFR edges. read the period (t1).
    note:
        this pwm doesn't take any syncin signal yet. but the phase shift will be 0 when there is a signal, which will be configured later.

    setup the above epwm to sync with ecap_instance_syncout which has smaller period, say 100, produces the syncout when its ctr=period.

    now repeat the above with tester, read the new period. the period should be smaller and note the period. the period should be smaller and equal to the ecap period. this confirms the sync event occured.
    */
    bool status = GOOD;
    uint16_t test_pwm_instance = 13; /* we have a configuration that sets the input */
    uint16_t pwm_monitor_instance = 9;
    /* arbitary on_time */
    uint16_t on_time = 10;
    uint16_t apwm_on_time = 10;

    /* large period */
    uint16_t period = 10000; /* 10000*5nS -> 50 uS*/
    uint16_t apwm_period = 100; /* considerably small pwm period */

    /* configuring the PWM input*/
    // util_gpio_pwm_input_configure();
    util_epmw_pinmux_configure(test_pwm_instance);

    /* configuring the InputXbar*/
    SOC_xbarSelectInputXBarInputSource(inputxbar_base_addr, 0, 0, pwm_input, 0);

    /* setup without sync pulse */
    util_epwm_enable(test_pwm_instance);
    util_epwm_setup(test_pwm_instance, on_time, period);

    /* configuring ECAP instance (9) to monitor the PWM input */
    util_setup_capture_module(pwm_monitor_instance, ECAP_INPUT_INPUTXBAR0, false);

    /* configure the ECAP module in APWM mode*/
    util_ecap_config_reset();
    ecap_config_ptr->ecap_mode_apwm = true;
    ecap_config_ptr->apwm_compare = 10;
    ecap_config_ptr->apwm_period = 100;
    ecap_config_ptr->sync_out_mode = ECAP_SYNC_OUT_DISABLED; /* dusabling the ECAP_SYNCOUT*/
    util_ecap_configure(instance);
    ECAP_startCounter(ecap_base_addr[instance]); /* Starting the ECAP counter to enable APWM*/

    /* setting the PWM to run */
    EPWM_setTimeBaseCounterMode(epwm_base_addr[test_pwm_instance], EPWM_COUNTER_MODE_UP);

    /* starting the pwm capture module*/
    util_ecap_start_timestamp(pwm_monitor_instance);

    /* wait for atleast a Period */
    ClockP_usleep(1000);

    util_ecap_stop_timestamp(pwm_monitor_instance);
    EPWM_setTimeBaseCounterMode(epwm_base_addr[test_pwm_instance], EPWM_COUNTER_MODE_STOP_FREEZE);
    ECAP_stopCounter(ecap_base_addr[instance]); /* Stoping the ECAP counter to enable APWM*/

    /* read the timestamps of the pwm_monitor_instance */
    uint32_t timestamp1 = 0;
    uint32_t timestamp2 = 0;

    /* uint32_t timestamp0 = ECAP_getEventTimeStamp(ecap_base_addr[pwm_monitor_instance], 0); // not usable */
    timestamp1 = ECAP_getEventTimeStamp(ecap_base_addr[pwm_monitor_instance], 1); // period
    timestamp2 = ECAP_getEventTimeStamp(ecap_base_addr[pwm_monitor_instance], 2); // on_time
    /* uint32_t timestamp3 = ECAP_getEventTimeStamp(ecap_base_addr[pwm_monitor_instance], 3); // off_time // not needed to be used */

    DebugP_testLog("on time : %d\ttimestamp2 : %d\tperiod : %d\ttimestamp1 : %d\t\r\n",on_time, timestamp2, period, timestamp1);

    /*sanity check*/
    if((!util_compare_in_range(timestamp1, period, 5))
        ||
       (!util_compare_in_range(timestamp2, on_time, 5)))
    {
        /* sanity failed test abort*/
        status = BAD;
        DebugP_logError("sanity failed\r\n");
    }
    else
    {
        /* enabling the sync_out_mode and setting to ECAP_SYNC_OUT_COUNTER_PRD sync.*/
        ECAP_setSyncOutMode(ecap_base_addr[instance], ECAP_SYNC_OUT_COUNTER_PRD);

        /* force EPWM TBCTR to 0 and set its sync in source to be the ECAPx SYNCO*/
        EPWM_setTimeBaseCounter(epwm_base_addr[test_pwm_instance], 0);
        EPWM_setSyncInPulseSource(epwm_base_addr[test_pwm_instance], EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_ECAP0 + instance);
        EPWM_enablePhaseShiftLoad(epwm_base_addr[test_pwm_instance]);
        EPWM_setPhaseShift(epwm_base_addr[test_pwm_instance], 0);

        /* configuring ECAP instance (9) to monitor the PWM input */
        util_setup_capture_module(pwm_monitor_instance, ECAP_INPUT_INPUTXBAR0, false);

        /* Running the ECAP APWM to generate the SyncOuts*/
        ECAP_startCounter(ecap_base_addr[instance]);

        /* Running the EPWM to recieve the SyncOuts*/
        EPWM_setTimeBaseCounterMode(epwm_base_addr[test_pwm_instance], EPWM_COUNTER_MODE_UP);

        /* Starting the Capture Module to generate timestamps*/
        util_ecap_start_timestamp(pwm_monitor_instance);

        /* wait for atleast a Period */
        ClockP_usleep(100);

        /* stop capture module, PWM and ECAP APWM*/
        util_ecap_stop_timestamp(pwm_monitor_instance);
        EPWM_setTimeBaseCounterMode(epwm_base_addr[test_pwm_instance], EPWM_COUNTER_MODE_STOP_FREEZE);
        ECAP_stopCounter(ecap_base_addr[instance]); /* Stoping the ECAP counter to enable APWM*/

        /* read the timestamps of the pwm_monitor_instance */
        timestamp1 = 0;
        timestamp2 = 0;

        /* uint32_t timestamp0 = ECAP_getEventTimeStamp(ecap_base_addr[pwm_monitor_instance], 0); // not usable */
        timestamp1 = ECAP_getEventTimeStamp(ecap_base_addr[pwm_monitor_instance], 1); // period
        timestamp2 = ECAP_getEventTimeStamp(ecap_base_addr[pwm_monitor_instance], 2); // on_time
        /* uint32_t timestamp3 = ECAP_getEventTimeStamp(ecap_base_addr[pwm_monitor_instance], 3); // off_time // not needed to be used */

        DebugP_testLog("on time : %d\ttimestamp2 : %d\tperiod : %d\ttimestamp1 : %d\t\r\n",on_time, timestamp2, period, timestamp1);

        if((!util_compare_in_range(timestamp1, apwm_period, 5))
            ||
           (!util_compare_in_range(timestamp2, apwm_on_time, 5)))
        {
            /* sync failed test abort*/
            status = BAD;
            DebugP_logError("sync failed\r\n");
        }
    }
    util_ecap_deconfigure_and_reset(pwm_monitor_instance);
    util_ecap_deconfigure_and_reset(instance);
    util_epwm_reset(test_pwm_instance);
    util_epwm_disable_all();
    util_epmw_pinmux_restore(test_pwm_instance);


    return status;
}

int32_t AM263x_ECAP_BTR_008(uint16_t instance)
{
    /* Test : APWM features of ECAP*/
    /* Features :
        - Active and shadow loads to period and compare
        - Emulation mode                 (covered in other tests)
        - Interrupt, ADCSOC, DMA trigger (covered in other test cases)
        - Phase shift on Sync IN         (covered in other test cases)
        - PWM generation                 (Period and Compare)
        - Sync Out
        */
    int errors = 0;
    bool status = GOOD;

    status = active_shadow_tests(instance);
    if(status == BAD)
    {
        DebugP_logError("active shadow load tests failed. Returning\r\n");
        errors++;
    }
    status = apwm_generation_test(instance);
    if(status == BAD)
    {
        DebugP_logError("apwm generation tests failed. Returning\r\n");
        errors++;
    }
    status = apwm_sync_in_sync_out_tests(instance);
    if(status == BAD)
    {
        DebugP_logError("apwm syncin syncout tests failed. Returning\r\n");
        errors++;
    }

    if(errors > 0)
    {
        return 1;
    }
    return 0;
}

static inline bool util_ecap_signal_monitoring_error_check(uint16_t wait_status, uint16_t monitor_window_min, uint16_t monitor_window_max, uint16_t check_value, uint16_t interrupt_flags)
{
    bool status = GOOD;
    if((monitor_window_min > check_value) || (check_value > monitor_window_max))
    {
        /* interrupt should be there and wait should have passed. */
        if(wait_status != GOOD)
        {
            DebugP_logError("pulse width not in window, but interrupt didn't occur.\r\n");
            status = BAD;
        }
        else
        {
            if( !(
                ((check_value < monitor_window_max) && (interrupt_flags & (ECAP_ISR_SOURCE_MUNIT_1_ERROR_EVT1 | ECAP_ISR_SOURCE_MUNIT_2_ERROR_EVT1)))
                ||
                ((check_value > monitor_window_max) && (interrupt_flags & (ECAP_ISR_SOURCE_MUNIT_1_ERROR_EVT2 | ECAP_ISR_SOURCE_MUNIT_2_ERROR_EVT2)))
                ))
            {
                /* min-max window violation happened. but the violation didn't flag appropriate flags */
                DebugP_logError("min-max window violation happened. but the violation didn't flag appropriate flags\r\n");
                status = BAD;
            }
        }
    }
    else
    {
        /* interrupt should not have happened and wait should have failed*/
        if(wait_status != BAD)
        {
            DebugP_logError("min-max window violation didn't happened. but flags are set\r\n");
            status = BAD;
        }
    }
    if(status == BAD)
    {
        DebugP_logError("mode : %d\t check_value : %d\t min_window : %d\t max_window : %d\t flags_set : 0x%x\r\n",
                        munit1.monitoring_type, check_value, monitor_window_min, monitor_window_max, interrupt_flags);
    }
    return status;
}

/*
mode :
    //! High Pulse Width
    ECAP_MUNIT_HIGH_PULSE_WIDTH = 0U,
    //! Low Pulse Width
    ECAP_MUNIT_LOW_PULSE_WIDTH = 1U,
    //! Period width from rise to rise
    ECAP_MUNIT_PERIOD_WIDTH_RISE_RISE = 2U,
    //! Period width from fall to fall
    ECAP_MUNIT_PERIOD_WIDTH_FALL_FALL = 3U,
    //! Monitor rise edge
    ECAP_MUNIT_MONITOR_RISE_EDGE = 4U,
    //! Monitor fall edge
    ECAP_MUNIT_MONITOR_FALL_EDGE = 5U,
*/
bool signal_monitor_mode_test(uint16_t instance, uint16_t mode)
{
    bool status = GOOD;

    /*
    configure the input waveform period = 32bit high value. duty cycle varies from 0-prd
    though the min-max registers are 32 bit long,
    */
    uint16_t test_pwm_instance = 13; /* we have a configuration that sets the input */
    uint16_t period = 0xFFFF;
    uint16_t on_value = 1000;

    util_epmw_pinmux_configure(test_pwm_instance);
    SOC_xbarSelectInputXBarInputSource(inputxbar_base_addr, 0, 0, pwm_input, 0);

    util_epwm_setup(test_pwm_instance, on_value, period);    // these on-time and period will be overwritten in the pulse setup
    util_epwm_enable(test_pwm_instance);

    uint16_t off_value = on_value + 3; /* to have a pulse that is atleast 3 sysclks wide */
    volatile uint16_t monitor_window_min, monitor_window_max;

    uint16_t min_window_step = 0x1000;
    uint16_t max_window_step = 0x1000;
    uint16_t period_step = 0x100;
    uint16_t on_value_step = 0x100;
    uint16_t off_value_step = 0x100;

    /* made for overnight looooooooong tests. expecting around 6-7 h of tests. */
    // uint16_t min_window_step = 0x100;
    // uint16_t max_window_step = 0x100;
    // uint16_t period_step = 0x10;
    // uint16_t on_value_step = 0x10;
    // uint16_t off_value_step = 0x10;

    uint16_t interrupt_sources =
                    ECAP_ISR_SOURCE_MUNIT_1_ERROR_EVT1 |
                    ECAP_ISR_SOURCE_MUNIT_1_ERROR_EVT2 |
                    ECAP_ISR_SOURCE_MUNIT_2_ERROR_EVT1 |
                    ECAP_ISR_SOURCE_MUNIT_2_ERROR_EVT2;
    volatile uint16_t interrupt_flags;


    ecap_config_ptr->input = ECAP_INPUT_INPUTXBAR0;
    ecap_config_ptr->capture_mode_continuous = ECAP_CONTINUOUS_CAPTURE_MODE;

    ecap_config_ptr->int_en = true;
    ecap_config_ptr->int_source = interrupt_sources;
    ecap_config_ptr->rearm = true;

    util_ecap_signal_monitoring_config_reset();
    ecap_config_ptr->munit1_ptr = &munit1;
    ecap_config_ptr->munit2_ptr = &munit2;
    munit1.enable = true;
    munit2.enable = true;
    munit1.monitoring_type = mode;
    munit2.monitoring_type = mode;

    for(monitor_window_min = 0; (monitor_window_min < (0xFFFF - min_window_step))&&(status); monitor_window_min += min_window_step)
// /* debug */    for(monitor_window_min = 0; (monitor_window_min < (0xFFFF - min_window_step))&&(1); monitor_window_min += min_window_step)
    {
// /* debug */        monitor_window_min = 53240; //  mode : 0        check_value : 1795      min_window : 53248      max_window : 53251      flags_set : 0x2
        for(monitor_window_max = monitor_window_min+3; (monitor_window_max < (0xFFFF - max_window_step))&&(status); monitor_window_max += max_window_step)
// /* debug */        for(monitor_window_max = monitor_window_min+3; (monitor_window_max < (0xFFFF - max_window_step))&&(1); monitor_window_max += max_window_step)
        {
            /* These loops sweep the entire min-max window range with steps */
// /* debug */            monitor_window_max = 53251;
            munit1.monitor_min = monitor_window_min;
            munit1.monitor_max = monitor_window_max;
            munit2.monitor_min = monitor_window_min;
            munit2.monitor_max = monitor_window_max;

            if((mode == ECAP_MUNIT_HIGH_PULSE_WIDTH) || (mode == ECAP_MUNIT_LOW_PULSE_WIDTH))
            {
                period = 0xFFFF;    //-> one period is 65535*sysclk or
                uint32_t sleep_ns = period*5;
// /* debug */                off_value = 65000;
                on_value = 0;

                /* in Pulse width monitoring minimum 2 captures, with the FR or RF should be configured. */
                if(mode == ECAP_MUNIT_HIGH_PULSE_WIDTH)
                {
                    ecap_config_ptr->polarity[0] = RISING;
                    ecap_config_ptr->polarity[1] = FALLING;
                }
                else
                {
                    ecap_config_ptr->polarity[0] = FALLING;
                    ecap_config_ptr->polarity[1] = RISING;
                }
                ecap_config_ptr->wrap_capture = events[1];
                ecap_config_ptr->sync_in_source = ECAP_SYNC_IN_PULSE_SRC_DISABLE;

                for(off_value = 3; (off_value < (period - off_value_step)) && (status); off_value += off_value_step)
// /* debug */                for(on_value = 3000; (on_value >= 0) && (1); on_value -= 1)
                {
                    volatile bool wait_status = GOOD;
                    uint32_t ON_time = off_value;
                    uint32_t OFF_time = period - off_value;
                    uint32_t check_value;

                    if(mode == ECAP_MUNIT_HIGH_PULSE_WIDTH)
                    {
                        check_value = ON_time;
                        check_value = off_value - on_value;
                    }
                    else
                    {
                        check_value = OFF_time;
                    }

                    /* stopping the PWM counter to not generate any waveform*/
                    EPWM_setTimeBaseCounterMode(epwm_base_addr[test_pwm_instance], EPWM_COUNTER_MODE_STOP_FREEZE);
                    /* setting the pulse in the PWM */
                    util_epwm_setup_pulse(test_pwm_instance, on_value, off_value, period);
                    /* running the input */
                    EPWM_setTimeBaseCounterMode(epwm_base_addr[test_pwm_instance], EPWM_COUNTER_MODE_UP);
                    ClockP_usleep((sleep_ns)/1000);
                    /* setting up the ECAP configurations */
                    util_ecap_configure(instance);
                    /* starting the ecap capture */
                    util_ecap_start_timestamp(instance);

// /* debug */                    for (uint16_t compb = on_value+3; compb <= period; compb+=2)
// /* debug */                    {
// /* debug */                        ECAP_clearGlobalInterrupt(ecap_base_addr[instance]);
// /* debug */                        ECAP_clearInterrupt(ecap_base_addr[instance], ECAP_ISR_SOURCE_ALL);
// /* debug */                        EPWM_setCounterCompareValue(epwm_base_addr[test_pwm_instance], EPWM_COUNTER_COMPARE_B, compb);
// /* debug */                        ECAP_enableSignalMonitoringUnit(ecap_base_addr[instance], ECAP_MONITORING_UNIT_1);
// /* debug */                        ECAP_enableSignalMonitoringUnit(ecap_base_addr[instance], ECAP_MONITORING_UNIT_2);
// /* debug */                        ECAP_enableTimeStampCapture(ecap_base_addr[instance]);
// /* debug */                        // ClockP_usleep((sleep_ns*10)/1000);
// /* debug */                        wait_status = util_ecap_wait_for_interrupt(instance);
// /* debug */                        (void) wait_status;
// /* debug */                        interrupt_flags = ECAP_getInterruptSource(ecap_base_addr[instance]);
// /* debug */                        check_value = compb - on_value;
// /* debug */                        DebugP_logInfo("mode : %d\t check_value : %d\t min_window : %d\t max_window : %d\t flags_set : 0x%x\r\n",
// /* debug */                        mode, check_value, monitor_window_min, monitor_window_max, interrupt_flags);
// /* debug */                        // status = util_ecap_signal_monitoring_error_check(wait_status, monitor_window_min, monitor_window_max, check_value, interrupt_flags);
// /* debug */                    }
                    ClockP_usleep((sleep_ns*2)/1000);
                    wait_status = util_ecap_wait_for_interrupt(instance);
                    interrupt_flags = ECAP_getInterruptSource(ecap_base_addr[instance]);
                    status = util_ecap_signal_monitoring_error_check(wait_status, monitor_window_min, monitor_window_max, check_value, interrupt_flags);
                    DebugP_logInfo("mode : %d\t check_value : %d\t min_window : %d\t max_window : %d\t flags_set : 0x%x\r\n",
                        mode, check_value, monitor_window_min, monitor_window_max, interrupt_flags);
                    /* stopping capture and pwm*/
                    util_ecap_stop_timestamp(instance);
                    EPWM_setTimeBaseCounterMode(epwm_base_addr[test_pwm_instance], EPWM_COUNTER_MODE_STOP_FREEZE);


                }
            }

            if((mode == ECAP_MUNIT_PERIOD_WIDTH_RISE_RISE) || (mode == ECAP_MUNIT_PERIOD_WIDTH_FALL_FALL))
            {
                /* here Period is swept from 0 to 0xFFFF,*/
                /* the On_value is kept at 0, and Off_value is set at period/2*/
                /* hence a RR period or FF period can be monitored */
                on_value = 0;

                if(mode == ECAP_MUNIT_PERIOD_WIDTH_RISE_RISE)
                {
                    ecap_config_ptr->polarity[0] = RISING;
                    ecap_config_ptr->polarity[1] = RISING;
                }
                else
                {
                    ecap_config_ptr->polarity[0] = FALLING;
                    ecap_config_ptr->polarity[1] = FALLING;
                }
                ecap_config_ptr->wrap_capture = events[1];
                ecap_config_ptr->sync_in_source = ECAP_SYNC_IN_PULSE_SRC_DISABLE;

                for(period = 0xF; (period < (0xFFFF - period_step)) && status; period += period_step)
                {
                    uint32_t check_value;
                    off_value = period >> 1;
                    uint32_t sleep_ns = period*5;
                    volatile bool wait_status = GOOD;
                    check_value = period;
                    /* stopping the PWM counter to not generate any waveform*/
                    EPWM_setTimeBaseCounterMode(epwm_base_addr[test_pwm_instance], EPWM_COUNTER_MODE_STOP_FREEZE);
                    /* setting the pulse in the PWM */
                    util_epwm_setup_pulse(test_pwm_instance, on_value, off_value, period);
                    /* setting up the ECAP configurations */
                    util_ecap_configure(instance);
                    /* running the input */
                    EPWM_setTimeBaseCounterMode(epwm_base_addr[test_pwm_instance], EPWM_COUNTER_MODE_UP);
                    /* starting the ecap capture */
                    util_ecap_start_timestamp(instance);
                    /* sleep at least period */
                    ClockP_usleep((sleep_ns*2)/1000);

                    wait_status = util_ecap_wait_for_interrupt(instance);
                    interrupt_flags = ECAP_getInterruptSource(ecap_base_addr[instance]);
                    /* stopping capture and pwm*/
                    util_ecap_stop_timestamp(instance);
                    EPWM_setTimeBaseCounterMode(epwm_base_addr[test_pwm_instance], EPWM_COUNTER_MODE_STOP_FREEZE);


                    DebugP_logInfo("mode : %d\t check_value : %d\t min_window : %d\t max_window : %d\t flags_set : 0x%x\r\n",
                        mode, check_value, monitor_window_min, monitor_window_max, interrupt_flags);

                    status = util_ecap_signal_monitoring_error_check(wait_status, monitor_window_min, monitor_window_max, check_value, interrupt_flags);
                }
            }

            if((mode == ECAP_MUNIT_MONITOR_RISE_EDGE) || (mode == ECAP_MUNIT_MONITOR_FALL_EDGE))
            {
                /* the the pulse width stays constant = 3 tbclks*/
                /* period stays constant */
                /* sync in enabled with same EPWM */
                /* only one capture, with required cap polarity. */
                if(mode == ECAP_MUNIT_MONITOR_RISE_EDGE)
                {
                    ecap_config_ptr->polarity[0] = RISING;
                }
                else
                {
                    ecap_config_ptr->polarity[0] = FALLING;
                }
                ecap_config_ptr->wrap_capture = events[0];
                ecap_config_ptr->load_counter_en = true;
                ecap_config_ptr->sync_in_source = ECAP_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM0 + test_pwm_instance;

                /* move the pulse of constant width [3 tbclks] around the min_max window*/
                period = 0xFFFF;

                ecap_config_ptr->phase_shift_count = 0;
                // for(on_value = 0; (on_value < (period - 3 - on_value_step))&&status; on_value += on_value_step)
                /*
                why not go till period ? and edge extend beyond max-window-value?
                    - because the edge monitoring doesn't support max window violation error (error2).
                    - this flags if there is no edge within the given min-max window and doesn't care for further monitor until a sync-in-pulse-from-pwm comes
                */
                for(on_value = 0; (on_value < monitor_window_max-3)&&(status); on_value += on_value_step)
                {
                    uint32_t check_value;
                    off_value = on_value+3;
                    uint32_t sleep_ns = 5*(period);
                    volatile bool wait_status = GOOD;

                    if(mode == ECAP_MUNIT_MONITOR_RISE_EDGE)
                    {
                        check_value = on_value;
                    }
                    else
                    {
                        check_value = off_value;
                    }

                    /* stopping the PWM counter to not generate any waveform*/
                    EPWM_setTimeBaseCounterMode(epwm_base_addr[test_pwm_instance], EPWM_COUNTER_MODE_STOP_FREEZE);
                    /* setting the pulse in the PWM */
                    util_epwm_setup_pulse(test_pwm_instance, on_value, off_value, period);
                    /* setting up the ECAP configurations */
                    util_ecap_configure(instance);

                    EPWM_forceSyncPulse(epwm_base_addr[test_pwm_instance]);

                    /* running the input */
                    EPWM_setTimeBaseCounterMode(epwm_base_addr[test_pwm_instance], EPWM_COUNTER_MODE_UP);
                    /* starting the ecap capture */
                    util_ecap_start_timestamp(instance);
                    /* sleep at least period */
                    ClockP_usleep((sleep_ns*2)/1000);
                    wait_status = util_ecap_wait_for_interrupt(instance);
                    interrupt_flags = ECAP_getInterruptSource(ecap_base_addr[instance]);
                    /* stopping capture and pwm*/
                    util_ecap_stop_timestamp(instance);
                    EPWM_setTimeBaseCounterMode(epwm_base_addr[test_pwm_instance], EPWM_COUNTER_MODE_STOP_FREEZE);


                    DebugP_logInfo("mode : %d\t check_value : %d\t min_window : %d\t max_window : %d\t flags_set : 0x%x\r\n",
                        mode, check_value, monitor_window_min, monitor_window_max, interrupt_flags);

                    status = util_ecap_signal_monitoring_error_check(wait_status, monitor_window_min, monitor_window_max, check_value, interrupt_flags);
                }
            }
        }
    }

    /* setup Capture configurations for the monitor_options */

    /* loop on the input waveform variations based on the mode */
    /* if mode is edge monitorings, then the input is supposed to be the internal singals / epwm */
    /* expect int flags only when violations otherwise error*/

    util_epmw_pinmux_restore(test_pwm_instance);

    return status;
}

bool global_strobe_load_tests(uint16_t instance)
{
    bool status = GOOD;
    uint16_t errors = 0;
    uint32_t shadow_value = 0xFEDCABCD;
    volatile CSL_ecapRegs *ecap_regs = (CSL_ecapRegs *) ecap_base_addr[instance];

    /* setting up another ecap module to capture the pulses from the input pwms to analyse
    and read the period changes so that we can confirm the global load strobe occurance */
    uint16_t ecap_capture_instance = 9;
    uint32_t inputxbar_instance = ECAP_INPUT_INPUTXBAR1;
    util_ecap_deconfigure_and_reset(ecap_capture_instance);
    util_setup_capture_module(ecap_capture_instance, inputxbar_instance, false);
    util_ecap_start_timestamp(ecap_capture_instance);

    /* setup ecap and munits*/
    util_ecap_config_reset();
    ecap_config_ptr->input = ECAP_INPUT_INPUTXBAR0;             //any input.
    ecap_config_ptr->capture_mode_continuous = ECAP_CONTINUOUS_CAPTURE_MODE;
    ecap_config_ptr->rearm = true;
    ecap_config_ptr->sync_in_source = ECAP_SYNC_IN_PULSE_SRC_DISABLE;
    util_ecap_signal_monitoring_config_reset();
    ecap_config_ptr->munit1_ptr = &munit1;
    ecap_config_ptr->munit2_ptr = &munit2;

    munit1.enable = true;
    munit1.monitoring_type = ECAP_MUNIT_HIGH_PULSE_WIDTH;
    munit1.sync_in_enable = true;               /* enables shadow mode and actions */
    munit1.monitor_min_shadow = shadow_value;
    munit1.monitor_max_shadow = shadow_value;

    munit2.enable = true;
    munit2.monitoring_type = ECAP_MUNIT_HIGH_PULSE_WIDTH;
    munit2.sync_in_enable = true;               /* enables shadow mode and actions */
    munit2.monitor_min_shadow = shadow_value;
    munit2.monitor_max_shadow = shadow_value;

    ECAP_MunitGlobalStrobeSelect global_load_strobe = ECAP_MUNIT_GLDSTRB_DISABLED;

    ecap_config_ptr->signal_monitor_global_load_strobe = global_load_strobe;
    util_ecap_configure(instance);

     /* shadow load mode is selected as sync in by default. no sync in so no active == shadow*/
    if(
        ((ecap_regs->MUNIT_1_MIN) == (ecap_regs->MUNIT_1_MIN_SHADOW)) ||
        ((ecap_regs->MUNIT_2_MIN) == (ecap_regs->MUNIT_2_MIN_SHADOW)) ||
        ((ecap_regs->MUNIT_1_MAX) == (ecap_regs->MUNIT_1_MAX_SHADOW)) ||
        ((ecap_regs->MUNIT_2_MAX) == (ecap_regs->MUNIT_2_MAX_SHADOW))
    )
    {
        errors++;
        DebugP_logError("shadow to active load happened without the global load strobe\r\n");
    }
    ecap_regs->MUNIT_1_MIN_SHADOW = (ecap_regs->MUNIT_1_MIN_SHADOW) >> 1;
    ecap_regs->MUNIT_2_MIN_SHADOW = (ecap_regs->MUNIT_2_MIN_SHADOW) >> 1;
    ecap_regs->MUNIT_1_MAX_SHADOW = (ecap_regs->MUNIT_1_MAX_SHADOW) >> 1;
    ecap_regs->MUNIT_2_MAX_SHADOW = (ecap_regs->MUNIT_2_MAX_SHADOW) >> 1;

    /* force sync at ecap */
    ECAP_enableSoftwareSync(ecap_base_addr[instance], ECAP_MONITORING_UNIT_1);
    ECAP_enableSoftwareSync(ecap_base_addr[instance], ECAP_MONITORING_UNIT_2);

    if(
        ((ecap_regs->MUNIT_1_MIN) != (ecap_regs->MUNIT_1_MIN_SHADOW)) ||
        ((ecap_regs->MUNIT_2_MIN) != (ecap_regs->MUNIT_2_MIN_SHADOW)) ||
        ((ecap_regs->MUNIT_1_MAX) != (ecap_regs->MUNIT_1_MAX_SHADOW)) ||
        ((ecap_regs->MUNIT_2_MAX) != (ecap_regs->MUNIT_2_MAX_SHADOW))
    )
    {
        errors++;
        DebugP_logError("shadow to active load didn't happen with SwSync\r\n");
    }

    for(global_load_strobe = ECAP_MUNIT_GLDSTRB_EPWM0;
    global_load_strobe <= ECAP_MUNIT_GLDSTRB_EPWM31;
    global_load_strobe++)
    {
        uint16_t pwm_instance = global_load_strobe - ECAP_MUNIT_GLDSTRB_EPWM0;
        volatile uint32_t period_before_GLS = 0, period_after_GLS = 0;

        /*--------Pwm To Ecap_Capture_Instance Loopback Setting---------*/

        util_epmw_pinmux_configure(pwm_instance);
        uint8_t pwm_pin = (uint8_t) ((epwm_pins[pwm_instance*2])>>2);
        SOC_xbarSelectInputXBarInputSource(inputxbar_base_addr, 1, 0, pwm_pin, 0);
        DebugP_logInfo("inputxbar 1 is set for the pwm_pin gpio of : %d for PWM instance : %d\r\n", pwm_pin, pwm_instance);

        /*--------Pwm To Ecap_Capture_Instance Loopback Setting Complete---------*/

        uint16_t on_time = 50;
        volatile uint16_t period = pwm_instance*10 + 100;
        volatile uint16_t period_after_global_load_strobe = period*100;

        util_epwm_enable(pwm_instance);
        util_epwm_setup(pwm_instance, on_time, period);
        EPWM_disableSyncOutPulseSource(epwm_base_addr[pwm_instance], EPWM_SYNC_OUT_PULSE_ON_CNTR_ZERO);

        util_epwm_enable_global_strobe(pwm_instance, period, period_after_global_load_strobe);

        ecap_config_ptr->signal_monitor_global_load_strobe = global_load_strobe;
        ecap_config_ptr->sync_in_source = ECAP_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM0 + pwm_instance;

        ecap_config_ptr->munit1_ptr->shadow_load_mode = ECAP_ACTIVE_LOAD_SYNC_EVT;
        ecap_config_ptr->munit2_ptr->shadow_load_mode = ECAP_ACTIVE_LOAD_SYNC_EVT;

        util_ecap_configure(instance);
        ecap_regs->MUNIT_1_MIN_SHADOW = shadow_value;
        ecap_regs->MUNIT_2_MIN_SHADOW = shadow_value;
        ecap_regs->MUNIT_1_MAX_SHADOW = shadow_value;
        ecap_regs->MUNIT_2_MAX_SHADOW = shadow_value;

        /* with sync pulse from EPWMs */
        EPWM_forceSyncPulse(epwm_base_addr[pwm_instance]);

        if(
            ((ecap_regs->MUNIT_1_MIN) != (ecap_regs->MUNIT_1_MIN_SHADOW)) ||
            ((ecap_regs->MUNIT_2_MIN) != (ecap_regs->MUNIT_2_MIN_SHADOW)) ||
            ((ecap_regs->MUNIT_1_MAX) != (ecap_regs->MUNIT_1_MAX_SHADOW)) ||
            ((ecap_regs->MUNIT_2_MAX) != (ecap_regs->MUNIT_2_MAX_SHADOW))
        )
        {
            errors++;
            DebugP_logError("shadow to active load didn't happen when sync_pulse occured | PWM_instance : %d sync_in_source : %d \r\n", pwm_instance, ecap_config_ptr->sync_in_source );
        }

        ecap_regs->MUNIT_1_MIN_SHADOW = (ecap_regs->MUNIT_1_MIN_SHADOW) >> 1;
        ecap_regs->MUNIT_2_MIN_SHADOW = (ecap_regs->MUNIT_2_MIN_SHADOW) >> 1;
        ecap_regs->MUNIT_1_MAX_SHADOW = (ecap_regs->MUNIT_1_MAX_SHADOW) >> 1;
        ecap_regs->MUNIT_2_MAX_SHADOW = (ecap_regs->MUNIT_2_MAX_SHADOW) >> 1;

        /* with global strobe load from epwms*/
        ecap_config_ptr->munit1_ptr->shadow_load_mode = ECAP_ACTIVE_LOAD_GLDLCSTRB_EVT;
        ecap_config_ptr->munit2_ptr->shadow_load_mode = ECAP_ACTIVE_LOAD_GLDLCSTRB_EVT;
        ECAP_selectShadowLoadMode(ecap_base_addr[instance], ECAP_MONITORING_UNIT_1, ecap_config_ptr->munit1_ptr->shadow_load_mode);
        ECAP_selectShadowLoadMode(ecap_base_addr[instance], ECAP_MONITORING_UNIT_2, ecap_config_ptr->munit2_ptr->shadow_load_mode);

        EPWM_disableSyncOutPulseSource(epwm_base_addr[pwm_instance], EPWM_SYNC_OUT_PULSE_ON_ALL);
        util_ecap_start_timestamp(ecap_capture_instance);

        EPWM_setTimeBaseCounterMode(epwm_base_addr[pwm_instance], 0);
        ClockP_usleep(100);
        period_before_GLS = ECAP_getEventTimeStamp(ecap_base_addr[ecap_capture_instance], events[1]);
        EPWM_forceGlobalLoadOneShotEvent(epwm_base_addr[pwm_instance]);
        ClockP_usleep(1000);
        period_after_GLS = ECAP_getEventTimeStamp(ecap_base_addr[ecap_capture_instance], events[1]);
        EPWM_setTimeBaseCounterMode(epwm_base_addr[pwm_instance], 3);


        DebugP_logInfo("Period before GLS : %d, period after GLS : %d for the pwm_instance : %d\r\n", period_before_GLS, period_after_GLS, pwm_instance);
        if(!util_compare_in_range(period_before_GLS, period_after_GLS/100, 2))
        {
            status = BAD;
        }
        else if(
            ((ecap_regs->MUNIT_1_MIN) != (ecap_regs->MUNIT_1_MIN_SHADOW)) ||
            ((ecap_regs->MUNIT_2_MIN) != (ecap_regs->MUNIT_2_MIN_SHADOW)) ||
            ((ecap_regs->MUNIT_1_MAX) != (ecap_regs->MUNIT_1_MAX_SHADOW)) ||
            ((ecap_regs->MUNIT_2_MAX) != (ecap_regs->MUNIT_2_MAX_SHADOW))
        )
        {
            errors++;
            DebugP_logError("shadow to active load didn't happen when global Load pulse occured | PWM_instance : %d, option selected : %d \r\n", pwm_instance, ecap_config_ptr->signal_monitor_global_load_strobe);
        }
        ecap_regs->MUNIT_1_MIN_SHADOW = (ecap_regs->MUNIT_1_MIN_SHADOW) >> 1;
        ecap_regs->MUNIT_2_MIN_SHADOW = (ecap_regs->MUNIT_2_MIN_SHADOW) >> 1;
        ecap_regs->MUNIT_1_MAX_SHADOW = (ecap_regs->MUNIT_1_MAX_SHADOW) >> 1;
        ecap_regs->MUNIT_2_MAX_SHADOW = (ecap_regs->MUNIT_2_MAX_SHADOW) >> 1;

        SOC_xbarSelectInputXBarInputSource(inputxbar_base_addr, 1, 0, 0, 0); // setting a a GPIO 0 as a reset value.. not necessarily a reset option, but something thats not a pwm we are looking for.

        util_epmw_pinmux_restore(pwm_instance);
        util_epwm_reset(pwm_instance);
        util_epwm_disable_all();
    }
    util_ecap_deconfigure_and_reset(ecap_capture_instance);

    if(errors > 0)
    {
        status = BAD;
    }
    return status;
}

int32_t AM263x_ECAP_BTR_009(uint16_t instance)
{
    /*
    Signal Monitoring Tests
        Features :
            1. Two Monitors that can check individual "pulse/edge actions [ECAP_MonitoringTypeSelect]" within their own Min-Max window.
            2. Each monitor can generate 2 "interrupts" each based on event before/after Min-Max window.
            3. Trip the monitoring (disable the monitoring) based on the TripOut signal from any of the EPWMs.
            4. Global Strobe Load input from any of the EPWM and load its shadow contents to active Min/Max registers.
            5. debug registers for the min-max ranges
    */

    /*
    monitoring : the Capture block configuraitons
    ---- enable interrupt for these and check the flags based on the variation of the input. ----
    1. Absolute mode
    2. Continuous mode (optional one shot mode/ limited use)
    3. Sync not enabled for pulse monitoring, Sync in mandatory for edge monitoring.
    4. minimum 2 captures are configured. i.e., stop_wrap >=1 [if more are used, the SMU will use ]
        - High Pulse : RF
        - Low Pulse  : RF
        - Period RR  : RR
        - Period FF  : FF
    5. minimum 1 capture should be selected for edge monitoring / supposed to be internal pwm/signal monitoring.
        - Counter should be synced with EPWM.
        - Absolute mode
        - Continuous mode
        - CAP polarity should be of given interest.
    */

    /*
    enable disable the debug features in any one of the above scenarios and see if the logs are ok.
    */

    /*
    change the Min-Max shadow registers and select different xbar / epwm for the global strobe and validate one of the above scenarios.
    */

    /*
    Change the trip function and use for disabling the MUNITs
    */

    volatile uint8_t errors = 0;
    volatile bool status = GOOD;

    status = signal_monitor_mode_test(instance, ECAP_MUNIT_HIGH_PULSE_WIDTH);
    if(status == BAD)
    {
        DebugP_logError("High Pulse monitoring failed.\r\n");
        errors++;
    }

    status = GOOD;
    status = signal_monitor_mode_test(instance, ECAP_MUNIT_LOW_PULSE_WIDTH);
    if(status == BAD)
    {
        DebugP_logError("Low Pulse monitoring failed.\r\n");
        errors++;
    }

    status = GOOD;
    status = signal_monitor_mode_test(instance, ECAP_MUNIT_PERIOD_WIDTH_RISE_RISE);
    if(status == BAD)
    {
        DebugP_logError("Rise to Rise Period monitoring failed.\r\n");
        errors++;
    }

    status = GOOD;
    status = signal_monitor_mode_test(instance, ECAP_MUNIT_PERIOD_WIDTH_FALL_FALL);
    if(status == BAD)
    {
        DebugP_logError("Fall to Fall Period monitoring failed.\r\n");
        errors++;
    }

    status = GOOD;
    status = signal_monitor_mode_test(instance, ECAP_MUNIT_MONITOR_RISE_EDGE);
    if(status == BAD)
    {
        DebugP_logError("Rise Edge monitoring failed.\r\n");
        errors++;
    }

    status = GOOD;
    status = signal_monitor_mode_test(instance, ECAP_MUNIT_MONITOR_FALL_EDGE);
    if(status == BAD)
    {
        DebugP_logError("Fall Edge monitoring failed.\r\n");
        errors++;
    }

    status = GOOD;
    status = global_strobe_load_tests(instance);
    if(status == BAD)
    {
        DebugP_logError("Global Strobe load test failed.\r\n");
        errors++;
    }

    if(errors > 0)
    {
        return 1;
    }
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
        case 8:
        {
            if(instance == 9)
            {
                DebugP_logError("resource busy for instance 9");
                failcount++;
                break;
            }
            failcount += AM263x_ECAP_BTR_008(instance);
            single_instance_test = true;
            break;
        }
        case 9:
        {
            if(instance == 9)
            {
                DebugP_logError("resource busy for instance 9");
                failcount++;
                break;
            }
            failcount += AM263x_ECAP_BTR_009(instance);
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