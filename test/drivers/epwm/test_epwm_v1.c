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
#include <stdlib.h>
#include<stdio.h>
#include <unity.h>
#include <drivers/epwm.h>
#include <drivers/ecap.h>
#include <drivers/dac.h>
#include <drivers/adc.h>
#include <drivers/cmpss.h>
#include <drivers/gpio.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <drivers/soc.h>
#include <../source/kernel/dpl/CycleCounterP.h>
/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define NUM_TESTS 45

#define CMD_TX_BUFSIZE      (200U)
#define CMD_RX_BUFSIZE      (200U)

#define CMD_SIZE            (64U)
#define RSP_SIZE            (64U)

#define LENGTH_OF_CAPTURE_INDEPENDENT_TEST_LIST   (32)
#define LENGTH_OF_CAPTURE_DEPENDENT_TEST_LIST     (13)

#define TIMEOUT_UART_MENU       (10000)
#define TIMEOUT_UART_TESTER     (10000)
#define TIMEOUT_UART_INFINITE   (SystemP_WAIT_FOREVER)
bool enableLog;
bool manual_testing;
/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
void tester_init(void);
void tester_command(char *str);
void util_setEPWMTB(uint32_t base, uint32_t emuMode, uint32_t clkDiv, uint32_t hsClkDiv, uint32_t tprd, uint32_t countMode);
void util_setEPWMCC(uint32_t base, uint32_t CC_A, uint32_t CC_B);
void util_resetEPWMTB(uint32_t base);
void util_resetEPWMCC(uint32_t base);
void util_resetEPWMAQ(uint32_t base);
void util_deinit_epwms(uint32_t i);
void util_deinit_dac();
void ECAP_inApwmMode(uint32_t base);
uint32_t util_CMPSS_getInstanceFromBase(uint32_t base);
void util_deinit_cmpss(uint32_t base);
void util_adc_init_configure_soc_source(uint16_t adc_instance, uint16_t epwm_instance);
void util_adc_reset(uint16_t adc_instance);
void util_EPWM_setup_adc_trigger(uint32_t epwm_base, EPWM_ADCStartOfConversionType adc_soc_type,
                                 uint32_t trigger_source, uint16_t prescale);
void EnablePinMux(void);
void EnableEPWMClk(void);
void ECAP_UART_read();
int startsWith(char *str, char *substr);
void Configure_Xbar_Sync_DUT(uint32_t epwm_offset);
/* Testcases */
static void EPWM_setClockPrescalerApiCheck(void *args);
static void EPWM_setRFEdgeDelayCountApiCheck(void *args);

static void EPWM_complementary_pwm(void *args);
static void EPWM_pwm_clk_sync(void *args);
static void EPWM_hrpwm_operation(void *args);
static void EPWM_hrpwm_operation_with_sfo(void *args);
static void EPWM_pwm_link_feature(void *args);
static void EPWM_pwm_chopper_operation(void *args);
static void EPWM_all_pwm_working_together(void *args);
static void EPWM_dac_load_upon_pwm_event(void *args);
static void EPWM_cmpss_load_upon_pwm_event(void *args);
static void EPWM_gpio_signal_trips_pwm(void *args);
static void EPWM_emustop_trip_pwm(void *args);
static void EPWM_cmpss_to_trip_pwm(void *args);
static void EPWM_syserror_trips_pwm(void *args);
static void EPWM_deadband_basic(void *args);
static void EPWM_diode_emulation_with_cmpss(void *args);
static void EPWM_adc_conversion_with_pwm_event(void *args);
static void EPWM_diode_emulation_and_mindb(void *args);
static void EPWM_sdfm_trips_pwm(void *args);
static void EPWM_illegal_combo_logic(void *args);
static void EPWM_epwmxlinkxload_feature(void *args);
static void EPWM_hrpwm_calibration(void *args);
static void EPWM_hrpwm_functionality(void *args);
static void EPWM_hrpwm_sfo_function_usage(void *args);
static void EPWM_pwm_latency_basic(void *args);
static void EPWM_pwm_latency_through_r5f_cores(void *args);
static void EPWM_pwm_latency_through_dma(void *args);
static void EPWM_latency_of_fast_access_bridge_registers_through_r5f(void *args);
static void EPWM_latency_of_fast_access_bridge_shadow_registers_through_r5f(void *args);
static void EPWM_latency_of_fast_access_bridge_registers_through_dma(void *args);
static void EPWM_latency_of_fast_access_bridge_shadow_registers_through_dma(void *args);
static void EPWM_latency_of_fast_access_bridge_hrpwm_registers_through_r5f(void *args);
static void EPWM_support_pwm_chopping(void *args);
static void EPWM_add_additional_syncin_sources(void *args);
static void EPWM_functionalities_for_mdl_and_icl(void *args);
static void EPWM_dc_cbc_latch(void *args);
static void EPWM_set_trip_zone_action(void *args);
static void EPWM_generation_of_adc_soc_and_cpu_interrupt_on_all_events(void *args);
static void EPWM_control_phase_shifting(void *args);
static void EPWM_generating_pwm_signals_using_16_bit_counter(void *args);
static void EPWM_programming_event_prescaling(void *args);
static void EPWM_one_shot_sync_out_trigger(void *args);
static void EPWM_support_deadband_generation(void *args);
static void EPWM_support_asychronous_override_control_of_pwm_signals_through_software(void *args);
static void EPWM_support_generation_of_2_pwm_outputs_with_dual_edge_symmetric_operation(void *args);
static void EPWM_select_one_shot_sync_out_trigger(void *args);
static void EPWM_grouping_aliasing_of_epwm_instances(void *args);
static void EPWM_XCMP(void *args);
static void EPWM_DEL(void *args);
static void EPWM_checkAPIs(void *args);

int32_t test_epwm_cases(uint8_t in);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
uint8_t gCmdTxBuffer[CMD_TX_BUFSIZE];
uint8_t gCmdRxBuffer[CMD_RX_BUFSIZE];
volatile uint32_t gNumBytesRead = 0U, gNumBytesWritten = 0U;
int32_t          transferOK;
int dummy;
UART_Transaction trans;
int32_t ts[4];
uint32_t tb_clk_vector[][3] = { {EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1, 0xFFFF},
                                {EPWM_CLOCK_DIVIDER_2, EPWM_HSCLOCK_DIVIDER_1, 0x5DC0},
                                {EPWM_CLOCK_DIVIDER_2, EPWM_HSCLOCK_DIVIDER_2, 0x5DC0},
                               };

uint32_t tb_countmode_vector[] = {EPWM_COUNTER_MODE_UP, EPWM_COUNTER_MODE_DOWN, EPWM_COUNTER_MODE_UP_DOWN};

uint32_t cc_values_vector[] = {0x3A98, 0x4650, 0x5DC0};

#ifdef SOC_AM263X

uint8_t isTestSupported[32][49] =
{
    {0, 1,0,1,0, 1,1,1,1, 1,1,1,1, 1,1,1,1, 1,1,1,1, 1,1,1,1, 1,1,1,1, 1,1,1,1, 1,1,1,1, 1,1,1,1, 1,1,1,1, 1,1,1,1},
    {1, 1,1,1,0, 0,1,0,1, 1,1,1,1, 1,1,1,1, 1,1,1,0, 1,1,1,1, 1,1,1,1, 1,1,1,1, 1,1,1,1, 1,1,1,1, 1,1,1,1, 1,1,1,1},
    {2, 1,0,1,0, 0,1,0,1, 1,1,1,1, 1,1,1,1, 1,1,1,0, 1,1,1,1, 1,1,1,1, 1,1,1,1, 1,1,1,1, 1,1,1,1, 1,1,1,1, 1,1,1,1},
    {3, 1,0,1,0, 0,1,0,1, 1,1,1,1, 1,1,1,1, 1,1,1,0, 1,1,1,1, 1,1,1,1, 1,1,1,1, 1,1,1,1, 1,1,1,1, 1,1,1,1, 1,1,1,1},
    {4, 1,0,1,0, 0,1,0,1, 1,1,0,1, 0,1,1,1, 1,1,1,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 1,1,1,1, 1,1,1,1, 1,1,1,1, 0,1,1,1},
    {5, 1,0,1,0, 0,1,0,1, 1,1,0,1, 0,1,1,1, 1,1,1,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 1,1,1,1, 1,1,1,1, 1,1,1,1, 0,1,1,1},
    {6, 1,0,1,0, 0,1,0,1, 1,1,0,1, 0,1,1,1, 1,1,1,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 1,1,1,1, 1,1,1,1, 1,1,1,1, 0,1,1,1},
    {7, 1,0,1,0, 0,1,0,1, 1,1,0,1, 0,1,1,1, 1,1,1,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 1,1,1,1, 1,1,1,1, 1,1,1,1, 0,1,1,1},
    {8, 0,0,0,0, 0,0,0,1, 1,1,0,1, 0,0,1,1, 1,1,1,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 1,1,1,1, 1,0,1,1, 0,1,0,1, 0,1,1,1},
    {9, 0,0,0,0, 0,0,0,1, 1,1,0,1, 0,0,1,1, 1,1,1,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 1,1,1,1, 1,0,1,1, 0,1,0,1, 0,1,1,1},
    {10, 0,0,0,0, 0,0,0,1, 1,1,0,1, 0,0,1,1, 1,1,1,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 1,1,1,1, 1,0,1,1, 0,1,0,1, 0,1,1,1},
    {11, 0,0,0,0, 0,0,0,1, 1,1,0,1, 0,0,1,1, 1,1,1,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 1,1,1,1, 1,0,1,1, 0,1,0,1, 0,1,1,1},
    {12, 0,0,0,0, 0,0,0,1, 1,1,0,1, 0,0,1,1, 1,1,1,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 1,1,1,1, 1,0,1,1, 0,1,0,1, 0,1,1,1},
    {13, 0,0,0,0, 0,0,0,1, 1,1,0,1, 0,0,1,1, 1,1,1,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 1,1,1,1, 1,0,1,1, 0,1,0,1, 0,1,1,1},
    {14, 0,0,0,0, 0,0,0,1, 1,1,0,1, 0,0,1,1, 1,1,1,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 1,1,1,1, 1,0,1,1, 0,1,0,1, 0,1,1,1},
    {15, 0,0,0,0, 0,0,0,1, 1,1,0,1, 0,0,1,1, 1,1,1,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 1,1,1,1, 1,0,1,1, 0,1,0,1, 0,1,1,1},
    {16, 1,0,0,0, 0,1,0,1, 1,1,0,1, 0,1,1,1, 1,1,1,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 1,1,1,1, 1,1,1,1, 1,1,0,1, 0,1,1,1},
    {17, 0,0,0,0, 0,0,0,1, 1,1,0,1, 0,0,1,1, 1,1,1,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 1,1,1,1, 1,0,1,1, 0,1,0,1, 0,1,1,1},
    {18, 0,0,0,0, 0,0,0,1, 1,1,0,1, 0,0,1,1, 1,1,1,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 1,1,1,1, 1,0,1,1, 0,1,0,1, 0,1,1,1},
    {19, 0,0,0,0, 0,0,0,1, 1,1,0,1, 0,0,1,1, 1,1,1,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 1,1,1,1, 1,0,1,1, 0,1,0,1, 0,1,1,1},
    {20, 0,0,0,0, 0,0,0,1, 1,1,0,1, 0,0,1,1, 1,1,1,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 1,1,1,1, 1,0,1,1, 0,1,0,1, 0,1,1,1},
    {21, 0,0,0,0, 0,0,0,1, 1,1,0,1, 0,0,1,1, 1,1,1,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 1,1,1,1, 1,0,1,1, 0,1,0,1, 0,1,1,1},
    {22, 0,0,0,0, 0,0,0,1, 1,1,0,1, 0,0,1,1, 1,1,1,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 1,1,1,1, 1,0,1,1, 0,1,0,1, 0,1,1,1},
    {23, 0,0,0,0, 0,0,0,1, 1,1,0,1, 0,0,1,1, 1,1,1,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 1,1,1,1, 1,0,1,1, 0,1,0,1, 0,1,1,1},
    {24, 0,0,0,0, 0,0,0,1, 1,1,0,1, 0,0,1,1, 1,1,1,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 1,1,1,1, 1,0,1,1, 0,1,0,1, 0,1,1,1},
    {25, 0,0,0,0, 0,0,0,1, 1,1,0,1, 0,0,1,1, 1,1,1,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 1,1,1,1, 1,0,1,1, 0,1,0,1, 0,1,1,1},
    {26, 0,0,0,0, 0,0,0,1, 1,1,0,1, 0,0,1,1, 1,1,1,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 1,1,1,1, 1,0,1,1, 0,1,0,1, 0,1,1,1},
    {27, 0,0,0,0, 0,0,0,1, 1,1,0,1, 0,0,1,1, 1,1,1,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 1,1,1,1, 1,0,1,1, 0,1,0,1, 0,1,1,1},
    {28, 0,0,0,0, 0,0,0,1, 1,1,0,1, 0,0,1,1, 1,1,1,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 1,1,1,1, 1,0,1,1, 0,1,0,1, 0,1,1,1},
    {29, 0,0,0,0, 0,0,0,1, 1,1,0,1, 0,0,1,1, 1,1,1,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 1,1,1,1, 1,0,1,1, 0,1,0,1, 0,1,1,1},
    {30, 0,0,0,0, 0,0,0,1, 1,1,0,1, 0,0,1,1, 1,1,1,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 1,1,1,1, 1,0,1,1, 0,1,0,1, 0,1,1,1},
    {31, 0,0,0,0, 0,0,0,1, 1,1,0,1, 0,0,1,1, 1,1,1,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 1,1,1,1, 1,0,1,1, 0,1,0,1, 0,1,1,1},
};

#endif
#ifdef SOC_AM263PX

uint8_t isTestSupported[32][49] =
{
    {0, 1,0,1,0, 1,1,1,1, 1,1,1,1, 1,1,1,1, 1,1,1,1, 1,1,1,1, 1,1,1,1, 1,1,1,1, 1,1,1,1, 1,1,1,1, 1,1,1,1, 1,1,1,1},
    {1, 1,1,1,0, 0,1,0,1, 1,1,1,1, 1,1,1,1, 1,1,1,0, 1,1,1,1, 1,1,1,1, 1,1,1,1, 1,1,1,1, 1,1,1,1, 1,1,1,1, 1,1,1,1},
    {2, 1,0,1,0, 0,1,0,1, 1,1,1,1, 1,1,1,1, 1,1,1,0, 1,1,1,1, 1,1,1,1, 1,1,1,1, 1,1,1,1, 1,1,1,1, 1,1,1,1, 1,1,1,1},
    {3, 1,0,1,0, 0,1,0,1, 1,1,1,1, 1,1,1,1, 1,1,1,0, 1,1,1,1, 1,1,1,1, 1,1,1,1, 1,1,1,1, 1,1,1,1, 1,1,1,1, 1,1,1,1},
    {4, 1,0,1,0, 0,1,0,1, 1,1,0,1, 0,1,1,1, 1,1,1,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 1,1,1,1, 1,1,1,1, 1,1,1,1, 0,1,1,1},
    {5, 1,0,1,0, 0,1,0,1, 1,1,0,1, 0,1,1,1, 1,1,1,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 1,1,1,1, 1,1,1,1, 1,1,1,1, 0,1,1,1},
    {6, 1,0,1,0, 0,1,0,1, 1,1,0,1, 0,1,1,1, 1,1,1,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 1,1,1,1, 1,1,1,1, 1,1,1,1, 0,1,1,1},
    {7, 1,0,1,0, 0,1,0,1, 1,1,0,1, 0,1,1,1, 1,1,1,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 1,1,1,1, 1,1,1,1, 1,1,1,1, 0,1,1,1},
    {8, 0,0,0,0, 0,0,0,1, 1,1,0,1, 0,0,1,1, 1,1,1,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 1,1,1,1, 1,0,1,1, 0,1,0,1, 0,1,1,1},
    {9, 0,0,0,0, 0,0,0,1, 1,1,0,1, 0,0,1,1, 1,1,1,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 1,1,1,1, 1,0,1,1, 0,1,0,1, 0,1,1,1},
    {10, 0,0,0,0, 0,0,0,1, 1,1,0,1, 0,0,1,1, 1,1,1,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 1,1,1,1, 1,0,1,1, 0,1,0,1, 0,1,1,1},
    {11, 0,0,0,0, 0,0,0,1, 1,1,0,1, 0,0,1,1, 1,1,1,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 1,1,1,1, 1,0,1,1, 0,1,0,1, 0,1,1,1},
    {12, 0,0,0,0, 0,0,0,1, 1,1,0,1, 0,0,1,1, 1,1,1,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 1,1,1,1, 1,0,1,1, 0,1,0,1, 0,1,1,1},
    {13, 0,0,0,0, 0,0,0,1, 1,1,0,1, 0,0,1,1, 1,1,1,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 1,1,1,1, 1,0,1,1, 0,1,0,1, 0,1,1,1},
    {14, 0,0,0,0, 0,0,0,1, 1,1,0,1, 0,0,1,1, 1,1,1,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 1,1,1,1, 1,0,1,1, 0,1,0,1, 0,1,1,1},
    {15, 0,0,0,0, 0,0,0,1, 1,1,0,1, 0,0,1,1, 1,1,1,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 1,1,1,1, 1,0,1,1, 0,1,0,1, 0,1,1,1},
    {16, 0,0,0,0, 0,0,0,1, 1,1,0,1, 0,0,1,1, 1,1,1,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 1,1,1,1, 1,0,1,1, 0,1,0,1, 0,1,1,1},
    {17, 0,0,0,0, 0,0,0,1, 1,1,0,1, 0,0,1,1, 1,1,1,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 1,1,1,1, 1,0,1,1, 0,1,0,1, 0,1,1,1},
    {18, 0,0,0,0, 0,0,0,1, 1,1,0,1, 0,0,1,1, 1,1,1,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 1,1,1,1, 1,0,1,1, 0,1,0,1, 0,1,1,1},
    {19, 0,0,0,0, 0,0,0,1, 1,1,0,1, 0,0,1,1, 1,1,1,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 1,1,1,1, 1,0,1,1, 0,1,0,1, 0,1,1,1},
    {20, 0,0,0,0, 0,0,0,1, 1,1,0,1, 0,0,1,1, 1,1,1,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 1,1,1,1, 1,0,1,1, 0,1,0,1, 0,1,1,1},
    {21, 0,0,0,0, 0,0,0,1, 1,1,0,1, 0,0,1,1, 1,1,1,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 1,1,1,1, 1,0,1,1, 0,1,0,1, 0,1,1,1},
    {22, 0,0,0,0, 0,0,0,1, 1,1,0,1, 0,0,1,1, 1,1,1,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 1,1,1,1, 1,0,1,1, 0,1,0,1, 0,1,1,1},
    {23, 0,0,0,0, 0,0,0,1, 1,1,0,1, 0,0,1,1, 1,1,1,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 1,1,1,1, 1,0,1,1, 0,1,0,1, 0,1,1,1},
    {24, 0,0,0,0, 0,0,0,1, 1,1,0,1, 0,0,1,1, 1,1,1,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 1,1,1,1, 1,0,1,1, 0,1,0,1, 0,1,1,1},
    {25, 0,0,0,0, 0,0,0,1, 1,1,0,1, 0,0,1,1, 1,1,1,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 1,1,1,1, 1,0,1,1, 0,1,0,1, 0,1,1,1},
    {26, 0,0,0,0, 0,0,0,1, 1,1,0,1, 0,0,1,1, 1,1,1,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 1,1,1,1, 1,0,1,1, 0,1,0,1, 0,1,1,1},
    {27, 0,0,0,0, 0,0,0,1, 1,1,0,1, 0,0,1,1, 1,1,1,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 1,1,1,1, 1,0,1,1, 0,1,0,1, 0,1,1,1},
    {28, 0,0,0,0, 0,0,0,1, 1,1,0,1, 0,0,1,1, 1,1,1,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 1,1,1,1, 1,0,1,1, 0,1,0,1, 0,1,1,1},
    {29, 0,0,0,0, 0,0,0,1, 1,1,0,1, 0,0,1,1, 1,1,1,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 1,1,1,1, 1,0,1,1, 0,1,0,1, 0,1,1,1},
    {30, 0,0,0,0, 0,0,0,1, 1,1,0,1, 0,0,1,1, 1,1,1,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 1,1,1,1, 1,0,1,1, 0,1,0,1, 0,1,1,1},
    {31, 0,0,0,0, 0,0,0,1, 1,1,0,1, 0,0,1,1, 1,1,1,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 1,1,1,1, 1,0,1,1, 0,1,0,1, 0,1,1,1},
};
#endif

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void test_main(void *args)
{
    /* Open drivers */
    Drivers_open();
    Board_driversOpen();
    EnablePinMux();
    EnableEPWMClk();

    tester_init();

    UNITY_BEGIN();


    char input_independent_tests[LENGTH_OF_CAPTURE_INDEPENDENT_TEST_LIST+1][100]={
        "All independent and Quit",
        "EPWM_setClockPrescalerApiCheck",
        "EPWM_setRFEdgeDelayCountApiCheck",
        "EPWM_pwm_link_feature",
        "EPWM_dac_load_upon_pwm_event",
        "EPWM_cmpss_load_upon_pwm_event",
        "EPWM_diode_emulation_with_cmpss",
        "EPWM_adc_conversion_with_pwm_event",
        "EPWM_diode_emulation_and_mindb",
        "EPWM_illegal_combo_logic",
        "EPWM_epwmxlinkxload_feature",
        "EPWM_add_additional_syncin_sources",
        "EPWM_dc_cbc_latch",
        "EPWM_generation_of_adc_soc_and_cpu_interrupt_on_all_events",
        "EPWM_control_phase_shifting",
        "EPWM_programming_event_prescaling",
        "EPWM_one_shot_sync_out_trigger",
        "EPWM_support_asychronous_override_control_of_pwm_signals_through_software",
        "EPWM_select_one_shot_sync_out_trigger",
        "EPWM_grouping_aliasing_of_epwm_instances",
        "EPWM_XCMP",
        "EPWM_DEL",

        "EPWM_emustop_trip_pwm",
        "EPWM_syserror_trips_pwm",
        "EPWM_pwm_latency_basic",
        "EPWM_pwm_latency_through_r5f_cores",
        "EPWM_pwm_latency_through_dma",
        "EPWM_latency_of_fast_access_bridge_registers_through_r5f",
        "EPWM_latency_of_fast_access_bridge_shadow_registers_through_r5f",
        "EPWM_latency_of_fast_access_bridge_registers_through_dma",
        "EPWM_latency_of_fast_access_bridge_shadow_registers_through_dma",
        "EPWM_latency_of_fast_access_bridge_hrpwm_registers_through_r5f",
        "EPWM_checkAPIs"
    };

    char input_dependent_tests[LENGTH_OF_CAPTURE_DEPENDENT_TEST_LIST+1][100] = {
        "EPWM_complementary_pwm",
        "EPWM_pwm_clk_sync",
        "EPWM_all_pwm_working_together",
        "EPWM_pwm_chopper_operation",
        "EPWM_gpio_signal_trips_pwm",
        "EPWM_cmpss_to_trip_pwm",
        "EPWM_deadband_basic",
        "EPWM_support_generation_of_2_pwm_outputs_with_dual_edge_symmetric_operation",
        "EPWM_generating_pwm_signals_using_16_bit_counter",
        "EPWM_set_trip_zone_action",
        "EPWM_hrpwm_operation",

        "EPWM_hrpwm_operation_with_sfo",
        "EPWM_sdfm_trips_pwm",
        "All tests and Quit"
    };

    int while_breaker = 0;

    while(!while_breaker)
    {
        enableLog = 0;
        manual_testing = 0;
        DebugP_log("\r\n********** EPWM Unit Test Menu **********\r\n");
        DebugP_log("\r\n01\t:\tmanual testing\r\n02\t:\tauto testing (using Tester Applicaiton)\r\n03\t:\tQuit\r\n");

        UART_Transaction_init(&trans);
        uint8_t optionBuffer[3];
        int option;
        uint8_t i;

        for(i=0;i<3;i++)
        {
            optionBuffer[i]=0;
        }

        trans.buf       = &optionBuffer[0U];
        trans.count     = 2;
        trans.timeout   = TIMEOUT_UART_MENU;
        transferOK      = UART_read(gUartHandle[CONFIG_UART0], &trans);

        if((SystemP_SUCCESS != (transferOK)) || (UART_TRANSFER_STATUS_SUCCESS != trans.status))
        {
            DebugP_log("\r\nInvalid input! \r\n Auto testing (using Tester Application) is selected by default");
            manual_testing = 0;
        }
        else
        {
            option = (int) atof((char *)optionBuffer);
            DebugP_log("\r\nEntered option: %02d\r\n", option);
            if(option == 1)
            {
                manual_testing = 1;
            }
            else if(option == 2)
            {
                DebugP_log("Auto testing (using Tester Application is selected)\r\n");
                DebugP_log("Please note:\r\n In Auto testing, Tester Application is used. The expected inputs to the tests are expected to be provided through the Tester Application\r\n");
            }
            else if (option == 3)
            {
                DebugP_log("Quitting\r\n");
                break;
            }
            else
            {
                DebugP_log("Invalid input.\r\n");
                continue;
            }
            if(manual_testing)
            {
                DebugP_log("Manual testing is selected\r\n");
                DebugP_log("Please note:\r\n In manual testing, please provide the required inputs for the tests\r\n");
            }

        }

        DebugP_log("\r\nCapture Independent Tests");
        for( i = 0; i < LENGTH_OF_CAPTURE_INDEPENDENT_TEST_LIST+1; i++)
        {
            DebugP_log("\r\n\t%02d:%s",i, input_independent_tests[i]);
        }

        DebugP_log("\r\n\r\nCapture Dependent Tests");
        for( i = 0; i < LENGTH_OF_CAPTURE_DEPENDENT_TEST_LIST+1; i++)
        {
            DebugP_log("\r\n\t%02d:%s",i+1+LENGTH_OF_CAPTURE_INDEPENDENT_TEST_LIST, input_dependent_tests[i]);
        }



        DebugP_log("\r\nEnter option (2 characters): \r\n");

        UART_Transaction_init(&trans);
        for(i=0;i<3;i++)
        {
            optionBuffer[i]=0;
        }
        trans.buf       = &optionBuffer[0U];
        trans.count     = 2;
        trans.timeout   = TIMEOUT_UART_MENU;
        transferOK      = UART_read(gUartHandle[CONFIG_UART0], &trans);

        if((SystemP_SUCCESS != (transferOK)) || (UART_TRANSFER_STATUS_SUCCESS != trans.status))
        {
            DebugP_log("\r\nInvalid input! \r\n selecting all and quit by default\r\n");
            option = LENGTH_OF_CAPTURE_INDEPENDENT_TEST_LIST+1 + LENGTH_OF_CAPTURE_DEPENDENT_TEST_LIST+1;     //""Run All tests and quit"
        }
        else
        {
            option = (int) atof((char *)optionBuffer);
            DebugP_log("\r\nEntered option: %02d\r\n", option);
        }

        char log_option;
        if(!manual_testing)
        {
            DebugP_log("Do you want to enable logs during the tests? [y/n]\t:\r\n");
            trans.buf       = &optionBuffer[0U];
            trans.count     = 1;
            trans.timeout   = TIMEOUT_UART_MENU;
            transferOK      = UART_read(gUartHandle[CONFIG_UART0], &trans);

            if((SystemP_SUCCESS != (transferOK)) || (UART_TRANSFER_STATUS_SUCCESS != trans.status))
            {
                DebugP_log("\r\nInvalid input! \r\n selecting disable logs by default\r\n");
                log_option = 'n';
            }
            else
            {
                log_option = optionBuffer[0];
                DebugP_log("\r\nEntered option: %c\r\n", log_option);
            }
            if ((log_option == 'y') || (log_option == 'Y'))
            {
                enableLog = 1;
            }
        }
        else
        {
            DebugP_log("Manual testing, Logs are enabled by default\r\n");
            enableLog = 1;
        }
        switch(option)
        {
            case 0:
                RUN_TEST(EPWM_setClockPrescalerApiCheck, 3327, NULL);
                RUN_TEST(EPWM_setRFEdgeDelayCountApiCheck,1,NULL);
                RUN_TEST(EPWM_pwm_link_feature , 3356, NULL);
                RUN_TEST(EPWM_dac_load_upon_pwm_event , 3359, NULL);
                RUN_TEST(EPWM_cmpss_load_upon_pwm_event , 3360, NULL);
                RUN_TEST(EPWM_diode_emulation_with_cmpss , 3366, NULL);
                RUN_TEST(EPWM_adc_conversion_with_pwm_event , 3367, NULL);
                RUN_TEST(EPWM_diode_emulation_and_mindb , 3368, NULL);
                RUN_TEST(EPWM_illegal_combo_logic , 3370, NULL);
                RUN_TEST(EPWM_epwmxlinkxload_feature , 3371, NULL);
                RUN_TEST(EPWM_add_additional_syncin_sources , 3415, NULL);
                RUN_TEST(EPWM_dc_cbc_latch , 3418, NULL);
                RUN_TEST(EPWM_generation_of_adc_soc_and_cpu_interrupt_on_all_events , 3420, NULL);
                RUN_TEST(EPWM_control_phase_shifting , 3421, NULL);
                RUN_TEST(EPWM_programming_event_prescaling , 3423, NULL);
                RUN_TEST(EPWM_one_shot_sync_out_trigger , 3427, NULL);
                RUN_TEST(EPWM_support_asychronous_override_control_of_pwm_signals_through_software , 3432, NULL);
                RUN_TEST(EPWM_select_one_shot_sync_out_trigger , 3439, NULL);
                RUN_TEST(EPWM_grouping_aliasing_of_epwm_instances , 3455, NULL);
                RUN_TEST(EPWM_XCMP,3850, NULL);
                RUN_TEST(EPWM_DEL,4213, NULL);
                /*
                * RUN_TEST(EPWM_emustop_trip_pwm , 3362, NULL);
                * RUN_TEST(EPWM_syserror_trips_pwm , 3364, NULL);
                */
                RUN_TEST(EPWM_pwm_latency_basic , 3375, NULL);
                /* RUN_TEST(EPWM_pwm_latency_through_r5f_cores , 3376, NULL);
                * RUN_TEST(EPWM_pwm_latency_through_dma , 3377, NULL);
                * RUN_TEST(EPWM_latency_of_fast_access_bridge_registers_through_r5f , 3378, NULL);
                * RUN_TEST(EPWM_latency_of_fast_access_bridge_shadow_registers_through_r5f , 3379, NULL);
                * RUN_TEST(EPWM_latency_of_fast_access_bridge_registers_through_dma , 3380, NULL);
                * RUN_TEST(EPWM_latency_of_fast_access_bridge_shadow_registers_through_dma , 3381, NULL);
                * RUN_TEST(EPWM_latency_of_fast_access_bridge_hrpwm_registers_through_r5f , 3382, NULL);
                */
                RUN_TEST(EPWM_checkAPIs, -1, NULL);
                while_breaker = 1;
                break;
            case 1:
                RUN_TEST(EPWM_setClockPrescalerApiCheck, 3327, NULL);
                break;
            case 2:
                RUN_TEST(EPWM_setRFEdgeDelayCountApiCheck,1,NULL);
                break;
            case 3:
                RUN_TEST(EPWM_pwm_link_feature , 3356, NULL);
                break;
            case 4:
                RUN_TEST(EPWM_dac_load_upon_pwm_event , 3359, NULL);
                break;
            case 5:
                RUN_TEST(EPWM_cmpss_load_upon_pwm_event , 3360, NULL);
                break;
            case 6:
                RUN_TEST(EPWM_diode_emulation_with_cmpss , 3366, NULL);
                break;
            case 7:
                RUN_TEST(EPWM_adc_conversion_with_pwm_event , 3367, NULL);
                break;
            case 8:
                RUN_TEST(EPWM_diode_emulation_and_mindb , 3368, NULL);
                break;
            case 9:
                RUN_TEST(EPWM_illegal_combo_logic , 3370, NULL);
                break;
            case 10:
                RUN_TEST(EPWM_epwmxlinkxload_feature , 3371, NULL);
                break;
            case 11:
                RUN_TEST(EPWM_add_additional_syncin_sources , 3415, NULL);
                break;
            case 12:
                RUN_TEST(EPWM_dc_cbc_latch , 3418, NULL);
                break;
            case 13:
                RUN_TEST(EPWM_generation_of_adc_soc_and_cpu_interrupt_on_all_events , 3420, NULL);
                break;
            case 14:
                RUN_TEST(EPWM_control_phase_shifting , 3421, NULL);
                break;
            case 15:
                RUN_TEST(EPWM_programming_event_prescaling , 3423, NULL);
                break;
            case 16:
                RUN_TEST(EPWM_one_shot_sync_out_trigger , 3427, NULL);
                break;
            case 17:
                RUN_TEST(EPWM_support_asychronous_override_control_of_pwm_signals_through_software , 3432, NULL);
                break;
            case 18:
                RUN_TEST(EPWM_select_one_shot_sync_out_trigger , 3439, NULL);
                break;
            case 19:
                RUN_TEST(EPWM_grouping_aliasing_of_epwm_instances , 3455, NULL);
                break;
            case 20:
                RUN_TEST(EPWM_XCMP,3850, NULL);
                break;
            case 21:
                RUN_TEST(EPWM_DEL,4213, NULL);
                break;
            /* case 22:                                                                                         */
            /*      RUN_TEST(EPWM_emustop_trip_pwm , 3362, NULL);                                               */
            /*     break;                                                                                       */
            /* case 23:                                                                                         */
            /*      RUN_TEST(EPWM_syserror_trips_pwm , 3364, NULL);                                             */
            /*     break;                                                                                       */
            case 24:
                 RUN_TEST(EPWM_pwm_latency_basic , 3375, NULL);
                break;
            /* case 25:                                                                                         */
            /*      RUN_TEST(EPWM_pwm_latency_through_r5f_cores , 3376, NULL);                                  */
            /*     break;                                                                                       */
            /* case 26:                                                                                         */
            /*      RUN_TEST(EPWM_pwm_latency_through_dma , 3377, NULL);                                        */
            /*     break;                                                                                       */
            /* case 27:                                                                                         */
            /*      RUN_TEST(EPWM_latency_of_fast_access_bridge_registers_through_r5f , 3378, NULL);            */
            /*     break;                                                                                       */
            /* case 28:                                                                                         */
            /*      RUN_TEST(EPWM_latency_of_fast_access_bridge_shadow_registers_through_r5f , 3379, NULL);     */
            /*     break;                                                                                       */
            /* case 29:                                                                                         */
            /*      RUN_TEST(EPWM_latency_of_fast_access_bridge_registers_through_dma , 3380, NULL);            */
            /*     break;                                                                                       */
            /* case 30:                                                                                         */
            /*      RUN_TEST(EPWM_latency_of_fast_access_bridge_shadow_registers_through_dma , 3381, NULL);     */
            /*     break;                                                                                       */
            /* case 31:                                                                                         */
            /*      RUN_TEST(EPWM_latency_of_fast_access_bridge_hrpwm_registers_through_r5f , 3382, NULL);      */
            /*     break;                                                                                       */

            case 32:
                RUN_TEST(EPWM_checkAPIs, -1, NULL);
                break;

            case 33:
                RUN_TEST(EPWM_complementary_pwm , 3352, NULL);
                break;
            case 34:
                RUN_TEST(EPWM_pwm_clk_sync , 3353, NULL);
                break;
            case 35:
                RUN_TEST(EPWM_all_pwm_working_together , 3358, NULL);
                break;
            case 36:
                RUN_TEST(EPWM_pwm_chopper_operation , 3357, NULL);
                break;
            case 37:
                RUN_TEST(EPWM_gpio_signal_trips_pwm , 3361, NULL);
                break;
            case 38:
                RUN_TEST(EPWM_cmpss_to_trip_pwm , 3363, NULL);
                break;
            case 39:
                RUN_TEST(EPWM_deadband_basic , 3365, NULL);
                break;
            case 40:
                RUN_TEST(EPWM_support_generation_of_2_pwm_outputs_with_dual_edge_symmetric_operation , 3438, NULL);
                break;
            case 41:
                RUN_TEST(EPWM_generating_pwm_signals_using_16_bit_counter , 3422, NULL);
                break;
            case 42:
                RUN_TEST(EPWM_set_trip_zone_action , 3419, NULL);
                break;
            case 43:
                RUN_TEST(EPWM_hrpwm_operation , 3354, NULL);
                break;
            case 44:
                /* RUN_TEST(EPWM_hrpwm_operation_with_sfo , 3355, NULL); */
                break;
            case 45:
                /* RUN_TEST(EPWM_sdfm_trips_pwm , 3369, NULL); */
                break;

            case 46:
                RUN_TEST(EPWM_setClockPrescalerApiCheck, 3327, NULL);
                RUN_TEST(EPWM_setRFEdgeDelayCountApiCheck,1,NULL);
                RUN_TEST(EPWM_pwm_link_feature , 3356, NULL);
                RUN_TEST(EPWM_dac_load_upon_pwm_event , 3359, NULL);
                RUN_TEST(EPWM_cmpss_load_upon_pwm_event , 3360, NULL);
                RUN_TEST(EPWM_diode_emulation_with_cmpss , 3366, NULL);
                RUN_TEST(EPWM_adc_conversion_with_pwm_event , 3367, NULL);
                RUN_TEST(EPWM_diode_emulation_and_mindb , 3368, NULL);
                RUN_TEST(EPWM_illegal_combo_logic , 3370, NULL);
                RUN_TEST(EPWM_epwmxlinkxload_feature , 3371, NULL);
                RUN_TEST(EPWM_add_additional_syncin_sources , 3415, NULL);
                RUN_TEST(EPWM_dc_cbc_latch , 3418, NULL);
                RUN_TEST(EPWM_generation_of_adc_soc_and_cpu_interrupt_on_all_events , 3420, NULL);
                RUN_TEST(EPWM_control_phase_shifting , 3421, NULL);
                RUN_TEST(EPWM_programming_event_prescaling , 3423, NULL);
                RUN_TEST(EPWM_one_shot_sync_out_trigger , 3427, NULL);
                RUN_TEST(EPWM_support_asychronous_override_control_of_pwm_signals_through_software , 3432, NULL);
                RUN_TEST(EPWM_select_one_shot_sync_out_trigger , 3439, NULL);
                RUN_TEST(EPWM_grouping_aliasing_of_epwm_instances , 3455, NULL);
                RUN_TEST(EPWM_XCMP,3850, NULL);


                /* RUN_TEST(EPWM_DEL,4213, NULL); */
                /* RUN_TEST(EPWM_emustop_trip_pwm , 3362, NULL); */
                /* RUN_TEST(EPWM_syserror_trips_pwm , 3364, NULL); */
                RUN_TEST(EPWM_pwm_latency_basic , 3375, NULL);
                /* RUN_TEST(EPWM_pwm_latency_through_r5f_cores , 3376, NULL); */
                /* RUN_TEST(EPWM_pwm_latency_through_dma , 3377, NULL); */
                /* RUN_TEST(EPWM_latency_of_fast_access_bridge_registers_through_r5f , 3378, NULL); */
                /* RUN_TEST(EPWM_latency_of_fast_access_bridge_shadow_registers_through_r5f , 3379, NULL); */
                /* RUN_TEST(EPWM_latency_of_fast_access_bridge_registers_through_dma , 3380, NULL); */
                /* RUN_TEST(EPWM_latency_of_fast_access_bridge_shadow_registers_through_dma , 3381, NULL); */
                /* RUN_TEST(EPWM_latency_of_fast_access_bridge_hrpwm_registers_through_r5f , 3382, NULL); */

                RUN_TEST(EPWM_checkAPIs, -1, NULL);

                RUN_TEST(EPWM_complementary_pwm , 3352, NULL);
                RUN_TEST(EPWM_pwm_clk_sync , 3353, NULL);
                RUN_TEST(EPWM_all_pwm_working_together , 3358, NULL);
                RUN_TEST(EPWM_pwm_chopper_operation , 3357, NULL);
                RUN_TEST(EPWM_gpio_signal_trips_pwm , 3361, NULL);
                RUN_TEST(EPWM_cmpss_to_trip_pwm , 3363, NULL);
                RUN_TEST(EPWM_deadband_basic , 3365, NULL);
                RUN_TEST(EPWM_support_generation_of_2_pwm_outputs_with_dual_edge_symmetric_operation , 3438, NULL);
                RUN_TEST(EPWM_generating_pwm_signals_using_16_bit_counter , 3422, NULL);
                RUN_TEST(EPWM_set_trip_zone_action , 3419, NULL);
                RUN_TEST(EPWM_hrpwm_operation , 3354, NULL);
                /* RUN_TEST(EPWM_hrpwm_operation_with_sfo , 3355, NULL); */
                /* RUN_TEST(EPWM_sdfm_trips_pwm , 3369, NULL); */
                while_breaker = 1;
                break;
            default :
                DebugP_log("Invalid Input. try again\r\n");
                break;
        }
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
}

void tearDown(void)
{
}



void util_setEPWMTB(uint32_t base, uint32_t emuMode, uint32_t clkDiv, uint32_t hsClkDiv, uint32_t tprd, uint32_t countMode)
{
   bool ret = false;
   EPWM_setEmulationMode(base, emuMode);
   EPWM_setClockPrescaler(base, clkDiv, hsClkDiv);

   EPWM_setTimeBasePeriod(base, tprd);
   if((HW_RD_REG16(base + CSL_EPWM_TBPRD) == (uint16_t)EPWM_getTimeBasePeriod(base))
    && (HW_RD_REG16(base + CSL_EPWM_TBPRD)) == tprd)
        ret = true;

   EPWM_setTimeBaseCounterMode(base, countMode);
   TEST_ASSERT_EQUAL_INT32(0x1, ret);
}

void util_resetEPWMTB(uint32_t base)
{
   EPWM_setEmulationMode(base, 0x0);
   EPWM_setClockPrescaler(base, 0x0, 0x1);
   EPWM_setTimeBasePeriod(base, 0x1);
   EPWM_setTimeBaseCounterMode(base, EPWM_COUNTER_MODE_STOP_FREEZE);
   EPWM_clearTimeBaseCounterOverflowEvent(base);
   EPWM_setTimeBaseCounter(base, 0);
}

void util_setEPWMCC(uint32_t base, uint32_t CC_A, uint32_t CC_B)
{
    bool ret = false;

    EPWM_setCounterCompareValue(base, EPWM_COUNTER_COMPARE_A, CC_A);
    if((HW_RD_REG32(base + CSL_EPWM_CMPA)>>CSL_EPWM_CMPA_CMPA_SHIFT == (uint32_t)EPWM_getCounterCompareValue(base, EPWM_COUNTER_COMPARE_A))
    && (HW_RD_REG32(base + CSL_EPWM_CMPA)>>CSL_EPWM_CMPA_CMPA_SHIFT) == CC_A)
        ret = true;

    EPWM_setCounterCompareValue(base, EPWM_COUNTER_COMPARE_B, CC_B);
    if(HW_RD_REG32(base + CSL_EPWM_CMPB)>>CSL_EPWM_CMPB_CMPB_SHIFT == (uint32_t)EPWM_getCounterCompareValue(base, EPWM_COUNTER_COMPARE_B)
    && (HW_RD_REG32(base + CSL_EPWM_CMPB)>>CSL_EPWM_CMPB_CMPB_SHIFT) == CC_B)
        ret |= true;

    TEST_ASSERT_EQUAL_INT32(0x1, ret);
}

void util_resetEPWMCC(uint32_t base)
{
    EPWM_setCounterCompareValue(base, EPWM_COUNTER_COMPARE_A, 0x0);
    EPWM_setCounterCompareValue(base, EPWM_COUNTER_COMPARE_B, 0x0);
}

void util_resetEPWMAQ(uint32_t base)
{
    int i,j;

    for(i = 0; i< 2;i ++)
    {
        for(j = 0; j < 11; j+=2)
        {
            EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_A + 4*i, 0x0, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO + j);
        }
    }
}
void util_deinit_epwms(uint32_t i)
{
    //Unlock CONTROLSS_CTRL
    HW_WR_REG32(CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_LOCK0_KICK0 ,0x01234567);
    HW_WR_REG32(CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_LOCK0_KICK1 ,0xFEDCBA8);

    HW_WR_REG32(CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_ETPWM0_RST + i*4, 0x07);
    HW_WR_REG32(CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_ETPWM0_RST + i*4, 0x00);
}

void util_deinit_dac()
{

    //Unlock CONTROLSS_CTRL
    HW_WR_REG32(CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_LOCK0_KICK0 ,0x01234567);
    HW_WR_REG32(CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_LOCK0_KICK1 ,0xFEDCBA8);

    HW_WR_REG32(CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_DAC_RST ,0x07);
    HW_WR_REG32(CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_DAC_RST ,0x00);

}

uint32_t util_CMPSS_getInstanceFromBase(uint32_t base)
{
    switch(base)
    {
    case CSL_CONTROLSS_CMPSSA0_U_BASE:
            return 0;
    case CSL_CONTROLSS_CMPSSA1_U_BASE:
            return 1;
    case CSL_CONTROLSS_CMPSSA2_U_BASE:
            return 2;
    case CSL_CONTROLSS_CMPSSA3_U_BASE:
            return 3;
    case CSL_CONTROLSS_CMPSSA4_U_BASE:
            return 4;
    case CSL_CONTROLSS_CMPSSA5_U_BASE:
            return 5;
    case CSL_CONTROLSS_CMPSSA6_U_BASE:
            return 6;
    case CSL_CONTROLSS_CMPSSA7_U_BASE:
            return 7;
    case CSL_CONTROLSS_CMPSSA8_U_BASE:
            return 8;
    case CSL_CONTROLSS_CMPSSA9_U_BASE:
            return 9;

    case CSL_CONTROLSS_CMPSSB0_U_BASE:
            return 10;
    case CSL_CONTROLSS_CMPSSB1_U_BASE:
            return 11;
    case CSL_CONTROLSS_CMPSSB2_U_BASE:
            return 12;
    case CSL_CONTROLSS_CMPSSB3_U_BASE:
            return 13;
    case CSL_CONTROLSS_CMPSSB4_U_BASE:
            return 14;
    case CSL_CONTROLSS_CMPSSB5_U_BASE:
            return 15;
    case CSL_CONTROLSS_CMPSSB6_U_BASE:
            return 16;
    case CSL_CONTROLSS_CMPSSB7_U_BASE:
            return 17;
    case CSL_CONTROLSS_CMPSSB8_U_BASE:
            return 18;
    case CSL_CONTROLSS_CMPSSB9_U_BASE:
            return 19;

    default:
            return 0xFFFFFFFF;
    }

}

void util_deinit_cmpss(uint32_t base)
{
    /*Clear all registers*/
    HW_WR_REG16(base + CSL_CMPSSA_COMPCTL         , 0x0000);
    HW_WR_REG16(base + CSL_CMPSSA_COMPHYSCTL      , 0x0000);
    HW_WR_REG16(base + CSL_CMPSSA_COMPSTS         , 0x0000);
    HW_WR_REG16(base + CSL_CMPSSA_COMPSTSCLR      , 0x0202);
    HW_WR_REG16(base + CSL_CMPSSA_COMPDACCTL      , 0x0000);
    HW_WR_REG16(base + CSL_CMPSSA_COMPDACCTL2     , 0x0000);
    HW_WR_REG16(base + CSL_CMPSSA_DACHVALS        , 0x0000);
    HW_WR_REG16(base + CSL_CMPSSA_DACHVALA        , 0x0000);
    HW_WR_REG16(base + CSL_CMPSSA_RAMPMAXREFA     , 0x0000);
    HW_WR_REG16(base + CSL_CMPSSA_RAMPMAXREFS     , 0x0000);
    HW_WR_REG16(base + CSL_CMPSSA_RAMPDECVALA     , 0x0000);
    HW_WR_REG16(base + CSL_CMPSSA_RAMPDECVALS     , 0x0000);
    HW_WR_REG16(base + CSL_CMPSSA_RAMPSTS         , 0x0000);
    HW_WR_REG16(base + CSL_CMPSSA_DACLVALS        , 0x0000);
    HW_WR_REG16(base + CSL_CMPSSA_DACLVALA        , 0x0000);
    HW_WR_REG16(base + CSL_CMPSSA_RAMPDLYA        , 0x0000);
    HW_WR_REG16(base + CSL_CMPSSA_RAMPDLYS        , 0x0000);
    HW_WR_REG16(base + CSL_CMPSSA_CTRIPLFILCTL    , 0x0000);
    HW_WR_REG16(base + CSL_CMPSSA_CTRIPLFILCLKCTL , 0x0000);
    HW_WR_REG16(base + CSL_CMPSSA_CTRIPHFILCTL    , 0x0000);
    HW_WR_REG16(base + CSL_CMPSSA_CTRIPHFILCLKCTL , 0x0000);
    HW_WR_REG16(base + CSL_CMPSSA_COMPLOCK        , 0x0000);
    HW_WR_REG16(base + CSL_CMPSSA_DACHVALS2       , 0x0000);
    HW_WR_REG16(base + CSL_CMPSSA_DACLVALS2       , 0x0000);
    HW_WR_REG16(base + CSL_CMPSSA_CONFIG1         , 0x0000);

    /*Module reset*/

    HW_WR_REG32(CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_LOCK0_KICK0 ,0x01234567);
    HW_WR_REG32(CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_LOCK0_KICK1 ,0xFEDCBA8);

    uint8_t i = util_CMPSS_getInstanceFromBase(base);

    HW_WR_REG32(CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_CMPSSA0_RST + (i*4),0x07);
    HW_WR_REG32(CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_CMPSSA0_RST + (i*4),0x00);


}
/* configures the given adc instance with the ECAP trigger */
void util_adc_init_configure_soc_source(uint16_t adc_instance, uint16_t epwm_instance)
{
    uint32_t adc_base = CSL_CONTROLSS_ADC0_U_BASE;

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

    /* disabling and enabling the interrupt, setting the interrupt source as the epwm socA signal*/
    ADC_disableInterrupt(adc_base, ADC_INT_NUMBER1);
    ADC_enableInterrupt(adc_base, ADC_INT_NUMBER1);
    ADC_setInterruptSource(adc_base, ADC_INT_NUMBER1, ADC_SOC_NUMBER0);
    ADC_clearInterruptStatus(adc_base, ADC_INT_NUMBER1);
    ADC_clearInterruptOverflowStatus(adc_base, ADC_INT_NUMBER1);

    /* disabling and enabling the interrupt, setting the interrupt source as the epwm socB signal*/
    ADC_disableInterrupt(adc_base, ADC_INT_NUMBER2);
    ADC_enableInterrupt(adc_base, ADC_INT_NUMBER2);
    ADC_setInterruptSource(adc_base, ADC_INT_NUMBER2, ADC_SOC_NUMBER1);
    ADC_clearInterruptStatus(adc_base, ADC_INT_NUMBER2);
    ADC_clearInterruptOverflowStatus(adc_base, ADC_INT_NUMBER2);

    /* Setting up the SOC configuration for the given ecap_instance SOC signal.*/
    uint16_t epwm_socA_trigger = ADC_TRIGGER_EPWM0_SOCA + (2*epwm_instance);
    ADC_setupSOC(adc_base, ADC_SOC_NUMBER0, epwm_socA_trigger, ADC_CH_ADCIN0, 16);

    uint16_t epwm_socB_trigger = ADC_TRIGGER_EPWM0_SOCA + (2*epwm_instance+1);
    ADC_setupSOC(adc_base, ADC_SOC_NUMBER1, epwm_socB_trigger, ADC_CH_ADCIN0, 16);

    return ;
}
/* resets the ADC instance */
void util_adc_reset(uint16_t adc_instance)
{
    SOC_generateAdcReset(adc_instance);
    return;
}
void util_EPWM_setup_adc_trigger(uint32_t epwm_base, EPWM_ADCStartOfConversionType adc_soc_type,
                                 uint32_t trigger_source, uint16_t prescale)
{
    //Clearing the previous selection

    EPWM_setADCTriggerSource(epwm_base, adc_soc_type, 0, 0);
    /* ADC trigger is set at counter == compare C while incrementing*/
    if(trigger_source != EPWM_SOC_TBCTR_MIXED_EVENT)
        {
            EPWM_setADCTriggerSource(epwm_base, adc_soc_type, trigger_source, 0);
        }
    else
        {
            EPWM_setADCTriggerSource(epwm_base, adc_soc_type, EPWM_SOC_TBCTR_MIXED_EVENT, 0x3FF);
        }
    /* enabling the ADC soc trigger from EPWM */
    EPWM_enableADCTrigger( epwm_base, adc_soc_type);

    EPWM_setADCTriggerEventPrescale(epwm_base, adc_soc_type, prescale);
	EPWM_enableADCTriggerEventCountInit(epwm_base, adc_soc_type);
	EPWM_setADCTriggerEventCountInitValue(epwm_base, adc_soc_type, 0);

    return;
}

/* Testcase 1 - Check the EPWM_setClockPrescaler API */
static void EPWM_setClockPrescalerApiCheck(void *args)
{
    /* Call the EPWM_setClockPrescaler API */
    EPWM_setClockPrescaler(CSL_CONTROLSS_G0_EPWM0_U_BASE, EPWM_CLOCK_DIVIDER_4, EPWM_HSCLOCK_DIVIDER_6);

    /* Check if the value was written correctly */
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG16(CSL_CONTROLSS_G0_EPWM0_U_BASE + CSL_EPWM_TBCTL)
        & CSL_EPWM_TBCTL_CLKDIV_MASK) >> CSL_EPWM_TBCTL_CLKDIV_SHIFT, EPWM_CLOCK_DIVIDER_4);

    TEST_ASSERT_EQUAL_INT32((HW_RD_REG16(CSL_CONTROLSS_G0_EPWM0_U_BASE + CSL_EPWM_TBCTL)
        & CSL_EPWM_TBCTL_HSPCLKDIV_MASK) >> CSL_EPWM_TBCTL_HSPCLKDIV_SHIFT, EPWM_HSCLOCK_DIVIDER_6);

    //*************************************De-initializing*************************************//
        EPWM_setClockPrescaler(CSL_CONTROLSS_G0_EPWM0_U_BASE, 0x0, 0x1);
    //****************************************************************************************//
}

/* Testcase 2 - Check the EPWM_setRisingEdgeDelayCount and EPWM_setFallingEdgeDelayCount API */
static void EPWM_setRFEdgeDelayCountApiCheck(void *args)
{
    EPWM_setRisingEdgeDelayCount(CSL_CONTROLSS_G0_EPWM0_U_BASE, 0x0020);
    /*Value of second parameter should be less than 0x4000U*/
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG16(CSL_CONTROLSS_G0_EPWM0_U_BASE + CSL_EPWM_DBRED)), 0x0020);

    EPWM_setFallingEdgeDelayCount(CSL_CONTROLSS_G0_EPWM0_U_BASE, 0x0040);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG16(CSL_CONTROLSS_G0_EPWM0_U_BASE + CSL_EPWM_DBFED)), 0x0040);

    /*************************************De-initializing*************************************/
        EPWM_setRisingEdgeDelayCount(CSL_CONTROLSS_G0_EPWM0_U_BASE, 0x0000);
        EPWM_setFallingEdgeDelayCount(CSL_CONTROLSS_G0_EPWM0_U_BASE, 0x0000);
    /****************************************************************************************/
}

static void EPWM_complementary_pwm(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_epwm_cases(1), 0);
}
static void EPWM_pwm_clk_sync(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_epwm_cases(2), 0);
}
static void EPWM_hrpwm_operation(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_epwm_cases(3), 0);
}
static void EPWM_hrpwm_operation_with_sfo(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_epwm_cases(4), 0);
}
static void EPWM_pwm_link_feature(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_epwm_cases(5), 0);
}
static void EPWM_pwm_chopper_operation(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_epwm_cases(6), 0);
}
static void EPWM_all_pwm_working_together(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_epwm_cases(7), 0);
}
static void EPWM_dac_load_upon_pwm_event(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_epwm_cases(8), 0);
}
static void EPWM_cmpss_load_upon_pwm_event(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_epwm_cases(9), 0);
}
static void EPWM_gpio_signal_trips_pwm(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_epwm_cases(10), 0);
}
static void EPWM_emustop_trip_pwm(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_epwm_cases(11), 0);
}
static void EPWM_cmpss_to_trip_pwm(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_epwm_cases(12), 0);
}
static void EPWM_syserror_trips_pwm(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_epwm_cases(13), 0);
}
static void EPWM_deadband_basic(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_epwm_cases(14), 0);
}
static void EPWM_diode_emulation_with_cmpss(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_epwm_cases(15), 0);
}
static void EPWM_adc_conversion_with_pwm_event(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_epwm_cases(16), 0);
}
static void EPWM_diode_emulation_and_mindb(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_epwm_cases(17), 0);
}
static void EPWM_sdfm_trips_pwm(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_epwm_cases(18), 0);
}
static void EPWM_illegal_combo_logic(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_epwm_cases(19), 0);
}
static void EPWM_epwmxlinkxload_feature(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_epwm_cases(20), 0);
}
static void EPWM_hrpwm_calibration(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_epwm_cases(21), 0);
}
static void EPWM_hrpwm_functionality(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_epwm_cases(22), 0);
}
static void EPWM_hrpwm_sfo_function_usage(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_epwm_cases(23), 0);
}
static void EPWM_pwm_latency_basic(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_epwm_cases(24), 0);
}
static void EPWM_pwm_latency_through_r5f_cores(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_epwm_cases(25), 0);
}
static void EPWM_pwm_latency_through_dma(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_epwm_cases(26), 0);
}
static void EPWM_latency_of_fast_access_bridge_registers_through_r5f(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_epwm_cases(27), 0);
}
static void EPWM_latency_of_fast_access_bridge_shadow_registers_through_r5f(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_epwm_cases(28), 0);
}
static void EPWM_latency_of_fast_access_bridge_registers_through_dma(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_epwm_cases(29), 0);
}
static void EPWM_latency_of_fast_access_bridge_shadow_registers_through_dma(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_epwm_cases(30), 0);
}
static void EPWM_latency_of_fast_access_bridge_hrpwm_registers_through_r5f(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_epwm_cases(31), 0);
}
static void EPWM_support_pwm_chopping(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_epwm_cases(32), 0);
}
static void EPWM_add_additional_syncin_sources(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_epwm_cases(33), 0);
}
static void EPWM_functionalities_for_mdl_and_icl(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_epwm_cases(34), 0);
}
static void EPWM_dc_cbc_latch(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_epwm_cases(35), 0);
}
static void EPWM_set_trip_zone_action(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_epwm_cases(36), 0);
}
static void EPWM_generation_of_adc_soc_and_cpu_interrupt_on_all_events(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_epwm_cases(37), 0);
}

static void EPWM_control_phase_shifting(void *args)
{
    /*Test the EPWM_setPhaseShift API */
    /*[MCUSDK-3618] AM263x: ePWM_setPhaseShift() API function doesn't correctly configure TBPHS */
    EPWM_setPhaseShift(CSL_CONTROLSS_G0_EPWM0_U_BASE, 0xFFFFU);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG32(CSL_CONTROLSS_G0_EPWM0_U_BASE + CSL_EPWM_TBPHS)
    & CSL_EPWM_TBPHS_TBPHS_MASK) >> CSL_EPWM_TBPHS_TBPHS_SHIFT, CSL_EPWM_TBPHS_TBPHS_MAX);

    /*************************************De-initializing*************************************/
       EPWM_setPhaseShift(CSL_CONTROLSS_G0_EPWM0_U_BASE, 0x0000U);
    /****************************************************************************************/
}
static void EPWM_generating_pwm_signals_using_16_bit_counter(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_epwm_cases(38), 0);
}
static void EPWM_programming_event_prescaling(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_epwm_cases(39), 0);
}
static void EPWM_one_shot_sync_out_trigger(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_epwm_cases(40), 0);
}
static void EPWM_support_deadband_generation(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_epwm_cases(41), 0);
}
static void EPWM_support_asychronous_override_control_of_pwm_signals_through_software(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_epwm_cases(42), 0);
}
static void EPWM_support_generation_of_2_pwm_outputs_with_dual_edge_symmetric_operation(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_epwm_cases(43), 0);
}
static void EPWM_select_one_shot_sync_out_trigger(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_epwm_cases(44), 0);
}
static void EPWM_grouping_aliasing_of_epwm_instances(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_epwm_cases(45), 0);
}
static void EPWM_XCMP(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_epwm_cases(46), 0);
}
static void EPWM_DEL(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_epwm_cases(47), 0);
}
static void EPWM_checkAPIs(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_epwm_cases(48), 0);
}
// EPWM_complementary_pwm
int32_t AM263x_EPWM_xTR_0001(uint32_t base, uint32_t i)
{
    /************************************* SETTING UP TESTER *************************************/
    char epwmA[CMD_SIZE];
    char epwmB[CMD_SIZE];
    char cap_epwmA[CMD_SIZE];
    char cap_epwmB[CMD_SIZE];
    char ts_epwmA[CMD_SIZE];
    char ts_epwmB[CMD_SIZE];

    if(i<10)
        {
            sprintf(epwmA, "setup epwm 0%u A", i);
            sprintf(epwmB, "setup epwm 0%u B", i);
            sprintf(cap_epwmA, "capture epwm 0%u A ", i);
            sprintf(cap_epwmB, "capture epwm 0%u B ", i);
            sprintf(ts_epwmA, "get timestamp 0%u A", i);
            sprintf(ts_epwmB, "get timestamp 0%u B", i);
        }
    else
        {
            sprintf(epwmA, "setup epwm %u A", i);
            sprintf(epwmB, "setup epwm %u B", i);
            sprintf(cap_epwmA, "capture epwm %u A ", i);
            sprintf(cap_epwmB, "capture epwm %u B ", i);
            sprintf(ts_epwmA, "get timestamp %u A", i);
            sprintf(ts_epwmB, "get timestamp %u B", i);
        }

    tester_command(epwmA);
    strcat(cap_epwmA, "continuously on edges FRFR in DELTA");
    tester_command(cap_epwmA);

    tester_command(epwmB);
    strcat(cap_epwmB, "continuously on edges FRFR in DELTA");
    tester_command(cap_epwmB);

    /****************************************************************************************/

    /* Configure Time Base */
    /* Configure Counter Compare */
    /* Configure Action Qualifier Events */
    uint8_t itr1, itr2, itr3;
    int32_t error=0;

    for(itr1 = 0; itr1< 3; itr1++) /* time base period */
        {
            for( itr2 = 0 ;itr2 < 3; itr2++) /* count mode */
            {
                for (itr3 = 0; itr3< 3; itr3++)  /* counter compare */
                {
                    if (tb_clk_vector[itr1][2] == cc_values_vector[itr3])
                        continue;
                    util_setEPWMTB(base, EPWM_EMULATION_FREE_RUN, tb_clk_vector[itr1][0], tb_clk_vector[itr1][1], tb_clk_vector[itr1][2], tb_countmode_vector[itr2]);
                    util_setEPWMCC(base, cc_values_vector[itr3], cc_values_vector[itr3]);
                    switch (itr2)
                    {
                        case 0:  /* Up count */
                            EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
                            EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
                            EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
                            EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
                            break;
                        case 1:  /* Down count */
                            EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
                            EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);
                            EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
                            EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
                            break;
                        case 2:  /* Up Down count */
                            EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
                            EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);
                            EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
                            EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
                            break;
                    }

                    ClockP_usleep(10000);

                   /************************************* VALIDATING FROM TESTER *************************************/
                    /*int32_t timestamp_event1_A=0; */
                    int32_t timestamp_event2_A=0;
                    int32_t timestamp_event3_A=0;
                    /*int32_t timestamp_event4_A=0; */

                    /* int32_t timestamp_event1_B=0; */
                    /* int32_t timestamp_event2_B=0; */
                    int32_t timestamp_event3_B=0;
                    int32_t timestamp_event4_B=0;

                    tester_command(ts_epwmA);
                    /*timestamp_event1_A = ts[0]; */ /* Width of high pulse of ePWMxA */
                    timestamp_event2_A = ts[1];   /* Width of low pulse of ePWMxA */
                    timestamp_event3_A = ts[2];   /* Width of high pulse of ePWMxA */
                    /*timestamp_event4_A = ts[3]; */ /* Width of low pulse of ePWMxA */

                    tester_command(ts_epwmB);
                    /*timestamp_event1_B = ts[0]; */ /* Width of high pulse of ePWMxB */
                    /*timestamp_event2_B = ts[1]; */ /* Width of low pulse of ePWMxB */
                    timestamp_event3_B = ts[2]; /* Width of high pulse of ePWMxB */
                    timestamp_event4_B = ts[3]; /* Width of low pulse of ePWMxB */

                    /*Validating if the timestamp captured by ecap is correct*/
                    uint8_t multiplier = 1;
                    if(itr2 == 2) multiplier = 2;

                    int32_t low_pulse_width = (1<<tb_clk_vector[itr1][0]) * (1<<tb_clk_vector[itr1][1]) * (cc_values_vector[itr3]) * multiplier;
                    int32_t high_pulse_width = (1<<tb_clk_vector[itr1][0]) * (1<<tb_clk_vector[itr1][1]) * (tb_clk_vector[itr1][2]- cc_values_vector[itr3]) * multiplier;
                    int32_t high_cap_diff=0, low_cap_diff=0;
                    low_cap_diff=abs(timestamp_event2_A-low_pulse_width);
                    high_cap_diff=abs(timestamp_event3_A-high_pulse_width);

                    if(enableLog)
                    {
                        DebugP_log("%d - %d, %d - %d\r\n",timestamp_event2_A,low_pulse_width, timestamp_event3_A, high_pulse_width);
                    }
                    if( (low_cap_diff >20 ) || (high_cap_diff  >20 ))
                    {
                        error++;
                        break;
                    }

                    /*Validating if the pulse generated by 2 output channels are complementary */
                    int32_t diff1= abs(timestamp_event2_A-timestamp_event3_B);  /*low pulse width of channel A should be equal to high pulse width of channel B */
                    int32_t diff2= abs(timestamp_event3_A-timestamp_event4_B);  /*high pulse width of channel A should be equal to low pulse width of channel B */

                    if( (diff1 >20 ) || (diff2 >20 ))
                    {
                        error++;
                    }

                    /****************************************************************************************/
                }
            }
        }
    //*************************************De-initializing*************************************//
        /*util_deinit_epwms(i);*/
        util_resetEPWMTB(base);
        util_resetEPWMCC(base);
        util_resetEPWMAQ(base);
    //****************************************************************************************//

    if(error==0)
    {
        if(enableLog)
        {
            DebugP_log("\r\nPass");
        }
        return 0;
    }
    else
    {
        if(enableLog)
        {
            DebugP_log("\r\nFail");
        }

        return 1;
    }
}

/*  EPWM_pwm_clk_sync */
int32_t AM263x_EPWM_xTR_0002(uint32_t dummy)
{
    /************************************* SETTING UP TESTER *************************************/
    uint32_t tbprd = 0x5DC0, cmpA = 0x4650, cmpB = 0x0001;
    /****************************************************************************************/

    /* EPWMx takes sync-in pulse from EPWMy so their TBCTR starts counting at the same time */
    uint8_t i;
    int32_t error=0;

    for(i = 1 ; i< 5; i++)
    {
        uint8_t tmp = i;
        if(i > 2) tmp-=2;   //FIXME for other instances
        /******************************/
        char epwmA[CMD_SIZE];
        char cap_epwmA[CMD_SIZE];
        char ts_epwmx[CMD_SIZE];
        char ts_epwmy[CMD_SIZE];
        /******************************/

        uint32_t base = CSL_CONTROLSS_G0_EPWM0_U_BASE + tmp * 0x1000;

        util_setEPWMTB(base, EPWM_EMULATION_FREE_RUN, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1, tbprd, EPWM_COUNTER_MODE_STOP_FREEZE);
        util_setEPWMCC(base, cmpA, cmpB);
        EPWM_enableSyncOutPulseSource(base, EPWM_SYNC_OUT_PULSE_ON_CNTR_ZERO);
        EPWM_enablePhaseShiftLoad(base);
        EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);

        /*Since all EPWM have same config till now, once they are synced with each other, their rising edge should occur at the same time */
        switch(i)
        {
            case 1:
                EPWM_setTimeBaseCounterMode(base, EPWM_COUNTER_MODE_UP_DOWN);
                EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);
                break;
            case 2:
                EPWM_setTimeBaseCounterMode(base, EPWM_COUNTER_MODE_UP_DOWN);
                EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);
                EPWM_setSyncInPulseSource(base, EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM1);  /* EPWM2 (Up down mode) takes syncin pulse from EPWM1 (Up down mode) */
                break;
            case 3:
                EPWM_setTimeBaseCounterMode(base, EPWM_COUNTER_MODE_UP);
                EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
                break;
            case 4:
                EPWM_setTimeBaseCounterMode(base, EPWM_COUNTER_MODE_UP);
                EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
                EPWM_setSyncInPulseSource(base, EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM1); /* EPWM2 (Up mode) takes syncin pulse from EPWM1 (Up mode) */
                break;
        }

        //******************************//

        sprintf(epwmA, "setup epwm 0%u A", tmp);
        tester_command(epwmA);
        if(tmp == 2)  /* The EPWM instances which send out the sync pulse */   /* FIXME for other instances */
        {
            Configure_Xbar_Sync_DUT(1); /* The epwm which syncs the other epwm is used to sync the ecap on */
            tester_command("sync with epwm 01 A");
            tester_command("sync with epwm 02 A");
            sprintf(cap_epwmA, "capture epwm 01 A continuously on edges FFRF in ABSOLUTE");
            tester_command(cap_epwmA);
            sprintf(cap_epwmA, "capture epwm 02 A continuously on edges FFRF in ABSOLUTE");
            tester_command(cap_epwmA);
        }
        ClockP_usleep(10000);

        //************************************* VALIDATING FROM TESTER *************************************//
        if(i%2) /* Validate only after the next EPWM instance gets synced by the current EPWM instance */
            continue;

        sprintf(ts_epwmx, "get timestamp 01 A");
        sprintf(ts_epwmy, "get timestamp 02 A");

        /* if(enableLog)
        {
            DebugP_log("EPWM_%d_A to ECAP%d ; EPWM_%d_B to ECAP %d\n",itr3,epwm_to_ecap[2*itr3], itr3, epwm_to_ecap[2*itr3+1]);
        }*/

        /* int32_t timestamp_event1_X=0; */
        int32_t timestamp_event2_X=0;
        int32_t timestamp_event3_X=0;
        int32_t timestamp_event4_X=0;

        /* int32_t timestamp_event1_Y=0; */
        int32_t timestamp_event2_Y=0;
        int32_t timestamp_event3_Y=0;
        int32_t timestamp_event4_Y=0;

        tester_command(ts_epwmx);
        /* timestamp_event1_X = ts[0]; */
        timestamp_event2_X = ts[1];
        timestamp_event3_X = ts[2];
        timestamp_event4_X = ts[3];

        tester_command(ts_epwmy);
        /* timestamp_event1_Y = ts[0]; */
        timestamp_event2_Y = ts[1];
        timestamp_event3_Y = ts[2];
        timestamp_event4_Y = ts[3];

        /* Validating if the timestamp captured by ecap is correct */

        /*The capture starts from TBCTR = 0 and ends on getting the falling edge so ts[1] is ~period value.
        Again the capture ecap starts counting from 0 on the start of next cycle as this epwm syncs ecap on TBCTR=0.
            ts[2] occurs at Rising edge, so low pulse width = ts[2]
            ts[3] occurs at Falling edge, since it's in absolute mode, ts[3] - ts[2] = high pulse width
        */
        int32_t expected_low_pulse_width = (1<<EPWM_CLOCK_DIVIDER_1) * (1<<EPWM_HSCLOCK_DIVIDER_1) * (cmpA);
        int32_t low_pulse_width = timestamp_event3_X;
        int32_t diff_low_pulse_width = abs(low_pulse_width - expected_low_pulse_width);

        int32_t expected_high_pulse_width;
        if( i == 2 )  /* UP_DOWN */
            expected_high_pulse_width= (1<<EPWM_CLOCK_DIVIDER_1) * (1<<EPWM_HSCLOCK_DIVIDER_1) * ((tbprd-cmpA) + (tbprd-cmpB)) ;
        else         /* UP count */
            expected_high_pulse_width = (1<<EPWM_CLOCK_DIVIDER_1) * (1<<EPWM_HSCLOCK_DIVIDER_1) * ((tbprd-cmpA));
        int32_t high_pulse_width = timestamp_event4_X - timestamp_event3_X;
        int32_t diff_high_pulse_width = abs(high_pulse_width - expected_high_pulse_width);

        /* To check if both the EPWM are in sync, the ts[1] in both have to be same as the capturing of both starts at the same time
            as the ecap instances capturing the timestamps are synced by EPWM1 and stops capturing on encountering the falling edge
            that is stored in ts[1]. The first timestamp i.e. ts[0] is ignored.
        */
        int32_t timestamp_diff = abs(timestamp_event2_X - timestamp_event2_Y);

        if(enableLog) {
            DebugP_log("%u - %u, %u - %u, %u - %u\r\n",timestamp_event2_X, timestamp_event2_Y, low_pulse_width, expected_low_pulse_width,
        high_pulse_width, expected_high_pulse_width);
            DebugP_log("\r\n On EPWM 1\r\n");
            DebugP_log("Timestamp captured for low pulse is %u and expected value is %u\r\n", timestamp_event3_X, expected_low_pulse_width);
            DebugP_log("Timestamp captured for high pulse is %u and expected value is %u\r\n", high_pulse_width, expected_high_pulse_width);
            DebugP_log("Absolute timestamp from TBCTR = 0  to falling edge is %u\n\r\n", timestamp_event2_X);
            DebugP_log("On EPWM 2\r\n");
            DebugP_log("Timestamp captured for low pulse is %u and expected value is %u\r\n", timestamp_event3_Y, expected_low_pulse_width);
            DebugP_log("Timestamp captured for high pulse is %u and expected value is %u\r\n", (timestamp_event4_Y - timestamp_event3_Y), expected_high_pulse_width);
            DebugP_log("Absolute timestamp from TBCTR = 0  to falling edge is %u\r\n", timestamp_event2_Y);
        }


        if( (diff_low_pulse_width > 10 ) || (diff_high_pulse_width  >10 ) || (timestamp_diff > 10))
        {
            error++;
        }

        //*************************************De-initializing*************************************//
        if(tmp ==2)
        {
            util_deinit_epwms(1);
            util_deinit_epwms(2);
        }
        for(int instance= 1;instance < 3; instance++)
        {
            uint32_t epwm_base = CSL_CONTROLSS_G0_EPWM0_U_BASE + instance * 0x1000;
            util_resetEPWMTB(epwm_base);
            util_resetEPWMCC(epwm_base);
            util_resetEPWMAQ(epwm_base);
            EPWM_disableSyncOutPulseSource(epwm_base, EPWM_SYNC_OUT_PULSE_ON_CNTR_ZERO);
            EPWM_disablePhaseShiftLoad(epwm_base);
        }
        //****************************************************************************************//
        ClockP_usleep(1000);
    }
        /****************************************************************************************/


    if(error==0)
    {
        if(enableLog)
        {
            DebugP_log("\r\nPass");
        }

        return 0;
    }
    else
    {
        if(enableLog)
        {
            DebugP_log("\r\nFail");
        }
        return 1;
    }
}

 /* EPWM_hrpwm_operation */
int32_t AM263x_EPWM_xTR_0003(uint32_t base, uint32_t i)
{
    //TBD
    int32_t error=0;

    HRPWM_enableAutoConversion(base);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG16(base + CSL_EPWM_HRCNFG)
    & CSL_EPWM_HRCNFG_AUTOCONV_MASK) >> CSL_EPWM_HRCNFG_AUTOCONV_SHIFT , 0x1);

    HRPWM_setMEPControlMode(base, HRPWM_CHANNEL_A, HRPWM_MEP_DUTY_PERIOD_CTRL);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG16(base + CSL_EPWM_HRCNFG)
    & CSL_EPWM_HRCNFG_CTLMODE_MASK) >> CSL_EPWM_HRCNFG_CTLMODE_SHIFT , HRPWM_MEP_DUTY_PERIOD_CTRL);

    HRPWM_setMEPControlMode(base, HRPWM_CHANNEL_B, HRPWM_MEP_DUTY_PERIOD_CTRL);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG16(base + CSL_EPWM_HRCNFG)
    & CSL_EPWM_HRCNFG_CTLMODEB_MASK) >> CSL_EPWM_HRCNFG_CTLMODEB_SHIFT , HRPWM_MEP_DUTY_PERIOD_CTRL);

    HRPWM_setHiResPhaseShift(base, 45U);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG32(base + CSL_EPWM_TBPHS)
    & CSL_EPWM_TBPHS_TBPHSHR_MASK) >> (CSL_EPWM_TBPHS_TBPHSHR_SHIFT + 8U) , 45U);

    HRPWM_setSyncPulseSource(base, HRPWM_PWMSYNC_SOURCE_PERIOD);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG16(base + CSL_EPWM_HRPCTL)
    & (CSL_EPWM_HRPCTL_PWMSYNCSELX_MASK | CSL_EPWM_HRPCTL_PWMSYNCSEL_MASK)) >> 0x1 , HRPWM_PWMSYNC_SOURCE_PERIOD);

    HRPWM_enablePhaseShiftLoad(base);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG16(base + CSL_EPWM_HRPCTL)
    & CSL_EPWM_HRPCTL_TBPHSHRLOADE_MASK) >> CSL_EPWM_HRPCTL_TBPHSHRLOADE_SHIFT , 0x1);

    HRPWM_setMEPEdgeSelect(base, HRPWM_CHANNEL_A, HRPWM_MEP_CTRL_RISING_EDGE);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG16(base + CSL_EPWM_HRCNFG)
    & CSL_EPWM_HRCNFG_EDGMODE_MASK) >> CSL_EPWM_HRCNFG_EDGMODE_SHIFT , HRPWM_MEP_CTRL_RISING_EDGE);

    HRPWM_setMEPEdgeSelect(base, HRPWM_CHANNEL_B, HRPWM_MEP_CTRL_FALLING_EDGE);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG16(base + CSL_EPWM_HRCNFG)
    & CSL_EPWM_HRCNFG_EDGMODEB_MASK) >> CSL_EPWM_HRCNFG_EDGMODEB_SHIFT , HRPWM_MEP_CTRL_FALLING_EDGE);

    for(uint16_t hiResVal = 0; hiResVal < 0xFF; hiResVal ++)
    {
        HRPWM_setHiResCounterCompareValue(base, HRPWM_COUNTER_COMPARE_A, hiResVal);
        TEST_ASSERT_EQUAL_INT32((HW_RD_REG32(base + CSL_EPWM_CMPA)
        & CSL_EPWM_CMPA_CMPAHR_MASK) >> (CSL_EPWM_CMPA_CMPAHR_SHIFT + 8U) ,
        HRPWM_getHiResCounterCompareValueOnly(base, HRPWM_COUNTER_COMPARE_A));

        if(HRPWM_getHiResCounterCompareValueOnly(base, HRPWM_COUNTER_COMPARE_A) != hiResVal)
            error++;
    }
    for(uint16_t hiResVal = 0; hiResVal < 0xFF; hiResVal ++)
    {
        HRPWM_setHiResCounterCompareValue(base, HRPWM_COUNTER_COMPARE_B, hiResVal);
        TEST_ASSERT_EQUAL_INT32((HW_RD_REG32(base + CSL_EPWM_CMPB)
        & CSL_EPWM_CMPB_CMPBHR_MASK) >> (CSL_EPWM_CMPB_CMPBHR_SHIFT + 8U) ,
        HRPWM_getHiResCounterCompareValueOnly(base, HRPWM_COUNTER_COMPARE_B));

        if(HRPWM_getHiResCounterCompareValueOnly(base, HRPWM_COUNTER_COMPARE_B) != hiResVal)
            error++;
    }

    HRPWM_setCounterCompareShadowLoadEvent(base, HRPWM_CHANNEL_A, HRPWM_LOAD_ON_CNTR_PERIOD);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG16(base + CSL_EPWM_HRCNFG)
    & CSL_EPWM_HRCNFG_HRLOAD_MASK) >> CSL_EPWM_HRCNFG_HRLOAD_SHIFT , HRPWM_LOAD_ON_CNTR_PERIOD);

    HRPWM_setCounterCompareShadowLoadEvent(base, HRPWM_CHANNEL_B, HRPWM_LOAD_ON_CNTR_ZERO_PERIOD);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG16(base + CSL_EPWM_HRCNFG)
    & CSL_EPWM_HRCNFG_HRLOADB_MASK) >> CSL_EPWM_HRCNFG_HRLOADB_SHIFT , HRPWM_LOAD_ON_CNTR_ZERO_PERIOD);

    HRPWM_enablePeriodControl(base);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG16(base + CSL_EPWM_HRPCTL)
    & CSL_EPWM_HRPCTL_HRPE_MASK) >> CSL_EPWM_HRPCTL_HRPE_SHIFT, 0x1);

    HRPWM_setHiResTimeBasePeriod(base, 0x2E);
    TEST_ASSERT_EQUAL_INT32(HW_RD_REG16(base + CSL_EPWM_TBPRDHR) >> 8U,
    HRPWM_getHiResTimeBasePeriod(base));

    if(HRPWM_getHiResTimeBasePeriod(base) != 0x2E)
        error++;

    HRPWM_setDeadbandMEPEdgeSelect(base, HRPWM_DB_MEP_CTRL_RED_FED);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG16(base + CSL_EPWM_HRCNFG2)
    & CSL_EPWM_HRCNFG2_EDGMODEDB_MASK) >> CSL_EPWM_HRCNFG2_EDGMODEDB_SHIFT, HRPWM_DB_MEP_CTRL_RED_FED);

    HRPWM_setHiResRisingEdgeDelay(base, 50);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG16(base + CSL_EPWM_HRCNFG2)
    & CSL_EPWM_HRCNFG2_EDGMODEDB_MASK) >> CSL_EPWM_HRCNFG2_EDGMODEDB_SHIFT, HRPWM_DB_MEP_CTRL_RED_FED);

    HRPWM_setRisingEdgeDelayLoadMode(base, HRPWM_LOAD_ON_CNTR_ZERO);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG16(base + CSL_EPWM_HRCNFG2)
    & CSL_EPWM_HRCNFG2_CTLMODEDBRED_MASK) >> CSL_EPWM_HRCNFG2_CTLMODEDBRED_SHIFT, HRPWM_LOAD_ON_CNTR_ZERO);

    HRPWM_setHiResFallingEdgeDelayOnly(base, 100);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG16(base + CSL_EPWM_DBFEDHR)
    & CSL_EPWM_DBFEDHR_DBFEDHR_MASK) >> CSL_EPWM_DBFEDHR_DBFEDHR_SHIFT, 100);

    HRPWM_setFallingEdgeDelayLoadMode(base, HRPWM_LOAD_ON_CNTR_PERIOD);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG16(base + CSL_EPWM_HRCNFG2)
    & CSL_EPWM_HRCNFG2_CTLMODEDBFED_MASK) >> CSL_EPWM_HRCNFG2_CTLMODEDBFED_SHIFT, HRPWM_LOAD_ON_CNTR_PERIOD);

    HRPWM_setOutputSwapMode(base, true);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG16(base + CSL_EPWM_HRCNFG)
    & CSL_EPWM_HRCNFG_SWAPAB_MASK) >> CSL_EPWM_HRCNFG_SWAPAB_SHIFT, 0x1);

    HRPWM_setChannelBOutputPath(base, HRPWM_OUTPUT_ON_B_INV_A);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG16(base + CSL_EPWM_HRCNFG)
    & CSL_EPWM_HRCNFG_SELOUTB_MASK) >> CSL_EPWM_HRCNFG_SELOUTB_SHIFT, HRPWM_OUTPUT_ON_B_INV_A);

    HRPWM_setPhaseShift(base, 0xABCDEF);
    TEST_ASSERT_EQUAL_INT32(HW_RD_REG32(base + CSL_EPWM_TBPHS)>>8U, 0xABCDEF);

    HRPWM_setCounterCompareValue(base, HRPWM_COUNTER_COMPARE_A, 0xB464);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG32(base + CSL_EPWM_CMPA)) >> 8U,
    HRPWM_getCounterCompareValue(base, HRPWM_COUNTER_COMPARE_A));

    if(HRPWM_getCounterCompareValue(base, HRPWM_COUNTER_COMPARE_A) != 0xB464)
        error++;

    //*************************************De-initializing*************************************//
        /*util_deinit_epwms(i);*/
    HRPWM_disableAutoConversion(base);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG16(base + CSL_EPWM_HRCNFG)
    & CSL_EPWM_HRCNFG_AUTOCONV_MASK) >> CSL_EPWM_HRCNFG_AUTOCONV_SHIFT , 0x0);

    HRPWM_disablePhaseShiftLoad(base);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG16(base + CSL_EPWM_HRPCTL)
    & CSL_EPWM_HRPCTL_TBPHSHRLOADE_MASK) >> CSL_EPWM_HRPCTL_TBPHSHRLOADE_SHIFT , 0x0);

    HRPWM_disablePeriodControl(base);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG16(base + CSL_EPWM_HRPCTL)
    & CSL_EPWM_HRPCTL_HRPE_MASK) >> CSL_EPWM_HRPCTL_HRPE_SHIFT, 0x0);
    //****************************************************************************************//
    if(error==0)
    {
        if(enableLog)
        {
            DebugP_log("\r\nPass");
        }
        return 0;
    }
    else
    {
        if(enableLog)
        {
            DebugP_log("\r\nFail");
        }
        return 1;
    }
}

/*  EPWM_pwm_link_feature */
int8_t AM263x_EPWM_xTR_0005(uint32_t dummy)
{
    /* Facilitates simultaneous writes to linked EPWM modules */
    uint8_t i;
    int32_t error=0;

    for(i = 0 ; i< 31; i++)
    {
        uint32_t base = CSL_CONTROLSS_G0_EPWM0_U_BASE + i * 0x1000;

        if(!(i%2))
        {
            /* EPWM0 links to EPWM1, EPWM2 links to EPWM3 and EPWM4 links to EPWM5 and so on.. */
            EPWM_setupEPWMLinks(base, i+1, EPWM_LINK_TBPRD);
            EPWM_setupEPWMLinks(base, i+1, EPWM_LINK_COMP_A);
            EPWM_setupEPWMLinks(base, i+1, EPWM_LINK_COMP_B);
        }
        else
        {
            /* Calling these APIs will simultaneously write to corresponding regsiters of the EPWM it is linked to.
            Eg: EPWM0_TBPRD will get 0xFEDC as it's linked to EPWM1. */
            EPWM_setTimeBasePeriod(base, 0xFEDC);
            EPWM_setCounterCompareValue(base, EPWM_COUNTER_COMPARE_A, 0xABCD);
            EPWM_setCounterCompareValue(base, EPWM_COUNTER_COMPARE_B, 0x9876);

            TEST_ASSERT_EQUAL_INT32(HW_RD_REG16(base - 0x1000 + CSL_EPWM_TBPRD), 0xFEDC);
            TEST_ASSERT_EQUAL_INT32((HW_RD_REG32(base - 0x1000 + CSL_EPWM_CMPA)
            & 0xFFFF0000) >> 0x10 , 0xABCD);
            TEST_ASSERT_EQUAL_INT32((HW_RD_REG32(base - 0x1000 + CSL_EPWM_CMPB)
            & 0xFFFF0000) >> 0x10 , 0x9876);
        }
    }
    //*************************************De-initializing*************************************//

    for(i=0; i<32 ; i++)
    {
        /*util_deinit_epwms(i);*/
        uint32_t base = CSL_CONTROLSS_G0_EPWM0_U_BASE + i * 0x1000;
        util_resetEPWMTB(base);
        util_resetEPWMCC(base);
        util_resetEPWMAQ(base);
        EPWM_setupEPWMLinks(base, i,EPWM_LINK_TBPRD);
        EPWM_setupEPWMLinks(base, i,EPWM_LINK_COMP_A);
        EPWM_setupEPWMLinks(base, i,EPWM_LINK_COMP_B);
    }
    //****************************************************************************************//
    if(error==0)
    {
        if(enableLog)
        {
            DebugP_log("\r\nPass");
        }

        return 0;
    }
    else
    {
        if(enableLog)
        {
            DebugP_log("\r\nFail");
        }

        return 1;
    }
}

 /* EPWM_pwm_chopper_operation */
int32_t AM263x_EPWM_xTR_0006(uint32_t base, uint32_t i)
{
    /************************************* SETTING UP TESTER *************************************/
    char epwmA[CMD_SIZE];
    char epwmB[CMD_SIZE];
    char sync_epwmA[CMD_SIZE];
    char sync_epwmB[CMD_SIZE];
    char cap_epwmA[CMD_SIZE];
    char cap_epwmB[CMD_SIZE];
    char ts_epwmA[CMD_SIZE];
    char ts_epwmB[CMD_SIZE];

    if(i<10)
        {
            sprintf(epwmA, "setup epwm 0%u A", i);
            sprintf(epwmB, "setup epwm 0%u B", i);
            sprintf(sync_epwmA, "sync with epwm 0%u A", i);
            sprintf(sync_epwmB, "sync with epwm 0%u B", i);
            sprintf(cap_epwmA, "capture epwm 0%u A ", i);
            sprintf(cap_epwmB, "capture epwm 0%u B ", i);
            sprintf(ts_epwmA, "get timestamp 0%u A", i);
            sprintf(ts_epwmB, "get timestamp 0%u B", i);
        }
    else
        {
            sprintf(epwmA, "setup epwm %u A", i);
            sprintf(epwmB, "setup epwm %u B", i);
            sprintf(sync_epwmA, "sync with epwm %u A", i);
            sprintf(sync_epwmB, "sync with epwm %u B", i);
            sprintf(cap_epwmA, "capture epwm %u A ", i);
            sprintf(cap_epwmB, "capture epwm %u B ", i);
            sprintf(ts_epwmA, "get timestamp %u A", i);
            sprintf(ts_epwmB, "get timestamp %u B", i);
        }


    /*****************************************************************************************/

    int32_t error=0;
    int32_t tmp = i;
    i%=4;

    util_setEPWMTB(base, EPWM_EMULATION_FREE_RUN, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1, 0x5DC0, EPWM_COUNTER_MODE_STOP_FREEZE);
    util_setEPWMCC(base, 0x4650, 0x4650);
    EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
    EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
    EPWM_enableChopper(base);
    EPWM_setChopperFreq(base, i+3);    /*  6.25MHz, 5MHz, 4.1667MHz, 3.57MHz i.e. 160ns, 200ns, 240ns, 280ns */
    EPWM_setChopperDutyCycle(base, i+1); /*  High zone: 40ns, 75ns, 120ns, 175ns */
    EPWM_setChopperFirstPulseWidth(base, i+2);  /* First pulse: 120ns, 160ns, 200ns, 240ns */
    EPWM_setTimeBaseCounterMode(base, EPWM_COUNTER_MODE_UP_DOWN);
    EPWM_enableSyncOutPulseSource(base, EPWM_SYNC_OUT_PULSE_ON_CNTR_COMPARE_B);


   /************************************* VALIDATING FROM TESTER *************************************/

    Configure_Xbar_Sync_DUT(tmp);
    tester_command(epwmA);
    tester_command(epwmB);
    /* Ask Tester ECAP to sync */
    tester_command(sync_epwmA); /* EPWMx syncout from DUT syncs EPWM 8 in Tester which in turn syncs ECAP in Tester */
    tester_command(sync_epwmB); /*  EPWMx syncout from DUT syncs EPWM 8 in Tester which in turn syncs ECAP in Tester */

    strcat(cap_epwmA, "in oneshot for edges FRFR in DELTA");
    tester_command(cap_epwmA);

    strcat(cap_epwmB, "in oneshot for edges FRFR in DELTA");
    tester_command(cap_epwmB);
    ClockP_usleep(10000);

    int32_t timestamp_event1_A=0;
    /* int32_t timestamp_event2_A=0; */
    int32_t timestamp_event3_A=0;
    int32_t timestamp_event4_A=0;

    int32_t timestamp_event1_B=0;
    /* int32_t timestamp_event2_B=0; */
    int32_t timestamp_event3_B=0;
    int32_t timestamp_event4_B=0;

    tester_command(ts_epwmA);
    timestamp_event1_A = ts[0]; /* Width of first high pulse of ePWMxA */
    /* timestamp_event2_A = ts[1]; */ /* Width of low pulse of ePWMxA */
    timestamp_event3_A = ts[2]; /* Width of high pulse of ePWMxA */
    timestamp_event4_A = ts[3]; /* Width of low pulse of ePWMxA */
    /* if(enableLog)
    {
        DebugP_log("%u %u %u %u\r\n", timestamp_event1_A, timestamp_event2_A, timestamp_event3_A, timestamp_event4_A);
    }*/

    tester_command(ts_epwmB);
    timestamp_event1_B = ts[0]; /* Width of first high pulse of ePWMxB */
    /* timestamp_event2_B = ts[1];  */ /* Width of low pulse of ePWMxB */
    timestamp_event3_B = ts[2]; /* Width of high pulse of ePWMxB */
    timestamp_event4_B = ts[3]; /* Width of low pulse of ePWMxB */

    /* Validating if the timestamp captured by ecap is correct */

    int32_t tbprd_captured = ((timestamp_event3_A + timestamp_event4_A) + (timestamp_event3_B + timestamp_event4_B))/2;
    int32_t expected_tbprd = 8*(1+(i+3));
    int32_t diff_tbprd = abs(tbprd_captured - expected_tbprd);

    int32_t high_pulse_width = (timestamp_event3_A + timestamp_event3_B)/2;
    int32_t expected_high_pulse_width = (expected_tbprd * (1+(i+1)))/8;
    int32_t diff_high_pulse_width = abs(high_pulse_width - expected_high_pulse_width);

    int32_t first_pulse_width = (timestamp_event1_A + timestamp_event1_B)/2;
    int32_t expected_first_pulse_width = (1+(i+2))*8;
    int32_t diff_first_pulse_width = abs(first_pulse_width - expected_first_pulse_width);

    if( (diff_tbprd >20 ) || (diff_high_pulse_width >20 ) || (diff_first_pulse_width >20))
    {
        error++;
    }
    if(enableLog)
    {
        /*DebugP_log("Channel A: 1: %d, 3: %d, 4: %d \n Channel B: 1: %d, 3: %d, 4: %d ", timestamp_event1_A, timestamp_event3_A, timestamp_event4_A,
        timestamp_event1_B, timestamp_event3_B, timestamp_event4_B);*/
        DebugP_log("%d - %d, %d - %d, %d - %d\r\n",tbprd_captured,expected_tbprd, high_pulse_width, expected_high_pulse_width,
    first_pulse_width, expected_first_pulse_width);
    }

    /*******************************************************************************************/

    //*************************************De-initializing*************************************//
    util_deinit_epwms(i);
    util_resetEPWMTB(base);
    util_resetEPWMCC(base);
    util_resetEPWMAQ(base);
    EPWM_disableChopper(base);
    EPWM_setChopperFreq(base, 0x0);
    EPWM_setChopperDutyCycle(base, 0x0);
    EPWM_setChopperFirstPulseWidth(base,0x0);

    //****************************************************************************************//
    if(error==0)
    {
        if(enableLog)
        {
            DebugP_log("\r\nPass");
        }

        return 0;
    }
    else
    {
        if(enableLog)
        {
            DebugP_log("\r\nFail");
        }

        return 1;
    }
}

/*  EPWM_all_pwm_working_together */
int32_t AM263x_EPWM_xTR_0007(uint32_t dummy)
{
    /************************************* SETTING UP TESTER *************************************/

    for(int i = 0; i<5; i++)  /* FIXME for all epwm instances --> At a time can test 2 channels of 5 instances as 10 ecaps are there */
    {
        char epwmA[CMD_SIZE];
        char epwmB[CMD_SIZE];
        char cap_epwmA[CMD_SIZE];
        char cap_epwmB[CMD_SIZE];

        if(i<10)
            {
                sprintf(epwmA, "setup epwm 0%u A", i);
                sprintf(epwmB, "setup epwm 0%u B", i);
                sprintf(cap_epwmA, "capture epwm 0%u A ", i);
                sprintf(cap_epwmB, "capture epwm 0%u B ", i);
            }
        else
            {
                sprintf(epwmA, "setup epwm %u A", i);
                sprintf(epwmB, "setup epwm %u B", i);
                sprintf(cap_epwmA, "capture epwm %u A ", i);
                sprintf(cap_epwmB, "capture epwm %u B ", i);
            }

        strcat(cap_epwmA, "continuously on edges FRFR in DELTA");
        strcat(cap_epwmB, "continuously on edges FRFR in DELTA");

        tester_command(epwmA);
        tester_command(cap_epwmA);

        tester_command(epwmB);
        tester_command(cap_epwmB);
    }
    /****************************************************************************************/

    /*  Configure Time Base */
    /* Configure Counter Compare */
    /* Configure Action Qualifier Events */
    uint8_t itr0, itr1, itr2, itr3;
    int32_t error=0;

    for(itr0 = 0 ; itr0 < 3; itr0++)   /* clkdiv, hsclkdiv, tbprd */
    {
        for(itr1= 0 ; itr1 < 3; itr1++)  /* count mode */
        {
            for(itr2 = 0; itr2 < 3; itr2++) /* counter compare values */
            {
                if (tb_clk_vector[itr0][2] == cc_values_vector[itr2])
                        continue;
                for(itr3 = 0 ;itr3 < 5; itr3++)   /* epwm instances   Change to 32 */
                {
                    char ts_epwmA[CMD_SIZE];
                    char ts_epwmB[CMD_SIZE];
                    uint32_t base = CSL_CONTROLSS_G0_EPWM0_U_BASE + itr3 * 0x1000;
                    util_setEPWMTB(base, EPWM_EMULATION_FREE_RUN, tb_clk_vector[itr0][0], tb_clk_vector[itr0][1], tb_clk_vector[itr0][2], tb_countmode_vector[itr1]);
                    util_setEPWMCC(base, cc_values_vector[itr2], cc_values_vector[itr2]);

                    switch (itr1)
                    {
                        case 0: /* Up count */
                            EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
                            EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
                            EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
                            EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
                            break;
                        case 1:  /* Down count */
                            EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
                            EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);
                            EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
                            EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
                            break;
                        case 2:  /* Up Down count */
                            EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
                            EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);
                            EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
                            EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);
                            break;
                    }
                ClockP_usleep(1000);
                //************************************* VALIDATING FROM TESTER *************************************//

                    sprintf(ts_epwmA, "get timestamp 0%u A", itr3);
                    sprintf(ts_epwmB, "get timestamp 0%u B", itr3);

                    /* int32_t timestamp_event1_A=0; */
                    /* int32_t timestamp_event2_A=0; */
                    int32_t timestamp_event3_A=0;
                    int32_t timestamp_event4_A=0;

                   /*  int32_t timestamp_event1_B=0;
                    int32_t timestamp_event2_B=0;
                    int32_t timestamp_event3_B=0;
                    int32_t timestamp_event4_B=0; */

                    tester_command(ts_epwmA);
                    /* timestamp_event1_A = ts[0]; */ /* Width of high pulse of ePWMxA */
                    /* timestamp_event2_A = ts[1]; */ /* Width of low pulse of ePWMxA */
                    timestamp_event3_A = ts[2]; /* Width of high pulse of ePWMxA */
                    timestamp_event4_A = ts[3]; /* Width of low pulse of ePWMxA */

                    tester_command(ts_epwmB);

                    /*timestamp_event1_B = ts[0]; //Width of high pulse of ePWMxB
                    timestamp_event2_B = ts[1]; //Width of low pulse of ePWMxB
                    timestamp_event3_B = ts[2]; //Width of high pulse of ePWMxB
                    timestamp_event4_B = ts[3]; //Width of low pulse of ePWMxB
                    */

                    /* Validating if the timestamp captured by ecap is correct */
                    uint8_t multiplier = 1; /* Count mode is UP or DOWN */
                    if(itr1 == 2) multiplier = 2; /* Count mode is UP_DOWN */

                    int32_t low_pulse_width = (1<<tb_clk_vector[itr0][0]) * (1<<tb_clk_vector[itr0][1]) * (cc_values_vector[itr2]) * multiplier;
                    int32_t high_pulse_width = (1<<tb_clk_vector[itr0][0]) * (1<<tb_clk_vector[itr0][1]) * (tb_clk_vector[itr0][2]- cc_values_vector[itr2]) * multiplier;
                    int32_t high_cap_diff=0, low_cap_diff=0;
                    low_cap_diff=abs(timestamp_event4_A-low_pulse_width);
                    high_cap_diff=abs(timestamp_event3_A-high_pulse_width);

                    if(enableLog)
                        {
                            DebugP_log("%u - %u, %u - %u\r\n",timestamp_event4_A,low_pulse_width, timestamp_event3_A, high_pulse_width);
                        }
                    if( (low_cap_diff > 20 ) || (high_cap_diff  >20 ))
                    {
                        error++;
                        //break;
                    }
                }
                /****************************************************************************************/
                //*************************************De-initializing*************************************//


                    for(int i = 0; i<8; i++)
                    {
                        /*util_deinit_epwms(i);*/
                        uint32_t base = CSL_CONTROLSS_G0_EPWM0_U_BASE + i * 0x1000;
                        util_resetEPWMTB(base);
                        util_resetEPWMCC(base);
                        util_resetEPWMAQ(base);
                    }

                //****************************************************************************************//

            }
        }
    }


    if(error==0)
    {
        if(enableLog)
        {
            DebugP_log("\r\nPass");
        }
        return 0;
    }
    else
    {
        if(enableLog)
        {
            DebugP_log("\r\nFail");
        }

        return 1;
    }
}

/* Global variables and objects */
static HwiP_Object  gEpwmHwiObject;
static SemaphoreP_Object  gEpwmSyncSemObject;
HwiP_Params  hwiPrms;

static void Dac_epwmIntrISR(void *handle)
{
    uint32_t base = (uint32_t )handle;
    EPWM_setTimeBaseCounterMode(base, EPWM_COUNTER_MODE_STOP_FREEZE);
    volatile bool status;
    status = EPWM_getEventTriggerInterruptStatus(base);
    if(status == true)
    {
        uint16_t DAC_outputValue= (( (base & 0x0000F000) >> 0xC)+1) * 100;
        if(DAC_getActiveValue(CSL_CONTROLSS_DAC0_U_BASE) != DAC_outputValue)
        {
            SemaphoreP_post(&gEpwmSyncSemObject);
        }

        EPWM_clearEventTriggerInterruptFlag(base);
    }
    EPWM_disableInterrupt(base);
    EPWM_setTimeBaseCounterMode(base, EPWM_COUNTER_MODE_UP_DOWN);
    return;
}

// EPWM_dac_load_upon_pwm_event
int32_t AM263x_EPWM_xTR_0008(uint32_t base, uint32_t i)
{
    /*
    Configuring EPWM as TBPRD = 65535 and in up-down count mode and it will send out sync-out pulse at period. Also CMPA=12000.
    DAC upon receiving this sync pulse shall load shadow value to Active register. Until that, the active register should be 0.
    To check this: EPWM will issue an interrupt whenever TBCTR counts UP to CMPA.
    The ISR will freeze the counter and at this point since TBCTR hasn't reached TBPRD, the DAC_ActiveValue should be 0.
    While exiting from ISR the counter will resume counting and then after reaching PRD we again check DAC_ActiveValue is updated
    or not.
    */

    /* variable to set period value in TB */
    uint16_t periodValue = 65535;
    uint16_t cmpA = 30000;
    int32_t error = 0 ;
    uint16_t DAC_outputValue = 100*(i+1);
    uint16_t numIsrCnt = 1;

//----------------------------------------Configuring the Interrupt---------------------------------------------------------------
    SOC_xbarSelectInterruptXBarInputSource(CSL_CONTROLSS_INTXBAR_U_BASE, i, ( 1<<i ), 0, 0, 0, 0, 0, 0);
    uint32_t status = SemaphoreP_constructBinary(&gEpwmSyncSemObject, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Register & enable interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = (CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_0 + i);
    hwiPrms.callback    = &Dac_epwmIntrISR;
    hwiPrms.args        = (void *)base;
    hwiPrms.isPulse     = 1;
    status              = HwiP_construct(&gEpwmHwiObject, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);
//---------------------------------------------------------------------------------------------------------------------------------

      /* Configuring the EPWMx for creating a sync signal on Time base reaching period value. */
    util_setEPWMTB(base, EPWM_EMULATION_FREE_RUN, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1, periodValue, EPWM_COUNTER_MODE_STOP_FREEZE);
    util_setEPWMCC(base, cmpA, 0);
    EPWM_setInterruptSource(base, EPWM_INT_TBCTR_U_CMPA, EPWM_INT_TBCTR_U_CMPA);
    EPWM_setInterruptEventCount(base, 1);
    HRPWM_setSyncPulseSource(base, HRPWM_PWMSYNC_SOURCE_PERIOD);
    EPWM_enableInterrupt(base);
    EPWM_clearEventTriggerInterruptFlag(base);

     /* initiating DAC to be loaded with PWMSync Signal and which EPWM's sync signal */
    DAC_setReferenceVoltage(CSL_CONTROLSS_DAC0_U_BASE, DAC_REF_VREF);
    DAC_setLoadMode(CSL_CONTROLSS_DAC0_U_BASE, DAC_LOAD_PWMSYNC);
    DAC_setPWMSyncSignal(CSL_CONTROLSS_DAC0_U_BASE, i+1);
    DAC_setShadowValue(CSL_CONTROLSS_DAC0_U_BASE, DAC_outputValue);
    ClockP_usleep(10);

    EPWM_setTimeBaseCounterMode(base, EPWM_COUNTER_MODE_UP_DOWN);  /* Start the counter */

    while(numIsrCnt--)
    {
        SemaphoreP_pend(&gEpwmSyncSemObject, SystemP_WAIT_FOREVER);
    }

    while(!EPWM_getTimeBaseCounterOverflowStatus(base));

    if(DAC_getActiveValue(CSL_CONTROLSS_DAC0_U_BASE) != DAC_outputValue)
    {
        error++;
    }
    ClockP_usleep(10);
    if(enableLog)
    {
        DebugP_log("After count mode %u\r\n", DAC_getActiveValue(CSL_CONTROLSS_DAC0_U_BASE));
    }
    //*************************************De-initializing*************************************//
    HwiP_destruct(&gEpwmHwiObject);
    SemaphoreP_destruct(&gEpwmSyncSemObject);

    util_deinit_epwms(i);

    util_resetEPWMTB(base);
    util_resetEPWMCC(base);
    HW_WR_REG16(base + CSL_EPWM_ETSEL,(HW_RD_REG16(base + CSL_EPWM_ETSEL) & ~CSL_EPWM_ETSEL_INTSELCMP_MASK));
    HW_WR_REG16(base + CSL_EPWM_ETSEL,((HW_RD_REG16(base + CSL_EPWM_ETSEL) & ~CSL_EPWM_ETSEL_INTSEL_MASK) | ~EPWM_INT_TBCTR_U_CMPA));
    EPWM_clearTimeBaseCounterOverflowEvent(base);
    EPWM_setInterruptEventCount(base, 0x0000);
    HRPWM_setSyncPulseSource(base, 0x0000);


    util_deinit_dac();
    //****************************************************************************************//


    if(error==0)
    {
        if(enableLog)
        {
            DebugP_log("\r\nPass");
        }

        return 0;
    }
    else
    {
        if(enableLog)
        {
            DebugP_log("\r\nFail");
        }

        return 1;
    }
}

static void Cmpss_epwmIntrISR(void *handle)
{
    uint32_t base = (uint32_t )handle;
    EPWM_setTimeBaseCounterMode(base, EPWM_COUNTER_MODE_STOP_FREEZE);
    volatile bool status;
    status = EPWM_getEventTriggerInterruptStatus(base);
    if(status == true)
    {
        uint16_t DAC_outputValue= (( (base & 0x0000F000) >> 0xC)+1) * 100;
        if(CMPSS_getDACValueHigh(CSL_CONTROLSS_CMPSSA0_U_BASE) != DAC_outputValue)
        {
            SemaphoreP_post(&gEpwmSyncSemObject);
        }

        EPWM_clearEventTriggerInterruptFlag(base);
    }
    EPWM_setTimeBaseCounterMode(base, EPWM_COUNTER_MODE_UP_DOWN);
    return;
}

 /* EPWM_cmpss_load_upon_pwm_event */
int32_t AM263x_EPWM_xTR_0009(uint32_t base, uint32_t i)
{
    /*
    Configuring EPWM as TBPRD = 65535 and in up-down count mode and it will send out sync-out pulse at period. Also CMPA=12000.
    CMPSS_DAC upon receiving this sync pulse shall load shadow value to Active register. Until that, the active register should be 0.
    To check this: EPWM will issue an interrupt whenever TBCTR counts UP to CMPA.
    The ISR will freeze the counter and at this point since TBCTR hasn't reached TBPRD, the CMPSS_DAC_ActiveValue should be 0.
    While exiting from ISR the counter will resume counting and then after reaching PRD we again check CMPSS_DAC_ActiveValue is updated
    or not.
    */

    /* variable to set period value in TB */
    uint16_t periodValue = 65535;
    uint16_t cmpA = 30000;
    int32_t error = 0 ;
    uint16_t DAC_outputValue = 100*(i+1);
    uint16_t numIsrCnt = 1;
//----------------------------------------Configuring the Interrupt---------------------------------------------------------------
    SOC_xbarSelectInterruptXBarInputSource(CSL_CONTROLSS_INTXBAR_U_BASE, i, ( 1<<i ), 0, 0, 0, 0, 0, 0);
    uint32_t status = SemaphoreP_constructBinary(&gEpwmSyncSemObject, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Register & enable interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = (CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_0 + i);
    hwiPrms.callback    = &Cmpss_epwmIntrISR;
    hwiPrms.args        = (void *)base;
    hwiPrms.isPulse     = 1;
    status              = HwiP_construct(&gEpwmHwiObject, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);
//-----------------------------------------------------------------------------------------------------------

    /*  Configuring the EPWMx for creating a sync signal on Time base reaching period value. */
    util_setEPWMTB(base, EPWM_EMULATION_FREE_RUN, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1, periodValue, EPWM_COUNTER_MODE_STOP_FREEZE);
    util_setEPWMCC(base, cmpA, 0);
    EPWM_enableInterrupt(base);
    EPWM_setInterruptSource(base, EPWM_INT_TBCTR_U_CMPA, EPWM_INT_TBCTR_U_CMPA);
    EPWM_setInterruptEventCount(base, 1);
    HRPWM_setSyncPulseSource(base, HRPWM_PWMSYNC_SOURCE_PERIOD);
    EPWM_clearEventTriggerInterruptFlag(base);

    /* initiating CMPSS to be loaded with PWMSync Signal and which EPWM's sync signal */
    CMPSS_enableModule(CSL_CONTROLSS_CMPSSA0_U_BASE);
	CMPSS_configHighComparator(CSL_CONTROLSS_CMPSSA0_U_BASE,(CMPSS_INSRC_DAC));
	CMPSS_configDAC(CSL_CONTROLSS_CMPSSA0_U_BASE,(CMPSS_DACVAL_PWMSYNC | CMPSS_DACREF_VDDA | CMPSS_DACSRC_SHDW));
    CMPSS_configRamp(CSL_CONTROLSS_CMPSSA0_U_BASE, 0, 0, 0, i+1, false);
	CMPSS_setDACValueHigh(CSL_CONTROLSS_CMPSSA0_U_BASE, DAC_outputValue);
	ClockP_usleep(10);

    EPWM_setTimeBaseCounterMode(base, EPWM_COUNTER_MODE_UP_DOWN);  /* Start the counter */

    while(numIsrCnt--)
    {
        SemaphoreP_pend(&gEpwmSyncSemObject, SystemP_WAIT_FOREVER);
    }

    EPWM_disableInterrupt(base);

    while( ! EPWM_getTimeBaseCounterOverflowStatus(base));

    if(CMPSS_getDACValueHigh(CSL_CONTROLSS_CMPSSA0_U_BASE) != DAC_outputValue)
    {
        error++;
    }

    //*************************************De-initializing*************************************//
    HwiP_destruct(&gEpwmHwiObject);
    SemaphoreP_destruct(&gEpwmSyncSemObject);

    /*util_deinit_epwms(i);*/

    util_resetEPWMTB(base);
    util_resetEPWMCC(base);
    HW_WR_REG16(base + CSL_EPWM_ETSEL,(HW_RD_REG16(base + CSL_EPWM_ETSEL) & ~CSL_EPWM_ETSEL_INTSELCMP_MASK));
    HW_WR_REG16(base + CSL_EPWM_ETSEL,((HW_RD_REG16(base + CSL_EPWM_ETSEL) & ~CSL_EPWM_ETSEL_INTSEL_MASK) | ~EPWM_INT_TBCTR_U_CMPA));
    EPWM_clearTimeBaseCounterOverflowEvent(base);
    EPWM_setInterruptEventCount(base, 0x0000);
    HRPWM_setSyncPulseSource(base, 0x0000);

    util_deinit_cmpss(CSL_CONTROLSS_CMPSSA0_U_BASE);
    //****************************************************************************************//

    if(error==0)
    {
        if(enableLog)
        {
            DebugP_log("\r\nPass");
        }

        return 0;
    }
    else
    {
        if(enableLog)
        {
            DebugP_log("\r\nFail");
        }

        return 1;
    }
}

 /* EPWM_gpio_signal_trips_pwm */

int32_t AM263x_EPWM_xTR_0010(uint32_t base, uint32_t i)
{
    /* connect Pin 49 (GPIO23, D7) on LP to Pin 73 (GPIO125, D13)
    Generate Dac voltage through commands. Remove the above physical connection */
    util_setEPWMCC(base, 6000, 0);
    uint32_t periodValue = 0x5DC0, cmpA = 6000;
    int8_t count = 2;
    int32_t error = 0;
    GPIO_setDirMode(GPIO_OUTPUT_BASE_ADDR, GPIO_OUTPUT_PIN, 0);
    GPIO_setDirMode(GPIO_INPUT_BASE_ADDR, GPIO_INPUT_PIN, 1);
    GPIO_pinWriteHigh(GPIO_OUTPUT_BASE_ADDR, GPIO_OUTPUT_PIN);

    util_setEPWMTB(base, EPWM_EMULATION_FREE_RUN, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1, periodValue, EPWM_COUNTER_MODE_UP_DOWN);
    util_setEPWMCC(base, cmpA, 0);
    EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    EPWM_selectDigitalCompareTripInput(base, EPWM_DC_TRIP_TRIPIN1, EPWM_DC_TYPE_DCAH);
    EPWM_setTripZoneDigitalCompareEventCondition(base, EPWM_TZ_DC_OUTPUT_A1, EPWM_TZ_EVENT_DCXH_LOW);
    EPWM_setTripZoneAction(base, EPWM_TZ_ACTION_EVENT_DCAEVT1, EPWM_TZ_ACTION_LOW);

    SOC_xbarSelectInputXBarInputSource(CSL_CONTROLSS_INPUTXBAR_U_BASE, 0, 0, GPIO_INPUT_PIN, 0);
    SOC_xbarSelectPWMXBarInputSource(CSL_CONTROLSS_PWMXBAR_U_BASE, 0, 0, 0, 0, 0, (EPWM_XBAR_INPUT_XBAR0), 0, 0, 0, 0);

    /************************************* VALIDATING FROM TESTER *************************************/
        char epwmA[CMD_SIZE];
        char epwmB[CMD_SIZE];
        char cap_epwmA[CMD_SIZE];
        char cap_epwmB[CMD_SIZE];
        char ts_epwmA[CMD_SIZE];
        char ts_epwmB[CMD_SIZE];

        if(i<10)
            {
                sprintf(epwmA, "setup epwm 0%u A", i);
                sprintf(epwmB, "setup epwm 0%u B", i);
                sprintf(cap_epwmA, "capture epwm 0%u A ", i);
                sprintf(cap_epwmB, "capture epwm 0%u B ", i);
                sprintf(ts_epwmA, "get timestamp 0%u A", i);
                sprintf(ts_epwmB, "get timestamp 0%u B", i);
            }
        else
            {
                sprintf(epwmA, "setup epwm %u A", i);
                sprintf(epwmB, "setup epwm %u B", i);
                sprintf(cap_epwmA, "capture epwm %u A ", i);
                sprintf(cap_epwmB, "capture epwm %u B ", i);
                sprintf(ts_epwmA, "get timestamp %u A", i);
                sprintf(ts_epwmB, "get timestamp %u B", i);
            }

        tester_command(epwmA);
        strcat(cap_epwmA, "continuously on edges FFFR in DELTA");
        tester_command(cap_epwmA);

        tester_command(epwmB);
        strcat(cap_epwmB, "continuously on edges FFFR in DELTA");
        tester_command(cap_epwmB);

        /* int32_t timestamp_event1_A=0; */
        int32_t timestamp_event2_A=0;
        int32_t timestamp_event3_A=0;
        int32_t timestamp_event4_A=0;

        int32_t timestamp_event1_B=0;
        int32_t timestamp_event2_B=0;
        int32_t timestamp_event3_B=0;
        int32_t timestamp_event4_B=0;

        while(count--)
       {
            GPIO_pinWriteHigh(GPIO_OUTPUT_BASE_ADDR, GPIO_OUTPUT_PIN);
            ClockP_usleep(500);
            GPIO_pinWriteLow(GPIO_OUTPUT_BASE_ADDR, GPIO_OUTPUT_PIN);
            ClockP_usleep(1000);
            if(count == 0)
            {
                tester_command(ts_epwmA);
                /* timestamp_event1_A = ts[0]; */ /* Width of high pulse of ePWMxA */
                timestamp_event2_A = ts[1]; /* Width of period of ePWMxA */
                timestamp_event3_A = ts[2]; /* Width of period of ePWMxA */
                timestamp_event4_A = ts[3]; /* Width of low pulse of ePWMxA */
                /* if(enableLog)
                {
                    DebugP_log("%u %u %u %u\r\n", timestamp_event1_A, timestamp_event2_A, timestamp_event3_A, timestamp_event4_A);
                }*/

                tester_command(ts_epwmB);
                timestamp_event1_B = ts[0]; /* Width of high pulse of ePWMxB */
                timestamp_event2_B = ts[1]; /* Width of period of ePWMxB */
                timestamp_event3_B = ts[2]; /* Width of period of ePWMxB */
                timestamp_event4_B = ts[3];/*  Width of low pulse of ePWMxB */
            }
            EPWM_clearTripZoneFlag(base, EPWM_TZ_FLAG_DCAEVT1);
       }

        /* Validating if the timestamp captured by ecap is correct */

        int32_t high_pulse_width = timestamp_event1_B;
        int32_t expected_high_pulse_width = (1<<EPWM_CLOCK_DIVIDER_1) * (1<<EPWM_HSCLOCK_DIVIDER_1) * (periodValue - cmpA) * 2;
        int32_t diff_high_pulse_width = abs(high_pulse_width - expected_high_pulse_width);

        int32_t low_pulse_width = timestamp_event4_B;
        int32_t expected_low_pulse_width = (1<<EPWM_CLOCK_DIVIDER_1) * (1<<EPWM_HSCLOCK_DIVIDER_1) * (cmpA) * 2 ;
        int32_t diff_low_pulse_width = abs(low_pulse_width - expected_low_pulse_width);

        /* Difference in low width pulse (or period) in two channels while trip occurs
        There should be significant difference between the 2 channels to see if the effect has taken place. 10000*5ns = 50 Us. */
        int32_t diff_low_chA_chB = abs( timestamp_event4_A - timestamp_event4_B);
        diff_low_chA_chB = diff_low_chA_chB>abs( timestamp_event3_A - timestamp_event3_B)? diff_low_chA_chB:abs( timestamp_event3_A - timestamp_event3_B);
        diff_low_chA_chB = diff_low_chA_chB>abs( timestamp_event2_A - timestamp_event2_B)? diff_low_chA_chB:abs( timestamp_event2_A - timestamp_event2_B);

        if( (diff_high_pulse_width > 10 ) || (diff_low_pulse_width > 10 ) || (diff_low_chA_chB < 10000))
        {
            error++;
        }

        if(enableLog)
        {
            DebugP_log("%u - %u, %u - %u, %u\r\n",high_pulse_width,expected_high_pulse_width, low_pulse_width, expected_low_pulse_width,
        diff_low_chA_chB);
        }

    //*************************************De-initializing*************************************//
    /*util_deinit_epwms(i);*/
    util_resetEPWMTB(base);
    util_resetEPWMCC(base);
    util_resetEPWMAQ(base);
    EPWM_selectDigitalCompareTripInput(base, 0x0, EPWM_DC_TYPE_DCAH);
    EPWM_setTripZoneDigitalCompareEventCondition(base, EPWM_TZ_DC_OUTPUT_A1, 0x0);
    EPWM_setTripZoneAction(base, EPWM_TZ_ACTION_EVENT_DCAEVT1, 0x0);

    //****************************************************************************************//
    if(error==0)
    {
        if(enableLog)
        {
            DebugP_log("\r\nPass");
        }
        return 0;
    }
    else
    {
        if(enableLog)
        {
            DebugP_log("\r\nFail");
        }

        return 1;
    }
}

/*  EPWM_cmpss_to_trip_pwm */
int32_t AM263x_EPWM_xTR_0012(uint32_t base, uint32_t i)
{
    /* connect Pin 23 (cmpssA0 Pin) on LP with Pin 50 (GPIO24, c8) */
    /*Generate Dac voltage through commands. Remove the above physical connection */
    uint16_t periodValue = 24000, cmpA = 6000;
    uint16_t count = 5;
    int32_t error = 0;

    SOC_xbarSelectPWMXBarInputSource(CSL_CONTROLSS_PWMXBAR_U_BASE, 1, ( EPWM_XBAR_CMPSSA0_CTRIPH ), 0, 0, 0, 0, 0, 0, 0, 0);

    GPIO_setDirMode(CMPSS_BASE_ADDR, CMPSS_PIN, 0);
    GPIO_pinWriteHigh(CMPSS_BASE_ADDR, CMPSS_PIN);
   /*  tester_command("gen dac voltage"); */

    CMPSS_configHighComparator(CSL_CONTROLSS_CMPSSA0_U_BASE,(CMPSS_INSRC_DAC));
	CMPSS_configDAC(CSL_CONTROLSS_CMPSSA0_U_BASE,(CMPSS_DACVAL_SYSCLK | CMPSS_DACREF_VDDA | CMPSS_DACSRC_SHDW));
	CMPSS_setDACValueHigh(CSL_CONTROLSS_CMPSSA0_U_BASE,2048);
	CMPSS_configOutputsHigh(CSL_CONTROLSS_CMPSSA0_U_BASE,(CMPSS_TRIPOUT_ASYNC_COMP | CMPSS_TRIP_ASYNC_COMP));
	CMPSS_configRamp(CSL_CONTROLSS_CMPSSA0_U_BASE,0,0,0,1,true);
	CMPSS_enableModule(CSL_CONTROLSS_CMPSSA0_U_BASE);
	ClockP_usleep(10);

    util_setEPWMTB(base, EPWM_EMULATION_FREE_RUN, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1, periodValue, EPWM_COUNTER_MODE_UP_DOWN);
    util_setEPWMCC(base, cmpA, 0);
    EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    EPWM_selectDigitalCompareTripInput(base, EPWM_DC_TRIP_TRIPIN2, EPWM_DC_TYPE_DCAH);
    EPWM_setTripZoneDigitalCompareEventCondition(base, EPWM_TZ_DC_OUTPUT_A1, EPWM_TZ_EVENT_DCXH_LOW);
    EPWM_setTripZoneAction(base, EPWM_TZ_ACTION_EVENT_DCAEVT1, EPWM_TZ_ACTION_LOW);

    /************************************* VALIDATING FROM TESTER *************************************/
        char epwmA[CMD_SIZE];
        char epwmB[CMD_SIZE];
        char cap_epwmA[CMD_SIZE];
        char cap_epwmB[CMD_SIZE];
        char ts_epwmA[CMD_SIZE];
        char ts_epwmB[CMD_SIZE];

        if(i<10)
            {
                sprintf(epwmA, "setup epwm 0%u A", i);
                sprintf(epwmB, "setup epwm 0%u B", i);
                sprintf(cap_epwmA, "capture epwm 0%u A ", i);
                sprintf(cap_epwmB, "capture epwm 0%u B ", i);
                sprintf(ts_epwmA, "get timestamp 0%u A", i);
                sprintf(ts_epwmB, "get timestamp 0%u B", i);
            }
        else
            {
                sprintf(epwmA, "setup epwm %u A", i);
                sprintf(epwmB, "setup epwm %u B", i);
                sprintf(cap_epwmA, "capture epwm %u A ", i);
                sprintf(cap_epwmB, "capture epwm %u B ", i);
                sprintf(ts_epwmA, "get timestamp %u A", i);
                sprintf(ts_epwmB, "get timestamp %u B", i);
            }
        tester_command(epwmA);
        strcat(cap_epwmA, "continuously on edges FFFR in DELTA");
        tester_command(cap_epwmA);

        tester_command(epwmB);
        strcat(cap_epwmB, "continuously on edges FFFR in DELTA");
        tester_command(cap_epwmB);

       /*  int32_t timestamp_event1_A=0; */
        int32_t timestamp_event2_A=0;
        int32_t timestamp_event3_A=0;
        int32_t timestamp_event4_A=0;

        int32_t timestamp_event1_B=0;
        int32_t timestamp_event2_B=0;
        int32_t timestamp_event3_B=0;
        int32_t timestamp_event4_B=0;

        while(count--)
        {
            ClockP_usleep(100);
            GPIO_pinWriteLow(CMPSS_BASE_ADDR, CMPSS_PIN);
            ClockP_usleep(5000);
            if(count == 0)
            {
                tester_command(ts_epwmA);
               /*timestamp_event1_A = ts[0]; */ /* Width of high pulse of ePWMxA */
                timestamp_event2_A = ts[1]; /* Width of period of ePWMxA */
                timestamp_event3_A = ts[2]; /* Width of period of ePWMxA */
                timestamp_event4_A = ts[3]; /* Width of low pulse of ePWMxA */
                /* if(enableLog)
                    {
                        DebugP_log("%u %u %u %u\r\n", timestamp_event1_A, timestamp_event2_A, timestamp_event3_A, timestamp_event4_A);
                    }*/
                tester_command(ts_epwmA);
                timestamp_event1_B = ts[0]; /* Width of high pulse of ePWMxB */
                timestamp_event2_B = ts[1];  /* Width of period of ePWMxB */
                timestamp_event3_B = ts[2]; /* Width of period of ePWMxB */
                timestamp_event4_B = ts[3]; /* Width of low pulse of ePWMxB */
            }
            if((EPWM_getTripZoneFlagStatus(base) & EPWM_TZ_FLAG_DCAEVT1) != 0U)
            {
                /* Clear trip flags*/
                EPWM_clearTripZoneFlag(base, EPWM_TZ_FLAG_DCAEVT1);
            }
            GPIO_pinWriteHigh(CMPSS_BASE_ADDR, CMPSS_PIN);
        }

        /* Validating if the timestamp captured by ecap is correct */

        int32_t high_pulse_width = timestamp_event1_B;
        int32_t expected_high_pulse_width = (1<<EPWM_CLOCK_DIVIDER_1) * (1<<EPWM_HSCLOCK_DIVIDER_1) * (periodValue - cmpA) * 2;
        int32_t diff_high_pulse_width = abs(high_pulse_width - expected_high_pulse_width);

        int32_t low_pulse_width = timestamp_event4_B;
        int32_t expected_low_pulse_width = (1<<EPWM_CLOCK_DIVIDER_1) * (1<<EPWM_HSCLOCK_DIVIDER_1) * (cmpA) * 2 ;
        int32_t diff_low_pulse_width = abs(low_pulse_width - expected_low_pulse_width);

        /* Difference in low width pulse (or period) in two channels while trip occurs */

        int32_t diff_low_chA_chB = abs( timestamp_event4_A - timestamp_event4_B);
        diff_low_chA_chB = diff_low_chA_chB>abs( timestamp_event3_A - timestamp_event3_B)? diff_low_chA_chB:abs( timestamp_event3_A - timestamp_event3_B);
        diff_low_chA_chB = diff_low_chA_chB>abs( timestamp_event2_A - timestamp_event2_B)? diff_low_chA_chB:abs( timestamp_event2_A - timestamp_event2_B);

        if( (diff_high_pulse_width > 10 ) || (diff_low_pulse_width > 10 ) || (diff_low_chA_chB < 10000))
        {
            error++;
        }

        if(enableLog)
        {
            DebugP_log("%u - %u, %u - %u, %u\r\n",high_pulse_width,expected_high_pulse_width, low_pulse_width, expected_low_pulse_width,
        diff_low_chA_chB);
        }

        /*******************************************************************************************/

    //*************************************De-initializing*************************************//
    /*util_deinit_epwms(i);*/
    util_resetEPWMTB(base);
    util_resetEPWMCC(base);
    util_resetEPWMAQ(base);
    EPWM_selectDigitalCompareTripInput(base, 0x0, EPWM_DC_TYPE_DCAH);
    EPWM_setTripZoneDigitalCompareEventCondition(base, EPWM_TZ_DC_OUTPUT_A1, 0x0);
    EPWM_setTripZoneAction(base, EPWM_TZ_ACTION_EVENT_DCAEVT1, 0x0);
    util_deinit_cmpss(CSL_CONTROLSS_CMPSSA0_U_BASE);
    //****************************************************************************************//

    if(error==0)
    {
        if(enableLog)
        {
            DebugP_log("\r\nPass");
        }

        return 0;
    }
    else
    {
        if(enableLog)
        {
            DebugP_log("\r\nFail");
        }

        return 1;
    }
}

/* EPWM_deadband_basic */
int32_t AM263x_EPWM_xTR_0014(uint32_t base, uint32_t i)
{
    /************************************* SETTING UP TESTER *************************************/
    char epwmA[CMD_SIZE];
    char epwmB[CMD_SIZE];
    char cap_epwmA[CMD_SIZE];
    char cap_epwmB[CMD_SIZE];
    char ts_epwmA[CMD_SIZE];
    char ts_epwmB[CMD_SIZE];

    if(i<10)
    {
        sprintf(epwmA, "setup epwm 0%u A", i);
        sprintf(epwmB, "setup epwm 0%u B", i);
        sprintf(cap_epwmA, "capture epwm 0%u A ", i);
        sprintf(cap_epwmB, "capture epwm 0%u B ", i);
        sprintf(ts_epwmA, "get timestamp 0%u A", i);
        sprintf(ts_epwmB, "get timestamp 0%u B", i);
    }
    else
    {
        sprintf(epwmA, "setup epwm %u A", i);
        sprintf(epwmB, "setup epwm %u B", i);
        sprintf(cap_epwmA, "capture epwm %u A ", i);
        sprintf(cap_epwmB, "capture epwm %u B ", i);
        sprintf(ts_epwmA, "get timestamp %u A", i);
        sprintf(ts_epwmB, "get timestamp %u B", i);
    }

    tester_command(epwmA);
    strcat(cap_epwmA, "continuously on edges FRFR in DELTA");
    tester_command(cap_epwmA);

    tester_command(epwmB);
    strcat(cap_epwmB, "continuously on edges FRFR in DELTA");
    tester_command(cap_epwmB);

    /****************************************************************************************/

    uint16_t periodValue = 24000, cmpA = 6000, itr;
    int32_t error = 0;
    util_setEPWMTB(base, EPWM_EMULATION_FREE_RUN, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1, periodValue, EPWM_COUNTER_MODE_UP_DOWN);
    util_setEPWMCC(base, cmpA, 0);

    EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);

    EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);

    EPWM_setDeadBandControlShadowLoadMode(base, EPWM_DB_LOAD_ON_CNTR_ZERO);
    EPWM_enableGlobalLoadRegisters(base, EPWM_GL_REGISTER_DBRED_DBREDHR);
    EPWM_setRisingEdgeDelayCountShadowLoadMode(base, EPWM_RED_LOAD_ON_CNTR_ZERO);
    EPWM_setRisingEdgeDelayCount(base, 512);
    EPWM_enableGlobalLoadRegisters(base, EPWM_GL_REGISTER_DBFED_DBFEDHR);
    EPWM_setFallingEdgeDelayCountShadowLoadMode(base, EPWM_FED_LOAD_ON_CNTR_ZERO);
    EPWM_setFallingEdgeDelayCount(base, 1024);
    EPWM_setDeadBandCounterClock(base, EPWM_DB_COUNTER_CLOCK_FULL_CYCLE);
    EPWM_setDeadBandDelayMode(base, EPWM_DB_RED, true); //S1 = 1
    EPWM_setDeadBandDelayMode(base, EPWM_DB_FED, true); //S0 = 1
    EPWM_enableGlobalLoadRegisters(base, EPWM_GL_REGISTER_DBCTL);

    for(itr = 0; itr < 2; itr++)
    {

        switch(itr)
        {
            case 0:
            /*Applying rising edge delay of 2.5US to ePWMxA and falling edge delay of 5Us to ePWMxB and inverting it.
              Both Output NOT swapped at the end.
            */
                EPWM_setRisingEdgeDeadBandDelayInput(base, EPWM_DB_INPUT_EPWMA);    //S4 = 0
                EPWM_setFallingEdgeDeadBandDelayInput(base, EPWM_DB_INPUT_EPWMB);   // S5 = 1  and S8 = 0
                EPWM_setDeadBandDelayPolarity(base, EPWM_DB_RED, EPWM_DB_POLARITY_ACTIVE_HIGH); // S2 =0
                EPWM_setDeadBandDelayPolarity(base, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);  //S3 = 1
                EPWM_setDeadBandOutputSwapMode(base, EPWM_DB_OUTPUT_A, false);  // S6 = 0
                EPWM_setDeadBandOutputSwapMode(base, EPWM_DB_OUTPUT_B, false);  // S7 = 0
                break;
            case 1:
            /*Applying rising edge delay of 2.5US to ePWMxA and falling edge delay of 5Us to the rising edge delayed signal.
              OutB will have the same signal as OutA as S6 is switched on.
            */
                EPWM_setRisingEdgeDeadBandDelayInput(base, EPWM_DB_INPUT_EPWMA);    //S4 = 0
                EPWM_setFallingEdgeDeadBandDelayInput(base, EPWM_DB_INPUT_DB_RED);   // S8 = 1
                EPWM_setDeadBandDelayPolarity(base, EPWM_DB_RED, EPWM_DB_POLARITY_ACTIVE_HIGH); // S2 =0
                EPWM_setDeadBandDelayPolarity(base, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_HIGH);  //S3 =0
                EPWM_setDeadBandOutputSwapMode(base, EPWM_DB_OUTPUT_A, true);
                EPWM_setDeadBandOutputSwapMode(base, EPWM_DB_OUTPUT_B, false);
                break;
        }
        ClockP_usleep(10000);

         /************************************* VALIDATING FROM TESTER *************************************/
            /* int32_t timestamp_event1_A=0;
            int32_t timestamp_event2_A=0; */
            int32_t timestamp_event3_A=0;
            /* int32_t timestamp_event4_A=0; */

           /*  int32_t timestamp_event1_B=0;
            int32_t timestamp_event2_B=0; */
            int32_t timestamp_event3_B=0;
            int32_t timestamp_event4_B=0;

            tester_command(ts_epwmA);
            /* timestamp_event1_A = ts[0]; */ /* Width of high pulse of ePWMxA */
            /* timestamp_event2_A = ts[1]; */ /* Width of low pulse of ePWMxA */
            timestamp_event3_A = ts[2]; /* Width of high pulse of ePWMxA */
            //timestamp_event4_A = ts[3]; /* Width of low pulse of ePWMxA */

            tester_command(ts_epwmB);
            /* timestamp_event1_B = ts[0];  */ /* Width of high pulse of ePWMxB */
            /* timestamp_event2_B = ts[1]; */ /* Width of low pulse of ePWMxB */
            timestamp_event3_B = ts[2]; /* Width of high pulse of ePWMxB */
            timestamp_event4_B = ts[3]; /* Width of low pulse of ePWMxB */

           /*  Validating if the timestamp captured by ecap is correct */

           /* Checking if the correct rising or falling delay is applied to the respective channels */

            /*Rising edge delay of 2.5US to ePWMxA and falling edge delay of 5Us to ePWMxB and inverting it.
              Both Output NOT swapped at the end.
            */
            if(itr == 0)
            {
                int32_t high_pulse_width_A = timestamp_event3_A;
                int32_t expected_high_pulse_width_A = 35500; /*  (180Us - 2.5 Us)/5ns = 35500; Rising delay of 2.5Us */
                int32_t diff_high_pulse_width_A = abs(high_pulse_width_A - expected_high_pulse_width_A);

                int32_t low_pulse_width_B = timestamp_event4_B;
                int32_t expected_low_pulse_width_B = 37000;  /* (180Us + 5 Us)/5ns = 37000; Falling delay of 5Us and invert */
                int32_t diff_low_pulse_width_B = abs(low_pulse_width_B - expected_low_pulse_width_B);

                if( (diff_high_pulse_width_A >30 ) || (diff_low_pulse_width_B >30 ))
                {
                    error++;
                }

                if(enableLog)
                {
                    DebugP_log("%u - %u = %u, %u - %u =%u\r\n",high_pulse_width_A,expected_high_pulse_width_A, diff_high_pulse_width_A,
                low_pulse_width_B, expected_low_pulse_width_B, diff_low_pulse_width_B);
                }
            }
            /*Rising edge delay of 2.5US to ePWMxA and falling edge delay of 5Us to the rising edge delayed signal.
              OutB will have the same signal as OutA as S6 is switched on.
            */
            else if(itr == 1)
            {
                int32_t high_pulse_width_A = timestamp_event3_A;
                int32_t expected_high_pulse_width_A = 36500; /*  (180Us - 2.5 +5 Us)/5ns = 355000; Rising delay of 2.5Us and falling of 5Us */
                int32_t diff_high_pulse_width_A = abs(high_pulse_width_A - expected_high_pulse_width_A);

                int32_t high_pulse_width_B = timestamp_event3_B;
                int32_t expected_high_pulse_width_B = 36500; /*  (180Us - 2.5 +5 Us)/5ns = 355000; Rising delay of 2.5Us and falling of 5Us */
                int32_t diff_high_pulse_width_B = abs(high_pulse_width_B - expected_high_pulse_width_B);

                if( (diff_high_pulse_width_A >30 ) || (diff_high_pulse_width_B >30 ))
                {
                    error++;
                }

                if(enableLog)
                {
                    DebugP_log("%u - %u = %u, %u - %u = %u\r\n",high_pulse_width_A,expected_high_pulse_width_A,
                diff_high_pulse_width_A, high_pulse_width_B, expected_high_pulse_width_B, diff_high_pulse_width_B);
                }
            }

        /************************************************************************************************/
    }

    //*************************************De-initializing*************************************//
    /*util_deinit_epwms(i);*/
    util_resetEPWMTB(base);
    util_resetEPWMCC(base);
    util_resetEPWMAQ(base);
    EPWM_disableDeadBandControlShadowLoadMode(base);
    EPWM_disableGlobalLoadRegisters(base, EPWM_GL_REGISTER_DBRED_DBREDHR);
    EPWM_disableGlobalLoadRegisters(base, EPWM_GL_REGISTER_DBFED_DBFEDHR);
    EPWM_disableGlobalLoadRegisters(base, EPWM_GL_REGISTER_DBCTL);
    EPWM_setRisingEdgeDelayCount(base, 0x0);
    EPWM_setFallingEdgeDelayCount(base, 0x0);
    EPWM_setDeadBandDelayMode(base, EPWM_DB_RED, false);
    EPWM_setDeadBandDelayMode(base, EPWM_DB_FED, false);
    EPWM_setFallingEdgeDeadBandDelayInput(base, EPWM_DB_INPUT_EPWMA);
    EPWM_setDeadBandDelayPolarity(base, EPWM_DB_RED, EPWM_DB_POLARITY_ACTIVE_HIGH);
    EPWM_setDeadBandDelayPolarity(base, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_HIGH);
    EPWM_setDeadBandOutputSwapMode(base, EPWM_DB_OUTPUT_A, false);
    EPWM_setDeadBandOutputSwapMode(base, EPWM_DB_OUTPUT_B, false);
    //****************************************************************************************//
    if(error==0)
    {
        if(enableLog)
        {
            DebugP_log("\r\nPass");
        }
        return 0;
    }
    else
    {
        if(enableLog)
        {
            DebugP_log("\r\nFail");
        }
        return 1;
    }
}

/*  EPWM_diode_emulation_with_cmpss */
int32_t AM263x_EPWM_xTR_0015(uint32_t base)
{
   /*  TBD */

    int32_t error=0;

    EPWM_configureDiodeEmulationTripSources(base, EPWM_DE_TRIP_SRC_CMPSSA9, EPWM_DE_TRIPH);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG32(base + CSL_EPWM_DECOMPSEL)
    & CSL_EPWM_DECOMPSEL_TRIPH_MASK) >> CSL_EPWM_DECOMPSEL_TRIPH_SHIFT, EPWM_DE_TRIP_SRC_CMPSSA9);

    EPWM_configureDiodeEmulationTripSources(base, EPWM_DE_TRIP_SRC_CMPSSB6, EPWM_DE_TRIPL);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG32(base + CSL_EPWM_DECOMPSEL)
    & CSL_EPWM_DECOMPSEL_TRIPL_MASK) >> CSL_EPWM_DECOMPSEL_TRIPL_SHIFT, EPWM_DE_TRIP_SRC_CMPSSB6);

    /* util_deinit_epwms(); */

    if(error==0)
    {
        if(enableLog)
        {
            DebugP_log("\r\nPass");
        }
        return 0;
    }
    else
    {
        if(enableLog)
        {
            DebugP_log("\r\nFail");
        }
        return 1;
    }
}

static HwiP_Object  gAdcHwiObject1, gAdcHwiObject2;
int32_t gAdc_ISR1_count =0 , gAdc_ISR2_count =0, gnumIsrCnt = 0;
static void Adc_epwmIntrISR(void *handle)
{
    uint32_t base = (uint32_t )handle;
    volatile bool status;
    status = EPWM_getEventTriggerInterruptStatus(base);
    if(status)
    {
        SemaphoreP_post(&gEpwmSyncSemObject);
        gnumIsrCnt++;
        EPWM_clearEventTriggerInterruptFlag(base);
    }

    return;
}
void Adc_IntrISR1(void *args)
{
    gAdc_ISR1_count++;
    ADC_clearInterruptStatus(CSL_CONTROLSS_ADC0_U_BASE, ADC_INT_NUMBER1);
}
void Adc_IntrISR2(void *args)
{
    gAdc_ISR2_count++;
    ADC_clearInterruptStatus(CSL_CONTROLSS_ADC0_U_BASE, ADC_INT_NUMBER2);
}
/*  EPWM_adc_conversion_with_pwm_event */
int32_t AM263x_EPWM_xTR_0016(uint32_t base, uint32_t i)
{
    /*
    Configure event trigger to issue interrupt and ADC start of conversion at every 5th CMPC match event when timer is incrementing
    */
    int32_t error = 0, diffA =0 , diffB = 0;
    gAdc_ISR1_count = 0;
    gAdc_ISR2_count = 0;
    gnumIsrCnt = 0;
    //----------------------------------------Configuring the Interrupt---------------------------------------------------------------
    /* Register & enable interrupt */
    /* setting up epwm interrupt to route through int xbar 0*/
    SOC_xbarSelectInterruptXBarInputSource(CSL_CONTROLSS_INTXBAR_U_BASE, 0, ( 1<<i ), 0, 0, 0, 0, 0, 0);

    uint32_t status = SemaphoreP_constructBinary(&gEpwmSyncSemObject, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = (CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_0);
    hwiPrms.callback    = &Adc_epwmIntrISR;
    hwiPrms.args        = (void *)base;
    hwiPrms.isPulse     = 1;
    status              = HwiP_construct(&gEpwmHwiObject, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);

    /* setting up ADCINT1 to route through int xbar 1*/
    SOC_xbarSelectInterruptXBarInputSource(CSL_CONTROLSS_INTXBAR_U_BASE, 1, 0, 0,(INT_XBAR_ADC0_INT1),0, 0, 0, 0);

    /* Initialising a Interrupt parameter
     * setting up the interrupt, callbacks.*/
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_1;
    hwiPrms.callback    = &Adc_IntrISR1;
    hwiPrms.isPulse     = 1;
    status              = HwiP_construct(&gAdcHwiObject1, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);

    /* setting up ADCINT2 to route through int xbar 2*/
    SOC_xbarSelectInterruptXBarInputSource(CSL_CONTROLSS_INTXBAR_U_BASE, 2, 0, 0,(INT_XBAR_ADC0_INT2),0, 0, 0, 0);

    /* Initialising a Interrupt parameter
     * setting up the interrupt, callbacks.*/
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_2;
    hwiPrms.callback    = &Adc_IntrISR2;
    hwiPrms.isPulse     = 1;
    status              = HwiP_construct(&gAdcHwiObject2, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);
    //---------------------------------------------------------------------------------------------------------------------------------

    /* configuring EPWM */
    uint16_t prescale = 5;
    util_EPWM_setup_adc_trigger(base, EPWM_SOC_A, EPWM_INT_TBCTR_U_CMPC, prescale);
    util_EPWM_setup_adc_trigger(base, EPWM_SOC_B, EPWM_INT_TBCTR_U_CMPC, prescale);
    util_setEPWMTB(base, EPWM_EMULATION_FREE_RUN, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1, 1000, EPWM_COUNTER_MODE_STOP_FREEZE);
    EPWM_setCounterCompareValue(base, EPWM_COUNTER_COMPARE_C, 300);
    EPWM_setInterruptSource(base, EPWM_INT_TBCTR_PERIOD, EPWM_INT_TBCTR_PERIOD);
    EPWM_setInterruptEventCount(base, 1);
    EPWM_clearEventTriggerInterruptFlag(base);
    EPWM_enableInterrupt(base);

    /* configuring ADC */
    uint32_t adc_instance = 0;
    util_adc_reset(adc_instance);
    util_adc_init_configure_soc_source(adc_instance, i);

    EPWM_setTimeBaseCounterMode(base, EPWM_COUNTER_MODE_UP);  /* Start the counter */
    while(gnumIsrCnt < 10)
    {
        SemaphoreP_pend(&gEpwmSyncSemObject, SystemP_WAIT_FOREVER);
    }
    EPWM_setTimeBaseCounterMode(base, EPWM_COUNTER_MODE_STOP_FREEZE);  /* Stop the counter */

    diffA = gnumIsrCnt - 5 * gAdc_ISR1_count;
    diffB = gnumIsrCnt - 5 * gAdc_ISR2_count;

    if(enableLog)
    {
        DebugP_log("gnumIsrCnt = %d, gAdc_ISR1_count = %d, gAdc_ISR2_count = %d\r\n ", gnumIsrCnt, gAdc_ISR1_count, gAdc_ISR2_count);
        DebugP_log("diffA = %d, diffB = %d\r\n ", diffA, diffB);
    }

    if( diffA > 1 || diffB > 1)
    {
        error++;
    }
    //****************************************************************************************//
    HwiP_destruct(&gEpwmHwiObject);
    HwiP_destruct(&gAdcHwiObject1);
    HwiP_destruct(&gAdcHwiObject2);
    SemaphoreP_destruct(&gEpwmSyncSemObject);
    util_deinit_epwms(i);
    util_adc_reset(adc_instance);

    if(error==0)
    {
        if(enableLog)
        {
            DebugP_log("\r\nPass");
        }
        return 0;
    }
    else
    {
        if(enableLog)
        {
            DebugP_log("\r\nFail");
        }
        return 1;
    }
}
/*  EPWM_diode_emulation_and_mindb */
int32_t AM263x_EPWM_xTR_0017(uint32_t base, uint32_t i)
{
   //TBD
    int32_t error=0;

    EPWM_enableMinimumDeadBand(base, EPWM_MINDB_BLOCK_A);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG32(base + CSL_EPWM_MINDBCFG)
    & CSL_EPWM_MINDBCFG_ENABLEA_MASK) >> CSL_EPWM_MINDBCFG_ENABLEA_SHIFT, CSL_EPWM_MINDBCFG_ENABLEA_MAX);

    EPWM_invertMinimumDeadBandSignal(base, EPWM_MINDB_BLOCK_B, 1);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG32(base + CSL_EPWM_MINDBCFG)
    & CSL_EPWM_MINDBCFG_INVERTB_MASK) >> CSL_EPWM_MINDBCFG_INVERTB_SHIFT, CSL_EPWM_MINDBCFG_INVERTB_MAX);

    EPWM_selectMinimumDeadBandAndOrLogic(base, EPWM_MINDB_BLOCK_B, 1);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG32(base + CSL_EPWM_MINDBCFG)
    & CSL_EPWM_MINDBCFG_POLSELB_MASK) >> CSL_EPWM_MINDBCFG_POLSELB_SHIFT, CSL_EPWM_MINDBCFG_POLSELB_MAX);

    EPWM_selectMinimumDeadBandBlockingSignal(base, EPWM_MINDB_BLOCK_B, 1);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG32(base + CSL_EPWM_MINDBCFG)
    & CSL_EPWM_MINDBCFG_SELBLOCKB_MASK) >> CSL_EPWM_MINDBCFG_SELBLOCKB_SHIFT, CSL_EPWM_MINDBCFG_SELBLOCKB_MAX);

    EPWM_selectMinimumDeadBandReferenceSignal(base, EPWM_MINDB_BLOCK_A, 12);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG32(base + CSL_EPWM_MINDBCFG)
    & CSL_EPWM_MINDBCFG_SELA_MASK) >> CSL_EPWM_MINDBCFG_SELA_SHIFT, 12);

    EPWM_setMinDeadBandDelay(base, EPWM_MINDB_BLOCK_A, 65477);
    if(!((HW_RD_REG32(base + CSL_EPWM_MINDBDLY) == (uint32_t)EPWM_getMinDeadBandDelay(base, EPWM_MINDB_BLOCK_A))
    && (HW_RD_REG32(base + CSL_EPWM_MINDBDLY)) == 65477))
        error++;

    EPWM_disableMinimumDeadBand(base, EPWM_MINDB_BLOCK_A);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG32(base + CSL_EPWM_MINDBCFG)
    & CSL_EPWM_MINDBCFG_ENABLEA_MASK) >> CSL_EPWM_MINDBCFG_ENABLEA_SHIFT, 0x0);

    /*util_deinit_epwms(i);*/

    if(error==0)
    {
        if(enableLog)
        {
            DebugP_log("\r\nPass");
        }
        return 0;
    }
    else
    {
        if(enableLog)
        {
            DebugP_log("\r\nFail");
        }
        return 1;
    }
}

/*  EPWM_illegal_combo_logic */
int32_t AM263x_EPWM_xTR_0019(uint32_t base)
{
    //TBD
    int32_t error=0;
    EPWM_enableIllegalComboLogic(base, EPWM_MINDB_BLOCK_A);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG32(base + CSL_EPWM_LUTCTLA)
    & CSL_EPWM_LUTCTLA_BYPASS_MASK) >> CSL_EPWM_LUTCTLA_BYPASS_SHIFT, 0x0);

    EPWM_selectXbarInput(base, EPWM_MINDB_BLOCK_A, 6);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG32(base + CSL_EPWM_LUTCTLA)
    & CSL_EPWM_LUTCTLA_SELXBAR_MASK) >> CSL_EPWM_LUTCTLA_SELXBAR_SHIFT, 0x6);

    EPWM_setLutDecX(base, EPWM_MINDB_BLOCK_A, 3, 1);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG32(base + CSL_EPWM_LUTCTLA)
    & CSL_EPWM_LUTCTLA_LUTDEC3_MASK) >> CSL_EPWM_LUTCTLA_LUTDEC3_SHIFT, CSL_EPWM_LUTCTLA_LUTDEC3_MAX);

    EPWM_disableIllegalComboLogic(base, EPWM_MINDB_BLOCK_A);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG32(base + CSL_EPWM_LUTCTLA)
    & CSL_EPWM_LUTCTLA_BYPASS_MASK) >> CSL_EPWM_LUTCTLA_BYPASS_SHIFT, CSL_EPWM_LUTCTLA_BYPASS_MAX);

    //util_deinit_epwms();

    if(error==0)
    {
        if(enableLog)
        {
            DebugP_log("\r\nPass");
        }
        return 0;
    }
    else
    {
        if(enableLog)
        {
            DebugP_log("\r\nFail");
        }

        return 1;
    }
}

/*  EPWM_epwmxlinkxload_feature */
int32_t AM263x_EPWM_xTR_0020(uint32_t base)
{
    /*
    ePWMs are grouped into odd and even. Initally all are configured with some value
    in TBPRD, CMP and XTBPRD and XCMP.
    The even ePWMs will be linked to their immediate next odd ePWMs.
    So writing 1 to XLOAD_STARTLD of odd ePWMs will simultaneously write to the
    even ones.
    Setting XLOAD_STARTLD means triggering the loading from shadow to active registers.
    We check this functionality by testing the value of active registers once BEFORE forcing this
    reload event in the odd ePWMs and once AFTER the forcing.
    BEFORE forcing odd ePWMs, even ePWMs should have XTBPRD_ACTIVE --> TBPRD and XCMP1_ACTIVE --> CMPA.
    AFTER forcing odd ePWMs, even ePWMs should have XTBPRD_SHDWx --> TBPRD and XCMP1_SHDWx --> CMPA.
    */

    int32_t i, j, error=0, shdwlevel;

    for(i = 0 ; i< 32; i++)
    {
        uint32_t base = CSL_CONTROLSS_G0_EPWM0_U_BASE + i * 0x1000;
        EPWM_setTimeBasePeriod(base, 3); //The shadow to active loading occurs 3 cycles before CNT_ZERO event
        EPWM_setTimeBaseCounterMode(base, EPWM_COUNTER_MODE_UP);

        EPWM_enableXCMPMode(base);
        EPWM_allocAXCMP(base, EPWM_XCMP_1_CMPA);
        EPWM_setXCMPLoadMode(base, EPWM_XCMP_XLOADCTL_LOADMODE_LOADONCE);
        EPWM_setXCMPShadowLevel(base, EPWM_XCMP_XLOADCTL_SHDWLEVEL_3);
        EPWM_setXCMPShadowRepeatBufxCount(base, EPWM_XCMP_SHADOW3, 1);
        EPWM_setXCMPShadowRepeatBufxCount(base, EPWM_XCMP_SHADOW2, 1);

        for(shdwlevel = 0; shdwlevel <= 3*(EPWM_XCMP1_SHADOW2-EPWM_XCMP1_SHADOW1); shdwlevel+=(EPWM_XCMP1_SHADOW2-EPWM_XCMP1_SHADOW1))
        {
            uint16_t cmp_val = 100*(shdwlevel + 10 + EPWM_XCMP1_ACTIVE); //equation to generate some random value
            uint16_t tbprd_val = 100*(shdwlevel + 10 + EPWM_XTBPRD_ACTIVE); //equation to generate some random value
            EPWM_setXCMPRegValue(base,(shdwlevel + EPWM_XCMP1_ACTIVE), cmp_val);
            EPWM_setXCMPRegValue(base,(shdwlevel + EPWM_XTBPRD_ACTIVE), tbprd_val);
        }

        if(i%2 == 0)  //Link the even instances to odd ones
            EPWM_setupEPWMLinks(base, i+1, EPWM_LINK_XLOAD);
    }

    ClockP_usleep(1000);

    /* Check the value of the active registers for the even ePWMs*/
    if(enableLog)
    {
        DebugP_log("***************** For Active register set *****************\r\n");
    }

    for(i = 0 ; i< 32; i+=2)
    {
        shdwlevel= 0;
        uint16_t expected_cmp_val = 100*(shdwlevel + 10 + EPWM_XCMP1_ACTIVE);
        uint16_t expected_tbprd_val = 100*(shdwlevel + 10 + EPWM_XTBPRD_ACTIVE);
        uint32_t base = CSL_CONTROLSS_G0_EPWM0_U_BASE + i * 0x1000;
        uint16_t tbprd = EPWM_getTimeBasePeriod(base);
        uint16_t cmpA = EPWM_getCounterCompareValue(base, EPWM_COUNTER_COMPARE_A);
        if(enableLog)
        {
            DebugP_log("EPWM %u, Expected TBPRD = %u , observed TBPRD = %u, Expected CMPA = %u , Observed CMPA = %u \r\n",
                i, expected_tbprd_val, tbprd, expected_cmp_val, cmpA);
        }
        if( tbprd != expected_tbprd_val )
        {
            error++;
        }
        if( cmpA != expected_cmp_val)
        {
            error++;
        }
    }

    if(enableLog)
    {
        DebugP_log("***************** For Shadow register set *****************\r\n");
    }

    /* start reload event */
    for(i = 1 ; i< 32; i+=2)
    {
        uint32_t odd_base = CSL_CONTROLSS_G0_EPWM0_U_BASE + i * 0x1000;
        uint32_t even_base = CSL_CONTROLSS_G0_EPWM0_U_BASE + (i-1) * 0x1000;

        /*Setting XLOAD_STARTLD bit of odd ePWMs will also write 1 to the
        corresponding even ePWMs being linked to
        */

        for(j = 3; j>=1; j--)
        {
            EPWM_setXCMPShadowBufPtrLoadOnce(even_base, j);
            EPWM_enableXLoad(odd_base);

            /* Check the value of the active registers of even ePWMs to see
            if the shadow to active loading has happened on each reload event triggered in odd ePWMs*/
            {
                shdwlevel = j *(EPWM_XCMP1_SHADOW2-EPWM_XCMP1_SHADOW1);
                uint16_t expected_cmp_val = 100*(shdwlevel + 10 + EPWM_XCMP1_ACTIVE);
                uint16_t expected_tbprd_val = 100*(shdwlevel + 10 + EPWM_XTBPRD_ACTIVE);
                ClockP_usleep(1000);
                uint16_t tbprd = EPWM_getTimeBasePeriod(even_base);
                uint16_t cmpA = EPWM_getCounterCompareValue(even_base, EPWM_COUNTER_COMPARE_A);
                if(enableLog)
                {
                    DebugP_log("EPWM %u, Expected TBPRD = %u , observed TBPRD = %u, Expected CMPA = %u , Observed CMPA = %u \r\n",
                (i-1), expected_tbprd_val, tbprd, expected_cmp_val, cmpA);
                }

                if( tbprd != expected_tbprd_val )
                {
                    error++;
                }
                if(cmpA != expected_cmp_val)
                {
                    error++;
                }
            }
        }
        if(enableLog)
        {
            DebugP_log("**************************************************\r\n");
        }
        util_deinit_epwms(i-1);
        util_deinit_epwms(i);
    }



    if(error==0)
    {
        if(enableLog) DebugP_log("\r\nPass");

        return 0;
    }
    else
    {
        if(enableLog) DebugP_log("\r\nFail");

        return 1;
    }
}
/* EPWM_pwm_latency_basic */
int32_t AM263x_EPWM_xTR_0024(uint32_t base, uint32_t i)
{
    int errors = 0;

    volatile uint64_t counter_start_stop_latency = 0;
    volatile uint64_t epwm_flag_read_latency = 0;
    volatile uint64_t epwm_compare_value_write_latency = 0;

    volatile uint16_t flag_read_dummy = 0;

    for(int test_iter = 2; test_iter > 0; test_iter--)
    {
        CycleCounterP_reset();
        counter_start_stop_latency = CycleCounterP_getCount32();
        CycleCounterP_reset();
        flag_read_dummy = EPWM_getTripZoneFlagStatus(base);
        epwm_flag_read_latency = CycleCounterP_getCount32() - counter_start_stop_latency;

        CycleCounterP_reset();
        EPWM_setTimeBaseCounter(base, 0xFFFF);
        epwm_compare_value_write_latency = CycleCounterP_getCount32() - counter_start_stop_latency;
    }

    if(enableLog)
    {
        DebugP_log("Value:%u and %d is epwm_flag_read_latency\r\r\n", flag_read_dummy, epwm_flag_read_latency);
    }
    /* observation around 25 cycles*/
    if((epwm_flag_read_latency < 21) || (epwm_flag_read_latency > 28))
    {
        errors++;
    }


    if(enableLog)
    {
        DebugP_log("%d is epwm_compare_value_write_latency\r\r\n", epwm_compare_value_write_latency);
    }

    /* observation 11 cycles*/
    if((epwm_compare_value_write_latency < 9) || (epwm_compare_value_write_latency > 13))
    {
        errors++;
    }
   /*  util_deinit_epwms(); */
    if(errors > 0)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}
//EPWM_pwm_latency_through_r5f_cores
/*
int32_t AM263x_EPWM_xTR_0025(uint32_t base, uint32_t i)
{
    int errors = 0;
    uint64_t counter_start_stop_latency = 0;
    uint64_t epwm_result_write_latency = 0;
    uint32_t result_addr = base + CSL_EPWM_CMPA;

    CycleCounterP_reset();
    counter_start_stop_latency = CycleCounterP_getCount32();

    for(i = 0; i< 70; i++)
    {
        CycleCounterP_reset();
        HW_WR_REG16(result_addr, 0x10*i); //Writing some value to ePWM register
        epwm_result_write_latency += (CycleCounterP_getCount32() - counter_start_stop_latency);
    }

    util_deinit_epwms();

    if(errors > 0)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}
*/
/* EPWM_pwm_latency_through_dma */
int32_t AM263x_EPWM_xTR_0026(uint32_t base, uint32_t i)
{
    return 0;
}

 /* EPWM_support_pwm_chopping */
int32_t AM263x_EPWM_xTR_0032(uint32_t base, uint32_t i)
{
    return AM263x_EPWM_xTR_0006(base, i);
}

void ECAP_inApwmMode(uint32_t base)
{
	/* Stops Time stamp counter */
	ECAP_stopCounter(base);
	/* Sets eCAP in APWM mode */
	ECAP_enableAPWMMode(base);
	/* Set eCAP APWM period */
	ECAP_setAPWMPeriod(base,2000);
	/* Set eCAP APWM on or off time count */
	ECAP_setAPWMCompare(base,500);
	/* Set eCAP APWM polarity */
	ECAP_setAPWMPolarity(base,ECAP_APWM_ACTIVE_HIGH);
	/* Configures Sync out signal mode */
	ECAP_setSyncOutMode(base,ECAP_SYNC_OUT_COUNTER_PRD);
	/* Starts Time stamp counter for CONFIG_ECAP0 */
    SOC_xbarSelectOutputXBarInputSource(CSL_CONTROLSS_OUTPUTXBAR_U_BASE, 14, 0, 0, 0, 0, 0, 0, 0, 0, 0, ( OUTPUT_XBAR_ECAP0_OUT ), 0);

    HW_WR_REG32(CSL_IOMUX_U_BASE + CSL_IOMUX_IO_CFG_KICK0,0x83E70B13);
    HW_WR_REG32(CSL_IOMUX_U_BASE + CSL_IOMUX_IO_CFG_KICK1,0x95A4F1E0);
    HW_WR_REG32(CSL_IOMUX_U_BASE + CSL_IOMUX_PR0_PRU1_GPO18_CFG_REG, 0x5);

	ECAP_startCounter(base);
}

/* EPWM_add_additional_syncin_sources */
int32_t AM263x_EPWM_xTR_0033(uint32_t base)
{
    //TBD
    int32_t error=0;

    EPWM_setSyncInPulseSource(base, EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_ECAP1);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG16(base + CSL_EPWM_EPWMSYNCINSEL)
    & CSL_EPWM_EPWMSYNCINSEL_SEL_MASK) >> CSL_EPWM_EPWMSYNCINSEL_SEL_SHIFT, EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_ECAP1);

   /*  util_deinit_epwms(); */

    //===================================
    /*uint32_t tbprd = 2000, cmpA = 500, cmpB = 500;
    util_setEPWMTB(base, EPWM_EMULATION_FREE_RUN, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1, tbprd, EPWM_COUNTER_MODE_UP);
    util_setEPWMCC(base, cmpA, cmpB);
    EPWM_enablePhaseShiftLoad(base);
    EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setSyncInPulseSource(base, EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_ECAP0);
    ECAP_inApwmMode(CSL_CONTROLSS_ECAP0_U_BASE);
    */
    //===================================

    if(error==0)
    {
        if(enableLog)
        {
            DebugP_log("\r\nPass");
        }
        return 0;
    }
    else
    {
        if(enableLog)
        {
            DebugP_log("\r\nFail");
        }
        return 1;
    }
}

/*  EPWM_functionalities_for_mdl_and_icl */
int32_t AM263x_EPWM_xTR_0034(uint32_t base, uint32_t i)
{
    return ( AM263x_EPWM_xTR_0017(base, i) && AM263x_EPWM_xTR_0019(base) );
}
/*  EPWM_dc_cbc_latch */
int32_t AM263x_EPWM_xTR_0035(uint32_t base)
{
    //TBD
    int32_t error=0;
    EPWM_setDigitalCompareCBCLatchMode(base, EPWM_DC_MODULE_B, EPWM_DC_EVENT_2, EPWM_DC_CBC_LATCH_ENABLED);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG16(base + CSL_EPWM_DCBCTL)
    & CSL_EPWM_DCBCTL_EVT2LATSEL_MASK) >> CSL_EPWM_DCBCTL_EVT2LATSEL_SHIFT, EPWM_DC_CBC_LATCH_ENABLED);

    EPWM_selectDigitalCompareCBCLatchClearEvent(base, EPWM_DC_MODULE_A, EPWM_DC_EVENT_2, EPWM_DC_CBC_LATCH_CLR_ON_CNTR_ZERO_PERIOD);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG16(base + CSL_EPWM_DCACTL)
    & CSL_EPWM_DCACTL_EVT2LATCLRSEL_MASK) >> CSL_EPWM_DCACTL_EVT2LATCLRSEL_SHIFT, EPWM_DC_CBC_LATCH_CLR_ON_CNTR_ZERO_PERIOD);

    /* util_deinit_epwms(); */

    if(error==0)
    {
        if(enableLog)
        {
            DebugP_log("\r\nPass");
        }
        return 0;
    }
    else
    {
        if(enableLog)
        {
            DebugP_log("\r\nFail");
        }
        return 1;
    }
}
/* EPWM_set_trip_zone_action */
int32_t AM263x_EPWM_xTR_0036(uint32_t base, uint32_t i)
{
    //TBD
    int32_t error=0;
    EPWM_setTripZoneAdvAction(base, EPWM_TZ_ADV_ACTION_EVENT_TZB_D, EPWM_TZ_ADV_ACTION_HIGH);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG16(base + CSL_EPWM_TZCTL2)
    & CSL_EPWM_TZCTL2_TZBD_MASK) >> CSL_EPWM_TZCTL2_TZBD_SHIFT, EPWM_TZ_ADV_ACTION_HIGH);

	EPWM_setTripZoneAdvDigitalCompareActionA(base, EPWM_TZ_ADV_ACTION_EVENT_DCxEVT1_D, EPWM_TZ_ADV_ACTION_LOW);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG16(base + CSL_EPWM_TZCTLDCA)
    & CSL_EPWM_TZCTLDCA_DCAEVT1D_MASK) >> CSL_EPWM_TZCTLDCA_DCAEVT1D_SHIFT, EPWM_TZ_ADV_ACTION_LOW);

    EPWM_setTripZoneAdvDigitalCompareActionB(base, EPWM_TZ_ADV_ACTION_EVENT_DCxEVT1_D, EPWM_TZ_ADV_ACTION_LOW);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG16(base + CSL_EPWM_TZCTLDCB)
    & CSL_EPWM_TZCTLDCB_DCBEVT1D_MASK) >> CSL_EPWM_TZCTLDCB_DCBEVT1D_SHIFT, EPWM_TZ_ADV_ACTION_LOW);

    TEST_ASSERT_EQUAL_INT32((HW_RD_REG16(base + CSL_EPWM_TZCTL2)
    & CSL_EPWM_TZCTL2_ETZE_MASK) >> CSL_EPWM_TZCTL2_ETZE_SHIFT, 0x1);

    EPWM_enableTripZoneAdvAction(base);
    TEST_ASSERT_EQUAL_INT32(HW_RD_REG16(base + CSL_EPWM_TZCTL2) >> CSL_EPWM_TZCTL2_ETZE_SHIFT,
    CSL_EPWM_TZCTL2_ETZE_MAX);

	EPWM_enableTripZoneSignals(base, EPWM_TZ_SIGNAL_OSHT2 | EPWM_TZ_SIGNAL_OSHT3);
	TEST_ASSERT_EQUAL_INT32(HW_RD_REG16(base + CSL_EPWM_TZSEL),
    CSL_EPWM_TZSEL_OSHT2_MASK | CSL_EPWM_TZSEL_OSHT3_MASK);

    EPWM_enableTripZoneSignals(base, EPWM_TZ_SIGNAL_CBC5 | EPWM_TZ_SIGNAL_CBC6);
	TEST_ASSERT_EQUAL_INT32(HW_RD_REG16(base + CSL_EPWM_TZSEL) &
    (CSL_EPWM_TZSEL_CBC5_MASK | CSL_EPWM_TZSEL_CBC6_MASK), (CSL_EPWM_TZSEL_CBC5_MASK | CSL_EPWM_TZSEL_CBC6_MASK));

    EPWM_selectCycleByCycleTripZoneClearEvent(base, EPWM_TZ_CBC_PULSE_CLR_CNTR_PERIOD);
    /*TEST_ASSERT_EQUAL_INT32((HW_RD_REG16(base + CSL_EPWM_TZFLG)
    & CSL_EPWM_TZFLG_CBCPULSE_MASK) >> CSL_EPWM_TZFLG_CBCPULSE_SHIFT, 0x0);*/

    EPWM_enableTripZoneInterrupt(base, EPWM_TZ_INTERRUPT_DCAEVT2);
    TEST_ASSERT_EQUAL_INT32(HW_RD_REG16(base + CSL_EPWM_TZEINT), EPWM_TZ_INTERRUPT_DCAEVT2);

    EPWM_forceTripZoneEvent(base, EPWM_TZ_FORCE_EVENT_CBC);
    TEST_ASSERT_EQUAL_INT32(HW_RD_REG16(base + CSL_EPWM_TZFLG), EPWM_getTripZoneFlagStatus(base));

    EPWM_enableTripZoneOutput(base, EPWM_TZ_SELECT_TRIPOUT_CBC);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG16(base + CSL_EPWM_TZTRIPOUTSEL)
    & CSL_EPWM_TZTRIPOUTSEL_CBC_MASK) >> CSL_EPWM_TZTRIPOUTSEL_CBC_SHIFT, 0x1);

    //*************************************De-initializing*************************************//
    EPWM_disableTripZoneSignals(base, EPWM_TZ_SIGNAL_OSHT2 | EPWM_TZ_SIGNAL_OSHT3 | EPWM_TZ_SIGNAL_CBC5 | EPWM_TZ_SIGNAL_CBC6);
	TEST_ASSERT_EQUAL_INT32(HW_RD_REG16(base + CSL_EPWM_TZSEL), 0x0);

    EPWM_disableTripZoneAdvAction(base);
    TEST_ASSERT_EQUAL_INT32(HW_RD_REG16(base + CSL_EPWM_TZCTL2) >> CSL_EPWM_TZCTL2_ETZE_SHIFT, 0x0);

    EPWM_disableTripZoneInterrupt(base, EPWM_TZ_INTERRUPT_DCAEVT2);
    TEST_ASSERT_EQUAL_INT32(HW_RD_REG16(base + CSL_EPWM_TZEINT), 0x0);

    EPWM_disableTripZoneOutput(base, EPWM_TZ_SELECT_TRIPOUT_CBC);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG16(base + CSL_EPWM_TZTRIPOUTSEL)
    & CSL_EPWM_TZTRIPOUTSEL_CBC_MASK) >> CSL_EPWM_TZTRIPOUTSEL_CBC_SHIFT, 0x0);
    //****************************************************************************************//

    util_deinit_epwms(i);

    if(error==0)
    {
        if(enableLog)
        {
            DebugP_log("\r\nPass");
        }
        return 0;
    }
    else
    {
        if(enableLog)
        {
            DebugP_log("\r\nFail");
        }
        return 1;
    }
}
void util_EPWM_setup_int_trigger(uint32_t epwm_base, uint32_t trigger_source, uint16_t prescale)
{
    if(trigger_source != EPWM_INT_TBCTR_ETINTMIX)
    {
        EPWM_setInterruptSource(epwm_base, trigger_source, 0);
    }
    else
    {
        EPWM_setInterruptSource(epwm_base, EPWM_INT_TBCTR_ETINTMIX, 0x3FF);  // Select all the sources
    }
    EPWM_setInterruptEventCount(epwm_base, prescale);
    EPWM_clearEventTriggerInterruptFlag(epwm_base);
    EPWM_enableInterrupt(epwm_base);
}
/* EPWM_generation_of_adc_soc_and_cpu_interrupt_on_all_events */
int32_t AM263x_EPWM_xTR_0037(uint32_t base, uint32_t i)
{
    /* Iterating through all the sources for triggering Interrupt from EPWM and triggering SOCA and SOCB.
       EPWM to generate interrupt on occurence of an event and the same event is used to trigger SOCA and SOCB.
       After doing all the configurations, the EPWM counter is allowed to count for 1 period cycle.
       Inside EPWM ISR we maintain a counter to check the number of times interrupt has been issued which implies
       the number of times the event has occured in the duration of 1 TBPRD (after 1 TBPRD we stop the counter and disable interrupts and SOCs).
       Similarly, on the ADC side, on EOCA and EOCB, ADC generates interrupts and their corresponding ISRs also increment their
       respective counters.
       At the end of one Period, we compare deviation in the counters incremented inside both EPWM and ADC ISRs. They shouldn't be
       differing by more than 1 as the events triggering all interrupts are same.
    */

    int32_t error=0, itr, adc_event, int_event, diffA =0 , diffB = 0;

    //----------------------------------------Configuring the Interrupt---------------------------------------------------------------
    /* Register & enable interrupt */
    /* setting up epwm interrupt to route through int xbar 0*/
    SOC_xbarSelectInterruptXBarInputSource(CSL_CONTROLSS_INTXBAR_U_BASE, 0, ( 1<<i ), 0, 0, 0, 0, 0, 0);

    uint32_t status = SemaphoreP_constructBinary(&gEpwmSyncSemObject, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = (CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_0);
    hwiPrms.callback    = &Adc_epwmIntrISR;
    hwiPrms.args        = (void *)base;
    hwiPrms.isPulse     = 1;
    status              = HwiP_construct(&gEpwmHwiObject, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);

    /* setting up ADCINT1 to route through int xbar 1*/
    SOC_xbarSelectInterruptXBarInputSource(CSL_CONTROLSS_INTXBAR_U_BASE, 1, 0, 0,(INT_XBAR_ADC0_INT1),0, 0, 0, 0);

    /* Initialising a Interrupt parameter
     * setting up the interrupt, callbacks.*/
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_1;
    hwiPrms.callback    = &Adc_IntrISR1;
    hwiPrms.isPulse     = 1;
    status              = HwiP_construct(&gAdcHwiObject1, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);

    /* setting up ADCINT2 to route through int xbar 2*/
    SOC_xbarSelectInterruptXBarInputSource(CSL_CONTROLSS_INTXBAR_U_BASE, 2, 0, 0,(INT_XBAR_ADC0_INT2),0, 0, 0, 0);

    /* Initialising a Interrupt parameter
     * setting up the interrupt, callbacks.*/
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_2;
    hwiPrms.callback    = &Adc_IntrISR2;
    hwiPrms.isPulse     = 1;
    status              = HwiP_construct(&gAdcHwiObject2, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);
    //---------------------------------------------------------------------------------------------------------------------------------


    /* configuring EPWM */
    uint32_t int_trigger_sources[11] = {EPWM_INT_TBCTR_ZERO, EPWM_INT_TBCTR_PERIOD, EPWM_INT_TBCTR_U_CMPA,
                        EPWM_INT_TBCTR_U_CMPC, EPWM_INT_TBCTR_D_CMPA, EPWM_INT_TBCTR_D_CMPC,
                        EPWM_INT_TBCTR_U_CMPB, EPWM_INT_TBCTR_U_CMPD, EPWM_INT_TBCTR_D_CMPB,
                        EPWM_INT_TBCTR_D_CMPD, EPWM_INT_TBCTR_ETINTMIX};

    uint32_t adc_soc_sources[12] = {EPWM_SOC_TBCTR_ZERO, EPWM_SOC_TBCTR_PERIOD, EPWM_SOC_TBCTR_U_CMPA,
                        EPWM_SOC_TBCTR_U_CMPC, EPWM_SOC_TBCTR_D_CMPA, EPWM_SOC_TBCTR_D_CMPC,
                        EPWM_SOC_TBCTR_U_CMPB, EPWM_SOC_TBCTR_U_CMPD, EPWM_SOC_TBCTR_D_CMPB,
                        EPWM_SOC_TBCTR_D_CMPD, EPWM_SOC_TBCTR_MIXED_EVENT, EPWM_SOC_DCxEVT1};

    /* configuring ADC */
    uint32_t adc_instance = 0;
    util_adc_reset(adc_instance);
    util_adc_init_configure_soc_source(adc_instance, i);

    for(itr = 0 ; itr < 11; itr++)
    {
        gAdc_ISR1_count = 0;
        gAdc_ISR2_count = 0;
        gnumIsrCnt = 0;

        util_setEPWMTB(base, EPWM_EMULATION_FREE_RUN, EPWM_CLOCK_DIVIDER_128, EPWM_HSCLOCK_DIVIDER_14, 65535, EPWM_COUNTER_MODE_STOP_FREEZE);
        util_setEPWMCC(base, 15000, 25000);
        EPWM_setCounterCompareValue(base, EPWM_COUNTER_COMPARE_C, 35000);
        EPWM_setCounterCompareValue(base, EPWM_COUNTER_COMPARE_D, 45000);

        int_event = int_trigger_sources[itr];
        adc_event = adc_soc_sources[itr];

        if(enableLog)
        {
            DebugP_log("---------------------------Event is %d---------------------------\r\n", int_event);
        }

        util_EPWM_setup_adc_trigger(base, EPWM_SOC_A, adc_event, 1);
        util_EPWM_setup_adc_trigger(base, EPWM_SOC_B, adc_event, 1);
        util_EPWM_setup_int_trigger(base, int_event, 1);

        EPWM_setTimeBaseCounterMode(base, EPWM_COUNTER_MODE_UP_DOWN);  /* Start the counter */
        ClockP_usleep(1200000); //Wait for ~1 TBPRD

        EPWM_setTimeBaseCounterMode(base, EPWM_COUNTER_MODE_STOP_FREEZE);  /* Stop the counter */

        diffA = abs(gnumIsrCnt - gAdc_ISR1_count);
        diffB = abs(gnumIsrCnt - gAdc_ISR2_count);

        if(enableLog)
        {
            DebugP_log("gnumIsrCnt = %d, gAdc_ISR1_count = %d, gAdc_ISR2_count = %d\r\n ", gnumIsrCnt, gAdc_ISR1_count, gAdc_ISR2_count);
            DebugP_log("diffA = %d, diffB = %d\r\n ", diffA, diffB);
        }

        if(diffA > 1 || diffB > 1)
        {
            error++;
        }

        util_deinit_epwms(i);
    }

    if(error==0)
    {
        if(enableLog)
        {
            DebugP_log("\r\nPass");
        }
        return 0;
    }
    else
    {
        if(enableLog)
        {
            DebugP_log("\r\nFail");
        }
        return 1;
    }
}

 /* EPWM_generating_pwm_signals_using_16_bit_counter */
int32_t AM263x_EPWM_xTR_0038(uint32_t base)
{
    return (AM263x_EPWM_xTR_0007(base));
}
 /* EPWM_programming_event_prescaling */
int32_t AM263x_EPWM_xTR_0039(uint32_t base,uint32_t i)
{
    //TBD
    int32_t error = 0;

    EPWM_setGlobalLoadEventPrescale(base, 6);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG16(base + CSL_EPWM_GLDCTL)
    & CSL_EPWM_GLDCTL_GLDPRD_MASK) >> CSL_EPWM_GLDCTL_GLDPRD_SHIFT , 6);

    util_deinit_epwms(i);

    if(error==0)
    {
        if(enableLog)
        {
            DebugP_log("\r\nPass");
        }
        return 0;
    }
    else
    {
        if(enableLog)
        {
            DebugP_log("\r\nFail");
        }
        return 1;
    }
}
 /* EPWM_one_shot_sync_out_trigger */
int32_t AM263x_EPWM_xTR_0040(uint32_t base, uint32_t i)
{
    //TBD
    int32_t error=0;

    return 0;
    EPWM_enableOneShotSync(base);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG16(base + CSL_EPWM_TBCTL2)
    & CSL_EPWM_TBCTL2_OSHTSYNCMODE_MASK) >> CSL_EPWM_TBCTL2_OSHTSYNCMODE_SHIFT , CSL_EPWM_TBCTL2_OSHTSYNCMODE_MAX);

    /*EPWM_startOneShotSync(base);
    */
    EPWM_disableOneShotSync(base);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG16(base + CSL_EPWM_TBCTL2)
    & CSL_EPWM_TBCTL2_OSHTSYNCMODE_MASK) >> CSL_EPWM_TBCTL2_OSHTSYNCMODE_SHIFT , 0x0);

    util_deinit_epwms(i);

    if(error==0)
    {
        if(enableLog)
        {
            DebugP_log("\r\nPass");
        }
        return 0;
    }
    else
    {
        if(enableLog)
        {
            DebugP_log("\r\nFail");
        }
        return 1;
    }
}
/* EPWM_support_asychronous_override_control_of_pwm_signals_through_software */
int32_t AM263x_EPWM_xTR_0042(uint32_t base, uint32_t i)
{
    //TBD
    int32_t error = 0;
    EPWM_setActionQualifierSWAction(CSL_CONTROLSS_G0_EPWM0_U_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG16(CSL_CONTROLSS_G0_EPWM0_U_BASE + CSL_EPWM_AQSFRC)
    & CSL_EPWM_AQSFRC_ACTSFA_MASK) >> CSL_EPWM_AQSFRC_ACTSFA_SHIFT , EPWM_AQ_OUTPUT_HIGH);

    util_deinit_epwms(i);

    if(error==0)
    {
        if(enableLog)
        {
            DebugP_log("\r\nPass");
        }
        return 0;
    }
    else
    {
        if(enableLog)
        {
            DebugP_log("\r\nFail");
        }
        return 1;
    }
}
/* EPWM_support_generation_of_2_pwm_outputs_with_dual_edge_symmetric_operation */
int32_t AM263x_EPWM_xTR_0043(uint32_t base, uint32_t i)
{
    /************************************* SETTING UP TESTER *************************************/
    char epwmA[CMD_SIZE];
    char epwmB[CMD_SIZE];
    char sync_epwmA[CMD_SIZE];
    char sync_epwmB[CMD_SIZE];
    char cap_epwmA[CMD_SIZE];
    char cap_epwmB[CMD_SIZE];
    char ts_epwmA[CMD_SIZE];
    char ts_epwmB[CMD_SIZE];

    if(i<10)
        {
            sprintf(epwmA, "setup epwm 0%u A", i);
            sprintf(epwmB, "setup epwm 0%u B", i);
            sprintf(sync_epwmA, "sync with epwm 0%u A", i);
            sprintf(sync_epwmB, "sync with epwm 0%u B", i);
            sprintf(cap_epwmA, "capture epwm 0%u A ", i);
            sprintf(cap_epwmB, "capture epwm 0%u B ", i);
            sprintf(ts_epwmA, "get timestamp 0%u A", i);
            sprintf(ts_epwmB, "get timestamp 0%u B", i);
        }
    else
        {
            sprintf(epwmA, "setup epwm %u A", i);
            sprintf(epwmB, "setup epwm %u B", i);
            sprintf(sync_epwmA, "sync with epwm %u A", i);
            sprintf(sync_epwmB, "sync with epwm %u B", i);
            sprintf(cap_epwmA, "capture epwm %u A ", i);
            sprintf(cap_epwmB, "capture epwm %u B ", i);
            sprintf(ts_epwmA, "get timestamp %u A", i);
            sprintf(ts_epwmB, "get timestamp %u B", i);
        }

    Configure_Xbar_Sync_DUT(i);
    tester_command(epwmA);
    tester_command(epwmB);
    /*  Ask Tester ECAP to sync */
    tester_command(sync_epwmA); /* EPWMx syncout from DUT syncs EPWM 0 in Tester which in turn syncs ECAP 0 in Tester */
    tester_command(sync_epwmB); /* EPWMx syncout from DUT syncs EPWM 0 in Tester which in turn syncs ECAP 0 in Tester */

    /*****************************************************************************************/
    int32_t error=0;
    /* Configure Time Base */
    /* Configure Counter Compare */
    /* Configure Action Qualifier Events */
    uint32_t tbprd = 0x5DC0, cmpA= 0x4650, cmpB = 0x3A98;

    util_setEPWMTB(base, EPWM_EMULATION_FREE_RUN, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1, tbprd ,EPWM_COUNTER_MODE_STOP_FREEZE); //EPWM_COUNTER_MODE_STOP_FREEZE
    util_setEPWMCC(base, cmpA, cmpB);
    EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
    EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);
    EPWM_enableSyncOutPulseSource(base, EPWM_SYNC_OUT_PULSE_ON_CNTR_ZERO);

    EPWM_setTimeBaseCounterMode(base, EPWM_COUNTER_MODE_UP_DOWN);
    ClockP_usleep(10000);

    /************************************* VALIDATING FROM TESTER *************************************/
    strcat(cap_epwmA, "continuously on edges RFRF in ABSOLUTE");
    tester_command(cap_epwmA);

    strcat(cap_epwmB, "continuously on edges RFRF in ABSOLUTE");
    tester_command(cap_epwmB);

    /* To sync DUT and tester */
    //C15 (Pin 71) on DUT LP connected to HSEC 50

    /* int32_t timestamp_event1_A=0;
    int32_t timestamp_event2_A=0; */
    int32_t timestamp_event3_A=0;
    int32_t timestamp_event4_A=0;

    /* int32_t timestamp_event1_B=0;
    int32_t timestamp_event2_B=0; */
    int32_t timestamp_event3_B=0;
    int32_t timestamp_event4_B=0;

    tester_command(ts_epwmA);
    /* timestamp_event1_A = ts[0]; */ /* Width of first high pulse of ePWMxA */
    /* timestamp_event2_A = ts[1]; */ /* Width of low pulse of ePWMxA */
    timestamp_event3_A = ts[2]; /* Width of high pulse of ePWMxA */
    timestamp_event4_A = ts[3]; /* Width of low pulse of ePWMxA */
    /* if(enableLog)
        {
            DebugP_log("%u %u %u %u\r\n", timestamp_event1_A, timestamp_event2_A, timestamp_event3_A, timestamp_event4_A);
        }
    */

    tester_command(ts_epwmB);
    /* timestamp_event1_B = ts[0]; */ /* Width of first high pulse of ePWMxB */
    /* timestamp_event2_B = ts[1]; */ /*  Width of low pulse of ePWMxB */
    timestamp_event3_B = ts[2]; /* Width of high pulse of ePWMxB */
    timestamp_event4_B = ts[3]; /* Width of low pulse of ePWMxB */

    /* Validating if the timestamp captured by ecap is correct */

    int32_t high_pulse_width_A = timestamp_event4_A - timestamp_event3_A;
    int32_t expected_high_pulse_width_A = (tbprd - cmpA) * 2;
    int32_t  diff_high_pulse_width_A = abs(high_pulse_width_A - expected_high_pulse_width_A);

    int32_t high_pulse_width_B = timestamp_event4_B - timestamp_event3_B;
    int32_t expected_high_pulse_width_B = (tbprd - cmpB) * 2;
    int32_t  diff_high_pulse_width_B = abs(high_pulse_width_B - expected_high_pulse_width_B);

    int32_t rising_edge_diff = timestamp_event3_A - timestamp_event3_B;
    int32_t falling_edge_diff = timestamp_event4_B - timestamp_event4_A;
    int32_t symm_diff = abs(rising_edge_diff - falling_edge_diff);
    int32_t symm_diff_from_ref = abs(rising_edge_diff - (int32_t)(cmpA-cmpB));

    if( (diff_high_pulse_width_A > 10 ) || (diff_high_pulse_width_B > 10 ) || ( symm_diff_from_ref > 10) || (symm_diff > 10))
    {
        error++;
    }
    if(enableLog)
    {
        DebugP_log("%d - %d, %d - %d, %d - %d\r\n",high_pulse_width_A,expected_high_pulse_width_A, high_pulse_width_B,
        expected_high_pulse_width_B, rising_edge_diff, falling_edge_diff);
    }
    /*******************************************************************************************/

    //*************************************De-initializing*************************************//

        util_deinit_epwms(i);
        util_resetEPWMTB(base);
        util_resetEPWMCC(base);
        util_resetEPWMAQ(base);
        //tester_command("De-init ECAP");
    //****************************************************************************************//

        if(error==0)
    {
        if(enableLog)
        {
            DebugP_log("\r\nPass");
        }
        return 0;
    }
    else
    {
        if(enableLog)
        {
            DebugP_log("\r\nFail");
        }
        return 1;
    }
}
 /* EPWM_select_one_shot_sync_out_trigger */
int32_t AM263x_EPWM_xTR_0044(uint32_t base, uint32_t i)
{
    //TBD
    int32_t error=0;
    EPWM_setOneShotSyncOutTrigger(CSL_CONTROLSS_G0_EPWM0_U_BASE, EPWM_OSHT_SYNC_OUT_TRIG_RELOAD);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG16(CSL_CONTROLSS_G0_EPWM0_U_BASE + CSL_EPWM_TBCTL3)
    & CSL_EPWM_TBCTL3_OSSFRCEN_MASK) >> CSL_EPWM_TBCTL3_OSSFRCEN_SHIFT , EPWM_OSHT_SYNC_OUT_TRIG_RELOAD);

    util_deinit_epwms(i);

    if(error==0)
    {
        if(enableLog)
        {
            DebugP_log("\r\nPass");
        }
        return 0;
    }
    else
    {
        if(enableLog)
        {
            DebugP_log("\r\nFail");
        }
        return 1;
    }
}
 /* EPWM_grouping_aliasing_of_epwm_instances */
int32_t AM263x_EPWM_xTR_0045(uint32_t dummy)
{
    int32_t error=0;
    uint8_t i, j;
    uint32_t group_base, epwm_base;

    SOC_setEpwmGroup(0, 0);  //Selecting EPWM0 from group 0
    SOC_setEpwmGroup(1, 1);  //Selecting EPWM1 from group 1
    SOC_setEpwmGroup(2, 2);  //Selecting EPWM2 from group 2
    SOC_setEpwmGroup(3, 3);  //Selecting EPWM3 from group 3

    /* Trying to write to EPWM instances of different groups and verifying if TBPRD is getting written in an instance belonging to a group which isn't selected
    For Eg: Trying to write to G0_EPWM3 is invalid as G3 is selected for EPWM3 so 0x500030C6(TBPRD for EPWM3 Group0) will be 0 but
    0x500C30C6(TBPRD for EPWM3 Group3) will have the value written to it. */
    for(i = 0; i< 4; i++)   /* Iterating through each group */
    {
        group_base = CSL_CONTROLSS_G0_EPWM0_U_BASE + i*0x40000;
        for(j =0; j< 4; j++)  /* Iterating through each epwm instance */
        {
            epwm_base = group_base + j* 0x1000;
            EPWM_setClockPrescaler(epwm_base, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);
            EPWM_setTimeBaseCounterMode(epwm_base, EPWM_COUNTER_MODE_UP);
            EPWM_setTimeBasePeriod(epwm_base, 24000);
        }
    }

    for(i = 0; i< 4; i++) /*  Iterating through each group */
    {
        group_base = CSL_CONTROLSS_G0_EPWM0_U_BASE + i*0x40000;

        for( j = 0; j< 4; j++) /* Iterating through first 4 epwm instances */
        {
            epwm_base = group_base + j* 0x1000;

            volatile uint16_t *reg = (uint16_t *)(epwm_base + CSL_EPWM_TBPRD);

            if(! ((j==i && *reg == 24000) || (j!=i && *reg!=24000))) /* EPWM0_G0_TBPRD = 24000 and (EPWM0_G1_TBPRD, EPWM0_G2_TBPRD, EPWM0_G3_TBPRD) != 24000, Similarly for other EPWM */
                 TEST_ASSERT_EQUAL_INT32(false, true);
        }
    }



    //*************************************De-initializing*************************************//

        for(i = 0; i< 4; i++)  /*  Iterating through each group */
        {
            /*util_deinit_epwms(i);*/
            uint32_t base = CSL_CONTROLSS_G0_EPWM0_U_BASE + i* 0x1000;
            SOC_setEpwmGroup(i,0);
            util_resetEPWMTB(base);
            util_resetEPWMCC(base);
            util_resetEPWMAQ(base);
        }
    //****************************************************************************************//

    if(error==0)
    {
        if(enableLog)
        {
            DebugP_log("\r\nPass");
        }
        return 0;
    }
    else
    {
        if(enableLog)
        {
            DebugP_log("\r\nFail");
        }
        return 1;
    }
}
/* EPWM_XCMP */
int32_t AM263x_EPWM_xTR_0046(uint32_t base, uint32_t i)
{
    int32_t error=0;

    EPWM_enableXCMPMode(base);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG32(base + CSL_EPWM_XCMPCTL1)
    & CSL_EPWM_XCMPCTL1_XCMPEN_MASK) >> CSL_EPWM_XCMPCTL1_XCMPEN_SHIFT , 0x1);

    EPWM_enableSplitXCMP(base);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG32(base + CSL_EPWM_XCMPCTL1)
    & CSL_EPWM_XCMPCTL1_XCMPSPLIT_MASK) >> CSL_EPWM_XCMPCTL1_XCMPSPLIT_SHIFT , 0x1);

    /*EPWM_enableXLoad(base);

    TEST_ASSERT_EQUAL_INT32(((HW_RD_REG32(base + CSL_EPWM_XLOAD)
    & CSL_EPWM_XLOAD_STARTLD_MASK) >> CSL_EPWM_XLOAD_STARTLD_SHIFT) , 0x1);

    EPWM_disableXLoad(base);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG32(base + CSL_EPWM_XLOAD)
    & CSL_EPWM_XLOAD_STARTLD_MASK) >> CSL_EPWM_XLOAD_STARTLD_SHIFT , 0x0);
    */

    EPWM_allocAXCMP(base, EPWM_XCMP_1_CMPA);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG32(base + CSL_EPWM_XCMPCTL1)
    & CSL_EPWM_XCMPCTL1_XCMPA_ALLOC_MASK) >> CSL_EPWM_XCMPCTL1_XCMPA_ALLOC_SHIFT , EPWM_XCMP_1_CMPA);

    EPWM_allocAXCMP(base, EPWM_XCMP_8_CMPA);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG32(base + CSL_EPWM_XCMPCTL1)
    & CSL_EPWM_XCMPCTL1_XCMPA_ALLOC_MASK) >> CSL_EPWM_XCMPCTL1_XCMPA_ALLOC_SHIFT , EPWM_XCMP_8_CMPA);

    EPWM_allocBXCMP(base, EPWM_XCMP_1_CMPB);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG32(base + CSL_EPWM_XCMPCTL1)
    & CSL_EPWM_XCMPCTL1_XCMPB_ALLOC_MASK) >> CSL_EPWM_XCMPCTL1_XCMPB_ALLOC_SHIFT, EPWM_XCMP_1_CMPB);

    EPWM_allocBXCMP(base, EPWM_XCMP_4_CMPB);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG32(base + CSL_EPWM_XCMPCTL1)
    & CSL_EPWM_XCMPCTL1_XCMPB_ALLOC_MASK) >> CSL_EPWM_XCMPCTL1_XCMPB_ALLOC_SHIFT, EPWM_XCMP_4_CMPB);

    EPWM_setXCMPRegValue(base, EPWM_XCMP2_ACTIVE, 0xA);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG32(base + CSL_EPWM_XCMP1_ACTIVE + EPWM_XCMP2_ACTIVE)
    & CSL_EPWM_XCMP1_ACTIVE_XCMP1_ACTIVE_MASK)>> CSL_EPWM_XCMP1_ACTIVE_XCMP1_ACTIVE_SHIFT, 0xA);

    EPWM_setXCMPRegValue(base, EPWM_XCMP2_SHADOW1, 0xFFFF);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG32(base + CSL_EPWM_XCMP1_ACTIVE + EPWM_XCMP2_SHADOW1)
    & CSL_EPWM_XCMP1_ACTIVE_XCMP1_ACTIVE_MASK)>> CSL_EPWM_XCMP1_ACTIVE_XCMP1_ACTIVE_SHIFT, 0xFFFF);

    HRPWM_setXCMPRegValue(base, HRPWM_XCMP2_ACTIVE, 0xB);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG32(base + CSL_EPWM_XCMP1_ACTIVE + HRPWM_XCMP2_ACTIVE)
    & CSL_EPWM_XCMP1_ACTIVE_XCMP1HR_ACTIVE_MASK)>> CSL_EPWM_XCMP1_ACTIVE_XCMP1HR_ACTIVE_SHIFT, 0xB);

    HRPWM_setXCMPRegValue(base, HRPWM_XCMP2_SHADOW1, 0xEEEE);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG32(base + CSL_EPWM_XCMP1_ACTIVE + HRPWM_XCMP2_SHADOW1)
    & CSL_EPWM_XCMP1_ACTIVE_XCMP1HR_ACTIVE_MASK)>> CSL_EPWM_XCMP1_ACTIVE_XCMP1HR_ACTIVE_SHIFT, 0xEEEE);

    EPWM_setXCMPActionQualifierAction(base, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG16(base + CSL_EPWM_XAQCTLA_ACTIVE + EPWM_AQ_OUTPUT_B/2)
    >> EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6) , EPWM_AQ_OUTPUT_HIGH);

    EPWM_setXCMPActionQualifierAction(base, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG16(base + CSL_EPWM_XAQCTLA_ACTIVE + EPWM_AQ_OUTPUT_B/2)
    >> EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6) , EPWM_AQ_OUTPUT_LOW);

    EPWM_setXCMPLoadMode(base, EPWM_XCMP_XLOADCTL_LOADMODE_LOADMULTIPLE);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG32(base + CSL_EPWM_XLOADCTL)
    & CSL_EPWM_XLOADCTL_LOADMODE_MASK) >> CSL_EPWM_XLOADCTL_LOADMODE_SHIFT, EPWM_XCMP_XLOADCTL_LOADMODE_LOADMULTIPLE);

    EPWM_setXCMPShadowLevel(base, EPWM_XCMP_XLOADCTL_SHDWLEVEL_3);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG32(base + CSL_EPWM_XLOADCTL)
    & CSL_EPWM_XLOADCTL_SHDWLEVEL_MASK) >> CSL_EPWM_XLOADCTL_SHDWLEVEL_SHIFT, EPWM_XCMP_XLOADCTL_SHDWLEVEL_3);

    EPWM_setXCMPShadowBufPtrLoadOnce(base, EPWM_XCMP_XLOADCTL_SHDWBUFPTR_TWO);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG32(base + CSL_EPWM_XLOADCTL)
    & CSL_EPWM_XLOADCTL_SHDWBUFPTR_LOADONCE_MASK) >> CSL_EPWM_XLOADCTL_SHDWBUFPTR_LOADONCE_SHIFT, EPWM_XCMP_XLOADCTL_SHDWBUFPTR_TWO);

    EPWM_setXCMPShadowRepeatBufxCount(base, EPWM_XCMP_SHADOW2, 0x7);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG32(base + CSL_EPWM_XLOADCTL)
    & CSL_EPWM_XLOADCTL_RPTBUF2PRD_MASK) >> CSL_EPWM_XLOADCTL_RPTBUF2PRD_SHIFT, 0x7);

    EPWM_setupEPWMLinks(base, EPWM_LINK_WITH_EPWM_27, EPWM_LINK_XLOAD);
    TEST_ASSERT_EQUAL_INT32(HW_RD_REG32(base + CSL_EPWM_EPWMXLINKXLOAD), EPWM_LINK_WITH_EPWM_27);

    EPWM_disableXCMPMode(base);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG32(base + CSL_EPWM_XCMPCTL1)
    & CSL_EPWM_XCMPCTL1_XCMPEN_MASK) >> CSL_EPWM_XCMPCTL1_XCMPEN_SHIFT , 0x0);

    EPWM_disableSplitXCMP(base);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG32(base + CSL_EPWM_XCMPCTL1)
    & CSL_EPWM_XCMPCTL1_XCMPSPLIT_MASK) >> CSL_EPWM_XCMPCTL1_XCMPSPLIT_SHIFT , 0x0);

    // Min Max Registers

    EPWM_setXMINMAXRegValue(base, EPWM_XMIN_ACTIVE, 0xABCD);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG32(base + CSL_EPWM_XMINMAX_ACTIVE)
    & CSL_EPWM_XMINMAX_ACTIVE_XMIN_ACTIVE_MASK)>> CSL_EPWM_XMINMAX_ACTIVE_XMIN_ACTIVE_SHIFT, 0xABCD);

    EPWM_setXMINMAXRegValue(base, EPWM_XMAX_ACTIVE, 0xABCD);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG16(base + CSL_EPWM_XMINMAX_ACTIVE)
    & CSL_EPWM_XMINMAX_ACTIVE_XMAX_ACTIVE_MASK)>> CSL_EPWM_XMINMAX_ACTIVE_XMAX_ACTIVE_SHIFT, 0xABCD);

    EPWM_setXMINMAXRegValue(base, EPWM_XMIN_SHADOW1, 0xABCD);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG32(base + CSL_EPWM_XMINMAX_ACTIVE)
    & CSL_EPWM_XMINMAX_SHDW1_XMIN_SHDW1_MASK)>> CSL_EPWM_XMINMAX_SHDW1_XMIN_SHDW1_SHIFT, 0xABCD);

    EPWM_setXMINMAXRegValue(base, EPWM_XMAX_SHADOW1, 0xABCD);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG32(base + CSL_EPWM_XMINMAX_ACTIVE)
    & CSL_EPWM_XMINMAX_SHDW1_XMAX_SHDW1_MASK)>> CSL_EPWM_XMINMAX_SHDW1_XMAX_SHDW1_SHIFT, 0xABCD);

    EPWM_setXMINMAXRegValue(base, EPWM_XMIN_SHADOW2, 0xABCD);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG32(base + CSL_EPWM_XMINMAX_ACTIVE)
    & CSL_EPWM_XMINMAX_SHDW2_XMIN_SHDW2_MASK)>> CSL_EPWM_XMINMAX_SHDW2_XMIN_SHDW2_SHIFT, 0xABCD);

    EPWM_setXMINMAXRegValue(base, EPWM_XMAX_SHADOW2, 0xABCD);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG32(base + CSL_EPWM_XMINMAX_ACTIVE)
    & CSL_EPWM_XMINMAX_SHDW2_XMAX_SHDW2_MASK)>> CSL_EPWM_XMINMAX_SHDW2_XMAX_SHDW2_SHIFT, 0xABCD);

     EPWM_setXMINMAXRegValue(base, EPWM_XMIN_SHADOW3, 0xABCD);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG32(base + CSL_EPWM_XMINMAX_ACTIVE)
    & CSL_EPWM_XMINMAX_SHDW3_XMIN_SHDW3_MASK)>> CSL_EPWM_XMINMAX_SHDW3_XMIN_SHDW3_SHIFT, 0xABCD);

    EPWM_setXMINMAXRegValue(base, EPWM_XMAX_SHADOW3, 0xABCD);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG32(base + CSL_EPWM_XMINMAX_ACTIVE)
    & CSL_EPWM_XMINMAX_SHDW3_XMAX_SHDW3_MASK)>> CSL_EPWM_XMINMAX_SHDW3_XMAX_SHDW3_SHIFT, 0xABCD);

    //----------------------------------------------------------

    // CMPC and CMPD Shadow registers

    EPWM_setCMPShadowRegValue(base, EPWM_CMPC_SHADOW1, 0xDCAB);
    TEST_ASSERT_EQUAL_INT32(HW_RD_REG16(base + CSL_EPWM_CMPC_SHDW1 + EPWM_CMPC_SHADOW1), 0xDCAB);

    EPWM_setCMPShadowRegValue(base, EPWM_CMPC_SHADOW2, 0xDCAB);
    TEST_ASSERT_EQUAL_INT32(HW_RD_REG16(base + CSL_EPWM_CMPC_SHDW1 + EPWM_CMPC_SHADOW2), 0xDCAB);

    EPWM_setCMPShadowRegValue(base, EPWM_CMPC_SHADOW3, 0xDCAB);
    TEST_ASSERT_EQUAL_INT32(HW_RD_REG16(base + CSL_EPWM_CMPC_SHDW1 + EPWM_CMPC_SHADOW3), 0xDCAB);

    EPWM_setCMPShadowRegValue(base, EPWM_CMPD_SHADOW1, 0xDCAB);
    TEST_ASSERT_EQUAL_INT32(HW_RD_REG16(base + CSL_EPWM_CMPC_SHDW1 + EPWM_CMPD_SHADOW1), 0xDCAB);

    EPWM_setCMPShadowRegValue(base, EPWM_CMPD_SHADOW2, 0xDCAB);
    TEST_ASSERT_EQUAL_INT32(HW_RD_REG16(base + CSL_EPWM_CMPC_SHDW1 + EPWM_CMPD_SHADOW2), 0xDCAB);

    EPWM_setCMPShadowRegValue(base, EPWM_CMPD_SHADOW3, 0xDCAB);
    TEST_ASSERT_EQUAL_INT32(HW_RD_REG16(base + CSL_EPWM_CMPC_SHDW1 + EPWM_CMPD_SHADOW3), 0xDCAB);


    /*util_deinit_epwms(i);*/

    if(error==0)
    {
        if(enableLog)
        {
            DebugP_log("\r\nPass");
        }
        return 0;
    }
    else
    {
        if(enableLog)
        {
            DebugP_log("\r\nFail");
        }
        return 1;
    }
}

/* EPWM_DEL */
int32_t AM263x_EPWM_xTR_0047(uint32_t base, uint32_t i)
{
    int32_t error=0;
    EPWM_enableDiodeEmulationMode(base);
    TEST_ASSERT_EQUAL_INT32( (HW_RD_REG32(base + CSL_EPWM_DECTL)
    & CSL_EPWM_DECTL_ENABLE_MASK) >> CSL_EPWM_DECTL_ENABLE_SHIFT, CSL_EPWM_DECTL_ENABLE_MAX);

    EPWM_setDiodeEmulationMode(base, EPWM_DIODE_EMULATION_OST);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG32(base + CSL_EPWM_DECTL)
    & CSL_EPWM_DECTL_MODE_MASK) >> CSL_EPWM_DECTL_MODE_SHIFT, EPWM_DIODE_EMULATION_OST);

    EPWM_setDiodeEmulationReentryDelay(base, 0xE7 );
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG32(base + CSL_EPWM_DECTL)
    & CSL_EPWM_DECTL_REENTRYDLY_MASK) >> CSL_EPWM_DECTL_REENTRYDLY_SHIFT, 0xE7);

    EPWM_configureDiodeEmulationTripSources(base, EPWM_DE_TRIP_SRC_CMPSSA5, EPWM_DE_TRIPH);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG32(base + CSL_EPWM_DECOMPSEL)
    & CSL_EPWM_DECOMPSEL_TRIPH_MASK) >> CSL_EPWM_DECOMPSEL_TRIPH_SHIFT, EPWM_DE_TRIP_SRC_CMPSSA5);

    EPWM_configureDiodeEmulationTripSources(base, EPWM_DE_TRIP_SRC_INPUTXBAR_OUT27, EPWM_DE_TRIPH);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG32(base + CSL_EPWM_DECOMPSEL)
    & CSL_EPWM_DECOMPSEL_TRIPH_MASK) >> CSL_EPWM_DECOMPSEL_TRIPH_SHIFT, EPWM_DE_TRIP_SRC_INPUTXBAR_OUT27);

    EPWM_selectDiodeEmulationPWMsignal(base, EPWM_DE_CHANNEL_B, EPWM_DE_SYNC_INV_TRIPHorL);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG32(base + CSL_EPWM_DEACTCTL)
    & CSL_EPWM_DEACTCTL_PWMB_MASK) >> CSL_EPWM_DEACTCTL_PWMB_SHIFT, EPWM_DE_SYNC_INV_TRIPHorL);

    EPWM_selectDiodeEmulationTripSignal(base, EPWM_DE_CHANNEL_A, EPWM_DE_TRIPL);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG32(base + CSL_EPWM_DEACTCTL)
    & CSL_EPWM_DEACTCTL_TRIPSELA_MASK) >> CSL_EPWM_DEACTCTL_TRIPSELA_SHIFT, EPWM_DE_TRIPL);

    EPWM_bypassDiodeEmulationLogic(base);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG32(base + CSL_EPWM_DEACTCTL)
    & CSL_EPWM_DEACTCTL_TRIPENABLE_MASK) >> CSL_EPWM_DEACTCTL_TRIPENABLE_SHIFT, 0x1);

    EPWM_forceDiodeEmulationActive(base);
    TEST_ASSERT_EQUAL_INT32(HW_RD_REG32(base + CSL_EPWM_DESTS), 0x1);

    EPWM_enableDiodeEmulationMonitorModeControl(base);
    TEST_ASSERT_EQUAL_INT32(HW_RD_REG32(base + CSL_EPWM_DEMONCTL), 0x1);

    EPWM_setDiodeEmulationMonitorModeStep(base, EPWM_DE_COUNT_UP, 0xA7);
    TEST_ASSERT_EQUAL_INT32(HW_RD_REG32(base + CSL_EPWM_DEMONSTEP), 0xA7);

    EPWM_setDiodeEmulationMonitorCounterThreshold(base, 0x3A);
    TEST_ASSERT_EQUAL_INT32(HW_RD_REG32(base + CSL_EPWM_DEMONTHRES), 0x3A);

    EPWM_disableDiodeEmulationMode(base);
    TEST_ASSERT_EQUAL_INT32(HW_RD_REG32(base + CSL_EPWM_DECTL) & CSL_EPWM_DECTL_ENABLE_MASK, 0x0);

    EPWM_nobypassDiodeEmulationLogic(base);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG32(base + CSL_EPWM_DEACTCTL)
    & CSL_EPWM_DEACTCTL_TRIPENABLE_MASK) >> CSL_EPWM_DEACTCTL_TRIPENABLE_SHIFT, 0x0);

    EPWM_disableDiodeEmulationMonitorModeControl(base);
    TEST_ASSERT_EQUAL_INT32(HW_RD_REG32(base + CSL_EPWM_DEMONCTL), 0x0);

    /*util_deinit_epwms(i);*/

    if(error==0)
    {
        if(enableLog)
        {
            DebugP_log("\r\nPass");
        }
        return 0;
    }
    else
    {
        if(enableLog)
        {
            DebugP_log("\r\nFail");
        }
        return 1;
    }
}

int32_t AM263x_EPWM_xTR_0048(uint32_t base)
{
    int i;
    volatile uint32_t error =0;

    for(i =0 ; i< 2; i++)
    {
        uint8_t mode = (uint8_t)EPWM_COUNT_MODE_DOWN_AFTER_SYNC + i;
        EPWM_setCountModeAfterSync(base, mode);
        TEST_ASSERT_EQUAL_INT32((HW_RD_REG16(base + CSL_EPWM_TBCTL)
        & CSL_EPWM_TBCTL_PHSDIR_MASK) >> CSL_EPWM_TBCTL_PHSDIR_SHIFT, mode);
    }

    for(i=0; i<2 ;i++)
    {
        uint8_t mode = (uint8_t)EPWM_PERIOD_SHADOW_LOAD + i;
        EPWM_setPeriodLoadMode(base, mode);
        TEST_ASSERT_EQUAL_INT32((HW_RD_REG16(base + CSL_EPWM_TBCTL)
        & CSL_EPWM_TBCTL_PRDLD_MASK) >> CSL_EPWM_TBCTL_PRDLD_SHIFT, mode);
    }

    for(i=0; i<3 ;i++)
    {
        uint8_t mode = (uint8_t)EPWM_SHADOW_LOAD_MODE_COUNTER_ZERO + i;
        EPWM_selectPeriodLoadEvent(base, mode);
        TEST_ASSERT_EQUAL_INT32((HW_RD_REG16(base + CSL_EPWM_TBCTL2)
        & CSL_EPWM_TBCTL2_PRDLDSYNC_MASK) >> CSL_EPWM_TBCTL2_PRDLDSYNC_SHIFT, mode);
    }


    EPWM_setTimeBaseCounter(base, 0xFFFF);
    if( !((HW_RD_REG16(base + CSL_EPWM_TBCTR) == (uint16_t)EPWM_getTimeBaseCounterValue(base))
    && (HW_RD_REG16(base + CSL_EPWM_TBCTR)) == 0xFFFF))
    {
        error++;
    }
    EPWM_setTimeBaseCounterMode(base, EPWM_COUNTER_MODE_UP);
    if(EPWM_getTimeBaseCounterDirection(base) != EPWM_TIME_BASE_STATUS_COUNT_UP)
    {
        error++;
    }

    EPWM_setTimeBaseCounterMode(base, EPWM_COUNTER_MODE_STOP_FREEZE);

   for(i=0; i<3 ;i++)
    {
        uint8_t source= (uint8_t)EPWM_AQ_TRIGGER_EVENT_TRIG_DCA_1 + i;
        EPWM_setActionQualifierT1TriggerSource(base, source);
        TEST_ASSERT_EQUAL_INT32((HW_RD_REG16(base + CSL_EPWM_AQTSRCSEL)
        & CSL_EPWM_AQTSRCSEL_T1SEL_MASK) >> CSL_EPWM_AQTSRCSEL_T1SEL_SHIFT, source);

        EPWM_setActionQualifierT2TriggerSource(base, source);
        TEST_ASSERT_EQUAL_INT32((HW_RD_REG16(base + CSL_EPWM_AQTSRCSEL)
        & CSL_EPWM_AQTSRCSEL_T2SEL_MASK) >> CSL_EPWM_AQTSRCSEL_T2SEL_SHIFT, source);
    }


   EPWM_setActionQualifierActionComplete(base, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH_ZERO);
   TEST_ASSERT_EQUAL_INT32((HW_RD_REG16(base + CSL_EPWM_AQCTLA)
   & CSL_EPWM_AQCTLA_ZRO_MASK) >> CSL_EPWM_AQCTLA_ZRO_SHIFT, EPWM_AQ_OUTPUT_HIGH_ZERO);

   EPWM_setActionQualifierActionComplete(base, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_HIGH_PERIOD);
   TEST_ASSERT_EQUAL_INT32(HW_RD_REG16(base + CSL_EPWM_AQCTLB), EPWM_AQ_OUTPUT_HIGH_PERIOD);

   EPWM_setActionQualifierContSWForceAction(base, EPWM_AQ_OUTPUT_A, EPWM_AQ_SW_OUTPUT_HIGH);
   TEST_ASSERT_EQUAL_INT32((HW_RD_REG16(base + CSL_EPWM_AQCSFRC)
   & CSL_EPWM_AQCSFRC_CSFA_MASK) >> CSL_EPWM_AQCSFRC_CSFA_SHIFT, EPWM_AQ_SW_OUTPUT_HIGH);

   EPWM_setActionQualifierContSWForceAction(base, EPWM_AQ_OUTPUT_B, EPWM_AQ_SW_OUTPUT_LOW);
   TEST_ASSERT_EQUAL_INT32((HW_RD_REG16(base + CSL_EPWM_AQCSFRC)
   & CSL_EPWM_AQCSFRC_CSFB_MASK) >> CSL_EPWM_AQCSFRC_CSFB_SHIFT, EPWM_AQ_SW_OUTPUT_LOW);

   EPWM_enableInterrupt(base);
   EPWM_forceEventTriggerInterrupt(base);
   TEST_ASSERT_EQUAL_INT32((HW_RD_REG16(base + CSL_EPWM_ETFLG)
   & CSL_EPWM_ETFLG_INT_MASK) >> CSL_EPWM_ETFLG_INT_SHIFT, EPWM_getEventTriggerInterruptStatus(base));

   EPWM_clearEventTriggerInterruptFlag(base);
   EPWM_disableInterrupt(base);

   EPWM_enableADCTrigger(base, EPWM_SOC_B);
   EPWM_forceADCTrigger(base, EPWM_SOC_B);
   TEST_ASSERT_EQUAL_INT32((HW_RD_REG16(base + CSL_EPWM_ETFLG)
   & CSL_EPWM_ETFLG_INT_MASK) >> CSL_EPWM_ETFLG_INT_SHIFT, EPWM_getADCTriggerFlagStatus(base, EPWM_SOC_B));
   EPWM_clearADCTriggerFlag(base, EPWM_SOC_B);
   EPWM_disableADCTrigger(base, EPWM_SOC_B);

   EPWM_enableDigitalCompareBlankingWindow(base);
   TEST_ASSERT_EQUAL_INT32((HW_RD_REG16(base + CSL_EPWM_DCFCTL)
   & CSL_EPWM_DCFCTL_BLANKE_MASK) >> CSL_EPWM_DCFCTL_BLANKE_SHIFT, 0x1);

   EPWM_enableDigitalCompareWindowInverseMode(base);
   TEST_ASSERT_EQUAL_INT32((HW_RD_REG16(base + CSL_EPWM_DCFCTL)
   & CSL_EPWM_DCFCTL_BLANKINV_MASK) >> CSL_EPWM_DCFCTL_BLANKINV_SHIFT, 0x1);

   EPWM_setDigitalCompareBlankingEvent(base, EPWM_DC_WINDOW_START_TBCTR_PERIOD, 0x0);
   TEST_ASSERT_EQUAL_INT32((HW_RD_REG16(base + CSL_EPWM_DCFCTL)
   & CSL_EPWM_DCFCTL_PULSESEL_MASK) >> CSL_EPWM_DCFCTL_PULSESEL_SHIFT, EPWM_DC_WINDOW_START_TBCTR_PERIOD);

   EPWM_setDigitalCompareFilterInput(base, EPWM_DC_WINDOW_SOURCE_DCAEVT2);
   TEST_ASSERT_EQUAL_INT32((HW_RD_REG16(base + CSL_EPWM_DCFCTL)
   & CSL_EPWM_DCFCTL_SRCSEL_MASK) >> CSL_EPWM_DCFCTL_SRCSEL_SHIFT, EPWM_DC_WINDOW_SOURCE_DCAEVT2);

   EPWM_enableDigitalCompareEdgeFilter(base);
   TEST_ASSERT_EQUAL_INT32((HW_RD_REG16(base + CSL_EPWM_DCFCTL)
   & CSL_EPWM_DCFCTL_EDGEFILTSEL_MASK) >> CSL_EPWM_DCFCTL_EDGEFILTSEL_SHIFT, 0x1);

   EPWM_setDigitalCompareEdgeFilterMode(base, EPWM_DC_EDGEFILT_MODE_FALLING);
   TEST_ASSERT_EQUAL_INT32((HW_RD_REG16(base + CSL_EPWM_DCFCTL)
   & CSL_EPWM_DCFCTL_EDGEMODE_MASK) >> CSL_EPWM_DCFCTL_EDGEMODE_SHIFT, EPWM_DC_EDGEFILT_MODE_FALLING);

   EPWM_setDigitalCompareEdgeFilterEdgeCount(base, EPWM_DC_EDGEFILT_EDGECNT_3);
   TEST_ASSERT_EQUAL_INT32((HW_RD_REG16(base + CSL_EPWM_DCFCTL)
   & CSL_EPWM_DCFCTL_EDGECOUNT_MASK) >> CSL_EPWM_DCFCTL_EDGECOUNT_SHIFT,
   EPWM_getDigitalCompareEdgeFilterEdgeCount(base));


   /* EPWM_setDigitalCompareWindowOffset(base, 0xFFFF);
   TEST_ASSERT_EQUAL_INT32((HW_RD_REG16(base + CSL_EPWM_DCFOFFSET)
   & CSL_EPWM_DCFOFFSET_DCFOFFSET_MASK) >> CSL_EPWM_DCFOFFSET_DCFOFFSET_SHIFT, 0xFFFF); */


   EPWM_setDigitalCompareWindowLength(base, 0xABCD);
   TEST_ASSERT_EQUAL_INT32((HW_RD_REG16(base + CSL_EPWM_DCFWINDOW)
   & CSL_EPWM_DCFWINDOW_DCFWINDOW_MASK) >> CSL_EPWM_DCFWINDOW_DCFWINDOW_SHIFT, 0xABCD);



   EPWM_setDigitalCompareEventSource(base, EPWM_DC_MODULE_B, EPWM_DC_EVENT_2, EPWM_DC_EVENT_SOURCE_ORIG_SIGNAL);
   TEST_ASSERT_EQUAL_INT32((HW_RD_REG16(base + CSL_EPWM_DCACTL)
   & CSL_EPWM_DCACTL_EVT2SRCSEL_MASK) >> CSL_EPWM_DCACTL_EVT2SRCSEL_SHIFT, EPWM_DC_EVENT_SOURCE_ORIG_SIGNAL);

   EPWM_setDigitalCompareEventSyncMode(base, EPWM_DC_MODULE_B, EPWM_DC_EVENT_2, EPWM_DC_EVENT_INPUT_SYNCED);
   TEST_ASSERT_EQUAL_INT32((HW_RD_REG16(base + CSL_EPWM_DCBCTL)
   & CSL_EPWM_DCACTL_EVT2FRCSYNCSEL_MASK) >> CSL_EPWM_DCACTL_EVT2FRCSYNCSEL_SHIFT, EPWM_DC_EVENT_INPUT_SYNCED);

   EPWM_enableDigitalCompareADCTrigger(base, EPWM_DC_MODULE_B);
   TEST_ASSERT_EQUAL_INT32((HW_RD_REG16(base + CSL_EPWM_DCBCTL)
   & CSL_EPWM_DCACTL_EVT1SOCE_MASK) >> CSL_EPWM_DCACTL_EVT1SOCE_SHIFT, 0x1);

   EPWM_enableDigitalCompareSyncEvent(base, EPWM_DC_MODULE_B);
   TEST_ASSERT_EQUAL_INT32((HW_RD_REG16(base + CSL_EPWM_DCBCTL)
   & CSL_EPWM_DCACTL_EVT1SYNCE_MASK) >> CSL_EPWM_DCACTL_EVT1SYNCE_SHIFT, 0x1);

   EPWM_enableDigitalCompareCounterCapture(base);
   TEST_ASSERT_EQUAL_INT32((HW_RD_REG16(base + CSL_EPWM_DCCAPCTL)
   & CSL_EPWM_DCCAPCTL_CAPE_MASK) >> CSL_EPWM_DCCAPCTL_CAPE_SHIFT, 0x1);

   EPWM_setDigitalCompareCounterShadowMode(base, false);
   TEST_ASSERT_EQUAL_INT32((HW_RD_REG16(base + CSL_EPWM_DCCAPCTL)
   & CSL_EPWM_DCCAPCTL_SHDWMODE_MASK) >> CSL_EPWM_DCCAPCTL_SHDWMODE_SHIFT, 0x1);

   EPWM_enableDigitalCompareTripCombinationInput(base, EPWM_DC_COMBINATIONAL_TRIPIN4, EPWM_DC_TYPE_DCBH);
   TEST_ASSERT_EQUAL_INT32((HW_RD_REG16(base + CSL_EPWM_DCBHTRIPSEL)
   & CSL_EPWM_DCBHTRIPSEL_TRIPINPUT4_MASK) >> CSL_EPWM_DCBHTRIPSEL_TRIPINPUT4_SHIFT, 0x1);
   TEST_ASSERT_EQUAL_INT32((HW_RD_REG16(base + CSL_EPWM_DCTRIPSEL)
   & CSL_EPWM_DCTRIPSEL_DCBHCOMPSEL_MASK) >> CSL_EPWM_DCTRIPSEL_DCBHCOMPSEL_SHIFT, 0xF);

   EPWM_disableDigitalCompareBlankingWindow(base);
   EPWM_disableDigitalCompareWindowInverseMode(base);
   EPWM_disableDigitalCompareEdgeFilter(base);
   EPWM_disableDigitalCompareADCTrigger(base, EPWM_DC_MODULE_B);
   EPWM_disableDigitalCompareSyncEvent(base, EPWM_DC_MODULE_B);
   EPWM_disableDigitalCompareCounterCapture(base);
   EPWM_disableDigitalCompareTripCombinationInput(base, EPWM_DC_COMBINATIONAL_TRIPIN4, EPWM_DC_TYPE_DCBH);


    /*----API checks for the DCCAP features start----*/
    /*  EPWM_enableTripZoneInterrupt(base, tzInterrupt);
        EPWM_disableTripZoneInterrupt(base, tzInterrupt);

        base is EPWM Base addresses
        tzInterrupt values are
            -> EPWM_TZ_INTERRUPT_CBC       (0x2)    - Trip Zones Cycle By Cycle interrupt
            -> EPWM_TZ_INTERRUPT_OST       (0x4)    - Trip Zones One Shot interrupt
            -> EPWM_TZ_INTERRUPT_DCAEVT1   (0x8)    - Digital Compare A Event 1 interrupt
            -> EPWM_TZ_INTERRUPT_DCAEVT2   (0x10)   - Digital Compare A Event 2 interrupt
            -> EPWM_TZ_INTERRUPT_DCBEVT1   (0x20)   - Digital Compare B Event 1 interrupt
            -> EPWM_TZ_INTERRUPT_DCBEVT2   (0x40)   - Digital Compare B Event 2 interrupt
            -> EPWM_TZ_INTERRUPT_CAPEVT    (0x80)   - Digital Capture Event interrupt
    */
    for (   uint16_t option = EPWM_TZ_INTERRUPT_CBC;
                     option <= EPWM_TZ_INTERRUPT_CAPEVT;
                     option = (option << 1))
    {
        /* write option to the API, read from the expected register. */
        volatile uint16_t (*value_enable)  = (uint16_t *) (base + CSL_EPWM_TZEINT);

        EPWM_enableTripZoneInterrupt(base, option);
        // TEST_ASSERT_BITS(mask, expected, actual)
        TEST_ASSERT_BITS(option, option, *value_enable);

        EPWM_disableTripZoneInterrupt(base, option);
        // TEST_ASSERT_BITS(mask, expected, actual)
        TEST_ASSERT_BITS(option, 0, *value_enable);
    }

    /* EPWM_forceTripZoneEvent(base, tzForceEvent)
        base is EPWM BASE ADDRESSES
        tzForceEvent takes values:
            - EPWM_TZ_FORCE_EVENT_CBC     (0x2) - Force Trip Zones Cycle By Cycle event
            - EPWM_TZ_FORCE_EVENT_OST     (0x4) - Force Trip Zones One Shot Event
            - EPWM_TZ_FORCE_EVENT_DCAEVT1 (0x8) - Force Digital Compare A Event 1
            - EPWM_TZ_FORCE_EVENT_DCAEVT2 (0x10)- Force Digital Compare A Event 2
            - EPWM_TZ_FORCE_EVENT_DCBEVT1 (0x20)- Force Digital Compare B Event 1
            - EPWM_TZ_FORCE_EVENT_DCBEVT2 (0x40)- Force Digital Compare B Event 2
            - EPWM_TZ_FORCE_EVENT_CAPEVT  (0x80)- Force Capture Event

        uint16_t EPWM_getTripZoneFlagStatus(base)
        base is epwm base address
        returns or-ed of the some following
             - EPWM_TZ_INTERRUPT    - Trip Zone interrupt was generated
                          due to the following TZ events.
            - EPWM_TZ_FLAG_CBC     (0x2) - Trip Zones Cycle By Cycle event status flag
            - EPWM_TZ_FLAG_OST     (0x4) - Trip Zones One Shot event status flag
            - EPWM_TZ_FLAG_DCAEVT1 (0x8) - Digital Compare A Event 1 status flag
            - EPWM_TZ_FLAG_DCAEVT2 (0x10)- Digital Compare A Event 2 status flag
            - EPWM_TZ_FLAG_DCBEVT1 (0x20)- Digital Compare B Event 1 status flag
            - EPWM_TZ_FLAG_DCBEVT2 (0x40)- Digital Compare B Event 2 status flag
            - EPWM_TZ_FLAG_CAPEVT  (0x80)- Digital Capture Event flag

        EPWM_clearTripZoneFlag (base, tzFlags)
        base is epwm base address
        tzFlags takes same args as EPWM_getTripZoneFlagStatus
        */

    for (   uint16_t option = EPWM_TZ_FORCE_EVENT_CBC;
                option <= EPWM_TZ_FORCE_EVENT_CAPEVT;
                option = (option << 1))
    {
        EPWM_forceTripZoneEvent(base, option);

        uint16_t status = EPWM_getTripZoneFlagStatus(base);
        volatile uint16_t (*status_reg)  = (uint16_t *) (base + CSL_EPWM_TZFLG);

        // TEST_ASSERT_EQUAL_INT16(expected, actual);
        TEST_ASSERT_EQUAL_INT16(status, *status_reg);

        // TEST_ASSERT_BITS(mask, expected, actual)
        TEST_ASSERT_BITS(option, option, status);

        EPWM_clearTripZoneFlag(base, option);
        status = EPWM_getTripZoneFlagStatus(base);
        // TEST_ASSERT_BITS(mask, expected, actual)
        TEST_ASSERT_BITS(option, 0, status);
    }


    /*  EPWM_enableCaptureInEvent(base)
        EPWM_disableCaptureInEvent(base)
            base is EPWM Base Address
            writes to CSL_EPWM_CAPCTL, at CSL_EPWM_CAPCTL_SRCSEL_MASK, with no shift
    */
    volatile uint16_t (*regValue) = (uint16_t*) (base + CSL_EPWM_CAPCTL);
    EPWM_enableCaptureInEvent(base);
    TEST_ASSERT_BITS(CSL_EPWM_CAPCTL_SRCSEL_MASK, CSL_EPWM_CAPCTL_SRCSEL_MASK, *regValue);

    EPWM_disableCaptureInEvent(base);
    TEST_ASSERT_BITS(CSL_EPWM_CAPCTL_SRCSEL_MASK, 0, *regValue);

    /*  EPWM_configCaptureGateInputPolarity(base, polSel)
            base is EPWM base Address
            polSel takes values :
                - EPWM_CAPGATE_INPUT_ALWAYS_ON  - always on
                - EPWM_CAPGATE_INPUT_ALWAYS_OFF  - always off
                - EPWM_CAPGATE_INPUT_SYNC  - CAPGATE.sync
                - EPWM_CAPGATE_INPUT_SYNC_INVERT  - CAPGATE.sync inverted
    */
    regValue = (uint16_t*) (base + CSL_EPWM_CAPCTL);
    for(uint8_t polSel = EPWM_CAPGATE_INPUT_ALWAYS_ON;
                polSel <= EPWM_CAPGATE_INPUT_SYNC_INVERT;
                polSel++)
    {
        EPWM_configCaptureGateInputPolarity(base, polSel);
        TEST_ASSERT_BITS(CSL_EPWM_CAPCTL_CAPGATEPOL_MASK, ((uint16_t)polSel) << CSL_EPWM_CAPCTL_CAPGATEPOL_SHIFT , *regValue);
    }

    /*  EPWM_invertCaptureInputPolarity(base, polSel)
            base is EPWM base Address
            polSel takes values :
                - EPWM_CAPTURE_INPUT_CAPIN_SYNC         - not inverted
                - EPWM_CAPTURE_INPUT_CAPIN_SYNC_INVERT  - inverted
    */
    regValue = (uint16_t*) (base + CSL_EPWM_CAPCTL);
    for(uint8_t polSel = EPWM_CAPTURE_INPUT_CAPIN_SYNC; polSel <= EPWM_CAPTURE_INPUT_CAPIN_SYNC_INVERT; polSel++)
    {
        EPWM_invertCaptureInputPolarity(base, polSel);
        TEST_ASSERT_BITS(CSL_EPWM_CAPCTL_CAPINPOL_MASK, ((uint16_t)polSel) << CSL_EPWM_CAPCTL_CAPINPOL_SHIFT , *regValue);
    }

    /*  EPWM_enableIndependentPulseLogic(base)
        EPWM_disableIndependentPulseLogic(base)
            base is EPWM base address
            writes to CSL_EPWM_CAPCTL, at CSL_EPWM_CAPCTL_PULSECTL_MASK
    */
    regValue = (uint16_t*) (base + CSL_EPWM_CAPCTL);
    EPWM_enableIndependentPulseLogic(base);
    TEST_ASSERT_BITS(CSL_EPWM_CAPCTL_PULSECTL_MASK, ((uint16_t)1U) << CSL_EPWM_CAPCTL_PULSECTL_SHIFT , *regValue);

    EPWM_disableIndependentPulseLogic(base);
    TEST_ASSERT_BITS(CSL_EPWM_CAPCTL_PULSECTL_MASK, 0, *regValue);

    /* EPWM_forceCaptureEventLoad(base) -> writes to a  CSL_EPWM_CAPCTL at CSL_EPWM_CAPCTL, but reads back 0. so functional check required
        //TODO : ADD funcitonal check*/

    /* EPWM_selectCaptureTripInput(base, tripSource, dcType)
        tripSource takes
            EPWM_DC_TRIP_TRIPIN1 = 0,  //!< Trip 1
            EPWM_DC_TRIP_TRIPIN2 = 1,  //!< Trip 2
            EPWM_DC_TRIP_TRIPIN3 = 2,  //!< Trip 3
            EPWM_DC_TRIP_TRIPIN4 = 3,  //!< Trip 4
            EPWM_DC_TRIP_TRIPIN5 = 4,  //!< Trip 5
            EPWM_DC_TRIP_TRIPIN6 = 5,  //!< Trip 6
            EPWM_DC_TRIP_TRIPIN7 = 6,  //!< Trip 7
            EPWM_DC_TRIP_TRIPIN8 = 7,  //!< Trip 8
            EPWM_DC_TRIP_TRIPIN9 = 8,  //!< Trip 9
            EPWM_DC_TRIP_TRIPIN10 = 9,  //!< Trip 10
            EPWM_DC_TRIP_TRIPIN11 = 10,  //!< Trip 11
            EPWM_DC_TRIP_TRIPIN12 = 11,  //!< Trip 12
            EPWM_DC_TRIP_TRIPIN13 = 12,  //!< Trip 13
            EPWM_DC_TRIP_TRIPIN14 = 13,  //!< Trip 14
            EPWM_DC_TRIP_TRIPIN15 = 14,  //!< Trip 15
            EPWM_DC_TRIP_COMBINATION = 15 //!< All Trips (Trip1 - Trip 15) are selected
        dcType takes
            - EPWM_CAPTURE_GATE (1)
            - EPWM_CAPTURE_INPUT (0)
    */

    for(uint8_t dcType = EPWM_CAPTURE_INPUT; dcType <= EPWM_CAPTURE_GATE; dcType++)
    {
        regValue = (uint16_t*) (base + CSL_EPWM_CAPTRIPSEL);
        uint16_t mask;
        uint16_t shift;
        EPWM_DigitalCompareTripInput tripSource;
        if(dcType == EPWM_CAPTURE_GATE)
        {
            mask = CSL_EPWM_CAPTRIPSEL_CAPGATECOMPSEL_MASK;
            shift =  CSL_EPWM_CAPTRIPSEL_CAPGATECOMPSEL_SHIFT;
        }
        else
        {
            mask = CSL_EPWM_CAPTRIPSEL_CAPINCOMPSEL_MASK;
            shift =  CSL_EPWM_CAPTRIPSEL_CAPINCOMPSEL_SHIFT;
        }
        for(tripSource = EPWM_DC_TRIP_TRIPIN1; tripSource <= EPWM_DC_TRIP_COMBINATION; tripSource++)
        {
            EPWM_selectCaptureTripInput(base, tripSource, dcType);
            // TEST_ASSERT_BITS(mask, expected, actual);
            TEST_ASSERT_BITS(mask, (tripSource << shift), *regValue);
        }
    }
    /* EPWM_enableCaptureTripCombinationInput(base, tripInput, dcType)
       EPWM_disableCaptureTripCombinationInput(base, tripInput, dcType)
       tripInput takes Or-ed values of EPWM_DC_COMBINATIONAL_TRIPINx for x in [0,15]
        */
    for(uint8_t dcType = EPWM_CAPTURE_INPUT; dcType <= EPWM_CAPTURE_GATE; dcType++)
    {
        uint16_t mask;
        uint16_t tripSource;
        mask = 0xFFFF;
        if(dcType == EPWM_CAPTURE_GATE)
        {
            regValue = (uint16_t*) (base + CSL_EPWM_CAPGATETRIPSEL);
        }
        else
        {
            regValue = (uint16_t*) (base + CSL_EPWM_CAPINTRIPSEL);
        }
        for(tripSource = EPWM_DC_COMBINATIONAL_TRIPIN1; tripSource < (EPWM_DC_COMBINATIONAL_TRIPIN15<<1) - 1; tripSource |= (tripSource<<1))
        {
            EPWM_enableCaptureTripCombinationInput(base, tripSource, dcType);
            // TEST_ASSERT_BITS(mask, expected, actual);
            TEST_ASSERT_BITS(mask, (uint16_t)(tripSource), *regValue);

            EPWM_disableCaptureTripCombinationInput(base, tripSource, dcType);
            TEST_ASSERT_BITS(tripSource, 0, *regValue);
        }
    }
    /*----API checks for the DCCAP features complete----*/

    /* ---- API checks for Optimized APIs ---- */
    /*
    EPWM_setCounterCompareValue_opt_cmpA(base, uint16_t compCount)
    EPWM_setCounterCompareValue_opt_cmpB(base, uint16_t compCount)
    EPWM_setCounterCompareValue_opt_cmpC(base, uint16_t compCount)
    EPWM_setCounterCompareValue_opt_cmpD(base, uint16_t compCount)
    */
    regValue = (uint16_t*) (base + CSL_EPWM_CMPA + 2U);
    for(uint16_t cmp = 0; cmp < 0xFFFF; cmp++)
    {
        EPWM_setCounterCompareValue_opt_cmpA(base, cmp);
        TEST_ASSERT_EQUAL_UINT16(*regValue, cmp);
    }
    regValue = (uint16_t*) (base + CSL_EPWM_CMPB + 2U);
    for(uint16_t cmp = 0; cmp < 0xFFFF; cmp++)
    {
        EPWM_setCounterCompareValue_opt_cmpB(base, cmp);
        TEST_ASSERT_EQUAL_UINT16(*regValue, cmp);
    }
    regValue = (uint16_t*) (base + CSL_EPWM_CMPC);
    for(uint16_t cmp = 0; cmp < 0xFFFF; cmp++)
    {
        EPWM_setCounterCompareValue_opt_cmpC(base, cmp);
        TEST_ASSERT_EQUAL_UINT16(*regValue, cmp);
    }
    regValue = (uint16_t*) (base + CSL_EPWM_CMPD);
    for(uint16_t cmp = 0; cmp < 0xFFFF; cmp++)
    {
        EPWM_setCounterCompareValue_opt_cmpD(base, cmp);
        TEST_ASSERT_EQUAL_UINT16(*regValue, cmp);
    }
    /*
    EPWM_setActionQualifierContSWForceAction_opt_outputs(uint32_t base, uint8_t outputAB)
        outputAB accepts values :
             - EPWM_AQ_A_SW_DISABLED_B_SW_DISABLED          - 0x0U
             - EPWM_AQ_A_SW_OUTPUT_LOW_B_SW_DISABLED
             - EPWM_AQ_A_SW_OUTPUT_HIGH_B_SW_DISABLED
             - EPWM_AQ_A_SW_DISABLED_B_SW_OUTPUT_LOW
             - EPWM_AQ_A_SW_OUTPUT_LOW_B_SW_OUTPUT_LOW
             - EPWM_AQ_A_SW_OUTPUT_HIGH_B_SW_OUTPUT_LOW
             - EPWM_AQ_A_SW_DISABLED_B_SW_OUTPUT_HIGH
             - EPWM_AQ_A_SW_OUTPUT_LOW_B_SW_OUTPUT_HIGH
             - EPWM_AQ_A_SW_OUTPUT_HIGH_B_SW_OUTPUT_HIGH    - 0xAU
    */
    volatile uint16_t (*regValue16) = (uint16_t*)(base + CSL_EPWM_AQCSFRC);
    for(uint8_t outputAB = EPWM_AQ_A_SW_DISABLED_B_SW_DISABLED;
                outputAB <= EPWM_AQ_A_SW_OUTPUT_HIGH_B_SW_OUTPUT_HIGH;
                outputAB++)
    {
        EPWM_setActionQualifierContSWForceAction_opt_outputs(base, outputAB);
        TEST_ASSERT_EQUAL_UINT16(*regValue16, outputAB);
    }

    /* EPWM_enableADCTriggerEventCountInit */
    EPWM_enableADCTriggerEventCountInit(base, EPWM_SOC_A);
    TEST_ASSERT_EQUAL_UINT16(HW_RD_REG16(base + CSL_EPWM_ETCNTINITCTL)&CSL_EPWM_ETCNTINITCTL_SOCAINITEN_MASK,
        CSL_EPWM_ETCNTINITCTL_SOCAINITEN_MASK);

    EPWM_enableADCTriggerEventCountInit(base, EPWM_SOC_B);
    TEST_ASSERT_EQUAL_UINT16(HW_RD_REG16(base + CSL_EPWM_ETCNTINITCTL)&CSL_EPWM_ETCNTINITCTL_SOCBINITEN_MASK,
        CSL_EPWM_ETCNTINITCTL_SOCBINITEN_MASK);

    /* EPWM_disableADCTriggerEventCountInit */
    EPWM_disableADCTriggerEventCountInit(base, EPWM_SOC_A);
    TEST_ASSERT_EQUAL_UINT16(HW_RD_REG16(base + CSL_EPWM_ETCNTINITCTL)&CSL_EPWM_ETCNTINITCTL_SOCAINITEN_MASK, 0u);

    EPWM_disableADCTriggerEventCountInit(base, EPWM_SOC_B);
    TEST_ASSERT_EQUAL_UINT16(HW_RD_REG16(base + CSL_EPWM_ETCNTINITCTL)&CSL_EPWM_ETCNTINITCTL_SOCBINITEN_MASK, 0u);

    for (uint8_t delayMode = EPWM_VALLEY_DELAY_MODE_SW_DELAY; delayMode <=EPWM_VALLEY_DELAY_MODE_VCNT_DELAY_SHIFT_4_SW_DELAY; delayMode++)
    {
        EPWM_setValleyDelayDivider(base, delayMode);
        TEST_ASSERT_EQUAL_UINT16((HW_RD_REG16(base + CSL_EPWM_VCAPCTL)&CSL_EPWM_VCAPCTL_VDELAYDIV_MASK)>>CSL_EPWM_VCAPCTL_VDELAYDIV_SHIFT, delayMode);
    }


    for(uint8_t output = EPWM_AQ_OUTPUT_A;;output = EPWM_AQ_OUTPUT_B)
    {
        for(uint32_t action = EPWM_AQ_OUTPUT_NO_CHANGE_UP_T1; action <= EPWM_AQ_OUTPUT_TOGGLE_DOWN_T2; action = (action==EPWM_AQ_OUTPUT_NO_CHANGE_UP_T1)? EPWM_AQ_OUTPUT_LOW_UP_T1:(action << 1))
        {
            EPWM_setAdditionalActionQualifierActionComplete(base, output, action);
            uint32_t registerTOffset = (uint32_t)CSL_EPWM_AQCTLA2 + (uint16_t)output;

            TEST_ASSERT_EQUAL_UINT16(HW_RD_REG16(base + registerTOffset), action);
        }
        if(output == EPWM_AQ_OUTPUT_B)
        {
            break;
        }
    }

    /* EPWM_getMinDeadBandDelay get API. needs functional */

    for(uint8_t trigger = EPWM_VALLEY_TRIGGER_EVENT_SOFTWARE; trigger<= EPWM_VALLEY_TRIGGER_EVENT_DCBEVT2; trigger++)
    {
        EPWM_setValleyTriggerSource(base, trigger);
        TEST_ASSERT_EQUAL_UINT16((HW_RD_REG16(base + CSL_EPWM_VCAPCTL) & (uint16_t)CSL_EPWM_VCAPCTL_TRIGSEL_MASK), (uint16_t)trigger << CSL_EPWM_VCAPCTL_TRIGSEL_SHIFT);
    }

    for(uint8_t action = EPWM_AQ_SW_SH_LOAD_ON_CNTR_ZERO; action <= EPWM_AQ_SW_IMMEDIATE_LOAD; action++)
    {
        EPWM_setActionQualifierContSWForceShadowMode(base, action);
        TEST_ASSERT_EQUAL_UINT16((HW_RD_REG16(base + CSL_EPWM_AQSFRC) & (uint16_t)CSL_EPWM_AQSFRC_RLDCSF_MASK), (uint16_t)((uint16_t)action << CSL_EPWM_AQSFRC_RLDCSF_SHIFT));
    }

    EPWM_enableValleyHWDelay(base);
    TEST_ASSERT_EQUAL_UINT16((HW_RD_REG16(base + CSL_EPWM_VCAPCTL) & (uint16_t)CSL_EPWM_VCAPCTL_EDGEFILTDLYSEL_MASK), CSL_EPWM_VCAPCTL_EDGEFILTDLYSEL_MASK);

    EPWM_disableValleyHWDelay(base);
    TEST_ASSERT_EQUAL_UINT16((HW_RD_REG16(base + CSL_EPWM_VCAPCTL) & (uint16_t)CSL_EPWM_VCAPCTL_EDGEFILTDLYSEL_MASK), 0);

    /* EPWM_setValleyTriggerEdgeCounts only testing the edges */
    EPWM_setValleyTriggerEdgeCounts(base, 15, 15);
    TEST_ASSERT_EQUAL_UINT16((HW_RD_REG16(base + CSL_EPWM_VCNTCFG) & (CSL_EPWM_VCNTCFG_STARTEDGE_MASK | CSL_EPWM_VCNTCFG_STOPEDGE_MASK)), 15|(15<<CSL_EPWM_VCNTCFG_STOPEDGE_SHIFT));

    EPWM_setValleyTriggerEdgeCounts(base, 0, 0);
    TEST_ASSERT_EQUAL_UINT16((HW_RD_REG16(base + CSL_EPWM_VCNTCFG) & (CSL_EPWM_VCNTCFG_STARTEDGE_MASK | CSL_EPWM_VCNTCFG_STOPEDGE_MASK)), 0);

    /* EPWM_getGlobalLoadEventCount get API. needs a functional test*/
    /* EPWM_getCounterCompareShadowStatus get API. needs a functional test*/
    /* EPWM_getADCTriggerEventCount get API. needs a functional test*/
    /* EPWM_getValleyCount get API. needs a functional test*/
    /* EPWM_getTimeBaseCounterDirection get API. needs a functional test*/
    /* EPWM_clearOneShotTripZoneFlag write only API. needs a functional test*/
    /* EPWM_forceInterruptEventCountInit write only API. needs a functional test*/
    /* EPWM_forceSyncPulse write only API. needs a functional test*/
    /* EPWM_lockRegisters write only API. needs a functional test*/
    /* EPWM_clearDiodeEmulationActive write only API. needs a functional test*/
    /* EPWM_getDigitalCompareBlankingWindowLengthCount get only API. needs a functional test*/
    /* EPWM_getInterruptEventCount get only API. needs a functional test*/
    /* EPWM_getDigitalCompareCaptureStatus get only API. needs a functional test*/
    /* EPWM_getDigitalCompareEdgeFilterEdgeStatus get only API. needs a functional test*/
    /* EPWM_getDigitalCompareCaptureCount get only API. needs a functional test*/
    /* EPWM_forceGlobalLoadOneShotEvent write only API. needs a functional test*/
    /* EPWM_getOneShotTripZoneFlagStatus write only API. needs a functional test*/
    /* EPWM_getValleyHWDelay write only API. needs a functional test*/
    /* EPWM_startOneShotSync write only API. needs a functional test*/
    /* EPWM_getCycleByCycleTripZoneFlagStatus get only API. needs a functional test*/
    /* EPWM_forceActionQualifierSWAction write only API. needs a functional test*/
    /* EPWM_clearCycleByCycleTripZoneFlag write only API. needs a functional test*/
    /* EPWM_getSyncStatus get only API. needs a functional test*/
    /* EPWM_getTimeBaseCounterValue get only API. needs a functional test*/
    /* EPWM_getValleyEdgeStatus get only API. needs a functional test*/
    /* EPWM_getTimeBaseCounterValue get only API. needs a functional test*/
    /* EPWM_clearSyncEvent write only API. needs a functional test*/
    /* EPWM_getDigitalCompareBlankingWindowOffsetCount get only API. needs a functional test*/
    /* EPWM_forceADCTriggerEventCountInit write only API. needs a functional test*/

    EPWM_disableRisingEdgeDelayCountShadowLoadMode(base);
    TEST_ASSERT_EQUAL_UINT16(HW_RD_REG16(base + CSL_EPWM_DBCTL) & CSL_EPWM_DBCTL_SHDWDBREDMODE_MASK, 0);

    EPWM_disableFallingEdgeDelayCountShadowLoadMode(base);
    TEST_ASSERT_EQUAL_UINT16(HW_RD_REG16(base + CSL_EPWM_DBCTL) & CSL_EPWM_DBCTL_SHDWDBFEDMODE_MASK, 0);

    EPWM_enableGlobalLoadOneShotMode(base);
    TEST_ASSERT_EQUAL_UINT16(HW_RD_REG16(base + CSL_EPWM_GLDCTL) & CSL_EPWM_GLDCTL_OSHTMODE_MASK, CSL_EPWM_GLDCTL_OSHTMODE_MASK);

    EPWM_disableGlobalLoadOneShotMode(base);
    TEST_ASSERT_EQUAL_UINT16(HW_RD_REG16(base + CSL_EPWM_GLDCTL) & CSL_EPWM_GLDCTL_OSHTMODE_MASK, 0);

    /*
    Read Only EPWM_startValleyCapture(base);
    TEST_ASSERT_EQUAL_UINT16(HW_RD_REG16(base + CSL_EPWM_VCAPCTL) & CSL_EPWM_VCAPCTL_VCAPSTART_MASK, CSL_EPWM_VCAPCTL_VCAPSTART_MASK);
    HW_WR_REG16(base + CSL_EPWM_VCAPCTL, (HW_RD_REG16(base + CSL_EPWM_VCAPCTL) & ~CSL_EPWM_VCAPCTL_VCAPSTART_MASK)); */

    EPWM_enableValleyCapture(base);
    TEST_ASSERT_EQUAL_UINT16(HW_RD_REG16(base + CSL_EPWM_VCAPCTL) & CSL_EPWM_VCAPCTL_VCAPE_MASK, CSL_EPWM_VCAPCTL_VCAPE_MASK);

    EPWM_disableValleyCapture(base);
    TEST_ASSERT_EQUAL_UINT16(HW_RD_REG16(base + CSL_EPWM_VCAPCTL) & CSL_EPWM_VCAPCTL_VCAPE_MASK, 0);

    uint16_t shadowModeOffset;
    shadowModeOffset = CSL_EPWM_AQCTL_SHDWAQAMODE_SHIFT + (uint16_t)EPWM_ACTION_QUALIFIER_A;
    EPWM_disableActionQualifierShadowLoadMode(base, EPWM_ACTION_QUALIFIER_A);
    TEST_ASSERT_EQUAL_UINT16(HW_RD_REG16(base + CSL_EPWM_AQCTL) & (CSL_EPWM_AQCTL_SHDWAQAMODE_MAX << shadowModeOffset), 0);
    shadowModeOffset = CSL_EPWM_AQCTL_SHDWAQAMODE_SHIFT + (uint16_t)EPWM_ACTION_QUALIFIER_B;
    EPWM_disableActionQualifierShadowLoadMode(base, EPWM_ACTION_QUALIFIER_B);
    TEST_ASSERT_EQUAL_UINT16(HW_RD_REG16(base + CSL_EPWM_AQCTL) & (CSL_EPWM_AQCTL_SHDWAQAMODE_MAX << shadowModeOffset), 0);

    EPWM_CounterCompareModule array[4] = {
        EPWM_COUNTER_COMPARE_A,
        EPWM_COUNTER_COMPARE_B,
        EPWM_COUNTER_COMPARE_C,
        EPWM_COUNTER_COMPARE_D,
    };

    for(uint8_t index = 0; index <4; index++)
    {
        EPWM_CounterCompareModule compModule = array[index];

        uint16_t shadowModeOffset;
        uint32_t registerOffset;

        if((compModule == EPWM_COUNTER_COMPARE_A) ||
        (compModule == EPWM_COUNTER_COMPARE_C))
        {
            shadowModeOffset = CSL_EPWM_CMPCTL_SHDWAMODE_SHIFT;
        }
        else
        {
            shadowModeOffset = CSL_EPWM_CMPCTL_SHDWBMODE_SHIFT;
        }

        //
        // Get the register offset.  CSL_EPWM_CMPCTL for A&B or
        // CSL_EPWM_CMPCTL2 for C&D
        //
        if((compModule == EPWM_COUNTER_COMPARE_A) ||
        (compModule == EPWM_COUNTER_COMPARE_B))
        {
            registerOffset = base + CSL_EPWM_CMPCTL;
        }
        else
        {
            registerOffset = base + CSL_EPWM_CMPCTL2;
        }

        EPWM_disableCounterCompareShadowLoadMode(base, compModule);
        TEST_ASSERT_EQUAL_UINT16((HW_RD_REG16(registerOffset) & (CSL_EPWM_CMPCTL_SHDWAMODE_MAX)) >> shadowModeOffset, 0);

        for(uint8_t loadMode = EPWM_COMP_LOAD_ON_CNTR_ZERO; loadMode <= EPWM_COMP_LOAD_ON_SYNC_CNTR_ZERO_PERIOD; loadMode++)
        {
            uint16_t syncModeOffset;
            uint16_t loadModeOffset;
            uint16_t shadowModeOffset;
            uint32_t registerOffset;

            if((compModule == EPWM_COUNTER_COMPARE_A) ||
                (compModule == EPWM_COUNTER_COMPARE_C))
            {
                syncModeOffset = CSL_EPWM_CMPCTL_LOADASYNC_SHIFT;
                loadModeOffset = CSL_EPWM_CMPCTL_LOADAMODE_SHIFT;
                shadowModeOffset = CSL_EPWM_CMPCTL_SHDWAMODE_SHIFT;
            }
            else
            {
                syncModeOffset = CSL_EPWM_CMPCTL_LOADBSYNC_SHIFT;
                loadModeOffset = CSL_EPWM_CMPCTL_LOADBMODE_SHIFT;
                shadowModeOffset = CSL_EPWM_CMPCTL_SHDWBMODE_SHIFT;
            }

            if((compModule == EPWM_COUNTER_COMPARE_A) ||
                (compModule == EPWM_COUNTER_COMPARE_B))
            {
                registerOffset = base + CSL_EPWM_CMPCTL;
            }
            else
            {
                registerOffset = base + CSL_EPWM_CMPCTL2;
            }

            EPWM_setCounterCompareShadowLoadMode(base, compModule, loadMode);
            uint16_t value = HW_RD_REG16(registerOffset);
            uint16_t mask  = (CSL_EPWM_CMPCTL_LOADASYNC_MAX << syncModeOffset) | (CSL_EPWM_CMPCTL_LOADAMODE_MAX << loadModeOffset) | (CSL_EPWM_CMPCTL_SHDWAMODE_MAX << shadowModeOffset);
            uint16_t field = (((uint16_t)loadMode >> 2U) << syncModeOffset) | (((uint16_t)loadMode & CSL_EPWM_CMPCTL_LOADASYNC_MAX) << loadModeOffset);

            TEST_ASSERT_EQUAL_UINT16(value & mask, field);
        }
    }

    for(uint16_t loadTrigger= EPWM_GL_LOAD_PULSE_CNTR_ZERO; loadTrigger <= EPWM_GL_LOAD_PULSE_CNTR_CMPD_D; loadTrigger++)
    {
        EPWM_setGlobalLoadTrigger(base, loadTrigger);
        TEST_ASSERT_EQUAL_UINT16((HW_RD_REG16(base + CSL_EPWM_GLDCTL) & (uint16_t)CSL_EPWM_GLDCTL_GLDMODE_MASK), (uint16_t)loadTrigger << CSL_EPWM_GLDCTL_GLDMODE_SHIFT);

        EPWM_setGlobalLoadTrigger(base, EPWM_GL_LOAD_PULSE_GLOBAL_FORCE);
        TEST_ASSERT_EQUAL_UINT16((HW_RD_REG16(base + CSL_EPWM_GLDCTL) & (uint16_t)CSL_EPWM_GLDCTL_GLDMODE_MASK), (uint16_t)EPWM_GL_LOAD_PULSE_GLOBAL_FORCE << CSL_EPWM_GLDCTL_GLDMODE_SHIFT);
    }

    EPWM_disableGlobalLoad(base);
    TEST_ASSERT_EQUAL_UINT16((HW_RD_REG16(base + CSL_EPWM_GLDCTL) & (uint16_t)CSL_EPWM_GLDCTL_GLD_MASK), 0);

    EPWM_disableInterruptEventCountInit(base);
    TEST_ASSERT_EQUAL_UINT16((HW_RD_REG16(base + CSL_EPWM_ETCNTINITCTL) & (uint16_t)CSL_EPWM_ETCNTINITCTL_INTINITEN_MASK), 0);

    EPWM_enableTripZone2Signals(base, EPWM_TZ_SIGNAL_CAPEVT_OST);
    TEST_ASSERT_EQUAL_UINT16((HW_RD_REG16(base + CSL_EPWM_TZSEL2) & (uint16_t)EPWM_TZ_SIGNAL_CAPEVT_OST), EPWM_TZ_SIGNAL_CAPEVT_OST);
    EPWM_enableTripZone2Signals(base, EPWM_TZ_SIGNAL_CAPEVT_CBC);
    TEST_ASSERT_EQUAL_UINT16((HW_RD_REG16(base + CSL_EPWM_TZSEL2) & (uint16_t)EPWM_TZ_SIGNAL_CAPEVT_CBC), EPWM_TZ_SIGNAL_CAPEVT_CBC);

    EPWM_disableTripZone2Signals(base, EPWM_TZ_SIGNAL_CAPEVT_OST);
    TEST_ASSERT_EQUAL_UINT16((HW_RD_REG16(base + CSL_EPWM_TZSEL2) & (uint16_t)EPWM_TZ_SIGNAL_CAPEVT_OST), 0);
    EPWM_disableTripZone2Signals(base, EPWM_TZ_SIGNAL_CAPEVT_CBC);
    TEST_ASSERT_EQUAL_UINT16((HW_RD_REG16(base + CSL_EPWM_TZSEL2) & (uint16_t)EPWM_TZ_SIGNAL_CAPEVT_CBC), 0);

    for(uint8_t aqModule = EPWM_ACTION_QUALIFIER_A;;aqModule = EPWM_ACTION_QUALIFIER_B)
    {
        for(uint16_t loadMode = EPWM_AQ_LOAD_ON_CNTR_ZERO; loadMode <= EPWM_AQ_LOAD_ON_SYNC_CNTR_ZERO_PERIOD; loadMode++)
        {
            uint16_t syncModeOffset;
            uint16_t shadowModeOffset;

            syncModeOffset = CSL_EPWM_AQCTL_LDAQASYNC_SHIFT + (uint16_t)aqModule;
            shadowModeOffset = CSL_EPWM_AQCTL_SHDWAQAMODE_SHIFT + (uint16_t)aqModule;

            EPWM_setActionQualifierShadowLoadMode(base, aqModule, loadMode);

            uint16_t mask = (((CSL_EPWM_AQCTL_LDAQAMODE_MASK << (uint16_t)aqModule) | (CSL_EPWM_AQCTL_LDAQASYNC_MAX << (uint16_t)syncModeOffset))) | (CSL_EPWM_AQCTL_SHDWAQAMODE_MAX << shadowModeOffset);
            uint16_t value = ((((uint16_t)loadMode >> 2U) << syncModeOffset) | (((uint16_t)loadMode & CSL_EPWM_AQCTL_LDAQAMODE_MASK) <<(uint16_t)aqModule));

            TEST_ASSERT_EQUAL_UINT16(HW_RD_REG16(base + CSL_EPWM_AQCTL) &mask, value);
        }
        if(aqModule == EPWM_ACTION_QUALIFIER_A)
        {
            break;
        }
    }

    EPWM_enableInterruptEventCountInit(base);
    TEST_ASSERT_EQUAL_UINT16(HW_RD_REG16(base + CSL_EPWM_ETCNTINITCTL) &CSL_EPWM_ETCNTINITCTL_INTINITEN_MASK, CSL_EPWM_ETCNTINITCTL_INTINITEN_MASK);
    EPWM_disableInterruptEventCountInit(base);
    TEST_ASSERT_EQUAL_UINT16(HW_RD_REG16(base + CSL_EPWM_ETCNTINITCTL) &CSL_EPWM_ETCNTINITCTL_INTINITEN_MASK, 0);

    EPWM_enableGlobalLoadOneShotMode(base);
    TEST_ASSERT_EQUAL_UINT16(HW_RD_REG16(base + CSL_EPWM_GLDCTL) &CSL_EPWM_GLDCTL_OSHTMODE_MASK, CSL_EPWM_GLDCTL_OSHTMODE_MASK);
    EPWM_disableGlobalLoadOneShotMode(base);
    TEST_ASSERT_EQUAL_UINT16(HW_RD_REG16(base + CSL_EPWM_GLDCTL) &CSL_EPWM_GLDCTL_OSHTMODE_MASK, 0);

    /* testing extreme values */
    EPWM_setInterruptEventCountInitValue(base,0);
    TEST_ASSERT_EQUAL_UINT16(HW_RD_REG16(base + CSL_EPWM_ETCNTINIT) &CSL_EPWM_ETCNTINIT_INTINIT_MASK, 0);
    EPWM_setInterruptEventCountInitValue(base,15);
    TEST_ASSERT_EQUAL_UINT16(HW_RD_REG16(base + CSL_EPWM_ETCNTINIT) &CSL_EPWM_ETCNTINIT_INTINIT_MASK, 15);

    EPWM_enableGlobalLoad(base);
    TEST_ASSERT_EQUAL_UINT16(HW_RD_REG16(base + CSL_EPWM_GLDCTL) &CSL_EPWM_GLDCTL_GLD_MASK, CSL_EPWM_GLDCTL_GLD_MASK);
    EPWM_disableGlobalLoad(base);
    TEST_ASSERT_EQUAL_UINT16(HW_RD_REG16(base + CSL_EPWM_GLDCTL) &CSL_EPWM_GLDCTL_GLD_MASK, 0);

    /* testing extreme values */
    EPWM_setValleySWDelayValue(base,0);
    TEST_ASSERT_EQUAL_UINT16(HW_RD_REG16(base + CSL_EPWM_SWVDELVAL) & CSL_EPWM_SWVDELVAL_SWVDELVAL_MASK, 0);
    EPWM_setValleySWDelayValue(base,0xffff);
    TEST_ASSERT_EQUAL_UINT16(HW_RD_REG16(base + CSL_EPWM_SWVDELVAL) & CSL_EPWM_SWVDELVAL_SWVDELVAL_MASK, 0xffff);

    return error;
}

int32_t test_epwm_cases(uint8_t in)
{
    int32_t failcount=0;

    uint32_t epwm_offset=0;
    uint32_t base;

    for(epwm_offset=0;epwm_offset<32; epwm_offset++)
        {
            base = CSL_CONTROLSS_G0_EPWM0_U_BASE + epwm_offset * 0x1000;

            if(isTestSupported[epwm_offset][in])
            {
                switch(in)
                {
                    case 1:
                        failcount += AM263x_EPWM_xTR_0001(base, epwm_offset);
                        break;
                    case 2:
                        failcount += AM263x_EPWM_xTR_0002(base);
                        break;
                    case 3:
                        failcount += AM263x_EPWM_xTR_0003(base, epwm_offset);
                        break;
                    /* case 4:
                        failcount += AM263x_EPWM_xTR_0004(base);
                        break; */
                    case 5:
                        failcount += AM263x_EPWM_xTR_0005(base);
                        break;
                    case 6:
                        failcount += AM263x_EPWM_xTR_0006(base, epwm_offset);
                        break;
                    case 7:
                        failcount += AM263x_EPWM_xTR_0007(base);
                        break;
                    case 8:
                        failcount += AM263x_EPWM_xTR_0008(base, epwm_offset);
                        break;
                    case 9:
                        failcount += AM263x_EPWM_xTR_0009(base, epwm_offset);
                        break;
                    /*case 10:
                        failcount += AM263x_EPWM_xTR_0010(base, epwm_offset);
                        break;
                    */
                    /*case 11:
                        failcount += AM263x_EPWM_xTR_0011(base);
                        break;*/
                    /*case 12:
                        failcount += AM263x_EPWM_xTR_0012(base, epwm_offset);
                        break;
                    */
                    /*case 13:
                        failcount += AM263x_EPWM_xTR_0013(base);
                        break;*/
                    case 14:
                        failcount += AM263x_EPWM_xTR_0014(base, epwm_offset);
                        break;
                    case 15:
                        failcount += AM263x_EPWM_xTR_0015(base);
                        break;
                    case 16:
                        failcount += AM263x_EPWM_xTR_0016(base, epwm_offset);
                        break;
                    case 17:
                        failcount += AM263x_EPWM_xTR_0017(base, epwm_offset);
                        break;
                /*  case 18:
                        failcount += AM263x_EPWM_xTR_0018(base);
                        break; */
                    case 19:
                        failcount += AM263x_EPWM_xTR_0019(base);
                        break;
                    case 20:
                        failcount += AM263x_EPWM_xTR_0020(base);
                        break;
                /*  case 21:
                        failcount += AM263x_EPWM_xTR_0021(base);
                        break;
                    case 22:
                        failcount += AM263x_EPWM_xTR_0022(base);
                        break;
                    case 23:
                        failcount += AM263x_EPWM_xTR_0023(base);
                        break;
                */
                    case 24:
                        failcount += AM263x_EPWM_xTR_0024(base, epwm_offset);
                        break;
                /*    case 25:
                        failcount += AM263x_EPWM_xTR_0025(base);
                        break;
                    case 26:
                        failcount += AM263x_EPWM_xTR_0026(base);
                        break;
                    case 27:
                        failcount += AM263x_EPWM_xTR_0027(base);
                        break;
                    case 28:
                        failcount += AM263x_EPWM_xTR_0028(base);
                        break;
                    case 29:
                        failcount += AM263x_EPWM_xTR_0029(base);
                        break;
                    case 30:
                        failcount += AM263x_EPWM_xTR_0030(base);
                        break;
                    case 31:
                        failcount += AM263x_EPWM_xTR_0031(base);
                        break;
                    case 32:
                        failcount += AM263x_EPWM_xTR_0032(base, epwm_offset);
                        break;
                        */
                    case 33:
                        failcount += AM263x_EPWM_xTR_0033(base);
                        break;
                    case 34:
                        failcount += AM263x_EPWM_xTR_0034(base, epwm_offset);
                        break;
                    case 35:
                        failcount += AM263x_EPWM_xTR_0035(base);
                        break;
                    case 36:
                        failcount += AM263x_EPWM_xTR_0036(base, epwm_offset);
                        break;
                    case 37:
                        failcount += AM263x_EPWM_xTR_0037(base, epwm_offset);
                        break;
                  case 38:
                        failcount += AM263x_EPWM_xTR_0038(base);
                        break;
                    case 39:
                        failcount += AM263x_EPWM_xTR_0039(base, epwm_offset);
                        break;
                    case 40:
                        failcount += AM263x_EPWM_xTR_0040(base, epwm_offset);
                        break;
                    /* case 41:
                        failcount += AM263x_EPWM_xTR_0041(base);
                        break; */
                    case 42:
                        failcount += AM263x_EPWM_xTR_0042(base, epwm_offset);
                        break;
                    case 43:
                        failcount += AM263x_EPWM_xTR_0043(base, epwm_offset);
                        break;
                    case 44:
                        failcount += AM263x_EPWM_xTR_0044(base, epwm_offset);
                        break;
                    case 45:
                        failcount += AM263x_EPWM_xTR_0045(base);
                        break;
                    case 46:
                        failcount += AM263x_EPWM_xTR_0046(base, epwm_offset);
                        break;
                    case 47:
                        failcount += AM263x_EPWM_xTR_0047(base, epwm_offset);
                        break;
                    case 48:
                        failcount += AM263x_EPWM_xTR_0048(base);
                        break;
                }
                util_deinit_epwms(epwm_offset);
            if(enableLog)
            {
                DebugP_log("\r\nEPWM offset %d failcount=%d\r\r\n", epwm_offset, failcount);
            }

                //if(in == 2 || in==3 || in==5 || in==7 || in==45)
                //break;
            }
        }



    if(failcount!=0)
    {
        if(enableLog)
        {
            DebugP_log("\r\nFAIL %d", failcount);
        }
        return 1;
    }
    else
    {
        if(enableLog)
        {
            DebugP_log("\r\nPass");
        }
        return 0;
    }

}

void tester_init(void)
{
    UART_Transaction_init(&trans);
    trans.timeout=TIMEOUT_UART_TESTER;

    if(enableLog)
    {
        DebugP_log("\r\nSending initialization command!!");
    }
    gNumBytesWritten = 0U;
    trans.buf   = &gCmdTxBuffer[0U];
    strncpy(trans.buf,"123456780000000000000000ABCDEFEF", CMD_SIZE);
    trans.count = CMD_SIZE;           //if not pulled up CMD_SIZE-1 expected. Send CMD_SIZE-1 only to sync. Unexpected 1.
    transferOK = UART_write(gUartHandle[TESTER_UART], &trans);
    /* Quickly wait to read ack. No debug prints here*/
    gNumBytesRead = 0U;
    trans.buf   = &gCmdRxBuffer[0U];
    trans.count = RSP_SIZE;
    transferOK = UART_read(gUartHandle[TESTER_UART], &trans);

    if(enableLog)
    {
        DebugP_log("\r\nReceived response. Tester Initialized!!");
    }
    /*Clear TX buffer for shorter commands*/
    uint8_t ind;
    for(ind=0;ind<CMD_SIZE;ind++)
    {
        gCmdTxBuffer[ind]=0;
    }
}

void wait_for_user_uart_input(void)
{

    bool correct_input = false;

    while(!correct_input)
    {
        DebugP_log("please enter 'y' after setting up required configuration for test\r\r\n");

        /*Clearing the Rx buffer for 1 char*/
        gCmdRxBuffer[0] = '\0';

        /* forever wait until the user inputs*/
        trans.timeout   = TIMEOUT_UART_INFINITE;
        trans.buf       = &gCmdRxBuffer[0U];
        trans.count     = 1;
        transferOK      = UART_read(gUartHandle[CONFIG_UART0], &trans);

        correct_input = (gCmdRxBuffer[0] == 'y')||(gCmdRxBuffer[0] == 'Y');
    }

    /* changing timeout back to default*/
    trans.timeout   = TIMEOUT_UART_TESTER;

    return;
}

void tester_command(char *str)
{
    if(manual_testing)
    {
        DebugP_log("%s\r\r\n", str);
        wait_for_user_uart_input();
        return;
    }

    if(enableLog)
    {
        DebugP_log("\r\nSending command!!");
    }
    /* Send command*/
    gNumBytesWritten = 0U;
    trans.buf   = &gCmdTxBuffer[0U];
    strncpy(trans.buf,str, strlen(str));
    trans.count = CMD_SIZE;
    transferOK = UART_write(gUartHandle[TESTER_UART], &trans);
    /* Quickly wait to read ack. No debug prints here*/
    gNumBytesRead = 0U;
    trans.buf   = &gCmdRxBuffer[0U];
    trans.count = RSP_SIZE;
    transferOK = UART_read(gUartHandle[TESTER_UART], &trans);

    if(startsWith(str, "get timestamp"))
        {
            ECAP_UART_read();
        }



    uint8_t ind;
    if(enableLog)
    {
        DebugP_log("\r\nReceived response!! (%d %d) : ", transferOK, trans.status);
        for(ind=0;ind<RSP_SIZE;ind++)
        {
            DebugP_log("%c", gCmdRxBuffer[ind]);
        }
        DebugP_log("\r\n");
    }

    /*Clear TX buffer for shorter commands*/
    for(ind=0;ind<RSP_SIZE;ind++)
    {
        gCmdTxBuffer[ind]=0;
    }
}


void EnablePinMux(void)
{
    HW_WR_REG32(CSL_IOMUX_U_BASE + CSL_IOMUX_IO_CFG_KICK0,0x83E70B13);
    HW_WR_REG32(CSL_IOMUX_U_BASE + CSL_IOMUX_IO_CFG_KICK1,0x95A4F1E0);

    for(uint8_t i=0; i<16; i++)
    {
        uint32_t baseA = CSL_IOMUX_EPWM0_A_CFG_REG + 8*i;
        uint32_t baseB = CSL_IOMUX_EPWM0_B_CFG_REG + 8*i;

        HW_WR_REG32(CSL_IOMUX_U_BASE + baseA, 0x500) ;
        HW_WR_REG32(CSL_IOMUX_U_BASE + baseB, 0x500) ;
    }

    for(uint8_t i=16; i<32; i++)
    {
        uint32_t baseA = CSL_IOMUX_EPWM0_A_CFG_REG + 8*i;
        uint32_t baseB = CSL_IOMUX_EPWM0_B_CFG_REG + 8*i;

        HW_WR_REG32(CSL_IOMUX_U_BASE + baseA, 0x505) ;
        HW_WR_REG32(CSL_IOMUX_U_BASE + baseB, 0x505) ;
    }
}

void EnableEPWMClk(void)
{
    HW_WR_REG32(CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_LOCK0_KICK0 ,0x01234567);
    HW_WR_REG32(CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_LOCK0_KICK1 ,0xFEDCBA8);

    HW_WR_REG32(CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_EPWM_CLKSYNC ,0xFFFFFFFF);
}

void ECAP_UART_read()
{
    int32_t j = 0, k = 0;
    char str[10];
    int comma_counter = 0;

    for(int ind=0;ind<RSP_SIZE;ind++)
    {
        str[k++] = gCmdRxBuffer[ind];
        if(gCmdRxBuffer[ind]==',')
        {
            comma_counter++;
            str[k] = '\0';
            uint32_t val = atoi(str);
            ts[j++] = val;
            k=0;
        }

        if(comma_counter==4) break;
    }
return;
}

int startsWith(char *str, char *substr)
{
    size_t sub_len = strlen(substr);
    return (strncmp(str, substr, sub_len) == 0);
}

void Configure_Xbar_Sync_DUT(uint32_t epwm_offset)
{
    SOC_xbarSelectOutputXBarInputSource(CSL_CONTROLSS_OUTPUTXBAR_U_BASE, 14, 0, 0, 0, 0, 0, 0, 0, 0, 0, OUTPUT_XBAR_EPWM_SYNCOUT_XBAR0, 0);
    SOC_xbarSelectLatchOutputXBarOutputSignal(CSL_CONTROLSS_OUTPUTXBAR_U_BASE, 0);
    SOC_xbarSelectStretchedPulseOutputXBarOutputSignal(CSL_CONTROLSS_OUTPUTXBAR_U_BASE, 0);
    SOC_xbarSelectPWMSyncOutXBarInput(CSL_CONTROLSS_PWMSYNCOUTXBAR_U_BASE, 0, (1<<epwm_offset));
    HW_WR_REG32(CSL_IOMUX_U_BASE + CSL_IOMUX_IO_CFG_KICK0,0x83E70B13);
    HW_WR_REG32(CSL_IOMUX_U_BASE + CSL_IOMUX_IO_CFG_KICK1,0x95A4F1E0);
    HW_WR_REG32(CSL_IOMUX_U_BASE + CSL_IOMUX_PR0_PRU1_GPO18_CFG_REG, 0x5);
}