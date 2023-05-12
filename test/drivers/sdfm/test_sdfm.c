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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <unity.h>
#include <drivers/sdfm.h>
#include <drivers/epwm.h>
#include <drivers/soc.h>
#include <drivers/ecap.h>
#include <drivers/edma.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

#include "menu.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* Testcases */
static void SDFM_setCompFilterHighThresholdApiCheck(void *args);

static void SDFM_configure_sdfm_input_control_unit_for_an_sdfm_channel(void *args);
static void SDFM_configure_manchester_decoding_mode(void *args);
static void SDFM_configure_data_filter_for_each_channel_with_pwm_sync_disabled(void *args);
static void SDFM_configure_data_filter_with_fifo_enabled_and_pwm_sync_disabled(void *args);
static void SDFM_configure_data_filter_with_fifo_enabled_and_pwm_sync_enabled(void *args);
static void SDFM_configure_comparator_filter(void *args);
static void SDFM_configure_event_filters_for_high_low_threshold_events(void *args);
static void SDFM_lock_access_to_event_filters_configuration_registers_for_an_sdfm_channel(void *args);
static void SDFM_configure_data_ack_event_as_trigger_for_data_ready_interrupt(void *args);
static void SDFM_configure_data_ack_event_as_trigger_for_sdint_interrupt(void *args);
static void SDFM_pwm_sync_sources(void *args);
static void SDFM_sdfm1_clock_source_from_sdfm0(void *args);
static void SDFM_sdfm_connectivity_to_dma_xbar(void *args);
static void SDFM_sdfm_connectivity_to_int_xbar(void *args);
static void SDFM_sdfm_connectivity_to_ecap_xbar(void *args);
static void SDFM_sdfm_connectivity_to_pwm_xbar(void *args);
static void SDFM_sdfm_connectivity_to_output_xbar(void *args);
static void SDFM_r5_to_sdfm_access_latency(void *args);

int32_t test_sdfm_cases(uint8_t in);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

#define TOTAL_TEST_CASES (19)

void test_main(void *args)
{
    /* Open drivers */
    Drivers_open();
    Board_driversOpen();

    tester_init();
    UNITY_BEGIN();
    char test_title[] = "----------------------SDFM-TEST-CASES----------------------";
    


    menu_input test_list[TOTAL_TEST_CASES] =
    {
        {0, 3210,  SDFM_configure_sdfm_input_control_unit_for_an_sdfm_channel ,                    "SDFM_configure_sdfm_input_control_unit_for_an_sdfm_channel" },                   
        {0, 3211,  SDFM_configure_manchester_decoding_mode ,                                       "SDFM_configure_manchester_decoding_mode" },                                      
        {0, 3212,  SDFM_configure_data_filter_for_each_channel_with_pwm_sync_disabled ,            "SDFM_configure_data_filter_for_each_channel_with_pwm_sync_disabled" },           
        {0, 3213,  SDFM_configure_data_filter_with_fifo_enabled_and_pwm_sync_disabled ,            "SDFM_configure_data_filter_with_fifo_enabled_and_pwm_sync_disabled" },           
        {0, 3214,  SDFM_configure_data_filter_with_fifo_enabled_and_pwm_sync_enabled ,             "SDFM_configure_data_filter_with_fifo_enabled_and_pwm_sync_enabled" },            
        {0, 3215,  SDFM_configure_comparator_filter ,                                              "SDFM_configure_comparator_filter" },                                             
        {0, 3216,  SDFM_configure_event_filters_for_high_low_threshold_events ,                    "SDFM_configure_event_filters_for_high_low_threshold_events" },                   
        {0, 3217,  SDFM_lock_access_to_event_filters_configuration_registers_for_an_sdfm_channel , "SDFM_lock_access_to_event_filters_configuration_registers_for_an_sdfm_channel" },
        {0, 3218,  SDFM_configure_data_ack_event_as_trigger_for_data_ready_interrupt ,             "SDFM_configure_data_ack_event_as_trigger_for_data_ready_interrupt" },            
        {0, 3219,  SDFM_configure_data_ack_event_as_trigger_for_sdint_interrupt ,                  "SDFM_configure_data_ack_event_as_trigger_for_sdint_interrupt" },                 
        {0, 3220,  SDFM_pwm_sync_sources ,                                                         "SDFM_pwm_sync_sources" },                                                        
        {0, 3221,  SDFM_sdfm1_clock_source_from_sdfm0 ,                                            "SDFM_sdfm1_clock_source_from_sdfm0" },                                           
        {0, 3222,  SDFM_sdfm_connectivity_to_dma_xbar ,                                            "SDFM_sdfm_connectivity_to_dma_xbar" },                                           
        {0, 3223,  SDFM_sdfm_connectivity_to_int_xbar ,                                            "SDFM_sdfm_connectivity_to_int_xbar" },                                           
        {0, 3224,  SDFM_sdfm_connectivity_to_ecap_xbar ,                                           "SDFM_sdfm_connectivity_to_ecap_xbar" },                                          
        {0, 3225,  SDFM_sdfm_connectivity_to_pwm_xbar ,                                            "SDFM_sdfm_connectivity_to_pwm_xbar" },                                           
        {0, 3226,  SDFM_sdfm_connectivity_to_output_xbar ,                                         "SDFM_sdfm_connectivity_to_output_xbar" },                                        
        {0, 3227,  SDFM_r5_to_sdfm_access_latency ,                                                "SDFM_r5_to_sdfm_access_latency" },                                               
        {1, 1   ,  SDFM_setCompFilterHighThresholdApiCheck,                                        "SDFM_setCompFilterHighThresholdApiCheck" },                                       
    };


    
    menu(TOTAL_TEST_CASES, test_list, test_title);

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
}

void tearDown(void)
{
}

/* Testcase 1 - Check the SDFM_setCompFilterHighThreshold API */
static void SDFM_setCompFilterHighThresholdApiCheck(void *args)
{
    uint32_t filterNum = 0;
    uint16_t val = 0xFF;

    /* Call the SDFM_setCompFilterHighThreshold API */
    SDFM_setCompFilterHighThreshold(CSL_CONTROLSS_SDFM0_U_BASE, filterNum, val);

    /* Check if the value was written correctly */
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG16(CSL_CONTROLSS_SDFM0_U_BASE + CSL_SDFM_SDFLT1CMPH1 +
              (filterNum*SDFM_SDFIL_OFFSET)) & CSL_SDFM_SDFLT1CMPH1_HLT_MASK) >> CSL_SDFM_SDFLT1CMPH1_HLT_SHIFT, val);
}

static void SDFM_configure_sdfm_input_control_unit_for_an_sdfm_channel(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_sdfm_cases(1), 0);
}
static void SDFM_configure_manchester_decoding_mode(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_sdfm_cases(2), 0);
}
static void SDFM_configure_data_filter_for_each_channel_with_pwm_sync_disabled(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_sdfm_cases(3), 0);
}
static void SDFM_configure_data_filter_with_fifo_enabled_and_pwm_sync_disabled(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_sdfm_cases(4), 0);
}
static void SDFM_configure_data_filter_with_fifo_enabled_and_pwm_sync_enabled(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_sdfm_cases(5), 0);
}
static void SDFM_configure_comparator_filter(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_sdfm_cases(6), 0);
}
static void SDFM_configure_event_filters_for_high_low_threshold_events(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_sdfm_cases(7), 0);
}
static void SDFM_lock_access_to_event_filters_configuration_registers_for_an_sdfm_channel(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_sdfm_cases(8), 0);
}
static void SDFM_configure_data_ack_event_as_trigger_for_data_ready_interrupt(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_sdfm_cases(9), 0);
}
static void SDFM_configure_data_ack_event_as_trigger_for_sdint_interrupt(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_sdfm_cases(10), 0);
}
static void SDFM_pwm_sync_sources(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_sdfm_cases(11), 0);
}
static void SDFM_sdfm1_clock_source_from_sdfm0(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_sdfm_cases(12), 0);
}
static void SDFM_sdfm_connectivity_to_dma_xbar(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_sdfm_cases(13), 0);
}
static void SDFM_sdfm_connectivity_to_int_xbar(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_sdfm_cases(14), 0);
}
static void SDFM_sdfm_connectivity_to_ecap_xbar(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_sdfm_cases(15), 0);
}
static void SDFM_sdfm_connectivity_to_pwm_xbar(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_sdfm_cases(16), 0);
}
static void SDFM_sdfm_connectivity_to_output_xbar(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_sdfm_cases(17), 0);
}
static void SDFM_r5_to_sdfm_access_latency(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_sdfm_cases(18), 0);
}



#define TEST_MODE_CPUISRREAD 0
#define TEST_MODE_CPUPOLL 1
#define TEST_MODE_DMAREAD 2
#define TEST_MODE_PWMXBAR_OUT 3
#define TEST_MODE_OUTPUTXBAR_OUT 4
#define TEST_MODE_ECAPMUX_OUT 5

#define PWM_SYNC_ENABLE 1
#define PWM_SYNC_DISABLE 0

#define FIFO_ENABLE 1
#define FIFO_DISABLE 0

#define COMPARATOR_ENABLE 1
#define COMPARATOR_DISABLE 0
#define COMPARATOR_AND_FILTER_ENABLE 2

#define COMMONCLOCK_ENABLE 1
#define COMMONCLOCK_DISABLE 0

#define SDFM0TO1CLOCK_ENABLE 1
#define SDFM0TO1CLOCK_DISABLE 0


struct  PWM_MODULES {
    uint16_t phase1_axis1_data;
    uint16_t phase1_axis2_data;
    uint16_t phase1_axis3_data;
    uint16_t phase2_axis1_data;
    uint16_t phase2_axis2_data;
    uint16_t phase2_axis3_data;
    uint16_t phase3_axis1_data;
    uint16_t phase3_axis2_data;
    uint16_t phase3_axis3_data;
} SDFMData;

struct  SDFM_data {
    uint16_t axis1_phase1_data ;
    uint16_t axis1_phase2_data ;
    uint16_t axis1_phase3_data ;
    uint16_t axis1_dc_bus_data ;
    uint16_t axis2_phase1_data ;
    uint16_t axis2_phase2_data ;
    uint16_t axis2_phase3_data ;
    uint16_t axis2_dc_bus_data ;
} SDFMData_2;

uint32_t g_current_base;
volatile uint32_t g_isr_count;
volatile uint32_t g_curr_dma_evt=0;

#define MAX_SAMPLES               100

int16_t  filter1Result[MAX_SAMPLES]={};
int16_t  filter2Result[MAX_SAMPLES]={};
int16_t  filter3Result[MAX_SAMPLES]={};
int16_t  filter4Result[MAX_SAMPLES]={};

int16_t  compFilter1Result[MAX_SAMPLES]={};
int16_t  compFilter2Result[MAX_SAMPLES]={};
int16_t  compFilter3Result[MAX_SAMPLES]={};
int16_t  compFilter4Result[MAX_SAMPLES]={};

uint32_t util_getsdfminstancefrombase(uint32_t base)
{
    switch(base)
    {
    case CSL_CONTROLSS_SDFM0_U_BASE:
            return 0;
    case CSL_CONTROLSS_SDFM1_U_BASE:
            return 1;
    default:
            return 0xFFFFFFFF;
    }

}

//QUERY: why is 0x00 write required after 0x7 write for resetting SDFM?

void util_deinit_sdfm(uint32_t base)
{
    //DebugP_log("\n util_deinit_sdfm");

    HW_WR_REG32(CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_LOCK0_KICK0 ,0x01234567);
    HW_WR_REG32(CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_LOCK0_KICK1 ,0xFEDCBA8);

    if(base==CSL_CONTROLSS_SDFM0_U_BASE)
    {
        HW_WR_REG32(CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_SDFM0_RST ,0x07);
        HW_WR_REG32(CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_SDFM0_RST ,0x00);
    }
    if(base==CSL_CONTROLSS_SDFM1_U_BASE)
    {
        HW_WR_REG32(CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_SDFM1_RST ,0x07);
        HW_WR_REG32(CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_SDFM1_RST ,0x00);
    }
}

void util_deinit_ecaps()
{
    //DebugP_log("\n util_deinit_ecaps");

    HW_WR_REG32(CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_LOCK0_KICK0 ,0x01234567);
    HW_WR_REG32(CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_LOCK0_KICK1 ,0xFEDCBA8);

    uint8_t i;

    for(i=0;i<10;i++)
    {
        HW_WR_REG32(CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_ECAP0_RST + (i*4),0x07);
        HW_WR_REG32(CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_ECAP0_RST + (i*4),0x00);
    }
}

void util_deinit_epwms()
{
    //DebugP_log("\n util_deinit_epwms");

    HW_WR_REG32(CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_LOCK0_KICK0 ,0x01234567);
    HW_WR_REG32(CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_LOCK0_KICK1 ,0xFEDCBA8);

    uint8_t i;

    for(i=0;i<32;i++)
    {
        HW_WR_REG32(CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_ETPWM0_RST + (i*4),0x07);
        HW_WR_REG32(CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_ETPWM0_RST + (i*4),0x00);
    }
}

void util_inittimer()
{
    HW_WR_REG32(CSL_ICSSM0_INTERNAL_U_BASE + 0x2E000, 0x551);
    HW_WR_REG32(CSL_ICSSM0_INTERNAL_U_BASE + 0x2E000 + 0x10, 00);
}

static inline uint32_t util_gettimertime()
{
    return HW_RD_REG32(CSL_ICSSM0_INTERNAL_U_BASE + 0x2E000 + 0x10);
}


void util_configPWM(uint32_t base, uint16_t period, uint16_t cmpa, uint16_t cmpb, uint32_t counterMode)
{
    //DebugP_log("\n EPWM %d configured", (base-CSL_CONTROLSS_G0_EPWM0_U_BASE)/0x1000);
    //config 3

    // Configure Time Base counter Clock - Write to CLKDIV and HSPCLKDIV bit
    EPWM_setClockPrescaler(base, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);
    ////DebugP_log("\nmain: CTL Configured");


    // Configure Time Base Counter Mode - Write to CTRMODE bit
    EPWM_setTimeBaseCounterMode( base, counterMode); //EPWM_COUNTER_MODE_UP_DOWN

    // Configure TBPRD value - Write to TBPRD bit
    EPWM_setTimeBasePeriod( base, period);

    //
    // Default Configurations.
    //
    EPWM_disablePhaseShiftLoad(base);

    EPWM_setPhaseShift(base, 0U);

    // Write to TBCTR register
    EPWM_setTimeBaseCounter(base, 0);
    ////DebugP_log("\nmain: CTR Configured");

    //
    // Setup shadow register load on ZERO
    //
    EPWM_setCounterCompareShadowLoadMode(base,
                                         EPWM_COUNTER_COMPARE_A,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);
    EPWM_setCounterCompareShadowLoadMode(base,
                                         EPWM_COUNTER_COMPARE_B,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);

    //
    // Set Compare values
    //
    EPWM_setCounterCompareValue(base, EPWM_COUNTER_COMPARE_A,
                                cmpa);

    EPWM_setCounterCompareValue(base, EPWM_COUNTER_COMPARE_B,
                                cmpb);

    if(counterMode==EPWM_COUNTER_MODE_UP_DOWN)
    {

//    // Set PWM output to toggle on CTR = PRD
//    // Write to PRD bit of AQCTLA register
//    EPWM_setActionQualifierAction(base,
//                                  EPWM_AQ_OUTPUT_A,
//                                  EPWM_AQ_OUTPUT_LOW,
//                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);

        // Write to PRD bit of AQCTLA register
        EPWM_setActionQualifierAction(base,
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_HIGH,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);


        // Write to PRD bit of AQCTLA register
        EPWM_setActionQualifierAction(base,
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);


        // Write to PRD bit of AQCTLA register
        EPWM_setActionQualifierAction(base,
                                      EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_HIGH,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);

        // Write to PRD bit of AQCTLA register
        EPWM_setActionQualifierAction(base,
                                      EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);

    }

    if(counterMode==EPWM_COUNTER_MODE_UP)
    {

        EPWM_setActionQualifierAction(base,
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_HIGH,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
        EPWM_setActionQualifierAction(base,
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);


        EPWM_setActionQualifierAction(base,
                                      EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_HIGH,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);

        EPWM_setActionQualifierAction(base,
                                      EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);

    }


    ////DebugP_log("\nmain: AQ Configured");
    //End of Config 3

}

void util_connect_PWM_SDFM_loopback_pin(uint32_t pin)
{
    //DebugP_log("\nEPWM SDFM loopback pin %d", pin);


//    uint32_t reg;
//    reg = HW_RD_REG32(CSL_HSM_SOC_CTRL_U_BASE +CSL_HSM_SOC_CTRL_ISC_CTRL_REG_R5SS0_CORE0);
//    reg = reg | 0x00070000;
//    HW_WR_REG32(CSL_HSM_SOC_CTRL_U_BASE +CSL_HSM_SOC_CTRL_ISC_CTRL_REG_R5SS0_CORE0,reg);
//
//    reg = HW_RD_REG32(CSL_HSM_SOC_CTRL_U_BASE +CSL_HSM_SOC_CTRL_ISC_CTRL_REG_R5SS0_CORE1);
//    reg = reg | 0x00070000;
//    HW_WR_REG32(CSL_HSM_SOC_CTRL_U_BASE +CSL_HSM_SOC_CTRL_ISC_CTRL_REG_R5SS0_CORE1,reg);
//
//    reg = HW_RD_REG32(CSL_HSM_SOC_CTRL_U_BASE +CSL_HSM_SOC_CTRL_ISC_CTRL_REG_R5SS1_CORE0);
//    reg = reg | 0x00070000;
//    HW_WR_REG32(CSL_HSM_SOC_CTRL_U_BASE +CSL_HSM_SOC_CTRL_ISC_CTRL_REG_R5SS1_CORE0,reg);
//
//    reg = HW_RD_REG32(CSL_HSM_SOC_CTRL_U_BASE +CSL_HSM_SOC_CTRL_ISC_CTRL_REG_R5SS1_CORE1);
//    reg = reg | 0x00070000;
//    HW_WR_REG32(CSL_HSM_SOC_CTRL_U_BASE +CSL_HSM_SOC_CTRL_ISC_CTRL_REG_R5SS1_CORE1,reg);
//
    HW_WR_REG32(CSL_IOMUX_U_BASE + CSL_IOMUX_IO_CFG_KICK0,0x83E70B13);
    HW_WR_REG32(CSL_IOMUX_U_BASE + CSL_IOMUX_IO_CFG_KICK1,0x95A4F1E0);

    if(pin==0)
    {
        HW_WR_REG32(CSL_IOMUX_U_BASE + CSL_IOMUX_EPWM0_A_CFG_REG, 0x500) ;
        HW_WR_REG32(CSL_IOMUX_U_BASE + CSL_IOMUX_EPWM0_B_CFG_REG, 0x500) ;

        HW_WR_REG32(CSL_IOMUX_U_BASE + CSL_IOMUX_SDFM0_CLK0_CFG_REG, 0x08) ;
        HW_WR_REG32(CSL_IOMUX_U_BASE + CSL_IOMUX_SDFM0_D0_CFG_REG, 0x08) ;
    }
    if(pin==1)
    {
        HW_WR_REG32(CSL_IOMUX_U_BASE + CSL_IOMUX_EPWM1_A_CFG_REG, 0x500) ;
        HW_WR_REG32(CSL_IOMUX_U_BASE + CSL_IOMUX_EPWM1_B_CFG_REG, 0x500) ;

        HW_WR_REG32(CSL_IOMUX_U_BASE + CSL_IOMUX_SDFM0_CLK1_CFG_REG, 0x08) ;
        HW_WR_REG32(CSL_IOMUX_U_BASE + CSL_IOMUX_SDFM0_D1_CFG_REG, 0x08) ;
    }
    if(pin==2)
    {
        HW_WR_REG32(CSL_IOMUX_U_BASE + CSL_IOMUX_EPWM2_A_CFG_REG, 0x500) ;
        HW_WR_REG32(CSL_IOMUX_U_BASE + CSL_IOMUX_EPWM2_B_CFG_REG, 0x500) ;

        HW_WR_REG32(CSL_IOMUX_U_BASE + CSL_IOMUX_SDFM0_CLK2_CFG_REG, 0x08) ;
        HW_WR_REG32(CSL_IOMUX_U_BASE + CSL_IOMUX_SDFM0_D2_CFG_REG, 0x08) ;
    }
    if(pin==3)
    {
        HW_WR_REG32(CSL_IOMUX_U_BASE + CSL_IOMUX_EPWM3_A_CFG_REG, 0x500) ;
        HW_WR_REG32(CSL_IOMUX_U_BASE + CSL_IOMUX_EPWM3_B_CFG_REG, 0x500) ;

        HW_WR_REG32(CSL_IOMUX_U_BASE + CSL_IOMUX_SDFM0_CLK3_CFG_REG, 0x08) ;
        HW_WR_REG32(CSL_IOMUX_U_BASE + CSL_IOMUX_SDFM0_D3_CFG_REG, 0x08) ;
    }
    if(pin==4)
    {
        HW_WR_REG32(CSL_IOMUX_U_BASE + CSL_IOMUX_EPWM4_A_CFG_REG, 0x00) ;
        HW_WR_REG32(CSL_IOMUX_U_BASE + CSL_IOMUX_EPWM4_B_CFG_REG, 0x00) ;

        HW_WR_REG32(CSL_IOMUX_U_BASE + CSL_IOMUX_MMC0_CLK_CFG_REG, 0x08);
        HW_WR_REG32(CSL_IOMUX_U_BASE + CSL_IOMUX_MMC0_CMD_CFG_REG, 0x08);
    }
    if(pin==5)
    {
        HW_WR_REG32(CSL_IOMUX_U_BASE + CSL_IOMUX_EPWM5_A_CFG_REG, 0x00) ;
        HW_WR_REG32(CSL_IOMUX_U_BASE + CSL_IOMUX_EPWM5_B_CFG_REG, 0x00) ;

        HW_WR_REG32(CSL_IOMUX_U_BASE + CSL_IOMUX_MMC0_D0_CFG_REG, 0x08);
        HW_WR_REG32(CSL_IOMUX_U_BASE + CSL_IOMUX_MMC0_D1_CFG_REG, 0x08);
    }
    if(pin==6)
    {
        HW_WR_REG32(CSL_IOMUX_U_BASE + CSL_IOMUX_EPWM6_A_CFG_REG, 0x00) ;
        HW_WR_REG32(CSL_IOMUX_U_BASE + CSL_IOMUX_EPWM6_B_CFG_REG, 0x00) ;

        HW_WR_REG32(CSL_IOMUX_U_BASE + CSL_IOMUX_MMC0_D2_CFG_REG, 0x08);
        HW_WR_REG32(CSL_IOMUX_U_BASE + CSL_IOMUX_MMC0_D3_CFG_REG, 0x08);
    }
    if(pin==7)
    {
        HW_WR_REG32(CSL_IOMUX_U_BASE + CSL_IOMUX_EPWM7_A_CFG_REG, 0x00) ;
        HW_WR_REG32(CSL_IOMUX_U_BASE + CSL_IOMUX_EPWM7_B_CFG_REG, 0x00) ;

        HW_WR_REG32(CSL_IOMUX_U_BASE + CSL_IOMUX_MMC0_WP_CFG_REG, 0x08);
        HW_WR_REG32(CSL_IOMUX_U_BASE + CSL_IOMUX_MMC0_CD_CFG_REG, 0x08);
    }

}

//#ifdef __cplusplus
//#pragma CODE_STATE (32)
//#else
//#pragma CODE_STATE (sdfmISR,32)
//#endif  /* #ifdef __cplusplus */

uint32_t sdfmReadFlagRegister[10] = {};
uint32_t isr_timestamp[10]={};
uint16_t loopCounter1 = 0;
uint32_t gFlagenableComparatorFilters;
uint32_t gFlagenableFifo;
uint32_t g_FIFO_INT_NUM=3;
uint32_t g_xbarout=0;
uint32_t g_ecapbase;
uint8_t g_dmaxbar_num=0;
uint8_t g_intxbar_num=0;
uint8_t g_pwm_num_sync=0;


static void sdfmISR(void *handle)
{

    //Writes to SDFM_SDDPARM1/2/3/4, bit 10 SDFM_SDDPARM1_DR_MASK
    //SDFM_FILTER_1/2/3/4 = 0/1/2/3
    //SDFM_DATA_FORMAT_16_BIT = 0
    SDFM_setOutputDataFormat(g_current_base, SDFM_FILTER_1, SDFM_DATA_FORMAT_16_BIT);
    SDFM_setOutputDataFormat(g_current_base, SDFM_FILTER_2, SDFM_DATA_FORMAT_16_BIT);
    SDFM_setOutputDataFormat(g_current_base, SDFM_FILTER_3, SDFM_DATA_FORMAT_16_BIT);
    SDFM_setOutputDataFormat(g_current_base, SDFM_FILTER_4, SDFM_DATA_FORMAT_16_BIT);

    if(loopCounter1<10)
    {
        sdfmReadFlagRegister[loopCounter1] = HW_RD_REG32(g_current_base + CSL_SDFM_SDIFLG);
    }

    if(gFlagenableComparatorFilters==COMPARATOR_DISABLE)
    {
        if(gFlagenableFifo!=1)
        {
            if(loopCounter1 < MAX_SAMPLES)
            {
                filter1Result[loopCounter1] = (int16_t)(SDFM_getFilterData(g_current_base, SDFM_FILTER_1) >> 16U);
                filter2Result[loopCounter1] = (int16_t)(SDFM_getFilterData(g_current_base, SDFM_FILTER_2) >> 16U);
                filter3Result[loopCounter1] = (int16_t)(SDFM_getFilterData(g_current_base, SDFM_FILTER_3) >> 16U);
                filter4Result[loopCounter1] = (int16_t)(SDFM_getFilterData(g_current_base, SDFM_FILTER_4) >> 16U);

                SDFM_clearInterruptFlag(g_current_base, SDFM_MAIN_INTERRUPT_FLAG | 0xFFFF);
            }
        }
        if(gFlagenableFifo==1)
        {

            if(loopCounter1 < MAX_SAMPLES)
            {
                uint16_t i;
                for(i = 0; i < g_FIFO_INT_NUM; i++)
                {
                    filter1Result[loopCounter1] = (int16_t)(SDFM_getFIFOData(g_current_base, SDFM_FILTER_1) >> 16U);
                    filter2Result[loopCounter1] = (int16_t)(SDFM_getFIFOData(g_current_base, SDFM_FILTER_2) >> 16U);
                    filter3Result[loopCounter1] = (int16_t)(SDFM_getFIFOData(g_current_base, SDFM_FILTER_3) >> 16U);
                    filter4Result[loopCounter1] = (int16_t)(SDFM_getFIFOData(g_current_base, SDFM_FILTER_4) >> 16U);
                }

                SDFM_clearInterruptFlag(g_current_base, SDFM_MAIN_INTERRUPT_FLAG | 0xFFFFFF);
            }

        }
    }
    if(gFlagenableComparatorFilters!=COMPARATOR_DISABLE)
    {
        if(loopCounter1 < MAX_SAMPLES)
        {
            compFilter1Result[loopCounter1] = (int16_t)(SDFM_getComparatorSincData(g_current_base, SDFM_FILTER_1));
            compFilter2Result[loopCounter1] = (int16_t)(SDFM_getComparatorSincData(g_current_base, SDFM_FILTER_2));
            compFilter3Result[loopCounter1] = (int16_t)(SDFM_getComparatorSincData(g_current_base, SDFM_FILTER_3));
            compFilter4Result[loopCounter1] = (int16_t)(SDFM_getComparatorSincData(g_current_base, SDFM_FILTER_4));
        }

        SDFM_clearInterruptFlag(g_current_base, SDFM_MAIN_INTERRUPT_FLAG | 0xFFFFFF);
    }

    //Notify test
    g_isr_count++;

    if(loopCounter1<10)
    {
        isr_timestamp[loopCounter1]=util_gettimertime();
    }

    loopCounter1++;

    // TBD: Acknowledge this __interrupt to receive more __interrupts from group 5
    //CSL_vimClrIntrPending( (CSL_vimRegs *)(uintptr_t)CSL_MCU_DOMAIN_VIM_BASE_ADDR0, CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_0 + g_intxbar_num );
    //CSL_vimAckIntr((CSL_vimRegs *)(uintptr_t)CSL_MCU_DOMAIN_VIM_BASE_ADDR0,0);


}

uint16_t get_sinc3_shift_from_dosr(uint32_t dosr)
{
    if(dosr<=31)
    {
        return 0;
    }
    else if(dosr<=40)
    {
        return 1;
    }
    else if(dosr<=50)
    {
        return 2;
    }
    else if(dosr<=63)
    {
        return 3;
    }
    else if(dosr<=80)
    {
        return 4;
    }
    else if(dosr<=101)
    {
        return 5;
    }
    else if(dosr<=127)
    {
        return 6;
    }
    else if(dosr<=161)
    {
        return 7;
    }
    else if(dosr<=181)
    {
        return 8;
    }
    else if(dosr<=203)
    {
        return 8;
    }
    else if(dosr<=255)
    {
        return 9;
    }
    else if(dosr==256)
    {
        return 10;
    }

    return 7; //Assume 128-160 as default
}


//
// util_initECAP - Configure eCAP
//
void util_initECAP(uint32_t base, uint32_t input)
{
    //
    // Disable ,clear all capture flags and interrupts
    //
    ECAP_disableInterrupt(base,
                          (ECAP_ISR_SOURCE_CAPTURE_EVENT_1  |
                          ECAP_ISR_SOURCE_CAPTURE_EVENT_2  |
                          ECAP_ISR_SOURCE_CAPTURE_EVENT_3  |
                          ECAP_ISR_SOURCE_CAPTURE_EVENT_4  |
                          ECAP_ISR_SOURCE_COUNTER_OVERFLOW |
                          ECAP_ISR_SOURCE_COUNTER_PERIOD   |
                          ECAP_ISR_SOURCE_COUNTER_COMPARE));
    ECAP_clearInterrupt(base,
                        (ECAP_ISR_SOURCE_CAPTURE_EVENT_1  |
                        ECAP_ISR_SOURCE_CAPTURE_EVENT_2  |
                        ECAP_ISR_SOURCE_CAPTURE_EVENT_3  |
                        ECAP_ISR_SOURCE_CAPTURE_EVENT_4  |
                        ECAP_ISR_SOURCE_COUNTER_OVERFLOW |
                        ECAP_ISR_SOURCE_COUNTER_PERIOD   |
                        ECAP_ISR_SOURCE_COUNTER_COMPARE));

    //
    // Disable CAP1-CAP4 register loads
    //
    ECAP_disableTimeStampCapture(base);

    //
    // Configure eCAP
    //    Enable capture mode.
    //    One shot mode, stop capture at event 4.
    //    Set polarity of the events to rising, falling, rising, falling edge.
    //    Set capture in time difference mode.
    //    Select input from XBAR7.
    //    Enable eCAP module.
    //    Enable interrupt.
    //
    ECAP_stopCounter(base);
    ECAP_enableCaptureMode(base);

    ECAP_setCaptureMode(base, ECAP_ONE_SHOT_CAPTURE_MODE, ECAP_EVENT_4);

    ECAP_setEventPolarity(base, ECAP_EVENT_1, ECAP_EVNT_FALLING_EDGE);
    ECAP_setEventPolarity(base, ECAP_EVENT_2, ECAP_EVNT_RISING_EDGE);
    ECAP_setEventPolarity(base, ECAP_EVENT_3, ECAP_EVNT_FALLING_EDGE);
    ECAP_setEventPolarity(base, ECAP_EVENT_4, ECAP_EVNT_RISING_EDGE);

    ECAP_enableCounterResetOnEvent(base, ECAP_EVENT_1);
    ECAP_enableCounterResetOnEvent(base, ECAP_EVENT_2);
    ECAP_enableCounterResetOnEvent(base, ECAP_EVENT_3);
    ECAP_enableCounterResetOnEvent(base, ECAP_EVENT_4);

    //XBAR_setInputPin(INPUTXBAR_BASE, XBAR_INPUT7, 16);
    ECAP_selectECAPInput(base, input);

    ECAP_enableLoadCounter(base);
    ECAP_setSyncOutMode(base, ECAP_SYNC_OUT_SYNCI);
    ECAP_startCounter(base);
    ECAP_enableTimeStampCapture(base);
    ECAP_reArm(base);

    ECAP_enableInterrupt(base, ECAP_ISR_SOURCE_CAPTURE_EVENT_4);
}



//PWM to SDFM loopback
//Use: force epwm_sdfm_connect_enable 1

//
//For SDFM-EPWM connection, below is the procedure , let me know if you see any issues
//Once your design is downloaded , do the following on QEL prompt –
//
//force epwm_sdfm_connect_enable 1
//
//This will enable below connections :
//
//assign SDFM0_CLK0_pad = epwm_sdfm_connect_enable ? EPWM0_A_pad : 'bz ;
//assign SDFM0_D0_pad = epwm_sdfm_connect_enable ? EPWM0_B_pad : 'bz ;
//assign SDFM1_CLK0_pad = epwm_sdfm_connect_enable ? EPWM1_A_pad : 'bz ;
//assign SDFM1_D0_pad = epwm_sdfm_connect_enable ? EPWM1_B_pad : 'bz ;
//assign SDFM2_CLK0_pad = epwm_sdfm_connect_enable ? EPWM2_A_pad : 'bz ;
//assign SDFM2_D0_pad = epwm_sdfm_connect_enable ? EPWM2_B_pad : 'bz ;
//assign SDFM3_CLK0_pad = epwm_sdfm_connect_enable ? EPWM3_A_pad : 'bz ;
//assign SDFM3_D0_pad = epwm_sdfm_connect_enable ? EPWM3_B_pad : 'bz ;
//assign MMC0_CLK_pad = epwm_sdfm_connect_enable ? EPWM4_A_pad : 'bz ;
//assign MMC0_CMD_pad = epwm_sdfm_connect_enable ? EPWM4_B_pad : 'bz ;
//assign MMC0_D0_pad = epwm_sdfm_connect_enable ? EPWM5_A_pad : 'bz ;
//assign MMC0_D1_pad = epwm_sdfm_connect_enable ? EPWM5_B_pad : 'bz ;
//assign MMC0_D2_pad = epwm_sdfm_connect_enable ? EPWM6_A_pad : 'bz ;
//assign MMC0_D3_pad = epwm_sdfm_connect_enable ? EPWM6_B_pad : 'bz ;
//assign MMC0_WP_pad = epwm_sdfm_connect_enable ? EPWM7_A_pad : 'bz ;
//assign MMC0_CD_pad = epwm_sdfm_connect_enable ? EPWM7_B_pad : 'bz ;

static HwiP_Object       gSdfmHwiObject;

int32_t util_sdfm_test(uint32_t base, uint32_t testMode, uint32_t enablePwmSync, uint32_t enableCommonClockSource, uint32_t dosr, uint32_t enableFifo, uint32_t enableComparatorFilters, uint32_t sdfm0to1Clock)
{
    //DebugP_log("\nutil_sdfm_test base=0x%08x, testMode=%u, enablePwmSync=%u, enableCommonClockSource=%u, dosr=%u, enableFifo=%u, enableComparatorFilters=%u, sdfm0to1Clock=%u", base, testMode, enablePwmSync, enableCommonClockSource, dosr, enableFifo, enableComparatorFilters, sdfm0to1Clock);

    int32_t error=0;




    //Init
    util_inittimer();

    gFlagenableComparatorFilters = enableComparatorFilters;
    gFlagenableFifo = enableFifo;
    g_current_base = base;

    //Initialize log variables
    g_isr_count = 0;
    loopCounter1=0;

    uint32_t i;
    for( i=0;i<MAX_SAMPLES;i++)
    {
        filter1Result[i] = 0;
        filter2Result[i] = 0;
        filter3Result[i] = 0;
        filter4Result[i] = 0;

        if(i<10)
        {
            sdfmReadFlagRegister[i] = 0;
        }

        if(i<10)
        {
            isr_timestamp[i]=0;
        }
    }

    //DebugP_log("\nInit: Setup pins and generate input stream");

    util_connect_PWM_SDFM_loopback_pin(0);
    util_connect_PWM_SDFM_loopback_pin(1);
    util_connect_PWM_SDFM_loopback_pin(2);
    util_connect_PWM_SDFM_loopback_pin(3);
    util_connect_PWM_SDFM_loopback_pin(4);
    util_connect_PWM_SDFM_loopback_pin(5);
    util_connect_PWM_SDFM_loopback_pin(6);
    util_connect_PWM_SDFM_loopback_pin(7);

    uint32_t sdfm0_in_period=100;
    uint32_t sdfm1_in_period=300;
    uint32_t epwmmode;

    if(sdfm0to1Clock==0)
    {
        if(enablePwmSync!=PWM_SYNC_ENABLE)
        {
            epwmmode=EPWM_COUNTER_MODE_UP_DOWN;
        }

        if(enablePwmSync==PWM_SYNC_ENABLE)
        {
            epwmmode=EPWM_COUNTER_MODE_UP;
        }

        if(base==CSL_CONTROLSS_SDFM0_U_BASE)
        {
            //Configure PWM0, PWM1, PWM2, PWM3
            util_configPWM(CSL_CONTROLSS_G0_EPWM0_U_BASE, sdfm0_in_period, sdfm0_in_period/2, sdfm0_in_period/4, epwmmode);
            util_configPWM(CSL_CONTROLSS_G0_EPWM1_U_BASE, sdfm0_in_period, sdfm0_in_period/2, sdfm0_in_period/4, epwmmode);
            util_configPWM(CSL_CONTROLSS_G0_EPWM2_U_BASE, sdfm0_in_period, sdfm0_in_period/2, sdfm0_in_period/4, epwmmode);
            util_configPWM(CSL_CONTROLSS_G0_EPWM3_U_BASE, sdfm0_in_period, sdfm0_in_period/2, sdfm0_in_period/4, epwmmode);
        }
        if(base==CSL_CONTROLSS_SDFM1_U_BASE)
        {
            //Configure PWM4, PWM5, PWM6, PWM7
            util_configPWM(CSL_CONTROLSS_G0_EPWM4_U_BASE, sdfm1_in_period, sdfm1_in_period/2, sdfm1_in_period/4, epwmmode);
            util_configPWM(CSL_CONTROLSS_G0_EPWM5_U_BASE, sdfm1_in_period, sdfm1_in_period/2, sdfm1_in_period/4, epwmmode);
            util_configPWM(CSL_CONTROLSS_G0_EPWM6_U_BASE, sdfm1_in_period, sdfm1_in_period/2, sdfm1_in_period/4, epwmmode);
            util_configPWM(CSL_CONTROLSS_G0_EPWM7_U_BASE, sdfm1_in_period, sdfm1_in_period/2, sdfm1_in_period/4, epwmmode);
        }
    }
    if(sdfm0to1Clock==1)
    {
        //DebugP_log("\nInit: Test for SDFM1 clock from SDFM0");

        HW_WR_REG32(CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_SDFM1_CLK0_SEL ,CSL_CONTROLSS_CTRL_SDFM1_CLK0_SEL_SEL_MASK);

        if(base==CSL_CONTROLSS_SDFM0_U_BASE)
        {
            //DebugP_log("\nInit: SDFM0 input generation");
        }
        if(base==CSL_CONTROLSS_SDFM1_U_BASE)
        {
            //DebugP_log("\nInit: SDFM0 input generation only");
            //DebugP_log("\nInit: Forcing enableCommonClockSource to 1");
            enableCommonClockSource=1;
        }

        //Configure PWM0, PWM1, PWM2, PWM3
        util_configPWM(CSL_CONTROLSS_G0_EPWM0_U_BASE, sdfm0_in_period, sdfm0_in_period/2, sdfm0_in_period/4, EPWM_COUNTER_MODE_UP_DOWN);
        util_configPWM(CSL_CONTROLSS_G0_EPWM1_U_BASE, sdfm0_in_period, sdfm0_in_period/2, sdfm0_in_period/4, EPWM_COUNTER_MODE_UP_DOWN);
        util_configPWM(CSL_CONTROLSS_G0_EPWM2_U_BASE, sdfm0_in_period, sdfm0_in_period/2, sdfm0_in_period/4, EPWM_COUNTER_MODE_UP_DOWN);
        util_configPWM(CSL_CONTROLSS_G0_EPWM3_U_BASE, sdfm0_in_period, sdfm0_in_period/2, sdfm0_in_period/4, EPWM_COUNTER_MODE_UP_DOWN);
    }



    uint32_t intnum;
    if(testMode==TEST_MODE_CPUISRREAD)
    {
        //DebugP_log("\nInit: TEST_MODE_CPUISRREAD");
        //DebugP_log("\nInit: Setup interrupt");

        intnum = CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_0 + g_intxbar_num;

        /* Register & enable interrupt */
        HwiP_Params         hwiPrms;
        int32_t             status;

        HwiP_Params_init(&hwiPrms);
        /* Integrate with Syscfg */
        hwiPrms.intNum      = intnum;
        hwiPrms.callback    = &sdfmISR;
        /* Integrate with Syscfg */
        hwiPrms.isPulse     = 1;
        status              = HwiP_construct(&gSdfmHwiObject, &hwiPrms);
        DebugP_assert(status == SystemP_SUCCESS);



        //DebugP_log("\nInit: Route interrupt");

        if(enableComparatorFilters==COMPARATOR_DISABLE)
        {
            if(base==CSL_CONTROLSS_SDFM0_U_BASE)
            {
                //DebugP_log(" SDFM0 DRINT1,2,3,4");
                SOC_xbarSelectInterruptXBarInputSource(CSL_CONTROLSS_INTXBAR_U_BASE, g_intxbar_num, 0, 0, 0, 0, 0x0000001EU, 0, 0);
            }

            if(base==CSL_CONTROLSS_SDFM1_U_BASE)
            {
                //DebugP_log(" SDFM1 DRINT1,2,3,4");
                SOC_xbarSelectInterruptXBarInputSource(CSL_CONTROLSS_INTXBAR_U_BASE, g_intxbar_num, 0, 0, 0, 0, 0x000003C0U, 0, 0);
            }
        }

        if(enableComparatorFilters!=COMPARATOR_DISABLE)
        {
            if(base==CSL_CONTROLSS_SDFM0_U_BASE)
            {
                //DebugP_log(" SDFM0 SDERR");
                SOC_xbarSelectInterruptXBarInputSource(CSL_CONTROLSS_INTXBAR_U_BASE, g_intxbar_num, 0, 0, 0, 0, 0x00000001U, 0, 0);
            }

            if(base==CSL_CONTROLSS_SDFM1_U_BASE)
            {
                //DebugP_log(" SDFM1 SDERR");
                SOC_xbarSelectInterruptXBarInputSource(CSL_CONTROLSS_INTXBAR_U_BASE, g_intxbar_num, 0, 0, 0, 0, 0x00000020U, 0, 0);
            }
        }


#ifdef DRINT_WORKAROUND

        if(base==CSL_CONTROLSS_SDFM0_U_BASE)
        {
            //DebugP_log(" SDFM0 DRINT1,2,3,4 and SDERR");
            SOC_xbarSelectInterruptXBarInputSource(CSL_CONTROLSS_INTXBAR_U_BASE, g_intxbar_num, 0, 0, 0, 0, 0x0000001FU, 0, 0);
        }

        if(base==CSL_CONTROLSS_SDFM1_U_BASE)
        {
            //DebugP_log(" SDFM1 DRINT1,2,3,4 and SDERR");
            SOC_xbarSelectInterruptXBarInputSource(CSL_CONTROLSS_INTXBAR_U_BASE, g_intxbar_num, 0, 0, 0, 0, 0x000003E0U, 0, 0);
        }
#endif


    }

#define DMADST (0x70011000)

    const uint32_t ch_num = 0;
    if(testMode==TEST_MODE_DMAREAD)
    {
        //DebugP_log("\nInit: TEST_MODE_DMAREAD");
        //DebugP_log("\nInit: Setup DMA");

        //uint32_t dmanum = 115 + g_dmaxbar_num;


        /* Configure the DMA for transfer */
        /* Initialize DMA */
        //const uint32_t tpcc_addr = (uint32_t) CSL_TPCC0_U_BASE;
        //dma_wrap_init(tpcc_addr);

        //hw_trig_dma_xfer(0, (uint32_t) (base + CSL_SDFM_SDDATA1), (uint32_t) DMADST, 4,
        //                 dmanum, ch_num);

//
//        /* AB-sync transfer. Copies 'numArr' arrays, each of size 'arsize'
//        Arrays are placed at (srcAddr + i * srcAddrIncr) and copied to (desAddr + i * desAddrIncr), where i = [0, numArr) */
//        uint32_t srcAddr = (uint32_t) (base + CSL_SDFM_SDDATA1);
//
//
//        uint32_t desAddr = (uint32_t) DMADST;
//        uint32_t arsize = 4;
//        uint32_t numArr = 4;
//        int16_t srcAddrIncr = 0x20;
//        int16_t desAddrIncr = 4;
//        //dma_wrap_cfg_AB_sync_channel(tpcc_addr, ch_num, srcAddr, desAddr, arsize, numArr, srcAddrIncr, desAddrIncr);
//        dmap_wrap_cfg_channel(tpcc_addr, ch_num, srcAddr, desAddr, arsize);       //Try with this
//
//        uint32_t regionId = 0;  // TODO what is region ID?
//        EDMA3ClrMissEvtRegion(tpcc_addr, regionId, ch_num);
//        EDMA3EnableDmaEvtRegion(tpcc_addr, regionId, ch_num);
//
//        //DebugP_log("\nInit: Route DMA event in EDMA Trigger XBAR");
//        //cslr_edma_trig_xbar.h
//        HW_WR_REG32(CSL_EDMA_TRIG_XBAR_U_BASE + (CSL_EDMA_TRIG_XBAR_MUXCNTL(ch_num)), dmanum | ( 1 << CSL_EDMA_TRIG_XBAR_MUXCNTL_INT_ENABLE_SHIFT) );


        //DebugP_log("\nInit: Routed DMA event in EDMA Trigger XBAR 0x%08x", HW_RD_REG32(CSL_EDMA_TRIG_XBAR_U_BASE+4));

        //DebugP_log("\nInit: Route DMA event in DMAXBAR");
        if(base==CSL_CONTROLSS_SDFM0_U_BASE)
        {
            SOC_xbarSelectDMAXBarInputSource(CSL_CONTROLSS_DMAXBAR_U_BASE, g_dmaxbar_num, 4, 0, 0, 0, 0, 0x0+g_curr_dma_evt, 0);    //mux(g): 4(g4), mux(g4): 0(sdfm0 drint1)
            //DebugP_log("\n0x%08x", HW_RD_REG32(CSL_CONTROLSS_DMAXBAR_U_BASE + CSL_CONTROLSS_DMAXBAR_DMAXBAR0_GSEL));
            //DebugP_log("\n0x%08x", HW_RD_REG32(CSL_CONTROLSS_DMAXBAR_U_BASE + CSL_CONTROLSS_DMAXBAR_DMAXBAR0_G4));
        }
        if(base==CSL_CONTROLSS_SDFM1_U_BASE)
        {
            SOC_xbarSelectDMAXBarInputSource(CSL_CONTROLSS_DMAXBAR_U_BASE, g_dmaxbar_num, 4, 0, 0, 0, 0, 0x4+g_curr_dma_evt, 0);    //mux(g): 4(g4), mux(g4): 4(sdfm1 drint1)
            //DebugP_log("\n0x%08x", HW_RD_REG32(CSL_CONTROLSS_DMAXBAR_U_BASE + CSL_CONTROLSS_DMAXBAR_DMAXBAR0_GSEL));
            //DebugP_log("\n0x%08x", HW_RD_REG32(CSL_CONTROLSS_DMAXBAR_U_BASE + CSL_CONTROLSS_DMAXBAR_DMAXBAR0_G4));
        }


    }


    //DebugP_log("\nInit: Setup SDFM");

    //Writes to CSL_SDFM_SDCTLPARM1, MOD field (CSL_SDFM_SDCTLPARM1_MOD_MASK),
    //and sets SDCLKSYNC (CSL_SDFM_SDCTLPARM1_SDCLKSYNC_MASK), SDDATASYNC(CSL_SDFM_SDCTLPARM1_SDDATASYNC_MASK)
    //SDFM_MODULATOR_CLK_EQUAL_DATA_RATE = 0
    //SDFM_FILTER_1/2/3/4 = 0/1/2/3
    SDFM_setupModulatorClock(base, SDFM_FILTER_1, SDFM_MODULATOR_CLK_EQUAL_DATA_RATE);
    SDFM_setupModulatorClock(base, SDFM_FILTER_2, SDFM_MODULATOR_CLK_EQUAL_DATA_RATE);
    SDFM_setupModulatorClock(base, SDFM_FILTER_3, SDFM_MODULATOR_CLK_EQUAL_DATA_RATE);
    SDFM_setupModulatorClock(base, SDFM_FILTER_4, SDFM_MODULATOR_CLK_EQUAL_DATA_RATE);

    if(enableCommonClockSource==1)
    {
        SDFM_selectClockSource(base, SDFM_FILTER_2, SDFM_CLK_SOURCE_SD1_CLK);
        SDFM_selectClockSource(base, SDFM_FILTER_3, SDFM_CLK_SOURCE_SD1_CLK);
        SDFM_selectClockSource(base, SDFM_FILTER_4, SDFM_CLK_SOURCE_SD1_CLK);
    }
    else
    {
        SDFM_selectClockSource(base, SDFM_FILTER_2, SDFM_CLK_SOURCE_CHANNEL_CLK);
        SDFM_selectClockSource(base, SDFM_FILTER_3, SDFM_CLK_SOURCE_CHANNEL_CLK);
        SDFM_selectClockSource(base, SDFM_FILTER_4, SDFM_CLK_SOURCE_CHANNEL_CLK);
    }


    if(enableComparatorFilters!=COMPARATOR_DISABLE)
    {
        //DebugP_log("\nInit: Comparator filter setup");

        uint16_t  hlt, llt;
        uint16_t cosr=32;

        //hlt = 0x7FFF;
        //llt = 0x0000;

        hlt = 0x8000; //1 000 0000 0000 0000, 15 bit min, -32768
        llt = 0x7FFF; //0 111 1111 1111 1111, 15 bit max, 32767

        //DebugP_log("\nInit: SDFM cosr=%u, hlt=%d, llt=%d", cosr, (int16_t)hlt, (int16_t)llt);

        //SDFM_FILTER_1/2/3/4 = 0/1/2/3
        //SDFM_FILTER_SINC_3 = 0x30
        //SDFM_SET_OSR(32) = (32-1) << 8
        //SDFM_SET_OSR(128) = (128-1) << 8
        //SDFM_GET_LOW_THRESHOLD(llt) = ((uint16_t)(llt))
        //SDFM_GET_HIGH_THRESHOLD(hlt) = ((uint16_t)((uint32_t)(hlt) >> 16U))
        SDFM_configComparator(base, (SDFM_FILTER_1 | SDFM_FILTER_SINC_3 | SDFM_SET_OSR(cosr)), (SDFM_GET_LOW_THRESHOLD(llt) | SDFM_GET_HIGH_THRESHOLD(hlt)), 0);
        SDFM_configComparator(base, (SDFM_FILTER_2 | SDFM_FILTER_SINC_3 | SDFM_SET_OSR(cosr)), (SDFM_GET_LOW_THRESHOLD(llt) | SDFM_GET_HIGH_THRESHOLD(hlt)), 0);
        SDFM_configComparator(base, (SDFM_FILTER_3 | SDFM_FILTER_SINC_3 | SDFM_SET_OSR(cosr)), (SDFM_GET_LOW_THRESHOLD(llt) | SDFM_GET_HIGH_THRESHOLD(hlt)), 0);
        SDFM_configComparator(base, (SDFM_FILTER_4 | SDFM_FILTER_SINC_3 | SDFM_SET_OSR(cosr)), (SDFM_GET_LOW_THRESHOLD(llt) | SDFM_GET_HIGH_THRESHOLD(hlt)), 0);

        if(enableComparatorFilters==COMPARATOR_AND_FILTER_ENABLE)
        {
            SDFM_CompEventFilterConfig config;
            config.clkPrescale=1;
            config.sampleWindow=32;
            config.threshold=17;

            SDFM_configCompEventHighFilter(base, SDFM_FILTER_1, &config);
            SDFM_configCompEventHighFilter(base, SDFM_FILTER_2, &config);
            SDFM_configCompEventHighFilter(base, SDFM_FILTER_3, &config);
            SDFM_configCompEventHighFilter(base, SDFM_FILTER_4, &config);

            SDFM_configCompEventLowFilter(base, SDFM_FILTER_1, &config);
            SDFM_configCompEventLowFilter(base, SDFM_FILTER_2, &config);
            SDFM_configCompEventLowFilter(base, SDFM_FILTER_3, &config);
            SDFM_configCompEventLowFilter(base, SDFM_FILTER_4, &config);

            SDFM_initCompEventHighFilter(base, SDFM_FILTER_1);
            SDFM_initCompEventLowFilter(base, SDFM_FILTER_1);
            SDFM_initCompEventHighFilter(base, SDFM_FILTER_2);
            SDFM_initCompEventLowFilter(base, SDFM_FILTER_2);
            SDFM_initCompEventHighFilter(base, SDFM_FILTER_3);
            SDFM_initCompEventLowFilter(base, SDFM_FILTER_3);
            SDFM_initCompEventHighFilter(base, SDFM_FILTER_4);
            SDFM_initCompEventLowFilter(base, SDFM_FILTER_4);

        }



        if(testMode==TEST_MODE_PWMXBAR_OUT)
        {
            //DebugP_log("\nInit: TEST_MODE_PWMXBAR_OUT");
            //Configure PWM Xbar
            if(util_getsdfminstancefrombase(base)==0)
            {
                SOC_xbarSelectPWMXBarInputSource(CSL_CONTROLSS_PWMXBAR_U_BASE, g_xbarout, 0, 0, 0x000FFF, 0, 0, 0, 0, 0, 0);
            }
            if(util_getsdfminstancefrombase(base)==1)
            {
                SOC_xbarSelectPWMXBarInputSource(CSL_CONTROLSS_PWMXBAR_U_BASE, g_xbarout, 0, 0, 0xFFF000, 0, 0, 0, 0, 0, 0);
            }
        }


        if(testMode==TEST_MODE_OUTPUTXBAR_OUT)
        {
            //DebugP_log("\nInit: TEST_MODE_OUTPUTXBAR_OUT");
            //Configure Output Xbar

            if(util_getsdfminstancefrombase(base)==0)
            {
                SOC_xbarSelectOutputXBarInputSource(CSL_CONTROLSS_OUTPUTXBAR_U_BASE, g_xbarout, 0,0,0,0,0,0x000FFF,0,0,0,0,0);
            }
            if(util_getsdfminstancefrombase(base)==1)
            {
                SOC_xbarSelectOutputXBarInputSource(CSL_CONTROLSS_OUTPUTXBAR_U_BASE, g_xbarout, 0,0,0,0,0,0xFFF000,0,0,0,0,0);
            }
        }


        if(testMode==TEST_MODE_ECAPMUX_OUT)
        {
            //DebugP_log("\nInit: TEST_MODE_ECAPMUX_OUT");
            //Configure ECAP
            util_initECAP(g_ecapbase, ECAP_INPUT_SDFM0_COMPARE1_HIGH + (util_getsdfminstancefrombase(base) * 12));


        }
    }

    uint16_t shift=get_sinc3_shift_from_dosr(dosr);
    //DebugP_log("\nInit: SDFM dosr=%u, shift=%u", dosr, shift);

    SDFM_configDataFilter(base, (SDFM_FILTER_1 | SDFM_FILTER_SINC_3 | SDFM_SET_OSR(dosr)), (SDFM_DATA_FORMAT_16_BIT | SDFM_FILTER_ENABLE | SDFM_SHIFT_VALUE(shift)));
    SDFM_configDataFilter(base, (SDFM_FILTER_2 | SDFM_FILTER_SINC_3 | SDFM_SET_OSR(dosr)), (SDFM_DATA_FORMAT_16_BIT | SDFM_FILTER_ENABLE | SDFM_SHIFT_VALUE(shift)));
    SDFM_configDataFilter(base, (SDFM_FILTER_3 | SDFM_FILTER_SINC_3 | SDFM_SET_OSR(dosr)), (SDFM_DATA_FORMAT_16_BIT | SDFM_FILTER_ENABLE | SDFM_SHIFT_VALUE(shift)));
    SDFM_configDataFilter(base, (SDFM_FILTER_4 | SDFM_FILTER_SINC_3 | SDFM_SET_OSR(dosr)), (SDFM_DATA_FORMAT_16_BIT | SDFM_FILTER_ENABLE | SDFM_SHIFT_VALUE(shift)));

    //Writes to CSL_SDFM_SDMFILEN
    //Sets Main Filter Enable bit. Bit 11 (CSL_SDFM_SDMFILEN_MFE_MASK)
    SDFM_enableMainFilter(base);

    //Writes to CSL_SDFM_SDDFPARM1/2/3/4
    //Clears SDSYNCEN bit. Bit 12 CSL_SDFM_SDDFPARM1_SDSYNCEN_MASK
    //Disables PWM synchronization of data filter
    SDFM_disableExternalReset(base, SDFM_FILTER_1);
    SDFM_disableExternalReset(base, SDFM_FILTER_2);
    SDFM_disableExternalReset(base, SDFM_FILTER_3);
    SDFM_disableExternalReset(base, SDFM_FILTER_4);

    if(enableComparatorFilters==COMPARATOR_DISABLE)
    {
        SDFM_enableInterrupt(base, SDFM_FILTER_1, (SDFM_MODULATOR_FAILURE_INTERRUPT | SDFM_DATA_FILTER_ACKNOWLEDGE_INTERRUPT));
        SDFM_enableInterrupt(base, SDFM_FILTER_2, (SDFM_MODULATOR_FAILURE_INTERRUPT | SDFM_DATA_FILTER_ACKNOWLEDGE_INTERRUPT));
        SDFM_enableInterrupt(base, SDFM_FILTER_3, (SDFM_MODULATOR_FAILURE_INTERRUPT | SDFM_DATA_FILTER_ACKNOWLEDGE_INTERRUPT));
        SDFM_enableInterrupt(base, SDFM_FILTER_4, (SDFM_MODULATOR_FAILURE_INTERRUPT | SDFM_DATA_FILTER_ACKNOWLEDGE_INTERRUPT));

        //SDFM_enableInterrupt(base, SDFM_FILTER_1, (SDFM_DATA_FILTER_ACKNOWLEDGE_INTERRUPT));
        //SDFM_enableInterrupt(base, SDFM_FILTER_2, (SDFM_DATA_FILTER_ACKNOWLEDGE_INTERRUPT));
        //SDFM_enableInterrupt(base, SDFM_FILTER_3, (SDFM_DATA_FILTER_ACKNOWLEDGE_INTERRUPT));
        //SDFM_enableInterrupt(base, SDFM_FILTER_4, (SDFM_DATA_FILTER_ACKNOWLEDGE_INTERRUPT));

        SDFM_disableInterrupt(base, SDFM_FILTER_1, (SDFM_CEVT2_INTERRUPT | SDFM_CEVT1_INTERRUPT));
        SDFM_disableInterrupt(base, SDFM_FILTER_2, (SDFM_CEVT2_INTERRUPT | SDFM_CEVT1_INTERRUPT));
        SDFM_disableInterrupt(base, SDFM_FILTER_3, (SDFM_CEVT2_INTERRUPT | SDFM_CEVT1_INTERRUPT));
        SDFM_disableInterrupt(base, SDFM_FILTER_4, (SDFM_CEVT2_INTERRUPT | SDFM_CEVT1_INTERRUPT));

        //Type 1 specific
        SDFM_disableComparator(base, SDFM_FILTER_1);
        SDFM_disableComparator(base, SDFM_FILTER_2);
        SDFM_disableComparator(base, SDFM_FILTER_3);
        SDFM_disableComparator(base, SDFM_FILTER_4);
    }

    if(enableComparatorFilters!=COMPARATOR_DISABLE)
    {
        SDFM_enableInterrupt(base, SDFM_FILTER_1, (SDFM_CEVT2_INTERRUPT | SDFM_CEVT1_INTERRUPT));
        SDFM_enableInterrupt(base, SDFM_FILTER_2, (SDFM_CEVT2_INTERRUPT | SDFM_CEVT1_INTERRUPT));
        SDFM_enableInterrupt(base, SDFM_FILTER_3, (SDFM_CEVT2_INTERRUPT | SDFM_CEVT1_INTERRUPT));
        SDFM_enableInterrupt(base, SDFM_FILTER_4, (SDFM_CEVT2_INTERRUPT | SDFM_CEVT1_INTERRUPT));

        SDFM_selectCompEventSource(base, SDFM_FILTER_1, SDFM_COMP_EVENT_1, SDFM_COMP_EVENT_SRC_COMPH1_L1);
        SDFM_selectCompEventSource(base, SDFM_FILTER_1, SDFM_COMP_EVENT_2, SDFM_COMP_EVENT_SRC_COMPH2_L2);

        SDFM_selectCompEventSource(base, SDFM_FILTER_2, SDFM_COMP_EVENT_1, SDFM_COMP_EVENT_SRC_COMPH1_L1);
        SDFM_selectCompEventSource(base, SDFM_FILTER_2, SDFM_COMP_EVENT_2, SDFM_COMP_EVENT_SRC_COMPH2_L2);

        SDFM_selectCompEventSource(base, SDFM_FILTER_3, SDFM_COMP_EVENT_1, SDFM_COMP_EVENT_SRC_COMPH1_L1);
        SDFM_selectCompEventSource(base, SDFM_FILTER_3, SDFM_COMP_EVENT_2, SDFM_COMP_EVENT_SRC_COMPH2_L2);

        SDFM_selectCompEventSource(base, SDFM_FILTER_4, SDFM_COMP_EVENT_1, SDFM_COMP_EVENT_SRC_COMPH1_L1);
        SDFM_selectCompEventSource(base, SDFM_FILTER_4, SDFM_COMP_EVENT_2, SDFM_COMP_EVENT_SRC_COMPH2_L2);

        //Type 1 specific
        SDFM_enableComparator(base, SDFM_FILTER_1);
        SDFM_enableComparator(base, SDFM_FILTER_2);
        SDFM_enableComparator(base, SDFM_FILTER_3);
        SDFM_enableComparator(base, SDFM_FILTER_4);

        if(enableComparatorFilters==COMPARATOR_AND_FILTER_ENABLE)
        {
            SDFM_selectCompEventLowSource(base, SDFM_FILTER_1, SDFM_COMPLOUT_SOURCE_FILTER);
            SDFM_selectCompEventLowSource(base, SDFM_FILTER_2, SDFM_COMPLOUT_SOURCE_FILTER);
            SDFM_selectCompEventLowSource(base, SDFM_FILTER_3, SDFM_COMPLOUT_SOURCE_FILTER);
            SDFM_selectCompEventLowSource(base, SDFM_FILTER_4, SDFM_COMPLOUT_SOURCE_FILTER);

            SDFM_selectCompEventHighSource(base, SDFM_FILTER_1, SDFM_COMPHOUT_SOURCE_FILTER);
            SDFM_selectCompEventHighSource(base, SDFM_FILTER_2, SDFM_COMPHOUT_SOURCE_FILTER);
            SDFM_selectCompEventHighSource(base, SDFM_FILTER_3, SDFM_COMPHOUT_SOURCE_FILTER);
            SDFM_selectCompEventHighSource(base, SDFM_FILTER_4, SDFM_COMPHOUT_SOURCE_FILTER);
        }

    }



    //FIFO
    if(enableFifo==1)
    {
        //DebugP_log("\nInit: Enabling FIFO");
        SDFM_enableInterrupt(base, SDFM_FILTER_1, SDFM_FIFO_INTERRUPT);
        SDFM_enableInterrupt(base, SDFM_FILTER_2, SDFM_FIFO_INTERRUPT);
        SDFM_enableInterrupt(base, SDFM_FILTER_3, SDFM_FIFO_INTERRUPT);
        SDFM_enableInterrupt(base, SDFM_FILTER_4, SDFM_FIFO_INTERRUPT);

        SDFM_enableFIFOBuffer(base, SDFM_FILTER_1);
        SDFM_enableFIFOBuffer(base, SDFM_FILTER_2);
        SDFM_enableFIFOBuffer(base, SDFM_FILTER_3);
        SDFM_enableFIFOBuffer(base, SDFM_FILTER_4);

        SDFM_setDataReadyInterruptSource(base, SDFM_FILTER_1, SDFM_DATA_READY_SOURCE_FIFO);
        SDFM_setDataReadyInterruptSource(base, SDFM_FILTER_2, SDFM_DATA_READY_SOURCE_FIFO);
        SDFM_setDataReadyInterruptSource(base, SDFM_FILTER_3, SDFM_DATA_READY_SOURCE_FIFO);
        SDFM_setDataReadyInterruptSource(base, SDFM_FILTER_4, SDFM_DATA_READY_SOURCE_FIFO);

        SDFM_setFIFOInterruptLevel(base, SDFM_FILTER_1, g_FIFO_INT_NUM);
        SDFM_setFIFOInterruptLevel(base, SDFM_FILTER_2, g_FIFO_INT_NUM);
        SDFM_setFIFOInterruptLevel(base, SDFM_FILTER_3, g_FIFO_INT_NUM);
        SDFM_setFIFOInterruptLevel(base, SDFM_FILTER_4, g_FIFO_INT_NUM);


    }

    //DebugP_log("\nSDFIFOCTLx=0x%04x 0x%04x 0x%04x 0x%04x", HW_RD_REG16(base + CSL_SDFM_SDFIFOCTL1), HW_RD_REG16(base + CSL_SDFM_SDFIFOCTL2), HW_RD_REG16(base + CSL_SDFM_SDFIFOCTL3), HW_RD_REG16(base + CSL_SDFM_SDFIFOCTL4));


    if(enablePwmSync==PWM_SYNC_ENABLE)
    {
        //DebugP_log("\nInit: Enabling PWM Sync");
        //TBD
        SDFM_enableExternalReset(base, SDFM_FILTER_1);
        SDFM_enableExternalReset(base, SDFM_FILTER_2);
        SDFM_enableExternalReset(base, SDFM_FILTER_3);
        SDFM_enableExternalReset(base, SDFM_FILTER_4);



        if(base==CSL_CONTROLSS_SDFM0_U_BASE)
        {
            if( (g_pwm_num_sync==0) || (g_pwm_num_sync==1) || (g_pwm_num_sync==2) || (g_pwm_num_sync==3) )
            {
                //Already intialized
            }
            else
            {
                util_configPWM(CSL_CONTROLSS_G0_EPWM0_U_BASE + (g_pwm_num_sync*0x1000), sdfm0_in_period, sdfm0_in_period/2, sdfm0_in_period/4, EPWM_COUNTER_MODE_UP);
            }

        }
        if(base==CSL_CONTROLSS_SDFM1_U_BASE)
        {
            if( (g_pwm_num_sync==4) || (g_pwm_num_sync==5) || (g_pwm_num_sync==6) || (g_pwm_num_sync==7) )
            {
                //Already intialized
            }
            else
            {
                util_configPWM(CSL_CONTROLSS_G0_EPWM0_U_BASE + (g_pwm_num_sync*0x1000), sdfm0_in_period, sdfm0_in_period/2, sdfm0_in_period/4, EPWM_COUNTER_MODE_UP);
            }

        }

        //Configure PWMx SOCA
        EPWM_enableADCTrigger(CSL_CONTROLSS_G0_EPWM0_U_BASE + (g_pwm_num_sync*0x1000), EPWM_SOC_A);
        EPWM_setADCTriggerSource(CSL_CONTROLSS_G0_EPWM0_U_BASE + (g_pwm_num_sync*0x1000), EPWM_SOC_A, EPWM_SOC_TBCTR_ZERO, 0);
        EPWM_setADCTriggerEventPrescale(CSL_CONTROLSS_G0_EPWM0_U_BASE + (g_pwm_num_sync*0x1000), EPWM_SOC_A, 0);

        SDFM_setPWMSyncSource(base, SDFM_FILTER_1, SDFM_SYNC_PWM0_SOCA + (SDFM_SYNC_PWM1_SOCA-SDFM_SYNC_PWM0_SOCA)*g_pwm_num_sync);
        SDFM_setPWMSyncSource(base, SDFM_FILTER_2, SDFM_SYNC_PWM0_SOCA + (SDFM_SYNC_PWM1_SOCA-SDFM_SYNC_PWM0_SOCA)*g_pwm_num_sync);
        SDFM_setPWMSyncSource(base, SDFM_FILTER_3, SDFM_SYNC_PWM0_SOCA + (SDFM_SYNC_PWM1_SOCA-SDFM_SYNC_PWM0_SOCA)*g_pwm_num_sync);
        SDFM_setPWMSyncSource(base, SDFM_FILTER_4, SDFM_SYNC_PWM0_SOCA + (SDFM_SYNC_PWM1_SOCA-SDFM_SYNC_PWM0_SOCA)*g_pwm_num_sync);

    }

    //Writes to CSL_SDFM_SDCTL, sets MIE bit. Bit 13 (CSL_SDFM_SDCTL_MIE_MASK)
    //Main SDy_ERR interrupt enable
    SDFM_enableMainInterrupt(base);







    //Trigger test and check pass/fail


    if(testMode==TEST_MODE_CPUPOLL)
    {
        //DebugP_log("\nRun: TEST_MODE_CPUPOLL: Polling for SDIFLG");
        uint32_t poll_count=100000;
        uint32_t poll_sdiflg[10]={0};
        while(poll_count>0)
        {
            poll_sdiflg[poll_count%10] = HW_RD_REG32(base + CSL_SDFM_SDIFLG);
            poll_count--;
        }

        for(poll_count=0;poll_count<10;poll_count++)
        {
            //DebugP_log("\nSDIFLG=0x%08x", poll_sdiflg[poll_count]);
        }

        if(enableComparatorFilters==COMPARATOR_DISABLE)
        {
            if(enableFifo!=1)
            {
                if( (poll_sdiflg[10-1]&(CSL_SDFM_SDIFLG_AF1_MASK | CSL_SDFM_SDIFLG_AF2_MASK | CSL_SDFM_SDIFLG_AF3_MASK | CSL_SDFM_SDIFLG_AF4_MASK)) != (CSL_SDFM_SDIFLG_AF1_MASK | CSL_SDFM_SDIFLG_AF2_MASK | CSL_SDFM_SDIFLG_AF3_MASK | CSL_SDFM_SDIFLG_AF4_MASK))
                {
                    //DebugP_log(" error");
                    error++;
                }
            }
        }
        if(enableComparatorFilters!=COMPARATOR_DISABLE)
        {

        }
    }

    if( (testMode==TEST_MODE_PWMXBAR_OUT) || (testMode==TEST_MODE_OUTPUTXBAR_OUT) || (testMode==TEST_MODE_ECAPMUX_OUT))
    {
        //Wait for comparator filters
        uint32_t poll_count=100000;
        uint32_t poll_sdiflg[10]={0};
        while(poll_count>0)
        {
            poll_sdiflg[poll_count%10] = HW_RD_REG32(base + CSL_SDFM_SDIFLG);
            poll_count--;
        }

        for(poll_count=0;poll_count<10;poll_count++)
        {
            ////DebugP_log("\nSDIFLG=0x%08x", poll_sdiflg[poll_count]);
        }

        //DebugP_log("\nRun: SDIFLG=0x%08x", HW_RD_REG32(base + CSL_SDFM_SDIFLG));
    }


    //Check PWM Xbar output status
    if(testMode==TEST_MODE_PWMXBAR_OUT)
    {
        //DebugP_log("\nRun: TEST_MODE_PWMXBAR_OUT");

        uint32_t status=SOC_xbarGetPWMXBarOutputSignalStatus(CSL_CONTROLSS_PWMXBAR_U_BASE);
        if(status != (0x1<<g_xbarout))
        {
            //DebugP_log("\nerror");
            error++;
        }
        //DebugP_log("\nPWMXBar %d SOC_xbarGetPWMXBarOutputSignalStatus returns 0x%08x", g_xbarout, status);
    }

    //Check Output Xbar output status
    if(testMode==TEST_MODE_OUTPUTXBAR_OUT)
    {
        //DebugP_log("\nRun: TEST_MODE_OUTPUTXBAR_OUT");
        uint32_t status=SOC_xbarGetOutputXBarOutputSignalStatus(CSL_CONTROLSS_OUTPUTXBAR_U_BASE);
        if(status != (0x1<<g_xbarout))
        {
            //DebugP_log("\nerror");
            error++;
        }
        //DebugP_log("\nOutputXbar %d SOC_xbarGetOutputXBarOutputSignalStatus returns 0x%08x", g_xbarout, status);
    }

    //Check ECAP status
    if(testMode==TEST_MODE_ECAPMUX_OUT)
    {
        //DebugP_log("\nRun: TEST_MODE_ECAPMUX_OUT");

        //Toggle input event
        //DebugP_log("\nRun: toggling input");
        uint32_t toggle;

        for(toggle=0;toggle<4;toggle++)
        {
            SDFM_disableComparator(base, SDFM_FILTER_1);

            uint32_t waitcount=1000;
            while(waitcount>0)
            {
                ECAP_getEventTimeStamp(g_ecapbase, ECAP_EVENT_1);
                waitcount--;
            }

            SDFM_enableComparator(base, SDFM_FILTER_1);

            waitcount=1000;
            while(waitcount>0)
            {
                ECAP_getEventTimeStamp(g_ecapbase, ECAP_EVENT_1);
                waitcount--;
            }

        }


        //Check ecap timestamps
        uint32_t timestamp_event1 = ECAP_getEventTimeStamp(g_ecapbase, ECAP_EVENT_1);
        uint32_t timestamp_event2 = ECAP_getEventTimeStamp(g_ecapbase, ECAP_EVENT_2);
        uint32_t timestamp_event3 = ECAP_getEventTimeStamp(g_ecapbase, ECAP_EVENT_3);
        uint32_t timestamp_event4 = ECAP_getEventTimeStamp(g_ecapbase, ECAP_EVENT_4);
        //DebugP_log("\ntimestamp_event1 = 0x%08x, timestamp_event2 = 0x%08x, timestamp_event3 = 0x%08x, timestamp_event4 = 0x%08x", timestamp_event1, timestamp_event2, timestamp_event3, timestamp_event4);
        //tbd: verify
        if(timestamp_event1==0)
        {
            if(timestamp_event2==0)
            {
                if(timestamp_event3==0)
                {
                    if(timestamp_event4==0)
                    {
                        //DebugP_log("\nerror");
                        error++;
                    }
                }
            }
        }
    }

    if(testMode==TEST_MODE_CPUISRREAD)
    {
        if(enableComparatorFilters==COMPARATOR_DISABLE)
        {
            //DebugP_log("\nRun: TEST_MODE_CPUISRREAD: Wait for periodic data ready interrupt");
        }
        if(enableComparatorFilters!=COMPARATOR_DISABLE)
        {
            //DebugP_log("\nRun: TEST_MODE_CPUISRREAD: Wait for periodic SDy_ERR interrupt");
        }


        uint32_t wait_count=100000;
        while(wait_count>0)
        {
            //uint32_t readval = HW_RD_REG32(base + CSL_SDFM_SDIFLG);
            HW_WR_REG16(0x50260000, HW_RD_REG16(0x50260000));
            wait_count--;
        }


        SDFM_disableMainInterrupt(base);

        if(g_isr_count==0)
        {
        //DebugP_log("\nWait time out");
        //DebugP_log("\nSDIFLG=0x%08x", HW_RD_REG32(base + CSL_SDFM_SDIFLG));
        error++;
        }
        else
        {
            //DebugP_log("\nISR count %d", g_isr_count);
            //DebugP_log("\nlast SDIFLG=0x%08x", HW_RD_REG32(base + CSL_SDFM_SDIFLG));

            uint16_t loopindex=0;
            uint16_t printcount=0;
            if(g_isr_count>10)
            {
                printcount=10;
            }
            else
            {
                printcount=g_isr_count; //g_isr_count is min 1 here
            }
            for(loopindex=0;loopindex<printcount;loopindex++)
            {
                if(enableComparatorFilters==COMPARATOR_DISABLE)
                {
                    //DebugP_log("\nfilterxResult[%d]: 0x%04x, 0x%04x, 0x%04x, 0x%04x", loopindex, filter1Result[loopindex], filter2Result[loopindex], filter3Result[loopindex], filter4Result[loopindex]);
                }
                if(enableComparatorFilters!=COMPARATOR_DISABLE)
                {
                    //DebugP_log("\ncompFilterxResult[%d]: 0x%04x, 0x%04x, 0x%04x, 0x%04x", loopindex, compFilter1Result[loopindex], compFilter2Result[loopindex], compFilter3Result[loopindex], compFilter4Result[loopindex]);
                }
                //DebugP_log(" SDIFLG=0x%08x", sdfmReadFlagRegister[loopindex]);
                //DebugP_log(" isr_timestamp=%u", isr_timestamp[loopindex]);
            }

            if(enableComparatorFilters==COMPARATOR_DISABLE)
            {
                if(enableFifo!=1)
                {
#define AFMASK (CSL_SDFM_SDIFLG_AF1_MASK | CSL_SDFM_SDIFLG_AF2_MASK | CSL_SDFM_SDIFLG_AF3_MASK | CSL_SDFM_SDIFLG_AF4_MASK)
                    if( (sdfmReadFlagRegister[printcount-1]&AFMASK) == 0)   //all flags may not be set
                    {
                        //DebugP_log(" error. No AFx set");
                        error++;
                    }
                    else
                    {
                        //DebugP_log(" ok AFx");
                    }
                }
                if(enableFifo==1)
                {
#define SDFFINTMASK (CSL_SDFM_SDIFLG_SDFFINT1_MASK | CSL_SDFM_SDIFLG_SDFFINT2_MASK | CSL_SDFM_SDIFLG_SDFFINT3_MASK | CSL_SDFM_SDIFLG_SDFFINT4_MASK)
                    if( (sdfmReadFlagRegister[printcount-1]&SDFFINTMASK) == 0)     //all flags may not be set
                    {
                        //DebugP_log(" error. No SDFFINTx set");
                        error++;
                    }
                    else
                    {
                        //DebugP_log(" ok SDFFINTx");
                    }
                }
            }
            if(enableComparatorFilters!=COMPARATOR_DISABLE)
            {
#define CEVTMASK (CSL_SDFM_SDIFLG_FLT1_FLG_CEVT1_MASK| CSL_SDFM_SDIFLG_FLT1_FLG_CEVT2_MASK| CSL_SDFM_SDIFLG_FLT2_FLG_CEVT1_MASK| CSL_SDFM_SDIFLG_FLT2_FLG_CEVT2_MASK| CSL_SDFM_SDIFLG_FLT3_FLG_CEVT1_MASK| CSL_SDFM_SDIFLG_FLT3_FLG_CEVT2_MASK| CSL_SDFM_SDIFLG_FLT4_FLG_CEVT1_MASK|CSL_SDFM_SDIFLG_FLT4_FLG_CEVT2_MASK)
                if( (sdfmReadFlagRegister[printcount-1]&CEVTMASK) == 0)
                {
                    //DebugP_log(" error. No FLTx_FLG_CEVTx bit set");
                    error++;
                }
                else
                {
                    //DebugP_log(" ok FLTx_FLG_CEVTx");
                }
            }

        }
    }

    if(testMode==TEST_MODE_DMAREAD)
    {
        //DebugP_log("\nRun: TEST_MODE_DMAREAD: Check dma transfer completion");


        /* Wait for the transfer to complete (polling mode) */
        volatile uint32_t cnt_param = 0;

        uint32_t wait_count=100000;
        while(wait_count>0)
        {
            cnt_param = (uint32_t)(0x2 & HW_RD_REG32(CSL_TPTC00_U_BASE + (uint32_t)EDMA_TC_INTSTAT));
            if(cnt_param == 2)
            {
                //DebugP_log("\ncnt_param=0x%08x", cnt_param);
                break;
            }
            wait_count--;
        }

        //DebugP_log("\nWait done. last cnt_param=0x%08x", cnt_param);
        //DebugP_log("\nSDIFLG=0x%08x", HW_RD_REG32(base + CSL_SDFM_SDIFLG));

        if(cnt_param != 2)
        {
            //DebugP_log(" error");
            error++;
        }

        //DebugP_log("\ndestination memory contents: 0x%08x 0x%08x 0x%08x 0x%08x", HW_RD_REG32(DMADST), HW_RD_REG32(DMADST+4), HW_RD_REG32(DMADST+8), HW_RD_REG32(DMADST+12));

        //
        HW_WR_REG32(CSL_TPTC00_U_BASE + (uint32_t)EDMA_TC_INTCLR, 0x02);

    }





    // Deinit


    if(testMode==TEST_MODE_CPUISRREAD)
    {
        //DebugP_log("\nDeinit: TEST_MODE_CPUISRREAD");
        //DebugP_log("\nDeinit intxbar");
        SOC_xbarSelectInterruptXBarInputSource(CSL_CONTROLSS_INTXBAR_U_BASE, g_intxbar_num, 0, 0, 0, 0, 0, 0, 0);
    }

    if(testMode==TEST_MODE_DMAREAD)
    {
        //DebugP_log("\nDeinit: TEST_MODE_DMAREAD");

        //DebugP_log("\nDeinit DMAXBAR");
        SOC_xbarSelectDMAXBarInputSource(CSL_CONTROLSS_DMAXBAR_U_BASE, g_dmaxbar_num, 0, 0, 0, 0, 0, 0, 0);

        //DebugP_log("\nDeinit EDMA Trigger XBAR");
        HW_WR_REG32(CSL_EDMA_TRIG_XBAR_U_BASE + (CSL_EDMA_TRIG_XBAR_MUXCNTL(ch_num)), 0 | ( 0 << CSL_EDMA_TRIG_XBAR_MUXCNTL_INT_ENABLE_SHIFT) );

    }

    if(testMode==TEST_MODE_PWMXBAR_OUT)
    {
        //DebugP_log("\nDeinit: TEST_MODE_PWMXBAR_OUT");
        //Deinit PWMXBAR
        SOC_xbarSelectPWMXBarInputSource(CSL_CONTROLSS_PWMXBAR_U_BASE, g_xbarout, 0, 0, 0x000000, 0, 0, 0, 0, 0, 0);
    }

    if(testMode==TEST_MODE_OUTPUTXBAR_OUT)
    {
        //DebugP_log("\nDeinit: TEST_MODE_OUTPUTXBAR_OUT");
        //Deinit OUTPUTXBAR
        SOC_xbarSelectOutputXBarInputSource(CSL_CONTROLSS_OUTPUTXBAR_U_BASE, g_xbarout, 0,0,0,0,0,0x000000,0,0,0,0,0);

    }

    if(testMode==TEST_MODE_ECAPMUX_OUT)
    {
        //DebugP_log("\nDeinit: TEST_MODE_ECAPMUX_OUT");
        //Deinit ECAP
        util_deinit_ecaps();

    }

    //DebugP_log("\nDeinit: SDFM_disableInterrupt ");
    SDFM_disableInterrupt(base, SDFM_FILTER_1, (SDFM_DATA_FILTER_ACKNOWLEDGE_INTERRUPT ));
    SDFM_disableInterrupt(base, SDFM_FILTER_2, (SDFM_DATA_FILTER_ACKNOWLEDGE_INTERRUPT ));
    SDFM_disableInterrupt(base, SDFM_FILTER_3, (SDFM_DATA_FILTER_ACKNOWLEDGE_INTERRUPT ));
    SDFM_disableInterrupt(base, SDFM_FILTER_4, (SDFM_DATA_FILTER_ACKNOWLEDGE_INTERRUPT ));

    if(enablePwmSync==PWM_SYNC_ENABLE)
    {
        //DebugP_log("\nDeinit: SDFM_disableExternalReset ");
        SDFM_disableExternalReset(base, SDFM_FILTER_4);
        SDFM_disableExternalReset(base, SDFM_FILTER_3);
        SDFM_disableExternalReset(base, SDFM_FILTER_2);
        SDFM_disableExternalReset(base, SDFM_FILTER_1);
    }

    util_deinit_sdfm(base);

    if(testMode==TEST_MODE_CPUISRREAD)
    {
        //DebugP_log("\nDeinit: Intc_IntDisable ");
        //Intc_IntDisable(intnum);
        HwiP_destruct(&gSdfmHwiObject);
    }

    if(sdfm0to1Clock==1)
    {
        //DebugP_log("\nDeinit: SDFM1 clock from SDFM0");

        HW_WR_REG32(CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_SDFM1_CLK0_SEL ,0);
    }
    //Deinit PWMs
    util_deinit_epwms();




    //Return pass/fail
    if(error==0)
    {
        //DebugP_log("\nPass");

        return 0;
    }
    else
    {
        //DebugP_log("\nFail");

        return 1;
    }


}

int32_t sdfm_ex1_filter_sync_cpuread(uint32_t base)
{
    //DebugP_log("\n\nSDFM[%d] sdfm_ex1_filter_sync_cpuread", util_getsdfminstancefrombase(base));

    return util_sdfm_test(base, TEST_MODE_CPUISRREAD, PWM_SYNC_DISABLE, COMMONCLOCK_DISABLE, 128, FIFO_DISABLE, COMPARATOR_DISABLE, SDFM0TO1CLOCK_DISABLE);
}

int32_t sdfm_latency(uint32_t base) __attribute__((__section__(".intvecs")));

int32_t sdfm_latency(uint32_t base)
{
    /*R5F shall read 18 bytes of data from the SDFM interface into TCM or OCRAM in <450ns (ADC data x 3 phases x 3 axis = 18 bytes).
     * Single read is estimated to take 20 cycles or 50ns.*/

    int32_t error=0;

    //DebugP_log("\nsdfm_latency");


    util_inittimer();

    uint32_t t1=util_gettimertime();
    uint32_t t2=util_gettimertime();

    SDFMData.phase1_axis1_data = HW_RD_REG16(CSL_CONTROLSS_SDFM0_U_BASE + CSL_SDFM_SDDATFIFO1);
    SDFMData.phase1_axis2_data = HW_RD_REG16(CSL_CONTROLSS_SDFM0_U_BASE + CSL_SDFM_SDDATFIFO2);
    SDFMData.phase1_axis3_data = HW_RD_REG16(CSL_CONTROLSS_SDFM0_U_BASE + CSL_SDFM_SDDATFIFO3);

    SDFMData.phase2_axis1_data = HW_RD_REG16(CSL_CONTROLSS_SDFM0_U_BASE + CSL_SDFM_SDDATFIFO1);
    SDFMData.phase2_axis2_data = HW_RD_REG16(CSL_CONTROLSS_SDFM0_U_BASE + CSL_SDFM_SDDATFIFO2);
    SDFMData.phase2_axis3_data = HW_RD_REG16(CSL_CONTROLSS_SDFM0_U_BASE + CSL_SDFM_SDDATFIFO3);

    SDFMData.phase3_axis1_data = HW_RD_REG16(CSL_CONTROLSS_SDFM0_U_BASE + CSL_SDFM_SDDATFIFO1);
    SDFMData.phase3_axis2_data = HW_RD_REG16(CSL_CONTROLSS_SDFM0_U_BASE + CSL_SDFM_SDDATFIFO2);
    SDFMData.phase3_axis3_data = HW_RD_REG16(CSL_CONTROLSS_SDFM0_U_BASE + CSL_SDFM_SDDATFIFO3);

    //TBD: Analyze latency using waveforms.

    //Automate measurement
    uint32_t t3=util_gettimertime();
    //DebugP_log("\n9 samples, 18 bytes: t2=%u ns, t3=%u ns, diff=%u ns", t2, t3, t3-t2-(t2-t1));

    if((t3-t2-(t2-t1))>450)
    {
        error++;
    }

    if( ((t3-t2-(t2-t1))>(455+5)) || ((t3-t2-(t2-t1))<(455-5)) ) //455 in C2R3. Only the SDFM read latency. TCM write latency not included
    {
        //DebugP_log("\nBA33 latency changed wrt C2R3. Expected:%d, actual:%d", 455, (t3-t2-(t2-t1)));
    }



    util_inittimer();

    t1=util_gettimertime();
    t2=util_gettimertime();

    SDFMData_2.axis1_phase1_data  = HW_RD_REG16(CSL_CONTROLSS_SDFM0_U_BASE + CSL_SDFM_SDDATFIFO1);
    SDFMData_2.axis1_phase2_data  = HW_RD_REG16(CSL_CONTROLSS_SDFM0_U_BASE + CSL_SDFM_SDDATFIFO2);
    SDFMData_2.axis1_phase3_data  = HW_RD_REG16(CSL_CONTROLSS_SDFM0_U_BASE + CSL_SDFM_SDDATFIFO3);
    SDFMData_2.axis1_dc_bus_data  = HW_RD_REG16(CSL_CONTROLSS_SDFM0_U_BASE + CSL_SDFM_SDDATFIFO4);
    SDFMData_2.axis2_phase1_data  = HW_RD_REG16(CSL_CONTROLSS_SDFM1_U_BASE + CSL_SDFM_SDDATFIFO1);
    SDFMData_2.axis2_phase2_data  = HW_RD_REG16(CSL_CONTROLSS_SDFM1_U_BASE + CSL_SDFM_SDDATFIFO2);
    SDFMData_2.axis2_phase3_data  = HW_RD_REG16(CSL_CONTROLSS_SDFM1_U_BASE + CSL_SDFM_SDDATFIFO3);
    SDFMData_2.axis2_dc_bus_data  = HW_RD_REG16(CSL_CONTROLSS_SDFM1_U_BASE + CSL_SDFM_SDDATFIFO4);

    //TBD: Analyze latency using waveforms.

    //Automate measurement
    t3=util_gettimertime();
    //DebugP_log("\n8 samples, 16 bytes: t2=%u ns, t3=%u ns, diff=%u ns", t2, t3, t3-t2-(t2-t1));

    if( ((t3-t2-(t2-t1))>(415+5)) || ((t3-t2-(t2-t1))<(415-5)) )  //415 in C2R3. 415 is only the read latency. TCM write latency not included
    {
        //DebugP_log("\nBA33 latency changed wrt C2R3. Expected:%d, actual:%d", 415, (t3-t2-(t2-t1)));
    }

    //Return pass/fail
    if(error==0)
    {
        //DebugP_log("\nPass");

        return 0;
    }
    else
    {
        //DebugP_log("\nFail");

        return 1;
    }
}

int32_t AM263x_SDFM_BTR_0001(uint32_t base)
{
    //Configure SDFM input control unit for an SDFM channel
    //Configure the input control unit by configuring the modulator clock mode for SDFM channels. Select the channel clock source, enable/disable synchronizer for data or clock
    //Check SD modulator data-filter output values and check if it corresponds to analog input given to SD ADC.
    //"SDFM_setupModulatorClock
    //SDFM_enableSynchronizer
    //SDFM_disableSynchronizer
    //SDFM_selectClockSource
    //"

    //DebugP_log("\n\nSDFM[%d] AM263x_SDFM_BTR_0001", util_getsdfminstancefrombase(base));

    return util_sdfm_test(base, TEST_MODE_CPUISRREAD, PWM_SYNC_DISABLE, COMMONCLOCK_ENABLE, 128, FIFO_DISABLE, COMPARATOR_DISABLE, SDFM0TO1CLOCK_DISABLE);
}

int32_t AM263x_SDFM_BTR_0002(uint32_t base)
{
    //Configure manchester decoding mode.
    //Select clock source for manchestor decoding mode. Read mannchestor clock period.
    //Check SD modulator data-filter output values and check if it corresponds to analog input given to SD ADC.
    //"SDFM_selectManchesterClockSource
    //SDFM_setupModulatorClock
    //SDFM_getManchesterClockPeriod
    //SDFM_getManchesterDecoderLockedStatus
    //SDFM_getManchesterDecoderFailedStatus"

    int32_t error=0;

    //DebugP_log("\n\nSDFM[%d] AM263x_SDFM_BTR_0002", util_getsdfminstancefrombase(base));

    //No API for manchester decoding mode

    if(error==0)
    {
        //DebugP_log("\nPass");

        return 0;
    }
    else
    {
        //DebugP_log("\nFail");

        return 1;
    }
}
int32_t AM263x_SDFM_BTR_0003(uint32_t base)
{
    //Configure Data filter for each channel with PWM Sync disabled.
    //Configure oversampling ratio, filter structure, enable data ack flag, disable pwm sync & enable the data filter. Configure o/p data format & shift value for 16-bit data. Configure source of drint as ack flag & read the data register value in ISR.
    //Check SD modulator data-filter output values and check if it corresponds to analog input given to SD ADC.
    //"SDFM_setupModulatorClock
    //SDFM_setFilterType
    //SDFM_setFilterOverSamplingRatio
    //SDFM_setDataShiftValue
    //SDFM_setOutputDataFormat
    //SDFM_enableFilter
    //SDFM_configDataFilter"

    //DebugP_log("\n\nSDFM[%d] AM263x_SDFM_BTR_0003", util_getsdfminstancefrombase(base));

    return util_sdfm_test(base, TEST_MODE_CPUISRREAD, PWM_SYNC_DISABLE, COMMONCLOCK_DISABLE, 128, FIFO_DISABLE, COMPARATOR_DISABLE, SDFM0TO1CLOCK_DISABLE);
}

int32_t AM263x_SDFM_BTR_0004(uint32_t base)
{
    //Configure Data Filter with  FIFO enabled & PWM sync disabled.
    //Configure fifo level which can generate interrupt, select source of drint as fifo data ready interrupt, enable FIFO, enable FIFO interrupt. Read the data items present in FIFO, read FIFO data in ISR.
    //Check SD modulator data-filter output values and check if it corresponds to analog input given to SD ADC.
    //"SDFM_enableFIFOBuffer
    //SDFM_disableFIFOBuffer
    //SDFM_setFIFOInterruptLevel
    //SDFM_setDataReadyInterruptSource
    //SDFM_disableExternalReset
    //SDFM_getFIFOISRStatus
    //SDFM_getFIFOData
    //SDFM_getFIFODataCount"


    //DebugP_log("\n\nSDFM[%d] AM263x_SDFM_BTR_0004", util_getsdfminstancefrombase(base));

    return util_sdfm_test(base, TEST_MODE_CPUISRREAD, PWM_SYNC_DISABLE, COMMONCLOCK_ENABLE, 128, FIFO_ENABLE, COMPARATOR_DISABLE, SDFM0TO1CLOCK_DISABLE);
}

int32_t AM263x_SDFM_BTR_0005(uint32_t base)
{
    //Configure Data Filter with  FIFO enabled & PWM sync enabled.
    //"Configure fifo level which can generate interrupt, select source of drint as fifo data ready interrupt, enable FIFO, enable FIFO interrupt.
    //Select source of sync, enable/disable wait for sync event to write data to FIFO, enable/disable FIFO clear on sync event.
    //Read the data items present in FIFO, read FIFO data in ISR."
    //Check SD modulator data-filter output values and check if it corresponds to analog input given to SD ADC.
    //"SDFM_setPWMSyncSource
    //SDFM_setFIFOClearOnSyncMode
    //SDFM_setWaitForSyncClearMode
    //SDFM_enableWaitForSync
    //SDFM_enableExternalReset"


    //DebugP_log("\n\nSDFM[%d] AM263x_SDFM_BTR_0005", util_getsdfminstancefrombase(base));

    return util_sdfm_test(base, TEST_MODE_CPUISRREAD, PWM_SYNC_ENABLE, COMMONCLOCK_ENABLE, 128, FIFO_ENABLE, COMPARATOR_DISABLE, SDFM0TO1CLOCK_DISABLE);
}

int32_t AM263x_SDFM_BTR_0006(uint32_t base)
{
    //Configure Comparator Filter
    //Configure oversampling ratio, filter structure,  high/level threshold 1 & 2 values, z threshold value, enable the high/low interrupts. Select source event for CEVT1/CEVT2 events. Enable in CPU & see if interupt is reaching to CPU. Check if the flag is getting set for z threshold.
    //Check SD modulator data-filter output values and check if it corresponds to analog input given to SD ADC.
    //"SDFM_setComparatorFilterType
    //SDFM_setCompFilterOverSamplingRatio
    //SDFM_setCompFilterHighThreshold
    //SDFM_setCompFilterLowThreshold
    //SDFM_setCompFilterZeroCrossThreshold
    //SDFM_enableZeroCrossEdgeDetect
    //SDFM_configComparator
    //SDFM_selectCompEventSource
    //SDFM_selectCompEventSource
    //SDFM_enableInterrupt
    //SDFM_enableMainInterrupt
    //SDFM_getZeroCrossTripStatus
    //SDFM_clearZeroCrossTripStatus
    //SDFM_getComparatorSincData"


    //DebugP_log("\n\nSDFM[%d] AM263x_SDFM_BTR_0006", util_getsdfminstancefrombase(base));

    return util_sdfm_test(base, TEST_MODE_CPUISRREAD, PWM_SYNC_DISABLE, COMMONCLOCK_DISABLE, 128, FIFO_DISABLE, COMPARATOR_ENABLE, SDFM0TO1CLOCK_DISABLE);
}

int32_t AM263x_SDFM_BTR_0007(uint32_t base)
{
    //Configure event filters for high/low threshold events
    //Configure sample window size, threshold & clock prescale for high/low event filters for a channel. Initialize low/high event filter. Initialize high/low event filters.
    //Check SD modulator data-filter output values and check if it corresponds to analog input given to SD ADC.
    //"SDFM_configCompEventHighFilter
    //SDFM_configCompEventLowFilter
    //SDFM_selectCompEventLowSource
    //SDFM_selectCompEventHighSource"


    //DebugP_log("\n\nSDFM[%d] AM263x_SDFM_BTR_0007", util_getsdfminstancefrombase(base));

    return util_sdfm_test(base, TEST_MODE_CPUISRREAD, PWM_SYNC_DISABLE, COMMONCLOCK_DISABLE, 128, FIFO_DISABLE, COMPARATOR_AND_FILTER_ENABLE, SDFM0TO1CLOCK_DISABLE);

}


#define IN1 (CSL_SDFM_SDCOMP1CTL_CEVT1DIGFILTSEL_MASK|CSL_SDFM_SDCOMP1CTL_CEVT2DIGFILTSEL_MASK)
#define IN2 (CSL_SDFM_SDCOMP1EVT2FLTCTL_SAMPWIN_MASK|CSL_SDFM_SDCOMP1EVT2FLTCTL_THRESH_MASK) //|CSL_SDFM_SDCOMP1EVT2FLTCTL_FILINIT_MASK)
#define IN3 (CSL_SDFM_SDCOMP1EVT2FLTCLKCTL_CLKPRESCALE_MASK)
#define IN4 (CSL_SDFM_SDCOMP1EVT1FLTCTL_SAMPWIN_MASK|CSL_SDFM_SDCOMP1EVT1FLTCTL_THRESH_MASK) //|CSL_SDFM_SDCOMP1EVT1FLTCTL_FILINIT_MASK)
#define IN5 (CSL_SDFM_SDCOMP1EVT1FLTCLKCTL_CLKPRESCALE_MASK)

int32_t util_sdfmlock_write_n_check(uint32_t base, uint16_t comp1, uint16_t comp2, uint16_t comp3, uint16_t comp4, uint16_t comp5)
{
    uint32_t error=0;
    uint32_t filterindex=0;
    uint16_t readval;

    for(filterindex=0;filterindex<4;filterindex++)
    {
        HW_WR_REG16(base + CSL_SDFM_SDCOMP1CTL + 0x10*filterindex, IN1);
        readval = HW_RD_REG16(base + CSL_SDFM_SDCOMP1CTL + 0x10*filterindex);
        if(readval != comp1)
        {
            //DebugP_log("\nerror SDCOMP[%d]CTL", filterindex+1);
            //DebugP_log(" writeval=0x%04x, readval=0x%04x", IN1, readval);
            error++;
        }
        HW_WR_REG16(base + CSL_SDFM_SDCOMP1EVT2FLTCTL + 0x10*filterindex, IN2);
        readval = HW_RD_REG16(base + CSL_SDFM_SDCOMP1EVT2FLTCTL + 0x10*filterindex);
        if(readval != comp2)
        {
            //DebugP_log("\nerror SDCOMP[%d]EVT2FLTCTL", filterindex+1);
            //DebugP_log(" writeval=0x%04x, readval=0x%04x", IN2, readval);
            error++;
        }
        HW_WR_REG16(base + CSL_SDFM_SDCOMP1EVT2FLTCLKCTL + 0x10*filterindex, IN3);
        readval = HW_RD_REG16(base + CSL_SDFM_SDCOMP1EVT2FLTCLKCTL + 0x10*filterindex);
        if(readval != comp3)
        {
            //DebugP_log("\nerror SDCOMP[%d]EVT2FLTCLKCTL", filterindex+1);
            //DebugP_log(" writeval=0x%04x, readval=0x%04x", IN3, readval);
            error++;
        }
        HW_WR_REG16(base + CSL_SDFM_SDCOMP1EVT1FLTCTL + 0x10*filterindex, IN4);
        readval = HW_RD_REG16(base + CSL_SDFM_SDCOMP1EVT1FLTCTL + 0x10*filterindex);
        if(readval != comp4)
        {
            //DebugP_log("\nerror SDCOMP[%d]EVT1FLTCTL", filterindex+1);
            //DebugP_log(" writeval=0x%04x, readval=0x%04x", IN4, readval);
            error++;
        }
        HW_WR_REG16(base + CSL_SDFM_SDCOMP1EVT1FLTCLKCTL + 0x10*filterindex, IN5);
        readval = HW_RD_REG16(base + CSL_SDFM_SDCOMP1EVT1FLTCLKCTL + 0x10*filterindex);
        if(readval != comp5)
        {
            //DebugP_log("\nerror SDCOMP[%d]EVT1FLTCLKCTL", filterindex+1);
            //DebugP_log(" writeval=0x%04x, readval=0x%04x", IN5, readval);
            error++;
        }
    }

    return error;
}

int32_t util_sdfm_print_digitalfilter_regs(uint32_t base)
{
    uint32_t filterindex=0;

    for(filterindex=0;filterindex<4;filterindex++)
    {
        //DebugP_log("\n0x%04x", HW_RD_REG16(base + CSL_SDFM_SDCOMP1CTL + 0x10*filterindex));
        //DebugP_log(" 0x%04x", HW_RD_REG16(base + CSL_SDFM_SDCOMP1EVT2FLTCTL + 0x10*filterindex));
        //DebugP_log(" 0x%04x", HW_RD_REG16(base + CSL_SDFM_SDCOMP1EVT2FLTCLKCTL + 0x10*filterindex));
        //DebugP_log(" 0x%04x", HW_RD_REG16(base + CSL_SDFM_SDCOMP1EVT1FLTCTL + 0x10*filterindex));
        //DebugP_log(" 0x%04x", HW_RD_REG16(base + CSL_SDFM_SDCOMP1EVT1FLTCLKCTL + 0x10*filterindex));
        //DebugP_log(" 0x%04x", HW_RD_REG16(base + CSL_SDFM_SDCOMP1LOCK + 0x10*filterindex));
    }


    return 0;
}

int32_t AM263x_SDFM_BTR_0008(uint32_t base)
{
    //Lock access to event filters configuration registers for an SDFM channel
    //Lock access to event filters configuration registers for an SDFM channel and try updating the registers, it should not get updated.
    //Check SD modulator data-filter output values and check if it corresponds to analog input given to SD ADC.
    //SDFM_lockCompEventFilterConfig

    int32_t error=0;
    uint32_t filterindex=0;

    //DebugP_log("\n\nSDFM[%d] AM263x_SDFM_BTR_0008", util_getsdfminstancefrombase(base));

    //SDCOMP1/2/3/4CTL
    //SDCOMP1/2/3/4EVT2FLTCTL
    //SDCOMP1/2/3/4EVT2FLTCLKCTL
    //SDCOMP1/2/3/4EVT1FLTCTL
    //SDCOMP1/2/3/4EVT1FLTCLKCTL

    //DebugP_log("\n1. Positive test. Check if writable without lock");
    error+=util_sdfmlock_write_n_check(base, IN1, IN2, IN3, IN4, IN5);
    //DebugP_log("\nerror=%d", error);
    util_deinit_sdfm(base);

    //DebugP_log("\nAfter deinit");
    util_sdfm_print_digitalfilter_regs(base);


    //DebugP_log("\n2. Positive test. Lock. Check if writable. If yes, fail");
    SDFM_lockCompEventFilterConfig(base, SDFM_FILTER_1, CSL_SDFM_SDCOMP1LOCK_SDCOMP1CTL_MASK|CSL_SDFM_SDCOMP1LOCK_COMP_MASK);
    SDFM_lockCompEventFilterConfig(base, SDFM_FILTER_2, CSL_SDFM_SDCOMP1LOCK_SDCOMP1CTL_MASK|CSL_SDFM_SDCOMP1LOCK_COMP_MASK);
    SDFM_lockCompEventFilterConfig(base, SDFM_FILTER_3, CSL_SDFM_SDCOMP1LOCK_SDCOMP1CTL_MASK|CSL_SDFM_SDCOMP1LOCK_COMP_MASK);
    SDFM_lockCompEventFilterConfig(base, SDFM_FILTER_4, CSL_SDFM_SDCOMP1LOCK_SDCOMP1CTL_MASK|CSL_SDFM_SDCOMP1LOCK_COMP_MASK);
    error+=util_sdfmlock_write_n_check(base, CSL_SDFM_SDCOMP1CTL_RESETVAL, CSL_SDFM_SDCOMP1EVT2FLTCTL_RESETVAL, CSL_SDFM_SDCOMP1EVT2FLTCLKCTL_RESETVAL, CSL_SDFM_SDCOMP1EVT1FLTCTL_RESETVAL, CSL_SDFM_SDCOMP1EVT1FLTCLKCTL_RESETVAL);
    //DebugP_log("\nerror=%d", error);

    //DebugP_log("\nAfter test");
    util_sdfm_print_digitalfilter_regs(base);

    //DebugP_log("\n3. Negative test. Remove lock/Check if writable after previous lock. If yes, fail");
    for(filterindex=0;filterindex<4;filterindex++)
    {
        HW_WR_REG16(base + CSL_SDFM_SDCOMP1LOCK +  ((uint32_t)filterindex * SDFM_DIGFIL_OFFSET), 0);
    }
    error+=util_sdfmlock_write_n_check(base, CSL_SDFM_SDCOMP1CTL_RESETVAL, CSL_SDFM_SDCOMP1EVT2FLTCTL_RESETVAL, CSL_SDFM_SDCOMP1EVT2FLTCLKCTL_RESETVAL, CSL_SDFM_SDCOMP1EVT1FLTCTL_RESETVAL, CSL_SDFM_SDCOMP1EVT1FLTCLKCTL_RESETVAL);
    //DebugP_log("\nerror=%d", error);
    util_deinit_sdfm(base);

    //QUERY: Instance reset removes lock?

    if(error==0)
    {
        //DebugP_log("\nPass");

        return 0;
    }
    else
    {
        //DebugP_log("\nFail");

        return 1;
    }
}

int32_t AM263x_SDFM_BTR_0009(uint32_t base)
{
    //Configure Data Ack Event as trigger for data ready(DRINT) interrupt
    //Configure SDM type to 1, enable data ack interrupt, configure data ack as DRINT trigger. Check if DRINT is getting triggered on data ack.
    //Check SD modulator data-filter output values and check if it corresponds to analog input given to SD ADC.
    //"SysCtl_configureType
    //SDFM_setDataReadyInterruptSource
    //SDFM_getNewFilterDataStatus
    //SDFM_getFilterData"

    int32_t error=0;

    //DebugP_log("\n\nSDFM[%d] AM263x_SDFM_BTR_0009", util_getsdfminstancefrombase(base));

    //SDFM Type not configurable in AM263. Hardcoded to 1

    if(error==0)
    {
        //DebugP_log("\nPass");

        return 0;
    }
    else
    {
        //DebugP_log("\nFail");

        return 1;
    }
}

int32_t AM263x_SDFM_BTR_0010(uint32_t base)
{
    //Configure Data Ack Event as trigger for SDINT interrupt
    //Configure SDM type to 0, enable data ack interrupt, configure data ack as DRINT trigger. Check if SDINT is getting triggered on data ack.
    //Check SD modulator data-filter output values and check if it corresponds to analog input given to SD ADC.
    //"SysCtl_configureType
    //SDFM_setDataReadyInterruptSource
    //SDFM_getNewFilterDataStatus
    //SDFM_getFilterData"

    int32_t error=0;

    //DebugP_log("\n\nSDFM[%d] AM263x_SDFM_BTR_0010", util_getsdfminstancefrombase(base));

    //SDFM Type not configurable in AM263. Hardcoded to 1

    if(error==0)
    {
        //DebugP_log("\nPass");

        return 0;
    }
    else
    {
        //DebugP_log("\nFail");

        return 1;
    }
}


int32_t AM263x_SDFM_ITR_0001(uint32_t base)
{
    //PWM sync sources
    //Test additional PWM sync sources. Sync sources are programmed in SDSYNCx.SYNCSEL
    //Functional Pass/Fail

    //DebugP_log("\n\nSDFM[%d] AM263x_SDFM_ITR_0001", util_getsdfminstancefrombase(base));


    uint32_t error=0;

    for(g_pwm_num_sync=0;g_pwm_num_sync<32;g_pwm_num_sync++)
    {
        error+=util_sdfm_test(base, TEST_MODE_CPUISRREAD, PWM_SYNC_ENABLE, COMMONCLOCK_DISABLE, 128, FIFO_DISABLE, COMPARATOR_DISABLE, SDFM0TO1CLOCK_DISABLE);
    }
    g_pwm_num_sync=0;

    if(error>0)
    {
        return 1;
    }
    else
    {
        return 0;
    }

}

int32_t AM263x_SDFM_ITR_0002(uint32_t base)
{
    //SDFM1 clock source from SDFM0
    //Feed SDFM0 clock to SDFM1 and check filter functionality
    //Functional Pass/Fail

    //DebugP_log("\n\nSDFM[%d] AM263x_SDFM_ITR_0002", util_getsdfminstancefrombase(base));

    return util_sdfm_test(base, TEST_MODE_CPUISRREAD, PWM_SYNC_DISABLE, COMMONCLOCK_ENABLE, 128, FIFO_DISABLE, COMPARATOR_DISABLE, SDFM0TO1CLOCK_ENABLE);

}

int32_t AM263x_SDFM_ITR_0003(uint32_t base)
{
    //SDFM connectivity to DMA Xbar
    //Trigger EDMA using SDFM interrupt (DRINT). Test whether DMA copies data from SDFM interface into TCM or OCRAM
    //Functional Pass/Fail

    //DebugP_log("\n\nSDFM[%d] AM263x_SDFM_ITR_0003", util_getsdfminstancefrombase(base));

    uint32_t error=0;

    for(g_dmaxbar_num=0;g_dmaxbar_num<16;g_dmaxbar_num++)
    {
        for(g_curr_dma_evt=0;g_curr_dma_evt<4;g_curr_dma_evt++)
        {
            error+=util_sdfm_test(base, TEST_MODE_DMAREAD, PWM_SYNC_DISABLE, COMMONCLOCK_ENABLE, 128, FIFO_ENABLE, COMPARATOR_DISABLE, SDFM0TO1CLOCK_DISABLE);
        }
    }
    g_dmaxbar_num=0;
    g_curr_dma_evt=0;

    if(error>0)
    {
        return 1;
    }
    else
    {
        return 0;
    }

}

int32_t AM263x_SDFM_ITR_0004(uint32_t base)
{
    //SDFM connectivity to INT Xbar
    //Trigger R5F ISR using SDFM interrupt DRINT and ERR
    //Functional Pass/Fail

    //DebugP_log("\n\nSDFM[%d] AM263x_SDFM_ITR_0004", util_getsdfminstancefrombase(base));

    uint32_t error=0;

    for(g_intxbar_num=0;g_intxbar_num<32;g_intxbar_num++)
    {
        error+=util_sdfm_test(base, TEST_MODE_CPUISRREAD, PWM_SYNC_DISABLE, COMMONCLOCK_DISABLE, 128, FIFO_DISABLE, COMPARATOR_DISABLE, SDFM0TO1CLOCK_DISABLE);
    }
    g_intxbar_num=0;

    if(error>0)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

int32_t AM263x_SDFM_ITR_0005(uint32_t base)
{
    //SDFM connectivity to ECAP Mux
    //Configure comparator events and feed to ECAP Mux
    //SDFM events reach ECAP

    //DebugP_log("\n\nSDFM[%d] AM263x_SDFM_ITR_0005", util_getsdfminstancefrombase(base));

    uint32_t error=0;

    uint32_t i;
    for(i=0;i<10;i++)
    {
        g_ecapbase = CSL_CONTROLSS_ECAP0_U_BASE + (i*0x1000);
        error+=util_sdfm_test(base, TEST_MODE_ECAPMUX_OUT, PWM_SYNC_DISABLE, COMMONCLOCK_DISABLE, 128, FIFO_DISABLE, COMPARATOR_ENABLE, SDFM0TO1CLOCK_DISABLE);
    }
    g_ecapbase = CSL_CONTROLSS_ECAP0_U_BASE + (0*0x1000);

    if(error>0)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

int32_t AM263x_SDFM_ITR_0006(uint32_t base)
{
    //SDFM connectivity to PWM xbar
    //Configure comparator events and feed to PWM xbar
    //PWM xbar latches SDFM events

    //DebugP_log("\n\nSDFM[%d] AM263x_SDFM_ITR_0006", util_getsdfminstancefrombase(base));

    uint32_t error=0;

    for(g_xbarout=0;g_xbarout<30;g_xbarout++)
    {
        error+=util_sdfm_test(base, TEST_MODE_PWMXBAR_OUT, PWM_SYNC_DISABLE, COMMONCLOCK_DISABLE, 128, FIFO_DISABLE, COMPARATOR_ENABLE, SDFM0TO1CLOCK_DISABLE);
    }
    g_xbarout=0;

    if(error>0)
    {
        return 1;
    }
    else
    {
        return 0;
    }



}

int32_t AM263x_SDFM_ITR_0007(uint32_t base)
{
    //SDFM connectivity to output xbar
    //Configure comparator events and feed to output xbar
    //Output xbar latches SDFM events

    //DebugP_log("\n\nSDFM[%d] AM263x_SDFM_ITR_0007", util_getsdfminstancefrombase(base));

    uint32_t error=0;

    for(g_xbarout=0;g_xbarout<16;g_xbarout++)
    {
        error+=util_sdfm_test(base, TEST_MODE_OUTPUTXBAR_OUT, PWM_SYNC_DISABLE, COMMONCLOCK_DISABLE, 128, FIFO_DISABLE, COMPARATOR_ENABLE, SDFM0TO1CLOCK_DISABLE);
    }
    g_xbarout=0;

    if(error>0)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}


int32_t AM263x_SDFM_TTR_0001(uint32_t base)
{
    //R5 to SDFM access latency
    //Measure latency of R5F reading 18 bytes of data from SDFM interface into TCM or OCRAM
    //<450ns

    //DebugP_log("\n\nSDFM[%d] AM263x_SDFM_TTR_0001", util_getsdfminstancefrombase(base));

    return sdfm_latency(base);
}

int32_t util_sdfm_api_write_test(uint32_t base)
{
    uint32_t errorcount=0;

    util_deinit_sdfm(base);

    return errorcount;
}


int32_t test_sdfm_cases(uint8_t in)
{
    int32_t failcount=0;

    uint32_t sdfm_offset=0;
    uint32_t base;

    HW_WR_REG32(CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_LOCK0_KICK0 ,0x01234567);
    HW_WR_REG32(CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_LOCK0_KICK1 ,0xFEDCBA8);

    HW_WR_REG32(CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_EPWM_CLKSYNC, 0xFFFFFFFF);

    for(sdfm_offset=0;sdfm_offset<=0; sdfm_offset=sdfm_offset+0x1000)
    {

        base = CSL_CONTROLSS_SDFM0_U_BASE + sdfm_offset;

        failcount += util_sdfm_api_write_test(base);

        switch(in)
        {
            case 1:
                failcount += AM263x_SDFM_BTR_0001(base);
                break;
            case 2:
                failcount += AM263x_SDFM_BTR_0002(base);
                break;
            case 3:
                failcount += AM263x_SDFM_BTR_0003(base);
                break;
            case 4:
                failcount += AM263x_SDFM_BTR_0004(base);
                break;
            case 5:
                failcount += AM263x_SDFM_BTR_0005(base);
                break;
            case 6:
                failcount += AM263x_SDFM_BTR_0006(base);
                break;
            case 7:
                failcount += AM263x_SDFM_BTR_0007(base);
                break;
            case 8:
                failcount += AM263x_SDFM_BTR_0008(base);
                break;
            case 9:
                failcount += AM263x_SDFM_BTR_0009(base);
                break;
            case 10:
                failcount += AM263x_SDFM_BTR_0010(base);
                break;
            case 11:
                failcount += AM263x_SDFM_ITR_0001(base);
                break;
            case 12:
                failcount += AM263x_SDFM_ITR_0002(base);
                break;
            case 13:
                failcount += AM263x_SDFM_ITR_0003(base);
                break;
            case 14:
                failcount += AM263x_SDFM_ITR_0004(base);
                break;
            case 15:
                failcount += AM263x_SDFM_ITR_0005(base);
                break;
            case 16:
                failcount += AM263x_SDFM_ITR_0006(base);
                break;
            case 17:
                failcount += AM263x_SDFM_ITR_0007(base);
                break;
            case 18:
                failcount += AM263x_SDFM_TTR_0001(base);
                break;
        }
        //DebugP_log("\nfailcount=%d", failcount);
    }


    if(failcount!=0)
    {
        //DebugP_log("\nFAIL %d", failcount);
        return 1;
    }
    else
    {
        //DebugP_log("\nPass");
        return 0;
    }

}