/*
 *  Copyright (C) 2021-2024 Texas Instruments Incorporated
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
#include <drivers/adc.h>
#include <drivers/ecap.h>
#include <drivers/edma.h>
#include <drivers/epwm.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <../source/kernel/dpl/CycleCounterP.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* None */
#define TEST_ALL_INSTANCES false
/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* Testcases */
static void ADC_setPrescalerApiCheck(void *args);

static void ADC_single_ended_conversion(void *args);
static void ADC_differential_conversion_with_prescaled_clock(void *args);
static void ADC_software_triggering(void *args);
static void ADC_triggering_from_timer(void *args);
static void ADC_inputxbar_trigger(void *args);
static void ADC_epwm_soc_trigger(void *args);

static void ADC_ecap_soc_trigger(void *args);

static void ADC_all_adcs_working_together(void *args);
static void ADC_force_multiple_socs_simultaneously(void *args);
static void ADC_enable_ppb_adcevt_cycle_by_cycle_mode(void *args);
static void ADC_clear_adcintovf_status_register(void *args);
static void ADC_ppb_error_calculation(void *args);
static void ADC_trigger_to_sample_delay_capture(void *args);
static void ADC_asynchronous_operation(void *args);
static void ADC_back_to_back_conversions_after_receiving_trigger(void *args);
static void ADC_ppbs_to_trip_epwms(void *args);
static void ADC_periodic_triggering_of_adc_conversion_from_epwm(void *args);
static void ADC_adc_read_latency(void *args);
static void ADC_adc_latch_to_r5_response_latency(void *args);
static void ADC_r5_rw_latencies_to_adc(void *args);
static void ADC_dma_latency(void *args);

int32_t test_adc_cases(uint8_t in);

static HwiP_Object  gEpwmHwiObject1;
static HwiP_Object  gEpwmHwiObject2;

static uint8_t trigger_test = 0;

static HwiP_Object  gAdcHwiObject1;
static HwiP_Object  gAdcHwiObject2;

volatile uint32_t gISR_latency;
volatile uint32_t gISR_complete;

static int32_t      gAdc_ISR1_count = 0;
static int32_t      gEpwm_ISR2_count = 0;

static uint8_t channel_availability = 6;

static SemaphoreP_Object  gEpwmSyncSemObject;
static SemaphoreP_Object  gEpwmIsrSemObject;

volatile uint16_t adc_output_16 __attribute__((__section__(".controldata")));
volatile uint32_t adc_output_32 __attribute__((__section__(".controldata")));
uint32_t adc_output_32_write[4] __attribute__((__section__(".controldata")));

/* TA-DUT UART communincation related definitions, macros, variables */

#define CMD_TX_BUFSIZE      (200U)
#define CMD_RX_BUFSIZE      (200U)

#define SIZE_STRING_TEST_NAME   70
#define TIMEOUT_UART_MENU       (10000)
#define TIMEOUT_UART_TESTER     (10000)
#define TIMEOUT_UART_INFINITE   (SystemP_WAIT_FOREVER)

#define CMD_SIZE            (64U)
#define RSP_SIZE            (64U)
char test_command[CMD_SIZE];

uint8_t gCmdTxBuffer[CMD_TX_BUFSIZE];
uint8_t gCmdRxBuffer[CMD_RX_BUFSIZE];
volatile uint32_t gNumBytesRead = 0U, gNumBytesWritten = 0U;
int32_t          transferOK;
UART_Transaction trans;

bool enableLog;
bool manual_testing;


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
        DebugP_log("please enter 'y' after setting up required configuration for test\r\n");

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
        DebugP_log("%s\r\n", str);
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
    for(ind=0;ind<CMD_SIZE;ind++)
    {
        gCmdTxBuffer[ind]=0;
    }
}





/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* Common channels between Am263x LP E2, Am263x CC E2*/
/* NO_CHANNEL value is used to imply the unavailable channel */

#define MAX_CHANNEL_COUNT 6
#define NO_CHANNEL        99
#define CONFIG_EDMA_NUM_INSTANCES (1U)

#define RESULTS_BUFFER_SIZE     1024
/* Event queue to be used for EDMA transfer */
#define EDMA_TEST_EVT_QUEUE_NO  0U
/* DMA channel number to transfer ADC0 and ADC1 conversion results*/
#define ADC0_EDMA_CHANNEL       (DMA_TRIG_XBAR_EDMA_MODULE_0)

/* for CC E2*/
int ADC_channels_list[5][6] = {
    {NO_CHANNEL, NO_CHANNEL, ADC_CH_ADCIN2, ADC_CH_ADCIN3, NO_CHANNEL, NO_CHANNEL},  /* ADC 0*/
    {ADC_CH_ADCIN0, ADC_CH_ADCIN1, ADC_CH_ADCIN2, ADC_CH_ADCIN3, ADC_CH_ADCIN4, ADC_CH_ADCIN5},  /* ADC 1*/
    {ADC_CH_ADCIN0, ADC_CH_ADCIN1, ADC_CH_ADCIN2, ADC_CH_ADCIN3, NO_CHANNEL, NO_CHANNEL},  /* ADC 2*/
    {ADC_CH_ADCIN0, ADC_CH_ADCIN1, ADC_CH_ADCIN2, ADC_CH_ADCIN3, ADC_CH_ADCIN4, ADC_CH_ADCIN5},  /* ADC 3*/
    {NO_CHANNEL, NO_CHANNEL, NO_CHANNEL, NO_CHANNEL, NO_CHANNEL, NO_CHANNEL}                           /* ADC 4*/
};

#define CONFIG_EDMA0 (0U)
static EDMA_Object gEdmaObjects[CONFIG_EDMA_NUM_INSTANCES];

/* For AM263Px, these are defined within the syscfg for the PMIC configurations*/
#ifndef SOC_AM263PX

/* EDMA Driver handles */
EDMA_Handle gEdmaHandle[CONFIG_EDMA_NUM_INSTANCES];

/* EDMA Driver Open Parameters */
EDMA_Params gEdmaParams[CONFIG_EDMA_NUM_INSTANCES] =
{
    {
        .intrEnable = TRUE,
    },
};
#endif

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
extern EDMA_Config gEdmaConfig[];
extern uint32_t gEdmaConfigNum;



/* ========================================================================== */
/*                          Function Definitions                              */
#define MIN_SAMPLE_WINDOW (17)

void util_ADC_init(uint32_t base,
                   ADC_ClkPrescale  prescalar,
                   ADC_SignalMode   signal_mode,
                   ADC_PriorityMode adc_priority_mode);

void util_ADC_setup_and_enable_trigger_source(
    uint32_t        base,
    ADC_SOCNumber   soc_number,
    ADC_Trigger     trigger,
    ADC_IntNumber   int_number,
    ADC_Channel     channel,
    uint32_t        sampleWindow);

void util_ADC_fire_soc_trigger(
    uint32_t        base,
    ADC_SOCNumber   soc_number,
    ADC_Trigger     trigger);

int util_ADC_wait_for_adc_interrupt(uint32_t base, ADC_IntNumber int_number);

uint16_t util_ADC_check_result(uint32_t base, ADC_SOCNumber soc_number);

void util_ADC_deinit(uint32_t base);

int test_trigger(
    uint32_t        base,
    ADC_Trigger     trigger,
    ADC_Trigger     false_trigger,
    ADC_IntNumber   int_number,
    uint32_t        sample_window);

int test_adc_operational_mode(uint32_t base, int mode);

void enable_all_epwms(void);

void disable_all_epwms(void);

void epwms_interrupt_isr_setup(void);

void util_EPWM_init(uint32_t epwm_base);

void util_EPWM_deinit(uint32_t epwm_base);

void util_EPWM_setup_adc_trigger(uint32_t epwm_base, EPWM_ADCStartOfConversionType adc_soc_type);

void util_EPWM_start_counter(uint32_t epwm_instance);

void util_EPWM_stop_counter(uint32_t epwm_base);

void RTI_INT_ISR(void);

void EPWM_INT_ISR(void *args);

void ADC_INT_ISR_for_periodic_triggering_from_epwm(void *args);

void EPWM_INT_ISR_for_periodic_triggering_from_epwm(void *args);

void util_EDMA_Open(void);

void util_EDMA_Close(void);
/* ========================================================================== */



#define LENGTH_OF_TEST_LIST 21
#define LENGTH_OF_INPUT_INDEPENDENT_TEST_LIST   (6)
#define LENGTH_OF_INPUT_DEPENDENT_TEST_LIST     (16)



void test_main(void *args)
{
    /* Open drivers */
    Drivers_open();
    Board_driversOpen();

    gEdmaConfig[0].attrs = &gEdmaAttrs[CONFIG_EDMA0];
    gEdmaConfig[0].object =  &gEdmaObjects[CONFIG_EDMA0];

    gEdmaConfigNum = CONFIG_EDMA_NUM_INSTANCES;

    UNITY_BEGIN();

    tester_init();


    char input_independent_tests[LENGTH_OF_INPUT_INDEPENDENT_TEST_LIST+1][100]={
        "All independent and Quit",
        "ADC_setPrescalerApiCheck",                 // Input independent
        "ADC_trigger_to_sample_delay_capture",      // Input independent
        "ADC_adc_read_latency",                     // Input independent
        "ADC_adc_latch_to_r5_response_latency",     // Input independent
        "ADC_r5_rw_latencies_to_adc",               // Input independent
        "ADC_dma_latency"                          // Input independent

    };

    char input_dependent_tests[LENGTH_OF_INPUT_DEPENDENT_TEST_LIST+1][100] = {
        "ADC_single_ended_conversion",                          // Input Dependent
        "ADC_differential_conversion_with_prescaled_clock",     // Input Dependent
        "ADC_software_triggering",                              // Input Dependent
        "ADC_triggering_from_timer",                            // Input Dependent
        "ADC_inputxbar_trigger",                                // Input Dependent
        "ADC_epwm_soc_trigger",                                 // Input Dependent
        "ADC_ecap_soc_trigger",                                 // Input Dependent
        "ADC_all_adcs_working_together",                        // Input Dependent
        "ADC_force_multiple_socs_simultaneously",               // Input Dependent
        "ADC_enable_ppb_adcevt_cycle_by_cycle_mode",            // Input Dependent
        "ADC_clear_adcintovf_status_register",                  // Input Dependent
        "ADC_ppb_error_calculation",                            // Input Dependent
        "ADC_asynchronous_operation",                           // Input Dependent
        "ADC_back_to_back_conversions_after_receiving_trigger", // Input Dependent
        "ADC_ppbs_to_trip_epwms",                               // Input Dependent
        "ADC_periodic_triggering_of_adc_conversion_from_epwm",  // Input Dependent
        "All tests and Quit"
    };

    int while_breaker = 0;

    while(!while_breaker)
    {
        enableLog = 0;

        manual_testing = 0;

        DebugP_log("\r\n********** ADC Unit Test Menu **********");

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

        DebugP_log("\r\nInput Independent Tests");
        for( i = 0; i < LENGTH_OF_INPUT_INDEPENDENT_TEST_LIST+1; i++)
        {
            DebugP_log("\r\n\t%02d:%s",i, input_independent_tests[i]);
        }

        DebugP_log("\r\n\r\nInput Dependent Tests");
        for( i = 0; i < LENGTH_OF_INPUT_DEPENDENT_TEST_LIST+1; i++)
        {
            DebugP_log("\r\n\t%02d:%s",i+1+LENGTH_OF_INPUT_INDEPENDENT_TEST_LIST, input_dependent_tests[i]);
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
            option = LENGTH_OF_INPUT_INDEPENDENT_TEST_LIST+1 + LENGTH_OF_INPUT_DEPENDENT_TEST_LIST+1;     //""Run All tests and quit"
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

        switch (option)
        {
            case 0:
                DebugP_log("Running all the input-independent tests\r\n");

                RUN_TEST(ADC_setPrescalerApiCheck, 1, NULL);
                RUN_TEST(ADC_trigger_to_sample_delay_capture , 3166, NULL);
                RUN_TEST(ADC_adc_read_latency , 3171, NULL);
                RUN_TEST(ADC_adc_latch_to_r5_response_latency , 3172, NULL);
                RUN_TEST(ADC_r5_rw_latencies_to_adc , 3173, NULL);
                RUN_TEST(ADC_dma_latency , 3174, NULL);

                while_breaker = 1;
                break;
            case 1:
                RUN_TEST(ADC_setPrescalerApiCheck, 1, NULL);
                break;
            case 2:
                RUN_TEST(ADC_trigger_to_sample_delay_capture , 3166, NULL);
                break;
            case 3:
                RUN_TEST(ADC_adc_read_latency , 3171, NULL);
                break;
            case 4:
                RUN_TEST(ADC_adc_latch_to_r5_response_latency , 3172, NULL);
                break;
            case 5:
                RUN_TEST(ADC_r5_rw_latencies_to_adc , 3173, NULL);
                break;
            case 6:
                RUN_TEST(ADC_dma_latency , 3174, NULL);
                break;
            case 7:
                RUN_TEST(ADC_single_ended_conversion, 3155, NULL);
                break;
            case 8:
                RUN_TEST(ADC_differential_conversion_with_prescaled_clock, 3156, NULL);
                break;
            case 9:
                RUN_TEST(ADC_software_triggering, 3157, NULL);
                break;
            case 10:
                RUN_TEST(ADC_triggering_from_timer , 3158, NULL);
                break;
            case 11:
                RUN_TEST(ADC_inputxbar_trigger , 3159, NULL);
                break;
            case 12:
                RUN_TEST(ADC_epwm_soc_trigger , 3160, NULL);
                break;
            case 13:
                RUN_TEST(ADC_ecap_soc_trigger, 7910,NULL);
                break;
            case 14:
                RUN_TEST(ADC_all_adcs_working_together , 3161, NULL);
                break;
            case 15:
                RUN_TEST(ADC_force_multiple_socs_simultaneously , 3162, NULL);
                break;
            case 16:
                RUN_TEST(ADC_enable_ppb_adcevt_cycle_by_cycle_mode , 3163, NULL);
                break;
            case 17:
                RUN_TEST(ADC_clear_adcintovf_status_register , 3164, NULL);
                break;
            case 18:
                RUN_TEST(ADC_ppb_error_calculation , 3165, NULL);
                break;
            case 19:
                RUN_TEST(ADC_asynchronous_operation , 3167, NULL);
                break;
            case 20:
                RUN_TEST(ADC_back_to_back_conversions_after_receiving_trigger , 3168, NULL);
                break;
            case 21:
                RUN_TEST(ADC_ppbs_to_trip_epwms , 3169, NULL);
                break;
            case 22:
                RUN_TEST(ADC_periodic_triggering_of_adc_conversion_from_epwm , 3170, NULL);
                break;

            case 23 :
                DebugP_log("Running All tests and ending.\r\n");

                RUN_TEST(ADC_setPrescalerApiCheck, 1, NULL);
                RUN_TEST(ADC_trigger_to_sample_delay_capture , 3166, NULL);
                RUN_TEST(ADC_adc_read_latency , 3171, NULL);
                RUN_TEST(ADC_adc_latch_to_r5_response_latency , 3172, NULL);
                RUN_TEST(ADC_r5_rw_latencies_to_adc , 3173, NULL);
                RUN_TEST(ADC_dma_latency , 3174, NULL);
                RUN_TEST(ADC_single_ended_conversion, 3155, NULL);
                RUN_TEST(ADC_differential_conversion_with_prescaled_clock, 3156, NULL);
                RUN_TEST(ADC_software_triggering, 3157, NULL);
                RUN_TEST(ADC_triggering_from_timer , 3158, NULL);
                RUN_TEST(ADC_inputxbar_trigger , 3159, NULL);
                RUN_TEST(ADC_epwm_soc_trigger , 3160, NULL);
                RUN_TEST(ADC_ecap_soc_trigger , 7910, NULL);
                RUN_TEST(ADC_all_adcs_working_together , 3161, NULL);
                RUN_TEST(ADC_force_multiple_socs_simultaneously , 3162, NULL);
                RUN_TEST(ADC_enable_ppb_adcevt_cycle_by_cycle_mode , 3163, NULL);
                RUN_TEST(ADC_clear_adcintovf_status_register , 3164, NULL);
                RUN_TEST(ADC_ppb_error_calculation , 3165, NULL);
                RUN_TEST(ADC_asynchronous_operation , 3167, NULL);
                RUN_TEST(ADC_back_to_back_conversions_after_receiving_trigger , 3168, NULL);
                RUN_TEST(ADC_ppbs_to_trip_epwms , 3169, NULL);
                RUN_TEST(ADC_periodic_triggering_of_adc_conversion_from_epwm , 3170, NULL);


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


/* Testcase 1 - Check the ADC_setPrescaler API */
static void ADC_setPrescalerApiCheck(void *args)
{
    /* Call the ADC_setPrescaler API */
    ADC_setPrescaler(CSL_CONTROLSS_ADC0_U_BASE, ADC_CLK_DIV_7_5);

    /* Check if the value was written correctly */
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG16(CSL_CONTROLSS_ADC0_U_BASE + CSL_ADC_ADCCTL2) & CSL_ADC_ADCCTL2_PRESCALE_MASK), ADC_CLK_DIV_7_5);

    /* Adding type 4 API checks for AM263Px */
#ifdef SOC_AM263PX

    ADC_enableAltDMATiming(CSL_CONTROLSS_ADC0_U_BASE);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG16(CSL_CONTROLSS_ADC0_U_BASE + CSL_ADC_ADCCTL1) & CSL_ADC_ADCCTL1_TDMAEN_MASK), CSL_ADC_ADCCTL1_TDMAEN_MASK);

    ADC_disableAltDMATiming(CSL_CONTROLSS_ADC0_U_BASE);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG16(CSL_CONTROLSS_ADC0_U_BASE + CSL_ADC_ADCCTL1) & CSL_ADC_ADCCTL1_TDMAEN_MASK), 0);

    ADC_enableExtMuxPreselect(CSL_CONTROLSS_ADC0_U_BASE);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG16(CSL_CONTROLSS_ADC0_U_BASE + CSL_ADC_ADCCTL1) & CSL_ADC_ADCCTL1_EXTMUXPRESELECTEN_MASK), CSL_ADC_ADCCTL1_EXTMUXPRESELECTEN_MASK);

    ADC_disableExtMuxPreselect(CSL_CONTROLSS_ADC0_U_BASE);
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG16(CSL_CONTROLSS_ADC0_U_BASE + CSL_ADC_ADCCTL1) & CSL_ADC_ADCCTL1_EXTMUXPRESELECTEN_MASK), 0);

    /* get API. need Funcitonal Check only.
     ADC_getIntResultStatus(uint32_t base, ADC_IntNumber adcIntNum) */

    for(ADC_PPBNumber ppbNumber = ADC_PPB_NUMBER1; ppbNumber <= ADC_PPB_NUMBER4; ppbNumber++ )
    {
        uint32_t ppbOffset = (ADC_ADCPPBxLIMIT_STEP * (uint32_t)ppbNumber) +
                CSL_ADC_ADCPPB1LIMIT;
        for(uint16_t limit = 0; limit <= CSL_ADC_ADCPPB1LIMIT_LIMIT_MAX; limit++)
        {
            ADC_setPPBCountLimit(CSL_CONTROLSS_ADC0_U_BASE, ppbNumber, limit);
            TEST_ASSERT_EQUAL_INT32(
                (HW_RD_REG16(CSL_CONTROLSS_ADC0_U_BASE + ppbOffset) & CSL_ADC_ADCPPB1LIMIT_LIMIT_MASK), limit);
        }

        /* ADC_readPPBPCount - get API. Need Functional test */
        /* ADC_readPPBPSum - get API. Need Functional test */
        /* ADC_readPPBPMax - get API. Need Functional test */
        /* ADC_readPPBPMin - get API. Need Functional test */
        /* ADC_readPPBPMaxIndex - get API. Need Functional test */
        /* ADC_readPPBPMinIndex - get API. Need Functional test */
        ppbOffset = (ADC_ADCPPBx_STEP * (uint32_t)ppbNumber) +
                    CSL_ADC_ADCPPB1CONFIG;
        ADC_enablePPBAbsoluteValue( CSL_CONTROLSS_ADC0_U_BASE, ppbNumber);
        TEST_ASSERT_EQUAL_INT32(
                (HW_RD_REG16(CSL_CONTROLSS_ADC0_U_BASE + ppbOffset) & CSL_ADC_ADCPPB1CONFIG_ABSEN_MASK), CSL_ADC_ADCPPB1CONFIG_ABSEN_MASK);

        ADC_disablePPBAbsoluteValue( CSL_CONTROLSS_ADC0_U_BASE, ppbNumber);
        TEST_ASSERT_EQUAL_INT32(
                (HW_RD_REG16(CSL_CONTROLSS_ADC0_U_BASE + ppbOffset) & CSL_ADC_ADCPPB1CONFIG_ABSEN_MASK), 0);

        ppbOffset = (ADC_ADCPPBxCONFIG2_STEP * (uint32_t)ppbNumber) +
                    CSL_ADC_ADCPPB1CONFIG2;

        for(uint16_t shiftVal = 0; shiftVal <= 10u; shiftVal++)
        {

            ADC_setPPBShiftValue(CSL_CONTROLSS_ADC0_U_BASE, ppbNumber, shiftVal);
            TEST_ASSERT_EQUAL_INT32(
                (HW_RD_REG16(CSL_CONTROLSS_ADC0_U_BASE + ppbOffset) & CSL_ADC_ADCPPB1CONFIG2_SHIFT_MASK), shiftVal << CSL_ADC_ADCPPB1CONFIG2_SHIFT_SHIFT);

        }

        for(uint16_t syncInput = ADC_SYNCIN_DISABLE; syncInput <= ADC_SYNCIN_CPSW_CTPS_SYNC; syncInput++ )
        {
            ADC_selectPPBSyncInput(CSL_CONTROLSS_ADC0_U_BASE, ppbNumber, syncInput);
            TEST_ASSERT_EQUAL_INT32(
                (HW_RD_REG16(CSL_CONTROLSS_ADC0_U_BASE + ppbOffset) & CSL_ADC_ADCPPB1CONFIG2_SYNCINSEL_MASK), syncInput << CSL_ADC_ADCPPB1CONFIG2_SYNCINSEL_SHIFT);

        }

        /* ADC_forcePPBSync A write only API. need functional test */

        for(uint16_t osIntSrc = ADC_PPB_OS_INT_1; osIntSrc <= ADC_PPB_OS_INT_2; osIntSrc++)
        {
            ADC_selectPPBOSINTSource(CSL_CONTROLSS_ADC0_U_BASE, ppbNumber, osIntSrc);
            TEST_ASSERT_EQUAL_INT32(
                (HW_RD_REG16(CSL_CONTROLSS_ADC0_U_BASE + ppbOffset) & CSL_ADC_ADCPPB1CONFIG2_OSINTSEL_MASK), osIntSrc << CSL_ADC_ADCPPB1CONFIG2_OSINTSEL_SHIFT);

        }

        for(uint16_t compSrc = ADC_PPB_COMPSOURCE_RESULT; compSrc <= ADC_PPB_COMPSOURCE_SUM; compSrc++)
        {
            ADC_selectPPBCompareSource(CSL_CONTROLSS_ADC0_U_BASE, ppbNumber, compSrc);
            TEST_ASSERT_EQUAL_INT32(
                (HW_RD_REG16(CSL_CONTROLSS_ADC0_U_BASE + ppbOffset) & CSL_ADC_ADCPPB1CONFIG2_COMPSEL_MASK), compSrc << CSL_ADC_ADCPPB1CONFIG2_COMPSEL_SHIFT);

        }

        ppbOffset = (ADC_PPBxTRIPLO_STEP * (uint32_t)ppbNumber) +
                  CSL_ADC_ADCPPB1TRIPLO;

        ADC_enablePPBExtendedLowLimit(CSL_CONTROLSS_ADC0_U_BASE, ppbNumber);
        TEST_ASSERT_EQUAL_INT32(
                (HW_RD_REG32(CSL_CONTROLSS_ADC0_U_BASE + ppbOffset) & CSL_ADC_ADCPPB1TRIPLO_LIMITLO2EN_MASK), CSL_ADC_ADCPPB1TRIPLO_LIMITLO2EN_MASK);

        ADC_disablePPBExtendedLowLimit(CSL_CONTROLSS_ADC0_U_BASE, ppbNumber);
        TEST_ASSERT_EQUAL_INT32(
                (HW_RD_REG32(CSL_CONTROLSS_ADC0_U_BASE + ppbOffset) & CSL_ADC_ADCPPB1TRIPLO_LIMITLO2EN_MASK), 0);

        for(ADC_SOCNumber socNumber = ADC_SOC_NUMBER0; socNumber <= ADC_SOC_NUMBER15; socNumber++)
        {
            for(ADC_SafetyCheckerInput scInput = ADC_SAFETY_CHECKER_INPUT_DISABLE; scInput <= ADC_SAFETY_CHECKER_INPUT_PPBSUMx; scInput++)
            {
                uint32_t socShift = ((uint32_t) socNumber) * 2U;
                ADC_configSOCSafetyCheckerInput(CSL_CONTROLSS_ADC0_U_BASE, socNumber, scInput);
                TEST_ASSERT_EQUAL_INT32(
                (HW_RD_REG32(CSL_CONTROLSS_ADC0_U_BASE + CSL_ADC_ADCSAFECHECKRESEN) & (CSL_ADC_ADCSAFECHECKRESEN_SOC0CHKEN_MASK << socShift)),
                    ((uint32_t) scInput << socShift));
            }
        }
        /* Safety Checker */
        ADC_enableSafetyChecker(CSL_CONTROLSS_ADCSAFE0_U_BASE);
        TEST_ASSERT_EQUAL_INT32(
                (HW_RD_REG16(CSL_CONTROLSS_ADCSAFE0_U_BASE + CSL_ADC_SAFETY_CHECKCONFIG) & CSL_ADC_SAFETY_CHECKCONFIG_CHKEN_MASK), CSL_ADC_SAFETY_CHECKCONFIG_CHKEN_MASK);
        ADC_disableSafetyChecker(CSL_CONTROLSS_ADCSAFE0_U_BASE);
        TEST_ASSERT_EQUAL_INT32(
                (HW_RD_REG16(CSL_CONTROLSS_ADCSAFE0_U_BASE + CSL_ADC_SAFETY_CHECKCONFIG) & CSL_ADC_SAFETY_CHECKCONFIG_CHKEN_MASK), 0);

        /* ADC_forceSafetyCheckerSync - Write only API. need Functional test */
        /* ADC_getSafetyCheckerStatus - get API. need Functional test */

        for(ADC_SafetyCheckInst checkInst = ADC_SAFETY_CHECK1; checkInst <= ADC_SAFETY_CHECK2; checkInst++)
        {
            for(ADC_Select adcInst = ADC_0; adcInst <= ADC_4; adcInst++)
            {
                for( ADC_ResultSelect adcResultInst = ADC_RESULT0; adcResultInst <= ADC_RESULT15; adcResultInst++)
                {

                    ADC_configureSafetyChecker(CSL_CONTROLSS_ADCSAFE0_U_BASE, checkInst, adcInst, adcResultInst);
                    TEST_ASSERT_EQUAL_INT32(
                        (HW_RD_REG16(CSL_CONTROLSS_ADCSAFE0_U_BASE + CSL_ADC_SAFETY_ADCRESSEL1+ ((uint16_t) checkInst)*4U) &
                        (CSL_ADC_SAFETY_ADCRESSEL1_ADCSEL_MASK| CSL_ADC_SAFETY_ADCRESSEL1_ADCRESULTSEL_MASK) ),
                        ((uint16_t)adcInst << CSL_ADC_SAFETY_ADCRESSEL1_ADCSEL_SHIFT) |
                        ((uint16_t)adcResultInst << CSL_ADC_SAFETY_ADCRESSEL1_ADCRESULTSEL_SHIFT));


                }
            }
        }


        for(uint32_t tolerance = 0; tolerance <= CSL_ADC_SAFETY_TOLERANCE_TOLERANCE_MAX; tolerance++)
        {
            ADC_setSafetyCheckerTolerance(CSL_CONTROLSS_ADCSAFE0_U_BASE, tolerance);

            TEST_ASSERT_EQUAL_INT32(
                (HW_RD_REG32(CSL_CONTROLSS_ADCSAFE0_U_BASE + CSL_ADC_SAFETY_TOLERANCE) & CSL_ADC_SAFETY_TOLERANCE_TOLERANCE_MASK), tolerance);
        }

        /* ADC_getSafetyCheckerResult - get API. need Functional test */

        for(ADC_SafetyCheckResult checkResult = ADC_SAFETY_CHECK_RES1OVF; checkResult <= ADC_SAFETY_CHECK_OOT; checkResult+=4)
        {
            for( ADC_Checker checkerNumber = ADC_SAFETY_CHECKER1; checkerNumber <= ADC_SAFETY_CHECKER12; checkerNumber++)
            {
                for( ADC_SafetyCheckEvent checkEvent = ADC_SAFETY_CHECK_EVENT1; checkEvent <= ADC_SAFETY_CHECK_EVENT4; checkEvent+=16)
                {
                    ADC_enableSafetyCheckEvt(CSL_CONTROLSS_ADCSAFE_EVENT_AGG_U_BASE, checkerNumber, checkEvent, checkResult);
                    uint32_t regOffset =  CSL_CONTROLSS_ADCSAFE_EVENT_AGG_U_BASE + CSL_ADC_SAFETY_AGGR_CHECKEVT1SEL1 + (uint32_t)checkEvent + (uint32_t)checkResult;
                    TEST_ASSERT_EQUAL_INT32(
                        (HW_RD_REG32(regOffset) & (1UL << (uint32_t)checkerNumber)), (1UL << (uint32_t)checkerNumber));

                    ADC_disableSafetyCheckEvt(CSL_CONTROLSS_ADCSAFE_EVENT_AGG_U_BASE, checkerNumber, checkEvent, checkResult);
                    TEST_ASSERT_EQUAL_INT32(
                        (HW_RD_REG32(regOffset) & (1UL << (uint32_t)checkerNumber)), 0);
                }

                ADC_enableSafetyCheckInt(CSL_CONTROLSS_ADCSAFE_EVENT_AGG_U_BASE, checkerNumber, checkResult);
                uint32_t regOffset_1 = CSL_CONTROLSS_ADCSAFE_EVENT_AGG_U_BASE + CSL_ADC_SAFETY_AGGR_CHECKINTSEL1 + (uint32_t)checkResult;
                TEST_ASSERT_EQUAL_INT32(
                        (HW_RD_REG32(regOffset_1) & (1UL << (uint32_t)checkerNumber)), (1UL << (uint32_t)checkerNumber));

                ADC_disableSafetyCheckInt(CSL_CONTROLSS_ADCSAFE_EVENT_AGG_U_BASE, checkerNumber, checkResult);
                TEST_ASSERT_EQUAL_INT32(
                        (HW_RD_REG32(regOffset_1) & (1UL << (uint32_t)checkerNumber)), 0);
            }
        }

        /* get, write only APIs. need functional tests
        ADC_getSafetyCheckStatus
        ADC_clearSafetyCheckStatus
        ADC_getSafetyCheckIntStatus
        ADC_clearSafetyCheckIntStatus */

        /* Trigger repeater */
        for(uint32_t repInstance = ADC_REPINST1; repInstance <= ADC_REPINST2; repInstance++)
        {
            for(ADC_RepMode mode = ADC_REPMODE_OVERSAMPLING; mode <= ADC_REPMODE_UNDERSAMPLING; mode++)
            {
                ADC_triggerRepeaterMode(CSL_CONTROLSS_ADC0_U_BASE, repInstance, mode);
                uint32_t regOffset = CSL_CONTROLSS_ADC0_U_BASE +  (repInstance * (ADC_REPxCTL_STEP));
                TEST_ASSERT_EQUAL_INT32(
                        (HW_RD_REG32(regOffset + CSL_ADC_REP1CTL) & CSL_ADC_REP1CTL_MODE_MASK), mode);
            }
            /*
            ADC_triggerRepeaterActiveMode - get API. need functional test
            ADC_triggerRepeaterModuleBusy - get API. need functional test
             */
            for(ADC_Trigger trigger = ADC_TRIGGER_SW_ONLY; trigger <= ADC_TRIGGER_RTI7; trigger++)
            {
                ADC_triggerRepeaterSelect(CSL_CONTROLSS_ADC0_U_BASE, repInstance, trigger);
                uint32_t regOffset = CSL_CONTROLSS_ADC0_U_BASE +  (repInstance * (ADC_REPxCTL_STEP));
                TEST_ASSERT_EQUAL_INT32(
                        (HW_RD_REG32(regOffset + CSL_ADC_REP1CTL) & CSL_ADC_REP1CTL_TRIGGER_MASK) >> CSL_ADC_REP1CTL_TRIGGER_SHIFT, (uint32_t)trigger);
            }

            for(ADC_SyncInput syncInput = ADC_SYNCIN_DISABLE; syncInput <= ADC_SYNCIN_CPSW_CTPS_SYNC; syncInput++)
            {
                ADC_triggerRepeaterSyncIn(CSL_CONTROLSS_ADC0_U_BASE, repInstance, syncInput);
                uint32_t regOffset = CSL_CONTROLSS_ADC0_U_BASE +  (repInstance * (ADC_REPxCTL_STEP));
                TEST_ASSERT_EQUAL_INT32(
                        (HW_RD_REG32(regOffset + CSL_ADC_REP1CTL) & CSL_ADC_REP1CTL_SYNCINSEL_MASK) >> CSL_ADC_REP1CTL_SYNCINSEL_SHIFT, (uint32_t)syncInput);
            }

            /*
            ADC_forceRepeaterTriggerSync - Write only API. needs functional test. */
            for(uint32_t repCount = 0; repCount <= CSL_ADC_REP1N_NSEL_MAX; repCount++)
            {
                ADC_triggerRepeaterCount(CSL_CONTROLSS_ADC0_U_BASE, repInstance, repCount);
                uint32_t regOffset = CSL_CONTROLSS_ADC0_U_BASE +  (repInstance * (ADC_REPxN_STEP));
                TEST_ASSERT_EQUAL_INT32(
                        (HW_RD_REG32(regOffset + CSL_ADC_REP1N) & CSL_ADC_REP1N_NSEL_MASK) >> CSL_ADC_REP1N_NSEL_SHIFT, (uint32_t)repCount);
            }

            for(uint32_t repPhase = 0; repPhase <= CSL_ADC_REP1PHASE_PHASE_MAX; repPhase++)
            {
                ADC_triggerRepeaterPhase(CSL_CONTROLSS_ADC0_U_BASE, repInstance, (uint16_t) repPhase);
                uint32_t regOffset = CSL_CONTROLSS_ADC0_U_BASE +  (repInstance * (ADC_REPxPHASE_STEP));
                TEST_ASSERT_EQUAL_INT32(
                        (HW_RD_REG32(regOffset + CSL_ADC_REP1PHASE) & CSL_ADC_REP1PHASE_PHASE_MASK) >> CSL_ADC_REP1PHASE_PHASE_SHIFT, (uint32_t)repPhase);
            }

            for(uint32_t repSpread = 0; repSpread <= CSL_ADC_REP1PHASE_PHASE_MAX; repSpread++)
            {
                ADC_triggerRepeaterSpread(CSL_CONTROLSS_ADC0_U_BASE, repInstance, (uint16_t) repSpread);
                uint32_t regOffset = CSL_CONTROLSS_ADC0_U_BASE +  (repInstance * (ADC_REPxSPREAD_STEP));
                TEST_ASSERT_EQUAL_INT32(
                        (HW_RD_REG32(regOffset + CSL_ADC_REP1SPREAD) & CSL_ADC_REP1SPREAD_SPREAD_MASK) >> CSL_ADC_REP1SPREAD_SPREAD_SHIFT, (uint32_t)repSpread);
            }

        }
    }

    /* SOC API r/w checks */
    for(uint32_t adcInstance = 0; adcInstance <= 6; adcInstance++)
    {   

        for(uint32_t channel = 0; channel <= 5; channel++)
        {
            uint32_t regOffset = CSL_TOP_CTRL_U_BASE;
            uint32_t channel_shift = (adcInstance <= 4)? channel : (channel + (adcInstance - 5)*4);
            // uint32_t mask = CSL_TOP_CTRL_ADC0_OSD_CHEN_ADC0_OSD_CHEN_CH_OSD_EN_MASK;
            if(adcInstance > 4)
            {
                regOffset += CSL_TOP_CTRL_ADCR01_OSD_CHEN;
                // mask = CSL_TOP_CTRL_ADCR01_OSD_CHEN_ADCR01_OSD_CHEN_CH_OSD_EN_MASK;
                if(channel > 3)
                {
                    continue;
                }
            }
            else
            {
                regOffset += CSL_TOP_CTRL_ADC0_OSD_CHEN + adcInstance*(CSL_TOP_CTRL_ADC1_OSD_CHEN - CSL_TOP_CTRL_ADC0_OSD_CHEN);
            }
            SOC_enableAdcOsdChannel(adcInstance, channel, TRUE);
            DebugP_log("%x %x %d %d\r\n", CSL_TOP_CTRL_U_BASE+CSL_TOP_CTRL_ADCR01_OSD_CHEN, regOffset, adcInstance, channel_shift);

            TEST_ASSERT_EQUAL_INT32(HW_RD_REG32(regOffset) & (1U<<channel_shift), (1U<<channel_shift)); 

            SOC_enableAdcOsdChannel(adcInstance, channel, FALSE);
            TEST_ASSERT_EQUAL_INT32(HW_RD_REG32(regOffset) & (1U<<channel_shift), 0); 

        }
        for(uint32_t config = 0; config <= 7; config++)
        {
            uint32_t regOffset = CSL_TOP_CTRL_U_BASE;
            uint32_t mask = CSL_TOP_CTRL_ADC0_OSD_CTRL_ADC0_OSD_CTRL_FUNCTION_MASK;
            if(adcInstance > 4)
            {
                regOffset += CSL_TOP_CTRL_ADCR01_OSD_CTRL;
                // mask = CSL_TOP_CTRL_ADCR01_OSD_CHEN_ADCR01_OSD_CHEN_CH_OSD_EN_MASK;
            }
            else
            {
                regOffset += CSL_TOP_CTRL_ADC0_OSD_CTRL + adcInstance*(CSL_TOP_CTRL_ADC1_OSD_CTRL - CSL_TOP_CTRL_ADC0_OSD_CTRL);
            }
            SOC_setAdcOsdConfig(adcInstance, config);
            TEST_ASSERT_EQUAL_INT32(HW_RD_REG32(regOffset) & mask, config); 
        }
        
        uint32_t regOffset = CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_ADCSOCFRCGBSEL;

        SOC_enableAdcGlobalForce(adcInstance, TRUE);
        TEST_ASSERT_EQUAL_INT32(HW_RD_REG32(regOffset) & CSL_CONTROLSS_CTRL_ADCSOCFRCGBSEL_ENABLE_MASK, (1U << adcInstance)); 
        SOC_enableAdcGlobalForce(adcInstance, FALSE);
        TEST_ASSERT_EQUAL_INT32(HW_RD_REG32(regOffset) & ~(1U << adcInstance), 0); 

    }
    
    /* SOC_adcSocGlobalForce(socNumber) write only. need functional test */
    for(uint32_t extChXbarOut = 0; extChXbarOut <= 9; extChXbarOut++)
    {
        for(uint32_t extChXbarIn = ADC0_EXTCHSEL_BIT0; extChXbarIn <= ADC_R2_EXTCHSEL_BIT1; extChXbarIn++)
        {
            uint32_t regOffset = CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_ADCEXTCHXBAR0_G0_SEL;
            regOffset += extChXbarOut * (CSL_CONTROLSS_CTRL_ADCEXTCHXBAR1_G0_SEL - CSL_CONTROLSS_CTRL_ADCEXTCHXBAR0_G0_SEL);
            SOC_selectAdcExtChXbar(extChXbarOut, extChXbarIn);
            TEST_ASSERT_EQUAL_INT32(HW_RD_REG32(regOffset) & CSL_CONTROLSS_CTRL_ADCEXTCHXBAR0_G0_SEL_SEL_MASK, extChXbarIn);
        }
    }

    uint32_t regOffset = CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_ADC_EXTCH_DLY_SEL;
    SOC_selextAdcExtChDelay(ADC_EXTCHSELCT_DELAY_3_CYCLES);
    TEST_ASSERT_EQUAL_INT32(HW_RD_REG32(regOffset) & CSL_CONTROLSS_CTRL_ADC_EXTCH_DLY_SEL_SEL_MASK, ADC_EXTCHSELCT_DELAY_3_CYCLES);
    SOC_selextAdcExtChDelay(ADC_EXTCHSELCT_DELAY_6_CYCLES);
    TEST_ASSERT_EQUAL_INT32(HW_RD_REG32(regOffset) & CSL_CONTROLSS_CTRL_ADC_EXTCH_DLY_SEL_SEL_MASK, ADC_EXTCHSELCT_DELAY_6_CYCLES);

#endif
}

static void ADC_single_ended_conversion(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_adc_cases(1), 0);
}
static void ADC_differential_conversion_with_prescaled_clock(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_adc_cases(2), 0);
}
static void ADC_software_triggering(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_adc_cases(3), 0);
}
static void ADC_triggering_from_timer(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_adc_cases(4), 0);
}
static void ADC_inputxbar_trigger(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_adc_cases(5), 0);
}
static void ADC_epwm_soc_trigger(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_adc_cases(6), 0);
}
static void ADC_ecap_soc_trigger(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_adc_cases(21), 0);
}
static void ADC_all_adcs_working_together(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_adc_cases(7), 0);
}
static void ADC_force_multiple_socs_simultaneously(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_adc_cases(8), 0);
}
static void ADC_enable_ppb_adcevt_cycle_by_cycle_mode(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_adc_cases(9), 0);
}
static void ADC_clear_adcintovf_status_register(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_adc_cases(10), 0);
}
static void ADC_ppb_error_calculation(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_adc_cases(11), 0);
}
static void ADC_trigger_to_sample_delay_capture(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_adc_cases(12), 0);
}
static void ADC_asynchronous_operation(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_adc_cases(13), 0);
}
static void ADC_back_to_back_conversions_after_receiving_trigger(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_adc_cases(14), 0);
}
static void ADC_ppbs_to_trip_epwms(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_adc_cases(15), 0);
}
static void ADC_periodic_triggering_of_adc_conversion_from_epwm(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_adc_cases(16), 0);
}
static void ADC_adc_read_latency(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_adc_cases(17), 0);
}
static void ADC_adc_latch_to_r5_response_latency(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_adc_cases(18), 0);
}
static void ADC_r5_rw_latencies_to_adc(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_adc_cases(19), 0);
}
static void ADC_dma_latency(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_adc_cases(20), 0);
}

void util_ADC_init(uint32_t base,
                   ADC_ClkPrescale prescalar,
                   ADC_SignalMode signal_mode,
                   ADC_PriorityMode adc_priority_mode)
{
    SOC_enableAdcReference((base & 0x0000F000)>>12);
    /*
     * this util function is meant to..
     * Set Prescalar to prescalar
     * Set Mode to signal_mode
     * Set interrupt Pulse Mode to interrupt_pulse_mode
     */
    ADC_setPrescaler(base, prescalar);
    ADC_setMode(base, ADC_RESOLUTION_12BIT, signal_mode);
    ADC_setInterruptPulseMode(base, ADC_PULSE_END_OF_CONV);
    ADC_setSOCPriority(base, adc_priority_mode);
    ADC_enableConverter(base);
    ClockP_usleep(500);
}

void util_ADC_deinit(uint32_t base)
{
    /*
     * this function reset ADC
     * and disables the ADC converter
     */

    /* resetting the ADC*/
    SOC_generateAdcReset((base & 0x0000F000)>>12);

    ADC_disableConverter(base);
}

void util_ADC_setup_and_enable_trigger_source(
    uint32_t        base,
    ADC_SOCNumber   soc_number,
    ADC_Trigger     trigger,
    ADC_IntNumber   int_number,
    ADC_Channel     channel,
    uint32_t        sample_window)
{
    ADC_disableInterrupt(base, int_number);
    if (trigger == ADC_TRIGGER_SW_ONLY)
    {
        /* No additional setup requried */
    }
    else if ((trigger >= ADC_TRIGGER_RTI0) && (trigger <= ADC_TRIGGER_RTI3))
    {
        /*
         * RTIx setup is configured through the syscfg.
         * No additional setup would be required
         *
         * Steps to setup in syscfg:
         *      Add RTI instances,
         *      enable counter 0/1,
         *      enable the compare event 0 and configure it for counter 0/1
         *      enable interrupt
         *      add the Interrupt ISR Callback
         */
    }
    else if (trigger == ADC_TRIGGER_INPUT_XBAR_OUT5)
    {
        /* GPIOx --> GPIOy loop back, GPIOy in INPUT_XBAR_OUT5 setup */
        /*
         * the required setup is done in the util_ADC_fire_soc_trigger
         * hence, no extra setup is required here.
         */
    }
    else if ((trigger >= ADC_TRIGGER_EPWM0_SOCA) && (trigger <= ADC_TRIGGER_EPWM31_SOCB))
    {
        /* setup the given EPWM */

        /*
         * setup given EPWM TB Counter in stop and freeze mode.
         * setup TB PRD to be largest (12000)
         * setup compare A to say 100
         * setup compare B to say 200
         *      fire trigger() shall change counter mode to up-down count mode.
         * set EPWM interrupt to be at counter == compare B
         * set EPWM INT to change the counter mode to stop and freeze.
         */

        uint8_t     epwm_instance   = (uint8_t) (trigger - 8)/2 ;
        uint32_t    epwm_base       = (0x00001000)*( (uint32_t) epwm_instance)
                                        + CSL_CONTROLSS_G0_EPWM0_U_BASE;

        util_EPWM_setup_adc_trigger(epwm_base, (trigger%2));


    }
    else if ((trigger >= ADC_TRIGGER_ECAP0_SOCEVT) && (trigger <= ADC_TRIGGER_ECAP9_SOCEVT))
    {
        /* Need to initialize and use ECAP module to trigger ADC, rather than forcing*/
        /* ECAP SOCEVT will be selected and it shall trigger the adc
           hence the no setup required. */
    }
    else
    {
        /* do nothing */
    }


    /* As per TRM, the sample_window (ACQPS) should be >= 16 sysclks*/
    sample_window = (sample_window > 17)? sample_window : 17;

    ADC_setupSOC(
        base,
        soc_number,
        trigger,
        channel,
        MIN_SAMPLE_WINDOW);

    /* enabling the interrupt*/
    ADC_enableInterrupt(base, int_number);
    /* Setting up the current soc_number (eoc) to trigger ADC_INT*/
    ADC_setInterruptSource(base, int_number, soc_number);

    if (enableLog)
    {
        DebugP_log("ADC base : 0x%x\tSOC number : %d\r\n", base, soc_number);
        DebugP_log("SOC trigger value : 0x%x\tChannel selected : %d\r\n",trigger, channel);
        DebugP_log("Interrupt %d is setup for SOC %d and is enabled\r\n", int_number, soc_number);
    }
}

void util_ADC_fire_soc_trigger(
    uint32_t        base,
    ADC_SOCNumber   soc_number,
    ADC_Trigger     trigger)
{

    if (trigger == ADC_TRIGGER_SW_ONLY)
    {
        ADC_forceSOC(base, soc_number);
    }
    else if ((trigger >= ADC_TRIGGER_RTI0) && (trigger <= ADC_TRIGGER_RTI3))
    {
        /* RTIx trigger is requried */
        /*
         * Enabling the counter should be able to trigger,
         * the interrupt will call its ISR, RTI_INT_ISR()
         *      this ISR will only disable counter, so that there wouldn't be
         *      repeated interrupts from the given RTIx.
         */
        switch (trigger)
        {
        case ADC_TRIGGER_RTI1:
            RTI_counterEnable(CONFIG_RTI1_BASE_ADDR, CONFIG_RTI1_COMP0_SRC);
            break;
        case ADC_TRIGGER_RTI2:
            RTI_counterEnable(CONFIG_RTI2_BASE_ADDR, CONFIG_RTI2_COMP0_SRC);
            break;
        case ADC_TRIGGER_RTI3:
            RTI_counterEnable(CONFIG_RTI3_BASE_ADDR, CONFIG_RTI3_COMP0_SRC);
            break;
        default:
            /* RTI0 is not available.
             * so by default do nothing.
             * This should be able to fail wait and should ultimately cause the
             * test to fail*/
            break;
        }
    }
    else if (trigger == ADC_TRIGGER_INPUT_XBAR_OUT5)
    {
        /* GPIOx --> GPIOy loop back, GPIOy in INPUT_XBAR_OUT5 setup */
        /* Using GPIO15(Input PIN) and GPIO16(Output PIN) as for loopback*/
        GPIO_setDirMode(INPUT_PIN_BASE_ADDR,
                        INPUT_PIN_PIN,
                        INPUT_PIN_DIR);

        /* while using the tester. */
        GPIO_setDirMode(OUTPUT_PIN_BASE_ADDR,
                        OUTPUT_PIN_PIN,
                        INPUT_PIN_DIR);

        sprintf(test_command,"%s","trigger on GPIO 24 for InputXbar[5] to trigger ADC");
        tester_command(test_command);
    }
    else if ((trigger >= ADC_TRIGGER_EPWM0_SOCA) &&
             (trigger <= ADC_TRIGGER_EPWM31_SOCB))
    {
        /* trigger the given EPWM */
        uint8_t epwm_instance   = (uint8_t) (trigger - 8)/2 ;
        uint32_t    epwm_base       = ((uint32_t) epwm_instance<<12) + CSL_CONTROLSS_G0_EPWM0_U_BASE;
        EPWM_clearADCTriggerFlag(epwm_base, (trigger%2));
        util_EPWM_start_counter (epwm_instance);

        SemaphoreP_pend(&gEpwmSyncSemObject, SystemP_WAIT_FOREVER);


        /* the counter starts running
         * generates the epwm_adc_soc_x trigger and then
         * generates the epwm interrupt, fires, ISR,
         * ISR clears the interrupt and stops the counter */
    }
    else if ((trigger >= ADC_TRIGGER_ECAP0_SOCEVT) && (trigger <= ADC_TRIGGER_ECAP9_SOCEVT))
    {
        /* Needs to setup proper ECAP EVT rather than forcing */
        int offset = 0x1000;
        int instance = trigger - 0x48;
        int ecap_base = CSL_CONTROLSS_ECAP0_U_BASE + instance*offset;

        ECAP_enableCaptureMode(ecap_base);
        ECAP_reArm(ecap_base);
        /* set ecap input as inputxbar 5 (this is already set up for the GPIO 24) */
        ECAP_selectECAPInput(ecap_base, ECAP_INPUT_INPUTXBAR5);
        ECAP_selectSocTriggerSource(ecap_base, ECAP_CAP_MODE_SOC_TRIGGER_SRC_CEVT1);

        ECAP_setEventPolarity(ecap_base, ECAP_EVENT_1, ECAP_EVNT_RISING_EDGE);

        /*toggling the GPIO*/
        GPIO_setDirMode(INPUT_PIN_BASE_ADDR,
                        INPUT_PIN_PIN,
                        INPUT_PIN_DIR);

        sprintf(test_command,"%s","trigger on GPIO 24 for InputXbar[5] to trigger ADC");
        tester_command(test_command);

        /* Disabling the trigger source*/
        ECAP_selectSocTriggerSource(ecap_base, ECAP_APWM_MODE_SOC_TRIGGER_SRC_DISABLED);
    }
    if(enableLog)
    {
        DebugP_log("trigger : 0x%x is triggered\r\n",trigger);
    }
}

/*
 * triggers the false trigger,
 * waits for the ADC INT to happen. wait should fail,
 * triggers the trigger,
 * waits for the ADC INT to happen, wait should succeed.
 * returns if there is error.
 */
int test_trigger(
    uint32_t        base,
    ADC_Trigger     trigger,
    ADC_Trigger     false_trigger,
    ADC_IntNumber   int_number,
    uint32_t        sample_window)
{
    trigger_test = 1;
    int     wait        = 0;
    int     errors      = 0;
    ADC_Channel     channel;
    ADC_SOCNumber   soc_number;

    util_ADC_init(
        base,
        ADC_CLK_DIV_4_0,
        ADC_MODE_DIFFERENTIAL,
        ADC_PRI_ALL_ROUND_ROBIN);

    channel = ADC_CH_ADCIN0;

    for (soc_number = ADC_SOC_NUMBER0;
        soc_number <= ADC_SOC_NUMBER15;
        soc_number++)
    {
        util_ADC_setup_and_enable_trigger_source(
            base,
            soc_number,
            trigger,
            int_number,
            channel,
            sample_window);

        /* Enabling and clearing the interrupt, overflow flags if any*/
        ADC_enableInterrupt(base, int_number);
        ADC_clearInterruptStatus(base, int_number);
        if (ADC_getInterruptOverflowStatus(base, int_number))
        {
            ADC_clearInterruptOverflowStatus(base, int_number);
        }

        /* firing false trigger*/
        /* Using the GPIO loopback for generating a InputXbar.out[5] trigger */
        util_ADC_fire_soc_trigger(
            base,
            soc_number,
            false_trigger);

        wait = util_ADC_wait_for_adc_interrupt(base, int_number);

        if (wait == 1)
        {
            /* wait successful. Conversion happened */
            /* false trigger triggered the SOC*/
            errors++;
            if(enableLog)
            {
                DebugP_log("ERROR : false trigger triggered the soc\r\n");
            }
        }
        wait = 0;

        /* clearing the interrupt, overflow flags if any*/
        ADC_clearInterruptStatus(base, int_number);
        if (ADC_getInterruptOverflowStatus(base, int_number))
        {
            ADC_clearInterruptOverflowStatus(base, int_number);
        }

        util_ADC_fire_soc_trigger(
            base,
            soc_number,
            trigger);

        if((trigger >= ADC_TRIGGER_EPWM0_SOCA) && (trigger <= ADC_TRIGGER_EPWM31_SOCB))
        {
            uint32_t epwm_instance = (trigger - 8)/2;
            uint32_t epwm_base = (epwm_instance * 0x00001000) +  CSL_CONTROLSS_G0_EPWM0_U_BASE;

            if(EPWM_getADCTriggerFlagStatus(epwm_base, (trigger%2)) != 1)
            {
                errors++;
                if(enableLog)
                {
                    DebugP_log("ERROR : no adc_soca/b flag\r\n");
                }
            }
            else
            {
                EPWM_clearADCTriggerFlag(epwm_base, (trigger%2));
            }
        }
        wait = util_ADC_wait_for_adc_interrupt(base, int_number);
        if (wait == 0)
        {
            /* wait unsuccessful. Conversion did not happen*/
            /* trigger did not trigger the SOC*/
            errors++;
            if(enableLog)
            {
                DebugP_log("ERROR : trigger %d did not trigger the SOC\r\n", trigger);
            }
        }
        wait = 0;
        if(enableLog)
        {
            volatile uint16_t result = util_ADC_check_result(base, soc_number);
            DebugP_log("\t\t\tresult : %d\r\n",result);
        }

    }

    if(enableLog)
    {
        DebugP_log("%d is errors for %d trigger\r\n", errors, trigger);
    }
    trigger_test = 0;

    return errors;
}

/*
 * Waits until the interrupt flag is set,
 * Clears the interrupt flag and clears interrupt overflow flag, if set.
 */
int util_ADC_wait_for_adc_interrupt(
    uint32_t        base,
    ADC_IntNumber   int_number)
{
    /* TODO : Need to include a better wait condition*/
    int count_value = 100000;

    while (ADC_getInterruptStatus(base, int_number) != 1)
    {
        if (count_value <= 0)
        {
            /* wait time out. unsuccessful*/
            if(enableLog)
            {
                DebugP_log("wait for adc interrupt time out\r\n");
            }
            return 0;
        }
        /* reducing a count and making a timeout*/
        count_value--;
    }
    ClockP_usleep(3);
    ADC_clearInterruptStatus(base, int_number);
    if (ADC_getInterruptOverflowStatus(base, int_number))
    {
        ADC_clearInterruptOverflowStatus(base, int_number);
    }
    if(enableLog)
    {
        DebugP_log("wait for adc interrupt successful. Clearing the ADC INT flag and ADC INT overflow flag\r\n");
    }
    return 1;
}

/* Reads the ADC result register for given SOC*/
uint16_t util_ADC_check_result(
    uint32_t        base,
    ADC_SOCNumber   soc_number)
{
    volatile uint32_t result_base = base - (uint32_t) 0x001c0000;
    return ADC_readResult(result_base, soc_number);
}

void enable_all_epwms(void)
{
    for(int i = 0; i<=31 ; i++)
    {
        SOC_setEpwmTbClk(i, TRUE);
        SOC_setEpwmGroup(i, 0);
    }
    if(enableLog)
    {
        DebugP_log("enabled TB CLK for all EPWM, set group as 0\r\n");
    }
}

void epwms_interrupt_isr_setup(void)
{
    /* this is to setup the interrupt xbar,
     * setup ISR for all the epwms. */

    SOC_xbarSelectInterruptXBarInputSource(
        CSL_CONTROLSS_INTXBAR_U_BASE,
        1,                     // instance number 1
        0xFFFFFFFF,            // group zero all signals i.e. all epwm ints.
        0, 0, 0, 0, 0, 0);     // no other group signals

    int32_t     status;

    /* Initialising a Interrupt parameter
     * setting up the interrupt, callbacks.*/
    static HwiP_Params     hwiPrms1;

    HwiP_Params_init(&hwiPrms1);
    hwiPrms1.intNum      = CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_1;
    hwiPrms1.callback    = &EPWM_INT_ISR;
    hwiPrms1.isPulse     = 1;
    status              = HwiP_construct(&gEpwmHwiObject1, &hwiPrms1);
    DebugP_assert(status == SystemP_SUCCESS);


    if(enableLog)
    {
        DebugP_log("INTXbar.out[1] is configured for all EPWM interrupts and EPWM_INT_ISR is setup\r\n");
    }
    /* sets up all the epwm interrupts to the given interrupt xbar instance*/
    /* here, INTXbar.out[1] is used for all the ewpm istances*/

}

void util_EPWM_init(uint32_t epwm_base)
{
/*
 * Initialized give epwm with TBPRD = 12000
 * counter = 0 and counter mode = stop and freeze
 * Compare A value = 100
 * Compare B value = 200
 * Action qualifer :
 *      to set high if counter = compare A counting up
 *      to set low if  counter = compare B counting up.
 * Event trigger   :
 *      EPWM interrupt to be triggered if the counter = compare A counting down
 *
 * the values choosen to configure a pulse and an interrupt, among many
 * combinations
 */
    uint16_t period;
    uint16_t compare_a;
    uint16_t compare_b;

    if(trigger_test)
    {
        period     = 10;
        compare_a  = 5;
        compare_b  = 5;
    }
    else
    {
        period     = 12000;
        compare_a  = 100;
        compare_b  = 200;
    }


    int epwm_instance = (epwm_base & 0x0001f000)>>12;
    if(enableLog)
    {
        DebugP_log("%x is epwm_base\r\n", epwm_base);
    }
    SOC_generateEpwmReset(epwm_instance);

    if(trigger_test)
    {
        EPWM_setClockPrescaler(
            epwm_base,
            EPWM_CLOCK_DIVIDER_1,
            EPWM_HSCLOCK_DIVIDER_1);
    }
    else
    {
        EPWM_setClockPrescaler(
            epwm_base,
            EPWM_CLOCK_DIVIDER_128,
            EPWM_HSCLOCK_DIVIDER_14);
    }

    /* counter is set to stop and freeze*/
    EPWM_setTimeBaseCounterMode( epwm_base, EPWM_COUNTER_MODE_STOP_FREEZE);

    /* setting up the time epwm_base period*/
    EPWM_setTimeBasePeriod( epwm_base, period);

    /* Default configurations */
    EPWM_disablePhaseShiftLoad(epwm_base);
    EPWM_setPhaseShift(epwm_base, 0U);
    EPWM_setTimeBaseCounter(epwm_base, 0);

    /* Setup shadow register load on ZERO*/
    EPWM_setCounterCompareShadowLoadMode(
        epwm_base,
        EPWM_COUNTER_COMPARE_A,
        EPWM_COMP_LOAD_ON_CNTR_ZERO);
    EPWM_setCounterCompareShadowLoadMode(
        epwm_base,
        EPWM_COUNTER_COMPARE_B,
        EPWM_COMP_LOAD_ON_CNTR_ZERO);

    /* Setting up Compare values */

    EPWM_setCounterCompareValue(
        epwm_base,
        EPWM_COUNTER_COMPARE_A,
        compare_a);

    EPWM_setCounterCompareValue(
        epwm_base,
        EPWM_COUNTER_COMPARE_B,
        compare_b);

    EPWM_setActionQualifierAction(
        epwm_base,
        EPWM_AQ_OUTPUT_A,
        EPWM_AQ_OUTPUT_HIGH,
        EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);

    EPWM_setActionQualifierAction(
        epwm_base,
        EPWM_AQ_OUTPUT_A,
        EPWM_AQ_OUTPUT_LOW,
        EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);

    EPWM_setActionQualifierAction(
        epwm_base,
        EPWM_AQ_OUTPUT_B,
        EPWM_AQ_OUTPUT_HIGH,
        EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);

    EPWM_setActionQualifierAction(
        epwm_base,
        EPWM_AQ_OUTPUT_B,
        EPWM_AQ_OUTPUT_LOW,
        EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);

    EPWM_enableInterrupt(epwm_base);
    EPWM_setInterruptEventCount(epwm_base, 1);

    EPWM_setInterruptSource(
        epwm_base,
        EPWM_INT_TBCTR_D_CMPA,
        EPWM_INT_TBCTR_D_CMPA);

    EPWM_clearEventTriggerInterruptFlag(epwm_base);

}

void util_EPWM_setup_adc_trigger(uint32_t epwm_base, EPWM_ADCStartOfConversionType adc_soc_type)
{
    /* enabling the ADC soc trigger from EPWM */
    EPWM_enableADCTrigger( epwm_base, adc_soc_type);

    /* ADC trigger is set at counter == compare A while incrementing*/
    EPWM_setADCTriggerSource(
        epwm_base,
        adc_soc_type,
        EPWM_SOC_TBCTR_U_CMPA,
        EPWM_SOC_TBCTR_U_CMPA);

    EPWM_setADCTriggerEventPrescale(epwm_base, adc_soc_type, 1);
	EPWM_enableADCTriggerEventCountInit(epwm_base, adc_soc_type);
	EPWM_setADCTriggerEventCountInitValue(epwm_base, adc_soc_type, 0);

    return;
}

void util_EPWM_deinit(uint32_t epwm_base)
{
    int epwm_instance   = (epwm_base & 0x0001f000)>>12;
    /*reset the EPWM*/
    SOC_generateEpwmReset(epwm_instance);

    /* disable interrupt if is still enabled*/
    EPWM_disableInterrupt(epwm_base);

    /* time base counter = 0 and mode = stop and freeze */
    EPWM_setTimeBaseCounterMode( epwm_base, EPWM_COUNTER_MODE_STOP_FREEZE);
    EPWM_setTimeBaseCounter(epwm_base, 0);
}

void util_EPWM_start_counter(uint32_t epwm_instance)
{
    uint32_t epwm_base = (epwm_instance*0x00001000) + CSL_CONTROLSS_G0_EPWM0_U_BASE;
    EPWM_setTimeBaseCounterMode(epwm_base, EPWM_COUNTER_MODE_UP_DOWN);
}

void util_EPWM_stop_counter(uint32_t epwm_base)
{
    EPWM_setTimeBaseCounterMode( epwm_base, EPWM_COUNTER_MODE_STOP_FREEZE);
}

void disable_all_epwms(void)
{
    for (int i = 0; i <= 31; i++)
    {
        SOC_setEpwmTbClk(i, FALSE);
    }
}



/* ISR for all the RTIx (x = 1-3)*/
void RTI_INT_ISR(void)
{
    /* Disabling all the counters whenever this ISR is called.*/
    RTI_counterDisable(CONFIG_RTI1_BASE_ADDR, CONFIG_RTI1_COMP0_SRC);
    RTI_counterDisable(CONFIG_RTI2_BASE_ADDR, CONFIG_RTI2_COMP0_SRC);
    RTI_counterDisable(CONFIG_RTI3_BASE_ADDR, CONFIG_RTI3_COMP0_SRC);
}

void EPWM_INT_ISR(void *args)
{
    /* stop counters of all the epwms*/
    int  i = 0;
    for (i = 0; i<=31 ; i++)
    {
        uint32_t epwm_base = CSL_CONTROLSS_G0_EPWM0_U_BASE + 0x1000*i;
        util_EPWM_stop_counter(epwm_base);
        EPWM_clearEventTriggerInterruptFlag(epwm_base);
    }
    SemaphoreP_post(&gEpwmSyncSemObject);
}

void ADC_INT_ISR_for_periodic_triggering_from_epwm(void *args)
{
    gAdc_ISR1_count++;
}

void EPWM_INT_ISR_for_periodic_triggering_from_epwm(void *args)
{
    if(gEpwm_ISR2_count >= 10)
    {
        SemaphoreP_post(&gEpwmIsrSemObject);
    }
    EPWM_clearEventTriggerInterruptFlag(CSL_CONTROLSS_G0_EPWM0_U_BASE);
}



void util_EDMA_Open(void)
{
    uint32_t instCnt;
    int32_t  status     = SystemP_SUCCESS;

    for(instCnt = 0U; instCnt < CONFIG_EDMA_NUM_INSTANCES; instCnt++)
    {
        gEdmaHandle[instCnt] = NULL;   /* Init to NULL so that we can exit gracefully */
    }

    /* Open all instances */
    for(instCnt = 0U; instCnt < CONFIG_EDMA_NUM_INSTANCES; instCnt++)
    {
        gEdmaHandle[instCnt] = EDMA_open(instCnt, &gEdmaParams[instCnt]);
        if(NULL == gEdmaHandle[instCnt])
        {
            DebugP_logError("EDMA open failed for instance %d !!!\r\n", instCnt);
            status = SystemP_FAILURE;
            break;
        }
    }

    if(SystemP_FAILURE == status)
    {
        util_EDMA_Close();   /* Exit gracefully */
    }

    return;
}


void util_EDMA_Close(void)
{
    uint32_t    instCnt;

    /* Close all instances that are open */
    for(instCnt = 0U; instCnt < CONFIG_EDMA_NUM_INSTANCES; instCnt++)
    {
        if(gEdmaHandle[instCnt] != NULL)
        {
            EDMA_close(gEdmaHandle[instCnt]);
            gEdmaHandle[instCnt] = NULL;
        }
    }

    return;
}

int test_adc_operational_mode(uint32_t base, int mode)
{
    int wait    = 0;
    int errors  = 0;
    int non_zero_values     = 0;
    int adc_instance        = 0;

    uint32_t        sample_window;
    volatile uint16_t        adc_result;
    ADC_SOCNumber   soc_number;
    ADC_Trigger     trigger;
    ADC_Channel     channel;
    ADC_IntNumber   int_number;


    int_number  =   ADC_INT_NUMBER1;
    trigger     =   ADC_TRIGGER_SW_ONLY;

    adc_instance = (base & 0x0000F000)>>12;

    /*
     * ADC_clock_cycle = (ADC_CLK_DIV_X_Y/2 +1)*input_clock_cycle
     * adding 1 at the end to make it >= actual adc_clock_cycle
     */
    sample_window = ((ADC_CLK_DIV_4_0 / 2 + 1) + 1);
    sample_window = (sample_window > MIN_SAMPLE_WINDOW)? sample_window : MIN_SAMPLE_WINDOW;

    if(mode == 0)
    {
        util_ADC_init(
            base,
            ADC_CLK_DIV_4_0,
            ADC_MODE_SINGLE_ENDED,
            ADC_PRI_ALL_ROUND_ROBIN);
    }
    else if(mode == 1)
    {
        util_ADC_init(
            base,
            ADC_CLK_DIV_4_0,
            ADC_MODE_DIFFERENTIAL,
            ADC_PRI_ALL_ROUND_ROBIN);
    }

    /* iteration over available channels */
    for (int channel_iter = 0; channel_iter < MAX_CHANNEL_COUNT; channel_iter++)
    {
        channel = ADC_channels_list[adc_instance][channel_iter];
        if (channel == NO_CHANNEL)
        {
            channel_availability--;
            continue;
        }

        float voltage;
        if((channel%2) == 0)
        {
            voltage = 2.000;
        }
        else
        {
            voltage = 1.000;
        }

        sprintf(test_command, "provide analog voltage of %.4fV on ADC %d Channel %d",voltage, adc_instance, channel);
        tester_command(test_command);   // even channels

        for (soc_number = ADC_SOC_NUMBER0;
            soc_number <= ADC_SOC_NUMBER15;
            soc_number++)
        {
            util_ADC_setup_and_enable_trigger_source(
                base,
                soc_number,
                trigger,
                int_number,
                channel,
                sample_window);

            /* Enabling and clearing the interrupt, overflow flags if any*/
            ADC_enableInterrupt(base, int_number);
            ADC_clearInterruptStatus(base, int_number);
            if (ADC_getInterruptOverflowStatus(base, int_number))
            {
                ADC_clearInterruptOverflowStatus(base, int_number);
            }

            util_ADC_fire_soc_trigger(
                base,
                soc_number,
                trigger);

            wait = util_ADC_wait_for_adc_interrupt(base, int_number);
            if (wait == 0)
            {
                /* wait successful. Conversion happened*/
                errors++;
            }

            adc_result = util_ADC_check_result(base, soc_number);

            if(enableLog)
            {
                DebugP_log("adc_instance : %d\tsoc_number : %d\tchannel : %d\tresult : %d\r\n",
                            adc_instance, soc_number, channel, adc_result);
            }

            if (adc_result > 0)
            {
                non_zero_values++;
            }
        }
    }
    ADC_disableInterrupt(base, int_number);

    if(channel_availability == 0)
    {
        /* reset channel_availability*/
        channel_availability = 6;
        /* No channels, so no conversion possible for channel result reading*/
        return 0;
        /* emulates a pass*/
    }

    if (non_zero_values < 15)
    {
        errors++;
    }

    if(enableLog)
    {
        DebugP_log("%d errors for base %x, %d non zero values\r\n", errors, base, non_zero_values);
    }

    /* resetting and disabling the ADC Converter*/
    util_ADC_deinit(base);

    return errors;
}

/*
 * AM263_ADC_BTR_0001	 Single ended conversion
 */
int32_t AM263x_ADC_BTR_001(uint32_t base)
{
    if(enableLog)
    {
        DebugP_log("\r\n\r\n------------------------------ base : %x -------------------------------------\r\n\r\n", base);
        DebugP_log("Test : Single ended conversion\r\n\r\n");
    }
    int errors = 0;

    /*expecting all to be non-zero values*/
    /*all the odd channels are connected a voltage divider
      arbitarily set at a positive voltage between 3.3 and 0*/

    /* passing 0 for single ended conversion*/

    errors = test_adc_operational_mode(base, 0);

    if (errors > 0)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/*
 * AM263_ADC_BTR_0002	 Double ended conversion
 */
int32_t AM263x_ADC_BTR_002(uint32_t base)
{
    if(enableLog)
    {
        DebugP_log("\r\n\r\n------------------------------ base : %x -------------------------------------\r\n\r\n", base);
        DebugP_log("Test : Double ended conversion\r\n\r\n");
    }
    int errors;

    /*expecting all to be non-zero values*/
    /*all the odd channels are connected a voltage divider
      arbitarily set at a positive voltage between 3.3 and 0*/
    /*even channels are not equal to odd channels */

    /* passing 1 for differential mode of conversion*/
    errors = test_adc_operational_mode(base, 1);
    if (errors > 0)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/*
 * AM263_ADC_BTR_0003	 Software triggering
 */
int32_t AM263x_ADC_BTR_003(uint32_t base)
{
    if(enableLog)
    {
        DebugP_log("\r\n\r\n------------------------------ base : %x -------------------------------------\r\n\r\n", base);
        DebugP_log("Test : Software triggering\r\n\r\n");
    }
    int errors = 0;

    ADC_Trigger trigger, false_trigger;
    uint32_t sample_window;

    ADC_IntNumber int_number;

    int_number      = ADC_INT_NUMBER1;
    trigger         = ADC_TRIGGER_SW_ONLY;
    false_trigger   = ADC_TRIGGER_INPUT_XBAR_OUT5;

    /*
     * ADC_clock_cycle = (ADC_CLK_DIV_X_Y/2 +1)*input_clock_cycle
     * adding 1 at the end to make it >= actual adc_clock_cycle
     */
    sample_window = ((ADC_CLK_DIV_4_0 / 2 + 1) + 1);
    sample_window = (sample_window > MIN_SAMPLE_WINDOW)? sample_window : MIN_SAMPLE_WINDOW;

    errors += test_trigger(
        base,
        trigger,
        false_trigger,
        int_number,
        sample_window);

    ADC_disableInterrupt(base, int_number);
    if(enableLog)
    {
        DebugP_log("%d errors for base %x\r\n", errors, base);
    }
    /* resetting and disabling the ADC Converter*/
    util_ADC_deinit(base);

    if (errors > 0)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/*
 * AM263_ADC_BTR_0004	 triggering from timer
 */
int32_t AM263x_ADC_BTR_004(uint32_t base)
{
    if(enableLog)
    {
        DebugP_log("\r\n\r\n------------------------------ base : %x -------------------------------------\r\n\r\n", base);
        DebugP_log("Test : triggering from timer\r\n\r\n");
    }

    int errors = 0;

    ADC_Trigger     trigger, false_trigger;
    ADC_IntNumber   int_number;
    uint32_t        sample_window;

    int_number      = ADC_INT_NUMBER1;
    false_trigger   = ADC_TRIGGER_INPUT_XBAR_OUT5;

    /*
     * ADC_clock_cycle = (ADC_CLK_DIV_X_Y/2 +1)*input_clock_cycle
     * adding 1 at the end to make it >= actual adc_clock_cycle
     */
    sample_window = ((ADC_CLK_DIV_4_0 / 2 + 1) + 1);
    sample_window = (sample_window > MIN_SAMPLE_WINDOW)? sample_window : MIN_SAMPLE_WINDOW;

    for (trigger = ADC_TRIGGER_RTI1;
         trigger <= ADC_TRIGGER_RTI3;
         trigger++)
    {
        /* loops through all the SOCs in the ADC*/
        errors += test_trigger(
            base,
            trigger,
            false_trigger,
            int_number,
            sample_window);
        ADC_disableInterrupt(base, int_number);
    }
    if(enableLog)
    {
        DebugP_log("%d errors for base %x\r\n", errors, base);
    }
    /* resetting and disabling the ADC Converter*/
    util_ADC_deinit(base);

    if(errors > 0)  /* Fail criteria */
    {
        return 1 ;
    }
    else            /* Pass Criteria*/
    {
        return 0;
    }
}

/*
 * AM263_ADC_BTR_0005	 InputXBar Trigger
 */
int32_t AM263x_ADC_BTR_005(uint32_t base)
{
    if(enableLog)
    {
        DebugP_log("\r\n\r\n------------------------------ base : %x -------------------------------------\r\n\r\n", base);
        DebugP_log("Test : InputXBar Trigger\r\n\r\n");
    }
    int errors = 0;

    uint32_t        sample_window;
    ADC_Trigger     trigger, false_trigger;
    ADC_IntNumber   int_number;

    int_number      = ADC_INT_NUMBER1;
    trigger         = ADC_TRIGGER_INPUT_XBAR_OUT5;
    false_trigger   = ADC_TRIGGER_RTI0;

    /*
     * ADC_clock_cycle = (ADC_CLK_DIV_X_Y/2 +1)*input_clock_cycle
     * adding 1 at the end to make it >= actual adc_clock_cycle
     */
    sample_window = ((ADC_CLK_DIV_4_0 / 2 + 1) + 1);
    sample_window = (sample_window > MIN_SAMPLE_WINDOW)? sample_window : MIN_SAMPLE_WINDOW;

    /* loops through all the SOCs in the ADC*/
    errors += test_trigger(
        base,
        trigger,
        false_trigger,
        int_number,
        sample_window);
    ADC_disableInterrupt(base, int_number);

    if(enableLog)
    {
        DebugP_log("%d errors for base %x\r\n", errors, base);
    }
    /* resetting and disabling the ADC Converter*/
    util_ADC_deinit(base);

    if (errors > 0)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/*
 * AM263_ADC_BTR_0006	 ePWM SOC Trigger
 */
int32_t AM263x_ADC_BTR_006(uint32_t base)
{

    int errors = 0;
    if(enableLog)
    {
        DebugP_log("\r\n\r\n------------------------------ base : %x -------------------------------------\r\n\r\n", base);
        DebugP_log("Test : ePWM SOC Trigger\r\n\r\n");
    }

    uint32_t        sample_window;
    ADC_Trigger     trigger, false_trigger;
    ADC_IntNumber   int_number;

    int_number      = ADC_INT_NUMBER1;
    false_trigger   = ADC_TRIGGER_INPUT_XBAR_OUT5;

    /*
     * ADC_clock_cycle = (ADC_CLK_DIV_X_Y/2 +1)*input_clock_cycle
     * adding 1 at the end to make it >= actual adc_clock_cycle
     */
    sample_window = ((ADC_CLK_DIV_4_0 / 2 + 1) + 1);
    sample_window = (sample_window > MIN_SAMPLE_WINDOW)? sample_window : MIN_SAMPLE_WINDOW;

    uint32_t status = SemaphoreP_constructBinary(&gEpwmSyncSemObject, 0);
    DebugP_assert(SystemP_SUCCESS == status);
    /* enabling all the epwms*/
    enable_all_epwms();

    /* setting up the isr for the epwms*/
    epwms_interrupt_isr_setup();

    for (trigger = ADC_TRIGGER_EPWM0_SOCA;
         trigger <= ADC_TRIGGER_EPWM15_SOCB;
         trigger++)
    {

        int epwm_instance = (trigger - 8)/2;
        uint32_t epwm_base = (epwm_instance*0x00001000) + CSL_CONTROLSS_G0_EPWM0_U_BASE;

        util_EPWM_init(epwm_base);

        /* loops through all the SOCs in the ADC*/
        errors += test_trigger(
            base,
            trigger,
            false_trigger,
            int_number,
            sample_window);

        if(errors > 0)
        {
            if(enableLog)
            {
                DebugP_log("trigger : %d complete. %d errors so far\r\n", trigger, errors);
            }
        }
        ADC_disableInterrupt(base, int_number);

        util_EPWM_deinit(epwm_base);
    }

    HwiP_destruct(&gEpwmHwiObject1);

    disable_all_epwms();
    /* resetting and disabling the ADC Converter*/
    util_ADC_deinit(base);

    if(errors > 0)  /* Fail criteria */
    {
        return 1 ;
    }
    else            /* Pass Criteria*/
    {
        return 0;
    }
}

/*
 * AM263_ADC_BTR_NEW    eCAP ADC SOC trigger
 */
uint32_t AM263_ADC_BTR_NEW(uint32_t base)
{
    if(enableLog)
    {
        DebugP_log("\r\n\r\n------------------------------ base : %x -------------------------------------\r\n\r\n", base);
        DebugP_log("Test : eCAP ADC SOC trigger\r\n\r\n");
    }
    int errors = 0;

    uint32_t        sample_window;
    ADC_Trigger     trigger, false_trigger;
    ADC_IntNumber   int_number;

    int_number      = ADC_INT_NUMBER1;
    false_trigger   = ADC_TRIGGER_RTI0;

    /*
     * ADC_clock_cycle = (ADC_CLK_DIV_X_Y/2 +1)*input_clock_cycle
     * adding 1 at the end to make it >= actual adc_clock_cycle
     */
    sample_window = ((ADC_CLK_DIV_4_0 / 2 + 1) + 1);
    sample_window = (sample_window > MIN_SAMPLE_WINDOW)? sample_window : MIN_SAMPLE_WINDOW;


    /* loops through all the SOCs in the ADC*/
    for(trigger = ADC_TRIGGER_ECAP0_SOCEVT;
        trigger <=ADC_TRIGGER_ECAP9_SOCEVT;
        trigger++ )
        {
            errors += test_trigger(
                base,
                trigger,
                false_trigger,
                int_number,
                sample_window);
        }

    ADC_disableInterrupt(base, int_number);

    if((errors > 0) && enableLog)
    {
        DebugP_log("%d errors for base %x\r\n", errors, base);
    }
    /* resetting and disabling the ADC Converter*/
    util_ADC_deinit(base);

    if (errors > 0)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/*
 * AM263_ADC_BTR_0007	 All ADCs working together
 */
int32_t AM263x_ADC_BTR_007( void )
{
    if(enableLog)
    {
        DebugP_log("Test :  All ADCs working together\r\n\r\n");
    }

    int errors  = 0;
    int wait    = 0;

    ADC_SOCNumber   soc_number;
    ADC_Trigger     trigger;
    ADC_Channel     channel;
    uint32_t        sample_window;
    uint32_t        base;

    uint32_t adc_offset = 0;
    channel         =   ADC_CH_ADCIN2;

    trigger         =   ADC_TRIGGER_RTI1;
    sample_window   =   ((ADC_CLK_DIV_4_0 / 2 + 1) + 1);
    sample_window   =   (sample_window > MIN_SAMPLE_WINDOW)? sample_window : MIN_SAMPLE_WINDOW;

    for (adc_offset = 0; adc_offset <= 0x4000; adc_offset = adc_offset + 0x1000)
    {

        base = CSL_CONTROLSS_ADC0_U_BASE + adc_offset;
        /*
         * Initialising each ADC in prescalar = ADC_CLK_DIV_4_0,
         * single ended mode and round robin mode
         */

        util_ADC_init(
            base,
            ADC_CLK_DIV_4_0,
            ADC_MODE_SINGLE_ENDED,
            ADC_PRI_ALL_ROUND_ROBIN
            );
        /* Disabling all the interrupts*/
        ADC_disableInterrupt(base, ADC_INT_NUMBER1);
        ADC_disableInterrupt(base, ADC_INT_NUMBER2);
        ADC_disableInterrupt(base, ADC_INT_NUMBER3);
        ADC_disableInterrupt(base, ADC_INT_NUMBER4);

        /* Enabling the interrupt*/
        ADC_enableInterrupt(base, ADC_INT_NUMBER1);
        ADC_disableContinuousMode(base, ADC_INT_NUMBER1);
        ADC_clearInterruptStatus(base, ADC_INT_NUMBER1);


        for (soc_number = ADC_SOC_NUMBER0;
            soc_number <= ADC_SOC_NUMBER15;
            soc_number ++)
        {
            /*
            * Setting up the SOCs to be triggered by the same trigger
            */

            ADC_setupSOC(
                base,
                soc_number,
                trigger,
                channel,
                sample_window
                );
            if(soc_number == ADC_SOC_NUMBER15)
            {
                /* for the last SOC in the round robin priority,
                    * adding the interrupt to be triggered at the end of
                    * conversion. this will be used for checking EOC*/
                ADC_setInterruptSource(base, ADC_INT_NUMBER1, soc_number);
            }
        }

        /* Clearing interrupts of all the ADCs*/
        ADC_clearInterruptStatus(base, ADC_INT_NUMBER1);
        ADC_clearInterruptOverflowStatus(base, ADC_INT_NUMBER1);
    }

    /*
     * All the SOCs of all the ADCs are setup for same trigger. Now, the trigger
     * has to be fired.
     *
     * RTI interrupt ISR shall disable the RTI counter. so there is only one
     * trigger occurred.
     */
    RTI_counterEnable(CONFIG_RTI1_BASE_ADDR, CONFIG_RTI1_COMP0_SRC);

    /*
     * Check for all the ADC_INT for all the ADCs.
     * this should result if All the ADCs-SOCs have been triggered, and
     * completed the conversions.
     */
    for (adc_offset = 0; adc_offset <= 0x4000; adc_offset = adc_offset + 0x1000)
    {
        base = CSL_CONTROLSS_ADC0_U_BASE + adc_offset;
        wait = util_ADC_wait_for_adc_interrupt(base, ADC_INT_NUMBER1);

        if(wait == 0)
        {
            /* wait unsuccessful. so error occurred*/
            errors++;
            if(enableLog)
            {
                DebugP_log("ERROR!! wait unsuccessful., base : %x\r\n", base);
            }
            wait = 0;
        }
    }

    for (adc_offset = 0; adc_offset <= 0x4000; adc_offset = adc_offset + 0x1000)
    {
        base = CSL_CONTROLSS_ADC0_U_BASE + adc_offset;

        /* resetting and disabling the ADC Converters*/
        util_ADC_deinit(base);
    }


    if(errors > 0)  /* Fail criteria */
    {
        return 1 ;
    }
    else            /* Pass Criteria*/
    {
        return 0;
    }
}

/*
 * AM263_ADC_BTR_0008	 Force multiple SOCs simultaneously
 */
int32_t AM263x_ADC_BTR_008(uint32_t base)
{
    if(enableLog)
    {
        DebugP_log("\r\n\r\n------------------------------ base : %x -------------------------------------\r\n\r\n", base);
        DebugP_log("Test :  Force multiple SOCs simultaneously\r\n\r\n");
    }
    int errors = 0;
    int wait   = 0;

    ADC_SOCNumber   soc_number;
    ADC_IntNumber   int_number;
    ADC_Trigger     trigger;
    ADC_Channel     channel;
    uint32_t        sample_window;

    int_number      =   ADC_INT_NUMBER1;
    trigger         =   ADC_TRIGGER_SW_ONLY;
    sample_window   =   ((ADC_CLK_DIV_4_0 / 2 + 1) + 1);
    sample_window   =   (sample_window > MIN_SAMPLE_WINDOW)? sample_window : MIN_SAMPLE_WINDOW;

    int adc_instance = (base & 0x0000f000)>>12;

    /*
     * multiple trigger will be used to select the bits corresponding to the
     * SOCs.
     * mask will be used to identify the last SOC in the round roubin.
     */

    int      max_value = 0;
    uint16_t multiple_trigger;
    uint16_t mask = 0x8000;

    util_ADC_init(
        base,
        ADC_CLK_DIV_4_0,
        ADC_MODE_SINGLE_ENDED,
        ADC_PRI_ALL_ROUND_ROBIN
        );

    ADC_disableInterrupt(base, int_number);
    for(int channel_iter = 0; channel_iter < MAX_CHANNEL_COUNT; channel_iter++)
    {
        channel = ADC_channels_list[adc_instance][channel_iter];
        if(channel == NO_CHANNEL)
        {
            continue;
        }
        for( multiple_trigger = 0xff00;
            multiple_trigger <= 0xffff;
            multiple_trigger++)
        {
            ADC_enableInterrupt(base, int_number);
            ADC_clearInterruptStatus(base, int_number);
            if (ADC_getInterruptOverflowStatus(base, int_number))
            {
                ADC_clearInterruptOverflowStatus(base, int_number);
            }

            mask =  0x8000;
            max_value = 0;
            /* setting up SOCs based on the bits set in the multiple trigger*/
            for(soc_number = 15; soc_number>= 0; soc_number--)
            {
                if((mask & multiple_trigger)!= 0)
                {
                    max_value =(max_value < soc_number)? soc_number : max_value;
                    ADC_setupSOC(
                        base,
                        soc_number,
                        trigger,
                        channel,
                        sample_window);
                }
                mask = mask >> 1;
                if(mask == 0)break;
            }

            /*
            * Setting up the last SOC in the selected multiple triggers
            * to trigger ADC_INT at its EOC.
            */
            soc_number = max_value;
            ADC_setInterruptSource(base, int_number, soc_number);

            /* Forcing multiple SOCs*/
            ADC_forceMultipleSOC(base, multiple_trigger);

            /*
            * waiting for ADC_INT generated at the end of last SOC in
            * the round robin
            */
            wait = util_ADC_wait_for_adc_interrupt(base, int_number);
            if(wait == 0)
            {
                /* wait unsuccessful. conversions didnt happen as intended.*/
                errors++;
                if(enableLog)
                {
                    DebugP_log("ERROR : wait unsuccessful. conversions didnt happen as intended\r\n");
                }
            }
            ADC_disableInterrupt(base, int_number);
            if(multiple_trigger == 0xffff) break;
        }
    }
    /* resetting and disabling the ADC Converter*/
    util_ADC_deinit(base);

    if(errors > 0)
    {
        if(enableLog)
        {
            DebugP_log("%d errors for base %x\r\n", errors, base);
        }
    }
    if(errors > 0)
    {
        /* Fail criteria */
        return 1 ;
    }
    else
    {
        /* Pass Criteria*/
        return 0;
    }

}

/*
 * AM263x_ADC_BTR_0009    Enable PPB ADCEVT cycle by cycle mode
 */
int32_t AM263x_ADC_BTR_009(uint32_t base)
{
    if(enableLog)
    {
        DebugP_log("\r\n\r\n------------------------------ base : %x -------------------------------------\r\n\r\n", base);
        DebugP_log("Test : Enable PPB ADCEVT cycle by cycle mode\r\n\r\n");
    }

    if(base == 0x502c4000)
    {
	    /* the ADC 4 has no channels available by default on the CC*/
	    /* Hence skipping with zero errors.*
	     */
	 return 0;

    }
    /* No need to be iterated over channels*/

    /*
     * PPB ADCEVT CBC clear mode functionality:
     *      the PPB ADCEVT flag will be cleared if there is no event and
     *      the next PPBxResult is updated.
     */

    int errors = 0;

    uint32_t        sample_window;
    ADC_IntNumber   int_number;
    ADC_Trigger     trigger;
    ADC_Channel     channel;

    ADC_SOCNumber   soc_number = ADC_SOC_NUMBER0;

    volatile uint32_t        result_base;

    result_base = base - (uint32_t)0x001c0000;

    channel         =   ADC_CH_ADCIN2;
    int_number      =   ADC_INT_NUMBER1;
    trigger         =   ADC_TRIGGER_SW_ONLY;
    sample_window   =   ((ADC_CLK_DIV_4_0 / 2 + 1) + 1);
    sample_window = (sample_window > MIN_SAMPLE_WINDOW)? sample_window : MIN_SAMPLE_WINDOW;
    float voltage;

    volatile uint16_t ppb_number;

    util_ADC_init(
        base,
        ADC_CLK_DIV_4_0,
        ADC_MODE_SINGLE_ENDED,
        ADC_PRI_ALL_ROUND_ROBIN
        );
    ADC_setupSOC(
        base,
        soc_number,
        trigger,
        channel,
        sample_window
        );

    ADC_enableInterrupt(base, int_number);
    ADC_setInterruptPulseMode(base, ADC_PULSE_END_OF_CONV);
    ADC_setInterruptSource(base, int_number, soc_number);
    ADC_disableContinuousMode(base, int_number);


    for(int iterations = 0 ; iterations <2; iterations++)
    {

        if(enableLog)
        {
            DebugP_log("iteration : %d\r\n", iterations);
        }
        for(ppb_number = ADC_PPB_NUMBER1;
            ppb_number <=ADC_PPB_NUMBER4;
            ppb_number++)
        {
            int adc_result;
            int ppb_result;
            if(enableLog)
            {
                DebugP_log("ppb_number : %d\t, base : %x\t errors so far : %d\r\n",ppb_number, base, errors);
            }

            ADC_setupPPB(base, ppb_number, soc_number);
            ADC_enablePPBEvent(base, ppb_number, ADC_EVT_TRIPHI);
            ADC_setPPBTripLimits(base, ppb_number, 3000, 1000);
            ADC_setPPBReferenceOffset(base, ppb_number, 0);

            ADC_clearPPBEventStatus(base, ppb_number, 0x7);
            /* setting the voltage to be between trip limits*/
            if(manual_testing)
            {
                DebugP_log("Please input voltage between trip values 2.34V and 0.78V");
            }

            voltage = 1.500;
            sprintf(test_command, "provide analog voltage of %.4fV on ADC %d Channel %d",voltage, 0, 0);
            tester_command(test_command);
            sprintf(test_command, "provide analog voltage of %.4fV on ADC %d Channel %d",voltage, 0, 1);
            tester_command(test_command);

            ADC_forceSOC(base, soc_number);
            if(util_ADC_wait_for_adc_interrupt(base, int_number) == false) errors++;
            ADC_clearInterruptStatus(base, int_number);
            adc_result = util_ADC_check_result(base, soc_number);
            ppb_result = ADC_readPPBResult(result_base, ppb_number);
            if(enableLog)
            {
                DebugP_log("adc_result : %x\tppb_result : %x\r\n", adc_result, ppb_result);
            }
            if(((ADC_getPPBEventStatus(base, ppb_number)) & (0x1)) == 1)
            {
                /* error : flag set*/
                errors++;
                if(enableLog)
                {
                    DebugP_log("1 flag set for ppb_number : %d\r\n",ppb_number);
                }
                ADC_clearPPBEventStatus(base, ppb_number, 0x7);
            }

            if(manual_testing)
            {
                DebugP_log("Please input voltage above trip hi value 2.34V");
            }

            voltage = 3.1000;
            sprintf(test_command, "provide analog voltage of %.4fV on ADC %d Channel %d",voltage, 0, 0);
            tester_command(test_command);
            sprintf(test_command, "provide analog voltage of %.4fV on ADC %d Channel %d",voltage, 0, 1);
            tester_command(test_command);


            ADC_forceSOC(base, soc_number);
            if(util_ADC_wait_for_adc_interrupt(base, int_number) == false) errors++;
            ADC_clearInterruptStatus(base, int_number);
            adc_result = ADC_readResult(result_base, soc_number);
            ppb_result = ADC_readPPBResult(result_base,ppb_number);
            if(enableLog)
            {
                DebugP_log("adc_result : %x\tppb_result : %x\r\n", adc_result, ppb_result);
            }
            if((ADC_getPPBEventStatus(base, ppb_number)& 0x0001) != 1)
            {
                /* error : flag not set*/
                errors++;
                if(enableLog)
                {
                    DebugP_log("2 flag not set for ppb_number : %d\r\n",ppb_number);
                }
            }

            /* setting the voltage to be between trip limits*/
            if(manual_testing)
            {
                DebugP_log("Please input voltage between trip values 2.34V and 0.78V");
            }

            voltage = 1.500;
            sprintf(test_command, "provide analog voltage of %.4fV on ADC %d Channel %d",voltage, 0, 0);
            tester_command(test_command);
            sprintf(test_command, "provide analog voltage of %.4fV on ADC %d Channel %d",voltage, 0, 1);
            tester_command(test_command);


            ADC_forceSOC(base, soc_number);
            if(util_ADC_wait_for_adc_interrupt(base, int_number) == false) errors++;
            ADC_clearInterruptStatus(base, int_number);
            adc_result = util_ADC_check_result(base, soc_number);
            ppb_result = ADC_readPPBResult(result_base,ppb_number);
            if(enableLog)
            {
                DebugP_log("adc_result : %x\tppb_result : %x\r\n", adc_result, ppb_result);
            }
            if((ADC_getPPBEventStatus(base, ppb_number)& 0x0001) != 1)
            {
                /* error : flag not set*/
                errors++;
                if(enableLog)
                {
                    DebugP_log("3 flag not set for ppb_number : %d\r\n",ppb_number);
                }
            }

            ADC_clearPPBEventStatus(base, ppb_number, ADC_EVT_TRIPHI);

            /* enabling PPB CBC evet clear */
            ADC_enablePPBEventCBCClear(base, ppb_number);

            /* setting the voltage to be between trip limits*/
            if(manual_testing)
            {
                DebugP_log("Please input voltage between trip values 2.34V and 0.78V");
            }

            voltage = 1.500;
            sprintf(test_command, "provide analog voltage of %.4fV on ADC %d Channel %d",voltage, 0, 0);
            tester_command(test_command);
            sprintf(test_command, "provide analog voltage of %.4fV on ADC %d Channel %d",voltage, 0, 1);
            tester_command(test_command);


            ADC_forceSOC(base, soc_number);
            if(util_ADC_wait_for_adc_interrupt(base, int_number) == false) errors++;
            ADC_clearInterruptStatus(base, int_number);
            adc_result = util_ADC_check_result(base, soc_number);
            ppb_result = ADC_readPPBResult(result_base,ppb_number);
            if(enableLog)
            {
                DebugP_log("adc_result : %x\tppb_result : %x\r\n", adc_result, ppb_result);
            }
            if((ADC_getPPBEventStatus(base, ppb_number)& 0x0001) == 1)
            {
                /* error : flag set*/
                errors++;
                if(enableLog)
                {
                    DebugP_log("4 flag set for ppb_number : %d\r\n",ppb_number);
                }
            }

            if(manual_testing)
            {
                DebugP_log("Please input voltage above trip hi value 2.34V");
            }

            voltage = 3.1000;
            sprintf(test_command, "provide analog voltage of %.4fV on ADC %d Channel %d",voltage, 0, 0);
            tester_command(test_command);
            sprintf(test_command, "provide analog voltage of %.4fV on ADC %d Channel %d",voltage, 0, 1);
            tester_command(test_command);

            ADC_forceSOC(base, soc_number);
            if(util_ADC_wait_for_adc_interrupt(base, int_number) == false) errors++;
            ADC_clearInterruptStatus(base, int_number);
            adc_result = util_ADC_check_result(base, soc_number);
            ppb_result = ADC_readPPBResult(result_base,ppb_number);
            if(enableLog)
            {
                DebugP_log("adc_result : %x\tppb_result : %x\r\n", adc_result, ppb_result);
            }
            if((ADC_getPPBEventStatus(base, ppb_number)& 0x0001) != 1)
            {
                /* error : flag not set*/
                errors++;
                if(enableLog)
                {
                    DebugP_log("5 flag not set for ppb_number : %d\r\n",ppb_number);
                }
            }

            /* setting the voltage to be between trip limits*/
            if(manual_testing)
            {
                DebugP_log("Please input voltage between trip values 2.34V and 0.78V");
            }


            voltage = 1.500;
            sprintf(test_command, "provide analog voltage of %.4fV on ADC %d Channel %d",voltage, 0, 0);
            tester_command(test_command);
            sprintf(test_command, "provide analog voltage of %.4fV on ADC %d Channel %d",voltage, 0, 1);
            tester_command(test_command);

            ADC_forceSOC(base, soc_number);
            if(util_ADC_wait_for_adc_interrupt(base, int_number) == false) errors++;
            ADC_clearInterruptStatus(base, int_number);
            adc_result = util_ADC_check_result(base, soc_number);
            ppb_result = ADC_readPPBResult(result_base,ppb_number);
            if(enableLog)
            {
                DebugP_log("adc_result : %x\tppb_result : %x\r\n", adc_result, ppb_result);
            }
            if((ADC_getPPBEventStatus(base, ppb_number)& 0x0001) == 1)
            {
                /* error : flag set*/
                errors++;
                if(enableLog)
                {
                    DebugP_log("6 flag set for ppb_number : %d\r\n",ppb_number);
                }
                ADC_clearPPBEventStatus(base, ppb_number, 0x7);
            }
            ADC_disablePPBEventCBCClear(base, ppb_number);
            ADC_clearPPBEventStatus(base, ppb_number, ADC_EVT_TRIPHI);
            ADC_disablePPBEvent(base, ppb_number, ADC_EVT_TRIPHI);
        }
    }
    util_ADC_deinit(base);

    if(errors > 0 )
    {
        /* fail criteria*/
        return 1;
    }
    else
    {
        /* Pass Criteria*/
        return 0;
    }

}


/*
 * AM263_ADC_BTR_0010	 clear ADCINTOVF status register
 */
int32_t AM263x_ADC_BTR_010(uint32_t base)
{
    if(enableLog)
    {
        DebugP_log("\r\n\r\n------------------------------ base : %x -------------------------------------\r\n\r\n", base);
        DebugP_log("Test : clear ADCINTOVF status register\r\n\r\n");
    }
    /* No need to iterate over channels*/
    int errors = 0;

    ADC_SOCNumber   soc_number = ADC_SOC_NUMBER0;
    ADC_IntNumber   int_number;
    ADC_Trigger     trigger;
    ADC_Channel     channel;
    uint32_t        sample_window;

    channel         =   ADC_CH_ADCIN2;
    trigger         =   ADC_TRIGGER_SW_ONLY;
    sample_window   =   ((ADC_CLK_DIV_4_0 / 2 + 1) + 1);
    sample_window   =   (sample_window > MIN_SAMPLE_WINDOW)? sample_window : MIN_SAMPLE_WINDOW;

    util_ADC_init(
        base,
        ADC_CLK_DIV_4_0,
        ADC_MODE_DIFFERENTIAL,
        ADC_PRI_ALL_ROUND_ROBIN);

    for (int_number = ADC_INT_NUMBER1;
         int_number <= ADC_INT_NUMBER4;
         int_number++)
    {
        /*for each interrupt checking the interrupt overflow status*/

        util_ADC_setup_and_enable_trigger_source(
            base,
            soc_number,
            trigger,
            int_number,
            channel,
            sample_window);
        /* clearing the interrupt status and overflow status*/
        ADC_clearInterruptStatus(base, int_number);
        ADC_clearInterruptOverflowStatus(base, int_number);

        /* trigger ADC conversion*/
        ADC_forceSOC(base, soc_number);
        int wait = util_ADC_wait_for_adc_interrupt(base, int_number);

        if(wait == 0 )
        {
            /* wait failed*/
            errors ++;
        }
        wait = 0;
        /* */
        if(ADC_getInterruptOverflowStatus(base, int_number) == 1)errors++;

        /* Not clearing the interrupt. forcing trigger again. this should
            * set the interrupt overflow flag */
        ADC_forceSOC(base, soc_number);
        wait = util_ADC_wait_for_adc_interrupt(base, int_number);

        if(wait == 0 )
        {
            /* wait failed*/
            errors ++;
        }
        wait = 0;
        /* Interrupt Overflow Occurred. now clear it*/
        ADC_clearInterruptOverflowStatus(base, int_number);
        if(ADC_getInterruptOverflowStatus(base, int_number) == 1) errors++;

        /* diabling interrupt source*/
        ADC_disableInterrupt(base, int_number);
    }
    util_ADC_deinit(base);

    if(errors > 0)
    {
        /* fail criteria*/
        return 1;
    }
    else
    {
        /* pass criteria*/
        return 0;
    }
}

/*
 * AM263_ADC_BTR_0011	 PPB error calculation
 */
int32_t AM263x_ADC_BTR_011(uint32_t base)
{
    if(enableLog)
    {
        DebugP_log("\r\n\r\n------------------------------ base : %x -------------------------------------\r\n\r\n", base);
        DebugP_log("Test : PPB error calculation\r\n\r\n");
    }
    int errors = 0;
    /* PPB referecne offset -->
                    PPB_result = ADC_result - reference offset*/
    /* PPB calibration offset -->
                    ADC_result = ADC_result - calibration offset*/

    /* By default PPBs are setup towards soc_number_0. this can interfere in
     * the test. so setting up the PPB towards the soc_number_1*/

    ADC_SOCNumber   soc_number      = ADC_SOC_NUMBER1;
    ADC_IntNumber   int_number      = ADC_INT_NUMBER1;
    ADC_Trigger     trigger         = ADC_TRIGGER_SW_ONLY;
    ADC_Channel     channel         = ADC_CH_ADCIN2;

    uint32_t        result_base = base - 0x001c0000;
    int             adc_instance = (base & 0x0000f000) >> 12;
    uint32_t        sample_window   = ((ADC_CLK_DIV_4_0 / 2 + 1) + 1);
    sample_window = (sample_window > MIN_SAMPLE_WINDOW)? sample_window : MIN_SAMPLE_WINDOW;

    float voltage;

    for (int channel_iter = 0; channel_iter < MAX_CHANNEL_COUNT; channel_iter++)
    {
        channel = ADC_channels_list[adc_instance][channel_iter];
        if (channel == NO_CHANNEL)
        {
            continue;
        }

        util_ADC_setup_and_enable_trigger_source(
                    base,
                    soc_number,
                    trigger,
                    int_number,
                    channel,
                    sample_window);

        ADC_PPBNumber   ppb_number;
        for(ppb_number  = ADC_PPB_NUMBER1;
            ppb_number <= ADC_PPB_NUMBER4;
            ppb_number++)
        {
            if(enableLog)
            {
                DebugP_log("---ppb_number : %d---\r\n",ppb_number);
            }
            ADC_setupPPB(base, ppb_number, soc_number);
            int16_t offset;

            voltage = 1.6500;
            sprintf(test_command, "provide analog voltage of %.4fV on ADC %d Channel %d",voltage, 0, 0);
            tester_command(test_command);
            sprintf(test_command, "provide analog voltage of %.4fV on ADC %d Channel %d",voltage, 0, 1);
            tester_command(test_command);

            for(offset = -512; offset <= 511; offset++)
            {
                uint16_t result_without_cal_offset = 0;
                int16_t result_without_ref_offset = 0;
                uint16_t result_with_cal_offset = 0;
                int16_t result_with_ref_offset = 0;


                for(int twos_complement_flag = 0;
                    twos_complement_flag <= 1;
                    twos_complement_flag++)
                {
                    if(twos_complement_flag == 0)
                    {
                        ADC_disablePPBTwosComplement(base, ppb_number);
                    }
                    else
                    {
                        ADC_enablePPBTwosComplement(base, ppb_number);
                    }
                    ADC_setPPBCalibrationOffset(base, ppb_number, 0);
                    ADC_setPPBReferenceOffset(base, ppb_number, 0);

                    ADC_forceSOC(base, soc_number);
                    if(util_ADC_wait_for_adc_interrupt(base, int_number) == false)
                    {
                        errors++;
                    }
                    ClockP_usleep(10);
                    result_without_cal_offset = util_ADC_check_result(base, soc_number);
                    result_without_ref_offset = ADC_readPPBResult(result_base, ppb_number);
                    (void)result_without_ref_offset; /* Presently set but not used. Suppress warning */

                    ADC_setPPBCalibrationOffset(base, ppb_number, offset);
                    ADC_setPPBReferenceOffset(base, ppb_number, offset);

                    ADC_forceSOC(base, soc_number);
                    if(util_ADC_wait_for_adc_interrupt(base, int_number) == false)
                    {
                        errors++;
                    }
                    ClockP_usleep(10);
                    result_with_cal_offset = util_ADC_check_result(base, soc_number);
                    result_with_ref_offset = ADC_readPPBResult(result_base, ppb_number);

                    if((int16_t) result_with_cal_offset != (int16_t)(result_without_cal_offset - offset))
                    {   if((result_with_cal_offset != 0) && (result_with_cal_offset != 0xffff))
                        {
                            errors++;
                            if(enableLog)
                            {
                                DebugP_log("ERROR!! cal offset fail for offset %d\r\n",offset);
                            }
                        }
                    }
                    if(twos_complement_flag == 0)
                    {
                        if((int16_t)result_with_ref_offset != (int16_t) (result_with_cal_offset - offset))
                        {
                            errors++;
                            if(enableLog)
                            {
                                DebugP_log("ERROR!! ref offset fail for offset without twos Complement %d\r\n",offset);
                            }
                        }
                    }
                    else
                    {
                        if((int16_t)result_with_ref_offset != (int16_t) (offset - result_with_cal_offset))
                        {
                            errors++;
                            if(enableLog)
                            {
                                DebugP_log("ERROR!! ref offset fail for offset with twos Complement %d\r\n",offset);
                            }

                        }
                    }
                }
            }
        }
    }

    util_ADC_deinit(base);

    if(errors > 0)
    {
        /* fail criteria */
        return 1;
    }
    else
    {
        /* pass criteria */
        return 0;
    }
}

/* AM263_ADC_BTR_0012	 Trigger to sample delay Capture*/
int32_t AM263x_ADC_BTR_012(uint32_t base)
{
    if(enableLog)
    {
        DebugP_log("\r\n\r\n------------------------------ base : %x -------------------------------------\r\n\r\n", base);
        DebugP_log("Test : Trigger to sample delay Capture\r\n\r\n");
    }

    int errors = 0;

    ADC_SOCNumber   soc_number;
    ADC_PPBNumber   ppb_number = ADC_PPB_NUMBER1;
    ADC_IntNumber   int_number = ADC_INT_NUMBER1;
    ADC_Trigger     trigger    = ADC_TRIGGER_INPUT_XBAR_OUT5;
    ADC_Channel     channel    = ADC_CH_ADCIN2;

    int ppb_1_delay_stamp = 0;
    int ppb_2_delay_stamp = 0;
    int ppb_3_delay_stamp = 0;
    int ppb_4_delay_stamp = 0;


    uint32_t        sample_window   =   ((ADC_CLK_DIV_4_0 / 2 + 1) + 1);
    sample_window = (sample_window > MIN_SAMPLE_WINDOW)? sample_window : MIN_SAMPLE_WINDOW;


    if(sample_window < MIN_SAMPLE_WINDOW)
    {
        sample_window = MIN_SAMPLE_WINDOW;
    }
    /* setting up all the SOC in roundrobin mode. enabling ADC instance*/
    util_ADC_init(
        base,
        ADC_CLK_DIV_4_0,
        ADC_MODE_SINGLE_ENDED,
        ADC_PRI_ALL_ROUND_ROBIN
        );

    /* enabling and clearing the ADC interrupt 1*/
    ADC_enableInterrupt(base, int_number);
    ADC_clearInterruptStatus(base, int_number);

    int_number = ADC_INT_NUMBER1;

    for(soc_number = ADC_SOC_NUMBER0;
        soc_number <=ADC_SOC_NUMBER15;
        soc_number++)
    {
        ADC_setupSOC(
            base,
            soc_number,
            trigger,
            channel,
            sample_window);

        /* setting soc_0, soc_5, soc_10, soc_15 to be configured
            * for PPBs  1 to 4 respectively */
        if((soc_number%5) == 0)
        {
            ADC_setupPPB(base, ppb_number, soc_number);
            ppb_number++;
        }
        if(soc_number == ADC_SOC_NUMBER15)
        {
            /* last soc to trigger interrupt*/
            ADC_setInterruptSource(base, int_number, soc_number);
        }
    }
    /* triggers all the SOCs*/
       GPIO_setDirMode(INPUT_PIN_BASE_ADDR,
                        INPUT_PIN_PIN,
                        INPUT_PIN_DIR);

    if(enableLog)
    {
        DebugP_log(" trigger for all SOCs is selected as INPUTxbar[5]\r\n");
    }
    util_ADC_fire_soc_trigger(base, soc_number, trigger);

    /* wait for last SOC to complete conversion and trigger ADC_interrupt*/
    if(util_ADC_wait_for_adc_interrupt(base, int_number) == false)
    {
        errors++;
    }
    ppb_1_delay_stamp = ADC_getPPBDelayTimeStamp(base, ADC_PPB_NUMBER1);
    ppb_2_delay_stamp = ADC_getPPBDelayTimeStamp(base, ADC_PPB_NUMBER2);
    ppb_3_delay_stamp = ADC_getPPBDelayTimeStamp(base, ADC_PPB_NUMBER3);
    ppb_4_delay_stamp = ADC_getPPBDelayTimeStamp(base, ADC_PPB_NUMBER4);

    if(enableLog)
    {
        DebugP_log("%d is ppb_1_delaystamp for soc0\r\n", ppb_1_delay_stamp );
    }
    if(enableLog)
    {
        DebugP_log("%d is ppb_2_delaystamp for soc5\r\n", ppb_2_delay_stamp );
    }
    if(enableLog)
    {
        DebugP_log("%d is ppb_3_delaystamp for soc10\r\n", ppb_3_delay_stamp);
    }
    if(enableLog)
    {
        DebugP_log("%d is ppb_4_delaystamp for soc15\r\n", ppb_4_delay_stamp);
    }

    if((ppb_4_delay_stamp <= ppb_3_delay_stamp) ||
       (ppb_3_delay_stamp <= ppb_2_delay_stamp) ||
       (ppb_2_delay_stamp <= ppb_1_delay_stamp) ||
       (ppb_1_delay_stamp)>  2)
    {
        errors++;
    }
    ADC_disableInterrupt(base, int_number);

    util_ADC_deinit(base);

    if(errors > 0)
    {
        /* fail criteria*/
        return 1;
    }
    else
    {
        /* pass criteria*/
        return 0;
    }
}

/*
 * AM263_ADC_BTR_0013	 Asynchronous operation
 */
int32_t AM263x_ADC_BTR_013(uint32_t base)
{
    if(enableLog)
    {
        DebugP_log("\r\n\r\n------------------------------ base : %x -------------------------------------\r\n\r\n", base);
        DebugP_log("Test : Asynchronous operation\r\n\r\n");
    }
    int errors = 0;
    ADC_SOCNumber   soc_number;
    uint32_t        sample_window   =   MIN_SAMPLE_WINDOW;

    /* INPUTXbar to be trigger for all SOCs of all ADCs*/
    ADC_Trigger     trigger    = ADC_TRIGGER_INPUT_XBAR_OUT5;
    ADC_Channel     channel    = ADC_CH_ADCIN2;
    ADC_IntNumber   int_number = ADC_INT_NUMBER1;

    uint32_t        base0 = base;
    uint32_t        adc_offset = 0;

    for(adc_offset = 0x0000;
        adc_offset <= 0x4000;
        adc_offset = adc_offset + 0x1000)
    {
        base = adc_offset + base0;
        util_ADC_init(
            base,
            ADC_CLK_DIV_4_0,
            ADC_MODE_SINGLE_ENDED,
            ADC_PRI_ALL_ROUND_ROBIN);

        ADC_enableInterrupt(base,int_number);
        ADC_clearInterruptStatus(base, int_number);
        for(soc_number = ADC_SOC_NUMBER0;
            soc_number <= ADC_SOC_NUMBER15;
            soc_number++ )
        {
            ADC_setupSOC(
                base,
                soc_number,
                trigger,
                channel,
                sample_window);
            if(soc_number == 15)
            {
                ADC_setInterruptSource(base, int_number, soc_number);
            }
        }
        /* sample_windows for all ADCs will be 16, 19, 22, 25, 28 */
        sample_window = sample_window + 3;
    }

    /* trigger through the inputxbar :: GPIO Loopback*/
    sprintf(test_command,"%s","trigger on GPIO 24 for InputXbar[5] to trigger ADC");
    tester_command(test_command);
    /* All SOCs of all ADCs are triggered.*/

    /* SOC15 of ADC4 should be the last one to be done
        * Hence its interrupt is last to be triggered */

    /* base is now base0 = 0x4000*/
    if(util_ADC_wait_for_adc_interrupt(base, int_number) == false)
    {
        errors++;
    }
    /* errors if other ADCs haven't completed */
    base = base0;
    for(adc_offset = 0x0000;
        adc_offset <=0x4000;
        adc_offset = adc_offset + 0x1000)
        {
            if(ADC_getInterruptStatus(base, int_number) != 1)
            {
                errors++;
            }
        }

    base = base0;
    for(adc_offset = 0x0000;
        adc_offset <= 0x4000;
        adc_offset = adc_offset + 0x1000)
    {
        ADC_disableInterrupt(base,int_number);
        util_ADC_deinit(base);
    }

    if(errors > 0)
    {
        /* fail criteria*/
        return 1;
    }
    else
    {
        return 0;
    }
}

/* AM263_ADC_ITR_0001	 Back to back conversions */
int32_t AM263_ADC_ITR_0001(uint32_t base)
{
    /* Burst Mode test. single trigger to SOC and all the soc's should work on
    single channel, triggered by previous EOC*/
    if(enableLog)
    {
        DebugP_log("\r\n\r\n------------------------------ base : %x -------------------------------------\r\n\r\n", base);
        DebugP_log("Test : Back to back conversions\r\n\r\n");
    }
    int              errors = 0;
    int              sample_window = MIN_SAMPLE_WINDOW;
    ADC_PriorityMode priority;
    volatile ADC_SOCNumber    first_in_burst_mode, last_in_burst_mode;
    volatile uint16_t         burst_size;

    for(priority = ADC_PRI_ALL_ROUND_ROBIN;
        priority  <= ADC_PRI_ALL_HIPRI;
        priority++)
    {

        volatile uint16_t max_burst_size = ADC_PRI_ALL_HIPRI - priority;

        for(burst_size = 1; burst_size <= max_burst_size; burst_size++)
        {
            util_ADC_deinit(base);
            util_ADC_init(
            base,
            ADC_CLK_DIV_4_0,
            ADC_MODE_SINGLE_ENDED,
            priority);

            ADC_setSOCPriority(base, priority);

            ADC_enableInterrupt(base, ADC_INT_NUMBER1);

            ADC_disableContinuousMode(base, ADC_INT_NUMBER1);

            ADC_setBurstModeConfig(base, ADC_TRIGGER_INPUT_XBAR_OUT5, burst_size);
            ADC_enableBurstMode(base);

            first_in_burst_mode = (ADC_SOCNumber) priority;
            last_in_burst_mode = first_in_burst_mode + burst_size - 1 ;

            /* enabling and setting up interrupt for the last soc
                in the high priority */
            ADC_setInterruptSource(
                base,
                ADC_INT_NUMBER1,
                last_in_burst_mode);


            for (ADC_SOCNumber soc_number = first_in_burst_mode; soc_number <= last_in_burst_mode; soc_number++)
            {
                ADC_setupSOC(base,
                            soc_number,
                            ADC_TRIGGER_SW_ONLY,
                            ADC_CH_ADCIN0,
                            sample_window);
            }

            ADC_clearInterruptStatus(base,ADC_INT_NUMBER1);

            util_ADC_fire_soc_trigger(base, first_in_burst_mode, ADC_TRIGGER_INPUT_XBAR_OUT5);

            /*triggering and waiting for the conversions to occur*/

            int wait_success = util_ADC_wait_for_adc_interrupt(base, ADC_INT_NUMBER1);
            if(wait_success == false)
            {
                if(enableLog)
                {
                    DebugP_log("ERROR : fail for %x base and %d first, %d last\r\n",base,first_in_burst_mode, last_in_burst_mode);
                }
                errors++;
            }

            if(enableLog)
            {
                for(int iter = first_in_burst_mode; iter<= last_in_burst_mode; iter++)
                {
                    uint16_t result = util_ADC_check_result(base, iter);
                    DebugP_log("\t\tSOC NUMBER : %d\tresult : %d\r\n", iter, result);
                }
                DebugP_log("\r\n");
            }
            ADC_disableBurstMode(base);
            ADC_disableInterrupt(base,ADC_INT_NUMBER1);
            ADC_disableInterrupt(base,ADC_INT_NUMBER2);
        }

        ADC_disableBurstMode(base);
        util_ADC_deinit(base);
    }

    if(errors > 0)
    {
        /* Fail criteria*/
        return 1;
    }
    else
    {
        /* Pass criteria*/
        return 0;
    }

}

/* AM263_ADC_ITR_0002	 PPBs to trip ePWMs */
int32_t AM263_ADC_ITR_0002(uint32_t base)
{
    if(enableLog)
    {
        DebugP_log("\r\n\r\n------------------------------ base : %x -------------------------------------\r\n\r\n", base);
        DebugP_log("Test : PPBs to trip ePWMs\r\n\r\n");
    }
    int             errors = 0;
    ADC_PPBNumber   ppb_number;
    ADC_Channel     channel;
    uint32_t        triphi_value, triplo_value;

    int             adc_instance = (base & 0x0000f000) >> 12;

    triphi_value    = 2112;
    triplo_value    = 0;
    float voltage;
    uint16_t evtflg = ADC_EVT_TRIPHI;

    util_ADC_init(
        base,
        ADC_CLK_DIV_4_0,
        ADC_MODE_SINGLE_ENDED,
        ADC_PRI_ALL_ROUND_ROBIN);

    enable_all_epwms();
    /* selecting all the adc evt to output[0] of PWM xbar */
    SOC_xbarSelectPWMXBarInputSource(CSL_CONTROLSS_PWMXBAR_U_BASE, 0, 0, 0, 0,
        (EPWM_XBAR_ADC0_EVT1 |
        EPWM_XBAR_ADC0_EVT2 |
        EPWM_XBAR_ADC0_EVT3 |
        EPWM_XBAR_ADC0_EVT4 |
        EPWM_XBAR_ADC1_EVT1 |
        EPWM_XBAR_ADC1_EVT2 |
        EPWM_XBAR_ADC1_EVT3 |
        EPWM_XBAR_ADC1_EVT4 |
        EPWM_XBAR_ADC2_EVT1 |
        EPWM_XBAR_ADC2_EVT2 |
        EPWM_XBAR_ADC2_EVT3 |
        EPWM_XBAR_ADC2_EVT4 |
        EPWM_XBAR_ADC3_EVT1 |
        EPWM_XBAR_ADC3_EVT2 |
        EPWM_XBAR_ADC3_EVT3 |
        EPWM_XBAR_ADC3_EVT4 |
        EPWM_XBAR_ADC4_EVT1 |
        EPWM_XBAR_ADC4_EVT2 |
        EPWM_XBAR_ADC4_EVT3 |
        EPWM_XBAR_ADC4_EVT4 ), 0, 0, 0, 0, 0);

    for(ppb_number = ADC_PPB_NUMBER1;
        ppb_number <=ADC_PPB_NUMBER4;
        ppb_number++)
    {
        if(enableLog)
        {
            DebugP_log("adc base 0x%x\r ppb_number : %d\r\n",base, ppb_number);
        }
        for(int channel_iter = 0; channel_iter < MAX_CHANNEL_COUNT; channel_iter++)
        {
            channel = ADC_channels_list[adc_instance][channel_iter];
            if(channel == NO_CHANNEL)
            {
                continue;
            }
            if(enableLog)
            {
                DebugP_log("---channel  : %d---\r\n",channel);
            }
            ADC_setupSOC(base, ADC_SOC_NUMBER0, ADC_TRIGGER_SW_ONLY,channel, 16);


            ADC_enableInterrupt(base, ADC_INT_NUMBER1);
            ADC_clearInterruptStatus(base, ADC_INT_NUMBER1);
            ADC_setInterruptSource(base, ADC_INT_NUMBER1, ADC_SOC_NUMBER0);

            ADC_setupPPB(base, ppb_number, ADC_SOC_NUMBER0);

            /* for each ppb_numbers, assign trip functionality */
            ADC_setPPBTripLimits(base, ppb_number, triphi_value, triplo_value);
            ADC_enablePPBEvent(base, ppb_number, evtflg);
            ADC_clearPPBEventStatus(base, ppb_number,evtflg);

            /* get adc input to be higher than the trip value and small wait*/
            if(manual_testing)
            {
                DebugP_log("Please input voltage higher than trip hi value 1.65V\r\n");
                wait_for_user_uart_input();
            }
            else
            {
                voltage = 3.000;
                sprintf(test_command, "provide analog voltage of %.4fV on ADC %d Channel %d",voltage, 0, 0);
                tester_command(test_command);
                sprintf(test_command, "provide analog voltage of %.4fV on ADC %d Channel %d",voltage, 0, 1);
                tester_command(test_command);
            }

            ADC_forceSOC(base, ADC_SOC_NUMBER0);
            if(util_ADC_wait_for_adc_interrupt(base, ADC_INT_NUMBER1) == false)
            {
                errors++;
            }

            ADC_clearInterruptStatus(base, ADC_INT_NUMBER1);

            if( (SOC_xbarGetPWMXBarOutputSignalStatus(CSL_CONTROLSS_PWMXBAR_U_BASE) != 1)
                ||
                ((ADC_getPPBEventStatus(base, ppb_number) & evtflg) != evtflg))
                {
                    errors++;
                    if(enableLog)
                    {
                        DebugP_log("Trip signal failed, ppbevtstats : %x.\r\n ppbresult : %x \t socresult : %x\r\n", (ADC_getPPBEventStatus(base, ppb_number) & evtflg) , ADC_readPPBResult((base - 0x001c0000),ppb_number), ADC_readResult((base - 0x001c0000), ADC_SOC_NUMBER0));
                    }
                }

            ADC_clearPPBEventStatus(base, ppb_number, evtflg);

            if((SOC_xbarGetPWMXBarOutputSignalStatus(CSL_CONTROLSS_PWMXBAR_U_BASE) == 1))
                {
                errors++;
                if(enableLog)
                {
                    DebugP_log("Trip signal is not cleared\r\n");
                }
                }
            ADC_disablePPBEvent(base, ppb_number, evtflg);

            if(manual_testing)
            {
                DebugP_log("the input voltage may be changed back to default\r\n");
                wait_for_user_uart_input();
            }
            else
            {
                voltage = 1.000;
                sprintf(test_command, "provide analog voltage of %.4fV on ADC %d Channel %d",voltage, 0, 0);
                tester_command(test_command);
                sprintf(test_command, "provide analog voltage of %.4fV on ADC %d Channel %d",voltage, 0, 1);
                tester_command(test_command);
            }

            /* There should not be any trip occuring now.*/
            ADC_forceSOC(base, ADC_SOC_NUMBER0);
            if(util_ADC_wait_for_adc_interrupt(base, ADC_INT_NUMBER1) == false)
            {
                errors++;
            }
            ADC_clearInterruptStatus(base, ADC_INT_NUMBER1);

            if( (SOC_xbarGetPWMXBarOutputSignalStatus(CSL_CONTROLSS_PWMXBAR_U_BASE) == 1)
            ||
            ((ADC_getPPBEventStatus(base, ppb_number) & evtflg) == evtflg))
            {
                errors++;
                if(enableLog)
                {
                    DebugP_log("ERROR : Trip signal occurred, ppbevtstats : %x.\r\n ppbresult : %x \t socresult : %x\r\n", (ADC_getPPBEventStatus(base, ppb_number) & evtflg) , ADC_readPPBResult((base - 0x001c0000),ppb_number), ADC_readResult((base - 0x001c0000), ADC_SOC_NUMBER0));
                }
            }
        }
    }

    disable_all_epwms();
    util_ADC_deinit(base);

    if(errors > 0)
    {
        /*fail criteria*/
        return 1;
    }
    else
    {
        /*pass criteria*/
        return 0;
    }

}

/* AM263_ADC_ITR_0003	 Periodic Triggering of ADC Conversion from ePWM */
int32_t AM263_ADC_ITR_0003(uint32_t base)
{
    if(enableLog)
    {
        DebugP_log("\r\n\r\n------------------------------ base : %x -------------------------------------\r\n\r\n", base);
        DebugP_log("Test : Periodic Triggering of ADC Conversion from ePWM\r\n\r\n");
    }
    /* No need to be iterated over channels*/

    /*-----------------------------variables---------------------------------*/
    int errors = 0;
    ADC_SOCNumber   soc_number      = ADC_SOC_NUMBER0;
    ADC_Trigger     trigger         = ADC_TRIGGER_EPWM0_SOCA;
    ADC_Channel     channel         = ADC_CH_ADCIN0;
    uint32_t        sample_window   = MIN_SAMPLE_WINDOW;

    uint32_t status = SemaphoreP_constructBinary(&gEpwmIsrSemObject, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    /* ----------disabling previous interrupt and adding new ones------------*/
    /* disabling previous interrupt for pwm*/
    HwiP_destruct(&gEpwmHwiObject1);

    /* setting up epwm interrupt to route through int xbar 1*/
    SOC_xbarSelectInterruptXBarInputSource(
        CSL_CONTROLSS_INTXBAR_U_BASE,
        0,                                      //instance
        ( INT_XBAR_EPWM0_INT ),                 //epwm 0 interrupt
        0, 0, 0, 0, 0, 0);                      //no other group

    /* Initialising a Interrupt parameter
     * setting up the interrupt, callbacks.*/
    HwiP_Params  hwiPrms;
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_0;
    hwiPrms.callback    = &EPWM_INT_ISR_for_periodic_triggering_from_epwm;
    hwiPrms.isPulse     = 1;
    status              = HwiP_construct(&gEpwmHwiObject2, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);

    /* setting up ADCINT1s to route through int xbar 2*/
    SOC_xbarSelectInterruptXBarInputSource(
        CSL_CONTROLSS_INTXBAR_U_BASE,
        1, 0, 0,
        (INT_XBAR_ADC0_INT1 | INT_XBAR_ADC1_INT1 | INT_XBAR_ADC2_INT1 | INT_XBAR_ADC3_INT1 | INT_XBAR_ADC4_INT1),
        0, 0, 0, 0);

    /* Initialising a Interrupt parameter
     * setting up the interrupt, callbacks.*/
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_1;
    hwiPrms.callback    = &ADC_INT_ISR_for_periodic_triggering_from_epwm;
    hwiPrms.isPulse     = 1;
    status              = HwiP_construct(&gAdcHwiObject1, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);

    /*--------------------------test setup-----------------------------*/
    enable_all_epwms();
    /* initialising epwm but the counter is frozen*/
    util_EPWM_init(CSL_CONTROLSS_G0_EPWM0_U_BASE);

    util_ADC_init(
        base,
        ADC_CLK_DIV_4_0,
        ADC_MODE_DIFFERENTIAL,
        ADC_PRI_ALL_ROUND_ROBIN);

    ADC_setupSOC(
        base,
        soc_number,
        trigger,
        channel,
        sample_window);

    ADC_enableInterrupt(base,ADC_INT_NUMBER1);
    ADC_enableContinuousMode(base, ADC_INT_NUMBER1);

    ADC_setInterruptSource(base, ADC_INT_NUMBER1, soc_number);

    util_EPWM_setup_adc_trigger(CSL_CONTROLSS_G0_EPWM0_U_BASE, EPWM_SOC_A);

    gAdc_ISR1_count     = 0;
    gEpwm_ISR2_count    = 0;
    EPWM_clearEventTriggerInterruptFlag(CSL_CONTROLSS_G0_EPWM0_U_BASE);

    util_EPWM_start_counter(0); //starting epwm0 counter

    SemaphoreP_pend(&gEpwmIsrSemObject, 0);

    util_EPWM_stop_counter(CSL_CONTROLSS_G0_EPWM0_U_BASE);
    EPWM_clearEventTriggerInterruptFlag(CSL_CONTROLSS_G0_EPWM0_U_BASE);
    int difference = gAdc_ISR1_count - gEpwm_ISR2_count;

    if((difference < -1) || (difference > 1))
    {
        errors++;
        if(enableLog)
        {
            DebugP_log("ERROR : ISR Counts did not match\r\n");
        }
    }

    HwiP_destruct(&gEpwmHwiObject2);
    HwiP_destruct(&gAdcHwiObject1);

    ADC_disableInterrupt(base,ADC_INT_NUMBER1);

    util_ADC_deinit(base);
    util_EPWM_deinit(CSL_CONTROLSS_G0_EPWM0_U_BASE);

    disable_all_epwms();

    if(errors > 0 )
    {
        /* fail criteria*/
        return 1;
    }
    else
    {
        /* pass criteria*/
        return 0;
    }
}

/* AM263_ADC_TTR_0001	 ADC Read latency */
int32_t AM263_ADC_TTR_0001(uint32_t base)
{
    if(enableLog)
    {
        DebugP_log("\r\n\r\n------------------------------ base : %x -------------------------------------\r\n\r\n", base);
        DebugP_log("Test : ADC Read latency\r\n\r\n");
    }
    int errors = 0;
    int latency_test_max_iterations = 2;

    if(enableLog)
    {
        DebugP_log("\r\nbase 0x%x\r\n",base);
    }
    for(int iter = latency_test_max_iterations; iter >= 0; iter--)
    {
        int errors_count = 0;
        volatile uint32_t counter_start_stop_read_latency = 0;
        volatile uint32_t adc_result_read_latency = 0;

        volatile uint16_t result_dummy = 0;

        CycleCounterP_reset();
        counter_start_stop_read_latency = CycleCounterP_getCount32();


        for(int soc_number = 0; soc_number <= 15; soc_number++)
        {
            uint32_t result_addr = (base - 0x001c0000);
            CycleCounterP_reset();
            result_dummy = ADC_readResult(result_addr, soc_number);
            adc_result_read_latency = CycleCounterP_getCount32();
            adc_result_read_latency -= counter_start_stop_read_latency;
            if((enableLog==1) && (iter == 0))
            {
                DebugP_log("%d is adc_result_read_latency, %d is result \r\n", adc_result_read_latency, result_dummy);
            }
            /* observation 19 or 20 cycles*/
            if((adc_result_read_latency < MIN_SAMPLE_WINDOW) || (adc_result_read_latency > 22))
            {
                errors_count++;
            }
        }

        if(iter == 0) errors = errors_count;
    }

    if(errors > 0)
    {
        /* fail criteria */
        return 1;
    }
    else
    {
        /* pass criteria */
        return 0;
    }
}


void ADC_ISR_for_ADC_TTR_0002(void* args)
{
    gISR_latency = CycleCounterP_getCount32();
    gISR_complete++;
}

/* AM263_ADC_TTR_0002	 ADC Latch to R5F response latency */
int32_t AM263_ADC_TTR_0002(uint32_t base)
{
    if(enableLog)
    {
        DebugP_log("\r\n\r\n------------------------------ base : %x -------------------------------------\r\n\r\n", base);
        DebugP_log("Test : ADC Latch to R5F response latency\r\n\r\n");
    }
    int errors = 0;
    int latency_test_max_iterations = 2;

    for(int iter = latency_test_max_iterations; iter >= 0; iter--)
    {
        HwiP_destruct(&gAdcHwiObject2);


        while(ADC_isBusy(base) == true);

        volatile uint32_t trigger_API_to_int_flg_latency[4] = {0,0,0,0};
        volatile uint32_t temp;

        gISR_latency = 0;
        gISR_complete = 0;

        for(int iterative=0; iterative<=3; iterative++)
        {
            util_ADC_deinit(base);

            util_ADC_init(
                base,
                ADC_CLK_DIV_4_0,
                ADC_MODE_DIFFERENTIAL,
                ADC_PRI_ALL_ROUND_ROBIN);

            ADC_setInterruptPulseMode(base,ADC_PULSE_END_OF_ACQ_WIN);

            ADC_setupSOC(
                base,
                ADC_SOC_NUMBER0,
                ADC_TRIGGER_SW_ONLY,
                ADC_CH_ADCIN2,
                16);

            ADC_enableInterrupt(base, iterative);
            ADC_clearInterruptStatus(base, iterative);
            ADC_setInterruptSource(base, iterative, ADC_SOC_NUMBER0);

            trigger_API_to_int_flg_latency[iterative] = 0;

            while(ADC_isBusy(base) == true);
            CycleCounterP_reset();

            ADC_forceSOC(base,ADC_SOC_NUMBER0);
            while(ADC_getInterruptStatus(base, iterative) != 1);
            temp = CycleCounterP_getCount32();

            trigger_API_to_int_flg_latency[iterative] = temp;

            ADC_clearInterruptStatus(base, iterative);
            ADC_disableInterrupt(base, iterative);

        }

        for( int iterative = 0; iterative<4 ; iterative++)
        {
            if((enableLog == 1) && (iter == 0 ) )
            {
                DebugP_log("API to interrupt %d flag latency :\t%d\r\n", iterative, trigger_API_to_int_flg_latency[iterative]);
            }
        }

        ClockP_sleep(1);

        uint32_t status;

        /* setting up interrupt route for all adc instance*/
        /* for Interrupt xbar instance 1*/

        SOC_xbarSelectInterruptXBarInputSource(
            CSL_CONTROLSS_INTXBAR_U_BASE,
            1, 0, 0,
            0x01FFFFFF,
            0, 0, 0, 0);

        HwiP_Params  hwiPrms;
        HwiP_Params_init(&hwiPrms);
        hwiPrms.intNum      = CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_1;
        hwiPrms.priority    = 0;
        hwiPrms.callback    = &ADC_ISR_for_ADC_TTR_0002;
        hwiPrms.isFIQ       = 1;
        hwiPrms.isPulse     = 1;




        ADC_setInterruptPulseMode(base, ADC_PULSE_END_OF_ACQ_WIN);

        for(ADC_IntNumber int_number = ADC_INT_NUMBER1;
                        int_number <= ADC_INT_NUMBER4;
                        int_number++)
        {
            util_ADC_deinit(base);

            util_ADC_init(
                base,
                ADC_CLK_DIV_4_0,
                ADC_MODE_DIFFERENTIAL,
                ADC_PRI_ALL_ROUND_ROBIN);

            ADC_setInterruptPulseMode(base,ADC_PULSE_END_OF_ACQ_WIN);

            ADC_setupSOC(
                base,
                ADC_SOC_NUMBER0,
                ADC_TRIGGER_SW_ONLY,
                ADC_CH_ADCIN2,
                MIN_SAMPLE_WINDOW);

            gISR_latency = 0;

            ADC_disableInterrupt(base, int_number);

            gISR_complete = 0;
            ADC_enableInterrupt(base, int_number);
            ADC_setInterruptSource(base, int_number, ADC_SOC_NUMBER0);
            ADC_clearInterruptStatus(base, int_number);
            status              = HwiP_construct(&gAdcHwiObject2, &hwiPrms);
            DebugP_assert(status == SystemP_SUCCESS);

            while(ADC_isBusy(base) == true);

            CycleCounterP_reset();
            ADC_forceSOC(base,ADC_SOC_NUMBER0);

            while(gISR_complete != 1);

            ADC_clearInterruptStatus(base, int_number);

            gISR_latency -= (trigger_API_to_int_flg_latency[int_number]);

            if((enableLog == 1) && (iter == 0))
            {
                DebugP_log("ADC INT %d latch to r5 response :\t%d\r\n",int_number, gISR_latency);
            }

            if(iter == 0)
            {
                if((gISR_latency <= 20)||(gISR_latency >= 30))
                errors++;
            }
            ADC_disableInterrupt(base, int_number);

            HwiP_destruct(&gAdcHwiObject2);

        }

        HwiP_destruct(&gAdcHwiObject2);

        util_ADC_deinit(base);

        ClockP_sleep(1);

    }
    if(errors > 0 )
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/* AM263_ADC_TTR_0003	 R5F R/W latencies to ADC */
int32_t AM263_ADC_TTR_0003(uint32_t base)
{
    if(enableLog)
    {
        DebugP_log("\r\n\r\n------------------------------ base : %x -------------------------------------\r\n\r\n", base);
        DebugP_log("Test : R5F R/W latencies to ADC\r\n\r\n");
    }
    int errors = 0;
    int latency_test_max_iterations = 2;
    for(int iter = latency_test_max_iterations; iter >= 0; iter--)
    {
        uint32_t counter_values_for_read[5] = {0,0,0,0,0};
        uint32_t counter_values_for_write[3] = {0,0,0};

        uint32_t counter_read_latency = 0;

        uint32_t temp_count = 0;

        uint32_t result_base = base - (uint32_t) 0x001c0000;

        CycleCounterP_reset();
        counter_read_latency = CycleCounterP_getCount32();

        while(true == ADC_isBusy(base));

        /* 16 bit read */
        CycleCounterP_reset();
        adc_output_16 = ADC_getPPBDelayTimeStamp(base,0);
        temp_count = CycleCounterP_getCount32();
        counter_values_for_read[0] = temp_count - counter_read_latency;
        temp_count = 0;
        (void)adc_output_16;

        /* 16 bit write */
        CycleCounterP_reset();
        // HW_WR_REG16((base + CSL_ADC_ADCINTSOCSEL1), adc_output_16);
        ADC_clearInterruptStatus(base, 0);
        temp_count = CycleCounterP_getCount32();
        counter_values_for_write[0]= temp_count - counter_read_latency;
        temp_count = 0;

        /* 32 bit read */
        CycleCounterP_reset();
        adc_output_32 = HW_RD_REG32(base + CSL_ADC_ADCINTSOCSEL1);
        temp_count = CycleCounterP_getCount32();
        counter_values_for_read[1] = temp_count - counter_read_latency;
        temp_count = 0;

        /* 32 bit write */
        CycleCounterP_reset();
        HW_WR_REG16((base + CSL_ADC_ADCINTSOCSEL1), adc_output_32);
        temp_count = CycleCounterP_getCount32();
        counter_values_for_write[1] = temp_count - counter_read_latency;
        temp_count = 0;

        /* 16 bit result space read */
        CycleCounterP_reset();
        // adc_output_16 = HW_RD_REG16(result_base + CSL_ADC_RESULT_ADCRESULT0);
        adc_output_16 = ADC_readResult(result_base, 0);
        temp_count = CycleCounterP_getCount32();
        counter_values_for_read[2] = temp_count - counter_read_latency;
        temp_count = 0;
        (void) adc_output_16;

        /*32 bit result space read */
        CycleCounterP_reset();
        adc_output_32 = HW_RD_REG32(result_base + CSL_ADC_RESULT_ADCRESULT0);
        temp_count = CycleCounterP_getCount32();
        counter_values_for_read[3] = temp_count - counter_read_latency;
        temp_count = 0;
        (void) adc_output_32;

        /* 32 bit burst write*/
        CycleCounterP_reset();
        memcpy( (void*)(base + CSL_ADC_ADCSOC0CTL), adc_output_32_write, 16);
        temp_count = CycleCounterP_getCount32();
        counter_values_for_write[2] = temp_count - counter_read_latency;
        temp_count = 0;

        /* 32 bit burst read*/
        CycleCounterP_reset();
        memcpy( adc_output_32_write, (void*)(base + CSL_ADC_ADCSOC0CTL), 16);
        temp_count = CycleCounterP_getCount32();
        counter_values_for_read[4] = temp_count - counter_read_latency;
        temp_count = 0;

        if(iter == 0)
        {
            /* the following comparisons are taken from a manual run. need to verify*/
            if ((counter_values_for_read[0] < 30-3) || (counter_values_for_read[0] > 30+3) )
            {
                errors++ ;    /* 16 bit read */
                if(enableLog)
                {
                    DebugP_log("ERROR FOR BASE : %X 16 bit read : %d\r\n",base,counter_values_for_read[0]);
                }
            }
            if ((counter_values_for_read[1] < 30-3) || (counter_values_for_read[1] > 30+3) )
            {
                errors++ ;    /* 32 bit read */
                if(enableLog)
                {
                    DebugP_log("ERROR FOR BASE : %X 32 bit read : %d\r\n",base,counter_values_for_read[1]);
                }
            }
            if ((counter_values_for_read[2] < 25-4) || (counter_values_for_read[2] > 25+4) )
            {
                errors++ ;    /* 16 bit result space read */
                if(enableLog)
                {
                    DebugP_log("ERROR FOR BASE : %X 16 bit result space read  : %d\r\n",base,counter_values_for_read[2]);
                }

            }
            if ((counter_values_for_read[3] < 22-3) || (counter_values_for_read[3] > 22+3) )
            {
                errors++ ;    /* 32 bit result space read */
                if(enableLog)
                {
                    DebugP_log("ERROR FOR BASE : %X 32 bit result space read  : %d\r\n",base,counter_values_for_read[3]);
                }
            }
            if ((counter_values_for_read[4] < 470-10) || (counter_values_for_read[4] > 470+10) )
            {
                errors++ ;  /* 32 bit result space burst read 4*32 bit*/
                if(enableLog)
                {
                    DebugP_log("ERROR FOR BASE : %X 32 bit result space burst read 4*32 bit : %d\r\n",base,counter_values_for_read[4]);
                }

            }
            if ((counter_values_for_write[0] < 11-3) || (counter_values_for_write[0] > 11+4) )
            {
                errors++ ;  /* 16 bit write */
                if(enableLog)
                {
                    DebugP_log("ERROR FOR BASE : %X 16 bit write : %d\r\n",base,counter_values_for_write[0]);
                }

            }
            if ((counter_values_for_write[1] < 11-3) || (counter_values_for_write[1] > 11+3) )
            {
                errors++ ;  /* 32 bit write */
                if(enableLog)
                DebugP_log("ERROR FOR BASE : %X 32 bit write : %d\r\n",base,counter_values_for_write[1]);

            }
            if ((counter_values_for_write[2] < 250-16) || (counter_values_for_write[2] > 250+16) )
            {
                errors++ ; /* 32 bit result space burst write 4*32 bit */
                if(enableLog)
                DebugP_log("ERROR FOR BASE : %X 32 bit result space burst write 4*32 bit : %d\r\n",base,counter_values_for_write[2]);

            }
            if(enableLog == 1)
            {
                DebugP_log("\r\nbase : 0x%x\r\n", base);
                DebugP_log("16 bit read latency : %d\r\n",counter_values_for_read[0]);
                DebugP_log("32 bit read latency : %d\r\n",counter_values_for_read[1]);
                DebugP_log("16 bit result read latency : %d\r\n",counter_values_for_read[2]);
                DebugP_log("32 bit result read latency : %d\r\n",counter_values_for_read[3]);
                DebugP_log("32 bit result burst read latency : %d\r\n",counter_values_for_read[4]);
                DebugP_log("16 bit write latency : %d\r\n",counter_values_for_write[0]);
                DebugP_log("32 bit write latency : %d\r\n",counter_values_for_write[1]);
                DebugP_log("32 bit result burst write latency : %d\r\n",counter_values_for_write[2]);
            }
        }
    }
if(errors > 0)
{
    /* fail criteria */
    return 1;
}
else
{
    /* pass criteria */
    return 0;
}

}

/* AM263_ADC_TTR_0004	 DMA Latency */
int32_t AM263_ADC_TTR_0004(uint32_t base)
{
    if(enableLog)
    {
        DebugP_log("\r\n\r\n------------------------------ base : %x -------------------------------------\r\n\r\n", base);
        DebugP_log("Test : DMA Latency \r\n\r\n");
    }

    int errors;
    int latency_test_max_iterations = 2;

    int errors_count = 0;
    int adc_instance = (base & 0x0000f000)>>12;
    uint32_t adc_result_base_addr = base - 0x001c0000;
    volatile uint32_t trigger_API_to_int_flg_latency = 0;
    volatile uint32_t dma_trigger_to_transfer_complete_latency;
    volatile uint32_t temp;

    for(int iter = latency_test_max_iterations; iter >= 0; iter--)
    {

        while(ADC_isBusy(base) == true);

        util_ADC_init(base, ADC_CLK_DIV_4_0, ADC_MODE_DIFFERENTIAL, ADC_PRI_ALL_ROUND_ROBIN);
        ADC_setInterruptPulseMode(base, ADC_PULSE_END_OF_ACQ_WIN);

        ClockP_usleep(1000);

        ADC_setupSOC( base, ADC_SOC_NUMBER0, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN2, 16);

        ADC_disableInterrupt(base, ADC_INT_NUMBER1);
        ADC_disableInterrupt(base, ADC_INT_NUMBER2);
        ADC_disableInterrupt(base, ADC_INT_NUMBER3);
        ADC_disableInterrupt(base, ADC_INT_NUMBER4);

        ADC_enableInterrupt(base, 0);
        ADC_setInterruptSource(base, 0 , ADC_SOC_NUMBER0);
        ADC_clearInterruptStatus(base, 0);

        ClockP_usleep(1000);
        while(ADC_isBusy(base) == true);

        CycleCounterP_reset();
        ADC_forceSOC(base,ADC_SOC_NUMBER0);

        while(ADC_getInterruptStatus(base, 0) != 1);
        temp = CycleCounterP_getCount32();

        trigger_API_to_int_flg_latency = temp;

        ADC_clearInterruptStatus(base, 0);

        if((enableLog == 1) && (iter == 0))
        {
            DebugP_log("API to interrupt %d flag latency :\t%d\r\n", ADC_INT_NUMBER1,trigger_API_to_int_flg_latency);
        }

        /* -----------------------------------EDMA Setup------------------------------------ */

        util_EDMA_Open();

        int DMA_xbar_input = (adc_instance)*5;
        SOC_xbarSelectEdmaTrigXbarInputSource(CSL_EDMA_TRIG_XBAR_U_BASE, DMA_TRIG_XBAR_EDMA_MODULE_0, DMA_TRIG_XBAR_DMA_XBAR_OUT_0);
        SOC_xbarSelectDMAXBarInputSource(CSL_CONTROLSS_DMAXBAR_U_BASE, 0, 2, 0, 0, DMA_xbar_input, 0, 0, 0);


        EDMACCPaRAMEntry    edmaParam;
        uint32_t            baseAddr, regionId;
        uint32_t            dmaCh, tcc, param;
        int32_t             testStatus = SystemP_SUCCESS;

        baseAddr = EDMA_getBaseAddr(gEdmaHandle[0]);
        DebugP_assert(baseAddr != 0);

        regionId = EDMA_getRegionId(gEdmaHandle[0]);
        DebugP_assert(regionId < SOC_EDMA_NUM_REGIONS);

        dmaCh = ADC0_EDMA_CHANNEL;
        testStatus = EDMA_allocDmaChannel(gEdmaHandle[0], &dmaCh);
        DebugP_assert(testStatus == SystemP_SUCCESS);

        tcc = EDMA_RESOURCE_ALLOC_ANY;
        testStatus = EDMA_allocTcc(gEdmaHandle[0], &tcc);
        DebugP_assert(testStatus == SystemP_SUCCESS);

        param = EDMA_RESOURCE_ALLOC_ANY;
        testStatus = EDMA_allocParam(gEdmaHandle[0], &param);
        DebugP_assert(testStatus == SystemP_SUCCESS);

        EDMA_configureChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
            dmaCh, tcc, param, EDMA_TEST_EVT_QUEUE_NO);

        EDMA_disableEvtIntrRegion(baseAddr, regionId, dmaCh);


        EDMA_ccPaRAMEntry_init(&edmaParam);
        edmaParam.srcAddr       = (uint32_t) SOC_virtToPhy((void *)(adc_result_base_addr));
        edmaParam.destAddr      = (uint32_t) SOC_virtToPhy((void *) 0x50260000);
        edmaParam.aCnt          = (uint16_t) 2*16;  /*data from 16 16-bit result registers to be transfered*/
        edmaParam.bCnt          = (uint16_t) 1;     /*single transfer*/
        edmaParam.cCnt          = (uint16_t) 1;
        edmaParam.bCntReload    = 0;
        edmaParam.srcBIdx       = (int16_t) EDMA_PARAM_BIDX(0);
        edmaParam.destBIdx      = (int16_t) EDMA_PARAM_BIDX(2);
        edmaParam.srcCIdx       = (int16_t) 0;
        edmaParam.destCIdx      = (int16_t) 0;
        edmaParam.linkAddr      = 0xFFFFU;
        edmaParam.srcBIdxExt    = (int8_t) EDMA_PARAM_BIDX_EXT(0);
        edmaParam.destBIdxExt   = (int8_t) EDMA_PARAM_BIDX_EXT(2);
        edmaParam.opt           = (EDMA_OPT_TCINTEN_MASK |
                                ((((uint32_t)tcc) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));

        EDMA_setPaRAM(baseAddr, param, &edmaParam);

        EDMA_enableTransferRegion(baseAddr, regionId, dmaCh,
                                EDMA_TRIG_MODE_EVENT);

        /* -----------------------------EDMA Setup Complete-------------------------------- */

        /* ----------------------Force ADC SOC to generate DMA trigger--------------------- */
        ADC_clearInterruptStatus(base, ADC_INT_NUMBER1);

        while(ADC_isBusy(base) == true);

        CycleCounterP_reset();
        ADC_forceSOC(base, ADC_SOC_NUMBER0);

        while(EDMA_readIntrStatusRegion(baseAddr, regionId, tcc) != 1);
        dma_trigger_to_transfer_complete_latency = CycleCounterP_getCount32();

        /*---------------------------- DMA TRANSFER COMPLETE------------------------------- */

        EDMA_clrIntrRegion(baseAddr, regionId, tcc);

        EDMA_freeChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
            dmaCh, EDMA_TRIG_MODE_EVENT, tcc, EDMA_TEST_EVT_QUEUE_NO);

        testStatus = EDMA_freeDmaChannel(gEdmaHandle[0], &dmaCh);
        DebugP_assert(testStatus == SystemP_SUCCESS);
        testStatus = EDMA_freeTcc(gEdmaHandle[0], &tcc);
        DebugP_assert(testStatus == SystemP_SUCCESS);
        testStatus = EDMA_freeParam(gEdmaHandle[0], &param);
        DebugP_assert(testStatus == SystemP_SUCCESS);

        util_EDMA_Close();

        util_ADC_deinit(base);

        if((enableLog == 1) && (iter == 0))
        {
            DebugP_log("total latency :\t%d\r\n",dma_trigger_to_transfer_complete_latency);
        }
        dma_trigger_to_transfer_complete_latency -= trigger_API_to_int_flg_latency;

        if(iter == 0)
        {
            /* from observation */
            if((dma_trigger_to_transfer_complete_latency > 40+10 ) || (dma_trigger_to_transfer_complete_latency < 40-10 ))
            {
                errors_count++;
            }
            errors = errors_count;
            if(enableLog)
            {
                if(errors)
                {
                    DebugP_log("error!!!  %d\r\n",dma_trigger_to_transfer_complete_latency);
                }
                DebugP_log("adc_instance : %d, DMA trigger to transfer complete latency  : %d\r\n",adc_instance, dma_trigger_to_transfer_complete_latency);
            }

        }
    }
    if(errors > 0)
    {
        /* fail criteria */
        return 1;
    }
    else
    {
        /* pass criteria */
        return 0;
    }
}

int32_t test_adc_cases(uint8_t in)
{
    int32_t failcount = 0;
    uint32_t adc_offset = 0;
    uint32_t base;

    bool single_run_test = false;

    for (adc_offset = 0x0000; adc_offset <= 0x4000; adc_offset = adc_offset + 0x1000)
    {

        base = CSL_CONTROLSS_ADC0_U_BASE + adc_offset;

        switch (in)
        {
        case 1:
            /* AM263_ADC_BTR_0001	 Single ended conversion */
            failcount += AM263x_ADC_BTR_001(base);
            break;
        case 2:
            /* AM263_ADC_BTR_0002	 Differential conversion with prescaled
                                     clock */
            failcount += AM263x_ADC_BTR_002(base);
            break;
        case 3:
            /* AM263_ADC_BTR_0003	 Software triggering */
            failcount += AM263x_ADC_BTR_003(base);
            break;
        case 4:
            /* AM263_ADC_BTR_0004	 Triggering from Timer */
            failcount += AM263x_ADC_BTR_004(base);
            break;
        case 5:
            /* AM263_ADC_BTR_0005	 InputXBar Trigger */
            failcount += AM263x_ADC_BTR_005(base);
            break;
        case 6:
            /* AM263_ADC_BTR_0006	 ePWM SOC Trigger */
            failcount += AM263x_ADC_BTR_006(base);
            break;
        case 7:
            /* AM263_ADC_BTR_0007	 All ADCs working together */
            /*
             * This test will include all the ADCs in one testcase. so there is
             * no need for this to be repeated.
             */
            failcount += AM263x_ADC_BTR_007();
            single_run_test = true;
            break;
        case 8:
            /* AM263_ADC_BTR_0008	 Force multiple SOCs simultaneously */
            failcount += AM263x_ADC_BTR_008(base);
            break;
        case 9:
            /* AM263_ADC_BTR_0009	 Enable PPB ADCEVT cycle by cycle mode*/
            failcount += AM263x_ADC_BTR_009(base);
            break;
        case 10:
            /* AM263_ADC_BTR_0010	 clear ADCINTOVF status register */
            failcount += AM263x_ADC_BTR_010(base);
            break;
        case 11:
            /* AM263_ADC_BTR_0011	 PPB error calculation */
            failcount += AM263x_ADC_BTR_011(base);
            break;
        case 12:
            /* AM263_ADC_BTR_0012	 Trigger to sample delay Capture */
            failcount += AM263x_ADC_BTR_012(base);
            break;
        case 13:
            /* AM263_ADC_BTR_0013	 Asynchronous operation */
            failcount += AM263x_ADC_BTR_013(base);
            single_run_test = true;
            break;
        case 14:
            /* AM263_ADC_ITR_0001	 Back to back conversions
                                     after receiving trigger */
            failcount += AM263_ADC_ITR_0001(base);
            break;
        case 15:
            /* AM263_ADC_ITR_0002	 PPBs to trip ePWMs */
            failcount += AM263_ADC_ITR_0002(base);
            break;
        case 16:
            /* AM263_ADC_ITR_0003	 Periodic Triggering of ADC Conversion
                                     from ePWM */
            failcount += AM263_ADC_ITR_0003(base);
            break;
        case 17:
            /* AM263_ADC_TTR_0001	 ADC Read latency */
            failcount += AM263_ADC_TTR_0001(base);
            break;
        case 18:
            /* AM263_ADC_TTR_0002	 ADC Latch to R5F response latency */
            failcount += AM263_ADC_TTR_0002(base);
            break;
        case 19:
            /* AM263_ADC_TTR_0003	 R5F R/W latencies to ADC */
            failcount += AM263_ADC_TTR_0003(base);
            break;
        case 20:
            /* AM263_ADC_TTR_0004	 DMA Latency */
            failcount += AM263_ADC_TTR_0004(base);
            break;
        case 21:
            /* ecap SOC triggers */
            failcount += AM263_ADC_BTR_NEW(base);
            break;
        }

        if(single_run_test || (!TEST_ALL_INSTANCES))
        {
            break;
        }
    }

    if (failcount != 0)
    {
        if(enableLog)
        {
            DebugP_log("%d is the failcount FYI\r\n", failcount);
        }
        return 1;
    }
    else
    {
        return 0;
    }
}