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
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

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

/* ========================================================================== */
/* TA-DUT UART communincation related definitions, macros, variables */
#define CMD_TX_BUFSIZE (200U)
#define CMD_RX_BUFSIZE (200U)

#define CMD_SIZE (64U)
#define RSP_SIZE (64U)

#define SIZE_STRING_TEST_NAME (70)
#define TIMEOUT_UART_MENU (10000)
#define TIMEOUT_UART_TESTER (10000)
#define TIMEOUT_UART_INFINITE (SystemP_WAIT_FOREVER)

char test_command[CMD_SIZE];
uint8_t gCmdTxBuffer[CMD_TX_BUFSIZE];
uint8_t gCmdRxBuffer[CMD_RX_BUFSIZE];

volatile uint32_t gNumBytesRead = 0U, gNumBytesWritten = 0U;

UART_Transaction trans;
int32_t transferOK;

bool enableLog;
bool manual_testing;

/* Initialises the tester-DUT UART Communication.*/
void tester_int(void);
/* waits for the user uart input of char 'y'*/
void wait_for_user_uart_input(void);
/* Sends out the str into UART based on the manual_testing*/
void tester_command(char *str);

void tester_init(void)
{
    UART_Transaction_init(&trans);
    trans.timeout = TIMEOUT_UART_TESTER;

    if (enableLog)
        DebugP_log("\r\nSending initialization command!!");

    gNumBytesWritten = 0U;
    trans.buf = &gCmdTxBuffer[0U];
    strncpy(trans.buf, "123456780000000000000000ABCDEFEF", CMD_SIZE);
    trans.count = CMD_SIZE; // if not pulled up CMD_SIZE-1 expected. Send CMD_SIZE-1 only to sync. Unexpected 1.
    transferOK = UART_write(gUartHandle[TESTER_UART], &trans);
    /* Quickly wait to read ack. No debug prints here*/
    gNumBytesRead = 0U;
    trans.buf = &gCmdRxBuffer[0U];
    trans.count = RSP_SIZE;
    transferOK = UART_read(gUartHandle[TESTER_UART], &trans);

    if (enableLog)
        DebugP_log("\r\nReceived response. Tester Initialized!!");

    /*Clear TX buffer for shorter commands*/
    uint8_t ind;
    for (ind = 0; ind < CMD_SIZE; ind++)
    {
        gCmdTxBuffer[ind] = 0;
    }
}

void wait_for_user_uart_input(void)
{

    bool correct_input = false;

    while (!correct_input)
    {
        DebugP_log("please enter 'y' after setting up required configuration for test\r\n");

        /*Clearing the Rx buffer for 1 char*/
        gCmdRxBuffer[0] = '\0';

        /* forever wait until the user inputs*/
        trans.timeout = TIMEOUT_UART_INFINITE;
        trans.buf = &gCmdRxBuffer[0U];
        trans.count = 1;
        transferOK = UART_read(gUartHandle[CONFIG_UART0], &trans);

        correct_input = (gCmdRxBuffer[0] == 'y') || (gCmdRxBuffer[0] == 'Y');
    }

    /* changing timeout back to default*/
    trans.timeout = TIMEOUT_UART_TESTER;

    return;
}

void tester_command(char *str)
{
    if (manual_testing)
    {
        DebugP_log("%s\r\n", str);
        wait_for_user_uart_input();
        return;
    }

    if (enableLog)
        DebugP_log("\r\nSending command!!");

    /* Send command*/
    gNumBytesWritten = 0U;
    trans.buf = &gCmdTxBuffer[0U];
    strncpy(trans.buf, str, strlen(str));
    trans.count = CMD_SIZE;
    transferOK = UART_write(gUartHandle[TESTER_UART], &trans);

    /* Quickly wait to read ack. No debug prints here*/
    gNumBytesRead = 0U;
    trans.buf = &gCmdRxBuffer[0U];
    trans.count = RSP_SIZE;
    transferOK = UART_read(gUartHandle[TESTER_UART], &trans);

    uint8_t ind;
    if (enableLog)
    {
        DebugP_log("\r\nReceived response!! (%d %d) : ", transferOK, trans.status);
        for (ind = 0; ind < RSP_SIZE; ind++)
        {
            DebugP_log("%c", gCmdRxBuffer[ind]);
        }
        DebugP_log("\r\n");
    }
    /*Clear TX buffer for shorter commands*/
    for (ind = 0; ind < CMD_SIZE; ind++)
    {
        gCmdTxBuffer[ind] = 0;
    }
}

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
    // todo add: qual_prd;
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
} ECAP_config_;

typedef ECAP_config_ *ECAP_config;

static ECAP_config ecap_config_ptr;

/*
    instance 0 prescaler div_1 all events disabled
    onshot - capture mode - stops at evt 1 (falling)
*/
void util_ECAP_config_reset(void)
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

    /* TBD */
    // ecap_config_ptr->qual_prd ;

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
static void ECAP_capture_event_polarity(void *args);
static void ECAP_setting_the_mode_of_capture(void *args);
static void ECAP_support_absolute_and_difference_mode_timestamp_capture(void *args);
static void ECAP_add_additional_input_trigger_sources(void *args);
static void ECAP_support_interrupt_generation_on_either_of_4_events(void *args);
static void ECAP_selection_of_soc_trigger_source(void *args);

int32_t test_ecap_cases(uint8_t in);

void util_ECAP_reset_all(void);
void util_start_capture_timestamp(uint16_t instance);
void ecap_configure(uint16_t instance);
void ecap_deconfigure_and_reset(uint16_t instance);
int16_t wait_for_ecap_interrupt(uint16_t instance);

int32_t AM263x_ECAP_BTR_001(uint16_t instance);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */
/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void test_main(void *args)
{
    /* Open drivers */
    Drivers_open();
    Board_driversOpen();

    /* Todo : Add tester init, menu functionalities.*/
    enableLog = true;

    UNITY_BEGIN();

    RUN_TEST(ECAP_setEventPrescalerApiCheck, 3330, NULL);

    RUN_TEST(ECAP_selection_of_sync_in_source, 3401, NULL);

    RUN_TEST(ECAP_high_resolution_functions_for_ecap, 3402, NULL);
    RUN_TEST(ECAP_capture_event_polarity, 3403, NULL);
    RUN_TEST(ECAP_setting_the_mode_of_capture, 3404, NULL);
    RUN_TEST(ECAP_support_absolute_and_difference_mode_timestamp_capture, 3405, NULL);
    RUN_TEST(ECAP_add_additional_input_trigger_sources, 3406, NULL);
    RUN_TEST(ECAP_support_interrupt_generation_on_either_of_4_events, 3407, NULL);
    RUN_TEST(ECAP_selection_of_soc_trigger_source, 7906, NULL);


    UNITY_END();

    /* Close drivers */
    Board_driversClose();
    Drivers_close();

    return;
}

/* Unity framework required information */
void setUp(void)
{
    util_ECAP_reset_all();
}

void tearDown(void)
{
    util_ECAP_reset_all();
}

void util_ECAP_reset_all(void)
{
    for (uint32_t instance = 0; instance < 10; instance++)
    {
        SOC_generateEcapReset(instance);
    }
}

/* resets ecap. sets the config_ptr to given configuration */
void ecap_configure(uint16_t instance)
{
    uint32_t base = ecap_base_addr[instance];

    /* Disables time stamp capture */
	ECAP_disableTimeStampCapture(base);
    /* Stops Time stamp counter */
	ECAP_stopCounter(base);


    /* Disable ,clear all capture flags and interrupts */
    /* TODO : Add Signal Monitoring events */
    ECAP_disableInterrupt(base,
		(ECAP_ISR_SOURCE_CAPTURE_EVENT_1  |
		 ECAP_ISR_SOURCE_CAPTURE_EVENT_2  |
		 ECAP_ISR_SOURCE_CAPTURE_EVENT_3  |
		 ECAP_ISR_SOURCE_CAPTURE_EVENT_4  |
		 ECAP_ISR_SOURCE_COUNTER_OVERFLOW |
		 ECAP_ISR_SOURCE_COUNTER_PERIOD   |
		 ECAP_ISR_SOURCE_COUNTER_COMPARE));
	ECAP_clearInterrupt(base,
	  	(ECAP_ISR_SOURCE_CAPTURE_EVENT_1 |
		ECAP_ISR_SOURCE_CAPTURE_EVENT_2  |
		ECAP_ISR_SOURCE_CAPTURE_EVENT_3  |
		ECAP_ISR_SOURCE_CAPTURE_EVENT_4  |
		ECAP_ISR_SOURCE_COUNTER_OVERFLOW |
		ECAP_ISR_SOURCE_COUNTER_PERIOD   |
		ECAP_ISR_SOURCE_COUNTER_COMPARE));

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
    }
    /* Sets a phase shift value count */
    ECAP_setPhaseShiftCount(base,ecap_config_ptr->phase_shift_count);

    if((ecap_config_ptr->load_counter_en) == 0)
    {
        /* Disable counter loading with phase shift vaSelect sync-in event to be the sync-out signal pass
throughlue */
        ECAP_disableLoadCounter(base);
    }
    uint16_t syncout_source = (ecap_config_ptr->sync_out_mode) >> 3;
    /* Configures Sync out signal mode */
    ECAP_setSyncOutMode(base,syncout_source);

    /* Configures emulation mode */
    ECAP_setEmulationMode(base,ecap_config_ptr->emu_mode);


    /* Set up the source for sync-in pulse */
    ECAP_setSyncInPulseSource(base, ecap_config_ptr->sync_in_source);

    /* Resetting the Modulo counter, Counter, Event flags */
    ECAP_resetCounters(base);
    /* Start Time stamp counter for instance  and enable Timestamp capture separately.*/
    /* ECAP_startCounter(base); */
    /* ECAP_enableTimeStampCapture(base); */
}

void util_start_capture_timestamp(uint16_t instance)
{
    uint32_t base = ecap_base_addr[instance];
    ECAP_startCounter(base);
    ECAP_enableTimeStampCapture(base);
}

/* resets given instance */
void ecap_deconfigure_and_reset(uint16_t instance)
{
    SOC_generateEcapReset(instance);
    util_ECAP_config_reset();
    ecap_configure(instance);
}

int16_t wait_for_ecap_interrupt(uint16_t instance)
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


/* Testcase 1 - Check the ECAP_setEventPrescaler API */
static void ECAP_setEventPrescalerApiCheck(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_ecap_cases(0), 0);
}

static void ECAP_selection_of_sync_in_source(void *args)
{
    UNITY_TEST_IGNORE("","TODO");
}
static void ECAP_high_resolution_functions_for_ecap(void *args)
{
    UNITY_TEST_IGNORE("","TODO");
}
static void ECAP_capture_event_polarity(void *args)
{
    UNITY_TEST_IGNORE("","TODO");
}
static void ECAP_setting_the_mode_of_capture(void *args)
{
    UNITY_TEST_IGNORE("","TODO");
}
static void ECAP_support_absolute_and_difference_mode_timestamp_capture(void *args)
{
    UNITY_TEST_IGNORE("","TODO");
}
static void ECAP_add_additional_input_trigger_sources(void *args)
{
    UNITY_TEST_IGNORE("","TODO");
}
static void ECAP_support_interrupt_generation_on_either_of_4_events(void *args)
{
    if(enableLog)
    DebugP_log("Test : Interrupt generation on various events\r\n");
    TEST_ASSERT_EQUAL_INT32(test_ecap_cases(1), 0);
}

static void ECAP_selection_of_soc_trigger_source(void *args)
{
    uint16_t instance, soc_trigger_source, actual_value;
    for (instance = 0; instance < NUM_ECAP_INSTANCES; instance++)
    {
        uint32_t ecap_base = ecap_base_addr[instance];

        for (soc_trigger_source = ECAP_CAP_MODE_SOC_TRIGGER_SRC_CEVT1;
             soc_trigger_source <= ECAP_APWM_MODE_SOC_TRIGGER_SRC_DISABLED;
             soc_trigger_source++)
        {
            if (soc_trigger_source >= ECAP_APWM_MODE_SOC_TRIGGER_SRC_PRD)
                actual_value = soc_trigger_source - (uint16_t)ECAP_APWM_MODE_SOC_TRIGGER_SRC_PRD;
            else
                actual_value = soc_trigger_source;
            ECAP_selectSocTriggerSource(ecap_base, soc_trigger_source);
            TEST_ASSERT_EQUAL_INT32((HW_RD_REG32(ecap_base + CSL_ECAP_ECCTL0) & CSL_ECAP_ECCTL0_SOCEVTSEL_MASK) >> CSL_ECAP_ECCTL0_SOCEVTSEL_SHIFT, actual_value);
        }
    }
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
    if(enableLog)
    DebugP_log("------------- instance : %d-------------\r\n", instance);

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
    ecap_deconfigure_and_reset(instance);

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

        if(wait_for_ecap_interrupt(instance) == false)
        {
            errors++;
            DebugP_log("wait failed for source : %x\r\n", interrupt_source[iter]);
        }
        else
        {
            interrupt_event_flag = ECAP_getInterruptSource(base);
            if(interrupt_event_flag != interrupt_source[iter])
            {
                errors++;
                DebugP_log("incorrect interrupt source failed\r\n");
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

int32_t test_ecap_cases(uint8_t in)
{
    int32_t failcount = 0;
    uint16_t instance;

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
            if((HW_RD_REG16(base + CSL_ECAP_ECCTL1) &
                             CSL_ECAP_ECCTL1_PRESCALE_MASK) >>
                                CSL_ECAP_ECCTL1_PRESCALE_SHIFT != val)
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
        }
    }

    if (enableLog)
        DebugP_log("%d is the failcount FYI\r\n", failcount);
    if (failcount != 0)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}
