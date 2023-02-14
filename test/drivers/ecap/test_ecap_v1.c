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
#include <drivers/edma.h>
#include <drivers/adc.h>
#include <drivers/gpio.h>
#include <drivers/pinmux.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
#define TEST_ALL_INSTANCES (true)
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

#define SIZE_STRING_TEST_NAME       (70)
#define TIMEOUT_UART_MENU           (10000)
#define TIMEOUT_UART_TESTER         (10000)
#define TIMEOUT_UART_INFINITE       (SystemP_WAIT_FOREVER)

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

uint32_t adc_base_addr[5] =
{
    CSL_CONTROLSS_ADC0_U_BASE,
    CSL_CONTROLSS_ADC1_U_BASE,
    CSL_CONTROLSS_ADC2_U_BASE,
    CSL_CONTROLSS_ADC3_U_BASE,
    CSL_CONTROLSS_ADC4_U_BASE,
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

uint32_t gpio_base_addr = CSL_GPIO0_U_BASE;

/* GPIO59 -> EPWM8_A (G3) */
uint32_t gpio_pin = (PIN_EPWM8_A)>>2;

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
    /* todo add: qual_prd; */
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

    /* TBD */
    /* ecap_config_ptr->qual_prd ; */

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
static void ECAP_capture_event_polarity(void *args);
static void ECAP_capture_mode_tests(void *args);
static void ECAP_add_additional_input_trigger_sources(void *args);
static void ECAP_support_interrupt_generation_on_either_of_4_events(void *args);
static void ECAP_selection_of_soc_trigger_source(void *args);
static void ECAP_selection_of_dma_trigger_source(void *args);

int32_t test_ecap_cases(uint8_t in);

/* ECAP util functions */
void util_ecap_reset_all(void);
void util_start_capture_timestamp(uint16_t instance);
void util_ecap_configure(uint16_t instance);
void util_ecap_deconfigure_and_reset(uint16_t instance);
bool util_wait_for_ecap_interrupt(uint16_t instance);
void util_configure_ecap_input(void);
void util_activate_input_pulses(void);
bool util_compare_in_range(int value1, int value2, int delta);
int util_validate_input_events(uint16_t instance, bool check_flags);

/* ADC util funcitons */
void util_clear_adc_interrupts(void);
bool util_wait_for_adc_interrupts(void);
void util_adc_init_configure_soc_source(uint16_t adc_instance,uint16_t ecap_instance);
void util_adc_reset(uint16_t adc_instace);

/* GPIO util funcitons */
void util_configure_gpio(bool input);
void util_gpio_toggle(uint16_t times, uint32_t uSec_delay);

/* EDMA util funcitons */
void util_edma_configure(uint16_t ecap_instance);
void util_edma_deconfigure(void);

/* test implementing functions */
int32_t AM263x_ECAP_BTR_001(uint16_t instance);
int32_t AM263x_ECAP_BTR_002(uint16_t instance);
int32_t AM263x_ECAP_BTR_003(uint16_t instance);
int32_t AM263x_ECAP_BTR_004(uint16_t instance);

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
    RUN_TEST(ECAP_capture_mode_tests, 3404, NULL);
    RUN_TEST(ECAP_add_additional_input_trigger_sources, 3406, NULL);
    RUN_TEST(ECAP_support_interrupt_generation_on_either_of_4_events, 3407, NULL);
    RUN_TEST(ECAP_selection_of_soc_trigger_source, 7906, NULL);
    RUN_TEST(ECAP_selection_of_dma_trigger_source, 9476, NULL);


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
    /* Start Time stamp counter for instance  and enable Timestamp capture separately
        the following may be used
        - ECAP_startCounter(base);
        - ECAP_enableTimeStampCapture(base); */
    if(ecap_config_ptr->rearm)
    ECAP_reArm(base);

    return;
}

void util_start_capture_timestamp(uint16_t instance)
{
    uint32_t base = ecap_base_addr[instance];
    ECAP_enableTimeStampCapture(base);
    ECAP_startCounter(base);
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
bool util_wait_for_ecap_interrupt(uint16_t instance)
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
void util_clear_adc_interrupts(void)
{
    for(int adc_instance = 0; adc_instance < 5; adc_instance++)
    {
        uint32_t adc_base = adc_base_addr[adc_instance];
        ADC_clearInterruptStatus(adc_base, ADC_INT_NUMBER1);
    }
    return;
}
/* returns 1 if wait is successful i.e., interrupt flag set */
bool util_wait_for_adc_interrupts(void)
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
            DebugP_log("failed at %d\r\n",adc_instance);
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
bool wait_for_dma_interrupts(void)
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
        DebugP_log("wait failed\r\n");
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
    ClockP_usleep(500);

    return ;
}
/* resets the ADC instance */
void util_adc_reset(uint16_t adc_instance)
{
    SOC_generateAdcReset(adc_instance);
    return;
}
/* configures GPIO (declared global) as input or output */
void util_configure_gpio(bool input)
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
    if(enableLog)
    DebugP_log("toggling GPIO %d\r\n", gpio_pin);
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
static void ECAP_capture_mode_tests(void *args)
{
    if(enableLog)
    DebugP_log("Test : Capture Mode tests. Tests the mode of captures and counter reset on each capture.\r\n");
    TEST_ASSERT_EQUAL_INT32(test_ecap_cases(5), 0);
}
static void ECAP_add_additional_input_trigger_sources(void *args)
{
    if(enableLog)
    DebugP_log("Test : Input Sources to ECAP\r\n");
    TEST_ASSERT_EQUAL_INT32(test_ecap_cases(4), 0);
}
static void ECAP_support_interrupt_generation_on_either_of_4_events(void *args)
{
    if(enableLog)
    DebugP_log("Test : Interrupt generation on various events\r\n");
    TEST_ASSERT_EQUAL_INT32(test_ecap_cases(1), 0);
}
static void ECAP_selection_of_soc_trigger_source(void *args)
{
    if(enableLog)
    DebugP_log("Test : ADC_SOC event generation on various events\r\n");
    TEST_ASSERT_EQUAL_INT32(test_ecap_cases(2), 0);
}
static void ECAP_selection_of_dma_trigger_source(void *args)
{
    if(enableLog)
    DebugP_log("Test : DMA event generation on various events\r\n");
    TEST_ASSERT_EQUAL_INT32(test_ecap_cases(3), 0);
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

        if(util_wait_for_ecap_interrupt(instance) == false)
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
    util_configure_gpio(OUTPUT);

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
                    wait_success = util_wait_for_adc_interrupts();
                }
                else
                {
                    wait_success = wait_for_dma_interrupts();
                }
                if(wait_success == false)
                {
                    /*Interrupts did not occur. i.e., trigger failed*/
                    errors++;
                    DebugP_log("wait failed. for source %d \r\n", source);
                }

                /* stopping the couter to avoid accidental triggers*/
                ECAP_stopCounter(base);

                /* resetting the counter*/
                ECAP_resetCounters(base);

                /* Clear Interrupts*/
                if(event_out == soc_trigger_check)
                {
                    util_clear_adc_interrupts();
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
                    wait_success = util_wait_for_adc_interrupts();
                }
                else
                {
                    wait_success = wait_for_dma_interrupts();
                }
                if(wait_success == false)
                {
                    /*Interrupts did not occur. i.e., trigger failed*/
                    errors++;
                    DebugP_log("wait failed. for source %d \r\n", source);
                }

                /* stopping the couter to avoid accidental triggers*/
                ECAP_stopCounter(base);
                /* resetting the counter*/
                ECAP_resetCounters(base);

                /* Clear Interrupts*/
                if(event_out == soc_trigger_check)
                {
                    util_clear_adc_interrupts();
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
                    wait_success = util_wait_for_adc_interrupts();
                }
                else
                {
                    wait_success = wait_for_dma_interrupts();
                }
                if(wait_success == false)
                {
                    /*Interrupts did not occur. i.e., trigger failed*/
                    errors++;
                    DebugP_log("wait failed. for source %d \r\n", source);
                }

                /* stopping the couter to avoid accidental triggers*/
                ECAP_stopCounter(base);
                /* resetting the counter*/
                ECAP_resetCounters(base);

                /* Clear Interrupts*/
                if(event_out == soc_trigger_check)
                {
                    util_clear_adc_interrupts();
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
                wait_success = util_wait_for_adc_interrupts();
            }
            else
            {
                wait_success = wait_for_dma_interrupts();
            }
            if(wait_success == false)
            {
                /*Interrupts did not occur. i.e., trigger failed*/
                errors++;
                DebugP_log("wait failed. for source %d \r\n", source);
            }

            /* Clear Interrupts*/
            if(event_out == soc_trigger_check)
            {
                util_clear_adc_interrupts();
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
    if(enableLog)
    DebugP_log("------------- instance : %d-------------\r\n", instance);

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
    if(enableLog)
    DebugP_log("------------- instance : %d-------------\r\n", instance);

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

void util_configure_ecap_input(void)
{
    ECAP_InputCaptureSignals input = ecap_config_ptr->input;


    if( (input >= ECAP_INPUT_INPUTXBAR0) && (input <= ECAP_INPUT_INPUTXBAR31) )
    {
        /* InputXbar type. expect four pulses so the stop/wrap capture should be event 4*/
        /* input source is xbar instance */
        /* set the input event count to be 4 and timestamps to be of 1 uSec */
        util_configure_gpio(OUTPUT);

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
    return;
}
void util_activate_input_pulses(void)
{
    ECAP_InputCaptureSignals input = ecap_config_ptr->input;

    if( (input >= ECAP_INPUT_INPUTXBAR0) && (input <= ECAP_INPUT_INPUTXBAR31) )
    {
        /* input source is xbar instance */
        /* toggle the GPIO 4 times with a delay of 1 uSec*/
        int num_pulses = 4;      /* +1 to satify indexing */
        util_gpio_toggle(num_pulses, 0);
    }
    return;
}

bool util_compare_in_range(int value1, int value2, int delta)
{
    int max_value = (value1 > value2)? value1 : value2;
    int min_value = (value1 > value2)? value2 : value1;

    if((max_value - min_value) >= delta)
    {
        DebugP_log("values differ out of range\r\n");
        return false;
    }
    return true;
}

int util_validate_input_events(uint16_t instance, bool check_flags)
{
    int errors = 0;
    uint32_t ecap_base = ecap_base_addr[instance];

    ECAP_InputCaptureSignals input = ecap_config_ptr->input;

    uint16_t flags = ECAP_getInterruptSource(ecap_base);
    /* for expected events are each bits. */
    uint16_t expected_flags = (1<<((ecap_config_ptr->wrap_capture)+1))-1;

    if(check_flags && ((flags>>1)!= (expected_flags)))
    {
        errors++;
        DebugP_log("ERROR: expected number of events did not occur.\r\n");
        DebugP_log("\t\tExpected flags: %x\r\n",expected_flags);
        DebugP_log("\r\n\t\tSet flags: %x\r\n", flags>>1);
        errors++;
    }
    if( (input >= ECAP_INPUT_INPUTXBAR0) && (input <= ECAP_INPUT_INPUTXBAR31) )
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

            if(util_compare_in_range(input_params->timestamps[iter], timestamp, 50))
            {
                continue;
            }
            errors++;
            DebugP_log("\tTimeStamps for event %d donot match\r\n",iter);
            DebugP_log("\t\tExpected: %d\r\n", input_params->timestamps[iter]);
            DebugP_log("\t\tReturned: %d\r\n", timestamp);
        }

    }

    else
    {
        /* invalid input source */
        errors++;
        DebugP_log("invalid input source\r\n");
    }
    return errors;
}
/* Tests the input sources to the ECAP */
int32_t AM263x_ECAP_BTR_004(uint16_t instance)
{
    if(enableLog)
    DebugP_log("------------- instance : %d-------------\r\n", instance);

    int errors = 0;
    /* Add additional signals too*/
    ECAP_InputCaptureSignals input;

    for(input = ECAP_INPUT_INPUTXBAR0;
        input <= ECAP_INPUT_INPUTXBAR31;
        input++)
    {
        if(enableLog)
        DebugP_log("Selected input %d\r\n",input);
        /* configure ecap for the oneshot mode to capture a max of 4 events. */
        ecap_config_ptr->capture_mode_continuous = false;
        /* configure input and select input source for ecap */
        ecap_config_ptr->input = input;
        /* setting rearm configuration ecap */
        ecap_config_ptr->rearm = true;
        /* wrap capture poit will be set inside the input configuration based on the input */
        util_configure_ecap_input();
        /* configure the ecap with the settings updated */
        util_ecap_configure(instance);
        /* start the counter in the ecap and enable capture timestamps */
        util_start_capture_timestamp(instance);
        /* activate input pulses */
        util_activate_input_pulses();
        /* check the ecap cap_registers, event flags to verify the input to expected output */
        errors += util_validate_input_events(instance, true);
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
    if(enableLog)
    DebugP_log("------------- instance : %d-------------\r\n", instance);

    int errors = 0;
    ECAP_CaptureMode capture_mode;

    util_configure_gpio(OUTPUT);
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
    DebugP_log("%x is the base\r\n",gbase);
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
            util_start_capture_timestamp(instance);

            /*
            set ISR for 4 events and toggle GPIO 8 times.
            the ISR shall read 8 for continuous mode and only 4 for oneshot mode.
            ISR shall stop the ecap counter at 8th ocurrance.
            this confirms the continuous/oneshot mode.
            */

            int num_times = 8;
            util_gpio_toggle(num_times, 0);
            HwiP_destruct(&gEcapHwiObject1);

            if(capture_mode == ECAP_CONTINUOUS_CAPTURE_MODE)
            {
                if(ecap_isr_count != num_times)
                {
                    errors++;
                    DebugP_log("ISR count didn't match. Expected %d recieved %d\r\n", num_times, ecap_isr_count);
                }
            }
            else
            {
                if(ecap_isr_count != 4)
                {
                    errors++;
                    DebugP_log("ISR count didn't match. Expected 4 recieved %d\r\n", ecap_isr_count);
                }
            }
            errors += util_validate_input_events(instance, false);
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
        // case n:
        // {
        //     failcount += error returning
        // }
        }

        if(!TEST_ALL_INSTANCES)
        break;
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
