/*
 *  Copyright (C) 2022 Texas Instruments Incorporated
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
#include <drivers/dac.h>
#include <drivers/epwm.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/ClockP.h>
#include <drivers/edma.h>
#include "ti_drivers_config.h"
#include <kernel/dpl/TimerP.h>


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

static void DAC_generate_a_constant_voltage_from_dac(void *args);
static void DAC_generate_a_square_wave_synchronized_with_sysclk(void *args);
static void DAC_dac_offset_adjustment(void *args);
static void DAC_reset_the_module_and_call_the_calibration_functions(void *args);
static void DAC_synchronization_with_epwm(void *args);
static void DAC_dma_latency(void *args);
static void DAC_setGetShadowValueApiCheck(void *args);

int32_t test_dac_cases(uint8_t in);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* for DMA */

/* Value for A count*/
#define EDMA_TEST_A_COUNT           (2U)
/* Value for B count */
#define EDMA_TEST_B_COUNT           (1U)
/* Value for C count */
#define EDMA_TEST_C_COUNT           (1U)
/* Event queue to be used  */
#define EDMA_TEST_EVT_QUEUE_NO      (0U)

#define EDMA_TEST_BUFFER_SIZE             (EDMA_TEST_A_COUNT * EDMA_TEST_B_COUNT * EDMA_TEST_C_COUNT)


/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void test_main(void *args)
{
    /* Open drivers */
    Drivers_open();
    Board_driversOpen();

    UNITY_BEGIN();

    RUN_TEST(DAC_generate_a_constant_voltage_from_dac,  3194, NULL);
    RUN_TEST(DAC_generate_a_square_wave_synchronized_with_sysclk, 3195, NULL);
    RUN_TEST(DAC_dac_offset_adjustment, 3196, NULL);
    RUN_TEST(DAC_reset_the_module_and_call_the_calibration_functions, 3197, NULL);
    RUN_TEST(DAC_synchronization_with_epwm, 3198, NULL);
    RUN_TEST(DAC_dma_latency, 3199, NULL);
    RUN_TEST(DAC_setGetShadowValueApiCheck, 3331, NULL);

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

static void DAC_generate_a_constant_voltage_from_dac(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_dac_cases(1), 0);
}
static void DAC_generate_a_square_wave_synchronized_with_sysclk(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_dac_cases(2), 0);
}
static void DAC_dac_offset_adjustment(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_dac_cases(3), 0);
}
static void DAC_reset_the_module_and_call_the_calibration_functions(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_dac_cases(4), 0);
}
static void DAC_synchronization_with_epwm(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_dac_cases(5), 0);
}
static void DAC_dma_latency(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_dac_cases(6), 0);
}
static void DAC_setGetShadowValueApiCheck(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_dac_cases(7), 0);
}


void util_inittimer()
{
    HW_WR_REG32(CSL_ICSSM0_INTERNAL_U_BASE + 0x2E000, 0x551);
    HW_WR_REG32(CSL_ICSSM0_INTERNAL_U_BASE + 0x2E000 + 0x10, 0);
}

void util_init_dac(uint32_t base, DAC_ReferenceVoltage voltageReference, DAC_LoadMode loadMode, uint16_t signal)
{
    if(voltageReference==DAC_REF_VREF){
        DAC_setReferenceVoltage(base, DAC_REF_VREF);
    }else{
        DAC_setReferenceVoltage(base, DAC_REF_VDDA);
    }

    if(loadMode == DAC_LOAD_SYSCLK){
        DAC_setLoadMode(base, DAC_LOAD_SYSCLK);
    }else{
        DAC_setPWMSyncSignal(base, signal);
        DAC_setLoadMode(base, DAC_LOAD_PWMSYNC);
    }
    DAC_enableOutput(base);
    ClockP_usleep(10);
}

void util_deinit_dac(uint32_t base)
{

    //Unlock CONTROLSS_CTRL
    HW_WR_REG32(CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_LOCK0_KICK0 ,0x01234567);
    HW_WR_REG32(CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_LOCK0_KICK1 ,0xFEDCBA8);

    if(base==CSL_CONTROLSS_DAC0_U_BASE)
    {
        HW_WR_REG32(CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_DAC_RST ,0x07);
        HW_WR_REG32(CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_DAC_RST ,0x00);
    }

}

void util_pulse_dac(uint32_t base, float usec)
{
    util_init_dac(base, DAC_REF_VREF, DAC_LOAD_SYSCLK, 0);
    DAC_setShadowValue(base, 0);
    DAC_setShadowValue(base, 4095);

    ClockP_usleep(usec);
    DAC_setShadowValue(base, 0);

    util_deinit_dac(base);
}



/* Testcase 7 - Check the DAC_setShadowValue and DAC_getShadowValue API */
int32_t setGetShadowValueApiCheck(uint32_t base) //DAC_setGetShadowValueApiCheck
{
    uint16_t val = 0xFF;

    /* Call the DAC_setShadowValue API */
    DAC_setShadowValue(base, val);

    /* Check if the value was written correctly */
    if(DAC_getShadowValue(base) != val)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}
int32_t DACBTR001(uint32_t base) //DAC_generate_a_constant_voltage_from_dac
{
    uint16_t val;
    uint8_t errors=0;

    util_pulse_dac(base, 50);

    util_init_dac(base, DAC_REF_VREF, DAC_LOAD_SYSCLK, 0);

    for(val=0;val<4096;val++)
    {
        DAC_setShadowValue(base, val);

        if(val!=DAC_getActiveValue(base))
        {
            errors++;
        }
        //Verify DAC OUT pin voltage

        if(val!=DAC_getShadowValue(base))
        {
            errors++;
        }

        ClockP_usleep(100);
    }

    util_deinit_dac(base);

    ClockP_sleep(3);

    if(errors!=0)
    {
        return 1;
    }
    else
    {
        return 0;
    }

}

int32_t DACBTR002(uint32_t base) //DAC_generate_a_square_wave_synchronised_with_sysclk
{
    uint32_t pulsecount=10000;
    uint8_t errors=0;

    util_pulse_dac(base, 50);


    util_init_dac(base, DAC_REF_VREF, DAC_LOAD_SYSCLK, 0);

    while(pulsecount > 0)
    {
        /* Set output to max voltage */
        DAC_setShadowValue(base, 4095);
        if(4095!=DAC_getActiveValue(base))
        {
            errors++;
        }
        /* Set output to min voltage */
        DAC_setShadowValue(base, 0);
        if(0!=DAC_getActiveValue(base))
        {
            errors++;
        }
        pulsecount--;
    }


    util_deinit_dac(base);

    ClockP_sleep(3);

    if(errors!=0)
    {
        return 1;
    }
    else
    {
        return 0;
    }

}

int32_t DACBTR003(uint32_t base) //DAC_dac_offset_adjustment
{

    int8_t val;
    uint8_t errors=0;

    util_pulse_dac(base, 50);


    util_init_dac(base, DAC_REF_VREF, DAC_LOAD_SYSCLK, 0);

    for(val=-128;val<127;val++)
    {
        DAC_setOffsetTrim(base, val);

        if(val!=DAC_getOffsetTrim(base))
        {
            errors++;
        }

        ClockP_usleep(1000);
        DAC_setShadowValue(base,2000);
    }

    util_deinit_dac(base);

    ClockP_sleep(3);

    if(errors!=0)
    {
        return 1;
    }
    else
    {
        return 0;
    }

}

int32_t DACBTR004(uint32_t base)    //DAC_reset_the_module_and_call_the_callibration_functions
{
    uint8_t errorInDeinit =0;
    uint8_t errorInInit =0;

    //Unlock CONTROLSS_CTRL
    HW_WR_REG32(CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_LOCK0_KICK0 ,0x01234567);
    HW_WR_REG32(CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_LOCK0_KICK1 ,0xFEDCBA8);

    //De-init DAC
    if(base==CSL_CONTROLSS_DAC0_U_BASE)
    {
        HW_WR_REG32(CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_DAC_RST ,0x07);
    }

    // de-initiated state.

    // Initiate DAC
    if(base==CSL_CONTROLSS_DAC0_U_BASE)
    {
        HW_WR_REG32(CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_DAC_RST ,0x00);
    }

    // Initiated state.
    // Default initialisation : SYSCLK LoadMode, Voltage Reference DAC_REF_VREF

    DAC_setShadowValue(base, 10);
    if(DAC_getActiveValue(base) != 10) errorInInit++;

    util_deinit_dac(base);

    //EFUSE_OVERRIDE_DAC_TRIM 0x50D8 0860
    //  28:16 DAC_TRIM
    //  2:0 OVERRIDE
    //EFUSE_OVERRIDE_DAC_CFG 0x50D8 0864
    //  24 ASYNC_MODE_EN
    //  17:16 IBIAS_CFG
    //  2:0 OVERRIDE
    if(errorInInit > 0 || errorInDeinit >0){
        return 1;
    }
    else{
        return 0;
    }

}

void util_deinit_epwms()
{
    //Debug_Printf(0, "\n util_deinit_epwms");
    uint8_t i;

    for(i=0;i<32;i++)
    {
        HW_WR_REG32(CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_ETPWM0_RST + (i*4),0x07);
        HW_WR_REG32(CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_ETPWM0_RST + (i*4),0x00);
    }
}

void util_configPWM(uint32_t base, uint16_t period, uint16_t cmpa, uint16_t cmpb, uint32_t counterMode)
{
    //Debug_Printf(0, "\n EPWM %d configured", (base-CSL_CONTROLSS_G0_EPWM0_U_BASE)/0x1000);
    //config 3
    // Configure Time Base counter Clock - Write to CLKDIV and HSPCLKDIV bit
    EPWM_setClockPrescaler(base, EPWM_CLOCK_DIVIDER_128, EPWM_HSCLOCK_DIVIDER_14);
    //Debug_Printf(0, "\nmain: CTL Configured");


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
    //Debug_Printf(0, "\nmain: CTR Configured");

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


    //Debug_Printf(0, "\nmain: AQ Configured");
    //End of Config 3

}




int32_t DACITR001(uint32_t base)    //DAC_synchronization_with_epwm
{
    //  initially setting DAC output value to 0.
    util_init_dac(base,DAC_REF_VREF, DAC_LOAD_SYSCLK, 0);
    DAC_setShadowValue(base,0);
    util_deinit_dac(base);

    // variable to set period value in TB
    uint16_t periodValue = 12000;

    // initiating a variable to count errors.
    uint32_t errors = 0;


    // iterating DAC sync check for all the epwms
    for (uint16_t i = 0; i<32; i++){
        //errors = 0;
        uint16_t DAC_outputValue =100*i;

        //  Configuring the EPWMx for creating a sync signal on Time base reaching period value.
        util_configPWM(CSL_CONTROLSS_G0_EPWM0_U_BASE + i*(0x1000) , periodValue, 100/2, 100/4, EPWM_COUNTER_MODE_UP);
        EPWM_setEmulationMode(CSL_CONTROLSS_G0_EPWM0_U_BASE + i*(0x1000),EPWM_EMULATION_STOP_AFTER_FULL_CYCLE);

        // initiating DAC to be loaded with PWMSync Signal and which EPWM's sync signal
        util_init_dac(base, DAC_REF_VREF, DAC_LOAD_PWMSYNC, i+1);

        //Setting Shadow Value and checking if its updated at Active.
        DAC_setShadowValue(base, 4095);
        if(DAC_getActiveValue(base)==4095){
            errors++;
        }

        // Set the EPWM SYNC PER Signal source. Here its set when TB Counter reaches Period.
        HRPWM_setSyncPulseSource(CSL_CONTROLSS_G0_EPWM0_U_BASE +i*(0x1000), HRPWM_PWMSYNC_SOURCE_PERIOD);

        // Wait while the Counter reaches Period
        volatile uint16_t *reg = (uint16_t *)( CSL_CONTROLSS_G0_EPWM0_U_BASE + i*(0x1000) + CSL_EPWM_TBCTR);
        while(*reg!= periodValue);

        DAC_setShadowValue(base, DAC_outputValue);
        if(DAC_getActiveValue(base) != DAC_outputValue){
            errors++;
        }

        util_deinit_epwms();
        util_deinit_dac(base);
    }
    if(errors!=0)
        {
            return 1;
        }
        else
        {
            return 0;
        }
}



/* Test to measure DMA latency for DAC path */
/* Tests covered : TTR001 */
int32_t DACTTR001(uint32_t base)    //DAC_dma_latency
{
    uint32_t errors = 0;
    uint32_t timerCount =0;

    util_init_dac(base,DAC_REF_VREF, DAC_LOAD_SYSCLK, 0);

    static uint8_t gEdmaTestSrcBuff[EDMA_TEST_BUFFER_SIZE] __attribute__((aligned(CacheP_CACHELINE_ALIGNMENT)));

    /*
     * Initiating variables for EDMA transfer
     */

    uint32_t            baseAddr, regionId;
    int32_t             testStatus = SystemP_SUCCESS;
    uint32_t            loopCnt = 0;
    uint8_t            *srcBuffPtr;
    EDMACCPaRAMEntry    edmaParam;
    uint32_t            dmaCh, tcc, param;


    /*
     * Timer Parameters and Setup
     */
    TimerP_Params timerParams;

    /* setup timer but dont start it */
    TimerP_Params_init(&timerParams);
    timerParams.inputPreScaler    = 1u;
    timerParams.inputClkHz        = 250u*1000u*1000u;
    timerParams.periodInUsec      = 10u*1000u;
    timerParams.oneshotMode       = 0;
    timerParams.enableOverflowInt = 1;
    TimerP_setup(CONFIG_TIMER0_BASE_ADDR, &timerParams);



    baseAddr = EDMA_getBaseAddr(gEdmaHandle[0]);
    // DebugP_assert(baseAddr != 0);

    regionId = EDMA_getRegionId(gEdmaHandle[0]);
    // DebugP_assert(regionId < SOC_EDMA_NUM_REGIONS);

    dmaCh = EDMA_RESOURCE_ALLOC_ANY;
    testStatus = EDMA_allocDmaChannel(gEdmaHandle[0], &dmaCh);
    // DebugP_assert(testStatus == SystemP_SUCCESS);

    tcc = EDMA_RESOURCE_ALLOC_ANY;
    testStatus = EDMA_allocTcc(gEdmaHandle[0], &tcc);
    // DebugP_assert(testStatus == SystemP_SUCCESS);

    param = EDMA_RESOURCE_ALLOC_ANY;
    testStatus = EDMA_allocParam(gEdmaHandle[0], &param);
    // DebugP_assert(testStatus == SystemP_SUCCESS);

    /*
     * Initialize the source address with a pattern
     */
    srcBuffPtr = (uint8_t *) gEdmaTestSrcBuff;
    for(loopCnt = 0U; loopCnt < EDMA_TEST_BUFFER_SIZE; loopCnt++)
    {
        srcBuffPtr[loopCnt] = (uint8_t)loopCnt*1000;
    }
    CacheP_wb((void *)srcBuffPtr, EDMA_TEST_BUFFER_SIZE, CacheP_TYPE_ALL);

    /* Request channel */
    EDMA_configureChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
         dmaCh, tcc, param, EDMA_TEST_EVT_QUEUE_NO);

    /* Program Param Set */
    EDMA_ccPaRAMEntry_init(&edmaParam);
    edmaParam.srcAddr       = (uint32_t) SOC_virtToPhy(srcBuffPtr);
    edmaParam.destAddr      = (uint32_t) SOC_virtToPhy((void *)(base + CSL_DAC_DACVALS));
    edmaParam.aCnt          = (uint16_t) EDMA_TEST_A_COUNT;
    edmaParam.bCnt          = (uint16_t) EDMA_TEST_B_COUNT;
    edmaParam.cCnt          = (uint16_t) EDMA_TEST_C_COUNT;
    edmaParam.bCntReload    = (uint16_t) EDMA_TEST_B_COUNT;
    edmaParam.srcBIdx       = (int16_t) EDMA_PARAM_BIDX(EDMA_TEST_A_COUNT);
    edmaParam.destBIdx      = (int16_t) EDMA_PARAM_BIDX(EDMA_TEST_A_COUNT);
    edmaParam.srcCIdx       = (int16_t) EDMA_TEST_A_COUNT;
    edmaParam.destCIdx      = (int16_t) EDMA_TEST_A_COUNT;
    edmaParam.linkAddr      = 0xFFFFU;
    edmaParam.srcBIdxExt    = (int8_t) EDMA_PARAM_BIDX_EXT(EDMA_TEST_A_COUNT);
    edmaParam.destBIdxExt   = (int8_t) EDMA_PARAM_BIDX_EXT(EDMA_TEST_A_COUNT);
    edmaParam.opt          |=
        (EDMA_OPT_TCINTEN_MASK | EDMA_OPT_ITCINTEN_MASK |
         ((((uint32_t)tcc) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));
    EDMA_setPaRAM(baseAddr, param, &edmaParam);

    /*
     * Transfer is done in A sync mode
     * Number of triggers required are B_COUNT * C_COUNT
     */

    /*
     * Read the timer value before the timer start
     */
    timerCount = TimerP_getCount(CONFIG_TIMER0_BASE_ADDR);
    // DebugP_log("timer count value initially is %d \n", timerCount);

    /*
     * Start and stop the timer to not down the delay caused by the timer start stop actions
     */
    TimerP_start(CONFIG_TIMER0_BASE_ADDR);
    TimerP_stop(CONFIG_TIMER0_BASE_ADDR);
    timerCount = TimerP_getCount(CONFIG_TIMER0_BASE_ADDR) - timerCount;
    // DebugP_log("time count for timer on off is %d \n", timerCount);


    timerCount = TimerP_getCount(CONFIG_TIMER0_BASE_ADDR);
    /*
     * Start the timer.
     */
    TimerP_start(CONFIG_TIMER0_BASE_ADDR);
    for(loopCnt = 0; loopCnt < (EDMA_TEST_B_COUNT * EDMA_TEST_C_COUNT); loopCnt++)
    {
        EDMA_enableTransferRegion(baseAddr, regionId, dmaCh, EDMA_TRIG_MODE_MANUAL);

        while(EDMA_readIntrStatusRegion(baseAddr, regionId, tcc) != 1);

        EDMA_clrIntrRegion(baseAddr, regionId, tcc);
    }

    TimerP_stop(CONFIG_TIMER0_BASE_ADDR);
    timerCount = TimerP_getCount(CONFIG_TIMER0_BASE_ADDR) - timerCount;
    // DebugP_log("time take for transfer is %d \n", timerCount);

    if (timerCount <= 27-5 || timerCount >= 27+5) errors++;

    /* Free channel */
    EDMA_freeChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
        dmaCh, EDMA_TRIG_MODE_MANUAL, tcc, EDMA_TEST_EVT_QUEUE_NO);

    /* Free the EDMA resources managed by driver. */
    testStatus = EDMA_freeDmaChannel(gEdmaHandle[0], &dmaCh);
    DebugP_assert(testStatus == SystemP_SUCCESS);
    testStatus = EDMA_freeTcc(gEdmaHandle[0], &tcc);
    DebugP_assert(testStatus == SystemP_SUCCESS);
    testStatus = EDMA_freeParam(gEdmaHandle[0], &param);
    DebugP_assert(testStatus == SystemP_SUCCESS);

    util_deinit_dac(base);

    if(errors != 0 )
    {
        return 1;
    }
    else
    {
        return 0;

    }


////    /* Configure RTI Timer */
////    RTI_Init();
//    util_inittimer();
//
//    /* Configure the DMA for transfer */
//    /* Initialize DMA */
//    const uint32_t tpcc_addr = (uint32_t) CSL_TPCC0_U_BASE;
//    const uint32_t ch_num = 0;
//    const uint32_t region_id = 0;
//    dma_wrap_init(tpcc_addr);
//
//    /* AB-sync transfer. Copies 'numArr' arrays, each of size 'arsize'
//    Arrays are placed at (srcAddr + i * srcAddrIncr) and copied to (desAddr + i * desAddrIncr), where i = [0, numArr) */
//    uint32_t srcAddr = (uint32_t) 0x70180000;
//    uint32_t desAddr = (uint32_t) (CSL_CONTROLSS_DAC0_U_BASE + CSL_DAC_DACCTL_ALT2_);
//    uint32_t arsize = 8; /* No of bytes to be transferred */
//    uint32_t numArr = 128; /* Total 512 bytes (8 bytes * 64 times) will be transferred */
//    int16_t srcAddrIncr = 8;
//    int16_t desAddrIncr = 0;
//    dma_wrap_cfg_AB_sync_channel(tpcc_addr, ch_num, srcAddr, desAddr, arsize, numArr, srcAddrIncr, desAddrIncr);
//
//    HW_WR_REG32(0x50000148, 1U);
//
//    /* Start the counter from 0 */
//    //RTI_StartCounter();
//
//    icss_start = HWREG(CSL_ICSSM0_INTERNAL_U_BASE + 0x2E000 + 0x10);
//
//    /* SW trigger the DMA transfer */
//    trigger_dma_transfer(tpcc_addr, ch_num);
//
//    /* Wait for the transfer to complete (polling mode) */
//    wait4_system_dma_xfer_cmpl(ch_num);
//
//    /* Stop the RTI Counter */
//    //RTI_StopCounter();
//    icss_end = HWREG(CSL_ICSSM0_INTERNAL_U_BASE + 0x2E000 + 0x10);
//
//    HWREG(CSL_TPTC00_U_BASE + 0x010C) |= 0x2;
//
//    /* Measure RTI timer value after transfer */
//    unsigned int cycles_taken = 0;
//    unsigned int frc, upc;
//    frc = HW_RD_REG32(CSL_RTI0_U_BASE + RTI_RTIFRC0);
//    upc = HW_RD_REG32(CSL_RTI0_U_BASE + RTI_RTIUC0);
//
//    /* Calculate Latency */
//    //cycles_taken = 2*((frc * 200 )+ upc);
//    cycles_taken = ((icss_end-icss_start)/2.5)-136;
//
//    printf("\nNo of cycles (ICSS Timer) taken to complete DMA transfer of 1024 Bytes from OCMC to DAC : %i", cycles_taken);
//    if(abs(cycles_taken - 1620) < 5)
//    {
//        test_pass();
//        printf("\nTransfer value within Limit");
//    }
//    else
//    {
//        test_fail();
//        printf("\nTransfer value not within Limit");
//    }
    // return 1;
}



/* Test to measure Self modifying DMA latency for DAC path */
/* Tests covered : TTR002 */
int32_t DACTTR002(uint32_t base)
{
//    /* Configure the DMA for transfer */
//    /* Initialize DMA */
//    const uint32_t tpcc_addr = (uint32_t) CSL_TPCC0_U_BASE;
//    const uint32_t ch_num = 0;
//    const uint32_t ch_num1 = 1;
//    const uint32_t region_id = 0;
//    dma_wrap_init(tpcc_addr);
//
//    /* AB-sync transfer. Copies 'numArr' arrays, each of size 'arsize'
//    Arrays are placed at (srcAddr + i * srcAddrIncr) and copied to (desAddr + i * desAddrIncr), where i = [0, numArr) */
//    uint32_t srcAddr = (uint32_t) (0x70180000);
//    uint32_t desAddr = (uint32_t) (CSL_CONTROLSS_DAC0_U_BASE + CSL_DAC_DACCTL_ALT2_);
//    uint32_t srcAddr1 = (uint32_t) (0x70180000+(2*2));
//    uint32_t desAddr1 = (uint32_t) (CSL_CONTROLSS_DAC0_U_BASE + CSL_DAC_DACVALS);
//    uint32_t arsize = 2; /* No of bytes to be transferred */
//    uint32_t numArr = 2; /* Total 8 bytes (2 bytes 4 times) will be transferred */
//    uint32_t numArr1 = 2; /* Total 8 bytes (2 bytes 4 times) will be transferred */
//    int16_t srcAddrIncr = 2;
//    int16_t desAddrIncr = 2;
//    dma_wrap_cfg_AB_sync_channel(tpcc_addr, ch_num, srcAddr, desAddr, arsize, numArr, srcAddrIncr, desAddrIncr);
//    dma_wrap_cfg_AB_sync_channel(tpcc_addr, ch_num1, srcAddr1, desAddr1, arsize, numArr, srcAddrIncr, desAddrIncr);
//    EDMA3ChainChannel(tpcc_addr, 0, 1, 0x00400000U);
//
//    HWREG(CSL_TPTC00_U_BASE + 0x010C) |= 0x2;
//    HWREG(desAddr1 + (numArr1*arsize)-4) = 0xABABABAB;
//    /* SW trigger the DMA transfer */
//    trigger_dma_transfer(tpcc_addr, ch_num);
//
//    /* Wait for the transfer to complete (polling mode) */
//    wait4_system_dma_xfer_cmpl(ch_num1);
//
//    HWREG(CSL_TPTC00_U_BASE + 0x010C) |= 0x2;
//
//    /* Measure RTI timer value after transfer */
//    unsigned int cycles_taken = 0;
//    unsigned int frc, upc;
//    frc = HW_RD_REG32(CSL_RTI0_U_BASE + RTI_RTIFRC0);
//    upc = HW_RD_REG32(CSL_RTI0_U_BASE + RTI_RTIUC0);
//
//    if(HWREG(desAddr1 + (numArr1*arsize)-4) != 0xABABABAB)
//    {
//        test_pass();
//        printf("\nDAC Self modifying DMA Test Passed");
//    }
//    else
//    {
//        test_fail();
//        printf("\nDAC Self modifying DMA Test failed");
//    }
    return 1;
}



int32_t test_dac_cases(uint8_t in)
{
    int32_t failcount=0;

    uint32_t dac_offset=0;
    uint32_t base;

    for(dac_offset=0;dac_offset<=0; dac_offset=dac_offset+0x1000)
    {

        base = CSL_CONTROLSS_DAC0_U_BASE + dac_offset;

        switch(in)
        {
            case 1:
                failcount += DACBTR001(base); //DAC_generate_a_constant_voltage_from_dac
                break;
            case 2:
                failcount += DACBTR002(base); //DAC_generate_a_square_wave_synchronised_with_sysclk
                break;
            case 3:
                failcount += DACBTR003(base); //DAC_dac_offset_adjustment
                break;
            case 4:
                failcount += DACBTR004(base); //DAC_reset_the_module_and_call_the_callibration_functions
                break;
            case 5:
                failcount += DACITR001(base); //DAC_synchronization_with_epwm
                break;
            case 6:
                failcount += DACTTR001(base); //DAC_dma_latency
                break;
            case 7:
                failcount += setGetShadowValueApiCheck(base); //DAC_setGetShadowValueApiCheck
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
