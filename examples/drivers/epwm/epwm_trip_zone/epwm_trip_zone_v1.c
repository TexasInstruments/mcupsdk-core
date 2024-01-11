/*
 *  Copyright (C) 2024 Texas Instruments Incorporated
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

//
// Included Files
//
#include <stdarg.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/AddrTranslateP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/HwiP.h>
#include <drivers/epwm.h>
#include <drivers/gpio.h>
#include <drivers/pinmux.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/*
This example configures EHRPWM0 and EHRPWM1 as follows

EHRPWM0 has TZ0 as one shot trip source
EHRPWM1 has TZ0 as cycle by cycle trip source
Initially tie TZ0 high. During the test, monitor EHRPWM0 or EHRPWM1 outputs on a scope. Pull TZ0 low to see the effect.

EHRPWM0_A is on GPIO_A
EHRPWM1_A is on GPIO_B

The TZ-Event is defined such that EHRPWM0_A will undergo a One-Shot Trip and EHRPWM1_A will undergo a Cycle-By-Cycle Trip.
 */

/* Output channel - A or B */
#define APP_EPWM_OUTPUT_CH              (EPWM_OUTPUT_CH_A)
/* Duty Cycle of PWM output signal in % - give value from 0 to 100 */
#define APP_EPWM_DUTY_CYCLE             (25U)
/* Frequency of PWM output signal in Hz - 1 KHz is selected */
#define APP_EPWM_OUTPUT_FREQ            (1U * 1000U)

/* TB frequency in Hz - so that /4 divider is used */
#define APP_EPWM_TB_FREQ                (CONFIG_EPWM0_FCLK / 4U)
/*
 *  PRD value - this determines the period
 *  PRD = (TBCLK/PWM FREQ) / 2
 *  /2 is added becasue up&down counter is selected. So period is 2 times
 */
#define APP_EPWM_PRD_VAL                ((APP_EPWM_TB_FREQ / APP_EPWM_OUTPUT_FREQ) / 2U)
/*
 *  COMPA value - this determines the duty cycle
 *  COMPA = (PRD - ((dutycycle * PRD) / 100)
 */
#define APP_EPWM_COMPA_VAL              (APP_EPWM_PRD_VAL - ((APP_EPWM_DUTY_CYCLE * \
                                            APP_EPWM_PRD_VAL) / 100U))

/* Global variables and objects */
static HwiP_Object       gEpwmHwiObject_1;
static HwiP_Object       gEpwmHwiObject_2;
static SemaphoreP_Object gEpwmSyncSemObject;


/* Function Prototypes */
static void App_epwmIntrISR_1(void *handle);
static void App_epwmIntrISR_2(void *handle);
static void App_epwmConfig(uint32_t epwmBaseAddr,
                           uint32_t epwmCh,
                           uint32_t epwmFuncClk);
static void App_pinmuxConfig(void);
static void App_manageEpwmTzReg(void(*tzRegFunction)(uint32_t, va_list), uint32_t epwmBaseAddr, ...);
static void App_tzEventStatusClear(uint32_t epwmBaseAddr, va_list args);
static void App_tzTripEventEnable(uint32_t epwmBaseAddr, va_list args);
static void App_tzIntrEnable(uint32_t epwmBaseAddr, va_list args);


/* Globals */
uint32_t  gEpwm1TZIntCount;
uint32_t  gEpwm2TZIntCount;
uint32_t gEpwm1BaseAddr;
uint32_t gEpwm2BaseAddr;


void epwm_trip_zone_main(void *args)
{
    int32_t             status;
    HwiP_Params         hwiPrms_1;
    HwiP_Params         hwiPrms_2;


    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    /* Setting gpio output to high intially */
    GPIO_setDirMode(GPIO_OUTPUT_BASE_ADDR, GPIO_OUTPUT_PIN, GPIO_OUTPUT_DIR);
    GPIO_pinWriteHigh(GPIO_OUTPUT_BASE_ADDR, GPIO_OUTPUT_PIN);

    /* Get Address of ePWM */
    gEpwm1BaseAddr = (uint32_t)AddrTranslateP_getLocalAddr(CONFIG_EPWM0_BASE_ADDR);
    gEpwm2BaseAddr = (uint32_t)AddrTranslateP_getLocalAddr(CONFIG_EPWM1_BASE_ADDR);

    /* Clearing One Shot tripzone flag */
    App_manageEpwmTzReg(App_tzEventStatusClear, gEpwm1BaseAddr, EPWM_TZ_STS_FLG_OST | EPWM_TZ_STS_FLG_INT);

    /* Clearing Cycle by Cycle tripzone flag */
    App_manageEpwmTzReg(App_tzEventStatusClear, gEpwm2BaseAddr, EPWM_TZ_STS_FLG_CBC | EPWM_TZ_STS_FLG_INT);

    DebugP_log("EPWM Trip Zone Test Started ...\r\n");

    status = SemaphoreP_constructBinary(&gEpwmSyncSemObject, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    /* For EPWM0 */

    /* Register & enable interrupt */
    HwiP_Params_init(&hwiPrms_1);
    /* Integrate with Syscfg */
    hwiPrms_1.intNum      = CONFIG_EPWM0_TRIP_INTR;
    hwiPrms_1.isPulse     = CONFIG_EPWM0_INTR_IS_PULSE;
    hwiPrms_1.callback    = &App_epwmIntrISR_1;
        status              = HwiP_construct(&gEpwmHwiObject_1, &hwiPrms_1);
    DebugP_assert(status == SystemP_SUCCESS);

    /* For EPWM1 */

    /* Register & enable interrupt */
    HwiP_Params_init(&hwiPrms_2);
    hwiPrms_2.intNum      = CONFIG_EPWM1_TRIP_INTR;
    hwiPrms_2.isPulse     = CONFIG_EPWM1_INTR_IS_PULSE;
    hwiPrms_2.callback    = &App_epwmIntrISR_2;
    status                = HwiP_construct(&gEpwmHwiObject_2, &hwiPrms_2);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Configure PWM */
    App_epwmConfig(gEpwm1BaseAddr, APP_EPWM_OUTPUT_CH, CONFIG_EPWM0_FCLK);
    App_epwmConfig(gEpwm2BaseAddr, APP_EPWM_OUTPUT_CH, CONFIG_EPWM1_FCLK);
    App_pinmuxConfig(); /* Configure GPIO pin to EHRPWM0_TZn_IN0 */

    gEpwm1TZIntCount=0;

    /* Clearing One Shot tripzone flag */
    App_manageEpwmTzReg(App_tzEventStatusClear, gEpwm1BaseAddr, EPWM_TZ_STS_FLG_OST | EPWM_TZ_STS_FLG_INT);

    /* Clearing Cycle By Cycle tripzone flag */
    App_manageEpwmTzReg(App_tzEventStatusClear, gEpwm2BaseAddr, EPWM_TZ_STS_FLG_CBC | EPWM_TZ_STS_FLG_INT);

    /* Enabling one shot trip */
    App_manageEpwmTzReg(App_tzTripEventEnable, gEpwm1BaseAddr, EPWM_TZ_EVENT_ONE_SHOT, 0);

    /* Enabling cycle by cycle trip */
    App_manageEpwmTzReg(App_tzTripEventEnable, gEpwm2BaseAddr, EPWM_TZ_EVENT_CYCLE_BY_CYCLE, 0);

    while(gEpwm1TZIntCount < 10)
    {
        /* enabling trip input */
        GPIO_pinWriteLow(GPIO_OUTPUT_BASE_ADDR, GPIO_OUTPUT_PIN);

        /* waiting on the trip zone interrupt */
        SemaphoreP_pend(&gEpwmSyncSemObject, SystemP_WAIT_FOREVER);
        DebugP_log("TZ OST interrupt hit %d times!!\r\n", gEpwm1TZIntCount);

        /* disabling trip input */
        GPIO_pinWriteHigh(GPIO_OUTPUT_BASE_ADDR, GPIO_OUTPUT_PIN);

        /* Clearing trip zone flags for next interrupt trigger */
        App_manageEpwmTzReg(App_tzEventStatusClear, gEpwm1BaseAddr, EPWM_TZ_STS_FLG_OST | EPWM_TZ_STS_FLG_INT);

        /* Introducing some delay for the epwm to recover */
        ClockP_usleep(1920 * 10); /* time period = 1920us */
    }



    gEpwm2TZIntCount=0;

    /* Clearing Cycle By Cycle tripzone flag */
    App_manageEpwmTzReg(App_tzEventStatusClear, gEpwm2BaseAddr, EPWM_TZ_STS_FLG_CBC | EPWM_TZ_STS_FLG_INT);
    while(gEpwm2TZIntCount < 10)
    {
        /* enabling trip input */
        GPIO_pinWriteLow(GPIO_OUTPUT_BASE_ADDR, GPIO_OUTPUT_PIN);

        /* waiting on the trip zone interrupt */
        SemaphoreP_pend(&gEpwmSyncSemObject, SystemP_WAIT_FOREVER);
        DebugP_log("TZ CBC interrupt hit %d times!!\r\n", gEpwm2TZIntCount);

        /* disabling trip input */
        GPIO_pinWriteHigh(GPIO_OUTPUT_BASE_ADDR, GPIO_OUTPUT_PIN);

        /* Clearing trip zone flags for next interrupt trigger */
        App_manageEpwmTzReg(App_tzEventStatusClear, gEpwm2BaseAddr, EPWM_TZ_STS_FLG_CBC | EPWM_TZ_STS_FLG_INT);

        /* Introducing some delay for the epwm to recover */
        ClockP_usleep(960 * 10); /* time period = 960us */
    }


    HwiP_destruct(&gEpwmHwiObject_1);
    HwiP_destruct(&gEpwmHwiObject_2);
    SemaphoreP_destruct(&gEpwmSyncSemObject);

    DebugP_log("EPWM Trip Zone Test Passed!!\r\n");
    DebugP_log("All tests have passed!!\r\n");

    Board_driversClose();
    Drivers_close();

}


static void App_epwmIntrISR_1(void *handle)
{
    volatile uint16_t status;
    gEpwm1TZIntCount++;

    SOC_allowEpwmTzReg(0, TRUE); /* Enabling writes to trip zone registers */
    status = EPWM_tzEventStatus(gEpwm1BaseAddr, EPWM_TZ_STS_FLG_INT); /* Get interrupt status */
    SOC_allowEpwmTzReg(0, FALSE); /* Disabling writes to trip zone registers */

    if(status)
    {
        SemaphoreP_post(&gEpwmSyncSemObject);
        EPWM_tzEventStatusClear(gEpwm1BaseAddr, EPWM_TZ_STS_FLG_INT);/* Clear trip zone flag */
    }

    return;
}

static void App_epwmIntrISR_2(void *handle)
{
    volatile uint16_t status;
    gEpwm2TZIntCount++;

    SOC_allowEpwmTzReg(1, TRUE); /* Enabling writes to trip zone registers */
    status = EPWM_tzEventStatus(gEpwm2BaseAddr, EPWM_TZ_STS_FLG_INT); /* Get interrupt status */
    SOC_allowEpwmTzReg(1, FALSE); /* Disabling writes to trip zone registers */

    if(status)
    {
        SemaphoreP_post(&gEpwmSyncSemObject);
        EPWM_tzEventStatusClear(gEpwm2BaseAddr, EPWM_TZ_STS_FLG_INT);/* Clear trip zone flag */
    }

    return;
}

static void App_epwmConfig(uint32_t epwmBaseAddr,
                           uint32_t epwmCh,
                           uint32_t epwmFuncClk)
{
    EPWM_AqActionCfg  aqConfig;

    /* Configure Time base submodule */
    EPWM_tbTimebaseClkCfg(epwmBaseAddr, APP_EPWM_TB_FREQ, epwmFuncClk);
    EPWM_tbPwmFreqCfg(epwmBaseAddr, APP_EPWM_TB_FREQ, APP_EPWM_OUTPUT_FREQ,
        EPWM_TB_COUNTER_DIR_UP_DOWN,
            EPWM_SHADOW_REG_CTRL_ENABLE);
    EPWM_tbSyncDisable(epwmBaseAddr);
    EPWM_tbSetSyncOutMode(epwmBaseAddr, EPWM_TB_SYNC_OUT_EVT_SYNCIN);
    EPWM_tbSetEmulationMode(epwmBaseAddr, EPWM_TB_EMU_MODE_FREE_RUN);

    /* Configure counter compare submodule */
    EPWM_counterComparatorCfg(epwmBaseAddr, EPWM_CC_CMP_A,
        APP_EPWM_COMPA_VAL, EPWM_SHADOW_REG_CTRL_ENABLE,
            EPWM_CC_CMP_LOAD_MODE_CNT_EQ_ZERO, TRUE);

    EPWM_counterComparatorCfg(epwmBaseAddr, EPWM_CC_CMP_B,
        APP_EPWM_COMPA_VAL, EPWM_SHADOW_REG_CTRL_ENABLE,
            EPWM_CC_CMP_LOAD_MODE_CNT_EQ_ZERO, TRUE);

    /* Configure Action Qualifier Submodule */
    aqConfig.zeroAction = EPWM_AQ_ACTION_DONOTHING;
    aqConfig.prdAction = EPWM_AQ_ACTION_DONOTHING;
    aqConfig.cmpAUpAction = EPWM_AQ_ACTION_HIGH;
    aqConfig.cmpADownAction = EPWM_AQ_ACTION_LOW;
    aqConfig.cmpBUpAction = EPWM_AQ_ACTION_HIGH;
    aqConfig.cmpBDownAction = EPWM_AQ_ACTION_LOW;
    EPWM_aqActionOnOutputCfg(epwmBaseAddr, epwmCh, &aqConfig);

    /* Configure Dead Band Submodule */
    EPWM_deadbandBypass(epwmBaseAddr);

    /* Configure Chopper Submodule */
    EPWM_chopperEnable(epwmBaseAddr, FALSE);

    /* Configure trip zone Submodule */
    if(epwmBaseAddr == gEpwm1BaseAddr)
    {
        App_manageEpwmTzReg(App_tzIntrEnable, epwmBaseAddr, EPWM_TZ_EVENT_ONE_SHOT); /* Enable one shot trip zone interrupt */
    }
    else
    {
        App_manageEpwmTzReg(App_tzIntrEnable, epwmBaseAddr, EPWM_TZ_EVENT_CYCLE_BY_CYCLE); /* Enable cycle by cycle trip zone interrupt */
    }
}

static void App_pinmuxConfig()
{

    /* Pinmux configuration to use TZ0 for ePWM trips */
    static Pinmux_PerCfg_t App_PinMuxMainDomainCfg[] = {
            /* TZ0 pin config */

        {
            PIN_GPMC0_AD2,
            ( PIN_MODE(3) | PIN_INPUT_ENABLE | PIN_PULL_DISABLE )
        },


        {PINMUX_END, PINMUX_END}
    };

    Pinmux_config(App_PinMuxMainDomainCfg, PINMUX_DOMAIN_ID_MAIN);
}

/* Wrapper function for managing trip zone action functions */
static void App_manageEpwmTzReg(void(*tzRegFunction)(uint32_t, va_list), uint32_t epwmBaseAddr, ...)
{

    va_list args;
    va_start(args, epwmBaseAddr);
    uint32_t epwmInstance;

    if(epwmBaseAddr == gEpwm1BaseAddr)
    {
        epwmInstance = 0;
    }
    else
    {
        epwmInstance = 1;
    }

    SOC_allowEpwmTzReg(epwmInstance, TRUE);
    tzRegFunction(epwmBaseAddr, args);
    SOC_allowEpwmTzReg(epwmInstance, FALSE);
    va_end(args);
}

static void App_tzEventStatusClear(uint32_t epwmBaseAddr, va_list args)
{
    uint32_t tzFlag = va_arg(args, uint32_t);

    EPWM_tzEventStatusClear(epwmBaseAddr, tzFlag); /* Clear trip zone flag */
}
static void App_tzTripEventEnable(uint32_t epwmBaseAddr, va_list args)
{
    uint32_t tzEvent = va_arg(args, uint32_t);
    uint32_t pinNum = va_arg(args, uint32_t);

    EPWM_tzTripEventEnable(epwmBaseAddr, tzEvent, pinNum); /* Enable Trip zone event */
}
static void App_tzIntrEnable(uint32_t epwmBaseAddr, va_list args)
{
    uint32_t tzEvent = va_arg(args, uint32_t);

    EPWM_tzIntrEnable(epwmBaseAddr, tzEvent); /* Enable trip zone interrupt */
}