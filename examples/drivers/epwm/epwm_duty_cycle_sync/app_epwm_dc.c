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

#include <stdint.h>
#include <math.h>
#include <drivers/hw_include/hw_types.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/AddrTranslateP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/HwiP.h>
#include <drivers/epwm.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"

#include "epwm_drv_aux.h"
#include "epwm_dc.h"

/* EPWM functional clock */
/* Functional clock is the same for all EPWMs */
#define APP_EPWM_FCLK_FREQ              ( CONFIG_EPWM0_FCLK )

/* EPWM functional clock dividers */
#define APP_EPWM_FCLK_HSPCLKDIV         ( 0x0 ) /* EPWM_TBCTL:HSPCLKDIV, High-Speed Time-base Clock Prescale Bits */
#define APP_EPWM_FCLK_CLKDIV            ( 0x0 ) /* EPWM_TBCTL:CLKDIV, Time-base Clock Prescale Bits */

/* Frequency of PWM output signal in Hz */
#define APP_EPWM_OUTPUT_FREQ_4K         ( 1U * 4000U )
#define APP_EPWM_OUTPUT_FREQ_8K         ( 1U * 8000U )
#define APP_EPWM_OUTPUT_FREQ_16K        ( 1U * 16000U )
#define APP_EPWM_OUTPUT_FREQ            ( APP_EPWM_OUTPUT_FREQ_4K ) /* EPWM output frequency */

/* Initial Duty Cycle of PWM output, % of EPWM period, 0.0 to 100.0 */
#define APP_EPWM0_DUTY_CYCLE            ( 50.0f )
#define APP_EPWM1_DUTY_CYCLE            ( 50.0f )
#define APP_EPWM2_DUTY_CYCLE            ( 50.0f )

/* Phase loaded on Sync In event, % of TB period, 0.0 to 100.0 */
#define APP_EPWM0_TB_PHASE              ( 0.0f )
#define APP_EPWM1_TB_PHASE              ( 0.0f )
#define APP_EPWM2_TB_PHASE              ( 0.0f )

/* Deadband RED/FED timer counts */
#define APP_EPWM_DB_RED_COUNT           ( 250 )     /* 1 usec @ 250 MHz, 250/250e6*1e6=1 */
#define APP_EPWM_DB_FED_COUNT           ( 250 )     /* 1 usec @ 250 MHz, 250/250e6*1e6=1 */

/* Sinusoid parameters */   
#define SIN_FREQ                        ( 50.0 )    /* sinusoid frequency */
#define SIN_AMP                         ( 0.9 )     /* sinusoid amplitude */

/* Global variables and objects */
static HwiP_Object       gEpwm0HwiObject;
static SemaphoreP_Object gEpwmSyncSemObject;

/* EPWM ISR */
static void AppEpwm_epwmIntrISR(void *handle);

/* EPWM base addresses */
uint32_t gEpwm0BaseAddr;
uint32_t gEpwm1BaseAddr;
uint32_t gEpwm2BaseAddr;

/* EPWM objects */
EPwmObj_t gEpwm0Obj;
EPwmObj_t gEpwm1Obj;
EPwmObj_t gEpwm2Obj;

/* EPWM handles */
Epwm_Handle hEpwm0;
Epwm_Handle hEpwm1;
Epwm_Handle hEpwm2;

/* Sinusoid variables */
float gSinAmp = SIN_AMP;    /* sinusoid amplitude */
float gSinFreq = SIN_FREQ;  /* sinusoid frequency */
uint32_t gSampCnt = 0;      /* sinusoid current sample count */

#define SET_SWSYNC_THR              ( 10U ) /* number of EPWM ISRs before SW force sync */
#define SET_UPDOUTISR_THR           ( 20U ) /* number of EPWM ISRs before enabling output generation in ISR */
#define APP_EPWM_RUN_TIME           ( 60U ) /* APP run time in seconds */

volatile Bool gUpdOutIsr = FALSE;   /* Flag for updating PWM output in ISR */
volatile uint32_t gEpwmIsrCnt = 0;  /* EPWM ISR count */

/* Debug */
uint32_t gLoopCnt = 0;              /* main loop count */

void epwm_duty_cycle_sync_main(void *args)
{
    int32_t         status;
    HwiP_Params     hwiPrms;
    EPwmCfgPrms_t   epwmCfgPrms;
   
    Drivers_open();

    DebugP_log("EPWM Duty Cycle Sync Test Started ...\r\n");
    DebugP_log("Please refer to the EXAMPLES_DRIVERS_EPWM_DUTY_CYCLE_SYNC example \
user guide for the test setup to probe the EPWM signals.\r\n");
    DebugP_log("App will wait for 60 seconds (using PWM period ISR) ...\r\n");

    /* Debug, configure GPIO */
    GPIO_setDirMode(CONFIG_GPIO0_BASE_ADDR, CONFIG_GPIO0_PIN, CONFIG_GPIO0_DIR);
    GPIO_pinWriteHigh(CONFIG_GPIO0_BASE_ADDR, CONFIG_GPIO0_PIN);
    GPIO_pinWriteLow(CONFIG_GPIO0_BASE_ADDR, CONFIG_GPIO0_PIN);

    /* Address translate */
    gEpwm0BaseAddr = (uint32_t)AddrTranslateP_getLocalAddr(CONFIG_EPWM0_BASE_ADDR);
    gEpwm1BaseAddr = (uint32_t)AddrTranslateP_getLocalAddr(CONFIG_EPWM1_BASE_ADDR);
    gEpwm2BaseAddr = (uint32_t)AddrTranslateP_getLocalAddr(CONFIG_EPWM2_BASE_ADDR);

    /* Create semaphore */
    status = SemaphoreP_constructBinary(&gEpwmSyncSemObject, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Register & enable EPWM0 interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = CONFIG_EPWM0_INTR;
    hwiPrms.callback    = &AppEpwm_epwmIntrISR;
    hwiPrms.args        = 0;
    hwiPrms.isPulse     = CONFIG_EPWM0_INTR_IS_PULSE;
    status              = HwiP_construct(&gEpwm0HwiObject, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Configure EPWM0 */
    epwmCfgPrms.epwmId = EPWM_ID_0;
    epwmCfgPrms.epwmBaseAddr = gEpwm0BaseAddr;
    epwmCfgPrms.epwmFclkFreq = APP_EPWM_FCLK_FREQ;
    epwmCfgPrms.hspClkDiv = APP_EPWM_FCLK_HSPCLKDIV;
    epwmCfgPrms.clkDiv = APP_EPWM_FCLK_CLKDIV;
    epwmCfgPrms.epwmOutFreq = APP_EPWM_OUTPUT_FREQ;
    epwmCfgPrms.epwmDutyCycle = APP_EPWM0_DUTY_CYCLE;
    epwmCfgPrms.epwmTbCounterDir = EPWM_TB_COUNTER_DIR_UP_DOWN;
    epwmCfgPrms.cfgTbSyncIn = TRUE;
    epwmCfgPrms.tbPhsValue = APP_EPWM0_TB_PHASE;
    epwmCfgPrms.tbSyncInCounterDir = EPWM_TB_COUNTER_DIR_UP;
    epwmCfgPrms.cfgTbSyncOut = TRUE;
    epwmCfgPrms.tbSyncOutMode = EPWM_TB_SYNC_OUT_EVT_SYNCIN;
    epwmCfgPrms.aqCfg.zeroAction = EPWM_AQ_ACTION_DONOTHING;
    epwmCfgPrms.aqCfg.prdAction = EPWM_AQ_ACTION_DONOTHING;
    epwmCfgPrms.aqCfg.cmpAUpAction = EPWM_AQ_ACTION_HIGH;
    epwmCfgPrms.aqCfg.cmpADownAction = EPWM_AQ_ACTION_LOW;
    epwmCfgPrms.aqCfg.cmpBUpAction = EPWM_AQ_ACTION_DONOTHING;
    epwmCfgPrms.aqCfg.cmpBDownAction = EPWM_AQ_ACTION_DONOTHING;
    epwmCfgPrms.cfgDb = TRUE;
    epwmCfgPrms.dbCfg.inputMode = EPWM_DB_IN_MODE_A_RED_A_FED;
    epwmCfgPrms.dbCfg.outputMode = EPWM_DB_OUT_MODE_A_RED_B_FED;
    epwmCfgPrms.dbCfg.polaritySelect = EPWM_DB_POL_SEL_ACTV_HIGH_COMPLEMENTARY;
    epwmCfgPrms.dbCfg.risingEdgeDelay = APP_EPWM_DB_RED_COUNT;
    epwmCfgPrms.dbCfg.fallingEdgeDelay = APP_EPWM_DB_FED_COUNT;
    epwmCfgPrms.cfgEt = TRUE;
    epwmCfgPrms.intSel = EPWM_ET_INTR_EVT_CNT_EQ_ZRO;
    epwmCfgPrms.intPrd = EPWM_ET_INTR_PERIOD_FIRST_EVT;
    hEpwm0 = epwmInit(&epwmCfgPrms, &gEpwm0Obj);
    DebugP_assert(hEpwm0 != NULL);   

    /* Configure EPWM1 */
    epwmCfgPrms.epwmId = EPWM_ID_1;
    epwmCfgPrms.epwmBaseAddr = gEpwm1BaseAddr;
    epwmCfgPrms.epwmFclkFreq = APP_EPWM_FCLK_FREQ;
    epwmCfgPrms.hspClkDiv = APP_EPWM_FCLK_HSPCLKDIV;
    epwmCfgPrms.clkDiv = APP_EPWM_FCLK_CLKDIV;
    epwmCfgPrms.epwmOutFreq = APP_EPWM_OUTPUT_FREQ;
    epwmCfgPrms.epwmDutyCycle = APP_EPWM1_DUTY_CYCLE;
    epwmCfgPrms.epwmTbCounterDir = EPWM_TB_COUNTER_DIR_UP_DOWN;
    epwmCfgPrms.cfgTbSyncIn = TRUE;
    epwmCfgPrms.tbPhsValue = APP_EPWM1_TB_PHASE;
    epwmCfgPrms.tbSyncInCounterDir = EPWM_TB_COUNTER_DIR_UP;
    epwmCfgPrms.cfgTbSyncOut = TRUE;
    epwmCfgPrms.tbSyncOutMode = EPWM_TB_SYNC_OUT_EVT_SYNCIN;
    epwmCfgPrms.aqCfg.zeroAction = EPWM_AQ_ACTION_DONOTHING;
    epwmCfgPrms.aqCfg.prdAction = EPWM_AQ_ACTION_DONOTHING;
    epwmCfgPrms.aqCfg.cmpAUpAction = EPWM_AQ_ACTION_HIGH;
    epwmCfgPrms.aqCfg.cmpADownAction = EPWM_AQ_ACTION_LOW;
    epwmCfgPrms.aqCfg.cmpBUpAction = EPWM_AQ_ACTION_DONOTHING;
    epwmCfgPrms.aqCfg.cmpBDownAction = EPWM_AQ_ACTION_DONOTHING;
    epwmCfgPrms.cfgDb = TRUE;
    epwmCfgPrms.dbCfg.inputMode = EPWM_DB_IN_MODE_A_RED_A_FED;
    epwmCfgPrms.dbCfg.outputMode = EPWM_DB_OUT_MODE_A_RED_B_FED;
    epwmCfgPrms.dbCfg.polaritySelect = EPWM_DB_POL_SEL_ACTV_HIGH_COMPLEMENTARY;
    epwmCfgPrms.dbCfg.risingEdgeDelay = APP_EPWM_DB_RED_COUNT;
    epwmCfgPrms.dbCfg.fallingEdgeDelay = APP_EPWM_DB_FED_COUNT;
    epwmCfgPrms.cfgEt = FALSE;
    hEpwm1 = epwmInit(&epwmCfgPrms, &gEpwm1Obj);
    DebugP_assert(hEpwm1 != NULL);   

    /* Configure EPWM2 */
    epwmCfgPrms.epwmId = EPWM_ID_2;
    epwmCfgPrms.epwmBaseAddr = gEpwm2BaseAddr;
    epwmCfgPrms.epwmFclkFreq = APP_EPWM_FCLK_FREQ;
    epwmCfgPrms.hspClkDiv = APP_EPWM_FCLK_HSPCLKDIV;
    epwmCfgPrms.clkDiv = APP_EPWM_FCLK_CLKDIV;
    epwmCfgPrms.epwmOutFreq = APP_EPWM_OUTPUT_FREQ;
    epwmCfgPrms.epwmDutyCycle = APP_EPWM2_DUTY_CYCLE;
    epwmCfgPrms.epwmTbCounterDir = EPWM_TB_COUNTER_DIR_UP_DOWN;
    epwmCfgPrms.cfgTbSyncIn = TRUE;
    epwmCfgPrms.tbPhsValue = APP_EPWM2_TB_PHASE;
    epwmCfgPrms.tbSyncInCounterDir = EPWM_TB_COUNTER_DIR_UP;
    epwmCfgPrms.cfgTbSyncOut = TRUE;
    epwmCfgPrms.tbSyncOutMode = EPWM_TB_SYNC_OUT_EVT_DISABLE;
    epwmCfgPrms.aqCfg.zeroAction = EPWM_AQ_ACTION_DONOTHING;
    epwmCfgPrms.aqCfg.prdAction = EPWM_AQ_ACTION_DONOTHING;
    epwmCfgPrms.aqCfg.cmpAUpAction = EPWM_AQ_ACTION_HIGH;
    epwmCfgPrms.aqCfg.cmpADownAction = EPWM_AQ_ACTION_LOW;
    epwmCfgPrms.aqCfg.cmpBUpAction = EPWM_AQ_ACTION_DONOTHING;
    epwmCfgPrms.aqCfg.cmpBDownAction = EPWM_AQ_ACTION_DONOTHING;
    epwmCfgPrms.cfgDb = TRUE;
    epwmCfgPrms.dbCfg.inputMode = EPWM_DB_IN_MODE_A_RED_A_FED;
    epwmCfgPrms.dbCfg.outputMode = EPWM_DB_OUT_MODE_A_RED_B_FED;
    epwmCfgPrms.dbCfg.polaritySelect = EPWM_DB_POL_SEL_ACTV_HIGH_COMPLEMENTARY;
    epwmCfgPrms.dbCfg.risingEdgeDelay = APP_EPWM_DB_RED_COUNT;
    epwmCfgPrms.dbCfg.fallingEdgeDelay = APP_EPWM_DB_FED_COUNT;
    epwmCfgPrms.cfgEt = FALSE;
    hEpwm2 = epwmInit(&epwmCfgPrms, &gEpwm2Obj);
    DebugP_assert(hEpwm2 != NULL);   
    
    /* Wait to force SW sync.
       Delay unnecessary, included so unsync'd EPWM signals can be observed at startup. */
    while (gEpwmIsrCnt < SET_SWSYNC_THR)
        ;

    /* Force SW sync for EPWM0.
       Other PWMs will be sync'd through chain. 
       SW sync simulates EPWM0SYNCI. */
    EPWM_tbTriggerSwSync(gEpwm0BaseAddr);

    /* Wait to enable output update in EPWM ISR.
       Delay unnecessary, included so sync'd signals can be observed at startup. */
    while (gEpwmIsrCnt < SET_UPDOUTISR_THR)
        ;

    /* Enable output in ISR */
    gUpdOutIsr = TRUE;

    while (gEpwmIsrCnt < (APP_EPWM_RUN_TIME * APP_EPWM_OUTPUT_FREQ))
    {
        SemaphoreP_pend(&gEpwmSyncSemObject, SystemP_WAIT_FOREVER);
        gLoopCnt++;
    }

    /* Disable and clear interrupts for EPWM0 */
    EPWM_etIntrDisable(gEpwm0BaseAddr); /* Disable interrupts */
    EPWM_etIntrClear(gEpwm0BaseAddr);   /* Clear pending interrupts */

    /* Destroy HWI & semaphore */
    HwiP_destruct(&gEpwm0HwiObject);
    SemaphoreP_destruct(&gEpwmSyncSemObject);

    DebugP_log("EPWM Duty Cycle Sync Test Passed!!\r\n");
    DebugP_log("All tests have passed!!\r\n");

    Drivers_close();
}

/* EPWM ISR */
static void AppEpwm_epwmIntrISR(void *args)
{
    uint16_t status;
    float sinVal0, sinVal1, sinVal2;  /* sin values */

    /* debug, show ISR timing */
    GPIO_pinWriteHigh(CONFIG_GPIO0_BASE_ADDR, CONFIG_GPIO0_PIN);
    /* debug, increment ISR count */
    gEpwmIsrCnt++;
    
    status = EPWM_etIntrStatus(gEpwm0BaseAddr);
    if (status & EPWM_ETFLG_INT_MASK)
    {
        if (gUpdOutIsr == TRUE)
        {
            /*
                Compute next sinusoid values:
                    A*sin(2*pi*fo*n/fs)
                    A*sin(2*pi*fo*n/fs + 2*pi/3)
                    A*sin(2*pi*fo*n/fs - 2*pi/3)                
            */
            sinVal0 = gSinAmp*sinf(2*M_PI*gSinFreq*gSampCnt/APP_EPWM_OUTPUT_FREQ);
            sinVal1 = gSinAmp*sinf(2*M_PI*gSinFreq*gSampCnt/APP_EPWM_OUTPUT_FREQ + 2*M_PI/3);
            sinVal2 = gSinAmp*sinf(2*M_PI*gSinFreq*gSampCnt/APP_EPWM_OUTPUT_FREQ - 2*M_PI/3);
            gSampCnt++;

            /* Update EPWM output */
            epwmUpdateOut(hEpwm0, sinVal0);
            epwmUpdateOut(hEpwm1, sinVal1);
            epwmUpdateOut(hEpwm2, sinVal2);
        }

        SemaphoreP_post(&gEpwmSyncSemObject);
        EPWM_etIntrClear(gEpwm0BaseAddr);
    }

    /* debug, show ISR timing */
    GPIO_pinWriteLow(CONFIG_GPIO0_BASE_ADDR, CONFIG_GPIO0_PIN);

    return;
}
