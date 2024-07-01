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

#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/HwiP.h>
#include <drivers/epwm.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/*
 * Example Description
 * This example configures EPWM0, EPWM1 and EPWM2 to produce signal with
 * independent modulation on EPWMxA and EPWMxB.
 *
 * The compare values CMPA and CMPB are modified within the ePWM's ISR.
 * The TB counter is in up count mode for this example.
 *
 * During the test, monitor EPWM0, EPWM1, and/or epwEPWM2 outputs
 * on an oscilloscope.
 *
 * On AM263x CC/ AM263Px CC with HSEC Dock,
 * Probe the following on the HSEC pins
 *  - EPWM 0A/0B : 49 / 51
 *  - EPWM 1A/1B : 53 / 55
 *  - EPWM 2A/2B : 50 / 52
 *
 * On AM263x LP/ AM263Px LP,
 * Probe the following on boosterpack
 *  - EPWM 0A/0B : J4 11 / J8 59
 *  - EPWM 1A/1B : J2 37 / J2 38
 *  - EPWM 2A/2B : J2 39 / J2 40
 */

/* FIXME : To be removed after syscfg integration */
#define APP_INT_IS_PULSE    (1U)

#define EPWM0_TIMER_TBPRD  2000  // Period register
#define EPWM0_MAX_CMPA     1950
#define EPWM0_MIN_CMPA       50
#define EPWM0_MAX_CMPB     1950
#define EPWM0_MIN_CMPB       50

#define EPWM1_TIMER_TBPRD  2000  // Period register
#define EPWM1_MAX_CMPA     1950
#define EPWM1_MIN_CMPA       50
#define EPWM1_MAX_CMPB     1950
#define EPWM1_MIN_CMPB       50

#define EPWM2_TIMER_TBPRD  2000  // Period register
#define EPWM2_MAX_CMPA      950
#define EPWM2_MIN_CMPA       50
#define EPWM2_MAX_CMPB     1950
#define EPWM2_MIN_CMPB     1050

#define EPWM_CMP_UP           1
#define EPWM_CMP_DOWN         0

/* Globals */
typedef struct
{
    uint32_t epwmModule;
    uint16_t epwmCompADirection;
    uint16_t epwmCompBDirection;
    uint16_t epwmTimerIntCount;
    uint16_t epwmMaxCompA;
    uint16_t epwmMinCompA;
    uint16_t epwmMaxCompB;
    uint16_t epwmMinCompB;
} epwmInfo;

epwmInfo epwm0Info;
epwmInfo epwm1Info;
epwmInfo epwm2Info;

volatile uint16_t compAVal, compBVal;

static HwiP_Object  gEpwmHwiObject_0;
static HwiP_Object  gEpwmHwiObject_1;
static HwiP_Object  gEpwmHwiObject_2;

/* Function Prototypes */
void initEPWM0(void);
void initEPWM1(void);
void initEPWM2(void);

void updateCompare(epwmInfo*);

static void App_epwmIntrISR_0(void *handle);
static void App_epwmIntrISR_1(void *handle);
static void App_epwmIntrISR_2(void *handle);

/* variable to hold base address of EPWM that is used */
uint32_t gEpwm0Base = CONFIG_EPWM0_BASE_ADDR;
uint32_t gEpwm1Base = CONFIG_EPWM1_BASE_ADDR;
uint32_t gEpwm2Base = CONFIG_EPWM2_BASE_ADDR;

void epwm_up_aq_main(void *args)
{

    int32_t  status;
    /* Initialising a Interrupt parameter */
    HwiP_Params  hwiPrms_0;
    HwiP_Params  hwiPrms_1;
    HwiP_Params  hwiPrms_2;

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    uint32_t epwmsMask = (1U << 0U) | (1U << 1U) | (1U << 2U);

    /* Check the syscfg for configurations */

    DebugP_log("EPWM Action Qualifier Module Test Started ...\r\n");
    DebugP_log("EPWM Action Qualifier Module using UP-Count mode Example runs for 30 Secs \r\n");

    /* Disabling tbclk sync for EPWMs 0-2 for configurations */
    SOC_setMultipleEpwmTbClk(epwmsMask, FALSE);

    initEPWM0();
    initEPWM1();
    initEPWM2();

    /* For EPWM 0 */
    /* Register & enable interrupt */
    HwiP_Params_init(&hwiPrms_0);
    hwiPrms_0.intNum      = CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_0;
    hwiPrms_0.isPulse     = APP_INT_IS_PULSE;
    hwiPrms_0.callback    = &App_epwmIntrISR_0;
    status              =   HwiP_construct(&gEpwmHwiObject_0, &hwiPrms_0);
    DebugP_assert(status == SystemP_SUCCESS);

    /* For EPWM 1 */
    /* Register & enable interrupt */
    HwiP_Params_init(&hwiPrms_1);
    hwiPrms_1.intNum      = CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_1;
    hwiPrms_1.isPulse     = APP_INT_IS_PULSE;
    hwiPrms_1.callback    = &App_epwmIntrISR_1;
    status              = HwiP_construct(&gEpwmHwiObject_1, &hwiPrms_1);
    DebugP_assert(status == SystemP_SUCCESS);

    /* For EPWM 2 */
    /* Register & enable interrupt */
    HwiP_Params_init(&hwiPrms_2);
    hwiPrms_2.intNum      = CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_2;
    hwiPrms_2.isPulse     = APP_INT_IS_PULSE;
    hwiPrms_2.callback    = &App_epwmIntrISR_2;
    status              = HwiP_construct(&gEpwmHwiObject_2, &hwiPrms_2);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Clear any pending interrupts if any */
    EPWM_clearEventTriggerInterruptFlag(gEpwm0Base);
    EPWM_clearEventTriggerInterruptFlag(gEpwm1Base);
    EPWM_clearEventTriggerInterruptFlag(gEpwm2Base);

    /* Enabling tbclk sync for EPWMs 0-2 after configurations */
    SOC_setMultipleEpwmTbClk(epwmsMask, TRUE);

    ClockP_sleep(30);

    DebugP_log("EPWM Action Qualifier Module Test Passed!!\r\n");
    DebugP_log("All Tests have Passed!!");

    Board_driversClose();
    Drivers_close();
}

static void App_epwmIntrISR_0(void *handle)
{

    /* Update the CMPA and CMPB values */
    updateCompare(&epwm0Info);

    /* Clear any pending interrupts if any */
    EPWM_clearEventTriggerInterruptFlag(gEpwm0Base);
}

static void App_epwmIntrISR_1(void *handle)
{

    /* Update the CMPA and CMPB values */
    updateCompare(&epwm1Info);

    /* Clear any pending interrupts if any */
    EPWM_clearEventTriggerInterruptFlag(gEpwm1Base);
}

static void App_epwmIntrISR_2(void *handle)
{

    /* Update the CMPA and CMPB values */
    updateCompare(&epwm2Info);

    /* Clear any pending interrupts if any */
    EPWM_clearEventTriggerInterruptFlag(gEpwm2Base);
}

/* initEPWM0 Information */
void initEPWM0()
{
   /* Note: This example uses to keep track
      of the direction the CMPA/CMPB values are
      moving, the min and max allowed values and
      a pointer to the correct ePWM registers
   */

    /* Start by increasing CMPA & CMPB */
    epwm0Info.epwmCompADirection = EPWM_CMP_UP;
    epwm0Info.epwmCompBDirection = EPWM_CMP_UP;

    /* Clear interrupt counter */
    epwm0Info.epwmTimerIntCount = 0;

    /* Set base as ePWM0 */
    epwm0Info.epwmModule = gEpwm0Base;

    /* Setup min/max CMPA/CMP values */
    epwm0Info.epwmMaxCompA = EPWM0_MAX_CMPA;
    epwm0Info.epwmMinCompA = EPWM0_MIN_CMPA;
    epwm0Info.epwmMaxCompB = EPWM0_MAX_CMPB;
    epwm0Info.epwmMinCompB = EPWM0_MIN_CMPB;
}

/* initEPWM1 Information */
void initEPWM1()
{
    /* Note: This example uses to keep track
       of the direction the CMPA/CMPB values are
       moving, the min and max allowed values and
       a pointer to the correct ePWM registers
    */

     /* Start by increasing CMPA & CMPB */
    epwm1Info.epwmCompADirection = EPWM_CMP_UP;
    epwm1Info.epwmCompBDirection = EPWM_CMP_UP;

    /* Clear interrupt counter */
    epwm1Info.epwmTimerIntCount = 0;

    /* Set base as ePWM1 */
    epwm1Info.epwmModule = gEpwm1Base;

    /* Setup min/max CMPA/CMP values */
    epwm1Info.epwmMaxCompA = EPWM1_MAX_CMPA;
    epwm1Info.epwmMinCompA = EPWM1_MIN_CMPA;
    epwm1Info.epwmMaxCompB = EPWM1_MAX_CMPB;
    epwm1Info.epwmMinCompB = EPWM1_MIN_CMPB;
}

/* initEPWM2 Information */
void initEPWM2()
{
    /* Note: This example uses to keep track
       of the direction the CMPA/CMPB values are
       moving, the min and max allowed values and
       a pointer to the correct ePWM registers
    */

     /* Start by increasing CMPA & CMPB */
    epwm2Info.epwmCompADirection = EPWM_CMP_UP;
    epwm2Info.epwmCompBDirection = EPWM_CMP_UP;

    /* Clear interrupt counter */
    epwm2Info.epwmTimerIntCount = 0;

    /* Set base as ePWM2 */
    epwm2Info.epwmModule = gEpwm2Base;

    /* Setup min/max CMPA/CMP values */
    epwm2Info.epwmMaxCompA = EPWM2_MAX_CMPA;
    epwm2Info.epwmMinCompA = EPWM2_MIN_CMPA;
    epwm2Info.epwmMaxCompB = EPWM2_MAX_CMPB;
    epwm2Info.epwmMinCompB = EPWM2_MIN_CMPB;
}

/* updateCompare - Update the compare values for the specified EPWM */
void updateCompare(epwmInfo *epwm_info)
{

   /* Every 10'th interrupt, change the CMPA/CMPB values */
   if(epwm_info->epwmTimerIntCount == 10)
   {
       epwm_info->epwmTimerIntCount = 0;
       compAVal = EPWM_getCounterCompareValue(epwm_info->epwmModule,
                                              EPWM_COUNTER_COMPARE_A);
       compBVal = EPWM_getCounterCompareValue(epwm_info->epwmModule,
                                              EPWM_COUNTER_COMPARE_B);

       /* If we were increasing CMPA, check to see if
          we reached the max value.  If not, increase CMPA
          else, change directions and decrease CMPA
       */
       if(epwm_info->epwmCompADirection == EPWM_CMP_UP)
       {
           if(compAVal < epwm_info->epwmMaxCompA)
           {
               EPWM_setCounterCompareValue(epwm_info->epwmModule,
                                           EPWM_COUNTER_COMPARE_A, ++compAVal);
           }
           else
           {
               epwm_info->epwmCompADirection = EPWM_CMP_DOWN;
               EPWM_setCounterCompareValue(epwm_info->epwmModule,
                                           EPWM_COUNTER_COMPARE_A, --compAVal);
           }
       }

       /* If we were decreasing CMPA, check to see if
          we reached the min value.  If not, decrease CMPA
          else, change directions and increase CMPA
       */
       else
       {
           if(compAVal == epwm_info->epwmMinCompA)
           {
               epwm_info->epwmCompADirection = EPWM_CMP_UP;
               EPWM_setCounterCompareValue(epwm_info->epwmModule,
                                           EPWM_COUNTER_COMPARE_A, ++compAVal);

           }
           else
           {
               EPWM_setCounterCompareValue(epwm_info->epwmModule,
                                           EPWM_COUNTER_COMPARE_A, --compAVal);
           }
       }

       /* If we were increasing CMPB, check to see if
          we reached the max value.  If not, increase CMPB
          else, change directions and decrease CMPB
       */
       if(epwm_info->epwmCompBDirection == EPWM_CMP_UP)
       {
           if(compBVal < epwm_info->epwmMaxCompB)
           {
               EPWM_setCounterCompareValue(epwm_info->epwmModule,
                                           EPWM_COUNTER_COMPARE_B, ++compBVal);
           }
           else
           {
               epwm_info->epwmCompBDirection = EPWM_CMP_DOWN;
               EPWM_setCounterCompareValue(epwm_info->epwmModule,
                                           EPWM_COUNTER_COMPARE_B, --compBVal);

           }
       }

       /* If we were decreasing CMPB, check to see if
          we reached the min value.  If not, decrease CMPB
          else, change directions and increase CMPB
       */
       else
       {
           if(compBVal == epwm_info->epwmMinCompB)
           {
               epwm_info->epwmCompBDirection = EPWM_CMP_UP;
               EPWM_setCounterCompareValue(epwm_info->epwmModule,
                                           EPWM_COUNTER_COMPARE_B, ++compBVal);
           }
           else
           {
               EPWM_setCounterCompareValue(epwm_info->epwmModule,
                                           EPWM_COUNTER_COMPARE_B, --compBVal);
           }
       }
   }

   /* Increment interrupt count if < 10 */
   else
   {
      epwm_info->epwmTimerIntCount++;
   }
   return;
}
