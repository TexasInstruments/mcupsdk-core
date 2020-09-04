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

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <unity.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/HwiP.h>
#include <drivers/epwm.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* Output channel - A or B */
#define APP_EPWM_OUTPUT_CH              (EPWM_OUTPUT_CH_A)
/* Duty Cycle of PWM output signal in % - give value from 0 to 100 */
#define APP_EPWM_DUTY_CYCLE_MIN         (1U)
#define APP_EPWM_DUTY_CYCLE_MAX         (99U)
#define APP_EPWM_DUTY_CYCLE             (25U)
/* Frequency of PWM output signal in Hz - 1 KHz is selected */
#define APP_EPWM_OUTPUT_FREQ_MIN        (2U)
#define APP_EPWM_OUTPUT_FREQ_MAX        (100U * 1000U)
#define APP_EPWM_OUTPUT_FREQ            (1U * 1000U)
/* APP run time in seconds */
#define APP_EPWM_RUN_TIME               (3U)

/* TB frequency in Hz - /1792 (maximum possible) divider is used */
#define APP_EPWM_TB_FREQ_MIN                (CONFIG_EPWM0_FCLK / 1792U)
/* TB frequency in Hz - /2 (minimum possible) divider is used */
#define APP_EPWM_TB_FREQ_MAX                (CONFIG_EPWM0_FCLK / 2U)
/* TB frequency in Hz - /4 divider is used */
#define APP_EPWM_TB_FREQ                    (CONFIG_EPWM0_FCLK / 4U)

/*
 *  PRD value - this determines the period
 *  PRD = (TBCLK/PWM FREQ) / 2
 *  /2 is added becasue up&down counter is selected. So period is 2 times
 */
#define APP_EPWM_PRD_VAL_MAX            ((APP_EPWM_TB_FREQ_MIN / APP_EPWM_OUTPUT_FREQ_MIN) / 2)
#define APP_EPWM_PRD_VAL_MIN            ((APP_EPWM_TB_FREQ_MAX / APP_EPWM_OUTPUT_FREQ_MAX) / 2)
#define APP_EPWM_PRD_VAL                ((APP_EPWM_TB_FREQ / APP_EPWM_OUTPUT_FREQ) / 2)

/*
 *  COMPA value - this determines the duty cycle
 *  COMPA = (PRD - ((dutycycle * PRD) / 100)
 */
#define APP_EPWM_COMPA_VAL_MIN          (APP_EPWM_PRD_VAL_MIN - ((APP_EPWM_DUTY_CYCLE_MIN * \
                                            APP_EPWM_PRD_VAL_MIN) / 100U))

#define APP_EPWM_COMPA_VAL_MAX          (APP_EPWM_PRD_VAL_MAX - ((APP_EPWM_DUTY_CYCLE_MAX * \
                                            APP_EPWM_PRD_VAL_MAX) / 100U))

#define APP_EPWM_COMPA_VAL              (APP_EPWM_PRD_VAL - ((APP_EPWM_DUTY_CYCLE * \
                                            APP_EPWM_PRD_VAL) / 100U))

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* Testcases */
static void test_epwm_max_freq_min_duty(void *args);
static void test_epwm_min_freq_max_duty(void *args);
static void test_epwm_chopper(void *args);
static void test_epwm_deadband(void *args);
static void test_epwm_tbclkcfg_api(void *args);
static void test_epwm_tbfreqcfg_api(void *args);
static void test_epwm_countercomparecfg_api(void *args);

/* Functions used in test */
static void App_epwmIntrISR(void *handle);
static void App_epwmConfigTest1(uint32_t epwmBaseAddr, uint32_t epwmCh,
                           uint32_t epwmFuncClk);
static void App_epwmConfigTest2(uint32_t epwmBaseAddr, uint32_t epwmCh,
                           uint32_t epwmFuncClk);
static void App_epwmConfigTest3(uint32_t epwmBaseAddr, uint32_t epwmCh,
                           uint32_t epwmFuncClk);
static void App_epwmConfigTest4(uint32_t epwmBaseAddr, uint32_t epwmCh,
                           uint32_t epwmFuncClk);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

static HwiP_Object       gEpwmHwiObject;
static SemaphoreP_Object gEpwmSyncSemObject;



/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void test_main(void *args)
{
    UNITY_BEGIN();

    /* Open drivers */
    Drivers_open();

    /* Run tests */
    RUN_TEST(test_epwm_max_freq_min_duty, 346, NULL);
    RUN_TEST(test_epwm_min_freq_max_duty, 347, NULL);
    RUN_TEST(test_epwm_chopper, 348, NULL);
    RUN_TEST(test_epwm_deadband, 349, NULL);
    RUN_TEST(test_epwm_tbclkcfg_api, 350, NULL);
    RUN_TEST(test_epwm_tbfreqcfg_api, 351, NULL);
    RUN_TEST(test_epwm_countercomparecfg_api, 352, NULL);

    UNITY_END();

    /* Close drivers */
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

/* Testcases */

/* Testcase 1 - Max frequency wave with min duty cycle */
static void test_epwm_max_freq_min_duty(void *args)
{
    int32_t             status;
    uint32_t            numIsrCnt = (APP_EPWM_RUN_TIME * APP_EPWM_OUTPUT_FREQ_MAX);
    HwiP_Params         hwiPrms;

    DebugP_log("EPWM max frequency min duty cycle test started ...\r\n");
    DebugP_log("App will produce signal for 3 seconds (using PWM period ISR) ...\r\n");

    status = SemaphoreP_constructCounting(&gEpwmSyncSemObject, 0, numIsrCnt);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Register & enable interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = CONFIG_EPWM0_INTR;
    hwiPrms.callback    = &App_epwmIntrISR;
    hwiPrms.isPulse     = CONFIG_EPWM0_INTR_IS_PULSE;
    status              = HwiP_construct(&gEpwmHwiObject, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Configure PWM */
    App_epwmConfigTest1(CSL_MSS_ETPWMA_U_BASE, APP_EPWM_OUTPUT_CH, CONFIG_EPWM0_FCLK);

    while(numIsrCnt > 0)
    {
        SemaphoreP_pend(&gEpwmSyncSemObject, SystemP_WAIT_FOREVER);
        numIsrCnt--;
    }

    EPWM_etIntrDisable(CSL_MSS_ETPWMA_U_BASE);
    EPWM_etIntrClear(CSL_MSS_ETPWMA_U_BASE);     /* Clear any pending interrupts if any */
    HwiP_destruct(&gEpwmHwiObject);
    SemaphoreP_destruct(&gEpwmSyncSemObject);
    return;
}

/* Testcase 2 - Min frequency wave with max duty cycle */
static void test_epwm_min_freq_max_duty(void *args)
{
    int32_t             status;
    uint32_t            numIsrCnt = (APP_EPWM_RUN_TIME * APP_EPWM_OUTPUT_FREQ_MIN);
    HwiP_Params         hwiPrms;

    DebugP_log("EPWM min frequency max duty cycle test started ...\r\n");
    DebugP_log("App will produce signal for 3 seconds (using PWM period ISR) ...\r\n");

    status = SemaphoreP_constructCounting(&gEpwmSyncSemObject, 0, numIsrCnt);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Register & enable interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = CONFIG_EPWM0_INTR;
    hwiPrms.callback    = &App_epwmIntrISR;
    hwiPrms.isPulse     = CONFIG_EPWM0_INTR_IS_PULSE;
    status              = HwiP_construct(&gEpwmHwiObject, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Configure PWM */
    App_epwmConfigTest2(CSL_MSS_ETPWMA_U_BASE, APP_EPWM_OUTPUT_CH, CONFIG_EPWM0_FCLK);

    while(numIsrCnt > 0)
    {
        SemaphoreP_pend(&gEpwmSyncSemObject, SystemP_WAIT_FOREVER);
        numIsrCnt--;
    }

    EPWM_etIntrDisable(CSL_MSS_ETPWMA_U_BASE);
    EPWM_etIntrClear(CSL_MSS_ETPWMA_U_BASE);     /* Clear any pending interrupts if any */
    HwiP_destruct(&gEpwmHwiObject);
    SemaphoreP_destruct(&gEpwmSyncSemObject);
    return;
}

/* Testcase 3 - Use chopper to generate higher frequency wave with configurable width of first and subsequent pulses */
static void test_epwm_chopper(void *args)
{
    int32_t             status;
    uint32_t            numIsrCnt = (APP_EPWM_RUN_TIME * APP_EPWM_OUTPUT_FREQ);
    HwiP_Params         hwiPrms;

    DebugP_log("EPWM Chopper module test ...\r\n");
    DebugP_log("App will produce signal for 3 seconds (using PWM period ISR) ...\r\n");

    status = SemaphoreP_constructCounting(&gEpwmSyncSemObject, 0, numIsrCnt);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Register & enable interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = CONFIG_EPWM0_INTR;
    hwiPrms.callback    = &App_epwmIntrISR;
    hwiPrms.isPulse     = CONFIG_EPWM0_INTR_IS_PULSE;
    status              = HwiP_construct(&gEpwmHwiObject, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Configure PWM */
    App_epwmConfigTest3(CSL_MSS_ETPWMA_U_BASE, APP_EPWM_OUTPUT_CH, CONFIG_EPWM0_FCLK);

    while(numIsrCnt > 0)
    {
        SemaphoreP_pend(&gEpwmSyncSemObject, SystemP_WAIT_FOREVER);
        numIsrCnt--;
    }

    EPWM_etIntrDisable(CSL_MSS_ETPWMA_U_BASE);
    EPWM_etIntrClear(CSL_MSS_ETPWMA_U_BASE);     /* Clear any pending interrupts if any */
    HwiP_destruct(&gEpwmHwiObject);
    SemaphoreP_destruct(&gEpwmSyncSemObject);
    return;
}

/* Testcase 4 - Use deadband to generate wave with configurable RED and FED values */
static void test_epwm_deadband(void *args)
{
    int32_t             status;
    uint32_t            numIsrCnt = (APP_EPWM_RUN_TIME * APP_EPWM_OUTPUT_FREQ);
    HwiP_Params         hwiPrms;

    DebugP_log("EPWM deadband test ...\r\n");
    DebugP_log("App will produce signal for 3 seconds (using PWM period ISR) ...\r\n");

    status = SemaphoreP_constructCounting(&gEpwmSyncSemObject, 0, numIsrCnt);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Register & enable interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = CONFIG_EPWM0_INTR;
    hwiPrms.callback    = &App_epwmIntrISR;
    hwiPrms.isPulse     = CONFIG_EPWM0_INTR_IS_PULSE;
    status              = HwiP_construct(&gEpwmHwiObject, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Configure PWM */
    App_epwmConfigTest4(CSL_MSS_ETPWMA_U_BASE, APP_EPWM_OUTPUT_CH, CONFIG_EPWM0_FCLK);

    while(numIsrCnt > 0)
    {
        SemaphoreP_pend(&gEpwmSyncSemObject, SystemP_WAIT_FOREVER);
        numIsrCnt--;
    }

    EPWM_etIntrDisable(CSL_MSS_ETPWMA_U_BASE);
    EPWM_etIntrClear(CSL_MSS_ETPWMA_U_BASE);     /* Clear any pending interrupts if any */
    HwiP_destruct(&gEpwmHwiObject);
    SemaphoreP_destruct(&gEpwmSyncSemObject);
    return;
}

/* Testcase 5 - Test Epwm_tbTimebaseClkCfg for various combinations of inputs */
static void test_epwm_tbclkcfg_api(void *args)
{
    uint32_t regval = 0U;
    uint32_t clkdivval = 0U;
    uint32_t hspclkdiv = 0U;

    /* Check for clock div = 1 (lowest possible div) */
    EPWM_tbTimebaseClkCfg(CSL_MSS_ETPWMA_U_BASE, CONFIG_EPWM0_FCLK, CONFIG_EPWM0_FCLK);
    regval = HW_RD_REG16(CSL_MSS_ETPWMA_U_BASE + CSL_EPWM_TBCTL);
    clkdivval = HW_GET_FIELD(regval, CSL_EPWM_TBCTL_CLKDIV);
    hspclkdiv = HW_GET_FIELD(regval, CSL_EPWM_TBCTL_HSPCLKDIV);
    TEST_ASSERT_EQUAL_UINT32(0, clkdivval);
    TEST_ASSERT_EQUAL_UINT32(0, hspclkdiv);

    /* Check for clock div = 1792 (highest possible div) */
    EPWM_tbTimebaseClkCfg(CSL_MSS_ETPWMA_U_BASE, APP_EPWM_TB_FREQ_MIN, CONFIG_EPWM0_FCLK);
    regval = HW_RD_REG16(CSL_MSS_ETPWMA_U_BASE + CSL_EPWM_TBCTL);
    clkdivval = HW_GET_FIELD(regval, CSL_EPWM_TBCTL_CLKDIV);
    hspclkdiv = HW_GET_FIELD(regval, CSL_EPWM_TBCTL_HSPCLKDIV);
    TEST_ASSERT_EQUAL_UINT32(7, clkdivval);
    TEST_ASSERT_EQUAL_UINT32(7, hspclkdiv);

    /* Check for clock div = 4 */
    EPWM_tbTimebaseClkCfg(CSL_MSS_ETPWMA_U_BASE, APP_EPWM_TB_FREQ, CONFIG_EPWM0_FCLK);
    regval = HW_RD_REG16(CSL_MSS_ETPWMA_U_BASE + CSL_EPWM_TBCTL);
    clkdivval = HW_GET_FIELD(regval, CSL_EPWM_TBCTL_CLKDIV);
    hspclkdiv = HW_GET_FIELD(regval, CSL_EPWM_TBCTL_HSPCLKDIV);
    TEST_ASSERT_EQUAL_UINT32(0, clkdivval);
    TEST_ASSERT_EQUAL_UINT32(2, hspclkdiv);

    return;
}

/* Testcase 6 - Test Epwm_tbPwmFreqCfg for various combinations of inputs */ 
static void test_epwm_tbfreqcfg_api(void *args)
{
    uint32_t regval = 0U;
    uint32_t prdld = 0U;
    uint32_t ctrmode = 0U;
    uint32_t tbprdval = 0U;

    /* Check for up down count mode */
    EPWM_tbPwmFreqCfg(CSL_MSS_ETPWMA_U_BASE, APP_EPWM_TB_FREQ, APP_EPWM_OUTPUT_FREQ, 
                      EPWM_TB_COUNTER_DIR_UP_DOWN, EPWM_SHADOW_REG_CTRL_ENABLE);
    regval = HW_RD_REG16(CSL_MSS_ETPWMA_U_BASE + CSL_EPWM_TBCTL);
    prdld = HW_GET_FIELD(regval, CSL_EPWM_TBCTL_PRDLD);
    ctrmode = HW_GET_FIELD(regval, CSL_EPWM_TBCTL_CTRMODE);
    TEST_ASSERT_EQUAL_UINT32(EPWM_SHADOW_REG_CTRL_ENABLE, prdld);
    TEST_ASSERT_EQUAL_UINT32(EPWM_TB_COUNTER_DIR_UP_DOWN, ctrmode);
    regval = HW_RD_REG16(CSL_MSS_ETPWMA_U_BASE + CSL_EPWM_TBPRD);
    tbprdval = HW_GET_FIELD(regval, CSL_EPWM_TBPRD);
    TEST_ASSERT_EQUAL_UINT32((APP_EPWM_TB_FREQ/APP_EPWM_OUTPUT_FREQ)/2U, tbprdval);

    /* Check for up count mode */
    EPWM_tbPwmFreqCfg(CSL_MSS_ETPWMA_U_BASE, APP_EPWM_TB_FREQ, APP_EPWM_OUTPUT_FREQ, 
                      EPWM_TB_COUNTER_DIR_UP, EPWM_SHADOW_REG_CTRL_DISABLE);
    regval = HW_RD_REG16(CSL_MSS_ETPWMA_U_BASE + CSL_EPWM_TBCTL);
    prdld = HW_GET_FIELD(regval, CSL_EPWM_TBCTL_PRDLD);
    ctrmode = HW_GET_FIELD(regval, CSL_EPWM_TBCTL_CTRMODE);
    TEST_ASSERT_EQUAL_UINT32(EPWM_SHADOW_REG_CTRL_DISABLE, prdld);
    TEST_ASSERT_EQUAL_UINT32(EPWM_TB_COUNTER_DIR_UP, ctrmode);
    regval = HW_RD_REG16(CSL_MSS_ETPWMA_U_BASE + CSL_EPWM_TBPRD);
    tbprdval = HW_GET_FIELD(regval, CSL_EPWM_TBPRD);
    TEST_ASSERT_EQUAL_UINT32((APP_EPWM_TB_FREQ/APP_EPWM_OUTPUT_FREQ)-1U, tbprdval);

    return;
}

/* Testcase 7 - Test Epwm_counterComparatorCfg for various combinations of inputs */
static void test_epwm_countercomparecfg_api(void *args)
{
    uint32_t regval = 0U;
    uint32_t cmpval = 0x100;
    uint32_t cmpval2 = 0x200;
    uint32_t status = 0U;
    uint32_t shdwmode = 0U;
    uint32_t loadmode = 0U;
    uint32_t cmpval3 = 0U;

    status = EPWM_counterComparatorCfg(CSL_MSS_ETPWMA_U_BASE, EPWM_CC_CMP_A, cmpval, 
                                       EPWM_SHADOW_REG_CTRL_ENABLE, 
                                       EPWM_CC_CMP_LOAD_MODE_CNT_EQ_ZERO, FALSE);
    regval = HW_RD_REG16(CSL_MSS_ETPWMA_U_BASE + CSL_EPWM_CMPA);
    cmpval3 = HW_GET_FIELD(regval, CSL_EPWM_CMPA);
    regval = HW_RD_REG16(CSL_MSS_ETPWMA_U_BASE + CSL_EPWM_CMPCTL);
    shdwmode = HW_GET_FIELD(regval, CSL_EPWM_CMPCTL_SHDWAMODE);
    loadmode = HW_GET_FIELD(regval, CSL_EPWM_CMPCTL_LOADAMODE);
    TEST_ASSERT_EQUAL_UINT32(TRUE, status);
    TEST_ASSERT_EQUAL_UINT32(cmpval, cmpval3);
    TEST_ASSERT_EQUAL_UINT32(EPWM_CC_CMP_LOAD_MODE_CNT_EQ_ZERO, loadmode);
    TEST_ASSERT_EQUAL_UINT32(EPWM_SHADOW_REG_CTRL_ENABLE, shdwmode);

    status = EPWM_counterComparatorCfg(CSL_MSS_ETPWMA_U_BASE, EPWM_CC_CMP_A, cmpval2, 
                                       EPWM_SHADOW_REG_CTRL_DISABLE, 
                                       EPWM_CC_CMP_LOAD_MODE_CNT_EQ_PRD, FALSE);
    regval = HW_RD_REG16(CSL_MSS_ETPWMA_U_BASE + CSL_EPWM_CMPA);
    cmpval3 = HW_GET_FIELD(regval, CSL_EPWM_CMPA);
    regval = HW_RD_REG16(CSL_MSS_ETPWMA_U_BASE + CSL_EPWM_CMPCTL);
    shdwmode = HW_GET_FIELD(regval, CSL_EPWM_CMPCTL_SHDWAMODE);
    loadmode = HW_GET_FIELD(regval, CSL_EPWM_CMPCTL_LOADAMODE);
    TEST_ASSERT_EQUAL_UINT32(FALSE, status);
    TEST_ASSERT_EQUAL_UINT32(cmpval, cmpval3);
    TEST_ASSERT_EQUAL_UINT32(EPWM_CC_CMP_LOAD_MODE_CNT_EQ_ZERO, loadmode);
    TEST_ASSERT_EQUAL_UINT32(EPWM_SHADOW_REG_CTRL_ENABLE, shdwmode);

    status = EPWM_counterComparatorCfg(CSL_MSS_ETPWMA_U_BASE, EPWM_CC_CMP_B, cmpval, 
                                       EPWM_SHADOW_REG_CTRL_ENABLE, 
                                       EPWM_CC_CMP_LOAD_MODE_CNT_EQ_ZERO_OR_PRD, FALSE);
    regval = HW_RD_REG16(CSL_MSS_ETPWMA_U_BASE + CSL_EPWM_CMPB);
    cmpval3 = HW_GET_FIELD(regval, CSL_EPWM_CMPB);
    regval = HW_RD_REG16(CSL_MSS_ETPWMA_U_BASE + CSL_EPWM_CMPCTL);
    shdwmode = HW_GET_FIELD(regval, CSL_EPWM_CMPCTL_SHDWBMODE);
    loadmode = HW_GET_FIELD(regval, CSL_EPWM_CMPCTL_LOADBMODE);
    TEST_ASSERT_EQUAL_UINT32(TRUE, status);
    TEST_ASSERT_EQUAL_UINT32(cmpval, cmpval3);
    TEST_ASSERT_EQUAL_UINT32(EPWM_CC_CMP_LOAD_MODE_CNT_EQ_ZERO_OR_PRD, loadmode);
    TEST_ASSERT_EQUAL_UINT32(EPWM_SHADOW_REG_CTRL_ENABLE, shdwmode);

    status = EPWM_counterComparatorCfg(CSL_MSS_ETPWMA_U_BASE, EPWM_CC_CMP_B, cmpval2, 
                                       EPWM_SHADOW_REG_CTRL_ENABLE, 
                                       EPWM_CC_CMP_LOAD_MODE_CNT_EQ_ZERO_OR_PRD, TRUE);
    regval = HW_RD_REG16(CSL_MSS_ETPWMA_U_BASE + CSL_EPWM_CMPB);
    cmpval3 = HW_GET_FIELD(regval, CSL_EPWM_CMPB);
    regval = HW_RD_REG16(CSL_MSS_ETPWMA_U_BASE + CSL_EPWM_CMPCTL);
    shdwmode = HW_GET_FIELD(regval, CSL_EPWM_CMPCTL_SHDWBMODE);
    loadmode = HW_GET_FIELD(regval, CSL_EPWM_CMPCTL_LOADBMODE);
    TEST_ASSERT_EQUAL_UINT32(TRUE, status);
    TEST_ASSERT_EQUAL_UINT32(cmpval2, cmpval3);
    TEST_ASSERT_EQUAL_UINT32(EPWM_CC_CMP_LOAD_MODE_CNT_EQ_ZERO_OR_PRD, loadmode);
    TEST_ASSERT_EQUAL_UINT32(EPWM_SHADOW_REG_CTRL_ENABLE, shdwmode);

    return;
}

static void App_epwmIntrISR(void *handle)
{
    volatile uint16_t status;

    status = EPWM_etIntrStatus(CSL_MSS_ETPWMA_U_BASE);
    if(status & EPWM_ETFLG_INT_MASK)
    {
        SemaphoreP_post(&gEpwmSyncSemObject);
        EPWM_etIntrClear(CSL_MSS_ETPWMA_U_BASE);
    }

    return;
}

static void App_epwmConfigTest1(uint32_t epwmBaseAddr,
                                uint32_t epwmCh,
                                uint32_t epwmFuncClk)
{
    EPWM_AqActionCfg  aqConfig;

    /* Configure Time base submodule */
    EPWM_tbTimebaseClkCfg(epwmBaseAddr, APP_EPWM_TB_FREQ_MAX, epwmFuncClk);
    EPWM_tbPwmFreqCfg(epwmBaseAddr, APP_EPWM_TB_FREQ_MAX, APP_EPWM_OUTPUT_FREQ_MAX,
        EPWM_TB_COUNTER_DIR_UP_DOWN,
            EPWM_SHADOW_REG_CTRL_ENABLE);
    EPWM_tbSyncDisable(epwmBaseAddr);
    EPWM_tbSetSyncOutMode(epwmBaseAddr, CSL_EPWM_TBCTL_SYNCOSEL_EPWMXSYNC);
    EPWM_tbSetEmulationMode(epwmBaseAddr, EPWM_TB_EMU_MODE_FREE_RUN);

    /* Configure counter compare submodule */
    EPWM_counterComparatorCfg(epwmBaseAddr, EPWM_CC_CMP_A,
        APP_EPWM_COMPA_VAL_MIN, EPWM_SHADOW_REG_CTRL_ENABLE,
            EPWM_CC_CMP_LOAD_MODE_CNT_EQ_ZERO, TRUE);
    EPWM_counterComparatorCfg(epwmBaseAddr, EPWM_CC_CMP_B,
        APP_EPWM_COMPA_VAL_MIN, EPWM_SHADOW_REG_CTRL_ENABLE,
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
    EPWM_tzTripEventDisable(epwmBaseAddr, EPWM_TZ_EVENT_ONE_SHOT, 0U);
    EPWM_tzTripEventDisable(epwmBaseAddr, EPWM_TZ_EVENT_CYCLE_BY_CYCLE, 0U);

    /* Configure event trigger Submodule */
    EPWM_etIntrCfg(epwmBaseAddr, EPWM_ET_INTR_EVT_CNT_EQ_ZRO,
        EPWM_ET_INTR_PERIOD_FIRST_EVT);
    EPWM_etIntrEnable(epwmBaseAddr);
}

static void App_epwmConfigTest2(uint32_t epwmBaseAddr,
                                uint32_t epwmCh,
                                uint32_t epwmFuncClk)
{
    EPWM_AqActionCfg  aqConfig;

    /* Configure Time base submodule */
    EPWM_tbTimebaseClkCfg(epwmBaseAddr, APP_EPWM_TB_FREQ_MIN, epwmFuncClk);
    EPWM_tbPwmFreqCfg(epwmBaseAddr, APP_EPWM_TB_FREQ_MIN, APP_EPWM_OUTPUT_FREQ_MIN,
        EPWM_TB_COUNTER_DIR_UP_DOWN,
            EPWM_SHADOW_REG_CTRL_ENABLE);
    EPWM_tbSyncDisable(epwmBaseAddr);
    EPWM_tbSetSyncOutMode(epwmBaseAddr, CSL_EPWM_TBCTL_SYNCOSEL_EPWMXSYNC);
    EPWM_tbSetEmulationMode(epwmBaseAddr, EPWM_TB_EMU_MODE_FREE_RUN);

    /* Configure counter compare submodule */
    EPWM_counterComparatorCfg(epwmBaseAddr, EPWM_CC_CMP_A,
        APP_EPWM_COMPA_VAL_MAX, EPWM_SHADOW_REG_CTRL_ENABLE,
            EPWM_CC_CMP_LOAD_MODE_CNT_EQ_ZERO, TRUE);
    EPWM_counterComparatorCfg(epwmBaseAddr, EPWM_CC_CMP_B,
        APP_EPWM_COMPA_VAL_MAX, EPWM_SHADOW_REG_CTRL_ENABLE,
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
    EPWM_tzTripEventDisable(epwmBaseAddr, EPWM_TZ_EVENT_ONE_SHOT, 0U);
    EPWM_tzTripEventDisable(epwmBaseAddr, EPWM_TZ_EVENT_CYCLE_BY_CYCLE, 0U);

    /* Configure event trigger Submodule */
    EPWM_etIntrCfg(epwmBaseAddr, EPWM_ET_INTR_EVT_CNT_EQ_ZRO,
        EPWM_ET_INTR_PERIOD_FIRST_EVT);
    EPWM_etIntrEnable(epwmBaseAddr);
}

static void App_epwmConfigTest3(uint32_t epwmBaseAddr,
                                uint32_t epwmCh,
                                uint32_t epwmFuncClk)
{
    EPWM_AqActionCfg  aqConfig;
    EPWM_ChopperCfg   chopperConfig;

    /* Configure Time base submodule */
    EPWM_tbTimebaseClkCfg(epwmBaseAddr, APP_EPWM_TB_FREQ, epwmFuncClk);
    EPWM_tbPwmFreqCfg(epwmBaseAddr, APP_EPWM_TB_FREQ, APP_EPWM_OUTPUT_FREQ,
        EPWM_TB_COUNTER_DIR_UP_DOWN,
            EPWM_SHADOW_REG_CTRL_ENABLE);
    EPWM_tbSyncDisable(epwmBaseAddr);
    EPWM_tbSetSyncOutMode(epwmBaseAddr, CSL_EPWM_TBCTL_SYNCOSEL_EPWMXSYNC);
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
    chopperConfig.dutyCycle = EPWM_CHP_DUTY_CYCLE_PERC_37PNT5;
    chopperConfig.clkFrequency = EPWM_CHP_CLK_FREQ_DIV_BY_2;
    chopperConfig.oneShotPulseWidth = EPWM_CHP_OSHT_WIDTH_12XSYSOUT_BY_8;
    EPWM_chopperEnable(epwmBaseAddr, TRUE);
    EPWM_chopperCfg(epwmBaseAddr, &chopperConfig);

    /* Configure trip zone Submodule */
    EPWM_tzTripEventDisable(epwmBaseAddr, EPWM_TZ_EVENT_ONE_SHOT, 0U);
    EPWM_tzTripEventDisable(epwmBaseAddr, EPWM_TZ_EVENT_CYCLE_BY_CYCLE, 0U);

    /* Configure event trigger Submodule */
    EPWM_etIntrCfg(epwmBaseAddr, EPWM_ET_INTR_EVT_CNT_EQ_ZRO,
        EPWM_ET_INTR_PERIOD_FIRST_EVT);
    EPWM_etIntrEnable(epwmBaseAddr);
}

static void App_epwmConfigTest4(uint32_t epwmBaseAddr,
                                uint32_t epwmCh,
                                uint32_t epwmFuncClk)
{
    EPWM_AqActionCfg  aqConfig;
    EPWM_DeadbandCfg  deadBandConfig;

    /* Configure Time base submodule */
    EPWM_tbTimebaseClkCfg(epwmBaseAddr, APP_EPWM_TB_FREQ, epwmFuncClk);
    EPWM_tbPwmFreqCfg(epwmBaseAddr, APP_EPWM_TB_FREQ, APP_EPWM_OUTPUT_FREQ,
        EPWM_TB_COUNTER_DIR_UP_DOWN,
            EPWM_SHADOW_REG_CTRL_ENABLE);
    EPWM_tbSyncDisable(epwmBaseAddr);
    EPWM_tbSetSyncOutMode(epwmBaseAddr, CSL_EPWM_TBCTL_SYNCOSEL_EPWMXSYNC);
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
    deadBandConfig.inputMode = EPWM_DB_IN_MODE_A_RED_A_FED;
    deadBandConfig.outputMode = EPWM_DB_OUT_MODE_A_RED_NO_FED;
    deadBandConfig.polaritySelect = EPWM_DB_POL_SEL_ACTV_HIGH;
    deadBandConfig.risingEdgeDelay = 200U;
    deadBandConfig.fallingEdgeDelay = 0U;
    EPWM_deadbandCfg(epwmBaseAddr, &deadBandConfig);

    /* Configure Chopper Submodule */
    EPWM_chopperEnable(epwmBaseAddr, FALSE);

    /* Configure trip zone Submodule */
    EPWM_tzTripEventDisable(epwmBaseAddr, EPWM_TZ_EVENT_ONE_SHOT, 0U);
    EPWM_tzTripEventDisable(epwmBaseAddr, EPWM_TZ_EVENT_CYCLE_BY_CYCLE, 0U);

    /* Configure event trigger Submodule */
    EPWM_etIntrCfg(epwmBaseAddr, EPWM_ET_INTR_EVT_CNT_EQ_ZRO,
        EPWM_ET_INTR_PERIOD_FIRST_EVT);
    EPWM_etIntrEnable(epwmBaseAddr);
}

