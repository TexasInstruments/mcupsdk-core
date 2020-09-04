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

#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/ClockP.h>
#include <drivers/adc.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/*
 * This example shows ADC conversion for all available input channels.
 *
 * ADC has eight input channels and all of them are used in this example.
 *
 * All channels are converted and their results are stored in FIFO 0,
 * upon the end of all conversions, the FIFO data is printed to the console.
 *
 * This example configures ADC for single shot conversion, that means
 * a single channel is converted only once.
 *
 * For this example the ADC does averaging of 16 samples to get the
 * result for each conversion.
 *
 * This example also shows how to use interrupts with ADC module
 * to get the required functionality.
 */

/* Reference voltage for ADC - should be given in mV */
#define APP_ADC_REF_VOLTAGE         (1800U)

/* Number of channels being converted */
#define APP_ADC_NUM_CH              (8U)

/* Global variables and objects */
static HwiP_Object gAdcHwiObject;
static SemaphoreP_Object gAdcSyncSemObject;

/* Function prototypes */
static void App_adcISR(void *handle);
static void App_adcInit(uint32_t baseAddr);
static void App_adcConfig(uint32_t baseAddr);
static void App_adcStart(uint32_t baseAddr);
static void App_adcStop(uint32_t baseAddr);
static void App_adcDeInit(uint32_t baseAddr);

void adc_singleshot_main(void *args)
{
    uint32_t baseAddr = CONFIG_ADC0_BASE_ADDR;
    HwiP_Params hwiPrms;
    uint32_t loopcnt, fifoData, fifoWordCnt, stepID, voltageLvl;
    int32_t status;

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    DebugP_log("ADC Single Shot Test Started ...\r\n");

    status = SemaphoreP_constructBinary(&gAdcSyncSemObject, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Register & enable interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum = CONFIG_ADC0_INTR;
    hwiPrms.callback = &App_adcISR;
    hwiPrms.priority = 1U;
    status = HwiP_construct(&gAdcHwiObject, &hwiPrms);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Initialize, Configure and Start the ADC module */
    App_adcInit(baseAddr);
    App_adcConfig(baseAddr);
    App_adcStart(baseAddr);

    /* Wait for the interrupt to occur */
    SemaphoreP_pend(&gAdcSyncSemObject, SystemP_WAIT_FOREVER);

    /* Get FIFO data */
    fifoWordCnt = ADCGetFIFOWordCount(baseAddr, ADC_FIFO_NUM_0);
    DebugP_log("Number of Samples in FIFO : %d\r\n", fifoWordCnt);
    DebugP_log("Step ID     Voltage Level\r\n");
    DebugP_log("-------     -------------\r\n");
    for (loopcnt = 0U; loopcnt < fifoWordCnt; loopcnt++)
    {
        fifoData = ADCGetFIFOData(baseAddr, ADC_FIFO_NUM_0);
        stepID   = ((fifoData & ADC_FIFODATA_ADCCHNLID_MASK) >>
                    ADC_FIFODATA_ADCCHNLID_SHIFT);
        fifoData = ((fifoData & ADC_FIFODATA_ADCDATA_MASK) >>
                    ADC_FIFODATA_ADCDATA_SHIFT);
        voltageLvl  = fifoData * (uint32_t) APP_ADC_REF_VOLTAGE;
        voltageLvl /= (uint32_t) ADC_GET_RANGE(CONFIG_ADC0_NUM_BITS);
        DebugP_log("%d           %d mV\r\n", (uint32_t)(stepID + 1U), (uint32_t)voltageLvl);
    }

    /* Stop and power down the ADC*/
    App_adcStop(baseAddr);
    App_adcDeInit(baseAddr);

    HwiP_destruct(&gAdcHwiObject);
    SemaphoreP_destruct(&gAdcSyncSemObject);

    DebugP_log("ADC Single Shot Test Completed!!\r\n");
    DebugP_log("All tests have passed!!\r\n");

    Board_driversClose();
    Drivers_close();
}

void App_adcISR(void *handle)
{
    uint32_t status;
    uint32_t baseAddr = CONFIG_ADC0_BASE_ADDR;

    /* Get interrupt status and clear */
    status = ADCGetIntrStatus(baseAddr);
    ADCClearIntrStatus(baseAddr, status);

    /* Process ISR */
    SemaphoreP_post(&gAdcSyncSemObject);

    /* Set EOI to generate next interrupt if any */
    ADCWriteEOI(baseAddr);
}

void App_adcConfig(uint32_t baseAddr)
{
    int32_t         configStatus;
    uint32_t        chCnt, adcStep;
    adcStepConfig_t adcConfig;

    /* Enable interrupts */
    ADCEnableIntr(baseAddr, (ADC_INTR_SRC_END_OF_SEQUENCE |
                             ADC_INTR_SRC_FIFO0_THRESHOLD |
                             ADC_INTR_SRC_FIFO0_OVERRUN |
                             ADC_INTR_SRC_FIFO0_UNDERFLOW |
                             ADC_INTR_SRC_FIFO1_THRESHOLD |
                             ADC_INTR_SRC_FIFO1_OVERRUN |
                             ADC_INTR_SRC_FIFO1_UNDERFLOW |
                             ADC_INTR_SRC_OUT_OF_RANGE));

    /*
     * Configure all ADC Steps
     */
    /* Initialize ADC configuration params */
    adcConfig.mode             = ADC_OPERATION_MODE_SINGLE_SHOT;
    adcConfig.openDelay        = 0x1U;
    adcConfig.sampleDelay      = 0U;
    adcConfig.rangeCheckEnable = 0U;
    adcConfig.averaging        = ADC_AVERAGING_16_SAMPLES;
    adcConfig.fifoNum          = ADC_FIFO_NUM_0;

    /* Configure all required steps - Step 1 to N mapped to Channel 1 to N */
    for(chCnt = 0U; chCnt < APP_ADC_NUM_CH; chCnt++)
    {
        adcConfig.channel = ADC_CHANNEL_1 + chCnt;
        adcStep = ADC_STEP_1 + chCnt;   /* Step -> Channel one to one mapped */
        configStatus = ADCSetStepParams(baseAddr, adcStep, &adcConfig);
        DebugP_assert(SystemP_SUCCESS == configStatus);
    }

    ADCStepIdTagEnable(baseAddr, TRUE);
    configStatus = ADCSetCPUFIFOThresholdLevel(baseAddr, ADC_FIFO_NUM_0, 40U);
    DebugP_assert(SystemP_SUCCESS == configStatus);

    /* Step enable */
    for(chCnt = 0U; chCnt < APP_ADC_NUM_CH; chCnt++)
    {
        adcStep = ADC_STEP_1 + chCnt;   /* Step -> Channel one to one mapped */
        ADCStepEnable(baseAddr, adcStep, TRUE);
    }
}

static void App_adcInit(uint32_t baseAddr)
{
    /* Clear All interrupt status */
    ADCClearIntrStatus(baseAddr, ADC_INTR_STATUS_ALL);

    /* Power up AFE */
    ADCPowerUp(baseAddr, TRUE);

    /* Wait for 4us at least */
    ClockP_usleep(5U);

    /* Do the internal calibration */
    ADCInit(baseAddr, FALSE, 0U, 0U);
}

static void App_adcStart(uint32_t baseAddr)
{
    adcSequencerStatus_t status;

    /* Check if FSM is idle */
    ADCGetSequencerStatus(baseAddr, &status);
    while ((ADC_ADCSTAT_FSM_BUSY_IDLE != status.fsmBusy) &&
           ADC_ADCSTAT_STEP_ID_IDLE != status.stepId)
    {
        ADCGetSequencerStatus(baseAddr, &status);
    }
    /* Start ADC conversion */
    ADCStart(baseAddr, TRUE);
}

static void App_adcStop(uint32_t baseAddr)
{
    uint32_t                chCnt, adcStep;
    adcSequencerStatus_t    status;

    /* Disable all/enabled steps */
    for(chCnt = 0U; chCnt < APP_ADC_NUM_CH; chCnt++)
    {
        adcStep = ADC_STEP_1 + chCnt;   /* Step -> Channel one to one mapped */
        ADCStepEnable(baseAddr, adcStep, FALSE);
    }

    /* Wait for FSM to go IDLE */
    ADCGetSequencerStatus(baseAddr, &status);
    while((ADC_ADCSTAT_FSM_BUSY_IDLE != status.fsmBusy) &&
           ADC_ADCSTAT_STEP_ID_IDLE  != status.stepId)
    {
        ADCGetSequencerStatus(baseAddr, &status);
    }

    /* Stop ADC */
    ADCStart(baseAddr, FALSE);

    /* Wait for FSM to go IDLE */
    ADCGetSequencerStatus(baseAddr, &status);
    while ((ADC_ADCSTAT_FSM_BUSY_IDLE != status.fsmBusy) &&
            ADC_ADCSTAT_STEP_ID_IDLE  != status.stepId)
    {
        ADCGetSequencerStatus(baseAddr, &status);
    }
}

static void App_adcDeInit(uint32_t baseAddr)
{
    ADCPowerUp(baseAddr, FALSE);
}
