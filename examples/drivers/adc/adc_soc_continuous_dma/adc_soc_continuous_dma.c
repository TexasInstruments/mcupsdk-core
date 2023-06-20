/*
 *  Copyright (C) 2023 Texas Instruments Incorporated
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
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/HwiP.h>
#include <drivers/epwm.h>
#include <drivers/adc.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/*
 * This example sets up two ADC channels to convert simultaneously. The
 * results will be transferred by DMA into a buffer in RAM.
 *
 * errata i2355 for AM263x:
 *  DMA trigger from the ADC INT (Late) occurs just before the Result space updation,
 *  as a result of which the stale data from the result space will be transferred by
 *  DMA.
 *  Workaround:
 *      use an empty channel transfer before the Actual transfer.
 *
 * It configures ePWM0 to trigger SOC0 on ADC1 and ADC2. EPWM is only used to
 * trigger the first ADC conversion. INT0 of ADC1 is configured to generate
 * interrupt after first conversion and will then disable EPWM SOC generation.
 * INT1 of both ADC's is configured to enable continuous conversion.
 *
 * DMA channel 0 is triggered at EOC0 of ADC1 and will copy conversion result
 * to a empty buffer in RAM, and triggers DMA Channel 1 to copy actual conversion
 * result to result buffer in RAM. Similarly, DMA Channel 2 is triggered at ADC 2 EOC0
 * and transfers ADC 2 result space an empty buffer in RAM and .
 * DMA will generate interrupt after the buffer is filled and will stop conversion on both ADCs.
 *
 * The below watch variables can be used to view ADC conversion results.
 *
 * External Connections
 * ADC1_AIN0 and ADC2_AIN0 pins should be connected to signals to be converted.
 * AM263X-CC :
 *  - Feed the External Volatage to the following
 *      - ADC1_AIN0 :   HSEC-PIN 12
 *      - ADC2_AIN0 :   HSEC-PIN 31
 * AM263X-LP :
 *  - Feed the External Volatage to the following
 *      - ADC1_AIN0 :   J1/3 24
 *      - ADC2_AIN0 :   J1/3 25
 *
 * Watch Variables
 * gAdc1DataBuffer - Buffers which stores conversion results from ADC1
 * gADC2DataBuffer - Buffers which stores conversion results from ADC2
 */

/* Size of buffer for storing conversion results */
#define RESULTS_BUFFER_SIZE     1024*16
/* Event queue to be used for EDMA transfer */
#define EDMA_TEST_EVT_QUEUE_NO_0  0U
#define EDMA_TEST_EVT_QUEUE_NO_1  1U

/* DMA channel number to transfer ADC1 and ADC2 conversion results*/
/* the following channels will be used for empty transaction to comply to errata i2355
   these chain the ADC result data transfer */
#define ADC1_EDMA_CHANNEL       (DMA_TRIG_XBAR_EDMA_MODULE_0)
#define ADC2_EDMA_CHANNEL       (DMA_TRIG_XBAR_EDMA_MODULE_2)
/* Note: the actual transfer channels are configured for EDMA_RESOURCE_ALLOC_ANY,
typically selecting the next available channels
    in this case, DMA_TRIG_XBAR_EDMA_MODULE_1 and DMA_TRIG_XBAR_EDMA_MODULE_3 hold the actual transfer.*/

/* Global variables and objects */
/* Buffers to store conversion results from ADC1 and ADC2 */
uint16_t gAdc1DataBuffer[RESULTS_BUFFER_SIZE];
uint16_t gAdc2DataBuffer[RESULTS_BUFFER_SIZE];
uint16_t gUnusedBuffer[RESULTS_BUFFER_SIZE];

static HwiP_Object  gAdcHwiObject;
/* Semaphore to indicate transfer completion */
static SemaphoreP_Object gEdmaTransferDoneSem;
/* ADC instance base addresses */
uint32_t gAdc1baseAddr = CONFIG_ADC1_BASE_ADDR;
uint32_t gAdc2baseAddr = CONFIG_ADC2_BASE_ADDR;

/* Function Prototypes */
uint16_t App_dmaConfigure(const uint16_t *table, uint16_t table_size,
        EDMA_Handle dma_handle, uint32_t dma_ch,
        uint32_t adc_base, uint32_t *tccAlloc, uint32_t event_queue_number);
static void App_adcISR(void *args);
static void App_dmach0ISR(Edma_IntrHandle intrHandle, void *args);

void adc_soc_continuous_dma_main(void *args)
{
    int32_t  status;
    Edma_IntrObject     intrObj;
    HwiP_Params  hwiPrms;
    uint32_t     loopCnt = 0;
    uint32_t tccAlloc0, tccAlloc1;

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    DebugP_log("ADC Continuous DMA transfer Test Started ...\r\n");

    /* Initialize both the result buffers with zeroes */
    for(loopCnt = 0U; loopCnt < RESULTS_BUFFER_SIZE; loopCnt++)
    {
        gUnusedBuffer[loopCnt] = 0U;
        gAdc1DataBuffer[loopCnt] = 0U;
        gAdc2DataBuffer[loopCnt] = 0U;
    }

    /* Perform a cache write back to the result buffers */
    CacheP_wb((void *)gAdc1DataBuffer, RESULTS_BUFFER_SIZE*2, CacheP_TYPE_ALL);
    CacheP_wb((void *)gAdc2DataBuffer, RESULTS_BUFFER_SIZE*2, CacheP_TYPE_ALL);

    /* Register & enable interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_0;
    hwiPrms.callback    = &App_adcISR;
    status              = HwiP_construct(&gAdcHwiObject, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Create a semaphore to signal EDMA transfer completion */
    status = SemaphoreP_constructBinary(&gEdmaTransferDoneSem, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Configure DMA channels to transfer both ADC results */
    App_dmaConfigure(gAdc1DataBuffer, RESULTS_BUFFER_SIZE, gEdmaHandle[0],
                ADC1_EDMA_CHANNEL, CONFIG_ADC1_RESULT_BASE_ADDR, &tccAlloc0, EDMA_TEST_EVT_QUEUE_NO_0);

    App_dmaConfigure(gAdc2DataBuffer, RESULTS_BUFFER_SIZE, gEdmaHandle[0],
                ADC2_EDMA_CHANNEL, CONFIG_ADC2_RESULT_BASE_ADDR, &tccAlloc1, EDMA_TEST_EVT_QUEUE_NO_1);

    /* Register interrupt */
    intrObj.tccNum = tccAlloc0;
    intrObj.cbFxn  = &App_dmach0ISR;
    intrObj.appData = (void *) &gEdmaTransferDoneSem;
    status = EDMA_registerIntr(gEdmaHandle[0], &intrObj);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Clear ADC Interrupt status */
    ADC_clearInterruptStatus(gAdc1baseAddr,ADC_INT_NUMBER1);
    ADC_clearInterruptStatus(gAdc1baseAddr,ADC_INT_NUMBER2);
    ADC_clearInterruptStatus(gAdc2baseAddr,ADC_INT_NUMBER1);
    ADC_clearInterruptStatus(gAdc2baseAddr,ADC_INT_NUMBER2);

    /* Enable event counter init for SOCA */
    EPWM_enableADCTriggerEventCountInit(CONFIG_EPWM0_BASE_ADDR, EPWM_SOC_A);
    /* Set the event counter init value to 0 */
    EPWM_setADCTriggerEventCountInitValue(CONFIG_EPWM0_BASE_ADDR, EPWM_SOC_A, 0);
    /* Force the event counter init value to event counter */
    EPWM_forceADCTriggerEventCountInit(CONFIG_EPWM0_BASE_ADDR, EPWM_SOC_A);
    EPWM_clearADCTriggerFlag(CONFIG_EPWM0_BASE_ADDR, EPWM_SOC_A);

    /* Enable ADC Trigger from EPWM to start ADC conversion */
	EPWM_enableADCTrigger(CONFIG_EPWM0_BASE_ADDR, EPWM_SOC_A);
	EPWM_setADCTriggerSource(CONFIG_EPWM0_BASE_ADDR, EPWM_SOC_A, EPWM_SOC_TBCTR_ZERO, EPWM_SOC_TBCTR_ZERO);
	EPWM_setADCTriggerEventPrescale(CONFIG_EPWM0_BASE_ADDR, EPWM_SOC_A, 1);
	EPWM_setTimeBaseCounterMode(CONFIG_EPWM0_BASE_ADDR, EPWM_COUNTER_MODE_UP);

    /*
     * Wait while DMA transfers ADC conversion results to buffer.
    */
    SemaphoreP_pend(&gEdmaTransferDoneSem, SystemP_WAIT_FOREVER);

    /* Invalidate destination buffer */
    CacheP_inv((void *)gUnusedBuffer, RESULTS_BUFFER_SIZE*2, CacheP_TYPE_ALL);
    CacheP_inv((void *)gAdc1DataBuffer, RESULTS_BUFFER_SIZE*2, CacheP_TYPE_ALL);
    CacheP_inv((void *)gAdc2DataBuffer, RESULTS_BUFFER_SIZE*2, CacheP_TYPE_ALL);

    DebugP_log("ADC1 : ADC2 Result register value -\r\n");

    loopCnt = 0;
    /* Print few elements from the result buffer */
    while(loopCnt < RESULTS_BUFFER_SIZE)
    {
        DebugP_log("%d : %d\r\n", gAdc1DataBuffer[loopCnt], gAdc2DataBuffer[loopCnt]);
        loopCnt += 100;
    }

    DebugP_log("ADC Continuous DMA transfer Test Passed\r\n");
    DebugP_log("All tests have passed!!\r\n");

    Board_driversClose();
    Drivers_close();
}

uint16_t App_dmaConfigure(
        const uint16_t *table, uint16_t table_size,
        EDMA_Handle dma_handle, uint32_t dma_ch,
        uint32_t adc_base, uint32_t *tccAlloc, uint32_t event_queue_number)
{

    uint32_t            baseAddr, regionId;
    EDMACCPaRAMEntry    empty_edmaParam, edmaParam;
    uint32_t            empty_dmaCh, dmaCh, empty_tcc, tcc, empty_param, param;
    // EDMACCPaRAMEntry    empty_edmaParam;
    // uint32_t            empty_dmaCh, empty_tcc, empty_param;
    int32_t             testStatus = SystemP_SUCCESS;

    /* Enable only TC and ITC chaining on dmaCh0.
       TC and ITC interrupt enable not required. */
    uint32_t chainOptions = (EDMA_OPT_TCCHEN_MASK |
                             EDMA_OPT_ITCCHEN_MASK);

    baseAddr = EDMA_getBaseAddr(gEdmaHandle[0]);
    DebugP_assert(baseAddr != 0);

    regionId = EDMA_getRegionId(gEdmaHandle[0]);
    DebugP_assert(regionId < SOC_EDMA_NUM_REGIONS);

    empty_dmaCh = dma_ch;
    testStatus = EDMA_allocDmaChannel(gEdmaHandle[0], &empty_dmaCh);
    DebugP_assert(testStatus == SystemP_SUCCESS);

    /* typically allocates the next available channel */
    empty_tcc = EDMA_RESOURCE_ALLOC_ANY;
    testStatus = EDMA_allocTcc(gEdmaHandle[0], &empty_tcc);
    DebugP_assert(testStatus == SystemP_SUCCESS);

    empty_param = EDMA_RESOURCE_ALLOC_ANY;
    testStatus = EDMA_allocParam(gEdmaHandle[0], &empty_param);
    DebugP_assert(testStatus == SystemP_SUCCESS);

    dmaCh = EDMA_RESOURCE_ALLOC_ANY;
    testStatus = EDMA_allocDmaChannel(gEdmaHandle[0], &dmaCh);
    DebugP_assert(testStatus == SystemP_SUCCESS);

    tcc = EDMA_RESOURCE_ALLOC_ANY;
    testStatus = EDMA_allocTcc(gEdmaHandle[0], &tcc);
    DebugP_assert(testStatus == SystemP_SUCCESS);
    /* Passing back the Channel parameter to register the interrupt ISR*/
    *tccAlloc = tcc;

    param = EDMA_RESOURCE_ALLOC_ANY;
    testStatus = EDMA_allocParam(gEdmaHandle[0], &param);
    DebugP_assert(testStatus == SystemP_SUCCESS);

    /* Request channel */
    EDMA_configureChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
         empty_dmaCh, empty_tcc, empty_param, event_queue_number);

    /* Program Param Set */
    EDMA_ccPaRAMEntry_init(&empty_edmaParam);
    empty_edmaParam.srcAddr       = (uint32_t) SOC_virtToPhy((void *)(adc_base+CSL_ADC_RESULT_ADCRESULT0));
    empty_edmaParam.destAddr      = (uint32_t) SOC_virtToPhy((void *)gUnusedBuffer);
    empty_edmaParam.aCnt          = (uint16_t) 2;
    empty_edmaParam.bCnt          = (uint16_t) table_size;
    empty_edmaParam.cCnt          = (uint16_t) 1;
    empty_edmaParam.bCntReload    = 0;
    empty_edmaParam.srcBIdx       = (int16_t) EDMA_PARAM_BIDX(0);
    empty_edmaParam.destBIdx      = (int16_t) EDMA_PARAM_BIDX(2);
    empty_edmaParam.srcCIdx       = (int16_t) 0;
    empty_edmaParam.destCIdx      = (int16_t) 0;
    empty_edmaParam.linkAddr      = 0xFFFFU;
    empty_edmaParam.srcBIdxExt    = (int8_t) EDMA_PARAM_BIDX_EXT(0);
    empty_edmaParam.destBIdxExt   = (int8_t) EDMA_PARAM_BIDX_EXT(2);
    empty_edmaParam.opt           = (EDMA_OPT_TCINTEN_MASK | EDMA_OPT_ITCINTEN_MASK |
                              ((((uint32_t)empty_tcc) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));
    /* Enabling in options for the,
        the Transfer-complete Interrupt          --> After total data is transferred
    ,   Intermediate transfer complete interrupt --> After each A data chunk transfer is comeplete
            this will be used for the chaining of the actual transfer.
    */

    EDMA_setPaRAM(baseAddr, empty_param, &empty_edmaParam);

    EDMA_configureChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
            dmaCh, tcc, param, event_queue_number);

    /* Program Param Set */
    EDMA_ccPaRAMEntry_init(&edmaParam);
    edmaParam.srcAddr       = (uint32_t) SOC_virtToPhy((void *)(adc_base+CSL_ADC_RESULT_ADCRESULT0));
    edmaParam.destAddr      = (uint32_t) SOC_virtToPhy((void *)table);
    edmaParam.aCnt          = (uint16_t) 2;
    edmaParam.bCnt          = (uint16_t) table_size;
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
    /* Enabling the Interrupt for Transfer complete of all the data */
    EDMA_setPaRAM(baseAddr, param, &edmaParam);

    /* Chain the empty transfer to the actual transfer.
    Chain options include Total transfer completion and intermediate channel completion flags
    from empty channel*/

    EDMA_chainChannel(baseAddr, empty_param, dmaCh, chainOptions);

    /* Enabling the transfer region for empty channel for the trigger mode set in the Syscfg through
    the DMA Xbars adn DMA TRIG Xbars.*/
    EDMA_enableTransferRegion(baseAddr, regionId, empty_dmaCh,
                              EDMA_TRIG_MODE_EVENT);
    return testStatus;
}

void App_adcISR(void *args)
{
    /* Remove ePWM trigger */
    EPWM_disableADCTrigger(CONFIG_EPWM0_BASE_ADDR, EPWM_SOC_A);

    /* Disable this interrupt from happening again */
    ADC_disableInterrupt(gAdc1baseAddr, ADC_INT_NUMBER1);
}

void App_dmach0ISR(Edma_IntrHandle intrHandle, void *args)
{
    SemaphoreP_Object *semObjPtr = (SemaphoreP_Object *)args;
    DebugP_assert(semObjPtr != NULL);

    /* Stop the ADCs by removing the trigger for SOC0 */
    ADC_setInterruptSOCTrigger(gAdc1baseAddr, ADC_SOC_NUMBER0,
                               ADC_INT_SOC_TRIGGER_NONE);
    ADC_setInterruptSOCTrigger(gAdc2baseAddr, ADC_SOC_NUMBER0,
                               ADC_INT_SOC_TRIGGER_NONE);

    /* Post the semaphore to signal end of DMA transfer */
    SemaphoreP_post(semObjPtr);
}