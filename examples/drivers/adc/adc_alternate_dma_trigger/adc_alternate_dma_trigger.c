/*
 *  Copyright (C) 2022-2024 Texas Instruments Incorporated
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
#include <kernel/dpl/HwiP.h>
#include <drivers/epwm.h>
#include <drivers/adc.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"


/*
 * Example Description
 *      This example showcases the usage of the alternate dma trigger from ADC,
 * in order to trigger DMA once the results have been latched in the result space,
 * instead of triggering the DMA at the end of conversion. ADC_enableAltDMATiming() 
 * will enable this trigger.
 * 
 * Configurations
 *      The Example Configures the SOC0 in both ADC0, ADC1 to be triggered by ADC_INT1.
 * the DMA channels 0,1 are configured for triggers from ADC0_INT1, ADC1_INT1 respectively.
 * 
 * External Connections
 *      ADC0-SOC0 Samples on Channel 2, where as ADC1-SOC1 samples on Channel 0.
 *  - on AM263x CC E2, AM263Px CC E2, with HSEC Dock 
 *      - Feed Analog input to ADC0_AIN2 - HSEC PIN 15  
 *      - Feed Analog input to ADC1_AIN0 - HSEC PIN 12  
 *  - on AM263x LP E2, AM263Px LP
 *      - Feed Analog Input to the ADC0_AIN2 - J7 Pin 66
 *      - Feed Analog Input to the ADC0_AIN2 - J3 Pin 24
 * 
 * Watch Variables 
 *  gAdc0DataBuffer[] - Buffer to store ADC0 SOC0 results, copied by DMA.
 *  gAdc1DataBuffer[] - Buffer to store ADC1 SOC0 results, copied by DMA.
 */



#define NUM_SOC_PER_TRANSFER           (1U)
#define NUM_BYTES_IN_SOC_RESULT        (2U)

/* Number of ADC conversions required */
#define RESULTS_BUFFER_SIZE     (NUM_SOC_PER_TRANSFER*256U)

/* Event queue to be used for EDMA transfer */
#define EDMA_TEST_EVT_QUEUE_NO_0  (0U)
#define EDMA_TEST_EVT_QUEUE_NO_1  (1U)


/* Semaphore to indicate transfer completion */
static SemaphoreP_Object gEdmaTransferDoneSem;

/* defining Macro for compiler attribute */
#define ADC_DATA_TCM __attribute__((__section__(".adcData")))

/* DMA channel number to transfer ADC0 and ADC1 conversion results*/
#define ADC0_EDMA_CHANNEL       (DMA_TRIG_XBAR_EDMA_MODULE_0)
#define ADC1_EDMA_CHANNEL       (DMA_TRIG_XBAR_EDMA_MODULE_1)

uint16_t ADC_DATA_TCM gAdc0DataBuffer[RESULTS_BUFFER_SIZE];
uint16_t ADC_DATA_TCM gAdc1DataBuffer[RESULTS_BUFFER_SIZE];

uint32_t gAdc0BaseAddr = CONFIG_ADC0_BASE_ADDR;
uint32_t gAdc1BaseAddr = CONFIG_ADC1_BASE_ADDR;

uint32_t gAdc0ResultBaseAddr = CONFIG_ADC0_RESULT_BASE_ADDR;
uint32_t gAdc1ResultBaseAddr = CONFIG_ADC1_RESULT_BASE_ADDR;


/* Function Prototypes */
void App_dmach0ISR(Edma_IntrHandle intrHandle, void *args);

/**
 * @brief configures the DMA
 * 
 * @param dest_table                : this will the buffer in the memory to store the ADC transfer values 
 * @param src_adcResultBase         : this is to mention with ADC result memories will be used. this can also be resBase + begin_socNumber
 * @param number_soc_per_transfer   : Number of consecutive SOCs to be transfered in one go.
 * @param dma_ch                    : Which DMA channel is used for the transfer
 * @param tccAlloc                  : Which tcc, to hold the channel transfer information. can be used to register interrupt
 * @param event_queue_number        : Whith event queue to be used for the transfer. 2 independnent queue may used at max
 */
static void App_dmaConfigure(
    const uint16_t *dest_table,            
    uint32_t src_adcResultBase,
    uint16_t number_soc_per_transfer,
    uint32_t dma_ch,
    uint32_t *tccAlloc,
    uint32_t event_queue_number
);

void adc_alternate_dma_trigger_main(void *args)
{

    /* variable to iterate ADC_CONVERSION_COUNT */

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    DebugP_log("ADC Alternate DMA trigger Test Started ...\r\n");

    uint32_t    loopCnt = 0, status;
    Edma_IntrObject     intrObj;
    uint32_t tccAlloc0, tccAlloc1;

    /* Initialize both the result buffers with zeroes */
    for(loopCnt = 0U; loopCnt < RESULTS_BUFFER_SIZE; loopCnt++)
    {
        gAdc0DataBuffer[loopCnt] = 0U;
        gAdc1DataBuffer[loopCnt] = 0U;
    }

    /* Perform a cache write back to the result buffers */
    CacheP_wb((void *)gAdc0DataBuffer, RESULTS_BUFFER_SIZE*2, CacheP_TYPE_ALL);
    CacheP_wb((void *)gAdc1DataBuffer, RESULTS_BUFFER_SIZE*2, CacheP_TYPE_ALL);

    /* setting up DMA for transfers */
    App_dmaConfigure(
        gAdc0DataBuffer,                // destination buffer   
        gAdc0ResultBaseAddr,            // source
        NUM_SOC_PER_TRANSFER,           // number of continuos SOC Results in each transfer 
        ADC0_EDMA_CHANNEL,              // DMA Channel
        &tccAlloc0,                     
        EDMA_TEST_EVT_QUEUE_NO_0
        );
    App_dmaConfigure(
        gAdc1DataBuffer,                // destination buffer   
        gAdc1ResultBaseAddr,            // source
        NUM_SOC_PER_TRANSFER,           // number of continuos SOC Results in each transfer 
        ADC1_EDMA_CHANNEL,              // DMA Channel
        &tccAlloc1,                     
        EDMA_TEST_EVT_QUEUE_NO_1
        );

    /* Create a semaphore to signal EDMA transfer completion */
    status = SemaphoreP_constructBinary(&gEdmaTransferDoneSem, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Register interrupt for dma channel transfer completion*/
    intrObj.tccNum = tccAlloc0;
    intrObj.cbFxn  = &App_dmach0ISR;
    intrObj.appData = (void *) &gEdmaTransferDoneSem;
    status = EDMA_registerIntr(gEdmaHandle[0], &intrObj);
    DebugP_assert(status == SystemP_SUCCESS);


    /* setting up for both ADCs to be triggered at the same time using the global software force */
    SOC_enableAdcGlobalForce(0, TRUE);  // for ADC0
    SOC_enableAdcGlobalForce(1, TRUE);  // for ADC1
    
    /* clearing interrupt status and overflow status */
    ADC_clearInterruptStatus(gAdc0BaseAddr, ADC_INT_NUMBER1);
    ADC_clearInterruptOverflowStatus(gAdc0BaseAddr, ADC_INT_NUMBER1);
    ADC_clearInterruptStatus(gAdc1BaseAddr, ADC_INT_NUMBER1);
    ADC_clearInterruptOverflowStatus(gAdc1BaseAddr, ADC_INT_NUMBER1);
    
    /* enabling alternate DMA timing */
    ADC_enableAltDMATiming(gAdc0BaseAddr);
    ADC_enableAltDMATiming(gAdc1BaseAddr);

    /* triggering both ADCs at once on SOC0 */
    SOC_adcSocGlobalForce((uint32_t) ADC_SOC_NUMBER0);
    
    /* removing the global trigger */
    SOC_enableAdcGlobalForce(0, FALSE);  // for ADC0
    SOC_enableAdcGlobalForce(1, FALSE);  // for ADC1

    /*
     * Wait while DMA transfers ADC conversion results to buffer.
     */
    SemaphoreP_pend(&gEdmaTransferDoneSem, SystemP_WAIT_FOREVER);

    /* Invalidate destination buffer */
    CacheP_inv((void *)gAdc0DataBuffer, RESULTS_BUFFER_SIZE*2, CacheP_TYPE_ALL);
    CacheP_inv((void *)gAdc1DataBuffer, RESULTS_BUFFER_SIZE*2, CacheP_TYPE_ALL);
    
    DebugP_log("ADC1 : ADC2 Result register value -\r\n");

    /* Print few elements from the result buffer */
    loopCnt = 0;
    while(loopCnt < RESULTS_BUFFER_SIZE)
    {
        DebugP_log("%d : %d\r\n", gAdc0DataBuffer[loopCnt], gAdc1DataBuffer[loopCnt]);
        loopCnt += RESULTS_BUFFER_SIZE/10;
    }

    ADC_disableAltDMATiming(gAdc0BaseAddr);
    ADC_disableAltDMATiming(gAdc1BaseAddr);

    DebugP_log("ADC Alternate DMA trigger Test Passed\r\n");
    DebugP_log("All tests have passed!!\r\n");

    Board_driversClose();
    Drivers_close();
}


static void App_dmaConfigure(
    const uint16_t *dest_table,            
    uint32_t src_adcResultBase,
    uint16_t number_soc_per_transfer,
    uint32_t dma_ch,
    uint32_t *tccAlloc,
    uint32_t event_queue_number
)
{
    uint32_t baseAddr, regId, tcc, param;
    EDMACCPaRAMEntry edmaParam;
    
    int32_t status = SystemP_SUCCESS;

    baseAddr = EDMA_getBaseAddr(gEdmaHandle[0]);
    DebugP_assert(baseAddr != 0);

    regId = EDMA_getRegionId(gEdmaHandle[0]);
    DebugP_assert(regId < SOC_EDMA_NUM_REGIONS);

    dma_ch = EDMA_RESOURCE_ALLOC_ANY;
    status = EDMA_allocDmaChannel(gEdmaHandle[0], &dma_ch);
    DebugP_assert(status == SystemP_SUCCESS);

    tcc = EDMA_RESOURCE_ALLOC_ANY;
    status = EDMA_allocTcc(gEdmaHandle[0], &tcc);
    DebugP_assert(status == SystemP_SUCCESS); 
    /* Passing back the Channel parameter to register the interrupt ISR*/
    *tccAlloc = tcc;

    param = EDMA_RESOURCE_ALLOC_ANY;
    status = EDMA_allocParam(gEdmaHandle[0], &param);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Request channel */
    EDMA_configureChannelRegion(baseAddr, regId, EDMA_CHANNEL_TYPE_DMA,
         dma_ch, tcc, param, event_queue_number);
    
    /* Program Param Set */
    EDMA_ccPaRAMEntry_init(&edmaParam);
    edmaParam.srcAddr       = (uint32_t) SOC_virtToPhy((void *)src_adcResultBase);
    edmaParam.destAddr      = (uint32_t) SOC_virtToPhy((void *)dest_table);
    edmaParam.aCnt          = (uint16_t) number_soc_per_transfer * (NUM_BYTES_IN_SOC_RESULT);
    edmaParam.bCnt          = (uint16_t) RESULTS_BUFFER_SIZE;
    edmaParam.cCnt          = (uint16_t) 1;
    edmaParam.bCntReload    = (uint16_t) 0;
    edmaParam.srcBIdx       = (int16_t) EDMA_PARAM_BIDX(0);
    edmaParam.destBIdx      = (int16_t) EDMA_PARAM_BIDX(number_soc_per_transfer * (NUM_BYTES_IN_SOC_RESULT));
    edmaParam.srcCIdx       = (int16_t) 0;
    edmaParam.destCIdx      = (int16_t) 0;
    edmaParam.linkAddr      = 0xFFFFU;
    edmaParam.srcBIdxExt    = (int8_t) EDMA_PARAM_BIDX_EXT(0);
    edmaParam.destBIdxExt   = (int8_t) EDMA_PARAM_BIDX_EXT(number_soc_per_transfer * (NUM_BYTES_IN_SOC_RESULT));
    /* Enabling the Interrupt for Transfer complete of all the data */
    edmaParam.opt          |=
        (EDMA_OPT_TCINTEN_MASK |
         ((((uint32_t)tcc) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));
    
    EDMA_setPaRAM(baseAddr, param, &edmaParam);

    /* 
    Note that these transfers end after RESULTS_BUFFER_SIZE number of triggers are reached
    If the application needs the dma transfers to keep on continuing forever, it is recommend to extend 
    the DMA configuration to have a linked transfer like param1 links to param2, param2 links to itself, 
    refer to the linked transfer example from the EDMA. 
    */

    /* Enabling the EDMA transfers for the EVENT. EVENT is ADCx_INT, triggered at tDMA */
    EDMA_enableTransferRegion(baseAddr, regId, dma_ch,
                              EDMA_TRIG_MODE_EVENT);
}


void App_dmach0ISR(Edma_IntrHandle intrHandle, void *args)
{
    SemaphoreP_Object *semObjPtr = (SemaphoreP_Object *)args;
    DebugP_assert(semObjPtr != NULL);

    /* Stop the ADCs by removing the trigger for SOC0 */
    ADC_setInterruptSOCTrigger(gAdc0BaseAddr, ADC_SOC_NUMBER0,
                               ADC_INT_SOC_TRIGGER_NONE);
    ADC_setInterruptSOCTrigger(gAdc1BaseAddr, ADC_SOC_NUMBER0,
                               ADC_INT_SOC_TRIGGER_NONE);

    /* Post the semaphore to signal end of DMA transfer */
    SemaphoreP_post(semObjPtr);
}