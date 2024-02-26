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
#include <kernel/dpl/ClockP.h>
#include <drivers/sdfm.h>
#include <drivers/epwm.h>
#include <drivers/soc.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"


/*
 * A SDFM example that reads filter data from CPU

 * In this example, SDFM filter data is read by DMA by SDFM FIFO DATA READTY INT trigger.
  * SDFM configuration is shown below:
 *  -  SDFM used in this example - SDFM0
 *  -  Input control mode selected - MODE0
 *  -  Data filter settings
 *      - Filter 1 is enabled
 *      - Sinc3 filter selected
 *      - OSR = 256
 *      - Synchronized by using MFE
 *       (Main Filter enable bit)
 *      - Filter output represented in 16 bit format
 *      - In order to convert 26 bit Data filter
 *        into 16 bit format user needs to right shift by 10 bits for
 *        Sinc3 filter with OSR = 256
 *      - FIFO is enabled for 16 levels. 
 *  - Interrupt module settings for SDFM filter
 *      - Filter 1 will generate interrupt when a new filter FIFO Threshold is crossed.
 *
 * External Connections
 *   -  Connect Sigma-Delta streams to
 *     SDFM0_CLK0, SDFM0_D0, SDFM0_CLK1, SDFM0_D1, SDFM0_CLK2, SDFM0_D2, SDFM0_CLK3, SDFM0_D3
 *
 * Watch  Variables
 * -   filter1Result - Output of filter 1
 */

/* Defines*/

#define NUM_FIFO_REG_PER_TRANSFER (16U)

/* 32 bit fifo register. valid data could be 16 or 32 depending on the configurations */
#define NUM_BYTES_IN_FIFO_REGISTER (2U)         // selecting 16 bit mode

/* Total number of 32 FIFO results to be copied */
#define RESULTS_BUFFER_SIZE (64U)

/* Total Buffer Size */
#define RESULTS_BUFFER_TOTAL_SIZE (RESULTS_BUFFER_SIZE * NUM_FIFO_REG_PER_TRANSFER * NUM_BYTES_IN_FIFO_REGISTER)

/* DMA channel number to transfer SDFM conversion results*/
#define SDFM_EDMA_CHANNEL_0 (DMA_TRIG_XBAR_EDMA_MODULE_0)

/* Event queue to be used for EDMA transfer */
#define EDMA_TEST_EVT_QUEUE_NO_0 (0U)

/* Globals*/
int16_t filter1Result[RESULTS_BUFFER_SIZE][NUM_FIFO_REG_PER_TRANSFER];
int16_t filter1Result2[RESULTS_BUFFER_SIZE][NUM_FIFO_REG_PER_TRANSFER];
uint32_t gSdfmFifo1Base = CONFIG_SDFM0_BASE_ADDR + CSL_SDFM_SDDATFIFO1;

/* Function Prototypes*/

// static void sdfmISR(void *handle);

/**
 * @brief Configures DMA in AB Transfer mode
 *
 * @param dest_table             : this will the buffer in the memory to store the SDFM FIFO transfer values
 * @param src_fifoAddr           : this is to mention fifo memories will be used for transfer.
 * @param fifo_size_in_transfer  : Number of consecutive FIFO Registers to be transfered in one go.
 * @param dma_ch                 : Which DMA channel is used for the transfer
 * @param tccAlloc               : Which tcc, to hold the channel transfer information. can be used to register interrupt
 * @param event_queue_number     : Whith event queue to be used for the transfer. 2 independnent queue may used at max
 */
static void App_dmaConfigure(
    uint32_t dest_table,
    uint32_t src_fifoAddr,
    uint16_t fifo_size_in_transfer,
    uint32_t dma_ch,
    uint32_t *tccAlloc,
    uint32_t event_queue_number);

void App_dmach0ISR(Edma_IntrHandle intrHandle, void *args);


static SemaphoreP_Object gEdmaTransferDoneSem;
uint32_t tccAlloc0;

void sdfm_filter_sync_dmaread(void *args)
{
   /* Open drivers to open the UART driver for console */
   Drivers_open();
   Board_driversOpen();

   DebugP_log("SDFM filter sync CPU read Test Started ...\r\n");

   /* Initializing the Results Buffer */
   for (int transfer_level = 0; transfer_level < RESULTS_BUFFER_SIZE; transfer_level++)
   {
      for (int result_index = 0; result_index < NUM_FIFO_REG_PER_TRANSFER; result_index ++)
      {
         filter1Result[transfer_level][result_index] = 0;
      }
   }
   
   /* Perform a cache write back to the result buffers */
   CacheP_wb((void *)filter1Result, RESULTS_BUFFER_TOTAL_SIZE, CacheP_TYPE_ALL);

   /* using the 16-bit mode so, the data is upper 16 bit in the FIFO register */
   if(NUM_BYTES_IN_FIFO_REGISTER == 2)
   {
      gSdfmFifo1Base += 2U;   
   }

   DebugP_log("gSdfmFifo1Base : %x\r\n\r\n",gSdfmFifo1Base);

   int32_t status = SystemP_SUCCESS;
   /* Create a semaphore to signal EDMA transfer completion */
   status = SemaphoreP_constructBinary(&gEdmaTransferDoneSem, 0);
   DebugP_assert(SystemP_SUCCESS == status);

   /* Tcc for DMA interrupt Registration */
   Edma_IntrObject intrObj;
   /* Setting up DMA */
   App_dmaConfigure(
       (uint32_t) filter1Result,              // destination buffer
       (uint32_t) gSdfmFifo1Base,             // source
       NUM_FIFO_REG_PER_TRANSFER,             // number of FIFO Results in each transfer
       SDFM_EDMA_CHANNEL_0,                   // DMA Channel
       &tccAlloc0,
       EDMA_TEST_EVT_QUEUE_NO_0);
   

   /* Register interrupt for dma channel transfer completion*/
   intrObj.tccNum = tccAlloc0;
   intrObj.cbFxn = &App_dmach0ISR;
   intrObj.appData = (void *)&gEdmaTransferDoneSem;
   status = EDMA_registerIntr(gEdmaHandle[0], &intrObj);
   DebugP_assert(status == SystemP_SUCCESS);

   /* EDMA setup complete */

   /* Setting up SDFM data format */
   SDFM_setOutputDataFormat(CONFIG_SDFM0_BASE_ADDR, SDFM_FILTER_1, SDFM_DATA_FORMAT_16_BIT);
   SDFM_clearInterruptFlag(CONFIG_SDFM0_BASE_ADDR, SDFM_FILTER_1_FIFO_INTERRUPT_FLAG | 0xFFFF);

   /* Setting the EPWM Counter Mode to UP Count. In other words, setting the SDFM clock in. */
   EPWM_setTimeBaseCounterMode(CONFIG_EPWM0_BASE_ADDR, EPWM_COUNTER_MODE_UP);

   SemaphoreP_pend(&gEdmaTransferDoneSem, SystemP_WAIT_FOREVER);

   /* Invalidate destination buffer */
   CacheP_inv((void *)filter1Result, RESULTS_BUFFER_TOTAL_SIZE, CacheP_TYPE_ALL);
   
   DebugP_log("Some of the copied FIFO data...\r\n");

   for (int transfer_level = 0; transfer_level < RESULTS_BUFFER_SIZE; transfer_level+= RESULTS_BUFFER_SIZE/5)
   {
      DebugP_log("Transfer number : %d | Data : ",transfer_level);
      for (int result_index = 0; result_index < NUM_FIFO_REG_PER_TRANSFER; result_index ++)
      {
         DebugP_log("%d ",filter1Result[transfer_level][result_index]);
      }
      DebugP_log("\r\n");
   }
   
   DebugP_log("\r\nAll tests have passed!!\r\n");
   
   SemaphoreP_destruct(&gEdmaTransferDoneSem);

   Board_driversClose();
   Drivers_close();
}

static void App_dmaConfigure(
    uint32_t dest_table,
    uint32_t src_fifoAddr,
    uint16_t fifo_size_in_transfer,
    uint32_t dma_ch,
    uint32_t *tccAlloc,
    uint32_t event_queue_number)
{
   uint32_t baseAddr, regId, tcc, param;
   EDMACCPaRAMEntry edmaParam;

   int32_t status = SystemP_SUCCESS;

   baseAddr = EDMA_getBaseAddr(gEdmaHandle[0]);
   DebugP_assert(baseAddr != 0);

   regId = EDMA_getRegionId(gEdmaHandle[0]);
   DebugP_assert(regId < SOC_EDMA_NUM_REGIONS);

   // dma_ch = EDMA_RESOURCE_ALLOC_ANY;
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

   edmaParam.srcAddr       = (uint32_t)SOC_virtToPhy((void *)(src_fifoAddr));
   edmaParam.destAddr      = (uint32_t)SOC_virtToPhy((void *)dest_table);
   edmaParam.aCnt          = (uint16_t)(NUM_BYTES_IN_FIFO_REGISTER);
   edmaParam.bCnt          = (uint16_t)NUM_FIFO_REG_PER_TRANSFER;
   edmaParam.cCnt          = (uint16_t)RESULTS_BUFFER_SIZE;
   edmaParam.bCntReload    = (uint16_t)NUM_FIFO_REG_PER_TRANSFER;
   edmaParam.srcBIdx       = (int16_t)EDMA_PARAM_BIDX(0);
   edmaParam.destBIdx      = (int16_t)EDMA_PARAM_BIDX(NUM_BYTES_IN_FIFO_REGISTER);
   edmaParam.srcCIdx       = (int16_t)0;
   edmaParam.destCIdx      = (int16_t)NUM_FIFO_REG_PER_TRANSFER * NUM_BYTES_IN_FIFO_REGISTER;
   edmaParam.linkAddr      = 0xFFFFU;
   edmaParam.srcBIdxExt    = (int8_t)EDMA_PARAM_BIDX_EXT(0);
   edmaParam.destBIdxExt          = (int8_t)EDMA_PARAM_BIDX_EXT(0);
   /* Enabling the Interrupt for Transfer complete of all the data */
   edmaParam.opt |=
       (EDMA_OPT_TCINTEN_MASK | EDMA_OPT_ITCINTEN_MASK | EDMA_OPT_SYNCDIM_MASK |
        ((((uint32_t)tcc) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));

   EDMA_setPaRAM(baseAddr, param, &edmaParam);

   /*
   Note that these transfers end after RESULTS_BUFFER_SIZE number of triggers are reached
   If the application needs the dma transfers to keep on continuing forever, it is recommend to extend
   the DMA configuration to have a linked transfer like param1 links to param2, param2 links to itself,
   refer to the linked transfer example from the EDMA.
   */

   /* Enabling the EDMA transfers for the EVENT. EVENT is ADCx_INT, triggered at tDMA */
   EDMA_clrIntrRegion(baseAddr, regId, tccAlloc0);
   EDMA_enableTransferRegion(baseAddr, regId, dma_ch,
                             EDMA_TRIG_MODE_EVENT);
}

volatile uint32_t counter = 0;

void App_dmach0ISR(Edma_IntrHandle intrHandle, void *args)
{  
   uint32_t baseAddr, regId;
   baseAddr = EDMA_getBaseAddr(gEdmaHandle[0]);
   regId = EDMA_getRegionId(gEdmaHandle[0]);
   
   counter++;
   SemaphoreP_Object *semObjPtr = (SemaphoreP_Object *)args;
   DebugP_assert(semObjPtr != NULL);
   /* Post the semaphore to signal end of DMA transfer */

   if(counter < RESULTS_BUFFER_SIZE)
   {
      EDMA_clrIntrRegion(baseAddr, regId, tccAlloc0);
   }
   else
   {
      SemaphoreP_post(&gEdmaTransferDoneSem);
   }
   SDFM_clearInterruptFlag(CONFIG_SDFM0_BASE_ADDR, SDFM_FILTER_1_FIFO_INTERRUPT_FLAG | 0xFFFF);
}
