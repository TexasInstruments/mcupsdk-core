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
#include <drivers/pinmux.h>
#include <drivers/soc.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"


/* Defines*/
#define MAX_SAMPLES    (256)
#define FIFO_DEPTH     (16)         // configured in Syscfg.
/* Globals*/
int16_t  filter2Result[MAX_SAMPLES/FIFO_DEPTH][FIFO_DEPTH];
int16_t  filter3Result[MAX_SAMPLES];

/* Function Prototypes*/
void done(void);
static void sdfmISR(void *handle);

static SemaphoreP_Object gSdfmSemObject;
static HwiP_Object       gSdfmHwiObject;

volatile uint32_t gIsrCount = 0;
uint32_t gSdfmBase = CONFIG_SDFM0_BASE_ADDR;

/*
 * Example Description
 *      ECAP in its APWM mode can be used a Clock for the SDFM. This example
 * demonstrates the configurations and usage of such internal loopback.
 * In AM263Px, the loopback configurations can be done to leverage the following
 * pairs.
 * for the Loopback configurations please refer to the example docs 
 * In this example, SDFM filters 2,3 are configured for the clock input to have
 * from filter 1 and filter 1 is configured to take the loopback clock. The Data
 * interrupts from the SDFM filter 2,3 invoke the ISR, where the CPU reads the data
 * form FIFO of Filter 2 and Data result of Filter 3.
 *
 * SDFM configuration is shown below:
 *  -  SDFM used in this example - SDFM0
 *  -  Input control mode selected - MODE0
 *  -  Data filter settings
 *      - filters 2,3  modules enabled
 *      - Sinc1 filter selected
 *      - OSR = 256
 *      - All the filters are synchronized by using MFE (Main Filter enable bit)
 *      - Filter output represented in 16 bit format

 *  - Interrupt module settings for SDFM filter
 *      - FIFO full interrupt from Filter 2 and Data ready from Filter 3 are enabled.
 *
 * External Connections
 *   - Connect Sigma-Delta data streams to SDFM0_D1, SDFM0_D2
 *   - Output xbar out 8 or SDFM0_CLK0 can be used for providing clock to external
 *     sdfm modulator.
 *
 * Watch  Variables
 * -   filter2Result - Output of filter 2
 * -   filter3Result - Output of filter 3
 *
 */

// void App_configSDFMClkPinMux(uint32_t config); // this configuration has been moved to syscfg

void sdfm_ecap_loop_back_main(void *args)
{
    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    /* enabling the Pinmux */

    DebugP_log("SDFM filter sync CPU read Test Started ...\r\n");

    /* ECAP is the clock source, and has been set in APWM mode. the EPWM is used as DATA can be synced with ECAP*/
    /* syncing the DATA EPWM */
    ECAP_loadCounter(CONFIG_ECAP0_BASE_ADDR);

    /* Register & enable interrupt */
    HwiP_Params         hwiPrms;
    int32_t             status;

    HwiP_Params_init(&hwiPrms);
    /* Integrate with Syscfg */
    hwiPrms.intNum      = CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_0;
    hwiPrms.callback    = &sdfmISR;
    /* Integrate with Syscfg */
    hwiPrms.isPulse     = 1;
    status              = HwiP_construct(&gSdfmHwiObject, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);

    status = SemaphoreP_constructBinary(&gSdfmSemObject, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    SDFM_clearInterruptFlag(gSdfmBase, SDFM_MAIN_INTERRUPT_FLAG | SDFM_FILTER_3_NEW_DATA_FLAG | SDFM_FILTER_2_FIFO_INTERRUPT_FLAG | 0xFFFF);

    /* Wait for interrupt*/

    SemaphoreP_pend(&gSdfmSemObject, SystemP_WAIT_FOREVER);

    DebugP_log("Max Samples read complete. Printing some of the values...\r\n");
    for(int iter = 0; iter < MAX_SAMPLES; iter+= MAX_SAMPLES/5)
    {
        DebugP_log("Filter out : \r\n");
        for(int iter2 = 0; ((iter/FIFO_DEPTH+iter2) < MAX_SAMPLES)&& (iter2<FIFO_DEPTH); iter2++)
        {
            DebugP_log(" %d", filter3Result[iter/FIFO_DEPTH+iter2]);
        }
        DebugP_log("\r\n");
        DebugP_log("Corresponding FIFO DATA\r\n");
        for(int iter2 = 0; iter2 <FIFO_DEPTH; iter2++)
        {
            DebugP_log(" %d", filter2Result[iter/FIFO_DEPTH][iter2]);
        }
        DebugP_log("\r\n\r\n");
    }

    DebugP_log("All tests have passed!!\r\n");

    SemaphoreP_destruct(&gSdfmSemObject);

    Board_driversClose();
    Drivers_close();
}

/* sdfmISR - SDFM ISR*/

static void sdfmISR(void *handle)
{

    if(gIsrCount < MAX_SAMPLES)
    {
        if(true == SDFM_getFIFOISRStatus(gSdfmBase, SDFM_FILTER_2))
        {
            /* read the FIFO Data */
            for(int iter = 0; iter < FIFO_DEPTH; iter++)
            {
                filter2Result[gIsrCount/FIFO_DEPTH][iter] = (int16_t) (SDFM_getFIFOData(gSdfmBase, SDFM_FILTER_2) >> 16U);
            }

            SDFM_clearInterruptFlag(gSdfmBase, SDFM_MAIN_INTERRUPT_FLAG  | SDFM_FILTER_2_FIFO_INTERRUPT_FLAG  | 0xFFF);

        }
        if(true == SDFM_getNewFilterDataStatus(gSdfmBase, SDFM_FILTER_3))
        {
            /* read the SDFM filter Data */
            filter3Result[gIsrCount] = (int16_t)(SDFM_getFilterData(gSdfmBase, SDFM_FILTER_3) >> 16U);
            gIsrCount++;
        }
        SDFM_clearInterruptFlag(gSdfmBase, SDFM_MAIN_INTERRUPT_FLAG  |  SDFM_FILTER_3_NEW_DATA_FLAG  | 0xFFF);
    }
    else
    {
        SemaphoreP_post(&gSdfmSemObject);
    }
}
