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
#include <kernel/dpl/ClockP.h>
#include <drivers/dac.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/*
 * Example Description:
 *      The DAC Shadow value syncs to DAC Active Value from EPWM Event. 
 * The EPWM Peripheral can send out a EPWMSYNCPER signal, on certain events,
 * (check syscfg, under timebase submodule for more details on events) which
 * may be used by the DAC peripheral to sync its shadow to active register 
 * transfers.
 *      The example showcases this in the EPWM ISR, where the event for sync
 *  signal and the event for the interrupt are same. now, if there is a new
 * shadow value set, it wouldn't reflect in the active until the next ISR
 * iteration. and similarly, the last set shadow value would appear in the 
 * active register of the DAC.
 * 
 * Note that, the events for EPWMSYNCPER signal and the Interrupt signal from 
 * EPWM are not dependant, in other words, the EPWMSYNCPER may be configured
 * without the need to configure the INT signal from the EPWM. 
 * 
 * Configurations
 *      DAC loadmode is set to the EPWM SYNC PER signal.
 *      EPWM SYNCPER signal is generated at its counter equals to period event
 *
 * External connections
 *      The status Pin, EPWM output A, DAC output may be probed to view the 
 * waveforms
 *          - On AM263x-CC or AM263Px-CC, with HSEC dock,
 *              - Probe the HSEC Pin 9 for DAC output.
 *              - Probe the HSEC Pin 49 for EPWM A output.
 *              - Probe the HSEC Pin 51 for STATUS (GPIO 44) output.
 *          - On AM263x-LP or AM263Px-LP,
 *              - Probe the J3 Pin 30
 *              - Probe the J4 Pin 11 for EPWM A output.
 *              - Probe the J8 Pin 59 for STATUS (GPIO 44) output.
 * 
 */
#define TEST_SHADOW_VALUES  (10)
#define EPWM_ISR_COUNT_MAX  (TEST_SHADOW_VALUES)

static HwiP_Object gEpwmHwiObject;

uint16_t gDacShadowValues[TEST_SHADOW_VALUES];
volatile uint16_t gEpwmIsrCount = 0;
volatile int32_t  gTestStatus   = SystemP_SUCCESS;

uint32_t gEpwmBaseAddr = CONFIG_EPWM0_BASE_ADDR;
uint32_t gDacBaseAddr = CONFIG_DAC0_BASE_ADDR;

void App_epwmIsr(void* args);

void dac_epwm_sync_main(void *args)
{
    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();
    
    DebugP_log("EPWM synced DAC Test Started ...\r\n");

    /* initiating the gDacShadowValues array */
    for(uint16_t iter = 0; iter < TEST_SHADOW_VALUES; iter++)
    {
        gDacShadowValues[iter] = (iter*4095)/TEST_SHADOW_VALUES;
    }

    /* Setting the status pin to low for initialisation */
    GPIO_pinWriteLow(STATUS_BASE_ADDR, STATUS_PIN);

    /* Registering the EPWM ISR */
    int32_t     status;
    HwiP_Params hwiPrms;

    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_0;
    hwiPrms.callback    = &App_epwmIsr;
    hwiPrms.priority    = 0;
    status              = HwiP_construct(&gEpwmHwiObject, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);

    /* setting the DAC shadow value */
    DAC_setShadowValue(gDacBaseAddr, gDacShadowValues[gEpwmIsrCount]);

    /* setting the STATUS pin to high for indicating the Shadow Value set */
    GPIO_pinWriteHigh(STATUS_BASE_ADDR, STATUS_PIN);

    /* Clearing any interrupt flags, for the EPWM INT ISR trigger */
    EPWM_clearEventTriggerInterruptFlag(gEpwmBaseAddr);

    /* setting the time base counter mode of the EPWM to up count. essentially running the EPWM */
    EPWM_setTimeBaseCounterMode(gEpwmBaseAddr, EPWM_COUNTER_MODE_UP);

    /* wait for EPWM_ISR_COUNT_MAX number of EPWM ISRs */
    while(gEpwmIsrCount < EPWM_ISR_COUNT_MAX)
    {
        if(gTestStatus == SystemP_FAILURE)
        {
            EPWM_disableInterrupt(gEpwmBaseAddr);
            DebugP_log("EPWM synced DAC test failed!! %d\r\n", gEpwmIsrCount);
            break;
        }
    }

    if(gTestStatus == SystemP_SUCCESS)
    {
        DebugP_log("EPWM synced DAC Test Passed!!\r\n");
        DebugP_log("All tests have passed!!\r\n");
    }
    else
    {
        DebugP_log("Some tests have failed!!\r\n");
    }

    /* Freezing the EPWM time base counter so further sync will not occur */
    EPWM_setTimeBaseCounterMode(gEpwmBaseAddr, EPWM_COUNTER_MODE_STOP_FREEZE);

    Board_driversClose();
    Drivers_close();
}

void App_epwmIsr(void* args)
{   
    /* 
    EPWM output A is set when the counter hits Period. so the DAC output 
    should have changed to its shadow value along with the EPWM waveform. 
    clearing the STATUS pin. Note that there could be latency from the EPWM INT to EPWM ISR 
    */
    GPIO_pinWriteLow(STATUS_BASE_ADDR, STATUS_PIN);

    /* The Shadow value set previously should have loaded to active value by this EPWM event */
    if(gDacShadowValues[gEpwmIsrCount] == DAC_getActiveValue(gDacBaseAddr))
    {
        gTestStatus   = SystemP_SUCCESS;

        gEpwmIsrCount++;
        if(gEpwmIsrCount >= EPWM_ISR_COUNT_MAX)
        {
            EPWM_disableInterrupt(gEpwmBaseAddr);
        }
        else
        {
            DAC_setShadowValue(gDacBaseAddr, gDacShadowValues[gEpwmIsrCount]);
            
            /* new Shadow value is written. changing the STATUS pin to HIGH */
            GPIO_pinWriteHigh(STATUS_BASE_ADDR, STATUS_PIN);
            
            if (gDacShadowValues[gEpwmIsrCount] == DAC_getActiveValue(gDacBaseAddr))
            {
                /* the new shadow value is set now, and should not get to active value until next EPWM event */
                gTestStatus   = SystemP_FAILURE;
            }
            else
            {
                /* Do Nothing */
            }
        }
    }

    if(gEpwmIsrCount >= EPWM_ISR_COUNT_MAX)
    {
        EPWM_disableInterrupt(gEpwmBaseAddr);
    }

    EPWM_getEventTriggerInterruptStatus(gEpwmBaseAddr);
    if(true == EPWM_getEventTriggerInterruptStatus(gEpwmBaseAddr))
    {
        EPWM_clearEventTriggerInterruptFlag(gEpwmBaseAddr);
    }

}
