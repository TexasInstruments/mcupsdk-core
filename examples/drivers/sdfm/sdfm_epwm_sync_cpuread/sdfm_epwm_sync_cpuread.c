/*
 *  Copyright (C) 2021-2023 Texas Instruments Incorporated
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
#include <drivers/sdfm.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/*
 * This example uses the SDFM module to perform an EPWM sync CPU read.
 */

/* Defines */
#define SDFM_INT_MASK  (0x8000F000U)
#define APP_RUN_TIME  (100)

/* Globals */
int16_t  filter1Result;
int16_t  filter2Result;
int16_t  filter3Result;
int16_t  filter4Result;

/* Function protoypes */
static void sdfmISR1(void *handle);
static void initSDFM(uint32_t sdfmInstance);
static HwiP_Object gSdfmHwiObject;
uint32_t gSdfmBaseAddr;
uint32_t gClockEpwmBaseAddr;
uint32_t gSyncEpwmBaseAddr;

volatile int32_t gInterruptCount;
volatile uint16_t gSdfmDataTimestamp;

void sdfm_epwm_sync_cpuread_main(void *args)
{
    gSdfmBaseAddr = CONFIG_SDFM0_BASE_ADDR;
    gClockEpwmBaseAddr = CONFIG_EPWM0_BASE_ADDR;
	gSyncEpwmBaseAddr = CONFIG_EPWM1_BASE_ADDR;

    DebugP_log("SDFM EPWM Sync CPU Read Test Started ...\r\n");
	DebugP_log("Please wait %d seconds ...\r\n", APP_RUN_TIME);

    /* Open drivers
		- Open the UART driver for console
		- Open SDFM and EPWM driver
		- Open XBAR driver for SDFM interrupt routing to R5F
	*/
    Drivers_open();
    Board_driversOpen();

    /* Register and enable interrupt */
    HwiP_Params hwiPrms;
    int32_t status;

    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_0;
    hwiPrms.callback    = &sdfmISR1;
    hwiPrms.isPulse     = 1;
    status              = HwiP_construct(&gSdfmHwiObject, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);

	/* Initialize SDFM*/
    initSDFM(gSdfmBaseAddr);

	gInterruptCount = 0;

	/* Check interrupt and data*/
	uint32_t loopCount = 0;
    while(loopCount < APP_RUN_TIME)
    {
		DebugP_log("\r\nSDFM ISR count: %u\r\n", gInterruptCount);
		gInterruptCount=0;

		if( (SDFM_getModulatorStatus(gSdfmBaseAddr, SDFM_FILTER_1) == true ) &&
			(SDFM_getModulatorStatus(gSdfmBaseAddr, SDFM_FILTER_2) == true ) &&
			(SDFM_getModulatorStatus(gSdfmBaseAddr, SDFM_FILTER_3) == true ) &&
			(SDFM_getModulatorStatus(gSdfmBaseAddr, SDFM_FILTER_4) == true ))
		{
			DebugP_log("Modulator operating normally\r\n");

			/* Read and display result values */
			DebugP_log("Filter 1 data : %d \r\n", filter1Result);
			DebugP_log("Filter 2 data : %d \r\n", filter2Result);
			DebugP_log("Filter 3 data : %d \r\n", filter3Result);
			DebugP_log("Filter 4 data : %d \r\n", filter4Result);

			/*DebugP_log("SDFM ISR timestamp (Sync EPWM Timebase counter value) = %u \r\n", gSdfmDataTimestamp);*/
		}
		else
		{
			DebugP_log("Modulator failure\r\n");
		}

		ClockP_sleep(1);
		loopCount++;
    }

	if(	(SDFM_getModulatorStatus(gSdfmBaseAddr, SDFM_FILTER_1) == true ) &&
		(SDFM_getModulatorStatus(gSdfmBaseAddr, SDFM_FILTER_2) == true ) &&
		(SDFM_getModulatorStatus(gSdfmBaseAddr, SDFM_FILTER_3) == true ) &&
		(SDFM_getModulatorStatus(gSdfmBaseAddr, SDFM_FILTER_4) == true ))
	{
		/*Pass if SDFM ISR period matches configured EPWM SOCA period 65535*5ns = 327.675us => 3051Hz.
			Sync EPWM Period configured using Sysconfig
		  Fail if SDFM ISR period matches (without sync) SD clock freq/OSR = 1MHz/256 = 3906.25Hz.
			SD clock frequency (Clock EPWM) configured using Sysconfig.
		*/

		if( ((gInterruptCount -3051) < 50) && ((gInterruptCount -3051) > -50) )
		{
			DebugP_log("SDFM EPWM Sync CPU Read Test Passed!!\r\n");
			DebugP_log("All tests have passed!!\r\n");
		}
		else
		{
			DebugP_log("Sync Fail!!\r\n");
		}
	}
	else
	{
		DebugP_log("Fail. Modulator failure!!\r\n");
	}

    Board_driversClose();
    Drivers_close();
}

/* SDFM 1 ISR */
static void sdfmISR1(void *handle)
{
	gSdfmDataTimestamp = EPWM_getTimeBaseCounterValue(gSyncEpwmBaseAddr);

    /* Read each SDFM filter output and store it in respective filter result array */
    if(SDFM_getNewFilterDataStatus(gSdfmBaseAddr, SDFM_FILTER_1) == true)
	{
		filter1Result = SDFM_getFilterData(gSdfmBaseAddr, SDFM_FILTER_1) >> CSL_SDFM_SDDATA1_DATA32HI_SHIFT;
	}
    if(SDFM_getNewFilterDataStatus(gSdfmBaseAddr, SDFM_FILTER_2) == true)
	{
		filter2Result = SDFM_getFilterData(gSdfmBaseAddr, SDFM_FILTER_2) >> CSL_SDFM_SDDATA1_DATA32HI_SHIFT;
	}
    if(SDFM_getNewFilterDataStatus(gSdfmBaseAddr, SDFM_FILTER_3) == true)
	{
		filter3Result = SDFM_getFilterData(gSdfmBaseAddr, SDFM_FILTER_3) >> CSL_SDFM_SDDATA1_DATA32HI_SHIFT;
	}
	if(SDFM_getNewFilterDataStatus(gSdfmBaseAddr, SDFM_FILTER_4) == true)
	{
		filter4Result = SDFM_getFilterData(gSdfmBaseAddr, SDFM_FILTER_4) >> CSL_SDFM_SDDATA1_DATA32HI_SHIFT;
	}

	gInterruptCount++;

    /* Clear SDFM flag register */
    SDFM_clearInterruptFlag(gSdfmBaseAddr, SDFM_INT_MASK);
}

static void initSDFM(uint32_t sdfmInstance)
{
    /* Configure Modulator Clock rate = Modulator data rate */
    SDFM_setupModulatorClock(sdfmInstance, SDFM_FILTER_1, SDFM_MODULATOR_CLK_EQUAL_DATA_RATE);
    SDFM_setupModulatorClock(sdfmInstance, SDFM_FILTER_2, SDFM_MODULATOR_CLK_EQUAL_DATA_RATE);
    SDFM_setupModulatorClock(sdfmInstance, SDFM_FILTER_3, SDFM_MODULATOR_CLK_EQUAL_DATA_RATE);
    SDFM_setupModulatorClock(sdfmInstance, SDFM_FILTER_4, SDFM_MODULATOR_CLK_EQUAL_DATA_RATE);

    /* Over value threshold settings */
    uint16_t hlt = 0x7FFF;
    /* Under value threshold settings */
    uint16_t llt = 0x0000;

    /* Configure Comparator module's comparator filter type and comparator's OSR
       value, higher threshold, lower threshold */
    SDFM_configComparator(sdfmInstance,
        (SDFM_FILTER_1 | SDFM_FILTER_SINC_3 | SDFM_SET_OSR(32)),
        (SDFM_THRESHOLD(hlt,llt)), 0);
    SDFM_configComparator(sdfmInstance,
        (SDFM_FILTER_2 | SDFM_FILTER_SINC_3 | SDFM_SET_OSR(32)),
        (SDFM_THRESHOLD(hlt,llt)), 0);
    SDFM_configComparator(sdfmInstance,
        (SDFM_FILTER_3 | SDFM_FILTER_SINC_3 | SDFM_SET_OSR(32)),
        (SDFM_THRESHOLD(hlt,llt)), 0);
    SDFM_configComparator(sdfmInstance,
        (SDFM_FILTER_4 | SDFM_FILTER_SINC_3 | SDFM_SET_OSR(32)),
        (SDFM_THRESHOLD(hlt,llt)), 0);

    /* Configure Data filter modules filter type, OSR value and enable / disable data filter */
    SDFM_configDataFilter(sdfmInstance, (SDFM_FILTER_1 | SDFM_FILTER_SINC_3 |
           SDFM_SET_OSR(256)), (SDFM_DATA_FORMAT_16_BIT | SDFM_FILTER_ENABLE |
           SDFM_SHIFT_VALUE(0x000A)));

    SDFM_configDataFilter(sdfmInstance, (SDFM_FILTER_2 | SDFM_FILTER_SINC_3 |
           SDFM_SET_OSR(256)), (SDFM_DATA_FORMAT_16_BIT | SDFM_FILTER_ENABLE |
           SDFM_SHIFT_VALUE(0x000A)));

    SDFM_configDataFilter(sdfmInstance, (SDFM_FILTER_3 | SDFM_FILTER_SINC_3 |
           SDFM_SET_OSR(256)), (SDFM_DATA_FORMAT_16_BIT | SDFM_FILTER_ENABLE |
           SDFM_SHIFT_VALUE(0x000A)));

    SDFM_configDataFilter(sdfmInstance, (SDFM_FILTER_4 | SDFM_FILTER_SINC_3 |
           SDFM_SET_OSR(256)), (SDFM_DATA_FORMAT_16_BIT | SDFM_FILTER_ENABLE |
           SDFM_SHIFT_VALUE(0x000A)));

    /* Enable Main filter bit: Unless this bit is set none of the filter modules
       can be enabled. All the filter modules are synchronized when main filter
       bit is enabled after individual filter modules are enabled. */
    SDFM_enableMainFilter(sdfmInstance);

    /* Enable data filter to be reset on EPWM sync */
    SDFM_enableExternalReset(sdfmInstance, SDFM_FILTER_1);
    SDFM_enableExternalReset(sdfmInstance, SDFM_FILTER_2);
    SDFM_enableExternalReset(sdfmInstance, SDFM_FILTER_3);
    SDFM_enableExternalReset(sdfmInstance, SDFM_FILTER_4);

    SDFM_setPWMSyncSource(sdfmInstance, SDFM_FILTER_1, SDFM_SYNC_PWM1_SOCA);
    SDFM_setPWMSyncSource(sdfmInstance, SDFM_FILTER_2, SDFM_SYNC_PWM1_SOCA);
    SDFM_setPWMSyncSource(sdfmInstance, SDFM_FILTER_3, SDFM_SYNC_PWM1_SOCA);
    SDFM_setPWMSyncSource(sdfmInstance, SDFM_FILTER_4, SDFM_SYNC_PWM1_SOCA);

    SDFM_selectClockSource(sdfmInstance, SDFM_FILTER_2, SDFM_CLK_SOURCE_SD1_CLK);
    SDFM_selectClockSource(sdfmInstance, SDFM_FILTER_3, SDFM_CLK_SOURCE_SD1_CLK);
    SDFM_selectClockSource(sdfmInstance, SDFM_FILTER_4, SDFM_CLK_SOURCE_SD1_CLK);

    /* Following SDFM interrupts can be enabled / disabled using this function.
       Enable / disable comparator high threshold
       Enable / disable comparator low threshold
       Enable / disable modulator clock failure
       Enable / disable filter acknowledge */
    SDFM_enableInterrupt(sdfmInstance, SDFM_FILTER_1,
            (SDFM_MODULATOR_FAILURE_INTERRUPT |
             SDFM_DATA_FILTER_ACKNOWLEDGE_INTERRUPT));

    SDFM_enableInterrupt(sdfmInstance, SDFM_FILTER_2,
            (SDFM_MODULATOR_FAILURE_INTERRUPT |
             SDFM_DATA_FILTER_ACKNOWLEDGE_INTERRUPT));

    SDFM_enableInterrupt(sdfmInstance, SDFM_FILTER_3,
            (SDFM_MODULATOR_FAILURE_INTERRUPT |
             SDFM_DATA_FILTER_ACKNOWLEDGE_INTERRUPT));

    SDFM_enableInterrupt(sdfmInstance, SDFM_FILTER_4,
            (SDFM_MODULATOR_FAILURE_INTERRUPT |
             SDFM_DATA_FILTER_ACKNOWLEDGE_INTERRUPT));

    SDFM_disableInterrupt(sdfmInstance, SDFM_FILTER_1,
            (SDFM_CEVT2_INTERRUPT |
             SDFM_CEVT1_INTERRUPT));

    SDFM_disableInterrupt(sdfmInstance, SDFM_FILTER_2,
            (SDFM_CEVT2_INTERRUPT |
             SDFM_CEVT1_INTERRUPT));

    SDFM_disableInterrupt(sdfmInstance, SDFM_FILTER_3,
            (SDFM_CEVT2_INTERRUPT |
             SDFM_CEVT1_INTERRUPT));

    SDFM_disableInterrupt(sdfmInstance, SDFM_FILTER_4,
            (SDFM_CEVT2_INTERRUPT |
             SDFM_CEVT1_INTERRUPT));

    /* Enable main interrupt so that any of the filter interrupts can trigger
       by SDFM interrupt to CPU */
    SDFM_enableMainInterrupt(sdfmInstance);
}