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


/* Defines*/
#define MAX_SAMPLES    (1024)

/* Globals*/
int16_t  filterResult[MAX_SAMPLES];

/* Function Prototypes*/
void done(void);
static void sdfmISR(void *handle);

static SemaphoreP_Object gSdfmSemObject;
static HwiP_Object       gSdfmHwiObject;


/*
 * A SDFM example that reads filter data from CPU

 * In this example, SDFM filter data is read by CPU in SDFM ISR routine. The
 * SDFM configuration is shown below:
 *  -  SDFM used in this example - SDFM0
 *  -  Input control mode selected - MODE0
 *  -  Comparator settings
 *       - Sinc3 filter selected
 *       - OSR = 32
 *       - HLT = 0x7FFF (Higher threshold setting)
 *       - LLT  = 0x0000(Lower threshold setting)
 *  -  Data filter settings
 *      - Single filter modules is enabled
 *      - Sinc3 filter selected
 *      - OSR = 128
 *      - single filter is synchronized by using MFE
 *       (Main Filter enable bit)
 *      - Filter output represented in 16 bit format
 *      - In order to convert 25 bit Data filter
 *        into 16 bit format user needs to right shift by 7 bits for
 *        Sinc3 filter with OSR = 128
 *  - Interrupt module settings for SDFM filter
 *      - Single higher threshold comparator interrupt disabled
 *      - Single lower threshold comparator interrupts disabled
 *      - Single modulator failure interrupt disabled
 *      - Single filter will generate interrupt when a new filter data
 *        is available.
 *
 * External Connections
 *   -  Connect Sigma-Delta streams to
 *     SDFM0_CLK1, SDFM0_D1
 *
 * Watch  Variables
 * -   filterResult - Output of filter 2
 *
 */

void sdfm_filter_sync_cpuread_single_channel(void *args)
{
    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    DebugP_log("SDFM filter sync CPU read Test Started ...\r\n");

    uint16_t  hlt, llt;

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

    /* Input Control Unit*/

    /* Configure Input Control Unit: Modulator Clock rate = Modulator data rate*/

    SDFM_setupModulatorClock(CONFIG_SDFM0_BASE_ADDR, SDFM_FILTER_2,
                             SDFM_MODULATOR_CLK_EQUAL_DATA_RATE);

    /* Comparator Unit - over and under value threshold settings*/

    hlt = 0x7FFF;
    llt = 0x0000;

    /* Configure Comparator Unit's comparator filter type and comparator's
       OSR value, higher threshold, lower threshold*/

    SDFM_configComparator(CONFIG_SDFM0_BASE_ADDR,
        (SDFM_FILTER_2 | SDFM_FILTER_SINC_3 | SDFM_SET_OSR(32)),
        (SDFM_THRESHOLD(hlt,llt)), 0);

    /* Data Filter Unit*/

    /* Configure Data Filter Unit - filter type, OSR value and
       enable / disable data filter*/

    SDFM_configDataFilter(CONFIG_SDFM0_BASE_ADDR, (SDFM_FILTER_2 | SDFM_FILTER_SINC_3 |
           SDFM_SET_OSR(128)), (SDFM_DATA_FORMAT_16_BIT | SDFM_FILTER_ENABLE |
           SDFM_SHIFT_VALUE(0x0007)));

    /* Enable Main filter bit: Unless this bit is set none of the filter modules
       can be enabled. All the filter modules are synchronized when main filter
       bit is enabled after individual filter modules are enabled.*/

    SDFM_enableMainFilter(CONFIG_SDFM0_BASE_ADDR);

    /* PWM11.CMPC, PWM11.CMPD, PWM12.CMPC and PWM12.CMPD signals cannot synchronize
     the filters. This option is not being used in this example.*/

    SDFM_disableExternalReset(CONFIG_SDFM0_BASE_ADDR, SDFM_FILTER_2);

    /* Enable interrupts*/

    /* Following SDFM interrupts can be enabled / disabled using this function.
       Enable / disable comparator high threshold
       Enable / disable comparator low threshold
       Enable / disable modulator clock failure
       Enable / disable data filter acknowledge*/

    SDFM_enableInterrupt(CONFIG_SDFM0_BASE_ADDR, SDFM_FILTER_2,
            (SDFM_MODULATOR_FAILURE_INTERRUPT |
             SDFM_DATA_FILTER_ACKNOWLEDGE_INTERRUPT));

    SDFM_disableInterrupt(CONFIG_SDFM0_BASE_ADDR, SDFM_FILTER_2,
            (SDFM_CEVT2_INTERRUPT |
             SDFM_CEVT1_INTERRUPT));

    /* Enable main interrupt so that any of the filter interrupts can trigger
       by SDFM interrupt to CPU*/

    SDFM_enableMainInterrupt(CONFIG_SDFM0_BASE_ADDR);

    /* Wait for interrupt*/

    SemaphoreP_pend(&gSdfmSemObject, SystemP_WAIT_FOREVER);

    DebugP_log("All tests have passed!!\r\n");

    SemaphoreP_destruct(&gSdfmSemObject);

    Board_driversClose();
    Drivers_close();
}

/* sdfmISR - SDFM ISR*/

static void sdfmISR(void *handle)
{
    static uint16_t loopCounter1 = 0;

    SDFM_setOutputDataFormat(CONFIG_SDFM0_BASE_ADDR, SDFM_FILTER_2,
                             SDFM_DATA_FORMAT_16_BIT);

    /* Read SDFM flag register (SDIFLG)*/

    HW_RD_REG32(CONFIG_SDFM0_BASE_ADDR + CSL_SDFM_SDIFLG);

    if(loopCounter1 < MAX_SAMPLES)
    {

        /* Read each SDFM filter output and store it in respective filter
           result array*/

        filterResult[loopCounter1++] =
              (int16_t)(SDFM_getFilterData(CONFIG_SDFM0_BASE_ADDR, SDFM_FILTER_2) >> 16U);

        /* Clear SDFM flag register (SDIFLG)*/

        SDFM_clearInterruptFlag(CONFIG_SDFM0_BASE_ADDR, SDFM_MAIN_INTERRUPT_FLAG |
                                            0xFFFF);

        /* Read SDFM flag register (SDIFLG)*/

        HW_RD_REG32(CONFIG_SDFM0_BASE_ADDR + CSL_SDFM_SDIFLG);
    }
    else
    {
        SemaphoreP_post(&gSdfmSemObject);
    }

}
