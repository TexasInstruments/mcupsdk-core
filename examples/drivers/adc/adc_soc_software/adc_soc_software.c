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
#include <drivers/adc.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/*
 * This example uses the ADC module to perform an ADC SOC conversion
 * triggered by software.
 *
 * In this example ADC0 is used to convert the SOC, the user can also
 * select a different one.
 *
 * This example also showcases how to configure and use the ADC module.
 */

void adc_soc_software_main(void *args)
{
    uint32_t baseAddr = CONFIG_ADC0_BASE_ADDR;
    uint32_t loopCnt = 10;

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    DebugP_log("ADC Software Triggered Conversion Test Started ...\r\n");

    while(loopCnt--)
    {
        /* Clear any pending interrupts */
        ADC_clearInterruptStatus(baseAddr, ADC_INT_NUMBER1);

        ADC_forceSOC(baseAddr, ADC_SOC_NUMBER0);
        while(ADC_getInterruptStatus(baseAddr, ADC_INT_NUMBER1) == false)
        {
            /* Wait for the SOC conversion to complete */
        }

        DebugP_log("ADC Result register value : %d\r\n", ADC_readResult(CONFIG_ADC0_RESULT_BASE_ADDR, ADC_SOC_NUMBER0));
    }

    /* Clear and disable interrupt */
    ADC_disableInterrupt(baseAddr, ADC_INT_NUMBER1);
    ADC_clearInterruptStatus(baseAddr, ADC_INT_NUMBER1);
    /* Power down the ADC */
    ADC_disableConverter(baseAddr);

    DebugP_log("ADC Software Triggered Conversion Test Passed!!\r\n");
    DebugP_log("All tests have passed!!\r\n");

    Board_driversClose();
    Drivers_close();
}
