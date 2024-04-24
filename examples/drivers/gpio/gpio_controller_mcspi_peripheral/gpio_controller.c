/*
 *  Copyright (C) 2024 Texas Instruments Incorporated
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

#include <stdio.h>
#include <kernel/dpl/DebugP.h>
#include "am263x-cc/r5fss0-0_nortos/gpio_controller.h"

/*
 * This example demonstrates the usage of GPIO module to emulate the SPI
 * controller on R5FSS0_0.
 * MCSPI module is configured as a peripheral on R5FSS0_1 to simulate a
 * real controller-peripheral environment.
 *
 * 4 different GPIO pins are configured to emulate SPI controller by bitbanging the data.
 * GPIO43 as SPIPICO, GPIO44 as SPIPOCI, GPIO45 as SPICLK & GPIO46 as Chip Select(CS)
 * It demonstrates 8-bit and 32-bit full-duplex transfers, with configurable word and
 * message sizes.
 *
 * The transfer is done in the worst case scenario at a mean frequency of 6.25MHz,
 * the controller SPICLK frequency can be fixed by using appropriate delays.
 *
 * When transfer is completed, TX and RX buffer data are compared.
 * If data is matched, test result is passed otherwise failed.

 * External Connections:
 * 		- GPIO43 -> SPI1_D1 (HSEC 49 -> HSEC 77)
 * 		- GPIO44 -> SPI1_D0 (HSEC 51 -> HSEC 75)
 * 		- GPIO45 -> SPI1_CLK (HSEC 53 -> HSEC 79)
 * 		- GPIO46 -> SPI1_CS0 (HSEC 55 -> HSEC 81)
 * 		- The above pins can be connected with an oscilloscope to view the transfer waveforms
 */

/* Function definitions */
static inline void App_bufferFill();

void gpio_controller_main(void *args)
{
    int32_t             status = SystemP_SUCCESS;
    uint32_t            i;

    Drivers_open();
    Board_driversOpen();

    DebugP_log("GPIO controller operation started...\r\n");

	/* Initialize tx and rx buffers */
    App_bufferFill();

	/* Wait for peripheral to be ready */
    IpcNotify_syncAll(SystemP_WAIT_FOREVER);

	/* Transmit and Receive operation */
    App_spiGpioTxRx();

    for(i=0; i<APP_GPIOSPI_MSGSIZE; i++)
    {
        if(gMcspiTxBuffer[i] != gMcspiRxBuffer[i])
        {
            status = SystemP_FAILURE;   /* Data mismatch */
            DebugP_log("Data Mismatch at offset %d\r\n", i);
            break;
        }
        DebugP_log("\r\nCONTROLLER | \t 0x%x \t | \t 0x%x\r\n", gMcspiTxBuffer[i], gMcspiRxBuffer[i]);
    }

    if(SystemP_SUCCESS == status)
    {
        DebugP_log("All tests have passed!!\r\n");
    }
    else
    {
        DebugP_log("Some tests have failed!!\r\n");
    }

    Board_driversClose();
    Drivers_close();
}

static inline void App_bufferFill()
{
	uint32_t	i;

    for(i=0; i<APP_GPIOSPI_MSGSIZE; i++)
    {
#ifdef APP_GPIOSPI_8BIT
        gMcspiTxBuffer[i] = i + 0xAA;
#else
        gMcspiTxBuffer[i] = i + 0xAAAAAAAA;
#endif
        gMcspiRxBuffer[i] = 0;
    }
}