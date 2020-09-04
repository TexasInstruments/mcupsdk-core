/*
 *  Copyright (C) 2022 Texas Instruments Incorporated
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

#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <kernel/dpl/DebugP.h>
#include <drivers/soc.h>
#include <drivers/pcie.h>
#include <string.h>

/******** Buffer *******/
#define BUF_SIZE 0x4000000u

/* 4MB buffer */
uint8_t dst_buf[BUF_SIZE]  __attribute__ ((section (".buffer"), aligned (0x4000000u)));

#define NUM_ITER  10

void Pcie_bufInit(void)
{
    memset (dst_buf, 0xFU, BUF_SIZE);

    return;
}

void pcie_benchmark_ep_main (void *args)
{
    int32_t status = SystemP_SUCCESS;
    int i;

    Drivers_open();
    Board_driversOpen();

    DebugP_log("Device in EP mode\r\n");

    Pcie_bufInit();

    /* Check if buffer is received using CPU copy */
    for (i = 0; i < NUM_ITER; i++)
    {
        /* Check if the buffer is received from RC */
        do
        {
            CacheP_inv (dst_buf, BUF_SIZE, CacheP_TYPE_ALL);
        } while (dst_buf[BUF_SIZE-1] != i);

        /* Send Legacy interrupt to RC */

        Pcie_legacyIrqSetParams irqSetParams;

        /* Assert legacy interrupt to RC */
        irqSetParams.intNum = 1;
        irqSetParams.assert = 1;

        status =  Pcie_epLegacyIrqSet(gPcieHandle[CONFIG_PCIE0], irqSetParams);

        /* Deassert legacy interrupt to RC */
        irqSetParams.intNum = 1;
        irqSetParams.assert = 0;

        status =  Pcie_epLegacyIrqSet(gPcieHandle[CONFIG_PCIE0], irqSetParams);
    }

    /* Check if buffer is received using UDMA transfer */
    for (i = 0; i < NUM_ITER; i++)
    {
        /* Check if the buffer is received from RC */
        do
        {
            CacheP_inv (dst_buf, BUF_SIZE, CacheP_TYPE_ALL);
        } while (dst_buf[BUF_SIZE-1] != i);

        /* Send Legacy interrupt to RC */

        Pcie_legacyIrqSetParams irqSetParams;

        /* Assert legacy interrupt to RC */
        irqSetParams.intNum = 1;
        irqSetParams.assert = 1;

        status =  Pcie_epLegacyIrqSet(gPcieHandle[CONFIG_PCIE0], irqSetParams);

        /* Deassert legacy interrupt to RC */
        irqSetParams.intNum = 1;
        irqSetParams.assert = 0;

        status =  Pcie_epLegacyIrqSet(gPcieHandle[CONFIG_PCIE0], irqSetParams);
    }

    if (status != SystemP_SUCCESS)
    {
        DebugP_log("ep main failed\r\n");
    }
    Board_driversClose();
    Drivers_close();

    return;
}
