/*
 *  Copyright (C) 2018-2021 Texas Instruments Incorporated
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

#include <stdlib.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include <drivers/bootloader.h>
#include <drivers/hsmclient/soc/am273x/hsmRtImg.h> /* hsmRt bin   header file */

const uint8_t gHsmRtFw[HSMRT_IMG_SIZE_IN_BYTES]__attribute__((section(".rodata.hsmrt")))
    = HSMRT_IMG;

/* call this API to stop the booting process and spin, do that you can connect
 * debugger, load symbols and then make the 'loop' variable as 0 to continue execution
 * with debugger connected.
 */
void loop_forever(void)
{
    volatile uint32_t loop = 1;
    while(loop)
        ;
}

int main(void)
{
    int32_t status = SystemP_SUCCESS;

    Bootloader_socConfigurePll();
    System_init();
    Drivers_open();

    DebugP_log("\r\n");
    Bootloader_socLoadHsmRtFw(gHsmRtFw, HSMRT_IMG_SIZE_IN_BYTES);
    DebugP_log("Starting NULL Bootloader ... \r\n");

    Bootloader_Params bootParams;
    Bootloader_BootImageInfo bootImageInfo;
    Bootloader_Handle bootHandle;

    Bootloader_Params_init(&bootParams);
    /* set default which will basically allow to simply power on reset the CPU and run a while(1) loop */
    Bootloader_BootImageInfo_init(&bootImageInfo);

    bootHandle = Bootloader_open(CONFIG_BOOTLOADER0, &bootParams);

    if(bootHandle != NULL)
    {
        if(status == SystemP_SUCCESS)
        {
            status = Bootloader_bootCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_C66SS0]);
        }
        if((status == SystemP_SUCCESS) && (BOOTLOADER_SOC_VARIANT_DUAL_R5F == Bootloader_socGetCoreVariant()))
        {
            status = Bootloader_bootCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS0_1]);
        }
        DebugP_log("NULL Bootloader Execution Complete... \r\n");
        UART_flushTxFifo(gUartHandle[CONFIG_UART0]);
        if(status == SystemP_SUCCESS)
        {
            /* Reset self cluster, both Core0 and Core 1. Init RAMs and run dummy loop  */
            status = Bootloader_bootSelfCpu(bootHandle, &bootImageInfo);
        }
        /* it should not return here, if it does, then there was some error */
        Bootloader_close(bootHandle);
    }

    Drivers_close();
    System_deinit();

    return 0;
}


