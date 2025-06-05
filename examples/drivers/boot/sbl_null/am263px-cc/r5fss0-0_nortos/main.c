/*
 *  Copyright (C) 2023-2024 Texas Instruments Incorporated
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
#include "ti_clocktree_pll_config.h"
#include <drivers/bootloader.h>
#include <security/security_common/drivers/hsmclient/hsmclient.h>
#include <security/security_common/drivers/hsmclient/soc/am263px/hsmRtImg.h> /* hsmRt bin   header file */

const uint8_t gHsmRtFw[HSMRT_IMG_SIZE_IN_BYTES]__attribute__((section(".rodata.hsmrt")))
    = HSMRT_IMG;

extern HsmClient_t gHSMClient ;
extern uint32_t gMcuLbistTestStatus;

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

/*  this API is a weak function definition for keyring_init function
    which is defined in generated files if keyring module is enabled
    in syscfg
*/
__attribute__((weak)) int32_t Keyring_init(HsmClient_t *gHSMClient)
{
    return SystemP_SUCCESS;
}

int main(void)
{
    int32_t status = SystemP_SUCCESS;

    Bootloader_profileReset();
    Bootloader_socConfigurePll();
    Bootloader_socSetAutoClock();

    System_init();
    Bootloader_profileAddProfilePoint("System_init");

    Drivers_open();
    Bootloader_profileAddProfilePoint("Drivers_open");

    DebugP_log("\r\n");
    if(gMcuLbistTestStatus == 0U)
    {
        Bootloader_socLoadHsmRtFw(&gHSMClient, gHsmRtFw, HSMRT_IMG_SIZE_IN_BYTES);
        Bootloader_socInitL2MailBoxMemory();
        Bootloader_profileAddProfilePoint("LoadHsmRtFw");

        status = Keyring_init(&gHSMClient);
        DebugP_assert(status == SystemP_SUCCESS);
    }

    if(gMcuLbistTestStatus == 1U)
    {
        gMcuLbistTestStatus = 0U;
    }
    else
    {
        gMcuLbistTestStatus = 1U;
    }
    /* Perform LBIST test on CPU. Once the test is complete, CPU will reset and start over.*/
    SDL_lbist_selftest();
    if(gMcuLbistTestStatus == 0U)
    {
        DebugP_log("STC Test Complete ... \r\n");
    }

    DebugP_log("Starting NULL Bootloader ... \r\n");


    Bootloader_Params bootParams;
    Bootloader_BootImageInfo bootImageInfo;
    Bootloader_Handle bootHandle;

    Bootloader_Params_init(&bootParams);
    /* set default which will basically allow to simply power on reset the CPU and run a while(1) loop */
    Bootloader_BootImageInfo_init(&bootImageInfo);

    bootHandle = Bootloader_open(CONFIG_BOOTLOADER0, &bootParams);
    Bootloader_Config *config = (Bootloader_Config *)bootHandle;
    void *CoreOpMode = config->socCoreOpMode;
    Bootloader_socCoreOpModeConfig *coreconfig = (Bootloader_socCoreOpModeConfig *)CoreOpMode;

    if(bootHandle != NULL)
    {
        if(status == SystemP_SUCCESS && (coreconfig->r5fss1_opMode == BOOTLOADER_OPMODE_STANDALONE))
        {
            status = Bootloader_bootCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS1_1]);
        }
        if(status == SystemP_SUCCESS)
        {
            status = Bootloader_bootCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS1_0]);
        }
        if(status == SystemP_SUCCESS && (coreconfig->r5fss0_opMode == BOOTLOADER_OPMODE_STANDALONE))
        {
            status = Bootloader_bootCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS0_1]);
        }
        Bootloader_profileAddProfilePoint("SBL End");
        Bootloader_profilePrintProfileLog();
        DebugP_log("NULL Bootloader Execution Complete... \r\n");
        UART_flushTxFifo(gUartHandle[CONFIG_UART0]);
        if(status == SystemP_SUCCESS)
        {
            /* Reset self cluster, Core0. Init RAMs and run dummy loop  */
            status = Bootloader_bootSelfCpu(bootHandle, &bootImageInfo);
        }
        /* it should not return here, if it does, then there was some error */
        Bootloader_close(bootHandle);
    }

    Drivers_close();
    System_deinit();

    return 0;
}
