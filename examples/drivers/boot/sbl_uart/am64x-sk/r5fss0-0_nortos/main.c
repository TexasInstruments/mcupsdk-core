/*
 *  Copyright (C) 2018-2026 Texas Instruments Incorporated
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
#include <string.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <drivers/sciclient.h>
#include <drivers/bootloader.h>
#include <drivers/bootloader/bootloader_xmodem.h>
#include <drivers/bootloader/bootloader_buf_io.h>

#define BOOTLOADER_UART_STATUS_LOAD_SUCCESS           (0x53554343) /* SUCC */
#define BOOTLOADER_UART_STATUS_LOAD_CPU_FAIL          (0x4641494C) /* FAIL */
#define BOOTLOADER_UART_STATUS_APPIMAGE_SIZE_EXCEEDED (0x45584344) /* EXCD */

#define BOOTLOADER_UART_CPU_RUN_WAIT_SECONDS          (5)
#define BOOTLOADER_END_OF_FILES_TRANSFER_WORD_LENGTH  (4) /* bytes */
#define BOOTLOADER_APP_IMAGE_LOADED                   (1)

#define SCRATCH_BUFFER_SIZE                           (0x2000U) /*Size of binary metadata*/

uint8_t gAppImageBuf[SCRATCH_BUFFER_SIZE] __attribute__((aligned(128), section(".bss.filebuf")));

uint8_t gEndOfFilesTransferWord[BOOTLOADER_END_OF_FILES_TRANSFER_WORD_LENGTH] = {0x45,0x4F,0x46,0x54}; /* Contain Magic word Indicating End Of File Transfer(EOFT) */;

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
    int32_t status;

    Bootloader_socWaitForFWBoot();

    if (!Bootloader_socIsMCUResetIsoEnabled())
    {
        /* Update devGrp to ALL to initialize MCU domain when reset isolation is
        not enabled */
        Sciclient_BoardCfgPrms_t boardCfgPrms_pm =
            {
                .boardConfigLow = (uint32_t)0,
                .boardConfigHigh = 0,
                .boardConfigSize = 0,
                .devGrp = DEVGRP_ALL,
            };

        status = Sciclient_boardCfgPm(&boardCfgPrms_pm);

        /* Enable MCU PLL. MCU PLL will not be enabled by DMSC when devGrp is set
        to Main in boardCfg */
        Bootloader_enableMCUPLL();
    }

    System_init();
    
    status = Bootloader_socOpenFirewalls();

    Bootloader_socNotifyFirewallOpen();

    DebugP_assertNoLog(status == SystemP_SUCCESS);

    Drivers_open();

    status = Board_driversOpen();
    DebugP_assert(status == SystemP_SUCCESS);

    if(SystemP_SUCCESS == status)
    {
        uint32_t fileSize;
        bool bEndOfTransfer = false;

        Bootloader_BootImageInfo bootImageInfo;
		Bootloader_Params bootParams;
        Bootloader_Handle bootHandle;

        Bootloader_BootImageInfo_init(&bootImageInfo);
        Bootloader_Params_init(&bootParams);
        bootParams.memArgsAppImageBaseAddr = (uint32_t)(&gAppImageBuf);
        bootParams.bufIoDeviceIndex = CONFIG_UART0;
        bootHandle = Bootloader_open(CONFIG_BOOTLOADER_0, &bootParams);

        /*Loop through all the files*/
        while(bEndOfTransfer == false)
        {
            /* Receive the image metadata */
            status = Bootloader_xmodemReceive(CONFIG_UART0, gAppImageBuf, SCRATCH_BUFFER_SIZE, &fileSize);
            CacheP_wb((void*)gAppImageBuf, (uint32_t)SCRATCH_BUFFER_SIZE, CacheP_TYPE_ALL);

            /* Check the end of transfer is reached using receieved chunk*/
            if(SystemP_SUCCESS == status && memcmp(gAppImageBuf, gEndOfFilesTransferWord, BOOTLOADER_END_OF_FILES_TRANSFER_WORD_LENGTH) == 0)
            {
                bEndOfTransfer = true;

                /* Delay 5 seconds for the user to connect to UART before the CPUs start running */
                ClockP_sleep(BOOTLOADER_UART_CPU_RUN_WAIT_SECONDS);

                /* Run CPUs */
                /* Do not run M4 when MCU domain is reset isolated */
                if (!Bootloader_socIsMCUResetIsoEnabled())
                {
                    if(status == SystemP_SUCCESS && (TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_M4FSS0_0)))
                    {
                        status = Bootloader_runCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_M4FSS0_0]);
                    }
                }
                if(status == SystemP_SUCCESS && (TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_R5FSS1_0)))
                {
                    status = Bootloader_runCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS1_0]);
                }
                if(status == SystemP_SUCCESS && (TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_R5FSS1_1)))
                {
                    status = Bootloader_runCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS1_1]);
                }
                if(status == SystemP_SUCCESS && (TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_A53SS0_0)))
                {
                    status = Bootloader_runCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_A53SS0_0]);
                }
                if(status == SystemP_SUCCESS && (TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_A53SS0_1)))
                {
                    status = Bootloader_runCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_A53SS0_1]);
                }
                
                if(status == SystemP_SUCCESS)
                {

                    status = Bootloader_runSelfCpu(bootHandle, &bootImageInfo);
                }
                /* it should not return here, if it does, then there was some error */
                Bootloader_close(bootHandle);
            }

            /* File transfer end is not reached*/
            else
            {
                if(SystemP_SUCCESS == status)
                {
                    /* Parse the received metadata ,requests program segment from host and load to load addresses */
                    Bootloader_UartParseAndLoadMultiCoreELF(bootHandle, &bootImageInfo);
                }
            }

        }

    }
    if(status != SystemP_SUCCESS)
    {
        DebugP_log("Some tests have failed!!\r\n");
    }
    Drivers_close();
    System_deinit();

    return 0;
}


