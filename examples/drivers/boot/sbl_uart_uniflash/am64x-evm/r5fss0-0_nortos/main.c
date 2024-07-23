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
#include <string.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <drivers/sciclient.h>
#include <drivers/bootloader.h>
#include <drivers/bootloader/bootloader_xmodem.h>
#include <drivers/bootloader/bootloader_uniflash/bootloader_uniflash.h>

#define BOOTLOADER_UNIFLASH_MAX_FILE_SIZE (0x800000) /* This has to match the size of DDR section in linker.cmd */
uint8_t gUniflashFileBuf[BOOTLOADER_UNIFLASH_MAX_FILE_SIZE] __attribute__((aligned(128), section(".bss.filebuf")));

#define BOOTLOADER_UNIFLASH_VERIFY_BUF_MAX_SIZE (32*1024)
uint8_t gUniflashVerifyBuf[BOOTLOADER_UNIFLASH_VERIFY_BUF_MAX_SIZE] __attribute__((aligned(128), section(".bss")));

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
    uint32_t done = 0U;
    uint32_t fileSize;
    Bootloader_UniflashConfig uniflashConfig;
    Bootloader_UniflashResponseHeader respHeader;


    Bootloader_socWaitForFWBoot();
    Bootloader_socOpenFirewalls();

    Bootloader_socNotifyFirewallOpen();

    System_init();
    Drivers_open();

    status = Board_driversOpen();
    DebugP_assert(status == SystemP_SUCCESS);

    while(!done)
    {
        /* Xmodem Receive */
        status = Bootloader_xmodemReceive(CONFIG_UART0, gUniflashFileBuf, BOOTLOADER_UNIFLASH_MAX_FILE_SIZE, &fileSize);

        /*
         * The `fileSize` wouldn't be the actual filesize, but (actual filesize + size of the header + padding bytes) added by xmodem.
         * This adds ~1KB. We can't know exactly how many bytes will be padded without checking the file header. But doing that
         * will unnecessary complicate the logic, so since the overhead is as small as ~1KB we could check for file size exceed
         * by checking * this `fileSize` returned by xmodem as well.
        */
        if(fileSize >= BOOTLOADER_UNIFLASH_MAX_FILE_SIZE)
        {
            /* Possible overflow, send error to host side */
            status = SystemP_FAILURE;

            respHeader.magicNumber = BOOTLOADER_UNIFLASH_RESP_HEADER_MAGIC_NUMBER;
            respHeader.statusCode = BOOTLOADER_UNIFLASH_STATUSCODE_FLASH_ERROR;

            Bootloader_xmodemTransmit(CONFIG_UART0, (uint8_t *)&respHeader, sizeof(Bootloader_UniflashResponseHeader));
        }

        if(status == SystemP_SUCCESS)
        {
            uniflashConfig.flashIndex = CONFIG_FLASH0;
            uniflashConfig.buf = gUniflashFileBuf;
            uniflashConfig.bufSize = 0; /* Actual fileSize will be parsed from the header */
            uniflashConfig.verifyBuf = gUniflashVerifyBuf;
            uniflashConfig.verifyBufSize = BOOTLOADER_UNIFLASH_VERIFY_BUF_MAX_SIZE;

            /* Process the flash commands and return a response */
            Bootloader_uniflashProcessFlashCommands(&uniflashConfig, &respHeader);

            status = Bootloader_xmodemTransmit(CONFIG_UART0, (uint8_t *)&respHeader, sizeof(Bootloader_UniflashResponseHeader));
        }
    }

    Drivers_close();
    System_deinit();

    return 0;
}
