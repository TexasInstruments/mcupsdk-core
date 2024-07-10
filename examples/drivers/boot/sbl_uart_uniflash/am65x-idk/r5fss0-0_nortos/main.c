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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdlib.h>
#include <string.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <drivers/sciclient.h>
#include <drivers/bootloader.h>
#include <drivers/bootloader/bootloader_xmodem.h>
#include <drivers/bootloader/bootloader_uniflash/bootloader_uniflash.h>

#ifdef HS
#include <drivers/sciclient/sciclient_default_boardcfg/am65x/sciclient_boardcfg_hs.h>
#else
#include <drivers/sciclient/sciclient_default_boardcfg/am65x/sciclient_boardcfg.h>
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */
#define SBL_SYSFW_MAX_SIZE         (0x42000U)
#define BOOTLOADER_UNIFLASH_MAX_FILE_SIZE (0x800000) /* This has to match the size of DDR section in linker.cmd */
#define BOOTLOADER_UNIFLASH_VERIFY_BUF_MAX_SIZE (32*1024)
#define SBL_SYSFW_UART_STATUS_LOAD_SUCCESS           (0x53554343) /* SUCC */
#define SBL_SYSFW_UART_STATUS_LOAD_CPU_FAIL          (0x4641494C) /* FAIL */

uint8_t gSysImageBuf[SBL_SYSFW_MAX_SIZE] __attribute__((aligned(128), section(".firmware")));
uint8_t gUniflashFileBuf[BOOTLOADER_UNIFLASH_MAX_FILE_SIZE] __attribute__((aligned(128), section(".bss.filebuf")));
uint8_t gUniflashVerifyBuf[BOOTLOADER_UNIFLASH_VERIFY_BUF_MAX_SIZE] __attribute__((aligned(128), section(".bss")));

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */
int32_t Bootloader_ReadSysfwImage();
int32_t Bootloader_loadFirmware();
static void Bootloader_sciclientBoardCfg();

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
/* call this API to stop the booting process and spin, do that you can connect
 * debugger, load symbols and then make the 'loop' variable as 0 to continue execution
 * with debugger connected.
 */
void loop_forever(void)
{
    volatile uint32_t loop = 1;
    while(loop){
        ;
    }
}

int main(void)
{
    int32_t status = SystemP_FAILURE;
    uint32_t done = 0U;
    uint32_t fileSize;
    Bootloader_UniflashConfig uniflashConfig;
    Bootloader_UniflashResponseHeader respHeader;

    System_init();
    Drivers_open();

    status = Board_driversOpen();
    DebugP_assert(status == SystemP_SUCCESS);

    status = Bootloader_ReadSysfwImage();
    if (SystemP_SUCCESS == status)
    {
        status = Bootloader_loadFirmware();
    }
    if (SystemP_SUCCESS == status)
    {
        Bootloader_sciclientBoardCfg();
    }
    System_lateInit();

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

int32_t Bootloader_ReadSysfwImage()
{
    uint32_t fileSize;
    int32_t status;

    status = Bootloader_xmodemReceive(CONFIG_UART0, gSysImageBuf, SBL_SYSFW_MAX_SIZE, &fileSize);
    if(SystemP_SUCCESS == status && fileSize == SBL_SYSFW_MAX_SIZE)
        {
            /* A file larger was sent, and xmodem probably dropped bytes */
            status = SystemP_FAILURE;
        }

    return status;
}

int32_t Bootloader_loadFirmware()
{
    int32_t status = SystemP_FAILURE;
    uint32_t size = SBL_SYSFW_MAX_SIZE;

    size = BOOTLOADER_ALIGN_SIZE(SBL_SYSFW_MAX_SIZE);

    CacheP_wbInv((uint8_t *)gSysImageBuf, size, CacheP_TYPE_ALL);
    status = Sciclient_loadFirmware((const uint32_t *)gSysImageBuf);

    return status;
}

static void Bootloader_sciclientBoardCfg()
{
    int32_t status = SystemP_FAILURE;

    static uint8_t boardCfgLow[] = SCICLIENT_BOARDCFG;
    Sciclient_BoardCfgPrms_t boardCfgPrms =
    {
        .boardConfigLow = (uint32_t) &boardCfgLow,
        .boardConfigHigh = 0,
        .boardConfigSize = SCICLIENT_BOARDCFG_SIZE_IN_BYTES,
        .devGrp = DEVGRP_ALL,
    };
    status = Sciclient_boardCfg(&boardCfgPrms);
    if (status != SystemP_SUCCESS)
    {
        DebugP_logError("[SCICLIENT] Sciclient Common Board Configuration has failed \r\n");
    }

    if(SystemP_SUCCESS == status)
    {
         static uint8_t boardCfgLow[] = SCICLIENT_BOARDCFG_PM;
         Sciclient_BoardCfgPrms_t boardCfgPrms_pm =
         {
            .boardConfigLow = (uint32_t)boardCfgLow,
            .boardConfigHigh = 0,
            .boardConfigSize = SCICLIENT_BOARDCFG_PM_SIZE_IN_BYTES,
            .devGrp = DEVGRP_ALL,
         };
         status = Sciclient_boardCfgPm(&boardCfgPrms_pm);
         if (status != SystemP_SUCCESS)
         {
             DebugP_logError("[SCICLIENT] PM Board Configuration has failed \r\n");
         }
    }

    if (status == SystemP_SUCCESS)
    {
        static uint8_t boardCfgLow[] = SCICLIENT_BOARDCFG_SECURITY;
        Sciclient_BoardCfgPrms_t boardCfgPrms_sec =
        {
            .boardConfigLow = (uint32_t)boardCfgLow,
            .boardConfigHigh = 0,
            .boardConfigSize = SCICLIENT_BOARDCFG_SECURITY_SIZE_IN_BYTES,
            .devGrp = DEVGRP_ALL,
        };
        status = Sciclient_boardCfgSec(&boardCfgPrms_sec) ;
        if (status != SystemP_SUCCESS)
        {
            DebugP_logError("[SCICLIENT] Security Board Configuration has failed \r\n");
        }
    }
    status = Bootloader_socOpenFirewalls();

    Bootloader_socNotifyFirewallOpen();

    DebugP_assertNoLog(status == SystemP_SUCCESS);

    if (SystemP_SUCCESS == status)
    {
        static uint8_t boardCfgLow[] = SCICLIENT_BOARDCFG_RM;
        Sciclient_BoardCfgPrms_t boardCfgPrms_rm =
        {
            .boardConfigLow = (uint32_t)boardCfgLow,
            .boardConfigHigh = 0,
            .boardConfigSize = SCICLIENT_BOARDCFG_RM_SIZE_IN_BYTES,
            .devGrp = DEVGRP_ALL,
        };
        status = Sciclient_boardCfgRm(&boardCfgPrms_rm);
        if (status != SystemP_SUCCESS)
        {
            DebugP_logError("[SCICLIENT] RM Board Configuration has failed \r\n");
        }
    }
}