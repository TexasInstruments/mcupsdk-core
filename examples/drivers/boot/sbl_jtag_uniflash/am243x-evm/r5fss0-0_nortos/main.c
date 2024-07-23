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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <kernel/dpl/DebugP.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <drivers/bootloader/bootloader_uniflash/bootloader_uniflash.h>

#define FILE_MAX_SIZE   (0x800000) /* This has to match the size of DDR section in linker.cmd */
uint8_t gFileBuf[FILE_MAX_SIZE] __attribute__((aligned(128), section(".bss.filebuf"))); 

#define VERIFY_BUF_MAX_SIZE (32*1024)
uint8_t gVerifyBuf[VERIFY_BUF_MAX_SIZE] __attribute__((aligned(128), section(".bss")));

#define INPUT_STR_MAX_LEN   (16u)
static char inputStr[INPUT_STR_MAX_LEN];

#define FILE_NAME_MAX_LEN   (384u)
static char filename[FILE_NAME_MAX_LEN];

char gMainMenu[] = {
    " \r\n"
    " \r\n"
    " ==================\r\n"
    " JTAG Uniflash Menu\r\n"
    " ==================\r\n"
    " \r\n"
    " 1: Erase Complete Flash\r\n"
    " 2: Write File to Flash and Verify\r\n"
    " 3: Verify file in Flash\r\n"
    " \r\n"
    " x: Exit\r\n"
    " \r\n"
    " Enter Choice: "
};

int32_t sbl_jtag_uniflash_load_file(char optype)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t offset;
    uint32_t fileSize;
    uint32_t eraseBlkSize;
    uint32_t flashSize;
    FILE *fp;

    DebugP_log("\r\n");
    DebugP_log(" Enter file name along with path to write or verify : ");
    gets(filename);
    fp = fopen(filename, "rb");
    if(fp == NULL)
    {   /* if file not present then exit */
        DebugP_log(" [FLASH WRITER] Unable to open file %s !!!\r\n", filename);
        status = SystemP_FAILURE;
    }
    if(status==SystemP_SUCCESS)
    {   /* if file size > buffer size then exit, we will load the file in one shot so
         * so file size has to be < buffer size
         */
        fseek(fp, 0, SEEK_END); 
        fileSize = ftell(fp);
        fseek(fp, 0, SEEK_SET);
        fclose(fp);

        if(fileSize > FILE_MAX_SIZE)
        {
            DebugP_log(" [FLASH WRITER] File is too large to flash !!!\r\n");
            status = SystemP_FAILURE;
        }
    }
    if(status==SystemP_SUCCESS)
    {
        DebugP_log(" Enter flash offset (in hex format) : ");
        gets(inputStr);
        offset = strtol(inputStr, NULL, INPUT_STR_MAX_LEN);
    }
    if(status==SystemP_SUCCESS)
    {
        Flash_Attrs *flashAttrs;
        flashAttrs = Flash_getAttrs(CONFIG_FLASH0);
        if(flashAttrs == NULL)
        {
            status=SystemP_FAILURE; 
            DebugP_log(" [FLASH WRITER] Flash attributes are invalid !!!\r\n");
        }    
        else
        {
            eraseBlkSize = flashAttrs->blockSize;
            flashSize = flashAttrs->flashSize;

            if( (offset % eraseBlkSize) != 0)
            {
                DebugP_log(" [FLASH WRITER] Flash offset MUST be multiple of erase block size of 0x%08x !!!\r\n",
                    eraseBlkSize
                    );
                status = SystemP_FAILURE;
            }
            if( (offset+fileSize) > flashSize)
            {
                DebugP_log(" [FLASH WRITER] Flash offset + file size MUST be <= flash size of %d bytes !!!\r\n",
                    flashSize
                    );
                status = SystemP_FAILURE;
            }
        }
    }
    if(status==SystemP_SUCCESS)
    {
        /* load file via CCS scripting console, since otherwise it is very slow */
        { /* convert \\ to / */
            uint32_t i = 0;
            while(filename[i]!=0)
            {
                if(filename[i]=='\\')
                    filename[i]='/';
                i++;
            }
        }
        /* clear buffer to reset stale data if any */
        memset(gFileBuf, 0, FILE_MAX_SIZE);

        /* Construct the header depending on the option provided so that Bootloader_uniflashProcessCommands can be used */
        Bootloader_UniflashFileHeader uniflashHeader;

        uniflashHeader.magicNumber = BOOTLOADER_UNIFLASH_FILE_HEADER_MAGIC_NUMBER;
        uniflashHeader.offset = offset;
        uniflashHeader.actualFileSize = fileSize;

        if(optype == '2')
        {
            /* Operation is FLASH */
            uniflashHeader.operationTypeAndFlags = BOOTLOADER_UNIFLASH_OPTYPE_FLASH;
        }
        else if(optype == '3')
        {
            /* Operation is FLASH VERIFY */
            uniflashHeader.operationTypeAndFlags = BOOTLOADER_UNIFLASH_OPTYPE_FLASH_VERIFY;
        }
        else
        {
            /* Operation is invalid */
            uniflashHeader.operationTypeAndFlags = 0U;
        }

        if(uniflashHeader.operationTypeAndFlags == 0U)
        {
            status = SystemP_FAILURE;
        }
        else
        {
            /* Load the image after the header */
            memcpy(gFileBuf, (void *)&uniflashHeader, sizeof(uniflashHeader));

            DebugP_log(" Enter below command in CCS scripting console to load the file data to memory.\r\n");
            DebugP_log(" AFTER the file load is done, enter '1' to continue ...\r\n");
            DebugP_log("\r\n");
            DebugP_log(" loadRaw(0x%08x, 0, \"%s\", 32, false);",
                gFileBuf + sizeof(uniflashHeader), filename);
            DebugP_log("\r\n");

            /* wait for user input */
            do {
                gets(inputStr);
            } while(inputStr[0]!='1');

            /* Process the buffer */
            Bootloader_UniflashConfig uniflashConfig;
            Bootloader_UniflashResponseHeader respHeader;

            uniflashConfig.flashIndex = CONFIG_FLASH0;
            uniflashConfig.buf = gFileBuf;
            uniflashConfig.bufSize = 0U; /* Actual filesize is part of the header */
            uniflashConfig.verifyBuf = gVerifyBuf;
            uniflashConfig.verifyBufSize = VERIFY_BUF_MAX_SIZE;

            /* Process the flash commands and return a response */
            Bootloader_uniflashProcessFlashCommands(&uniflashConfig, &respHeader);

            if(respHeader.statusCode == BOOTLOADER_UNIFLASH_STATUSCODE_SUCCESS)
            {
                if(optype == '2')
                {
                    DebugP_log(" [FLASH WRITER] Flashing success!!... \r\n");
                }
                else if(optype == '3')
                {
                    DebugP_log(" [FLASH WRITER] Verifying success!!... \r\n");
                }
                status = SystemP_SUCCESS;
            }
            else
            {
                status = SystemP_FAILURE;
            }
        }
    }

    return status;
}

void sbl_jtag_uniflash_erase_all(void)
{
    int32_t status;
    DebugP_log(" [FLASH WRITER] Erasing complete flash ... \r\n");
    DebugP_log(" [FLASH WRITER] This can take few minutes, so please wait ... \r\n");

    status = Flash_eraseBlk(gFlashHandle[CONFIG_FLASH0], (uint32_t)-1);

    if(status!=SystemP_SUCCESS)
    {
        DebugP_log(" [FLASH WRITER] Erasing complete flash ... ERROR !!!\r\n");
    }
    else
    {
        DebugP_log(" [FLASH WRITER] Erasing complete flash ... SUCCESS !!!\r\n");
    }
}

int main(void)
{
    int32_t status = SystemP_SUCCESS;
    char ch[16];

    System_init();
    /* Open OSPI Driver, among others */
    Drivers_open();

    /* Open Flash drivers with OSPI instance as input */
    status = Board_driversOpen();
    if(status!=SystemP_SUCCESS)
    {
        DebugP_log(" [FLASH WRITER] Unable to open FLASH !!!\r\n");
    }

    if(status==SystemP_SUCCESS)
    {
        while(1)
        {
            DebugP_log(gMainMenu);

            gets(ch);

            switch(ch[0])
            {
                case '1':
                    sbl_jtag_uniflash_erase_all();
                    break;
                case '2':
                case '3':
                    status = sbl_jtag_uniflash_load_file(ch[0]);
                    if(status != SystemP_SUCCESS)
                    {
                        if (ch[0] == '2')
                        {
                            DebugP_log(" [FLASH WRITER] Write Failed !!!\r\n");
                        }
                        else if (ch[0] == '3')
                        {
                            DebugP_log(" [FLASH WRITER] Verify Failed !!!\r\n");
                        }
                    }
                    break;
                case 'x':
                break;
                default:
                    DebugP_log(" Enter valid option !!!\r\n");
                break;                
            }
            if(ch[0] == 'x')
                break;
        }
    }
    if(status != SystemP_SUCCESS)
    {
        DebugP_log("Some tests have failed!!\r\n");
    }
    else
    {
        DebugP_log("\r\n");
        DebugP_log(" [FLASH WRITER] Application exited !!!\r\n");
        DebugP_log("All tests have passed!!\r\n");
    }

    Board_driversClose();
    Drivers_close();
}
