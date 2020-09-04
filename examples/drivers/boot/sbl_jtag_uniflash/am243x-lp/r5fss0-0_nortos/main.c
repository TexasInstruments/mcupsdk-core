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

#define FILE_MAX_SIZE   (0x150000) /* This has to match the size of MSRAM_2 section in linker.cmd */
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

int32_t sbl_jtag_uniflash_load_file(uint32_t *flashOffset, uint32_t *fileSize)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t offset;
    uint32_t size;    
    FILE *fp;

    *flashOffset = 0;
    *fileSize = 0;

    DebugP_log("\r\n");
    DebugP_log(" Enter file name to write or verify : ");
    gets(filename);
    fp = fopen(filename, "rb");
    if(fp==NULL)
    {   /* if file not present then exit */
        DebugP_log(" [FLASH WRITER] Unable to open file %s !!!\r\n", filename);
        status = SystemP_FAILURE;
    }
    if(status==SystemP_SUCCESS)
    {   /* if file size > buffer size then exit, we will load the file in one shot so
         * so file size has to be < buffer size
         */
        fseek(fp, 0, SEEK_END); 
        size = ftell(fp); 
        fseek(fp, 0, SEEK_SET);
        fclose(fp);

        if(size > FILE_MAX_SIZE)
        {
            DebugP_log(" [FLASH WRITER] File is too large to flash !!!\r\n");
            status = SystemP_FAILURE;
        }
    }
    if(status==SystemP_SUCCESS)
    {
        DebugP_log(" Enter flash offset (in hex) : ");
        gets(inputStr);
        offset = strtol(inputStr, NULL, 16);
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
            uint32_t eraseBlkSize = flashAttrs->blockSize;
            uint32_t flashSize = flashAttrs->flashSize;

            if( (offset % eraseBlkSize) != 0 )
            {
                DebugP_log(" [FLASH WRITER] Flash offset MUST be multiple of erase block size of 0x%08x !!!\r\n",
                    eraseBlkSize
                    );
                status = SystemP_FAILURE;
            }
            if( (offset+size) > flashSize)
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
        
        DebugP_log(" Enter below command in CCS scripting console to load the file data to memory.\r\n");
        DebugP_log(" AFTER the file load is done, enter '1' to continue ...\r\n");
        DebugP_log("\r\n");
        DebugP_log(" loadRaw(0x%08x, 0, \"%s\", 32, false);", 
            gFileBuf, filename);
        DebugP_log("\r\n");

        /* wait for user input */
        do {
            gets(inputStr);
        } while(inputStr[0]!='1');
    }
    if(status==SystemP_SUCCESS)
    {
        *flashOffset = offset;
        *fileSize = size;
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
        DebugP_log(" [FLASH WRITER] Erasing complete flash ... DONE !!!\r\n");
    }
}

int32_t sbl_jtag_uniflash_write_file(uint32_t flashOffset, uint32_t fileSize)
{
    int32_t status = SystemP_SUCCESS;
    Flash_Attrs *flashAttrs;
    uint32_t eraseBlockSize;

    flashAttrs = Flash_getAttrs(CONFIG_FLASH0);
    if(flashAttrs == NULL)
    {
       status=SystemP_FAILURE; 
       DebugP_log(" [FLASH WRITER] Flash attributes are invalid !!!\r\n");
    }
    else
    {
        eraseBlockSize = flashAttrs->pageCount * flashAttrs->pageSize;
        DebugP_assert(eraseBlockSize < FILE_MAX_SIZE);
    }
    if(status==SystemP_SUCCESS)
    {
        uint32_t curOffset, totalChunks, curChunk, chunkSize, remainSize, blockNum, pageNum;
        uint8_t *srcAddr;

        /* start writing from buffer to flash */
        DebugP_log(" [FLASH WRITER] Writing %d bytes @ offset 0x%08x ... \r\n", fileSize, flashOffset);

        chunkSize = eraseBlockSize;

        srcAddr = gFileBuf;
        remainSize = fileSize;
        curOffset = flashOffset;
        curChunk = 1;
        totalChunks = (fileSize + (chunkSize-1))/chunkSize;        
        while(curChunk <= totalChunks)
        {
            if(remainSize < chunkSize)
            {
                chunkSize = remainSize;
            }

            DebugP_log(" [FLASH WRITER] %d of %d : Writing %d bytes @ offset 0x%08x ... \r\n", 
                curChunk, totalChunks, chunkSize, curOffset
                );
            
            status = Flash_offsetToBlkPage(gFlashHandle[CONFIG_FLASH0], curOffset, &blockNum, &pageNum);
            if(status == SystemP_SUCCESS)
            {
                status = Flash_eraseBlk(gFlashHandle[CONFIG_FLASH0], blockNum);
                if(status == SystemP_SUCCESS)
                {
                    status = Flash_write(gFlashHandle[CONFIG_FLASH0], curOffset, 
                                            srcAddr, chunkSize
                        );
                }
            }
            curOffset += chunkSize;
            srcAddr += chunkSize;
            remainSize -= chunkSize;
            curChunk++;
            if(status != SystemP_SUCCESS)
            {
                DebugP_log(" [FLASH WRITER] Flash write error, aborting !!!\r\n");
            }
        }
        if(status != SystemP_SUCCESS)
        {
            DebugP_log(" [FLASH WRITER] Writing %d bytes @ offset 0x%08x ... ERROR !!!\r\n", fileSize, flashOffset);
        }
        else
        { 
            DebugP_log(" [FLASH WRITER] Writing %d bytes @ offset 0x%08x ... DONE !!!\r\n", fileSize, flashOffset);
        }
    }
    return status;
}

int32_t sbl_jtag_uniflash_verify_file(uint32_t flashOffset, uint32_t fileSize)
{
        int32_t status = SystemP_SUCCESS;

    if(status==SystemP_SUCCESS)
    {
        uint32_t curOffset, totalChunks, curChunk, chunkSize, remainSize;
        uint8_t *srcAddr;
        int32_t diff;


        /* start writing from buffer to flash */
        DebugP_log(" [FLASH WRITER] Verifying %d bytes @ offset 0x%08x ... \r\n", fileSize, flashOffset);

        chunkSize = VERIFY_BUF_MAX_SIZE;

        srcAddr = gFileBuf;
        remainSize = fileSize;
        curOffset = flashOffset;
        curChunk = 1;
        totalChunks = (fileSize + (chunkSize-1))/chunkSize;        
        while(curChunk <= totalChunks)
        {
            if(remainSize < chunkSize)
            {
                chunkSize = remainSize;
            }

            DebugP_log(" [FLASH WRITER] %d of %d : Verifying %d bytes @ offset 0x%08x ... \r\n", 
                curChunk, totalChunks, chunkSize, curOffset
                );

            /* clear verify buf to avoid comparing stale data */            
            memset(gVerifyBuf, 0, VERIFY_BUF_MAX_SIZE);

            status = Flash_read(gFlashHandle[CONFIG_FLASH0], curOffset, 
                                    gVerifyBuf, chunkSize
                );

            if(status == SystemP_SUCCESS)
            { 
                /* check if data read from flash matches, data read from file */
                diff = memcmp(gVerifyBuf, srcAddr, chunkSize);

                if(diff != 0)
                {
                    status = SystemP_FAILURE;
                }
            }
            curOffset += chunkSize;
            srcAddr += chunkSize;
            remainSize -= chunkSize;
            curChunk++;
            if(status != SystemP_SUCCESS)
            {
                DebugP_log(" [FLASH WRITER] Flash verify error, aborting !!!\r\n");
            }
        }
        if(status != SystemP_SUCCESS)
        {
            DebugP_log(" [FLASH WRITER] Verifying %d bytes @ offset 0x%08x ... ERROR !!!\r\n", fileSize, flashOffset);
        }
        else
        { 
            DebugP_log(" [FLASH WRITER] Verifying %d bytes @ offset 0x%08x ... DONE !!!\r\n", fileSize, flashOffset);
        }
    }
    return status;
}

int main(void)
{
    int32_t status = SystemP_SUCCESS;

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
        uint32_t fileSize, flashOffset;
        char ch[16];
        
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
                    status = sbl_jtag_uniflash_load_file(&flashOffset, &fileSize);
                    if(status == SystemP_SUCCESS)
                    {
                        status = sbl_jtag_uniflash_write_file(flashOffset, fileSize);
                        if(status == SystemP_SUCCESS)
                        {
                            sbl_jtag_uniflash_verify_file(flashOffset, fileSize);
                        }
                    }
                    break;
                case '3':
                    status = sbl_jtag_uniflash_load_file(&flashOffset, &fileSize);
                    if(status == SystemP_SUCCESS)
                    {
                        status = sbl_jtag_uniflash_verify_file(flashOffset, fileSize);
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
    DebugP_log("\r\n");
    DebugP_log(" [FLASH WRITER] Application exited !!!\r\n");
    DebugP_log("All tests have passed!!\r\n");

    Board_driversClose();
    Drivers_close();
}
