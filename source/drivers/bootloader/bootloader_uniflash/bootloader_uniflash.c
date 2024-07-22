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

#include <string.h>
#include <board/flash.h>
#include <drivers/bootloader/bootloader_priv.h>
#include <drivers/bootloader/bootloader_uniflash/bootloader_uniflash.h>
#include <drivers/bootloader/bootloader_uniflash/bootloader_uniflash_mcelf/bootloader_uniflash_mcelf.h>
#include <drivers/bootloader/bootloader_uniflash/bootloader_uniflash_rprc/bootloader_uniflash_rprc.h>
#include <drivers/bootloader/bootloader_uniflash/bootloader_uniflash_common.h>

#ifdef DRV_VERSION_MMCSD_V0
#include <drivers/mmcsd.h>
#include <drivers/bootloader/bootloader_mmcsd_raw.h>
#endif

static void Bootloader_uniflashInitRespHeader(Bootloader_UniflashResponseHeader *respHeader);

#ifdef DRV_VERSION_MMCSD_V0
static int32_t Bootloader_uniflashFlashFileMMCSDRaw(uint32_t flashIndex, uint8_t *buf, uint32_t fileSize, uint32_t flashOffset);
static int32_t Bootloader_uniflashFlashVerifyFileMMCSDRaw(uint32_t flashIndex, uint8_t *fileBuf, uint32_t fileSize, uint8_t *verifyBuf, uint32_t verifyBufSize, uint32_t flashOffset);
#endif

int32_t Bootloader_uniflashProcessFlashCommands(Bootloader_UniflashConfig *config, Bootloader_UniflashResponseHeader *respHeader)
{
	int32_t status = SystemP_SUCCESS;
	Bootloader_UniflashFileHeader fileHeader;

	memcpy(&fileHeader, config->buf, sizeof(Bootloader_UniflashFileHeader));

	Bootloader_uniflashInitRespHeader(respHeader);

	if(fileHeader.magicNumber != BOOTLOADER_UNIFLASH_FILE_HEADER_MAGIC_NUMBER)
	{
	    respHeader->statusCode = BOOTLOADER_UNIFLASH_STATUSCODE_MAGIC_ERROR;
	    status = SystemP_FAILURE;
	}

	/* Obtain the actual filesize */
	config->bufSize = fileHeader.actualFileSize;

	/* Check if the actual filesize is 16 B aligned. This is smallest program granularity suppported by NOR flashes.
	 * If it is not aligned, we have to write 1-15 bytes extra. Since xmodem would have already padded zeros into the
	 * file buffer for 1024B alignment, we can assume that these 1-15 bytes would be zero.
	 */
	uint32_t remainder = config->bufSize % 16U;
	if(remainder != 0)
	{
		config->bufSize += (16U - remainder);
	}
	else
	{
		/* do nothing */
	}

	if(SystemP_SUCCESS == status)
	{
	    uint32_t opType = (fileHeader.operationTypeAndFlags) & (uint32_t)0xFF;

	    switch(opType)
	    {
	        case BOOTLOADER_UNIFLASH_OPTYPE_FLASH:
	            /* flash the file at the given offset*/
	            status = Bootloader_uniflashFlashFile(config->flashIndex, config->buf + sizeof(Bootloader_UniflashFileHeader), config->bufSize, fileHeader.offset);
	            if(status != SystemP_SUCCESS)
	            {
	                respHeader->statusCode = BOOTLOADER_UNIFLASH_STATUSCODE_FLASH_ERROR;
	            }
	            else
	            {
	                /* verify the file at the given offset */
	                status = Bootloader_uniflashFlashVerifyFile(config->flashIndex, config->buf + sizeof(Bootloader_UniflashFileHeader), config->bufSize, config->verifyBuf, config->verifyBufSize, fileHeader.offset);
	                if(status != SystemP_SUCCESS)
	                {
	                    respHeader->statusCode = BOOTLOADER_UNIFLASH_STATUSCODE_FLASH_VERIFY_ERROR;
	                }
	            }
	            break;
            case BOOTLOADER_UNIFLASH_OPTYPE_FLASH_SECTOR:
	            /* flash the file at the given offset*/
	            status = Bootloader_uniflashFlashFileSector(config->flashIndex, config->buf + sizeof(Bootloader_UniflashFileHeader), config->bufSize, fileHeader.offset);
	            if(status != SystemP_SUCCESS)
	            {
	                respHeader->statusCode = BOOTLOADER_UNIFLASH_STATUSCODE_FLASH_ERROR;
	            }
	            else
	            {
	                /* verify the file at the given offset */
	                status = Bootloader_uniflashFlashVerifyFile(config->flashIndex, config->buf + sizeof(Bootloader_UniflashFileHeader), config->bufSize, config->verifyBuf, config->verifyBufSize, fileHeader.offset);
	                if(status != SystemP_SUCCESS)
	                {
	                    respHeader->statusCode = BOOTLOADER_UNIFLASH_STATUSCODE_FLASH_VERIFY_ERROR;
	                }
	            }
	            break;

	        case BOOTLOADER_UNIFLASH_OPTYPE_FLASH_VERIFY:
	            /* verify the file at the given offset */
	            status = Bootloader_uniflashFlashVerifyFile(config->flashIndex, config->buf + sizeof(Bootloader_UniflashFileHeader), config->bufSize, config->verifyBuf, config->verifyBufSize, fileHeader.offset);
	            if(status != SystemP_SUCCESS)
	            {
	                respHeader->statusCode = BOOTLOADER_UNIFLASH_STATUSCODE_FLASH_VERIFY_ERROR;
	            }
	            break;

	        case BOOTLOADER_UNIFLASH_OPTYPE_FLASH_ERASE:
	        	/* erase the flash from given offset for the given size */
	        	status = Bootloader_uniflashFlashErase(config->flashIndex, fileHeader.offset, fileHeader.eraseSize);
	        	if(status != SystemP_SUCCESS)
	            {
	                respHeader->statusCode = BOOTLOADER_UNIFLASH_STATUSCODE_FLASH_ERASE_ERROR;
	            }
	            break;

	        case BOOTLOADER_UNIFLASH_OPTYPE_FLASH_XIP:
	            /* flash the XIP file, flash offsets are within the file itself */
				if(config->imageFormatType == BOOTLOADER_UNIFLASH_IMAGE_FORMAT_TYPE_MCELF)
				{
					status = Bootloader_Uniflash_MCELF_flashXIPFile(config->flashIndex, config->buf + sizeof(Bootloader_UniflashFileHeader), config->bufSize);
				}
				else
				{
					status = Bootloader_Uniflash_RPRC_flashXIPFile(config->flashIndex, config->buf + sizeof(Bootloader_UniflashFileHeader), config->bufSize);
				}
	            if(status != SystemP_SUCCESS)
	            {
	                respHeader->statusCode = BOOTLOADER_UNIFLASH_STATUSCODE_FLASH_ERROR;
	            }
	            else
	            {
	                /* verify the file at the given offset */
					if(config->imageFormatType == BOOTLOADER_UNIFLASH_IMAGE_FORMAT_TYPE_MCELF)
					{
						status = Bootloader_Uniflash_MCELF_flashVerifyXIPFile(config->flashIndex, config->buf + sizeof(Bootloader_UniflashFileHeader), config->bufSize, config->verifyBuf, config->verifyBufSize);
					}
					else
					{
						status = Bootloader_Uniflash_RPRC_flashVerifyXIPFile(config->flashIndex, config->buf + sizeof(Bootloader_UniflashFileHeader), config->bufSize, config->verifyBuf, config->verifyBufSize);
					}
	                if(status != SystemP_SUCCESS)
	                {
	                    respHeader->statusCode = BOOTLOADER_UNIFLASH_STATUSCODE_FLASH_VERIFY_ERROR;
	                }
	            }
	            break;

	        case BOOTLOADER_UNIFLASH_OPTYPE_FLASH_VERIFY_XIP:
	            /* verify the file. Flash offsets are within the file itself */
				if(config->imageFormatType == BOOTLOADER_UNIFLASH_IMAGE_FORMAT_TYPE_MCELF)
				{
					status = Bootloader_Uniflash_MCELF_flashVerifyXIPFile(config->flashIndex, config->buf + sizeof(Bootloader_UniflashFileHeader), config->bufSize, config->verifyBuf, config->verifyBufSize);
				}
				else
				{
					status = Bootloader_Uniflash_RPRC_flashVerifyXIPFile(config->flashIndex, config->buf + sizeof(Bootloader_UniflashFileHeader), config->bufSize, config->verifyBuf, config->verifyBufSize);
				}
	            if(status != SystemP_SUCCESS)
	            {
	                respHeader->statusCode = BOOTLOADER_UNIFLASH_STATUSCODE_FLASH_VERIFY_ERROR;
	            }
	            break;

	        case BOOTLOADER_UNIFLASH_OPTYPE_FLASH_TUNING_DATA:
	            /* verify the file. Flash offsets are within the file itself */
	            status = Bootloader_uniflashFlashPhyTuningData(config->flashIndex);
	            if(status != SystemP_SUCCESS)
	            {
	                respHeader->statusCode = BOOTLOADER_UNIFLASH_STATUSCODE_FLASH_ERROR;
	            }
	            break;

#ifdef DRV_VERSION_MMCSD_V0
            case BOOTLOADER_UNIFLASH_OPTYPE_EMMC_FLASH:
                /* flash the file at the given offset of MMCSD */
	            status = Bootloader_uniflashFlashFileMMCSDRaw(config->flashIndex, config->buf + sizeof(Bootloader_UniflashFileHeader), config->bufSize, fileHeader.offset);
	            if(status != SystemP_SUCCESS)
	            {
	                respHeader->statusCode = BOOTLOADER_UNIFLASH_STATUSCODE_FLASH_ERROR;
	            }
                else
	            {
	                /* verify the file at the given offset */
	                status = Bootloader_uniflashFlashVerifyFileMMCSDRaw(config->flashIndex, config->buf + sizeof(Bootloader_UniflashFileHeader), config->bufSize, config->verifyBuf, config->verifyBufSize, fileHeader.offset);
	                if(status != SystemP_SUCCESS)
	                {
	                    respHeader->statusCode = BOOTLOADER_UNIFLASH_STATUSCODE_FLASH_VERIFY_ERROR;
	                }
	            }
                break;

            case BOOTLOADER_UNIFLASH_OPTYPE_EMMC_VERIFY:
                /* verify the file at the given offset */
	            status = Bootloader_uniflashFlashVerifyFileMMCSDRaw(config->flashIndex, config->buf + sizeof(Bootloader_UniflashFileHeader), config->bufSize, config->verifyBuf, config->verifyBufSize, fileHeader.offset);
	            if(status != SystemP_SUCCESS)
	            {
	                respHeader->statusCode = BOOTLOADER_UNIFLASH_STATUSCODE_FLASH_VERIFY_ERROR;
	            }
	            break;
#endif
	        default:
	            respHeader->statusCode = BOOTLOADER_UNIFLASH_STATUSCODE_OPTYPE_ERROR;
	            break;
	    }
	}

	return status;
}

void Bootloader_uniflashInitRespHeader(Bootloader_UniflashResponseHeader *respHeader)
{
	if(respHeader != NULL)
	{
	    respHeader->magicNumber = BOOTLOADER_UNIFLASH_RESP_HEADER_MAGIC_NUMBER;
	    respHeader->statusCode  = BOOTLOADER_UNIFLASH_STATUSCODE_SUCCESS;
	    respHeader->rsv0        = 0xDEADBABE;
	    respHeader->rsv1        = 0xDEADBABE;
	}
}


#ifdef DRV_VERSION_MMCSD_V0
static int32_t Bootloader_uniflashFlashFileMMCSDRaw(uint32_t mmcsdIndex, uint8_t *buf, uint32_t fileSize, uint32_t flashOffset)
{
    int32_t status = SystemP_SUCCESS;

    MMCSD_Handle handle = MMCSD_getHandle(mmcsdIndex);
    if(handle == NULL)
    {
        status = SystemP_FAILURE;
    }
    else if (MMCSD_CARD_TYPE_NO_DEVICE == ((MMCSD_Config *)handle)->object->cardType)
    {
    	status = SystemP_FAILURE;
    }
    else
    {
        status = MMCSD_enableBootPartition(handle, 1);
    }

    if(status != SystemP_SUCCESS)
    {

    }
    else
    {
        status = Bootloader_MmcsdRaw_writeToOffset(handle, buf, fileSize, flashOffset);
    }

    return status;
}

static int32_t Bootloader_uniflashFlashVerifyFileMMCSDRaw(uint32_t mmcsdIndex, uint8_t *fileBuf, uint32_t fileSize, uint8_t *verifyBuf, uint32_t verifyBufSize, uint32_t flashOffset)
{
	int32_t status = SystemP_SUCCESS;

    MMCSD_Handle handle = MMCSD_getHandle(mmcsdIndex);

	if(handle == NULL)
	{
	   status = SystemP_FAILURE;
	}
    else
    {
        status = MMCSD_enableBootPartition(handle, 1);
    }

    if(status != SystemP_SUCCESS)
    {

    }
    else
	{
	    uint32_t curOffset, totalChunks, curChunk, chunkSize, remainSize;
	    uint8_t *srcAddr;
	    int32_t diff;


	    /* start writing from buffer to flash */
	    chunkSize = verifyBufSize;

	    srcAddr = fileBuf;
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

	        /* clear verify buf to avoid comparing stale data */
	        memset(verifyBuf, 0, verifyBufSize);

            status = Bootloader_MmcsdRaw_readFromOffset(handle, verifyBuf, chunkSize, curOffset);

	        if(status == SystemP_SUCCESS)
	        {
	            /* check if data read from flash matches, data read from file */
	            diff = memcmp(verifyBuf, srcAddr, chunkSize);

	            if(diff != 0)
	            {
	                status = SystemP_FAILURE;
	            }
	        }
	        curOffset += chunkSize;
	        srcAddr += chunkSize;
	        remainSize -= chunkSize;
	        curChunk++;
	    }
	}

	return status;
}

#endif

