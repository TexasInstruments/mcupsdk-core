
#include <stdio.h>
#include <string.h>
#include <board/flash.h>
#include "bootloader_uniflash_common.h"

int32_t Bootloader_uniflashFlashFile(uint32_t flashIndex, uint8_t *buf, uint32_t fileSize, uint32_t flashOffset)
{
	int32_t status = SystemP_SUCCESS;

	Flash_Attrs *flashAttrs;
	Flash_Handle flashHandle;
	uint32_t eraseBlockSize;

	flashAttrs = Flash_getAttrs(flashIndex);
	flashHandle = Flash_getHandle(flashIndex);

	if(flashAttrs == NULL || flashHandle == NULL)
	{
	   status=SystemP_FAILURE;
	}
	else
	{
	    eraseBlockSize = flashAttrs->pageCount * flashAttrs->pageSize;
	}

	if((status == SystemP_SUCCESS) && ((flashOffset % eraseBlockSize) != 0))
	{
		/* Only flash to offsets which are a multiple of blockSize */
		status=SystemP_FAILURE;
	}

	if(status==SystemP_SUCCESS)
	{
	    uint32_t curOffset, totalChunks, curChunk, chunkSize, remainSize, blockNum, pageNum;
	    uint8_t *srcAddr;

	    /* start writing from buffer to flash */
	    chunkSize = eraseBlockSize;

	    srcAddr = buf;
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

	        status = Flash_offsetToBlkPage(flashHandle, curOffset, &blockNum, &pageNum);
	        if(status == SystemP_SUCCESS)
	        {
	            status = Flash_eraseBlk(flashHandle, blockNum);
	            if(status == SystemP_SUCCESS)
	            {
	                status = Flash_write(flashHandle, curOffset, srcAddr, chunkSize);
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

int32_t Bootloader_uniflashFlashFileSector(uint32_t flashIndex, uint8_t *buf, uint32_t fileSize, uint32_t flashOffset)
{
	int32_t status = SystemP_SUCCESS;

	Flash_Attrs *flashAttrs;
	Flash_Handle flashHandle;
	uint32_t eraseSectorSize;

	flashAttrs = Flash_getAttrs(flashIndex);
	flashHandle = Flash_getHandle(flashIndex);

	if(flashAttrs == NULL || flashHandle == NULL)
	{
	   status=SystemP_FAILURE;
	}
	else
	{
	    eraseSectorSize = flashAttrs->sectorSize;
	}

	if((status == SystemP_SUCCESS) && ((flashOffset % eraseSectorSize) != 0))
	{
		/* Only flash to offsets which are a multiple of sectorSize */
		status=SystemP_FAILURE;
	}

	if(status==SystemP_SUCCESS)
	{
	    uint32_t curOffset, totalChunks, curChunk, chunkSize, remainSize, sectorNum, pageNum;
	    uint8_t *srcAddr;

	    /* start writing from buffer to flash */
	    chunkSize = eraseSectorSize;

	    srcAddr = buf;
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

	        status = Flash_offsetToSectorPage(flashHandle, curOffset, &sectorNum, &pageNum);
	        if(status == SystemP_SUCCESS)
	        {
	            status = Flash_eraseSector(flashHandle, sectorNum);
	            if(status == SystemP_SUCCESS)
	            {
	                status = Flash_write(flashHandle, curOffset, srcAddr, chunkSize);
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

int32_t Bootloader_uniflashFlashVerifyFile(uint32_t flashIndex, uint8_t *fileBuf, uint32_t fileSize, uint8_t *verifyBuf, uint32_t verifyBufSize, uint32_t flashOffset)
{
	int32_t status = SystemP_SUCCESS;

	Flash_Attrs *flashAttrs;
	Flash_Handle flashHandle;
	flashAttrs = Flash_getAttrs(flashIndex);
	flashHandle = Flash_getHandle(flashIndex);

	if(flashAttrs == NULL || flashHandle == NULL)
	{
	   status=SystemP_FAILURE;
	}

	if(status==SystemP_SUCCESS)
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

	        status = Flash_read(flashHandle, curOffset, verifyBuf, chunkSize);

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

int32_t Bootloader_uniflashFlashErase(uint32_t flashIndex, uint32_t flashOffset, uint32_t eraseSize)
{
	int32_t status = SystemP_SUCCESS;

	Flash_Attrs *flashAttrs;
	Flash_Handle flashHandle;
	uint32_t eraseBlockSize;
	uint32_t flashSize;

	flashAttrs = Flash_getAttrs(flashIndex);
	flashHandle = Flash_getHandle(flashIndex);

	if(flashAttrs == NULL || flashHandle == NULL)
	{
	   status=SystemP_FAILURE;
	}
	else
	{
	    eraseBlockSize = flashAttrs->pageCount * flashAttrs->pageSize;
	    flashSize = eraseBlockSize * flashAttrs->blockCount;
	}

	if((status == SystemP_SUCCESS) && ((flashOffset % eraseBlockSize) != 0))
	{
		/* Only flash to offsets which are a multiple of blockSize */
		status=SystemP_FAILURE;
	}

	if((status == SystemP_SUCCESS) && (eraseSize > flashSize))
	{
		status=SystemP_FAILURE;
	}

	if(status==SystemP_SUCCESS)
	{
	    uint32_t curOffset, totalChunks, curChunk, chunkSize, remainSize, blockNum, pageNum;

	    /* start writing from buffer to flash */
	    chunkSize = eraseBlockSize;

	    remainSize = eraseSize;
	    curOffset = flashOffset;
	    curChunk = 1;
	    totalChunks = (eraseSize + (chunkSize-1))/chunkSize;
	    while(curChunk <= totalChunks)
	    {
	        if(remainSize < chunkSize)
	        {
	            chunkSize = remainSize;
	        }

	        status = Flash_offsetToBlkPage(flashHandle, curOffset, &blockNum, &pageNum);
	        if(status == SystemP_SUCCESS)
	        {
	            status = Flash_eraseBlk(flashHandle, blockNum);
	        }
	        curOffset += chunkSize;
	        remainSize -= chunkSize;
	        curChunk++;
	    }
	}
	return status;
}


int32_t Bootloader_uniflashFlashPhyTuningData(uint32_t flashIndex)
{
	int32_t status = SystemP_SUCCESS;

	Flash_Attrs *flashAttrs;
	Flash_Handle flashHandle;

	flashAttrs = Flash_getAttrs(flashIndex);
	flashHandle = Flash_getHandle(flashIndex);

	if(flashAttrs == NULL || flashHandle == NULL)
	{
	   status=SystemP_FAILURE;
	}
	else
	{
		uint32_t phyTuningData, phyTuningDataSize;
		uint32_t phyTuningDataFlashOffset;
		uint32_t blockNum, pageNum;

		OSPI_phyGetTuningData(&phyTuningData, &phyTuningDataSize);
		phyTuningDataFlashOffset =  Flash_getPhyTuningOffset(flashHandle);

		status = Flash_offsetToBlkPage(flashHandle, phyTuningDataFlashOffset, &blockNum, &pageNum);
		if(status == SystemP_SUCCESS)
		{
		    status = Flash_eraseBlk(flashHandle, blockNum);
		}
		status = Flash_write(flashHandle, phyTuningDataFlashOffset, (uint8_t *)phyTuningData, phyTuningDataSize);
	}

	return status;
}
