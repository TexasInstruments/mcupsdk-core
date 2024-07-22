
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <board/flash.h>
#include "bootloader_uniflash_mcelf.h"
#include "middleware/tiELFuParser/tielfup32.h"
#include <drivers/bootloader/bootloader_uniflash/bootloader_uniflash_common.h>

static int32_t MCELF_flashVerifyXIPFile(uint32_t flashIndex, uint8_t *fileBuf, uint32_t fileSize, uint8_t *verifyBuf, uint32_t verifyBufSize)
{
	int32_t retval = SystemP_FAILURE;
	Flash_Attrs *flashAttrs;
	Flash_Handle flashHandle;
	uint32_t eraseBlockSize;

	if(fileBuf != NULL)
	{
		ELFUP_ELFPH pht[4];
		ELFUP_Handle elfuph;
		int32_t status = SystemP_FAILURE;

		memset(pht, 0xff, sizeof(pht));

		ELFUP_init(&elfuph, pht, 4);

		flashAttrs = Flash_getAttrs(flashIndex);
		flashHandle = Flash_getHandle(flashIndex);
		if(flashAttrs == NULL || flashHandle == NULL)
		{
			status = SystemP_FAILURE;
		}
		else
		{
			eraseBlockSize = flashAttrs->pageCount * flashAttrs->pageSize;
		}

		for(uint32_t offset = 0; offset < fileSize && !((elfuph.stateNext == ELFUP_PARSER_STATE_END || elfuph.stateNext == ELFUP_PARSER_STATE_ERROR)); offset++)
		{
			ELFUP_update(&elfuph, fileBuf[offset]);
		}

		if(elfuph.stateNext == ELFUP_PARSER_STATE_END)
		{
			// now we have the program header table
			for(uint32_t  i =0; i < elfuph.ELFHeader.ELFH.e_phnum; i++)
			{
				ELFUP_ELFPH32 * entry = &(pht[i].ELFPH);
				if(entry->type == PT_LOAD)
				{
					uint32_t dest_addr = entry->paddr & ~(0xF0000000);
					uint32_t src_addr = (uint32_t)fileBuf + entry->offset;
					uint32_t size = entry->memsz;

					if((dest_addr % eraseBlockSize) == 0)
					{
						/* Only flash to offsets which are a multiple of blockSize */

						if(NULL != verifyBuf && verifyBufSize > 0)
						{
							status = Bootloader_uniflashFlashVerifyFile(flashIndex, (uint8_t*)src_addr, size, verifyBuf, verifyBufSize, dest_addr);
						}
						else
						{
							status = Bootloader_uniflashFlashFile(flashIndex, (uint8_t*)src_addr, size, dest_addr);
						}
						if(status == SystemP_FAILURE)
						{
							break;
						}
					}
					else
					{
						status = SystemP_FAILURE;
						break;
					}
				}
			}
		}
		retval = status;
	}
	return(retval);
}

int32_t Bootloader_Uniflash_MCELF_flashVerifyXIPFile(uint32_t flashIndex, uint8_t *fileBuf, uint32_t fileSize, uint8_t *verifyBuf, uint32_t verifyBufSize)
{
	return MCELF_flashVerifyXIPFile(flashIndex, fileBuf, fileSize, verifyBuf, verifyBufSize);
}

int32_t Bootloader_Uniflash_MCELF_flashXIPFile(uint32_t flashIndex, uint8_t *fileBuf, uint32_t fileSize)
{
	return MCELF_flashVerifyXIPFile(flashIndex, fileBuf, fileSize, NULL, 0);
}