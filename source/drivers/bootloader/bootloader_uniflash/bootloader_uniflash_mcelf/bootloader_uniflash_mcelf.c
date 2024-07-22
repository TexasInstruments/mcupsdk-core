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