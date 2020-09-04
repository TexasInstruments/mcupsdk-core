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

#ifndef BOOTLOADER_UNIFLASH_H_
#define BOOTLOADER_UNIFLASH_H_

#ifdef __cplusplus
extern "C"
{
#endif


#include <stdint.h>
#include <drivers/hw_include/soc_config.h>

#define BOOTLOADER_UNIFLASH_FILE_HEADER_MAGIC_NUMBER  (0x46554C42) /* BLXF */
#define BOOTLOADER_UNIFLASH_RESP_HEADER_MAGIC_NUMBER  (0x52554C42) /* BLXR */

#define BOOTLOADER_UNIFLASH_OPTYPE_FLASH             (0xF0)
#define BOOTLOADER_UNIFLASH_OPTYPE_FLASH_VERIFY      (0xF1)
#define BOOTLOADER_UNIFLASH_OPTYPE_FLASH_XIP         (0xF2)
#define BOOTLOADER_UNIFLASH_OPTYPE_FLASH_VERIFY_XIP  (0xF3)
#define BOOTLOADER_UNIFLASH_OPTYPE_FLASH_TUNING_DATA (0xF4)
#define BOOTLOADER_UNIFLASH_OPTYPE_FLASH_ERASE       (0xFE)

#ifdef DRV_VERSION_MMCSD_V0
#define BOOTLOADER_UNIFLASH_OPTYPE_EMMC_FLASH        (0xF5)
#define BOOTLOADER_UNIFLASH_OPTYPE_EMMC_VERIFY       (0xF6)
#endif

#define BOOTLOADER_UNIFLASH_STATUSCODE_SUCCESS                   (0x00000000)
#define BOOTLOADER_UNIFLASH_STATUSCODE_MAGIC_ERROR               (0x10000001)
#define BOOTLOADER_UNIFLASH_STATUSCODE_OPTYPE_ERROR              (0x20000001)
#define BOOTLOADER_UNIFLASH_STATUSCODE_FLASH_ERROR               (0x30000001)
#define BOOTLOADER_UNIFLASH_STATUSCODE_FLASH_VERIFY_ERROR        (0x40000001)
#define BOOTLOADER_UNIFLASH_STATUSCODE_FLASH_ERASE_ERROR         (0x50000001)

typedef struct Bootloader_UniflashFileHeader_s
{
	uint32_t magicNumber;
	/* BOOTLOADER_UNIFLASH_FILE_HEADER_MAGIC_NUMBER */

	uint32_t operationTypeAndFlags;
	/* LSByte - Operation Type:flash, verify flash or erase */

	uint32_t offset;
	/* Offset to flash, verify flash or erase flash */

	uint32_t eraseSize;
	/* Size of flash to erase */

	uint32_t actualFileSize;
	/* Size of the file sent. This is needed because xmodem returns a padded file size */

	uint32_t rsv1;
	uint32_t rsv2;
	uint32_t rsv3;
	/* Reserved */
} Bootloader_UniflashFileHeader;

typedef struct  Bootloader_xmodemResponseHeader_s
{
	uint32_t magicNumber;
	/* BOOTLOADER_UNIFLASH_RESP_HEADER_MAGIC_NUMBER */

	uint32_t statusCode;
	/* LSByte - 0  if success, 1 if error. Error code in first 3 bytes*/

	uint32_t rsv0;
	uint32_t rsv1;
	uint32_t rsv2;
	uint32_t rsv3;
	uint32_t rsv4;
	uint32_t rsv5;
	/* Reserved */
} Bootloader_UniflashResponseHeader;

typedef struct Bootloader_UniflashConfig_s
{
	uint32_t flashIndex;
	uint8_t *buf;
	uint32_t bufSize;
	uint8_t *verifyBuf;
	uint32_t verifyBufSize;

} Bootloader_UniflashConfig;

int32_t Bootloader_uniflashProcessFlashCommands(Bootloader_UniflashConfig *config, Bootloader_UniflashResponseHeader *respHeader);


#ifdef __cplusplus
}
#endif

#endif