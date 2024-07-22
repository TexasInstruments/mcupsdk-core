#ifndef __BOOTLOADER_UNIFLASH_COMMON_H__
#define __BOOTLOADER_UNIFLASH_COMMON_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <kernel/dpl/SystemP.h>

int32_t Bootloader_uniflashFlashFile(uint32_t flashIndex, uint8_t *buf, uint32_t fileSize, uint32_t flashOffset);

int32_t Bootloader_uniflashFlashFileSector(uint32_t flashIndex, uint8_t *buf, uint32_t fileSize, uint32_t flashOffset);

int32_t Bootloader_uniflashFlashVerifyFile(uint32_t flashIndex, uint8_t *fileBuf, uint32_t fileSize, uint8_t *verifyBuf, uint32_t verifyBufSize, uint32_t flashOffset);

int32_t Bootloader_uniflashFlashErase(uint32_t flashIndex, uint32_t flashOffset, uint32_t flashSize);

int32_t Bootloader_uniflashFlashPhyTuningData(uint32_t flashIndex);


#ifdef __cplusplus
}
#endif

#endif