#ifndef __BOOTLOADER_UNIFLASH_RPRC__
#define __BOOTLOADER_UNIFLASH_RPRC__


#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <kernel/dpl/SystemP.h>

int32_t Bootloader_Uniflash_RPRC_flashXIPFile(uint32_t flashIndex, uint8_t *fileBuf, uint32_t fileSize);

int32_t Bootloader_Uniflash_RPRC_flashVerifyXIPFile(uint32_t flashIndex, uint8_t *fileBuf, uint32_t fileSize, uint8_t *verifyBuf, uint32_t verifyBufSize);


#ifdef __cplusplus
}
#endif

#endif