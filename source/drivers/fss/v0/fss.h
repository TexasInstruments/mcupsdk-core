
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

#ifndef __FSS_H__
#define __FSS_H__

#include <stdint.h>
#include <kernel/dpl/SystemP.h>
#include <drivers/hw_include/cslr_fss.h>
#include <drivers/hw_include/hw_types.h>
#include <drivers/hw_include/csl_types.h>

#define Region_Index_0    ((uint32_t)(0))
#define Region_Index_1    ((uint32_t)(1))
#define Region_Index_2    ((uint32_t)(2))
#define Region_Index_3    ((uint32_t)(3))

typedef void* FSS_Handle;

typedef struct{

    uint32_t size;
    uint32_t startAddress;
    uint32_t regionIndex;
}FSS_ECCRegionConfig;


typedef struct fss_config_s_t
{
    uint32_t ipBaseAddress;
    uint32_t extFlashSize;
}
FSS_Config;

/**
 * @brief Mask the address bits
 *
 * Use this function to remap external flash. Lower 12 bits cannot be masked.
 *
 * The address that will be sent to external flash will be (address & bitMask) | ((bitMask + 1) * segment)
 *
 * Suppose external flash is 32MB and it is required to map upper 16MB to address 0 so that read from address 0
 * would automatically translar to 16MB read. In this case the bitMask would be 0xffffff and segment would be 1.
 * address that would be sent to flash is (0xffffff * address) | 0x1000000
 *
 * @param handle handle to FSS instance
 * @param bitMask bit mask
 * @param segment segment
 * @return SystemP_SUCCESS on successfull execution
 */
int32_t FSS_addressBitMask(FSS_Handle handle, uint32_t bitMask, uint8_t segment);

/**
 * @brief Disable remap
 * 
 * @param handle handle to FSS instance
 * @return SystemP_SUCCESS on successfull execution
 */
int32_t FSS_disableAddressRemap(FSS_Handle handle);

/**
 * @brief Map region A of flash to initial position
 * 
 * @param handle handle to FSS instance
 * @return SystemP_SUCCESS on successfull execution
 */
int32_t FSS_selectRegionA(FSS_Handle handle);

/**
 * @brief Map region N of flash to initial position
 * 
 * @param handle handle to FSS instance
 * @return SystemP_SUCCESS on successfull execution
 */
int32_t FSS_selectRegionB(FSS_Handle handle);

/**
 * @brief Enable ECC for Flash 
 * 
 * This function enabled ECC for the data that is stored in flash.
 * For every 32 Bytes of data, 4 bytes of ECC is assumed. 
 * Make sure the data is in flash is prepared at compile time.
 *
 */
void FSS_enableECC(void);

/**
 * @brief Disable ECC 
 * 
 */
void FSS_disableECC(void);

/**
 * @brief Set size of an ECCM region
 * 
 * @param regionIndex region ID
 * @param size_in_bytes size of region in bytes 
 */
void FSS_setECCRegionSize(uint32_t regionIndex, uint32_t size_in_bytes);

/**
 * @brief Set start address of an ECCM region
 * 
 * @param regionIndex region ID
 * @param ecc_reg_start start address
 */
void FSS_setECCRegionStart(uint32_t regionIndex, uint32_t ecc_reg_start);

int32_t FSS_configECCMRegion(FSS_ECCRegionConfig *parameter);

/**
 * @brief Configure ECCM hardware
 * 
 * @param numRegion region ID
 * @param parameter region config data
 */
int32_t FSS_ConfigEccm(uint32_t numRegion, FSS_ECCRegionConfig *parameter);


#endif