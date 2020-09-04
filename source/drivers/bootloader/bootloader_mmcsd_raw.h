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

#ifndef BOOTLOADER_MMCSD_RAW_H
#define BOOTLOADER_MMCSD_RAW_H

#ifdef __cplusplus
extern "C"
{
#endif


#include <drivers/bootloader.h>
#include <drivers/mmcsd.h>

/**
 * \brief Arguments to be passed to driver implementation callbacks when boot media is MMCSD
 */
typedef struct Bootloader_MmcsdArgs_s
{
    uint32_t MMCSDIndex;
    uint32_t curOffset;
    uint32_t appImageOffset;

} Bootloader_MmcsdArgs;

extern Bootloader_Fxns gBootloaderMmcsdFxns;

/**
 * \brief Read data from an offset from MMCSD. The offset need not be aligned with the blocksize.
 *
 * Make sure the required partition is selected to read from
 * \param handle [in]    MMCSD_Handle retured from #MMCSD_open()
 * \param dst    [out]   Destination to write the read data from MMCSD
 * \param len    [in]    Length of data to read from MMCSD
 * \param offset [in]    Offset in MMCSD from where data is to be read
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t Bootloader_MmcsdRaw_readFromOffset(MMCSD_Handle handle, void *dst, uint32_t len, uint32_t offset);

/**
 * \brief Write data to an offset to MMCSD. The offset need not be aligned with the blocksize.
 *
 * Make sure the required partition is selected to read from
 * \param handle [in]    MMCSD_Handle retured from #MMCSD_open()
 * \param buf    [in]    Source buffer to write to MMCSD
 * \param len    [in]    Length of data to write to MMCSD
 * \param offset [in]    Offset in MMCSD to where data is to be written
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t Bootloader_MmcsdRaw_writeToOffset(MMCSD_Handle handle, void *buf, uint32_t len, uint32_t offset);


#ifdef __cplusplus
}
#endif

#endif  /* BOOTLOADER_MMCSD_RAW_H */