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

#ifndef QSPI__NOR_FLASH_1S_H_
#define QSPI__NOR_FLASH_1S_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <drivers/qspi/v0/lld/qspi_lld.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* ========================================================================== */
/*                             Structure Definitions                          */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 *  \brief  This function initializes the NOR flash.
 *
 *  \pre    QSPI controller has been opened using #QSPI_open()
 *
 *  \param  handle  A #QSPILLD_Handle returned from a #QSPI_open()
 *
 *  \return #SystemP_SUCCESS on success, #SystemP_FAILURE otherwise
 */
int32_t QSPI_norFlashInit(QSPILLD_Handle handle);

/**
 *  \brief  This function tries to read the JEDEC ID from the NOR flash connected to the QSPI peripheral
 *
 *  \pre    QSPI controller has been opened using #QSPI_open()
 *
 *  \param  handle         A #QSPILLD_Handle returned from a #QSPI_open()
 *  \param  manufacturerId Pointer to a uint32_t variable. This will be filled with the manufacturer ID on success
 *  \param  deviceId       Pointer to a uint32_t variable. This will be filled with the device ID on success
 *
 *  \return #SystemP_SUCCESS on success, #SystemP_FAILURE otherwise
 */
int32_t QSPI_norFlashReadId(QSPILLD_Handle handle, uint32_t *manufacturerId, uint32_t *deviceId);

/**
 *  \brief  This function writes data to the flash at a specified offset in config mode
 *
 *  \pre    QSPI controller has been opened using #QSPI_open()
 *
 *  \param  handle    A #QSPILLD_Handle returned from a #QSPI_open()
 *  \param  offset    Offset at which the data is to be written
 *  \param  buf       Buffer which has the data to be written to the flash
 *  \param  len       Number of bytes to be written to the flash
 *
 *  \return #SystemP_SUCCESS on success, #SystemP_FAILURE otherwise
 */
int32_t QSPI_norFlashWrite(QSPILLD_Handle handle, uint32_t offset, uint8_t *buf, uint32_t len);

/**
 *  \brief  This function writes data to the flash at a specified offset using interrupt method
 *
 *  \pre    QSPI controller has been opened using #QSPI_open()
 *
 *  \param  handle    A #QSPILLD_Handle returned from a #QSPI_open()
 *  \param  offset    Offset at which the data is to be written
 *  \param  buf       Buffer which has the data to be written to the flash
 *  \param  len       Number of bytes to be written to the flash
 *
 *  \return #SystemP_SUCCESS on success, #SystemP_FAILURE otherwise
 */
int32_t QSPI_norFlashWriteIntr(QSPILLD_Handle handle, QSPILLD_WriteCmdParams *wrMsg);

/**
 *  \brief  This function reads data from the flash from a specified offset
 *
 *  \pre    QSPI controller has been opened using #QSPI_open()
 *
 *  \param  handle    A #QSPILLD_Handle returned from a #QSPI_open()
 *  \param  offset    Offset at which the data is to be written
 *  \param  buf       Buffer to which data will be read into
 *  \param  len       Number of bytes to be read from the flash
 *
 *  \return #SystemP_SUCCESS on success, #SystemP_FAILURE otherwise
 */
int32_t QSPI_norFlashRead(QSPILLD_Handle handle, uint32_t offset, uint8_t *buf, uint32_t len);

/**
 *  \brief  This function reads the write status flag from the flash memory
 *
 *  \pre    QSPI controller has been opened using #QSPI_open()
 *
 *  \param  handle    A #QSPILLD_Handle returned from a #QSPI_open()
 *  \param  timeOut   Timeout for the wait ready functionality
 *
 *  \return #SystemP_SUCCESS on success, #SystemP_FAILURE otherwise
 */
int32_t QSPI_norFlashWaitReady(QSPILLD_Handle handle, uint32_t timeOut);
/**
 *  \brief  This function reads data from the flash from a specified offset in DMA mode
 *
 *  \pre    QSPI controller has been opened using #QSPI_open()
 *
 *  \param  handle    A #QSPILLD_Handle returned from a #QSPI_open()
 *  \param  offset    Offset at which the data is to be written
 *  \param  buf       Buffer to which data will be read into
 *  \param  len       Number of bytes to be read from the flash
 *
 *  \return #SystemP_SUCCESS on success, #SystemP_FAILURE otherwise
 */
int32_t QSPI_norFlashReadDma(QSPILLD_Handle handle, uint32_t offset, uint8_t *buf, uint32_t len);

/**
 *  \brief  This function erases 1 block of data starting from a provided address
 *
 *  \pre    QSPI controller has been opened using #QSPI_open()
 *
 *  \param  handle    A #QSPILLD_Handle returned from a #QSPI_open()
 *  \param  address   Address of the data block to be erased. This address should be block aligned.
 *
 *  \return #SystemP_SUCCESS on success, #SystemP_FAILURE otherwise
 */
int32_t QSPI_norFlashErase(QSPILLD_Handle handle, uint32_t address);

/**
 *  \brief  This function gets the serial flash discoverable parameter information from the flash
 *
 *  \pre    QSPI controller has been opened using #QSPI_open()
 *
 *  \param  handle    A #QSPI_Handle returned from a #QSPI_open()
 *  \param  offset    Address in the flash to read SFDP information
 *  \param  buf       Buffer to which data will be read into
 *  \param  len       Number of bytes to be read from the flash
 *
 *  \return #SystemP_SUCCESS on success, #SystemP_FAILURE otherwise
 */
int32_t QSPI_norFlashReadSfdp(QSPILLD_Handle handle, uint32_t offset, uint8_t *buf, uint32_t len);
#ifdef __cplusplus
}
#endif

#endif /* #ifndef QSPI__NOR_FLASH_1S_H_ */

/** @} */

