/*
 *  Copyright (C) 2021-2024 Texas Instruments Incorporated
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

#ifndef FLASH_H_
#define FLASH_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stddef.h>
#include <kernel/dpl/SystemP.h>
#include <drivers/hw_include/soc_config.h>
#include <board/flash/flash_config.h>

#define FLASH_INVALID_VALUE (0xFFFFFFFFU)

#define CONFIG_FLASH_TYPE_SERIAL            ((uint32_t)0x1)
#define CONFIG_FLASH_TYPE_PARALLEL          ((uint32_t)0x2)

/**
 * \brief Flash type supported
 */
#define CONFIG_FLASH_TYPE_SERIAL_NOR               (0x00U)
#define CONFIG_FLASH_TYPE_SERIAL_NAND              (0x01U)
#define CONFIG_FLASH_TYPE_PARALLEL_NOR             (0x02U)
#define CONFIG_FLASH_TYPE_PARALLEL_NAND            (0x03U)
#define CONFIG_FLASH_TYPE_INVALID                  (0xFFU)

/**
 *  \defgroup BOARD_FLASH_MODULE APIs for FLASH
 *  \ingroup BOARD_MODULE
 *
 *  This module contains APIs to program and use the Flash module on the board like XSPI NOR Flash.
 *  See \ref BOARD_FLASH_PAGE for more details.
 *
 *  @{
 */


/**
 * \brief Handle to the FLash driver returned by Flash_opem()
 */
typedef void  *Flash_Handle;

/**
 * \brief Forward declaration of \ref Flash_Config
 */
typedef struct Flash_Config_s Flash_Config;

/**
 * \brief Forward declaration of \ref Flash_Params
 */
typedef struct Flash_Params_s Flash_Params;

/**
 * \brief Flash device config. This will be part of the flash config, so has to
 *        be filled by sysconfig or otherwise before invoking Flash_open
 */

typedef struct
{
    /* data */
    uint8_t  cmdWrsr;
    uint8_t  cmdPageLoad;
    uint8_t  cmdPageProg;
    uint32_t srWipReg;
    uint32_t xspiRdsrDummy;
    uint32_t progStatusReg;
    uint32_t xspiProgStatusReg;
    uint32_t eraseStatusReg;
    uint32_t xspiEraseStatusReg;
    uint8_t  srProgStatus;
    uint8_t  srEraseStatus;
    uint8_t  srWriteProtectReg;
    uint8_t  srWriteProtectMask;

}Flash_NandConfig;


typedef struct Flash_DevConfig_s {

    uint8_t  cmdExtType;
    uint8_t  byteOrder;
    uint8_t  enable4BAddr;
    uint8_t  addrByteSupport;

    uint8_t  fourByteAddrEnSeq;
    uint8_t  cmdWren;
    uint8_t  cmdRdsr;
    uint8_t  srWip;

    uint8_t  cmdPageLoadCyc1;
    uint8_t  cmdPageLoadCyc2;
    uint8_t  cmdRandomReadCyc1;
    uint8_t  cmdRandomReadCyc2;
    uint8_t  cmdRandomInput;
    uint8_t  cmdPageProgCyc1;
    uint8_t  cmdPageProgCyc2;
    uint8_t  pageColAddrCyc;
    uint8_t  pageRowAddrCyc;
    uint8_t  cmdReadStatus;
    uint8_t  cmdReset;


    uint8_t  srWel;
    uint8_t  resetType;
    uint8_t  deviceBusyType;
    uint8_t  xspiWipRdCmd;

    uint32_t xspiWipReg;
    uint32_t xspiWipBit;
    uint32_t flashWriteTimeout;
    uint32_t flashBusyTimeout;

    FlashCfg_EraseConfig eraseCfg;
    FlashCfg_ReadIDConfig idCfg;
    FlashCfg_ProtoEnConfig protocolCfg;
    Flash_NandConfig *nandCfg;

} Flash_DevConfig;

/**
 * \name Flash driver implementation callbacks
 *
 * @{
 */

/**
 * \brief Driver implementation to open a specific flash driver
 *
 * Typically this callback is hidden from the end application and is implemented
 * when a new type of flash device needs to be implemented.
 *
 * \param config [in] Flash configuration for the specific flash device
 * \param params [in] User controllable parameters when opening the flash device
 *
 * \return SystemP_SUCCESS on success, else failure
 */
typedef int32_t (*Flash_OpenFxn)(Flash_Config *config, Flash_Params *params);

/**
 * \brief Driver implementation to close a specific flash driver
 *
 * Typically this callback is hidden from the end application and is implemented
 * when a new type of flash device needs to be implemented.
 *
 * \param config [in] Flash configuration for the specific flash device
 *
 * \return SystemP_SUCCESS on success, else failure
 */
typedef void (*Flash_CloseFxn)(Flash_Config *config);

/**
 * \brief Driver implementation to read from flash using a specific flash driver
 *
 * Typically this callback is hidden from the end application and is implemented
 * when a new type of flash device needs to be implemented.
 *
 * \param config [in] Flash configuration for the specific flash device
 * \param offset [in] Offset in the flash from where to start the read
 * \param buf   [in] Buffer into which to read the data into
 * \param len [in] Length of the data to read, in bytes
 *
 * \return SystemP_SUCCESS on success, else failure
 */
typedef int32_t (*Flash_ReadFxn)(Flash_Config *config, uint32_t offset,
                                   uint8_t *buf, uint32_t len) ;

/**
 * \brief Driver implementation to write to flash using specific flash driver
 *
 * Typically this callback is hidden from the end application and is implemented
 * when a new type of flash device needs to be implemented.
 *
 * \param config [in] Flash configuration for the specific flash device
 * \param offset [in] Offset in the flash from where to start the write.
 * \param buf   [in] Buffer which has the data to write.
 * \param len [in] Length of the data to write, in bytes
 *
 * \return SystemP_SUCCESS on success, else failure
 */
typedef int32_t (*Flash_WriteFxn)(Flash_Config *config, uint32_t offset,
                                   uint8_t *buf, uint32_t len) ;

/**
 * \brief Driver implementation to erase a block using a specific flash driver
 *
 * Typically this callback is hidden from the end application and is implemented
 * when a new type of flash device needs to be implemented.
 *
 * \param config [in] Flash configuration for the specific flash device
 * \param blockNum [in] Block number to erase.
 *
 * \return SystemP_SUCCESS on success, else failure
 */
typedef int32_t (*Flash_EraseFxn)(Flash_Config *config, uint32_t blockNum);

/**
 * \brief Driver implementation to erase a sector using a specific flash driver
 *
 * Typically this callback is hidden from the end application and is implemented
 * when a new type of flash device needs to be implemented.
 *
 * \param config [in] Flash configuration for the specific flash device
 * \param sectorNum [in] Sector number to erase.
 *
 * \return SystemP_SUCCESS on success, else failure
 */
typedef int32_t (*Flash_EraseSectorFxn)(Flash_Config *config, uint32_t sectorNum);

/**
 * \brief Driver implementation to soft reset the flash
 *
 * Typically this callback is hidden from the end application and is implemented
 * when a new type of flash device needs to be implemented.
 *
 * \param config [in] Flash configuration for the specific flash device
 * \param sectorNum [in] Sector number to erase.
 *
 * \return SystemP_SUCCESS on success, else failure
 */
typedef int32_t (*Flash_ResetFxn)(Flash_Config *config);

/**
 * \brief Driver implementation to enable DAC mode in Flash
 *
 * Typically this callback is hidden from the end application and is implemented
 * when a new type of flash device needs to be implemented.
 *
 * \param config [in] Flash configuration for the specific flash device
 *
 * \return SystemP_SUCCESS on success, else failure
 */
typedef int32_t (*Flash_EnableDacModeFxn)(Flash_Config *config);

/**
 * \brief Driver implementation to enable DAC mode in Flash
 *
 * Typically this callback is hidden from the end application and is implemented
 * when a new type of flash device needs to be implemented.
 *
 * \param config [in] Flash configuration for the specific flash device
 *
 * \return SystemP_SUCCESS on success, else failure
 */
typedef int32_t (*Flash_DisableDacModeFxn)(Flash_Config *config);

/**
 * \brief User implementation of a custom function to handle vendor specific quirks
 *
 * Typically this callback is hidden from the end application and is implemented
 * when a new type of flash device needs to be implemented.
 *
 * \param config [in] Flash configuration for the specific flash device
 *
 * \return SystemP_SUCCESS on success, else failure
 */
typedef int32_t (*Flash_quirksFxn)(Flash_Config *config);

/**
 * \brief User implementation of a custom function to configure flash to operate in a specific protocol
 *
 * \param config [in] Flash configuration for the specific flash device
 *
 * \return SystemP_SUCCESS on success, else failure
 */
typedef int32_t (*Flash_custProtocolFxn)(Flash_Config *config);

/** @} */


/**
 * \brief Parameters passed during Flash_open()
 */
typedef struct Flash_Params_s {

    Flash_quirksFxn quirksFxn;
    Flash_custProtocolFxn custProtoFxn;

} Flash_Params;

/**
 * \brief Driver implementation callbacks
 */
typedef struct Flash_Fxns_s
{
    Flash_OpenFxn  openFxn;  /**< Flash driver implementation specific callback */
    Flash_CloseFxn closeFxn; /**< Flash driver implementation specific callback */
    Flash_ReadFxn  readFxn;  /**< Flash driver implementation specific callback */
    Flash_WriteFxn writeFxn; /**< Flash driver implementation specific callback */
    Flash_EraseFxn eraseFxn; /**< Flash driver implementation specific callback */
    Flash_EraseSectorFxn eraseSectorFxn; /**< Flash driver implementation specific callback */
    Flash_ResetFxn resetFxn; /**< Flash driver implementation specific callback */
    Flash_EnableDacModeFxn enableDacModeFxn; /**< Flash driver implementation specific callback */
    Flash_DisableDacModeFxn disableDacModeFxn; /**< Flash driver implementation specific callback */

} Flash_Fxns;

/**
 * \brief Flash device attributes, these are filled by SysCfg based on the flash device that is selected.
 */
typedef struct Flash_Attrs_s {

    uint32_t flashType;      /**< Flash type. Whether it's NAND or NOR */
    char *flashName;         /**< Flash name. Taken from Sysconfig */
    uint32_t deviceId;       /**< Flash device ID as read form the flash device, this will be filled when Flash_open() is called */
    uint32_t manufacturerId; /**< Flash manufacturer ID as read form the flash device, this will be filled when Flash_open() is called */
    uint32_t driverInstance; /**< Underlying SPI peripheral driver instance that is used by the flash driver, e.g OSPI driver */
    uint32_t flashSize;      /**< Flash size, in bytes */
    uint32_t blockCount;     /**< Number of blocks in the flash the flash */
    uint32_t blockSize;      /**< Size of each block, in bytes */
    uint32_t pageCount;      /**< Number of pages per block */
    uint32_t pageSize;       /**< Size of each page, in bytes */
    uint32_t sectorCount;    /**< Number of sectors in the flash, if flash supports sectors */
    uint32_t sectorSize;     /**< Size of each flash sector, in bytes */
    uint32_t spareAreaSize;  /**< Size of spare area in flash*/
    uint32_t phyTuningOffset;/**< Flash offset at which phy tuning vector will be written*/
} Flash_Attrs;

/**
 * \brief Flash driver configuration, these are filled by SysCfg based on the flash device that is selected.
 */
typedef struct Flash_Config_s
{
    Flash_Attrs                *attrs;       /**< Flash device attributes */
    Flash_Fxns                 *fxns;        /**< Flash device implementation functions */
    Flash_DevConfig            *devConfig;  /**< Flash device specific config, like command ID for read, erase, etc */
    void                       *object;      /**< Flash driver object, used to maintain driver implementation state */
    uint32_t                   skipHwInit;  /**< Option to skip the HW initialization of the flash */
    uint32_t                   rwOffset;    /**< Global read write offset*/
} Flash_Config;

/* Flash specific includes */
#if defined (DRV_VERSION_FLASH_V0)
#include <board/flash/ospi/flash_nor_ospi.h>
#endif

#if defined (DRV_VERSION_FLASH_V1)
#include <board/flash/qspi/flash_nor_qspi.h>
#endif

#if defined (DRV_VERSION_FLASH_V2)
#include <board/flash/ospi/flash_nor_ospi.h>
#include <board/flash/ospi/flash_nand_ospi.h>
#endif

#if defined (DRV_VERSION_GPMC_V0)
#include <board/flash/gpmc/flash_nand_gpmc.h>
#endif

/**
 * \brief Set default parameters in the \ref Flash_Params structure
 *
 * Call this API to set defaults and then override the fields as needed before calling  \ref Flash_open.
 *
 * \param params    [out] Initialized parameters
 */
void Flash_Params_init(Flash_Params *params);

/**
 * \brief Open flash driver
 *
 * Make sure the SOC peripheral driver is opened before calling this API.
 * Drivers_open function generated by SysCfg opens the underlying SOC peripheral driver, e.g OSPI.
 *
 * Internally this API also reads the device and manufacture ID and checks if it matches the
 * expected value for the flash device, if there is mismatch then `NULL` is returned.
 *
 * Global variables `Flash_Config gFlashConfig[]` and `uint32_t gFlashConfigNum` is instantiated by SysCfg
 * to describe the flash configuration based on user selection in SysCfg.
 *
 * \param instanceId [in] Index within `Flash_Config gFlashConfig[]` denoting the flash driver to open
 * \param params    [in] Open parameters
 *
 * \return Handle to flash driver which should be used in subsequent API call
 * \return NULL in case of failure
 */
Flash_Handle Flash_open(uint32_t instanceId, Flash_Params *params);

/**
 * \brief Close flash driver
 *
 * \param handle    [in] Flash driver handle from \ref Flash_open
 */
void Flash_close(Flash_Handle handle);

/**
 * \brief Get handle to flash driver
 *
 * \param instanceId    [in] Index within `Flash_Config gFlashConfig[]`
 *
 * \return Handle to flash driver
 * \return NULL in case of failure
 */
Flash_Handle Flash_getHandle(uint32_t instanceId);

/**
 * \brief Read data from flash
 *
 * Internally it will use DMA and do the needed cache sync operations as needed.
 *
 * \param handle [in] Flash driver handle from \ref Flash_open
 * \param offset [in] Offset in the flash from where to start the read
 * \param buf   [in] Buffer into which to read the data into
 * \param len [in] Length of the data to read, in bytes
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t Flash_read(Flash_Handle handle, uint32_t offset, uint8_t *buf, uint32_t len);

/**
 * \brief Write to flash
 *
 * Make sure the block is erased before writing
 *
 * \param handle [in] Flash driver handle from \ref Flash_open
 * \param offset [in] Offset in the flash from where to start the write.
 * \param buf   [in] Buffer which has the data to write.
 * \param len [in] Length of the data to write, in bytes
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t Flash_write(Flash_Handle handle, uint32_t offset, uint8_t *buf, uint32_t len);

/**
 * \brief Utility API to convert (Block Num, Page Num) to offset in bytes
 *
 * \param handle [in] Flash driver handle from \ref Flash_open
 * \param offset [out] Offset in the flash, in bytes.
 * \param block [in] Block number to convert
 * \param page [in] Page number within the block
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t Flash_blkPageToOffset(Flash_Handle handle, uint32_t *offset, uint32_t block, uint32_t page);

/**
 * \brief Utility API to convert offset in bytes to (Block Num, Page Num)
 *
 * \param handle [in] Flash driver handle from \ref Flash_open
 * \param offset [in] Offset in the flash, in bytes. MUST be page size aligned.
 * \param block [out] Converted Block number
 * \param page [out] Converted Page number within the block
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t Flash_offsetToBlkPage(Flash_Handle handle, uint32_t  offset, uint32_t *block, uint32_t *page);

/**
 * \brief Utility API to convert (Sector Num, Page Num) to offset in bytes
 *
 * \param handle [in] Flash driver handle from \ref Flash_open
 * \param offset [out] Offset in the flash, in bytes.
 * \param sector [in] Sector number to convert
 * \param page [in] Page number within the block
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t Flash_SectorPageToOffset(Flash_Handle handle, uint32_t *offset, uint32_t sector, uint32_t page);

/**
 * \brief Utility API to convert offset in bytes to (Sector Num, Page Num)
 *
 * \param handle [in] Flash driver handle from \ref Flash_open
 * \param offset [in] Offset in the flash, in bytes. MUST be page size aligned.
 * \param sector [out] Converted sector number
 * \param page [out] Converted Page number within the block
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t Flash_offsetToSectorPage(Flash_Handle handle, uint32_t  offset, uint32_t *sector, uint32_t *page);

/**
 * \brief Erase a block from flash
 *
 * Use the utility API \ref Flash_offsetToBlkPage to convert a offset to block number
 *
 * \param handle [in] Flash driver handle from \ref Flash_open
 * \param blockNum [in] Block number to erase.
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t Flash_eraseBlk(Flash_Handle handle, uint32_t blockNum);

/**
 * \brief Erase a sector from flash
 *
 * Use the utility API \ref Flash_offsetToSectorPage to convert a offset to block number
 *
 * \param handle [in] Flash driver handle from \ref Flash_open
 * \param sectorNum [in] Sector number to erase.
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t Flash_eraseSector(Flash_Handle handle, uint32_t sectorNum);

/**
 * \brief Do a soft reset of the flash
 *
 * \param handle [in] Flash driver handle from \ref Flash_open
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t Flash_reset(Flash_Handle handle);

/**
 * \brief Enables DAC mode in flash
 *
 * \param handle [in] Flash driver handle from \ref Flash_open
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t Flash_enableDacMode(Flash_Handle handle);

/**
 * \brief Disable DAC mode in flash
 *
 * \param handle [in] Flash driver handle from \ref Flash_open
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t Flash_disableDacMode(Flash_Handle handle);

/**
 * \brief Return flash offset to write PHY tuning data
 *
 * \param handle   [in] Flash driver handle from \ref Flash_open
 *
 * \return Tuning offset on SUCCESS, else 0xFFFFFFFF if handle is invalid
 */
uint32_t Flash_getPhyTuningOffset(Flash_Handle handle);

/**
 * \brief Return flash attributes
 *
 * \param instanceId   [in] Flash instance ID
 *
 * \return \ref Flash_Attrs, else NULL if instanceId is invalid
 */
Flash_Attrs *Flash_getAttrs(uint32_t instanceId);

/** @} */


#ifdef __cplusplus
}
#endif

#endif /* FLASH_H_ */