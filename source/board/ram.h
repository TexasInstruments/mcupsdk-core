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

#ifndef RAM_H_
#define RAM_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <kernel/dpl/SystemP.h>
#include <drivers/hw_include/soc_config.h>

/**
 *  \defgroup BOARD_RAM_MODULE APIs for RAM
 *  \ingroup BOARD_MODULE
 *
 *  This module contains APIs to program and use the Ram module on the board like GPMC pSRAM.
 *  See \ref BOARD_RAM_PAGE for more details.
 *
 *  @{
 */
/**
 * \brief Handle to the RAM driver returned by Ram_open()
 */
typedef void  *Ram_Handle;

/**
 * \brief Forward declaration of \ref Ram_Config
 */
typedef struct Ram_Config_s Ram_Config;

/**
 * \brief Forward declaration of \ref Ram_Params
 */
typedef struct Ram_Params_s Ram_Params;

typedef struct Ram_Attrs_s {

    uint32_t ramType;      /**< Ram type. Whether it's NAND or NOR */
    char *ramName;         /**< Ram name. Taken from Sysconfig */
    uint32_t deviceId;       /**< Ram device ID as read form the Ram device*/
    uint32_t manufacturerId; /**< Ram manufacturer ID as read form the Ram device */
    uint32_t driverInstance; /**< Underlying peripheral driver instance that is used by the Ram driver, e.g GPMC driver */
    uint32_t ramSize;      /**< Ram size, in bytes */

} Ram_Attrs;

/**
 * \name RAM driver implementation callbacks
 *
 * @{
 */

/**
 * \brief Driver implementation to open a specific RAM driver
 *
 * Typically this callback is hidden from the end application and is implemented
 * when a new type of RAM device needs to be implemented.
 *
 * \param config [in] RAM configuration for the specific RAM device
 * \param params [in] User controllable parameters when opening the RAM device
 *
 * \return SystemP_SUCCESS on success, else failure
 */
typedef int32_t (*Ram_OpenFxn)(Ram_Config *config);

/**
 * \brief Driver implementation to close a specific RAM driver
 *
 * Typically this callback is hidden from the end application and is implemented
 * when a new type of RAM device needs to be implemented.
 *
 * \param config [in] RAM configuration for the specific RAM device
 *
 * \return SystemP_SUCCESS on success, else failure
 */
typedef void (*Ram_CloseFxn)(Ram_Config *config);

/**
 * \brief Driver implementation to read from RAM using a specific RAM driver
 *
 * Typically this callback is hidden from the end application and is implemented
 * when a new type of RAM device needs to be implemented.
 *
 * \param config [in] RAM configuration for the specific RAM device
 * \param offset [in] Offset in the RAM from where to start the read
 * \param buf   [in] Buffer into which to read the data into
 * \param len [in] Length of the data to read, in bytes
 *
 * \return SystemP_SUCCESS on success, else failure
 */
typedef int32_t (*Ram_ReadFxn)(Ram_Config *config, uint32_t offset,
                                   uint8_t *buf, uint32_t len) ;

/**
 * \brief Driver implementation to write to RAM using specific RAM driver
 *
 * Typically this callback is hidden from the end application and is implemented
 * when a new type of RAM device needs to be implemented.
 *
 * \param config [in] RAM configuration for the specific RAM device
 * \param offset [in] Offset in the RAM from where to start the write.
 * \param buf   [in] Buffer which has the data to write.
 * \param len [in] Length of the data to write, in bytes
 *
 * \return SystemP_SUCCESS on success, else failure
 */
typedef int32_t (*Ram_WriteFxn)(Ram_Config *config, uint32_t offset,
                                   uint8_t *buf, uint32_t len) ;

/**
 * \brief Driver implementation callbacks
 */
typedef struct Ram_Fxns_s
{
    Ram_OpenFxn  openFxn;  /**< RAM driver implementation specific callback */
    Ram_CloseFxn closeFxn; /**< RAM driver implementation specific callback */
    Ram_ReadFxn  readFxn;  /**< RAM driver implementation specific callback */
    Ram_WriteFxn writeFxn; /**< RAM driver implementation specific callback */
} Ram_Fxns;

/**
 * \brief Ram driver configuration, these are filled by SysCfg based on the ram device that is selected.
 */
typedef struct Ram_Config_s
{
    Ram_Attrs                *attrs;       /**< Ram device attributes */
    Ram_Fxns                 *fxns;        /**< Ram device implementation functions */
    void                       *object;      /**< Ram driver object, used to maintain driver implementation state */

} Ram_Config;

/**
 * \brief Ram device config. This will be part of the ram config, so has to
 *        be filled by sysconfig or otherwise before invoking Ram_open
 */

/**
 *
 * \brief User implementation of a custom function to handle vendor specific quirks
 *
 * Typically this callback is hidden from the end application and is implemented
 * when a new type of RAM device needs to be implemented.
 *
 * \param config [in] RAM configuration for the specific RAM device
 *
 * \return SystemP_SUCCESS on success, else failure
 */
typedef int32_t (*Ram_quirksFxn)(Ram_Config *config);

/**
 * \brief User implementation of a custom function to configure RAM to operate in a specific protocol
 *
 * \param config [in] RAM configuration for the specific RAM device
 *
 * \return SystemP_SUCCESS on success, else failure
 */
typedef int32_t (*Ram_custProtocolFxn)(Ram_Config *config);

/** @} */


/**
 * \brief Parameters passed during RAM_open()
 */
typedef struct Ram_Params_s {

    Ram_quirksFxn quirksFxn;
    Ram_custProtocolFxn custProtoFxn;

} Ram_Params;

#if defined (DRV_VERSION_PSRAM_V0)
#include <board/ram/gpmc/psram_gpmc.h>
#endif

Ram_Attrs *Ram_getAttrs(uint32_t instanceId);

/**
 * \brief Open RAM driver
 *
 * Make sure the SOC peripheral driver is opened before calling this API.
 * Drivers_open function generated by SysCfg opens the underlying SOC peripheral driver, e.g OSPI.
 *
 * Global variables `RAM_Config gRAMConfig[]` and `uint32_t gRAMConfigNum` is instantiated by SysCfg
 * to describe the RAM configuration based on user selection in SysCfg.
 *
 * \param instanceId [in] Index within `RAM_Config gRAMConfig[]` denoting the RAM driver to open
 * \param params    [in] Open parameters
 *
 * \return Handle to RAM driver which should be used in subsequent API call
 * \return NULL in case of failure
 */
Ram_Handle Ram_open(uint32_t instanceId, Ram_Params *params);

/**
 * \brief Close RAM driver
 *
 * \param handle    [in] RAM driver handle from \ref Ram_open
 */
void Ram_close(Ram_Handle handle);

/**
 * \brief Write to RAM device
 *
 * \param handle [in] RAM driver handle from \ref Ram_open
 * \param offset [in] Offset in the RAM from where to start the write.
 * \param buf   [in] Buffer which has the data to write.
 * \param len [in] Length of the data to write, in bytes
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t Ram_write(Ram_Handle handle, uint32_t offset, uint8_t *buf, uint32_t len);

/**
 * \brief Read data from RAM device
 *
 * \param handle [in] RAM driver handle from \ref Ram_open
 * \param offset [in] Offset in the RAM from where to start the read
 * \param buf   [in] Buffer into which to read the data into
 * \param len [in] Length of the data to read, in bytes
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t Ram_read(Ram_Handle handle, uint32_t offset, uint8_t *buf, uint32_t len);

/** @} */


#ifdef __cplusplus
}
#endif

#endif /* RAM_H_ */