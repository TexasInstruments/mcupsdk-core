/*
 *  Copyright (C) 2021-2023 Texas Instruments Incorporated
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

#ifndef PSRAM_H_
#define PSRAM_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <kernel/dpl/SystemP.h>
#include <drivers/hw_include/soc_config.h>

/**
 *  \defgroup BOARD_PSRAM_MODULE APIs for PSRAM
 *  \ingroup BOARD_MODULE
 *
 *  This module contains APIs to program and use the Psram module on the board like XSPI NOR Psram.
 *  See \ref BOARD_PSRAM_PAGE for more details.
 *
 *  @{
 */
/**
 * \brief Handle to the PSRAM driver returned by Psram_opem()
 */
typedef void  *Psram_Handle;

/**
 * \brief Forward declaration of \ref Psram_Config
 */
typedef struct Psram_Config_s Psram_Config;

/**
 * \brief Forward declaration of \ref Psram_Params
 */
typedef struct Psram_Params_s Psram_Params;

typedef struct Psram_Attrs_s {

    uint32_t psramType;      /**< Psram type. Whether it's NAND or NOR */
    char *psramName;         /**< Psram name. Taken from Sysconfig */
    uint32_t deviceId;       /**< Psram device ID as read form the Psram device, this will be filled when Psram_open() is called */
    uint32_t manufacturerId; /**< Psram manufacturer ID as read form the Psram device, this will be filled when Psram_open() is called */
    uint32_t driverInstance; /**< Underlying SPI peripheral driver instance that is used by the Psram driver, e.g OSPI driver */
    uint32_t psramSize;      /**< Psram size, in bytes */

} Psram_Attrs;

typedef int32_t (*Psram_OpenFxn)(Psram_Config *config);

typedef void (*Psram_CloseFxn)(Psram_Config *config);
typedef int32_t (*Psram_ReadFxn)(Psram_Config *config, uint32_t offset,
                                   uint8_t *buf, uint32_t len) ;
typedef int32_t (*Psram_WriteFxn)(Psram_Config *config, uint32_t offset,
                                   uint8_t *buf, uint32_t len) ;

/**
 * \brief Driver implementation callbacks
 */
typedef struct Psram_Fxns_s
{
    Psram_OpenFxn  openFxn;  /**< Flash driver implementation specific callback */
    Psram_CloseFxn closeFxn; /**< Flash driver implementation specific callback */
    Psram_ReadFxn  readFxn;  /**< Flash driver implementation specific callback */
    Psram_WriteFxn writeFxn; /**< Flash driver implementation specific callback */
} Psram_Fxns;

/**
 * \brief Psram driver configuration, these are filled by SysCfg based on the psram device that is selected.
 */
typedef struct Psram_Config_s
{
    Psram_Attrs                *attrs;       /**< Psram device attributes */
    Psram_Fxns                 *fxns;        /**< Psram device implementation functions */
    void                       *object;      /**< Psram driver object, used to maintain driver implementation state */

} Psram_Config;

/**
 * \brief Psram device config. This will be part of the psram config, so has to
 *        be filled by sysconfig or otherwise before invoking Psram_open
 */

/**
 *
 * \brief User implementation of a custom function to handle vendor specific quirks
 *
 * Typically this callback is hidden from the end application and is implemented
 * when a new type of flash device needs to be implemented.
 *
 * \param config [in] Flash configuration for the specific flash device
 *
 * \return SystemP_SUCCESS on success, else failure
 */
typedef int32_t (*Psram_quirksFxn)(Psram_Config *config);

/**
 * \brief User implementation of a custom function to configure flash to operate in a specific protocol
 *
 * \param config [in] Flash configuration for the specific flash device
 *
 * \return SystemP_SUCCESS on success, else failure
 */
typedef int32_t (*Psram_custProtocolFxn)(Psram_Config *config);

/** @} */


/**
 * \brief Parameters passed during Flash_open()
 */
typedef struct Psram_Params_s {

    Psram_quirksFxn quirksFxn;
    Psram_custProtocolFxn custProtoFxn;

} Psram_Params;

#if defined (DRV_VERSION_PSRAM_V0)
#include <board/psram/gpmc/psram_gpmc.h>
#endif

Psram_Attrs *Psram_getAttrs(uint32_t instanceId);
Psram_Handle Psram_open(uint32_t instanceId, Psram_Params *params);
void Psram_close(Psram_Handle handle);
int32_t Psram_write(Psram_Handle handle, uint32_t offset, uint8_t *buf, uint32_t len);
int32_t Psram_read(Psram_Handle handle, uint32_t offset, uint8_t *buf, uint32_t len);

/** @} */


#ifdef __cplusplus
}
#endif

#endif /* PSRAM_H_ */