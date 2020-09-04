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

/**
 *  \defgroup BOARD_EEPROM_MODULE APIs for I2C based EEPROM
 *  \ingroup BOARD_MODULE
 *
 *  This module contains APIs to program and use the I2C based EEPROM module
 *  on the board like AT24CM01.
 *  See \ref BOARD_EEPROM_PAGE for more details.
 *
 *  @{
 */

#ifndef EEPROM_H_
#define EEPROM_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <kernel/dpl/SystemP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <drivers/i2c.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/**
 *  \brief Max EEPROM page size used to allocate temp write buffer so that
 *  address (offset) and data can be done in single I2C operation
 */
#define EEPROM_PAGE_SIZE                (256U)
/**
 *  \brief Temp write buffer to hold address offset and data for page write
 *  operation - 2 bytes for offset and remaining for one page data
 */
#define EEPROM_WR_BUF_SIZE              (2U + EEPROM_PAGE_SIZE)

/** \brief Handle to the EEPROM driver returned by #EEPROM_open() */
typedef void *EEPROM_Handle;
/** \brief Forward declaration of \ref EEPROM_Config_s */
typedef struct EEPROM_Config_s EEPROM_Config;
/** \brief Forward declaration of \ref EEPROM_Params_s */
typedef struct EEPROM_Params_s EEPROM_Params;

/**
 * \name EEPROM driver implementation callbacks
 *
 * @{
 */
/**
 *  \brief Driver implementation to open a specific EEPROM driver
 *
 *  Typically this callback is hidden from the end application and is implemented
 *  when a new type of EEPROM device needs to be implemented.
 *
 *  \param config [IN] EEPROM configuration for the specific EEPROM device
 *  \param params [IN] User controllable parameters when opening the EEPROM device
 *
 *  \return SystemP_SUCCESS on success, else failure
 */
typedef int32_t (*EEPROM_OpenFxn)(EEPROM_Config *config,
                                  const EEPROM_Params *params);

/**
 *  \brief Driver implementation to close a specific EEPROM driver
 *
 *  Typically this callback is hidden from the end application and is implemented
 *  when a new type of EEPROM device needs to be implemented.
 *
 *  \param config [IN] EEPROM configuration for the specific EEPROM device
 *
 *  \return SystemP_SUCCESS on success, else failure
 */
typedef void (*EEPROM_CloseFxn)(EEPROM_Config *config);

/**
 *  \brief Driver implementation to read from EEPROM using a specific EEPROM driver
 *
 *  Typically this callback is hidden from the end application and is implemented
 *  when a new type of EEPROM device needs to be implemented.
 *
 *  \param config [IN] EEPROM configuration for the specific EEPROM device
 *  \param offset [IN] Offset in the EEPROM from where to start the read
 *  \param buf    [IN] Buffer into which to read the data into
 *  \param len    [IN] Length of the data to read, in bytes
 *
 *  \return SystemP_SUCCESS on success, else failure
 */
typedef int32_t (*EEPROM_ReadFxn)(EEPROM_Config *config,
                                  uint32_t offset,
                                  uint8_t *buf,
                                  uint32_t len);

/**
 *  \brief Driver implementation to write to EEPROM using specific EEPROM driver
 *
 *  Typically this callback is hidden from the end application and is implemented
 *  when a new type of EEPROM device needs to be implemented.
 *
 *  \param config [IN] EEPROM configuration for the specific EEPROM device
 *  \param offset [IN] Offset in the EEPROM from where to start the write.
 *  \param buf    [IN] Buffer which has the data to write.
 *  \param len    [IN] Length of the data to write, in bytes
 *
 *  \return SystemP_SUCCESS on success, else failure
 */
typedef int32_t (*EEPROM_WriteFxn)(EEPROM_Config *config,
                                   uint32_t offset,
                                   const uint8_t *buf,
                                   uint32_t len);
/** @} */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 *  \brief Parameters passed during EEPROM_open()
 */
struct EEPROM_Params_s
{
    uint32_t        driverInstance;
    /**< Underlying peripheral driver instance that is used by the
     *   EEPROM driver */
    uint32_t        i2cAddress;
    /**< EEPROM I2C address */
};

/**
 *  \brief Driver implementation callbacks
 */
typedef struct EEPROM_Fxns_s
{
    EEPROM_OpenFxn  openFxn;
    /**< EEPROM driver implementation specific callback */
    EEPROM_CloseFxn closeFxn;
    /**< EEPROM driver implementation specific callback */
    EEPROM_ReadFxn  readFxn;
    /**< EEPROM driver implementation specific callback */
    EEPROM_WriteFxn writeFxn;
    /**< EEPROM driver implementation specific callback */
} EEPROM_Fxns;

/**
 *  \brief EEPROM device attributes, these are filled by SysCfg based
 *  on the device that is selected.
 */
typedef struct EEPROM_Attrs_s
{
    uint32_t        size;
    /**< Size of EEPROM in bytes */
    uint32_t        pageCount;
    /**< Number of pages */
    uint32_t        pageSize;
    /**< Size of each page, in bytes */
} EEPROM_Attrs;

/**
 *  \brief EEPROM driver configuration, these are filled by SysCfg based
 *  on the device that is selected.
 */
struct EEPROM_Config_s
{
    EEPROM_Attrs   *attrs;
    /**< EEPROM device attributes */
    EEPROM_Fxns    *fxns;
    /**< EEPROM device implementation functions */
    void           *object;
    /**< EEPROM driver object, used to maintain driver implementation state */
};

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 *  \brief Set default parameters in the \ref EEPROM_Params_s structure
 *
 *  Call this API to set defaults and then override the fields as needed
 *  before calling  \ref EEPROM_open.
 *
 *  \param params   [OUT] Initialized parameters
 */
void EEPROM_Params_init(EEPROM_Params *params);

/**
 *  \brief Open EEPROM driver
 *
 *  Make sure the SOC peripheral driver is open'ed before calling this API.
 *  Drivers_open function generated by SysCfg opens the underlying SOC
 *  peripheral driver, e.g I2C.
 *
 *  Global variables `EEPROM_Config gEepromConfig[]` and
 *  `uint32_t gEepromConfigNum` is instantiated by SysCfg
 *  to describe the EEPROM configuration based on user selection in SysCfg.
 *
 *  \param instanceId   [IN] Index within `EEPROM_Config gEepromConfig[]`
 *                      denoting the EEPROM driver to open
 *  \param params       [IN] Open parameters
 *
 *  \return Handle to EEPROM driver which should be used in subsequent API call
 *          Else returns NULL in case of failure
 */
EEPROM_Handle EEPROM_open(uint32_t instanceId, const EEPROM_Params *params);

/**
 *  \brief Open EEPROM driver
 *
 *  \param handle    [IN] EEPROM driver handle from \ref EEPROM_open
 */
void EEPROM_close(EEPROM_Handle handle);

/**
 * \brief Read data from EEPROM
 *
 * \param handle    [IN] EEPROM driver handle from \ref EEPROM_open
 * \param offset    [IN] Offset in the EEPROM from where to start the read
 * \param buf       [IN] Buffer into which to read the data into
 * \param len       [IN] Length of the data to read, in bytes
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t EEPROM_read(EEPROM_Handle handle,
                    uint32_t offset,
                    uint8_t *buf,
                    uint32_t len);

/**
 * \brief Write to EEPROM
 *
 * Make sure the block is erased before writing
 *
 * \param handle    [IN] EEPROM driver handle from \ref EEPROM_open
 * \param offset    [IN] Offset in the EEPROM from where to start the write.
 * \param buf       [IN] Buffer which has the data to write.
 * \param len       [IN] Length of the data to write, in bytes
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t EEPROM_write(EEPROM_Handle handle,
                     uint32_t offset,
                     const uint8_t *buf,
                     uint32_t len);

/**
 * \brief Return EEPROM attributes
 *
 * \param instanceId    [IN] EEPROM instance ID
 *
 * \return \ref EEPROM_Attrs, else NULL if instanceId is invalid
 */
const EEPROM_Attrs *EEPROM_getAttrs(uint32_t instanceId);

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                  Internal/Private Structure Declarations                   */
/* ========================================================================== */

/**
 *  \brief EEPROM driver object - not to be used by application
 */
typedef struct
{
    I2C_Handle          i2cHandle;
    /**< I2C driver handle */
    uint32_t            driverInstance;
    /**< Underlying peripheral driver instance that is used by the
     *   EEPROM driver */
    uint32_t            i2cAddress;
    /**< EEPROM I2C address */
    uint8_t             pageWrBuf[EEPROM_WR_BUF_SIZE];
    /**< EEPROM page write buffer */

    void               *lock;
    /**< Mutex to protect EEPROM access. */
    SemaphoreP_Object   lockObj;
    /**< Mutex object. */
} EEPROM_Object;

#ifdef __cplusplus
}
#endif

#endif /* #ifndef EEPROM_H_ */

/** @} */
