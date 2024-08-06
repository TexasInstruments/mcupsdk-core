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

/**
 *  \defgroup BOARD_IO_EXPANDER_TCA6408_MODULE APIs for TCA6408 IO Expander driver
 *  \ingroup BOARD_MODULE
 *
 *  This module contains APIs to program and use I2C based TCA6408 IO Expander
 *  module on the board.
 *
 *  @{
 */

#ifndef IO_EXP_TCA6408_H_
#define IO_EXP_TCA6408_H_

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
 *  \anchor TCA6408_Mode
 *  \name IO pin mode - Input or Output
 *  @{
 */
/** \brief Configure IO pin as input */
#define TCA6408_MODE_INPUT              (0U)
/** \brief Configure IO pin as output */
#define TCA6408_MODE_OUTPUT             (1U)
/** @} */

/**
 *  \anchor TCA6408_OutState
 *  \name IO pin output state - HIGH or LOW
 *  @{
 */
/** \brief Configure IO pin output as LOW */
#define TCA6408_OUT_STATE_LOW           (0U)
/** \brief Configure IO pin output as HIGH */
#define TCA6408_OUT_STATE_HIGH          (1U)
/** @} */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 *  \brief Parameters passed during TCA6408_open()
 */
typedef struct TCA6408_Params_s
{
    uint32_t        i2cInstance;
    /**< Underlying peripheral driver instance that is used by the
     *   IO Expander driver incase of I2C controlled IO Expander */
    uint32_t        i2cAddress;
    /**< I2C address for IO expander */
} TCA6408_Params;

/**
 *  \brief IO Expander device attributes.
 */
typedef struct TCA6408_Attrs_s
{
    uint32_t        numIo;
    /**< Number of IO supported by device */
} TCA6408_Attrs;

/**
 *  \brief IO Expander driver configuration. This is the driver object used to
 *  store state variables
 */
typedef struct TCA6408_Config_s
{
    TCA6408_Params      params;
    /**< Parameters */
    TCA6408_Attrs       attrs;
    /**< Attributes */
    I2C_Handle          i2cHandle;
    /**< I2C driver handle */
    void               *lock;
    /**< Mutex to protect IO expander access. */
    SemaphoreP_Object   lockObj;
    /**< Mutex object. */
} TCA6408_Config;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 *  \brief Open TCA6408 driver
 *
 *  Make sure the I2C driver is opened before calling this API.
 *
 *  \param config       [IN] Driver object. Caller need to allocate memory for this.
 *  \param params       [IN] Open parameters
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t TCA6408_open(TCA6408_Config *config, const TCA6408_Params *params);

/**
 *  \brief Close TCA6408 driver
 *
 *  \param config    [IN] TCA6408 driver config from \ref TCA6408_open
 */
void TCA6408_close(TCA6408_Config *config);

/**
 * \brief API to set a IO pin of TCA6408 as input or output
 *
 * \param config    [IN] TCA6408 driver config from \ref TCA6408_open
 * \param ioIndex   [IN] Index to the TCA6408 IO which needs to be set/reset.
 * \param mode      [IN] Refer \ref TCA6408_Mode
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t TCA6408_config(TCA6408_Config *config, uint32_t ioIndex, uint32_t mode);

/**
 * \brief API to set a IO pin of TCA6408 to either HIGH or LOW
 *
 * \param config    [IN] TCA6408 driver config from \ref TCA6408_open
 * \param ioIndex   [IN] Index to the TCA6408 IO which needs to be set/reset.
 * \param state     [IN] Refer \ref TCA6408_OutState
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t TCA6408_setOutput(TCA6408_Config *config, uint32_t ioIndex, uint32_t state);

/**
 * \brief Returns TCA6408 attributes
 *
 * \param config    [IN] TCA6408 driver config from \ref TCA6408_open
 * \param attrs     [IN/OUT] Structure where the attribute is returned
 *
 */
void TCA6408_getAttrs(TCA6408_Config *config, TCA6408_Attrs *attrs);

/**
 *  \brief Set default parameters in the \ref TCA6408_Params structure
 *
 *  Call this API to set defaults and then override the fields as needed
 *  before calling  \ref TCA6408_open.
 *
 *  \param params   [OUT] Initialized parameters
 */
void TCA6408_Params_init(TCA6408_Params *params);

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                  Internal/Private Structure Declarations                   */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
}
#endif

#endif /* #ifndef TCA6408_H_ */

/** @} */
