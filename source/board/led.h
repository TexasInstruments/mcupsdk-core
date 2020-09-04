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
 *  \defgroup BOARD_LED_MODULE APIs for I2C/GPIO based LED driver
 *  \ingroup BOARD_MODULE
 *
 *  This module contains APIs to program and use I2C or GPIO based LED module
 *  on the board.
 *
 *  See \ref BOARD_LED_PAGE for more details.
 *
 *  @{
 */

#ifndef LED_H_
#define LED_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <kernel/dpl/SystemP.h>
#include <drivers/i2c.h>
#include <drivers/gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/** \brief Handle to the LED driver returned by #LED_open() */
typedef void *LED_Handle;
/** \brief Forward declaration of \ref LED_Config_s */
typedef struct LED_Config_s LED_Config;
/** \brief Forward declaration of \ref LED_Params_s */
typedef struct LED_Params_s LED_Params;

/**
 * \name LED driver implementation callbacks
 *
 * @{
 */
/**
 *  \brief Driver implementation to open a specific LED driver
 *
 *  Typically this callback is hidden from the end application and is implemented
 *  when a new type of LED device needs to be implemented.
 *
 *  \param config [IN] LED configuration for the specific LED device
 *  \param params [IN] User controllable parameters when opening the LED device
 *
 *  \return SystemP_SUCCESS on success, else failure
 */
typedef int32_t (*LED_OpenFxn)(LED_Config *config, const LED_Params *params);

/**
 *  \brief Driver implementation to close a specific LED driver
 *
 *  Typically this callback is hidden from the end application and is implemented
 *  when a new type of LED device needs to be implemented.
 *
 *  \param config [IN] LED configuration for the specific LED device
 *
 *  \return SystemP_SUCCESS on success, else failure
 */
typedef void (*LED_CloseFxn)(LED_Config *config);

/**
 *  \brief Driver implementation to power on an LED using a specific LED driver
 *
 *  Typically this callback is hidden from the end application and is implemented
 *  when a new type of LED device needs to be implemented.
 *
 *  \param config [IN] LED configuration for the specific LED device
 *  \param index  [IN] Index to the LED group which needs to be turned on
 *
 *  \return SystemP_SUCCESS on success, else failure
 */
typedef int32_t (*LED_OnFxn)(LED_Config *config, uint32_t index);

/**
 *  \brief Driver implementation to power off an LED using a specific LED driver
 *
 *  Typically this callback is hidden from the end application and is implemented
 *  when a new type of LED device needs to be implemented.
 *
 *  \param config [IN] LED configuration for the specific LED device
 *  \param index  [IN] Index to the LED group which needs to be turned off
 *
 *  \return SystemP_SUCCESS on success, else failure
 */
typedef int32_t (*LED_OffFxn)(LED_Config *config, uint32_t index);

/**
 *  \brief Driver implementation to set group mask using a specific LED driver
 *
 *  Typically this callback is hidden from the end application and is implemented
 *  when a new type of LED device needs to be implemented.
 *
 *  \param config [IN] LED configuration for the specific LED device
 *  \param mask   [IN] Bit Mask to set at one go
 *
 *  \return SystemP_SUCCESS on success, else failure
 */
typedef int32_t (*LED_SetMaskFxn)(LED_Config *config, uint32_t mask);
/** @} */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 *  \brief Parameters passed during LED_open()
 */
struct LED_Params_s
{
    uint32_t        gpioBaseAddr;
    /**< Underlying GPIO instance that is used by the LED driver incase of
     *   GPIO controlled LED */
    uint32_t        gpioPinNum;
    /**< GPIO pin to be used */
    uint32_t        i2cInstance;
    /**< Underlying peripheral driver instance that is used by the
     *   LED driver incase of I2C controlled LED */
    uint32_t        i2cAddress;
    /**< I2C address for IO expander or external LED driver */
    uint32_t        ioIndex;
    /**< Index to the IO expander which needs to be set/reset.
     *   Applicable only for IO expander based LED. */
};

/**
 *  \brief Driver implementation callbacks
 */
typedef struct LED_Fxns_s
{
    LED_OpenFxn     openFxn;
    /**< LED driver implementation specific callback */
    LED_CloseFxn    closeFxn;
    /**< LED driver implementation specific callback */
    LED_OnFxn       onFxn;
    /**< LED driver implementation specific callback */
    LED_OffFxn      offFxn;
    /**< LED driver implementation specific callback */
    LED_SetMaskFxn  setMaskFxn;
    /**< LED driver implementation specific callback */
} LED_Fxns;

/**
 *  \brief LED device attributes, these are filled by SysCfg based
 *  on the device that is selected.
 */
typedef struct 
{
    uint32_t        numLedPerGroup;
    /**< Number of LED in a group */
} LED_Attrs;

/**
 *  \brief LED driver configuration, these are filled by SysCfg based
 *  on the device that is selected.
 */
struct LED_Config_s
{
    LED_Attrs      *attrs;
    /**< LED device attributes */
    LED_Fxns       *fxns;
    /**< LED device implementation functions */
    void           *object;
    /**< LED driver object, used to maintain driver implementation state */
};

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 *  \brief Set default parameters in the \ref LED_Params_s structure
 *
 *  Call this API to set defaults and then override the fields as needed
 *  before calling  \ref LED_open.
 *
 *  \param params   [OUT] Initialized parameters
 */
void LED_Params_init(LED_Params *params);

/**
 *  \brief Open LED driver
 *
 *  Make sure the SOC peripheral driver is open'ed before calling this API.
 *  Drivers_open function generated by SysCfg opens the underlying SOC
 *  peripheral driver, e.g I2C.
 *
 *  Global variables `LED_Config gLedConfig[]` and
 *  `uint32_t gLedConfigNum` is instantiated by SysCfg
 *  to describe the LED configuration based on user selection in SysCfg.
 *
 *  \param instanceId   [IN] Index within `LED_Config gLedConfig[]`
 *                      denoting the LED driver to open
 *  \param params       [IN] Open parameters
 *
 *  \return Handle to LED driver which should be used in subsequent API call
 *          Else returns NULL in case of failure
 */
LED_Handle LED_open(uint32_t instanceId, const LED_Params *params);

/**
 *  \brief Open LED driver
 *
 *  \param handle    [IN] LED driver handle from \ref LED_open
 */
void LED_close(LED_Handle handle);

/**
 * \brief API to power on the LED
 *
 * \param handle    [IN] LED driver handle from \ref LED_open
 * \param index     [IN] Index to the LED group which needs to be turned on.
 *                       In case of GPIO controlled LED, set this to 0.
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t LED_on(LED_Handle handle, uint32_t index);

/**
 * \brief API to power off the LED
 *
 * \param handle    [IN] LED driver handle from \ref LED_open
 * \param index     [IN] Index to the LED group which needs to be turned off.
 *                       In case of GPIO controlled LED, set this to 0.
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t LED_off(LED_Handle handle, uint32_t index);

/**
 * \brief API to set the group mask incase of I2C controlled LED having more
 *        than one LED connected to the controller
 *
 * \param handle    [IN] LED driver handle from \ref LED_open
 * \param mask      [IN] Bit Mask to set at one go. Only bits upto the
 *                       number of LED present in a group will be used by this
 *                       function.
 *                       Set bit to 0 for OFF.
 *                       Set bit to 1 for ON.
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t LED_setMask(LED_Handle handle, uint32_t mask);

/**
 * \brief Return LED attributes
 *
 * \param instanceId    [IN] LED instance ID
 *
 * \return \ref LED_Attrs, else NULL if instanceId is invalid
 */
const LED_Attrs *LED_getAttrs(uint32_t instanceId);

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                  Internal/Private Structure Declarations                   */
/* ========================================================================== */

/**
 *  \brief LED driver object - not to be used by application
 */
typedef struct
{
    I2C_Handle      i2cHandle;
    /**< I2C driver handle */
    uint32_t        gpioBaseAddr;
    /**< Underlying GPIO instance that is used by the LED driver incase of
     *   GPIO controlled LED */
    uint32_t        gpioPinNum;
    /**< GPIO pin to be used */
    uint32_t        i2cInstance;
    /**< Underlying I2C driver instance that is used by the
     *   LED driver incase of I2C controlled LED */
    uint32_t        i2cAddress;
    /**< I2C address for IO expander or external LED driver */
    uint32_t        ioIndex;
    /**< Index to the IO expander which needs to be set/reset.
     *   Applicable only for IO expander based LED. */
} LED_Object;

#ifdef __cplusplus
}
#endif

#endif /* #ifndef LED_H_ */

/** @} */
