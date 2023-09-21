/*
 *  Copyright (C) 2022-23 Texas Instruments Incorporated
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
 *  \defgroup BOARD_PMIC_MODULE APIs for MCSPI based PMIC driver
 *  \ingroup BOARD_MODULE
 *
 *  This module contains APIs to program and use MCSPI based PMIC module
 *  on the board.
 *
 *  See \ref BOARD_PMIC_PAGE for more details.
 *
 *  @{
 */

#ifndef PMIC_H_
#define PMIC_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <kernel/dpl/SystemP.h>
#include <drivers/mcspi.h>
#include <drivers/edma.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/** \brief Handle to the PMIC driver returned by #PMIC_open() */
typedef void *PMIC_Handle;
/** \brief Forward declaration of \ref PMIC_Config_s */
typedef struct PMIC_Config_s PMIC_Config;
/** \brief Forward declaration of \ref PMIC_Params_s */
typedef struct PMIC_Params_s PMIC_Params;

/**
 * \name PMIC driver implementation callbacks
 *
 * @{
 */
/**
 *  \brief Driver implementation to open a specific PMIC driver
 *
 *  Typically this callback is hidden from the end application and is implemented
 *  when a new type of PMIC device needs to be implemented.
 *
 *  \param config [IN] PMIC configuration for the specific PMIC device
 *  \param params [IN] User controllable parameters when opening the PMIC device
 *
 *  \return SystemP_SUCCESS on success, else failure
 */
typedef int32_t (*PMIC_OpenFxn)(PMIC_Config *config, const PMIC_Params *params);

/**
 *  \brief Driver implementation to configure the specific PMIC driver
 *
 *  Typically this callback is hidden from the end application and is implemented
 *  when a new type of PMIC device needs to be implemented.
 *
 *  \param config [IN] PMIC configuration for the specific PMIC device
 *
 *  \return SystemP_SUCCESS on success, else failure
 */
typedef int32_t (*PMIC_ConfigureFxn)(PMIC_Config *config);

/**
 *  \brief Driver implementation to close a specific PMIC driver
 *
 *  Typically this callback is hidden from the end application and is implemented
 *  when a new type of PMIC device needs to be implemented.
 *
 *  \param config [IN] PMIC configuration for the specific PMIC device
 *
 *  \return SystemP_SUCCESS on success, else failure
 */
typedef void (*PMIC_CloseFxn)(PMIC_Config *config);

/** @} */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 *  \brief Parameters passed during PMIC_open()
 */
struct PMIC_Params_s
{
    uint32_t        mcspiInstance;
    /**< Underlying peripheral driver instance that is used by the
     *   PMIC driver incase of MCSPI controlled PMIC */
};

/**
 *  \brief Driver implementation callbacks
 */
typedef struct PMIC_Fxns_s
{
    PMIC_OpenFxn     openFxn;
    /**< PMIC driver implementation specific callback */
    PMIC_ConfigureFxn configureFxn;
    /**< PMIC driver implementation specific callback */
    PMIC_CloseFxn    closeFxn;
    /**< PMIC driver implementation specific callback */
} PMIC_Fxns;


/**
 *  \brief PMIC driver configuration, these are filled by SysCfg based
 *  on the device that is selected.
 */
struct PMIC_Config_s
{
    PMIC_Fxns       *fxns;
    /**< PMIC device implementation functions */
    void           *object;
    /**< PMIC driver object, used to maintain driver implementation state */
};

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 *  \brief Set default parameters in the \ref PMIC_Params_s structure
 *
 *  Call this API to set defaults and then override the fields as needed
 *  before calling  \ref PMIC_open.
 *
 *  \param params   [OUT] Initialized parameters
 */
void PMIC_Params_init(PMIC_Params *params);

/**
 *  \brief Open PMIC driver
 *
 *  Make sure the SOC peripheral driver is open'ed before calling this API.
 *  Drivers_open function generated by SysCfg opens the underlying SOC
 *  peripheral driver, e.g MCSPI
 *
 *  Global variables `PMIC_Config gPmicConfig[]` and
 *  `uint32_t gPmicConfigNum` is instantiated by SysCfg
 *  to describe the PMIC configuration based on user selection in SysCfg.
 *
 *  \param instanceId   [IN] Index within `PMIC_Config gPmicConfig[]`
 *                      denoting the PMIC driver to open
 *  \param params       [IN] Open parameters
 *
 *  \return Handle to PMIC driver which should be used in subsequent API call
 *          Else returns NULL in case of failure
 */
PMIC_Handle PMIC_open(uint32_t instanceId, const PMIC_Params *params);

/**
 *  \brief Configure PMIC driver
 *
 *  \param handle    [IN] PMIC driver handle from \ref PMIC_open
 */
int32_t PMIC_configure(PMIC_Handle handle);

/**
 *  \brief Close PMIC driver
 *
 *  \param handle    [IN] PMIC driver handle from \ref PMIC_open
 */
void PMIC_close(PMIC_Handle handle);


/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                  Internal/Private Structure Declarations                   */
/* ========================================================================== */

/**
 *  \brief PMIC driver object - not to be used by application
 */
typedef struct
{
    MCSPI_Handle      mcspiHandle;
    /**< MCSPI driver handle */
    uint32_t        mcspiInstance;
    /**< Underlying MCSPI driver instance that is used by the
     *   PMIC driver incase of MCSPI controlled PMIC */
} PMIC_Object;

#ifdef __cplusplus
}
#endif

#endif /* #ifndef PMIC_H_ */

/** @} */
