/******************************************************************************
 * Copyright (c) 2024 Texas Instruments Incorporated - http://www.ti.com
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
 *
 *****************************************************************************/

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <board/pmic.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

extern PMIC_Config gPmicConfig[];
extern uint32_t gPmicConfigNum;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/**
 * @brief Open a PMIC instance.
 *
 * This function opens a PMIC instance specified by the instance ID and returns
 * a handle to it.
 *
 * @param instanceId The instance ID of the PMIC to open.
 * @param params Pointer to the PMIC parameters structure. Can be NULL if not
 * required.
 *
 * @return PMIC_Handle Returns a handle to the opened PMIC instance if
 * successful; otherwise, returns NULL.
 */
PMIC_Handle PMIC_open(uint32_t instanceId, const PMIC_Params *params) {
    PMIC_Config *config = NULL;

    if (instanceId < gPmicConfigNum) {
        config = &gPmicConfig[instanceId];
        if (config->fxns && config->fxns->openFxn) {
            int32_t status;

            status = config->fxns->openFxn(config, params);
            if (status != SystemP_SUCCESS) {
                config = NULL;
            }
        }
    }

    return (config);
}

/**
 * @brief Configure a PMIC instance.
 *
 * This function configures a PMIC instance using the provided handle.
 *
 * @param handle Handle to the PMIC instance to configure.
 *
 * @return int32_t Returns SystemP_SUCCESS if the configuration is successful;
 * otherwise, returns SystemP_FAILURE.
 */
int32_t PMIC_configure(PMIC_Handle handle) {
    int32_t status = SystemP_FAILURE;

    PMIC_Config *config = (PMIC_Config *)handle;

    if (config && config->fxns && config->fxns->configureFxn) {
        config->fxns->configureFxn(config);
    }

    return status;
}

/**
 * @brief Close a PMIC instance.
 *
 * This function closes a PMIC instance specified by the handle.
 *
 * @param handle Handle to the PMIC instance to close.
 */
void PMIC_close(PMIC_Handle handle) {
    PMIC_Config *config = (PMIC_Config *)handle;

    if (config && config->fxns && config->fxns->closeFxn) {
        config->fxns->closeFxn(config);
    }

    return;
}
