/*
 *  Copyright (c) Texas Instruments Incorporated 2020
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

/*!
 * \file     enet_apputils_k3.h
 *
 * \brief    This file contains the function prototypes of the K3 utils functions.
 *
 * NOTE: This library is meant only for Enet examples. Customers are not
 * encouraged to use this layer as these are very specific to the examples
 * written and the API behaviour and signature can change at any time to
 * suit the examples.
 */

#ifndef ENET_APPUTILS_K3_H_
#define ENET_APPUTILS_K3_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <enet.h>
#include <include/core/enet_rm.h>
#include <include/mod/cpsw_stats.h>
#include <include/mod/cpsw_macport.h>

#include <include/core/enet_dma.h>

#include "enet_ethutils.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 *  \brief  Clkout frequency select enum
 */
typedef enum
{
    ENETAPPUTILS_CLKOUT_FREQ_50MHZ = 0U,
    /**< Select 50MHz output on clkout pin */
    ENETAPPUTILS_CLKOUT_FREQ_25MHZ,
    /**< Select 25MHz output on clkout pin */
} EnetAppUtils_ClkOutFreqType;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 * \brief Utility function to enable/disable CPSW module clocks via SCI_CLIENT
 *
 *  \param  moduleId Module for which the state should be set.
 *          Refer \ref Sciclient_PmDeviceIds.
 *  \param  requiredState - on/off state flag
 *  \param  appFlags - additional flag that can be set to alter the device state
 */
void EnetAppUtils_setDeviceState(uint32_t moduleId,
                                 uint32_t requiredState,
                                 uint32_t appFlags);

/**
 *  \brief Utility function to disable CPSW module clocks via SCI_CLIENT
 */
void EnetAppUtils_clkRateSetState(uint32_t moduleId,
                                     uint32_t clockId,
                                     uint32_t additionalFlag,
                                     uint32_t state);

/**
 *  \brief Utility function to disable CPSW module clocks via SCI_CLIENT
 */
void EnetAppUtils_clkRateSet(uint32_t moduleId,
                             uint32_t clkId,
                             uint64_t clkRateHz);

/**
 *  \brief Configure CLK_OUT for CPSW2G and 9G.
 */
void EnetAppUtils_enableClkOut(Enet_Type enetType,
                               EnetAppUtils_ClkOutFreqType clkOut);

int32_t EnetAppUtils_setTimeSyncRouter(Enet_Type enetType,
                                       uint32_t input,
                                       uint32_t output);
/*!
 * \brief Sets up and readies SCI client RM/PM server.
 */
void EnetAppUtils_setupSciServer(void);

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */


#ifdef __cplusplus
}
#endif

#endif  /* ENET_APPUTILS_K3_H_ */
