/*
 *  Copyright (C) 2017-2018 Texas Instruments Incorporated
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
 */

/**
 * \file sciclient_boardcfg.h
 *
 * \brief Wrapper function to send the board configuration message to DMSC.
 */
#ifndef SCICLIENT_BOARDCFG_H_
#define SCICLIENT_BOARDCFG_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */
/**
 *  \brief Parameters for #Sciclient_boardCfg
 *         Pointer to this is passed to #Sciclient_boardCfg.
 */
typedef struct
{
    uint32_t boardConfigLow;
    /**< Low 32-bits of physical pointer to \#tisci_boardConfig /
    \#tisci_boardcfg_pm / \#local_rm_boardcfg / \#tisci_boardcfg_sec .
     struct.
     */
    uint32_t boardConfigHigh;
    /**< High 32-bits of physical pointer to \#tisci_boardConfig /
    \#tisci_boardcfg_pm / \#local_rm_boardcfg / \#tisci_boardcfg_sec .
     struct.
     */
    uint16_t boardConfigSize;
    /**< (uint16_t) sizeof(<board configuration structure>) */
    uint8_t  devGrp;
    /**< SoC defined SYSFW devgrp. Example: #DEVGRP_ALL
     */
} Sciclient_BoardCfgPrms_t;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
/**
 *  \brief  One time board configuration to be done from R5.
 *
 *          User needs to define a section ".boardcfg_data" in the
 *          linker file for the default configuration, which needs to be present
 *          in OCMRAM . If user provides custom board_cfg, then the data must
 *          be present on OCMRAM.
 *
 *  \param pInPrms        [IN]  Pointer to #Sciclient_BoardCfgPrms_t . NULL
 *                          results in default config.
 *
 *  \return SystemP_SUCCESS on success, else failure .
 *
 *
 */
int32_t Sciclient_boardCfg(const Sciclient_BoardCfgPrms_t * pInPrms);

/**
 *  \brief  One time board configuration to be done from R5 for PM.
 *          User needs to define a section ".boardcfg_data" in the
 *          linker file for the default configuration, which needs to be present
 *          in OCMRAM . If user provides custom board_cfg, then the data must
 *          be present on OCMRAM.
 *
 *  \param pInPrms        [IN]  Pointer to \#Sciclient_BoardCfgPrms_t . NULL
 *                          results in default config.
 *
 *  \return SystemP_SUCCESS on success, else failure .
 *
 *
 */
int32_t Sciclient_boardCfgPm(const Sciclient_BoardCfgPrms_t * pInPrms);

/**
 *  \brief  One time board configuration to be done from R5 for RM .
 *          User needs to define a section ".boardcfg_data" in the
 *          linker file for the default configuration, which needs to be present
 *          in OCMRAM . If user provides custom board_cfg, then the data must
 *          be present on OCMRAM.
 *
 *  \param pInPrms        [IN]  Pointer to \#Sciclient_BoardCfgPrms_t . NULL
 *                          results in default config.
 *
 *  \return SystemP_SUCCESS on success, else failure .
 *
 *
 */
int32_t Sciclient_boardCfgRm(const Sciclient_BoardCfgPrms_t * pInPrms);

/**
 *  \brief  One time board configuration to be done from R5 for SECURITY
 *          User needs to define a section ".boardcfg_data" in the
 *          linker file for the default configuration, which needs to be present
 *          in OCMRAM . If user provides custom board_cfg, then the data must
 *          be present on OCMRAM.
 *
 *  \param pInPrms        [IN]  Pointer to \#Sciclient_BoardCfgPrms_t . NULL
 *                          results in default config.
 *
 *  \return SystemP_SUCCESS on success, else failure .
 *
 *
 */
int32_t Sciclient_boardCfgSec(const Sciclient_BoardCfgPrms_t * pInPrms);

#ifdef __cplusplus
}
#endif

#endif /* SCICLIENT_BOARDCFG_H_ */
