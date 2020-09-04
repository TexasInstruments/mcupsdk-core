/**********************************************************************
 * Copyright (C) 2012-2021 Cadence Design Systems, Inc.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 **********************************************************************
 * WARNING: This file is auto-generated using api-generator utility.
 *          api-generator: 12.02.13bb8d5
 *          Do not edit it manually.
 **********************************************************************
 * Cadence Core Driver for the Cadence memory controller core. This
 * header file provides values and types to be used while interfacing
 * with the memory core.
 **********************************************************************/

#ifndef LPDDR4_32BIT_IF_H
#define LPDDR4_32BIT_IF_H

#ifdef __cplusplus
extern "C"
{
#endif


#include "../common/cdn_stdtypes.h"

/** @defgroup ConfigInfo  Configuration and Hardware Operation Information
 *  The following definitions specify the driver operation environment that
 *  is defined by hardware configuration or client code. These defines are
 *  located in the header file of the core driver.
 *  @{
 */

 /**********************************************************************
 * Defines
 **********************************************************************/
/** Number of chip-selects */
#define	LPDDR4_INTR_MAX_CS (2U)

/** Number of accessible registers for controller. */
#define	LPDDR4_INTR_CTL_REG_COUNT (459U)

/** Number of accessible registers for PHY Independent Module. */
#define	LPDDR4_INTR_PHY_INDEP_REG_COUNT (300U)

/** Number of accessible registers for PHY. */
#define	LPDDR4_INTR_PHY_REG_COUNT (1423U)

/**
 *  @}
 */


/** @defgroup DataStructure Dynamic Data Structures
 *  This section defines the data structures used by the driver to provide
 *  hardware information, modification and dynamic operation of the driver.
 *  These data structures are defined in the header file of the core driver
 *  and utilized by the API.
 *  @{
 */


/**********************************************************************
 * Enumerations
 **********************************************************************/
/** Controller status or error interrupts. */
typedef enum {
    LPDDR4_INTR_RESET_DONE = 0U,
    LPDDR4_INTR_BUS_ACCESS_ERROR = 1U,
    LPDDR4_INTR_MULTIPLE_BUS_ACCESS_ERROR = 2U,
    LPDDR4_INTR_ECC_MULTIPLE_CORR_ERROR = 3U,
    LPDDR4_INTR_ECC_MULTIPLE_UNCORR_ERROR = 4U,
    LPDDR4_INTR_ECC_WRITEBACK_EXEC_ERROR = 5U,
    LPDDR4_INTR_ECC_SCRUB_DONE = 6U,
    LPDDR4_INTR_ECC_SCRUB_ERROR = 7U,
    LPDDR4_INTR_PORT_COMMAND_ERROR = 8U,
    LPDDR4_INTR_MC_INIT_DONE = 9U,
    LPDDR4_INTR_LP_DONE = 10U,
    LPDDR4_INTR_BIST_DONE = 11U,
    LPDDR4_INTR_WRAP_ERROR = 12U,
    LPDDR4_INTR_INVALID_BURST_ERROR = 13U,
    LPDDR4_INTR_RDLVL_ERROR = 14U,
    LPDDR4_INTR_RDLVL_GATE_ERROR = 15U,
    LPDDR4_INTR_WRLVL_ERROR = 16U,
    LPDDR4_INTR_CA_TRAINING_ERROR = 17U,
    LPDDR4_INTR_DFI_UPDATE_ERROR = 18U,
    LPDDR4_INTR_MRR_ERROR = 19U,
    LPDDR4_INTR_PHY_MASTER_ERROR = 20U,
    LPDDR4_INTR_WRLVL_REQ = 21U,
    LPDDR4_INTR_RDLVL_REQ = 22U,
    LPDDR4_INTR_RDLVL_GATE_REQ = 23U,
    LPDDR4_INTR_CA_TRAINING_REQ = 24U,
    LPDDR4_INTR_LEVELING_DONE = 25U,
    LPDDR4_INTR_PHY_ERROR = 26U,
    LPDDR4_INTR_MR_READ_DONE = 27U,
    LPDDR4_INTR_TEMP_CHANGE = 28U,
    LPDDR4_INTR_TEMP_ALERT = 29U,
    LPDDR4_INTR_SW_DQS_COMPLETE = 30U,
    LPDDR4_INTR_DQS_OSC_BV_UPDATED = 31U,
    LPDDR4_INTR_DQS_OSC_OVERFLOW = 32U,
    LPDDR4_INTR_DQS_OSC_VAR_OUT = 33U,
    LPDDR4_INTR_MR_WRITE_DONE = 34U,
    LPDDR4_INTR_INHIBIT_DRAM_DONE = 35U,
    LPDDR4_INTR_DFI_INIT_STATE = 36U,
    LPDDR4_INTR_DLL_RESYNC_DONE = 37U,
    LPDDR4_INTR_TDFI_TO = 38U,
    LPDDR4_INTR_DFS_DONE = 39U,
    LPDDR4_INTR_DFS_STATUS = 40U,
    LPDDR4_INTR_REFRESH_STATUS = 41U,
    LPDDR4_INTR_ZQ_STATUS = 42U,
    LPDDR4_INTR_SW_REQ_MODE = 43U,
    LPDDR4_INTR_LOR_BITS = 44U
} LPDDR4_INTR_CtlInterrupt;

/** PHY Independent Module status or error interrupts. */
typedef enum {
    LPDDR4_INTR_PHY_INDEP_INIT_DONE_BIT = 0U,
    LPDDR4_INTR_PHY_INDEP_CONTROL_ERROR_BIT = 1U,
    LPDDR4_INTR_PHY_INDEP_CA_PARITY_ERR_BIT = 2U,
    LPDDR4_INTR_PHY_INDEP_RDLVL_ERROR_BIT = 3U,
    LPDDR4_INTR_PHY_INDEP_RDLVL_G_ERROR_BIT = 4U,
    LPDDR4_INTR_PHY_INDEP_WRLVL_ERROR_BIT = 5U,
    LPDDR4_INTR_PHY_INDEP_CALVL_ERROR_BIT = 6U,
    LPDDR4_INTR_PHY_INDEP_WDQLVL_ERROR_BIT = 7U,
    LPDDR4_INTR_PHY_INDEP_UPDATE_ERROR_BIT = 8U,
    LPDDR4_INTR_PHY_INDEP_RDLVL_REQ_BIT = 9U,
    LPDDR4_INTR_PHY_INDEP_RDLVL_GATE_REQ_BIT = 10U,
    LPDDR4_INTR_PHY_INDEP_WRLVL_REQ_BIT = 11U,
    LPDDR4_INTR_PHY_INDEP_CALVL_REQ_BIT = 12U,
    LPDDR4_INTR_PHY_INDEP_WDQLVL_REQ_BIT = 13U,
    LPDDR4_INTR_PHY_INDEP_LVL_DONE_BIT = 14U,
    LPDDR4_INTR_PHY_INDEP_BIST_DONE_BIT = 15U,
    LPDDR4_INTR_PHY_INDEP_TDFI_INIT_TIME_OUT_BIT = 16U,
    LPDDR4_INTR_PHY_INDEP_DLL_LOCK_STATE_CHANGE_BIT = 17U
} LPDDR4_INTR_PhyIndepInterrupt;

/**
 *  @}
 */



#ifdef __cplusplus
}
#endif

#endif	/* LPDDR4_32BIT_IF_H */
