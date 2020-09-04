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

#ifndef LPDDR4_16BIT_IF_H
#define LPDDR4_16BIT_IF_H

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
#define	LPDDR4_INTR_CTL_REG_COUNT (423U)

/** Number of accessible registers for PHY Independent Module. */
#define	LPDDR4_INTR_PHY_INDEP_REG_COUNT (345U)

/** Number of accessible registers for PHY. */
#define	LPDDR4_INTR_PHY_REG_COUNT (1406U)

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
    LPDDR4_INTR_TIMEOUT_ZQ_CAL_INIT = 0U,
    LPDDR4_INTR_TIMEOUT_ZQ_CALLATCH = 1U,
    LPDDR4_INTR_TIMEOUT_ZQ_CALSTART = 2U,
    LPDDR4_INTR_TIMEOUT_MRR_TEMP = 3U,
    LPDDR4_INTR_TIMEOUT_DQS_OSC_REQ = 4U,
    LPDDR4_INTR_TIMEOUT_DFI_UPDATE = 5U,
    LPDDR4_INTR_TIMEOUT_LP_WAKEUP = 6U,
    LPDDR4_INTR_TIMEOUT_AUTO_REFRESH_MAX = 7U,
    LPDDR4_INTR_ECC_ERROR = 8U,
    LPDDR4_INTR_LP_DONE = 9U,
    LPDDR4_INTR_LP_TIMEOUT = 10U,
    LPDDR4_INTR_PORT_TIMEOUT = 11U,
    LPDDR4_INTR_RFIFO_TIMEOUT = 12U,
    LPDDR4_INTR_TRAINING_ZQ_STATUS = 13U,
    LPDDR4_INTR_TRAINING_DQS_OSC_DONE = 14U,
    LPDDR4_INTR_TRAINING_DQS_OSC_UPDATE_DONE = 15U,
    LPDDR4_INTR_TRAINING_DQS_OSC_OVERFLOW = 16U,
    LPDDR4_INTR_TRAINING_DQS_OSC_VAR_OUT = 17U,
    LPDDR4_INTR_USERIF_OUTSIDE_MEM_ACCESS = 18U,
    LPDDR4_INTR_USERIF_MULTI_OUTSIDE_MEM_ACCESS = 19U,
    LPDDR4_INTR_USERIF_PORT_CMD_ERROR = 20U,
    LPDDR4_INTR_USERIF_WRAP = 21U,
    LPDDR4_INTR_USERIF_INVAL_SETTING = 22U,
    LPDDR4_INTR_MISC_MRR_TRAFFIC = 23U,
    LPDDR4_INTR_MISC_SW_REQ_MODE = 24U,
    LPDDR4_INTR_MISC_CHANGE_TEMP_REFRESH = 25U,
    LPDDR4_INTR_MISC_TEMP_ALERT = 26U,
    LPDDR4_INTR_MISC_REFRESH_STATUS = 27U,
    LPDDR4_INTR_BIST_DONE = 28U,
    LPDDR4_INTR_CRC = 29U,
    LPDDR4_INTR_DFI_UPDATE_ERROR = 30U,
    LPDDR4_INTR_DFI_PHY_ERROR = 31U,
    LPDDR4_INTR_DFI_BUS_ERROR = 32U,
    LPDDR4_INTR_DFI_STATE_CHANGE = 33U,
    LPDDR4_INTR_DFI_DLL_SYNC_DONE = 34U,
    LPDDR4_INTR_DFI_TIMEOUT = 35U,
    LPDDR4_INTR_DIMM = 36U,
    LPDDR4_INTR_FREQ_DFS_REQ_HW_IGNORE = 37U,
    LPDDR4_INTR_FREQ_DFS_HW_TERMINATE = 38U,
    LPDDR4_INTR_FREQ_DFS_HW_DONE = 39U,
    LPDDR4_INTR_FREQ_DFS_REQ_SW_IGNORE = 40U,
    LPDDR4_INTR_FREQ_DFS_SW_TERMINATE = 41U,
    LPDDR4_INTR_FREQ_DFS_SW_DONE = 42U,
    LPDDR4_INTR_INIT_MEM_RESET_DONE = 43U,
    LPDDR4_INTR_MC_INIT_DONE = 44U,
    LPDDR4_INTR_INIT_POWER_ON_STATE = 45U,
    LPDDR4_INTR_MRR_ERROR = 46U,
    LPDDR4_INTR_MR_READ_DONE = 47U,
    LPDDR4_INTR_MR_WRITE_DONE = 48U,
    LPDDR4_INTR_PARITY_ERROR = 49U,
    LPDDR4_INTR_LOR_BITS = 50U
} LPDDR4_INTR_CtlInterrupt;

/** PHY Independent Module status or error interrupts. */
typedef enum {
    LPDDR4_INTR_PHY_INDEP_INIT_DONE_BIT = 0U,
    LPDDR4_INTR_PHY_INDEP_CA_PARITY_ERR_BIT = 1U,
    LPDDR4_INTR_PHY_INDEP_RDLVL_ERROR_BIT = 2U,
    LPDDR4_INTR_PHY_INDEP_RDLVL_G_ERROR_BIT = 3U,
    LPDDR4_INTR_PHY_INDEP_WRLVL_ERROR_BIT = 4U,
    LPDDR4_INTR_PHY_INDEP_CALVL_ERROR_BIT = 5U,
    LPDDR4_INTR_PHY_INDEP_WDQLVL_ERROR_BIT = 6U,
    LPDDR4_INTR_PHY_INDEP_UPDATE_ERROR_BIT = 7U,
    LPDDR4_INTR_PHY_INDEP_RDLVL_REQ_BIT = 8U,
    LPDDR4_INTR_PHY_INDEP_RDLVL_GATE_REQ_BIT = 9U,
    LPDDR4_INTR_PHY_INDEP_WRLVL_REQ_BIT = 10U,
    LPDDR4_INTR_PHY_INDEP_CALVL_REQ_BIT = 11U,
    LPDDR4_INTR_PHY_INDEP_WDQLVL_REQ_BIT = 12U,
    LPDDR4_INTR_PHY_INDEP_LVL_DONE_BIT = 13U,
    LPDDR4_INTR_PHY_INDEP_BIST_DONE_BIT = 14U,
    LPDDR4_INTR_PHY_INDEP_TDFI_INIT_TIME_OUT_BIT = 15U,
    LPDDR4_INTR_PHY_INDEP_DLL_LOCK_STATE_CHANGE_BIT = 16U,
    LPDDR4_INTR_PHY_INDEP_MEM_RST_VALID_BIT = 17U,
    LPDDR4_INTR_PHY_INDEP_ZQ_STATUS_BIT = 18U,
    LPDDR4_INTR_PHY_INDEP_PERIPHERAL_MRR_DONE_BIT = 19U,
    LPDDR4_INTR_PHY_INDEP_WRITE_NODEREG_DONE_BIT = 20U,
    LPDDR4_INTR_PHY_INDEP_FREQ_CHANGE_DONE_BIT = 21U,
    LPDDR4_INTR_PHY_INDEP_RDLVL_G_DONE_BIT = 22U,
    LPDDR4_INTR_PHY_INDEP_RDLVL_DONE_BIT = 23U,
    LPDDR4_INTR_PHY_INDEP_WRLVL_DONE_BIT = 24U,
    LPDDR4_INTR_PHY_INDEP_CALVL_DONE__BIT = 25U,
    LPDDR4_INTR_PHY_INDEP_WDQLVL_DONE_BIT = 26U,
    LPDDR4_INTR_PHY_INDEP_VREF_DONE_BIT = 27U,
    LPDDR4_INTR_PHY_INDEP_ANY_VALID_BIT = 28U
} LPDDR4_INTR_PhyIndepInterrupt;

/**
 *  @}
 */



#ifdef __cplusplus
}
#endif

#endif	/* LPDDR4_16BIT_IF_H */
