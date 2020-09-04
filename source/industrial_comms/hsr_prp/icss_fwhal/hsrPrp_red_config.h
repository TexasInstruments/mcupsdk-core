/**
 * \file hsrPrp_red_config.h
 * \brief Include file for hsrPrp_red_config.c
 *
 * \par
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
 * \par
 */

#ifndef RED_CONFIG_H_
#define RED_CONFIG_H_

#ifdef __cplusplus
extern "C"
{
#endif


/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "hsrPrp_red_hsr.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/** RED Config */
typedef struct _RED_Config
{

    uint32_t nodeTableSize;               /**< Size of the node table [0..128] */
    uint32_t nodeTableArbitration;        /**< Busy slave flag and busy master flag */
    uint32_t duplicateHostTableSize;      /**< Size and setup (N and M) of duplicate host table */
    uint32_t duplicatePortTableSize;      /**< Size and setup (N and M) of duplicate port table */
    uint32_t nodeForgetTime;              /**< Time after which a node entry is cleared (10 ms resolution) */
    uint32_t duplicateForgetTime;         /**< Time after which an entry is removed from the duplicate table (10 ms resolution) */
    uint32_t brokenPathDifference;        /**< Supervision frame counter - minimum difference to detect a broken path */
    uint32_t duplicatePortCheckInterval;  /**< Time interval to check the port duplicate table */
    uint32_t duplicateHostCheckInterval;  /**< Time interval to check the host duplicate table */
    uint32_t nodeTableCheckInterval;      /**< Time interval to check the node table */
    uint32_t duplicatePort0Counter;       /**< Time counter to trigger the port_dupli_table check task for PRU0 */
    uint32_t duplicatePort1Counter;       /**< Time counter to trigger the port_dupli_table check task for PRU1 */
    uint32_t duplicateHostCounter;        /**< Time counter to trigger the host_dupli_table check task */
    uint32_t nodeTableCounter;            /**< Time counter to trigger the Node_Table check task */
    uint32_t clearNodeTable;              /**< Register set by host to clear the node table */
    uint32_t supAddressHi;                /**< Supervision MAC addresses - HI word */
    uint32_t supAddressLow;               /**< Supervision MAC addresses - LOW word */
    uint32_t hostDuplicateArbitration;    /**< Master/Slave flag for host table arbitrations */

} RED_CONFIG;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
/**
 *
 *  \brief Reads the configuration from DMEM.
 *  \param pruicssHandle Pointer to PRU ICSS Handle, parent structure containing all switch information
 *  \param pCfg Pointer to the HSR_CONFIGURATION structure
 *
 *  \return RED_STATUS RED_OK on success, RED_ERR otherwise
 *
 */
RED_STATUS RedGetConfiguration(RED_CONFIG *pCfg,
                               PRUICSS_Handle pruicssHandle);
/**
 *
 *  \brief Initialises DMEM with default configuration values.
 *
 *  \param pruicssHandle Handle to PRU ICSS instance. Contains pointers to base addresses
 *
 *
 */
void       RedInit(PRUICSS_Handle pruicssHandle);

/**
 *
 *  \brief Loads firmware.
 *
 *  \param pruicssHandle Handle to PRU ICSS instance. Contains pointers to base addresses
 *
 *  \return SystemP_success on success
 *
 */
uint8_t    RedLoadFirmware(PRUICSS_Handle pruicssHandle);


#ifdef __cplusplus
}
#endif

#endif /* RED_CONFIG_H_ */
