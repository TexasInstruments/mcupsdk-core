/*
 *  Copyright (C) 2018-2020 Texas Instruments Incorporated
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
 *  \file sciclient_priv.h
 *
 *  \brief This file contains the handle structure used internally by sciclient.
 */
#ifndef SCICLIENT_PRIV_H_
#define SCICLIENT_PRIV_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <drivers/hw_include/hw_types.h>
#include <drivers/hw_include/csl_types.h>
#include <drivers/sciclient/csl_sec_proxy.h>

#include <drivers/sciclient.h>
#include <drivers/sciclient/sciclient_romMessages.h>
#include <drivers/sciclient/soc/sciclient_soc_priv.h>
#include <drivers/hw_include/cslr_soc.h>


#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/**
 *    \brief Maximum number of messages waiting to be read.
 *           Cannot be greater than 256.
 */
#define SCICLIENT_MAX_QUEUE_SIZE            (7U)

/* Current context is SECURE */
#define SCICLIENT_SECURE_CONTEXT            (0U)

/* Current context is NON-SECURE */
#define SCICLIENT_NON_SECURE_CONTEXT        (1U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 *  \brief Map structure used by #Sciclient_init function.
 */
typedef struct
{
    uint32_t context;
    /**< context(sec/non-sec) **/

    uint32_t hostId;
    /**< CPU ID of the A53/A72/R5F/DSP */

    uint32_t reqLowPrioThreadId;
    /**< Thread ID of the low priority thread(write) allowed for the CPU */

    uint32_t respThreadId;
    /**< Thread ID of the response thread(read) available for the CPU */

    uint32_t respIntrNum;
    /**< Response Interrupt Number. */
} Sciclient_MapStruct_t;

/**
 *  \brief Handle for #Sciclient_service function
 */
typedef struct
{
    uint32_t              currSeqId;
    /**< Sequence ID of the current request **/

    uint32_t              coreId;
    /**< ID of the core on which driver is running **/

    uint32_t              devIdCore;
    /**< TISCI Device ID of the core on which driver is running **/

    uint32_t              secureContextId;
    /**< Secure context ID of the core, invalid if the context is non-secure **/

    uint32_t              nonSecureContextId;
    /**< Non-Secure context ID of the core, invalid if the context is secure **/

    uint32_t              maxMsgSizeBytes;
    /**< Max size of an sciclient message in bytes. Dependent on the secure proxy configuration of the SOC **/
} Sciclient_ServiceHandle_t;


/**
 *  \anchor Sciclient_proxyMap
 *  \name Sciclient map structure
 *  @{
 *  Map structure for R5F,A53,GPU and ICSSG \n
 *  in different contexts.
 */
extern const Sciclient_MapStruct_t gSciclientMap[SCICLIENT_CONTEXT_MAX_NUM];
/* @} */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**<
 *  \brief   API to send the board configuration messsage to the firmware .
 *          Valid(not NULL) pointer to Sciclient_BoardCfgPrms_t will use the
 *          provided values for tisci_msg_board_config_req, otherwise
 *          default values are used .
 *
 *  \param pInPrms   [IN] Pointer to #Sciclient_BoardCfgPrms_t .
 *
 *  \return  status    Status of the message.
 */
int32_t Sciclient_boardCfg(const Sciclient_BoardCfgPrms_t * pInPrms);

/**
 *  \brief   API to identify which mode the CPU is operating in. This utility
 *           function would read CPU related registers to know which mode
 *           (secure or non secure) the CPU is in and then would determine the
 *           context to be used. If more than one context is required for a
 *           a given code, users of SCICLENT would need to modify this function
 *           and recompile.
 *
 *  \param   messageType The Message ID to be checked.
 *
 *  \return  retVal     SCICLENT Context of the CPU
 */
uint32_t Sciclient_getCurrentContext(uint16_t messageType);

#ifdef __cplusplus
}
#endif

#endif /* #ifndef SCICLIENT_PRIV_H_*/
