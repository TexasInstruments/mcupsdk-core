/*!
 *  \file
 *
 *  \brief
 *  utilities for IOLink Integration Layer
 *
 *  \author
 *  KUNBUS GmbH
 *
 *  \date
 *  2022-07-22
 *
 *  \copyright
 *  Copyright (c) 2022, KUNBUS GmbH<br /><br />
 *  All rights reserved.<br />
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:<br />
 *  <ol>
 *  <li>Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimer.</li>
 *  <li>Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.</li>
 *  <li>Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.</li>
 *  </ol>
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#if !(defined PROTECT_IILFUNCTION_H)
#define PROTECT_IILFUNCTION_H      1

#include <stdint.h>
#include <osal.h>

#include "gw_errorhandling.h"

#if (defined __cplusplus)
extern "C" {
#endif

// maximum simultaneous processing of SMI commands
#define IIL_MAX_CALLBACKS       10U

// value for a full callback list
#define IIL_LIST_INVALID         0xFFU

// typedef of callback function for SMI responses in CallbackMode
typedef void(*IIL_CALLBACK_t)(uint32_t errorCodeSMI_p);

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  typedef struct of open callback command
 *
 * */
typedef struct IIL_SMI_SCallbackCmd
{
    uint8_t        clientId;       /**! \brief ClientId differentiates between sources such as Ethernet or SMIdirect */
    uint8_t        portNr;         /**! \brief used IOLink port number to identify callback, 0 == IOLink Master */
    uint16_t       command;        /**! \brief command identifer to identify callback */
    void*          pResultBuffer;  /**! \brief buffer for dend data or receive data */
    uint16_t*      pResultLength;  /**! \brief send data length or receive data length */
    IIL_CALLBACK_t cbSMICallback;  /**! \brief callback function, NULL==WaitingMode */
} IIL_SMI_SCallbackCmd_t;

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  typedef struct of open callback entry, all information for asynchron SMI callbacks
 *
 * */
typedef struct IIL_SMI_SCallbackEntry
{
    bool                       busy;       /**! \brief FALSE==entry is free, TRUE==entry is in use */
    uint32_t                   errorCode;  /**! \brief received error from IOLink */
    OSAL_SCHED_SEventHandle_t* pEvent;     /**! \brief event handle used for WaitingMode */
    IIL_SMI_SCallbackCmd_t     sCmdValues; /**! \brief parameter to identify callback */
} IIL_SMI_SCallbackEntry_t;

extern IIL_SMI_SCallbackEntry_t IIL_aCallbackList_g[IIL_MAX_CALLBACKS];
extern  OSAL_SCHED_SMutexHandle_t* IIL_pMutexList_g;

extern void IIL_wakeupTask(
    const uint8_t  listEntry_p,
    const uint16_t errorCodeSMI_p);

extern GW_ECIOL_ERRORCODE IIL_waitTask(
    const uint8_t listEntry_p);

extern uint8_t IIL_searchCallback(
    const uint8_t  clientId_p,
    const uint8_t  portNr_p,
    const uint16_t command_p);

extern void IIL_freeCallbackEntry(
    const uint8_t listEntry_p);

extern uint8_t IIL_setCallbackEntry(
    const IIL_SMI_SCallbackCmd_t* const psCmdValues_p);

#if (defined __cplusplus)
}
#endif

#endif /* PROTECT_IILFUNCTION_H */
