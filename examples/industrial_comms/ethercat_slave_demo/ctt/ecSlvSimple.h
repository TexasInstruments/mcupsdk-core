/*!
 *  \example ecSlvSimple.h
 *
 *  \brief
 *  Brief description of purpose and functionality.
 *
 *  \author
 *  KUNBUS GmbH
 *
 *  \date
 *  2021-05-18
 *
 *  \copyright
 *  Copyright (c) 2021, KUNBUS GmbH<br /><br />
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

#if !(defined __ECSLVSIMPLE_H__)
#define __ECSLVSIMPLE_H__		1

#include <osal.h>
#include <ecSlvApi.h>

#include <ESL_os.h>

/*-----------------------------------------------------------------------------------------
------
------    Includes
------
-----------------------------------------------------------------------------------------*/

typedef struct EC_SLV_APP_Sapplication
{
    uint32_t                        selectedPruInstance;

    /* Threads */
    TaskP_Object                    mainThreadHandle;
    TaskP_Params                    mainThreadParam;
    void*                           loopThreadHandle;

    /* Resources */
    void*                           gpioHandle;
    void*                           remoteHandle;
    void*                           ioexpLedHandle;

    int32_t                         msec,
                                    trigger;

    uint8_t                         state;
    uint8_t                         rsvd[3]; /* better be uint32_t aligned */
    clock_t                         prev, diff;

    EC_API_SLV_SCoE_Object_t*       ptRecObjOut;
    EC_API_SLV_SCoE_Object_t*       pt2002RecObj;
    EC_API_SLV_SCoE_Object_t*       pt2007RecObj;
    EC_API_SLV_SCoE_Object_t*       pt200FRecObj;

    EC_API_SLV_SPdo_t*              ptRxPdo1600;
    EC_API_SLV_SPdo_t*              ptRxPdo1601;
    EC_API_SLV_SPdo_t*              ptTxPdo1A00;
    EC_API_SLV_SPdo_t*              ptTxPdo1A01;

    EC_API_SLV_SHandle_t*           ptEcSlvApi;
} EC_SLV_APP_Sapplication_t;

#if (defined __cplusplus)
extern "C" {
#endif

extern void EC_SLV_APP_initBoardFunctions             (EC_SLV_APP_Sapplication_t*     pAppInstance_p);
extern void EC_SLV_APP_registerStacklessBoardFunctions(EC_SLV_APP_Sapplication_t*     pAppInstance_p);
extern void EC_SLV_APP_applicationInit                (EC_SLV_APP_Sapplication_t*     pAppInstance_p);
extern void EC_SLV_APP_applicationDeInit              (EC_SLV_APP_Sapplication_t*     pAppInstance_p);

#if (defined __cplusplus)
}
#endif

#endif /* __ECSLVSIMPLE_H__ */
