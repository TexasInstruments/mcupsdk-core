/*
 *  Copyright (c) Texas Instruments Incorporated 2023
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

#ifndef __TSNINIT_H__
#define __TSNINIT_H__

#include <tsn_combase/tilld/lldtype.h>

#if defined(SAFERTOS)
#define TSN_TSK_STACK_SIZE                         (16U * 1024U)
#define TSN_TSK_STACK_ALIGN                        TSN_TSK_STACK_SIZE
#else
#if _DEBUG_ == 1
/* 16k is enough for debug mode */
#define TSN_TSK_STACK_SIZE                         (16U * 1024U)
#else
#define TSN_TSK_STACK_SIZE                         (8U * 1024U)
#endif
#define TSN_TSK_STACK_ALIGN                        (32U)
#endif

typedef struct AppTsnCfg {
    Logger_onConsoleOut consoleOutCb; //<! A callback function for log output on console.
    char *netdevs[LLDENET_MAX_PORTS+1]; //!< A list of network interfaces each is a string, terminated by NULL;
} AppTsnCfg_t;

int EnetApp_initTsnByCfg(AppTsnCfg_t *cfg);
/* start and stop a single TSN module, @ref EnetApp_TsnTask_Idx_t for the moduleIdx */
int EnetApp_startTsnModule(int moduleIdx);
void EnetApp_stopTsnModule(int moduleIdx);
/* start and stop all the TSN modules */
int EnetApp_startTsn(void);
void EnetApp_stopTsn(void);
void EnetApp_deInitTsn(void);

#endif
