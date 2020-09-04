/**
 * \file hsrPrp_handle.h
 * \brief Include file for hsrPrp_red_hsr.c
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

#ifndef RED_HSR_HANDLE_H_
#define RED_HSR_HANDLE_H_

#ifdef __cplusplus
extern "C"
{
#endif


#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/TaskP.h>

/**
 *  \brief  hsrPrp_Config
 *              Structure storing the global variables
 */
typedef struct hsrPrp_Config_s
{
    /*Sequence number for non supervision frames*/
    uint16_t redSeqNr ;
    /*Sequence number for Supervision frames*/
    uint16_t supSeqNr ;
    /*Integer pointers to external variable*/
    int *collision_pkt_dropped;
    /*Integer pointers to external variable*/
    int *num_of_collision_occured;
    /*ICSS emac handle*/
    ICSS_EMAC_Handle    icssEmacHandle;
    /*Host Table timer flag*/
    uint32_t redPruCheckTimerHostTableFlag;
    /*node table clear flag*/
    uint32_t redPruCheckTimerNodeTableClear;
    /*timer*/
    ClockP_Object redPruCheckTimer;
    /*timer*/
    ClockP_Object redLifeCheckTimer;
    /*pointer to a RED_FRAME | generates the supervision frame*/
    RED_FRAME *redSupFrame;
    /*task handle to check successful creation of RedLifeCheckTask*/
    TaskP_Object redSupTask;
    /*task handle to check successful creation of hsrPrp_nodetable_refresh*/
    TaskP_Object hsrPrpNodetableRefreshTask;
    /*semaphore*/
    SemaphoreP_Object redLifeCheckSemaphore;
    /*semaphore*/
    SemaphoreP_Object nodesTableSemaphore;
    /*Index Array base address*/
    RED_INDEX_ARRAY_ENTRY *indexArrayBase;
    /*Bin Array base address*/
    RED_BIN_ARRAY_ENTRY *binArrayBase;
    /*Nodetable base address*/
    RED_NODE_TABLE_ENTRY *nodeTableBase;
} hsrPrp_Config;

typedef struct hsrPrp_Config_s hsrPrpHandle;


#ifdef __cplusplus
}
#endif

#endif /* RED_HSR_HANDLE_H_ */
