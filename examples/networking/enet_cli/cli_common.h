/*
 * Copyright (C) 2024 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*!
 * \file  cli_common.h
 *
 * \brief This is the common header file of the CLI application.
 */

#ifndef _CLI_COMMON_H_
#define _CLI_COMMON_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>

#include <include/core/enet_osal.h>
#include <kernel/dpl/TaskP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/SemaphoreP.h>

#include <enet.h>
#include <enet_cfg.h>
#include <include/core/enet_dma.h>
#include <include/per/cpsw.h>

#include <enet_apputils.h>
#include <enet_appmemutils.h>
#include <enet_appmemutils_cfg.h>

/* SDK includes */
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "ti_board_config.h"
#include "ti_enet_open_close.h"
#include "ti_enet_config.h"

/* FreeRTOS CLI library */
#include "FreeRTOS.h"
#include "FreeRTOS_CLI.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define MAX_NUM_DMA_CH	2

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/* Structure with parameters of an Ethernet object */
typedef struct EnetApp_Obj_s
{
    /* Enet driver */
    Enet_Type enetType;
    uint32_t instId;
    uint32_t coreId;
    uint32_t coreKey;
    uint32_t boardId;
    uint8_t numMacPorts;
    Enet_MacPort macPort[ENET_MAC_PORT_NUM];
    emac_mode macMode; /* MAC mode (defined in board library) */
    Enet_Handle hEnet;
    uint8_t hostMacAddr[ENET_MAC_ADDR_LEN];
    int8_t numTxDmaCh;
    int8_t numRxDmaCh;

    /* Packet transmission */
    EnetDma_TxChHandle hTxCh[MAX_NUM_DMA_CH];
    EnetDma_PktQ txFreePktInfoQ[MAX_NUM_DMA_CH];
    SemaphoreP_Object txSemObj[MAX_NUM_DMA_CH];

    /* Packet reception */
    EnetDma_RxChHandle hRxCh[MAX_NUM_DMA_CH];
    TaskP_Object rxTaskObj[MAX_NUM_DMA_CH];
    EnetDma_PktQ rxFreeQ[MAX_NUM_DMA_CH];
    EnetDma_PktQ rxReadyQ[MAX_NUM_DMA_CH];
    SemaphoreP_Object rxSemObj[MAX_NUM_DMA_CH];
    int8_t rxRunFlag[MAX_NUM_DMA_CH];

    /* Flag to indicate whether Enet object is initialized */
    int8_t initFlag;
} EnetApp_Obj;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

void EnetApp_createClock(void);

void EnetApp_deleteClock(void);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* Enet object instance declaration */
EnetApp_Obj EnetApp_Inst;

#ifdef __cplusplus
}
#endif

#endif /* _CLI_COMMON_H_ */
