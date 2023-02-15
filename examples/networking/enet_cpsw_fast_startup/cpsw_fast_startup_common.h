/*
 * Copyright (C) 2023 Texas Instruments Incorporated
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

#ifndef _CPSW_FAST_STARTUP_COMMON_H_
#define _CPSW_FAST_STARTUP_COMMON_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <string.h>
#include <assert.h>

#include <include/core/enet_osal.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/EventP.h>
#include <enet.h>
#include <enet_cfg.h>
#include <include/core/enet_dma.h>
#include <include/per/cpsw.h>

#include <enet_apputils.h>
#include <enet_appmemutils.h>
#include <enet_appmemutils_cfg.h>

#include "ti_board_config.h"
#include "ti_enet_open_close.h"
#include "ti_enet_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

#ifdef __cplusplus
extern "C" {
#endif


/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* Max number of ports supported per context */
#define ENETAPP_PORT_MAX                           (2U)

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/* Test parameters for each port in the L2 cpsw test */
typedef struct EnetApp_TestParams_s
{
    /* Peripheral type */
    Enet_Type enetType;

    /* Peripheral instance */
    uint32_t instId;

    /* Peripheral's MAC ports to use */
    Enet_MacPort macPort[ENETAPP_PORT_MAX];

	/* Number of MAC ports in macPorts array */
    uint32_t macPortNum;

    /* Name of this port to be used for logging */
    char *name;
} EnetApp_TestParams;

typedef enum AppEventId_e
{
    AppEventId_NONE = 0,
    AppEventId_RXPKT = (1 << 0),
    AppEventId_TXPKT = (1 << 1),
    AppEventId_PERIODIC_POLL = (1 << 2),
    AppEventId_TERMINATE = (1 << 3),
    AppEventId_ANY_EVENT = (AppEventId_RXPKT |
                             AppEventId_TXPKT |
                             AppEventId_PERIODIC_POLL |
                             AppEventId_TERMINATE),
} AppEventId_t;

typedef struct EnetApp_Obj_s
{
    /* Peripheral type */
    Enet_Type enetType;

    /* Peripheral instance */
    uint32_t instId;

    /* Peripheral's MAC ports to use */
    Enet_MacPort  macPort[ENETAPP_PORT_MAX];

    /* Number of MAC ports in macPorts array */
    uint32_t macPortNum;

    /* Name of this port to be used for logging */
    char *name;

    /* Enet driver handle for this peripheral type/instance */
    Enet_Handle hEnet;

    /* MAC address. It's port's MAC address in Dual-MAC or
     * host port's MAC addres in Switch */
    uint8_t macAddr[ENET_MAC_ADDR_LEN];

    /* TX channel handle */
    EnetDma_TxChHandle hTxCh;

    /* RX channel handle for regular traffic */
    EnetDma_RxChHandle hRxCh;

    /* Core key returned by Enet RM after attaching this core */
    uint32_t coreKey;

    /* Flag which indicates if test shall run */
    volatile bool run;

    /* This core's id */
    uint32_t coreId;

    /* Queue of free TX packets */
    EnetDma_PktQ txFreePktInfoQ;
            
    EventP_Object appEvents;
} EnetApp_Obj;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

void EnetApp_mainTask(void *args);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* Enet l2 cpsw test object */
EnetApp_Obj gEnetApp;

#ifdef __cplusplus
}
#endif

#endif /* _CPSW_FAST_STARTUP_COMMON_H_ */
