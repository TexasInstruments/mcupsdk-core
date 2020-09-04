/*
 *  Copyright (c) Texas Instruments Incorporated 2021
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

/*!
 * \file  loopback_cfg.c
 *
 * \brief This file contains the configuration related APIs of enet loopback app.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "txsg_common.h"
#include "txsg_cfg.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* 100-ms periodic tick */
#define ENETTXSG_PERIODIC_TICK_MS                (100U)

/* Counting Semaphore count */
#define COUNTING_SEM_COUNT                       (10U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

typedef struct EnetTxSG_EnetTypeMenu_s
{
    const char *text;
    Enet_Type enetType;
    uint32_t instId;
} EnetTxSG_EnetTypeMenu;

typedef struct EnetTxSG_PortMenu_s
{
    const char *text;
    Enet_MacPort macPort;
    emac_mode macMode;
    uint32_t boardId;
} EnetTxSG_PortMenu;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static void EnetTxSG_showTxSGMenu(EnetTxSG_type *loopbackType);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* Enet loopback test object */
EnetTxSG_Obj gEnetTxSG;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void EnetTxSG_showMenu(void)
{
    gEnetTxSG.macPort  = ENET_MAC_PORT_1;
    gEnetTxSG.macMode  = RGMII;
    gEnetTxSG.boardId  = ENETBOARD_CPB_ID;
    EnetTxSG_showTxSGMenu(&gEnetTxSG.testLoopBackType);
}


/* ========================================================================== */
/*                   Static Function Definitions                              */
/* ========================================================================== */


static void EnetTxSG_showTxSGMenu(EnetTxSG_type *loopbackType)
{
    bool retry;
    int32_t choice = -1;

    do
    {
        EnetAppUtils_print("Select loopback type:\r\n");
        EnetAppUtils_print("0: External (PHY loopback)\r\n");
        EnetAppUtils_print("1: Transmit to network\r\n");

        DebugP_scanf("%i", &choice);

        switch (choice)
        {
            case 0:
                *loopbackType = TXSG_LOOPBACK_TYPE_PHY;
                retry = false;
                break;
            case 1:
                *loopbackType = TXSG_LOOPBACK_TYPE_NONE;
                retry = false;
                break;
            default:
                EnetAppUtils_print("Wrong option, try again...\r\n\n");
                retry = true;
                break;
        }

    }
    while (retry);
}

