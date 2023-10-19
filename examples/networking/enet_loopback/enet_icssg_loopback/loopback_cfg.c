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

#include "loopback_common.h"
#include "loopback_cfg.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* 100-ms periodic tick */
#define ENETLPBK_PERIODIC_TICK_MS                (100U)

/* Counting Semaphore count */
#define COUNTING_SEM_COUNT                       (10U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
static const char loopbackTestMenu[] =
{
    " \r\n 1: External PHY loopback"
    " \r\n"
    " \r\n Enter option: "
};

/* Enet loopback test object */
EnetLpbk_Obj gEnetLpbk;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void EnetApp_showMenu(void)
{

    int8_t option = -1;
    bool retry = true;
    DebugP_log("%s", loopbackTestMenu);
    while(retry)
    {
        DebugP_scanf("%d", &option);

        switch(option)
        {
            case 1U:
                gEnetLpbk.testLoopBackType = LOOPBACK_TYPE_PHY;
                gEnetLpbk.macMode          = RGMII;
                gEnetLpbk.boardId          = ENETBOARD_CPB_ID;
                retry = false;
                break;
            default:
               DebugP_log(" \r\n Invalid option... Try Again!!!\r\n");
               DebugP_log("%s", loopbackTestMenu);
               break;
        }
        TaskP_yield();
    }
}

/* ========================================================================== */
/*                   Static Function Definitions                              */
/* ========================================================================== */
