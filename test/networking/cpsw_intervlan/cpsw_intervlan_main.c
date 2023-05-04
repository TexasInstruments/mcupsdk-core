/*
 *  Copyright (c) Texas Instruments Incorporated 2022
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
 * \file  cpsw_intervlan_main.c
 *
 * \brief This file contains the main task of the Cpsw intervlan example.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "cpsw_intervlan_common.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */


/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* Enet cpswintervlan test object */
EnetCpswInterVlan_Obj gEnetCpswInterVlan;


/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void EnetCpswInterVlan_mainTask(void *args)
{
    int32_t status;

    Drivers_open();
    Board_driversOpen();

    /* Initialize test config */
    memset(&gEnetCpswInterVlan, 0, sizeof(gEnetCpswInterVlan));
    gEnetCpswInterVlan.exitFlag = false;

    EnetApp_getEnetInstInfo(CONFIG_ENET_CPSW0,
                            &gEnetCpswInterVlan.enetType,
                            &gEnetCpswInterVlan.instId);

    EnetApp_getEnetInstMacInfo(gEnetCpswInterVlan.enetType,
                                gEnetCpswInterVlan.instId,
                                gEnetCpswInterVlan.macPortList,
                                &gEnetCpswInterVlan.numMacPorts);

    gEnetCpswInterVlan.macMode          = RGMII;
    gEnetCpswInterVlan.boardId          = ENETBOARD_CPB_ID;


    EnetAppUtils_print("=====================\r\n");
    EnetAppUtils_print(" Enet Cpsw Intervlan \r\n");
    EnetAppUtils_print("=====================\r\n");

    /* Run the test */
    status = EnetApp_cpswInterVlanTest();

    /* Sleep at end of each iteration to allow idle task to delete all terminated tasks */
    ClockP_usleep(1000);

    if (status == ENET_SOK)
    {
        EnetAppUtils_print("CpswInterVlan application completed\r\n");
		EnetAppUtils_print("All tests have passed!!\r\n");
    }
    else
    {
        EnetAppUtils_print("CpswInterVlan application failed to complete\r\n");
    }

    return;
}
