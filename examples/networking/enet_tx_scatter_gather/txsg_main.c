/*
 *  Copyright (c) Texas Instruments Incorporated 2020
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
 * \file  txsg_main.c
 *
 * \brief This file contains the main task of the Enet txsg example.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "txsg_common.h"
#include "txsg_cfg.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define APP_ENABLE_STATIC_CFG                      (0U)

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

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void EnetTxSG_mainTask(void *args)
{
    int32_t status;
    Enet_MacPort macPortList[ENET_MAC_PORT_NUM];
    uint8_t numMacPorts;

	Drivers_open();
    Board_driversOpen();
    /* Initialize txsg test config */
    memset(&gEnetTxSG, 0, sizeof(gEnetTxSG));
    gEnetTxSG.exitFlag = false;

    EnetApp_getEnetInstInfo(CONFIG_ENET_CPSW0, &gEnetTxSG.enetType,
                                &gEnetTxSG.instId);

    EnetApp_getEnetInstMacInfo(gEnetTxSG.enetType,
                                gEnetTxSG.instId,
                                macPortList,
                                &numMacPorts);
    EnetAppUtils_assert(numMacPorts == 1);

#if (1U == APP_ENABLE_STATIC_CFG)
    gEnetTxSG.instId           = 0U;
    gEnetTxSG.testLoopBackType = TXSG_LOOPBACK_TYPE_PHY;
    gEnetTxSG.macPort          = ENET_MAC_PORT_1;
    gEnetTxSG.macMode          = RGMII;
    gEnetTxSG.enetType         = ENET_CPSW_2G;
    gEnetTxSG.boardId          = ENETBOARD_CPB_ID;
#else
    EnetTxSG_showMenu();
    gEnetTxSG.macPort          = macPortList[0];
#endif


    /* Note: Clock create/delete must be done per iteration to account for
     * memory allocation (from heap) done in snprintf for code reentrancy.
     * Moving this out will result in heap memory leak error in external
     * txsg mode on A72. */

    EnetAppUtils_print("=============================\r\n");
    EnetAppUtils_print(" Enet TxSG: Iteration %u \r\n", 1);
    EnetAppUtils_print("=============================\r\n");

    /* Run the txsg test */
    status = EnetTxSG_txsgTest();

    /* Sleep at end of each iteration to allow idle task to delete all terminated tasks */
    ClockP_usleep(1000);

    if (status == ENET_SOK)
    {
        EnetAppUtils_print("TxSG application completed\r\n");
		EnetAppUtils_print("All tests have passed!!\r\n");
    }
    else
    {
        EnetAppUtils_print("TxSG application failed to complete\r\n");
    }

    return;
}
