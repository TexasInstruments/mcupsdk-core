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
 * \file  loopback_main.c
 *
 * \brief This file contains the main task of the Enet loopback example.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "loopback_common.h"
#include "loopback_cfg.h"

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

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void EnetLpbk_mainTask(void *args)
{
    uint32_t i;
    int32_t status;
    Enet_MacPort macPortList[ENET_MAC_PORT_NUM];
    uint8_t numMacPorts;

    Drivers_open();
    Board_driversOpen();

    /* Initialize loopback test config */
    memset(&gEnetLpbk, 0, sizeof(gEnetLpbk));
    gEnetLpbk.exitFlag = false;

    EnetApp_getEnetInstInfo(CONFIG_ENET_CPSW0, &gEnetLpbk.enetType,
                                &gEnetLpbk.instId);

    EnetApp_getEnetInstMacInfo(gEnetLpbk.enetType,
                                gEnetLpbk.instId,
                                macPortList,
                                &numMacPorts);

    EnetAppUtils_assert(numMacPorts == 1);
    gEnetLpbk.macPort          = macPortList[0];

    for (i = 0U; i < ENETLPBK_NUM_ITERATION; i++)
    {
        /* Note: Clock create/delete must be done per iteration to account for
         * memory allocation (from heap) done in snprintf for code reentrancy.
         * Moving this out will result in heap memory leak error in external
         * loopback mode on A72. */

        EnetAppUtils_print("=============================\r\n");
        EnetAppUtils_print(" Enet Loopback: Iteration %u \r\n", i + 1);
        EnetAppUtils_print("=============================\r\n");

        /* Run the loopback test */
        status = EnetApp_loopbackTest();

        /* Sleep at end of each iteration to allow idle task to delete all terminated tasks */
        ClockP_usleep(1000);
    }

    if (status == ENET_SOK)
    {
        EnetAppUtils_print("Loopback application completed\r\n");
        EnetAppUtils_print("All tests have passed!!\r\n");
    }
    else
    {
        EnetAppUtils_print("Loopback application failed to complete\r\n");
    }

    return;
}
