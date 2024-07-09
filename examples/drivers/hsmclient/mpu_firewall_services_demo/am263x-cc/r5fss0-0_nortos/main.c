/*
 *  Copyright (C) 2018-2022 Texas Instruments Incorporated
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

#include <stdlib.h>
#include "ti_drivers_config.h"
#include "ti_board_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <security/security_common/drivers/secure_ipc_notify/sipc_notify.h>
#include <security/security_common/drivers/hsmclient/hsmclient.h>

/* ========================================================================== */
/*                          Macros And Typedefs                               */
/* ========================================================================== */

#define APP_CLIENT_ID                  (0x02)
#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  ((byte) & 0x80 ? '1' : '0'), \
  ((byte) & 0x40 ? '1' : '0'), \
  ((byte) & 0x20 ? '1' : '0'), \
  ((byte) & 0x10 ? '1' : '0'), \
  ((byte) & 0x08 ? '1' : '0'), \
  ((byte) & 0x04 ? '1' : '0'), \
  ((byte) & 0x02 ? '1' : '0'), \
  ((byte) & 0x01 ? '1' : '0')

#define MPU_INT_ENABLED_STATUS_CLEAR    (0x03)
#define MPU_INT_FAULT_CLEAR             (0x01)
#define MPU_INT_ENABLE                  (0x03)

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

extern FirewallRegionReq_t gMpuFirewallRegionConfig_0[FIREWALL_ARRAY0_NUM_REGIONS] ;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void mpu_firewall_demo_main();

void mpuFirewallReq(void)
{
    int32_t status ;
    HsmClient_t HsmClient ;
    /* struct instance used for sending set firewall request */
    FirewallReq_t FirewallReqObj;
    /* struct instance used for sending firewall interrupt clear/enable request */
    FirewallIntrReq_t FirewallIntrReqObj;

    /* HSM Client Register */
    status = HsmClient_register(&HsmClient,APP_CLIENT_ID);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Clearing the fault registers and raw status in same firewall interrupt request to HSM*/
    FirewallIntrReqObj.firewallId = CSL_FW_R5SS0_CORE1_AHB_MST_ID;
    FirewallIntrReqObj.interruptEnableStatusClear = MPU_INT_ENABLED_STATUS_CLEAR;
    FirewallIntrReqObj.faultClear = MPU_INT_FAULT_CLEAR;
    status = HsmClient_FirewallIntr(&HsmClient, &FirewallIntrReqObj, SystemP_WAIT_FOREVER);
    DebugP_assert(status==SystemP_SUCCESS);

    /* clearing the MPU firewall interrupt object */
    memset(&FirewallIntrReqObj, 0, sizeof(FirewallIntrReqObj));

    /* Enabling the interrupt for both address violation and protection violation for MPU_R5SS0_CORE1_AHB*/
    FirewallIntrReqObj.firewallId = CSL_FW_R5SS0_CORE1_AHB_MST_ID;
    FirewallIntrReqObj.interruptEnable = MPU_INT_ENABLE;
    status = HsmClient_FirewallIntr(&HsmClient, &FirewallIntrReqObj, SystemP_WAIT_FOREVER);
    DebugP_assert(status==SystemP_SUCCESS);

    DebugP_log("\r\n Enabling the interrupt for both address violation and protection violation for MPU_R5SS0_CORE1_AHB \r\n");

    /******* SET FIREWALL SERVICE REQ *******/

    /* region count and region configuration array is generated via sysconfig and is used
        for populating  FirewallReqObj*/
    FirewallReqObj.regionCount = FIREWALL_ARRAY0_NUM_REGIONS;
    FirewallReqObj.FirewallRegionArr = gMpuFirewallRegionConfig_0;

    /* This request configures peripheral region(0x5230 0000 - 0x5230 03FF) and revokes all permissions for R5FSS0_1 */
    status = HsmClient_setFirewall(&HsmClient,&FirewallReqObj,SystemP_WAIT_FOREVER);

    DebugP_log("Firewall request #1 status = "BYTE_TO_BINARY_PATTERN" "BYTE_TO_BINARY_PATTERN "\r\n",
        BYTE_TO_BINARY(FirewallReqObj.statusFirewallRegionArr>>8), BYTE_TO_BINARY(FirewallReqObj.statusFirewallRegionArr));


    DebugP_log("\r\n Permissions for peripheral region(0x5230 0000 - 0x5230 03FF) are revoked for R5FSS0_1 \r\n");

    /* HSM Client Un-Register */
    HsmClient_unregister(&HsmClient,APP_CLIENT_ID);

}

int main(void)
{
    System_init();
    Board_init();

    Drivers_open();
    Board_driversOpen();

    mpuFirewallReq();
    mpu_firewall_demo_main();

    Board_driversClose();
    Drivers_close();

    Board_deinit();
    System_deinit();

    return 0 ;
}
