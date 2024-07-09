/*
 *  Copyright (C) 2023 Texas Instruments Incorporated
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

#include <stdio.h>
#include <kernel/dpl/CycleCounterP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/SystemP.h>
#include <security/security_common/drivers/hsmclient/hsmclient.h>
#include <security/security_common/drivers/secure_ipc_notify/sipc_notify.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <kernel/dpl/SystemP.h>
#include <string.h>
#include <drivers/hw_include/csl_types.h>
#include <drivers/hw_include/cslr_soc.h>
#include <drivers/mpu_firewall.h>
#include <drivers/soc.h>

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

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

extern FirewallRegionReq_t gMpuFirewallRegionConfig_0[FIREWALL_ARRAY0_NUM_REGIONS] ;
extern FirewallRegionReq_t gMpuFirewallRegionConfig_1[FIREWALL_ARRAY1_NUM_REGIONS] ;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/* Demo Application code on R5 */
void HsmFirewallApp(HsmClient_t *client)
{
    /* loop through and request get version from HSM */
    /* also calculate the time spent doing get version */
    int32_t status ;
    uint16_t regionCount ;
    /* struct instance used for fetching mpu region value */
    MPU_FIREWALL_RegionParams mpuParams;
    /* struct instance used for sending set firewall request */
    FirewallReq_t FirewallReqObj;

    /******* SET FIREWALL SERVICE DEMO *******/

    /* region count and region configuration array is generated via sysconfig and is used
        for populating  FirewallReqObj*/
    FirewallReqObj.regionCount = FIREWALL_ARRAY0_NUM_REGIONS;
    FirewallReqObj.FirewallRegionArr = gMpuFirewallRegionConfig_0;

    /* This requests configures user given firewall request-1 in syscfg and allow all permissions to only R5FSS0_0 */
    status = HsmClient_setFirewall(client,&FirewallReqObj,SystemP_WAIT_FOREVER);

    DebugP_log("Firewall request #1 status = "BYTE_TO_BINARY_PATTERN" "BYTE_TO_BINARY_PATTERN "\r\n",
        BYTE_TO_BINARY(FirewallReqObj.statusFirewallRegionArr>>8), BYTE_TO_BINARY(FirewallReqObj.statusFirewallRegionArr));

    /* MPU_FIREWALL_getRegion is API from firewall driver for fetching MPU region configuration */
    for(regionCount = 0U; regionCount < FIREWALL_ARRAY0_NUM_REGIONS; regionCount++)
    {
        mpuParams.id = gMpuFirewallRegionConfig_0[regionCount].firewallId;
        mpuParams.regionNumber = gMpuFirewallRegionConfig_0[regionCount].region;

        status = MPU_FIREWALL_getRegion(&mpuParams);

        DebugP_log("Firewall Id = %d\r\n",mpuParams.id);
        DebugP_log("Firewall region number = %x\r\n",mpuParams.regionNumber);
        DebugP_log("Start Address = 0x%x\r\n",mpuParams.startAddress);
        DebugP_log("End Address = 0x%x\r\n",mpuParams.endAddress);
        DebugP_log("Aid Config = 0x%x\r\n",mpuParams.aidConfig);
        DebugP_log("Supervisor Read = %x\r\n",mpuParams.supervisorReadConfig);
        DebugP_log("Supervisor Write = %x\r\n",mpuParams.supervisorWriteConfig);
        DebugP_log("Supervisor Execute = %x\r\n",mpuParams.supervisorExecConfig);
        DebugP_log("User Read = %x\r\n",mpuParams.userReadConfig);
        DebugP_log("User Write = %x\r\n",mpuParams.userWriteConfig);
        DebugP_log("User Execute = %x\r\n",mpuParams.userExecConfig);
        DebugP_log("Non Secure Access = %x\r\n",mpuParams.nonSecureConfig);
        DebugP_log("Emulation = %x\r\n\r\n",mpuParams.debugConfig);
    }



    /* region count and region configuration array is generated via sysconfig and is used
        for populating  FirewallReqObj*/
    FirewallReqObj.regionCount = FIREWALL_ARRAY1_NUM_REGIONS;
    FirewallReqObj.FirewallRegionArr = gMpuFirewallRegionConfig_1;

     /* This requests configures user given firewall request-2 in syscfg and and allow
        all permissions to all R5 cores except R5FSS0_0 */
    status = HsmClient_setFirewall(client,&FirewallReqObj,SystemP_WAIT_FOREVER);

    DebugP_log("Firewall request #2 status = "BYTE_TO_BINARY_PATTERN" "BYTE_TO_BINARY_PATTERN "\r\n",
        BYTE_TO_BINARY(FirewallReqObj.statusFirewallRegionArr>>8), BYTE_TO_BINARY(FirewallReqObj.statusFirewallRegionArr));

    /* MPU_FIREWALL_getRegion is API from firewall driver for fetching MPU region configuration */
    for(regionCount = 0U; regionCount < FIREWALL_ARRAY1_NUM_REGIONS; regionCount++)
    {
        mpuParams.id = gMpuFirewallRegionConfig_1[regionCount].firewallId;
        mpuParams.regionNumber = gMpuFirewallRegionConfig_1[regionCount].region;

        status = MPU_FIREWALL_getRegion(&mpuParams);
        DebugP_assert(status == FWL_DRV_RETURN_SUCCESS);

        DebugP_log("Firewall Id = %d\r\n",mpuParams.id);
        DebugP_log("Firewall region number = %x\r\n",mpuParams.regionNumber);
        DebugP_log("Start Address = 0x%x\r\n",mpuParams.startAddress);
        DebugP_log("End Address = 0x%x\r\n",mpuParams.endAddress);
        DebugP_log("Aid Config = 0x%x\r\n",mpuParams.aidConfig);
        DebugP_log("Supervisor Read = %x\r\n",mpuParams.supervisorReadConfig);
        DebugP_log("Supervisor Write = %x\r\n",mpuParams.supervisorWriteConfig);
        DebugP_log("Supervisor Execute = %x\r\n",mpuParams.supervisorExecConfig);
        DebugP_log("User Read = %x\r\n",mpuParams.userReadConfig);
        DebugP_log("User Write = %x\r\n",mpuParams.userWriteConfig);
        DebugP_log("User Execute = %x\r\n",mpuParams.userExecConfig);
        DebugP_log("Non Secure Access = %x\r\n",mpuParams.nonSecureConfig);
        DebugP_log("Emulation = %x\r\n\r\n",mpuParams.debugConfig);
    }
}