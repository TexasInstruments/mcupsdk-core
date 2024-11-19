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
/*                            Global Variables                                */
/* ========================================================================== */

static HwiP_Object          gMpuFirewallHwiObject;
static SemaphoreP_Object    gSemObject;
HsmClient_t                 gHsmClient ;

#if defined (SOC_AM263X) || defined (SOC_AM263PX)
/* main core that starts the message exchange */
uint32_t gMainCoreId = CSL_CORE_ID_R5FSS0_0;
/* remote cores that echo messages from main core, make sure to NOT list main core in this list */
uint32_t gRemoteCoreId[] = {
    CSL_CORE_ID_R5FSS0_1,
    CSL_CORE_ID_R5FSS1_0,
    CSL_CORE_ID_R5FSS1_1,
    CSL_CORE_ID_MAX /* this value indicates the end of the array */
};
#endif

#if defined (SOC_AM261X)
/* main core that starts the message exchange */
uint32_t gMainCoreId = CSL_CORE_ID_R5FSS0_0;
/* remote cores that echo messages from main core, make sure to NOT list main core in this list */
uint32_t gRemoteCoreId[] = {
    CSL_CORE_ID_R5FSS0_0,
    CSL_CORE_ID_R5FSS0_1,
    CSL_CORE_ID_MAX /* this value indicates the end of the array */
};
#endif

#define APP_CLIENT_ID                   (0x02)
#define IPC_CLIENT_ID                   (0x02)
#define IPC_MSG_VAL                     (0xFF)
#define MPU_INT_ENABLED_STATUS_CLEAR    (0x03)
#define MPU_INT_FAULT_CLEAR             (0x01)
#define MPU_INT_ENABLE                  (0x03)

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void remote_core_handler(uint32_t remoteCoreId, uint16_t localClientId, uint32_t msgValue, int32_t crcStatus, void *args)
{
    /*Read CSL_UART0_U_BASE register */
    CSL_REG32_RD((uint32_t *) CSL_UART0_U_BASE);
}

void host_core_handler(uint32_t remoteCoreId, uint16_t localClientId, uint32_t msgValue, int32_t crcStatus, void *args)
{
    /* empty */
}

void host_core_exception_isr()
{
    /* struct instance used for sending firewall interrupt clear/enable request */
    FirewallIntrReq_t FirewallIntrReqObj;
    int32_t status ;

    /* Clearing the fault registers and raw status in one request */
    FirewallIntrReqObj.firewallId = CSL_FW_R5SS0_CORE1_AHB_MST_ID;
    FirewallIntrReqObj.interruptEnableStatusClear = MPU_INT_ENABLED_STATUS_CLEAR;
    FirewallIntrReqObj.faultClear = MPU_INT_FAULT_CLEAR;
    status = HsmClient_FirewallIntr(&gHsmClient, &FirewallIntrReqObj, SystemP_WAIT_FOREVER);
    DebugP_assert(status==SystemP_SUCCESS);

    CSL_mss_ctrlRegs *ptrMSSCtrlRegs = (CSL_mss_ctrlRegs *)CSL_MSS_CTRL_U_BASE;

    /* clear interrupt(#70) from the processor */
    CSL_REG32_FINS_RAW (&ptrMSSCtrlRegs->MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS,
    CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_MASK_MPU_CR5B0_AHB_PROT_ERR_MASK,
    CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_MASK_MPU_CR5B0_AHB_PROT_ERR_SHIFT, 1U);

    /* clear raw status from the processor */
    CSL_REG32_FINS_RAW (&ptrMSSCtrlRegs->MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_RAW,
    CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_MASK_MPU_CR5B0_AHB_PROT_ERR_MASK,
    CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_MASK_MPU_CR5B0_AHB_PROT_ERR_SHIFT, 1U);

    SemaphoreP_post(&gSemObject);

}

void mpu_firewall_demo_main_core_start()
{
    HwiP_Params     hwiPrms;
    int32_t status ;

    /* Construct Semaphore */
    SemaphoreP_constructBinary(&gSemObject, 0);
    HwiP_Params_init(&hwiPrms);

    /* Register HSM Client */
    status = HsmClient_register(&gHsmClient,APP_CLIENT_ID);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Register Interrupt */
    hwiPrms.intNum      = CSLR_R5FSS0_CORE0_INTR_MPU_PROT_ERRAGG;
    hwiPrms.callback    = &host_core_exception_isr;
    hwiPrms.priority    = 15U;
    HwiP_construct(&gMpuFirewallHwiObject, &hwiPrms);

    DebugP_log("\r\n Interrupt registeration successful, notifying remote core to read CSL_UART0_U_BASE register \r\n");

    /* register a handler to receive messages in IPC */
    status = IpcNotify_registerClient(IPC_CLIENT_ID, host_core_handler, NULL);
    DebugP_assert(status==SystemP_SUCCESS);

    /* Notify remote core to read the UART0 register space) */
    IpcNotify_sendMsg(CSL_CORE_ID_R5FSS0_1, IPC_CLIENT_ID, IPC_MSG_VAL, 1);
    DebugP_assert(status==SystemP_SUCCESS);

    /* Semaphore is pending until remote core goes into fault and host core exits ISR */
    SemaphoreP_pend(&gSemObject, SystemP_WAIT_FOREVER);

    status = IpcNotify_unregisterClient(IPC_CLIENT_ID);

    DebugP_log("Success");

}

void mpu_firewall_demo_remote_core_start()
{
    int32_t status;
    IpcNotify_Params notifyParams;

    /* initialize parameters to default */
    IpcNotify_Params_init(&notifyParams);

    /* specify the core on which this API is called */
    notifyParams.selfCoreId = CSL_CORE_ID_R5FSS0_1;

    /* list the cores that will do IPC Notify with this core
     * Make sure to NOT list `self` core in the list below
     */
    notifyParams.numCores = 1;
    notifyParams.coreIdList[0] = CSL_CORE_ID_R5FSS0_0;

    status = IpcNotify_init(&notifyParams);
    DebugP_assert(status==SystemP_SUCCESS);

    /* register a handler to receive messages */
    status = IpcNotify_registerClient(IPC_CLIENT_ID, remote_core_handler, NULL);
    DebugP_assert(status==SystemP_SUCCESS);

    SemaphoreP_pend(&gSemObject, SystemP_WAIT_FOREVER);

    status = IpcNotify_unregisterClient(IPC_CLIENT_ID);
}

void mpu_firewall_demo_main(void *args)
{
    if(IpcNotify_getSelfCoreId()==gMainCoreId)
    {
        mpu_firewall_demo_main_core_start();
    }
    else
    {
        mpu_firewall_demo_remote_core_start();
    }
}



