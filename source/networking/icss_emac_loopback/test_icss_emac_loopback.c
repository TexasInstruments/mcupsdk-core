/*
 * Copyright (C) 2021 Texas Instruments Incorporated
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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <string.h>
#include "ti_drivers_config.h"
#include "test_icss_emac_loopback_utils.h"
#include "tiemac_pruicss_intc_mapping.h"
#include <firmware/icss_emac_mmap.h>
#include <kernel/dpl/TaskP.h>
#include <networking/icss_emac/icss_emac.h>
#include <kernel/dpl/MpuP_armv7.h>
#include <kernel/dpl/CacheP.h>
#include <drivers/hw_include/hw_types.h>
#include "ti_drivers_open_close.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define TEST_TASK_PRIORITY            (8)
#define TEST_TASK_STACK_SIZE          (0x1000)

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

int32_t ICSS_EMAC_testPruicssInstanceSetup(void);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

uint8_t ICSS_EMAC_testEvmType = ICSS_EMAC_TEST_BOARD_AM64x;

PRUICSS_Handle ICSS_EMAC_testPruIcssHandle1 = NULL;
PRUICSS_Handle ICSS_EMAC_testPruIcssHandle2 = NULL;

extern ICSS_EMAC_Handle ICSS_EMAC_testHandle2;
extern ICSS_EMAC_Handle ICSS_EMAC_testHandle3;

extern uint8_t ICSS_EMAC_testLclMac2[];
extern uint8_t ICSS_EMAC_testLclMac3[];

extern PRUICSS_Config gPruicssConfig[2];
extern ETHPHY_Handle gEthPhyHandle[2];

uint32_t gTestTaskStack[TEST_TASK_STACK_SIZE/sizeof(uint32_t)] __attribute__((aligned(32)));

TaskP_Object testTaskObject;

extern ICSS_EMAC_FwStaticMmap icss_emacFwStaticCfg[2];
extern ICSS_EMAC_FwDynamicMmap icss_emacFwDynamicCfg;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void test_main(void *args)
{
    TaskP_Params    taskParams;
    uint32_t        status = SystemP_FAILURE;
    uint32_t        icssBaseAddr;

    Drivers_open();
    status = Board_driversOpen();
    DebugP_assert(status==SystemP_SUCCESS);

    ICSS_EMAC_testBoardInit();

    ICSS_EMAC_testPruicssInstanceSetup();

    /*Setup RAT configuration for buffer region*/

    /* Setting up RAT config to map emacBaseAddr->l3OcmcBaseAddr to C30 constant of PRUICSS */
    /* Mapping 0xE0000000 (C30 constant of PRUICSS) to l3OcmcBaseAddr */
    icssBaseAddr = (uint32_t)((PRUICSS_HwAttrs *)(ICSS_EMAC_testPruIcssHandle2->hwAttrs)->baseAddr);

    HW_WR_REG32(icssBaseAddr + CSL_ICSS_RAT_REGS_0_BASE + 0x24, (0xE0000000)); //rat0 base0
    HW_WR_REG32(icssBaseAddr + CSL_ICSS_RAT_REGS_0_BASE + 0x28, (0x70000000)); //rat0 trans_low0
    HW_WR_REG32(icssBaseAddr + CSL_ICSS_RAT_REGS_0_BASE + 0x2C, (0x00000000)); //rat0 trans_low0
    HW_WR_REG32(icssBaseAddr + CSL_ICSS_RAT_REGS_0_BASE + 0x20, (1u << 31) | (22)); //rat0 ctrl0

    HW_WR_REG32(icssBaseAddr + CSL_ICSS_RAT_REGS_1_BASE + 0x24, (0xE0000000)); //rat0 base0
    HW_WR_REG32(icssBaseAddr + CSL_ICSS_RAT_REGS_1_BASE + 0x28, (0x70000000)); //rat0 trans_low0
    HW_WR_REG32(icssBaseAddr + CSL_ICSS_RAT_REGS_1_BASE + 0x2C, (0x00000000)); //rat0 trans_low0
    HW_WR_REG32(icssBaseAddr + CSL_ICSS_RAT_REGS_1_BASE + 0x20, (1u << 31) | (22)); //rat0 ctrl0

    TaskP_Params_init(&taskParams);
    taskParams.name = "TestTask";
    taskParams.stackSize = TEST_TASK_STACK_SIZE;
    taskParams.stack = (uint8_t *)gTestTaskStack;
    taskParams.priority = TEST_TASK_PRIORITY;
    taskParams.taskMain = (TaskP_FxnMain)ICSS_EMAC_testTask;
    status = TaskP_construct(&testTaskObject, &taskParams);

    if(status != SystemP_SUCCESS)
    {
        DebugP_log("Test Task Creation failed\r\n");
    }
    return;
}

int32_t ICSS_EMAC_testPruicssInstanceSetup(void)
{
    PRUICSS_IntcInitData    pruss_intc_initdata = PRUSS_INTC_INITDATA;
    ICSS_EMAC_Params        icssEmacParams;

    ICSS_EMAC_testPruIcssHandle2 = PRUICSS_open(CONFIG_PRU_ICSS1);

    /*PRU2 ETH0 initializations*/
    ICSS_EMAC_Params_init(&icssEmacParams);
    icssEmacParams.pruicssIntcInitData = &pruss_intc_initdata;
    icssEmacParams.fwStaticMMap = &(icss_emacFwStaticCfg[1]);
    icssEmacParams.fwDynamicMMap = &icss_emacFwDynamicCfg;
    icssEmacParams.fwVlanFilterParams = &icss_emacFwVlanFilterCfg;
    icssEmacParams.fwMulticastFilterParams = &icss_emacFwMulticastFilterCfg;
    icssEmacParams.pruicssHandle = ICSS_EMAC_testPruIcssHandle2;
    icssEmacParams.callBackObject.port0LinkCallBack.callBack = (ICSS_EMAC_CallBack)ICSS_EMAC_testLinkIsrCb;
    icssEmacParams.callBackObject.port0LinkCallBack.userArg = (void*)ICSS_EMAC_TEST_PRU2ETH0;
    icssEmacParams.callBackObject.rxNRTCallBack.callBack = (ICSS_EMAC_CallBack)ICSS_EMAC_testCallbackRxPacket2;
    icssEmacParams.callBackObject.txCallBack.callBack = (ICSS_EMAC_CallBack)ICSS_EMAC_testCallbackTxComplete;
    icssEmacParams.ethphyHandle[0] = gEthPhyHandle[CONFIG_ETHPHY0];
    memcpy(&(icssEmacParams.macId[0]), &(ICSS_EMAC_testLclMac2[0]), 6);
    ICSS_EMAC_testHandle2 = ICSS_EMAC_open(CONFIG_ICSS_EMAC0, &icssEmacParams);
    DebugP_assert(ICSS_EMAC_testHandle2 != NULL);

    /* Test ICSS_EMAC_DeInit */
    ICSS_EMAC_close(ICSS_EMAC_testHandle2);
    ICSS_EMAC_testHandle2 = ICSS_EMAC_open(0, &icssEmacParams);

    /*PRU2 ETH1 initializations*/
    ICSS_EMAC_Params_init(&icssEmacParams);
    icssEmacParams.pruicssIntcInitData = &pruss_intc_initdata;
    icssEmacParams.fwStaticMMap = &(icss_emacFwStaticCfg[1]);
    icssEmacParams.fwDynamicMMap = &icss_emacFwDynamicCfg;
    icssEmacParams.fwVlanFilterParams = &icss_emacFwVlanFilterCfg;
    icssEmacParams.fwMulticastFilterParams = &icss_emacFwMulticastFilterCfg;
    icssEmacParams.pruicssHandle = ICSS_EMAC_testPruIcssHandle2;
    icssEmacParams.callBackObject.port1LinkCallBack.callBack = (ICSS_EMAC_CallBack)ICSS_EMAC_testLinkIsrCb;
    icssEmacParams.callBackObject.port1LinkCallBack.userArg = (void*)ICSS_EMAC_TEST_PRU2ETH1;
    icssEmacParams.callBackObject.rxNRTCallBack.callBack = (ICSS_EMAC_CallBack)ICSS_EMAC_testCallbackRxPacket3;
    icssEmacParams.callBackObject.txCallBack.callBack = (ICSS_EMAC_CallBack)ICSS_EMAC_testCallbackTxComplete;
    icssEmacParams.ethphyHandle[0] = gEthPhyHandle[CONFIG_ETHPHY1];
    memcpy(&(icssEmacParams.macId[0]), &(ICSS_EMAC_testLclMac3[0]), 6);

    ICSS_EMAC_testHandle3 = ICSS_EMAC_open(CONFIG_ICSS_EMAC1, &icssEmacParams);
    DebugP_assert(ICSS_EMAC_testHandle3 != NULL);

    /* Test ICSS_EMAC_DeInit */
    ICSS_EMAC_close(ICSS_EMAC_testHandle3);
    ICSS_EMAC_testHandle3 = ICSS_EMAC_open(3 , &icssEmacParams);

    ICSS_EMAC_testTimerSetup(ICSS_EMAC_testHandle2);

    return 0;
}
