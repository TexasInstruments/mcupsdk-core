/*
 *   Copyright (c) Texas Instruments Incorporated 2023
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
 *
 */

 /**
 *  \file     ecc_main.c
 *
 * \brief This file demonstrates using the Error Correcting Code Module (ECC),
 *         utilizing the ECC and ESM Software Diagnostic Reference (SDL) functions.
 *
 *  \details  ESM Safety Example module tests
 **/

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <drivers/sciclient.h>
#if defined (R5F_CORE)
#include <drivers/pcie.h>
#include <drivers/hw_include/cslr_serdes.h>
#include <drivers/hw_include/cslr_serdes_pcie.h>
#include <kernel/dpl/AddrTranslateP.h>
#endif
#include <sdl/include/sdl_types.h>
#include <sdl/sdl_ecc.h>
#include <dpl_interface.h>
#include <kernel/dpl/DebugP.h>
#include <unity.h>
#include "ecc_func.h"


/* ========================================================================== */
/*                                Macros                                      */
/* ========================================================================== */



/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
volatile bool esmError = false;
/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */
void ECC_func_app(void *args);
/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/* ========================================================================== */
/*                          EXternal Function Definitions                              */
/* ========================================================================== */
extern int32_t ECC_funcTest(void);
/* ========================================================================== */
/*                 Internal Function Definitions                              */
/* ========================================================================== */

#ifdef UNITY_INCLUDE_CONFIG_H
/*
 *  ======== Unity set up and tear down ========
 */
void setUp(void)
{
    /* Do nothing */
}

void tearDown(void)
{
    /* Do nothing */
}
#endif

int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInst,
                                            SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,
                                            uint32_t index,
                                            uint32_t intSrc,
                                            uintptr_t *arg)
{

    SDL_ECC_MemType eccmemtype;
    SDL_Ecc_AggrIntrSrc eccIntrSrc;
    SDL_ECC_ErrorInfo_t eccErrorInfo;
    int32_t retVal;

    DebugP_log("\r\n  ESM Call back function called : instType 0x%x, intType 0x%x, " \
                "grpChannel 0x%x, index 0x%x, intSrc 0x%x \n",
                esmInst, esmIntrType, grpChannel, index, intSrc);
    DebugP_log("\r  Take action \n");
    if(esmIntrType == 1u){
        DebugP_log("\r High Priority Interrupt Executed\n");
    }
    else{
        DebugP_log("\r Low Priority Interrupt Executed\n");
    }
    retVal = SDL_ECC_getESMErrorInfo(esmInst, intSrc, &eccmemtype, &eccIntrSrc);
    if (retVal != SDL_PASS)
    {
        DebugP_log("\rSDL_ECC_getESMErrorInfo failed\n");
    }
    else
    {
        /* Any additional customer specific actions can be added here */
        retVal = SDL_ECC_getErrorInfo(eccmemtype, eccIntrSrc, &eccErrorInfo);
    }

    if (retVal == SDL_PASS)
    {
        DebugP_log("\r\n  ECC Error Call back function called : eccMemType %d, errorSrc 0x%x, " \
                   "ramId %d, bitErrorOffset 0x%04x%04x, bitErrorGroup %d\n",
                   eccmemtype, eccIntrSrc, eccErrorInfo.memSubType, (uint32_t)(eccErrorInfo.bitErrorOffset >> 32),
                   (uint32_t)(eccErrorInfo.bitErrorOffset & 0x00000000FFFFFFFF), eccErrorInfo.bitErrorGroup);
        if (eccErrorInfo.injectBitErrCnt != 0)
        {
            SDL_ECC_clearNIntrPending(eccmemtype, eccErrorInfo.memSubType, eccIntrSrc, SDL_ECC_AGGR_ERROR_SUBTYPE_INJECT, eccErrorInfo.injectBitErrCnt);
        }
        if (eccErrorInfo.bitErrCnt != 0)
	{
            SDL_ECC_clearNIntrPending(eccmemtype, eccErrorInfo.memSubType, eccIntrSrc, SDL_ECC_AGGR_ERROR_SUBTYPE_NORMAL, eccErrorInfo.bitErrCnt);
        }
        SDL_ECC_clearNIntrPending(eccmemtype, eccErrorInfo.memSubType, eccIntrSrc, SDL_ECC_AGGR_ERROR_SUBTYPE_NORMAL, 0x1);

        retVal = SDL_ECC_ackIntr(eccmemtype, eccIntrSrc);
    }
    esmError = true;

    return retVal;
}

/* SDL_ECC_applicationCallbackFunction is expected to be defined by the application. It is
 * required by the SDL ECC module. It is called by the SDL ECC module to notify the
 * application of certain ECC errors that are reported as Exception events.
 * Note, however, that it is not executed in this example */
void SDL_ECC_applicationCallbackFunction(SDL_ECC_MemType eccMemType,
                                         uint32_t errorSrc,
                                         uint32_t address,
                                         uint32_t ramId,
                                         uint64_t bitErrorOffset,
                                         uint32_t bitErrorGroup)
{

    DebugP_log("\r\n  ECC Error Call back function called : eccMemType %d, errorSrc 0x%x, " \
                "address 0x%x, ramId %d, bitErrorOffset 0x%04x%04x, bitErrorGroup %d\n",
                eccMemType, errorSrc, address, ramId, (uint32_t)(bitErrorOffset >> 32),
                (uint32_t)(bitErrorOffset & 0x00000000FFFFFFFF), bitErrorGroup);
    DebugP_log("\r  Take action \n");

    /* Any additional customer specific actions can be added here */

}

#define AUX_NUM_DEVICES 29

int32_t status = SDL_PASS;
uint32_t aux_devices[AUX_NUM_DEVICES] =
{
  TISCI_DEV_MMCSD1,
  TISCI_DEV_COMPUTE_CLUSTER0,
  TISCI_DEV_COMPUTE_CLUSTER0_PBIST_0,
  TISCI_DEV_ADC0,
  TISCI_DEV_SA2_UL0,
  TISCI_DEV_MCAN0,
  TISCI_DEV_DMASS0,
  TISCI_DEV_MCAN1,
  TISCI_DEV_PRU_ICSSG1,
  TISCI_DEV_PRU_ICSSG0,
  TISCI_DEV_FSS0_OSPI_0,
  TISCI_DEV_CPSW0,
  TISCI_DEV_GICSS0,
  TISCI_DEV_PCIE0,
  TISCI_DEV_USB0,
  TISCI_DEV_DMSC0,
  TISCI_DEV_MCU_M4FSS0,
  TISCI_DEV_MCU_M4FSS0_CORE0,
  TISCI_DEV_MMCSD0,
  TISCI_DEV_VTM0,
  TISCI_DEV_R5FSS0_CORE0,
  TISCI_DEV_R5FSS0_CORE1,
  TISCI_DEV_R5FSS1_CORE0,
  TISCI_DEV_R5FSS1_CORE1,
  TISCI_DEV_A53SS0,
  TISCI_DEV_A53SS0_CORE_0,
  TISCI_DEV_A53SS0_CORE_1,
  TISCI_DEV_TIMERMGR0,
  TISCI_DEV_SERDES_10G0
};

static int32_t sdlApp_dplInit(void)
{
    SDL_ErrType_t ret = SDL_PASS;

    ret = SDL_TEST_dplInit();
    if (ret != SDL_PASS)
    {
        DebugP_log("\rError: Init Failed\n");
    }

    for (int i = 0; i < AUX_NUM_DEVICES; i++)
    {
        /* Power up RTI */
        status = Sciclient_pmSetModuleState(aux_devices[i],
                                            TISCI_MSG_VALUE_DEVICE_SW_STATE_ON,
                                            TISCI_MSG_FLAG_AOP,
                                            SystemP_WAIT_FOREVER);

        if (status != SDL_PASS)
        {
            DebugP_log("\r   Sciclient_pmSetModuleState 0x%x ...FAILED: retValue %d\n",
                        aux_devices[i], status);
        }
    }

    return ret;
}

void ECC_func_app(void *args)
{
    int32_t    testResult;
    testResult = ECC_funcTest();
    DebugP_log("\r\n ECC func Test");
    if (testResult == SDL_PASS)
    {
        DebugP_log("\r\nAll test have passed.\n");
    }
    else
    {
        DebugP_log("\r\nSome test have failed.\n");
    }
}


#if defined(R5F_CORE) /* PCIE/Serdes driver is only supported from R5F core */
#define SERDES_LANE_MASK  (0x3U)

/* Minimal init taken from Drivers_open() for PCIE */
void sdlApp_initSerdes(void)
{
    CSL_SerdesLaneEnableParams serdesLaneEnableParams;
    uint32_t i, laneNum;

    /* Since AM64x has a single PCIe instance initialize serdes corresponding to it */
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, 2);

    memset(&serdesLaneEnableParams, 0, sizeof(serdesLaneEnableParams));
    serdesLaneEnableParams.serdesInstance = (CSL_SerdesInstance)CSL_TORRENT_SERDES0;

    /* For SERDES0 */
    serdesLaneEnableParams.baseAddr         = (uint32_t)AddrTranslateP_getLocalAddr(CSL_SERDES_10G0_BASE);
    serdesLaneEnableParams.refClock         = CSL_SERDES_REF_CLOCK_100M;
    serdesLaneEnableParams.refClkSrc        = CSL_SERDES_REF_CLOCK_INT;
    serdesLaneEnableParams.linkRate         = CSL_SERDES_LINK_RATE_3p125G;
    serdesLaneEnableParams.numLanes         = 1;
    serdesLaneEnableParams.laneMask         = SERDES_LANE_MASK;
    serdesLaneEnableParams.SSC_mode         = CSL_SERDES_NO_SSC;
    serdesLaneEnableParams.phyType          = CSL_SERDES_PHY_TYPE_PCIe;
    serdesLaneEnableParams.pcieGenType      = PCIE_GEN1;
    serdesLaneEnableParams.operatingMode    = CSL_SERDES_FUNCTIONAL_MODE;
    serdesLaneEnableParams.phyInstanceNum   = 0;
    serdesLaneEnableParams.refClkOut        = CSL_SERDES_REFCLK_OUT_EN;

    for(i = 0; i< serdesLaneEnableParams.numLanes; i++)
    {
        serdesLaneEnableParams.laneCtrlRate[i] = CSL_SERDES_LANE_FULL_RATE;
        serdesLaneEnableParams.loopbackMode[i] = CSL_SERDES_LOOPBACK_DISABLED; /* still have to change to correct loopback mode */
    }
    CSL_serdesPorReset(serdesLaneEnableParams.baseAddr);

    /* Select the IP type, IP instance num, Serdes Lane Number */
    for (laneNum = 0; laneNum < serdesLaneEnableParams.numLanes; laneNum++)
    {
        CSL_serdesIPSelect((uint32_t)AddrTranslateP_getLocalAddr(CSL_CTRL_MMR0_CFG0_BASE),
                           serdesLaneEnableParams.phyType,
                           serdesLaneEnableParams.phyInstanceNum,
                           serdesLaneEnableParams.serdesInstance,
                           laneNum);
    }

    /* selects the appropriate clocks for all serdes based on the protocol chosen */
    status = CSL_serdesRefclkSel((uint32_t)AddrTranslateP_getLocalAddr(CSL_CTRL_MMR0_CFG0_BASE),
                                  serdesLaneEnableParams.baseAddr,
                                  serdesLaneEnableParams.refClock,
                                  serdesLaneEnableParams.refClkSrc,
                                  serdesLaneEnableParams.serdesInstance,
                                  serdesLaneEnableParams.phyType);

    /* Assert PHY reset and disable all lanes */
    CSL_serdesDisablePllAndLanes(serdesLaneEnableParams.baseAddr, serdesLaneEnableParams.numLanes, serdesLaneEnableParams.laneMask);

    /*Load the Serdes Config File */
    status = CSL_serdesPCIeInit(&serdesLaneEnableParams); /* Use this for PCIe serdes config load */

    /* Set this to standard mode defined by Cadence */
    for (laneNum=0; laneNum < serdesLaneEnableParams.numLanes; laneNum++)
    {
        CSL_serdesPCIeModeSelect(serdesLaneEnableParams.baseAddr, serdesLaneEnableParams.pcieGenType, laneNum);
    }

    /* Common Lane Enable API for lane enable, pll enable etc */
    status = CSL_serdesLaneEnable(&serdesLaneEnableParams);
}
#endif

void ecc_app_runner(void)
{
#if defined(UNITY_INCLUDE_CONFIG_H)
    UNITY_BEGIN();
    RUN_TEST(ECC_func_app,0, NULL);
    UNITY_END();
#else
    ECC_func_app();
#endif
}


int32_t test_main(void)
{
#if defined (R5F_CORE)
    sdlApp_initSerdes();
#endif

    sdlApp_dplInit();
    ecc_app_runner();

    return (0);
}

/* Nothing past this point */
