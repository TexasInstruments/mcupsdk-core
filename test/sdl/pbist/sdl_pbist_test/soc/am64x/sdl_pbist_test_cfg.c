/*
 *   Copyright (c) Texas Instruments Incorporated 2022
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
 *  \file     sdl_pbist_test_cfg.c
 *
 *  \brief    This file contains PBIST test configuration
 *
 *  \details  PBIST Test Configuration
 **/

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>
#include <sdl/include/sdl_types.h>
#include <sdl/sdl_pbist.h>
#include <drivers/sciclient.h>
#include <kernel/dpl/AddrTranslateP.h>

#include "power_seq.h"
#include "pbist_test_cfg.h"

/* ========================================================================== */
/*                                Macros                                      */
/* ========================================================================== */


/* ========================================================================== */
/*                            Local function prototypes                       */
/* ========================================================================== */
/*
    InitRestore functions : Initialize or Restore based on init flag
    init : TRUE  --> Initialize
    init : FALSE --> Restore
*/
int32_t PBIST_MPUAuxInitRestore(bool init);
int32_t PBIST_InfraAuxInitRestore(bool init);
int32_t PBIST_McuAuxInitRestore(bool init);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
uint32_t PBIST_MPUAuxDevList[MPU_NUM_AUX_DEVICES] =
{
    TISCI_DEV_COMPUTE_CLUSTER0,
    TISCI_DEV_A53SS0,
};

uint32_t PBIST_InfraAuxDevList[INFRA_NUM_AUX_DEVICES] =
{
    TISCI_DEV_DEBUGSS_WRAP0,
    TISCI_DEV_MCAN0,
    TISCI_DEV_MCAN1,
    TISCI_DEV_CPSW0,
    TISCI_DEV_USB0,
    TISCI_DEV_PCIE0,
    TISCI_DEV_MMCSD0,
    TISCI_DEV_MMCSD1,
    TISCI_DEV_SA2_UL0,
    TISCI_DEV_PRU_ICSSG0,
    TISCI_DEV_PRU_ICSSG1,
};

uint32_t PBIST_McuAuxDevList[MCU_NUM_AUX_DEVICES] =
{
    TISCI_DEV_ADC0,
    TISCI_DEV_MMCSD0,
    TISCI_DEV_MMCSD1,
};

PBIST_TestHandle_t PBIST_TestHandleArray[PBIST_MAX_INSTANCE+1] =
{
#if defined (M4F_CORE)
    /* Pulsar Instance 0 */
    {
        .testName               = "Pulsar Instance 0",
        .pbistInst              = SDL_PBIST_INST_R5F0,
        .numPostPbistToCheck    = 0u,
        .tisciPBISTDeviceId     = TISCI_DEV_PBIST2,    /* PBIST device id  */
        .procRstNeeded          = true,
        .secondaryCoreNeeded    = true,                /* Secondary core needed */
        .coreName               = "Main R5F0 core0",   /* Primary core   */
        .secCoreName            = "Main R5F0 core1",   /* Secondary core */
        .tisciProcId            = SCICLIENT_PROCID_R5_CL0_C0, /* Main R5F core 0 Proc Id */
        .tisciSecProcId         = SCICLIENT_PROCID_R5_CL0_C1, /* Main R5F core 1 Proc Id */
        .tisciDeviceId          = TISCI_DEV_R5FSS0_CORE0,   /* Main R5F core 0 Device Id */
        .tisciSecDeviceId       = TISCI_DEV_R5FSS0_CORE1,   /* Main R5F core 1 Device Id */
        .coreCustPwrSeqNeeded   = false,
        .numAuxDevices          = 0u,                   /* No Aux devices */
        .auxInitRestoreFunction = NULL,                 /* Auxilliary init function */
    },
#endif
    /* Pulsar Instance 1 */
    {
        .testName               = "Pulsar Instance 1",
        .pbistInst              = SDL_PBIST_INST_R5F1,
        .numPostPbistToCheck    = 0u,
        .tisciPBISTDeviceId     = TISCI_DEV_PBIST3,    /* PBIST device id  */
        .procRstNeeded          = true,
        .secondaryCoreNeeded    = true,                /* Secondary core needed */
        .coreName               = "Main R5F1 core0",   /* Primary core   */
        .secCoreName            = "Main R5F1 core1",   /* Secondary core */
        .tisciProcId            = SCICLIENT_PROCID_R5_CL1_C0, /* Main R5F core 0 Proc Id */
        .tisciSecProcId         = SCICLIENT_PROCID_R5_CL1_C1, /* Main R5F core 1 Proc Id */
        .tisciDeviceId          = TISCI_DEV_R5FSS1_CORE0,   /* Main R5F core 0 Device Id */
        .tisciSecDeviceId       = TISCI_DEV_R5FSS1_CORE1,   /* Main R5F core 1 Device Id */
        .coreCustPwrSeqNeeded   = false,
        .numAuxDevices          = 0u,                   /* No Aux devices */
        .auxInitRestoreFunction = NULL,                 /* Auxilliary init function */
    },
    /* MPU */
    {
        .testName               = "MPU PBIST",
        .pbistInst              = SDL_PBIST_INST_MPU,
        .numPostPbistToCheck    = 0u,
        .tisciPBISTDeviceId     = TISCI_DEV_COMPUTE_CLUSTER0_PBIST_0, /* Device Id for A72 PBIST */
        .procRstNeeded          = false,
        .secondaryCoreNeeded    = true,                /* Secondary core needed */
        .coreName               = "MPU core 0",        /* Primary core   */
        .secCoreName            = "MPU core 1",        /* Secondary core */
        .tisciProcId            = SCICLIENT_PROCID_A53_CL0_C0,  /* A53 core 0 Proc Id */
        .tisciSecProcId         = SCICLIENT_PROCID_A53_CL0_C1,  /* A53 core 1 Proc Id */
        .tisciDeviceId          = TISCI_DEV_A53SS0_CORE_0,      /* A53 core 0 Device Id */
        .tisciSecDeviceId       = TISCI_DEV_A53SS0_CORE_1,      /* A53 core 1 Device Id */
        .coreCustPwrSeqNeeded   = true,                    /* MPU needs custom powerdown sequence steps */
        .numAuxDevices          = MPU_NUM_AUX_DEVICES,     /* Number of Aux devices   */
        .auxDeviceIdsP          = &PBIST_MPUAuxDevList[0], /* Array of Aux device ids */
        .auxInitRestoreFunction = PBIST_MPUAuxInitRestore, /* Auxilliary init function */
    },
    /* Infra */
    {
        .testName               = "Infra PBIST",
        .pbistInst              = SDL_PBIST_INST_INFRA,
        .numPostPbistToCheck    = 0u,
        .tisciPBISTDeviceId     = TISCI_DEV_PBIST0,    /* PBIST device id  */
        .procRstNeeded          = false,
        .secondaryCoreNeeded    = false,               /* Secondary core needed */
        .coreName               = "Infra",             /* No coreName   */
        .tisciProcId            = 0x0u,                /* No Proc Id needed for Main Intrastructure */
        .tisciDeviceId          = 0x0u,                /* No Device Id needed for Main infrastructure */
        .coreCustPwrSeqNeeded   = false,
        .numAuxDevices          = INFRA_NUM_AUX_DEVICES,    /* No Aux devices */
        .auxDeviceIdsP          = &PBIST_InfraAuxDevList[0], /* Array of Aux device ids */
        .auxInitRestoreFunction = PBIST_InfraAuxInitRestore, /* Auxilliary init function */
    },
    /* MCU PBIST */
    {
        .testName               = "MCU PBIST",
        .pbistInst              = SDL_PBIST_INST_MCU,
        .numPostPbistToCheck    = 0u,
        .tisciPBISTDeviceId     = TISCI_DEV_PBIST1,     /* PBIST device id  */
        .procRstNeeded          = true,
        .secondaryCoreNeeded    = false,                /* Secondary core needed */
        .coreName               = "M4F core",           /* Primary core */
        .tisciProcId            = SCICLIENT_PROCID_MCU_M4FSS0_C0, /* M4FSS Core */
        .tisciDeviceId          = TISCI_DEV_MCU_M4FSS0_CORE0,     /* M4FSS Core */
        .coreCustPwrSeqNeeded   = false,
        .numAuxDevices          = MCU_NUM_AUX_DEVICES,     /* No Aux devices */
        .auxDeviceIdsP          = &PBIST_McuAuxDevList[0], /* Array of Aux device ids */
        .auxInitRestoreFunction = PBIST_McuAuxInitRestore, /* Auxilliary init function */
    },
};

/* Captures common Initialization: currently nothing needed */
int32_t PBIST_commonInit(void)
{
    SDL_ErrType_t status = SDL_PASS;

    return status;
}

/* define the unlock and lock values */
#define KICK0_UNLOCK_VAL 0x68EF3490
#define KICK1_UNLOCK_VAL 0xD172BC5A
#define KICK_LOCK_VAL    0x00000000

/*
    InitRestore functions : Initialize or Restore based on init flag
    init : TRUE  --> Initialize
    init : FALSE --> Restore
*/
int32_t PBIST_MPUAuxInitRestore(bool init)
{
    int32_t testResult = 0;
    uint32_t baseAddr;

    baseAddr = (uint32_t) AddrTranslateP_getLocalAddr(CSL_CTRL_MMR0_CFG0_BASE);

    *((uint32_t *)(((uint32_t)baseAddr) + CSL_MAIN_CTRL_MMR_CFG0_LOCK4_KICK0)) = KICK0_UNLOCK_VAL;
    *((uint32_t *)(((uint32_t)baseAddr) + CSL_MAIN_CTRL_MMR_CFG0_LOCK4_KICK1)) = KICK1_UNLOCK_VAL;
    /* BISOR override is needed to verify DC memories in cfg */
    if (init)
    {
        *((uint32_t *)(((uint32_t)baseAddr) + CSL_MAIN_CTRL_MMR_CFG0_A53SS_DFT_CTL)) = 0x1;
    }
    else
    {
        *((uint32_t *)(((uint32_t)baseAddr) + CSL_MAIN_CTRL_MMR_CFG0_A53SS_DFT_CTL)) = 0x0;
    }

    return testResult;
}

int32_t PBIST_InfraAuxInitRestore(bool init)
{
    int32_t testResult = 0;
    uint32_t baseAddr, baseAddrMcu;

    baseAddr = (uint32_t) AddrTranslateP_getLocalAddr(CSL_CTRL_MMR0_CFG0_BASE);
    baseAddrMcu = (uint32_t) AddrTranslateP_getLocalAddr(CSL_MCU_CTRL_MMR0_CFG0_BASE);
    *((uint32_t *)(baseAddr + CSL_MAIN_CTRL_MMR_CFG0_LOCK3_KICK0)) = KICK0_UNLOCK_VAL;
    *((uint32_t *)(baseAddr + CSL_MAIN_CTRL_MMR_CFG0_LOCK3_KICK1)) = KICK1_UNLOCK_VAL;
    *((uint32_t *)(baseAddrMcu + CSL_MCU_CTRL_MMR_CFG0_LOCK6_KICK0)) = KICK0_UNLOCK_VAL;
    *((uint32_t *)(baseAddrMcu + CSL_MCU_CTRL_MMR_CFG0_LOCK6_KICK1)) = KICK1_UNLOCK_VAL;
    *((uint32_t *)(baseAddr + CSL_MAIN_CTRL_MMR_CFG0_LOCK2_KICK0)) = KICK0_UNLOCK_VAL;
    *((uint32_t *)(baseAddr + CSL_MAIN_CTRL_MMR_CFG0_LOCK2_KICK1)) = KICK1_UNLOCK_VAL;

    if (init)
    {
        *((uint32_t *)(baseAddr + CSL_MAIN_CTRL_MMR_CFG0_PBIST_EN)) = 0x113;
        *((uint32_t *)(baseAddrMcu + CSL_MCU_CTRL_MMR_CFG0_MAIN_CLKGATE_CTRL0)) = 0x90360000;
        *((uint32_t *)(baseAddr + CSL_MAIN_CTRL_MMR_CFG0_ICSSG0_CLKSEL)) = 0x1;
        *((uint32_t *)(baseAddr + CSL_MAIN_CTRL_MMR_CFG0_ICSSG1_CLKSEL)) = 0x1;
    }
    else
    {
        *((uint32_t *)(baseAddr + CSL_MAIN_CTRL_MMR_CFG0_PBIST_EN)) = 0x0;
    }

    return testResult;
}

int32_t PBIST_McuAuxInitRestore(bool init)
{
    int32_t testResult = 0;
    uint32_t baseAddr, baseAddrMcu;

    baseAddr = (uint32_t) AddrTranslateP_getLocalAddr(CSL_CTRL_MMR0_CFG0_BASE);
    baseAddrMcu = (uint32_t) AddrTranslateP_getLocalAddr(CSL_MCU_CTRL_MMR0_CFG0_BASE);

    *((uint32_t *)(baseAddr + CSL_MAIN_CTRL_MMR_CFG0_PBIST_EN)) = 0x1;
    *((uint32_t *)(baseAddrMcu + CSL_MCU_CTRL_MMR_CFG0_MAIN_CLKGATE_CTRL0)) = 0x01018000;

    return testResult;
}

/* Nothing past this point */
