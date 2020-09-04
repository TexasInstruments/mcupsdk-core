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
 * \file     enet_apputils_k3.c
 *
 * \brief    Common Enet application utility functions for K3 SOCs.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include <drivers/hw_include/cslr_soc.h>
#include <csl_cpswitch.h>

#include <enet.h>
#include <enet_cfg.h>
#include <include/per/cpsw.h>

#include <drivers/uart.h>

#include <drivers/sciclient.h>

#include "include/enet_apputils.h"
#include "include/enet_appboardutils.h"

#include "include/enet_appsoc.h"
#include "include/enet_apprm.h"

//TODO - private dependency
#include <priv/mod/cpsw_clks.h>
#include <kernel/dpl/SystemP.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/* Don't use EnetAppUtils_assert or print functions as UART might not be
 * initialized when this function is called.  */
void EnetAppUtils_setDeviceState(uint32_t moduleId,
                                 uint32_t requiredState,
                                 uint32_t appFlags)
{
    int32_t status;
    uint32_t moduleState      = 0U;
    uint32_t resetState       = 0U;
    uint32_t contextLossState = 0U;
    bool turnOn;
    bool turnOff;

    /* Get the module state */
    status = Sciclient_pmGetModuleState(moduleId,
                                        &moduleState,
                                        &resetState,
                                        &contextLossState,
                                        SystemP_WAIT_FOREVER);
    assert(status == CSL_PASS);

    /* No need to change if already in the requested state. Note that
     * getter (HW_STATE_*) and setter (SW_STATE_*) can't be directly compared */
    turnOn = (moduleState == TISCI_MSG_VALUE_DEVICE_HW_STATE_OFF) &&
             (requiredState == TISCI_MSG_VALUE_DEVICE_SW_STATE_ON);
    turnOff = (moduleState == TISCI_MSG_VALUE_DEVICE_HW_STATE_ON) &&
              (requiredState == TISCI_MSG_VALUE_DEVICE_SW_STATE_AUTO_OFF);

    /* We assert in this function upon failure as this is first function called
     * from the app to enable clocks and app can't recover from this failure */
    if (turnOn || turnOff)
    {
        status = Sciclient_pmSetModuleState(moduleId,
                                            requiredState,
                                            (appFlags |
                                             TISCI_MSG_FLAG_AOP |
                                             TISCI_MSG_FLAG_DEVICE_RESET_ISO),
                                             SystemP_WAIT_FOREVER);
        assert(status == CSL_PASS);

        if (requiredState == TISCI_MSG_VALUE_DEVICE_SW_STATE_ON)
        {
            /* Reset if changed state to enabled */
            status = Sciclient_pmSetModuleRst(moduleId,
                                              0x0U /*resetBit*/,
                                              SystemP_WAIT_FOREVER);
            assert(status == CSL_PASS);
        }
    }

}

void EnetAppUtils_clkRateSetState(uint32_t moduleId,
                                     uint32_t clockId,
                                     uint32_t additionalFlag,
                                     uint32_t state)
{
    int32_t status;

    status = Sciclient_pmModuleClkRequest
                 (moduleId,
                 clockId,
                 state,
                 additionalFlag,
                 SystemP_WAIT_FOREVER);
    if (status != CSL_PASS)
    {
        EnetAppUtils_print("Sciclient_pmModuleClkRequest failed for module=%d, clockId=%d\r\n",
                           moduleId, clockId);
        EnetAppUtils_assert(false);
    }
}

void EnetAppUtils_clkRateSet(uint32_t moduleId,
                             uint32_t clkId,
                             uint64_t clkRateHz)
{
#if !defined(SOC_AM64X) && !defined(SOC_AM243X)
    int32_t status;
    uint64_t currClkFreqHz;

    status = PMLIBClkRateGet(moduleId, clkId, &currClkFreqHz);
    if ((status == CSL_PASS) &&
        (currClkFreqHz != clkRateHz))
    {
        status = PMLIBClkRateSet(moduleId, clkId, clkRateHz);
        if (status != CSL_PASS)
        {
            EnetAppUtils_print("PMLIBClkRateSet failed for clock Id = %d\r\n", clkId);
            EnetAppUtils_assert(false);
        }
    }
#endif
}

static EnetAppUtils_MmrLockState EnetAppUtils_unlockMmr(volatile uint32_t *kick0,
                                                        volatile uint32_t *kick1)
{
    EnetAppUtils_MmrLockState prevLockState;
    uint32_t kick0Val;

    /* if kick lock registers are locked*/
    if ((CSL_REG32_RD(kick0) & 0x1U) == 0U)
    {
        /* unlock the partition by writing the unlock values to the kick lock registers*/
        CSL_REG32_WR(kick0, MMR_KICK0_UNLOCK_VAL);
        CSL_REG32_WR(kick1, MMR_KICK1_UNLOCK_VAL);

        prevLockState = ENETAPPUTILS_LOCK_MMR;
    }
    else
    {
        prevLockState = ENETAPPUTILS_UNLOCK_MMR;
    }

    kick0Val = CSL_REG32_FEXT(kick0, MMR_KICK_UNLOCKED);

    /* check to see if unlock bit in lock0 register is set */
    if (1U != kick0Val)
    {
        EnetAppUtils_assert(false);
    }

    return prevLockState;
}

static EnetAppUtils_MmrLockState EnetAppUtils_lockMmr(volatile uint32_t *kick0,
                                                      volatile uint32_t *kick1)
{
    EnetAppUtils_MmrLockState prevLockState;
    uint32_t kick0Val;

    /* check to see if either of the kick registers are unlocked.*/
    if ((CSL_REG32_RD(kick0) & 0x1) == 1U)
    {
        /* write the kick lock value to the kick lock registers to lock the partition*/
        CSL_REG32_WR(kick0, MMR_KICK_LOCK_VAL);
        CSL_REG32_WR(kick1, MMR_KICK_LOCK_VAL);

        prevLockState = ENETAPPUTILS_LOCK_MMR;
    }
    else
    {
        prevLockState = ENETAPPUTILS_UNLOCK_MMR;
    }

    /* check to see if unlock bit in lock0 register is set */
    kick0Val = CSL_REG32_FEXT(kick0, MMR_KICK_UNLOCKED);

    if (0U != kick0Val)
    {
        EnetAppUtils_assert(false);
    }

    return prevLockState;
}

EnetAppUtils_MmrLockState EnetAppUtils_mcuMmrCtrl(EnetAppUtils_CtrlMmrType mmrNum,
                                                  EnetAppUtils_MmrLockState lock)
{
    CSL_mcu_ctrl_mmr_cfg0Regs *regs =
        (CSL_mcu_ctrl_mmr_cfg0Regs *)(uintptr_t)CSL_CTRL_MMR0_CFG0_BASE;
    volatile uint32_t *kick0 = NULL, *kick1 = NULL;
    EnetAppUtils_MmrLockState prevLockState = ENETAPPUTILS_LOCK_MMR;

    switch (mmrNum)
    {
        case ENETAPPUTILS_MMR_LOCK0:
            kick0 = &regs->LOCK0_KICK0;
            kick1 = &regs->LOCK0_KICK1;
            break;

        case ENETAPPUTILS_MMR_LOCK1:
            kick0 = &regs->LOCK1_KICK0;
            kick1 = &regs->LOCK1_KICK1;
            break;

        case ENETAPPUTILS_MMR_LOCK2:
            kick0 = &regs->LOCK2_KICK0;
            kick1 = &regs->LOCK2_KICK1;
            break;

        case ENETAPPUTILS_MMR_LOCK3:
            kick0 = &regs->LOCK3_KICK0;
            kick1 = &regs->LOCK3_KICK1;
            break;

        case ENETAPPUTILS_MMR_LOCK4:
            kick0 = &regs->LOCK4_KICK0;
            kick1 = &regs->LOCK4_KICK1;
            break;

        default:
            EnetAppUtils_assert(false);
            break;
    }

    if ((NULL != kick0) && (NULL != kick1))
    {
        if (ENETAPPUTILS_UNLOCK_MMR == lock)
        {
            prevLockState = EnetAppUtils_unlockMmr(kick0, kick1);
        }
        else
        {
            prevLockState = EnetAppUtils_lockMmr(kick0, kick1);
        }
    }

    return prevLockState;
}

EnetAppUtils_MmrLockState EnetAppUtils_mainMmrCtrl(EnetAppUtils_CtrlMmrType mmrNum,
                                                   EnetAppUtils_MmrLockState lock)
{
    CSL_main_ctrl_mmr_cfg0Regs *regs =
        (CSL_main_ctrl_mmr_cfg0Regs *)(uintptr_t)CSL_CTRL_MMR0_CFG0_BASE;
    volatile uint32_t *kick0 = NULL, *kick1 = NULL;
    EnetAppUtils_MmrLockState prevLockState = ENETAPPUTILS_LOCK_MMR;

    switch (mmrNum)
    {
        case ENETAPPUTILS_MMR_LOCK0:
            kick0 = &regs->LOCK0_KICK0;
            kick1 = &regs->LOCK0_KICK1;
            break;

        case ENETAPPUTILS_MMR_LOCK1:
            kick0 = &regs->LOCK1_KICK0;
            kick1 = &regs->LOCK1_KICK1;
            break;

        case ENETAPPUTILS_MMR_LOCK2:
            kick0 = &regs->LOCK2_KICK0;
            kick1 = &regs->LOCK2_KICK1;
            break;

        case ENETAPPUTILS_MMR_LOCK3:
            kick0 = &regs->LOCK3_KICK0;
            kick1 = &regs->LOCK3_KICK1;
            break;

        case ENETAPPUTILS_MMR_LOCK4:
            kick0 = &regs->LOCK4_KICK0;
            kick1 = &regs->LOCK4_KICK1;
            break;

#if defined(SOC_AM64X) || defined(SOC_AM243X)
        case ENETAPPUTILS_MMR_LOCK6:
            kick0 = &regs->LOCK6_KICK0;
            kick1 = &regs->LOCK6_KICK1;
            break;
#endif

#if !defined(SOC_AM64X) && !defined(SOC_AM243X)
        case ENETAPPUTILS_MMR_LOCK7:
            kick0 = &regs->LOCK7_KICK0;
            kick1 = &regs->LOCK7_KICK1;
            break;
#endif

        default:
            EnetAppUtils_assert(false);
            break;
    }

    if ((NULL != kick0) && (NULL != kick1))
    {
        if (ENETAPPUTILS_UNLOCK_MMR == lock)
        {
            prevLockState = EnetAppUtils_unlockMmr(kick0, kick1);
        }
        else
        {
            prevLockState = EnetAppUtils_lockMmr(kick0, kick1);
        }
    }

    return prevLockState;
}

void EnetAppUtils_enableClkOut(Enet_Type enetType,
                               EnetAppUtils_ClkOutFreqType clkOut)
{
    EnetAppUtils_MmrLockState prevLockState;
    uint32_t clkVal;

    if (ENETAPPUTILS_CLKOUT_FREQ_50MHZ == clkOut)
    {
        clkVal = 0U;
    }
    else
    {
        /* 25 MHz */
        clkVal = 1U;
    }

    switch (enetType)
    {
#if defined(SOC_AM64X) || defined(SOC_AM243X)
    case ENET_CPSW_3G:
        {
            CSL_main_ctrl_mmr_cfg0Regs *mainRegs;
            mainRegs = (CSL_main_ctrl_mmr_cfg0Regs *)(uintptr_t)CSL_CTRL_MMR0_CFG0_BASE;

            prevLockState = EnetAppUtils_mainMmrCtrl(ENETAPPUTILS_MMR_LOCK1, ENETAPPUTILS_UNLOCK_MMR);
            CSL_REG32_FINS(&mainRegs->CLKOUT_CTRL,
                           MAIN_CTRL_MMR_CFG0_CLKOUT_CTRL_CLK_SEL,
                           clkVal);
            CSL_REG32_FINS(&mainRegs->CLKOUT_CTRL,
                           MAIN_CTRL_MMR_CFG0_CLKOUT_CTRL_CLK_EN,
                           1U);
            if (prevLockState == ENETAPPUTILS_LOCK_MMR)
            {
                EnetAppUtils_mainMmrCtrl(ENETAPPUTILS_MMR_LOCK1, ENETAPPUTILS_LOCK_MMR);
            }
        }
        break;
#endif

        default:
            /* Assert if case new mac mode gets added - ex. for SGMII clock config */
            EnetAppUtils_assert(false);
            break;
    }

}

#if (ENET_ENABLE_PER_CPSW == 1)
static void EnetAppUtils_selectCptsClock(Enet_Type enetType,
                                         EnetAppUtils_CptsClkSelMux clkSelMux)
{
    uint32_t muxVal;

    muxVal = (uint32_t) clkSelMux;

#if defined(SOC_AM64X) || defined(SOC_AM243X)
    switch (enetType)
    {
        case ENET_CPSW_3G:
        {
            CSL_cptsRegs *regs = NULL;

            regs = (CSL_cptsRegs *)(uintptr_t)(CSL_CPSW0_NUSS_BASE +
                                               CPSW_CPTS_OFFSET);
            CSL_CPTS_setRFTCLKSelectReg(regs, muxVal);
        }
        break;

        default:
            EnetAppUtils_assert(false);
            break;
    }
#else
#error "Unsupported platform"
#endif
}
#endif

#define CPSW_SOC_RGMII_MHZ_250_CLK_VAL        (250000000U)
#define CPSW_SOC_RGMII_MHZ_50_CLK_VAL         (50000000U)
#define CPSW_SOC_RGMII_MHZ_5_CLK_VAL          (5000000U)

void EnetAppUtils_enableClocks(Enet_Type enetType, uint32_t instId)
{
    uint32_t moduleId = 0U;
    uint32_t appFlags = 0U;
    uint64_t cppiClkFreqHz;
    uint32_t cppiClkId;
    uint32_t rgmii250MHzClkId;
    uint32_t rgmii50MHzClkId;
    uint32_t rgmii5MHzClkId;

    EnetAppUtils_print("Enabling clocks!\r\n");

    cppiClkFreqHz = EnetSoc_getClkFreq(enetType, instId, CPSW_CPPI_CLK);

    switch (enetType)
    {
#if defined(SOC_AM64X) || defined(SOC_AM243X)
        case ENET_CPSW_3G:
        {
            moduleId = TISCI_DEV_CPSW0;
            cppiClkId        = TISCI_DEV_CPSW0_CPPI_CLK_CLK,
            rgmii250MHzClkId = TISCI_DEV_CPSW0_RGMII_MHZ_250_CLK,
            rgmii50MHzClkId  = TISCI_DEV_CPSW0_RGMII_MHZ_50_CLK,
            rgmii5MHzClkId   = TISCI_DEV_CPSW0_RGMII_MHZ_5_CLK,
            EnetAppUtils_assert(cppiClkFreqHz != 0U);
            EnetAppUtils_clkRateSet(moduleId, cppiClkId       , cppiClkFreqHz);
            EnetAppUtils_clkRateSet(moduleId, rgmii250MHzClkId, CPSW_SOC_RGMII_MHZ_250_CLK_VAL);
            EnetAppUtils_clkRateSet(moduleId, rgmii50MHzClkId , CPSW_SOC_RGMII_MHZ_50_CLK_VAL);
            EnetAppUtils_clkRateSet(moduleId, rgmii5MHzClkId  , CPSW_SOC_RGMII_MHZ_5_CLK_VAL);
        }
        break;
#endif

#if (ENET_ENABLE_PER_ICSSG == 1)
        case ENET_ICSSG_DUALMAC:
        {
            if ((instId == 0U) || (instId == 1U))
            {
                moduleId = TISCI_DEV_PRU_ICSSG0;
            }
            else if ((instId == 2U) || (instId == 3U))
            {
                moduleId = TISCI_DEV_PRU_ICSSG1;
            }
            else
            {
                EnetAppUtils_assert(false);
            }
        }
        break;

        case ENET_ICSSG_SWITCH:
        {
            if (instId == 0U)
            {
                moduleId = TISCI_DEV_PRU_ICSSG0;
            }
            else if (instId == 1)
            {
                moduleId = TISCI_DEV_PRU_ICSSG1;
            }
            else
            {
                EnetAppUtils_assert(false);
            }
        }
        break;
#endif
        default:
            EnetAppUtils_assert(false);
    }

    EnetAppUtils_setDeviceState(moduleId, TISCI_MSG_VALUE_DEVICE_SW_STATE_ON, appFlags);

#if (ENET_ENABLE_PER_CPSW == 1)
    if (Enet_isCpswFamily(enetType))
    {
        EnetAppUtils_CptsClkSelMux clkSelMux;
#if defined(SOC_AM64X) || defined(SOC_AM243X)
        //confirm this
        clkSelMux = ENETAPPUTILS_CPTS_CLKSEL_CPSWHSDIV_CLKOUT2;
#endif

        EnetAppUtils_selectCptsClock(enetType, clkSelMux);
    }
#endif
}

void EnetAppUtils_disableClocks(Enet_Type enetType, uint32_t instId)
{
    uint32_t moduleId = 0U;
    uint32_t appFlags = 0U;
    uint32_t cppiClkId;
    uint32_t rgmii250MHzClkId;
    uint32_t rgmii50MHzClkId;
    uint32_t rgmii5MHzClkId;

    EnetAppUtils_print("Disabling clocks for CPSW!\r\n");

    switch (enetType)
    {
#if defined(SOC_AM64X) || defined(SOC_AM243X)
        case ENET_CPSW_3G:
        {
            moduleId = TISCI_DEV_CPSW0;
            cppiClkId        = TISCI_DEV_CPSW0_CPPI_CLK_CLK,
            rgmii250MHzClkId = TISCI_DEV_CPSW0_RGMII_MHZ_250_CLK,
            rgmii50MHzClkId  = TISCI_DEV_CPSW0_RGMII_MHZ_50_CLK,
            rgmii5MHzClkId   = TISCI_DEV_CPSW0_RGMII_MHZ_5_CLK,
            /* set clock set to auto so when module is disabled, clocks shuts off */
            EnetAppUtils_clkRateSetState(moduleId,
                                         cppiClkId,
                                         0,
                                         TISCI_MSG_VALUE_CLOCK_SW_STATE_AUTO);
            EnetAppUtils_clkRateSetState(moduleId,
                                         rgmii250MHzClkId,
                                         0,
                                         TISCI_MSG_VALUE_CLOCK_SW_STATE_AUTO);
            EnetAppUtils_clkRateSetState(moduleId,
                                         rgmii50MHzClkId,
                                         0,
                                         TISCI_MSG_VALUE_CLOCK_SW_STATE_AUTO);
            EnetAppUtils_clkRateSetState(moduleId,
                                         rgmii5MHzClkId,
                                         0,
                                         TISCI_MSG_VALUE_CLOCK_SW_STATE_AUTO);
        }
        break;
#endif

#if (ENET_ENABLE_PER_ICSSG == 1)
        case ENET_ICSSG_DUALMAC:
        {
            if ((instId == 0U) || (instId == 1U))
            {
                moduleId = TISCI_DEV_PRU_ICSSG0;
            }
            else if ((instId == 2U) || (instId == 3U))
            {
                moduleId = TISCI_DEV_PRU_ICSSG1;
            }
            else
            {
                EnetAppUtils_assert(false);
            }
            //MUSTFIX - add ICSSG clocks
        }
        break;

        case ENET_ICSSG_SWITCH:
        {
            if (instId == 0U)
            {
                moduleId = TISCI_DEV_PRU_ICSSG0;
            }
            else if (instId == 1)
            {
                moduleId = TISCI_DEV_PRU_ICSSG1;
            }
            else
            {
                EnetAppUtils_assert(false);
            }
        }
        break;
#endif
        default:
            EnetAppUtils_assert(false);
    }

    /* Set module set to HW AUTO */
    EnetAppUtils_setDeviceState(moduleId, TISCI_MSG_VALUE_DEVICE_SW_STATE_AUTO_OFF, appFlags);
}

int32_t EnetAppUtils_setTimeSyncRouter(Enet_Type enetType, uint32_t input, uint32_t output)
{
    int32_t  status = ENET_SOK;

#if defined(SOC_AM64X) || defined(SOC_AM243X)
    EnetAppUtils_assert(enetType == ENET_CPSW_3G);
#endif

    return status;
}

void EnetAppUtils_setupSciServer(void)
{
    return;
}

/* end of file */
