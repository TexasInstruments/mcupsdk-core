/*
 *  Copyright (C) 2022 Texas Instruments Incorporated
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

#include <stdint.h>
#include <string.h>
#include <drivers/pcie/pcie.h>
#include <drivers/pcie/v0/pcie_v0.h>
#include <drivers/pcie/pcie_priv.h>
#include <drivers/pcie/v0/pcie_v0_reg.h>
#include <drivers/hw_include/cslr_soc.h>
#include <drivers/hw_include/cslr_pcie.h>
#include <drivers/hw_include/cslr_ringacc.h>
#include <drivers/soc.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/HwiP.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* Legacy IRQ register read/write macros */
#define INTD_CFG_ENABLE_REG_SYS_EN_PCIE_LEGACY_0_MASK       (0x00400000U)
#define INTD_CFG_ENABLE_REG_SYS_EN_PCIE_LEGACY_0_SHIFT      (0x00000016U)
#define INTD_CFG_ENABLE_REG_SYS_EN_PCIE_LEGACY_0_MAX        (0x00000001U)

#define INTD_CFG_ENABLE_REG_SYS_EN_PCIE_LEGACY_1_MASK       (0x00800000U)
#define INTD_CFG_ENABLE_REG_SYS_EN_PCIE_LEGACY_1_SHIFT      (0x00000017U)
#define INTD_CFG_ENABLE_REG_SYS_EN_PCIE_LEGACY_1_MAX        (0x00000001U)

#define INTD_CFG_ENABLE_REG_SYS_EN_PCIE_LEGACY_2_MASK       (0x01000000U)
#define INTD_CFG_ENABLE_REG_SYS_EN_PCIE_LEGACY_2_SHIFT      (0x00000018U)
#define INTD_CFG_ENABLE_REG_SYS_EN_PCIE_LEGACY_2_MAX        (0x00000001U)

#define INTD_CFG_ENABLE_REG_SYS_EN_PCIE_LEGACY_3_MASK       (0x02000000U)
#define INTD_CFG_ENABLE_REG_SYS_EN_PCIE_LEGACY_3_SHIFT      (0x00000019U)
#define INTD_CFG_ENABLE_REG_SYS_EN_PCIE_LEGACY_3_MAX        (0x00000001U)


#define INTD_CFG_ENABLE_CLR_REG_SYS_EN_PCIE_LEGACY_0_CLR_MASK       (0x00400000U)
#define INTD_CFG_ENABLE_CLR_REG_SYS_EN_PCIE_LEGACY_0_CLR_SHIFT      (0x00000016U)
#define INTD_CFG_ENABLE_CLR_REG_SYS_EN_PCIE_LEGACY_0_CLR_MAX        (0x00000001U)

#define INTD_CFG_ENABLE_CLR_REG_SYS_EN_PCIE_LEGACY_1_CLR_MASK       (0x00800000U)
#define INTD_CFG_ENABLE_CLR_REG_SYS_EN_PCIE_LEGACY_1_CLR_SHIFT      (0x00000017U)
#define INTD_CFG_ENABLE_CLR_REG_SYS_EN_PCIE_LEGACY_1_CLR_MAX        (0x00000001U)

#define INTD_CFG_ENABLE_CLR_REG_SYS_EN_PCIE_LEGACY_2_CLR_MASK       (0x01000000U)
#define INTD_CFG_ENABLE_CLR_REG_SYS_EN_PCIE_LEGACY_2_CLR_SHIFT      (0x00000018U)
#define INTD_CFG_ENABLE_CLR_REG_SYS_EN_PCIE_LEGACY_2_CLR_MAX        (0x00000001U)

#define INTD_CFG_ENABLE_CLR_REG_SYS_EN_PCIE_LEGACY_3_CLR_MASK       (0x02000000U)
#define INTD_CFG_ENABLE_CLR_REG_SYS_EN_PCIE_LEGACY_3_CLR_SHIFT      (0x00000019U)
#define INTD_CFG_ENABLE_CLR_REG_SYS_EN_PCIE_LEGACY_3_CLR_MAX        (0x00000001U)


#define INTD_CFG_STATUS_SYS_PCIE_LEGACY_0_MASK                      (0x00400000U)
#define INTD_CFG_STATUS_SYS_PCIE_LEGACY_0_SHIFT                     (0x00000016U)
#define INTD_CFG_STATUS_SYS_PCIE_LEGACY_0_MAX                       (0x00000001U)

#define INTD_CFG_STATUS_SYS_PCIE_LEGACY_1_MASK                      (0x00800000U)
#define INTD_CFG_STATUS_SYS_PCIE_LEGACY_1_SHIFT                     (0x00000017U)
#define INTD_CFG_STATUS_SYS_PCIE_LEGACY_1_MAX                       (0x00000001U)

#define INTD_CFG_STATUS_SYS_PCIE_LEGACY_2_MASK                      (0x01000000U)
#define INTD_CFG_STATUS_SYS_PCIE_LEGACY_2_SHIFT                     (0x00000018U)
#define INTD_CFG_STATUS_SYS_PCIE_LEGACY_2_MAX                       (0x00000001U)

#define INTD_CFG_STATUS_SYS_PCIE_LEGACY_3_MASK                      (0x02000000U)
#define INTD_CFG_STATUS_SYS_PCIE_LEGACY_3_SHIFT                     (0x00000019U)
#define INTD_CFG_STATUS_SYS_PCIE_LEGACY_3_MAX                       (0x00000001U)

/* AM64x legacy IRQ number */
#define PCIE_LEGACY_INTNUM_AM64X                                    (234u)

/* AM64x Number of legacy interrupts */
#define PCIE_NUM_LEGACY_INT                                         (4u)

/* Offset to write MSI */
#define PCIE_MSI_IRQ_ADDR_OFFSET                                    (0xFCu)

/* Offset to write MSIx */
#define PCIE_MSIX_IRQ_ADDR_OFFSET                                   (0xFCu)

/* Index of ring (from the range allocated) used for MSI */
#define MSI_RING_ACC_INDEX         (0U)
/* Index of global event (from the range allocated) used for MSI */
#define MSI_GLOBAL_EVENT_INDEX     (50U)
/* Index of virtual interrupt (from the range allocated) used for MSI */
#define MSI_VINT_INDEX             (5U)

/* Index of ring (from the range allocated) used for MSI */
#define MSIX_RING_ACC_INDEX         (1U)
/* Index of global event (from the range allocated) used for MSIX */
#define MSIX_GLOBAL_EVENT_INDEX     (51U)
/* Index of virtual interrupt (from the range allocated) used for MSIX */
#define MSIX_VINT_INDEX             (6U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */
typedef struct Pcie_VintToCoreIntMap
{
    uint32_t coreId;
    uint32_t vintStart;
    uint32_t vintNum;
    uint32_t coreIntStart;
}Pcie_VintToCoreIntMap;

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

Pcie_DevParams Pcie_devParamsDev1 =
{
    (volatile uint32_t *)(CSL_CTRL_MMR0_CFG0_BASE + CSL_MAIN_CTRL_MMR_CFG0_PCIE0_CTRL),
    (volatile uint32_t *)(CSL_CTRL_MMR0_CFG0_BASE + CSL_MAIN_CTRL_MMR_CFG0_PCIE0_CLKSEL),
    (void *)CSL_PCIE0_CORE_USER_CFG_USER_CFG_BASE,
    (void *)CSL_PCIE0_CORE_PCIE_INTD_CFG_INTD_CFG_BASE,
    1, /* one lane */
    PCIE_GEN2, /* default to GEN2 */
    0  /* use PF 0 */
};

Pcie_DeviceCfgBaseAddrs Pcie_cfgBaseAddrDev1 =
{
    (void *)CSL_PCIE0_CORE_DBN_CFG_PCIE_CORE_BASE,
    /* The mapped offset is 0x10010000, but that points to registers
    that normally occur at base + 0x1000, hence the - 0x1000.
    No access to 0x0 to 0x0FFF is normal for other devices but
    they have separate structure that can be pointed here */
    (uint32_t)0x10000u
};

Pcie_DeviceCfgBaseAddr pcieBaseAddrDev1 =
{
    (void *)&Pcie_cfgBaseAddrDev1,
    (void *)CSL_PCIE0_DAT0_BASE,
    0U,
    (void *)&Pcie_devParamsDev1
};

Pcie_InitCfg Pcie_initCfg =
{
  {
    {
      &pcieBaseAddrDev1,
      NULL, /* &pcieBaseAddrDev2, */
      NULL, /* &pcieBaseAddrDev3, */
      NULL /* &pcieBaseAddrDev4 */
    },
  }
};

static Pcie_VintToCoreIntMap gPcieVintToCoreIntMap[] =
{
    {
        .coreId = TISCI_DEV_R5FSS0_CORE0,
        .vintStart = 40,
        .vintNum = 32,
        .coreIntStart = 64,
    },
    {
        .coreId = TISCI_DEV_R5FSS0_CORE1,
        .vintStart = 40,
        .vintNum = 32,
        .coreIntStart = 64,
    },
    {
        .coreId = TISCI_DEV_R5FSS1_CORE0,
        .vintStart = 88,
        .vintNum = 32,
        .coreIntStart = 64,
    },
    {
        .coreId = TISCI_DEV_R5FSS1_CORE1,
        .vintStart = 84,
        .vintNum = 32,
        .coreIntStart = 64,
    },
    {
        .coreId = 0xFFFFFFFFU,
        .vintStart = 0,
        .vintNum = 0,
        .coreIntStart = 0,
    }
};

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* Get core interrupt number from VINT number */
static int32_t Pcie_getIntNumForVint(uint32_t coreId, uint32_t vintNum,
                                                uint32_t *intNum);
/* Handler for MSI interrupts */
static void Pcie_msiHandler(void * args);

/* Handler for MSIx interrupts */
static void Pcie_msixHandler(void * args);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

static int32_t Pcie_getIntNumForVint(uint32_t coreId, uint32_t vintNum,
                                        uint32_t *intNum)
{
    int32_t status = SystemP_FAILURE;
    uint32_t i = 0;

    while (gPcieVintToCoreIntMap[i].coreId != 0xFFFFFFFFU)
    {
        if (gPcieVintToCoreIntMap[i].coreId == coreId)
        {
            if ((vintNum >= gPcieVintToCoreIntMap[i].vintStart)
                && (vintNum <= (gPcieVintToCoreIntMap[i].vintStart +
                        gPcieVintToCoreIntMap[i].vintNum)))
            {
                *intNum = gPcieVintToCoreIntMap[i].coreIntStart
                        + (vintNum - gPcieVintToCoreIntMap[i].vintStart);

                status = SystemP_SUCCESS;
            }
            break;
        }
    }

    return status;
}

static void Pcie_msiHandler(void * args)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t msiData;
    Pcie_Config *pcieCfg;
    uint32_t ringNum;
    CSL_RingAccCfg pCfg;
    CSL_RingAccRingCfg pRing;
    uint32_t msiIrqNum;

    if(args != NULL)
    {
        pcieCfg = (Pcie_Config *)args;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    if (status == SystemP_SUCCESS)
    {
        ringNum = pcieCfg->attrs->msiRingNum;

        pCfg.pFifoRegs = (CSL_ringacc_fifosRegs *)CSL_DMASS0_RINGACC_SRC_FIFOS_BASE;
        pRing.ringNum = ringNum;
        pRing.mode = CSL_RINGACC_RING_MODE_MESSAGE;

        /* Read MSI data received by poping from the Ring accelerator */
        status = CSL_ringaccPop32(&pCfg, &pRing, &msiData, NULL);
    }

    if (status == SystemP_SUCCESS)
    {
        msiIrqNum = msiData & PCIE_MSI_IRQNUM_MASK;

        if (pcieCfg->attrs->msiIsrCtrl->isr[msiIrqNum] != NULL)
        {
            pcieCfg->attrs->msiIsrCtrl->isr[msiIrqNum](pcieCfg->attrs->msiIsrCtrl->isrArgs[msiIrqNum],
                                                        msiData);
        }
    }
}

void Pcie_srisControl (Pcie_Handle handle, uint32_t enable)
{
    Pcie_DeviceCfgBaseAddr *cfg = Pcie_handleGetBases (handle);

    DebugP_assert(NULL != cfg);

    Pcie_DevParams *params = (Pcie_DevParams*)cfg->devParams;

    DebugP_assert(NULL != params);

    CSL_user_cfgRegs *userCfg = (CSL_user_cfgRegs *)params->userCfgBase;
    uint32_t val = userCfg->INITCFG;

    PCIE_SETBITS(val, CSL_USER_CFG_INITCFG_SRIS_ENABLE, enable);

    userCfg->INITCFG = val;
}

void Pcie_unlockMMRCtrlReg (void)
{
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, 1);

    return;
}

void Pcie_lockMMRCtrlReg (void)
{
    SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MAIN, 1);

    return;
}

int32_t Pcie_readLnkCtrlReg (const CSL_pcie_ep_coreRegs *baseAddr, Pcie_LnkCtrlReg *swReg)
{
    /* AM64x supports on single lane operation */
    swReg->lnkMode = 0;

    return SystemP_SUCCESS;
}

int32_t Pcie_writeLnkCtrlReg (CSL_pcie_ep_coreRegs *baseAddr, Pcie_LnkCtrlReg *swReg)
{
    /* Lane count is not configurable in AM64x (Only single lane is supported) */

    return SystemP_SUCCESS;
}

/*****************************************************************************
 * Combine and write a Legacy Interrupt Enable Set swRegister
 ****************************************************************************/
int32_t Pcie_writeLegacyIrqEnableSetReg (CSL_intd_cfgRegs *baseAddr, Pcie_LegacyIrqEnableSetReg *swReg,
                        int_fast32_t swRegNum)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t new_val;

    if ((baseAddr != NULL) && (swReg != NULL))
    {
        new_val = swReg->raw;

        switch (swRegNum)
        {
            case 0:
                PCIE_SETBITS(new_val, INTD_CFG_ENABLE_REG_SYS_EN_PCIE_LEGACY_0, swReg->legacyIrqEnSet);
                break;
            case 1:
                PCIE_SETBITS(new_val, INTD_CFG_ENABLE_REG_SYS_EN_PCIE_LEGACY_1, swReg->legacyIrqEnSet);
                break;
            case 2:
                PCIE_SETBITS(new_val, INTD_CFG_ENABLE_REG_SYS_EN_PCIE_LEGACY_2, swReg->legacyIrqEnSet);
                break;
            case 3:
                PCIE_SETBITS(new_val, INTD_CFG_ENABLE_REG_SYS_EN_PCIE_LEGACY_3, swReg->legacyIrqEnSet);
                break;
        }

        baseAddr->ENABLE_REG_SYS_1 = swReg->raw = new_val;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* Pcie_writeLegacyIrqEnableSetReg */

/*****************************************************************************
 * Combine and write the Legacy Interrupt Enable Clear swRegister
 ****************************************************************************/
int32_t Pcie_writeLegacyIrqEnableClrReg (CSL_intd_cfgRegs *baseAddr, Pcie_LegacyIrqEnableClrReg *swReg,
                                int_fast32_t swRegNum)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t new_val;

    if ((baseAddr != NULL) && (swReg != NULL))
    {
        new_val = swReg->raw;

        switch (swRegNum)
        {
            case 0:
                PCIE_SETBITS(new_val, INTD_CFG_ENABLE_CLR_REG_SYS_EN_PCIE_LEGACY_0_CLR, swReg->legacyIrqEnClr);
                break;
            case 1:
                PCIE_SETBITS(new_val, INTD_CFG_ENABLE_CLR_REG_SYS_EN_PCIE_LEGACY_1_CLR, swReg->legacyIrqEnClr);
                break;
            case 2:
                PCIE_SETBITS(new_val, INTD_CFG_ENABLE_CLR_REG_SYS_EN_PCIE_LEGACY_2_CLR, swReg->legacyIrqEnClr);
                break;
            case 3:
                PCIE_SETBITS(new_val, INTD_CFG_ENABLE_CLR_REG_SYS_EN_PCIE_LEGACY_3_CLR, swReg->legacyIrqEnClr);
                break;
        }
        baseAddr->ENABLE_CLR_REG_SYS_1 = swReg->raw = new_val;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* Pcie_writeLegacyIrqEnableClrReg */

/*****************************************************************************
 * Combine and write a Legacy Interrupt Status swRegister
 ****************************************************************************/
int32_t Pcie_writeLegacyIrqStatusReg (CSL_intd_cfgRegs *baseAddr, Pcie_LegacyIrqStatusReg *swReg,
                                    int_fast32_t swRegNum)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t new_val;

    if ((baseAddr != NULL) && (swReg != NULL))
    {
        new_val = swReg->raw;
        switch (swRegNum)
        {
            case 0:
                PCIE_SETBITS(new_val, INTD_CFG_STATUS_SYS_PCIE_LEGACY_0, swReg->legacyIrqStatus);
                break;
            case 1:
                PCIE_SETBITS(new_val, INTD_CFG_STATUS_SYS_PCIE_LEGACY_1, swReg->legacyIrqStatus);
                break;
            case 2:
                PCIE_SETBITS(new_val, INTD_CFG_STATUS_SYS_PCIE_LEGACY_2, swReg->legacyIrqStatus);
                break;
            case 3:
                PCIE_SETBITS(new_val, INTD_CFG_STATUS_SYS_PCIE_LEGACY_3, swReg->legacyIrqStatus);
                break;
        }

        baseAddr->STATUS_REG_SYS_0 = swReg->raw = new_val;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* Pcie_writeLegacyIrqStatusReg */

/*****************************************************************************
 * Read and split up a Legacy Interrupt Enable Set swRegister
 ****************************************************************************/
int32_t Pcie_readLegacyIrqEnableSetReg(const CSL_intd_cfgRegs *baseAddr, Pcie_LegacyIrqEnableSetReg *swReg,
                                            int_fast32_t swRegNum)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t val;

    if ((baseAddr != NULL) && (swReg != NULL))
    {
        val = swReg->raw = baseAddr->ENABLE_REG_SYS_1;

        switch (swRegNum)
        {
            case 0:
                PCIE_GETBITS(val, INTD_CFG_ENABLE_REG_SYS_EN_PCIE_LEGACY_0, swReg->legacyIrqEnSet);
                break;
            case 1:
                PCIE_GETBITS(val, INTD_CFG_ENABLE_REG_SYS_EN_PCIE_LEGACY_1, swReg->legacyIrqEnSet);
                break;
            case 2:
                PCIE_GETBITS(val, INTD_CFG_ENABLE_REG_SYS_EN_PCIE_LEGACY_2, swReg->legacyIrqEnSet);
                break;
            case 3:
                PCIE_GETBITS(val, INTD_CFG_ENABLE_REG_SYS_EN_PCIE_LEGACY_3, swReg->legacyIrqEnSet);
                break;
        }
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* Pcie_readLegacyIrqEnableSetReg */

/*****************************************************************************
 * Read and split up the Legacy Interrupt Enable Clear swRegister
 ****************************************************************************/
int32_t Pcie_readLegacyIrqEnableClrReg (const CSL_intd_cfgRegs *baseAddr, Pcie_LegacyIrqEnableClrReg *swReg,
                            int_fast32_t swRegNum)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t val;

    if ((baseAddr != NULL) && (swReg != NULL))
    {
        val = swReg->raw = baseAddr->ENABLE_CLR_REG_SYS_1;

        switch (swRegNum)
        {
            case 0:
                PCIE_GETBITS(val, INTD_CFG_ENABLE_CLR_REG_SYS_EN_PCIE_LEGACY_0_CLR, swReg->legacyIrqEnClr);
                break;
            case 1:
                PCIE_GETBITS(val, INTD_CFG_ENABLE_CLR_REG_SYS_EN_PCIE_LEGACY_1_CLR, swReg->legacyIrqEnClr);
                break;
            case 2:
                PCIE_GETBITS(val, INTD_CFG_ENABLE_CLR_REG_SYS_EN_PCIE_LEGACY_2_CLR, swReg->legacyIrqEnClr);
                break;
            case 3:
                PCIE_GETBITS(val, INTD_CFG_ENABLE_CLR_REG_SYS_EN_PCIE_LEGACY_3_CLR, swReg->legacyIrqEnClr);
                break;
        }
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* Pcie_readLegacyIrqEnableClrReg */

/*****************************************************************************
 * Read and split up a Legacy Interrupt Status swRegister
 ****************************************************************************/
int32_t Pcie_readLegacyIrqStatusReg(const CSL_intd_cfgRegs *baseAddr, Pcie_LegacyIrqStatusReg *swReg,
                                    int_fast32_t swRegNum)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t val;

    if ((baseAddr != NULL) && (swReg != NULL))
    {
        val = swReg->raw = baseAddr->STATUS_REG_SYS_0;

        switch (swRegNum)
        {
            case 0:
                PCIE_GETBITS(val, INTD_CFG_STATUS_SYS_PCIE_LEGACY_0, swReg->legacyIrqStatus);
                break;
            case 1:
                PCIE_GETBITS(val, INTD_CFG_STATUS_SYS_PCIE_LEGACY_1, swReg->legacyIrqStatus);
                break;
            case 2:
                PCIE_GETBITS(val, INTD_CFG_STATUS_SYS_PCIE_LEGACY_2, swReg->legacyIrqStatus);
                break;
            case 3:
                PCIE_GETBITS(val, INTD_CFG_STATUS_SYS_PCIE_LEGACY_3, swReg->legacyIrqStatus);
                break;
        }
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* Pcie_readLegacyIrqStatusReg */

int32_t Pcie_rcLegacyIrqEnable (Pcie_Handle handle, uint32_t enable)
{
    int32_t                           status;
    Pcie_LegacyIrqEnableSetReg        rcLegacyEnable;
    Pcie_Registers                    regs;
    uint8_t                           setVal;

    memset (&regs, 0, sizeof(Pcie_Registers));

    for (int i = 0; i < 4; i++)
    {
        memset (&rcLegacyEnable, 0, sizeof(rcLegacyEnable));
        regs.legacyIrqEnableSet[i] = &rcLegacyEnable;

        /* Read current value */
        status = Pcie_readRegs (handle, PCIE_LOCATION_LOCAL, &regs);

        if (status == SystemP_SUCCESS)
        {
            if (enable == 1)
            {
                setVal = 1;
            }
            else
            {
                setVal = 0;
            }

            rcLegacyEnable.legacyIrqEnSet = setVal;

            /* Write back */
            status = Pcie_writeRegs (handle, PCIE_LOCATION_LOCAL, &regs);
        }
        else
        {
            break;
        }
    }

    return status;
}

int32_t Pcie_rclegacyIrqIsrRegister (Pcie_Handle handle, Pcie_legacyIrqRegisterParam irqParams)
{
    int32_t status = SystemP_SUCCESS;
    HwiP_Params hwiParams;
    HwiP_Object hwiObj;

    HwiP_Params_init(&hwiParams);
    hwiParams.intNum = PCIE_LEGACY_INTNUM_AM64X;
    hwiParams.callback = irqParams.funcPtr;
    hwiParams.isPulse = 1;
    HwiP_construct(&hwiObj, &hwiParams);

    return status;
}

int32_t Pcie_epLegacyIrqSet(Pcie_Handle handle, Pcie_legacyIrqSetParams irqSetParams)
{
    int32_t status = SystemP_SUCCESS;
    Pcie_Registers regs;
    Pcie_EpIrqSetReg intxAssert;

    memset (&regs, 0, sizeof(Pcie_Registers));

    /* Read register */
    regs.epIrqSet = &intxAssert;
    status = Pcie_readRegs (handle, PCIE_LOCATION_LOCAL, &regs);

    if(status == SystemP_SUCCESS)
    {
        /* Write back register */
        intxAssert.epIrqSet = irqSetParams.assert;
        status = Pcie_writeRegs (handle, PCIE_LOCATION_LOCAL, &regs);
    }

    return status;
}

int32_t Pcie_rcEnableMSI (Pcie_Handle handle, uint32_t drvIndex,
                            Pcie_MsiParams msiParams)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t self_core = 0;
    int32_t ring_index = 0;
    int32_t global_event = 0;
    int32_t vint = 0;
    uint32_t intNum = 0;
    uint32_t irqRouteSet = 0;
    HwiP_Params hwiParams;
    HwiP_Object hwiObj;

    struct tisci_msg_rm_get_resource_range_req  req;
    struct tisci_msg_rm_get_resource_range_resp res;

    struct tisci_msg_rm_ring_cfg_req    rmRingReq;
    struct tisci_msg_rm_ring_cfg_resp   rmRingResp;

    struct tisci_msg_rm_irq_set_req     rmIrqReq;
    struct tisci_msg_rm_irq_set_resp    rmIrqResp;

    struct tisci_msg_rm_irq_release_req rmIrqReleaseReq;

    Pcie_Config *pcieCfg;
    Pcie_Registers regs;
    Pcie_MsiCapReg msiCapReg;
    Pcie_MsiLo32Reg msiLo32;
    Pcie_MsiUp32Reg msiUp32;
    Pcie_MsiDataReg msiData;

    if (handle != NULL)
    {
        pcieCfg = (Pcie_Config *)handle;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    if (status == SystemP_SUCCESS)
    {
        /* Get Ring accelerator range for the core */
        memset (&req, 0, sizeof(req));
        memset (&res, 0, sizeof(res));

        req.type           = TISCI_DEV_DMASS0_RINGACC_0;
        req.subtype        = TISCI_RESASG_SUBTYPE_RA_GENERIC_IPC;
        req.secondary_host = TISCI_MSG_VALUE_RM_UNUSED_SECONDARY_HOST;

        status = Sciclient_rmGetResourceRange(&req, &res, SystemP_WAIT_FOREVER);

        if (((drvIndex*2) + MSI_RING_ACC_INDEX)  >= res.range_num)
        {
            DebugP_logError ("Unable to register MSI as Ring is not available\r\n");

            status = SystemP_FAILURE;
        }
    }

    /* Configure ring accelerator */
    if (status == SystemP_SUCCESS)
    {
        ring_index = res.range_start + (drvIndex*2) + MSI_RING_ACC_INDEX;

        memset (&rmRingReq, 0, sizeof(rmRingReq));
        memset (&rmRingReq, 0, sizeof(rmRingReq));

        rmRingReq.valid_params   =  TISCI_MSG_VALUE_RM_RING_ADDR_LO_VALID |
                                    TISCI_MSG_VALUE_RM_RING_ADDR_HI_VALID |
                                    TISCI_MSG_VALUE_RM_RING_COUNT_VALID |
                                    TISCI_MSG_VALUE_RM_RING_MODE_VALID |
                                    TISCI_MSG_VALUE_RM_RING_SIZE_VALID |
                                    TISCI_MSG_VALUE_RM_RING_ORDER_ID_VALID |
                                    TISCI_MSG_VALUE_RM_RING_ASEL_VALID;

        rmRingReq.nav_id = TISCI_DEV_DMASS0_RINGACC_0;
        rmRingReq.index = ring_index;
        rmRingReq.count = 1;
        rmRingReq.mode = TISCI_MSG_VALUE_RM_RING_MODE_MESSAGE;
        rmRingReq.size = TISCI_MSG_VALUE_RM_RING_SIZE_4B;
        rmRingReq.order_id = 1;
        rmRingReq.asel = 0;

        rmRingReq.addr_lo = (uint32_t)pcieCfg->attrs->msiRingMem;
        rmRingReq.addr_hi = (uint32_t)((uint64_t)(pcieCfg->attrs->msiRingMem) >> 32);

        status = Sciclient_rmRingCfg(
                        &rmRingReq, &rmRingResp, SystemP_WAIT_FOREVER);
    }

    /* Get global event for the core */
    if (status == SystemP_SUCCESS)
    {
        pcieCfg->attrs->msiRingNum = ring_index;

        memset (&req, 0, sizeof(req));
        memset (&res, 0, sizeof(res));

        req.type           = TISCI_DEV_DMASS0_INTAGGR_0;
        req.subtype        = TISCI_RESASG_SUBTYPE_GLOBAL_EVENT_SEVT;
        req.secondary_host = TISCI_MSG_VALUE_RM_UNUSED_SECONDARY_HOST;

        status = Sciclient_rmGetResourceRange(&req, &res, SystemP_WAIT_FOREVER);
    }

    if (status == SystemP_SUCCESS)
    {
        if (((drvIndex*2) + MSI_GLOBAL_EVENT_INDEX) >= res.range_num)
        {
            DebugP_logError ("Unable to register MSI as Global event is not available\r\n");
            status = SystemP_FAILURE;
        }
        else
        {
            global_event = res.range_start;
            global_event += (MSI_GLOBAL_EVENT_INDEX + (drvIndex*2));
        }
    }

    /* Get VINT resource ID for the core */
    if (status == SystemP_SUCCESS)
    {
        pcieCfg->attrs->msiGlobalEventNum = global_event;

        memset (&req, 0, sizeof(req));
        memset (&res, 0, sizeof(res));

        req.type           = TISCI_DEV_DMASS0_INTAGGR_0;
        req.subtype        = TISCI_RESASG_SUBTYPE_IA_VINT;
        req.secondary_host = TISCI_MSG_VALUE_RM_UNUSED_SECONDARY_HOST;

        status = Sciclient_rmGetResourceRange(&req, &res, SystemP_WAIT_FOREVER);
    }

    if (status == SystemP_SUCCESS)
    {
        if(((drvIndex*2) + MSI_VINT_INDEX) >= res.range_num)
        {
            DebugP_logError ("Unable to register MSI as Global event is not available\r\n");
            status = SystemP_FAILURE;
        }
        else
        {
            vint = res.range_start;
            vint += (MSI_VINT_INDEX + (drvIndex*2));
        }
    }

    if (status == SystemP_SUCCESS)
    {
        memset(&rmIrqReq, 0, sizeof(rmIrqReq));
        memset(&rmIrqResp, 0, sizeof(rmIrqResp));

        rmIrqReq.valid_params  |= TISCI_MSG_VALUE_RM_GLOBAL_EVENT_VALID;
        rmIrqReq.valid_params |= TISCI_MSG_VALUE_RM_IA_ID_VALID;
        rmIrqReq.valid_params |= TISCI_MSG_VALUE_RM_VINT_VALID;
        rmIrqReq.valid_params |= TISCI_MSG_VALUE_RM_VINT_STATUS_BIT_INDEX_VALID;

        rmIrqReq.src_id = TISCI_DEV_DMASS0_RINGACC_0;
        rmIrqReq.src_index = ring_index;

        rmIrqReq.ia_id = TISCI_DEV_DMASS0_INTAGGR_0;

        rmIrqReq.vint = vint;
        rmIrqReq.global_event = global_event;
        rmIrqReq.vint_status_bit_index = 0;

        status = Sciclient_rmIrqSet(&rmIrqReq, &rmIrqResp, SystemP_WAIT_FOREVER);

        if(status == SystemP_SUCCESS)
        {
            irqRouteSet = 1;
        }
    }

    /* Register Core interrupt */
    if (status == SystemP_SUCCESS)
    {
        /* Get self core ID to map VINT to core interrupt number */
        self_core = Sciclient_getSelfDevIdCore();

        /* Get core interrupt number for the corresponding VINT */
        status = Pcie_getIntNumForVint(self_core, vint, &intNum);

        pcieCfg->attrs->msiIntNum = intNum;

        HwiP_Params_init(&hwiParams);

        hwiParams.intNum = intNum;
        hwiParams.callback = Pcie_msiHandler;
        hwiParams.args = (void *)handle;
        hwiParams.isPulse = 0;

        status = HwiP_construct(&hwiObj, &hwiParams);
    }

    if (status == SystemP_SUCCESS)
    {
        memset(&regs, 0, sizeof(regs));

        regs.msiCap = &msiCapReg;
        regs.msiLo32 = &msiLo32;
        regs.msiUp32 = &msiUp32;
        regs.msiData = &msiData;

        status = Pcie_readRegs (handle, PCIE_LOCATION_REMOTE, &regs);

        if(status == SystemP_SUCCESS)
        {
            if (msiParams.enable)
            {
                pcieCfg->attrs->msiIrqEnableFlag = 1;
                msiCapReg.msiEn = 1;
            }
            msiData.data = msiParams.data;
            msiLo32.addr = msiParams.loAddr >> 2;
            msiUp32.addr = msiParams.upAddr;

            status = Pcie_writeRegs (handle, PCIE_LOCATION_REMOTE, &regs);
        }

    }

    /* Release IRQ and exit gracefuly in case of error */
    if ((status != SystemP_SUCCESS) && (irqRouteSet == 1))
    {
        memset(&rmIrqReleaseReq, 0, sizeof(rmIrqReleaseReq));

        rmIrqReleaseReq.valid_params  |= TISCI_MSG_VALUE_RM_GLOBAL_EVENT_VALID;
        rmIrqReleaseReq.valid_params |= TISCI_MSG_VALUE_RM_IA_ID_VALID;
        rmIrqReleaseReq.valid_params |= TISCI_MSG_VALUE_RM_VINT_VALID;
        rmIrqReleaseReq.valid_params |= TISCI_MSG_VALUE_RM_VINT_STATUS_BIT_INDEX_VALID;

        rmIrqReleaseReq.src_id = TISCI_DEV_DMASS0_RINGACC_0;
        rmIrqReleaseReq.src_index = ring_index;

        rmIrqReleaseReq.ia_id = TISCI_DEV_DMASS0_INTAGGR_0;

        rmIrqReleaseReq.vint = vint;
        rmIrqReleaseReq.global_event = global_event;
        rmIrqReleaseReq.vint_status_bit_index = 0;

        Sciclient_rmIrqRelease(&rmIrqReleaseReq, SystemP_WAIT_FOREVER);
    }

    return status;
}

int32_t Pcie_rcRegisterMsiIsr (Pcie_Handle handle, Pcie_RegisterMsiIsrParams params)
{
    int32_t status = SystemP_SUCCESS;

    Pcie_Config *pcieCfg;

    if(handle != NULL)
    {
        pcieCfg = (Pcie_Config *)handle;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    if(params.intNum > (PCIE_MAX_MSI_IRQ-1))
    {
        status = SystemP_FAILURE;
    }

    if (status == SystemP_SUCCESS)
    {
        pcieCfg->attrs->msiIsrCtrl->isr[params.intNum] = params.isr;
        pcieCfg->attrs->msiIsrCtrl->isrArgs[params.intNum] = params.arg;
    }

    return status;
}

int32_t Pcie_epSendMsiIrq(Pcie_Handle handle, uint32_t intNum, Pcie_sendMsiParams params)
{
    int32_t status = SystemP_SUCCESS;
    uint64_t msiAddr;

    if (handle == NULL)
    {
        status = SystemP_FAILURE;
    }

    if (intNum > (PCIE_MAX_MSI_IRQ-1))
    {
        status = SystemP_FAILURE;
    }

    if (params.addr == 0)
    {
        status = SystemP_FAILURE;
    }
    else
    {
        msiAddr = params.addr;
        msiAddr += PCIE_MSI_IRQ_ADDR_OFFSET;
    }

    if (status == SystemP_SUCCESS)
    {
        uint32_t data = (intNum & PCIE_MSI_IRQNUM_MASK) | (params.data & ~PCIE_MSI_IRQNUM_MASK);
        *((uintptr_t *)msiAddr) = data;
    }

    return status;
}

int32_t Pcie_rcEnableMSIX (Pcie_Handle handle, uint32_t drvIndex,
                            Pcie_MsixParams msixParams)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t self_core = 0;
    int32_t ring_index = 0;
    int32_t global_event = 0;
    int32_t vint = 0;
    uint32_t intNum = 0;
    uint32_t irqRouteSet = 0;
    HwiP_Params hwiParams;
    HwiP_Object hwiObj;

    struct tisci_msg_rm_get_resource_range_req  req;
    struct tisci_msg_rm_get_resource_range_resp res;

    struct tisci_msg_rm_ring_cfg_req    rmRingReq;
    struct tisci_msg_rm_ring_cfg_resp   rmRingResp;

    struct tisci_msg_rm_irq_set_req     rmIrqReq;
    struct tisci_msg_rm_irq_set_resp    rmIrqResp;

    struct tisci_msg_rm_irq_release_req rmIrqReleaseReq;

    Pcie_Config *pcieCfg;
    Pcie_Registers regs;
    Pcie_MsixCapReg msixCapReg;

    if (handle != NULL)
    {
        pcieCfg = (Pcie_Config *)handle;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    if (status == SystemP_SUCCESS)
    {
        /* Get Ring accelerator range for the core */
        memset (&req, 0, sizeof(req));
        memset (&res, 0, sizeof(res));

        req.type           = TISCI_DEV_DMASS0_RINGACC_0;
        req.subtype        = TISCI_RESASG_SUBTYPE_RA_GENERIC_IPC;
        req.secondary_host = TISCI_MSG_VALUE_RM_UNUSED_SECONDARY_HOST;

        status = Sciclient_rmGetResourceRange(&req, &res, SystemP_WAIT_FOREVER);

        if (((drvIndex*2) + MSIX_RING_ACC_INDEX) >= res.range_num)
        {
            DebugP_logError ("Unable to register MSIx as Ring is not available\r\n");

            status = SystemP_FAILURE;
        }
    }

    /* Configure ring accelerator */
    if (status == SystemP_SUCCESS)
    {
        ring_index = res.range_start + (drvIndex*2) + MSIX_RING_ACC_INDEX;

        memset (&rmRingReq, 0, sizeof(rmRingReq));
        memset (&rmRingReq, 0, sizeof(rmRingReq));

        rmRingReq.valid_params   =  TISCI_MSG_VALUE_RM_RING_ADDR_LO_VALID |
                                    TISCI_MSG_VALUE_RM_RING_ADDR_HI_VALID |
                                    TISCI_MSG_VALUE_RM_RING_COUNT_VALID |
                                    TISCI_MSG_VALUE_RM_RING_MODE_VALID |
                                    TISCI_MSG_VALUE_RM_RING_SIZE_VALID |
                                    TISCI_MSG_VALUE_RM_RING_ORDER_ID_VALID |
                                    TISCI_MSG_VALUE_RM_RING_ASEL_VALID;

        rmRingReq.nav_id = TISCI_DEV_DMASS0_RINGACC_0;
        rmRingReq.index = ring_index;
        rmRingReq.count = 1;
        rmRingReq.mode = TISCI_MSG_VALUE_RM_RING_MODE_MESSAGE;
        rmRingReq.size = TISCI_MSG_VALUE_RM_RING_SIZE_4B;
        rmRingReq.order_id = 1;
        rmRingReq.asel = 0;

        rmRingReq.addr_lo = (uint32_t)pcieCfg->attrs->msixRingMem;
        rmRingReq.addr_hi = (uint32_t)((uint64_t)(pcieCfg->attrs->msixRingMem) >> 32);

        status = Sciclient_rmRingCfg(
                        &rmRingReq, &rmRingResp, SystemP_WAIT_FOREVER);
    }

    /* Get global event for the core */
    if (status == SystemP_SUCCESS)
    {
        pcieCfg->attrs->msixRingNum = ring_index;

        memset (&req, 0, sizeof(req));
        memset (&res, 0, sizeof(res));

        req.type           = TISCI_DEV_DMASS0_INTAGGR_0;
        req.subtype        = TISCI_RESASG_SUBTYPE_GLOBAL_EVENT_SEVT;
        req.secondary_host = TISCI_MSG_VALUE_RM_UNUSED_SECONDARY_HOST;

        status = Sciclient_rmGetResourceRange(&req, &res, SystemP_WAIT_FOREVER);
    }

    if (status == SystemP_SUCCESS)
    {
        if (((drvIndex*2) + MSIX_GLOBAL_EVENT_INDEX) >= res.range_num)
        {
            DebugP_logError ("Unable to register MSIx as Global event is not available\r\n");
            status = SystemP_FAILURE;
        }
        else
        {
            global_event = res.range_start;
            global_event += (MSIX_GLOBAL_EVENT_INDEX + (drvIndex*2));
        }
    }

    /* Get VINT resource ID for the core */
    if (status == SystemP_SUCCESS)
    {
        pcieCfg->attrs->msiGlobalEventNum = global_event;

        memset (&req, 0, sizeof(req));
        memset (&res, 0, sizeof(res));

        req.type           = TISCI_DEV_DMASS0_INTAGGR_0;
        req.subtype        = TISCI_RESASG_SUBTYPE_IA_VINT;
        req.secondary_host = TISCI_MSG_VALUE_RM_UNUSED_SECONDARY_HOST;

        status = Sciclient_rmGetResourceRange(&req, &res, SystemP_WAIT_FOREVER);
    }

    if (status == SystemP_SUCCESS)
    {
        if(((drvIndex*2) + MSIX_VINT_INDEX) >= res.range_num)
        {
            DebugP_logError ("Unable to register MSIx as Global event is not available\r\n");
            status = SystemP_FAILURE;
        }
        else
        {
            vint = res.range_start;
            vint += (MSIX_VINT_INDEX + (drvIndex*2));
        }
    }

    if (status == SystemP_SUCCESS)
    {
        memset(&rmIrqReq, 0, sizeof(rmIrqReq));
        memset(&rmIrqResp, 0, sizeof(rmIrqResp));

        rmIrqReq.valid_params  |= TISCI_MSG_VALUE_RM_GLOBAL_EVENT_VALID;
        rmIrqReq.valid_params |= TISCI_MSG_VALUE_RM_IA_ID_VALID;
        rmIrqReq.valid_params |= TISCI_MSG_VALUE_RM_VINT_VALID;
        rmIrqReq.valid_params |= TISCI_MSG_VALUE_RM_VINT_STATUS_BIT_INDEX_VALID;

        rmIrqReq.src_id = TISCI_DEV_DMASS0_RINGACC_0;
        rmIrqReq.src_index = ring_index;

        rmIrqReq.ia_id = TISCI_DEV_DMASS0_INTAGGR_0;

        rmIrqReq.vint = vint;
        rmIrqReq.global_event = global_event;
        rmIrqReq.vint_status_bit_index = 0;

        status = Sciclient_rmIrqSet(&rmIrqReq, &rmIrqResp, SystemP_WAIT_FOREVER);

        if(status == SystemP_SUCCESS)
        {
            irqRouteSet = 1;
        }
    }

    /* Register Core interrupt */
    if (status == SystemP_SUCCESS)
    {
        /* Get self core ID to map VINT to core interrupt number */
        self_core = Sciclient_getSelfDevIdCore();

        /* Get core interrupt number for the corresponding VINT */
        status = Pcie_getIntNumForVint(self_core, vint, &intNum);

        pcieCfg->attrs->msixIntNum = intNum;

        HwiP_Params_init(&hwiParams);

        hwiParams.intNum = intNum;
        hwiParams.callback = Pcie_msixHandler;
        hwiParams.args = (void *)handle;
        hwiParams.isPulse = 0;

        status = HwiP_construct(&hwiObj, &hwiParams);
    }

    if (status == SystemP_SUCCESS)
    {
        memset(&regs, 0, sizeof(regs));

        regs.msixCap = &msixCapReg;

        status = Pcie_readRegs (handle, PCIE_LOCATION_REMOTE, &regs);

        if(status == SystemP_SUCCESS)
        {
            if (msixParams.enable)
            {
                pcieCfg->attrs->msixIrqEnableFlag = 1;
                msixCapReg.msixEn = 1;
            }

            status = Pcie_writeRegs (handle, PCIE_LOCATION_REMOTE, &regs);
        }

    }

    /* Release IRQ and exit gracefuly in case of error */
    if ((status != SystemP_SUCCESS) && (irqRouteSet == 1))
    {
        memset(&rmIrqReleaseReq, 0, sizeof(rmIrqReleaseReq));

        rmIrqReleaseReq.valid_params  |= TISCI_MSG_VALUE_RM_GLOBAL_EVENT_VALID;
        rmIrqReleaseReq.valid_params |= TISCI_MSG_VALUE_RM_IA_ID_VALID;
        rmIrqReleaseReq.valid_params |= TISCI_MSG_VALUE_RM_VINT_VALID;
        rmIrqReleaseReq.valid_params |= TISCI_MSG_VALUE_RM_VINT_STATUS_BIT_INDEX_VALID;

        rmIrqReleaseReq.src_id = TISCI_DEV_DMASS0_RINGACC_0;
        rmIrqReleaseReq.src_index = ring_index;

        rmIrqReleaseReq.ia_id = TISCI_DEV_DMASS0_INTAGGR_0;

        rmIrqReleaseReq.vint = vint;
        rmIrqReleaseReq.global_event = global_event;
        rmIrqReleaseReq.vint_status_bit_index = 0;

        Sciclient_rmIrqRelease(&rmIrqReleaseReq, SystemP_WAIT_FOREVER);
    }

    return status;
}

static void Pcie_msixHandler(void * args)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t msixData;
    Pcie_Config *pcieCfg;
    uint32_t ringNum;
    CSL_RingAccCfg pCfg;
    CSL_RingAccRingCfg pRing;
    uint32_t msixIrqNum;

    if(args != NULL)
    {
        pcieCfg = (Pcie_Config *)args;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    if (status == SystemP_SUCCESS)
    {
        ringNum = pcieCfg->attrs->msixRingNum;

        pCfg.pFifoRegs = (CSL_ringacc_fifosRegs *)CSL_DMASS0_RINGACC_SRC_FIFOS_BASE;
        pRing.ringNum = ringNum;
        pRing.mode = CSL_RINGACC_RING_MODE_MESSAGE;

        /* Read MSI data received by poping from the Ring accelerator */
        status = CSL_ringaccPop32(&pCfg, &pRing, &msixData, NULL);
    }

    if (status == SystemP_SUCCESS)
    {
        msixIrqNum = msixData & PCIE_MSIX_IRQNUM_MASK;

        if (pcieCfg->attrs->msixIsrCtrl->isr[msixIrqNum] != NULL)
        {
            pcieCfg->attrs->msixIsrCtrl->isr[msixIrqNum](pcieCfg->attrs->msixIsrCtrl->isrArgs[msixIrqNum],
                                                        msixData);
        }
    }
}

int32_t Pcie_rcRegisterMsixIsr (Pcie_Handle handle, Pcie_RegisterMsixIsrParams params)
{
    int32_t status = SystemP_SUCCESS;

    Pcie_Config *pcieCfg;

    if(handle != NULL)
    {
        pcieCfg = (Pcie_Config *)handle;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    if(params.intNum > (PCIE_MAX_MSIX_IRQ-1))
    {
        status = SystemP_FAILURE;
    }

    if (status == SystemP_SUCCESS)
    {
        pcieCfg->attrs->msixIsrCtrl->isr[params.intNum] = params.isr;
        pcieCfg->attrs->msixIsrCtrl->isrArgs[params.intNum] = params.arg;
    }

    return status;
}

int32_t Pcie_epSetMsixTblIntr (Pcie_Handle handle, Pcie_sendMsixParams params)
{
    int32_t status = SystemP_SUCCESS;
    Pcie_Config *cfg;

    if (handle == NULL)
    {
        status = SystemP_FAILURE;
    }

    if (params.intNum > (PCIE_MAX_MSIX_IRQ-1))
    {
        status = SystemP_FAILURE;
    }

    if (status == SystemP_SUCCESS)
    {
        cfg = (Pcie_Config *)handle;

        cfg->attrs->epMsixTbl->tbl[params.intNum].addr = params.addr;
        cfg->attrs->epMsixTbl->tbl[params.intNum].data = params.data;
    }

    return status;
}

int32_t Pcie_epSendMsixIrq(Pcie_Handle handle, uint32_t intNum, uint64_t addr)
{
    int32_t status = SystemP_SUCCESS;
    Pcie_Config *config;
    uint64_t msiAddr;
    uint32_t tblData;
    uint32_t writeData;

    if (handle == NULL)
    {
        status = SystemP_FAILURE;
    }
    else
    {
        config = (Pcie_Config *)handle;
    }

    if (intNum > (PCIE_MAX_MSIX_IRQ-1))
    {
        status = SystemP_FAILURE;
    }

    if (status == SystemP_SUCCESS)
    {
        if (addr == 0)
        {
            status = SystemP_FAILURE;
        }
        else
        {
            msiAddr = addr + config->attrs->epMsixTbl->tbl[intNum].addr;
            msiAddr += PCIE_MSIX_IRQ_ADDR_OFFSET;
        }

        tblData = config->attrs->epMsixTbl->tbl[intNum].data;
    }

    if (status == SystemP_SUCCESS)
    {
        writeData = (intNum & PCIE_MSIX_IRQNUM_MASK) | (tblData & ~PCIE_MSIX_IRQNUM_MASK);
        *((uintptr_t *)msiAddr) = writeData;
    }

    return status;
}
