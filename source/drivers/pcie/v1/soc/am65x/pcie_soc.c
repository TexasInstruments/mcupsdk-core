/*
 *  Copyright (C) 2024 Texas Instruments Incorporated
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
#include <drivers/pcie/v1/pcie_v1.h>
#include <drivers/pcie/pcie_priv.h>
#include <drivers/pcie/v1/pcie_v1_reg.h>
#include <drivers/hw_include/cslr_soc.h>
#include <drivers/hw_include/cslr_pcie.h>
#include <drivers/hw_include/cslr_ringacc.h>
#include <drivers/soc.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/HwiP.h>

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

Pcie_DeviceCfgBaseAddrs Pcie_cfgBaseAddrDev1 =
{
    (void *)CSL_PCIE0_DAT_BASE,
    /* The mapped offset is 0x10010000, but that points to registers
    that normally occur at base + 0x1000, hence the - 0x1000.
    No access to 0x0 to 0x0FFF is normal for other devices but
    they have separate structure that can be pointed here */
    (uint32_t)0x10000u - 0x1000u

};

Pcie_DeviceCfgBaseAddrs Pcie_cfgBaseAddrDev2 =
{
    (void *)CSL_PCIE1_DAT_BASE,
    /* The mapped offset is 0x10010000, but that points to registers
    that normally occur at base + 0x1000, hence the - 0x1000.
    No access to 0x0 to 0x0FFF is normal for other devices but
    they have separate structure that can be pointed here */
    (uint32_t)0x10000u - 0x1000u

};

Pcie_DevParams pcieDevParamsDev1 =
{
    (volatile uint32_t *)(CSL_CTRL_MMR0_CFG0_BASE + CSL_MAIN_CTRL_MMR_CFG0_PCIE0_CTRL)
};

Pcie_DevParams pcieDevParamsDev2 =
{
    (volatile uint32_t *)(CSL_CTRL_MMR0_CFG0_BASE + CSL_MAIN_CTRL_MMR_CFG0_PCIE1_CTRL)
};

Pcie_DeviceCfgBaseAddr pcieBaseAddrDev1 =
{
    (void *)&Pcie_cfgBaseAddrDev1,
    (void *)CSL_PCIE0_DAT0_BASE,
    0U,
    (void *)&pcieDevParamsDev1
};

Pcie_DeviceCfgBaseAddr pcieBaseAddrDev2 =
{
    (void *)&Pcie_cfgBaseAddrDev2,
    (void *)CSL_PCIE1_DAT0_BASE,
    0U,
    (void *)&pcieDevParamsDev2
};

Pcie_InitCfg Pcie_initCfg =
{
    {
        {
            &pcieBaseAddrDev1,
            &pcieBaseAddrDev2,   /* &pcieBaseAddrDev2, */
            NULL,                /* &pcieBaseAddrDev3, */
            NULL                 /* &pcieBaseAddrDev4 */
        },
    }
};

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
    swReg->lnkMode = 0;

    return SystemP_SUCCESS;
}

int32_t Pcie_writeLnkCtrlReg (CSL_pcie_ep_coreRegs *baseAddr, Pcie_LnkCtrlReg *swReg)
{
    return SystemP_SUCCESS;
}
