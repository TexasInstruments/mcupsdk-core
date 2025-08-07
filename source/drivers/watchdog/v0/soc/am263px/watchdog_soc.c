/*
 *  Copyright (C) 2021 Texas Instruments Incorporated
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

/**
 *  \file   watchdog_soc.c
 *
 *  \brief  File containing SOC related APIs to enable WDT.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <drivers/soc.h>
#include <drivers/watchdog.h>
#include <drivers/hw_include/cslr_soc.h>

void Watchdog_reset(Watchdog_Handle handle)
{
    CSL_mss_rcmRegs* ptrRCMRegs = (CSL_mss_rcmRegs*)(CSL_MSS_RCM_U_BASE);

    Watchdog_Config*        ptrWatchdogConfig;
    Watchdog_HwAttrs*       ptrHwCfg;

    /* Get the Watchdog Configuration */
    ptrWatchdogConfig = (Watchdog_Config*)handle;

    /* Get the hardware configuration: */
    ptrHwCfg = (Watchdog_HwAttrs*)ptrWatchdogConfig->hwAttrs;

    uint8_t wdtInstance = ptrHwCfg->instance;

    switch (wdtInstance)
    {
        case WATCHDOG_INST_ID_0:
            if (CSL_FEXT(ptrRCMRegs->WDT0_RST_CTRL,
                 MSS_RCM_WDT0_RST_CTRL_ASSERT) != 0)
            {
                CSL_FINS(ptrRCMRegs->WDT0_RST_CTRL,
                         MSS_RCM_WDT0_RST_CTRL_ASSERT,
                         0x7U);
            }
            break;
        case WATCHDOG_INST_ID_1:
            if (CSL_FEXT(ptrRCMRegs->WDT1_RST_CTRL,
                 MSS_RCM_WDT1_RST_CTRL_ASSERT) != 0)
            {
                CSL_FINS(ptrRCMRegs->WDT1_RST_CTRL,
                         MSS_RCM_WDT1_RST_CTRL_ASSERT,
                         0x7U);
            }
            break;
        case WATCHDOG_INST_ID_2:
            if (CSL_FEXT(ptrRCMRegs->WDT2_RST_CTRL,
                 MSS_RCM_WDT2_RST_CTRL_ASSERT) != 0)
            {
                CSL_FINS(ptrRCMRegs->WDT2_RST_CTRL,
                         MSS_RCM_WDT2_RST_CTRL_ASSERT,
                         0x7U);
            }
            break;
        case WATCHDOG_INST_ID_3:
            if (CSL_FEXT(ptrRCMRegs->WDT3_RST_CTRL,
                 MSS_RCM_WDT3_RST_CTRL_ASSERT) != 0)
            {
                CSL_FINS(ptrRCMRegs->WDT3_RST_CTRL,
                         MSS_RCM_WDT3_RST_CTRL_ASSERT,
                         0x7U);
            }
            break;
    }

}

void Watchdog_configureWarmReset(Watchdog_Handle handle)
{
    CSL_top_rcmRegs* ptrTopRCMRegs = (CSL_top_rcmRegs*)CSL_TOP_RCM_U_BASE;

    Watchdog_Config*        ptrWatchdogConfig;
    Watchdog_HwAttrs*       ptrHwCfg;

    /* Get the Watchdog Configuration */
    ptrWatchdogConfig = (Watchdog_Config*)handle;

    /* Get the hardware configuration: */
    ptrHwCfg = (Watchdog_HwAttrs*)ptrWatchdogConfig->hwAttrs;

    uint8_t wdtInstance = ptrHwCfg->instance;

    switch (wdtInstance)
    {
        case WATCHDOG_INST_ID_0:
            /* Unlock TOP_RCM registers */
            SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, TOP_RCM_PARTITION0);

            CSL_FINS(ptrTopRCMRegs->WARM_RESET_CONFIG,
             TOP_RCM_WARM_RESET_CONFIG_WDOG0_RST_EN,
             0x7U);

            /* Lock TOP_RCM registers */
            SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MAIN, TOP_RCM_PARTITION0);
            break;
        case WATCHDOG_INST_ID_1:
            /* Unlock TOP_RCM registers */
            SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, TOP_RCM_PARTITION0);

            CSL_FINS(ptrTopRCMRegs->WARM_RESET_CONFIG,
             TOP_RCM_WARM_RESET_CONFIG_WDOG1_RST_EN,
             0x7U);
            
            /* Lock TOP_RCM registers */
            SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MAIN, TOP_RCM_PARTITION0);
            break;
        case WATCHDOG_INST_ID_2:
            /* Unlock TOP_RCM registers */
            SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, TOP_RCM_PARTITION0);

            CSL_FINS(ptrTopRCMRegs->WARM_RESET_CONFIG,
             TOP_RCM_WARM_RESET_CONFIG_WDOG2_RST_EN,
             0x7U);
            
            /* Lock TOP_RCM registers */
            SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MAIN, TOP_RCM_PARTITION0);
            break;
        case WATCHDOG_INST_ID_3:
            /* Unlock TOP_RCM registers */
            SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, TOP_RCM_PARTITION0);

            CSL_FINS(ptrTopRCMRegs->WARM_RESET_CONFIG,
             TOP_RCM_WARM_RESET_CONFIG_WDOG3_RST_EN,
             0x7U);
            
            /* Lock TOP_RCM registers */
            SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MAIN, TOP_RCM_PARTITION0);
            break;
    }

}
