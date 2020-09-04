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

#include <drivers/watchdog.h>
#include <drivers/hw_include/cslr_soc.h>

void Watchdog_reset(Watchdog_Handle handle)
{
    CSL_mss_rcmRegs* ptrMSSRCMRegs = (CSL_mss_rcmRegs*)(CSL_MSS_RCM_U_BASE);
    CSL_dss_rcmRegs* ptrDSSRCMRegs = (CSL_dss_rcmRegs*)(CSL_DSS_RCM_U_BASE);

    if (CSL_FEXT(ptrMSSRCMRegs->MSS_WDT_RST_CTRL,
                 MSS_RCM_MSS_WDT_RST_CTRL_MSS_WDT_RST_CTRL_ASSERT) != 0)
    {
        CSL_FINS(ptrMSSRCMRegs->MSS_WDT_RST_CTRL,
                 MSS_RCM_MSS_WDT_RST_CTRL_MSS_WDT_RST_CTRL_ASSERT,
                 0x7U);
    }

    if (CSL_FEXT(ptrDSSRCMRegs->DSS_WDT_RST_CTRL,
                 DSS_RCM_DSS_WDT_RST_CTRL_DSS_WDT_RST_CTRL_ASSERT) != 0)
    {
        CSL_FINS(ptrDSSRCMRegs->DSS_WDT_RST_CTRL,
                 DSS_RCM_DSS_WDT_RST_CTRL_DSS_WDT_RST_CTRL_ASSERT,
                 0x7U);
    }

}

void Watchdog_configureWarmReset(Watchdog_Handle handle)
{
    CSL_mss_toprcmRegs* ptrTopRCMRegs = (CSL_mss_toprcmRegs*)CSL_MSS_TOPRCM_U_BASE;

    CSL_FINS(ptrTopRCMRegs->WARM_RESET_CONFIG,
             MSS_TOPRCM_WARM_RESET_CONFIG_WARM_RESET_CONFIG_WDOG_RST_EN,
             0x7U);

}
