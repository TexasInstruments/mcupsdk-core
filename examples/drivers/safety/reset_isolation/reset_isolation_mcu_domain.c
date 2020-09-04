/*
 *  Copyright (C) 2018-2022 Texas Instruments Incorporated
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
#include <kernel/dpl/DebugP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "stdbool.h"
#include <drivers/soc.h>

void resetReqIsr(void *args)
{
    int32_t status;
    uint32_t resetSrc;
    uint32_t pscDomainState;
    uint32_t pscModuleStateMain2MCU = 0;
    uint32_t pscModuleStateMCU2Main = 0;

    /* Disable LPSC Main2MCU */
    status = SOC_getPSCState(SOC_PSC_DOMAIN_ID_MCU, CSL_MCU_GP_CORE_CTL_MCU,
            CSL_MCU_LPSC_MAIN2MCU, &pscDomainState, &pscModuleStateMain2MCU);
    if ((status == SystemP_SUCCESS) && (pscDomainState == SOC_PSC_DOMAIN_ON)
        && (pscModuleStateMain2MCU != SOC_PSC_DISABLE))
    {
        status = SOC_setPSCState(SOC_PSC_DOMAIN_ID_MCU, CSL_MCU_GP_CORE_CTL_MCU, \
                        CSL_MCU_LPSC_MAIN2MCU, SOC_PSC_DISABLE);
    }

    if (status == SystemP_SUCCESS)
    {
        /* Disable LPSC MCU2Main */
        status = SOC_getPSCState(SOC_PSC_DOMAIN_ID_MCU, CSL_MCU_GP_CORE_CTL_MCU,
                CSL_MCU_LPSC_MCU2MAIN, &pscDomainState, &pscModuleStateMCU2Main);
        if ((status == SystemP_SUCCESS) && (pscDomainState == SOC_PSC_DOMAIN_ON)
            && (pscModuleStateMCU2Main != SOC_PSC_DISABLE))
        {
            status = SOC_setPSCState(SOC_PSC_DOMAIN_ID_MCU, CSL_MCU_GP_CORE_CTL_MCU, \
                            CSL_MCU_LPSC_MCU2MAIN, SOC_PSC_DISABLE);
        }
    }

    if (status == SystemP_SUCCESS)
    {
        /* Clear reset source */
        resetSrc = SOC_getWarmResetCauseMcuDomain();
        SOC_clearResetCauseMainMcuDomain (resetSrc);

        /* Allow main domain reset to propogate */
        SOC_setMCUResetIsolationDone(0);

        SOC_waitMainDomainReset();

        /* Enable back reset isolation */
        SOC_setMCUResetIsolationDone(1);
    }

    /* Restore back previous state of LSPC Main2MCU */
    if ((status == SystemP_SUCCESS) && (pscModuleStateMain2MCU != SOC_PSC_DISABLE))
    {
        SOC_setPSCState(SOC_PSC_DOMAIN_ID_MCU, CSL_MCU_GP_CORE_CTL_MCU, \
                        CSL_MCU_LPSC_MAIN2MCU, pscModuleStateMain2MCU);
    }

    /* Restore back previous state of LSPC MCU2Main */
    if (((status == SystemP_SUCCESS)) && (pscModuleStateMCU2Main != SOC_PSC_DISABLE))
    {
        SOC_setPSCState(SOC_PSC_DOMAIN_ID_MCU, CSL_MCU_GP_CORE_CTL_MCU, \
                        CSL_MCU_LPSC_MCU2MAIN, pscModuleStateMCU2Main);
    }
}

void reset_isolation_main (void * args)
{
    int32_t status = SystemP_FAILURE;
    uint32_t pscMain2MCUDisable, pscMCU2MainDisable, debugIsolationEnable;

    Drivers_open();
    Board_driversOpen();

    /* Disabling Main2MCU PSC. This would restrict the main domain from accessing
    MCU domain peripherals/registers. Care must be taken no Main domain cores access
    MCU domain registers after this */
    pscMain2MCUDisable = 1;
    /* Disabling MCU2Main PSC. This would restrict the MCU domain from accessing
    Main domain peripherals/registers. Care must be taken no MCU domain cores access
    Main domain registers after this */
    pscMCU2MainDisable = 1;
    /* Enabling debug isolation will restrict the JTAG access to MCU domain */
    debugIsolationEnable = 1;

    status = SOC_enableResetIsolation (pscMain2MCUDisable, pscMCU2MainDisable, \
                                            debugIsolationEnable);
    DebugP_assert (status == SystemP_SUCCESS);

    {
        HwiP_Params resetHwiParams;
        HwiP_Object resetObject;

        HwiP_Params_init(&resetHwiParams);
        resetHwiParams.intNum = 25;
        resetHwiParams.callback = resetReqIsr;
        resetHwiParams.isPulse = 0;
        HwiP_construct(&resetObject, &resetHwiParams);
    }

    {
        uint32_t count = 0;

        volatile int x = 1;
        while(x)
        {
            ClockP_sleep(1);
            DebugP_log("I am running (M4) !!:- %d\r\n", count++);
        }
    }

    Board_driversClose();
    Drivers_close();
}
