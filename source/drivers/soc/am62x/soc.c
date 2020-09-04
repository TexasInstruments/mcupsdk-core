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

#include <drivers/soc.h>
#include <kernel/dpl/AddrTranslateP.h>

#define CSL_MAIN_CTRL_MMR_LOCKn_KICK0_OFFSET(n)   (0x1008 + 0x4000*(n))
#define CSL_MCU_CTRL_MMR_LOCKn_KICK0_OFFSET(n)    (0x1008 + 0x4000*(n))

/* define the unlock and lock values */
#define KICK_LOCK_VAL                           (0x00000000U)
#define KICK0_UNLOCK_VAL                        (0x68EF3490U)
#define KICK1_UNLOCK_VAL                        (0xD172BC5AU)

int32_t SOC_moduleClockEnable(uint32_t moduleId, uint32_t enable)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t moduleState = TISCI_MSG_VALUE_DEVICE_HW_STATE_TRANS;
    uint32_t resetState = 0U;
    uint32_t contextLossState = 0U;


    /* Get the module state.
       No need to change the module state if it
       is already in the required state
     */
    status = Sciclient_pmGetModuleState(moduleId,
                                        &moduleState,
                                        &resetState,
                                        &contextLossState,
                                        SystemP_WAIT_FOREVER);
    if(status == SystemP_SUCCESS)
    {
        if(moduleState == TISCI_MSG_VALUE_DEVICE_HW_STATE_OFF && (enable == 1))
        {
            /* enable the module */
            status = Sciclient_pmSetModuleState(moduleId,
                                                TISCI_MSG_VALUE_DEVICE_SW_STATE_ON,
                                                (TISCI_MSG_FLAG_AOP |
                                                TISCI_MSG_FLAG_DEVICE_RESET_ISO),
                                                SystemP_WAIT_FOREVER);
            if (status == SystemP_SUCCESS)
            {
                status = Sciclient_pmSetModuleRst(moduleId,
                                                0x0U,
                                                SystemP_WAIT_FOREVER);
            }
        }
        else
        if(moduleState == TISCI_MSG_VALUE_DEVICE_HW_STATE_ON && (enable == 0))
        {
            /* disable the module */
            status = Sciclient_pmSetModuleState(moduleId,
                                                TISCI_MSG_VALUE_DEVICE_SW_STATE_AUTO_OFF,
                                                (TISCI_MSG_FLAG_AOP),
                                                SystemP_WAIT_FOREVER);
        }
    }
    return status;
}

int32_t SOC_moduleSetClockFrequency(uint32_t moduleId, uint32_t clkId, uint64_t clkRate)
{
  int32_t status = SystemP_SUCCESS;
    uint32_t i = 0U;
    uint64_t respClkRate = 0;
    uint32_t numParents = 0U;
    uint32_t moduleClockParentChanged = 0U;
    uint32_t clockStatus = 0U;
    uint32_t origParent = 0U;
    uint32_t foundParent = 0U;

    /* Check if the clock is enabled or not */
    status = Sciclient_pmModuleGetClkStatus(moduleId,
                                            clkId,
                                            &clockStatus,
                                            SystemP_WAIT_FOREVER);
    if (status == SystemP_SUCCESS)
    {
        /* Get the number of parents for the clock */
        status = Sciclient_pmGetModuleClkNumParent(moduleId,
                                                   clkId,
                                                   &numParents,
                                                   SystemP_WAIT_FOREVER);
    }
    if (status == SystemP_SUCCESS)
    {
        if(numParents > 1U)
        {
            /* save the original parent to restore later */
            status = Sciclient_pmGetModuleClkParent(moduleId,
                                                    clkId,
                                                    &origParent,
                                                    SystemP_WAIT_FOREVER);
        }
    }
    if (status == SystemP_SUCCESS)
    {
        /* Disable the clock before changing the frequency */
        status = Sciclient_pmModuleClkRequest(moduleId,
                                              clkId,
                                              TISCI_MSG_VALUE_CLOCK_SW_STATE_UNREQ,
                                              0U,
                                              SystemP_WAIT_FOREVER);
    }
    if (status == SystemP_SUCCESS)
    {
        foundParent = 0U;
        /* For each parent query and check if frequency can be set at that parent */
        for(i=0U;i<numParents;i++)
        {
            if (numParents > 1U)
            {
                /* Setting the new parent */
                status = Sciclient_pmSetModuleClkParent(moduleId,
                                                        clkId,
                                                        clkId+i+1,
                                                        SystemP_WAIT_FOREVER);

                if (status == SystemP_SUCCESS)
                {
                    moduleClockParentChanged = 1U;
                }
            }
            if (status == SystemP_SUCCESS)
            {
                /* Check if the clock can be set to desired freq at this parent */
                status = Sciclient_pmQueryModuleClkFreq(moduleId,
                                                        clkId,
                                                        clkRate,
                                                        &respClkRate,
                                                        SystemP_WAIT_FOREVER);
            }
            if (status == SystemP_SUCCESS)
            {
                if(respClkRate == clkRate)
                {
                    /* yes, found a parent at which this frequency can be set */
                    foundParent = 1U;
                }
            }
            if(foundParent)
            {
                break; /* found a parent to set clock frequency, rebak form the loop */
            }
        }
    }
    if (status == SystemP_SUCCESS)
    {
        if(foundParent == 1U)
        {
            /* Set the clock at the desired frequency at the currently selected parent */
            status = Sciclient_pmSetModuleClkFreq(moduleId,
                                                  clkId,
                                                  clkRate,
                                                  TISCI_MSG_FLAG_CLOCK_ALLOW_FREQ_CHANGE,
                                                  SystemP_WAIT_FOREVER);
        }
        else
        {
            /* no parent found to set the desired frequency */
            status = SystemP_FAILURE;
        }

    }
    if (status == SystemP_SUCCESS)
    {
        if (clockStatus == TISCI_MSG_VALUE_CLOCK_SW_STATE_UNREQ)
        {
            /* Restore the clock again to original state */
            status = Sciclient_pmModuleClkRequest(moduleId,
                                                  clkId,
                                                  clockStatus,
                                                  0U,
                                                  SystemP_WAIT_FOREVER);
        }
    }
    if (status != SystemP_SUCCESS)
    {
        if (moduleClockParentChanged == 1U)
        {
            /* No parent found or some error, restore the parent to original value */
            Sciclient_pmSetModuleClkParent(moduleId,
                                           clkId,
                                           origParent,
                                           SystemP_WAIT_FOREVER);
            /* let the failure status be returned, so not checking status for this API call */
        }
    }
    return status;
}

const char *SOC_getCoreName(uint16_t coreId)
{
    static char *coreIdNames[CSL_CORE_ID_MAX+1] = {
        "m4f0-0",
        "r5f0-0",
        "a530-0",
        "a530-1",
        "a531-0",
        "a531-1",
        "unknown"
    };
    const char *name;

    if(coreId < CSL_CORE_ID_MAX)
    {
        name = coreIdNames[coreId];
    }
    else
    {
        name = coreIdNames[CSL_CORE_ID_MAX];
    }
    return name;
}

uint64_t SOC_getSelfCpuClk(void)
{
    uint64_t cpuClockRate = 0U;
    Sciclient_pmGetModuleClkFreq(
                    Sciclient_getSelfDevIdCore(),
                    0,
                    &cpuClockRate,
                    SystemP_WAIT_FOREVER);

    return cpuClockRate;
}

void SOC_controlModuleLockMMR(uint32_t domainId, uint32_t partition)
{
    uint32_t            baseAddr;
    volatile uint32_t  *kickAddr;

    if(SOC_DOMAIN_ID_MAIN == domainId)
    {
        #if 0 /* in AM62x, main dowmin MMRs are left unlocked since when working with linux kernel, linux kernel assumes MMRs are unlocked */
        baseAddr = (uint32_t) AddrTranslateP_getLocalAddr(CSL_CTRL_MMR0_CFG0_BASE);
        kickAddr = (volatile uint32_t *) (baseAddr + CSL_MAIN_CTRL_MMR_LOCKn_KICK0_OFFSET(partition));
        CSL_REG32_WR(kickAddr, KICK_LOCK_VAL);      /* KICK 0 */
        kickAddr++;
        CSL_REG32_WR(kickAddr, KICK_LOCK_VAL);      /* KICK 1 */
        #endif
    }

    if(SOC_DOMAIN_ID_MCU == domainId)
    {
        baseAddr = (uint32_t) AddrTranslateP_getLocalAddr(CSL_WKUP_CTRL_MMR1_CFG0_BASE);
        kickAddr = (volatile uint32_t *) (baseAddr + CSL_MCU_CTRL_MMR_LOCKn_KICK0_OFFSET(partition));
        CSL_REG32_WR(kickAddr, KICK_LOCK_VAL);      /* KICK 0 */
        kickAddr++;
        CSL_REG32_WR(kickAddr, KICK_LOCK_VAL);      /* KICK 1 */
    }

    return;
}

void SOC_controlModuleUnlockMMR(uint32_t domainId, uint32_t partition)
{
    uint32_t            baseAddr;
    volatile uint32_t  *kickAddr;

    if(SOC_DOMAIN_ID_MAIN == domainId)
    {
        baseAddr = (uint32_t) AddrTranslateP_getLocalAddr(CSL_CTRL_MMR0_CFG0_BASE);
        kickAddr = (volatile uint32_t *) (baseAddr + CSL_MAIN_CTRL_MMR_LOCKn_KICK0_OFFSET(partition));
        CSL_REG32_WR(kickAddr, KICK0_UNLOCK_VAL);   /* KICK 0 */
        kickAddr++;
        CSL_REG32_WR(kickAddr, KICK1_UNLOCK_VAL);   /* KICK 1 */
    }

    if(SOC_DOMAIN_ID_MCU == domainId)
    {
        baseAddr = (uint32_t) AddrTranslateP_getLocalAddr(CSL_WKUP_CTRL_MMR1_CFG0_BASE);
        kickAddr = (volatile uint32_t *) (baseAddr + CSL_MCU_CTRL_MMR_LOCKn_KICK0_OFFSET(partition));
        CSL_REG32_WR(kickAddr, KICK0_UNLOCK_VAL);   /* KICK 0 */
        kickAddr++;
        CSL_REG32_WR(kickAddr, KICK1_UNLOCK_VAL);   /* KICK 1 */
    }

    return;
}