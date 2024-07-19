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

#include <drivers/soc.h>
#include <kernel/dpl/AddrTranslateP.h>
#include <kernel/dpl/CpuIdP.h>

#define CSL_MAIN_CTRL_MMR_LOCKn_KICK0_OFFSET(n)   (0x1008 + 0x4000*(n))
#define CSL_MCU_CTRL_MMR_LOCKn_KICK0_OFFSET(n)    (0x1008 + 0x4000*(n))

/* Device type status register */
#define CSL_SYS_STATUS_REG              (0x44234100U)
#define CSL_SYS_STATUS_DEV_TYPE_MASK    (0x0000000FU)
#define CSL_SYS_STATUS_DEV_SUBTYPE_MASK (0x00000F00U)

#define CSL_SYS_STATUS_DEV_TYPE_GP      (0x03U)
#define CSL_SYS_STATUS_DEV_TYPE_TEST    (0x05U)
#define CSL_SYS_STATUS_DEV_SUBTYPE_FS   (0x00000A00U)

/* define the unlock and lock values */
#define KICK_LOCK_VAL                           (0x00000000U)
#define KICK0_UNLOCK_VAL                        (0x68EF3490U)
#define KICK1_UNLOCK_VAL                        (0xD172BC5AU)

/* PSC (Power Sleep Controller) timeout */
#define PSC_TIMEOUT                 (1000U)

/* PSC (Power Sleep Controller) Domain enable */
#define PSC_MODSTATE_ENABLE         (0x3U)

typedef struct
{
    uint32_t tcmaSize;
    uint32_t tcmbSize;

    CSL_ArmR5CPUInfo cpuInfo;

}SOC_VirtToPhyMap;

SOC_VirtToPhyMap virtToPhymap;

uint8_t isMapAvailable = 0u;

uint32_t SOC_isR5FDualCoreMode(CSL_ArmR5CPUInfo *cpuInfo)
{
    uint32_t retVal = FALSE, cpuId;
    int32_t status = SystemP_SUCCESS;
    uint32_t sciclientCpuProcId;
    struct tisci_msg_proc_get_status_resp proc_get_status;

    proc_get_status.config_flags_1 = 0;
    if ((cpuInfo->grpId == CSL_ARM_R5_CLUSTER_GROUP_ID_0) &&
        (cpuInfo->cpuID == CSL_ARM_R5_CPU_ID_0))
    {
        cpuId = CSL_CORE_ID_R5FSS0_0;
        sciclientCpuProcId = SCICLIENT_PROCID_R5_CL0_C0;
    }
    else
    {
        cpuId = CSL_CORE_ID_R5FSS0_1;
        sciclientCpuProcId = SCICLIENT_PROCID_R5_CL0_C1;
    }

    status = Sciclient_procBootGetProcessorState(sciclientCpuProcId,
                &proc_get_status,
                SystemP_WAIT_FOREVER);
    if(status != SystemP_SUCCESS)
    {
        DebugP_logError("CPU get state failed for %s\r\n", cpuId);
    }
    if(status == SystemP_SUCCESS)
    {
        if ((proc_get_status.config_flags_1 & TISCI_MSG_VAL_PROC_BOOT_CFG_FLAG_R5_SINGLE_CORE) !=
            TISCI_MSG_VAL_PROC_BOOT_CFG_FLAG_R5_SINGLE_CORE)
        {
            retVal = TRUE;
        }
    }
    return retVal;
}

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
int32_t SOC_moduleSetClockFrequencyWithParent(uint32_t moduleId, uint32_t clkId, uint32_t clkParent, uint64_t clkRate)
{
    int32_t status = SystemP_SUCCESS;
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
        /* Check if given parent is valid */
        if (clkParent > (clkId + numParents))
        {
            status = SystemP_FAILURE;
        }
    }
    if (status == SystemP_SUCCESS)
    {
        /* Check if a parent change is needed, if yes set to new parent */
        if (clkParent != origParent)
        {
            status = Sciclient_pmSetModuleClkParent(moduleId,
                                                        clkId,
                                                        clkParent,
                                                        SystemP_WAIT_FOREVER);
            if (status == SystemP_SUCCESS)
            {
                moduleClockParentChanged = 1U;
            }
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
        if (clockStatus == TISCI_MSG_VALUE_CLOCK_HW_STATE_NOT_READY)
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
        "r5f0-0",
        "r5f0-1",
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
        #ifndef SOC_AM65X /* in AM65x, main dowmin MMRs are left unlocked since when working with linux kernel, linux kernel assumes MMRs are unlocked */
        baseAddr = (uint32_t) AddrTranslateP_getLocalAddr(CSL_CTRL_MMR0_CFG0_BASE);
        kickAddr = (volatile uint32_t *) (baseAddr + CSL_MAIN_CTRL_MMR_LOCKn_KICK0_OFFSET(partition));
        CSL_REG32_WR(kickAddr, KICK_LOCK_VAL);      /* KICK 0 */
        kickAddr++;
        CSL_REG32_WR(kickAddr, KICK_LOCK_VAL);      /* KICK 1 */
        #endif
    }

    if(SOC_DOMAIN_ID_MCU == domainId)
    {
        baseAddr = (uint32_t) AddrTranslateP_getLocalAddr(CSL_MCU_CTRL_MMR0_CFG0_BASE);
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
        baseAddr = (uint32_t) AddrTranslateP_getLocalAddr(CSL_MCU_CTRL_MMR0_CFG0_BASE);
        kickAddr = (volatile uint32_t *) (baseAddr + CSL_MCU_CTRL_MMR_LOCKn_KICK0_OFFSET(partition));
        CSL_REG32_WR(kickAddr, KICK0_UNLOCK_VAL);   /* KICK 0 */
        kickAddr++;
        CSL_REG32_WR(kickAddr, KICK1_UNLOCK_VAL);   /* KICK 1 */
    }

    return;
}

uint64_t SOC_virtToPhy(void *virtAddr)
{
    uintptr_t   temp = (uintptr_t) virtAddr;
    uint64_t    phyAddr = (uint64_t) temp;                  /* Default case */

    /* R5F overrides */
#if (__ARM_ARCH == 7) && (__ARM_ARCH_PROFILE == 'R')
    uint32_t  retVal;

    if (0u == isMapAvailable)
    {
        /* Get Core ID Info */
        CSL_armR5GetCpuID(&virtToPhymap.cpuInfo);

        retVal = SOC_isR5FDualCoreMode(&virtToPhymap.cpuInfo);
        /* Dual Core Mode TCM Size is 128KB */

        if (virtToPhymap.cpuInfo.grpId == (uint32_t)CSL_ARM_R5_CLUSTER_GROUP_ID_0)
	    {
            if (retVal == TRUE)
            {
                virtToPhymap.tcmaSize = (CSL_MCU_ATCM_SIZE * 2U);
                virtToPhymap.tcmbSize = (CSL_MCU_BTCM_SIZE * 2U);
            }
            else
            {
                virtToPhymap.tcmaSize = (CSL_MCU_ATCM_SIZE);
                virtToPhymap.tcmbSize = (CSL_MCU_BTCM_SIZE);
            }
        }

        isMapAvailable = 1u;
    }

    if (virtToPhymap.cpuInfo.grpId == (uint32_t)CSL_ARM_R5_CLUSTER_GROUP_ID_0) /* R5SS0-0 */
    {
        if(virtToPhymap.cpuInfo.cpuID == CSL_ARM_R5_CPU_ID_0)
        {
            /* TCMA R5FSS0-0 */
            if(temp < (CSL_MCU_ATCM_BASE + virtToPhymap.tcmaSize))
            {
                phyAddr -= CSL_MCU_ATCM_BASE;
                phyAddr += CSL_MCU_ATCM_BASE;
            }

            /* TCMB R5FSS0-0 */
            else if((temp >= CSL_MCU_BTCM_BASE) &&
               (temp < (CSL_MCU_BTCM_BASE + virtToPhymap.tcmbSize)))
            {
                phyAddr -= CSL_MCU_BTCM_BASE;
                phyAddr += CSL_MCU_BTCM_BASE;
            }
        }
        else if(virtToPhymap.cpuInfo.cpuID == CSL_ARM_R5_CPU_ID_1)
        {
            /* TCMA R5FSS0-1 */
            if(temp < (CSL_MCU_ATCM_BASE + CSL_MCU_ATCM_SIZE))
            {
                phyAddr -= CSL_MCU_ATCM_BASE;
                phyAddr += CSL_MCU_ATCM_BASE;
            }

            /* TCMB R5FSS0-1 */
            else if((temp >= CSL_MCU_BTCM_BASE) &&
               (temp < (CSL_MCU_BTCM_BASE + CSL_MCU_BTCM_SIZE)))
            {
                phyAddr -= CSL_MCU_BTCM_BASE;
                phyAddr += CSL_MCU_BTCM_BASE;
            }
        }
    }

#endif

    return (phyAddr);
}

void *SOC_phyToVirt(uint64_t phyAddr)
{
    void       *virtAddr = (void *) ((uintptr_t) phyAddr);  /* Default case */

    /* R5F overrides */
#if (__ARM_ARCH == 7) && (__ARM_ARCH_PROFILE == 'R')
    uint32_t  retVal;

    if (0u == isMapAvailable)
    {
        /* Get Core ID Info */
        CSL_armR5GetCpuID(&virtToPhymap.cpuInfo);

        retVal = SOC_isR5FDualCoreMode(&virtToPhymap.cpuInfo);
        /* Dual Core Mode TCM Size is 128KB */

        if (virtToPhymap.cpuInfo.grpId == (uint32_t)CSL_ARM_R5_CLUSTER_GROUP_ID_0)
	    {
            if (retVal == TRUE)
            {
                virtToPhymap.tcmaSize = (CSL_MCU_ATCM_SIZE * 2U);
                virtToPhymap.tcmbSize = (CSL_MCU_BTCM_SIZE * 2U);
            }
            else
            {
                virtToPhymap.tcmaSize = (CSL_MCU_ATCM_SIZE);
                virtToPhymap.tcmbSize = (CSL_MCU_BTCM_SIZE);
            }
        }
        isMapAvailable = 1u;
    }


    if (virtToPhymap.cpuInfo.grpId == (uint32_t)CSL_ARM_R5_CLUSTER_GROUP_ID_0) /* R5SS0-0 */
    {
        if(virtToPhymap.cpuInfo.cpuID == CSL_ARM_R5_CPU_ID_0)
        {
            /* TCMA - R5FSS0-0 */
            if(phyAddr < (CSL_MCU_ATCM_BASE + virtToPhymap.tcmaSize))
            {
                phyAddr -= CSL_MCU_ATCM_BASE;
                phyAddr += CSL_MCU_ATCM_BASE;
                virtAddr = (void *) ((uintptr_t) phyAddr);
            }
            /* TCMB - R5FSS0-0 */
            else if((phyAddr >= CSL_MCU_BTCM_BASE) &&
               (phyAddr < (CSL_MCU_BTCM_BASE + virtToPhymap.tcmbSize)))
            {
                phyAddr -= CSL_MCU_BTCM_BASE;
                phyAddr += CSL_MCU_BTCM_BASE;
                virtAddr = (void *) ((uintptr_t) phyAddr);
            }
        }
        else if(virtToPhymap.cpuInfo.cpuID == CSL_ARM_R5_CPU_ID_1)
        {
            /* TCMA - R5FSS0-1 */
            if(phyAddr < (CSL_MCU_ATCM_BASE + CSL_MCU_ATCM_SIZE))
            {
                phyAddr -= CSL_MCU_ATCM_BASE;
                phyAddr += CSL_MCU_ATCM_BASE;
                virtAddr = (void *) ((uintptr_t) phyAddr);
            }
            /* TCMB - R5FSS0-1 */
            else if((phyAddr >= CSL_MCU_BTCM_BASE) &&
               (phyAddr < (CSL_MCU_BTCM_BASE + CSL_MCU_BTCM_SIZE)))
            {
                phyAddr -= CSL_MCU_BTCM_BASE;
                phyAddr += CSL_MCU_BTCM_BASE;
                virtAddr = (void *) ((uintptr_t) phyAddr);
            }
        }
    }

#endif

    return (virtAddr);
}

void SOC_setDevStat(uint32_t bootMode)
{
    /* Unlock CTLR_MMR0 registers */
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, 0);

    /* Change bootmode by setting devstat register */
    CSL_REG32_WR(CSL_CTRL_MMR0_CFG0_BASE + CSL_MAIN_CTRL_MMR_CFG0_MAIN_DEVSTAT, bootMode);

    /* Lock CTLR_MMR0 registers */
    SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MAIN, 0);

    return;
}

int32_t SOC_moduleGetClockFrequency(uint32_t moduleId, uint32_t clkId, uint64_t *clkRate)
{
    int32_t status = SystemP_SUCCESS;

    status = Sciclient_pmGetModuleClkFreq(moduleId,
                                            clkId, clkRate,
                                            SystemP_WAIT_FOREVER);

    return status;
}

void SOC_waitForFwlUnlock(void)
{
    uint32_t *psramPtr = (uint32_t *)AddrTranslateP_getLocalAddr(CSL_PSRAMECC0_RAM_BASE);

    while (CSL_REG32_RD(psramPtr) != SOC_FWL_OPEN_MAGIC_NUM);
}


int32_t SOC_isHsDevice(void)
{
    int32_t secDev = TRUE;
    int32_t devType = CSL_REG32_RD(CSL_SYS_STATUS_REG) & CSL_SYS_STATUS_DEV_TYPE_MASK;

    if((devType == CSL_SYS_STATUS_DEV_TYPE_GP) ||
       (devType == CSL_SYS_STATUS_DEV_TYPE_TEST))
    {
        secDev = FALSE;
    }
    else
    {
        secDev = TRUE;
    }

    return secDev;
}

uint32_t SOC_getFlashDataBaseAddr(void)
{
    return CSL_MCU_FSS0_DAT_REG1_BASE;
}