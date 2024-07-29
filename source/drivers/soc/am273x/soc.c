/*
 *  Copyright (C) 2021-2023 Texas Instruments Incorporated
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

#include <drivers/hw_include/cslr_soc.h>
#include <drivers/soc.h>
#include <kernel/dpl/CpuIdP.h>

/* define the unlock and lock values */
#define KICK_LOCK_VAL                           (0x00000000U)
#define KICK0_UNLOCK_VAL                        (0x01234567U)
#define KICK1_UNLOCK_VAL                        (0x0FEDCBA8U)
#define IOMUX_KICK0_UNLOCK_VAL                  (0x83E70B13U)
#define IOMUX_KICK1_UNLOCK_VAL                  (0x95A4F1E0U)

#define ARM_M4F_VIRT_TO_PHY_OFFSET              (0x50000000U)

#define EDMA_M4F_VIRT_TO_PHY_OFFSET             (0x20020000U)

typedef struct
{
    uint32_t tcmaSize;
    uint32_t tcmbSize;

    CSL_ArmR5CPUInfo cpuInfo;

}SOC_VirtToPhyMap;

SOC_VirtToPhyMap virtToPhymap;
uint8_t isMapAvailable = 0u;

const char *SOC_getCoreName(uint16_t coreId)
{
    static char *coreIdNames[CSL_CORE_ID_MAX + 1] =
    {
        "r5f0-0",
        "r5f0-1",
        "c66ss0",
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

int32_t SOC_moduleClockEnable(uint32_t moduleId, uint32_t enable)
{
    int32_t status = SystemP_SUCCESS;

    return status;
}

int32_t SOC_moduleSetClockFrequency(uint32_t moduleId, uint32_t clkId, uint64_t clkRate)
{
    return (SOC_rcmSetPeripheralClock((SOC_RcmPeripheralId) moduleId, (SOC_RcmPeripheralClockSource) clkId, clkRate));
}

uint64_t SOC_getSelfCpuClk(void)
{
    uint64_t cpuClockRate = SOC_rcmGetR5Clock();

    return cpuClockRate;
}

void SOC_controlModuleLockMMR(uint32_t domainId, uint32_t partition)
{
    uint32_t            baseAddr;
    volatile uint32_t  *kickAddr;

    if(SOC_DOMAIN_ID_MSS_TOP_RCM == domainId)
    {
        baseAddr = CSL_MSS_TOPRCM_U_BASE;
        kickAddr = (volatile uint32_t *) (baseAddr + CSL_MSS_TOPRCM_LOCK0_KICK0);
        CSL_REG32_WR(kickAddr, KICK_LOCK_VAL);      /* KICK 0 */
        kickAddr = (volatile uint32_t *) (baseAddr + CSL_MSS_TOPRCM_LOCK0_KICK1);
        CSL_REG32_WR(kickAddr, KICK_LOCK_VAL);      /* KICK 1 */
    }

    if(SOC_DOMAIN_ID_MSS_RCM == domainId)
    {
        baseAddr = CSL_MSS_RCM_U_BASE;
        kickAddr = (volatile uint32_t *) (baseAddr + CSL_MSS_RCM_LOCK0_KICK0);
        CSL_REG32_WR(kickAddr, KICK_LOCK_VAL);      /* KICK 0 */
        kickAddr = (volatile uint32_t *) (baseAddr + CSL_MSS_RCM_LOCK0_KICK1);
        CSL_REG32_WR(kickAddr, KICK_LOCK_VAL);      /* KICK 1 */
    }

    if(SOC_DOMAIN_ID_DSS_RCM == domainId)
    {
        baseAddr = CSL_DSS_RCM_U_BASE;
        kickAddr = (volatile uint32_t *) (baseAddr + CSL_DSS_RCM_LOCK0_KICK0);
        CSL_REG32_WR(kickAddr, KICK_LOCK_VAL);      /* KICK 0 */
        kickAddr = (volatile uint32_t *) (baseAddr + CSL_DSS_RCM_LOCK0_KICK1);
        CSL_REG32_WR(kickAddr, KICK_LOCK_VAL);      /* KICK 1 */
    }

    if(SOC_DOMAIN_ID_RCSS_RCM == domainId)
    {
        baseAddr = CSL_RCSS_RCM_U_BASE;
        kickAddr = (volatile uint32_t *) (baseAddr + CSL_RCSS_RCM_LOCK0_KICK0);
        CSL_REG32_WR(kickAddr, KICK_LOCK_VAL);      /* KICK 0 */
        kickAddr = (volatile uint32_t *) (baseAddr + CSL_RCSS_RCM_LOCK0_KICK1);
        CSL_REG32_WR(kickAddr, KICK_LOCK_VAL);      /* KICK 1 */
    }

    if(SOC_DOMAIN_ID_MSS_CTRL == domainId)
    {
        baseAddr = CSL_MSS_CTRL_U_BASE;
        kickAddr = (volatile uint32_t *) (baseAddr + CSL_MSS_CTRL_LOCK0_KICK0);
        CSL_REG32_WR(kickAddr, KICK_LOCK_VAL);      /* KICK 0 */
        kickAddr = (volatile uint32_t *) (baseAddr + CSL_MSS_CTRL_LOCK0_KICK1);
        CSL_REG32_WR(kickAddr, KICK_LOCK_VAL);      /* KICK 1 */
    }

    if(SOC_DOMAIN_ID_DSS_CTRL == domainId)
    {
        baseAddr = CSL_DSS_CTRL_U_BASE;
        kickAddr = (volatile uint32_t *) (baseAddr + CSL_DSS_CTRL_LOCK0_KICK0);
        CSL_REG32_WR(kickAddr, KICK_LOCK_VAL);      /* KICK 0 */
        kickAddr = (volatile uint32_t *) (baseAddr + CSL_DSS_CTRL_LOCK0_KICK1);
        CSL_REG32_WR(kickAddr, KICK_LOCK_VAL);      /* KICK 1 */
    }

    if(SOC_DOMAIN_ID_RCSS_CTRL == domainId)
    {
        baseAddr = CSL_RCSS_CTRL_U_BASE;
        kickAddr = (volatile uint32_t *) (baseAddr + CSL_RCSS_CTRL_LOCK0_KICK0);
        CSL_REG32_WR(kickAddr, KICK_LOCK_VAL);      /* KICK 0 */
        kickAddr = (volatile uint32_t *) (baseAddr + CSL_RCSS_CTRL_LOCK0_KICK1);
        CSL_REG32_WR(kickAddr, KICK_LOCK_VAL);      /* KICK 1 */
    }

    if(SOC_DOMAIN_ID_MSS_IOMUX == domainId)
    {
        baseAddr = CSL_MSS_IOMUX_U_BASE;
        kickAddr = (volatile uint32_t *) (baseAddr + CSL_MSS_IOMUX_IOCFGKICK0);
        CSL_REG32_WR(kickAddr, KICK_LOCK_VAL);      /* KICK 0 */
        kickAddr = (volatile uint32_t *) (baseAddr + CSL_MSS_IOMUX_IOCFGKICK1);
        CSL_REG32_WR(kickAddr, KICK_LOCK_VAL);      /* KICK 1 */
    }

    return;
}

void SOC_controlModuleUnlockMMR(uint32_t domainId, uint32_t partition)
{
    uint32_t            baseAddr;
    volatile uint32_t  *kickAddr;

    if(SOC_DOMAIN_ID_MSS_TOP_RCM == domainId)
    {
        baseAddr = CSL_MSS_TOPRCM_U_BASE;
        kickAddr = (volatile uint32_t *) (baseAddr + CSL_MSS_TOPRCM_LOCK0_KICK0);
        CSL_REG32_WR(kickAddr, KICK0_UNLOCK_VAL);   /* KICK 0 */
        kickAddr = (volatile uint32_t *) (baseAddr + CSL_MSS_TOPRCM_LOCK0_KICK1);
        CSL_REG32_WR(kickAddr, KICK1_UNLOCK_VAL);   /* KICK 1 */
    }

    if(SOC_DOMAIN_ID_MSS_RCM == domainId)
    {
        baseAddr = CSL_MSS_RCM_U_BASE;
        kickAddr = (volatile uint32_t *) (baseAddr + CSL_MSS_RCM_LOCK0_KICK0);
        CSL_REG32_WR(kickAddr, KICK0_UNLOCK_VAL);   /* KICK 0 */
        kickAddr = (volatile uint32_t *) (baseAddr + CSL_MSS_RCM_LOCK0_KICK1);
        CSL_REG32_WR(kickAddr, KICK1_UNLOCK_VAL);   /* KICK 1 */
    }

    if(SOC_DOMAIN_ID_DSS_RCM == domainId)
    {
        baseAddr = CSL_DSS_RCM_U_BASE;
        kickAddr = (volatile uint32_t *) (baseAddr + CSL_DSS_RCM_LOCK0_KICK0);
        CSL_REG32_WR(kickAddr, KICK0_UNLOCK_VAL);   /* KICK 0 */
        kickAddr = (volatile uint32_t *) (baseAddr + CSL_DSS_RCM_LOCK0_KICK1);
        CSL_REG32_WR(kickAddr, KICK1_UNLOCK_VAL);   /* KICK 1 */
    }

    if(SOC_DOMAIN_ID_RCSS_RCM == domainId)
    {
        baseAddr = CSL_RCSS_RCM_U_BASE;
        kickAddr = (volatile uint32_t *) (baseAddr + CSL_RCSS_RCM_LOCK0_KICK0);
        CSL_REG32_WR(kickAddr, KICK0_UNLOCK_VAL);   /* KICK 0 */
        kickAddr = (volatile uint32_t *) (baseAddr + CSL_RCSS_RCM_LOCK0_KICK1);
        CSL_REG32_WR(kickAddr, KICK1_UNLOCK_VAL);   /* KICK 1 */
    }

    if(SOC_DOMAIN_ID_MSS_CTRL == domainId)
    {
        baseAddr = CSL_MSS_CTRL_U_BASE;
        kickAddr = (volatile uint32_t *) (baseAddr + CSL_MSS_CTRL_LOCK0_KICK0);
        CSL_REG32_WR(kickAddr, KICK0_UNLOCK_VAL);   /* KICK 0 */
        kickAddr = (volatile uint32_t *) (baseAddr + CSL_MSS_CTRL_LOCK0_KICK1);
        CSL_REG32_WR(kickAddr, KICK1_UNLOCK_VAL);   /* KICK 1 */
    }

    if(SOC_DOMAIN_ID_DSS_CTRL == domainId)
    {
        baseAddr = CSL_DSS_CTRL_U_BASE;
        kickAddr = (volatile uint32_t *) (baseAddr + CSL_DSS_CTRL_LOCK0_KICK0);
        CSL_REG32_WR(kickAddr, KICK0_UNLOCK_VAL);   /* KICK 0 */
        kickAddr = (volatile uint32_t *) (baseAddr + CSL_DSS_CTRL_LOCK0_KICK1);
        CSL_REG32_WR(kickAddr, KICK1_UNLOCK_VAL);   /* KICK 1 */
    }

    if(SOC_DOMAIN_ID_RCSS_CTRL == domainId)
    {
        baseAddr = CSL_RCSS_CTRL_U_BASE;
        kickAddr = (volatile uint32_t *) (baseAddr + CSL_RCSS_CTRL_LOCK0_KICK0);
        CSL_REG32_WR(kickAddr, KICK0_UNLOCK_VAL);   /* KICK 0 */
        kickAddr = (volatile uint32_t *) (baseAddr + CSL_RCSS_CTRL_LOCK0_KICK1);
        CSL_REG32_WR(kickAddr, KICK1_UNLOCK_VAL);   /* KICK 1 */
    }

    if(SOC_DOMAIN_ID_MSS_IOMUX == domainId)
    {
        baseAddr = CSL_MSS_IOMUX_U_BASE;
        kickAddr = (volatile uint32_t *) (baseAddr + CSL_MSS_IOMUX_IOCFGKICK0);
        CSL_REG32_WR(kickAddr, IOMUX_KICK0_UNLOCK_VAL); /* KICK 0 */
        kickAddr = (volatile uint32_t *) (baseAddr + CSL_MSS_IOMUX_IOCFGKICK1);
        CSL_REG32_WR(kickAddr, IOMUX_KICK1_UNLOCK_VAL); /* KICK 1 */
    }

    return;
}

void SOC_setEpwmTbClk(uint32_t epwmInstance, uint32_t enable)
{
    if(epwmInstance < CSL_EPWM_PER_CNT)
    {

        uint32_t epwmPartition = 1;
        SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MSS_RCM, epwmPartition);
        uint32_t bitPositionOffset = 24;
        if(TRUE == enable)
        {
            /* Enable Time base clock in CTRL MMR */
            CSL_REG32_WR(CSL_MSS_CTRL_U_BASE + CSL_MSS_CTRL_MSS_EPWM_CFG,
                ((CSL_REG32_RD(CSL_MSS_CTRL_U_BASE +
                  CSL_MSS_CTRL_MSS_EPWM_CFG) & 0x7000000) | (1 << (epwmInstance+ bitPositionOffset))));
        }
        else
        {
            /* Disable Time base clock in CTRL MMR */
            CSL_REG32_WR(CSL_MSS_CTRL_U_BASE + CSL_MSS_CTRL_MSS_EPWM_CFG,
                ((CSL_REG32_RD(CSL_MSS_CTRL_U_BASE +
                  CSL_MSS_CTRL_MSS_EPWM_CFG) & 0x7000000) & ~(1 << (epwmInstance+ bitPositionOffset))));
        }

        /* Lock CTRL_MMR0 registers */
        SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MSS_RCM, epwmPartition);
    }
}



uint64_t SOC_virtToPhy(void *virtAddr)
{
    uintptr_t   temp = (uintptr_t) virtAddr;
    uint64_t    phyAddr = (uint64_t) temp;                  /* Default case */

    /* R5F overrides */
#if (__ARM_ARCH == 7) && (__ARM_ARCH_PROFILE == 'R')
    uint32_t    retVal;

    if (0u == isMapAvailable)
    {
        /* Get Core ID Info */
        CSL_armR5GetCpuID(&virtToPhymap.cpuInfo);

        retVal = SOC_rcmIsR5FInLockStepMode(virtToPhymap.cpuInfo.grpId);
        /* LockStep Mode TCM Size is 64KB */

        if (virtToPhymap.cpuInfo.grpId == (uint32_t)CSL_ARM_R5_CLUSTER_GROUP_ID_0)
	    {
            if (retVal == TRUE)
            {
                virtToPhymap.tcmaSize = (CSL_MSS_TCMA_RAM_SIZE * 2U);
                virtToPhymap.tcmbSize = (CSL_MSS_TCMB_RAM_SIZE * 2U);
            }
            else
            {
                virtToPhymap.tcmaSize = (CSL_MSS_TCMA_RAM_SIZE);
                virtToPhymap.tcmbSize = (CSL_MSS_TCMB_RAM_SIZE);
            }
        }

        isMapAvailable = 1u;
    }

    if (virtToPhymap.cpuInfo.grpId == (uint32_t)CSL_ARM_R5_CLUSTER_GROUP_ID_0) /* R5SS0-0 */
    {
        if(virtToPhymap.cpuInfo.cpuID == CSL_ARM_R5_CPU_ID_0)
        {
            /* TCMA R5FSS0-0 */
            if((temp >= CSL_MSS_TCMA_RAM_BASE) &&
               (temp < (CSL_MSS_TCMA_RAM_BASE + virtToPhymap.tcmaSize)))
            {
                phyAddr -= CSL_MSS_TCMA_RAM_BASE;
                phyAddr += CSL_MSS_TCMA_CR5A_U_BASE;
            }

            /* TCMB R5FSS0-0 */
            else if((temp >= CSL_MSS_TCMB_RAM_BASE) &&
               (temp < (CSL_MSS_TCMB_RAM_BASE + virtToPhymap.tcmbSize)))
            {
                phyAddr -= CSL_MSS_TCMB_RAM_BASE;
                phyAddr += CSL_MSS_TCMB_CR5A_U_BASE;
            }
        }
        else if(virtToPhymap.cpuInfo.cpuID == CSL_ARM_R5_CPU_ID_1)
        {
            /* TCMA R5FSS0-1 */
            if((temp >= CSL_MSS_TCMA_RAM_BASE) &&
               (temp < (CSL_MSS_TCMA_RAM_BASE + CSL_MSS_TCMA_RAM_SIZE)))
            {
                phyAddr -= CSL_MSS_TCMA_RAM_BASE;
                phyAddr += CSL_MSS_TCMA_CR5B_U_BASE;
            }

            /* TCMB R5FSS0-1 */
            else if((temp >= CSL_MSS_TCMB_RAM_BASE) &&
               (temp < (CSL_MSS_TCMB_RAM_BASE + CSL_MSS_TCMB_RAM_SIZE)))
            {
                phyAddr -= CSL_MSS_TCMB_RAM_BASE;
                phyAddr += CSL_MSS_TCMB_CR5B_U_BASE;
            }
        }
    }

    /* MSS L2 */
    if((temp >= CSL_MSS_L2_RAM_BASE) &&
       (temp < (CSL_MSS_L2_RAM_BASE + CSL_MSS_L2_RAM_SIZE)))
    {
        phyAddr -= CSL_MSS_L2_RAM_BASE;
        phyAddr += CSL_GLOB_MSS_L2_RAM_BASE;
    }

    /* DSS L2 and L3 - same view and no change needed */
#endif

    /* C66x overrides */
#if defined(_TMS320C6X)
    /* L1P */
    if((temp >= CSL_DSP_L1P_U_BASE) &&
       (temp < (CSL_DSP_L1P_U_BASE + CSL_DSP_L1P_SIZE)))
    {
        phyAddr -= CSL_DSP_L1P_U_BASE;
        phyAddr += CSL_GLOB_DSP_L1P_U_BASE;
    }
    /* L1D */
    else if((temp >= CSL_DSP_L1D_U_BASE) &&
       (temp < (CSL_DSP_L1D_U_BASE + CSL_DSP_L1D_SIZE)))
    {
        phyAddr -= CSL_DSP_L1D_U_BASE;
        phyAddr += CSL_GLOB_DSP_L1D_U_BASE;
    }
    /* L2 */
    else if((temp >= CSL_DSP_L2_U_BASE) &&
       (temp < (CSL_DSP_L2_U_BASE + CSL_DSP_L2_RAM_SIZE)))
    {
        phyAddr -= CSL_DSP_L2_U_BASE;
        phyAddr += CSL_GLOB_DSP_L2_U_BASE;
    }

    /* DSS L3 and MSS L2 - same view and no change needed */
#endif

#if (__ARM_ARCH == 7) && (__ARM_ARCH_PROFILE == 'M')

    if ( ((temp >=  CSL_MSS_SPIA_RAM_U_BASE) &&
        (temp < CSL_MSS_MCANA_CFG_U_BASE)) ||
        ((temp >= CSL_MSS_TPCC_A_U_BASE) &&
        (temp < CSL_MSS_ETPWMC_U_BASE)) )
    {
        phyAddr += ARM_M4F_VIRT_TO_PHY_OFFSET;
    }

    if ( temp >= (CSL_HSM_M4_RAM_BASE - CSL_HSM_M4_RAM_BASE) && 
        (temp < CSL_HSM_M4_RAM_SIZE))
    {
        phyAddr += EDMA_M4F_VIRT_TO_PHY_OFFSET;
    }

#endif

    return (phyAddr);
}

void *SOC_phyToVirt(uint64_t phyAddr)
{
    void       *virtAddr = (void *) ((uintptr_t) phyAddr);  /* Default case */

    /* R5F overrides */
#if (__ARM_ARCH == 7) && (__ARM_ARCH_PROFILE == 'R')
    uint32_t    retVal;

    if (0u == isMapAvailable)
    {
        /* Get Core ID Info */
        CSL_armR5GetCpuID(&virtToPhymap.cpuInfo);

        retVal = SOC_rcmIsR5FInLockStepMode(virtToPhymap.cpuInfo.grpId);
        /* LockStep Mode TCM Size is 64KB */

        if (virtToPhymap.cpuInfo.grpId == (uint32_t)CSL_ARM_R5_CLUSTER_GROUP_ID_0)
	    {
            if (retVal == TRUE)
            {
                virtToPhymap.tcmaSize = (CSL_MSS_TCMA_RAM_SIZE * 2U);
                virtToPhymap.tcmbSize = (CSL_MSS_TCMB_RAM_SIZE * 2U);
            }
            else
            {
                virtToPhymap.tcmaSize = (CSL_MSS_TCMA_RAM_SIZE);
                virtToPhymap.tcmbSize = (CSL_MSS_TCMB_RAM_SIZE);
            }
        }

        isMapAvailable = 1u;
    }

    if (virtToPhymap.cpuInfo.grpId == (uint32_t)CSL_ARM_R5_CLUSTER_GROUP_ID_0) /* R5SS0-0 */
    {
        if(virtToPhymap.cpuInfo.cpuID == CSL_ARM_R5_CPU_ID_0)
        {
            /* TCMA - R5FSS0-0 */
            if((phyAddr >= CSL_MSS_TCMA_CR5A_U_BASE) &&
               (phyAddr < (CSL_MSS_TCMA_CR5A_U_BASE + virtToPhymap.tcmaSize)))
            {
                phyAddr -= CSL_MSS_TCMA_CR5A_U_BASE;
                phyAddr += CSL_MSS_TCMA_RAM_BASE;
                virtAddr = (void *) ((uintptr_t) phyAddr);
            }
            /* TCMB - R5FSS0-0 */
            else if((phyAddr >= CSL_MSS_TCMB_CR5A_U_BASE) &&
               (phyAddr < (CSL_MSS_TCMB_CR5A_U_BASE + virtToPhymap.tcmbSize)))
            {
                phyAddr -= CSL_MSS_TCMB_CR5A_U_BASE;
                phyAddr += CSL_MSS_TCMB_RAM_BASE;
                virtAddr = (void *) ((uintptr_t) phyAddr);
            }
        }
        else if(virtToPhymap.cpuInfo.cpuID == CSL_ARM_R5_CPU_ID_1)
        {
            /* TCMA - R5FSS0-1 */
            if((phyAddr >= CSL_MSS_TCMA_CR5B_U_BASE) &&
               (phyAddr < (CSL_MSS_TCMA_CR5B_U_BASE + CSL_MSS_TCMA_RAM_SIZE)))
            {
                phyAddr -= CSL_MSS_TCMA_CR5B_U_BASE;
                phyAddr += CSL_MSS_TCMA_RAM_BASE;

                virtAddr = (void *) ((uintptr_t) phyAddr);
            }
            /* TCMB - R5FSS0-1 */
            else if((phyAddr >= CSL_MSS_TCMB_CR5B_U_BASE) &&
               (phyAddr < (CSL_MSS_TCMB_CR5B_U_BASE + CSL_MSS_TCMB_RAM_SIZE)))
            {
                phyAddr -= CSL_MSS_TCMB_CR5B_U_BASE;
                phyAddr += CSL_MSS_TCMB_RAM_BASE;
                virtAddr = (void *) ((uintptr_t) phyAddr);
            }
        }
    }

    /* MSS L2 */
    if((phyAddr >= CSL_GLOB_MSS_L2_RAM_BASE) &&
       (phyAddr < (CSL_GLOB_MSS_L2_RAM_BASE + CSL_MSS_L2_RAM_SIZE)))
    {
        phyAddr -= CSL_GLOB_MSS_L2_RAM_BASE;
        phyAddr += CSL_MSS_L2_RAM_BASE;
        virtAddr = (void *) ((uintptr_t) phyAddr);
    }

    /* DSS L2 and L3 - same view and no change needed */
#endif

    /* C66x overrides */
#if defined(_TMS320C6X)
    /* L1P */
    if((phyAddr >= CSL_GLOB_DSP_L1P_U_BASE) &&
       (phyAddr < (CSL_GLOB_DSP_L1P_U_BASE + CSL_DSP_L1P_SIZE)))
    {
        phyAddr -= CSL_GLOB_DSP_L1P_U_BASE;
        phyAddr += CSL_DSP_L1P_U_BASE;
        virtAddr = (void *) ((uintptr_t) phyAddr);
    }
    /* L1D */
    else if((phyAddr >= CSL_GLOB_DSP_L1D_U_BASE) &&
       (phyAddr < (CSL_GLOB_DSP_L1D_U_BASE + CSL_DSP_L1D_SIZE)))
    {
        phyAddr -= CSL_GLOB_DSP_L1D_U_BASE;
        phyAddr += CSL_DSP_L1D_U_BASE;
        virtAddr = (void *) ((uintptr_t) phyAddr);
    }
    /* L2 */
    else if((phyAddr >= CSL_GLOB_DSP_L2_U_BASE) &&
       (phyAddr < (CSL_GLOB_DSP_L2_U_BASE + CSL_DSP_L2_RAM_SIZE)))
    {
        phyAddr -= CSL_GLOB_DSP_L2_U_BASE;
        phyAddr += CSL_DSP_L2_U_BASE;
        virtAddr = (void *) ((uintptr_t) phyAddr);
    }

    /* DSS L3 and MSS L2 - same view and no change needed */
#endif

    return (virtAddr);
}

void SOC_logAllClockHz(void)
{
    uint32_t clkHz;

    clkHz = SOC_rcmGetR5Clock();
    DebugP_log("R5F         = %9d Hz\r\n", clkHz);
    clkHz = SOC_rcmGetDspClock();
    DebugP_log("DSP         = %9d Hz\r\n", clkHz);
    clkHz = SOC_rcmGetPeripheralClock(SOC_RcmPeripheralId_CSIRX);
    DebugP_log("CSIRX       = %9d Hz\r\n", clkHz);
    clkHz = SOC_rcmGetPeripheralClock(SOC_RcmPeripheralId_MSS_MCANA);
    DebugP_log("MSS_MCANA   = %9d Hz\r\n", clkHz);
    clkHz = SOC_rcmGetPeripheralClock(SOC_RcmPeripheralId_MSS_MCANB);
    DebugP_log("MSS_MCANB   = %9d Hz\r\n", clkHz);
    clkHz = SOC_rcmGetPeripheralClock(SOC_RcmPeripheralId_MSS_QSPI);
    DebugP_log("MSS_QSPI    = %9d Hz\r\n", clkHz);
    clkHz = SOC_rcmGetPeripheralClock(SOC_RcmPeripheralId_MSS_RTIA);
    DebugP_log("MSS_RTIA    = %9d Hz\r\n", clkHz);
    clkHz = SOC_rcmGetPeripheralClock(SOC_RcmPeripheralId_MSS_RTIB);
    DebugP_log("MSS_RTIB    = %9d Hz\r\n", clkHz);
    clkHz = SOC_rcmGetPeripheralClock(SOC_RcmPeripheralId_MSS_RTIC);
    DebugP_log("MSS_RTIC    = %9d Hz\r\n", clkHz);
    clkHz = SOC_rcmGetPeripheralClock(SOC_RcmPeripheralId_MSS_WDT);
    DebugP_log("MSS_WDT     = %9d Hz\r\n", clkHz);
    clkHz = SOC_rcmGetPeripheralClock(SOC_RcmPeripheralId_MSS_SPIA);
    DebugP_log("MSS_SPIA    = %9d Hz\r\n", clkHz);
    clkHz = SOC_rcmGetPeripheralClock(SOC_RcmPeripheralId_MSS_SPIB);
    DebugP_log("MSS_SPIB    = %9d Hz\r\n", clkHz);
    clkHz = SOC_rcmGetPeripheralClock(SOC_RcmPeripheralId_MSS_I2C);
    DebugP_log("MSS_I2C     = %9d Hz\r\n", clkHz);
    clkHz = SOC_rcmGetPeripheralClock(SOC_RcmPeripheralId_MSS_SCIA);
    DebugP_log("MSS_SCIA    = %9d Hz\r\n", clkHz);
    clkHz = SOC_rcmGetPeripheralClock(SOC_RcmPeripheralId_MSS_SCIB);
    DebugP_log("MSS_SCIB    = %9d Hz\r\n", clkHz);
    clkHz = SOC_rcmGetPeripheralClock(SOC_RcmPeripheralId_MSS_CPTS);
    DebugP_log("MSS_CPTS    = %9d Hz\r\n", clkHz);
    clkHz = SOC_rcmGetPeripheralClock(SOC_RcmPeripheralId_MSS_CPSW);
    DebugP_log("MSS_CPSW    = %9d Hz\r\n", clkHz);
    clkHz = SOC_rcmGetPeripheralClock(SOC_RcmPeripheralId_DSS_RTIA);
    DebugP_log("DSS_RTIA    = %9d Hz\r\n", clkHz);
    clkHz = SOC_rcmGetPeripheralClock(SOC_RcmPeripheralId_DSS_RTIB);
    DebugP_log("DSS_RTIB    = %9d Hz\r\n", clkHz);
    clkHz = SOC_rcmGetPeripheralClock(SOC_RcmPeripheralId_DSS_WDT);
    DebugP_log("DSS_WDT     = %9d Hz\r\n", clkHz);
    clkHz = SOC_rcmGetPeripheralClock(SOC_RcmPeripheralId_DSS_SCIA);
    DebugP_log("DSS_SCIA    = %9d Hz\r\n", clkHz);
    clkHz = SOC_rcmGetPeripheralClock(SOC_RcmPeripheralId_RCSS_I2CA);
    DebugP_log("RCSS_I2CA   = %9d Hz\r\n", clkHz);
    clkHz = SOC_rcmGetPeripheralClock(SOC_RcmPeripheralId_RCSS_I2CB);
    DebugP_log("RCSS_I2CB   = %9d Hz\r\n", clkHz);
    clkHz = SOC_rcmGetPeripheralClock(SOC_RcmPeripheralId_RCSS_SCIA);
    DebugP_log("RCSS_SCIA   = %9d Hz\r\n", clkHz);
    clkHz = SOC_rcmGetPeripheralClock(SOC_RcmPeripheralId_RCSS_SPIA);
    DebugP_log("RCSS_SPIA   = %9d Hz\r\n", clkHz);
    clkHz = SOC_rcmGetPeripheralClock(SOC_RcmPeripheralId_RCSS_SPIB);
    DebugP_log("RCSS_SPIB   = %9d Hz\r\n", clkHz);
    clkHz = SOC_rcmGetPeripheralClock(SOC_RcmPeripheralId_RCSS_ATL);
    DebugP_log("RCSS_ATL    = %9d Hz\r\n", clkHz);
    clkHz = SOC_rcmGetPeripheralClock(SOC_RcmPeripheralId_RCSS_MCASPA_AUX);
    DebugP_log("RCSS_MCASPA = %9d Hz\r\n", clkHz);
    clkHz = SOC_rcmGetPeripheralClock(SOC_RcmPeripheralId_RCSS_MCASPB_AUX);
    DebugP_log("RCSS_MCASPB = %9d Hz\r\n", clkHz);
    clkHz = SOC_rcmGetPeripheralClock(SOC_RcmPeripheralId_RCSS_MCASPC_AUX);
    DebugP_log("RCSS_MCASPC = %9d Hz\r\n", clkHz);
}


uint32_t SOC_getFlashDataBaseAddr(void)
{
    return CSL_EXT_FLASH_U_BASE;
}
