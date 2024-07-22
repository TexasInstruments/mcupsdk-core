/*
 *  Copyright (C) 2021-2024 Texas Instruments Incorporated
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

#include <string.h>
#include <drivers/bootloader.h>
#include <drivers/bootloader/bootloader_priv.h>
#include <drivers/bootloader/soc/bootloader_soc.h>
#include <kernel/dpl/CacheP.h>
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/AddrTranslateP.h>

#define BOOTLOADER_M4F_IRAM_BASE (0x00000000)
#define BOOTLOADER_M4F_DRAM_BASE (0x00030000)

#define BOOTLOADER_SYSFW_MAX_SIZE (0x42000U)

#define BOOTLOADER_SYS_STATUS_REG              (0x44234100U)
#define BOOTLOADER_SYS_STATUS_DEV_TYPE_MASK    (0x0000000FU)
#define BOOTLOADER_SYS_STATUS_DEV_SUBTYPE_MASK (0x00000F00U)

#define BOOTLOADER_SYS_STATUS_DEV_TYPE_GP      (0x03U)
#define BOOTLOADER_SYS_STATUS_DEV_TYPE_TEST    (0x05U)
#define BOOTLOADER_SYS_STATUS_DEV_SUBTYPE_FS   (0x00000A00U)

#define BOOTLOADER_DEVICE_FEATURE_REG             (0x43000060)
#define BOOTLOADER_DEVICE_FEATURE_R5F0_DUAL_MASK  (0x00020000)
#define BOOTLOADER_DEVICE_FEATURE_R5F1_DUAL_MASK  (0x00080000)

#define BOOTLOADER_DEVICE_JTAG_USERID_REG            (0x43000018)
#define BOOTLOADER_DEVICE_JTAG_USERID_CORENUM_MASK   (0x000F0000)

#define BOOTLOADER_RST_SRC_COLD_BOOT   (0U)
#define BOOTLOADER_RST_SRC_SW_POR_MCU  (1 << 24)
#define BOOTLOADER_RST_SRC_SW_POR_MAIN (1 << 25)

#define PLLCTRL_PLLCTL_OFFSET       (0x100U)

Bootloader_resMemSections gResMemSection =
{
    .numSections    = 1,
    .memSection[0].memStart   = 0x70000000,
    .memSection[0].memEnd     = 0x70080000,
};

#undef BOOTLOADER_SOC_ATCM_FILL
#undef BOOTLOADER_SOC_BTCM_FILL

/* we will load a dummy while loop here, when there is nothing to load for A53.
 * This is used with SBL NULL, to init A53, so that we can load and run via CCS without
 * GEL files
 */
#define BOOTLOADER_A53_WHILELOOP_LOAD_ADDR      (0x70000040)

/* list the R5F cluster where this bootloader runs, this is fixed to R5FSS0-0, R5FSS0-1 in this SOC */
uint32_t gBootloaderSelfCpuList[] = {
    CSL_CORE_ID_R5FSS0_0,
    CSL_CORE_ID_R5FSS0_1,
    BOOTLOADER_INVALID_ID,
};

const uint32_t gSOC_r5fVectors[18] =
{
    0xE59FF018,
    0xE59FF018,
    0xE59FF018,
    0xE59FF018,
    0xE59FF018,
    0xE59FF018,
    0xE59FF018,
    0xE59FF018,
    0x00000040,
    0x00000040,
    0x00000040,
    0x00000040,
    0x00000040,
    0x00000040,
    0x00000040,
    0x00000040,
    0xE320F003, /* WFI */
    0xEBFFFFFD, /* loop back to WFI */
};

const uint32_t gSOC_a53WhileLoop[2]  =
{
    0xD503207F, /* WFI */
    0x17FFFFFF, /* loop back to WFI */
};

Bootloader_CoreBootInfo gCoreBootInfo[] =
{
    {
        .tisciProcId    = SCICLIENT_PROCID_MCU_M4FSS0_C0,
        .tisciDevId     = TISCI_DEV_MCU_M4FSS0_CORE0,
        .tisciClockId   = TISCI_DEV_MCU_M4FSS0_CORE0_VBUS_CLK,
        .defaultClockHz = (uint32_t)(400*1000000),
        .coreName       = "m4f0-0",
    },

    {
        .tisciProcId    = SCICLIENT_PROCID_R5_CL0_C0,
        .tisciDevId     = TISCI_DEV_R5FSS0_CORE0,
        .tisciClockId   = TISCI_DEV_R5FSS0_CORE0_CPU_CLK,
        .defaultClockHz = (uint32_t)(800*1000000),
        .coreName       = "r5f0-0",
    },

    {
        .tisciProcId    = SCICLIENT_PROCID_R5_CL0_C1,
        .tisciDevId     = TISCI_DEV_R5FSS0_CORE1,
        .tisciClockId   = TISCI_DEV_R5FSS0_CORE1_CPU_CLK,
        .defaultClockHz = (uint32_t)(800*1000000),
        .coreName       = "r5f0-1",
    },

    {
        .tisciProcId    = SCICLIENT_PROCID_R5_CL1_C0,
        .tisciDevId     = TISCI_DEV_R5FSS1_CORE0,
        .tisciClockId   = TISCI_DEV_R5FSS1_CORE0_CPU_CLK,
        .defaultClockHz = (uint32_t)(800*1000000),
        .coreName       = "r5f1-0 ",
    },

    {
        .tisciProcId    = SCICLIENT_PROCID_R5_CL1_C1,
        .tisciDevId     = TISCI_DEV_R5FSS1_CORE1,
        .tisciClockId   = TISCI_DEV_R5FSS1_CORE1_CPU_CLK,
        .defaultClockHz = (uint32_t)(800*1000000),
        .coreName       = "r5f1-1",
    },

    {
        .tisciProcId    = SCICLIENT_PROCID_A53_CL0_C0,
        .tisciDevId     = TISCI_DEV_A53SS0_CORE_0,
        .tisciClockId   = TISCI_DEV_A53SS0_COREPAC_ARM_CLK_CLK,
        .defaultClockHz = (uint32_t)(800*1000000),
        .coreName       = "a530-0",
    },

    {
        .tisciProcId    = SCICLIENT_PROCID_A53_CL0_C1,
        .tisciDevId     = TISCI_DEV_A53SS0_CORE_1,
        .tisciClockId   = TISCI_DEV_A53SS0_COREPAC_ARM_CLK_CLK,
        .defaultClockHz = (uint32_t)(800*1000000),
        .coreName       = "a530-1",
    },
};

Bootloader_CoreAddrTranslateInfo gAddrTranslateInfo[] =
{
    /* CSL_CORE_ID_M4FSS0_0 */
    {
        .numRegions = 2,
        .addrRegionInfo =
        {
            {
                .cpuLocalAddr = BOOTLOADER_M4F_IRAM_BASE,
                .socAddr      = CSL_MCU_M4FSS0_IRAM_BASE,
                .regionSize   = CSL_MCU_M4FSS0_IRAM_SIZE,
            },
            {
                .cpuLocalAddr = BOOTLOADER_M4F_DRAM_BASE,
                .socAddr      = CSL_MCU_M4FSS0_DRAM_BASE,
                .regionSize   = CSL_MCU_M4FSS0_DRAM_SIZE,
            },
        },
    },

    /* CSL_CORE_ID_R5FSS0_0 */
    {
        .numRegions = 2,
        .addrRegionInfo =
        {
            {
                .cpuLocalAddr = CSL_R5FSS0_ATCM_BASE,
                .socAddr      = CSL_R5FSS0_CORE0_ATCM_BASE,
                .regionSize   = CSL_R5FSS0_ATCM_SIZE,
            },
            {
                .cpuLocalAddr = CSL_R5FSS0_BTCM_BASE,
                .socAddr      = CSL_R5FSS0_CORE0_BTCM_BASE,
                .regionSize   = CSL_R5FSS0_BTCM_SIZE,
            },
        },
    },

    /* CSL_CORE_ID_R5FSS0_1 */
    {
        .numRegions = 2,
        .addrRegionInfo =
        {
            {
                .cpuLocalAddr = CSL_R5FSS0_ATCM_BASE,
                .socAddr      = CSL_R5FSS0_CORE1_ATCM_BASE,
                .regionSize   = CSL_R5FSS0_ATCM_SIZE,
            },
            {
                .cpuLocalAddr = CSL_R5FSS0_BTCM_BASE,
                .socAddr      = CSL_R5FSS0_CORE1_BTCM_BASE,
                .regionSize   = CSL_R5FSS0_BTCM_SIZE,
            },
        },
    },

    /* CSL_CORE_ID_R5FSS1_0 */
    {
        .numRegions = 2,
        .addrRegionInfo =
        {
            {
                .cpuLocalAddr = CSL_R5FSS0_ATCM_BASE,
                .socAddr      = CSL_R5FSS1_CORE0_ATCM_BASE,
                .regionSize   = CSL_R5FSS0_ATCM_SIZE,
            },
            {
                .cpuLocalAddr = CSL_R5FSS0_BTCM_BASE,
                .socAddr      = CSL_R5FSS1_CORE0_BTCM_BASE,
                .regionSize   = CSL_R5FSS0_BTCM_SIZE,
            },
        },
    },

    /* CSL_CORE_ID_R5FSS1_1 */
    {
        .numRegions = 2,
        .addrRegionInfo =
        {
            {
                .cpuLocalAddr = CSL_R5FSS0_ATCM_BASE,
                .socAddr      = CSL_R5FSS1_CORE1_ATCM_BASE,
                .regionSize   = CSL_R5FSS0_ATCM_SIZE,
            },
            {
                .cpuLocalAddr = CSL_R5FSS0_BTCM_BASE,
                .socAddr      = CSL_R5FSS1_CORE1_BTCM_BASE,
                .regionSize   = CSL_R5FSS0_BTCM_SIZE,
            },
        },
    },

    /* CSL_CORE_ID_A53SS0_0 */
    {
        .numRegions = 0,
    },
    /* CSL_CORE_ID_A53SS0_1 */
    {
        .numRegions = 0,
    },
};

uint32_t Bootloader_socIsR5FSSDual(uint32_t ssNum)
{
    CSL_ArmR5CPUInfo cpuInfo;

    uint32_t isDual = TRUE;

    if(BOOTLOADER_R5FSS0 == ssNum)
    {
        cpuInfo.grpId = CSL_ARM_R5_CLUSTER_GROUP_ID_0;
        cpuInfo.cpuID = CSL_ARM_R5_CPU_ID_0;
        isDual = SOC_isR5FDualCoreMode(&cpuInfo);
    }
    else if(BOOTLOADER_R5FSS1 == ssNum)
    {
        cpuInfo.grpId = CSL_ARM_R5_CLUSTER_GROUP_ID_1;
        cpuInfo.cpuID = CSL_ARM_R5_CPU_ID_0;
        isDual = SOC_isR5FDualCoreMode(&cpuInfo);
    }
    else
    {
        /* do nothing */
    }

    return isDual;
}

uint32_t Bootloader_socGetCoreVariant(void)
{
    uint32_t coreVariant = BOOTLOADER_DEVICE_VARIANT_QUAD_CORE;

    coreVariant = (CSL_REG32_RD(BOOTLOADER_DEVICE_JTAG_USERID_REG) & BOOTLOADER_DEVICE_JTAG_USERID_CORENUM_MASK);

    if((coreVariant != BOOTLOADER_DEVICE_VARIANT_QUAD_CORE) && \
       (coreVariant != BOOTLOADER_DEVICE_VARIANT_DUAL_CORE) && \
       (coreVariant != BOOTLOADER_DEVICE_VARIANT_SINGLE_CORE))
    {
        coreVariant = BOOTLOADER_INVALID_ID;
    }

    return coreVariant;
}

uint32_t Bootloader_socRprcToCslCoreId(uint32_t rprcCoreId)
{
    uint32_t cslCoreId = CSL_CORE_ID_MAX;
    uint32_t i;

    uint32_t rprcCoreIds[CSL_CORE_ID_MAX] =
    {
        14U, 4U, 5U, 6U, 7U, 0U, 1U
    };

    for(i = 0U; i < CSL_CORE_ID_MAX; i++)
    {
        if(rprcCoreId == rprcCoreIds[i])
        {
            cslCoreId = i;
            break;
        }
    }

    return cslCoreId;
}

uint32_t Bootloader_socGetSciclientCpuProcId(uint32_t cpuId)
{
    uint32_t procId = BOOTLOADER_INVALID_ID;

    if(cpuId < CSL_CORE_ID_MAX)
    {
        procId = gCoreBootInfo[cpuId].tisciProcId;
    }

    return procId;
}

uint32_t Bootloader_socGetSciclientCpuDevId(uint32_t cpuId)
{
    uint32_t devId = BOOTLOADER_INVALID_ID;

    if(cpuId < CSL_CORE_ID_MAX)
    {
        devId = gCoreBootInfo[cpuId].tisciDevId;
    }

    return devId;
}

uint32_t Bootloader_socGetSciclientCpuClkId(uint32_t cpuId)
{
    uint32_t clockId = BOOTLOADER_INVALID_ID;

    if(cpuId < CSL_CORE_ID_MAX)
    {
        clockId = gCoreBootInfo[cpuId].tisciClockId;
    }

    return clockId;
}

uint32_t Bootloader_socCpuGetClkDefault(uint32_t cpuId)
{
    uint32_t defClock = 0U;

    if(cpuId < CSL_CORE_ID_MAX)
    {
        defClock = gCoreBootInfo[cpuId].defaultClockHz;
    }

    return defClock;
}

char* Bootloader_socGetCoreName(uint32_t cpuId)
{
    char *pName = NULL;

    if(cpuId < CSL_CORE_ID_MAX)
    {
        pName = gCoreBootInfo[cpuId].coreName;
    }

    return pName;
}

uint32_t* Bootloader_socGetSelfCpuList(void)
{
	if(Bootloader_socIsR5FSSDual(BOOTLOADER_R5FSS0) == FALSE)
    {
        gBootloaderSelfCpuList[1] = BOOTLOADER_INVALID_ID;
    }
    return &gBootloaderSelfCpuList[0];
}

void Bootloader_socGetR5fAtcmAddrAndSize(uint32_t cpuId, uint32_t *addr, uint32_t *size)
{
    *size = 32*1024;

    switch(cpuId)
    {
        case CSL_CORE_ID_R5FSS0_0:
            *addr = CSL_R5FSS0_CORE0_ATCM_BASE;
            break;
        case CSL_CORE_ID_R5FSS0_1:
            *addr = CSL_R5FSS0_CORE1_ATCM_BASE;
            break;
        case CSL_CORE_ID_R5FSS1_0:
            *addr = CSL_R5FSS1_CORE0_ATCM_BASE;
            break;
        case CSL_CORE_ID_R5FSS1_1:
            *addr = CSL_R5FSS1_CORE1_ATCM_BASE;
            break;
        default:
            *addr = BOOTLOADER_INVALID_ID;
            *size = 0;
            break;
    }
}

void Bootloader_socGetR5fBtcmAddrAndSize(uint32_t cpuId, uint32_t *addr, uint32_t *size)
{
    *size = 32*1024;

    switch(cpuId)
    {
        case CSL_CORE_ID_R5FSS0_0:
            *addr = CSL_R5FSS0_CORE0_BTCM_BASE;
            break;
        case CSL_CORE_ID_R5FSS0_1:
            *addr = CSL_R5FSS0_CORE1_BTCM_BASE;
            break;
        case CSL_CORE_ID_R5FSS1_0:
            *addr = CSL_R5FSS1_CORE0_BTCM_BASE;
            break;
        case CSL_CORE_ID_R5FSS1_1:
            *addr = CSL_R5FSS1_CORE1_BTCM_BASE;
            break;
        default:
            *addr = BOOTLOADER_INVALID_ID;
            *size = 0;
            break;
    }
}

/* init ATCM and BTCM and load valid reset vectors and wait instruction */
void Bootloader_socInitR5FAtcmBtcm(uint32_t cpuId)
{
    uint32_t addr, size, i;
    volatile uint32_t *pAddr;

    Bootloader_socGetR5fAtcmAddrAndSize(cpuId, &addr, &size);
    if(addr != BOOTLOADER_INVALID_ID && size > 0)
    {
        #ifdef BOOTLOADER_SOC_ATCM_FILL
        pAddr = (volatile uint32_t *)addr;
        for(i=0; i< size/sizeof(uint32_t); i++)
        {
            pAddr[i] = 0xFFFFFFFF;
        }
        #endif
        pAddr = (volatile uint32_t *)addr;
        for(i=0; i< sizeof(gSOC_r5fVectors)/sizeof(uint32_t); i++)
        {
            pAddr[i] = gSOC_r5fVectors[i];
        }
    }
    Bootloader_socGetR5fBtcmAddrAndSize(cpuId, &addr, &size);
    #ifdef BOOTLOADER_SOC_BTCM_FILL
    if(addr != BOOTLOADER_INVALID_ID && size > 0)
    {
        pAddr = (volatile uint32_t *)addr;
        for(i=0; i< size/sizeof(uint32_t); i++)
        {
            pAddr[i] = 0xFFFFFFFF;
        }
    }
    #endif
}

/* init M4 IRAM with valid reset vector and valid wait instruction */
void Bootloader_socInitM4fIram(void)
{
    uint32_t m4f_iram_base_addr = 0x05000000;

    *(volatile uint32_t *)(m4f_iram_base_addr + 0) = 0x1000; /* stack size */
    *(volatile uint32_t *)(m4f_iram_base_addr + 4) = 0x400 + 1; /* reset vector */
    *(volatile uint32_t *)(m4f_iram_base_addr + 0x400) = 0xBF30BF30; /* WFI instruction */
}

int32_t Bootloader_socCpuRequest(uint32_t cpuId)
{
    int32_t status = SystemP_FAILURE;
    uint32_t sciclientCpuProcId;

    sciclientCpuProcId = Bootloader_socGetSciclientCpuProcId(cpuId);
    if(sciclientCpuProcId != BOOTLOADER_INVALID_ID)
    {
        status = Sciclient_procBootRequestProcessor(sciclientCpuProcId, SystemP_WAIT_FOREVER);
        if(status != SystemP_SUCCESS)
        {
            DebugP_logError("CPU request failed for %s\r\n", Bootloader_socGetCoreName(cpuId));
        }
    }
    return status;
}

int32_t Bootloader_socCpuRelease(uint32_t cpuId)
{
    int32_t status = SystemP_FAILURE;
    uint32_t sciclientCpuProcId;

    sciclientCpuProcId = Bootloader_socGetSciclientCpuProcId(cpuId);
    if(sciclientCpuProcId != BOOTLOADER_INVALID_ID)
    {
        status = Sciclient_procBootReleaseProcessor(sciclientCpuProcId, TISCI_MSG_FLAG_AOP, SystemP_WAIT_FOREVER);
        if(status != SystemP_SUCCESS)
        {
            DebugP_logError("CPU release failed for %s\r\n", Bootloader_socGetCoreName(cpuId));
        }
    }

    return status;
}

int32_t Bootloader_socCpuSetClock(uint32_t cpuId, uint32_t cpuHz)
{
    int32_t status = SystemP_FAILURE;
    uint32_t sciclientCpuDevId;
    uint32_t sciclientCpuClkId;

    sciclientCpuDevId = Bootloader_socGetSciclientCpuDevId(cpuId);
    sciclientCpuClkId = Bootloader_socGetSciclientCpuClkId(cpuId);
    if(sciclientCpuDevId != BOOTLOADER_INVALID_ID && sciclientCpuClkId != BOOTLOADER_INVALID_ID)
    {
        status = Sciclient_pmSetModuleClkFreq(sciclientCpuDevId,
                                                sciclientCpuClkId,
                                                cpuHz,
                                                TISCI_MSG_FLAG_AOP,
                                                SystemP_WAIT_FOREVER);
        if(status != SystemP_SUCCESS)
        {
            DebugP_logError("CPU clock set for %d Hz failed for %s\r\n", cpuHz, Bootloader_socGetCoreName(cpuId));
        }
    }
    return status;
}

uint64_t Bootloader_socCpuGetClock(uint32_t cpuId)
{
    int32_t status = SystemP_FAILURE;
    uint64_t clkRate = 0;
    uint32_t sciclientCpuDevId;
    uint32_t sciclientCpuClkId;

    sciclientCpuDevId = Bootloader_socGetSciclientCpuDevId(cpuId);
    sciclientCpuClkId = Bootloader_socGetSciclientCpuClkId(cpuId);
    if(sciclientCpuDevId != BOOTLOADER_INVALID_ID && sciclientCpuClkId != BOOTLOADER_INVALID_ID)
    {
        status = Sciclient_pmGetModuleClkFreq(
                    sciclientCpuDevId,
                    sciclientCpuClkId,
                    &clkRate,
                    SystemP_WAIT_FOREVER);

        if(status != SystemP_SUCCESS)
        {
            DebugP_logError("CPU clock get failed for %s\r\n", Bootloader_socGetCoreName(cpuId));
        }
    }
    return clkRate;
}

int32_t Bootloader_socCpuPowerOnResetM4f(uint32_t cpuId, uint32_t initRam)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t sciclientCpuDevId;

    sciclientCpuDevId = Bootloader_socGetSciclientCpuDevId(cpuId);

    status = Sciclient_pmSetModuleState(sciclientCpuDevId,
                    TISCI_MSG_VALUE_DEVICE_SW_STATE_AUTO_OFF,
                    TISCI_MSG_FLAG_AOP,
                    SystemP_WAIT_FOREVER);
    if(status != SystemP_SUCCESS)
    {
        DebugP_logError("CPU power off failed for %s\r\n", Bootloader_socGetCoreName(cpuId));
    }

    if(status == SystemP_SUCCESS)
    {
        status = Sciclient_pmSetModuleRst(sciclientCpuDevId, 1, SystemP_WAIT_FOREVER);
        if(status != SystemP_SUCCESS)
        {
            DebugP_logError("CPU reset assert failed for %s\r\n", Bootloader_socGetCoreName(cpuId));
        }
    }
    if(status == SystemP_SUCCESS)
    {
        status = Sciclient_pmSetModuleState(sciclientCpuDevId,
                            TISCI_MSG_VALUE_DEVICE_SW_STATE_ON,
                            TISCI_MSG_FLAG_AOP,
                            SystemP_WAIT_FOREVER);
        if(status != SystemP_SUCCESS)
        {
            DebugP_logError("CPU power ON failed for %s\r\n", Bootloader_socGetCoreName(cpuId));
        }
    }
    if(status == SystemP_SUCCESS)
    {
        if(initRam)
        {
            /* initialize the RAMs only if requested
             */
            Bootloader_socInitM4fIram();
        }
    }
    return status;
}

/* applicable for all R5F's except the boot R5F cluster */
int32_t Bootloader_socCpuPowerOnResetR5f(uint32_t cpuId, uintptr_t entry_point, uint32_t initRam)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t sciclientCpuProcId, sciclientCpuDevId;
    struct tisci_msg_proc_get_status_resp proc_get_status;
    struct tisci_msg_proc_set_config_req  proc_set_config;

    proc_get_status.processor_id = 0;

    sciclientCpuProcId = Bootloader_socGetSciclientCpuProcId(cpuId);
    sciclientCpuDevId = Bootloader_socGetSciclientCpuDevId(cpuId);

    status = Sciclient_pmSetModuleState(sciclientCpuDevId,
        TISCI_MSG_VALUE_DEVICE_SW_STATE_AUTO_OFF,
        TISCI_MSG_FLAG_AOP,
        SystemP_WAIT_FOREVER);
    if(status != SystemP_SUCCESS)
    {
        DebugP_logError("CPU power off failed for %s\r\n", Bootloader_socGetCoreName(cpuId));
    }

    if(status == SystemP_SUCCESS)
    {
        status = Sciclient_procBootGetProcessorState(sciclientCpuProcId,
                    &proc_get_status,
                    SystemP_WAIT_FOREVER);
        if(status != SystemP_SUCCESS)
        {
            DebugP_logError("CPU get state failed for %s\r\n", Bootloader_socGetCoreName(cpuId));
        }
    }
    if(status == SystemP_SUCCESS)
    {
        proc_set_config.processor_id = proc_get_status.processor_id;
        proc_set_config.bootvector_lo = entry_point;
        proc_set_config.bootvector_hi = 0;
        proc_set_config.config_flags_1_set = 0;
        proc_set_config.config_flags_1_clear = 0;
        proc_set_config.config_flags_1_set |= TISCI_MSG_VAL_PROC_BOOT_CFG_FLAG_R5_ATCM_EN;
        proc_set_config.config_flags_1_set |= (TISCI_MSG_VAL_PROC_BOOT_CFG_FLAG_R5_BTCM_EN |
                                                TISCI_MSG_VAL_PROC_BOOT_CFG_FLAG_R5_TCM_RSTBASE);

        if(Bootloader_socIsR5FSSDual(BOOTLOADER_R5FSS1) == FALSE)
        {
            proc_set_config.config_flags_1_set |= TISCI_MSG_VAL_PROC_BOOT_CFG_FLAG_R5_SINGLE_CORE;
        }

        status = Sciclient_procBootSetProcessorCfg(&proc_set_config, SystemP_WAIT_FOREVER);
        if(status != SystemP_SUCCESS)
        {
            DebugP_logError("CPU set config failed for %s\r\n", Bootloader_socGetCoreName(cpuId));
        }
    }
    if(status == SystemP_SUCCESS)
    {
        status =  Sciclient_procBootSetSequenceCtrl(sciclientCpuProcId,
                            TISCI_MSG_VAL_PROC_BOOT_CTRL_FLAG_R5_CORE_HALT,
                            0,
                            TISCI_MSG_FLAG_AOP,
                            SystemP_WAIT_FOREVER);
        if(status != SystemP_SUCCESS)
        {
            DebugP_logError("CPU halt set failed for %s\r\n", Bootloader_socGetCoreName(cpuId));
        }
    }
    if(status == SystemP_SUCCESS)
    {
        status = Sciclient_pmSetModuleState(sciclientCpuDevId,
                        TISCI_MSG_VALUE_DEVICE_SW_STATE_ON,
                        TISCI_MSG_FLAG_AOP,
                        SystemP_WAIT_FOREVER);
        /* In single core mode, LPSCs of both cores need to be enabled for core 0 to work  */
        if((CSL_CORE_ID_R5FSS1_0 == cpuId) && (Bootloader_socIsR5FSSDual(BOOTLOADER_R5FSS1) == FALSE))
        {
            status = Sciclient_pmSetModuleState(Bootloader_socGetSciclientCpuDevId(CSL_CORE_ID_R5FSS1_1),
                        TISCI_MSG_VALUE_DEVICE_SW_STATE_ON,
                        TISCI_MSG_FLAG_AOP,
                        SystemP_WAIT_FOREVER);
        }
        if(status != SystemP_SUCCESS)
        {
            DebugP_logError("CPU power on failed for %s\r\n", Bootloader_socGetCoreName(cpuId));
        }
    }
    if(status == SystemP_SUCCESS)
    {
        if(initRam)
        {
            /* initialize the RAMs only if requested
             */
            Bootloader_socInitR5FAtcmBtcm(cpuId);
        }
    }
    return status;
}

int32_t Bootloader_socCpuPowerOnResetA53(uint32_t cpuId)
{
    int32_t status = SystemP_SUCCESS;

    /* Power on A53, this can be skipped if we only want to load code for A53 */
    status = Sciclient_pmSetModuleState(TISCI_DEV_A53SS0, TISCI_MSG_VALUE_DEVICE_SW_STATE_ON, 0, SystemP_WAIT_FOREVER);
    if(status != SystemP_SUCCESS)
    {
        DebugP_logError("CPU cluster power on failed for %s\r\n", Bootloader_socGetCoreName(cpuId));
    }

    /* copy while(1) loop to load addr and flush it to memory */
    memcpy( (void*)BOOTLOADER_A53_WHILELOOP_LOAD_ADDR, gSOC_a53WhileLoop, sizeof(gSOC_a53WhileLoop));
    CacheP_wbInv((void*)BOOTLOADER_A53_WHILELOOP_LOAD_ADDR, sizeof(gSOC_a53WhileLoop), CacheP_TYPE_ALL);

    return status;
}

/* Power ON, init the RAMs, load a dummy while loop and hold CPU in reset, do not release the reset */
int32_t Bootloader_socCpuPowerOnReset(uint32_t cpuId, void *socCoreOpMode)
{
    int32_t status = SystemP_FAILURE;

    switch(cpuId)
    {
        case CSL_CORE_ID_R5FSS0_0:
        case CSL_CORE_ID_R5FSS0_1:
            /* nothing to do CPU's are already powred on, we dont want to hold
               them in reset since bootloader is running here
             */
            break;

        case CSL_CORE_ID_R5FSS1_0:
        case CSL_CORE_ID_R5FSS1_1:
            status = Bootloader_socCpuPowerOnResetR5f(cpuId, 0, 1);
            break;
        case CSL_CORE_ID_M4FSS0_0:
            status = Bootloader_socCpuPowerOnResetM4f(cpuId, 1);
            break;
        case CSL_CORE_ID_A53SS0_0:
        case CSL_CORE_ID_A53SS0_1:
            status = Bootloader_socCpuPowerOnResetA53(cpuId);
            break;
    }
    return status;
}

int32_t Bootloader_socCpuResetRelease(uint32_t cpuId, uintptr_t entryPoint)
{
    int32_t status = SystemP_FAILURE;
    uint32_t sciclientCpuProcId, sciclientCpuDevId;

    struct tisci_msg_proc_set_config_req  proc_set_config;

    sciclientCpuProcId = Bootloader_socGetSciclientCpuProcId(cpuId);
    sciclientCpuDevId = Bootloader_socGetSciclientCpuDevId(cpuId);

    switch(cpuId)
    {
        case CSL_CORE_ID_R5FSS0_0:
        case CSL_CORE_ID_R5FSS0_1:
            /* use SOC_cpuResetReleaseSelf instead */
            break;
        case CSL_CORE_ID_R5FSS1_0:
        case CSL_CORE_ID_R5FSS1_1:
            /* set boot address */
            {
                proc_set_config.processor_id = sciclientCpuProcId;
                proc_set_config.bootvector_lo = entryPoint;
                proc_set_config.bootvector_hi = 0;
                proc_set_config.config_flags_1_set = 0;
                proc_set_config.config_flags_1_clear = 0;

                status = Sciclient_procBootSetProcessorCfg(&proc_set_config, SystemP_WAIT_FOREVER);
                if(status != SystemP_SUCCESS)
                {
                    DebugP_logError("CPU set boot address failed for %s\r\n", Bootloader_socGetCoreName(cpuId));
                }
            }
            if(status == SystemP_SUCCESS)
            {
                status =  Sciclient_procBootSetSequenceCtrl(sciclientCpuProcId,
                                    0,
                                    TISCI_MSG_VAL_PROC_BOOT_CTRL_FLAG_R5_CORE_HALT,
                                    TISCI_MSG_FLAG_AOP,
                                    SystemP_WAIT_FOREVER);
                if(status != SystemP_SUCCESS)
                {
                    DebugP_logError("CPU halt clear failed for %s\r\n", Bootloader_socGetCoreName(cpuId));
                }
            }
            break;
        case CSL_CORE_ID_M4FSS0_0:
            /* entry point is not used, M4F always boots from 0x0 */
            {
                /* reset release */
                status = Sciclient_pmSetModuleRst(sciclientCpuDevId, 0, SystemP_WAIT_FOREVER);
                if(status != SystemP_SUCCESS)
                {
                    DebugP_logError("CPU reset release failed for %s\r\n", Bootloader_socGetCoreName(cpuId));
                }
            }
            break;
        case CSL_CORE_ID_A53SS0_0:
        case CSL_CORE_ID_A53SS0_1:
            /* set boot address */
            {
                if(entryPoint==0)
                {
                    /* start A53 pointing to previously loaded while(1) loop */
                    entryPoint = (uintptr_t)BOOTLOADER_A53_WHILELOOP_LOAD_ADDR;
                }
                proc_set_config.processor_id = sciclientCpuProcId;
                proc_set_config.bootvector_lo = entryPoint;
                proc_set_config.bootvector_hi = 0;
                proc_set_config.config_flags_1_set = 0;
                proc_set_config.config_flags_1_clear = 0;

                status = Sciclient_procBootSetProcessorCfg(&proc_set_config, SystemP_WAIT_FOREVER);
                if(status != SystemP_SUCCESS)
                {
                    DebugP_logError("CPU set boot address failed for %s\r\n", Bootloader_socGetCoreName(cpuId));
                }

                status = Sciclient_pmSetModuleState(sciclientCpuDevId,
                        TISCI_MSG_VALUE_DEVICE_SW_STATE_AUTO_OFF,
                        TISCI_MSG_FLAG_AOP,
                        SystemP_WAIT_FOREVER);
                if(status != SystemP_SUCCESS)
                {
                    DebugP_logError("CPU power off failed for %s\r\n", Bootloader_socGetCoreName(cpuId));
                }

                status = Sciclient_pmSetModuleState(sciclientCpuDevId,
                        TISCI_MSG_VALUE_DEVICE_SW_STATE_ON,
                        TISCI_MSG_FLAG_AOP,
                        SystemP_WAIT_FOREVER);
                if(status != SystemP_SUCCESS)
                {
                    DebugP_logError("CPU power on failed for %s\r\n", Bootloader_socGetCoreName(cpuId));
                }
            }

            break;
    }
    return status;
}

int32_t Bootloader_socCpuSetEntryPoint(uint32_t cpuId, uintptr_t entryPoint)
{
    int32_t status = SystemP_SUCCESS;

    uint32_t sciclientCpuProcId = Bootloader_socGetSciclientCpuProcId(cpuId);

    struct tisci_msg_proc_set_config_req  proc_set_config;

    /* set entry point for core0 */
    proc_set_config.processor_id = sciclientCpuProcId;
    proc_set_config.bootvector_lo = entryPoint;
    proc_set_config.bootvector_hi = 0;
    proc_set_config.config_flags_1_set = 0;
    proc_set_config.config_flags_1_clear = 0;
    proc_set_config.config_flags_1_set |= TISCI_MSG_VAL_PROC_BOOT_CFG_FLAG_R5_ATCM_EN;
    proc_set_config.config_flags_1_set |= (TISCI_MSG_VAL_PROC_BOOT_CFG_FLAG_R5_BTCM_EN |
                                                TISCI_MSG_VAL_PROC_BOOT_CFG_FLAG_R5_TCM_RSTBASE);

    if(Bootloader_socIsR5FSSDual(BOOTLOADER_R5FSS0) == FALSE)
    {
        proc_set_config.config_flags_1_set |= TISCI_MSG_VAL_PROC_BOOT_CFG_FLAG_R5_SINGLE_CORE;
    }

    status = Sciclient_procBootSetProcessorCfg(&proc_set_config, SystemP_WAIT_FOREVER);
    if(status != SystemP_SUCCESS)
    {
        DebugP_logError("CPU set config failed for %s\r\n", Bootloader_socGetCoreName(cpuId));
    }

    return status;
}

int32_t Bootloader_socCpuResetReleaseSelf(void)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t sciclientCpuProcIdCore0, sciclientCpuDevIdCore0;
    uint32_t sciclientCpuProcIdCore1 = BOOTLOADER_INVALID_ID, sciclientCpuDevIdCore1 = BOOTLOADER_INVALID_ID;
    uint32_t bDualSelfR5F = FALSE;

    sciclientCpuProcIdCore0 = Bootloader_socGetSciclientCpuProcId(CSL_CORE_ID_R5FSS0_0);
    sciclientCpuDevIdCore0 = Bootloader_socGetSciclientCpuDevId(CSL_CORE_ID_R5FSS0_0);


    if(Bootloader_socIsR5FSSDual(BOOTLOADER_R5FSS0) == TRUE)
    {
        bDualSelfR5F = TRUE;
    }


    if(bDualSelfR5F == TRUE)
    {
        sciclientCpuProcIdCore1 = Bootloader_socGetSciclientCpuProcId(CSL_CORE_ID_R5FSS0_1);
        sciclientCpuDevIdCore1 = Bootloader_socGetSciclientCpuDevId(CSL_CORE_ID_R5FSS0_1);
    }

    /*
     *   DMSC will block until a WFI is issued, thus allowing the following commands
     *   to be queued so this cluster may be reset by DMSC (queue length is defined in
     *   "source/drivers/sciclient/include/tisci/{soc}/tisci_sec_proxy.h". If these commands
     *   were to be issued and executed prior to WFI, the cluster would enter reset and
     *   bootloader would not be able to tell DMSC to take itself out of reset.
     */
    status = Sciclient_procBootWaitProcessorState(sciclientCpuProcIdCore0,
                    1, 1, 0, 3, 0, 0, 0, SystemP_WAIT_FOREVER);
    if(status != SystemP_SUCCESS)
    {
        DebugP_logError("CPU boot wait command failed for %s\r\n", Bootloader_socGetCoreName(CSL_CORE_ID_R5FSS0_0));
    }

    if(status==SystemP_SUCCESS)
    {
        /* hold core0 and core 1 in reset */
        if(bDualSelfR5F == TRUE)
        {
            status = Sciclient_pmSetModuleRst_flags(sciclientCpuDevIdCore1, 1, 0, SystemP_WAIT_FOREVER);
        }

        /* after this point you cannot single step in CCS */
        if(status==SystemP_SUCCESS)
        {
            status = Sciclient_pmSetModuleRst_flags(sciclientCpuDevIdCore0, 1, 0, SystemP_WAIT_FOREVER);
        }
        /*
         * Un-halt Core1 (Core0 is not halted)
         */
        if(bDualSelfR5F == TRUE)
        {
            if(status==SystemP_SUCCESS)
            {
                status = Sciclient_procBootSetSequenceCtrl(sciclientCpuProcIdCore1,
                                0,
                                TISCI_MSG_VAL_PROC_BOOT_CTRL_FLAG_R5_CORE_HALT,
                                0,
                                SystemP_WAIT_FOREVER);
            }
        }
        /* release the CPUs */
        if(status==SystemP_SUCCESS)
        {
            status = Sciclient_procBootReleaseProcessor(sciclientCpuProcIdCore0, 0, SystemP_WAIT_FOREVER);
        }
        if(bDualSelfR5F == TRUE)
        {
            if(status==SystemP_SUCCESS)
            {
                status = Sciclient_procBootReleaseProcessor(sciclientCpuProcIdCore1, 0, SystemP_WAIT_FOREVER);
            }
        }
        /* release the reset for the CPUs */
        if(status==SystemP_SUCCESS)
        {
            status = Sciclient_pmSetModuleRst_flags(sciclientCpuDevIdCore0, 0, 0, SystemP_WAIT_FOREVER);
        }
        if(bDualSelfR5F == TRUE)
        {
            if(status==SystemP_SUCCESS)
            {
                status = Sciclient_pmSetModuleRst_flags(sciclientCpuDevIdCore1, 0, 0, SystemP_WAIT_FOREVER);
            }
        }
        if(status==SystemP_SUCCESS)
        {
            status = Bootloader_socSecHandover();
        }
        if(status==SystemP_SUCCESS)
        {
            /* disable interrupts if enabled */
            HwiP_disable();

            /* flush all caches */
            CacheP_wbInvAll(CacheP_TYPE_ALL);

            /* execute wfi, now SYSFW will execute the above commands and reset core0 and core 1 */
            __asm__ __volatile__ ("wfi" "\n\t": : : "memory");
        }
        if(status != SystemP_SUCCESS)
        {
            DebugP_logError("CPU reset sequence failed for %s\r\n", Bootloader_socGetCoreName(CSL_CORE_ID_R5FSS0_0));
        }
    }
    return status;
}

uint32_t Bootloader_socTranslateSectionAddr(uint32_t cslCoreId, uint32_t addr)
{
    uint32_t outputAddr = addr;

    Bootloader_CoreAddrTranslateInfo *addrTranslateInfo = &gAddrTranslateInfo[cslCoreId];

    uint32_t i;

    for(i=0; i<addrTranslateInfo->numRegions; i++)
    {
        uint32_t cpuLocalAddr = addrTranslateInfo->addrRegionInfo[i].cpuLocalAddr;
        uint32_t socAddr      = addrTranslateInfo->addrRegionInfo[i].socAddr;
        uint32_t regionSize   = addrTranslateInfo->addrRegionInfo[i].regionSize;

        if((addr >= cpuLocalAddr) && (addr <  cpuLocalAddr + regionSize))
        {
            uint32_t offset = addr - cpuLocalAddr;
            outputAddr = socAddr + offset;
            break;
        }
    }

    return outputAddr;
}

int32_t Bootloader_socMemInitCpu(uint32_t cpuId)
{
    int32_t status = SystemP_SUCCESS;

    switch(cpuId) {
        case CSL_CORE_ID_R5FSS0_0:
        case CSL_CORE_ID_R5FSS0_1:
        case CSL_CORE_ID_R5FSS1_0:
        case CSL_CORE_ID_R5FSS1_1:
            Bootloader_socInitR5FAtcmBtcm(cpuId);
            break;

        case CSL_CORE_ID_M4FSS0_0:
            Bootloader_socInitM4fIram();

        default:
            break;
    }

    return status;
}

int32_t Bootloader_socSecHandover(void)
{
    int32_t status = SystemP_SUCCESS;

    status = Sciclient_triggerSecHandover();

    return status;
}

uint32_t Bootloader_socIsAuthRequired(void)
{
    uint32_t isAuthRequired = TRUE;

    uint32_t devType    = CSL_REG32_RD(BOOTLOADER_SYS_STATUS_REG) & BOOTLOADER_SYS_STATUS_DEV_TYPE_MASK;

    if((devType == BOOTLOADER_SYS_STATUS_DEV_TYPE_GP)   ||
       (devType == BOOTLOADER_SYS_STATUS_DEV_TYPE_TEST))
    {
        isAuthRequired = FALSE;
    }
    else
    {
        isAuthRequired = TRUE;
    }

    return isAuthRequired;
}

int32_t Bootloader_socWaitForFWBoot(void)
{
    return Sciclient_waitForBootNotification();
}

static int32_t Bootloader_socOpenFirewallRegion(uint16_t fwl, uint16_t region, uint32_t control, uint32_t startAddr, uint32_t endAddr)
{
    int32_t status = SystemP_FAILURE;

    /* Change ownership of firewall to R5F0-0 (Host ID = TISCI_HOST_ID_MAIN_0_R5_0 because SBL would be secure host) */
    const struct tisci_msg_fwl_change_owner_info_req fwl_owner_req =
    {
        .fwl_id = fwl,
        .region = region,
        .owner_index = TISCI_HOST_ID_MAIN_0_R5_0,
    };

    struct tisci_msg_fwl_change_owner_info_resp fwl_owner_resp = { {0} };
    status = Sciclient_firewallChangeOwnerInfo(&fwl_owner_req, &fwl_owner_resp, SystemP_TIMEOUT);

    if(SystemP_SUCCESS == status)
    {
        /* Unlock MSRAM firewalls */
        const struct tisci_msg_fwl_set_firewall_region_req fwl_set_req =
        {
            .fwl_id = fwl,
            .region = region,
            .n_permission_regs = 3,
            /*
             * The firewall control register layout is
             *  ---------------------------------------------------------------------------
             * |  31:10   |      9     |     8      |     7:5    |      4      |   3:0     |
             *  ---------------------------------------------------------------------------
             * | Reserved | Cache Mode | Background |  Reserved  | Lock Config |  Enable   |
             *  ---------------------------------------------------------------------------
             *
             * Enable = 0xA implies firewall is enabled. Any other value means not enabled
             *
             */
            .control = control,
            /*
             * The firewall permission register layout is
             *  ---------------------------------------------------------------------------
             * |  31:24   |    23:16   |  15:12     |   11:8     |   7:4      |   3:0      |
             *  ---------------------------------------------------------------------------
             * | Reserved |   Priv ID  | NSUSR-DCRW | NSPRI-DCRW | SUSER-DCRW | SPRIV-DCRW |
             *  ---------------------------------------------------------------------------
             *
             * PRIV_ID = 0xC3 implies all.
             * In each of the 4 nibbles from 15:0 the 4 bits means Debug, Cache, Read, Write Access for
             * Non-secure user, Non-secure Priv, Secure user, Secure Priv respectively. To enable all access
             * bits for all users, we set each of these nibbles to 0b1111 = 0xF. So 15:0 becomes 0xFFFF
             *
             */
            .permissions[0] = 0xC3FFFF,
            .permissions[1] = 0xC3FFFF,
            .permissions[2] = 0xC3FFFF,
            .start_address  = startAddr,
            .end_address    = endAddr,
        };
        struct tisci_msg_fwl_set_firewall_region_resp fwl_set_resp = { 0 };

        status = Sciclient_firewallSetRegion(&fwl_set_req, &fwl_set_resp, SystemP_TIMEOUT);
    }

    return status;
}

int32_t Bootloader_socOpenFirewalls(void)
{
    int32_t status = SystemP_FAILURE;
    uint32_t i;

    /* There are 8 firewalls, 1 per MSRAM memory bank. We need to open all these banks.

        FWL 14 : 0x70000000 -> 0x7003FFFF BANK0
        FWL 15 : 0x70040000 -> 0x7007FFFF BANK1
        FWL 16 : 0x70080000 -> 0x7007FFFF BANK2
        FWL 19 : 0x700C0000 -> 0x700FFFFF BANK3
        FWL 18 : 0x70100000 -> 0x7013FFFF BANK4
        FWL 17 : 0x70140000 -> 0x7017FFFF BANK5
        FWL 23 : 0x70180000 -> 0x701BFFFF BANK6
        FWL 24 : 0x701C0000 -> 0x701FFFFF BANK7

        FWL 24 doesn't need any configuration, already done for applicable region by SYSFW
        FWL 23 - Disable region 1, re-configure region 0
    */
    uint8_t fwlIds[] = { 14, 15, 16, 19, 18, 17 };
    uint32_t startAddr = CSL_MSRAM_256K0_RAM_BASE;

    uint32_t fwlControl = 0x30A;
    /* Nibbles from left to right, 3 implies cached, background region, 0 implies config is unlocked, and A implies enable firewall  */

    for(i = 0U; i < 6; i++)
    {
        status = Bootloader_socOpenFirewallRegion(fwlIds[i], 0, fwlControl, startAddr, (startAddr + CSL_MSRAM_256K0_RAM_SIZE - 1));

        if(SystemP_SUCCESS == status)
        {
            /* Update start address */
            startAddr += CSL_MSRAM_256K0_RAM_SIZE;
        }
        else
        {
            break;
        }
    }

    /* FWL 23 : Disable region 1, and reprogram region 0 */
    if(SystemP_SUCCESS == status)
    {
        fwlControl = 0x300;
        /* Here we are disabling the firewall, so the addresses don't matter */
        status = Bootloader_socOpenFirewallRegion(23, 1, fwlControl, 0, 0);
    }
    else
    {
        status = SystemP_FAILURE;
    }
    if(SystemP_SUCCESS == status)
    {
        fwlControl = 0x30A;
        /* Open firewalls for MSRAM BANK6 */
        status = Bootloader_socOpenFirewallRegion(23, 0, fwlControl, startAddr, (startAddr + CSL_MSRAM_256K0_RAM_SIZE - 1));
    }
    else
    {
        status = SystemP_FAILURE;
    }
    /* FWL 24 is already configured, first 128 KB is accessible to all. Last 128 KB will be used by SYSFW */

    return status;
}

int32_t Bootloader_socAuthImage(uint32_t certLoadAddr)
{
    int32_t status = SystemP_FAILURE;

    struct tisci_msg_proc_auth_boot_req authReq;

    /* Request TIFS (SYSFW) to authenticate (and decrypt if mentioned in the x509 cert) the image */
    authReq.certificate_address_hi = 0U;
    authReq.certificate_address_lo = certLoadAddr;

    status = Sciclient_procBootAuthAndStart(&authReq, SystemP_WAIT_FOREVER);

    return status;
}

void Bootloader_socResetWorkaround(void)
{
    uint32_t resetSource = SOC_getWarmResetCauseMcuDomain();

    if((resetSource == BOOTLOADER_RST_SRC_COLD_BOOT) ||
       (resetSource & (BOOTLOADER_RST_SRC_SW_POR_MCU | BOOTLOADER_RST_SRC_SW_POR_MAIN)))
    {
        /* Clear reset source */
        SOC_clearResetCauseMainMcuDomain(resetSource);

        /* Do warm reset */
        SOC_generateSwWarmResetMcuDomain();
    }
}

Bootloader_resMemSections* Bootloader_socGetSBLMem(void)
{
    return &gResMemSection;
}

void Bootloader_enableMCUPLL(void)
{
    uint32_t baseAddr = CSL_MCU_PLLCTRL0_BASE;
    uint32_t value;

    baseAddr = (uint32_t)AddrTranslateP_getLocalAddr(baseAddr);
    value = CSL_REG32_RD(baseAddr + PLLCTRL_PLLCTL_OFFSET);

    /* Enable PLL */
    /*
    * The PLL control register layout is
    *  ---------------------------------------------------------------------------
    * |  31:10   |      9      |    8    |   7     |   6      |    5     |
    *  ---------------------------------------------------------------------------
    * | Reserved |   EXCLKSRC  | CLKMODE | PLLSELB | Reserved | PLLENSRC |
    *  ---------------------------------------------------------------------------
    *
    *  ---------------------------------------------------------------------------
    * |     4     |      3       |      2      |      1      |     0     |
    * |  PLLDIS   |    PLLRST    |   Reserved  |   PLLPWRDN  |   PLLEN   |
    *
    */
    value = (value & ~(1<<5)) | 0x1u;

    CSL_REG32_WR(baseAddr + PLLCTRL_PLLCTL_OFFSET, value);

}

uint32_t Bootloader_socIsMCUResetIsoEnabled(void)
{
    uint32_t status = 0;

    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, 6);

    /* If MAGIC WORD is non zero reset isolation is enabled */
    if (CSL_REG32_RD(AddrTranslateP_getLocalAddr(CSL_CTRL_MMR0_CFG0_BASE + \
                                CSL_MAIN_CTRL_MMR_CFG0_RST_MAGIC_WORD)))
    {
        status = 1;
    }

    SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MAIN, 6);

    return status;
}

int32_t Bootloader_socEnableICSSCores(uint32_t clkFreq)
{
    int32_t status = SystemP_SUCCESS;

    /* Enable the ICCSG0 core */
    status = SOC_moduleClockEnable(TISCI_DEV_PRU_ICSSG0, 1);

    if(SystemP_SUCCESS == status)
    {
        status = SOC_moduleSetClockFrequency(
                TISCI_DEV_PRU_ICSSG0,
                TISCI_DEV_PRU_ICSSG0_CORE_CLK,
                clkFreq
                );
    }

    /* Enable the ICCSG1 core */
    if(SystemP_SUCCESS == status)
    {
        status = SOC_moduleClockEnable(TISCI_DEV_PRU_ICSSG1, 1);
    }

    if(SystemP_SUCCESS == status)
    {
        status = SOC_moduleSetClockFrequency(
                TISCI_DEV_PRU_ICSSG1,
                TISCI_DEV_PRU_ICSSG1_CORE_CLK,
                clkFreq
                );
    }

    return status;
}

void Bootloader_socNotifyFirewallOpen(void)
{
    /* PSRAM is used to signal firewall open from SBL */
    uint32_t *psramPtr = (uint32_t *)AddrTranslateP_getLocalAddr(CSL_PSRAMECC0_RAM_BASE);

    CSL_REG32_WR(psramPtr, SOC_FWL_OPEN_MAGIC_NUM);
}

void Bootloader_socGetBootSeqOid(uint8_t* boot_seq_oid){
    uint8_t boot_seq[] = {0x06, 0x09, 0x2B, 0x06, 0x01, 0x04, 0x01, 0x82, 0x26, 0x01, 0x22};
    memcpy(boot_seq_oid, boot_seq, sizeof(boot_seq));
}

int32_t Bootloader_socCpuSetAppEntryPoint(uint32_t cpuId, uintptr_t entryPoint)
{
   int32_t status = SystemP_SUCCESS;
   /* dummy api call */

    return status;
}