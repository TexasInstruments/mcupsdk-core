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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <string.h>
#include <kernel/dpl/ClockP.h>
#include <drivers/bootloader.h>
#include <drivers/bootloader/bootloader_priv.h>
#include <drivers/bootloader/soc/bootloader_soc.h>
#include <kernel/dpl/CacheP.h>
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/AddrTranslateP.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

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
/* we will load a dummy while loop here, when there is nothing to load for A53.
 * This is used with SBL NULL, to init A53, so that we can load and run via CCS without
 * GEL files
 */
#define BOOTLOADER_A53_WHILELOOP_LOAD_ADDR      (0x70000040)
#define BOOTLOADER_SYSFW_MAX_SIZE               (0x42000U)

/* ========================================================================== */
/*                             Global Variables                               */
/* ========================================================================== */

Bootloader_resMemSections gResMemSection =
{
    .numSections    = 1,
    .memSection[0].memStart   = 0x41C00100,
    .memSection[0].memEnd     = 0x41C3E000,
};

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
        .tisciProcId    = SCICLIENT_PROCID_R5_CL0_C0,
        .tisciDevId     = TISCI_DEV_MCU_ARMSS0_CPU0,
        .tisciClockId   = TISCI_DEV_MCU_ARMSS0_CPU0_BUS_CPU_CLK,
        .defaultClockHz = (uint32_t)(400*1000000),
        .coreName       = "r5f0-0",
    },

    {
        .tisciProcId    = SCICLIENT_PROCID_R5_CL0_C1,
        .tisciDevId     = TISCI_DEV_MCU_ARMSS0_CPU1,
        .tisciClockId   = TISCI_DEV_MCU_ARMSS0_CPU1_BUS_CPU_CLK,
        .defaultClockHz = (uint32_t)(400*1000000),
        .coreName       = "r5f0-1",
    },

    {
        .tisciProcId    = SCICLIENT_PROCID_A53_CL0_C0,
        .tisciDevId     = TISCI_DEV_COMPUTE_CLUSTER_A53_0,
        .tisciClockId   = TISCI_DEV_COMPUTE_CLUSTER_A53_0_BUS_ARM0_CLK,
        .defaultClockHz = (uint32_t)(800*1000000),
        .coreName       = "a530-0",
    },

    {
        .tisciProcId    = SCICLIENT_PROCID_A53_CL0_C1,
        .tisciDevId     = TISCI_DEV_COMPUTE_CLUSTER_A53_1,
        .tisciClockId   = TISCI_DEV_COMPUTE_CLUSTER_A53_1_BUS_ARM0_CLK,
        .defaultClockHz = (uint32_t)(800*1000000),
        .coreName       = "a530-1",
    },

        {
        .tisciProcId    = SCICLIENT_PROCID_A53_CL1_C0,
        .tisciDevId     = TISCI_DEV_COMPUTE_CLUSTER_A53_2,
        .tisciClockId   = TISCI_DEV_COMPUTE_CLUSTER_A53_2_BUS_ARM1_CLK,
        .defaultClockHz = (uint32_t)(800*1000000),
        .coreName       = "a531-0",
    },

    {
        .tisciProcId    = SCICLIENT_PROCID_A53_CL1_C1,
        .tisciDevId     = TISCI_DEV_COMPUTE_CLUSTER_A53_3,
        .tisciClockId   = TISCI_DEV_COMPUTE_CLUSTER_A53_3_BUS_ARM1_CLK,
        .defaultClockHz = (uint32_t)(800*1000000),
        .coreName       = "a531-1",
    },
};

Bootloader_CoreAddrTranslateInfo gAddrTranslateInfo[] =
{
    /* CSL_CORE_ID_R5FSS0_0 */
    {
        .numRegions = 2,
        .addrRegionInfo =
        {
            {
                .cpuLocalAddr = CSL_MCU_ATCM_BASE,
                .socAddr      = CSL_MCU_ARMSS0_CORE0_ATCM_BASE,
                .regionSize   = CSL_MCU_ATCM_SIZE,
            },
            {
                .cpuLocalAddr = CSL_MCU_BTCM_BASE,
                .socAddr      = CSL_MCU_ARMSS0_CORE0_BTCM_BASE,
                .regionSize   = CSL_MCU_BTCM_SIZE,
            },
        },
    },

    /* CSL_CORE_ID_R5FSS0_1 */
    {
        .numRegions = 2,
        .addrRegionInfo =
        {
            {
                .cpuLocalAddr = CSL_MCU_ATCM_BASE,
                .socAddr      = CSL_MCU_ARMSS0_CORE1_ATCM_BASE,
                .regionSize   = CSL_MCU_ATCM_SIZE,
            },
            {
                .cpuLocalAddr = CSL_MCU_BTCM_BASE,
                .socAddr      = CSL_MCU_ARMSS0_CORE1_BTCM_BASE,
                .regionSize   = CSL_MCU_BTCM_SIZE,
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
    /* CSL_CORE_ID_A53SS1_0 */
    {
        .numRegions = 0,
    },
    /* CSL_CORE_ID_A53SS1_1 */
    {
        .numRegions = 0,
    },
};

/* ========================================================================== */
/*                             Function Definitions                           */
/* ========================================================================== */

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
        4U, 5U, 0U, 1U, 2U, 3U
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

int32_t Bootloader_socCpuPowerOnResetA53(uint32_t cpuId)
{
    int32_t status = SystemP_SUCCESS;

    /* nothing to do, we keep A53 powered off since we dont need it powered-on to load code for it */
    status = Sciclient_pmSetModuleState(TISCI_DEV_COMPUTE_CLUSTER_CPAC0, TISCI_MSG_VALUE_DEVICE_SW_STATE_ON, 0, SystemP_WAIT_FOREVER);
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
            status = SystemP_SUCCESS;
            break;

        case CSL_CORE_ID_A53SS0_0:
        case CSL_CORE_ID_A53SS0_1:
        case CSL_CORE_ID_A53SS1_0:
        case CSL_CORE_ID_A53SS1_1:
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
            status = SystemP_SUCCESS;
            break;

        case CSL_CORE_ID_A53SS0_0:
        case CSL_CORE_ID_A53SS0_1:
        case CSL_CORE_ID_A53SS1_0:
        case CSL_CORE_ID_A53SS1_1:
            /* set boot address */
            {
                status = Bootloader_socCpuRequest(CSL_CORE_ID_A53SS0_0);

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

                if(SystemP_SUCCESS == status)
                {
                    status = Bootloader_socCpuSetClock(CSL_CORE_ID_A53SS0_0, 800000000);
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
    uint32_t sciclientCpuDevId = Bootloader_socGetSciclientCpuDevId(cpuId);

    struct tisci_msg_proc_set_config_req  proc_set_config;

    Bootloader_resMemSections *resMem;
    resMem = Bootloader_socGetSBLMem();

    /* Ensure Power is OFF for each core before configuring TCMs */
    /* SBL running on MCU0, don't fool around with its power */
    if(cpuId == CSL_CORE_ID_R5FSS0_1)
    {
        Sciclient_pmSetModuleState(sciclientCpuDevId, TISCI_MSG_VALUE_DEVICE_SW_STATE_AUTO_OFF, TISCI_MSG_FLAG_AOP, SystemP_WAIT_FOREVER);
    }

    /* Ensure to configure the r5f0_0 with sbl entry point before loading the rprc image*/
    if(entryPoint != 0)
    {
        entryPoint = resMem->memSection[0].memStart;
    }
    if(cpuId == CSL_CORE_ID_R5FSS0_1)
    {
        sciclientCpuProcId = Bootloader_socGetSciclientCpuProcId(cpuId-1);
    }

    /* set entry point for core0 */
    proc_set_config.processor_id = sciclientCpuProcId;
    proc_set_config.bootvector_lo = entryPoint;
    proc_set_config.bootvector_hi = 0;
    proc_set_config.config_flags_1_set = 0;
    proc_set_config.config_flags_1_clear = 0;
    proc_set_config.config_flags_1_clear |= TISCI_MSG_VAL_PROC_BOOT_CFG_FLAG_R5_ATCM_EN;
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

int32_t Bootloader_socCpuSetAppEntryPoint(uint32_t cpuId, uintptr_t entryPoint)
{
   int32_t status = SystemP_SUCCESS;
   uint32_t sciclientCpuProcIdCore;
   struct tisci_msg_proc_set_config_req  proc_set_config;

   sciclientCpuProcIdCore = Bootloader_socGetSciclientCpuProcId(cpuId);
   status = Bootloader_socCpuRequest(cpuId);

    if(SystemP_SUCCESS == status)
    {
       proc_set_config.processor_id = sciclientCpuProcIdCore;
       proc_set_config.bootvector_lo = entryPoint;
       proc_set_config.bootvector_hi = 0x0;
       proc_set_config.config_flags_1_set = 0;
       proc_set_config.config_flags_1_clear = 0;

       if (entryPoint != BOOTLOADER_INVALID_ID) /* Set entry point only is valid */
       {
          status = Sciclient_procBootSetProcessorCfg(&proc_set_config, SystemP_WAIT_FOREVER);

          if(SystemP_SUCCESS == status)
          {
             status = Bootloader_socCpuSetClock(cpuId, 400000000);
          }
       }
    }
    if(status != SystemP_SUCCESS)
    {
        DebugP_logError("CPU set config failed for %s\r\n", Bootloader_socGetCoreName(cpuId));
    }

    return status;
}

int32_t Bootloader_socCpuResetReleaseSelf()
{
    int32_t status = SystemP_SUCCESS;
    uint32_t sciclientCpuProcIdCore0, sciclientCpuDevIdCore0;
    uint32_t sciclientCpuProcIdCore1 = BOOTLOADER_INVALID_ID;
    uint32_t sciclientCpuDevIdCore1 = BOOTLOADER_INVALID_ID;
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

    status = Bootloader_socCpuRequest(CSL_CORE_ID_R5FSS0_0);
    if(status != SystemP_SUCCESS)
    {
       DebugP_logError("CPU request core 0 failed..\r\n");
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

    /* AM65x case (can't use local reset flags): Power down core running SBL */
    Sciclient_pmSetModuleState(sciclientCpuDevIdCore0, TISCI_MSG_VALUE_DEVICE_SW_STATE_AUTO_OFF, 0, SystemP_WAIT_FOREVER);
    if (status != SystemP_SUCCESS)
    {
        DebugP_log("\n Sciclient_pmSetModuleState off failed \n");
    }

    /* release the CPUs */
    Sciclient_procBootReleaseProcessor(sciclientCpuProcIdCore0, 0, SystemP_WAIT_FOREVER);
    Sciclient_procBootReleaseProcessor(sciclientCpuProcIdCore1, 0, SystemP_WAIT_FOREVER);

    /* AM65x case (can't use local reset flags): Power ON CPU0 core, then power ON CPU1 core if necessary */
    Sciclient_pmSetModuleState(sciclientCpuDevIdCore0, TISCI_MSG_VALUE_DEVICE_SW_STATE_ON, 0, SystemP_WAIT_FOREVER);
    if (bDualSelfR5F == TRUE) /* Set entry point only is valid */
    {
       Sciclient_pmSetModuleState(sciclientCpuDevIdCore1, TISCI_MSG_VALUE_DEVICE_SW_STATE_ON, 0, SystemP_WAIT_FOREVER);
    }

    if (status != SystemP_SUCCESS)
    {
        DebugP_log("\n Sciclient_pmSetModuleState on failed \n");
    }

    /* execute wfi, now SYSFW will execute the above commands and reset core0 and core 1 */
    __asm__ __volatile__ ("wfi" "\n\t": : : "memory");

    if(status != SystemP_SUCCESS)
    {
       DebugP_logError("CPU reset sequence failed for %s\r\n", Bootloader_socGetCoreName(CSL_CORE_ID_R5FSS0_0));
    }

    return status;
}

uint32_t Bootloader_socTranslateSectionAddr(uint32_t cslCoreId, uint32_t addr)
{
    uint32_t outputAddr = addr;
    return outputAddr;
}

int32_t Bootloader_socMemInitCpu(uint32_t cpuId)
{
    int32_t status = SystemP_SUCCESS;

    switch(cpuId) {
        case CSL_CORE_ID_R5FSS0_0:
        case CSL_CORE_ID_R5FSS0_1:
            break;

        default:
            break;
    }

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

int32_t Bootloader_socOpenFirewalls(void)
{
    int32_t status = SystemP_FAILURE;
    /* Secure ROM has left firewall regions for FSS DAT0 set.  Disable them for DMA usage. */
    uint16_t i;
    struct tisci_msg_fwl_set_firewall_region_resp respFwCtrl = {0};
    struct tisci_msg_fwl_set_firewall_region_req reqFwCtrl =
    {
        .fwl_id = (uint16_t) CSL_FW_IFSS_MCU_0_FSS_S0_ID,
        .region = (uint16_t) 0,
        .n_permission_regs = (uint32_t) 3,
        /* Set .control to zero to disable the firewall region */
        .control = (uint32_t) 0,
        .permissions[0] = (uint32_t) 0,
        .permissions[1] = (uint32_t) 0,
        .permissions[2] = (uint32_t) 0,
        .start_address = 0,
        .end_address = 0
    };

    for (i = 0; i < CSL_FW_IFSS_MCU_0_FSS_S0_NUM_REGIONS; i++)
    {
        reqFwCtrl.region = i;
        status = Sciclient_firewallSetRegion(&reqFwCtrl, &respFwCtrl, SystemP_TIMEOUT);
        if(status != SystemP_SUCCESS)
        {
            DebugP_logError("MCU FSS0_S0 firewall region # %d disable...FAILED \n", i);
        }
    }
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

Bootloader_resMemSections* Bootloader_socGetSBLMem(void)
{
    return &gResMemSection;
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

int32_t Bootloader_authStart(uintptr_t startAddr, uint32_t size)
{
    return SystemP_SUCCESS;
}

int32_t Bootloader_authUpdate(uintptr_t startAddr, uint32_t size, uint8_t enc)
{
    return SystemP_SUCCESS;
}

int32_t Bootloader_authFinish()
{
    return SystemP_SUCCESS;
}
