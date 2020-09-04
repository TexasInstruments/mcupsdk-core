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
 *  \file bootloader_profile.c
 *
 *  \brief Bootloader Driver API source file.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <kernel/dpl/SystemP.h>
#include <kernel/dpl/CycleCounterP.h>

#include <drivers/bootloader.h>
#include <drivers/bootloader/bootloader_profile.h>
#include <string.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define BOOTLOADER_PROFILE_MAX_LOGS  (32U)
#define BOOTLOADER_PROFILE_MAX_CORES (CSL_CORE_ID_MAX)

/* ========================================================================== */
/*                             Structure Definitions                          */
/* ========================================================================== */
typedef struct
{
    char *pName;
    uint32_t cycleCount;

} Bootloader_ProfileInfo;

typedef struct
{
    uint32_t logIndex;
    /* Index of the last log item */

    Bootloader_ProfileInfo info[BOOTLOADER_PROFILE_MAX_LOGS];
    /* Array of bootloader profile information structs corresponding to each log item */

    uint32_t appimageSize;
    /* Size of the appimage being booted */

    uint32_t bootMediaID;
    /* ID of bootMedia used, defined in bootloader.h */

    uint32_t bootMediaClk;
    /* OSPI/QSPI/MMCSD clock frequency used in bootloader */

    uint32_t numCores;
    /* Number of cores present in the appimage */

    uint8_t coresPresent[BOOTLOADER_PROFILE_MAX_CORES];
    /* Array to hold IDs of the core present */

} Bootloader_ProfileObject;

/* ========================================================================== */
/*                             Global Variables                               */
/* ========================================================================== */

Bootloader_ProfileObject gProfileObj;

/* ========================================================================== */
/*                             Function Definitions                           */
/* ========================================================================== */
void Bootloader_profileReset(void)
{
    gProfileObj.logIndex     = 0U;
    gProfileObj.numCores     = 0U;
    gProfileObj.bootMediaClk = 0U;
    gProfileObj.bootMediaID  = 0xFFFFFFFFU;

    CycleCounterP_reset();

    /* Add the 0th profile point */
    Bootloader_profileAddProfilePoint("SBL Start");
}

void Bootloader_profileAddProfilePoint(char *pointName)
{
    /* Get PMU count before anything else so as to not waste any cycles */
    uint32_t cycleCount = CycleCounterP_getCount32();

    if(gProfileObj.logIndex < BOOTLOADER_PROFILE_MAX_LOGS)
    {
        uint32_t idx = gProfileObj.logIndex;

        gProfileObj.info[idx].cycleCount = cycleCount;
        gProfileObj.info[idx].pName      = pointName;

        gProfileObj.logIndex++;
    }
    else
    {
        /* Overflow */
    }
}

void Bootloader_profileUpdateAppimageSize(uint32_t size)
{
    gProfileObj.appimageSize = size;
}

void Bootloader_profileUpdateMediaAndClk(uint32_t id, uint32_t clk)
{
    gProfileObj.bootMediaClk = clk;
    gProfileObj.bootMediaID  = id;
}

void Bootloader_profileAddCore(uint32_t coreId)
{
    gProfileObj.coresPresent[gProfileObj.numCores++] = coreId;
}

void Bootloader_profilePrintProfileLog(void)
{
    uint32_t cpuMHz = 0U;
    uint32_t i;

    cpuMHz = SOC_getSelfCpuClk()/1000000;

    /* Assumption: 0th profile point is SBL start and last profile point is SBL end.
    Print diffs for all the points in between and print (SBL end - SBL start) at the end as overall SBL time */

    char *bootMediaName = NULL;

    switch(gProfileObj.bootMediaID)
    {
        case BOOTLOADER_MEDIA_FLASH:
            bootMediaName = "NOR SPI FLASH";
            break;
        case BOOTLOADER_MEDIA_EMMC:
            bootMediaName = "eMMC";
            break;
        case BOOTLOADER_MEDIA_SD:
            bootMediaName = "SD Card";
            break;
        default:
            bootMediaName = "undefined";
            break;
    }
    uint32_t clk = gProfileObj.bootMediaClk;
    
    DebugP_log("[BOOTLOADER_PROFILE] Boot Media       : %s \r\n", bootMediaName);
    /* If boot media clock is not given don't print that information */
    if(clk != 0)
    {
        DebugP_log("[BOOTLOADER_PROFILE] Boot Media Clock : %.3f MHz \r\n", (float)clk/1000000.0);
    }
    DebugP_log("[BOOTLOADER_PROFILE] Boot Image Size  : %d KB \r\n", gProfileObj.appimageSize/1024);
    DebugP_log("[BOOTLOADER_PROFILE] Cores present    : \r\n");

    for(i = 0; i < gProfileObj.numCores; i++)
    {
        DebugP_log("%s\r\n", SOC_getCoreName(gProfileObj.coresPresent[i]));
    }

    for(i = 1; i < gProfileObj.logIndex - 1; i++)
    {
        uint32_t timeDiff = (gProfileObj.info[i].cycleCount - gProfileObj.info[i-1].cycleCount)/cpuMHz;
        DebugP_log("[BOOTLOADER PROFILE] %-32s : %10uus \r\n", gProfileObj.info[i].pName, timeDiff);
    }

    uint32_t sblTotalTime = (gProfileObj.info[gProfileObj.logIndex-1].cycleCount - gProfileObj.info[0].cycleCount)/cpuMHz;

    DebugP_log("[BOOTLOADER_PROFILE] %-32s : %10uus \r\n", "SBL Total Time Taken", sblTotalTime);
    DebugP_log("\r\n");
}
