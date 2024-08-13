/*
 *  Copyright (C) 2022-23 Texas Instruments Incorporated
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
 *
 */

/**
 *  \file bootloader_hsmRt_load.c
 *
 *  \brief Implementation of hsm runtime boot test for CCS initialization
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <drivers/bootloader/soc/bootloader_soc.h>
#include <kernel/dpl/SystemP.h>
#include <kernel/nortos/dpl/r5/HwiP_armv7r_vim.h>
#include <security/security_common/drivers/hsmclient/hsmclient.h>
#include <kernel/dpl/DebugP.h>
#include <drivers/hw_include/cslr_soc.h>
/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
#define AM263Px_SR11    (0x00000003u)
/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */


extern CSL_top_ctrlRegs * ptrTopCtrlRegs;

/* ========================================================================== */
/*                             Function Definitions                           */
/* ========================================================================== */

void Bootloader_socLoadHsmRtFw(HsmClient_t *gHSMClient, const uint8_t *HsmRtFw, uint32_t hsmRTSize)
{
    int32_t retVal = SystemP_SUCCESS;

    /* Load hsmRt firmware only when device is HSSE or HSFS */
    if(ptrTopCtrlRegs->EFUSE_DEVICE_TYPE == BOOTLOADER_DEVTYPE_HSSE)
    {
        DebugP_logInfo("DevType : HSSE  \r\n");
        retVal = Hsmclient_loadHSMRtFirmware(gHSMClient, (uint8_t*)HsmRtFw);
        DebugP_logInfo("HSMRT Size in Bytes : %ld \r\n", (uint32_t)hsmRTSize);
    }
    else if(ptrTopCtrlRegs->EFUSE_DEVICE_TYPE == BOOTLOADER_DEVTYPE_HSFS)
    {
        DebugP_logInfo("Device Type : HSFS  \r\n");
        retVal = Hsmclient_loadHSMRtFirmware(gHSMClient, (uint8_t*)HsmRtFw);
        DebugP_logInfo("HSMRT Size in Bytes : %ld \r\n", (uint32_t)hsmRTSize);
    }
    else
    {
        /* None */
    }

    if(retVal == SystemP_FAILURE)
    {
        DebugP_logInfo("hsm runtime firmware load failure ... \r\n");
    }
    else if(retVal == SystemP_SUCCESS)
    {
        DebugP_logInfo("hsm runtime firmware load complete ... \r\n");
    }
    else
    {
        /* None */
    }
    DebugP_assertNoLog(retVal==SystemP_SUCCESS);
}

void Bootloader_socLoadHsmRtFwNonBlocking(HsmClient_t *gHSMClient, const uint8_t *HsmRtFw, uint32_t hsmRTSize)
{
    int32_t retVal = SystemP_SUCCESS;

    /* Load hsmRt firmware only when device is HSSE or HSFS */
    if(ptrTopCtrlRegs->EFUSE_DEVICE_TYPE == BOOTLOADER_DEVTYPE_HSSE)
    {
        DebugP_logInfo("DevType : HSSE  \r\n");
        retVal = Hsmclient_loadHSMRtFirmwareNonBlocking((uint8_t*)HsmRtFw);
        DebugP_logInfo("HSMRT Size in Bytes : %ld \r\n", (uint32_t)hsmRTSize);
    }
    else if(ptrTopCtrlRegs->EFUSE_DEVICE_TYPE == BOOTLOADER_DEVTYPE_HSFS)
    {
        DebugP_logInfo("Device Type : HSFS  \r\n");
        retVal = Hsmclient_loadHSMRtFirmware(gHSMClient, (uint8_t*)HsmRtFw);
        DebugP_logInfo("HSMRT Size in Bytes : %ld \r\n", (uint32_t)hsmRTSize);
    }
    else
    {
        /* None */
    }

    if(retVal == SystemP_FAILURE)
    {
        DebugP_logInfo("hsm runtime firmware load failure ... \r\n");
    }
    else if(retVal == SystemP_SUCCESS)
    {
        DebugP_logInfo("hsm runtime firmware load complete ... \r\n");
    }
    else
    {
        /* None */
    }
    DebugP_assertNoLog(retVal==SystemP_SUCCESS);
}
