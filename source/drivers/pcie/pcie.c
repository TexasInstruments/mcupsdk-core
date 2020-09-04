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

#include <string.h>
#include <drivers/pcie/pcie.h>
#include <drivers/hw_include/csl_types.h>
#include <kernel/dpl/DebugP.h>

/** \brief PCIe device instance details (base address, lanes, speed supported) */
extern Pcie_InitCfg Pcie_initCfg;

void Pcie_init(void)
{
    uint32_t        cnt;
    Pcie_Object     *object;
    uint32_t        deviceNum;

    /* Initialize driver instances */
    for (cnt = 0U; cnt < gPcieConfigNum; cnt++)
    {
        /* Initialize object variables */
        object = gPcieConfig[cnt].object;
        DebugP_assert(NULL != object);

        deviceNum = gPcieConfig[cnt].attrs->deviceNum;
        DebugP_assert(PCIE_MAX_PERIPHS > deviceNum);

        memset(object, 0, sizeof(Pcie_Object));

        /* Initialize the base address for the device instance */
        gPcieConfig[cnt].object->bases = Pcie_initCfg.dev.basesPtr[deviceNum];

    }
}

Pcie_DeviceCfgBaseAddr *Pcie_handleGetBases (Pcie_Handle handle)
{
    Pcie_DeviceCfgBaseAddr *bases = NULL;

    if (handle != NULL)
    {
        Pcie_Config *cfg = (Pcie_Config *)handle;

        bases = cfg->object->bases;
    }

    return bases;
}
