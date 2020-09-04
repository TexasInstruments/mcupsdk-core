/**********************************************************************
* Copyright (C) 2012-2021 Cadence Design Systems, Inc.
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* 1. Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* 3. Neither the name of the copyright holder nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
**********************************************************************
* WARNING: This file is auto-generated using api-generator utility.
*          api-generator: 12.02.13bb8d5
*          Do not edit it manually.
**********************************************************************
* Cadence Core Driver for LPDDR4.
**********************************************************************/
#include "../include/common/lpddr4_obj_if.h"

/* parasoft suppress item METRICS-41-3 "Number of blocks of comments per statement" */

LPDDR4_OBJ *LPDDR4_GetInstance(void)
{
    static LPDDR4_OBJ driver =
    {
        .probe = LPDDR4_Probe,
        .init = LPDDR4_Init,
        .start = LPDDR4_Start,
        .ReadReg = LPDDR4_ReadReg,
        .WriteReg = LPDDR4_WriteReg,
        .GetMmrRegister = LPDDR4_GetMmrRegister,
        .SetMmrRegister = LPDDR4_SetMmrRegister,
        .WriteCtlConfig = LPDDR4_WriteCtlConfig,
        .WritePhyConfig = LPDDR4_WritePhyConfig,
        .WritePhyIndepConfig = LPDDR4_WritePhyIndepConfig,
        .ReadCtlConfig = LPDDR4_ReadCtlConfig,
        .ReadPhyConfig = LPDDR4_ReadPhyConfig,
        .ReadPhyIndepConfig = LPDDR4_ReadPhyIndepConfig,
        .GetCtlInterruptMask = LPDDR4_GetCtlInterruptMask,
        .SetCtlInterruptMask = LPDDR4_SetCtlInterruptMask,
        .CheckCtlInterrupt = LPDDR4_CheckCtlInterrupt,
        .AckCtlInterrupt = LPDDR4_AckCtlInterrupt,
        .GetPhyIndepInterruptMask = LPDDR4_GetPhyIndepInterruptMask,
        .SetPhyIndepInterruptMask = LPDDR4_SetPhyIndepInterruptMask,
        .CheckPhyIndepInterrupt = LPDDR4_CheckPhyIndepInterrupt,
        .AckPhyIndepInterrupt = LPDDR4_AckPhyIndepInterrupt,
        .GetDebugInitInfo = LPDDR4_GetDebugInitInfo,
        .GetLpiWakeUpTime = LPDDR4_GetLpiWakeUpTime,
        .SetLpiWakeUpTime = LPDDR4_SetLpiWakeUpTime,
        .GetEccEnable = LPDDR4_GetEccEnable,
        .SetEccEnable = LPDDR4_SetEccEnable,
        .GetReducMode = LPDDR4_GetReducMode,
        .SetReducMode = LPDDR4_SetReducMode,
        .GetDbiReadMode = LPDDR4_GetDbiReadMode,
        .GetDbiWriteMode = LPDDR4_GetDbiWriteMode,
        .SetDbiMode = LPDDR4_SetDbiMode,
        .GetRefreshRate = LPDDR4_GetRefreshRate,
        .SetRefreshRate = LPDDR4_SetRefreshRate,
        .RefreshPerChipSelect = LPDDR4_RefreshPerChipSelect,
        .DeferredRegVerify = LPDDR4_DeferredRegVerify,
    };

    return &driver;
}
