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
*          api-generator: 13.05.b3ee589
*          Do not edit it manually.
**********************************************************************
* XHCI driver for both host and device mode header file
**********************************************************************/

#include "cdn_xhci_obj_if.h"

/* parasoft suppress item METRICS-41-3 "Number of blocks of comments per statement, DRV-4926" */

USBSSP_OBJ *USBSSP_GetInstance(void)
{
    static USBSSP_OBJ driver =
    {
        .transferData = USBSSP_TransferData,
        .transferVectorData = USBSSP_TransferVectorData,
        .stopEndpoint = USBSSP_StopEndpoint,
        .resetEndpoint = USBSSP_ResetEndpoint,
        .resetDevice = USBSSP_ResetDevice,
        .isr = USBSSP_Isr,
        .SetMemRes = USBSSP_SetMemRes,
        .init = USBSSP_Init,
        .getDescriptor = USBSSP_GetDescriptor,
        .setAddress = USBSSP_SetAddress,
        .resetRootHubPort = USBSSP_ResetRootHubPort,
        .issueGenericCommand = USBSSP_IssueGenericCommand,
        .endpointSetFeature = USBSSP_EndpointSetFeature,
        .setConfiguration = USBSSP_SetConfiguration,
        .nBControlTransfer = USBSSP_NBControlTransfer,
        .noOpTest = USBSSP_NoOpTest,
        .calcFsLsEPIntrptInterval = USBSSP_CalcFsLsEPIntrptInterval,
        .enableSlot = USBSSP_EnableSlot,
        .disableSlot = USBSSP_DisableSlot,
        .enableEndpoint = USBSSP_EnableEndpoint,
        .disableEndpoint = USBSSP_DisableEndpoint,
        .getMicroFrameIndex = USBSSP_GetMicroFrameIndex,
        .setEndpointExtraFlag = USBSSP_SetEndpointExtraFlag,
        .cleanEndpointExtraFlag = USBSSP_CleanEndpointExtraFlag,
        .getEndpointExtraFlag = USBSSP_GetEndpointExtraFlag,
        .setFrameID = USBSSP_SetFrameID,
        .addEventDataTRB = USBSSP_AddEventDataTRB,
        .forceHeader = USBSSP_ForceHeader,
        .setPortOverrideReg = USBSSP_SetPortOverrideReg,
        .getPortOverrideReg = USBSSP_GetPortOverrideReg,
        .setPortControlReg = USBSSP_SetPortControlReg,
        .getPortControlReg = USBSSP_GetPortControlReg,
        .SaveState = USBSSP_SaveState,
        .RestoreState = USBSSP_RestoreState,
        .getPortConnected = USBSSP_GetPortConnected,
        .getDevAddressState = USBSSP_GetDevAddressState,
    };

    return &driver;
}
