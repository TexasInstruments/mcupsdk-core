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
* Layer interface for the Cadence USB device controller family
**********************************************************************/

#include "cusbd_obj_if.h"

/* parasoft suppress item METRICS-41-3 "Number of blocks of comments per statement, DRV-4926" */

CUSBD_OBJ *CUSBD_GetInstance(void)
{
    static CUSBD_OBJ driver =
    {
        .probe = CUSBD_Probe,
        .init = CUSBD_Init,
        .destroy = CUSBD_Destroy,
        .start = CUSBD_Start,
        .stop = CUSBD_Stop,
        .isr = CUSBD_Isr,
        .epEnable = CUSBD_EpEnable,
        .epDisable = CUSBD_EpDisable,
        .epSetHalt = CUSBD_EpSetHalt,
        .epSetWedge = CUSBD_EpSetWedge,
        .epFifoStatus = CUSBD_EpFifoStatus,
        .epFifoFlush = CUSBD_EpFifoFlush,
        .reqQueue = CUSBD_ReqQueue,
        .reqDequeue = CUSBD_ReqDequeue,
        .getDevInstance = CUSBD_GetDevInstance,
        .dGetFrame = CUSBD_DGetFrame,
        .dSetSelfpowered = CUSBD_DSetSelfpowered,
        .dClearSelfpowered = CUSBD_DClearSelfpowered,
        .dGetConfigParams = CUSBD_DGetConfigParams,
    };

    return &driver;
}
