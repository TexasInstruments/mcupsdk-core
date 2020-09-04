/**
 * \file hsrPrp_red_vlanTable.c
 * \brief Contains VLAN Table management routines
 *
 * \par
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
 * \par
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <networking/icss_emac/icss_emac.h>

#include "hsrPrp_red_vlanTable.h"
#include "hsrPrp_firmwareOffsets.h"

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/* Command can take the following values:
 * VLAN_FLTR_DIS                        0x0
 * VLAN_FLTR_ENA                        0x1
 */
RED_STATUS vlan_filter_config(PRUICSS_Handle pruicssHandle,
                              uint8_t command)
{
    uint8_t *vlanFilterControlByte = ((uint8_t *)(((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->sharedDramBase +
                                      VLAN_FLTR_CTRL_BYTE));

    if(command == VLAN_FLTR_DIS)
    {
        *vlanFilterControlByte |= (1 << VLAN_FLTR_CTRL_SHIFT);
        *vlanFilterControlByte ^= (1 << VLAN_FLTR_CTRL_SHIFT);

#if DEBUG_VLAN_FLT
        DebugP_log("VLAN_FLTR_DIS\n");
#endif
    }

    else    /*VLAN_FLTR_ENA*/
    {
        *vlanFilterControlByte |= (1 << VLAN_FLTR_CTRL_SHIFT);

#if DEBUG_VLAN_FLT
        DebugP_log("VLAN_FLTR_ENA\n");
#endif
    }

    return RED_OK;
}

/* Command can take the following values:
 * VLAN_FLTR_UNTAG_HOST_RCV_ALL         0x0
 * VLAN_FLTR_UNTAG_HOST_RCV_NAL         0x1
 */
RED_STATUS vlan_untagged_frames_config(PRUICSS_Handle pruicssHandle,
                                       uint8_t command)
{
    uint8_t *vlanFilterControlByte = ((uint8_t *)(((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->sharedDramBase +
                                      VLAN_FLTR_CTRL_BYTE));

    if(command == VLAN_FLTR_UNTAG_HOST_RCV_ALL)
    {
        *vlanFilterControlByte |= (1 << VLAN_FLTR_UNTAG_HOST_RCV_CTRL_SHIFT);
        *vlanFilterControlByte ^= (1 << VLAN_FLTR_UNTAG_HOST_RCV_CTRL_SHIFT);

#if DEBUG_VLAN_FLT
        DebugP_log("VLAN_FLTR_UNTAG_HOST_RCV_ALL\n");
#endif
    }

    else    /*VLAN_FLTR_UNTAG_HOST_RCV_NAL*/
    {
        *vlanFilterControlByte |= (1 << VLAN_FLTR_UNTAG_HOST_RCV_CTRL_SHIFT);

#if DEBUG_VLAN_FLT
        DebugP_log("VLAN_FLTR_UNTAG_HOST_RCV_NAL\n");
#endif
    }

    return RED_OK;
}

/* Command can take the following values:
 * VLAN_FLTR_PRIOTAG_HOST_RCV_ALL       0x0
 * VLAN_FLTR_PRIOTAG_HOST_RCV_NAL       0x1
 */
RED_STATUS vlan_priotag_frames_config(PRUICSS_Handle pruicssHandle,
                                      uint8_t command)
{
    uint8_t *vlanFilterControlByte = ((uint8_t *)(((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->sharedDramBase +
                                      VLAN_FLTR_CTRL_BYTE));

    if(command == VLAN_FLTR_PRIOTAG_HOST_RCV_ALL)
    {
        *vlanFilterControlByte |= (1 << VLAN_FLTR_PRIOTAG_HOST_RCV_CTRL_SHIFT);
        *vlanFilterControlByte ^= (1 << VLAN_FLTR_PRIOTAG_HOST_RCV_CTRL_SHIFT);

#if DEBUG_VLAN_FLT
        DebugP_log("VLAN_FLTR_PRIOTAG_HOST_RCV_ALL\n");
#endif
    }

    else    /*VLAN_FLTR_PRIOTAG_HOST_RCV_NAL*/
    {
        *vlanFilterControlByte |= (1 << VLAN_FLTR_PRIOTAG_HOST_RCV_CTRL_SHIFT);

#if DEBUG_VLAN_FLT
        DebugP_log("VLAN_FLTR_PRIOTAG_HOST_RCV_NAL\n");
#endif
    }

    return RED_OK;
}

/* Command can take the following values:
 * VLAN_FLTR_SV_VLAN_FLOW_SKIP       0x0
 * VLAN_FLTR_SV_VLAN_FLOW_TAKE       0x1
 */
RED_STATUS vlan_sv_frames_config(PRUICSS_Handle pruicssHandle,
                                 uint8_t command)
{
    uint8_t *vlanFilterControlByte = ((uint8_t *)(((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->sharedDramBase +
                                      VLAN_FLTR_CTRL_BYTE));

    if(command == VLAN_FLTR_SV_VLAN_FLOW_SKIP)
    {
        *vlanFilterControlByte |= (1 << VLAN_FLTR_SV_CTRL_SHIFT);
        *vlanFilterControlByte ^= (1 << VLAN_FLTR_SV_CTRL_SHIFT);

#if DEBUG_VLAN_FLT
        DebugP_log("VLAN_FLTR_SV_VLAN_FLOW_SKIP\n");
#endif
    }

    else    /*VLAN_FLTR_SV_VLAN_FLOW_TAKE*/
    {
        *vlanFilterControlByte |= (1 << VLAN_FLTR_SV_CTRL_SHIFT);

#if DEBUG_VLAN_FLT
        DebugP_log("VLAN_FLTR_SV_VLAN_FLOW_TAKE\n");
#endif
    }

    return RED_OK;
}

/* 4096 VIDs | 1 bit per VID => 4096 bits ~ 512 bytes ~ 0x200 bytes
 *
 * Update the bit in vlan table as specified by command. Command can take 2 values:
 *          0 : allow packet to host | ADD_VLAN_VID : set bit corresponding to VID to 1
 *          1 : do not allow packet to host | REMOVE_VLAN_VID : set bit corresponding to VID to 0
 */
RED_STATUS vlan_filter_update_vid(PRUICSS_Handle pruicssHandle,
                                  uint16_t vid, uint8_t command)
{
    uint8_t *vlanTableBaseAddr = ((uint8_t *)(((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->sharedDramBase +
                                  VLAN_FLTR_TBL_BASE_ADDR));

#if DEBUG_VLAN_FLT
    DebugP_log("vlanTableBaseAddr:%x\n", vlanTableBaseAddr);
#endif

    uint8_t *vlanTableBytePtr;
    uint16_t vlanTargetByte;
    uint8_t vlanTargetBit;

    vlanTargetByte = vid / 8;
    vlanTargetBit = vid & 0x07;

#if DEBUG_VLAN_FLT
    DebugP_log("vid:%d vlanTargetByte:%x vlanTargetByte:%d vlanTargetBit:%x\n",
                vid, vlanTargetByte, vlanTargetByte, vlanTargetBit);
#endif

    vlanTableBytePtr = vlanTableBaseAddr + vlanTargetByte;

#if DEBUG_VLAN_FLT
    DebugP_log("vlanTableBytePtr:%x\\n", vlanTableBytePtr);
#endif

    if(command == ADD_VLAN_VID)
    {
        *vlanTableBytePtr = *vlanTableBytePtr | (1 << vlanTargetBit);
#if DEBUG_VLAN_FLT
        DebugP_log("ADD_VLAN_VID\n");
#endif
    }

    else    /*REM_VLAN_VID*/
    {
        *vlanTableBytePtr = *vlanTableBytePtr | (1 << vlanTargetBit);
        *vlanTableBytePtr = *vlanTableBytePtr ^ (1 << vlanTargetBit);
#if DEBUG_VLAN_FLT
        DebugP_log("REM_VLAN_VID\n");
#endif
    }

#if DEBUG_VLAN_FLT
    DebugP_log("vlanTableBytePtr updated:%x\n", vlanTableBytePtr);
#endif

    return RED_OK;
}
