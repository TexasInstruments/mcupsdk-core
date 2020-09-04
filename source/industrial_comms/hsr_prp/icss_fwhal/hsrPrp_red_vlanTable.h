/**
 * \file hsrPrp_red_vlanTable.h
 * \brief Include file for hsrPrp_red_vlanTable.c
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

#ifndef RED_VLANTABLE_H_
#define RED_VLANTABLE_H_

#ifdef __cplusplus
extern "C"
{
#endif


/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "hsrPrp_red_vlanTable.h"
#include "hsrPrp_red_common.h"

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
/**
 *  \brief set the VLAN_FLTR_CTRL_SHIFT to enable/disable vlan filtering
 *
 *  \param pruicssHandle Handle to PRU ICSS instance. Contains pointers to base addresses & offsets
 *  \param command instruction to enable/disable vlan filtering. Command can take 2 values:
 *                      VLAN_FLTR_DIS                               0x00
 *                      VLAN_FLTR_ENA                               0x01
 *
 *  \return RED_OK on success, RED_ERR otherwise
 *
 */
RED_STATUS vlan_filter_config(PRUICSS_Handle pruicssHandle,
                              uint8_t command);

/**
 *  \brief set the VLAN_FLTR_UNTAG_HOST_RCV_CTRL_SHIFT to enable/disable untagged frame host receive
 *
 *  \param pruicssHandle Handle to PRU ICSS instance. Contains pointers to base addresses & offsets
 *  \param command instruction to enable/disable untagged frame host receive. Command can take 2 values:
 *                      VLAN_FLTR_UNTAG_HOST_RCV_ALL           0x00
 *                      VLAN_FLTR_UNTAG_HOST_RCV_NAL           0x01
 *
 *  \return RED_OK on success, RED_ERR otherwise
 *
 */
RED_STATUS vlan_untagged_frames_config(PRUICSS_Handle pruicssHandle,
                                       uint8_t command);

/**
 *  \brief set the VLAN_FLTR_PRIOTAG_HOST_RCV_CTRL_SHIFT to enable/disable priority frame host receive
 *
 *  \param pruicssHandle Handle to PRU ICSS instance. Contains pointers to base addresses & offsets
 *  \param command instruction to enable/disable priority frame host receive. Command can take 2 values:
 *                      VLAN_FLTR_PRIOTAG_HOST_RCV_ALL           0x00
 *                      VLAN_FLTR_PRIOTAG_HOST_RCV_ALL           0x01
 *
 *  \return RED_OK on success, RED_ERR otherwise
 *
 */
RED_STATUS vlan_priotag_frames_config(PRUICSS_Handle pruicssHandle,
                                      uint8_t command);

/**
 *  \brief set the VLAN_FLTR_SV_CTRL_SHIFT for SV frames to skip/enter VLAN flow
 *
 *  \param pruicssHandle Handle to PRU ICSS instance. Contains pointers to base addresses & offsets
 *  \param command instruction to enable/disable priority frame host receive. Command can take 2 values:
 *                      VLAN_FLTR_SV_VLAN_FLOW_SKIP           0x00
 *                      VLAN_FLTR_SV_VLAN_FLOW_TAKE           0x01
 *
 *  \return RED_OK on success, RED_ERR otherwise
 *
 */
RED_STATUS vlan_sv_frames_config(PRUICSS_Handle pruicssHandle,
                                 uint8_t command);

/**
 *  \brief insert/delete a VID in the VLAN filter table
 *
 *  \param pruicssHandle Handle to PRU ICSS instance. Contains pointers to base addresses & offsets
 *  \param vid vid to be enabled/disabled in the VLAN filter table
 *  \param command instruction to enable/disable VID host receive. Command can take 2 values:
 *          0 : allow packet to host | ADD_VLAN_VID : set VID to 0x01
 *          1 : do not allow packet to host | REM_VLAN_VID | set VID to 0x00
 *
 *  \return RED_OK on success, RED_ERR otherwise
 *
 */
RED_STATUS vlan_filter_update_vid(PRUICSS_Handle
                                  pruicssHandle,
                                  uint16_t vid, uint8_t command);


#ifdef __cplusplus
}
#endif

#endif /* RED_VLANTABLE_H_ */
