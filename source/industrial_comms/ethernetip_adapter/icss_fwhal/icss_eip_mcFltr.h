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

#ifndef ICSS_EIP_MCFLTR_H_
#define ICSS_EIP_MCFLTR_H_

#ifdef __cplusplus
extern "C"
{
#endif


/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <networking/icss_emac/icss_emac.h>

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
/**
 *  \brief configuration to enable/disable multicast filtering
 *
 *  \param pruicssHandle Handle to PRUICSS instance. Contains pointers to base addresses & offsets
 *  \param command instruction to enable/disable multicast filtering. Command can take 2 values:
 *   MC_FLTR_DIS                         0x0
 *   MC_FLTR_ENA                         0x1
 *
 */
void eip_multicast_filter_config(PRUICSS_Handle pruicssHandle,
                                 uint8_t        command);

/**
 *  \brief configuration to enable/disable storm prevention for multicast traffic
 *
 *  \param pruicssHandle Handle to PRUICSS instance. Contains pointers to base addresses & offsets
 *  \param command instruction to enable/disable multicast filtering. Command can take 2 values:
 *   MC_SP_ENA                           0x0
 *   MC_SP_DIS                           0x1
 *
 */
void eip_multicast_stromPrev_config(PRUICSS_Handle  pruicssHandle,
                                    uint8_t         command);

/**
 *  \brief insert/delete a multicast MAC ID in the multicast filter table
 *
 *  \param pruicssHandle Handle to PRUICSS instance. Contains pointers to base addresses & offsets
 *  \param multicastAddr multicast address to be inserted/deleted into the multicast filter table
 *  \param command instruction to enable/disable multicast filtering. Command can take 2 values:
 *                      0 : allow packet to host | ADD_MULTICAST_MAC_ID
 *                      1 : do not allow packet to host | REMOVE_MULTICAST_MAC_ID
 *
 */
void eip_multicast_filter_update_macid(PRUICSS_Handle   pruicssHandle,
                                       char             *multicastAddr,
                                       uint8_t          command);

/**
 *  \brief configure the mask
 *
 *  \param pruicssHandle Handle to PRUICSS instance. Contains pointers to base addresses & offsets
 *  \param mask mask to be set
 *
 */
void eip_multicast_filter_override_hashmask(PRUICSS_Handle  pruicssHandle,
                                            char            *mask);

/**
 *  \brief multicast filtering init
 *
 *  \param pruicssHandle Handle to PRUICSS instance. Contains pointers to base addresses & offsets
 *
 */
void eip_multicast_filter_init(PRUICSS_Handle pruicssHandle);


#ifdef __cplusplus
}
#endif

#endif /* ICSS_EIP_MCFLTR_H_ */
