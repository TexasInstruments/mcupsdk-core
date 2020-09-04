/**
 * \file hsrPrp_red_multicastTable.c
 * \brief Contains Node Table management routines
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
#include <stdio.h>
#include <string.h>


#include <networking/icss_emac/icss_emac.h>

#include "hsrPrp_red_multicastTable.h"
#include "hsrPrp_firmwareOffsets.h"

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/* Command can take the following values:
 * MULTICAST_FILTER_DISABLED                               0x00
 * MULTICAST_FILTER_ENABLED                                0x01
*/

RED_STATUS hsrPrp_multicast_filter_config(PRUICSS_Handle pruicssHandle,
        uint8_t command)
{
    uint8_t *multicastTableControlBit = ((uint8_t*)((((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->pru1DramBase +
                                         M_MULTICAST_TABLE_SEARCH_OP_CONTROL_BIT)));
#if DEBUG_MC_FLT

    if(command == MULTICAST_FILTER_DISABLED)
    {
        DebugP_log("MULTICAST_FILTER_DISABLED\n");
    }

    else    /*MULTICAST_FILTER_ENABLED*/
    {
        DebugP_log("MULTICAST_FILTER_ENABLED\n");
    }

#endif
    *multicastTableControlBit = command;
    return RED_OK;
}

RED_STATUS hsrPrp_multicast_filter_override_hashmask(PRUICSS_Handle
        pruicssHandle,
        uint8_t *mask)
{
    uint8_t *multicastFilterMask;
    uint8_t maskTemp[ETHER_ADDR_LEN];
#if DEBUG_MC_FLT
    DebugP_log(mask);
#endif
    /*extract the hexadecimal values from the incoming mask*/
    sscanf((char *)mask, "%x %x %x %x %x %x", (unsigned int *)&maskTemp[0], (unsigned int *)&maskTemp[1], (unsigned int *)&maskTemp[2],
           (unsigned int *)&maskTemp[3], (unsigned int *)&maskTemp[4], (unsigned int *)&maskTemp[5]);

    multicastFilterMask = ((uint8_t*)((((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->pru1DramBase +
                                       MULTICAST_FILTER_MASK)));

    memcpy(multicastFilterMask , maskTemp, ETHER_ADDR_LEN);
    return RED_OK;
}

/* multicastAddr (48 bit) & multicastFilterMask (48 bit) | XOR the result to obtain a hashVal
 *
 * Update the byte in multicast table as specified by command. Command can take 2 values:
 *          0 : allow packet to host | ADD_MULTICAST_MAC_ID
 *          1 : do not allow packet to host | REMOVE_MULTICAST_MAC_ID
 * */
RED_STATUS hsrPrp_multicast_filter_update_macid(PRUICSS_Handle
        pruicssHandle,
        uint8_t *multicastAddr, uint8_t command)
{
    uint8_t *multicastTableBaseAddr = ((uint8_t *)((((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->pru1DramBase +
                                       MULTICAST_FILTER_TABLE)));

    uint8_t *multicastTablePtr;

    uint8_t *multicastFilterMask = ((uint8_t *)((((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->pru1DramBase +
                                    MULTICAST_FILTER_MASK)));
    uint32_t multicastAddrTemp[ETHER_ADDR_LEN];
    uint8_t hashVal, i;

    /*extract the hexadecimal values from the incoming multicastAddr*/
    sscanf((char *)multicastAddr, "%x %x %x %x %x %x", &multicastAddrTemp[0],
           &multicastAddrTemp[1], &multicastAddrTemp[2], &multicastAddrTemp[3],
           &multicastAddrTemp[4], &multicastAddrTemp[5]);
    /* compute the hashVal by XORing all 6 bytes of multicastAddr*/
    for(i = 0, hashVal = 0; i < ETHER_ADDR_LEN; i++)
    {
        multicastAddrTemp[i] = multicastFilterMask[i] & multicastAddrTemp[i];
#if DEBUG_MC_FLT
        DebugP_log("macid i %x %x\n", multicastAddr[i], multicastAddrTemp[i]);
#endif
        hashVal = hashVal ^ multicastAddrTemp[i];
    }

#if DEBUG_MC_FLT
    DebugP_log("hashVal %x\n", hashVal);
#endif
    multicastTablePtr = multicastTableBaseAddr + hashVal;

    if(command == ADD_MULTICAST_MAC_ID)
    {
        *multicastTablePtr = MULTICAST_FILTER_HOST_RCV_ALLOWED;
    }

    else    /*REMOVE_MULTICAST_MAC_ID*/
    {
        *multicastTablePtr = MULTICAST_FILTER_HOST_RCV_NOT_ALLOWED ;
    }

#if DEBUG_MC_FLT
    DebugP_log("multicastAddr:%s hashVal hex:%x hashVal dec:%d status:%d",
                multicastAddr, hashVal, hashVal, command);
    if(multicastTablePtr < 0)
    {
        return RED_ERR;
    }
#endif

    return RED_OK;
}
