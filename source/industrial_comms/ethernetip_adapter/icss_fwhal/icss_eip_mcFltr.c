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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdio.h>
#include <string.h>
#include "icss_eip_mcFltr.h"
#include "icss_eip_firmware.h"

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/* Command can take the following values:
*   MC_FLTR_DIS                         0x0
*   MC_FLTR_ENA                         0x1
*/
void eip_multicast_filter_config(PRUICSS_Handle pruicssHandle,
                                 uint8_t        command)
{
    uint8_t *multicastFilterControlBit = ((uint8_t *)(((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->pru0DramBase +
                                          MC_OP_CTRL_BYT));

    if(command == MC_FLTR_DIS)
    {
        DebugP_log("MULTICAST FILTER DISABLED\n");
        /*clear bit 0*/
        *multicastFilterControlBit &= ~(1 << MC_FLTR_CTRL_BIT);
    }

    else    /*MC_FLTR_ENA*/
    {
        DebugP_log("MULTICAST FILTER ENABLED\n");
        /*set bit 0*/
        *multicastFilterControlBit |= (1 << MC_FLTR_CTRL_BIT);
    }
}

/* Command can take the following values:
*   MC_SP_ENA                           0x0
*   MC_SP_DIS                           0x1
*/
void eip_multicast_stromPrev_config(PRUICSS_Handle  pruicssHandle,
                                    uint8_t         command)
{
    uint8_t *multicastStromPrevControlBit = ((uint8_t *)(((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->pru0DramBase +
                                            MC_OP_CTRL_BYT));

    if(command == MC_SP_DIS)
    {
        DebugP_log("MULTICAST STORM PREVENTION DISABLED\n");
        /*set bit 1*/
        *multicastStromPrevControlBit |= (1 << MC_SP_CTRL_BIT);
    }

    else    /*MC_SP_ENA*/
    {
        DebugP_log("MULTICAST STORM PREVENTION ENABLED\n");
        /*clear bit 1*/
        *multicastStromPrevControlBit &= ~(1 << MC_SP_CTRL_BIT);
    }
}

void eip_multicast_filter_override_hashmask(PRUICSS_Handle  pruicssHandle,
                                            char            *mask)
{
    unsigned int maskTemp[ETHER_ADDR_LEN];
    uint8_t *multicastFilterMask = ((uint8_t *)(((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->pru0DramBase +
                                    MC_FLTR_MSK));
    uint8_t i;

    /*TODO: Review this*/
// #if DEBUG_MC_FLT
//     UART_puts(mask, -2);
// #endif

    /*Extract the hexadecimal values from the incoming mask*/
    sscanf(mask, "%u %u %u %u %u %u*c", &maskTemp[0], &maskTemp[1], &maskTemp[2],
           &maskTemp[3], &maskTemp[4], &maskTemp[5]);

    /*configure the new mask*/
    for(i = 0; i < ETHER_ADDR_LEN; i++)
    {
        multicastFilterMask[i] = (uint8_t) maskTemp[i];
    }
}

/* multicastAddr (48 bit) & multicastFilterMask (48 bit) | XOR the result to obtain a hashVal
 *
 * Update the byte in multicast table as specified by command. Command can take 2 values:
 *          0 : allow packet to host | ADD_MULTICAST_MAC_ID
 *          1 : do not allow packet to host | REMOVE_MULTICAST_MAC_ID
 * */
void eip_multicast_filter_update_macid(PRUICSS_Handle   pruicssHandle,
                                       char             *multicastAddr,
                                       uint8_t          command)
{
    uint8_t *multicastTableBaseAddr = ((uint8_t *)(((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->pru0DramBase +
                                       MC_FLTR_TABLE));

    uint8_t *multicastTablePtr;

    uint8_t *multicastFilterMask = ((uint8_t *)(((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->pru0DramBase +
                                    MC_FLTR_MSK));
    unsigned int multicastAddrTemp[ETHER_ADDR_LEN];
    uint8_t hashVal, i;

    /*Extract the hexadecimal values from the incoming multicastAddr*/
    sscanf(multicastAddr, "%u %u %u %u %u %u%*c", &multicastAddrTemp[0],
           &multicastAddrTemp[1], &multicastAddrTemp[2],
           &multicastAddrTemp[3], &multicastAddrTemp[4], &multicastAddrTemp[5]);

    /* compute the hashVal by XORing all 6 bytes of multicastAddr*/
    for(i = 0, hashVal = 0; i < ETHER_ADDR_LEN; i++)
    {
        multicastAddrTemp[i] = multicastFilterMask[i] & (uint8_t)multicastAddrTemp[i];
#if DEBUG_MC_FLT
        DebugP_log("macid i %x %x\n", multicastAddr[i], (uint8_t)multicastAddrTemp[i]);
#endif
        hashVal = hashVal ^ (uint8_t)multicastAddrTemp[i];
    }

#if DEBUG_MC_FLT
    DebugP_log("hashVal %x\n", hashVal);
#endif
    multicastTablePtr = multicastTableBaseAddr + hashVal;

    if(command == ADD_MC_MAC_ID)
    {
        *multicastTablePtr = MC_FLTR_HOST_RCV_ALL;
    }

    else    /*REMOVE MULTICAST MAC ID*/
    {
        *multicastTablePtr = MC_FLTR_HOST_RCV_NAL ;
    }

#if DEBUG_MC_FLT
    DebugP_log("multicastAddr:%s hashVal hex:%x hashVal dec:%d status:%d",
                multicastAddr, hashVal, hashVal, command);
    DebugP_assert(multicastTablePtr != NULL);
#endif
}

void eip_multicast_filter_init(PRUICSS_Handle pruicssHandle)
{
    memset(((uint8_t *)((((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->pru0DramBase) +
                        MC_FLTR_MSK)), MC_FLTR_INIT_VAL , ETHER_ADDR_LEN);

    *((uint8_t *)((((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->pru0DramBase) +
                  MC_OP_CTRL_BYT)) = MC_OP_CTRL_BYT_DFT;
}
