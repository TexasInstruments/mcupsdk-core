/**
 * \file hsrPrp_red_tx.c
 * \brief Contains HSR PRP Tx interface routines
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
#include <networking/icss_emac/source/icss_emac_local.h>

#include "hsrPrp_red_tx.h"
#include "hsrPrp_firmwareOffsets.h"
#include "hsrPrp_handle.h"
#ifdef ICSS_PROTOCOL_PRP
#include "hsrPrp_red_prp.h"
#endif
/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* Moved to hsrPrpHandle to facilitate multi-instance capability */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

#ifdef ICSS_PROTOCOL_HSR

RED_STATUS RedTxPacketHsrModeT(ICSS_EMAC_Handle icssEmacHandle,
                               const uint8_t *packetData, int32_t packetLength, int32_t queuePriority)
{
    int32_t ret1, ret2;

    if(packetData == NULL)
    {
        return (RED_ERR);
    }

    ICSS_EMAC_TxArgument    txArgs;

    txArgs.icssEmacHandle = icssEmacHandle;
    txArgs.srcAddress = packetData;
    txArgs.queuePriority = queuePriority;
    txArgs.portNumber = ICSS_EMAC_PORT_1;
    txArgs.lengthOfPacket = packetLength;

    ret1 = ICSS_EMAC_txPacket(&txArgs, NULL);

    txArgs.portNumber = ICSS_EMAC_PORT_2;
    ret2 =ICSS_EMAC_txPacket(&txArgs, NULL);

    if((ret1 == SystemP_FAILURE) || (ret2 == SystemP_FAILURE))
    {
        RED_DEBUG_MSG("%s: TxPacketEnqueue failed!\n", __FUNCTION__);
        return (RED_ERR);
    }

    return (RED_OK);
}

RED_STATUS RedTxPacketHsrModeM(ICSS_EMAC_Handle icssEmacHandle,
                               const uint8_t *packetData, int32_t packetLength, int32_t queuePriority, int32_t port)
{
    if(packetData == NULL)
    {
        return (RED_ERR);
    }

    ICSS_EMAC_TxArgument    txArgs;

    txArgs.icssEmacHandle = icssEmacHandle;
    txArgs.srcAddress = packetData;
    txArgs.portNumber = port;
    txArgs.queuePriority = queuePriority;
    txArgs.lengthOfPacket = packetLength;

    if(ICSS_EMAC_txPacket(&txArgs, NULL) != SystemP_SUCCESS)
    {
        RED_DEBUG_MSG("%s: TxPacketEnqueue failed!\n", __FUNCTION__);
        return (RED_ERR);
    }

    return (RED_OK);
}
#endif /* ICSS_PROTOCOL_HSR */

RED_STATUS RedTxPacketEnqueue(hsrPrpHandle *hsrPrphandle,
                              ICSS_EMAC_Handle icssEmacHandle,
                              RED_FRAME *pRedFrame, int32_t portNumber, int32_t queuePriority)
{
    uint16_t buffer_offset_computed = 0;
    uint16_t queue_wr_ptr = 0;
    uint16_t wrk_queue_wr_ptr = 0;
    uint16_t queue_rd_ptr = 0;
    uint16_t num_of_bytes = 0;
    uint32_t temp = 0;
    uint32_t collision_queue_selected = 0;
    uint16_t col_queue_already_occupied = 0;
    ICSS_EMAC_QueueParams *txQueue;
    ICSS_EMAC_HostStatistics *hostStatPtr;
    ICSS_EMAC_FwStaticMmap *pStaticMMap = (&((ICSS_EMAC_Object *)
                                            icssEmacHandle->object)->fwStaticMMap);
    PRUICSS_Handle              pruicssHandle = ((ICSS_EMAC_Object *)icssEmacHandle->object)->pruicssHandle;
    PRUICSS_HwAttrs const       *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs);

    if(pRedFrame == NULL)
    {
        return (RED_ERR);
    }

    if((portNumber != ICSS_EMAC_PORT_1) && (portNumber != ICSS_EMAC_PORT_2))
    {
        return (RED_ERR);
    }

    /*    Channel all the host traffic to Q3 or Q4 because Tx queues bifurcated as shown:
            Q1, Q2 : for port to port forwarding traffic | PRU is the writer | Q2 is by default the lower priority queue
            Q3, Q4 : for host egress traffic | Host is the writer | Q4 is by default the lower priority queue
    */
    queuePriority = queuePriority / 2 + ICSS_EMAC_QUEUE3;

    if((queuePriority < ICSS_EMAC_QUEUE3)
           || (queuePriority > ICSS_EMAC_QUEUE4))
        {
            return (RED_ERR);
        }

    if((pRedFrame->bufferLen > ICSS_EMAC_MAXMTU_HSR)
            || (pRedFrame->bufferLen < ICSS_EMAC_MINMTU))
    {
        return (RED_ERR);
    }

    if(((ICSS_EMAC_Object *)icssEmacHandle->object)->linkStatus[portNumber - 1] == 0)
    {
        ICSS_EMAC_ioctl(icssEmacHandle, ICSS_EMAC_IOCTL_PORT_FLUSH_CTRL, portNumber, NULL);
        return (RED_ERR);
    }

    ICSS_EMAC_PortParams *sPort;
    sPort = &(((ICSS_EMAC_Object *)
               hsrPrphandle->icssEmacHandle->object)->switchPort[portNumber]);

    txQueue = &(sPort->queue[queuePriority]);

    /* Check whether Queue is busy */

    collision_queue_selected = *((uint32_t *)((pruicssHwAttrs-> pru1DramBase) +
                                 txQueue->queue_desc_offset + 4))
                               & 0x00000100;

    if(collision_queue_selected != 0)
    {

        ((*(hsrPrphandle->num_of_collision_occured))++);

        col_queue_already_occupied = *((uint8_t *)((pruicssHwAttrs->pru1DramBase) +
                                       pStaticMMap->colStatusAddr + portNumber));
        if(col_queue_already_occupied != 0)
        {
            ((*(hsrPrphandle->collision_pkt_dropped))++);
            return (RED_ERR); /* No space in collision queue */
        }

        /* Queue is busy - put the packet in the collision Queue */
        txQueue = &(sPort->queue[ICSS_EMAC_COLQUEUE]);
    }

    else
    {
        /* Queue is NOT busy - acquire the Queue by setting "busy_s" bit */

        *((uint8_t *)((pruicssHwAttrs->pru1DramBase) +
                      txQueue->queue_desc_offset + 4)) = 1;

        /* Again check if host acquired the queue successfully by checking the busy_m bit */

        collision_queue_selected = *((uint32_t *)((pruicssHwAttrs->pru1DramBase) +
                                     txQueue->queue_desc_offset + 4))
                                   & 0x00000100;

        if(collision_queue_selected != 0)
        {
            ((*(hsrPrphandle->num_of_collision_occured))++);


            *((uint8_t *)((pruicssHwAttrs->pru1DramBase) +
                          txQueue->queue_desc_offset + 4)) = 0;

            /* Queue is busy - put the packet in the collision Queue */
            txQueue = &(sPort->queue[ICSS_EMAC_COLQUEUE]);
        }
    }

    /* Read the write pointer from Queue descriptor */

    temp = *((uint32_t *)((pruicssHwAttrs->pru1DramBase) + txQueue->queue_desc_offset));
    queue_wr_ptr = (temp >> 16);
    queue_rd_ptr = (temp & 0x0000ffff);

    /* Get the value of new queue write pointer */
    wrk_queue_wr_ptr = (pRedFrame->bufferLen >> 5); /* Divide by 32 */
    wrk_queue_wr_ptr = (wrk_queue_wr_ptr <<
                        2); /* Multiply by 4 as one descriptor represents 32 bytes and BD takes 4 bytes */

    if((pRedFrame->bufferLen & 0x0000001f) != 0)
    {
        wrk_queue_wr_ptr = wrk_queue_wr_ptr + 4;
    }

    wrk_queue_wr_ptr = wrk_queue_wr_ptr + queue_wr_ptr;

    /* Check if queue is full and there is an wrap around */
    if(((queue_wr_ptr + 4) % txQueue->queue_size) == 0)
    {
        if(queue_rd_ptr ==
                txQueue->buffer_desc_offset)    /* Since queue is not starting from 0 */
        {

            txQueue->qStat.errCount++;
            *((uint8_t *)((pruicssHwAttrs->pru1DramBase) +
                          txQueue->queue_desc_offset + 4)) = 0;
            return (RED_ERR);

        }
    }

    /* Check if the Queue is already full */
    if((queue_wr_ptr + 4) == queue_rd_ptr)
    {

        txQueue->qStat.errCount++;
        *((uint8_t *)((pruicssHwAttrs->pru1DramBase) +
                      txQueue->queue_desc_offset + 4)) = 0;
        return (RED_ERR);
    }

    /* Three cases arise between wr_ptr and rd_ptr */
    if(queue_wr_ptr == queue_rd_ptr)
    {
        /* Check for wrap around */
        if(wrk_queue_wr_ptr >= txQueue->queue_size)
        {
            wrk_queue_wr_ptr = (wrk_queue_wr_ptr % txQueue->queue_size);
            wrk_queue_wr_ptr = wrk_queue_wr_ptr + txQueue->buffer_desc_offset;
        }
    }

    else if(queue_wr_ptr > queue_rd_ptr)
    {
        /* Check for wrap around */
        if(wrk_queue_wr_ptr >= txQueue->queue_size)
        {
            wrk_queue_wr_ptr = (wrk_queue_wr_ptr % txQueue->queue_size);
            wrk_queue_wr_ptr = wrk_queue_wr_ptr + txQueue->buffer_desc_offset;

            if(wrk_queue_wr_ptr >= queue_rd_ptr)
            {
                //TX_OUT_OF_BD(txQueue->queue_desc_offset, txQueue->qStat.errCount);
                txQueue->qStat.errCount++;
                *((uint8_t *)((pruicssHwAttrs->pru1DramBase) +
                              txQueue->queue_desc_offset + 4)) = 0;
                return (RED_ERR);
            }
        }
    }

    else if(queue_wr_ptr < queue_rd_ptr)
    {
        if(wrk_queue_wr_ptr >= queue_rd_ptr)
        {

            txQueue->qStat.errCount++;
            *((uint8_t *)((pruicssHwAttrs->pru1DramBase) +
                          txQueue->queue_desc_offset + 4)) = 0;
            return (RED_ERR);
        }
    }

    /* Compute the offset of buffer descriptor in ICSS shared RAM - queue_wr_ptr points to currently available free buffer */
    buffer_offset_computed = txQueue->buffer_offset + ((queue_wr_ptr -
                             txQueue->buffer_desc_offset) * 8);

    /* Check if queue wrap around has happened. If yes then data can't be stored sequentially. */
    if((wrk_queue_wr_ptr < queue_wr_ptr)
            && (wrk_queue_wr_ptr != txQueue->buffer_desc_offset))
    {
        num_of_bytes = (txQueue->queue_size - queue_wr_ptr) * 8; /* divide by 4 * 32! */

        memcpy(((int32_t *)(((ICSS_EMAC_Attrs *)
               hsrPrphandle->icssEmacHandle->attrs)->l3OcmcBaseAddr +
                         buffer_offset_computed)), (int32_t *)pRedFrame->pDataBuffer, num_of_bytes);

        if(collision_queue_selected != 0)
        {
            buffer_offset_computed = buffer_offset_computed + num_of_bytes;
        }

        else
        {
            buffer_offset_computed = txQueue->buffer_offset;
        }

        memcpy(((int32_t *)(((ICSS_EMAC_Attrs *)
               hsrPrphandle->icssEmacHandle->attrs)->l3OcmcBaseAddr +
                         buffer_offset_computed)), (int32_t *)(pRedFrame->pDataBuffer + num_of_bytes),
               pRedFrame->bufferLen - num_of_bytes);
    }

    else
    {
        memcpy(((int32_t *)(((ICSS_EMAC_Attrs *)
               hsrPrphandle->icssEmacHandle->attrs)->l3OcmcBaseAddr +
                         buffer_offset_computed)), (int32_t *)pRedFrame->pDataBuffer, pRedFrame->bufferLen);
    }

    /* Compute the buffer descriptor:
     *  - length is from bit 18 to 28,
     *  - HSR flag is at bit 4 */
#ifdef ICSS_PROTOCOL_HSR

    *((uint32_t *)((pruicssHwAttrs->sharedDramBase) +
                   queue_wr_ptr)) = ((pRedFrame->bufferLen << 18) | (1 << 4));
#else

    *((uint32_t *)((pruicssHwAttrs->sharedDramBase) +
                   queue_wr_ptr)) = (pRedFrame->bufferLen << 18) | (PRP_VS_EMAC_MODE_MASK);
#endif /* ICSS_PROTOCOL_HSR */

    /* Write new wr_ptr in the queue descriptor */

    *((uint16_t *)((pruicssHwAttrs->pru1DramBase) +
                   txQueue->queue_desc_offset + 2)) = wrk_queue_wr_ptr;
    txQueue->qStat.rawCount++;

    /* Release the Queue. Even if collision queue was selected then below line won't have any impact */

    *((uint8_t *)((pruicssHwAttrs->pru1DramBase) +
                  txQueue->queue_desc_offset + 4)) = 0;

    /* If packet was put in collision queue then indicate it to collision task */
    if(collision_queue_selected != 0)
    {

        *((uint8_t *)((pruicssHwAttrs->pru1DramBase) +
                      pStaticMMap->colStatusAddr + portNumber)) = ((queuePriority << 1) | 0x0001);

    }

    hostStatPtr = (ICSS_EMAC_HostStatistics *)(((ICSS_EMAC_Object *)
                  icssEmacHandle->object)->hostStat);
    hostStatPtr += (portNumber - 1);

    ICSS_EMAC_updateTxStats(((uint8_t *)pRedFrame->pDataBuffer),
                           (uint32_t)(pRedFrame->bufferLen + 4), hostStatPtr);

    return (RED_OK);
}
