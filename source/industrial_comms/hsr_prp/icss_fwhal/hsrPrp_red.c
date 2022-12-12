/**
 * \file hsrPrp_red.c
 * \brief Contains HSR PRP initialization, timer, semaphore routines
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

#include <stdlib.h>

#include <kernel/dpl/TaskP.h>
#include <drivers/hw_include/cslr_icss.h>
#include <drivers/hw_include/hw_types.h>


#include <networking/icss_emac/icss_emac.h>
#include <networking/icss_emac/source/icss_emac_local.h>

#include "hsrPrp_firmwareOffsets.h"
#include "hsrPrp_red_tx.h"
#include "hsrPrp_red.h"
#include "hsrPrp_red_nodeTable.h"
#include "hsrPrp_handle.h"

/* ========================================================================== */
/*                             Protocol Set Up                                */
/* ========================================================================== */

static RED_FRAME *(*RedSupFrameAllocate)(hsrPrpHandle *)                     =
    NULL;
static void (*RedSupFrameUpdateSrcAdd)(RED_FRAME *, uint8_t *)   = NULL;
static void (*RedSupFrameUpdateSeqNr)(hsrPrpHandle *,
                                      RED_FRAME *)            = NULL;
static void (*RedSupFrameIncrementSeqNr)(hsrPrpHandle *)               = NULL;
static void (*RedFrameFill)(hsrPrpHandle *, RED_FRAME *, const uint8_t *,
                            int32_t) = NULL;
static void (*RedFrameUpdateLan)(RED_FRAME *, uint16_t)         = NULL;

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define RED_LIFE_CHECK_INIT_TASK_NAME "redSupTask"
#define RED_LIFE_CHECK_INIT_TASK_PRIO         (10)
#define RED_LIFE_CHECK_INIT_TASK_STACK_SIZE   (0X4000)

#define RED_NODE_TABLE_REFRESH_TASK_NAME "RedNodetableRefresh"
#define RED_NODE_TABLE_REFRESH_TASK_PRIO         (5)
#define RED_NODE_TABLE_REFRESH_TASK_STACK_SIZE   (0X4000)

#define RED_LIFE_CHECK_TIMER_PERIOD      (2000000)  /* 2 s */
#define RED_PRU_CHECK_TIMER_PERIOD       (5000)  /* 5000 us */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

uint32_t gtaskRedSupTaskStack[RED_LIFE_CHECK_INIT_TASK_STACK_SIZE/sizeof(uint32_t)] __attribute__((aligned(32)));
uint32_t gtaskRedNodetableRefreshStack[RED_NODE_TABLE_REFRESH_TASK_STACK_SIZE/sizeof(uint32_t)] __attribute__((aligned(32)));

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static RED_FRAME *RedFrameAllocate(const uint8_t *pFrame, int32_t frameSize);
static void       RedFrameFree(RED_FRAME *pRedFrame);
static RED_STATUS RedFrameSend(hsrPrpHandle *, ICSS_EMAC_Handle icssEmacHandle,
                               RED_FRAME *pRedFrame, const uint8_t *pFrame, int32_t frameSize,
                               int32_t queuePriority, uint8_t portNum);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/**
 *  \brief Verifies if outgoing packet is a gratuitous ARP
 *
 *  \param pPacket pointer to packet data
 *
 *  \return RED_OK on positive verification, RED_ERR otherwise
 *
 */
static RED_STATUS isGratuitousArp(const uint8_t *pPacket)
{
    uint16_t prot;
    uint8_t *ipSrc;
    uint8_t *ipDst;

    if(pPacket == NULL)
    {
        return (RED_ERR);
    }

    prot = OS_NetToHost16(*((uint16_t *)(pPacket) + ETHER_TYPE_OFFSET / sizeof(
                                uint16_t)));
    ipSrc = (uint8_t *)(pPacket) + ARP_IP_SRC_OFFSET;
    ipDst = (uint8_t *)(pPacket) + ARP_IP_DST_OFFSET;

    if((prot == ETHER_TYPE_ARP) &&
            (ipSrc[3] == ipDst[3]) && (ipSrc[2] == ipDst[2]) &&
            (ipSrc[1] == ipDst[1]) && (ipSrc[0] == ipDst[0]))
    {
        return (RED_OK);
    }

    return (RED_ERR);
}

void RedProtocolInit(void)
{
#ifdef ICSS_PROTOCOL_PRP
    RedSupFrameAllocate       = PrpSupFrameAllocate;
    RedSupFrameUpdateSrcAdd   = PrpSupFrameUpdateSrcAdd;
    RedSupFrameUpdateSeqNr    = PrpSupFrameUpdateSeqNr;
    RedSupFrameIncrementSeqNr = PrpSupFrameIncrementSeqNr;
    RedFrameFill              = PrpFrameFill;
    RedFrameUpdateLan         = PrpFrameUpdateLanId;
#endif /* ICSS_PROTOCOL_PRP */
#ifdef ICSS_PROTOCOL_HSR
    RedSupFrameAllocate       = HsrSupFrameAllocate;
    RedSupFrameUpdateSrcAdd   = HsrSupFrameUpdateSrcAdd;
    RedSupFrameUpdateSeqNr    = HsrSupFrameUpdateSeqNr;
    RedSupFrameIncrementSeqNr = HsrSupFrameIncrementSeqNr;
    RedFrameFill              = HsrFrameFill;
    RedFrameUpdateLan         = HsrFrameUpdatePathId;
#endif /* ICSS_PROTOCOL_HSR */
}

void RedNodeTableClear(hsrPrpHandle *hsrPrphandle)
{
    PRUICSS_Handle pruicssHandle = ((ICSS_EMAC_Object *)hsrPrphandle->icssEmacHandle->object)->pruicssHandle;

    RED_INDEX_ARRAY_ENTRY *indexArrayBase = hsrPrphandle->indexArrayBase;

    RED_BIN_ARRAY_ENTRY *binArrayBase = hsrPrphandle->binArrayBase;

    RED_NODE_TABLE_ENTRY *nodeTableBase = hsrPrphandle->nodeTableBase;

    uint32_t *lreCntNodes = ((uint32_t *)(((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->sharedDramBase +
                                          LRE_CNT_NODES));

    *lreCntNodes = 0;

    uint16_t i;

    /*loop to initialize pindexArrayNode entries*/
    for(i = 0; i < INDEX_TABLE_MAX_ENTRIES; i++)
    {
        (indexArrayBase + i)->bin_offset = 0 ;
        (indexArrayBase + i)->binNoEntries = 0;
        (indexArrayBase + i)->bitLinBin = 0;
    }

    /*loop to initialize pbinArrayNode->nodetable_offset to INVALID state */
    for(i = 0; i < BIN_ARRAY_MAX_ENTRIES; i++)
    {
        (binArrayBase + i)->nodetable_offset = CONST_NODETABLE_INVALID ;
    }

    uint16_t *nextFreeSlot = (uint16_t *)(((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->sharedDramBase +
                                          NEXT_FREE_SLOT);
    *nextFreeSlot = 0;

    uint8_t *binArrayLock = (uint8_t *)(((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->sharedDramBase +
                                        BIN_ARRAY_LOCK);
    *binArrayLock = 0;

    /*loop to initialize state to 0x10: entry is not valid  */
    for(i = 0; i < NODE_TABLE_NT_MAX_ENTRIES; i++)
    {
        (nodeTableBase + i)->state = 0x10 ;
    }
}

void RedIncrementCounter(uint32_t offset, PRUICSS_Handle  pruicssHandle)
{
    uint32_t reg;
    reg = *((uint32_t *)((((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->sharedDramBase) + offset));
    *((uint32_t *)((((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->sharedDramBase) + offset)) =
                          reg + 1;
}


void RedSupFrameUpdateTlv(hsrPrpHandle *hsrPrphandle, uint8_t type)
{
#ifdef ICSS_PROTOCOL_PRP
    PrpSupFrameUpdateTlv(hsrPrphandle->redSupFrame, type);
#endif /* ICSS_PROTOCOL_PRP */
}

int32_t RedTxPacket(void *icssEmachandle, ICSS_EMAC_TxArgument *txArg, void *userArg)
{
    RED_STATUS ret;
    RED_FRAME *redFrame;
    ICSS_EMAC_Handle icssEmacHandle;
    PRUICSS_Handle pruicssHandle;
    const uint8_t *packetData;
    uint16_t packetLength;
    uint8_t queuePriority;
    uint8_t portNum;
    hsrPrpHandle *hsrPrphandle = (hsrPrpHandle *) userArg ;

    if(NULL == hsrPrphandle)
    {
        RED_DEBUG_MSG("%s: hsrPrphandle failed!\n", __FUNCTION__);
        return (RED_ERR);
    }

    pruicssHandle = ((ICSS_EMAC_Object *)hsrPrphandle->icssEmacHandle->object)->pruicssHandle;

#ifdef ICSS_PROTOCOL_HSR
    uint8_t txPort;
    int16_t hsrMode;
#endif

    icssEmacHandle = txArg->icssEmacHandle;
    packetData = txArg->srcAddress;
    packetLength = txArg->lengthOfPacket;
    queuePriority = txArg->queuePriority;
    portNum = txArg->portNumber;

    /*For non PTP frames the we send out on both ports*/
    if(userArg == NULL)
    {
        portNum = 0;
    }

    if(!(((ICSS_EMAC_Object *)icssEmacHandle->object)->linkStatus[ICSS_EMAC_PORT_1 -
            1]))
    {
        ICSS_EMAC_ioctl(icssEmacHandle, ICSS_EMAC_IOCTL_PORT_FLUSH_CTRL, ICSS_EMAC_PORT_1, NULL);
    }

    if(!(((ICSS_EMAC_Object *)icssEmacHandle->object)->linkStatus[ICSS_EMAC_PORT_2 -
            1]))

    {

        ICSS_EMAC_ioctl(icssEmacHandle, ICSS_EMAC_IOCTL_PORT_FLUSH_CTRL, ICSS_EMAC_PORT_2, NULL);

    }

#ifdef ICSS_PROTOCOL_HSR
    hsrMode = *((uint16_t *)((((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->pru0DramBase) + LRE_HSR_MODE));
#endif /* ICSS_PROTOCOL_HSR */

    if(packetData == NULL)
    {
        return (RED_ERR);
    }

    if((queuePriority < ICSS_EMAC_QUEUE1)
           || (queuePriority > ICSS_EMAC_QUEUE4))
    {
        return (RED_ERR);
    }

    if((packetLength < ICSS_EMAC_MINMTU) || (packetLength > ICSS_EMAC_MAXMTU_HSR))
    {
        return (RED_ERR);
    }

    if(isGratuitousArp(packetData) == RED_OK)
    {
        return (RED_OK);
    }

#ifdef ICSS_PROTOCOL_HSR

    switch(hsrMode)
    {
        case MODET:

            if(RedTxPacketHsrModeT(icssEmacHandle, packetData, packetLength,
                                   queuePriority) == RED_OK)
            {
                RedIncrementCounter(LRE_CNT_RX_C, pruicssHandle);
                return (RED_OK);
            }

            return (RED_ERR);

        case MODEM:

            // TODO: implement local criteria verification here
            txPort = (int32_t)ICSS_EMAC_findMAC(packetData,
                                    ((ICSS_EMAC_Object *)(icssEmacHandle)->object)->macTable);

            if((txPort == ICSS_EMAC_PORT_1) || (txPort == ICSS_EMAC_PORT_2))
            {
                if(RedTxPacketHsrModeM(icssEmacHandle, packetData, packetLength, queuePriority,
                                       txPort) == RED_OK)
                {
                    RedIncrementCounter(LRE_CNT_RX_C, pruicssHandle);
                    return (RED_OK);
                }

                return (RED_ERR);
            }

            break;

        default:
            break;
    }

#endif /* ICSS_PROTOCOL_HSR */

    redFrame = RedFrameAllocate(packetData, packetLength);

    if(!redFrame)
    {
        RED_DEBUG_MSG("%s: RedFrameAllocate failed!\n", __FUNCTION__);
        return (RED_ERR);
    }

    ret = RedFrameSend(hsrPrphandle, icssEmacHandle, redFrame, packetData,
                       packetLength,
                       queuePriority, portNum);

    RedFrameFree(redFrame);
    redFrame = NULL;

    if(ret)
    {
        RED_DEBUG_MSG("%s: RedFrameSend failed!\n", __FUNCTION__);
        return (RED_ERR);
    }

    RedIncrementCounter(LRE_CNT_RX_C, pruicssHandle);
    return (RED_OK);
}

int32_t RedRxPktGet(void *icssEmachandle, ICSS_EMAC_RxArgument *rxVoidArg, void *userArg)
{
    ICSS_EMAC_RxArgument *rxArg = (ICSS_EMAC_RxArgument *)rxVoidArg;
    uint16_t queue_rd_ptr = 0;
    uint16_t queue_wr_ptr = 0;
    uint16_t rd_buf_desc_num = 0;
    uint32_t rd_buf_desc = 0;
    uint16_t rd_packet_length = 0;
    uint32_t rd_buffer_l3_addr = 0;
    uint16_t size = 0;
    uint16_t update_rd_ptr = 0;
    uint16_t rx_num_of_bytes = 0;
    uint16_t new_size = 0;
    uint16_t checkVlanTagTpid = 0;
    ICSS_EMAC_QueueParams *rxQueue = NULL;
    hsrPrpHandle *hsrPrphandle = (hsrPrpHandle *) userArg ;
    ICSS_EMAC_Queue *qDesc =  NULL;
    uint16_t    shadow = 0;

    ICSS_EMAC_Handle icssEmacHandle;
    uint32_t destAddress = 0;
    uint8_t queueNumber = 0;
    uint8_t isSupFrame = 0;

#ifdef DEBUG
    uint8_t ntLookupSucessFirmware = 0;
#endif

    uint8_t *srcMacId = NULL;
    uint8_t *destMacId = NULL;
    uint8_t *macId = NULL;

    uint16_t *typeProt = NULL;
    uint16_t  typeProt1 = 0;
    uint16_t  typeProt2 = 0;

    ICSS_EMAC_HashTable *macTablePtr = NULL;
    ICSS_EMAC_HostStatistics *hostStatPtr = NULL;

    ICSS_EMAC_CallBackConfig *learningExcCallback;
    uint8_t emacMode = 0;

    ICSS_EMAC_PortParams *sPort;

#ifdef ICSS_PROTOCOL_HSR
    Uint16 start_offset;
#endif /* ICSS_PROTOCOL_HSR */

#ifdef ICSS_PROTOCOL_HSR
    start_offset = 0;
#endif /* ICSS_PROTOCOL_HSR */

    /*Set to 1 if it's a PTP frame*/
    uint8_t ptp_pkt = 0;
    /*Used to copy timestamp*/
    uint16_t aligned_length = 0;
    uint16_t true_rd_ptr = 0;

    icssEmacHandle = rxArg->icssEmacHandle;
    destAddress = rxArg->destAddress;
    queueNumber = rxArg->queueNumber;


    ICSS_EMAC_FwStaticMmap *pStaticMMap = (&((ICSS_EMAC_Object *)
                                            icssEmacHandle->object)->fwStaticMMap);
    ICSS_EMAC_FwDynamicMmap *pDynamicMMap = (&((ICSS_EMAC_Object *)
                                            icssEmacHandle->object)->fwDynamicMMap);
    uint32_t hostQDescOffset  = pDynamicMMap->hostQ1RxContextOffset + 64U;


    if(((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->portMask ==
            ICSS_EMAC_MODE_SWITCH)
    {
        emacMode = 0;
    }

    else
    {
        emacMode = 1U;
    }

#ifdef SWITCH_DEBUG
    genSeqOfEvents(RX_PACKET_GET);
#endif

    if(emacMode == 0U)   /*Switch Mode*/
    {
        qDesc = (ICSS_EMAC_Queue *)((((ICSS_EMAC_Object *) hsrPrphandle->icssEmacHandle->object)->pruicssHandle->hwAttrs)->pru1DramBase +
                                   pStaticMMap->p0QueueDescOffset + (queueNumber * 8));
    }

    else
    {
        qDesc = (ICSS_EMAC_Queue *)((((ICSS_EMAC_Object *) hsrPrphandle->icssEmacHandle->object)->pruicssHandle->hwAttrs)->sharedDramBase +
                                   hostQDescOffset  + (queueNumber * 8));
    }

    queue_wr_ptr = qDesc->wr_ptr;
    queue_rd_ptr = qDesc->rd_ptr;
    sPort = &(((ICSS_EMAC_Object *)
               hsrPrphandle->icssEmacHandle->object)->switchPort[ICSS_EMAC_PORT_0]);

    if(qDesc->overflow_cnt > 0)
    {
        sPort->queue[queueNumber].qStat.errCount +=
            qDesc->overflow_cnt;        /* increment missed packets to error counter */
        qDesc->overflow_cnt = 0;
        /* reset to zero as limited to 256 anyway */
    }

    rd_buf_desc = HW_RD_REG32((((ICSS_EMAC_Object *) hsrPrphandle->icssEmacHandle->object)->pruicssHandle->hwAttrs)->sharedDramBase +
                        queue_rd_ptr);

    sPort = &(((ICSS_EMAC_Object *)hsrPrphandle->
               icssEmacHandle->object)->switchPort[ICSS_EMAC_PORT_0]);
    rxQueue = &(sPort->queue[queueNumber]);

#ifdef ICSS_PROTOCOL_HSR
    start_offset = (rd_buf_desc & 0x00000001) ? HSR_TAG_SIZE : 0;
#endif /* ICSS_PROTOCOL_HSR */

    if(emacMode == 0U)   /*Switch Mode*/
    {
        /* Determine the address of the first buffer descriptor from the rd_ptr */
        /*Check if Packet was received in collision queue or not */
        shadow = ((uint16_t)((rd_buf_desc & 0x00004000U) >> 14U));

        if(shadow != 0)
        {
            /* Pick the data from collision buffer's */
#ifdef ICSS_PROTOCOL_HSR
            rd_buffer_l3_addr = (((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->l3OcmcBaseAddr +
                                 pDynamicMMap->p0ColBufferOffset + start_offset);
#else
            rd_buffer_l3_addr = (((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->l3OcmcBaseAddr +
                                 pDynamicMMap->p0ColBufferOffset);
#endif
        }

        else
        {

            rd_buf_desc_num = (queue_rd_ptr - rxQueue->buffer_desc_offset) >> 2;
            rd_buffer_l3_addr = (((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->l3OcmcBaseAddr +
                                 (rd_buf_desc_num * 32) + rxQueue->buffer_offset);

#ifdef ICSS_PROTOCOL_HSR
            rd_buffer_l3_addr = (((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->l3OcmcBaseAddr +
                                 (rd_buf_desc_num * 32) + rxQueue->buffer_offset + start_offset);
#else

            rd_buffer_l3_addr = (((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->l3OcmcBaseAddr +
                                 (rd_buf_desc_num * 32) + rxQueue->buffer_offset);
#endif /* ICSS_PROTOCOL_HSR */
        }
    }

    else
    {
        rd_buf_desc_num = (queue_rd_ptr - rxQueue->buffer_desc_offset) >> 2;
        rd_buffer_l3_addr = (((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->l3OcmcBaseAddr +
                             (rd_buf_desc_num * 32) + rxQueue->buffer_offset);
    }

    /* Take out the port number - it may have changed */
    rxArg->port = (0x00030000 & rd_buf_desc) >> 16;

    srcMacId = (uint8_t *)(rd_buffer_l3_addr + 6);

    destMacId = (uint8_t *)rd_buffer_l3_addr;

    rd_packet_length = (0x1ffc0000 & rd_buf_desc) >> 18;

    size = (rd_packet_length >> 2);

    if((rd_packet_length & 0x00000003) != 0)
    {
        size = size + 1;
    }

    /*Compute number of buffer desc required & update rd_ptr in queue */
    update_rd_ptr = ((rd_packet_length >> 5) * 4) + queue_rd_ptr;

    if((rd_packet_length & 0x0000001f) !=
            0)  /* checks multiple of 32 else need to increment by 4 */
    {
        update_rd_ptr += 4;
    }

    true_rd_ptr = update_rd_ptr;

    /*If timestamp bit is set then we need to account
     * for additional 32B used for Rx timestamp*/
    if(rd_buf_desc & 0x8000)
    {
        ptp_pkt = 1;
        update_rd_ptr += 4U;
    }

    /*Check for wrap around */
    if(update_rd_ptr >= rxQueue->queue_size)
    {
        update_rd_ptr = update_rd_ptr - (rxQueue->queue_size -
                                         rxQueue->buffer_desc_offset);
    }
    if(true_rd_ptr >= rxQueue->queue_size)
    {
        true_rd_ptr = true_rd_ptr - (rxQueue->queue_size - rxQueue->buffer_desc_offset);
    }

    if(rd_packet_length <=
            ICSS_EMAC_MAXMTU_HSR)        /* make sure we do not have too big packets */
    {

        if(((ICSS_EMAC_Attrs *)
                icssEmacHandle->attrs)->learningEnable)  /*Switch Mode*/
        {

            learningExcCallback = &((((ICSS_EMAC_Object *)icssEmacHandle->object)->callBackObject).learningExCallBack);
            macTablePtr = (ICSS_EMAC_HashTable *)(&(((ICSS_EMAC_Object *)icssEmacHandle->object)->macTable[0]));
            ICSS_EMAC_updateHashTable(icssEmacHandle, srcMacId, rxArg->port, macTablePtr, learningExcCallback);
        }

#ifdef ICSS_PROTOCOL_HSR
        rd_packet_length -= start_offset;
#endif /* ICSS_PROTOCOL_HSR */

        /* Copy the data from switch buffers to DDR */
        if( (true_rd_ptr < queue_rd_ptr)
                && (true_rd_ptr != rxQueue->buffer_desc_offset))
        {
            typeProt = (uint16_t *)rd_buffer_l3_addr + 6;
            typeProt1 = (*typeProt << 8);
            typeProt2 = (*typeProt >> 8);
            typeProt1 = typeProt1 | typeProt2;
            rx_num_of_bytes = (rxQueue->queue_size - queue_rd_ptr);
            rx_num_of_bytes = (rx_num_of_bytes >> 2);
            rx_num_of_bytes = (rx_num_of_bytes << 5);

#ifdef ICSS_PROTOCOL_HSR
            rx_num_of_bytes -= start_offset;
#endif /* ICSS_PROTOCOL_HSR */

            memcpy((int32_t *)destAddress, (int32_t *)rd_buffer_l3_addr, rx_num_of_bytes);
            destAddress = destAddress + rx_num_of_bytes;
            new_size = rd_packet_length - rx_num_of_bytes;

            if(emacMode == 0U)   /*Switch Mode*/
            {
                if(shadow != 0)
                {
                    rd_buffer_l3_addr = rd_buffer_l3_addr + rx_num_of_bytes;
                }

                else
                {
                    rd_buffer_l3_addr = (((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->l3OcmcBaseAddr +
                                         rxQueue->buffer_offset);
                }
            }

            else
            {
                rd_buffer_l3_addr = (((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->l3OcmcBaseAddr +
                                     rxQueue->buffer_offset);
            }

            memcpy((int32_t *)destAddress, (int32_t *)rd_buffer_l3_addr, new_size);
            /*Copy and append the timestamp to packet if it's a PTP frame*/
            if(ptp_pkt)
            {
				if((new_size & 0x1f) == 0)
					aligned_length = new_size;
				else
					aligned_length = (new_size & 0xFFE0) + 32;
                memcpy((void*)(destAddress + new_size), (void*)(rd_buffer_l3_addr + aligned_length), (size_t)10);
            }
        }

        else
        {

            memcpy((int32_t *)destAddress, (int32_t *)rd_buffer_l3_addr, rd_packet_length);
            /*Copy and append the timestamp to packet if it's a PTP frame*/
            if(ptp_pkt)
            {
				if((true_rd_ptr == rxQueue->buffer_desc_offset) && (true_rd_ptr < queue_rd_ptr))
				{
					rd_buffer_l3_addr = (((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->l3OcmcBaseAddr +
                                         rxQueue->buffer_offset);
					aligned_length = 0;
				}
				else
				{
					if((rd_packet_length & 0x1f) == 0)
						aligned_length = rd_packet_length;
					else
						aligned_length = (rd_packet_length & 0xFFE0) + 32;
				}
                memcpy((void*)(destAddress + rd_packet_length), (void*)(rd_buffer_l3_addr + aligned_length), (size_t)10);
            }
            typeProt = (uint16_t *)destAddress + 6;
            typeProt1 = ((uint16_t)((*typeProt) << 8U));
            typeProt2 = ((uint16_t)((*typeProt) >> 8U));
            typeProt1 = typeProt1 | typeProt2;
        }
    }

    else  /* wrong packet size (exceeds ICSS_EMAC_MAXMTU)*/
    {
        rxQueue->qStat.errCount++;
        return -1;
    }


    if(emacMode == 0U)   /*Switch Mode*/
    {

        /* Write back to queue */
        HW_WR_REG16((((ICSS_EMAC_Object *) icssEmacHandle->object)->pruicssHandle->hwAttrs)->pru1DramBase +
               pStaticMMap->p0QueueDescOffset + (queueNumber * 8), update_rd_ptr);

        /* Check if Host needs to change the wr_ptr for collision queue as well */
        if(shadow != 0)
        {
            ICSS_EMAC_Queue *qDescCol = (ICSS_EMAC_Queue *)((((ICSS_EMAC_Object *) icssEmacHandle->object)->pruicssHandle->hwAttrs)->pru1DramBase +
                                       pStaticMMap->p0ColQueueDescOffset);
            //Write back to collision queue desc
            HW_WR_REG16((((ICSS_EMAC_Object *) icssEmacHandle->object)->pruicssHandle->hwAttrs)->pru1DramBase +
                   pStaticMMap->p0ColQueueDescOffset + 2,  qDescCol->rd_ptr);
            //clear the bit indicating that host is still reading collision queue
            HW_WR_REG8((((ICSS_EMAC_Object *) icssEmacHandle->object)->pruicssHandle->hwAttrs)->pru1DramBase +
                   pStaticMMap->colStatusAddr, 0);
            /*Fix for successive Host collision issue. Clearing the bit to indicate that
             * Host has emptied the collision queue*/
            HW_WR_REG8((((ICSS_EMAC_Object *) icssEmacHandle->object)->pruicssHandle->hwAttrs)->pru1DramBase +
                   pStaticMMap->colStatusAddr + 3, 0);
        }
    }

    else
    {
        HW_WR_REG16((((ICSS_EMAC_Object *) icssEmacHandle->object)->pruicssHandle->hwAttrs)->sharedDramBase +
               hostQDescOffset  + (queueNumber * 8),  update_rd_ptr);
    }

    rxQueue->qStat.rawCount++;

    rxArg->more = 0;

    if(emacMode == 0U)   /*Switch Mode*/
    {
        qDesc = (ICSS_EMAC_Queue *)((((ICSS_EMAC_Object *) icssEmacHandle->object)->pruicssHandle->hwAttrs)->pru1DramBase +
                                    pStaticMMap->p0QueueDescOffset + (queueNumber * 8));
    }

    else
    {
        qDesc = (ICSS_EMAC_Queue *)((((ICSS_EMAC_Object *) icssEmacHandle->object)->pruicssHandle->hwAttrs)->sharedDramBase +
                                   hostQDescOffset + (queueNumber * 8));
    }

    queue_wr_ptr = qDesc->wr_ptr;

    if(update_rd_ptr != queue_wr_ptr)
        rxArg->more = 1;

    hostStatPtr = (ICSS_EMAC_HostStatistics *)((((ICSS_EMAC_Object *)
                                         icssEmacHandle->object)->hostStat));

    if(!emacMode) /*In Switch Mode both the stat structures are in single handle.Depending on port update the corresponding structure*/
    {
        hostStatPtr += (rxArg->port - 1);
    }

    ICSS_EMAC_updateRxStats(destMacId, rd_packet_length, typeProt1, hostStatPtr);

    /*Rx Buffer Descriptor : Bit 5 is supervision frame bit
     *      0: not a supervision frame
     *      1: supervision frame*/
    isSupFrame = rd_buf_desc & (1 << 5);

#ifdef DEBUG
    uint8_t i;

    for(i = 0; i < ETHER_ADDR_LEN; i++)
    {
        DebugP_log("%x ", *(destMacId + i));
    }

    for(i = 0; i < ETHER_ADDR_LEN; i++)
    {
        DebugP_log("%x ", *(destMacId + ETHER_ADDR_LEN + i));
    }

    DebugP_log("Sup Frame %x\n", *(uint16_t *)(destMacId + 12));
#endif

#if (DEBUG_MC_FLT || DEBUG_VLAN_FLT)
    uint16_t i;

    for(i = 0; i < rd_packet_length; i++)
    {
        DebugP_log("%x ", *(destMacId + i));
    }

    DebugP_log("\n");
#endif

#ifdef DEBUG
    /*Rx Buffer Descriptor : Bit 6 is nodetable lookup in host bit
     *      0: NT not accessed in firmware => MACID not present in NT , needs to be inserted in host
     *      1: NT accessed => present in NT, skip nodetable lookup in host*/
    ntLookupSucessFirmware = rd_buf_desc & (1 << 6);

    //    if(!ntLookupSucessFirmware)
#endif
    {
        if(isSupFrame)
        {
#ifdef DEBUG
            DebugP_log("Sup Frame %x\n", *(uint16_t *)(destMacId + 12));
#endif
            /*Search for the srcMacId of the incoming packet in the supervision frame body*/
            macId = destMacId + ETHER_ADDR_LEN + ETHER_ADDR_LEN + 8;

#ifdef DEBUG
            uint8_t i;

            for(i = 0; i < ETHER_ADDR_LEN; i++)
            {
                DebugP_log("%x ", *(macId + i));
            }

            DebugP_log("isSupFrame : %x ", isSupFrame);
            DebugP_log("ntLookupSuccessFirmware : %x\n", ntLookupSucessFirmware);
#endif


            memcpy(&checkVlanTagTpid,
                   (uint8_t *)(destMacId + ETHER_ADDR_LEN + ETHER_ADDR_LEN) , 2);

            uint16_t descFlags = rd_buf_desc & 0xffff;
#ifdef DEBUG
            DebugP_log("descFlags : %x\n", descFlags);
#endif
            if (descFlags & (1 << 3)) //If VDAN flag is set
                descFlags &= ~(1 << 2); //Clear RBX flag

            if(checkVlanTagTpid == VLAN_TAG_TPID_BIG_ENDIAN)
            {
                macId += VLAN_TAG_SIZE;
                descFlags |= 2;
            }

            RedNodetableSearchOp(macId, rxArg->port, isSupFrame, hsrPrphandle, descFlags);

        }

        else
        {
#ifdef ICSS_PROTOCOL_PRP
            memcpy(&checkVlanTagTpid, (uint8_t *)(destMacId + ETHER_ADDR_LEN + ETHER_ADDR_LEN) , 2);
            macId = destMacId + ETHER_ADDR_LEN;
            uint16_t descFlags = rd_buf_desc & 0xffff;
#ifdef DEBUG
            DebugP_log("descFlags : %x\n", descFlags);
#endif
            if (descFlags & (1 << 3)) //If VDAN flag is set
                descFlags &= ~(1 << 2); //Clear RBX flag

            if(checkVlanTagTpid == VLAN_TAG_TPID_BIG_ENDIAN)
            {
                macId += VLAN_TAG_SIZE;
                descFlags |= 2;
            }
#ifdef DEBUG
            uint8_t i;
            for(i = 0; i < ETHER_ADDR_LEN; i++)
            {
                DebugP_log("%x ", *(macId + i));
            }

            DebugP_log("isSupFrame : %x ", isSupFrame);
            DebugP_log("ntLookupSuccessFirmware : %x\n", ntLookupSucessFirmware);
#endif
            RedNodetableSearchOp(macId, rxArg->port, isSupFrame, hsrPrphandle, descFlags);
#endif  // ICSS_PROTOCOL_PRP
        }
    }
    return rd_packet_length;
}

/**
 *  \brief Function which updates PRU Check Flags
 *
 *  \param obj ClockP_object
 *
 *  \param arg arg
 *
 *  \return none
 *
 */
static void RedPruCheckTimerHandler(ClockP_Object *obj, void *arg)
{
    uint32_t regVal = 0;

    hsrPrpHandle *hsrPrphandle = (hsrPrpHandle *)arg;

    ICSS_EMAC_Handle icssEmacHandle;
    icssEmacHandle = hsrPrphandle->icssEmacHandle;

    if(hsrPrphandle->redPruCheckTimerHostTableFlag)
    {
        regVal |= HOST_TIMER_NODE_TABLE_CHECK_BIT;
        regVal |= HOST_TIMER_HOST_TABLE_CHECK_BIT;

        hsrPrphandle->redPruCheckTimerHostTableFlag =
            0;  // Do Port(s) Table check next time
    }

    else
    {
        regVal |= HOST_TIMER_PORT_TABLE_CHECK_BIT;

        hsrPrphandle->redPruCheckTimerHostTableFlag =
            1;  // Do Host Table check next time

        if(hsrPrphandle->redPruCheckTimerNodeTableClear)
        {
            regVal |= HOST_TIMER_NODE_TABLE_CLEAR_BIT;

            hsrPrphandle->redPruCheckTimerNodeTableClear = 0;
        }
    }

    *((uint32_t *)(((((ICSS_EMAC_Object *) icssEmacHandle->object)->pruicssHandle->hwAttrs)->pru1DramBase) +
                   HOST_TIMER_CHECK_FLAGS)) = regVal;
}

/**
 *  \brief Readies the RED Life Check task waiting for the semaphore
 *
 *  \param obj ClockP_Object
 *
 *  \param arg arg
 *
 *  \return none
 *
 */
static void RedLifeCheckTimerHandler(ClockP_Object *obj, void *arg)
{
    hsrPrpHandle *hsrPrphandle = (hsrPrpHandle *)arg;
    SemaphoreP_post(&(hsrPrphandle->redLifeCheckSemaphore));
}

/**
 *  \brief Function which delays start of sending RED Supervision Frames until Link Up
 *
 *  \param args arg
 *
 *
 *  \return none
 *
 */
void RedLifeCheckTask(void *args)
{
    uint8_t *srcAddrPtr;
    uint8_t srcAddr[ETHER_ADDR_LEN];

    int32_t ret1, ret2;

    ICSS_EMAC_Handle icssEmacHandle;

    hsrPrpHandle *hsrPrphandle = (hsrPrpHandle *)args;
    icssEmacHandle = hsrPrphandle->icssEmacHandle;


    /* Wait for link up */
    while(1)
    {
        if(((ICSS_EMAC_Object *)icssEmacHandle->object)->linkStatus[ICSS_EMAC_PORT_1 - 1]
                || ((ICSS_EMAC_Object *)icssEmacHandle->object)->linkStatus[ICSS_EMAC_PORT_2 -
                        1])
        {
            break;
        }

        ClockP_usleep(ClockP_ticksToUsec(100));
    }

    /* Get DANH MAC address */

    srcAddrPtr = (uint8_t *)(((ICSS_EMAC_Object *)icssEmacHandle->object)->macId);
    memcpy(srcAddr, srcAddrPtr, ETHER_ADDR_LEN);
    RedSupFrameUpdateSrcAdd(hsrPrphandle->redSupFrame, srcAddr);


    /* Send RED Supervision frame every 2 s */
    ClockP_start(&(hsrPrphandle->redLifeCheckTimer));

    /* Main loop */
    while(1)
    {
        SemaphoreP_pend(&(hsrPrphandle->redLifeCheckSemaphore), SystemP_WAIT_FOREVER);

        RedSupFrameUpdateSeqNr(hsrPrphandle, hsrPrphandle->redSupFrame);

        RedFrameUpdateLan(hsrPrphandle->redSupFrame, RED_LAN_A_MAGIC);
        ret1 = RedTxPacketEnqueue(hsrPrphandle, icssEmacHandle,
                                  hsrPrphandle->redSupFrame,
                                  ICSS_EMAC_PORT_1,
                                  ICSS_EMAC_QUEUE4);
        RedFrameUpdateLan(hsrPrphandle->redSupFrame, RED_LAN_B_MAGIC);
        ret2 = RedTxPacketEnqueue(hsrPrphandle, icssEmacHandle,
                                  hsrPrphandle->redSupFrame,
                                  ICSS_EMAC_PORT_2,
                                  ICSS_EMAC_QUEUE4);

        if((ret1) && (ret2))
        {
            continue;
        }

        RedSupFrameIncrementSeqNr(hsrPrphandle);
    }
}

RED_STATUS RedLifeCheckTaskCreate(hsrPrpHandle *hsrPrphandle,
                                  PRUICSS_Handle      pruicssHandle)
{

    int32_t status_redPruCheckTimer = SystemP_FAILURE;
    int32_t status_redLifeCheckSemaphore = SystemP_FAILURE;
    int32_t status_nodesTableSemaphore = SystemP_FAILURE;
    int32_t status_redLifeCheckTimer = SystemP_FAILURE;
    int32_t status_redSupTask = SystemP_FAILURE;
    int32_t status_hsrPrpNodetableRefreshTask = SystemP_FAILURE;

    TaskP_Params taskParams;
    ClockP_Params clockParams;


    hsrPrphandle->redSupFrame = RedSupFrameAllocate(hsrPrphandle);

    if(&(hsrPrphandle->redSupFrame) == NULL)
    {
        goto exit;
    }

    /*Setup clock for Neighbor timeout check on port 0*/

    ClockP_Params_init(&clockParams);
    clockParams.period = ClockP_usecToTicks(RED_PRU_CHECK_TIMER_PERIOD);
    clockParams.start = 0;  /*Manual start*/
    clockParams.args = hsrPrphandle;
    clockParams.callback = (ClockP_FxnCallback)RedPruCheckTimerHandler;
    clockParams.timeout = ClockP_usecToTicks(RED_PRU_CHECK_TIMER_PERIOD);


    status_redPruCheckTimer = ClockP_construct(&(hsrPrphandle->redPruCheckTimer),
                                      &clockParams);


    if(status_redPruCheckTimer != SystemP_SUCCESS)
    {
        goto exit;
    }

    ClockP_start(&(hsrPrphandle->redPruCheckTimer));

    status_redLifeCheckSemaphore = SemaphoreP_constructBinary(&(hsrPrphandle->redLifeCheckSemaphore), 0);

    if(status_redLifeCheckSemaphore != SystemP_SUCCESS)
    {
        goto exit;
    }

    status_nodesTableSemaphore = SemaphoreP_constructBinary(&(hsrPrphandle->nodesTableSemaphore), 1);

    if(status_nodesTableSemaphore != SystemP_SUCCESS)
    {
        goto exit;
    }

    ClockP_Params_init(&clockParams);
    clockParams.period = ClockP_usecToTicks(RED_LIFE_CHECK_TIMER_PERIOD);
    clockParams.start = 0;  /*Manual start*/
    clockParams.callback = (ClockP_FxnCallback)RedLifeCheckTimerHandler;
    clockParams.args = hsrPrphandle;
    clockParams.timeout = ClockP_usecToTicks(RED_LIFE_CHECK_TIMER_PERIOD);

    status_redLifeCheckTimer = ClockP_construct(&(hsrPrphandle->redLifeCheckTimer), &clockParams);

    if(status_redLifeCheckTimer !=  SystemP_SUCCESS)
    {
        goto exit;
    }

    ClockP_start(&(hsrPrphandle->redLifeCheckTimer));

    TaskP_Params_init(&taskParams);
    taskParams.priority = RED_LIFE_CHECK_INIT_TASK_PRIO;
    taskParams.stackSize = RED_LIFE_CHECK_INIT_TASK_STACK_SIZE;
    taskParams.stack = (uint8_t *) gtaskRedSupTaskStack;
    taskParams.name = RED_LIFE_CHECK_INIT_TASK_NAME;
    taskParams.args = hsrPrphandle;
    taskParams.taskMain = (TaskP_FxnMain)RedLifeCheckTask;

    status_redSupTask = TaskP_construct(&(hsrPrphandle->redSupTask), &taskParams);

    if(status_redSupTask !=  SystemP_SUCCESS)
    {
        goto exit;
    }

    TaskP_Params_init(&taskParams);
    taskParams.priority = RED_NODE_TABLE_REFRESH_TASK_PRIO;
    taskParams.stackSize = RED_NODE_TABLE_REFRESH_TASK_STACK_SIZE;
    taskParams.stack = (uint8_t *)gtaskRedNodetableRefreshStack ;
    taskParams.name = RED_NODE_TABLE_REFRESH_TASK_NAME;
    taskParams.args = hsrPrphandle;
    taskParams.taskMain = (TaskP_FxnMain)RedNodetableRefresh;
    status_hsrPrpNodetableRefreshTask = TaskP_construct(&(hsrPrphandle->hsrPrpNodetableRefreshTask),
            &taskParams);

    if(status_hsrPrpNodetableRefreshTask !=  SystemP_SUCCESS)
    {
        goto exit;
    }


    return (RED_OK);

exit:

    if(status_redLifeCheckTimer)
    {
        ClockP_destruct(&(hsrPrphandle->redLifeCheckTimer));
    }

    if(status_redLifeCheckSemaphore)
    {
        SemaphoreP_destruct(&(hsrPrphandle->redLifeCheckSemaphore));
    }

    if(status_nodesTableSemaphore)
    {
        SemaphoreP_destruct(&(hsrPrphandle->nodesTableSemaphore));
    }

    if(status_redPruCheckTimer)
    {
        ClockP_destruct(&hsrPrphandle->redPruCheckTimer);
    }

    if(hsrPrphandle->redSupFrame)
    {
        RedFrameFree(hsrPrphandle->redSupFrame);
        hsrPrphandle->redSupFrame = NULL;
    }


    if(status_hsrPrpNodetableRefreshTask)
    {
        TaskP_destruct(&(hsrPrphandle->hsrPrpNodetableRefreshTask));
    }

    if(status_hsrPrpNodetableRefreshTask)
    {
        TaskP_destruct(&(hsrPrphandle->redSupTask));
    }


    return (RED_ERR);
}

void RedLifeCheckTaskDelete(hsrPrpHandle *hsrPrphandle)
{
        TaskP_destruct(&(hsrPrphandle->redSupTask));

        TaskP_destruct(&(hsrPrphandle->hsrPrpNodetableRefreshTask));

        ClockP_destruct(&(hsrPrphandle->redLifeCheckTimer));

        SemaphoreP_destruct(&hsrPrphandle->redLifeCheckSemaphore);

        SemaphoreP_destruct(&hsrPrphandle->nodesTableSemaphore);

        ClockP_destruct(&hsrPrphandle->redPruCheckTimer);

    if(hsrPrphandle->redSupFrame)
    {
        RedFrameFree(hsrPrphandle->redSupFrame);
        hsrPrphandle->redSupFrame = NULL;
    }
}

/**
 *  \brief Allocates the memory used by a RED_FRAME
 *
 *  \param pFrame memory to copy packet data from
 *  \param frameSize length of the packet
 *
 *  \return pointer to RED_FRAME structure
 *
 */
static RED_FRAME *RedFrameAllocate(const uint8_t *pFrame, int32_t frameSize)
{
    RED_FRAME *pRedFrame = NULL;
    uint16_t     vlanTag;
    uint16_t     redFrameMinSize;

    if(pFrame == NULL)
    {
        return (NULL);
    }

    /* Allocate the RED frame structure */
    pRedFrame = (RED_FRAME *) malloc(sizeof(RED_FRAME));

    if(!pRedFrame)
    {
        RED_DEBUG_MSG("%s: pRedFrame == NULL\n", __FUNCTION__);
        return (NULL);
    }

    /* Check for VLAN tag */
    vlanTag = OS_NetToHost16(*(uint16_t *)(pFrame + ETHER_TYPE_OFFSET));

    if(vlanTag == ETHER_TYPE_VLAN)
    {
        pRedFrame->vlanTagSize = ETHER_VLAN_PT_LEN;
        redFrameMinSize = RED_FRAME_VLAN_MIN_SIZE;
    }

    else
    {
        pRedFrame->vlanTagSize = 0;
        redFrameMinSize = RED_FRAME_MIN_SIZE;
    }

    /* Calculate new size */
    pRedFrame->bufferLen = frameSize + RED_TAG_SIZE;

    if(pRedFrame->bufferLen < redFrameMinSize)
    {
        pRedFrame->paddingSize = redFrameMinSize - pRedFrame->bufferLen;
        pRedFrame->bufferLen = redFrameMinSize;
    }

    else
    {
        pRedFrame->paddingSize = 0;
    }

    /* Allocate the buffer */
    pRedFrame->pDataBuffer = (uint8_t *) malloc(pRedFrame->bufferLen);

    if(pRedFrame->pDataBuffer == NULL)
    {
        RED_DEBUG_MSG("%s: pDataBuffer == NULL\n", __FUNCTION__);
        free(pRedFrame);
        return (NULL);
    }

    return (pRedFrame);
}

/**
 *  \brief Deallocates the memory used by a RED_FRAME
 *
 *  \param pRedFrame pointer to a RED_FRAME
 *
 *  \return None
 *
 */
static void RedFrameFree(RED_FRAME *pRedFrame)
{
    if(pRedFrame)
    {
        if(pRedFrame->pDataBuffer)
        {
            free(pRedFrame->pDataBuffer);
            pRedFrame->pDataBuffer = NULL;
        }

        free(pRedFrame);
    }
}

/**
 *  \brief Tags and enqueues redundancy frames to transmit
 *
 *  \param hsrPrphandle
 *  \param icssEmacHandle
 *  \param pRedFrame memory to copy packet data to
 *  \param pFrame memory to copy packet data from
 *  \param frameSize length of the packet
 *  \param queuePriority which queue to write data to
 *  \param portNum
 *
 *  \return RED_OK on success, RED_ERR otherwise
 *
 */
static RED_STATUS RedFrameSend(hsrPrpHandle *hsrPrphandle,
                               ICSS_EMAC_Handle icssEmacHandle,
                               RED_FRAME *pRedFrame, const uint8_t *pFrame, int32_t frameSize, int32_t queuePriority,
                               uint8_t portNum)
{
    int32_t ret1 = 0, ret2 = 0;

    if(pRedFrame == NULL)
    {
        return (RED_ERR);
    }

    if(pFrame == NULL)
    {
        return (RED_ERR);
    }

    RedFrameFill(hsrPrphandle, pRedFrame, pFrame, frameSize);

    if(portNum == ICSS_EMAC_PORT_1 || portNum == 0)
    {
        RedFrameUpdateLan(pRedFrame, RED_LAN_A_MAGIC);
        ret1 = RedTxPacketEnqueue(hsrPrphandle, icssEmacHandle, pRedFrame,
                                  ICSS_EMAC_PORT_1,
                                  queuePriority);
    }

    if(portNum == ICSS_EMAC_PORT_2 || portNum == 0)
    {
        RedFrameUpdateLan(pRedFrame, RED_LAN_B_MAGIC);
        ret2 = RedTxPacketEnqueue(hsrPrphandle, icssEmacHandle, pRedFrame,
                                  ICSS_EMAC_PORT_2,
                                  queuePriority);
    }

    if((ret1) && (ret2))
    {
        return (RED_ERR);
    }

    return (RED_OK);
}

/**
 *  \brief Start all HSR/PRP Timers
 *
 *  \param  pruicssHandle Pointer to PRU ICSS Handle, parent structure containing all switch information
 *
 *  \return RED_STATUS RED_OK on success, RED_ERR otherwise
 *
 */
RED_STATUS RedProtocolStart(hsrPrpHandle *hsrPrphandle,
                            PRUICSS_Handle      pruicssHandle)
{

    if(RedLifeCheckTaskCreate(hsrPrphandle, pruicssHandle))
    {
        return (RED_ERR);
    }

    return (RED_OK);
}

void  RedProtocolConfigure(PRUICSS_Handle      pruicssHandle)
{

    /* Configure the RX Frame Size */
    HW_WR_FIELD32((((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->miiRtCfgRegBase) +
                 CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_RX_FRMS0,
                  CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_RX_FRMS0_RX_MAX_FRM0 , RED_RX_FRAME_BYTE_CNT_MAX - 1);
    HW_WR_FIELD32((((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->miiRtCfgRegBase) +
                  CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_RX_FRMS0,
                  CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_RX_FRMS0_RX_MIN_FRM0 , RED_RX_FRAME_BYTE_CNT_MIN - 1);

    HW_WR_FIELD32((((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->miiRtCfgRegBase) +
                  CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_RX_FRMS1,
                  CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_RX_FRMS1_RX_MAX_FRM1 , RED_RX_FRAME_BYTE_CNT_MAX - 1);
    HW_WR_FIELD32((((PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs))->miiRtCfgRegBase) +
                  CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_RX_FRMS1,
                  CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_CFG_RX_FRMS1_RX_MIN_FRM1 , RED_RX_FRAME_BYTE_CNT_MIN - 1);
}


void RedProtocolStop(hsrPrpHandle *hsrPrphandle, PRUICSS_Handle      pruicssHandle)
{
    RedLifeCheckTaskDelete(hsrPrphandle);
}

