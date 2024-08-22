/*
 * Copyright (C) 2024 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


/**
 *   @file  canfd_soc.c
 *
 *   @brief
 *      The file implements the Controller Area Network Driver Flexible data.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>
#include <string.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/CacheP.h>
#include <kernel/dpl/SystemP.h>
#include <drivers/mcan/v0/mcan.h>
#include <drivers/mcan/v0/canfd.h>

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t CANFD_writeDma(CANFD_MsgObjHandle handle, uint32_t id, CANFD_MCANFrameType frameType, uint32_t numMsgs, const void* data)
{
    CANFD_MessageObject*    ptrCanMsgObj;
    CANFD_Object*           ptrCanFdObj;
    int32_t                 retVal = SystemP_FAILURE;
    uint32_t                baseAddr = 0U;
    MCAN_TxBufElement       txBuffElem;
    uint32_t                index;
    //uintptr_t               key;
    uint8_t                 padSize = 0U;

    if(NULL != handle)
    {
        /* Get the message object pointer */
        ptrCanMsgObj = (CANFD_MessageObject*)handle;
        /* Get the pointer to the CAN Driver Block */
        ptrCanFdObj = (CANFD_Object*)ptrCanMsgObj->canfdHandle->object;
        baseAddr = ptrCanFdObj->regBaseAddress;
        retVal = CANFD_isDataSizeValid(ptrCanMsgObj->dataLength);

        /* Check for pending messages */
        index = (uint32_t)1U << ptrCanMsgObj->txElement;
        if ((index == (MCAN_getTxBufReqPend(baseAddr) & index)) || (retVal != SystemP_SUCCESS))
        {
            retVal = SystemP_FAILURE;
        }
        else
        {
            /* populate the Tx buffer message element */
            txBuffElem.rtr = (uint32_t)0;
            txBuffElem.esi = (uint32_t)0;
            txBuffElem.efc = (uint32_t)0;
            txBuffElem.mm = (uint32_t)0;

            if(frameType == CANFD_MCANFrameType_CLASSIC)
            {
                txBuffElem.brs = (uint32_t)0;
                txBuffElem.fdf = (uint32_t)0;
            }
            else
            {
                txBuffElem.brs = (uint32_t)1U;
                txBuffElem.fdf = (uint32_t)1U;
            }
            /* Populate the Id */
            if (ptrCanMsgObj->msgIdType == CANFD_MCANXidType_11_BIT)
            {
                txBuffElem.xtd = CANFD_MCANXidType_11_BIT;
                txBuffElem.id = (id & STD_MSGID_MASK) << STD_MSGID_SHIFT;
            }
            else
            {
                txBuffElem.xtd = CANFD_MCANXidType_29_BIT;
                txBuffElem.id = id & XTD_MSGID_MASK;
            }

            /* Copy the data of first message */
            (void)memcpy ((void*)&txBuffElem.data, data, ptrCanMsgObj->dataLength);

            /* Compute the DLC value */
            for(index = (uint32_t)0U ; index < 16U ; index++)
            {
                if((uint8_t)ptrCanMsgObj->dataLength <= ptrCanFdObj->mcanDataSize[index])
                {
                    txBuffElem.dlc = index;
                    padSize = (uint8_t)(ptrCanFdObj->mcanDataSize[index] - (uint8_t)ptrCanMsgObj->dataLength);
                    break;
                }
            }
            txBuffElem.dlc = index;
            if (index == 16U)
            {
                retVal = MCAN_INVALID_PARAM;
            }
            else
            {
                /* Pad the unused data in payload */
                index = ptrCanMsgObj->dataLength;
                while (padSize != (uint8_t)0U)
                {
                    txBuffElem.data[index] = (uint8_t)0xCCU;
                    index++;
                    padSize--;
                }
                /* Copy the first msg in msg ram. Subsequent msgs are written by the dma. */
                MCAN_writeMsgRam(baseAddr, MCAN_MEM_TYPE_BUF, ptrCanMsgObj->txElement, &txBuffElem);

                /* Configure the dma to copy the subsequent msgs */
                retVal = CANFD_configureDmaTx(ptrCanFdObj, ptrCanMsgObj, ptrCanMsgObj->dataLength, numMsgs, data);

                /* Critical Section Protection */
                //key = HwiP_disable();

                retVal += MCAN_txBufAddReq(baseAddr, ptrCanMsgObj->txElement);

                ptrCanFdObj->txStatus[ptrCanMsgObj->txElement] = (uint8_t)1;

                /* Release the critical section: */
                //HwiP_restore(key);

                /* Increment the stats */
                ptrCanMsgObj->messageProcessed++;
            }
        }
    }
    return retVal;
}

int32_t CANFD_writeDmaTriggerNext(CANFD_MsgObjHandle handle)
{
    CANFD_MessageObject*    ptrCanMsgObj;
    CANFD_Object*           ptrCanFdObj;
    int32_t                 retVal = MCAN_STATUS_SUCCESS;
    uint32_t                baseAddr;
    uint32_t                index;

    /* Get the message object pointer */
    ptrCanMsgObj = (CANFD_MessageObject*)handle;
    if (ptrCanMsgObj == NULL)
    {
        retVal = MCAN_INVALID_PARAM;
    }
    if (retVal == MCAN_STATUS_SUCCESS)
    {
        /* Get the pointer to the CAN Driver Block */
        ptrCanFdObj = (CANFD_Object*)ptrCanMsgObj->canfdHandle->object;

        if (ptrCanFdObj == NULL)
        {
            retVal = MCAN_INVALID_PARAM;
        }
    }
    if (retVal == MCAN_STATUS_SUCCESS)
    {
        baseAddr = ptrCanFdObj->regBaseAddress;

        /* Check for pending messages */
        index = (uint32_t)1U << ptrCanMsgObj->txElement;
        if (index == (MCAN_getTxBufReqPend(baseAddr) & index))
        {
            retVal = MCAN_STATUS_BUSY;
        }
    }
    if (retVal == MCAN_STATUS_SUCCESS)
    {
        retVal = MCAN_txBufAddReq(baseAddr, ptrCanMsgObj->txElement);

        ptrCanFdObj->txStatus[ptrCanMsgObj->txElement] = (uint8_t)1;
    }
    return retVal;
}

uint32_t CANFD_getFilterEventConfig(uint32_t eventNum)
{
    uint32_t retVal = ((eventNum + 1U) & 0x7U) << 6U;
    return retVal;
}
