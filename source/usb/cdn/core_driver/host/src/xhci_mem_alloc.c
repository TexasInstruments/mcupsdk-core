/******************************************************************************
 *
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
 *
 ******************************************************************************
 * xhci_mem_alloc.c
 * USB Host driver,
 *
 * Memory allocation for USBSSP hardware controller
 *****************************************************************************/

#include "cdn_log.h"
#include "cdn_xhci_structs_if.h"
#include <string.h>
/**
 * Maximal number of driver instances used in demoTB
 */
#define MAX_INSTANCE_NUM 1U

/* Forward declaration */
uint32_t USBSSP_SetMemResCallback(USBSSP_DriverResourcesT *res);

#if defined USBSSP_DEMO_TB
/*
 * see table 54 in chapter 6 of XHCI specification for allocation requirements
 * details
 */
typedef struct  {
    /** Scratch pad buffers pool */
    uint8_t scratchpadPool[USBSSP_SCRATCHPAD_BUFF_NUM * USBSSP_PAGE_SIZE] __attribute__((aligned(USBSSP_PAGE_SIZE)));
    /** Event Ring */
    USBSSP_RingElementT eventPool[USBSSP_MAX_NUM_INTERRUPTERS][USBSSP_EVENT_QUEUE_SIZE] __attribute__((aligned(1024)));
    /** Transfer ring for all endpoints */
    USBSSP_RingElementT epRingPool[USBSSP_PRODUCER_QUEUE_SIZE * (USBSSP_MAX_EP_CONTEXT_NUM + 2U)] __attribute__((aligned(1024)));
    /** Device context base array structure */
    USBSSP_DcbaaT dcbaa __attribute__((aligned(USBSSP_DCBAA_ALIGNMENT)));
    /** Input context structure */
    USBSSP_InputContexT inputContext __attribute__((aligned(1024)));
    /** Output context structure */
    USBSSP_OutputContexT outputContext __attribute__((aligned(1024)));
    /** Scratch pad buffer array (extra element for last pointer = NULL) */
    uint64_t scratchpad[USBSSP_SCRATCHPAD_BUFF_NUM + 1U] __attribute__((aligned(64)));
    /** event ring segment entry */
    uint64_t eventRingSegmentEntry[USBSSP_MAX_NUM_INTERRUPTERS * USBSSP_ERST_ENTRY_SIZE] __attribute__((aligned(USBSSP_ERST_ALIGNMENT)));
    /** allocated memory for stream objects */
    USBSSP_ProducerQueueT streamMemoryPool[USBSSP_MAX_EP_NUM_STRM_EN][USBSSP_STREAM_ARRAY_SIZE];
    /** allocation memory for stream rings */
    USBSSP_RingElementT streamRing[USBSSP_MAX_EP_NUM_STRM_EN][USBSSP_STREAM_ARRAY_SIZE][USBSSP_PRODUCER_QUEUE_SIZE] __attribute__((aligned(1024)));
    /** Ep0 Buffer */
    uint8_t ep0Buffer[USBSSP_EP0_DATA_BUFF_SIZE] __attribute__((aligned(64)));
} XhciResT;

static void cleanMemRes(XhciResT * memRes) {
    (void) memset(&memRes->dcbaa, 0, sizeof (memRes->dcbaa));

    /** clear endpoint xfer ring memory for all EPs*/
    (void) memset(memRes->epRingPool, 0, sizeof (memRes->epRingPool));
    (void) memset(memRes->eventPool, 0, sizeof (memRes->eventPool));
    (void) memset(memRes->eventRingSegmentEntry, 0, sizeof (memRes->eventRingSegmentEntry));
    (void) memset(&memRes->inputContext, 0, sizeof (memRes->inputContext));
    (void) memset(&memRes->outputContext, 0, sizeof (memRes->outputContext));
    (void) memset(memRes->scratchpad, 0, sizeof (memRes->scratchpad));
    (void) memset(memRes->scratchpadPool, 0, sizeof (memRes->scratchpadPool));
    (void) memset(memRes->ep0Buffer, 0, sizeof (memRes->ep0Buffer));
}

/**
 * Allocate memory for XHCI interface
 * @param res
 * @return
 */
static uint32_t xhciAllocMem(USBSSP_DriverResourcesT *res)
{
    uint32_t ret = CDN_EOK;
    static XhciResT xhciResArray[MAX_INSTANCE_NUM];

    /*
     * resources are organized in array for many driver resources instances
     */
    static USBSSP_XhciResourcesT memResArray[MAX_INSTANCE_NUM];

    if ((res->instanceNo + 1U) > MAX_INSTANCE_NUM) {
        vDbgMsg(USBSSP_DBG_DRV, DBG_CRIT, "Instance incorrectly selected : %d, max number is: %d\n", res->instanceNo, MAX_INSTANCE_NUM);
        ret = CDN_ENOMEM;
    }

    if (ret == CDN_EOK) {

        uint32_t interrupterIdx = 0U;
        memResArray[res->instanceNo].dcbaa = &xhciResArray[res->instanceNo].dcbaa;
        memResArray[res->instanceNo].epRingPool = xhciResArray[res->instanceNo].epRingPool;

        for (interrupterIdx = 0; interrupterIdx < USBSSP_MAX_NUM_INTERRUPTERS; interrupterIdx++) {
            memResArray[res->instanceNo].eventPool[interrupterIdx] = xhciResArray[res->instanceNo].eventPool[interrupterIdx];
        }

        memResArray[res->instanceNo].eventRingSegmentEntry = xhciResArray[res->instanceNo].eventRingSegmentEntry;
        memResArray[res->instanceNo].inputContext = &xhciResArray[res->instanceNo].inputContext;
        memResArray[res->instanceNo].outputContext = &xhciResArray[res->instanceNo].outputContext;
        memResArray[res->instanceNo].scratchpad = xhciResArray[res->instanceNo].scratchpad;
        memResArray[res->instanceNo].scratchpadPool = xhciResArray[res->instanceNo].scratchpadPool;
        memResArray[res->instanceNo].streamMemoryPool = &xhciResArray[res->instanceNo].streamMemoryPool;
        memResArray[res->instanceNo].streamRing = &xhciResArray[res->instanceNo].streamRing;
        memResArray[res->instanceNo].ep0Buffer = xhciResArray[res->instanceNo].ep0Buffer;

        res->xhciMemRes = &memResArray[res->instanceNo];
        cleanMemRes(&xhciResArray[res->instanceNo]);
    }
    return ret;
}
#endif

/**
 * Function called from driver for setting res->xhciMemRes
 * @param res
 */
uint32_t USBSSP_SetMemResCallback(USBSSP_DriverResourcesT *res) {

    uint32_t ret = CDN_ENOMEM;
    res->xhciMemRes = NULL;

#if defined USBSSP_DEMO_TB
    ret = xhciAllocMem(res);
#endif

    return ret;
}
