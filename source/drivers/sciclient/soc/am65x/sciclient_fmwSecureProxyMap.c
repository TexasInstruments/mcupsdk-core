/*
 *  Copyright (C) 2024 Texas Instruments Incorporated
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
 *
 */

/**
 *  \file sciclient_fmwSecureProxyMap.c
 *
 *  \brief File containing the secure proxy map for all hosts.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <drivers/sciclient.h>
#include <drivers/sciclient/sciclient_priv.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

const Sciclient_MapStruct_t gSciclientMap[SCICLIENT_CONTEXT_MAX_NUM] =
{
    {
        /** Context **/
        SCICLIENT_NON_SECURE_CONTEXT,

        /** CPU ID of the A53/A72/R5F/DSP */
        TISCI_HOST_ID_R5_0,

        /** Thread ID of the low priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_R5_0_LOW_PRIORITY_WRITE_THREAD_ID,

        /** Thread ID of the response thread(read) available for the CPU */
        TISCI_SEC_PROXY_R5_0_RESPONSE_READ_THREAD_ID,

        /** Notification Interrupt Number */
        CSL_MCU0_INTR_NAVSS0_R5_0_PEND_1,

        /** Thread ID of the high priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_R5_0_HIGH_PRIORITY_WRITE_THREAD_ID

    },

    {
        /** Context **/
        SCICLIENT_SECURE_CONTEXT,

        /** CPU ID of the A53/A72/R5F/DSP */
        TISCI_HOST_ID_R5_1,

        /** Thread ID of the low priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_R5_1_LOW_PRIORITY_WRITE_THREAD_ID,

        /** Thread ID of the response thread(read) available for the CPU */
        TISCI_SEC_PROXY_R5_1_RESPONSE_READ_THREAD_ID,

        /** Notification Interrupt Number */
        CSL_MCU0_INTR_NAVSS0_R5_0_PEND_3,

        /** Thread ID of the high priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_R5_1_HIGH_PRIORITY_WRITE_THREAD_ID

    },
    {
        /** Context **/
        SCICLIENT_NON_SECURE_CONTEXT,

        /** CPU ID of the A53/A72/R5F/DSP */
        TISCI_HOST_ID_R5_2,

        /** Thread ID of the low priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_R5_2_LOW_PRIORITY_WRITE_THREAD_ID,

        /** Thread ID of the response thread(read) available for the CPU */
        TISCI_SEC_PROXY_R5_2_RESPONSE_READ_THREAD_ID,

        /** Notification Interrupt Number */
        CSL_MCU0_INTR_NAVSS0_R5_0_PEND_1,

        /** Thread ID of the high priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_R5_2_HIGH_PRIORITY_WRITE_THREAD_ID

    },

    {
        /** Context **/
        SCICLIENT_SECURE_CONTEXT,

        /** CPU ID of the A53/A72/R5F/DSP */
        TISCI_HOST_ID_R5_3,

        /** Thread ID of the low priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_R5_3_LOW_PRIORITY_WRITE_THREAD_ID,

        /** Thread ID of the response thread(read) available for the CPU */
        TISCI_SEC_PROXY_R5_3_RESPONSE_READ_THREAD_ID,

        /** Notification Interrupt Number */
        CSL_MCU0_INTR_NAVSS0_R5_0_PEND_3,

        /** Thread ID of the high priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_R5_3_HIGH_PRIORITY_WRITE_THREAD_ID

    },

    {
        /** Context **/
        SCICLIENT_SECURE_CONTEXT,

        /** CPU ID of the A53/A72/R5F/DSP */
        TISCI_HOST_ID_A53_0,

        /** Thread ID of the low priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_A53_0_LOW_PRIORITY_WRITE_THREAD_ID,

        /** Thread ID of the response thread(read) available for the CPU */
        TISCI_SEC_PROXY_A53_0_RESPONSE_READ_THREAD_ID,

        /** Notification Interrupt Number */
        CSL_GIC0_INTR_NAVSS0_BUS_A53_PEND_1,

        /** Thread ID of the high priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_A53_0_HIGH_PRIORITY_WRITE_THREAD_ID

    },

    {
        /** Context **/
        SCICLIENT_SECURE_CONTEXT,

        /** CPU ID of the A53/A72/R5F/DSP */
        TISCI_HOST_ID_A53_1,

        /** Thread ID of the low priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_A53_1_LOW_PRIORITY_WRITE_THREAD_ID,

        /** Thread ID of the response thread(read) available for the CPU */
        TISCI_SEC_PROXY_A53_1_RESPONSE_READ_THREAD_ID,

        /** Notification Interrupt Number */
        CSL_GIC0_INTR_NAVSS0_BUS_A53_PEND_3,

        /** Thread ID of the high priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_A53_1_HIGH_PRIORITY_WRITE_THREAD_ID

    },

    {
        /** Context **/
        SCICLIENT_NON_SECURE_CONTEXT,

        /** CPU ID of the A53/A72/R5F/DSP */
        TISCI_HOST_ID_A53_2,

        /** Thread ID of the low priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_A53_2_LOW_PRIORITY_WRITE_THREAD_ID,

        /** Thread ID of the response thread(read) available for the CPU */
        TISCI_SEC_PROXY_A53_2_RESPONSE_READ_THREAD_ID,

        /** Notification Interrupt Number */
        CSL_GIC0_INTR_NAVSS0_BUS_A53_PEND_5,

        /** Thread ID of the high priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_A53_2_HIGH_PRIORITY_WRITE_THREAD_ID

    },

    {
        /** Context **/
        SCICLIENT_NON_SECURE_CONTEXT,

        /** CPU ID of the A53/A72/R5F/DSP */
        TISCI_HOST_ID_A53_3,

        /** Thread ID of the low priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_A53_3_LOW_PRIORITY_WRITE_THREAD_ID,

        /** Thread ID of the response thread(read) available for the CPU */
        TISCI_SEC_PROXY_A53_3_RESPONSE_READ_THREAD_ID,

        /** Notification Interrupt Number */
        CSL_GIC0_INTR_NAVSS0_BUS_A53_PEND_7,

        /** Thread ID of the high priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_A53_3_HIGH_PRIORITY_WRITE_THREAD_ID

    },

    {
        /** Context **/
        SCICLIENT_NON_SECURE_CONTEXT,

        /** CPU ID of the A53/A72/R5F/DSP */
        TISCI_HOST_ID_A53_4,

        /** Thread ID of the low priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_A53_4_LOW_PRIORITY_WRITE_THREAD_ID,

        /** Thread ID of the response thread(read) available for the CPU */
        TISCI_SEC_PROXY_A53_4_RESPONSE_READ_THREAD_ID,

        /** Notification Interrupt Number */
        CSL_GIC0_INTR_NAVSS0_BUS_A53_PEND_9,

        /** Thread ID of the high priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_A53_4_HIGH_PRIORITY_WRITE_THREAD_ID

    },

    {
        /** Context **/
        SCICLIENT_NON_SECURE_CONTEXT,

        /** CPU ID of the A53/A72/R5F/DSP */
        TISCI_HOST_ID_A53_5,

        /** Thread ID of the low priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_A53_5_LOW_PRIORITY_WRITE_THREAD_ID,

        /** Thread ID of the response thread(read) available for the CPU */
        TISCI_SEC_PROXY_A53_5_RESPONSE_READ_THREAD_ID,

        /** Notification Interrupt Number */
        CSL_GIC0_INTR_NAVSS0_BUS_A53_PEND_11,

        /** Thread ID of the high priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_A53_5_HIGH_PRIORITY_WRITE_THREAD_ID

    },

    {
        /** Context **/
        SCICLIENT_NON_SECURE_CONTEXT,

        /** CPU ID of the A53/A72/R5F/DSP */
        TISCI_HOST_ID_A53_6,

        /** Thread ID of the low priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_A53_6_LOW_PRIORITY_WRITE_THREAD_ID,

        /** Thread ID of the response thread(read) available for the CPU */
        TISCI_SEC_PROXY_A53_6_RESPONSE_READ_THREAD_ID,

        /** Notification Interrupt Number */
        CSL_GIC0_INTR_NAVSS0_BUS_A53_PEND_13,

        /** Thread ID of the high priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_A53_6_HIGH_PRIORITY_WRITE_THREAD_ID

    },

    {
        /** Context **/
        SCICLIENT_NON_SECURE_CONTEXT,

        /** CPU ID of the A53/A72/R5F/DSP */
        TISCI_HOST_ID_A53_7,

        /** Thread ID of the low priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_A53_7_LOW_PRIORITY_WRITE_THREAD_ID,

        /** Thread ID of the response thread(read) available for the CPU */
        TISCI_SEC_PROXY_A53_7_RESPONSE_READ_THREAD_ID,

        /** Notification Interrupt Number */
        CSL_GIC0_INTR_NAVSS0_BUS_A53_PEND_15,

        /** Thread ID of the high priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_A53_7_HIGH_PRIORITY_WRITE_THREAD_ID

    },

    {
        /** Context **/
        SCICLIENT_NON_SECURE_CONTEXT,

        /** CPU ID of the A53/A72/R5F/DSP */
        TISCI_HOST_ID_GPU_0,

        /** Thread ID of the low priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_GPU_0_LOW_PRIORITY_WRITE_THREAD_ID,

        /** Thread ID of the response thread(read) available for the CPU */
        TISCI_SEC_PROXY_GPU_0_RESPONSE_READ_THREAD_ID,

        /** Notification Interrupt Number.
        *    TODO: Interrupt usage from DMSC is still not clear. */
        0U,

        /** Thread ID of the high priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_GPU_0_HIGH_PRIORITY_WRITE_THREAD_ID

    },
    {
        /** Context **/
        SCICLIENT_NON_SECURE_CONTEXT,

        /** CPU ID of the A53/A72/R5F/DSP */
        TISCI_HOST_ID_GPU_1,

        /** Thread ID of the low priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_GPU_1_LOW_PRIORITY_WRITE_THREAD_ID,

        /** Thread ID of the response thread(read) available for the CPU */
        TISCI_SEC_PROXY_GPU_1_RESPONSE_READ_THREAD_ID,

        /** Notification Interrupt Number.
        *    TODO: Interrupt usage from DMSC is still not clear. */
        0U,

        /** Thread ID of the high priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_GPU_1_HIGH_PRIORITY_WRITE_THREAD_ID

    },
    {
        /** Context **/
        SCICLIENT_NON_SECURE_CONTEXT,

        /** CPU ID of the A53/A72/R5F/DSP */
        TISCI_HOST_ID_ICSSG_0,

        /** Thread ID of the low priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_ICSSG_0_LOW_PRIORITY_WRITE_THREAD_ID,

        /** Thread ID of the response thread(read) available for the CPU */
        TISCI_SEC_PROXY_ICSSG_0_RESPONSE_READ_THREAD_ID,

        /** Notification Interrupt Number.
        *    TODO: Interrupt usage from DMSC is still not clear. */
        0U,

        /** Thread ID of the high priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_ICSSG_0_HIGH_PRIORITY_WRITE_THREAD_ID

    },
    {
        /** Context **/
        SCICLIENT_NON_SECURE_CONTEXT,

        /** CPU ID of the A53/A72/R5F/DSP */
        TISCI_HOST_ID_ICSSG_1,

        /** Thread ID of the low priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_ICSSG_1_LOW_PRIORITY_WRITE_THREAD_ID,

        /** Thread ID of the response thread(read) available for the CPU */
        TISCI_SEC_PROXY_ICSSG_1_RESPONSE_READ_THREAD_ID,

        /** Notification Interrupt Number.
        *    TODO: Interrupt usage from DMSC is still not clear. */
        0U,

        /** Thread ID of the high priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_ICSSG_1_HIGH_PRIORITY_WRITE_THREAD_ID

    },
    {
        /** Context **/
        SCICLIENT_NON_SECURE_CONTEXT,

        /** CPU ID of the A53/A72/R5F/DSP */
        TISCI_HOST_ID_ICSSG_2,

        /** Thread ID of the low priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_ICSSG_2_LOW_PRIORITY_WRITE_THREAD_ID,

        /** Thread ID of the response thread(read) available for the CPU */
        TISCI_SEC_PROXY_ICSSG_2_RESPONSE_READ_THREAD_ID,

        /** Notification Interrupt Number.
        *    TODO: Interrupt usage from DMSC is still not clear. */
        0U,

        /** Thread ID of the high priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_ICSSG_2_HIGH_PRIORITY_WRITE_THREAD_ID

    }
};

int32_t Sciclient_getTxThreadId(uint32_t contextId)
{
    uint32_t  txThread;

    txThread = gSciclientMap[contextId].reqHighPrioThreadId;
    return txThread;
}

int32_t Sciclient_getRxThreadId(uint32_t contextId)
{
    uint32_t  rxThread;

    rxThread = gSciclientMap[contextId].respThreadId;
    return rxThread;
}