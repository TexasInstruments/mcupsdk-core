/*
 *  Copyright (C) 2022 Texas Instruments Incorporated
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
 *  \file am62x/sciclient_fmwSecureProxyMap.c
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
        SCICLIENT_SECURE_CONTEXT,

        /** CPU ID of the A53/A72/R5F/DSP */
        TISCI_HOST_ID_MAIN_0_R5_0,

        /** Thread ID of the low priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_MAIN_0_R5_0_WRITE_LOW_PRIORITY_THREAD_ID,

        /** Thread ID of the response thread(read) available for the CPU */
        TISCI_SEC_PROXY_MAIN_0_R5_0_READ_RESPONSE_THREAD_ID,

        /** Notification Interrupt Number */
        0

    },

    {
        /** Context **/
        SCICLIENT_NON_SECURE_CONTEXT,

        /** CPU ID of the A53/A72/R5F/DSP */
        TISCI_HOST_ID_MAIN_0_R5_1,

        /** Thread ID of the low priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_MAIN_0_R5_1_WRITE_LOW_PRIORITY_THREAD_ID,

        /** Thread ID of the response thread(read) available for the CPU */
        TISCI_SEC_PROXY_MAIN_0_R5_1_READ_RESPONSE_THREAD_ID,

        /** Notification Interrupt Number */
        0

    },
    {
        /** Context **/
        SCICLIENT_SECURE_CONTEXT,

        /** CPU ID of the A53/A72/R5F/DSP */
        TISCI_HOST_ID_MAIN_0_R5_2,

        /** Thread ID of the low priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_MAIN_0_R5_2_WRITE_LOW_PRIORITY_THREAD_ID,

        /** Thread ID of the response thread(read) available for the CPU */
        TISCI_SEC_PROXY_MAIN_0_R5_2_READ_RESPONSE_THREAD_ID,

        /** Notification Interrupt Number */
        0

    },

    {
        /** Context **/
        SCICLIENT_NON_SECURE_CONTEXT,

        /** CPU ID of the A53/A72/R5F/DSP */
        TISCI_HOST_ID_MAIN_0_R5_3,

        /** Thread ID of the low priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_MAIN_0_R5_3_WRITE_LOW_PRIORITY_THREAD_ID,

        /** Thread ID of the response thread(read) available for the CPU */
        TISCI_SEC_PROXY_MAIN_0_R5_3_READ_RESPONSE_THREAD_ID,

        /** Notification Interrupt Number */
        0

    },

    {
        /** Context **/
        SCICLIENT_SECURE_CONTEXT,

        /** CPU ID of the A53/A72/R5F/DSP */
        TISCI_HOST_ID_A53_0,

        /** Thread ID of the low priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_A53_0_WRITE_LOW_PRIORITY_THREAD_ID,

        /** Thread ID of the response thread(read) available for the CPU */
        TISCI_SEC_PROXY_A53_0_READ_RESPONSE_THREAD_ID,

        /** Notification Interrupt Number */
        0

    },

    {
        /** Context **/
        SCICLIENT_NON_SECURE_CONTEXT,

        /** CPU ID of the A53/A72/R5F/DSP */
        TISCI_HOST_ID_A53_1,

        /** Thread ID of the low priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_A53_1_WRITE_LOW_PRIORITY_THREAD_ID,

        /** Thread ID of the response thread(read) available for the CPU */
        TISCI_SEC_PROXY_A53_1_READ_RESPONSE_THREAD_ID,

        /** Notification Interrupt Number */
        0

    },

    {
        /** Context **/
        SCICLIENT_NON_SECURE_CONTEXT,

        /** CPU ID of the A53/A72/R5F/DSP */
        TISCI_HOST_ID_A53_2,

        /** Thread ID of the low priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_A53_2_WRITE_LOW_PRIORITY_THREAD_ID,

        /** Thread ID of the response thread(read) available for the CPU */
        TISCI_SEC_PROXY_A53_2_READ_RESPONSE_THREAD_ID,

        /** Notification Interrupt Number */
        0

    },

    {
        /** Context **/
        SCICLIENT_NON_SECURE_CONTEXT,

        /** CPU ID of the A53/A72/R5F/DSP */
        TISCI_HOST_ID_A53_3,

        /** Thread ID of the low priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_A53_3_WRITE_LOW_PRIORITY_THREAD_ID,

        /** Thread ID of the response thread(read) available for the CPU */
        TISCI_SEC_PROXY_A53_3_READ_RESPONSE_THREAD_ID,

        /** Notification Interrupt Number */
        0

    },

    {
        /** Context **/
        SCICLIENT_NON_SECURE_CONTEXT,

        /** CPU ID of the A53/A72/R5F/DSP */
        TISCI_HOST_ID_M4_0,

        /** Thread ID of the low priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_M4_0_WRITE_LOW_PRIORITY_THREAD_ID,

        /** Thread ID of the response thread(read) available for the CPU */
        TISCI_SEC_PROXY_M4_0_READ_RESPONSE_THREAD_ID,

        /** Notification Interrupt Number */
        0

    },

    {
        /** Context **/
        SCICLIENT_NON_SECURE_CONTEXT,

        /** CPU ID of the A53/A72/R5F/DSP */
        TISCI_HOST_ID_GPU,

        /** Thread ID of the low priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_GPU_WRITE_LOW_PRIORITY_THREAD_ID,

        /** Thread ID of the response thread(read) available for the CPU */
        TISCI_SEC_PROXY_GPU_READ_RESPONSE_THREAD_ID,

        /** Notification Interrupt Number */
        0

    },

    {
        /** Context **/
        SCICLIENT_NON_SECURE_CONTEXT,

        /** CPU ID of the A53/A72/R5F/DSP */
        TISCI_HOST_ID_ICSSG_0,

        /** Thread ID of the low priority thread(write) allowed for the CPU */
        TISCI_SEC_PROXY_ICSSG_0_WRITE_LOW_PRIORITY_THREAD_ID,

        /** Thread ID of the response thread(read) available for the CPU */
        TISCI_SEC_PROXY_ICSSG_0_READ_RESPONSE_THREAD_ID,

        /** Notification Interrupt Number.**/
        0U

    }
};

