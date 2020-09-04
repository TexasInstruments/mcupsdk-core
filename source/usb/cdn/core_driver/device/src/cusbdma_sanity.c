/**********************************************************************
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
**********************************************************************
* WARNING: This file is auto-generated using api-generator utility.
*          api-generator: 13.05.b3ee589
*          Do not edit it manually.
**********************************************************************
* Layer interface for the Cadence DMA controller
**********************************************************************/

/* parasoft-begin-suppress METRICS-18-3 "Follow the Cyclomatic Complexity limit of 10, DRV-4789" */
/* parasoft-begin-suppress METRIC.CC-3 "Follow the Cyclomatic Complexity limit of 30, DRV-4417" */
/* parasoft-begin-suppress METRICS-36-3 "A function should not be called from more than 5 different functions, DRV-3823" */
/* parasoft-begin-suppress METRICS-39-3 "The value of VOCF metric for a function should not be higher than 4, DRV-4790" */
/* parasoft-begin-suppress METRICS-41-3 "Number of blocks of comments per statement, DRV-4926" */
/* parasoft-begin-suppress MISRA2012-RULE-8_7 "Functions and objects should not be defined with external linkage if they are referenced in only one translation unit, DRV-4139" */

/**
 * This file contains sanity API functions. The purpose of sanity functions
 * is to check input parameters validity. They take the same parameters as
 * original API functions and return 0 on success or CDN_EINVAL on wrong parameter
 * value(s).
 */

#include "cdn_stdtypes.h"
#include "cdn_errno.h"
#include "cusbdma_if.h"
#include "cusbdma_structs_if.h"
#include "cusbdma_sanity.h"

/**
 * Function to validate struct Config
 *
 * @param[in] obj pointer to struct to be verified
 * @returns 0 for valid
 * @returns CDN_EINVAL for invalid
 */
uint32_t CUSBDMA_ConfigSF(const CUSBDMA_Config *obj)
{
    uint32_t ret = 0;

    if (obj == NULL)
    {
        ret = CDN_EINVAL;
    }

    return ret;
}

/**
 * Function to validate struct ChannelParams
 *
 * @param[in] obj pointer to struct to be verified
 * @returns 0 for valid
 * @returns CDN_EINVAL for invalid
 */
uint32_t CUSBDMA_ChannelParamsSF(const CUSBDMA_ChannelParams *obj)
{
    uint32_t ret = 0;

    if (obj == NULL)
    {
        ret = CDN_EINVAL;
    }

    return ret;
}

/**
 * Function to validate struct DmaChannel
 *
 * @param[in] obj pointer to struct to be verified
 * @returns 0 for valid
 * @returns CDN_EINVAL for invalid
 */
uint32_t CUSBDMA_DmaChannelSF(const CUSBDMA_DmaChannel *obj)
{
    uint32_t ret = 0;

    if (obj == NULL)
    {
        ret = CDN_EINVAL;
    }
    else
    {
        if (
            (obj->status != CUSBDMA_STATUS_UNKNOW) &&
            (obj->status != CUSBDMA_STATUS_FREE) &&
            (obj->status != CUSBDMA_STATUS_STALLED) &&
            (obj->status != CUSBDMA_STATUS_BUSY) &&
            (obj->status != CUSBDMA_STATUS_ARMED)
            )
        {
            ret = CDN_EINVAL;
        }
    }

    return ret;
}

/**
 * Function to validate struct DmaController
 *
 * @param[in] obj pointer to struct to be verified
 * @returns 0 for valid
 * @returns CDN_EINVAL for invalid
 */
uint32_t CUSBDMA_DmaControllerSF(const CUSBDMA_DmaController *obj)
{
    uint32_t ret = 0;

    if (obj == NULL)
    {
        ret = CDN_EINVAL;
    }
    else
    {
        uint32_t idx_rx;

        for (idx_rx = 0; idx_rx < CUSBDMA_MAX_DMA_CHANNELS; idx_rx++)
        {
            if (CUSBDMA_DmaChannelSF(&obj->rx[idx_rx]) == CDN_EINVAL)
            {
                ret = CDN_EINVAL;
            }
        }
        uint32_t idx_tx;

        for (idx_tx = 0; idx_tx < CUSBDMA_MAX_DMA_CHANNELS; idx_tx++)
        {
            if (CUSBDMA_DmaChannelSF(&obj->tx[idx_tx]) == CDN_EINVAL)
            {
                ret = CDN_EINVAL;
            }
        }
    }

    return ret;
}

/**
 * Function to validate struct DmaTransferParam
 *
 * @param[in] obj pointer to struct to be verified
 * @returns 0 for valid
 * @returns CDN_EINVAL for invalid
 */
uint32_t CUSBDMA_DmaTransferParamSF(const CUSBDMA_DmaTransferParam *obj)
{
    uint32_t ret = 0;

    if (obj == NULL)
    {
        ret = CDN_EINVAL;
    }

    return ret;
}

/**
 * A common function to check the validity of API functions with
 * following parameter types
 * @param[in] config driver/hardware configuration required
 * @param[out] sysReq sysReq returns the size of memory allocations required
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t CUSBDMA_SanityFunction1(const CUSBDMA_Config* config, const CUSBDMA_SysReq* sysReq)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (sysReq == NULL)
    {
        ret = CDN_EINVAL;
    }
    else if (CUSBDMA_ConfigSF(config) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else
    {
        /*
         * All 'if ... else if' constructs shall be terminated with an 'else' statement
         * (MISRA2012-RULE-15_7-3)
         */
    }

    return ret;
}

/**
 * A common function to check the validity of API functions with
 * following parameter types
 * @param[in] pD driver state info specific to this instance
 * @param[in] config specifies driver/hardware configuration
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t CUSBDMA_SanityFunction2(const CUSBDMA_DmaController* pD, const CUSBDMA_Config* config)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (CUSBDMA_DmaControllerSF(pD) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else if (CUSBDMA_ConfigSF(config) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else
    {
        /*
         * All 'if ... else if' constructs shall be terminated with an 'else' statement
         * (MISRA2012-RULE-15_7-3)
         */
    }

    return ret;
}

/**
 * A common function to check the validity of API functions with
 * following parameter types
 * @param[in] pD driver state info specific to this instance
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t CUSBDMA_SanityFunction3(const CUSBDMA_DmaController* pD)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (CUSBDMA_DmaControllerSF(pD) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }

    return ret;
}

/**
 * A common function to check the validity of API functions with
 * following parameter types
 * @param[in] pD driver state info specific to this instance
 * @param[out] channelPtr address of channel pointer
 * @param[in] channelParams Channel parameters
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t CUSBDMA_SanityFunction4(const CUSBDMA_DmaController* pD, const CUSBDMA_DmaChannel** channelPtr, const CUSBDMA_ChannelParams* channelParams)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (channelPtr == NULL)
    {
        ret = CDN_EINVAL;
    }
    else if (CUSBDMA_DmaControllerSF(pD) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else if (CUSBDMA_ChannelParamsSF(channelParams) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else
    {
        /*
         * All 'if ... else if' constructs shall be terminated with an 'else' statement
         * (MISRA2012-RULE-15_7-3)
         */
    }

    return ret;
}

/**
 * A common function to check the validity of API functions with
 * following parameter types
 * @param[in] pD driver state info specific to this instance
 * @param[in] channel channel pointer to DMA channel
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t CUSBDMA_SanityFunction5(const CUSBDMA_DmaController* pD, const CUSBDMA_DmaChannel* channel)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (CUSBDMA_DmaControllerSF(pD) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else if (CUSBDMA_DmaChannelSF(channel) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else
    {
        /*
         * All 'if ... else if' constructs shall be terminated with an 'else' statement
         * (MISRA2012-RULE-15_7-3)
         */
    }

    return ret;
}

/**
 * A common function to check the validity of API functions with
 * following parameter types
 * @param[in] pD driver state info specific to this instance
 * @param[in] channel channel pointer to DMA channel for which transfer will be started
 * @param[in] params transfer parameters container
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t CUSBDMA_SanityFunction7(const CUSBDMA_DmaController* pD, const CUSBDMA_DmaChannel* channel, const CUSBDMA_DmaTransferParam* params)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (CUSBDMA_DmaControllerSF(pD) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else if (CUSBDMA_DmaChannelSF(channel) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else if (CUSBDMA_DmaTransferParamSF(params) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else
    {
        /*
         * All 'if ... else if' constructs shall be terminated with an 'else' statement
         * (MISRA2012-RULE-15_7-3)
         */
    }

    return ret;
}

/* parasoft-end-suppress MISRA2012-RULE-8_7 */
/* parasoft-end-suppress METRICS-41-3 */
/* parasoft-end-suppress METRICS-39-3 */
/* parasoft-end-suppress METRICS-36-3 */
/* parasoft-end-suppress METRIC.CC-3 */
/* parasoft-end-suppress METRICS-18-3 */
