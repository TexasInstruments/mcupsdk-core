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
* Layer interface for the Cadence USB device controller family
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
#include "cusbd_if.h"
#include "cusbd_structs_if.h"
#include "cusbdma_if.h"
#include "cusbdma_structs_if.h"
#include "cusbdma_sanity.h"
#include "cusbd_sanity.h"

/**
 * Function to validate struct Config
 *
 * @param[in] obj pointer to struct to be verified
 * @returns 0 for valid
 * @returns CDN_EINVAL for invalid
 */
uint32_t CUSBD_ConfigSF(const CUSBD_Config *obj)
{
    uint32_t ret = 0;

    if (obj == NULL)
    {
        ret = CDN_EINVAL;
    }
    else
    {
        if (
            (obj->dmaInterfaceWidth != CUSBD_DMA_64_WIDTH)
            )
        {
            ret = CDN_EINVAL;
        }
    }

    return ret;
}

/**
 * Function to validate struct Req
 *
 * @param[in] obj pointer to struct to be verified
 * @returns 0 for valid
 * @returns CDN_EINVAL for invalid
 */
uint32_t CUSBD_ReqSF(const CUSBD_Req *obj)
{
    uint32_t ret = 0;

    if (obj == NULL)
    {
        ret = CDN_EINVAL;
    }

    return ret;
}

/**
 * Function to validate struct Ep
 *
 * @param[in] obj pointer to struct to be verified
 * @returns 0 for valid
 * @returns CDN_EINVAL for invalid
 */
uint32_t CUSBD_EpSF(const CUSBD_Ep *obj)
{
    uint32_t ret = 0;

    if (obj == NULL)
    {
        ret = CDN_EINVAL;
    }

    return ret;
}

/**
 * Function to validate struct Callbacks
 *
 * @param[in] obj pointer to struct to be verified
 * @returns 0 for valid
 * @returns CDN_EINVAL for invalid
 */
uint32_t CUSBD_CallbacksSF(const CUSBD_Callbacks *obj)
{
    uint32_t ret = 0;

    if (obj == NULL)
    {
        ret = CDN_EINVAL;
    }

    return ret;
}

/**
 * Function to validate struct PrivateData
 *
 * @param[in] obj pointer to struct to be verified
 * @returns 0 for valid
 * @returns CDN_EINVAL for invalid
 */
uint32_t CUSBD_PrivateDataSF(const CUSBD_PrivateData *obj)
{
    uint32_t ret = 0;

    if (obj == NULL)
    {
        ret = CDN_EINVAL;
    }
    else
    {
        if (CUSBD_DevSF(&obj->device) == CDN_EINVAL)
        {
            ret = CDN_EINVAL;
        }
        if (CUSBD_ConfigSF(&obj->config) == CDN_EINVAL)
        {
            ret = CDN_EINVAL;
        }
        if (CUSBD_EpPrivateSF(&obj->ep0) == CDN_EINVAL)
        {
            ret = CDN_EINVAL;
        }
        if (
            (obj->ep0NextState != CH9_EP0_UNCONNECTED) &&
            (obj->ep0NextState != CH9_EP0_SETUP_PHASE) &&
            (obj->ep0NextState != CH9_EP0_DATA_PHASE) &&
            (obj->ep0NextState != CH9_EP0_STATUS_PHASE)
            )
        {
            ret = CDN_EINVAL;
        }
        uint32_t idx_ep_in_container;

        for (idx_ep_in_container = 0; idx_ep_in_container < 16U; idx_ep_in_container++)
        {
            if (CUSBD_EpPrivateSF(&obj->ep_in_container[idx_ep_in_container]) == CDN_EINVAL)
            {
                ret = CDN_EINVAL;
            }
        }
        uint32_t idx_ep_out_container;

        for (idx_ep_out_container = 0; idx_ep_out_container < 16U; idx_ep_out_container++)
        {
            if (CUSBD_EpPrivateSF(&obj->ep_out_container[idx_ep_out_container]) == CDN_EINVAL)
            {
                ret = CDN_EINVAL;
            }
        }
    }

    return ret;
}

/**
 * Function to validate struct Dev
 *
 * @param[in] obj pointer to struct to be verified
 * @returns 0 for valid
 * @returns CDN_EINVAL for invalid
 */
uint32_t CUSBD_DevSF(const CUSBD_Dev *obj)
{
    uint32_t ret = 0;

    if (obj == NULL)
    {
        ret = CDN_EINVAL;
    }
    else
    {
        if (
            (obj->speed != CH9_USB_SPEED_UNKNOWN) &&
            (obj->speed != CH9_USB_SPEED_LOW) &&
            (obj->speed != CH9_USB_SPEED_FULL) &&
            (obj->speed != CH9_USB_SPEED_HIGH) &&
            (obj->speed != CH9_USB_SPEED_SUPER) &&
            (obj->speed != CH9_USB_SPEED_SUPER_PLUS)
            )
        {
            ret = CDN_EINVAL;
        }
        if (
            (obj->maxSpeed != CH9_USB_SPEED_UNKNOWN) &&
            (obj->maxSpeed != CH9_USB_SPEED_LOW) &&
            (obj->maxSpeed != CH9_USB_SPEED_FULL) &&
            (obj->maxSpeed != CH9_USB_SPEED_HIGH) &&
            (obj->maxSpeed != CH9_USB_SPEED_SUPER) &&
            (obj->maxSpeed != CH9_USB_SPEED_SUPER_PLUS)
            )
        {
            ret = CDN_EINVAL;
        }
        if (
            (obj->state != CH9_USB_STATE_NONE) &&
            (obj->state != CH9_USB_STATE_ATTACHED) &&
            (obj->state != CH9_USB_STATE_POWERED) &&
            (obj->state != CH9_USB_STATE_DEFAULT) &&
            (obj->state != CH9_USB_STATE_ADDRESS) &&
            (obj->state != CH9_USB_STATE_CONFIGURED) &&
            (obj->state != CH9_USB_STATE_SUSPENDED) &&
            (obj->state != CH9_USB_STATE_ERROR)
            )
        {
            ret = CDN_EINVAL;
        }
    }

    return ret;
}

/**
 * Function to validate struct EpPrivate
 *
 * @param[in] obj pointer to struct to be verified
 * @returns 0 for valid
 * @returns CDN_EINVAL for invalid
 */
uint32_t CUSBD_EpPrivateSF(const CUSBD_EpPrivate *obj)
{
    uint32_t ret = 0;

    if (obj == NULL)
    {
        ret = CDN_EINVAL;
    }
    else
    {
        if (
            (obj->ep_state != CUSBD_EP_DISABLED) &&
            (obj->ep_state != CUSBD_EP_ENABLED) &&
            (obj->ep_state != CUSBD_EP_STALLED)
            )
        {
            ret = CDN_EINVAL;
        }
    }

    return ret;
}

/**
 * A common function to check the validity of API functions with
 * following parameter types
 * @param[in] config driver/hardware configuration required
 * @param[out] sysReqCusbd returns the size of memory allocations required
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t CUSBD_SanityFunction1(const CUSBD_Config* config, const CUSBD_SysReq* sysReqCusbd)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (sysReqCusbd == NULL)
    {
        ret = CDN_EINVAL;
    }
    else if (CUSBD_ConfigSF(config) == CDN_EINVAL)
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
 * @param[out] pD driver state info specific to this instance
 * @param[in,out] config specifies driver/hardware configuration
 * @param[in] callbacks client-supplied callback functions
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t CUSBD_SanityFunction2(const CUSBD_PrivateData* pD, const CUSBD_Config* config, const CUSBD_Callbacks* callbacks)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (pD == NULL)
    {
        ret = CDN_EINVAL;
    }
    else if (config == NULL)
    {
        ret = CDN_EINVAL;
    }
    else if (CUSBD_CallbacksSF(callbacks) == CDN_EINVAL)
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
uint32_t CUSBD_SanityFunction3(const CUSBD_PrivateData* pD)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (CUSBD_PrivateDataSF(pD) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }

    return ret;
}

/**
 * A common function to check the validity of API functions with
 * following parameter types
 * @param[in] pD driver state info specific to this instance
 * @param[in] ep endpoint being configured
 * @param[in] desc endpoint descriptor
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t CUSBD_SanityFunction7(const CUSBD_PrivateData* pD, const CUSBD_Ep* ep, const uint8_t* desc)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (desc == NULL)
    {
        ret = CDN_EINVAL;
    }
    else if (CUSBD_PrivateDataSF(pD) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else if (CUSBD_EpSF(ep) == CDN_EINVAL)
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
 * @param[in] ep endpoint being unconfigured
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t CUSBD_SanityFunction8(const CUSBD_PrivateData* pD, const CUSBD_Ep* ep)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (CUSBD_PrivateDataSF(pD) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else if (CUSBD_EpSF(ep) == CDN_EINVAL)
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
 * @param[in] ep endpoint associated with the request
 * @param[in] req request being submitted
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t CUSBD_SanityFunction13(const CUSBD_PrivateData* pD, const CUSBD_Ep* ep, const CUSBD_Req* req)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (CUSBD_PrivateDataSF(pD) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else if (CUSBD_EpSF(ep) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else if (CUSBD_ReqSF(req) == CDN_EINVAL)
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
 * @param[out] dev returns pointer to CUSBD instance
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t CUSBD_SanityFunction15(const CUSBD_PrivateData* pD, const CUSBD_Dev** dev)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (dev == NULL)
    {
        ret = CDN_EINVAL;
    }
    else if (CUSBD_PrivateDataSF(pD) == CDN_EINVAL)
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
 * @param[out] numOfFrame returns number of USB frame
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t CUSBD_SanityFunction16(const CUSBD_PrivateData* pD, const uint32_t* numOfFrame)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (numOfFrame == NULL)
    {
        ret = CDN_EINVAL;
    }
    else if (CUSBD_PrivateDataSF(pD) == CDN_EINVAL)
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
 * @param[out] configParams pointer to CH9_ConfigParams structure
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t CUSBD_SanityFunction19(const CUSBD_PrivateData* pD, const CH9_ConfigParams* configParams)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (configParams == NULL)
    {
        ret = CDN_EINVAL;
    }
    else if (CUSBD_PrivateDataSF(pD) == CDN_EINVAL)
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
