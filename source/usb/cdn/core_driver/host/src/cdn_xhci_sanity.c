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
* XHCI driver for both host and device mode header file
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
#include "cdn_xhci_if.h"
#include "cdn_xhci_structs_if.h"
#include "cusb_ch9_sanity.h"
#include "cdn_xhci_sanity.h"

/**
 * Function to validate struct XhciResourcesT
 *
 * @param[in] obj pointer to struct to be verified
 * @returns 0 for valid
 * @returns CDN_EINVAL for invalid
 */
uint32_t USBSSP_XhciResourcesTSF(const USBSSP_XhciResourcesT *obj)
{
    uint32_t ret = 0;

    if (obj == NULL)
    {
        ret = CDN_EINVAL;
    }

    return ret;
}

/**
 * Function to validate struct DriverResourcesT
 *
 * @param[in] obj pointer to struct to be verified
 * @returns 0 for valid
 * @returns CDN_EINVAL for invalid
 */
uint32_t USBSSP_DriverResourcesTSF(const USBSSP_DriverResourcesT *obj)
{
    uint32_t ret = 0;

    if (obj == NULL)
    {
        ret = CDN_EINVAL;
    }
    else
    {
        if (
            (obj->actualSpeed != CH9_USB_SPEED_UNKNOWN) &&
            (obj->actualSpeed != CH9_USB_SPEED_LOW) &&
            (obj->actualSpeed != CH9_USB_SPEED_FULL) &&
            (obj->actualSpeed != CH9_USB_SPEED_HIGH) &&
            (obj->actualSpeed != CH9_USB_SPEED_SUPER) &&
            (obj->actualSpeed != CH9_USB_SPEED_SUPER_PLUS)
            )
        {
            ret = CDN_EINVAL;
        }
        if (
            (obj->ep0State != USBSSP_EP0_UNCONNECTED) &&
            (obj->ep0State != USBSSP_EP0_SETUP_PHASE) &&
            (obj->ep0State != USBSSP_EP0_DATA_PHASE) &&
            (obj->ep0State != USBSSP_EP0_STATUS_PHASE)
            )
        {
            ret = CDN_EINVAL;
        }
    }

    return ret;
}

/**
 * Function to validate struct DriverContextT
 *
 * @param[in] obj pointer to struct to be verified
 * @returns 0 for valid
 * @returns CDN_EINVAL for invalid
 */
uint32_t USBSSP_DriverContextTSF(const USBSSP_DriverContextT *obj)
{
    uint32_t ret = 0;

    if (obj == NULL)
    {
        ret = CDN_EINVAL;
    }

    return ret;
}

/**
 * Function to validate struct ForceHdrParams
 *
 * @param[in] obj pointer to struct to be verified
 * @returns 0 for valid
 * @returns CDN_EINVAL for invalid
 */
uint32_t USBSSP_ForceHdrParamsSF(const USBSSP_ForceHdrParams *obj)
{
    uint32_t ret = 0;

    if (obj == NULL)
    {
        ret = CDN_EINVAL;
    }

    return ret;
}

/**
 * Function to validate struct XferBufferDesc
 *
 * @param[in] obj pointer to struct to be verified
 * @returns 0 for valid
 * @returns CDN_EINVAL for invalid
 */
uint32_t USBSSP_XferBufferDescSF(const USBSSP_XferBufferDesc *obj)
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
 * @param[in] res Driver resources
 * @param[in] epIndex index of endpoint according to xhci spec e.g for ep1out
 *    epIndex=2, for ep1in epIndex=3, for ep2out epIndex=4 end so on
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t USBSSP_SanityFunction1(const USBSSP_DriverResourcesT* res, const uint8_t epIndex)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (USBSSP_DriverResourcesTSF(res) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else if ((epIndex < (USBSSP_EP_CONT_OFFSET)) || (epIndex > (USBSSP_EP_CONT_MAX - (uint8_t)1)))
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
 * @param[in] res Driver resources
 * @param[in] epIndex index of endpoint according to xhci spec e.g for ep1out
 *    epIndex=2, for ep1in epIndex=3, for ep2out epIndex=4 end so on
 * @param[in] bufferDesc Pointer to an array of buffer descriptors
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t USBSSP_SanityFunction2(const USBSSP_DriverResourcesT* res, const uint8_t epIndex, const USBSSP_XferBufferDesc* bufferDesc)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (USBSSP_DriverResourcesTSF(res) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else if ((epIndex < (USBSSP_EP_CONT_OFFSET)) || (epIndex > (USBSSP_EP_CONT_MAX - (uint8_t)1)))
    {
        ret = CDN_EINVAL;
    }
    else if (USBSSP_XferBufferDescSF(bufferDesc) == CDN_EINVAL)
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
 * @param[in] res Driver resources
 * @param[in] endpoint Index of endpoint to stop
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t USBSSP_SanityFunction3(const USBSSP_DriverResourcesT* res, const uint8_t endpoint)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (USBSSP_DriverResourcesTSF(res) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else if ((endpoint < (USBSSP_EP0_CONT_OFFSET)) || (endpoint > (USBSSP_EP_CONT_MAX - (uint8_t)1)))
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
 * @param[in] res Driver resources
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t USBSSP_SanityFunction5(const USBSSP_DriverResourcesT* res)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (USBSSP_DriverResourcesTSF(res) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }

    return ret;
}

/**
 * A common function to check the validity of API functions with
 * following parameter types
 * @param[in] res Driver resources
 * @param[in] memRes User defined memory resources.
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t USBSSP_SanityFunction7(const USBSSP_DriverResourcesT* res, const USBSSP_XhciResourcesT* memRes)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (USBSSP_DriverResourcesTSF(res) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else if (USBSSP_XhciResourcesTSF(memRes) == CDN_EINVAL)
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
 * @param[in] res Driver resources
 * @param[in] setup Keeps setup packet
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t USBSSP_SanityFunction15(const USBSSP_DriverResourcesT* res, const CH9_UsbSetup* setup)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (USBSSP_DriverResourcesTSF(res) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else if (CH9_UsbSetupSF(setup) == CDN_EINVAL)
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
 * @param[in] res Driver resources
 * @param[in] desc pointer to endpoint descriptor
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t USBSSP_SanityFunction20(const USBSSP_DriverResourcesT* res, const uint8_t* desc)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (desc == NULL)
    {
        ret = CDN_EINVAL;
    }
    else if (USBSSP_DriverResourcesTSF(res) == CDN_EINVAL)
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
 * @param[in] res driver resources
 * @param[out] index Micro Frame Index returned by function.
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t USBSSP_SanityFunction22(const USBSSP_DriverResourcesT* res, const uint32_t* index)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (index == NULL)
    {
        ret = CDN_EINVAL;
    }
    else if (USBSSP_DriverResourcesTSF(res) == CDN_EINVAL)
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
 * @param[in] res driver resources
 * @param[in] flags Endpoint Extra Flag
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t USBSSP_SanityFunction23(const USBSSP_DriverResourcesT* res, const USBSSP_ExtraFlagsEnumT flags)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (USBSSP_DriverResourcesTSF(res) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else if (
        (flags != USBSSP_EXTRAFLAGSENUMT_UNDEFINED) &&
        (flags != USBSSP_EXTRAFLAGSENUMT_NODORBELL) &&
        (flags != USBSSP_EXTRAFLAGSENUMT_FORCELINKTRB)
        )
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
 * @param[in] res driver resources
 * @param[in] trbDwords trb words
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t USBSSP_SanityFunction28(const USBSSP_DriverResourcesT* res, const USBSSP_ForceHdrParams* trbDwords)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (USBSSP_DriverResourcesTSF(res) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else if (USBSSP_ForceHdrParamsSF(trbDwords) == CDN_EINVAL)
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
 * @param[in] res driver resources
 * @param[in] portRegIdx port control register ID
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t USBSSP_SanityFunction31(const USBSSP_DriverResourcesT* res, const USBSSP_PortControlRegIdx portRegIdx)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (USBSSP_DriverResourcesTSF(res) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else if (
        (portRegIdx != USBSSP_PORTSC_REG_IDX) &&
        (portRegIdx != USBSSP_PORTPMSC_REG_IDX) &&
        (portRegIdx != USBSSP_PORTLI_REG_IDX) &&
        (portRegIdx != USBSSP_PORTHLPMC_REG_IDX)
        )
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
 * @param[in] res Driver resources
 * @param[in] portRegIdx port control register ID
 * @param[out] regValue Register value
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t USBSSP_SanityFunction32(const USBSSP_DriverResourcesT* res, const USBSSP_PortControlRegIdx portRegIdx, const uint32_t* regValue)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (regValue == NULL)
    {
        ret = CDN_EINVAL;
    }
    else if (USBSSP_DriverResourcesTSF(res) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else if (
        (portRegIdx != USBSSP_PORTSC_REG_IDX) &&
        (portRegIdx != USBSSP_PORTPMSC_REG_IDX) &&
        (portRegIdx != USBSSP_PORTLI_REG_IDX) &&
        (portRegIdx != USBSSP_PORTHLPMC_REG_IDX)
        )
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
 * @param[in] res Driver resources
 * @param[in] drvContext Pointer to driver context struct
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t USBSSP_SanityFunction33(const USBSSP_DriverResourcesT* res, const USBSSP_DriverContextT* drvContext)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (USBSSP_DriverResourcesTSF(res) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else if (USBSSP_DriverContextTSF(drvContext) == CDN_EINVAL)
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
