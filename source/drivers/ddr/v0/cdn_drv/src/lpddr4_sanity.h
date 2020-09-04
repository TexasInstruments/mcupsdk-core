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
*          api-generator: 12.02.13bb8d5
*          Do not edit it manually.
**********************************************************************
* Cadence Core Driver for LPDDR4.
**********************************************************************/

/* parasoft-begin-suppress METRICS-18-3 "Follow the Cyclomatic Complexity limit of 10" */
/* parasoft-begin-suppress METRICS-36-3 "A function should not be called from more than 5 different functions" */
/* parasoft-begin-suppress METRICS-39-3 "The value of VOCF metric for a function should not be higher than 4" */
/* parasoft-begin-suppress METRICS-41-3 "Number of blocks of comments per statement" */

/**
 * This file contains sanity API functions. The purpose of sanity functions
 * is to check input parameters validity. They take the same parameters as
 * original API functions and return 0 on success or CDN_EINVAL on wrong parameter
 * value(s).
 */

#ifndef LPDDR4_SANITY_H
#define LPDDR4_SANITY_H

#include "../include/common/cdn_errno.h"
#include "../include/common/cdn_stdtypes.h"
#include "../include/common/lpddr4_if.h"
#ifdef __cplusplus
extern "C" {
#endif

static inline uint32_t LPDDR4_ConfigSF(const LPDDR4_Config *obj);
static inline uint32_t LPDDR4_PrivateDataSF(const LPDDR4_PrivateData *obj);

static inline uint32_t LPDDR4_SanityFunction1(const LPDDR4_Config* config, const uint16_t* configSize);
static inline uint32_t LPDDR4_SanityFunction2(const LPDDR4_PrivateData* pD, const LPDDR4_Config* cfg);
static inline uint32_t LPDDR4_SanityFunction3(const LPDDR4_PrivateData* pD);
static inline uint32_t LPDDR4_SanityFunction4(const LPDDR4_PrivateData* pD, const LPDDR4_RegBlock cpp, const uint32_t* regValue);
static inline uint32_t LPDDR4_SanityFunction5(const LPDDR4_PrivateData* pD, const LPDDR4_RegBlock cpp);
static inline uint32_t LPDDR4_SanityFunction6(const LPDDR4_PrivateData* pD, const uint64_t* mmrValue, const uint8_t* mmrStatus);
static inline uint32_t LPDDR4_SanityFunction7(const LPDDR4_PrivateData* pD, const uint8_t* mrwStatus);
static inline uint32_t LPDDR4_SanityFunction14(const LPDDR4_PrivateData* pD, const uint64_t* mask);
static inline uint32_t LPDDR4_SanityFunction15(const LPDDR4_PrivateData* pD, const uint64_t* mask);
static inline uint32_t LPDDR4_SanityFunction16(const LPDDR4_PrivateData* pD, const uint32_t* mask);
static inline uint32_t LPDDR4_SanityFunction18(const LPDDR4_PrivateData* pD, const LPDDR4_DebugInfo* debugInfo);
static inline uint32_t LPDDR4_SanityFunction19(const LPDDR4_PrivateData* pD, const LPDDR4_LpiWakeUpParam* lpiWakeUpParam, const LPDDR4_CtlFspNum* fspNum, const uint32_t* cycles);
static inline uint32_t LPDDR4_SanityFunction21(const LPDDR4_PrivateData* pD, const LPDDR4_EccEnable* eccParam);
static inline uint32_t LPDDR4_SanityFunction22(const LPDDR4_PrivateData* pD, const LPDDR4_EccEnable* eccParam);
static inline uint32_t LPDDR4_SanityFunction23(const LPDDR4_PrivateData* pD, const LPDDR4_ReducMode* mode);
static inline uint32_t LPDDR4_SanityFunction24(const LPDDR4_PrivateData* pD, const LPDDR4_ReducMode* mode);
static inline uint32_t LPDDR4_SanityFunction25(const LPDDR4_PrivateData* pD, const bool* on_off);
static inline uint32_t LPDDR4_SanityFunction27(const LPDDR4_PrivateData* pD, const LPDDR4_DbiMode* mode);
static inline uint32_t LPDDR4_SanityFunction28(const LPDDR4_PrivateData* pD, const LPDDR4_CtlFspNum* fspNum, const uint32_t* tref_val, const uint32_t* tras_max_val);
static inline uint32_t LPDDR4_SanityFunction29(const LPDDR4_PrivateData* pD, const LPDDR4_CtlFspNum* fspNum, const uint32_t* tref, const uint32_t* tras_max);

#define LPDDR4_ProbeSF LPDDR4_SanityFunction1
#define LPDDR4_InitSF LPDDR4_SanityFunction2
#define LPDDR4_StartSF LPDDR4_SanityFunction3
#define LPDDR4_ReadRegSF LPDDR4_SanityFunction4
#define LPDDR4_WriteRegSF LPDDR4_SanityFunction5
#define LPDDR4_GetMmrRegisterSF LPDDR4_SanityFunction6
#define LPDDR4_SetMmrRegisterSF LPDDR4_SanityFunction7
#define LPDDR4_WriteCtlConfigSF LPDDR4_SanityFunction3
#define LPDDR4_WritePhyConfigSF LPDDR4_SanityFunction3
#define LPDDR4_WritePhyIndepConfigSF LPDDR4_SanityFunction3
#define LPDDR4_ReadCtlConfigSF LPDDR4_SanityFunction3
#define LPDDR4_ReadPhyConfigSF LPDDR4_SanityFunction3
#define LPDDR4_ReadPhyIndepConfigSF LPDDR4_SanityFunction3
#define LPDDR4_GetCtlInterruptMaskSF LPDDR4_SanityFunction14
#define LPDDR4_SetCtlInterruptMaskSF LPDDR4_SanityFunction15
#define LPDDR4_GetPhyIndepInterruptMSF LPDDR4_SanityFunction16
#define LPDDR4_SetPhyIndepInterruptMSF LPDDR4_SanityFunction16
#define LPDDR4_GetDebugInitInfoSF LPDDR4_SanityFunction18
#define LPDDR4_GetLpiWakeUpTimeSF LPDDR4_SanityFunction19
#define LPDDR4_SetLpiWakeUpTimeSF LPDDR4_SanityFunction19
#define LPDDR4_GetEccEnableSF LPDDR4_SanityFunction21
#define LPDDR4_SetEccEnableSF LPDDR4_SanityFunction22
#define LPDDR4_GetReducModeSF LPDDR4_SanityFunction23
#define LPDDR4_SetReducModeSF LPDDR4_SanityFunction24
#define LPDDR4_GetDbiReadModeSF LPDDR4_SanityFunction25
#define LPDDR4_GetDbiWriteModeSF LPDDR4_SanityFunction25
#define LPDDR4_SetDbiModeSF LPDDR4_SanityFunction27
#define LPDDR4_GetRefreshRateSF LPDDR4_SanityFunction28
#define LPDDR4_SetRefreshRateSF LPDDR4_SanityFunction29
#define LPDDR4_RefreshPerChipSelectSF LPDDR4_SanityFunction3
#define LPDDR4_DeferredRegVerifySF LPDDR4_SanityFunction5

/**
 * Function to validate struct Config
 *
 * @param[in] obj pointer to struct to be verified
 * @returns 0 for valid
 * @returns CDN_EINVAL for invalid
 */
static inline uint32_t LPDDR4_ConfigSF(const LPDDR4_Config *obj)
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
static inline uint32_t LPDDR4_PrivateDataSF(const LPDDR4_PrivateData *obj)
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
 * @param[in] config Driver/hardware configuration required.
 * @param[out] configSize Size of memory allocations required.
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
static inline uint32_t LPDDR4_SanityFunction1(const LPDDR4_Config* config, const uint16_t* configSize)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (configSize == NULL)
    {
        ret = CDN_EINVAL;
    }
    else if (LPDDR4_ConfigSF(config) == CDN_EINVAL)
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
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] cfg Specifies driver/hardware configuration.
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
static inline uint32_t LPDDR4_SanityFunction2(const LPDDR4_PrivateData* pD, const LPDDR4_Config* cfg)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (LPDDR4_PrivateDataSF(pD) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else if (LPDDR4_ConfigSF(cfg) == CDN_EINVAL)
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
 * @param[in] pD Driver state info specific to this instance.
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
static inline uint32_t LPDDR4_SanityFunction3(const LPDDR4_PrivateData* pD)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (LPDDR4_PrivateDataSF(pD) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }

    return ret;
}

/**
 * A common function to check the validity of API functions with
 * following parameter types
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] cpp Indicates whether controller, PHY or PHY Independent Module register
 * @param[out] regValue Register value read
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
static inline uint32_t LPDDR4_SanityFunction4(const LPDDR4_PrivateData* pD, const LPDDR4_RegBlock cpp, const uint32_t* regValue)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (regValue == NULL)
    {
        ret = CDN_EINVAL;
    }
    else if (LPDDR4_PrivateDataSF(pD) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else if (
        (cpp != LPDDR4_CTL_REGS) &&
        (cpp != LPDDR4_PHY_REGS) &&
        (cpp != LPDDR4_PHY_INDEP_REGS)
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
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] cpp Indicates whether controller, PHY or PHY Independent Module register
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
static inline uint32_t LPDDR4_SanityFunction5(const LPDDR4_PrivateData* pD, const LPDDR4_RegBlock cpp)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (LPDDR4_PrivateDataSF(pD) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else if (
        (cpp != LPDDR4_CTL_REGS) &&
        (cpp != LPDDR4_PHY_REGS) &&
        (cpp != LPDDR4_PHY_INDEP_REGS)
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
 * @param[in] pD Driver state info specific to this instance.
 * @param[out] mmrValue Value which is read from memory mode register(mmr) for all devices.
 * @param[out] mmrStatus Status of mode register read(mrr) instruction.
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
static inline uint32_t LPDDR4_SanityFunction6(const LPDDR4_PrivateData* pD, const uint64_t* mmrValue, const uint8_t* mmrStatus)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (mmrValue == NULL)
    {
        ret = CDN_EINVAL;
    }
    else if (mmrStatus == NULL)
    {
        ret = CDN_EINVAL;
    }
    else if (LPDDR4_PrivateDataSF(pD) == CDN_EINVAL)
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
 * @param[in] pD Driver state info specific to this instance.
 * @param[out] mrwStatus Status of mode register write(mrw) instruction.
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
static inline uint32_t LPDDR4_SanityFunction7(const LPDDR4_PrivateData* pD, const uint8_t* mrwStatus)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (mrwStatus == NULL)
    {
        ret = CDN_EINVAL;
    }
    else if (LPDDR4_PrivateDataSF(pD) == CDN_EINVAL)
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
 * @param[in] pD Driver state info specific to this instance.
 * @param[out] mask Value of interrupt mask
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
static inline uint32_t LPDDR4_SanityFunction14(const LPDDR4_PrivateData* pD, const uint64_t* mask)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (mask == NULL)
    {
        ret = CDN_EINVAL;
    }
    else if (LPDDR4_PrivateDataSF(pD) == CDN_EINVAL)
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
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] mask Value of interrupt mask to be written
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
static inline uint32_t LPDDR4_SanityFunction15(const LPDDR4_PrivateData* pD, const uint64_t* mask)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (mask == NULL)
    {
        ret = CDN_EINVAL;
    }
    else if (LPDDR4_PrivateDataSF(pD) == CDN_EINVAL)
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
 * @param[in] pD Driver state info specific to this instance.
 * @param[out] mask Value of interrupt mask
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
static inline uint32_t LPDDR4_SanityFunction16(const LPDDR4_PrivateData* pD, const uint32_t* mask)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (mask == NULL)
    {
        ret = CDN_EINVAL;
    }
    else if (LPDDR4_PrivateDataSF(pD) == CDN_EINVAL)
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
 * @param[in] pD Driver state info specific to this instance.
 * @param[out] debugInfo status
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
static inline uint32_t LPDDR4_SanityFunction18(const LPDDR4_PrivateData* pD, const LPDDR4_DebugInfo* debugInfo)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (debugInfo == NULL)
    {
        ret = CDN_EINVAL;
    }
    else if (LPDDR4_PrivateDataSF(pD) == CDN_EINVAL)
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
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] lpiWakeUpParam LPI timing parameter
 * @param[in] fspNum Frequency copy
 * @param[out] cycles Timing value(in cycles)
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
static inline uint32_t LPDDR4_SanityFunction19(const LPDDR4_PrivateData* pD, const LPDDR4_LpiWakeUpParam* lpiWakeUpParam, const LPDDR4_CtlFspNum* fspNum, const uint32_t* cycles)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (lpiWakeUpParam == NULL)
    {
        ret = CDN_EINVAL;
    }
    else if (fspNum == NULL)
    {
        ret = CDN_EINVAL;
    }
    else if (cycles == NULL)
    {
        ret = CDN_EINVAL;
    }
    else if (LPDDR4_PrivateDataSF(pD) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else if (
        (*lpiWakeUpParam != LPDDR4_LPI_PD_WAKEUP_FN) &&
        (*lpiWakeUpParam != LPDDR4_LPI_SR_SHORT_WAKEUP_FN) &&
        (*lpiWakeUpParam != LPDDR4_LPI_SR_LONG_WAKEUP_FN) &&
        (*lpiWakeUpParam != LPDDR4_LPI_SR_LONG_MCCLK_GATE_WAKEUP_FN) &&
        (*lpiWakeUpParam != LPDDR4_LPI_SRPD_SHORT_WAKEUP_FN) &&
        (*lpiWakeUpParam != LPDDR4_LPI_SRPD_LONG_WAKEUP_FN) &&
        (*lpiWakeUpParam != LPDDR4_LPI_SRPD_LONG_MCCLK_GATE_WAKEUP_FN)
        )
    {
        ret = CDN_EINVAL;
    }
    else if (
        (*fspNum != LPDDR4_FSP_0) &&
        (*fspNum != LPDDR4_FSP_1) &&
        (*fspNum != LPDDR4_FSP_2)
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
 * @param[in] pD Driver state info specific to this instance.
 * @param[out] eccParam ECC parameter setting
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
static inline uint32_t LPDDR4_SanityFunction21(const LPDDR4_PrivateData* pD, const LPDDR4_EccEnable* eccParam)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (eccParam == NULL)
    {
        ret = CDN_EINVAL;
    }
    else if (LPDDR4_PrivateDataSF(pD) == CDN_EINVAL)
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
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] eccParam ECC control parameter setting
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
static inline uint32_t LPDDR4_SanityFunction22(const LPDDR4_PrivateData* pD, const LPDDR4_EccEnable* eccParam)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (eccParam == NULL)
    {
        ret = CDN_EINVAL;
    }
    else if (LPDDR4_PrivateDataSF(pD) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else if (
        (*eccParam != LPDDR4_ECC_DISABLED) &&
        (*eccParam != LPDDR4_ECC_ENABLED) &&
        (*eccParam != LPDDR4_ECC_ERR_DETECT) &&
        (*eccParam != LPDDR4_ECC_ERR_DETECT_CORRECT)
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
 * @param[in] pD Driver state info specific to this instance.
 * @param[out] mode Half Datapath setting
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
static inline uint32_t LPDDR4_SanityFunction23(const LPDDR4_PrivateData* pD, const LPDDR4_ReducMode* mode)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (mode == NULL)
    {
        ret = CDN_EINVAL;
    }
    else if (LPDDR4_PrivateDataSF(pD) == CDN_EINVAL)
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
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] mode Half Datapath setting
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
static inline uint32_t LPDDR4_SanityFunction24(const LPDDR4_PrivateData* pD, const LPDDR4_ReducMode* mode)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (mode == NULL)
    {
        ret = CDN_EINVAL;
    }
    else if (LPDDR4_PrivateDataSF(pD) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else if (
        (*mode != LPDDR4_REDUC_ON) &&
        (*mode != LPDDR4_REDUC_OFF)
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
 * @param[in] pD Driver state info specific to this instance.
 * @param[out] on_off DBI read value
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
static inline uint32_t LPDDR4_SanityFunction25(const LPDDR4_PrivateData* pD, const bool* on_off)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (on_off == NULL)
    {
        ret = CDN_EINVAL;
    }
    else if (LPDDR4_PrivateDataSF(pD) == CDN_EINVAL)
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
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] mode status
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
static inline uint32_t LPDDR4_SanityFunction27(const LPDDR4_PrivateData* pD, const LPDDR4_DbiMode* mode)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (mode == NULL)
    {
        ret = CDN_EINVAL;
    }
    else if (LPDDR4_PrivateDataSF(pD) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else if (
        (*mode != LPDDR4_DBI_RD_ON) &&
        (*mode != LPDDR4_DBI_RD_OFF) &&
        (*mode != LPDDR4_DBI_WR_ON) &&
        (*mode != LPDDR4_DBI_WR_OFF)
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
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] fspNum Frequency set number
 * @param[out] tref_val Refresh rate (in cycles)
 * @param[out] tras_max_val Maximum row active time (in cycles)
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
static inline uint32_t LPDDR4_SanityFunction28(const LPDDR4_PrivateData* pD, const LPDDR4_CtlFspNum* fspNum, const uint32_t* tref_val, const uint32_t* tras_max_val)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (fspNum == NULL)
    {
        ret = CDN_EINVAL;
    }
    else if (tref_val == NULL)
    {
        ret = CDN_EINVAL;
    }
    else if (tras_max_val == NULL)
    {
        ret = CDN_EINVAL;
    }
    else if (LPDDR4_PrivateDataSF(pD) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else if (
        (*fspNum != LPDDR4_FSP_0) &&
        (*fspNum != LPDDR4_FSP_1) &&
        (*fspNum != LPDDR4_FSP_2)
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
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] fspNum Frequency set number
 * @param[in] tref Refresh rate (in cycles)
 * @param[in] tras_max Maximum row active time (in cycles)
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
static inline uint32_t LPDDR4_SanityFunction29(const LPDDR4_PrivateData* pD, const LPDDR4_CtlFspNum* fspNum, const uint32_t* tref, const uint32_t* tras_max)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (fspNum == NULL)
    {
        ret = CDN_EINVAL;
    }
    else if (tref == NULL)
    {
        ret = CDN_EINVAL;
    }
    else if (tras_max == NULL)
    {
        ret = CDN_EINVAL;
    }
    else if (LPDDR4_PrivateDataSF(pD) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else if (
        (*fspNum != LPDDR4_FSP_0) &&
        (*fspNum != LPDDR4_FSP_1) &&
        (*fspNum != LPDDR4_FSP_2)
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

#ifdef __cplusplus
}
#endif

/* parasoft-end-suppress METRICS-41-3 */
/* parasoft-end-suppress METRICS-39-3 */
/* parasoft-end-suppress METRICS-36-3 */
/* parasoft-end-suppress METRICS-18-3 */

#endif  /* LPDDR4_SANITY_H */
