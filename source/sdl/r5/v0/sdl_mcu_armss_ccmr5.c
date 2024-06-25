/********************************************************************
 * Copyright (C) 2022-2024 Texas Instruments Incorporated.
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
 *  Name        : sdl_mcu_armss_ccmr5.h
*/
#include "sdl_mcu_armss_ccmr5.h"
#include <sdl/include/sdl_types.h>
#include <sdl/sdlr.h>

#define         SDL_MCU_ARMSS_CCMR5_REGID_UNDEFINED     ((uint32_t) (255u))
#define         SDL_MCU_ARMSS_CCMR5_NULL_ADDR           ((void *) 0 )

static uint32_t SDL_armR5GetRegOffset(SDL_McuArmssCcmR5RegId  regId);


static uint32_t SDL_armR5GetRegOffset(SDL_McuArmssCcmR5RegId  regId)
{
    uint32_t    offset = SDL_MCU_ARMSS_CCMR5_REGID_UNDEFINED;

    switch (regId)
    {
        case SDL_MCU_ARMSS_CCMR5_CCMSR1_REGID:
            offset = SDL_MCU_ARMSS_CCMR5_COMPARE_WRAPPER_CFG_MMRS_CCMSR1;
            break;
        case SDL_MCU_ARMSS_CCMR5_CCMKEYR1_REGID:
            offset = SDL_MCU_ARMSS_CCMR5_COMPARE_WRAPPER_CFG_MMRS_CCMKEYR1;
            break;
        case SDL_MCU_ARMSS_CCMR5_CCMSR2_REGID:
            offset = SDL_MCU_ARMSS_CCMR5_COMPARE_WRAPPER_CFG_MMRS_CCMSR2;
            break;
        case SDL_MCU_ARMSS_CCMR5_CCMKEYR2_REGID:
            offset = SDL_MCU_ARMSS_CCMR5_COMPARE_WRAPPER_CFG_MMRS_CCMKEYR2;
            break;
        case SDL_MCU_ARMSS_CCMR5_CCMSR3_REGID:
            offset = SDL_MCU_ARMSS_CCMR5_COMPARE_WRAPPER_CFG_MMRS_CCMSR3;
            break;
        case SDL_MCU_ARMSS_CCMR5_CCMKEYR3_REGID:
            offset = SDL_MCU_ARMSS_CCMR5_COMPARE_WRAPPER_CFG_MMRS_CCMKEYR3;
            break;
        case SDL_MCU_ARMSS_CCMR5_POLCNTRL_REGID:
            offset  = SDL_MCU_ARMSS_CCMR5_COMPARE_WRAPPER_CFG_MMRS_CCMPOLCNTRL;
            break;
#if defined (SOC_AM263PX) || (SOC_AM261X)
        case SDL_MCU_ARMSS_CCMR5_CCMSR5_REGID:
            offset = SDL_MCU_ARMSS_CCMR5_COMPARE_WRAPPER_CFG_MMRS_CCMSR5;
            break;
        case SDL_MCU_ARMSS_CCMR5_CCMKEYR5_REGID:
            offset = SDL_MCU_ARMSS_CCMR5_COMPARE_WRAPPER_CFG_MMRS_CCMKEYR5;
            break;
        case SDL_MCU_ARMSS_CCMR5_CCMSR6_REGID:
            offset = SDL_MCU_ARMSS_CCMR5_COMPARE_WRAPPER_CFG_MMRS_CCMSR6;
            break;
        case SDL_MCU_ARMSS_CCMR5_CCMKEYR6_REGID:
            offset = SDL_MCU_ARMSS_CCMR5_COMPARE_WRAPPER_CFG_MMRS_CCMKEYR6;
            break;
#endif
        default:
            break;
    }
    return (offset);
}

/**
 *   Design: PROC_SDL-2052
 */
int32_t SDL_armR5ReadCCMRegister (
                   uintptr_t                baseAddress,
                   SDL_McuArmssCcmR5RegId   regId,
                   uint32_t                 *pValToBeRead,
                   int32_t                  *pMetaInfo
)
{
    int32_t    retVal = SDL_PASS;
    int32_t    failReason = SDL_PASS;
    uint32_t   addr, offset = SDL_MCU_ARMSS_CCMR5_REGID_UNDEFINED;
    void       *pChkValRd = (void *) pValToBeRead;
    void       *pChkMetaInfo = (void *) pMetaInfo;


    /* Error check for the arguments passed in */
    if ( (baseAddress  == (uintptr_t) NULL) ||
         (pChkValRd    == SDL_MCU_ARMSS_CCMR5_NULL_ADDR) )
    {
        retVal         = SDL_EFAIL;
        failReason     = SDL_EBADARGS;
    }

    /* Read the register intended */
    if ( retVal == SDL_PASS)
    {
        offset = SDL_armR5GetRegOffset(regId);
    }

     if (offset == SDL_MCU_ARMSS_CCMR5_REGID_UNDEFINED)
     {
         retVal         = SDL_EFAIL;
         failReason     = SDL_EBADARGS;
     }
     else
     {
         addr           = ((uint32_t) baseAddress + offset);
         *pValToBeRead   = SDL_REG32_RD(addr);
     }

    /* Update the fail reason if requested */
    if (pChkMetaInfo != SDL_MCU_ARMSS_CCMR5_NULL_ADDR)
    {
        *pMetaInfo = failReason;
    }
    return (retVal);
}

/**
 *   Design: PROC_SDL-2053
 */
int32_t SDL_armR5ConfigureCCMRegister (
                   uintptr_t                baseAddress,
                   SDL_McuArmssCcmR5RegId   regId,
                   uint32_t                 valToBeWritten,
                   int32_t                  *pMetaInfo)
{
    int32_t    retVal;
    int32_t    failReason;
    uint32_t   addr;
    uint32_t    offset = SDL_MCU_ARMSS_CCMR5_REGID_UNDEFINED;
    void       *pChkMetaInfo = (void *) pMetaInfo;

    /* Error check for the arguments passed in */
    if (baseAddress == (uintptr_t)(NULL)  )
    {
        retVal         = SDL_EFAIL;
        failReason     = SDL_EBADARGS;
    }
    else
    {
        retVal         = SDL_PASS;
        failReason     = (int32_t) NULL;
    }

    /* get the offset for the register intended */
    if ( retVal == SDL_PASS)
    {
        offset = SDL_armR5GetRegOffset(regId);
    }

    if (offset == SDL_MCU_ARMSS_CCMR5_REGID_UNDEFINED)
    {
        retVal         = SDL_EFAIL;
        failReason     = SDL_EBADARGS;
    }
    else
    {
        addr           = ((uint32_t) baseAddress + offset);
        SDL_REG32_WR(addr, valToBeWritten);
    }

    /* Update the fail reason if requested */
    if (pChkMetaInfo != SDL_MCU_ARMSS_CCMR5_NULL_ADDR)
    {
        *pMetaInfo = failReason;
    }

    return (retVal);
}

/**
 *   Design: PROC_SDL-2054
 */
int32_t SDL_armR5CCMSetOperationModeKey (
                   uintptr_t                    baseAddress,
                   SDL_McuArmssCcmR5ModuleId    moduleId,
                   SDL_McuArmssCcmR5OpModeKey   opModeKey,
                   int32_t                     *pMetaInfo
)
{
    int32_t    retVal = SDL_PASS;
    int32_t    failReason = (int32_t) NULL;
    volatile   uint32_t  addr;
    uint32_t    mKey;
    void       *pChkMetaInfo = (void *) pMetaInfo;
    uint32_t    offset = SDL_MCU_ARMSS_CCMR5_REGID_UNDEFINED;

    /* Error check for the arguments passed in */
    if (baseAddress == (uintptr_t)(NULL)  )
    {
        retVal         = SDL_EFAIL;
        failReason     = SDL_EBADARGS;
    }

    /* map the op mode key unsighed integer */
    switch (opModeKey)
    {
            case SDL_MCU_ARMSS_CCMR5_MKEY_CMP_MODE_ACTIVE:
                    mKey = ((uint32_t) 0u);
                     break;
            case SDL_MCU_ARMSS_CCMR5_MKEY_SELF_TEST_MODE:
                    mKey = ((uint32_t) 6u);
                     break;
            case SDL_MCU_ARMSS_CCMR5_MKEY_ERR_FORCE_MODE:
                     mKey = ((uint32_t) 9u);
                     break;
            case SDL_MCU_ARMSS_CCMR5_MKEY_SELF_TEST_ERR_FORCE_MODE:
                    mKey = ((uint32_t) 15u);
                     break;
            default:
                    retVal         = SDL_EFAIL;
                    failReason     = SDL_EBADARGS;
                     break;
    }

    /* write the key in the CCM Key register */
    if ( retVal == SDL_PASS)
    {
        switch (moduleId)
        {
            case SDL_MCU_ARMSS_CCMR5_CPU_MODULE_ID:
                 offset = SDL_armR5GetRegOffset(SDL_MCU_ARMSS_CCMR5_CCMKEYR1_REGID);
                 addr           = ((uint32_t) baseAddress + offset);
                 SDL_REG32_FINS(addr, \
                     MCU_ARMSS_CCMR5_COMPARE_WRAPPER_CFG_MMRS_CCMKEYR1_MKEY1, \
                     mKey);
                 break;

            case SDL_MCU_ARMSS_CCMR5_VIM_MODULE_ID:
                offset = SDL_armR5GetRegOffset(SDL_MCU_ARMSS_CCMR5_CCMKEYR2_REGID);
                addr           = ((uint32_t) baseAddress + offset);
                 SDL_REG32_FINS(addr, \
                      MCU_ARMSS_CCMR5_COMPARE_WRAPPER_CFG_MMRS_CCMKEYR2_MKEY2, \
                      mKey);
                 break;

            case SDL_MCU_ARMSS_CCMR5_INACTIVITY_MONITOR_MODULE_ID:
                offset = SDL_armR5GetRegOffset(SDL_MCU_ARMSS_CCMR5_CCMKEYR3_REGID);
                addr           = ((uint32_t) baseAddress + offset);
                 SDL_REG32_FINS(addr, \
                      MCU_ARMSS_CCMR5_COMPARE_WRAPPER_CFG_MMRS_CCMKEYR3_MKEY3, \
                      mKey);
                 break;
#if defined (SOC_AM263PX) || (SOC_AM261X)
            case SDL_MCU_ARMSS_CCMR5_TMU_MODULE_ID:
                offset = SDL_armR5GetRegOffset(SDL_MCU_ARMSS_CCMR5_CCMKEYR5_REGID);
                addr           = ((uint32_t) baseAddress + offset);
                 SDL_REG32_FINS(addr, \
                      MCU_ARMSS_CCMR5_COMPARE_WRAPPER_CFG_MMRS_CCMKEYR5_MKEY5, \
                      mKey);
                 break;

            case SDL_MCU_ARMSS_CCMR5_RL2_MODULE_ID:
                offset = SDL_armR5GetRegOffset(SDL_MCU_ARMSS_CCMR5_CCMKEYR6_REGID);
                addr           = ((uint32_t) baseAddress + offset);
                 SDL_REG32_FINS(addr, \
                      MCU_ARMSS_CCMR5_COMPARE_WRAPPER_CFG_MMRS_CCMKEYR6_MKEY6, \
                      mKey);
                 break;
#endif
            default:
                 retVal         = SDL_EFAIL;
                 failReason     = SDL_EBADARGS;
                 break;
        }
    }

    /* Update the fail reason if requested */
    if (pChkMetaInfo != SDL_MCU_ARMSS_CCMR5_NULL_ADDR)
    {
        *pMetaInfo = failReason;
    }

    return (retVal);
}

/**
 *   Design: PROC_SDL-2055
 */
int32_t SDL_armR5CCMGetCompareError (
                   uintptr_t                      baseAddress,
                   SDL_McuArmssCcmR5ModuleId      moduleId,
                   uint32_t                      *pCmpError,
                   int32_t                       *pMetaInfo
)
{
    int32_t    retVal = SDL_PASS;
    int32_t    failReason = (int32_t) NULL;
    uint32_t   addr;
    void       *pChkMetaInfo = (void *) pMetaInfo;
    void       *pChkCmpErr   = (void *) pCmpError;
    uint32_t    offset = SDL_MCU_ARMSS_CCMR5_REGID_UNDEFINED;

    /* Error check for the arguments passed in */
    if ( (baseAddress == (uintptr_t)(NULL))||
         (pChkCmpErr  ==  SDL_MCU_ARMSS_CCMR5_NULL_ADDR) )
    {
        retVal         = SDL_EFAIL;
        failReason     = SDL_EBADARGS;
    }

    /* Read the register intended */
    if ( retVal == SDL_PASS)
    {
        switch (moduleId)
        {
            case SDL_MCU_ARMSS_CCMR5_CPU_MODULE_ID:
                offset = SDL_armR5GetRegOffset(SDL_MCU_ARMSS_CCMR5_CCMSR1_REGID);
                addr           = ((uint32_t) baseAddress + offset);
                *pCmpError  = SDL_REG32_FEXT(addr, \
                     MCU_ARMSS_CCMR5_COMPARE_WRAPPER_CFG_MMRS_CCMSR1_CMPE1);
                     break;

            case SDL_MCU_ARMSS_CCMR5_VIM_MODULE_ID:
                offset = SDL_armR5GetRegOffset(SDL_MCU_ARMSS_CCMR5_CCMSR2_REGID);
                addr           = ((uint32_t) baseAddress + offset);
                *pCmpError = SDL_REG32_FEXT(addr, \
                      MCU_ARMSS_CCMR5_COMPARE_WRAPPER_CFG_MMRS_CCMSR2_CMPE2);
                  break;

            case SDL_MCU_ARMSS_CCMR5_INACTIVITY_MONITOR_MODULE_ID:
                offset = SDL_armR5GetRegOffset(SDL_MCU_ARMSS_CCMR5_CCMSR3_REGID);
                addr           = ((uint32_t) baseAddress + offset);
                *pCmpError = SDL_REG32_FEXT(addr, \
                      MCU_ARMSS_CCMR5_COMPARE_WRAPPER_CFG_MMRS_CCMSR3_CMPE3);
                  break;
#if defined (SOC_AM263PX) || (SOC_AM261X)
            case SDL_MCU_ARMSS_CCMR5_TMU_MODULE_ID:
                offset = SDL_armR5GetRegOffset(SDL_MCU_ARMSS_CCMR5_CCMSR5_REGID);
                addr           = ((uint32_t) baseAddress + offset);
                *pCmpError = SDL_REG32_FEXT(addr, \
                      MCU_ARMSS_CCMR5_COMPARE_WRAPPER_CFG_MMRS_CCMSR5_CMPE5);
                  break;

            case SDL_MCU_ARMSS_CCMR5_RL2_MODULE_ID:
                offset = SDL_armR5GetRegOffset(SDL_MCU_ARMSS_CCMR5_CCMSR5_REGID);
                addr           = ((uint32_t) baseAddress + offset);
                *pCmpError = SDL_REG32_FEXT(addr, \
                      MCU_ARMSS_CCMR5_COMPARE_WRAPPER_CFG_MMRS_CCMSR6_CMPE6);
                  break;
#endif
            default:
                  retVal         = SDL_EFAIL;
                  failReason     = SDL_EBADARGS;
                  break;
        }
    }

    /* Update the fail reason if requested */
    if (pChkMetaInfo != SDL_MCU_ARMSS_CCMR5_NULL_ADDR)
    {
        *pMetaInfo = failReason;
    }

    return (retVal);
}

/**
 *   Design: PROC_SDL-2056
 */
int32_t SDL_armR5CCMGetOperationModeKey (
                   uintptr_t                      baseAddress,
                   SDL_McuArmssCcmR5ModuleId      moduleId,
                   SDL_McuArmssCcmR5OpModeKey    *pOpModeKey,
                   int32_t                       *pMetaInfo
)
{
    int32_t    retVal = SDL_PASS;
    int32_t    failReason = (int32_t) NULL;
    uint32_t   addr, mKey = 255u;
    void       *pChkMetaInfo = (void *) pMetaInfo;
    void       *pChkOpMKey = (void *) pOpModeKey;
    uint32_t    offset = SDL_MCU_ARMSS_CCMR5_REGID_UNDEFINED;

    /* Error check for the arguments passed in */
    if ( (baseAddress == (uintptr_t)(NULL))||
         (pChkOpMKey  == SDL_MCU_ARMSS_CCMR5_NULL_ADDR) )
    {
        retVal         = SDL_EFAIL;
        failReason     = SDL_EBADARGS;
    }

    /* Read the register intended */
    if ( retVal == SDL_PASS)
    {
        switch (moduleId)
        {
            case SDL_MCU_ARMSS_CCMR5_CPU_MODULE_ID:
                     offset = SDL_armR5GetRegOffset(SDL_MCU_ARMSS_CCMR5_CCMKEYR1_REGID);
                     addr           = ((uint32_t) baseAddress + offset);
                     mKey = (uint32_t) SDL_REG32_FEXT(addr, \
                      MCU_ARMSS_CCMR5_COMPARE_WRAPPER_CFG_MMRS_CCMKEYR1_MKEY1);
                     break;

            case SDL_MCU_ARMSS_CCMR5_VIM_MODULE_ID:
                    offset = SDL_armR5GetRegOffset(SDL_MCU_ARMSS_CCMR5_CCMKEYR2_REGID);
                    addr           = ((uint32_t) baseAddress + offset);
                     mKey = (uint32_t) SDL_REG32_FEXT(addr, \
                      MCU_ARMSS_CCMR5_COMPARE_WRAPPER_CFG_MMRS_CCMKEYR2_MKEY2);
                     break;

            case SDL_MCU_ARMSS_CCMR5_INACTIVITY_MONITOR_MODULE_ID:
                    offset = SDL_armR5GetRegOffset(SDL_MCU_ARMSS_CCMR5_CCMKEYR3_REGID);
                    addr           = ((uint32_t) baseAddress + offset);
                     mKey    = (uint32_t)SDL_REG32_FEXT(addr, \
                      MCU_ARMSS_CCMR5_COMPARE_WRAPPER_CFG_MMRS_CCMKEYR3_MKEY3);
                     break;

#if defined (SOC_AM263PX) || (SOC_AM261X)
            case SDL_MCU_ARMSS_CCMR5_TMU_MODULE_ID:
                    offset = SDL_armR5GetRegOffset(SDL_MCU_ARMSS_CCMR5_CCMKEYR3_REGID);
                    addr           = ((uint32_t) baseAddress + offset);
                     mKey    = (uint32_t)SDL_REG32_FEXT(addr, \
                      MCU_ARMSS_CCMR5_COMPARE_WRAPPER_CFG_MMRS_CCMKEYR3_MKEY3);
                     break;

            case SDL_MCU_ARMSS_CCMR5_RL2_MODULE_ID:
                    offset = SDL_armR5GetRegOffset(SDL_MCU_ARMSS_CCMR5_CCMKEYR3_REGID);
                    addr           = ((uint32_t) baseAddress + offset);
                     mKey    = (uint32_t)SDL_REG32_FEXT(addr, \
                      MCU_ARMSS_CCMR5_COMPARE_WRAPPER_CFG_MMRS_CCMKEYR3_MKEY3);
                     break;
#endif
            default:
                     retVal         = SDL_EFAIL;
                     failReason     = SDL_EBADARGS;
                     break;
        }
    }

    /* map the register to enum return */
    switch (mKey)
    {
            case 0u:
                    *pOpModeKey = SDL_MCU_ARMSS_CCMR5_MKEY_CMP_MODE_ACTIVE;
                     break;
            case 6u:
                    *pOpModeKey = SDL_MCU_ARMSS_CCMR5_MKEY_SELF_TEST_MODE;
                     break;
            case 9u:
                    *pOpModeKey = SDL_MCU_ARMSS_CCMR5_MKEY_ERR_FORCE_MODE;
                     break;
            case 15u:
                    *pOpModeKey = SDL_MCU_ARMSS_CCMR5_MKEY_SELF_TEST_ERR_FORCE_MODE;
                     break;
            default:
                     break;
    }

    /* Update the fail reason if requested */
    if (pChkMetaInfo != SDL_MCU_ARMSS_CCMR5_NULL_ADDR)
    {
        *pMetaInfo = failReason;
    }

    return (retVal);
}

/**
 *   Design: PROC_SDL-2057
 */
int32_t SDL_armR5CCMClearCompareError (
                   uintptr_t                      baseAddress,
                   SDL_McuArmssCcmR5ModuleId      moduleId,
                   int32_t                       *pMetaInfo
)
{
    int32_t    retVal = SDL_PASS;
    int32_t    failReason = (int32_t) NULL;
    uint32_t   addr;
    uint32_t   cmpE = ((uint32_t)(1u));
    void       *pChkMetaInfo = (void *) pMetaInfo;
    uint32_t    offset = SDL_MCU_ARMSS_CCMR5_REGID_UNDEFINED;

    /* Error check for the arguments passed in */
    if ( baseAddress == (uintptr_t)(NULL) )
    {
        retVal         = SDL_EFAIL;
        failReason     = SDL_EBADARGS;
    }

    /* Read the register intended */
    if ( retVal == SDL_PASS)
    {
        switch (moduleId)
        {
            case SDL_MCU_ARMSS_CCMR5_CPU_MODULE_ID:
                      offset = SDL_armR5GetRegOffset(SDL_MCU_ARMSS_CCMR5_CCMSR1_REGID);
                      addr           = ((uint32_t) baseAddress + offset);

                      SDL_REG32_FINS(addr, \
                      MCU_ARMSS_CCMR5_COMPARE_WRAPPER_CFG_MMRS_CCMSR1_CMPE1, \
                      cmpE);
                     break;

            case SDL_MCU_ARMSS_CCMR5_VIM_MODULE_ID:
                    offset = SDL_armR5GetRegOffset(SDL_MCU_ARMSS_CCMR5_CCMSR2_REGID);
                    addr           = ((uint32_t) baseAddress + offset);

                     SDL_REG32_FINS(addr, \
                      MCU_ARMSS_CCMR5_COMPARE_WRAPPER_CFG_MMRS_CCMSR2_CMPE2, \
                      cmpE);
                     break;

            case SDL_MCU_ARMSS_CCMR5_INACTIVITY_MONITOR_MODULE_ID:
                     offset = SDL_armR5GetRegOffset(SDL_MCU_ARMSS_CCMR5_CCMSR3_REGID);
                     addr           = ((uint32_t) baseAddress + offset);

                     SDL_REG32_FINS(addr, \
                      MCU_ARMSS_CCMR5_COMPARE_WRAPPER_CFG_MMRS_CCMSR3_CMPE3, \
                      cmpE);
                     break;

#if defined (SOC_AM263PX) || (SOC_AM261X)
            case SDL_MCU_ARMSS_CCMR5_TMU_MODULE_ID:
                     offset = SDL_armR5GetRegOffset(SDL_MCU_ARMSS_CCMR5_CCMSR3_REGID);
                     addr           = ((uint32_t) baseAddress + offset);

                     SDL_REG32_FINS(addr, \
                      MCU_ARMSS_CCMR5_COMPARE_WRAPPER_CFG_MMRS_CCMSR3_CMPE3, \
                      cmpE);
                     break;

            case SDL_MCU_ARMSS_CCMR5_RL2_MODULE_ID:
                     offset = SDL_armR5GetRegOffset(SDL_MCU_ARMSS_CCMR5_CCMSR3_REGID);
                     addr           = ((uint32_t) baseAddress + offset);

                     SDL_REG32_FINS(addr, \
                      MCU_ARMSS_CCMR5_COMPARE_WRAPPER_CFG_MMRS_CCMSR3_CMPE3, \
                      cmpE);
                     break;

#endif

            default:
                     retVal         = SDL_EFAIL;
                     failReason     = SDL_EBADARGS;
                     break;
        }
    }

    /* Update the fail reason if requested */
    if (pChkMetaInfo != SDL_MCU_ARMSS_CCMR5_NULL_ADDR)
    {
        *pMetaInfo = failReason;
    }

    return (retVal);
}

