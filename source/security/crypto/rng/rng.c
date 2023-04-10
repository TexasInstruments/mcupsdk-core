/*
 *  Copyright (C) 2023 Texas Instruments Incorporated
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
 */

/**
 *  \file   rng.c
 *
 *  \brief  This file contains the implementation of RNG driver
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <string.h>
#include <stddef.h>
#include <security/crypto/rng/rng.h>
#include <kernel/dpl/ClockP.h>
#include <drivers/hw_include/cslr.h>
#include <security/crypto/pka/hw_include/cslr_cp_ace.h>
#include <drivers/hw_include/cslr_soc.h>
/* ========================================================================== */
/*                          Global variables                                  */
/* ========================================================================== */
/** Rng drbg default seed array */
uint32_t gRngDrbgDefaultSeed[RNG_DRBG_SEED_MAX_ARRY_SIZE_IN_DWORD]={0x425F4941, 0x37CCE532, 0x2C07C03E, 0x14CAEA55,
                               0x57DF93B5, 0xC277D946, 0xAE4728C1, 0x7FEBA982,
                               0xB3E156DA, 0xE7993855, 0x45EE2421, 0x506F53D1 };
/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

RNG_Handle RNG_open(uint32_t index)
{
    RNG_Return_t    status  = RNG_RETURN_SUCCESS;
    RNG_Handle      handle  = NULL;
    RNG_Config      *config = NULL;
    RNG_Attrs       *attrs  = NULL;

    #if (((defined (SOC_AM263X) || defined (SOC_AM273X)) && defined(__ARM_ARCH_7R__)))
        CSL_top_ctrlRegs * ptrTopCtrlRegs = (CSL_top_ctrlRegs *)CSL_TOP_CTRL_U_BASE;
        if(ptrTopCtrlRegs->EFUSE_DEVICE_TYPE == DEVTYPE_HSSE)
        {
            status = RNG_RETURN_FAILURE;
            return (handle);
        }
    #endif

    /* Check instance */
    if(index >= gRngConfigNum)
    {
        status = RNG_RETURN_FAILURE;
    }
    else
    {
        config = &gRngConfig[index];
        DebugP_assert(NULL != config->attrs);
        attrs = config->attrs;
        if(TRUE == attrs->isOpen)
        {
            /* Handle is already opened */
            status = RNG_close((RNG_Handle) config);
            status = RNG_RETURN_FAILURE;
            attrs->faultStatus = status;
        }
        else
        {
            attrs->isOpen = TRUE;
            handle = (RNG_Handle) config;
            attrs->faultStatus = status;
        }
    }

    return (handle);
}

RNG_Return_t RNG_close(RNG_Handle handle)
{
    RNG_Return_t status  = RNG_RETURN_FAILURE;
    RNG_Config  *config;
    RNG_Attrs   *attrs;
    config  = (RNG_Config *) handle;

    if((NULL != config) && (config->attrs->isOpen != (uint32_t)FALSE))
    {
        attrs = config->attrs;
        DebugP_assert(NULL != attrs);
        attrs->isOpen = FALSE;
        /* TO module disable */
        handle = NULL;
        status  = RNG_RETURN_SUCCESS;
    }
    return (status);
}

RNG_Return_t RNG_setup(RNG_Handle handle)
{
    uint32_t val = 0, updated_bits = 0, i = 0;
    RNG_Return_t retVal = RNG_RETURN_FAILURE;
    RNG_Config *config  = (RNG_Config *)handle;
    CSL_Cp_aceTrngRegs *pTrngRegs;

    if (NULL != handle)
    {
        pTrngRegs = (CSL_Cp_aceTrngRegs *)config->attrs->rngBaseAddr;

        CSL_REG_WR(&pTrngRegs->TRNG_CONTROL, 0U);
        CSL_REG_WR(&pTrngRegs->TRNG_CONTROL, 0U);

        /* Initialize TRNG_CONFIG to 0 */
        val = ((uint32_t) 0U);
        val |= ((((uint32_t) 5U) << CSL_CP_ACE_TRNG_CONFIG_SAMPLE_CYCLES_SHIFT) & (CSL_CP_ACE_TRNG_CONFIG_SAMPLE_CYCLES_MASK));
        CSL_REG_WR(&pTrngRegs->TRNG_CONFIG, val);

        /* Leave the ALARMCNT register at its reset value */
        val = ((uint32_t) 0xFFU);
        CSL_REG_WR(&pTrngRegs->TRNG_ALARMCNT, val);

        /* write zeros to ALARMMASK and ALARMSTOP registers */
        val = ((uint32_t) 0U);
        CSL_REG_WR(&pTrngRegs->TRNG_ALARMMASK, val);
        CSL_REG_WR(&pTrngRegs->TRNG_ALARMSTOP, val);

        /* We have 8 FRO's in the RNG */
        val = ((uint32_t) 0xFFU);
        CSL_REG_WR(&pTrngRegs->TRNG_FROENABLE, val);

        if(config->attrs->mode == RNG_DRBG_MODE)
        {
            /* Enable DRBG first */
            val = (((uint32_t) 1U) << RNG_CONTROL_DRBG_EN_SHIFT);
            CSL_REG_WR(&pTrngRegs->TRNG_CONTROL, val);

            /* Enable TRNG and DRBG  Section 5.2.5 */
            val = ((((uint32_t) 1U) << RNG_CONTROL_ENABLE_TRNG_SHIFT) |
                (((uint32_t) 1U) << RNG_CONTROL_DRBG_EN_SHIFT));
            CSL_REG_WR(&pTrngRegs->TRNG_CONTROL, val);

            /* Loop until the reseed_ai bit is 1 */
            do {
                val = CSL_REG_RD(&pTrngRegs->TRNG_STATUS);
            } while ((val & RNG_STATUS_RESEED_AI_MASK) != RNG_STATUS_RESEED_AI_MASK);

            /*
            * Write the personalization string (384 bits). This does not
            * need to be secret but must be unique. So we use the UID.
            * UID is only 256 bits long. So we cycle through the UID.
            */
            /* Not doing a verify as the register is write only */
            if((config->attrs->seedSizeInDwords != 0) && (config->attrs->seedSizeInDwords <= RNG_DRBG_SEED_MAX_ARRY_SIZE_IN_DWORD))
            {
                for(i = 0; i< config->attrs->seedSizeInDwords; i++)
                {
                    CSL_REG_WR(&pTrngRegs->TRNG_PS_AI[i], config->attrs->seedValue[i]);
                }
                if(config->attrs->seedSizeInDwords < RNG_DRBG_SEED_MAX_ARRY_SIZE_IN_DWORD)
                {
                    while(i < RNG_DRBG_SEED_MAX_ARRY_SIZE_IN_DWORD)
                    {
                        CSL_REG_WR(&pTrngRegs->TRNG_PS_AI[i], gRngDrbgDefaultSeed[i]);
                        i++;
                    }
                }
            }
            else
            {
                for(i = 0; i< RNG_DRBG_SEED_MAX_ARRY_SIZE_IN_DWORD; i++)
                {
                    CSL_REG_WR(&pTrngRegs->TRNG_PS_AI[i], gRngDrbgDefaultSeed[i]);
                }
            }
            /* Setting the engine to generate DRBG with requested bit */
            val = CSL_REG_RD(&pTrngRegs->TRNG_CONTROL);
            /* We always request the maximum number of blocks possible */
            updated_bits |= RNG_CONTROL_DATA_BLOCKS_MASK;
            /* Set the request data bit */
            updated_bits |= RNG_CONTROL_REQUEST_DATA_MASK;

            val |= updated_bits;
            CSL_REG_WR(&pTrngRegs->TRNG_CONTROL, val);
            retVal = RNG_RETURN_SUCCESS;
        }
        else
        {
            /* Start the actual engine by setting the TRNG_CONTROL[10] ENABLE_TRNG register bit*/
            val = ((((uint32_t) 1U) << CSL_CP_ACE_TRNG_CONTROL_ENABLE_TRNG_SHIFT));
            CSL_REG_WR(&pTrngRegs->TRNG_CONTROL, val);
            retVal = RNG_RETURN_SUCCESS;
        }
    }
    return (retVal);
}

RNG_Return_t RNG_read(RNG_Handle handle, uint32_t *out)
{
    RNG_Return_t retVal  = RNG_RETURN_FAILURE;
    uint32_t     val     = 0U, mask = 0U;
    uint32_t     ready   = 0U;
    RNG_Config   *config;
    CSL_Cp_aceTrngRegs  *pTrngRegs;

    if (NULL != handle)
    {
        config = (RNG_Config *)handle;
        pTrngRegs  = (CSL_Cp_aceTrngRegs *)config->attrs->rngBaseAddr;
        val = CSL_REG_RD(&pTrngRegs->TRNG_CONTROL);
        mask = ((((uint32_t) 1U) << CSL_CP_ACE_TRNG_CONTROL_ENABLE_TRNG_SHIFT));

        if ((val & mask) == mask)
        {
            retVal = RNG_RETURN_SUCCESS;
        }
    }
    else
    {
        retVal = RNG_RETURN_FAILURE;
        DebugP_assert(RNG_RETURN_SUCCESS == retVal);
    }

    if(RNG_RETURN_SUCCESS == retVal)
    {
        /* Check if random data is available */
        val = CSL_REG_RD(&pTrngRegs->TRNG_STATUS);
        ready =  (val & CSL_CP_ACE_TRNG_STATUS_READY_MASK) >> CSL_CP_ACE_TRNG_STATUS_READY_SHIFT;
        while (ready != 1U)
        {
            val = CSL_REG_RD(&pTrngRegs->TRNG_STATUS);
            ready =  (val & CSL_CP_ACE_TRNG_STATUS_READY_MASK) >> CSL_CP_ACE_TRNG_STATUS_READY_SHIFT;
        }
        /* If data is available, read it into the output buffer */
        out[0]  = CSL_REG_RD(&pTrngRegs->TRNG_INPUT_0);
        out[1]  = CSL_REG_RD(&pTrngRegs->TRNG_INPUT_1);
        out[2]  = CSL_REG_RD(&pTrngRegs->TRNG_INPUT_2);
        out[3]  = CSL_REG_RD(&pTrngRegs->TRNG_INPUT_3);

        /*Set the INTACK and go back*/
        CSL_REG_WR(&pTrngRegs->TRNG_STATUS, (CSL_CP_ACE_TRNG_INTACK_READY_ACK_MASK << CSL_CP_ACE_TRNG_INTACK_READY_ACK_SHIFT));
    }
    return (retVal);
}