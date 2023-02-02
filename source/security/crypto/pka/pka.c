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
 */

/**
 *  \file   pka.c
 *
 *  \brief  This file contains the implementation of PKA ( Ultra lite Security Accelerator)(Public Key Accelerator) driver
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <string.h>
#include <stddef.h>
#include <kernel/dpl/ClockP.h>
#include <security/crypto/pka/pka.h>
#include <drivers/hw_include/cslr.h>
#include <drivers/hw_include/cslr_soc.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define PKA_CLK_CTRL_REG_CLK_ON_SHIFT              (0)
#define PKA_CLK_CTRL_SEQ_CLK_ON_SHIFT              (1)
#define PKA_CLK_CTRL_PKCP_CLK_ON_SHIFT             (2)
#define PKA_CLK_CTRL_LNME_CLK_ON_SHIFT             (3)
#define PKA_CLK_CTRL_LNME_REG_CLK_ON_SHIFT         (4)
#define PKA_CLK_CTRL_GF2M_CLK_ON_SHIFT             (5)
#define PKA_CLK_CTRL_DATA_RAM_CLK_ON_SHIFT         (6)

#define PKA_CLK_CTRL_REG_CLK_EN_SHIFT              (16)
#define PKA_CLK_CTRL_SEQ_CLK_EN_SHIFT              (17)
#define PKA_CLK_CTRL_PKCP_CLK_EN_SHIFT             (18)
#define PKA_CLK_CTRL_LNME_CLK_EN_SHIFT             (19)
#define PKA_CLK_CTRL_LNME_REG_CLK_EN_SHIFT         (20)
#define PKA_CLK_CTRL_GF2M_CLK_EN_SHIFT             (21)
#define PKA_CLK_CTRL_DATA_RAM_CLK_EN_SHIFT         (22)

#define PKA_SEQ_CTRL_RESET_SHIFT                   (31)
#define PKA_SEQ_CTRL_RESULT_SHIFT                  (8)
#define PKA_SW_REV_FIRMWARE_VERSION_SHIFT          (16)
#define PKA_SEQ_CTRL_RESULT_MASK                   (0xffUL << 8)
#define PKA_SW_REV_FIRMWARE_VERSION_MASK           (0xfffUL << 16)

#define PKA_CLK_CTRL_REG_CLK_OFF_SHIFT             (8)
#define PKA_CLK_CTRL_SEQ_CLK_OFF_SHIFT             (9)
#define PKA_CLK_CTRL_PKCP_CLK_OFF_SHIFT            (10)
#define PKA_CLK_CTRL_LNME_CLK_OFF_SHIFT            (11)
#define PKA_CLK_CTRL_LNME_REG_CLK_OFF_SHIFT        (12)
#define PKA_CLK_CTRL_GF2M_CLK_OFF_SHIFT            (13)
#define PKA_CLK_CTRL_DATA_RAM_CLK_OFF_SHIFT        (14)

#define PKA_FUNCTION_COMPARE_SHIFT                 (10)
#define PKA_FUNCTION_RUN_SHIFT                     (15)
#define PKA_FUNCTION_RUN_MASK                      (1UL << 15)
#define PKA_FUNCTION_CMD_HI_SHIFT                  (16)
#define PKA_FUNCTION_CMD_LO_SHIFT                  (12)
#define PKA_FUNCTION_RUN_SHIFT                     (15)

#define PKA_COMPARE_A_LT_B_MASK                    (1UL << 1)

/** PKA RAM size (4kbytes) in words */
#define PKA_RAM_SIZE_WORDS                         (1024U)

#define PKA_SEQ_CTRL_DONE_MASK                     (1UL << 8)
#define PKA_SEQ_CTRL_DONE_SHIFT                    (8)

/** Result code when a PKA firmware command was successfUL */
#define PKA_COMMAND_RESULT_SUCCESS                 (0x1U)

/*
 * Timeout values in microsecs
 */
/**
 * Timeout for register updates to take effect - 10us
 */
#define PKA_REG_TIMEOUT                            (10U)

/**
 * Timeout for compare of 2 bignums - 100us
 */
#define PKA_COMPARE_TIMEOUT                        (100U)

/**
 * Timeout for modexp CRT operation - 50ms
 */
#define PKA_MODEXP_CRT_TIMEOUT                     (50000U)

/**
 * Timeout for modexp operation - 10ms
 */
#define PKA_MODEXP_TIMEOUT                         (10000U)

/**
 * Timeout for ECDSA verify operation - 10ms
 */
#define PKA_ECDSA_VERIFY_TIMEOUT                   (10000U)

/**
 * Timeout for ECDSA sign operation - 10ms
 */
#define PKA_ECDSA_SIGN_TIMEOUT                     (10000U)

/** Command to PKA firmware - MODEXP_CRT */
#define PKA_MODEXP_CRT_CMD   (((uint32_t) 0x0U) << PKA_FUNCTION_CMD_HI_SHIFT) | \
    (((uint32_t) 0x1U) << PKA_FUNCTION_CMD_LO_SHIFT)

/** Command to PKA firmware - MODEXP */
#define PKA_MODEXP_CMD   (((uint32_t) 0x0U) << PKA_FUNCTION_CMD_HI_SHIFT) |    \
    (((uint32_t) 0x6U) << PKA_FUNCTION_CMD_LO_SHIFT)

/** Command to PKA firmware - ECDSA SIGN */
#define PKA_ECDSA_SIGN_CMD   (((uint32_t) 0x2U) << PKA_FUNCTION_CMD_HI_SHIFT) | \
    (((uint32_t) 0x2U) << PKA_FUNCTION_CMD_LO_SHIFT)

/** Command to PKA firmware - ECDSA VERIFY */
#define PKA_ECDSA_VERIFY_CMD   (((uint32_t) 0x2U) << PKA_FUNCTION_CMD_HI_SHIFT) | \
    (((uint32_t) 0x3U) << PKA_FUNCTION_CMD_LO_SHIFT)

/** \brief device type HSSE */
#define DEVTYPE_HSSE         (0x0AU)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

static PKA_Return_t PKA_loadFirmware(PKA_Attrs *attrs, uint32_t inst);
static PKA_Return_t PKA_enable(PKA_Attrs *attrs, uint32_t inst);
static void PKA_disable(PKA_Attrs *attrs);
static void PKA_cpyz(volatile uint32_t dest[PKA_EC_BIGINT_MAX], uint32_t dest_len, const uint32_t bn[PKA_EC_BIGINT_MAX]);
static inline uint32_t PKA_dwAlign(uint32_t size);
static void PKA_delay(int32_t delayCount);
static uint32_t PKA_bigIntBitLen(const uint32_t bn[PKA_EC_BIGINT_MAX]);
static PKA_Return_t PKA_isBigIntZero(const uint32_t bn[PKA_BIGINT_MAX]);
static CSL_Eip_29t2_ramRegs* PKA_getBaseAddress(PKA_Attrs *attrs);
static void PKA_setALength(CSL_Eip_29t2_ramRegs *pka_regs, uint32_t size);
static void PKA_setBLength(CSL_Eip_29t2_ramRegs *pka_regs, uint32_t size);
static void PKA_setAPtr(CSL_Eip_29t2_ramRegs *pka_regs, uint32_t offset);
static void PKA_setBPtr(CSL_Eip_29t2_ramRegs *pka_regs, uint32_t offset);
static void PKA_setCPtr(CSL_Eip_29t2_ramRegs *pka_regs, uint32_t offset);
static void PKA_setDPtr(CSL_Eip_29t2_ramRegs *pka_regs, uint32_t offset);

#if defined(SOC_AM64X) || defined(SOC_AM243X)
static CSL_Cp_aceRegs* PKA_getCaBaseAddress(PKA_Attrs *attrs);
#endif /* defined(SOC_AM64X) || defined(SOC_AM243X) */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

static CSL_Eip_29t2_ramRegs* PKA_getBaseAddress(PKA_Attrs *attrs)
{
    CSL_Eip_29t2_ramRegs *pka_regs;
    pka_regs = (CSL_Eip_29t2_ramRegs *)attrs->pkaBaseAddr;
    return pka_regs;
}

#if defined(SOC_AM64X) || defined(SOC_AM243X)
static CSL_Cp_aceRegs* PKA_getCaBaseAddress(PKA_Attrs *attrs)
{
    CSL_Cp_aceRegs *pCaRegs = (CSL_Cp_aceRegs *)attrs->caBaseAddr;
    return pCaRegs;
}
#endif /* defined(SOC_AM64X) || defined(SOC_AM243X) */

static void PKA_setALength(CSL_Eip_29t2_ramRegs *pka_regs, uint32_t size)
{
    CSL_REG_WR(&pka_regs->EIP_27B_EIP27_REGISTERS.PKA_ALENGTH, size);

}

static void PKA_setBLength(CSL_Eip_29t2_ramRegs *pka_regs, uint32_t size)
{
    CSL_REG_WR(&pka_regs->EIP_27B_EIP27_REGISTERS.PKA_BLENGTH, size);
}

static void PKA_setAPtr(CSL_Eip_29t2_ramRegs *pka_regs, uint32_t offset)
{
    CSL_REG_WR(&pka_regs->EIP_27B_EIP27_REGISTERS.PKA_APTR, offset);
}

static void PKA_setBPtr(CSL_Eip_29t2_ramRegs *pka_regs, uint32_t offset)
{
    CSL_REG_WR(&pka_regs->EIP_27B_EIP27_REGISTERS.PKA_BPTR, offset);
}

static void PKA_setCPtr(CSL_Eip_29t2_ramRegs *pka_regs, uint32_t offset)
{
    CSL_REG_WR(&pka_regs->EIP_27B_EIP27_REGISTERS.PKA_CPTR, offset);
}

static void PKA_setDPtr(CSL_Eip_29t2_ramRegs *pka_regs, uint32_t offset)
{
    CSL_REG_WR(&pka_regs->EIP_27B_EIP27_REGISTERS.PKA_DPTR, offset);
}

PKA_Handle PKA_open(uint32_t index)
{
    uint32_t        status  = PKA_RETURN_SUCCESS;
    PKA_Handle      handle  = NULL;
    PKA_Config      *config = NULL;
    PKA_Attrs       *attrs  = NULL;
    /* Check instance */
    if(index >= gPkaConfigNum)
    {
        status = PKA_RETURN_FAILURE;
    }
    else
    {
        config = &gPkaConfig[index];
        DebugP_assert(NULL != config->attrs);
        attrs = config->attrs;
        if(TRUE == attrs->isOpen)
        {
            /* Handle is already opened */
            status = PKA_RETURN_FAILURE;
        }
        else
        {
            /* Enabling PKA clock and Load PKA Firmware */
            #if (((defined (SOC_AM263X) || defined (SOC_AM273X)) && !defined(__ARM_ARCH_7R__)) \
                             || defined (SOC_AM243X) \
                             || defined (SOC_AM64X))
            status = PKA_enable(config->attrs, index);
            #endif
        }
    }

    #if (((defined (SOC_AM263X) || defined (SOC_AM273X)) && defined(__ARM_ARCH_7R__)))
        CSL_top_ctrlRegs * ptrTopCtrlRegs = (CSL_top_ctrlRegs *)CSL_TOP_CTRL_U_BASE;
        if(ptrTopCtrlRegs->EFUSE_DEVICE_TYPE == DEVTYPE_HSSE)
        {
            status = PKA_RETURN_FAILURE;
        }
    #endif

    if(PKA_RETURN_SUCCESS == status)
    {
        attrs->isOpen = TRUE;
        handle = (PKA_Handle) config;
    }

    return (handle);
}

PKA_Return_t PKA_close(PKA_Handle handle)
{
    PKA_Return_t status  = PKA_RETURN_FAILURE;
    PKA_Config      *config;
    PKA_Attrs *attrs;
    config = (PKA_Config *) handle;

    if((NULL != config) && (config->attrs->isOpen != (uint32_t)FALSE))
    {
        attrs = config->attrs;
        DebugP_assert(NULL != attrs);
        PKA_disable(attrs);
        attrs->isOpen = FALSE;
        /* TO disable module*/
        handle = NULL;
        status  = PKA_RETURN_SUCCESS;
    }
    return (status);
}

PKA_Return_t PKA_RSAPrivate(PKA_Handle handle,
                    const uint32_t m[PKA_BIGINT_MAX],
                    const struct PKA_RSAPrivkey *k,
                    uint32_t result[PKA_BIGINT_MAX])
{
    PKA_Return_t status = PKA_RETURN_FAILURE;
    uint64_t curTimeInUsecs, totalTimeInUsecs = 0;
    uint32_t size, offset, reg, wssize, shift, tmp, numCount;
    CSL_Eip_29t2_ramRegs *pka_regs;
    PKA_Config      *config;
    PKA_Attrs *attrs;
    config  = (PKA_Config *) handle;
    attrs   = config->attrs;
    size = k->p[0];

    /* check sizes, sizes of s and n must match. */
    if ((!((size <= 1U) || (size > ((PKA_BIGINT_MAX - 1U) >> 1)) ||
           (k->q[0] > size) || (k->dp[0] > size) || (k->dq[0] > size) ||
           (k->coefficient[0] > size) || (m[0] > (size * 2U)))))
    {
        /* Checking handle is opened or not */
        if((attrs->isOpen) && (NULL != handle))
        {
            status = PKA_RETURN_SUCCESS;
        }
    }
    if(PKA_RETURN_SUCCESS == status)
    {
        pka_regs = PKA_getBaseAddress(attrs);

        PKA_setALength(pka_regs, size);
        PKA_setBLength(pka_regs, size);

        /* Vectors A has Dp[Alen], [pad], Dq[Alen] */
        offset = 0U;
        PKA_cpyz(&pka_regs->EIP_29T2_RAM_PKA_RAM.PKA_RAM[offset], size, k->dp);
        offset += PKA_dwAlign(size);
        PKA_cpyz(&pka_regs->EIP_29T2_RAM_PKA_RAM.PKA_RAM[offset], size, k->dq);
        offset += PKA_dwAlign(size);

        /* Vectors B has p[Blen], [1], [pad] q[Blen], [1] */
        PKA_setBPtr(pka_regs, offset);
        PKA_cpyz(&pka_regs->EIP_29T2_RAM_PKA_RAM.PKA_RAM[offset], size + 1U, k->p);
        offset += PKA_dwAlign(size + 1U);

        /* Temporarily set A offset to compare */
        PKA_setAPtr(pka_regs, offset);
        PKA_cpyz(&pka_regs->EIP_29T2_RAM_PKA_RAM.PKA_RAM[offset], size + 1U, k->q);
        offset += PKA_dwAlign(size + 1U);

        /* Compare and check if p > q */
        CSL_REG_WR(&pka_regs->EIP_27B_EIP27_REGISTERS.PKA_FUNCTION,
               (((uint32_t) 1U) << PKA_FUNCTION_COMPARE_SHIFT) |
               (((uint32_t) 1U) << PKA_FUNCTION_RUN_SHIFT));

        /* Wait for completion */
        curTimeInUsecs = ClockP_getTimeUsec();
        /* TODO: MCUSDK-8047: Timeout values need to be calibrated for SoCs an checks need to be added back */
        while(((PKA_FUNCTION_RUN_MASK & CSL_REG_RD(&pka_regs->EIP_27B_EIP27_REGISTERS.PKA_FUNCTION)) != 0U))
        {
            totalTimeInUsecs = ClockP_getTimeUsec() - curTimeInUsecs;
        }
        totalTimeInUsecs = 0;

        if(totalTimeInUsecs > PKA_COMPARE_TIMEOUT)
        {
            status = PKA_RETURN_FAILURE;
        }
        if(status == PKA_RETURN_SUCCESS)
        {
            reg = CSL_REG_RD(&pka_regs->EIP_27B_EIP27_REGISTERS.PKA_COMPARE);
            if ((reg & PKA_COMPARE_A_LT_B_MASK) != 0U)
            {
                /* Restore A offset to zero */
                PKA_setAPtr(pka_regs, 0U);

                /* Vectors C has qInv[Blen] */
                PKA_setCPtr(pka_regs, offset);
                PKA_cpyz(&pka_regs->EIP_29T2_RAM_PKA_RAM.PKA_RAM[offset],
                         size, k->coefficient);
                offset += PKA_dwAlign(size);

                /* Vectors D has M [2*Blen], [1], [pad]
                * WorkSpace */
                PKA_setDPtr(pka_regs, offset);
                PKA_cpyz(&pka_regs->EIP_29T2_RAM_PKA_RAM.PKA_RAM[offset],
                           size * 2U, m);

                /*
                 * Higher shift value will increase number of
                 * pre-computed odd powers and speed up
                 * exponentiation operation.
                 *
                 * Need workspace size of at least max of
                 * {3 x (BLen + 2 - (BLen MOD 2)) + 10,
                 * (shift + 1) x (BLen + 2 - (BLen MOD 2))} for
                 * computation.
                 *
                 * With PKA_RAM size of 4KB, shift value of 4
                 * will always fit, so latter expression will
                 * always be greater. Shift is determined based
                 * on that.
                 */

                /* Ger the free workspace size available for
                 * computation */

                wssize = PKA_RAM_SIZE_WORDS -
                     (offset + PKA_dwAlign((size * 2U) + 1U));
                tmp = size + (2U - (size & 1U));

                /* Compute best shift for fast computation */
                shift = (wssize / tmp) - 1U;

                /* Maximum shift value is 16 but restrict to 8
                 * max due to diminishing returns on performance
                 * for additional odd powers */
                if (shift > 8U)
                {
                    shift = 8U;
                }

                CSL_REG_WR(&pka_regs->EIP_27B_EIP27_REGISTERS.PKA_SHIFT, shift);

                CSL_REG_WR(&pka_regs->EIP_27B_EIP27_REGISTERS.PKA_FUNCTION, PKA_MODEXP_CRT_CMD |
                       (((uint32_t) 1U) << PKA_FUNCTION_RUN_SHIFT));

                /* Wait for completion */
                totalTimeInUsecs = 0;
                curTimeInUsecs = ClockP_getTimeUsec();
                /* TODO: MCUSDK-8047: Timeout values need to be calibrated for SoCs an checks need to be added back */
                while(((PKA_SEQ_CTRL_DONE_MASK & CSL_REG_RD(&pka_regs->EIP_28PX12_GF2_2PRAM_EIP28_REGISTERS.PKA_SEQ_CTRL)) != ((uint32_t) 1U) << PKA_SEQ_CTRL_DONE_SHIFT))
                {
                    totalTimeInUsecs = ClockP_getTimeUsec() - curTimeInUsecs;
                }

                totalTimeInUsecs = 0;

                if(totalTimeInUsecs > PKA_MODEXP_CRT_TIMEOUT)
                {
                    status = PKA_RETURN_FAILURE;
                }

                if(status == PKA_RETURN_SUCCESS)
                {
                    reg = CSL_REG_RD(&pka_regs->EIP_28PX12_GF2_2PRAM_EIP28_REGISTERS.PKA_SEQ_CTRL);
                    if ((reg & PKA_SEQ_CTRL_RESULT_MASK) == (PKA_COMMAND_RESULT_SUCCESS << PKA_SEQ_CTRL_RESULT_SHIFT))
                    {
                        /* Copy result vector from dptr
                         */
                        result[0] = size * 2U;

                        for(numCount = 0; numCount < (size * 2U); numCount++)
                        {
                            result[1 + numCount] = pka_regs->EIP_29T2_RAM_PKA_RAM.PKA_RAM[offset + numCount];
                        }

                        status = PKA_RETURN_SUCCESS;
                    }
                    else
                    {
                        status = PKA_RETURN_FAILURE;
                    }
                }
            }
            else
            {
                status = PKA_RETURN_FAILURE;
            }
        }
    }
    return (status);
}

PKA_Return_t PKA_RSAPublic(PKA_Handle handle,
                    const uint32_t m[PKA_BIGINT_MAX],
                    const struct PKA_RSAPubkey *k,
                    uint32_t result[PKA_BIGINT_MAX])
{
    PKA_Return_t status = PKA_RETURN_FAILURE;
    uint64_t curTimeInUsecs, totalTimeInUsecs = 0;
    uint32_t size, offset, reg, numCount;
    CSL_Eip_29t2_ramRegs *pka_regs;
    PKA_Config      *config;
    PKA_Attrs *attrs;
    config  = (PKA_Config *) handle;
    attrs   = config->attrs;

    size = k->n[0];

    /* check sizes, sizes of s and n must match. */
    if ((!((size <= 1U) || (size > (PKA_BIGINT_MAX - 1U)) ||
           (m[0] != size) || (k->e[0] > (PKA_BIGINT_MAX - 1U)))))
    {
        /* Checking handle is opened or not */
        if((attrs->isOpen) && (NULL != handle))
        {
            status = PKA_RETURN_SUCCESS;
        }
    }
    if(status == PKA_RETURN_SUCCESS)
    {
        pka_regs = PKA_getBaseAddress(attrs);

        offset = 0U;
        for(numCount = 0; numCount < (k->e[0]); numCount++)
        {
            pka_regs->EIP_29T2_RAM_PKA_RAM.PKA_RAM[offset + numCount] = k->e[1 + numCount];
        }

        offset += PKA_dwAlign(k->e[0]);
        PKA_setBPtr(pka_regs, offset);
        for(numCount = 0; numCount < (k->n[0]); numCount++)
        {
            pka_regs->EIP_29T2_RAM_PKA_RAM.PKA_RAM[offset + numCount] = k->n[1 + numCount];
        }
        PKA_setBLength(pka_regs, k->n[0]);

        /* Vectors B and C must be followed by an empty 32-bit buffer
         * word */
        offset += PKA_dwAlign(k->n[0] + 1U);

        PKA_setCPtr(pka_regs, offset);

        for(numCount = 0; numCount < m[0]; numCount++)
        {
            pka_regs->EIP_29T2_RAM_PKA_RAM.PKA_RAM[offset + numCount] = m[1 + numCount];
        }

        /* check if m is less than n. temporarily use Aptr and Asize */
        PKA_setAPtr(pka_regs, offset);

        PKA_setALength(pka_regs, m[0]);

        CSL_REG_WR(&pka_regs->EIP_27B_EIP27_REGISTERS.PKA_FUNCTION,
               (((uint32_t) 1U) << PKA_FUNCTION_COMPARE_SHIFT) |
               (((uint32_t) 1U) << PKA_FUNCTION_RUN_SHIFT));

        /* Wait for completion */
        curTimeInUsecs = ClockP_getTimeUsec();
        /* TODO: MCUSDK-8047: Timeout values need to be calibrated for SoCs an checks need to be added back */
        while(((PKA_FUNCTION_RUN_MASK & CSL_REG_RD(&pka_regs->EIP_27B_EIP27_REGISTERS.PKA_FUNCTION)) != 0U))
        {
            totalTimeInUsecs = ClockP_getTimeUsec() - curTimeInUsecs;
        }

        totalTimeInUsecs = 0;

        if(totalTimeInUsecs > PKA_COMPARE_TIMEOUT)
        {
            status = PKA_RETURN_FAILURE;
        }

        if (status == PKA_RETURN_SUCCESS)
        {
            reg = CSL_REG_RD(&pka_regs->EIP_27B_EIP27_REGISTERS.PKA_COMPARE);
            if ((reg & PKA_COMPARE_A_LT_B_MASK) != 0U)
            {
                /* Restore the value of Aptr and Aoffset to
                 *point to E */
                PKA_setAPtr(pka_regs, 0U);
                PKA_setALength(pka_regs, k->e[0]);

                /* To save PKA-RAM space, the MODEXP operation
                 * allows the input (M) to be located at the
                 * start of the workspace.
                 * So PKA_CPTR and PKA_DPTR  are allowed to be
                 * identical
                 */
                PKA_setDPtr(pka_regs, offset);
                CSL_REG_WR(&pka_regs->EIP_27B_EIP27_REGISTERS.PKA_SHIFT, 1U);

                CSL_REG_WR(&pka_regs->EIP_27B_EIP27_REGISTERS.PKA_FUNCTION, PKA_MODEXP_CMD |
                       (((uint32_t) 1U) << PKA_FUNCTION_RUN_SHIFT));

                /* Wait for completion */
                totalTimeInUsecs = 0;
                curTimeInUsecs = ClockP_getTimeUsec();
                /* TODO: MCUSDK-8047: Timeout values need to be calibrated for SoCs an checks need to be added back */
                while(((PKA_SEQ_CTRL_DONE_MASK & CSL_REG_RD(&pka_regs->EIP_28PX12_GF2_2PRAM_EIP28_REGISTERS.PKA_SEQ_CTRL)) != ((uint32_t) 1U) << PKA_SEQ_CTRL_DONE_SHIFT))
                {
                    totalTimeInUsecs = ClockP_getTimeUsec() - curTimeInUsecs;
                }

                totalTimeInUsecs = 0;

                if(totalTimeInUsecs > PKA_MODEXP_TIMEOUT)
                {
                    status = PKA_RETURN_FAILURE;
                }

                if (status == PKA_RETURN_SUCCESS)
                {
                    reg = CSL_REG_RD(&pka_regs->EIP_28PX12_GF2_2PRAM_EIP28_REGISTERS.PKA_SEQ_CTRL);
                    if ((reg & PKA_SEQ_CTRL_RESULT_MASK) ==
                        (PKA_COMMAND_RESULT_SUCCESS <<
                         PKA_SEQ_CTRL_RESULT_SHIFT))
                    {
                        /*
                         * Success
                         * Copy result vector from dptr
                         */
                        result[0] = size;
                        for(numCount = 0; numCount < size; numCount++)
                        {
                            result[1 + numCount] = pka_regs->EIP_29T2_RAM_PKA_RAM.PKA_RAM[offset + numCount];
                        }
                        status = PKA_RETURN_SUCCESS;
                    }
                    else
                    {
                        status = PKA_RETURN_FAILURE;
                    }
                }
            }
            else
            {
                status = PKA_RETURN_FAILURE;
            }
        }
    }
    return (status);
}

PKA_Return_t PKA_ECDSASign(PKA_Handle handle,
                        const struct PKA_ECPrimeCurveP *cp,
                        const uint32_t priv[PKA_EC_BIGINT_MAX],
                        const uint32_t k[PKA_EC_BIGINT_MAX],
                        const uint32_t h[PKA_EC_BIGINT_MAX],
                        struct PKA_ECDSASig *sig)
{
    PKA_Return_t status = PKA_RETURN_FAILURE;
    uint64_t curTimeInUsecs, totalTimeInUsecs = 0;
    uint32_t offset, reg, size, numCount;
    uint32_t bn_one[2] = { 1U, 1U };
    CSL_Eip_29t2_ramRegs *pka_regs;
    PKA_Config      *config;
    PKA_Attrs *attrs;
    config  = (PKA_Config *) handle;
    attrs   = config->attrs;

    size = cp->prime[0];

    /* check sizes */
    if ((!((size <= 2U) || (size > (PKA_EC_BIGINT_MAX - 1U)) ||
           (size != cp->order[0]) || (size < cp->a[0]) ||
           (size < cp->b[0]) || (size < cp->g.x[0]) ||
           (size < cp->g.y[0]) || (size < priv[0]) ||
           (size < h[0]) || (size < k[0]))) &&
           (PKA_bigIntBitLen(cp->order) >= PKA_bigIntBitLen(h)))
    {
        /* Checking handle is opened or not */
        if((attrs->isOpen) && (NULL != handle))
        {
            status = PKA_RETURN_SUCCESS;
        }
    }
    if(status == PKA_RETURN_SUCCESS)
    {
        pka_regs = PKA_getBaseAddress(attrs);

        PKA_setALength(pka_regs, size);
        PKA_setBLength(pka_regs, size);

        offset = 0U;
        /* Vector B has p, a, b, gz, gy and Rz (must be 1) */
        PKA_setBPtr(pka_regs, offset);
        PKA_cpyz(&pka_regs->EIP_29T2_RAM_PKA_RAM.PKA_RAM[offset], size + 2U, cp->prime);
        offset += PKA_dwAlign(size + 2U);

        PKA_cpyz(&pka_regs->EIP_29T2_RAM_PKA_RAM.PKA_RAM[offset], size + 2U, cp->a);
        offset += PKA_dwAlign(size + 2U);

        PKA_cpyz(&pka_regs->EIP_29T2_RAM_PKA_RAM.PKA_RAM[offset], size + 2U, cp->b);
        offset += PKA_dwAlign(size + 2U);

        PKA_cpyz(&pka_regs->EIP_29T2_RAM_PKA_RAM.PKA_RAM[offset], size + 2U, cp->order);
        offset += PKA_dwAlign(size + 2U);

        PKA_cpyz(&pka_regs->EIP_29T2_RAM_PKA_RAM.PKA_RAM[offset], size + 2U, cp->g.x);
        offset += PKA_dwAlign(size + 2U);

        PKA_cpyz(&pka_regs->EIP_29T2_RAM_PKA_RAM.PKA_RAM[offset], size + 2U, cp->g.y);
        offset += PKA_dwAlign(size + 2U);

        PKA_cpyz(&pka_regs->EIP_29T2_RAM_PKA_RAM.PKA_RAM[offset], size + 2U, bn_one);
        offset += PKA_dwAlign(size + 2U);

        /* Vector C has h */
        PKA_setCPtr(pka_regs, offset);
        PKA_cpyz(&pka_regs->EIP_29T2_RAM_PKA_RAM.PKA_RAM[offset], size + 2U, h);
        offset += PKA_dwAlign(size + 2U);

        PKA_setAPtr(pka_regs, offset);
        PKA_cpyz(&pka_regs->EIP_29T2_RAM_PKA_RAM.PKA_RAM[offset], size + 2U, priv);
        offset += PKA_dwAlign(size + 2U);

        PKA_setDPtr(pka_regs, offset);
        PKA_cpyz(&pka_regs->EIP_29T2_RAM_PKA_RAM.PKA_RAM[offset], size + 2U, k);

        CSL_REG_WR(&pka_regs->EIP_27B_EIP27_REGISTERS.PKA_FUNCTION, PKA_ECDSA_SIGN_CMD | (((uint32_t) 1U) << PKA_FUNCTION_RUN_SHIFT));

        /* Wait for completion */
        curTimeInUsecs = ClockP_getTimeUsec();
        /* TODO: MCUSDK-8047: Timeout values need to be calibrated for SoCs an checks need to be added back */
        while(((PKA_SEQ_CTRL_DONE_MASK & CSL_REG_RD(&pka_regs->EIP_28PX12_GF2_2PRAM_EIP28_REGISTERS.PKA_SEQ_CTRL)) != ((uint32_t) 1U) << PKA_SEQ_CTRL_DONE_SHIFT) )
        {
            totalTimeInUsecs = ClockP_getTimeUsec() - curTimeInUsecs;
        }

        totalTimeInUsecs = 0;

        if(totalTimeInUsecs > PKA_ECDSA_SIGN_TIMEOUT)
        {
            status = PKA_RETURN_FAILURE;
        }

        if(status == PKA_RETURN_SUCCESS)
        {
            reg = CSL_REG_RD(&pka_regs->EIP_28PX12_GF2_2PRAM_EIP28_REGISTERS.PKA_SEQ_CTRL);
            if((reg & PKA_SEQ_CTRL_RESULT_MASK) == (PKA_COMMAND_RESULT_SUCCESS << PKA_SEQ_CTRL_RESULT_SHIFT))
            {
                sig->r[0] = size;
                for(numCount = 0; numCount < size; numCount++)
                {
                    sig->r[1 + numCount] = pka_regs->EIP_29T2_RAM_PKA_RAM.PKA_RAM[offset + numCount];
                }
                offset += PKA_dwAlign(size + 2U);
                sig->s[0] = size;
                for(numCount = 0; numCount < size; numCount++)
                {
                    sig->s[1 + numCount] = pka_regs->EIP_29T2_RAM_PKA_RAM.PKA_RAM[offset + numCount];
                }
                status = PKA_RETURN_SUCCESS;
            }
            else
            {
                status = PKA_RETURN_FAILURE;
            }
        }
    }
    return (status);
}

PKA_Return_t PKA_ECDSAVerify(PKA_Handle handle,
                        const struct PKA_ECPrimeCurveP *cp,
                        const struct PKA_ECPoint *pub,
                        const struct PKA_ECDSASig *sig,
                        const uint32_t h[PKA_EC_BIGINT_MAX])
{
    PKA_Return_t status = PKA_RETURN_FAILURE;
    uint64_t curTimeInUsecs, totalTimeInUsecs = 0;
    uint32_t offset, reg, size;
    uint32_t bn_one[2] = { 1U, 1U };
    CSL_Eip_29t2_ramRegs *pka_regs;
    PKA_Config      *config;
    PKA_Attrs *attrs;
    config  = (PKA_Config *) handle;
    attrs   = config->attrs;

    size = cp->prime[0];

    /* check sizes */
    if ((!((size <= 2U) || (size > (PKA_EC_BIGINT_MAX - 1U)) ||
           (size != cp->order[0]) || (size < cp->a[0]) ||
           (size < cp->b[0]) || (size < cp->g.x[0]) ||
           (size < cp->g.y[0]) || (size < pub->x[0]) ||
           (size < pub->y[0]) || (size < sig->r[0]) ||
           (size < sig->s[0]) || (size < h[0]))) &&
            (PKA_bigIntBitLen(cp->order) >= PKA_bigIntBitLen(h)) &&
            PKA_isBigIntZero(sig->r) && PKA_isBigIntZero(sig->s))
    {
        /* Checking handle is opened or not */
        if((attrs->isOpen) && (NULL != handle))
        {
            status = PKA_RETURN_SUCCESS;
        }
        if(status == PKA_RETURN_SUCCESS)
        {
            pka_regs = PKA_getBaseAddress(attrs);

            PKA_setALength(pka_regs, size);
            PKA_setBLength(pka_regs, size);

            offset = 0;
            /* Vector B has p, a, b, gz, gy and Rz (must be 1) */
            PKA_setBPtr(pka_regs, offset);
            PKA_cpyz(&pka_regs->EIP_29T2_RAM_PKA_RAM.PKA_RAM[offset], size + 2U, cp->prime);
            offset += PKA_dwAlign(size + 2U);

            PKA_cpyz(&pka_regs->EIP_29T2_RAM_PKA_RAM.PKA_RAM[offset], size + 2U, cp->a);
            offset += PKA_dwAlign(size + 2U);

            PKA_cpyz(&pka_regs->EIP_29T2_RAM_PKA_RAM.PKA_RAM[offset], size + 2U, cp->b);
            offset += PKA_dwAlign(size + 2U);

            PKA_cpyz(&pka_regs->EIP_29T2_RAM_PKA_RAM.PKA_RAM[offset], size + 2U, cp->order);
            offset += PKA_dwAlign(size + 2U);

            PKA_cpyz(&pka_regs->EIP_29T2_RAM_PKA_RAM.PKA_RAM[offset], size + 2U, cp->g.x);
            offset += PKA_dwAlign(size + 2U);

            PKA_cpyz(&pka_regs->EIP_29T2_RAM_PKA_RAM.PKA_RAM[offset], size + 2U, cp->g.y);
            offset += PKA_dwAlign(size + 2U);

            PKA_cpyz(&pka_regs->EIP_29T2_RAM_PKA_RAM.PKA_RAM[offset], size + 2U, bn_one);
            offset += PKA_dwAlign(size + 2U);

            /* Vector C has h */
            PKA_setCPtr(pka_regs, offset);
            PKA_cpyz(&pka_regs->EIP_29T2_RAM_PKA_RAM.PKA_RAM[offset], size + 2U, h);
            offset += PKA_dwAlign(size + 2U);

            /* Vector A has px, py and R'z (must be 1) */
            PKA_setAPtr(pka_regs, offset);
            PKA_cpyz(&pka_regs->EIP_29T2_RAM_PKA_RAM.PKA_RAM[offset], size + 2U, pub->x);
            offset += PKA_dwAlign(size + 2U);

            PKA_cpyz(&pka_regs->EIP_29T2_RAM_PKA_RAM.PKA_RAM[offset], size + 2U, pub->y);
            offset += PKA_dwAlign(size + 2U);

            PKA_cpyz(&pka_regs->EIP_29T2_RAM_PKA_RAM.PKA_RAM[offset], size + 2U, bn_one);
            offset += PKA_dwAlign(size + 2U);

            PKA_setDPtr(pka_regs, offset);
            PKA_cpyz(&pka_regs->EIP_29T2_RAM_PKA_RAM.PKA_RAM[offset], size + 2U, sig->r);
            offset += PKA_dwAlign(size + 2U);

            PKA_cpyz(&pka_regs->EIP_29T2_RAM_PKA_RAM.PKA_RAM[offset], size + 2U, sig->s);

            CSL_REG_WR(&pka_regs->EIP_27B_EIP27_REGISTERS.PKA_FUNCTION, PKA_ECDSA_VERIFY_CMD | (((uint32_t) 1U) << PKA_FUNCTION_RUN_SHIFT));

            /* Wait for completion */
            curTimeInUsecs = ClockP_getTimeUsec();
            /* TODO: MCUSDK-8047: Timeout values need to be calibrated for SoCs an checks need to be added back */
            while(((PKA_SEQ_CTRL_DONE_MASK & CSL_REG_RD(&pka_regs->EIP_28PX12_GF2_2PRAM_EIP28_REGISTERS.PKA_SEQ_CTRL)) != ((uint32_t) 1U) << PKA_SEQ_CTRL_DONE_SHIFT) )
            {
                totalTimeInUsecs = ClockP_getTimeUsec() - curTimeInUsecs;
            }

            totalTimeInUsecs = 0;

            if(totalTimeInUsecs > PKA_ECDSA_VERIFY_TIMEOUT)
            {
                status = PKA_RETURN_FAILURE;
            }
            else
            {
                reg = CSL_REG_RD(&pka_regs->EIP_28PX12_GF2_2PRAM_EIP28_REGISTERS.PKA_SEQ_CTRL);
                if((reg & PKA_SEQ_CTRL_RESULT_MASK) == (PKA_COMMAND_RESULT_SUCCESS << PKA_SEQ_CTRL_RESULT_SHIFT))
                {
                    status = PKA_RETURN_SUCCESS;
                }
                else
                {
                    status = PKA_RETURN_FAILURE;
                }
            }
        }
    }
    return (status);
}

static PKA_Return_t PKA_enable(PKA_Attrs *attrs, uint32_t inst)
{
    PKA_Return_t status = PKA_RETURN_SUCCESS;
    uint32_t reg;
    uint64_t curTimeInUsecs, totalTimeInUsecs = 0;

    /* PKA Base address */
    CSL_Eip_29t2_ramRegs *pka_regs = PKA_getBaseAddress(attrs);

    /* Engine enable registers are available only in am64x and am243x */
#if defined(SOC_AM64X) || defined(SOC_AM243X)
    /* Crypto Accelerator Base Adders*/
    CSL_Cp_aceRegs *pCaRegs = PKA_getCaBaseAddress(attrs);

    /* Enable PKA engine modules */
    reg = CSL_REG_RD(&pCaRegs->UPDATES.ENGINE_ENABLE);

    CSL_FINS(reg, CP_ACE_UPDATES_ENGINE_ENABLE_PKA_EN, 1u);

    CSL_REG_WR(&pCaRegs->UPDATES.ENGINE_ENABLE, reg);

    reg = CSL_CP_ACE_CMD_STATUS_PKA_EN_MASK;

    /* Wait for PKA engine to be Enable */
    curTimeInUsecs = ClockP_getTimeUsec();
    /* TODO: MCUSDK-8047: Timeout values need to be calibrated for SoCs an
     * checks need to be added back */
    while(((reg & CSL_REG_RD(&pCaRegs->MMR.CMD_STATUS)) != reg))
    {
        totalTimeInUsecs = ClockP_getTimeUsec() - curTimeInUsecs;
    }

    totalTimeInUsecs = 0;

    if(totalTimeInUsecs > PKA_REG_TIMEOUT)
    {
        status = PKA_RETURN_FAILURE;
    }
#endif
    if(PKA_RETURN_SUCCESS == status)
    {
        CSL_REG_WR(&pka_regs->EIP_29T2_RAM_HOST_REGISTERS.PKA_CLK_CTRL,
            (((uint32_t) 1U) << PKA_CLK_CTRL_REG_CLK_ON_SHIFT) |
            (((uint32_t) 1U) << PKA_CLK_CTRL_SEQ_CLK_ON_SHIFT) |
            (((uint32_t) 1U) << PKA_CLK_CTRL_PKCP_CLK_ON_SHIFT) |
            (((uint32_t) 1U) << PKA_CLK_CTRL_LNME_CLK_ON_SHIFT) |
            (((uint32_t) 1U) << PKA_CLK_CTRL_LNME_REG_CLK_ON_SHIFT) |
            (((uint32_t) 1U) << PKA_CLK_CTRL_GF2M_CLK_ON_SHIFT) |
            (((uint32_t) 1U) << PKA_CLK_CTRL_DATA_RAM_CLK_ON_SHIFT));

        reg = (((uint32_t) 1U) << PKA_CLK_CTRL_REG_CLK_EN_SHIFT) |
            (((uint32_t) 1U) << PKA_CLK_CTRL_SEQ_CLK_EN_SHIFT) |
            (((uint32_t) 1U) << PKA_CLK_CTRL_PKCP_CLK_EN_SHIFT) |
            (((uint32_t) 1U) << PKA_CLK_CTRL_LNME_CLK_EN_SHIFT) |
            (((uint32_t) 1U) << PKA_CLK_CTRL_LNME_REG_CLK_EN_SHIFT) |
            (((uint32_t) 1U) << PKA_CLK_CTRL_GF2M_CLK_EN_SHIFT) |
            (((uint32_t) 1U) << PKA_CLK_CTRL_DATA_RAM_CLK_EN_SHIFT);

        /* Wait for PKA internal clocks to be active */
        totalTimeInUsecs = 0;
        curTimeInUsecs = ClockP_getTimeUsec();
        /* TODO: MCUSDK-8047: Timeout values need to be calibrated for SoCs an checks need to be added back */
        while(((reg & CSL_REG_RD(&pka_regs->EIP_29T2_RAM_HOST_REGISTERS.PKA_CLK_CTRL)) != reg))
        {
            totalTimeInUsecs = ClockP_getTimeUsec() - curTimeInUsecs;
        }

        totalTimeInUsecs = 0;

        if(totalTimeInUsecs > PKA_REG_TIMEOUT)
        {
            status = PKA_RETURN_FAILURE;
        }
    }
    if(PKA_RETURN_SUCCESS == status)
    {
        status = PKA_loadFirmware(attrs, inst);
    }
    if(PKA_RETURN_SUCCESS != status)
    {
        PKA_disable(attrs);
        /*
         * If firmware is already marked as loaded, check the PKA internal
         * clocks are active as a sanity check.
         */
        reg = (((uint32_t) 1U) << PKA_CLK_CTRL_REG_CLK_EN_SHIFT) |
              (((uint32_t) 1U) << PKA_CLK_CTRL_SEQ_CLK_EN_SHIFT) |
              (((uint32_t) 1U) << PKA_CLK_CTRL_PKCP_CLK_EN_SHIFT) |
              (((uint32_t) 1U) << PKA_CLK_CTRL_LNME_CLK_EN_SHIFT) |
              (((uint32_t) 1U) << PKA_CLK_CTRL_LNME_REG_CLK_EN_SHIFT) |
              (((uint32_t) 1U) << PKA_CLK_CTRL_GF2M_CLK_EN_SHIFT) |
              (((uint32_t) 1U) << PKA_CLK_CTRL_DATA_RAM_CLK_EN_SHIFT);
        /* Wait for PKA internal clocks to be active */
        totalTimeInUsecs = 0;
        curTimeInUsecs = ClockP_getTimeUsec();
        /* TODO: MCUSDK-8047: Timeout values need to be calibrated for SoCs an checks need to be added back */
        while(((reg & CSL_REG_RD(&pka_regs->EIP_29T2_RAM_HOST_REGISTERS.PKA_CLK_CTRL)) != reg))
        {
            totalTimeInUsecs = ClockP_getTimeUsec() - curTimeInUsecs;
        }

        totalTimeInUsecs = 0;

        if(totalTimeInUsecs > PKA_REG_TIMEOUT)
        {
            status = PKA_RETURN_FAILURE;
        }
    }

    return (status);
}

static PKA_Return_t PKA_loadFirmware(PKA_Attrs *attrs, uint32_t inst)
{
    volatile int32_t i;
    PKA_Return_t status = PKA_RETURN_FAILURE;

    CSL_Eip_29t2_ramRegs *pka_regs = PKA_getBaseAddress(attrs);

    /* Put EIP-29t2 (PKA) in reset, set bit 31 of PKA_SEQ_CTRL */
    CSL_REG_WR(&pka_regs->EIP_28PX12_GF2_2PRAM_EIP28_REGISTERS.PKA_SEQ_CTRL, ((uint32_t) 1U) << PKA_SEQ_CTRL_RESET_SHIFT);

    /*
     * Copy firmware to PKA program memory, which is aliased at the PKA_RAM
     * location when then PKA is in reset
     */
    for (i = 0; i < EIP29T2_FW_IMAGE_LEN_WORDS; i++) {
        pka_regs->EIP_29T2_RAM_PKA_RAM.PKA_RAM[i] = eip29t2_fw_image[i];
    }

    /* Take EIP-29t2 (PKA) out of reset */
    CSL_REG_WR(&pka_regs->EIP_28PX12_GF2_2PRAM_EIP28_REGISTERS.PKA_SEQ_CTRL, ((uint32_t) 0U) << PKA_SEQ_CTRL_RESET_SHIFT);

    /* Wait for a few cycles */
    PKA_delay(10U);

    /* Check status in PKA_SEQ_CTRL */
    if((CSL_REG_RD(&pka_regs->EIP_28PX12_GF2_2PRAM_EIP28_REGISTERS.PKA_SEQ_CTRL) & PKA_SEQ_CTRL_RESULT_MASK) ==  ((uint32_t)1U) << PKA_SEQ_CTRL_RESULT_SHIFT)
    {   /* Put EIP-29t2 (PKA) back into reset */
        CSL_REG_WR(&pka_regs->EIP_28PX12_GF2_2PRAM_EIP28_REGISTERS.PKA_SEQ_CTRL, ((uint32_t) 1U) << PKA_SEQ_CTRL_RESET_SHIFT);

        /* Verify the firmware content in the PKA program RAM */
        for (i = 0; i < EIP29T2_FW_IMAGE_LEN_WORDS; i++)
        {
            if (pka_regs->EIP_29T2_RAM_PKA_RAM.PKA_RAM[i] != eip29t2_fw_image[i])
            {
                break;
            }
        }
        if (i == EIP29T2_FW_IMAGE_LEN_WORDS)
        {
            /* Take EIP-29t2 (PKA) out of reset */
            CSL_REG_WR(&pka_regs->EIP_28PX12_GF2_2PRAM_EIP28_REGISTERS.PKA_SEQ_CTRL,
                   ((uint32_t) 0U) << PKA_SEQ_CTRL_RESET_SHIFT);

            /* Wait for a few cycles */
            PKA_delay(10U);

            /* Check status in PKA_SEQ_CTRL */
            if ((CSL_REG_RD(&pka_regs->EIP_28PX12_GF2_2PRAM_EIP28_REGISTERS.PKA_SEQ_CTRL) & PKA_SEQ_CTRL_RESULT_MASK) == (((uint32_t) 1U) << PKA_SEQ_CTRL_RESULT_SHIFT))
            {
                /* Wait for a few cycles */
                PKA_delay(50U);

                /* Check the firmware revision, 3 nibbles of
                 * major, minor
                 * and patch level are located from bits 27:16
                 **/
                if ((CSL_REG_RD(&pka_regs->EIP_28PX12_GF2_2PRAM_EIP28_REGISTERS.PKA_SW_REV) & PKA_SW_REV_FIRMWARE_VERSION_MASK) == (EIP29T2_FW_VERSION << PKA_SW_REV_FIRMWARE_VERSION_SHIFT))
                {
                    status = PKA_RETURN_SUCCESS;
                }
            }
        }
    }
    return (status);
}

static void PKA_disable(PKA_Attrs *attrs)
{
    /* Engine enable registers are available only in am64x and am243x */
#if defined(SOC_AM64X) || defined(SOC_AM243X)
    uint32_t reg;
#endif
    /* PKA Base address */
    CSL_Eip_29t2_ramRegs *pka_regs = PKA_getBaseAddress(attrs);

    /* Engine enable registers are available only in am64x and am243x */
#if defined(SOC_AM64X) || defined(SOC_AM243X)
    /* Crypto Accelerator Base Adders*/
    CSL_Cp_aceRegs *pCaRegs = PKA_getCaBaseAddress(attrs);

    reg = CSL_REG_RD(&pCaRegs->UPDATES.ENGINE_ENABLE);

    /* Disable PKA Engine */
    CSL_FINS(reg, CP_ACE_UPDATES_ENGINE_ENABLE_PKA_EN, 0u);

    CSL_REG_WR(&pCaRegs->UPDATES.ENGINE_ENABLE, reg);
#endif

#if (((defined (SOC_AM263X) || defined (SOC_AM273X)) && !defined(__ARM_ARCH_7R__)) \
                             || defined (SOC_AM243X) \
                             || defined (SOC_AM64X))
    CSL_REG_WR(&pka_regs->EIP_29T2_RAM_HOST_REGISTERS.PKA_CLK_CTRL,
        (((uint32_t) 1U) << PKA_CLK_CTRL_REG_CLK_OFF_SHIFT) |
        (((uint32_t) 1U) << PKA_CLK_CTRL_SEQ_CLK_OFF_SHIFT) |
        (((uint32_t) 1U) << PKA_CLK_CTRL_PKCP_CLK_OFF_SHIFT) |
        (((uint32_t) 1U) << PKA_CLK_CTRL_LNME_CLK_OFF_SHIFT) |
        (((uint32_t) 1U) << PKA_CLK_CTRL_LNME_REG_CLK_OFF_SHIFT) |
        (((uint32_t) 1U) << PKA_CLK_CTRL_GF2M_CLK_OFF_SHIFT) |
        (((uint32_t) 1U) << PKA_CLK_CTRL_DATA_RAM_CLK_OFF_SHIFT));
#else
    (void) pka_regs;
#endif
    return;
}

static void PKA_delay(int32_t delayCount)
{
    volatile int32_t tempDelay = delayCount;

    while(tempDelay != 0UL)
    {
        tempDelay = tempDelay -1U;
    }
    return;
}

/**
 * \brief Copy to destination with zero padding if necessary
 *
 * Copy with zero padding, useful to copy bigint operands to PKA RAM
 *
 * \param dest Destination address to copy to
 * \param dest_len Length of the destination buffer
 * \param bn Bigint to copy from
 */
static void PKA_cpyz(volatile uint32_t dest[PKA_EC_BIGINT_MAX],
               uint32_t dest_len, const uint32_t bn[PKA_EC_BIGINT_MAX])
{
    uint32_t i;

    for (i = 0U; (i < bn[0]) && (i < dest_len); i++) {
        dest[i] = bn[i + 1U];
    }
    for (; i < dest_len; i++) {
        dest[i] = 0U;
    }
    return;
}

/** Get a double-word aligned number for bigints used by PKA */
static inline uint32_t PKA_dwAlign(uint32_t size)
{
    return (size + (size & 1U));
}

uint32_t PKA_countLeadingZeros(uint32_t x)
{
    uint32_t bit_count = 0, lz = 0;

    bit_count = sizeof(x)*8;

    /* Left shift until Most significant bit doesn become 1 */

    while( (x & (1 << (bit_count - 1))) == 0)
    {
        x <<= 1;
        lz++;
    }

    return (lz);
}

/**
 * \brief Return the size in bits of a bigint
 *
 * \param bn Input number
 *
 * \return Length in bits of the big number
 */
static uint32_t PKA_bigIntBitLen(const uint32_t bn[PKA_EC_BIGINT_MAX])
{
    uint32_t i, status;

    for (i = bn[0]; i > 0U; i--)
    {
        if (bn[i] != 0U)
        {
            break;
        }
    }
    if (i == 0U)
    {
        status = 0U;
    }
    else
    {
        status = (i * 32U) - PKA_countLeadingZeros((int32_t) bn[i]);
    }
    return (status);
}

/**
 * \brief Check if the bigint is zero
 *
 * \param bn Input number
 *
 * \return PKA_RETURN_SUCCESS if the number if zero
 */
static PKA_Return_t PKA_isBigIntZero(const uint32_t bn[PKA_BIGINT_MAX])
{
    uint32_t i;
    PKA_Return_t ret = PKA_RETURN_SUCCESS;

    for (i = 0U; i <= bn[0]; i++)
    {
        if (bn[i] != 0U)
        {
            ret = PKA_RETURN_FAILURE;
            break;
        }
    }
    return (ret);
}
