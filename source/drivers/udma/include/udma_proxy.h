/*
 *  Copyright (c) 2024 Texas Instruments Incorporated
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
 *  \ingroup DRV_UDMA_MODULE
 *  \defgroup DRV_UDMA_PROXY_MODULE UDMA Driver Proxy API
 *            This is UDMA driver proxy related configuration parameters and
 *            API
 *
 *  @{
 */

/**
 *  \file udma_proxy.h
 *
 *  \brief UDMA proxy related parameters and API.
 */

#ifndef UDMA_PROXY_H_
#define UDMA_PROXY_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/**
 * \brief Macro used to specify that proxy ID is invalid.
 */
#define UDMA_PROXY_INVALID              ((uint16_t) 0xFFFFU)
/**
 * \brief Macro used to specify any available free proxy while requesting
 * one. Used in the API #Udma_proxyAlloc.
 */
#define UDMA_PROXY_ANY                  ((uint16_t) 0xFFFEU)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 *  \brief This structure contains configuration parameters for each proxy
 *  thread
 */
typedef struct
{
    uint32_t        proxyMode;
    /**< Initial queue access mode (see CSL_ProxyQueueAccessMode) */
    uint32_t        elemSize;
    /**< Ring element size.
     *   Refer \ref Udma_RingElemSize for supported values. */
    uint16_t        ringNum;
    /**< Ring number in the target to use for the proxy thread */
} Udma_ProxyCfg;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 *  \brief API to write 64-bit data to proxy
 *
 *  Requirement: DOX_REQ_TAG(PDK-4156)
 *
 *  \param proxyAddr    [IN] Proxy data address (pre-calculated for a 64-bit
 *                           write).
 *  \param data         [IN] 64-bit data to write.
 */
static inline void Udma_proxyWrite64(uintptr_t proxyAddr, uint64_t data);

/**
 *  \brief API to read 64-bit data from proxy
 *
 *  Requirement: DOX_REQ_TAG(PDK-4156)
 *
 *  \param proxyAddr    [IN] Proxy data address (pre-calculated for a 64-bit
 *                           read).
 *  \param data         [OUT] Pointer to 64-bit word where the data is
 *                            read to.
 */
static inline void Udma_proxyRead64(uintptr_t proxyAddr, uint64_t *data);

/* ========================================================================== */
/*                  Internal/Private Structure Declarations                   */
/* ========================================================================== */

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

static inline void Udma_proxyWrite64(uintptr_t proxyAddr, uint64_t data)
{
#if defined (__aarch64__) || defined (__C7100__)
    CSL_REG64_WR((uint64_t *) proxyAddr, data);
#else
    /* For 32-bit cores enforce the order as the compiler may not guarantee
     * the order */
    volatile uint32_t  *proxyAddr32 = (volatile uint32_t *) proxyAddr;
    uint32_t            wordLow, wordHigh;

    /* Write low word first and then the high word which triggers the actual
     *  write from proxy */
    wordLow = (uint32_t) data;
    wordHigh = (uint32_t) (data >> 32U);
    CSL_REG32_WR(proxyAddr32, wordLow);
    CSL_REG32_WR(proxyAddr32 + 1U, wordHigh);
#endif

    return;
}

static inline void Udma_proxyRead64(uintptr_t proxyAddr, uint64_t *data)
{
#if defined (__aarch64__) || defined (__C7100__)
    *data = CSL_REG64_RD((uint64_t *) proxyAddr);
#else
    /* For 32-bit cores enforce the order as the compiler may not guarantee
     * the order */
    volatile uint32_t  *proxyAddr32 = (volatile uint32_t *) proxyAddr;
    uint32_t            wordLow, wordHigh;

    /* Read low word first (this triggers the entire proxy read) and then the
     * high word which ends the proxy read */
    wordLow = CSL_REG32_RD(proxyAddr32);
    wordHigh = CSL_REG32_RD(proxyAddr32 + 1U);
    *data = ((uint64_t) wordLow) | (((uint64_t) wordHigh) << 32U);
#endif

    return;
}

#ifdef __cplusplus
}
#endif

#endif /* #ifndef UDMA_PROXY_H_ */

/* @} */
