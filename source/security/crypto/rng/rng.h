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
 *  \defgroup SECURITY_RNG_MODULE APIs for RNG
 *  \ingroup  SECURITY_MODULE
 *
 *  This module contains APIs to program and use the RNG.
 *
 *  @{
 */

/**
 *  \file rng.h
 *
 *  \brief This file contains the prototype of RNG driver APIs
 */

#ifndef RNG_H_
#define RNG_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <string.h>
#include <security/crypto/pka/hw_include/cslr_cp_ace.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
/** Rng drbg mode macro */
#define RNG_DRBG_MODE                                           (TRUE)
/** Rng drbg mode disable*/
#define RNG_DRBG_DISABLE_MODE                                   (FALSE)
/** Rng control request data mask */
#define RNG_CONTROL_REQUEST_DATA_MASK                           (0x00010000U)
/** Rng control data block mask */
#define RNG_CONTROL_DATA_BLOCKS_MASK                            (0xFFF00000U)
/** Rng control drbg enable shift */
#define RNG_CONTROL_DRBG_EN_SHIFT                               (0x0000000CU)
/** Rng control enable trng shift */
#define RNG_CONTROL_ENABLE_TRNG_SHIFT                           (0x0000000AU)
/** Rng satus reseed Ai mask */
#define RNG_STATUS_RESEED_AI_MASK                               (0x00000400U)
/** Maximum seed array size in dword */
#define RNG_DRBG_SEED_MAX_ARRY_SIZE_IN_DWORD                    (12U)
/** \brief Handle to the RNG driver */
typedef void *RNG_Handle;

/**
 * \brief
 *  RNG Driver Error code
 *
 * \details
 *  The enumeration describes all the possible return and error codes which
 *  the RNG Driver can return
 */
typedef enum RNG_Return_e
{
    RNG_RETURN_SUCCESS                  = 0xDE3BA502U, /*!< Success/pass return code */
    RNG_RETURN_FAILURE                  = 0xF33BE03EU, /*!< General or unspecified failure/error */
}RNG_Return_t;

/** \brief RNG attributes */
typedef struct
{
    /*
     * SOC configuration
     */
    uint32_t                caBaseAddr;
    /**< Crypto Accelerator Base Adders*/
    uint32_t                rngBaseAddr;
    /**< RNG Base address */
    uint32_t                mode;
    /**< Flag to enable different modes*/
    uint32_t                isOpen;
    /**< Flag to indicate whether the instance is opened already */
    uint32_t                faultStatus;
    /**< Flag to ascertain whether the handle was opened successfully */
    uint32_t                *seedValue;
    /**< pointer to hold seeding value from user*/
    uint32_t                seedSizeInDwords;
    /**< seed size in words*/
} RNG_Attrs;

/** \brief RNG driver context */
typedef struct
{
    RNG_Attrs             *attrs;
    /**< Driver params passed during open */
} RNG_Config;

/** \brief device type HSSE */
#define DEVTYPE_HSSE         (0x0AU)

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/** \brief Externally defined driver configuration array */
extern RNG_Config            gRngConfig[];
/** \brief Externally defined driver configuration Num */
extern uint32_t              gRngConfigNum;

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

/**
 * \brief Function to Open RNG instance, Initialize clocks
 *
 * \return        A #RNG_Handle on success or a NULL on an error or if it has been
 *                opened already
 */
RNG_Handle RNG_open(uint32_t index);

/**
 *  \brief  Function to close a RNG module specified by the RNG handle
 *
 *  \param  handle  #RNG_Handle returned from #RNG_open()
 *
 *  \return returns #RNG_RETURN_SUCCESS if requested data is available.
 *                 #RNG_RETURN_FAILURE if RNG is not initialized.
 */
RNG_Return_t RNG_close(RNG_Handle handle);

/**
 * \brief setup the RNG module.
 *
 * \param  handle  #RNG_Handle returned from #RNG_open()
 *
 * \return returns #RNG_RETURN_SUCCESS if requested data is available.
 *                 #RNG_RETURN_FAILURE if RNG is not initialized.
 */
RNG_Return_t RNG_setup(RNG_Handle handle);

/**
 * \brief Read random numbers into the output buffer.
 *
 * \param  handle  #RNG_Handle returned from #RNG_open()
 *
 * \param out pointer to the buffer for the 128 bit random data.
 *
 * \return returns #RNG_RETURN_SUCCESS if requested data is available.
 *                 #RNG_RETURN_FAILURE if RNG is not initialized.
 */
RNG_Return_t RNG_read(RNG_Handle handle, uint32_t *out);

#ifdef __cplusplus
}
#endif

#endif /* RNG_H_ */

/** @} */