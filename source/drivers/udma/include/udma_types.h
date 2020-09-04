/*
 *  Copyright (C) 2018-2021 Texas Instruments Incorporated
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
 *  \defgroup DRV_UDMA_TYPE_MODULE UDMA Common Data Types
 *            This is UDMA driver common parameters and API
 *
 *  @{
 */

/**
 *  \file udma_types.h
 *
 *  \brief UDMA Low Level Driver API/interface data types file.
 */

#ifndef UDMA_TYPES_H_
#define UDMA_TYPES_H_

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

/** \brief UDMA driver handle */
typedef void *                          Udma_DrvHandle;
/** \brief UDMA channel handle */
typedef void *                          Udma_ChHandle;
/** \brief UDMA event handle */
typedef void *                          Udma_EventHandle;
/** \brief UDMA ring handle */
typedef void *                          Udma_RingHandle;
/** \brief UDMA flow handle */
typedef void *                          Udma_FlowHandle;

/** \brief Cache line size for alignment of descriptor and buffers */
#define UDMA_CACHELINE_ALIGNMENT        (128U)

/** \brief Macro to align the size in bytes to UDMA cache line alignment */
#define UDMA_ALIGN_SIZE(x)              (((x) + UDMA_CACHELINE_ALIGNMENT) & ~(UDMA_CACHELINE_ALIGNMENT - 1U))

/** \brief Default UDMA channel disable timeout */
#define UDMA_DEFAULT_CH_DISABLE_TIMEOUT (100U)

/**
 *  \anchor Udma_ErrorCodes
 *  \name UDMA Error Codes
 *
 *  Error codes returned by UDMA APIs
 *
 *  @{
 */
/** \brief API call successful */
#define UDMA_SOK                        ( (int32_t) (0))
/** \brief API call returned with error as failed. Used for generic error.
 *  It may be some hardware failure and/or software failure. */
#define UDMA_EFAIL                      (-(int32_t) (1))
/** \brief API call returned with error as bad arguments.
 *  Typically, NULL pointer passed to the API where its not expected. */
#define UDMA_EBADARGS                   (-(int32_t) (2))
/** \brief API call returned with error as invalid parameters. Typically
 *  when parameters passed are not valid or out of range. */
#define UDMA_EINVALID_PARAMS            (-(int32_t) (3))
/** \brief API call returned with error as timed out. Typically API is
 *  waiting for some condition and returned as condition not happened
 *  in the timeout period. */
#define UDMA_ETIMEOUT                   (-(int32_t) (4))
/** \brief API call returned with error as allocation failed. */
#define UDMA_EALLOC                     (-(int32_t) (5))
/** @} */

/**
 *  \anchor Udma_InstanceId
 *  \name UDMA Instance ID
 *
 *  This represents the various Instances in an SOC. The actual Instances present
 *  in the chip is SOC dependent. Refer soc file for the actual instance
 *  present. Kindly use \ref Udma_InstanceIdSoc macros for SOC specific name.
 *
 *  @{
 */
#define UDMA_INST_ID_0                  (0U)
#define UDMA_INST_ID_1                  (1U)
#define UDMA_INST_ID_2                  (2U)
#define UDMA_INST_ID_3                  (3U)
/** @} */

/**
 *  \anchor Udma_MappedGrp
 *  \name UDMA Mapped Group
 *
 *  This represents the various Mapped TX & RX Channels/Rings in the SOC. The actual
 *  Mapped TX & RX groups present in the chip is SOC dependent. Refer soc file
 *  for the actual instance present.
 *  Kindly use \ref Udma_MappedTxGrpSoc macros for Mapped TX SOC specific name
 *  and \ref Udma_MappedRxGrpSoc macros for Mapped RX SOC specific name.
 *
 *  @{
 */
#define UDMA_MAPPED_GROUP0              (0U)
#define UDMA_MAPPED_GROUP1              (1U)
#define UDMA_MAPPED_GROUP2              (2U)
#define UDMA_MAPPED_GROUP3              (3U)
#define UDMA_MAPPED_GROUP4              (4U)
#define UDMA_MAPPED_GROUP5              (5U)
#define UDMA_MAPPED_GROUP6              (6U)
#define UDMA_MAPPED_GROUP7              (7U)
/** @} */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
}
#endif

#endif /* #ifndef UDMA_TYPES_H_ */

/** @} */
