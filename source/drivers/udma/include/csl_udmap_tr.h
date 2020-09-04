/*
 *  Copyright (C) 2016-2019 Texas Instruments Incorporated.
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
 */
/**
 *  \file   csl_udmap_tr.h
 *
 *  \brief  This file provides support for the UDMAP Transfer Request (TR)
 *          functionality.
 */
/**
 *  \ingroup DRV_UDMA_MODULE
 *  \defgroup CSL_UDMAP_TR UDMA TR API
 *      This is the interface for the different Transfer Request (TR) structures
 *      and different make APIs to construct and use TR
 *  @{
 */

#ifndef CSL_UDMAP_TR_H_
#define CSL_UDMAP_TR_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <drivers/hw_include/cslr.h>

/*-----------------------------------------------------------------------------
 *  Transfer Request (TR) support
 *---------------------------------------------------------------------------*/

/** \brief CSL_UdmapTR specifies a Transfer Request. */
struct CSL_UdmapTR_t
{
    uint32_t flags;     /**< Specifies type of TR and how the TR should be handled. It also supports the TR triggering and output events. */
    uint16_t icnt0;     /**< Total loop iteration count for level 0 (innermost) */
    uint16_t icnt1;     /**< Total loop iteration count for level 1 */
    uint64_t addr;      /**< Starting address for the source data or destination data if it is a half duplex write */
    int32_t  dim1;      /**< Signed dimension for loop level 1 for the source data */
    uint16_t icnt2;     /**< Total loop iteration count for level 2 */
    uint16_t icnt3;     /**< Total loop iteration count for level 3 (outermost) */
    int32_t  dim2;      /**< Signed dimension for loop level 2 */
    int32_t  dim3;      /**< Signed dimension for loop level 3 */
    uint32_t fmtflags;  /**< Tells how the data is formatted either between the input and the output or if the data should use different addressing schemes or sizes */
    int32_t  ddim1;     /**< Signed dimension for loop level 1 for the destination data */
    uint64_t daddr;     /**< Starting address for the destination of the data */
    int32_t  ddim2;     /**< Signed dimension for loop level 2 for the destination data */
    int32_t  ddim3;     /**< Signed dimension for loop level 3 for the destination data */
    uint16_t dicnt0;    /**< Total loop iteration count for level 0 (innermost) used for destination */
    uint16_t dicnt1;    /**< Total loop iteration count for level 1 used for destination */
    uint16_t dicnt2;    /**< Total loop iteration count for level 2 used for destination */
    uint16_t dicnt3;    /**< Total loop iteration count for level 3 used for destination */
} __attribute__((__packed__));

typedef struct CSL_UdmapTR_t CSL_UdmapTR;

/** \brief CSL_UdmapTR0 specifies a Type 0 (One dimensional data move) Transfer Request. */
struct CSL_UdmapTR0_t
{
    uint32_t flags;     /**< Specifies type of TR and how the TR should be handled. It also supports the TR triggering and output events. */
    uint32_t icnt0;     /**< Total loop iteration count for level 0 (innermost) */
    uint64_t addr;      /**< Starting address for the source data or destination data if it is a half duplex write */
} __attribute__((__packed__));

typedef struct CSL_UdmapTR0_t CSL_UdmapTR0;

/** \brief CSL_UdmapTR1 specifies a Type 1 (Two dimensional data move) Transfer Request. */
struct CSL_UdmapTR1_t
{
    uint32_t flags;     /**< Specifies type of TR and how the TR should be handled. It also supports the TR triggering and output events. */
    uint16_t icnt0;     /**< Total loop iteration count for level 0 (innermost) */
    uint16_t icnt1;     /**< Total loop iteration count for level 1 */
    uint64_t addr;      /**< Starting address for the source data or destination data if it is a half duplex write */
    int32_t  dim1;      /**< Signed dimension for loop level 1 for the source data */
} __attribute__((__packed__));

typedef struct CSL_UdmapTR1_t CSL_UdmapTR1;

/** \brief CSL_UdmapTR2 specifies a Type 2 (Three dimensional data move) Transfer Request. */
struct CSL_UdmapTR2_t
{
    uint32_t flags;     /**< Specifies type of TR and how the TR should be handled. It also supports the TR triggering and output events. */
    uint16_t icnt0;     /**< Total loop iteration count for level 0 (innermost) */
    uint16_t icnt1;     /**< Total loop iteration count for level 1 */
    uint64_t addr;      /**< Starting address for the source data or destination data if it is a half duplex write */
    int32_t  dim1;      /**< Signed dimension for loop level 1 for the source data */
    uint32_t icnt2;     /**< Total loop iteration count for level 2 */
    int32_t  dim2;      /**< Signed dimension for loop level 2 */
} __attribute__((__packed__));

typedef struct CSL_UdmapTR2_t CSL_UdmapTR2;

/** \brief CSL_UdmapTR3 specifies a Type 3 (Four dimensional data move) Transfer Request. */
struct CSL_UdmapTR3_t
{
    uint32_t flags;     /**< Specifies type of TR and how the TR should be handled. It also supports the TR triggering and output events. */
    uint16_t icnt0;     /**< Total loop iteration count for level 0 (innermost) */
    uint16_t icnt1;     /**< Total loop iteration count for level 1 */
    uint64_t addr;      /**< Starting address for the source data or destination data if it is a half duplex write */
    int32_t  dim1;      /**< Signed dimension for loop level 1 for the source data */
    uint16_t icnt2;     /**< Total loop iteration count for level 2 */
    uint16_t icnt3;     /**< Total loop iteration count for level 3 (outermost) */
    int32_t  dim2;      /**< Signed dimension for loop level 2 */
    int32_t  dim3;      /**< Signed dimension for loop level 3 */
} __attribute__((__packed__));

typedef struct CSL_UdmapTR3_t CSL_UdmapTR3;

/** \brief CSL_UdmapTR4 specifies a Type 4 (Four dimensional data move with data formatting) Transfer Request. */
struct CSL_UdmapTR4_t
{
    uint32_t flags;     /**< Specifies type of TR and how the TR should be handled. It also supports the TR triggering and output events. */
    uint16_t icnt0;     /**< Total loop iteration count for level 0 (innermost) */
    uint16_t icnt1;     /**< Total loop iteration count for level 1 */
    uint64_t addr;      /**< Starting address for the source data or destination data if it is a half duplex write */
    int32_t  dim1;      /**< Signed dimension for loop level 1 for the source data */
    uint16_t icnt2;     /**< Total loop iteration count for level 2 */
    uint16_t icnt3;     /**< Total loop iteration count for level 3 (outermost) */
    int32_t  dim2;      /**< Signed dimension for loop level 2 */
    int32_t  dim3;      /**< Signed dimension for loop level 3 */
    uint32_t fmtflags;  /**< Tells how the data is formatted either between the input and the output or if the data should use different addressing schemes or sizes */
} __attribute__((__packed__));

typedef struct CSL_UdmapTR4_t CSL_UdmapTR4;

/** \brief CSL_UdmapTR5 specifies a Type 5 (Four dimensional cache warm) Transfer Request. */
struct CSL_UdmapTR5_t
{
    uint32_t flags;     /**< Specifies type of TR and how the TR should be handled. It also supports the TR triggering and output events. */
    uint16_t icnt0;     /**< Total loop iteration count for level 0 (innermost) */
    uint16_t icnt1;     /**< Total loop iteration count for level 1 */
    uint64_t addr;      /**< Starting address for the source data or destination data if it is a half duplex write */
    int32_t  dim1;      /**< Signed dimension for loop level 1 for the source data */
    uint16_t icnt2;     /**< Total loop iteration count for level 2 */
    uint16_t icnt3;     /**< Total loop iteration count for level 3 (outermost) */
    int32_t  dim2;      /**< Signed dimension for loop level 2 */
    int32_t  dim3;      /**< Signed dimension for loop level 3 */
    uint32_t cacheflags;    /**< Tells how the data is formatted either between the input and the output or if the data should use different addressing schemes or sizes */
} __attribute__((__packed__));

typedef struct CSL_UdmapTR5_t CSL_UdmapTR5;

/** \brief CSL_UdmapTR8 specifies a Type 8 Transfer Request. */
struct CSL_UdmapTR8_t
{
    uint32_t flags;     /**< Specifies type of TR and how the TR should be handled. It also supports the TR triggering and output events. */
    uint16_t icnt0;     /**< Total loop iteration count for level 0 (innermost) */
    uint16_t icnt1;     /**< Total loop iteration count for level 1 */
    uint64_t addr;      /**< Starting address for the source data or destination data if it is a half duplex write */
    int32_t  dim1;      /**< Signed dimension for loop level 1 for the source data */
    uint16_t icnt2;     /**< Total loop iteration count for level 2 */
    uint16_t icnt3;     /**< Total loop iteration count for level 3 (outermost) */
    int32_t  dim2;      /**< Signed dimension for loop level 2 */
    int32_t  dim3;      /**< Signed dimension for loop level 3 */
    uint32_t fmtflags;  /**< Tells how the data is formatted either between the input and the output or if the data should use different addressing schemes or sizes */
    int32_t  ddim1;     /**< Signed dimension for loop level 1 for the destination data */
    uint64_t daddr;     /**< Starting address for the destination of the data */
    int32_t  ddim2;     /**< Signed dimension for loop level 2 for the destination data */
    int32_t  ddim3;     /**< Signed dimension for loop level 3 for the destination data */
} __attribute__((__packed__));

typedef struct CSL_UdmapTR8_t CSL_UdmapTR8;

/** \brief CSL_UdmapTR9 specifies a Type 9 Transfer Request. */
struct CSL_UdmapTR9_t
{
    uint32_t flags;     /**< Specifies type of TR and how the TR should be handled. It also supports the TR triggering and output events. */
    uint16_t icnt0;     /**< Total loop iteration count for level 0 (innermost) */
    uint16_t icnt1;     /**< Total loop iteration count for level 1 */
    uint64_t addr;      /**< Starting address for the source data or destination data if it is a half duplex write */
    int32_t  dim1;      /**< Signed dimension for loop level 1 for the source data */
    uint16_t icnt2;     /**< Total loop iteration count for level 2 */
    uint16_t icnt3;     /**< Total loop iteration count for level 3 (outermost) */
    int32_t  dim2;      /**< Signed dimension for loop level 2 */
    int32_t  dim3;      /**< Signed dimension for loop level 3 */
    uint32_t fmtflags;  /**< Tells how the data is formatted either between the input and the output or if the data should use different addressing schemes or sizes */
    int32_t  ddim1;     /**< Signed dimension for loop level 1 for the destination data */
    uint64_t daddr;     /**< Starting address for the destination of the data */
    int32_t  ddim2;     /**< Signed dimension for loop level 2 for the destination data */
    int32_t  ddim3;     /**< Signed dimension for loop level 3 for the destination data */
    uint16_t dicnt0;    /**< Total loop iteration count for level 0 (innermost) used for destination */
    uint16_t dicnt1;    /**< Total loop iteration count for level 1 used for destination */
    uint16_t dicnt2;    /**< Total loop iteration count for level 2 used for destination */
    uint16_t dicnt3;    /**< Total loop iteration count for level 3 used for destination */
} __attribute__((__packed__));

typedef struct CSL_UdmapTR9_t CSL_UdmapTR9;

/** \brief CSL_UdmapTR10 specifies a Type 10 Transfer Request. */
struct CSL_UdmapTR10_t
{
    uint32_t flags;     /**< Specifies type of TR and how the TR should be handled. It also supports the TR triggering and output events. */
    uint16_t icnt0;     /**< Total loop iteration count for level 0 (innermost) */
    uint16_t icnt1;     /**< Total loop iteration count for level 1 */
    uint64_t addr;      /**< Starting address for the source data or destination data if it is a half duplex write */
    int32_t  dim1;      /**< Signed dimension for loop level 1 for the source data */
    uint32_t rsvd[3];   /**< Reserved */
    uint32_t fmtflags;  /**< Tells how the data is formatted either between the input and the output or if the data should use different addressing schemes or sizes */
    int32_t  ddim1;     /**< Signed dimension for loop level 1 for the destination data */
    uint64_t daddr;     /**< Starting address for the destination of the data */
} __attribute__((__packed__));

typedef struct CSL_UdmapTR10_t CSL_UdmapTR10;

/** \brief CSL_UdmapTR11 specifies a Type 11 Transfer Request. */
struct CSL_UdmapTR11_t
{
    uint32_t flags;     /**< Specifies type of TR and how the TR should be handled. It also supports the TR triggering and output events. */
    uint16_t icnt0;     /**< Total loop iteration count for level 0 (innermost) */
    uint16_t icnt1;     /**< Total loop iteration count for level 1 */
    uint64_t addr;      /**< Starting address for the source data or destination data if it is a half duplex write */
    int32_t  dim1;      /**< Signed dimension for loop level 1 for the source data */
    uint32_t rsvd0[3];  /**< Reserved */
    uint32_t fmtflags;  /**< Tells how the data is formatted either between the input and the output or if the data should use different addressing schemes or sizes */
    int32_t  ddim1;     /**< Signed dimension for loop level 1 for the destination data */
    uint64_t daddr;     /**< Starting address for the destination of the data */
    uint32_t rsvd1[2];  /**< Reserved */
    uint16_t dicnt0;    /**< Total loop iteration count for level 0 (innermost) used for destination */
    uint16_t dicnt1;    /**< Total loop iteration count for level 1 used for destination */
} __attribute__((__packed__));

typedef struct CSL_UdmapTR11_t CSL_UdmapTR11;

/** \brief CSL_UdmapTR15 specifies a Type 15 Transfer Request. */
struct CSL_UdmapTR15_t
{
    uint32_t flags;     /**< Specifies type of TR and how the TR should be handled. It also supports the TR triggering and output events. */
    uint16_t icnt0;     /**< Total loop iteration count for level 0 (innermost) */
    uint16_t icnt1;     /**< Total loop iteration count for level 1 */
    uint64_t addr;      /**< Starting address for the source data or destination data if it is a half duplex write */
    int32_t  dim1;      /**< Signed dimension for loop level 1 for the source data */
    uint16_t icnt2;     /**< Total loop iteration count for level 2 */
    uint16_t icnt3;     /**< Total loop iteration count for level 3 (outermost) */
    int32_t  dim2;      /**< Signed dimension for loop level 2 */
    int32_t  dim3;      /**< Signed dimension for loop level 3 */
    uint32_t fmtflags;  /**< Tells how the data is formatted either between the input and the output or if the data should use different addressing schemes or sizes */
    int32_t  ddim1;     /**< Signed dimension for loop level 1 for the destination data */
    uint64_t daddr;     /**< Starting address for the destination of the data */
    int32_t  ddim2;     /**< Signed dimension for loop level 2 for the destination data */
    int32_t  ddim3;     /**< Signed dimension for loop level 3 for the destination data */
    uint16_t dicnt0;    /**< Total loop iteration count for level 0 (innermost) used for destination */
    uint16_t dicnt1;    /**< Total loop iteration count for level 1 used for destination */
    uint16_t dicnt2;    /**< Total loop iteration count for level 2 used for destination */
    uint16_t dicnt3;    /**< Total loop iteration count for level 3 used for destination */
} __attribute__((__packed__));

typedef struct CSL_UdmapTR15_t CSL_UdmapTR15;

/**
 *  \anchor CSL_UdmapTrFlags_t
 *  \name CSL UDMAP TR Flags
 *
 *  CSL UDMAP TR Flags.
 *
 *  @{
 */
#define CSL_UDMAP_TR_FLAGS_TYPE_SHIFT                    ((uint32_t) 0U)
#define CSL_UDMAP_TR_FLAGS_TYPE_MASK                     (((uint32_t) 0xFU) << CSL_UDMAP_TR_FLAGS_TYPE_SHIFT)
#define CSL_UDMAP_TR_FLAGS_STATIC_SHIFT                  ((uint32_t) 4U)
#define CSL_UDMAP_TR_FLAGS_STATIC_MASK                   (((uint32_t) 1U) << CSL_UDMAP_TR_FLAGS_STATIC_SHIFT)
#define CSL_UDMAP_TR_FLAGS_WAIT_SHIFT                    ((uint32_t) 5U)
#define CSL_UDMAP_TR_FLAGS_WAIT_MASK                     (((uint32_t) 1U) << CSL_UDMAP_TR_FLAGS_WAIT_SHIFT)
#define CSL_UDMAP_TR_FLAGS_EVENT_SIZE_SHIFT              ((uint32_t) 6U)
#define CSL_UDMAP_TR_FLAGS_EVENT_SIZE_MASK               (((uint32_t) 3U) << CSL_UDMAP_TR_FLAGS_EVENT_SIZE_SHIFT)
#define CSL_UDMAP_TR_FLAGS_TRIGGER0_SHIFT                ((uint32_t) 8U)
#define CSL_UDMAP_TR_FLAGS_TRIGGER0_MASK                 (((uint32_t) 3U) << CSL_UDMAP_TR_FLAGS_TRIGGER0_SHIFT)
#define CSL_UDMAP_TR_FLAGS_TRIGGER0_TYPE_SHIFT           ((uint32_t) 10U)
#define CSL_UDMAP_TR_FLAGS_TRIGGER0_TYPE_MASK            (((uint32_t) 3U) << CSL_UDMAP_TR_FLAGS_TRIGGER0_TYPE_SHIFT)
#define CSL_UDMAP_TR_FLAGS_TRIGGER1_SHIFT                ((uint32_t) 12U)
#define CSL_UDMAP_TR_FLAGS_TRIGGER1_MASK                 (((uint32_t) 3U) << CSL_UDMAP_TR_FLAGS_TRIGGER1_SHIFT)
#define CSL_UDMAP_TR_FLAGS_TRIGGER1_TYPE_SHIFT           ((uint32_t) 14U)
#define CSL_UDMAP_TR_FLAGS_TRIGGER1_TYPE_MASK            (((uint32_t) 3U) << CSL_UDMAP_TR_FLAGS_TRIGGER1_TYPE_SHIFT)
#define CSL_UDMAP_TR_FLAGS_CMD_ID_SHIFT                  ((uint32_t) 16U)
#define CSL_UDMAP_TR_FLAGS_CMD_ID_MASK                   (((uint32_t) 0xFFU) << CSL_UDMAP_TR_FLAGS_CMD_ID_SHIFT)
#define CSL_UDMAP_TR_FLAGS_CFG_FLAGS_SHIFT               ((uint32_t) 24U)
#define CSL_UDMAP_TR_FLAGS_CFG_FLAGS_MASK                (((uint32_t) 0xFFU) << CSL_UDMAP_TR_FLAGS_CFG_FLAGS_SHIFT)
#define CSL_UDMAP_TR_FLAGS_SA_INDIRECT_SHIFT             ((uint32_t) 24U)
#define CSL_UDMAP_TR_FLAGS_SA_INDIRECT_MASK              (((uint32_t) 1U) << CSL_UDMAP_TR_FLAGS_SA_INDIRECT_SHIFT)
#define CSL_UDMAP_TR_FLAGS_DA_INDIRECT_SHIFT             ((uint32_t) 25U)
#define CSL_UDMAP_TR_FLAGS_DA_INDIRECT_MASK              (((uint32_t) 1U) << CSL_UDMAP_TR_FLAGS_DA_INDIRECT_SHIFT)
#define CSL_UDMAP_TR_FLAGS_SUPR_EVT_SHIFT                ((uint32_t) 26U)
#define CSL_UDMAP_TR_FLAGS_SUPR_EVT_MASK                 (((uint32_t) 1U) << CSL_UDMAP_TR_FLAGS_SUPR_EVT_SHIFT)
#define CSL_UDMAP_TR_FLAGS_EOL_SHIFT                     ((uint32_t) 28U)
#define CSL_UDMAP_TR_FLAGS_EOL_MASK                      (((uint32_t) 7U) << CSL_UDMAP_TR_FLAGS_EOL_SHIFT)
#define CSL_UDMAP_TR_FLAGS_EOP_SHIFT                     ((uint32_t) 31U)
#define CSL_UDMAP_TR_FLAGS_EOP_MASK                      (((uint32_t) 1U) << CSL_UDMAP_TR_FLAGS_EOP_SHIFT)
/** @} */

/**
 * \brief This enumerator defines the the type of TR being sent
 *
 *  \anchor CSL_UdmapTrFlagsType
 *  \name UDMAP TR flags type
 *
 *  @{
 */
typedef uint32_t CSL_UdmapTrFlagsType;
#define CSL_UDMAP_TR_FLAGS_TYPE_1D_DATA_MOVE                        ((uint32_t) 0U)
#define CSL_UDMAP_TR_FLAGS_TYPE_2D_DATA_MOVE                        ((uint32_t) 1U)
#define CSL_UDMAP_TR_FLAGS_TYPE_3D_DATA_MOVE                        ((uint32_t) 2U)
#define CSL_UDMAP_TR_FLAGS_TYPE_4D_DATA_MOVE                        ((uint32_t) 3U)
#define CSL_UDMAP_TR_FLAGS_TYPE_4D_DATA_MOVE_FORMATTING             ((uint32_t) 4U)
#define CSL_UDMAP_TR_FLAGS_TYPE_4D_CACHE_WARM                       ((uint32_t) 5U)
#define CSL_UDMAP_TR_FLAGS_TYPE_4D_BLOCK_MOVE                       ((uint32_t) 8U)
#define CSL_UDMAP_TR_FLAGS_TYPE_4D_BLOCK_MOVE_REPACKING             ((uint32_t) 9U)
#define CSL_UDMAP_TR_FLAGS_TYPE_2D_BLOCK_MOVE                       ((uint32_t) 10U)
#define CSL_UDMAP_TR_FLAGS_TYPE_2D_BLOCK_MOVE_REPACKING             ((uint32_t) 11U)
#define CSL_UDMAP_TR_FLAGS_TYPE_4D_BLOCK_MOVE_REPACKING_INDIRECTION ((uint32_t) 15U)
/** @} */

/**
 * \brief This enumerator specifies when an event is generated for each TR
 *
 *  \anchor CSL_UdmapTrFlagsEventSize
 *  \name UDMAP TR flags event size
 *
 *  @{
 */
typedef uint32_t CSL_UdmapTrFlagsEventSize;
/* When TR is complete and all status for the TR has been received */
#define CSL_UDMAP_TR_FLAGS_EVENT_SIZE_COMPLETION    ((uint32_t) 0U)
/* Type 0: when the last data transaction is sent for the TR; Type 1-11: when ICNT1 is decremented */
#define CSL_UDMAP_TR_FLAGS_EVENT_SIZE_ICNT1_DEC     ((uint32_t) 1U)
/* Type 0-1,10-11: when the last data transaction is sent for the TR; All other types: when ICNT2 is decremented */
#define CSL_UDMAP_TR_FLAGS_EVENT_SIZE_ICNT2_DEC     ((uint32_t) 2U)
/* Type 0-2,10-11: when the last data transaction is sent for the TR; All other types: when ICNT3 is decremented */
#define CSL_UDMAP_TR_FLAGS_EVENT_SIZE_ICNT3_DEC     ((uint32_t) 3U)
/** @} */

/**
 * \brief This enumerator specifies the type of trigger used to enable the TR
 *        to transfer data as specified by CSL_UdmapTrFlagsTriggerType
 *
 *  \anchor CSL_UdmapTrFlagsTrigger
 *  \name UDMAP TR flags trigger
 *
 *  @{
 */
typedef uint32_t CSL_UdmapTrFlagsTrigger;
#define CSL_UDMAP_TR_FLAGS_TRIGGER_NONE             ((uint32_t) 0U)
#define CSL_UDMAP_TR_FLAGS_TRIGGER_GLOBAL0          ((uint32_t) 1U)
#define CSL_UDMAP_TR_FLAGS_TRIGGER_GLOBAL1          ((uint32_t) 2U)
#define CSL_UDMAP_TR_FLAGS_TRIGGER_LOCAL_EVENT      ((uint32_t) 3U)
/** @} */

/**
 * \brief This enumerator specifies the type of data transfer that will be
 *        enabled by receiving a trigger as specified by CSL_UdmapTrFlagsTrigger
 *
 *  \anchor CSL_UdmapTrFlagsTriggerType
 *  \name UDMAP TR flags trigger type
 *
 *  @{
 */
typedef uint32_t CSL_UdmapTrFlagsTriggerType;
#define CSL_UDMAP_TR_FLAGS_TRIGGER_TYPE_ICNT1_DEC   ((uint32_t) 0U)
#define CSL_UDMAP_TR_FLAGS_TRIGGER_TYPE_ICNT2_DEC   ((uint32_t) 1U)
#define CSL_UDMAP_TR_FLAGS_TRIGGER_TYPE_ICNT3_DEC   ((uint32_t) 2U)
#define CSL_UDMAP_TR_FLAGS_TRIGGER_TYPE_ALL         ((uint32_t) 3U)
/** @} */

/**
 *  \anchor CSL_UdmapTrFormatFlags_t
 *  \name CSL UDMAP TR Format Flags
 *
 *  CSL UDMAP TR Format Flags.
 *
 *  @{
 */
#define CSL_UDMAP_TR_FMTFLAGS_AMODE_SHIFT                ((uint32_t) 0U)
#define CSL_UDMAP_TR_FMTFLAGS_AMODE_MASK                 (((uint32_t) 7U) << CSL_UDMAP_TR_FMTFLAGS_AMODE_SHIFT)
#define CSL_UDMAP_TR_FMTFLAGS_DIR_SHIFT                  ((uint32_t) 3U)
#define CSL_UDMAP_TR_FMTFLAGS_DIR_MASK                   (((uint32_t) 1U) << CSL_UDMAP_TR_FMTFLAGS_DIR_SHIFT)
#define CSL_UDMAP_TR_FMTFLAGS_ELYPE_SHIFT                ((uint32_t) 4U)
#define CSL_UDMAP_TR_FMTFLAGS_ELYPE_MASK                 (((uint32_t) 0xFU) << CSL_UDMAP_TR_FMTFLAGS_ELYPE_SHIFT)
#define CSL_UDMAP_TR_FMTFLAGS_DFMT_SHIFT                 ((uint32_t) 8U)
#define CSL_UDMAP_TR_FMTFLAGS_DFMT_MASK                  (((uint32_t) 0xFU) << CSL_UDMAP_TR_FMTFLAGS_DFMT_SHIFT)
#define CSL_UDMAP_TR_FMTFLAGS_SECTR_SHIFT                ((uint32_t) 12U)
#define CSL_UDMAP_TR_FMTFLAGS_SECTR_MASK                 (((uint32_t) 3U) << CSL_UDMAP_TR_FMTFLAGS_SECTR_SHIFT)
#define CSL_UDMAP_TR_FMTFLAGS_AMODE_SPECIFIC_CBK0_SHIFT  ((uint32_t) 16U)
#define CSL_UDMAP_TR_FMTFLAGS_AMODE_SPECIFIC_CBK0_MASK   (((uint32_t) 0xFU) << CSL_UDMAP_TR_FMTFLAGS_AMODE_SPECIFIC_CBK0_SHIFT)
#define CSL_UDMAP_TR_FMTFLAGS_AMODE_SPECIFIC_CBK1_SHIFT  ((uint32_t) 20U)
#define CSL_UDMAP_TR_FMTFLAGS_AMODE_SPECIFIC_CBK1_MASK   (((uint32_t) 0xFU) << CSL_UDMAP_TR_FMTFLAGS_AMODE_SPECIFIC_CBK1_SHIFT)
#define CSL_UDMAP_TR_FMTFLAGS_AMODE_SPECIFIC_AM0_SHIFT   ((uint32_t) 24U)
#define CSL_UDMAP_TR_FMTFLAGS_AMODE_SPECIFIC_AM0_MASK    (((uint32_t) 3U) << CSL_UDMAP_TR_FMTFLAGS_AMODE_SPECIFIC_AM0_SHIFT)
#define CSL_UDMAP_TR_FMTFLAGS_AMODE_SPECIFIC_AM1_SHIFT   ((uint32_t) 26U)
#define CSL_UDMAP_TR_FMTFLAGS_AMODE_SPECIFIC_AM1_MASK    (((uint32_t) 3U) << CSL_UDMAP_TR_FMTFLAGS_AMODE_SPECIFIC_AM1_SHIFT)
#define CSL_UDMAP_TR_FMTFLAGS_AMODE_SPECIFIC_AM2_SHIFT   ((uint32_t) 28U)
#define CSL_UDMAP_TR_FMTFLAGS_AMODE_SPECIFIC_AM2_MASK    (((uint32_t) 3U) << CSL_UDMAP_TR_FMTFLAGS_AMODE_SPECIFIC_AM2_SHIFT)
#define CSL_UDMAP_TR_FMTFLAGS_AMODE_SPECIFIC_AM3_SHIFT   ((uint32_t) 30U)
#define CSL_UDMAP_TR_FMTFLAGS_AMODE_SPECIFIC_AM3_MASK    (((uint32_t) 3U) << CSL_UDMAP_TR_FMTFLAGS_AMODE_SPECIFIC_AM3_SHIFT)
#define CSL_UDMAP_TR_CACHEFLAGS_CACHEID_SHIFT            ((uint32_t) 0U)
#define CSL_UDMAP_TR_CACHEFLAGS_CACHEID_MASK             (((uint32_t) 0xFFU) << CSL_UDMAP_TR_CACHEFLAGS_CACHEID_SHIFT)
#define CSL_UDMAP_TR_CACHEFLAGS_CACHEOP_SHIFT            ((uint32_t) 24U)
#define CSL_UDMAP_TR_CACHEFLAGS_CACHEOP_MASK             (((uint32_t) 0xFFU) << CSL_UDMAP_TR_CACHEFLAGS_CACHEOP_SHIFT)
/** @} */

/**
 * \brief This enumerator specifies the addressing mode of TR that is being sent
 *
 *  \anchor CSL_UdmapTrFmtflagsAmode
 *  \name UDMAP TR format flags amode
 *
 *  @{
 */
typedef uint32_t CSL_UdmapTrFmtflagsAmode;
#define CSL_UDMAP_TR_FMTFLAGS_AMODE_LINEAR          ((uint32_t) 0U)
#define CSL_UDMAP_TR_FMTFLAGS_AMODE_CIRCULAR        ((uint32_t) 1U)
/** @} */

/**
 * \brief This enumerator specifies CBK0 and CBK1 values used to compute the
 *        circular block sizes for circular addressing
 *
 *  \anchor CSL_UdmapTrFmtflagsAmodeSpecificCbk
 *  \name UDMAP TR format flags amode specific CBK values
 *
 *  @{
 */
typedef uint32_t CSL_UdmapTrFmtflagsAmodeSpecificCbk;
#define CSL_UDMAP_TR_FMTFLAGS_AMODE_SPECIFIC_CBK_512B   ((uint32_t) 0U)
#define CSL_UDMAP_TR_FMTFLAGS_AMODE_SPECIFIC_CBK_1KB    ((uint32_t) 1U)
#define CSL_UDMAP_TR_FMTFLAGS_AMODE_SPECIFIC_CBK_2KB    ((uint32_t) 2U)
#define CSL_UDMAP_TR_FMTFLAGS_AMODE_SPECIFIC_CBK_4KB    ((uint32_t) 3U)
#define CSL_UDMAP_TR_FMTFLAGS_AMODE_SPECIFIC_CBK_8KB    ((uint32_t) 4U)
#define CSL_UDMAP_TR_FMTFLAGS_AMODE_SPECIFIC_CBK_16KB   ((uint32_t) 5U)
#define CSL_UDMAP_TR_FMTFLAGS_AMODE_SPECIFIC_CBK_32KB   ((uint32_t) 6U)
#define CSL_UDMAP_TR_FMTFLAGS_AMODE_SPECIFIC_CBK_64KB   ((uint32_t) 7U)
#define CSL_UDMAP_TR_FMTFLAGS_AMODE_SPECIFIC_CBK_128KB  ((uint32_t) 8U)
#define CSL_UDMAP_TR_FMTFLAGS_AMODE_SPECIFIC_CBK_256KB  ((uint32_t) 9U)
#define CSL_UDMAP_TR_FMTFLAGS_AMODE_SPECIFIC_CBK_512KB  ((uint32_t) 10U)
#define CSL_UDMAP_TR_FMTFLAGS_AMODE_SPECIFIC_CBK_1GB    ((uint32_t) 11U)
#define CSL_UDMAP_TR_FMTFLAGS_AMODE_SPECIFIC_CBK_2GB    ((uint32_t) 12U)
#define CSL_UDMAP_TR_FMTFLAGS_AMODE_SPECIFIC_CBK_4GB    ((uint32_t) 13U)
#define CSL_UDMAP_TR_FMTFLAGS_AMODE_SPECIFIC_CBK_8GB    ((uint32_t) 14U)
#define CSL_UDMAP_TR_FMTFLAGS_AMODE_SPECIFIC_CBK_16GB   ((uint32_t) 15U)
/** @} */

/**
 * \brief This enumerator specifies the addressing mode of TR that is being sent
 *
 *  \anchor CSL_UdmapTrFmtflagsAmodeSpecificAmode
 *  \name UDMAP TR format flags amode specific amode
 *
 *  @{
 */
typedef uint32_t CSL_UdmapTrFmtflagsAmodeSpecificAmode;
#define CSL_UDMAP_TR_FMTFLAGS_AMODE_SPECIFIC_AMODE_LINEAR   ((uint32_t) 0U)
#define CSL_UDMAP_TR_FMTFLAGS_AMODE_SPECIFIC_AMODE_CBK0     ((uint32_t) 1U)
#define CSL_UDMAP_TR_FMTFLAGS_AMODE_SPECIFIC_AMODE_CBK1     ((uint32_t) 2U)
/** @} */

/**
 * \brief This enumerator specifies the addressing mode of TR that is being sent
 *
 *  \anchor CSL_UdmapTrFmtflagsDir
 *  \name UDMAP TR format flags direction
 *
 *  @{
 */
typedef uint32_t CSL_UdmapTrFmtflagsDir;
#define CSL_UDMAP_TR_FMTFLAGS_DIR_SRC_USES_AMODE        ((uint32_t) 0U)
#define CSL_UDMAP_TR_FMTFLAGS_DIR_DST_USES_AMODE        ((uint32_t) 1U)
/** @} */

/**
 * \brief This enumerator specifies the basic unit that is used for the
 *        innermost loop
 *
 *  \anchor CSL_UdmapTrFmtflagsEltype
 *  \name UDMAP TR format flags element type
 *
 *  @{
 */
typedef uint32_t CSL_UdmapTrFmtflagsEltype;
/* 1 Byte per element */
#define CSL_UDMAP_TR_FMTFLAGS_ELYPE_1           ((uint32_t) 0U)
/* 1.5 Bytes (12 bits) per element */
#define CSL_UDMAP_TR_FMTFLAGS_ELYPE_1p5         ((uint32_t) 1U)
/* 2 Bytes per element */
#define CSL_UDMAP_TR_FMTFLAGS_ELYPE_2           ((uint32_t) 2U)
/* 3 Bytes per element */
#define CSL_UDMAP_TR_FMTFLAGS_ELYPE_3           ((uint32_t) 3U)
/* 4 Bytes per element */
#define CSL_UDMAP_TR_FMTFLAGS_ELYPE_4           ((uint32_t) 4U)
/* 5 Bytes per element */
#define CSL_UDMAP_TR_FMTFLAGS_ELYPE_5           ((uint32_t) 5U)
/* 16 Bytes per element */
#define CSL_UDMAP_TR_FMTFLAGS_ELYPE_16          ((uint32_t) 6U)
/* 32 Bytes per element */
#define CSL_UDMAP_TR_FMTFLAGS_ELYPE_32          ((uint32_t) 7U)
/* 1 Byte per input element 2 Bytes per output element */
#define CSL_UDMAP_TR_FMTFLAGS_ELYPE_1_2         ((uint32_t) 8U)
/* 1.5 Bytes per input element 2 Bytes per output element */
#define CSL_UDMAP_TR_FMTFLAGS_ELYPE_1p5_2       ((uint32_t) 9U)
/* 2 Bytes per input element 1 Byte per output element */
#define CSL_UDMAP_TR_FMTFLAGS_ELYPE_2_1         ((uint32_t) 10U)
/* 2 Bytes per input element 1.5 Bytes per output element */
#define CSL_UDMAP_TR_FMTFLAGS_ELYPE_2_1p5       ((uint32_t) 11U)
/** @} */

/**
 * \brief This enumerator specifies a manipulation of the data between how it
 *        is read and how it is sent
 *
 *  \anchor CSL_UdmapTrFmtflagsDfmt
 *  \name UDMAP TR format flags dfmt
 *
 *  @{
 */
typedef uint32_t CSL_UdmapTrFmtflagsDfmt;
/* The input and output block will remain identical. */
#define CSL_UDMAP_TR_FMTFLAGS_DFMT_NO_CHANGE            ((uint32_t) 0U)
/* The input block is not an address but the address is up to a 64 bit constant. */
#define CSL_UDMAP_TR_FMTFLAGS_DFMT_CONSTANT_COPY        ((uint32_t) 1U)
/* The inner and second most inner loops are swapped so that rows become columns and columns become rows. */
#define CSL_UDMAP_TR_FMTFLAGS_DFMT_TRANSPOSE            ((uint32_t) 2U)
/* The data in the row will be accessed in the reverse of the order that it is read. */
#define CSL_UDMAP_TR_FMTFLAGS_DFMT_REVERSE              ((uint32_t) 3U)
/* The data will be written in the reverse of the order that is read as well as transposed. */
#define CSL_UDMAP_TR_FMTFLAGS_DFMT_REVERSE_TRANSPOSE    ((uint32_t) 4U)
/** @} */

/**
 * \brief This enumerator specifies the cache operation for a cache warm TR type
 *
 *  \anchor CSL_UdmapTrCacheflagsOp
 *  \name UDMAP TR format cache flags operation
 *
 *  @{
 */
typedef uint32_t CSL_UdmapTrCacheflagsOp;
#define CSL_UDMAP_TR_CACHEFLAGS_OP_PREWARM_CACHE        ((uint32_t) 0U)
#define CSL_UDMAP_TR_CACHEFLAGS_OP_PREWARM_MMU          ((uint32_t) 1U)
/** @} */

/*-----------------------------------------------------------------------------
 *  Secondary TR support
 *---------------------------------------------------------------------------*/
/** \brief CSL_UdmapSecTR specifies a secondary Transfer Request. */
struct CSL_UdmapSecTR_t
{
    uint64_t addr;      /**< Address */
    uint32_t flags;     /**< Flags */
    uint32_t data[13];  /**< Data */
} __attribute__((__packed__));

typedef struct CSL_UdmapSecTR_t CSL_UdmapSecTR;

/**
 *  \anchor CSL_UdmapSecTrFlags_t
 *  \name CSL UDMAP Secondary TR Flags
 *
 *  CSL UDMAP Secondary TR Flags.
 *
 *  @{
 */
#define CSL_UDMAP_SECTR_FLAGS_TYPE_SHIFT                 ((uint32_t) 0U)
#define CSL_UDMAP_SECTR_FLAGS_TYPE_MASK                  (((uint32_t) 0xFU) << CSL_UDMAP_SECTR_FLAGS_TYPE_SHIFT)
#define CSL_UDMAP_SECTR_FLAGS_TYPE_SPECIFIC_SHIFT        ((uint32_t) 4U)
#define CSL_UDMAP_SECTR_FLAGS_TYPE_SPECIFIC_MASK         (((uint32_t) 0xFFFFFFF0U) << CSL_UDMAP_SECTR_FLAGS_TYPE_SPECIFIC_SHIFT)
/** @} */

/**
 * \brief This enumerator specifies if the TR requires an additional TR located
 *        in memory
 *
 *  \anchor CSL_UdmapTrFmtflagsSectr
 *  \name UDMAP TR format flags secondary TR
 *
 *  @{
 */
typedef uint32_t CSL_UdmapTrFmtflagsSectr;
/* The TR does not require a secondary TR. */
#define CSL_UDMAP_TR_FMTFLAGS_SECTR_NONE        ((uint32_t) 0U)
/* The TR will fetch a 64 byte secondary TR prior to the initial read. */
#define CSL_UDMAP_TR_FMTFLAGS_SECTR_64          ((uint32_t) 1U)
/* The TR will fetch a 128 byte secondary TR prior to the initial read. */
#define CSL_UDMAP_TR_FMTFLAGS_SECTR_128         ((uint32_t) 2U)
/** @} */

/*-----------------------------------------------------------------------------
 *  TR response support
 *---------------------------------------------------------------------------*/
struct CSL_UdmapTrResponse_t
{
    uint32_t flags;     /**< Status flags */
} __attribute__((__packed__));

typedef struct CSL_UdmapTrResponse_t CSL_UdmapTrResponse;

/**
 *  \anchor CSL_UdmapTrResponseFlags_t
 *  \name CSL UDMAP TR Response Flags
 *
 *  CSL UDMAP TR Response Flags.
 *
 *  @{
 */
#define CSL_UDMAP_TR_RESPONSE_STATUS_TYPE_SHIFT          ((uint32_t) 0U)
#define CSL_UDMAP_TR_RESPONSE_STATUS_TYPE_MASK           (((uint32_t) 0xFU) << CSL_UDMAP_TR_RESPONSE_STATUS_TYPE_SHIFT)
#define CSL_UDMAP_TR_RESPONSE_STATUS_INFO_SHIFT          ((uint32_t) 4U)
#define CSL_UDMAP_TR_RESPONSE_STATUS_INFO_MASK           (((uint32_t) 0xFU) << CSL_UDMAP_TR_RESPONSE_STATUS_FIELD_SHIFT)
#define CSL_UDMAP_TR_RESPONSE_CMDID_SHIFT                ((uint32_t) 16U)
#define CSL_UDMAP_TR_RESPONSE_CMDID_MASK                 (((uint32_t) 0xFFU) << CSL_UDMAP_TR_RESPONSE_CMDID_SHIFT)
#define CSL_UDMAP_TR_RESPONSE_CFG_SPECIFIC_SHIFT         ((uint32_t) 24U)
#define CSL_UDMAP_TR_RESPONSE_CFG_SPECIFIC_MASK          (((uint32_t) 0xFFU) << CSL_UDMAP_TR_RESPONSE_CFG_SPECIFIC_SHIFT)
/** @} */

/**
 * \brief This enumerator is used to determine what type of status is being
 *        returned
 *
 *  \anchor CSL_UdmapTrResponseStatus
 *  \name UDMAP TR response status
 *
 *  @{
 */
typedef uint32_t CSL_UdmapTrResponseStatus;
#define CSL_UDMAP_TR_RESPONSE_STATUS_COMPLETE           ((uint32_t) 0U)
#define CSL_UDMAP_TR_RESPONSE_STATUS_TRANSFER_ERR       ((uint32_t) 1U)
#define CSL_UDMAP_TR_RESPONSE_STATUS_ABORTED_ERR        ((uint32_t) 2U)
#define CSL_UDMAP_TR_RESPONSE_STATUS_SUBMISSION_ERR     ((uint32_t) 3U)
#define CSL_UDMAP_TR_RESPONSE_STATUS_UNSUPPORTED_ERR    ((uint32_t) 4U)
/** @} */

/**
 * \brief This enumerator is returned for a TR that is received that can not be
 *        run
 *
 *  \anchor CSL_UdmapTrResponseStatusSubmission
 *  \name UDMAP TR response status submission
 *
 *  @{
 */
typedef uint32_t CSL_UdmapTrResponseStatusSubmission;
/* ICNT0 was 0 */
#define CSL_UDMAP_TR_RESPONSE_STATUS_SUBMISSION_ICNT0       ((uint32_t) 0U)
/* Channel FIFO was full when TR received */
#define CSL_UDMAP_TR_RESPONSE_STATUS_SUBMISSION_FIFO_FULL   ((uint32_t) 1U)
/* Channel is not owned by the submitter */
#define CSL_UDMAP_TR_RESPONSE_STATUS_SUBMISSION_OWN         ((uint32_t) 2U)
/** @} */

/**
 * \brief This enumerator is returned for a TR that is received that can not be
 *        run because it specifies a feature that is optional and not supported
 *        by the UTC that received the TR
 *
 *  \anchor CSL_UdmapTrResponseStatusUnsupported
 *  \name UDMAP TR response status unsupported
 *
 *  @{
 */
typedef uint32_t CSL_UdmapTrResponseStatusUnsupported;
#define CSL_UDMAP_TR_RESPONSE_STATUS_UNSUPPORTED_TR_TYPE        ((uint32_t) 0U)
#define CSL_UDMAP_TR_RESPONSE_STATUS_UNSUPPORTED_STATIC         ((uint32_t) 1U)
#define CSL_UDMAP_TR_RESPONSE_STATUS_UNSUPPORTED_EOL            ((uint32_t) 2U)
#define CSL_UDMAP_TR_RESPONSE_STATUS_UNSUPPORTED_CFG_SPECIFIC   ((uint32_t) 3U)
#define CSL_UDMAP_TR_RESPONSE_STATUS_UNSUPPORTED_AMODE          ((uint32_t) 4U)
#define CSL_UDMAP_TR_RESPONSE_STATUS_UNSUPPORTED_ELTYPE         ((uint32_t) 5U)
#define CSL_UDMAP_TR_RESPONSE_STATUS_UNSUPPORTED_DFMT           ((uint32_t) 6U)
#define CSL_UDMAP_TR_RESPONSE_STATUS_UNSUPPORTED_SECTR          ((uint32_t) 7U)
#define CSL_UDMAP_TR_RESPONSE_STATUS_UNSUPPORTED_AMODE_SPECIFIC ((uint32_t) 8U)
/** @} */

/**
 * \brief This enumerator defines the the type of EOL packet sent in
 *        configurations specific flag in the TR.
 *        This is only valid if TR type(UDMAP_TR_FLAGS_TYPE) is
 *        CSL_UDMAP_TR_FLAGS_TYPE_1D_DATA_MOVE or
 *        CSL_UDMAP_TR_FLAGS_TYPE_2D_DATA_MOVE or
 *        CSL_UDMAP_TR_FLAGS_TYPE_3D_DATA_MOVE or
 *        CSL_UDMAP_TR_FLAGS_TYPE_4D_DATA_MOVE
 *
 *  \anchor CSL_UdmapTrEOLType
 *  \name UDMAP TR EOL packet type
 *
 *  @{
 */
typedef uint32_t CSL_UdmapTrFlagsEol;
/* For Source (Read) split TRs, SOL/EOL match SOP/EOP
 * On destination (Write) split TRs, Ignore EOL */
#define CSL_UDMAP_TR_FLAGS_EOL_MATCH_SOL_EOL                    ((uint32_t) 0U)
/* For Source (Read) split TRs, SOL/EOL boundaries are each ICNT0 bytes
 * On destination (Write) split TRs, Line length is ICNT0 bytes. Clear any remaining ICNT0 bytes and increment ICNT1 by 1 */
#define CSL_UDMAP_TR_FLAGS_EOL_ICNT0                            ((uint32_t) 1U)
/* For Source (Read) split TRs, SOL/EOL boundaries are each ICNT0—ICNT1 bytes
 * On destination (Write) split TRs, Line length is ICNT0—ICNT1 bytes. Clear any remaining ICNT0/1 bytes and increment ICNT2 by 1 */
#define CSL_UDMAP_TR_FLAGS_EOL_ICNT0_ICNT1                      ((uint32_t) 2U)
/* For Source (Read) split TRs, SOL/EOL boundaries are each ICNT0—ICNT1—ICNT2 bytes
 * On destination (Write) split TRs, Line length is ICNT0—ICNT1—ICNT2 bytes. Clear any remaining ICNT0/1/2 bytes and increment ICNT3 by 1 */
#define CSL_UDMAP_TR_FLAGS_EOL_ICNT0_ICNT1_ICNT2                ((uint32_t) 3U)
/* For Source (Read) split TRs, SOL/EOL boundaries are each ICNT0—ICNT1—ICNT2—ICNT3 bytes
 * On destination (Write) split TRs, Line length is ICNT0—ICNT1—ICNT2—ICNT3 bytes. Move on to next TR */
#define CSL_UDMAP_TR_FLAGS_EOL_ICNT0_ICNT1_ICNT2_ICNT3          ((uint32_t) 4U)
/* Reserved */
#define CSL_UDMAP_TR_FLAGS_EOL_RESERVED                         ((uint32_t) 5U)
/** @} */

/**
 *  \brief Return the TR response status type
 *
 *  This function returns the status type of the specified TR response.
 *  See \ref CSL_UdmapTrResponseStatus for available status types.
 *
 *  \param pTrResponse  [IN]    Pointer to the TR Response structure
 *
 *  \return The status type of the specified TR response
 */
static inline CSL_UdmapTrResponseStatus CSL_udmapTrResponseGetStatusType( const CSL_UdmapTrResponse *pTrResponse )
{
    return (CSL_UdmapTrResponseStatus)CSL_FEXT( pTrResponse->flags, UDMAP_TR_RESPONSE_STATUS_TYPE );
}

/*-----------------------------------------------------------------------------
 *  Teardown response support
 *---------------------------------------------------------------------------*/
struct CSL_UdmapTdResponse_t
{
    uint32_t tdIndicator;
    /**< TRUE: Indicates that a teardown has completed. */
    uint32_t chId;
    /**< Indicates which channel the teardown completed on. */
    uint32_t forced;
    /**< FALSE: Indicates that the teardown was graceful and data was not lost.
     *   TRUE : Indicates that the teardown was not graceful and data was
      *  potentially lost */
} __attribute__((__packed__));

typedef struct CSL_UdmapTdResponse_t CSL_UdmapTdResponse;

/**
 *  \anchor CSL_UdmapTeardownResponseFlags_t
 *  \name CSL UDMAP Teardown Response Flags
 *
 *  CSL UDMAP Teardown Response Flags.
 *
 *  @{
 */
#define CSL_UDMAP_TD_RESPONSE_TD_INDICATOR_SHIFT    ((uint32_t) 0U)
#define CSL_UDMAP_TD_RESPONSE_TD_INDICATOR_MASK     (((uint32_t) 0xFU) << CSL_UDMAP_TD_RESPONSE_TD_INDICATOR_SHIFT)
#define CSL_UDMAP_TD_RESPONSE_CHAN_ID_SHIFT         ((uint32_t) 4U)
#define CSL_UDMAP_TD_RESPONSE_CHAN_ID_MASK          (((uint32_t) 0x3FFU) << CSL_UDMAP_TD_RESPONSE_CHAN_ID_SHIFT)
#define CSL_UDMAP_TD_RESPONSE_FORCED_SHIFT          ((uint32_t) 31U)
#define CSL_UDMAP_TD_RESPONSE_FORCED_MASK           (((uint32_t) 0x1U) << CSL_UDMAP_TD_RESPONSE_FORCED_SHIFT)
/** @} */

/**
 *  \brief Parses the TD response word
 *
 *  This function parses the teardown response word received from the
 *  completion queue.
 *
 *  \param tdResponseWord   [IN]    TD Response word to parse
 *  \param pTdResponse      [OUT]   Pointer to the TD Response structure to be filled
 */
static inline void CSL_udmapGetTdResponse(uint64_t tdResponseWord,
                                          CSL_UdmapTdResponse *pTdResponse);
static inline void CSL_udmapGetTdResponse(uint64_t tdResponseWord,
                                          CSL_UdmapTdResponse *pTdResponse)
{
    if(NULL != pTdResponse)
    {
        uint32_t tdResponseWordLow = (uint32_t)tdResponseWord;

        pTdResponse->tdIndicator = CSL_FEXT(tdResponseWordLow, UDMAP_TD_RESPONSE_TD_INDICATOR);
        pTdResponse->chId = CSL_FEXT(tdResponseWordLow, UDMAP_TD_RESPONSE_CHAN_ID);
        pTdResponse->forced = CSL_FEXT(tdResponseWordLow, UDMAP_TD_RESPONSE_FORCED);
    }
}

#ifdef __cplusplus
}
#endif

#endif  /* #ifndef CSL_UDMAP_TR_H_ */
/** @} */
