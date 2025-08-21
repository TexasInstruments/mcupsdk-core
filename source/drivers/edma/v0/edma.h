/*
 * Copyright (C) 2022-24 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 *  \defgroup DRV_EDMA_MODULE APIs for EDMA
 *  \ingroup DRV_MODULE
 *
 *  This module contains APIs to program and use the EDMA.
 *
 *  @{
 */

/**
 *  \file v0/edma.h
 *
 *  \brief This file contains the prototype of EDMA driver APIs
 *
 *  \details  Please find the below detailed description of edma dal.
 *            -# Programming sequence for initializing the edma controller
 *               -# Perform the clock configuration of channel controller and
 *                  transfer controllers.
 *               -# To perform initialization of controller use the
 *                  -# #EDMA_init with appropriate #EDMA_InitParams
 *                     which is part of the #EDMA_Attrs
 *            -# Programming sequence for setting up edma channel.
 *               -# To configure the PaRAM sets use #EDMA_setPaRAM. For
 *                  custom use
 *               -# To start transfer use #EDMA_enableTransferRegion.
 *               -# To stop transfer use #EDMA_disableTransferRegion.
 */

#ifndef EDMA_V0_H_
#define EDMA_V0_H_

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <drivers/hw_include/cslr_edma.h>
#include <drivers/hw_include/cslr_soc.h>
#include <drivers/hw_include/tistdtypes.h>
#include <kernel/dpl/SystemP.h>
#include <kernel/dpl/HwiP.h>

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */
/** \anchor EDMA_QUEUE_NUM_DEFS
*   \name EDMA Queue Number Configuration.
*  @{
*/
#define EDMACC_DMAQNUM_CLR(chNum) \
    (~((uint32_t) 0x7U << (((chNum) % 8U) * 4U)))
/**< DMAQNUM bits Clear */
#define EDMACC_DMAQNUM_SET(chNum, queNum) \
    (((uint32_t) 0x7U & (queNum)) << (((chNum) % 8U) * 4U))
/**< DMAQNUM bits Set */
#define EDMACC_QDMAQNUM_CLR(chNum) \
    (~((uint32_t) 0x7U << ((chNum) * 4U)))
/**< QDMAQNUM bits Clear */
#define EDMACC_QDMAQNUM_SET(chNum, queNum) \
    (((uint32_t) 0x7U & (queNum)) << ((chNum) * 4U))
/**< QDMAQNUM bits Set */
/** @} */

/** \anchor EDMA_QDMA_MAPPING_DEFS
*   \name EDMA Qdma Channel mapping Configuration.
*  @{
*/
#define EDMACC_QCHMAP_PAENTRY_CLR ((uint32_t) (~((uint32_t)EDMA_TPCC_QCHMAPN_PAENTRY_MASK)))
/**< QCHMAP Param ID Clear */
#define EDMACC_QCHMAP_PAENTRY_SET(paRAMId)                               \
    (((EDMA_TPCC_QCHMAPN_PAENTRY_MASK >> EDMA_TPCC_QCHMAPN_PAENTRY_SHIFT) \
      & (paRAMId)) << EDMA_TPCC_QCHMAPN_PAENTRY_SHIFT)                    \
/**< QCHMAP-PaRAMEntry bitfield Set */
#define EDMACC_QCHMAP_TRWORD_CLR  ((uint32_t) (~((uint32_t)EDMA_TPCC_QCHMAPN_TRWORD_MASK)))
/**< QCHMAP-TrigWord bitfield Clear */
#define EDMACC_QCHMAP_TRWORD_SET(paRAMId)                                \
    (((EDMA_TPCC_QCHMAPN_TRWORD_MASK >> EDMA_TPCC_QCHMAPN_TRWORD_SHIFT) & \
      (paRAMId)) << EDMA_TPCC_QCHMAPN_TRWORD_SHIFT)
/**< QCHMAP-TrigWord bitfield Set */
/** @} */

/** \anchor EDMA_BIDX_DEFS
*   \name EDMA PaRAM BIDX configuration helper macros.
*  @{
*/
#define EDMA_PARAM_BIDX(val)   (val & 0xFFFF)
/**< Macro to derive BIDX Lower bits to be programmed in srcBIdx and dstBIdx*/
#define EDMA_PARAM_BIDX_EXT(val)  ((val & 0xFF0000) >> 16)
/**< Macro to derive BIDX Higher bits to be programmed in srcBIdxExt and dstBIdxExt*/
/** @} */

/** \anchor EDMA_TRIGGER_MODE_DEFS
*   \name EDMA Trigger Mode Configuration.
*  @{
*/
#define EDMA_TRIG_MODE_MANUAL                ((uint32_t) 0U)
/**< Manually trigger EDMA transfer */
#define EDMA_TRIG_MODE_QDMA                  ((uint32_t) 1U)
/**< Manually trigger QDMA transfer */
#define EDMA_TRIG_MODE_EVENT                 ((uint32_t) 2U)
/**< Trigger EDMA transfer upon Event */
/** @} */

/** \anchor EDMA_CHANNEL_TYPE_DEFS
*   \name EDMA channel type definitions.
*  @{
*/
/** \brief Values that can be used for parameter chType in API's
*/
#define EDMA_CHANNEL_TYPE_DMA                ((uint32_t) 0U)
/**< Channel Typr DMA */
#define EDMA_CHANNEL_TYPE_QDMA               ((uint32_t) 1U)
/**< Channel Typr QDMA */
/** @} */

/** \anchor EDMA_EVENT_STATUS_DEFS
*   \name EDMA event status definitions.
*  @{
*/
/** \brief Values that can be used to specify different event status
*/
#define EDMA_XFER_COMPLETE                   ((uint32_t) 0U)
/**< Transfer Complete */
#define EDMA_CC_DMA_EVT_MISS                 ((uint32_t) 1U)
/**< DMA Event Miss */
#define EDMA_CC_QDMA_EVT_MISS                ((uint32_t) 2U)
/**< QDMA Event Miss */
/** @} */

/** \anchor EDMA_TPTC_NUM_DEFS
*   \name EDMA TPTC Number definitions
*  @{
*/
/** \brief Values that can be used to specify TPTC Number
*/
#define EDMA_TPTC0                   ((uint32_t) 0U)
/**< EDMA Transfer Controller 0 */
#define EDMA_TPTC1                   ((uint32_t) 1U)
/**< EDMA Transfer Controller 1 */
/** @} */

/** \anchor EDMA_TPTC_ERROR_ENABLE_DEFS
*   \name EDMA TPTC Error Enable/Disabel definitions
*  @{
*/
/** \brief Values that can be used to enable/disable EDMA TPTC Errors
*/
#define EDMA_TPTC_ERROR_DISABLE       ((uint32_t) 0U)
/**< Disable EDMA Transfer Controller Errors */
#define EDMA_TPTC_ERROR_ENABLE        ((uint32_t) 1U)
/**< Enable EDMA Transfer Controller Errors */
/** @} */

/** \anchor EDMA_TRANSFER_TYPE_DEFS
*   \name EDMA transfer type definitions.
*  @{
*/
/**
 *   \brief Values that can be used to specify different
 *   synchronization events
 */
#define EDMA_SYNC_A                          ((uint32_t) 0U)
/**< A Sync Transfer */
#define EDMA_SYNC_AB                         ((uint32_t) 1U)
/** AB Sync Transfer */
/** @} */

/** \anchor EDMA_ADDRESSING_MODE_DEFS
*   \name EDMA addressing modes definitions.
*  @{
*/
/** \brief Values that can be used to specify different
* addressing modes (relevant for SAM and DAM sub-fields in OPT field).
*/
#define EDMA_ADDRESSING_MODE_LINEAR          ((uint32_t) 0U)
/**< Incremental addressing (INCR), not FIFO */
#define EDMA_ADDRESSING_MODE_FIFO_WRAP       ((uint32_t) 1U)
/**< Constant addressing (CONST) within the FIFO array, wraps around upon
    reaching FIFO width */
/** @} */

/** \anchor EDMA_FIFO_WIDTH_DEFS
*   \name EDMA FIFO width definitions.
*  @{
*/
/** \brief Values that can be used to specify different FIFO widths (FWID in OPT field).
*/
#define EDMA_FIFO_WIDTH_8BIT     ((uint32_t) EDMA_TPCC_OPT_FWID_FIFOWIDTH8BIT)
/**< 8-bit FIFO width */
#define EDMA_FIFO_WIDTH_16BIT    ((uint32_t) EDMA_TPCC_OPT_FWID_FIFOWIDTH16BIT)
/**< 16-bit FIFO width */
#define EDMA_FIFO_WIDTH_32BIT    ((uint32_t) EDMA_TPCC_OPT_FWID_FIFOWIDTH32BIT)
/**< 32-bit FIFO width */
#define EDMA_FIFO_WIDTH_64BIT    ((uint32_t) EDMA_TPCC_OPT_FWID_FIFOWIDTH64BIT)
/**< 64-bit FIFO width */
#define EDMA_FIFO_WIDTH_128BIT   ((uint32_t) EDMA_TPCC_OPT_FWID_FIFOWIDTH128BIT)
/**< 128-bit FIFO width */
#define EDMA_FIFO_WIDTH_256BIT   ((uint32_t) DMA_TPCC_OPT_FWID_FIFOWIDTH256BIT)
/**< 256-bit FIFO width */
/** @} */

/** \anchor EDMA_CLEAR_CC_ERROR_DEFS
*   \name EDMA Clear Channel controller Error.
*  @{
*/
/** \brief Values that can be used to Clear any Channel controller Errors
*/
#define EDMACC_CLR_TCCERR         ((uint32_t) EDMA_TPCC_CCERRCLR_TCERR_MASK)
/**< Clear TCC Error */
#define EDMACC_CLR_QTHRQ0         ((uint32_t) EDMA_TPCC_CCERRCLR_QTHRXCD0_MASK)
/**< Clear Queue threshold 0 Error */
#define EDMACC_CLR_QTHRQ1         ((uint32_t) EDMA_TPCC_CCERRCLR_QTHRXCD1_MASK)
/**< Clear Queue threshold 1 Error */
/** @} */

/** \anchor EDMA_CC_ERROR_STATUS_DEFS
*   \name EDMA Channel controller Error status.
*  @{
*/
/** \brief Values that can be used to read any Channel controller Error status
*/
#define EDMACC_TCCERR_STAT         ((uint32_t) EDMA_TPCC_CCERR_TCERR_MASK)
/**< Clear TCC Error */
#define EDMACC_QTHRQ0_STAT         ((uint32_t) EDMA_TPCC_CCERR_QTHRXCD0_MASK)
/**< Clear Queue threshold 0 Error */
#define EDMACC_QTHRQ1_STAT         ((uint32_t) EDMA_TPCC_CCERR_QTHRXCD1_MASK)
/**< Clear Queue threshold 1 Error */
/** @} */

/** \anchor EDMA_PARAM_OPT_FIELD_DEFS
*   \name EDMA Param OPT fields.
*  @{
*/
/** \brief Values that are used to Chain two specified channels */
#define EDMA_OPT_TCCHEN_MASK      ((uint32_t) EDMA_TPCC_OPT_TCCHEN_MASK)
/**< Transfer Complete Chaining enable */
#define EDMA_OPT_ITCCHEN_MASK     ((uint32_t) EDMA_TPCC_OPT_ITCCHEN_MASK)
/**< Intermediate Transfer Complete Chaining enable */
#define EDMA_OPT_TCINTEN_MASK     ((uint32_t) EDMA_TPCC_OPT_TCINTEN_MASK)
/**< Transfer Complete Interrupt enable */
#define EDMA_OPT_ITCINTEN_MASK    ((uint32_t) EDMA_TPCC_OPT_ITCINTEN_MASK)
/**< Intermediate Transfer Complete Interrupt enable */
#define EDMA_OPT_TCC_MASK         ((uint32_t) EDMA_TPCC_OPT_TCC_MASK)
/**< Transfer Complete Code mask */
#define EDMA_OPT_TCC_SHIFT        ((uint32_t) EDMA_TPCC_OPT_TCC_SHIFT)
/**< Transfer Complete Code shift */
#define EDMA_OPT_SYNCDIM_MASK     ((uint32_t) EDMA_TPCC_OPT_SYNCDIM_MASK)
/**< Sync Type shift */
#define EDMA_OPT_SYNCDIM_SHIFT    ((uint32_t) EDMA_TPCC_OPT_SYNCDIM_SHIFT)
/**< Sync Type mask */
#define EDMA_OPT_STATIC_MASK      ((uint32_t) EDMA_TPCC_OPT_STATIC_MASK)
/**< Param Static mask */
#define EDMA_OPT_STATIC_SHIFT     ((uint32_t) EDMA_TPCC_OPT_STATIC_SHIFT)
/**< Param Static mask */
#define EDMACC_OPT_TCC_CLR        ((uint32_t) (~EDMA_TPCC_OPT_TCC_MASK))
/**< OPT-TCC bitfield Clear */
#define EDMACC_OPT_TCC_SET(tcc)                                      \
    (((EDMA_TPCC_OPT_TCC_MASK >> EDMA_TPCC_OPT_TCC_SHIFT) & (tcc)) << \
     EDMA_TPCC_OPT_TCC_SHIFT)
/**< OPT-TCC bitfield Set */
#define EDMA_OPT_SAM_MASK         ((uint32_t) EDMA_TPCC_OPT_SAM_MASK)
/**< Source Addressing Mode mask */
#define EDMA_OPT_SAM_SHIFT        ((uint32_t) EDMA_TPCC_OPT_SAM_SHIFT)
/**< Source Addressing Mode shift */
#define EDMA_OPT_DAM_MASK         ((uint32_t) EDMA_TPCC_OPT_DAM_SHIFT)
/**< Destination Addressing Mode mask */
#define EDMA_OPT_DAM_SHIFT        ((uint32_t) EDMA_TPCC_OPT_DAM_SHIFT)
/**< Destination Addressing Mode shift */
/** @} */

/** \anchor EDMA_PARAM_FIELD_DEFS
*   \name EDMA Param fields.
*  @{
*/
/** \brief PaRAMEntry Fields */
#define    EDMACC_PARAM_ENTRY_OPT            ((uint32_t) 0x0U)
/**< The OPT field (Offset Address 0x0 Bytes) */
#define    EDMACC_PARAM_ENTRY_SRC            ((uint32_t) 0x1U)
/**< The SRC field (Offset Address 0x4 Bytes)*/
#define    EDMACC_PARAM_ENTRY_ACNT_BCNT      ((uint32_t) 0x2U)
/**< The (ACNT+BCNT) field (Offset Address 0x8 Bytes)*/
#define    EDMACC_PARAM_ENTRY_DST            ((uint32_t) 0x3U)
/**< The DST field (Offset Address 0xC Bytes)*/
#define    EDMACC_PARAM_ENTRY_SRC_DST_BIDX   ((uint32_t) 0x4U)
/**< The (SRCBIDX+DSTBIDX) field (Offset Address 0x10 Bytes)*/
#define    EDMACC_PARAM_ENTRY_LINK_BCNTRLD   ((uint32_t) 0x5U)
/**< The (LINK+BCNTRLD) field (Offset Address 0x14 Bytes)*/
#define    EDMACC_PARAM_ENTRY_SRC_DST_CIDX   ((uint32_t) 0x6U)
/**< The (SRCCIDX+DSTCIDX) field (Offset Address 0x18 Bytes)*/
#define    EDMACC_PARAM_ENTRY_CCNT           ((uint32_t) 0x7U)
/**< The (CCNT+RSVD) field (Offset Address 0x1C Bytes)*/
#define    EDMACC_PARAM_FIELD_OFFSET         ((uint32_t) 0x4U)
/**< The offset for each PaRAM Entry field */
#define    EDMACC_PARAM_ENTRY_FIELDS         ((uint32_t) 0x8U)
/**< Number of PaRAM Entry fields.
 *   OPT, SRC, A_B_CNT, DST, SRC_DST_BIDX, LINK_BCNTRLD, SRC_DST_CIDX
 *   and CCNT
 */
#define EDMA_NUM_TCC                        ((uint32_t) SOC_EDMA_NUM_DMACH)
/**< Number of TCC's available */
/** @} */

/** \anchor EDMA_RESOURCE_TYPE_DEFS
*   \name EDMA resource type definition used for resource allocation and freeing.
*  @{
*/
/** \brief Values that can be used for parameter resType in API's
*/
#define EDMA_RESOURCE_TYPE_DMA                ((uint32_t) 0U)
/**< Resource Type DMA */
#define EDMA_RESOURCE_TYPE_QDMA               ((uint32_t) 1U)
/**< Resource Type QDMA */
#define EDMA_RESOURCE_TYPE_TCC                ((uint32_t) 2U)
/**< Resource Type TCC */
#define EDMA_RESOURCE_TYPE_PARAM              ((uint32_t) 3U)
/**< Resource Type PARAM */
#define EDMA_RESOURCE_ALLOC_ANY               ((uint32_t) 0xFFFFU)
/**< Allocate any available resource */
/** @} */

#define EDMA_SET_ALL_BITS                    ((uint32_t) 0xFFFFFFFFU)
/**< Used to set all Bits in a register */
#define EDMA_CLR_ALL_BITS                    ((uint32_t) 0x00000000U)
/**< Used to clear all Bits in a register */
#define EDMACC_COMPL_HANDLER_RETRY_COUNT     ((uint32_t) 10U)
/**< Completion interrupt handler retry count */
#define EDMACC_ERR_HANDLER_RETRY_COUNT       ((uint32_t) 10U)
/**< Error interrupt handler retry count */

/* ========================================================================== */
/*                         Structures                                         */
/* ========================================================================== */
/**
 * \brief EDMA Parameter RAM Set in User Configurable format
 *        This is a mapping of the EDMA PaRAM set provided to the user
 *        for ease of modification of the individual fields
 */
typedef struct {
    /* \brief OPT field of PaRAM Set */
    uint32_t opt;
    /**
     * \brief Starting byte address of Source
     * For FIFO mode, srcAddr must be a 256-bit aligned address.
     */
    uint32_t srcAddr;
    /* \brief Number of bytes in each Array (ACNT) */
    uint16_t aCnt;
    /* \brief Number of Arrays in each Frame (BCNT) */
    uint16_t bCnt;
    /**
     * \brief Starting byte address of destination
     * For FIFO mode, destAddr must be a 256-bit aligned address.
     * i.e. 5 LSBs should be 0.
     */
    uint32_t destAddr;
    /**
     * \brief Index between consec. arrays of a Source Frame (SRCBIDX).
     * SRCBIDX is 24 Bit field. Lower 16 Bits are stored in this field.
     * Higher 8 Bits are stored in srcBIdxExt.
     */
    int16_t  srcBIdx;
    /**
     * \brief Index between consec. arrays of a Destination Frame (DSTBIDX).
     * DSTBIDX is 24 Bit field. Lower 16 Bits are stored in this field.
     * Higher 8 Bits are stored in destBIdxExt.
     */
    int16_t  destBIdx;
    /**
     * \brief Address for linking (AutoReloading of a PaRAM Set)
     * This must point to a valid aligned 32-byte PaRAM set
     * A value of 0xFFFF means no linking
     */
    uint16_t linkAddr;
    /**
     * \brief Reload value of the numArrInFrame (BCNT)
     * Relevant only for A-sync transfers
     */
    uint16_t bCntReload;
    /* \brief Index between consecutive frames of a Source Block (SRCCIDX) */
    int16_t  srcCIdx;
    /* \brief Index between consecutive frames of a Dest Block (DSTCIDX) */
    int16_t  destCIdx;
    /* \brief Number of Frames in a block (CCNT) */
    uint16_t cCnt;
    /* \brief Stores higher 8 Bits of SRCBIDX */
    int8_t  srcBIdxExt;
    /* \brief Stores higher 8 Bits of DSTBIDX */
    int8_t  destBIdxExt;

} __attribute__((packed))
EDMACCPaRAMEntry;

/**
 * \brief EDMA resource allocation structure
 *
 */
typedef struct
{
    /**
     * \brief DMA channels allocated. Each channel will be defined with 1 bit.
     *        ownDmaCh[0] used for 0 to 31, ownDmaCh[1] for 32 to 63 etc.
     */
    uint32_t    dmaCh[SOC_EDMA_NUM_DMACH/32U];
    /* \brief QDMA channels allocated. Each channel will be defined with 1 bit. */
    uint32_t    qdmaCh;
    /**
     * \brief TCCs allocated. Each tcc will be defined with 1 bit.
     *        ownTcc[0] used for 0 to 31, ownTcc[1] for 32 to 63 etc.
     */
    uint32_t    tcc[EDMA_NUM_TCC/32U];
    /**
     * \brief PaRAMs owned. Each PaRAM will be defined with 1 bit.
     *        ownParamSet[0] used for 0 to 31, ownParamSet[1] for 32 to 63 etc.
     */
    uint32_t    paramSet[SOC_EDMA_NUM_PARAMSETS/32U];
} EDMA_ResourceObject;

/**
 * \brief EDMA initialization structure used for EDMAInitialize
 *
 */
typedef struct {
    /* \brief EDMA region to be used */
    uint32_t    regionId;
    /* \brief EDMA Event queue to be used for all channels */
    uint32_t    queNum;
    /* \brief Parameter to reset the PaRAM memory of the owned PaRAMs */
    uint32_t    initParamSet;
    /* \brief owned resource configuration */
    EDMA_ResourceObject ownResource;
    /* \brief Dma channels reserved for Event triggered transfers */
    uint32_t    reservedDmaCh[SOC_EDMA_NUM_DMACH/32U];
}EDMA_InitParams;

/**
 * \brief EDMA Error Info structure used storing TPCC Errors
 *
 */
typedef struct {
    uint32_t    dmaEventMissStatusLow;
    /* \brief Each bit denotes Event miss status for DMA channels 0 - 31 */
    uint32_t    dmaEventMissStatusHigh;
    /* \brief Each bit denotes Event miss status for DMA channels 32 - 63 */
    uint32_t    qdmaEventMissStatus;
    /* \brief Each bit denotes Event miss status for QDMA Channels 0 to 7 */
    uint32_t    isEventQueueThresholdExceeded;
    /* \brief Each bit denotes if Threshold is exceeded for Queues */
    uint32_t    isTccLimitExceeded;
    /* \brief Set to TRUE if TCC limit exceeded for a transfer */
}EDMA_CcErrorInfo;

/**
 * \brief EDMA Error Info structure used storing TPTC Errors
 *
 */
typedef struct {
    uint32_t    isTransferRequestError;
    /* \brief Set to TRUE if a TR is detected that violates transfer rules */
    uint32_t    isInvalidAccessError;
    /* \brief Set to TRUE if an Invalid address write or read access is requested */
    uint32_t    isBusError;
    /* \brief Set to TRUE if a bus error event occurs. Error information is stored in Error code */
    uint32_t    errorCode;
    /* \brief Contains error code for the transaction that caused the error */
    uint32_t    transferCompletionCode;
    /* \brief Contains TCC value for the transaction that caused the error */
    uint32_t    isChainingEnabled;
    /* \brief Set to TRUE if TCCHEN is High in the transaction  that caused the error */
    uint32_t    isTransferInterruptEnabled;
    /* \brief Set to TRUE if TCINTEN is High in the transaction  that caused the error */
}EDMA_TcErrorInfo;

/**
 * \brief EDMA Error Info structure used for Error Handling
 *
 */
typedef struct {
    uint32_t          errorStatus;
    /* \brief EDMA Aggregated Error status value */
    EDMA_CcErrorInfo  ccErrorInfo;
    /* \brief TPCC Error information */
    EDMA_TcErrorInfo  tcErrorInfo[SOC_EDMA_NUM_TPTC];
    /* \brief TPCC Error information */
}EDMA_ErrorInfo;

/**
 * \brief EDMA open parameters passed to #EDMA_open() function.
 */
typedef struct
{
    uint32_t                intrEnable;
    /**< Enable interrupt mode */
    uint32_t                errIntrEnable;
    /**< Enable error interrupt */
} EDMA_Params;

/**
 * \brief EDMA interrupt handle returned from #EDMA_registerIntr() function.
 */
typedef struct Edma_IntrObject_t   *Edma_IntrHandle;

/**
 * \brief EDMA interrupt callback function prototype
 */
typedef void (*Edma_EventCallback)(Edma_IntrHandle intrHandle,
                                   void *appData);

/**
 * \brief EDMA error interrupt callback function prototype
 */
typedef void (*Edma_ErrorCallback)(EDMA_ErrorInfo *errorInfo,
                                   void *appData);

/**
 * \brief EDMA interrupt configuration object. The object is passed to the
 *        #EDMA_registerIntr() function. This is held by the driver till the
 *        #EDMA_unregisterIntr() function is called with this obect. The
 *        application should not modify this object in between.
 *
 */
typedef struct Edma_IntrObject_t
{
    /* \brief TCC number for which the callback to be reistered. */
    uint32_t                  tccNum;
    /* \brief Application data pointer passed to callback function. */
    void                     *appData;
    /* \brief Callback function pointer. */
    Edma_EventCallback        cbFxn;
    /**
     * \brief pointer to next interrupt object. Used internally by driver.
     *        Should not be modified by the application.
     */
    Edma_IntrHandle  nextIntr;
    /**
     * \brief pointer to previous interrupt object. Used internally by driver.
     *        Should not be modified by the application.
     */
    Edma_IntrHandle  prevIntr;
} Edma_IntrObject;

/** \brief A handle that is returned from a #EDMA_open() call */
typedef void *EDMA_Handle;

/**
 *  \brief EDMA driver object
 */
typedef struct
{
    /*
     * User parameters
     */
    EDMA_Handle             handle;
    /**< Instance handle to which this object belongs */
    EDMA_Params             openPrms;
    /**< Init params */
    uint32_t                isOpen;
    /**< Flag to indicate whether the instance is opened already */
    EDMA_ResourceObject     allocResource;
    /**< Object to hold the allocated resources */
    void                   *hwiHandle;
    /**< Interrupt handle for master ISR */
    HwiP_Object             hwiObj;
    /**< Interrupt object */
    void                   *errHwiHandle;
    /**< Interrupt handle for error ISR */
    HwiP_Object             errHwiObj;
    /**< Error Interrupt object */
    Edma_ErrorCallback      errCallback;
    /**< Error Interrupt Callback */
    void*                   errCallbackArgs;
    /**< User arguments for Error Interrupt Callback */
    EDMA_ErrorInfo          errorInfo;
    /**< Error Information passed to application */
    Edma_IntrHandle  firstIntr;
} EDMA_Object;

/** \brief EDMA instance attributes - used during init time */
typedef struct
{
    /*
     * SOC configuration
     */
    uint32_t                baseAddr;
    /**< TPCC base address */
    uint32_t                tcBaseAddr[SOC_EDMA_NUM_TPTC];
    /**< TPCC base address */
    EDMA_InitParams         initPrms;
    /**< Init params */
    uint32_t                compIntrNumber;
    /**< Completion interrupt number. */
    uint8_t                 intrPriority;
    /**< Interrupt priority. */
    uint32_t                errIntrNumber;
    /**< Error interrupt number. */
    uint8_t                 errIntrPriority;
    /**< Error Interrupt priority. */
    uint32_t                intrAggEnableAddr;
    /**< Interrupt Aggregator enable address */
    uint32_t                intrAggEnableMask;
    /**< Interrupt Aggregator enable mask */
    uint32_t                intrAggStatusAddr;
    /**< Interrupt Aggregator enable address */
    uint32_t                intrAggClearMask;
    /**< Interrupt Aggregator clear mask */
    uint32_t                errIntrAggEnableAddr;
    /**< Error Interrupt Aggregator enable address */
    uint32_t                errIntrAggEnableMask;
    /**< Error Interrupt Aggregator enable mask */
    uint32_t                errIntrAggStatusAddr;
    /**< Error Interrupt Aggregator status address */
    uint32_t                errIntrAggRawStatusAddr;
    /**< Error Interrupt Aggregator raw status address */
    uint32_t                errIntrAggClearMask;
    /**< Error Interrupt Aggregator clear mask */
} EDMA_Attrs;

/** \brief EDMA Instance Configuration.
 *         Pointer to this object is returned as handle by driver open.
 */
typedef struct
{
    EDMA_Attrs       *attrs;
    /**< Pointer to driver specific attributes */
    EDMA_Object      *object;
    /**< Pointer to driver specific data object */
} EDMA_Config;

/** \brief Externally defined driver configuration array */
extern EDMA_Config gEdmaConfig[];
/** \brief Externally defined driver configuration array size */
extern uint32_t    gEdmaConfigNum;
/** \brief Externally defined driver init parameters array */
extern EDMA_InitParams gEdmaInitParams[];

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 *  \brief   Structure initialization function for EDMA_InitParams
 *
 *  \param  initParam               Sets default initialization parameter used by API
 *                                  EDMAInitialize. This will set all DMA, QDMA, PaRAMs as own channels.
 *                                  Application should override this with appropriate owned resources.
 *                                  Overriding is must when EDMA is shared by multiple cores.
 *
 */
void EDMA_initParamsInit(EDMA_InitParams *initParam);

/**
 *  \brief   Clear a PaRAM Set .
 *
 *  \param   paramEntry               Parameter RAM set to be cleared.
 *
 */
void EDMA_ccPaRAMEntry_init(EDMACCPaRAMEntry *paramEntry);

/**
 * \brief  Enable channel to Shadow region mapping
 *
 * This API allocates DMA/QDMA channels or TCCs, and the same resources are
 * enabled in the shadow region specific register (DRAE/DRAEH/QRAE).
 * Here only one shadow region is used since, there is only one Master.
 *
 *  \param   baseAddr     Memory address of the EDMA instance used.\n
 *
 *  \param   regionId     Region id to be used.
 *                        Valid values are 0 to SOC_EDMA_NUM_REGIONS-1 \n
 *
 *  \param   chType       (DMA/QDMA) Channel
 *                        For Example: For DMA it is,
 *                        EDMA_CHANNEL_TYPE_DMA.\n
 *
 *  \param   chNum       Allocated channel number.
 *                       Valid values are 0 to SOC_EDMA_NUM_DMACH-1 for DMA
 *                       Valid values are 0 to SOC_EDMA_NUM_QDMACH-1 for QDMA \n
 *  chType can have values
 *        EDMA_CHANNEL_TYPE_DMA\n
 *        EDMA_CHANNEL_TYPE_QDMA
 *
 */
void EDMA_enableChInShadowRegRegion(uint32_t baseAddr,
                                    uint32_t regionId,
                                    uint32_t chType,
                                    uint32_t chNum);

/**
 * \brief  Disable channel to Shadow region mapping
 *
 * This API deallocates DMA/QDMA channels or TCCs, and the same resources are
 * disabled in the shadow region specific register (DRAE/DRAEH/QRAE).
 * Here only one shadow region is used since, there is only one Master.
 *
 * \param   baseAddr   Memory address of the EDMA instance used.\n
 *
 * \param   regionId     Region id to be used.
*                        Valid values are 0 to SOC_EDMA_NUM_REGIONS-1 \n
 *
 * \param   chType      (DMA/QDMA) Channel
 *
 * \param   chNum       Allocated channel number.
 *                      Valid values are 0 to SOC_EDMA_NUM_DMACH-1 for DMA
 *                      Valid values are 0 to SOC_EDMA_NUM_QDMACH-1 for QDMA \n
 *
 *
 *  chType can have values
 *        EDMA_CHANNEL_TYPE_DMA\n
 *        EDMA_CHANNEL_TYPE_QDMA
 *
 */
void EDMA_disableChInShadowRegRegion(uint32_t baseAddr,
                                     uint32_t regionId,
                                     uint32_t chType,
                                     uint32_t chNum);

/**
 *  \brief   This function maps DMA channel to any of the PaRAM sets
 *           in the PaRAM memory map.
 *
 *  \param   baseAddr   Memory address of the EDMA instance used.
 *
 *  \param   channel   The DMA channel number required to be mapped.
 *
 *  \param   paramSet  It specifies the paramSet to which DMA channel
 *                     required to be mapped.
 *                     Valid values are 0 to SOC_EDMA_NUM_PARAMSETS-1
 *
 */
void EDMA_channelToParamMap(uint32_t baseAddr,
                            uint32_t channel,
                            uint32_t paramSet);

/**
 *  \brief  Map channel to Event Queue
 *
 *  This API maps DMA/QDMA channels to the Event Queue
 *
 *  \param  baseAddr    Memory address of the EDMA instance used.\n
 *
 *  \param  chType     (DMA/QDMA) Channel
 *                     For Example: For QDMA it is
 *                     EDMA_CHANNEL_TYPE_QDMA.\n
 *
 *  \param   chNum     Allocated channel number.
 *                     Valid values are 0 to SOC_EDMA_NUM_DMACH-1 for DMA
 *                     Valid values are 0 to SOC_EDMA_NUM_QDMACH-1 for QDMA \n
 *
 *  \param  evtQNum    Event Queue Number to which the channel
 *                     will be mapped (valid only for the
 *                     Master Channel (DMA/QDMA) request).\n
 *
 *  chtype can have values
 *        EDMA_CHANNEL_TYPE_DMA\n
 *        EDMA_CHANNEL_TYPE_QDMA
 *
 */
void EDMA_mapChToEvtQ(uint32_t baseAddr,
                      uint32_t chType,
                      uint32_t chNum,
                      uint32_t evtQNum);

/**
 *  \brief  Remove Mapping of channel to Event Queue
 *
 *  This API Unmaps DMA/QDMA channels from the Event Queue allocated
 *
 *  \param  baseAddr    Memory address of the EDMA instance used.\n
 *
 *  \param  chType     (DMA/QDMA) Channel
 *                     For Example: For DMA it is
 *                     EDMA_CHANNEL_TYPE_DMA.\n
 *
 *  \param   chNum     Allocated channel number.
 *                     Valid values are 0 to SOC_EDMA_NUM_DMACH-1 for DMA
 *                     Valid values are 0 to SOC_EDMA_NUM_QDMACH-1 for QDMA \n
 *
 *  chtype can have values
 *        EDMA_CHANNEL_TYPE_DMA\n
 *        EDMA_CHANNEL_TYPE_QDMA
 *
 */
void EDMA_unmapChToEvtQ(uint32_t baseAddr,
                        uint32_t chType,
                        uint32_t chNum);

/**
 *  \brief  Enables the user to map a QDMA channel to PaRAM set
 *          This API Needs to be called before programming the paRAM sets for
 *          the QDMA Channels.Application needs to maitain the paRAMId
 *          provided by this API.This paRAMId is used to set paRAM and get
 *          paRAM. Refer corresponding API's for more details.
 *
 *  \param  baseAddr                  Memory address of the EDMA instance used.\n
 *
 *  \param   chNum                    Allocated channel number.
 *                                    Valid values are 0 to SOC_EDMA_NUM_QDMACH-1 for QDMA \n
 *
 *  \param  paRAMId                  PaRAM Id to which the QDMA channel will be
 *                                   mapped to.
 *                                   mapped to.
 *
 *  Note : The PaRAMId requested must be greater than 32(SOC_EDMA_NUM_DMACH).
 *         and lesser than SOC_EDMA_NUM_DMACH + chNum  Because, the first
 *         32 PaRAM's are directly mapped to first 32 DMA channels and (32 - 38)
 *         for QDMA Channels. (32 - 38) is assigned by driver in this API.
 *
 */
void EDMA_mapQdmaChToPaRAM(uint32_t        baseAddr,
                           uint32_t        chNum,
                           const uint32_t *paRAMId);

/**
 *  \brief  Returns the PaRAM associated with the DMA/QDMA channel.
 *
 * \param  baseAddr     Memory address of the EDMA instance used.\n
 *
 * \param   chNum       Allocated channel number.
 *                      Valid values are 0 to SOC_EDMA_NUM_DMACH-1 for DMA
 *                      Valid values are 0 to SOC_EDMA_NUM_QDMACH-1 for QDMA \n
 *
 * \param  chType       (DMA/QDMA) Channel
 *                       For Example: For QDMA it is, EDMA_CHANNEL_TYPE_QDMA.\n
 *
 * \param  paramId      Associated paramId is returned in this pointer.
 *
 * \return  TRUE if parameters are valid else return FALSE.
 *
 */
uint32_t EDMA_getMappedPaRAM(uint32_t baseAddr,
                             uint32_t chNum,
                             uint32_t chType,
                             uint32_t *paramId);
/**
 * \brief  Assign a Trigger Word to the specified QDMA channel
 *
 * This API sets the Trigger word for the specific QDMA channel in the QCHMAP
 * Register. Default QDMA trigger word is CCNT.
 *
 * \param  baseAddr             Memory address of the EDMA instance used.\n
 *
 * \param  chNum               QDMA Channel which needs to be assigned
 *                             the Trigger Word.
 *                             Valid values are 0 to SOC_EDMA_NUM_QDMACH-1 for QDMA \n
 *
 * \param  trigWord            The Trigger Word for the QDMA channel.
 *                             Trigger Word is the word in the PaRAM
 *                             Register Set which, when written to by CPU,
 *                             will start the QDMA transfer automatically.
 *
 */
void EDMA_setQdmaTrigWord(uint32_t baseAddr,
                          uint32_t chNum,
                          uint32_t trigWord);

/**
 *  \brief   Enables the user to Clear any missed event
 *
 *  \param   baseAddr                Memory address of the EDMA instance used.\n
 *
 *  \param   regionId               Region id to be used.
 *                                  Valid values are 0 to SOC_EDMA_NUM_REGIONS-1 \n
 *
 *  \param   chNum                  Allocated channel number.\n
 *                                  Valid values are 0 to SOC_EDMA_NUM_DMACH-1 for DMA
 */
void EDMA_clrMissEvtRegion(uint32_t baseAddr, uint32_t regionId, uint32_t chNum);

/**
 *  \brief   Enables the user to Clear any QDMA missed event
 *
 *
 *  \param   baseAddr                Memory address of the EDMA instance used.\n
 *
 *  \param   regionId               Region id to be used.
 *                                  Valid values are 0 to SOC_EDMA_NUM_REGIONS-1 \n
 *
 *  \param   chNum                  Allocated channel number.\n
 *                                  Valid values are 0 to SOC_EDMA_NUM_QDMACH-1 for QDMA
 *
 */
void EDMA_qdmaClrMissEvtRegion(uint32_t baseAddr, uint32_t regionId, uint32_t chNum);

/**
 *  \brief   Enables the user to Clear any Channel controller Errors
 *
 *  \param   baseAddr              Memory address of the EDMA instance used.\n
 *
 *  \param   flags                Masks to be passed.\n
 *
 *  flags can have values:
 *
 *  EDMACC_CLR_TCCERR            Clears the TCCERR bit in the EDMACC
 *                                ERR Reg\n
 *  EDMACC_CLR_QTHRQ0            Queue threshold error clear for queue 0.\n
 *  EDMACC_CLR_QTHRQ1            Queue threshold error clear for queue 1.
 *
 */
void EDMA_clrCCErr(uint32_t baseAddr, uint32_t flags);

/**
 *  \brief   Enables the user to Set an event. This API helps user to manually
 *           set events to initiate DMA transfer requests.
 *
 *  \param   baseAddr                Memory address of the EDMA instance used.\n
 *
 *  \param   regionId               Region id to be used.\n
 *
 *  \param   chNum                  Allocated channel number.
 *                                  Valid values are 0 to SOC_EDMA_NUM_DMACH-1 for DMA
 *
 *  Note :   This API is generally used during Manual transfers.\n
 */
void EDMA_setEvtRegion(uint32_t baseAddr, uint32_t regionId, uint32_t chNum);

/**
 *  \brief   Enables the user to Clear an event.
 *
 *  \param   baseAddr                Memory address of the EDMA instance used.\n
 *
 *  \param   regionId               Region id to be used.
 *                                  Valid values are 0 to SOC_EDMA_NUM_REGIONS-1 \n
 *
 *  \param   chNum                  Allocated channel number.\n
 *                                  Valid values are 0 to SOC_EDMA_NUM_DMACH-1 for DMA
 *
 *  Note :   This API is generally used during Manual transfers.\n
 */
void EDMA_clrEvtRegion(uint32_t baseAddr, uint32_t regionId, uint32_t chNum);

/**
 *  \brief   Enables the user to enable an DMA event.
 *
 *  \param   baseAddr                Memory address of the EDMA instance used.\n
 *
 *  \param   regionId               Region id to be used.
 *                                  Valid values are 0 to SOC_EDMA_NUM_REGIONS-1 \n
 *
 *  \param   chNum                  Allocated channel number.
 *                                  Valid values are 0 to SOC_EDMA_NUM_DMACH-1 for DMA
 *
 *  Note :   Writes of 1 to the bits in EESR sets the corresponding event
 *           bits in EER. This is generally used for Event Based transfers.\n
 */
void EDMA_enableDmaEvtRegion(uint32_t baseAddr, uint32_t regionId, uint32_t chNum);

/**
 *  \brief   Enables the user to Disable an DMA event.
 *
 *  \param   baseAddr                Memory address of the EDMA instance used.\n
 *
 *  \param   regionId               Region id to be used.
 *                                  Valid values are 0 to SOC_EDMA_NUM_REGIONS-1 \n
 *
 *  \param   chNum                  Allocated channel number.
 *                                  Valid values are 0 to SOC_EDMA_NUM_DMACH-1 for DMA
 *
 *  Note :   Writes of 1 to the bits in EECR clear the corresponding event bits
 *           in EER; writes of 0 have no effect.. This is generally used for
 *           Event Based transfers.\n
 */
void EDMA_disableDmaEvtRegion(uint32_t baseAddr, uint32_t regionId, uint32_t chNum);

/**
 *  \brief   Enables the user to enable an QDMA event.
 *
 *  \param   baseAddr                Memory address of the EDMA instance used.\n
 *
 *  \param   regionId               Region id to be used.
 *                                  Valid values are 0 to SOC_EDMA_NUM_REGIONS-1 \n
 *
 *  \param   chNum                  Allocated channel number.
 *                                  Valid values are 0 to SOC_EDMA_NUM_QDMACH-1 for QDMA
 *
 *  Note :   Writes of 1 to the bits in QEESR sets the corresponding event
 *            bits in QEER.\n
 */
void EDMA_enableQdmaEvtRegion(uint32_t baseAddr, uint32_t regionId, uint32_t chNum);

/**
 *  \brief   Enables the user to disable an QDMA event.
 *
 *  \param   baseAddr                Memory address of the EDMA instance used.\n
 *
 *  \param   regionId               Region id to be used.
 *                                  Valid values are 0 to SOC_EDMA_NUM_REGIONS-1 \n
 *
 *  \param   chNum                  Allocated channel number.
 *                                  Valid values are 0 to SOC_EDMA_NUM_QDMACH-1 for QDMA
 *
 *  Note :   Writes of 1 to the bits in QEECR clears the corresponding event
 *            bits in QEER.\n
 */
void EDMA_disableQdmaEvtRegion(uint32_t baseAddr, uint32_t regionId, uint32_t chNum);

/**
 *  \brief   This function returns interrupts status of those events
 *           which is less than 32.
 *
 *  \param   baseAddr                Memory address of the EDMA instance used.\n
 *
 *  \param   regionId               Region id to be used.
 *                                  Valid values are 0 to SOC_EDMA_NUM_REGIONS-1 \n
 *
 * \return   Interrupt status register value
 *
 **/
uint32_t EDMA_getIntrStatusRegion(uint32_t baseAddr, uint32_t regionId);

/**
 *  \brief   Enables the user to enable the transfer completion interrupt
 *           generation by the EDMACC for all DMA/QDMA channels.
 *
 *  \param   baseAddr                Memory address of the EDMA instance used.\n
 *
 *  \param   regionId               Region id to be used.
 *                                  Valid values are 0 to SOC_EDMA_NUM_REGIONS-1 \n
 *
 *  \param   chNum                  Allocated channel number.
 *                                  Valid values are 0 to SOC_EDMA_NUM_DMACH-1 for DMA
 *
 *  Note :   To set any interrupt bit in IER, a 1 must be written to the
 *           corresponding interrupt bit in the interrupt enable set register.
 */
void EDMA_enableEvtIntrRegion(uint32_t baseAddr, uint32_t regionId, uint32_t chNum);

/**
 *  \brief   Enables the user to clear CC interrupts
 *
 *  \param   baseAddr                Memory address of the EDMA instance used.\n
 *
 *  \param   regionId               Region id to be used.
 *                                  Valid values are 0 to SOC_EDMA_NUM_REGIONS-1 \n
 *
 *  \param   chNum                  Allocated channel number.
 *                                  Valid values are 0 to SOC_EDMA_NUM_DMACH-1 for DMA
 *
 *  Note :   Writes of 1 to the bits in IECR clear the corresponding interrupt
 *           bits in the interrupt enable registers (IER); writes of 0 have
 *           no effect.\n
 */
void EDMA_disableEvtIntrRegion(uint32_t baseAddr, uint32_t regionId, uint32_t chNum);

/**
 *  \brief   Enables the user to Clear an Interrupt.
 *
 *  \param   baseAddr                Memory address of the EDMA instance used.
 *
 *  \param   regionId               Region id to be used.
 *                                  Valid values are 0 to SOC_EDMA_NUM_REGIONS-1 \n
 *
 *  \param   value                  Value to be set to clear the Interrupt
 *                                  Status.
 *
 */
void EDMA_clrIntrRegion(uint32_t baseAddr, uint32_t regionId, uint32_t value);

/**
 *  \brief   This function returns interrupt enable status of events which
 *           are less than 32.
 *
 *  \param   baseAddr                Memory address of the EDMA instance used.\n
 *
 *  \param   regionId               Region id to be used.
 *                                  Valid values are 0 to SOC_EDMA_NUM_REGIONS-1 \n
 *
 * \return   Interrupt status register value
 *
 **/
uint32_t EDMA_getEnabledIntrRegion(uint32_t baseAddr, uint32_t regionId);

/**
 *  \brief   This function returns interrupt enable status of events which
 *           are more than 32.
 *
 *  \param   baseAddr                Memory address of the EDMA instance used.\n
 *
 *  \param   regionId               Region id to be used.
 *                                  Valid values are 0 to SOC_EDMA_NUM_REGIONS-1 \n
 *
 * \return   Interrupt status register value
 *
 **/
uint32_t EDMA_getEnabledIntrHighRegion(uint32_t baseAddr, uint32_t regionId);

/**
 *  \brief   Retrieve existing PaRAM set associated with specified logical
 *           channel (DMA/Link).
 *
 *  \param   baseAddr                Memory address of the EDMA instance used.\n
 *
 *  \param   paRAMId                paRAMset ID whose parameter set is
 *                                  requested.
 *                                  Valid values are 0 to SOC_EDMA_NUM_PARAMSETS-1\n
 *
 *  \param   currPaRAM              User gets the existing PaRAM here.\n
 *
 */
void EDMA_getPaRAM(uint32_t           baseAddr,
                   uint32_t           paRAMId,
                   EDMACCPaRAMEntry *currPaRAM);

/**
 * \brief   Retrieve existing PaRAM set associated with specified logical
 *          channel (QDMA).
 *
 * \param   baseAddr                Memory address of the EDMA instance used.\n
 *
 * \param   paRAMId                paRAMset ID whose parameter set is
 *                                 requested.
 *                                 Valid values are 0 to SOC_EDMA_NUM_PARAMSETS-1\n
 *
 * \param   currPaRAM              User gets the existing PaRAM here.\n
 *
 */
void EDMA_qdmaGetPaRAM(uint32_t           baseAddr,
                       uint32_t           paRAMId,
                       EDMACCPaRAMEntry *currPaRAM);

/**
 * \brief   Copy the user specified PaRAM Set onto the PaRAM Set associated
 *          with the logical channel (DMA/Link).
 *
 * This API takes a PaRAM Set as input and copies it onto the actual PaRAM Set
 * associated with the logical channel. OPT field of the PaRAM Set is written
 * first and the CCNT field is written last.
 *
 *
 * \param   baseAddr                Memory address of the EDMA instance used.\n
 *
 * \param   paRAMId                paRAMset ID whose parameter set has to be
 *                                 updated
 *                                 Valid values are 0 to SOC_EDMA_NUM_PARAMSETS-1
 *
 * \param   newPaRAM               Parameter RAM set to be copied onto existing
 *                                 PaRAM.\n
 *
 */
void EDMA_setPaRAM(uint32_t           baseAddr,
                   uint32_t           paRAMId,
                   const EDMACCPaRAMEntry *newPaRAM);

/**
 * \brief   Copy the user specified PaRAM Set onto the PaRAM Set associated
 *          with the logical channel (QDMA only).
 *
 * This API takes a PaRAM Set as input and copies it onto the actual PaRAM Set
 * associated with the logical channel. OPT field of the PaRAM Set is written
 * first and the CCNT field is written last.
 *
 *
 * \param   baseAddr                Memory address of the EDMA instance used.\n
 *
 *
 * \param  paRAMId                 paRaMset ID whose parameter set has to be
 *                                 updated
 *                                 Valid values are 0 to SOC_EDMA_NUM_PARAMSETS-1
 *
 * \param   newPaRAM               Parameter RAM set to be copied onto existing
 *                                 PaRAM.\n
 *
 */
void EDMA_qdmaSetPaRAM(uint32_t           baseAddr,
                       uint32_t           paRAMId,
                       const EDMACCPaRAMEntry *newPaRAM);

/**
 * \brief   Set a particular PaRAM set entry of the specified PaRAM set
 *
 * \param   baseAddr           Memory address of the EDMA instance used.\n
 *
 * \param   paRAMId           PaRAM Id to which the QDMA channel is
 *                            mapped to.
 *
 * \param   paRAMEntry        Specify the PaRAM set entry which needs
 *                            to be set.
 *
 * \param   newPaRAMEntryVal  The new field setting. Make sure this field is
 *                            packed for setting certain fields in paRAM.
 *
 *  EDMACC_PARAM_ENTRY_OPT
 *  EDMACC_PARAM_ENTRY_SRC
 *  EDMACC_PARAM_ENTRY_ACNT_BCNT
 *  EDMACC_PARAM_ENTRY_DST
 *  EDMACC_PARAM_ENTRY_SRC_DST_BIDX
 *  EDMACC_PARAM_ENTRY_LINK_BCNTRLD
 *  EDMACC_PARAM_ENTRY_SRC_DST_CIDX
 *  EDMACC_PARAM_ENTRY_CCNT
 *
 * \note    This API should be used while setting the PaRAM set entry
 *          for QDMA channels. If EDMAQdmaSetPaRAMEntry() used,
 *          it will trigger the QDMA channel before complete
 *          PaRAM set entry is written.
 */
void EDMA_qdmaSetPaRAMEntry(uint32_t baseAddr,
                            uint32_t paRAMId,
                            uint32_t paRAMEntry,
                            uint32_t newPaRAMEntryVal);

/**
 * \brief   Get a particular PaRAM entry of the specified PaRAM set
 *
 * \param   baseAddr           Memory address of the EDMA instance used.\n
 *
 * \param   paRAMId           PaRAM Id to which the QDMA channel is
 *                            mapped to.
 *
 * \param   paRAMEntry        Specify the PaRAM set entry which needs
 *                            to be read.
 *
 *  paRAMEntry can have values:
 *
 *  EDMACC_PARAM_ENTRY_OPT
 *  EDMACC_PARAM_ENTRY_SRC
 *  EDMACC_PARAM_ENTRY_ACNT_BCNT
 *  EDMACC_PARAM_ENTRY_DST
 *  EDMACC_PARAM_ENTRY_SRC_DST_BIDX
 *  EDMACC_PARAM_ENTRY_LINK_BCNTRLD
 *  EDMACC_PARAM_ENTRY_SRC_DST_CIDX
 *  EDMACC_PARAM_ENTRY_CCNT
 *
 * \return  paRAMEntryVal     The value of the paRAM field pointed by the
 *                            paRAMEntry.
 *
 * \note    This API should be used while reading the PaRAM set entry
 *          for QDMA channels. And the paRAMEntryVal is a packed value for
 *          certain fields of paRAMEntry.The user has to make sure the value
 *          is unpacked appropriately.
 *          For example, the third field is A_B_CNT. Hence he will have to
 *          unpack it to two 16 bit fields to get ACNT and BCNT.
 */
uint32_t EDMA_qdmaGetPaRAMEntry(uint32_t baseAddr,
                                uint32_t paRAMId,
                                uint32_t paRAMEntry);

/**
 * \brief   Set a particular PaRAM set entry of the specified PaRAM set
 *
 * \param   baseAddr           Memory address of the EDMA instance used.\n
 *
 * \param   paRAMId           PaRAM Id to which the DMA channel is
 *                            mapped to.
 *
 * \param   paRAMEntry        Specify the PaRAM set entry which needs
 *                            to be set.
 *
 * \param   newPaRAMEntryVal  The new field setting. Make sure this field is
 *                            packed for setting certain fields in paRAM.
 *
 *  EDMACC_PARAM_ENTRY_OPT
 *  EDMACC_PARAM_ENTRY_SRC
 *  EDMACC_PARAM_ENTRY_ACNT_BCNT
 *  EDMACC_PARAM_ENTRY_DST
 *  EDMACC_PARAM_ENTRY_SRC_DST_BIDX
 *  EDMACC_PARAM_ENTRY_LINK_BCNTRLD
 *  EDMACC_PARAM_ENTRY_SRC_DST_CIDX
 *  EDMACC_PARAM_ENTRY_CCNT
 *
 * \note    This API should be used while setting the PaRAM set entry
 *          for DMA channels.
 */
void EDMA_dmaSetPaRAMEntry(uint32_t baseAddr,
                            uint32_t paRAMId,
                            uint32_t paRAMEntry,
                            uint32_t newPaRAMEntryVal);

/**
 * \brief   Get a particular PaRAM entry of the specified PaRAM set
 *
 * \param   baseAddr           Memory address of the EDMA instance used.\n
 *
 * \param   paRAMId           PaRAM Id to which the DMA channel is
 *                            mapped to.
 *
 * \param   paRAMEntry        Specify the PaRAM set entry which needs
 *                            to be read.
 *
 *  paRAMEntry can have values:
 *
 *  EDMACC_PARAM_ENTRY_OPT
 *  EDMACC_PARAM_ENTRY_SRC
 *  EDMACC_PARAM_ENTRY_ACNT_BCNT
 *  EDMACC_PARAM_ENTRY_DST
 *  EDMACC_PARAM_ENTRY_SRC_DST_BIDX
 *  EDMACC_PARAM_ENTRY_LINK_BCNTRLD
 *  EDMACC_PARAM_ENTRY_SRC_DST_CIDX
 *  EDMACC_PARAM_ENTRY_CCNT
 *
 * \return  paRAMEntryVal     The value of the paRAM field pointed by the
 *                            paRAMEntry.
 *
 * \note    This API should be used while reading the PaRAM set entry
 *          for DMA channels. And the paRAMEntryVal is a packed value for
 *          certain fields of paRAMEntry.The user has to make sure the value
 *          is unpacked appropriately.
 *          For example, the third field is A_B_CNT. Hence he will have to
 *          unpack it to two 16 bit fields to get ACNT and BCNT.
 */
uint32_t EDMA_dmaGetPaRAMEntry(uint32_t baseAddr,
                                uint32_t paRAMId,
                                uint32_t paRAMEntry);

/**
 *  \brief Request a DMA/QDMA/Link channel.
 *
 *  Each channel (DMA/QDMA/Link) must be requested  before initiating a DMA
 *  transfer on that channel.
 *
 *  This API is used to allocate a logical channel (DMA/QDMA/Link) along with
 *  the associated resources. For DMA and QDMA channels, TCC and PaRAM Set are
 *  also allocated along with the requested channel.
 *
 *  User can request a specific logical channel by passing the channel number
 *  in 'chNum'.
 *
 *  For DMA/QDMA channels, after allocating all the EDMA resources, this API
 *  sets the TCC field of the OPT PaRAM Word with the allocated TCC. It also
 *  sets the event queue for the channel allocated. The event queue needs to
 *  be specified by the user.
 *
 *  For DMA channel, it also sets the DCHMAP register.
 *
 *  For QDMA channel, it sets the QCHMAP register and CCNT as trigger word and
 *  enables the QDMA channel by writing to the QEESR register.
 *
 *  \param  baseAddr                  Memory address of the EDMA instance used.\n
 *
 *  \param   regionId                 Region id to be used.
 *                                    Valid values are 0 to SOC_EDMA_NUM_REGIONS-1 \n
 *
 *  \param  chType                   (DMA/QDMA) Channel
 *                                    For Example: For DMA it is
 *                                    EDMA_CHANNEL_TYPE_DMA.\n
 *
 *  \param  chNum                    This is the channel number requested for a
 *                                   particular event.
 *                                   Valid values are 0 to SOC_EDMA_NUM_DMACH-1 for DMA
 *                                   Valid values are 0 to SOC_EDMA_NUM_QDMACH-1 for QDMA \n
 *
 *  \param  tccNum                   The tcc number on which the
 *                                   completion/error interrupt is generated.
 *                                   Not used if user requested for a Link
 *                                   channel.\n
 *
 *  \param  paramId                  The param number currosponding to the
 *                                   DMA/QDMA channel.\n
 *
 *  \param  evtQNum                  Event Queue Number to which the channel
 *                                   will be mapped (valid only for the
 *                                   Master Channel (DMA/QDMA) request).\n
 *
 *  \return  TRUE if parameters are valid, else FALSE
 */
uint32_t EDMA_configureChannelRegion(uint32_t baseAddr,
                                     uint32_t regionId,
                                     uint32_t chType,
                                     uint32_t chNum,
                                     uint32_t tccNum,
                                     uint32_t paramId,
                                     uint32_t evtQNum);

/**
 *  \brief    Free the specified channel (DMA/QDMA/Link) and its associated
 *            resources (PaRAM Set, TCC etc) and removes various mappings.
 *
 *  For Link channels, this API only frees the associated PaRAM Set.
 *
 *  For DMA/QDMA channels, it does the following operations:
 *  1) Disable any ongoing transfer on the channel,\n
 *  2) Remove the channel to Event Queue mapping,\n
 *  3) For DMA channels, clear the DCHMAP register, if available\n
 *  4) For QDMA channels, clear the QCHMAP register,\n
 *  5) Frees the DMA/QDMA channel in the end.\n
 *
 *  \param  baseAddr                  Memory address of the EDMA instance used.\n
 *
 *  \param   regionId                 Region id to be used.
 *                                    Valid values are 0 to SOC_EDMA_NUM_REGIONS-1 \n
 *
 *  \param  chType                    (DMA/QDMA) Channel
 *                                      For Example: For QDMA it is,
 *                                      EDMA_CHANNEL_TYPE_QDMA.\n
 *
 *  \param  chNum                    This is the channel number requested for a
 *                                   particular event.
 *                                   Valid values are 0 to SOC_EDMA_NUM_DMACH-1 for DMA
 *                                   Valid values are 0 to SOC_EDMA_NUM_QDMACH-1 for QDMA \n
 *
 *  \param  trigMode                 Mode of triggering start of transfer.\n
 *
 *  \param  tccNum                   The channel number on which the
 *                                   completion/error interrupt is generated.
 *                                   Not used if user requested for a Link
 *                                   channel.\n
 *
 *  \param  evtQNum                  Event Queue Number to which the channel
 *                                   will be unmapped (valid only for the
 *                                   Master Channel (DMA/QDMA) request).\n
 *
 *  trigMode can have values:
 *        EDMA_TRIG_MODE_MANUAL\n
 *        EDMA_TRIG_MODE_QDMA\n
 *        EDMA_TRIG_MODE_EVENT
 *
 *  \return  TRUE if parameters are valid else return FALSE
 */
uint32_t EDMA_freeChannelRegion(uint32_t baseAddr,
                                uint32_t regionId,
                                uint32_t chType,
                                uint32_t chNum,
                                uint32_t trigMode,
                                uint32_t tccNum,
                                uint32_t evtQNum);

/**
 *  \brief    Start EDMA transfer on the specified channel.
 *
 *  There are multiple ways to trigger an EDMA transfer. The triggering mode
 *  option allows choosing from the available triggering modes: Event,
 *  Manual or QDMA.
 *
 *  In event triggered, a peripheral or an externally generated event triggers
 *  the transfer. This API clears the Event and Event Miss Register and then
 *  enables the DMA channel by writing to the EESR.
 *
 *  In manual triggered mode, CPU manually triggers a transfer by writing a 1
 *  in the Event Set Register ESR. This API writes to the ESR to start the
 *  transfer.
 *
 *  In QDMA triggered mode, a QDMA transfer is triggered when a CPU (or other
 *  EDMA programmer) writes to the trigger word of the QDMA channel PaRAM set
 *  (auto-triggered) or when the EDMACC performs a link update on a PaRAM set
 *  that has been mapped to a QDMA channel (link triggered). This API enables
 *  the QDMA channel by writing to the QEESR register.
 *
 *  \param  baseAddr         Memory address of the EDMA instance used.\n
 *
 *  \param   regionId       Region id to be used.
*                           Valid values are 0 to SOC_EDMA_NUM_REGIONS-1 \n
 *
 *  \param  chNum           Channel being used to enable transfer.
 *                          Valid values are 0 to SOC_EDMA_NUM_DMACH-1 for DMA
 *
 *  \param  trigMode        Mode of triggering start of transfer (Manual,
 *                          QDMA or Event).\n
 *
 *  trigMode can have values:
 *        EDMA_TRIG_MODE_MANUAL\n
 *        EDMA_TRIG_MODE_QDMA\n
 *        EDMA_TRIG_MODE_EVENT\n
 *
 *  \return  retVal         TRUE or FALSE depending on the param passed.\n
 *
 */
uint32_t EDMA_enableTransferRegion(uint32_t baseAddr,
                                   uint32_t regionId,
                                   uint32_t chNum,
                                   uint32_t trigMode);

/**
 *  \brief   Disable DMA transfer on the specified channel
 *
 *  There are multiple ways by which an EDMA transfer could be triggered.
 *  The triggering mode option allows choosing from the available triggering
 *  modes.
 *
 *  To disable a channel which was previously triggered in manual mode,
 *  this API clears the Secondary Event Register and Event Miss Register,
 *  if set, for the specific DMA channel.
 *
 *  To disable a channel which was previously triggered in QDMA mode, this
 *  API clears the QDMA Event Enable Register, for the specific QDMA channel.
 *
 *  To disable a channel which was previously triggered in event mode, this API
 *  clears the Event Enable Register, Event Register, Secondary Event Register
 *  and Event Miss Register, if set, for the specific DMA channel.
 *
 *
 *  \param  baseAddr         Memory address of the EDMA instance used.\n
 *
 *  \param   regionId       Region id to be used.
 *                          Valid values are 0 to SOC_EDMA_NUM_REGIONS-1 \n
 *
 *  \param  chNum           Channel being used to enable transfer.
 *                          Valid values are 0 to SOC_EDMA_NUM_DMACH-1 for DMA
 *
 *  \param  trigMode        Mode of triggering start of transfer (Manual,
 *                          QDMA or Event).\n
 *  \return  retVal         TRUE or FALSE depending on the param passed.\n
 *
 */
uint32_t EDMA_disableTransferRegion(uint32_t baseAddr,
                                    uint32_t regionId,
                                    uint32_t chNum,
                                    uint32_t trigMode);

/**
 *  \brief  Clears Event Register and Error Register for a specific
 *          DMA channel and brings back EDMA to its initial state.
 *
 *  This API clears the Event register, Event Miss register, Event Enable
 *  register for a specific DMA channel. It also clears the CC Error register.
 *
 *  \param  baseAddr         Memory address of the EDMA instance used.\n
 *
 *  \param   regionId       Region id to be used.
 *                          Valid values are 0 to SOC_EDMA_NUM_REGIONS-1 \n
 *
 *  \param  chNum           This is the channel number requested for a
 *                          particular event.
 *                          Valid values are 0 to SOC_EDMA_NUM_DMACH-1 for DMA\n
 *
 *  \param  evtQNum         Event Queue Number to which the channel
 *                          will be unmapped (valid only for the
 *                          Master Channel (DMA/QDMA) request).\n
 *
 */
void EDMA_clearErrorBitsRegion(uint32_t baseAddr,
                               uint32_t regionId,
                               uint32_t chNum,
                               uint32_t evtQNum);

/**
 *  \brief   This returns EDMA CC error status.
 *
 *  \param   baseAddr                Memory address of the EDMA instance used.\n
 *
 *  \return  value                  Status of the Interrupt Pending Register
 *
 */
uint32_t EDMA_getCCErrStatus(uint32_t baseAddr);

/**
 *  \brief   This returns error interrupt status for those events whose
 *           event number is less than 32.
 *
 *  \param   baseAddr                Memory address of the EDMA instance used.\n
 *
 *  \return  value                  Status of the Interrupt Pending Register
 *
 */
uint32_t EDMA_getErrIntrStatus(uint32_t baseAddr);

/**
 *  \brief   This returns QDMA error interrupt status.
 *
 *  \param   baseAddr            Memory address of the EDMA instance used.\n
 *
 *  \return  value              Status of the QDMA Interrupt Pending Register
 *
 */
uint32_t EDMA_qdmaGetErrIntrStatus(uint32_t baseAddr);

/**
 * \brief   This API return the revision Id of the peripheral.
 *
 * \param   baseAddr     Memory address of the EDMA instance used.\n
 *
 * \return  value        Revision ID of the peripheral
 **/
uint32_t EDMA_peripheralIdGet(uint32_t baseAddr);

/**
 *  \brief   This function returns interrupt status of those events
 *           which are greater than 32.
 *
 *  \param   baseAddr                Memory address of the EDMA instance used.\n
 *
 *  \param   regionId                Region id to be used.\n
 *
 *  \return  value                   Status of the Interrupt Pending High Register
 *
 **/
uint32_t EDMA_intrStatusHighGetRegion(uint32_t baseAddr, uint32_t regionId);

/**
 *  \brief   This function reads interrupt status.
 *
 *  \param   baseAddr                Memory address of the EDMA instance used.\n
 *
 *  \param   regionId                Region id to be used.\n
 *
 *  \param   tccNum                  The channel number on which the
 *                                   completion/error interrupt is generated.
 *
 *  \return  value                   Status of the Interrupt Pending register
 *
 **/
uint32_t EDMA_readIntrStatusRegion(uint32_t baseAddr, uint32_t regionId, uint32_t tccNum);

/**
 *  \brief   This function returns status of those events
 *           which are less than 32.
 *
 *  \param   baseAddr                Memory address of the EDMA instance used.\n
 *
 * \return   value                   Event pending status register value
 *
 **/
uint32_t EDMA_getEventStatus(uint32_t baseAddr);


/**
 *  \brief   This function returns status of those events
 *           which are greater than 32.
 *
 *  \param   baseAddr                Memory address of the EDMA instance used.\n
 *
 *  \return  value                   Status of the Event Pending High Register
 *
 **/
uint32_t EDMA_getEventStatusHigh(uint32_t baseAddr);

/**
 *  \brief   This function reads Event pending status.
 *
 *  \param   baseAddr                Memory address of the EDMA instance used.\n
 *
 *  \param   chNum                   Channel being used for transfer.
 *                                   Valid values are 0 to SOC_EDMA_NUM_DMACH-1 for DMA
 *
 *  \return  value                   Status of the Event Pending register
 *
 **/
uint32_t EDMA_readEventStatusRegion(uint32_t baseAddr, uint32_t chNum);

/**
 *  \brief   This returns error interrupt status for those events whose
 *           event number is greater than 32.
 *
 *  \param   baseAddr                Memory address of the EDMA instance used.\n
 *
 *  \return  value                  Status of the Interrupt Pending Register
 *
 */
uint32_t EDMA_errIntrHighStatusGet(uint32_t baseAddr);

/**
 *  \brief  Chain the two specified channels
 *
 *  This API is used to chain a DMA channel to a previously allocated DMA/QDMA
 *  channel
 *
 *  Chaining is different from Linking. The EDMA link feature reloads the
 *  current channel parameter set with the linked parameter set. The EDMA
 *  chaining feature does not modify or update any channel parameter set;
 *  it provides a synchronization event (or trigger) to the chained DMA channel,
 *  as soon as the transfer (final or intermediate) completes on the main
 *  DMA/QDMA channel.
 *
 *  \param  baseAddr         Memory address of the EDMA instance used.\n
 *
 *  \param  paRAMId1        PaRAM set ID of physical channel1 to which
 *                          particular paRAM set will be chained
 *                          or
 *                          PaRAM set ID in case another PaRAM set is being
 *                          chained to this PaRAM set
 *
 *  \param  chId2           DMA channel which needs to be chained to
 *                          the first DMA/QDMA channel.
 *
 *  \param   chainOptions   combination of the following masks which control
 *                          individual PaRAM OPT fields related to
 *                          intermediate/final completion chaining and
 *                          intermediate/final completion interrupt :
 *                            - EDMA_OPT_TCCHEN_MASK
 *                            - EDMA_OPT_ITCCHEN_MASK
 *                            - EDMA_OPT_TCINTEN_MASK
 *                            - EDMA_OPT_ITCINTEN_MASK
 *                          e.g to enable final completion chaining and enable
 *                          interrupt only for intermediate completion,
 *                          set chainOptions as :
 *                          EDMA_OPT_TCCHEN_MASK | EDMA_OPT_ITCINTEN_MASK
 *
 */
void EDMA_chainChannel(uint32_t baseAddr,
                       uint32_t paRAMId1,
                       uint32_t chId2,
                       uint32_t chainOptions);

/**
 *  \brief  Link two channels.
 *
 *  This API is used to link two previously allocated logical (DMA/QDMA/Link)
 *  channels.
 *
 *  It sets the Link field of the PaRAM set associated with first
 *  channel (chId1) to point it to the PaRAM set associated with second
 *  channel (chId2).
 *
 *  It also sets the TCC field of PaRAM set of second channel to the
 *  same as that of the first channel.
 *
 *  \param  baseAddr         Memory address of the EDMA instance used.\n
 *
 *  \param  paRAMId1        PaRAM set ID of physical channel1 to which
 *                          particular paRAM set will be linked
 *                          or
 *                          PaRAM set ID in case another PaRAM set is being
 *                          linked to this PaRAM set
 *
 *  \param  paRAMId2        PaRAM set ID which is linked to
 *                          channel with parameter ID paRAMId1
 *
 *                          After the transfer based on the PaRAM set
 *                          of channel1 is over, the PaRAM set paRAMId2 will
 *                          be copied to the PaRAM set of channel1 and
 *                          transfer will resume.
 *                          For DMA channels, another sync event is
 *                          required to initiate the transfer on the
 *                          Link channel.
 *
 */
void EDMA_linkChannel(uint32_t baseAddr, uint32_t paRAMId1, uint32_t paRAMId2);

/**
 *  \brief  This function initializes the EDMA driver object and controller
 *
 */
void EDMA_init(void);

/**
 *  \brief  This function Deinitializes the EDMA driver object and controller
 *
 */
void EDMA_deinit(void);

/**
 *  \brief  Function to check if EDMA is enabled or not.
 *
 *  \pre    #EDMA_open() has to be called first
 *
 *  \param  handle      #EDMA_Handle returned from #EDMA_open()
 *
 *  \return TRUE or FALSE.
 *
 *  \sa     #EDMA_open()
 */
uint32_t EDMA_isInitialized(EDMA_Handle handle);

/**
 *  \brief  This function opens a given EDMA instance.
 *
 *  \param  index       Index of config to use in the *EDMA_Config* array
 *  \param  prms        Pointer to open parameters.
 *
 *  \return A #EDMA_Handle on success or a NULL on an error or if it has been
 *          opened already
 *
 */
EDMA_Handle EDMA_open(uint32_t index, const EDMA_Params *prms);

/**
 *  \brief  This function returns the handle of an open EDMA Instance from the instance index
 *
 *  \pre    EDMA instance has been opened using #EDMA_open()
 *
 *  \param  index Index of config to use in the *EDMA_Config* array
 *
 *  \return A #EDMA_Handle if it has been opened already or NULL otherwise
 *
 *  \sa     #EDMA_init()
 *  \sa     #EDMA_open()
 */
EDMA_Handle EDMA_getHandle(uint32_t index);

/**
 *  \brief  Function to close a EDMA peripheral specified by the EDMA handle
 *
 *  \pre    #EDMA_open() has to be called first
 *
 *  \param  handle      #EDMA_Handle returned from #EDMA_open()
 *
 *  \sa     #EDMA_open()
 */
void EDMA_close(EDMA_Handle handle);

/**
 *  \brief  Function to check if EDMA interrupt is enabled.
 *
 *  \pre    #EDMA_open() has to be called first
 *
 *  \param  handle      #EDMA_Handle returned from #EDMA_open()
 *
 *  \sa     #EDMA_open()
 */
uint32_t EDMA_isInterruptEnabled(EDMA_Handle handle);

/**
 *  \brief  Function to register callback function for a TCC
 *
 *  \pre    #EDMA_open() has to be called first
 *
 *  \param  handle      #EDMA_Handle returned from #EDMA_open()
 *  \param  intrObj     Pointer to interrupt parameters. intrObj will be held
 *                      by the driver till #EDMA_unregisterIntr() is called
 *                      with the same intrObj and application should not modify
 *                      the memory allocated to intrObj.
 *
 *  \return #SystemP_SUCCESS if started successfully; else error on failure
 *
 *  \sa     #EDMA_open()
 */
int32_t EDMA_registerIntr(EDMA_Handle handle, Edma_IntrObject *intrObj);

/**
 *  \brief  Function to unregister callback function for a TCC
 *
 *  \pre    #EDMA_registerIntr() has to be called first
 *
 *  \param  handle      #EDMA_Handle returned from #EDMA_open()
 *  \param  intrObj     Pointer to interrupt parameters object. This is the
 *                      same object passed to #EDMA_registerIntr() for interrupt
 *                      registeration.
 *
 *  \return #SystemP_SUCCESS if started successfully; else error on failure
 *
 *  \sa     #EDMA_open()
 *  \sa     #EDMA_registerIntr()
 */
int32_t EDMA_unregisterIntr(EDMA_Handle handle, Edma_IntrObject *intrObj);

/**
 *  \brief  Function to get the edma base address
 *
 *  \pre    #EDMA_open() has to be called first
 *
 *  \param  handle      #EDMA_Handle returned from #EDMA_open()
 *
 *  \return edma base address if successful;
 *          0 (NULL) on failure
 *
 *  \sa     #EDMA_open()
 */
uint32_t EDMA_getBaseAddr(EDMA_Handle handle);

/**
 *  \brief  Function to get the edma region
 *
 *  \pre    #EDMA_open() has to be called first
 *
 *  \param  handle      #EDMA_Handle returned from #EDMA_open()
 *
 *  \return region Id corresponding to the edma handle if successful
 *          SOC_EDMA_NUM_REGIONS upon failure which is invalid region number.
 *          valid region numbers are from 0 to (SOC_EDMA_NUM_REGIONS - 1)
 *
 *  \sa     #EDMA_open()
 */
uint32_t EDMA_getRegionId(EDMA_Handle handle);

/**
 *  \brief  Function to allocate the Dma Channel
 *
 *  \pre    #EDMA_open() has to be called first
 *
 *  \param  handle      #EDMA_Handle returned from #EDMA_open()
 *  \param  dmaCh       Set the value to DMA channel number to be allocated.
 *                      Set to EDMA_RESOURCE_ALLOC_ANY to allocate any owned
 *                      channel. Allocated channel number is stored in this.
 *
 *  \return #SystemP_SUCCESS if started successfully; else error on failure
 *
 *  \sa     #EDMA_open()
 */
int32_t EDMA_allocDmaChannel(EDMA_Handle handle, uint32_t *dmaCh);

/**
 *  \brief  Function to allocate the Qdma Channel
 *
 *  \pre    #EDMA_open() has to be called first
 *
 *  \param  handle      #EDMA_Handle returned from #EDMA_open()
 *  \param  qdmaCh      Set the value to DMA channel number to be allocated.
 *                      Set to EDMA_RESOURCE_ALLOC_ANY to allocate any owned
 *                      channel. Allocated channel number is stored in this.
 *
 *  \return #SystemP_SUCCESS if started successfully; else error on failure
 *
 *  \sa     #EDMA_open()
 */
int32_t EDMA_allocQdmaChannel(EDMA_Handle handle, uint32_t *qdmaCh);

/**
 *  \brief  Function to allocate Tcc
 *
 *  \pre    #EDMA_open() has to be called first
 *
 *  \param  handle      #EDMA_Handle returned from #EDMA_open()
 *  \param  tcc         Set the value to QDMA channel number to be allocated.
 *                      Set to EDMA_RESOURCE_ALLOC_ANY to allocate any owned
 *                      Qdma channel. Allocated channel number is stored in this.
 *
 *  \return #SystemP_SUCCESS if started successfully; else error on failure
 *
 *  \sa     #EDMA_open()
 */
int32_t EDMA_allocTcc(EDMA_Handle handle, uint32_t *tcc);

/**
 *  \brief  Function to allocate Param Set
 *
 *  \pre    #EDMA_open() has to be called first
 *
 *  \param  handle      #EDMA_Handle returned from #EDMA_open()
 *  \param  param       Set the value to TCC to be allocated.
 *                      Set to EDMA_RESOURCE_ALLOC_ANY to allocate any owned
 *                      TCC. Allocated TCC number is stored in this.
 *
 *  \return #SystemP_SUCCESS if started successfully; else error on failure
 *
 *  \sa     #EDMA_open()
 */
int32_t EDMA_allocParam(EDMA_Handle handle, uint32_t *param);

/**
 *  \brief  Function to free the Dma Channel
 *
 *  \pre    #EDMA_open() has to be called first
 *
 *  \param  handle      #EDMA_Handle returned from #EDMA_open()
 *  \param  dmaCh       dma channel allocated using the call to
 *                      #EDMA_allocDmaChannel()
 *
 *  \return #SystemP_SUCCESS if started successfully; else error on failure
 *
 *  \sa     #EDMA_open()
 */
int32_t EDMA_freeDmaChannel(EDMA_Handle handle, uint32_t *dmaCh);

/**
 *  \brief  Function to free the Qdma Channel
 *
 *  \pre    #EDMA_open() has to be called first
 *
 *  \param  handle      #EDMA_Handle returned from #EDMA_open()
 *  \param  qdmaCh      qdma channel allocated using the call to
 *                      #EDMA_allocQdmaChannel()
 *
 *  \return #SystemP_SUCCESS if started successfully; else error on failure
 *
 *  \sa     #EDMA_open()
 */
int32_t EDMA_freeQdmaChannel(EDMA_Handle handle, uint32_t *qdmaCh);

/**
 *  \brief  Function to free the tcc Channel
 *
 *  \pre    #EDMA_open() has to be called first
 *
 *  \param  handle      #EDMA_Handle returned from #EDMA_open()
 *  \param  tcc         tcc channel allocated using the call to
 *                      #EDMA_allocTcc()
 *
 *  \return #SystemP_SUCCESS if started successfully; else error on failure
 *
 *  \sa     #EDMA_open()
 */
int32_t EDMA_freeTcc(EDMA_Handle handle, uint32_t *tcc);

/**
 *  \brief  Function to free the Param
 *
 *  \pre    #EDMA_open() has to be called first
 *
 *  \param  handle      #EDMA_Handle returned from #EDMA_open()
 *  \param  param       param allocated using the call to
 *                      #EDMA_allocParam()
 *
 *  \return #SystemP_SUCCESS if started successfully; else error on failure
 *
 *  \sa     #EDMA_open()
 */
int32_t EDMA_freeParam(EDMA_Handle handle, uint32_t *param);

/**
 *  \brief  Function to register error callback function
 *
 *  \pre    #EDMA_open() has to be called first
 *
 *  \param  handle         #EDMA_Handle returned from #EDMA_open()
 *  \param  errorCallback  Application callback for EDMA errors
 *  \param  args           Arguments passed from application to callback function
 *
 *  \return #SystemP_SUCCESS if registered successfully; else error on failure
 *
 *  \sa     #EDMA_open()
 */
int32_t EDMA_registerErrorCallback(EDMA_Handle handle, Edma_ErrorCallback errorCallback, void* args);

/**
 *  \brief  Function to unregister error callback
 *
 *  \pre    #EDMA_open() has to be called first
 *
 *  \param  handle         #EDMA_Handle returned from #EDMA_open()
 *
 *  \return #SystemP_SUCCESS if unregistered successfully; else error on failure
 *
 *  \sa     #EDMA_open()
 */
int32_t EDMA_unregisterErrorCallback(EDMA_Handle handle);

#ifdef __cplusplus
}
#endif

#endif  /* #ifndef EDMA_V0_H_ */

/** @} */

