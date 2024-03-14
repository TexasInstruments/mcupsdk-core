/**
 * @file  csl_cpdma.h
 *
 * @brief
 *  Header file containing various enumerations, structure definitions and function
 *  declarations for the CPDMA submodule of EMAC (CPSW).
 *
 *  ============================================================================
 *  @n   (C) Copyright 2020, Texas Instruments, Inc.
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

#ifndef CSL_CPDMA_H
#define CSL_CPDMA_H

#ifdef __cplusplus
extern "C" {
#endif

#include <cslr_cpdma.h>
#include <enet_cfg.h>
/** ============================================================================
 *
 * @defgroup CSL_CPDMA_API_V0 Ethernet CPDMA submodule (CPDMA - V0)
 * @ingroup CSL_CPSW_API
 *
 * ============================================================================
 */
/**
@defgroup CSL_CPDMA_SYMBOL  CPDMA Symbols Defined
@ingroup CSL_CPDMA_API_V0
*/
/**
@defgroup CSL_CPDMA_DATASTRUCT  CPDMA Data Structures
@ingroup CSL_CPDMA_API_V0
*/
/**
@defgroup CSL_CPDMA_FUNCTION  CPDMA Functions
@ingroup CSL_CPDMA_API_V0
*/
/**
@defgroup CSL_CPDMA_ENUM CPDMA Enumerated Data Types
@ingroup CSL_CPDMA_API_V0
*/
/** @addtogroup CSL_CPDMA_DATASTRUCT
 @{ */

/** @brief
 *
 *  Holds the Ethernet switch CPDMA Tx/Rx subsystem's version info.
 */
typedef struct {
    /**  Minor version value */
    Uint32      minorVer;

    /**  Major version value */
    Uint32      majorVer;

    /**  RTL version value */
    Uint32      rtlVer;

    /**  Identification value */
    Uint32      id;
} CSL_CPSW_CPDMA_VERSION;

/** @brief
 *
 *  Defines CPSW CPDMA EOI indications
 */
typedef uint32_t CSL_CPSW_CPDMA_EOI_IND;
#define CPSW_CPDMA_EOI_RX_THRESH     ((uint32_t) 0U)
#define CPSW_CPDMA_EOI_RX            ((uint32_t) 1U)
#define CPSW_CPDMA_EOI_TX            ((uint32_t) 2U)
#define CPSW_CPDMA_EOI_MISC          ((uint32_t) 3U)

/** @brief
 *
 *  Defines CPSW CPDMA Transmit Queue Priority Type
 */
typedef uint32_t CSL_CPSW_CPDMA_TX_PTYPE;
/** The queue uses a round robin scheme to select the next channel
    for transmission */
#define CPSW_CPDMA_TX_PTYPE_ROUND_ROBIN     ((uint32_t) 0U)
/** The queue uses a fixed (channel 7 highest priority) priority scheme
    to select the next channel for transmission */
#define CPSW_CPDMA_TX_PTYPE_FIXED           ((uint32_t) 1U)

/** @brief
 *
 *  Defines CPSW CPDMA Receive Ownership BIT
 */
typedef uint32_t CSL_CPSW_CPDMA_RX_OWNERSHIP_BIT;
/** The CPDMA writes the receive ownership bit to zero at the end of
    packet processing */
#define CPSW_CPDMA_RX_OWNERSHIP_BIT_0       ((uint32_t) 0U)
/** The CPDMA writes the receive ownership bit to one at the end of
    packet processing */
#define CPSW_CPDMA_RX_OWNERSHIP_BIT_1       ((uint32_t) 1U)

/** @brief
 *
 *  Defines CPSW CPDMA Transmit Rate Limit Channel Bus
 */
typedef uint32_t CSL_CPSW_CPDMA_TX_RLIM_TYPE;
#define CPSW_CPDMA_TX_RLIM_NONE             ((uint32_t) 0x00U)
#define CPSW_CPDMA_TX_RLIM_CH7              ((uint32_t) 0x80U)
#define CPSW_CPDMA_TX_RLIM_CH7_6            ((uint32_t) 0xC0U)
#define CPSW_CPDMA_TX_RLIM_CH7_5            ((uint32_t) 0xE0U)
#define CPSW_CPDMA_TX_RLIM_CH7_4            ((uint32_t) 0xF0U)
#define CPSW_CPDMA_TX_RLIM_CH7_3            ((uint32_t) 0xF8U)
#define CPSW_CPDMA_TX_RLIM_CH7_2            ((uint32_t) 0xFCU)
#define CPSW_CPDMA_TX_RLIM_CH7_1            ((uint32_t) 0xFEU)
#define CPSW_CPDMA_TX_RLIM_CH7_0            ((uint32_t) 0xFFU)

/** @brief
 *
 *  Holds the CPSW CPDMA control register info.
 */
typedef struct {
    /**  Transmit Queue Priority Type */
    CSL_CPSW_CPDMA_TX_PTYPE         txPtype;

    /** Receive Ownership Write Bit Value */
    CSL_CPSW_CPDMA_RX_OWNERSHIP_BIT rxOnwBit;

    /**  Receive Offset/Length word write block Enable: When set, Block all CPDMA DMA controller
         writes to the receive buffer descriptor offset/buffer length words during CPPI packet
         processing
      */
    uint32_t                        rxOffLenBlockEn;

    /**  Command Idle:
         0: Idle not commanded
         1: Idle commanded
      */
    uint32_t                        idleCmd;

    /** RX Copy Error Frames Enable: When set, Enables DMA overrun frames to be transferred to memory
        (up to the point of overrun). The overrun error bit will be set in the frame EOP buffer descriptor
      */
    uint32_t                        rxCEFEn;

    /** Receive VLAN Encapsulation Enable:
        0: VLAN is not included at the receive buffer
        1: VLAN is included at the receive buffer
      */
    uint32_t                        rxVLANEn;

    /** Receive Timestamp Encapsulation Enable:
        0: Timestamp is not included at the receive buffer
        1: Timestamp is included at the receive buffer
      */
    uint32_t                        rxTSEn;

    /** Transmit Rate Limit Channel Bus configuration */
    CSL_CPSW_CPDMA_TX_RLIM_TYPE     txRlimType;

} CSL_CPSW_CPDMA_CONFIG;

/** @brief
 *
 *  Defines CPSW CPDMA Transmit Host Errors
 */
typedef uint32_t CSL_CPSW_CPDMA_TX_HOST_ERR;
/** No error */
#define CPSW_CPDMA_TX_HOST_ERR_NOERR        ((uint32_t) 0U)
/** SOP Error */
#define CPSW_CPDMA_TX_HOST_ERR_SOP          ((uint32_t) 1U)
/** Ownership bit not set in SOP buffer */
#define CPSW_CPDMA_TX_HOST_ERR_OWNER        ((uint32_t) 2U)
/** Zero Next Buffer Descriptor Pointer Without EOP */
#define CPSW_CPDMA_TX_HOST_ERR_ZERO_NEXT_BD ((uint32_t) 3U)
/** Zero Buffer Pointer */
#define CPSW_CPDMA_TX_HOST_ERR_ZERO_BP      ((uint32_t) 4U)
/**  Zero buffer length on non-SOP descriptor */
#define CPSW_CPDMA_TX_HOST_ERR_ZERO_BLEN    ((uint32_t) 5U)
/** Packet Length Error (sum of buffers is less than packet length */
#define CPSW_CPDMA_TX_HOST_ERR_PKT_LEN      ((uint32_t) 6U)

/** @brief
 *
 *  Defines CPSW CPDMA Receive Host Errors
 */
typedef uint32_t CSL_CPSW_CPDMA_RX_HOST_ERR;
/** No error */
#define CPSW_CPDMA_RX_HOST_ERR_NOERR        ((uint32_t) 0U)
/** Ownership bit not set in input buffer */
#define CPSW_CPDMA_RX_HOST_ERR_OWNER        ((uint32_t) 2U)
/** Zero Buffer Pointer */
#define CPSW_CPDMA_RX_HOST_ERR_ZERO_BP      ((uint32_t) 4U)
/**  Zero buffer length on non-SOP descriptor */
#define CPSW_CPDMA_RX_HOST_ERR_ZERO_BLEN    ((uint32_t) 5U)
/**  SOP buffer length not greater than offset  */
#define CPSW_CPDMA_RX_HOST_ERR_SOP_BLEN     ((uint32_t) 6U)

/** Interrupt Events per core */
#define CPSW_CPDMA_NUM_INT_EVENTS_PER_CORE (4U)
/** Get EOI ack value based on coreId and interrupt */
#define CPSW_CPDMA_GET_CORE_EOI(intr_type, coreId)  (intr_type + (CPSW_CPDMA_NUM_INT_EVENTS_PER_CORE * coreId))

/** @brief
 *
 *  Holds the CPSW CPDMA status register info.
 */
typedef struct {
    /**  Rx Host Error Channel */
    uint32_t                    rxErrCh;

    /** Rx Host Error Code */
    CSL_CPSW_CPDMA_RX_HOST_ERR  rxErrCode;

    /**  Tx Host Error Channel */
    uint32_t                    txErrCh;

    /** Rx Host Error Code */
    CSL_CPSW_CPDMA_TX_HOST_ERR  txErrCode;

    /** Idle Status: when set, the CPDMA is not transferring a packet on
        transmit or receive.
      */
    uint32_t                    idle;

} CSL_CPSW_CPDMA_STATUS;

/**
@}
*/

/** @addtogroup CSL_CPSW_FUNCTION
@{ */

/********************************************************************************
***************************** Ethernet CPDMA Submodule **************************
********************************************************************************/

/**
 * \brief    This function retrieves the ethernet switch CPDMA Tx version
 *           information.
 *
 * \param   hCpdmaRegs    Pointer to CSL_CpdmaRegs structure
 * \param   pVersionInfo  Pointer to CSL_CPSW_CPDMA_VERSION structure that needs to
 *                        be populated with the version info read from the hardware
 * \return  None
 */
void CSL_CPSW_getCpdmaTxVersionInfo (CSL_CpdmaRegs           *hCpdmaRegs,
                                     CSL_CPSW_CPDMA_VERSION  *pVersionInfo);

/**
 * \brief    This function retrieves the ethernet switch CPDMA Rx version
 *           information.
 *
 * \param   hCpdmaRegs    Pointer to CSL_CpdmaRegs structure
 * \param   pVersionInfo  Pointer to CSL_CPSW_CPDMA_VERSION structure that needs to
 *                        be populated with the version info read from the hardware
 * \return  None
 */
void CSL_CPSW_getCpdmaRxVersionInfo (CSL_CpdmaRegs           *hCpdmaRegs,
                                     CSL_CPSW_CPDMA_VERSION  *pVersionInfo);

/**
 * \brief   Configures the CPDMA module by writing the configuration value
 *          to the DMA control register.
 *
 * \param   hCpdmaRegs    Pointer to CSL_CpdmaRegs structure
 * \param   pConfig       Pointer to CSL_CPSW_CPDMA_CONFIG to be populated
 *                        to contents of the CPDMA control register
 *
 * \return  None
 *
 **/
void CSL_CPSW_configCpdma
(
    CSL_CpdmaRegs           *hCpdmaRegs,
    CSL_CPSW_CPDMA_CONFIG   *pConfig
);

/**
 * \brief   This function retreives the contents of CPDMA status register.
 *
 * \param   hCpdmaRegs    Pointer to CSL_CpdmaRegs structure
 * \param   pStatusInfo   Pointer to CSL_CPSW_CPDMA_STATUS to be retrieved
 *                        from contents of the CPDMA status register
 * \return  None
 *
 **/
void CSL_CPSW_getCpdmaStatus
(
    CSL_CpdmaRegs           *hCpdmaRegs,
    CSL_CPSW_CPDMA_STATUS   *pStatusInfo
);

/**
 * \brief   This function retrieves the reset startus from the CPSW Softreset register.
 *
 * \param   hCpdmaRegs    Pointer to CSL_CpdmaRegs structure
 *
 * \return  TRUE if CPDMA softreset is complete.
 *
 */
Uint32 CSL_CPSW_isCpdmaResetDone
(
    CSL_CpdmaRegs           *hCpdmaRegs
);

/** ============================================================================
 * \brief   This function triggers the CPDMA softreset.
 *
 * \param   hCpdmaRegs    Pointer to CSL_CpdmaRegs structure
 *
 * \return  None
 *
 */
void CSL_CPSW_resetCpdma
(
    CSL_CpdmaRegs           *hCpdmaRegs
);

/**
 * \brief   This function returns the Tx teardown status.
 *
 * \param   hCpdmaRegs    Pointer to CSL_CpdmaRegs structure
 *
 * \return  Tx teardown status
 *
 */
Uint32 CSL_CPSW_isCpdmaTxTeardownReady
(
    CSL_CpdmaRegs           *hCpdmaRegs
);

/**
 * \brief   This function triggers channel teardown operation for the specified
 *          Tx channel
 *
 * \param   hCpdmaRegs    Pointer to CSL_CpdmaRegs structure
 * \param   channel       Channel number to be torn down
 *
 * \return  None
 *
 **/
void CSL_CPSW_cpdmaTxTeardown
(
    CSL_CpdmaRegs           *hCpdmaRegs,
    Uint32 channel
);

/**
 * \brief   This function returns the Rx teardown status.
 *
 * \param   hCpdmaRegs    Pointer to CSL_CpdmaRegs structure
 *
 * \return  Rx teardown status
 *
 */
Uint32 CSL_CPSW_isCpdmaRxTeardownReady
(
    CSL_CpdmaRegs           *hCpdmaRegs
);

/**
 * \brief   This function triggers channel teardown operation for the specified
 *          Rx channel
 *
 * \param   hCpdmaRegs    Pointer to CSL_CpdmaRegs structure
 * \param   channel       Channel number to be torn down
 *
 * \return  None
 *
 **/
void CSL_CPSW_cpdmaRxTeardown
(
    CSL_CpdmaRegs           *hCpdmaRegs,
    Uint32 channel
);

/**
 * \brief   Enables the TXPULSE Interrupt Generation.
 *
 * \param   hCpdmaRegs    Pointer to CSL_CpdmaRegs structure
 * \param   channel       Channel number for which interrupt to be enabled
 *
 * \return  None
 *
 **/
void CSL_CPSW_enableCpdmaTxInt
(
    CSL_CpdmaRegs           *hCpdmaRegs,
    Uint32 channel
);

/**
 * \brief   Enables the RXPULSE Interrupt Generation.
 *
 * \param   hCpdmaRegs    Pointer to CSL_CpdmaRegs structure
 * \param   channel       Channel number for which interrupt to be enabled
 *
 * \return  None
 *
 **/
void CSL_CPSW_enableCpdmaRxInt
(
    CSL_CpdmaRegs           *hCpdmaRegs,
    Uint32 channel
);

/**
 * \brief   Enables the CPDMA Miscellaneous Interrupt Generation.
 *
 * \param   hCpdmaRegs    Pointer to CSL_CpdmaRegs structure
 * \param   bitMask       Takes
 *                        CSL_CPDMA_DMA_INTMASK_SET_STAT_INT_MASK_MASK |
 *                        CSL_CPDMA_DMA_INTMASK_SET_HOST_ERR_INT_MASK_MASK
 *
 * \return  None
 *
 **/
void CSL_CPSW_enableCpdmaDmaInt
(
    CSL_CpdmaRegs           *hCpdmaRegs,
    Uint32 bitMask
);

/**
 * \brief   Enables the CPSW Stats Overflow Interrupt Generation.
 *
 * \param   hCpdmaRegs    Pointer to CSL_CpdmaRegs structure
 *
 * \return  None
 *
 **/
void CSL_CPSW_enableCpdmaStatsInt
(
    CSL_CpdmaRegs           *hCpdmaRegs
);

/**
 * \brief   Enables the CPDMA Host Error Interrupt Generation.
 *
 * \param   hCpdmaRegs    Pointer to CSL_CpdmaRegs structure
 *
 * \return  None
 *
 **/
void CSL_CPSW_enableCpdmaHostErrInt
(
    CSL_CpdmaRegs           *hCpdmaRegs
);

/**
 * \brief   Disables the TXPULSE Interrupt Generation.
 *
 * \param   hCpdmaRegs    Pointer to CSL_CpdmaRegs structure
 * \param   channel       Channel number for which interrupt to be disabled
 *
 * \return  None
 *
 **/
void CSL_CPSW_disableCpdmaTxInt
(
    CSL_CpdmaRegs           *hCpdmaRegs,
    Uint32 channel
);

/**
 * \brief   Disables the RXPULSE Interrupt Generation.
 *
 * \param   hCpdmaRegs    Pointer to CSL_CpdmaRegs structure
 * \param   channel       Channel number for which interrupt to be disabled
 *
 * \return  None
 *
 **/
void CSL_CPSW_disableCpdmaRxInt
(
    CSL_CpdmaRegs           *hCpdmaRegs,
    Uint32 channel
);

/**
 * \brief   yDisables the CPDMA Miscellaneous Interrupt Generation.
 *
 * \param   hCpdmaRegs    Pointer to CSL_CpdmaRegs structure
 * \param   bitMask       Takes
 *                        CSL_CPDMA_DMA_INTMASK_CLEAR_STAT_INT_MASK_MASK |
 *                        CSL_CPDMA_DMA_INTMASK_CLEAR_HOST_ERR_INT_MASK_MASK
 *
 * \return  None
 *
 **/
void CSL_CPSW_disableCpdmaDmaInt
(
    CSL_CpdmaRegs           *hCpdmaRegs,
    Uint32 bitMask
);

/**
 * \brief   Disables the CPSW Stats Overflow Interrupt Generation.
 *
 * \param   hCpdmaRegs    Pointer to CSL_CpdmaRegs structure
 *
 * \return  None
 *
 **/
void CSL_CPSW_disableCpdmaStatsInt
(
    CSL_CpdmaRegs           *hCpdmaRegs
);

/**
 * \brief   Disables the CPDMA Host Error Interrupt Generation.
 *
 * \param   hCpdmaRegs    Pointer to CSL_CpdmaRegs structure
 *
 * \return  None
 *
 **/
void CSL_CPSW_disableCpdmaHostErrInt
(
    CSL_CpdmaRegs           *hCpdmaRegs
);

/**
 * \brief   This function enables the CPDMA transmit operation.
 *          After the transmit is enabled, any write to TXHDP of
 *          a channel will start transmission
 *
 * \param   hCpdmaRegs    Pointer to CSL_CpdmaRegs structure
 *
 * \return  None
 *
 **/
void CSL_CPSW_enableCpdmaTx
(
    CSL_CpdmaRegs           *hCpdmaRegs
);

/**
 * \brief   This function enables the CPDMA receive operation.
 *          After the receive is enabled, any write to RXHDP of
 *          a channel, the data can be received in the destination
 *          specified by the corresponding RX buffer descriptor.
 *
 * \param   hCpdmaRegs    Pointer to CSL_CpdmaRegs structure
 *
 * \return  None
 *
 **/
void CSL_CPSW_enableCpdmaRx
(
    CSL_CpdmaRegs           *hCpdmaRegs
);

#if ENET_CFG_IS_ON(CPDMA_CH_OVERRIDE)
/**
 * \brief   This function set the thost_ch_override bit in the CPDMA Control Register.
 *
 * \param   hCpdmaRegs    Pointer to CSL_CpdmaRegs structure
 *
 * \return  None
 *
 **/
void CSL_CPSW_enableCpdmaChOverride
(
    CSL_CpdmaRegs           *hCpdmaRegs
);
#endif

/**
 * \brief   API to write the TX HDP register. If transmit is enabled,
 *          write to the TX HDP will immediately start transmission.
 *          The data will be taken from the buffer pointer of the TX buffer
 *          descriptor queue starting from the TX HDP
 *
 * \param   hCpdmaRegs    Pointer to CSL_CpdmaRegs structure
 * \param   descHdr       Address of the TX buffer descriptor
 * \param   channel       Channel Number
 *
 * \return  None
 *
 **/
void CSL_CPSW_setCpdmaTxHdrDescPtr
(
    CSL_CpdmaRegs           *hCpdmaRegs,
    Uint32                   descHdr,
    Uint32                   channel
);

/**
 * \brief   API to write the RX HDP register. If receive is enabled,
 *          write to the RX HDP will enable packet reception to point to
 *          the corresponding RX buffer descriptor queue starting
 *          from the RX HDP.
 *
 * \param   hCpdmaRegs    Pointer to CSL_CpdmaRegs structure
 * \param   descHdr       Address of the RX buffer descriptor
 * \param   channel       Channel Number
 *
 * \return  None
 *
 **/
static inline void CSL_CPSW_setCpdmaRxHdrDescPtr
(
    CSL_CpdmaRegs            *hCpdmaRegs,
    Uint32                    descHdr,
    Uint32                    channel
);/* for misra warnings*/
static inline void CSL_CPSW_setCpdmaRxHdrDescPtr
(
    CSL_CpdmaRegs            *hCpdmaRegs,
    Uint32                    descHdr,
    Uint32                    channel
)
{
    hCpdmaRegs->SRAM.RX_HDP[channel] = descHdr;
}

/**
 * \brief   This function writes Tx indication to the DMA End Of Interrupt
 *          Vector to terminate tx interrupt.
 *
 * \param   hCpdmaRegs    Pointer to CSL_CpdmaRegs structure
 *
 * \return  None
 *
 **/
static inline void CSL_CPSW_setCpdmaTxEndOfIntVector
(
    CSL_CpdmaRegs           *hCpdmaRegs,
    uint32_t                 coreId
);/* for misra warnings*/
static inline void CSL_CPSW_setCpdmaTxEndOfIntVector
(
    CSL_CpdmaRegs           *hCpdmaRegs,
    uint32_t                 coreId
)
{
    /* Acknowledge the CPDMA */
    hCpdmaRegs->INTERRUPT.DMA_EOI_VECTOR = CPSW_CPDMA_GET_CORE_EOI(CPSW_CPDMA_EOI_TX, coreId);
}

/**
 * \brief   This function writes Rx indication to the DMA End Of Interrupt
 *          Vector to terminate rx interrupt.
 *
 * \param   hCpdmaRegs    Pointer to CSL_CpdmaRegs structure
 *
 * \return  None
 *
 **/
static inline void CSL_CPSW_setCpdmaRxEndOfIntVector
(
    CSL_CpdmaRegs           *hCpdmaRegs,
    uint32_t                 coreId
);/* for misra warnings*/
static inline void CSL_CPSW_setCpdmaRxEndOfIntVector
(
    CSL_CpdmaRegs           *hCpdmaRegs,
    uint32_t                 coreId
)
{
    /* Acknowledge the CPDMA */
    hCpdmaRegs->INTERRUPT.DMA_EOI_VECTOR = CPSW_CPDMA_GET_CORE_EOI(CPSW_CPDMA_EOI_RX, coreId);
}

/**
 * \brief   This function writes Rx Threshold indication to the DMA End Of
 *          Interrupt Vector to terminate rx Thresh interrupt.
 *
 * \param   hCpdmaRegs    Pointer to CSL_CpdmaRegs structure
 *
 * \return  None
 *
 **/
static inline void CSL_CPSW_setCpdmaRxThresholdEndOfIntVector
(
    CSL_CpdmaRegs           *hCpdmaRegs,
    uint32_t                 coreId
);/* for misra warnings*/
static inline void CSL_CPSW_setCpdmaRxThresholdEndOfIntVector
(
    CSL_CpdmaRegs           *hCpdmaRegs,
    uint32_t                 coreId
)
{
    /* Acknowledge the CPDMA */
    hCpdmaRegs->INTERRUPT.DMA_EOI_VECTOR = CPSW_CPDMA_GET_CORE_EOI(CPSW_CPDMA_EOI_RX_THRESH, coreId);
}

/**
 * \brief   This function writes Misc indication to the DMA End Of Interrupt
 *          Vector to terminate Misc interrupt.
 *
 * \param   hCpdmaRegs    Pointer to CSL_CpdmaRegs structure
 *
 * \return  None
 *
 **/
static inline void CSL_CPSW_setCpdmaMiscEndOfIntVector
(
    CSL_CpdmaRegs           *hCpdmaRegs,
    uint32_t                 coreId
);/* for misra warnings*/
static inline void CSL_CPSW_setCpdmaMiscEndOfIntVector
(
    CSL_CpdmaRegs           *hCpdmaRegs,
    uint32_t                 coreId
)
{
    /* Acknowledge the CPDMA */
    hCpdmaRegs->INTERRUPT.DMA_EOI_VECTOR = CPSW_CPDMA_GET_CORE_EOI(CPSW_CPDMA_EOI_MISC, coreId);
}

/**
 * \brief   This function writes the the TX Completion Pointer of a specific channel
 *
 * \param   hCpdmaRegs    Pointer to CSL_CpdmaRegs structure
 * \param   channel       Channel Number.
 * \param   comPtr        Completion Pointer Value to be written
 *
 * \return  None
 *
 **/
static inline void CSL_CPSW_setCpdmaTxCp
(
    CSL_CpdmaRegs           *hCpdmaRegs,
    Uint32                   channel,
    Uint32                   comPtr
);/* for misra warnings*/
static inline void CSL_CPSW_setCpdmaTxCp
(
    CSL_CpdmaRegs           *hCpdmaRegs,
    Uint32                   channel,
    Uint32                   comPtr
)
{
	hCpdmaRegs->SRAM.TX_CP[channel] = comPtr;
}

/**
 * \brief   This functiojn writes the the RX Completion Pointer of a specific channel
 *
 * \param   hCpdmaRegs    Pointer to CSL_CpdmaRegs structure
 * \param   channel       Channel Number.
 * \param   comPtr        Completion Pointer Value to be written
 *
 * \return  None
 *
 **/
static inline void CSL_CPSW_setCpdmaRxCp
(
    CSL_CpdmaRegs           *hCpdmaRegs,
    Uint32                   channel,
    Uint32                   comPtr
);/* for misra warnings*/
static inline void CSL_CPSW_setCpdmaRxCp
(
    CSL_CpdmaRegs           *hCpdmaRegs,
    Uint32                   channel,
    Uint32                   comPtr
)
{
    hCpdmaRegs->SRAM.RX_CP[channel] = comPtr;
}

/**
 * \brief   This function reads the the TX Completion Pointer of a specific channel
 *
 * \param   hCpdmaRegs    Pointer to CSL_CpdmaRegs structure
 * \param   channel       Channel Number.
 * \param   comPtr        Pointer to store the Completion Pointer Value
 *
 * \return  None
 *
 **/
static inline void CSL_CPSW_getCpdmaTxCp
(
    CSL_CpdmaRegs           *hCpdmaRegs,
    Uint32                   channel,
    Uint32                  *comPtr
);/* for misra warnings*/
static inline void CSL_CPSW_getCpdmaTxCp
(
    CSL_CpdmaRegs           *hCpdmaRegs,
    Uint32                   channel,
    Uint32                  *comPtr
)
{
    *comPtr = hCpdmaRegs->SRAM.TX_CP[channel];
}

/**
 * \brief   This function reads the the RX Completion Pointer of a specific channel
 *
 * \param   hCpdmaRegs    Pointer to CSL_CpdmaRegs structure
 * \param   channel       Channel Number.
 * \param   comPtr        Pointer to store the Completion Pointer Value
 *
 * \return  None
 *
 **/
static inline void CSL_CPSW_getCpdmaRxCp
(
    CSL_CpdmaRegs           *hCpdmaRegs,
    Uint32                    channel,
    Uint32                   *comPtr
);/* for misra warnings*/
static inline void CSL_CPSW_getCpdmaRxCp
(
    CSL_CpdmaRegs           *hCpdmaRegs,
    Uint32                    channel,
    Uint32                   *comPtr
)
{
    *comPtr = hCpdmaRegs->SRAM.RX_CP[channel];
}

/**
 * \brief   This function sets the number of free buffers of a specific
 *          channel
 *
 * \param   hCpdmaRegs    Pointer to CSL_CpdmaRegs structure
 * \param   channel       Channel Number.
 * \param   nBuf          Number of free buffers
 *
 * \return  None
 *
 **/
void CSL_CPSW_setCpdmaNumFreeBuf
(
    CSL_CpdmaRegs           *hCpdmaRegs,
    Uint32                   channel,
    Uint32                   nBuf
);

/**
 * \brief   Enable the command idle mode for CPDMA. When this API is called, the
 *          CPSW stops all the reception and transmission. However, if receiving
 *          the current frame will be received completely before going to the idle
 *          state. Also, while transmitting, the contents in the fifo will be sent
 *          fully.
 *
 * \param   hCpdmaRegs    Pointer to CSL_CpdmaRegs structure
 *
 * \return  None
 *
 **/
void CSL_CPSW_enableCpdmaIdle
(
    CSL_CpdmaRegs           *hCpdmaRegs
);

/**
 * \brief   Disable the command idle mode for CPDMA.
 *
 * \param   hCpdmaRegs    Pointer to CSL_CpdmaRegs structure
 *
 * \return  None
 *
 **/
void CSL_CPSW_disableCpdmaCmdIdle
(
    CSL_CpdmaRegs           *hCpdmaRegs
);

/**
 * \brief  This function indicates if the CPSW is at Software Idle mode where
 *         no packets will be started to be unloaded from ports.
 *
 * \param  hCpdmaRegs     Pointer to CSL_CpdmaRegs structure
 *
 * \return TRUE     if Software Idle mode enabled.
 *         FALSE    if Software Idle mode disabled.
 *
 **/
Uint32 CSL_CPSW_isCpdmaIdle
(
    CSL_CpdmaRegs           *hCpdmaRegs
);

/**
 * \brief   Sets the RX buffer offset value. The RX buffer offset will be
 *          written by the port into each frame SOP buffer descriptor
 *          buffer_offset field. The frame data will begin after the
 *          rx_buffer_offset value of bytes. This value will be used for
 *          all the channels .
 *
 * \param   hCpdmaRegs    Pointer to CSL_CpdmaRegs structure
 * \param   bufOff        Buffer offset value
 *
 * \return  None
 *
 **/
void CSL_CPSW_setCpdmaRxBufOffset
(
    CSL_CpdmaRegs           *hCpdmaRegs,
    Uint32                   bufOff
);

/**
 * \brief   Returns the raw transmit interrupt pending status.
 *
 * \param   hCpdmaRegs    Pointer to CSL_CpdmaRegs structure
 *
 * \return  Raw transmit interrupt status of all channels\n
 *          bits for the channel will be set if interrupt is pending \n
 *
 **/
Uint32 CSL_CPSW_getCpdmaTxIntStatRaw
(
    CSL_CpdmaRegs           *hCpdmaRegs
);

/**
 * \brief   Returns the masked transmit interrupt pending status.
 *
 * \param   hCpdmaRegs    Pointer to CSL_CpdmaRegs structure
 *
 * \return  transmit interrupt status for all  the masked channels \n
 *          bits for the channel will be set if interrupt is pending \n
 *
 **/
Uint32 CSL_CPSW_getCpdmaTxIntStatMasked
(
    CSL_CpdmaRegs           *hCpdmaRegs
);

/**
 * \brief   Returns the raw receive interrupt pending status.
 *
 * \param   hCpdmaRegs    Pointer to CSL_CpdmaRegs structure
 *
 * \return  Raw receive interrupt status of all channels\n
 *          bits for the channel will be set if interrupt is pending \n
 *
 **/
Uint32 CSL_CPSW_getCpdmaRxIntStatRaw
(
    CSL_CpdmaRegs           *hCpdmaRegs
);

/**
 * \brief   Returns the masked receive interrupt pending status.
 *
 * \param   hCpdmaRegs    Pointer to CSL_CpdmaRegs structure
 *
 * \return  receive interrupt status for all  the masked channels \n
 *          bits for the channel will be set if interrupt is pending \n
 *
 **/
Uint32 CSL_CPSW_getCpdmaRxIntStatMasked
(
    CSL_CpdmaRegs           *hCpdmaRegs
);

/**
 * \brief   Returns the raw receive thresh interrupt pending status.
 *
 * \param   hCpdmaRegs    Pointer to CSL_CpdmaRegs structure
 *
 * \return  Raw receive thresh interrupt status of all channels\n
 *          bits for the channel will be set if interrupt is pending \n
 **/
Uint32 CSL_CPSW_getCpdmaRxThreshIntStatRaw
(
    CSL_CpdmaRegs           *hCpdmaRegs
);

/**
 * \brief   Returns the masked receive thresh interrupt pending status.
 *
 * \param   hCpdmaRegs    Pointer to CSL_CpdmaRegs structure
 *
 * \return  Receive thresh interrupt status of all masked channels\n
 *          bits for the channel will be set if interrupt is pending \n
 *
 **/
Uint32 CSL_CPSW_getCpdmaRxThreshIntStatMasked
(
    CSL_CpdmaRegs           *hCpdmaRegs
);

/**
 * \brief   Returns the raw miscellaneous interrupt pending status.
 *
 * \param   hCpdmaRegs    Pointer to CSL_CpdmaRegs structure
 *
 * \return  Raw miscellaneous interrupt status \n
 *          bits for the events will be set if interrupt is pending \n
 *
 **/
Uint32 CSL_CPSW_getCpdmaDmaIntStatRaw
(
    CSL_CpdmaRegs           *hCpdmaRegs
);

/**
 * \brief   Returns the masked miscellaneous interrupt pending status.
 *
 * \param   hCpdmaRegs    Pointer to CSL_CpdmaRegs structure
 *
 * \return  miscellaneous interrupt status of thr masked events\n
 *          bits for the masked events will be set if interrupt is pending \n
 **/
Uint32 CSL_CPSW_getCpdmaDmaIntStatMasked
(
    CSL_CpdmaRegs           *hCpdmaRegs
);

/**
 * \brief   This function sets the thost_ts_encap bit in the CPDMA Control Register.
 *
 * \param   hCpdmaRegs    Pointer to CSL_CpdmaRegs structure
 *
 * \return  None
 *
 **/
void CSL_CPSW_enableCpdmaThostTsEncap
(
    CSL_CpdmaRegs           *hCpdmaRegs
);

/**
@}
*/

#ifdef __cplusplus
}
#endif

#endif /* CSL_CPDMA_H */

