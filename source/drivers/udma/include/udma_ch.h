/*
 *  Copyright (C) 2018-2024 Texas Instruments Incorporated
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
 *  \defgroup DRV_UDMA_CH_MODULE UDMA Channel API
 *            This is UDMA driver channel related configuration parameters and
 *            API
 *
 *  @{
 */

/**
 *  \file udma_ch.h
 *
 *  \brief UDMA Channel related parameters and API.
 */

#ifndef UDMA_CH_H_
#define UDMA_CH_H_

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
 * \brief Macro used to specify that DMA Channel ID is invalid.
 * Used in the API #Udma_chOpen.
 */
#define UDMA_DMA_CH_INVALID             ((uint32_t) 0xFFFF0000U)
/**
 * \brief Macro used to specify any available DMA Channel while requesting
 * one. Used in the API #Udma_chOpen.
 */
#define UDMA_DMA_CH_ANY                 ((uint32_t) 0xFFFF0001U)
/**
 * \brief Macro used to specify that the DMA Channel is not applicable for a
 * particular mode.
 */
#define UDMA_DMA_CH_NA                  ((uint32_t) 0xFFFF0002U)
/** \brief Macro used to specify that the Mapped Channel Group is invalid. */
#define UDMA_MAPPED_GROUP_INVALID       ((uint32_t) 0xFFFF0004U)

/** \brief DMSC Extended Channel Type Flag for BCDMA Block Copy */
#define UDMA_DMSC_EXTENDED_CH_TYPE_BCDMA_BLK_CPY     ((uint8_t) 1U)

/** \brief DMSC Extended Channel Type Flag for BCDMA split TR TX channels */
#define UDMA_DMSC_EXTENDED_CH_TYPE_BCDMA_SPLIT_TR_TX ((uint8_t) 0U)


/**
 *  \anchor Udma_ChFlag
 *  \name UDMA Channel Flag
 *
 *  UDMA channel flags bit field used to form the channel type.
 *
 *  @{
 */
/** \brief TX channel flag */
#define UDMA_CH_FLAG_TX                 ((uint32_t) 0x0001U)
/** \brief RX channel flag */
#define UDMA_CH_FLAG_RX                 ((uint32_t) 0x0002U)
/** \brief Block copy mode channel flag */
#define UDMA_CH_FLAG_BLK_COPY           ((uint32_t) 0x0004U)
/** \brief PDMA channel flag */
#define UDMA_CH_FLAG_PDMA               ((uint32_t) 0x0008U)
/** \brief PSIL channel flag meant for periperals like Ethernet, SA2UL */
#define UDMA_CH_FLAG_PSIL               ((uint32_t) 0x0010U)
/** \brief UTC channel flag */
#define UDMA_CH_FLAG_UTC                ((uint32_t) 0x0020U)
/** \brief High capacity channel flag */
#define UDMA_CH_FLAG_HC                 ((uint32_t) 0x0040U)
/** \brief Ultra high capacity channel flag */
#define UDMA_CH_FLAG_UHC                ((uint32_t) 0x0080U)
/** \brief Mapped TX/RX channel flag */
#define UDMA_CH_FLAG_MAPPED             ((uint32_t) 0x0100U)
/** @} */

/**
 *  \anchor Udma_ChType
 *  \name UDMA Channel Type
 *
 *  UDMA channel type formed based on channel flags.
 *
 *  @{
 */
/** \brief TR block copy type - TX/RX pair */
#define UDMA_CH_TYPE_TR_BLK_COPY        (UDMA_CH_FLAG_BLK_COPY |        \
                                         UDMA_CH_FLAG_TX |              \
                                         UDMA_CH_FLAG_RX)
/** \brief High capacity TR block copy type - TX/RX pair */
#define UDMA_CH_TYPE_TR_BLK_COPY_HC     (UDMA_CH_FLAG_BLK_COPY |        \
                                         UDMA_CH_FLAG_TX |              \
                                         UDMA_CH_FLAG_RX |              \
                                         UDMA_CH_FLAG_HC)
/** \brief Ultra high capacity TR block copy type - TX/RX pair */
#define UDMA_CH_TYPE_TR_BLK_COPY_UHC    (UDMA_CH_FLAG_BLK_COPY |        \
                                         UDMA_CH_FLAG_TX |              \
                                         UDMA_CH_FLAG_RX |              \
                                         UDMA_CH_FLAG_UHC)
/** \brief TX channel type */
#define UDMA_CH_TYPE_TX                 (UDMA_CH_FLAG_TX | UDMA_CH_FLAG_PSIL)
/** \brief High capacity TX channel type */
#define UDMA_CH_TYPE_TX_HC              (UDMA_CH_FLAG_TX | UDMA_CH_FLAG_PSIL | UDMA_CH_FLAG_HC)
/** \brief Ultra high capacity TX channel type */
#define UDMA_CH_TYPE_TX_UHC             (UDMA_CH_FLAG_TX | UDMA_CH_FLAG_PSIL | UDMA_CH_FLAG_UHC)

/** \brief RX channel type */
#define UDMA_CH_TYPE_RX                 (UDMA_CH_FLAG_RX | UDMA_CH_FLAG_PSIL)
/** \brief High capacity RX channel type */
#define UDMA_CH_TYPE_RX_HC              (UDMA_CH_FLAG_RX | UDMA_CH_FLAG_PSIL | UDMA_CH_FLAG_HC)
/** \brief Ultra high capacity RX channel type */
#define UDMA_CH_TYPE_RX_UHC             (UDMA_CH_FLAG_RX | UDMA_CH_FLAG_PSIL | UDMA_CH_FLAG_UHC)

/** \brief PDMA TX channel type */
#define UDMA_CH_TYPE_PDMA_TX            (UDMA_CH_FLAG_TX | UDMA_CH_FLAG_PDMA)
/** \brief High capacity PDMA TX channel type */
#define UDMA_CH_TYPE_PDMA_TX_HC         (UDMA_CH_FLAG_TX | UDMA_CH_FLAG_PDMA | UDMA_CH_FLAG_HC)
/** \brief Ultra high capacity PDMA TX channel type */
#define UDMA_CH_TYPE_PDMA_TX_UHC        (UDMA_CH_FLAG_TX | UDMA_CH_FLAG_PDMA | UDMA_CH_FLAG_UHC)

/** \brief PDMA RX channel type */
#define UDMA_CH_TYPE_PDMA_RX            (UDMA_CH_FLAG_RX | UDMA_CH_FLAG_PDMA)
/** \brief High capacity PDMA RX channel type */
#define UDMA_CH_TYPE_PDMA_RX_HC         (UDMA_CH_FLAG_RX | UDMA_CH_FLAG_PDMA | UDMA_CH_FLAG_HC)
/** \brief Ultra high capacity PDMA RX channel type */
#define UDMA_CH_TYPE_PDMA_RX_UHC        (UDMA_CH_FLAG_RX | UDMA_CH_FLAG_PDMA | UDMA_CH_FLAG_UHC)

/**
 *  \brief Mapped TX channel.
 *  This could be different type of mapped TX channels.
 *  See \ref Udma_MappedTxGrpSoc for differnt types of SOC specific mapped TX channels.
 */
#define UDMA_CH_TYPE_TX_MAPPED          (UDMA_CH_FLAG_TX | UDMA_CH_FLAG_PSIL | UDMA_CH_FLAG_MAPPED)
/**
 *  \brief Mapped RX channel.
 *  This could be different type of mapped RX channels.
 *  See \ref Udma_MappedRxGrpSoc for differnt types of SOC specific mapped RX channels.
 */
#define UDMA_CH_TYPE_RX_MAPPED          (UDMA_CH_FLAG_RX | UDMA_CH_FLAG_PSIL | UDMA_CH_FLAG_MAPPED)
/**
 *  \brief UTC channel. This could be
 *      - UTC with descriptor posted through UDMA external channel like VPAC/DMPAC
 *      - DRU channel with direct mode with descriptor posted through direct
 *        DRU register writes or with indirect mode through External channel
 */
#define UDMA_CH_TYPE_UTC                (UDMA_CH_FLAG_UTC)
/** @} */

/**
 *  \anchor Udma_PdmaElemSize
 *  \name UDMA PDMA element size
 *
 *  @{
 */
#define UDMA_PDMA_ES_8BITS              ((uint32_t) 0x00U)
#define UDMA_PDMA_ES_16BITS             ((uint32_t) 0x01U)
#define UDMA_PDMA_ES_24BITS             ((uint32_t) 0x02U)
#define UDMA_PDMA_ES_32BITS             ((uint32_t) 0x03U)
#define UDMA_PDMA_ES_64BITS             ((uint32_t) 0x04U)
/** \brief Set this for MCAN element size - not applicable */
#define UDMA_PDMA_ES_DONTCARE           ((uint32_t) 0x00U)
/** @} */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */



/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 *  \brief UDMA open channel.
 *
 *  Opens the UDMA channel based on the channel parameters. This also does
 *  the PSILCFG pairing based on the peer thread ID provided.
 *
 *  Requirement: DOX_REQ_TAG(PDK-2578)
 *
 *  \param drvHandle    [IN] UDMA driver handle pointer passed during
 *                           #Udma_init
 *  \param chHandle     [IN/OUT] UDMA channel handle. The caller need to
 *                           allocate memory for this object and pass this
 *                           pointer to all further APIs. The caller should
 *                           not change any parameters as this is owned and
 *                           maintained by the driver.
 *                           This parameter can't be NULL.
 *  \param chType       [IN] UDMA channel type. Refer \ref Udma_ChType.
 *  \param chPrms       [IN] UDMA channel parameters.
 *                           This parameter can't be NULL.
 *
 *  \return \ref Udma_ErrorCodes
 */
int32_t Udma_chOpen(Udma_DrvHandleInt drvHandle,
                    Udma_ChHandleInt chHandle,
                    uint32_t chType,
                    const Udma_ChPrms *chPrms);

/**
 *  \brief UDMA close channel.
 *
 *  Closes the UDMA channel and frees all associated resources.
 *
 *  Requirement: DOX_REQ_TAG(PDK-2579)
 *
 *  \param chHandle     [IN] UDMA channel handle.
 *                           This parameter can't be NULL.
 *
 *  \return \ref Udma_ErrorCodes
 */
int32_t Udma_chClose(Udma_ChHandleInt chHandle);

/**
 *  \brief UDMA configure TX channel.
 *
 *  Configures the TX channel parameters. Note: This is applicable only
 *  when the channel type is TX
 *
 *  Note: This API can't be called after channel enable.
 *
 *  Requirement: DOX_REQ_TAG(PDK-2580)
 *
 *  \param chHandle     [IN] UDMA channel handle.
 *                           This parameter can't be NULL.
 *  \param txPrms       [IN] UDMA TX channel parameter.
 *                           Refer #Udma_ChTxPrms.
 *
 *  \return \ref Udma_ErrorCodes
 */
int32_t Udma_chConfigTx(Udma_ChHandleInt chHandle, const Udma_ChTxPrms *txPrms);

/**
 *  \brief UDMA configure RX channel.
 *
 *  Configures the RX channel parameters. Note: This is applicable only
 *  when the channel type is RX
 *  In case of BCDMA Block Copy, there is no need to configure RX Channel.
 *  Therfore the function returns gracefully, without doing anything.
 *
 *  Note: This API can't be called after channel enable.
 *
 *  Requirement: DOX_REQ_TAG(PDK-2581)
 *
 *  \param chHandle     [IN] UDMA channel handle.
 *                           This parameter can't be NULL.
 *  \param rxPrms       [IN] UDMA RX channel parameter.
 *                           Refer #Udma_ChRxPrms.
 *
 *  \return \ref Udma_ErrorCodes
 */
int32_t Udma_chConfigRx(Udma_ChHandleInt chHandle, const Udma_ChRxPrms *rxPrms);

/**
 *  \brief UDMA configure PDMA channel (peerChNum as part of #Udma_ChPrms)
 *  paired with the UDMAP channel.
 *
 *  This configures the PDMA channel static X,Y,Z parameters.
 *
 *  Requirement: DOX_REQ_TAG(PDK-2583)
 *
 *  \param chHandle     [IN] UDMA channel handle.
 *                           This parameter can't be NULL.
 *  \param pdmaPrms     [IN] UDMA RX channel parameter.
 *                           Refer #Udma_ChPdmaPrms.
 *
 *  \return \ref Udma_ErrorCodes
 */
int32_t Udma_chConfigPdma(Udma_ChHandleInt chHandle,
                          const Udma_ChPdmaPrms *pdmaPrms);

/**
 *  \brief UDMA channel enable API.
 *
 *  This function will enable the UDMA channel.
 *
 *  Requirement: DOX_REQ_TAG(PDK-2584)
 *
 *  \param chHandle     [IN] UDMA channel handle.
 *                           This parameter can't be NULL.
 *
 *  \return \ref Udma_ErrorCodes
 */
int32_t Udma_chEnable(Udma_ChHandleInt chHandle);

/**
 *  \brief UDMA channel teardown and disable API.
 *
 *  This function will perform the channel teardown and eventually disables
 *  the UDMA channel.
 *  This initiates the force teardown sequence based on the channel type and
 *  wait for teardown to complete gracefully.
 *  If the teardown doesn't complete within the timeout provided, then this
 *  will initiate a force teardown sequence.
 *
 *  Caution: This API is blocking. Hence cannot be called from ISR context!!
 *
 *  Requirement: DOX_REQ_TAG(PDK-2585)
 *
 *  \param chHandle     [IN] UDMA channel handle.
 *                           This parameter can't be NULL.
 *  \param timeout      [IN] Timeout in ms.
 *                           Use #SystemP_WAIT_FOREVER to wait forever.
 *
 *  \return \ref Udma_ErrorCodes
 */
int32_t Udma_chDisable(Udma_ChHandleInt chHandle, uint32_t timeout);

/**
 *  \brief UDMA channel pause API.
 *
 *  This function will pause the UDMA channel by setting the pause bit of the
 *  UDMAP runtime register.
 *
 *  Requirement: DOX_REQ_TAG(PDK-2977)
 *
 *  \param chHandle     [IN] UDMA channel handle.
 *                           This parameter can't be NULL.
 *
 *  \return \ref Udma_ErrorCodes
 */
int32_t Udma_chPause(Udma_ChHandleInt chHandle);

/**
 *  \brief UDMA channel resume API.
 *
 *  This function will resume the UDMA channel by clearing the pause bit of the
 *  UDMAP runtime register.
 *
 *  Requirement: DOX_REQ_TAG(PDK-2977)
 *
 *  \param chHandle     [IN] UDMA channel handle.
 *                           This parameter can't be NULL.
 *
 *  \return \ref Udma_ErrorCodes
 */
int32_t Udma_chResume(Udma_ChHandleInt chHandle);

/**
 *  \brief Returns the channel number offset with in a channel type - TX, RX
 *  and External (UTC) channel types.
 *
 *  In case of UTC type, this returns the relative offset from the start
 *  of UTC it belongs to (and not from the external channel start).
 *
 *  \param chHandle     [IN] UDMA channel handle.
 *                           This parameter can't be NULL.
 *
 *  \return Channel number. Returns #UDMA_DMA_CH_INVALID for error.
 */
uint32_t Udma_chGetNum(Udma_ChHandleInt chHandle);

/**
 *  \brief Returns the default free ring handle of the channel.
 *
 *  \param chHandle     [IN] UDMA channel handle.
 *                           This parameter can't be NULL.
 *
 *  \return Free ring handle. Returns NULL for error.
 */
Udma_RingHandle Udma_chGetFqRingHandle(Udma_ChHandleInt chHandle);

/**
 *  \brief Returns the default completion ring handle of the channel.
 *
 *  \param chHandle     [IN] UDMA channel handle.
 *                           This parameter can't be NULL.
 *
 *  \return Completion ring handle. Returns NULL for error.
 */
Udma_RingHandle Udma_chGetCqRingHandle(Udma_ChHandleInt chHandle);

/**
 *  \brief Returns the teardown completion ring handle of the channel.
 *
 *  \param chHandle     [IN] UDMA channel handle.
 *                           This parameter can't be NULL.
 *
 *  \return Teardown completion ring handle. Returns NULL for error.
 */
Udma_RingHandle Udma_chGetTdCqRingHandle(Udma_ChHandleInt chHandle);

/**
 *  \brief Returns the default free ring number to be programmed
 *  in descriptor.
 *
 *  \param chHandle     [IN] UDMA channel handle.
 *                           This parameter can't be NULL.
 *
 *  \return Free ring number. Returns #UDMA_RING_INVALID for error.
 */
uint16_t Udma_chGetFqRingNum(Udma_ChHandleInt chHandle);

/**
 *  \brief Returns the default completion ring number to be programmed in
 *  descriptor.
 *
 *  Requirement: DOX_REQ_TAG(PDK-2586)
 *
 *  \param chHandle     [IN] UDMA channel handle.
 *                           This parameter can't be NULL.
 *
 *  \return Completion ring number. Returns #UDMA_RING_INVALID for error.
 */
uint16_t Udma_chGetCqRingNum(Udma_ChHandleInt chHandle);

/**
 *  \brief Returns the default flow handle of the RX channel.
 *
 *  \param chHandle     [IN] UDMA channel handle.
 *                           This parameter can't be NULL.
 *
 *  \return Default flow handle. Returns NULL for error.
 */
Udma_FlowHandle Udma_chGetDefaultFlowHandle(Udma_ChHandleInt chHandle);

/**
 *  \brief Returns the global trigger event for the channel
 *
 *  This function will return the appropriate global 0/1 trigger event for the channel.
 *
 *  Notes: Trigger is not supported for external channels
 *         and the function will return #UDMA_EVENT_INVALID.
 *
 *  \param chHandle     [IN] UDMA channel handle.
 *                           This parameter can't be NULL.
 *
 *  \param trigger      [IN] Global0 or Global 1 Trigger - refer
 *                          \ref CSL_UdmapTrFlagsTrigger
 *
 *  \return Global trigger event
 */
uint32_t Udma_chGetTriggerEvent(Udma_ChHandleInt chHandle, uint32_t trigger);

/**
 *  \brief Returns the software trigger register address for the channel
 *
 *  This function will return the appropriate SW trigger register.
 *  Incase of UDMAP channels, it returns the 32-bit TX SWTRIG register address.
 *  Incase of DRU channels, it returns the 64-bit DRU CHRT_SWTRIG register address.
 *
 *  Notes: SW trigger is not supported for RX channels.
 *         Incase of TX channels, only global trigger 0 is supported.
 *         Incase of DRU channels, global trigger 0/1 and local events are
 *         supported.
 *
 *  Requirement: DOX_REQ_TAG(PDK-2594)
 *
 *  \param chHandle     [IN] UDMA channel handle.
 *                           This parameter can't be NULL.
 *
 *  \return SW trigger register address
 */
uint32_t *Udma_chGetSwTriggerRegister(Udma_ChHandleInt chHandle);

/**
 *  \brief Sets the software trigger register based on the trigger mode
 *  provided.
 *
 *  This function will set the appropriate SW trigger register.
 *  Incase of UDMAP channels, it will set in the TX SWTRIG register.
 *  Incase of DRU channels, it will set in the DRU CHRT_SWTRIG register.
 *
 *  Notes: SW trigger is not supported for RX channels.
 *         Incase of TX channels, only global trigger 0 is supported.
 *         Incase of DRU channels, global trigger 0/1 and local events are
 *         supported.
 *
 *  Requirement: DOX_REQ_TAG(PDK-2594)
 *
 *  \param chHandle     [IN] UDMA channel handle.
 *                           This parameter can't be NULL.
 *  \param trigger      [IN] Global0 or Global 1 Trigger - refer
 *                          \ref CSL_UdmapTrFlagsTrigger
 *
 *  \return \ref Udma_ErrorCodes
 */
int32_t Udma_chSetSwTrigger(Udma_ChHandleInt chHandle, uint32_t trigger);

/**
 *  \brief Chains the trigger channel with the chained channel.
 *
 *  This programs the trigger channel TR event register (OES) to the global
 *  trigger (0 or 1) event of the chained channel.
 *
 *  Once this is done, the application should set the TR trigger (0 or 1)
 *  of the trigger channel while submitting TR to the trigger channel.
 *  Based on the trigger type (full, ICNT0, INCT1, ICNT3), the trigger
 *  channel will trigger the "chained" channel through the channel OES.
 *
 *  Note: Only global0 and global1 triggers are supported.
 *
 *  \param triggerChHandle  [IN] UDMA channel handle which triggers the chain.
 *                          This parameter can't be NULL.
 *  \param chainedChHandle  [IN] UDMA channel handle which gets triggered.
 *                          This parameter can't be NULL.
 *  \param trigger          [IN] Global0 or Global 1 Trigger - refer
 *                          \ref CSL_UdmapTrFlagsTrigger
 *
 *  \return \ref Udma_ErrorCodes
 */
int32_t Udma_chSetChaining(Udma_ChHandleInt triggerChHandle,
                           Udma_ChHandleInt chainedChHandle,
                           uint32_t trigger);

/**
 *  \brief Breaks the chaining by resetting the trigger channel's OES.
 *
 *  Note: Only global0 and global1 triggers are supported.
 *
 *  \param triggerChHandle  [IN] UDMA channel handle which triggers the chain.
 *                          This parameter can't be NULL.
 *  \param chainedChHandle  [IN] UDMA channel handle which gets triggered.
 *                          This parameter can't be NULL.
 *
 *  \return \ref Udma_ErrorCodes
 */
int32_t Udma_chBreakChaining(Udma_ChHandleInt triggerChHandle,
                             Udma_ChHandleInt chainedChHandle);

/*
 * Structure Init functions
 */
/**
 *  \brief Udma_ChPrms structure init function.
 *
 *  \param chPrms       [IN] Pointer to #Udma_ChPrms structure.
 *  \param chType       [IN] UDMA channel type. Refer \ref Udma_ChType.
 *
 */
void UdmaChPrms_init(Udma_ChPrms *chPrms, uint32_t chType);

/**
 *  \brief Udma_ChTxPrms structure init function.
 *
 *  \param txPrms       [IN] Pointer to #Udma_ChTxPrms structure.
 *  \param chType       [IN] UDMA channel type. Refer \ref Udma_ChType.
 *
 */
void UdmaChTxPrms_init(Udma_ChTxPrms *txPrms, uint32_t chType);

/**
 *  \brief Udma_ChRxPrms structure init function.
 *
 *  \param rxPrms       [IN] Pointer to #Udma_ChRxPrms structure.
 *  \param chType       [IN] UDMA channel type. Refer \ref Udma_ChType.
 *
 */
void UdmaChRxPrms_init(Udma_ChRxPrms *rxPrms, uint32_t chType);

/**
 *  \brief Udma_ChPdmaPrms structure init function.
 *
 *  \param pdmaPrms     [IN] Pointer to #Udma_ChPdmaPrms structure.
 *
 */
void UdmaChPdmaPrms_init(Udma_ChPdmaPrms *pdmaPrms);

/**
 *  \brief Get real-time channel statistics.
 *
 *  Requirement: PRSDK-5609
 *
 *  \param chHandle     [IN] UDMA channel handle.
 *                           This parameter can't be NULL.
 *  \param chStats      [IN] Pointer to #Udma_ChStats.
 *                           This parameter can't be NULL.
 *
 *  \return \ref Udma_ErrorCodes
 */
int32_t Udma_chGetStats(Udma_ChHandleInt chHandle, Udma_ChStats *chStats);

/**
 *  \brief Get real-time peer data which contains number of bytes written.
 *
 *  \param chHandle     [IN]    UDMA channel handle.
 *                              This parameter can't be NULL.
 *  \param peerData     [INOUT] Pointer to peer data.
 *
 *  \return \ref Udma_ErrorCodes
 */
int32_t Udma_getPeerData(Udma_ChHandleInt chHandle, uint32_t *peerData);

/**
 *  \brief Clear real-time peer data which contains number of bytes written.
 *
 *  \param chHandle     [IN]    UDMA channel handle.
 *                              This parameter can't be NULL.
 *  \param peerData     [IN] Peer data.
 *
 *  \return \ref Udma_ErrorCodes
 */
int32_t Udma_clearPeerData(Udma_ChHandleInt chHandle, uint32_t peerData);

#if (UDMA_SOC_CFG_RA_NORMAL_PRESENT == 1)
/**
 *  \brief Returns the global trigger event for the channel
 *
 *  This function will return the appropriate global 0/1 trigger event for the channel.
 *
 *  Notes: Trigger is not supported for external channels
 *         and the function will return #UDMA_EVENT_INVALID.
 *
 *  \param chHandle     [IN] UDMA channel handle.
 *                           This parameter can't be NULL.
 *
 *  \param trigger      [IN] Global0 or Global 1 Trigger - refer
 *                          \ref CSL_UdmapTrFlagsTrigger
 *
 *  \return Global trigger event
 */
int32_t Udma_chDequeueTdResponse(Udma_ChHandleInt chHandle,
                                 CSL_UdmapTdResponse *tdResponse);
#endif
/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                  Internal/Private Structure Declarations                   */
/* ========================================================================== */

/**
 *  \brief Opaque UDMA channel object.
 */
typedef struct Udma_ChObject_t
{
    #if defined (SOC_AM65X)
        uintptr_t rsv[200U];
    #else
        uintptr_t rsv[150U];
    #endif
    /**< reserved, should NOT be modified by end users */
} Udma_ChObject;

#ifdef __cplusplus
}
#endif

#endif /* #ifndef UDMA_CH_H_ */

/** @} */
