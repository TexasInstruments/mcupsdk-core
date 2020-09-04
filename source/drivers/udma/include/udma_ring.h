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
 *  \defgroup DRV_UDMA_RING_MODULE UDMA Ring API
 *            This is UDMA driver ring related configuration parameters and
 *            API
 *
 *  @{
 */

/**
 *  \file udma_ring.h
 *
 *  \brief UDMA ring related parameters and API.
 */

#ifndef UDMA_RING_H_
#define UDMA_RING_H_

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
 * \brief Macro used to specify that ring ID is invalid.
 * Used in the API #Udma_ringGetNum.
 */
#define UDMA_RING_INVALID               ((uint16_t) TISCI_MSG_VALUE_RM_NULL_RING_TYPE)
/**
 * \brief Macro used to specify any available free ring while requesting
 * one. Used in the API #Udma_ringAlloc.
 */
#define UDMA_RING_ANY                   ((uint16_t) 0xFFFEU)
/**
 * \brief Macro used to specify that ring virt ID is invalid.
 * Used in the API #Udma_ringAlloc.
 */
#define UDMA_RING_VIRTID_INVALID        ((uint16_t) 0xFFFFU)

/** \brief Macro used to skip the ring size check by driver */
#define UDMA_RING_SIZE_CHECK_SKIP       (0xABDCABCDU)

/**
 * \brief Macro used to specificy the maximum ring order id value
 */
#define UDMA_RING_ORDERID_MAX           (0x0FU)

/**
 *  \anchor Udma_RingElemSize
 *  \name UDMA Ring element size
 *
 *  Encoded ring element size to be programmed into the elsize field of the
 *  ring's RING_SIZE register.  To calculate the encoded size use the
 *  formula (log2(size_bytes) - 2), where "size_bytes" cannot be greater than
 *  256 bytes. This calculation is already taken care in below macro.
 *
 *  @{
 */
/** \brief 4 bytes Element size */
#define UDMA_RING_ES_4BYTES             ((uint8_t) 0x00U)
/** \brief 8 bytes Element size */
#define UDMA_RING_ES_8BYTES             ((uint8_t) 0x01U)
/** \brief 16 bytes Element size */
#define UDMA_RING_ES_16BYTES            ((uint8_t) 0x02U)
/** \brief 32 bytes Element size */
#define UDMA_RING_ES_32BYTES            ((uint8_t) 0x03U)
/** \brief 64 bytes Element size */
#define UDMA_RING_ES_64BYTES            ((uint8_t) 0x04U)
/** \brief 128 bytes Element size */
#define UDMA_RING_ES_128BYTES           ((uint8_t) 0x05U)
/** \brief 256 bytes Element size */
#define UDMA_RING_ES_256BYTES           ((uint8_t) 0x06U)
/** @} */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 *  \brief UDMA ring parameters.
 */
typedef struct
{
    void                   *ringMem;
    /**< Pointer to ring memory.
     *   Incase of FQ and CQ rings, this cannot be NULL except for DRU
     *   direct TR mode where the rings are not used.
     *   Incase of TD CQ, this can be NULL when TD response is supressed via
     *   supressTdCqPkt channel parameter.
     *   Note: This is a virtual pointer. */
    uint32_t                ringMemSize;
    /**< Size of the memory in bytes allocated. This is used by the driver
     *   to validate the allocated memory is sufficient or not.
     *
     *   Note: By default this parameter will be set to
     *   #UDMA_RING_SIZE_CHECK_SKIP by #UdmaRingPrms_init API to enable
     *   backward combatibility when this is not set rightly by the caller */
    uint8_t                 mode;
    /**< Ring mode. Refer \ref tisci_msg_rm_ring_cfg_req::mode */
    uint16_t                virtId;
    /**< Ring virt ID. Refer \ref tisci_msg_rm_ring_cfg_req::virtid */
    uint32_t                elemCnt;
    /**< Ring element count.
     *      Set to queue depth of the ring.
     *      Set to 0 for DRU direct TR mode. */
    uint8_t                 elemSize;
    /**< Ring element size.
     *   Refer \ref Udma_RingElemSize for supported values. */
    uint8_t                 orderId;
    /**< Ring bus order ID value to be programmed into the orderid field of
     *   the ring's RING_ORDERID register. */
    uint8_t                 asel;
    /**< Ring ASEL (address select) value to be set into the ASEL field of the ring's
    *    RING_BA_HI register.
    *    Refer \ref Udma_RingAccAselEndpointSoc for supported values.
    *    This field is not supported on some SoCs.
    *    On SoCs that do not support this field the input is quietly ignored.
    *    Note: By default this parameter will be set to
    *    #UDMA_RINGACC_ASEL_ENDPOINT_PHYSADDR by #UdmaRingPrms_init API */
    uint32_t                mappedRingGrp;
    /**< The Mapped ring group to use when channel type is
     *   #UDMA_CH_TYPE_TX_MAPPED or #UDMA_CH_TYPE_RX_MAPPED.
     *
     *   Refer \ref Udma_MappedTxGrpSoc macro for details about mapped TX ring groups
     *   or \ref Udma_MappedRxGrpSoc macro for details about mapped RX ring groups.
     *
     *   For unmapped case, set to #UDMA_MAPPED_GROUP_INVALID
     */
    uint32_t                mappedChNum;
    /**< The assigned mapped channel number when channel type is
     *   #UDMA_CH_TYPE_TX_MAPPED or #UDMA_CH_TYPE_RX_MAPPED.
     *
     *   This is used to allocate the corresponding mapped ring for the particular channel.
     *   RM will derive an intersecting pool based on the rings reserved for the core (in rmcfg)
     *   and the permissible range for the given channel(rings reserved for specific channels)
     *   such that the allocated ring will be from this intersecting pool.
     *
     *   For example, If the rings idx reserved for the core are 10 to 20 and
     *   the rings for the channel are 15 to 25. Then the intersecting pool of ring idx
     *   will be 15 - 20 and rm will allocate from this range.
     */
} Udma_RingPrms;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 *  \brief UDMA ring allocation and configuration API.
 *
 *  \param drvHandle    [IN] UDMA driver handle pointer passed during
 *                           #Udma_init
 *  \param ringHandle   [IN/OUT] UDMA ring handle. The caller need to
 *                           allocate memory for this object and pass this
 *                           pointer to all further APIs. The caller should
 *                           not change any parameters as this is owned and
 *                           maintained by the driver.
 *  \param ringNum      [IN] Ring number. If set to #UDMA_RING_ANY, will
 *                           allocate from free ring pool. Else will try to
 *                           allocate the mentioned ring itself.
 *  \param ringPrms     [IN] UDMA ring parameters.
 *                           This parameter can't be NULL.
 *
 *  \return \ref Udma_ErrorCodes
 */
int32_t Udma_ringAlloc(Udma_DrvHandle drvHandle,
                       Udma_RingHandle ringHandle,
                       uint16_t ringNum,
                       const Udma_RingPrms *ringPrms);

/**
 *  \brief UDMA free ring.
 *
 *  Freeup the ring resources.
 *
 *  \param ringHandle   [IN] UDMA ring handle.
 *                           This parameter can't be NULL.
 *
 *  \return \ref Udma_ErrorCodes
 */
int32_t Udma_ringFree(Udma_RingHandle ringHandle);

/**
 *  \brief UDMA ring attach API. This API is used to attach to an already
 *  allocated and configured ring. This API differs from ring alloc API in
 *  this aspect - it doesn't allocate resource from RM and doesn't configure
 *  the ring through sciclient/DMSC API.
 *
 *  Post this attach operation, other standard ring operations can be performed.
 *  This API is provided for usecases where a ring is configured by a remote
 *  entity and needs to be used for runtime operation from another entity.
 *
 *  Requirement: DOX_REQ_TAG(PDK-3419)
 *
 *  \param drvHandle    [IN] UDMA driver handle pointer passed during
 *                           #Udma_init
 *  \param ringHandle   [IN/OUT] UDMA ring handle. The caller need to
 *                           allocate memory for this object and pass this
 *                           pointer to all further APIs. The caller should
 *                           not change any parameters as this is owned and
 *                           maintained by the driver.
 *  \param ringNum      [IN] Ring number to attach with. This paramter should
 *                           be a valid ring number allowed to be used by a
 *                           core. The driver doesn't check the validity of
 *                           this field at the time of attach. But the runtime
 *                           ring API may fail if wrong ring index is used or
 *                           when the core does ring operation when it doesn't
 *                           own the ring based on credential and DMSC board
 *                           config.
 *
 *  \return \ref Udma_ErrorCodes
 */
int32_t Udma_ringAttach(Udma_DrvHandle drvHandle,
                        Udma_RingHandle ringHandle,
                        uint16_t ringNum);

/**
 *  \brief UDMA detach ring API.
 *
 *  Since no allocation is done in attach, this API just clears up the
 *  ring handle.
 *
 *  Requirement: DOX_REQ_TAG(PDK-3419)
 *
 *  \param ringHandle   [IN] UDMA ring handle.
 *                           This parameter can't be NULL.
 *
 *  \return \ref Udma_ErrorCodes
 */
int32_t Udma_ringDetach(Udma_RingHandle ringHandle);

/**
 *  \brief UDMA queue descriptor to a ring - raw version
 *  (Takes all physical pointers)
 *
 *  This function will push the descriptor to the ring as identified by
 *  the ring handle.
 *
 *  Incase of exposed/"RING" mode, this will use the ring door bell mechanism.
 *  For other modes, this will push the descriptor to the ring through the
 *  proxy allocated to the driver handle.
 *
 *  Writing through a proxy is required for ring push operation when the
 *  ring is not in "RING" mode and when the host/core cannot perform a
 *  64-bit atomic write operation.
 *
 *  This API is thread safe for a ring instance and can be called from
 *  interrupt or task context and also from multiple threads.
 *
 *  Requirement: DOX_REQ_TAG(PDK-2587)
 *  Requirement: DOX_REQ_TAG(PDK-2633)
 *
 *  \param ringHandle   [IN] UDMA ring handle.
 *                           This parameter can't be NULL.
 *  \param phyDescMem   [IN] Descriptor memory physical pointer to push to the
 *                           ring.
 *
 *  \return \ref Udma_ErrorCodes
 */
int32_t Udma_ringQueueRaw(Udma_RingHandle ringHandle, uint64_t phyDescMem);

/**
 *  \brief UDMA dequeue descriptor from a ring - raw version
 *  (Takes all physical pointers).
 *
 *  This function will pop the descriptor from the ring as identified by
 *  the ring handle.
 *
 *  Incase of exposed/"RING" mode, this will use the ring door bell mechanism.
 *  For other modes, this will pop the descriptor from the ring through the
 *  proxy allocated to the driver handle.
 *
 *  Reading through a proxy is required for ring pop operation when the
 *  ring is not in "RING" mode and when the host/core cannot perform a
 *  64-bit atomic read operation.
 *
 *  This API is thread safe for a ring instance and can be called from
 *  interrupt or task context and also from multiple threads.
 *
 *  This is non-blocking and will return timeout error #UDMA_ETIMEOUT
 *  when the queue is empty.
 *
 *  Caution: Dequeuing from a ring (free queue) to which the UDMA reads
 *  should be performed only when the channel is disabled and using
 *  #Udma_ringFlushRaw API.
 *
 *  Requirement: DOX_REQ_TAG(PDK-2588)
 *  Requirement: DOX_REQ_TAG(PDK-2633)
 *
 *  \param ringHandle   [IN] UDMA ring handle.
 *                           This parameter can't be NULL.
 *  \param phyDescMem   [OUT] Descriptor memory physical pointer read from the
 *                          ring. This will be NULL if there is
 *                          nothing to pop from the ring.
 *
 *  \return \ref Udma_ErrorCodes
 */
int32_t Udma_ringDequeueRaw(Udma_RingHandle ringHandle, uint64_t *phyDescMem);

/**
 *  \brief UDMA dequeue descriptor from a ring when UDMA channel is disabled -
 *  raw version (Takes all physical pointers).
 *
 *  This function will pop the unprocessed descriptor from the the ring (say
 *  the free ring which is used by UDMA channel).
 *
 *  This is non-blocking and will return timeout error #UDMA_ETIMEOUT
 *  when the queue is empty.
 *
 *  Caution: Dequeuing from a ring (free queue) to which the UDMA reads
 *  should be performed only when the channel is disabled.
 *
 *  Requirement: DOX_REQ_TAG(PDK-3238)
 *
 *  \param ringHandle   [IN] UDMA ring handle.
 *                           This parameter can't be NULL.
 *  \param phyDescMem   [OUT] Descriptor memory physical pointer read from the
 *                          ring. This will be NULL if there is
 *                          nothing to pop from the ring.
 *
 *  \return \ref Udma_ErrorCodes
 */
int32_t Udma_ringFlushRaw(Udma_RingHandle ringHandle, uint64_t *phyDescMem);

/**
 *  \brief UDMA prime descriptor to a exposed/"RING" mode ring - raw version
 *  (Takes all physical pointers). This will write the descriptor to the
 *  ring memory without setting the doorbell (doesn't commit the push).
 *
 *  This API can be used to prime multiple request to the free queue ring and then set the
 *  doorbell using #Udma_ringSetDoorBell API.
 *
 *  Also no cache operation is performed to let the caller do the cache ops
 *  once for the entire ring after priming multiple elements. This will yeild
 *  better performance instead of doing cache ops for each ring push.
 *
 *  Note: No error check is performed by this API to minimize the CPU cycles.
 *  The caller should ensure that the ring is in exposed/"RING" mode and
 *  there are enough room in the ring and the ring pointer is non-null.
 *
 *  This API is thread safe for a ring instance and can be called from
 *  interrupt or task context and also from multiple threads.
 *
 *  Requirement: DOX_REQ_TAG(PDK-3669)
 *
 *  \param ringHandle   [IN] UDMA ring handle.
 *                           This parameter can't be NULL.
 *  \param phyDescMem   [IN] Descriptor memory physical pointer to push to the
 *                           ring.
 */
void Udma_ringPrime(Udma_RingHandle ringHandle, uint64_t phyDescMem);

/**
 *  \brief UDMA read descriptor from a exposed/"RING" mode ring - raw version
 *  (Reads physical pointers). This will read the descriptor address from the
 *  ring memory without setting the doorbell (doesn't commit the pop).
 *
 *  This API can be used to read multiple descriptor addresses from the completion queue
 *  ring and then set the doorbell using #Udma_ringSetDoorBell API.
 *
 *  Also no cache operation is performed to let the caller do the cache ops
 *  once for the entire ring after reading multiple elements. This will yeild
 *  better performance instead of doing cache ops for each ring pop.
 *
 *  Note: No error check is performed by this API to minimize the CPU cycles.
 *  The caller should ensure that the ring is in exposed/"RING" mode and
 *  descriptor addresses are in the ring and the ring pointer is non-null.
 *  Also make sure that its not reading more than the what #Udma_ringGetReverseRingOcc
 *  returns.
 *
 *  This API is thread safe for a ring instance and can be called from
 *  interrupt or task context and also from multiple threads.
 *
 *  Requirement: DOX_REQ_TAG(PDK-3669)
 *
 *  \param ringHandle   [IN] UDMA ring handle.
 *                           This parameter can't be NULL.
 *  \param phyDescMem   [IN] Descriptor memory physical pointer to pop from the
 *                           ring.
 */
void Udma_ringPrimeRead(Udma_RingHandle ringHandle, uint64_t *phyDescMem);

/**
 *  \brief UDMA ring API to set the doorbell in exposed/"RING" mode ring.
 *  This will commit the previously primed operation using #Udma_ringPrime API.
 *
 *  Note: The count will be positive when ring elements are queued into the ring
 *  and count will be negative when ring elements are dequeued from the ring.
 *  In case of devices like AM64x with LCDMA ring accelerator,
 *  when the count is positive, it sets the forward doorbell of the common ring
 *  and when the count is negative, it sets the reverse doorbell of the common ring.
 *  For other devices with normal ring accelerator,
 *  these sets the doorbell of the ring. Here its meaningful to
 *  pass the ringHandle of Free Queue Ring when the count is positive
 *  and pass the ringHandle of Completion Queue Ring when the count is negative.
 *
 *  Note: No error check is performed by this API to minimize the CPU cycles.
 *  The caller should ensure that the ring is in exposed/"RING" mode and
 *  there are enough room in te ring and the ring pointer is non-null.
 *
 *  This API is thread safe for a ring instance and can be called from
 *  interrupt or task context and also from multiple threads.
 *
 *  Requirement: DOX_REQ_TAG(PDK-3669)
 *
 *  \param ringHandle   [IN] UDMA ring handle.
 *                           This parameter can't be NULL.
 *  \param count        [IN] Number of count to commit.
 */
void Udma_ringSetDoorBell(Udma_RingHandle ringHandle, int32_t count);

/**
 *  \brief Returns the ring number allocated for this ring.
 *
 *  \param ringHandle   [IN] UDMA ring handle.
 *                           This parameter can't be NULL.
 *
 *  \return The ring number on success or #UDMA_RING_INVALID on error
 */
uint16_t Udma_ringGetNum(Udma_RingHandle ringHandle);

/**
 *  \brief Returns the ring memory pointer which is passed during ring alloc.
 *
 *  Requirement: DOX_REQ_TAG(PDK-3668)
 *
 *  \param ringHandle   [IN] UDMA ring handle.
 *                           This parameter can't be NULL.
 *
 *  \return Ring memory pointer on success or NULL on error
 */
void *Udma_ringGetMemPtr(Udma_RingHandle ringHandle);

/**
 *  \brief Returns the ring mode which is configured during ring alloc.
 *
 *  Requirement: DOX_REQ_TAG(PDK-5665)
 *
 *  \param ringHandle   [IN] UDMA ring handle.
 *                           This parameter can't be NULL.
 *
 *  \return Ring mode on success or CSL_RINGACC_RING_MODE_INVALID on error
 */
uint32_t Udma_ringGetMode(Udma_RingHandle ringHandle);

/**
 *  \brief Returns the ring element count which is passed during ring alloc.
 *
 *  Requirement: DOX_REQ_TAG(PDK-5665)
 *
 *  \param ringHandle   [IN] UDMA ring handle.
 *                           This parameter can't be NULL.
 *
 *  \return Ring element count on success or zero on error
 */
uint32_t Udma_ringGetElementCnt(Udma_RingHandle ringHandle);

/**
 *  \brief Returns the forward ring occupancy.
 *
 *  Note: In case of devices like AM64x with LCDMA ring accelerator,
 *  this returns the forward ring occupancy count of the common ring.
 *  For other devices with normal ring accelerator,
 *  this returns the the occupancy count of the ring. Here its meaningful
 *  to pass the ringHandle of Free Queue Ring.
 *
 *  Requirement: DOX_REQ_TAG(PDK-5665)
 *
 *  \param ringHandle   [IN] UDMA ring handle.
 *                           This parameter can't be NULL.
 *
 *  \return Ring occupancy value from the register
 */
uint32_t Udma_ringGetForwardRingOcc(Udma_RingHandle ringHandle);

/**
 *  \brief Returns the reverse ring occupancy.
 *
 *  Note: In case of devices like AM64x with LCDMA ring accelerator,
 *  this returns the reverse ring occupancy count of the common ring.
 *  For other devices with normal ring accelerator,
 *  this returns the the occupancy count of the ring. Here its meaningful
 *  to pass the ringHandle of Completion Queue Ring.
 *
 *  Requirement: DOX_REQ_TAG(PDK-5665)
 *
 *  \param ringHandle   [IN] UDMA ring handle.
 *                           This parameter can't be NULL.
 *
 *  \return Ring occupancy value from the register
 */
uint32_t Udma_ringGetReverseRingOcc(Udma_RingHandle ringHandle);

/**
 *  \brief Returns the ring write index value.
 *
 *  Note: In case of devices like AM64x with LCDMA ring accelerator,
 *  this returns the write index value of the common ring.
 *  For other devices with normal ring accelerator,
 *  this returns the the read/write index value of the ring. Here its meaningful
 *  to pass the ringHandle of Free Queue Ring.
 *
 *  Requirement: DOX_REQ_TAG(PDK-5665)
 *
 *  \param ringHandle   [IN] UDMA ring handle.
 *                           This parameter can't be NULL.
 *
 *  \return Ring read/write index value
 */
uint32_t Udma_ringGetWrIdx(Udma_RingHandle ringHandle);

/**
 *  \brief Returns the ring read index value.
 *
 *  Note: In case of devices like AM64x with LCDMA ring accelerator,
 *  this returns the read index value of the common ring.
 *  For other devices with normal ring accelerator,
 *  this returns the the read/write index value of the ring. Here its meaningful
 *  to pass the ringHandle of Completion Queue Ring.
 *
 *  Requirement: DOX_REQ_TAG(PDK-5665)
 *
 *  \param ringHandle   [IN] UDMA ring handle.
 *                           This parameter can't be NULL.
 *
 *  \return Ring read/write index value
 */
uint32_t Udma_ringGetRdIdx(Udma_RingHandle ringHandle);

/*
 * Structure Init functions
 */
/**
 *  \brief Udma_RingPrms structure init function.
 *
 *  \param ringPrms     [IN] Pointer to #Udma_RingPrms structure.
 *
 */
void UdmaRingPrms_init(Udma_RingPrms *ringPrms);

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                  Internal/Private Structure Declarations                   */
/* ========================================================================== */

/**
 *  \brief Opaque UDMA ring object.
 */
typedef struct Udma_RingObject_t
{
    uintptr_t rsv[30U];
    /**< reserved, should NOT be modified by end users */
} Udma_RingObject;

#ifdef __cplusplus
}
#endif

#endif /* #ifndef UDMA_RING_H_ */

/** @} */
