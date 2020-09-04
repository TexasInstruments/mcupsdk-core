/*
 *  Copyright (c) Texas Instruments Incorporated 2021
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
 *  \file cbuff_transfer.c
 *
 *      The file sets up the transfers for the CBUFF Driver.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <string.h>
#include <drivers/cbuff.h>

/* This is the size of the Chirp Parameters (CP) in CBUFF Units */
#define CP_SIZE_CBUFF_UNITS                         (2U)

/***************************************************************************************
 * Internal Exported Defintions:
 ***************************************************************************************/
extern CBUFF_setupTransferFxn   gInterleavedTransferFxn[];
extern CBUFF_setupTransferFxn   gNonInterleavedTransferFxn[];
extern CBUFF_setupTransferFxn   gMultipleChirpNonInterleavedTransferFxn[];

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* CBUFF Linked List API: */
static void CBUFF_initLinkedList   (CBUFF_Session* ptrSession);
static void CBUFF_addLinkedList    (CBUFF_Session* ptrSession, CBUFF_LLType type, CBUFF_LinkListParams* ptrLLParams);
static void CBUFF_removeLinkedList (CBUFF_Session* ptrSession, uint8_t listIndex);
static void CBUFF_displayLinkedList (CBUFF_Session* ptrSession, uint8_t linkListIndex);

/* CBUFF User Buffer API: */
static void    CBUFF_addUserBufferLLEntry (CBUFF_Session* ptrSession, CBUFF_LinkListParams* ptrLLParams);
static int32_t CBUFF_configUserBufferEDMA (CBUFF_Session* ptrSession);

/* CBUFF CQ API: */
static void    CBUFF_addCQLLEntry (CBUFF_Session* ptrSession, CBUFF_LinkListParams* ptrLLParams);
static int32_t CBUFF_configCQEDMA (CBUFF_Session* ptrSession);

/* CBUFF Header API: */
static int32_t CBUFF_setupHeader (CBUFF_Session* ptrSession, CBUFF_LinkListParams* ptrLinkListParams, int32_t* errCode);

/* EDMA Exported API: */
extern int32_t CBUFF_configEDMA(CBUFF_Session* ptrSession, uint32_t srcAddress, uint32_t transferSize, bool isLast);
extern void CBUFF_closeEDMA(CBUFF_Session* ptrSession);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/**
 *  @b Description
 *  @n
 *      The function is used to log the specified linked list entry for debugging.
 *
 *  @param[in]  ptrSession
 *      Pointer to the CBUFF session
 *  @param[in]  linkListIndex
 *      Linked List Index to be displayed
 *
 *  \ingroup CBUFF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
static void CBUFF_displayLinkedList (CBUFF_Session* ptrSession, uint8_t linkListIndex)
{
    CBUFF_LinkListParams*   ptrLinkListParams;

    /* Get the link list parameters associated with the session: */
    ptrLinkListParams = &ptrSession->linkedListTracker[linkListIndex];

    /* Display the linked list entry: */
    DebugP_logInfo ("********************************************************\n");
    DebugP_logInfo ("Session 0x%x Linked List Entry %d \n", (uintptr_t)ptrSession, linkListIndex);
    DebugP_logInfo ("CRC         : %s\n", (uintptr_t)((ptrLinkListParams->crcEnable == 1U) ? "Enabled" : "Disabled"));
    DebugP_logInfo ("LP Header   : %s\n", (uintptr_t)((ptrLinkListParams->lpHdrEn    == 1U) ? "New Packet" : "Continuation"));
    DebugP_logInfo ("Size        : %d\n", ptrLinkListParams->transferSize);
    DebugP_logInfo ("HSYNC Start : %d\n", ptrLinkListParams->hsyncStart);
    DebugP_logInfo ("HSYNC End   : %d\n", ptrLinkListParams->hsyncEnd);
    DebugP_logInfo ("DMA Number  : %d\n", ptrSession->dmaNum);
    DebugP_logInfo ("********************************************************\n");

    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to add an entry to the CBUFF Linked List
 *
 *  @param[in]  ptrSession
 *      Pointer to the CBUFF session
 *  @param[in]  type
 *      Linked List Type which is being added
 *  @param[in]  ptrLLParams
 *      Pointer to the Linked List parameters
 *
 *  \ingroup CBUFF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
static void CBUFF_addLinkedList
(
    CBUFF_Session*          ptrSession,
    CBUFF_LLType            type,
    CBUFF_LinkListParams*   ptrLLParams
)
{
    uint8_t     linkListIndex;

    /* Was there a transfer size specified? */
    if (ptrLLParams->transferSize == 0U)
    {
        /* NO: There is no need to program the Linked List Entry */
        return;
    }

    /* Sanity Check: Ensure that the Linked List Index is within the allowed range */
    DebugP_assert (ptrSession->linkListIndex < CBUFF_MAX_LINKED_LIST_SIZE);

    /* Get the linked list index: */
    linkListIndex = ptrSession->linkListIndex;

    /* When we add to the linked list; the entry is always considered to be the last */
    ptrLLParams->hsyncEnd = 1U;

    /* Store the type */
    ptrLLParams->type = type;

    /* Finalize the linked list entry: */
    ptrSession->ptrDriverMCB->interfaceFxn->finalizeLinkListParamsFxn (ptrSession, ptrLLParams);

    /* Track the linked list entry in the session: */
    memcpy ((void*)&ptrSession->linkedListTracker[linkListIndex], (void*)ptrLLParams, sizeof(CBUFF_LinkListParams));

    /* Is this the head of the linked list? */
    if (linkListIndex != 0U)
    {
        /* NO: Get the previous head of the list and reset the HSYNC End */
        ptrSession->linkedListTracker[linkListIndex - 1U].hsyncEnd = 0U;
    }

    /* For Debug: Display the linked list entry for the session. */
    CBUFF_displayLinkedList (ptrSession, linkListIndex);

    /* Increment to the next entry: */
    ptrSession->linkListIndex = linkListIndex + 1U;

    /* There are certain fields which are valid only for the first LL entry. Here we reset these fields
     * so that as the chain is created there is no chance to mess up.  */
    ptrLLParams->hsyncStart = 0U;
    ptrLLParams->lpHdrEn    = 0U;

    /**********************************************************************************************
     * Overwrite and set the DMA Number to be 7 for all subsequent entries.
     **********************************************************************************************/
    ptrLLParams->threshold = CSL_FINSR (ptrLLParams->threshold, 18U, 16U, 7U);

    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to delete the CBUFF linked list
 *
 *  @param[in]  ptrSession
 *      Pointer to the CBUFF session
 *  @param[in]  listIndex
 *      Linked List Index to be deleted
 *
 *  \ingroup CBUFF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
static void CBUFF_removeLinkedList
(
    CBUFF_Session*  ptrSession,
    uint8_t         listIndex
)
{
    /* Reset the linked list tracking entry: */
    memset ((void*)&ptrSession->linkedListTracker[listIndex], 0, sizeof(CBUFF_LinkListParams));
}

/**
 *  @b Description
 *  @n
 *      The function is used to initialize the CBUFF linked list
 *      for the CBUFF session
 *
 *  @param[in]  ptrSession
 *      Pointer to the CBUFF session
 *
 *  \ingroup CBUFF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
static void CBUFF_initLinkedList (CBUFF_Session* ptrSession)
{
    uint8_t    index;

    /* Reset the linked list index: */
    ptrSession->linkListIndex = 0U;

    /* Remove each entry from the linked list: */
    for (index = 0U; index < CBUFF_MAX_LINKED_LIST_SIZE; index++)
    {
        CBUFF_removeLinkedList (ptrSession, index);
    }
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to program the linked list entry for the user
 *      buffers. This is treated a bit differently because user buffers could
 *      be optional from at least 1 to CBUFF_MAX_USER_BUFFER.
 *
 *  @param[in]  ptrSession
 *      Pointer to the CBUFF session
 *  @param[in]  ptrLLParams
 *      Pointer to the initialized linked list entry which is used to program
 *      the linked list.
 *
 *  \ingroup CBUFF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
static void CBUFF_addUserBufferLLEntry (CBUFF_Session* ptrSession, CBUFF_LinkListParams* ptrLLParams)
{
    uint8_t index;

    /* Cycle through all possible user buffers: */
    for (index = 0; index < CBUFF_MAX_USER_BUFFER; index++)
    {
        /* Are we operating in Hardware or Software Triggered Execution mode? */
        if (ptrSession->sessionCfg.executionMode == CBUFF_SessionExecuteMode_HW)
        {
            /* Hardware Triggered Execution Mode: */
            ptrLLParams->transferSize = ptrSession->sessionCfg.u.hwCfg.userBufferInfo[index].size;
        }
        else
        {
            /* Software Triggered Execution Mode: */
            ptrLLParams->transferSize = ptrSession->sessionCfg.u.swCfg.userBufferInfo[index].size;
        }
        ptrLLParams->crcEnable    = 0U;
        CBUFF_addLinkedList (ptrSession, CBUFF_LLType_USER, ptrLLParams);
    }
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to program the EDMA transfers for the user buffers.
 *      This is treated a bit differently because user buffers could be optional
 *      from at least 1 to CBUFF_MAX_USER_BUFFER.
 *
 *  @param[in]  ptrSession
 *      Pointer to the CBUFF session
 *
 *  \ingroup CBUFF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   Error code
 */
static int32_t CBUFF_configUserBufferEDMA (CBUFF_Session* ptrSession)
{
    bool                isLast = false;
    int32_t             retVal = CBUFF_STATUS_SUCCESS;
    CBUFF_BufferCfg*    ptrUserBuffer;
    uint8_t             index;
    uint8_t             lastUserBufferIndex = 0U;

    /****************************************************************************
     * Determine the last *configured* user buffer
     ****************************************************************************/
    while (lastUserBufferIndex < CBUFF_MAX_USER_BUFFER)
    {
        /* Are we operating in Hardware or Software Triggered Execution mode? */
        if (ptrSession->sessionCfg.executionMode == CBUFF_SessionExecuteMode_HW)
        {
            /* Hardware Triggered Execution Mode: */
            ptrUserBuffer = &ptrSession->sessionCfg.u.hwCfg.userBufferInfo[lastUserBufferIndex];
        }
        else
        {
            /* Software Triggered Execution Mode: */
            ptrUserBuffer = &ptrSession->sessionCfg.u.swCfg.userBufferInfo[lastUserBufferIndex];
        }

        /* Is this the last buffer */
        if (ptrUserBuffer->size == 0U)
        {
            /* YES: This is the last buffer. */
            break;
        }

        /* Increment the last buffer index. */
        lastUserBufferIndex = lastUserBufferIndex + 1U;
    }

    /* Was there a user buffer specified? */
    if (lastUserBufferIndex == 0U)
    {
        /* Error: Control comes here implies that there was no user buffer specified.
         * This is an invalid argument because at least 1 user buffer should have been
         * provided. */
        retVal = CBUFF_EINVAL;
        DebugP_assert(0);
    }

    /****************************************************************************
     * Configure the EDMA Transfers
     ****************************************************************************/
    for (index = 0U; index < lastUserBufferIndex; index++)
    {
        /* Are we operating in Hardware or Software Triggered Execution mode? */
        if (ptrSession->sessionCfg.executionMode == CBUFF_SessionExecuteMode_HW)
        {
            /* Hardware Triggered Execution Mode: */
            ptrUserBuffer = &ptrSession->sessionCfg.u.hwCfg.userBufferInfo[index];
        }
        else
        {
            /* Software Triggered Execution Mode: */
            ptrUserBuffer = &ptrSession->sessionCfg.u.swCfg.userBufferInfo[index];
        }

        /* Are there more user supplied buffers? */
        if (index == (lastUserBufferIndex - 1U))
        {
            /* NO: This is the last user buffer specified. */
            isLast = true;
        }

        /**********************************************************************
         * Transfer N+1:
         * - Source Address: User Buffer
         * - Transfer Size : Size of the User Buffer
         **********************************************************************/
        retVal = CBUFF_configEDMA (ptrSession,
                                   ptrUserBuffer->address,
                                   ptrUserBuffer->size,
                                   isLast);

        DebugP_assert(retVal == CBUFF_STATUS_SUCCESS);
    }

    /* EDMA Transfers configured successfully */
    retVal = 0;

    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to program the linked list entry for the chirp
 *      quality.
 *
 *  @param[in]  ptrSession
 *      Pointer to the CBUFF session
 *  @param[in]  ptrLLParams
 *      Pointer to the initialized linked list entry which is used to program
 *      the linked list.
 *
 *  \ingroup CBUFF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
static void CBUFF_addCQLLEntry (CBUFF_Session* ptrSession, CBUFF_LinkListParams* ptrLLParams)
{
    uint8_t index;

    /* Cycle through all chirp quality: */
    for (index = 0; index < ptrSession->numCQ; index++)
    {
        ptrLLParams->crcEnable    = 0U;
        ptrLLParams->transferSize = ptrSession->cqBuffer[index].size;
        CBUFF_addLinkedList (ptrSession, CBUFF_LLType_CQ, ptrLLParams);
    }
}

/**
 *  @b Description
 *  @n
 *      The function is used to program the EDMA transfers for the chirp quality.
 *
 *  @param[in]  ptrSession
 *      Pointer to the CBUFF session
 *
 *  \ingroup CBUFF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   Error code
 */
static int32_t CBUFF_configCQEDMA (CBUFF_Session* ptrSession)
{
    bool        isLast = false;
    int32_t     retVal = CBUFF_STATUS_SUCCESS;
    uint8_t     index;

    /****************************************************************************
     * Configure the EDMA Transfers
     ****************************************************************************/
    for (index = 0U; index < ptrSession->numCQ; index++)
    {
        /* Determine which data format:  */
        if (ptrSession->sessionCfg.u.hwCfg.dataFormat == CBUFF_DataFmt_CP_ADC_CQ)
        {
            /* CP_ADC_CQ: This implies that the last CQ will be the last entry in
             * the EDMA chain. */
            if (index == (ptrSession->numCQ - 1U))
            {
                /* YES: This is the last CQ specified. */
                isLast = true;
            }
        }
        else
        {
            /* CP_ADC_CQ_USER: This implies that the user data will also follow the
             * CQ Data. The CQ data will *never* be the last entry in the EDMA chain. */
        }

        /**********************************************************************
         * Transfer N+1:
         * - Source Address: CQ Base Address
         * - Transfer Size : Size of the CQ
         **********************************************************************/
        retVal = CBUFF_configEDMA (ptrSession,
                                   ptrSession->cqBuffer[index].address,
                                   ptrSession->cqBuffer[index].size,
                                   isLast);

        DebugP_assert(retVal == CBUFF_STATUS_SUCCESS);
    }

    /* EDMA Transfers configured successfully */
    retVal = 0;

    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to setup and program the optional header. The function
 *      will create the Linked List Entry and will also setup the EDMA channel for
 *      the header if one is to be configured
 *
 *  @param[in]  ptrSession
 *      Pointer to the CBUFF session
 *  @param[in]  ptrLinkListParams
 *      Pointer to the Linked List Params
 *  @param[out]  errCode
 *      Error code populated on error
 *
 *  \ingroup CBUFF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t CBUFF_setupHeader
(
    CBUFF_Session*          ptrSession,
    CBUFF_LinkListParams*   ptrLinkListParams,
    int32_t*                errCode
)
{
    int32_t     retVal = CBUFF_STATUS_SUCCESS;

    /************************************************************
     * [Optional]: Header
     ************************************************************/
    if (ptrSession->sessionCfg.header.size != 0U)
    {
        /* Program the header in the Linked List: */
        ptrLinkListParams->crcEnable    = 0U;
        ptrLinkListParams->transferSize = ptrSession->sessionCfg.header.size;
        CBUFF_addLinkedList (ptrSession, CBUFF_LLType_HEADER, ptrLinkListParams);

        /**********************************************************************
         * Configure the EDMA Transfer:
         *  Transfer 1 [Optional]:
         *  - Source Address: Header Address
         *  - Transfer Size : Size of the Header
         **********************************************************************/
        *errCode = CBUFF_configEDMA (ptrSession,
                                     ptrSession->sessionCfg.header.address,
                                     ptrSession->sessionCfg.header.size,
                                     false);

        DebugP_assert(*errCode == CBUFF_STATUS_SUCCESS);
    }

    /* Header has been configured: */
    retVal = 0;

    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to setup the data transfers to handle the following
 *      use case:-
 *          - Interleaved
 *          - ADC Data Only
 *
 *  @param[in]  ptrDriverMCB
 *      Pointer to the CBUFF Driver
 *  @param[in]  ptrSession
 *      Pointer to the CBUFF Session
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup CBUFF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -  <0
 */
int32_t CBUFF_setupInterleaved_ADC
(
    CBUFF_Object*       ptrDriverMCB,
    CBUFF_Session*      ptrSession,
    int32_t*            errCode
)
{
    CBUFF_LinkListParams    linkListParams;
    int32_t                 retVal = CBUFF_STATUS_SUCCESS;

    /* Initialize the error code: */
    *errCode = 0;

    /* Initialize the linked list configuration: */
    ptrDriverMCB->interfaceFxn->initLinkListParamsFxn (ptrSession, &linkListParams);

    /* Setup the [Optional] Header: */
    retVal = CBUFF_setupHeader (ptrSession, &linkListParams, errCode);
    DebugP_assert(retVal == CBUFF_STATUS_SUCCESS);

    /************************************************************
     * ADC Data:
     ************************************************************/
    linkListParams.crcEnable    = ptrDriverMCB->initCfg.crcEnable;
    linkListParams.transferSize = (ptrSession->adcTransferSize *
                                   ptrSession->numActiveADCChannels);
    CBUFF_addLinkedList (ptrSession, CBUFF_LLType_ADCDATA, &linkListParams);

    /**********************************************************************
     * Transfer 2:
     * - Source Address: ADC Buffer
     * - Transfer Size : ADC Buffer for all channels
     **********************************************************************/
    *errCode = CBUFF_configEDMA (ptrSession,
                                 ptrDriverMCB->hwAttrs->adcBufBaseAddr,
                                 (ptrSession->adcTransferSize * ptrSession->numActiveADCChannels),
                                 true);

    DebugP_assert(*errCode == CBUFF_STATUS_SUCCESS);

    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to setup the data transfers to handle the following
 *      use case:-
 *          - Non Interleaved
 *          - ADC Data Only
 *
 *  @param[in]  ptrDriverMCB
 *      Pointer to the CBUFF Driver
 *  @param[in]  ptrSession
 *      Pointer to the CBUFF Session
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup CBUFF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -  <0
 */
int32_t CBUFF_setupNonInterleaved_ADC
(
    CBUFF_Object*       ptrDriverMCB,
    CBUFF_Session*      ptrSession,
    int32_t*            errCode
)
{
    CBUFF_LinkListParams    linkListParams;
    int32_t                 retVal = CBUFF_STATUS_SUCCESS;
    uint8_t                 index;
    bool                    isLast;
    uint8_t                 numChannelsProcessed = 0U;

    /* Initialize the error code: */
    *errCode = 0;

    /* Initialize the linked list configuration: */
    ptrDriverMCB->interfaceFxn->initLinkListParamsFxn (ptrSession, &linkListParams);

    /* Setup the [Optional] Header: */
    retVal = CBUFF_setupHeader (ptrSession, &linkListParams, errCode);
    DebugP_assert(retVal == CBUFF_STATUS_SUCCESS);

    /* Cycle through all the ADC Channels: */
    for (index = 0U; index < SOC_ADCBUF_NUM_RX_CHANNEL; index++)
    {
        /* Is the channel active? */
        if (ptrSession->rxChannelAddress[index] != 0U)
        {
            /************************************************************
             * ADC Data:
             ************************************************************/
            linkListParams.transferSize = ptrSession->adcTransferSize;
            linkListParams.crcEnable    = ptrDriverMCB->initCfg.crcEnable;
            CBUFF_addLinkedList (ptrSession, CBUFF_LLType_ADCDATA, &linkListParams);

            /* Done with the channel */
            numChannelsProcessed++;

            /* YES: Is this the last channel? */
            if (numChannelsProcessed == ptrSession->numActiveADCChannels)
            {
                /* YES: Set the flag to indicate so. */
                isLast = true;
            }
            else
            {
                /* NO: Reset the flag */
                isLast = false;
            }

            /**********************************************************************
             * Transfer 2, 3, 4, 5
             * - Source Address: ADC Data
             * - Transfer Size : Size of the ADC Data
             **********************************************************************/
            *errCode = CBUFF_configEDMA (ptrSession,
                                         ptrSession->rxChannelAddress[index],
                                         ptrSession->adcTransferSize,
                                         isLast);

            DebugP_assert(*errCode == CBUFF_STATUS_SUCCESS);
        }
    }

    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to setup the data transfers to handle the following
 *      use case:-
 *          - Interleaved
 *          - CP_ADC
 *
 *  @param[in]  ptrDriverMCB
 *      Pointer to the CBUFF Driver MCB
 *  @param[in]  ptrSession
 *      Pointer to the session
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup CBUFF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -  <0
 */
int32_t CBUFF_setupInterleaved_CP_ADC
(
    CBUFF_Object*       ptrDriverMCB,
    CBUFF_Session*      ptrSession,
    int32_t*            errCode
)
{
    CBUFF_LinkListParams    linkListParams;
    int32_t                 retVal = CBUFF_STATUS_SUCCESS;

    /* Initialize the error code: */
    *errCode = 0;

    /************************************************************
     * Program the CBUFF Linked List
     * - Initialize the linked list configuration
     ************************************************************/
    ptrDriverMCB->interfaceFxn->initLinkListParamsFxn (ptrSession, &linkListParams);

    /* Setup the [Optional] Header: */
    retVal = CBUFF_setupHeader (ptrSession, &linkListParams, errCode);
    DebugP_assert(retVal == CBUFF_STATUS_SUCCESS);

    /************************************************************
     * Chirp Parameters:
     ************************************************************/
    linkListParams.crcEnable    = 0U;
    linkListParams.transferSize = ptrSession->cpTotalTransferSize;
    CBUFF_addLinkedList (ptrSession, CBUFF_LLType_CP, &linkListParams);

    /**********************************************************************
     * Transfer 2:
     *  - Source Address: Chirp Parameters
     *  - Transfer Size : Size of all the Chirp Parameters
     **********************************************************************/
    *errCode = CBUFF_configEDMA (ptrSession,
                                 ptrDriverMCB->hwAttrs->cpSingleChirpInterleavedAddr[0],
                                 ptrSession->cpTotalTransferSize,
                                 false);

    DebugP_assert(*errCode == CBUFF_STATUS_SUCCESS);

    /************************************************************
     * ADC Data:
     ************************************************************/
    linkListParams.crcEnable    = ptrDriverMCB->initCfg.crcEnable;
    linkListParams.transferSize = (ptrSession->adcTransferSize * ptrSession->numActiveADCChannels);
    CBUFF_addLinkedList (ptrSession, CBUFF_LLType_ADCDATA, &linkListParams);

    /**********************************************************************
     * Transfer 3:
     *  - Source Address: ADC Buffer
     *  - Transfer Size : ADC Buffer size for all channels
     *  - This is the last transfer
     **********************************************************************/
    *errCode = CBUFF_configEDMA (ptrSession,
                                 ptrDriverMCB->hwAttrs->adcBufBaseAddr,
                                 (ptrSession->adcTransferSize * ptrSession->numActiveADCChannels),
                                 true);

    DebugP_assert(*errCode == CBUFF_STATUS_SUCCESS);

    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to setup the data transfers to handle the following
 *      use case:-
 *          - Non Interleaved
 *          - CP_ADC
 *
 *  @param[in]  ptrDriverMCB
 *      Pointer to the CBUFF Driver MCB
 *  @param[in]  ptrSession
 *      Pointer to the session
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup CBUFF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -  <0
 */
int32_t CBUFF_setupNonInterleaved_CP_ADC
(
    CBUFF_Object*       ptrDriverMCB,
    CBUFF_Session*      ptrSession,
    int32_t*            errCode
)
{
    CBUFF_LinkListParams    linkListParams;
    int32_t                 retVal = CBUFF_STATUS_SUCCESS;
    uint8_t                 index;
    bool                    isLast;
    uint8_t                 numChannelsProcessed = 0U;

    /* Initialize the error code: */
    *errCode = 0;

    /* Initialize the linked list parameters */
    ptrDriverMCB->interfaceFxn->initLinkListParamsFxn (ptrSession, &linkListParams);

    /* Setup the [Optional] Header: */
    retVal = CBUFF_setupHeader (ptrSession, &linkListParams, errCode);
    DebugP_assert(retVal == CBUFF_STATUS_SUCCESS);

    /**********************************************************************
     * Configure the EDMA Transfer(s): For each ACTIVE Channels; the
     * transfers are done as follows:
     *      [CP Rx0] [ADC data Rx0]
     *      [CP Rx1] [ADC data Rx1]
     *      [CP Rx2] [ADC data Rx2]
     *      [CP Rx3] [ADC data Rx3]
     **********************************************************************/
    for (index = 0U; index < SOC_ADCBUF_NUM_RX_CHANNEL; index++)
    {
        /* Is the channel active? */
        if (ptrSession->rxChannelAddress[index] != 0U)
        {
            /************************************************************
             * Chirp Parameters:
             ************************************************************/
            linkListParams.crcEnable    = 0U;
            linkListParams.transferSize = CP_SIZE_CBUFF_UNITS;
            CBUFF_addLinkedList (ptrSession, CBUFF_LLType_CP, &linkListParams);

            /**********************************************************************
             * Transfer 2, 4, 6, 8:
             *  - Source Address: Chirp Parameter 0,1,2,3
             *  - Transfer Size : Size of the Chirp Parameters 0,1,2,3
             **********************************************************************/
            *errCode = CBUFF_configEDMA (ptrSession,
                                         ptrDriverMCB->hwAttrs->cpSingleChirpNonInterleavedAddr[index],
                                         CP_SIZE_CBUFF_UNITS,
                                         false);
            DebugP_assert(*errCode == CBUFF_STATUS_SUCCESS);

            /* Done with the channel */
            numChannelsProcessed++;

            /* YES: Is this the last channel? */
            if (numChannelsProcessed == ptrSession->numActiveADCChannels)
            {
                /* YES: Set the flag to indicate so. */
                isLast = true;
            }
            else
            {
                /* NO: Reset the flag */
                isLast = false;
            }

            /************************************************************
             * ADC Data:
             ************************************************************/
            linkListParams.transferSize = ptrSession->adcTransferSize;
            linkListParams.crcEnable    = ptrDriverMCB->initCfg.crcEnable;
            CBUFF_addLinkedList (ptrSession, CBUFF_LLType_ADCDATA, &linkListParams);

            /**********************************************************************
             * Transfer 3, 5, 7, 9:
             *  - Source Address: ADC Buffer
             *  - Transfer Size : ADC Buffer size
             *  - If this is the last active channel then this is the last transfer
             **********************************************************************/
            *errCode = CBUFF_configEDMA (ptrSession,
                                        ptrSession->rxChannelAddress[index],
                                        ptrSession->adcTransferSize,
                                        isLast);

            DebugP_assert(*errCode == CBUFF_STATUS_SUCCESS);
        }
    }

    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to setup the data transfers to handle the following
 *      use case:-
 *          - Interleaved
 *          - ADC_CP
 *
 *  @param[in]  ptrDriverMCB
 *      Pointer to the CBUFF Driver MCB
 *  @param[in]  ptrSession
 *      Pointer to the session
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup CBUFF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -  <0
 */
int32_t CBUFF_setupInterleaved_ADC_CP
(
    CBUFF_Object*       ptrDriverMCB,
    CBUFF_Session*      ptrSession,
    int32_t*            errCode
)
{
    CBUFF_LinkListParams    linkListParams;
    int32_t                 retVal = CBUFF_STATUS_SUCCESS;

    /* Initialize the error code: */
    *errCode = 0;

    /************************************************************
     * Program the CBUFF Linked List
     ************************************************************/
    ptrDriverMCB->interfaceFxn->initLinkListParamsFxn (ptrSession, &linkListParams);

    /* Setup the [Optional] Header: */
    retVal = CBUFF_setupHeader (ptrSession, &linkListParams, errCode);
    DebugP_assert(retVal == CBUFF_STATUS_SUCCESS);

    /************************************************************
     * ADC Data:
     ************************************************************/
    linkListParams.crcEnable    = ptrDriverMCB->initCfg.crcEnable;
    linkListParams.transferSize = (ptrSession->adcTransferSize *
                                   ptrSession->numActiveADCChannels);
    CBUFF_addLinkedList (ptrSession, CBUFF_LLType_ADCDATA, &linkListParams);

    /**********************************************************************
     * Transfer 2:
     *  - Source Address: ADC Buffer
     *  - Transfer Size : ADC Buffer size for all channels
     **********************************************************************/
    *errCode = CBUFF_configEDMA (ptrSession,
                                 ptrDriverMCB->hwAttrs->adcBufBaseAddr,
                                 (ptrSession->adcTransferSize * ptrSession->numActiveADCChannels),
                                 false);

    DebugP_assert(*errCode == CBUFF_STATUS_SUCCESS);

    /************************************************************
     * Chirp Parameters:
     ************************************************************/
    linkListParams.crcEnable    = 0U;
    linkListParams.transferSize = ptrSession->cpTotalTransferSize;
    CBUFF_addLinkedList (ptrSession, CBUFF_LLType_CP, &linkListParams);

    /**********************************************************************
     * Transfer 3:
     *  - Source Address: Chirp Parameters 0,1,2,3
     *  - Transfer Size : Size of all the Chirp Parameters
     *  - This is the last transfer
     **********************************************************************/
    *errCode = CBUFF_configEDMA (ptrSession,
                                 ptrDriverMCB->hwAttrs->cpSingleChirpInterleavedAddr[0],
                                 ptrSession->cpTotalTransferSize,
                                 true);
    DebugP_assert(*errCode == CBUFF_STATUS_SUCCESS);

    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to setup the data transfers to handle the following
 *      use case:-
 *          - Non Interleaved
 *          - ADC_CP
 *
 *  @param[in]  ptrDriverMCB
 *      Pointer to the CBUFF Driver MCB
 *  @param[in]  ptrSession
 *      Pointer to the session
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup CBUFF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -  <0
 */
int32_t CBUFF_setupNonInterleaved_ADC_CP
(
    CBUFF_Object*       ptrDriverMCB,
    CBUFF_Session*      ptrSession,
    int32_t*            errCode
)
{
    CBUFF_LinkListParams    linkListParams;
    int32_t                 retVal = CBUFF_STATUS_SUCCESS;
    uint8_t                 index;
    bool                    isLast;
    uint8_t                 numChannelsProcessed = 0U;

    /* Initialize the error code: */
    *errCode = 0;

    /* Initialize the linked list parameters */
    ptrDriverMCB->interfaceFxn->initLinkListParamsFxn (ptrSession, &linkListParams);

    /* Setup the [Optional] Header: */
    retVal = CBUFF_setupHeader (ptrSession, &linkListParams, errCode);
    DebugP_assert(retVal == CBUFF_STATUS_SUCCESS);

    /**********************************************************************
     * Configure the EDMA Transfer(s): For each ACTIVE Channels; the
     * transfers are done as follows:
     *      [ADC data Rx0] [CP Rx0]
     *      [ADC data Rx1] [CP Rx1]
     *      [ADC data Rx2] [CP Rx2]
     *      [ADC data Rx3] [CP Rx3]
     **********************************************************************/
    for (index = 0U; index < SOC_ADCBUF_NUM_RX_CHANNEL; index++)
    {
        /* Is the channel active? */
        if (ptrSession->rxChannelAddress[index] != 0U)
        {
            /************************************************************
             * ADC Data:
             ************************************************************/
            linkListParams.transferSize = ptrSession->adcTransferSize;
            linkListParams.crcEnable    = ptrDriverMCB->initCfg.crcEnable;
            CBUFF_addLinkedList (ptrSession, CBUFF_LLType_ADCDATA, &linkListParams);

            /**********************************************************************
             * Transfer 2, 4, 6, 8:
             *  - Source Address: ADC Buffer
             *  - Transfer Size : ADC Buffer size
             **********************************************************************/
            *errCode = CBUFF_configEDMA (ptrSession,
                                         ptrSession->rxChannelAddress[index],
                                         ptrSession->adcTransferSize,
                                         false);
            DebugP_assert(*errCode == CBUFF_STATUS_SUCCESS);

            /* Done with the channel */
            numChannelsProcessed++;

            /* YES: Is this the last channel? */
            if (numChannelsProcessed == ptrSession->numActiveADCChannels)
            {
                /* YES: Set the flag to indicate so. */
                isLast = 1U;
            }
            else
            {
                /* NO: Reset the flag */
                isLast = 0U;
            }

            /************************************************************
             * Chirp Parameters:
             ************************************************************/
            linkListParams.crcEnable    = 0U;
            linkListParams.transferSize = CP_SIZE_CBUFF_UNITS;
            CBUFF_addLinkedList (ptrSession, CBUFF_LLType_CP, &linkListParams);

            /**********************************************************************
             * Transfer 3, 5, 7, 9:
             *  - Source Address: Chirp Parameters 0,1,2,3
             *  - Transfer Size : Size of the Chirp Parameters 0,1,2,3
             *  - If this is the last active channel then this will be the last
             *    transfer
             **********************************************************************/
            *errCode = CBUFF_configEDMA (ptrSession,
                                         ptrDriverMCB->hwAttrs->cpSingleChirpNonInterleavedAddr[index],
                                         CP_SIZE_CBUFF_UNITS,
                                         isLast);
            DebugP_assert(*errCode == CBUFF_STATUS_SUCCESS);
        }
    }

    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to setup the data transfers to handle the following
 *      use case:-
 *          - Interleaved
 *          - CP_ADC_CQ
 *
 *  @param[in]  ptrDriverMCB
 *      Pointer to the CBUFF Driver MCB
 *  @param[in]  ptrSession
 *      Pointer to the session
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup CBUFF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -  <0
 */
int32_t CBUFF_setupInterleaved_CP_ADC_CQ
(
    CBUFF_Object*       ptrDriverMCB,
    CBUFF_Session*      ptrSession,
    int32_t*            errCode
)
{
    CBUFF_LinkListParams    linkListParams;
    int32_t                 retVal = CBUFF_STATUS_SUCCESS;

    /* Initialize the error code: */
    *errCode = 0;

    /************************************************************
     * Program the CBUFF Linked List
     * - Initialize the linked list configuration
     ************************************************************/
    ptrDriverMCB->interfaceFxn->initLinkListParamsFxn (ptrSession, &linkListParams);

    /* Setup the [Optional] Header: */
    retVal = CBUFF_setupHeader (ptrSession, &linkListParams, errCode);
    DebugP_assert(retVal == CBUFF_STATUS_SUCCESS);

    /************************************************************
     * Chirp Parameters:
     ************************************************************/
    linkListParams.crcEnable    = 0U;
    linkListParams.transferSize = ptrSession->cpTotalTransferSize;
    CBUFF_addLinkedList (ptrSession, CBUFF_LLType_CP, &linkListParams);

    /**********************************************************************
     * Transfer 2:
     *  - Source Address: Chirp Parameters
     *  - Transfer Size : Size of all the Chirp Parameters
     **********************************************************************/
    *errCode = CBUFF_configEDMA (ptrSession,
                                 ptrDriverMCB->hwAttrs->cpSingleChirpInterleavedAddr[0],
                                 ptrSession->cpTotalTransferSize,
                                 false);

    DebugP_assert(*errCode == CBUFF_STATUS_SUCCESS);

    /************************************************************
     * ADC Data:
     ************************************************************/
    linkListParams.crcEnable    = ptrDriverMCB->initCfg.crcEnable;
    linkListParams.transferSize = (ptrSession->adcTransferSize *
                                   ptrSession->numActiveADCChannels);
    CBUFF_addLinkedList (ptrSession, CBUFF_LLType_ADCDATA, &linkListParams);

    /**********************************************************************
     * Transfer 3:
     *  - Source Address: ADC Buffer
     *  - Transfer Size : ADC Buffer size for all channels
     **********************************************************************/
    *errCode = CBUFF_configEDMA (ptrSession,
                                 ptrDriverMCB->hwAttrs->adcBufBaseAddr,
                                 (ptrSession->adcTransferSize * ptrSession->numActiveADCChannels),
                                 false);

    DebugP_assert(*errCode == CBUFF_STATUS_SUCCESS);

    /************************************************************
     * Chirp Quality:
     ************************************************************/
    CBUFF_addCQLLEntry (ptrSession, &linkListParams);

    /**********************************************************************
     * Transfer 4, 5:
     *  - Source Address: Chirp Quality(CQ1), Chirp Quality(CQ2)
     *  - Transfer Size : Chirp Quality Size for CQ1, CQ2
     **********************************************************************/
    *errCode = CBUFF_configCQEDMA (ptrSession);
    DebugP_assert(*errCode == CBUFF_STATUS_SUCCESS);

    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to setup the data transfers to handle the following
 *      use case:-
 *          - Non Interleaved
 *          - CP_ADC_CQ
 *
 *  @param[in]  ptrDriverMCB
 *      Pointer to the CBUFF Driver MCB
 *  @param[in]  ptrSession
 *      Pointer to the session
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup CBUFF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -  <0
 */
int32_t CBUFF_setupNonInterleaved_CP_ADC_CQ
(
    CBUFF_Object*       ptrDriverMCB,
    CBUFF_Session*      ptrSession,
    int32_t*            errCode
)
{
    CBUFF_LinkListParams    linkListParams;
    int32_t                 retVal = CBUFF_STATUS_SUCCESS;
    uint8_t                 index;

    /* Initialize the error code: */
    *errCode = 0;

    /* Initialize the linked list configuration: */
    ptrDriverMCB->interfaceFxn->initLinkListParamsFxn (ptrSession, &linkListParams);

    /* Setup the [Optional] Header: */
    retVal = CBUFF_setupHeader (ptrSession, &linkListParams, errCode);
    DebugP_assert(retVal == CBUFF_STATUS_SUCCESS);

    /**********************************************************************
     * Configure the EDMA Transfer(s): For each ACTIVE Channels; the
     * transfers are done as follows:
     *      [CP Rx0] [ADC data Rx0]
     *      [CP Rx1] [ADC data Rx1]
     *      [CP Rx2] [ADC data Rx2]
     *      [CP Rx3] [ADC data Rx3]
     **********************************************************************/
    for (index = 0U; index < SOC_ADCBUF_NUM_RX_CHANNEL; index++)
    {
        /* Is the channel active? */
        if (ptrSession->rxChannelAddress[index] != 0U)
        {
            /************************************************************
             * Chirp Parameters:
             ************************************************************/
            linkListParams.crcEnable    = 0U;
            linkListParams.transferSize = CP_SIZE_CBUFF_UNITS;
            CBUFF_addLinkedList (ptrSession, CBUFF_LLType_CP, &linkListParams);

            /**********************************************************************
             * Transfer 2, 4, 6, 8:
             *  - Source Address: Chirp Parameter 0,1,2,3
             *  - Transfer Size : Size of the Chirp Parameters 0,1,2,3
             **********************************************************************/
            *errCode = CBUFF_configEDMA (ptrSession,
                                         ptrDriverMCB->hwAttrs->cpSingleChirpNonInterleavedAddr[index],
                                         CP_SIZE_CBUFF_UNITS,
                                         false);

            DebugP_assert(*errCode == CBUFF_STATUS_SUCCESS);

            /************************************************************
             * ADC Data:
             ************************************************************/
            linkListParams.transferSize = ptrSession->adcTransferSize;
            linkListParams.crcEnable    = ptrDriverMCB->initCfg.crcEnable;
            CBUFF_addLinkedList (ptrSession, CBUFF_LLType_ADCDATA, &linkListParams);

            /**********************************************************************
             * Transfer 3, 5, 7, 9:
             *  - Source Address: ADC Buffer
             *  - Transfer Size : ADC Buffer size for each channel
             **********************************************************************/
            *errCode = CBUFF_configEDMA (ptrSession,
                                         ptrSession->rxChannelAddress[index],
                                         ptrSession->adcTransferSize,
                                         false);

            DebugP_assert(*errCode == CBUFF_STATUS_SUCCESS);
        }
    }

    /************************************************************
     * Chirp Quality:
     ************************************************************/
    CBUFF_addCQLLEntry (ptrSession, &linkListParams);

    /**********************************************************************
     * Transfer 10, 11:
     *  - Source Address: Chirp Quality(CQ1), Chirp Quality(CQ2)
     *  - Transfer Size : Chirp Quality Size for CQ1, CQ2
     **********************************************************************/
    *errCode = CBUFF_configCQEDMA (ptrSession);
    DebugP_assert(*errCode == CBUFF_STATUS_SUCCESS);

    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to setup the data transfers to handle the following
 *      use case:-
 *          - Interleaved
 *          - ADC Data
 *          - User Buffer
 *
 *  @param[in]  ptrDriverMCB
 *      Pointer to the CBUFF Driver MCB
 *  @param[in]  ptrSession
 *      Pointer to the session
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup CBUFF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -  <0
 */
int32_t CBUFF_setupInterleaved_ADC_USER
(
    CBUFF_Object*       ptrDriverMCB,
    CBUFF_Session*      ptrSession,
    int32_t*            errCode
)
{
    CBUFF_LinkListParams    linkListParams;
    int32_t                 retVal = CBUFF_STATUS_SUCCESS;

    /* Initialize the error code: */
    *errCode = 0;

    /************************************************************
     * Program the CBUFF Linked List
     * - Initialize the linked list configuration
     ************************************************************/
    ptrDriverMCB->interfaceFxn->initLinkListParamsFxn (ptrSession, &linkListParams);

    /* Setup the [Optional] Header: */
    retVal = CBUFF_setupHeader (ptrSession, &linkListParams, errCode);
    DebugP_assert(retVal == CBUFF_STATUS_SUCCESS);

    /************************************************************
     * Chirp Mode: ADC Data
     ************************************************************/
    linkListParams.crcEnable    = ptrDriverMCB->initCfg.crcEnable;
    linkListParams.transferSize = (ptrSession->adcTransferSize *
                                   ptrSession->numActiveADCChannels);
    CBUFF_addLinkedList (ptrSession, CBUFF_LLType_ADCDATA, &linkListParams);

    /**********************************************************************
     * Configure the EDMA Transfer:
     * Transfer 2:
     *  - Source Address: ADC Buffer
     *  - Transfer Size : ADC Buffer for all channels
     **********************************************************************/
    *errCode = CBUFF_configEDMA (ptrSession, ptrDriverMCB->hwAttrs->adcBufBaseAddr,
                                 (ptrSession->adcTransferSize * ptrSession->numActiveADCChannels),
                                 false);

    DebugP_assert(*errCode == CBUFF_STATUS_SUCCESS);

    /************************************************************
     * User Data:
     ************************************************************/
    CBUFF_addUserBufferLLEntry (ptrSession, &linkListParams);

    /**********************************************************************
     * Transfer 3, 4, 5:
     *  - Source Address: User Buffer1/UserBuffer2/UserBuffer3
     *  - Transfer Size : Size of the UserBuffer1/UserBuffer2/UserBuffer3
     **********************************************************************/
    *errCode = CBUFF_configUserBufferEDMA (ptrSession);
    DebugP_assert(*errCode == CBUFF_STATUS_SUCCESS);

    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to setup the data transfers to handle the following
 *      use case:-
 *          - Non Interleaved
 *          - ADC Data
 *          - User Buffers
 *
 *  @param[in]  ptrDriverMCB
 *      Pointer to the CBUFF Driver MCB
 *  @param[in]  ptrSession
 *      Pointer to the session
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup CBUFF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -  <0
 */
int32_t CBUFF_setupNonInterleaved_ADC_USER
(
    CBUFF_Object*       ptrDriverMCB,
    CBUFF_Session*      ptrSession,
    int32_t*            errCode
)
{
    CBUFF_LinkListParams    linkListParams;
    int32_t                 retVal = CBUFF_STATUS_SUCCESS;
    uint32_t                index;

    /* Initialize the error code: */
    *errCode = 0;

    /* Initialize the linked list configuration: */
    ptrDriverMCB->interfaceFxn->initLinkListParamsFxn (ptrSession, &linkListParams);

    /* Setup the [Optional] Header: */
    retVal = CBUFF_setupHeader (ptrSession, &linkListParams, errCode);
    DebugP_assert(retVal == CBUFF_STATUS_SUCCESS);

    /* Cycle through all the ADC Channels: */
    for (index = 0U; index < SOC_ADCBUF_NUM_RX_CHANNEL; index++)
    {
        /* Is the channel active? */
        if (ptrSession->rxChannelAddress[index] != 0U)
        {
            /************************************************************
             * ADC Data:
             ************************************************************/
            linkListParams.transferSize = ptrSession->adcTransferSize;
            linkListParams.crcEnable    = ptrDriverMCB->initCfg.crcEnable;
            CBUFF_addLinkedList (ptrSession, CBUFF_LLType_ADCDATA, &linkListParams);

            /**********************************************************************
             * Transfer 2, 3, 4, 5
             * - Source Address: ADC Data
             * - Transfer Size : Size of the ADC Data
             **********************************************************************/
            *errCode = CBUFF_configEDMA (ptrSession,
                                          ptrSession->rxChannelAddress[index],
                                          ptrSession->adcTransferSize,
                                          false);

            DebugP_assert(*errCode == CBUFF_STATUS_SUCCESS);
        }
    }

    /************************************************************
     * User Data:
     ************************************************************/
    CBUFF_addUserBufferLLEntry (ptrSession, &linkListParams);

    /**********************************************************************
     * Transfer 6, 7, 8:
     *  - Source Address: User Buffer1/UserBuffer2/UserBuffer3
     *  - Transfer Size : Size of the UserBuffer1/UserBuffer2/UserBuffer3
     **********************************************************************/
    *errCode = CBUFF_configUserBufferEDMA (ptrSession);
    DebugP_assert(*errCode == CBUFF_STATUS_SUCCESS);

    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to setup the data transfers to handle the following
 *      use case:-
 *          - Interleaved
 *          - CP_ADC_CQ
 *          - User Data
 *
 *  @param[in]  ptrDriverMCB
 *      Pointer to the CBUFF Driver MCB
 *  @param[in]  ptrSession
 *      Pointer to the session
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup CBUFF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -  <0
 */
int32_t CBUFF_setupInterleaved_CP_ADC_CQ_USER
(
    CBUFF_Object*       ptrDriverMCB,
    CBUFF_Session*      ptrSession,
    int32_t*            errCode
)
{
    CBUFF_LinkListParams    linkListParams;
    int32_t                 retVal = CBUFF_STATUS_SUCCESS;

    /************************************************************
     * Program the CBUFF Linked List
     * - Initialize the linked list configuration
     ************************************************************/
    ptrDriverMCB->interfaceFxn->initLinkListParamsFxn (ptrSession, &linkListParams);

    /* Setup the [Optional] Header: */
    retVal = CBUFF_setupHeader (ptrSession, &linkListParams, errCode);
    DebugP_assert(retVal == CBUFF_STATUS_SUCCESS);

    /************************************************************
     * Chirp Parameters:
     ************************************************************/
    linkListParams.crcEnable    = 0U;
    linkListParams.transferSize = ptrSession->cpTotalTransferSize;
    CBUFF_addLinkedList (ptrSession, CBUFF_LLType_CP, &linkListParams);

    /**********************************************************************
     * Transfer 2:
     *  - Source Address: Chirp Parameters
     *  - Transfer Size : Size of all the Chirp Parameters
     **********************************************************************/
    *errCode = CBUFF_configEDMA (ptrSession,
                                 ptrDriverMCB->hwAttrs->cpSingleChirpInterleavedAddr[0],
                                 ptrSession->cpTotalTransferSize,
                                 false);
    DebugP_assert(*errCode == CBUFF_STATUS_SUCCESS);

    /************************************************************
     * ADC Data:
     ************************************************************/
    linkListParams.crcEnable    = ptrDriverMCB->initCfg.crcEnable;
    linkListParams.transferSize = (ptrSession->adcTransferSize *
                                   ptrSession->numActiveADCChannels);
    CBUFF_addLinkedList (ptrSession, CBUFF_LLType_ADCDATA, &linkListParams);

    /**********************************************************************
     * Transfer 3:
     *  - Source Address: ADC Buffer
     *  - Transfer Size : ADC Buffer size for all channels
     **********************************************************************/
    *errCode = CBUFF_configEDMA (ptrSession,
                                 ptrDriverMCB->hwAttrs->adcBufBaseAddr,
                                 (ptrSession->adcTransferSize * ptrSession->numActiveADCChannels),
                                 false);
    DebugP_assert(*errCode == CBUFF_STATUS_SUCCESS);

    /************************************************************
     * Chirp Quality:
     ************************************************************/
    CBUFF_addCQLLEntry (ptrSession, &linkListParams);

    /**********************************************************************
     * Transfer 4, 5:
     *  - Source Address: Chirp Quality(CQ1), Chirp Quality(CQ2)
     *  - Transfer Size : Chirp Quality Size for CQ1, CQ2
     **********************************************************************/
    *errCode = CBUFF_configCQEDMA (ptrSession);
    DebugP_assert(*errCode == CBUFF_STATUS_SUCCESS);

    /************************************************************
     * User Data:
     ************************************************************/
    CBUFF_addUserBufferLLEntry (ptrSession, &linkListParams);

    /**********************************************************************
     * Transfer 6, 7, 8:
     *  - Source Address: User Buffer1/UserBuffer2/UserBuffer3
     *  - Transfer Size : Size of the UserBuffer1/UserBuffer2/UserBuffer3
     **********************************************************************/
    *errCode = CBUFF_configUserBufferEDMA (ptrSession);
    DebugP_assert(*errCode == CBUFF_STATUS_SUCCESS);

    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to setup the data transfers to handle the following
 *      use case:-
 *          - Non Interleaved
 *          - CP_ADC_CQ
 *          - User Data
 *
 *  @param[in]  ptrDriverMCB
 *      Pointer to the CBUFF Driver MCB
 *  @param[in]  ptrSession
 *      Pointer to the session
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup CBUFF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -  <0
 */
int32_t CBUFF_setupNonInterleaved_CP_ADC_CQ_USER
(
    CBUFF_Object*       ptrDriverMCB,
    CBUFF_Session*      ptrSession,
    int32_t*            errCode
)
{
    CBUFF_LinkListParams    linkListParams;
    int32_t                 retVal = CBUFF_STATUS_SUCCESS;
    uint8_t                 index;

    /* Initialize the linked list configuration: */
    ptrDriverMCB->interfaceFxn->initLinkListParamsFxn (ptrSession, &linkListParams);

    /* Setup the [Optional] Header: */
    retVal = CBUFF_setupHeader (ptrSession, &linkListParams, errCode);
    DebugP_assert(retVal == CBUFF_STATUS_SUCCESS);

    /**********************************************************************
     * Configure the EDMA Transfer(s): For each ACTIVE Channels; the
     * transfers are done as follows:
     *      [CP Rx0] [ADC data Rx0]
     *      [CP Rx1] [ADC data Rx1]
     *      [CP Rx2] [ADC data Rx2]
     *      [CP Rx3] [ADC data Rx3]
     **********************************************************************/
    for (index = 0U; index < SOC_ADCBUF_NUM_RX_CHANNEL; index++)
    {
        /* Is the channel active? */
        if (ptrSession->rxChannelAddress[index] != 0U)
        {
            /************************************************************
             * Chirp Parameters:
             ************************************************************/
            linkListParams.crcEnable    = 0U;
            linkListParams.transferSize = CP_SIZE_CBUFF_UNITS;
            CBUFF_addLinkedList (ptrSession, CBUFF_LLType_CP, &linkListParams);

            /**********************************************************************
             * Transfer 2, 4, 6, 8:
             *  - Source Address: Chirp Parameters
             *  - Transfer Size : Size of the Chirp Parameter
             **********************************************************************/
            *errCode = CBUFF_configEDMA (ptrSession,
                                         ptrDriverMCB->hwAttrs->cpSingleChirpNonInterleavedAddr[index],
                                         CP_SIZE_CBUFF_UNITS,
                                         false);
            DebugP_assert(*errCode == CBUFF_STATUS_SUCCESS);

            /************************************************************
             * ADC Data:
             ************************************************************/
            linkListParams.transferSize = ptrSession->adcTransferSize;
            linkListParams.crcEnable    = ptrDriverMCB->initCfg.crcEnable;
            CBUFF_addLinkedList (ptrSession, CBUFF_LLType_ADCDATA, &linkListParams);

            /**********************************************************************
             * Transfer 3, 5, 7, 9:
             *  - Source Address: ADC Buffer
             *  - Transfer Size : ADC Buffer size for all channels
             **********************************************************************/
            *errCode = CBUFF_configEDMA (ptrSession,
                                         ptrSession->rxChannelAddress[index],
                                         ptrSession->adcTransferSize,
                                         false);
            DebugP_assert(*errCode == CBUFF_STATUS_SUCCESS);
        }
    }

    /************************************************************
     * Chirp Quality:
     ************************************************************/
    CBUFF_addCQLLEntry (ptrSession, &linkListParams);

    /**********************************************************************
     * Transfer 10, 11:
     *  - Source Address: Chirp Quality(CQ1), Chirp Quality(CQ2)
     *  - Transfer Size : Chirp Quality Size for CQ1, CQ2
     **********************************************************************/
    *errCode = CBUFF_configCQEDMA (ptrSession);
    DebugP_assert(*errCode == CBUFF_STATUS_SUCCESS);

    /************************************************************
     * User Data:
     ************************************************************/
    CBUFF_addUserBufferLLEntry (ptrSession, &linkListParams);

    /**********************************************************************
     * Transfer 12, 13, 14:
     *  - Source Address: User Buffer1/UserBuffer2/UserBuffer3
     *  - Transfer Size : Size of the UserBuffer1/UserBuffer2/UserBuffer3
     **********************************************************************/
    *errCode = CBUFF_configUserBufferEDMA (ptrSession);
    DebugP_assert(*errCode == CBUFF_STATUS_SUCCESS);

    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to setup the data transfers to handle the following
 *      use case:-
 *          - User Buffer
 *      For User data we ignore the Interleaved/Non-interleaved
 *
 *  @param[in]  ptrDriverMCB
 *      Pointer to the CBUFF Driver MCB
 *  @param[in]  ptrSession
 *      Pointer to the session
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup CBUFF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -  <0
 */
int32_t CBUFF_setup_USER (CBUFF_Object* ptrDriverMCB, CBUFF_Session* ptrSession, int32_t* errCode)
{
    CBUFF_LinkListParams    linkListParams;
    int32_t                 retVal = CBUFF_STATUS_SUCCESS;

    /* Initialize the error code: */
    *errCode = 0;

    /************************************************************
     * Program the CBUFF Linked List
     * - Initialize the linked list configuration
     ************************************************************/
    ptrDriverMCB->interfaceFxn->initLinkListParamsFxn (ptrSession, &linkListParams);

    /* Setup the [Optional] Header: */
    retVal = CBUFF_setupHeader (ptrSession, &linkListParams, errCode);
    DebugP_assert(retVal == CBUFF_STATUS_SUCCESS);

    /************************************************************
     * User Data:
     ************************************************************/
    CBUFF_addUserBufferLLEntry (ptrSession, &linkListParams);

    /**********************************************************************
     * Transfer 2, 3, 4:
     *  - Source Address: User Buffer1/UserBuffer2/UserBuffer3
     *  - Transfer Size : Size of the UserBuffer1/UserBuffer2/UserBuffer3
     **********************************************************************/
    *errCode = CBUFF_configUserBufferEDMA (ptrSession);
    DebugP_assert(*errCode == CBUFF_STATUS_SUCCESS);

    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to setup the data transfers to handle the following
 *      use case:-
 *          - Multiple Chirp Mode
 *          - Non Interleaved
 *          - CP_ADC
 *
 *  @param[in]  ptrDriverMCB
 *      Pointer to the CBUFF Driver MCB
 *  @param[in]  ptrSession
 *      Pointer to the session
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup CBUFF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -  <0
 */
int32_t CBUFF_setupNonInterleaved_MultiChirp_CP_ADC
(
    CBUFF_Object*       ptrDriverMCB,
    CBUFF_Session*      ptrSession,
    int32_t*            errCode
)
{
    CBUFF_LinkListParams    linkListParams;
    int32_t                 retVal = CBUFF_STATUS_SUCCESS;
    uint8_t                 index;
    bool                    isLast;
    uint8_t                 numChannelsProcessed = 0U;

    /* Initialize the error code: */
    *errCode = 0;

    /* Sanity Check: This is only to be invoked in Multiple Chirp Mode */
    DebugP_assert (ptrSession->sessionCfg.u.hwCfg.chirpMode > 1U);

    /* Initialize the linked list parameters */
    ptrDriverMCB->interfaceFxn->initLinkListParamsFxn (ptrSession, &linkListParams);

    /* Setup the [Optional] Header: */
    retVal = CBUFF_setupHeader (ptrSession, &linkListParams, errCode);
    DebugP_assert(retVal == CBUFF_STATUS_SUCCESS);

    /************************************************************
     * Chirp Parameters for Multiple Chirps:
     ************************************************************/
    for (index = 1U; index <= ptrSession->sessionCfg.u.hwCfg.chirpMode; index++)
    {
        /************************************************************
         * Chirp Parameters:
         ************************************************************/
        linkListParams.crcEnable    = 0U;
        linkListParams.transferSize = CP_SIZE_CBUFF_UNITS * SOC_ADCBUF_NUM_RX_CHANNEL;
        CBUFF_addLinkedList (ptrSession, CBUFF_LLType_CP, &linkListParams);

        /**********************************************************************
         * Transfer 2, 3, 4, 5, 6, 7, 8 and 9:
         *  - Source Address: CPx
         *  - Transfer Size : Size of the Chirp Parameters
         * Where X is the Chirp Number
         **********************************************************************/
        *errCode = CBUFF_configEDMA (ptrSession,
                                     ptrDriverMCB->hwAttrs->cpMultipleChirpNonInterleavedAddr[index - 1U],
                                     (CP_SIZE_CBUFF_UNITS * SOC_ADCBUF_NUM_RX_CHANNEL),
                                     false);
        DebugP_assert(*errCode == CBUFF_STATUS_SUCCESS);
    }

    /************************************************************
     * ADC Data
     ************************************************************/
    for (index = 0U; index < SOC_ADCBUF_NUM_RX_CHANNEL; index++)
    {
        /* Is the channel active? */
        if (ptrSession->rxChannelAddress[index] != 0U)
        {
            /************************************************************
             * ADC Data:
             ************************************************************/
            linkListParams.transferSize = ptrSession->adcTransferSize;
            linkListParams.crcEnable    = ptrDriverMCB->initCfg.crcEnable;
            CBUFF_addLinkedList (ptrSession, CBUFF_LLType_ADCDATA, &linkListParams);

            /* Done with the channel */
            numChannelsProcessed++;

            /* YES: Is this the last channel? */
            if (numChannelsProcessed == ptrSession->numActiveADCChannels)
            {
                /* YES: Set the flag to indicate so. */
                isLast = true;
            }
            else
            {
                /* NO: Reset the flag */
                isLast = false;
            }

            /**********************************************************************
             * Transfer 10, 11, 12, 13
             * - Source Address: ADC Data
             * - Transfer Size : Size of the ADC Data
             **********************************************************************/
            *errCode = CBUFF_configEDMA (ptrSession,
                                         ptrSession->rxChannelAddress[index],
                                         ptrSession->adcTransferSize,
                                         isLast);
            DebugP_assert(*errCode == CBUFF_STATUS_SUCCESS);
        }
    }

    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to setup the data transfers to handle the following
 *      use case:-
 *          - Multiple Chirp Mode
 *          - Non Interleaved
 *          - ADC_CP
 *
 *  @param[in]  ptrDriverMCB
 *      Pointer to the CBUFF Driver MCB
 *  @param[in]  ptrSession
 *      Pointer to the session
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup CBUFF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -  <0
 */
int32_t CBUFF_setupNonInterleaved_MultiChirp_ADC_CP
(
    CBUFF_Object*       ptrDriverMCB,
    CBUFF_Session*      ptrSession,
    int32_t*            errCode
)
{
    CBUFF_LinkListParams    linkListParams;
    int32_t                 retVal = 0;
    uint8_t                 index;
    bool                    isLast;

    /* Initialize the error code: */
    *errCode = 0;

    /* Sanity Check: This is only to be invoked in Multiple Chirp Mode */
    DebugP_assert (ptrSession->sessionCfg.u.hwCfg.chirpMode > 1U);

    /* Initialize the linked list parameters */
    ptrDriverMCB->interfaceFxn->initLinkListParamsFxn (ptrSession, &linkListParams);

    /* Setup the [Optional] Header: */
    retVal = CBUFF_setupHeader (ptrSession, &linkListParams, errCode);
    DebugP_assert(retVal == CBUFF_STATUS_SUCCESS);

    /************************************************************
     * ADC Data
     ************************************************************/
    for (index = 0U; index < SOC_ADCBUF_NUM_RX_CHANNEL; index++)
    {
        /* Is the channel active? */
        if (ptrSession->rxChannelAddress[index] != 0U)
        {
            /************************************************************
             * ADC Data:
             ************************************************************/
            linkListParams.transferSize = ptrSession->adcTransferSize;
            linkListParams.crcEnable    = ptrDriverMCB->initCfg.crcEnable;
            CBUFF_addLinkedList (ptrSession, CBUFF_LLType_ADCDATA, &linkListParams);

            /**********************************************************************
             * Transfer 2, 3, 4, 5:
             *  - Source Address: ADC Buffer
             *  - Transfer Size : ADC Buffer size
             **********************************************************************/
            *errCode = CBUFF_configEDMA (ptrSession,
                                         ptrSession->rxChannelAddress[index],
                                         ptrSession->adcTransferSize,
                                         false);
            DebugP_assert(*errCode == CBUFF_STATUS_SUCCESS);
        }
    }

    /************************************************************
     * Chirp Parameters for Multiple Chirps:
     ************************************************************/
    for (index = 1U; index <= ptrSession->sessionCfg.u.hwCfg.chirpMode; index++)
    {
        /************************************************************
         * Chirp Parameters:
         ************************************************************/
        linkListParams.crcEnable    = 0U;
        linkListParams.transferSize = CP_SIZE_CBUFF_UNITS * SOC_ADCBUF_NUM_RX_CHANNEL;
        CBUFF_addLinkedList (ptrSession, CBUFF_LLType_CP, &linkListParams);

        /* Is this the last Chirp? */
        if (index == ptrSession->sessionCfg.u.hwCfg.chirpMode)
        {
            /* YES: Set the flag to indicate so. */
            isLast = true;
        }
        else
        {
            /* NO: Reset the flag */
            isLast = false;
        }

        /**********************************************************************
         * Transfer 6, 7, 8, 9, 10, 11, 12, 13
         *  - Source Address: CPx
         *  - Transfer Size : Size of the Chirp Parameters
         * Where X is the Chirp Number
         **********************************************************************/
        *errCode = CBUFF_configEDMA (ptrSession,
                                     ptrDriverMCB->hwAttrs->cpMultipleChirpNonInterleavedAddr[index - 1U],
                                     (CP_SIZE_CBUFF_UNITS * SOC_ADCBUF_NUM_RX_CHANNEL),
                                     isLast);
        DebugP_assert(*errCode == CBUFF_STATUS_SUCCESS);
    }

    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to setup the data transfers to handle the following
 *      use case:-
 *          - Multiple Chirp Mode
 *          - Non Interleaved
 *          - CP_ADC_CQ
 *
 *  @param[in]  ptrDriverMCB
 *      Pointer to the CBUFF Driver MCB
 *  @param[in]  ptrSession
 *      Pointer to the session
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup CBUFF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -  <0
 */
int32_t CBUFF_setupNonInterleaved_MultiChirp_CP_ADC_CQ
(
    CBUFF_Object*       ptrDriverMCB,
    CBUFF_Session*      ptrSession,
    int32_t*            errCode
)
{
    CBUFF_LinkListParams    linkListParams;
    int32_t                 retVal = CBUFF_STATUS_SUCCESS;
    uint8_t                 index;

    /* Initialize the error code: */
    *errCode = 0;

    /* Sanity Check: This is only to be invoked in Multiple Chirp Mode */
    DebugP_assert (ptrSession->sessionCfg.u.hwCfg.chirpMode > 1U);

    /* Initialize the linked list parameters */
    ptrDriverMCB->interfaceFxn->initLinkListParamsFxn (ptrSession, &linkListParams);

    /* Setup the [Optional] Header: */
    retVal = CBUFF_setupHeader (ptrSession, &linkListParams, errCode);
    DebugP_assert(retVal == CBUFF_STATUS_SUCCESS);

    /************************************************************
     * Chirp Parameters for Multiple Chirps:
     ************************************************************/
    for (index = 1U; index <= ptrSession->sessionCfg.u.hwCfg.chirpMode; index++)
    {
        /************************************************************
         * Chirp Parameters:
         ************************************************************/
        linkListParams.crcEnable    = 0U;
        linkListParams.transferSize = CP_SIZE_CBUFF_UNITS * SOC_ADCBUF_NUM_RX_CHANNEL;
        CBUFF_addLinkedList (ptrSession, CBUFF_LLType_CP, &linkListParams);

        /**********************************************************************
         * Transfer 2, 3, 4, 5, 6, 7, 8 and 9:
         *  - Source Address: CPx
         *  - Transfer Size : Size of the Chirp Parameters
         * Where X is the Chirp Number
         **********************************************************************/
        *errCode = CBUFF_configEDMA (ptrSession,
                                     ptrDriverMCB->hwAttrs->cpMultipleChirpNonInterleavedAddr[index - 1U],
                                     (CP_SIZE_CBUFF_UNITS * SOC_ADCBUF_NUM_RX_CHANNEL),
                                     false);
        DebugP_assert(*errCode == CBUFF_STATUS_SUCCESS);
    }

    /************************************************************
     * ADC Data
     ************************************************************/
    for (index = 0U; index < SOC_ADCBUF_NUM_RX_CHANNEL; index++)
    {
        /* Is the channel active? */
        if (ptrSession->rxChannelAddress[index] != 0U)
        {
            /************************************************************
             * ADC Data:
             ************************************************************/
            linkListParams.transferSize = ptrSession->adcTransferSize;
            linkListParams.crcEnable    = ptrDriverMCB->initCfg.crcEnable;
            CBUFF_addLinkedList (ptrSession, CBUFF_LLType_ADCDATA, &linkListParams);

            /**********************************************************************
             * Transfer 10, 11, 12, 13
             * - Source Address: ADC Data
             * - Transfer Size : Size of the ADC Data
             **********************************************************************/
            *errCode = CBUFF_configEDMA (ptrSession,
                                         ptrSession->rxChannelAddress[index],
                                         ptrSession->adcTransferSize,
                                         false);
            DebugP_assert(*errCode == CBUFF_STATUS_SUCCESS);
        }
    }

    /************************************************************
     * Chirp Quality:
     ************************************************************/
    CBUFF_addCQLLEntry (ptrSession, &linkListParams);

    /**********************************************************************
     * Transfer 14, 15:
     *  - Source Address: Chirp Quality(CQ1), Chirp Quality(CQ2)
     *  - Transfer Size : Chirp Quality Size for CQ1, CQ2
     **********************************************************************/
    *errCode = CBUFF_configCQEDMA (ptrSession);
    DebugP_assert(*errCode == CBUFF_STATUS_SUCCESS);

    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to setup the data transfers to handle the following
 *      use case:-
 *          - Multiple Chirp Mode
 *          - Non Interleaved
 *          - CP_ADC_CQ
 *          - User Data
 *
 *  @param[in]  ptrDriverMCB
 *      Pointer to the CBUFF Driver MCB
 *  @param[in]  ptrSession
 *      Pointer to the session
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  \ingroup CBUFF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -  <0
 */

int32_t CBUFF_setupNonInterleaved_MultiChirp_CP_ADC_CQ_USER
(
    CBUFF_Object*       ptrDriverMCB,
    CBUFF_Session*      ptrSession,
    int32_t*            errCode
)
{
    CBUFF_LinkListParams    linkListParams;
    int32_t                 retVal = CBUFF_STATUS_SUCCESS;
    uint8_t                 index;

    /* Initialize the error code: */
    *errCode = 0;

    /* Sanity Check: This is only to be invoked in Multiple Chirp Mode */
    DebugP_assert (ptrSession->sessionCfg.u.hwCfg.chirpMode > 1U);

    /* Initialize the linked list parameters */
    ptrDriverMCB->interfaceFxn->initLinkListParamsFxn (ptrSession, &linkListParams);

    /* Setup the [Optional] Header: */
    retVal = CBUFF_setupHeader (ptrSession, &linkListParams, errCode);
    DebugP_assert(retVal == CBUFF_STATUS_SUCCESS);

    /************************************************************
     * Chirp Parameters for Multiple Chirps:
     ************************************************************/
    for (index = 1U; index <= ptrSession->sessionCfg.u.hwCfg.chirpMode; index++)
    {
        /************************************************************
         * Chirp Parameters:
         ************************************************************/
        linkListParams.crcEnable    = 0U;
        linkListParams.transferSize = CP_SIZE_CBUFF_UNITS * SOC_ADCBUF_NUM_RX_CHANNEL;
        CBUFF_addLinkedList (ptrSession, CBUFF_LLType_CP, &linkListParams);

        /**********************************************************************
         * Transfer 2, 3, 4, 5, 6, 7, 8 and 9:
         *  - Source Address: CPx
         *  - Transfer Size : Size of the Chirp Parameters
         * Where X is the Chirp Number
         **********************************************************************/
        *errCode = CBUFF_configEDMA (ptrSession,
                                     ptrDriverMCB->hwAttrs->cpMultipleChirpNonInterleavedAddr[index - 1],
                                     (CP_SIZE_CBUFF_UNITS * SOC_ADCBUF_NUM_RX_CHANNEL),
                                     false);
        DebugP_assert(*errCode == CBUFF_STATUS_SUCCESS);
    }

    /************************************************************
     * ADC Data
     ************************************************************/
    for (index = 0U; index < SOC_ADCBUF_NUM_RX_CHANNEL; index++)
    {
        /* Is the channel active? */
        if (ptrSession->rxChannelAddress[index] != 0U)
        {
            /************************************************************
             * ADC Data:
             ************************************************************/
            linkListParams.transferSize = ptrSession->adcTransferSize;
            linkListParams.crcEnable    = ptrDriverMCB->initCfg.crcEnable;
            CBUFF_addLinkedList (ptrSession, CBUFF_LLType_ADCDATA, &linkListParams);

            /**********************************************************************
             * Transfer 10, 11, 12, 13
             * - Source Address: ADC Data
             * - Transfer Size : Size of the ADC Data
             **********************************************************************/
            *errCode = CBUFF_configEDMA (ptrSession,
                                         ptrSession->rxChannelAddress[index],
                                         ptrSession->adcTransferSize,
                                         false);
            DebugP_assert(*errCode == CBUFF_STATUS_SUCCESS);
        }
    }

    /************************************************************
     * Chirp Quality:
     ************************************************************/
    CBUFF_addCQLLEntry (ptrSession, &linkListParams);

    /**********************************************************************
     * Transfer 14, 15:
     *  - Source Address: Chirp Quality(CQ1), Chirp Quality(CQ2)
     *  - Transfer Size : Chirp Quality Size for CQ1, CQ2
     **********************************************************************/
    *errCode = CBUFF_configCQEDMA (ptrSession);
    DebugP_assert(*errCode == CBUFF_STATUS_SUCCESS);DebugP_assert(*errCode == CBUFF_STATUS_SUCCESS);

    /************************************************************
     * User Data:
     ************************************************************/
    CBUFF_addUserBufferLLEntry (ptrSession, &linkListParams);

    /**********************************************************************
     * Transfer 16, 17, 18:
     *  - Source Address: User Buffer1/UserBuffer2/UserBuffer3
     *  - Transfer Size : Size of the UserBuffer1/UserBuffer2/UserBuffer3
     **********************************************************************/
    *errCode = CBUFF_configUserBufferEDMA (ptrSession);
    DebugP_assert(*errCode == CBUFF_STATUS_SUCCESS);

    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to setup the data transfers
 *
 *  @param[in]  ptrSession
 *      Pointer to the session
 *  @param[out] errCode
 *      Error code populated by the API
 *
 *  \ingroup CBUFF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t CBUFF_setupTransfer (CBUFF_Session* ptrSession, int32_t* errCode)
{
    uint32_t                packetFormatIndex;
    int32_t                 retVal = CBUFF_STATUS_SUCCESS;
    CBUFF_setupTransferFxn  ptrSetupTransferFxn;

    /* Sanity Check: Ensure that the backpointer to the driver is valid */
    DebugP_assert (ptrSession->ptrDriverMCB != NULL);

    /* Initialize the linked list in the session tracker: */
    CBUFF_initLinkedList (ptrSession);

    /* Is the session operating in hardware or software triggered execution mode? */
    if (ptrSession->sessionCfg.executionMode == CBUFF_SessionExecuteMode_HW)
    {
        /*****************************************************************************
         * Hardware Execution Mode:
         *****************************************************************************/
        for (packetFormatIndex = 0; packetFormatIndex < 1; packetFormatIndex++)
        {
            /* Index into the table using the data format: However before we do this lets make
             * sure that the data format does not exceed the MAX perimissible */
            if (ptrSession->sessionCfg.u.hwCfg.dataFormat >= CBUFF_DataFmt_MAX)
            {
                /* Error: Data format is invalid and exceeds the MAX permissible */
                *errCode = CBUFF_EINVAL;
                DebugP_assert(0);
            }

            /* Are we operating in single or multiple chirp mode? */
            if (ptrSession->sessionCfg.u.hwCfg.chirpMode <= 1U)
            {
                /*****************************************************************
                 * Single Chirp or Continuous Mode
                 * - Setup the transfer function
                 *****************************************************************/
                if (ptrSession->sessionCfg.u.hwCfg.dataMode == CBUFF_DataMode_INTERLEAVED)
                {
                    /* Interleaved mode: */
                    ptrSetupTransferFxn = gInterleavedTransferFxn[ptrSession->sessionCfg.u.hwCfg.dataFormat];
                }
                else
                {
                    /* Non-Interleaved mode: */
                    ptrSetupTransferFxn = gNonInterleavedTransferFxn[ptrSession->sessionCfg.u.hwCfg.dataFormat];
                }
            }
            else
            {
                /*****************************************************************
                 * Multiple Chirp Mode:
                 * - Only XWR16xx/XWR18xx/XWR68xx in Non-Interleaved mode supports this
                 * - Setup the transfer function
                 *****************************************************************/
                if (ptrSession->sessionCfg.u.hwCfg.dataMode == CBUFF_DataMode_INTERLEAVED)
                {
                    /* Error: Multiple Chirp is not supported in Interleaved Mode */
                    *errCode = CBUFF_ENOTSUP;
                    DebugP_assert(0);
                }

                /* Get the transfer function: */
                ptrSetupTransferFxn = gMultipleChirpNonInterleavedTransferFxn[ptrSession->sessionCfg.u.hwCfg.dataFormat];
            }

            /* Do we have a valid transfer function? Application could have customized the tables to
             * reduce the size of the driver libraries. */
            if (ptrSetupTransferFxn != NULL)
            {
                /* Invoke the transfer function: */
                retVal = (ptrSetupTransferFxn)(ptrSession->ptrDriverMCB, ptrSession, errCode);
                DebugP_assert(retVal == CBUFF_STATUS_SUCCESS);
            }
            else
            {
                /* Error: Application is trying to use a mode & data format which has been built
                 * out of the driver */
                *errCode = CBUFF_ENOTSUP;
                DebugP_assert(0);
            }
        }
    }
    else
    {
        /*****************************************************************************
         * Software Execution Mode:
         * - Only user transfers are supported
         *****************************************************************************/
        retVal = CBUFF_setup_USER (ptrSession->ptrDriverMCB, ptrSession, errCode);
        DebugP_assert(retVal == CBUFF_STATUS_SUCCESS);
    }

    /************************************************************************
     * Sanity Checking: Do we meet the minimum criteria? The first linked list
     * entry tracks the total transfer size for all
     ************************************************************************/
    if (ptrSession->linkedListTracker[0].totalTransferSize < CBUFF_MIN_TRANSFER_SIZE_CBUFF_UNITS)
    {
        /* Error: The transfer size is below and should be padded by the application */
        *errCode = CBUFF_ELIMIT;
        DebugP_assert(0);
    }

    /* Control comes here implies that the transfers were successfully configured. */
    retVal = 0;

    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to shutdown the data transfer which have been
 *      configured. This will involve the cleaning up on the Shadow Linked
 *      List and EDMA channels.
 *
 *  @param[in]  ptrSession
 *      Pointer to the session
 *  @param[out] errCode
 *      Error code populated by the API
 *
 *  \ingroup CBUFF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t CBUFF_shutdownTransfer (CBUFF_Session* ptrSession, int32_t* errCode)
{
    /* Initialize the error code: */
    *errCode = 0;

    /* Initialize the linked list tracker for the session back to the defaults. */
    CBUFF_initLinkedList (ptrSession);

    /* Close the EDMA Channels associated with the session */
    CBUFF_closeEDMA (ptrSession);

    /* Transfers have been shutdown */
    return 0;
}

