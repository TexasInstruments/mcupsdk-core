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

/* CBUFF Header API: */
static int32_t CBUFF_setupHeader (CBUFF_Session* ptrSession, CBUFF_LinkListParams* ptrLinkListParams, int32_t* errCode);

/* EDMA3 Exported API: */
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
        /* Software Triggered Execution Mode: */
        ptrLLParams->transferSize = ptrSession->sessionCfg.swCfg.userBufferInfo[index].size;

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
        /* Software Triggered Execution Mode: */
        ptrUserBuffer = &ptrSession->sessionCfg.swCfg.userBufferInfo[lastUserBufferIndex];

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

        /* Software Triggered Execution Mode: */
        ptrUserBuffer = &ptrSession->sessionCfg.swCfg.userBufferInfo[index];

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
         * Configure the EDMA3 Transfer:
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
    int32_t                 retVal = CBUFF_STATUS_SUCCESS;

    /* Sanity Check: Ensure that the backpointer to the driver is valid */
    DebugP_assert (ptrSession->ptrDriverMCB != NULL);

    /* Initialize the linked list in the session tracker: */
    CBUFF_initLinkedList (ptrSession);

    /*****************************************************************************
     * Software Execution Mode:
     * - Only user transfers are supported
     *****************************************************************************/
    retVal = CBUFF_setup_USER (ptrSession->ptrDriverMCB, ptrSession, errCode);
    DebugP_assert(retVal == CBUFF_STATUS_SUCCESS);

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

