/*
 *  Copyright (C) 2021-2023 Texas Instruments Incorporated
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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <networking/icss_emac/icss_emac.h>
#include "icss_emac_local.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/**Minimum supported size of Ethernet frame*/
#define ETHERNET_FRAME_SIZE_60          (60U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* Default ICSS_EMAC parameters structure */
const ICSS_EMAC_Params ICSS_EMAC_defaultParams =
{
    NULL,               /*  pruicssHandle           */
    NULL,               /*  pruicssIntcInitData     */
    NULL,               /*  fwStaticMMap            */
    NULL,               /*  fwDynamicMMap           */
    NULL,               /*  fwVlanFilterParams      */
    NULL,               /*  fwMulticastFilterParams */
    {
        {NULL, NULL},   /*  port0LinkCallBack       */
        {NULL, NULL},   /*  port1LinkCallBack       */
        {NULL, NULL},   /*  rxNRTCallBack           */
        {NULL, NULL},   /*  rxRTCallBack            */
        {NULL, NULL},   /*  txCallback              */
        {NULL, NULL},   /*  learningExCallBack      */
        {NULL, NULL},   /*  customTxCallBack        */
        {NULL, NULL},   /*  customRxCallBack        */
    },
    {NULL, NULL},       /*  ethphyHandle            */
    {0, 0, 0, 0, 0, 0}  /*  macId[6]                */
};

/* ========================================================================== */
/*                       Function Declarations                                */
/* ========================================================================== */

/** \brief  API to copy a packet from DDR to Tx Queue memory on L3 and synchronize
 *          with firmware
 *
 *  \param[in]  icssEmacHandle  Handle to ICSS_EMAC instance
 *  \param[in]  srcAddress      Base address of the buffer where the frame to be
 *                              transmitted resides
 *  \param[in]  portNumber      Port on which frame has to be transmitted.
 *                              Valid values are: 1 for PORT1, 2 for PORT2
 *  \param[in]  queuePriority   Queue number in which frame will be queued
 *                              for transmission
 *  \param[in] lengthOfPacket   Length of the frame in bytes
 *
 *  \return     Code from \ref ICSS_EMAC_ErrorCodes
 */
int32_t ICSS_EMAC_txPacketEnqueue(ICSS_EMAC_Handle  icssEmacHandle,
                                  const uint8_t     *srcAddress,
                                  uint8_t           portNumber,
                                  uint8_t           queuePriority,
                                  uint16_t          lengthOfPacket);

/**
 *  @b Description
 *  @n
 *       API to retrieve the information about the received frame which
 *       is then used to dequeue the frame from the host queues.
 *       This version fo the API also allows for pointer to the
 *       address of read buffer in L3 address space to be returned
 *       as part of .ICSS_EMAC_PktInfo.
 *       This is an enhancement to the ICSS_EMAC_RxPktInfo API.
 *
 *  @param[in]  icssEmacHandle handle to ICSS_EMAC Instance.
 *  @param[out]  pRxPktInfo    Return pointer of receive packet info which includes
                        port number where frame was received,  host queue where the received frame is queued
                        and address of read buffer in L3 address space.

 *  @retval     Length of packet or 0 if no packet found
 */
int32_t ICSS_EMAC_rxPktInfo2(ICSS_EMAC_Handle   icssEmacHandle,
                             ICSS_EMAC_PktInfo  *pRxPktInfo);

/* TODO: Review if this is the correct place */
static inline void ICSS_EMAC_pend(void *handle, int32_t interrupt_type);

/**
 *  \name ICSS_EMAC_updatePhyStatus
 *  @brief Update phy status for port number specified
 *
  *  @param  portNum port number
 *  @param  icssEmacHandle handle to ICSS_EMAC Instance.
 *
 *  @retval none
 *
 */
void ICSS_EMAC_updatePhyStatus(uint8_t portNum, ICSS_EMAC_Handle icssEmacHandle);

static inline void ICSS_EMAC_pollPkt(ICSS_EMAC_Handle icssEmacHandle);

void ICSS_EMAC_enableRxInterrupt(ICSS_EMAC_Handle icssEmacHandle);

void ICSS_EMAC_disableRxInterrupt(ICSS_EMAC_Handle icssEmacHandle);

/**
 *  \name ICSS_EMAC_clearIrq
 *  @brief Clears interrupt Function. There are 2 possible sequence for clearing interrupt.
 *         Failure to follow this sequence may result into race condition.
 *
 *         1. Level sensitive interrupt controllers: a) clear interrupt via SECR
 *
 *         2. Edge sensitive interrupt controllers: a) disable interrupt via EICR
 *                                                  b) clear interrupt via SECR
 *                                                  c) enable interrupt via EISR.
 *
 *  @param Handle: icss_emac handle
 *         intNum: interrupt number
 *
 *  @retval none
 *
 */
static inline void ICSS_EMAC_clearIrq(ICSS_EMAC_Handle icssEmacHandle, uint32_t intNum);

static inline void ICSS_EMAC_clearRxIrq(ICSS_EMAC_Handle icssEmacHandle);

static inline void ICSS_EMAC_clearTxIrq(ICSS_EMAC_Handle icssEmacHandle);

/* Local function for polling link status. This can be called from ISR context or poll control API */
static inline void ICSS_EMAC_pollLink(ICSS_EMAC_Handle icssEmacHandle, uint32_t pollSource);

static inline void ICSS_EMAC_memcpyLocal(void *dst, const void *src, size_t len);

static inline void ICSS_EMAC_portFlush(ICSS_EMAC_Handle icssEmacHandle, uint8_t portNumber);


/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

extern ICSS_EMAC_Config gIcssEmacConfig[];
extern uint32_t gIcssEmacConfigNum;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void ICSS_EMAC_init(void)
{
    ICSS_EMAC_Handle    handle;
    uint32_t            i;

    for (i = 0; i < gIcssEmacConfigNum; i++)
    {
        handle = &gIcssEmacConfig[i];
        /* Input parameter validation */
        if ((handle->object != NULL) && (handle->attrs != NULL))
        {
            /* Mark the object as available */
            ((ICSS_EMAC_Object *)handle->object)->isOpen = (bool)false;
        }
    }
}

void ICSS_EMAC_deinit(void)
{
    /* Do Nothing */
}

void ICSS_EMAC_Params_init(ICSS_EMAC_Params *params)
{
    /* Input parameter validation */
    if (params != NULL)
    {
        *params = ICSS_EMAC_defaultParams;
    }
}

/*TODO: Review this code*/
ICSS_EMAC_Handle ICSS_EMAC_open(uint32_t idx, const ICSS_EMAC_Params *params)
{
    int32_t                 status = SystemP_SUCCESS;
    uint8_t                 port = 0U;
    uintptr_t               key;
    ICSS_EMAC_Handle        icssEmacHandle = NULL;
    ICSS_EMAC_Object        *icssEmacObject = NULL;
    const ICSS_EMAC_Attrs   *icssEmacAttrs = NULL;

    if((idx >= gIcssEmacConfigNum) || (params == NULL))
    {
        status = SystemP_FAILURE;
    }

    if(status == SystemP_SUCCESS)
    {
        icssEmacHandle = (ICSS_EMAC_Handle)&(gIcssEmacConfig[idx]);
        /* Get the pointer to the object and attrs */
        icssEmacObject = (ICSS_EMAC_Object *)icssEmacHandle->object;
        icssEmacAttrs = (const ICSS_EMAC_Attrs *)icssEmacHandle->attrs;
    }
    if(SystemP_SUCCESS == status)
    {
        DebugP_assert(icssEmacObject != NULL);
        DebugP_assert(icssEmacAttrs != NULL);
        DebugP_assert(sizeof(ICSS_EMAC_InternalObject) >= sizeof(ICSS_EMAC_Object));
        DebugP_assert((icssEmacAttrs->emacMode == ICSS_EMAC_MODE_SWITCH) ||
                      (icssEmacAttrs->emacMode == ICSS_EMAC_MODE_MAC1) ||
                      (icssEmacAttrs->emacMode == ICSS_EMAC_MODE_MAC2) ||
                      (icssEmacAttrs->emacMode == (ICSS_EMAC_MODE_DUALMAC | ICSS_EMAC_MODE_MAC1)) ||
                      (icssEmacAttrs->emacMode == (ICSS_EMAC_MODE_DUALMAC | ICSS_EMAC_MODE_MAC2)));
        DebugP_assert(icssEmacAttrs->phyToMacInterfaceMode <= ICSS_EMAC_RGMII_MODE);
        DebugP_assert(icssEmacAttrs->halfDuplexEnable < 2);
        DebugP_assert(icssEmacAttrs->enableIntrPacing <= ICSS_EMAC_DISABLE_PACING);
        if(icssEmacAttrs->enableIntrPacing == ICSS_EMAC_ENABLE_PACING)
        {
            DebugP_assert(icssEmacAttrs->intrPacingMode <= ICSS_EMAC_INTR_PACING_MODE1);
        }
        DebugP_assert(icssEmacAttrs->learningEnable <= ICSS_EMAC_LEARNING_ENABLE);
        DebugP_assert((icssEmacAttrs->portMask == ICSS_EMAC_MODE_SWITCH) ||
                      (icssEmacAttrs->portMask == ICSS_EMAC_MODE_MAC1) ||
                      (icssEmacAttrs->portMask == ICSS_EMAC_MODE_MAC2));
        DebugP_assert(icssEmacAttrs->txInterruptEnable < 2);
        DebugP_assert(params->pruicssHandle != NULL);
        DebugP_assert(params->pruicssIntcInitData != NULL);
        DebugP_assert(params->fwStaticMMap != NULL);
        DebugP_assert(params->fwDynamicMMap != NULL);
        DebugP_assert(params->ethphyHandle[0] != NULL);
        if(icssEmacAttrs->emacMode == ICSS_EMAC_MODE_SWITCH)
        {
            DebugP_assert(params->ethphyHandle[1] != NULL);
        }

        /* Determine if handle with this index was already opened */
        key = HwiP_disable();

        if(icssEmacObject->isOpen == (bool)true)
        {
            status = SystemP_FAILURE;
        }
        else
        {
            /* Mark the handle as being used */
            icssEmacObject->isOpen = (bool)true;
        }

        HwiP_restore(key);
    }

    if(SystemP_SUCCESS == status)
    {
        icssEmacObject->pruicssHandle = params->pruicssHandle;

        memcpy((void *)(&(icssEmacObject->fwStaticMMap)), (void *)(params->fwStaticMMap), sizeof(ICSS_EMAC_FwStaticMmap));

        memcpy((void *)(&(icssEmacObject->fwDynamicMMap)), (void *)(params->fwDynamicMMap), sizeof(ICSS_EMAC_FwDynamicMmap));

        if(params->fwVlanFilterParams != NULL)
        {
            icssEmacObject->fwVlanFilterConfigProvided = (bool)true;
            memcpy((void *)(&(icssEmacObject->fwVlanFilterParams)), (void *)(params->fwVlanFilterParams), sizeof(ICSS_EMAC_FwVlanFilterParams));
        }
        else
        {
            icssEmacObject->fwVlanFilterConfigProvided = (bool)false;
        }

        if(params->fwMulticastFilterParams != NULL)
        {
            icssEmacObject->fwMulticastFilterConfigProvided = (bool)true;
            memcpy((void *)(&(icssEmacObject->fwMulticastFilterParams)), (void *)(params->fwMulticastFilterParams), sizeof(ICSS_EMAC_FwMulticastFilterParams));
        }
        else
        {
            icssEmacObject->fwMulticastFilterConfigProvided = (bool)false;
        }

        memcpy((void *)(&(icssEmacObject->callBackObject)), (void *)(&(params->callBackObject)), sizeof(ICSS_EMAC_CallBackObject));

        icssEmacObject->ethphyHandle[0] = params->ethphyHandle[0];
        icssEmacObject->ethphyHandle[1] = params->ethphyHandle[1];

        memcpy((void *)(&(icssEmacObject->macId[0])), (void *)(&(params->macId[0])), sizeof(params->macId));

        /*Detect ICSS Revision*/
        icssEmacObject->icssRevision = PRUICSS_getVersion(icssEmacObject->pruicssHandle);
    }

    if(SystemP_SUCCESS == status)
    {
        switch(icssEmacAttrs->emacMode)
        {
            case ICSS_EMAC_MODE_SWITCH:
                ICSS_EMAC_switchInit(icssEmacHandle, params->pruicssIntcInitData);
                ICSS_EMAC_mdioIntrEnableSwitch(ICSS_EMAC_PORT_1, icssEmacHandle);
                port = ICSS_EMAC_MODE_SWITCH;
                break;
            case ICSS_EMAC_MODE_MAC1 | ICSS_EMAC_MODE_DUALMAC:
                ICSS_EMAC_hostInit(icssEmacHandle, params->pruicssIntcInitData);
                ICSS_EMAC_macInit((uint8_t)ICSS_EMAC_PORT_1, icssEmacHandle);
                ICSS_EMAC_mdioIntrEnable(ICSS_EMAC_PORT_1, icssEmacHandle);
                port = ICSS_EMAC_PORT_1;
                break;
            case ICSS_EMAC_MODE_MAC1:
                ICSS_EMAC_hostInit(icssEmacHandle, params->pruicssIntcInitData);
                ICSS_EMAC_macInit((uint8_t)ICSS_EMAC_PORT_1, icssEmacHandle);
                ICSS_EMAC_mdioIntrEnable(ICSS_EMAC_PORT_1, icssEmacHandle);
                port = ICSS_EMAC_PORT_1;
                break;
            case ICSS_EMAC_MODE_MAC2 | ICSS_EMAC_MODE_DUALMAC:
                ICSS_EMAC_hostInit(icssEmacHandle, params->pruicssIntcInitData);
                ICSS_EMAC_macInit((uint8_t)ICSS_EMAC_PORT_2, icssEmacHandle);
                ICSS_EMAC_mdioIntrEnable(ICSS_EMAC_PORT_2, icssEmacHandle);
                port = ICSS_EMAC_PORT_2;
                break;
            case ICSS_EMAC_MODE_MAC2:
                ICSS_EMAC_hostInit(icssEmacHandle, params->pruicssIntcInitData);
                ICSS_EMAC_macInit((uint8_t)ICSS_EMAC_PORT_2, icssEmacHandle);
                ICSS_EMAC_mdioIntrEnable(ICSS_EMAC_PORT_2, icssEmacHandle);
                port = ICSS_EMAC_PORT_2;
                break;
            default:
                status = SystemP_FAILURE;
                break;
        }
    }

    if(SystemP_SUCCESS == status)
    {
        if(ICSS_EMAC_MODE_SWITCH == icssEmacAttrs->emacMode)
        {
            ICSS_EMAC_initLinkState(icssEmacHandle, 0U, port);
            ICSS_EMAC_initLinkState(icssEmacHandle, 1U, port);
        }
        else
        {
            ICSS_EMAC_initLinkState(icssEmacHandle, 0U, port);
        }

        status = ICSS_EMAC_osInit(icssEmacHandle);
    }

    if(SystemP_FAILURE == status)
    {
        icssEmacHandle = NULL;
    }

    return icssEmacHandle;
}

void ICSS_EMAC_close(ICSS_EMAC_Handle icssEmacHandle)
{
    int32_t emacMode;

    if(NULL != icssEmacHandle)
    {
        emacMode = ((ICSS_EMAC_Attrs *)(icssEmacHandle->attrs))->emacMode;
        switch(emacMode)
        {
            case ICSS_EMAC_MODE_SWITCH:
                ICSS_EMAC_mdioIntrDisableSwitch(ICSS_EMAC_PORT_1, icssEmacHandle);
                break;
            case ICSS_EMAC_MODE_MAC1 | ICSS_EMAC_MODE_DUALMAC:
                ICSS_EMAC_mdioIntrDisable(ICSS_EMAC_PORT_1, icssEmacHandle);
                break;
            case ICSS_EMAC_MODE_MAC1:
                ICSS_EMAC_mdioIntrDisable(ICSS_EMAC_PORT_1, icssEmacHandle);
                break;
            case ICSS_EMAC_MODE_MAC2 | ICSS_EMAC_MODE_DUALMAC:
                ICSS_EMAC_mdioIntrDisable(ICSS_EMAC_PORT_2, icssEmacHandle);
                break;
            case ICSS_EMAC_MODE_MAC2:
                ICSS_EMAC_mdioIntrDisable(ICSS_EMAC_PORT_2, icssEmacHandle);
                break;
            default:
                break;
        }
        ICSS_EMAC_osDeinit(icssEmacHandle);

        ((ICSS_EMAC_Object *)(icssEmacHandle->object))->isOpen = (bool)false;
    }
    return;
}

int32_t ICSS_EMAC_ioctl(ICSS_EMAC_Handle icssEmacHandle,
                        uint32_t         ioctlCommand,
                        uint8_t          portNo,
                        void             *ioctlParams)
{
    uint8_t                 *pControl;
    int32_t                 retVal = SystemP_FAILURE;
    ICSS_EMAC_IoctlCmd      *ioctlCmd = (ICSS_EMAC_IoctlCmd *)ioctlParams;
    void                    *ioctlData = ioctlCmd->ioctlVal;
    uint32_t                temp_addr = 0U;
    PRUICSS_Handle          pruicssHandle = ((ICSS_EMAC_Object *)icssEmacHandle->object)->pruicssHandle;
    PRUICSS_HwAttrs const   *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs);
    ICSS_EMAC_FwStaticMmap  *pStaticMMap = (&((ICSS_EMAC_Object *)icssEmacHandle->object)->fwStaticMMap);

    switch (ioctlCommand)
    {
        case ICSS_EMAC_IOCTL_PROMISCUOUS_CTRL:
            /*TODO: Create macro for following if condition*/
            if(1U == *((uint32_t*)ioctlData))
            {
                retVal = ICSS_EMAC_promiscuousModeInit(portNo, icssEmacHandle);
            }
            else
            {
                retVal = ICSS_EMAC_promiscuousModeDeinit(portNo, icssEmacHandle);
            }
            break;

        case ICSS_EMAC_IOCTL_PORT_CTRL:
            if((uint8_t)ICSS_EMAC_PORT_1 == portNo)
            {
                temp_addr = (pruicssHwAttrs->pru0DramBase + pStaticMMap->portControlAddr);
                pControl = (uint8_t*)(temp_addr);
                retVal = SystemP_SUCCESS;
            }
            else if((uint8_t)ICSS_EMAC_PORT_2 == portNo)
            {
                temp_addr = (pruicssHwAttrs->pru1DramBase + pStaticMMap->portControlAddr);
                pControl = (uint8_t*)(temp_addr);
                retVal = SystemP_SUCCESS;
            }
            else
            {
                break;
            }

            *pControl = (*((uint8_t *)ioctlData));
            break;

        case ICSS_EMAC_IOCTL_LEARNING_CTRL:
            switch(ioctlCmd->command)
            {
                case ICSS_EMAC_LEARN_CTRL_UPDATE_TABLE:
                    retVal = ICSS_EMAC_updateHashTable(icssEmacHandle,
                                                       (uint8_t *)ioctlData,
                                                       portNo,
                                                       &(((ICSS_EMAC_Object *)icssEmacHandle->object)->macTable[0]),
                                                       (const ICSS_EMAC_CallBackConfig *)&((((ICSS_EMAC_Object *)icssEmacHandle->object)->callBackObject).learningExCallBack));
                    break;

                case ICSS_EMAC_LEARN_CTRL_CLR_TABLE:
                    if(portNo == (uint8_t)ICSS_EMAC_PORT_1)
                    {
                        retVal = ICSS_EMAC_purgeTable(portNo,
                                                      &(((ICSS_EMAC_Object *)icssEmacHandle->object)->macTable[portNo - 1]));
                    }
                    if(portNo == (uint8_t)ICSS_EMAC_PORT_2)
                    {
                        retVal = ICSS_EMAC_purgeTable(portNo,
                                                      &(((ICSS_EMAC_Object *)icssEmacHandle->object)->macTable[portNo - 1]));
                    }
                    break;

                case ICSS_EMAC_LEARN_CTRL_AGEING:
                    if(portNo == (uint8_t)ICSS_EMAC_PORT_1)
                    {
                        retVal = ICSS_EMAC_ageingRoutine(portNo,
                                                         &(((ICSS_EMAC_Object *)icssEmacHandle->object)->macTable[portNo - 1]));
                    }
                    if(portNo == (uint8_t)ICSS_EMAC_PORT_2)
                    {
                        retVal = ICSS_EMAC_ageingRoutine(portNo,
                                                         &(((ICSS_EMAC_Object *)icssEmacHandle->object)->macTable[portNo - 1]));
                    }
                    break;

                case ICSS_EMAC_LEARN_CTRL_FIND_MAC:
                    retVal = ((int8_t)(ICSS_EMAC_findMAC((uint8_t *)ioctlData,
                                                         &(((ICSS_EMAC_Object *)icssEmacHandle->object)->macTable[0]))));
                    break;

                case ICSS_EMAC_LEARN_CTRL_REMOVE_MAC:
                    retVal = ICSS_EMAC_removeMAC((uint8_t *)ioctlData,
                                                 &(((ICSS_EMAC_Object *)icssEmacHandle->object)->macTable[0]));
                    break;

                case ICSS_EMAC_LEARN_CTRL_INC_COUNTER:
                    retVal = ICSS_EMAC_incrementCounter(&(((ICSS_EMAC_Object *)icssEmacHandle->object)->macTable[0]));
                    break;

                case ICSS_EMAC_LEARN_CTRL_INIT_TABLE:
                    retVal = ICSS_EMAC_initLearningTable(&(((ICSS_EMAC_Object *)icssEmacHandle->object)->macTable[0]));
                    break;

                case ICSS_EMAC_LEARN_CTRL_SET_PORTSTATE:
                    retVal = ICSS_EMAC_changePortState((*((uint32_t *)ioctlData)),
                                              &(((ICSS_EMAC_Object *)icssEmacHandle->object)->macTable[0]));
                    break;

                default:
                    retVal = SystemP_FAILURE;
                    break;
            }
            break;

        case ICSS_EMAC_IOCTL_STORM_PREV_CTRL:
            retVal = ICSS_EMAC_validateFeatureSet(icssEmacHandle, portNo, ICSS_EMAC_FW_STORM_PREVENTIION_FEATURE_CTRL);
            if (retVal == SystemP_SUCCESS)
            {
                switch(ioctlCmd->command)
                {
                    case ICSS_EMAC_STORM_PREV_CTRL_ENABLE:
                        retVal = ICSS_EMAC_enableStormPrevention(portNo, icssEmacHandle, ICSS_EMAC_BC_STORM_PREVENTION);
                        if(retVal == SystemP_SUCCESS)
                        {
                            retVal = ICSS_EMAC_setCreditValue((*((uint16_t *)ioctlData)), (ICSS_EMAC_StormPrevention *)(&(((ICSS_EMAC_Object *)icssEmacHandle->object)->stormPrev)), ICSS_EMAC_BC_STORM_PREVENTION);
                        }
                        if(retVal == SystemP_SUCCESS)
                        {
                            retVal = ICSS_EMAC_enableStormPrevention(portNo, icssEmacHandle, ICSS_EMAC_MC_STORM_PREVENTION);
                        }
                        if(retVal == SystemP_SUCCESS)
                        {
                            retVal = ICSS_EMAC_setCreditValue((*((uint16_t *)ioctlData)), (ICSS_EMAC_StormPrevention *)(&(((ICSS_EMAC_Object *)icssEmacHandle->object)->stormPrev)), ICSS_EMAC_MC_STORM_PREVENTION);
                        }
                        if(retVal == SystemP_SUCCESS)
                        {
                            retVal = ICSS_EMAC_enableStormPrevention(portNo, icssEmacHandle, ICSS_EMAC_UC_STORM_PREVENTION);
                        }
                        if(retVal == SystemP_SUCCESS)
                        {
                            retVal = ICSS_EMAC_setCreditValue((*((uint16_t *)ioctlData)), (ICSS_EMAC_StormPrevention *)(&(((ICSS_EMAC_Object *)icssEmacHandle->object)->stormPrev)), ICSS_EMAC_UC_STORM_PREVENTION);
                        }
                        break;
                    case ICSS_EMAC_STORM_PREV_CTRL_DISABLE:
                        retVal = ICSS_EMAC_disableStormPrevention(portNo, icssEmacHandle, ICSS_EMAC_BC_STORM_PREVENTION);
                        if(retVal == SystemP_SUCCESS)
                        {
                            retVal = ICSS_EMAC_disableStormPrevention(portNo, icssEmacHandle, ICSS_EMAC_MC_STORM_PREVENTION);
                        }
                        if(retVal == SystemP_SUCCESS)
                        {
                            retVal = ICSS_EMAC_disableStormPrevention(portNo, icssEmacHandle, ICSS_EMAC_UC_STORM_PREVENTION);
                        }
                        break;
                    case ICSS_EMAC_STORM_PREV_CTRL_SET_CREDITS:
                        retVal = ICSS_EMAC_setCreditValue((*((uint16_t *)ioctlData)), (ICSS_EMAC_StormPrevention *)(&(((ICSS_EMAC_Object *)icssEmacHandle->object)->stormPrev)), ICSS_EMAC_BC_STORM_PREVENTION);
                        if(retVal == SystemP_SUCCESS)
                        {
                            retVal = ICSS_EMAC_setCreditValue((*((uint16_t *)ioctlData)), (ICSS_EMAC_StormPrevention *)(&(((ICSS_EMAC_Object *)icssEmacHandle->object)->stormPrev)), ICSS_EMAC_MC_STORM_PREVENTION);
                        }
                        if(retVal == SystemP_SUCCESS)
                        {
                            retVal = ICSS_EMAC_setCreditValue((*((uint16_t *)ioctlData)), (ICSS_EMAC_StormPrevention *)(&(((ICSS_EMAC_Object *)icssEmacHandle->object)->stormPrev)), ICSS_EMAC_UC_STORM_PREVENTION);
                        }
                        break;
                    case ICSS_EMAC_STORM_PREV_CTRL_INIT:
                        retVal = ICSS_EMAC_initStormPreventionTable(portNo, icssEmacHandle, ICSS_EMAC_BC_STORM_PREVENTION);
                        if(retVal == SystemP_SUCCESS)
                        {
                            retVal = ICSS_EMAC_initStormPreventionTable(portNo, icssEmacHandle, ICSS_EMAC_MC_STORM_PREVENTION);
                        }
                        if(retVal == SystemP_SUCCESS)
                        {
                            retVal = ICSS_EMAC_initStormPreventionTable(portNo, icssEmacHandle, ICSS_EMAC_UC_STORM_PREVENTION);
                        }
                        break;
                    case ICSS_EMAC_STORM_PREV_CTRL_RESET:
                        retVal = ICSS_EMAC_resetStormPreventionCounter(icssEmacHandle, ICSS_EMAC_BC_STORM_PREVENTION);
                        if(retVal == SystemP_SUCCESS)
                        {
                            retVal = ICSS_EMAC_resetStormPreventionCounter(icssEmacHandle, ICSS_EMAC_MC_STORM_PREVENTION);
                        }
                        if(retVal == SystemP_SUCCESS)
                        {
                            retVal = ICSS_EMAC_resetStormPreventionCounter(icssEmacHandle, ICSS_EMAC_UC_STORM_PREVENTION);
                        }
                        break;
                    case ICSS_EMAC_STORM_PREV_CTRL_ENABLE_BC:
                        retVal = ICSS_EMAC_enableStormPrevention(portNo, icssEmacHandle, ICSS_EMAC_BC_STORM_PREVENTION);
                        if(retVal == SystemP_SUCCESS)
                        {
                            retVal = ICSS_EMAC_setCreditValue((*((uint16_t *)ioctlData)), (ICSS_EMAC_StormPrevention *)(&(((ICSS_EMAC_Object *)icssEmacHandle->object)->stormPrev)), ICSS_EMAC_BC_STORM_PREVENTION);
                        }
                        break;
                    case ICSS_EMAC_STORM_PREV_CTRL_DISABLE_BC:
                        retVal = ICSS_EMAC_disableStormPrevention(portNo, icssEmacHandle, ICSS_EMAC_BC_STORM_PREVENTION);
                        break;
                    case ICSS_EMAC_STORM_PREV_CTRL_SET_CREDITS_BC:
                        retVal = ICSS_EMAC_setCreditValue((*((uint16_t *)ioctlData)), (ICSS_EMAC_StormPrevention *)(&(((ICSS_EMAC_Object *)icssEmacHandle->object)->stormPrev)), ICSS_EMAC_BC_STORM_PREVENTION);
                        break;
                    case ICSS_EMAC_STORM_PREV_CTRL_INIT_BC:
                        retVal = ICSS_EMAC_initStormPreventionTable(portNo, icssEmacHandle, ICSS_EMAC_BC_STORM_PREVENTION);
                        break;
                    case ICSS_EMAC_STORM_PREV_CTRL_RESET_BC:
                        retVal = ICSS_EMAC_resetStormPreventionCounter(icssEmacHandle, ICSS_EMAC_BC_STORM_PREVENTION);
                        break;
                    case ICSS_EMAC_STORM_PREV_CTRL_ENABLE_MC:
                        retVal = ICSS_EMAC_enableStormPrevention(portNo, icssEmacHandle, ICSS_EMAC_MC_STORM_PREVENTION);
                        if(retVal == SystemP_SUCCESS)
                        {
                            retVal = ICSS_EMAC_setCreditValue((*((uint16_t *)ioctlData)), (ICSS_EMAC_StormPrevention *)(&(((ICSS_EMAC_Object *)icssEmacHandle->object)->stormPrev)), ICSS_EMAC_MC_STORM_PREVENTION);
                        }
                        break;
                    case ICSS_EMAC_STORM_PREV_CTRL_DISABLE_MC:
                        retVal = ICSS_EMAC_disableStormPrevention(portNo, icssEmacHandle, ICSS_EMAC_MC_STORM_PREVENTION);
                        break;
                    case ICSS_EMAC_STORM_PREV_CTRL_SET_CREDITS_MC:
                        retVal = ICSS_EMAC_setCreditValue((*((uint16_t *)ioctlData)), (ICSS_EMAC_StormPrevention *)(&(((ICSS_EMAC_Object *)icssEmacHandle->object)->stormPrev)), ICSS_EMAC_MC_STORM_PREVENTION);
                        break;
                    case ICSS_EMAC_STORM_PREV_CTRL_INIT_MC:
                        retVal = ICSS_EMAC_initStormPreventionTable(portNo, icssEmacHandle, ICSS_EMAC_MC_STORM_PREVENTION);
                        break;
                    case ICSS_EMAC_STORM_PREV_CTRL_RESET_MC:
                        retVal = ICSS_EMAC_resetStormPreventionCounter(icssEmacHandle, ICSS_EMAC_MC_STORM_PREVENTION);
                        break;
                    case ICSS_EMAC_STORM_PREV_CTRL_ENABLE_UC:
                        retVal = ICSS_EMAC_enableStormPrevention(portNo, icssEmacHandle, ICSS_EMAC_UC_STORM_PREVENTION);
                        if(retVal == SystemP_SUCCESS)
                        {
                            retVal = ICSS_EMAC_setCreditValue((*((uint16_t *)ioctlData)), (ICSS_EMAC_StormPrevention *)(&(((ICSS_EMAC_Object *)icssEmacHandle->object)->stormPrev)), ICSS_EMAC_UC_STORM_PREVENTION);
                        }
                        break;
                    case ICSS_EMAC_STORM_PREV_CTRL_DISABLE_UC:
                        retVal = ICSS_EMAC_disableStormPrevention(portNo, icssEmacHandle, ICSS_EMAC_UC_STORM_PREVENTION);
                        break;
                    case ICSS_EMAC_STORM_PREV_CTRL_SET_CREDITS_UC:
                        retVal = ICSS_EMAC_setCreditValue((*((uint16_t *)ioctlData)), (ICSS_EMAC_StormPrevention *)(&(((ICSS_EMAC_Object *)icssEmacHandle->object)->stormPrev)), ICSS_EMAC_UC_STORM_PREVENTION);
                        break;
                    case ICSS_EMAC_STORM_PREV_CTRL_INIT_UC:
                        retVal = ICSS_EMAC_initStormPreventionTable(portNo, icssEmacHandle, ICSS_EMAC_UC_STORM_PREVENTION);
                        break;
                    case ICSS_EMAC_STORM_PREV_CTRL_RESET_UC:
                        retVal = ICSS_EMAC_resetStormPreventionCounter(icssEmacHandle, ICSS_EMAC_UC_STORM_PREVENTION);
                        break;
                    default:
                        retVal = SystemP_FAILURE;
                        break;
                }
            }
            break;

        case ICSS_EMAC_IOCTL_STATS_CTRL:
            switch(ioctlCmd->command)
            {
                case ICSS_EMAC_IOCTL_STAT_CTRL_GET:
                    retVal = ICSS_EMAC_readStats(icssEmacHandle, portNo, (ICSS_EMAC_PruStatistics *)ioctlData);
                    break;
                case ICSS_EMAC_IOCTL_STAT_CTRL_CLEAR:
                    retVal = ICSS_EMAC_purgeStats(icssEmacHandle, portNo);
                    break;
                default:
                    retVal = SystemP_FAILURE;
                    break;
            }
            break;

        case ICSS_EMAC_IOCTL_MULTICAST_FILTER_CTRL:

            if(((ICSS_EMAC_Object *)icssEmacHandle->object)->fwMulticastFilterConfigProvided == (bool)true)
            {
                retVal = ICSS_EMAC_validateFeatureSet(icssEmacHandle, portNo, ICSS_EMAC_FW_MULTICAST_FILTER_FEATURE_CTRL);
                if (retVal == SystemP_SUCCESS)
                {
                    if((uint8_t)ICSS_EMAC_PORT_1 == portNo)
                    {
                        temp_addr = (pruicssHwAttrs->pru0DramBase);
                        pControl = (uint8_t*)(temp_addr);
                        retVal = SystemP_SUCCESS;
                    }
                    else if((uint8_t)ICSS_EMAC_PORT_2 == portNo)
                    {
                        temp_addr = (pruicssHwAttrs->pru1DramBase);
                        pControl = (uint8_t*)(temp_addr);
                        retVal = SystemP_SUCCESS;
                    }
                    else
                    {
                        retVal = SystemP_FAILURE;
                        break;
                    }
                    retVal = ICSS_EMAC_multicastFilterConfig((&((ICSS_EMAC_Object *)icssEmacHandle->object)->fwMulticastFilterParams), (uintptr_t) pControl, ioctlCmd->command, ioctlCmd->ioctlVal);
                }
            }
            else
            {
                retVal = SystemP_FAILURE;
            }
            break;

        case ICSS_EMAC_IOCTL_VLAN_FILTER_CTRL:
            if(((ICSS_EMAC_Object *)icssEmacHandle->object)->fwVlanFilterConfigProvided == (bool)true)
            {
                retVal = ICSS_EMAC_validateFeatureSet(icssEmacHandle, portNo, ICSS_EMAC_FW_VLAN_FILTER_FEATURE_CTRL);
                if (retVal == SystemP_SUCCESS)
                {
                    if((uint8_t)ICSS_EMAC_PORT_1 == portNo)
                    {
                        temp_addr = (pruicssHwAttrs->pru0DramBase);
                        pControl = (uint8_t*)(temp_addr);
                        retVal = SystemP_SUCCESS;
                    }
                    else if((uint8_t)ICSS_EMAC_PORT_2 == portNo)
                    {
                        temp_addr = (pruicssHwAttrs->pru1DramBase);
                        pControl = (uint8_t*)(temp_addr);
                        retVal = SystemP_SUCCESS;
                    }
                    else
                    {
                        retVal = SystemP_FAILURE;
                        break;
                    }
                    retVal = ICSS_EMAC_vlanFilterConfig((&((ICSS_EMAC_Object *)icssEmacHandle->object)->fwVlanFilterParams), (uintptr_t) pControl, ioctlCmd->command, ioctlCmd->ioctlVal);
                }
            }
            else
            {
                retVal = SystemP_FAILURE;
            }
            break;

        case ICSS_EMAC_IOCTL_PORT_FLUSH_CTRL:
            ICSS_EMAC_portFlush(icssEmacHandle, portNo);
            retVal = SystemP_SUCCESS;
            break;

        default:
            retVal = SystemP_FAILURE;
            break;
    }
    return retVal;
}

int32_t ICSS_EMAC_rxPktGet(ICSS_EMAC_RxArgument *rxArg, void *userArg)
{
    uint16_t                    queue_rd_ptr;
    uint16_t                    queue_wr_ptr;
    uint16_t                    rd_buf_desc_num;
    uint32_t                    rd_buf_desc = 0;
    uint16_t                    rd_packet_length;
    uint32_t                    rd_buffer_l3_addr;
    uint16_t                    size = 0;
    uint16_t                    update_rd_ptr = 0;
    uint16_t                    rx_num_of_bytes = 0;
    uint16_t                    new_size = 0;
    ICSS_EMAC_QueueParams       *rxQueue;
    ICSS_EMAC_Queue             *qDesc;
    uint16_t                    shadow = 0;
    uint8_t                     *srcMacId;
    uint8_t                     *destMacId;
    uint16_t                    *typeProt;
    uint16_t                    typeProt1;
    uint16_t                    typeProt2;
    uint8_t                     emacMode = 0;
    uint32_t                    temp_addr = 0U;
    uint32_t                    temp_var1 = 0U;
    uint32_t                    temp_var2 = 0U;
    uint8_t                     ret_flag = 0U;
    int32_t                     ret_val = SystemP_SUCCESS;
    /*Set to 1 if it's a PTP frame*/
    uint8_t                     ptp_pkt = 0;
    /*Used to copy timestamp*/
    uint16_t                    aligned_length = 0;
    uint16_t                    true_rd_ptr = 0;
    ICSS_EMAC_Handle            icssEmacHandle = rxArg->icssEmacHandle;
    uint32_t                    destAddress = rxArg->destAddress;
    uint8_t                     queueNumber = rxArg->queueNumber;
    ICSS_EMAC_FwStaticMmap      *pStaticMMap = (&((ICSS_EMAC_Object *)icssEmacHandle->object)->fwStaticMMap);
    ICSS_EMAC_FwDynamicMmap     *pDynamicMMap = (&((ICSS_EMAC_Object *)icssEmacHandle->object)->fwDynamicMMap);
    uint32_t                    hostQDescOffset  = pDynamicMMap->hostQ1RxContextOffset + 64U;
    ICSS_EMAC_PortParams        *sPort;
    PRUICSS_Handle              pruicssHandle = ((ICSS_EMAC_Object *)icssEmacHandle->object)->pruicssHandle;
    PRUICSS_HwAttrs const       *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs);

    if(((((ICSS_EMAC_Object *)icssEmacHandle->object)->callBackObject).customRxCallBack).callBack != NULL)
    {
        /* Invoke  custom Rx packet callback function */
        ret_val = (((((ICSS_EMAC_Object *)icssEmacHandle->object)->callBackObject).customRxCallBack).callBack)(
                        (void *)icssEmacHandle,
                        (void *)rxArg,
                        ((ICSS_EMAC_Object *)icssEmacHandle->object)->callBackObject.customRxCallBack.userArg);
        return(ret_val);
    }

    if(((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->portMask == ICSS_EMAC_MODE_SWITCH)
    {
        emacMode = 0;
    }
    else
    {
        emacMode = 1U;
    }
    /*TODO: Move these declarations above if block*/
    ICSS_EMAC_HashTable         *macTablePtr;
    ICSS_EMAC_HostStatistics    *hostStatPtr;
    ICSS_EMAC_CallBackConfig    *learningExcCallback;

    if(emacMode == 0U)
    {   /*Switch Mode*/
        temp_addr = (pruicssHwAttrs->pru1DramBase + pStaticMMap->p0QueueDescOffset + (((uint32_t)(queueNumber))*8U));
        qDesc = (ICSS_EMAC_Queue *)(temp_addr);
    }
    else
    {
        temp_addr = (pruicssHwAttrs->sharedDramBase + hostQDescOffset + (((uint32_t)(queueNumber))*8U));
        qDesc = (ICSS_EMAC_Queue *)(temp_addr);
    }
    queue_wr_ptr = qDesc->wr_ptr;
    queue_rd_ptr = qDesc->rd_ptr;
    sPort = &(((ICSS_EMAC_Object *)icssEmacHandle->object)->switchPort[ICSS_EMAC_PORT_0]);
    if(qDesc->overflow_cnt > 0)
    {
        sPort->queue[queueNumber].qStat.errCount += qDesc->overflow_cnt;        /* increment missed packets to error counter */
        qDesc->overflow_cnt = 0;
        /* reset to zero as limited to 256 anyway */
    }
    temp_addr = (pruicssHwAttrs->sharedDramBase + ((uint32_t)queue_rd_ptr));
    rd_buf_desc = HW_RD_REG32(temp_addr);
    rxQueue = &(sPort->queue[queueNumber]);

    if(emacMode == 0U)
    { /*Switch Mode*/
        /* Determine the address of the first buffer descriptor from the rd_ptr */
        /*Check if Packet was received in collision queue or not */
        shadow = ((uint16_t)((rd_buf_desc & 0x00004000U) >> 14U));
        if(shadow != 0)
        {
        /* Pick the data from collision buffer's */
            rd_buffer_l3_addr = ((((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->l3OcmcBaseAddr) +  pDynamicMMap->p0ColBufferOffset);
        }
        else
        {
            rd_buf_desc_num = (queue_rd_ptr - rxQueue->buffer_desc_offset) >> 2;
            temp_var1 = (((uint32_t)(rd_buf_desc_num)) * 32U);
            temp_var2 = (rxQueue->buffer_offset);
            rd_buffer_l3_addr = ((((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->l3OcmcBaseAddr) + temp_var1 + temp_var2);
        }
    }
    else
    {
        rd_buf_desc_num = (queue_rd_ptr - rxQueue->buffer_desc_offset) >> 2;
        temp_var1 = ((uint32_t)(rd_buf_desc_num)) * 32U;
        temp_var2 = (rxQueue->buffer_offset);
        rd_buffer_l3_addr = ((((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->l3OcmcBaseAddr) + temp_var1 + temp_var2);
    }
    /* Take out the port number - it may have changed */
    rxArg->port = (0x00030000U & rd_buf_desc)>>16;

    temp_addr = (rd_buffer_l3_addr + 6U);
    srcMacId = (uint8_t*)(temp_addr);
    destMacId = (uint8_t*)rd_buffer_l3_addr;

    rd_packet_length = ((uint16_t)((0x1ffc0000U & rd_buf_desc) >> 18));

    size = (rd_packet_length >>2);
    if( (rd_packet_length & 0x00000003U) != 0U )
    {
        size = size + 1u;
    }

    /*Compute number of buffer desc required & update rd_ptr in queue */
    update_rd_ptr = ((rd_packet_length >> 5U)*4U) + queue_rd_ptr;
    if( (rd_packet_length & 0x0000001fU) != 0U) /* checks multiple of 32 else need to increment by 4 */
    {
        update_rd_ptr += 4U;
    }

    true_rd_ptr = update_rd_ptr;

    /*If timestamp bit is set then we need to account
    * for additional 32B used for Rx timestamp*/
    if(rd_buf_desc & 0x8000)
    {
        ptp_pkt = 1;
        update_rd_ptr += 4U;
    }

    /*Check for wrap around */
    if(update_rd_ptr >= rxQueue->queue_size)
    {
        update_rd_ptr = update_rd_ptr - (rxQueue->queue_size - rxQueue->buffer_desc_offset);

    }
    if(true_rd_ptr >= rxQueue->queue_size)
    {
        true_rd_ptr = true_rd_ptr - (rxQueue->queue_size - rxQueue->buffer_desc_offset);
    }
    if(rd_packet_length <= ICSS_EMAC_MAXMTU)        /* make sure we do not have too big packets */
    {
        /* Switch Mode */
        if(ICSS_EMAC_LEARNING_ENABLE == ((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->learningEnable)
        {
            learningExcCallback = &((((ICSS_EMAC_Object *)icssEmacHandle->object)->callBackObject).learningExCallBack);
            /*TODO: Review this*/
            macTablePtr = (ICSS_EMAC_HashTable *)(&(((ICSS_EMAC_Object *)icssEmacHandle->object)->macTable[0]));
            ICSS_EMAC_updateHashTable(icssEmacHandle, srcMacId, rxArg->port, macTablePtr, learningExcCallback);
        }

        /* Copy the data from switch buffers to DDR */
        if( (true_rd_ptr < queue_rd_ptr) && (true_rd_ptr != rxQueue->buffer_desc_offset))
        {
            typeProt = (uint16_t*)rd_buffer_l3_addr + 6;
            typeProt1 = ((uint16_t)((*typeProt) << 8U));
            typeProt2 = ((uint16_t)((*typeProt) >> 8U));
            typeProt1 = typeProt1 | typeProt2;
            rx_num_of_bytes = (rxQueue->queue_size - queue_rd_ptr);
            rx_num_of_bytes = (rx_num_of_bytes >> 2);
            rx_num_of_bytes = (uint16_t)((rx_num_of_bytes) << 5);

            ICSS_EMAC_memcpyLocal((int32_t*)destAddress, (int32_t*)rd_buffer_l3_addr, (size_t)rx_num_of_bytes);
            destAddress = destAddress + rx_num_of_bytes;
            new_size = rd_packet_length - rx_num_of_bytes;

            if(emacMode == 0U)
            { /*Switch Mode*/
                if(shadow != 0)
                {
                    rd_buffer_l3_addr = rd_buffer_l3_addr + rx_num_of_bytes;
                }
                else
                {
                    rd_buffer_l3_addr = ((((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->l3OcmcBaseAddr)  + rxQueue->buffer_offset);
                }
            }
            else
            {
                rd_buffer_l3_addr = ((((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->l3OcmcBaseAddr)  + rxQueue->buffer_offset);
            }
            ICSS_EMAC_memcpyLocal((int32_t*)destAddress, (int32_t*)rd_buffer_l3_addr, (size_t)new_size);
            /*Copy and append the timestamp to packet if it's a PTP frame*/
            if(ptp_pkt)
            {
                aligned_length = (new_size & 0xFFE0) + 32;
                ICSS_EMAC_memcpyLocal((int32_t*)(destAddress + new_size), (int32_t*)(rd_buffer_l3_addr + aligned_length), (size_t)10);
            }
        }
        else
        {
            ICSS_EMAC_memcpyLocal((int32_t*)destAddress, (int32_t*)rd_buffer_l3_addr, (size_t)rd_packet_length);
            /*Copy and append the timestamp to packet if it's a PTP frame*/
            if(ptp_pkt)
            {
                aligned_length = (rd_packet_length & 0xFFE0) + 32;
                ICSS_EMAC_memcpyLocal((int32_t*)(destAddress + rd_packet_length), (int32_t*)(rd_buffer_l3_addr + aligned_length), (size_t)10);
            }
            typeProt = (uint16_t*)destAddress + 6;
            typeProt1 = ((uint16_t)((*typeProt) << 8U));
            typeProt2 = ((uint16_t)((*typeProt) >> 8U));
            typeProt1 = typeProt1 | typeProt2;
        }
    }
    else  /* wrong packet size (exceeds ICSS_EMAC_MAXMTU)*/
    {
        rxQueue->qStat.errCount++;
        ret_flag = 1U;
        ret_val = SystemP_FAILURE;
    }

    if(ret_flag == 0U)
    {
        if(emacMode == 0U)
        { /*Switch Mode*/
            temp_addr = (pruicssHwAttrs->pru1DramBase  + pStaticMMap->p0QueueDescOffset + (((uint32_t)(queueNumber))*8U));
            /* Write back to queue */
            HW_WR_REG16(temp_addr, update_rd_ptr);
            /* Check if Host needs to change the wr_ptr for collision queue as well */
            if(shadow != 0)
            {
                temp_addr = (pruicssHwAttrs->pru1DramBase  + pStaticMMap->p0ColQueueDescOffset);
                ICSS_EMAC_Queue *qDescCol = (ICSS_EMAC_Queue *)(temp_addr);
                temp_addr = (pruicssHwAttrs->pru1DramBase  + pStaticMMap->p0ColQueueDescOffset +2U);
                /*Write back to collision queue desc */
                HW_WR_REG16(temp_addr, qDescCol->rd_ptr);

                temp_addr = (pruicssHwAttrs->pru1DramBase  +  pStaticMMap->colStatusAddr);
                HW_WR_REG8(temp_addr, 0);

                /*Indicate to firmware the Collision queue has been emptied*/
                temp_var1 = temp_addr + 3U;
                HW_WR_REG8(temp_var1, 0);
            }
        }
        else
        {
            temp_addr = (pruicssHwAttrs->sharedDramBase + hostQDescOffset + (((uint32_t)(queueNumber))*8U));
            HW_WR_REG16(temp_addr, update_rd_ptr);
        }
        rxQueue->qStat.rawCount++;

        rxArg->more = 0;
        if(emacMode == 0U)
        { /*Switch Mode*/
            temp_addr = (pruicssHwAttrs->pru1DramBase  + pStaticMMap->p0QueueDescOffset + (((uint32_t)(queueNumber))*8U));
            /* get new pointer data in case new packets received in meantime - experimental.. */
            qDesc = (ICSS_EMAC_Queue *)(temp_addr);
        }
        else
        {
            temp_addr = (pruicssHwAttrs->sharedDramBase + hostQDescOffset + (((uint32_t)(queueNumber))*8U));
            qDesc = (ICSS_EMAC_Queue *)(temp_addr);
        }
        queue_wr_ptr = qDesc->wr_ptr;

        if(update_rd_ptr != queue_wr_ptr)
        {
            rxArg->more = 1;
        }

        hostStatPtr = (ICSS_EMAC_HostStatistics *)(&(((ICSS_EMAC_Object *)icssEmacHandle->object)->hostStat));
        if(emacMode == 0U) /*In Switch Mode both the statistics structures are in single handle.Depending on port update the corresponding structure*/
        {
            hostStatPtr += (rxArg->port - 1);
        }
        ICSS_EMAC_updateRxStats(destMacId,(uint32_t)rd_packet_length, typeProt1, hostStatPtr);

        ret_val = (int32_t)rd_packet_length;
    }
    return (ret_val);
}

int32_t ICSS_EMAC_rxPktInfo(ICSS_EMAC_Handle icssEmacHandle,
                            int32_t          *portNumber,
                            int32_t          *queueNumber)
{
    int32_t             retVal;
    ICSS_EMAC_PktInfo   rxPktInfo;

    retVal = ICSS_EMAC_rxPktInfo2(icssEmacHandle, &rxPktInfo);

    (*queueNumber) = rxPktInfo.queueNumber;
    (*portNumber) = rxPktInfo.portNumber;
    return retVal;
}

int32_t ICSS_EMAC_txPacket(const ICSS_EMAC_TxArgument *txArg, void *userArg)
{
    int32_t             ret1 = SystemP_FAILURE;
    int32_t             ret2 = SystemP_FAILURE;
    const uint8_t       *macId;
    uint8_t             txPort;
    uint8_t             trigTx1;
    uint8_t             trigTx2;
    ICSS_EMAC_Handle    icssEmacHandle = txArg->icssEmacHandle;
    uint8_t             portNumber= txArg->portNumber;
    uint8_t             queuePriority = txArg->queuePriority;
    uint16_t            lengthOfPacket = txArg->lengthOfPacket;

    if(((((ICSS_EMAC_Object *)icssEmacHandle->object)->callBackObject).customTxCallBack).callBack != NULL)
    {
        /* Invoke  custom Tx packet callback function */
        ret1 = (((((ICSS_EMAC_Object *)icssEmacHandle->object)->callBackObject).customTxCallBack).callBack)(
                        (void *)icssEmacHandle, (void *)txArg,
                        ((ICSS_EMAC_Object *)icssEmacHandle->object)->callBackObject.customTxCallBack.userArg);
        return ret1;
    }
    macId = txArg->srcAddress;

    if (portNumber == ICSS_EMAC_PORT_0)    /* special case to use MAC learning */
    {
        /* get the Port number from MAC tables.. */
        /*TODO: Review this*/
        txPort = ICSS_EMAC_findMAC(macId, &(((ICSS_EMAC_Object *)(icssEmacHandle)->object)->macTable[0]));

        switch (txPort)
        {
            case 0U: /* need to send on both ports */
                trigTx1 = 1U;
                trigTx2 = 1U;
                break;
            case 1U: /* ICSS_EMAC_PORT_1 */
                trigTx1 = 1U;
                trigTx2 = 0U;
                break;
            case 2U: /* ICSS_EMAC_PORT_2 */
                trigTx1 = 0U;
                trigTx2 = 1U;
                break;
            default:    /* ICSS_EMAC_PORT_0 */
                trigTx1 = 1U;
                trigTx2 = 0U;
                break;
        }

        if(trigTx1 == 1U)
        {
            ret1 = ICSS_EMAC_txPacketEnqueue(icssEmacHandle,
                                            txArg->srcAddress,
                                            (uint8_t)ICSS_EMAC_PORT_1,
                                            queuePriority,
                                            lengthOfPacket);
        }

        if(trigTx2 == 1U)
        {
            ret2 = ICSS_EMAC_txPacketEnqueue(icssEmacHandle,
                                            txArg->srcAddress,
                                            (uint8_t)ICSS_EMAC_PORT_2,
                                            queuePriority,
                                            lengthOfPacket);
        }
    }
    else
    {
        ret1 = ICSS_EMAC_txPacketEnqueue(icssEmacHandle,
                                        txArg->srcAddress,
                                        portNumber,
                                        queuePriority,
                                        lengthOfPacket);
    }

    return ((ret2 << 16) | (ret1));
}

int32_t ICSS_EMAC_txPacketEnqueue(ICSS_EMAC_Handle  icssEmacHandle,
                                  const uint8_t     *srcAddress,
                                  uint8_t           portNumber,
                                  uint8_t           queuePriority,
                                  uint16_t          lengthOfPacket)
{
    uint32_t                    buffer_offset_computed = 0;
    uint32_t                    buffer_des = 0;
    uint16_t                    queue_wr_ptr = 0;
    uint16_t                    wrk_queue_wr_ptr = 0;
    uint16_t                    size = 0;
    uint16_t                    queue_rd_ptr = 0;
    uint16_t                    num_of_bytes = 0;
    uint16_t                    new_packet_length = 0;
    uint32_t                    temp = 0;
    uint16_t                    i = 0;
    uint32_t                    collision_queue_selected = 0;
    uint16_t                    collision_status = 0;
    uint16_t                    col_queue_already_occupied = 0;
    uint16_t                    original_length_of_packet = 0;
    uint16_t                    packet_min_size_padding = 0;
    uint16_t                    remaining_valid_frame_data_length = 0;
    uint8_t                     *macAddr;
    uint32_t                    pruSharedMem = 0;
    ICSS_EMAC_QueueParams       *txQueue;
    ICSS_EMAC_HostStatistics    *hostStatPtr;
    uint8_t                     linkStatus = 0;
    uint8_t                     emacMode = 0;
    uint16_t                    temp_var = 0;
    uint32_t                    temp_addr = 0U;
    ICSS_EMAC_FwDynamicMmap     *pDynamicMMap = (&((ICSS_EMAC_Object *)icssEmacHandle->object)->fwDynamicMMap);
    ICSS_EMAC_FwStaticMmap      *pStaticMMap = (&((ICSS_EMAC_Object *)icssEmacHandle->object)->fwStaticMMap);
    PRUICSS_Handle              pruicssHandle = ((ICSS_EMAC_Object *)icssEmacHandle->object)->pruicssHandle;
    PRUICSS_HwAttrs const       *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs);

    if(((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->portMask == ICSS_EMAC_MODE_SWITCH)
    {
        emacMode = 0;
    }
    else
    {
        emacMode = 1U;
    }

    if(emacMode)
    {   /*MAC Mode*/

        if(ICSS_EMAC_PORT_1 == portNumber)
        {
            pruSharedMem = pruicssHwAttrs->pru0DramBase;
        }
        if(ICSS_EMAC_PORT_2 == portNumber)
        {
            pruSharedMem = pruicssHwAttrs->pru1DramBase;
        }
        linkStatus = ((ICSS_EMAC_Object *)icssEmacHandle->object)->linkStatus[0];
        hostStatPtr = (ICSS_EMAC_HostStatistics *)(&(((ICSS_EMAC_Object *)icssEmacHandle->object)->hostStat));
    }
    else
    {
        linkStatus = ((ICSS_EMAC_Object *)icssEmacHandle->object)->linkStatus[portNumber - 1U];
        hostStatPtr = (ICSS_EMAC_HostStatistics *)(&(((ICSS_EMAC_Object *)icssEmacHandle->object)->hostStat));
        hostStatPtr += (portNumber - 1U);/*Switch mode. Points to correct structure depending on port*/
    }

    if((portNumber != ICSS_EMAC_PORT_1) && (portNumber != ICSS_EMAC_PORT_2))
    {
        hostStatPtr->txDroppedPackets++;
        return ((int32_t)ICSS_EMAC_SWITCH_INVALID_PORT);
    }
    if(queuePriority > ((uint8_t)(pDynamicMMap->numQueues) - (uint8_t)1U))
    {
        hostStatPtr->txDroppedPackets++;
        return ((int32_t)ICSS_EMAC_ERR_SWITCH_INVALID_PARAM);
    }
    if(lengthOfPacket > ICSS_EMAC_MAXMTU)
    {
        hostStatPtr->txDroppedPackets++;
        return ((int32_t)ICSS_EMAC_ERR_BADPACKET);
    }
    if(lengthOfPacket < ICSS_EMAC_MINMTU)
    {
        hostStatPtr->txDroppedPackets++;
        return ((int32_t)ICSS_EMAC_ERR_BADPACKET);
    }

    if(lengthOfPacket < ETHERNET_FRAME_SIZE_60)
    {
        original_length_of_packet = (uint16_t)lengthOfPacket;
        packet_min_size_padding =1U;
        lengthOfPacket = ETHERNET_FRAME_SIZE_60;
    }
    else
    {
        packet_min_size_padding = 0;
    }

    macAddr = (uint8_t*)srcAddress;

    if(linkStatus == 0U)
    {
        hostStatPtr->txDroppedPackets++;
        return ((int32_t)ICSS_EMAC_ERR_TX_NO_LINK);
    }

    ICSS_EMAC_PortParams *sPort;
    sPort = &(((ICSS_EMAC_Object *)icssEmacHandle->object)->switchPort[portNumber]);
    if(emacMode == 0U)
    {   /*Switch Mode*/
        txQueue = &(sPort->queue[queuePriority]);
        /* Check whether Queue is busy.If yes then put the packet in the collision Queue. Set the busy_s bit because Host is always a Slave. */
        temp_addr = (pruicssHwAttrs->pru1DramBase + txQueue->queue_desc_offset + 4U);
        temp = HW_RD_REG32(temp_addr);
        collision_queue_selected =  (temp & 0x00000100U);  /* Check the "busy_m" bit */
        if(collision_queue_selected != 0U)
        {
            hostStatPtr->txNumCollision = hostStatPtr->txNumCollision + 1;
            /* Queue is busy  .. put the packet in the collision Queue */
            txQueue = &(sPort->queue[ICSS_EMAC_COLQUEUE]);
            if(portNumber == ICSS_EMAC_PORT_1)
            {
                temp_addr = (pruicssHwAttrs->pru1DramBase + pStaticMMap->colStatusAddr+ 1U);
                col_queue_already_occupied = HW_RD_REG8(temp_addr);
            }
            else
            {
                temp_addr = (pruicssHwAttrs->pru1DramBase +  pStaticMMap->colStatusAddr + 2U);
                col_queue_already_occupied = HW_RD_REG8(temp_addr);
            }
            if(col_queue_already_occupied != 0)
            {
                hostStatPtr->txDroppedPackets++;
                hostStatPtr->txCollisionDroppedPackets++;
                return ((int32_t)ICSS_EMAC_ERR_COLLISION_FAIL);   /*No space in collision queue */
            }
        }
        else
        {
            temp_addr = (pruicssHwAttrs->pru1DramBase + txQueue->queue_desc_offset + 4U);
            /* Queue is Not Busy .. Acquire the Queue by setting "busy_s" bit */
            HW_WR_REG8(temp_addr, 1U); /* Set the "busy_s" bit */
            /*Again check if host acquired the queue successfully by checking the busy_m bit */
            temp_addr = (pruicssHwAttrs->pru1DramBase + txQueue->queue_desc_offset + 4U);
            temp = HW_RD_REG32(temp_addr);
            collision_queue_selected =  (temp & 0x00000100U);  /* Check the "busy_m" bit */
            if(collision_queue_selected != 0U)
            {
                hostStatPtr->txNumCollision = hostStatPtr->txNumCollision + 1;
                temp_addr = ( pruicssHwAttrs->pru1DramBase+ txQueue->queue_desc_offset + 4U);
                HW_WR_REG8(temp_addr, 0); /* Clear the busy_s bit */
                /* Queue is busy  .. put the packet in the collision Queue */
                txQueue = &(sPort->queue[ICSS_EMAC_COLQUEUE]);
            }
        }
    }
    else
    {   /*MAC Mode*/
        txQueue = &(sPort->queue[queuePriority]);
        /* Queue is Not Busy .. Acquire the Queue by setting "busy_s" bit */
        temp_addr = (pruSharedMem + txQueue->queue_desc_offset + 4U);
        HW_WR_REG8(temp_addr, 1U); /* Set the "busy_s" bit */
    }
    /*  Compute the buffer descriptor ..length is from bit 18 to 28 */
    buffer_des = (((uint32_t)(lengthOfPacket)) << 18U);

    if(emacMode == 0U)
    { /*Switch Mode*/
        temp_addr = (pruicssHwAttrs->pru1DramBase + txQueue->queue_desc_offset);
        /*  Read the write pointer from Queue descriptor */
        temp = HW_RD_REG32(temp_addr);
    }
    else
    {
        temp_addr = (pruSharedMem + txQueue->queue_desc_offset);
        temp = HW_RD_REG32(temp_addr);
    }

    queue_wr_ptr = ((uint16_t)(temp >> 16));
    queue_rd_ptr = ((uint16_t)(temp & 0x0000ffffU));

    wrk_queue_wr_ptr = (((uint16_t)(lengthOfPacket)) >> 5U);  /* Divide by 32 */
    wrk_queue_wr_ptr = (uint16_t)((wrk_queue_wr_ptr) << 2);  /* Multiply by 4 ..as one descriptor represents 32 bytes and BD takes 4 bytes */
    if((((uint32_t)(lengthOfPacket)) & 0x0000001fU) != 0U)
    {
         wrk_queue_wr_ptr = wrk_queue_wr_ptr + 4U;
    }

    /* Add to get the value of new queue write pointer */
    wrk_queue_wr_ptr = wrk_queue_wr_ptr + queue_wr_ptr;
    size = txQueue->queue_size;
    /*Check if queue is full and there is an wrap around */
    if(((queue_wr_ptr + 4U) % size) == 0U)
    {
        if(queue_rd_ptr == txQueue->buffer_desc_offset) /* Since queue is not starting from 0. */
        {
            txQueue->qStat.errCount++;
            if(emacMode == 0U)
            { /*Switch Mode*/
                temp_addr = (pruicssHwAttrs->pru1DramBase + txQueue->queue_desc_offset + 4U);
                HW_WR_REG8(temp_addr, 0);
            }
            else
            {
                temp_addr = (pruSharedMem + txQueue->queue_desc_offset + 4U);
                HW_WR_REG8(temp_addr, 0U);
            }
            hostStatPtr->txDroppedPackets++;
            return ((int32_t)ICSS_EMAC_ERR_TX_OUT_OF_BD);   /* No space in queue */
        }
    }
    /* Check if the Queue is already full */
    if((queue_wr_ptr + 4U) == queue_rd_ptr)
    {
        txQueue->qStat.errCount++;
        if(emacMode == 0U)
        {   /*Switch Mode*/
            temp_addr = (pruicssHwAttrs->pru1DramBase+ txQueue->queue_desc_offset + 4U);
            HW_WR_REG8(temp_addr, 0);
        }
        else
        {
            temp_addr = (pruSharedMem + txQueue->queue_desc_offset + 4U);
            HW_WR_REG8(temp_addr, 0U);
        }
        hostStatPtr->txDroppedPackets++;
        return ((int32_t)ICSS_EMAC_ERR_TX_OUT_OF_BD);       /* No space in queue */
    }
    /* Three cases arise between wr_ptr and rd_ptr */
    if(queue_wr_ptr == queue_rd_ptr)
    {
        /*Check for wrap around */
        if(wrk_queue_wr_ptr >=  size)
        {
            wrk_queue_wr_ptr = (wrk_queue_wr_ptr % size);
            /*Add offset as queue doesn't start from 0. */
            wrk_queue_wr_ptr = wrk_queue_wr_ptr + txQueue->buffer_desc_offset;
        }
    }
    else if(queue_wr_ptr > queue_rd_ptr)
    {
        /*Check for wrap around */
        if(wrk_queue_wr_ptr >=  size)
        {
            wrk_queue_wr_ptr = (wrk_queue_wr_ptr % size);
            wrk_queue_wr_ptr = wrk_queue_wr_ptr + txQueue->buffer_desc_offset;
            if(wrk_queue_wr_ptr >= queue_rd_ptr)
            {
                txQueue->qStat.errCount++;
                if(emacMode == 0U)
                {   /*Switch Mode*/
                    temp_addr = (pruicssHwAttrs->pru1DramBase + txQueue->queue_desc_offset + 4U);
                    HW_WR_REG8(temp_addr, 0);
                }
                else
                {
                    temp_addr = (pruSharedMem + txQueue->queue_desc_offset + 4U);
                    HW_WR_REG8(temp_addr, 0U);
                }
                hostStatPtr->txDroppedPackets++;
                return ((int32_t)ICSS_EMAC_ERR_TX_OUT_OF_BD);      /* No space in queue */
            }
        }
    }
    else
    {
        if(wrk_queue_wr_ptr >= queue_rd_ptr)
        {
            txQueue->qStat.errCount++;
            if(emacMode == 0U)
            {   /*Switch Mode*/
                temp_addr = (pruicssHwAttrs->pru1DramBase + txQueue->queue_desc_offset + 4U);
                HW_WR_REG8(temp_addr, 0);
            }
            else
            {
                temp_addr = (pruSharedMem + txQueue->queue_desc_offset + 4U);
                HW_WR_REG8(temp_addr, 0U);
            }
            hostStatPtr->txDroppedPackets++;
            return ((int32_t)ICSS_EMAC_ERR_TX_OUT_OF_BD);                          /* No space in queue */
        }
    }
    /* Compute the offset of buffer descriptor in ICSS shared RAM */
    temp_var = (txQueue->buffer_offset) + ((queue_wr_ptr - txQueue->buffer_desc_offset)*((uint16_t)8U));
    buffer_offset_computed = (uint32_t)temp_var;    /* queue_wr_ptr points to currently available free buffer */
    /*  Add the offset of Tx Queues */
    buffer_offset_computed = buffer_offset_computed + pDynamicMMap->transmitQueuesBufferOffset;

    /* Check if queue wrap around has happened. If yes then data can't be stored sequentially. */
    if( (wrk_queue_wr_ptr < queue_wr_ptr) &&  (wrk_queue_wr_ptr != txQueue->buffer_desc_offset))
    {
        num_of_bytes = (size - queue_wr_ptr);
        num_of_bytes *= 8U;                    /* divide by 4 * 32! */
        /* check if Padding has to be done. If yes then Pad with Zero's to reach the minimum size for the ethernet frame. */
        if(packet_min_size_padding == 1U)
        {
            if( num_of_bytes <= original_length_of_packet)
            {
                temp_addr = ((((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->l3OcmcBaseAddr) + ((uint32_t)buffer_offset_computed));
                ICSS_EMAC_memcpyLocal((int32_t*)(temp_addr), (int32_t*) srcAddress, (size_t)num_of_bytes);
            }
            else
            {
                temp_addr = ((((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->l3OcmcBaseAddr) + ((uint32_t)buffer_offset_computed));
                /* Copy the valid packet data first and then Pad with zero's.  */
                ICSS_EMAC_memcpyLocal((int32_t*)(temp_addr), (int32_t*) srcAddress, (size_t)original_length_of_packet);
                /* Pad the remaining bytes with Zero's */
                for(i=0; i< (num_of_bytes - original_length_of_packet); i++)
                {
                    temp_addr = ((((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->l3OcmcBaseAddr)+ ((uint32_t)buffer_offset_computed) + original_length_of_packet + i);
                    HW_WR_REG8(temp_addr, 0);
                }
            }
        }
        else
        {
            temp_addr = ((((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->l3OcmcBaseAddr) + ((uint32_t)buffer_offset_computed));
            ICSS_EMAC_memcpyLocal((int32_t*)(temp_addr), (int32_t*) srcAddress, (size_t)num_of_bytes);
        }
        new_packet_length = ((uint16_t)lengthOfPacket) - num_of_bytes;
        srcAddress = srcAddress + num_of_bytes;

        if(emacMode == 0U)
        { /*Switch Mode*/
            if(collision_queue_selected != 0)
            {
                buffer_offset_computed = buffer_offset_computed + num_of_bytes;
            }
            else
            {
                buffer_offset_computed = txQueue->buffer_offset;
                /*  Add the offset of Tx Queues */
                buffer_offset_computed = buffer_offset_computed + pDynamicMMap->transmitQueuesBufferOffset;
            }
        }
        else
        { /*MAC Mode*/
            buffer_offset_computed = txQueue->buffer_offset;
        }

        if( packet_min_size_padding == 1u)
        {
            if(original_length_of_packet <= num_of_bytes)
            {
                /* Pad all the remaining bytes with Zero's */
                for(i=0; i< new_packet_length; i++)
                {
                    temp_addr = ((((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->l3OcmcBaseAddr) + ((uint32_t)buffer_offset_computed) + i);
                    HW_WR_REG8(temp_addr, 0);
                }
            }
            else
            {
                /* Fill the frame data  */
                remaining_valid_frame_data_length = (original_length_of_packet- num_of_bytes);
                temp_addr = ((((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->l3OcmcBaseAddr) + ((uint32_t)buffer_offset_computed));
                ICSS_EMAC_memcpyLocal((int32_t*)(temp_addr),(int32_t*) srcAddress, (size_t)remaining_valid_frame_data_length);
                /*Pad the remaining bytes with Zero's */
                for(i=0; i< (new_packet_length - remaining_valid_frame_data_length); i++)
                {
                    temp_addr = ((((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->l3OcmcBaseAddr) + ((uint32_t)buffer_offset_computed) + remaining_valid_frame_data_length+ i);
                    HW_WR_REG8(temp_addr, 0);
                }
            }
        }
        else
        {
            temp_addr = ((((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->l3OcmcBaseAddr) + ((uint32_t)buffer_offset_computed));
            ICSS_EMAC_memcpyLocal((int32_t*)(temp_addr),(int32_t*) srcAddress, (size_t)new_packet_length);
        }
    }
    else
    {
        if( packet_min_size_padding  == 1u)
        {
            temp_addr = ((((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->l3OcmcBaseAddr) + ((uint32_t)buffer_offset_computed));
            ICSS_EMAC_memcpyLocal((int32_t*)(temp_addr), (int32_t*)srcAddress, (size_t)original_length_of_packet);
            /* Padd the remaining bytes with Zero's */
            for(i=0; i< (((uint16_t)lengthOfPacket) - original_length_of_packet); i++)
            {
                temp_addr = ((((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->l3OcmcBaseAddr) + ((uint32_t)buffer_offset_computed) + original_length_of_packet + i);
                HW_WR_REG8(temp_addr, 0);
            }
        }
        else
        {
            temp_addr = ((((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->l3OcmcBaseAddr) + ((uint32_t)buffer_offset_computed));
            ICSS_EMAC_memcpyLocal((int32_t*)(temp_addr), (int32_t*)srcAddress, (size_t)lengthOfPacket);
        }
    }
    if(emacMode == 0U)
    {   /*Switch Mode*/
        temp_addr = (pruicssHwAttrs->sharedDramBase + ((uint32_t)queue_wr_ptr) );
        HW_WR_REG32(temp_addr, buffer_des);
        temp_addr = (pruicssHwAttrs->pru1DramBase + txQueue->queue_desc_offset +2U);
        /* Write new wr_ptr in the queue descriptor */
        HW_WR_REG16(temp_addr, wrk_queue_wr_ptr);
    }
    else
    {
        temp_addr = (pruSharedMem + queue_wr_ptr );
        HW_WR_REG32(temp_addr, buffer_des);
        temp_addr = (pruSharedMem + txQueue->queue_desc_offset +2U);
        HW_WR_REG16(temp_addr, wrk_queue_wr_ptr);
    }

    txQueue->qStat.rawCount++;

    if(emacMode == 0U)
    {   /*Switch Mode*/
        temp_addr = (pruicssHwAttrs->pru1DramBase + txQueue->queue_desc_offset + 4U);
        /* Release the Queue. Clear the "busy_s" bit .. even if collision queue was selected then below line won't have any impact. */
        HW_WR_REG8(temp_addr, 0);
    }
    else
    {
        temp_addr = (pruSharedMem + txQueue->queue_desc_offset + 4U);
        HW_WR_REG8(temp_addr, 0U);
    }

    if(emacMode == 0U)
    {   /*Switch Mode*/
        /* If packet was put in collision queue then indiciate it to collision task */
        if(collision_queue_selected != 0)
        {
            if(portNumber ==ICSS_EMAC_PORT_1)
            {
                collision_status = ((uint16_t)queuePriority);
                collision_status = (uint16_t)((collision_status) << 1);
                collision_status = (collision_status | 0x0001U);
                temp_addr = (pruicssHwAttrs->pru1DramBase +  pStaticMMap->colStatusAddr + 1U);
                HW_WR_REG8(temp_addr, (uint8_t)collision_status);
            }
            else
            {
                collision_status = ((uint16_t)queuePriority);
                collision_status = (uint16_t)((collision_status) << 1);
                collision_status = (collision_status | 0x0001U);
                temp_addr = (pruicssHwAttrs->pru1DramBase +  pStaticMMap->colStatusAddr + 2U);
                HW_WR_REG8(temp_addr, (uint8_t)collision_status);
            }
        }
    }
    ICSS_EMAC_updateTxStats(macAddr,(uint32_t)lengthOfPacket, hostStatPtr);
    return ICSS_EMAC_SWITCH_SUCCESS;
}

void ICSS_EMAC_osRxTaskFnc(void *args)
{
    uint8_t             pacingEnabled;
    uint8_t             pacingMode;
    ICSS_EMAC_Handle    icssEmacHandle;

    icssEmacHandle = (ICSS_EMAC_Handle)args;
    pacingEnabled = ((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->enableIntrPacing;
    pacingMode = ((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->intrPacingMode;

    while(1)
    {
        ICSS_EMAC_pend(icssEmacHandle, ICSS_EMAC_INTR_SRC_RX);

        ICSS_EMAC_pollPkt(icssEmacHandle);
        if(pacingEnabled == ICSS_EMAC_ENABLE_PACING)
        {
            /*Enable interrupts*/
            if(pacingMode == ICSS_EMAC_INTR_PACING_MODE1)
            {
                ICSS_EMAC_enableRxInterrupt(icssEmacHandle);
            }
        }
    }
}

void ICSS_EMAC_osLinkTaskFnc(void *args)
{
    ICSS_EMAC_Handle icssEmacHandle;
    icssEmacHandle = (ICSS_EMAC_Handle)args;

    while(1)
    {
        ICSS_EMAC_pend(icssEmacHandle, ICSS_EMAC_INTR_SRC_LINK);

        if(ICSS_EMAC_MODE_SWITCH == ((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->portMask)
        {
            /*Update flags in memory*/
            ICSS_EMAC_updatePhyStatus(ICSS_EMAC_PORT_1, icssEmacHandle);
            /*Update flags in memory*/
            ICSS_EMAC_updatePhyStatus(ICSS_EMAC_PORT_2, icssEmacHandle);
        }
        else if(ICSS_EMAC_MODE_MAC1 == ((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->portMask)
        {
            ICSS_EMAC_updatePhyStatus(ICSS_EMAC_PORT_1, icssEmacHandle);
        }
        else if(ICSS_EMAC_MODE_MAC2 == ((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->portMask)
        {
            ICSS_EMAC_updatePhyStatus(ICSS_EMAC_PORT_2, icssEmacHandle);
        }
    }
}

void ICSS_EMAC_osTxTaskFnc(void *args)
{
    ICSS_EMAC_Handle icssEmacHandle;
    icssEmacHandle = (ICSS_EMAC_Handle)args;

    while(1)
    {
        ICSS_EMAC_pend((void *)icssEmacHandle, ICSS_EMAC_INTR_SRC_TX);

        // TODO: Review this
        // ((ICSS_EMAC_Object *)icssEmacHandle->object)->txCallback(icssEmacHandle, NULL);
        if(((((ICSS_EMAC_Object *)icssEmacHandle->object)->callBackObject).txCallBack).callBack != NULL)
        {
            ((((ICSS_EMAC_Object *)icssEmacHandle->object)->callBackObject).txCallBack).callBack(
                (void *)icssEmacHandle,
                NULL,
                ((((ICSS_EMAC_Object *)icssEmacHandle->object)->callBackObject).txCallBack).userArg);
        }

    }
}

void ICSS_EMAC_rxInterruptHandler(void *args)
{
    uint8_t             pacingEnabled;
    ICSS_EMAC_Handle    icssEmacHandle = (ICSS_EMAC_Handle)args;

    ICSS_EMAC_clearRxIrq((ICSS_EMAC_Handle)args);

    pacingEnabled = ((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->enableIntrPacing;

    /*disable Rx interrupt on ARM, PRU line stays high*/
    if(pacingEnabled == ICSS_EMAC_ENABLE_PACING)
    {
        ICSS_EMAC_disableRxInterrupt(icssEmacHandle);
    }
    SemaphoreP_post(&(((ICSS_EMAC_Object *)icssEmacHandle->object)->rxSemaphoreObject));
}

void ICSS_EMAC_txInterruptHandler(void *args)
{
    ICSS_EMAC_Handle        icssEmacHandle = (ICSS_EMAC_Handle)args;
    uint32_t                intStatus;
    PRUICSS_Handle          pruicssHandle = ((ICSS_EMAC_Object *)icssEmacHandle->object)->pruicssHandle;
    PRUICSS_HwAttrs const   *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs);

    intStatus = HW_RD_REG32(pruicssHwAttrs->intcRegBase + CSL_ICSS_PR1_ICSS_INTC_INTC_SLV_ENA_STATUS_REG0);
    if (((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->portMask == (uint8_t)ICSS_EMAC_MODE_MAC1)
    {
        if(TX_COMPLETION0_PRU_EVT_MASK & intStatus)
        {
            SemaphoreP_post(&(((ICSS_EMAC_Object *)icssEmacHandle->object)->txSemaphoreObject));
            ICSS_EMAC_clearTxIrq(icssEmacHandle);
        }
    }
    if (((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->portMask == (uint8_t)ICSS_EMAC_MODE_MAC2)
    {
        if(TX_COMPLETION1_PRU_EVT_MASK & intStatus)
        {
            SemaphoreP_post(&(((ICSS_EMAC_Object *)icssEmacHandle->object)->txSemaphoreObject));
            ICSS_EMAC_clearTxIrq(icssEmacHandle);
        }
    }
}

void ICSS_EMAC_linkISR(void* arg)
{
    ICSS_EMAC_Handle icssEmacHandle = (ICSS_EMAC_Handle)arg;
    ICSS_EMAC_pollLink(icssEmacHandle, ICSS_EMAC_POLL_FROM_ISR);
}

int32_t ICSS_EMAC_getPortLinkStatus(ICSS_EMAC_Handle    icssEmacHandle,
                                    uint8_t             portNum)
{
    int32_t retVal = SystemP_SUCCESS;

    if(((ICSS_EMAC_Object *)icssEmacHandle->object)->linkStatus[portNum] == 0)
    {
        retVal = SystemP_FAILURE;
    }

    return retVal;
}

int32_t ICSS_EMAC_rxPktInfo2(ICSS_EMAC_Handle   icssEmacHandle,
                             ICSS_EMAC_PktInfo  *pRxPktInfo)
{
    uint16_t                    queue_rd_ptr;
    uint16_t                    queue_wr_ptr;
    uint32_t                    rd_buf_desc = 0;
    uint16_t                    rd_packet_length;
    int32_t                     packet_found = 0;
    ICSS_EMAC_Queue             *qDesc;
    uint8_t                     emacMode = 0;
    uint32_t                    temp_addr = 0U;
    uint32_t                    temp_var1 = 0U;
    uint16_t                    shadow=0;
    uint32_t                    rd_buffer_l3_addr;
    uint16_t                    rd_buf_desc_num;
    ICSS_EMAC_QueueParams       *rxQueue;
    ICSS_EMAC_PortParams        *sPort;
    ICSS_EMAC_FwStaticMmap      *pStaticMMap = (&((ICSS_EMAC_Object *)icssEmacHandle->object)->fwStaticMMap);
    ICSS_EMAC_FwDynamicMmap     *pDynamicMMap = (&((ICSS_EMAC_Object *)icssEmacHandle->object)->fwDynamicMMap);
    PRUICSS_Handle              pruicssHandle = ((ICSS_EMAC_Object *)icssEmacHandle->object)->pruicssHandle;
    PRUICSS_HwAttrs const       *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs);
    uint8_t                     initPrioQueue = (uint8_t)ICSS_EMAC_QUEUE1;
    uint8_t                     finalPrioQueue = (uint8_t)(pDynamicMMap->numQueues-1U);
    uint8_t                     i = (uint8_t)ICSS_EMAC_QUEUE1;

    switch(((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->portMask)
    {
        case ICSS_EMAC_MODE_SWITCH:
            emacMode = 0;
            initPrioQueue = (uint8_t)ICSS_EMAC_QUEUE1;
            finalPrioQueue = (uint8_t)(pDynamicMMap->numQueues-1U);
            break;
        case ICSS_EMAC_MODE_MAC1:
            emacMode = 1u;
            initPrioQueue = (uint8_t)ICSS_EMAC_QUEUE1;
            finalPrioQueue = (uint8_t)ICSS_EMAC_QUEUE2;
            break;
        case ICSS_EMAC_MODE_MAC2:
            emacMode = 1u;
            initPrioQueue = (uint8_t)ICSS_EMAC_QUEUE3;
            finalPrioQueue = (uint8_t)ICSS_EMAC_QUEUE4;
            break;
        default:
            break;
    }

    i = initPrioQueue;

    while((packet_found == 0) && (i <= finalPrioQueue))
    {
        if(emacMode == 0U)
        {   /*Switch Mode*/
            temp_var1 = ((uint32_t)(i))*8U;
            temp_addr = (pruicssHwAttrs->pru1DramBase + pStaticMMap->p0QueueDescOffset + temp_var1);
            qDesc = (ICSS_EMAC_Queue *)(temp_addr);
        }
        else
        {
            temp_var1 = ((uint32_t)(i))*8U;
            temp_addr = (pruicssHwAttrs->sharedDramBase + pDynamicMMap->hostQ1RxContextOffset + 64U + temp_var1);
            qDesc = (ICSS_EMAC_Queue *)(temp_addr);
        }
        queue_wr_ptr = qDesc->wr_ptr;
        queue_rd_ptr = qDesc->rd_ptr;
        if(qDesc->overflow_cnt > 0)
        {
            sPort = &(((ICSS_EMAC_Object *)icssEmacHandle->object)->switchPort[ICSS_EMAC_PORT_0]);
            sPort->queue[i].qStat.errCount += qDesc->overflow_cnt;  /* increment missed packets to error counter */
            qDesc->overflow_cnt = 0;    /* reset to zero as limited to 256 anyway */
        }
        if(queue_rd_ptr != queue_wr_ptr)
        {
            pRxPktInfo->queueNumber = (int32_t)i;
            temp_addr = (pruicssHwAttrs->sharedDramBase + ((uint32_t)queue_rd_ptr));
            rd_buf_desc = HW_RD_REG32(temp_addr);
            sPort = &(((ICSS_EMAC_Object *)icssEmacHandle->object)->switchPort[ICSS_EMAC_PORT_0]);
             rxQueue = &(sPort->queue[i]);
            /* Take out the port number */
            temp_addr = ((0x00030000U & rd_buf_desc) >> 16U);
            pRxPktInfo->portNumber = (int32_t)(temp_addr);

            /* Get the length */
            rd_packet_length = ((uint16_t)((0x1ffc0000U & rd_buf_desc) >> 18U));
            packet_found = 1;
            /*
             * Determine the address of the first buffer descriptor
             * from the rd_ptr.  First, check if the packet was
             * received in the collision queue or not.
             */
            shadow = (uint16_t)(((rd_buf_desc & 0x00004000U) >> 14));
            if(shadow != 0)
            {
                /* The data is in the collision buffer's */
                rd_buffer_l3_addr = ((((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->l3OcmcBaseAddr) +  pDynamicMMap->p0ColBufferOffset);
            }
            else
            {
                rd_buf_desc_num = (queue_rd_ptr -
                                     rxQueue->buffer_desc_offset) >> 2;
                rd_buffer_l3_addr = (uint32_t)(((((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->l3OcmcBaseAddr)
                                    + (rd_buf_desc_num * 32U)
                                    + rxQueue->buffer_offset));
            }
            pRxPktInfo->rdBufferL3Addr = rd_buffer_l3_addr;
        }

        if(((ICSS_EMAC_Attrs *)(icssEmacHandle->attrs))->splitQueue)
        {
            if(i == finalPrioQueue)
            {
                break;
            }

            if(i < ((uint8_t)(pDynamicMMap->numQueues) / 2))
            {
                i = (i + (((uint8_t)(pDynamicMMap->numQueues) / 2)));
            }
            else
            {
                /*i > maxQueueNo/2*/
                i = ((i + 1) - (((uint8_t)(pDynamicMMap->numQueues) / 2)));
            }
        }
        else
        {
            i++;
        }
    }
    /* Received IRQ but can't find the packet in any queue */
    if(packet_found == 0)
    {
        rd_packet_length = 0;
    }
    return (int32_t)rd_packet_length;
}

static inline void ICSS_EMAC_pend(void *handle, int32_t interrupt_type)
{
    ICSS_EMAC_Handle icssEmacHandle = (ICSS_EMAC_Handle)handle;
    /* link = 0; rx = 1; tx = 2; tts = 3 */
    switch (interrupt_type)
    {
        case ICSS_EMAC_INTR_SRC_LINK:
            SemaphoreP_pend(&(((ICSS_EMAC_Object *)icssEmacHandle->object)->linkSemaphoreObject), SystemP_WAIT_FOREVER);
            break;
        case ICSS_EMAC_INTR_SRC_RX:
            SemaphoreP_pend(&(((ICSS_EMAC_Object *)icssEmacHandle->object)->rxSemaphoreObject), SystemP_WAIT_FOREVER);
            break;
        case ICSS_EMAC_INTR_SRC_TX:
            SemaphoreP_pend(&(((ICSS_EMAC_Object *)icssEmacHandle->object)->txSemaphoreObject), SystemP_WAIT_FOREVER);
            break;
        default:
            break;
    }
}

void ICSS_EMAC_updatePhyStatus(uint8_t portNum, ICSS_EMAC_Handle icssEmacHandle)
{
    volatile uint8_t            *portStatusPtr = NULL;
    volatile uint8_t            portStatus = 0;
    uint8_t                     linkStatus = 0;
    uint8_t                     prevlinkStatus = 0;
    volatile uint32_t           *phySpeedStatusPtr = NULL;
    uint8_t                     index = 0;
    uint8_t                     fullDuplex = 1U;
    uint32_t                    temp_addr = 0U;
    ICSS_EMAC_FwStaticMmap      *pStaticMMap = (&((ICSS_EMAC_Object *)icssEmacHandle->object)->fwStaticMMap);
    PRUICSS_Handle              pruicssHandle = ((ICSS_EMAC_Object *)icssEmacHandle->object)->pruicssHandle;
    PRUICSS_HwAttrs const       *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs);
    ETHPHY_SpeedDuplexConfig    speedDuplexConfig;
    int32_t                     status = SystemP_FAILURE;

    if(ICSS_EMAC_MODE_SWITCH == ((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->portMask)
    {
        index= portNum - 1U;
    }
    else
    {
        index = 0;
    }

    linkStatus = ((ICSS_EMAC_Object *)icssEmacHandle->object)->linkStatus[index];
    prevlinkStatus = ((ICSS_EMAC_Object *)icssEmacHandle->object)->prevlinkStatus[index];

    if(linkStatus ^ prevlinkStatus)
    {
        if(portNum == ((uint8_t)(ICSS_EMAC_PORT_1)))
        {
            temp_addr = (pruicssHwAttrs->pru0DramBase + pStaticMMap->phySpeedOffset);
            phySpeedStatusPtr = (uint32_t*)(temp_addr);
            temp_addr = (pruicssHwAttrs->pru0DramBase + pStaticMMap->portStatusOffset);
            portStatusPtr = (uint8_t*)(temp_addr);
        }

        if(portNum == ((uint8_t)(ICSS_EMAC_PORT_2)))
        {
            temp_addr = (pruicssHwAttrs->pru1DramBase + pStaticMMap->phySpeedOffset);
            phySpeedStatusPtr = (uint32_t*)(temp_addr);
            temp_addr = (pruicssHwAttrs->pru1DramBase + pStaticMMap->portStatusOffset);
            portStatusPtr = (uint8_t*)(temp_addr);
        }

        if(linkStatus)
        {
            status = ETHPHY_command(((ICSS_EMAC_Object *)icssEmacHandle->object)->ethphyHandle[index],
                                    ETHPHY_CMD_GET_SPEED_AND_DUPLEX_CONFIG,
                                    (void *)&speedDuplexConfig,
                                    sizeof(speedDuplexConfig));

            if(SystemP_SUCCESS != status)
            {
                speedDuplexConfig.config = ETHPHY_SPEED_DUPLEX_CONFIG_INVALID;
            }

            if(phySpeedStatusPtr != NULL)
            {
                switch(speedDuplexConfig.config)
                {
                    case  ETHPHY_SPEED_DUPLEX_CONFIG_100FD:
                        *(phySpeedStatusPtr) = (uint32_t)Hundred_Mbps;
                        fullDuplex = 1u;
                        break;
                    case  ETHPHY_SPEED_DUPLEX_CONFIG_100HD:
                        *(phySpeedStatusPtr) = (uint32_t)Hundred_Mbps;
                        fullDuplex = 0;
                        break;
                    case  ETHPHY_SPEED_DUPLEX_CONFIG_10FD:
                        *(phySpeedStatusPtr) = (uint32_t)Ten_Mbps;
                        fullDuplex = 1u;
                        break;
                    case  ETHPHY_SPEED_DUPLEX_CONFIG_10HD:
                        *(phySpeedStatusPtr) = (uint32_t)Ten_Mbps;
                        fullDuplex = 0;
                        break;
                    default:
                        /* Use 100M Full Duplex configuration by default */
                        *(phySpeedStatusPtr) = (uint32_t)Hundred_Mbps;
                        fullDuplex = 1u;
                        break;
                }
            }
            if(((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->halfDuplexEnable)
            {
                if(portStatusPtr != NULL)
                {
                    portStatus = *portStatusPtr;
                    /*set flags for HD*/
                    if(fullDuplex == 0U)
                    {
                        portStatus |= (uint8_t)ICSS_EMAC_PORT_IS_HD_MASK;
                    }
                    else
                    {
                        portStatus &= ((uint8_t)~(ICSS_EMAC_PORT_IS_HD_MASK));
                    }

                    /*write back*/
                    *(portStatusPtr) = portStatus;
                }
            }
        }
        ((ICSS_EMAC_Object *)icssEmacHandle->object)->prevlinkStatus[index] = linkStatus;
    }
}

static inline void ICSS_EMAC_pollPkt(ICSS_EMAC_Handle icssEmacHandle)
{
    int16_t                 pLength;
    int32_t                 allQueuesEempty = 0;
    int8_t                  dest_address[ICSS_EMAC_MAXMTU];
    uint16_t                numPacketsInLoop = 0;
    uint16_t                isNRT = 0;
    ICSS_EMAC_RxArgument    rxArg;
    ICSS_EMAC_PktInfo       rxPktInfo;

    while((allQueuesEempty != 1) && (numPacketsInLoop <= (((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->pacingThreshold)))
    {
        pLength = ((int16_t)(ICSS_EMAC_rxPktInfo2(icssEmacHandle, &rxPktInfo)));
        isNRT = 0;
        if(pLength > 0)
        {
            /* Check if split queue is enabled */
            if(((ICSS_EMAC_Attrs *)(icssEmacHandle->attrs))->splitQueue)
            {
                /*Check if queue is being used for port1 (Queue_No. > Max_Queue/2) */
                if(rxPktInfo.queueNumber >= (((((ICSS_EMAC_Object *)icssEmacHandle->object)->fwDynamicMMap).numQueues) / 2))
                {
                    /*If the queue is NRT*/
                    if((rxPktInfo.queueNumber - (((((ICSS_EMAC_Object *)icssEmacHandle->object)->fwDynamicMMap).numQueues) / 2)) >= ((ICSS_EMAC_Attrs *)(icssEmacHandle->attrs))->ethPrioQueue)
                    {
                        isNRT = 1;
                    }
                }
                else
                {
                    /*Queue is being used for port0, check if the queue is NRT*/
                    if(rxPktInfo.queueNumber >= ((ICSS_EMAC_Attrs *)(icssEmacHandle->attrs))->ethPrioQueue)
                    {
                        isNRT = 1;
                    }
                }
            }
            else
            {
                if(rxPktInfo.queueNumber >= ((ICSS_EMAC_Attrs *)(icssEmacHandle->attrs))->ethPrioQueue)
                    {
                        isNRT = 1;
                    }
            }

            if(isNRT && ((((ICSS_EMAC_Object *)icssEmacHandle->object)->callBackObject).rxNRTCallBack).callBack != NULL)
            {

                /* Invoke  receive packet callback function */
                (((((ICSS_EMAC_Object *)icssEmacHandle->object)->callBackObject).rxNRTCallBack).callBack)(
                    (void *)icssEmacHandle,
                    (void *)rxPktInfo.queueNumber,
                    ((ICSS_EMAC_Object *)icssEmacHandle->object)->callBackObject.rxNRTCallBack.userArg);
            }
            else
            {
                if(((((ICSS_EMAC_Object *)icssEmacHandle->object)->callBackObject).rxRTCallBack).callBack != NULL)
                {
                    ((((ICSS_EMAC_Object *)icssEmacHandle->object)->callBackObject).rxRTCallBack).callBack(
                        (void *)icssEmacHandle,
                        (void *)rxPktInfo.queueNumber,
                        ((((ICSS_EMAC_Object *)icssEmacHandle->object)->callBackObject).rxRTCallBack).userArg);
                }
                else
                {
                    /* TODO: Review this */
                    // if((((ICSS_EMAC_Object *)icssEmacHandle->object)->callBackObject != NULL)
                    //                 && ((((ICSS_EMAC_Object *)icssEmacHandle->object)->callBackObject)->rxCallBack != NULL)
                    //                 && (((((ICSS_EMAC_Object *)icssEmacHandle->object)->callBackObject)->rxCallBack)->callBack != NULL))
                    rxArg.icssEmacHandle = icssEmacHandle;
                    rxArg.destAddress = (uint32_t)dest_address;
                    rxArg.more = 0;
                    rxArg.queueNumber = rxPktInfo.queueNumber;
                    rxArg.port = rxPktInfo.portNumber;
                    ICSS_EMAC_rxPktGet(&rxArg, NULL);   /* just dump the packet here so we do no stall the queues */
                }
            }
            numPacketsInLoop++;
        }
        else
        {
            allQueuesEempty =1;
        }
    }
}

/*TODO: Review this function*/
void ICSS_EMAC_enableRxInterrupt(ICSS_EMAC_Handle icssEmacHandle)
{
    uint32_t intNum = ((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->rxIntNum;
    HwiP_enableInt((int32_t)intNum);
}

void ICSS_EMAC_disableRxInterrupt(ICSS_EMAC_Handle icssEmacHandle)
{
    uint32_t intNum = ((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->rxIntNum;
    HwiP_disableInt((int32_t)intNum);
}

/* TODO: Review this function */
static inline void ICSS_EMAC_clearIrq(ICSS_EMAC_Handle icssEmacHandle, uint32_t intNum)
{
    PRUICSS_Handle          pruicssHandle = ((ICSS_EMAC_Object *)icssEmacHandle->object)->pruicssHandle;
    PRUICSS_HwAttrs const   *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs);

    if(32 > intNum)
    {
        HW_WR_REG32((pruicssHwAttrs->intcRegBase + CSL_ICSS_PR1_ICSS_INTC_INTC_SLV_ENA_STATUS_REG0),
                    ((uint32_t)1U) << intNum);
    }
    else
    {
        HW_WR_REG32((pruicssHwAttrs->intcRegBase + CSL_ICSS_PR1_ICSS_INTC_INTC_SLV_ENA_STATUS_REG1),
                    ((uint32_t)1U) << (intNum-(uint32_t)32U));
    }
}

static inline void ICSS_EMAC_clearRxIrq(ICSS_EMAC_Handle icssEmacHandle)
{
    PRUICSS_Handle          pruicssHandle = ((ICSS_EMAC_Object *)icssEmacHandle->object)->pruicssHandle;
    PRUICSS_HwAttrs const   *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs);

    if(((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->portMask == ICSS_EMAC_MODE_MAC2)
    {
        ICSS_EMAC_clearIrq(icssEmacHandle, 21);
    }
    else
    {
        if(((ICSS_EMAC_Attrs *)(icssEmacHandle->attrs))->splitQueue)
        {
            uint32_t intStatus = HW_RD_REG32((pruicssHwAttrs->intcRegBase + CSL_ICSS_PR1_ICSS_INTC_INTC_SLV_ENA_STATUS_REG0));

            if (intStatus & (((uint32_t)1U) << 20))
                ICSS_EMAC_clearIrq(icssEmacHandle, 20);

            if (intStatus & (((uint32_t)1U) << 21))
                ICSS_EMAC_clearIrq(icssEmacHandle, 21);
        }
        else
        {
            ICSS_EMAC_clearIrq(icssEmacHandle, 20);
        }
    }
}

static inline void ICSS_EMAC_clearTxIrq(ICSS_EMAC_Handle icssEmacHandle)
{
    if(((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->portMask == ICSS_EMAC_MODE_MAC2)
    {
        ICSS_EMAC_clearIrq(icssEmacHandle, 23);
    }
    else
    {
        ICSS_EMAC_clearIrq(icssEmacHandle, 22);
    }
}

static inline void ICSS_EMAC_pollLink(ICSS_EMAC_Handle icssEmacHandle, uint32_t pollSource)
{
    int32_t                     retVal = SystemP_FAILURE;
    volatile uint8_t            *portStatusPtr = NULL;
    volatile uint8_t            portStatus = 0;
    volatile uint32_t           linkStatus;
    volatile uint32_t           *intStatusPtr;
    ICSS_EMAC_IoctlCmd          ioctlParams;
    uint8_t                     ioctlvalue = 0;
    ICSS_EMAC_HostStatistics    *hostStatPtr = (ICSS_EMAC_HostStatistics *)(&(((ICSS_EMAC_Object *)icssEmacHandle->object)->hostStat));
    ICSS_EMAC_FwStaticMmap      *pStaticMMap = (&((ICSS_EMAC_Object *)icssEmacHandle->object)->fwStaticMMap);
    uint32_t                    temp_addr = 0U;
    uint32_t                    temp_val;
    bool                        linkStatusChange = FALSE;
    PRUICSS_Handle              pruicssHandle = ((ICSS_EMAC_Object *)icssEmacHandle->object)->pruicssHandle;
    PRUICSS_HwAttrs const       *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs);

    temp_addr = (pruicssHwAttrs->intcRegBase + CSL_ICSS_PR1_ICSS_INTC_INTC_SLV_ENA_STATUS_REG1);
    /*Find out which port it is*/
    intStatusPtr = (uint32_t*)(temp_addr);
    if((((uint32_t)LINK0_PRU_EVT_MASK & *intStatusPtr) != 0U) && ((((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->portMask == (uint8_t)ICSS_EMAC_MODE_SWITCH) || (((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->portMask == (uint8_t)ICSS_EMAC_MODE_MAC1)))
    {
        /*TODO: Review this*/
        /**Link 0 Port event*/
        temp_addr = (pruicssHwAttrs->miiMdioRegBase + CSL_ICSS_PR1_MDIO_V1P7_MDIO_LINK_INT_MASKED_REG);
        /*clear interrupt in MDIO*/
        HW_WR_REG32(temp_addr, 0x01U);
        /* Issue read to make sure value written has taken affect, if not re-read once again*/
        temp_val = HW_RD_REG32(temp_addr);
        if ((temp_val & 0x1U) != 0U)
        {
            HW_RD_REG32(temp_addr);
        }

        /*clear PRU-ICSS INTC interrupt*/
        ICSS_EMAC_clearIrq(icssEmacHandle, 41);

        hostStatPtr->linkBreak++;

        linkStatus = 0;
        retVal = MDIO_phyLinkStatus(pruicssHwAttrs->miiMdioRegBase,
                                        ((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->phyAddr[0]);

        if(retVal == SystemP_SUCCESS)
            linkStatus = 1;

        ((ICSS_EMAC_Object *)icssEmacHandle->object)->linkStatus[0] = (uint8_t)linkStatus;

        temp_addr = (pruicssHwAttrs->pru0DramBase + pStaticMMap->portStatusOffset);
        portStatusPtr = (uint8_t*)(temp_addr);

        if(linkStatus)
        {
            portStatus |= (uint8_t)(ICSS_EMAC_PORT_LINK_MASK);
            ioctlvalue = (uint8_t)(ICSS_EMAC_IOCTL_PORT_CTRL_ENABLE);
            ioctlParams.ioctlVal = &ioctlvalue;
            ICSS_EMAC_ioctl(icssEmacHandle, ICSS_EMAC_IOCTL_PORT_CTRL, (uint8_t)ICSS_EMAC_PORT_1, (void*)&ioctlParams);
        }
        else
        {
            portStatus &= ((uint8_t)~(ICSS_EMAC_PORT_LINK_MASK));
            ioctlvalue = ICSS_EMAC_IOCTL_PORT_CTRL_DISABLE;
            ioctlParams.ioctlVal = &ioctlvalue;
            ICSS_EMAC_ioctl(icssEmacHandle, ICSS_EMAC_IOCTL_PORT_CTRL, (uint8_t)ICSS_EMAC_PORT_1, (void*)&ioctlParams);
        }

        if(portStatusPtr != NULL)
        {
            /*write back*/
            *(portStatusPtr) = portStatus;
        }
        /*Protocol specific processing*/
        /* TODO: Review this*/
        if(((((ICSS_EMAC_Object *)icssEmacHandle->object)->callBackObject).port0LinkCallBack).callBack != NULL)
        {
            ((((ICSS_EMAC_Object *)icssEmacHandle->object)->callBackObject).port0LinkCallBack).callBack(
                (void *)icssEmacHandle,
                (void *)linkStatus,
                ((((ICSS_EMAC_Object *)icssEmacHandle->object)->callBackObject).port0LinkCallBack).userArg);
        }
        linkStatusChange = TRUE;
    }

    if((((uint32_t)LINK1_PRU_EVT_MASK & *intStatusPtr) != 0U) && ((((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->portMask == (uint8_t)ICSS_EMAC_MODE_SWITCH) || (((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->portMask == (uint8_t)ICSS_EMAC_MODE_MAC2)))
    {
        /**Link 1 Port event*/

        /*TODO: Review this change*/
        /**Link 1 Port event*/
        temp_addr = (pruicssHwAttrs->miiMdioRegBase + CSL_ICSS_PR1_MDIO_V1P7_MDIO_LINK_INT_MASKED_REG);
        /*clear interrupt in MDIO*/
        HW_WR_REG32(temp_addr, 0x02U);
        /* Issue read to make sure value written has taken affect, if not re-read once again*/
        temp_val = HW_RD_REG32(temp_addr);
        if ((temp_val & 0x2U) != 0U)
        {
            HW_RD_REG32(temp_addr);
        }

        /*clear PRU-ICSS INTC interrupt*/
        ICSS_EMAC_clearIrq(icssEmacHandle, 53);

        hostStatPtr->linkBreak++;
        if(ICSS_EMAC_MODE_SWITCH != ((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->portMask)
        {
            linkStatus = 0;
            retVal = MDIO_phyLinkStatus(pruicssHwAttrs->miiMdioRegBase,
                                        ((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->phyAddr[0]);
            if(retVal == SystemP_SUCCESS)
                linkStatus = 1;
            ((ICSS_EMAC_Object *)icssEmacHandle->object)->linkStatus[0]=(uint8_t)linkStatus;
        }
        else
        {
            linkStatus = 0;
            retVal = MDIO_phyLinkStatus(pruicssHwAttrs->miiMdioRegBase,
                                        ((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->phyAddr[1]);
            if(retVal == SystemP_SUCCESS)
                linkStatus = 1;
            ((ICSS_EMAC_Object *)icssEmacHandle->object)->linkStatus[ICSS_EMAC_PORT_2-1U] = (uint8_t)linkStatus;
        }
        temp_addr = (pruicssHwAttrs->pru1DramBase + pStaticMMap->portStatusOffset);
        portStatusPtr = (uint8_t*)(temp_addr);

        if(linkStatus)
        {
            portStatus |= ICSS_EMAC_PORT_LINK_MASK;
            ioctlvalue = ICSS_EMAC_IOCTL_PORT_CTRL_ENABLE;
            ioctlParams.ioctlVal = &ioctlvalue;
            ICSS_EMAC_ioctl(icssEmacHandle, ICSS_EMAC_IOCTL_PORT_CTRL, (uint8_t)ICSS_EMAC_PORT_2, (void*)&ioctlParams);
        }
        else
        {
            portStatus &=  ((uint8_t)~((ICSS_EMAC_PORT_LINK_MASK)));
            ioctlvalue = ICSS_EMAC_IOCTL_PORT_CTRL_DISABLE;
            ioctlParams.ioctlVal = &ioctlvalue;
            ICSS_EMAC_ioctl(icssEmacHandle, ICSS_EMAC_IOCTL_PORT_CTRL, (uint8_t)ICSS_EMAC_PORT_2, (void*)&ioctlParams);
        }

        if(portStatusPtr != NULL)
        {
            /*write back*/
            *(portStatusPtr) = portStatus;
        }

        /* TODO: Review this*/
        if(((((ICSS_EMAC_Object *)icssEmacHandle->object)->callBackObject).port1LinkCallBack).callBack != NULL)
        {
            ((((ICSS_EMAC_Object *)icssEmacHandle->object)->callBackObject).port1LinkCallBack).callBack(
                (void *)icssEmacHandle,
                (void *)linkStatus,
                ((((ICSS_EMAC_Object *)icssEmacHandle->object)->callBackObject).port1LinkCallBack).userArg);
        }
        linkStatusChange = TRUE;
    }

    if(linkStatusChange == TRUE)
    {
        if (pollSource == ICSS_EMAC_POLL_FROM_ISR)
        {
            SemaphoreP_post(&(((ICSS_EMAC_Object *)icssEmacHandle->object)->linkSemaphoreObject));
        }
        else
        {
            if(ICSS_EMAC_MODE_SWITCH == ((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->portMask)
            {
                /*Update flags in memory*/
                ICSS_EMAC_updatePhyStatus(ICSS_EMAC_PORT_1, icssEmacHandle);
                ICSS_EMAC_updatePhyStatus(ICSS_EMAC_PORT_2, icssEmacHandle);
            }
            else if(ICSS_EMAC_MODE_MAC1 == ((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->portMask)
            {
                ICSS_EMAC_updatePhyStatus(ICSS_EMAC_PORT_1, icssEmacHandle);
            }
            else if(ICSS_EMAC_MODE_MAC2 == ((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->portMask)
            {
                ICSS_EMAC_updatePhyStatus(ICSS_EMAC_PORT_2, icssEmacHandle);
            }
        }
    }
}

static inline void ICSS_EMAC_memcpyLocal(void *dst, const void *src, size_t len)
{
    memcpy(dst, src, len);
}

static inline void ICSS_EMAC_portFlush(ICSS_EMAC_Handle icssEmacHandle, uint8_t portNumber)
{
    uint32_t qCount = 0U;
    ICSS_EMAC_PortParams *sPort;
    uint32_t bufferOffsets[ICSS_EMAC_NUMQUEUES];
    uint32_t bdOffsets[ICSS_EMAC_NUMQUEUES];
    ICSS_EMAC_FwDynamicMmap *pDynamicMMap = (&((ICSS_EMAC_Object*)icssEmacHandle->object)->fwDynamicMMap);
    ICSS_EMAC_FwStaticMmap *pStaticMMap = (&((ICSS_EMAC_Object*)icssEmacHandle->object)->fwStaticMMap);

    switch ( portNumber )
    {
        case (int32_t)ICSS_EMAC_PORT_1:
        ICSS_EMAC_calcPort1BufferOffset(icssEmacHandle, bufferOffsets,bdOffsets);
        sPort = &(((ICSS_EMAC_Object*)icssEmacHandle->object)->switchPort[ICSS_EMAC_PORT_1]);
        for (qCount = 0U; qCount < pDynamicMMap->numQueues; qCount++)
        {
            sPort->queue[qCount].buffer_offset      = bufferOffsets[qCount];
            sPort->queue[qCount].buffer_desc_offset = bdOffsets[qCount];
            sPort->queue[qCount].queue_desc_offset  = pStaticMMap->p0QueueDescOffset + 32U + (qCount * 8U);
            sPort->queue[qCount].queue_size         = (uint16_t)(((uint16_t)pDynamicMMap->txQueueSize[qCount]) << 2) + (uint16_t)bdOffsets[qCount];
        }

        sPort->queue[ICSS_EMAC_COLQUEUE].buffer_offset      = bufferOffsets[ICSS_EMAC_COLQUEUE];
        sPort->queue[ICSS_EMAC_COLQUEUE].buffer_desc_offset = bdOffsets[ICSS_EMAC_COLQUEUE];
        sPort->queue[ICSS_EMAC_COLQUEUE].queue_desc_offset  = pStaticMMap->p0ColQueueDescOffset + 8U;
        sPort->queue[ICSS_EMAC_COLQUEUE].queue_size         = (uint16_t)(((uint16_t)pDynamicMMap->collisionQueueSize) << 2) +  (uint16_t)bdOffsets[ICSS_EMAC_COLQUEUE];

        break;

        case (int32_t)ICSS_EMAC_PORT_2:
        sPort = &(((ICSS_EMAC_Object*)icssEmacHandle->object)->switchPort[ICSS_EMAC_PORT_2]);
        ICSS_EMAC_calcPort2BufferOffset(icssEmacHandle, bufferOffsets,bdOffsets);
        for (qCount = 0U; qCount < pDynamicMMap->numQueues; qCount++)
        {
            sPort->queue[qCount].buffer_offset      = bufferOffsets[qCount];
            sPort->queue[qCount].buffer_desc_offset = bdOffsets[qCount];
            sPort->queue[qCount].queue_desc_offset  = pStaticMMap->p0QueueDescOffset + 64U + (qCount * 8U);
            sPort->queue[qCount].queue_size         = (uint16_t)(((uint16_t)pDynamicMMap->txQueueSize[qCount]) << 2) + (uint16_t)bdOffsets[qCount];
        }

        sPort->queue[ICSS_EMAC_COLQUEUE].buffer_offset      = bufferOffsets[ICSS_EMAC_COLQUEUE];
        sPort->queue[ICSS_EMAC_COLQUEUE].buffer_desc_offset = bdOffsets[ICSS_EMAC_COLQUEUE];
        sPort->queue[ICSS_EMAC_COLQUEUE].queue_desc_offset  = pStaticMMap->p0ColQueueDescOffset + 16U;
        sPort->queue[ICSS_EMAC_COLQUEUE].queue_size         = (uint16_t)(((uint16_t)pDynamicMMap->collisionQueueSize) << 2U) +  (uint16_t)bdOffsets[ICSS_EMAC_COLQUEUE];

        break;

        default:
        break;
    }
}
