/*
 *  Copyright (C) 2021 Texas Instruments Incorporated
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
#include <string.h>
#include <stdlib.h>
#include "icss_dlr.h"
#include "icss_switch_dlr.h"
#include "icss_eip_driver.h"
#include <networking/icss_emac/icss_emac.h>
#include <drivers/mdio.h>
#include <drivers/hw_include/hw_types.h>

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/**Neighbor check request MAC ID for easy comparison*/
uint8_t DLR_NCREQ_MAC_ID[6] = {0x01, 0x21, 0x6C, 0x00, 0x00, 0x02};

/**A trace packet used to find out network loops*/
uint8_t tracePkt[DEFAULT_DLR_PACKET_SIZE] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00,
                                             0x45, 0x00, 0x00, 0x2E, 0x00, 0x00, 0x40, 0x00, 0x40, 0x00, 0x3A, 0xD1
                                            };

/**learning update frame skeleton*/
uint8_t learnUpdateFrame[DEFAULT_DLR_PACKET_SIZE] = {0x01, 0x21, 0x6c, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                                                     0x80, 0xE1, 0x02, 0x01, 0x0A, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0
                                                    };

#ifdef DLR_DEBUG
uint8_t sequenceOfEvents[MAX_EVENTS_CAPTURED] = {0};
uint32_t seqNumber = 0;
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
/*DLR Packet Generation Macros*/
#define addFrameOffset(src, offset) *(src + DLR_FRAME_TYPE_OFFSET) = offset;
#define addSourcePort(src, port) *(src + DLR_SOURCE_PORT_OFFSET) = port;
#define addReqSrcPort(src, port) *(src + DLR_REQ_SRC_PORT_OFFSET) = port;
#define addLinkStat(src, status) *(src + DLR_LINK_STATUS_OFFSET) = status;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
*  @internal
*  @brief   Update the new Supervisor details in the DLR handle
*
*  @param  dlrHandle    [in] DLR Handle
*
*  @retval  none
*
*/
void EIP_DLR_updateSupervisorDetails(EIP_DLRHandle dlrHandle);

/**
* @brief Add a word to the packet stream
* @param src pointer to the packet stream
* @param word the word which is to be added
* @return none
*/
void EIP_DLR_addWord(uint8_t *src, uint32_t word);

/**
* @brief Copy MAC ID to a stream
* @param src - pointer to stream
* @param macID pointer to MAC ID
*/
void EIP_DLR_addMACID(uint8_t *src, uint8_t *macID);

/**
* @brief Get the MAC Id from the network stream, convert it to little endian format and return it
* assumption : MAC is present in big endian byte format and starts from zero offset
* @param packet - byte stream
* @param macId pointer to array of six bytes which contain the MAC Id in little endian format
*/
void EIP_DLR_getMACId(uint8_t *packet, uint8_t *macId);

/**
* @brief Get a word from the network stream, convert it to little endian format and return it
* assumption word is present in big endian byte format and starts from zero offset
* @param byte pointer to byte stream
* @return word in little endian format
*/
uint32_t EIP_DLR_convBigEndianToLittleEndianWord(uint8_t *byte);

/**
* @brief get a halfword from the network stream, convert it to little endian format and return it
* assumption halfword is present in big endian byte format and starts from zero offset
* @param byte pointer to byte stream
* @return halfword halfword in little endian format
*/
uint16_t EIP_DLR_convBigEndianToLittleEndianHalfWord(uint8_t *byte);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/**
*  @brief  API to initialize the DLR driver
*
*          Initializes variables and timers & clocks, call once at the beginning
*
*  @param  dlrHandle [in] DLR driver handle
*
*  @retval  none
*
*/
void EIP_DLR_init(EIP_DLRHandle dlrHandle)
{

    /*Timer initialization variables*/
    ClockP_Params clockParams;
    uint8_t *ifMacID;
    uint32_t status = SystemP_FAILURE;

    /*Common frame header which is to be copied for every frame*/
    uint8_t dlrCommonFrameHeader[DLR_COMMON_FRAME_HEADER_SIZE] =
    {
        0x81, 0x00,                             /*802.1 Tag Type*/
        0xE0, 0x00,                             /*802.1 Tag Control*/
        0x80, 0xE1,                             /*Ring Ethertype*/
        0x02,                                   /*Ring Subtype*/
        0x01                                    /*Ring Protocol Ver*/
    };

    /*TODO: Review this*/
    ifMacID = (uint8_t *)(&(dlrHandle->macId[0]));

    /*Copy common DLR part*/
    EIP_DLR_initDLRFrameHeader(dlrHandle->dlrEmptyFrame, dlrCommonFrameHeader);

    /*Initialize the state machine variables on DRAM*/
    EIP_DLR_dRAMInit(dlrHandle);

    /*Initialize DLR Object parameters*/
    EIP_DLR_setDefaultValue(dlrHandle);

    EIP_DLR_set_pdi_wd_trigger_mode(dlrHandle, PDI_WD_TRIGGER_LATCH_IN);

    /*Initialize the watch dogs*/
    EIP_DLR_setDivider_WD_IEP(dlrHandle);

    /*TODO: Review this. Check if ClockP_init needs to be called*/
    /*Create clock for DLR periodic processing*/
    ClockP_Params_init(&clockParams);

    clockParams.start = 1;
    clockParams.timeout = DEFAULT_DLR_PERIODIC_INTERVAL;
    clockParams.period = DEFAULT_DLR_PERIODIC_INTERVAL;
    clockParams.callback = (ClockP_FxnCallback)EIP_DLR_periodicProcessing;
    clockParams.args = (void *)dlrHandle;

    status = ClockP_construct(&(dlrHandle->dlrPeriodicTimerObject), &clockParams);

    if(status == SystemP_FAILURE)
    {
        DebugP_log("\n\r DLR Periodic Clock create failed\n");
        /*TODO: Review if this is need or some alternate is better.*/
        exit(-1);
    }

    /*Setup clock for Neighbor timeout check on port 0*/
    ClockP_Params_init(&clockParams);

    clockParams.start = 0;
    clockParams.timeout = DEFAULT_NEIGHBOR_TIMEOUT_INTERVAL;
    clockParams.period = DEFAULT_NEIGHBOR_TIMEOUT_INTERVAL;
    clockParams.callback = (ClockP_FxnCallback)EIP_DLR_neighborTimeoutISR0;
    clockParams.args = (void *)dlrHandle;

    status = ClockP_construct(&(dlrHandle->dlrNeighborTimeoutClock[ICSS_EMAC_PORT_1 - 1]), &clockParams);

    if(status == SystemP_FAILURE)
    {
        DebugP_log("\n\r DLR Neighbor Timeout Clock create failed\n");
        exit(-1);
    }

    /*set up clock for Neighbor timeout check on port 1*/
    ClockP_Params_init(&clockParams);

    clockParams.start = 0;
    clockParams.timeout = DEFAULT_NEIGHBOR_TIMEOUT_INTERVAL;
    clockParams.period = DEFAULT_NEIGHBOR_TIMEOUT_INTERVAL;
    clockParams.callback = (ClockP_FxnCallback)EIP_DLR_neighborTimeoutISR1;
    clockParams.args = (void *)dlrHandle;

    status = ClockP_construct(&(dlrHandle->dlrNeighborTimeoutClock[ICSS_EMAC_PORT_2 - 1]), &clockParams);

    if(status == SystemP_FAILURE)
    {
        DebugP_log("\n\r DLR Neighbor Timeout Clock create failed\n");
        exit(-1);
    }

    if(EIP_DLR_isrInit(dlrHandle) != SystemP_SUCCESS)
    {
        DebugP_log("\n\r DLR ISR initialization failed\n");
        exit(-1);
    }

    /*Populate MAC ID*/
    EIP_DLR_addMACID(dlrHandle->dlrEmptyFrame + DLR_SRC_MAC_OFFSET, ifMacID);

    /*Populate source MAC ID*/
    EIP_DLR_addMACID(tracePkt + 6, ifMacID);
    EIP_DLR_addMACID(learnUpdateFrame + 6, ifMacID);

    /*Populate IP Address*/
    EIP_DLR_addWord(dlrHandle->dlrEmptyFrame + DLR_SOURCE_IP_OFFSET, dlrHandle->deviceIP);
    EIP_DLR_addWord(learnUpdateFrame + DLR_LEARN_UPDATE_IP_OFFSET, dlrHandle->deviceIP);
}

void EIP_DLR_deinit(EIP_DLRHandle dlrHandle)
{
    /*Delete clocks*/
    ClockP_destruct(&(dlrHandle->dlrPeriodicTimerObject));
    ClockP_destruct(&(dlrHandle->dlrNeighborTimeoutClock[ICSS_EMAC_PORT_1 - 1]));
    ClockP_destruct(&(dlrHandle->dlrNeighborTimeoutClock[ICSS_EMAC_PORT_2 - 1]));

    /*Delete Hwis*/
    HwiP_destruct(&(dlrHandle->dlrObj->port0IntObject));
    HwiP_destruct(&(dlrHandle->dlrObj->port1IntObject));
    HwiP_destruct(&(dlrHandle->dlrObj->beaconTimeoutIntP0Object));
    HwiP_destruct(&(dlrHandle->dlrObj->beaconTimeoutIntP1Object));
}

/**
*  @brief  API to start the DLR driver
*          Calling this enables DLR on the device
*
*  @param  dlrHandle [in] DLR driver handle
*
*  @retval  none
*
*/
void EIP_DLR_start(EIP_DLRHandle dlrHandle)
{

    volatile uint8_t *dlrEnabledBitPtr;
    PRUICSS_HwAttrs const *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(dlrHandle->pruicssHandle->hwAttrs);

    /*Enable DLR state machine on Firmware*/
    dlrEnabledBitPtr = (uint8_t *)(pruicssHwAttrs->sharedDramBase +
                                   DLR_ENABLED_OFFSET);
    *(dlrEnabledBitPtr) = DLR_TRUE;
}

/**
*  @brief  API to stop the DLR driver
*          Halt DLR. Calling this disables DLR on the device
*
*  @param  dlrHandle [in] DLR driver handle
*
*  @retval  none
*
*/
void EIP_DLR_stop(EIP_DLRHandle dlrHandle)
{
    volatile uint8_t *dlrEnabledBitPtr;
    PRUICSS_HwAttrs const *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(dlrHandle->pruicssHandle->hwAttrs);
    /*Enable DLR state machine on Firmware*/
    dlrEnabledBitPtr = (uint8_t *)(pruicssHwAttrs->sharedDramBase +
                                   DLR_ENABLED_OFFSET);
    *(dlrEnabledBitPtr) = DLR_FALSE;
}

/**
*  @brief  Fast ISR for Port 0, bypasses the buffer copy and NDK
*
*  @param  arg [in] user argument. DLR handle
*
*  @retval  none
*
*/
void EIP_DLR_port0ISR(uintptr_t arg)
{
    uint32_t retVal = SystemP_FAILURE;
    uint32_t linkStatus;
    volatile uint32_t *portEvtsPtr;
    volatile uint8_t *commonEvtsPtr;
    volatile uint32_t *otherPortPtr;
    volatile uint32_t *intStatusPtr;
    volatile uint32_t portEvtsFlag;
    volatile uint8_t commonEvtsFlag;
    volatile uint32_t otherPortFlag;
    ICSS_EMAC_TxArgument txArg;
    EIP_DLRHandle dlrHandle = (EIP_DLRHandle)arg;
    ICSS_EMAC_Handle dlricssEmacHandle = dlrHandle->emacHandle;
    PRUICSS_HwAttrs const *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(dlrHandle->pruicssHandle->hwAttrs);
    ICSS_EMAC_IoctlCmd ioctlParams;

    /*TODO: Review this*/
    intStatusPtr = (uint32_t *)(uint32_t)(pruicssHwAttrs->intcRegBase +
                                          CSL_ICSS_G_PR1_ICSS_INTC_INTC_SLV_ENA_STATUS_REG0);

    portEvtsPtr = (uint32_t *)(pruicssHwAttrs->pru0DramBase +
                               DLR_PORT_EVENTS_OFFSET);
    portEvtsFlag = *(portEvtsPtr);

    commonEvtsPtr = (uint8_t *)(pruicssHwAttrs->sharedDramBase +
                                DLR_STATE_MACHINE_OFFSET + 3);
    commonEvtsFlag = *(commonEvtsPtr);

    /*when a new supervisor comes online, timers are restarted*/
    if(commonEvtsFlag & DLR_STOP_BOTH_TIMERS_MASK)
    {

#ifdef DLR_DEBUG
        genSeqOfEvents(STOP_BOTH_TIMERS_PORT0);
#endif
        /*set the flag*/
        commonEvtsFlag |= (DLR_RESET_EVENT_OCCURED_MASK);
        *(commonEvtsPtr) = commonEvtsFlag;

        otherPortPtr = (uint32_t *)(pruicssHwAttrs->pru1DramBase +
                                    DLR_PORT_EVENTS_OFFSET);
        otherPortFlag = *(otherPortPtr);

        /*clear the flag*/
        commonEvtsFlag &= ~(DLR_STOP_BOTH_TIMERS_MASK);
        /*write back the flag*/
        *(commonEvtsPtr) = commonEvtsFlag;
        /*stop the timer on port1*/
        EIP_DLR_disable_WD_IEP(dlrHandle, PORT0_WATCH_DOG_ID);

        /*stop the timer on port1*/
        EIP_DLR_disable_WD_IEP(dlrHandle, PORT1_WATCH_DOG_ID);

        /*clear the flags*/
        otherPortFlag &= ~DLR_TIMER_RUNNING_MASK;
        *(otherPortPtr) = otherPortFlag;

        portEvtsFlag &= ~DLR_TIMER_RUNNING_MASK;

    }

    /*when the node receives a beacon with ring state set to ring fault*/
    if(portEvtsFlag & DLR_RING_FAULT_RCVD_MASK)
    {
#ifdef DLR_DEBUG
        genSeqOfEvents(RING_FAULT_RCVD_PORT0);
#endif
        otherPortPtr = (uint32_t *)(pruicssHwAttrs->pru1DramBase +
                                    DLR_PORT_EVENTS_OFFSET);
        otherPortFlag = *(otherPortPtr);

        /*stop the timer on port1*/
        EIP_DLR_disable_WD_IEP(dlrHandle, PORT1_WATCH_DOG_ID);

        /*write back the flag*/
        otherPortFlag &= ~DLR_TIMER_RUNNING_MASK;
        *(otherPortPtr) = otherPortFlag;

        /*flush learning table*/
        ioctlParams.command = ICSS_EMAC_LEARN_CTRL_CLR_TABLE;
        ICSS_EMAC_ioctl(dlricssEmacHandle, ICSS_EMAC_IOCTL_LEARNING_CTRL,
                       ICSS_EMAC_PORT_1, (void *)&ioctlParams);
        ICSS_EMAC_ioctl(dlricssEmacHandle, ICSS_EMAC_IOCTL_LEARNING_CTRL,
                       ICSS_EMAC_PORT_2, (void *)&ioctlParams);


        /*change CIP object variables*/
        EIP_DLR_switchToFault(dlrHandle);

        /*clear the flag*/
        portEvtsFlag &= ~DLR_RING_FAULT_RCVD_MASK;

        /*write back the flag*/
        *(portEvtsPtr) = portEvtsFlag;
    }

    /*ring transitions from idle to fault*/
    if(commonEvtsFlag & DLR_RING_FAULT_TRANSITION_MASK)
    {
#ifdef DLR_DEBUG
        genSeqOfEvents(RING_FAULT_TRANSITION_PORT0);
#endif
        /*clear the flag*/
        commonEvtsFlag &= ~(DLR_RING_FAULT_TRANSITION_MASK);
        /*write back the flag*/
        *(commonEvtsPtr) = commonEvtsFlag;

        EIP_DLR_updateSupervisorDetails(dlrHandle);

        /*change CIP object variables*/
        EIP_DLR_switchToFault(dlrHandle);

        /*flush learning table*/
        ioctlParams.command = ICSS_EMAC_LEARN_CTRL_CLR_TABLE;
        ICSS_EMAC_ioctl(dlricssEmacHandle, ICSS_EMAC_IOCTL_LEARNING_CTRL,
                       ICSS_EMAC_PORT_1, (void *)&ioctlParams);
        ICSS_EMAC_ioctl(dlricssEmacHandle, ICSS_EMAC_IOCTL_LEARNING_CTRL,
                       ICSS_EMAC_PORT_2, (void *)&ioctlParams);

    }

    /*ring transitions from fault to normal*/
    if(commonEvtsFlag & DLR_RING_NORMAL_TRANSITION_MASK)
    {
#ifdef DLR_DEBUG
        genSeqOfEvents(RING_NORMAL_TRANSITION_PORT0);
#endif

        /*flush learning table*/
        ioctlParams.command = ICSS_EMAC_LEARN_CTRL_CLR_TABLE;
        ICSS_EMAC_ioctl(dlricssEmacHandle, ICSS_EMAC_IOCTL_LEARNING_CTRL,
                       ICSS_EMAC_PORT_1, (void *)&ioctlParams);
        ICSS_EMAC_ioctl(dlricssEmacHandle, ICSS_EMAC_IOCTL_LEARNING_CTRL,
                       ICSS_EMAC_PORT_2, (void *)&ioctlParams);

        /*change CIP object variables*/
        EIP_DLR_switchToNormal(dlrHandle);

        /*clear the flag*/
        commonEvtsFlag &= ~(DLR_RING_NORMAL_TRANSITION_MASK);
        *(commonEvtsPtr) = commonEvtsFlag;
    }

    /*start the timer if the flag is set*/
    if(portEvtsFlag & DLR_START_TIMER_MASK)
    {
#ifdef DLR_DEBUG
        genSeqOfEvents(START_TIMER0);
#endif
        /*start the timer on port0*/
        EIP_DLR_enable_WD_IEP(dlrHandle, PORT0_WATCH_DOG_ID);
        /*set the flag*/
        portEvtsFlag |= DLR_TIMER_RUNNING_MASK;
        /*clear the flag*/
        portEvtsFlag &= ~DLR_START_TIMER_MASK;
    }

    /*Process a neighbor check response for port 0*/
    if(portEvtsFlag & DLR_NCRES_RCVD_MASK)
    {
#ifdef DLR_DEBUG
        genSeqOfEvents(NCRES_RCVD_PORT0);
#endif
        /*stop the neighbor timeout clock for port 0*/
        ClockP_stop(&(dlrHandle->dlrNeighborTimeoutClock[ICSS_EMAC_PORT_1 - 1]));
        portEvtsFlag &= ~(DLR_NCRES_RCVD_MASK);
    }

    /*Process a locate fault response*/
    if(portEvtsFlag & DLR_LOCFAULT_RCVD_MASK)
    {

#ifdef DLR_DEBUG
        genSeqOfEvents(LOCFAULT_RCVD_PORT0);
#endif

        /*Both links are up, send neighbor request on both frames*/
        /*Check link status of DLR_PORT1*/
        /*TODO: Review this*/
        // if(((ICSS_EmacObject *)dlricssEmacHandle->object)->linkStatus[DLR_PORT1 - 1])
        linkStatus = 0;
        retVal =  MDIO_phyLinkStatus(pruicssHwAttrs->miiMdioRegBase,
                                    ((ICSS_EMAC_Attrs *)(dlricssEmacHandle->attrs))->phyAddr[DLR_PORT1 - 1]);
        if(retVal == SystemP_SUCCESS)
            linkStatus = 1;

        if(linkStatus)
        {
            EIP_DLR_genNCReqFrame(dlrHandle, dlrHandle->dlrEmptyFrame, DLR_PORT0);
            /*TODO: Review this and other similar places*/
            // ICSS_EmacTxPacketEnqueue(dlricssEmacHandle, dlrHandle->dlrEmptyFrame, DLR_PORT0,
            //                          1,
            //                          DEFAULT_DLR_PACKET_SIZE);
            txArg.icssEmacHandle = dlricssEmacHandle;
            txArg.lengthOfPacket = DEFAULT_DLR_PACKET_SIZE;
            txArg.portNumber = DLR_PORT0;
            txArg.queuePriority = 1;
            txArg.srcAddress = dlrHandle->dlrEmptyFrame;
            ICSS_EMAC_txPacket(&txArg, NULL);

            ClockP_start(&(dlrHandle->dlrNeighborTimeoutClock[ICSS_EMAC_PORT_1 - 1]));

            EIP_DLR_genNCReqFrame(dlrHandle, dlrHandle->dlrEmptyFrame, DLR_PORT1);
            // ICSS_EmacTxPacketEnqueue(dlricssEmacHandle, dlrHandle->dlrEmptyFrame, DLR_PORT1,
            //                          1,
            //                          DEFAULT_DLR_PACKET_SIZE);
            txArg.icssEmacHandle = dlricssEmacHandle;
            txArg.lengthOfPacket = DEFAULT_DLR_PACKET_SIZE;
            txArg.portNumber = DLR_PORT1;
            txArg.queuePriority = 1;
            txArg.srcAddress = dlrHandle->dlrEmptyFrame;
            ICSS_EMAC_txPacket(&txArg, NULL);

            ClockP_start(&(dlrHandle->dlrNeighborTimeoutClock[ICSS_EMAC_PORT_2 - 1]));
        }
        else     /*Send a link status to supervisor*/
        {
            /*Since the message has come to port 0 hence port 1 is down*/
            EIP_DLR_genNeighborLinkStatFrame(dlrHandle, dlrHandle->dlrEmptyFrame, DLR_PORT0,
                                             IS_A_LINK_STATUS_FRAME,
                                             DLR_PORT1_FAIL);
            // ICSS_EmacTxPacketEnqueue(dlricssEmacHandle, dlrHandle->dlrEmptyFrame, DLR_PORT0,
            //                          1,
            //                          DEFAULT_DLR_PACKET_SIZE);
            txArg.icssEmacHandle = dlricssEmacHandle;
            txArg.lengthOfPacket = DEFAULT_DLR_PACKET_SIZE;
            txArg.portNumber = DLR_PORT0;
            txArg.queuePriority = 1;
            txArg.srcAddress = dlrHandle->dlrEmptyFrame;
            ICSS_EMAC_txPacket(&txArg, NULL);
        }

        /*clear the flag*/
        portEvtsFlag &= ~(DLR_LOCFAULT_RCVD_MASK);
    }

    /*Process a flush table frame*/
    if(portEvtsFlag & DLR_FLUSH_TABLE_RCVD_MASK)
    {
        /*flush learning table*/
        ioctlParams.command = ICSS_EMAC_LEARN_CTRL_CLR_TABLE;
        ICSS_EMAC_ioctl(dlricssEmacHandle, ICSS_EMAC_IOCTL_LEARNING_CTRL,
                       ICSS_EMAC_PORT_1, (void *)&ioctlParams);
        ICSS_EMAC_ioctl(dlricssEmacHandle, ICSS_EMAC_IOCTL_LEARNING_CTRL,
                       ICSS_EMAC_PORT_2, (void *)&ioctlParams);

        /*check for learning update*/
        if(portEvtsFlag & DLR_SEND_LEARNING_UPDATE_MASK)
        {
            /*Add sequence ID*/
            EIP_DLR_addWord(learnUpdateFrame + DLR_LEARN_UPDATE_SEQ_ID_OFFSET,
                    dlrHandle->sequenceID++);
            /*Send the packet*/
            // ICSS_EmacTxPacketEnqueue(dlricssEmacHandle, learnUpdateFrame, DLR_PORT0, 1,
            //                          DEFAULT_DLR_PACKET_SIZE);
            txArg.icssEmacHandle = dlricssEmacHandle;
            txArg.lengthOfPacket = DEFAULT_DLR_PACKET_SIZE;
            txArg.portNumber = DLR_PORT0;
            txArg.queuePriority = 1;
            txArg.srcAddress = learnUpdateFrame;
            ICSS_EMAC_txPacket(&txArg, NULL);

            /*clear the flag*/
            portEvtsFlag &= ~(DLR_SEND_LEARNING_UPDATE_MASK);
        }

        /*clear the flag*/
        portEvtsFlag &= ~(DLR_FLUSH_TABLE_RCVD_MASK);

    }

    /*write back the flag*/
    *(portEvtsPtr) = portEvtsFlag;

    /*Clear the interrupt by writing into PRUSS INTC Reg.
     * interrupt number ICSS_DLR_PORT0_INTNUM*/
    *intStatusPtr = ICSS_DLR_PORT0_INT_FLAG;


}
/**
*  @brief  Fast ISR for Port 1, bypasses the buffer copy and NDK
*
*  @param  arg [in] user argument. DLR handle
*
*  @retval  none
*
*/
void EIP_DLR_port1ISR(uintptr_t arg)
{
    uint32_t retVal = SystemP_FAILURE;
    uint32_t linkStatus;
    volatile uint32_t *portEvtsPtr;
    volatile uint8_t *commonEvtsPtr;
    volatile uint32_t *otherPortPtr;
    volatile uint32_t *intStatusPtr;
    volatile uint32_t portEvtsFlag;
    volatile uint8_t commonEvtsFlag;
    volatile uint32_t otherPortFlag;
    ICSS_EMAC_TxArgument txArg;
    EIP_DLRHandle dlrHandle = (EIP_DLRHandle)arg;
    ICSS_EMAC_Handle dlricssEmacHandle = dlrHandle->emacHandle;
    PRUICSS_HwAttrs const *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(dlrHandle->pruicssHandle->hwAttrs);

    ICSS_EMAC_IoctlCmd ioctlParams;

    portEvtsPtr = (uint32_t *)(pruicssHwAttrs->pru1DramBase +
                               DLR_PORT_EVENTS_OFFSET);
    portEvtsFlag = *(portEvtsPtr);

    commonEvtsPtr = (uint8_t *)(pruicssHwAttrs->sharedDramBase +
                                DLR_STATE_MACHINE_OFFSET + 3);
    commonEvtsFlag = *(commonEvtsPtr);

    intStatusPtr = (uint32_t *)(uint32_t)(pruicssHwAttrs->intcRegBase +
                                          CSL_ICSS_G_PR1_ICSS_INTC_INTC_SLV_ENA_STATUS_REG0);

    /*when a new supervisor comes online, timers are restarted*/
    if(commonEvtsFlag & DLR_STOP_BOTH_TIMERS_MASK)
    {

#ifdef DLR_DEBUG
        genSeqOfEvents(STOP_BOTH_TIMERS_PORT1);
#endif
        /*set the flag*/
        commonEvtsFlag |= (DLR_RESET_EVENT_OCCURED_MASK);
        *(commonEvtsPtr) = commonEvtsFlag;

        otherPortPtr = (uint32_t *)(pruicssHwAttrs->pru0DramBase +
                                    DLR_PORT_EVENTS_OFFSET);
        otherPortFlag = *(otherPortPtr);

        /*clear the flag*/
        commonEvtsFlag &= ~(DLR_STOP_BOTH_TIMERS_MASK);
        /*write back the flag*/
        *(commonEvtsPtr) = commonEvtsFlag;
        /*stop the timer on port1*/
        EIP_DLR_disable_WD_IEP(dlrHandle, PORT0_WATCH_DOG_ID);
        /*stop the timer on port1*/
        EIP_DLR_disable_WD_IEP(dlrHandle, PORT1_WATCH_DOG_ID);

        otherPortFlag &= ~DLR_TIMER_RUNNING_MASK;
        *(otherPortPtr) = otherPortFlag;

        portEvtsFlag &= ~DLR_TIMER_RUNNING_MASK;

        /*write back the flag*/
        *(portEvtsPtr) = portEvtsFlag;

    }

    /*when the node receives a beacon with ring state set to ring fault*/
    if(portEvtsFlag & DLR_RING_FAULT_RCVD_MASK)
    {
#ifdef DLR_DEBUG
        genSeqOfEvents(RING_FAULT_RCVD_PORT1);
#endif
        otherPortPtr = (uint32_t *)(pruicssHwAttrs->pru0DramBase +
                                    DLR_PORT_EVENTS_OFFSET);
        otherPortFlag = *(otherPortPtr);

        /*stop the timer on port0*/
        EIP_DLR_disable_WD_IEP(dlrHandle, PORT0_WATCH_DOG_ID);

        /*write back the flag*/
        otherPortFlag &= ~DLR_TIMER_RUNNING_MASK;
        *(otherPortPtr) = otherPortFlag;

        /*flush learning table*/
        ioctlParams.command = ICSS_EMAC_LEARN_CTRL_CLR_TABLE;
        ICSS_EMAC_ioctl(dlricssEmacHandle, ICSS_EMAC_IOCTL_LEARNING_CTRL,
                       ICSS_EMAC_PORT_1, (void *)&ioctlParams);
        ICSS_EMAC_ioctl(dlricssEmacHandle, ICSS_EMAC_IOCTL_LEARNING_CTRL,
                       ICSS_EMAC_PORT_2, (void *)&ioctlParams);
        /*change CIP object variables*/
        EIP_DLR_switchToFault(dlrHandle);

        /*clear the flag*/
        portEvtsFlag &= ~DLR_RING_FAULT_RCVD_MASK;
    }

    /*ring transitions from idle to fault*/
    if(commonEvtsFlag & DLR_RING_FAULT_TRANSITION_MASK)
    {
#ifdef DLR_DEBUG
        genSeqOfEvents(RING_FAULT_TRANSITION_PORT1);
#endif
        /*clear the flag*/
        commonEvtsFlag &= ~(DLR_RING_FAULT_TRANSITION_MASK);
        *(commonEvtsPtr) = commonEvtsFlag;

        EIP_DLR_updateSupervisorDetails(dlrHandle);

        /*change CIP object variables*/
        EIP_DLR_switchToFault(dlrHandle);

        /*flush learning table*/
        ioctlParams.command = ICSS_EMAC_LEARN_CTRL_CLR_TABLE;
        ICSS_EMAC_ioctl(dlricssEmacHandle, ICSS_EMAC_IOCTL_LEARNING_CTRL,
                       ICSS_EMAC_PORT_1, (void *)&ioctlParams);
        ICSS_EMAC_ioctl(dlricssEmacHandle, ICSS_EMAC_IOCTL_LEARNING_CTRL,
                       ICSS_EMAC_PORT_2, (void *)&ioctlParams);

    }

    /*ring transitions from fault to normal*/
    if(commonEvtsFlag & DLR_RING_NORMAL_TRANSITION_MASK)
    {
#ifdef DLR_DEBUG
        genSeqOfEvents(RING_NORMAL_TRANSITION_PORT1);
#endif
        /*clear the flag*/
        commonEvtsFlag &= ~(DLR_RING_NORMAL_TRANSITION_MASK);
        *(commonEvtsPtr) = commonEvtsFlag;

        /*change CIP object variables*/
        EIP_DLR_switchToNormal(dlrHandle);

        /*flush learning table*/
        ioctlParams.command = ICSS_EMAC_LEARN_CTRL_CLR_TABLE;
        ICSS_EMAC_ioctl(dlricssEmacHandle, ICSS_EMAC_IOCTL_LEARNING_CTRL,
                       ICSS_EMAC_PORT_1, (void *)&ioctlParams);
        ICSS_EMAC_ioctl(dlricssEmacHandle, ICSS_EMAC_IOCTL_LEARNING_CTRL,
                       ICSS_EMAC_PORT_2, (void *)&ioctlParams);

    }

    /*start the timer if the flag is set*/
    if(portEvtsFlag & DLR_START_TIMER_MASK)
    {
#ifdef DLR_DEBUG
        genSeqOfEvents(START_TIMER1);
#endif
        /*start the timer on port0*/
        EIP_DLR_enable_WD_IEP(dlrHandle, PORT1_WATCH_DOG_ID);
        /*set the flag*/
        portEvtsFlag |= DLR_TIMER_RUNNING_MASK;
        /*clear the flag*/
        portEvtsFlag &= ~DLR_START_TIMER_MASK;
    }

    /*Process a neighbor check response for port 0*/
    if(portEvtsFlag & DLR_NCRES_RCVD_MASK)
    {
#ifdef DLR_DEBUG
        genSeqOfEvents(NCRES_RCVD_PORT1);
#endif
        /*stop the neighbor timeout clock for port 0*/
        ClockP_stop(&(dlrHandle->dlrNeighborTimeoutClock[ICSS_EMAC_PORT_2 - 1]));
        portEvtsFlag &= ~(DLR_NCRES_RCVD_MASK);
    }

    /*Process a locate fault response*/
    if(portEvtsFlag & DLR_LOCFAULT_RCVD_MASK)
    {

#ifdef DLR_DEBUG
        genSeqOfEvents(LOCFAULT_RCVD_PORT1);
#endif

        /*Both links are up, send neighbor request on both frames*/
        /*Check link status of DLR_PORT0*/
        /*TODO: Review this and other similar pieces of code*/
        // if(((ICSS_EmacObject *)dlricssEmacHandle->object)->linkStatus[DLR_PORT0 - 1])
        linkStatus = 0;
        retVal =  MDIO_phyLinkStatus(pruicssHwAttrs->miiMdioRegBase,
                                    ((ICSS_EMAC_Attrs *)(dlricssEmacHandle->attrs))->phyAddr[DLR_PORT0 - 1]);
        if(retVal == SystemP_SUCCESS)
            linkStatus = 1;

        if(linkStatus)
        {

            EIP_DLR_genNCReqFrame(dlrHandle, dlrHandle->dlrEmptyFrame, DLR_PORT0);
            // ICSS_EmacTxPacketEnqueue(dlricssEmacHandle, dlrHandle->dlrEmptyFrame, DLR_PORT0,
            //                          1,
            //                          DEFAULT_DLR_PACKET_SIZE);
            txArg.icssEmacHandle = dlricssEmacHandle;
            txArg.lengthOfPacket = DEFAULT_DLR_PACKET_SIZE;
            txArg.portNumber = DLR_PORT0;
            txArg.queuePriority = 1;
            txArg.srcAddress = dlrHandle->dlrEmptyFrame;
            ICSS_EMAC_txPacket(&txArg, NULL);

            ClockP_start(&(dlrHandle->dlrNeighborTimeoutClock[ICSS_EMAC_PORT_1 - 1]));

            EIP_DLR_genNCReqFrame(dlrHandle, dlrHandle->dlrEmptyFrame, DLR_PORT1);
            // ICSS_EmacTxPacketEnqueue(dlricssEmacHandle, dlrHandle->dlrEmptyFrame, DLR_PORT1,
            //                          1,
            //                          DEFAULT_DLR_PACKET_SIZE);
            txArg.icssEmacHandle = dlricssEmacHandle;
            txArg.lengthOfPacket = DEFAULT_DLR_PACKET_SIZE;
            txArg.portNumber = DLR_PORT1;
            txArg.queuePriority = 1;
            txArg.srcAddress = dlrHandle->dlrEmptyFrame;
            ICSS_EMAC_txPacket(&txArg, NULL);
            ClockP_start(&(dlrHandle->dlrNeighborTimeoutClock[ICSS_EMAC_PORT_2 - 1]));

        }

        else
        {
            /*Since the message has come to port 1 hence port 0 is down*/
            EIP_DLR_genNeighborLinkStatFrame(dlrHandle, dlrHandle->dlrEmptyFrame, DLR_PORT1,
                                             IS_A_LINK_STATUS_FRAME,
                                             DLR_PORT0_FAIL);
            // ICSS_EmacTxPacketEnqueue(dlricssEmacHandle, dlrHandle->dlrEmptyFrame, DLR_PORT1,
            //                          1,
            //                          DEFAULT_DLR_PACKET_SIZE);
            txArg.icssEmacHandle = dlricssEmacHandle;
            txArg.lengthOfPacket = DEFAULT_DLR_PACKET_SIZE;
            txArg.portNumber = DLR_PORT1;
            txArg.queuePriority = 1;
            txArg.srcAddress = dlrHandle->dlrEmptyFrame;
            ICSS_EMAC_txPacket(&txArg, NULL);
        }

        /*clear the flag*/
        portEvtsFlag &= ~(DLR_LOCFAULT_RCVD_MASK);
    }

    /*Process a flush table frame*/
    if(portEvtsFlag & DLR_FLUSH_TABLE_RCVD_MASK)
    {

        /*flush learning table*/
        ioctlParams.command = ICSS_EMAC_LEARN_CTRL_CLR_TABLE;
        ICSS_EMAC_ioctl(dlricssEmacHandle, ICSS_EMAC_IOCTL_LEARNING_CTRL,
                       ICSS_EMAC_PORT_1, (void *)&ioctlParams);
        ICSS_EMAC_ioctl(dlricssEmacHandle, ICSS_EMAC_IOCTL_LEARNING_CTRL,
                       ICSS_EMAC_PORT_2, (void *)&ioctlParams);

        /*check for learning update*/
        if(portEvtsFlag & DLR_SEND_LEARNING_UPDATE_MASK)
        {

            /*Add sequence ID*/
            EIP_DLR_addWord(learnUpdateFrame + DLR_LEARN_UPDATE_SEQ_ID_OFFSET,
                    dlrHandle->sequenceID++);
            /*Send the packet*/
            // ICSS_EmacTxPacketEnqueue(dlricssEmacHandle, learnUpdateFrame, DLR_PORT1, 1,
            //                          DEFAULT_DLR_PACKET_SIZE);
            txArg.icssEmacHandle = dlricssEmacHandle;
            txArg.lengthOfPacket = DEFAULT_DLR_PACKET_SIZE;
            txArg.portNumber = DLR_PORT1;
            txArg.queuePriority = 1;
            txArg.srcAddress = learnUpdateFrame;

            ICSS_EMAC_txPacket(&txArg, NULL);

            /*clear the flag*/
            portEvtsFlag &= ~(DLR_SEND_LEARNING_UPDATE_MASK);
        }

        /*clear the flag*/
        portEvtsFlag &= ~(DLR_FLUSH_TABLE_RCVD_MASK);

    }

    /*write back the flag*/
    *(portEvtsPtr) = portEvtsFlag;

    /*Clear the interrupt by writing into PRUSS INTC Reg.
         * interrupt number ICSS_DLR_PORT0_INTNUM*/
    *intStatusPtr = ICSS_DLR_PORT1_INT_FLAG;

}

/**
*  @brief  ISR for Beacon timeout for Port 0
*
*  @param  arg [in] user argument. DLR handle
*
*  @retval  none
*
*/
void EIP_DLR_beaconTimeoutISR_P0(uintptr_t arg)
{
    volatile uint16_t *stateMachinePtr;
    volatile uint8_t *commonEvtsPtr;
    volatile uint32_t *portEvtsPtr;
    volatile uint8_t *ringStatePtr;
    volatile uint8_t *isrAckPtr;

    volatile uint8_t lastPortBcnRcvd;
    volatile uint8_t node_state;

    volatile uint8_t commonEvtsFlag;
    volatile uint8_t ringState;
    volatile uint16_t stateMachine;
    volatile uint32_t portEvtsFlag;
    volatile uint32_t intStatusOffset;

    uint8_t portNum;

    ICSS_EMAC_IoctlCmd ioctlParams;

    EIP_DLRHandle dlrHandle = (EIP_DLRHandle)arg;
    ICSS_EMAC_Handle dlricssEmacHandle = dlrHandle->emacHandle;
    PRUICSS_HwAttrs const *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(dlrHandle->pruicssHandle->hwAttrs);

    intStatusOffset = (uint32_t)(pruicssHwAttrs->intcRegBase
                                 + CSL_ICSS_G_PR1_ICSS_INTC_INTC_SLV_ENA_STATUS_REG0);

    portEvtsPtr = (uint32_t *)(pruicssHwAttrs->pru0DramBase +
                               DLR_PORT_EVENTS_OFFSET);
    isrAckPtr = (uint8_t *)(pruicssHwAttrs->pru0DramBase +
                            DLR_TIMEOUT_ISR_ACK_OFFSET);

    /*Clear the interrupt*/
    HW_WR_REG32(intStatusOffset, PORT0_WD_ISR_MASK);
    portNum = DLR_PORT0;

    /*Need to restart the Watchdog because it's not an auto load timer
     * Once it expires it stays expired
     */
    EIP_DLR_disable_WD_IEP(dlrHandle, PORT0_WATCH_DOG_ID);
    EIP_DLR_enable_WD_IEP(dlrHandle, PORT0_WATCH_DOG_ID);

    portEvtsFlag = *(portEvtsPtr);

    /*Find out if Beacon ISR's are stalling the Host
     * Stop the timer if that's the case*/

    if((dlrHandle->ISRcountPort[portNum - 1])++ > BEACON_CPU_STALL_THRESHOLD)
    {
        dlrHandle->ISRcountPort[portNum - 1] = 0;
        EIP_DLR_disable_WD_IEP(dlrHandle, portNum - 1);

        /*clear timer flag*/
        portEvtsFlag &= ~DLR_TIMER_RUNNING_MASK;
        *(portEvtsPtr) = portEvtsFlag;

#ifdef DLR_DEBUG
        genSeqOfEvents(PORT0_BEACON_STALL);
#endif
        /*Clear the memory location so more interrupts can come*/
        *isrAckPtr = DLR_FALSE;

        return;
    }

    commonEvtsPtr = (uint8_t *)(pruicssHwAttrs->sharedDramBase +
                                DLR_STATE_MACHINE_OFFSET + 3);
    commonEvtsFlag = *(commonEvtsPtr);

    stateMachinePtr = (uint16_t *)(pruicssHwAttrs->sharedDramBase +
                                   DLR_STATE_MACHINE_OFFSET);
    stateMachine = *(stateMachinePtr);

    ringStatePtr = (uint8_t *)(pruicssHwAttrs->sharedDramBase +
                               DLR_STATE_MACHINE_OFFSET + 2);
    ringState = *(ringStatePtr);

    node_state = stateMachine & 0xff;
    lastPortBcnRcvd = (uint8_t)((stateMachine >> 8) & 0xff);

#ifdef DLR_DEBUG
    genSeqOfEvents(BEACON0_MISSED);
#endif

    if(NODE_FAULT == node_state)
    {
        if(lastPortBcnRcvd == portNum)
        {
#ifdef DLR_DEBUG
            genSeqOfEvents(BEACON0_MISSED_FAULT);
#endif
            /*Stop neighbor checkout timers*/
            ClockP_stop(&(dlrHandle->dlrNeighborTimeoutClock[ICSS_EMAC_PORT_1 - 1]));
            ClockP_stop(&(dlrHandle->dlrNeighborTimeoutClock[ICSS_EMAC_PORT_2 - 1]));

            /*Transition to idle*/
            EIP_DLR_resetStateMachine(dlrHandle);
        }

    }

    if(NODE_NORMAL == node_state)
    {
        /*transition to fault*/
        node_state = NODE_FAULT;

#ifdef DLR_DEBUG
        genSeqOfEvents(BEACON0_MISSED_NORMAL);
#endif
        lastPortBcnRcvd = DLR_PORT1;
        commonEvtsFlag |= DLR_PORT1_BEACON_RCVD_MASK;
        commonEvtsFlag &= ~DLR_PORT0_BEACON_RCVD_MASK;

        stateMachine = 0;
        ringState = 0;
        stateMachine |= (node_state & 0xff);
        ringState |= DLR_RING_FAULT_MASK;
        stateMachine |= ((lastPortBcnRcvd & 0xff) << 8);

        /*write back the flag*/
        *(stateMachinePtr) = stateMachine;
        *(commonEvtsPtr) = commonEvtsFlag;
        *(ringStatePtr) = ringState;

        /*change CIP object variables*/
        EIP_DLR_switchToFault(dlrHandle);

        /*flush the learning table*/
        ioctlParams.command = ICSS_EMAC_LEARN_CTRL_CLR_TABLE;
        ICSS_EMAC_ioctl(dlricssEmacHandle, ICSS_EMAC_IOCTL_LEARNING_CTRL,
                       ICSS_EMAC_PORT_1, (void *)&ioctlParams);
        ICSS_EMAC_ioctl(dlricssEmacHandle, ICSS_EMAC_IOCTL_LEARNING_CTRL,
                       ICSS_EMAC_PORT_2, (void *)&ioctlParams);
    }

    /*Clear the memory location so more interrupts can come*/
    *isrAckPtr = DLR_FALSE;

}

/**
*  @brief  ISR for Beacon timeout for Port 1
*
*  @param  arg [in] user argument. DLR handle
*
*  @retval  none
*
*/
void EIP_DLR_beaconTimeoutISR_P1(uintptr_t arg)
{
    volatile uint16_t *stateMachinePtr;
    volatile uint8_t *commonEvtsPtr;
    volatile uint8_t *isrAckPtr;
    volatile uint32_t *portEvtsPtr;
    volatile uint8_t *ringStatePtr;

    volatile uint8_t lastPortBcnRcvd;
    volatile uint8_t node_state;

    volatile uint8_t commonEvtsFlag;
    volatile uint8_t ringState;
    volatile uint16_t stateMachine;
    volatile uint32_t portEvtsFlag;
    volatile uint32_t intStatusOffset;

    uint8_t portNum;

    ICSS_EMAC_IoctlCmd ioctlParams;

    EIP_DLRHandle dlrHandle = (EIP_DLRHandle)arg;
    ICSS_EMAC_Handle dlricssEmacHandle = dlrHandle->emacHandle;
    PRUICSS_HwAttrs const *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(dlrHandle->pruicssHandle->hwAttrs);

    intStatusOffset = (uint32_t)(pruicssHwAttrs->intcRegBase
                                 + CSL_ICSS_G_PR1_ICSS_INTC_INTC_SLV_ENA_STATUS_REG0);

    portEvtsPtr = (uint32_t *)(pruicssHwAttrs->pru1DramBase +
                               DLR_PORT_EVENTS_OFFSET);
    isrAckPtr = (uint8_t *)(pruicssHwAttrs->pru1DramBase +
                            DLR_TIMEOUT_ISR_ACK_OFFSET);

    /*clear the interrupt*/
    HW_WR_REG32(intStatusOffset, PORT1_WD_ISR_MASK);
    portNum = DLR_PORT1;

    /*Need to restart the Watchdog because it's not an auto load timer
     * Once it expires it stays expired
     */
    EIP_DLR_disable_WD_IEP(dlrHandle, PORT1_WATCH_DOG_ID);
    EIP_DLR_enable_WD_IEP(dlrHandle, PORT1_WATCH_DOG_ID);

    portEvtsFlag = *(portEvtsPtr);

    /*Find out if Beacon ISR's are stalling the Host
     * Stop the timer if that's the case*/

    if((dlrHandle->ISRcountPort[portNum - 1])++ > BEACON_CPU_STALL_THRESHOLD)
    {
        dlrHandle->ISRcountPort[portNum - 1] = 0;
        EIP_DLR_disable_WD_IEP(dlrHandle, portNum - 1);

        /*clear timer flag*/
        portEvtsFlag &= ~DLR_TIMER_RUNNING_MASK;
        *(portEvtsPtr) = portEvtsFlag;

#ifdef DLR_DEBUG
        genSeqOfEvents(PORT1_BEACON_STALL);
#endif

        /*Clear the memory location so more interrupts can come*/
        *isrAckPtr = DLR_FALSE;
        return;
    }

    commonEvtsPtr = (uint8_t *)(pruicssHwAttrs->sharedDramBase +
                                DLR_STATE_MACHINE_OFFSET + 3);
    commonEvtsFlag = *(commonEvtsPtr);

    stateMachinePtr = (uint16_t *)(pruicssHwAttrs->sharedDramBase +
                                   DLR_STATE_MACHINE_OFFSET);
    stateMachine = *(stateMachinePtr);

    ringStatePtr = (uint8_t *)(pruicssHwAttrs->sharedDramBase +
                               DLR_STATE_MACHINE_OFFSET + 2);
    ringState = *(ringStatePtr);

    node_state = stateMachine & 0xff;
    lastPortBcnRcvd = (uint8_t)((stateMachine >> 8) & 0xff);

#ifdef DLR_DEBUG
    genSeqOfEvents(BEACON1_MISSED);
#endif

    if(NODE_FAULT == node_state)
    {
        if(lastPortBcnRcvd == portNum)
        {
#ifdef DLR_DEBUG
            genSeqOfEvents(BEACON0_MISSED_FAULT);
#endif
            /*Stop neighbor checkout timers*/
            ClockP_stop(&(dlrHandle->dlrNeighborTimeoutClock[ICSS_EMAC_PORT_1 - 1]));
            ClockP_stop(&(dlrHandle->dlrNeighborTimeoutClock[ICSS_EMAC_PORT_2 - 1]));

            /*Transition to idle*/
            EIP_DLR_resetStateMachine(dlrHandle);
        }

    }

    if(NODE_NORMAL == node_state)
    {
        /*transition to fault*/
        node_state = NODE_FAULT;

#ifdef DLR_DEBUG
        genSeqOfEvents(BEACON1_MISSED_NORMAL);
#endif
        lastPortBcnRcvd = DLR_PORT0;
        commonEvtsFlag |= DLR_PORT0_BEACON_RCVD_MASK;
        commonEvtsFlag &= ~DLR_PORT1_BEACON_RCVD_MASK;

        stateMachine = 0;
        ringState = 0;
        stateMachine |= (node_state & 0xff);
        ringState |= DLR_RING_FAULT_MASK;
        stateMachine |= ((lastPortBcnRcvd & 0xff) << 8);

        /*write back the flag*/
        *(stateMachinePtr) = stateMachine;
        *(commonEvtsPtr) = commonEvtsFlag;
        *(ringStatePtr) = ringState;

        /*change CIP object variables*/
        EIP_DLR_switchToFault(dlrHandle);

        /*flush the learning table*/
        ioctlParams.command = ICSS_EMAC_LEARN_CTRL_CLR_TABLE;
        ICSS_EMAC_ioctl(dlricssEmacHandle, ICSS_EMAC_IOCTL_LEARNING_CTRL,
                       ICSS_EMAC_PORT_1, (void *)&ioctlParams);
        ICSS_EMAC_ioctl(dlricssEmacHandle, ICSS_EMAC_IOCTL_LEARNING_CTRL,
                       ICSS_EMAC_PORT_2, (void *)&ioctlParams);
    }

    /*Clear the memory location so more interrupts can come*/
    *isrAckPtr = DLR_FALSE;
}

void EIP_DLR_neighborTimeoutISR0(ClockP_Object *obj, void *arg)
{
    EIP_DLRHandle dlrHandle = (EIP_DLRHandle)arg;
    ICSS_EMAC_Handle dlricssEmacHandle = dlrHandle->emacHandle;
    PRUICSS_HwAttrs const *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(dlrHandle->pruicssHandle->hwAttrs);

    volatile uint8_t *bytePtr = (uint8_t *)(pruicssHwAttrs->pru0DramBase +
                                            DLR_NCREQ_NUM_RETRIES_OFFSET);
    volatile uint16_t *stateMachinePtr;

    /*Number of times slave will ping it's neighbor. This is set in PRU*/
    volatile uint8_t numRetries;
    volatile uint16_t stateMachine;
    ICSS_EMAC_TxArgument txArg;

    /*Load the state machine*/
    stateMachinePtr = (uint16_t *)(pruicssHwAttrs->sharedDramBase +
                                   DLR_STATE_MACHINE_OFFSET);
    stateMachine = *(stateMachinePtr);

    /*Don't run neighbor check if state has switched to normal*/
    if((stateMachine & 0xff) == DLR_NORMAL_STATE_VAL)
    {
        *(bytePtr) = 0;
        /*stop the timer*/
        ClockP_stop(&(dlrHandle->dlrNeighborTimeoutClock[ICSS_EMAC_PORT_1 - 1]));
    }

    numRetries = *(bytePtr);

    if(numRetries < MAX_NUM_RETRIES)
    {
#ifdef DLR_DEBUG
        genSeqOfEvents(NEIGHBOR_TIMEOUT_PORT0_RETRY);
#endif
        /*send a Neighbor check request again*/
        EIP_DLR_genNCReqFrame(dlrHandle, dlrHandle->dlrEmptyFrame, DLR_PORT0);
        // ICSS_EmacTxPacketEnqueue(dlricssEmacHandle, dlrHandle->dlrEmptyFrame, DLR_PORT0,
        //                          1,
        //                          DEFAULT_DLR_PACKET_SIZE);
        txArg.icssEmacHandle = dlricssEmacHandle;
        txArg.lengthOfPacket = DEFAULT_DLR_PACKET_SIZE;
        txArg.portNumber = DLR_PORT0;
        txArg.queuePriority = 1;
        txArg.srcAddress = dlrHandle->dlrEmptyFrame;

        ICSS_EMAC_txPacket(&txArg, NULL);

        numRetries++;
        *(bytePtr) = numRetries;

    }

    else
    {
#ifdef DLR_DEBUG
        genSeqOfEvents(NEIGHBOR_TIMEOUT_PORT0_MAX);
#endif
        /*The neighbor has failed to respond,
         * send a neighbor status frame to the host on the other port
         */
        EIP_DLR_genNeighborLinkStatFrame(dlrHandle, dlrHandle->dlrEmptyFrame, DLR_PORT1,
                                         IS_A_NEIGHBOR_STAT_FRAME,
                                         DLR_PORT0_FAIL);
        // ICSS_EmacTxPacketEnqueue(dlricssEmacHandle, dlrHandle->dlrEmptyFrame, DLR_PORT1,
        //                          1,
        //                          DEFAULT_DLR_PACKET_SIZE);
        txArg.icssEmacHandle = dlricssEmacHandle;
        txArg.lengthOfPacket = DEFAULT_DLR_PACKET_SIZE;
        txArg.portNumber = DLR_PORT1;
        txArg.queuePriority = 1;
        txArg.srcAddress = dlrHandle->dlrEmptyFrame;

        ICSS_EMAC_txPacket(&txArg, NULL);
        /*stop the timer*/
        ClockP_stop(&(dlrHandle->dlrNeighborTimeoutClock[ICSS_EMAC_PORT_1 - 1]));

        *(bytePtr) = 0;
    }

}

void EIP_DLR_neighborTimeoutISR1(ClockP_Object *obj, void *arg)
{
    EIP_DLRHandle dlrHandle = (EIP_DLRHandle)arg;
    ICSS_EMAC_Handle dlricssEmacHandle = dlrHandle->emacHandle;
    PRUICSS_HwAttrs const *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(dlrHandle->pruicssHandle->hwAttrs);

    volatile uint8_t *bytePtr = (uint8_t *)(pruicssHwAttrs->pru1DramBase +
                                            DLR_NCREQ_NUM_RETRIES_OFFSET);
    volatile uint16_t *stateMachinePtr;

    volatile uint8_t numRetries;
    volatile uint16_t stateMachine;
    ICSS_EMAC_TxArgument txArg;

    /*Load the state machine*/
    stateMachinePtr = (uint16_t *)(pruicssHwAttrs->sharedDramBase +
                                   DLR_STATE_MACHINE_OFFSET);
    stateMachine = *(stateMachinePtr);

    /*Don't run neighbor check if state has switched to normal*/
    if((stateMachine & 0xff) == DLR_NORMAL_STATE_VAL)
    {
        *(bytePtr) = 0;
        /*stop the timer*/
        ClockP_stop(&(dlrHandle->dlrNeighborTimeoutClock[ICSS_EMAC_PORT_2 - 1]));
    }

    numRetries = *(bytePtr);

    if(numRetries < MAX_NUM_RETRIES)
    {

#ifdef DLR_DEBUG
        genSeqOfEvents(NEIGHBOR_TIMEOUT_PORT1_RETRY);
#endif
        /*send a Neighbor check request again*/
        EIP_DLR_genNCReqFrame(dlrHandle, dlrHandle->dlrEmptyFrame, DLR_PORT1);
        // ICSS_EmacTxPacketEnqueue(dlricssEmacHandle, dlrHandle->dlrEmptyFrame, DLR_PORT1,
        //                          1,
        //                          DEFAULT_DLR_PACKET_SIZE);
        txArg.icssEmacHandle = dlricssEmacHandle;
        txArg.lengthOfPacket = DEFAULT_DLR_PACKET_SIZE;
        txArg.portNumber = DLR_PORT1;
        txArg.queuePriority = 1;
        txArg.srcAddress = dlrHandle->dlrEmptyFrame;

        ICSS_EMAC_txPacket(&txArg, NULL);
        numRetries++;
        *(bytePtr) = numRetries;
    }

    else
    {

#ifdef DLR_DEBUG
        genSeqOfEvents(NEIGHBOR_TIMEOUT_PORT1_MAX);
#endif
        /*The neighbor has failed to respond,
         * send a neighbor status frame to the host on the other port
         */
        EIP_DLR_genNeighborLinkStatFrame(dlrHandle, dlrHandle->dlrEmptyFrame, DLR_PORT0,
                                         IS_A_NEIGHBOR_STAT_FRAME,
                                         DLR_PORT1_FAIL);
        // ICSS_EmacTxPacketEnqueue(dlricssEmacHandle, dlrHandle->dlrEmptyFrame, DLR_PORT0,
        //                          1,
        //                          DEFAULT_DLR_PACKET_SIZE);
        txArg.icssEmacHandle = dlricssEmacHandle;
        txArg.lengthOfPacket = DEFAULT_DLR_PACKET_SIZE;
        txArg.portNumber = DLR_PORT0;
        txArg.queuePriority = 1;
        txArg.srcAddress = dlrHandle->dlrEmptyFrame;

        ICSS_EMAC_txPacket(&txArg, NULL);
        /*stop the timer*/
        ClockP_stop(&(dlrHandle->dlrNeighborTimeoutClock[ICSS_EMAC_PORT_2 - 1]));
        *(bytePtr) = 0;
    }
}
/**
*  @brief Processes a sign on and Neighbor check request frame
*
*  @param  dlrHandle [in] DLR handle
*  @param  pktBuffer [in] pointer to bytestream
*  @param  portNum   [in] Port number wherer the packet arrived
*  @param  size      [in] size of packet
*
*  @retval  none
*
*/
void EIP_DLR_processDLRFrame(EIP_DLRHandle dlrHandle, uint8_t *pktBuffer,
                             uint8_t portNum, uint16_t size)
{

    ICSS_EMAC_Handle dlricssEmacHandle = dlrHandle->emacHandle;
    PRUICSS_HwAttrs const *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(dlrHandle->pruicssHandle->hwAttrs);

    volatile uint8_t *signOnRcvdStatus;
    volatile uint8_t signOnRcvdFlag;

    volatile uint32_t *portEvtsPtr;
    volatile uint32_t portEvtsFlag;

    uint8_t *ifMacID;
    activeSuperAddr *actSupAddrPtr;
    /*Number of slaves in the network*/
    volatile uint16_t numNodes;
    volatile uint16_t offset;

    uint8_t *dlrSrc = (uint8_t *)pktBuffer;
    ICSS_EMAC_TxArgument txArg;

    /*TODO: Review this*/
    ifMacID = (uint8_t *)(&(dlrHandle->macId[0]));

    signOnRcvdStatus = (uint8_t *)(pruicssHwAttrs->sharedDramBase +
                                   DLR_SIGNON_PKT_RCVD_OFFSET);
    signOnRcvdFlag = *(signOnRcvdStatus);

    if(portNum)
    {
        portEvtsPtr = (uint32_t *)(pruicssHwAttrs->pru1DramBase +
                                   DLR_PORT_EVENTS_OFFSET);
        portEvtsFlag = *(portEvtsPtr);
    }

    else
    {
        portEvtsPtr = (uint32_t *)(pruicssHwAttrs->pru0DramBase +
                                   DLR_PORT_EVENTS_OFFSET);
        portEvtsFlag = *(portEvtsPtr);
    }

    /*Process a neighbor check request*/
    if(portEvtsFlag & DLR_NCREQ_RCVD_MASK)
    {

        volatile uint32_t sequenceId;
        volatile uint8_t reqSrcPort;

#ifdef DLR_DEBUG

        if(portNum)
        {
            genSeqOfEvents(NCREQ_RCVD_PORT1);
        }

        else
        {
            genSeqOfEvents(NCREQ_RCVD_PORT0);
        }

#endif
        sequenceId = EIP_DLR_convBigEndianToLittleEndianWord((uint8_t *)(
                         dlrSrc + DLR_SEQUENCE_ID_OFFSET));

        reqSrcPort = *(dlrSrc + DLR_SOURCE_PORT_OFFSET);

        /*generate the neighbor check response frame*/
        EIP_DLR_genNCResFrame(dlrHandle->dlrEmptyFrame, portNum + 1, reqSrcPort,
                              sequenceId);
        /*send the neighbor check response frame*/
        // ICSS_EmacTxPacketEnqueue(dlricssEmacHandle, dlrHandle->dlrEmptyFrame,
        //                          portNum + 1, 1,
        //                          DEFAULT_DLR_PACKET_SIZE);
        txArg.icssEmacHandle = dlricssEmacHandle;
        txArg.lengthOfPacket = DEFAULT_DLR_PACKET_SIZE;
        txArg.portNumber = portNum + 1;
        txArg.queuePriority = 1;
        txArg.srcAddress = dlrHandle->dlrEmptyFrame;

        ICSS_EMAC_txPacket(&txArg, NULL);
        /*clear the flag*/
        portEvtsFlag &= ~(DLR_NCREQ_RCVD_MASK);
        *(portEvtsPtr) = portEvtsFlag;
    }

    if(signOnRcvdFlag != DLR_TRUE)
    {
        return;

    }

    else
    {
        /*clear the flag*/
        signOnRcvdFlag =  DLR_FALSE;
        *(signOnRcvdStatus) = signOnRcvdFlag;
    }

    actSupAddrPtr = &(dlrHandle->dlrObj->addr);

#ifdef DLR_DEBUG
    genSeqOfEvents(COMPLETE_SIGNON_FRAME_RCVD);
#endif

    /*send on the other port*/
    portNum++;
    portNum = ~portNum;
    portNum = portNum & 0x3;

    /*check if the size exceeds max size
     * If it does then send the frame directly to host*/
    if(size > DLR_SIGNON_FRAME_SIZE)
    {
        /*modify the src and destination mac address field*/
        /*Add the source MAC id to DLR packet*/
        EIP_DLR_addMACID(dlrSrc + DLR_SRC_MAC_OFFSET, ifMacID);
        /*This packet is meant for the ring supervisor*/
        EIP_DLR_addMACID(dlrSrc, actSupAddrPtr->supMACAddress);
        // ICSS_EmacTxPacketEnqueue(dlricssEmacHandle, dlrSrc, portNum, 1, size);
        txArg.icssEmacHandle = dlricssEmacHandle;
        txArg.lengthOfPacket = size;
        txArg.portNumber = portNum;
        txArg.queuePriority = 1;
        txArg.srcAddress = dlrSrc;

        ICSS_EMAC_txPacket(&txArg, NULL);
    }

    else
    {

        /*Modify the source MAC Id field*/
        EIP_DLR_addMACID(dlrSrc + 6, ifMacID);

        /*get a list of all devices*/
        numNodes = EIP_DLR_convBigEndianToLittleEndianHalfWord(dlrSrc +
                   DLR_SIGNON_NUM_NODES_OFFSET);
        /*jump straight to where new mac id and ip address needs to be added*/
        offset = (DLR_SIGNON_SUP_MAC_ADD_OFFSET + (numNodes * 10));

        numNodes++;
        /*Increment the number of nodes in the network*/
        EIP_DLR_addSignOnNumNodes(dlrSrc, numNodes);

        dlrSrc += offset;
        /*append the device's own mac id and IP address*/
        EIP_DLR_addMACID(dlrSrc, ifMacID);
        EIP_DLR_addWord(dlrSrc + 6, dlrHandle->deviceIP);
        offset += 10;

        if(offset < 60)
        {
            // ICSS_EmacTxPacketEnqueue(dlricssEmacHandle, pktBuffer, portNum, 1, 60);
            txArg.icssEmacHandle = dlricssEmacHandle;
            txArg.lengthOfPacket = 60;
            txArg.portNumber = portNum;
            txArg.queuePriority = 1;
            txArg.srcAddress = pktBuffer;

            ICSS_EMAC_txPacket(&txArg, NULL);
        }

        else
        {
            // ICSS_EmacTxPacketEnqueue(dlricssEmacHandle, pktBuffer, portNum, 1, offset);
            txArg.icssEmacHandle = dlricssEmacHandle;
            txArg.lengthOfPacket = offset;
            txArg.portNumber = portNum;
            txArg.queuePriority = 1;
            txArg.srcAddress = pktBuffer;

            ICSS_EMAC_txPacket(&txArg, NULL);
        }

    }
}

/**
*  @brief Process DLR state machine in the event of a link break on Port0
*
*  @param  linkStatus [in] link status of the port. Up/Down. 1/0
*  @param  arg2       [in] argument DLR handle
*
*  @retval  none
*
*/
void EIP_DLR_port0ProcessLinkBrk(uint8_t linkStatus, void *arg2)
{
    uint32_t tempLinkStatus;
    uint32_t retVal = SystemP_FAILURE;
    EIP_DLRHandle dlrHandle = (EIP_DLRHandle)arg2;
    ICSS_EMAC_Handle dlricssEmacHandle = dlrHandle->emacHandle;
    PRUICSS_HwAttrs const *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(dlrHandle->pruicssHandle->hwAttrs);
    ICSS_EMAC_TxArgument txArg;

    volatile uint16_t *stateMachinePtr;
    volatile uint32_t *port0EvtsPtr;
    volatile uint8_t *commonEvtsPtr;
    volatile uint8_t *ringStatePtr;

    volatile uint16_t stateMachine;
    volatile uint32_t port0EvtsFlag;
    volatile uint8_t commonEvtsFlag;
    volatile uint8_t ringState;

    volatile uint8_t node_state;
    volatile uint8_t lastPortBcnRcvd;

    volatile uint8_t *loopDetectedPtr;
    ICSS_EMAC_IoctlCmd ioctlParams;

    if(NODE_IDLE == dlrHandle->dlrObj->SMVariables.node_state)
    {
        /*If link broken*/
        if(!linkStatus)
        {
            /*Since there is a link break update loop in network status*/
            loopDetectedPtr = (uint8_t *)(pruicssHwAttrs->sharedDramBase +
                                          DLR_LOOP_DETECTED_OFFSET);
            /*TODO: Check this. FALSE_VAL->0*/
            // *(loopDetectedPtr) =  FALSE_VAL;
            *(loopDetectedPtr) =  0;
            dlrHandle->dlrObj->SMVariables.status = NORMAL_STAT;
            dlrHandle->checkForLoop = DLR_FALSE;

        }

        else
        {
            /*Check link status of ICSS_EMAC_PORT_2*/
            // if(((ICSS_EmacObject *)dlricssEmacHandle->object)->linkStatus[ICSS_EMAC_PORT_2 - 1])
            tempLinkStatus = 0;
            retVal =  MDIO_phyLinkStatus(pruicssHwAttrs->miiMdioRegBase,
                                        ((ICSS_EMAC_Attrs *)(dlricssEmacHandle->attrs))->phyAddr[ICSS_EMAC_PORT_2 - 1]);
            if(retVal == SystemP_SUCCESS)
                tempLinkStatus = 1;

            if(tempLinkStatus)
            {
                dlrHandle->checkForLoop = DLR_TRUE;
            }

            else
            {
                dlrHandle->checkForLoop = DLR_FALSE;
            }
        }

        return;
    }

    /*Load port 0 events flag*/
    port0EvtsPtr = (uint32_t *)(pruicssHwAttrs->pru0DramBase +
                                DLR_PORT_EVENTS_OFFSET);
    port0EvtsFlag = *(port0EvtsPtr);

    /*Load the state machine*/
    stateMachinePtr = (uint16_t *)(pruicssHwAttrs->sharedDramBase +
                                   DLR_STATE_MACHINE_OFFSET);
    stateMachine = *(stateMachinePtr);

    ringStatePtr = (uint8_t *)(pruicssHwAttrs->sharedDramBase +
                               DLR_STATE_MACHINE_OFFSET + 2);
    ringState = *(ringStatePtr);

    /*Load common events flag*/
    commonEvtsPtr = (uint8_t *)(pruicssHwAttrs->sharedDramBase +
                                DLR_STATE_MACHINE_OFFSET + 3);
    commonEvtsFlag = *(commonEvtsPtr);

    node_state = stateMachine & 0xff;
    lastPortBcnRcvd = (uint8_t)((stateMachine >> 8) & 0xff);

    if(DLR_FALSE == linkStatus)   /*Link 0 down*/
    {

        /*In case state machine is still running with both links disabled*/
        /*Check link status of ICSS_EMAC_PORT_2*/
        // if(((ICSS_EmacObject *)dlricssEmacHandle->object)->linkStatus[ICSS_EMAC_PORT_2 - 1])
        tempLinkStatus = 0;
        retVal =  MDIO_phyLinkStatus(pruicssHwAttrs->miiMdioRegBase,
                                    ((ICSS_EMAC_Attrs *)(dlricssEmacHandle->attrs))->phyAddr[ICSS_EMAC_PORT_2 - 1]);
        if(retVal == SystemP_SUCCESS)
            tempLinkStatus = 1;

        if(tempLinkStatus)
        {
            EIP_DLR_resetStateMachine(dlrHandle);
            return;
        }

        /*Fault state*/
        if(NODE_FAULT == node_state)
        {
#ifdef DLR_DEBUG
            genSeqOfEvents(LINK1_FAULT_BREAK);
#endif

            /*check last port on which beacon was received*/
            if(lastPortBcnRcvd == DLR_PORT0)
            {
                /*Stop neighbor checkout timers*/
                ClockP_stop(&(dlrHandle->dlrNeighborTimeoutClock[ICSS_EMAC_PORT_1 - 1]));
                ClockP_stop(&(dlrHandle->dlrNeighborTimeoutClock[ICSS_EMAC_PORT_2 - 1]));

                /*if it's the same port then go to idle state*/
                EIP_DLR_resetStateMachine(dlrHandle);
            }

            else
            {
                /*send a link status message to active
                 * ring supervisor on link 1 that link 0 has failed */
                EIP_DLR_genNeighborLinkStatFrame(dlrHandle, dlrHandle->dlrEmptyFrame, DLR_PORT1,
                                                 IS_A_LINK_STATUS_FRAME,
                                                 DLR_PORT0_FAIL);
                // ICSS_EmacTxPacketEnqueue(dlricssEmacHandle, dlrHandle->dlrEmptyFrame, DLR_PORT1,
                //                          1,
                //                          DEFAULT_DLR_PACKET_SIZE);
                txArg.icssEmacHandle = dlricssEmacHandle;
                txArg.lengthOfPacket = DEFAULT_DLR_PACKET_SIZE;
                txArg.portNumber = DLR_PORT1;
                txArg.queuePriority = 1;
                txArg.srcAddress = dlrHandle->dlrEmptyFrame;

                ICSS_EMAC_txPacket(&txArg, NULL);
            }

        }

        /*Normal state*/
        if(NODE_NORMAL == node_state)
        {
#ifdef DLR_DEBUG
            genSeqOfEvents(LINK1_NORMAL_BREAK);
#endif
            /*transition to fault*/
            node_state = NODE_FAULT;
            EIP_DLR_disable_WD_IEP(dlrHandle, PORT0_WATCH_DOG_ID);

            port0EvtsFlag &= ~DLR_TIMER_RUNNING_MASK;
            *(port0EvtsPtr) = port0EvtsFlag;

            /*flush learning table*/
            ioctlParams.command = ICSS_EMAC_LEARN_CTRL_CLR_TABLE;
            ICSS_EMAC_ioctl(dlricssEmacHandle, ICSS_EMAC_IOCTL_LEARNING_CTRL,
                           ICSS_EMAC_PORT_1, (void *)&ioctlParams);
            ICSS_EMAC_ioctl(dlricssEmacHandle, ICSS_EMAC_IOCTL_LEARNING_CTRL,
                           ICSS_EMAC_PORT_2, (void *)&ioctlParams);
            stateMachine = 0;
            ringState = 0;
            lastPortBcnRcvd = DLR_PORT1;
            stateMachine |= (node_state & 0xff);
            ringState |= DLR_RING_FAULT_MASK;
            stateMachine |= (lastPortBcnRcvd << 8);
            *(stateMachinePtr) = stateMachine;
            *(ringStatePtr) = ringState;

            commonEvtsFlag |= DLR_PORT1_BEACON_RCVD_MASK;
            commonEvtsFlag &= ~DLR_PORT0_BEACON_RCVD_MASK;

            *(commonEvtsPtr) = commonEvtsFlag;

            /*send link status frame to active ring supervisor*/
            EIP_DLR_genNeighborLinkStatFrame(dlrHandle, dlrHandle->dlrEmptyFrame, DLR_PORT1,
                                             IS_A_LINK_STATUS_FRAME,
                                             DLR_PORT0_FAIL);
            // ICSS_EmacTxPacketEnqueue(dlricssEmacHandle, dlrHandle->dlrEmptyFrame, DLR_PORT1,
            //                          1,
            //                          DEFAULT_DLR_PACKET_SIZE);
            txArg.icssEmacHandle = dlricssEmacHandle;
            txArg.lengthOfPacket = DEFAULT_DLR_PACKET_SIZE;
            txArg.portNumber = DLR_PORT1;
            txArg.queuePriority = 1;
            txArg.srcAddress = dlrHandle->dlrEmptyFrame;

            ICSS_EMAC_txPacket(&txArg, NULL);
            /*change CIP object variables*/
            EIP_DLR_switchToFault(dlrHandle);
        }

    }

}
/**
*  @brief Process DLR state machine in the event of a link break on Port1
*
*  @param  linkStatus [in] link status of the port. Up/Down. 1/0
*  @param  arg2       [in] argument DLR handle
*
*  @retval  none
*
*/
void EIP_DLR_port1ProcessLinkBrk(uint8_t linkStatus, void *arg2)
{
    uint32_t tempLinkStatus;
    uint32_t retVal = SystemP_FAILURE;
    EIP_DLRHandle dlrHandle = (EIP_DLRHandle)arg2;
    ICSS_EMAC_Handle dlricssEmacHandle = dlrHandle->emacHandle;
    PRUICSS_HwAttrs const *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(dlrHandle->pruicssHandle->hwAttrs);
    ICSS_EMAC_TxArgument txArg;

    volatile uint16_t *stateMachinePtr;
    volatile uint32_t *port1EvtsPtr;
    volatile uint8_t *commonEvtsPtr;
    volatile uint8_t *ringStatePtr;

    volatile uint16_t stateMachine;
    volatile uint32_t port1EvtsFlag;
    volatile uint8_t commonEvtsFlag;
    volatile uint8_t ringState;

    volatile uint8_t node_state;
    volatile uint8_t lastPortBcnRcvd;

    volatile uint8_t *loopDetectedPtr;
    ICSS_EMAC_IoctlCmd ioctlParams;

    if(NODE_IDLE == dlrHandle->dlrObj->SMVariables.node_state)
    {
        /*If link broken*/
        if(!linkStatus)
        {
            /*Since there is a link break update loop in network status*/
            loopDetectedPtr = (uint8_t *)(pruicssHwAttrs->sharedDramBase +
                                          DLR_LOOP_DETECTED_OFFSET);
            /*TODO: Review this. FALSE_VAL -> 0*/
            *(loopDetectedPtr) =  0;
            dlrHandle->dlrObj->SMVariables.status = NORMAL_STAT;
            dlrHandle->checkForLoop = DLR_FALSE;

        }

        else
        {
            /*Check link status of ICSS_EMAC_PORT_1*/
            // if(((ICSS_EmacObject *)dlricssEmacHandle->object)->linkStatus[ICSS_EMAC_PORT_1 - 1])
            tempLinkStatus = 0;
            retVal =  MDIO_phyLinkStatus(pruicssHwAttrs->miiMdioRegBase,
                                        ((ICSS_EMAC_Attrs *)(dlricssEmacHandle->attrs))->phyAddr[ICSS_EMAC_PORT_1 - 1]);
            if(retVal == SystemP_SUCCESS)
                tempLinkStatus = 1;

            if(tempLinkStatus)
            {
                dlrHandle->checkForLoop = DLR_TRUE;
            }

            else
            {
                dlrHandle->checkForLoop = DLR_FALSE;
            }
        }

        return;
    }

    /*Load port 1 events flag*/
    port1EvtsPtr = (uint32_t *)(pruicssHwAttrs->pru1DramBase +
                                DLR_PORT_EVENTS_OFFSET);
    port1EvtsFlag = *(port1EvtsPtr);

    /*Load the state machine*/
    stateMachinePtr = (uint16_t *)(pruicssHwAttrs->sharedDramBase +
                                   DLR_STATE_MACHINE_OFFSET);
    stateMachine = *(stateMachinePtr);

    ringStatePtr = (uint8_t *)(pruicssHwAttrs->sharedDramBase +
                               DLR_STATE_MACHINE_OFFSET + 2);
    ringState = *(ringStatePtr);

    /*Load common events flag*/
    commonEvtsPtr = (uint8_t *)(pruicssHwAttrs->sharedDramBase +
                                DLR_STATE_MACHINE_OFFSET + 3);
    commonEvtsFlag = *(commonEvtsPtr);

    node_state = stateMachine & 0xff;
    lastPortBcnRcvd = (uint8_t)((stateMachine >> 8) & 0xff);

    if(DLR_FALSE == linkStatus)     /*link 1 down*/
    {

        /*In case state machine is still running with both links disabled*/
        /*Check link status of ICSS_EMAC_PORT_1*/
        // if(((ICSS_EmacObject *)dlricssEmacHandle->object)->linkStatus[ICSS_EMAC_PORT_1 - 1])
        tempLinkStatus = 0;
        retVal =  MDIO_phyLinkStatus(pruicssHwAttrs->miiMdioRegBase,
                                    ((ICSS_EMAC_Attrs *)(dlricssEmacHandle->attrs))->phyAddr[ICSS_EMAC_PORT_1 - 1]);
        if(retVal == SystemP_SUCCESS)
            tempLinkStatus = 1;

        if(tempLinkStatus)
        {
            EIP_DLR_resetStateMachine(dlrHandle);
            return;
        }

        /*Fault state*/
        if(NODE_FAULT == node_state)
        {
#ifdef DLR_DEBUG
            genSeqOfEvents(LINK2_FAULT_BREAK);
#endif

            /*check last port on which beacon was received*/
            if(lastPortBcnRcvd == DLR_PORT1)
            {
                /*Stop neighbor checkout timers*/
                ClockP_stop(&(dlrHandle->dlrNeighborTimeoutClock[ICSS_EMAC_PORT_1 - 1]));
                ClockP_stop(&(dlrHandle->dlrNeighborTimeoutClock[ICSS_EMAC_PORT_2 - 1]));

                /*if it's the same port then go to idle state*/
                EIP_DLR_resetStateMachine(dlrHandle);
            }

            else
            {
                /*send a link status message to active
                 * ring supervisor on link 0 that link 1 has failed */
                EIP_DLR_genNeighborLinkStatFrame(dlrHandle, dlrHandle->dlrEmptyFrame, DLR_PORT0,
                                                 IS_A_LINK_STATUS_FRAME,
                                                 DLR_PORT1_FAIL);
                // ICSS_EmacTxPacketEnqueue(dlricssEmacHandle, dlrHandle->dlrEmptyFrame, DLR_PORT0,
                //                          1,
                //                          DEFAULT_DLR_PACKET_SIZE);
                txArg.icssEmacHandle = dlricssEmacHandle;
                txArg.lengthOfPacket = DEFAULT_DLR_PACKET_SIZE;
                txArg.portNumber = DLR_PORT0;
                txArg.queuePriority = 1;
                txArg.srcAddress = dlrHandle->dlrEmptyFrame;

                ICSS_EMAC_txPacket(&txArg, NULL);
            }
        }

        /*Normal state*/
        if(NODE_NORMAL == node_state)
        {
#ifdef DLR_DEBUG
            genSeqOfEvents(LINK2_NORMAL_BREAK);
#endif
            /*set to fault*/
            node_state = NODE_FAULT;
            EIP_DLR_disable_WD_IEP(dlrHandle, PORT1_WATCH_DOG_ID);

            port1EvtsFlag &= ~DLR_TIMER_RUNNING_MASK;
            *(port1EvtsPtr) = port1EvtsFlag;

            /*flush learning table*/
            ioctlParams.command = ICSS_EMAC_LEARN_CTRL_CLR_TABLE;
            ICSS_EMAC_ioctl(dlricssEmacHandle, ICSS_EMAC_IOCTL_LEARNING_CTRL,
                           ICSS_EMAC_PORT_1, (void *)&ioctlParams);
            ICSS_EMAC_ioctl(dlricssEmacHandle, ICSS_EMAC_IOCTL_LEARNING_CTRL,
                           ICSS_EMAC_PORT_2, (void *)&ioctlParams);
            stateMachine = 0;
            ringState = 0;
            lastPortBcnRcvd = DLR_PORT0;
            stateMachine |= (node_state & 0xff);
            ringState |= DLR_RING_FAULT_MASK;
            stateMachine |= (lastPortBcnRcvd << 8);
            *(stateMachinePtr) = stateMachine;
            *(ringStatePtr) = ringState;

            commonEvtsFlag |= DLR_PORT1_BEACON_RCVD_MASK;
            commonEvtsFlag &= ~DLR_PORT0_BEACON_RCVD_MASK;

            *(commonEvtsPtr) = commonEvtsFlag;

            /*send link status frame to active ring supervisor*/
            EIP_DLR_genNeighborLinkStatFrame(dlrHandle, dlrHandle->dlrEmptyFrame, DLR_PORT0,
                                             IS_A_LINK_STATUS_FRAME,
                                             DLR_PORT1_FAIL);
            // ICSS_EmacTxPacketEnqueue(dlricssEmacHandle, dlrHandle->dlrEmptyFrame, DLR_PORT0,
            //                          1,
            //                          DEFAULT_DLR_PACKET_SIZE);
            txArg.icssEmacHandle = dlricssEmacHandle;
            txArg.lengthOfPacket = DEFAULT_DLR_PACKET_SIZE;
            txArg.portNumber = DLR_PORT0;
            txArg.queuePriority = 1;
            txArg.srcAddress = dlrHandle->dlrEmptyFrame;

            ICSS_EMAC_txPacket(&txArg, NULL);
            /*change CIP object variables*/
            EIP_DLR_switchToFault(dlrHandle);
        }
    }
}
/**
*  @internal
*  @brief  Initialize DLR control variables in the DRAM
*
*          This is defined separately because this needs to be called as part of
*           the switch initialization process after PRU code has been loaded
*
*  @param  dlrHandle [in] DLR handle
*
*  @retval  none
*
*/
void EIP_DLR_dRAMInit(EIP_DLRHandle dlrHandle)
{

    volatile uint16_t *SMPointer;
    volatile uint8_t *commonEventsPtr;
    volatile uint8_t *ringStatePtr;
    volatile uint32_t *port0EventsPtr;
    volatile uint32_t *port1EventsPtr;
    volatile uint8_t *signOnRcvdStatus;
    volatile uint32_t *activeSupMacAddr;

    PRUICSS_HwAttrs const *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(dlrHandle->pruicssHandle->hwAttrs);

    /*read the DLR stats flag*/
    signOnRcvdStatus = (uint8_t *)(pruicssHwAttrs->sharedDramBase +
                                   DLR_SIGNON_PKT_RCVD_OFFSET);
    SMPointer = (uint16_t *)(pruicssHwAttrs->sharedDramBase +
                             DLR_STATE_MACHINE_OFFSET);
    ringStatePtr = (uint8_t *)(pruicssHwAttrs->sharedDramBase +
                               DLR_STATE_MACHINE_OFFSET + 2);
    commonEventsPtr = (uint8_t *)(pruicssHwAttrs->sharedDramBase +
                                  DLR_STATE_MACHINE_OFFSET + 3);
    port0EventsPtr = (uint32_t *)(pruicssHwAttrs->pru0DramBase +
                                  DLR_PORT_EVENTS_OFFSET);
    port1EventsPtr = (uint32_t *)(pruicssHwAttrs->pru1DramBase +
                                  DLR_PORT_EVENTS_OFFSET);
    activeSupMacAddr = (uint32_t *)(pruicssHwAttrs->sharedDramBase +
                                    DLR_ACTIVE_SUP_MAC_0123);

    *(SMPointer) = 0;
    *(ringStatePtr) = 1;
    *(commonEventsPtr) = 0;
    *(port0EventsPtr) = 0;
    *(port1EventsPtr) = 0;
    *(signOnRcvdStatus) = 0;

    *(activeSupMacAddr) = 0;
    activeSupMacAddr = (uint32_t *)(pruicssHwAttrs->sharedDramBase +
                                    DLR_ACTIVE_SUP_MAC_45);
    *(activeSupMacAddr) = 0;

}
/**
*  @brief  Initialize the state machine when it goes back to idle state
*
*  @param  dlrHandle [in] DLR handle
*
*  @retval  none
*
*/
void EIP_DLR_resetStateMachine(EIP_DLRHandle dlrHandle)
{

#ifdef DLR_DEBUG
    genSeqOfEvents(DLR_RESET_MACHINE);
#endif

    ICSS_EMAC_Handle dlricssEmacHandle = dlrHandle->emacHandle;

    ICSS_EMAC_IoctlCmd ioctlParams;
    /*stop the timers*/
    EIP_DLR_disable_WD_IEP(dlrHandle, PORT0_WATCH_DOG_ID);
    EIP_DLR_disable_WD_IEP(dlrHandle, PORT1_WATCH_DOG_ID);

    /*clear memory*/
    EIP_DLR_dRAMInit(dlrHandle);

    /*Set default values*/
    EIP_DLR_setDefaultValue(dlrHandle);

    /*flush the learning table*/
    ioctlParams.command = ICSS_EMAC_LEARN_CTRL_CLR_TABLE;
    ICSS_EMAC_ioctl(dlricssEmacHandle, ICSS_EMAC_IOCTL_LEARNING_CTRL,
                   ICSS_EMAC_PORT_1, (void *)&ioctlParams);
    ICSS_EMAC_ioctl(dlricssEmacHandle, ICSS_EMAC_IOCTL_LEARNING_CTRL,
                   ICSS_EMAC_PORT_2, (void *)&ioctlParams);
}

/**
*  @internal
*  @brief  Set Default values for the DLR CIP Object
*
*  @param  dlrHandle [in] DLR handle
*
*  @retval  none
*
*/
void EIP_DLR_setDefaultValue(EIP_DLRHandle dlrHandle)
{

    /*Set default values*/

    dlrHandle->dlrObj->dlrCapabilities = DLR_DEFAULT_CAPABILITIES;
    dlrHandle->dlrObj->supConfig.superEnable = FALSE;
    dlrHandle->dlrObj->SMVariables.node_state = NODE_IDLE;
    dlrHandle->dlrObj->SMVariables.ring_state = RING_FAULT;
    dlrHandle->dlrObj->SMVariables.topology = LINEAR_TOP;
    dlrHandle->dlrObj->SMVariables.superStatus = RING_NODE;
    dlrHandle->dlrObj->SMVariables.status = NORMAL_STAT;

    dlrHandle->dlrObj->addr.supIPAddress = 0;
    memset(dlrHandle->dlrObj->addr.supMACAddress, 0x0, 6);

#ifdef IS_A_DLR_SUPERVISOR
    dlrHandle->dlrObj->numRingFaultsPowerUp = 0;
    dlrHandle->dlrObj - .ringParticipantsCount = 0;
#endif
}

/**
*  @internal
*  @brief   Set CIP Object values for fault status
*
*  @param  dlrHandle [in] DLR handle
*
*  @retval  none
*
*/
void EIP_DLR_switchToFault(EIP_DLRHandle dlrHandle)
{

    dlrHandle->dlrObj->SMVariables.node_state = NODE_FAULT;
    dlrHandle->dlrObj->SMVariables.ring_state = RING_FAULT;
    dlrHandle->dlrObj->SMVariables.topology = RING_TOP;
    dlrHandle->dlrObj->SMVariables.status = RING_FAULT_STAT;
}
/**
*  @internal
*  @brief   Set CIP Object values for normal status
*
*  @param  dlrHandle [in] DLR handle
*
*  @retval  none
*
*/
void EIP_DLR_switchToNormal(EIP_DLRHandle dlrHandle)
{
    PRUICSS_HwAttrs const *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(dlrHandle->pruicssHandle->hwAttrs);

    volatile uint8_t *loopDetectedPtr;
    loopDetectedPtr = (uint8_t *)(pruicssHwAttrs->sharedDramBase +
                                  DLR_LOOP_DETECTED_OFFSET);

    *(loopDetectedPtr) = DLR_TRUE;

    dlrHandle->dlrObj->SMVariables.node_state = NODE_NORMAL;
    dlrHandle->dlrObj->SMVariables.ring_state = RING_NORMAL;
    dlrHandle->dlrObj->SMVariables.topology = RING_TOP;
    dlrHandle->dlrObj->SMVariables.status = NORMAL_STAT;
}

/**
*  @internal
*  @brief Copies sequence id into byte stream in big-endian byte order
*
*  @param  src    [in] Byte stream pointer
*  @param  header [in] Byte stream header pointer
*
*  @retval  none
*
*/
void EIP_DLR_initDLRFrameHeader(uint8_t *src, uint8_t *header)
{

    /*processing common to all frames*/
    memcpy(src + DLR_COMMON_FRAME_OFFSET, header, DLR_COMMON_FRAME_HEADER_SIZE);
}

/*Helper Functions*/

/**
*  @internal
*  @brief   Add number of nodes to the sign on frame
*
*  @param  src      [in] pointer to stream where IP Addr is to be added
*  @param  numNodes [in] number of nodes in the ring
*
*  @retval  none
*
*/
void EIP_DLR_addSignOnNumNodes(uint8_t *src, uint16_t numNodes)
{

    *(src + DLR_SIGNON_NUM_NODES_OFFSET) = (numNodes >> 8) & 0xff;
    *(src + DLR_SIGNON_NUM_NODES_OFFSET + 1) = numNodes & 0xff;
}

/**
*  @internal
*  @brief   Generates a Neighbor check request frame
*
*           When a node receives a locate fault request from the supervisor
*           it sends neighbor check request frames out of both ports to get the
*           status of it's neighbors, if the neighbor doesn't respond it sends
*           a link status frame to the supervisor
*
*  @param  dlrHandle  [in] DLR handle
*  @param  src        [in] pointer to the packet
*  @param  sourcePort [in] port on which the frame will be sent
*
*  @retval  none
*
*/
void EIP_DLR_genNCReqFrame(EIP_DLRHandle dlrHandle, uint8_t *src,
                           uint8_t sourcePort)
{

    dlrHandle->sequenceID++;
    EIP_DLR_addWord(src + DLR_SEQUENCE_ID_OFFSET, dlrHandle->sequenceID);

    EIP_DLR_addMACID(src, DLR_NCREQ_MAC_ID);

    addSourcePort(src, sourcePort);

    addFrameOffset(src, DLR_NCREQ_FRAME_TYPE);
    memset(src + DLR_NCREQ_RESERVED_BYTES_OFFSET, 0x0,
           DLR_NCREQ_RESERVED_BYTES_SIZE);

}
/**
*  @internal
*  @brief   Generates a Neighbor check response frame
*
*           A neighbor check response frame is sent in response to a
*           neighbor request frame
*
*  @param  src        [in] pointer to the packet
*  @param  sourcePort [in] port on which the frame will be sent
*  @param  reqSrcPort [in] request source port of the neighbor check request
*  @param  sequenceId [in] sequence Id of the neighbor check request
*
*  @retval  none
*
*/
void EIP_DLR_genNCResFrame(uint8_t *src, uint8_t sourcePort, uint8_t reqSrcPort,
                           uint32_t sequenceId)
{

    EIP_DLR_addWord(src + DLR_SEQUENCE_ID_OFFSET, sequenceId);

    /*Add MAC identifier*/
    EIP_DLR_addMACID(src, DLR_NCREQ_MAC_ID);

    addSourcePort(src, sourcePort);
    addReqSrcPort(src, reqSrcPort);

    addFrameOffset(src, DLR_NCRES_FRAME_TYPE);
    memset(src + DLR_NCRES_RESERVED_BYTES_OFFSET, 0x0,
           DLR_NCRES_RESERVED_BYTES_SIZE);
}
/**
*  @internal
*  @brief   Generates a Link/Neighbor status frame
*
*           A neighbor check response frame is sent in response to a
*           neighbor request frame
*
*  @param  dlrHandle      [in] DLR handle
*  @param  src            [in] pointer to the packet
*  @param  sourcePort     [in] port on which the frame will be sent
*  @param  linkOrNeighbor [in] whether the packet is a link status or a neighbor status frame
*  @param  linkStatus     [in] indicates which link is down. Possible values are DLR_PORT0_FAIL or DLR_PORT1_FAIL
*
*  @retval  none
*
*/
void EIP_DLR_genNeighborLinkStatFrame(EIP_DLRHandle dlrHandle, uint8_t *src,
                                      uint8_t sourcePort,
                                      uint8_t linkOrNeighbor, uint8_t linkStatus)
{

    uint8_t status = 0;

    activeSuperAddr *actSupAddrPtr = &(dlrHandle->dlrObj->addr);

    dlrHandle->sequenceID++;
    EIP_DLR_addWord(src + DLR_SEQUENCE_ID_OFFSET, dlrHandle->sequenceID);

    /*This packet is meant for the ring supervisor*/
    EIP_DLR_addMACID(src, actSupAddrPtr->supMACAddress);

    /*if port 1 has failed it means port 0 is active*/
    if(DLR_PORT1_FAIL == linkStatus)
    {
        status |= 0x1;
    }

    else
    {
        status |= 0x2;
    }

    /*if it's a link status frame, set the appropriate bits*/
    if(!linkOrNeighbor)
    {
        status |= 0x80;
    }

    addLinkStat(src, status);
    addSourcePort(src, 0);

    addFrameOffset(src, DLR_LINKSTAT_FRAME_TYPE);
    memset(src + DLR_LSTAT_RESERVED_BYTES_OFFSET, 0x0,
           DLR_LSTAT_RESERVED_BYTES_SIZE);

}
/**
*  @internal
*  @brief   Add the new VLAN ID to the message
*
*  @param  src    [in] pointer to the packet
*  @param  vlanID [in] VLAN ID of supervisor
*
*  @retval  none
*
*/
void EIP_DLR_addVlanID(uint8_t *src, uint16_t vlanID)
{

    uint16_t tagPlusVlan = 0xE000;
    volatile uint8_t *bytePtr;

    bytePtr = (uint8_t *)(src + DLR_START_OF_VLAN_ID);

    tagPlusVlan += vlanID;

    *(bytePtr) = ((tagPlusVlan >> 8) & 0xff);
    *(bytePtr + 1) = (tagPlusVlan & 0xff);

}
/**
*  @internal
*  @brief  Initialize DLR ISR's
*
*  @param  dlrHandle [in] DLR handle
*
*  @retval 0 - On success
*          <0 - On failure
*
*/
int32_t EIP_DLR_isrInit(EIP_DLRHandle dlrHandle)
{
    HwiP_Params hwiParams;
    uint32_t status = SystemP_FAILURE;

    HwiP_Params_init(&hwiParams);

    hwiParams.intNum = dlrHandle->dlrObj->port0IntNum;
    hwiParams.callback = (HwiP_FxnCallback)EIP_DLR_port0ISR;
    hwiParams.args = (void *)dlrHandle;
    /*TODO: evtId is not available. Review the impact*/
    // hwiParams.evtId = dlrHandle->dlrObj->port0IntNum;
    status = HwiP_construct(&(dlrHandle->dlrObj->port0IntObject), &hwiParams);

    if(status == SystemP_SUCCESS)
    {
        HwiP_enableInt(dlrHandle->dlrObj->port0IntNum);

        hwiParams.intNum = dlrHandle->dlrObj->port1IntNum;
        hwiParams.callback = (HwiP_FxnCallback)EIP_DLR_port1ISR;
        hwiParams.args = (void *)dlrHandle;
        /*TODO: evtId is not available. Review the impact*/
        // hwiParams.evtId = dlrHandle->dlrObj->port1IntNum;
        status = HwiP_construct(&(dlrHandle->dlrObj->port1IntObject), &hwiParams);
    }

    if(status == SystemP_SUCCESS)
    {
        HwiP_enableInt(dlrHandle->dlrObj->port1IntNum);

        hwiParams.intNum = dlrHandle->dlrObj->beaconTimeoutIntNum_P0;
        hwiParams.callback = (HwiP_FxnCallback)EIP_DLR_beaconTimeoutISR_P0;
        hwiParams.args = (void *)dlrHandle;
        /*TODO: evtId is not available. Review the impact*/
        // hwiParams.evtId = dlrHandle->dlrObj->beaconTimeoutIntNum_P0;
        status = HwiP_construct(&(dlrHandle->dlrObj->beaconTimeoutIntP0Object), &hwiParams);
    }

    if(status == SystemP_SUCCESS)
    {
        HwiP_enableInt(dlrHandle->dlrObj->beaconTimeoutIntNum_P0);

        hwiParams.intNum = dlrHandle->dlrObj->beaconTimeoutIntNum_P1;
        hwiParams.callback = (HwiP_FxnCallback)EIP_DLR_beaconTimeoutISR_P1;
        hwiParams.args = (void *)dlrHandle;
        /*TODO: evtId is not available. Review the impact*/
        // hwiParams.evtId = dlrHandle->dlrObj->beaconTimeoutIntNum_P1;
        status = HwiP_construct(&(dlrHandle->dlrObj->beaconTimeoutIntP1Object), &hwiParams);
    }

    if(status == SystemP_SUCCESS)
    {
        HwiP_enableInt(dlrHandle->dlrObj->beaconTimeoutIntNum_P1);
    }
    return status;
}
/**
*  @internal
*  @brief Add a macId to the exception list
*
*  @param dlrHandle [in] DLR handle
*  @param macId     [in] pointer to MAC ID
*
*  @retval none
*
*/
void EIP_DLR_addToExceptionList(EIP_DLRHandle dlrHandle, uint8_t *macId)
{

    dlrHandle->exclusionList->numExceptions = 1;

    /*TODO: Review this*/
    // ICSS_EMAC_copyMAC(dlrHandle->exclusionList->MAC, macId);
    memcpy((void *)(dlrHandle->exclusionList->MAC), (void *)macId, 6);
}
/**
*  @internal
*  @brief    Add a macId to the exception list
*
*  @param dlrHandle [in] DLR handle
*
*  @retval none
*
*/
void EIP_DLR_clearExceptionList(EIP_DLRHandle dlrHandle)
{
    /*do a dirty deletion*/
    dlrHandle->exclusionList->numExceptions = 0;
}

/**
*  @internal
*  @brief
*
*  @param macId     [in] pointer to MAC ID
*  @param dlrHandle [in] DLR handle
*
*  @retval
*
*/
uint8_t EIP_DLR_checkSupervisorException(uint8_t *macId,
        EIP_DLRHandle dlrHandle)
{
    /*check the exclusion list*/
    if(dlrHandle->exclusionList->numExceptions)
    {
        /*if the source mac id matches the exclusion list return*/
        /*TODO: Review this*/
        // if(COMPARE_MAC(dlrHandle->exclusionList->MAC,  macId))
        if(memcmp((void *)(dlrHandle->exclusionList->MAC), (void *)(macId), 6U) == 0)
        {
            return 0;
        }

        else
        {
            return 1;
        }

    }

    return 1;
}

void EIP_DLR_periodicProcessing(ClockP_Object *obj, void *userArg)
{

    volatile uint8_t ifLoopExists;
    volatile uint8_t *loopDetectedPtr;
    volatile uint32_t *wordPtr;
    volatile uint8_t *commonEvtsPtr;
    volatile uint8_t commonEvtsFlag;
    uint8_t currSupMACAddress[6];

    EIP_DLRHandle dlrHandle = (EIP_DLRHandle)userArg;
    ICSS_EMAC_Handle dlricssEmacHandle = dlrHandle->emacHandle;
    PRUICSS_HwAttrs const *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(dlrHandle->pruicssHandle->hwAttrs);
    ICSS_EMAC_TxArgument txArg;

    activeSuperAddr *actSupAddrPtr = &(dlrHandle->dlrObj->addr);

    /*update the current supervisor MAC id*/
    wordPtr = (uint32_t *)(pruicssHwAttrs->sharedDramBase +
                           DLR_ACTIVE_SUP_MAC_0123);
    EIP_DLR_getMACId((uint8_t *)wordPtr, currSupMACAddress);

    if(0 != memcmp(currSupMACAddress, actSupAddrPtr->supMACAddress, 6))
    {
        /* Workaround for the case where the SUP mac is not updated*/
        EIP_DLR_disable_WD_IEP(dlrHandle, PORT0_WATCH_DOG_ID);
        EIP_DLR_disable_WD_IEP(dlrHandle, PORT1_WATCH_DOG_ID);
        EIP_DLR_updateSupervisorDetails(dlrHandle);
        EIP_DLR_enable_WD_IEP(dlrHandle, PORT0_WATCH_DOG_ID);
        EIP_DLR_enable_WD_IEP(dlrHandle, PORT1_WATCH_DOG_ID);
    }

    /*Update Supervisor IP address periodically. This is required if the Supervisor IP is changed after state change */
    wordPtr = (uint32_t *)(pruicssHwAttrs->sharedDramBase +
                           DLR_ACTIVE_SUP_IP_OFFSET);
    actSupAddrPtr->supIPAddress = EIP_DLR_convBigEndianToLittleEndianWord((
                                      uint8_t *)wordPtr);

    commonEvtsPtr = (uint8_t *)(pruicssHwAttrs->sharedDramBase +
                                DLR_STATE_MACHINE_OFFSET + 3);
    commonEvtsFlag = *(commonEvtsPtr);

    if((((dlrHandle->stateMachineCount)++ & 0x7) == 0))
    {
        if(commonEvtsFlag & DLR_RESET_EVENT_OCCURED_MASK)
        {
            /*clear the reset event flag*/
            commonEvtsFlag &= ~(DLR_RESET_EVENT_OCCURED_MASK);
            /*write back*/
            *(commonEvtsPtr) = commonEvtsFlag;
            EIP_DLR_resetStateMachine(dlrHandle);
        }

        dlrHandle->stateMachineCount = 0;
    }

    if(NODE_IDLE != dlrHandle->dlrObj->SMVariables.node_state)
    {
        return;
    }

    /*Find out if a loop has been detected*/
    loopDetectedPtr = (uint8_t *)(pruicssHwAttrs->sharedDramBase +
                                  DLR_LOOP_DETECTED_OFFSET);
    ifLoopExists = *(loopDetectedPtr);

    /*Check is unnecessary if DLR state machine is not running*/
    /*If there is a loop and there is no supervisor set the flag*/
    if(ifLoopExists)
    {
        dlrHandle->dlrObj->SMVariables.status = UNEXPECTED_LOOP;
    }
    else
    {
        dlrHandle->dlrObj->SMVariables.status = NORMAL_STAT;
    }

    /*Once both ports are connected and Ring is not operational, the device sends three packets on both ports at an interval of 1 second*/
    if((dlrHandle->checkForLoop) && ((dlrHandle->pktSendCounter) < 3))
    {
        dlrHandle->tracePktIntervalCount++;

        if((((dlrHandle->tracePktIntervalCount) & 0xf) == 0)
                || (0 == (dlrHandle->pktSendCounter)))    /*modulo 16 or interval of 1.6 seconds*/
        {
            // ICSS_EmacTxPacketEnqueue(dlricssEmacHandle, tracePkt, ICSS_EMAC_PORT_1, 1,
            //                          ETHERNET_FRAME_SIZE_60);
            // ICSS_EmacTxPacketEnqueue(dlricssEmacHandle, tracePkt, ICSS_EMAC_PORT_2, 1,
            //                          ETHERNET_FRAME_SIZE_60);

            txArg.icssEmacHandle = dlricssEmacHandle;
            txArg.lengthOfPacket = ETHERNET_FRAME_SIZE_60;
            txArg.portNumber = ICSS_EMAC_PORT_1;
            txArg.queuePriority = 1;
            txArg.srcAddress = tracePkt;

            ICSS_EMAC_txPacket(&txArg, NULL);

            txArg.icssEmacHandle = dlricssEmacHandle;
            txArg.lengthOfPacket = ETHERNET_FRAME_SIZE_60;
            txArg.portNumber = ICSS_EMAC_PORT_2;
            txArg.queuePriority = 1;
            txArg.srcAddress = tracePkt;
            ICSS_EMAC_txPacket(&txArg, NULL);
            /*reset the counter*/
            dlrHandle->tracePktIntervalCount = 0;
            dlrHandle->pktSendCounter++;
        }
    }

    else
    {
        /*Handle the case when the physical ring got disconnected after the firmware reported Unexpected
         * loop detected. Checks for the state and sends packets periodically*/
        if(dlrHandle->dlrObj->SMVariables.status == UNEXPECTED_LOOP)
        {
            dlrHandle->tracePktIntervalCount++;
            if(((dlrHandle->tracePktIntervalCount) & 0xf) == 0)
            {
                // ICSS_EmacTxPacketEnqueue(dlricssEmacHandle, tracePkt, ICSS_EMAC_PORT_1, 1,
                //                          ETHERNET_FRAME_SIZE_60);
                // ICSS_EmacTxPacketEnqueue(dlricssEmacHandle, tracePkt, ICSS_EMAC_PORT_2, 1,
                //                          ETHERNET_FRAME_SIZE_60);
                txArg.icssEmacHandle = dlricssEmacHandle;
                txArg.lengthOfPacket = ETHERNET_FRAME_SIZE_60;
                txArg.portNumber = ICSS_EMAC_PORT_1;
                txArg.queuePriority = 1;
                txArg.srcAddress = tracePkt;

                ICSS_EMAC_txPacket(&txArg, NULL);

                txArg.icssEmacHandle = dlricssEmacHandle;
                txArg.lengthOfPacket = ETHERNET_FRAME_SIZE_60;
                txArg.portNumber = ICSS_EMAC_PORT_2;
                txArg.queuePriority = 1;
                txArg.srcAddress = tracePkt;
                ICSS_EMAC_txPacket(&txArg, NULL);
                /*reset the counter*/
                dlrHandle->tracePktIntervalCount = 0;
                loopDetectedPtr = (uint8_t *)(pruicssHwAttrs->sharedDramBase +
                                              DLR_LOOP_DETECTED_OFFSET);
                /*TODO: Review FALSE_VAL -> 0*/
                *(loopDetectedPtr) =  0;
            }
        }
        else
        {
            dlrHandle->pktSendCounter = 0;
            dlrHandle->checkForLoop = DLR_FALSE;
        }
    }

}

#ifdef DLR_DEBUG
void genSeqOfEvents(uint8_t event)
{
    if(seqNumber < MAX_EVENTS_CAPTURED)
    {
        sequenceOfEvents[seqNumber++] = event;
    }
}
#endif
/**
*  @brief  Add a IP address to DLR
*
*  @param dlrHandle  [in] DLR handle
*  @param newIP      [in] IP Address in unsigned int 32 format
*
*  @retval  none
*
*/
void EIP_DLR_addModuleIPAddress(EIP_DLRHandle dlrHandle, uint32_t newIP)
{
    dlrHandle->deviceIP = newIP;
    EIP_DLR_addWord(dlrHandle->dlrEmptyFrame + DLR_SOURCE_IP_OFFSET, dlrHandle->deviceIP);
    EIP_DLR_addWord(learnUpdateFrame + DLR_LEARN_UPDATE_IP_OFFSET, dlrHandle->deviceIP);
}

/**
*  @brief  Sets the clock divider to 1us for IEP watch dog timers
*
*  @param dlrHandle  [in] DLR handle
*
*  @retval  none
*
*/
void EIP_DLR_setDivider_WD_IEP(EIP_DLRHandle dlrHandle)
{
    uintptr_t iepBaseAddress = (((PRUICSS_HwAttrs const *)(dlrHandle->pruicssHandle->hwAttrs))->iep0RegBase);

    /*Configure pre-divider for 1us increment value*/
    HW_WR_REG16(iepBaseAddress + CSL_ICSS_G_PR1_IEP0_SLV_WD_PREDIV_REG, IEP_WD_PRE_DIV_10US);
}

/**
*  @brief  Enable the IEP Watch dog timers
*
*  @param dlrHandle  [in] DLR handle
*  @param id         [in] 0/1 for WD_PD and WD_PDI respectively
*
*  @retval  none
*
*/
void EIP_DLR_enable_WD_IEP(EIP_DLRHandle dlrHandle, uint8_t id)
{
    uintptr_t iepBaseAddress = (((PRUICSS_HwAttrs const *)(dlrHandle->pruicssHandle->hwAttrs))->iep0RegBase);

    /*Make sure we are not overwriting Watchdog enable bit for the other port*/
    uint32_t enableFlag = HW_RD_REG32(iepBaseAddress +  CSL_ICSS_G_PR1_IEP0_SLV_WD_CTRL_REG);

    if(PORT0_WATCH_DOG_ID == id)
    {
        enableFlag |= 1;
    }

    else
    {
        enableFlag |= (1 << 16);
    }

    EIP_DLR_setTimeout_WD_IEP(dlrHandle,
                              dlrHandle->dlrObj->supConfig.beaconInterval / 10, id);
    HW_WR_REG32(iepBaseAddress +  CSL_ICSS_G_PR1_IEP0_SLV_WD_CTRL_REG, enableFlag);
}

/**
*  @brief  Disable the IEP Watch dog timers
*
*  @param dlrHandle  [in] DLR handle
*  @param id         [in] 0/1 for WD_PD/WD_PDI respectively
*
*  @retval  none
*
*/
void EIP_DLR_disable_WD_IEP(EIP_DLRHandle dlrHandle, uint8_t id)
{
    uintptr_t iepBaseAddress = (((PRUICSS_HwAttrs const *)(dlrHandle->pruicssHandle->hwAttrs))->iep0RegBase);

    uint32_t regValue = 0;
    uint32_t enableFlag = 0;

    regValue = HW_RD_REG32(iepBaseAddress +  CSL_ICSS_G_PR1_IEP0_SLV_WD_CTRL_REG);

    if(PORT0_WATCH_DOG_ID == id)
    {
        enableFlag |= 1;
        HW_WR_REG16(iepBaseAddress +  CSL_ICSS_G_PR1_IEP0_SLV_PD_WD_TIM_REG, 0);
    }

    else
    {
        enableFlag |= (1 << 16);
        HW_WR_REG16(iepBaseAddress +  CSL_ICSS_G_PR1_IEP0_SLV_PDI_WD_TIM_REG, 0);
    }

    /*clear the bit*/
    regValue &= ~(enableFlag);

    HW_WR_REG32(iepBaseAddress +  CSL_ICSS_G_PR1_IEP0_SLV_WD_CTRL_REG, regValue);
}

/**
*  @brief  Set the timeout value in watchdog
*
*  @param dlrHandle  [in] DLR handle
*  @param periodInMicroSec [in] Timeout value in microseconds
*  @param id [in] watch dog ID
*
*  @retval  none
*
*/
void EIP_DLR_setTimeout_WD_IEP(EIP_DLRHandle dlrHandle,
                               uint16_t periodInMicroSec, uint8_t id)
{
    uintptr_t iepBaseAddress = (((PRUICSS_HwAttrs const *)(dlrHandle->pruicssHandle->hwAttrs))->iep0RegBase);

    if(PORT0_WATCH_DOG_ID == id)
    {
        HW_WR_REG16(iepBaseAddress +  CSL_ICSS_G_PR1_IEP0_SLV_PD_WD_TIM_REG, periodInMicroSec);
    }

    else
    {
        HW_WR_REG16(iepBaseAddress +  CSL_ICSS_G_PR1_IEP0_SLV_PDI_WD_TIM_REG, periodInMicroSec);
    }

}

void EIP_DLR_set_pdi_wd_trigger_mode(EIP_DLRHandle dlrHandle, uint32_t mode)
{
    /* PDI WD trigger also depends on DIGIO h/w events in ICSS and by default triggered on RX SOF events on both ports
       This function can be used to override this behaviour in h/w - it is not possible to disable DIGIO dependency and make
       PDI WD trigger manually in firmware for every command send - so allow user to configure - trigger mechansim
       LATCH_IN  event or SYNC0/1_out - New application default is LATCH_IN event */

    uintptr_t iepBaseAddress = (((PRUICSS_HwAttrs const *)(dlrHandle->pruicssHandle->hwAttrs))->iep0RegBase);

    HW_WR_REG32(iepBaseAddress +  CSL_ICSS_G_PR1_IEP0_SLV_DIGIO_CTRL_REG, mode);
}

void EIP_DLR_updateSupervisorDetails(EIP_DLRHandle dlrHandle)
{
    volatile uint32_t *wordPtr;
    volatile uint32_t regVal;
    activeSuperAddr *actSupAddrPtr = &(dlrHandle->dlrObj->addr);
    PRUICSS_HwAttrs const *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(dlrHandle->pruicssHandle->hwAttrs);

    wordPtr = (uint32_t *)(pruicssHwAttrs->sharedDramBase +
                           DLR_BEACON_TIMEOUT_OFFSET);
    /*read beacon timeout value*/
    dlrHandle->dlrObj->supConfig.beaconTimeout = *(wordPtr);

    EIP_DLR_setTimeout_WD_IEP(dlrHandle,
                              dlrHandle->dlrObj->supConfig.beaconTimeout / 10, PORT0_WATCH_DOG_ID);
    EIP_DLR_setTimeout_WD_IEP(dlrHandle,
                                  dlrHandle->dlrObj->supConfig.beaconTimeout / 10, PORT1_WATCH_DOG_ID);

    wordPtr = (uint32_t *)(pruicssHwAttrs->sharedDramBase +
                               DLR_SCALED_BEACON_TIMEOUT);
    *(wordPtr) = dlrHandle->dlrObj->supConfig.beaconTimeout / 10;

    wordPtr = (uint32_t *)(pruicssHwAttrs->sharedDramBase +
                               DLR_BEACON_INTERVAL_OFFSET);
    /*read beacon timeout value*/
    dlrHandle->dlrObj->supConfig.beaconInterval = *(wordPtr);

    wordPtr = (uint32_t *)(pruicssHwAttrs->sharedDramBase +
                               DLR_VLAN_ID_SUPID_OFFSET);
    regVal = *(wordPtr);
    /*read beacon timeout value*/
    dlrHandle->dlrObj->supConfig.vLanId = (uint16_t)(regVal & 0xffff);
    dlrHandle->dlrObj->supConfig.supPrecedence = (uint16_t)((
                    regVal & 0xff0000) >> 16);
    dlrHandle->dlrObj->activeSuperPred = dlrHandle->dlrObj->supConfig.supPrecedence;

    EIP_DLR_addVlanID(dlrHandle->dlrEmptyFrame,
                          dlrHandle->dlrObj->supConfig.vLanId);

    /*update the current supervisor MAC id*/
    wordPtr = (uint32_t *)(pruicssHwAttrs->sharedDramBase +
                               DLR_ACTIVE_SUP_MAC_0123);
    EIP_DLR_getMACId((uint8_t *)wordPtr, actSupAddrPtr->supMACAddress);
    EIP_DLR_addToExceptionList(dlrHandle, actSupAddrPtr->supMACAddress);

    wordPtr = (uint32_t *)(pruicssHwAttrs->sharedDramBase +
                               DLR_ACTIVE_SUP_IP_OFFSET);
    actSupAddrPtr->supIPAddress = EIP_DLR_convBigEndianToLittleEndianWord((
                                          uint8_t *)wordPtr);

}

void EIP_DLR_addWord(uint8_t *src, uint32_t word)
{
    *(src) = (word >> 24) & 0xff;
    *(src + 1) = (word >> 16) & 0xff;
    *(src + 2) = (word >> 8) & 0xff;
    *(src + 3) = word & 0xff;
}

void EIP_DLR_addMACID(uint8_t *src, uint8_t *macID)
{
    *(src) = *(macID);
    *(src + 1) = *(macID + 1);
    *(src + 2) = *(macID + 2);
    *(src + 3) = *(macID + 3);
    *(src + 4) = *(macID + 4);
    *(src + 5) = *(macID + 5);
}

void EIP_DLR_getMACId(uint8_t *packet, uint8_t *macId)
{
    uint8_t *src;
    uint8_t i;
    src = packet;

    //conversion from big endian byte order to little endian byte order
    for(i = 0; i < 6; i++)
    {
        *(macId + i) = *(src++);
    }
}

uint32_t EIP_DLR_convBigEndianToLittleEndianWord(uint8_t *byte)
{
    uint8_t *src;
    uint32_t word = 0;
    src = byte;

    /*conversion from big endian byte order to little endian byte order*/
    word |= *(src++);
    word <<= 8;
    word |= *(src++);
    word <<= 8;
    word |= *(src++);
    word <<= 8;
    word |= *(src++);

    return word;
}

uint16_t EIP_DLR_convBigEndianToLittleEndianHalfWord(uint8_t *byte)
{
    uint8_t *src;
    uint32_t halfWord = 0;
    src = byte;

    //conversion from big endian byte order to little endian byte order
    halfWord |= *(src++);
    halfWord <<= 8;
    halfWord |= *(src++);

    return halfWord;
}
