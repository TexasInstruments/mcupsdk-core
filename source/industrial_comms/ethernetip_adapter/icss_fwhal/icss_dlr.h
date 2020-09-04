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

#ifndef ICSS_DLR_H_
#define ICSS_DLR_H_

#ifdef __cplusplus
extern "C"
{
#endif


/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>
#include <networking/icss_emac/icss_emac.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/HwiP.h>

#ifdef TEST_DEBUG
#include "testing.h"
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
/*#define DLR_DEBUG*/

/**For future use when Ring Supervisor is implemented*/
/*#define IS_A_DLR_SUPERVISOR*/

#define DEFAULT_DLR_PACKET_SIZE         60

#define ETHERNET_FRAME_SIZE_60 60

/**Value written to IEP WD Predivider register to configure 1us increment*/
#define IEP_WD_PRE_DIV_10US     2000

/**ID for PD Watchdog in IEP*/
#define PORT0_WATCH_DOG_ID   0      /*PD_WD*/
/**ID for PDI Watchdog in IEP*/
#define PORT1_WATCH_DOG_ID   1      /*PDI_WD*/

#define IS_A_LINK_STATUS_FRAME      DLR_TRUE
#define IS_A_NEIGHBOR_STAT_FRAME    DLR_FALSE

/*Generic flags*/
#define BOTH_LINKS_UP           0x0
#define PORT0_IS_DOWN           0x2
#define PORT1_IS_DOWN           0x1
#define BOTH_LINKS_DOWN         0x3

/*PRUSS INTC Mask for PRU EVENT 5*/
#define PORT0_WD_ISR_MASK       0x2000000
/*PRUSS INTC Mask for PRU EVENT 5*/
#define PORT1_WD_ISR_MASK       0x4000000

/**
* \def PDI_WD_TRIGGER_RX_SOF
*  Watchdog RX Start of Frame Trigger
*/
#define PDI_WD_TRIGGER_RX_SOF       (0 << 4)

/**
* \def PDI_WD_TRIGGER_LATCH_IN
*  Watchdog LATCH IN Trigger
*/
#define PDI_WD_TRIGGER_LATCH_IN     (1 << 4)

/**
* \def PDI_WD_TRIGGER_SYNC0_OUT
*  Watchdog SYNC0 Trigger
*/
#define PDI_WD_TRIGGER_SYNC0_OUT    (2 << 4)

/**
* \def PDI_WD_TRIGGER_SYNC1_OUT
*  Watchdog SYNC1 Trigger
*/
#define PDI_WD_TRIGGER_SYNC1_OUT    (3 << 4)

#ifdef DLR_DEBUG
#define MAX_EVENTS_CAPTURED     1000

/*DLR events for debugging*/
#define LINK1_BREAK     1
#define LINK2_BREAK     2
#define BOTH_LINK_DISABLED  3
#define EMPTY_SIGNON_FRAME_RCVD 4
#define COMPLETE_SIGNON_FRAME_RCVD  5

#define RING_FAULT_RCVD_PORT0   6
#define RING_FAULT_RCVD_PORT1   7

#define RING_FAULT_TRANSITION_PORT0 8
#define RING_FAULT_TRANSITION_PORT1 9

#define RING_NORMAL_TRANSITION_PORT0    10
#define RING_NORMAL_TRANSITION_PORT1    11

#define START_TIMER0            12
#define START_TIMER1            13

#define NCRES_RCVD_PORT0        14
#define NCRES_RCVD_PORT1        15

#define NCREQ_RCVD_PORT0        16
#define NCREQ_RCVD_PORT1        17

#define LOCFAULT_RCVD_PORT0     18
#define LOCFAULT_RCVD_PORT1     19

#define BEACON0_MISSED_FAULT    20
#define BEACON1_MISSED_FAULT    21
#define BEACON0_MISSED_NORMAL   22
#define BEACON1_MISSED_NORMAL   23

#define LINK1_FAULT_BREAK   24
#define LINK2_FAULT_BREAK   25

#define LINK1_NORMAL_BREAK      27
#define LINK2_NORMAL_BREAK      28

#define NEIGHBOR_TIMEOUT_PORT0_RETRY    29
#define NEIGHBOR_TIMEOUT_PORT1_RETRY    30
#define NEIGHBOR_TIMEOUT_PORT0_MAX      31
#define NEIGHBOR_TIMEOUT_PORT1_MAX      32

#define DLR_RESET_MACHINE   33

#define BEACON0_MISSED  34
#define BEACON1_MISSED  35

#define STOP_BOTH_TIMERS_PORT0 36
#define STOP_BOTH_TIMERS_PORT1 37

#define PORT0_BEACON_STALL   38
#define PORT1_BEACON_STALL   39
#endif

/**DLR Timer default values, this is redundant since values are eventually read from supervisor*/
#define DEFAULT_BEACON_INTERVAL_VARIABLE 400 /*in microseconds*/
#define DEFAULT_BEACON_TIMEOUT_VARIABLE (DEFAULT_BEACON_INTERVAL_VARIABLE * 4) /*in microseconds*/

#define DEFAULT_NEIGHBOR_TIMEOUT_INTERVAL   100 /*in milliseconds*/

#define DEFAULT_DLR_PERIODIC_INTERVAL   100 /*in milliseconds*/

/**Number of neighbor retries before a neighbor status message is sent*/
#define MAX_NUM_RETRIES     2

/**Number of times beacon timer is triggered before it is disabled*/
#define BEACON_CPU_STALL_THRESHOLD  4

/**the value 1 indicates that this is only a
 * ring node capable device, for other values see below*/

#define DLR_DEFAULT_CAPABILITIES    (1 << 7) | (1 << 1)

/**
* \def DLR_SIGNON_FRAME_SIZE
*  If size of packet is greater than DLR_SIGNON_FRAME_SIZE in
*  \ref EIP_DLR_processDLRFrame, then the frame is sent directly to host.
*  Value pf "ICSS_EMAC_MAXMTU - 10U" is used because 10 bytes space in the frame
*  is needed so that the own MAC and IP addresses can be added.
*/
#define DLR_SIGNON_FRAME_SIZE           (ICSS_EMAC_MAXMTU - 10U)

/*DLR Packet Generation Offsets*/
#define DLR_COMMON_FRAME_HEADER_SIZE    18
#define DLR_COMMON_FRAME_OFFSET         12

/**DLR port 0 Interrupt Flag for ICSS INTC*/
#define ICSS_DLR_PORT0_INT_FLAG              0x200000
/**DLR port 1 Interrupt Flag for ICSS INTC*/
#define ICSS_DLR_PORT1_INT_FLAG              0x400000

typedef struct dlr_Config_s *EIP_DLRHandle;

/**
 * \brief node state machine states
 */
typedef enum
{
    /**idle state*/
    NODE_IDLE = 0,
    /**fault state*/
    NODE_FAULT = 1,
    /**normal state*/
    NODE_NORMAL = 2
} nodeState;
/**
 * \brief ring state values
 */
typedef enum
{
    /**normal ring*/
    RING_NORMAL = 1,
    /**faulty ring*/
    RING_FAULT = 2
} ringState;

/**
 * \brief network topology : possible values for Attribute ID 1
 */
typedef enum
{
    /**linear topology*/
    LINEAR_TOP = 0,
    /**ring topology*/
    RING_TOP = 1
} nwTopology;

/**
 * \brief network status : possible values for Attribute ID 2
 */
typedef enum
{
    /**normal status*/
    NORMAL_STAT = 0,
    /**ring fault status*/
    RING_FAULT_STAT = 1,
    /**unexpected loop detected*/
    UNEXPECTED_LOOP = 2,
    /**ring is partially faulty*/
    PARTIAL_FAULT = 3,
    /**there is rapid connection/reset*/
    RAPID_FAULT = 4
} nwStatus;

/**
 * \brief device role: possible values for Attribute ID 3
 */
typedef enum
{
    /*indicates the node is functioning as a backup*/
    BACKUP_NODE = 0,
    /*indicates the device is functioning as the active ring supervisor*/
    ACTIVE_RING_SUPERVISOR = 1,
    /*indicates the device is functioning as a normal ring node*/
    RING_NODE = 2,
    /*indicates the device is operating in a non-DLR topology*/
    NON_DLR = 3,
    /**indicates the device cannot support the currently operating
      ring parameters (Beacon Interval and/or Beacon Timeout)*/
    FEATURE_UNSUPPORTED = 4

} supervisorStatus;

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 * \brief List of MAC ID's which are exempted from Learning, this is for DLR implementation
 */
typedef struct
{
    /**MAC ID of supervisor which is exempt from learning*/
    uint8_t MAC[6];
    /**number of exceptions, this is a flag where value greater than 1 indicates that exception exists*/
    uint8_t numExceptions;

} exceptionList;

/**
 * \brief State machine variables, part of DLR Object and L2 implementation
 */
typedef struct
{
    /**device state. Internal*/
    uint8_t node_state;
    /**ring state. Internal*/
    uint8_t ring_state;
    /**network topology. Attribute ID 1*/
    uint8_t topology;
    /**network status. Attribute ID 2*/
    uint8_t status;
    /**supervisor status. Attribute ID 3*/
    uint8_t superStatus;

} dlrStateMachineVar;

/**
 * \brief Supervisor configuration. Attribute ID 4
 */
typedef struct
{

    /** device configured to run as supervisor
     * default : false
     */
    uint8_t  superEnable;
    /**beacon interval value (microseconds)*/
    uint32_t beaconInterval;
    /**beacon timeout value (microseconds)*/
    uint32_t beaconTimeout;

    /**VLAN Id*/
    uint16_t vLanId;

    /**Supervisor precedence*/
    uint8_t supPrecedence;

} superConfig;

/**
 * \brief Supervisor address, part of DLR Object. Attribute ID 10
 */
typedef struct
{

    /**supervisors mac address*/
    uint8_t supMACAddress[6];

    /**supervisor ip address*/
    uint32_t supIPAddress;

} activeSuperAddr;

/**
 * \brief Last active node at the end of the chain
 * Class for Attributes 6 and 7
 */
typedef struct
{
    /**IP Address of the last active Node*/
    uint32_t ipAddr;
    /**MAC of the last node*/
    uint8_t macAddr[6];

    /**from 0-N where N is the number of nodes (excluding supervisor) in the ring*/
    uint16_t deviceNum;
} lastActiveNode;

/**
 * \brief IP and MAC of the Ring devices. Attribute ID 9
 */
typedef struct
{
    uint32_t ipAddr;
    uint8_t macAddr[6];
} protocolParticipants;

/**
 * \brief DLR parent structure through which all other structures can be accessed
 */
typedef struct
{
    /**Supervisor config. Attribute ID 4*/
    superConfig supConfig;
    /**Supervisor address. Attribute ID 10*/
    activeSuperAddr addr;
    /**State Machine variables. Attributes 1 through 3*/
    dlrStateMachineVar SMVariables;

#ifdef IS_A_DLR_SUPERVISOR
    /**Attribute ID's 6 and 7*/
    lastActiveNode activeNode[2];
#endif
    /**DLR Capabilities, flag
    The Map is as follows
     * 0 : Announce Based Ring Node
     * 1 : Beacon based Ring Node
     * 2-4 : Reserved
     * 5 : Supervisor capable
     * 6 : Redundant capable
     * 7 : Flush Table frame capable
     * 8-31 : Reserved
     */
    /**DLR Capabilities of the device. Attribute ID 12*/
    uint32_t dlrCapabilities;

    /**Active Supervisor precedence. Attribute ID 11*/
    uint8_t activeSuperPred;

    HwiP_Object port0IntObject;
    HwiP_Object port1IntObject;
    HwiP_Object beaconTimeoutIntP0Object;
    HwiP_Object beaconTimeoutIntP1Object;

    /**DLR Port 0 Interrupt number for ARM*/
    uint8_t port0IntNum;

    /**DLR Port 1 Interrupt number for ARM*/
    uint8_t port1IntNum;

    /**DLR interrupt for beacon timeout for Port 0*/
    uint8_t beaconTimeoutIntNum_P0;

    /**DLR interrupt for beacon timeout for Port 1*/
    uint8_t beaconTimeoutIntNum_P1;

#ifdef IS_A_DLR_SUPERVISOR
    /**Number of Ring Faults since power up. Attribute ID 5*/
    uint32_t numRingFaultsPowerUp;

    /**Number of ring participants. Attribute ID 8*/
    uint16_t ringParticipantsCount;

    /**pointer to array of structures. Attribute ID 9*/
    protocolParticipants **ringNodes;
#endif

} dlrStruct;


typedef struct dlr_Config_s
{
    dlrStruct           *dlrObj;
    ICSS_EMAC_Handle    emacHandle;
    uint8_t             macId[6];
    /* MAC ID passed to ICSS-EMAC during ICSS-EMAC initialization*/
    PRUICSS_Handle      pruicssHandle;
    /**Device IP*/
    uint32_t            deviceIP;
    /**Sequence ID as per DLR spec*/
    uint32_t            sequenceID;
    /**Learning table exception for MAC ID*/
    exceptionList       *exclusionList;
    /**Clock handle for neighbor timeout timers**/
    ClockP_Object       dlrNeighborTimeoutClock[ICSS_EMAC_MAX_PORTS_PER_INSTANCE];
    /*Clock handle for DLR periodic processing*/
    ClockP_Object       dlrPeriodicTimerObject;
    /**Buffer used for General DLR Messages **/
    uint8_t             dlrEmptyFrame[DEFAULT_DLR_PACKET_SIZE];
    /**Variable which keeps count of whether loop exists*/
    uint8_t             checkForLoop;
    uint32_t            tracePktIntervalCount;
    uint32_t            stateMachineCount;
    uint8_t             pktSendCounter;
    uint8_t             ISRcountPort[ICSS_EMAC_MAX_PORTS_PER_INSTANCE];
} dlr_Config;

/* ========================================================================== */
/*                       Function Declarations                                */
/* ========================================================================== */

/** \addtogroup EIP_DLR
 @{ */

/**
*  \brief  API to initialize the DLR driver
*
*          Initializes variables and timers & clocks, call once at the beginning
*
*  \param  dlrHandle [in] DLR driver handle
*
*/
void EIP_DLR_init(EIP_DLRHandle dlrHandle);
/**
*  \brief  API to de-initialize the DLR driver
*
*  \param  dlrHandle [in] DLR driver handle
*
*/
void EIP_DLR_deinit(EIP_DLRHandle dlrHandle);
/**
*  \brief  API to start the DLR driver
*          Calling this enables DLR on the device
*
*  \param  dlrHandle [in] DLR driver handle
*
*/
void EIP_DLR_start(EIP_DLRHandle dlrHandle) ;
/**
*  \brief  API to stop the DLR driver
*          Halt DLR. Calling this disables DLR on the device
*
*  \param  dlrHandle [in] DLR driver handle
*
*/
void EIP_DLR_stop(EIP_DLRHandle dlrHandle);
/**
*  \brief  Fast ISR for Port 0, bypasses the buffer copy and NDK
*
*  \param  arg [in] user argument. DLR handle
*
*/
void EIP_DLR_port0ISR(uintptr_t arg);
/**
*  \brief  Fast ISR for Port 1, bypasses the buffer copy and NDK
*
*  \param  arg [in] user argument. DLR handle
*
*/
void EIP_DLR_port1ISR(uintptr_t arg);
/**
*  \brief  ISR for beacon timeout for Port 0
*
*  \param  arg [in] user argument. DLR handle
*
*/
void EIP_DLR_beaconTimeoutISR_P0(uintptr_t arg);

/**
*  \brief  ISR for beacon timeout for Port 1
*
*  \param  arg [in] user argument. DLR handle
*
*/
void EIP_DLR_beaconTimeoutISR_P1(uintptr_t arg);

/**
*  \brief Process DLR state machine in the event of a link break on Port0
*
*  \param  linkStatus [in] link status of the port. Up/Down. 1/0
*  \param  arg2       [in] argument DLR handle
*
*/
void EIP_DLR_port0ProcessLinkBrk(uint8_t linkStatus, void *arg2);
/**
*  \brief Process DLR state machine in the event of a link break on Port1
*
*  \param  linkStatus [in] link status of the port. Up/Down. 1/0
*  \param  arg2       [in] argument DLR handle
*
*/
void EIP_DLR_port1ProcessLinkBrk(uint8_t linkStatus, void *arg2);
/**
*  \brief ISR for Neighbor timeout timer for port 0
*
*  \param   obj [in] Clock object associated with this callback
*  \param   arg [in] user argument. DLR handle
*
*/
void EIP_DLR_neighborTimeoutISR0(ClockP_Object *obj, void *arg);
/**
*  \brief ISR for Neighbor timeout timer for port 1
*
*  \param   obj [in] Clock object associated with this callback
*  \param   arg [in] user argument. DLR handle
*
*/
void EIP_DLR_neighborTimeoutISR1(ClockP_Object *obj, void *arg);
/**
*  \internal
*  \brief   Add the new VLAN ID to the message
*
*  \param  src    [in] pointer to the packet
*  \param  vlanID [in] VLAN ID of supervisor
*
*/
void EIP_DLR_addVlanID(uint8_t *src, uint16_t vlanID);
/**
*  \internal
*  \brief   Generates a Neighbor check request frame
*
*           When a node receives a locate fault request from the supervisor
*           it sends neighbor check request frames out of both ports to get the
*           status of it's neighbors, if the neighbor doesn't respond it sends
*           a link status frame to the supervisor
*
*  \param  dlrHandle  [in] DLR handle
*  \param  src        [in] pointer to the packet
*  \param  sourcePort [in] port on which the frame will be sent
*
*/
void EIP_DLR_genNCReqFrame(EIP_DLRHandle dlrHandle, uint8_t *src,
                           uint8_t sourcePort);
/**
*  \internal
*  \brief   Generates a Neighbor check response frame
*
*           A neighbor check response frame is sent in response to a
*           neighbor request frame
*
*  \param  src        [in] pointer to the packet
*  \param  sourcePort [in] port on which the frame will be sent
*  \param  reqSrcPort [in] request source port of the neighbor check request
*  \param  sequenceId [in] sequence Id of the neighbor check request
*
*/
void EIP_DLR_genNCResFrame(uint8_t *src, uint8_t sourcePort, uint8_t reqSrcPort,
                           uint32_t sequenceId);
/**
*  \internal
*  \brief   Generates a Link/Neighbor status frame
*
*           A neighbor check response frame is sent in response to a
*           neighbor request frame
*
*  \param  dlrHandle      [in] DLR handle
*  \param  src            [in] pointer to the packet
*  \param  sourcePort     [in] port on which the frame will be sent
*  \param  linkOrNeighbor [in] whether the packet is a link status or a neighbor status frame
*  \param  linkStatus     [in] indicates which link is down. Possible values are DLR_PORT0_FAIL or DLR_PORT1_FAIL
*
*/
void EIP_DLR_genNeighborLinkStatFrame(EIP_DLRHandle dlrHandle, uint8_t *src,
                                      uint8_t sourcePort,
                                      uint8_t linkOrNeighbor, uint8_t linkStatus);
/**
*  \internal
*  \brief Copies sequence id into byte stream in big-endian byte order
*
*  \param  src    [in] Byte stream pointer
*  \param  header [in] Byte stream header pointer
*
*/
void EIP_DLR_initDLRFrameHeader(uint8_t *src, uint8_t *header);

/**
*  \internal
*  \brief   Add number of nodes to the sign on frame
*
*  \param  src      [in] pointer to stream where IP Addr is to be added
*  \param  numNodes [in] number of nodes in the ring
*
*/
void EIP_DLR_addSignOnNumNodes(uint8_t *src, uint16_t numNodes);
/**
*  \brief Processes a sign on and Neighbor check request frame.
*
*         In case of sign on frames, if size of packet is greater than
*         \ref DLR_SIGNON_FRAME_SIZE then the frame is sent directly to host.
*         Therefore the 'size' argument must be the size of the incoming frame,
*         because it is used as the size of the outgoing frame, only if the
*         frame already has maximum size and is sent unchanged but directed to
*         the supervisor. Also, 'pktBuffer' passed to the function must provide
*         'size'+10 bytes space so that the own MAC and IP addresses can be
*         added.
*
*  \param  dlrHandle [in] DLR handle
*  \param  pktBuffer [in] pointer to bytestream
*  \param  portNum   [in] Port number wherer the packet arrived
*  \param  size      [in] size of packet
*
*/
void EIP_DLR_processDLRFrame(EIP_DLRHandle dlrHandle, uint8_t *pktBuffer,
                             uint8_t portNum, uint16_t size);
/**
*  \internal
*  \brief  Initialize DLR control variables in the DRAM
*
*          This is defined separately because this needs to be called as part of
*           the switch initialization process after PRU code has been loaded
*
*  \param  dlrHandle [in] DLR handle
*
*/
void EIP_DLR_dRAMInit(EIP_DLRHandle dlrHandle);
/**
*  \brief  Initialize the state machine when it goes back to idle state
*
*  \param  dlrHandle [in] DLR handle
*
*/
void EIP_DLR_resetStateMachine(EIP_DLRHandle dlrHandle);
/**
*  \internal
*  \brief  Set Default values for the DLR CIP Object
*
*  \param  dlrHandle [in] DLR handle
*
*/
void EIP_DLR_setDefaultValue(EIP_DLRHandle dlrHandle);
/**
*  \internal
*  \brief   Set CIP Object values for fault status
*
*  \param  dlrHandle [in] DLR handle
*
*/
void EIP_DLR_switchToFault(EIP_DLRHandle dlrHandle);
/**
*  \internal
*  \brief   Set CIP Object values for normal status
*
*  \param  dlrHandle [in] DLR handle
*
*/
void EIP_DLR_switchToNormal(EIP_DLRHandle dlrHandle);
/**
*  \internal
*  \brief  Initialize DLR ISR's
*
*  \param  dlrHandle [in] DLR handle
*
*  \retval
*
*/
int32_t EIP_DLR_isrInit(EIP_DLRHandle dlrHandle);
/**
*  \internal
*  \brief   This is called periodically at DEFAULT_DLR_PERIODIC_INTERVAL
*
*  \param  obj      [in] Clock object associated with this callback
*  \param  userArg  [in] DLR handle is passed to this parameter
*
*  \retval
*
*/
void EIP_DLR_periodicProcessing(ClockP_Object *obj, void *userArg);

#ifdef DLR_DEBUG
void genSeqOfEvents(uint8_t event);
#endif
/**
*  \internal
*  \brief Add a macId to the exception list
*
*  \param dlrHandle [in] DLR handle
*  \param macId     [in] pointer to MAC ID
*
*  \retval none
*
*/
void EIP_DLR_addToExceptionList(EIP_DLRHandle dlrHandle, uint8_t *macId);
/**
*  \internal
*  \brief    Add a macId to the exception list
*
*  \param dlrHandle [in] DLR handle
*
*  \retval none
*
*/
void EIP_DLR_clearExceptionList(EIP_DLRHandle dlrHandle);

/**
*  \internal
*  \brief
*
*  \param macId     [in] pointer to MAC ID
*  \param dlrHandle [in] DLR handle
*
*  \retval
*
*/
uint8_t EIP_DLR_checkSupervisorException(uint8_t *macId,
        EIP_DLRHandle dlrHandle);
/**
*  \internal
*  \brief  Add a IP address to DLR
*
*  \param dlrHandle  [in] DLR handle
*  \param newIP      [in] IP Address in unsigned int 32 format
*
*/
void EIP_DLR_addModuleIPAddress(EIP_DLRHandle dlrHandle, uint32_t newIP);

/**
*  \brief  Sets the clock divider to 1us for IEP watch dog timers
*
*  \param dlrHandle  [in] DLR handle
*
*/
void EIP_DLR_setDivider_WD_IEP(EIP_DLRHandle dlrHandle);

/**
*  \brief  Enable the IEP Watch dog timers
*
*  \param dlrHandle  [in] DLR handle
*  \param id         [in] 0/1 for WD_PD and WD_PDI respectively
*
*/
void EIP_DLR_enable_WD_IEP(EIP_DLRHandle dlrHandle, uint8_t id);

/**
*  \brief  Disable the IEP Watch dog timers
*
*  \param dlrHandle  [in] DLR handle
*  \param id         [in] 0/1 for WD_PD/WD_PDI respectively
*
*/
void EIP_DLR_disable_WD_IEP(EIP_DLRHandle dlrHandle, uint8_t id);

/**
*  \brief  Set the timeout value in watchdog
*
*  \param dlrHandle  [in] DLR handle
*  \param periodInMicroSec [in] Timeout value in microseconds
*  \param id [in] watch dog ID
*
*/
void EIP_DLR_setTimeout_WD_IEP(EIP_DLRHandle dlrHandle,
                               uint16_t periodInMicroSec, uint8_t id);
/**
*  \brief  Set the PDI WD trigger mode
*
*  \param dlrHandle  [in] DLR handle
*  \param mode       [in] mode to be set in DGIO ctrl reg
*
*/
void EIP_DLR_set_pdi_wd_trigger_mode(EIP_DLRHandle dlrHandle, uint32_t mode);

/**
@}
*/

#ifdef __cplusplus
}
#endif

#endif /* ICSS_DLR_H_ */
