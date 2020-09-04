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

#ifndef PN_HANDLE_DEF_H_
#define PN_HANDLE_DEF_H_

#ifdef __cplusplus
extern "C"
{
#endif


/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "pnDrvConfig.h"
#include "iPtcpDrv.h"
#include "iPtcpUtils.h"
#include "iRtcDrv2.h"
#include "iPnOs.h"
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/TaskP.h>
#include <stdint.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/*TODO: Review this*/
#define PN_TASK_STACK_SIZE (2048)

typedef void (*PN_getMACAddress)(uint8_t index, uint8_t *lclMac);

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */
/*
*  \brief     PN_PtcpDebug
*             Structure storing the debug info for PTCP
*/
typedef struct PN_PtcpDebug_s
{
    /*! Debug delay info      */
    debugDelay_t debugDelay[PTCP_NUM_PORTS];
    /*! Debug delay index      */
    int32_t debugDelayIndex;
    /*! Debug sync index       */
    int32_t debugSyncIndex;
    /*! Debug max deltaT       */
    int32_t maxDeltaT;
    /*! Debug min deltaT       */
    int32_t minDeltaT;
    /*! Debug deltaT our of range      */
    int32_t deltaTOutOfRange;
    /*! Debug max delay        */
    int32_t maxDelay;
    /*! Debug min delay        */
    int32_t minDelay;
    /*! Debug delay out of range       */
    int32_t delayOutOfRange;
    /*! Debug Sync Miss counter        */
    uint32_t syncmissCounter;
} PN_PtcpDebug;

/*
*  \brief     PN_TimerAttrs
*             Structure storing the interrupt configurations
*/
typedef struct PN_PtcpTimerAttrs_s
{
    /*! Timer ID */
    uint32_t ptcpTimerID;
    /*! Timer handle used in Sync processing */
    void *ptcpTimerHandle;
} PN_PtcpTimerAttrs;

/*TODO: Review this*/
/*
*  \brief     PN_PtcpConfig
*             Structure storing the PTCP info
*/
typedef struct PN_PtcpConfig_s
{
    /*! PTCP Cycle period                 */
    uint32_t pnCyclePeriod;
    /*! Mandatory counter adjustment for inter delay request gap        */
    uint32_t mandCtrAdj;
    /*! T2 previous                       */
    uint32_t T2_prev[PTCP_NUM_PORTS];
    /*! T1 previous                       */
    uint32_t T1_prev[PTCP_NUM_PORTS];
    /*! T1 cycle counter              */
    uint32_t T1_CTR_prev[PTCP_NUM_PORTS];
    /*! Peer rate compensation factor                     */
    float rcf_prev[PTCP_NUM_PORTS];
    /*! Sequence ID                       */
    uint16_t seqId;
    /*! Cable delay and line delay index                      */
    int32_t delayIndex;
    /*! Sync Index                        */
    int32_t syncIndex;
    /*! Number of Sync packets since PTCP reset                       */
    uint32_t numInSync;
    /*! Delta values used in SMA filter.                      */
    int32_t syncDeltaTs[SYNC_FILTER_SIZE];
    /*! Original time of master clock - in Nanosecs                   */
    uint32_t *pSyncTorgNs;
    /*! Sync in delay + line delay            */
    uint32_t *pSyncInDelayPlusLD;
    /*! Sync RX SOF                       */
    uint32_t *pSyncRxSOF;
    /*! Original time of master clock - in Secs               */
    uint32_t *pSyncTorgSecs;
    /*! Cycle counter                     */
    uint16_t *pPmCycleCounter;
    /*! Phase counter                     */
    uint16_t *pPmPhaseCounter;
    /*! Sub domain UUID                   */
    uint8_t *pSubDomainUUID;
    /*! Master MAC                        */
    uint8_t *pSyncMasterMac;
    /*! Init Flag                         */
    uint8_t *pSyncInitFlag;
    /*! Clock change notification delay   */
    uint32_t clkChangeNotifyDelay;
    /*! Previous cable delay              */
    uint32_t prev_cDelay[PTCP_NUM_PORTS];
    /*! Previous line Delay               */
    uint32_t prev_lDelay[PTCP_NUM_PORTS];
    /*! Init Cycle counter done           */
    uint8_t initPmCycleCtrDone;
    /*! init cycle counter pending        */
    uint8_t cycleCtrInitPending;
    /*! calculated cycle counter          */
    uint16_t calculatedCycleCtr;
    /*! Master change                     */
    uint8_t masterChange;
    /*! phase counter change              */
    uint8_t phaseCtrChange;
    /*! First delay burst                 */
    int32_t firstDelayBurst;
    /*! Max seq id                        */
    uint16_t maxSeqId;
    /*! Min seq id                        */
    uint16_t minSeqId;
    /*! Sync Info                         */
    volatile ptcpSyncInfo_t deviceSyncInfo;
    /*! SYNC Start configuration                  */
    uint32_t ptcpSync0PinStart;
    /*! SYNC Pulse width configuration            */
    uint32_t ptcpSync0PinPulseWidth;
    /*! Current status                    */
    volatile currentPtcpStatus_t currentPtcpStatus;
    /*! Port delays                       */
    ptcpPortDelayVal_t portTimes[PTCP_NUM_PORTS];
    /*! Device delays                     */
    deviceDelays_t deviceDelays[PTCP_NUM_PORTS];
    /*! Port offsets                      */
    devicePortOffsets_t devicePortOffsets[PTCP_NUM_PORTS];
    /*! Debug delay out of range                  */
    /* Semaphores */
    /*! Semaphore to start PTCP                   */
    SemaphoreP_Object ptcpStartSem;
    /*! Semaphore to trigger measurement          */
    SemaphoreP_Object ptcpTriggerMeasurementSem;
    /* Callbacks */
    /*! Callback to update sync status            */
    ptcpCallBack_t ptcpSyncStatusCall;
    /*! Callback to update delay                  */
    ptcpCallBack_t ptcpDelayUpdateCall;
    /*TODO: Check if this should be always included*/
    /* Debug                                      */
    /*! Debug information                         */
    PN_PtcpDebug pnPtcpDebugAttrs;
    LatchVars   g_Latch;
    PNIO_TimeStamp  g_T1;
    uint8_t ptcpSyncFilterfactor;
    /*! Set to 1 if SOC supports slow compensation */
    uint8_t ptcpEnableSlowCompensation;
    /* Timer for PTCP */
    PN_PtcpTimerAttrs ptcpTimer;
} PN_PtcpConfig;


typedef struct PN_IsoMConfig_s
{
    /*! Interrupt handle */
    HwiP_Object isoMInterruptObject;
    /*! Flag to indicate Interrupt is created. Used incase of common interrupt for Ti and To */
    uint8_t isoMIntCreateFlag;
    /*! Flag to indicate whether we have common inettrupt for Ti and To */
    uint8_t isoMNumEvents;
    /*! isoM interrupt details */
    PN_IntAttrs isoMIntConfig;

    uint32_t event1Handle;

    uint32_t event2Handle;

}PN_IsoMConfig;

/*TODO: Review the changes here*/
/*
*  \brief     PN_Config
*             Profinet handle - stores the Profinet configuration
*/
typedef struct PN_Config_s
{
    /*! Flag to indicate RTC driver init done              */
    uint8_t initRtcDrvFlag;
    /*! Flag to indicate RTC Memory config init done       */
    uint8_t initRtcMemFlag;
    /*! CPM list data        */
    t_descList cpmList;
    /*! PPM list data        */
    t_descList ppmList;
    /*! Main structure storing Profinet config info       */
    t_cfgPN currPN;
    /*! Block of PPM Buffer memory in DataRam0/1          */
    t_ppmBlock ppmBlock[PTCP_NUM_PORTS];
    /*! CPM buffer block - L3 SRAM                        */
    t_cpmBlock cpmBlock;
    /*! State for MRP state machine. Controls the state machine           */
    tMrpStates mrpState;
    /*! IRT LEGACY mode state, controls the state machine. Initialized to \ref NOINIT         */
    tLegStates legState;
    /*! PPM information for packet to be send by the IRT LEGACY mode state machine            */
    t_rtcPacket *pLegPkt;
    /*! Callback that updates the MRP state machine                                           */
    pnLegCallBack_t
    irtLegStateCall; /* Required? Just sets the value for legState(above), can be done directly? */
    /*! Callback that defines the packet send by the MRP state machine                        */
    pnLegCallBack_t irtLegPktCall;
    /*! Watchdog timer period                 */
    int32_t icssWachDogTimerPeriod;
    /*! Watchdog enabled Flag                 */
    int32_t icssWatchDogEnabled;
    /*! ICSSEMAC Handle                   */
    ICSS_EMAC_Handle emacHandle;
    /*TODO: Review this*/
    /*! PRUICSS LLD Handle                   */
    PRUICSS_Handle pruicssHandle;
    /*! Profinet Interrupt configuration      */
    PN_IntConfig pnIntConfig;
    /*! PTCP Configuration                    */
    PN_PtcpConfig pnPtcpConfig;
    /* Task Handles */
    /*! PTCP Task Handle                      */
    TaskP_Object PTCPTaskObject;
    /*! PTCP Sync monitor task handle         */
    TaskP_Object SyncMonitorTaskObject;
    /*! Legacy mode task handle               */
    TaskP_Object LegModeTaskObject;
    /*! MRP Task handle                       */
    TaskP_Object MrpMachineTaskObject;
    /*! Watchdog timer Task handle            */
    TaskP_Object WatchDogTimerTaskObject;
    /*TODO: Review this change*/
    /*! ISOM handle                           */
    uint32_t pnIsoMObject;
    /*! ISOM config                           */
    PN_IsoMConfig pnIsoMConfig;
    /*TODO: Review this*/
    /*! Function pointer to get MAC address   */
    PN_getMACAddress getMACAddress;
    /*TODO: Review this*/
    SemaphoreP_Object switchReady;
#ifdef PTCP_SUPPORT
    uint32_t PTCP_taskStack[PN_TASK_STACK_SIZE/sizeof(uint32_t)]            __attribute__((aligned(32)));
    uint32_t PTCP_syncMonitorTaskStack[PN_TASK_STACK_SIZE/sizeof(uint32_t)] __attribute__((aligned(32)));
#endif /*PTCP_SUPPORT*/
#ifdef IRT_LEGACY_STARTUP_SUPPORT
    uint32_t IRT_legacyTaskStack[PN_TASK_STACK_SIZE/sizeof(uint32_t)]       __attribute__((aligned(32)));
#endif /*IRT_LEGACY_STARTUP_SUPPORT*/
#ifdef MRP_SUPPORT
    uint32_t MRP_CPMTaskStack[PN_TASK_STACK_SIZE/sizeof(uint32_t)]          __attribute__((aligned(32)));
#endif /*MRP_SUPPORT*/
#ifdef WATCHDOG_SUPPORT
    uint32_t tapWatchDog_taskStack[PN_TASK_STACK_SIZE/sizeof(uint32_t)]     __attribute__((aligned(32)));
#endif /*WATCHDOG_SUPPORT*/

} PN_Config;


#ifdef __cplusplus
}
#endif

#endif /* PN_HANDLE_DEF_H_ */
