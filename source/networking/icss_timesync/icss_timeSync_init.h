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

#ifndef ICSS_TIMESYNC_INIT_H_
#define ICSS_TIMESYNC_INIT_H_

#ifdef __cplusplus
extern "C"
{
#endif


/**
 *  \defgroup NETWORKING_ICSS_TIMESYNC_MODULE APIs for ICSS TimeSync
 *  \ingroup NETWORKING_MODULE
 *
 *  APIs for PTP/1588 v2 slave implementation on PRU-ICSS
 *
 *  @{
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include "icss_timeSync.h"
#include "icss_timeSyncApi.h"
#include "icss_timeSync_osal.h"

/**
 * \def TS_LEAP_61_INDEX
 *      Index for Leap 61 status in ptp_flags[] array
 */
#define  TS_LEAP_61_INDEX                                   0
/**
 * \def TS_LEAP_59_INDEX
 *      Index for Leap 59 status
 */
#define  TS_LEAP_59_INDEX                                   1
/**
 * \def TS_UTC_REASONABLE_INDEX
 *      Index for UTC reasonable value in ptp_flags[] array
 */
#define  TS_UTC_REASONABLE_INDEX                            2
/**
 * \def TS_PTP_TIMESCALE_INDEX
 *      Index for value indicating whether PTP Timescale
 *      in ptp_flags[] array
 */
#define  TS_PTP_TIMESCALE_INDEX                             3
/**
 * \def TS_TIME_TRACEABLE_INDEX
 *      Index for value indicating whether
 *      time traceable in ptp_flags[] array
 */
#define  TS_TIME_TRACEABLE_INDEX                            4
/**
 * \def TS_FREQ_TRACEABLE_INDEX
 *      Index for value indicating whether
 *      freq traceable in ptp_flags[] array
 */
#define  TS_FREQ_TRACEABLE_INDEX                            5
/**
 * \def TS_ALTERNATE_MASTER_INDEX
 *      Index for value indicating whether
 *      alternate master in ptp_flags[] array
 */
#define  TS_ALTERNATE_MASTER_INDEX                          6
/**
 * \def TS_PTP_TWO_STEP_INDEX
 *      Index for value indicating whether
 *      two step sync in ptp_flags[] array
 */
#define  TS_PTP_TWO_STEP_INDEX                              7
/**
 * \def TS_PTP_UNICAST
 *      Index for value indicating whether
 *      messaging is unicast in ptp_flags[] array
 */
#define  TS_PTP_UNICAST                                     8
/**
 * \def TS_PROFILE_SPECIFIC_1_INDEX
 *      Index for value indicating alternate
 *      PTP Profile in ptp_flags[] array
 *
 */
#define  TS_PROFILE_SPECIFIC_1_INDEX                        9
/**
 * \def TS_PROFILE_SPECIFIC_2_INDEX
 *      Index for value indicating alternate
 *      PTP Profile in ptp_flags[] array
 *
 */
#define  TS_PROFILE_SPECIFIC_2_INDEX                        10
/**
 * \def TS_PTP_SECURITY_INDEX
 *  Index for value indicating whether PTP security
 *  is enabled or not in ptp_flags[] array
 */
#define  TS_PTP_SECURITY_INDEX                              11

/*------------Flag masks for different frames as per standard--------*/
/* These are derived indirectly from Table 20 of the standard*/
/**
 * \def TS_SYNC_BYTE0_MASK
 *  selects the flags relevant to Sync frame for
 *  first byte of the 2 byte flag
 */
#define  TS_SYNC_BYTE0_MASK                                 0xFF
/**
 * \def TS_ANNOUNCE_BYTE0_MASK
 *  selects the flags relevant to Announce frame for
 *  first byte of the 2 byte flag
 */
#define  TS_ANNOUNCE_BYTE0_MASK                             0xFD

/**
 * \def TS_FOLLOW_UP_BYTE0_MASK
 *  selects the flags relevant to Follow Up frame for
 *  first byte of the 2 byte flag
 */
#define  TS_FOLLOW_UP_BYTE0_MASK                            0xFD
/**
 * \def TS_PDELAY_RESP_BYTE0_MASK
 *  selects the flags relevant to Pdelay Resp frame for
 *  first byte of the 2 byte flag
 */
#define  TS_PDELAY_RESP_BYTE0_MASK                          0xFE
/**
 * \def TS_DELAY_RESP_BYTE0_MASK
 *  selects the flags relevant to Delay Resp frame for
 *  first byte of the 2 byte flag
 */
#define  TS_DELAY_RESP_BYTE0_MASK                           0xFD
/**
 * \def TS_OTHER_FRAMES_BYTE0_MASK
 *  selects the flags relevant to Pdelay Req
 *  and Delay Request for
 *  first byte of the 2 byte flag
 */
#define  TS_OTHER_FRAMES_BYTE0_MASK                         0xFC

/**
 * \def TIMESYNC_PDELAY_BUF_SIZE
 *  Size of peer delay buffers on DUT
 *  This includes peer delay request, response and respone follow up
 */
#define  TIMESYNC_PDELAY_BUF_SIZE                           68

/**
 * \def TIMESYNC_SYNC_BUF_ANNEX_F_SIZE
 *  Size of Sync buffers on DUT for Annex F mode
 */
#define  TIMESYNC_SYNC_BUF_ANNEX_F_SIZE                     58

/**
 * \def TIMESYNC_FOLLOW_UP_BUF_ANNEX_F_SIZE
 *  Size of Follow Up buffers on DUT for Annex F mode
 */
#define  TIMESYNC_FOLLOW_UP_BUF_ANNEX_F_SIZE                63

/**
 * \def TIMESYNC_ANNOUNCE_BUF_ANNEX_F_SIZE
 *  Size of Announce Up buffers on DUT for Annex F mode
 */
#define  TIMESYNC_ANNOUNCE_BUF_ANNEX_F_SIZE                 78

/**
 * \def TIMESYNC_SYNC_BUF_ANNEX_E_SIZE
 *  Size of Sync buffers on DUT for Annex E mode
 */
#define  TIMESYNC_SYNC_BUF_ANNEX_E_SIZE                     86

/**
 * \def TIMESYNC_FOLLOW_UP_BUF_ANNEX_E_SIZE
 *  Size of Follow Up buffers on DUT for Annex E mode
 */
#define  TIMESYNC_FOLLOW_UP_BUF_ANNEX_E_SIZE                86

/**
 * \def TIMESYNC_ANNOUNCE_BUF_ANNEX_E_SIZE
 *  Size of Announce Up buffers on DUT for Annex E mode.
 */
#define  TIMESYNC_ANNOUNCE_BUF_ANNEX_E_SIZE                 106

/**
 * \def TIMESYNC_DELAY_REQ_BUF_SIZE
 *  Size of Delay Request buffer on DUT (only applicable to E2E).
 */
#define  TIMESYNC_DELAY_REQ_BUF_SIZE                        86

/*-----------------Default PTP Master init params-------------------*/

/* These are used in BMCA*/
/**
 * \def Default Priority 1 value communicated in Announce frame
 */
#define  TIMESYNC_DEFAULT_PRIO_1                            128
/**
 * \def Default Priority 1 value communicated in Announce frame
 */
#define  TIMESYNC_DEFAULT_PRIO_2                            128
/**
 * \def Default clock accuracy communicated in Announce frame.
 */
#define  TIMESYNC_DEFAULT_CLOCK_ACCURACY                    0x31
/**
 * \def Default clock class communicated in Announce frame
 */
#define  TIMESYNC_DEFAULT_CLOCK_CLASS                       248
/**
 * \def Default clock variance from GM communicated in Announce frame
 */
#define  TIMESYNC_DEFAULT_CLOCK_VARIANCE                    0
/**
 * \def Default steps removed for Boundary clock
 */
#define  TIMESYNC_DEFAULT_STEPS_REMOVED                     0
/**
 * \def Default UTC offset
 */
#define  TIMESYNC_UTC_OFFSET                                0
/**
 * \def Default Time source used for clock. Set to Internal oscillator
 */
#define  TIMESYNC_DEFAULT_TIME_SOURCE                       0xa0


/**
 * \def Default drift threshold used in offset stabilization algo
 */
#define  TIMESYNC_OFFSET_STABLE_ALGO_THRESHOLD              15

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 * \brief Initializes variables and timers & clocks, call once at the beginning
 * \param timeSyncHandle pointer to PTP Handle structure
 * \return Error Value
 */
int8_t TimeSync_drvInit(TimeSync_ParamsHandle_t timeSyncHandle);

/**
* \brief Allocate Rx and Tx buffers for frames like Sync, Announce etc
* \param timeSyncHandle pointer to PTP Handle structure
* \return Error Value
*/
int8_t TimeSync_alloc_PktBuffer(TimeSync_ParamsHandle_t timeSyncHandle);

/**
* \brief Unallocate Rx and Tx buffers for frames like Sync, Announce etc
* \param timeSyncHandle pointer to PTP Handle structure
*/
void TimeSync_unAlloc_PktBuffer(TimeSync_ParamsHandle_t timeSyncHandle);

/**
 * \brief Enable PTP firmware
 * \param timeSyncHandle pointer to PTP Handle structure
 */
void TimeSync_drvEnable(TimeSync_ParamsHandle_t timeSyncHandle);

/**
 * \brief Disable PTP firmware
 * \param timeSyncHandle pointer to PTP Handle structure
 */
void TimeSync_drvDisable(TimeSync_ParamsHandle_t timeSyncHandle);

/**
 * \brief Initialize PTP stored variables in the DRAM
 * \param timeSyncHandle pointer to PTP Handle structure
 */
void TimeSync_dramInit(TimeSync_ParamsHandle_t timeSyncHandle);


/**
 * \brief Set Default values for the PTP Object based on clock type
 *
 * \param timeSyncHandle pointer to PTP Handle structure
 *
 */
void TimeSync_setDefaultValue(TimeSync_ParamsHandle_t timeSyncHandle);

/**
 * \brief Prepare PTP frames with fields, mac addresses etc.
 * \param timeSyncHandle pointer to PTP Handle structure
 * \param  ifMacID pointer to MAC ID
 */
void TimeSync_formatPTPFrames(TimeSync_ParamsHandle_t timeSyncHandle,
                              uint8_t *ifMacID);
/**
 * \brief Populate MAC ID, clock identity etc in Follow Up frame.
 *        Should be called everytime we change to Master from Slave
 * \param timeSyncHandle pointer to PTP Handle structure
 */
void TimeSync_initializeFollowUp(TimeSync_ParamsHandle_t timeSyncHandle);

/**
 * \brief Update IP and Modify checksum whenever IP is assigned or changed
 * \param timeSyncHandle pointer to PTP Handle structure
 * \param IP IP address in unsigned 32 bit format.
 */
void TimeSync_addIP(TimeSync_ParamsHandle_t timeSyncHandle, uint32_t IP);

/** @} */


#ifdef __cplusplus
}
#endif

#endif /* ICSS_TIMESYNC_INIT_H_ */
