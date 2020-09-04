/*
 *  Copyright (c) Texas Instruments Incorporated 2022
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

/*!
 * \file timeSync_ptp_init_priv.h
 *
 * \brief This file contains the interface to initialization functions
 *        of PTP stack
 */

#ifndef TIMESYNC_PTP_INIT_PRIV_H_
#define TIMESYNC_PTP_INIT_PRIV_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/**
 * @def TIMESYNC_PTP_LEAP_61_INDEX
 *      Index for Leap 61 status in ptpFlags[] array
 */
#define  TIMESYNC_PTP_LEAP_61_INDEX                              (0U)

/**
 * @def TIMESYNC_PTP_LEAP_59_INDEX
 *      Index for Leap 59 status
 */
#define  TIMESYNC_PTP_LEAP_59_INDEX                              (1U)

/**
 * @def TIMESYNC_PTP_UTC_REASONABLE_INDEX
 *      Index for UTC reasonable value in ptpFlags[] array
 */
#define  TIMESYNC_PTP_UTC_REASONABLE_INDEX                       (2U)

/**
 * @def TIMESYNC_PTP_TIMESCALE_INDEX
 *      Index for value indicating whether PTP Timescale
 *      in ptpFlags[] array
 */
#define  TIMESYNC_PTP_TIMESCALE_INDEX                            (3U)

/**
 * @def TIMESYNC_PTP_TIME_TRACEABLE_INDEX
 *      Index for value indicating whether
 *      time traceable in ptpFlags[] array
 */
#define  TIMESYNC_PTP_TIME_TRACEABLE_INDEX                       (4U)

/**
 * @def TIMESYNC_PTP_FREQ_TRACEABLE_INDEX
 *      Index for value indicating whether
 *      freq traceable in ptpFlags[] array
 */
#define  TIMESYNC_PTP_FREQ_TRACEABLE_INDEX                       (5U)

/**
 * @def TIMESYNC_PTP_ALTERNATE_MASTER_INDEX
 *      Index for value indicating whether
 *      alternate master in ptpFlags[] array
 */
#define  TIMESYNC_PTP_ALTERNATE_MASTER_INDEX                     (6U)

/**
 * @def TIMESYNC_PTP_TWO_STEP_INDEX
 *      Index for value indicating whether
 *      two step sync in ptpFlags[] array
 */
#define  TIMESYNC_PTP_TWO_STEP_INDEX                             (7U)

/**
 * @def TIMESYNC_PTP_UNICAST
 *      Index for value indicating whether
 *      messaging is unicast in ptpFlags[] array
 */
#define  TIMESYNC_PTP_UNICAST                                    (8U)

/**
 * @def TIMESYNC_PTP_PROFILE_SPECIFIC_1_INDEX
 *      Index for value indicating alternate
 *      PTP Profile in ptpFlags[] array
 *
 */
#define  TIMESYNC_PTP_PROFILE_SPECIFIC_1_INDEX                   (9U)

/**
 * @def TIMESYNC_PTP_PROFILE_SPECIFIC_2_INDEX
 *      Index for value indicating alternate
 *      PTP Profile in ptpFlags[] array
 *
 */
#define  TIMESYNC_PTP_PROFILE_SPECIFIC_2_INDEX                   (10U)

/**
 * @def TIMESYNC_PTP_SECURITY_INDEX
 *  Index for value indicating whether PTP security
 *  is enabled or not in ptpFlags[] array
 */
#define  TIMESYNC_PTP_SECURITY_INDEX                             (11U)

/*------------Flag masks for different frames as per standard--------*/
/* These are derived indirectly from Table 20 of the standard*/

/**
 * @def TIMESYNC_PTP_SYNC_BYTE0_MASK
 *  selects the flags relevant to Sync frame for
 *  first byte of the 2 byte flag
 */
#define  TIMESYNC_PTP_SYNC_BYTE0_MASK                            (0xFFU)

/**
 * @def TIMESYNC_PTP_ANNOUNCE_BYTE0_MASK
 *  selects the flags relevant to Announce frame for
 *  first byte of the 2 byte flag
 */
#define  TIMESYNC_PTP_ANNOUNCE_BYTE0_MASK                        (0xFDU)

/**
 * @def TIMESYNC_PTP_FOLLOW_UP_BYTE0_MASK
 *  selects the flags relevant to Follow Up frame for
 *  first byte of the 2 byte flag
 */
#define  TIMESYNC_PTP_FOLLOW_UP_BYTE0_MASK                       (0xFDU)

/**
 * @def TIMESYNC_PTP_PDELAY_RESP_BYTE0_MASK
 *  selects the flags relevant to Pdelay Resp frame for
 *  first byte of the 2 byte flag
 */
#define  TIMESYNC_PTP_PDELAY_RESP_BYTE0_MASK                     (0xFEU)

/**
 * @def TIMESYNC_PTP_DELAY_RESP_BYTE0_MASK
 *  selects the flags relevant to Delay Resp frame for
 *  first byte of the 2 byte flag
 */
#define  TIMESYNC_PTP_DELAY_RESP_BYTE0_MASK                      (0xFDU)

/**
 * @def TIMESYNC_OTHER_FRAMES_BYTE0_MASK
 *  selects the flags relevant to Pdelay Req
 *  and Delay Request for
 *  first byte of the 2 byte flag
 */
#define  TIMESYNC_OTHER_FRAMES_BYTE0_MASK                        (0xFCU)

/**
 * @def TIMESYNC_PTP_PDELAY_BUF_SIZE
 *  Size of peer delay buffers on DUT
 *  This includes peer delay request, response and respone follow up
 */
#define  TIMESYNC_PTP_PDELAY_BUF_SIZE                            (68U)

/**
 * @def TIMESYNC_PTP_SYNC_BUF_ANNEX_F_SIZE
 *  Size of Sync buffers on DUT for Annex F mode
 */
#define  TIMESYNC_PTP_SYNC_BUF_ANNEX_F_SIZE                      (58U)

/**
 * @def TIMESYNC_PTP_FOLLOW_UP_BUF_ANNEX_F_SIZE
 *  Size of Follow Up buffers on DUT for Annex F mode
 */
#define  TIMESYNC_PTP_FOLLOW_UP_BUF_ANNEX_F_SIZE                 (60U)

/**
 * @def TIMESYNC_PTP_ANNOUNCE_BUF_ANNEX_F_SIZE
 *  Size of Announce Up buffers on DUT for Annex F mode
 */
#define  TIMESYNC_PTP_ANNOUNCE_BUF_ANNEX_F_SIZE                  (78U)

/**
 * @def TIMESYNC_PTP_SYNC_BUF_ANNEX_E_SIZE
 *  Size of Sync buffers on DUT for Annex E mode
 */
#define  TIMESYNC_PTP_SYNC_BUF_ANNEX_E_SIZE                      (86U)

/**
 * @def TIMESYNC_PTP_FOLLOW_UP_BUF_ANNEX_E_SIZE
 *  Size of Follow Up buffers on DUT for Annex E mode
 */
#define  TIMESYNC_PTP_FOLLOW_UP_BUF_ANNEX_E_SIZE                 (86U)

/**
 * @def TIMESYNC_PTP_ANNOUNCE_BUF_ANNEX_E_SIZE
 *  Size of Announce Up buffers on DUT for Annex E mode.
 */
#define  TIMESYNC_PTP_ANNOUNCE_BUF_ANNEX_E_SIZE                  (106U)

/**
 * @def TIMESYNC_PTP_DELAY_REQ_BUF_SIZE
 *  Size of Delay Request buffer on DUT (only applicable to E2E).
 */
#define  TIMESYNC_PTP_DELAY_REQ_BUF_SIZE                         (86U)

/*-----------------Default PTP Master init params-------------------*/

/* These are used in BMCA*/

/**
 * @def Default Priority 1 value communicated in Announce frame
 */
#define  TIMESYNC_PTP_DEFAULT_PRIO_1                             (128U)

/**
 * @def Default Priority 1 value communicated in Announce frame
 */
#define  TIMESYNC_PTP_DEFAULT_PRIO_2                             (128U)

/**
 * @def Default clock accuracy communicated in Announce frame.
 */
#define  TIMESYNC_PTP_DEFAULT_CLOCK_ACCURACY                     (0x31U)

/**
 * @def Default clock class communicated in Announce frame
 */
#define  TIMESYNC_PTP_DEFAULT_CLOCK_CLASS                        (248U)

/**
 * @def Default clock variance from GM communicated in Announce frame
 */
#define  TIMESYNC_PTP_DEFAULT_CLOCK_VARIANCE                     (0U)

/**
 * @def Default steps removed for Boundary clock
 */
#define  TIMESYNC_PTP_DEFAULT_STEPS_REMOVED                      (0U)

/**
 * @def Default UTC offset
 */
#define  TIMESYNC_PTP_DEFAULT_UTC_OFFSET                         (0U)

/**
 * @def Default Time source used for clock. Set to Internal oscillator
 */
#define  TIMESYNC_PTP_DEFAULT_TIME_SOURCE                        (0xa0U)

/**
 * @def Default drift threshold used in offset stabilization algo
 */
#define  TIMESYNC_PTP_OFFSET_STABLE_ALGO_THRESHOLD               (15U)

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/** @addtogroup TIMESYNC_PTP_API
   @{ */

/**
 * @brief Allocate Rx and Tx buffers for frames like Sync, Announce etc
 * @param hTimeSyncPtp pointer to PTP Handle structure
 * @return None
 */
void TimeSyncPtp_allocPktBuffer(TimeSyncPtp_Handle hTimeSyncPtp);

/**
 * @brief Unallocate Rx and Tx buffers for frames like Sync, Announce etc
 * @param hTimeSyncPtp pointer to PTP Handle structure
 * @return None
 */
void TimeSyncPtp_unAllocPktBuffer(TimeSyncPtp_Handle hTimeSyncPtp);

/**
 * @brief Set Default values for the PTP Object based on clock type
 *
 * @param hTimeSyncPtp pointer to PTP Handle structure
 *
 * @return none
 */
void TimeSyncPtp_setDefaultValue(TimeSyncPtp_Handle hTimeSyncPtp);
/** @} */

#endif /* TIMESYNC_PTP_INIT_PRIV_H_ */
