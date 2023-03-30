/**
 * @file  csl_timer_mgr.h
 *
 * @brief
 *  Header file containing various enumerations, structure definitions and function
 *  declarations for the timer_mgr IP.
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2017-2018, Texas Instruments, Inc.
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
 *    Neither the name of Texas Instruments Incorpotimermgred nor the names of
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
 *
*/
#ifndef CSL_TIMER_MGR_H_
#define CSL_TIMER_MGR_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
//#include <ti/csl/cslr_timer_mgr.h>
#include <drivers/hw_include/cslr.h>
#include <drivers/hw_include/cslr_timer_mgr.h>
/** ===========================================================================
 *
 * @defgroup CSL_TIMER_MGR_API Timer_mgr
 *
 * @section Introduction
 *
 * @subsection Overview
 *  This is the CSL-FL API documentation for the timer_mgr module.
 *
 *  The following sequence of API calls must be followed to properly configure
 *  and initialize the timer_mgr.
 *
 *  1. Allocate and initialize a #CSL_TimermgrCfg structure. A pointer to this
 *     structure must be passed to every API function.
 *
 *  2. Set the maximum number of timers to be monitored by calling the
 *     #CSL_timermgrSetMaxTimers function.
 *
 *  3. Write initial counter setup values to the desired timers by calling the
 *     #CSL_timermgrSetTimerCnt function for each timer. All timers that will
 *     be used initially must have a setup value written to them before the
 *     timer_mgr is enabled.
 *
 *  4. If timer events are desired, call the #CSL_timermgrSetEventIdx function
 *     to configure the event index for each desired timer.
 *
 *  5. Enable the desired timers. There are two ways to do this:
 *     a) To enable all timers, call the #CSL_timermgrEnableAllTimers function.
 *        This will enable all timers from 0 .. #CSL_timermgrGetMaxTimers.
 *     b) Enable (or disable) individual timers by calling the
 *        #CSL_timermgrSetTimerEnable function for each timer.
 *
 *     If less than half of the available timers are to be enabled initially,
 *     it is faster to enable individual timers by calling
 *     #CSL_timermgrSetTimerEnable with bEnable=true. If more than half of the
 *     available timers are to be enabled initially, it is faster to call
 *     #CSL_timermgrEnableAllTimers and then disable specific timers by calling
 *     #CSL_timermgrSetTimerEnable with bEnable=false.
 *
 *  6. Enable the timer_mgr by calling the #CSL_timermgrSetEnable function
 *     with bEnable=true.
 *
 *  Once the timer_mgr has been configured and enabled, the following functions
 *  can be called to service the active timers:
 *      #CSL_timermgrIsTimerExpired
 *      #CSL_timermgrGetExpiredTimers
 *      #CSL_timermgrTouchTimer
 *
 * @subsection References
 *    - timermgr Functional Specification, version 1.0.3
 *
 * ============================================================================
 */
/**
@defgroup CSL_TIMER_MGR_DATASTRUCT  TIMER_MGR Data Structures
@ingroup CSL_TIMER_MGR_API
*/
/**
@defgroup CSL_TIMER_MGR_FUNCTION  TIMER_MGR Functions
@ingroup CSL_TIMER_MGR_API
*/
/**
@defgroup CSL_TIMER_MGR_ENUM TIMER_MGR Enumerated Data Types
@ingroup CSL_TIMER_MGR_API
*/

/** ===========================================================================
 *  @addtogroup CSL_TIMER_MGR_ENUM
    @{
 * ============================================================================
 */

/** @} */

/** ============================================================================
 *  @addtogroup CSL_TIMER_MGR_DATASTRUCT
    @{
 * =============================================================================
 */

/** \brief Timer Manager configuration
 *
 *  This structure contains configuration information for the Timer Manager
 *
 */
typedef struct
{
    CSL_timer_mgrRegs        *pCfgRegs;     /** Pointer to timer_mgr cfg registers */
    CSL_timer_mgr_timersRegs *pTimerRegs;   /** Pointer to timer_mgr timer cfg registers */
    CSL_timer_mgr_oesRegs    *pOesRegs;     /** Pointer to timer_mgr oes registers */
} CSL_TimermgrCfg;

/** @} */

/** ===========================================================================
 *  @addtogroup CSL_TIMER_MGR_FUNCTION
    @{
 * ============================================================================
 */

/**
 *  \brief Return revision of the timer_mgr module
 *
 *  This function returns the contents of the timer_mgr revision register.
 *  Consult the timer_mgr module documentation for a description of the
 *  contents of the revision register.
 *
 *  \param pCfg             [IN]    Pointer to the #CSL_TimermgrCfg structure
 *
 *  \return The 32-bit revision register is returned.
 */
extern uint32_t CSL_timermgrGetRevision( const CSL_TimermgrCfg *pCfg );

/**
 *  \brief Return maximum number of timers available or being monitored
 *
 *  This function returns the maximum number of timers available or being
 *  monitored.
 *
 *  If this function is called before the #CSL_timermgrSetMaxTimers
 *  function is called, the default maximum number of timers available is
 *  returned. Otherwise, the maxTimers value specified in the
 *  #CSL_timermgrSetMaxTimers function call is returned.
 *
 *  \param pCfg             [IN]    Pointer to the #CSL_TimermgrCfg structure
 *
 *  \return Maximum number of timers available or being monitored
 */
extern uint32_t CSL_timermgrGetMaxTimers( const CSL_TimermgrCfg *pCfg );

/**
 *  \brief Sets the maximum number of timers to be monitored
 *
 *  This function sets the maximum number of timers to be monitored.
 *  All timer numbers >= to this value will be ignored by the timer_mgr.
 *
 *  This function will only operate when the timer_mgr is disabled.
 *  It is intended to be called during initial timer_mgr configuration.
 *
 *  \param pCfg             [IN]    Pointer to the #CSL_TimermgrCfg structure
 *  \param maxTimers        [IN]    Max number of timers to monitor
 *
 *  \return  0 = Success
 *          -1 = Error: maxTimer is greater than that supported by the
 *               timer_mgr, or the timer_mgr is currently enabled
 */
extern int32_t CSL_timermgrSetMaxTimers( CSL_TimermgrCfg *pCfg, uint32_t maxTimers );

/**
 *  \brief Sets the counter value for a specific timer
 *
 *  This function sets the counter value for the specified timer.
 *
 *  This function is intended to be called during initial configuration of
 *  the timer_mgr (before it is enabled). It can be called during timer_mgr
 *  operation to change an active timer's counter value, but more commonly
 *  the #CSL_timermgrTouchTimer function should be called to reload the
 *  timer's counter value.
 *
 *  \param pCfg             [IN]    Pointer to the #CSL_TimermgrCfg structure
 *  \param timerNum         [IN]    Timer number (0..#CSL_timermgrGetMaxTimers)
 *  \param timerCnt         [IN]    Counter value
 *
 *  \return  0 = Success
 *          -1 = Error: timerNum is greater than that configured in
 *               the timer_mgr
 */
extern int32_t CSL_timermgrSetTimerCnt( CSL_TimermgrCfg *pCfg, uint32_t timerNum, uint32_t timerCnt );

/**
 *  \brief Enable or disable a specific timer
 *
 *  This function enables or disables the specified timer.
 *
 *  \param pCfg             [IN]    Pointer to the #CSL_TimermgrCfg structure
 *  \param timerNum         [IN]    Timer number (0..#CSL_timermgrGetMaxTimers)
 *  \param bEnable          [IN]    true = enable timer, false = disable it
 *
 *  \return  0 = Success
 *          -1 = Error: timerNum is greater than that configured in
 *               the timer_mgr
 */
extern int32_t CSL_timermgrSetTimerEnable( CSL_TimermgrCfg *pCfg, uint32_t timerNum, bool bEnable );

/**
 *  \brief Enables all timers
 *
 *  This function enables all timers (0..#CSL_timermgrGetMaxTimers) that are
 *  currently configured in the timer_mgr.
 *
 *  This function will only operate when the timer_mgr is disabled.
 *  It is intended to be called during initial timer_mgr configuration.
 *
 *  \param pCfg             [IN]    Pointer to the #CSL_TimermgrCfg structure
 *
 *  \return  0 = Success
 *          -1 = Error: timerNum is greater than that configured in
 *               the timer_mgr
 */
extern int32_t CSL_timermgrEnableAllTimers( CSL_TimermgrCfg *pCfg );

/**
 *  \brief Enable or disable the timer_mgr
 *
 *  This function enables or disables the timer_mgr.
 *
 *  Before enabling the timer_mgr, you must set the maximum number of timers
 *  to be used (see #CSL_timermgrSetMaxTimers), set the counter values for the
 *  desired timers (see #CSL_timermgrSetTimerCnt), and enable desired timers
 *  (see #CSL_timermgrSetTimerEnable and #CSL_timermgrEnableAllTimers).
 *
 *  \param pCfg             [IN]    Pointer to the #CSL_TimermgrCfg structure
 *  \param bEnable          [IN]    true = enable timer_mgr, false = disable it
 *
 *  \return  None
 */
extern void CSL_timermgrSetEnable( CSL_TimermgrCfg *pCfg, bool bEnable );

/**
 *  \brief Return expiration status for a timer
 *
 *  This function returns the expiration status for the specified timer.
 *
 *  \param pCfg             [IN]    Pointer to the #CSL_TimermgrCfg structure
 *  \param timerNum         [IN]    Timer number (0..#CSL_timermgrGetMaxTimers)
 *
 *  \return  0 = Success (timer is not expired)
 *           1 = Success (timer is expired)
 *          -1 = Error: timerNum is greater than that configured in
 *               the timer_mgr
 */
extern int32_t CSL_timermgrIsTimerExpired( CSL_TimermgrCfg *pCfg, uint32_t timerNum );

/**
 *  \brief Return a list of expired timers
 *
 *  This function returns an array of timer numbers corresponding to the timers
 *  that have expired.
 *
 *  Note that the array must be large enough to contain the maximum number of
 *  timers configured via the #CSL_timermgrSetMaxTimers function.
 *
 *  \param pCfg             [IN]    Pointer to the #CSL_TimermgrCfg structure
 *  \param pExpiredTimerNums [OUT]   Pointer to an array where expired timer
 *                                  numbers are returned
 *
 *  \return  The number of expired timers is returned. This value is used to
 *           determine the valid values in the array.
 */
extern uint32_t CSL_timermgrGetExpiredTimers( CSL_TimermgrCfg *pCfg, uint32_t *pExpiredTimerNums );

/**
 *  \brief Touch a timer
 *
 *  This function is used to touch (reload the count value) of the specified
 *  timer.
 *
 *  \param pCfg             [IN]    Pointer to the #CSL_TimermgrCfg structure
 *  \param timerNum         [IN]    Timer number (0..#CSL_timermgrGetMaxTimers)
 *
 *  \return  0 = Success
 *          -1 = Error: timerNum is greater than that configured in
 *               the timer_mgr
 */
extern int32_t CSL_timermgrTouchTimer( CSL_TimermgrCfg *pCfg, uint32_t timerNum );

/**
 *  \brief Return the current timer_mgr counter value
 *
 *  This function returns the current timer_mgr counter value.
 *
 *  \param pCfg             [IN]    Pointer to the #CSL_TimermgrCfg structure
 *
 *  \return The current timer_mgr counter value is returned.
 */
extern uint32_t CSL_timermgrGetCurrCnt( const CSL_TimermgrCfg *pCfg );

/**
 *  \brief Set the event index for a timer
 *
 *  This function is used to set the event index for the specified
 *  timer.
 *
 *  \param pCfg             [IN]    Pointer to the #CSL_TimermgrCfg structure
 *  \param timerNum         [IN]    Timer number (0..#CSL_timermgrGetMaxTimers)
 *  \param eventIdx         [IN]    Event index
 *
 *  \return  0 = Success
 *          -1 = Error: timerNum is greater than that configured in
 *               the timer_mgr
 */
extern int32_t CSL_timermgrSetEventIdx( CSL_TimermgrCfg *pCfg, uint32_t timerNum, uint32_t eventIdx );

/**
 *  \brief Get the event index for a timer
 *
 *  This function is used to get the event index for the specified
 *  timer.
 *
 *  \param pCfg             [IN]    Pointer to the #CSL_TimermgrCfg structure
 *  \param timerNum         [IN]    Timer number (0..#CSL_timermgrGetMaxTimers)
 *
 *  \return  -1 = Error: timerNum is greater than that configured in
 *                the timer_mgr
 *           Otherwise, the event index for the specified timer is returned
 */
extern int32_t CSL_timermgrGetEventIdx( CSL_TimermgrCfg *pCfg, uint32_t timerNum );

/**
 *  \brief Enable or disable the auto reset function for a timer
 *
 *  This function enables or disables the auto reset function for
 *  the specified timer.
 *
 *  \param pCfg             [IN]    Pointer to the #CSL_TimermgrCfg structure
 *  \param timerNum         [IN]    Timer number (0..#CSL_timermgrGetMaxTimers)
 *  \param bEnable          [IN]    true = enable auto reset, false = disable it
 *
 *  \return  0 = Success
 *          -1 = Error: timerNum is greater than that configured in
 *               the timer_mgr
 *          -2 = This feature is not available in the timermgr being used
 */
extern int32_t CSL_timermgrEnableAutoReset( CSL_TimermgrCfg *pCfg, uint32_t timerNum, bool bEnable );

/** @} */

#ifdef __cplusplus
}
#endif

#endif
