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

#ifndef CLOCKP_H
#define CLOCKP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <kernel/dpl/SystemP.h>

/**
 * \defgroup KERNEL_DPL_CLOCK APIs for Clock
 * \ingroup KERNEL_DPL
 *
 * For more details and example usage, see \ref KERNEL_DPL_CLOCK_PAGE
 *
 * @{
 */

/**
 * \brief Max size of clock object across no-RTOS and all OS's
 */
#if defined (OS_SAFERTOS)
#define ClockP_OBJECT_SIZE_MAX    (120u)
#else
#define ClockP_OBJECT_SIZE_MAX    (104u)
#endif
/**
 * \brief Opaque clock object used with the clock APIs
 */
typedef struct ClockP_Object_ {

    /* uintptr_t translates to uint64_t for A53 and uint32_t for R5 and M4 */
    /* This accounts for the 64bit pointer in A53 and 32bit pointer in R5 and M4 */
    uintptr_t rsv[ClockP_OBJECT_SIZE_MAX/sizeof(uint32_t)]; /**< reserved, should NOT be modified by end users */

} ClockP_Object;


/**
 * \brief ClockP module config, set as part of SysConfig, not to be set by end-users directly
 */
typedef struct ClockP_Config_
{
    uint32_t timerBaseAddr; /**< HW Timer MMR base address */
    uint32_t timerHwiIntNum; /**< CPU interrupt number for this timer */
    uint32_t timerInputClkHz; /**< Timer clock in units of Hz */
    uint32_t timerInputPreScaler; /**< Timer divider to apply to the input clock */
    uint32_t usecPerTick; /**< period of one timer tick in units of usecs */

} ClockP_Config;

/**
 * \brief Callback that is called when the clock expires
 *
 * \param obj [in] Clock object associated with this callback
 * \param args [in] user specific argument pointer that was passed via \ref ClockP_Params
 */
typedef void (*ClockP_FxnCallback)(ClockP_Object *obj, void *args);

/**
 * \brief Parameters passed during \ref ClockP_construct
 */
typedef struct ClockP_Params_ {

    uint32_t start; /**< 0: do not start the clock after construct, \n 1: start the clock after construct */
    uint32_t timeout; /**< clock period for first execution, in units of clock ticks */
    uint32_t period; /**< clock period for subsequent periodic execution, in units of clock ticks. \n Set to 0 for one-shot mode of operation */
    ClockP_FxnCallback callback; /**< User callback to invoke when timer expires.
        \note Callback could be called in ISR context, so user should not block within the ISR
        */
    void *args; /**< User argument that is available inside the callback */

    char *name; /**< Name to associate with this object */

} ClockP_Params;



/**
 * \brief Initialize the clock module
 *
 * The API is called during system init to setup a timer to run at a periodic time internval
 * of 'n' micro seconds.
 *
 * 'n' can be configued by the user via SysConfig, default value for 'n' is typically 1000 us
 *
 * Using this single timer, the clock API can be used to start multiple 'clock's in units of
 * clock ticks.
 */
void ClockP_init();

/**
 * \brief Set default values to ClockP_Params
 *
 * Strongly recommended to be called before seting values in ClockP_Params
 *
 * \param params [out] parameter structure to set to default
 */
void ClockP_Params_init(ClockP_Params *params);

/**
 * \brief Create a clock object
 *
 * when ClockP_Params.start = 1, this also starts the clock object
 *
 * \param obj [out] created object
 * \param params [in] parameter structure
 *
 * \return \ref SystemP_SUCCESS on success, \ref SystemP_FAILURE on error
 */
int32_t ClockP_construct(ClockP_Object *obj, ClockP_Params *params);

/**
 * \brief Cleanup, delete, destruct a clock object
 *
 * \param obj [in] object
 */
void ClockP_destruct(ClockP_Object *obj);

/**
 * \brief Start the clock, if not already started.
 *
 * If clock is already started, then this restarts it with updated timeout and period, if any.
 *
 * \param obj [in] object
 */
void ClockP_start(ClockP_Object *obj);

/**
 * \brief Stop the clock, if not already stopped. No effect if clock is already stopped.
 *
 * \param obj [in] object
 */
void ClockP_stop(ClockP_Object *obj);

/**
 * \brief Check if clock is active i.e not expired
 *
 * For clock setup in periodic mode, clock will always be active after it is started and before it is stopped.
 *
 * For clock setup in one-shot mode , clock will be active after it is started and will be inactive after clock expires or it is stopped.
 *
 * \param obj [in] object
 *
 * \return 0: clock is not-active or expired, \n 1: clock is active or not expired
 */
uint32_t ClockP_isActive(ClockP_Object *obj);

/**
 * \brief Set clock timeout value, takes effect for next clock start
 *
 * \param obj [in] object
 * \param timeout [in] clock expiry period of first clock execution, in units of clock ticks
 */
void ClockP_setTimeout(ClockP_Object *obj, uint32_t timeout);

/**
 * \brief Get current remaining time in units of ticks
 *
 * \param obj [in] object
 *
 * \return clock expiry period of next clock execution, in units of clock ticks
 */
uint32_t ClockP_getTimeout(ClockP_Object *obj);

/**
 * \brief Get current clock ticks
 *
 * \return number of clock ticks that have elasped since ClockP_init()
 */
uint32_t ClockP_getTicks();

/**
 * \brief Convert usecs to clock ticks
 *
 * \param usecs [in] time in micro seconds
 *
 * \return nearest integer clock ticks
 */
uint32_t ClockP_usecToTicks(uint64_t usecs);

/**
 * \brief Convert clock ticks to usecs
 *
 * \param ticks [in] number of clocks ticks
 *
 * \return nearest integer micro seconds
 */
uint64_t ClockP_ticksToUsec(uint32_t ticks);


/**
 * \brief Get current time in units of usecs
 */
uint64_t ClockP_getTimeUsec();

/**
 * \brief Sleep for user specified usecs
 *
 * \param usec [in] Time to sleep in units of usecs
 *
 * \note Actual sleep will be in the range of `usec - ClockP_ticksToUsec(1)`
 *       to `usec`. If you need to guarantee atleast minimum
 *       sleep of `usec`, you need to sleep for `usec + ClockP_ticksToUsec(1)`.
 */
void ClockP_usleep(uint32_t usec);

/**
 * \brief Sleep for user specified seconds
 *
 * \note Actual sleep will be in the range of `sec - ClockP_ticksToUsec(1)`
 *       to `sec`. If you need to guarantee atleast minimum
 *       sleep of `sec`, you need to sleep for `sec + ClockP_ticksToUsec(1)`.
 *
 * \param sec [in] Time to sleep in units of secs
 */
void ClockP_sleep(uint32_t sec);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* CLOCKP_H */
