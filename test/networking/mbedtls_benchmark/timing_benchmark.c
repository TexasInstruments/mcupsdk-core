/*
 * Copyright (C) 2023 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* ========================================================================== */
/*                                  INCLUDES                                  */
/* ========================================================================== */

#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/TimerP.h>
#include <kernel/dpl/HwiP.h>
#include "timing_alt.h"
#include <kernel/nortos/dpl/common/ClockP_nortos_priv.h>
#include <ti_dpl_config.h>

/* ========================================================================== */
/*                               GLOBAL VARIABLES                             */
/* ========================================================================== */


volatile int mbedtls_timing_alarmed = 0;
extern uint32_t gTimerBaseAddr[TIMER_NUM_INSTANCES];

void mbedtls_timer_alarm(void *param);

/**
 * \brief          Return the CPU cycle counter value
 *
 * \warning        This is only a best effort! Do not rely on this!
 *                 In particular, it is known to be unreliable on virtual
 *                 machines.
 *
 * \note           This value starts at an unspecified origin and
 *                 may wrap around.
 */
unsigned long mbedtls_timing_hardclock()
{
    return (unsigned long)ClockP_getTimeUsec()*1000;
}

/**
 * \brief          Return the elapsed time in milliseconds
 *
 * \param val      points to a timer structure
 * \param reset    If 0, query the elapsed time. Otherwise (re)start the timer.
 *
 * \return         Elapsed time since the previous reset in ms. When
 *                 restarting, this is always 0.
 *
 * \note           To initialize a timer, call this function with reset=1.
 *
 *                 Determining the elapsed time and resetting the timer is not
 *                 atomic on all platforms, so after the sequence
 *                 `{ get_timer(1); ...; time1 = get_timer(1); ...; time2 =
 *                 get_timer(0) }` the value time1+time2 is only approximately
 *                 the delay since the first reset.
 */
unsigned long mbedtls_timing_get_timer( struct mbedtls_timing_hr_time *val, int reset )
{
    if( reset )
    {
        mbedtls_timing_alarmed = 0;
        TimerP_stop(CONFIG_TIMER0_BASE_ADDR);
        TimerP_Params timerParams;

        TimerP_Params_init(&timerParams);
        timerParams.inputPreScaler    = CONFIG_TIMER0_INPUT_PRE_SCALER;
        timerParams.inputClkHz        = CONFIG_TIMER0_INPUT_CLK_HZ;
        timerParams.periodInUsec      = CONFIG_TIMER0_NSEC_PER_TICK;
        timerParams.oneshotMode       = 1;
        timerParams.enableOverflowInt = 1;
        timerParams.enableDmaTrigger  = 0;

        TimerP_setup(gTimerBaseAddr[CONFIG_TIMER0], &timerParams);

        TimerP_start(gTimerBaseAddr[CONFIG_TIMER0]);

        return( 0 );
    }
    else
    {
        return (unsigned long)TimerP_getCount(gTimerBaseAddr[CONFIG_TIMER0]);
    }
}

/**
 * \brief          Setup an alarm clock
 *
 * \param seconds  delay before the "mbedtls_timing_alarmed" flag is set
 *                 (must be >=0)
 *
 * \warning        Only one alarm at a time  is supported. In a threaded
 *                 context, this means one for the whole process, not one per
 *                 thread.
 */
void mbedtls_set_alarm( int seconds )
{
    TimerP_stop(gTimerBaseAddr[CONFIG_TIMER0]);
    mbedtls_timing_alarmed = 0;

    TimerP_Params timerParams;

    TimerP_Params_init(&timerParams);
        timerParams.inputPreScaler    = CONFIG_TIMER0_INPUT_PRE_SCALER;
        timerParams.inputClkHz        = CONFIG_TIMER0_INPUT_CLK_HZ;
        timerParams.periodInUsec      = CONFIG_TIMER0_NSEC_PER_TICK;
        timerParams.oneshotMode       = 1;
        timerParams.enableOverflowInt = 1;
        timerParams.enableDmaTrigger  = 0;

    TimerP_setup(gTimerBaseAddr[CONFIG_TIMER0], &timerParams);

    TimerP_start(gTimerBaseAddr[CONFIG_TIMER0]);
}

/**
 * \brief          Set a pair of delays to watch
 *                 (See \c mbedtls_timing_get_delay().)
 *
 * \param data     Pointer to timing data.
 *                 Must point to a valid \c mbedtls_timing_delay_context struct.
 * \param int_ms   First (intermediate) delay in milliseconds.
 *                 The effect if int_ms > fin_ms is unspecified.
 * \param fin_ms   Second (final) delay in milliseconds.
 *                 Pass 0 to cancel the current delay.
 *
 * \note           To set a single delay, either use \c mbedtls_timing_set_timer
 *                 directly or use this function with int_ms == fin_ms.
 */
void mbedtls_timing_set_delay( void *data, uint32_t int_ms, uint32_t fin_ms )
{
    mbedtls_timing_delay_context *ctx = (mbedtls_timing_delay_context *) data;
    ctx->int_ms = int_ms;
    ctx->fin_ms = fin_ms;

    if( fin_ms != 0 )
        (void) mbedtls_timing_get_timer( &ctx->timer, 1 );
}

/**
 * \brief          Get the status of delays
 *                 (Memory helper: number of delays passed.)
 *
 * \param data     Pointer to timing data
 *                 Must point to a valid \c mbedtls_timing_delay_context struct.
 *
 * \return         -1 if cancelled (fin_ms = 0),
 *                  0 if none of the delays are passed,
 *                  1 if only the intermediate delay is passed,
 *                  2 if the final delay is passed.
 */
int mbedtls_timing_get_delay( void *data )
{
    mbedtls_timing_delay_context *ctx = (mbedtls_timing_delay_context *) data;
    unsigned long elapsed_ms;

    if( ctx->fin_ms == 0 )
        return( -1 );

    elapsed_ms = mbedtls_timing_get_timer( &ctx->timer, 0 );

    if( elapsed_ms >= ctx->fin_ms )
        return( 2 );

    if( elapsed_ms >= ctx->int_ms )
        return( 1 );

    return( 0 );
}

void mbedtls_timer_alarm(void *param)
{
    mbedtls_timing_alarmed = 1;
    TimerP_stop(gTimerBaseAddr[CONFIG_TIMER0]);
}
