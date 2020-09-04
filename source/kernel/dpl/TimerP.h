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

#ifndef TIMERP_H
#define TIMERP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <kernel/dpl/SystemP.h>

/**
 * \defgroup KERNEL_DPL_TIMER  APIs for timer setup and control
 * \ingroup KERNEL_DPL
 *
 * This module define's generic APIs to configure and control a timer
 * Depending on the SOC there can be different timer implementation's
 * 
 * For more details and example usage, see \ref KERNEL_DPL_TIMER_PAGE
 *
 * Timer is used by \ref KERNEL_DPL_CLOCK_PAGE to generate system ticks.
 *
 * Currently below timer implementations are supported
 * - DM Timer (AM64x, AM243x)
 * - RTI Timer (AM263x, AM273x, AWR294x)
 *
 * @{
 */

/**
 * \brief Parameters for \ref TimerP_setup
 */
typedef struct TimerP_Params_ {

    uint32_t inputPreScaler; /**< input pre-scaler divisor ro apply
                              *
                              * \note MUST be power of 2 and between 1 and 256 for GP Timer
                              * \note MAKE sure this value is not 0
                              * \note This field is valid only when underlying timer is DM Timer.
                              * \note This field is not valid when underlying timer is RTI Timer. Set to 1 in this case.
                              */
    uint32_t inputClkHz;  /**< Timer input clock in unit of Hz before pre-scaler, system initialization MUST
                            * make any system level muxes, PLLs, power required to input this clock are setup properly
                            *
                            * \note MAKE sure this value is not 0
                            */
    uint32_t periodInUsec; /**< Timer period in units of usecs, internally \ref TimerP_Params.inputClkHz
                            * and TimerP_Params.inputPreScaler is used to compute the value to be put inside the timer HW register
                            *
                            * \note When value is 0, \ref periodInNsec is used instead
                            * \note When both \ref periodInUsec and \ref periodInNsec are non-zero, \ref periodInNsec is used
                            */
    uint32_t periodInNsec; /**< Timer period in units of nsecs, internally \ref TimerP_Params.inputClkHz
                            * and TimerP_Params.inputPreScaler is used to compute the value to be put inside the timer HW register
                            *
                            * \note When value is 0, \ref periodInUsec is used instead
                            * \note When both \ref periodInUsec and \ref periodInNsec are non-zero, \ref periodInNsec is used
                            */
    uint32_t oneshotMode; /**< 0: continuous mode of operation, 1: oneshot mode of operation
                            *
                            * \note NOT supported for RTI timer, always set to 0 in this case.
                            */
    uint32_t enableOverflowInt; /**< 0: Do not enable timer overflow interrupt, 1: enable timer overflow interrupt */
    uint32_t enableDmaTrigger;  /**< 0: Do not enable DMA trigger from timer, 1: enable DMA trigger from timer */

} TimerP_Params;

/**
 * \brief Set default value for \ref TimerP_Params
 *
 * \param params [out] timer parameters initalized to default
 */
void TimerP_Params_init(TimerP_Params *params);

/**
 * \brief Setup timer but does not start it
 *
 * \param baseAddr [in] HW timer base addresss
 * \param params [in] timer parameters
 */
void TimerP_setup(uint32_t baseAddr, TimerP_Params *params);

/**
 * \brief Start timer
 *
 * \param baseAddr [in] HW timer base addresss
 */
void TimerP_start(uint32_t baseAddr);

/**
 * \brief Stop timer
 *
 * \param baseAddr [in] HW timer base addresss
 */
void TimerP_stop(uint32_t baseAddr);

/**
 * \brief Get timer current count
 *
 * \param baseAddr [in] HW timer base addresss
 *
 * \return current timer count value
 */
uint32_t TimerP_getCount(uint32_t baseAddr);

/**
 * \brief Get timer reload count
 *
 * \param baseAddr [in] HW timer base addresss
 *
 * \return reload count value
 */
uint32_t TimerP_getReloadCount(uint32_t baseAddr);

/**
 * \brief Clear timer overflow interrupt
 *
 * \param baseAddr [in] HW timer base addresss
 */
void TimerP_clearOverflowInt(uint32_t baseAddr);


/**
 * \brief Check if timer is overflowed
 *
 * \note make sure to clear overflow status \ref TimerP_clearOverflowInt
 *
 * \param baseAddr [in] HW timer base addresss
 */
uint32_t TimerP_isOverflowed(uint32_t baseAddr);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* TIMERP_H */

